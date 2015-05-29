/*
 * Copyright (C) 2015 The University of Texas at Austin and the
 * Institute of Human Machine Cognition. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 2.1 of
 * the License, or (at your option) any later version. See
 * <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

#include <controlit_robot_models/rbdl_robot_urdfreader.hpp>
#include <controlit/logging/RealTimeLogging.hpp>

#include <tinyxml.h>
#include <XmlRpcValue.h>
#include <XmlRpcException.h>

#include <assert.h>
#include <iostream>
#include <map>
#include <stack>
#include <string>
#include <fstream>
#include <streambuf>

#include <urdf/model.h>

using std::vector;
using std::map;
using std::stack;
using RigidBodyDynamics::Body;
using RigidBodyDynamics::Joint;
using RigidBodyDynamics::Math::Vector3d;
using RigidBodyDynamics::Math::Matrix3d;
using RigidBodyDynamics::Math::SpatialVector;
using RigidBodyDynamics::Math::SpatialTransform;
using RigidBodyDynamics::Math::Xrot;
using RigidBodyDynamics::Math::Xtrans;
using RigidBodyDynamics::JointTypeFixed;

namespace controlit {
namespace rbdl_robot_urdfreader {

// Uncomment one of the following lines to enable/disable detailed debug statements.
#define PRINT_DEBUG_STATEMENT(ss)
// #define PRINT_DEBUG_STATEMENT(ss) CONTROLIT_DEBUG_RT << ss;

#define PRINT_DEBUG_STATEMENT_ALWAYS(ss) CONTROLIT_DEBUG_RT << ss;
#define PRINT_ERROR_STATEMENT(ss) CONTROLIT_ERROR_RT << ss;

// Private functions

/*!
 * Construct an RBDL model from a URDF model
 *
 * \param[in] urdf_model URDF model we create the RBDL model from
 * \param[out] robot A pointer to where the resulting model should be saved.
 * \param[out] linkToJointMap Map that stores relationship between link names and joint names.
 * \param[out] limits pointer to where joint limits should be saved
 * \param verbose Whether to print status messages.
 */
bool construct_model(urdf::Model const& urdf_model,
                     RigidBodyDynamics::Model *rbdl_robot,
                     std::map<std::string, std::string> * linkToJointMap,
                     WBCJointLimits *limits,
                     bool verbose);

/** \brief Changes the specified joints in the URDF Model to fixed.
 *  \author R. W. Sinnet (ryan@rwsinnet.com)
 *  \param[in] urdf_model The urdf::Model object to be modified.
 *  \param[in] fixed_joint_list A vector of joints to be set to fixed in the
 *  URDF Model.
 */
void fix_joints(urdf::Model &urdf_model,
                const std::vector<std::string> &fixed_joint_list);

/*!
 * Augment URDF XML by adding a floating body
 *
 * \param[in] root_link_name Name of the root body
 * \param[out] robot_xml Augmented XML URDF description
 */
bool augment_xml(const std::string& root_link_name,
                 TiXmlDocument& robot_xml);

/*!
 * Adds a floating body description to a URDF model
 *
 * \param[in] urdf_description Unaugmented URDF description
 * \param[in] root_link_name Name of the root body
 * \param[out] urdf_model Augmented URDF model object
 */
bool add_floating_joint(std::string const& urdf_description,
                        std::string const& root_link_name,
                        urdf::Model& urdf_model);

/*!
 * DEPRECATED.. Need to stop duplicating code everywhere
 */
// bool add_floating_joint(char const* filename,
//   const std::string& root_link_name,
//   urdf::Model& urdf_model);


// Public API
bool read_urdf_model_from_file(std::string const& filename,
                               RigidBodyDynamics::Model *robot,
                               map<std::string, std::string> * linkToJointMap,
                               bool verbose)
{
    assert(robot);

    std::ifstream file(filename);
    if (file.good())
    {
        std::string fileContents((std::istreambuf_iterator<char>(file)),
                                 std::istreambuf_iterator<char>());

        return read_urdf_model_from_string(fileContents, robot, linkToJointMap, nullptr, verbose);
    }
    else
    {
        PRINT_ERROR_STATEMENT("Failed to open '" << filename << "' for reading.")
        return false;
    }
}

bool read_urdf_model_from_string(const std::string &urdf_description,
                                 const std::vector<std::string> &fixed_joint_list,
                                 RigidBodyDynamics::Model *robot,
                                 map<std::string, std::string> *linkToJointMap,
                                 bool verbose)
{
    return load_urdf_model(urdf_description, fixed_joint_list,
                           robot, linkToJointMap, nullptr, verbose);
}

bool read_urdf_model_from_string(const std::string &urdf_description,
                                 RigidBodyDynamics::Model *robot,
                                 map<std::string, std::string> * linkToJointMap,
                                 bool verbose)
{

    std::vector<std::string> fixed_joint_list;
    return load_urdf_model(urdf_description, fixed_joint_list,
                           robot, linkToJointMap, nullptr, verbose);
}

bool read_urdf_model_from_string(const std::string &urdf_description,
                                 RigidBodyDynamics::Model *robot,
                                 map<std::string, std::string> * linkToJointMap,
                                 WBCJointLimits *limits,
                                 bool verbose)
{

    std::vector<std::string> fixed_joint_list;
    return load_urdf_model(urdf_description, fixed_joint_list,
                           robot, linkToJointMap, limits, verbose);
}

bool load_urdf_model(const std::string &urdf_description,
                     const std::vector<std::string> &fixed_joint_list,
                     RigidBodyDynamics::Model *robot,
                     map<std::string, std::string> * linkToJointMap,
                     WBCJointLimits *limits,
                     bool verbose)
{

    assert (robot);

    if (urdf_description.empty())
    {
        PRINT_ERROR_STATEMENT("Failed to read URDF from string, string was empty")
        return false;
    }

    urdf::Model urdf_model;
    if (not urdf_model.initString(urdf_description))
    {
        PRINT_ERROR_STATEMENT("URDF failed to parse URDF description")
        return false;
    }

    if ((urdf_model.getRoot()->name) != "world") //the model *should* be floating, but there is no floating joint in the urdf specification...this needs to be fixed for the parser to work
    {
        // Add the virtual 6 DOFs to the model
        if (!add_floating_joint(urdf_description, urdf_model.getRoot()->name, urdf_model))
        {
            PRINT_ERROR_STATEMENT("Failed to add floating joint to the model for rbdl parsing")
            return false;
        }
    }

    // Go through the URDF and replace the appropriate movable joints with fixed
    // joints.
    if (!fixed_joint_list.empty())
        fix_joints(urdf_model, fixed_joint_list);


    if (!construct_model(urdf_model, robot, linkToJointMap, limits, verbose))
    {
        PRINT_ERROR_STATEMENT("Error constructing model from urdf file.")
        return false;
    }

    robot->gravity.set(0, 0, -9.81);  // This is now set by controlit::controller_library::Controller
    return true;
}

// bool read_urdf_submodel(const char * filename,
//                         const std::vector<std::string> * links,
//                         RigidBodyDynamics::Model * robot,
//                         std::map<std::string, std::string> * linkToJointMap,
//                         bool verbose)
// {
//   return true;
// }

// bool read_urdf_submodel(const std::string urdf_description,
//                         const std::vector<std::string> * links,
//                         RigidBodyDynamics::Model * robot,
//                         std::map<std::string, std::string> * linkToJointMap,
//                         bool verbose)
// {
//   return true;
// }

// Private function implementation
void fix_joints(urdf::Model &urdf_model,
                const std::vector<std::string> &fixed_joint_list)
{
    // Fix the joints here, man.
    for (std::vector<std::string>::const_iterator it = fixed_joint_list.begin();
         it != fixed_joint_list.end(); ++it)
    {
        // Check if the joint exists and set it to fixed if it does.
        if (urdf_model.joints_.find(*it) != urdf_model.joints_.end())
            urdf_model.joints_.find(*it)->second->type =
                urdf::Joint::FIXED;
        else
            CONTROLIT_WARN_RT << "Could not set joint `" << *it << "' to fixed, joint not found in urdf model.";
    }
}

bool construct_model(urdf::Model const& urdf_model,
                     RigidBodyDynamics::Model *rbdl_robot,
                     std::map<std::string, std::string>  * linkToJointMap,
                     WBCJointLimits *limits,
                     bool verbose)
{
    typedef boost::shared_ptr<urdf::Link> LinkPtr;
    typedef boost::shared_ptr<urdf::Joint> JointPtr;

    // typedef vector<LinkPtr> URDFLinkVector;
    // typedef vector<JointPtr> URDFJointVector;
    typedef map<std::string, LinkPtr > URDFLinkMap;
    typedef map<std::string, JointPtr > URDFJointMap;

    PRINT_DEBUG_STATEMENT("root_link_ is named '" << urdf_model.getRoot()->name << "'")

    if (linkToJointMap != nullptr)
        linkToJointMap->clear();

    // Initialize stack with root node
    URDFLinkMap link_map = urdf_model.links_;
    stack<int> joint_index_stack;
    stack<LinkPtr > link_stack;
    link_stack.push (link_map[(urdf_model.getRoot()->name)]);
    if (link_stack.top()->child_joints.size() > 0)
    {
        joint_index_stack.push(0);
    }

    // With stack initialized, add bodies in a depth-first order of the model tree
    URDFJointMap joint_map = urdf_model.joints_;
    vector<std::string> joint_names;
    while (link_stack.size() > 0)
    {
        LinkPtr cur_link = link_stack.top();
        int joint_idx = joint_index_stack.top();

        //Find non-fixed parent joint and add to the linkToJontMap
        if(cur_link->parent_joint!=0) //base node has null pointer
        {
            bool parent_fixed = true; //Guilty until proven innocent
            std::string movable_parent_joint_name = cur_link->parent_joint->name;
            while(parent_fixed)
            {
                if( joint_map[movable_parent_joint_name]->type != urdf::Joint::FIXED )
                    parent_fixed = false;
                else
                {
                    if(link_map[joint_map[movable_parent_joint_name]->parent_link_name]->parent_joint != 0) //check for null pointer
                    {
                        movable_parent_joint_name = std::string(link_map[joint_map[movable_parent_joint_name]->parent_link_name]->parent_joint->name);
                    }
                    else
                        break;
                }
            }
            if (linkToJointMap != nullptr)
                linkToJointMap->insert(make_pair(std::string(cur_link->name),movable_parent_joint_name));
        }

        // Convert the number of child joints into a signed value
        int numChildJoints = (int)cur_link->child_joints.size();

        if (joint_idx < numChildJoints)
        {
            JointPtr cur_joint = cur_link->child_joints[joint_idx];

            // increment joint index
            joint_index_stack.pop();
            joint_index_stack.push(joint_idx + 1);

            link_stack.push (link_map[cur_joint->child_link_name]);
            joint_index_stack.push(0);

            if (verbose)
            {
                for (size_t i = 1; i < joint_index_stack.size() - 1; i++)
                {
                    std::cout << "  ";
                }
                PRINT_DEBUG_STATEMENT("Joint '" << cur_joint->name << "', child link '" << link_stack.top()->name << ", type = " << cur_joint->type)
            }

            joint_names.push_back(cur_joint->name);
        }
        else
        {
            link_stack.pop();
            joint_index_stack.pop();
        }
    }

    // Count the number of revolute and continuous joints.  This is assumed to be the number of
    // actual DOFs in the robot.  Initialize the limits object with this size.
    int num_joints = 0;
    if (limits != nullptr)
    {
        for (size_t jj = 0; jj < joint_names.size(); jj++)
        {
            JointPtr urdf_joint = joint_map[joint_names[jj]];
            if (urdf_joint->type == urdf::Joint::REVOLUTE || urdf_joint->type == urdf::Joint::CONTINUOUS)
            {
                num_joints++;
            }
        }

        limits->init(num_joints);
    }

    num_joints = 0;  // used as index into limits vector variables

    for (size_t jj = 0; jj < joint_names.size(); jj++)
    {
        JointPtr urdf_joint = joint_map[joint_names[jj]];
        LinkPtr urdf_parent = link_map[urdf_joint->parent_link_name];
        LinkPtr urdf_child = link_map[urdf_joint->child_link_name];

        // determine where to add the current joint and child body
        unsigned int rbdl_parent_id = 0;

        if (urdf_parent->name != "world" && rbdl_robot->mBodies.size() != 1)
            rbdl_parent_id = rbdl_robot->GetBodyId (urdf_parent->parent_joint->name.c_str()); //RBDLRobot Bodies have the name of their parent joint

        // cout << "joint: " << joint_names[j] << "\tparent = " << urdf_joint->parent_link_name << "\t child = " << urdf_joint->child_link_name << "\t parent_id = " << rbdl_parent_id << endl;

        // create the joint
        Joint rbdl_joint;
        if (urdf_joint->type == urdf::Joint::REVOLUTE || urdf_joint->type == urdf::Joint::CONTINUOUS)
        {
            rbdl_joint = Joint(SpatialVector (urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z, 0, 0, 0));


            // Save the joint limit information
            if (limits != nullptr)
            {
                limits->positionUpperLimits(num_joints) = urdf_joint->limits->upper;
                limits->positionLowerLimits(num_joints) = urdf_joint->limits->lower;
                limits->torqueLimits(num_joints) = urdf_joint->limits->effort;
                limits->velocityLimits(num_joints) = urdf_joint->limits->velocity;
                num_joints++;
            }

        }
        else if (urdf_joint->type == urdf::Joint::PRISMATIC)
        {
            rbdl_joint = Joint (SpatialVector (0, 0, 0, urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z));
        }
        else if (urdf_joint->type == urdf::Joint::FIXED)
        {
            rbdl_joint = Joint (JointTypeFixed);
        }
        else if (urdf_joint->type == urdf::Joint::FLOATING)
        {
            // todo: what order of DoF should be used?
            rbdl_joint = Joint (
                SpatialVector (0, 0, 0, 1, 0, 0),
                SpatialVector (0, 0, 0, 0, 1, 0),
                SpatialVector (0, 0, 0, 0, 0, 1),
                SpatialVector (0, 0, 1, 0, 0, 0),
                SpatialVector (0, 1, 0, 0, 0, 0),
                SpatialVector (1, 0, 0, 0, 0, 0));
        }
        else if (urdf_joint->type == urdf::Joint::PLANAR)
        {
            // todo: which two directions should be used that are perpendicular
            // to the specified axis?
            PRINT_ERROR_STATEMENT("Error while processing joint '" << urdf_joint->name << "': planar joints not yet supported!")
            return false;
        }

        // compute the joint transformation
        Vector3d joint_rpy;
        Vector3d joint_translation;
        urdf_joint->parent_to_joint_origin_transform.rotation.getRPY (joint_rpy[0], joint_rpy[1], joint_rpy[2]);
        if(urdf_joint->type == urdf::Joint::FLOATING)
        {
            joint_translation.set(0,0,0);
        }
        else
        {
            joint_translation.set (
                urdf_joint->parent_to_joint_origin_transform.position.x,
                urdf_joint->parent_to_joint_origin_transform.position.y,
                urdf_joint->parent_to_joint_origin_transform.position.z
            );
        }
        SpatialTransform rbdl_joint_frame =
            Xrot (joint_rpy[0], Vector3d (1., 0., 0.))
            * Xrot (joint_rpy[1], Vector3d (0., 1., 0.))
            * Xrot (joint_rpy[2], Vector3d (0., 0., 1.))
            * Xtrans (Vector3d (
                          joint_translation
                      ));

        // assemble the body
        Vector3d link_inertial_position;
        Vector3d link_inertial_rpy;
        Matrix3d link_inertial_inertia = Matrix3d::Zero();
        double link_inertial_mass;

        // but only if we actually have inertial data
        if (urdf_child->inertial)
        {
            link_inertial_mass = urdf_child->inertial->mass;

            link_inertial_position.set (
                urdf_child->inertial->origin.position.x,
                urdf_child->inertial->origin.position.y,
                urdf_child->inertial->origin.position.z
            );
            urdf_child->inertial->origin.rotation.getRPY (link_inertial_rpy[0], link_inertial_rpy[1], link_inertial_rpy[2]);

            link_inertial_inertia(0,0) = urdf_child->inertial->ixx;
            link_inertial_inertia(0,1) = urdf_child->inertial->ixy;
            link_inertial_inertia(0,2) = urdf_child->inertial->ixz;

            link_inertial_inertia(1,0) = urdf_child->inertial->ixy;
            link_inertial_inertia(1,1) = urdf_child->inertial->iyy;
            link_inertial_inertia(1,2) = urdf_child->inertial->iyz;

            link_inertial_inertia(2,0) = urdf_child->inertial->ixz;
            link_inertial_inertia(2,1) = urdf_child->inertial->iyz;
            link_inertial_inertia(2,2) = urdf_child->inertial->izz;

            if (link_inertial_rpy != Vector3d (0., 0., 0.))
            {
                PRINT_ERROR_STATEMENT("Error while processing body '" << urdf_child->name << ".  "
                    "Rotation of body frames not yet supported. Please rotate the joint frame instead.");
                return false;
            }
        }

        Body rbdl_body = Body (link_inertial_mass, link_inertial_position, link_inertial_inertia);

        if (verbose)
        {
            std::stringstream buff;

            buff << "+ Adding Body " << "\n";
            buff << "  parent_id  : " << rbdl_parent_id << "\n";
            buff << "  joint_frame: " << rbdl_joint_frame << "\n";
            buff << "  joint dofs : " << rbdl_joint.mDoFCount << "\n";
            for (size_t ii = 0; ii < rbdl_joint.mDoFCount; ii++)
            {
                buff << "    " << ii << ": " << rbdl_joint.mJointAxes[ii].transpose() << "\n";
            }
            buff << "  body inertia: " << std::endl << rbdl_body.mSpatialInertia << "\n";
            buff << "  bodyName  : " << urdf_child->name << "\n";
            buff << "  joint_name  : " << joint_names[jj];

            CONTROLIT_INFO << buff.str();
        }

        rbdl_robot->AddBody(rbdl_parent_id, rbdl_joint_frame, rbdl_joint, rbdl_body, joint_names[jj]); //RBDL Bodies have the name of their parent joint
    }

    // Debug output!  Print the properties of the final robot model
    // std::stringstream ss;
    // std::vector<RigidBodyDynamics::Body> & bodyList = rbdl_robot->mBodies;
    // for (size_t ii = 0; ii < bodyList.size(); ii++)
    // {
    //     ss << "  - Body " << ii << ", mass: " << bodyList[ii].mMass << ", COM: " << bodyList[ii].mCenterOfMass.transpose()
    //        << ", mInertia: \n" << bodyList[ii].mInertia << std::endl;
    // }
    // PRINT_DEBUG_STATEMENT("Final Robot Model:\n" << ss.str());

    return true;
}

bool augment_xml(std::string const& root_link_name,
                 TiXmlDocument& robot_xml)
{
    TiXmlElement * urdf_root(robot_xml.FirstChildElement("robot"));
    //Need to generate this xml for rbdl to parse virtual DOFs consistently
    /*
  <link name="world">
    <origin rpy="0 0 0" xyz="0 0 0"/>
  </link>

  <joint name="rigid6DoF" type="floating">
    <origin rpy="0.0 0 0" xyz="0 0 0"/>
    <parent link="world"/>
    <child link="robot_body_1"/>
  </joint>
  */
    TiXmlElement * floating_joint = new TiXmlElement("joint");
    floating_joint->SetAttribute("name", "rigid6DoF");
    floating_joint->SetAttribute("type", "floating");
    TiXmlElement * floating_joint_origin = new TiXmlElement("origin");
    floating_joint_origin->SetAttribute("xzy", "0.0 0.0 0.0");
    floating_joint_origin->SetAttribute("rpy", "0.0 0.0 0.0");
    TiXmlElement * floating_joint_parent = new TiXmlElement("parent");
    floating_joint_parent->SetAttribute("link", "world");
    TiXmlElement * floating_joint_child = new TiXmlElement("child");
    floating_joint_child->SetAttribute("link", root_link_name);

    floating_joint->LinkEndChild(floating_joint_origin);
    floating_joint->LinkEndChild(floating_joint_parent);
    floating_joint->LinkEndChild(floating_joint_child);

    TiXmlElement * world_link = new TiXmlElement("link");
    world_link->SetAttribute("name", "world");
    TiXmlElement * world_link_origin = new TiXmlElement("origin");
    world_link_origin->SetAttribute("xzy", "0.0 0.0 0.0");
    world_link_origin->SetAttribute("rpy", "0.0 0.0 0.0");

    world_link->LinkEndChild(world_link_origin);

    urdf_root->LinkEndChild(world_link);
    urdf_root->LinkEndChild(floating_joint);
    return true;
}

bool add_floating_joint(std::string const& urdf_description,
                        std::string const& root_link_name,
                        urdf::Model& urdf_model)
{
    // CONTROLIT_INFO << "Adding floating joint to link \"" << root_link_name << "\"";
    TiXmlDocument robot_xml;
    robot_xml.Parse(urdf_description.c_str(), 0, TIXML_ENCODING_UTF8);
    augment_xml(root_link_name, robot_xml);
    return urdf_model.initXml(&robot_xml);
}

// bool add_floating_joint(const char* filename,
//   const std::string& root_link_name,
//   urdf::Model& urdf_model)
// {
//   TiXmlDocument robot_xml(filename);
//   if(robot_xml.LoadFile())
//   {
//     augment_xml(robot_xml, root_link_name);
//     return(urdf_model.initXml(&robot_xml));
//   }
//   else
//     return robot_xml.LoadFile();
// }

} // namespace rbdl_robot_urdfreader
} // namespace controlit
