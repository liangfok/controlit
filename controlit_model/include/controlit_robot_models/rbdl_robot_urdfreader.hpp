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

#ifndef _RBDL_ROBOT_URDFREADER_H
#define _RBDL_ROBOT_URDFREADER_H

#include <rbdl/rbdl.h>

#include <vector>
#include <map>

#include <controlit_robot_models/WBCJointLimits.hpp>

namespace controlit {
namespace rbdl_robot_urdfreader {

/*!
 * Create a model based on URDF specifications.
 *
 * \param[in] filename The name of the file containing the URDF specification.
 * \param[out] robot A pointer to where the resulting model should be saved.
 * \param[out] linkToJointMap A reference to the map that stores the mapping between
 * the link names and joint names.
 * \param[in] verbose Whether to print status messages.
 */
bool read_urdf_model_from_file( std::string const& filename,
                                RigidBodyDynamics::Model *robot,
                                std::map<std::string, std::string> * linkToJointMap,
                                bool verbose = false);

/*!
 * Create a model based on URDF specifications.
 *
 * \param[in] urdf_description The URDF specification.
 * \param[out] robot A pointer to where the resulting model should be saved.
 * \param[out] linkToJointMap A reference to the map that stores the mapping between
 * the link names and joint names.
 * \param[in] verbose Whether to print status messages.
 */
bool read_urdf_model_from_string(const std::string &urdf_description,
                                 RigidBodyDynamics::Model *robot,
                                 std::map<std::string, std::string> * linkToJointMap,
                                 bool verbose = false);


/*!
 * Create a model based on URDF specifications.
 *
 * \param[in] urdf_description The URDF specification.
 * \param[out] robot A pointer to where the resulting model should be saved.
 * \param[out] linkToJointMap A reference to the map that stores the mapping between
 * the link names and joint names.
 * \param[out] limits pointer to where joint limits should be saved
 * \param[in] verbose Whether to print status messages.
 */
bool read_urdf_model_from_string(const std::string &urdf_description,
                                 RigidBodyDynamics::Model *robot,
                                 std::map<std::string, std::string> * linkToJointMap,
                                 WBCJointLimits *limits,
                                 bool verbose = false);

/*!
 * Create a model based on URDF specifications.
 *
 * \param[in] urdf_description The URDF specification.
 * \param[in] fixed_joint_list A list of joints which should explicitly be
 * fixed.
 * \param[out] robot A pointer to where the resulting model should be saved.
 * \param[out] linkToJointMap A reference to the map that stores the mapping
 * between the link names and joint names.
 * \param[in] verbose Whether to print status messages.
 */
bool read_urdf_model_from_string(const std::string &urdf_description,
                                 const std::vector<std::string> &fixed_joint_list,
                                 RigidBodyDynamics::Model *robot,
                                 std::map<std::string, std::string> *linkToJointMap,
                                 bool verbose = false);

/*!
 * \brief Create a model based on URDF specifications.
 *
 * \param[in] urdf_model The URDF specification.
 * \param[in] fixed_joint_list A list of joints which should explicitly be
 * fixed.
 * \param[in] fixed_joint_list A vector of joints which should explicitly be set
 * to fixed in the returned RBDL Model.
 * \param[out] robot A pointer to where the resulting model should be saved.
 * \param[out] linkToJointMap A reference to the map that stores the mapping
 * between the link names and joint names.
 * \param[out] limits pointer to where joint limits should be saved
 * \param[in] verbose Whether to print status messages.
 */
bool load_urdf_model(const std::string &urdf_description,
                     const std::vector<std::string> &fixed_joint_list,
                     RigidBodyDynamics::Model *robot,
                     std::map<std::string, std::string> * linkToJointMap,
                     WBCJointLimits *limits,
                     bool verbose);

/*!
 * Parse the sub-model that only includes the links in the "links" vector.
 * If the vector of links includes the root body, then any branches which
 * have been trimmed will be fused.  If the vector of links does *not* include
 * the root body, then a new root body will be set as the parent link of the
 * first specified joint.
 *
 * \param[in] filename The name of the file containing the URDF specification.
 * \param[in] links The links to include the in the sub model.
 * \param[out] robot A pointer to where the resulting model should be saved.
 * \param[out] linkToJointMap A reference to the map that stores the mapping between
 * the link names and joint names.
 * \param verbose Whether to print status messages.
 */
// bool read_urdf_submodel(const char * filename,
//                         const std::vector<std::string> * links,
//                         RigidBodyDynamics::Model * robot,
//                         std::map<std::string, std::string> &linkToJointMap,
//                         bool verbose = false);

/*!
 * Parse the sub-model that only includes the links in the "links" vector.
 * If the vector of links includes the root body, then any branches which
 * have been trimmed will be fused.  If the vector of links does *not* include
 * the root body, then a new root body will be set as the parent link of the
 * first specified joint.
 *
 * \param urdf_description The URDF specification.
 * \param links The links to include the in the sub model.
 * \param[out] robot A pointer to where the resulting model should be saved.
 * \param[out] linkToJointMap A reference to the map that stores the mapping between
 * the link names and joint names.
 * \param verbose Whether to print status messages.
 */
// bool read_urdf_submodel(const std::string urdf_description,
//                         const std::vector<std::string> * links,
//                         RigidBodyDynamics::Model * robot,
//                         std::map<std::string, std::string> &linkToJointMap,
//                         bool verbose = false);

} // namespace rbdl_robot_urdfreader
} // namespace controlit

/* _RBDL_URDFREADER_H */
#endif
