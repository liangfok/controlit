#include <gtest/gtest.h>

#include <controlit/ConstraintSetFactory.hpp>
#include <controlit/ControlModel.hpp>
#include <controlit/Constraint.hpp>
#include <controlit/Parameter.hpp>
#include <controlit/RobotState.hpp>

#include <ros/package.h> // for ros::package::getPath(...)

#include <Eigen/Dense>

//MODIFY BY POLAM
#include <RigidBodyDynamics/Extras/rbdl_extras.hpp>


using RigidBodyDynamics::Math::Xtrans;
using Eigen::Vector3d;
using RigidBodyDynamics::Math::SpatialVector;

class ConstraintSetFactoryTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        ros::Time::init();
        model = new RigidBodyDynamics::Model();
    
        model->Init();
        model->gravity = Vector3d(0.,0.,-9.81);
        
    
        // Body A
        RigidBodyDynamics::Body body_a(1., Vector3d(0.0, 0.0, 0.0), Vector3d(1.0, 1.0, 1.0));
        RigidBodyDynamics::Joint joint_a(SpatialVector(0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
                           SpatialVector(0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
                           SpatialVector(0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
                           SpatialVector(1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                           SpatialVector(0.0, 1.0, 0.0, 0.0, 0.0, 0.0),
                           SpatialVector(0.0, 0.0, 1.0, 0.0, 0.0, 0.0));
        model->AppendBody(Xtrans(Vector3d(0.0, 0.0, 0.0)), joint_a, body_a, "rigid6DoF");
        
    
        // Body B
        RigidBodyDynamics::Body body_b(1., Vector3d(0.0, 0.0, 0.25), Vector3d(1.0, 1.0, 1.0));
        RigidBodyDynamics::Joint joint_b(RigidBodyDynamics::JointTypeRevolute, Vector3d(1.0, 0.0, 0.0));
        model->AppendBody(Xtrans(Vector3d(0.0,0.0,0.5)), joint_b, body_b, "revolute1DoF_1");
        
    
        // Body C
        RigidBodyDynamics::Body body_c(0., Vector3d (0.0, 0.0, 0.25), Vector3d (1.0, 1.0, 1.0));
        RigidBodyDynamics::Joint joint_c(RigidBodyDynamics::JointTypeRevolute, Vector3d(0.0, 1.0, 0.0));
        model->AppendBody(Xtrans(Vector3d(0.0, 0.0, 0.5)), joint_c, body_c, "revolute1DoF_2");
        
    
        // Make a ControlModel object
        controlModel.reset( new controlit::ControlModel() );
    
        // Create a RobotState object
        robotState.reset(new controlit::RobotState());
    }
  
    virtual void TearDown()
    {
        controlModel.reset();
    }
  
    std::shared_ptr<controlit::RobotState> robotState;
    std::unique_ptr<controlit::ControlModel> controlModel;
    RigidBodyDynamics::Model * model;
};

/* ----------------------------------------------------------------------------
 *	TaskFactory tests
 * --------------------------------------------------------------------------*/
TEST_F(ConstraintSetFactoryTest, Load)
{
    std::string source_dir = ros::package::getPath("controlit");
    controlit::ControlModel::LinkNameToJointNameMap_t * l2jmap = new controlit::ControlModel::LinkNameToJointNameMap_t();
  
    controlit::ConstraintSetFactory constraintSetFactory;
  	controlit::ConstraintSet* cs = constraintSetFactory.loadFromFile(source_dir + "/tests/factories/constraint_set_factory_test.yaml");
    controlModel->init(model, robotState, l2jmap, cs);
  
  	EXPECT_TRUE(cs != NULL) << "Unable to create ConstraintSet!";
  
    // EXPECT_TRUE(cs->isConstrained(6)) << "Body 1 should be constrained but it isn't";
    // EXPECT_FALSE(cs->isConstrained(7)) << "Body 2 should not be constrained but it is";
    // EXPECT_FALSE(cs->isConstrained(8)) << "Body 3 should be constrained but it is";
  
    controlit::Parameter* intParam = cs->lookupParameter("RightFootContact.intVal");
    EXPECT_TRUE(intParam != NULL) << "intParam is NULL";
    EXPECT_TRUE(*(intParam->getInteger()) == -3) << "Value: " << *(intParam->getInteger());
  
    controlit::Parameter* realParam = cs->lookupParameter("RightFootContact.realVal");
    EXPECT_TRUE(realParam != NULL);
    EXPECT_TRUE(*(realParam->getReal()) == 4.5) << "Value: " << *(realParam->getReal());
  
    controlit::Parameter* stringParam = cs->lookupParameter("RightFootContact.stringVal");
    EXPECT_TRUE(stringParam != NULL);
    EXPECT_TRUE(*(stringParam->getString()) == "world2") << *(realParam->getString());
  
    controlit::Parameter* p = cs->lookupParameter("RightFootContact.masterNodeName");
    EXPECT_TRUE(p != NULL) << "Unable to obtain parameter RightFootContact.masterNodeName";
    EXPECT_TRUE(*(p->getString()) == "rigid6DoF") << "masterNodeName not correct. Expected rigid6DoF, got " << *(p->getString());
}
