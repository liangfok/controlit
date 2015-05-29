#include <gtest/gtest.h>
#include <RigidBodyDynamics/Extras/rbdl_extras.hpp>
#include <controlit/ControlModel.hpp>
#include <controlit/RobotState.hpp>
#include <controlit/addons/eigen/LinearAlgebra.hpp>
#include <drc/common/math_utilities.hpp>

using RigidBodyDynamics::Math::Vector3d;
using RigidBodyDynamics::Math::VectorNd;
using RigidBodyDynamics::Math::Matrix3d;
using RigidBodyDynamics::Math::MatrixNd;
using RigidBodyDynamics::Math::SpatialVector;
using RigidBodyDynamics::Math::Xtrans;

using controlit::RobotState;

/*----------------------------------------------------------------------------
 * rbdl_extras tests
 *--------------------------------------------------------------------------*/
class RbdlExtrasTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    RigidBodyDynamics::Model * model = new RigidBodyDynamics::Model();
    model->Init();
    model->gravity = Vector3d(0, 0, -9.81);

    controlit::ControlModel::LinkNameToJointNameMap_t * l2jmap = new controlit::ControlModel::LinkNameToJointNameMap_t();

    // Body A:
    //  - mass: 1
    //  - COM: (0, 0, 0)
    //  - radii of gyration: (1, 1, 1)
    RigidBodyDynamics::Body body_a(1, Vector3d(0, 0, 0), Vector3d(1, 1, 1));

    // Joint A:
    //  - name: rigid6DoF
    //  - 6 DOF joint, first three joints are prismatic tx, ty, tz, second three
    //    joints are revolute rx, ry, yz
    RigidBodyDynamics::Joint joint_a(SpatialVector(0, 0, 0, 1, 0, 0),
                                     SpatialVector(0, 0, 0, 0, 1, 0),
                                     SpatialVector(0, 0, 0, 0, 0, 1),
                                     SpatialVector(1, 0, 0, 0, 0, 0),
                                     SpatialVector(0, 1, 0, 0, 0, 0),
                                     SpatialVector(0, 0, 1, 0, 0, 0));

    // Body A attaches to the world via joint rigid6DoF, which is located
    // at the world frame's (0, 0, 0).  This means rigid6DoF's frame is equal
    // to the world frame.
    model->AppendBody(Xtrans(Vector3d(0, 0, 0)), joint_a, body_a, "rigid6DoF");

    // Body B:
    //  - mass: 1
    //  - COM: (0, 0, 0.25)
    //  - radii of gyration: (1, 1, 1)
    RigidBodyDynamics::Body body_b(1, Vector3d(0, 0, 0.25), Vector3d(1, 1, 1));

    // Joint revolute1DoF_1:
    //  - 1 DOF joint around the X axis of rigid6DoF's frame.
    RigidBodyDynamics::Joint joint_b(RigidBodyDynamics::JointTypeRevolute, Vector3d(1, 0, 0));

    // Body B attaches to Body A via joint 'revolute1DoF_1', which is located
    // at rigid6DoF's frame's (0, 0, 0).
    model->AppendBody(Xtrans(Vector3d(0, 0, 0)), joint_b, body_b, "revolute1DoF_1");

    // Body C:
    //  - mass: 1
    //  - COM: (0, 0, 0.25)
    //  - radii of gyration: (1, 1, 1)
    RigidBodyDynamics::Body body_c(1, Vector3d (0, 0, 0.25), Vector3d (1, 1, 1));

    // Joint revolute1DoF_2:
    //  - 1 DOF joint around the Y axis of revolute1DoF_1's frame.
    RigidBodyDynamics::Joint joint_c(RigidBodyDynamics::JointTypeRevolute, Vector3d(0, 1, 0));

    // Body C attaches to Body B via joint 'revolute1DoF_1', which is located
    // revolute1DoF_1's frame's (0, 0, 0.5).
    model->AppendBody(Xtrans(Vector3d(0, 0, 0.5)), joint_c, body_c, "revolute1DoF_2");

    // Make a RobotState object
    robotState.reset(new RobotState);
    robotState->init(model->dof_count);

    // Make a ControlModel object
    controlModel.reset(new controlit::ControlModel());
    controlModel->init(model, robotState, l2jmap, new controlit::ConstraintSet());
  }

  virtual void TearDown()
  {
    // controlModel.reset();
  }

  std::shared_ptr<controlit::RobotState> robotState;
  std::unique_ptr<controlit::ControlModel> controlModel;
};



TEST_F(RbdlExtrasTest, COMTest)
{
  // Verify that the model's DOF count is correct
  EXPECT_TRUE(controlModel->getNumDOFs() == 8)
    << "DOF count is " << controlModel->getNumDOFs() << ", expected 8";

  // Verify that the models number of actuable DOFs is correct
  EXPECT_TRUE(controlModel->getNActuableDOFs() == 2)
    << "Actual DOF count is " << controlModel->getNActuableDOFs() << ", expected 2";

  // Specify the joint state.
  // All joints are at position zero except for the last one (Q(1), i.e., revolute1DoF_2),
  // which is at position PI/2.
  // Vector Q(controlModel->getNumDOFs() - 6); Q.setZero(); Q(1) = boost::math::constants::pi<double>() / 2;
  // Vector Qd(controlModel->getNumDOFs() - 6); Qd.setZero();
  // Vector Qdd(controlModel->getNumDOFs() - 6); Qdd.setZero();

  // RobotState robotState;
  robotState->init(controlModel->getNActuableDOFs());
  robotState->setJointPosition(1, boost::math::constants::pi<double>() / 2);

  // Specify the robot's base state
  Vector3d position; position.setZero();
  Vector twist(6); twist.setZero();
  drc::common::aliases::Quaternion orientation;
  orientation.setIdentity();

  robotState->setRobotBaseState(position, orientation, twist);

  // Update the model
  controlModel->updateJointState();
  controlModel->update();

  // Compute the robot's COM
  // Vector3d COM = RigidBodyDynamics::Extras::calcRobotCOM(controlModel->rbdlModel(), Q);
  Vector3d COM = RigidBodyDynamics::Extras::calcRobotCOM(controlModel->rbdlModel(), controlModel->getQ());

  /*
   * The robot's expected COM is as follows:
   *
   *  - X: (1*0 + 1*0 + 1*0.25) / 3 = 0.083333333    (X points forward)
   *  - Y: (1*0 + 1*0 + 1*0) / 3 = 0                 (Y points left)
   *  - Z: (1*0 + 1*0.25 + 1*0.5) / 3 = 0.25         (Z points up)
   */
  Vector3d expectedCOM;
  expectedCOM(0) = 0.25 / 3;
  expectedCOM(1) = 0;
  expectedCOM(2) = 0.25;

  EXPECT_TRUE(COM == expectedCOM)
    << "COM = " << COM.transpose() << ", expected: " << expectedCOM.transpose();

  // Compute the COM jacobian matrix
  Matrix JvCOM(3, controlModel->getNumDOFs());
  RigidBodyDynamics::Extras::calcRobotJvCOM(controlModel->rbdlModel(), robotState->getJointPosition(), JvCOM);

  // TODO: Verify that expectedJvCOM is correct
  Matrix expectedJvCOM(3, controlModel->getNumDOFs());
  expectedJvCOM <<
    1, 0, 0,     0,    0.25,      0,     0,       0,
    0, 1, 0, -0.25,       0, 0.25/3, -0.25,       0,
    0, 0, 1,     0, -0.25/3,      0,     0, -0.25/3;

  EXPECT_TRUE(JvCOM == expectedJvCOM)
    << std::scientific << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1)
    << "JvCOM = \n" << JvCOM << "\nexpected:\n" << expectedJvCOM.transpose();

  // Compute FrCOM, the reaction force at the COM
  std::vector<VectorNd> FrCOM;
  std::vector<unsigned int> body_id; body_id.push_back(controlModel->rbdlModel().GetBodyId("revolute1DoF_2"));
  VectorNd Tau(controlModel->getNActuableDOFs());
  Tau << 0, 1;
  MatrixNd U(controlModel->getNActuableDOFs(), controlModel->getNumDOFs());
  U << 0, 0, 0, 0, 0, 0, 1, 0,
       0, 0, 0, 0, 0, 0, 0, 1;

  RigidBodyDynamics::Extras::calcRxnFrCOM(controlModel->rbdlModel(), controlModel->getQ(),
    controlModel->getAinv(), U, controlModel->getGrav(), body_id, Tau, FrCOM);

  // TODO: Verify by hand that expectedFrCOM0 is correct
  VectorNd expectedFrCOM0(6);
  // expectedFrCOM0 << -0.0875241061130335, 0, 3.2289529411764701, 0, -0.1725647058823536, 0;
  // expectedFrCOM0 << -0.1181982489727980, 0, 3.4610438286509200, 0, 1.1463018877476050, 0;
  //expectedFrCOM0 << 0.0064325732736819, 0, 3.2993119122377852, 0, 0.5009623113248758, 0;
  expectedFrCOM0 << 0.0426758938869666, 0, 3.7427999999999990, 0, -2.5167999999999999, 0;

  EXPECT_TRUE((FrCOM[0] - expectedFrCOM0).norm() < 1e-10)
    << std::scientific << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1)
    << "Unexpected FrCOM[0]\n"
    << " - computed: " << FrCOM[0].transpose() << "\n"
    << " - expected: " << expectedFrCOM0.transpose() << ",\n"
    << "norm of difference: " << (FrCOM[0] - expectedFrCOM0).norm();

  // Compute the Center Of Pressure (COP) in the WORLD frame using FrCOM (the reaction force at the COM)
  std::vector<VectorNd>COP;
  std::vector<VectorNd> contactNormal; contactNormal.push_back(Vector3d(0, 0, 1));
  std::vector<VectorNd> contactPlanePoint; contactPlanePoint.push_back(Vector3d(0, 0, 0.5));
  // RigidBodyDynamics::Extras::calcCOPfromFrCOM(controlModel->rbdlModel(), Q, body_id, FrCOM, contactNormal, contactPlanePoint, COP,
  //   RigidBodyDynamics::Extras::Frame::WORLD,
  //   RigidBodyDynamics::Extras::Frame::WORLD,
  //   RigidBodyDynamics::Extras::Frame::WORLD,
  //   RigidBodyDynamics::Extras::Frame::WORLD);
  RigidBodyDynamics::Extras::calcCOPfromFrCOM(controlModel->rbdlModel(), controlModel->getQ(), body_id, FrCOM, contactNormal, contactPlanePoint, COP,
    RigidBodyDynamics::Extras::Frame::WORLD,
    RigidBodyDynamics::Extras::Frame::WORLD,
    RigidBodyDynamics::Extras::Frame::WORLD,
    RigidBodyDynamics::Extras::Frame::WORLD);

  // TODO: Verify by hand that expectedCOP0World is correct
  VectorNd expectedCOP0World(3);
  // expectedCOP0World << 0.0602194382987092, 1.5707963267948968, 0.5;
  // expectedCOP0World << -0.4859554611827720, 1.5707963267948963, 0.5;
  // expectedCOP0World << -0.2300204097756557, 1.5707963267948968, 0.5;
  expectedCOP0World << 0.9224377471411781, 0, 0.5;

  EXPECT_TRUE((COP[0] - expectedCOP0World).norm() < 1e-10)
    << std::scientific << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1)
    << "Unexpected COP[0]:\n"
    << " - computed: " << COP[0].transpose() << "\n"
    << " - expected: " << expectedCOP0World.transpose() << "\n"
    << " - norm of difference: " << (COP[0] - expectedCOP0World).norm();

  // Compute the Center Of Pressure (COP) in the LOCAL frame using FrCOM (the reaction force at the COM)
  std::vector<VectorNd>cop;
  // RigidBodyDynamics::Extras::calcCOPfromFrCOM(controlModel->rbdlModel(), Q, body_id, FrCOM, contactNormal, contactPlanePoint, cop,
  RigidBodyDynamics::Extras::calcCOPfromFrCOM(controlModel->rbdlModel(), controlModel->getQ(), body_id, FrCOM, contactNormal, contactPlanePoint, cop,
    RigidBodyDynamics::Extras::Frame::WORLD,
    RigidBodyDynamics::Extras::Frame::WORLD,
    RigidBodyDynamics::Extras::Frame::WORLD,
    RigidBodyDynamics::Extras::Frame::LOCAL);

  // TODO: Verify by hand that expectedCOP0Local is correct
  VectorNd expectedCOP0Local(3);
  expectedCOP0Local.setZero(3);
  // expectedCOP0Local << 0.0602194382987092, 0, 0;
  // expectedCOP0Local << 0.3918353125405912, -0.0000000000000002, 0.2874331193505331;
  // expectedCOP0Local << 0.2183425987501994, 0.0000000000000002, 0.0723595085967974;
  expectedCOP0Local << 0, 0, 0.9224377471411781;
  EXPECT_TRUE((cop[0] - expectedCOP0Local).norm() < 1e-10)
    << std::scientific << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1)
    << "Unexpected cop[0]:\n"
    << " - computed: " << cop[0].transpose() << "\n"
    << " - expected: " << expectedCOP0Local.transpose() << "\n"
    << " - norm of difference: " << (cop[0] - expectedCOP0Local).norm();

  std::vector<VectorNd> FrCOP;
  //cop[0][2] = 0.25;
  RigidBodyDynamics::Extras::calcRxnFr(controlModel->rbdlModel(), controlModel->getQ(),
    controlModel->getAinv(), U, controlModel->getGrav(), body_id, COP, Tau, FrCOP);

  // TODO: Verify by hand that expectedFrCOP0 is correct
  VectorNd expectedFrCOP0(6);
  // expectedFrCOP0 << 0.0426758938869666, 0, 3.7427999999999990, -5.8791764919279386,
  //   -1.5785300816412318, 0.0670351373603359;
  // expectedFrCOP0 << 0.0426758938869666, 0, 3.7427999999999990, -5.8791764919279368,
  //   -1.6018385836952282, 0.0670351373603359;
  // expectedFrCOP0 << 0.0426758938869666, 0, 3.7427999999999990, -5.8791764919279386,
  //   -1.5909163265994226, 0.0670351373603359;
  expectedFrCOP0 << 0.0426758938869666, 0, 3.7427999999999990, 0, -1.5417341445856703, 0;

  EXPECT_TRUE((FrCOP[0] - expectedFrCOP0).norm() < 1e-10)
    << std::scientific << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1)
    << "Unexpected FrCOP[0]:\n"
    << " - computed = " << FrCOP[0].transpose() << "\n"
    << " - expected: " << expectedFrCOP0.transpose() << "\n"
    << " - norm of difference: " << (FrCOP[0] - expectedFrCOP0).norm();

  /*Vector3d mcop, fcop;
  Matrix3d R;
  for(int ii=0; ii<3; ii++)
  {
    mcop[ii] = FrCOP[0][ii+3];
    fcop[ii] = FrCOP[0][ii];
  }
  R = RigidBodyDynamics::CalcBodyWorldOrientation(controlModel->rbdlModel(),Q,2,false);
  mcop = R*mcop;
  fcop = R*fcop;*/

  Vector3d v1(0.3, 0.2, 0.4), v2(1, 0, 0);
  v1.normalize();
  Matrix3d Rfromv1tov2;
  Rfromv1tov2 = RigidBodyDynamics::Extras::calcRFromV1toV2(v1, v2);
  std::cout << "v1 = " << v1 << std::endl;
  std::cout << "R = " << Rfromv1tov2 << std::endl;
  std::cout << "R * v1 = " << Rfromv1tov2 * v1 << std::endl;
  std::cout << "R * R^T = " << Rfromv1tov2 * Rfromv1tov2.transpose() << std::endl;
  Vector3d check = Rfromv1tov2 * v1 - v2;
  EXPECT_TRUE(check.norm() < 1e-10)
    << std::scientific << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1)
    << "check.norm() = " << check.norm() << " exceeds 1e-10!";
}
