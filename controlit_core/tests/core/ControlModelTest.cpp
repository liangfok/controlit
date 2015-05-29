#include <gtest/gtest.h>
#include <resource_retriever/retriever.h>

#include <controlit/RobotState.hpp>
#include <controlit/ControlModel.hpp>
#include <controlit/utility/ControlItParameters.hpp>

// #include <drc/common/math_utilities.hpp>

using controlit::utility::ControlItParameters;

bool loadResource(std::string const& resourceUrl, std::string& contents)
{
    resource_retriever::MemoryResource resource;

    try
    {
        resource_retriever::Retriever r;
        resource = r.get(resourceUrl);
    }
    catch (resource_retriever::Exception& e)
    {
        CONTROLIT_WARN << "Failed to retrieve file: " << resourceUrl;
        return false;
    }

    // Convert loaded resource to string
    contents.resize(resource.size);
    memcpy(&contents[0], resource.data.get(), resource.size);

    return true;
}

#define DEG2RAD (boost::math::constants::pi<double>()/180.0)

struct TestData
{
    // Make test cases a little more readable
    typedef double Angle;
    typedef double LinearVelocity;
    typedef double AngularVelocity;

    typedef std::tuple<Angle, Angle, Angle> Orientation;
    typedef std::tuple<LinearVelocity, LinearVelocity, LinearVelocity, AngularVelocity, AngularVelocity, AngularVelocity> Twist;
    typedef std::tuple<Orientation, Twist> Input;
    typedef std::vector<Input> TestCases;
    TestCases testCases;

    TestData()
    {
        // Declared test cases
        testCases.push_back( std::make_tuple(
            std::make_tuple( 80 * DEG2RAD, -30 * DEG2RAD, -50 * DEG2RAD ),
            std::make_tuple( 0.5, 0, -1.0, 10 * DEG2RAD, -30 * DEG2RAD, -50 * DEG2RAD )
        ));

        // Declared test cases
        testCases.push_back( std::make_tuple(
            std::make_tuple( 0 * DEG2RAD, -30 * DEG2RAD, 50 * DEG2RAD ),
            std::make_tuple( 0.5, 0.5, 1.0, -60 * DEG2RAD, -50 * DEG2RAD, 180 * DEG2RAD )
        ));

        // Declared test cases
        testCases.push_back( std::make_tuple(
            std::make_tuple( 30 * DEG2RAD, -120 * DEG2RAD, -20 * DEG2RAD ),
            std::make_tuple( 0, -0.3, -1.0, 0 * DEG2RAD, 50 * DEG2RAD, 80 * DEG2RAD )
        ));

        // Declared test cases
        testCases.push_back( std::make_tuple(
            std::make_tuple(120 * DEG2RAD, -120 * DEG2RAD, 120 * DEG2RAD),
            std::make_tuple( 0.5, 0, 0, -120 * DEG2RAD, 10 * DEG2RAD, -5 * DEG2RAD )
        ));

        // Declared test cases
        testCases.push_back( std::make_tuple(
            std::make_tuple(-45 * DEG2RAD, -30 * DEG2RAD, -20 * DEG2RAD),
            std::make_tuple( 0, 0, 0, 0 * DEG2RAD, 0 * DEG2RAD, 0 * DEG2RAD )
        ));

        // Declared test cases
        testCases.push_back( std::make_tuple(
            std::make_tuple( 90 * DEG2RAD, 30 * DEG2RAD, -20 * DEG2RAD ),
            std::make_tuple( -2, 1, 0.3, 30 * DEG2RAD, -115 * DEG2RAD, 22.7 * DEG2RAD )
        ));
    };
};

bool testQuaternionEquality(drc::common::Quaternion const& q1, drc::common::Quaternion const& q2)
{
    if ((q1.w() > 0 && q2.w() > 0) ||
        (q1.w() <= 0 && q2.w() <= 0))
    {
        // if the signs of w are the same, do this check
        if ((fabs(q1.w() - q2.w()) < 1e-5) &&
            (fabs(q1.x() - q2.x()) < 1e-5) &&
            (fabs(q1.y() - q2.y()) < 1e-5) &&
            (fabs(q1.z() - q2.z()) < 1e-5))
        {
            return true;
        }
    }
    else
    {
        // signs of w's are different. check if q1 == -q2
        if ((fabs(q1.w() + q2.w()) < 1e-5) &&
            (fabs(q1.x() + q2.x()) < 1e-5) &&
            (fabs(q1.y() + q2.y()) < 1e-5) &&
            (fabs(q1.z() + q2.z()) < 1e-5))
        {
            return true;
        }
    }

    return false;
}

bool testVectorEquality(drc::common::Vector const& x1, drc::common::Vector const& x2)
{
    if (x1.rows() != x2.rows())
    {
        return false;
    }

    for (unsigned int n=0; n < x1.rows(); n++)
    {
        if (fabs(x1(n) - x2(n)) > 1e-6)
        {
            return false;
        }
    }

    return true;
}

class ControlModelTest : public ::testing::Test
{
protected:

    virtual void SetUp()
    {
        // Load the robot description
        std::string robotDescriptionUri = "package://controlit/tests/core/testData/controlModel_robotDescription.urdf";
        std::string robotDescription;
        EXPECT_TRUE(controlit::addons::ros::loadResource(robotDescriptionUri, robotDescription))
            << "Failed to load resource '" << robotDescriptionUri << "'";

        // Instantiate a ControlItParameters object
        ControlItParameters controlitParams;

        // Create a control model
        robotState.reset(new controlit::RobotState());
        controlModel.reset(controlit::ControlModel::createModel(robotDescription, "", robotState, &controlitParams));
        EXPECT_TRUE(controlModel.get() != nullptr) << "ControlModel::createModel() returned a NULL pointer.";

        robotState->init(controlModel->getNActuableDOFs());

        // Get the 'rigid6DoF' body ID
        bodyId = controlModel->rbdlModel().GetBodyId("rigid6DoF");
        EXPECT_FALSE(bodyId == std::numeric_limits<unsigned int>::max()) << "Body 'rigid6DoF' is not a valid model body.";

        testNumber = 0;
    }

    virtual void TearDown()
    {
    }

    std::shared_ptr<controlit::RobotState> robotState;
    std::shared_ptr<controlit::ControlModel> controlModel;

    unsigned int bodyId;

    TestData testData;

    unsigned int testNumber;
};

TEST_F(ControlModelTest, VirtualLinkOrientationTest)
{

    for (auto const& testCase : testData.testCases)
    {
        TestData::Orientation orientationData;
        TestData::Twist twistData;
        std::tie(orientationData, twistData) = testCase;

        TestData::Angle a1(0), a2(0), a3(0);
        std::tie(a1, a2, a3) = orientationData;

        // Create fake odometry data
        // NOTE: Creating this data is independent of the Euler sequence convention used by ControlModel,
        // the URDF -> RBLD parser, or your grandmother. Just want to create an arbitrary quaternion as
        // if it came from an IMU.
        drc::common::Quaternion quat_testcase = Eigen::AngleAxisd(a1, Eigen::Vector3d::UnitZ())
                            * Eigen::AngleAxisd(a2, Eigen::Vector3d::UnitX())
                            * Eigen::AngleAxisd(a3, Eigen::Vector3d::UnitZ());

        // Give the control model the desired orientation
        robotState->setRobotBaseState(Eigen::Vector3d::Zero(), quat_testcase, drc::common::Vector::Zero(6));
        controlModel->updateJointState();
        controlModel->update();

        // Now ask RBDL for the orientation of the rigid6Dof body.
        drc::common::Matrix3d R_rbdl = RigidBodyDynamics::CalcBodyWorldOrientation(controlModel->rbdlModel(),
            controlModel->getQ(), bodyId, false);

        // NOTE! RBDL returns NOT the transform of the body coordinates in the world frame but the transpose..
        // so the other way around. It returns the world frame w.r.t the body frame. Retarded.
        // Or said in another retarded way, it uses a right (or left handed?) notation. Who cares... non-conventional.
        // So compute its transpose before computing the quaternion to do the check.
        drc::common::Quaternion quat_rbdl( R_rbdl.transpose() );

        EXPECT_TRUE( testQuaternionEquality(quat_testcase, quat_rbdl) ) << "Failed test case " << testNumber++ << std::endl
           << "    quat_testcase: " << quat_testcase.w() << ", " << quat_testcase.x() << ", " << quat_testcase.y() << ", " << quat_testcase.z() << std::endl
           << "    quat_rbdl: " << quat_rbdl.w() << ", " << quat_rbdl.x() << ", " << quat_rbdl.y() << ", " << quat_rbdl.z() << std::endl;
    }
}

TEST_F(ControlModelTest, VirtualLinkAngularVelocityTest)
{
    for (auto const& testCase : testData.testCases)
    {
        TestData::Orientation orientationData;
        TestData::Twist twistData;
        std::tie(orientationData, twistData) = testCase;

        TestData::Angle a1(0), a2(0), a3(0);
        std::tie(a1, a2, a3) = orientationData;

        // Create fake odometry data
        // NOTE: Creating this data is independent of the Euler sequence convention used by ControlModel,
        // the URDF -> RBLD parser, or your grandmother. Just want to create an arbitrary quaternion as
        // if it came from an IMU.
        drc::common::Quaternion quat_testcase = Eigen::AngleAxisd(a1, Eigen::Vector3d::UnitZ())
                            * Eigen::AngleAxisd(a2, Eigen::Vector3d::UnitX())
                            * Eigen::AngleAxisd(a3, Eigen::Vector3d::UnitZ());

        // And a fake twist from an IMU. w1, w2, w3 are the components of the twist velocties of
        // the base frame expressed in the world coordinate system
        TestData::LinearVelocity v1(0), v2(0), v3(0);
        TestData::AngularVelocity w1(0), w2(0), w3(0);
        std::tie(v1, v2, v3, w1, w2, w3) = twistData;

        drc::common::Vector x_dot_test = drc::common::Vector::Zero(6);
        x_dot_test(0) = v1;
        x_dot_test(1) = v2;
        x_dot_test(2) = v3;
        x_dot_test(3) = w1;
        x_dot_test(4) = w2;
        x_dot_test(5) = w3;

        // Give the control model the desired orientation
        robotState->setRobotBaseState(Eigen::Vector3d::Zero(), quat_testcase, x_dot_test);
        controlModel->updateJointState();
        controlModel->update();

        // To check, we will get the jacobian and compute what we sent in.
        drc::common::Matrix Jv(3, controlModel->getNumDOFs());
        RigidBodyDynamics::CalcPointJacobian(controlModel->rbdlModel(),
            controlModel->getQ(), bodyId, Eigen::Vector3d::Zero(), Jv, false);
        drc::common::Matrix Jw(3, controlModel->getNumDOFs());
        RigidBodyDynamics::CalcPointJacobianW(controlModel->rbdlModel(),
            controlModel->getQ(), bodyId, Eigen::Vector3d::Zero(), Jw, false);

        drc::common::Matrix J(6, controlModel->getNumDOFs());
        J.topRows(3) = Jv;
        J.bottomRows(3) = Jw;
        drc::common::Vector x_dot_check = J * controlModel->getQd();

        EXPECT_TRUE(x_dot_test.isApprox(x_dot_check, 1e-6))
            << "Failed test case " << testNumber++ << std::endl
            << "    x_dot_test: " << x_dot_test.transpose() << std::endl
            << "    x_dot_check: " << x_dot_check.transpose() << std::endl;
    }
}