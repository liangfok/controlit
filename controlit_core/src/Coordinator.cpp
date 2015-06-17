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

#include <controlit/Coordinator.hpp>

#include <limits>
#include <typeinfo>
#include <time.h>

#include <controlit/Constraint.hpp>
#include <controlit/ConstraintSet.hpp>
#include <controlit/Controller.hpp>
#include <controlit/CompoundTaskFactory.hpp>
#include <controlit_robot_models/rbdl_robot_urdfreader.hpp>
#include <controlit/utility/string_utility.hpp>
#include <controlit/Diagnostics.hpp>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <RigidBodyDynamics/Extras/rbdl_extras.hpp>

#include <chrono>
#include <math.h>

#include <controlit/logging/RealTimeLogging.hpp>

// #include <std_msgs/Float64MultiArray.h> // for transmitting Float64MultiArray command messages
// #include <std_msgs/Float64.h>  // for transmitting the model latency

namespace controlit {

// Uncomment one of the following lines to enable/disable detailed debug statements.
#define PRINT_INFO_STATEMENT(ss)
// #define PRINT_INFO_STATEMENT(ss) CONTROLIT_INFO_RT << ss;

#define PRINT_INFO_STATEMENT_RT(ss)
// #define PRINT_INFO_STATEMENT_RT(ss) CONTROLIT_INFO_RT << ss;

#define PRINT_INFO_STATEMENT_RT_ALWAYS(ss) CONTROLIT_INFO_RT << ss;
// #define PRINT_INFO_STATEMENT_RT_ALWAYS(ss) std::cout << ss << std::endl;

#define NUM_SERVO_UPDATE_INTERNAL_LATENCIES 7
#define INDEX_LATENCY_READ 0
#define INDEX_LATENCY_PUBLISH_ODOM 1
#define INDEX_LATENCY_MODEL_UDPATE 2
#define INDEX_LATENCY_COMPUTE_COMMAND 3
#define INDEX_LATENCY_EVENTS 4
#define INDEX_LATENCY_WRITE 5
#define INDEX_LATENCY_SERVO 6

Coordinator::Coordinator() :
    model(nullptr),
    jointStatePublisher("/joint_states", 1),
    modelStalenessPublisher("diagnostics/modelStaleness", 1),
    servoComputeLatencyPublisher("diagnostics/servoComputeLatency", 1),
    servoFrequencyPublisher("diagnostics/servoFrequency", 1),
    initialized(false),
    running(false),
    taskUpdater(nullptr),
    servoClock(nullptr),
    robotInterface(nullptr),
    compoundTask(nullptr),
    controller(nullptr)
    // isFirstState(true),
    // isFirstCommand(true)
{
}

Coordinator::~Coordinator()
{
    PRINT_INFO_STATEMENT("Method called.")

    PRINT_INFO_STATEMENT("Unbinding the compound tasks' parameters.")

    if (compoundTask != nullptr) bindingManager.unbindParameters(*compoundTask);

    PRINT_INFO_STATEMENT("Deleting the RTControlModel.")

    if (model != nullptr) delete model;

    PRINT_INFO_STATEMENT("Deleting the task updater.")

    if (taskUpdater != nullptr) delete taskUpdater;

    PRINT_INFO_STATEMENT("Done method call.")
}

bool Coordinator::init()
{
    PRINT_INFO_STATEMENT("Method called, initialized = " <<  (initialized ? "true" : "false"))

    // Ensure this controller is only initialized once.
    assert(!initialized);

    // Create the ROS node handle
    ros::NodeHandle nh;

    PRINT_INFO_STATEMENT("Initializing ControlIt! controller named \"" <<  nh.getNamespace() << "\"...")

    // Load the parameters
    if (!loadParameters(nh)) return false;

    // Instantiate the control model, which may be single or multi threaded.
    if (controlitParameters.useSingleThreadedControlModel())
    {
        PRINT_INFO_STATEMENT("Using single-threaded Control Model.");
        model = new SingleThreadedControlModel();
    }
    else
    {
        PRINT_INFO_STATEMENT("Using multi-threaded Control Model.");
        model = new RTControlModel();
    }

    // Initialize diagnostics and the parameter binding manager.
    diagnostics.init(nh, this);
    bindingManager.init(nh);

    try
    {
        PRINT_INFO_STATEMENT("Loading model...");
        if (!loadModel(nh)) return false;

        PRINT_INFO_STATEMENT("Loading compound task...");
        if (!loadCompoundTask(nh)) return false;

        // PRINT_INFO_STATEMENT("Loading sensor set...");
        // if (!loadSensorSet(nh)) return false;

        PRINT_INFO_STATEMENT("Applying parameters...");
        if (!applyParameters()) return false;

        // Verify that the WBC parameters are valid
        PRINT_INFO_STATEMENT("Checking parameters...");
        if (!controlitParameters.checkParameters()) return false;

        // Create and initialize a WBC controller
        controller.reset(controllerFactory.createController(controlitParameters.getControllerType()));
        if (controller.get() == nullptr)
        {
            CONTROLIT_ERROR 
                << "Failed to create a whole body controller of type \"" 
                << controlitParameters.getControllerType() << "\"!";
            return false;
        }
    }
    catch (std::exception & err)
    {
        CONTROLIT_ERROR_RT << "Exception thrown during initialization: " << err.what();
        assert(false);
        return false;
    }

    // Create and initialize the robot interface
    std::string robotInterfaceType = controlitParameters.getRobotInterfaceType();
    PRINT_INFO_STATEMENT("Creating robot interface of type \"" << robotInterfaceType << "\"...");
    robotInterface.reset(robotInterfaceFactory.create(robotInterfaceType));

    if (robotInterface.get() != nullptr)
    {
        PRINT_INFO_STATEMENT("Initializing robot interface...");
        if (robotInterface->init(nh, model))
        {
            servoFreqTimer = robotInterface->getTimer();
            servoLatencyTimer = robotInterface->getTimer();
            PRINT_INFO_STATEMENT("Done initializing robot interface...");
        }
        else
        {
            CONTROLIT_ERROR_RT << "Unable to initialize robot interface!";
            return false;
        }
    }
    else
    {
        CONTROLIT_ERROR_RT << "Unable to create robot interface!";
        return false;
    }

    PRINT_INFO_STATEMENT("Initializing controller");
    controller->init(nh, *model->get(), & controlitParameters, robotInterface->getTimer());

    // Create and initialize the servoClock
    std::string servoClockType = controlitParameters.getServoClockType();
    PRINT_INFO_STATEMENT("Creating servo clock of type \"" << servoClockType << "\"...");
    servoClock.reset(servoClockFactory.create(servoClockType));

    if (servoClock.get() != nullptr)
    {
        PRINT_INFO_STATEMENT("Initilizing robot servo clock...");
        if (servoClock->init(this))
        {
            PRINT_INFO_STATEMENT("Done initializing servo clock...");
        }
        else
        {
            CONTROLIT_ERROR_RT << "Unable to initialize servo clock!";
            return false;
        }
    }
    else
    {
        CONTROLIT_ERROR_RT << "Unable to create servo clock!";
        return false;
    }

    // Create a real-time publisher of the model staleness
    
    while (!modelStalenessPublisher.trylock()) usleep(200);
    modelStalenessPublisher.msg_.data = 0;
    modelStalenessPublisher.unlockAndPublish();

    // Create a real-time publisher of the servo compute latency
    while (!servoComputeLatencyPublisher.trylock()) usleep(200);
    servoComputeLatencyPublisher.msg_.layout.dim.resize(NUM_SERVO_UPDATE_INTERNAL_LATENCIES);
    servoComputeLatencyPublisher.msg_.layout.dim[0].stride = NUM_SERVO_UPDATE_INTERNAL_LATENCIES;
    servoComputeLatencyPublisher.msg_.layout.dim[0].size = NUM_SERVO_UPDATE_INTERNAL_LATENCIES;
    servoComputeLatencyPublisher.msg_.layout.dim[1].stride = 1;
    servoComputeLatencyPublisher.msg_.layout.dim[1].size = 1;
    servoComputeLatencyPublisher.msg_.data.resize(NUM_SERVO_UPDATE_INTERNAL_LATENCIES);
    servoComputeLatencyPublisher.unlockAndPublish();

    // Create a real-time publisher of the servo frequency
    while (!servoFrequencyPublisher.trylock()) usleep(200);
    servoFrequencyPublisher.msg_.data = 0;
    servoFrequencyPublisher.unlockAndPublish();

    // Create a real-time publisher of the joint states. This information is used by
    // robot_state_publisher to publish the transforms for RViz visualization.
    while (!jointStatePublisher.trylock()) usleep(200);

    const std::vector<std::string> & jointNames = model->get()->getRealJointNamesVector();

    for (auto & name : jointNames)
    {
        jointStatePublisher.msg_.name.push_back(name);
        jointStatePublisher.msg_.position.push_back(0.0);  // allocate memory for the joint states
        jointStatePublisher.msg_.velocity.push_back(0.0);
        jointStatePublisher.msg_.effort.push_back(0.0);
    }

    jointStatePublisher.unlockAndPublish();
    

    // Create a service for getting the controller configuration
    getControllerConfigService = nh.advertiseService("diagnostics/getControllerConfiguration",
        &Coordinator::getControllerConfigServiceHandler, this);

    PRINT_INFO_STATEMENT("Controller initialized.")

    initialized = true;
    running = false;

    PRINT_INFO_STATEMENT("Done method call.")

    return true;
}

bool Coordinator::getControllerConfigServiceHandler(controlit_core::getControllerConfig::Request & req,
    controlit_core::getControllerConfig::Response &res)
{
    std::stringstream msgBuff;
    msgBuff << "===========================================================" << std::endl;
    // msgBuff << "State as of time offset " << deltaTime << /*sstream.str() <<*/ "s:" << std::endl;
    
    compoundTask->dump(msgBuff, "");
    model->get()->getConstraintSet()->dump(msgBuff, "");
    msgBuff << "===========================================================" << std::endl;
    res.controllerConfig.data = msgBuff.str();
    return true;
}

bool Coordinator::getTaskParameters(std::vector<std::string> & keys, std::vector<std::string> & values)
{
    return compoundTask->getParameters(keys, values);
}

bool Coordinator::getConstraintParameters(std::vector<std::string> & keys, std::vector<std::string> & values)
{
    model->getSwapLock();  // lock the active control model
    bool result = model->get()->constraints().getParameters(keys, values);
    model->releaseSwapLock(); // unlock the active control model (allow it to be swaped)
    return result;
}

bool Coordinator::forceUpdateControlModel(ControlModel * controlModel)
{
    robotInterface->read(latestRobotState); // Get the latest robot state
    controlModel->updateJointState();       // Give the latest robot state to the provided control model.
    controlModel->update();                 // update the control model using the new joint state
    return true;
}

bool Coordinator::start()
{
    PRINT_INFO_STATEMENT("Method Called!")

    // Ensure starting() is not called twice without stopping() being called inbetween.
    if (running)
    {
        CONTROLIT_ERROR << "Attempted to start twice.";
        return false;
    }

    // Start the model update thread
    model->startThread();

    // Start the task updater thread
    taskUpdater->startThread();

    running = true;

    CONTROLIT_INFO_RT << "Starting whole body controller...";

    servoClock->start(controlitParameters.getServoFrequency());

    return true;
}

// This is called by the ServoClock once after it is started.
void Coordinator::servoInit()
{
    forceUpdateControlModel(model->get());     // Update the active control model
    servoFreqTimer->start();                   // Start the servo frequency timer. First measurement will be wrong but can be ignored.
}

// This is periodically called by the ServoClock.
void Coordinator::servoUpdate()
{
    // #define TIME_CONTROLIT_CONTROLLER_UPDATE 1

    // Start recording the servo compute time.
    servoLatencyTimer->start();

    // If possible, publish the servo frequency.
    if(servoFrequencyPublisher.trylock())
    {
        double elapsedTime = servoFreqTimer->getTime();
        servoFrequencyPublisher.msg_.data = 1.0 / elapsedTime;
        servoFrequencyPublisher.unlockAndPublish();
    }

    // Update the time when the servo loop last executed
    servoFreqTimer->start();
    
    // updateLatencyStat.startTimer();

    // #ifdef TIME_CONTROLIT_CONTROLLER_UPDATE
    // std::shared_ptr<Timer> servoUpdateTimer = robotInterface->getTimer();
    // servoUpdateTimer->start();
    // #endif

    // Get the latest robot state information.
    bool readSuccess = robotInterface->read(latestRobotState);

    // Abort this cycle of the servo loop if we failed to get updated state information.
    if (!readSuccess)
    {
        // updateLatencyStat.cancelTimer();
        return;
    }
    // else
    // {
    //     if (isFirstState)
    //     {
    //         char buffer[30];
    //         struct timeval tv;
    //         time_t curtime;

    //         gettimeofday(&tv, NULL); 
    //         curtime=tv.tv_sec;
          
    //         strftime(buffer,30,"%m-%d-%Y  %T.", localtime(&curtime));

    //         CONTROLIT_INFO_RT << "Receiving first state at time " << buffer << tv.tv_usec;

    //         isFirstState = false;
    //     }
    // }

    latencyRead = servoLatencyTimer->getTime();

    // Publish the current joint states
    if(jointStatePublisher.trylock())
    {
        const Vector & jointPosition = latestRobotState.getJointPosition();
        const Vector & jointVelocity = latestRobotState.getJointVelocity();
        const Vector & jointEffort = latestRobotState.getJointEffort();

        for (int ii = 0; ii < jointPosition.size(); ii++)
        {
            jointStatePublisher.msg_.position[ii] = jointPosition[ii];
            jointStatePublisher.msg_.velocity[ii] = jointVelocity[ii];
            jointStatePublisher.msg_.effort[ii] = jointEffort[ii];
        }

        jointStatePublisher.unlockAndPublish();
    }

    // Publish the odometry information.  This is the state of the virtual 6 DOFs that connect the 
    // robot to the world.
    geometry_msgs::TransformStamped odom_trans;

    odom_trans.header.frame_id = "world";
    odom_trans.child_frame_id = model->get()->getBaseLinkName();

    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x = latestRobotState.getVirtualJointPosition()(0, 0); // x
    odom_trans.transform.translation.y = latestRobotState.getVirtualJointPosition()(1, 0); // y
    odom_trans.transform.translation.z = latestRobotState.getVirtualJointPosition()(2, 0); // z

    // The virtual DOFs in latestRobotState are stored in x,y,z,y,p,r order
    tf::Quaternion virtualDOFQuat = tf::createQuaternionFromRPY(
        latestRobotState.getVirtualJointPosition()(5, 0),  // roll
        latestRobotState.getVirtualJointPosition()(4, 0),  // pitch
        latestRobotState.getVirtualJointPosition()(3, 0)); // yaw

    odom_trans.transform.rotation.x = virtualDOFQuat.x();
    odom_trans.transform.rotation.y = virtualDOFQuat.y();
    odom_trans.transform.rotation.z = virtualDOFQuat.z();
    odom_trans.transform.rotation.w = virtualDOFQuat.w();

    tfBroadcaster.sendTransform(odom_trans);

    latencyPublishOdom = servoLatencyTimer->getTime();

    // #ifdef TIME_CONTROLIT_CONTROLLER_UPDATE
    // double doneRead = servoUpdateTimer->getTime();
    // #endif

    // Update the model
    updateModel();

    latencyModelUpdate = servoLatencyTimer->getTime();

    // #ifdef TIME_CONTROLIT_CONTROLLER_UPDATE
    // double doneUpdateModel = servoUpdateTimer->getTime();
    // #endif

    // Ensure the model is not stale!
    if(model->get()->isStale())
    {
        CONTROLIT_WARN_RT 
            << "Attempted to compute command using a stale model!"
            << "Aborting this round of the servo loop!";
    }

    // Compute command
    computeCommand();

    latencyComputeCmd = servoLatencyTimer->getTime();

    // #ifdef TIME_CONTROLIT_CONTROLLER_UPDATE
    // double doneComputeCommand = servoUpdateTimer->getTime();
    // #endif

    // Emit events
    emitEvents();

    latencyEvents = servoLatencyTimer->getTime();

    // #ifdef TIME_CONTROLIT_CONTROLLER_UPDATE
    // double doneEmitEvents = servoUpdateTimer->getTime();
    // #endif


    // Save the first command issued.  This is used by the diagnostic subsystem.
    // if (isFirstCommand)
    // {
    //     firstCommand = command;
    //     isFirstCommand = false;
    // }

    // Check to ensure command is valid
    if (!controlit::addons::eigen::checkMagnitude(command.getEffortCmd(), 1e4))
    {
        CONTROLIT_ERROR_RT << "Invalid effort command!  Not writing it to the robot.\n"
            << " - command.getEffortCmd(): " << command.getEffortCmd().transpose() << "\n"
            << " - model->get()->getQ(): " << model->get()->getQ().transpose() << "\n"
            << " - model->get()->getQd(): " << model->get()->getQd().transpose() << "\n"
            << " - latest joint position: " << latestRobotState.getVirtualJointPosition().transpose() << "   " << latestRobotState.getJointPosition().transpose() << "\n"
            << " - latestRobotState.getJointVelocity(): " << latestRobotState.getVirtualJointVelocity().transpose() << "   " << latestRobotState.getJointVelocity().transpose();
        // assert(false);
    }
    else
    {
        // if (isFirstCommand)
        // {
        //     char buffer[30];
        //     struct timeval tv;
        //     time_t curtime;

        //     gettimeofday(&tv, NULL); 
        //     curtime=tv.tv_sec;
          
        //     strftime(buffer,30,"%m-%d-%Y  %T.", localtime(&curtime));

        //     CONTROLIT_INFO_RT << "Sending first command at time " << buffer << tv.tv_usec;
        //     isFirstCommand = false;
        // }
        
        // Write command to the robot
        robotInterface->write(command);
    }

    latencyWrite = servoLatencyTimer->getTime();

    // #ifdef TIME_CONTROLIT_CONTROLLER_UPDATE
    // double doneWrite = servoUpdateTimer->getTime();
    // #endif

    // #ifdef TIME_CONTROLIT_CONTROLLER_UPDATE
    // PRINT_INFO_STATEMENT_RT_ALWAYS("Latency Results (ms):\n"
    //     " - read: " << doneRead << "\n"
    //     " - update model: " << (doneUpdateModel - doneRead) << "\n"
    //     " - compute command: " << (doneComputeCommand - doneUpdateModel) << "\n"
    //     " - emit events: " << (doneEmitEvents - doneComputeCommand) << "\n"
    //     " - write: " << (doneWrite - doneEmitEvents));
    // #endif

    PRINT_INFO_STATEMENT_RT("Effort Command:\n" << controlit::utility::prettyPrintJointSpaceCommand(model->get()->getActuatedJointNamesVector(), command.getEffortCmd(), "  "))
    PRINT_INFO_STATEMENT_RT("Gravity:\n" << controlit::utility::prettyPrintJointSpaceCommand(model->get()->getActuatedJointNamesVector(),
        model->get()->getGrav().segment(model->get()->getNumVirtualDOFs(), model->get()->getNActuableDOFs()), "  "))

    // If possible, publish the servo compute latency measurement.
    if(servoComputeLatencyPublisher.trylock())
    {
        latencyServo = servoLatencyTimer->getTime();

        // CONTROLIT_INFO << "Computing servo compute latency...";
        servoComputeLatencyPublisher.msg_.data[INDEX_LATENCY_READ]            = latencyRead;
        servoComputeLatencyPublisher.msg_.data[INDEX_LATENCY_PUBLISH_ODOM]    = latencyPublishOdom - latencyRead;
        servoComputeLatencyPublisher.msg_.data[INDEX_LATENCY_MODEL_UDPATE]    = latencyModelUpdate - latencyPublishOdom;
        servoComputeLatencyPublisher.msg_.data[INDEX_LATENCY_COMPUTE_COMMAND] = latencyComputeCmd - latencyModelUpdate;
        servoComputeLatencyPublisher.msg_.data[INDEX_LATENCY_EVENTS]          = latencyEvents - latencyComputeCmd;
        servoComputeLatencyPublisher.msg_.data[INDEX_LATENCY_WRITE]           = latencyWrite - latencyEvents;
        servoComputeLatencyPublisher.msg_.data[INDEX_LATENCY_SERVO]           = latencyServo;

        servoComputeLatencyPublisher.unlockAndPublish();
    }
}

bool Coordinator::stop()
{
    PRINT_INFO_STATEMENT("Method Called!")

    assert(running);

    if (!servoClock->stop()) return false;
    if (model != nullptr) model->stopThread();
    if (taskUpdater != nullptr) taskUpdater->stopThread();
    // if (sensorSetUpdater != nullptr) sensorSetUpdater->stopThread();

    model->setStale();  // Mark the control models as being stale to prevent stale models from being used when the controller spins back up

    running = false;

    CONTROLIT_INFO_RT << "Stopping controller.";

    return true;
}

void Coordinator::updateModel()
{
    // #define TIME_CONTROLIT_CONTROLLER_UPDATE_MODEL 1

    #ifdef TIME_CONTROLIT_CONTROLLER_UPDATE_MODEL
    std::shared_ptr<Timer> modelUpdateTimer = robotInterface->getTimer();
    modelUpdateTimer->start();
    #endif

    checkForTaskAndModelUpdates();

    // Try to get the lock on the RTControlModel.
    // We know we got the lock if a pointer to the inactive control model is returned.

    #ifdef TIME_CONTROLIT_CONTROLLER_UPDATE_MODEL
    double startTryLock = modelUpdateTimer->getTime();
    #endif

    ControlModel * inactiveCtrlModel = model->trylock();

    // Only update the model if the lock is successfully obtained.
    if (inactiveCtrlModel != nullptr)
    {
        PRINT_INFO_STATEMENT_RT("Got lock!  Updating model.")

        #ifdef TIME_CONTROLIT_CONTROLLER_UPDATE_MODEL
        double startUpdateJointState = modelUpdateTimer->getTime();
        #endif

        // Update model with new joint states
        inactiveCtrlModel->updateJointState();

        #ifdef TIME_CONTROLIT_CONTROLLER_UPDATE_MODEL
        double startUnlockAndUpdate = modelUpdateTimer->getTime();
        #endif

        // Update the model's kinematics, etc...
        model->unlockAndUpdate();

        #ifdef TIME_CONTROLIT_CONTROLLER_UPDATE_MODEL
        double endUnlockAndUpdate = modelUpdateTimer->getTime();
        #endif

        #ifdef TIME_CONTROLIT_CONTROLLER_UPDATE_MODEL
        PRINT_INFO_STATEMENT_RT_ALWAYS("UpdateModel Latency Results (ms):\n"
            " - checkForUpdates: " << startTryLock << "\n"
            " - modelTryLock: " << (startUpdateJointState - startTryLock) << "\n"
            " - updateJointState: " << (startUnlockAndUpdate - startUpdateJointState) << "\n"
            " - unlockAndUpdate: " << (endUnlockAndUpdate - startUnlockAndUpdate));
        #endif
    }
    else
    {
        PRINT_INFO_STATEMENT_RT("Failed to get lock, not updating model.")
    }
}

bool Coordinator::loadCompoundTask(ros::NodeHandle &nh)
{
    // Create compound task
    std::string yamlParameters;

    if (!nh.getParam("controlit/parameters", yamlParameters))
    {
        CONTROLIT_ERROR_RT << "Parameter '" << nh.getNamespace() << "/controlit/parameters' is not set";
        return false;
    }

    compoundTask.reset(compoundTaskFactory.loadFromString(yamlParameters));
    if (compoundTask.get() == NULL)
    {
        CONTROLIT_ERROR_RT << "Failed to load compound task from file";
        return false;
    }

    // Initialize the compound task
    if (not compoundTask->init(*model->get()))
    {
        CONTROLIT_ERROR_RT << "Failed to initialize compound task";
        return false;
    }

    // Bind parameters to ROS topics
    try
    {
        if (!compoundTask->bindParameters(nh, bindingManager)) 
            return false;
        // if (!bindingManager.bindParameters(nh, *(compoundTask.get())))
        //     return false;
    }
    catch (std::invalid_argument const& err)
    {
        CONTROLIT_ERROR_RT << "Failed to bind parameter. Reason: " << err.what();
        return false;
    }

    if (controlitParameters.useSingleThreadedTaskUpdater())
    {
        PRINT_INFO_STATEMENT("Using single-threaded task updater.");
        taskUpdater = new controlit::SingleThreadedTaskUpdater();
    }
    else
    {
        PRINT_INFO_STATEMENT("Using multi-threaded task updater.");
        taskUpdater = new controlit::TaskUpdater();
    }

    if (!compoundTask->addTasksToUpdater(taskUpdater)) return false;
    
    PRINT_INFO_STATEMENT("Compound task and task updater loaded.");

    return true;
}

bool Coordinator::loadModel(ros::NodeHandle &nh)
{
    PRINT_INFO_STATEMENT("Method called!");

    // Initialize the RTControlModel
    if (!model->init(nh, &latestRobotState, & bindingManager, & controlitParameters))
    {
        CONTROLIT_ERROR_RT << "Failed to load control model";
        return false;
    }

    // Initialize the command
    PRINT_INFO_STATEMENT("Initializing the command...");
    command.init(model->get()->getActuatedJointNamesVector());

    // Initialize the robot state, i.e., the joint states, velocities, and accelerations.
    latestRobotState.init(model->get()->getRealJointNamesVector());

    PRINT_INFO_STATEMENT("Model loaded.");

    return true;
}

bool Coordinator::loadParameters(ros::NodeHandle & nh)
{
    return controlitParameters.init(nh);
}

bool Coordinator::applyParameters()
{
    PRINT_INFO_STATEMENT("Method called!")

    // Set the gravity vector
    model->setGravityVector(controlitParameters.getGravityVector());

    // Decouple the A matrix if the decoupled joint groups are specified
    if(controlitParameters.hasCoupledJointGroups())
    {
        // Get the A matrix mask
        const std::vector<std::vector<std::string>> & mask = controlitParameters.getCoupledJointGroups();

        // Set the A matrix mask
        model->setAMask(mask);
    }

    // Set the gravity mask if it's been specified
    if(controlitParameters.hasGravCompMask())
    {
        std::vector< std::string> mask = controlitParameters.getGravCompMask();
        model->setGravMask(mask);
    }

    PRINT_INFO_STATEMENT("ControlIt! parameters loaded.")

    return true;
}

void Coordinator::printModelDetails()
{
    // Print out the model's name to ID map
    std::stringstream msgBuff;
    std::map< std::string, unsigned int> robotJointList = (model->get())->rbdlModel().mBodyNameMap;

    for (std::map<std::string, unsigned int>::iterator iter = robotJointList.begin();
        iter != robotJointList.end(); iter++)
    {
        // std::indexString =
        // if (iter->second > (model.get())->rbdlModel().mBodies.size())
        msgBuff << "  Joint: " << iter->first << ", Index: " << iter->second  << "\n";
    }

    std::string msg = msgBuff.str();

    // remove trailing new line
    if (!msg.empty() && msg[msg.length() - 1] == '\n')
        msg.erase(msg.length() - 1);

    // std::stringstream indicesMsgBuff;
    // for (size_t ii = 0; ii < rbdlJointIndices.size(); ii++)
    // {
    //     indicesMsgBuff << rbdlJointIndices[ii];
    //     if (ii < rbdlJointIndices.size() - 1)
    //         indicesMsgBuff << ", ";
    // }

    PRINT_INFO_STATEMENT("Joints and their IDs:\n"
        << msg << "\n"
        << "Total number of joints: " << model->get()->rbdlModel().mBodies.size());
}

void Coordinator::addEventListener(boost::function<void(std::string const&)> listener)
{
    // Register against compound task and ..
    compoundTask->addListener(listener);

    // constraint set
    model->addListenerToConstraintSet(listener);
}

bool Coordinator::computeCommand()
{
    PRINT_INFO_STATEMENT_RT("Method Called!")

    // #define TIME_CONTROLIT_CONTROLLER_COMPUTE_COMMAND 1

    #ifdef TIME_CONTROLIT_CONTROLLER_COMPUTE_COMMAND
    std::shared_ptr<Timer> computeCommandTimer = robotInterface->getTimer();
    computeCommandTimer->start();
    #endif

    // Check if we can swap the ControlModel
    checkForTaskAndModelUpdates();

    #ifdef TIME_CONTROLIT_CONTROLLER_COMPUTE_COMMAND
    double startComputeCommand = computeCommandTimer->getTime();
    #endif

    bool result = controller->computeCommand(*(model->get()), *(compoundTask.get()), command);

    #ifdef TIME_CONTROLIT_CONTROLLER_COMPUTE_COMMAND
    double endComputeCommand = computeCommandTimer->getTime();
    #endif

    if (!result) 
    {
        return false;
        // TODO: go into a safe mode that parks the robot.
    }
    else
    {
        if (modelStalenessPublisher.trylock())
        {
            double age = model->get()->getAge();
            modelStalenessPublisher.msg_.data = age;
            modelStalenessPublisher.unlockAndPublish();   
        }
    }

    #ifdef TIME_CONTROLIT_CONTROLLER_COMPUTE_COMMAND
    PRINT_INFO_STATEMENT_RT_ALWAYS("ComputeCommand Latency Results (ms):\n"
        " - checkForTaskAndModelUpdates: " << startComputeCommand << "\n"
        " - computeCommand: " << (endComputeCommand - startComputeCommand));
    #endif

    return true;
}

void Coordinator::checkForTaskAndModelUpdates()
{
    taskUpdater->checkTasksForUpdates();

    // Only attempt to update the control model when the TaskUpdater is IDLE
    if (taskUpdater->getState() == TaskUpdater::State::IDLE)
    {
        // This is needed to account for the following situation:
        // (1) TaskUpdater::checkTasksForUpdates() begins to check some of the tasks
        // (2) TaskUpdaterupdateLoop() interrupts and runs to completion updating all of the tasks.
        // (3) TaskUpdater::checkTasksForUpdates() runs to completion.
        // Note that when this occurs, some of the updated tasks are missed.
        taskUpdater->checkTasksForUpdates();

        // Check if we can swap the ControlModel
        bool updateOccured = model->checkUpdate();

        if (updateOccured)
            taskUpdater->updateTasks(model->get());
    }
}

bool Coordinator::emitEvents()
{
    if (!compoundTask->emitEvents()) return false;
    if (!(model->get())->constraints().emitEvents()) return false;
    return true;
}

const std::vector<std::string> & Coordinator::getActuatedJointNames() const
{
    return model->get()->getActuatedJointNamesVector();
}

const std::vector<std::string> & Coordinator::getRealJointNames() const
{
    return model->get()->getRealJointNamesVector();
}

} // namespace controlit
