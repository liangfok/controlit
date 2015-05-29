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

#include <controlit/RTControlModel.hpp>

#include <controlit/Constraint.hpp>

#include <controlit/logging/RealTimeLogging.hpp>
#include <std_msgs/Float64MultiArray.h> // for transmitting Float64MultiArray messages containing the gravity vector

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;

namespace controlit {

// Uncomment one of the following lines to enable/disable detailed debug statements.
#define PRINT_DEBUG_STATEMENT(ss)
// #define PRINT_DEBUG_STATEMENT(ss) CONTROLIT_DEBUG << ss;

#define PRINT_DEBUG_STATEMENT_RT(ss)
// #define PRINT_DEBUG_STATEMENT_RT(ss) CONTROLIT_DEBUG_RT << ss;

// #define PRINT_DEBUG_STATEMENT_RT_ALWAYS(ss) CONTROLIT_DEBUG_RT << ss;
#define PRINT_DEBUG_STATEMENT_RT_ALWAYS(ss) std::cout << ss << std::endl;

RTControlModel::RTControlModel() :
    state(State::IDLE),
    isRunning(false),
    keepRunning(true),
    initialized(false),
    activeModel(nullptr),
    inactiveModel(nullptr),
    parameterBindingManager(nullptr),
    modelUpdateLatencyPublisher("diagnostics/modelUpdateLatency", 1)
    // controlModelUpdateStat("Control Model Update", MODEL_UPDATE_NUM_SAMPLES)
{
    PRINT_DEBUG_STATEMENT("Method Called!")
}

RTControlModel::~RTControlModel()
{
    PRINT_DEBUG_STATEMENT("Method called!")

    if (isRunning)
    {
        stopThread();
    }

    if (activeModel != nullptr && parameterBindingManager != nullptr)
    {
        parameterBindingManager->unbindParameters(activeModel->constraints());
        delete activeModel;
    }

    if (inactiveModel != nullptr && parameterBindingManager != nullptr)
    {
        parameterBindingManager->unbindParameters(inactiveModel->constraints());
        delete inactiveModel;
    }
}

bool RTControlModel::init(ros::NodeHandle & nh, RobotState * latestRobotState,
    BindingManager * parameterBindingManager,
    controlit::utility::ControlItParameters * params)
{
    PRINT_DEBUG_STATEMENT("Method called!")

    if (isRunning)
    {
        CONTROLIT_ERROR << "Attempted to initialize a RTControlModel that is already running!";
        return false;
    }

    this->parameterBindingManager = parameterBindingManager;

    PRINT_DEBUG_STATEMENT("Creating the active control model.")

    activeModel = ControlModel::createModel(nh, latestRobotState, params);
    if (activeModel == NULL) return false;

    PRINT_DEBUG_STATEMENT("Creating the inactive control model.")

    inactiveModel = ControlModel::createModel(nh, latestRobotState, params);
    if (inactiveModel == NULL) return false;

    activeModel->setName(std::string("ControlModel1"));
    inactiveModel->setName(std::string("ControlModel2"));

    try
    {
        parameterBindingManager->bindParameters(nh, activeModel->constraints());
    }
    catch (std::invalid_argument const& err)
    {
        CONTROLIT_ERROR << "Failed to bind constraint set parameters of active "
            "control model. Reason: " << err.what();
        return false;
    }

    try
    {
        parameterBindingManager->bindParameters(nh, inactiveModel->constraints());
    }
    catch (std::invalid_argument const& err)
    {
        CONTROLIT_ERROR_RT << "Failed to bind constraint set parameters of inactive "
            "control model. Reason: " << err.what();
        return false;
    }

    // For publishing the new gravity vector when the active model is updated
    gravityPublisher = nh.advertise<std_msgs::Float64MultiArray>("diagnostics/gravityVector", 10);

    // controlModelUpdateStat.reset();

    // Create the service handlers
    constraintJacobianService = nh.advertiseService("diagnostics/getConstraintJacobianMatrices",
        &RTControlModel::getConstraintJacobiansHandler, this);


    // modelUpdateLatencyPublisher.reset(
        // new controlit::addons::ros::RealtimePublisher<std_msgs::Float64>(nh, "diagnostics/modelUpdateLatency", 1));
    if(modelUpdateLatencyPublisher.trylock())
    {
        modelUpdateLatencyPublisher.msg_.data = 0;
        modelUpdateLatencyPublisher.unlockAndPublish();
    }
    else
    {
        CONTROLIT_ERROR_RT << "Unable to initialize modelUpdateLatencyPublisher!";
        return false;
    }

    initialized = true;
    return true;
}

void RTControlModel::setStale()
{
    activeModel->setStale();
    inactiveModel->setStale();
}

void RTControlModel::setGravityVector(const Vector & gravityVector)
{
    activeModel->rbdlModel().gravity.set(gravityVector(0), gravityVector(1), gravityVector(2));
    inactiveModel->rbdlModel().gravity.set(gravityVector(0), gravityVector(1), gravityVector(2));
}

void RTControlModel::setAMask(const std::vector<std::vector<std::string>> & mask)
{
  activeModel->setAMask(mask);
  inactiveModel->setAMask(mask);
}

void RTControlModel::setGravMask(const std::vector<std::string> & mask)
{
  activeModel->setGravMask(mask);
  inactiveModel->setGravMask(mask);
}

void RTControlModel::addListenerToConstraintSet(boost::function<void(std::string const&)> listener)
{
    activeModel->constraints().addListener(listener);
    inactiveModel->constraints().addListener(listener);
}

void RTControlModel::startThread()
{
    PRINT_DEBUG_STATEMENT("Method Called\n"
        " - std::this_thread::get_id = " << std::this_thread::get_id() << "\n"
        " - CPU: " << sched_getcpu())

    assert(initialized);

    keepRunning = true;
    thread = std::thread(&RTControlModel::updateLoop, this);

    // // Set the affinity of the main thread to cpu 1
    // cpu_set_t cpuset;
    // CPU_ZERO(&cpuset);
    // // CPU_SET(1, &cpuset); // cpu 1
    // CPU_SET(0, &cpuset); // cpu 0
    // const int err = pthreadsetaffinity_np(thread.native_handle(), sizeof(cpuset), &cpuset);
    // if (err != 0)
    // {
    //   CONTROLIT_ERROR_RT << "Failed to set thread affinity, error = " << err;
    // }

    // // Set the priority level of the thread (currently doesn't work)
    // sched_param sch;
    // int policy;
    // pthreadgetschedparam(thread.native_handle(), &policy, &sch);
    // sch.sched_priority = 1;
    // if(pthreadsetschedparam(thread.native_handle(), SCHED_FIFO, &sch))
    // {
    //     CONTROLIT_ERROR_RT << "Failed to set thread priority, error = " << std::strerror(errno);
    // }
    // else
    // {
    //   CONTROLIT_INFO_RT << "Successfully set thread priority level to be 1.";
    // }

    PRINT_DEBUG_STATEMENT("Done.")
}

void RTControlModel::stopThread()
{
    PRINT_DEBUG_STATEMENT("Method Called")
    assert(initialized);

    if (!isRunning)
    {
        PRINT_DEBUG_STATEMENT("Not running!")

        return;
    }

    PRINT_DEBUG_STATEMENT("Trying to get lock...")

    mutex.lock();

    // Note: This while loop should be able to be replaced with "mutex.lock()"
    // while (!mutex.try_lock())
    // {
    //     #define MUTEX_WAIT_DURATION_MS 100

    //     PRINT_DEBUG_STATEMENT("Could not get lock on mutex, waiting for "
    //         << MUTEX_WAIT_DURATION_MS << " ms.")

    //     std::this_thread::sleep_for(
    //         std::chrono::milliseconds(MUTEX_WAIT_DURATION_MS));
    // }

    PRINT_DEBUG_STATEMENT("Got the lock on mutex!")

    PRINT_DEBUG_STATEMENT("Setting keepRunning = false")

    keepRunning = false;

    PRINT_DEBUG_STATEMENT("Calling cv.notify_one()")

    cv.notify_one();  // So the model update thread can exit

    PRINT_DEBUG_STATEMENT("Unlocking the mutex.")

    mutex.unlock();

    PRINT_DEBUG_STATEMENT("Calling thread.join()")

    thread.join();

    PRINT_DEBUG_STATEMENT("Done.")
}

void RTControlModel::getSwapLock()
{
    assert(initialized);
    swapMutex.lock();
}

void RTControlModel::releaseSwapLock()
{
    assert(initialized);
    swapMutex.unlock();
}

ControlModel * RTControlModel::trylock()
{
    assert(initialized);

    PRINT_DEBUG_STATEMENT_RT("Method Called")

    if (mutex.try_lock())
    {
        PRINT_DEBUG_STATEMENT_RT("Got lock.")

        switch(state)
        {
            case State::IDLE:
                PRINT_DEBUG_STATEMENT_RT("State is IDLE, returning a pointer to the inactive model")

                return inactiveModel;

            case State::UPDATING_MODEL:

                PRINT_DEBUG_STATEMENT_RT("State is UPDATING_MODEL, releasing lock and returning nullptr")

                mutex.unlock();
                return nullptr;

                // if (keepRunning)
                // {

                // }
                // else
                // {
                //     PRINT_DEBUG_STATEMENT("State is UPDATING_MODEL and keepRunning = false, returning a pointer to the inactive model")

                //     return inactiveModel;
                // }

            case State::UPDATED_MODEL_READY:

                PRINT_DEBUG_STATEMENT_RT("State is UPDATED_MODEL_READY, releasing lock and returning nullptr")

                mutex.unlock();
                return nullptr;

                // if (keepRunning)
                // {
                //     PRINT_DEBUG_STATEMENT("State is UPDATED_MODEL_READY and keepRunning = true, releasing lock and returning nullptr")

                //     mutex.unlock();
                //     return nullptr;
                // }
                // else
                // {
                //     PRINT_DEBUG_STATEMENT("State is UPDATED_MODEL_READY and keepRunning = false, returning a pointer to the inactive model")

                //     return inactiveModel;
                // }

            default:

                CONTROLIT_ERROR_RT << "Unknown state, releasing lock and returning nullptr";

                mutex.unlock();
                return nullptr;
        }
    }
    else
    {
        PRINT_DEBUG_STATEMENT_RT("Could not obtain lock (ModleUpdate thread must be busy), returning nullptr.")
        return nullptr;
    }
}

void RTControlModel::unlockAndUpdate()
{
    assert(initialized);

    // #define TIME_UNLOCK_AND_UPDATE 1

    PRINT_DEBUG_STATEMENT_RT("Method called, setting state to be UPDATING_MODEL")

    state = State::UPDATING_MODEL;

    PRINT_DEBUG_STATEMENT_RT("Calling notify_one() on the condition variable.")

    #ifdef TIME_UNLOCK_AND_UPDATE
    ros::Time startNotifyOne = ros::Time::now();
    #endif

    cv.notify_one();

    PRINT_DEBUG_STATEMENT_RT("Releasing lock.")

    #ifdef TIME_UNLOCK_AND_UPDATE
    ros::Time startMutexUnlock = ros::Time::now();
    #endif

    mutex.unlock();

    #ifdef TIME_UNLOCK_AND_UPDATE
    ros::Time endMutexUnlock = ros::Time::now();
    #endif

    #ifdef TIME_UNLOCK_AND_UPDATE
    PRINT_DEBUG_STATEMENT_RT_ALWAYS("UnlockAndUpdate Latency Results (ms):\n"
        " - notifyOne: " << (startMutexUnlock - startNotifyOne).toSec() * 1000 << "\n"
        " - unlock: " << (endMutexUnlock - startMutexUnlock).toSec() * 1000);
    #endif

    PRINT_DEBUG_STATEMENT_RT("Done.")
}

ControlModel * RTControlModel::get()
{
    assert(initialized);
    return activeModel;
}

bool RTControlModel::checkUpdate()
{
    PRINT_DEBUG_STATEMENT_RT("Method called, state = " << stateToString(state))
    assert(initialized);

    // This is executed by the RT thread.  If a new update is
    // available and the RT thread is able to obtain the lock,
    // swap the ControlModel.
    if (state == State::UPDATED_MODEL_READY)
    {
        PRINT_DEBUG_STATEMENT_RT("A new ControlModel is available.")

        if (mutex.try_lock())
        {
            if (swapMutex.try_lock())
            {
                PRINT_DEBUG_STATEMENT_RT("Lock obtained, updating the ControlModel.")

                swap();

                // Now that the model is updated, publish the new gravity vector
                const Vector & grav = get()->getGrav();
                std_msgs::Float64MultiArray gravityMsg;

                gravityMsg.layout.dim.resize(2);
                gravityMsg.layout.dim[0].stride = grav.size();
                gravityMsg.layout.dim[0].size = grav.size();
                gravityMsg.layout.dim[1].stride = 1;
                gravityMsg.layout.dim[1].size = 1;
                gravityMsg.data.resize(grav.size());

                for (int ii = 0; ii < grav.size(); ii++)
                {
                    gravityMsg.data[ii] = grav[ii];
                }

                gravityPublisher.publish(gravityMsg);


                PRINT_DEBUG_STATEMENT_RT("Done updating the ControlModel, setting ModelUpdate Thread's state to be IDLE")

                state = State::IDLE;

                PRINT_DEBUG_STATEMENT_RT("Releasing lock.")

                mutex.unlock();
                swapMutex.unlock();

                return true; // performed a swap
            }
            else
            {
                PRINT_DEBUG_STATEMENT_RT("Failed to obtain swap lock, sticking to current ControlModel.  "
                    "  Some other thread must be using the active control model.")
            }
        }
        else
        {
            PRINT_DEBUG_STATEMENT_RT("Failed to obtain lock, sticking to current ControlModel.  "
                " The ModelUpdate thread must still be busy.")
        }
    }
    else
    {
        PRINT_DEBUG_STATEMENT_RT("No new ControlModel is available, sticking to current one.")
    }

    return false; // did not perform a swap
}

void RTControlModel::updateLoop()
{
    PRINT_DEBUG_STATEMENT("Method Called\n"
        " - std::this_thread::get_id = " << std::this_thread::get_id())

    isRunning = true;
    state = State::IDLE;

    // Grab the lock on the mutex.  The ModelUpdate thread will always have the lock
    // unless it is waiting on the condition variable (cv).
    std::unique_lock<std::mutex> lk(mutex);

    while (keepRunning)
    {

        PRINT_DEBUG_STATEMENT("Waiting on condition variable...")

        /*
         * Wait on the mutex (this thread will temporarily release the lock on the mutex
         * while waiting.  This enables another thread to lock the mutex
         * and call notify_one() or notify_all() on it, which will wake up this thread.
         * After waking up but before moving on from this method call, two conditions
         * must be met:
         *
         *  1. The boolean expression within the lambda function must evaluate to true
         *  2. This thread must re-obtain the lock on the mutex
         */
        cv.wait(lk, [this] { return (state == State::UPDATING_MODEL) || (not keepRunning); });
        // cv.wait(lk, [&state, &keepRunning] { return (state == State::UPDATING_MODEL) || (not keepRunning); });

        // CONTROLIT_INFO_RT << "CPU " << sched_getcpu();

        // #define TIME_RT_CONTROL_MODEL_UPDATE 1

        // #ifdef TIME_RT_CONTROL_MODEL_UPDATE
        // ros::Time startSwap = ros::Time::now();
        // #endif

        // CONTROLIT_INFO_RT << "Done waiting on condition variable.\n"
        //                " - turn_ = " << (turn_ == Turn::UPDATE_LOOP ? "UPDATE_LOOP" : "REALTIME") << "\n"
        //                " - keepRunning = " << keepRunning << "\n"
        //                " - lk.owns_lock() = " << lk.owns_lock() << "\n"
        //                " - lk.mutex() = " << lk.mutex();

        // Swap model.. active model becomes the inactive model and vice versa

        // CONTROLIT_INFO_RT << "Swapping the model.";
        // swap();

        // #ifdef TIME_RT_CONTROL_MODEL_UPDATE
        // ros::Time doneSwap = ros::Time::now();
        // #endif

        // We have everything we need to proceed with an update
        if (keepRunning)
        {
            PRINT_DEBUG_STATEMENT("Updating the inactive model!")

            // controlModelUpdateStat.startTimer();

            // inactiveModel->checkForSysIdUpdates();

            modelUpdateStartTime = high_resolution_clock::now();

            // Do the big model update!
            inactiveModel->update();

            modelUpdateEndTime = high_resolution_clock::now();

            // controlModelUpdateStat.stopTimer();


            PRINT_DEBUG_STATEMENT("Done updating the inactive model. Setting the state to be UPDATED_MODEL_READY!")

            state = State::UPDATED_MODEL_READY;

            if (modelUpdateLatencyPublisher.trylock())
            {
                std::chrono::nanoseconds timeSpan 
                    = duration_cast<std::chrono::nanoseconds>(modelUpdateEndTime - modelUpdateStartTime);
                double period = timeSpan.count() / 1e9;
                modelUpdateLatencyPublisher.msg_.data = period;
                modelUpdateLatencyPublisher.unlockAndPublish();   
            }
        }
    }

    PRINT_DEBUG_STATEMENT("Stopping RTControlModel child thread.  "
        "Number of updates: " << numUpdates);

    // numUpdates = 0;
    isRunning = false;
}

void RTControlModel::swap()
{
    PRINT_DEBUG_STATEMENT_RT("Swapping the active and inactive control models.")

    std::swap(activeModel, inactiveModel);

    PRINT_DEBUG_STATEMENT_RT("Done Swapping the active and inactive control models.")
}

std::string RTControlModel::stateToString(State state) const
{
    switch(state)
    {
        case State::IDLE: return "IDLE";
        case State::UPDATING_MODEL: return "UPDATING_MODEL";
        case State::UPDATED_MODEL_READY: return "UPDATED_MODEL_READY";
        default: return "UNKNOWN";
    }
}

bool RTControlModel::getConstraintJacobiansHandler(
    controlit_core::get_constraint_set_jacobians::Request  &req,
    controlit_core::get_constraint_set_jacobians::Response &res)
{
    // Clear out the old data
    res.constraints.clear();

    // Obtain lock to prevent the active control model from being swapped by RT thread
    getSwapLock();

    ControlModel * controlModel = this->get();
    ConstraintSet & cs =  controlModel->constraints();

    // Get the constraint set's jacobian matrix.
    // This is an aggregation of every constraint's jacobian matrix.
    const Matrix & csJc = cs.getJacobian();

    int startingRow = 0;

    // cs.getConstraintSet() returns a ConstraintList_t, which is a std::vector< std::shared_ptr<Constraint> >
    //controlit::ConstraintList_t csElem = cs.getConstraintSet();
    for (std::shared_ptr<Constraint> constraint : cs.getConstraintSet())
    {
        unsigned int numJcRows = constraint->getNConstrainedDOFs();

        hc.header.stamp = controlModel->getTimeStamp();
        hc.header.frame_id = constraint->getMasterNodeName();

        const Vector * contactPoint = nullptr;
        if (constraint->hasParameter("contactPoint"))
            contactPoint = constraint->lookupParameter("contactPoint")->getVector();
        else if (constraint->hasParameter("contactPlanePoint"))
            contactPoint = constraint->lookupParameter("contactPlanePoint")->getVector();
        else
        {
            CONTROLIT_WARN_RT << "Unable to get contact point for constraint " << constraint->getInstanceName();
        }

        if (contactPoint != nullptr)
        {
            hc.point.x = (*contactPoint)[0];
            hc.point.y = (*contactPoint)[1];
            hc.point.z = (*contactPoint)[2];
        }

        // Get the constraint's jacobian matrix
        const Eigen::Block<const Matrix> Jc = csJc.block(startingRow, 0, numJcRows, csJc.cols());

        hc.jacobian.layout.data_offset = 0;
        hc.jacobian.layout.dim.resize(1);
        hc.jacobian.layout.dim[0].label = constraint->getInstanceName();
        hc.jacobian.layout.dim[0].size = Jc.cols();
        hc.jacobian.layout.dim[0].stride = Jc.cols() * Jc.rows();

        hc.jacobian.data.clear();
        for (int ii = 0; ii < Jc.rows(); ii++)
        {
            for (int jj = 0; jj < Jc.cols(); jj++)
            {
                hc.jacobian.data.push_back(Jc(ii, jj));
            }
        }

        startingRow += numJcRows;

        res.constraints.push_back(hc);
    }

    // Release lock to allow active control model to be swapped by RT thread
    releaseSwapLock();

    return true;
}

} // namespace controlit
