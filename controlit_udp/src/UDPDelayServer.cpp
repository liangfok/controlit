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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>  // for memset

#include <controlit_udp/UDPDelayServer.hpp>

using namespace std::chrono;

namespace controlit_udp{

    UDPDelayServer::UDPDelayServer():
        RxThread(), initialized(false),
        commandDelay(7), stateDelay(7), controllerLoopRate(1000),
        stateQSize(0), commandQSize(0) {}

    UDPDelayServer::~UDPDelayServer()
    {
        WaitForRxThreadExit();
    }

    // initializes threads to listend on send
    // and receive ports, starts this thread to
    // accumulate queues, uses default values for delays
    bool UDPDelayServer::init(uint32_t rxStPort, uint32_t rxCmdPort,
                    uint32_t txStPort, uint32_t txCmdPort,
                    std::string address)
    {
        // txCmdMsg.received_check = 0;
        // rxCmdMsg.received_check = 0;
        if(!initRxState(rxStPort, txCmdPort, address))
            return false;

        if(!initRxCommand(rxCmdPort, txStPort, address))
            return false;

        // resize lists, size = 600 * delay/1000
        stateQSize = (int) std::max( ceil(controllerLoopRate * stateDelay / 1000),1.0);
        commandQSize = (int) std::max( ceil(controllerLoopRate * commandDelay / 1000),1.0);

        initialized = StartRxThread();
        return initialized;
    }

    bool UDPDelayServer::init(uint32_t rxStPort, uint32_t rxCmdPort,
                    uint32_t txStPort, uint32_t txCmdPort,
                    std::string address,
                    double sDelay_ms, double cDelay_ms, double servo_hz)
    {
        stateDelay = sDelay_ms;
        commandDelay = cDelay_ms;
        controllerLoopRate = servo_hz;
        return init(rxStPort, rxCmdPort, txStPort, txCmdPort, address);
    }

    // delays specified in ms
    void UDPDelayServer::setDelay(double delay)
    {
        commandDelay = delay;
        stateDelay = delay;
        resizeQueues();
    }

    void UDPDelayServer::setDelay(double sDelay, double cDelay)
    {
        commandDelay = cDelay;
        stateDelay = sDelay;
        resizeQueues();
    }

    void UDPDelayServer::setCommandDelay(double cDelay)
    {
        commandDelay = cDelay;
        resizeQueues();
    }

    void UDPDelayServer::setStateDelay(double sDelay)
    {
        stateDelay = sDelay;
        resizeQueues();
    }

    void UDPDelayServer::setControllerLoopRate(double rate)
    {
        controllerLoopRate = rate;
        resizeQueues();
    }

    void UDPDelayServer::resizeQueues()
    {
        bool reinit = initialized;
        initialized = false;
        stateQSize = (int) std::max( ceil(controllerLoopRate * stateDelay / 1000),1.0);
        commandQSize = (int) std::max( ceil(controllerLoopRate * commandDelay / 1000),1.0);
        stateQueue.clear();
        commandQueue.clear();
        initialized = reinit;
    }

    bool UDPDelayServer::initRxState(uint32_t rxStPort,
                     uint32_t txCmdPort,
                     std::string address)
    {
        return rxStateUDP.init(&txCmdMsg, &rxStateMsg,
                        txCmdPort, rxStPort, address);
    }

    bool UDPDelayServer::initRxCommand(uint32_t rxCmdPort,
                       uint32_t txStPort,
                       std::string address)
    {
        return rxCommandUDP.init(&txStateMsg,&rxCmdMsg,
                                 txStPort, rxCmdPort, address);
    }

    // To be executed continuously
    void UDPDelayServer::RxThreadEnterLoop()
    {
        double elapsed;
#if USE_ROS_TIME
        t0 = ros::Time::now();
        t1 = ros::Time::now();
        ros::Duration T = t1 - t0;
        elapsed = T.toSec();
#else
        t0 = high_resolution_clock::now();
        t1 = high_resolution_clock::now();
        duration<double> T = duration_cast<duration<double>>(t1 - t0);
        elapsed = T.count();
#endif

        while(1)
        {
            if(initialized) // temporarily false if queue parameters are being changed.
            {
                if(elapsed >= 1.0 / controllerLoopRate)
                {
                    // fprintf(stderr, "elapsed = %f, state queue length = %lu, command queue length =  %lu\n",
                    //              elapsed, stateQueue.size(), commandQueue.size());
                    CommandMsg newCmdMsg = rxCmdMsg;
                    commandQueue.push_front(newCmdMsg);
                    txCmdMsg = commandQueue.back();
                    rxStateUDP.sendCommand();
                    if(commandQueue.size() > commandQSize && commandQueue.size()>1)
                        commandQueue.pop_back();

                    StateMsg newStateMsg = rxStateMsg;
                    stateQueue.push_front(newStateMsg);
                    txStateMsg = stateQueue.back();
                    rxCommandUDP.sendState();
                    if(stateQueue.size() > stateQSize && stateQueue.size()>1)
                        stateQueue.pop_back();

                    t0 = t1;
                }
#if USE_ROS_TIME
                t1 = ros::Time::now();
                T = t1 - t0;
                elapsed = T.toSec();
#else
                t1 = high_resolution_clock::now();
                T = duration_cast<duration<double>>(t1 - t0);
                elapsed = T.count();
#endif
            }
            else
            {
#if USE_ROS_TIME
                t0 = ros::Time::now();
                t1 = ros::Time::now();
#else
                t0 = high_resolution_clock::now();
                t1 = high_resolution_clock::now();
#endif
            }
        }

    }
}