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

#ifndef _UDP_DELAY_HPP__
#define _UDP_DELAY_HPP__

#define USE_ROS_TIME 1

// ROS includes
#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>

// controlit includes (UDP)
#include <controlit_udp/TxRxUDP.hpp>

// system c++ includes
#include <math.h>
#include <list>
#include <mutex>
#include <chrono>

using namespace std::chrono;

namespace controlit_udp{

	class UDPDelayServer : public RxThread {

		public:

			UDPDelayServer();
			~UDPDelayServer();

			// initializes threads to listend on send
			// and receive ports, starts this thread to
			// accumulate queues, uses default values for delays
			bool init(uint32_t rxStPort, uint32_t rxCmdPort,
                          	uint32_t txStPort, uint32_t txCmdPort,
                          	std::string address);

			// initializes threads to listend on send
			// and receive ports, starts this thread to
			// accumulate queues.
			bool init(uint32_t rxStPort, uint32_t rxCmdPort,
                          	uint32_t txStPort, uint32_t txCmdPort,
                          	std::string address,
                          	double sDelay_ms, double cDelay_ms, double servo_hz);

			// delays specified in ms
			void setDelay(double delay);
			void setDelay(double sDelay, double cDelay);
			void setCommandDelay(double cDelay);
			void setStateDelay(double sDelay);
			void setControllerLoopRate(double rate);

		protected:
			void resizeQueues();

			bool initRxState(uint32_t rxStPort,
                          	 uint32_t txCmdPort,
                          	std::string address);

			bool initRxCommand(uint32_t rxCmdPort,
                          	   uint32_t txStPort,
                          	   std::string address);

			// To be executed continuously
			virtual void RxThreadEnterLoop();

		private:

			CommandMsg rxCmdMsg;
			StateMsg txStateMsg;

			StateMsg rxStateMsg;
			CommandMsg txCmdMsg;

			std::list<CommandMsg> commandQueue;
			std::list<StateMsg> stateQueue;

			TxStateRxCommandUDP rxCommandUDP;
			TxCommandRxStateUDP rxStateUDP;

			// std::mutex stateMutex;
			// std::mutex cmdMutex;

			//don't send or receive until this is true
			bool initialized;

			//TODO: Ideally initialize these from parameters!
			double commandDelay; //ms
			double stateDelay;	//ms
			double controllerLoopRate; //Hz

			unsigned int stateQSize; //ceil(Hz * delay/1000)
			unsigned int commandQSize; //ceil(Hz * delay/1000)

#if USE_ROS_TIME
			ros::Time t0;
			ros::Time t1;
#else
			high_resolution_clock::time_point t0;
			high_resolution_clock::time_point t1;
#endif

	};
} // namepsace controlit_udp
#endif //_UDP_DELAY_HPP__
