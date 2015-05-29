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

#ifndef __TX_RX_UDP_HPP__
#define __TX_RX_UDP_HPP__

#include <mutex>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <controlit_udp/RxThread.hpp>
#include <controlit_udp/TxRxDataTypes.hpp>

namespace controlit_udp {

	class TxStateRxCommandUDP : public RxThread
	{
	public:

		TxStateRxCommandUDP();
		~TxStateRxCommandUDP();

		virtual bool init(StateMsg * txStateMsg,
						  CommandMsg * rxCommandMessage,
						  uint32_t statePort = DEFAULT_STATE_PORT,
						  uint32_t cmdPort = DEFAULT_CMD_PORT,
						  std::string address = DEFAULT_INET_ADDR);

		virtual void sendState();
		// haven't used this, but leaving it for now.
		virtual void sendState(StateMsg * sMsg);

	protected:
		// Code to be executed by thread goes here (i.e., revcfrom)
		virtual void RxThreadEnterLoop();

	private:

		sockaddr_in si_me;
		sockaddr_in si_other;
		sockaddr_in si_to;

		int s_send_state_socket;
		int s_recv_command_socket;
		uint32_t seqno;

		std::mutex stateMutex;
		std::mutex cmdMutex;

		//Crap...check allocation?
		CommandMsg * cmdMsg;
		StateMsg * stateMsg;
	};

	class TxCommandRxStateUDP : public RxThread
	{
	public:

		TxCommandRxStateUDP();
		virtual ~TxCommandRxStateUDP();

		virtual bool init(CommandMsg * txCommandMsg,
						  StateMsg * rxStateMsg,
						  uint32_t cmdPort = DEFAULT_CMD_PORT,
						  uint32_t statePort = DEFAULT_STATE_PORT,
						  std::string address = DEFAULT_INET_ADDR);

		virtual void sendCommand();
		// haven't used this, but leaving it for now.
		virtual void sendCommand(CommandMsg * cmdMsg);

	protected:
		// Code to be executed by thread goes here (i.e., revcfrom)
		virtual void RxThreadEnterLoop();

	private:

		sockaddr_in si_me;
		sockaddr_in si_other;
		sockaddr_in si_to;

		int s_send_command_socket;
		int s_recv_state_socket;

		std::mutex stateMutex;
		std::mutex cmdMutex;

		//Crap...check allocation?
		CommandMsg * cmdMsg;
		StateMsg * stateMsg;
	};

}  //namespace controlit_udp
#endif //__TX_RX_UDP_HPP__
