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

#include "controlit_udp/TxRxUDP.hpp"

using namespace controlit_udp;
/////////////////////////////////////////////////
//Implementation of class TxStateRxCommandUDP////
/////////////////////////////////////////////////
TxStateRxCommandUDP::TxStateRxCommandUDP() :
    RxThread(),
    s_send_state_socket(0), s_recv_command_socket(0), seqno(0)
{
        memset((char *) &si_me, 0, sizeof(si_me));
        memset((char *) &si_other, 0, sizeof(si_other));
        memset((char *) &si_to, 0, sizeof(si_to));
}

TxStateRxCommandUDP::~TxStateRxCommandUDP()
{
    //Safer if this just stops the thread?
    WaitForRxThreadExit();
}

void TxStateRxCommandUDP::RxThreadEnterLoop()
{
    int slen = sizeof(si_other);
    while(1)
    {
        if(cmdMutex.try_lock())
        {
            recvfrom(s_recv_command_socket, cmdMsg, sizeof(CommandMsg), 0, (struct sockaddr *) & si_other,
            (socklen_t *) & slen);
            // cmdMsg->received_check = 0;
            cmdMutex.unlock();
        }
    }

}

bool TxStateRxCommandUDP::init(StateMsg * txStateMsg,
                          CommandMsg * rxCommandMsg,
                          uint32_t statePort,
                          uint32_t cmdPort,
                          std::string address)
{
    if((s_recv_command_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        fprintf(stderr, "Failed to initialize socket to recieve command \n");
        return false;
    }

    if((s_send_state_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        fprintf(stderr, "Failed to initialize socket to send state data \n");
        return false;
    }

    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(cmdPort);
    si_other.sin_addr.s_addr = inet_addr(address.c_str());

    si_to.sin_family = AF_INET;
    si_to.sin_port = htons(statePort);
    si_to.sin_addr.s_addr = inet_addr(address.c_str());

    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(cmdPort);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);
    bind(s_recv_command_socket, (struct sockaddr * ) & si_me, sizeof(si_me));

    this->stateMsg = txStateMsg;
    this->cmdMsg = rxCommandMsg;

    return StartRxThread();
}

//TODO: add size checks
void TxStateRxCommandUDP::sendState()
{
    int slen = sizeof(si_to);
    if(stateMutex.try_lock())
    {
        this->stateMsg->seqno = seqno++;
        sendto(s_send_state_socket, stateMsg, sizeof(StateMsg), 0, (struct sockaddr *) & si_to, slen);
        stateMutex.unlock();
    }
}

void TxStateRxCommandUDP::sendState(StateMsg * sMsg)
{
    int slen = sizeof(si_to);
    sMsg->seqno = seqno++;
    sendto(s_send_state_socket, stateMsg, sizeof(StateMsg), 0, (struct sockaddr *) & si_to, slen);
}

/////////////////////////////////////////////////
//Implementation of class TxCommandRxStateUDP////
/////////////////////////////////////////////////
TxCommandRxStateUDP::TxCommandRxStateUDP() :
    RxThread(),
    s_send_command_socket(0), s_recv_state_socket(0)
{
        memset((char *) &si_me, 0, sizeof(si_me));
        memset((char *) &si_other, 0, sizeof(si_other));
        memset((char *) &si_to, 0, sizeof(si_to));
}

TxCommandRxStateUDP::~TxCommandRxStateUDP()
{
    //Better to stop thread?
    WaitForRxThreadExit();
}

void TxCommandRxStateUDP::RxThreadEnterLoop()
{
    int slen = sizeof(si_other);
    while(1)
    {
        if(stateMutex.try_lock())
        {
            recvfrom(s_recv_state_socket, stateMsg, sizeof(StateMsg), 0, (struct sockaddr *) & si_other,
            (socklen_t *) & slen);
            stateMutex.unlock();
        }
    }

}

bool TxCommandRxStateUDP::init(CommandMsg * txCommandMsg,
                          StateMsg * rxStateMsg,
                          uint32_t cmdPort,
                          uint32_t statePort,
                          std::string address)
{
    if((s_recv_state_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        fprintf(stderr, "Failed to initialize socket to recieve command \n");
        return false;
    }

    if((s_send_command_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        fprintf(stderr, "Failed to initialize socket to send state data \n");
        return false;
    }

    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(statePort);
    si_other.sin_addr.s_addr = inet_addr(address.c_str());

    si_to.sin_family = AF_INET;
    si_to.sin_port = htons(cmdPort);
    si_to.sin_addr.s_addr = inet_addr(address.c_str());

    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(statePort);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);
    bind(s_recv_state_socket, (struct sockaddr * ) & si_me, sizeof(si_me));

    this->stateMsg = rxStateMsg;
    this->cmdMsg = txCommandMsg;

    return StartRxThread();
}

void TxCommandRxStateUDP::sendCommand()
{
    int slen = sizeof(si_to);
    if(cmdMutex.try_lock())
    {
        // cmdMsg->received_check++;
        sendto(s_send_command_socket, cmdMsg, sizeof(CommandMsg), 0, (struct sockaddr *) & si_to, slen);
        cmdMutex.unlock();
    }
}

void TxCommandRxStateUDP::sendCommand(CommandMsg * cMsg)
{
    // cmdMsg->received_check++;
    int slen = sizeof(si_to);
    sendto(s_send_command_socket, cMsg, sizeof(CommandMsg), 0, (struct sockaddr *) & si_to, slen);
}
