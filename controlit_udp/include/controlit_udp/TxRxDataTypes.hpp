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

#ifndef __TX_RX_UDP_TYPES_HPP__
#define __TX_RX_UDP_TYPES_HPP__

#define DEFAULT_CMD_PORT                51124
#define DEFAULT_STATE_PORT              51125
#define DEFAULT_INET_ADDR               "127.0.0.1"
#define DEFAULT_NUM_DOFS                3

namespace controlit_udp {

// This contains the current state of the robot and is transmitted
// by UDP_Adapter.
// TODO : is there a better way to deal with num_dofs?
typedef struct
{
    uint32_t    num_dofs;
    double      position[DEFAULT_NUM_DOFS];
    double      velocity[DEFAULT_NUM_DOFS];
    double      effort[DEFAULT_NUM_DOFS];
} State;

// TODO: Check how seqno is being used.
typedef struct
{
    State      state;
    int32_t    seqno;
} StateMsg;

// This message is received by UDP_Adapter.  It contains the command
// from the controller.
typedef struct
{
    uint32_t    num_dofs;
    double      effort[DEFAULT_NUM_DOFS];
    double      position[DEFAULT_NUM_DOFS];
    double      velocity[DEFAULT_NUM_DOFS];
} Command;

typedef struct
{
    Command    command;
    int32_t    seqno;
} CommandMsg;

} //namespace controlit_udp

#endif //__TX_RX_UDP_TYPES_HPP__