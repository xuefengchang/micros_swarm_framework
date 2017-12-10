/**
Software License Agreement (BSD)
\file      packet_type.h
\authors Xuefeng Chang <changxuefengcn@163.com>
\copyright Copyright (c) 2016, the micROS Team, HPCL (National University of Defense Technology), All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of micROS Team, HPCL, nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef PACKET_TYPE_H_
#define PACKET_TYPE_H_

#include <iostream>

namespace micros_swarm{

    /*
    *GSDFPacket type
    */
    enum GSDFPacketType {
        SINGLE_ROBOT_BROADCAST_BASE,  //broadcast id
        
        SINGLE_ROBOT_JOIN_SWARM,  //robot join in a swarm
        SINGLE_ROBOT_LEAVE_SWARM,  //robot leave a swarm
        SINGLE_ROBOT_SWARM_LIST,  //broadcast swarm list
        
        VIRTUAL_STIGMERGY_QUERY,  //query a value of virtual stigmergy
        VIRTUAL_STIGMERGY_PUT,  //put a value in virtual stigmergy
        VIRTUAL_STIGMERGY_PUTS,  //put a value in virtual stigmergy in cycles

        BLACKBOARD_QUERY,  //query a value of blackboard
        BLACKBOARD_QUERY_ACK,  //query ack
        BLACKBOARD_PUT,  //put a value into blackboard

        SCDS_PSO_PUT,
        SCDS_PSO_GET,
        
        NEIGHBOR_BROADCAST_KEY_VALUE,  //broadcast <key, value> tuple
        
        BARRIER_SYN,  //userd for barrier, syn
        BARRIER_ACK,  //used for barrier, ack
        
        GSDF_PACKET_TYPE_COUNT  //GSDF Packet type count
    };
};
#endif
