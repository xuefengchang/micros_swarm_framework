/**
Software License Agreement (BSD)
\file      message.h
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

#ifndef MESSAGE_H_
#define MESSAGE_H_

#include <iostream>
#include <string>
#include <time.h>
#include <stdlib.h>
#include <vector>
#include <stack>
#include <map>
#include <set>
#include <queue>
#include <algorithm>

#include <fstream>
#include <sstream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/string.hpp> 
#include <boost/serialization/vector.hpp>

#include "ros/ros.h"

#include "micros_swarm_framework/data_type.h"

namespace micros_swarm_framework{

    /*
    *MSFPPacket type
    */
    enum MSFPPacketType
    {
        SINGLE_ROBOT_BROADCAST_BASE,  //broadcast id
        
        SINGLE_ROBOT_JOIN_SWARM,  //robot join in a swarm
        SINGLE_ROBOT_LEAVE_SWARM,  //robot leave a swarm
        SINGLE_ROBOT_SWARM_LIST,  //broadcast swarm list
        
        VIRTUAL_STIGMERGY_QUERY,  //query a value of virtual stigmergy
        VIRTUAL_STIGMERGY_PUT,  //put a value in virtual stigmergy
        
        NEIGHBOR_BROADCAST_KEY_VALUE,  //broadcast <key, value> tuple
        
        BARRIER_SYN,  //userd for barrier, syn
        BARRIER_ACK,  //used for barrier, ack
        
        MSFP_PACKET_TYPE_COUNT  //MSFPPacket type count
    };

    struct SingleRobotBroadcastBase{
        int robot_id;
            
        float robot_x;
        float robot_y;
        float robot_z;
            
        float robot_vx;
        float robot_vy;
        float robot_vz;
        
        int valid;
            
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & robot_id;
            ar & robot_x;
            ar & robot_y;
            ar & robot_z;
            ar & robot_vx;
            ar & robot_vy;
            ar & robot_vz;
            ar & valid;
        }
            
        SingleRobotBroadcastBase():
            robot_id(-1), robot_x(0), robot_y(0), robot_z(0),  robot_vx(0), robot_vy(0), robot_vz(0), valid(-1){}
        SingleRobotBroadcastBase(int robot_id_, float robot_x_ ,float robot_y_, float robot_z_, float robot_vx_, float robot_vy_, float robot_vz_, int valid_):
            robot_id(robot_id_), robot_x(robot_x_), robot_y(robot_y_), robot_z(robot_z_),  robot_vx(robot_vx_), robot_vy(robot_vy_), robot_vz(robot_vz_), valid(valid_){}
    };
    
    struct SingleRobotJoinSwarm{
        int robot_id;
        int swarm_id;
            
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & robot_id;
            ar & swarm_id;
        }
            
        SingleRobotJoinSwarm(){}
        SingleRobotJoinSwarm(int robot_id_, int swarm_id_):
            robot_id(robot_id_), swarm_id(swarm_id_){}
    };
    
    struct SingleRobotLeaveSwarm{
        int robot_id;
        int swarm_id;
            
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & robot_id;
            ar & swarm_id;
        }
            
        SingleRobotLeaveSwarm(){}
        SingleRobotLeaveSwarm(int robot_id_, int swarm_id_):
            robot_id(robot_id_), swarm_id(swarm_id_){}
    };
    
    struct SingleRobotSwarmList{
        int robot_id;
        std::vector<int> swarm_list;
            
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & robot_id;
            ar & swarm_list;
        }
            
        SingleRobotSwarmList(){}
        SingleRobotSwarmList(int robot_id_, const std::vector<int>& swarm_list_):
            robot_id(robot_id_), swarm_list(swarm_list_){}
    };
    
    struct VirtualStigmergyQuery
    {
        int virtual_stigmergy_id;
        std::string virtual_stigmergy_key;
        std::string virtual_stigmergy_value;
        time_t virtual_stigmergy_timestamp;
        int robot_id;
            
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & virtual_stigmergy_id;
            ar & virtual_stigmergy_key;
            ar & virtual_stigmergy_value;
            ar & virtual_stigmergy_timestamp;
            ar & robot_id;
        }
            
        VirtualStigmergyQuery(){}

        VirtualStigmergyQuery(int id_, const std::string& key_, const std::string& value_, time_t time_now_, int robot_id_):
            virtual_stigmergy_id(id_), virtual_stigmergy_key(key_), virtual_stigmergy_value(value_),
            virtual_stigmergy_timestamp(time_now_), robot_id(robot_id_){}
    };

    struct VirtualStigmergyPut
    {
        int virtual_stigmergy_id;
        std::string virtual_stigmergy_key;
        std::string virtual_stigmergy_value;
        time_t virtual_stigmergy_timestamp;
        int robot_id;
            
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & virtual_stigmergy_id;
            ar & virtual_stigmergy_key;
            ar & virtual_stigmergy_value;
            ar & virtual_stigmergy_timestamp;
            ar & robot_id;
        }
            
        VirtualStigmergyPut(){};
            
        VirtualStigmergyPut(int id_, const std::string& key_, const std::string& value_, time_t time_now_, int robot_id_):
            virtual_stigmergy_id(id_), virtual_stigmergy_key(key_), virtual_stigmergy_value(value_),
            virtual_stigmergy_timestamp(time_now_), robot_id(robot_id_){}
    };
    
    struct NeighborBroadcastKeyValue{
        std::string key;
        std::string value;
            
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & key;
            ar & value;
        }
            
        NeighborBroadcastKeyValue(){}
        NeighborBroadcastKeyValue(const std::string& key_, const std::string& value_): key(key_), value(value_){}
    };
    
    struct Barrier_Syn
    {
        std::string s;
            
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & s;
        }
            
        Barrier_Syn(){};
        Barrier_Syn(const std::string& s_):s(s_){}
    };
    
    struct Barrier_Ack
    {
        int robot_id;
            
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & robot_id;
        }
       
        Barrier_Ack(){};
        Barrier_Ack(unsigned int robot_id_):robot_id(robot_id_){}
    };
};
#endif
