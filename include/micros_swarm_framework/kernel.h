/**
Software License Agreement (BSD)
\file      kernel.h
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

#ifndef KERNEL_H_
#define KERNEL_H_

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

#include <boost/interprocess/sync/named_mutex.hpp>

#include "ros/ros.h"

#include "micros_swarm_framework/data_type.h"
#include "micros_swarm_framework/message.h"
#include "micros_swarm_framework/MSFPPacket.h"

namespace micros_swarm_framework{

    class  KernelInitializer{
        private:
            //subscriber of the packet in the kernel
            ros::Subscriber packet_subscriber_;
            
            //parser for the MSFPPacket
            void PacketParser(const micros_swarm_framework::MSFPPacket& msfp_packet);
            void packetCallback(const micros_swarm_framework::MSFPPacket& packet)
            {
                PacketParser(packet);
            }
        public:
            static int unique_robot_id_;
            static void initRobotID(int robot_id);
            
            KernelInitializer();
            ~KernelInitializer();
    };
    
    class KernelHandle{
        public:
            static unsigned int getRobotID();
            static void setRobotID(unsigned int robot_id);
            
            static double getNeighborDistance();
            static void setNeighborDistance(double neighbor_distance);
            
            static Base getRobotBase();
            static void setRobotBase(Base robot_base);
        
            static std::map<unsigned int, NeighborBase> getNeighbors();
            static void insertOrUpdateNeighbor(unsigned int robot_id, float distance, float azimuth, float elevation, float x, float y, float z, float vx, float vy, float vz);
            //delete an neighbor robot according to id
            static void deleteNeighbor(unsigned int robot_id);
            static void printNeighbor();
            
            static void insertOrUpdateSwarm(unsigned int swarm_id, bool value);
            //check if the local robot is in a swarm 
            static bool getSwarm(unsigned int swarm_id);
            //get the swarm list of the local robot
            static std::vector<unsigned int> getSwarmList();
            static void deleteSwarm(unsigned int swarm_id);
            static void printSwarm();
            
            static void joinNeighborSwarm(unsigned int robot_id, unsigned int swarm_id);
            static void leaveNeighborSwarm(unsigned int robot_id, unsigned int swarm_id);
            static void insertOrRefreshNeighborSwarm(unsigned int robot_id, std::vector<unsigned int> swarm_list);
            //get the member robot set of a swarm 
            static std::set<unsigned int> getSwarmMembers(unsigned int swarm_id);
            static void deleteNeighborSwarm(unsigned int robot_id);
            static void printNeighborSwarm();
            
            static void createVirtualStigmergy(unsigned int id);
            static void insertOrUpdateVirtualStigmergy(unsigned int id, std::string key_std, std::string value_std, time_t time_now, unsigned int robot_id);
            static VstigTuple getVirtualStigmergyTuple(unsigned int id, std::string key_std);
            static unsigned int getVirtualStigmergySize(unsigned int id);
            static void deleteVirtualStigmergy(unsigned int id);
            static void deleteVirtualStigmergyValue(unsigned int id, std::string key_std);
            static void printVirtualStigmergy();
            
            //publish a packet
            static void publishPacket(micros_swarm_framework::MSFPPacket msfp_packet);
    };
    
};

#endif
