/* 
 *  kernel.h - micros_swarm_framework kernel lib
 *  Copyright (C) 2016 Xuefeng Chang
 *  
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
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
            
            static Location getRobotLocation();
            static void setRobotLocation(Location robot_location);
        
            static std::map<unsigned int, NeighborLocation> getNeighbors();
            static void insertOrUpdateNeighbor(unsigned int robot_id, float distance, float azimuth, float elevation, float x, float y, float z);
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
