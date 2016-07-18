/**
Software License Agreement (BSD)
\file      runtime_platform.h
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

#ifndef RUNTIME_PLATFORM_H_
#define RUNTIME_PLATFORM_H_

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

#include <boost/shared_ptr.hpp> 

#include "ros/ros.h"

#ifdef ROS
#include "micros_swarm_framework/MSFPPacket.h"
#endif

#ifdef OPENSPLICE_DDS
#include "opensplice_dds/MSFPPacket.h"
#include "opensplice_dds/check_status.h"
#include "opensplice_dds/publisher.h"
#include "opensplice_dds/subscriber.h"
#endif

#include "micros_swarm_framework/data_type.h"

namespace micros_swarm_framework{

    class  RuntimePlatform{
        private:
            int robot_id_;
            int robot_type_;
            int robot_status_;
            Base robot_base_;
            std::map<int, NeighborBase> neighbors_;
            std::map<int, bool> swarms_;
            std::map<int, NeighborSwarmTuple> neighbor_swarms_;
            std::map<int, std::map<std::string, VirtualStigmergyTuple> > virtual_stigmergy_;
            double neighbor_distance_;
            std::set<int> barrier_;
            std::map<std::string, NeighborCommunicationCallBack> callback_functions_;
        public:
            RuntimePlatform();
            RuntimePlatform(int robot_id);
            
            //static RuntimePlatform& Instance();
            
            int getRobotID();
            void setRobotID(int robot_id);
            
            Base getRobotBase();
            void setRobotBase(Base robot_base);
            void printRobotBase();
        
            std::map<int, NeighborBase> getNeighbors();
            void insertOrUpdateNeighbor(int robot_id, float distance, float azimuth, float elevation, float x, float y, float z, float vx, float vy, float vz);
            //delete an neighbor robot according to id
            void deleteNeighbor(int robot_id);
            void printNeighbor();
            
            void insertOrUpdateSwarm(int swarm_id, bool value);
            //check if the local robot is in a swarm 
            bool getSwarm(int swarm_id);
            //get the swarm list of the local robot
            std::vector<int> getSwarmList();
            void deleteSwarm(int swarm_id);
            void printSwarm();
            
            //bool inNeighborSwarm(int robot_id, int swarm_id);
            void joinNeighborSwarm(int robot_id, int swarm_id);
            void leaveNeighborSwarm(int robot_id, int swarm_id);
            void insertOrRefreshNeighborSwarm(int robot_id, std::vector<int> swarm_list);
            //get the member robot set of a swarm 
            std::set<int> getSwarmMembers(int swarm_id);
            void deleteNeighborSwarm(int robot_id);
            void printNeighborSwarm();
            
            void createVirtualStigmergy(int id);
            void insertOrUpdateVirtualStigmergy(int id, std::string key, std::string value, time_t time_now, int robot_id);
            VirtualStigmergyTuple getVirtualStigmergyTuple(int id, std::string key);
            int getVirtualStigmergySize(int id);
            void deleteVirtualStigmergy(int id);
            void deleteVirtualStigmergyValue(int id, std::string key);
            void printVirtualStigmergy();
            
            double getNeighborDistance();
            void setNeighborDistance(double neighbor_distance);
            
            void insertBarrier(int robot_id);
            int getBarrierSize();
            
            void insertOrUpdateCallbackFunctions(std::string key, NeighborCommunicationCallBack cb);
            void doNothing();
            NeighborCommunicationCallBack getCallbackFunctions(std::string key);
            void deleteCallbackFunctions(std::string key);
    };
};

#endif
