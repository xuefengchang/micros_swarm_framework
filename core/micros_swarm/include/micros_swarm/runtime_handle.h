/**
Software License Agreement (BSD)
\file      runtime_handle.h
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

#ifndef RUNTIME_HANDLE_H_
#define RUNTIME_HANDLE_H_

#include <iostream>
#include <vector>
#include <map>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>

#include "micros_swarm/data_type.h"
#include "micros_swarm/listener_helper.h"
#include "micros_swarm/check_neighbor.h"

namespace micros_swarm{

    class  RuntimeHandle{
        public:
            RuntimeHandle();
            
            int getRobotID();
            void setRobotID(int robot_id);
            
            int getRobotType();
            void setRobotType(int robot_type);
            
            int getRobotStatus();
            void setRobotStatus(int robot_status);
            
            const Base& getRobotBase();
            void setRobotBase(const Base& robot_base);
            void printRobotBase();
            
            void getNeighbors(std::map<int, NeighborBase>& neighbors);
            std::map<int, NeighborBase> getNeighbors();
            int getNeighborSize();
            bool getNeighborBase(int robot_id, NeighborBase& nb);
            void clearNeighbors();
            void insertOrUpdateNeighbor(int robot_id, float distance, float azimuth, float elevation, float x, float y, float z, float vx, float vy, float vz);
            //delete an neighbor robot according to id
            void deleteNeighbor(int robot_id);
            bool inNeighbors(int robot_id);
            void printNeighbor();
            
            void insertOrUpdateSwarm(int swarm_id, bool value);
            //check if the local robot is in a swarm 
            bool getSwarmFlag(int swarm_id);
            //get the swarm list of the local robot
            void getSwarmList(std::vector<int>& swarm_list);
            void deleteSwarm(int swarm_id);
            void printSwarm();
            
            //check if a robot is in a swarm
            bool inNeighborSwarm(int robot_id, int swarm_id);
            void joinNeighborSwarm(int robot_id, int swarm_id);
            void leaveNeighborSwarm(int robot_id, int swarm_id);
            void insertOrRefreshNeighborSwarm(int robot_id, const std::vector<int>& swarm_list);
            //get the member robot set of a swarm 
            void getSwarmMembers(int swarm_id, std::set<int>& swarm_members);
            void deleteNeighborSwarm(int robot_id);
            void printNeighborSwarm();
            
            void createVirtualStigmergy(int id);
            void insertOrUpdateVirtualStigmergy(int id, const std::string& key, const std::vector<uint8_t>& value, \
                                                       unsigned int lclock, time_t wtime, unsigned int rcount, int robot_id);
            bool isVirtualStigmergyTupleExist(int id, const std::string& key);
            bool getVirtualStigmergyTuple(int id, const std::string& key, VirtualStigmergyTuple& vstig_tuple);
            void updateVirtualStigmergyTupleReadCount(int id, const std::string& key, int count);
            int getVirtualStigmergySize(int id);
            void deleteVirtualStigmergy(int id);
            void deleteVirtualStigmergyValue(int id, const std::string& key);
            void printVirtualStigmergy();
            bool checkNeighborsOverlap(int robot_id);

            void createBlackBoard(int id);
            void insertOrUpdateBlackBoard(int id, const std::string& key, const std::vector<uint8_t>& value, const ros::Time& timestamp, int robot_id);
            bool isBlackBoardTupleExist(int id, const std::string& key);
            void getBlackBoardTuple(int id, const std::string& key, BlackBoardTuple& bb_tuple);
            int getBlackBoardSize(int id);
            void deleteBlackBoard(int id);
            void deleteBlackBoardValue(int id, const std::string& key);
            void printBlackBoard();
            
            const float& getNeighborDistance();
            void setNeighborDistance(float neighbor_distance);
            
            void insertOrUpdateListenerHelper(const std::string& key, const boost::shared_ptr<ListenerHelper> helper);
            const boost::shared_ptr<ListenerHelper> getListenerHelper(const std::string& key);
            void deleteListenerHelper(const std::string& key);
            
            void insertBarrier(int robot_id);
            int getBarrierSize();

            bool getSCDSPSOValue(const std::string& aKey, SCDSPSODataTuple& aT);
            void insertOrUpdateSCDSPSOValue(const std::string& aKey, const SCDSPSODataTuple& aT);

        private:
            int robot_id_;
            boost::shared_mutex id_mutex_;
            int robot_type_;  //TODO
            boost::shared_mutex type_mutex_;
            int robot_status_;  //TODO
            boost::shared_mutex status_mutex_;
            Base robot_base_;
            boost::shared_mutex base_mutex_;
            std::map<int, NeighborBase> neighbors_;
            boost::shared_mutex neighbor_mutex_;
            std::map<int, bool> swarms_;
            boost::shared_mutex swarm_mutex_;
            std::map<int, NeighborSwarmTuple> neighbor_swarms_;
            boost::shared_mutex neighbor_swarm_mutex_;
            std::map<int, std::map<std::string, VirtualStigmergyTuple> > virtual_stigmergy_;
            boost::shared_mutex vstig_mutex_;
            std::map<int, std::map<std::string, BlackBoardTuple> > blackboard_;
            boost::shared_mutex bb_mutex_;
            float neighbor_distance_;
            boost::shared_mutex neighbor_distance_mutex_;
            std::map<std::string, boost::shared_ptr<ListenerHelper> > listener_helpers_;
            boost::shared_mutex listener_helpers_mutex_;
            std::set<int> barrier_;
            boost::shared_mutex barrier_mutex_;
            std::map<std::string, SCDSPSODataTuple> scds_pso_tuple_;
            boost::shared_mutex scds_pso_tuple_mutex_;
            boost::shared_ptr<CheckNeighborInterface> cni_;
    };
};

#endif
