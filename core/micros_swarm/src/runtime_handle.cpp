/**
Software License Agreement (BSD)
\file      runtime_handle.cpp
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

#include "micros_swarm/runtime_handle.h"

namespace micros_swarm{

    RuntimeHandle::RuntimeHandle()
    {
        robot_id_ = 0;
        robot_base_ = Base(0,0,0,0,0,0,-1);
        neighbors_.clear();
        swarms_.clear();
        neighbor_swarms_.clear();
        virtual_stigmergy_.clear();
        blackboard_.clear();
        listener_helpers_.clear();
        listener_helpers_.insert(std::pair<std::string, boost::shared_ptr<ListenerHelper> >("" , NULL));
        barrier_.clear();
        scds_pso_tuple_.clear();
    }

    int RuntimeHandle::getRobotID()
    {
        boost::shared_lock<boost::shared_mutex> lock(id_mutex_);
        return robot_id_;
    }
    
    void RuntimeHandle::setRobotID(int robot_id)
    {
        boost::unique_lock<boost::shared_mutex> lock(id_mutex_);
        robot_id_ = robot_id;
    }
    
    int RuntimeHandle::getRobotType()
    {
        boost::shared_lock<boost::shared_mutex> lock(type_mutex_);
        return robot_type_;
    }
    
    void RuntimeHandle::setRobotType(int robot_type)
    {
        boost::unique_lock<boost::shared_mutex> lock(type_mutex_);
        robot_type_ = robot_type;
    }
    
    int RuntimeHandle::getRobotStatus()
    {
        boost::shared_lock<boost::shared_mutex> lock(status_mutex_);
        return robot_status_;
    }
    
    void RuntimeHandle::setRobotStatus(int robot_status)
    {
        boost::unique_lock<boost::shared_mutex> lock(status_mutex_);
        robot_status_ = robot_status;
    }
    
    const Base& RuntimeHandle::getRobotBase()
    {
        boost::shared_lock<boost::shared_mutex> lock(base_mutex_);
        return robot_base_;
    }
    
    void RuntimeHandle::setRobotBase(const Base& robot_base)
    {
        boost::unique_lock<boost::shared_mutex> lock(base_mutex_);
        robot_base_ = robot_base;
        if(robot_base.valid == -1) {
            robot_base_.valid = 1;
        }
    }    
    
    void RuntimeHandle::printRobotBase()
    {
        boost::shared_lock<boost::shared_mutex> lock(base_mutex_);
        std::cout<<"robot base: "<<robot_base_.x<<", "<<robot_base_.y<<", "<<\
            robot_base_.z<<", "<<robot_base_.vx<<", "<<robot_base_.vy<<", "<<\
            robot_base_.vz<<std::endl;
    }
        
    void RuntimeHandle::getNeighbors(std::map<int, NeighborBase>& neighbors)
    {
        boost::shared_lock<boost::shared_mutex> lock(neighbor_mutex_);
        neighbors = neighbors_;
    }

    bool RuntimeHandle::getNeighborBase(int robot_id, NeighborBase& nb)
    {
        boost::shared_lock<boost::shared_mutex> lock(neighbor_mutex_);
        std::map<int, NeighborBase>::iterator n_it = neighbors_.find(robot_id);

        if(n_it != neighbors_.end()) {
            nb = n_it->second;
            return true;
        }

        return false;
    }

    void RuntimeHandle::clearNeighbors()
    {
        boost::upgrade_lock<boost::shared_mutex> lock(neighbor_mutex_);
        if(neighbors_.size() > 0) {
            boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
            neighbors_.clear();
        }
    }

    std::map<int, NeighborBase> RuntimeHandle::getNeighbors()
    {
        boost::shared_lock<boost::shared_mutex> lock(neighbor_mutex_);
        return neighbors_;
    };

    int RuntimeHandle::getNeighborSize()
    {
        boost::shared_lock<boost::shared_mutex> lock(neighbor_mutex_);
        return neighbors_.size();
    }
     
    void RuntimeHandle::insertOrUpdateNeighbor(int robot_id, float distance, float azimuth, float elevation, float x, float y, float z, float vx, float vy, float vz)
    {
        boost::upgrade_lock<boost::shared_mutex> lock(neighbor_mutex_);
        std::map<int, NeighborBase>::iterator n_it = neighbors_.find(robot_id);
    
        if(n_it != neighbors_.end()) {
            NeighborBase new_neighbor_base(distance, azimuth, elevation, x, y, z,vx, vy, vz);
            boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
            n_it->second = new_neighbor_base;
        }
        else {
            NeighborBase new_neighbor_base(distance, azimuth, elevation, x, y, z, vx, vy, vz);
            boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
            neighbors_.insert(std::pair<int, NeighborBase>(robot_id ,new_neighbor_base));
        }
    }
    
    void RuntimeHandle::deleteNeighbor(int robot_id)
    {
        boost::unique_lock<boost::shared_mutex> lock(neighbor_mutex_);
        neighbors_.erase(robot_id);
    }
    
    bool RuntimeHandle::inNeighbors(int robot_id)
    {
        boost::shared_lock<boost::shared_mutex> lock(neighbor_mutex_);
        std::map<int, NeighborBase>::iterator n_it = neighbors_.find(robot_id);
    
        if(n_it != neighbors_.end()) {
            return true;
        }
        
        return false;
    }
    
    void RuntimeHandle::printNeighbor()
    {
        std::map<int, NeighborBase>::iterator n_it;
        boost::shared_lock<boost::shared_mutex> lock(neighbor_mutex_);
        for (n_it = neighbors_.begin(); n_it != neighbors_.end(); n_it++) {
            std::cout<<n_it->first<<": ";
            std::cout<<n_it->second.distance<<","<<n_it->second.azimuth<<","<<n_it->second.elevation<<","<<\
                n_it->second.x<<","<<n_it->second.y<<","<<n_it->second.z<<", "<<
                n_it->second.vx<<","<<n_it->second.vy<<","<<n_it->second.vz;
            std::cout<<std::endl;
        }
    }
     
    void RuntimeHandle::insertOrUpdateSwarm(int swarm_id, bool value)
    {
        boost::upgrade_lock<boost::shared_mutex> lock(swarm_mutex_);
        std::map<int, bool>::iterator s_it = swarms_.find(swarm_id);
    
        if(s_it != swarms_.end()) {
            boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
            s_it->second = value;
        }
        else {
            boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
            swarms_.insert(std::pair<int, bool>(swarm_id, value));
        }
    }
    
    bool RuntimeHandle::getSwarmFlag(int swarm_id)
    {
        boost::shared_lock<boost::shared_mutex> lock(swarm_mutex_);
        std::map<int, bool>::iterator s_it = swarms_.find(swarm_id);
    
        if(s_it != swarms_.end()) {
            return s_it->second;
        }
    
        return false;
    }
    
    void RuntimeHandle::getSwarmList(std::vector<int>& swarm_list)
    {
        swarm_list.clear();
        std::map<int, bool>::iterator s_it;
        boost::shared_lock<boost::shared_mutex> lock(swarm_mutex_);
        for(s_it = swarms_.begin(); s_it != swarms_.end(); s_it++) {
            if(s_it->second) {
                swarm_list.push_back(s_it->first);
            }
        }
    }
    
    void RuntimeHandle::deleteSwarm(int swarm_id)
    {
        boost::unique_lock<boost::shared_mutex> lock(swarm_mutex_);
        swarms_.erase(swarm_id);
    }
    
    void RuntimeHandle::printSwarm()
    {
        std::map<int, bool>::iterator s_it;
        boost::shared_lock<boost::shared_mutex> lock(swarm_mutex_);
        for(s_it = swarms_.begin(); s_it != swarms_.end(); s_it++) {
            std::cout<<s_it->first<<": ";
            std::cout<<s_it->second;
            std::cout<<std::endl;
        }
    }
    
    bool RuntimeHandle::inNeighborSwarm(int robot_id, int swarm_id)
    {
        std::map<int, NeighborSwarmTuple>::iterator os_it;
        boost::shared_lock<boost::shared_mutex> lock(neighbor_swarm_mutex_);
        os_it = neighbor_swarms_.find(robot_id);
        if(os_it != neighbor_swarms_.end()) {
            if(os_it->second.swarmIDExist(swarm_id)) {
               return true;
            }
            else {
                return false;
            }
        }
        else {
            return false;
        }
    }
    
    void RuntimeHandle::joinNeighborSwarm(int robot_id, int swarm_id)
    {
        std::map<int, NeighborSwarmTuple>::iterator os_it;
        boost::upgrade_lock<boost::shared_mutex> lock(neighbor_swarm_mutex_);
        os_it = neighbor_swarms_.find(robot_id);
    
        if(os_it != neighbor_swarms_.end()) {
            if(os_it->second.swarmIDExist(swarm_id)) {
                boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
                os_it->second.age = 0;
            }
            else {
                boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
                os_it->second.addSwarmID(swarm_id);
                os_it->second.age = 0;
            }
        }
        else {
            std::vector<int> swarm_list;
            swarm_list.push_back(swarm_id);
            NeighborSwarmTuple new_neighbor_swarm(swarm_list, 0);
            
            boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
            neighbor_swarms_.insert(std::pair<int, NeighborSwarmTuple>(robot_id, new_neighbor_swarm));
        }
    }
    
    void RuntimeHandle::leaveNeighborSwarm(int robot_id, int swarm_id)
    {
        std::map<int, NeighborSwarmTuple>::iterator os_it;
        boost::upgrade_lock<boost::shared_mutex> lock(neighbor_swarm_mutex_);
        os_it = neighbor_swarms_.find(robot_id);
    
        if(os_it != neighbor_swarms_.end()) {
            if(os_it->second.swarmIDExist(swarm_id)) {
                boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
                os_it->second.removeSwarmID(swarm_id);
                os_it->second.age = 0;
            }
            else {
                std::cout<<"robot"<<robot_id<<" is not in swarm"<<swarm_id<<"."<<std::endl;
            }
        }
        else {  //not exist
            std::cout<<"robot_id "<<robot_id<<" neighbor_swarm tuple is not exist."<<std::endl;
            return;
        }
    }
            
    void RuntimeHandle::insertOrRefreshNeighborSwarm(int robot_id, const std::vector<int>& swarm_list)
    {
        std::map<int, NeighborSwarmTuple>::iterator os_it;
        boost::upgrade_lock<boost::shared_mutex> lock(neighbor_swarm_mutex_);
        os_it = neighbor_swarms_.find(robot_id);
    
        if(os_it != neighbor_swarms_.end()) {
            NeighborSwarmTuple new_neighbor_swarm(swarm_list, 0);
            boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
            os_it->second = new_neighbor_swarm;
        }
        else {
            NeighborSwarmTuple new_neighbor_swarm(swarm_list, 0);
            boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
            neighbor_swarms_.insert(std::pair<int, NeighborSwarmTuple>(robot_id ,new_neighbor_swarm));   
        }
    }
    
    void RuntimeHandle::getSwarmMembers(int swarm_id, std::set<int>& swarm_members)
    {
        std::map<int, NeighborSwarmTuple>::iterator os_it;
        swarm_members.clear();
        if(getSwarmFlag(swarm_id)) {
            swarm_members.insert(robot_id_);
        }
        boost::shared_lock<boost::shared_mutex> lock(neighbor_swarm_mutex_);
        for(os_it = neighbor_swarms_.begin(); os_it != neighbor_swarms_.end(); os_it++) {
            if(os_it->second.swarmIDExist(swarm_id)) {
                swarm_members.insert(os_it->first);
            }
        }
    }
    
    void RuntimeHandle::deleteNeighborSwarm(int robot_id)
    {
        boost::unique_lock<boost::shared_mutex> lock(neighbor_swarm_mutex_);
        neighbor_swarms_.erase(robot_id);
    }
    
    void RuntimeHandle::printNeighborSwarm()
    {
        std::map<int, NeighborSwarmTuple>::iterator os_it;
        boost::shared_lock<boost::shared_mutex> lock(neighbor_swarm_mutex_);
        for(os_it = neighbor_swarms_.begin(); os_it != neighbor_swarms_.end(); os_it++) {
            std::cout<<"neighbor swarm "<<os_it->first<<": ";
            std::vector<int> temp = os_it->second.swarm_id_vector;
            for(int i=0; i< temp.size(); i++) {
                std::cout<<temp[i]<<",";
            }
            std::cout<<"age: "<<os_it->second.age;
            std::cout<<std::endl;
        }
    }
            
    void RuntimeHandle::createVirtualStigmergy(int id)
    {
        std::map<int, std::map<std::string, VirtualStigmergyTuple> >::iterator vst_it;
        boost::upgrade_lock<boost::shared_mutex> lock(vstig_mutex_);
        vst_it = virtual_stigmergy_.find(id);
        if(vst_it != virtual_stigmergy_.end()) {
            return;
        }
        else {
            std::map<std::string, VirtualStigmergyTuple> vst;
            boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
            virtual_stigmergy_.insert(std::pair<int, std::map<std::string, VirtualStigmergyTuple> >(id, vst)); 
        }
    }
    
    void RuntimeHandle::insertOrUpdateVirtualStigmergy(int id, const std::string& key, const std::vector<uint8_t>& value, \
                                                       unsigned int lclock, time_t wtime, unsigned int rcount, int robot_id)
    {
        std::map<int, std::map<std::string, VirtualStigmergyTuple> >::iterator vst_it;
        boost::upgrade_lock<boost::shared_mutex> lock(vstig_mutex_);
        vst_it = virtual_stigmergy_.find(id);
    
        if(vst_it != virtual_stigmergy_.end()) {
            std::map<std::string, VirtualStigmergyTuple>::iterator svstt_it = vst_it->second.find(key);
            if(svstt_it != vst_it->second.end()) {
                VirtualStigmergyTuple new_tuple(value, lclock, wtime, rcount, robot_id);
                boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
                svstt_it->second = new_tuple;
            }
            else {
                VirtualStigmergyTuple new_tuple(value, lclock, wtime, rcount, robot_id);
                boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
                vst_it->second.insert(std::pair<std::string, VirtualStigmergyTuple>(key ,new_tuple));
            }
        }
        else {
            std::cout<<"ID "<<id<<" VirtualStigmergy is not exist."<<std::endl;
            return;
        }
    }

    bool RuntimeHandle::isVirtualStigmergyTupleExist(int id, const std::string& key)
    {
        std::map<int, std::map<std::string, VirtualStigmergyTuple> >::iterator vst_it;
        boost::shared_lock<boost::shared_mutex> lock(vstig_mutex_);
        vst_it = virtual_stigmergy_.find(id);
        if(vst_it != virtual_stigmergy_.end()) {
            std::map<std::string, VirtualStigmergyTuple>::iterator svstt_it = vst_it->second.find(key);
            if(svstt_it != vst_it->second.end()) {
                return true;
            }
        }
        return false;
    }
    
    bool RuntimeHandle::getVirtualStigmergyTuple(int id, const std::string& key, VirtualStigmergyTuple& vstig_tuple)
    {
        std::map<int, std::map<std::string, VirtualStigmergyTuple> >::iterator vst_it;
        boost::shared_lock<boost::shared_mutex> lock(vstig_mutex_);
        vst_it = virtual_stigmergy_.find(id);
        if(vst_it != virtual_stigmergy_.end()) {
            std::map<std::string, VirtualStigmergyTuple>::iterator svstt_it = vst_it->second.find(key);
            if(svstt_it != vst_it->second.end()) {
                vstig_tuple = svstt_it->second;
                return true;
            }
        }
        return false;
    }

    void RuntimeHandle::updateVirtualStigmergyTupleReadCount(int id, const std::string& key, int count)
    {
        std::map<int, std::map<std::string, VirtualStigmergyTuple> >::iterator vst_it;
        boost::upgrade_lock<boost::shared_mutex> lock(vstig_mutex_);
        vst_it = virtual_stigmergy_.find(id);

        if(vst_it != virtual_stigmergy_.end()) {
            std::map<std::string, VirtualStigmergyTuple>::iterator svstt_it = vst_it->second.find(key);
            if(svstt_it != vst_it->second.end()) {
                boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
                svstt_it->second.read_count = count;
            }
            else {
                std::cout<<"ID: "<<id<<" VirtualStigmergy, key: "<<key<<" is not exist."<<std::endl;
            }
        }
        else {
            std::cout<<"ID "<<id<<" VirtualStigmergy is not exist."<<std::endl;
            return;
        }
    }
    
    int RuntimeHandle::getVirtualStigmergySize(int id)
    {
        std::map<int, std::map<std::string, VirtualStigmergyTuple> >::iterator vst_it;
        boost::shared_lock<boost::shared_mutex> lock(vstig_mutex_);
        vst_it = virtual_stigmergy_.find(id);
        if(vst_it != virtual_stigmergy_.end()) {
            return vst_it->second.size();
        }
        
        return 0;
    }
    
    void RuntimeHandle::deleteVirtualStigmergy(int id)
    {
        boost::unique_lock<boost::shared_mutex> lock(vstig_mutex_);
        virtual_stigmergy_.erase(id);
    }
    
    void RuntimeHandle::deleteVirtualStigmergyValue(int id, const std::string& key)
    {
        std::map<int, std::map<std::string, VirtualStigmergyTuple> >::iterator vst_it;
        boost::upgrade_lock<boost::shared_mutex> lock(vstig_mutex_);
        vst_it = virtual_stigmergy_.find(id);
        if(vst_it != virtual_stigmergy_.end()) {
            std::map<std::string, VirtualStigmergyTuple>::iterator svstt_it = vst_it->second.find(key);
            if(svstt_it != vst_it->second.end()) {
                boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
                vst_it->second.erase(key);
            }
            else {
                return;
            }
        }
        else {
            return;
        }
    }
    
   void RuntimeHandle::printVirtualStigmergy()
   {
        std::map<int, std::map<std::string, VirtualStigmergyTuple> >::iterator vst_it;
        std::map<std::string, VirtualStigmergyTuple>::iterator svstt_it;
        boost::shared_lock<boost::shared_mutex> lock(vstig_mutex_);
        for (vst_it = virtual_stigmergy_.begin(); vst_it != virtual_stigmergy_.end(); vst_it++) {
            std::cout<<"["<<vst_it->first<<":"<<std::endl;
            std::map<std::string, VirtualStigmergyTuple>* svstt_pointer = &(vst_it->second);
            for (svstt_it = svstt_pointer->begin(); svstt_it != svstt_pointer->end(); svstt_it++) {
                std::cout<<svstt_it->first<<" ";
                /*std::cout<<"("<<svstt_it->first<<",  "<< \
                    svstt_it->second.vstig_value<<",  "<<svstt_it->second.lamport_clock<<",  "<<\
                    svstt_it->second.write_timestamp<<",  "<<svstt_it->second.read_count<<",  "<<\
                    svstt_it->second.robot_id<<")"<<std::endl;*/
            }
            std::cout<<"]"<<std::endl;
            std::cout<<std::endl;
        }
    }

    bool RuntimeHandle::checkNeighborsOverlap(int robot_id)
    {
        if(inNeighbors(robot_id)) {
            boost::shared_lock<boost::shared_mutex> lock(neighbor_mutex_);
            NeighborBase nb;
            if(!getNeighborBase(robot_id, nb)) {
                return false;
            }
            Base msg_src_neighbor(nb.x, nb.y, nb.z, nb.vx, nb.vy, nb.vz, 1);
            std::map<int, NeighborBase>::iterator it = neighbors_.begin();
            for(it = neighbors_.begin(); it != neighbors_.end(); it++) {
                if(it->first == robot_id) {
                    continue;
                }
                Base neighbor(it->second.x, it->second.y, it->second.z, it->second.vx, it->second.vy, it->second.vz, 1);
                if(!cni_->isNeighbor(msg_src_neighbor, neighbor)) {
                    return false;
                }
            }
            return true;
        }
        return false;
    }

    void RuntimeHandle::createBlackBoard(int id)
    {
        std::map<int, std::map<std::string, BlackBoardTuple> >::iterator bb_it;
        boost::upgrade_lock<boost::shared_mutex> lock(bb_mutex_);
        bb_it = blackboard_.find(id);

        if(bb_it != blackboard_.end()) {
            return;
        }
        else {
            std::map<std::string, BlackBoardTuple> bb;
            boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
            blackboard_.insert(std::pair<int, std::map<std::string, BlackBoardTuple> >(id, bb));
        }
    }

    void RuntimeHandle::insertOrUpdateBlackBoard(int id, const std::string& key, const std::vector<uint8_t>& value, const ros::Time& timestamp, int robot_id)
    {
        std::map<int, std::map<std::string, BlackBoardTuple> >::iterator bb_it;
        boost::upgrade_lock<boost::shared_mutex> lock(bb_mutex_);
        bb_it = blackboard_.find(id);

        if(bb_it != blackboard_.end()) {
            std::map<std::string, BlackBoardTuple>::iterator sbbt_it = bb_it->second.find(key);
            if(sbbt_it != bb_it->second.end()) {
                BlackBoardTuple new_tuple(value, timestamp, robot_id);
                boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
                sbbt_it->second = new_tuple;
            }
            else {
                BlackBoardTuple new_tuple(value, timestamp, robot_id);
                boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
                bb_it->second.insert(std::pair<std::string, BlackBoardTuple>(key ,new_tuple));
            }
        }
        else {
            std::cout<<"ID "<<id<<" BlackBoard is not exist."<<std::endl;
            return;
        }
    }

    bool RuntimeHandle::isBlackBoardTupleExist(int id, const std::string& key)
    {
        std::map<int, std::map<std::string, BlackBoardTuple> >::iterator bb_it;
        boost::shared_lock<boost::shared_mutex> lock(bb_mutex_);
        bb_it = blackboard_.find(id);
        if(bb_it != blackboard_.end()) {
            std::map<std::string, BlackBoardTuple>::iterator sbbt_it = bb_it->second.find(key);
            if(sbbt_it != bb_it->second.end()) {
                return true;
            }
        }
        return false;
    }

    void RuntimeHandle::getBlackBoardTuple(int id, const std::string& key, BlackBoardTuple& bb_tuple)
    {
        std::map<int, std::map<std::string, BlackBoardTuple> >::iterator bb_it;
        boost::shared_lock<boost::shared_mutex> lock(bb_mutex_);
        bb_it = blackboard_.find(id);
        if(bb_it != blackboard_.end()) {
            std::map<std::string, BlackBoardTuple>::iterator sbbt_it = bb_it->second.find(key);
            if(sbbt_it != bb_it->second.end()) {
                bb_tuple = sbbt_it->second;
            }
        }
    }

    int RuntimeHandle::getBlackBoardSize(int id)
    {
        std::map<int, std::map<std::string, BlackBoardTuple> >::iterator bb_it;
        boost::shared_lock<boost::shared_mutex> lock(bb_mutex_);
        bb_it = blackboard_.find(id);

        if(bb_it != blackboard_.end()) {
            return bb_it->second.size();
        }

        return 0;
    }

    void RuntimeHandle::deleteBlackBoard(int id)
    {
        boost::unique_lock<boost::shared_mutex> lock(bb_mutex_);
        blackboard_.erase(id);
    }

    void RuntimeHandle::deleteBlackBoardValue(int id, const std::string& key)
    {
        std::map<int, std::map<std::string, BlackBoardTuple> >::iterator bb_it;
        boost::upgrade_lock<boost::shared_mutex> lock(bb_mutex_);
        bb_it = blackboard_.find(id);

        if(bb_it != blackboard_.end()) {
            std::map<std::string, BlackBoardTuple>::iterator sbbt_it=bb_it->second.find(key);
            if(sbbt_it != bb_it->second.end()) {
                boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
                bb_it->second.erase(key);
            }
            else {
                return;
            }
        }
        else {
            return;
        }
    }

    void RuntimeHandle::printBlackBoard()
    {
        std::map<int, std::map<std::string, BlackBoardTuple> >::iterator bb_it;
        std::map<std::string, BlackBoardTuple>::iterator sbbt_it;

        boost::shared_lock<boost::shared_mutex> lock(bb_mutex_);
        for (bb_it = blackboard_.begin(); bb_it != blackboard_.end(); bb_it++) {
            std::cout<<"["<<bb_it->first<<":"<<std::endl;
            std::map<std::string, BlackBoardTuple>* sbbt_pointer = &(bb_it->second);
            for (sbbt_it = sbbt_pointer->begin(); sbbt_it != sbbt_pointer->end(); sbbt_it++) {
                /*std::cout<<"("<<sbbt_it->first<<","<< \
                    sbbt_it->second.bb_value<<","<<sbbt_it->second.timestamp.sec<<","<<\
                    sbbt_it->second.robot_id<<")"<<std::endl;*/
            }
            std::cout<<"]"<<std::endl;
            std::cout<<std::endl;
        }
    }
    
    const float& RuntimeHandle::getNeighborDistance()
    {
        boost::shared_lock<boost::shared_mutex> lock(neighbor_distance_mutex_);
        return neighbor_distance_;
    }
    
    void RuntimeHandle::setNeighborDistance(float neighbor_distance)
    {
        boost::unique_lock<boost::shared_mutex> lock(neighbor_distance_mutex_);
        neighbor_distance_ = neighbor_distance;
        cni_.reset(new CheckNeighbor(neighbor_distance_));
        std::cout<<"neighbor distance is set to "<<neighbor_distance_<<std::endl;
        clearNeighbors();
    }
    
    void RuntimeHandle::insertOrUpdateListenerHelper(const std::string& key, const boost::shared_ptr<ListenerHelper> helper)
    {
        std::map<std::string, boost::shared_ptr<ListenerHelper> >::iterator lh_it;
        boost::upgrade_lock<boost::shared_mutex> lock(listener_helpers_mutex_);
        lh_it = listener_helpers_.find(key);
    
        if(lh_it != listener_helpers_.end()) {
            boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
            lh_it->second = helper;
        }
        else {
            boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
            listener_helpers_.insert(std::pair<std::string, boost::shared_ptr<ListenerHelper> >(key ,helper));
        } 
    }
    
    const boost::shared_ptr<ListenerHelper> RuntimeHandle::getListenerHelper(const std::string& key)
    {
        std::map<std::string, boost::shared_ptr<ListenerHelper> >::iterator lh_it;
        boost::shared_lock<boost::shared_mutex> lock(listener_helpers_mutex_);
        lh_it = listener_helpers_.find(key);
    
        if(lh_it != listener_helpers_.end()) {
            return lh_it->second;
        }
        
        std::cout<<"could not get the callback function which has the key "<<key<<"!"<<std::endl;
        return NULL;
    }
    
    void RuntimeHandle::deleteListenerHelper(const std::string& key)
    {
        boost::unique_lock<boost::shared_mutex> lock(listener_helpers_mutex_);
        listener_helpers_.erase(key);
    }
    
    void RuntimeHandle::insertBarrier(int robot_id)
    {
        boost::unique_lock<boost::shared_mutex> lock(barrier_mutex_);
        barrier_.insert(robot_id);
    }
    
    int RuntimeHandle::getBarrierSize()
    {
        boost::shared_lock<boost::shared_mutex> lock(barrier_mutex_);
        return barrier_.size();
    }

    bool RuntimeHandle::getSCDSPSOValue(const std::string& aKey, SCDSPSODataTuple& aT)
    {
        boost::shared_lock<boost::shared_mutex> lock(scds_pso_tuple_mutex_);
        std::map<std::string, SCDSPSODataTuple>::iterator iter = scds_pso_tuple_.find(aKey);
        if (iter != scds_pso_tuple_.end()) {
            aT = iter->second;
            return true;
        }
        else return false;
    }

    void RuntimeHandle::insertOrUpdateSCDSPSOValue(const std::string& aKey, const SCDSPSODataTuple& aT)
    {
        boost::upgrade_lock<boost::shared_mutex> lock(scds_pso_tuple_mutex_);
        std::map<std::string, SCDSPSODataTuple>::iterator iter = scds_pso_tuple_.find(aKey);
        /*if (iter!=scds_pso_tuple_.end()) {
            //if (iter->second.val < aT.val)
                scds_pso_tuple_[aKey]=aT;
        }
        else
            scds_pso_tuple_[aKey]=aT;*/
        if(iter != scds_pso_tuple_.end()) {
            boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
            iter->second = aT;
        }
        else {
            boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
            scds_pso_tuple_[aKey] = aT;
        }
    }
};
