/**
Software License Agreement (BSD)
\file      runtime_platform.cpp 
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

#include <boost/thread.hpp> 
#include "ros/ros.h"

#include "micros_swarm_framework/data_type.h"
#include "micros_swarm_framework/runtime_platform.h"

namespace micros_swarm_framework{
    
    RuntimePlatform::RuntimePlatform(int robot_id)
    {
        robot_id_=robot_id;
        robot_base_=Base(0,0,0,0,0,0);
        neighbors_.clear();
        swarms_.clear();
        neighbor_swarms_.clear();
        virtual_stigmergy_.clear();
        barrier_.clear();
        listener_helpers_.clear();
        listener_helpers_.insert(std::pair<std::string, boost::shared_ptr<ListenerHelper> >("" , NULL));
    }
    
    int RuntimePlatform::getRobotID()
    {
        boost::shared_lock<boost::shared_mutex> lock(mutex1_);
        return robot_id_;
    }
    
    void RuntimePlatform::setRobotID(int robot_id)
    {
        boost::unique_lock<boost::shared_mutex> lock(mutex1_);
        robot_id_=robot_id;
    }
    
    int RuntimePlatform::getRobotType()
    {
        boost::shared_lock<boost::shared_mutex> lock(mutex2_);
        return robot_type_;
    }
    
    void RuntimePlatform::setRobotType(int robot_type)
    {
        boost::unique_lock<boost::shared_mutex> lock(mutex2_);
        robot_type_=robot_type;
    }
    
    int RuntimePlatform::getRobotStatus()
    {
        boost::shared_lock<boost::shared_mutex> lock(mutex3_);
        return robot_status_;
    }
    
    void RuntimePlatform::setRobotStatus(int robot_status)
    {
        boost::unique_lock<boost::shared_mutex> lock(mutex3_);
        robot_status_=robot_status;
    }
    
    const Base& RuntimePlatform::getRobotBase()
    {
        boost::shared_lock<boost::shared_mutex> lock(mutex4_);
        return robot_base_;
    }
    
    void RuntimePlatform::setRobotBase(const Base& robot_base)
    {
        boost::unique_lock<boost::shared_mutex> lock(mutex4_);
        robot_base_=robot_base;
        
        if(robot_base.valid==-1)
            robot_base_.valid=1;
    }    
    
    void RuntimePlatform::printRobotBase()
    {
        boost::shared_lock<boost::shared_mutex> lock(mutex4_);
        std::cout<<"robot base: "<<robot_base_.x<<", "<<robot_base_.y<<", "<<\
            robot_base_.z<<", "<<robot_base_.vx<<", "<<robot_base_.vy<<", "<<\
            robot_base_.vz<<std::endl;
    }
        
    void RuntimePlatform::getNeighbors(std::map<int, NeighborBase>& neighbors)
    {
        boost::shared_lock<boost::shared_mutex> lock(mutex5_);
        neighbors=neighbors_;
    }
     
    void RuntimePlatform::insertOrUpdateNeighbor(int robot_id, float distance, float azimuth, float elevation, float x, float y, float z, float vx, float vy, float vz)
    {
        boost::upgrade_lock<boost::shared_mutex> lock(mutex5_);
        std::map<int, NeighborBase>::iterator n_it=neighbors_.find(robot_id);
    
        if(n_it!=neighbors_.end())
        {
            NeighborBase new_neighbor_base(distance, azimuth, elevation, x, y, z,vx, vy, vz);
            boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
            n_it->second = new_neighbor_base;
        }
        else
        {
            NeighborBase new_neighbor_base(distance, azimuth, elevation, x, y, z, vx, vy, vz);
            boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
            neighbors_.insert(std::pair<int, NeighborBase>(robot_id ,new_neighbor_base));
        }
    }
    
    void RuntimePlatform::deleteNeighbor(int robot_id)
    {
        boost::unique_lock<boost::shared_mutex> lock(mutex5_);
        neighbors_.erase(robot_id);
    }
    
    bool RuntimePlatform::inNeighbors(int robot_id)
    {
        boost::shared_lock<boost::shared_mutex> lock(mutex5_);
        std::map<int, NeighborBase>::iterator n_it=neighbors_.find(robot_id);
    
        if(n_it!=neighbors_.end())
        {
            return true;
        }
        
        return false;
    }
    
    void RuntimePlatform::printNeighbor()
    {
        std::map<int, NeighborBase>::iterator n_it;
        
        boost::shared_lock<boost::shared_mutex> lock(mutex5_);
        for (n_it=neighbors_.begin(); n_it!=neighbors_.end(); n_it++)
        {
            std::cout<<n_it->first<<": ";
            
            std::cout<<n_it->second.distance<<","<<n_it->second.azimuth<<","<<n_it->second.elevation<<","<<\
                n_it->second.x<<","<<n_it->second.y<<","<<n_it->second.z<<", "<<
                n_it->second.vx<<","<<n_it->second.vy<<","<<n_it->second.vz;
            std::cout<<std::endl;
        }
    }
     
    void RuntimePlatform::insertOrUpdateSwarm(int swarm_id, bool value)
    {
        boost::upgrade_lock<boost::shared_mutex> lock(mutex6_);
        std::map<int, bool>::iterator s_it=swarms_.find(swarm_id);
    
        if(s_it!=swarms_.end())
        {
            boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
            s_it->second=value;
        }
        else
        {
            boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
            swarms_.insert(std::pair<int, bool>(swarm_id, value));
        }
    }
    
    bool RuntimePlatform::getSwarmFlag(int swarm_id)
    {
        boost::shared_lock<boost::shared_mutex> lock(mutex6_);
        std::map<int, bool>::iterator s_it=swarms_.find(swarm_id);
    
        if(s_it!=swarms_.end())
        {
            return s_it->second;
        }
    
        return false;
    }
    
    void RuntimePlatform::getSwarmList(std::vector<int>& swarm_list)
    {
        swarm_list.clear();
  
        std::map<int, bool>::iterator s_it;
        
        boost::shared_lock<boost::shared_mutex> lock(mutex6_);
        for(s_it=swarms_.begin();s_it!=swarms_.end();s_it++)
        {
            if(s_it->second)
                swarm_list.push_back(s_it->first);
        }
    }
    
    void RuntimePlatform::deleteSwarm(int swarm_id)
    {
        boost::unique_lock<boost::shared_mutex> lock(mutex6_);
        swarms_.erase(swarm_id);
    }
    
    void RuntimePlatform::printSwarm()
    {
        std::map<int, bool>::iterator s_it;
        
        boost::shared_lock<boost::shared_mutex> lock(mutex6_);
        for(s_it=swarms_.begin();s_it!=swarms_.end();s_it++)
        {
            std::cout<<s_it->first<<": ";
            std::cout<<s_it->second;
            std::cout<<std::endl;
        }
    }
    
    bool RuntimePlatform::inNeighborSwarm(int robot_id, int swarm_id)
    {
        std::map<int, NeighborSwarmTuple>::iterator os_it;
        boost::shared_lock<boost::shared_mutex> lock(mutex7_);
        os_it=neighbor_swarms_.find(robot_id);
    
        if(os_it!=neighbor_swarms_.end())
        {
            if(os_it->second.swarmIDExist(swarm_id))
            {
               return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }
    
    void RuntimePlatform::joinNeighborSwarm(int robot_id, int swarm_id)
    {
        std::map<int, NeighborSwarmTuple>::iterator os_it;
        boost::upgrade_lock<boost::shared_mutex> lock(mutex7_);
        os_it=neighbor_swarms_.find(robot_id);
    
        if(os_it!=neighbor_swarms_.end())
        {
            if(os_it->second.swarmIDExist(swarm_id))
            {
                boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
                os_it->second.age=0;
            }
            else
            {           
                boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
                os_it->second.addSwarmID(swarm_id);
                os_it->second.age=0;
            }
        }
        else
        {
            std::vector<int> swarm_list;
            swarm_list.push_back(swarm_id);
            NeighborSwarmTuple new_neighbor_swarm(swarm_list, 0);
            
            boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
            neighbor_swarms_.insert(std::pair<int, NeighborSwarmTuple>(robot_id, new_neighbor_swarm));
        }
    }
    
    void RuntimePlatform::leaveNeighborSwarm(int robot_id, int swarm_id)
    {
        std::map<int, NeighborSwarmTuple>::iterator os_it;
        boost::upgrade_lock<boost::shared_mutex> lock(mutex7_);
        os_it=neighbor_swarms_.find(robot_id);
    
        if(os_it!=neighbor_swarms_.end())
        {
            if(os_it->second.swarmIDExist(swarm_id))
            {
                boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
                os_it->second.removeSwarmID(swarm_id);
                os_it->second.age=0;
            }
            else
            {
                std::cout<<"robot"<<robot_id<<" is not in swarm"<<swarm_id<<"."<<std::endl;
            }
        }
        else  //not exist
        {
            std::cout<<"robot_id "<<robot_id<<" neighbor_swarm tuple is not exist."<<std::endl;
            return;
        }
    }
            
    void RuntimePlatform::insertOrRefreshNeighborSwarm(int robot_id, const std::vector<int>& swarm_list)
    {
        std::map<int, NeighborSwarmTuple>::iterator os_it;
        boost::upgrade_lock<boost::shared_mutex> lock(mutex7_);
        os_it=neighbor_swarms_.find(robot_id);
    
        if(os_it!=neighbor_swarms_.end())
        {
            NeighborSwarmTuple new_neighbor_swarm(swarm_list, 0);
            boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
            os_it->second=new_neighbor_swarm;
        }
        else
        {
            NeighborSwarmTuple new_neighbor_swarm(swarm_list, 0);
            boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
            neighbor_swarms_.insert(std::pair<int, NeighborSwarmTuple>(robot_id ,new_neighbor_swarm));   
        }
    }
    
    void RuntimePlatform::getSwarmMembers(int swarm_id, std::set<int>& swarm_members)
    {
        std::map<int, NeighborSwarmTuple>::iterator os_it;
        swarm_members.clear();
        
        boost::shared_lock<boost::shared_mutex> lock(mutex7_);
        for(os_it=neighbor_swarms_.begin(); os_it!=neighbor_swarms_.end(); os_it++)
        {
            if(os_it->second.swarmIDExist(swarm_id))
                swarm_members.insert(os_it->first);
        }
    }
    
    void RuntimePlatform::deleteNeighborSwarm(int robot_id)
    {
        boost::unique_lock<boost::shared_mutex> lock(mutex7_);
        neighbor_swarms_.erase(robot_id);
    }
    
    void RuntimePlatform::printNeighborSwarm()
    {
        std::map<int, NeighborSwarmTuple>::iterator os_it;
        
        boost::shared_lock<boost::shared_mutex> lock(mutex7_);
        for(os_it=neighbor_swarms_.begin(); os_it!=neighbor_swarms_.end(); os_it++)
        {
            std::cout<<"neighbor swarm "<<os_it->first<<": ";
            
            std::vector<int> temp=os_it->second.swarm_id_vector;
            for(int i=0;i<temp.size();i++)
            {
                std::cout<<temp[i]<<",";
            }
            std::cout<<"age: "<<os_it->second.age;
            std::cout<<std::endl;
        }
    }
            
    void RuntimePlatform::createVirtualStigmergy(int id)
    {
        std::map<int, std::map<std::string, VirtualStigmergyTuple> >::iterator vst_it;
        boost::upgrade_lock<boost::shared_mutex> lock(mutex8_);
        vst_it=virtual_stigmergy_.find(id);
    
        if(vst_it!=virtual_stigmergy_.end())
        {
            return;
        }
        else
        {
            std::map<std::string, VirtualStigmergyTuple> vst;
            boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
            virtual_stigmergy_.insert(std::pair<int, std::map<std::string, VirtualStigmergyTuple> >(id, vst)); 
        }
    }
    
    void RuntimePlatform::insertOrUpdateVirtualStigmergy(int id, const std::string& key, const std::string& value, const time_t& time_now, int robot_id)
    {
        std::map<int, std::map<std::string, VirtualStigmergyTuple> >::iterator vst_it;
        boost::upgrade_lock<boost::shared_mutex> lock(mutex8_);
        vst_it=virtual_stigmergy_.find(id);
    
        if(vst_it!=virtual_stigmergy_.end())
        {
            std::map<std::string, VirtualStigmergyTuple>::iterator svstt_it=vst_it->second.find(key);
        
            if(svstt_it!=vst_it->second.end())
            {
                VirtualStigmergyTuple new_tuple(value, time_now, robot_id);
                boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
                svstt_it->second = new_tuple;
            }
            else
            {
                VirtualStigmergyTuple new_tuple(value, time_now, robot_id);
                boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
                vst_it->second.insert(std::pair<std::string, VirtualStigmergyTuple>(key ,new_tuple));
            }
        }
        else
        {
            std::cout<<"ID "<<id<<" VirtualStigmergy is not exist."<<std::endl;
            return;
        }
    }
    
    void RuntimePlatform::getVirtualStigmergyTuple(int id, const std::string& key, VirtualStigmergyTuple& vstig_tuple)
    {
        std::map<int, std::map<std::string, VirtualStigmergyTuple> >::iterator vst_it;
        boost::shared_lock<boost::shared_mutex> lock(mutex8_);
        vst_it=virtual_stigmergy_.find(id);
        if(vst_it!=virtual_stigmergy_.end())
        {
            std::map<std::string, VirtualStigmergyTuple>::iterator svstt_it=vst_it->second.find(key);
        
            if(svstt_it!=vst_it->second.end())
            {
                vstig_tuple=svstt_it->second;
            }
        }
    }
    
    int RuntimePlatform::getVirtualStigmergySize(int id)
    {
        std::map<int, std::map<std::string, VirtualStigmergyTuple> >::iterator vst_it;
        boost::shared_lock<boost::shared_mutex> lock(mutex8_);
        vst_it=virtual_stigmergy_.find(id);
    
        if(vst_it!=virtual_stigmergy_.end())
        {
            return vst_it->second.size();
        }
        
        return 0;
    }
    
    void RuntimePlatform::deleteVirtualStigmergy(int id)
    {
        boost::unique_lock<boost::shared_mutex> lock(mutex8_);
        virtual_stigmergy_.erase(id);
    }
    
    void RuntimePlatform::deleteVirtualStigmergyValue(int id, const std::string& key)
    {
        std::map<int, std::map<std::string, VirtualStigmergyTuple> >::iterator vst_it;
        boost::upgrade_lock<boost::shared_mutex> lock(mutex8_);
        vst_it=virtual_stigmergy_.find(id);
    
        if(vst_it!=virtual_stigmergy_.end())
        {
            std::map<std::string, VirtualStigmergyTuple>::iterator svstt_it=vst_it->second.find(key);
        
            if(svstt_it!=vst_it->second.end())
            {
                boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
                vst_it->second.erase(key);
            }
            else
            {
                return;
            }
        }
        else
        {
            return;
        }
    }
    
   void RuntimePlatform::printVirtualStigmergy()
    {
        std::map<int, std::map<std::string, VirtualStigmergyTuple> >::iterator vst_it;
        std::map<std::string, VirtualStigmergyTuple>::iterator svstt_it;
        
        boost::shared_lock<boost::shared_mutex> lock(mutex8_);
        for (vst_it=virtual_stigmergy_.begin(); vst_it!=virtual_stigmergy_.end(); vst_it++)
        {
            std::cout<<"["<<vst_it->first<<":"<<std::endl;
            std::map<std::string, VirtualStigmergyTuple>* svstt_pointer=&(vst_it->second);
            for (svstt_it=svstt_pointer->begin(); svstt_it!=svstt_pointer->end(); svstt_it++)
            {
                std::cout<<"("<<svstt_it->first<<","<< \
                    svstt_it->second.vstig_value<<","<<svstt_it->second.vstig_timestamp<<","<<\
                    svstt_it->second.robot_id<<")"<<std::endl;
            }
            std::cout<<"]"<<std::endl;
            std::cout<<std::endl;
        }
    }
    
    float RuntimePlatform::getNeighborDistance()
    {
        boost::shared_lock<boost::shared_mutex> lock(mutex9_);
        return neighbor_distance_;
    }
    
    void RuntimePlatform::setNeighborDistance(float neighbor_distance)
    {
        boost::unique_lock<boost::shared_mutex> lock(mutex9_);
        neighbor_distance_=neighbor_distance;
    }
    
    void RuntimePlatform::insertOrUpdateListenerHelper(const std::string& key, const boost::shared_ptr<ListenerHelper> helper)
    {
        std::map<std::string, boost::shared_ptr<ListenerHelper> >::iterator lh_it;
        boost::upgrade_lock<boost::shared_mutex> lock(mutex10_);
        lh_it=listener_helpers_.find(key);
    
        if(lh_it!=listener_helpers_.end())
        {
            boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
            lh_it->second = helper;
        }
        else
        {
            boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
            listener_helpers_.insert(std::pair<std::string, boost::shared_ptr<ListenerHelper> >(key ,helper));
        } 
    }
    
    const boost::shared_ptr<ListenerHelper> RuntimePlatform::getListenerHelper(const std::string& key)
    {
        std::map<std::string, boost::shared_ptr<ListenerHelper> >::iterator lh_it;
        boost::shared_lock<boost::shared_mutex> lock(mutex10_);
        lh_it=listener_helpers_.find(key);
    
        if(lh_it!=listener_helpers_.end())
        {
            return lh_it->second;
        }
        
        std::cout<<"could not get the callback function which has the key "<<key<<"!"<<std::endl;
        return NULL;
    }
    
    void RuntimePlatform::deleteListenerHelper(const std::string& key)
    {
        boost::unique_lock<boost::shared_mutex> lock(mutex10_);
        listener_helpers_.erase(key);
    }
    
     void RuntimePlatform::insertBarrier(int robot_id)
    {
        boost::unique_lock<boost::shared_mutex> lock(mutex11_);
        barrier_.insert(robot_id);
    }
    
    int RuntimePlatform::getBarrierSize()
    {
        boost::shared_lock<boost::shared_mutex> lock(mutex11_);
        return barrier_.size();
    }
};
