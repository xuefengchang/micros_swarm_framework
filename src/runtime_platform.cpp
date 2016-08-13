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

#include "ros/ros.h"

#include "micros_swarm_framework/data_type.h"
#include "micros_swarm_framework/runtime_platform.h"

namespace micros_swarm_framework{

    RuntimePlatform::RuntimePlatform()
    {
        robot_id_=-1;
        
        neighbors_.clear();
        swarms_.clear();
        neighbor_swarms_.clear();
        virtual_stigmergy_.clear();
        barrier_.clear();
        callback_functions_.clear();
    }
    
    RuntimePlatform::RuntimePlatform(int robot_id)
    {
        robot_id_=robot_id;
        
        neighbors_.clear();
        swarms_.clear();
        neighbor_swarms_.clear();
        virtual_stigmergy_.clear();
        barrier_.clear();
        callback_functions_.clear();
    }
    
    int RuntimePlatform::getRobotID()
    {
        return robot_id_;
    }
    
    void RuntimePlatform::setRobotID(int robot_id)
    {
        robot_id_=robot_id;
    }
    
    Base RuntimePlatform::getRobotBase()
    {
        return robot_base_;
    }
    
    void RuntimePlatform::setRobotBase(Base robot_base)
    {
        robot_base_=robot_base;
    }
    
    void RuntimePlatform::printRobotBase()
    {
        std::cout<<"robot base: "<<robot_base_.getX()<<", "<<robot_base_.getY()<<", "<<\
            robot_base_.getZ()<<", "<<robot_base_.getVX()<<", "<<robot_base_.getVY()<<", "<<\
            robot_base_.getVZ()<<std::endl;
    }
     
    std::map<int, NeighborBase> RuntimePlatform::getNeighbors()
    {
        return neighbors_;
    }
     
    void RuntimePlatform::insertOrUpdateNeighbor(int robot_id, float distance, float azimuth, float elevation, float x, float y, float z, float vx, float vy, float vz)
    {
        std::map<int, NeighborBase>::iterator n_it=neighbors_.find(robot_id);
    
        if(n_it!=neighbors_.end())
        {
            NeighborBase new_neighbor_base(distance, azimuth, elevation, x, y, z,vx, vy, vz);
            n_it->second = new_neighbor_base;
        }
        else
        {
            NeighborBase new_neighbor_base(distance, azimuth, elevation, x, y, z, vx, vy, vz);
            neighbors_.insert(std::pair<int, NeighborBase>(robot_id ,new_neighbor_base));
        }
    }
    
    void RuntimePlatform::deleteNeighbor(int robot_id)
    { 
        neighbors_.erase(robot_id);
    }
    
    void RuntimePlatform::printNeighbor()
    {
        std::map<int, NeighborBase>::iterator n_it;
        
        for (n_it=neighbors_.begin(); n_it!=neighbors_.end(); n_it++)
        {
            std::cout<<n_it->first<<": ";
            
            std::cout<<n_it->second.getDistance()<<","<<n_it->second.getAzimuth()<<","<<n_it->second.getElevation()<<","<<\
                n_it->second.getX()<<","<<n_it->second.getY()<<","<<n_it->second.getZ()<<", "<<
                n_it->second.getVX()<<","<<n_it->second.getVY()<<","<<n_it->second.getVZ();
            std::cout<<std::endl;
        }
    }
     
    void RuntimePlatform::insertOrUpdateSwarm(int swarm_id, bool value)
    {
        std::map<int, bool>::iterator s_it=swarms_.find(swarm_id);
    
        if(s_it!=swarms_.end())
        {
            s_it->second=value;
        }
        else
        {
            swarms_.insert(std::pair<int, bool>(swarm_id, value));
        }
    }
    
    bool RuntimePlatform::getSwarm(int swarm_id)
    {
        std::map<int, bool>::iterator s_it=swarms_.find(swarm_id);
    
        if(s_it!=swarms_.end())
        {
            return s_it->second;
        }
    
        return false;
    }
    
    std::vector<int> RuntimePlatform::getSwarmList()
    {
        std::vector<int> result;
        result.clear();
  
        std::map<int, bool>::iterator s_it;
        
        for(s_it=swarms_.begin();s_it!=swarms_.end();s_it++)
        {
            if(s_it->second)
                result.push_back(s_it->first);
        }
    
        return result;
    }
    
    void RuntimePlatform::deleteSwarm(int swarm_id)
    {
        swarms_.erase(swarm_id);
    }
    
    void RuntimePlatform::printSwarm()
    {
        std::map<int, bool>::iterator s_it;
        
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
        os_it=neighbor_swarms_.find(robot_id);
    
        if(os_it!=neighbor_swarms_.end())
        {
            if(os_it->second.swarmIDExist(swarm_id))
            {
                os_it->second.setAge(0);
            }
            else
            {           
                os_it->second.addSwarmID(swarm_id);
                os_it->second.setAge(0);
            }
        }
        else
        {
            std::vector<int> swarm_list;
            swarm_list.push_back(swarm_id);
            NeighborSwarmTuple new_neighbor_swarm(swarm_list, 0);
            
            neighbor_swarms_.insert(std::pair<int, NeighborSwarmTuple>(robot_id, new_neighbor_swarm));
        }
    }
    
    void RuntimePlatform::leaveNeighborSwarm(int robot_id, int swarm_id)
    {
        std::map<int, NeighborSwarmTuple>::iterator os_it;
        os_it=neighbor_swarms_.find(robot_id);
    
        if(os_it!=neighbor_swarms_.end())
        {
            if(os_it->second.swarmIDExist(swarm_id))
            {
                os_it->second.removeSwarmID(swarm_id);
                os_it->second.setAge(0);
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
            
    void RuntimePlatform::insertOrRefreshNeighborSwarm(int robot_id, std::vector<int> swarm_list)
    {
        std::map<int, NeighborSwarmTuple>::iterator os_it;
        os_it=neighbor_swarms_.find(robot_id);
    
        if(os_it!=neighbor_swarms_.end())
        {
            NeighborSwarmTuple new_neighbor_swarm(swarm_list, 0);
            os_it->second=new_neighbor_swarm;
        }
        else
        {
            NeighborSwarmTuple new_neighbor_swarm(swarm_list, 0);  
            neighbor_swarms_.insert(std::pair<int, NeighborSwarmTuple>(robot_id ,new_neighbor_swarm));   
        }
    }
    
    std::set<int> RuntimePlatform::getSwarmMembers(int swarm_id)
    {
        std::set<int> result;
        result.clear();
        
        std::map<int, NeighborSwarmTuple>::iterator os_it;
        
        for(os_it=neighbor_swarms_.begin(); os_it!=neighbor_swarms_.end(); os_it++)
        {
            if(os_it->second.swarmIDExist(swarm_id))
                result.insert(os_it->first);
        }
        
        return result;
    }
    
    void RuntimePlatform::deleteNeighborSwarm(int robot_id)
    {
        neighbor_swarms_.erase(robot_id);
    }
    
    void RuntimePlatform::printNeighborSwarm()
    {
        std::map<int, NeighborSwarmTuple>::iterator os_it;
        
        for(os_it=neighbor_swarms_.begin(); os_it!=neighbor_swarms_.end(); os_it++)
        {
            std::cout<<"neighbor swarm "<<os_it->first<<": ";
            
            std::vector<int> temp=os_it->second.getSwarmIDVector();
            for(int i=0;i<temp.size();i++)
            {
                std::cout<<temp[i]<<",";
            }
            std::cout<<"age: "<<os_it->second.getAge();
            std::cout<<std::endl;
        }
    }
            
    void RuntimePlatform::createVirtualStigmergy(int id)
    {
        std::map<int, std::map<std::string, VirtualStigmergyTuple> >::iterator vst_it;
        vst_it=virtual_stigmergy_.find(id);
    
        if(vst_it!=virtual_stigmergy_.end())
        {
            return;
        }
        else
        {
            std::map<std::string, VirtualStigmergyTuple> vst;
            virtual_stigmergy_.insert(std::pair<int, std::map<std::string, VirtualStigmergyTuple> >(id, vst)); 
        }
    }
    
    void RuntimePlatform::insertOrUpdateVirtualStigmergy(int id, std::string key, std::string value, time_t time_now, int robot_id)
    {
        std::map<int, std::map<std::string, VirtualStigmergyTuple> >::iterator vst_it;
        vst_it=virtual_stigmergy_.find(id);
    
        if(vst_it!=virtual_stigmergy_.end())
        {
            std::map<std::string, VirtualStigmergyTuple>::iterator svstt_it=vst_it->second.find(key);
        
            if(svstt_it!=vst_it->second.end())
            {
                VirtualStigmergyTuple new_tuple(value, time_now, robot_id);
                svstt_it->second = new_tuple;
            }
            else
            {
                VirtualStigmergyTuple new_tuple(value, time_now, robot_id);
                vst_it->second.insert(std::pair<std::string, VirtualStigmergyTuple>(key ,new_tuple));
            }
        }
        else
        {       
            std::cout<<"ID "<<id<<" VirtualStigmergy is not exist."<<std::endl;
            return;
        }
    }
    
    VirtualStigmergyTuple RuntimePlatform::getVirtualStigmergyTuple(int id, std::string key)
    {
        std::map<int, std::map<std::string, VirtualStigmergyTuple> >::iterator vst_it;
        vst_it=virtual_stigmergy_.find(id);
    
        if(vst_it!=virtual_stigmergy_.end())
        {
            std::map<std::string, VirtualStigmergyTuple>::iterator svstt_it=vst_it->second.find(key);
        
            if(svstt_it!=vst_it->second.end())
            {     
                return svstt_it->second;
            }
        }
        
        std::string value="";
        time_t time_now=0;
        unsigned int robot_id=0;
        VirtualStigmergyTuple vst_tuple(value, time_now, robot_id);
            
        return vst_tuple;
    }
    
    int RuntimePlatform::getVirtualStigmergySize(int id)
    {
        std::map<int, std::map<std::string, VirtualStigmergyTuple> >::iterator vst_it;
        vst_it=virtual_stigmergy_.find(id);
    
        if(vst_it!=virtual_stigmergy_.end())
        {
            return vst_it->second.size();
        }
        
        return 0;
    }
    
    void RuntimePlatform::deleteVirtualStigmergy(int id)
    {
        virtual_stigmergy_.erase(id);
    }
    
    void RuntimePlatform::deleteVirtualStigmergyValue(int id, std::string key)
    {
        std::map<int, std::map<std::string, VirtualStigmergyTuple> >::iterator vst_it;
        vst_it=virtual_stigmergy_.find(id);
    
        if(vst_it!=virtual_stigmergy_.end())
        {
            std::map<std::string, VirtualStigmergyTuple>::iterator svstt_it=vst_it->second.find(key);
        
            if(svstt_it!=vst_it->second.end())
            {
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
        
        for (vst_it=virtual_stigmergy_.begin(); vst_it!=virtual_stigmergy_.end(); vst_it++)
        {
            std::cout<<"["<<vst_it->first<<":"<<std::endl;
            std::map<std::string, VirtualStigmergyTuple>* svstt_pointer=&(vst_it->second);
            for (svstt_it=svstt_pointer->begin(); svstt_it!=svstt_pointer->end(); svstt_it++)
            {
                std::cout<<"("<<svstt_it->first<<","<< \
                    svstt_it->second.getVirtualStigmergyValue()<<","<<svstt_it->second.getVirtualStigmergyTimestamp()<<","<<\
                    svstt_it->second.getRobotID()<<")"<<std::endl;
            }
            std::cout<<"]"<<std::endl;
            std::cout<<std::endl;
        }
    }
    
    double RuntimePlatform::getNeighborDistance()
    {
        return neighbor_distance_;
    }
    
    void RuntimePlatform::setNeighborDistance(double neighbor_distance)
    {
        neighbor_distance_=neighbor_distance;
    }
    
    void RuntimePlatform::insertBarrier(int robot_id)
    {
        barrier_.insert(robot_id);
    }
    
    int RuntimePlatform::getBarrierSize()
    {
        return barrier_.size();
    }
    
    void RuntimePlatform::insertOrUpdateCallbackFunctions(std::string key, boost::function<void(const std::string&)> cb)
    {
        std::map<std::string, boost::function<void(const std::string&)> >::iterator nccb_it;
        nccb_it=callback_functions_.find(key);
    
        if(nccb_it!=callback_functions_.end())
        {
            nccb_it->second = cb;
        }
        else
        {
            callback_functions_.insert(std::pair<std::string, boost::function<void(const std::string&)> >(key ,cb));
        } 
    }
    
    void RuntimePlatform::doNothing(const std::string& value_str)
    {
        
    }
    
    boost::function<void(const std::string&)> RuntimePlatform::getCallbackFunctions(std::string key)
    {
        std::map<std::string, boost::function<void(const std::string&)> >::iterator nccb_it;
        nccb_it=callback_functions_.find(key);
    
        if(nccb_it!=callback_functions_.end())
        {
            return nccb_it->second;
        }
        
        std::cout<<"could not get the callback function which has the key "<<key<<"!"<<std::endl;
        boost::function<void(const std::string&)> f=boost::bind(&RuntimePlatform::doNothing, this, _1);
        return f;
    }
    
    void RuntimePlatform::deleteCallbackFunctions(std::string key)
    {
        callback_functions_.erase(key);
    }
};
