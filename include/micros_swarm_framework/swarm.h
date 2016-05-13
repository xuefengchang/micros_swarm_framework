/**
Software License Agreement (BSD)
\file      swarm.h
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

#ifndef SAWARM_H_
#define SWARM_H_

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
#include <functional>

#include "ros/ros.h"

#include "micros_swarm_framework/kernel.h"

namespace micros_swarm_framework{
    
    class Swarm{
        private:
            unsigned int swarm_id_;
        public:
            Swarm(){swarm_id_=-1;};
            Swarm(unsigned int swarm_id);
            
            std::set<unsigned int> getSwarmMembers();
            void joinSwarm();
            void leaveSwarm();
            void selectSwarm(boost::function<bool()> bf);
            void unselectSwarm(boost::function<bool()> bf);
            bool inSwarm();
            //execute a function
            void execute(boost::function<void()> f);
            void breakupSwarm();
            Swarm intersectionSwarm(Swarm s, unsigned int new_swarm_id);
            Swarm unionSwarm(Swarm s, unsigned int new_swarm_id);
            Swarm differenceSwarm(Swarm s, unsigned int new_swarm_id);
            Swarm negationSwarm(unsigned int new_swarm_id);
    };
    
    Swarm::Swarm(unsigned int swarm_id)
    {
        swarm_id_=swarm_id;
        
        micros_swarm_framework::KernelHandle kh;
        kh.insertOrUpdateSwarm(swarm_id_, 0);
    }
    
    std::set<unsigned int> Swarm::getSwarmMembers()
    {
        micros_swarm_framework::KernelHandle kh;
        std::set<unsigned int> result=kh.getSwarmMembers(swarm_id_);
        return result;
    }
    
    void Swarm::joinSwarm()
    {
        micros_swarm_framework::KernelHandle kh;
        unsigned int robot_id=kh.getRobotID();
        kh.insertOrUpdateSwarm(swarm_id_, 1);
        
        SingleRobotJoinSwarm srjs(robot_id, swarm_id_);
        
        std::ostringstream archiveStream;
        boost::archive::text_oarchive archive(archiveStream);
        archive<<srjs;
        std::string srjs_str=archiveStream.str();   
                      
        micros_swarm_framework::MSFPPacket p;
        p.packet_source=robot_id;
        p.packet_version=1;
        p.packet_type=SINGLE_ROBOT_JOIN_SWARM;
        p.packet_data=srjs_str;
        p.package_check_sum=0;
                
        kh.publishPacket(p);
    }
    
    void Swarm::leaveSwarm()
    {
        micros_swarm_framework::KernelHandle kh;
        unsigned int robot_id=kh.getRobotID();
        kh.insertOrUpdateSwarm(swarm_id_, 0);
        
        SingleRobotLeaveSwarm srls(robot_id, swarm_id_);
        
        std::ostringstream archiveStream;
        boost::archive::text_oarchive archive(archiveStream);
        archive<<srls;
        std::string srjs_str=archiveStream.str();   
                      
        micros_swarm_framework::MSFPPacket p;
        p.packet_source=robot_id;
        p.packet_version=1;
        p.packet_type=SINGLE_ROBOT_LEAVE_SWARM;
        p.packet_data=srjs_str;
        p.package_check_sum=0;
                
        kh.publishPacket(p);
    }
    
    void Swarm::selectSwarm(boost::function<bool()> bf)
    {
        if(bf())
        {
           joinSwarm();
        }
        else
        {
            //do nothiong
        }
    }
            
    void Swarm::unselectSwarm(boost::function<bool()> bf)
    {
        if(bf())
        {
           leaveSwarm();
        }
        else
        {
            //do nothiong
        }
    }
            
    bool Swarm::inSwarm()
    {
        micros_swarm_framework::KernelHandle kh;
        if(kh.getSwarm(swarm_id_))
            return true;
        return false;
    }
    
    void Swarm::execute(boost::function<void()> f)
    {
        if(inSwarm())
            f();
    }
    
    /*
    void Swarm::breakupSwarm()
    {
        micros_swarm_framework::KernelHandle kh;
        kh.deleteSwarm(swarm_id_);
        this->~Swarm();
    }
    */
    
    Swarm Swarm::intersectionSwarm(Swarm s, unsigned int new_swarm_id)
    {
        std::set<unsigned int> result;
        
        std::set<unsigned int> a = getSwarmMembers();
        std::set<unsigned int> b = s.getSwarmMembers();
        
        std::set_intersection(a.begin(), a.end(), b.begin(), b.end(),
            std::insert_iterator<std::set<unsigned int> >(result, result.begin()));
    
        Swarm result_swarm(new_swarm_id);
    
        micros_swarm_framework::KernelHandle kh;
        unsigned int robot_id=kh.getRobotID();
    
        std::set<unsigned int>::iterator it;  
        it = result.find(robot_id);
        if(it != result.end())
        {
            result_swarm.joinSwarm();
        }
        
        ros::Duration(0.1).sleep();
        
        return result_swarm;
    }
            
    Swarm Swarm::unionSwarm(Swarm s, unsigned int new_swarm_id)
    {
        std::set<unsigned int> result;
        
        std::set<unsigned int> a = getSwarmMembers();
        std::set<unsigned int> b = s.getSwarmMembers();
        
        std::set_union(a.begin(), a.end(), b.begin(), b.end(),
            std::insert_iterator<std::set<unsigned int> >(result, result.begin()));
    
        Swarm result_swarm(new_swarm_id);
    
        micros_swarm_framework::KernelHandle kh;
        unsigned int robot_id=kh.getRobotID();
    
        std::set<unsigned int>::iterator it;  
        it = result.find(robot_id);
        if(it != result.end())
        {
            result_swarm.joinSwarm();
        }
        
        ros::Duration(0.1).sleep();
        
        return result_swarm;
    }
            
    Swarm Swarm::differenceSwarm(Swarm s, unsigned int new_swarm_id)
    {
        std::set<unsigned int> result;
        
        std::set<unsigned int> a = getSwarmMembers();
        std::set<unsigned int> b = s.getSwarmMembers();
        
        std::set_difference(a.begin(), a.end(), b.begin(), b.end(),
            std::insert_iterator<std::set<unsigned int> >(result, result.begin()));
    
        Swarm result_swarm(new_swarm_id);
    
        micros_swarm_framework::KernelHandle kh;
        unsigned int robot_id=kh.getRobotID();
    
        std::set<unsigned int>::iterator it;  
        it = result.find(robot_id);
        if(it != result.end())
        {
            result_swarm.joinSwarm();
        }
        
        ros::Duration(0.1).sleep();
        
        return result_swarm;
    }
            
    Swarm Swarm::negationSwarm(unsigned int new_swarm_id)
    {
        Swarm result_swarm(new_swarm_id);
    
        micros_swarm_framework::KernelHandle kh;
        unsigned int robot_id=kh.getRobotID();
    
        std::set<unsigned int>::iterator it;  
        it = getSwarmMembers().find(robot_id);
        if(it == getSwarmMembers().end())
        {
            result_swarm.joinSwarm();
        }
        
        ros::Duration(0.1).sleep();
        
        return result_swarm;
    }
};
#endif
