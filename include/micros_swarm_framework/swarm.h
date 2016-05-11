/* 
 *  swarm.h - micros_swarm_framework swarm
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
            void selectSwarm(BOOL_FUNCTION_OBJECT bf);
            void unselectSwarm(BOOL_FUNCTION_OBJECT bf);
            bool inSwarm();
            //execute a function
            void execute(FUNCTION_OBJECT f);
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
        p.packet_ttl=1;
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
        p.packet_ttl=1;
        p.package_check_sum=0;
                
        kh.publishPacket(p);
    }
    
    void Swarm::selectSwarm(BOOL_FUNCTION_OBJECT bf)
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
            
    void Swarm::unselectSwarm(BOOL_FUNCTION_OBJECT bf)
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
    
    void Swarm::execute(FUNCTION_OBJECT f)
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
