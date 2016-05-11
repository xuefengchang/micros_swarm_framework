/* 
 *  message.h - micros_swarm_framework message
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
        SINGLE_ROBOT_BROADCAST_ID,  //broadcast id
        
        SINGLE_ROBOT_JOIN_SWARM,  //robot join in a swarm
        SINGLE_ROBOT_LEAVE_SWARM,  //robot leave a swarm
        SINGLE_ROBOT_SWARM_LIST,  //broadcast swarm list
        
        VIRTUAL_STIGMERGY_QUERY,  //query a value of virtual stigmergy
        VIRTUAL_STIGMERGY_PUT,  //put a value in virtual stigmergy
        
        NEIGHBOR_BROADCAST_KEY_VALUE,  //broadcast <key, value> tuple
        
        MSFP_PACKET_TYPE_COUNT  //MSFPPacket type count
    };

    class SingleRobotBroadcastID{
        private:   
            unsigned int robot_id_;
            
            float robot_x_;
            float robot_y_;
            float robot_z_;
            
            //using boost to serialize
            friend class boost::serialization::access;
            template<class Archive>
            void serialize(Archive & ar, const unsigned int version)
            {
                ar & robot_id_;
                ar & robot_x_;
                ar & robot_y_;
                ar & robot_z_;
            }
        public:
            SingleRobotBroadcastID(){}
            SingleRobotBroadcastID(unsigned int robot_id, float robot_x ,float robot_y, float robot_z):\
                 robot_id_(robot_id), robot_x_(robot_x), robot_y_(robot_y), robot_z_(robot_z){}
                 
            unsigned int getRobotID(){return robot_id_;}
            float getRobotX(){return robot_x_;}
            float getRobotY(){return robot_y_;}
            float getRobotZ(){return robot_z_;}
        
    };
    
    class SingleRobotJoinSwarm{
        private:
            unsigned int robot_id_;
            unsigned int swarm_id_;
            
            friend class boost::serialization::access;
            template<class Archive>
            void serialize(Archive & ar, const unsigned int version)
            {
                ar & robot_id_;
                ar & swarm_id_;
            }
        public:
            SingleRobotJoinSwarm(){}
            SingleRobotJoinSwarm(unsigned int robot_id, unsigned int swarm_id):\
                robot_id_(robot_id), swarm_id_(swarm_id){}
                
            unsigned int getRobotID(){return robot_id_;}
            unsigned int getSwarmID(){return swarm_id_;}
    };
    
    class SingleRobotLeaveSwarm{
        private:
            unsigned int robot_id_;
            unsigned int swarm_id_;
            
            friend class boost::serialization::access;
            template<class Archive>
            void serialize(Archive & ar, const unsigned int version)
            {
                ar & robot_id_;
                ar & swarm_id_;
            }
        public:
            SingleRobotLeaveSwarm(){}
            SingleRobotLeaveSwarm(unsigned int robot_id, unsigned int swarm_id):\
                robot_id_(robot_id), swarm_id_(swarm_id){}
                
            unsigned int getRobotID(){return robot_id_;}
            unsigned int getSwarmID(){return swarm_id_;}
    };
    
    class SingleRobotSwarmList{
        private:
            unsigned int robot_id_;
            std::vector<unsigned int> swarm_list_;
            
            friend class boost::serialization::access;
            template<class Archive>
            void serialize(Archive & ar, const unsigned int version)
            {
                ar & robot_id_;
                ar & swarm_list_;
            }
        public:
            SingleRobotSwarmList(){}
            SingleRobotSwarmList(unsigned int robot_id, std::vector<unsigned int> swarm_list):\
                robot_id_(robot_id), swarm_list_(swarm_list){}
                
            unsigned int getRobotID(){return robot_id_;}
            std::vector<unsigned int> getSwarmList(){return swarm_list_;}
    };
    
    class TaskAssignSwarmTask{
        private:
            unsigned int swarm_task_id_;
            unsigned int swarm_id_;
            std::string task_data_link_name_;
            
            friend class boost::serialization::access;
            template<class Archive>
            void serialize(Archive & ar, const unsigned int version)
            {
                ar & swarm_task_id_;
                ar & swarm_id_;
                ar & task_data_link_name_;
            }
        public:
            TaskAssignSwarmTask()
            {
                swarm_task_id_=-1;
                swarm_id_=-1;
                task_data_link_name_="";
            }
            
            TaskAssignSwarmTask(unsigned int swarm_task_id, unsigned int swarm_id, std::string task_data_link_name):\
                swarm_task_id_(swarm_task_id), swarm_id_(swarm_id), task_data_link_name_(task_data_link_name){}
        
            unsigned int getSwarmTaskID(){return swarm_task_id_;}
            unsigned int getSwarmID(){return swarm_id_;}
            std::string getTaskDataLinkName(){return task_data_link_name_;}
            
    };
    
    class SingleRobotReceiveSwarmTask{
        private:
            unsigned int swarm_task_id_;
            unsigned int swarm_id_;
            
            friend class boost::serialization::access;
            template<class Archive>
            void serialize(Archive & ar, const unsigned int version)
            {
                ar & swarm_task_id_;
                ar & swarm_id_;
            }
        public:
            SingleRobotReceiveSwarmTask(){}
            SingleRobotReceiveSwarmTask(unsigned int swarm_task_id, unsigned int swarm_id):\
                swarm_task_id_(swarm_task_id), swarm_id_(swarm_id){}
                
            unsigned int getSwarmTaskID(){return swarm_task_id_;}
            unsigned int getSwarmID(){return swarm_id_;}
    
    };
    
    class TaskAssignSingleTask{
        private:
            unsigned int single_task_id_;
            unsigned int robot_id_;
            std::string task_data_link_name_;
            
            friend class boost::serialization::access;
            template<class Archive>
            void serialize(Archive & ar, const unsigned int version)
            {
                ar & single_task_id_;
                ar & robot_id_;
                ar & task_data_link_name_;
            }
        public:
            TaskAssignSingleTask(){}
            TaskAssignSingleTask(unsigned int single_task_id, unsigned int robot_id, std::string task_data_link_name):\
                single_task_id_(single_task_id), robot_id_(robot_id), task_data_link_name_(task_data_link_name){}
        
            unsigned int getSingleTaskID(){return single_task_id_;}
            unsigned int getRobotID(){return robot_id_;}
            std::string getTaskDataLinkName(){return task_data_link_name_;}
    };
    
    class SingleRobotReceiveSingleTask{
        private:
            unsigned int single_task_id_;
            unsigned int robot_id_;
            
            friend class boost::serialization::access;
            template<class Archive>
            void serialize(Archive & ar, const unsigned int version)
            {
                ar & single_task_id_;
                ar & robot_id_;
            }
        public:
            SingleRobotReceiveSingleTask(){}
            SingleRobotReceiveSingleTask(unsigned int single_task_id, unsigned int robot_id):\
                single_task_id_(single_task_id), robot_id_(robot_id){}
                
            unsigned int getSingleTaskID(){return single_task_id_;}
            unsigned int getRobotID(){return robot_id_;}
    };
    
    class VirtualStigmergyQuery
    {
        private:
            unsigned int virtual_stigmergy_id_;
            std::string virtual_stigmergy_key_;
            std::string virtual_stigmergy_value_;
            time_t virtual_stigmergy_timestamp_;
            unsigned int robot_id_;
            
            friend class boost::serialization::access;
            template<class Archive>
            void serialize(Archive & ar, const unsigned int version)
            {
                ar & virtual_stigmergy_id_;
                ar & virtual_stigmergy_key_;
                ar & virtual_stigmergy_value_;
                ar & virtual_stigmergy_timestamp_;
                ar & robot_id_;
            }
        public:
            VirtualStigmergyQuery(){}

            VirtualStigmergyQuery(unsigned int id, std::string key, std::string value, time_t time_now, unsigned int robot_id):\
                virtual_stigmergy_id_(id), virtual_stigmergy_key_(key), virtual_stigmergy_value_(value),\
                virtual_stigmergy_timestamp_(time_now), robot_id_(robot_id){}
                
            unsigned int getVirtualStigmergyID(){return virtual_stigmergy_id_;}
            std::string getVirtualStigmergyKey(){return virtual_stigmergy_key_;}
            std::string getVirtualStigmergyValue(){return virtual_stigmergy_value_;}
            time_t getVirtualStigmergyTimestamp(){return virtual_stigmergy_timestamp_;}
            unsigned int getRobotID(){return robot_id_;}
    };

    class VirtualStigmergyPut
    {
        private:
            unsigned int virtual_stigmergy_id_;
            std::string virtual_stigmergy_key_;
            std::string virtual_stigmergy_value_;
            time_t virtual_stigmergy_timestamp_;
            unsigned int robot_id_;
            
            friend class boost::serialization::access;
            template<class Archive>
            void serialize(Archive & ar, const unsigned int version)
            {
                ar & virtual_stigmergy_id_;
                ar & virtual_stigmergy_key_;
                ar & virtual_stigmergy_value_;
                ar & virtual_stigmergy_timestamp_;
                ar & robot_id_;
            }
        public:
            VirtualStigmergyPut(){};
            
             VirtualStigmergyPut(unsigned int id, std::string key, std::string value, time_t time_now, unsigned int robot_id):\
                virtual_stigmergy_id_(id), virtual_stigmergy_key_(key), virtual_stigmergy_value_(value),\
                virtual_stigmergy_timestamp_(time_now), robot_id_(robot_id){}
                
            unsigned int getVirtualStigmergyID(){return virtual_stigmergy_id_;}
            std::string getVirtualStigmergyKey(){return virtual_stigmergy_key_;}
            std::string getVirtualStigmergyValue(){return virtual_stigmergy_value_;}
            time_t getVirtualStigmergyTimestamp(){return virtual_stigmergy_timestamp_;}
            unsigned int getRobotID(){return robot_id_;}
    };
    
    template<class Type>
    class NeighborBroadcastKeyValue{
        private:   
            std::string key_;
            Type value_;
            
            friend class boost::serialization::access;
            template<class Archive>
            void serialize(Archive & ar, const unsigned int version)
            {
                ar & key_;
                ar & value_;
            }
        public:
            NeighborBroadcastKeyValue(){}
            NeighborBroadcastKeyValue(std::string key, Type value): key_(key), value_(value){}
                 
            std::string getKey(){return key_;}
            Type getValue(){return value_;}
    };
};
#endif
