/* 
 *  neighbor_communication.h - micros_swarm_framework neighbor communication
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

#ifndef NEIGHBOR_COMMUNICATION_H_
#define NEIGHBOR_COMMUNICATION_H_

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
    
    template<class Type>
    class NeighborCommunication{
        private:
            std::string key_;
            boost::function<void(Type)> f_;
            ros::Subscriber neighbor_packet_subscriber_;
        public:
        
            NeighborCommunication(std::string key)
            {
                key_ = key;
            }
            
            void neighborPacketCallback(const micros_swarm_framework::MSFPPacket& packet)
            {
                micros_swarm_framework::KernelHandle kh;
                unsigned int shm_rid=kh.getRobotID();
                //ignore the packet from local robot
                if(packet.packet_source==shm_rid)
                {
                    return;
                }
    
                const unsigned int packet_type=packet.packet_type;
                std::string packet_data=packet.packet_data;
        
                std::istringstream archiveStream(packet_data);
                boost::archive::text_iarchive archive(archiveStream);
        
                switch(packet_type)
                {
                    case NEIGHBOR_BROADCAST_KEY_VALUE:{
                        micros_swarm_framework::NeighborBroadcastKeyValue<Type> nbkv;
                        archive>>nbkv;
                        
                        std::string key=nbkv.getKey();
                        Type value=nbkv.getValue();
                        
                        if(key_==key)
                            f_(value);
                        else
                            std::cout<<"received wrong key."<<std::endl;
               
                        break;
                    }
                    default:
                    {
                        std::cout<<"UNDEFINED PACKET TYPE!"<<std::endl;
                    }
                }
            }
            
            void neighborBroadcast(Type value)
            {
                micros_swarm_framework::KernelHandle kh;
                std::string topic_name="/neighbor_topic_"+key_;
                
                
                ros::NodeHandle n;
                static ros::Publisher packet_publisher = n.advertise<micros_swarm_framework::MSFPPacket>(topic_name, 1000, true);
                static bool flag=false;
                
                if(!flag)
                {
                    ros::Duration(1).sleep();
                    if(!packet_publisher)
                    {
                        ROS_INFO("packet_publisher could not initialize.");
                    }
            
                    flag=true;
                }
                
                
                micros_swarm_framework::NeighborBroadcastKeyValue<Type> nbkv(key_, value);
                
                std::ostringstream archiveStream;
                boost::archive::text_oarchive archive(archiveStream);
                archive<<nbkv;
                std::string nbkv_str=archiveStream.str();  
                
                micros_swarm_framework::MSFPPacket p;
                p.packet_source=kh.getRobotID();
                p.packet_version=1;
                p.packet_type=NEIGHBOR_BROADCAST_KEY_VALUE;
                p.packet_data=nbkv_str;
                p.packet_ttl=1;
                p.package_check_sum=0;
                
                packet_publisher.publish(p);
            }
            
            void neighborListen(boost::function<void(Type)> f)
            {
                f_=f;
                ros::NodeHandle n;
                std::string topic_name="/neighbor_topic_"+key_;
                neighbor_packet_subscriber_ = n.subscribe(topic_name, 1000, &NeighborCommunication::neighborPacketCallback, this);
            }
            
            void neighborIgnore()
            {
                 neighbor_packet_subscriber_.shutdown();
            }
    };
};
#endif
