/**
Software License Agreement (BSD)
\file      neighbor_communication.h 
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
