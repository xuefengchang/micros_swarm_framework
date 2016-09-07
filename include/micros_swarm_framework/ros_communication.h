/**
Software License Agreement (BSD)
\file      ros_communication.h
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

#ifndef ROS_COMMUNICATION_H_
#define ROS_COMMUNICATION_H_

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

#include "micros_swarm_framework/communication_interface.h"
#include "micros_swarm_framework/MSFPPacket.h"

namespace micros_swarm_framework{
    
    class ROSCommunication : public CommunicationInterface{
        public:
            ROSCommunication()
            {
                name_="ERROR";
            }
        
            ROSCommunication(ros::NodeHandle node_handle)
            {
                name_="ROS";
                node_handle_=node_handle;
                packet_publisher_ = node_handle_.advertise<micros_swarm_framework::MSFPPacket>("/micros_swarm_framework_topic", 1000, true);
            }
            
            void broadcast(const MSFPPacket& msfp_packet)
            {
                static bool flag=false;
                if(!flag)
                {
                    ros::Duration(1).sleep();
                    if(!packet_publisher_)
                    {
                        ROS_INFO("ROS communicator could not initialize!");
                        exit(-1);
                    }
                    flag=true;
                }
        
                if(ros::ok())
                {
                    packet_publisher_.publish(msfp_packet);
                }
            }
            
            void callback(const MSFPPacket& packet)
            {
                parser_(packet);
            }
            
            void receive(boost::function<void(const MSFPPacket&)> parser)
            {
                parser_=parser;
                packet_subscriber_ = node_handle_.subscribe("/micros_swarm_framework_topic", 1000, &ROSCommunication::callback, this, ros::TransportHints().udp());
            }
            
        private:
            ros::NodeHandle node_handle_;
            ros::Publisher packet_publisher_;
            ros::Subscriber packet_subscriber_;
    };
};
#endif
