/**
Software License Agreement (BSD)
\file      ros_broker.cpp
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
#include <ros/ros.h>

#include "ros_broker/ros_broker.h"

PLUGINLIB_EXPORT_CLASS(ros_broker::ROSBroker, micros_swarm::CommInterface)

namespace ros_broker{

    ROSBroker::ROSBroker()
    {
        packet_publisher_ = node_handle_.advertise<ros_broker::GSDFPacket>("/micros_swarm_framework_topic", 2000, true);
    }

    void ROSBroker::init(std::string name, const micros_swarm::PacketParser& parser)
    {
        name_ = name;
        parser_ = parser;
    }
            
    void ROSBroker::broadcast(const std::vector<uint8_t>& msg_data)
    {
        static bool flag = false;
        if(!flag) {
            ros::Duration(1).sleep();
            if(!packet_publisher_) {
                ROS_INFO("ROS communicator could not initialize!");
                exit(-1);
            }
            flag=true;
        }
        
        if(ros::ok()) {
            ros_broker::GSDFPacket ros_msg;
            ros_msg.data = msg_data;
            packet_publisher_.publish(ros_msg);
        }
    }
            
    void ROSBroker::callback(const ros_broker::GSDFPacket& ros_msg)
    {
        parser_.parse(ros_msg.data);
    }

    void ROSBroker::receive()
    {
        packet_subscriber_ = node_handle_.subscribe("/micros_swarm_framework_topic", 1000, &ROSBroker::callback, this, ros::TransportHints().udp());
    }
};
