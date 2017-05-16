/**
Software License Agreement (BSD)
\file      ros_comm.cpp
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

#include "micros_swarm/ros_comm.h"

PLUGINLIB_EXPORT_CLASS(micros_swarm::ROSComm, micros_swarm::CommInterface)

namespace micros_swarm{

    ROSComm::ROSComm()
    {
        packet_publisher_ = node_handle_.advertise<micros_swarm::MSFPPacket>("/micros_swarm_framework_topic", 2000, true);
    }

    void ROSComm::init(std::string name, boost::function<void(const CommPacket& packet)> func)
    {
        name_=name;
        parser_func_=func;
    }
            
    void ROSComm::broadcast(const CommPacket& packet)
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
            micros_swarm::MSFPPacket ros_msg;
            ros_msg.packet_source=packet.packet_source;
            ros_msg.packet_version=packet.packet_version;
            ros_msg.packet_type=packet.packet_type;
            ros_msg.packet_data=packet.packet_data;
            ros_msg.package_check_sum=packet.package_check_sum;
            packet_publisher_.publish(ros_msg);
        }
    }
            
    void ROSComm::callback(const MSFPPacket& ros_msg)
    {
        CommPacket packet;
        packet.packet_source=ros_msg.packet_source;
        packet.packet_version=ros_msg.packet_version;
        packet.packet_type=ros_msg.packet_type;
        packet.packet_data=ros_msg.packet_data;
        packet.package_check_sum=ros_msg.package_check_sum;

        parser_func_(packet);
    }

    void ROSComm::receive()
    {
        packet_subscriber_ = node_handle_.subscribe("/micros_swarm_framework_topic", 1000, &ROSComm::callback, this, ros::TransportHints().udp());
    }
};
