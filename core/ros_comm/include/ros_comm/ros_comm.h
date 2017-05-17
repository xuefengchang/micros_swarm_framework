/**
Software License Agreement (BSD)
\file      ros_comm.h
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

#ifndef ROS_COMM_H_
#define ROS_COMM_H_

#include <iostream>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

#include "micros_swarm/comm_interface.h"
#include "ros_comm/GSDFPacket.h"

namespace ros_comm{
    
    class ROSComm : public micros_swarm::CommInterface{
        public:
            ROSComm();
            void init(std::string name, boost::function<void(const micros_swarm::CommPacket& packet)> func);
            void broadcast(const micros_swarm::CommPacket& packet);
            void callback(const GSDFPacket& ros_msg);
            void receive();
        private:
            std::string name_;
            boost::function<void(const micros_swarm::CommPacket& packet)> parser_func_;
            ros::NodeHandle node_handle_;
            ros::Publisher packet_publisher_;
            ros::Subscriber packet_subscriber_;
    };
};
#endif
