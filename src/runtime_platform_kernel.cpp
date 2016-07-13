/**
Software License Agreement (BSD)
\file      runtime_platform_kernel.cpp 
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
#include <nodelet/nodelet.h>
#include "std_msgs/String.h"
#include <pluginlib/class_list_macros.h>

#include "micros_swarm_framework/communication_interface.h"

namespace micros_swarm_framework{

    void packetParser(const MSFPPacket& packet)
    {
        MSFPPacket p=packet;
        std::cout<<p.packet_source<<", "<<p.packet_version<<", "<<p.packet_type<<", "<<\
            p.packet_data<<", "<<p.package_check_sum<<std::endl;
    }
    
    boost::shared_ptr<CommunicationInterface> communicator_;
    
    class RuntimePlatformKernel : public nodelet::Nodelet
    {
        public:
            ros::NodeHandle node_handle_;
            //boost::shared_ptr<CommunicationInterface> communicator_;
            ros::Timer timer_;
            
            RuntimePlatformKernel();
            ~RuntimePlatformKernel();
            void (*packetCallBack_)(const MSFPPacket& packet);
            void timerCallback(const ros::TimerEvent&);
            virtual void onInit();
    };

    RuntimePlatformKernel::RuntimePlatformKernel()
    {
        packetCallBack_=packetParser;
    }
    
    RuntimePlatformKernel::~RuntimePlatformKernel()
    {
    }
    
    void RuntimePlatformKernel::timerCallback(const ros::TimerEvent&)
    {
        MSFPPacket p;
        p.packet_source=1;
        p.packet_version=2;
        p.packet_type=3;
        p.packet_data="test";
        p.package_check_sum=19911203;
        
        communicator_->broadcast(p);
    }
    
    void RuntimePlatformKernel::onInit()
    {
        std::cout << "Initializing nodelet 1..." << std::endl;
        NODELET_DEBUG("Initializing nodelet 2...");
        RuntimePlatformKernel::node_handle_ = getPrivateNodeHandle();
    
        communicator_.reset(new ROSCommunication(node_handle_));
        communicator_->receive(packetCallBack_);
    
        //chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    
        //std::cout << "Initializing nodelet 3..." << std::endl;
        timer_ = node_handle_.createTimer(ros::Duration(0.1), &RuntimePlatformKernel::timerCallback, this);
        //std::cout << "Initializing nodelet 4..." << std::endl;
        
    }
};

// Register the nodelet
PLUGINLIB_EXPORT_CLASS(micros_swarm_framework::RuntimePlatformKernel, nodelet::Nodelet)

