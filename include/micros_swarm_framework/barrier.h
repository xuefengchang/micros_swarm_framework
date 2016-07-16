/**
Software License Agreement (BSD)
\file      barrier.h 
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

#ifndef BARRIER_H_
#define BARRIER_H_

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

#include "micros_swarm_framework/runtime_platform.h"
#include "micros_swarm_framework/communication_interface.h"
#include "micros_swarm_framework/ros_communication.h"

namespace micros_swarm_framework{
    
    class Barrier{
        private:
            boost::shared_ptr<RuntimePlatform> rtp_;
            boost::shared_ptr<CommunicationInterface> communicator_;
            ros::Timer barrier_timer_;
            int num_;
        public:
            Barrier(ros::NodeHandle nh, int num);
            ~Barrier();
            void check(const ros::TimerEvent&);
    };
    
    void Barrier::check(const ros::TimerEvent&)
    {
        int barrier_size=rtp_->getBarrierSize();
        if(barrier_size>=num_-1)
        {
            std::cout<<"The micros_swarm_framework_kernel started successfully."<<std::endl;
            std::cout<<"local robot id is: "<<rtp_->getRobotID()<<std::endl;
            
            barrier_timer_.stop();
        }
                
        //barrier
        int robot_id=rtp_->getRobotID();
    
        micros_swarm_framework::Barrier_Syn bs("SYN");
                
        std::ostringstream archiveStream;
        boost::archive::text_oarchive archive(archiveStream);
        archive<<bs;
        std::string bs_string=archiveStream.str();
    
        micros_swarm_framework::MSFPPacket p;
        p.packet_source=robot_id;
        p.packet_version=1;
        p.packet_type=micros_swarm_framework::BARRIER_SYN;
        #ifdef ROS
        p.packet_data=bs_string;
        #endif
        #ifdef OPENSPLICE_DDS
        p.packet_data=bs_string.data();
        #endif
        p.package_check_sum=0;
    
        communicator_->broadcast(p);
    }
    
    Barrier::Barrier(ros::NodeHandle nh, int num)
    {
        num_=num;
        barrier_timer_=nh.createTimer(ros::Duration(1), &Barrier::check, this);
    }
    
    Barrier::~Barrier()
    {
    
    }
};
#endif
