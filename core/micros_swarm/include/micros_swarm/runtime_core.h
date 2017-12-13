/**
Software License Agreement (BSD)
\file      runtime_core.h
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

#ifndef RUNTIME_CORE_H_
#define RUNTIME_CORE_H_

#include <iostream>
#include <unistd.h>
#include <ros/ros.h>
#include <pluginlib/class_loader.h>

#include "micros_swarm/singleton.h"
#include "micros_swarm/packet_type.h"
#include "micros_swarm/serialize.h"
#include "micros_swarm/runtime_handle.h"
#include "micros_swarm/comm_interface.h"
#include "micros_swarm/msg_queue_manager.h"
#include "micros_swarm/packet_parser.h"
#include "micros_swarm/app_manager.h"

namespace micros_swarm{

    class RuntimeCore
    {
    public:
        ros::NodeHandle node_handle_;
        boost::shared_ptr<RuntimeHandle> rth_;
        boost::shared_ptr<CommInterface> communicator_;
        std::string comm_type_;
        pluginlib::ClassLoader<micros_swarm::CommInterface> ci_loader_;
        boost::shared_ptr<micros_swarm::MsgQueueManager> msg_queue_manager_;
        boost::shared_ptr<micros_swarm::PacketParser> parser_;
        boost::shared_ptr<AppManager> app_manager_;

        ros::Timer publish_robot_base_timer_;
        ros::Timer publish_swarm_list_timer_;
        ros::Timer barrier_timer_;
        ros::Timer spin_timer_;

        double publish_robot_base_duration_;
        double publish_swarm_list_duration_;
        double default_neighbor_distance_;
        int total_robot_numbers_;
        int robot_id_;
        int worker_num_;

        boost::thread* spin_thread_;

        RuntimeCore();
        ~RuntimeCore();
        void initialize();
        void shutdown();
        void setParameters();
        void spin_msg_queue();
        void publish_robot_base(const ros::TimerEvent&);
        void publish_swarm_list(const ros::TimerEvent&);
        void barrier_check(const ros::TimerEvent&);
    };
};

#endif
