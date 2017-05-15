/**
Software License Agreement (BSD)
\file      runtime_platform_manager.h
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

#ifndef RUNTIME_PLATFORM_MANAGER_H_
#define RUNTIME_PLATFORM_MANAGER_H_

#include <iostream>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>

#include "micros_swarm_framework/singleton.h"
#include "micros_swarm_framework/AppLoad.h"
#include "micros_swarm_framework/AppUnload.h"
#include "micros_swarm_framework/RTPDestroy.h"
#include "micros_swarm_framework/application.h"
#include "micros_swarm_framework/runtime_platform_core.h"

namespace micros_swarm_framework{
    struct AppInstance{
        std::string app_name_;
        std::string app_type_;
        boost::shared_ptr<Application> app_ptr_;
        bool running_;
    };

    class RTPManager
    {
        public:
            RTPManager();
            ~RTPManager();
            void shutdown();
        private:
            void startApp(const ros::TimerEvent&);
            bool loadService(micros_swarm_framework::AppLoad::Request &req, micros_swarm_framework::AppLoad::Response &resp);
            bool unloadService(micros_swarm_framework::AppUnload::Request &req, micros_swarm_framework::AppUnload::Response &resp);
            boost::shared_ptr<micros_swarm_framework::RuntimePlatformCore> rtp_core_;
            std::vector<AppInstance> apps_;
            pluginlib::ClassLoader<micros_swarm_framework::Application> app_loader_;
            ros::ServiceServer app_load_srv_, app_unload_srv_;
            ros::Timer start_app_timer_;
            //ros::Publisher rtp_manager_destroy_pub_;
            ros::ServiceClient client_;
    };
};

#endif
