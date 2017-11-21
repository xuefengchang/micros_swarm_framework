/**
Software License Agreement (BSD)
\file      app_loader.cpp
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

#include "app_loader/app_loader.h"

namespace app_loader{

    AppLoader::AppLoader()
    {
        ros::NodeHandle private_nh("~");
        ros::NodeHandle nh;

        rtp_manager_destroy_ = false;

        //get app name and type
        private_nh.param("app_name", app_name_, std::string("app_demo"));
        private_nh.param("app_type", app_type_, std::string("micros_swarm/AppDemo"));

        ros::ServiceClient client = nh.serviceClient<app_loader::AppLoad>("app_loader_load_app");
        app_loader::AppLoad srv;
        srv.request.name = app_name_;
        srv.request.type = app_type_;
        if (client.call(srv))
        {
            ROS_INFO("[AppLoader]: App %s loaded successfully.", app_name_.c_str());
        }
        else
        {
            ROS_ERROR("[AppLoader]: Failed to load App %s.", app_name_.c_str());
        }

        //when the rtp manager was destroyed, automatically unload the apps
        std::string topic_name = "runtime_core_destroy_" + app_name_;
        rtp_manager_destroy_srv_ = nh.advertiseService(topic_name, &AppLoader::rtpManagerDestroyCB, this);
    }

    AppLoader::~AppLoader()
    {
        rtp_manager_destroy_srv_.shutdown();
        if(!rtp_manager_destroy_) {
            ros::NodeHandle nh;
            ros::ServiceClient client = nh.serviceClient<app_loader::AppUnload>("app_loader_unload_app");
            app_loader::AppUnload srv;
            srv.request.name = app_name_;
            srv.request.type = app_type_;

            if (client.call(srv)) {
                ROS_INFO("[AppLoader]: App %s was unloaded successfully.", app_name_.c_str());
            } else {
                ROS_ERROR("[AppLoader]: Failed to unload App %s.", app_name_.c_str());
            }
        }
        else{
            ROS_INFO("RTPManager was destroyed before the AppLoader.");
        }
    }

    bool AppLoader::rtpManagerDestroyCB(app_loader::RTDestroy::Request &req, app_loader::RTDestroy::Response &resp)
    {
        resp.success = false;
        if(req.code == 1) {
            rtp_manager_destroy_ = true;
            resp.success = true;
        }

        ros::shutdown();
        return true;
    }

    bool AppLoader::ok()
    {
        return !rtp_manager_destroy_;
    }
};


