/**
Software License Agreement (BSD)
\file      runtime_platform_manager.cpp
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

#include "micros_swarm/runtime_platform_manager.h"

namespace micros_swarm{

    RTPManager::RTPManager():app_loader_("micros_swarm", "micros_swarm::Application")
    {
        ros::NodeHandle nh;
        rtp_core_.reset(new micros_swarm::RuntimePlatformCore());
        rtp_core_->initialize();
        /*boost::thread rtp_core_thread([&] ()
                      {
                          rtp_core_->initialize();
                      });*/
        app_load_srv_ = nh.advertiseService("micros_swarm_framework_load_app", &RTPManager::loadService, this);
        app_unload_srv_ = nh.advertiseService("micros_swarm_framework_unload_app", &RTPManager::unloadService, this);
        apps_.clear();
        start_app_timer_=nh.createTimer(ros::Duration(1), &RTPManager::startApp, this);

        //rtp_manager_destroy_pub_ = nh.advertise<std_msgs::Int8>("micros_swarm_framework_rtp_manager_destroy", 1000);
    }

    void RTPManager::shutdown()
    {
        ros::NodeHandle nh;
        std::vector<AppInstance>::iterator app_it;
        for(app_it=apps_.begin(); app_it!=apps_.end(); app_it++)
        {
            std::string topic_name = "micros_swarm_framework_rtp_manager_destroy_" + app_it->app_name_;
            //ros::ServiceClient client = nh.serviceClient<micros_swarm_framework::RTPDestroy>("micros_swarm_framework_rtp_manager_destroy");
            ros::ServiceClient client = nh.serviceClient<app_loader::RTPDestroy>(topic_name);
            app_loader::RTPDestroy srv;
            srv.request.code = 1;

            if (client.call(srv))
            {
                //ROS_INFO("[RTPManager]: App %s unloaded successfully.", app_it->app_name_.c_str());
            }
            else
            {
                //ROS_ERROR("[RTPManager]: Failed to unload App %s.", app_it->app_name_.c_str());
            }
        }
    }

    RTPManager::~RTPManager()
    {
        std::vector<AppInstance>::iterator app_it;
        for(app_it=apps_.begin(); app_it!=apps_.end(); app_it++)
        {
            std::string app_type = app_it->app_type_;
            app_it->app_ptr_.reset();
            app_loader_.unloadLibraryForClass(app_type);
        }
        std::cout<<"[RTPManager]: all Apps were unloaded successfully."<<std::endl;
    }

    void RTPManager::startApp(const ros::TimerEvent &)
    {
        std::vector<AppInstance>::iterator app_it;
        for(app_it=apps_.begin(); app_it!=apps_.end(); app_it++)
        {
            if(!app_it->running_)
            {
                app_it->app_ptr_->start();
                app_it->running_=true;
            }
        }
    }

    bool RTPManager::loadService(app_loader::AppLoad::Request &req, app_loader::AppLoad::Response &resp)
    {
        std::string app_name = req.name;
        std::string app_type = req.type;

        boost::shared_ptr<micros_swarm::Application> app;

        //std::map<std::string, boost::shared_ptr<micros_swarm_framework::Application> >::iterator app_it = apps_.find(app_name);
        std::vector<AppInstance>::iterator app_it;
        bool app_exist=false;
        for(app_it=apps_.begin(); app_it!=apps_.end(); app_it++)
        {
            if(app_it->app_name_==app_name)
            {
                app_exist=true;
                break;
            }
        }

        if(app_exist)
        {
            ROS_WARN("[RTPManager]: App %s was already existed.", app_name.c_str());
            resp.success = false;
            return false;
        }
        else
        {
            try
            {
                app = app_loader_.createInstance(app_type);
            }
            catch(pluginlib::PluginlibException& ex)
            {
                ROS_ERROR("[RTPManager]: App %s failed to load for some reason. Error: %s", app_name.c_str(), ex.what());
            }

            AppInstance app_instance;
            app_instance.app_name_=app_name;
            app_instance.app_type_=app_type;
            app_instance.app_ptr_=app;
            app_instance.running_= false;
            apps_.push_back(app_instance);
            ROS_INFO("[RTPManager]: App %s was loaded successfully.", app_name.c_str());
            resp.success = true;
            return true;
        }

        /*if(app_it!=apps_.end())
        {
            ROS_WARN("App %s was already existed.", app_name.c_str());
            resp.success = false;
            return false;
        }
        else
        {
            try
            {
                app = app_loader_.createInstance(app_type);
            }
            catch(pluginlib::PluginlibException& ex)
            {
                ROS_ERROR("App %s failed to load for some reason. Error: %s", app_name, ex.what());
            }

            apps_.insert(std::pair<std::string, boost::shared_ptr<micros_swarm_framework::Application> >(app_name, app));
            ROS_INFO("App %s was loaded successfully.", app_name.c_str());
            resp.success = true;
            //start to run app
            app->start();
            return true;
        }*/
    }

    bool RTPManager::unloadService(app_loader::AppUnload::Request &req, app_loader::AppUnload::Response &resp)
    {
        std::string app_name = req.name;
        std::string app_type = req.type;
        //std::map<std::string, boost::shared_ptr<micros_swarm_framework::Application> >::iterator app_it = apps_.find(app_name);

        /*if(app_it!=apps_.end())
        {
            app_it->second.reset();
            apps_.erase(app_name);
            app_loader_.unloadLibraryForClass(app_type);
            ROS_INFO("App %s was unloaded successfully.", app_name.c_str());
            resp.success = true;
        }
        else
        {
            ROS_WARN("App %s does not exist.", app_name.c_str());
            resp.success = false;
        }*/

        std::vector<AppInstance>::iterator app_it;
        for(app_it=apps_.begin(); app_it!=apps_.end();)
        {
            if(app_it->app_name_==app_name)
            {
                //app_exist=true;
                //break;
                app_it->app_ptr_.reset();
                app_loader_.unloadLibraryForClass(app_type);
                app_it = apps_.erase(app_it);
                ROS_INFO("[RTPManager]: App %s was unloaded successfully.", app_name.c_str());
                resp.success = true;
                return true;
            }
            else
            {
                ++app_it;
            }
        }

        ROS_WARN("[RTPManager]: App %s does not exist.", app_name.c_str());
        resp.success = false;
        return false;
    }
};
