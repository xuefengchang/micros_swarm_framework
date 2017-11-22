/**
Software License Agreement (BSD)
\file      app_manager.h
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

#ifndef APP_MANAGER_H_
#define APP_MANAGER_H_

#include <iostream>
#include <vector>
#include <map>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>

#include "micros_swarm/application.h"
#include "app_loader/AppLoad.h"
#include "app_loader/AppUnload.h"
#include "app_loader/RTDestroy.h"

namespace micros_swarm{
    struct AppInstance{
        std::string app_name_;
        std::string app_type_;
        boost::shared_ptr<Application> app_ptr_;
        bool running_;
        uint8_t status_;
    };

    struct Worker{
    public:
        Worker(int id);
        ~Worker();
        void addApp(AppInstance* app);
        void removeApp(const std::string& app_name);
        int getAppNum();
        AppInstance* getAppInstance(const std::string& app_name);
    private:
        void workFunc();
        int id_;
        volatile bool run_;
        boost::thread* thread_;
        std::vector<AppInstance*> apps_;
    };

    class AppManager
    {
    public:
        AppManager();
        AppManager(int worker_num);
        ~AppManager();
        void stop();
    private:
        bool recordExist(const std::string& name);
        void addRecord(const std::string& name, int worker_id);
        void removeRecord(const std::string& name);
        int allocateWorker();
        void insertOrUpdatePluginUseCount(const std::string& type);
        void decreasePluginUseCount(const std::string& type);
        int getPluginUseCount(const std::string& type);
        bool loadService(app_loader::AppLoad::Request &req, app_loader::AppLoad::Response &resp);
        bool unloadService(app_loader::AppUnload::Request &req, app_loader::AppUnload::Response &resp);
        pluginlib::ClassLoader<micros_swarm::Application> app_loader_;
        ros::ServiceServer app_load_srv_, app_unload_srv_;
        ros::ServiceClient client_;
        int worker_num_;
        std::vector<Worker*> worker_table_;
        std::vector<uint16_t> load_table_;
        std::map<std::string, int> apps_record_;
        std::map<std::string, int> plugin_use_count_;
    };
};

#endif
