/**
Software License Agreement (BSD)
\file      app_manager.cpp
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

#include "micros_swarm/app_manager.h"

namespace micros_swarm{

    Worker::Worker(int id)
    {
        id_ = id;
        thread_ = new boost::thread(&Worker::workFunc, this);
        run_ = true;
        apps_.clear();
    }

    Worker::~Worker()
    {
        run_ = false;
        for(int i = 0; i < apps_.size(); i++) {
            apps_[i]->app_ptr_.reset();
            delete apps_[i];
        }
        apps_.clear();
        thread_->interrupt();
        thread_->join();
        delete thread_;
    }

    void Worker::addApp(AppInstance* app)
    {
        apps_.push_back(app);
    }

    void Worker::removeApp(const std::string& app_name)
    {
        std::vector<AppInstance*>::iterator app_it;
        for(app_it = apps_.begin(); app_it != apps_.end(); app_it++) {
            if((*app_it)->app_name_ == app_name) {
                //(*app_it)->app_ptr_->stop();
                (*app_it)->running_ = false;
                (*app_it)->app_ptr_.reset();
                delete (*app_it);
                app_it = apps_.erase(app_it);
                return;
            }
        }
    }

    int Worker::getAppNum()
    {
        return apps_.size();
    }

    AppInstance* Worker::getAppInstance(const std::string& app_name)
    {
        std::vector<AppInstance*>::iterator app_it;
        for(app_it = apps_.begin(); app_it != apps_.end(); app_it++) {
            if((*app_it)->app_name_ == app_name) {
                return (*app_it);
            }
        }

        AppInstance *app_ins = NULL;
        return app_ins;
    }

    void Worker::workFunc()
    {
        while(run_) {
            std::vector<AppInstance*>::iterator app_it;
            for(app_it = apps_.begin(); app_it != apps_.end(); app_it++) {
                if(!(*app_it)->running_) {
                    (*app_it)->app_ptr_->init();
                    (*app_it)->app_ptr_->start();
                    (*app_it)->running_ = true;
                }
            }
            ros::Duration(0.2).sleep();
        }
    }

    AppManager::AppManager():app_loader_("micros_swarm", "micros_swarm::Application")
    {
        ros::NodeHandle nh;
        app_load_srv_ = nh.advertiseService("app_loader_load_app", &AppManager::loadService, this);
        app_unload_srv_ = nh.advertiseService("app_loader_unload_app", &AppManager::unloadService, this);
        worker_num_ = 4;
        worker_table_.clear();
        load_table_.clear();
        for(int i = 0; i < worker_num_; i++) {
            Worker *worker = new Worker(i);
            worker_table_.push_back(worker);
            load_table_.push_back(0);
        }
        apps_record_.clear();
        plugin_use_count_.clear();
    }

    AppManager::AppManager(int worker_num):app_loader_("micros_swarm", "micros_swarm::Application")
    {
        ros::NodeHandle nh;
        app_load_srv_ = nh.advertiseService("app_loader_load_app", &AppManager::loadService, this);
        app_unload_srv_ = nh.advertiseService("app_loader_unload_app", &AppManager::unloadService, this);
        worker_num_ = worker_num;
        worker_table_.clear();
        load_table_.clear();
        for(int i = 0; i < worker_num_; i++) {
            Worker *worker = new Worker(i);
            worker_table_.push_back(worker);
            load_table_.push_back(0);
        }
        apps_record_.clear();
        plugin_use_count_.clear();
    }

    void AppManager::stop()
    {
        ros::NodeHandle nh;
        std::map<std::string, int>::iterator it;
        for(it = apps_record_.begin(); it != apps_record_.end(); it++) {
            std::string topic_name = "runtime_core_destroy_" + it->first;
            ros::ServiceClient client = nh.serviceClient<app_loader::RTDestroy>(topic_name);
            app_loader::RTDestroy srv;
            srv.request.code = 1;

            if (client.call(srv)) {
                //ROS_INFO("[RTPManager]: App %s unloaded successfully.", app_it->app_name_.c_str());
            }
            else {
                //ROS_ERROR("[RTPManager]: Failed to unload App %s.", app_it->app_name_.c_str());
            }
            ros::Duration(0.1).sleep();
        }

        for(it = apps_record_.begin(); it != apps_record_.end(); ) {
            int worker_id = it->second;
            AppInstance *instance = worker_table_[worker_id]->getAppInstance(it->first);
            if(instance != NULL) {
                instance->app_ptr_->stop();
                instance->app_ptr_.reset();
                std::string app_type = instance->app_type_;
                worker_table_[worker_id]->removeApp(it->first);
                load_table_[worker_id] -= 1;
                decreasePluginUseCount(app_type);
                if(getPluginUseCount(app_type) == 0) {
                    app_loader_.unloadLibraryForClass(app_type);
                }
            }

            apps_record_.erase(it++);
        }
        std::cout<<"[AppManager]: all Apps were unloaded successfully."<<std::endl;

        for(int i = 0; i < worker_table_.size(); i++) {
            delete worker_table_[i];
        }
        worker_table_.clear();
        std::cout<<"[AppManager]: destroy all the worker successfully."<<std::endl;
    }

    AppManager::~AppManager() {}

    bool AppManager::recordExist(const std::string& name)
    {
        std::map<std::string, int>::iterator it = apps_record_.find(name);

        if(it != apps_record_.end()) {
            return true;
        }

        return false;
    }

    void AppManager::addRecord(const std::string& name, int worker_id)
    {
        apps_record_.insert(std::pair<std::string, int>(name, worker_id));
    }

    void AppManager::removeRecord(const std::string& name)
    {
        apps_record_.erase(name);
    }

    int AppManager::allocateWorker()
    {
        int min_index = 0;
        for(int i = 1; i < worker_num_; i++) {
            if(load_table_[i] < load_table_[min_index]) {
                min_index = i;
            }
        }

        return min_index;
    }

    void AppManager::insertOrUpdatePluginUseCount(const std::string& type)
    {
        std::map<std::string, int>::iterator it;
        it = plugin_use_count_.find(type);
        if(it != plugin_use_count_.end()) {
            it->second += 1;
        }
        else {
            plugin_use_count_.insert(std::pair<std::string, int>(type, 1));
        }
    }

    void AppManager::decreasePluginUseCount(const std::string& type)
    {
        std::map<std::string, int>::iterator it;
        it = plugin_use_count_.find(type);
        if(it != plugin_use_count_.end()) {
            it->second -= 1;
            if(it->second == 0) {
                plugin_use_count_.erase(type);
            }
        }
    }

    int AppManager::getPluginUseCount(const std::string& type)
    {
        std::map<std::string, int>::iterator it;
        it = plugin_use_count_.find(type);
        if(it != plugin_use_count_.end()) {
            return plugin_use_count_[type];
        }

        return 0;
    }

    bool AppManager::loadService(app_loader::AppLoad::Request &req, app_loader::AppLoad::Response &resp)
    {
        std::string app_name = req.name;
        std::string app_type = req.type;

        bool app_exist = recordExist(app_name);
        if(app_exist) {
            ROS_WARN("[AppManager]: App %s was already existed.", app_name.c_str());
            ros::Duration(0.1).sleep();
            resp.success = false;
            return false;
        }
        else {
            boost::shared_ptr<micros_swarm::Application> app;
            try {
                app = app_loader_.createInstance(app_type);
            }
            catch(pluginlib::PluginlibException& ex) {
                ROS_ERROR("[AppManager]: App %s failed to load for some reason. Error: %s", app_name.c_str(), ex.what());
            }

            AppInstance* app_instance = new AppInstance();
            app_instance->app_name_ = app_name;
            app_instance->app_type_ = app_type;
            app_instance->app_ptr_ = app;
            app_instance->running_ = false;

            int worker_index = allocateWorker();
            worker_table_[worker_index]->addApp(app_instance);
            load_table_[worker_index] += 1;
            addRecord(app_name, worker_index);
            insertOrUpdatePluginUseCount(app_type);
            ROS_INFO("[AppManager]: App %s was loaded successfully.", app_name.c_str());
            ros::Duration(0.1).sleep();
            resp.success = true;
            return true;
        }
    }

    bool AppManager::unloadService(app_loader::AppUnload::Request &req, app_loader::AppUnload::Response &resp)
    {
        std::string app_name = req.name;
        std::string app_type = req.type;

        if(recordExist(app_name)) {
            int worker_id = apps_record_[app_name];
            AppInstance *instance = worker_table_[worker_id]->getAppInstance(app_name);
            if(instance == NULL) {
                ROS_WARN("[AppManager]: Get App instance %s failed.", app_name.c_str());
                ros::Duration(0.1).sleep();
                resp.success = false;
                return false;
            }
            instance->app_ptr_.reset();
            worker_table_[worker_id]->removeApp(app_name);
            load_table_[worker_id] -= 1;
            removeRecord(app_name);
            decreasePluginUseCount(app_type);
            if(getPluginUseCount(app_type) == 0) {
                app_loader_.unloadLibraryForClass(app_type);
            }
            ROS_INFO("[AppManager]: App %s was unloaded successfully.", app_name.c_str());
            ros::Duration(0.1).sleep();
            resp.success = true;
            return true;
        }
        ROS_WARN("[AppManager]: App %s does not exist.", app_name.c_str());
        ros::Duration(0.1).sleep();
        resp.success = false;
        return false;
    }
};
