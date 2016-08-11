/**
Software License Agreement (BSD)
\file      application.h
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

#ifndef APPLICATION_H_
#define APPLICATION_H_

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
#include <functional>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "micros_swarm_framework/runtime_platform.h"

namespace micros_swarm_framework{
    
    class Application
    {
        public:
            boost::shared_ptr<RuntimePlatform> rtp_;
            ros::NodeHandle nh_;
            
            Application()
            {
            
            }
            
            Application(ros::NodeHandle nh)
            {
                nh_=nh;
                rtp_=Singleton<RuntimePlatform>::getSingleton();
            }
            
            virtual ~Application()
            {
            
            }
            
            //application api
            int getRobotID()
            {
                return rtp_->getRobotID();
            }
            
            void setRobotID(int robot_id)
            {
                rtp_->setRobotID(robot_id);
            }
            
            Base getRobotBase()
            {
                return rtp_->getRobotBase();
            }
            
            void setRobotBase(Base robot_base)
            {
                rtp_->setRobotBase(robot_base);
            }
            
            double getNeighborDistance()
            {
                return rtp_->getNeighborDistance();
            }
            
            void setNeighborDistance(double neighbor_distance)
            {
                rtp_->setNeighborDistance(neighbor_distance);
            }
            
            //entry function
            virtual void start()=0;
    };
};

#endif
