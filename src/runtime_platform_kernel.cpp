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
#include <string>
#include <time.h>
#include <stdlib.h>
#include <limits.h>
#include <vector>
#include <stack>
#include <map>
#include <set>
#include <queue>
#include <algorithm>
#include <functional>
#include <sstream>
#include <fstream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/string.hpp> 
#include <boost/serialization/vector.hpp>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "micros_swarm_framework/singleton.h"
#include "micros_swarm_framework/message.h"
#include "micros_swarm_framework/runtime_platform.h"
#include "micros_swarm_framework/communication_interface.h"
#ifdef ROS
#include "micros_swarm_framework/ros_communication.h"
#endif
#ifdef OPENSPLICE_DDS
#include "micros_swarm_framework/opensplice_dds_communication.h"
#endif
#include "micros_swarm_framework/packet_parser.h"

//#define PUBLISH_ROBOT_ID_DURATION 0.1
//#define PUBLISH_SWARM_LIST_DURATION 5

namespace micros_swarm_framework{
    
    class RuntimePlatformKernel : public nodelet::Nodelet
    {
        public:
            ros::NodeHandle node_handle_;
            boost::shared_ptr<RuntimePlatform> rtp_;
            boost::shared_ptr<CommunicationInterface> communicator_;
            boost::shared_ptr<PacketParser> parser_;
            
            ros::Timer publish_robot_base_timer_;
            ros::Timer publish_swarm_list_timer_;
            ros::Timer barrier_timer_;
            
            float publish_robot_base_duration_;
            float publish_swarm_list_duration_;
            float default_neighbor_distance_;
            int total_robot_numbers_;
            int robot_id_;
            
            RuntimePlatformKernel();
            ~RuntimePlatformKernel();
            virtual void onInit();
            void setParameters();
            void publish_robot_base(const ros::TimerEvent&);
            void publish_swarm_list(const ros::TimerEvent&);
            void barrier_check(const ros::TimerEvent&);
    };

    RuntimePlatformKernel::RuntimePlatformKernel()
    {
    }
    
    RuntimePlatformKernel::~RuntimePlatformKernel()
    {
    }
    
    void RuntimePlatformKernel::barrier_check(const ros::TimerEvent&)
    {
        int barrier_size=rtp_->getBarrierSize();
        if(barrier_size>=total_robot_numbers_-1)
        {
            std::cout<<"robot "<<rtp_->getRobotID()<<" runtime_platform_kernel started successfully."<<std::endl;    
            barrier_timer_.stop();
        }
                
        //barrier
        int robot_id=rtp_->getRobotID();
    
        std::string syn="SYN";
        micros_swarm_framework::Barrier_Syn bs(syn);
                
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
    
    void RuntimePlatformKernel::publish_robot_base(const ros::TimerEvent&)
    {
        int robot_id=rtp_->getRobotID();
        
        const Base& l=rtp_->getRobotBase();
        
        SingleRobotBroadcastBase srbb(robot_id, l.x, l.y, l.z, l.vx, l.vy, l.vz, l.valid);
        
        std::ostringstream archiveStream;
        boost::archive::text_oarchive archive(archiveStream);
        archive<<srbb;
        std::string srbb_str=archiveStream.str();
                      
        micros_swarm_framework::MSFPPacket p;
        p.packet_source=robot_id;
        p.packet_version=1;
        p.packet_type=SINGLE_ROBOT_BROADCAST_BASE;
        #ifdef ROS
        p.packet_data=srbb_str;
        #endif
        #ifdef OPENSPLICE_DDS
        p.packet_data=srbb_str.data();
        #endif
        p.package_check_sum=0;

        communicator_->broadcast(p);
    }
    
    void RuntimePlatformKernel::publish_swarm_list(const ros::TimerEvent&)
    {
        int robot_id=rtp_->getRobotID();
        
        std::vector<int> swarm_list;
        swarm_list.clear();
        rtp_->getSwarmList(swarm_list);
        
        SingleRobotSwarmList srsl(robot_id, swarm_list);
        
        std::ostringstream archiveStream;
        boost::archive::text_oarchive archive(archiveStream);
        archive<<srsl;
        std::string srsl_str=archiveStream.str();
                      
        micros_swarm_framework::MSFPPacket p;
        p.packet_source=robot_id;
        p.packet_version=1;
        p.packet_type=SINGLE_ROBOT_SWARM_LIST;
        #ifdef ROS
        p.packet_data=srsl_str;
        #endif
        #ifdef OPENSPLICE_DDS
        p.packet_data=srsl_str.data();
        #endif
        p.package_check_sum=0;

        communicator_->broadcast(p);
    }
    
    void RuntimePlatformKernel::setParameters()
    {
        bool param_ok =node_handle_.getParam("/publish_robot_id_duration", publish_robot_base_duration_);
        if(!param_ok)
        {
            std::cout<<"could not get parameter publish_robot_id_duration! use the default value."<<std::endl;
            publish_robot_base_duration_=0.1;
        }else{
            std::cout<<"publish_robot_id_duration = "<<publish_robot_base_duration_<<std::endl;
        }
        
        param_ok =node_handle_.getParam("/publish_swarm_list_duration", publish_swarm_list_duration_);
        if(!param_ok)
        {
            std::cout<<"could not get parameter publish_swarm_list_duration! use the default value."<<std::endl;
            publish_swarm_list_duration_=5.0;
        }else{
            std::cout<<"publish_swarm_list_duration = "<<publish_swarm_list_duration_<<std::endl;
        }
        
        param_ok =node_handle_.getParam("/default_neighbor_distance", default_neighbor_distance_);
        if(!param_ok)
        {
            std::cout<<"could not get parameter default_neighbor_distance! use the default value."<<std::endl;
            default_neighbor_distance_=50;
        }else{
            std::cout<<"default_neighbor_distance = "<<default_neighbor_distance_<<std::endl;
        }
        
        param_ok =node_handle_.getParam("/total_robot_numbers", total_robot_numbers_);
        if(!param_ok)
        {
            std::cout<<"could not get parameter total_robot_numbers! use the default value."<<std::endl;
            total_robot_numbers_=1;
        }else{
            std::cout<<"total_robot_numbers = "<<total_robot_numbers_<<std::endl;
        }
    
        param_ok =node_handle_.getParam("unique_robot_id", robot_id_);
        if(!param_ok)
        {
            std::cout<<"could not get parameter unique_robot_id! use the default value."<<std::endl;
            robot_id_=0;
        }else{
            std::cout<<"unique_robot_id = "<<robot_id_<<std::endl;
        }
    }
    
    void RuntimePlatformKernel::onInit()
    {
        RuntimePlatformKernel::node_handle_ = getPrivateNodeHandle();
        
        setParameters();
    
        //construct runtime platform
        rtp_=Singleton<RuntimePlatform>::getSingleton(robot_id_);
        rtp_->setNeighborDistance(default_neighbor_distance_);
        //construct communicator
        #ifdef ROS
        communicator_=Singleton<ROSCommunication>::getSingleton(node_handle_);
        #endif
        #ifdef OPENSPLICE_DDS
        communicator_=Singleton<OpenSpliceDDSCommunication>::getSingleton();
        #endif
        //construct packet parser
        parser_.reset(new PacketParser());
        boost::function<void(const MSFPPacket& packet)> parser_func=boost::bind(&PacketParser::parser, parser_, _1);
        //transfer the parser function to the communicator 
        communicator_->receive(parser_func);
        
        publish_robot_base_timer_ = node_handle_.createTimer(ros::Duration(publish_robot_base_duration_), &RuntimePlatformKernel::publish_robot_base, this);
        publish_swarm_list_timer_ = node_handle_.createTimer(ros::Duration(publish_swarm_list_duration_), &RuntimePlatformKernel::publish_swarm_list, this);
        barrier_timer_=node_handle_.createTimer(ros::Duration(1), &RuntimePlatformKernel::barrier_check, this);
    }
};

// Register the nodelet
PLUGINLIB_EXPORT_CLASS(micros_swarm_framework::RuntimePlatformKernel, nodelet::Nodelet)

