/**
Software License Agreement (BSD)
\file      runtime_core.cpp
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

#include "micros_swarm/runtime_core.h"

//#define PUBLISH_ROBOT_ID_DURATION 0.1
//#define PUBLISH_SWARM_LIST_DURATION 5

namespace micros_swarm{
    RuntimeCore::RuntimeCore():ci_loader_("micros_swarm", "micros_swarm::CommInterface")
    {
    }

    RuntimeCore::~RuntimeCore()
    {
        spin_thread_->interrupt();
        spin_thread_->join();
        delete spin_thread_;

        rth_.reset();
        parser_.reset();
        communicator_.reset();
        ci_loader_.unloadLibraryForClass(comm_type_);
    }
    
    void RuntimeCore::spin_msg_queue()
    {
        for(;;)
        {
            boost::unique_lock<boost::mutex> lock(rth_->getOutMsgQueue()->msg_queue_mutex);
    
            if(!rth_->getOutMsgQueue()->baseMsgQueueEmpty())
            {
                communicator_->broadcast(rth_->getOutMsgQueue()->baseMsgQueueFront());
                rth_->getOutMsgQueue()->popBaseMsgQueue();
            }
            if(!rth_->getOutMsgQueue()->ncMsgQueueEmpty())
            {
                communicator_->broadcast(rth_->getOutMsgQueue()->ncMsgQueueFront());
                rth_->getOutMsgQueue()->popNcMsgQueue();
            }
            if(!rth_->getOutMsgQueue()->swarmMsgQueueEmpty())
            {
                communicator_->broadcast(rth_->getOutMsgQueue()->swarmMsgQueueFront());
                rth_->getOutMsgQueue()->popSwarmMsgQueue();
            }
            if(!rth_->getOutMsgQueue()->vstigMsgQueueEmpty())
            {
                communicator_->broadcast(rth_->getOutMsgQueue()->vstigMsgQueueFront());
                rth_->getOutMsgQueue()->popVstigMsgQueue();
            }
            if(!rth_->getOutMsgQueue()->bbMsgQueueEmpty())
            {
                communicator_->broadcast(rth_->getOutMsgQueue()->bbMsgQueueFront());
                rth_->getOutMsgQueue()->popBbMsgQueue();
            }
            if(!rth_->getOutMsgQueue()->barrierMsgQueueEmpty())
            {
                communicator_->broadcast(rth_->getOutMsgQueue()->barrierMsgQueueFront());
                rth_->getOutMsgQueue()->popBarrierMsgQueue();
            }
            
            while(rth_->getOutMsgQueue()->baseMsgQueueEmpty()&&rth_->getOutMsgQueue()->swarmMsgQueueEmpty()&&
                  rth_->getOutMsgQueue()->vstigMsgQueueEmpty()&&rth_->getOutMsgQueue()->bbMsgQueueEmpty()&&
                  rth_->getOutMsgQueue()->ncMsgQueueEmpty()&&rth_->getOutMsgQueue()->barrierMsgQueueEmpty())
            {
                rth_->getOutMsgQueue()->msg_queue_condition.wait(lock);
            }
        }
    }
    
    void RuntimeCore::barrier_check(const ros::TimerEvent&)
    {
        int barrier_size=rth_->getBarrierSize();
        if(barrier_size>=total_robot_numbers_-1)
        {
            std::cout<<"robot "<<rth_->getRobotID()<<" runtime core start."<<std::endl;
            barrier_timer_.stop();
        }
                
        //barrier
        int robot_id=rth_->getRobotID();
    
        std::string syn="SYN";
        micros_swarm::Barrier_Syn bs(syn);
                
        std::ostringstream archiveStream;
        boost::archive::text_oarchive archive(archiveStream);
        archive<<bs;
        std::string bs_string=archiveStream.str();
    
        micros_swarm::CommPacket p;
        p.packet_source=robot_id;
        p.packet_version=1;
        p.packet_type=micros_swarm::BARRIER_SYN;
        p.packet_data=bs_string;
        p.package_check_sum=0;
        rth_->getOutMsgQueue()->pushBarrierMsgQueue(p);
    }
    
    void RuntimeCore::publish_robot_base(const ros::TimerEvent&)
    {
        int robot_id=rth_->getRobotID();
        
        const Base& l=rth_->getRobotBase();
        
        SingleRobotBroadcastBase srbb(robot_id, l.x, l.y, l.z, l.vx, l.vy, l.vz, l.valid);
        
        std::ostringstream archiveStream;
        boost::archive::text_oarchive archive(archiveStream);
        archive<<srbb;
        std::string srbb_str=archiveStream.str();
                      
        micros_swarm::CommPacket p;
        p.packet_source=robot_id;
        p.packet_version=1;
        p.packet_type=SINGLE_ROBOT_BROADCAST_BASE;
        p.packet_data=srbb_str;
        p.package_check_sum=0;
        
        rth_->getOutMsgQueue()->pushBaseMsgQueue(p);
    }
    
    void RuntimeCore::publish_swarm_list(const ros::TimerEvent&)
    {
        int robot_id=rth_->getRobotID();
        
        std::vector<int> swarm_list;
        swarm_list.clear();
        rth_->getSwarmList(swarm_list);
        
        SingleRobotSwarmList srsl(robot_id, swarm_list);
        
        std::ostringstream archiveStream;
        boost::archive::text_oarchive archive(archiveStream);
        archive<<srsl;
        std::string srsl_str=archiveStream.str();
                      
        micros_swarm::CommPacket p;
        p.packet_source=robot_id;
        p.packet_version=1;
        p.packet_type=SINGLE_ROBOT_SWARM_LIST;
        p.packet_data=srsl_str;
        p.package_check_sum=0;

        rth_->getOutMsgQueue()->pushSwarmMsgQueue(p);
    }
    
    void RuntimeCore::setParameters()
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

        param_ok =node_handle_.getParam("/comm_type", comm_type_);
        if(!param_ok)
        {
            std::cout<<"could not get parameter comm_type, use the default ros_comm."<<std::endl;
            comm_type_="ros_comm/ROSComm";
        }else{
            std::cout<<"rt comm_type: "<<comm_type_<<std::endl;
        }
    
        /*param_ok =node_handle_.getParam("unique_robot_id", robot_id_);
        if(!param_ok)
        {
            std::cout<<"could not get parameter unique_robot_id! use the default value."<<std::endl;
            robot_id_=0;
        }else{
            std::cout<<"unique_robot_id = "<<robot_id_<<std::endl;
        }*/
        ros::NodeHandle private_nh("~");
        private_nh.param("unique_robot_id", robot_id_, 0);
        std::cout<<"unique_robot_id = "<<robot_id_<<std::endl;
        //private_nh.param<std::string>("comm_type", comm_type_, "ros_comm/ROSComm");
        //private_nh.param<std::string>("comm_type", comm_type_, "opensplice_dds_comm/OpenSpliceDDSComm");
    }
    
    void RuntimeCore::initialize()
    {
        setParameters();
        //construct runtime platform
        rth_=Singleton<RuntimeHandle>::getSingleton(robot_id_);
        rth_->setNeighborDistance(default_neighbor_distance_);
        //construct communicator
        communicator_ = ci_loader_.createInstance(comm_type_);
        Singleton<CommInterface>::makeSingleton(communicator_);
        //construct packet parser
        parser_ = Singleton<PacketParser>::getSingleton();
        //parser_.reset(new PacketParser());
        boost::function<void(const micros_swarm::CommPacket& packet)> parser_func=boost::bind(&PacketParser::parser, parser_, _1);
        //transfer the parser function to the communicator
        communicator_->init(comm_type_, parser_func);
        communicator_->receive();
        app_manager_=Singleton<AppManager>::getSingleton();
        
        //boost::thread spin_thread(&RuntimePlatformCore::spin_msg_queue, this);
        spin_thread_ = new boost::thread(&RuntimeCore::spin_msg_queue, this);
        publish_robot_base_timer_ = node_handle_.createTimer(ros::Duration(publish_robot_base_duration_), &RuntimeCore::publish_robot_base, this);
        publish_swarm_list_timer_ = node_handle_.createTimer(ros::Duration(publish_swarm_list_duration_), &RuntimeCore::publish_swarm_list, this);
        barrier_timer_=node_handle_.createTimer(ros::Duration(1), &RuntimeCore::barrier_check, this);
    }
};


