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

namespace micros_swarm{
    RuntimeCore::RuntimeCore():ci_loader_("micros_swarm", "micros_swarm::CommInterface") {}
    RuntimeCore::~RuntimeCore() {}
    
    void RuntimeCore::spin_msg_queue()
    {
        for(;;) {
            std::vector<std::vector<uint8_t> > msg_vec;
            msg_queue_manager_->spinAllOutMsgQueueOnce(msg_vec);
            for(int i = 0; i < msg_vec.size(); i++) {
                communicator_->broadcast(msg_vec[i]);
            }
        }
    }
    
    void RuntimeCore::barrier_check(const ros::TimerEvent&)
    {
        int barrier_size = rth_->getBarrierSize();
        if(barrier_size >= total_robot_numbers_-1) {
            std::cout<<"robot "<<rth_->getRobotID()<<" daemon node start."<<std::endl;
            barrier_timer_.stop();
        }
                
        //barrier
        int robot_id = rth_->getRobotID();
    
        std::string syn="SYN";
        gsdf_msgs::BarrierSyn bs;
        bs.s = syn;
        std::vector<uint8_t> bs_vec = serialize_ros(bs);
    
        gsdf_msgs::CommPacket p;
        p.header.source = robot_id;
        p.header.type = micros_swarm::BARRIER_SYN;
        p.header.data_len = bs_vec.size();
        p.header.version = 1;
        p.header.checksum = 0;
        p.content.buf = bs_vec;
        std::vector<uint8_t> msg_data = serialize_ros(p);
        msg_queue_manager_->getOutMsgQueue("barrier")->push(msg_data);
    }
    
    void RuntimeCore::publish_robot_base(const ros::TimerEvent&)
    {
        int robot_id = rth_->getRobotID();
        const Base& l = rth_->getRobotBase();
        gsdf_msgs::RobotBase rb;
        rb.id = robot_id;
        rb.px = l.x;
        rb.py = l.y;
        rb.pz = l.z;
        rb.vx = l.vx;
        rb.vy = l.vy;
        rb.vz = l.vz;
        rb.theta = 0;
        rb.valid = l.valid;
        std::vector<uint8_t> rb_vec = serialize_ros(rb);
                      
        gsdf_msgs::CommPacket p;
        p.header.source = robot_id;
        p.header.type = SINGLE_ROBOT_BROADCAST_BASE;
        p.header.data_len = rb_vec.size();
        p.header.version = 1;
        p.header.checksum = 0;
        p.content.buf = rb_vec;
        std::vector<uint8_t> msg_data = serialize_ros(p);
        msg_queue_manager_->getOutMsgQueue("base")->push(msg_data);
    }
    
    void RuntimeCore::publish_swarm_list(const ros::TimerEvent&)
    {
        int robot_id = rth_->getRobotID();
        std::vector<int> swarm_list;
        swarm_list.clear();
        rth_->getSwarmList(swarm_list);
        
        gsdf_msgs::SwarmList sl;
        sl.robot_id = robot_id;
        sl.swarm_list = swarm_list;
        std::vector<uint8_t> sl_vec = serialize_ros(sl);
                      
        gsdf_msgs::CommPacket p;
        p.header.source = robot_id;
        p.header.type = SINGLE_ROBOT_SWARM_LIST;
        p.header.data_len = sl_vec.size();
        p.header.version = 1;
        p.header.checksum = 0;
        p.content.buf = sl_vec;
        std::vector<uint8_t> msg_data = serialize_ros(p);
        msg_queue_manager_->getOutMsgQueue("swarm")->push(msg_data);
    }
    
    void RuntimeCore::setParameters()
    {
        bool param_ok = node_handle_.getParam("/publish_robot_id_duration", publish_robot_base_duration_);
        if(!param_ok) {
            std::cout<<"could not get parameter publish_robot_id_duration! use the default value."<<std::endl;
            publish_robot_base_duration_ = 0.1;
        }
        else {
            std::cout<<"publish_robot_id_duration: "<<publish_robot_base_duration_<<std::endl;
        }
        
        param_ok = node_handle_.getParam("/publish_swarm_list_duration", publish_swarm_list_duration_);
        if(!param_ok) {
            std::cout<<"could not get parameter publish_swarm_list_duration! use the default value."<<std::endl;
            publish_swarm_list_duration_ = 5.0;
        }
        else {
            std::cout<<"publish_swarm_list_duration: "<<publish_swarm_list_duration_<<std::endl;
        }
        
        param_ok = node_handle_.getParam("/default_neighbor_distance", default_neighbor_distance_);
        if(!param_ok) {
            std::cout<<"could not get parameter default_neighbor_distance! use the default value."<<std::endl;
            default_neighbor_distance_ = 50;
        }
        else {
            std::cout<<"default_neighbor_distance: "<<default_neighbor_distance_<<std::endl;
        }
        
        param_ok = node_handle_.getParam("/total_robot_numbers", total_robot_numbers_);
        if(!param_ok) {
            std::cout<<"could not get parameter total_robot_numbers! use the default value."<<std::endl;
            total_robot_numbers_ = 1;
        }
        else {
            std::cout<<"total_robot_numbers: "<<total_robot_numbers_<<std::endl;
        }

        param_ok = node_handle_.getParam("/comm_type", comm_type_);
        if(!param_ok) {
            std::cout<<"could not get parameter comm_type, use the default ros_comm."<<std::endl;
            comm_type_ = "ros_comm/ROSComm";
        }
        else {
            std::cout<<"comm_type: "<<comm_type_<<std::endl;
        }

        ros::NodeHandle private_nh("~");
        private_nh.param("robot_id", robot_id_, 0);
        std::cout<<"robot_id: "<<robot_id_<<std::endl;
        private_nh.param("worker_num", worker_num_, 4);
        std::cout<<"worker_num: "<<worker_num_<<std::endl;
    }
    
    void RuntimeCore::initialize()
    {
        //srand(time(NULL) + robot_id_);
        srand(time(NULL) + (int)getpid());
        setParameters();
        //construct runtime platform
        rth_ = Singleton<RuntimeHandle>::getSingleton();
        rth_->setRobotID(robot_id_);
        rth_->setNeighborDistance(default_neighbor_distance_);
        //construct packet parser
        parser_ = Singleton<PacketParser>::getSingleton();
        //construct communicator
        communicator_ = ci_loader_.createInstance(comm_type_);
        communicator_->init(comm_type_, *parser_);
        communicator_->receive();
        //construct message queue manager
        msg_queue_manager_ = Singleton<MsgQueueManager>::getSingleton();
        msg_queue_manager_->createOutMsgQueue("base", 1000);
        msg_queue_manager_->createOutMsgQueue("swarm", 1000);
        msg_queue_manager_->createOutMsgQueue("vstig", 50000);
        msg_queue_manager_->createOutMsgQueue("bb", 10000);
        msg_queue_manager_->createOutMsgQueue("nc", 10000);
        msg_queue_manager_->createOutMsgQueue("barrier", 10000);
        msg_queue_manager_->createOutMsgQueue("scds_pso", 10000);
        //construct app manager
        app_manager_ = Singleton<AppManager>::getSingleton(worker_num_);
        //construct timers
        publish_robot_base_timer_ = node_handle_.createTimer(ros::Duration(publish_robot_base_duration_), &RuntimeCore::publish_robot_base, this);
        publish_swarm_list_timer_ = node_handle_.createTimer(ros::Duration(publish_swarm_list_duration_), &RuntimeCore::publish_swarm_list, this);
        //barrier_timer_=node_handle_.createTimer(ros::Duration(1), &RuntimeCore::barrier_check, this);
        spin_thread_ = new boost::thread(&RuntimeCore::spin_msg_queue, this);
        std::cout<<"robot "<<rth_->getRobotID()<<" daemon node start."<<std::endl;
    }

    void RuntimeCore::shutdown()
    {
        spin_thread_->interrupt();
        spin_thread_->join();
        delete spin_thread_;

        app_manager_->stop();
        Singleton<AppManager>::deleteSingleton();
        app_manager_.reset();
        Singleton<CommInterface>::deleteSingleton();
        communicator_.reset();
        ci_loader_.unloadLibraryForClass(comm_type_);
        Singleton<PacketParser>::deleteSingleton();
        parser_.reset();
        Singleton<RuntimeHandle>::deleteSingleton();
        rth_.reset();
    }
};