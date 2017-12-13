/**
Software License Agreement (BSD)
\file      testbb.cpp
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

#include "testbb/testbb.h"

// Register the application
PLUGINLIB_EXPORT_CLASS(testbb::TestBb, micros_swarm::Application)

namespace testbb{

    TestBb::TestBb() {}

    TestBb::~TestBb() {}

    void TestBb::init() {}

    void TestBb::stop() {}

    void TestBb::baseCallback(const nav_msgs::Odometry& lmsg)
    {
        static int msg_count = 0;
        float x=lmsg.pose.pose.position.x;
        float y=lmsg.pose.pose.position.y;

        float vx=lmsg.twist.twist.linear.x;
        float vy=lmsg.twist.twist.linear.y;

        micros_swarm::Base l(x, y, 0, vx, vy, 0, 1);
        set_base(l);

        msg_count++;
        if(msg_count >= 5) {
            std::cout<<"shutdown sub"<<std::endl;
            sub.shutdown();
        }
        //std::cout<<"<<<"<<x<<", "<<y<<">>>"<<std::endl;
    }
    
    void TestBb::loop_put(const ros::TimerEvent&)
    {   
        static int count=0;
        std::string robot_id_string="robot_"+boost::lexical_cast<std::string>(get_id());
        std_msgs::Int32 val;
        val.data = get_id()+count;
        bb.put(robot_id_string, val);
        count++;
    }

    void TestBb::loop_get(const ros::TimerEvent&)
    {
        std::string robot_id_string="robot_"+boost::lexical_cast<std::string>(get_id());
        std_msgs::Int32 val = bb.get(robot_id_string);
        std::cout<<robot_id_string<<": "<<val.data<<std::endl;
    }
    
    void TestBb::start()
    {
        ros::NodeHandle nh;
        sub = nh.subscribe("base_pose_ground_truth", 1000, &TestBb::baseCallback, this, ros::TransportHints().udp());
        ros::Duration(1).sleep();
        bb = micros_swarm::BlackBoard<std_msgs::Int32>(0,0);

        //test put
        //timer = nh.createTimer(ros::Duration(0.1), &TestBb::loop_put, this);

        //test get
        std::string robot_id_string="robot_" + boost::lexical_cast<std::string>(get_id());
        std_msgs::Int32 val;
        val.data = get_id();
        bb.put(robot_id_string, val);
        timer = nh.createTimer(ros::Duration(0.1), &TestBb::loop_get, this);
    }
};

