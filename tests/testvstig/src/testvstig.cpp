/**
Software License Agreement (BSD)
\file      testvstig.cpp 
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

#include "testvstig/testvstig.h"

// Register the application
PLUGINLIB_EXPORT_CLASS(testvstig::TestVstig, micros_swarm::Application)

namespace testvstig{

    TestVstig::TestVstig() {}
    
    TestVstig::~TestVstig() {}

    void TestVstig::init() {}

    void TestVstig::stop() {}
    
    void TestVstig::loop_puts(const ros::TimerEvent&)
    {
        std::string robot_id_string = "robot_"+boost::lexical_cast<std::string>(get_id());
        static int count = 1;
        std_msgs::Int32 pval;
        pval.data = get_id() + count;
        vs.puts(robot_id_string, pval);
        count++;
        std::cout<<robot_id_string<<": "<<vs.size()<<std::endl;
        //vs.print();
        /*static bool flag = true;
        if(flag) {
            if(vs.size() == 60) {
                std::cout<<get_id()<<" get all tuple"<<std::endl;
                flag = false;
            }
        }*/
    }

    void TestVstig::loop_gets(const ros::TimerEvent&)
    {
        //for(int j = 0; j < 5; j++) {
            std::string robot_id_string = "robot_" + boost::lexical_cast<std::string>(get_id());
            std_msgs::Int32 gval = vs.gets(robot_id_string);
            std::cout << robot_id_string << ": " << vs.size() << ", " << gval.data << std::endl;
        //}
    }

    void TestVstig::loop_put(const ros::TimerEvent&)
    {
        std::string robot_id_string = "robot_"+boost::lexical_cast<std::string>(get_id());
        static int count = 1;
        std_msgs::Int32 pval;
        pval.data = get_id() + count;
        vs.put(robot_id_string, pval);
        count++;
        std::cout<<robot_id_string<<": "<<vs.size()<<std::endl;
        //vs.print();
    }

    void TestVstig::loop_get(const ros::TimerEvent&)
    {
        //for(int j = 0; j < 5; j++) {
            std::string robot_id_string = "robot_" + boost::lexical_cast<std::string>(get_id());
            std_msgs::Int32 gval = vs.get(robot_id_string);
            std::cout << robot_id_string << ": " << vs.size() << ", " << gval.data << std::endl;
        //}
    }

    void TestVstig::not_loop_put(const ros::TimerEvent&)
    {
        static int count = 1;
        std_msgs::Int32 pval;
        pval.data = get_id() + count;

        std::string cur_string="robot_"+boost::lexical_cast<std::string>(get_id())+"/"+boost::lexical_cast<std::string>(count);
        std::string front_string="robot_"+boost::lexical_cast<std::string>(get_id())+"/"+boost::lexical_cast<std::string>(count-1);
        vs.put(cur_string, pval);
        //vs.remove(front_string);

        std::cout<<cur_string<<": "<<vs.size()<<std::endl;
        count++;
    }

    void TestVstig::not_loop_get(const ros::TimerEvent&)
    {
        //for(int j = 0; j < 5; j++) {
            int i = micros_swarm::random_int(0, 2047);
            std::string cur_string = "tuple/" + boost::lexical_cast<std::string>(i);
            std_msgs::Int32 gval = vs.get(cur_string);
            std::cout << cur_string << ": " << vs.size() << ", " << gval.data << std::endl;
        //}
    }

    void TestVstig::baseCallback(const nav_msgs::Odometry& lmsg)
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
    
    void TestVstig::start()
    {
        ros::NodeHandle nh;
        sub = nh.subscribe("base_pose_ground_truth", 1000, &TestVstig::baseCallback, this, ros::TransportHints().udp());
        //ros::Duration(1).sleep();
        //test virtual stigmergy
        vs=micros_swarm::VirtualStigmergy<std_msgs::Int32>(1);
        std_msgs::Int32 val;
        val.data = get_id();

        /*
         * loop write and read
         */
        //SCDS puts
        //std::string robot_id_string="robot_"+boost::lexical_cast<std::string>(get_id());
        //vs.puts(robot_id_string, val);
        //timer = nh.createTimer(ros::Duration(0.1), &TestVstig::loop_puts, this);

        //SCDS gets
        /*std::string robot_id_string="robot_"+boost::lexical_cast<std::string>(get_id());
        vs.put(robot_id_string, val);
        timer = nh.createTimer(ros::Duration(0.1), &TestVstig::loop_gets, this);*/

        //Vstig put
        /*std::string robot_id_string="robot_"+boost::lexical_cast<std::string>(get_id());
        vs.put(robot_id_string, val);
        timer = nh.createTimer(ros::Duration(0.1), &TestVstig::loop_put, this);*/

        //Vstig get
        /*std::string robot_id_string="robot_"+boost::lexical_cast<std::string>(get_id());
        vs.put(robot_id_string, val);
        timer = nh.createTimer(ros::Duration(0.1), &TestVstig::loop_get, this);*/

        /*
         * not loop write and read
         */
        //put
        /*std::string cur_string="robot_"+boost::lexical_cast<std::string>(get_id())+"/"+boost::lexical_cast<std::string>(0);
        vs.put(cur_string, val);
        timer = nh.createTimer(ros::Duration(0.5), &TestVstig::not_loop_put, this);*/

        //get
        for(int i = 0; i < 2048; i++){
            std::string cur_string="tuple/"+boost::lexical_cast<std::string>(i);
            std_msgs::Int32 pval;
            pval.data = i;
            vs.put(cur_string, pval);
        }
        ros::Duration(20).sleep();
        timer = nh.createTimer(ros::Duration(0.1), &TestVstig::not_loop_get, this);
    }
};

