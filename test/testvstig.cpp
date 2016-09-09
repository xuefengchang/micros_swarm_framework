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

#include "apps/testvstig.h"

namespace micros_swarm_framework{

    TestVstig::TestVstig(ros::NodeHandle node_handle):Application(node_handle)
    {
    }
    
    TestVstig::~TestVstig()
    {
    }
    
    void TestVstig::loop(const ros::TimerEvent&)
    {
        float tmp=0;
        //std::string robot_id_string=boost::lexical_cast<std::string>(getRobotID());
        //tmp=vs.get(robot_id_string);
        if(vs.size()<20)
        {
            std::cout<<"robot "<<robot_id()<<", vs :"<<tmp<<", size: "<<vs.size()<<std::endl;
            micros_swarm_framework::Neighbors<micros_swarm_framework::NeighborBase> n(true);
            n.print();
        }
        //vs.put(robot_id_string, tmp+0.01);
    }
    
    void TestVstig::baseCallback(const nav_msgs::Odometry& lmsg)
    {
        float x=lmsg.pose.pose.position.x;
        float y=lmsg.pose.pose.position.y;
    
        float vx=lmsg.twist.twist.linear.x;
        float vy=lmsg.twist.twist.linear.y;
    
        micros_swarm_framework::Base l(x, y, 0, vx, vy, 0);
        set_base(l);
    }
    
    void TestVstig::start()
    {
        set_neighbor_distance(1.2);
        sub = nh.subscribe("base_pose_ground_truth", 1000, &TestVstig::baseCallback, this, ros::TransportHints().udp());
        
        ros::Duration(5).sleep();  //TODO. this is necessary, in order that the runtime platform kernel of the robot has enough time to publish it's base information.
        
        //test virtual stigmergy
        vs=micros_swarm_framework::VirtualStigmergy<float>(1);
        std::string robot_id_string=boost::lexical_cast<std::string>(robot_id());
        vs.put(robot_id_string, 3.14);
        
        timer = nh.createTimer(ros::Duration(0.1), &TestVstig::loop, this);
    }
};

