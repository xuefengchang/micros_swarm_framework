/**
Software License Agreement (BSD)
\file      app1.cpp 
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

#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

#include "micros_swarm_framework/micros_swarm_framework.h"

#define BARRIER_VSTIG  1
#define ROBOT_SUM 20

int delta = 4;
int epsilon = 100;

struct XY
{
    float x;
    float y;
};

float force_mag(float dist)
{
    return -(epsilon/dist) *(pow(delta/dist, 4) - pow(delta/dist, 2));
}

XY force_sum(micros_swarm_framework::NeighborBase n, XY &s)
{
    micros_swarm_framework::KernelHandle kh;
    micros_swarm_framework::Base l=kh.getRobotBase();
    float xl=l.getX();
    float yl=l.getY();
    
    float xn=n.getX();
    float yn=n.getY();
    
    float dist=sqrt(pow((xl-xn),2)+pow((yl-yn),2));
    
    float fm = force_mag(dist)/1000;
    if(fm>0.5) fm=0.5;
    
    float fx=(fm/dist)*(xn-xl);
    float fy=(fm/dist)*(yn-yl);
    
    //std::cout<<"virtual force: "<<fx<<", "<<fy<<std::endl;
    
    s.x+=fx;
    s.y+=fy;
    return s;
}

XY direction()
{
    XY sum;
    sum.x=0;
    sum.y=0;
    
    micros_swarm_framework::Neighbors<micros_swarm_framework::NeighborBase> n(true);
    sum=n.neighborsReduce(force_sum, sum);
    
    return sum;
}

void motion()
{
    ros::NodeHandle nh;
    
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    
    ros::Rate loop_rate(10);
    
    while(ros::ok())
    {
        XY v=direction();
        geometry_msgs::Twist t;
        t.linear.x=v.x;
        t.linear.y=v.y;
        
        pub.publish(t);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void barrier_wait()
{
    //barrier
    micros_swarm_framework::KernelHandle kh;
    micros_swarm_framework::VirtualStigmergy<bool> barrier(BARRIER_VSTIG);
    std::string robot_id_string=boost::lexical_cast<std::string>(micros_swarm_framework::KernelInitializer::unique_robot_id_);
    barrier.virtualStigmergyPut(robot_id_string, 1);
    
    ros::Rate loop_rate(1);
    while(ros::ok())
    {
        if(barrier.virtualStigmergySize()==ROBOT_SUM)
            break;
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void baseCallback(const nav_msgs::Odometry& lmsg)
{
    float x=lmsg.pose.pose.position.x;
    float y=lmsg.pose.pose.position.y;
    
    float vx=lmsg.twist.twist.linear.x;
    float vy=lmsg.twist.twist.linear.y;
    
    micros_swarm_framework::KernelHandle kh;
    micros_swarm_framework::Base l(x, y, 0, vx, vy, 0);
    kh.setRobotBase(l);
}

int main(int argc, char** argv){
    
    ros::init(argc, argv, "micros_swarm_framework_app1_node");
    ros::NodeHandle nh;
    
    int robot_id=-1;
    bool param_ok = ros::param::get ("~unique_robot_id", robot_id);
    if(!param_ok)
    {
        std::cout<<"could not get parameter unique_robot_id"<<std::endl;
        exit(1);
    }else{
        std::cout<<"unique_robot_id = "<<robot_id<<std::endl;
    }
    micros_swarm_framework::KernelInitializer::initRobotID(robot_id);
    
    ros::Subscriber sub = nh.subscribe("base_pose_ground_truth", 1000, baseCallback, ros::TransportHints().udp());
    
    boost::thread barrier(&barrier_wait);
    barrier.join();
    
    boost::thread robot_move(&motion);
    
    ros::spin();
    
    return 0;
}
