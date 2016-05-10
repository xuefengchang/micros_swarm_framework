/* 
 *  app1.cpp - micros_swarm_framework app1
 *  Copyright (C) 2016 Xuefeng Chang
 *  
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <micros_swarm_framework/micros_swarm_framework.h>

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

XY force_sum(micros_swarm_framework::NeighborLocation n, XY &s)
{
    micros_swarm_framework::KernelHandle kh;
    micros_swarm_framework::Location l=kh.getRobotLocation();
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
    
    micros_swarm_framework::Neighbors<micros_swarm_framework::NeighborLocation> n(true);
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

void locationCallback(const nav_msgs::Odometry& lmsg)
{
    float x=lmsg.pose.pose.position.x;
    float y=lmsg.pose.pose.position.y;
    
    micros_swarm_framework::KernelHandle kh;
    micros_swarm_framework::Location l(x, y, 0);
    kh.setRobotLocation(l);
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
    
    ros::Subscriber sub = nh.subscribe("base_pose_ground_truth", 1000, locationCallback);
    
    boost::thread barrier(&barrier_wait);
    barrier.join();
    
    boost::thread robot_move(&motion);
    
    ros::spin();
    
    return 0;
}
