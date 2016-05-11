/* 
 *  app2.cpp - micros_swarm_framework app2
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

#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

#include "micros_swarm_framework/micros_swarm_framework.h"

#define BARRIER_VSTIG  1
#define ROBOT_SUM 20

#define RED_SWARM 1
#define BLUE_SWARM 2

int delta_kin = 5;
int epsilon_kin = 100;

int delta_nonkin = 30;
int epsilon_nonkin = 1000;

struct XY
{
    float x;
    float y;
};

float force_mag_kin(float dist)
{
    return -(epsilon_kin/dist) *(pow(delta_kin/dist, 4) - pow(delta_kin/dist, 2));
}

float force_mag_nonkin(float dist)
{
    return -(epsilon_nonkin/dist) *(pow(delta_nonkin/dist, 4) - pow(delta_nonkin/dist, 2));
}

XY force_sum_kin(micros_swarm_framework::NeighborLocation n, XY &s)
{
    micros_swarm_framework::KernelHandle kh;
    micros_swarm_framework::Location l=kh.getRobotLocation();
    float xl=l.getX();
    float yl=l.getY();
    
    float xn=n.getX();
    float yn=n.getY();
    
    float dist=sqrt(pow((xl-xn),2)+pow((yl-yn),2));
    
    float fm = force_mag_kin(dist)/1000;
    if(fm>0.5) fm=0.5;
    
    float fx=(fm/dist)*(xn-xl);
    float fy=(fm/dist)*(yn-yl);
    
    //std::cout<<"virtual force: "<<fx<<", "<<fy<<std::endl;
    
    s.x+=fx;
    s.y+=fy;
    return s;
}

XY force_sum_nonkin(micros_swarm_framework::NeighborLocation n, XY &s)
{
    micros_swarm_framework::KernelHandle kh;
    micros_swarm_framework::Location l=kh.getRobotLocation();
    float xl=l.getX();
    float yl=l.getY();
    
    float xn=n.getX();
    float yn=n.getY();
    
    float dist=sqrt(pow((xl-xn),2)+pow((yl-yn),2));
    
    float fm = force_mag_nonkin(dist)/1000;
    if(fm>0.5) fm=0.5;
    
    float fx=(fm/dist)*(xn-xl);
    float fy=(fm/dist)*(yn-yl);
    
    //std::cout<<"virtual force: "<<fx<<", "<<fy<<std::endl;
    
    s.x+=fx;
    s.y+=fy;
    return s;
}

XY direction_red()
{
    XY sum;
    sum.x=0;
    sum.y=0;
    
    micros_swarm_framework::Neighbors<micros_swarm_framework::NeighborLocation> n(true);
    sum=n.neighborsKin(RED_SWARM).neighborsReduce(force_sum_kin, sum);
    sum=n.neighborsNonKin(RED_SWARM).neighborsReduce(force_sum_nonkin, sum);
    
    return sum;
}

XY direction_blue()
{
    XY sum;
    sum.x=0;
    sum.y=0;
    
    micros_swarm_framework::Neighbors<micros_swarm_framework::NeighborLocation> n(true);
    sum=n.neighborsKin(BLUE_SWARM).neighborsReduce(force_sum_kin, sum);
    sum=n.neighborsNonKin(BLUE_SWARM).neighborsReduce(force_sum_nonkin, sum);
    
    return sum;
}

bool red(unsigned int id)
{
    if(id<=9)
        return true;
    return false;
}

bool blue(unsigned int id)
{
    if(id>=10)
        return true;
    return false;
}

void motion_red()
{
    ros::NodeHandle nh;
    micros_swarm_framework::KernelHandle kh;
    
    static ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    
    ros::Rate loop_rate(10);
    
    while(ros::ok())
    {
        XY v=direction_red();
        geometry_msgs::Twist t;
        t.linear.x=v.x;
        t.linear.y=v.y;
        
        pub.publish(t);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void motion_blue()
{
    ros::NodeHandle nh;
    micros_swarm_framework::KernelHandle kh;
    
    static ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    
    ros::Rate loop_rate(10);
    
    while(ros::ok())
    {
        XY v=direction_blue();
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
    
    ros::init(argc, argv, "micros_swarm_framework_app2_node");
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
    
    micros_swarm_framework::KernelHandle kh;
    BOOL_FUNCTION_OBJECT bfred=BIND_FUNCTION_AND_PARAMETER_VALUES(&red, kh.getRobotID());
    BOOL_FUNCTION_OBJECT bfblue=BIND_FUNCTION_AND_PARAMETER_VALUES(&blue, kh.getRobotID());
    
    micros_swarm_framework::Swarm red_swarm(RED_SWARM);
    red_swarm.selectSwarm(bfred);
    micros_swarm_framework::Swarm blue_swarm(BLUE_SWARM);
    blue_swarm.selectSwarm(bfblue);
    
    red_swarm.execute(&motion_red);
    blue_swarm.execute(&motion_blue);
    
    ros::spin();
    
    return 0;
}
