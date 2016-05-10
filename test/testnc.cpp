/* 
 *  testnc.cpp - micros_swarm_framework test neighbor communication
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

static float nbres=0;

void testnb(float value, int a)
{
    nbres=nbres+value+a;
    std::cout<<"nbres="<<nbres<<std::endl;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "micros_swarm_framework_testnc_node");
    ros::NodeHandle nh;
    
    micros_swarm_framework::KernelInitializer::initRobotID(0);

    micros_swarm_framework::NeighborCommunication<float> nc("test");
    NEIGHBOR_FUNCTION_OBJECT(float) nfo = BIND_FUNCTION_AND_PARAMETER_VALUES(&testnb, NB_VALUE, 1);
    nc.neighborListen(nfo);
    ros::Rate loop_rate(10);
    
    while(ros::ok())
    {
      if(nbres>314)
          nc.neighborIgnore();
      ros::spinOnce();
      loop_rate.sleep();
    }
    
    ros::spin();
    
    return 0;
}
