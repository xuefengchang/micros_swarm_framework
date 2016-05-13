/**
Software License Agreement (BSD)
\file      testnc.cpp 
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
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

#include "micros_swarm_framework/micros_swarm_framework.h"

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
    boost::function<void(float)> nfo = boost::bind(&testnb, _1, 1);
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
