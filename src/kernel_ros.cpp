/**
Software License Agreement (BSD)
\file      kernel_ros.cpp 
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

#define PUBLISH_ROBOT_ID_DURATION 0.1
#define PUBLISH_SWARM_LIST_DURATION 5

namespace micros_swarm_framework{
    
    void publish_robot_id(const ros::TimerEvent&)
    {
        micros_swarm_framework::KernelHandle kh;
        
        unsigned int robot_id=kh.getRobotID();
        
        micros_swarm_framework::Base l;
        l.setX(-1);
        l.setY(-1);
        l.setZ(-1);
        l=kh.getRobotBase();
        
        SingleRobotBroadcastID srbi(robot_id, l.getX(), l.getY(), l.getZ());
        
        std::ostringstream archiveStream;
        boost::archive::text_oarchive archive(archiveStream);
        archive<<srbi;
        std::string srbi_str=archiveStream.str();
                      
        micros_swarm_framework::MSFPPacket p;
        p.packet_source=robot_id;
        p.packet_version=1;
        p.packet_type=SINGLE_ROBOT_BROADCAST_ID;
        p.packet_data=srbi_str;
        p.package_check_sum=0;

        kh.publishPacket(p);
    }
    
    void publish_swarm_list(const ros::TimerEvent&)
    {
        micros_swarm_framework::KernelHandle kh;
        
        unsigned int robot_id=kh.getRobotID();
        
        std::vector<unsigned int> swarm_list;
        swarm_list.clear();
        swarm_list=kh.getSwarmList();
        
        SingleRobotSwarmList srsl(robot_id, swarm_list);
        
        std::ostringstream archiveStream;
        boost::archive::text_oarchive archive(archiveStream);
        archive<<srsl;
        std::string srsl_str=archiveStream.str();
                      
        micros_swarm_framework::MSFPPacket p;
        p.packet_source=robot_id;
        p.packet_version=1;
        p.packet_type=SINGLE_ROBOT_SWARM_LIST;
        p.packet_data=srsl_str;
        p.package_check_sum=0;

        kh.publishPacket(p);
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "micros_swarm_framework_kernel_node");
    ros::NodeHandle nh;
    
    int robot_id=-1;
    bool param_ok = ros::param::get ("~unique_robot_id", robot_id);
    if(!param_ok)
    {
        std::cout<<"could not get parameter unique_robot_id"<<std::endl;
        exit(1);
    }else{
        //std::cout<<"unique_robot_id = "<<robot_id<<std::endl;
    }
    micros_swarm_framework::KernelInitializer::initRobotID(robot_id);
    
    micros_swarm_framework::KernelInitializer kernel_init;
    
    micros_swarm_framework::KernelHandle kh;
    kh.setRobotID((unsigned int)robot_id);
    
    ros::Timer publish_robot_id_timer = nh.createTimer(ros::Duration(PUBLISH_ROBOT_ID_DURATION), &micros_swarm_framework::publish_robot_id);
    ros::Timer publish_swarm_list_timer = nh.createTimer(ros::Duration(PUBLISH_SWARM_LIST_DURATION), &micros_swarm_framework::publish_swarm_list);
    
    std::cout<<"The micros_swarm_framework_kernel started successfully."<<std::endl;
    std::cout<<"local robot id is: "<<robot_id<<std::endl;
    
    ros::spin();

    return 0;
}
