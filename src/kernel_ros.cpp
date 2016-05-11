/* 
 *  kernel_ros.cpp - micros_swarm_framework kernel
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

#define ROBOT_ID_FREQUENCY 1
#define SWARM_LIST_FREQUENCY 0.1

namespace micros_swarm_framework{
    
    void publish_robot_id()
    {
        ros::NodeHandle n;
        micros_swarm_framework::KernelHandle kh;
        ros::Publisher pub = n.advertise<micros_swarm_framework::MSFPPacket>("/micros_swarm_framework_topic", 1000);
   
        ros::Rate loop_rate(ROBOT_ID_FREQUENCY);
    
        unsigned int robot_id=kh.getRobotID();
        micros_swarm_framework::Location l;

        while (ros::ok())
        {
            l.setX(-1);
            l.setY(-1);
            l.setZ(-1);
            l=kh.getRobotLocation();
        
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
            p.packet_ttl=1;
            p.package_check_sum=0;

            pub.publish(p);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    
    void publish_swarm_list()
    {
        ros::NodeHandle n;
        micros_swarm_framework::KernelHandle kh;
        ros::Publisher pub = n.advertise<micros_swarm_framework::MSFPPacket>("/micros_swarm_framework_topic", 1000);
        ros::Rate loop_rate(SWARM_LIST_FREQUENCY);
    
        unsigned int robot_id=kh.getRobotID();
        std::vector<unsigned int> swarm_list;

        while (ros::ok())
        {
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
            p.packet_ttl=1;
            p.package_check_sum=0;

            pub.publish(p);
            ros::spinOnce();
            loop_rate.sleep();
        }
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
        std::cout<<"unique_robot_id = "<<robot_id<<std::endl;
    }
    micros_swarm_framework::KernelInitializer::initRobotID(robot_id);
    
    micros_swarm_framework::KernelInitializer kernel_init;
    
    micros_swarm_framework::KernelHandle kh;
    kh.setRobotID((unsigned int)robot_id);
    
    boost::thread publish_robot_id_thread(&micros_swarm_framework::publish_robot_id);
    boost::thread publish_swarm_list_thread(&micros_swarm_framework::publish_swarm_list);
    
    std::cout<<"_________________________________________________________"<<std::endl;
    std::cout<<"|                                                       |"<<std::endl;
    std::cout<<"|The micros_swarm_framework_kernel started successfully.|"<<std::endl;
    std::cout<<"|                                                       |"<<std::endl;
    std::cout<<"|            Designed by microser in HPCL.              |"<<std::endl;
    std::cout<<"|                                                       |"<<std::endl;
    std::cout<<"|              changxuefengcn@163.com                   |"<<std::endl;
    std::cout<<"|_______________________________________________________|"<<std::endl;
    
    ros::spin();

    return 0;
}
