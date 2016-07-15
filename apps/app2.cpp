/**
Software License Agreement (BSD)
\file      app2.cpp 
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
#include <iostream>
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

#include "micros_swarm_framework/micros_swarm_framework.h"

#define BARRIER_VSTIG  1
#define ROBOT_SUM 20

#define RED_SWARM 1
#define BLUE_SWARM 2


namespace micros_swarm_framework{

    int delta_kin = 5;
    int epsilon_kin = 100;

    int delta_nonkin = 30;
    int epsilon_nonkin = 1000;

    struct XY
    {
        float x;
        float y;
    };
    
    boost::shared_ptr<RuntimePlatform> rtp_=Singleton<RuntimePlatform>::getSingleton();
    //std::cout<<"*************"<<rtp_->getRobotID()<<","<<rtp_.use_count()<<std::endl;
    boost::shared_ptr<CommunicationInterface> communicator_=Singleton<ROSCommunication>::getSingleton();
    
    ros::NodeHandle nh;
    
    class App2 : public nodelet::Nodelet
    {
        public:
            ros::NodeHandle node_handle_;
            boost::shared_ptr<RuntimePlatform> rtp_;
            boost::shared_ptr<CommunicationInterface> communicator_;
            boost::shared_ptr<PacketParser> parser_;
            ros::Timer timer_;
            
            App2();
            ~App2();
            //XY force_sum_kin(micros_swarm_framework::NeighborBase n, XY &s);
            //XY force_sum_nonkin(micros_swarm_framework::NeighborBase n, XY &s);
            //XY direction_red();
            //XY direction_blue();
            //bool red(int id);
            //bool blue(int id);
            //void motion_red();
            //void motion_blue();
            //void barrier_wait();
            //void baseCallback(const nav_msgs::Odometry& lmsg);
            virtual void onInit();
    };

    App2::App2()
    {
        
    }
    
    App2::~App2()
    {
    }
    
    float force_mag_kin(float dist)
    {
        return -(epsilon_kin/dist) *(pow(delta_kin/dist, 4) - pow(delta_kin/dist, 2));
    }

    float force_mag_nonkin(float dist)
    {
        return -(epsilon_nonkin/dist) *(pow(delta_nonkin/dist, 4) - pow(delta_nonkin/dist, 2));
    }

    XY force_sum_kin(micros_swarm_framework::NeighborBase n, XY &s)
    {
        micros_swarm_framework::Base l=rtp_->getRobotBase();
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

    XY force_sum_nonkin(micros_swarm_framework::NeighborBase n, XY &s)
    {
        micros_swarm_framework::Base l=rtp_->getRobotBase();
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
    
        micros_swarm_framework::Neighbors<micros_swarm_framework::NeighborBase> n(true);
        sum=n.neighborsKin(RED_SWARM).neighborsReduce(force_sum_kin, sum);
        sum=n.neighborsNonKin(RED_SWARM).neighborsReduce(force_sum_nonkin, sum);
    
        return sum;
    }

    XY direction_blue()
    {
        XY sum;
        sum.x=0;
        sum.y=0;
    
        micros_swarm_framework::Neighbors<micros_swarm_framework::NeighborBase> n(true);
        sum=n.neighborsKin(BLUE_SWARM).neighborsReduce(force_sum_kin, sum);
        sum=n.neighborsNonKin(BLUE_SWARM).neighborsReduce(force_sum_nonkin, sum);
    
        return sum;
    }

    bool red(int id)
    {
        if(id<=9)
            return true;
        return false;
    }

    bool blue(int id)
    {
        if(id>=10)
            return true;
        return false;
    }

    void motion_red()
    {
        //ros::NodeHandle nh;
    
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
        //ros::NodeHandle nh;
    
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
        int robot_id=rtp_->getRobotID();
    
        micros_swarm_framework::Barrier_Syn bs("SYN");
                
        std::ostringstream archiveStream;
        boost::archive::text_oarchive archive(archiveStream);
        archive<<bs;
        std::string bs_string=archiveStream.str();
    
        micros_swarm_framework::MSFPPacket p;
        p.packet_source=robot_id;
        p.packet_version=1;
        p.packet_type=micros_swarm_framework::BARRIER_SYN;
        #ifdef ROS
        p.packet_data=bs_string;
        #endif
        #ifdef OPENSPLICE_DDS
        p.packet_data=bs_string.data();
        #endif
        p.package_check_sum=0;
    
        ros::Rate loop_rate(1);
        while(ros::ok())
        {
            communicator_->broadcast(p);
            int barrier_size=rtp_->getBarrierSize();
            if(barrier_size==ROBOT_SUM-1)
                break;
        
            ros::spinOnce();
            loop_rate.sleep();
            std::cout<<robot_id<<"barrier_size: "<<barrier_size<<std::endl;
        }
    }
    
    void baseCallback(const nav_msgs::Odometry& lmsg)
    {
        float x=lmsg.pose.pose.position.x;
        float y=lmsg.pose.pose.position.y;
    
        float vx=lmsg.twist.twist.linear.x;
        float vy=lmsg.twist.twist.linear.y;
    
        micros_swarm_framework::Base l(x, y, 0, vx, vy, 0);
        rtp_->setRobotBase(l);
    }
    
    void App2::onInit()
    {
        node_handle_ = getPrivateNodeHandle();
        nh = getPrivateNodeHandle();
        rtp_=Singleton<RuntimePlatform>::getSingleton();
        //std::cout<<"*************"<<rtp_->getRobotID()<<","<<rtp_.use_count()<<std::endl;
        communicator_=Singleton<ROSCommunication>::getSingleton();
        //std::cout<<"*************"<<communicator_->name_<<","<<communicator_.use_count()<<std::endl;
        //communicator_->receive(packetParser);
    
        ros::Subscriber sub = node_handle_.subscribe("base_pose_ground_truth", 1000, baseCallback, ros::TransportHints().udp());
    
        boost::thread barrier(&barrier_wait);
        barrier.join();  
        
        boost::function<bool()> bfred=boost::bind(&red, rtp_->getRobotID());
        boost::function<bool()> bfblue=boost::bind(&blue, rtp_->getRobotID());
    
        micros_swarm_framework::Swarm red_swarm(RED_SWARM);
        red_swarm.selectSwarm(bfred);
        micros_swarm_framework::Swarm blue_swarm(BLUE_SWARM);
        blue_swarm.selectSwarm(bfblue);
    
        //red_swarm.printSwarm();
        //blue_swarm.printSwarm();
    
        red_swarm.execute(&motion_red);
        blue_swarm.execute(&motion_blue);
    }
};

// Register the nodelet
PLUGINLIB_EXPORT_CLASS(micros_swarm_framework::App2, nodelet::Nodelet)
