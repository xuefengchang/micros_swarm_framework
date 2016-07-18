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

namespace micros_swarm_framework{

    struct XY
    {
        float x;
        float y;
    };
    
    class App1 : public nodelet::Nodelet
    {
        public:
            ros::NodeHandle node_handle_;
            boost::shared_ptr<RuntimePlatform> rtp_;
            boost::shared_ptr<CommunicationInterface> communicator_;
            ros::Timer timer_;
            ros::Publisher pub_;
            ros::Subscriber sub_;
            
            //app parameters
            int delta;
            int epsilon;
            
            App1();
            ~App1();
            virtual void onInit();
            
            //app functions
            float force_mag(float dist);
            XY force_sum(micros_swarm_framework::NeighborBase n, XY &s);
            XY direction();
            void motion();
            void publish_cmd(const ros::TimerEvent&);
            void baseCallback(const nav_msgs::Odometry& lmsg);  
    };

    App1::App1()
    {
        //set parameters
        delta = 4;
        epsilon = 100;
    }
    
    App1::~App1()
    {
    }
    
    float App1::force_mag(float dist)
    {
        return -(epsilon/(dist+0.1)) *(pow(delta/(dist+0.1), 4) - pow(delta/(dist+0.1), 2));
    }

    XY App1::force_sum(micros_swarm_framework::NeighborBase n, XY &s)
    {
        micros_swarm_framework::Base l=rtp_->getRobotBase();
        float xl=l.getX();
        float yl=l.getY();
    
        float xn=n.getX();
        float yn=n.getY();
    
        float dist=sqrt(pow((xl-xn),2)+pow((yl-yn),2));
     
        float fm = force_mag(dist)/1000;
        if(fm>0.5) fm=0.5;
    
        float fx=(fm/(dist+0.1))*(xn-xl);
        float fy=(fm/(dist+0.1))*(yn-yl);
    
        s.x+=fx;
        s.y+=fy;
        return s;
    }

    XY App1::direction()
    {
        XY sum;
        sum.x=0;
        sum.y=0;
    
        micros_swarm_framework::Neighbors<micros_swarm_framework::NeighborBase> n(true);
        boost::function<XY(NeighborBase, XY &)> bf=boost::bind(&App1::force_sum, this, _1, _2);
        sum=n.neighborsReduce(bf, sum);
    
        return sum;
    }
    
    void App1::publish_cmd(const ros::TimerEvent&)
    {
        
        XY v=direction();
        geometry_msgs::Twist t;
        t.linear.x=v.x;
        t.linear.y=v.y;
        
        pub_.publish(t);
        
    }

    void App1::motion()
    {
        timer_ = node_handle_.createTimer(ros::Duration(0.1), &App1::publish_cmd, this);
    }
    
    void App1::baseCallback(const nav_msgs::Odometry& lmsg)
    {
        float x=lmsg.pose.pose.position.x;
        float y=lmsg.pose.pose.position.y;
    
        float vx=lmsg.twist.twist.linear.x;
        float vy=lmsg.twist.twist.linear.y;
    
        micros_swarm_framework::Base l(x, y, 0, vx, vy, 0);
        rtp_->setRobotBase(l);
    }
    
    void App1::onInit()
    {
        node_handle_ = getNodeHandle();
        rtp_=Singleton<RuntimePlatform>::getSingleton();
        #ifdef ROS
        communicator_=Singleton<ROSCommunication>::getSingleton();
        #endif
        #ifdef OPENSPLICE_DDS
        communicator_=Singleton<OpenSpliceDDSCommunication>::getSingleton();
        #endif
    
        sub_ = node_handle_.subscribe("base_pose_ground_truth", 1000, &App1::baseCallback, this, ros::TransportHints().udp());
        pub_ = node_handle_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    
        motion();
    }
};

// Register the nodelet
PLUGINLIB_EXPORT_CLASS(micros_swarm_framework::App1, nodelet::Nodelet)
