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

#include "apps/app2.h"

#define BARRIER_VSTIG  1
#define ROBOT_SUM 20

#define RED_SWARM 1
#define BLUE_SWARM 2

namespace micros_swarm_framework{

    struct XY
    {
        float x;
        float y;
    };

    App2::App2(ros::NodeHandle node_handle):Application(node_handle)
    {
        
    }
    
    App2::~App2()
    {
    }
    
    void App2::init()
    {
        //set parameters
        delta_kin = 5;
        epsilon_kin = 100;

        delta_nonkin = 30;
        epsilon_nonkin = 1000;
    }
    
    float App2::force_mag_kin(float dist)
    {
        return -(epsilon_kin/(dist+0.1)) *(pow(delta_kin/(dist+0.1), 4) - pow(delta_kin/(dist+0.1), 2));
    }

    float App2::force_mag_nonkin(float dist)
    {
        return -(epsilon_nonkin/(dist+0.1)) *(pow(delta_nonkin/(dist+0.1), 4) - pow(delta_nonkin/(dist+0.1), 2));
    }

    XY App2::force_sum_kin(micros_swarm_framework::NeighborBase n, XY &s)
    {
        micros_swarm_framework::Base l=base();
        float xl=l.x;
        float yl=l.y;
    
        float xn=n.x;
        float yn=n.y;
    
        float dist=sqrt(pow((xl-xn),2)+pow((yl-yn),2));
    
        float fm = force_mag_kin(dist)/1000;
        if(fm>0.5) fm=0.5;
    
        float fx=(fm/(dist+0.1))*(xn-xl);
        float fy=(fm/(dist+0.1))*(yn-yl);
    
        s.x+=fx;
        s.y+=fy;
        return s;
    }

    XY App2::force_sum_nonkin(micros_swarm_framework::NeighborBase n, XY &s)
    {
        micros_swarm_framework::Base l=base();
        float xl=l.x;
        float yl=l.y;
    
        float xn=n.x;
        float yn=n.y;
    
        float dist=sqrt(pow((xl-xn),2)+pow((yl-yn),2));
    
        float fm = force_mag_nonkin(dist)/1000;
        if(fm>0.5) fm=0.5;
    
        float fx=(fm/(dist+0.1))*(xn-xl);
        float fy=(fm/(dist+0.1))*(yn-yl);
    
        s.x+=fx;
        s.y+=fy;
        return s;
    }

    XY App2::direction_red()
    {
        XY sum;
        sum.x=0;
        sum.y=0;
    
        micros_swarm_framework::Neighbors<micros_swarm_framework::NeighborBase> n(true);
        boost::function<XY(NeighborBase, XY &)> bf_kin=boost::bind(&App2::force_sum_kin, this, _1, _2);
        boost::function<XY(NeighborBase, XY &)> bf_nonkin=boost::bind(&App2::force_sum_nonkin, this, _1, _2);
        sum=n.kin(RED_SWARM).reduce(bf_kin, sum);
        sum=n.nonkin(RED_SWARM).reduce(bf_nonkin, sum);
    
        return sum;
    }

    XY App2::direction_blue()
    {
        XY sum;
        sum.x=0;
        sum.y=0;
    
        micros_swarm_framework::Neighbors<micros_swarm_framework::NeighborBase> n(true);
        boost::function<XY(NeighborBase, XY &)> bf_kin=boost::bind(&App2::force_sum_kin, this, _1, _2);
        boost::function<XY(NeighborBase, XY &)> bf_nonkin=boost::bind(&App2::force_sum_nonkin, this, _1, _2);
        sum=n.kin(BLUE_SWARM).reduce(bf_kin, sum);
        sum=n.nonkin(BLUE_SWARM).reduce(bf_nonkin, sum);
    
        return sum;
    }

    bool App2::red(int id)
    {
        if(id<=9)
            return true;
        return false;
    }

    bool App2::blue(int id)
    {
        if(id>=10)
            return true;
        return false;
    }
    
    void App2::publish_red_cmd(const ros::TimerEvent&)
    {
        XY v=direction_red();
        geometry_msgs::Twist t;
        t.linear.x=v.x;
        t.linear.y=v.y;
        
        pub.publish(t);
    }
    
    void App2::publish_blue_cmd(const ros::TimerEvent&)
    {
        XY v=direction_blue();
        geometry_msgs::Twist t;
        t.linear.x=v.x;
        t.linear.y=v.y;
        
        pub.publish(t);
    }

    void App2::motion_red()
    {
        red_timer = nh.createTimer(ros::Duration(0.1), &App2::publish_red_cmd, this);
    }

    void App2::motion_blue()
    {
        blue_timer = nh.createTimer(ros::Duration(0.1), &App2::publish_blue_cmd, this);
    }
    
    void App2::baseCallback(const nav_msgs::Odometry& lmsg)
    {
        float x=lmsg.pose.pose.position.x;
        float y=lmsg.pose.pose.position.y;
    
        float vx=lmsg.twist.twist.linear.x;
        float vy=lmsg.twist.twist.linear.y;
    
        micros_swarm_framework::Base l(x, y, 0, vx, vy, 0);
        set_base(l);
    }
    
    void App2::start()
    {
        init();
    
        pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
        sub = nh.subscribe("base_pose_ground_truth", 1000, &App2::baseCallback, this, ros::TransportHints().udp());
        ros::Duration(5).sleep();  //this is necessary, in order that the runtime platform kernel of the robot has enough time to publish it's base information.
        
        boost::function<bool()> bfred=boost::bind(&App2::red, this, robot_id());
        boost::function<bool()> bfblue=boost::bind(&App2::blue, this, robot_id());
    
        micros_swarm_framework::Swarm red_swarm(RED_SWARM);
        red_swarm.select(bfred);
        micros_swarm_framework::Swarm blue_swarm(BLUE_SWARM);
        blue_swarm.select(bfblue);
        
        red_swarm.execute(boost::bind(&App2::motion_red, this));
        blue_swarm.execute(boost::bind(&App2::motion_blue, this));
        
        red_swarm.print();
        blue_swarm.print();
    }
};
