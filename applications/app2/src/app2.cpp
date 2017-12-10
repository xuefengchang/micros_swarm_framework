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

#include "app2/app2.h"

#define BARRIER_VSTIG  1
#define ROBOT_SUM 20

#define RED_SWARM 1
#define BLUE_SWARM 2

// Register the application
PLUGINLIB_EXPORT_CLASS(app2::App2, micros_swarm::Application)

namespace app2{

    struct XY
    {
        float x;
        float y;
    };

    App2::App2() {}
    
    App2::~App2() {}

    void App2::stop() {}
    
    void App2::init()
    {
        //set parameters
        delta_kin = 5;
        epsilon_kin = 100;

        delta_nonkin = 25;
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

    XY App2::force_sum_kin(micros_swarm::NeighborBase n, XY &s)
    {
        micros_swarm::Base l = get_base();
        float xl = l.x;
        float yl = l.y;
    
        float xn = n.x;
        float yn = n.y;
    
        float dist = sqrt(pow((xl-xn),2)+pow((yl-yn),2));
    
        float fm = force_mag_kin(dist)/1000;
        if(fm>0.5) {
            fm=0.5;
        }
    
        float fx = (fm/(dist+0.1))*(xn-xl);
        float fy = (fm/(dist+0.1))*(yn-yl);
    
        s.x += fx;
        s.y += fy;
        return s;
    }

    XY App2::force_sum_nonkin(micros_swarm::NeighborBase n, XY &s)
    {
        micros_swarm::Base l = get_base();
        float xl = l.x;
        float yl = l.y;
    
        float xn = n.x;
        float yn = n.y;
    
        float dist = sqrt(pow((xl-xn),2)+pow((yl-yn),2));
    
        float fm = force_mag_nonkin(dist)/1000;
        if(fm>0.5) {
            fm=0.5;
        }
    
        float fx = (fm/(dist+0.1))*(xn-xl);
        float fy = (fm/(dist+0.1))*(yn-yl);
    
        s.x += fx;
        s.y += fy;
        return s;
    }

    XY App2::direction_red()
    {
        XY sum;
        sum.x = 0;
        sum.y = 0;
    
        micros_swarm::Neighbors<micros_swarm::NeighborBase> n(true);
        boost::function<XY(micros_swarm::NeighborBase, XY &)> bf_kin = boost::bind(&App2::force_sum_kin, this, _1, _2);
        boost::function<XY(micros_swarm::NeighborBase, XY &)> bf_nonkin = boost::bind(&App2::force_sum_nonkin, this, _1, _2);
        sum = n.kin(RED_SWARM).reduce(bf_kin, sum);
        sum = n.nonkin(RED_SWARM).reduce(bf_nonkin, sum);
    
        return sum;
    }

    XY App2::direction_blue()
    {
        XY sum;
        sum.x = 0;
        sum.y = 0;
    
        micros_swarm::Neighbors<micros_swarm::NeighborBase> n(true);
        boost::function<XY(micros_swarm::NeighborBase, XY &)> bf_kin = boost::bind(&App2::force_sum_kin, this, _1, _2);
        boost::function<XY(micros_swarm::NeighborBase, XY &)> bf_nonkin = boost::bind(&App2::force_sum_nonkin, this, _1, _2);
        sum = n.kin(BLUE_SWARM).reduce(bf_kin, sum);
        sum = n.nonkin(BLUE_SWARM).reduce(bf_nonkin, sum);
    
        return sum;
    }

    bool App2::red(int id)
    {
        if(id <= 9) {
            return true;
        }
        return false;
    }

    bool App2::blue(int id)
    {
        if(id >= 10) {
            return true;
        }
        return false;
    }
    
    void App2::publish_red_cmd(const ros::TimerEvent&)
    {
        XY v = direction_red();
        geometry_msgs::Twist t;
        t.linear.x = v.x;
        t.linear.y = v.y;
        
        pub.publish(t);
    }
    
    void App2::publish_blue_cmd(const ros::TimerEvent&)
    {
        XY v = direction_blue();
        geometry_msgs::Twist t;
        t.linear.x = v.x;
        t.linear.y = v.y;
        
        pub.publish(t);
    }

    void App2::motion_red()
    {
        ros::NodeHandle nh;
        red_timer = nh.createTimer(ros::Duration(0.1), &App2::publish_red_cmd, this);
    }

    void App2::motion_blue()
    {
        ros::NodeHandle nh;
        blue_timer = nh.createTimer(ros::Duration(0.1), &App2::publish_blue_cmd, this);
    }
    
    void App2::baseCallback(const nav_msgs::Odometry& lmsg)
    {
        float x = lmsg.pose.pose.position.x;
        float y = lmsg.pose.pose.position.y;
    
        float vx = lmsg.twist.twist.linear.x;
        float vy = lmsg.twist.twist.linear.y;
    
        micros_swarm::Base l(x, y, 0, vx, vy, 0, 1);
        set_base(l);
    }
    
    void App2::start()
    {
        init();

        ros::NodeHandle nh;
        set_dis(40);
        pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
        sub = nh.subscribe("base_pose_ground_truth", 1000, &App2::baseCallback, this, ros::TransportHints().udp());
        
        boost::function<bool()> bfred = boost::bind(&App2::red, this, get_id());
        boost::function<bool()> bfblue = boost::bind(&App2::blue, this, get_id());
    
        micros_swarm::Swarm red_swarm(RED_SWARM);
        red_swarm.select(bfred);
        micros_swarm::Swarm blue_swarm(BLUE_SWARM);
        blue_swarm.select(bfblue);

        red_swarm.print();
        blue_swarm.print();
        
        red_swarm.execute(boost::bind(&App2::motion_red, this));
        blue_swarm.execute(boost::bind(&App2::motion_blue, this));
    }
};
