/**
Software License Agreement (BSD)
\file      gazebo_flocking.cpp
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

#include "gazebo_flocking/gazebo_flocking.h"
#include "hector_uav_msgs/EnableMotors.h"
#include <tf/tf.h>
#include <cmath>

double my_theta = 0;
double PI =acos(-1);
#define EPSILON 0.1
#define A 5
#define B 5
const double C = abs(A-B) / sqrt(4*A*B);
#define H 0.2
#define D 7
#define R 9
#define C1 0.3
#define C2 0.6
#define speedlimit 2

double pm1=0.3,pm2=1,pm3=0.3;

using namespace std;

class NeighborHandle
{
public:
    double _px,_py,_vx,_vy;
    pair<double,double> _position,_velocity;
    int _r_id;
    double mypm_g;
    NeighborHandle(int r_id, float x, float y, float vx, float vy)
    {

        _px=x;
        _py=y;
        _vx=vx;
        _vy=vy;
        _position=pair<double,double>(x,y);
        _velocity=pair<double,double>(vx,vy);
        _r_id = r_id;
        mypm_g=1;
    }
};

static list<NeighborHandle*> neighbor_list;
pair<double,double> my_position=pair<double,double>(0,0);
pair<double,double> my_velocity=pair<double,double>(0,0);

pair<double,double> get_vector(pair<double,double> start,pair<double,double> end)
{
    pair<double,double> re=pair<double,double>(0,0);
    re.first=end.first-start.first;
    re.second=end.second-start.second;
    return re;
}

double segma_norm(pair<double,double> v)
{
    double re = EPSILON*(v.first*v.first+v.second*v.second);
    re = sqrt(1+re)-1;
    re /= EPSILON;
    return re;
}

double R_alpha = segma_norm(pair<double,double>(R,0));
double D_alpha = segma_norm(pair<double,double>(D,0));

pair<double,double> segma_epsilon(pair<double,double> v)
{
    pair<double,double> re = pair<double,double>(0,0);
    double scale = 1+EPSILON*(v.first*v.first+v.second*v.second);
    scale = sqrt(scale);
    re.first = v.first / scale;
    re.second = v.second / scale;
    return re;
}

double segma_1(double z)
{
    return z / sqrt(1+z*z);
}

double phi(double z)
{
    return 0.5*((A+B)*segma_1(z+C)+A-B);
}

double rho(double z)
{
    if(z<H)
        return 1;
    if(z>1)
        return 0;
    return 0.5*(1+cos(PI*(z-H)/(1-H)));
}

double phi_alpha(double z)
{
    return rho(z/R_alpha)*phi(z-D_alpha);
}

pair<double,double> f_g()
{
    pair<double,double> re = pair<double,double>(0,0);
    for(list<NeighborHandle*>::iterator i=neighbor_list.begin();i!=neighbor_list.end();i++)
    {
        pair<double,double> q_ij = get_vector(my_position,(*i)->_position);
        pair<double,double> n_ij = segma_epsilon(q_ij);
        re.first += phi_alpha(segma_norm(q_ij))*n_ij.first*(*i)->mypm_g;
        re.second += phi_alpha(segma_norm(q_ij))*n_ij.second*(*i)->mypm_g;
    }
    return re;
}

double a_ij(pair<double,double> j_p)
{
    return rho(segma_norm(get_vector(my_position,j_p)) / R_alpha);
}

pair<double,double> f_d()
{
    pair<double,double> re = pair<double,double>(0,0);
    for(list<NeighborHandle*>::iterator i=neighbor_list.begin();i!=neighbor_list.end();i++)
    {
        pair<double,double> p_ij = get_vector(my_velocity,(*i)->_velocity);
        re.first += a_ij((*i)->_position) * p_ij.first;
        re.second += a_ij((*i)->_position) * p_ij.second;
    }
    return re;
}

double interval = 0.1;
pair<double,double> p_r = pair<double,double>(3,0);
pair<double,double> q_r = pair<double,double>(-40,-40);
pair<double,double> f_r()
{
    pair<double,double> re = pair<double,double>(0,0);



    re.first = -C1*get_vector(q_r,my_position).first - C2*get_vector(p_r,my_velocity).first;
    re.second = -C1*get_vector(q_r,my_position).second - C2*get_vector(p_r,my_velocity).second;
    return re;
}

// Register the application
PLUGINLIB_EXPORT_CLASS(gazebo_flocking::GazeboFlocking, micros_swarm::Application)

namespace gazebo_flocking{

    GazeboFlocking::GazeboFlocking() {}

    GazeboFlocking::~GazeboFlocking() {}

    void GazeboFlocking::stop() {}

    void GazeboFlocking::init()
    {
        //set parameters
        hz = 10;
        interval = 1.0/hz;
    }

    int vel_loop_count =0;

    bool test_location(float a, float b, float x, float y)
    {
        float s = sqrt((a-x)*(a-x)+(b-y)*(b-y));
        if(s <= 2) {
            return true;
        }
        return false;
    }

    void GazeboFlocking::publish_cmd(const ros::TimerEvent&)
    {
        static double base_angle = 0;
        vel_loop_count++;
        int time_count = vel_loop_count;
        int hz = 10;
        double baseomega = 0;
        double basespeed = 3;
        /*
            //if(q_r.first == 40 && q_r.second == -40)
            if(vel_loop_count==230)
            //if(test_location(q_r.first, q_r.second, 40, -40))
            {
                p_r.first = 0;
                p_r.second = 3;
            }

            //if(q_r.first == 40 && q_r.second == 40)
            if(vel_loop_count==440)
            //if(test_location(q_r.first, q_r.second, 40, 40))
            {
                p_r.first = -3;
                p_r.second = 0;
            }

            //if(q_r.first == -40 && q_r.second == 40)
            if(vel_loop_count==500)
            if(test_location(q_r.first, q_r.second, -40, 40))
            {
                p_r.first = 0;
                p_r.second = -3;
            }
            if(vel_loop_count==620)
            if(test_location(q_r.first, q_r.second, -40, 40))
            {
                p_r.first = 3;
                p_r.second = 0;
            }     */

        if(time_count>=23*hz && time_count<=23*hz+10*hz)
            baseomega = PI/2.0/10;//PI/30;
        else if(time_count >= 49.5*hz && time_count <= 49.5*hz+10*hz)
            baseomega = PI/2.0/10;
        else if (time_count >= 76*hz && time_count<=76*hz+21*hz)
            baseomega = (PI-0.15)/2.0/10;
        else if (time_count >=103*hz)
        {
            basespeed=0;
            //else
            baseomega =0;
        }

        base_angle += baseomega /hz;
        p_r.first =  basespeed * cos(base_angle);
        p_r.second = basespeed * sin(base_angle);
        q_r.first += p_r.first * interval;
        q_r.second += p_r.second * interval;

        geometry_msgs:: Twist msg;
        micros_swarm::Neighbors<micros_swarm::NeighborBase> n(true);

        typename std::map<int, micros_swarm::NeighborBase>::iterator it;
        for(it=n.data().begin();it!=n.data().end();it++)
        {
            NeighborHandle* nh=new NeighborHandle(it->first, it->second.x, it->second.y, it->second.vx, it->second.vy);
            neighbor_list.push_back(nh);
        }

        micros_swarm::Base nl = get_base();

        my_position=pair<double,double>(nl.x, nl.y);
        my_velocity=pair<double,double>(nl.vx, nl.vy);

        msg.linear.x += (f_g().first*pm1+f_d().first*pm2+f_r().first*pm3)/hz;
        msg.linear.y += (f_g().second*pm1+f_d().second*pm2+f_r().second*pm3)/hz;
        //cout<<f_g().second<<' '<<f_d().second<<' '<<msg.linear.y<<endl;

        if (msg.linear.x >speedlimit)
            msg.linear.x=speedlimit;
        if (msg.linear.x <-speedlimit)
            msg.linear.x=-speedlimit;
        if (msg.linear.y >speedlimit)
            msg.linear.y=speedlimit;
        if (msg.linear.y <-speedlimit)
            msg.linear.y=-speedlimit;

        msg.linear.x +=p_r.first;
        msg.linear.y +=p_r.second;


        //theta handle
        geometry_msgs::Twist sendmsg;
        sendmsg.linear.x =msg.linear.x;
        sendmsg.linear.y = msg.linear.y;
        if(sendmsg.linear.x==0 && sendmsg.linear.y==0)
            pub.publish(sendmsg);
        else{
            double fi = PI/2;
            if(sendmsg.linear.x!=0)
            {
                fi=atan(sendmsg.linear.y/sendmsg.linear.x);
                if(sendmsg.linear.x<0&&sendmsg.linear.y>=0)
                    fi+=PI;
                else if (sendmsg.linear.x<0 && sendmsg.linear.y<0)
                    fi-=PI;
            }
            else if (sendmsg.linear.y<0)
                fi= -PI/2;
            double v_scale = sqrt(sendmsg.linear.x*sendmsg.linear.x+sendmsg.linear.y*sendmsg.linear.y);
            sendmsg.linear.x = v_scale*cos(fi-my_theta);
            sendmsg.linear.y = v_scale *sin(fi-my_theta);
            pub.publish(sendmsg);}
        //pub.publish(msg);

        neighbor_list.clear();
    }

    void GazeboFlocking::baseCallback(const nav_msgs::Odometry& lmsg)
    {
        float x = lmsg.pose.pose.position.x;
        float y = lmsg.pose.pose.position.y;

        float vx = lmsg.twist.twist.linear.x;
        float vy = lmsg.twist.twist.linear.y;
        my_theta = tf::getYaw(lmsg.pose.pose.orientation);
        micros_swarm::Base l(x, y, 0, vx, vy, 0, 1);
        set_base(l);
    }

    void GazeboFlocking::start()
    {
        init();

        ros::NodeHandle nh;
        ros::ServiceClient client = nh.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");
        hector_uav_msgs::EnableMotors srv;
        srv.request.enable = true;
        if (client.call(srv))
        {
            ;
        }
        else
        {
            ROS_ERROR("Failed to call service enable_motors");
        }
        
        set_dis(12);
        pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
        ros::Rate loop_rate(10);
        for(int i = 0; i < 7*10; i++) {
            geometry_msgs:: Twist upmsg;
            upmsg.linear.z = 1;
            pub.publish(upmsg);
            loop_rate.sleep();
        }
        //sub = nh.subscribe("base_pose_ground_truth", 1000, &App3::baseCallback, this, ros::TransportHints().udp());

        sub = nh.subscribe("ground_truth/state", 1000, &GazeboFlocking::baseCallback, this, ros::TransportHints().udp());
        //ros::Duration(5).sleep();  //this is necessary, in order that the runtime platform kernel of the robot has enough time to publish it's base information.

        timer = nh.createTimer(ros::Duration(interval), &GazeboFlocking::publish_cmd, this);
    }
};

