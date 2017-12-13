/**
Software License Agreement (BSD)
\file      uav_statistic.cpp
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
#include <fstream>
#include <sstream>
#include <ros/ros.h>
#include <math.h>

#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

using namespace std;

int SWARM_SIZE = 3;
int NEI_DIS = 5;
int COM_DIS = 12;

struct Position{
    float x;
    float y;
};

float pos_dis(Position self, Position nei)
{
    return sqrt((self.x - nei.x)*(self.x - nei.x) + (self.y - nei.y)*(self.y - nei.y));
}

std::map<int, Position> pos_map;
std::map<int, std::vector<int> > neighbor_map;

void callback(const nav_msgs::OdometryConstPtr &lmsg, int r_id)
{
    Position p;
    p.x = lmsg->pose.pose.position.x;
    p.y = lmsg->pose.pose.position.y;
    std::map<int, Position >::iterator pos_it;
    pos_it = pos_map.find(r_id);
    if(pos_it != pos_map.end()) {
        pos_it->second = p;
    }
    else {
        pos_map.insert(std::pair<int, Position>(r_id, p));
    }

    /*Position self = p;
    Position nei;
    std::vector<int> neis;
    std::map<int, Position >::iterator it;
    for(it = pos_map.begin(); it != pos_map.end(); it++) {
        if(it->first == r_id) {
            continue;
        }
        else {
            nei = it->second;
            if (pos_dis(self, nei) < COM_DIS) {
                neis.push_back(it->first);
            }
        }
    }

    std::map<int, std::vector<int> >::iterator nei_it;
    nei_it = neighbor_map.find(r_id);
    if(nei_it != neighbor_map.end()) {
        nei_it->second = neis;
    }
    else {
        neighbor_map.insert(std::pair < int, std::vector < int > > (r_id, neis));
    }*/
}

Position get_pos(int r_id)
{
    std::map<int, Position >::iterator it;
    it = pos_map.find(r_id);
    if(it != pos_map.end()) {
        return it->second;
    }

    Position p;
    p.x = 0;
    p.y = 0;
    return  p;
}

std::vector<int> get_neighbor(int r_id)
{
    std::map<int, std::vector<int> >::iterator it;
    it = neighbor_map.find(r_id);
    if(it != neighbor_map.end()) {
        return it->second;
    }

    std::vector<int> nei;
    nei.clear();
    return  nei;
}

float nei_dis(int id1, int id2)
{
    Position p1 = get_pos(id1);
    Position p2 = get_pos(id2);

    return pos_dis(p1, p2);
}

float cal_error()
{
    float err = 0;
    int count = 0;
    for(int i = 0; i < SWARM_SIZE; i++) {
        std::vector<int> nei = get_neighbor(i);
        for(int j = 0; j < nei.size(); j++) {
            err += abs((nei_dis(i, nei[j]) - NEI_DIS));
            count++;
        }
    }

    if(count > 0) {
        return err/count;
    }
    else {
        return -1;
    }
}

void process()
{
    while(1) {
        for (int i = 0; i < SWARM_SIZE; i++) {
            int r_id = i;
            Position self = get_pos(i);
            Position nei;
            std::vector<int> neis;
            std::map<int, Position>::iterator it;
            for (it = pos_map.begin(); it != pos_map.end(); it++) {
                if (it->first == r_id) {
                    continue;
                } else {
                    nei = it->second;
                    if (pos_dis(self, nei) < COM_DIS) {
                        neis.push_back(it->first);
                    }
                }
            }

            std::map < int, std::vector < int > > ::iterator nei_it;
            nei_it = neighbor_map.find(r_id);
            if (nei_it != neighbor_map.end()) {
                nei_it->second = neis;
            } else {
                neighbor_map.insert(std::pair < int, std::vector < int > > (r_id, neis));
            }
        }
        ros::Duration(0.1).sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "app3_statistic");
    ros::NodeHandle nh;

    std::vector<ros::Subscriber> sub_vec;
    sub_vec.resize(SWARM_SIZE);

    for(int i = 0; i < SWARM_SIZE; i++) {
        stringstream ss;
        ss<<"/uav"<<i<<"/fix_odom";
        sub_vec[i] = nh.subscribe<nav_msgs::Odometry>(ss.str(), 1000, boost::bind(callback, _1, i));
    }

    ros::Duration(1).sleep();

    boost::thread* process_thread_ = new boost::thread(process);

    ofstream file;
    file.open("uav_statistic.txt");

    int step = 0;
    while(ros::ok()) {
        float err = cal_error();
        std::cout<<step<<" "<<err<<std::endl;
        file<<step<<" "<<err<<std::endl;
        step++;
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }

    ros::spin();
    return 0;
}
