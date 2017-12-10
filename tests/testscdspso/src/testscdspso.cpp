/**
Software License Agreement (BSD)
\file      testvstig.cpp 
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

#include "testscdspso/testscdspso.h"

#define PI 3.1415926

// Register the application
PLUGINLIB_EXPORT_CLASS(testscdspso::TestSCDSPSO, micros_swarm::Application)

namespace testscdspso{

    TestSCDSPSO::TestSCDSPSO() {}

    TestSCDSPSO::~TestSCDSPSO() {}

    void TestSCDSPSO::init() {}

    void TestSCDSPSO::stop() {}
    
    void TestSCDSPSO::loop(const ros::TimerEvent&)
    {
        static int i=0;
        if(i<50)
            i++;
        micros_swarm::SCDSPSODataTuple data;
        data.val = get_id() + i;
        tuple.put("test_scds_pso", data);

        micros_swarm::SCDSPSODataTuple best = tuple.get("test_scds_pso");
        std::cout<<"robot: "<<get_id()<<", best: "<<best.val<<std::endl;
    }

    void TestSCDSPSO::baseCallback(const nav_msgs::Odometry& lmsg)
    {
        float x=lmsg.pose.pose.position.x;
        float y=lmsg.pose.pose.position.y;

        float vx=lmsg.twist.twist.linear.x;
        float vy=lmsg.twist.twist.linear.y;

        micros_swarm::Base l(x, y, 0, vx, vy, 0, 1);
        set_base(l);
    }

    float TestSCDSPSO::fitness(const std::vector<float>& vec)
    {
        float x = vec[0];
        float y = vec[1];
        float fitness = -(20+x*x+y*y-10*cos(2*PI*x)-10*cos(2*PI*y));
        return fitness;
    }
    
    void TestSCDSPSO::start()
    {
        agent.set_param(1, 1.49445, 1.49445);
        agent.set_dim(2);
        agent.set_fitness(boost::bind(&TestSCDSPSO::fitness, this, _1));

        std::vector<float> max_pos;
        max_pos.resize(2);
        max_pos[0] = 5.12;
        max_pos[1] = 5.12;
        agent.set_max_pos(max_pos);

        std::vector<float> min_pos;
        min_pos.resize(2);
        min_pos[0] = -5.12;
        min_pos[1] = -5.12;
        agent.set_min_pos(min_pos);

        std::vector<float> max_vel;
        max_vel.resize(2);
        max_vel[0] = 1;
        max_vel[1] = 1;
        agent.set_max_vel(max_vel);

        std::vector<float> min_vel;
        min_vel.resize(2);
        min_vel[0] = -1;
        min_vel[1] = -1;
        agent.set_min_vel(min_vel);

        agent.rand_init();
        agent.start(1500);
    }
};

