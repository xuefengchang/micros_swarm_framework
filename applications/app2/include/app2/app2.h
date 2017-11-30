/**
Software License Agreement (BSD)
\file      app2.h
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

#ifndef APP2_H_
#define APP2_H_

#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

#include "micros_swarm/micros_swarm.h"

namespace app2{
    
    struct XY;
    
    class App2 : public micros_swarm::Application
    {
        public:
            ros::Timer red_timer;
            ros::Timer blue_timer;
            ros::Publisher pub;
            ros::Subscriber sub;
            
            //app parameters
            int delta_kin;
            int epsilon_kin;
            int delta_nonkin;
            int epsilon_nonkin;
            
            App2();
            ~App2();
            virtual void init();
            virtual void start();
            virtual void stop();
            
            //app functions
            float force_mag_kin(float dist);
            float force_mag_nonkin(float dist);
            XY force_sum_kin(micros_swarm::NeighborBase n, XY &s);
            XY force_sum_nonkin(micros_swarm::NeighborBase n, XY &s);
            XY direction_red();
            XY direction_blue();
            bool red(int id);
            bool blue(int id);
            void motion_red();
            void motion_blue();
            void publish_red_cmd(const ros::TimerEvent&);
            void publish_blue_cmd(const ros::TimerEvent&);
            void baseCallback(const nav_msgs::Odometry& lmsg);  
    };
};

#endif
