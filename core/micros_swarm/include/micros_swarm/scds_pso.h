/**
Software License Agreement (BSD)
\file      scds_pso.h
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

#ifndef SCDS_PSO_H_
#define SCDS_PSO_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <set>
#include <boost/function.hpp>
#include <ros/ros.h>

#include "micros_swarm/scds_pso_tuple.h"

namespace micros_swarm{

    class Agent{
        public:
            Agent();
            Agent(std::string name);
            ~Agent();
            void set_param(float w, float c1, float c2);
            void set_dim(int dim);
            void set_fitness(const boost::function<float(const std::vector<float>& )>& fitness);
            void set_min_pos(const std::vector<float>& pos);
            void set_max_pos(const std::vector<float>& pos);
            void set_min_vel(const std::vector<float>& pos);
            void set_max_vel(const std::vector<float>& pos);
            void init_pos(const std::vector<float>& pos);
            void init_vel(const std::vector<float>& vel);
            void rand_init();
            void start();
            void start(int loop_gen);
            void stop();
        private:
            bool has_pos_limit(int index);
            bool has_vel_limit(int index);

            void loop(const ros::TimerEvent&);

            std::string name_;
            bool run_;
            int dim_;
            float w_;
            float c1_;
            float c2_;
            boost::function<float(const std::vector<float>& )> fitness_;
            std::vector<float> min_pos_;
            std::vector<float> max_pos_;
            std::vector<float> min_vel_;
            std::vector<float> max_vel_;
            std::vector<float> cur_pos_;
            std::vector<float> cur_vel_;
            int cur_gen_;
            int gen_limit_;
            SCDSPSODataTuple pbest_;
            SCDSPSODataTuple gbest_;
            SCDSPSOTuple best_tuple_;

            int robot_id_;
            boost::shared_ptr<RuntimeHandle> rth_;

            ros::NodeHandle nh_;
            ros::Timer timer_;

            std::ofstream file;
    };
};

#endif