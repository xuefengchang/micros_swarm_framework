/**
Software License Agreement (BSD)
\file      scds_pso.cpp
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

#include <boost/function.hpp>
#include "micros_swarm/scds_pso.h"

#define c1 1.49445
#define c2 1.49445
#define w 0.7

namespace micros_swarm {
    void Agent::set_dim(int dim)
    {
        dim_ = dim;
        min_pos_.resize(dim_);
        for(int i = 0; i < dim_; i++) {
            min_pos_[i] = 0;
        }
        max_pos_.resize(dim_);
        for(int i = 0; i < dim_; i++) {
            max_pos_[i] = 0;
        }
        min_vel_.resize(dim_);
        for(int i = 0; i < dim_; i++) {
            min_vel_[i] = 0;
        }
        max_vel_.resize(dim_);
        for(int i = 0; i < dim_; i++) {
            max_vel_[i] = 0;
        }
        cur_pos_.resize(dim_);
        for(int i = 0; i < dim_; i++) {
            cur_pos_[i] = 0;
        }
        cur_vel_.resize(dim_);
        for(int i = 0; i < dim_; i++) {
            cur_vel_[i] = 0;
        }
        /*pbest_.resize(dim_);
        for(int i = 0; i < dim_; i++) {
            pbest_[i] = 0;
        }
        gbest_.resize(dim_);
        for(int i = 0; i < dim_; i++) {
            gbest_[i] = 0;
        }*/
    }

    void Agent::set_fitness(const boost::function<float(const std::vector<float>& )>& fitness)
    {
        fitness_ = fitness;
    }

    void Agent::set_min_pos(const std::vector<float>& pos)
    {
        for(int i = 0; i < pos.size(); i++) {
            min_pos_[i] = pos[i];
        }
    }

    void Agent::set_max_pos(const std::vector<float>& pos)
    {
        for(int i = 0; i < pos.size(); i++) {
            max_pos_[i] = pos[i];
        }
    }

    void Agent::set_min_vel(const std::vector<float>& vel)
    {
        for(int i = 0; i < vel.size(); i++) {
            min_vel_[i] = vel[i];
        }
    }

    void Agent::set_max_vel(const std::vector<float>& vel)
    {
        for(int i = 0; i < vel.size(); i++) {
            max_pos_[i] = vel[i];
        }
    }

    void Agent::init_pos(const std::vector<float>& pos)
    {
        for(int i = 0; i < pos.size(); i++) {
            cur_pos_[i] = pos[i];
        }
        //pbest_ = cur_pos_;
        //pbest_val_ = fitness_(pbest_);
        pbest_.pos = cur_pos_;
        pbest_.val = fitness_(pbest_.pos);
        pbest_.robot_id = robot_id_;
        pbest_.gen = cur_gen_;
        pbest_.timestamp = time(NULL);
        //gbest_ = cur_pos_;
        //gbest_val_ = fitness_(gbest_);
        gbest_.pos = cur_pos_;
        gbest_.val = fitness_(gbest_.pos);
        gbest_.robot_id = robot_id_;
        gbest_.gen = cur_gen_;
        gbest_.timestamp = time(NULL);

        best_tuple_.put("scds_pso", gbest_);
    }

    void Agent::init_vel(const std::vector<float>& vel)
    {
        for(int i = 0; i < vel.size(); i++) {
            cur_vel_[i] = vel[i];
        }
    }

    bool Agent::has_pos_limit(int index)
    {
        if((min_pos_[index] == 0) && (max_pos_[index] == 0))
            return false;
        return true;
    }

    bool Agent::has_vel_limit(int index)
    {
        if((min_vel_[index] == 0) && (max_vel_[index] == 0))
            return false;
        return true;
    }

    void Agent::rand_init()
    {
        for(int i = 0; i < dim_; i++) {
            if(has_pos_limit(i)) {
                cur_pos_[i] = random_float(min_pos_[i], max_pos_[i], robot_id_);
            }
            else {
                cur_pos_[i] = random_float(-1000, 1000, robot_id_);
            }

            pbest_.pos = cur_pos_;
            pbest_.val = fitness_(pbest_.pos);
            pbest_.robot_id = robot_id_;
            pbest_.gen = cur_gen_;
            pbest_.timestamp = time(NULL);
            gbest_.pos = cur_pos_;
            gbest_.val = fitness_(gbest_.pos);
            gbest_.robot_id = robot_id_;
            gbest_.gen = cur_gen_;
            gbest_.timestamp = time(NULL);

            best_tuple_.put("scds_pso", gbest_);

            if(has_vel_limit(i)) {
                cur_vel_[i] = random_float(min_vel_[i], max_vel_[i], robot_id_);
            }
            else {
                cur_vel_[i] = random_float(-100, 100, robot_id_);
            }
        }
    }

    void Agent::loop(const ros::TimerEvent &)
    {
        if(!run_)
            return;
        cur_gen_++;

        float r1 = random_float(0, 1, time(NULL));
        float r2 = random_float(0, 1, time(NULL));
        SCDSPSODataTuple data_tuple = best_tuple_.get("scds_pso");

        for(int i = 0; i < dim_; i++) {
            cur_vel_[i] = w*cur_vel_[i] + c1*r1*(pbest_.pos[i] - cur_pos_[i]) + c2*r2*(gbest_.pos[i] - cur_pos_[i]);
            if(has_vel_limit(i)) {
                if(cur_vel_[i] > max_vel_[i]) {
                    cur_vel_[i] = max_vel_[i];
                }
                if(cur_vel_[i] < min_vel_[i]) {
                    cur_vel_[i] = min_vel_[i];
                }
            }

            cur_pos_[i] = cur_pos_[i] + cur_vel_[i];
            if(has_pos_limit(i)) {
                if(cur_pos_[i] > max_pos_[i]) {
                    cur_pos_[i] = max_pos_[i];
                }
                if(cur_pos_[i] < min_pos_[i]) {
                    cur_pos_[i] = min_pos_[i];
                }
            }
        }

        float cur_val = fitness_(cur_pos_);
        if(cur_val > pbest_.val) {
            pbest_.pos = cur_pos_;
            pbest_.val = cur_val;
            pbest_.robot_id = robot_id_;
            pbest_.gen = cur_gen_;
            pbest_.timestamp = time(NULL);
        }

        if(cur_val > data_tuple.val) {
            gbest_.pos = cur_pos_;
            gbest_.val = cur_val;
            gbest_.robot_id = robot_id_;
            gbest_.gen = cur_gen_;
            gbest_.timestamp = time(NULL);

            best_tuple_.put("scds_pso", gbest_);
        }

        std::cout<<"robot_id: "<<robot_id_<<", gbest: "<<gbest_.val<<", gen: "<<cur_gen_<<std::endl;

        ros::spinOnce();
        ros::Duration(0.1).sleep();

        //if(cur_gen_>15)
        //    break;
    }

    void Agent::start()
    {
        if(!dim_) {
            std::cout<<"dimension is not set!"<<std::endl;
            return;
        }

        if(fitness_ == 0) {
            std::cout<<"fitness is not set!"<<std::endl;
            return;
        }

        run_ = true;

        //ros::NodeHandle nh;
        timer = nh.createTimer(ros::Duration(0.1), &Agent::loop, this);

        /*while(run_)
        {
            cur_gen_++;
            float r1 = random_float(0, 1, time(NULL));
            float r2 = random_float(0, 1, time(NULL));
            for(int i = 0; i < dim_; i++) {
                cur_vel_[i] = w*cur_vel_[i] + c1*r1*(pbest_.pos[i] - cur_pos_[i]) + c2*r2*(gbest_.pos[i] - cur_pos_[i]);
                if(has_vel_limit(i)) {
                    if(cur_vel_[i] > max_vel_[i]) {
                        cur_vel_[i] = max_vel_[i];
                    }
                    if(cur_vel_[i] < min_vel_[i]) {
                        cur_vel_[i] = min_vel_[i];
                    }
                }

                cur_pos_[i] = cur_pos_[i] + cur_vel_[i];
                if(has_pos_limit(i)) {
                    if(cur_pos_[i] > max_pos_[i]) {
                        cur_pos_[i] = max_pos_[i];
                    }
                    if(cur_pos_[i] < min_pos_[i]) {
                        cur_pos_[i] = min_pos_[i];
                    }
                }
            }

            float cur_val = fitness_(cur_pos_);
            if(cur_val > pbest_.val) {
                pbest_.pos = cur_pos_;
                pbest_.val = cur_val;
                pbest_.robot_id = robot_id_;
                pbest_.gen = cur_gen_;
                pbest_.timestamp = time(NULL);
            }

            SCDSPSODataTuple data_tuple = best_tuple_.get("scds_pso");

            if(cur_val > data_tuple.val) {
                gbest_.pos = cur_pos_;
                gbest_.val = cur_val;
                gbest_.robot_id = robot_id_;
                gbest_.gen = cur_gen_;
                gbest_.timestamp = time(NULL);

                best_tuple_.put("scds_pso", gbest_);
            }

            std::cout<<"robot_id: "<<robot_id_<<", gbest: "<<gbest_.val<<", gen: "<<cur_gen_<<std::endl;

            ros::spinOnce();
            ros::Duration(0.2).sleep();

            if(cur_gen_>15)
                break;
        }*/
    }

    void Agent::stop()
    {
        run_ = false;
    }
};