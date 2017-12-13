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

namespace micros_swarm {
    Agent::Agent():name_(""), run_(false), dim_(0), fitness_(0)
    {
        //srand(time(NULL));
        w_ = 1;
        c1_ = 1.49445;
        c2_ = 1.49445;
        best_tuple_ = SCDSPSOTuple();
        rth_ = Singleton<RuntimeHandle>::getSingleton();
        robot_id_ = rth_->getRobotID();
        cur_gen_ = 0;
        gen_limit_ = 0;
        if(robot_id_ == 0) {
            file.open("~/catkin_ws/scds_pso.data");
        }
    }

    Agent::Agent(std::string name):name_(name), run_(false), dim_(0), fitness_(0)
    {
        //srand(time(NULL));
        w_ = 1;
        c1_ = 1.49445;
        c2_ = 1.49445;
        best_tuple_ = SCDSPSOTuple();
        rth_ = Singleton<RuntimeHandle>::getSingleton();
        robot_id_ = rth_->getRobotID();
        cur_gen_ = 0;
        gen_limit_ = 0;
        if(robot_id_ == 0) {
            file.open("~/catkin_ws/scds_pso.data");
        }
    }

    Agent::~Agent()
    {
        rth_.reset();
    }

    void Agent::set_param(float w, float c1, float c2)
    {
        w_ = w;
        c1_ = c1;
        c2_ = c2;
    }

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
            max_vel_[i] = vel[i];
        }
    }

    void Agent::init_pos(const std::vector<float>& pos)
    {
        for(int i = 0; i < pos.size(); i++) {
            cur_pos_[i] = pos[i];
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
    }

    void Agent::init_vel(const std::vector<float>& vel)
    {
        for(int i = 0; i < vel.size(); i++) {
            cur_vel_[i] = vel[i];
        }
    }

    bool Agent::has_pos_limit(int index)
    {
        if((min_pos_[index] == 0) && (max_pos_[index] == 0)) {
            return false;
        }
        return true;
    }

    bool Agent::has_vel_limit(int index)
    {
        if((min_vel_[index] == 0) && (max_vel_[index] == 0)) {
            return false;
        }
        return true;
    }

    void Agent::rand_init()
    {
        for(int i = 0; i < dim_; i++) {
            if(has_pos_limit(i)) {
                cur_pos_[i] = random_float(min_pos_[i], max_pos_[i]);
            }
            else {
                cur_pos_[i] = random_float(-5.12, 5.12);
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
                cur_vel_[i] = random_float(min_vel_[i], max_vel_[i]);
            }
            else {
                cur_vel_[i] = random_float(-5.12, 5.12);
            }
        }
    }

    void Agent::loop(const ros::TimerEvent &)
    {
        if(!run_) {
            return;
        }

        cur_gen_++;
        float  r1 = (float)rand()/RAND_MAX;
        float  r2 = (float)rand()/RAND_MAX;
        gbest_ = best_tuple_.get("scds_pso");

        for(int i = 0; i < dim_; i++) {
            float s1 = c1_*r1*(pbest_.pos[i] - cur_pos_[i]);
            float s2 = c2_*r2*(gbest_.pos[i] - cur_pos_[i]);
            cur_vel_[i] = w_*cur_vel_[i] + s1 + s2;

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

        if(cur_val > gbest_.val) {
            gbest_.pos = cur_pos_;
            gbest_.val = cur_val;
            gbest_.robot_id = robot_id_;
            gbest_.gen = cur_gen_;
            gbest_.timestamp = time(NULL);

            best_tuple_.put("scds_pso", gbest_);
        }

        //if(robot_id_ == 0) {
        //    std::cout << "robot_id: " << robot_id_ << " cur_gen: " << cur_gen_ << ", cur_val: " << cur_val
        //              << ", gbest: " << gbest_.val << ", gen: " << gbest_.gen << std::endl;
        //}

        if(robot_id_ == 0) {
            std::cout<<cur_gen_<<" "<<gbest_.val<< std::endl;
            file<<cur_gen_<<" "<<gbest_.val<< std::endl;
        }

        if(gen_limit_) {
            if(cur_gen_ > gen_limit_) {
                timer_.stop();
            }
        }
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
        timer_ = nh_.createTimer(ros::Duration(1), &Agent::loop, this);
    }

    void Agent::start(int loop_gen)
    {
        gen_limit_ = loop_gen;
        start();
    }

    void Agent::stop()
    {
        run_ = false;
        dim_ = 0;
        name_ = "";
        cur_gen_ = 0;
        gen_limit_ = 0;
    }
};