/**
Software License Agreement (BSD)
\file      swarm.cpp
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

#include "micros_swarm/swarm.h"

namespace micros_swarm{

    Swarm::Swarm()
    {
        swarm_id_ = -1;
    }

    Swarm::Swarm(int swarm_id)
    {
        swarm_id_ = swarm_id;
        rth_ = Singleton<RuntimeHandle>::getSingleton();
        mqm_ = Singleton<MsgQueueManager>::getSingleton();
        rth_->insertOrUpdateSwarm(swarm_id_, 0);
    }

    Swarm::Swarm(const Swarm& s)
    {
        rth_ = Singleton<RuntimeHandle>::getSingleton();
        mqm_ = Singleton<MsgQueueManager>::getSingleton();
        swarm_id_ = s.swarm_id_;
    }

    Swarm& Swarm::operator=(const Swarm& s)
    {
        if(this == &s) {
            return *this;
        }
        rth_ = Singleton<RuntimeHandle>::getSingleton();
        mqm_ = Singleton<MsgQueueManager>::getSingleton();
        swarm_id_ = s.swarm_id_;
        return *this;
    }

    Swarm::~Swarm()
    {
        rth_.reset();
        mqm_.reset();
    }

    const int Swarm::id()  const
    {
        return swarm_id_;
    }

    const std::set<int> Swarm::members()
    {
        std::set<int> s;
        rth_->getSwarmMembers(swarm_id_, s);

        return s;
    }

    void Swarm::join()
    {
        int robot_id = rth_->getRobotID();
        rth_->insertOrUpdateSwarm(swarm_id_, 1);

        gsdf_msgs::JoinSwarm js;
        js.robot_id = robot_id;
        js.swarm_id = swarm_id_;
        std::vector<uint8_t> js_vec = serialize_ros(js);

        gsdf_msgs::CommPacket p;
        p.header.source = robot_id;
        p.header.type = SINGLE_ROBOT_JOIN_SWARM;
        p.header.data_len = js_vec.size();
        p.header.version = 1;
        p.header.checksum = 0;
        p.content.buf = js_vec;
        std::vector<uint8_t> msg_data = serialize_ros(p);
        mqm_->getOutMsgQueue("swarm")->push(msg_data);
    }

    void Swarm::leave()
    {
        int robot_id = rth_->getRobotID();
        rth_->insertOrUpdateSwarm(swarm_id_, 0);

        gsdf_msgs::LeaveSwarm ls;
        ls.robot_id = robot_id;
        ls.swarm_id = swarm_id_;
        std::vector<uint8_t> ls_vec = serialize_ros(ls);

        gsdf_msgs::CommPacket p;
        p.header.source = robot_id;
        p.header.type = SINGLE_ROBOT_LEAVE_SWARM;
        p.header.data_len = ls_vec.size();
        p.header.version = 1;
        p.header.checksum = 0;
        p.content.buf = ls_vec;
        std::vector<uint8_t> msg_data = serialize_ros(p);
        mqm_->getOutMsgQueue("swarm")->push(msg_data);
    }

    void Swarm::select(const boost::function<bool()>& bf)
    {
        if(bf()) {
            join();
        }
        else {
            //do nothiong
        }
    }

    void Swarm::unselect(const boost::function<bool()>& bf)
    {
        if(bf()) {
            leave();
        }
        else {
            //do nothiong
        }
    }

    const bool Swarm::in() const
    {
        if(rth_->getSwarmFlag(swarm_id_)) {
            return true;
        }
        return false;
    }

    //execute a function
    void Swarm::execute(const boost::function<void()>& f)
    {
        if(in()) {
            f();
        }
    }

    void Swarm::breakup()
    {
        if(in()) {
            leave();
        }
        rth_->deleteSwarm(swarm_id_);
        this->~Swarm();
    }

    const Swarm Swarm::intersection(const Swarm& s, int new_swarm_id)
    {
        std::set<int> result;
        result.clear();

        std::set<int> a;
        rth_->getSwarmMembers(swarm_id_, a);
        std::set<int> b;
        rth_->getSwarmMembers(s.id(), b);

        std::set_intersection(a.begin(), a.end(), b.begin(), b.end(),
                              std::insert_iterator<std::set<int> >(result, result.begin()));

        Swarm result_swarm(new_swarm_id);

        int robot_id=rth_->getRobotID();

        std::set<int>::iterator it;
        it = result.find(robot_id);
        if(it != result.end()) {
            result_swarm.join();
        }

        return result_swarm;
    }

    const Swarm Swarm::swarm_union(const Swarm& s, int new_swarm_id)
    {
        std::set<int> result;
        result.clear();

        std::set<int> a;
        rth_->getSwarmMembers(swarm_id_, a);
        std::set<int> b;
        rth_->getSwarmMembers(s.id(), b);

        std::set_union(a.begin(), a.end(), b.begin(), b.end(),
                       std::insert_iterator<std::set<int> >(result, result.begin()));

        Swarm result_swarm(new_swarm_id);

        int robot_id = rth_->getRobotID();

        std::set<int>::iterator it;
        it = result.find(robot_id);
        if(it != result.end()) {
            result_swarm.join();
        }

        return result_swarm;
    }

    const Swarm Swarm::difference(const Swarm& s, int new_swarm_id)
    {
        std::set<int> result;

        std::set<int> a;
        rth_->getSwarmMembers(swarm_id_, a);
        std::set<int> b;
        rth_->getSwarmMembers(s.id(), b);

        std::set_difference(a.begin(), a.end(), b.begin(), b.end(),
                            std::insert_iterator<std::set<int> >(result, result.begin()));

        Swarm result_swarm(new_swarm_id);

        int robot_id = rth_->getRobotID();

        std::set<int>::iterator it;
        it = result.find(robot_id);
        if(it != result.end()) {
            result_swarm.join();
        }

        return result_swarm;
    }

    const Swarm Swarm::negation(int new_swarm_id)
    {
        Swarm result_swarm(new_swarm_id);
        std::set<int> a;
        rth_->getSwarmMembers(swarm_id_, a);
        int robot_id = rth_->getRobotID();

        std::set<int>::iterator it;
        it = a.find(robot_id);
        if(it == a.end()) {
            result_swarm.join();
        }

        return result_swarm;
    }

    void Swarm::print() const
    {
        std::set<int> s;
        rth_->getSwarmMembers(swarm_id_, s);

        int robot_id = rth_->getRobotID();

        std::set<int>::iterator it;
        std::cout<<"swarm "<<swarm_id_<<" members: "<<std::endl;
        for(it = s.begin(); it != s.end(); it++) {
            std::cout<<*it<<", ";
        }
        std::cout<<std::endl;
    }
};
