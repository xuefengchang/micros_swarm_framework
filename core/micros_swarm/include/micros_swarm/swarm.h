/**
Software License Agreement (BSD)
\file      swarm.h
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

#ifndef SWARM_H_
#define SWARM_H_

#include <iostream>
#include <vector>
#include <set>
#include <ros/ros.h>

#include "micros_swarm/singleton.h"
#include "micros_swarm/runtime_handle.h"
#include "micros_swarm/comm_interface.h"
#include "gsdf_msgs/JoinSwarm.h"
#include "gsdf_msgs/LeaveSwarm.h"
#include "micros_swarm/packet_type.h"
#include "micros_swarm/serialize.h"

namespace micros_swarm{
    
    class Swarm{
        public:
            Swarm()
            {
                swarm_id_ = -1;
            }
            
            Swarm(int swarm_id)
            {
                swarm_id_ = swarm_id;
                rth_ = Singleton<RuntimeHandle>::getSingleton();
                communicator_ = Singleton<micros_swarm::CommInterface>::getExistedSingleton();
                rth_->insertOrUpdateSwarm(swarm_id_, 0);
            }
            
            Swarm(const Swarm& s)
            {
                rth_ = Singleton<RuntimeHandle>::getSingleton();
                communicator_ = Singleton<micros_swarm::CommInterface>::getExistedSingleton();
                swarm_id_ = s.swarm_id_;
            }
            
            Swarm& operator=(const Swarm& s)
            {
                if(this == &s) {
                    return *this;
                }
                rth_ = Singleton<RuntimeHandle>::getSingleton();
                communicator_ = Singleton<CommInterface>::getExistedSingleton();
                swarm_id_ = s.swarm_id_;
                return *this;
            }
            
            ~Swarm()
            {
                rth_.reset();
                communicator_.reset();
            }
            
            const int id()  const
            {
                return swarm_id_;
            }
            
            const std::set<int> members()
            {
                std::set<int> s;
                rth_->getSwarmMembers(swarm_id_, s);
                
                return s;
            }
            
            void join()
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
                rth_->getOutMsgQueue()->pushSwarmMsgQueue(msg_data);
            }
            
            void leave()
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
                rth_->getOutMsgQueue()->pushSwarmMsgQueue(msg_data);
            }
            
            void select(const boost::function<bool()>& bf)
            {
                if(bf()) {
                    join();
                }
                else {
                    //do nothiong
                }
            }
            
            void unselect(const boost::function<bool()>& bf)
            {
                if(bf()) {
                    leave();
                }
                else {
                    //do nothiong
                }
            }
            
            const bool in() const
            {
                if(rth_->getSwarmFlag(swarm_id_)) {
                    return true;
                }
                return false;
            }
            
            //execute a function
            void execute(const boost::function<void()>& f)
            {
                if(in()) {
                    f();
                }
            }
            
            void breakup()
            {
                if(in()) {
                    leave();
                }
                rth_->deleteSwarm(swarm_id_);
                this->~Swarm();
            }
            
            const Swarm intersection(const Swarm& s, int new_swarm_id)
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
            
            const Swarm swarm_union(const Swarm& s, int new_swarm_id)
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
            
            const Swarm difference(const Swarm& s, int new_swarm_id)
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
            
            const Swarm negation(int new_swarm_id)
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
            
            void print() const
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
        private:
            int swarm_id_; 
            boost::shared_ptr<RuntimeHandle> rth_;
            boost::shared_ptr<CommInterface> communicator_;
    };
};
#endif
