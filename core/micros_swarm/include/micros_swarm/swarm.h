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
#include "micros_swarm/msg_queue_manager.h"
#include "micros_swarm/packet_type.h"
#include "micros_swarm/serialize.h"
#include "gsdf_msgs/CommPacket.h"
#include "gsdf_msgs/JoinSwarm.h"
#include "gsdf_msgs/LeaveSwarm.h"

namespace micros_swarm{
    
    class Swarm{
        public:
            Swarm();
            Swarm(int swarm_id);
            Swarm(const Swarm& s);
            Swarm& operator=(const Swarm& s);
            ~Swarm();
            const int id() const;
            const std::set<int> members();
            void join();
            void leave();
            void select(const boost::function<bool()>& bf);
            void unselect(const boost::function<bool()>& bf);
            const bool in() const;
            //execute a function
            void execute(const boost::function<void()>& f);
            void breakup();
            const Swarm intersection(const Swarm& s, int new_swarm_id);
            const Swarm swarm_union(const Swarm& s, int new_swarm_id);
            const Swarm difference(const Swarm& s, int new_swarm_id);
            const Swarm negation(int new_swarm_id);
            void print() const;
        private:
            int swarm_id_; 
            boost::shared_ptr<micros_swarm::RuntimeHandle> rth_;
            boost::shared_ptr<micros_swarm::MsgQueueManager> mqm_;
    };
};
#endif
