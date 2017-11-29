/**
Software License Agreement (BSD)
\file      neighbor_comm.h 
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

#ifndef NEIGHBOR_COMM_H_
#define NEIGHBOR_COMM_H_

#include <iostream>

#include "micros_swarm/runtime_handle.h"
#include "micros_swarm/listener_helper.h"
#include "micros_swarm/msg_queue_manager.h"
#include "gsdf_msgs/NeighborBroadcastKeyValue.h"

namespace micros_swarm{
    
    template<class Type>
    class Broadcaster{
        public:
            Broadcaster(const std::string& key)
            {
                key_ = key;
                rth_ = Singleton<RuntimeHandle>::getSingleton();
                mqm_ = Singleton<MsgQueueManager>::getSingleton();
            }

            ~Broadcaster()
            {
                rth_.reset();
                mqm_.reset();
            }
            
            void broadcast(const Type& value)
            {
                std::vector<uint8_t> value_vec = serialize_ros(value);
                gsdf_msgs::NeighborBroadcastKeyValue nbkv;

                nbkv.key = key_;
                nbkv.value = value_vec;
                std::vector<uint8_t> nbkv_vec = serialize_ros(nbkv);
                
                gsdf_msgs::CommPacket p;
                p.header.source = rth_->getRobotID();
                p.header.type = NEIGHBOR_BROADCAST_KEY_VALUE;
                p.header.data_len = nbkv_vec.size();
                p.header.version = 1;
                p.header.checksum = 0;
                p.content.buf = nbkv_vec;
                std::vector<uint8_t> msg_data = serialize_ros(p);
                mqm_->getOutMsgQueue("nc")->push(msg_data);
            }
        private:
            boost::shared_ptr<RuntimeHandle> rth_;
            boost::shared_ptr<micros_swarm::MsgQueueManager> mqm_;
            std::string key_;
    };
    
    template<typename Type>
    class Listener{
        public:        
            Listener(const std::string& key, const boost::function<void(const Type&)>& callback)
            {
                key_ = key;
                rth_ = Singleton<RuntimeHandle>::getSingleton();
                
                helper_.reset(new ListenerHelperT<Type>(key, callback));
                rth_->insertOrUpdateListenerHelper(key_, helper_);
            }

            ~Listener()
            {
                rth_.reset();
                helper_.reset();
            }
            
            void ignore()
            {
                rth_->deleteListenerHelper(key_);
            }
        private:
            std::string key_;
            boost::shared_ptr<RuntimeHandle> rth_;
            boost::shared_ptr<ListenerHelper> helper_;
    };
};
#endif
