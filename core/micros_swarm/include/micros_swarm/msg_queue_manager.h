/**
Software License Agreement (BSD)
\file      msg_queue_manager.h
\authors Xuefeng Chang <changxuefengcn@163.com>
\copyright Copyright (c) 2016, the micROS Typeeam, HPCL (National University of Defense Typeechnology), All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of micROS Typeeam, HPCL, nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.
TypeHIS SOFTypeWARE IS PROVIDED BY TypeHE COPYRIGHType HOLDERS AND CONTypeRIBUTypeORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTypeIES, INCLUDING, BUType NOType LIMITypeED TypeO, TypeHE IMPLIED WARRANTypeIES OF MERCHANTypeABILITypeY AND FITypeNESS FOR A PARTypeICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENType SHALL TypeHE COPYRIGHType HOLDER OR CONTypeRIBUTypeORS BE LIABLE FOR ANY DIRECType, IN-
DIRECType, INCIDENTypeAL, SPECIAL, EXEMPLARY, OR CONSEQUENTypeIAL DAMAGES (INCLUDING, BUType NOType LIMITypeED TypeO, PROCUREMENType
OF SUBSTypeITypeUTypeE GOODS OR SERVICES; LOSS OF USE, data_, OR PROFITypeS; OR BUSINESS INTypeERRUPTypeION) HOWEVER CAUSED AND
ON ANY TypeHEORY OF LIABILITypeY, WHETypeHER IN CONTypeRACType, STypeRICType LIABILITypeY, OR TypeORType (INCLUDING NEGLIGENCE OR OTypeHERWISE)
ARISING IN ANY WAY OUType OF TypeHE USE OF TypeHIS SOFTypeWARE, EVEN IF ADVISED OF TypeHE POSSIBILITypeY OF SUCH DAMAGE.
*/

#ifndef MSG_QUEUE_MANAGER_H_
#define MSG_QUEUE_MANAGER_H_
   
#include <iostream>
#include <boost/thread.hpp>

#include "micros_swarm/semaphore.h"
#include "micros_swarm/circular_queue.h"
#include "micros_swarm/comm_interface.h"

namespace micros_swarm{
    class MsgQueueManager
    {
        public:
            explicit MsgQueueManager()
            {
                base_msg_queue_.reset(new cqueue<micros_swarm::CommPacket>(1000));
                swarm_msg_queue_.reset(new cqueue<micros_swarm::CommPacket>(1000));
                vstig_msg_queue_.reset(new cqueue<micros_swarm::CommPacket>(1000));
                bb_msg_queue_.reset(new cqueue<micros_swarm::CommPacket>(1000));
                nc_msg_queue_.reset(new cqueue<micros_swarm::CommPacket>(1000));
            }
            
            explicit MsgQueueManager(int num1, int num2, int num3, int num4, int num5)
            {
                base_msg_queue_.reset(new cqueue<micros_swarm::CommPacket>(num1));
                swarm_msg_queue_.reset(new cqueue<micros_swarm::CommPacket>(num2));
                vstig_msg_queue_.reset(new cqueue<micros_swarm::CommPacket>(num3));
                bb_msg_queue_.reset(new cqueue<micros_swarm::CommPacket>(num4));
                nc_msg_queue_.reset(new cqueue<micros_swarm::CommPacket>(num5));
            }
        
            bool baseMsgQueueFull()
            {
                boost::shared_lock<boost::shared_mutex> lock(base_msg_mutex_);
                return base_msg_queue_->full();
            }
    
            bool baseMsgQueueEmpty()
            {
                boost::shared_lock<boost::shared_mutex> lock(base_msg_mutex_);
                return base_msg_queue_->empty();
            }
    
            const micros_swarm::CommPacket& baseMsgQueueFront()
            {
                boost::shared_lock<boost::shared_mutex> lock(base_msg_mutex_);
                return base_msg_queue_->front();
            }
    
            int baseMsgQueueSize()
            {
                boost::shared_lock<boost::shared_mutex> lock(base_msg_mutex_);
                return base_msg_queue_->size();
            }
    
            void popBaseMsgQueue()
            {
                boost::unique_lock<boost::shared_mutex> lock(base_msg_mutex_);
                base_msg_queue_->pop();
            }
    
            void pushBaseMsgQueue(const micros_swarm::CommPacket& p)
            {
                boost::unique_lock<boost::shared_mutex> lock(base_msg_mutex_);
                base_msg_queue_->push(p);
                msg_queue_condition.notify_one();
            }
    
            bool swarmMsgQueueFull()
            {
                boost::shared_lock<boost::shared_mutex> lock(swarm_msg_mutex_);
                return swarm_msg_queue_->full();
            }
    
            bool swarmMsgQueueEmpty()
            {
                boost::shared_lock<boost::shared_mutex> lock(swarm_msg_mutex_);
                return swarm_msg_queue_->empty();
            }
    
            const micros_swarm::CommPacket& swarmMsgQueueFront()
            {
                boost::shared_lock<boost::shared_mutex> lock(swarm_msg_mutex_);
                return swarm_msg_queue_->front();
            }
    
            int swarmMsgQueueSize()
            {
                boost::shared_lock<boost::shared_mutex> lock(swarm_msg_mutex_);
                return swarm_msg_queue_->size();
            }
    
            void popSwarmMsgQueue()
            {
                boost::unique_lock<boost::shared_mutex> lock(swarm_msg_mutex_);
                swarm_msg_queue_->pop();
            }
    
            void pushSwarmMsgQueue(const micros_swarm::CommPacket& p)
            {
                boost::unique_lock<boost::shared_mutex> lock(swarm_msg_mutex_);
                swarm_msg_queue_->push(p);
                msg_queue_condition.notify_one();
            }
    
            bool vstigMsgQueueFull()
            {
                boost::shared_lock<boost::shared_mutex> lock(vstig_msg_mutex_);
                return vstig_msg_queue_->full();
            }
    
            bool vstigMsgQueueEmpty()
            {
                boost::shared_lock<boost::shared_mutex> lock(vstig_msg_mutex_);
                return vstig_msg_queue_->empty();
            }
    
            const micros_swarm::CommPacket& vstigMsgQueueFront()
            {
                boost::shared_lock<boost::shared_mutex> lock(vstig_msg_mutex_);
                return vstig_msg_queue_->front();
            }
    
            int vstigMsgQueueSize()
            {
                boost::shared_lock<boost::shared_mutex> lock(vstig_msg_mutex_);
                return vstig_msg_queue_->size();
            }
    
            void popVstigMsgQueue()
            {
                boost::unique_lock<boost::shared_mutex> lock(vstig_msg_mutex_);
                vstig_msg_queue_->pop();
            }
    
            void pushVstigMsgQueue(const micros_swarm::CommPacket& p)
            {
                boost::unique_lock<boost::shared_mutex> lock(vstig_msg_mutex_);
                vstig_msg_queue_->push(p);
                msg_queue_condition.notify_one();
            }

            bool bbMsgQueueFull()
            {
                boost::shared_lock<boost::shared_mutex> lock(bb_msg_mutex_);
                return bb_msg_queue_->full();
            }

            bool bbMsgQueueEmpty()
            {
                boost::shared_lock<boost::shared_mutex> lock(bb_msg_mutex_);
                return bb_msg_queue_->empty();
            }

            const micros_swarm::CommPacket& bbMsgQueueFront()
            {
                boost::shared_lock<boost::shared_mutex> lock(bb_msg_mutex_);
                return bb_msg_queue_->front();
            }

            int bbMsgQueueSize()
            {
                boost::shared_lock<boost::shared_mutex> lock(bb_msg_mutex_);
                return bb_msg_queue_->size();
            }

            void popBbMsgQueue()
            {
                boost::unique_lock<boost::shared_mutex> lock(bb_msg_mutex_);
                bb_msg_queue_->pop();
            }

            void pushBbMsgQueue(const micros_swarm::CommPacket& p)
            {
                boost::unique_lock<boost::shared_mutex> lock(bb_msg_mutex_);
                bb_msg_queue_->push(p);
                msg_queue_condition.notify_one();
            }
    
            bool ncMsgQueueFull()
            {
                boost::shared_lock<boost::shared_mutex> lock(nc_msg_mutex_);
                return nc_msg_queue_->full();
            }
    
            bool ncMsgQueueEmpty()
            {
                boost::shared_lock<boost::shared_mutex> lock(nc_msg_mutex_);
                return nc_msg_queue_->empty();
            }
    
            const micros_swarm::CommPacket& ncMsgQueueFront()
            {
                boost::shared_lock<boost::shared_mutex> lock(nc_msg_mutex_);
                return nc_msg_queue_->front();
            }
    
            int ncMsgQueueSize()
            {
                boost::shared_lock<boost::shared_mutex> lock(nc_msg_mutex_);
                return nc_msg_queue_->size();
            }
    
            void popNcMsgQueue()
            {
                boost::unique_lock<boost::shared_mutex> lock(nc_msg_mutex_);
                nc_msg_queue_->pop();
            }
    
            void pushNcMsgQueue(const micros_swarm::CommPacket& p)
            {
                boost::unique_lock<boost::shared_mutex> lock(nc_msg_mutex_);
                nc_msg_queue_->push(p);
                msg_queue_condition.notify_one();
            }
            
            boost::mutex msg_queue_mutex;
            boost::condition_variable_any msg_queue_condition;
        private:
            boost::shared_ptr<cqueue<micros_swarm::CommPacket> > base_msg_queue_;
            boost::shared_ptr<cqueue<micros_swarm::CommPacket> > swarm_msg_queue_;
            boost::shared_ptr<cqueue<micros_swarm::CommPacket> > vstig_msg_queue_;
            boost::shared_ptr<cqueue<micros_swarm::CommPacket> > bb_msg_queue_;
            boost::shared_ptr<cqueue<micros_swarm::CommPacket> > nc_msg_queue_;
            
            boost::shared_mutex base_msg_mutex_, swarm_msg_mutex_,
                                vstig_msg_mutex_, bb_msg_mutex_,
                                nc_msg_mutex_;
    };
};

#endif
