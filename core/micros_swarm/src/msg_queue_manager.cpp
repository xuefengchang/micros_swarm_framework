/**
Software License Agreement (BSD)
\file      msg_queue_manager.cpp
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

#include "micros_swarm/msg_queue_manager.h"

namespace micros_swarm{

    OutMsgQueue::OutMsgQueue(const std::string& name, int size, MsgQueueManager *manager_ptr): name_(name), size_(size), queue_manager_ptr_(manager_ptr)
    {
        queue_.reset(new cqueue<std::vector<uint8_t> >(size_));
    }

    OutMsgQueue::~OutMsgQueue()
    {
        queue_.reset();
    }

    bool OutMsgQueue::full()
    {
        boost::shared_lock<boost::shared_mutex> lock(mutex_);
        return queue_->full();
    }

    bool OutMsgQueue::empty()
    {
        boost::shared_lock<boost::shared_mutex> lock(mutex_);
        return queue_->empty();
    }

    int OutMsgQueue::size()
    {
        boost::shared_lock<boost::shared_mutex> lock(mutex_);
        return queue_->size();
    }

    const std::vector<uint8_t>& OutMsgQueue::front()
    {
        boost::shared_lock<boost::shared_mutex> lock(mutex_);
        return queue_->front();
    }

    void OutMsgQueue::pop()
    {
        boost::unique_lock<boost::shared_mutex> lock(mutex_);
        queue_->pop();
    }

    void OutMsgQueue::push(const std::vector<uint8_t>& msg)
    {
        boost::unique_lock<boost::shared_mutex> lock(mutex_);
        queue_->push(msg);
        queue_manager_ptr_->msg_queue_condition.notify_one();
    }

    MsgQueueManager::MsgQueueManager()
    {
        queue_map_.clear();
    }

    MsgQueueManager::~MsgQueueManager()
    {
        std::map<std::string, OutMsgQueue*>::iterator it;
        for(it = queue_map_.begin(); it != queue_map_.end(); it++) {
            delete (it->second);
        }
        queue_map_.clear();
    }

    void MsgQueueManager::createOutMsgQueue(std::string name, int size)
    {
        OutMsgQueue *queue = new OutMsgQueue(name, size, this);
        queue_map_.insert(std::pair<std::string, OutMsgQueue*>(name, queue));
    }

    OutMsgQueue* MsgQueueManager::getOutMsgQueue(std::string name)
    {
        std::map<std::string, OutMsgQueue*>::iterator it = queue_map_.find(name);
        if(it != queue_map_.end()) {
            return it->second;
        }
        else {
            std::cout<<"get out msg queue "<<name<<" failed!"<<std::endl;
            exit(-1);
        }
    }

    void MsgQueueManager::spinAllOutMsgQueueOnce(std::vector<std::vector<uint8_t> >& msg_vec)
    {
        boost::shared_lock<boost::shared_mutex> lock(msg_queue_mutex);
        std::map<std::string, OutMsgQueue*>::iterator it;
        for(it = queue_map_.begin(); it != queue_map_.end(); it++) {
            std::string name = it->first;
            OutMsgQueue *msg_queue = getOutMsgQueue(name);
            if(name == "scds_pso") {
                for(int i = 0; i < msg_queue->size(); i++) {
                    msg_vec.push_back(msg_queue->front());
                    msg_queue->pop();
                }
            }
            else {
                if (!msg_queue->empty()) {
                    msg_vec.push_back(msg_queue->front());
                    msg_queue->pop();
                }
            }
        }
        while(allOutMsgQueueEmpty()) {
            msg_queue_condition.wait(lock);
        }
    }

    bool MsgQueueManager::allOutMsgQueueEmpty()
    {
        std::map<std::string, OutMsgQueue*>::iterator it;
        for(it = queue_map_.begin(); it != queue_map_.end(); it++) {
            if(!it->second->empty()) {
                return false;
            }
        }
        return true;
    }
};
