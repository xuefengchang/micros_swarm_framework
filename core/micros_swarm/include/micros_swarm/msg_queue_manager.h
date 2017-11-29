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
#include "micros_swarm/circular_queue.h"

namespace micros_swarm{

    class MsgQueueManager;

    class OutMsgQueue
    {
    public:
        OutMsgQueue(const std::string& name, int size, MsgQueueManager *manager_ptr);
        ~OutMsgQueue();
        bool full();
        bool empty();
        int size();
        const std::vector<uint8_t>& front();
        void pop();
        void push(const std::vector<uint8_t>& msg);
    private:
        std::string name_;
        int size_;
        MsgQueueManager *queue_manager_ptr_;
        boost::shared_mutex mutex_;
        boost::shared_ptr<cqueue<std::vector<uint8_t> > > queue_;
    };

    class MsgQueueManager
    {
        public:
            MsgQueueManager();
            ~MsgQueueManager();
            void createOutMsgQueue(std::string name, int size);
            OutMsgQueue* getOutMsgQueue(std::string name);
            void spinAllOutMsgQueueOnce(std::vector<std::vector<uint8_t> >& msg_vec);
            bool allOutMsgQueueEmpty();
            
            boost::shared_mutex msg_queue_mutex;
            boost::condition_variable_any msg_queue_condition;
        private:
            std::map<std::string, OutMsgQueue*> queue_map_;
    };
};

#endif
