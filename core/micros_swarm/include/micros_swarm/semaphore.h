/**
Software License Agreement (BSD)
\file      semaphore.h
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

#ifndef MY_SEMAPHORE_H_
#define MY_SEMAPHORE_H_

#include <iostream>
#include <boost/thread.hpp>

namespace micros_swarm{
    class Semaphore{
        public:
            Semaphore(int value): value_(value){}
            Semaphore(const Semaphore&) = delete;

            void P()
            {
                boost::unique_lock<boost::mutex> lock(mutex_);
                while(value_ == 0) {
                    condition_.wait(lock);
                }

                value_--;
            }

            void V()
            {
                boost::unique_lock<boost::mutex> lock(mutex_);
                value_++;

                condition_.notify_one();
            }
        private:
            int value_;
            boost::mutex mutex_;
            boost::condition_variable condition_;
    };
};

#endif