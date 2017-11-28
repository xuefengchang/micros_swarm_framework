/**
Software License Agreement (BSD)
\file      listener_helper.h 
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

#ifndef LISTENER_HELPER_H_
#define LISTENER_HELPER_H_

#include <iostream>
#include <boost/function.hpp>
#include "micros_swarm/packet_type.h"
#include "micros_swarm/serialize.h"

namespace micros_swarm{
    
    class ListenerHelper{
        public:
            virtual void call(const std::vector<uint8_t>& value_vec)=0;
    };
    
    template<typename Type>
    class ListenerHelperT : public ListenerHelper{
        public:
            ListenerHelperT(const std::string& key, const boost::function<void(const Type&)>& callback)
            {
                key_ = key;
                callback_ = callback;
            }
    
            virtual void call(const std::vector<uint8_t>& value_vec)
            {
                callback_(convertType(value_vec));
            }
        
            Type convertType(const std::vector<uint8_t>& value_vec)
            {
                Type value = deserialize_ros<Type>(value_vec);
                return value;
            }
        
        private:
            boost::function<void(const Type&)> callback_;
            std::string key_;
    };
};
#endif
