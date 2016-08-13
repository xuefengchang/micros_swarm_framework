/**
Software License Agreement (BSD)
\file      neighbor_communication.h 
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

#ifndef NEIGHBOR_COMMUNICATION_H_
#define NEIGHBOR_COMMUNICATION_H_

#include <iostream>
#include <string>
#include <time.h>
#include <stdlib.h>
#include <vector>
#include <stack>
#include <map>
#include <set>
#include <queue>
#include <algorithm>

#include "ros/ros.h"

#include "micros_swarm_framework/runtime_platform.h"
#include "micros_swarm_framework/communication_interface.h"
#ifdef ROS
#include "micros_swarm_framework/ros_communication.h"
#endif
#ifdef OPENSPLICE_DDS
#include "micros_swarm_framework/opensplice_dds_communication.h"
#endif

namespace micros_swarm_framework{
    
    template<class Type>
    class Broadcaster{
        private:
            boost::shared_ptr<RuntimePlatform> rtp_;
            boost::shared_ptr<CommunicationInterface> communicator_;
            std::string key_;
        public:
            Broadcaster(std::string key)
            {
                key_=key;
                rtp_=Singleton<RuntimePlatform>::getSingleton();
                #ifdef ROS
                communicator_=Singleton<ROSCommunication>::getSingleton();
                #endif
                #ifdef OPENSPLICE_DDS
                communicator_=Singleton<OpenSpliceDDSCommunication>::getSingleton();
                #endif
            }
            
            void broadcast(Type value)
            {
                std::ostringstream archiveStream;
                boost::archive::text_oarchive archive(archiveStream);
                archive<<value;
                std::string value_str=archiveStream.str();
                
                NeighborBroadcastKeyValue nbkv(key_, value_str);
                
                std::ostringstream archiveStream2;
                boost::archive::text_oarchive archive2(archiveStream2);
                archive2<<nbkv;
                std::string nbkv_str=archiveStream2.str();  
                
                micros_swarm_framework::MSFPPacket p;
                p.packet_source=rtp_->getRobotID();
                p.packet_version=1;
                p.packet_type=NEIGHBOR_BROADCAST_KEY_VALUE;
                #ifdef ROS
                p.packet_data=nbkv_str;
                #endif
                #ifdef OPENSPLICE_DDS
                p.packet_data=nbkv_str.data();
                #endif
                p.package_check_sum=0;
                
                communicator_->broadcast(p);
            }
    };
    
    class Listener{
        private:
            boost::shared_ptr<RuntimePlatform> rtp_;
            boost::shared_ptr<CommunicationInterface> communicator_;
            std::string key_;
        public:
            Listener(std::string key)
            {
                key_=key;
                rtp_=Singleton<RuntimePlatform>::getSingleton();
                #ifdef ROS
                communicator_=Singleton<ROSCommunication>::getSingleton();
                #endif
                #ifdef OPENSPLICE_DDS
                communicator_=Singleton<OpenSpliceDDSCommunication>::getSingleton();
                #endif
            }
            
            void listen(boost::function<void(const std::string&)> f)
            {
                rtp_->insertOrUpdateCallbackFunctions(key_, f);
            }
            
            //void listen(void (*f)(std::string value))
            //{
                
            //}
            
            void ignore()
            {
                rtp_->deleteCallbackFunctions(key_);
            }
    };
    
    template<class Type>
    Type convertToType(const std::string& value_str)  //TODO, 
    {
        std::istringstream archiveStream(value_str);
        boost::archive::text_iarchive archive(archiveStream); 
        Type value;
        archive>>value;
                
        return value;
    }
};
#endif
