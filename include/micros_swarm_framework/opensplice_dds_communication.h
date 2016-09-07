/**
Software License Agreement (BSD)
\file      opensplice_dds_communication.h
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

#ifndef OPENSPLICE_DDS_COMMUNICATION_H_
#define OPENSPLICE_DDS_COMMUNICATION_H_

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

#include "micros_swarm_framework/communication_interface.h"

#include "opensplice_dds/MSFPPacket.h"
#include "opensplice_dds/check_status.h"
#include "opensplice_dds/publisher.h"
#include "opensplice_dds/subscriber.h"

namespace micros_swarm_framework{
    
    class OpenSpliceDDSCommunication : public CommunicationInterface{
        public:
            OpenSpliceDDSCommunication()
            {
                name_="OPENSPLICE_DDS";
                packet_publisher_.reset(new micros_swarm_framework::Publisher("micros_swarm_framework_topic"));
                packet_subscriber_.reset(new micros_swarm_framework::Subscriber("micros_swarm_framework_topic"));
            }
            
            void broadcast(const MSFPPacket& msfp_packet)
            {
                packet_publisher_->publish(msfp_packet); 
            }
            
            void receive(boost::function<void(const MSFPPacket&)> parser)
            {
                parser_=parser;
                packet_subscriber_->subscribe(parser_);
            }
            
        private:
            boost::shared_ptr<micros_swarm_framework::Publisher> packet_publisher_;
            boost::shared_ptr<micros_swarm_framework::Subscriber> packet_subscriber_;
    };
};
#endif
