/**
Software License Agreement (BSD)
\file      opensplice_dds_comm.h
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

#ifndef OPENSPLICE_DDS_COMM_H_
#define OPENSPLICE_DDS_COMM_H_

#include <iostream>

#include "micros_swarm_framework/comm_interface.h"
#include "MSFPPacket.h"
#include "check_status.h"
#include "publisher.h"
#include "subscriber.h"

namespace micros_swarm_framework{
    
    class OpenSpliceDDSComm : public CommInterface{
        public:
            OpenSpliceDDSComm()
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
