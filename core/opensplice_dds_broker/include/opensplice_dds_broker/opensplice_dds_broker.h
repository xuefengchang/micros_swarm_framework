/**
Software License Agreement (BSD)
\file      opensplice_dds_broker.h
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

#ifndef OPENSPLICE_DDS_BROKER_H_
#define OPENSPLICE_DDS_BROKER_H_

#include <iostream>
#include <pluginlib/class_list_macros.h>
#include <class_loader/multi_library_class_loader.h>

#include "GSDFPacket.h"
#include "check_status.h"
#include "publisher.h"
#include "subscriber.h"

#include "micros_swarm/comm_interface.h"

namespace opensplice_dds_broker{
    
    class OpenSpliceDDSBroker : public micros_swarm::CommInterface{
        public:
            OpenSpliceDDSBroker();
            void init(std::string name, const micros_swarm::PacketParser& parser);
            void broadcast(const std::vector<uint8_t>& msg_data);
            void receive();
        private:
            void callback(const opensplice_dds_broker::GSDFPacket& dds_msg);
            std::string name_;
            micros_swarm::PacketParser parser_;
            boost::shared_ptr<opensplice_dds_broker::Publisher> packet_publisher_;
            boost::shared_ptr<opensplice_dds_broker::Subscriber> packet_subscriber_;
    };
};
#endif
