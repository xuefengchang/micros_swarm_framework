/**
Software License Agreement (BSD)
\file      opensplice_dds_broker.cpp
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

#include <iostream>
#include "opensplice_dds_broker/opensplice_dds_broker.h"

PLUGINLIB_EXPORT_CLASS(opensplice_dds_broker::OpenSpliceDDSBroker, micros_swarm::CommInterface)

namespace opensplice_dds_broker{

    OpenSpliceDDSBroker::OpenSpliceDDSBroker()
    {
        packet_publisher_.reset(new opensplice_dds_broker::Publisher("micros_swarm_framework_topic"));
        packet_subscriber_.reset(new opensplice_dds_broker::Subscriber("micros_swarm_framework_topic"));
    }

    void OpenSpliceDDSBroker::init(std::string name, const micros_swarm::PacketParser& parser)
    {
        name_ = name;
        parser_ = parser;
    }

    void OpenSpliceDDSBroker::broadcast(const std::vector<uint8_t>& msg_data)
    {
        opensplice_dds_broker::GSDFPacket dds_msg;
        unsigned int bufsize = msg_data.size();
        if(!msg_data.empty()) {
            dds_msg.data.replace(bufsize, bufsize, (char*)(&msg_data[0]), false);
            packet_publisher_->publish(dds_msg);
        }
    }

    void OpenSpliceDDSBroker::callback(const opensplice_dds_broker::GSDFPacket& dds_msg)
    {
        int msgLen = dds_msg.data.length();
        if (msgLen == 0) {
            std::cout<<"opensplice dds recv error."<<std::endl;
        }
        else {
            uint8_t* buf = (uint8_t*)malloc(sizeof(uint8_t)*msgLen);
            memcpy(buf, dds_msg.data.get_buffer(), msgLen);
            parser_.parse(buf, msgLen);
        }
    }
            
    void OpenSpliceDDSBroker::receive()
    {
        boost::function<void(const opensplice_dds_broker::GSDFPacket&)> func = boost::bind(&OpenSpliceDDSBroker::callback, this, _1);
        packet_subscriber_->subscribe(func);
    }
};
