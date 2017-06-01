/**
Software License Agreement (BSD)
\file      opensplice_dds_comm.cpp
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

#include "opensplice_dds_comm/opensplice_dds_comm.h"

PLUGINLIB_EXPORT_CLASS(opensplice_dds_comm::OpenSpliceDDSComm, micros_swarm::CommInterface)

namespace opensplice_dds_comm{

    OpenSpliceDDSComm::OpenSpliceDDSComm()
    {
        packet_publisher_.reset(new opensplice_dds_comm::Publisher("micros_swarm_framework_topic"));
        packet_subscriber_.reset(new opensplice_dds_comm::Subscriber("micros_swarm_framework_topic"));
    }

    void OpenSpliceDDSComm::init(std::string name, const micros_swarm::PacketParser& parser)
    {
        name_=name;
        parser_=parser;
    }

    void OpenSpliceDDSComm::broadcast(const micros_swarm::CommPacket& packet)
    {
        opensplice_dds_comm::GSDFPacket dds_msg;
        dds_msg.packet_source=packet.packet_source;
        dds_msg.packet_version=packet.packet_version;
        dds_msg.packet_type=packet.packet_type;
        dds_msg.packet_data=packet.packet_data.data();
        dds_msg.package_check_sum=packet.package_check_sum;

        packet_publisher_->publish(dds_msg);
    }

    void OpenSpliceDDSComm::callback(const opensplice_dds_comm::GSDFPacket& dds_msg)
    {
        micros_swarm::CommPacket packet;
        packet.packet_source=dds_msg.packet_source;
        packet.packet_version=dds_msg.packet_version;
        packet.packet_type=dds_msg.packet_type;
        packet.packet_data=dds_msg.packet_data;
        packet.package_check_sum=dds_msg.package_check_sum;

        parser_.parse(packet);
    }
            
    void OpenSpliceDDSComm::receive()
    {
        boost::function<void(const opensplice_dds_comm::GSDFPacket&)> func=boost::bind(&OpenSpliceDDSComm::callback,this,_1);
        packet_subscriber_->subscribe(func);
    }
};
