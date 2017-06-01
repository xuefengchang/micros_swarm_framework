/**
Software License Agreement (BSD)
\file      publisher.h
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

#ifndef PUBLISHER_H_
#define PUBLISHER_H_

#include <string>
#include <iostream>
#include <vector>
#include "ccpp_dds_dcps.h"
#include "check_status.h"
#include "ccpp_GSDFPacket.h"
#include "example_main.h"

using namespace DDS;

namespace opensplice_dds_broker{
    class Publisher
    {
        public:
            Publisher(const std::string& topic_name);
            void publish(const GSDFPacket& packet);
            ~Publisher();
        private:
            DomainId_t  domain;
            const char *topic_name_;
            char  *GSDFPacketTypeName;
            
            DomainParticipantFactory_var  dpf;
            DomainParticipant_var  participant;
            Topic_var  GSDFPacketTopic;
            Publisher_var  publisher_;
            DataWriter_ptr  parentWriter;
            TopicQos  topic_qos;
            PublisherQos  pub_qos;
            DataWriterQos  dw_qos;

            ReturnCode_t  status;

            GSDFPacketTypeSupport_var  GSDFPacketTS;
            GSDFPacketDataWriter_var  GSDFPacketDW;
    };
};

#endif
