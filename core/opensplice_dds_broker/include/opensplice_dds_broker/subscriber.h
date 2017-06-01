/**
Software License Agreement (BSD)
\file      subscriber.h
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

#ifndef SUBSCRIBER_H_
#define SUBSCRIBER_H_

#include <iostream>
#include <string.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include "ccpp_dds_dcps.h"
#include "check_status.h"
#include "ccpp_GSDFPacket.h"
#include "example_main.h"
#include "GSDFPacket_listener.h"

using namespace DDS;

namespace opensplice_dds_broker{
    class Subscriber
    {
        public:
            Subscriber(const std::string& topic_name);
            void subscribe(void (*callBack)(const GSDFPacket& packet));
            void subscribe(boost::function<void(const GSDFPacket&)> callBack);
            ~Subscriber();
        private:
            DomainId_t  domain;
            ReturnCode_t  status;
            const char *topic_name_;
            char  *GSDFPacketTypeName;
            
            //Generic DDS entities
            DomainParticipantFactory_var  dpf;
            DomainParticipant_var  participant;
            Topic_var  GSDFPacketTopic;
            Subscriber_var  subscriber_;
            DataReader_ptr  parentReader;

            GSDFPacketTypeSupport_var  GSDFPacketTS;
            GSDFPacketDataReader_var  GSDFPacketDR;

            //QosPolicy holders
            TopicQos  topic_qos;
            SubscriberQos  sub_qos;
            DataReaderQos  dr_qos;
    };
};

#endif

