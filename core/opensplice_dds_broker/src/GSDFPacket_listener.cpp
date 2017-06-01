/**
Software License Agreement (BSD)
\file      GSDFPacket_listener.cpp
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

#include "opensplice_dds_broker/GSDFPacket_listener.h"
#include "opensplice_dds_broker/check_status.h"
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/string.hpp> 
#include <boost/serialization/vector.hpp>
#include <sstream>
#include <fstream>

using namespace DDS;

namespace opensplice_dds_broker{

    void GSDFPacketListener::on_data_available(DDS::DataReader_ptr reader)
      THROW_ORB_EXCEPTIONS
    {
        DDS::ReturnCode_t status;
        GSDFPacketSeq packetSeq;
        SampleInfoSeq infoSeq;
        
        GSDFPacketDataReader_var GSDFPacketDR_ = GSDFPacketDataReader::_narrow(reader);

        status = GSDFPacketDR_->take(packetSeq, infoSeq, LENGTH_UNLIMITED,
        ANY_SAMPLE_STATE, ANY_VIEW_STATE, ANY_INSTANCE_STATE);
        checkStatus(status, "GSDFPacketDataReader::read");

        for (CORBA::ULong i = 0; i < packetSeq.length(); i++)
        {
            try{
                callBack_(packetSeq[i]);
            }catch(const std::bad_alloc&){
                return;
            }
        }
        status = GSDFPacketDR_->return_loan(packetSeq, infoSeq);
        checkStatus(status, "GSDFPacketDataReader::return_loan");
    };

    void GSDFPacketListener::on_requested_deadline_missed(DDS::DataReader_ptr
      reader, const DDS::RequestedDeadlineMissedStatus &status)THROW_ORB_EXCEPTIONS
    {
        printf("\n=== [GSDFPacketListener::on_requested_deadline_missed] : triggered\n");
        printf("\n=== [GSDFPacketListener::on_requested_deadline_missed] : stopping\n");
    };

    void GSDFPacketListener::on_requested_incompatible_qos(DDS::DataReader_ptr
      reader, const DDS::RequestedIncompatibleQosStatus &status)
      THROW_ORB_EXCEPTIONS
    {
        printf("\n=== [GSDFPacketListener::on_requested_incompatible_qos] : triggered\n");
    };

    void GSDFPacketListener::on_sample_rejected(DDS::DataReader_ptr reader, const
      DDS::SampleRejectedStatus &status)THROW_ORB_EXCEPTIONS
    {
        printf("\n=== [GSDFPacketListener::on_sample_rejected] : triggered\n");
    };

    void GSDFPacketListener::on_liveliness_changed(DDS::DataReader_ptr reader,
      const DDS::LivelinessChangedStatus &status)THROW_ORB_EXCEPTIONS
    {
        printf("\n=== [GSDFPacketListener::on_liveliness_changed] : triggered\n");
    };

    void GSDFPacketListener::on_subscription_matched(DDS::DataReader_ptr reader,
      const DDS::SubscriptionMatchedStatus &status)THROW_ORB_EXCEPTIONS
    {
        printf("\n=== [GSDFPacketListener::on_subscription_matched] : triggered\n");
    };

    void GSDFPacketListener::on_sample_lost(DDS::DataReader_ptr reader, const DDS
      ::SampleLostStatus &status)THROW_ORB_EXCEPTIONS
    {
        printf("\n=== [GSDFPacketListener::on_sample_lost] : triggered\n");
    };
};
