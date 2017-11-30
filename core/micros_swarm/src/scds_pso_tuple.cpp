/**
Software License Agreement (BSD)
\file      scds_pso_tuple.cpp
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

#include "micros_swarm/scds_pso_tuple.h"

namespace micros_swarm{

    SCDSPSOTuple::SCDSPSOTuple()
    {
        rth_ = Singleton<RuntimeHandle>::getSingleton();
        mqm_ = Singleton<MsgQueueManager>::getSingleton();
    }

    SCDSPSOTuple::SCDSPSOTuple(const SCDSPSOTuple& t)
    {
        rth_ = Singleton<RuntimeHandle>::getSingleton();
        mqm_ = Singleton<MsgQueueManager>::getSingleton();
    }

    SCDSPSOTuple& SCDSPSOTuple::operator=(const SCDSPSOTuple& t)
    {
        if(this == &t) {
            return *this;
        }
        rth_ = Singleton<RuntimeHandle>::getSingleton();
        mqm_ = Singleton<MsgQueueManager>::getSingleton();
        return *this;
    }

    SCDSPSOTuple::~SCDSPSOTuple()
    {
        rth_.reset();
        mqm_.reset();
    }

    void SCDSPSOTuple::put(const std::string& key, const SCDSPSODataTuple& data)
    {
        rth_->insertOrUpdateSCDSPSOValue(key, data);

        gsdf_msgs::SCDSPSOPut scds_put;
        scds_put.key = key;
        scds_put.pos = data.pos;
        scds_put.val = data.val;
        scds_put.robot_id = data.robot_id;
        scds_put.gen = data.gen;
        scds_put.timestamp = data.timestamp;
        std::vector<uint8_t> scds_put_vec = serialize_ros(scds_put);

        gsdf_msgs::CommPacket p;
        p.header.source = rth_->getRobotID();
        p.header.type = SCDS_PSO_PUT;
        p.header.data_len = scds_put_vec.size();
        p.header.version = 1;
        p.header.checksum = 0;
        p.content.buf = scds_put_vec;
        std::vector<uint8_t> msg_data = serialize_ros(p);
        mqm_->getOutMsgQueue("scds_pso")->push(msg_data);
    }

    SCDSPSODataTuple SCDSPSOTuple::get(const std::string& key)
    {
        SCDSPSODataTuple data;
        if (!rth_->getSCDSPSOValue(key, data)) {
            std::cout<<"scds pso tuple, "<<key<<" is not exist."<<std::endl;
            exit(-1);
        }

        gsdf_msgs::SCDSPSOGet scds_get;
        scds_get.key = key;
        scds_get.pos = data.pos;
        scds_get.val = data.val;
        scds_get.robot_id = data.robot_id;
        scds_get.gen = data.gen;
        scds_get.timestamp = data.timestamp;
        std::vector<uint8_t> scds_get_vec = serialize_ros(scds_get);

        gsdf_msgs::CommPacket p;
        p.header.source = rth_->getRobotID();
        p.header.type = SCDS_PSO_GET;
        p.header.data_len = scds_get_vec.size();
        p.header.version = 1;
        p.header.checksum = 0;
        p.content.buf = scds_get_vec;
        std::vector<uint8_t> msg_data = serialize_ros(p);
        mqm_->getOutMsgQueue("scds_pso")->push(msg_data);

        return data;
    }
}