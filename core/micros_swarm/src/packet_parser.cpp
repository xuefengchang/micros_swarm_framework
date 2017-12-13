/**
Software License Agreement (BSD)
\file      packet_parser.cpp
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

#include "micros_swarm/packet_parser.h"

namespace micros_swarm{

    PacketParser::PacketParser()
    {
        rth_ = Singleton<RuntimeHandle>::getSingleton();
        cni_.reset(new CheckNeighbor(rth_->getNeighborDistance()));
        mqm_ = Singleton<MsgQueueManager>::getSingleton();
    }

    PacketParser::~PacketParser()
    {
        rth_.reset();
        cni_.reset();
        mqm_.reset();
    }

    void PacketParser::parse(const std::vector<uint8_t>& data)
    {
        gsdf_msgs::CommPacket packet = deserialize_ros<gsdf_msgs::CommPacket>(data);
        int packet_source = packet.header.source;
        int shm_rid = rth_->getRobotID();
        
        //ignore the packet of the local robot
        if(packet_source == shm_rid) {
            return;
        }

        try {
            const int packet_type = packet.header.type;
            std::vector<uint8_t> packet_data = packet.content.buf;

            switch(packet_type) {
                case SINGLE_ROBOT_BROADCAST_BASE: {
                    gsdf_msgs::RobotBase rb = deserialize_ros<gsdf_msgs::RobotBase>(packet_data);
                
                    //if(rb.valid != 1) {  //ignore the default Base value.
                    //    return;
                    //}
                
                    const Base& self = rth_->getRobotBase();
                    Base neighbor(rb.px, rb.py, rb.pz, rb.vx, rb.vy, rb.vz, rb.valid);
                
                    if(cni_->isNeighbor(self, neighbor)) {
                        rth_->insertOrUpdateNeighbor(rb.id, 0, 0, 0, rb.px, rb.py, rb.pz, rb.vx, rb.vy, rb.vz);
                    }
                    else {
                        rth_->deleteNeighbor(rb.id);
                    }
                
                    break;
                }
                case SINGLE_ROBOT_JOIN_SWARM: {
                    if(!rth_->inNeighbors(packet_source)){
                        return;
                    }
                    gsdf_msgs::JoinSwarm srjs = deserialize_ros<gsdf_msgs::JoinSwarm>(packet_data);
                    rth_->joinNeighborSwarm(srjs.robot_id, srjs.swarm_id);
                
                    break;
                }
                case SINGLE_ROBOT_LEAVE_SWARM: {
                    if(!rth_->inNeighbors(packet_source)){
                        return;
                    }
                    gsdf_msgs::LeaveSwarm srls = deserialize_ros<gsdf_msgs::LeaveSwarm>(packet_data);
                    rth_->leaveNeighborSwarm(srls.robot_id, srls.swarm_id);
                
                    break;
                }
                case SINGLE_ROBOT_SWARM_LIST: {
                    if(!rth_->inNeighbors(packet_source)){
                        return;
                    }
                    gsdf_msgs::SwarmList srsl = deserialize_ros<gsdf_msgs::SwarmList>(packet_data);
                    rth_->insertOrRefreshNeighborSwarm(srsl.robot_id, srsl.swarm_list);
                
                    break;
                }
                case VIRTUAL_STIGMERGY_QUERY: {
                    if(!rth_->inNeighbors(packet_source)) {
                        return;
                    }
                    gsdf_msgs::VirtualStigmergyQuery vsq = deserialize_ros<gsdf_msgs::VirtualStigmergyQuery>(packet_data);
                    VirtualStigmergyTuple local;
                    bool exist = rth_->getVirtualStigmergyTuple(vsq.vstig_id, vsq.key, local);
                
                    //local tuple is not exist or the local timestamp is smaller
                    if(!exist||(local.lamport_clock<vsq.lamport_clock)) {
                        rth_->createVirtualStigmergy(vsq.vstig_id);
                        rth_->insertOrUpdateVirtualStigmergy(vsq.vstig_id, vsq.key, vsq.value, vsq.lamport_clock, time(NULL), 0, vsq.robot_id);

                        if(!rth_->checkNeighborsOverlap(packet_source)) {
                            gsdf_msgs::VirtualStigmergyPut vsp_new;
                            vsp_new.vstig_id = vsq.vstig_id;
                            vsp_new.key = vsq.key;
                            vsp_new.value = vsq.value;
                            vsp_new.lamport_clock = vsq.lamport_clock;
                            vsp_new.robot_id = vsq.robot_id;
                            std::vector<uint8_t> vsp_new_vec = serialize_ros(vsp_new);

                            gsdf_msgs::CommPacket p;
                            p.header.source = shm_rid;
                            p.header.type = VIRTUAL_STIGMERGY_PUT;
                            p.header.data_len = vsp_new_vec.size();
                            p.header.version = 1;
                            p.header.checksum = 0;
                            p.content.buf = vsp_new_vec;
                            std::vector<uint8_t> msg_data = serialize_ros(p);
                            mqm_->getOutMsgQueue("vstig")->push(msg_data);
                        }
                    }
                    else if(local.lamport_clock > vsq.lamport_clock) {  //local timestamp is larger
                        gsdf_msgs::VirtualStigmergyPut vsp;
                        vsp.vstig_id = vsq.vstig_id;
                        vsp.key = vsq.key;
                        vsp.value = local.vstig_value;
                        vsp.lamport_clock = local.lamport_clock;
                        vsp.robot_id = local.robot_id;
                        std::vector<uint8_t> vsp_vec = serialize_ros(vsp);
                    
                        gsdf_msgs::CommPacket p;
                        p.header.source = shm_rid;
                        p.header.type = VIRTUAL_STIGMERGY_PUT;
                        p.header.data_len = vsp_vec.size();
                        p.header.version = 1;
                        p.header.checksum = 0;
                        p.content.buf = vsp_vec;
                        std::vector<uint8_t> msg_data = serialize_ros(p);
                        mqm_->getOutMsgQueue("vstig")->push(msg_data);
                    }
                    else if((local.lamport_clock == vsq.lamport_clock) && (local.robot_id != vsq.robot_id)) {
                        //std::cout<<"query conflict"<<std::endl;
                    }
                    else {
                        //do nothing
                    }
                
                    break;
                }
                case VIRTUAL_STIGMERGY_PUT: {
                    if(!rth_->inNeighbors(packet_source)){
                        return;
                    }
                    gsdf_msgs::VirtualStigmergyPut vsp = deserialize_ros<gsdf_msgs::VirtualStigmergyPut>(packet_data);
                
                    VirtualStigmergyTuple local;
                    bool exist = rth_->getVirtualStigmergyTuple(vsp.vstig_id, vsp.key, local);
                
                    //local tuple is not exist or local timestamp is smaller
                    if(!exist||(local.lamport_clock < vsp.lamport_clock)) {
                        rth_->createVirtualStigmergy(vsp.vstig_id);
                        rth_->insertOrUpdateVirtualStigmergy(vsp.vstig_id, vsp.key, vsp.value, vsp.lamport_clock, time(NULL), 0, vsp.robot_id);

                        if(!rth_->checkNeighborsOverlap(packet_source)) {
                            gsdf_msgs::VirtualStigmergyPut vsp_new;
                            vsp_new.vstig_id = vsp.vstig_id;
                            vsp_new.key = vsp.key;
                            vsp_new.value = vsp.value;
                            vsp_new.lamport_clock = vsp.lamport_clock;
                            vsp_new.robot_id = vsp.robot_id;
                            std::vector<uint8_t> vsp_new_vec = serialize_ros(vsp_new);

                            gsdf_msgs::CommPacket p;
                            p.header.source = shm_rid;
                            p.header.type = VIRTUAL_STIGMERGY_PUT;
                            p.header.data_len = vsp_new_vec.size();
                            p.header.version = 1;
                            p.header.checksum = 0;
                            p.content.buf = vsp_new_vec;
                            std::vector<uint8_t> msg_data = serialize_ros(p);
                            mqm_->getOutMsgQueue("vstig")->push(msg_data);
                        }
                    }
                    else if((local.lamport_clock==vsp.lamport_clock)&&(local.robot_id!=vsp.robot_id)) {
                        //std::cout<<"put conflict"<<std::endl;
                    }
                    else {
                        //do nothing
                    }
                        
                    break;
                }
                case VIRTUAL_STIGMERGY_PUTS: {
                    if(!rth_->inNeighbors(packet_source)){
                        return;
                    }
                    gsdf_msgs::VirtualStigmergyPuts vsps = deserialize_ros<gsdf_msgs::VirtualStigmergyPuts>(packet_data);

                    bool process_msg = false;
                    int neighbor_size = rth_->getNeighborSize();
                    float prob = vsps.prob;
                    float rand_prob = (float)rand()/RAND_MAX;

                    if(neighbor_size < 3) {
                        process_msg = true;
                    }
                    else {
                        if(rand_prob < prob) {
                            process_msg = true;
                        }
                        else {
                            process_msg = false;
                        }
                    }

                    //std::cout<<rth_->getRobotID()<<": "<<rand_prob<<", "<<prob<<", "<<process_msg<<std::endl;

                    if(!process_msg) {
                        return;
                    }

                    VirtualStigmergyTuple local;
                    bool exist = rth_->getVirtualStigmergyTuple(vsps.vstig_id, vsps.key, local);

                    //local tuple is not exist or local timestamp is smaller
                    if(!exist||(local.lamport_clock < vsps.lamport_clock)) {
                        rth_->createVirtualStigmergy(vsps.vstig_id);
                        rth_->insertOrUpdateVirtualStigmergy(vsps.vstig_id, vsps.key, vsps.value, vsps.lamport_clock, time(NULL), 0, vsps.robot_id);

                        if(!rth_->checkNeighborsOverlap(packet_source)) {
                            gsdf_msgs::VirtualStigmergyPuts vsps_new;
                            vsps_new.vstig_id = vsps.vstig_id;
                            vsps_new.key = vsps.key;
                            vsps_new.value = vsps.value;
                            vsps_new.lamport_clock = vsps.lamport_clock;
                            vsps_new.robot_id = vsps.robot_id;
                            int neighbor_size = rth_->getNeighborSize();
                            if(neighbor_size < 3) {
                                vsps_new.prob = 1;
                            }
                            else {
                                vsps_new.prob = 2.0/neighbor_size;
                            }
                            std::vector<uint8_t> vsps_new_vec = serialize_ros(vsps_new);

                            gsdf_msgs::CommPacket p;
                            p.header.source = shm_rid;
                            p.header.type = VIRTUAL_STIGMERGY_PUTS;
                            p.header.data_len = vsps_new_vec.size();
                            p.header.version = 1;
                            p.header.checksum = 0;
                            p.content.buf = vsps_new_vec;
                            std::vector<uint8_t> msg_data = serialize_ros(p);
                            mqm_->getOutMsgQueue("vstig")->push(msg_data);
                        }
                    }
                    else if((local.lamport_clock==vsps.lamport_clock)&&(local.robot_id!=vsps.robot_id)) {
                        //std::cout<<"put conflict"<<std::endl;
                    }
                    else {
                        //do nothing
                    }

                    break;
                }
                case BLACKBOARD_PUT: {
                    if(!rth_->inNeighbors(packet_source)){
                        return;
                    }
                    gsdf_msgs::BlackBoardPut bbp = deserialize_ros<gsdf_msgs::BlackBoardPut>(packet_data);
                    int robot_id = rth_->getRobotID();
                    std::string bb_key = bbp.key;
                    if(bbp.on_robot_id == robot_id) {
                        rth_->createBlackBoard(bbp.bb_id);
                        BlackBoardTuple local;
                        rth_->getBlackBoardTuple(bbp.bb_id, bbp.key, local);

                        //local tuple is not exist or the local timestamp is smaller
                        if(!rth_->isBlackBoardTupleExist(bbp.bb_id, bbp.key) || local.timestamp < bbp.timestamp) {
                            rth_->insertOrUpdateBlackBoard(bbp.bb_id, bbp.key, bbp.value, bbp.timestamp, bbp.robot_id);
                        }
                    }
                    else{

                    }

                    break;
                }
                case BLACKBOARD_QUERY: {
                    if(!rth_->inNeighbors(packet_source)){
                        return;
                    }
                    gsdf_msgs::BlackBoardQuery bbq = deserialize_ros<gsdf_msgs::BlackBoardQuery>(packet_data);
                    int robot_id = rth_->getRobotID();
                    std::string bb_key = bbq.key;

                    if(bbq.on_robot_id == robot_id) {
                        BlackBoardTuple local;
                        rth_->getBlackBoardTuple(bbq.bb_id, bbq.key, local);
                        gsdf_msgs::BlackBoardAck bba;
                        bba.bb_id = bbq.bb_id;
                        bba.on_robot_id = bbq.on_robot_id;
                        bba.key = bbq.key;
                        bba.value = local.bb_value;
                        bba.timestamp = local.timestamp;
                        bba.robot_id = bbq.robot_id;
                        std::vector<uint8_t> bbqa_vec = serialize_ros(bba);

                        gsdf_msgs::CommPacket p;
                        p.header.source = shm_rid;
                        p.header.type = BLACKBOARD_QUERY_ACK;
                        p.header.data_len = bbqa_vec.size();
                        p.header.version = 1;
                        p.header.checksum = 0;
                        p.content.buf = bbqa_vec;
                        std::vector<uint8_t> msg_data = serialize_ros(p);
                        mqm_->getOutMsgQueue("bb")->push(msg_data);
                    }
                    else{

                    }

                    break;
                }
                case BLACKBOARD_QUERY_ACK: {
                    if(!rth_->inNeighbors(packet_source)){
                        return;
                    }
                    gsdf_msgs::BlackBoardAck bba = deserialize_ros<gsdf_msgs::BlackBoardAck>(packet_data);
                    int robot_id = rth_->getRobotID();
                    std::string bb_key = bba.key;

                    if(bba.on_robot_id == robot_id) {

                    }
                    else {
                        rth_->createBlackBoard(bba.bb_id);
                        rth_->insertOrUpdateBlackBoard(bba.bb_id, bba.key, bba.value, bba.timestamp, bba.robot_id);
                    }

                    break;
                }
                case SCDS_PSO_PUT: {
                    if(!rth_->inNeighbors(packet_source)){
                        return;
                    }
                    gsdf_msgs::SCDSPSOPut scds_put = deserialize_ros<gsdf_msgs::SCDSPSOPut>(packet_data);
                    SCDSPSODataTuple local;
                    bool exist = rth_->getSCDSPSOValue(scds_put.key, local);

                    //local tuple is not exist or local value is smaller
                    if ((!exist) || (local.val < scds_put.val)) {
                        SCDSPSODataTuple new_data;
                        new_data.pos = scds_put.pos;
                        new_data.val = scds_put.val;
                        new_data.robot_id = scds_put.robot_id;
                        new_data.gen = scds_put.gen;
                        new_data.timestamp = scds_put.timestamp;
                        rth_->insertOrUpdateSCDSPSOValue(scds_put.key, new_data);

                        if(!rth_->checkNeighborsOverlap(packet_source)) {
                            gsdf_msgs::CommPacket p;
                            p.header.source = shm_rid;
                            p.header.type = SCDS_PSO_PUT;
                            p.header.data_len = packet_data.size();
                            p.header.version = 1;
                            p.header.checksum = 0;
                            p.content.buf = packet_data;
                            std::vector <uint8_t> msg_data = serialize_ros(p);
                            mqm_->getOutMsgQueue("scds_pso")->push(msg_data);
                        }
                    }

                    break;
                }
                case SCDS_PSO_GET: {
                    if(!rth_->inNeighbors(packet_source)){
                        return;
                    }
                    gsdf_msgs::SCDSPSOGet scds_get = deserialize_ros<gsdf_msgs::SCDSPSOGet>(packet_data);
                    SCDSPSODataTuple local;
                    bool exist = rth_->getSCDSPSOValue(scds_get.key, local);

                    //local tuple is not exist or local value is smaller
                    if ((!exist) || (local.val < scds_get.val)) {
                        SCDSPSODataTuple new_data;
                        new_data.pos = scds_get.pos;
                        new_data.val = scds_get.val;
                        new_data.robot_id = scds_get.robot_id;
                        new_data.gen = scds_get.gen;
                        new_data.timestamp = scds_get.timestamp;
                        rth_->insertOrUpdateSCDSPSOValue(scds_get.key, new_data);

                        if(!rth_->checkNeighborsOverlap(packet_source)) {
                            gsdf_msgs::CommPacket p;
                            p.header.source = shm_rid;
                            p.header.type = SCDS_PSO_PUT;
                            p.header.data_len = packet_data.size();
                            p.header.version = 1;
                            p.header.checksum = 0;
                            p.content.buf = packet_data;
                            std::vector <uint8_t> msg_data = serialize_ros(p);
                            mqm_->getOutMsgQueue("scds_pso")->push(msg_data);
                        }
                    }
                    else if(local.val > scds_get.val) {
                        gsdf_msgs::SCDSPSOPut scds_put;
                        scds_put.key = scds_get.key;
                        scds_put.pos = local.pos;
                        scds_put.val = local.val;
                        scds_put.robot_id = local.robot_id;
                        scds_put.gen = local.gen;
                        scds_put.timestamp = local.timestamp;
                        std::vector<uint8_t> scds_put_vec = serialize_ros(scds_put);;

                        gsdf_msgs::CommPacket p;
                        p.header.source = shm_rid;
                        p.header.type = SCDS_PSO_PUT;
                        p.header.data_len = scds_put_vec.size();
                        p.header.version = 1;
                        p.header.checksum = 0;
                        p.content.buf = scds_put_vec;
                        std::vector<uint8_t> msg_data = serialize_ros(p);
                        mqm_->getOutMsgQueue("scds_pso")->push(msg_data);
                    }
                    else {

                    }

                    break;
                }
                case NEIGHBOR_BROADCAST_KEY_VALUE: {
                    if(!rth_->inNeighbors(packet_source)){
                        return;
                    }
                    gsdf_msgs::NeighborBroadcastKeyValue nbkv = deserialize_ros<gsdf_msgs::NeighborBroadcastKeyValue>(packet_data);
                    boost::shared_ptr<ListenerHelper> helper = rth_->getListenerHelper(nbkv.key);
                    if(helper == NULL) {
                        return;
                    }
                    helper->call(nbkv.value);
               
                    break;
                }
                case BARRIER_SYN: {
                    gsdf_msgs::BarrierSyn bs = deserialize_ros<gsdf_msgs::BarrierSyn>(packet_data);
                    if(bs.s != "SYN") {
                        return;
                    }

                    gsdf_msgs::BarrierAck ba;
                    ba.robot_id = packet.header.source;
                    std::vector<uint8_t> ba_vec = serialize_ros(ba);
                    
                    gsdf_msgs::CommPacket p;
                    p.header.source = shm_rid;
                    p.header.type = BARRIER_ACK;
                    p.header.data_len = ba_vec.size();
                    p.header.version = 1;
                    p.header.checksum = 0;
                    p.content.buf = ba_vec;
                    std::vector<uint8_t> msg_data = serialize_ros(p);
                    mqm_->getOutMsgQueue("barrier")->push(msg_data);
                    break;
                }
                case BARRIER_ACK: {
                    gsdf_msgs::BarrierAck ba = deserialize_ros<gsdf_msgs::BarrierAck>(packet_data);
                
                    if(shm_rid == ba.robot_id) {
                        rth_->insertBarrier(packet.header.source);
                    }
                    
                    break;
                }
            
                default: {
                    std::cout<<"UNDEFINED PACKET TYPE!"<<std::endl;
                }
            }
        }
        catch(char *err_str) {
            std::cout<<err_str<<std::endl;
            return;
        }
    }

    void PacketParser::PacketParser::parse(const std::vector<char>& data)
    {
        std::vector<uint8_t> seq;
        for(int i = 0; i < data.size(); i++) {
            uint8_t c = (uint8_t)(data[i]);
            seq.push_back(c);
        }
        parse(seq);
    }

    void PacketParser::parse(uint8_t* data, int len)
    {
        std::vector<uint8_t> seq;
        for(int i = 0; i < len; i++) {
            seq.push_back(*(data+i));
        }
        parse(seq);
    }

    void PacketParser::parse(char* data, int len)
    {
        uint8_t *p = (uint8_t*)data;
        parse(p, len);
    }
};
