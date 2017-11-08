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

#include <iostream>
#include <time.h>
#include <vector>
#include <map>

#include "micros_swarm/message.h"
#include "micros_swarm/singleton.h"
#include "micros_swarm/check_neighbor.h"

#include "micros_swarm/packet_parser.h"

namespace micros_swarm{

    PacketParser::PacketParser()
    {
        rth_=Singleton<RuntimeHandle>::getSingleton();
        float neighbor_distance=rth_->getNeighborDistance();
        cni_.reset(new CheckNeighbor(neighbor_distance));
    }

    void PacketParser::parse(const micros_swarm::CommPacket& packet)
    {
        int shm_rid=rth_->getRobotID();
        int packet_source=packet.packet_source;
        
        //ignore the packet of the local robot
        if(packet_source==shm_rid)
            return;
        try
        {
            const int packet_type=packet.packet_type;
            std::string packet_data=packet.packet_data;
        
            std::istringstream archiveStream(packet_data);
            boost::archive::text_iarchive archive(archiveStream);

            switch(packet_type)
            {
                case SINGLE_ROBOT_BROADCAST_BASE:{
                    micros_swarm::SingleRobotBroadcastBase srbb;
                    archive>>srbb;
                
                    if(srbb.valid!=1)  //ignore the default Base value.
                        return;
                
                    const Base& self=rth_->getRobotBase();
                    Base neighbor(srbb.robot_x, srbb.robot_y, srbb.robot_z, srbb.robot_vx, srbb.robot_vy, srbb.robot_vz);
                
                    //float neighbor_distance=rth_->getNeighborDistance();
                    //boost::shared_ptr<CheckNeighborInterface> cni(new CheckNeighbor(neighbor_distance));
                
                    if(cni_->isNeighbor(self, neighbor))
                        rth_->insertOrUpdateNeighbor(srbb.robot_id, 0, 0, 0, srbb.robot_x, srbb.robot_y, srbb.robot_z, srbb.robot_vx, srbb.robot_vy, srbb.robot_vz);
                    else
                        rth_->deleteNeighbor(srbb.robot_id);
                
                    break;
                }
                case SINGLE_ROBOT_JOIN_SWARM:{
                    SingleRobotJoinSwarm srjs;
                    archive>>srjs;

                    rth_->joinNeighborSwarm(srjs.robot_id, srjs.swarm_id);
                
                    break;
                }
                case SINGLE_ROBOT_LEAVE_SWARM:
                {
                    SingleRobotLeaveSwarm srls;
                    archive>>srls;

                    rth_->leaveNeighborSwarm(srls.robot_id, srls.swarm_id);
                
                    break;
                }
                case SINGLE_ROBOT_SWARM_LIST:
                {
                    SingleRobotSwarmList srsl;
                    archive>>srsl;
                
                    rth_->insertOrRefreshNeighborSwarm(srsl.robot_id, srsl.swarm_list);
                
                    break;
                }
                case VIRTUAL_STIGMERGY_QUERY:
                {
                    VirtualStigmergyQuery vsq;
                    archive>>vsq;
                
                    VirtualStigmergyTuple local;
                    bool exist=rth_->getVirtualStigmergyTuple(vsq.virtual_stigmergy_id, vsq.virtual_stigmergy_key, local);
                
                    //local tuple is not exist or the local timestamp is smaller
                    if(!exist||(local.lamport_clock<vsq.lamport_clock))
                    {
                        rth_->createVirtualStigmergy(vsq.virtual_stigmergy_id);
                        rth_->insertOrUpdateVirtualStigmergy(vsq.virtual_stigmergy_id, vsq.virtual_stigmergy_key, vsq.virtual_stigmergy_value,
                                                                 vsq.lamport_clock, time(NULL), 0, vsq.robot_id);

                        if(!rth_->checkNeighborsOverlap(packet_source)) {
                            VirtualStigmergyPut vsp_new(vsq.virtual_stigmergy_id, vsq.virtual_stigmergy_key,
                                                        vsq.virtual_stigmergy_value,
                                                        vsq.lamport_clock, vsq.robot_id);

                            std::ostringstream archiveStream2;
                            boost::archive::text_oarchive archive2(archiveStream2);
                            archive2 << vsp_new;
                            std::string vsp_new_string = archiveStream2.str();

                            micros_swarm::CommPacket p;
                            p.packet_source = shm_rid;
                            p.packet_version = 1;
                            p.packet_type = VIRTUAL_STIGMERGY_PUT;
                            p.packet_data = vsp_new_string;
                            p.package_check_sum = 0;

                            rth_->getOutMsgQueue()->pushVstigMsgQueue(p);
                        }
                    }
                    else if(local.lamport_clock>vsq.lamport_clock)  //local timestamp is larger
                    {
                        VirtualStigmergyPut vsp(vsq.virtual_stigmergy_id, vsq.virtual_stigmergy_key, local.vstig_value,
                                                local.lamport_clock, local.robot_id);
                        std::ostringstream archiveStream2;
                        boost::archive::text_oarchive archive2(archiveStream2);
                        archive2<<vsp;
                        std::string vsp_string=archiveStream2.str();
                    
                        micros_swarm::CommPacket p;
                        p.packet_source=shm_rid;
                        p.packet_version=1;
                        p.packet_type=VIRTUAL_STIGMERGY_PUT;
                        p.packet_data=vsp_string;
                        p.package_check_sum=0;
                    
                        rth_->getOutMsgQueue()->pushVstigMsgQueue(p);
                    }
                    else if((local.lamport_clock==vsq.lamport_clock)&&(local.robot_id!=vsq.robot_id))
                    {
                        //std::cout<<"query conflict"<<std::endl;
                    }
                    else
                    {
                        //do nothing
                    }
                
                    break;
                }
                case VIRTUAL_STIGMERGY_PUT:
                {
                    VirtualStigmergyPut vsp;
                    archive>>vsp;
                
                    VirtualStigmergyTuple local;
                    bool exist = rth_->getVirtualStigmergyTuple(vsp.virtual_stigmergy_id, vsp.virtual_stigmergy_key, local);
                
                    //local tuple is not exist or local timestamp is smaller
                    if(!exist||(local.lamport_clock<vsp.lamport_clock))
                    {
                        rth_->createVirtualStigmergy(vsp.virtual_stigmergy_id);
                        rth_->insertOrUpdateVirtualStigmergy(vsp.virtual_stigmergy_id, vsp.virtual_stigmergy_key, vsp.virtual_stigmergy_value,
                                                             vsp.lamport_clock, time(NULL), 0, vsp.robot_id);

                        if(!rth_->checkNeighborsOverlap(packet_source)) {
                            VirtualStigmergyPut vsp_new(vsp.virtual_stigmergy_id, vsp.virtual_stigmergy_key,
                                                        vsp.virtual_stigmergy_value,
                                                        vsp.lamport_clock, vsp.robot_id);
                            std::ostringstream archiveStream2;
                            boost::archive::text_oarchive archive2(archiveStream2);
                            archive2 << vsp_new;
                            std::string vsp_new_string = archiveStream2.str();

                            micros_swarm::CommPacket p;
                            p.packet_source = shm_rid;
                            p.packet_version = 1;
                            p.packet_type = VIRTUAL_STIGMERGY_PUT;
                            p.packet_data = vsp_new_string;
                            p.package_check_sum = 0;

                            rth_->getOutMsgQueue()->pushVstigMsgQueue(p);
                        }
                    }
                    else if((local.lamport_clock==vsp.lamport_clock)&&(local.robot_id!=vsp.robot_id))
                    {
                        //std::cout<<"put conflict"<<std::endl;
                    }
                    else
                    {
                        //do nothing
                    }
                        
                    break;
                }
                case BLACKBOARD_PUT:
                {
                    BlackBoardPut bbp;
                    archive>>bbp;
                    int robot_id=rth_->getRobotID();
                    std::string bb_key=bbp.bb_key;
                    if(bbp.on_robot_id==robot_id)
                    {
                        rth_->createBlackBoard(bbp.bb_id);
                        BlackBoardTuple bbt(bbp.bb_value, bbp.bb_timestamp, bbp.robot_id);
                        BlackBoardTuple local;
                        rth_->getBlackBoardTuple(bbp.bb_id, bbp.bb_key, local);

                        //local tuple is not exist or the local timestamp is smaller
                        if((local.bb_timestamp==0)||(local.bb_timestamp<bbp.bb_timestamp))
                        {
                            rth_->insertOrUpdateBlackBoard(bbp.bb_id, bbp.bb_key, bbp.bb_value, bbp.bb_timestamp, bbp.robot_id);
                        }
                    }
                    else{}

                    break;
                }
                case BLACKBOARD_QUERY:
                {
                    BlackBoardQuery bbq;
                    archive>>bbq;
                    int robot_id=rth_->getRobotID();
                    std::string bb_key=bbq.bb_key;

                    if(bbq.on_robot_id==robot_id)
                    {
                        BlackBoardTuple local;
                        rth_->getBlackBoardTuple(bbq.bb_id, bbq.bb_key, local);

                        BlackBoardQueryAck bbqa(bbq.bb_id, bbq.on_robot_id, bbq.bb_key, local.bb_value, time(0), bbq.robot_id);
                        std::ostringstream archiveStream2;
                        boost::archive::text_oarchive archive2(archiveStream2);
                        archive2<<bbqa;
                        std::string bbqa_string=archiveStream2.str();

                        micros_swarm::CommPacket p;
                        p.packet_source=shm_rid;
                        p.packet_version=1;
                        p.packet_type=BLACKBOARD_QUERY_ACK;
                        p.packet_data=bbqa_string;
                        p.package_check_sum=0;
                        rth_->getOutMsgQueue()->pushBbMsgQueue(p);
                    }
                    else{}

                    break;
                }
                case BLACKBOARD_QUERY_ACK:
                {
                    BlackBoardQueryAck bbqa;
                    archive>>bbqa;
                    int robot_id=rth_->getRobotID();
                    std::string bb_key=bbqa.bb_key;

                    if(bbqa.on_robot_id==robot_id){}
                    else
                    {
                        rth_->createBlackBoard(bbqa.bb_id);
                        rth_->insertOrUpdateBlackBoard(bbqa.bb_id, bbqa.bb_key, bbqa.bb_value, bbqa.bb_timestamp, bbqa.robot_id);
                    }

                    break;
                }
                case SCDS_PSO_PUT:
                {
                    SCDSPSOPut scds_put;
                    archive>>scds_put;

                    SCDSPSODataTuple local;
                    bool exist = rth_->getSCDSPSOValue(scds_put.key, local);

                    //local tuple is not exist or local timestamp is smaller
                    if ((!exist) || (local.val < scds_put.val))
                    {
                        //std::cout<<"iiiiiii"<<std::endl;
                        SCDSPSODataTuple new_data;
                        new_data.pos = scds_put.pos;
                        new_data.val = scds_put.val;
                        new_data.robot_id = scds_put.robot_id;
                        new_data.gen = scds_put.gen;
                        new_data.timestamp = scds_put.timestamp;
                        rth_->insertOrUpdateSCDSPSOValue(scds_put.key, new_data);

                        micros_swarm::CommPacket p;
                        p.packet_source=shm_rid;
                        p.packet_version=1;
                        p.packet_type=SCDS_PSO_PUT;
                        p.packet_data=packet_data;
                        p.package_check_sum=0;
                        rth_->getOutMsgQueue()->pushSCDSPSOMsgQueue(p);
                    }

                    break;
                }
                case SCDS_PSO_GET:
                {
                    SCDSPSOGet scds_get;
                    archive>>scds_get;

                    SCDSPSODataTuple local;
                    bool exist = rth_->getSCDSPSOValue(scds_get.key, local);

                    //local tuple is not exist or local timestamp is smaller
                    if ((!exist) || (local.val < scds_get.val))
                    {
                        SCDSPSODataTuple new_data;
                        new_data.pos = scds_get.pos;
                        new_data.val = scds_get.val;
                        new_data.robot_id = scds_get.robot_id;
                        new_data.gen = scds_get.gen;
                        new_data.timestamp = scds_get.timestamp;
                        rth_->insertOrUpdateSCDSPSOValue(scds_get.key, new_data);

                        micros_swarm::CommPacket p;
                        p.packet_source=shm_rid;
                        p.packet_version=1;
                        p.packet_type=SCDS_PSO_PUT;
                        p.packet_data=packet_data;
                        p.package_check_sum=0;
                        rth_->getOutMsgQueue()->pushSCDSPSOMsgQueue(p);
                    }
                    else if(local.val > scds_get.val)
                    {
                        SCDSPSOPut scds_put(scds_get.key, local.pos, local.val, local.robot_id, local.gen, local.timestamp);

                        std::ostringstream archiveStream2;
                        boost::archive::text_oarchive archive2(archiveStream2);
                        archive2<<scds_put;
                        std::string scds_put_string=archiveStream2.str();

                        micros_swarm::CommPacket p;
                        p.packet_source=shm_rid;
                        p.packet_version=1;
                        p.packet_type=SCDS_PSO_PUT;
                        p.packet_data=scds_put_string;
                        p.package_check_sum=0;

                        rth_->getOutMsgQueue()->pushSCDSPSOMsgQueue(p);
                    }
                    else
                    {

                    }

                    break;
                }
                case NEIGHBOR_BROADCAST_KEY_VALUE:
                {
                    NeighborBroadcastKeyValue nbkv;
                    archive>>nbkv;
                        
                    boost::shared_ptr<ListenerHelper> helper=rth_->getListenerHelper(nbkv.key);
                    if(helper==NULL)
                        return;
                    helper->call(nbkv.value);
               
                    break;
                }
                case BARRIER_SYN:
                {
                    Barrier_Syn bs;
                    archive>>bs;
                    if(bs.s!="SYN")
                        return;
                
                    Barrier_Ack ba(packet.packet_source);
                
                    std::ostringstream archiveStream2;
                    boost::archive::text_oarchive archive2(archiveStream2);
                    archive2<<ba;
                    std::string ba_string=archiveStream2.str();
                    
                    micros_swarm::CommPacket p;
                    p.packet_source=shm_rid;
                    p.packet_version=1;
                    p.packet_type=BARRIER_ACK;
                    p.packet_data=ba_string;
                    p.package_check_sum=0;

                    rth_->getOutMsgQueue()->pushBarrierMsgQueue(p);
                    break;
                }
                case BARRIER_ACK:
                {
                    Barrier_Ack ba;
                    archive>>ba;
                
                    if(shm_rid==ba.robot_id)
                        rth_->insertBarrier(packet.packet_source);
                    
                    break;
                }
            
                default:
                {
                    std::cout<<"UNDEFINED PACKET TYPE!"<<std::endl;
                }
            }
        }
        catch(const boost::archive::archive_exception&)
        {
            return;
        }
    }
};
