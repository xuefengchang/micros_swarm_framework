/**
Software License Agreement (BSD)
\file      packet_parser.h
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

#ifndef PACKET_PARSER_H_
#define PACKET_PARSER_H_

#include <iostream>
#include <time.h>
#include <vector>
#include <map>

#include "micros_swarm_framework/data_type.h"
#include "micros_swarm_framework/message.h"
#include "micros_swarm_framework/singleton.h"
#include "micros_swarm_framework/runtime_platform.h"
#include "micros_swarm_framework/check_neighbor.h"
#include "micros_swarm_framework/random.h"

#include "micros_swarm_framework/comm_interface.h"
#ifdef ROS
#include "ros_comm.h"
#endif
#ifdef OPENSPLICE_DDS
#include "opensplice_dds_comm.h"
#endif

namespace micros_swarm_framework{

    class PacketParser{
        public:
            PacketParser()
            {
                rtp_=Singleton<RuntimePlatform>::getSingleton();   
                #ifdef ROS
                communicator_=Singleton<ROSComm>::getSingleton();
                #endif
                #ifdef OPENSPLICE_DDS
                communicator_=Singleton<OpenSpliceDDSComm>::getSingleton();
                #endif
            }

            void parser(const MSFPPacket& packet)
            {
                int shm_rid=rtp_->getRobotID();
                int packet_source=packet.packet_source;
        
                //ignore the packet of the local robot
                if(packet_source==shm_rid)
                    return;
        
                try{
                const int packet_type=packet.packet_type;
                #ifdef ROS
                std::string packet_data=packet.packet_data;
                #endif
                #ifdef OPENSPLICE_DDS
                std::string packet_data=(std::string)packet.packet_data;
                #endif
        
                std::istringstream archiveStream(packet_data);
                boost::archive::text_iarchive archive(archiveStream);

                switch(packet_type)
                {
                    case SINGLE_ROBOT_BROADCAST_BASE:{
                        micros_swarm_framework::SingleRobotBroadcastBase srbb;
                        archive>>srbb;
                
                        if(srbb.valid!=1)  //ignore the default Base value.
                            return;
                
                        const Base& self=rtp_->getRobotBase();
                        Base neighbor(srbb.robot_x, srbb.robot_y, srbb.robot_z, srbb.robot_vx, srbb.robot_vy, srbb.robot_vz);
                
                        float neighbor_distance=rtp_->getNeighborDistance();
                        boost::shared_ptr<CheckNeighborInterface> cni(new CheckNeighbor(neighbor_distance));
                
                        if(cni->isNeighbor(self, neighbor))
                            rtp_->insertOrUpdateNeighbor(srbb.robot_id, 0, 0, 0, srbb.robot_x, srbb.robot_y, srbb.robot_z, srbb.robot_vx, srbb.robot_vy, srbb.robot_vz);
                        else
                            rtp_->deleteNeighbor(srbb.robot_id);
                
                        break;
                    }
                    case SINGLE_ROBOT_JOIN_SWARM:{
                        SingleRobotJoinSwarm srjs;
                        archive>>srjs;
                
                        rtp_->joinNeighborSwarm(srjs.robot_id, srjs.swarm_id);
                
                        /*
                        if(!rtp_->inNeighborSwarm(robot_id, swarm_id))
                        {
                            rtp_->joinNeighborSwarm(robot_id, swarm_id);
                    
                            std::ostringstream archiveStream2;
                            boost::archive::text_oarchive archive2(archiveStream2);
                            archive2<<srjs;
                            std::string srjs_str=archiveStream2.str();   
                      
                            micros_swarm_framework::MSFPPacket p;
                            p.packet_source=shm_rid;
                            p.packet_version=1;
                            p.packet_type=SINGLE_ROBOT_JOIN_SWARM;
                            #ifdef ROS
                            p.packet_data=srjs_str;
                            #endif
                            #ifdef OPENSPLICE_DDS
                            p.packet_data=srjs_str.data();
                            #endif
                            p.package_check_sum=0;
                
                            rtp_->getOutMsgQueue()->pushSwarmMsgQueue(p);
                        }
                        */
                
                        break;
                    }
                    case SINGLE_ROBOT_LEAVE_SWARM:
                    {
                        SingleRobotLeaveSwarm srls;
                        archive>>srls;
                
                        rtp_->leaveNeighborSwarm(srls.robot_id, srls.swarm_id);
                
                        /*
                        if(rtp_->inNeighborSwarm(robot_id, swarm_id))
                        {
                            rtp_->leaveNeighborSwarm(robot_id, swarm_id);
                    
                            std::ostringstream archiveStream;
                            boost::archive::text_oarchive archive2(archiveStream2);
                            archive2<<srls;
                            std::string srls_str=archiveStream2.str();   
                      
                            micros_swarm_framework::MSFPPacket p;
                            p.packet_source=shm_rid;
                            p.packet_version=1;
                            p.packet_type=SINGLE_ROBOT_LEAVE_SWARM;
                            #ifdef ROS
                            p.packet_data=srls_str;
                            #endif
                            #ifdef OPENSPLICE_DDS
                            p.packet_data=srls_str.data();
                            #endif
                            p.package_check_sum=0;
                
                            rtp_->getOutMsgQueue()->pushSwarmMsgQueue(p);
                        }
                        */
                
                        break;
                    }
                    case SINGLE_ROBOT_SWARM_LIST:
                    {
                        SingleRobotSwarmList srsl;
                        archive>>srsl;
                
                        rtp_->insertOrRefreshNeighborSwarm(srsl.robot_id, srsl.swarm_list);
                
                        break;
                    }
                    case VIRTUAL_STIGMERGY_QUERY:
                    {
                        VirtualStigmergyQuery vsq;
                        archive>>vsq;

                        float probability = random_float(0.0, 1.0, time(NULL));
                        if((vsq.certain_receiving_id != shm_rid) && (probability>vsq.receiving_probability))
                            return;
                
                        VirtualStigmergyTuple local;
                        rtp_->getVirtualStigmergyTuple(vsq.virtual_stigmergy_id, vsq.virtual_stigmergy_key, local);
                
                        //local tuple is not exist or the local timestamp is smaller
                        if((local.vstig_timestamp==0)||(local.vstig_timestamp<vsq.virtual_stigmergy_timestamp))
                        {
                            rtp_->createVirtualStigmergy(vsq.virtual_stigmergy_id);
                            rtp_->insertOrUpdateVirtualStigmergy(vsq.virtual_stigmergy_id, vsq.virtual_stigmergy_key, vsq.virtual_stigmergy_value,
                                                                 vsq.virtual_stigmergy_timestamp, vsq.robot_id);

                            std::map<int, NeighborBase> neighbors;
                            rtp_->getNeighbors(neighbors);
                            if(neighbors.size()==0)
                                return;
                            int random_neighbor_index=random_int(0,neighbors.size()-1, time(NULL));
                            std::map<int, NeighborBase>::iterator it=neighbors.begin();
                            int loop_index=0;
                            for(loop_index=0;loop_index<=random_neighbor_index;loop_index++)
                                it++;
                            int certain_receiving_id=it->first;
                            int flooding_factor=rtp_->getFloodingFactor();
                            int flooding_radix=(neighbors.size()-1)>=flooding_factor?(neighbors.size()-1):flooding_factor;
                            float receiving_probability=(float)flooding_factor/flooding_radix;

                            VirtualStigmergyPut vsp_new(vsq.virtual_stigmergy_id, vsq.virtual_stigmergy_key, vsq.virtual_stigmergy_value, 
                                                        vsq.virtual_stigmergy_timestamp, vsq.robot_id, certain_receiving_id, receiving_probability);
                    
                            std::ostringstream archiveStream2;
                            boost::archive::text_oarchive archive2(archiveStream2);
                            archive2<<vsp_new;
                            std::string vsp_new_string=archiveStream2.str();
                    
                            micros_swarm_framework::MSFPPacket p;
                            p.packet_source=shm_rid;
                            p.packet_version=1;
                            p.packet_type=VIRTUAL_STIGMERGY_PUT;
                            #ifdef ROS
                            p.packet_data=vsp_new_string;
                            #endif
                            #ifdef OPENSPLICE_DDS
                            p.packet_data=vsp_new_string.data();
                            #endif
                            p.package_check_sum=0;
                    
                            rtp_->getOutMsgQueue()->pushVstigMsgQueue(p);
                        }
                        else if(local.vstig_timestamp>vsq.virtual_stigmergy_timestamp)  //local timestamp is larger
                        {
                            std::map<int, NeighborBase> neighbors;
                            rtp_->getNeighbors(neighbors);
                            if(neighbors.size()==0)
                                return;
                            int random_neighbor_index=random_int(0,neighbors.size()-1, time(NULL));
                            std::map<int, NeighborBase>::iterator it=neighbors.begin();
                            int loop_index=0;
                            for(loop_index=0;loop_index<=random_neighbor_index;loop_index++)
                            {
                                it++;
                            }
                            int certain_receiving_id=it->first;
                            int flooding_factor=rtp_->getFloodingFactor();
                            int flooding_radix=(neighbors.size()-1)>=flooding_factor?(neighbors.size()-1):flooding_factor;
                            float receiving_probability=(float)flooding_factor/flooding_radix;

                            VirtualStigmergyPut vsp(vsq.virtual_stigmergy_id, vsq.virtual_stigmergy_key, local.vstig_value,
                                                    local.vstig_timestamp, local.robot_id, certain_receiving_id, receiving_probability);
                            std::ostringstream archiveStream2;
                            boost::archive::text_oarchive archive2(archiveStream2);
                            archive2<<vsp;
                            std::string vsp_string=archiveStream2.str();
                    
                            micros_swarm_framework::MSFPPacket p;
                            p.packet_source=shm_rid;
                            p.packet_version=1;
                            p.packet_type=VIRTUAL_STIGMERGY_PUT;
                            #ifdef ROS
                            p.packet_data=vsp_string;
                            #endif
                            #ifdef OPENSPLICE_DDS
                            p.packet_data=vsp_string.data();
                            #endif
                            p.package_check_sum=0;
                    
                            rtp_->getOutMsgQueue()->pushVstigMsgQueue(p);
                        }
                        else if((local.vstig_timestamp==vsq.virtual_stigmergy_timestamp)&&(local.robot_id!=vsq.robot_id))
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

                        float probability = random_float(0.0, 1.0, time(NULL));
                        if((vsp.certain_receiving_id != shm_rid) && (probability>vsp.receiving_probability))
                            return;
                
                        VirtualStigmergyTuple local;
                        rtp_->getVirtualStigmergyTuple(vsp.virtual_stigmergy_id, vsp.virtual_stigmergy_key, local);
                
                        //local tuple is not exist or local timestamp is smaller
                        if((local.vstig_timestamp==0)||(local.vstig_timestamp<vsp.virtual_stigmergy_timestamp))
                        {
                            rtp_->createVirtualStigmergy(vsp.virtual_stigmergy_id);
                
                            rtp_->insertOrUpdateVirtualStigmergy(vsp.virtual_stigmergy_id, vsp.virtual_stigmergy_key, vsp.virtual_stigmergy_value, 
                                                                 vsp.virtual_stigmergy_timestamp, vsp.robot_id);

                            std::map<int, NeighborBase> neighbors;
                            rtp_->getNeighbors(neighbors);
                            if(neighbors.size()==0)
                                return;
                            int random_neighbor_index=random_int(0,neighbors.size()-1, time(NULL));
                            std::map<int, NeighborBase>::iterator it=neighbors.begin();
                            int loop_index=0;
                            for(loop_index=0;loop_index<=random_neighbor_index;loop_index++)
                            {
                                it++;
                            }
                            int certain_receiving_id=it->first;
                            int flooding_factor=rtp_->getFloodingFactor();
                            int flooding_radix=(neighbors.size()-1)>=flooding_factor?(neighbors.size()-1):flooding_factor;
                            float receiving_probability=(float)flooding_factor/flooding_radix;
                            VirtualStigmergyPut vsp_new(vsp.virtual_stigmergy_id, vsp.virtual_stigmergy_key, vsp.virtual_stigmergy_value,
                                                        vsp.virtual_stigmergy_timestamp, vsp.robot_id, certain_receiving_id, receiving_probability);
                            std::ostringstream archiveStream2;
                            boost::archive::text_oarchive archive2(archiveStream2);
                            archive2<<vsp_new;
                            std::string vsp_new_string=archiveStream2.str();
                    
                            micros_swarm_framework::MSFPPacket p;
                            p.packet_source=shm_rid;
                            p.packet_version=1;
                            p.packet_type=VIRTUAL_STIGMERGY_PUT;
                            #ifdef ROS
                            p.packet_data=vsp_new_string;
                            #endif
                            #ifdef OPENSPLICE_DDS
                            //std::cout<<"vsp_string.data(): "<<vsp_string.data()<<std::endl;
                            p.packet_data=vsp_new_string.data();
                            #endif
                            p.package_check_sum=0;
                    
                            rtp_->getOutMsgQueue()->pushVstigMsgQueue(p);
                        }
                        else if((local.vstig_timestamp==vsp.virtual_stigmergy_timestamp)&&(local.robot_id!=vsp.robot_id))
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
                        int robot_id=rtp_->getRobotID();
                        std::string bb_key=bbp.bb_key;
                        if(bbp.on_robot_id==robot_id)
                        {
                            rtp_->createBlackBoard(bbp.bb_id);
                            BlackBoardTuple bbt(bbp.bb_value, bbp.bb_timestamp, bbp.robot_id);
                            BlackBoardTuple local;
                            rtp_->getBlackBoardTuple(bbp.bb_id, bbp.bb_key, local);

                            //local tuple is not exist or the local timestamp is smaller
                            if((local.bb_timestamp==0)||(local.bb_timestamp<bbp.bb_timestamp))
                            {
                                rtp_->insertOrUpdateBlackBoard(bbp.bb_id, bbp.bb_key, bbp.bb_value, bbp.bb_timestamp, bbp.robot_id);
                            }
                        }
                        else{}

                        break;
                    }
                    case BLACKBOARD_QUERY:
                    {
                        BlackBoardQuery bbq;
                        archive>>bbq;
                        int robot_id=rtp_->getRobotID();
                        std::string bb_key=bbq.bb_key;

                        if(bbq.on_robot_id==robot_id)
                        {
                            BlackBoardTuple local;
                            rtp_->getBlackBoardTuple(bbq.bb_id, bbq.bb_key, local);

                            BlackBoardQueryAck bbqa(bbq.bb_id, bbq.on_robot_id, bbq.bb_key, local.bb_value, time(0), bbq.robot_id);
                            std::ostringstream archiveStream2;
                            boost::archive::text_oarchive archive2(archiveStream2);
                            archive2<<bbqa;
                            std::string bbqa_string=archiveStream2.str();

                            micros_swarm_framework::MSFPPacket p;
                            p.packet_source=shm_rid;
                            p.packet_version=1;
                            p.packet_type=BLACKBOARD_QUERY_ACK;
                            #ifdef ROS
                            p.packet_data=bbqa_string;
                            #endif
                            #ifdef OPENSPLICE_DDS
                            p.packet_data=bbqa_string.data();
                            #endif
                            p.package_check_sum=0;
                            rtp_->getOutMsgQueue()->pushBbMsgQueue(p);
                        }
                        else{}

                        break;
                    }
                    case BLACKBOARD_QUERY_ACK:
                    {
                        BlackBoardQueryAck bbqa;
                        archive>>bbqa;
                        int robot_id=rtp_->getRobotID();
                        std::string bb_key=bbqa.bb_key;

                        if(bbqa.on_robot_id==robot_id){}
                        else
                        {
                            rtp_->createBlackBoard(bbqa.bb_id);
                            rtp_->insertOrUpdateBlackBoard(bbqa.bb_id, bbqa.bb_key, bbqa.bb_value, bbqa.bb_timestamp, bbqa.robot_id);
                        }

                        break;
                    }
                    case NEIGHBOR_BROADCAST_KEY_VALUE:
                    {
                        NeighborBroadcastKeyValue nbkv;
                        archive>>nbkv;
                        
                        boost::shared_ptr<ListenerHelper> helper=rtp_->getListenerHelper(nbkv.key);
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
                    
                        micros_swarm_framework::MSFPPacket p;
                        p.packet_source=shm_rid;
                        p.packet_version=1;
                        p.packet_type=BARRIER_ACK;
                        #ifdef ROS
                        p.packet_data=ba_string;
                        #endif
                        #ifdef OPENSPLICE_DDS
                        p.packet_data=ba_string.data();
                        #endif
                        p.package_check_sum=0;
                    
                        communicator_->broadcast(p);
                        //ros::Duration(0.1).sleep();
                        break;
                    }
                    case BARRIER_ACK:
                    {
                        Barrier_Ack ba;
                        archive>>ba;
                
                        if(shm_rid==ba.robot_id)
                            rtp_->insertBarrier(packet.packet_source);
                    
                        break;
                    }
            
                    default:
                    {
                        std::cout<<"UNDEFINED PACKET TYPE!"<<std::endl;
                    }
                }
                }catch(const boost::archive::archive_exception&){
                    return;
                }
            }
        
        private:
            boost::shared_ptr<RuntimePlatform> rtp_;   
            boost::shared_ptr<CommInterface> communicator_;
    };
};

#endif
