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
#include <string>
#include <time.h>
#include <stdlib.h>
#include <vector>
#include <stack>
#include <map>
#include <set>
#include <queue>
#include <algorithm>

#include "ros/ros.h"

#include "micros_swarm_framework/data_type.h"
#include "micros_swarm_framework/message.h"

#ifdef ROS
#include "micros_swarm_framework/MSFPPacket.h"
#endif

#ifdef OPENSPLICE_DDS
#include "opensplice_dds/MSFPPacket.h"
#include "opensplice_dds/check_status.h"
#include "opensplice_dds/publisher.h"
#include "opensplice_dds/subscriber.h"
#endif

#include "micros_swarm_framework/communication_interface.h"
#ifdef ROS
#include "micros_swarm_framework/ros_communication.h"
#endif
#ifdef OPENSPLICE_DDS
#include "micros_swarm_framework/opensplice_dds_communication.h"
#endif

namespace micros_swarm_framework{

    void packetParser(const MSFPPacket& packet)
    {
        static boost::shared_ptr<RuntimePlatform> rtp=Singleton<RuntimePlatform>::getSingleton();   
        #ifdef ROS
        static boost::shared_ptr<CommunicationInterface> communicator=Singleton<ROSCommunication>::getSingleton();
        #endif
        #ifdef OPENSPLICE_DDS
        static boost::shared_ptr<CommunicationInterface> communicator=Singleton<OpenSpliceDDSCommunication>::getSingleton();
        #endif
        
        int shm_rid=rtp->getRobotID();
        
        //ignore the packet of the local robot
        if(packet.packet_source==shm_rid)
        {
            return;
        }
        
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
            case SINGLE_ROBOT_BROADCAST_ID:{
                //std::cout<<"SINGLE_ROBOT_BROADCAST_ID"<<std::endl;
                micros_swarm_framework::SingleRobotBroadcastID srbi;
                
                archive>>srbi;
                
                int robot_id=srbi.getRobotID();
                float distance=0;
                float azimuth=0;
                float elevation=0;
                float x=srbi.getRobotX();
                float y=srbi.getRobotY();
                float z=srbi.getRobotZ();
                float vx=srbi.getRobotVX();
                float vy=srbi.getRobotVY();
                float vz=srbi.getRobotVZ();
                
                Base self=rtp->getRobotBase();
                Base neighbor(x, y, z, vx, vy, vz);
                
                double neighbor_distance=rtp->getNeighborDistance();
                CheckNeighbor cn(neighbor_distance);
                micros_swarm_framework::CheckNeighborInterface *cn_p=&cn;
                
                if(cn_p->isNeighbor(self, neighbor))
                {
                    rtp->insertOrUpdateNeighbor(robot_id, distance, azimuth, elevation, x, y, z, vx, vy, vz);
                }
                else
                {
                    rtp->deleteNeighbor(robot_id);
                }
                
                break;
            }
            case SINGLE_ROBOT_JOIN_SWARM:{
                //std::cout<<"SINGLE_ROBOT_JOIN_SWARM"<<std::endl;
                SingleRobotJoinSwarm srjs;
                archive>>srjs;
             
                int robot_id=srjs.getRobotID();
                int swarm_id=srjs.getSwarmID();
                rtp->joinNeighborSwarm(robot_id, swarm_id);
                
                /*
                if(!rtp->inNeighborSwarm(robot_id, swarm_id))
                {
                    rtp->joinNeighborSwarm(robot_id, swarm_id);
                    
                    std::ostringstream archiveStream;
                    boost::archive::text_oarchive archive(archiveStream);
                    archive<<srjs;
                    std::string srjs_str=archiveStream.str();   
                      
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
                
                    communicator->broadcast(p);
                }
                */
                
                break;
            }
            case SINGLE_ROBOT_LEAVE_SWARM:
            {
                SingleRobotLeaveSwarm srls;
                archive>>srls;
                
                int robot_id=srls.getRobotID();
                int swarm_id=srls.getSwarmID();
                rtp->leaveNeighborSwarm(robot_id, swarm_id);
                
                /*
                if(rtp->inNeighborSwarm(robot_id, swarm_id))
                {
                    rtp->leaveNeighborSwarm(robot_id, swarm_id);
                    
                    std::ostringstream archiveStream;
                    boost::archive::text_oarchive archive(archiveStream);
                    archive<<srls;
                    std::string srls_str=archiveStream.str();   
                      
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
                
                    communicator->broadcast(p);
                }
                */
                
                break;
            }
            case SINGLE_ROBOT_SWARM_LIST:
            {
                //std::cout<<"SINGLE_ROBOT_SWARM_LIST"<<std::endl;
                SingleRobotSwarmList srsl;
                archive>>srsl;
                
                int robot_id=srsl.getRobotID();
                std::vector<int> swarm_list=srsl.getSwarmList();
                rtp->insertOrRefreshNeighborSwarm(robot_id, swarm_list);
                std::vector<int>().swap(swarm_list);
                break;
            }
            case VIRTUAL_STIGMERGY_QUERY:
            {
                //std::cout<<"VIRTUAL_STIGMERGY_PUT"<<std::endl;
                VirtualStigmergyQuery vsq;
                archive>>vsq;
                
                int id=vsq.getVirtualStigmergyID();
                std::string key_std=vsq.getVirtualStigmergyKey();
                std::string value_std=vsq.getVirtualStigmergyValue();
                time_t time_now=vsq.getVirtualStigmergyTimestamp();
                int robot_id=vsq.getRobotID();
                
                VirtualStigmergyTuple local=rtp->getVirtualStigmergyTuple(id, key_std);
                
                //local tuple is not exist or the local timestamp is smaller
                if((local.getVirtualStigmergyTimestamp()==0)|| \
                   (local.getVirtualStigmergyTimestamp()<time_now))
                {
                    rtp->createVirtualStigmergy(id);
                    rtp->insertOrUpdateVirtualStigmergy(id, key_std, value_std, time_now, robot_id);
                    
                    VirtualStigmergyPut vsp(id, key_std, value_std, time_now, robot_id);
                    
                    std::ostringstream archiveStream;
                    boost::archive::text_oarchive archive(archiveStream);
                    archive<<vsp;
                    std::string vsp_string=archiveStream.str();
                    
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
                    
                    communicator->broadcast(p);
                }
                else if(local.getVirtualStigmergyTimestamp()>time_now)  //local timestamp is larger
                {
                    VirtualStigmergyPut vsp(id, key_std, local.getVirtualStigmergyValue(), local.getVirtualStigmergyTimestamp(), local.getRobotID());
                    std::ostringstream archiveStream;
                    boost::archive::text_oarchive archive(archiveStream);
                    archive<<vsp;
                    std::string vsp_string=archiveStream.str();
                    
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
                    
                    communicator->broadcast(p);
                }
                else if((local.getVirtualStigmergyTimestamp()==time_now)&& \
                        (local.getRobotID()!=robot_id))
                {
                    std::cout<<"query conflict"<<std::endl;
                }
                else
                {
                    //do nothing
                }
                
                break;
            }
            case VIRTUAL_STIGMERGY_PUT:
            {
                //std::cout<<"VIRTUAL_STIGMERGY_PUT"<<std::endl;
                VirtualStigmergyPut vsp;
                archive>>vsp;
                
                int id=vsp.getVirtualStigmergyID();
                std::string key_std=vsp.getVirtualStigmergyKey();
                std::string value_std=vsp.getVirtualStigmergyValue();
                time_t time_now=vsp.getVirtualStigmergyTimestamp();
                int robot_id=vsp.getRobotID();
                
                VirtualStigmergyTuple local=rtp->getVirtualStigmergyTuple(id, key_std);
                
                //local tuple is not exist or local timestamp is smaller
                if((local.getVirtualStigmergyTimestamp()==0)|| \
                   (local.getVirtualStigmergyTimestamp()<time_now))
                {
                    rtp->createVirtualStigmergy(id);
                
                    rtp->insertOrUpdateVirtualStigmergy(id, key_std, value_std, time_now, robot_id);
                    
                    VirtualStigmergyPut vsp(id, key_std, value_std, time_now, robot_id);
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
                    //std::cout<<"vsp_string.data(): "<<vsp_string.data()<<std::endl;
                    p.packet_data=vsp_string.data();
                    #endif
                    p.package_check_sum=0;
                    
                    communicator->broadcast(p);
                }
                else if((local.getVirtualStigmergyTimestamp()==time_now)&& \
                        (local.getRobotID()!=robot_id))
                {
                    std::cout<<"put conflict"<<std::endl;
                }
                else
                {
                    //do nothing
                }
                //std::cout<<"virtual stigmergy size: "<<rtp->getVirtualStigmergySize(id)<<std::endl;
                break;
            }
            case BARRIER_SYN:
            {
                //std::cout<<"BARRIER_SYN"<<std::endl;            
                Barrier_Syn bs;
                archive>>bs;
                if(bs.getString()!="SYN")
                {
                    return;
                }
                
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
                    
                communicator->broadcast(p);
                
                break;
            }
            case BARRIER_ACK:
            {
                //std::cout<<"BARRIER_ACK"<<std::endl;
                Barrier_Ack ba;
                archive>>ba;
                
                if(shm_rid==ba.getRobotID())
                    rtp->insertBarrier(packet.packet_source);
                    
                break;
            }
            case NEIGHBOR_BROADCAST_KEY_VALUE:
            {
                //std::cout<<"NEIGHBOR_BROADCAST_KEY_VALUE"<<std::endl;
                NeighborBroadcastKeyValue nbkv;
                archive>>nbkv;

                std::string type=nbkv.getType();
                std::string key=nbkv.getKey();
                std::string value=nbkv.getValue();
                
                if(type=="int")
                {
                    int r=stringToNumber<int>(value);
                    
                    NeighborCommunicationCallBack cb=rtp->getCallbackFunctions(key);
                    if(cb.which()==5)  //void(void), callback function not exist
                        return;
                    boost::function<void(int)> f=boost::get<boost::function<void(int)> >(cb);
                    f(r);
                }
                else if(type=="float")
                {
                    float r=stringToNumber<float>(value);
                    
                    NeighborCommunicationCallBack cb=rtp->getCallbackFunctions(key);
                    if(cb.which()==5)  //void(void), callback function not exist
                        return;
                    boost::function<void(float)> f=boost::get<boost::function<void(float)> >(cb);
                    f(r);
                }
                else if(type=="double")
                {
                    double r=stringToNumber<double>(value);
                    
                    NeighborCommunicationCallBack cb=rtp->getCallbackFunctions(key);  
                    if(cb.which()==5)  //void(void), callback function not exist
                        return;
                    boost::function<void(double)> f=boost::get<boost::function<void(double)> >(cb);
                    f(r);
                }
                else if(type=="bool")
                {
                    bool r=stringToNumber<bool>(value);
                    
                    NeighborCommunicationCallBack cb=rtp->getCallbackFunctions(key);  
                    if(cb.which()==5)  //void(void), callback function not exist
                        return;
                    boost::function<void(bool)> f=boost::get<boost::function<void(bool)> >(cb);
                    f(r);
                }
                else if(type=="string")
                { 
                    NeighborCommunicationCallBack cb=rtp->getCallbackFunctions(key);
                    if(cb.which()==5)  //void(void), callback function not exist
                        return;
                    boost::function<void(std::string)> f=boost::get<boost::function<void(std::string)> >(cb);
                    f(value);
                }
                else
                {
                    std::cout<<"wrong data type in neighbor communication!"<<std::endl;
                    return;
                }
               
                break;
            }
            
            default:
            {
                //std::cout<<"UNDEFINED PACKET TYPE!"<<std::endl;
            }
        }
        }catch(const boost::archive::archive_exception&){
            return;
        }
    }
    
};

#endif
