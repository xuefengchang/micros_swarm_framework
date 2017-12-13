/**
Software License Agreement (BSD)
\file      virtual_stigmergy.h 
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

#ifndef VIRTUAL_STIGMERGY_H_
#define VIRTUAL_STIGMERGY_H_

#include <iostream>
#include <vector>
#include <map>
#include <ros/ros.h>

#include "micros_swarm/random.h"
#include "micros_swarm/singleton.h"
#include "micros_swarm/runtime_handle.h"
#include "micros_swarm/msg_queue_manager.h"
#include "micros_swarm/packet_type.h"
#include "micros_swarm/serialize.h"
#include "gsdf_msgs/CommPacket.h"
#include "gsdf_msgs/VirtualStigmergyQuery.h"
#include "gsdf_msgs/VirtualStigmergyPut.h"
#include "gsdf_msgs/VirtualStigmergyPuts.h"

namespace micros_swarm{
    
    template<class Type>
    class VirtualStigmergy{
        public:
            VirtualStigmergy(){vstig_id_=-1;}
            
            VirtualStigmergy(int vstig_id)
            {
                vstig_id_ = vstig_id;
                rth_ = Singleton<RuntimeHandle>::getSingleton();
                mqm_ = Singleton<MsgQueueManager>::getSingleton();
                rth_->createVirtualStigmergy(vstig_id_);
                robot_id_ = rth_->getRobotID();
            }
            
            VirtualStigmergy(const VirtualStigmergy& vs)
            {
                rth_ = Singleton<RuntimeHandle>::getSingleton();
                mqm_ = Singleton<MsgQueueManager>::getSingleton();
                vstig_id_ = vs.vstig_id_;
                robot_id_ = vs.robot_id_;
            }
            
            VirtualStigmergy& operator=(const VirtualStigmergy& vs)
            {
                if(this == &vs) {
                    return *this;
                }
                rth_ = Singleton<RuntimeHandle>::getSingleton();
                mqm_ = Singleton<MsgQueueManager>::getSingleton();
                vstig_id_ = vs.vstig_id_;
                robot_id_ = vs.robot_id_;
                return *this;
            }
            
            ~VirtualStigmergy()
            {
                rth_.reset();
                mqm_.reset();
            }
            
            void put(const std::string& key, const Type& data)
            {
                std::vector<uint8_t> s = serialize_ros(data);

                if(rth_->isVirtualStigmergyTupleExist(vstig_id_, key)) {
                    VirtualStigmergyTuple vst;
                    bool success = rth_->getVirtualStigmergyTuple(vstig_id_, key, vst);
                    if(success) {
                        rth_->insertOrUpdateVirtualStigmergy(vstig_id_, key, s, vst.lamport_clock + 1, time(NULL), 0, robot_id_);
                    }
                }
                else {
                    rth_->insertOrUpdateVirtualStigmergy(vstig_id_, key, s, 1, time(NULL), 0, robot_id_);
                }

                VirtualStigmergyTuple local;
                bool success = rth_->getVirtualStigmergyTuple(vstig_id_, key, local);
                if(success) {
                    gsdf_msgs::VirtualStigmergyPut vsp;
                    vsp.vstig_id = vstig_id_;
                    vsp.key = key;
                    vsp.value = local.vstig_value;
                    vsp.lamport_clock = local.lamport_clock;
                    vsp.robot_id = robot_id_;
                    std::vector<uint8_t> vsp_vec = serialize_ros(vsp);

                    gsdf_msgs::CommPacket p;
                    p.header.source = rth_->getRobotID();
                    p.header.type = VIRTUAL_STIGMERGY_PUT;
                    p.header.data_len = vsp_vec.size();
                    p.header.version = 1;
                    p.header.checksum = 0;
                    p.content.buf = vsp_vec;
                    std::vector<uint8_t> msg_data = serialize_ros(p);
                    mqm_->getOutMsgQueue("vstig")->push(msg_data);
                }
            }
            
            Type get(const std::string& key)
            {
                VirtualStigmergyTuple vst;
                bool success = rth_->getVirtualStigmergyTuple(vstig_id_, key, vst);
                
                if(!success) {
                    std::cout<<"ID "<<vstig_id_<<" virtual stigmergy, "<<key<<" is not exist."<<std::endl;
                    exit(-1);
                }

                std::vector<uint8_t> data_vec = vst.vstig_value;
                Type data = deserialize_ros<Type>(data_vec);

                rth_->updateVirtualStigmergyTupleReadCount(vstig_id_, key, vst.read_count+1);
                VirtualStigmergyTuple new_local;
                bool get_new_local = rth_->getVirtualStigmergyTuple(vstig_id_, key, new_local);
                if(get_new_local) {
                    //cold-hot dada distinction
                    double temperature = new_local.getTemperature();
                    double  rt = (double)rand()/RAND_MAX;
                    //std::cout<<"<<"<<temperature<<", "<<rt<<">>"<<std::endl;
                    if(rt <= temperature) {
                        gsdf_msgs::VirtualStigmergyQuery vsq;
                        vsq.vstig_id = vstig_id_;
                        vsq.key = key;
                        vsq.value = new_local.vstig_value;
                        vsq.lamport_clock = new_local.lamport_clock;
                        vsq.robot_id = new_local.robot_id;
                        std::vector<uint8_t> vsq_vec = serialize_ros(vsq);

                        gsdf_msgs::CommPacket p;
                        p.header.source = robot_id_;
                        p.header.type = VIRTUAL_STIGMERGY_QUERY;
                        p.header.data_len = vsq_vec.size();
                        p.header.version = 1;
                        p.header.checksum = 0;
                        p.content.buf = vsq_vec;
                        std::vector<uint8_t> msg_data = serialize_ros(p);
                        mqm_->getOutMsgQueue("vstig")->push(msg_data);
                    }
                }
                
                return data;  
            }

            void puts(const std::string& key, const Type& data)
            {
                std::vector<uint8_t> s = serialize_ros(data);

                if(rth_->isVirtualStigmergyTupleExist(vstig_id_, key)) {
                    VirtualStigmergyTuple vst;
                    bool success = rth_->getVirtualStigmergyTuple(vstig_id_, key, vst);
                    if(success) {
                        rth_->insertOrUpdateVirtualStigmergy(vstig_id_, key, s, vst.lamport_clock + 1, time(NULL), 0, robot_id_);
                    }
                }
                else {
                    rth_->insertOrUpdateVirtualStigmergy(vstig_id_, key, s, 1, time(NULL), 0, robot_id_);
                }

                VirtualStigmergyTuple local;
                bool success = rth_->getVirtualStigmergyTuple(vstig_id_, key, local);
                if(success) {
                    gsdf_msgs::VirtualStigmergyPuts vsps;
                    vsps.vstig_id = vstig_id_;
                    vsps.key = key;
                    vsps.value = local.vstig_value;
                    vsps.lamport_clock = local.lamport_clock;
                    vsps.robot_id = robot_id_;
                    int neighbor_size = rth_->getNeighborSize();
                    if(neighbor_size < 3) {
                        vsps.prob = 1.0;
                    }
                    else {
                        vsps.prob = 2.0/neighbor_size;
                    }

                    std::vector<uint8_t> vsps_vec = serialize_ros(vsps);

                    gsdf_msgs::CommPacket p;
                    p.header.source = rth_->getRobotID();
                    p.header.type = VIRTUAL_STIGMERGY_PUTS;
                    p.header.data_len = vsps_vec.size();
                    p.header.version = 1;
                    p.header.checksum = 0;
                    p.content.buf = vsps_vec;
                    std::vector<uint8_t> msg_data = serialize_ros(p);
                    mqm_->getOutMsgQueue("vstig")->push(msg_data);
                }
            }

            Type gets(const std::string& key)
            {
                VirtualStigmergyTuple vst;
                bool success = rth_->getVirtualStigmergyTuple(vstig_id_, key, vst);

                if(!success) {
                    std::cout<<"ID "<<vstig_id_<<" virtual stigmergy, "<<key<<" is not exist."<<std::endl;
                    exit(-1);
                }

                std::vector<uint8_t> data_vec = vst.vstig_value;
                Type data = deserialize_ros<Type>(data_vec);

                return data;
            }

            void remove(const std::string& key)
            {
                rth_->deleteVirtualStigmergyValue(vstig_id_, key);
            }
            
            int size()
            {
                return rth_->getVirtualStigmergySize(vstig_id_);
            }

            void print()
            {
                rth_->printVirtualStigmergy();
            }
        private:
            int vstig_id_;
            int robot_id_;
            boost::shared_ptr<RuntimeHandle> rth_;
            boost::shared_ptr<micros_swarm::MsgQueueManager> mqm_;
    };
}
#endif
