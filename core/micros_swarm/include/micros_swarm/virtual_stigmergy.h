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
#include "micros_swarm/message.h"
#include "micros_swarm/singleton.h"
#include "micros_swarm/runtime_handle.h"
#include "micros_swarm/comm_interface.h"

namespace micros_swarm{
    
    template<class Type>
    class VirtualStigmergy{
        public:
            VirtualStigmergy(){vstig_id_=-1;}
            
            VirtualStigmergy(int vstig_id)
            {
                vstig_id_=vstig_id;
                rth_=Singleton<RuntimeHandle>::getSingleton();
                communicator_=Singleton<CommInterface>::getExistedSingleton();
                rth_->createVirtualStigmergy(vstig_id_);
                robot_id_=rth_->getRobotID();
            }
            
            VirtualStigmergy(const VirtualStigmergy& vs)
            {
                rth_=Singleton<RuntimeHandle>::getSingleton();
                communicator_=Singleton<CommInterface>::getExistedSingleton();
                vstig_id_=vs.vstig_id_;
                robot_id_=vs.robot_id_;
            }
            
            VirtualStigmergy& operator=(const VirtualStigmergy& vs)
            {
                if(this==&vs)
                    return *this;
                rth_=Singleton<RuntimeHandle>::getSingleton();
                communicator_=Singleton<CommInterface>::getExistedSingleton();
                vstig_id_=vs.vstig_id_;
                robot_id_=vs.robot_id_;
                return *this;
            }
            
            ~VirtualStigmergy()
            {
                rth_.reset();
                communicator_.reset();
            }
            
            void put(const std::string& key, const Type& data)
            {
                std::ostringstream archiveStream;
                boost::archive::text_oarchive archive(archiveStream);
                archive<<data;
                std::string s=archiveStream.str();

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
                    VirtualStigmergyPut vsp(vstig_id_, key, local.vstig_value, local.lamport_clock, robot_id_);

                    std::ostringstream archiveStream2;
                    boost::archive::text_oarchive archive2(archiveStream2);
                    archive2 << vsp;
                    std::string vsp_str = archiveStream2.str();

                    micros_swarm::CommPacket p;
                    p.packet_source = rth_->getRobotID();
                    p.packet_version = 1;
                    p.packet_type = VIRTUAL_STIGMERGY_PUT;
                    p.packet_data = vsp_str;
                    p.package_check_sum = 0;

                    rth_->getOutMsgQueue()->pushVstigMsgQueue(p);
                }
            }
            
            Type get(const std::string& key)
            {
                VirtualStigmergyTuple vst;
                bool success = rth_->getVirtualStigmergyTuple(vstig_id_, key, vst);
                
                if(!success)
                {
                    std::cout<<"ID "<<vstig_id_<<" virtual stigmergy, "<<key<<" is not exist."<<std::endl;
                    exit(-1);
                }

                std::string data_str = vst.vstig_value;
                Type data;
                std::istringstream archiveStream(data_str);
                boost::archive::text_iarchive archive(archiveStream);
                archive >> data;

                rth_->updateVirtualStigmergyTupleReadCount(vstig_id_, key, vst.read_count+1);
                VirtualStigmergyTuple new_local;
                bool get_new_local = rth_->getVirtualStigmergyTuple(vstig_id_, key, new_local);
                if(get_new_local) {
                    //cold-hot dada distinction
                    double temperature = new_local.getTemperature();
                    double  rt = (double)rand()/RAND_MAX;
                    //std::cout<<"<<"<<temperature<<", "<<rt<<">>"<<std::endl;
                    if(rt <= temperature) {
                        VirtualStigmergyQuery vsq(vstig_id_, key, new_local.vstig_value, new_local.lamport_clock,
                                                  new_local.robot_id);

                        std::ostringstream archiveStream2;
                        boost::archive::text_oarchive archive2(archiveStream2);
                        archive2 << vsq;
                        std::string vsq_str = archiveStream2.str();

                        micros_swarm::CommPacket p;
                        p.packet_source = robot_id_;
                        p.packet_version = 1;
                        p.packet_type = VIRTUAL_STIGMERGY_QUERY;
                        p.packet_data = vsq_str;
                        p.package_check_sum = 0;

                        rth_->getOutMsgQueue()->pushVstigMsgQueue(p);
                    }
                }
                
                return data;  
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
            boost::shared_ptr<CommInterface> communicator_;
    };
}
#endif
