/**
Software License Agreement (BSD)
\file      blackboard.h
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

#ifndef BLACKBOARD_H_
#define BLACKBOARD_H_

#include <iostream>
#include <ros/ros.h>

#include "micros_swarm/singleton.h"
#include "micros_swarm/packet_type.h"
#include "micros_swarm/serialize.h"
#include "micros_swarm/runtime_handle.h"
#include "micros_swarm/msg_queue_manager.h"
#include "gsdf_msgs/CommPacket.h"
#include "gsdf_msgs/BlackBoardPut.h"
#include "gsdf_msgs/BlackBoardQuery.h"
#include "gsdf_msgs/BlackBoardAck.h"

namespace micros_swarm{
    
    template<class Type>
    class BlackBoard{
        public:
            BlackBoard()
            {
                bb_id_ = -1;
                on_robot_id_ = -1;
                robot_id_ = -1;
                is_local_ = false;
            }

            BlackBoard(int bb_id, int on_robot_id)
            {
                bb_id_ = bb_id;
                on_robot_id_ = on_robot_id;
                rth_ = Singleton<RuntimeHandle>::getSingleton();
                robot_id_ = rth_->getRobotID();
                mqm_ = Singleton<MsgQueueManager>::getSingleton();
                is_local_ = false;
                if(on_robot_id_ == robot_id_) {
                    rth_->createBlackBoard(bb_id_);
                    is_local_ = true;
                }
            }

            BlackBoard(const BlackBoard& bb)
            {
                rth_ = Singleton<RuntimeHandle>::getSingleton();
                mqm_ = Singleton<MsgQueueManager>::getSingleton();
                bb_id_ = bb.bb_id_;
                on_robot_id_ = bb.on_robot_id_;
                robot_id_ = bb.robot_id_;
                is_local_ = bb.is_local_;
            }

            BlackBoard& operator=(const BlackBoard& bb)
            {
                if(this == &bb) {
                    return *this;
                }
                rth_ = Singleton<RuntimeHandle>::getSingleton();
                mqm_ = Singleton<MsgQueueManager>::getSingleton();
                bb_id_ = bb.bb_id_;
                on_robot_id_ = bb.on_robot_id_;
                robot_id_ = bb.robot_id_;
                is_local_ = bb.is_local_;
                return *this;
            }
            
            ~BlackBoard()
            {
                rth_.reset();
                mqm_.reset();
            }
            
            void put(const std::string& key, const Type& data)
            {
                std::vector<uint8_t> s = serialize_ros(data);
                ros::Time timestamp = ros::Time::now();
                if(is_local_) {
                    rth_->insertOrUpdateBlackBoard(bb_id_, key, s, timestamp, rth_->getRobotID());
                }
                else {
                    gsdf_msgs::BlackBoardPut bbp;
                    bbp.bb_id = bb_id_;
                    bbp.on_robot_id = on_robot_id_;
                    bbp.key = key;
                    bbp.value = s;
                    bbp.timestamp = timestamp;
                    bbp.robot_id = rth_->getRobotID();
                    std::vector<uint8_t> bbp_vec = serialize_ros(bbp);

                    gsdf_msgs::CommPacket p;
                    p.header.source = rth_->getRobotID();
                    p.header.type = BLACKBOARD_PUT;
                    p.header.data_len = bbp_vec.size();
                    p.header.version = 1;
                    p.header.checksum = 0;
                    p.content.buf = bbp_vec;
                    std::vector<uint8_t> msg_data = serialize_ros(p);
                    mqm_->getOutMsgQueue("bb")->push(msg_data);
                }
            }
            
            Type get(const std::string& key)
            {
                if(is_local_) {
                    BlackBoardTuple bb;
                    rth_->getBlackBoardTuple(bb_id_, key, bb);

                    if ((bb.timestamp.sec == 0) && (bb.timestamp.nsec == 0)) {
                        std::cout << "ID " << bb_id_ << " blackboard, " << key << " is not exist." << std::endl;
                        exit(-1);
                    }

                    Type data = deserialize_ros<Type>(bb.bb_value);
                    return data;
                }
                else {
                    gsdf_msgs::BlackBoardQuery bbq;
                    bbq.bb_id = bb_id_;
                    bbq.on_robot_id = on_robot_id_;
                    bbq.key = key;
                    bbq.timestamp = ros::Time::now();
                    bbq.robot_id = robot_id_;
                    std::vector<uint8_t> bbq_vec = serialize_ros(bbq);

                    gsdf_msgs::CommPacket p;
                    p.header.source = rth_->getRobotID();
                    p.header.type = BLACKBOARD_QUERY;
                    p.header.data_len = bbq_vec.size();
                    p.header.version = 1;
                    p.header.checksum = 0;
                    p.content.buf = bbq_vec;
                    rth_->createBlackBoard(bb_id_);
                    ros::Time timestamp;
                    timestamp.sec = 0;
                    timestamp.nsec = 0;
                    std::vector<uint8_t> empty_vec;
                    empty_vec.clear();
                    rth_->insertOrUpdateBlackBoard(bb_id_, key, empty_vec, timestamp, -1);
                    std::vector<uint8_t> msg_data = serialize_ros(p);
                    mqm_->getOutMsgQueue("bb")->push(msg_data);

                    Type data;
                    BlackBoardTuple bbt;
                    ros::Rate loop_rate(100);
                    int count = 0;
                    while(count < 500) {
                        rth_->getBlackBoardTuple(bb_id_, key, bbt);
                        if(bbt.bb_value.size() != 0) {
                            data = deserialize_ros<Type>(bbt.bb_value);
                            rth_->deleteBlackBoardValue(bb_id_, key);
                            break;
                        }
                        loop_rate.sleep();
                        ros::spinOnce();
                    }

                    return data;
                }
            }
            
            int size()
            {
                if(is_local_) {
                    return rth_->getBlackBoardSize(bb_id_);
                }
                else {

                }
            }
        private:
            int robot_id_;
            int bb_id_;
            int on_robot_id_;
            bool is_local_;
            boost::shared_ptr<RuntimeHandle> rth_;
            boost::shared_ptr<micros_swarm::MsgQueueManager> mqm_;
    };
}
#endif
