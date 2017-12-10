/**
Software License Agreement (BSD)
\file      testrtp.cpp 
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

#include <ros/ros.h>
#include "micros_swarm/micros_swarm.h"

using namespace micros_swarm;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "micros_swarm_framework_main_node");
    ros::NodeHandle nh;
    
    RuntimeHandle rth;
    rth.setRobotID(1);
    
    Base base(1,1,1,2,2,2,1);
    rth.setRobotBase(base);
    rth.printRobotBase();
    std::cout<<std::endl;
    
    rth.insertOrUpdateNeighbor(2, 0, 0, 0, 1, 1, 1,2, 2, 2);
    rth.insertOrUpdateNeighbor(3, 1, 2, 3, 2, 2, 2,3, 3, 3);
    rth.insertOrUpdateNeighbor(4, 2, 2, 2, 1, 2, 3,3, 3, 3);
    rth.printNeighbor();
    std::cout<<"------"<<std::endl;
    rth.insertOrUpdateNeighbor(2, 1, 2, 3, 4, 5, 6,7, 8, 9);
    rth.deleteNeighbor(3);
    rth.deleteNeighbor(16);
    rth.printNeighbor();
    std::cout<<std::endl;
    
    rth.insertOrUpdateSwarm(1, true);
    rth.insertOrUpdateSwarm(2, false);
    rth.insertOrUpdateSwarm(3, true);
    rth.printSwarm();
    std::cout<<"------"<<std::endl;
    rth.insertOrUpdateSwarm(2, true);
    rth.deleteSwarm(1);
    rth.deleteSwarm(16);
    rth.printSwarm();
    std::cout<<rth.getSwarmFlag(1)<<std::endl;
    std::cout<<rth.getSwarmFlag(2)<<std::endl;
    std::cout<<std::endl;
    
    std::vector<int> s1, s2;
    s1.push_back(1); s1.push_back(2); s1.push_back(3);
    s2.push_back(4); s2.push_back(5); s2.push_back(6);
    rth.insertOrRefreshNeighborSwarm(2, s1);
    rth.insertOrRefreshNeighborSwarm(3, s2);
    rth.printNeighborSwarm();
    std::cout<<rth.inNeighborSwarm(2, 2)<<std::endl;
    std::cout<<rth.inNeighborSwarm(2, 4)<<std::endl;
    std::cout<<rth.inNeighborSwarm(3, 5)<<std::endl;
    std::cout<<rth.inNeighborSwarm(3, 1)<<std::endl;
    std::cout<<"------"<<std::endl;
    rth.joinNeighborSwarm(2, 4);
    rth.joinNeighborSwarm(2, 5);
    rth.leaveNeighborSwarm(2, 2);
    rth.leaveNeighborSwarm(2, 3);
    rth.leaveNeighborSwarm(2, 17);
    rth.printNeighborSwarm();
    std::cout<<"------"<<std::endl;
    rth.deleteNeighborSwarm(2);
    rth.deleteNeighborSwarm(9);
    rth.printNeighborSwarm();
    std::cout<<std::endl;
    
    rth.createVirtualStigmergy(1);
    rth.createVirtualStigmergy(2);
    std::vector<uint8_t> val1;
    val1.push_back(1);
    rth.insertOrUpdateVirtualStigmergy(1, "key1", val1, 1, time(0), 0, 1);
    std::vector<uint8_t> val2;
    val2.push_back(2);
    rth.insertOrUpdateVirtualStigmergy(1, "key2", val2, 1, time(0), 0, 1);
    std::vector<uint8_t> val3;
    val3.push_back(3);
    rth.insertOrUpdateVirtualStigmergy(1, "key3", val3, 1, time(0), 0, 1);
    rth.printVirtualStigmergy();
    VirtualStigmergyTuple vst1;
    rth.getVirtualStigmergyTuple(1, "key2", vst1);
    vst1.print();
    VirtualStigmergyTuple vst2;
    rth.getVirtualStigmergyTuple(2, "key3", vst2);
    vst2.print();
    std::cout<<rth.getVirtualStigmergySize(1)<<std::endl;;
    std::cout<<rth.getVirtualStigmergySize(2)<<std::endl;
    rth.deleteVirtualStigmergyValue(1, "key2");
    rth.deleteVirtualStigmergyValue(1, "key6");
    rth.deleteVirtualStigmergyValue(2, "key3");
    rth.deleteVirtualStigmergyValue(5, "key7");
    rth.printVirtualStigmergy();
    VirtualStigmergyTuple vst3;
    rth.getVirtualStigmergyTuple(2, "key7", vst3);
    vst3.print();
    VirtualStigmergyTuple vst4;
    rth.getVirtualStigmergyTuple(7, "key11", vst4);
    vst4.print();
    rth.deleteVirtualStigmergy(2);
    rth.deleteVirtualStigmergy(5);
    rth.printVirtualStigmergy();
    
    ros::spin();
    return 0;
}
