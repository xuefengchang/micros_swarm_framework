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

#include "ros/ros.h"
#include "micros_swarm_framework/runtime_platform.h"

using namespace micros_swarm_framework;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "micros_swarm_framework_main_node");
    ros::NodeHandle nh;
    
    RuntimePlatform rtp(1);
    
    Base base(1,1,1,2,2,2);
    rtp.setRobotBase(base);
    rtp.printRobotBase();
    std::cout<<std::endl;
    
    rtp.insertOrUpdateNeighbor(2, 0, 0, 0, 1, 1, 1,2, 2, 2);
    rtp.insertOrUpdateNeighbor(3, 1, 2, 3, 2, 2, 2,3, 3, 3);
    rtp.insertOrUpdateNeighbor(4, 2, 2, 2, 1, 2, 3,3, 3, 3);
    rtp.printNeighbor();
    std::cout<<"------"<<std::endl;
    rtp.insertOrUpdateNeighbor(2, 1, 2, 3, 4, 5, 6,7, 8, 9);
    rtp.deleteNeighbor(3);
    rtp.deleteNeighbor(16);
    rtp.printNeighbor();
    std::cout<<std::endl;
    
    rtp.insertOrUpdateSwarm(1, true);
    rtp.insertOrUpdateSwarm(2, false);
    rtp.insertOrUpdateSwarm(3, true);
    rtp.printSwarm();
    std::cout<<"------"<<std::endl;
    rtp.insertOrUpdateSwarm(2, true);
    rtp.deleteSwarm(1);
    rtp.deleteSwarm(16);
    rtp.printSwarm();
    std::cout<<rtp.getSwarm(1)<<std::endl;
    std::cout<<rtp.getSwarm(2)<<std::endl;
    std::cout<<std::endl;
    
    std::vector<int> s1, s2;
    s1.push_back(1); s1.push_back(2); s1.push_back(3);
    s2.push_back(4); s2.push_back(5); s2.push_back(6);
    rtp.insertOrRefreshOthersSwarm(2, s1);
    rtp.insertOrRefreshOthersSwarm(3, s2);
    rtp.printOthersSwarm();
    std::cout<<rtp.inOthersSwarm(2, 2)<<std::endl;
    std::cout<<rtp.inOthersSwarm(2, 4)<<std::endl;
    std::cout<<rtp.inOthersSwarm(3, 5)<<std::endl;
    std::cout<<rtp.inOthersSwarm(3, 1)<<std::endl;
    std::cout<<"------"<<std::endl;
    rtp.joinOthersSwarm(2, 4);
    rtp.joinOthersSwarm(2, 5);
    rtp.leaveOthersSwarm(2, 2);
    rtp.leaveOthersSwarm(2, 3);
    rtp.leaveOthersSwarm(2, 17);
    rtp.printOthersSwarm();
    std::cout<<"------"<<std::endl;
    rtp.deleteOthersSwarm(2);
    rtp.deleteOthersSwarm(9);
    rtp.printOthersSwarm();
    std::cout<<std::endl;
    
    rtp.createVirtualStigmergy(1);
    rtp.createVirtualStigmergy(2);
    rtp.insertOrUpdateVirtualStigmergy(1, "key1", "value1", time(0), 1);
    rtp.insertOrUpdateVirtualStigmergy(1, "key2", "value2", time(0), 1);
    rtp.insertOrUpdateVirtualStigmergy(1, "key2", "value2", time(0), 1);
    rtp.insertOrUpdateVirtualStigmergy(1, "key3", "value3", time(0), 1);
    rtp.insertOrUpdateVirtualStigmergy(2, "key1", "value1", time(0), 2);
    rtp.insertOrUpdateVirtualStigmergy(2, "key2", "value2", time(0), 2);
    rtp.insertOrUpdateVirtualStigmergy(2, "key2", "value2", time(0), 2);
    rtp.insertOrUpdateVirtualStigmergy(2, "key3", "value3", time(0), 2);
    rtp.printVirtualStigmergy();
    rtp.getVirtualStigmergyTuple(1, "key2").print();
    rtp.getVirtualStigmergyTuple(2, "key3").print();
    std::cout<<rtp.getVirtualStigmergySize(1)<<std::endl;;
    std::cout<<rtp.getVirtualStigmergySize(2)<<std::endl;
    rtp.deleteVirtualStigmergyValue(1, "key2");
    rtp.deleteVirtualStigmergyValue(1, "key6");
    rtp.deleteVirtualStigmergyValue(2, "key3");
    rtp.deleteVirtualStigmergyValue(5, "key7");
    rtp.printVirtualStigmergy();
    rtp.getVirtualStigmergyTuple(2, "key7").print();
    rtp.getVirtualStigmergyTuple(7, "key11").print();
    rtp.deleteVirtualStigmergy(2);
    rtp.deleteVirtualStigmergy(5);
    rtp.printVirtualStigmergy();
    
    ros::spin();
    return 0;
}
