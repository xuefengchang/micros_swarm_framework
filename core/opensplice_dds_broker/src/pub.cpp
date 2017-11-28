/**
Software License Agreement (BSD)
\file      pub.cpp
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
#include <vector>
#include "opensplice_dds_broker/check_status.h"
#include "opensplice_dds_broker/publisher.h"
#include "gsdf_msgs/CommPacket.h"
#include "gsdf_msgs/JoinSwarm.h"
#include "micros_swarm/message.h"

#define MAX_PACKET_LEN 1024
#define NUM_PACKET 1000000

using namespace DDS;

void dump_string(const std::string& s)
{
    std::cout<<"length: "<<s.length()<<", data: ";
    for(int i = 0; i < s.length(); i++) {
        std::cout<<(int)s[i];
    }
    std::cout<<std::endl;
}

void dump_char_seq(char* s, int len)
{
    std::cout<<"length: "<<len<<", data: ";
    for(int i = 0; i < len; i++) {
        std::cout<<(int)(*(s+i));
    }
    std::cout<<std::endl;
}

void dump_char_vec(std::vector<uint8_t> vec)
{
    std::cout<<"length: "<<vec.size()<<", data: ";
    for(int i = 0; i < vec.size(); i++) {
        std::cout<<(int)(vec[i]);
    }
    std::cout<<std::endl;
}

int main()
{
    opensplice_dds_broker::GSDFPacket packet;
    checkHandle(&packet, "new MSFPPacket");
    char buf[MAX_PACKET_LEN];
    
    std::string test = "";
    opensplice_dds_broker::Publisher publisher("micros_swarm_framework_topic");

    gsdf_msgs::JoinSwarm js;
    js.robot_id = 1;
    js.swarm_id = 2;
    std::vector<uint8_t> msg_data = micros_swarm::serialize_ros(js);

    unsigned int bufsize = msg_data.size();
    std::cout<<bufsize<<std::endl;
    packet.data.replace(bufsize, bufsize, (char*)(&msg_data[0]), false);

    for (int i = 1; i <= NUM_PACKET; i++) {
        dump_char_vec(msg_data);
        publisher.publish(packet);
        sleep (1); 
    }
    return 0;
}
