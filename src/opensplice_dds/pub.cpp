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
#include "opensplice_dds/check_status.h"
#include "opensplice_dds/publisher.h"

#define MAX_PACKET_LEN 256
#define NUM_PACKET 1000000

using namespace DDS;

int main()
{
    micros_swarm_framework::MSFPPacket packet;
    checkHandle(&packet, "new MSFPPacket");
    
    char buf[MAX_PACKET_LEN];
    
    std::string test=NULL;
    
    micros_swarm_framework::Publisher publisher("micros_swarm_framework_topic");
    
    for (int i = 1; i <= NUM_PACKET; i++) {
        packet.packet_source = 1;
        packet.packet_version = 0;
        packet.packet_type = 0;
        snprintf(buf, MAX_PACKET_LEN, "Packet no. %d", i);
        packet.packet_data = string_dup(buf);
        packet.package_check_sum=0;
        std::cout << "Writing packet: \"" << packet.packet_data << "\"" << endl;
        publisher.publish(packet);
        sleep (1); 
    }

    return 0;
}
