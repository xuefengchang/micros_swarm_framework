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
    
    micros_swarm_framework::Publisher publisher("micros_swarm_framework_topic", 1);
    
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
