#include <iostream>
#include "opensplice_dds/check_status.h"
#include "opensplice_dds/subscriber.h"

using namespace DDS;

void callBack(const micros_swarm_framework::MSFPPacket& packet)
{
    //std::cout<<"Hello World!"<<std::endl;
    std::cout<<packet.packet_source<<": "<<packet.packet_version<<", "<<packet.packet_type<<", "<<packet.packet_data<<", "<<packet.package_check_sum<<std::endl;
}

int main()
{
    micros_swarm_framework::Subscriber subscriber("micros_swarm_framework_topic");
    subscriber.subscribe(callBack);
    
    while(1)
    {
        sleep(1);
    }
    
    return 0;
}
