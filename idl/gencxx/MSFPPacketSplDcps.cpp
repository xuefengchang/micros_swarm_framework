#include "MSFPPacketSplDcps.h"
#include "ccpp_MSFPPacket.h"
#include "dds_type_aliases.h"

const char *
__micros_swarm_framework_MSFPPacket__name(void)
{
    return (const char*)"micros_swarm_framework::MSFPPacket";
}

const char *
__micros_swarm_framework_MSFPPacket__keys(void)
{
    return (const char*)"packet_source";
}

#include <v_kernel.h>
#include <v_topic.h>
#include <os_stdlib.h>
#include <string.h>
#include <os_report.h>

c_bool
__micros_swarm_framework_MSFPPacket__copyIn(
    c_base base,
    struct ::micros_swarm_framework::MSFPPacket *from,
    struct _micros_swarm_framework_MSFPPacket *to)
{
    c_bool result = OS_C_TRUE;
    (void) base;

    to->packet_source = (c_long)from->packet_source;
    to->packet_version = (c_ushort)from->packet_version;
    to->packet_type = (c_ushort)from->packet_type;
#ifdef OSPL_BOUNDS_CHECK
    if(from->packet_data){
        to->packet_data = c_stringNew(base, from->packet_data);
    } else {
        OS_REPORT (OS_ERROR, "copyIn", 0,"Member 'micros_swarm_framework::MSFPPacket.packet_data' of type 'c_string' is NULL.");
        result = OS_C_FALSE;
    }
#else
    to->packet_data = c_stringNew(base, from->packet_data);
#endif
    to->package_check_sum = (c_longlong)from->package_check_sum;
    return result;
}

void
__micros_swarm_framework_MSFPPacket__copyOut(
    void *_from,
    void *_to)
{
    struct _micros_swarm_framework_MSFPPacket *from = (struct _micros_swarm_framework_MSFPPacket *)_from;
    struct ::micros_swarm_framework::MSFPPacket *to = (struct ::micros_swarm_framework::MSFPPacket *)_to;
    to->packet_source = (::DDS::Long)from->packet_source;
    to->packet_version = (::DDS::UShort)from->packet_version;
    to->packet_type = (::DDS::UShort)from->packet_type;
    to->packet_data = DDS::string_dup(from->packet_data ? from->packet_data : "");
    to->package_check_sum = (::DDS::LongLong)from->package_check_sum;
}

