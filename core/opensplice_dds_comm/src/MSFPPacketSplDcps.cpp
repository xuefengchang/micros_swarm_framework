#include "opensplice_dds_comm/MSFPPacketSplDcps.h"
#include "opensplice_dds_comm/ccpp_MSFPPacket.h"
#include "dds_type_aliases.h"

const char *
__opensplice_dds_comm_MSFPPacket__name(void)
{
    return (const char*)"opensplice_dds_comm::MSFPPacket";
}

const char *
__opensplice_dds_comm_MSFPPacket__keys(void)
{
    return (const char*)"";
}

#include <v_kernel.h>
#include <v_topic.h>
#include <os_stdlib.h>
#include <string.h>
#include <os_report.h>

c_bool
__opensplice_dds_comm_MSFPPacket__copyIn(
    c_base base,
    struct ::opensplice_dds_comm::MSFPPacket *from,
    struct _opensplice_dds_comm_MSFPPacket *to)
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
        OS_REPORT (OS_ERROR, "copyIn", 0,"Member 'opensplice_dds_comm::MSFPPacket.packet_data' of type 'c_string' is NULL.");
        result = OS_C_FALSE;
    }
#else
    to->packet_data = c_stringNew(base, from->packet_data);
#endif
    to->package_check_sum = (c_longlong)from->package_check_sum;
    return result;
}

void
__opensplice_dds_comm_MSFPPacket__copyOut(
    void *_from,
    void *_to)
{
    struct _opensplice_dds_comm_MSFPPacket *from = (struct _opensplice_dds_comm_MSFPPacket *)_from;
    struct ::opensplice_dds_comm::MSFPPacket *to = (struct ::opensplice_dds_comm::MSFPPacket *)_to;
    to->packet_source = (::DDS::Long)from->packet_source;
    to->packet_version = (::DDS::UShort)from->packet_version;
    to->packet_type = (::DDS::UShort)from->packet_type;
    to->packet_data = DDS::string_dup(from->packet_data ? from->packet_data : "");
    to->package_check_sum = (::DDS::LongLong)from->package_check_sum;
}

