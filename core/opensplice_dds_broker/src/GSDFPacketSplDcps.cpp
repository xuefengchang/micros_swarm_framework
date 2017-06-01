#include "opensplice_dds_broker/GSDFPacketSplDcps.h"
#include "opensplice_dds_broker/ccpp_GSDFPacket.h"
#include "dds_type_aliases.h"

const char *
__opensplice_dds_broker_GSDFPacket__name(void)
{
    return (const char*)"opensplice_dds_broker::GSDFPacket";
}

const char *
__opensplice_dds_broker_GSDFPacket__keys(void)
{
    return (const char*)"";
}

#include <v_kernel.h>
#include <v_topic.h>
#include <os_stdlib.h>
#include <string.h>
#include <os_report.h>

c_bool
__opensplice_dds_broker_GSDFPacket__copyIn(
    c_base base,
    struct ::opensplice_dds_broker::GSDFPacket *from,
    struct _opensplice_dds_broker_GSDFPacket *to)
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
        OS_REPORT (OS_ERROR, "copyIn", 0,"Member 'opensplice_dds_broker::GSDFPacket.packet_data' of type 'c_string' is NULL.");
        result = OS_C_FALSE;
    }
#else
    to->packet_data = c_stringNew(base, from->packet_data);
#endif
    to->package_check_sum = (c_longlong)from->package_check_sum;
    return result;
}

void
__opensplice_dds_broker_GSDFPacket__copyOut(
    void *_from,
    void *_to)
{
    struct _opensplice_dds_broker_GSDFPacket *from = (struct _opensplice_dds_broker_GSDFPacket *)_from;
    struct ::opensplice_dds_broker::GSDFPacket *to = (struct ::opensplice_dds_broker::GSDFPacket *)_to;
    to->packet_source = (::DDS::Long)from->packet_source;
    to->packet_version = (::DDS::UShort)from->packet_version;
    to->packet_type = (::DDS::UShort)from->packet_type;
    to->packet_data = DDS::string_dup(from->packet_data ? from->packet_data : "");
    to->package_check_sum = (::DDS::LongLong)from->package_check_sum;
}

