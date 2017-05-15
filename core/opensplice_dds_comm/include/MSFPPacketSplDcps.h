#ifndef MSFPPACKETSPLTYPES_H
#define MSFPPACKETSPLTYPES_H

#include "ccpp_MSFPPacket.h"

#include <c_base.h>
#include <c_misc.h>
#include <c_sync.h>
#include <c_collection.h>
#include <c_field.h>

extern c_metaObject __MSFPPacket_micros_swarm_framework__load (c_base base);

extern c_metaObject __micros_swarm_framework_MSFPPacket__load (c_base base);
extern const char * __micros_swarm_framework_MSFPPacket__keys (void);
extern const char * __micros_swarm_framework_MSFPPacket__name (void);
struct _micros_swarm_framework_MSFPPacket ;
extern  c_bool __micros_swarm_framework_MSFPPacket__copyIn(c_base base, struct micros_swarm_framework::MSFPPacket *from, struct _micros_swarm_framework_MSFPPacket *to);
extern  void __micros_swarm_framework_MSFPPacket__copyOut(void *_from, void *_to);
struct _micros_swarm_framework_MSFPPacket {
    c_long packet_source;
    c_ushort packet_version;
    c_ushort packet_type;
    c_string packet_data;
    c_longlong package_check_sum;
};

#endif
