#ifndef MSFPPACKETSPLTYPES_H
#define MSFPPACKETSPLTYPES_H

#include "ccpp_MSFPPacket.h"

#include <c_base.h>
#include <c_misc.h>
#include <c_sync.h>
#include <c_collection.h>
#include <c_field.h>

extern c_metaObject __MSFPPacket_opensplice_dds_comm__load (c_base base);

extern c_metaObject __opensplice_dds_comm_MSFPPacket__load (c_base base);
extern const char * __opensplice_dds_comm_MSFPPacket__keys (void);
extern const char * __opensplice_dds_comm_MSFPPacket__name (void);
struct _opensplice_dds_comm_MSFPPacket ;
extern  c_bool __opensplice_dds_comm_MSFPPacket__copyIn(c_base base, struct opensplice_dds_comm::MSFPPacket *from, struct _opensplice_dds_comm_MSFPPacket *to);
extern  void __opensplice_dds_comm_MSFPPacket__copyOut(void *_from, void *_to);
struct _opensplice_dds_comm_MSFPPacket {
    c_long packet_source;
    c_ushort packet_version;
    c_ushort packet_type;
    c_string packet_data;
    c_longlong package_check_sum;
};

#endif
