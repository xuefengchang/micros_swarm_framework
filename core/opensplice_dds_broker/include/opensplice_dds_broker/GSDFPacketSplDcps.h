#ifndef GSDFPACKETSPLTYPES_H
#define GSDFPACKETSPLTYPES_H

#include "ccpp_GSDFPacket.h"

#include <c_base.h>
#include <c_misc.h>
#include <c_sync.h>
#include <c_collection.h>
#include <c_field.h>

extern c_metaObject __GSDFPacket_opensplice_dds_broker__load (c_base base);

extern c_metaObject __opensplice_dds_broker_CharSeq__load (c_base base);
typedef c_sequence _opensplice_dds_broker_CharSeq;

extern c_metaObject __opensplice_dds_broker_GSDFPacket__load (c_base base);
extern const char * __opensplice_dds_broker_GSDFPacket__keys (void);
extern const char * __opensplice_dds_broker_GSDFPacket__name (void);
struct _opensplice_dds_broker_GSDFPacket ;
extern  c_bool __opensplice_dds_broker_GSDFPacket__copyIn(c_base base, struct opensplice_dds_broker::GSDFPacket *from, struct _opensplice_dds_broker_GSDFPacket *to);
extern  void __opensplice_dds_broker_GSDFPacket__copyOut(void *_from, void *_to);
struct _opensplice_dds_broker_GSDFPacket {
    _opensplice_dds_broker_CharSeq data;
};

#endif
