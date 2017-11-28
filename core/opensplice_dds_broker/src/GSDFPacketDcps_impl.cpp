#include "opensplice_dds_broker/GSDFPacketDcps_impl.h"
#include "gapi.h"
#include "gapi_loanRegistry.h"
#include "opensplice_dds_broker/GSDFPacketSplDcps.h"
#include "ccpp_DataReader_impl.h"
#include "ccpp_DataReaderView_impl.h"


extern c_bool
__opensplice_dds_broker_GSDFPacket__copyIn(
    c_base base,
    struct opensplice_dds_broker::GSDFPacket *from,
    struct _opensplice_dds_broker_GSDFPacket *to);

extern void
__opensplice_dds_broker_GSDFPacket__copyOut(
    void *_from,
    void *_to);

// DDS opensplice_dds_broker::GSDFPacket TypeSupportFactory Object Body

::DDS::DataWriter_ptr
opensplice_dds_broker::GSDFPacketTypeSupportFactory::create_datawriter (gapi_dataWriter handle)
{
    return new opensplice_dds_broker::GSDFPacketDataWriter_impl(handle);
}

::DDS::DataReader_ptr
opensplice_dds_broker::GSDFPacketTypeSupportFactory::create_datareader (gapi_dataReader handle)
{
    return new opensplice_dds_broker::GSDFPacketDataReader_impl (handle);
}


::DDS::DataReaderView_ptr
opensplice_dds_broker::GSDFPacketTypeSupportFactory::create_view (gapi_dataReaderView handle)
{
    return new opensplice_dds_broker::GSDFPacketDataReaderView_impl (handle);
}

// DDS opensplice_dds_broker::GSDFPacket TypeSupport Object Body

opensplice_dds_broker::GSDFPacketTypeSupport::GSDFPacketTypeSupport(void) :
    TypeSupport_impl(
        __opensplice_dds_broker_GSDFPacket__name(),
        __opensplice_dds_broker_GSDFPacket__keys(),
        opensplice_dds_broker::GSDFPacketTypeSupport::metaDescriptor,
        (gapi_copyIn) __opensplice_dds_broker_GSDFPacket__copyIn,
        (gapi_copyOut) __opensplice_dds_broker_GSDFPacket__copyOut,
        (gapi_readerCopy) ::DDS::ccpp_DataReaderCopy<opensplice_dds_broker::GSDFPacketSeq, opensplice_dds_broker::GSDFPacket>,
        new  opensplice_dds_broker::GSDFPacketTypeSupportFactory(),
        opensplice_dds_broker::GSDFPacketTypeSupport::metaDescriptorArrLength)
{
    // Parent constructor takes care of everything.
}

opensplice_dds_broker::GSDFPacketTypeSupport::~GSDFPacketTypeSupport(void)
{
    // Parent destructor takes care of everything.
}

::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketTypeSupport::register_type(
    ::DDS::DomainParticipant_ptr domain,
    const char * type_name) THROW_ORB_EXCEPTIONS
{
    return TypeSupport_impl::register_type(domain, type_name);
}

char *
opensplice_dds_broker::GSDFPacketTypeSupport::get_type_name() THROW_ORB_EXCEPTIONS
{
    return TypeSupport_impl::get_type_name();
}

// DDS opensplice_dds_broker::GSDFPacket DataWriter_impl Object Body

opensplice_dds_broker::GSDFPacketDataWriter_impl::GSDFPacketDataWriter_impl (
    gapi_dataWriter handle
) : ::DDS::DataWriter_impl(handle)
{
    // Parent constructor takes care of everything.
}

opensplice_dds_broker::GSDFPacketDataWriter_impl::~GSDFPacketDataWriter_impl(void)
{
    // Parent destructor takes care of everything.
}

::DDS::InstanceHandle_t
opensplice_dds_broker::GSDFPacketDataWriter_impl::register_instance(
    const opensplice_dds_broker::GSDFPacket & instance_data) THROW_ORB_EXCEPTIONS
{
    return DataWriter_impl::register_instance(&instance_data);
}

::DDS::InstanceHandle_t
opensplice_dds_broker::GSDFPacketDataWriter_impl::register_instance_w_timestamp(
    const GSDFPacket & instance_data,
    const ::DDS::Time_t & source_timestamp) THROW_ORB_EXCEPTIONS
{
    return DataWriter_impl::register_instance_w_timestamp(&instance_data, source_timestamp);
}

::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataWriter_impl::unregister_instance(
    const opensplice_dds_broker::GSDFPacket & instance_data,
    ::DDS::InstanceHandle_t handle) THROW_ORB_EXCEPTIONS
{
    return DataWriter_impl::unregister_instance(&instance_data, handle);
}

::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataWriter_impl::unregister_instance_w_timestamp(
    const GSDFPacket & instance_data,
    ::DDS::InstanceHandle_t handle,
    const ::DDS::Time_t & source_timestamp) THROW_ORB_EXCEPTIONS
{
    return DataWriter_impl::unregister_instance_w_timestamp(&instance_data, handle, source_timestamp);
}

::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataWriter_impl::write(
    const opensplice_dds_broker::GSDFPacket & instance_data,
    ::DDS::InstanceHandle_t handle) THROW_ORB_EXCEPTIONS
{
    return DataWriter_impl::write(&instance_data, handle);
}

::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataWriter_impl::write_w_timestamp(
    const GSDFPacket & instance_data,
    ::DDS::InstanceHandle_t handle,
    const ::DDS::Time_t & source_timestamp) THROW_ORB_EXCEPTIONS
{
    return DataWriter_impl::write_w_timestamp(&instance_data, handle, source_timestamp);
}

::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataWriter_impl::dispose(
    const opensplice_dds_broker::GSDFPacket & instance_data,
    ::DDS::InstanceHandle_t handle) THROW_ORB_EXCEPTIONS
{
    return DataWriter_impl::dispose(&instance_data, handle);
}

::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataWriter_impl::dispose_w_timestamp(
    const GSDFPacket & instance_data,
    ::DDS::InstanceHandle_t handle,
    const ::DDS::Time_t & source_timestamp) THROW_ORB_EXCEPTIONS
{
    return DataWriter_impl::dispose_w_timestamp(&instance_data, handle, source_timestamp);
}

::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataWriter_impl::writedispose(
    const opensplice_dds_broker::GSDFPacket & instance_data,
    ::DDS::InstanceHandle_t handle) THROW_ORB_EXCEPTIONS
{
    return DataWriter_impl::writedispose(&instance_data, handle);
}

::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataWriter_impl::writedispose_w_timestamp(
    const GSDFPacket & instance_data,
    ::DDS::InstanceHandle_t handle,
    const ::DDS::Time_t & source_timestamp) THROW_ORB_EXCEPTIONS
{
    return DataWriter_impl::writedispose_w_timestamp(&instance_data, handle, source_timestamp);
}

::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataWriter_impl::get_key_value(
    GSDFPacket & key_holder,
    ::DDS::InstanceHandle_t handle) THROW_ORB_EXCEPTIONS
{
    return DataWriter_impl::get_key_value(&key_holder, handle);
}

::DDS::InstanceHandle_t
opensplice_dds_broker::GSDFPacketDataWriter_impl::lookup_instance(
    const opensplice_dds_broker::GSDFPacket & instance_data) THROW_ORB_EXCEPTIONS
{
    return DataWriter_impl::lookup_instance(&instance_data);
}

// DDS opensplice_dds_broker::GSDFPacket DataReader_impl Object Body

opensplice_dds_broker::GSDFPacketDataReader_impl::GSDFPacketDataReader_impl (
    gapi_dataReader handle
) : ::DDS::DataReader_impl(handle, ::DDS::ccpp_DataReaderParallelDemarshallingMain<opensplice_dds_broker::GSDFPacketSeq>)
{
    // Parent constructor takes care of everything.
}

opensplice_dds_broker::GSDFPacketDataReader_impl::~GSDFPacketDataReader_impl(void)
{
    // Parent destructor takes care of everything.
}


::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataReader_impl::read(
    opensplice_dds_broker::GSDFPacketSeq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::SampleStateMask sample_states,
    ::DDS::ViewStateMask view_states,
    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DataReader_impl::read(&received_data, info_seq, max_samples, sample_states, view_states, instance_states);
    }
    return status;
}

::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataReader_impl::take(
    opensplice_dds_broker::GSDFPacketSeq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::SampleStateMask sample_states,
    ::DDS::ViewStateMask view_states,
    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DataReader_impl::take(&received_data, info_seq, max_samples, sample_states, view_states, instance_states);
    }
    return status;
}

::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataReader_impl::read_w_condition(
    opensplice_dds_broker::GSDFPacketSeq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::ReadCondition_ptr a_condition) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DataReader_impl::read_w_condition(&received_data, info_seq, max_samples, a_condition);
    }
    return status;
}

::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataReader_impl::take_w_condition(
    opensplice_dds_broker::GSDFPacketSeq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::ReadCondition_ptr a_condition) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DataReader_impl::take_w_condition(&received_data, info_seq, max_samples, a_condition);
    }
    return status;
}


::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataReader_impl::read_next_sample(
    opensplice_dds_broker::GSDFPacket & received_data,
    ::DDS::SampleInfo & sample_info) THROW_ORB_EXCEPTIONS
{
    return DataReader_impl::read_next_sample(&received_data, sample_info);
}


::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataReader_impl::take_next_sample(
    opensplice_dds_broker::GSDFPacket & received_data,
    ::DDS::SampleInfo & sample_info) THROW_ORB_EXCEPTIONS
{
    return DataReader_impl::take_next_sample(&received_data, sample_info);
}


::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataReader_impl::read_instance(
    opensplice_dds_broker::GSDFPacketSeq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::InstanceHandle_t a_handle,
    ::DDS::SampleStateMask sample_states,
    ::DDS::ViewStateMask view_states,
    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DataReader_impl::read_instance(&received_data, info_seq, max_samples, a_handle, sample_states, view_states, instance_states);
    }
    return status;
}

::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataReader_impl::take_instance(
    opensplice_dds_broker::GSDFPacketSeq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::InstanceHandle_t a_handle,
    ::DDS::SampleStateMask sample_states,
    ::DDS::ViewStateMask view_states,
    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DataReader_impl::take_instance(&received_data, info_seq, max_samples, a_handle, sample_states, view_states, instance_states);
    }
    return status;
}

::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataReader_impl::read_next_instance(
    opensplice_dds_broker::GSDFPacketSeq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::InstanceHandle_t a_handle,
    ::DDS::SampleStateMask sample_states,
    ::DDS::ViewStateMask view_states,
    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DataReader_impl::read_next_instance(&received_data, info_seq, max_samples, a_handle, sample_states, view_states, instance_states);
    }
    return status;
}

::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataReader_impl::take_next_instance(
    opensplice_dds_broker::GSDFPacketSeq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::InstanceHandle_t a_handle,
    ::DDS::SampleStateMask sample_states,
    ::DDS::ViewStateMask view_states,
    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DataReader_impl::take_next_instance(&received_data, info_seq, max_samples, a_handle, sample_states, view_states, instance_states);
    }
    return status;
}


::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataReader_impl::read_next_instance_w_condition(
    opensplice_dds_broker::GSDFPacketSeq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::InstanceHandle_t a_handle,
    ::DDS::ReadCondition_ptr a_condition) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DataReader_impl::read_next_instance_w_condition(&received_data, info_seq, max_samples, a_handle, a_condition);
    }
    return status;
}


::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataReader_impl::take_next_instance_w_condition(
    opensplice_dds_broker::GSDFPacketSeq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::InstanceHandle_t a_handle,
    ::DDS::ReadCondition_ptr a_condition) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DataReader_impl::take_next_instance_w_condition(&received_data, info_seq, max_samples, a_handle, a_condition);
    }
    return status;
}


::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataReader_impl::return_loan(
    opensplice_dds_broker::GSDFPacketSeq & received_data,
    ::DDS::SampleInfoSeq & info_seq) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status = ::DDS::RETCODE_OK;

    if ( received_data.length() > 0 ) {
        if (received_data.length() == info_seq.length() &&
            received_data.release() == info_seq.release() ) {
            if (!received_data.release()) {
                status = DataReader_impl::return_loan( received_data.get_buffer(),
                                                       info_seq.get_buffer() );

                if ( status == ::DDS::RETCODE_OK ) {
                    if ( !received_data.release() ) {
                        opensplice_dds_broker::GSDFPacketSeq::freebuf( received_data.get_buffer(false) );
                        received_data.replace(0, 0, NULL, false);
                        ::DDS::SampleInfoSeq::freebuf( info_seq.get_buffer(false) );
                        info_seq.replace(0, 0, NULL, false);
                    }
                } else if ( status == ::DDS::RETCODE_NO_DATA ) {
                    if ( received_data.release() ) {
                        status = ::DDS::RETCODE_OK;
                    } else {
                        status = ::DDS::RETCODE_PRECONDITION_NOT_MET;
                    }
                }
            }
        } else {
            status = ::DDS::RETCODE_PRECONDITION_NOT_MET;
        }
    }
    return status;
}


::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataReader_impl::get_key_value(
    opensplice_dds_broker::GSDFPacket & key_holder,
    ::DDS::InstanceHandle_t handle) THROW_ORB_EXCEPTIONS
{
    return DataReader_impl::get_key_value(&key_holder, handle);
}

::DDS::InstanceHandle_t
opensplice_dds_broker::GSDFPacketDataReader_impl::lookup_instance(
    const opensplice_dds_broker::GSDFPacket & instance) THROW_ORB_EXCEPTIONS
{
    return DataReader_impl::lookup_instance(&instance);
}

::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataReader_impl::check_preconditions(
    opensplice_dds_broker::GSDFPacketSeq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples)
{
    ::DDS::ReturnCode_t status = ::DDS::RETCODE_PRECONDITION_NOT_MET;

    if ( received_data.length() == info_seq.length() &&
         received_data.maximum() == info_seq.maximum() &&
         received_data.release() == info_seq.release() ) {
        if ( received_data.maximum() == 0 || received_data.release() ) {
            if (received_data.maximum() == 0 ||
                max_samples <= static_cast<DDS::Long>(received_data.maximum()) ||
                max_samples == ::DDS::LENGTH_UNLIMITED ) {
                status = ::DDS::RETCODE_OK;
            }
        }
    }
    return status;
}


// DDS opensplice_dds_broker::GSDFPacket DataReaderView_impl Object Body

opensplice_dds_broker::GSDFPacketDataReaderView_impl::GSDFPacketDataReaderView_impl (
    gapi_dataReaderView handle
) : ::DDS::DataReaderView_impl(handle)
{
    // Parent constructor takes care of everything.
}

opensplice_dds_broker::GSDFPacketDataReaderView_impl::~GSDFPacketDataReaderView_impl(void)
{
    // Parent destructor takes care of everything.
}


::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataReaderView_impl::read(
    opensplice_dds_broker::GSDFPacketSeq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::SampleStateMask sample_states,
    ::DDS::ViewStateMask view_states,
    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = opensplice_dds_broker::GSDFPacketDataReader_impl::check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DataReaderView_impl::read(&received_data, info_seq, max_samples, sample_states, view_states, instance_states);
    }
    return status;
}

::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataReaderView_impl::take(
    opensplice_dds_broker::GSDFPacketSeq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::SampleStateMask sample_states,
    ::DDS::ViewStateMask view_states,
    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = opensplice_dds_broker::GSDFPacketDataReader_impl::check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DataReaderView_impl::take(&received_data, info_seq, max_samples, sample_states, view_states, instance_states);
    }
    return status;
}

::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataReaderView_impl::read_w_condition(
    opensplice_dds_broker::GSDFPacketSeq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::ReadCondition_ptr a_condition) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = opensplice_dds_broker::GSDFPacketDataReader_impl::check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DataReaderView_impl::read_w_condition(&received_data, info_seq, max_samples, a_condition);
    }
    return status;
}

::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataReaderView_impl::take_w_condition(
    opensplice_dds_broker::GSDFPacketSeq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::ReadCondition_ptr a_condition) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = opensplice_dds_broker::GSDFPacketDataReader_impl::check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DataReaderView_impl::take_w_condition(&received_data, info_seq, max_samples, a_condition);
    }
    return status;
}


::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataReaderView_impl::read_next_sample(
    opensplice_dds_broker::GSDFPacket & received_data,
    ::DDS::SampleInfo & sample_info) THROW_ORB_EXCEPTIONS
{
    return DataReaderView_impl::read_next_sample(&received_data, sample_info);
}


::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataReaderView_impl::take_next_sample(
    opensplice_dds_broker::GSDFPacket & received_data,
    ::DDS::SampleInfo & sample_info) THROW_ORB_EXCEPTIONS
{
    return DataReaderView_impl::take_next_sample(&received_data, sample_info);
}


::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataReaderView_impl::read_instance(
    opensplice_dds_broker::GSDFPacketSeq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::InstanceHandle_t a_handle,
    ::DDS::SampleStateMask sample_states,
    ::DDS::ViewStateMask view_states,
    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = opensplice_dds_broker::GSDFPacketDataReader_impl::check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DataReaderView_impl::read_instance(&received_data, info_seq, max_samples, a_handle, sample_states, view_states, instance_states);
    }
    return status;
}

::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataReaderView_impl::take_instance(
    opensplice_dds_broker::GSDFPacketSeq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::InstanceHandle_t a_handle,
    ::DDS::SampleStateMask sample_states,
    ::DDS::ViewStateMask view_states,
    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = opensplice_dds_broker::GSDFPacketDataReader_impl::check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DataReaderView_impl::take_instance(&received_data, info_seq, max_samples, a_handle, sample_states, view_states, instance_states);
    }
    return status;
}

::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataReaderView_impl::read_next_instance(
    opensplice_dds_broker::GSDFPacketSeq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::InstanceHandle_t a_handle,
    ::DDS::SampleStateMask sample_states,
    ::DDS::ViewStateMask view_states,
    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = opensplice_dds_broker::GSDFPacketDataReader_impl::check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DataReaderView_impl::read_next_instance(&received_data, info_seq, max_samples, a_handle, sample_states, view_states, instance_states);
    }
    return status;
}

::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataReaderView_impl::take_next_instance(
    opensplice_dds_broker::GSDFPacketSeq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::InstanceHandle_t a_handle,
    ::DDS::SampleStateMask sample_states,
    ::DDS::ViewStateMask view_states,
    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = opensplice_dds_broker::GSDFPacketDataReader_impl::check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DataReaderView_impl::take_next_instance(&received_data, info_seq, max_samples, a_handle, sample_states, view_states, instance_states);
    }
    return status;
}


::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataReaderView_impl::read_next_instance_w_condition(
    opensplice_dds_broker::GSDFPacketSeq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::InstanceHandle_t a_handle,
    ::DDS::ReadCondition_ptr a_condition) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = opensplice_dds_broker::GSDFPacketDataReader_impl::check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DataReaderView_impl::read_next_instance_w_condition(&received_data, info_seq, max_samples, a_handle, a_condition);
    }
    return status;
}


::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataReaderView_impl::take_next_instance_w_condition(
    opensplice_dds_broker::GSDFPacketSeq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::InstanceHandle_t a_handle,
    ::DDS::ReadCondition_ptr a_condition) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = opensplice_dds_broker::GSDFPacketDataReader_impl::check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DataReaderView_impl::take_next_instance_w_condition(&received_data, info_seq, max_samples, a_handle, a_condition);
    }
    return status;
}


::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataReaderView_impl::return_loan(
    opensplice_dds_broker::GSDFPacketSeq & received_data,
    ::DDS::SampleInfoSeq & info_seq) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status = ::DDS::RETCODE_OK;

    if ( received_data.length() > 0 ) {
        if (received_data.length() == info_seq.length() &&
            received_data.release() == info_seq.release() ) {
            if (!received_data.release()) {
                status = DataReaderView_impl::return_loan( received_data.get_buffer(),
                                                       info_seq.get_buffer() );

                if ( status == ::DDS::RETCODE_OK ) {
                    if ( !received_data.release() ) {
                        opensplice_dds_broker::GSDFPacketSeq::freebuf( received_data.get_buffer(false) );
                        received_data.replace(0, 0, NULL, false);
                        ::DDS::SampleInfoSeq::freebuf( info_seq.get_buffer(false) );
                        info_seq.replace(0, 0, NULL, false);
                    }
                } else if ( status == ::DDS::RETCODE_NO_DATA ) {
                    if ( received_data.release() ) {
                        status = ::DDS::RETCODE_OK;
                    } else {
                        status = ::DDS::RETCODE_PRECONDITION_NOT_MET;
                    }
                }
            }
        } else {
            status = ::DDS::RETCODE_PRECONDITION_NOT_MET;
        }
    }
    return status;
}


::DDS::ReturnCode_t
opensplice_dds_broker::GSDFPacketDataReaderView_impl::get_key_value(
    opensplice_dds_broker::GSDFPacket & key_holder,
    ::DDS::InstanceHandle_t handle) THROW_ORB_EXCEPTIONS
{
    return DataReaderView_impl::get_key_value(&key_holder, handle);
}

::DDS::InstanceHandle_t
opensplice_dds_broker::GSDFPacketDataReaderView_impl::lookup_instance(
    const opensplice_dds_broker::GSDFPacket & instance) THROW_ORB_EXCEPTIONS
{
    return DataReaderView_impl::lookup_instance(&instance);
}



const char * ::opensplice_dds_broker::GSDFPacketTypeSupport::metaDescriptor[] = {"<MetaData version=\"1.0.0\"><Module name=\"opensplice_dds_broker\"><TypeDef name=\"CharSeq\"><Sequence>",
"<Char/></Sequence></TypeDef><Struct name=\"GSDFPacket\"><Member name=\"data\"><Type name=\"CharSeq\"/>",
"</Member></Struct></Module></MetaData>"};
const ::DDS::ULong (::opensplice_dds_broker::GSDFPacketTypeSupport::metaDescriptorArrLength) = 3;
