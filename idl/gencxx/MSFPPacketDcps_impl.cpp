#include "MSFPPacketDcps_impl.h"
#include "gapi.h"
#include "gapi_loanRegistry.h"
#include "MSFPPacketSplDcps.h"
#include "ccpp_DataReader_impl.h"
#include "ccpp_DataReaderView_impl.h"


extern c_bool
__micros_swarm_framework_MSFPPacket__copyIn(
    c_base base,
    struct micros_swarm_framework::MSFPPacket *from,
    struct _micros_swarm_framework_MSFPPacket *to);

extern void
__micros_swarm_framework_MSFPPacket__copyOut(
    void *_from,
    void *_to);

// DDS micros_swarm_framework::MSFPPacket TypeSupportFactory Object Body

::DDS::DataWriter_ptr
micros_swarm_framework::MSFPPacketTypeSupportFactory::create_datawriter (gapi_dataWriter handle)
{
    return new micros_swarm_framework::MSFPPacketDataWriter_impl(handle);
}

::DDS::DataReader_ptr
micros_swarm_framework::MSFPPacketTypeSupportFactory::create_datareader (gapi_dataReader handle)
{
    return new micros_swarm_framework::MSFPPacketDataReader_impl (handle);
}


::DDS::DataReaderView_ptr
micros_swarm_framework::MSFPPacketTypeSupportFactory::create_view (gapi_dataReaderView handle)
{
    return new micros_swarm_framework::MSFPPacketDataReaderView_impl (handle);
}

// DDS micros_swarm_framework::MSFPPacket TypeSupport Object Body

micros_swarm_framework::MSFPPacketTypeSupport::MSFPPacketTypeSupport(void) :
    TypeSupport_impl(
        __micros_swarm_framework_MSFPPacket__name(),
        __micros_swarm_framework_MSFPPacket__keys(),
        micros_swarm_framework::MSFPPacketTypeSupport::metaDescriptor,
        (gapi_copyIn) __micros_swarm_framework_MSFPPacket__copyIn,
        (gapi_copyOut) __micros_swarm_framework_MSFPPacket__copyOut,
        (gapi_readerCopy) ::DDS::ccpp_DataReaderCopy<micros_swarm_framework::MSFPPacketSeq, micros_swarm_framework::MSFPPacket>,
        new  micros_swarm_framework::MSFPPacketTypeSupportFactory(),
        micros_swarm_framework::MSFPPacketTypeSupport::metaDescriptorArrLength)
{
    // Parent constructor takes care of everything.
}

micros_swarm_framework::MSFPPacketTypeSupport::~MSFPPacketTypeSupport(void)
{
    // Parent destructor takes care of everything.
}

::DDS::ReturnCode_t
micros_swarm_framework::MSFPPacketTypeSupport::register_type(
    ::DDS::DomainParticipant_ptr domain,
    const char * type_name) THROW_ORB_EXCEPTIONS
{
    return TypeSupport_impl::register_type(domain, type_name);
}

char *
micros_swarm_framework::MSFPPacketTypeSupport::get_type_name() THROW_ORB_EXCEPTIONS
{
    return TypeSupport_impl::get_type_name();
}

// DDS micros_swarm_framework::MSFPPacket DataWriter_impl Object Body

micros_swarm_framework::MSFPPacketDataWriter_impl::MSFPPacketDataWriter_impl (
    gapi_dataWriter handle
) : ::DDS::DataWriter_impl(handle)
{
    // Parent constructor takes care of everything.
}

micros_swarm_framework::MSFPPacketDataWriter_impl::~MSFPPacketDataWriter_impl(void)
{
    // Parent destructor takes care of everything.
}

::DDS::InstanceHandle_t
micros_swarm_framework::MSFPPacketDataWriter_impl::register_instance(
    const micros_swarm_framework::MSFPPacket & instance_data) THROW_ORB_EXCEPTIONS
{
    return DataWriter_impl::register_instance(&instance_data);
}

::DDS::InstanceHandle_t
micros_swarm_framework::MSFPPacketDataWriter_impl::register_instance_w_timestamp(
    const MSFPPacket & instance_data,
    const ::DDS::Time_t & source_timestamp) THROW_ORB_EXCEPTIONS
{
    return DataWriter_impl::register_instance_w_timestamp(&instance_data, source_timestamp);
}

::DDS::ReturnCode_t
micros_swarm_framework::MSFPPacketDataWriter_impl::unregister_instance(
    const micros_swarm_framework::MSFPPacket & instance_data,
    ::DDS::InstanceHandle_t handle) THROW_ORB_EXCEPTIONS
{
    return DataWriter_impl::unregister_instance(&instance_data, handle);
}

::DDS::ReturnCode_t
micros_swarm_framework::MSFPPacketDataWriter_impl::unregister_instance_w_timestamp(
    const MSFPPacket & instance_data,
    ::DDS::InstanceHandle_t handle,
    const ::DDS::Time_t & source_timestamp) THROW_ORB_EXCEPTIONS
{
    return DataWriter_impl::unregister_instance_w_timestamp(&instance_data, handle, source_timestamp);
}

::DDS::ReturnCode_t
micros_swarm_framework::MSFPPacketDataWriter_impl::write(
    const micros_swarm_framework::MSFPPacket & instance_data,
    ::DDS::InstanceHandle_t handle) THROW_ORB_EXCEPTIONS
{
    return DataWriter_impl::write(&instance_data, handle);
}

::DDS::ReturnCode_t
micros_swarm_framework::MSFPPacketDataWriter_impl::write_w_timestamp(
    const MSFPPacket & instance_data,
    ::DDS::InstanceHandle_t handle,
    const ::DDS::Time_t & source_timestamp) THROW_ORB_EXCEPTIONS
{
    return DataWriter_impl::write_w_timestamp(&instance_data, handle, source_timestamp);
}

::DDS::ReturnCode_t
micros_swarm_framework::MSFPPacketDataWriter_impl::dispose(
    const micros_swarm_framework::MSFPPacket & instance_data,
    ::DDS::InstanceHandle_t handle) THROW_ORB_EXCEPTIONS
{
    return DataWriter_impl::dispose(&instance_data, handle);
}

::DDS::ReturnCode_t
micros_swarm_framework::MSFPPacketDataWriter_impl::dispose_w_timestamp(
    const MSFPPacket & instance_data,
    ::DDS::InstanceHandle_t handle,
    const ::DDS::Time_t & source_timestamp) THROW_ORB_EXCEPTIONS
{
    return DataWriter_impl::dispose_w_timestamp(&instance_data, handle, source_timestamp);
}

::DDS::ReturnCode_t
micros_swarm_framework::MSFPPacketDataWriter_impl::writedispose(
    const micros_swarm_framework::MSFPPacket & instance_data,
    ::DDS::InstanceHandle_t handle) THROW_ORB_EXCEPTIONS
{
    return DataWriter_impl::writedispose(&instance_data, handle);
}

::DDS::ReturnCode_t
micros_swarm_framework::MSFPPacketDataWriter_impl::writedispose_w_timestamp(
    const MSFPPacket & instance_data,
    ::DDS::InstanceHandle_t handle,
    const ::DDS::Time_t & source_timestamp) THROW_ORB_EXCEPTIONS
{
    return DataWriter_impl::writedispose_w_timestamp(&instance_data, handle, source_timestamp);
}

::DDS::ReturnCode_t
micros_swarm_framework::MSFPPacketDataWriter_impl::get_key_value(
    MSFPPacket & key_holder,
    ::DDS::InstanceHandle_t handle) THROW_ORB_EXCEPTIONS
{
    return DataWriter_impl::get_key_value(&key_holder, handle);
}

::DDS::InstanceHandle_t
micros_swarm_framework::MSFPPacketDataWriter_impl::lookup_instance(
    const micros_swarm_framework::MSFPPacket & instance_data) THROW_ORB_EXCEPTIONS
{
    return DataWriter_impl::lookup_instance(&instance_data);
}

// DDS micros_swarm_framework::MSFPPacket DataReader_impl Object Body

micros_swarm_framework::MSFPPacketDataReader_impl::MSFPPacketDataReader_impl (
    gapi_dataReader handle
) : ::DDS::DataReader_impl(handle, ::DDS::ccpp_DataReaderParallelDemarshallingMain<micros_swarm_framework::MSFPPacketSeq>)
{
    // Parent constructor takes care of everything.
}

micros_swarm_framework::MSFPPacketDataReader_impl::~MSFPPacketDataReader_impl(void)
{
    // Parent destructor takes care of everything.
}


::DDS::ReturnCode_t
micros_swarm_framework::MSFPPacketDataReader_impl::read(
    micros_swarm_framework::MSFPPacketSeq & received_data,
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
micros_swarm_framework::MSFPPacketDataReader_impl::take(
    micros_swarm_framework::MSFPPacketSeq & received_data,
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
micros_swarm_framework::MSFPPacketDataReader_impl::read_w_condition(
    micros_swarm_framework::MSFPPacketSeq & received_data,
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
micros_swarm_framework::MSFPPacketDataReader_impl::take_w_condition(
    micros_swarm_framework::MSFPPacketSeq & received_data,
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
micros_swarm_framework::MSFPPacketDataReader_impl::read_next_sample(
    micros_swarm_framework::MSFPPacket & received_data,
    ::DDS::SampleInfo & sample_info) THROW_ORB_EXCEPTIONS
{
    return DataReader_impl::read_next_sample(&received_data, sample_info);
}


::DDS::ReturnCode_t
micros_swarm_framework::MSFPPacketDataReader_impl::take_next_sample(
    micros_swarm_framework::MSFPPacket & received_data,
    ::DDS::SampleInfo & sample_info) THROW_ORB_EXCEPTIONS
{
    return DataReader_impl::take_next_sample(&received_data, sample_info);
}


::DDS::ReturnCode_t
micros_swarm_framework::MSFPPacketDataReader_impl::read_instance(
    micros_swarm_framework::MSFPPacketSeq & received_data,
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
micros_swarm_framework::MSFPPacketDataReader_impl::take_instance(
    micros_swarm_framework::MSFPPacketSeq & received_data,
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
micros_swarm_framework::MSFPPacketDataReader_impl::read_next_instance(
    micros_swarm_framework::MSFPPacketSeq & received_data,
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
micros_swarm_framework::MSFPPacketDataReader_impl::take_next_instance(
    micros_swarm_framework::MSFPPacketSeq & received_data,
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
micros_swarm_framework::MSFPPacketDataReader_impl::read_next_instance_w_condition(
    micros_swarm_framework::MSFPPacketSeq & received_data,
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
micros_swarm_framework::MSFPPacketDataReader_impl::take_next_instance_w_condition(
    micros_swarm_framework::MSFPPacketSeq & received_data,
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
micros_swarm_framework::MSFPPacketDataReader_impl::return_loan(
    micros_swarm_framework::MSFPPacketSeq & received_data,
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
                        micros_swarm_framework::MSFPPacketSeq::freebuf( received_data.get_buffer(false) );
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
micros_swarm_framework::MSFPPacketDataReader_impl::get_key_value(
    micros_swarm_framework::MSFPPacket & key_holder,
    ::DDS::InstanceHandle_t handle) THROW_ORB_EXCEPTIONS
{
    return DataReader_impl::get_key_value(&key_holder, handle);
}

::DDS::InstanceHandle_t
micros_swarm_framework::MSFPPacketDataReader_impl::lookup_instance(
    const micros_swarm_framework::MSFPPacket & instance) THROW_ORB_EXCEPTIONS
{
    return DataReader_impl::lookup_instance(&instance);
}

::DDS::ReturnCode_t
micros_swarm_framework::MSFPPacketDataReader_impl::check_preconditions(
    micros_swarm_framework::MSFPPacketSeq & received_data,
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


// DDS micros_swarm_framework::MSFPPacket DataReaderView_impl Object Body

micros_swarm_framework::MSFPPacketDataReaderView_impl::MSFPPacketDataReaderView_impl (
    gapi_dataReaderView handle
) : ::DDS::DataReaderView_impl(handle)
{
    // Parent constructor takes care of everything.
}

micros_swarm_framework::MSFPPacketDataReaderView_impl::~MSFPPacketDataReaderView_impl(void)
{
    // Parent destructor takes care of everything.
}


::DDS::ReturnCode_t
micros_swarm_framework::MSFPPacketDataReaderView_impl::read(
    micros_swarm_framework::MSFPPacketSeq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::SampleStateMask sample_states,
    ::DDS::ViewStateMask view_states,
    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = micros_swarm_framework::MSFPPacketDataReader_impl::check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DataReaderView_impl::read(&received_data, info_seq, max_samples, sample_states, view_states, instance_states);
    }
    return status;
}

::DDS::ReturnCode_t
micros_swarm_framework::MSFPPacketDataReaderView_impl::take(
    micros_swarm_framework::MSFPPacketSeq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::SampleStateMask sample_states,
    ::DDS::ViewStateMask view_states,
    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = micros_swarm_framework::MSFPPacketDataReader_impl::check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DataReaderView_impl::take(&received_data, info_seq, max_samples, sample_states, view_states, instance_states);
    }
    return status;
}

::DDS::ReturnCode_t
micros_swarm_framework::MSFPPacketDataReaderView_impl::read_w_condition(
    micros_swarm_framework::MSFPPacketSeq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::ReadCondition_ptr a_condition) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = micros_swarm_framework::MSFPPacketDataReader_impl::check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DataReaderView_impl::read_w_condition(&received_data, info_seq, max_samples, a_condition);
    }
    return status;
}

::DDS::ReturnCode_t
micros_swarm_framework::MSFPPacketDataReaderView_impl::take_w_condition(
    micros_swarm_framework::MSFPPacketSeq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::ReadCondition_ptr a_condition) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = micros_swarm_framework::MSFPPacketDataReader_impl::check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DataReaderView_impl::take_w_condition(&received_data, info_seq, max_samples, a_condition);
    }
    return status;
}


::DDS::ReturnCode_t
micros_swarm_framework::MSFPPacketDataReaderView_impl::read_next_sample(
    micros_swarm_framework::MSFPPacket & received_data,
    ::DDS::SampleInfo & sample_info) THROW_ORB_EXCEPTIONS
{
    return DataReaderView_impl::read_next_sample(&received_data, sample_info);
}


::DDS::ReturnCode_t
micros_swarm_framework::MSFPPacketDataReaderView_impl::take_next_sample(
    micros_swarm_framework::MSFPPacket & received_data,
    ::DDS::SampleInfo & sample_info) THROW_ORB_EXCEPTIONS
{
    return DataReaderView_impl::take_next_sample(&received_data, sample_info);
}


::DDS::ReturnCode_t
micros_swarm_framework::MSFPPacketDataReaderView_impl::read_instance(
    micros_swarm_framework::MSFPPacketSeq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::InstanceHandle_t a_handle,
    ::DDS::SampleStateMask sample_states,
    ::DDS::ViewStateMask view_states,
    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = micros_swarm_framework::MSFPPacketDataReader_impl::check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DataReaderView_impl::read_instance(&received_data, info_seq, max_samples, a_handle, sample_states, view_states, instance_states);
    }
    return status;
}

::DDS::ReturnCode_t
micros_swarm_framework::MSFPPacketDataReaderView_impl::take_instance(
    micros_swarm_framework::MSFPPacketSeq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::InstanceHandle_t a_handle,
    ::DDS::SampleStateMask sample_states,
    ::DDS::ViewStateMask view_states,
    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = micros_swarm_framework::MSFPPacketDataReader_impl::check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DataReaderView_impl::take_instance(&received_data, info_seq, max_samples, a_handle, sample_states, view_states, instance_states);
    }
    return status;
}

::DDS::ReturnCode_t
micros_swarm_framework::MSFPPacketDataReaderView_impl::read_next_instance(
    micros_swarm_framework::MSFPPacketSeq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::InstanceHandle_t a_handle,
    ::DDS::SampleStateMask sample_states,
    ::DDS::ViewStateMask view_states,
    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = micros_swarm_framework::MSFPPacketDataReader_impl::check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DataReaderView_impl::read_next_instance(&received_data, info_seq, max_samples, a_handle, sample_states, view_states, instance_states);
    }
    return status;
}

::DDS::ReturnCode_t
micros_swarm_framework::MSFPPacketDataReaderView_impl::take_next_instance(
    micros_swarm_framework::MSFPPacketSeq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::InstanceHandle_t a_handle,
    ::DDS::SampleStateMask sample_states,
    ::DDS::ViewStateMask view_states,
    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = micros_swarm_framework::MSFPPacketDataReader_impl::check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DataReaderView_impl::take_next_instance(&received_data, info_seq, max_samples, a_handle, sample_states, view_states, instance_states);
    }
    return status;
}


::DDS::ReturnCode_t
micros_swarm_framework::MSFPPacketDataReaderView_impl::read_next_instance_w_condition(
    micros_swarm_framework::MSFPPacketSeq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::InstanceHandle_t a_handle,
    ::DDS::ReadCondition_ptr a_condition) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = micros_swarm_framework::MSFPPacketDataReader_impl::check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DataReaderView_impl::read_next_instance_w_condition(&received_data, info_seq, max_samples, a_handle, a_condition);
    }
    return status;
}


::DDS::ReturnCode_t
micros_swarm_framework::MSFPPacketDataReaderView_impl::take_next_instance_w_condition(
    micros_swarm_framework::MSFPPacketSeq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::InstanceHandle_t a_handle,
    ::DDS::ReadCondition_ptr a_condition) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = micros_swarm_framework::MSFPPacketDataReader_impl::check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DataReaderView_impl::take_next_instance_w_condition(&received_data, info_seq, max_samples, a_handle, a_condition);
    }
    return status;
}


::DDS::ReturnCode_t
micros_swarm_framework::MSFPPacketDataReaderView_impl::return_loan(
    micros_swarm_framework::MSFPPacketSeq & received_data,
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
                        micros_swarm_framework::MSFPPacketSeq::freebuf( received_data.get_buffer(false) );
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
micros_swarm_framework::MSFPPacketDataReaderView_impl::get_key_value(
    micros_swarm_framework::MSFPPacket & key_holder,
    ::DDS::InstanceHandle_t handle) THROW_ORB_EXCEPTIONS
{
    return DataReaderView_impl::get_key_value(&key_holder, handle);
}

::DDS::InstanceHandle_t
micros_swarm_framework::MSFPPacketDataReaderView_impl::lookup_instance(
    const micros_swarm_framework::MSFPPacket & instance) THROW_ORB_EXCEPTIONS
{
    return DataReaderView_impl::lookup_instance(&instance);
}



const char * ::micros_swarm_framework::MSFPPacketTypeSupport::metaDescriptor[] = {"<MetaData version=\"1.0.0\"><Module name=\"micros_swarm_framework\"><Struct name=\"MSFPPacket\"><Member name=\"packet_source\">",
"<Long/></Member><Member name=\"packet_version\"><UShort/></Member><Member name=\"packet_type\"><UShort/>",
"</Member><Member name=\"packet_data\"><String/></Member><Member name=\"package_check_sum\"><LongLong/>",
"</Member></Struct></Module></MetaData>"};
const ::DDS::ULong (::micros_swarm_framework::MSFPPacketTypeSupport::metaDescriptorArrLength) = 4;
