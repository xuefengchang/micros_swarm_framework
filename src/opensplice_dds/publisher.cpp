
#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include "ccpp_dds_dcps.h"
#include "opensplice_dds/check_status.h"
#include "opensplice_dds/ccpp_MSFPPacket.h"
#include "opensplice_dds/example_main.h"
#include "opensplice_dds/publisher.h"

using namespace DDS;

namespace micros_swarm_framework{
    
    Publisher::Publisher(std::string topic_name)
    {
        domain = 0;
        
        topic_name_ = topic_name.data();
        
        //robot_id_ = robot_id;
        
        MSFPPacketTypeName = NULL;  
        
        //Create a DomainParticipantFactory and a DomainParticipant, using Default QoS settings
        dpf = DomainParticipantFactory::get_instance ();
        checkHandle(dpf.in(), "DDS::DomainParticipantFactory::get_instance");
        participant = dpf->create_participant(domain, PARTICIPANT_QOS_DEFAULT, NULL, STATUS_MASK_NONE);
        checkHandle(participant.in(), "DDS::DomainParticipantFactory::create_participant");

        //Register the required datatype
        MSFPPacketTS = new MSFPPacketTypeSupport();
        checkHandle(MSFPPacketTS.in(), "new MSFPPacketTypeSupport");
        MSFPPacketTypeName = MSFPPacketTS->get_type_name();
        status = MSFPPacketTS->register_type(
            participant.in(),
            MSFPPacketTypeName);
        checkStatus(status, "micros_swarm_framework::MSFPPacketTypeSupport::register_type");

        //Set the ReliabilityQosPolicy to BEST_EFFORT_RELIABILITY
        status = participant->get_default_topic_qos(topic_qos);
        checkStatus(status, "DDS::DomainParticipant::get_default_topic_qos");
        //topic_qos.reliability.kind = DDS::BEST_EFFORT_RELIABILITY_QOS;
        topic_qos.reliability.kind = RELIABLE_RELIABILITY_QOS;
        //topic_qos.durability_service.history_kind=KEEP_LAST_HISTORY_QOS;

        //Make the tailored QoS the new default
        status = participant->set_default_topic_qos(topic_qos);
        checkStatus(status, "DDS::DomainParticipant::set_default_topic_qos");

        //Use the changed policy when defining the MSFPPacket topic
        MSFPPacketTopic = participant->create_topic(
            topic_name_,
            MSFPPacketTypeName,
            topic_qos,
            NULL,
            STATUS_MASK_NONE);
        checkHandle(MSFPPacketTopic.in(), "DDS::DomainParticipant::create_topic (MSFPPacket)");

        //Adapt the default PublisherQos to write into the "micros_swarm_framework_default_partion" Partition
        status = participant->get_default_publisher_qos (pub_qos);
        checkStatus(status, "DDS::DomainParticipant::get_default_publisher_qos");
        pub_qos.partition.name.length(1);
        pub_qos.partition.name[0] = "micros_swarm_framework_partion";

        //Create a Publisher for the application
        publisher_ = participant->create_publisher(pub_qos, NULL, STATUS_MASK_NONE);
        checkHandle(publisher_.in(), "DDS::DomainParticipant::create_publisher");
        
        status = publisher_->get_default_datawriter_qos(dw_qos);
        dw_qos.destination_order.kind = BY_SOURCE_TIMESTAMP_DESTINATIONORDER_QOS;
        dw_qos.history.kind = KEEP_ALL_HISTORY_QOS;
        dw_qos.durability.kind = TRANSIENT_LOCAL_DURABILITY_QOS;

        //Create a DataWriter for the MSFPPacket Topic (using the appropriate QoS)
        parentWriter = publisher_->create_datawriter(
            MSFPPacketTopic.in(),
            //DATAWRITER_QOS_USE_TOPIC_QOS,
            dw_qos,
            NULL,
            STATUS_MASK_NONE);
        checkHandle(parentWriter, "DDS::Publisher::create_datawriter (MSFPPacket)");

        //Narrow the abstract parent into its typed representative
        MSFPPacketDW = MSFPPacketDataWriter::_narrow(parentWriter);
        checkHandle(MSFPPacketDW.in(), "micros_swarm_framework::MSFPPacketDataWriter::_narrow");
        
        //packet_=new micros_swarm_framework::MSFPPacket();
        //packet_->packet_source = robot_id_;
        //userHandle = MSFPPacketDW->register_instance(*packet_);
    }
    
    void Publisher::publish(const MSFPPacket& packet)
    {
        //packet_->packet_source = robot_id_;
        //packet_->packet_version=1;
        //packet_->packet_type=packet.packet_type;
        //packet_->packet_data=packet.packet_data;
        //packet_->package_check_sum=111;
        //status = MSFPPacketDW->write(*packet_, userHandle);
        //checkStatus(status, "micros_swarm_framework::MSFPPacketDataWriter::write");
        //userHandle = MSFPPacketDW->register_instance(packet);
        status = MSFPPacketDW->write(packet, DDS::HANDLE_NIL);
        checkStatus(status, "micros_swarm_framework::MSFPPacketDataWriter::write");
    }
    

    /*
    void Publisher::publish(MSFPPacket* packet)
    {
        packet_ = packet;
        status = MSFPPacketDW->write(*packet_, userHandle);
        checkStatus(status, "micros_swarm_framework::MSFPPacketDataWriter::write");
    }
    */
    
    Publisher::~Publisher()
    {
        //status = MSFPPacketDW->dispose(*packet_, userHandle);
        //checkStatus(status, "micros_swarm_framework::MSFPPacketDataWriter::dispose");
        //status = MSFPPacketDW->unregister_instance(*packet_, userHandle);
        //checkStatus(status, "micros_swarm_framework::MSFPPacketDataWriter::unregister_instance");

        //Release the data-samples
        //delete packet_;     

        //Remove the DataWriters 
        status = publisher_->delete_datawriter(MSFPPacketDW.in() );
        checkStatus(status, "DDS::Publisher::delete_datawriter (MSFPPacketDW)");

        //Remove the Publisher
        status = participant->delete_publisher(publisher_.in() );
        checkStatus(status, "DDS::DomainParticipant::delete_publisher");

        status = participant->delete_topic( MSFPPacketTopic.in() );
        checkStatus(status, "DDS::DomainParticipant::delete_topic (MSFPPacketTopic)");

        //Remove the type-names
        string_free(MSFPPacketTypeName);

        //Remove the DomainParticipant
        status = dpf->delete_participant( participant.in() );
        checkStatus(status, "DDS::DomainParticipantFactory::delete_participant");

        cout << "Completed ter example" << endl;
    }
};
