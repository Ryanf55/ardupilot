#include "AP_XRCE_Client.h"

#if AP_XRCE_ENABLED

AP_HAL::UARTDriver *xrce_port;


const AP_Param::GroupInfo AP_XRCE_Client::var_info[]={
    // @Param: TYPE
    // @DisplayName: XRCE_TYPE
    // @Description: Type of XRCE protocol to use
    // @Values: 0:DDS,1:uROS(micro-ROS)
    // @User: Standard
    AP_GROUPINFO("TYPE", 1, AP_XRCE_Client, xrce_type, 0),

    // @Param: TOPIC
    // @DisplayName: XRCE_TOPIC
    // @Description: Type of XRCE topic to use
    // @Values: 0:AP_DDS_NUM,1:AP_DDS_BARO,2:AP_DDS_GPS,3:AP_DDS_COMPASS,4:AP_DDS_INS,5:AP_ROS2_Bool,6:AP_ROS2_Byte,7:AP_ROS2_8Int,8:AP_ROS2_8UInt,9:AP_ROS2_Char,10:AP_ROS2_16Int,11:AP_ROS2_16UInt,12:AP_ROS2_32Int,13:AP_ROS2_32UInt,14:AP_ROS2_64Int,15:AP_ROS2_64UInt,16:AP_ROS2_32Float,17:AP_ROS2_64Float,18:AP_ROS2_String,19:AP_ROS2_ColorRGBA,20:AP_ROS2_Header
    // @User: Standard
    AP_GROUPINFO("TOPIC", 2, AP_XRCE_Client, xrce_topic_key, 0),

    AP_GROUPEND
};

// Constructor (takes maximum number of topics as argument,by default it is 1)
AP_XRCE_Client::AP_XRCE_Client(uint32_t maxtopics)
    : max_topics(maxtopics)
{
    relativeSerialClientAddr=1;
    relativeSerialAgentAddr=0;
    connected=true;
}

bool AP_XRCE_Client::init()
{
    AP_SerialManager *serial_manager = AP_SerialManager::get_singleton();
    xrce_port = serial_manager->find_serial(AP_SerialManager::SerialProtocol_ROS2, 0);
    if (xrce_port == nullptr) {
        return false;
    }

    if (!uxr_init_serial_transport(&serial_transport,fd,relativeSerialAgentAddr,relativeSerialClientAddr)) {
        return false;
    }
    uxr_init_session(&session, &serial_transport.comm, 0xAAAABBBB);

    if (!uxr_create_session(&session)) {
        return false;
    }
    
    reliable_in=uxr_create_input_reliable_stream(&session,input_reliable_stream,BUFFER_SIZE_SERIAL,STREAM_HISTORY);
    reliable_out=uxr_create_output_reliable_stream(&session,output_reliable_stream,BUFFER_SIZE_SERIAL,STREAM_HISTORY);
    
    xrce_topic = set_topic_instance(xrce_topic_key.get());

    if(xrce_topic == nullptr) {
        return false;
    }

    if( !xrce_topic->topic_initialize(xrce_type.get())) {
        return false; 
    }

    return true;
}
bool AP_XRCE_Client::create()
{
    WITH_SEMAPHORE(csem);
    participant_id=uxr_object_id(0x01,UXR_PARTICIPANT_ID);
    ExpandingString* temp = new ExpandingString();
    temp->printf("<dds>"
                    "<participant>"
                        "<rtps>"
                            "<name>"
                                "%s"
                            "</name>"
                        "</rtps>"
                    "</participant>"
                "</dds>",xrce_topic->get_participant_name());

    if(temp->has_failed_allocation()){
        return false;
    }

    participant_req=uxr_buffer_create_participant_xml(&session,reliable_out,participant_id,0,temp->get_string(),UXR_REPLACE);

    temp->~ExpandingString();
    temp = new ExpandingString();
    topic_id=uxr_object_id(0x01,UXR_TOPIC_ID);

    temp->printf("<dds>"
                    "<topic>"
                            "<name>"
                                "%s"
                            "</name>"
                            "<dataType>"
                                "%s"
                            "</dataType>"
                    "</topic>"
                "</dds>",xrce_topic->get_topic_name(),xrce_topic->get_datatype_name());

    if(temp->has_failed_allocation()){
        return false;
    }

    topic_req=uxr_buffer_create_topic_xml(&session,reliable_out,topic_id,participant_id,temp->get_string(),UXR_REPLACE);

    pub_id=uxr_object_id(0x01,UXR_PUBLISHER_ID);
    const char* pub_xml = "";
    pub_req = uxr_buffer_create_publisher_xml(&session,reliable_out,pub_id,participant_id,pub_xml,UXR_REPLACE);

    temp->~ExpandingString();
    temp = new ExpandingString();
    dwriter_id = uxr_object_id(0x01,UXR_DATAWRITER_ID);
    
    temp->printf("<dds>"
                    "<data_writer>"
                        "<topic>"
                            "<kind>NO_KEY</kind>"
                            "<name>"
                                "%s"
                            "</name>"
                            "<dataType>"
                                "%s"
                            "</dataType>"
                        "</topic>"
                    "</data_writer>"
                  "</dds>",xrce_topic->get_topic_name(),xrce_topic->get_datatype_name());

    if(temp->has_failed_allocation()){
        return false;
    }

    dwriter_req = uxr_buffer_create_datawriter_xml(&session,reliable_out,dwriter_id,pub_id,temp->get_string(),UXR_REPLACE);

    temp->~ExpandingString();

    uint16_t requests[4] = {participant_req,topic_req,pub_req,dwriter_req};
    
    if (!uxr_run_session_until_all_status(&session,1000,requests,status,4)) {
        return false;
    }

    return true;
}

void AP_XRCE_Client::write()
{
    WITH_SEMAPHORE(csem);
    if(connected)
    {
        ucdrBuffer ub;
        uint32_t topic_size = xrce_topic->size_of_topic(0);
        uxr_prepare_output_stream(&session,reliable_out,dwriter_id,&ub,topic_size);
        xrce_topic->serialize_topic(&ub);
    }
    
}

void AP_XRCE_Client::update()
{
    if (xrce_topic == nullptr) {
        return;
    }
    WITH_SEMAPHORE(csem);
    xrce_topic->update_topic();
    connected = uxr_run_session_time(&session,1000);
}

/*
  implement C functions for serial transport
 */
extern "C" {
#include <uxr/client/profile/transport/serial/serial_transport_platform.h>
}

bool uxr_init_serial_platform(void* args, int fd, uint8_t remote_addr, uint8_t local_addr)
{
    return true;
}

bool uxr_close_serial_platform(void* args)
{
    return true;
}

size_t uxr_write_serial_data_platform(void* args, const uint8_t* buf, size_t len, uint8_t* errcode)
{
    if (xrce_port == nullptr) {
        *errcode = 1;
        return 0;
    }
    size_t bytes_written = xrce_port->write(buf, (size_t)len);
    if (bytes_written == 0) {
        *errcode = 1;
        return 0;
    }
    *errcode = 0;
    return bytes_written;
}

size_t uxr_read_serial_data_platform(void* args, uint8_t* buf, size_t len, int timeout, uint8_t* errcode)
{
    if (xrce_port == nullptr) {
        *errcode = 1;
        return 0;
    }
    while (timeout > 0 && xrce_port->available() < len) {
        hal.scheduler->delay(1);
        timeout--;
    }
    size_t bytes_read = xrce_port->read(buf, (size_t)len);
    if (bytes_read <= 0) {
        *errcode = 1;
        return 0;
    }
    *errcode = 0;
    return bytes_read;
}
#endif // AP_XRCE_ENABLED


