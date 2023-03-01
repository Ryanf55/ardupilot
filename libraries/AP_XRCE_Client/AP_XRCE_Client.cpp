#include "AP_XRCE_Client.h"
#include "AP_XRCE_ROS2_Builtin_Interfaces_Topics.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_SerialManager/AP_SerialManager.h>


#if AP_XRCE_ENABLED

AP_HAL::UARTDriver *xrce_port;


const AP_Param::GroupInfo AP_XRCE_Client::var_info[]={
    // @Param: TYPE
    // @DisplayName: XRCE_TYPE
    // @Description: Type of XRCE protocol to use
    // @Values: 0:DDS,1:uROS(micro-ROS)
    // @User: Standard
    AP_GROUPINFO("TYPE", 1, AP_XRCE_Client, xrce_type, 0),

    AP_GROUPEND
};

#include "AP_XRCE_Topic_Table.h"

// Constructor (takes maximum number of topics as argument,by default it is 1)
AP_XRCE_Client::AP_XRCE_Client()
{
    relativeSerialClientAddr=1;
    relativeSerialAgentAddr=0;
    connected=true;
}

bool AP_XRCE_Client::init()
{
    AP_SerialManager *serial_manager = AP_SerialManager::get_singleton();
    xrce_port = serial_manager->find_serial(AP_SerialManager::SerialProtocol_DDS_XRCE, 0);
    if (xrce_port == nullptr) {
        return false;
    }

    if (!uxr_init_serial_transport(&serial_transport,fd,relativeSerialAgentAddr,relativeSerialClientAddr)) {
        return false;
    }
    constexpr uint32_t uniqueClientKey = 0xAAAABBBB;
    uxr_init_session(&session, &serial_transport.comm, uniqueClientKey);

    if (!uxr_create_session(&session)) {
        return false;
    }

    reliable_in=uxr_create_input_reliable_stream(&session,input_reliable_stream,BUFFER_SIZE_SERIAL,STREAM_HISTORY);
    reliable_out=uxr_create_output_reliable_stream(&session,output_reliable_stream,BUFFER_SIZE_SERIAL,STREAM_HISTORY);

    // Duplicate check for xrce_port, could simplify
    if (xrce_port != nullptr) {
         // do allocations
        xrce_topic = new ROS2_BuiltinInterfacesTimeTopic();
    }


    if(xrce_topic == nullptr) {
        return false;
    }

    return true;
}
bool AP_XRCE_Client::create()
{
    WITH_SEMAPHORE(csem);

    // Participant
    participant_id=uxr_object_id(0x01,UXR_PARTICIPANT_ID);
    const char* participant_ref = "participant_profile";
    participant_req=uxr_buffer_create_participant_ref(&session, reliable_out, participant_id,0,participant_ref,UXR_REPLACE);

    // Topic
    topic_id=uxr_object_id(0x01,UXR_TOPIC_ID);
    const char* topic_ref = "my_qos_label__t";
    topic_req=uxr_buffer_create_topic_ref(&session,reliable_out,topic_id,participant_id,topic_ref,UXR_REPLACE);

    // Publisher
    pub_id=uxr_object_id(0x01,UXR_PUBLISHER_ID);
    const char* pub_xml = "";
    pub_req = uxr_buffer_create_publisher_xml(&session,reliable_out,pub_id,participant_id,pub_xml,UXR_REPLACE);

    // Data Writer
    dwriter_id = uxr_object_id(0x01,UXR_DATAWRITER_ID);
    const char* data_writer_ref = "my_qos_label__dw";
    dwriter_req = uxr_buffer_create_datawriter_ref(&session,reliable_out,dwriter_id,pub_id,data_writer_ref,UXR_REPLACE);
    

    constexpr uint8_t nRequests = 4;
    uint16_t requests[nRequests] = {participant_req,topic_req,pub_req,dwriter_req};

    if (!uxr_run_session_until_all_status(&session,1000,requests,status,nRequests)) {
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
        const bool success = xrce_topic->serialize_topic(&ub);
        if (!success) {
            // AP_HAL::panic("FATAL: XRCE_Client failed to serialize %s\n",
            //     xrce_topic->get_datatype_name());
        }

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
        hal.scheduler->delay(1); // TODO select or poll this is limiting speed (1mS)
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

#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
extern "C" {
    int clock_gettime(clockid_t clockid, struct timespec *ts);
}

int clock_gettime(clockid_t clockid, struct timespec *ts)
{
    uint64_t utc_usec;
    if (!AP::rtc().get_utc_usec(utc_usec)) {
        return -1;
    }
    ts->tv_sec = utc_usec / 1000000ULL;
    ts->tv_nsec = (utc_usec % 1000000ULL) * 1000UL;
    return 0;
}
#endif // CONFIG_HAL_BOARD

#endif // AP_XRCE_ENABLED


