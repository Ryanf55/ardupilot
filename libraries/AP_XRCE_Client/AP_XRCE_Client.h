#pragma once

#if AP_XRCE_ENABLED

#include "uxr/client/client.h"
#include "ucdr/microcdr.h"
#include "AP_XRCE_ROS2_Builtin_Interfaces_Topics.h"
#include "Time.h"
#include "AP_XRCE_Generic_Fn_T.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Scheduler.h>
#include <AP_HAL/Semaphores.h>
#include <AP_AHRS/AP_AHRS.h>

#include "fcntl.h"

#include <AP_Param/AP_Param.h>
#define STREAM_HISTORY 8
#define BUFFER_SIZE_SERIAL UXR_CONFIG_SERIAL_TRANSPORT_MTU * STREAM_HISTORY

extern const AP_HAL::HAL& hal;

class AP_XRCE_Client {

private:

    // Serial Allocation
    uxrSerialTransport serial_transport; // client uxr serial transport
    uxrSession session; //Session

    // Input Stream
    uint8_t input_reliable_stream[BUFFER_SIZE_SERIAL];
    uxrStreamId reliable_in;

    // Output Stream
    uint8_t output_reliable_stream[BUFFER_SIZE_SERIAL];
    uxrStreamId reliable_out;

    // Topic
    builtin_interfaces_msg_Time time_topic;

    // Data Writer
    const uxrObjectId dwriter_id = {
        .id = 0x01,
        .type = UXR_DATAWRITER_ID
    };

    HAL_Semaphore csem;

    // connection parametrics
    bool connected = true;

    AP_Int8 xrce_type;
public:
    // Constructor
    AP_XRCE_Client();

    void main_loop(void);

    // 
    [[nodiscard]] bool init();



    // lookup serial manager to find the port
    // then call all the new 
    // if it works, then call init to set up the thread
    static void doAllocation();
    
    [[nodiscard]] bool create();
    void write();
    void update();
    static const struct AP_Param::GroupInfo var_info[];


    struct Topic_table {
        const uint8_t topic_id;
        const uint8_t pub_id;
        const char* topic_profile_label;
        const char* dw_profile_label;
        Generic_serialize_topic_fn_t serialize;
        Generic_deserialize_topic_fn_t deserialize;
        Generic_size_of_topic_fn_t size_of;
    };
    static const struct Topic_table topics[];


};

#endif // AP_XRCE_ENABLED


