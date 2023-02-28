#if AP_XRCE_ENABLED

#include "AP_XRCE_Topic.h"
#include "AP_XRCE_ROS2_Sensor_Msgs_Topics.h"
#include "AP_XRCE_ROS2_Builtin_Interfaces_Topics.h"
#include <stdio.h>

XRCE_Generic_Topic::XRCE_Generic_Topic() 
{
}

XRCE_Generic_Topic* set_topic_instance(uint16_t topic_key)
{
    XRCE_Generic_Topic* topic = nullptr;
    switch(topic_key){
        // case XRCE_TOPIC::AP_ROS2_BatteryState:
        //     topic = new ROS2_SensorMsgsBatteryStateTopic();
        //     break;
        case XRCE_TOPIC::AP_ROS2_Time:
            topic = new ROS2_BuiltinInterfacesTimeTopic();
            break;
        default:
            // TODO handle unsupported parameter error
            break;
    }
    return topic;
}

#endif // AP_XRCE_ENABLED