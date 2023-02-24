#if AP_XRCE_ENABLED

#include "AP_XRCE_Topic.h"
#include "AP_XRCE_ROS2_Sensor_Msgs_Topics.h"
#include "AP_XRCE_ROS2_Builtin_Interfaces_Topics.h"
#include <stdio.h>

XRCE_Generic_Topic::XRCE_Generic_Topic() 
{
    particpant_name = (char*)"Ardupilot_DDS_XRCE_Client";
}

char* XRCE_Generic_Topic::get_participant_name()
{
    return particpant_name;
}

char* XRCE_Generic_Topic::get_topic_name()
{
    return topic_name;
}

char* XRCE_Generic_Topic::get_datatype_name()
{
    return datatype_name;
}

bool XRCE_Generic_Topic::uros_initialize(const char* packageName)
{
    ExpandingString *temptopic = new ExpandingString();
    temptopic->printf("rt/");
    if(topic_name != nullptr && !temptopic->has_failed_allocation()){
        if(temptopic->append(topic_name,strlen(topic_name))){
            topic_name = temptopic->get_writeable_string();
        } else {
            return false;
        }
    } else {
        return false;
    }
    ExpandingString *tempdtype = new ExpandingString();
    tempdtype->printf("%s::msg::dds_::", packageName);
    if(datatype_name != nullptr && !tempdtype->has_failed_allocation()){
        if(tempdtype->append(datatype_name,strlen(datatype_name))){
            if(tempdtype->append("_",(uint32_t)strlen("_"))){
                datatype_name = tempdtype->get_writeable_string();
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    } else {
        return false;
    }
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