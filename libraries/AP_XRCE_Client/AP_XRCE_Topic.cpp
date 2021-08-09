#include "AP_XRCE_Topic.h"

XRCE_Generic_Topic::XRCE_Generic_Topic() 
{
    particpant_name="Ardupilot_XRCE_Client";
}

std::string XRCE_Generic_Topic::get_participant_name()
{
    return particpant_name;
}

std::string XRCE_Generic_Topic::get_topic_name()
{
    return topic_name;
}

std::string XRCE_Generic_Topic::get_datatype_name()
{
    return datatype_name;
}
