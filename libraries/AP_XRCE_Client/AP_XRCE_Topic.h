#include <stdint.h>
#include <stdbool.h>
#include <string>

enum XRCE_TYPE{
    DDS=0,
    uROS=1,
};

enum XRCE_TOPIC{
    AP_DDS_NUM=0,
    AP_DDS_BARO=1,
    AP_DDS_GPS=2,
    AP_DDS_COMPASS=3,
    AP_DDS_INS=4,
    AP_ROS2_Bool=5,
    AP_ROS2_Byte=6,
    AP_ROS2_8Int=7,
    AP_ROS2_8UInt=8,
    AP_ROS2_Char=9,
    AP_ROS2_16Int=10,
    AP_ROS2_16UInt=11,
    AP_ROS2_32Int=12,
    AP_ROS2_32UInt=13,
    AP_ROS2_64Int=14,
    AP_ROS2_64UInt=15,
    AP_ROS2_32Float=16,
    AP_ROS2_64Float=17,
    AP_ROS2_String=18,
    AP_ROS2_ColorRGBA=19,
    AP_ROS2_Header=20,
};

struct ucdrBuffer;

/*
XRCE_Custom_Topic
Info - Parent class for the Ardupilot's custom XRCE/ROS2 topics. 
*/
class XRCE_Generic_Topic {

public:

    XRCE_Generic_Topic();
    virtual void topic_initialize(uint8_t xrce_type) = 0;
    virtual bool serialize_topic(ucdrBuffer *writer) = 0;
    virtual bool deserialize_topic(ucdrBuffer *reader) = 0;
    virtual uint32_t size_of_topic(uint32_t size) = 0;
    virtual void update_topic() = 0;
    std::string get_participant_name();
    std::string get_topic_name();
    std::string get_datatype_name();

protected:

    std::string particpant_name;
    std::string topic_name;
    std::string datatype_name;
};

XRCE_Generic_Topic* set_topic_instance(uint16_t topic_key);
