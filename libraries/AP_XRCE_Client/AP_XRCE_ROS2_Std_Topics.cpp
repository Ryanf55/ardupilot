#if AP_XRCE_ENABLED

#include "AP_XRCE_ROS2_Std_Topics.h"
#include "ucdr/microcdr.h"

ROS2_Bool_Topic::ROS2_Bool_Topic()
:XRCE_Generic_Topic()
{
    topic_name = (char *)"AP_ROS2_Bool";
    datatype_name = (char *)"Bool";
}

bool ROS2_Bool_Topic::topic_initialize(uint8_t xrcetype)
{
    data = false;
    if(xrcetype == XRCE_TYPE::uROS){
        return (uros_initialize(true));
    }

    return true;
}

bool ROS2_Bool_Topic::serialize_topic(ucdrBuffer* writer)
{
    (void) ucdr_serialize_bool(writer,data);
    return !writer->error;
}

bool ROS2_Bool_Topic::deserialize_topic(ucdrBuffer* reader)
{
    (void) ucdr_deserialize_bool(reader,&data);
    return !reader->error;
}

uint32_t ROS2_Bool_Topic::size_of_topic(uint32_t size)
{
    uint32_t prevSize = size;
    size += ucdr_alignment(size,4) + sizeof(data);
    return (size-prevSize);
}

void ROS2_Bool_Topic::update_topic()
{
    data = !data;
}

ROS2_8Bit_Topic::ROS2_8Bit_Topic(uint8_t type8bit)
:XRCE_Generic_Topic()
{
    type_8bit = type8bit;
    if(type_8bit == XRCE_TOPIC::AP_ROS2_Byte) {
        topic_name = (char *)"AP_ROS2_Byte";
        datatype_name = (char *)"Byte";
    } else if (type_8bit == XRCE_TOPIC::AP_ROS2_8Int) {
        topic_name = (char *)"AP_ROS2_Int8";
        datatype_name = (char *)"Int8";
    } else {
        topic_name = (char *)"AP_ROS2_UInt8";
        datatype_name = (char *)"UInt8";
    }
}

bool ROS2_8Bit_Topic::topic_initialize(uint8_t xrcetype)
{
    if ((type_8bit == XRCE_TOPIC::AP_ROS2_Byte) || (type_8bit == XRCE_TOPIC::AP_ROS2_8UInt)) {
        u_data=0;
    } else {
        data=0;
    }
    if(xrcetype == XRCE_TYPE::uROS){
        return (uros_initialize(true));
    }

    return true;
}

bool ROS2_8Bit_Topic::serialize_topic(ucdrBuffer* writer)
{   
    if ((type_8bit == XRCE_TOPIC::AP_ROS2_Byte) || (type_8bit == XRCE_TOPIC::AP_ROS2_8UInt)) {
        (void) ucdr_serialize_uint8_t(writer,u_data);
    } else {
        (void) ucdr_serialize_int8_t(writer,data);
    }
    return !writer->error;
}

bool ROS2_8Bit_Topic::deserialize_topic(ucdrBuffer* reader)
{
    if ((type_8bit == XRCE_TOPIC::AP_ROS2_Byte) || (type_8bit == XRCE_TOPIC::AP_ROS2_8UInt)) {
        (void) ucdr_deserialize_uint8_t(reader,&u_data);
    } else {
        (void) ucdr_deserialize_int8_t(reader,&data);
    }
    return !reader->error;
}

uint32_t ROS2_8Bit_Topic::size_of_topic(uint32_t size)
{
    uint32_t prevSize = size;
    if ((type_8bit == XRCE_TOPIC::AP_ROS2_Byte) || (type_8bit == XRCE_TOPIC::AP_ROS2_8UInt)) {
        size += ucdr_alignment(size,4) + sizeof(u_data);
    } else {
        size += ucdr_alignment(size,4) + sizeof(data);
    }
    return (size-prevSize);
}

void ROS2_8Bit_Topic::update_topic()
{
    if ((type_8bit == XRCE_TOPIC::AP_ROS2_Byte) || (type_8bit == XRCE_TOPIC::AP_ROS2_8UInt)) {
        u_data++;
    } else {
        data++;
    }
}

ROS2_Char_Topic::ROS2_Char_Topic()
:XRCE_Generic_Topic()
{
    topic_name = (char *)"AP_ROS2_Char";
    datatype_name = (char *)"Char";
}

bool ROS2_Char_Topic::topic_initialize(uint8_t xrcetype)
{
    data = 'a';
    if(xrcetype == XRCE_TYPE::uROS){
        return (uros_initialize(true));
    }

    return true;
}

bool ROS2_Char_Topic::serialize_topic(ucdrBuffer* writer)
{
    (void) ucdr_serialize_char(writer,data);
    return !writer->error;
}

bool ROS2_Char_Topic::deserialize_topic(ucdrBuffer* reader)
{
    (void) ucdr_deserialize_char(reader,&data);
    return !reader->error;
}

uint32_t ROS2_Char_Topic::size_of_topic(uint32_t size)
{
    uint32_t prevSize = size;
    size += ucdr_alignment(size,4) + sizeof(data);
    return (size-prevSize);
}

void ROS2_Char_Topic::update_topic()
{
    data ++;
}

ROS2_16Bit_Topic::ROS2_16Bit_Topic(uint8_t type16bit)
:XRCE_Generic_Topic()
{
    type_16bit = type16bit;
    if(type_16bit == XRCE_TOPIC::AP_ROS2_16Int) {
        topic_name = (char *)"AP_ROS2_Int16";
        datatype_name = (char *)"Int16";
    } else {
        topic_name = (char *)"AP_ROS2_UInt16";
        datatype_name = (char *)"UInt16";
    }
}

bool ROS2_16Bit_Topic::topic_initialize(uint8_t xrcetype)
{
    if (type_16bit == XRCE_TOPIC::AP_ROS2_16UInt) {
        u_data=0;
    } else {
        data=0;
    }
    if(xrcetype == XRCE_TYPE::uROS){
        return (uros_initialize(true));
    }

    return true;
}

bool ROS2_16Bit_Topic::serialize_topic(ucdrBuffer* writer)
{   
    if (type_16bit == XRCE_TOPIC::AP_ROS2_16UInt) {
        (void) ucdr_serialize_uint16_t(writer,u_data);
    } else {
        (void) ucdr_serialize_int16_t(writer,data);
    }
    return !writer->error;
}

bool ROS2_16Bit_Topic::deserialize_topic(ucdrBuffer* reader)
{
    if (type_16bit == XRCE_TOPIC::AP_ROS2_16UInt) {
        (void) ucdr_deserialize_uint16_t(reader,&u_data);
    } else {
        (void) ucdr_deserialize_int16_t(reader,&data);
    }
    return !reader->error;
}

uint32_t ROS2_16Bit_Topic::size_of_topic(uint32_t size)
{
    uint32_t prevSize = size;
    if (type_16bit == XRCE_TOPIC::AP_ROS2_16UInt) {
        size += ucdr_alignment(size,4) + sizeof(u_data);
    } else {
        size += ucdr_alignment(size,4) + sizeof(data);
    }
    return (size-prevSize);
}

void ROS2_16Bit_Topic::update_topic()
{
    if (type_16bit == XRCE_TOPIC::AP_ROS2_16UInt) {
        u_data++;
    } else {
        data++;
    }
}

ROS2_32Bit_Topic::ROS2_32Bit_Topic(uint8_t type32bit)
:XRCE_Generic_Topic()
{
    type_32bit = type32bit;
    if(type_32bit == XRCE_TOPIC::AP_ROS2_32Int) {
        topic_name = (char *)"AP_ROS2_Int32";
        datatype_name = (char *)"Int32";
    } else {
        topic_name = (char *)"AP_ROS2_UInt32";
        datatype_name = (char *)"UInt32";
    }
}

bool ROS2_32Bit_Topic::topic_initialize(uint8_t xrcetype)
{
    if (type_32bit == XRCE_TOPIC::AP_ROS2_32UInt) {
        u_data=0;
    } else {
        data=0;
    }
    if(xrcetype == XRCE_TYPE::uROS){
        return (uros_initialize(true));
    }

    return true;
}

bool ROS2_32Bit_Topic::serialize_topic(ucdrBuffer* writer)
{   
    if (type_32bit == XRCE_TOPIC::AP_ROS2_32UInt) {
        (void) ucdr_serialize_uint32_t(writer,u_data);
    } else {
        (void) ucdr_serialize_int32_t(writer,data);
    }
    return !writer->error;
}

bool ROS2_32Bit_Topic::deserialize_topic(ucdrBuffer* reader)
{
    if (type_32bit == XRCE_TOPIC::AP_ROS2_32UInt) {
        (void) ucdr_deserialize_uint32_t(reader,&u_data);
    } else {
        (void) ucdr_deserialize_int32_t(reader,&data);
    }
    return !reader->error;
}

uint32_t ROS2_32Bit_Topic::size_of_topic(uint32_t size)
{
    uint32_t prevSize = size;
    if (type_32bit == XRCE_TOPIC::AP_ROS2_32UInt) {
        size += ucdr_alignment(size,4) + sizeof(u_data);
    } else {
        size += ucdr_alignment(size,4) + sizeof(data);
    }
    return (size-prevSize);
}

void ROS2_32Bit_Topic::update_topic()
{
    if (type_32bit == XRCE_TOPIC::AP_ROS2_32UInt) {
        u_data++;
    } else {
        data++;
    }
}

ROS2_64Bit_Topic::ROS2_64Bit_Topic(uint8_t type64bit)
:XRCE_Generic_Topic()
{
    type_64bit = type64bit;
    if(type_64bit == XRCE_TOPIC::AP_ROS2_64Int) {
        topic_name = (char *)"AP_ROS2_Int64";
        datatype_name = (char *)"Int64";
    } else {
        topic_name = (char *)"AP_ROS2_UInt64";
        datatype_name = (char *)"UInt64";
    }
}

bool ROS2_64Bit_Topic::topic_initialize(uint8_t xrcetype)
{
    if (type_64bit == XRCE_TOPIC::AP_ROS2_64UInt) {
        u_data=0;
    } else {
        data=0;
    }
    if(xrcetype == XRCE_TYPE::uROS){
        return (uros_initialize(true));
    }

    return true;
}

bool ROS2_64Bit_Topic::serialize_topic(ucdrBuffer* writer)
{   
    if (type_64bit == XRCE_TOPIC::AP_ROS2_64UInt) {
        (void) ucdr_serialize_uint64_t(writer,u_data);
    } else {
        (void) ucdr_serialize_int64_t(writer,data);
    }
    return !writer->error;
}

bool ROS2_64Bit_Topic::deserialize_topic(ucdrBuffer* reader)
{
    if (type_64bit == XRCE_TOPIC::AP_ROS2_64UInt) {
        (void) ucdr_deserialize_uint64_t(reader,&u_data);
    } else {
        (void) ucdr_deserialize_int64_t(reader,&data);
    }
    return !reader->error;
}

uint32_t ROS2_64Bit_Topic::size_of_topic(uint32_t size)
{
    uint32_t prevSize = size;
    if (type_64bit == XRCE_TOPIC::AP_ROS2_64UInt) {
        size += ucdr_alignment(size,4) + sizeof(u_data);
    } else {
        size += ucdr_alignment(size,4) + sizeof(data);
    }
    return (size-prevSize);
}

void ROS2_64Bit_Topic::update_topic()
{
    if (type_64bit == XRCE_TOPIC::AP_ROS2_64UInt) {
        u_data++;
    } else {
        data++;
    }
}

ROS2_Float32_Topic::ROS2_Float32_Topic()
:XRCE_Generic_Topic()
{
    topic_name = (char *)"AP_ROS2_Float32";
    datatype_name = (char *)"Float32";
}

bool ROS2_Float32_Topic::topic_initialize(uint8_t xrcetype)
{
    data = 0.0;
    if(xrcetype == XRCE_TYPE::uROS){
        return (uros_initialize(true));
    }

    return true;
}

bool ROS2_Float32_Topic::serialize_topic(ucdrBuffer* writer)
{
    (void) ucdr_serialize_float(writer,data);
    return !writer->error;
}

bool ROS2_Float32_Topic::deserialize_topic(ucdrBuffer* reader)
{
    (void) ucdr_deserialize_float(reader,&data);
    return !reader->error;
}

uint32_t ROS2_Float32_Topic::size_of_topic(uint32_t size)
{
    uint32_t prevSize = size;
    size += ucdr_alignment(size,4) + sizeof(data);
    return (size-prevSize);
}

void ROS2_Float32_Topic::update_topic()
{
    data += 0.5;
}

ROS2_Float64_Topic::ROS2_Float64_Topic()
:XRCE_Generic_Topic()
{
    topic_name = (char *)"AP_ROS2_Float64";
    datatype_name = (char *)"Float64";
}

bool ROS2_Float64_Topic::topic_initialize(uint8_t xrcetype)
{
    data = 0.0;
    if(xrcetype == XRCE_TYPE::uROS){
        return (uros_initialize(true));
    }

    return true;
}

bool ROS2_Float64_Topic::serialize_topic(ucdrBuffer* writer)
{
    (void) ucdr_serialize_double(writer,data);
    return !writer->error;
}

bool ROS2_Float64_Topic::deserialize_topic(ucdrBuffer* reader)
{
    (void) ucdr_deserialize_double(reader,&data);
    return !reader->error;
}

uint32_t ROS2_Float64_Topic::size_of_topic(uint32_t size)
{
    uint32_t prevSize = size;
    size += ucdr_alignment(size,4) + sizeof(data);
    return (size-prevSize);
}

void ROS2_Float64_Topic::update_topic()
{
    data += 0.5;
}

ROS2_String_Topic::ROS2_String_Topic()
:XRCE_Generic_Topic()
{
    topic_name = (char *)"AP_ROS2_String";
    datatype_name = (char *)"String";
}

bool ROS2_String_Topic::topic_initialize(uint8_t xrcetype)
{
    data = (char *)"Hello from Ardupilot";
    if(xrcetype == XRCE_TYPE::uROS){
        return(uros_initialize(true));
    }

    return true;
}

bool ROS2_String_Topic::serialize_topic(ucdrBuffer* writer)
{
    (void) ucdr_serialize_string(writer,data);
    return !writer->error;
}

bool ROS2_String_Topic::deserialize_topic(ucdrBuffer* reader)
{

    (void) ucdr_deserialize_string(reader,data,strlen(data));
    return !reader->error;
}

uint32_t ROS2_String_Topic::size_of_topic(uint32_t size)
{
    uint32_t prevSize = size;
    size += ucdr_alignment(size,4) + (strlen(data)+1)*sizeof(char);
    return (size-prevSize);
}

void ROS2_String_Topic::update_topic()
{
    data = (char *)"Hello from Ardupilot";
}

ROS2_ColorRGBA_Topic::ROS2_ColorRGBA_Topic()
:XRCE_Generic_Topic()
{
    topic_name = (char *)"AP_ROS2_ColorRGBA";
    datatype_name = (char *)"ColorRGBA";
}

bool ROS2_ColorRGBA_Topic::topic_initialize(uint8_t xrcetype)
{
    r = 0.0;
    g = 0.0;
    b = 0.0;
    a = 0.0;
    if(xrcetype == XRCE_TYPE::uROS){
        return (uros_initialize(true));
    }

    return true;
}

bool ROS2_ColorRGBA_Topic::serialize_topic(ucdrBuffer* writer)
{

    (void) ucdr_serialize_float(writer,r);
    (void) ucdr_serialize_float(writer,g);
    (void) ucdr_serialize_float(writer,b);
    (void) ucdr_serialize_float(writer,a);
    return !writer->error;
}

bool ROS2_ColorRGBA_Topic::deserialize_topic(ucdrBuffer* reader)
{
    (void) ucdr_deserialize_float(reader,&r);
    (void) ucdr_deserialize_float(reader,&g);
    (void) ucdr_deserialize_float(reader,&b);
    (void) ucdr_deserialize_float(reader,&a);
    return !reader->error;
}

uint32_t ROS2_ColorRGBA_Topic::size_of_topic(uint32_t size)
{
    uint32_t prevSize = size;
    size += ucdr_alignment(size,4) + sizeof(r);
    size += ucdr_alignment(size,4) + sizeof(g);
    size += ucdr_alignment(size,4) + sizeof(b);
    size += ucdr_alignment(size,4) + sizeof(a);
    return (size-prevSize);
}

void ROS2_ColorRGBA_Topic::update_topic()
{
    r += 0.5;
    g += 0.5;
    b += 0.5;
    a += 0.5;
}

ROS2_Header_Topic::ROS2_Header_Topic()
:XRCE_Generic_Topic()
{
    topic_name = (char *)"AP_ROS2_Header";
    datatype_name = (char *)"Header";
}

bool ROS2_Header_Topic::topic_initialize(uint8_t xrcetype)
{
    sec = 0;
    nanosec = 0;
    frame_id = (char *)"Ardupilot";
    if(xrcetype == XRCE_TYPE::uROS){
        return (uros_initialize(true));
    }

    return true;
}

bool ROS2_Header_Topic::serialize_topic(ucdrBuffer* writer)
{
    (void) ucdr_serialize_int32_t(writer,sec);
    (void) ucdr_serialize_int32_t(writer,nanosec);
    (void) ucdr_serialize_string(writer,frame_id);
    return !writer->error;
}

bool ROS2_Header_Topic::deserialize_topic(ucdrBuffer* reader)
{
    (void) ucdr_deserialize_int32_t(reader,&sec);
    (void) ucdr_deserialize_int32_t(reader,&nanosec);
    (void) ucdr_deserialize_string(reader,frame_id,strlen(frame_id));
    return !reader->error;
}

uint32_t ROS2_Header_Topic::size_of_topic(uint32_t size)
{
    uint32_t prevSize = size;
    size += ucdr_alignment(size,4) + sizeof(sec);
    size += ucdr_alignment(size,4) + sizeof(nanosec);
    size += ucdr_alignment(size,4) + (strlen(frame_id)+1)*sizeof(char);
    return (size-prevSize);
}

void ROS2_Header_Topic::update_topic()
{
    sec ++;
    nanosec ++;
}

XRCE_Generic_Topic* set_topic_instance(uint16_t topic_key)
{
    XRCE_Generic_Topic* topic;
    switch(topic_key){
        case XRCE_TOPIC::AP_ROS2_Header:
            topic = new ROS2_Header_Topic();
            break;
        case XRCE_TOPIC::AP_ROS2_ColorRGBA:
            topic = new ROS2_ColorRGBA_Topic();
            break;
        case XRCE_TOPIC::AP_ROS2_String:
            topic = new ROS2_String_Topic();
            break;
        case XRCE_TOPIC::AP_ROS2_64Float:
            topic = new ROS2_Float32_Topic();
            break;
        case XRCE_TOPIC::AP_ROS2_32Float:
            topic = new ROS2_Float64_Topic();
            break;
        case XRCE_TOPIC::AP_ROS2_64UInt:
            topic = new ROS2_64Bit_Topic(XRCE_TOPIC::AP_ROS2_64UInt);
            break;
        case XRCE_TOPIC::AP_ROS2_64Int:
            topic = new ROS2_64Bit_Topic(XRCE_TOPIC::AP_ROS2_64Int);
            break;
        case XRCE_TOPIC::AP_ROS2_32UInt:
            topic = new ROS2_32Bit_Topic(XRCE_TOPIC::AP_ROS2_32UInt);
            break;
        case XRCE_TOPIC::AP_ROS2_32Int:
            topic = new ROS2_32Bit_Topic(XRCE_TOPIC::AP_ROS2_32Int);
            break;
        case XRCE_TOPIC::AP_ROS2_16UInt:
            topic = new ROS2_16Bit_Topic(XRCE_TOPIC::AP_ROS2_16UInt);
            break;
        case XRCE_TOPIC::AP_ROS2_16Int:
            topic = new ROS2_16Bit_Topic(XRCE_TOPIC::AP_ROS2_16Int);
            break;
        case XRCE_TOPIC::AP_ROS2_Char:
            topic = new ROS2_Char_Topic();
            break;
        case XRCE_TOPIC::AP_ROS2_8UInt:
            topic = new ROS2_8Bit_Topic(XRCE_TOPIC::AP_ROS2_8UInt);
            break;
        case XRCE_TOPIC::AP_ROS2_8Int:
            topic = new ROS2_8Bit_Topic(XRCE_TOPIC::AP_ROS2_8Int);
            break;
        case XRCE_TOPIC::AP_ROS2_Byte:
            topic = new ROS2_8Bit_Topic(XRCE_TOPIC::AP_ROS2_Byte);
            break;
        case XRCE_TOPIC::AP_ROS2_Bool:
            topic = new ROS2_Bool_Topic();
            break;
        case XRCE_TOPIC::AP_DDS_INS:
            topic = new AP_INSTopic();
            break;
        case XRCE_TOPIC::AP_DDS_COMPASS:
            topic = new AP_CompassTopic();
            break;
        case XRCE_TOPIC::AP_DDS_GPS:
            topic = new AP_GPSTopic();
            break;
        case XRCE_TOPIC::AP_DDS_BARO:
            topic = new AP_BaroTopic();
            break;
        case XRCE_TOPIC::AP_DDS_NUM:
        default:
            topic = new AP_NumTopic();
            break;
    }
    return topic;
}

#endif // AP_XRCE_ENABLED