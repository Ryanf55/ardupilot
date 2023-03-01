#if AP_XRCE_ENABLED

#include "AP_XRCE_ROS2_Sensor_Msgs_Topics.h"
#include "ucdr/microcdr.h"
#include <string_view> 
#include <AP_HAL/AP_HAL.h>


constexpr char packageName[] = "sensor_msgs";

ROS2_SensorMsgsBatteryStateTopic::ROS2_SensorMsgsBatteryStateTopic()
{
}

bool ROS2_SensorMsgsBatteryStateTopic::serialize_topic(ucdrBuffer* writer)
{
    bool success = true;

    //header.stamp.sec
    success &= ucdr_serialize_int32_t(writer, 0);
    //header.stamp.nanosec
    success &= ucdr_serialize_uint32_t(writer, 1);
    //header.frame_id
    success &= ucdr_serialize_string(writer,"");
    //voltage
    success &= ucdr_serialize_float(writer, 0.0f);
    //temperature
    success &= ucdr_serialize_float(writer, 0.0f);
    //current
    success &= ucdr_serialize_float(writer, 0.0f);
    //capacity
    success &= ucdr_serialize_float(writer, 0.0f);
    //design_capacity
    success &= ucdr_serialize_float(writer, 0.0f);
    //percentage
    success &= ucdr_serialize_float(writer, 0.0f);
    //power_supply_status
    success &= ucdr_serialize_uint8_t(writer, 0.0f);
    //power_supply_health
    success &= ucdr_serialize_uint8_t(writer, 0.0f);
    //power_supply_technology
    success &= ucdr_serialize_uint8_t(writer, 0.0f);
    //present
    success &= ucdr_serialize_bool(writer, true);
    //cell_voltage
    const float voltage_array[0] = {};
    success &= ucdr_serialize_array_float(writer, voltage_array, 0);
    //cell_temperature
    const float temperature_array[0] = {};
    success &= ucdr_serialize_array_float(writer, temperature_array, 0);
    //location
    success &= ucdr_serialize_string(writer, "");
    //serial_number
    success &= ucdr_serialize_string(writer, "");

    return success & !writer->error;
}

bool ROS2_SensorMsgsBatteryStateTopic::deserialize_topic(ucdrBuffer* reader)
{
    // (void) ucdr_deserialize_bool(reader,&data);
    return !reader->error;
}

uint32_t ROS2_SensorMsgsBatteryStateTopic::size_of_topic(uint32_t size)
{
    uint32_t prevSize = size;

    // TODO calculate this
    // size += header.size_of_topic(&topic->header, size);

    size += ucdr_alignment(size, 4) + 4;
    size += ucdr_alignment(size, 4) + 4;
    size += ucdr_alignment(size, 4) + 4;
    size += ucdr_alignment(size, 4) + 4;
    size += ucdr_alignment(size, 4) + 4;
    size += ucdr_alignment(size, 4) + 4;

    size += ucdr_alignment(size, 1) + 1;
    size += ucdr_alignment(size, 1) + 1;
    size += ucdr_alignment(size, 1) + 1;
    size += ucdr_alignment(size, 1) + 1;

    size += ucdr_alignment(size, 4) + 4;
    // TODO add the rest of these in
    // size += ucdr_alignment(size, 4) + (topic->cell_voltage_size * 4);

    // size += ucdr_alignment(size, 4) + 4 + (uint32_t)(strlen(topic->location) + 1);
    // size += ucdr_alignment(size, 4) + 4 + (uint32_t)(strlen(topic->serial_number) + 1);

    return (size-prevSize);
}

void ROS2_SensorMsgsBatteryStateTopic::update_topic()
{
    // TODO
}

#endif // AP_XRCE_ENABLED