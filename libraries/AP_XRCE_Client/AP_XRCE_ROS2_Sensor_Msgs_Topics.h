#pragma once

#include "ucdr/microcdr.h"
class ROS2_SensorMsgsBatteryStateTopic {

public:

    ROS2_SensorMsgsBatteryStateTopic();
    bool serialize_topic(ucdrBuffer *writer);
    bool deserialize_topic(ucdrBuffer *reader);
    uint32_t size_of_topic(uint32_t size);
    void update_topic();
};