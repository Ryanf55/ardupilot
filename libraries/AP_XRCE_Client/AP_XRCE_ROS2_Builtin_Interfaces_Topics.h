#pragma once

#include "ucdr/microcdr.h"
class ROS2_BuiltinInterfacesTimeTopic {

public:

    ROS2_BuiltinInterfacesTimeTopic();
    bool serialize_topic(ucdrBuffer *writer);
    bool deserialize_topic(ucdrBuffer *reader);
    uint32_t size_of_topic(uint32_t size);
    void update_topic();

protected: 
    int32_t sec;
    uint32_t nanosec;
};