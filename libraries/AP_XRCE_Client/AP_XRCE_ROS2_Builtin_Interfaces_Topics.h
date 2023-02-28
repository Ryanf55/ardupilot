#pragma once

#include "AP_XRCE_Topic.h"

class ROS2_BuiltinInterfacesTimeTopic:public XRCE_Generic_Topic {

public:

    ROS2_BuiltinInterfacesTimeTopic();
    bool serialize_topic(ucdrBuffer *writer) override;
    bool deserialize_topic(ucdrBuffer *reader) override;
    uint32_t size_of_topic(uint32_t size) override;
    void update_topic() override;

protected: 
    int32_t sec;
    uint32_t nanosec;
};