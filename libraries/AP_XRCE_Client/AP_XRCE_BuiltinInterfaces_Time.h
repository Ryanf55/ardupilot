#pragma once

namespace builtin_interfaces {


class Time
{
public: 
  int32_t sec;
  uint32_t nanosec;


  bool serialize(ucdrBuffer* writer, const Time* topic)
  {
    (void) ucdr_serialize_int32_t(writer, topic->sec);
    (void) ucdr_serialize_uint32_t(writer, topic->nanosec);

    return !writer->error;
  }

  uint32_t size_of_topic(uint32_t size)
  {
    uint32_t previousSize = size;
    size += ucdr_alignment(size, 4) + 4;
    size += ucdr_alignment(size, 4) + 4;

    return size - previousSize;
  }

};

} // namespace builtin_interfaces
