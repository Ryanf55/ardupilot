#include "AP_XRCE_Custom_Topics.h"
#include "ucdr/microcdr.h"
#include <AP_Baro/AP_Baro.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_InertialSensor/AP_InertialSensor.h>

AP_NumTopic::AP_NumTopic()
:XRCE_Generic_Topic()
{
    topic_name = "AP_NumTopic";
    datatype_name = "APNum";
}

void AP_NumTopic::topic_initialize(uint8_t xrcetype)
{
    num = 0;
    if(xrcetype == XRCE_TYPE::uROS){
        topic_name = "rt/"+topic_name;
        datatype_name = "ardupilotmsgs::msg::dds_::"+datatype_name+"_";
    }
}

bool AP_NumTopic::serialize_topic(ucdrBuffer *writer)
{
    (void) ucdr_serialize_int32_t(writer,num);
    return !writer->error;
}

bool AP_NumTopic::deserialize_topic(ucdrBuffer *reader)
{
    (void) ucdr_deserialize_int32_t(reader,&num);
    return !reader->error;
}

uint32_t AP_NumTopic::size_of_topic(uint32_t size)
{
    uint32_t prevSize = size;
    size += ucdr_alignment(size,4)+sizeof(num);
    return (size - prevSize); 
}

void AP_NumTopic::update_topic()
{
    num++;
}

AP_BaroTopic::AP_BaroTopic()
:XRCE_Generic_Topic()
{
    topic_name = "AP_BaroTopic";
    datatype_name = "APBaro";
}

void AP_BaroTopic::topic_initialize(uint8_t xrcetype)
{
    healthy=false;
    pressure=0.0;
    pressure_correction=0.0;
    temperature=0.0;
    altitude=0.0;
    if(xrcetype == XRCE_TYPE::uROS){
        topic_name = "rt/"+topic_name;
        datatype_name = "ardupilotmsgs::msg::dds_::"+datatype_name+"_";
    }
}

bool AP_BaroTopic::serialize_topic(ucdrBuffer *writer)
{
    (void) ucdr_serialize_bool(writer,healthy);
    (void) ucdr_serialize_float(writer,pressure);
    (void) ucdr_serialize_float(writer,pressure_correction);
    (void) ucdr_serialize_float(writer,temperature);
    (void) ucdr_serialize_float(writer,altitude);
    return !writer->error;
}

bool AP_BaroTopic::deserialize_topic(ucdrBuffer *reader)
{
    (void) ucdr_deserialize_bool(reader,&healthy);
    (void) ucdr_deserialize_float(reader,&pressure);
    (void) ucdr_deserialize_float(reader,&pressure_correction);
    (void) ucdr_deserialize_float(reader,&temperature);
    (void) ucdr_deserialize_float(reader,&altitude);
    return !reader->error;
}

uint32_t AP_BaroTopic::size_of_topic(uint32_t size)
{
    uint32_t prevSize = size;
    size += ucdr_alignment(size,4)+sizeof(healthy);
    size += ucdr_alignment(size,4)+sizeof(pressure);
    size += ucdr_alignment(size,4)+sizeof(pressure_correction);
    size += ucdr_alignment(size,4)+sizeof(temperature);
    size += ucdr_alignment(size,4)+sizeof(altitude);
    return (size-prevSize);
}

void AP_BaroTopic::update_topic()
{
    healthy = AP::baro().healthy();
    pressure = AP::baro().get_pressure();
    pressure_correction = AP::baro().get_pressure_correction();
    temperature = AP::baro().get_temperature();
    altitude = AP::baro().get_altitude();
}

AP_GPSTopic::AP_GPSTopic()
:XRCE_Generic_Topic()
{
    topic_name = "AP_GPSTopic";
    datatype_name = "APGPS";
}

void AP_GPSTopic::topic_initialize(uint8_t xrcetype)
{
    healthy=false;
    locationLat=0;
    locationLong=0;
    locationAlt=0;
    velocityN=0.0;
    velocityE=0.0;
    velocityD=0.0;
    groundSpeed=0;
    groundCourse=0;
    numSat=0;
    timeOfWeek=0;
    timeWeek=0;
    hortizontalDOP=0;
    verticalDOP=0;

    if(xrcetype == XRCE_TYPE::uROS){
        topic_name = "rt/"+topic_name;
        datatype_name = "ardupilotmsgs::msg::dds_::"+datatype_name+"_";
    }
}

bool AP_GPSTopic::serialize_topic(ucdrBuffer *writer)
{
    (void) ucdr_serialize_bool(writer,healthy);
    
    (void) ucdr_serialize_int32_t(writer,locationAlt);
    (void) ucdr_serialize_int32_t(writer,locationLat);
    (void) ucdr_serialize_int32_t(writer,locationLong);

    (void) ucdr_serialize_float(writer,velocityN);
    (void) ucdr_serialize_float(writer,velocityE);
    (void) ucdr_serialize_float(writer,velocityD);
    (void) ucdr_serialize_uint32_t(writer,groundSpeed);
    (void) ucdr_serialize_float(writer,groundCourse);

    (void) ucdr_serialize_uint8_t(writer,numSat);
    
    (void) ucdr_serialize_uint16_t(writer,timeWeek);
    (void) ucdr_serialize_uint32_t(writer,timeOfWeek);

    (void) ucdr_serialize_uint16_t(writer,hortizontalDOP);
    (void) ucdr_serialize_uint16_t(writer,verticalDOP);

    return !writer->error;
}

bool AP_GPSTopic::deserialize_topic(ucdrBuffer *reader)
{
    (void) ucdr_deserialize_bool(reader,&healthy);
    
    (void) ucdr_deserialize_int32_t(reader,&locationAlt);
    (void) ucdr_deserialize_int32_t(reader,&locationLat);
    (void) ucdr_deserialize_int32_t(reader,&locationLong);

    (void) ucdr_deserialize_float(reader,&velocityN);
    (void) ucdr_deserialize_float(reader,&velocityE);
    (void) ucdr_deserialize_float(reader,&velocityD);
    (void) ucdr_deserialize_uint32_t(reader,&groundSpeed);
    (void) ucdr_deserialize_float(reader,&groundCourse);

    (void) ucdr_deserialize_uint8_t(reader,&numSat);
    
    (void) ucdr_deserialize_uint16_t(reader,&timeWeek);
    (void) ucdr_deserialize_uint32_t(reader,&timeOfWeek);

    (void) ucdr_deserialize_uint16_t(reader,&hortizontalDOP);
    (void) ucdr_deserialize_uint16_t(reader,&verticalDOP);
    return !reader->error;
}

uint32_t AP_GPSTopic::size_of_topic(uint32_t size)
{
    uint32_t prevSize = size;
    size += ucdr_alignment(size,4)+sizeof(healthy);

    size += ucdr_alignment(size,4)+sizeof(locationAlt);
    size += ucdr_alignment(size,4)+sizeof(locationLat);
    size += ucdr_alignment(size,4)+sizeof(locationLong);

    size += ucdr_alignment(size,4)+sizeof(velocityN);
    size += ucdr_alignment(size,4)+sizeof(velocityE);
    size += ucdr_alignment(size,4)+sizeof(velocityD);
    size += ucdr_alignment(size,4)+sizeof(groundSpeed);
    size += ucdr_alignment(size,4)+sizeof(groundCourse);

    size += ucdr_alignment(size,4)+sizeof(numSat);

    size += ucdr_alignment(size,4)+sizeof(timeWeek);
    size += ucdr_alignment(size,4)+sizeof(timeOfWeek);
    
    size += ucdr_alignment(size,4)+sizeof(hortizontalDOP);
    size += ucdr_alignment(size,4)+sizeof(verticalDOP);

    return (size-prevSize);
}

void AP_GPSTopic::update_topic()
{
    healthy = AP::gps().is_healthy();
    
    numSat = AP::gps().num_sats();
    
    timeWeek = AP::gps().time_week();
    timeOfWeek = AP::gps().time_week_ms();

    hortizontalDOP = AP::gps().get_hdop();
    verticalDOP = AP::gps().get_vdop();

    const Vector3f& tempVector = AP::gps().velocity();
    velocityN = tempVector.x;
    velocityE = tempVector.y;
    velocityD = tempVector.z;

    const Location& tempLocation = AP::gps().location();
    locationAlt = tempLocation.alt;
    locationLat = tempLocation.lat;
    locationLong = tempLocation.lng;

    groundSpeed = AP::gps().ground_speed();
    groundCourse = AP::gps().ground_course();
}

AP_CompassTopic::AP_CompassTopic()
:XRCE_Generic_Topic()
{
    topic_name = "AP_CompassTopic";
    datatype_name = "APCompass";
}

void AP_CompassTopic::topic_initialize(uint8_t xrcetype)
{
    healthy=false;
    magfieldX=0.0;
    magfieldY=0.0;
    magfieldZ=0.0;
    if(xrcetype == XRCE_TYPE::uROS){
        topic_name = "rt/"+topic_name;
        datatype_name = "ardupilotmsgs::msg::dds_::"+datatype_name+"_";
    }
}

bool AP_CompassTopic::serialize_topic(ucdrBuffer *writer)
{
    (void) ucdr_serialize_bool(writer,healthy);
    (void) ucdr_serialize_float(writer,magfieldX);
    (void) ucdr_serialize_float(writer,magfieldY);
    (void) ucdr_serialize_float(writer,magfieldZ);
    return !writer->error;
}

bool AP_CompassTopic::deserialize_topic(ucdrBuffer *reader)
{
    (void) ucdr_deserialize_bool(reader,&healthy);
    (void) ucdr_deserialize_float(reader,&magfieldX);
    (void) ucdr_deserialize_float(reader,&magfieldY);
    (void) ucdr_deserialize_float(reader,&magfieldZ);
    return !reader->error;
}

uint32_t AP_CompassTopic::size_of_topic(uint32_t size)
{
    uint32_t prevSize = size;
    size += ucdr_alignment(size,4)+sizeof(healthy);
    size += ucdr_alignment(size,4)+sizeof(magfieldX);
    size += ucdr_alignment(size,4)+sizeof(magfieldY);
    size += ucdr_alignment(size,4)+sizeof(magfieldZ);
    return (size-prevSize);
}

void AP_CompassTopic::update_topic()
{
    healthy = AP::compass().healthy();
    const Vector3f& tempField = AP::compass().get_field();
    magfieldX = tempField.x;
    magfieldY = tempField.y;
    magfieldZ = tempField.z;
}

AP_INSTopic::AP_INSTopic()
:XRCE_Generic_Topic()
{
    topic_name = "AP_INSTopic";
    datatype_name = "APINS";
}

void AP_INSTopic::topic_initialize(uint8_t xrcetype)
{
    gyroHealthy = false;
    gyroCount = 0;
    gyroX = 0.0;
    gyroY = 0.0;
    gyroZ = 0.0;

    accelHealthy = false;
    accelCount = 0;
    accelX = 0.0;
    accelY = 0.0;
    accelZ = 0.0;

    if(xrcetype == XRCE_TYPE::uROS){
        topic_name = "rt/"+topic_name;
        datatype_name = "ardupilotmsgs::msg::dds_::"+datatype_name+"_";
    }
}

bool AP_INSTopic::serialize_topic(ucdrBuffer *writer)
{
    (void) ucdr_serialize_bool(writer,gyroHealthy);
    (void) ucdr_serialize_uint8_t(writer,gyroCount);
    (void) ucdr_serialize_float(writer,gyroX);
    (void) ucdr_serialize_float(writer,gyroY);
    (void) ucdr_serialize_float(writer,gyroZ);

    (void) ucdr_serialize_bool(writer,accelHealthy);
    (void) ucdr_serialize_uint8_t(writer,accelCount);
    (void) ucdr_serialize_float(writer,accelX);
    (void) ucdr_serialize_float(writer,accelY);
    (void) ucdr_serialize_float(writer,accelZ);
    return !writer->error;
}

bool AP_INSTopic::deserialize_topic(ucdrBuffer *reader)
{
    (void) ucdr_deserialize_bool(reader,&gyroHealthy);
    (void) ucdr_deserialize_uint8_t(reader,&gyroCount);
    (void) ucdr_deserialize_float(reader,&gyroX);
    (void) ucdr_deserialize_float(reader,&gyroY);
    (void) ucdr_deserialize_float(reader,&gyroZ);

    (void) ucdr_deserialize_bool(reader,&accelHealthy);
    (void) ucdr_deserialize_uint8_t(reader,&accelCount);
    (void) ucdr_deserialize_float(reader,&accelX);
    (void) ucdr_deserialize_float(reader,&accelY);
    (void) ucdr_deserialize_float(reader,&accelZ);
    return !reader->error;
}

uint32_t AP_INSTopic::size_of_topic(uint32_t size)
{
    uint32_t prevSize = size;
    size += ucdr_alignment(size,4)+sizeof(gyroHealthy);
    size += ucdr_alignment(size,4)+sizeof(gyroCount);
    size += ucdr_alignment(size,4)+sizeof(gyroX);
    size += ucdr_alignment(size,4)+sizeof(gyroY);
    size += ucdr_alignment(size,4)+sizeof(gyroZ);

    size += ucdr_alignment(size,4)+sizeof(accelHealthy);
    size += ucdr_alignment(size,4)+sizeof(accelCount);
    size += ucdr_alignment(size,4)+sizeof(accelX);
    size += ucdr_alignment(size,4)+sizeof(accelY);
    size += ucdr_alignment(size,4)+sizeof(accelZ);
    return (size-prevSize);
}

void AP_INSTopic::update_topic()
{
    gyroHealthy = AP::ins().get_gyro_health();
    gyroCount = AP::ins().get_gyro_count();
    const Vector3f& tempGyro = AP::ins().get_gyro();
    gyroX = tempGyro.x;
    gyroY = tempGyro.y;
    gyroZ = tempGyro.z;

    accelHealthy = AP::ins().get_accel_health();
    accelCount = AP::ins().get_accel_count();
    const Vector3f& tempAccel = AP::ins().get_accel();
    accelX = tempAccel.x;
    accelY = tempAccel.y;
    accelZ = tempAccel.z;
}
