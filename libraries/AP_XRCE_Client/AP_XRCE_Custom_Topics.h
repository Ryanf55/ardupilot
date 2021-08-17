#if AP_XRCE_ENABLED

#include "AP_XRCE_Topic.h"

class AP_NumTopic:public XRCE_Generic_Topic {

public:

    AP_NumTopic();
    bool topic_initialize(uint8_t xrcetype) override;
    bool serialize_topic(ucdrBuffer *writer) override;
    bool deserialize_topic(ucdrBuffer *reader) override;
    uint32_t size_of_topic(uint32_t size) override;
    void update_topic() override;

private:

    int32_t num;
};

class AP_BaroTopic:public XRCE_Generic_Topic {

public:

    AP_BaroTopic();
    bool topic_initialize(uint8_t xrcetype) override;
    bool serialize_topic(ucdrBuffer *writer) override;
    bool deserialize_topic(ucdrBuffer *reader) override;
    uint32_t size_of_topic(uint32_t size) override;
    void update_topic() override;

private:

    bool  healthy;
    float pressure;
    float pressure_correction;
    float temperature;
    float altitude;
};

class AP_GPSTopic:public XRCE_Generic_Topic {

public:

    AP_GPSTopic();
    bool topic_initialize(uint8_t xrcetype) override;
    bool serialize_topic(ucdrBuffer *writer) override;
    bool deserialize_topic(ucdrBuffer *reader) override;
    uint32_t size_of_topic(uint32_t size) override;
    void update_topic() override;

private:

    bool healthy;

    int32_t locationAlt;
    int32_t locationLat;
    int32_t locationLong;

    float velocityN;
    float velocityE;
    float velocityD;
    uint32_t groundSpeed;
    float groundCourse;

    uint8_t numSat;

    uint16_t timeWeek;
    uint32_t timeOfWeek;

    uint16_t hortizontalDOP;
    uint16_t verticalDOP;

};

class AP_CompassTopic:public XRCE_Generic_Topic {

public:

    AP_CompassTopic();
    bool topic_initialize(uint8_t xrcetype) override;
    bool serialize_topic(ucdrBuffer *writer) override;
    bool deserialize_topic(ucdrBuffer *reader) override;
    uint32_t size_of_topic(uint32_t size) override;
    void update_topic() override;

private:

    bool healthy;

    float magfieldX;
    float magfieldY;
    float magfieldZ;
};

class AP_INSTopic:public XRCE_Generic_Topic {

public:

    AP_INSTopic();
    bool topic_initialize(uint8_t xrcetype) override;
    bool serialize_topic(ucdrBuffer *writer) override;
    bool deserialize_topic(ucdrBuffer *reader) override;
    uint32_t size_of_topic(uint32_t size) override;
    void update_topic() override;

private:

    bool gyroHealthy;
    uint8_t gyroCount;
    float gyroX;
    float gyroY;
    float gyroZ;

    bool accelHealthy;
    uint8_t accelCount;
    float accelX;
    float accelY;
    float accelZ;
};

#endif // AP_XRCE_ENABLED