#if AP_XRCE_ENABLED

#include "AP_XRCE_Custom_Topics.h"

class ROS2_Bool_Topic:public XRCE_Generic_Topic {

    public:

        ROS2_Bool_Topic();
        bool topic_initialize(uint8_t xrcetype) override;
        bool serialize_topic(ucdrBuffer *writer) override;
        bool deserialize_topic(ucdrBuffer *reader) override;
        uint32_t size_of_topic(uint32_t size) override;
        void update_topic() override;        

    private:

        bool data;
};

class ROS2_8Bit_Topic:public XRCE_Generic_Topic {

    public:

        ROS2_8Bit_Topic(uint8_t type8bit);
        bool topic_initialize(uint8_t xrcetype) override;
        bool serialize_topic(ucdrBuffer *writer) override;
        bool deserialize_topic(ucdrBuffer *reader) override;
        uint32_t size_of_topic(uint32_t size) override;
        void update_topic() override; 

    private:

        uint8_t u_data;
        int8_t data;
        uint8_t type_8bit;
};

class ROS2_Char_Topic:public XRCE_Generic_Topic {

    public:

        ROS2_Char_Topic();
        bool topic_initialize(uint8_t xrcetype) override;
        bool serialize_topic(ucdrBuffer *writer) override;
        bool deserialize_topic(ucdrBuffer *reader) override;
        uint32_t size_of_topic(uint32_t size) override;
        void update_topic() override; 

    private:

        char data;
};

class ROS2_16Bit_Topic:public XRCE_Generic_Topic {

    public:

        ROS2_16Bit_Topic(uint8_t type16bit);
        bool topic_initialize(uint8_t xrcetype) override;
        bool serialize_topic(ucdrBuffer *writer) override;
        bool deserialize_topic(ucdrBuffer *reader) override;
        uint32_t size_of_topic(uint32_t size) override;
        void update_topic() override;
 
    private:

        uint16_t u_data;
        int16_t data;
        uint8_t type_16bit;
};

class ROS2_32Bit_Topic:public XRCE_Generic_Topic {

    public:

        ROS2_32Bit_Topic(uint8_t type32bit);
        bool topic_initialize(uint8_t xrcetype) override;
        bool serialize_topic(ucdrBuffer *writer) override;
        bool deserialize_topic(ucdrBuffer *reader) override;
        uint32_t size_of_topic(uint32_t size) override;
        void update_topic() override;
 
    private:

        uint32_t u_data;
        int32_t data;
        uint8_t type_32bit;
};

class ROS2_64Bit_Topic:public XRCE_Generic_Topic {

    public:

        ROS2_64Bit_Topic(uint8_t type64bit);
        bool topic_initialize(uint8_t xrcetype) override;
        bool serialize_topic(ucdrBuffer *writer) override;
        bool deserialize_topic(ucdrBuffer *reader) override;
        uint32_t size_of_topic(uint32_t size) override;
        void update_topic() override;
 
    private:

        uint64_t u_data;
        int64_t data;
        uint8_t type_64bit;
};

class ROS2_Float32_Topic:public XRCE_Generic_Topic {

    public:

        ROS2_Float32_Topic();
        bool topic_initialize(uint8_t xrcetype) override;
        bool serialize_topic(ucdrBuffer *writer) override;
        bool deserialize_topic(ucdrBuffer *reader) override;
        uint32_t size_of_topic(uint32_t size) override;
        void update_topic() override;
 
    private:

        float data;
};

class ROS2_Float64_Topic:public XRCE_Generic_Topic {

    public:

        ROS2_Float64_Topic();
        bool topic_initialize(uint8_t xrcetype) override;
        bool serialize_topic(ucdrBuffer *writer) override;
        bool deserialize_topic(ucdrBuffer *reader) override;
        uint32_t size_of_topic(uint32_t size) override;
        void update_topic() override;
 
    private:

        double data;
};

class ROS2_String_Topic:public XRCE_Generic_Topic {

    public:

        ROS2_String_Topic();
        bool topic_initialize(uint8_t xrcetype) override;
        bool serialize_topic(ucdrBuffer *writer) override;
        bool deserialize_topic(ucdrBuffer *reader) override;
        uint32_t size_of_topic(uint32_t size) override;
        void update_topic() override;

    private:

        char* data;
};

class ROS2_ColorRGBA_Topic:public XRCE_Generic_Topic {

    public:

        ROS2_ColorRGBA_Topic();
        bool topic_initialize(uint8_t xrcetype) override;
        bool serialize_topic(ucdrBuffer *writer) override;
        bool deserialize_topic(ucdrBuffer *reader) override;
        uint32_t size_of_topic(uint32_t size) override;
        void update_topic() override;
 
    private:

        float r;
        float g;
        float b;
        float a;
};

class ROS2_Header_Topic:public XRCE_Generic_Topic {

    public:

        ROS2_Header_Topic();
        bool topic_initialize(uint8_t xrcetype) override;
        bool serialize_topic(ucdrBuffer *writer) override;
        bool deserialize_topic(ucdrBuffer *reader) override;
        uint32_t size_of_topic(uint32_t size) override;
        void update_topic() override;
 
    private:

        char* frame_id;
        int32_t sec;
        int32_t nanosec;
};

#endif // AP_XRCE_ENABLED