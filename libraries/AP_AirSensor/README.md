# ArduPilot Air Sensor Library

This library is an abstraction over the various air sensing libraries to provide a common interface for retrieving sensor data that describes the wind. In the future, 
it could provide other data such as temperature, humidity, tubulence, and adverse 
weather conditions such as rain, hail, fog that may impact flight safety.

# Architecture

This follows the battery monitor and GPS architecture with different types such as AP_Battery_Scripting.

```mermaid
---
title: Class Diagram
---
classDiagram

    class Type {
        <<enumeration>>
        NONE = 0
        AIRSPEED = 1
        WINDVANE = 2
        SCRIPT = 3
    }

    class State {
        <<struct>>
        Vector3f airspeed
        bool has_temperature
        float temperature
        bool has_pressure
        float pressure
    }

    class AP_AirSensor {
        +bool enabled(const uint8_t i) const
        +bool healthy(const uint8_t i) const
        +void update()
        +uint8_t num_sensors() const
        +bool get_temperature(uint8_t i, float &temperature) const
        +bool get_pressure(uint8_t i, float &pressure) const
        +bool can_calibrate() const
        +CalibrationState get_calibration_state() const
        +enum Type configured_type(uint8_t instance) const
        +Vector3f get_airspeed(const uint8_t i) const
        +static AP_AirSensor* get_singleton()
        -Array[State] state[AIR_SENSOR_MAX_SENSORS]
        -AP_AirSensorBackend* sensor[AIR_SENSOR_MAX_SENSORS]
    }

    class AP_AirSensorBackend {
        +^void update()
        +void calibrate(bool in_startup)
        +Type configured_type(uint8_t instance) const
        +bool get_temperature(uint8_t i, float &temperature) const
    }

    class AP_AirSensorAirspeed {
        +^bool enabled(const uint8_t i) const
        +^void update()
        +^bool healthy(const uint8_t i) const
        +^Vector3f get_airspeed(const uint8_t i) const
    }

    class AP_AirSensorWindVane {
        +^bool enabled(const uint8_t i) const
        +^void update()
        +^bool healthy(const uint8_t i) const
        +^Vector3f get_airspeed(const uint8_t i) const
    }

    class AP_AirSensorScripting {
        +^bool enabled(const uint8_t i) const
        +^void update()
        +^bool healthy(const uint8_t i) const
        +^Vector3f get_airspeed(const uint8_t i) const
    }

    class AP_Airspeed {
        float get_airspeed(uint8_t i) const
    }

    AP_AirSensorBackend <|-- AP_AirSensorAirspeed
    AP_AirSensorBackend <|-- AP_AirSensorWindVane
    AP_AirSensorBackend <|-- AP_AirSensorScripting

    AP_AirSensorBackend ..> AP_AirSensorAirspeed : "friend"
    AP_AirSensor *-- Type : "enum"
    AP_AirSensor *-- State : "state"
```