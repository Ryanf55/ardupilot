#pragma once

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_SYNTHETIC_ENABLED

#include "AP_Airspeed_Backend.h"

// An airspeed estimate based on drag and throttle.
class AP_Airspeed_Synthetic : public AP_Airspeed_Backend {
public:

    using AP_Airspeed_Backend::AP_Airspeed_Backend;

    AP_Airspeed_Synthetic(Plane &plane) : _plane(plane)
    
    bool init() override { return true; }

    // true if sensor reads airspeed directly, not via pressure
    bool has_airspeed() override { return true; }

    // return the current temperature in degrees C, if available
    bool get_temperature(float &temperature) override { return false; };

    // return airspeed in m/s if available
    bool get_airspeed(float& airspeed) override;


private:
    Plane &_plane;

};


#endif  // AP_AIRSPEED_SYNTHETIC_ENABLED
