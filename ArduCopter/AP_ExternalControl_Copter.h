/*
  external control library for copter
 */
#pragma once

#include <AP_ExternalControl/AP_ExternalControl.h>

class AP_ExternalControl_Copter : public AP_ExternalControl {
public:
    /*
      set linear velocity and yaw rate. Pass NaN for yaw_rate_rads to not control yaw
      velocity is in earth frame, NED, m/s
     */
    bool set_linear_velocity_and_yaw_rate(const Vector3f &linear_velocity, float yaw_rate_rads) override;
};
