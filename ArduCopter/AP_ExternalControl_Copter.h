/*
  external control library for copter
 */
#pragma once

#include <AP_ExternalControl/AP_ExternalControl.h>

class AP_ExternalControl_Copter : public AP_ExternalControl {
public:
    /*
      Set linear velocity and yaw rate. Pass NaN for yaw_rate_rads to not control yaw.
      Velocity is in earth frame, NED [m/s].
      Yaw is in earth frame, NED [rad/s].
     */
    bool set_linear_velocity_and_yaw_rate(const Vector3f &linear_velocity, float yaw_rate_rads) override;
};
