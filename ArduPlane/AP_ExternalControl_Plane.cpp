/*
  external control library for plane
 */


#include "AP_ExternalControl_Plane.h"
#if AP_EXTERNAL_CONTROL_ENABLED

#include "Plane.h"

/*
  Sets the target global position for a loiter point.
*/
bool AP_ExternalControl_Plane::set_global_position(const Location& loc)
{

    // set_target_location already checks if plane is ready for external control.
    // It doesn't check if flying or armed, just that it's in guided mode.
    return plane.set_target_location(loc);
}

bool AP_ExternalControl_Plane::set_trajectory(const Location trajectory[5], const uint8_t count)
{
  plane.mode_guided.trajectory.clear();
  for (size_t i = 0; i < count; i++) {
    plane.mode_guided.trajectory.push_back(trajectory[i]);
  }
  plane.mode_guided.trajectory_start();

  return true; 
}


#endif // AP_EXTERNAL_CONTROL_ENABLED
