/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
  backend driver for airspeed from a I2C SDP3X sensor

  with thanks to https://github.com/PX4/Firmware/blob/master/src/drivers/sdp3x_airspeed
 */
#include "AP_Airspeed_Synthetic.h"

#if AP_AIRSPEED_SYNTHETIC_ENABLED

#include <SRV_Channel/SRV_Channel.h>

bool AP_Airspeed_Synthetic::get_airspeed(float& airspeed)
{
    const float throttle_out = MAX(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle), 1);
    const float ratio  =  throttle_out / aparm.throttle_cruise;

    // // interpolate between airspeed_max and airspeed_min
    // airspeed = linear_interpolate(plane.aparm.airspeed_min, plane.aparm.airspeed_max, )
    // // This case is constrained tighter as we don't have real speed info
    // speed_scaler = constrain_float(speed_scaler, 0.6f, 1.67f);

    return true;
}

#endif  // AP_AIRSPEED_SYNTHETIC_ENABLED
