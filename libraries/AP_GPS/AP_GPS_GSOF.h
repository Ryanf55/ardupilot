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

//
//  Trimble GPS driver for ArduPilot.
//	Code by Michael Oborne
//

// test - ardupilot gps relay packet.
// Applanix could run sitl on their computer, point sitl at the com port with option to get replay logs, and we get a replay.
// If binary data has both timing and whole data, we can convert.
// We would love to have replay logs from a moving vehicle.
// Replay format includes ALL binary data (block timestamp on a read()).
// Best to be done against a ublox GPS in flight.
// OK to walk around and deal with heights.
// May want to write this as an external_ahrs driver to allow secting primary EKF as external solution vs EKF3 for example to allow comparing solution. AP_AHRS should be called AP_INS.
// Weak point - log analysis because no way to look at how well it's working to debug why its wrong.

// Most important - robustness of resync. Do we lost 1 message, or dozens?  Can test for that. Parameter in SITL can set percentage byte loss
// 2 lost packets is ok, ideal is 1 if done properly.
// The messages all after the header go back into the original buffer.
// We can't do that with this style parser, there is no ability for history. See richenpower. See MoveHeaderInBuffer if crc doesn't parse.


// In SITL, there is no GSOF sim. We don't have that yet. Either need a sim, a replay log, or both
// See libraries/SITL/SIM_GPS.cpp line 1198

// Does GSOF have packet for what port am I on?
// Sims are one-way devices. They only produce the GPS messages

// TODO run the GSOF driver with valgrind. sim_vehicle.py -V & set gps type.

// Default with replay file is to loop

// GSOF is not run yet in autotest, but it could in tools/autotest/GPSTypes

// Great task - add replay mechanicsm to work with external AHRS
// Add an AHRS backend with replay to replay logs


#pragma once

#include "AP_GPS.h"
#include "GPS_Backend.h"

#if AP_GPS_GSOF_ENABLED
class AP_GPS_GSOF : public AP_GPS_Backend
{
public:
    AP_GPS_GSOF(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    AP_GPS::GPS_Status highest_supported_status(void) override WARN_IF_UNUSED {
        return AP_GPS::GPS_OK_FIX_3D_RTK_FIXED;
    }

    // Methods
    bool read() override WARN_IF_UNUSED;

    const char *name() const override { return "GSOF"; }

private:


    enum class State
    {
        STARTTX = 0,
        STATUS,
        PACKETTYPE,
        LENGTH,
        DATA,
        CHECKSUM,
        ENDTX
    };
    
    struct Msg_Parser
    {
        uint8_t status;
        uint8_t packettype;
        uint8_t length;
        uint8_t data[256];
        uint8_t checksum;
        uint8_t endtx;
        uint16_t read;
        uint8_t checksumcalc;
    } msg;

    Msg_Parser gsof_msg;

    bool parse(const uint8_t temp) WARN_IF_UNUSED;
    bool process_message(const Msg_Parser& msg, AP_GPS::state& state) WARN_IF_UNUSED;
    void requestBaud(const uint8_t portindex);
    void requestGSOF(const uint8_t messagetype, const uint8_t portindex);
    double SwapDouble(const uint8_t* src, const uint32_t pos) const WARN_IF_UNUSED;
    float SwapFloat(const uint8_t* src, const uint32_t pos) const WARN_IF_UNUSED;
    uint32_t SwapUint32(const uint8_t* src, const uint32_t pos) const WARN_IF_UNUSED;
    uint16_t SwapUint16(const uint8_t* src, const uint32_t pos) const WARN_IF_UNUSED;

    static const uint8_t STX = 0x02;
    static const uint8_t ETX = 0x03;

    uint8_t packetcount = 0;
    State state = State::STARTTX;
    uint32_t gsofmsg_time = 0;
    uint8_t gsofmsgreq_index = 0;
    uint8_t gsofmsgreq[5] = {1,2,8,9,12};
};
#endif
