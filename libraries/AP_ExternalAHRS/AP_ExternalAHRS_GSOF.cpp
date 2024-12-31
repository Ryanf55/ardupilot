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

//   Usage in SITL with hardware for debugging:
//     $ sim_vehicle.py -v Plane --console --map -DG
//     param set AHRS_EKF_TYPE 11
//     param set EAHRS_TYPE 6
//

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_GSOF_ENABLED

#include "AP_ExternalAHRS_GSOF.h"
#include "AP_Compass/AP_Compass_config.h"
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/utility/Socket.h>

static const char* LOG_FMT = "%s ExternalAHRS: %s";

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_GSOF::AP_ExternalAHRS_GSOF(AP_ExternalAHRS *_frontend,
        AP_ExternalAHRS::state_t &_state): AP_ExternalAHRS_backend(_frontend, _state)
{

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_GSOF::update_thread, void), "AHRS", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_BoardConfig::allocation_error("GSOF ExternalAHRS failed to allocate ExternalAHRS update thread");
    }

    // Offer GPS even through it's a tightly coupled EKF.
    set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::GPS));

    hal.scheduler->delay(5000);
    if (!initialised()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, LOG_FMT, get_name(), "failed to initialise.");
    }
}

void AP_ExternalAHRS_GSOF::update_thread(void)
{
    auto& network = AP::network();
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, LOG_FMT, get_name(), "starting network");
    network.startup_wait();
    // const char *dest_ip = param.remote_ip.get_str();
    auto *sock = new SocketAPM(true);
    if (sock == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, LOG_FMT, get_name(), "failed to create socket");
        return;
    }

    // const char* data = "foo";
    // Can use sendto to send config data to address without binding:
    // sock->sendto((const void*)data, strlen(data), dest_ip, 44448);
    
    sock->bind("0.0.0.0", 44444);
    uint8_t data[AP_GSOF::MAX_PACKET_SIZE];

    AP_GSOF::MsgTypes expected;
    expected.set(AP_GSOF::POS_TIME);
    expected.set(AP_GSOF::INS_FULL_NAV);
    expected.set(AP_GSOF::INS_RMS);
    expected.set(AP_GSOF::LLH_MSL);
    // TODO configure receiver to output expected data.
    
    while (true) {
        if (sock->pollin(1)) {
            auto const recv_res = sock->recv(data, AP_GSOF::MAX_PACKET_SIZE, 1000);
            if (recv_res != -1) {
                AP_GSOF::MsgTypes parsed;
                const auto parse_res = parse_buf(data, recv_res, parsed);
                if (parse_res != PARSED_GSOF_DATA) {
                    continue;
                }

                auto const now = AP_HAL::millis();

                if (parsed.get(AP_GSOF::POS_TIME)) {
                    last_pos_time_ms = now;

                    gps_data.satellites_in_view = pos_time.num_sats;
                    gps_data.fix_type = AP_GSOF::pos_flags_to_fix_type(pos_time.pos_flags1, pos_time.pos_flags2);
                }

                if (parsed.get(AP_GSOF::INS_FULL_NAV)) {
                    last_ins_full_nav_ms = now;

                    gps_data.gps_week = ins_full_nav.gps_week;
                    gps_data.ms_tow = ins_full_nav.gps_time_ms;
                    gps_data.ned_vel_north = ins_full_nav.vel_n;
                    gps_data.ned_vel_east = ins_full_nav.vel_e;
                    gps_data.ned_vel_down = ins_full_nav.vel_d;
                }
                if (parsed.get(AP_GSOF::INS_RMS)) {
                    last_ins_rms_ms = now;

                    gps_data.horizontal_pos_accuracy = Vector2d(ins_rms.pos_rms_n, ins_rms.pos_rms_e).length();
                    gps_data.vertical_pos_accuracy = ins_rms.pos_rms_d;
                    gps_data.horizontal_vel_accuracy = Vector2d(ins_rms.vel_rms_n, ins_rms.vel_rms_e).length();


                }
                if (parsed.get(AP_GSOF::LLH_MSL)) {
                    last_llh_msl_ms = now;

                    gps_data.longitude = static_cast<int32_t>(llh_msl.longitude * 1E-7);
                    gps_data.latitude = static_cast<int32_t>(llh_msl.latitude * 1E-7);
                    gps_data.msl_altitude = static_cast<int32_t>(llh_msl.altitude_msl * 1E-2);
                    
                }

                // TODO only send GNSS data if we got what we needed.
                uint8_t instance;
                if (AP::gps().get_first_external_instance(instance)) {
                    AP::gps().handle_external(gps_data, instance);
                }
                
            }
        }
  
        hal.scheduler->delay_microseconds(100);
        check_initialise_state();
    }
}

void AP_ExternalAHRS_GSOF::check_initialise_state(void)
{
    const auto new_init_state = initialised();
    // Only send the message after fully booted up, otherwise it gets dropped.
    if (!last_init_state && new_init_state && AP_HAL::millis() > 5000) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, LOG_FMT, get_name(), "initialised.");
        last_init_state = new_init_state;
    }
}

void AP_ExternalAHRS_GSOF::post_filter() const
{
    WITH_SEMAPHORE(state.sem);
    state.velocity = Vector3f{ins_full_nav.vel_n, ins_full_nav.vel_e, ins_full_nav.vel_d};
    state.have_velocity = true;

    // TODO check the altitude datum conversion.
    state.location = Location(ins_full_nav.latitude * 1E7, ins_full_nav.longitude * 1E7, ins_full_nav.altitude * 1E2, Location::AltFrame::ABSOLUTE);
    state.have_location = true;
}

// Get model/type name
const char* AP_ExternalAHRS_GSOF::get_name() const
{
    return "GSOF";
}

bool AP_ExternalAHRS_GSOF::healthy(void) const
{
    return times_healthy() && filter_healthy();
}

bool AP_ExternalAHRS_GSOF::initialised(void) const
{
    return last_ins_full_nav_ms != 0 && last_ins_rms_ms != 0 && last_llh_msl_ms != 0;
}

bool AP_ExternalAHRS_GSOF::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!initialised()) {
        hal.util->snprintf(failure_msg, failure_msg_len, LOG_FMT, get_name(), "not initialised");
        return false;
    }
    if (!times_healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, LOG_FMT, get_name(), "data is stale");
        return false;
    }
    if (!filter_healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, LOG_FMT, get_name(), "filter is unhealthy");
        return false;
    }
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, LOG_FMT, get_name(), "unhealthy");
        return false;
    }

    return true;
}

void AP_ExternalAHRS_GSOF::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));
    status.flags.initalized = initialised();
}

bool AP_ExternalAHRS_GSOF::times_healthy() const
{
    auto const now = AP_HAL::millis();
    // 100Hz = 10mS.
    auto const ins_full_nav_healthy = now - last_ins_full_nav_ms <= 2 * 10;
    if (!ins_full_nav_healthy) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s: INS Full nav delayed by %u ms", get_name(), now - last_ins_full_nav_ms);
        // hal.util->snprintf("INS FULL NAV TIME: %u", 18, now - last_ins_full_nav_ms);
    }
    // 5Hz = 200mS.
    auto const ins_rms_healthy = now - last_ins_rms_ms < 2 * 200;
    // 100Hz = 10mS.
    auto const llh_msl_healthy = now - last_llh_msl_ms < 2 * 10;

    return ins_full_nav_healthy && ins_rms_healthy && llh_msl_healthy;
}

bool AP_ExternalAHRS_GSOF::filter_healthy() const
{
    // TODO get the right threshold from Trimble.
    return ins_rms.gnss_quality >= 1;
}

#endif // AP_EXTERNAL_AHRS_GSOF_ENABLED
