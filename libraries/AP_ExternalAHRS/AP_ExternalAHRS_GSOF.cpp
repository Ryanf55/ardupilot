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
#include <AP_SerialManager/AP_SerialManager.h>

static const char* LOG_FMT = "%s ExternalAHRS: %s";

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_GSOF::AP_ExternalAHRS_GSOF(AP_ExternalAHRS *_frontend,
        AP_ExternalAHRS::state_t &_state): AP_ExternalAHRS_backend(_frontend, _state)
{
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);

    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    if (!uart) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, LOG_FMT, get_name(), "no UART");
        return;
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_GSOF::update_thread, void), "AHRS", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_BoardConfig::allocation_error("GSOF ExternalAHRS failed to allocate ExternalAHRS update thread");
    }

    // don't offer IMU by default, at 100Hz it is too slow for many aircraft
    set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::GPS) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::BARO) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::COMPASS));

    hal.scheduler->delay(5000);
    if (!initialised()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, LOG_FMT, get_name(), "failed to initialise.");
    }
}

void AP_ExternalAHRS_GSOF::update_thread(void)
{
    auto& network = AP::network();
    [[maybe_unused]] const char *dest_ip = param.remote_ip.get_str();
    network.startup_wait();
    if (!port_open) {
        port_open = true;
        uart->begin(baudrate);
    }

    while (true) {
        // TODO read data
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


// Builds packets by looking at each individual byte, once a full packet has been read in it checks the checksum then handles the packet.
void AP_ExternalAHRS_GSOF::build_packet()
{
    if (uart == nullptr) {
        return;
    }

    WITH_SEMAPHORE(sem);
    uint32_t nbytes = MIN(uart->available(), 2048u);
    while (nbytes--> 0) {
        uint8_t b;
        if (!uart->read(b)) {
            break;
        }
        // TODO handle bytes
    }
}

void AP_ExternalAHRS_GSOF::post_filter() const
{

}

int8_t AP_ExternalAHRS_GSOF::get_port(void) const
{
    if (!uart) {
        return -1;
    }
    return port_num;
};

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
    const bool got_packets = last_gsof49_time != 0 && last_gsof50_time != 0;
    return got_packets;
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
}

void AP_ExternalAHRS_GSOF::send_status_report(GCS_MAVLINK &link) const
{
    // TODO
}

bool AP_ExternalAHRS_GSOF::times_healthy() const
{
    // uint32_t now = AP_HAL::millis();
    const bool times_healthy = false; // TODO

    return times_healthy;
}

bool AP_ExternalAHRS_GSOF::filter_healthy() const
{
    // TODO
    return false;
}

#endif // AP_EXTERNAL_AHRS_GSOF_ENABLED
