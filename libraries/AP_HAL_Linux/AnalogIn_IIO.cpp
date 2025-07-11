#include "AnalogIn_IIO.h"

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL &hal;

const char* AnalogSource_IIO::analog_sources[] = {
    "in_voltage0_raw",
    "in_voltage1_raw",
    "in_voltage2_raw",
    "in_voltage3_raw",
    "in_voltage4_raw",
    "in_voltage5_raw",
    "in_voltage6_raw",
    "in_voltage7_raw",
};

AnalogSource_IIO::AnalogSource_IIO(int16_t pin, float initial_value, float voltage_scaling) :
    _value(initial_value),
    _sum_value(0),
    _voltage_scaling(voltage_scaling),
    _sum_count(0),
    _pin(pin),
    _pin_fd(-1)
{
    init_pins();
    select_pin();
}

void AnalogSource_IIO::init_pins(void)
{
    static_assert(ARRAY_SIZE(AnalogSource_IIO::analog_sources) == ARRAY_SIZE(fd_analog_sources), "AnalogIn_IIO channels count mismatch");

    char buf[100];
    for (unsigned int i = 0; i < ARRAY_SIZE(AnalogSource_IIO::analog_sources); i++) {
        // Construct the path by appending strings
        strncpy(buf, IIO_ANALOG_IN_DIR, sizeof(buf));
        strncat(buf, AnalogSource_IIO::analog_sources[i], sizeof(buf) - strlen(buf) - 1);

        fd_analog_sources[i] = open(buf, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    }
}

/*
  selects a different file descriptor among in the fd_analog_sources array
 */
void AnalogSource_IIO::select_pin(void)
{
    if (0 <= _pin && (size_t)_pin < ARRAY_SIZE(fd_analog_sources)) {
        _pin_fd = fd_analog_sources[_pin];
    } else {
        _pin_fd = -1;
    }
}

float AnalogSource_IIO::read_average()
{
    read_latest();
    WITH_SEMAPHORE(_semaphore);

    if (_sum_count == 0) {
        return _value;
    }

    _value = _sum_value / _sum_count;
    _sum_value = 0;
    _sum_count = 0;

    return _value;
}

float AnalogSource_IIO::read_latest()
{
    char sbuf[10];

    if (_pin_fd == -1) {
        _latest = 0;
        return 0;
    }

    memset(sbuf, 0, sizeof(sbuf));
    if (pread(_pin_fd, sbuf, sizeof(sbuf) - 1, 0) < 0) {
        _latest = 0;
        return 0;
    }
    WITH_SEMAPHORE(_semaphore);

    _latest = atoi(sbuf) * _voltage_scaling;
    _sum_value += _latest;
    _sum_count++;

    return _latest;
}

// output is in volts
float AnalogSource_IIO::voltage_average()
{
    return read_average();
}

float AnalogSource_IIO::voltage_latest()
{
    read_latest();
    return _latest;
}

bool AnalogSource_IIO::set_pin(uint8_t pin)
{
    if (_pin == pin) {
        return true;
    }

    WITH_SEMAPHORE(_semaphore);

    _pin = pin;
    _sum_value = 0;
    _sum_count = 0;
    _latest = 0;
    _value = 0;
    select_pin();
    return true;
}

AnalogIn_IIO::AnalogIn_IIO()
{}

void AnalogIn_IIO::init()
{}


AP_HAL::AnalogSource* AnalogIn_IIO::channel(int16_t pin) {
    return NEW_NOTHROW AnalogSource_IIO(pin, 0.0f, AP_HAL_LINUX_ANALOGIN_IIO_VOLTAGE_SCALING);
}
