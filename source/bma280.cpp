/*
 * Copyright (c) 2016, CATIE, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "bma280.h"

namespace sixtron {

#define BMA280_CHIP_ID                 0xFB /*!< BMA280 chip identification code */
#define BMA280_SWITCHED_TIME              5 /*!< wait time after a reset or changing power mode (in ms) */
#define BMA280_CALIBRATION_TIMEOUT      500 /*!< timeout of calibration (in ms) */

BMA280::BMA280(I2C * i2c, I2CAddress address, int hz):
        _i2cAddress(address), _range(Range::Range_2g)
{
    _i2c = i2c;
    _i2c->frequency(hz);
}

bool BMA280::initialize(Range range, Bandwidth bandwidth)
{
    char reg = 0;
    reset();
    i2c_read_register(RegisterAddress::ChipId, &reg);
    if (reg != BMA280_CHIP_ID) {
        wait_ms(20); //BMA280 may have not finishing to boot !
        i2c_read_register(RegisterAddress::ChipId, &reg);
        if (reg != BMA280_CHIP_ID) {
            return false;
        }
    }
    _chipId = reg;

    set_power_mode(PowerMode::PowerMode_NORMAL);
    set_range(range);
    set_bandwidth(bandwidth);

    return true;
}

bma280_accel_t BMA280::acceleration()
{
    static int16_t acc[3];
    double to_meters_per_seconds = 0;
    bma280_accel_t acceleration_values;

    i2c_read_vector(RegisterAddress::X_Axis_Lsb, acc);

    switch(_range) {
        case Range::Range_2g:
            to_meters_per_seconds = 0.244*9.80665/1000;
            break;
        case Range::Range_4g:
            to_meters_per_seconds = 0.488*9.80665/1000;
            break;
        case Range::Range_8g:
            to_meters_per_seconds = 0.977*9.80665/1000;
            break;
        case Range::Range_16g:
            to_meters_per_seconds = 1.953*9.80665/1000;
        break;
        default:
            break;
    }
    acceleration_values.x = static_cast<double>(acc[0] >> 2)*to_meters_per_seconds; //The 2 first bits are not data
    acceleration_values.y = static_cast<double>(acc[1] >> 2)*to_meters_per_seconds; //The 2 first bits are not data
    acceleration_values.z = static_cast<double>(acc[2] >> 2)*to_meters_per_seconds; //The 2 first bits are not data

    return acceleration_values;
}

double BMA280::acceleration_x()
{
    static int16_t acc;
    double to_meters_per_seconds = 0.0;
    double acceleration_x_value = 0.0;

    i2c_read_two_bytes_register(RegisterAddress::X_Axis_Lsb, &acc);

    switch(_range) {
        case Range::Range_2g:
            to_meters_per_seconds = 0.244*9.80665/1000;
            break;
        case Range::Range_4g:
            to_meters_per_seconds = 0.488*9.80665/1000;
            break;
        case Range::Range_8g:
            to_meters_per_seconds = 0.977*9.80665/1000;
            break;
        case Range::Range_16g:
            to_meters_per_seconds = 1.953*9.80665/1000;
        break;
        default:
            break;
    }
    acceleration_x_value = static_cast<double>(acc >> 2)*to_meters_per_seconds; //The 2 first bits are not data

    return acceleration_x_value;
}

double BMA280::acceleration_y()
{
    static int16_t acc;
    double to_meters_per_seconds = 0.0;
    double acceleration_y_value = 0.0;

    i2c_read_two_bytes_register(RegisterAddress::Y_Axis_Lsb, &acc);

    switch(_range) {
        case Range::Range_2g:
            to_meters_per_seconds = 0.244*9.80665/1000;
            break;
        case Range::Range_4g:
            to_meters_per_seconds = 0.488*9.80665/1000;
            break;
        case Range::Range_8g:
            to_meters_per_seconds = 0.977*9.80665/1000;
            break;
        case Range::Range_16g:
            to_meters_per_seconds = 1.953*9.80665/1000;
        break;
        default:
            break;
    }
    acceleration_y_value = static_cast<double>(acc >> 2)*to_meters_per_seconds; //The 2 first bits are not data

    return acceleration_y_value;
}

double BMA280::acceleration_z()
{
    static int16_t acc;
    double to_meters_per_seconds = 0.0;
    double acceleration_z_value = 0.0;

    i2c_read_two_bytes_register(RegisterAddress::Z_Axis_Lsb, &acc);

    switch(_range) {
        case Range::Range_2g:
            to_meters_per_seconds = 0.244*9.80665/1000;
            break;
        case Range::Range_4g:
            to_meters_per_seconds = 0.488*9.80665/1000;
            break;
        case Range::Range_8g:
            to_meters_per_seconds = 0.977*9.80665/1000;
            break;
        case Range::Range_16g:
            to_meters_per_seconds = 1.953*9.80665/1000;
        break;
        default:
            break;
    }
    acceleration_z_value = static_cast<double>(acc >> 2)*to_meters_per_seconds; //The 2 first bits are not data

    return acceleration_z_value;
}
double BMA280::temperature()
{
    char data;
    double temperature_value = 0.0;

    i2c_read_register(RegisterAddress::Temp, &data);

    temperature_value = static_cast<double>(data*0.5 + 23.0);

    return temperature_value;

}

/** read the 3 axis offsets. 1 LSB = 1/128 m/s²
 *
 * @param offsets pointer to the offset structure to store the read values
 *
 */
bma280_offset_t BMA280::offsets()
{
    char data[3];
    bma280_offset_t offset_values;

    i2c_read_register(RegisterAddress::Offset_X_Axis, data);
    i2c_read_register(RegisterAddress::Offset_Y_Axis, data+1);
    i2c_read_register(RegisterAddress::Offset_Z_Axis, data+2);

    offset_values.x = data[0];
    offset_values.y = data[1];
    offset_values.z = data[2];

    return offset_values;
}

/** write the 3 axis offsets. 1 LSB = 1/128 m/s²
 *
 * @param offsets pointer to the offset structure to write
 *
 */
void BMA280::set_offsets(bma280_offset_t* offsets)
{
    char data[3];
    data[0] = offsets->x;
    data[1] = offsets->y;
    data[2] = offsets->z;

    i2c_set_register(RegisterAddress::Offset_X_Axis, data[0]);
    i2c_set_register(RegisterAddress::Offset_Y_Axis, data[1]);
    i2c_set_register(RegisterAddress::Offset_Z_Axis, data[2]);
}

void BMA280::set_power_mode(PowerMode mode)
{
    char data;
    i2c_read_register(RegisterAddress::ModeCtrl, &data);

    data = (data & 0x1F) | (static_cast<char>(mode) << 5);
    i2c_set_register(RegisterAddress::ModeCtrl, data);
    wait_ms(BMA280_SWITCHED_TIME);
}

void BMA280::set_range(Range range)
{
    char data = static_cast<char>(range);
    if((i2c_set_register(RegisterAddress::RangeSelect, data)) == 0) {
        _range = range;
    }
}

void BMA280::set_bandwidth(Bandwidth bandwidth)
{
    char data = static_cast<char>(bandwidth);
    i2c_set_register(RegisterAddress::BwSelect, data);
}

void BMA280::enable_slow_offset_compensation(bool x_axis, bool y_axis, bool z_axis)
{
    char data = ((z_axis & 0x01) << 2) | ((y_axis & 0x01) << 1) | (x_axis & 0x01);
    i2c_set_register(RegisterAddress::OffsetCtrl, data);
}

bool BMA280::set_fast_offsets_calibration_x(OffsetTarget target)
{
    bool calibration_done = false;
    Range old_range = _range;
    char data;
    int timeout = BMA280_CALIBRATION_TIMEOUT;

    set_range(Range::Range_2g); // Sensor need to be in 2g range to perform fast calibration
    i2c_set_register(RegisterAddress::OffsetCtrl, 0x01); // Perform fast calibration on X axis

    while ((calibration_done != true) && (timeout > 0)) {
        i2c_read_register(RegisterAddress::OffsetCtrl, &data);
        if ((data & 0x10) == 0x10) {
            calibration_done = true;
        }
        wait_ms(1);
        timeout--;
    }
    set_range(old_range);

    return calibration_done;
}

bool BMA280::set_fast_offsets_calibration_y(OffsetTarget target)
{
    bool calibration_done = false;
    Range old_range = _range;
    char data;
    int timeout = BMA280_CALIBRATION_TIMEOUT;

    set_range(Range::Range_2g); // Sensor need to be in 2g range to perform fast calibration
    i2c_set_register(RegisterAddress::OffsetCtrl, 0x02); // Perform fast calibration on Y axis

    while ((calibration_done != true) && (timeout > 0)) {
        i2c_read_register(RegisterAddress::OffsetCtrl, &data);
        if ((data & 0x10) == 0x10) {
            calibration_done = true;
        }
        wait_ms(1);
        timeout--;
    }
    set_range(old_range);

    return calibration_done;
}

bool BMA280::set_fast_offsets_calibration_z(OffsetTarget target)
{
    bool calibration_done = false;
    Range old_range = _range;
    char data;
    int timeout = BMA280_CALIBRATION_TIMEOUT;

    set_range(Range::Range_2g); // Sensor need to be in 2g range to perform fast calibration
    i2c_set_register(RegisterAddress::OffsetCtrl, 0x03); // Perform fast calibration on Z axis

    while ((calibration_done != true) && (timeout > 0)) {
        i2c_read_register(RegisterAddress::OffsetCtrl, &data);
        if ((data & 0x10) == 0x10) {
            calibration_done = true;
        }
        wait_ms(1);
        timeout--;
    }
    set_range(old_range);

    return calibration_done;
}


void BMA280::reset()
{
    i2c_set_register(RegisterAddress::Rst, 0xB6);
    wait_ms(BMA280_SWITCHED_TIME);
}

char BMA280::chip_id()
{
    return _chipId;
}

void BMA280::set_sleep_duration(SleepDuration sleep_duration)
{
    char register_value = 0x00;

    // read current value register
    i2c_read_register(RegisterAddress::ModeCtrl, &register_value);
    // clear and set new value
    register_value &= 0xE0;
    register_value |= (static_cast<char>(sleep_duration) << 1);
    // set new value
    i2c_set_register(RegisterAddress::ModeCtrl, register_value);
}

/** Set register value
 *
 * @param registerAddress register address
 * @param value value to write
 *
 * @returns
 *      O on success,
 *      non-0 on failure
 */
int BMA280::i2c_set_register(RegisterAddress registerAddress, char value)
{
    char data[2];
    data[0] = static_cast<char>(registerAddress);
    data[1] = value;
    if (_i2c->write(static_cast<int>(_i2cAddress) << 1, data, 2, false) != 0) {
        return -1;
    }
    return 0;
}

/** Get register value
 *
 * @param registerAddress register address
 * @param value pointer to store read value to
 *
 * @returns
 *      O on success,
 *      non-0 on failure
 */
int BMA280::i2c_read_register(RegisterAddress registerAddress, char *value)
{
    char data = static_cast<char>(registerAddress);
    if (_i2c->write(static_cast<int>(_i2cAddress) << 1, &data, 1, true) != 0) {
        return -1;
    }
    if (_i2c->read(static_cast<int>(_i2cAddress) << 1, value, 1, false) != 0) {
        return -2;
    }
    return 0;
}

/** Get multi-byte register value (two-bytes)
 *
 * @param registerAddress register address of LSB
 * @param value pointer to store read value to
 *
 * @returns
 *      O on success,
 *      non-0 on failure
 */
int BMA280::i2c_read_two_bytes_register(RegisterAddress registerAddress, int16_t *value)
{
    static char data[2];
    data[0] = static_cast<char>(registerAddress);
    if (_i2c->write(static_cast<int>(_i2cAddress) << 1, data, 1, true) != 0) {
        return -1;
    }
    if (_i2c->read(static_cast<int>(_i2cAddress) << 1, data, 2, false) != 0) {
        return -2;
    }
    *value = (data[1] << 8) | (0xFF & data[0]);

    return 0;
}

/** Get multi-byte register value (3*2-bytes) that are stored in a 3 dimensions vector
 *
 * @param registerAddress register address of LSB
 * @param value pointer to store read value to
 *
 * @returns
 *      O on success,
 *      non-0 on failure
 */
int BMA280::i2c_read_vector(RegisterAddress registerAddress, int16_t value[3])
{
    static char data[6];
    data[0] = static_cast<char>(registerAddress);
    if (_i2c->write(static_cast<int>(_i2cAddress) << 1, data, 1, true) != 0) {
        return -1;
    }
    if (_i2c->read(static_cast<int>(_i2cAddress) << 1, data, 6, false) != 0) {
        return -2;
    }
    value[0] = (data[1] << 8) | (0xFF & data[0]);
    value[1] = (data[3] << 8) | (0xFF & data[2]);
    value[2] = (data[5] << 8) | (0xFF & data[4]);

    return 0;
}

} // namespace sixtron
