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

BMA280::BMA280(I2C * i2c, I2CAddress address, int hz):
		_i2cAddress(address), _range(Range::Range_2g)
{
	_i2c = i2c;
    _i2c->frequency(hz);
}

bool BMA280::initialize(Range range, Bandwidth width)
{
	char reg = 0;
	printf("Initializing BMA280 ... \n");
	reset();
	i2c_read_register(RegisterAddress::ChipId, &reg);
	if (reg != 0XFB) {
		wait_ms(20); //BMA280 may have not finishing to boot !
		i2c_read_register(RegisterAddress::ChipId, &reg);
		if (reg != 0XFB) {
			return false;
		}
	}
	_chipId = reg;

	set_power_mode(PowerMode::PowerMode_NORMAL);
	set_range(range);
	set_bandwidth(width);

	return true;
}

void BMA280::read_accel(bma280_accel_t* accel)
{
	static int16_t acc[3];
	i2c_read_vector(RegisterAddress::X_Axis_Lsb, acc);

	double to_meters_per_seconds = 0;
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
	accel->x = static_cast<double>(acc[0] >> 2)*to_meters_per_seconds; //The 2 first bits are not data
	accel->y = static_cast<double>(acc[1] >> 2)*to_meters_per_seconds; //The 2 first bits are not data
	accel->z = static_cast<double>(acc[2] >> 2)*to_meters_per_seconds; //The 2 first bits are not data
}

void BMA280::read_X_accel(double* accel_x)
{
	static int16_t acc;
	i2c_read_two_bytes_register(RegisterAddress::X_Axis_Lsb, &acc);

	double to_meters_per_seconds = 0;
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
	*accel_x = static_cast<double>(acc >> 2)*to_meters_per_seconds; //The 2 first bits are not data
}

void BMA280::read_Y_accel(double* accel_y)
{
	static int16_t acc;
	i2c_read_two_bytes_register(RegisterAddress::Y_Axis_Lsb, &acc);

	double to_meters_per_seconds = 0;
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
	*accel_y = static_cast<double>(acc >> 2)*to_meters_per_seconds; //The 2 first bits are not data
}

void BMA280::read_Z_accel(double* accel_z)
{
	static int16_t acc;
	i2c_read_two_bytes_register(RegisterAddress::Z_Axis_Lsb, &acc);

	double to_meters_per_seconds = 0;
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
	*accel_z = static_cast<double>(acc >> 2)*to_meters_per_seconds; //The 2 first bits are not data
}
void BMA280::read_temperature(float* temperature)
{
	char data;
	i2c_read_register(RegisterAddress::Temp, &data);

	*temperature = static_cast<float>(data*0.5 + 23.0);
}

void BMA280::set_power_mode(PowerMode mode)
{
	char data;
	i2c_read_register(RegisterAddress::ModeCtrl, &data);

	data = (data & 0x1F) | (static_cast<char>(mode) << 5);
    i2c_set_register(RegisterAddress::ModeCtrl, data);
    wait_ms(5);
}

void BMA280::set_range(Range range)
{
	char data = static_cast<char>(range);
	i2c_set_register(RegisterAddress::RangeSelect, data);
}

void BMA280::set_bandwidth(Bandwidth width)
{
	char data = static_cast<char>(width);
	i2c_set_register(RegisterAddress::BwSelect, data);
}

void BMA280::enable_slow_offset_compensation(bool x_axis, bool y_axis, bool z_axis)
{
	char data = ((z_axis & 0x01) << 2) | ((y_axis & 0x01) << 1) | (x_axis & 0x01);
	i2c_set_register(RegisterAddress::OffsetCtrl, data);
}
void BMA280::reset()
{
	i2c_set_register(RegisterAddress::Rst, 0xB6);
	wait_ms(5);
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
