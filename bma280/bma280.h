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
#ifndef CATIE_SIXTRON_LIBTEMPLATE_H_
#define CATIE_SIXTRON_LIBTEMPLATE_H_

#include "mbed.h"

namespace sixtron {

/*!
 * @brief Accelerometer values (X, Y, Z) in m/s²
 */
typedef struct
{
    double x;
    double y;
    double z;
} bma280_accel_t;

/*!
 * @brief Accelerometer 3 axes (X, Y, Z) offset data
 */
typedef struct {
    int8_t x;
    int8_t y;
    int8_t z;
} bma280_offset_t;


class BMA280
{
public:
    enum class I2CAddress {
        Address1 = 0x18,
        Address2 = 0x19
    };

    enum class RegisterAddress : char {
        ChipId                      =    (0x00),
        /** DATA ADDRESS DEFINITIONS */
        X_Axis_Lsb                  =    (0x02),
        X_Axis_Msb                  =    (0x03),
        Y_Axis_Lsb                  =    (0x04),
        Y_Axis_Msb                  =    (0x05),
        Z_Axis_Lsb                  =    (0x06),
        Z_Axis_Msb                  =    (0x07),
        Temp                        =    (0x08),
        /**STATUS ADDRESS DEFINITIONS */
        Stat1                       =    (0x09),
        Stat2                       =    (0x0A),
        StatTapSlop                 =    (0x0B),
        StatOrientHigh              =    (0x0C),
        StatFifo                    =    (0x0E),
        /**STATUS ADDRESS DEFINITIONS */
        RangeSelect                 =    (0x0F),
        BwSelect                    =    (0x10),
        ModeCtrl                    =    (0x11),
        LowNoiseCtrl                =    (0x12),
        DataCtrl                    =    (0x13),
        Rst                         =    (0x14),
        /**INTERUPT ADDRESS DEFINITIONS */
        IntrEnable1                 =    (0x16),
        IntrEnable2                 =    (0x17),
        IntrSlowNoMotion            =    (0x18),
        Intr1PadSelect              =    (0x19),
        IntrDataSelect              =    (0x1A),
        Intr2PadSelect              =    (0x1B),
        IntrSource                  =    (0x1E),
        IntrSet                     =    (0x20),
        IntrCtrl                    =    (0x21),
        /** FEATURE ADDRESS DEFINITIONS */
        Low_Durn                    =    (0x22),
        Low_Thres                   =    (0x23),
        Low_High_Hyst               =    (0x24),
        High_Durn                   =    (0x25),
        High_Thres                  =    (0x26),
        Slope_Durn                  =    (0x27),
        Slope_Thres                 =    (0x28),
        SlowNoMotionThres           =    (0x29),
        TapParam                    =    (0x2A),
        TapThres                    =    (0x2B),
        OrientParam                 =    (0x2C),
        ThetaBlock                  =    (0x2D),
        ThetaFlat                   =    (0x2E),
        FlatHoldTime                =    (0x2F),
        SelfTest                    =    (0x32),
        EepromCtrl                  =    (0x33),
        SerialCtrl                  =    (0x34),
        /**OFFSET ADDRESS DEFINITIONS */
        OffsetCtrl                  =    (0x36),
        OffsetParams                =    (0x37),
        Offset_X_Axis               =    (0x38),
        Offset_Y_Axis               =    (0x39),
        Offset_Z_Axis               =    (0x3A),
        /**GP ADDRESS DEFINITIONS */
        Gp0                         =    (0x3B),
        Gp1                         =    (0x3C),
        /**FIFO ADDRESS DEFINITIONS */
        FifoMode                    =    (0x3E),
        FifoDataOutput              =    (0x3F),
        FifoWmlTrig                 =    (0x30),
    };

    enum class PowerMode: char {
        PowerMode_NORMAL       	 = (0X00),
        PowerMode_DEEP_SUSPEND   = (0X01),
        PowerMode_LOW_POWER      = (0X02),
        PowerMode_SUSPEND        = (0x04)
    };

    enum class Range: char {
        Range_2g        = (0X03),
        Range_4g        = (0X05),
        Range_8g        = (0X08),
        Range_16g       = (0x0C)
    };

    enum class Bandwidth: char {
        Bandwidth_7_81_Hz      = (0x08),
        Bandwidth_15_63_Hz     = (0x09),
        Bandwidth_31_25_Hz     = (0x0A),
        Bandwidth_62_25_Hz     = (0x0B),
        Bandwidth_125_Hz       = (0x0C),
        Bandwidth_250_Hz       = (0x0D),
        Bandwidth_500_Hz       = (0x0E),
        Bandwidth_1000_Hz      = (0x0F)
    };

    enum class OffsetTarget: char {
        Target_0g           = (0x00),
        Target_1g           = (0x01),
        Target_Minus_1g     = (0x02)
    };

    BMA280(I2C * i2c, I2CAddress address = I2CAddress::Address1, int hz = 400000);

    bool initialize(Range range = Range::Range_2g, Bandwidth width = Bandwidth::Bandwidth_1000_Hz);
    void set_power_mode(PowerMode mode);
    void set_range(Range range);
    void set_bandwidth(Bandwidth width);
    // TODO configure sleep duration for low power modes
    // TODO configure interrupt

    void read_accel(bma280_accel_t* accel);
    void read_X_accel(double* accel_x);
    void read_Y_accel(double* accel_y);
    void read_Z_accel(double* accel_z);
    void read_temperature(float* temperature);

    void read_offsets(bma280_offset_t* offsets);
    void set_offsets(bma280_offset_t* offsets);
    void enable_slow_offset_compensation(bool x_axis, bool y_axis, bool z_axis);
    void fast_offsets_calibration_X(OffsetTarget target);
    void fast_offsets_calibration_Y(OffsetTarget target);
    void fast_offsets_calibration_Z(OffsetTarget target);

    char chip_id() { return _chipId; }

private:

    int i2c_set_register(RegisterAddress registerAddress, char value);
    int i2c_read_register(RegisterAddress registerAddress, char *value);
    int i2c_read_two_bytes_register(RegisterAddress registerAddress, int16_t *value);
    int i2c_read_vector(RegisterAddress registerAddress, int16_t value[3]);
    void reset();

    I2C* _i2c;
    I2CAddress _i2cAddress;
    Range _range;
    char _chipId = 0;
};


} // namespace sixtron

#endif // CATIE_SIXTRON_LIBTEMPLATE_H_
