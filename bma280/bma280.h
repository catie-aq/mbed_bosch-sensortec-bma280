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
 * \brief Accelerometer values (X, Y, Z) in m/s²
 */
typedef struct {
    double x;
    double y;
    double z;
} bma280_acceleration_t;

/*!
 * \brief Accelerometer 3 axes (X, Y, Z) offset data
 */
typedef struct {
    int8_t x;
    int8_t y;
    int8_t z;
} bma280_offset_t;

/*!
 *  \class BMA280
 *  BMA280 accelerometer driver
 */
class BMA280
{
public:
    enum class I2CAddress {
        Address1 = 0x18,
        Address2 = 0x19
    };

    enum class RegisterAddress : char {
        ChipId                      = (0x00),
        /** DATA ADDRESS DEFINITIONS */
        X_Axis_Lsb                  = (0x02),
        X_Axis_Msb                  = (0x03),
        Y_Axis_Lsb                  = (0x04),
        Y_Axis_Msb                  = (0x05),
        Z_Axis_Lsb                  = (0x06),
        Z_Axis_Msb                  = (0x07),
        Temp                        = (0x08),
        /**STATUS ADDRESS DEFINITIONS */
        Stat1                       = (0x09),
        Stat2                       = (0x0A),
        StatTapSlop                 = (0x0B),
        StatOrientHigh              = (0x0C),
        StatFifo                    = (0x0E),
        /**STATUS ADDRESS DEFINITIONS */
        RangeSelect                 = (0x0F),
        BwSelect                    = (0x10),
        ModeCtrl                    = (0x11),
        LowNoiseCtrl                = (0x12),
        DataCtrl                    = (0x13),
        Rst                         = (0x14),
        /**INTERUPT ADDRESS DEFINITIONS */
        IntrEnable1                 = (0x16),
        IntrEnable2                 = (0x17),
        IntrSlowNoMotion            = (0x18),
        Intr1PadSelect              = (0x19),
        IntrDataSelect              = (0x1A),
        Intr2PadSelect              = (0x1B),
        IntrSource                  = (0x1E),
        IntrSet                     = (0x20),
        IntrCtrl                    = (0x21),
        /** FEATURE ADDRESS DEFINITIONS */
        Low_Durn                    = (0x22),
        Low_Thres                   = (0x23),
        Low_High_Hyst               = (0x24),
        High_Durn                   = (0x25),
        High_Thres                  = (0x26),
        Slope_Durn                  = (0x27),
        Slope_Thres                 = (0x28),
        SlowNoMotionThres           = (0x29),
        TapParam                    = (0x2A),
        TapThres                    = (0x2B),
        OrientParam                 = (0x2C),
        ThetaBlock                  = (0x2D),
        ThetaFlat                   = (0x2E),
        FlatHoldTime                = (0x2F),
        SelfTest                    = (0x32),
        EepromCtrl                  = (0x33),
        SerialCtrl                  = (0x34),
        /**OFFSET ADDRESS DEFINITIONS */
        OffsetCtrl                  = (0x36),
        OffsetParams                = (0x37),
        Offset_X_Axis               = (0x38),
        Offset_Y_Axis               = (0x39),
        Offset_Z_Axis               = (0x3A),
        /**GP ADDRESS DEFINITIONS */
        Gp0                         = (0x3B),
        Gp1                         = (0x3C),
        /**FIFO ADDRESS DEFINITIONS */
        FifoMode                    = (0x3E),
        FifoDataOutput              = (0x3F),
        FifoWmlTrig                 = (0x30),
    };

    enum class PowerMode : char {
        PowerMode_NORMAL         = (0X00),
        PowerMode_DEEP_SUSPEND   = (0X01),
        PowerMode_LOW_POWER      = (0X02),
        PowerMode_SUSPEND        = (0x04)
    };

    enum class Range : char {
        Range_2g        = (0X03),
        Range_4g        = (0X05),
        Range_8g        = (0X08),
        Range_16g       = (0x0C)
    };

    enum class Bandwidth : char {
        Bandwidth_7_81_Hz      = (0x08),
        Bandwidth_15_63_Hz     = (0x09),
        Bandwidth_31_25_Hz     = (0x0A),
        Bandwidth_62_50_Hz     = (0x0B),
        Bandwidth_125_Hz       = (0x0C),
        Bandwidth_250_Hz       = (0x0D),
        Bandwidth_500_Hz       = (0x0E),
        Bandwidth_1000_Hz      = (0x0F)
    };

    enum class OffsetTarget : char {
        Target_0g           = (0x00),
        Target_1g           = (0x01),
        Target_Minus_1g     = (0x02)
    };

    enum class SleepDuration : char {
        _500_US             = (0x05),
        _1_MS               = (0x06),
        _2_MS               = (0x07),
        _4_MS               = (0x08),
        _6_MS               = (0x09),
        _10_MS              = (0x0A),
        _25_MS              = (0x0B),
        _50_MS              = (0x0C),
        _100_MS             = (0x0D),
        _500_MS             = (0x0E),
        _1000_MS            = (0x0F)
    };

    /*! Constructor
     *
     * \param i2c pointer to mbed I2C object
     * \param i2c_address i2c address of the bma280
     * \param i2c_frequency frequency of the I2C interface
     *
     */
    BMA280(I2C *i2c, I2CAddress i2c_address = I2CAddress::Address1, int i2c_frequency = 400000);

    /*!
     * \brief Initialize the device
     *
     * \param range acceleration ranges 2G/4G/8G/16G
     * \param bandwidth Low-pass filter bandwidths 500Hz - <8Hz
     *
     * \return true on success, false on failure
     */
    bool initialize(Range range = Range::Range_2g, Bandwidth bandwidth = Bandwidth::Bandwidth_1000_Hz);

    /*! Set the BMA280 power mode
     *
     * \param mode Power mode to be applied
     *
     */
    void set_power_mode(PowerMode mode);

    /*! Set BMA280 range configuration
     *
     * \param range acceleration range 2g/4g/8g/16g
     *
     */
    void set_range(Range range);

    /*! Set BMA280  bandwidth configuration
     *
     * \param bandwidth Low-pass filter 7.81Hz/15.63Hz/31.25Hz/62.5Hz/125Hz/250Hz/500Hz/1000Hz
     *
     */
    void set_bandwidth(Bandwidth bandwidth);

    /*! Get the acceleration values
     *
     * \return the acceleration values (X, Y and Z) structure in m/s²
     *
     */
    bma280_acceleration_t acceleration();

    /*! Get the acceleration values
     *
     * \return the X acceleration values in m/s²
     *
     */
    double acceleration_x();

    /*! Get the acceleration values
     *
     * \return the Y acceleration values in m/s²
     *
     */
    double acceleration_y();

    /*! Get the acceleration values
     *
     * \return the Z acceleration values in m/s²
     *
     */
    double acceleration_z();

    /*! Get the acceleration values
     *
     * \return the temperature value in °C
     *
     */
    double temperature();

    /*! Get the 3 axis offsets. 1 LSB = 1/128 m/s²
     *
     * \return the offset structure data to store the read values
     *
     */
    bma280_offset_t offsets();

    /*! Set the 3 axis offsets. 1 LSB = 1/128 m/s²
     *
     * \param offsets pointer to the offset structure to write
     *
     */
    void set_offsets(bma280_offset_t *offsets);

    /*!
     * \brief Enable the slow offset compensation
     *
     * \param x_axis enable (true) / disable (false) the slow offset compensation for X axis
     * \param y_axis enable (true) / disable (false) the slow offset compensation for Y axis
     * \param z_axis enable (true) / disable (false) the slow offset compensation for Z axis
     *
     */
    void enable_slow_offset_compensation(bool x_axis, bool y_axis, bool z_axis);

    /*!
     * \brief set fast offset calibration for X axis
     *
     * return true on calibration done, false in failure
     */
    bool set_fast_offsets_calibration_x();

    /*!
     * \brief set fast offset calibration for Y axis
     *
     * return true on calibration done, false in failure
     */
    bool set_fast_offsets_calibration_y();

    /*!
     * \brief set fast offset calibration for Z axis
     *
     * return true on calibration done, false in failure
     */
    bool set_fast_offsets_calibration_z();

    /*!
     * \brief Get the BMA280 chip ID
     *
     * \return BMA280 chip ID
     *
     */
    char chip_id();

    /*!
     * \brief Set the sleep duration
     *
     * \param sleep_duration the sleep phase duration in LOW_POWER mode
     *
     */
    void set_sleep_duration(SleepDuration sleep_duration);

    /*!
     * \brief BMA280 software reset
     *
     */
    void reset();

private:
    /*! Set register value
     *
     * \param registerAddress register address
     * \param value, value to write
     *
     * \returns 0 on success,
     *          no-0 on failure
     */
    int i2c_set_register(RegisterAddress registerAddress, char value);

    /*! Get register value
     *
     * \param registerAddress register address
     * \param value pointer to store read value
     *
     * \returns 0 on success,
     *          no-0 on failure
     */
    int i2c_read_register(RegisterAddress registerAddress, char *value);

    /*! Get multi-byte register value (two-bytes)
     *
     * \param registerAddress register address of LSB
     * \param value pointer to store read value
     *
     * \returns 0 on success,
     *          no-0 on failure
     */
    int i2c_read_two_bytes_register(RegisterAddress registerAddress, int16_t *value);

    /*! Get multi-byte register value (3*2-bytes) that are stored in a 3 dimensions vector
     *
     * \param registerAddress register address of LSB
     * \param value pointer to store read value to
     *
     * \returns 0 on success,
     *          no-0 on failure
     */
    int i2c_read_vector(RegisterAddress registerAddress, int16_t value[3]);

    I2C *_i2c;
    I2CAddress _i2cAddress;
    Range _range;
    char _chipId = 0;
};


} // namespace sixtron

#endif // CATIE_SIXTRON_LIBTEMPLATE_H_
