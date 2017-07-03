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

namespace sixtron {

class BMA280
{
public:
    enum class I2CAddress {
        Address1 = 0x18,
        Address2 = 0x19
    };

    enum class RegisterAddress : char {
    ChipId						=    (0x00),
    /** DATA ADDRESS DEFINITIONS */
    X_Axis_Lsb                  =    (0x02),
    X_Axis_Msb                  =    (0x03),
    Y_Axis_Lsb                  =    (0x04),
    Y_Axis_Msb                  =    (0x05),
    Z_Axis_Lsb                  =    (0x06),
    Z_Axis_Msb                  =    (0x07),
    Temp						=    (0x08),
    /**STATUS ADDRESS DEFINITIONS */
    Stat1						=    (0x09),
    Stat2						=    (0x0A),
    StatTapSlop					=    (0x0B),
    StatOrientHigh				=    (0x0C),
    StatFifo					=    (0x0E),
    /**STATUS ADDRESS DEFINITIONS */
    RangeSelect					=    (0x0F),
    BwSelect   	                =    (0x10),
    ModeCtrl                    =    (0x11),
    LowNoiseCtrl	            =    (0x12),
    DataCtrl                    =    (0x13),
    Rst                         =    (0x14),
    /**INTERUPT ADDRESS DEFINITIONS */
    IntrEnable1                 =    (0x16),
    IntrEnable2                 =    (0x17),
    IntrSlowNoMotion	        =    (0x18),
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

};

} // namespace sixtron

#endif // CATIE_SIXTRON_LIBTEMPLATE_H_
