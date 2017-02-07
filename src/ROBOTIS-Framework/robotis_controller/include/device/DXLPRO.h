/*
 *=====================================================
 * File   :  DXLPRO.h
 * Author :  zerom <zerom@robotis.com>
 * Copyright (C) ROBOTIS, 2015
 *=====================================================
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#ifndef DXLPRO_H_
#define DXLPRO_H_

#include "GenericDevice.h"

namespace ROBOTIS
{

class DXLPRO : public GenericDevice
{
public:

    long rad2Value(double radian)   {
        long value = 0;
        if(radian > 0) {
            if(MAX_VALUE <= 0)
                return MAX_VALUE;
            value = radian * MAX_VALUE / MAX_RADIAN;
        }
        else if(radian < 0) {
            if(MIN_VALUE >= 0)
                return MIN_VALUE;
            value = radian * MIN_VALUE / MIN_RADIAN;
        }
        else
            value = RAD_0_POSITION_VALUE;

        if(value > MAX_VALUE)
            return MAX_VALUE;
        else if(value < MIN_VALUE)
            return MIN_VALUE;

        return value;
    }
    double value2Rad(long value)    {
        double rad = 0.0;
        if(value > RAD_0_POSITION_VALUE) {
            if(MAX_RADIAN <= 0)
                return MAX_RADIAN;
            rad = (double)value * MAX_RADIAN / (double)MAX_VALUE;
        }
        else if(value < RAD_0_POSITION_VALUE) {
            if(MIN_RADIAN >= 0)
                return MIN_RADIAN;
            rad = (double)value * MIN_RADIAN / (double)MIN_VALUE;
        }
        else
            rad = 0;

        if(rad > MAX_RADIAN)
            return MAX_RADIAN;
        else if(rad < MIN_RADIAN)
            return MIN_RADIAN;

        return rad;
    }

    ~DXLPRO() { }
    DXLPRO(PortHandler *port, long min_value, long max_value, long rad_0_position_value, double min_radian, double max_radian)
        : GenericDevice(port, min_value, max_value, rad_0_position_value, min_radian, max_radian)
    {
        ADDR_MODEL_NUMBER                               = P_MODEL_NUMBER;
        ADDR_FIRMWARE_VERSION                           = P_FIRMWARE_VERSION;
        ADDR_ID                                         = P_ID;
        ADDR_BAUD_RATE                                  = P_BAUD_RATE;
        ADDR_RETURN_DELAY_TIME                          = P_RETURN_DELAY_TIME;
        ADDR_RETURN_LEVEL                               = P_RETURN_LEVEL;
        ADDR_MIN_POSITION_LIMIT                         = P_MIN_POSITION_LIMIT;
        ADDR_MAX_POSITION_LIMIT                         = P_MAX_POSITION_LIMIT;
        ADDR_TORQUE_ENABLE                              = P_TORQUE_ENABLE;
        ADDR_POSITION_D_GAIN                            = P_POSITION_D_GAIN;
        ADDR_POSITION_I_GAIN                            = P_POSITION_I_GAIN;
        ADDR_POSITION_P_GAIN                            = P_POSITION_P_GAIN;
        ADDR_GOAL_POSITION                              = P_GOAL_POSITION;
        ADDR_GOAL_VELOCITY                              = P_GOAL_VELOCITY;
        ADDR_GOAL_TORQUE                                = P_PRESENT_CURRENT;
        ADDR_PRESENT_POSITION                           = P_PRESENT_POSITION;
        ADDR_PRESENT_VELOCITY                           = P_PRESENT_VELOCITY;
        ADDR_PRESENT_LOAD                               = P_PRESENT_LOAD;
        ADDR_MOVING                                     = P_MOVING;

        addr_length[P_MODEL_NUMBER]                     = 2;
        addr_length[P_MODEL_INFORMATION]                = 4;
        addr_length[P_FIRMWARE_VERSION]                 = 1;
        addr_length[P_ID]                               = 1;
        addr_length[P_BAUD_RATE]                        = 1;
        addr_length[P_RETURN_DELAY_TIME]                = 1;
        addr_length[P_INVERTER_MODE]                    = 1;
        addr_length[P_OPERATING_MODE]                   = 1;
        addr_length[P_DIRECTION_SETTING]                = 1;
        addr_length[P_HOMING_OFFSET]                    = 4;
        addr_length[P_MOVING_THRESHOLD]                 = 4;
        addr_length[P_TEMPERATURE_LIMIT]                = 1;
        addr_length[P_MAX_VOLTAGE_LIMIT]                = 2;
        addr_length[P_MIN_VOLTAGE_LIMIT]                = 2;
        addr_length[P_ACCELATION_LIMIT]                 = 4;
        addr_length[P_TORQUE_LIMIT]                     = 2;
        addr_length[P_VELOCITY_LIMIT]                   = 4;
        addr_length[P_MAX_POSITION_LIMIT]               = 4;
        addr_length[P_MIN_POSITION_LIMIT]               = 4;
        addr_length[P_EXTERNAL_PORT_MODE_1]             = 1;
        addr_length[P_EXTERNAL_PORT_MODE_2]             = 1;
        addr_length[P_EXTERNAL_PORT_MODE_3]             = 1;
        addr_length[P_EXTERNAL_PORT_MODE_4]             = 1;
        addr_length[P_SHUTDOWN_CONFIG]                  = 1;
        addr_length[P_INDIRECT_ADDRESS_0]               = 2;

        addr_length[P_INDIRECT_ADDRESS_255]             = 2;
        addr_length[P_RESET_OPTION]                     = 1;
        addr_length[P_TORQUE_ENABLE]                    = 1;
        addr_length[P_LED_RED]                          = 1;
        addr_length[P_LED_GREEN]                        = 1;
        addr_length[P_LED_BLUE]                         = 1;
        addr_length[P_CURRENT_I_GAIN]                   = 2;
        addr_length[P_CURRENT_P_GAIN]                   = 2;
        addr_length[P_ACCELATION_FEED_FORWARD_GAIN]     = 4;
        addr_length[P_VELOCITY_FEED_FORWARD_GAIN]       = 4;
        addr_length[P_POSITION_LPF]                     = 4;
        addr_length[P_VELOCITY_LPF]                     = 4;
        addr_length[P_VELOCITY_I_GAIN]                  = 2;
        addr_length[P_VELOCITY_P_GAIN]                  = 2;
        addr_length[P_POSITION_D_GAIN]                  = 2;
        addr_length[P_POSITION_I_GAIN]                  = 2;
        addr_length[P_POSITION_P_GAIN]                  = 2;
        addr_length[P_GOAL_POSITION]                    = 4;
        addr_length[P_GOAL_VELOCITY]                    = 4;
        addr_length[P_GOAL_TORQUE]                      = 2;
        addr_length[P_GOAL_ACCELERATION]                = 4;
        addr_length[P_MOVING]                           = 1;
        addr_length[P_PRESENT_POSITION]                 = 4;
        addr_length[P_PRESENT_VELOCITY]                 = 4;
        addr_length[P_PRESENT_LOAD]                     = 2;
        addr_length[P_PRESENT_CURRENT]                  = 2;
        addr_length[P_PRESENT_VOLTAGE]                  = 2;
        addr_length[P_PRESENT_TEMPERATURE]              = 1;
        addr_length[P_EXTERNAL_PORT_DATA_1]             = 2;
        addr_length[P_EXTERNAL_PORT_DATA_2]             = 2;
        addr_length[P_EXTERNAL_PORT_DATA_3]             = 2;
        addr_length[P_EXTERNAL_PORT_DATA_4]             = 2;
        addr_length[P_INDIRECT_DATA_0]                  = 1;

        addr_length[P_INDIRECT_DATA_255]                = 1;
        addr_length[P_SELF_TEST]                        = 1;
        addr_length[P_RETURN_LEVEL]                     = 1;
        addr_length[P_SHUTDOWN_CANCEL]                  = 1;
    }

    enum
    {
        P_MODEL_NUMBER                                  = 0,
        P_MODEL_INFORMATION                             = 2,
        P_FIRMWARE_VERSION                              = 6,
        P_ID                                            = 7,
        P_BAUD_RATE                                     = 8,
        P_RETURN_DELAY_TIME                             = 9,
        P_INVERTER_MODE                                 = 10,
        P_OPERATING_MODE                                = 11,
        P_DIRECTION_SETTING                             = 12,
        P_HOMING_OFFSET                                 = 13,
        P_MOVING_THRESHOLD                              = 17,
        P_TEMPERATURE_LIMIT                             = 21,
        P_MAX_VOLTAGE_LIMIT                             = 22,
        P_MIN_VOLTAGE_LIMIT                             = 24,
        P_ACCELATION_LIMIT                              = 26,
        P_TORQUE_LIMIT                                  = 30,
        P_VELOCITY_LIMIT                                = 32,
        P_MAX_POSITION_LIMIT                            = 36,
        P_MIN_POSITION_LIMIT                            = 40,
        P_EXTERNAL_PORT_MODE_1                          = 44,
        P_EXTERNAL_PORT_MODE_2                          = 45,
        P_EXTERNAL_PORT_MODE_3                          = 46,
        P_EXTERNAL_PORT_MODE_4                          = 47,
        P_SHUTDOWN_CONFIG                               = 48,
        P_INDIRECT_ADDRESS_0                            = 49,

        P_INDIRECT_ADDRESS_255                          = 559,
        P_RESET_OPTION                                  = 561,
        P_TORQUE_ENABLE                                 = 562,
        P_LED_RED                                       = 563,
        P_LED_GREEN                                     = 564,
        P_LED_BLUE                                      = 565,
        P_CURRENT_I_GAIN                                = 566,
        P_CURRENT_P_GAIN                                = 568,
        P_ACCELATION_FEED_FORWARD_GAIN                  = 570,
        P_VELOCITY_FEED_FORWARD_GAIN                    = 574,
        P_POSITION_LPF                                  = 578,
        P_VELOCITY_LPF                                  = 582,
        P_VELOCITY_I_GAIN                               = 586,
        P_VELOCITY_P_GAIN                               = 588,
        P_POSITION_D_GAIN                               = 590,
        P_POSITION_I_GAIN                               = 592,
        P_POSITION_P_GAIN                               = 594,
        P_GOAL_POSITION                                 = 596,
        P_GOAL_VELOCITY                                 = 600,
        P_GOAL_TORQUE                                   = 604,
        P_GOAL_ACCELERATION                             = 606,
        P_MOVING                                        = 610,
        P_PRESENT_POSITION                              = 611,
        P_PRESENT_VELOCITY                              = 615,
        P_PRESENT_LOAD                                  = 619,
        P_PRESENT_CURRENT                               = 621,
        P_PRESENT_VOLTAGE                               = 623,
        P_PRESENT_TEMPERATURE                           = 625,
        P_EXTERNAL_PORT_DATA_1                          = 626,
        P_EXTERNAL_PORT_DATA_2                          = 628,
        P_EXTERNAL_PORT_DATA_3                          = 630,
        P_EXTERNAL_PORT_DATA_4                          = 632,
        P_INDIRECT_DATA_0                               = 634,

        P_INDIRECT_DATA_255                             = 889,
        P_SELF_TEST                                     = 890,
        P_RETURN_LEVEL                                  = 891,
        P_SHUTDOWN_CANCEL                               = 892,
    };
};


}

#endif /* DXLPRO_H_ */
