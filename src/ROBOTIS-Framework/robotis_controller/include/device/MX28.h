/*
 *=====================================================
 * File   :  MX28.h
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

#ifndef _MX28_H_
#define _MX28_H_

#include "GenericDevice.h"

namespace ROBOTIS
{

class MX28 : public GenericDevice
{
public:

    long rad2Value(double radian)   {
        long value;
        if(radian > 0) {
            if(MAX_VALUE <= RAD_0_POSITION_VALUE)
                return MAX_VALUE;
            value = radian * (MAX_VALUE - RAD_0_POSITION_VALUE) / MAX_RADIAN + RAD_0_POSITION_VALUE;
        }
        else if(radian < 0) {
            if(MIN_VALUE >= RAD_0_POSITION_VALUE)
                return MIN_VALUE;
            value = radian * (MIN_VALUE - RAD_0_POSITION_VALUE) / MIN_RADIAN + RAD_0_POSITION_VALUE;
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
            rad = (double)(value - RAD_0_POSITION_VALUE) * MAX_RADIAN / (double)(MAX_VALUE - RAD_0_POSITION_VALUE);
        }
        else if(value < RAD_0_POSITION_VALUE) {
            if(MIN_RADIAN >= 0)
                return MIN_RADIAN;
            rad = (double)(value - RAD_0_POSITION_VALUE) * MIN_RADIAN / (double)(MIN_VALUE - RAD_0_POSITION_VALUE);
        }

        if(rad > MAX_RADIAN)
            return MAX_RADIAN;
        else if(rad < MIN_RADIAN)
            return MIN_RADIAN;

        return rad;
    }

    ~MX28() { }
    MX28(PortHandler *port) : GenericDevice(port, 0, 4095, 2048, -PI, PI)
    {
        ADDR_MODEL_NUMBER                           = P_MODEL_NUMBER;
        ADDR_FIRMWARE_VERSION                       = P_FIRMWARE_VERSION;
        ADDR_ID                                     = P_ID;
        ADDR_BAUD_RATE                              = P_BAUD_RATE;
        ADDR_RETURN_DELAY_TIME                      = P_RETURN_DELAY_TIME;
        ADDR_RETURN_LEVEL                           = P_RETURN_LEVEL;
        ADDR_MIN_POSITION_LIMIT                     = P_MIN_POSITION_LIMIT;
        ADDR_MAX_POSITION_LIMIT                     = P_MAX_POSITION_LIMIT;
        ADDR_TORQUE_ENABLE                          = P_TORQUE_ENABLE;
        ADDR_POSITION_D_GAIN                        = P_POSITION_D_GAIN;
        ADDR_POSITION_I_GAIN                        = P_POSITION_I_GAIN;
        ADDR_POSITION_P_GAIN                        = P_POSITION_P_GAIN;
        ADDR_GOAL_POSITION                          = P_GOAL_POSITION;
        ADDR_GOAL_VELOCITY                          = P_GOAL_VELOCITY;
        ADDR_GOAL_TORQUE                            = P_GOAL_TORQUE;
        ADDR_PRESENT_POSITION                       = P_PRESENT_POSITION;
        ADDR_PRESENT_VELOCITY                       = P_PRESENT_VELOCITY;
        ADDR_PRESENT_LOAD                           = P_PRESENT_LOAD;
        ADDR_MOVING                                 = P_MOVING;

        addr_length[P_MODEL_NUMBER]                 = 2;
        addr_length[P_FIRMWARE_VERSION]             = 1;
        addr_length[P_ID]                           = 1;
        addr_length[P_BAUD_RATE]                    = 1;
        addr_length[P_RETURN_DELAY_TIME]            = 1;
        addr_length[P_MIN_POSITION_LIMIT]           = 2;
        addr_length[P_MAX_POSITION_LIMIT]           = 2;
        addr_length[P_TEMPERATURE_LIMIT]            = 1;
        addr_length[P_MIN_VOLTAGE_LIMIT]            = 1;
        addr_length[P_MAX_VOLTAGE_LIMIT]            = 1;
        addr_length[P_TORQUE_LIMIT]                 = 2;
        addr_length[P_RETURN_LEVEL]                 = 1;
        addr_length[P_ALARM_LED_CONFIG]             = 1;
        addr_length[P_SHUTDOWN_CONFIG]              = 1;
        addr_length[P_OPERATING_MODE]               = 1;
        addr_length[P_MULTI_TURN_OFFSET]            = 2;
        addr_length[P_RESOLUTION_DIVIDER]           = 1;
        addr_length[P_TORQUE_ENABLE]                = 1;
        addr_length[P_LED]                          = 1;
        addr_length[P_POSITION_D_GAIN]              = 1;
        addr_length[P_POSITION_I_GAIN]              = 1;
        addr_length[P_POSITION_P_GAIN]              = 1;
        addr_length[P_GOAL_POSITION]                = 2;
        addr_length[P_GOAL_VELOCITY]                = 2;
        addr_length[P_GOAL_TORQUE]                  = 2;
        addr_length[P_PRESENT_POSITION]             = 2;
        addr_length[P_PRESENT_VELOCITY]             = 2;
        addr_length[P_PRESENT_LOAD]                 = 2;
        addr_length[P_PRESENT_VOLTAGE]              = 1;
        addr_length[P_PRESENT_TEMPERATURE]          = 1;
        addr_length[P_REGISTERED_INSTRUCTION]       = 1;
        addr_length[P_MOVING]                       = 1;
        addr_length[P_LOCK]                         = 1;
        addr_length[P_PUNCH]                        = 2;
        addr_length[P_GOAL_ACCELERATION]            = 1;
    }

    enum
    {
        P_MODEL_NUMBER                              = 0,
        P_FIRMWARE_VERSION                          = 2,
        P_ID                                        = 3,
        P_BAUD_RATE                                 = 4,
        P_RETURN_DELAY_TIME                         = 5,
        P_MIN_POSITION_LIMIT                        = 6,
        P_MAX_POSITION_LIMIT                        = 8,
        P_TEMPERATURE_LIMIT                         = 11,
        P_MIN_VOLTAGE_LIMIT                         = 12,
        P_MAX_VOLTAGE_LIMIT                         = 13,
        P_TORQUE_LIMIT                              = 14,
        P_RETURN_LEVEL                              = 16,
        P_ALARM_LED_CONFIG                          = 17,
        P_SHUTDOWN_CONFIG                           = 18,
        P_OPERATING_MODE                            = 19,
        P_MULTI_TURN_OFFSET                         = 20,
        P_RESOLUTION_DIVIDER                        = 22,
        P_TORQUE_ENABLE                             = 24,
        P_LED                                       = 25,
        P_POSITION_D_GAIN                           = 26,
        P_POSITION_I_GAIN                           = 27,
        P_POSITION_P_GAIN                           = 28,
        P_GOAL_POSITION                             = 30,
        P_GOAL_VELOCITY                             = 32,
        P_GOAL_TORQUE                               = 34,
        P_PRESENT_POSITION                          = 36,
        P_PRESENT_VELOCITY                          = 38,
        P_PRESENT_LOAD                              = 40,
        P_PRESENT_VOLTAGE                           = 42,
        P_PRESENT_TEMPERATURE                       = 43,
        P_REGISTERED_INSTRUCTION                    = 44,
        P_MOVING                                    = 46,
        P_LOCK                                      = 47,
        P_PUNCH                                     = 48,
        P_GOAL_ACCELERATION                         = 73,
    };
};


}

#endif /* _MX28_H_ */
