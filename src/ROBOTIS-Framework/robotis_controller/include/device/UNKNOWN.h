/*
 * UNKNOWN.h
 *
 *  Created on: 2015. 9. 23.
 *      Author: zerom
 */

#ifndef UNKNOWN_H_
#define UNKNOWN_H_


#include "GenericDevice.h"

namespace ROBOTIS
{

class UNKNOWN : public GenericDevice
{
public:

    long rad2Value(double radian)   { return 0; }
    double value2Rad(long value)    { return 0; }

    ~UNKNOWN() { }
    UNKNOWN(PortHandler *port) : GenericDevice(port, 0, 4095, 2048, -PI, PI)
    {
        ADDR_MODEL_NUMBER                           = P_MODEL_NUMBER;
        ADDR_FIRMWARE_VERSION                       = P_FIRMWARE_VERSION;
        ADDR_ID                                     = P_ID;
        ADDR_BAUD_RATE                              = P_BAUD_RATE;
        ADDR_RETURN_DELAY_TIME                      = P_RETURN_DELAY_TIME;
        ADDR_RETURN_LEVEL                           = P_RETURN_LEVEL;

        addr_length[P_MODEL_NUMBER]                 = 2;
        addr_length[P_FIRMWARE_VERSION]             = 1;
        addr_length[P_ID]                           = 1;
        addr_length[P_BAUD_RATE]                    = 1;
        addr_length[P_RETURN_DELAY_TIME]            = 1;
        addr_length[P_RETURN_LEVEL]                 = 1;
    }

    enum
    {
        P_MODEL_NUMBER                              = 0,
        P_FIRMWARE_VERSION                          = 2,
        P_ID                                        = 3,
        P_BAUD_RATE                                 = 4,
        P_RETURN_DELAY_TIME                         = 5,
        P_RETURN_LEVEL                              = 16,
    };
};


}


#endif /* UNKNOWN_H_ */
