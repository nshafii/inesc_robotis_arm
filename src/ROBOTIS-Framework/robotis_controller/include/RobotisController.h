/*
 *=====================================================
 * File   :  RobotisController.h
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

#ifndef ROBOTISCONTROLLER_H_
#define ROBOTISCONTROLLER_H_


#include <vector>

#include "device/GenericDevice.h"
#include "handler/PacketHandler.h"

namespace ROBOTIS
{


class RobotisController
{
private:
    GenericDevice               *dxlList[MAX_ID+1];
    std::vector<PortHandler *>   portList;
    std::vector<PacketHandler *> packetHandlerList;

public:
    std::vector<int>            idList;

    RobotisController();
    virtual ~RobotisController();

    bool    initialize();

    bool    addSerialPort(const char* port_name, int baudrate = PortHandler::DEFAULT_BAUDRATE);
    void    addDevice(PortHandler* port, int id, std::string joint_name, std::string model, float protocol_ver = 2.0);

    GenericDevice *getDevice(int id);

    int     read(int id, int address, long *data, int *error = 0);
    int     read(int id, int address, long *data, LENGTH_TYPE length, int *error = 0);

    int     write(int id, int address, long data, int *error = 0);
    int     write(int id, int address, long data, LENGTH_TYPE length, int *error = 0);

    int     getTorqueEnable(int id, int *enable);
    int     setTorqueEnable(int id, int enable);

    int     getPresentPositionRadian(int id, double *radian);
    int     getPresentPositionValue(int id, long *position);
    int     getPresentVelocity(int id, long *velocity);
    int     getPresentLoad(int id, long *load);

    int     getGoalPositionRadian(int id, double *radian);
    int     setGoalPositionRadian(int id, double radian);
    int     getGoalPositionValue(int id, long *position);
    int     setGoalPositionValue(int id, long position);

    int     getGoalVelocity(int id, long *velocity);
    int     setGoalVelocity(int id, long velocity);

    int     getGoalTorque(int id, long *torque);
    int     setGoalTorque(int id, long torque);

    int     getPositionPGain(int id, int *pgain);
    int     setPositionPGain(int id, int pgain);

    int     getPositionIGain(int id, int *igain);
    int     setPositionIGain(int id, int igain);

    int     getPositionDGain(int id, int *dgain);
    int     setPositionDGain(int id, int dgain);

    int     isMoving(int id, bool *ismoving);

//    ADDR_RETURN_DELAY_TIME;
//    ADDR_RETURN_LEVEL;
//    ADDR_MIN_POSITION_LIMIT;
//    ADDR_MAX_POSITION_LIMIT;
};


}



#endif /* ROBOTISCONTROLLER_H_ */
