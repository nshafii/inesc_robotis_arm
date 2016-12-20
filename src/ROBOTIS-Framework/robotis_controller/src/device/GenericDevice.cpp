/*
 *=====================================================
 * File   :  DXL.cpp
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

#include <string.h>

#include "../../include/device/DXLPRO.h"
#include "../../include/device/GenericDevice.h"
#include "../../include/device/MX28.h"
#include "../../include/device/UNKNOWN.h"

using namespace ROBOTIS;

GenericDevice::GenericDevice(PortHandler *port, long min_value, long max_value, long center_value, double min_radian, double max_radian) :
    comPort(port), packetHandler(PacketHandler::getPacketHandler(2.0)),
    MIN_VALUE(min_value), MAX_VALUE(max_value), RAD_0_POSITION_VALUE(center_value), MIN_RADIAN(min_radian), MAX_RADIAN(max_radian),
    ID(0), PROTOCOL_VERSION(2.0)
{
    ADDR_MODEL_NUMBER           = -1;
    ADDR_FIRMWARE_VERSION       = -1;
    ADDR_ID                     = -1;
    ADDR_BAUD_RATE              = -1;
    ADDR_RETURN_DELAY_TIME      = -1;
    ADDR_RETURN_LEVEL           = -1;
    ADDR_MIN_POSITION_LIMIT     = -1;
    ADDR_MAX_POSITION_LIMIT     = -1;
    ADDR_TORQUE_ENABLE          = -1;
    ADDR_POSITION_D_GAIN        = -1;
    ADDR_POSITION_I_GAIN        = -1;
    ADDR_POSITION_P_GAIN        = -1;
    ADDR_GOAL_POSITION          = -1;
    ADDR_GOAL_VELOCITY          = -1;
    ADDR_GOAL_TORQUE            = -1;
    ADDR_PRESENT_POSITION       = -1;
    ADDR_PRESENT_VELOCITY       = -1;
    ADDR_PRESENT_LOAD           = -1;
    ADDR_MOVING                 = -1;
}

int GenericDevice::getAddrLength(int addr)
{
    std::map<int, int>::iterator iter_addr_length;
    iter_addr_length = addr_length.find(addr);
    if(iter_addr_length != addr_length.end())
        return iter_addr_length->second;

    return -1;
}

GenericDevice *GenericDevice::getInstance(PortHandler *port, int id, std::string joint_name, std::string model, float protocol_ver)
{
    GenericDevice *ret = 0;

    if(model == "MX-28" || model == "MX28" || model == "mx-28" || model == "mx28")
        ret = new MX28(port);
    else if(model == "L42-10-S300-R")
        ret = new DXLPRO(port, -2047, 2048, 0, -PI, PI);
    else if(model =="L54-50-S290-R")
        ret = new DXLPRO(port, -103860, 103860, 0, -PI, PI);
    else if(model =="L54-30-S400-R")
        ret = new DXLPRO(port, -144198, 144198, 0, -PI, PI);
    else if(model =="L54-50-S500-R" || model =="L54-30-S500-R")
        ret = new DXLPRO(port, -180684, 180684, 0, -PI, PI);
    else if(model =="M42-10-S260-R")
        ret = new DXLPRO(port, -131584, 131584, 0, -PI, PI);
    else if(model =="M54-40-S250-R" || model =="M54-60-S250-R")
        ret = new DXLPRO(port, -125700, 125700, 0, -PI, PI);
    else if(model =="H42-20-S300-R")
        ret = new DXLPRO(port, -151900, 151900, 0, -PI, PI);
    else if(model =="H54-100-S500-R" || model =="H54-200-S500-R" || model =="H54-200-B500-R")
        ret = new DXLPRO(port, -250950, 250950, 0, -PI, PI);
    else if(model =="GRIPPER")
        ret = new DXLPRO(port, 0, 110000, 0, 0, PI/4.0);
    else
        ret = new UNKNOWN(port);

    if(ret != 0)
    {
        ret->PROTOCOL_VERSION = protocol_ver;
        ret->packetHandler = PacketHandler::getPacketHandler(protocol_ver);
        ret->ID = id;
        ret->jointName = joint_name;
    }

    return ret;
}

std::string GenericDevice::getJointName()
{
    return jointName;
}

PortHandler* GenericDevice::getSerialPort()
{
    return comPort;
}

int GenericDevice::read(int address, long *data, int *error)
{
    int data_len = getAddrLength(address);

    if(data_len < 0)
    	return COMM_NOT_AVAILABLE;

    unsigned char *read_data = new unsigned char[data_len];

    int result = packetHandler->read(comPort, ID, address, data_len, read_data, error);

    switch(data_len)
    {
    case 1:
        *data = read_data[0];
        break;
    case 2:
        *data = DXL_MAKEWORD(read_data[0], read_data[1]);
        break;
    case 4:
        *data = DXL_MAKEDWORD(DXL_MAKEWORD(read_data[0], read_data[1]), DXL_MAKEWORD(read_data[2], read_data[3]));
        break;
    default:
        break;
    }

    return result;
}

int GenericDevice::read(int address, long *data, LENGTH_TYPE length, int *error)
{
    if(address < 0)
        return COMM_NOT_AVAILABLE;

    unsigned char *read_data = new unsigned char[length];

    int result = packetHandler->read(comPort, ID, address, length, read_data, error);

    switch(length)
    {
    case 1:
        *data = read_data[0];
        break;
    case 2:
        *data = DXL_MAKEWORD(read_data[0], read_data[1]);
        break;
    case 4:
        *data = DXL_MAKEDWORD(DXL_MAKEWORD(read_data[0], read_data[1]), DXL_MAKEWORD(read_data[2], read_data[3]));
        break;
    default:
        break;
    }

    return result;
}

int GenericDevice::write(int address, long data, int *error)
{
    int data_len = getAddrLength(address);

    if(data_len < 0)
    	return COMM_NOT_AVAILABLE;

    unsigned char *write_data = new unsigned char[data_len];

    switch(data_len)
    {
    case 1:
        write_data[0] = data;
        break;
    case 2:
        write_data[0] = DXL_LOBYTE(data);
        write_data[1] = DXL_HIBYTE(data);
        break;
    case 4:
        write_data[0] = DXL_LOBYTE(DXL_LOWORD(data));
        write_data[1] = DXL_HIBYTE(DXL_LOWORD(data));
        write_data[2] = DXL_LOBYTE(DXL_HIWORD(data));
        write_data[3] = DXL_HIBYTE(DXL_HIWORD(data));
        break;
    default:
        break;
    }

    return packetHandler->write(comPort, ID, address, data_len, write_data, error);
}

int GenericDevice::write(int address, long data, LENGTH_TYPE length, int *error)
{
    if(address < 0)
        return COMM_NOT_AVAILABLE;

    unsigned char *write_data = new unsigned char[length];

    switch(length)
    {
    case 1:
        write_data[0] = data;
        break;
    case 2:
        write_data[0] = DXL_LOBYTE(data);
        write_data[1] = DXL_HIBYTE(data);
        break;
    case 4:
        write_data[0] = DXL_LOBYTE(DXL_LOWORD(data));
        write_data[1] = DXL_HIBYTE(DXL_LOWORD(data));
        write_data[2] = DXL_LOBYTE(DXL_HIWORD(data));
        write_data[3] = DXL_HIBYTE(DXL_HIWORD(data));
        break;
    default:
        break;
    }

    return packetHandler->write(comPort, ID, address, length, write_data, error);
}
