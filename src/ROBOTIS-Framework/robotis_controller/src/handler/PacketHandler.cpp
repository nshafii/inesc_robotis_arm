/*
 *=====================================================
 * File   :  PacketHandler.cpp
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

#include "../../include/handler/PacketHandler.h"

#include "../../include/handler/PacketProtocol1.h"
#include "../../include/handler/PacketProtocol2.h"

#define LATENCY_TIME        16 //ms (USB2Dynamixel Default Latency Time)

using namespace ROBOTIS;

PacketHandler::PacketHandler()
{
}

PacketHandler *PacketHandler::getPacketHandler(float protocol_ver)
{
    if(protocol_ver == 1.0)
        return PacketProtocol1::getInstance();
    else if(protocol_ver == 2.0)
        return PacketProtocol2::getInstance();

    return PacketProtocol2::getInstance();
}


