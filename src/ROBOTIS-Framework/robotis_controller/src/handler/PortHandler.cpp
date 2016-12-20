/*
 *=====================================================
 * File   :  PortHandler.cpp
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

#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include "../../include/handler/PortHandler.h"
#include "iostream"
using namespace std;

#define LATENCY_TIME        16 //ms (USB2Dynamixel Default Latency Time)

using namespace ROBOTIS;

PortHandler::PortHandler(const char* port_name)
    : socketFD(-1), portBaudRate(DEFAULT_BAUDRATE), DEBUG_PRINT(true),
      packetStartTime(0.0), packetWaitTime(0.0), byteTransferTime(0.0)
{
    setPortName(port_name);


}

PortHandler::~PortHandler() {
    closePort();
}

void PortHandler::setPortName(const char* port_name)
{
    strcpy(portName, port_name);
}

char* PortHandler::getPortName()
{
    return portName;
}

void PortHandler::setBaudRate(int baudrate)
{
    portBaudRate = baudrate;
}

int PortHandler::getBaudRate()
{
    return portBaudRate;
}

bool PortHandler::changeBaudRate(int baudrate)
{
//    if(socketFD < 0)
//    {
//        if(DEBUG_PRINT == true)
//            printf("[PortHandler::changeBaudRate] Port is not opened!\n");
//        return false;
//    }

    int baud = getBaud(baudrate);

    closePort();

    if(baud <= 0)
    {
        setupSerialPort(B38400);
        portBaudRate = baudrate;
        return setBaudDevisor(baudrate);
    }
    else
    {
        portBaudRate = baudrate;
        return setupSerialPort(baud);
    }
}

void PortHandler::setPacketTimeout(int packet_len)
{
    packetStartTime = getCurrentTime();
    packetWaitTime = (byteTransferTime * (double)packet_len) + (2.0 * (double)LATENCY_TIME) + 2.0;
}

void PortHandler::setPacketTimeout(double msec)
{
    packetStartTime = getCurrentTime();
    packetWaitTime = msec;
}

bool PortHandler::isPacketTimeout()
{
    if(getPacketTime() > packetWaitTime)
        return true;
    return false;
}

double PortHandler::getPacketTime()
{
    double time;

    time = getCurrentTime() - packetStartTime;
    if(time < 0.0)
        packetStartTime = getCurrentTime();

    return time;
}

double PortHandler::getCurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return ((double)tv.tv_sec*1000.0 + (double)tv.tv_usec/1000.0);
}

int PortHandler::getBytesAvailable()
{
    int bytes_available;
    ioctl(socketFD, FIONREAD, &bytes_available);
    return bytes_available;
}

bool PortHandler::setBaudDevisor(int speed)
{
    // try to set a custom divisor
    struct serial_struct ss;
    if(ioctl(socketFD, TIOCGSERIAL, &ss) != 0)
    {
        if(DEBUG_PRINT == true)
            printf("[PortHandler::setBaudDevisor] TIOCGSERIAL failed!\n");
        return false;
    }

    ss.flags = (ss.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
    ss.custom_divisor = (ss.baud_base + (speed / 2)) / speed;
    int closest_speed = ss.baud_base / ss.custom_divisor;

    if(closest_speed < speed * 98 / 100 || closest_speed > speed * 102 / 100)
    {
        if(DEBUG_PRINT == true)
            printf("[PortHandler::setBaudDevisor] Cannot set speed to %d, closest is %d \n", speed, closest_speed);
        return false;
    }

    if(ioctl(socketFD, TIOCSSERIAL, &ss) < 0)
    {
        if(DEBUG_PRINT == true)
            printf("[PortHandler::setBaudDevisor] TIOCSSERIAL failed!\n");
        return false;
    }
    byteTransferTime = (1000.0 / (double)speed) * 10.0;
    return true;
}

int PortHandler::getBaud(int baud)
{
    switch (baud)
    {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    case 460800:
        return B460800;
    case 500000:
        return B500000;
    case 576000:
        return B576000;
    case 921600:
        return B921600;
    case 1000000:
        return B1000000;
    case 1152000:
        return B1152000;
    case 1500000:
        return B1500000;
    case 2000000:
        return B2000000;
    case 2500000:
        return B2500000;
    case 3000000:
        return B3000000;
    case 3500000:
        return B3500000;
    case 4000000:
        return B4000000;
    default:
        return -1;
    }
}

bool PortHandler::setupSerialPort(int baud)
{
    struct termios newtio;

    socketFD = open(portName, O_RDWR|O_NOCTTY|O_NONBLOCK);
    if(socketFD < 0)
    {
        if(DEBUG_PRINT == true)
            printf("[PortHandler::setupSerialPort] Error opening serial port!\n");
        return false;
    }

    bzero(&newtio, sizeof(newtio)); // clear struct for new port settings

    newtio.c_cflag = baud | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag      = 0;
    newtio.c_lflag      = 0;
    newtio.c_cc[VTIME]  = 0;
    newtio.c_cc[VMIN]   = 0;

    // clean the buffer and activate the settings for the port
    tcflush(socketFD, TCIFLUSH);
    tcsetattr(socketFD, TCSANOW, &newtio);

    byteTransferTime = (1000.0 / (double)portBaudRate) * 10.0;
    return true;
}

bool PortHandler::openPort()
{
    return changeBaudRate(portBaudRate);
}

void PortHandler::closePort()
{
    if(socketFD != -1)
        close(socketFD);
    socketFD = -1;
}

void PortHandler::clearPort()
{
    tcflush(socketFD, TCIOFLUSH);
}

int PortHandler::writePort(unsigned char* packet, int packet_len)
{
    return write(socketFD, packet, packet_len);
}

int PortHandler::readPort(unsigned char* packet, int packet_len)
{
    return read(socketFD, packet, packet_len);
}




