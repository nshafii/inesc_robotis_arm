/*
 *=====================================================
 * File   :  PortHandler.h
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

#ifndef PORTHANDLER_H_
#define PORTHANDLER_H_


namespace ROBOTIS {

class PortHandler {
private:
    int     socketFD;
    int     portBaudRate;
    char    portName[20];

    double  packetStartTime;
    double  packetWaitTime;
    double  byteTransferTime;

    int     getBaud(int baud);
    bool    setupSerialPort(int baud);
    bool    setBaudDevisor(int speed);

public:
    static const int DEFAULT_BAUDRATE = 1000000;
    bool    DEBUG_PRINT;

    PortHandler(const char* port_name);
    virtual ~PortHandler();

    void    setPacketTimeout(int packet_len);
    void    setPacketTimeout(double msec);
    bool    isPacketTimeout();
    double  getPacketTime();
    double  getCurrentTime();

    bool    openPort();
    void    closePort();
    void    clearPort();
    void    setPortName(const char* port_name);
    char*   getPortName();
    void    setBaudRate(int baudrate);
    int     getBaudRate();
    bool    changeBaudRate(int baudrate);

    int     getBytesAvailable();

    int     writePort(unsigned char* packet, int packet_len);
    int     readPort(unsigned char* packet, int packet_len);
};

}


#endif /* PORTHANDLER_H_ */
