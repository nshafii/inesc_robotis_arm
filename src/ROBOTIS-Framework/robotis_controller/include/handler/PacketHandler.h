/*
 *=====================================================
 * File   :  PacketHandler.h
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

#ifndef PACKETHANDLER_H_
#define PACKETHANDLER_H_

#include <vector>

#include "../handler/BulkReadData.h"
#include "PortHandler.h"

#define MAXNUM_TXPACKET     (4*1024)
#define MAXNUM_RXPACKET     (4*1024)

#define MAX_ID          0xFC
#define BROADCAST_ID    0xFE

///////////////// MACRO for Control Table Value /////////////////
#define DXL_MAKEWORD(a, b)      ((unsigned short)(((unsigned char)(((unsigned long)(a)) & 0xff)) | ((unsigned short)((unsigned char)(((unsigned long)(b)) & 0xff))) << 8))
#define DXL_MAKEDWORD(a, b)     ((unsigned int)(((unsigned short)(((unsigned long)(a)) & 0xffff)) | ((unsigned int)((unsigned short)(((unsigned long)(b)) & 0xffff))) << 16))
#define DXL_LOWORD(l)           ((unsigned short)(((unsigned long)(l)) & 0xffff))
#define DXL_HIWORD(l)           ((unsigned short)((((unsigned long)(l)) >> 16) & 0xffff))
#define DXL_LOBYTE(w)           ((unsigned char)(((unsigned long)(w)) & 0xff))
#define DXL_HIBYTE(w)           ((unsigned char)((((unsigned long)(w)) >> 8) & 0xff))

///////////////// Instruction for DXL Protocol /////////////////
#define INST_PING               1
#define INST_READ               2
#define INST_WRITE              3
#define INST_REG_WRITE          4
#define INST_ACTION             5
#define INST_FACTORY_RESET      6
#define INST_SYNC_WRITE         131     // 0x83
#define INST_BULK_READ          146     // 0x92
// --- Only for 2.0 --- //
#define INST_REBOOT             8
#define INST_STATUS             85      // 0x55
#define INST_SYNC_READ          130     // 0x82
#define INST_BULK_WRITE         147     // 0x93

// Communication Result
#define COMM_SUCCESS        0       // tx or rx packet communication success
#define COMM_TX_FAIL        -1001   // Failed transmit instruction packet
#define COMM_RX_FAIL        -1002   // Failed get status packet
#define COMM_TX_ERROR       -2000   // Incorrect instruction packet
#define COMM_RX_WAITING     -3000   // Now recieving status packet
#define COMM_RX_TIMEOUT     -3001   // There is no status packet
#define COMM_RX_CORRUPT     -3002   // Incorrect status packet
#define COMM_NOT_AVAILABLE  -9000   //

namespace ROBOTIS
{


enum LENGTH_TYPE
{
    LENGTH_1BYTE    = 1,
    LENGTH_2BYTE    = 2,
    LENGTH_4BYTE    = 4
};

class PacketHandler
{
protected:
    PacketHandler();

public:
    static PacketHandler *getPacketHandler(float protocol_ver = 2.0);
    virtual ~PacketHandler() { };

    virtual int txPacket(PortHandler *port, unsigned char *txpacket) = 0;
    virtual int rxPacket(PortHandler *port, unsigned char *rxpacket) = 0;
    virtual int txRxPacket(PortHandler *port, unsigned char *txpacket, unsigned char *rxpacket, int *error = 0) = 0;

    virtual int bulkReadTxPacket(PortHandler *port, std::vector<BulkReadData>& data) = 0;
    virtual int bulkReadRxPacket(PortHandler *port, std::vector<BulkReadData>& data) = 0;

    virtual int ping(PortHandler *port, int id, int *error = 0) = 0;

    virtual int read(PortHandler *port, int id, int address, int length, unsigned char *data, int *error = 0) = 0;
    virtual int write(PortHandler *port, int id, int address, int length, unsigned char *data, int *error = 0) = 0;
    virtual int syncWrite(PortHandler *port, int start_addr, int data_length, unsigned char* param, int param_length) = 0;
};

}


#endif /* PACKETHANDLER_H_ */
