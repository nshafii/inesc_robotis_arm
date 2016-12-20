/*
 *=====================================================
 * File   :  PacketProtocol1.cpp
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

#include "../../include/handler/PacketProtocol1.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
using namespace std;
///////////////// for Protocol 1.0 Packet /////////////////
#define PKT_HEADER0             0
#define PKT_HEADER1             1
#define PKT_ID                  2
#define PKT_LENGTH              3
#define PKT_INSTRUCTION         4
#define PKT_ERROR               4
#define PKT_PARAMETER0          5

///////////////// Protocol 1.0 Error bit /////////////////
#define ERRBIT_VOLTAGE          1       // Supplied voltage is out of the range (operating volatage set in the control table)
#define ERRBIT_ANGLE            2       // Goal position is written out of the range (from CW angle limit to CCW angle limit)
#define ERRBIT_OVERHEAT         4       // Temperature is out of the range (operating temperature set in the control table)
#define ERRBIT_RANGE            8       // Command(setting value) is out of the range for use.
#define ERRBIT_CHECKSUM         16      // Instruction packet checksum is incorrect.
#define ERRBIT_OVERLOAD         32      // The current load cannot be controlled by the set torque.
#define ERRBIT_INSTRUCTION      64      // Undefined instruction or delivering the action command without the reg_write command.

using namespace ROBOTIS;

PacketProtocol1 *PacketProtocol1::uniqueInstance = new PacketProtocol1();

PacketProtocol1::PacketProtocol1() : PacketHandler()
{

}

int PacketProtocol1::txPacket(PortHandler *port, unsigned char *txpacket)
{
    int packet_tx_len, real_tx_len;
    int length;
    unsigned char checksum = 0;

    //TODO: check port using

    length = txpacket[PKT_LENGTH];

    // Check Max packet length
    if(length > MAXNUM_TXPACKET)
    {
        //TODO: port free?
        return COMM_TX_ERROR;
    }

    // Make Packet Header
    txpacket[PKT_HEADER0]   = 0xFF;
    txpacket[PKT_HEADER1]   = 0xFF;

    // Add Checksum
    for(int _idx = 0; _idx < txpacket[PKT_LENGTH] + 1; _idx++)
        checksum += txpacket[_idx + 2];
    txpacket[txpacket[PKT_LENGTH] + 3] = ~checksum;

    // Tx Packet
    port->clearPort();
    packet_tx_len = length + 4;
    real_tx_len = port->writePort(txpacket, packet_tx_len);
    if(packet_tx_len != real_tx_len)
    {
        //TODO: port free?
        return COMM_TX_FAIL;
    }

    return COMM_SUCCESS;
}

int PacketProtocol1::rxPacket(PortHandler *port, unsigned char *rxpacket)
{
    int rx_length = 0, wait_length = PKT_LENGTH + 2 + 1;  // 2 : ERROR CHKSUM
    int i;
    int result = COMM_TX_FAIL;
    unsigned char checksum = 0;

    while(1)
    {
        rx_length += port->readPort(&rxpacket[rx_length], wait_length - rx_length);
        if(rx_length >= wait_length)    // wait_length minimum : 6
        {
            // find packet header
            for(i = 0; i < (rx_length - 1); i++)
            {
                if(rxpacket[i] == 0xFF && rxpacket[i+1] == 0xFF)
                    break;
            }

            if(i == 0)
            {
                // check length
                wait_length = rxpacket[PKT_LENGTH] + PKT_LENGTH + 1;
                if(rx_length < wait_length)
                {
                    // check timeout
                    if(port->isPacketTimeout() == true)
                    {
                        if(rx_length == 0)
                            result = COMM_RX_TIMEOUT;
                        else
                            result = COMM_RX_CORRUPT;

                        //TODO: bus free?

                        break;
                    }

                    continue;
                }

                // check checksum
                for(int _idx = 0; _idx < rxpacket[PKT_LENGTH] + 1; _idx++)
                    checksum += rxpacket[_idx + 2];
                checksum = ~checksum;
                if(rxpacket[rxpacket[PKT_LENGTH] + 3] == checksum)
                    result = COMM_SUCCESS;
                else
                    result = COMM_RX_CORRUPT;

                //TODO: bus free?

                break;
            }
            else
            {
                // remove unnecessary packets
                memcpy(&rxpacket[0], &rxpacket[i], rx_length - i);
                rx_length -= i;
            }
        }
        else
        {
            // check timeout
            if(port->isPacketTimeout() == true)
            {
                if(rx_length == 0)
                    result = COMM_RX_TIMEOUT;
                else
                    result = COMM_RX_CORRUPT;

                //TODO: bus free?

                break;
            }
        }
    }

    //TODO: port free?
    return result;
}

int PacketProtocol1::txRxPacket(PortHandler *port, unsigned char *txpacket, unsigned char *rxpacket, int *error)
{
    int result = COMM_TX_FAIL;

    //TODO: check bus idle?

    result = txPacket(port, txpacket);

    // Check Tx packet result
    if(result != COMM_SUCCESS)
        return result;

    // Set Rx timeout
    if(txpacket[PKT_INSTRUCTION] == INST_READ)
        port->setPacketTimeout(txpacket[PKT_PARAMETER0+1] + 6);
    else
        port->setPacketTimeout(PKT_LENGTH + 2 + 1); // 2 : ERROR CHKSUM

    // Broadcast ID && !BulkRead == no need to wait for a rxpacket
    if(txpacket[PKT_ID] == BROADCAST_ID &&
            (txpacket[PKT_INSTRUCTION] != INST_BULK_READ && txpacket[PKT_INSTRUCTION] != INST_SYNC_READ))
    {
        //TODO: bus free?
        return COMM_SUCCESS;
    }

    result = rxPacket(port, rxpacket);
    if((result == COMM_SUCCESS) && (txpacket[PKT_ID] != BROADCAST_ID) && (txpacket[PKT_ID] != rxpacket[PKT_ID]))
        result = rxPacket(port, rxpacket);   // ID does not match -> retry

    if(result == COMM_SUCCESS && txpacket[PKT_ID] != BROADCAST_ID)
    {
        if(error != 0)
            *error = (int)rxpacket[PKT_ERROR];
    }

    return result;
}

int PacketProtocol1::bulkReadTxPacket(PortHandler *port, std::vector<BulkReadData>& data)
{
    int result = COMM_TX_FAIL;

    if(data.size() == 0)
        return result;

    int wait_length = 0;
    int num = data.size();
    int param_length = data.size() * 3; // 3: ID ADDR LENGTH
    int pkt_length = param_length + 3;  // 3: INST PARAMETER0 CHKSUM
    unsigned char txpacket[MAXNUM_TXPACKET] = {0};

    txpacket[PKT_ID]            = (unsigned char)BROADCAST_ID;
    txpacket[PKT_LENGTH]        = pkt_length;
    txpacket[PKT_INSTRUCTION]   = INST_BULK_READ;
    txpacket[PKT_PARAMETER0]    = (unsigned char)0x0;
    for(unsigned int i = 0; i < data.size(); i++)
    {
        txpacket[PKT_PARAMETER0+3*i+1] = data[i].dataLength;
        txpacket[PKT_PARAMETER0+3*i+2] = data[i].id;
        txpacket[PKT_PARAMETER0+3*i+3] = data[i].startAddr;
    }

    for(int n = 0; n < num; n++)
        wait_length += data[n].dataLength + 6;

    result = txPacket(port, txpacket);

//    for(int cnt = 0; cnt < txpacket[PKT_LENGTH] + 3; cnt++)
//        fprintf(stderr, " 0x%02X", txpacket[cnt]);
//    fprintf(stderr, "\n");

    if(result == 0)
        port->setPacketTimeout(wait_length);

    return result;
}

int PacketProtocol1::bulkReadRxPacket(PortHandler *port, std::vector<BulkReadData>& data)
{
    int result = COMM_RX_FAIL;

    int num = data.size();
    unsigned char rxpacket[MAXNUM_RXPACKET] = {0};

    for(int n = 0; n < num; n++)
    {
        result = rxPacket(port, rxpacket);

        if(result == COMM_SUCCESS)
        {
            data[n].commResult = result;
            data[n].error = rxpacket[PKT_ERROR];
        }
        else
        {
            printf("[ID:%d] bulkread result : %d \n", data[n].id, result);
            data[n].commResult = result;
            return result;
        }

        memcpy(data[n].data, &rxpacket[PKT_PARAMETER0], data[n].dataLength);
    }

    return result;
}

int PacketProtocol1::ping(PortHandler *port, int id, int *error)
{
    int result = COMM_TX_FAIL;

    unsigned char txpacket[6]   = {0};
    unsigned char rxpacket[6]   = {0};

    txpacket[PKT_ID]            = (unsigned char)id;
    txpacket[PKT_LENGTH]        = 0x02;
    txpacket[PKT_INSTRUCTION]   = INST_PING;

    return txRxPacket(port, txpacket, rxpacket, error);
}

int PacketProtocol1::read(PortHandler *port, int id, int address, int length, unsigned char *data, int *error)
{
    int result = COMM_TX_FAIL;
    unsigned char txpacket[8]   = {0};
    unsigned char *rxpacket     = new unsigned char[length + 6];

    txpacket[PKT_ID]            = (unsigned char)id;
    txpacket[PKT_LENGTH]        = 0x04;
    txpacket[PKT_INSTRUCTION]   = INST_READ;
    txpacket[PKT_PARAMETER0+0]  = (unsigned char)address;
    txpacket[PKT_PARAMETER0+1]  = (unsigned char)length;

    result = txRxPacket(port, txpacket, rxpacket, error);
    if(result == COMM_SUCCESS && id != BROADCAST_ID)
        memcpy(data, &rxpacket[PKT_PARAMETER0], length);
    else
      cout<<"Here is the error of Nima 1"<<endl;

    delete[] rxpacket;
    return result;
}

int PacketProtocol1::write(PortHandler *port, int id, int address, int length, unsigned char *data, int *error)
{
    int result = COMM_TX_FAIL;
    unsigned char* txpacket     = new unsigned char[length+6];
    unsigned char rxpacket[6]   = {0};

    txpacket[PKT_ID]            = (unsigned char)id;
    txpacket[PKT_LENGTH]        = length+3;
    txpacket[PKT_INSTRUCTION]   = INST_WRITE;
    txpacket[PKT_PARAMETER0]    = (unsigned char)address;

    memcpy(&txpacket[PKT_PARAMETER0+1], data, length);

    result = txRxPacket(port, txpacket, rxpacket, error);

    if(!(result == COMM_SUCCESS && id != BROADCAST_ID))
      cout<<"Here is the error of Nima 2"<<endl;

    delete[] txpacket;
    return result;
}

int PacketProtocol1::syncWrite(PortHandler *port, int start_addr, int data_length, unsigned char* param, int param_length)
{
    int result = COMM_TX_FAIL;
    int pkt_length = param_length + 4;  // 4 : INST / START_ADDR / DATA_LEN / CHKSUM
    unsigned char* txpacket     = new unsigned char[pkt_length+4];  // 4 : HEADER0 / HEADER1 / ID / LENGTH
    unsigned char rxpacket[12]  = {0};

    if((pkt_length + 3) > 254)
        return result;

    txpacket[PKT_ID]            = BROADCAST_ID;
    txpacket[PKT_LENGTH]        = pkt_length;
    txpacket[PKT_INSTRUCTION]   = INST_SYNC_WRITE;
    txpacket[PKT_PARAMETER0+0]  = start_addr;
    txpacket[PKT_PARAMETER0+1]  = data_length;
    memcpy(&txpacket[PKT_PARAMETER0+2], param, param_length);

    result = txRxPacket(port, txpacket, rxpacket, 0);
    if(!(result == COMM_SUCCESS))
      cout<<"Here is the error of Nima 2"<<endl;

    free(txpacket);

    return result;
}
