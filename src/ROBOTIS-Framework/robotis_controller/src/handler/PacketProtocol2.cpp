/*
 *=====================================================
 * File   :  PacketProtocol2.cpp
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

#include "../../include/handler/PacketProtocol2.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

///////////////// for Protocol 2.0 Packet /////////////////
#define PKT_HEADER0             0
#define PKT_HEADER1             1
#define PKT_HEADER2             2
#define PKT_RESERVED            3
#define PKT_ID                  4
#define PKT_LENGTH_L            5
#define PKT_LENGTH_H            6
#define PKT_INSTRUCTION         7
#define PKT_ERROR               8
#define PKT_PARAMETER0          8

///////////////// Protocol 2.0 Error bit /////////////////
#define ERRNUM_RESULT_FAIL      1       // Failed to process the instruction packet.
#define ERRNUM_INSTRUCTION      2       // Instruction error
#define ERRNUM_CRC              3       // CRC check error
#define ERRNUM_DATA_RANGE       4       // Data range error
#define ERRNUM_DATA_LENGTH      5       // Data length error
#define ERRNUM_DATA_LIMIT       6       // Data limit error
#define ERRNUM_ACCESS           7       // Access error

#define ERRBIT_ALERT            128     //When the device has a problem, this bit is set to 1. Check "Device Status Check" value.

using namespace ROBOTIS;

PacketProtocol2 *PacketProtocol2::uniqueInstance = new PacketProtocol2();

PacketProtocol2::PacketProtocol2() : PacketHandler()
{

}

unsigned short PacketProtocol2::updateCRC(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
    unsigned short i, j;
    unsigned short crc_table[256] = {0x0000,
    0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
    0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
    0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
    0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
    0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
    0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
    0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
    0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
    0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
    0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
    0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
    0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
    0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
    0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
    0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
    0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
    0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
    0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
    0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
    0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
    0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
    0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
    0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
    0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
    0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
    0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
    0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
    0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
    0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
    0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
    0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
    0x820D, 0x8207, 0x0202 };

    for(j = 0; j < data_blk_size; j++)
    {
        i = ((unsigned short)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}
void PacketProtocol2::addStuffing(unsigned char *packet)
{
    int i = 0, index = 0;
    int packet_length_in = DXL_MAKEWORD(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H]);
    int packet_length_out = packet_length_in;
    unsigned char temp[MAXNUM_TXPACKET] = {0};

    memcpy(temp, packet, PKT_LENGTH_H+1);    // FF FF FD XX ID LEN_L LEN_H
    index = PKT_INSTRUCTION;
    for( i = 0; i < packet_length_in - 2; i++)  // except CRC
    {
        temp[index++] = packet[i+PKT_INSTRUCTION];
        if(packet[i+PKT_INSTRUCTION] == 0xFD && packet[i+PKT_INSTRUCTION-1] == 0xFF && packet[i+PKT_INSTRUCTION-2] == 0xFF)
        {   // FF FF FD
            temp[index++] = 0xFD;
            packet_length_out++;
        }
    }
    temp[index++] = packet[PKT_INSTRUCTION+packet_length_in-2];
    temp[index++] = packet[PKT_INSTRUCTION+packet_length_in-1];


    //////////////////////////
    if(packet_length_in != packet_length_out)
        packet = (unsigned char*)realloc(packet, index * sizeof(unsigned char));

    ///////////////////////////

    memcpy(packet, temp, index);
    packet[PKT_LENGTH_L] = DXL_LOBYTE(packet_length_out);
    packet[PKT_LENGTH_H] = DXL_HIBYTE(packet_length_out);
}
void PacketProtocol2::removeStuffing(unsigned char *packet)
{
    int i = 0, index = 0;
    int packet_length_in = DXL_MAKEWORD(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H]);
    int packet_length_out = packet_length_in;

    index = PKT_INSTRUCTION;
    for( i = 0; i < packet_length_in - 2; i++)  // except CRC
    {
        if(packet[i+PKT_INSTRUCTION] == 0xFD && packet[i+PKT_INSTRUCTION+1] == 0xFD && packet[i+PKT_INSTRUCTION-1] == 0xFF && packet[i+PKT_INSTRUCTION-2] == 0xFF)
        {   // FF FF FD FD
            packet_length_out--;
            i++;
        }
        packet[index++] = packet[i+PKT_INSTRUCTION];
    }
    packet[index++] = packet[PKT_INSTRUCTION+packet_length_in-2];
    packet[index++] = packet[PKT_INSTRUCTION+packet_length_in-1];

    packet[PKT_LENGTH_L] = DXL_LOBYTE(packet_length_out);
    packet[PKT_LENGTH_H] = DXL_HIBYTE(packet_length_out);
}

int PacketProtocol2::txPacket(PortHandler *port, unsigned char *txpacket)
{
    int packet_tx_len, real_tx_len;
    int length;
    unsigned short crc = 0;

    //TODO: check port using

    // Byte stuffing
    addStuffing(txpacket);
    length = DXL_MAKEWORD(txpacket[PKT_LENGTH_L], txpacket[PKT_LENGTH_H]);

    // Check Max packet length
    if(length > MAXNUM_TXPACKET)
    {
        //TODO: port free?
        return COMM_TX_ERROR;
    }

    // Make Packet Header
    txpacket[PKT_HEADER0]   = 0xFF;
    txpacket[PKT_HEADER1]   = 0xFF;
    txpacket[PKT_HEADER2]   = 0xFD;
    txpacket[PKT_RESERVED]  = 0x00; // reserved

    // Add CRC16
    crc = updateCRC(0, txpacket, length+PKT_LENGTH_H+1-2);  // -2 : CRC16
    txpacket[length+PKT_LENGTH_H-1] = DXL_LOBYTE(crc);      // last - 1
    txpacket[length+PKT_LENGTH_H-0] = DXL_HIBYTE(crc);      // last - 0

    // Tx Packet
    port->clearPort();
    packet_tx_len = length + PKT_LENGTH_H + 1;
    real_tx_len = port->writePort(txpacket, packet_tx_len);
    if(packet_tx_len != real_tx_len)
    {
        //TODO: port free?
        return COMM_TX_FAIL;
    }

    return COMM_SUCCESS;
}

int PacketProtocol2::rxPacket(PortHandler *port, unsigned char *rxpacket)
{
    int rx_length = 0, wait_length = PKT_LENGTH_H + 4 + 1;  // 4 : INST ERROR CHKSUM_L CHKSUM_H
    int i;
    int result = COMM_RX_FAIL;
    unsigned short crc = 0;

    while(1)
    {
        rx_length += port->readPort(&rxpacket[rx_length], wait_length - rx_length);
        if(rx_length >= wait_length)    // wait_length minimum : 11
        {
            // find packet header
            for(i = 0; i < (rx_length - 2); i++)
            {
                if(rxpacket[i] == 0xFF && rxpacket[i+1] == 0xFF && rxpacket[i+2] == 0xFD)
                    break;
            }

            if(i == 0)
            {
                // check length
                wait_length = DXL_MAKEWORD(rxpacket[PKT_LENGTH_L], rxpacket[PKT_LENGTH_H]) + PKT_LENGTH_H + 1;
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

                // check CRC16
                crc = DXL_MAKEWORD(rxpacket[wait_length-2], rxpacket[wait_length-1]);
                if(updateCRC(0, rxpacket, wait_length-2) == crc) // -2 : except CRC16
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

    if(result == COMM_SUCCESS)
        removeStuffing(rxpacket);

    //TODO: port free?
    return result;
}

int PacketProtocol2::txRxPacket(PortHandler *port, unsigned char *txpacket, unsigned char *rxpacket, int *error)
{
    int result = COMM_TX_FAIL;

    //TODO: check bus idle?

    result = txPacket(port, txpacket);

    // Check Tx packet result
    if(result != COMM_SUCCESS)
        return result;

    // Set Rx timeout
    if(txpacket[PKT_INSTRUCTION] == INST_READ)
        port->setPacketTimeout(DXL_MAKEWORD(txpacket[PKT_PARAMETER0+2], txpacket[PKT_PARAMETER0+3]) + 11);
    else
        port->setPacketTimeout(PKT_LENGTH_H + 4 + 1); // 4 : INST ERROR CHKSUM_L CHKSUM_H

    // Broadcast ID && !BulkRead == no need to wait for a rxpacket
    if(txpacket[PKT_ID] == BROADCAST_ID && txpacket[PKT_INSTRUCTION] != INST_BULK_READ)
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

int PacketProtocol2::bulkReadTxPacket(PortHandler *port, std::vector<BulkReadData>& data)
{
	int result = COMM_TX_FAIL;

	if(data.size() == 0)
		return result;

	int wait_length = 0;
	int num = data.size();
    int param_length = data.size() * 5; // 5: ID ADDR_L ADDR_H LENGTH_L LENGTH_H
	int pkt_length = param_length + 3;	// 3: INST CRC_L CRC_H
	unsigned char txpacket[MAXNUM_TXPACKET] = {0};

	txpacket[PKT_ID]			= (unsigned char)BROADCAST_ID;
	txpacket[PKT_LENGTH_L]		= (unsigned char)DXL_LOBYTE(pkt_length);
	txpacket[PKT_LENGTH_H]		= (unsigned char)DXL_HIBYTE(pkt_length);
	txpacket[PKT_INSTRUCTION]	= INST_BULK_READ;
	for(unsigned int i = 0; i < data.size(); i++)
	{
		txpacket[PKT_PARAMETER0+5*i+0] = data[i].id;
		txpacket[PKT_PARAMETER0+5*i+1] = DXL_LOBYTE(data[i].startAddr);
		txpacket[PKT_PARAMETER0+5*i+2] = DXL_HIBYTE(data[i].startAddr);
		txpacket[PKT_PARAMETER0+5*i+3] = DXL_LOBYTE(data[i].dataLength);
		txpacket[PKT_PARAMETER0+5*i+4] = DXL_HIBYTE(data[i].dataLength);
	}

	for(int n = 0; n < num; n++)
		wait_length += data[n].dataLength + 11;

	result = txPacket(port, txpacket);

	if(result == 0)
		port->setPacketTimeout(wait_length);

	return result;
}

int PacketProtocol2::bulkReadRxPacket(PortHandler *port, std::vector<BulkReadData>& data)
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

		memcpy(data[n].data, &rxpacket[PKT_PARAMETER0+1], data[n].dataLength);
	}

	return result;
}

int PacketProtocol2::ping(PortHandler *port, int id, int *error)
{ return ping(port, id, 0, 0, error); }

int PacketProtocol2::ping(PortHandler *port, int id, int *model_num, int *firm_ver, int *error)
{
    int result = COMM_TX_FAIL;
    unsigned char txpacket[10]  = {0};
    unsigned char rxpacket[14]  = {0};

    if(id > 253)
        return result;

    txpacket[PKT_ID]            = (unsigned char)id;
    txpacket[PKT_LENGTH_L]      = 0x03;
    txpacket[PKT_LENGTH_H]      = 0x00;
    txpacket[PKT_INSTRUCTION]   = INST_PING;

    result = txRxPacket(port, txpacket, rxpacket, error);
    if(result == COMM_SUCCESS && id != BROADCAST_ID)
    {
        if(model_num != 0)
            *model_num = DXL_MAKEWORD(rxpacket[PKT_PARAMETER0+1], rxpacket[PKT_PARAMETER0+2]);
        if(firm_ver != 0)
            *firm_ver = rxpacket[PKT_PARAMETER0+3];
    }

    return result;
}

int PacketProtocol2::read(PortHandler *port, int id, int address, int length, unsigned char *data, int *error)
{
    int result = COMM_TX_FAIL;
    unsigned char txpacket[14]  = {0};
    unsigned char* rxpacket     = new unsigned char[length + 11 + (length/3)]; // length/3 : consider stuffing

    txpacket[PKT_ID]            = (unsigned char)id;
    txpacket[PKT_LENGTH_L]      = 0x07;
    txpacket[PKT_LENGTH_H]      = 0x00;
    txpacket[PKT_INSTRUCTION]   = INST_READ;
    txpacket[PKT_PARAMETER0+0]  = (unsigned char)DXL_LOBYTE(address);
    txpacket[PKT_PARAMETER0+1]  = (unsigned char)DXL_HIBYTE(address);
    txpacket[PKT_PARAMETER0+2]  = (unsigned char)DXL_LOBYTE(length);
    txpacket[PKT_PARAMETER0+3]  = (unsigned char)DXL_HIBYTE(length);

    result = txRxPacket(port, txpacket, rxpacket, error);
    if(result == COMM_SUCCESS && id != BROADCAST_ID)
        memcpy(data, &rxpacket[PKT_PARAMETER0+1], length);

    delete[] rxpacket;
    return result;
}

int PacketProtocol2::write(PortHandler *port, int id, int address, int length, unsigned char *data, int *error)
{
    int result = COMM_TX_FAIL;
    unsigned char* txpacket     = new unsigned char[length + 12];
    unsigned char rxpacket[12]  = {0};

    txpacket[PKT_ID]            = (unsigned char)id;
    txpacket[PKT_LENGTH_L]      = DXL_LOBYTE(length+5);
    txpacket[PKT_LENGTH_H]      = DXL_HIBYTE(length+5);
    txpacket[PKT_INSTRUCTION]   = INST_WRITE;
    txpacket[PKT_PARAMETER0+0]  = (unsigned char)DXL_LOBYTE(address);
    txpacket[PKT_PARAMETER0+1]  = (unsigned char)DXL_HIBYTE(address);

    memcpy(&txpacket[PKT_PARAMETER0+2], data, length);

    result = txRxPacket(port, txpacket, rxpacket, error);

    delete[] txpacket;
    return result;
}

int PacketProtocol2::syncWrite(PortHandler *port, int start_addr, int data_length, unsigned char* param, int param_length)
{
    int result = COMM_TX_FAIL;
    int pkt_length = param_length + 7;  // 7 : INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CHKSUM_L CHKSUM_H
    unsigned char* txpacket     = new unsigned char[pkt_length+7];
    unsigned char rxpacket[12]  = {0};

    txpacket[PKT_ID]            = BROADCAST_ID;
    txpacket[PKT_LENGTH_L]      = DXL_LOBYTE(pkt_length);
    txpacket[PKT_LENGTH_H]      = DXL_HIBYTE(pkt_length);
    txpacket[PKT_INSTRUCTION]   = INST_SYNC_WRITE;
    txpacket[PKT_PARAMETER0+0]  = DXL_LOBYTE(start_addr);
    txpacket[PKT_PARAMETER0+1]  = DXL_HIBYTE(start_addr);
    txpacket[PKT_PARAMETER0+2]  = DXL_LOBYTE(data_length);
    txpacket[PKT_PARAMETER0+3]  = DXL_HIBYTE(data_length);
    memcpy(&txpacket[PKT_PARAMETER0+4], param, param_length);

    result = txRxPacket(port, txpacket, rxpacket, 0);
    delete[] txpacket;

    return result;
}
