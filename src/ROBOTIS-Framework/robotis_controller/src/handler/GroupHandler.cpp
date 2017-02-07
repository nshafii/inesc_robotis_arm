/*
 * GroupHandler.cpp
 *
 *  Created on: 2015. 9. 24.
 *      Author: zerom
 */


#include "../../include/handler/GroupHandler.h"

using namespace ROBOTIS;

GroupHandler::GroupHandler(RobotisController *controller)
    : robotisController(controller), comPort(0), packetHandler(0)
{ }

bool GroupHandler::pushBulkRead(int id, int start_addr, int length)
{
    if(start_addr <= 0 || length < 0)
        return false;

    if(length == 0)
    {
        length = robotisController->getDevice(id)->getAddrLength(start_addr);
        if(length < 0)
            return false;
    }

    if(bulkReadData.size() != 0)
    {
        for(unsigned int i = 0; i < bulkReadData.size(); i++)
        {
            if(bulkReadData[i].id == id)
                return false;   // already exist.
        }

        if(comPort->getPortName() != robotisController->getDevice(id)->getSerialPort()->getPortName())
            return false;
        if(robotisController->getDevice(bulkReadData[0].id)->PROTOCOL_VERSION != robotisController->getDevice(id)->PROTOCOL_VERSION)
            return false;
    }
    else
    {
        comPort = robotisController->getDevice(id)->getSerialPort();
        packetHandler = PacketHandler::getPacketHandler(robotisController->getDevice(id)->PROTOCOL_VERSION);
    }

    BulkReadData data;
    data.id = id;
    data.startAddr = start_addr;
    data.dataLength = length;
    data.data = new unsigned char[length];

    data.error = 0;
    data.commResult = -1;

    bulkReadData.push_back(data);

    return true;
}

bool GroupHandler::deleteBulkRead(int id)
{
    if(bulkReadData.size() != 0)
    {
        for(unsigned int i = 0; i < bulkReadData.size(); i++)
        {
            if(bulkReadData[i].id == id)
                bulkReadData.erase(bulkReadData.begin() + i);
        }
    }
}

bool GroupHandler::changeBulkRead(int id, int start_addr, int length)
{
    if(start_addr <= 0 || length < 0)
        return false;

    if(length == 0)
    {
        length = robotisController->getDevice(id)->getAddrLength(start_addr);
        if(length < 0)
            return false;
    }

	if(bulkReadData.size() == 0)
		return false;

	for(unsigned int i = 0; i < bulkReadData.size(); i++)
	{
		if(bulkReadData[i].id == id)
		{
			bulkReadData[i].startAddr = start_addr;
			bulkReadData[i].dataLength = length;
			if(bulkReadData[i].data != 0)
				delete[] bulkReadData[i].data;
			bulkReadData[i].data = new unsigned char[length];

			return true;
		}
	}

	return false;
}

void GroupHandler::clearBulkRead()
{
	if(bulkReadData.size() != 0)
	{
		for(unsigned int i = 0; i < bulkReadData.size(); i++)
		{
			if(bulkReadData[i].data != 0)
				delete[] bulkReadData[i].data;
		}
		bulkReadData.clear();
		comPort = 0;
		packetHandler = 0;
	}
}

void GroupHandler::runBulkRead()
{
	if(bulkReadData.size() == 0)
		return;

	for(unsigned int i = 0; i < bulkReadData.size(); i++)
		bulkReadData[i].commResult = -1;

	packetHandler->bulkReadTxPacket(comPort, bulkReadData);
	packetHandler->bulkReadRxPacket(comPort, bulkReadData);
}

bool GroupHandler::getReadData(int id, int addr, long *data, int length)
{
    if(addr <= 0 || length < 0){
    	return false;
    }

    if(length == 0)
    {
    	length = robotisController->getDevice(id)->getAddrLength(addr);
        if(length < 0){
        	return false;

        }
    }

    if(bulkReadData.size() == 0){
    	return false;
    }
    for(unsigned int i = 0; i < bulkReadData.size(); i++)
    {
        if(bulkReadData[i].id == id)
        {
            if (bulkReadData[i].commResult == -1 ||
                addr < bulkReadData[i].startAddr ||
                bulkReadData[i].startAddr + bulkReadData[i].dataLength - length < addr)
            {
            	return false;
            }

            switch(length)
            {
            case 1:
                *data = bulkReadData[i].data[addr - bulkReadData[i].startAddr];
                break;
            case 2:
                *data = DXL_MAKEWORD(bulkReadData[i].data[addr - bulkReadData[i].startAddr],
                                     bulkReadData[i].data[addr - bulkReadData[i].startAddr + 1]);
                break;
            case 4:
                *data = DXL_MAKEDWORD(DXL_MAKEWORD(bulkReadData[i].data[addr - bulkReadData[i].startAddr],
                                                   bulkReadData[i].data[addr - bulkReadData[i].startAddr + 1]),
                                      DXL_MAKEWORD(bulkReadData[i].data[addr - bulkReadData[i].startAddr + 2],
                                                   bulkReadData[i].data[addr - bulkReadData[i].startAddr + 3]));
                break;
            default:
                return false;
            }
            return true;
        }
    }

    return false;
}


bool GroupHandler::getReadCurrent(int id, int addr, long *data, int length)
{
	int startAddr = 621;
	int length_current = 2;
	int dataLength = 2;

	if(addr <= 0 || length < 0){
    	std::cerr<<"false 1"<<std::endl;
    	return false;
    }

    if(length == 0)
    {
    	length = robotisController->getDevice(id)->getAddrLength(addr);
        if(length < 0){
        	std::cerr<<"false 2"<<std::endl;

        	return false;

        }
    }

    if(bulkReadData.size() == 0){
    	std::cerr<<"false 3"<<std::endl;

    	return false;
    }
    for(unsigned int i = 0; i < bulkReadData.size(); i++)
    {
        if(bulkReadData[i].id == id)
        {
            if (bulkReadData[i].commResult == -1 ||
                addr < startAddr)
            {
            	std::cerr<<"false 5"<<std::endl;
            	std::cerr<<"start Add "<<startAddr<<std::endl;

            	std::cerr<<"dataLength "<<dataLength <<std::endl;
            	std::cerr<<"bulckSize "<< bulkReadData.size()<< std::endl;
            	std::cerr<<"length "<< length << std::endl;
            	return false;
            }

            *data = DXL_MAKEWORD(bulkReadData[i].data[addr - startAddr],
                                                 bulkReadData[i].data[addr - startAddr + 1]);
            return true;
        }
    }

	std::cerr<<"false 6"<<std::endl;

    return false;
}

int GroupHandler::syncWrite(int start_addr, int data_length, unsigned char* param, int param_length)
{
    comPort = robotisController->getDevice(param[0])->getSerialPort();
    packetHandler = PacketHandler::getPacketHandler(robotisController->getDevice(param[0])->PROTOCOL_VERSION);
    return packetHandler->syncWrite(comPort, start_addr, data_length, param, param_length);
}
