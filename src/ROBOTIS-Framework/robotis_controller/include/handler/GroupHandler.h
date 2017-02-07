/*
 * GroupHandler.h
 *
 *  Created on: 2015. 9. 24.
 *      Author: zerom
 */

#ifndef GROUPHANDLER_H_
#define GROUPHANDLER_H_


#include <vector>
#include <iostream>

#include "../handler/BulkReadData.h"
#include "../RobotisController.h"

namespace ROBOTIS
{

class GroupHandler
{
private:
    RobotisController   *robotisController;
    PortHandler         *comPort;
    PacketHandler       *packetHandler;
	std::vector<BulkReadData> bulkReadData;

public:
	GroupHandler(RobotisController *controller);
	virtual ~GroupHandler() { }

	bool pushBulkRead(int id, int start_addr, int length = 0);
	bool deleteBulkRead(int id);
	bool changeBulkRead(int id, int start_addr, int length = 0);
	void clearBulkRead();

	void runBulkRead();

	bool getReadData(int id, int addr, long *data, int length = 0);
	bool getReadCurrent(int id, int addr, long *data, int length = 0);

	int syncWrite(int start_addr, int data_length, unsigned char* param, int param_length);
};

}


#endif /* GROUPHANDLER_H_ */
