/*
 * BulkReadData.h
 *
 *  Created on: 2015. 9. 24.
 *      Author: zerom
 */

#ifndef BULKREADDATA_H_
#define BULKREADDATA_H_


namespace ROBOTIS
{

class BulkReadData
{
public:
	int				id;
	int				startAddr;
	int				dataLength;
	int				error;
	int				commResult;
	unsigned char 	*data;

	BulkReadData() : id(0), startAddr(0), dataLength(0), error(0), commResult(-1), data(0) { }
};

}


#endif /* BULKREADDATA_H_ */
