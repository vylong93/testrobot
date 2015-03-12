/*
 * data_manipulation.h
 *
 *  Created on: Mar 12, 2015
 *      Author: VyLong
 */

#ifndef DATA_MANIPULATION_H_
#define DATA_MANIPULATION_H_

//*****************************************************************************
// Convert 4 bytes of an array to a 32-bit value
// @param x: The converted byte array
// @return : a 32-bit value.
//*****************************************************************************
#define construct4Byte(x)	((*x << 24) | (*(x+1) << 16) |	\
							(*(x+2) << 8) | *(x+3))


#define parse32bitTo4Bytes(x, y)	{*(x) = y >> 24; \
									 *(x + 1) = y >> 16; \
									 *(x + 2) = y >> 8; \
									 *(x + 3) = y;}

#define construct2Byte(x)	((*x << 8) | *(x+1))

#define parse16bitTo2Bytes(x, y)	{*(x) = y >> 8; \
									 *(x + 1) = y;}

#endif /* DATA_MANIPULATION_H_ */
