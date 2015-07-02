/*
 * BinMessageParser.h
 *
 *  Created on: 14.01.2015
 *      Author: poppi
 */

#include "stdint.h"
#ifndef BINMESSAGEPARSER_H_
#define BINMESSAGEPARSER_H_


class BinMessageParser
{
public:
	unsigned int fromInt8ToBin(const char value,char* binStr,const unsigned int bufLen,const unsigned int startPos);
	unsigned int fromInt16ToBin(const unsigned short value,char* binStr,const unsigned int bufLen,const unsigned int startPos);
	unsigned int fromInt32ToBin(const unsigned long value,char* binStr,const unsigned int bufLen,const unsigned int startPos);

	unsigned int fromString8ToBin(const char* value,char* binStr,const unsigned int bufLen,const unsigned int startPos);
	unsigned int fromString32ToBin(const char* value,char* binStr,const unsigned int bufLen,const unsigned int startPos);




	uint8_t fromBinToInt8(const char* binStr,const unsigned int bufLen,const unsigned int startPos);
	uint16_t fromBinToInt16(const char* binStr,const unsigned int bufLen,const unsigned int startPos);
	uint32_t fromBinToInt32(const char* binStr,const unsigned int bufLen,const unsigned int startPos);

	unsigned int fromBinToString8(char* value,const unsigned int maxStrlen,const char* binStr,const unsigned int bufLen,const unsigned int startPos);
	unsigned int fromBinToString32(char* value,const unsigned int maxStrlen,const char* binStr,const unsigned int bufLen,const unsigned int startPos);


};


#endif /* BINMESSAGEPARSER_H_ */
