/*
 * HexConverter.h
 *
 *  Created on: 10.01.2015
 *      Author: poppi
 */

#ifndef HEXCONVERTER_H_
#define HEXCONVERTER_H_
#include "stdint.h"

typedef uint32_t u_int32_t;
typedef uint16_t u_int16_t;

class HexConverter
{
public:

	unsigned int strlen(const  char* value);

	int decodeFromHex(const  char* hexStr,const unsigned int hexStrLen, char* binStr,const unsigned int startPos,const unsigned int binStrSize);
	int encodeToHex(const  char* binStr,const unsigned int startPos,const unsigned int endPos, char* hexStr,const unsigned int hexStrLength, uint32_t hexPosStart);
	int estimatedHexBufLen(unsigned int binStrlen);
	int estimatedBinBufLen( int hexStrlen);

};


#endif /* HEXCONVERTER_H_ */
