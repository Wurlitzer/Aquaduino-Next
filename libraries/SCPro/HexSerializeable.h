/*
 * HexSerializeable.h
 *
 *  Created on: 19.06.2015
 *      Author: poppi
 */

#ifndef HEXSERIALIZEABLE_H_
#define HEXSERIALIZEABLE_H_
#include "stdint.h"
#include "HexConverter.h"
#include "BinMessageParser.h"

typedef uint32_t u_int32_t;


class CHexSerializeable
{

public:
	virtual uint32_t toHex( char* hexBuf,const uint32_t hexbufLen,const uint32_t startPos,HexConverter* hexConverter,BinMessageParser* binMessageParser)=0;
	virtual uint32_t fromHex( char* hexBuf,const uint32_t hexbufLen,const uint32_t startPos,HexConverter* hexConverter,BinMessageParser* binMessageParser)=0;
	virtual ~CHexSerializeable();

	uint32_t getSize();
};


#endif /* HEXSERIALIZEABLE_H_ */
