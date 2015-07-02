/*
 * PutchannelRequestChannel.h
 *
 *  Created on: 19.06.2015
 *      Author: poppi
 */

#ifndef PUTCHANNELREQUESTCHANNEL_H_
#define PUTCHANNELREQUESTCHANNEL_H_
#include "HexSerializeable.h"
#include "stdint.h"

typedef uint32_t u_int32_t;
typedef uint16_t u_int16_t;
typedef uint8_t u_int8_t;

class CPutchannelRequestChannel:public CHexSerializeable
{
public:
	u_int8_t chName;
	u_int32_t value;
	u_int16_t changeDateOffset;

	CPutchannelRequestChannel();
	virtual ~CPutchannelRequestChannel();

	virtual uint32_t toHex( char* hexBuf,const uint32_t hexbufLen,const uint32_t startPos,HexConverter* hexConverter,BinMessageParser* binMessageParser);
	virtual uint32_t fromHex( char* hexBuf,const uint32_t hexbufLen,const uint32_t startPos,HexConverter* hexConverter,BinMessageParser* binMessageParser);

	uint32_t getSize();
};



#endif /* PUTCHANNELREQUESTCHANNEL_H_ */
