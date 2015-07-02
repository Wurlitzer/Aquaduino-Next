/*
 * PutChannelRequestSource.h
 *
 *  Created on: 19.06.2015
 *      Author: poppi
 */

#ifndef PUTCHANNELREQUESTSOURCE_H_
#define PUTCHANNELREQUESTSOURCE_H_

#include "HexSerializeable.h"
#include "PutchannelRequestChannel.h"
#include "stdint.h"

typedef uint32_t u_int32_t;
typedef uint16_t u_int16_t;
typedef uint8_t u_int8_t;

class CPutChannelRequestSource:public CHexSerializeable
{
public:
	u_int32_t sourceId;
	u_int8_t type;
	u_int16_t channelSize;
	CPutchannelRequestChannel** channels;
	u_int16_t warnSize;
	// childs of warings
	u_int16_t errSize;
	// childs of Errors

	CPutChannelRequestSource();
	virtual ~CPutChannelRequestSource();

	virtual uint32_t toHex( char* hexBuf,const uint32_t hexbufLen,const uint32_t startPos,HexConverter* hexConverter,BinMessageParser* binMessageParser);
	virtual uint32_t fromHex( char* hexBuf,const uint32_t hexbufLen,const uint32_t startPos,HexConverter* hexConverter,BinMessageParser* binMessageParser);

	uint32_t getSize();
};


#endif /* PUTCHANNELREQUESTSOURCE_H_ */
