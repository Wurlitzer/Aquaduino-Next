/*
 * PutchannelRequest.h
 *
 *  Created on: 19.06.2015
 *      Author: poppi
 */

#ifndef PUTCHANNELREQUEST_H_
#define PUTCHANNELREQUEST_H_

#include "HexSerializeable.h"
#include "PutChannelRequestSource.h"
#include "stdint.h"

typedef uint32_t u_int32_t;
typedef uint16_t u_int16_t;



class CPutchannelRequest :public CHexSerializeable
{
public:
	uint32_t timestamp;
	uint16_t sourceSize;
	CPutChannelRequestSource** sources;

	CPutchannelRequest();
	virtual ~CPutchannelRequest();

	virtual uint32_t toHex(char* hexBuf,const uint32_t hexbufLen,const uint32_t startPos,HexConverter* hexConverter,BinMessageParser* binMessageParser);
	virtual uint32_t fromHex(char* hexBuf,const uint32_t hexbufLen,const uint32_t startPos,HexConverter* hexConverter,BinMessageParser* binMessageParser);

	void updateValue(int8_t sourceId,int8_t channel, int8_t type, int32_t value);
	//TODO
	// void addValue (int8_t sourceId,int8_t channel, int8_t type, int32_t value, int32_t timestamp);

	uint32_t getSize();
};


#endif /* PUTCHANNELREQUEST_H_ */
