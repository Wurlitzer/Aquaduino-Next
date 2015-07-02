/*
 * PutchannelRequestChannel.cpp
 *
 *  Created on: 19.06.2015
 *      Author: poppi
 */

#include "PutchannelRequestChannel.h"


CPutchannelRequestChannel::CPutchannelRequestChannel()
{
	chName=0;
	value=0;
	changeDateOffset=0;
}

CPutchannelRequestChannel::~CPutchannelRequestChannel()
{

}


uint32_t CPutchannelRequestChannel::toHex( char* hexBuf,const uint32_t hexbufLen,const uint32_t startPosStart,HexConverter* hexConverter,BinMessageParser* binMessageParser)
{
	uint32_t binStrLen=4;
	 char binStr[4];
	uint32_t startPos=startPosStart;
	binMessageParser->fromInt8ToBin(chName,binStr,binStrLen,0);
	startPos=hexConverter->encodeToHex(binStr,0,1,hexBuf,hexbufLen,startPos);

	binMessageParser->fromInt32ToBin(value,binStr,binStrLen,0);
	startPos=hexConverter->encodeToHex(binStr,0,4,hexBuf,hexbufLen,startPos);

	binMessageParser->fromInt16ToBin(changeDateOffset,binStr,binStrLen,0);
	startPos=hexConverter->encodeToHex(binStr,0,2,hexBuf,hexbufLen,startPos);

	return startPos;
}
uint32_t CPutchannelRequestChannel::fromHex( char* hexBuf,const uint32_t hexbufLen,const uint32_t startPos,HexConverter* hexConverter,BinMessageParser* binMessageParser)
{
}
uint32_t CPutchannelRequestChannel::getSize() {
	return 7;
}
