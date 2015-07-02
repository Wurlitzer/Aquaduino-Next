/*
 uint32_t CPutchannelRequest::getSize() {
 } * PutchannelRequestSource.cpp
 *
 *  Created on: 19.06.2015
 *      Author: poppi
 */

#include "PutChannelRequestSource.h"
#include "stdlib.h"
//#include <iostream>

CPutChannelRequestSource::CPutChannelRequestSource() {
	sourceId = 0;
	type = 0;
	channelSize = 0;
	warnSize = 0;
	errSize = 0;
	channels = (CPutchannelRequestChannel**) malloc(10);
}

CPutChannelRequestSource::~CPutChannelRequestSource() {

}

uint32_t CPutChannelRequestSource::toHex(char* hexBuf, const uint32_t hexbufLen,
		const uint32_t startPosStart, HexConverter* hexConverter,
		BinMessageParser* binMessageParser) {
	uint32_t binStrLen = 4;
	char binStr[4];
	uint32_t startPos = startPosStart;
	binMessageParser->fromInt32ToBin(sourceId, binStr, binStrLen, 0);
	startPos = hexConverter->encodeToHex(binStr, 0, 4, hexBuf, hexbufLen,
			startPos);

	binMessageParser->fromInt8ToBin(type, binStr, binStrLen, 0);
	startPos = hexConverter->encodeToHex(binStr, 0, 1, hexBuf, hexbufLen,
			startPos);

	binMessageParser->fromInt16ToBin(channelSize, binStr, binStrLen, 0);
	startPos = hexConverter->encodeToHex(binStr, 0, 2, hexBuf, hexbufLen,
			startPos);

	uint16_t x = 0;
	for (x = 0; x < channelSize; x++) {
		startPos = channels[x]->toHex(hexBuf, hexbufLen, startPos, hexConverter,
				binMessageParser);
	}

	binMessageParser->fromInt16ToBin(warnSize, binStr, binStrLen, 0);
	startPos = hexConverter->encodeToHex(binStr, 0, 2, hexBuf, hexbufLen,
			startPos);

	binMessageParser->fromInt16ToBin(errSize, binStr, binStrLen, 0);
	startPos = hexConverter->encodeToHex(binStr, 0, 2, hexBuf, hexbufLen,
			startPos);
	//std::cout << startPos << std::endl;

	return startPos;
}

uint32_t CPutChannelRequestSource::fromHex(char* hexBuf,
		const uint32_t hexbufLen, const uint32_t startPos,
		HexConverter* hexConverter, BinMessageParser* binMessageParser) {
	return 0;
}
uint32_t CPutChannelRequestSource::getSize() {
	uint32_t size = 11;
	for (uint32_t i = 0; i < channelSize; i++) {
		CPutchannelRequestChannel* c = channels[i];
		size += c->getSize();

	}
	return size;
}
