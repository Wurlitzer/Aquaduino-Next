/*
 * PutchannelRequest.cpp
 *
 *  Created on: 19.06.2015
 *      Author: poppi
 */

#include "PutchannelRequest.h"
#include "BinMessageParser.h"
#include "HexConverter.h"
#include <Time.h>

#include <Arduino.h>
extern int freeRam();

CPutchannelRequest::CPutchannelRequest() {
	timestamp = 0;
	sourceSize = 0;
	//Max 1 Source
	sources = (CPutChannelRequestSource**) malloc(1*sizeof(CPutChannelRequestSource*));
}

CPutchannelRequest::~CPutchannelRequest() {
	if (sourceSize > 0) {
		uint16_t x = 0;
		for (x = 0; x < sourceSize; x++) {
			//delete sources[x];
		}
	}
}

uint32_t CPutchannelRequest::toHex(char* hexBuf, const uint32_t hexbufLen,
		const uint32_t startPosStart, HexConverter* hexConverter,
		BinMessageParser* binMessageParser) {

	uint32_t binStrLen = 4;
	char binStr[4];
	uint32_t startPos = startPosStart;
	binMessageParser->fromInt32ToBin(timestamp - 1400000000, binStr, binStrLen,
			0);
	startPos = hexConverter->encodeToHex(binStr, 0, 4, hexBuf, hexbufLen,
			startPos);

	binMessageParser->fromInt16ToBin(sourceSize, binStr, binStrLen, 0);
	startPos = hexConverter->encodeToHex(binStr, 0, 2, hexBuf, hexbufLen,
			startPos);

	uint16_t x = 0;
	for (x = 0; x < sourceSize; x++) {
		startPos = sources[x]->toHex(hexBuf, hexbufLen, startPos, hexConverter,
				binMessageParser);
	}

	//return startPos;
}
uint32_t CPutchannelRequest::fromHex(char* hexBuf, const uint32_t hexbufLen,
		const uint32_t startPos, HexConverter* hexConverter,
		BinMessageParser* binMessageParser) {
}
void CPutchannelRequest::updateValue(int8_t sourceId, int8_t channel,
		int8_t type, int32_t value) {

	//Serial.print(F("updateValue Free Ram: "));
	//Serial.println(freeRam());

	//sourceSize = 1;
	timestamp = now() - 1400000000;

	CPutChannelRequestSource* putSource = 0;
	CPutchannelRequestChannel* putChannel = 0;
	for (int8_t x = 0; x < sourceSize; x++) {

		CPutChannelRequestSource* source = sources[x];
		if (source->sourceId == sourceId) {
			putSource = source;
			for (int8_t y = 0; y < source->channelSize; y++) {
				CPutchannelRequestChannel* chan = source->channels[y];
				if (chan->chName == channel) {

					putChannel = chan;

				}

			}
		}
	}
	if (putSource == 0) {
		putSource = new CPutChannelRequestSource();

		sources[sourceSize] = putSource;
		sourceSize++;
		putSource->sourceId = sourceId;
		putSource->channelSize = 0;
		putSource->errSize = 0;
		putSource->warnSize = 0;
		putSource->type = type;
	}
	if (putChannel == 0) {
		putChannel = new CPutchannelRequestChannel();

		putSource->channels[putSource->channelSize] = putChannel;
		putSource->channelSize++;
		putChannel->chName = channel;
		putChannel->changeDateOffset = 0;

	}

	putChannel->value = value;

}
uint32_t CPutchannelRequest::getSize() {
	uint32_t size = 6;
	for (uint32_t i = 0; i < sourceSize; i++) {
		CPutChannelRequestSource* c = sources[i];
		size += c->getSize();

	}
	return size;
}

