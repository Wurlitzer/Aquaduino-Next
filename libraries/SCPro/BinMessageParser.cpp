/*
 * BinMessageParser.cpp
 *
 *  Created on: 14.01.2015
 *      Author: poppi
 */

#include "BinMessageParser.h"
#include "string.h"

unsigned int BinMessageParser::fromInt8ToBin(const char value, char* binStr, const unsigned int bufLen, const unsigned int startPos) {
	unsigned int currentPos = startPos;

	binStr[currentPos] = value;
	currentPos++;

	return currentPos;
}
unsigned int BinMessageParser::fromInt16ToBin(const unsigned short value, char* binStr, const unsigned int bufLen, const unsigned int startPos) {
	unsigned int currentPos = startPos;
	binStr[currentPos] = (value >> 8) & 0xFF;
	currentPos++;
	binStr[currentPos] = value & 0xFF;
	currentPos++;

	return currentPos;
}
unsigned int BinMessageParser::fromInt32ToBin(const unsigned long value, char* binStr, const unsigned int bufLen, const unsigned int startPos) {
	unsigned int currentPos = startPos;
	binStr[currentPos] = (value >> 24) & 0xFF;
	currentPos++;
	binStr[currentPos] = (value >> 16) & 0xFF;
	currentPos++;
	binStr[currentPos] = (value >> 8) & 0xFF;
	currentPos++;
	binStr[currentPos] = value & 0xFF;
	currentPos++;

	return currentPos;
}

unsigned int BinMessageParser::fromString8ToBin(const char* value, char* binStr, const unsigned int bufLen, const unsigned int startPos) {
	unsigned int currentPos = startPos;

	unsigned int len = strlen(value);
	currentPos = fromInt8ToBin(len, binStr, bufLen, currentPos);

	for (unsigned int x = 0; x < len; x++) {
		binStr[currentPos + x] = value[x];
	}

	return currentPos + len + 1;

}
unsigned int BinMessageParser::fromString32ToBin(const char* value, char* binStr, const unsigned int bufLen, const unsigned int startPos) {
	unsigned int currentPos = startPos;

	unsigned int len = strlen(value);
	currentPos = fromInt16ToBin(len, binStr, bufLen, currentPos);

	for (unsigned int x = 0; x < len; x++) {
		binStr[currentPos + x] = value[x];
	}

	return currentPos + len + 4;
}

uint8_t BinMessageParser::fromBinToInt8(const char* binStr, const unsigned int bufLen, const unsigned int startPos) {
	unsigned int currentPos = startPos;

	return binStr[currentPos];
}
uint16_t BinMessageParser::fromBinToInt16(const char* binStr, const unsigned int bufLen, const unsigned int startPos) {
	uint8_t currentPos = startPos;

	uint16_t val = 0;

	val = binStr[currentPos];
	val = val << 8;
	//val=binStr[currentPos]<<8;
	if (binStr[currentPos + 1] < 0) {
		val |= (binStr[currentPos + 1] + 256);

	} else {
		val |= binStr[currentPos + 1];
	}
	return val;

}
uint32_t BinMessageParser::fromBinToInt32(const char* binStr, const unsigned int bufLen, const unsigned int startPos) {
	unsigned int currentPos = startPos;

	uint32_t val = 0;

	val = binStr[currentPos] << 24;
	val |= binStr[currentPos + 1] << 16;
	val |= binStr[currentPos + 2] << 8;
	val |= binStr[currentPos + 3];
	return val;

}

unsigned int BinMessageParser::fromBinToString8(char* value, const unsigned int maxStrlen, const char* binStr, const unsigned int bufLen, const unsigned int startPos) {
	uint8_t currentPos = startPos;
	uint8_t len = fromBinToInt8(binStr, bufLen, currentPos);
	currentPos++;

	for (unsigned int x = 0; x < len; x++) {
		value[x] = binStr[currentPos + x];
	}
	value[len] = 0;	// 0 terminated
	return currentPos + len;
}
unsigned int BinMessageParser::fromBinToString32(char* value, const unsigned int maxStrlen, const char* binStr, const unsigned int bufLen, const unsigned int startPos) {
	unsigned int currentPos = startPos;
	unsigned int len = fromBinToInt16(binStr, bufLen, currentPos);
	currentPos += 2;

	for (unsigned int x = 0; x < len; x++) {
		value[x] = binStr[currentPos + x];
	}
	value[len] = 0;	// 0 terminated
	return currentPos + len;
}

