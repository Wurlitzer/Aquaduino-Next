/*
 * HexConverter.cpp
 *
 *  Created on: 10.01.2015
 *      Author: poppi
 */

#include "HexConverter.h"
//#include <iostream>

unsigned int HexConverter::strlen(const char* value) {
	unsigned int len = 0;
	char ch = value[0];
	while (ch != 0 && len < 2000000000) {
		len++;
		ch = value[len];
	}
	return len;
}

int HexConverter::decodeFromHex(const char* hexStr, const unsigned int hexStrLen, char* binStr, const unsigned int startPos, const unsigned int binStrSize) {
	unsigned int neededSize = estimatedBinBufLen(hexStrLen) + startPos;
	if (neededSize > binStrSize)
		return 0;

	unsigned int binPos = startPos;
	for (unsigned int x = 0; x < hexStrLen; x += 2) {
		char high = hexStr[x];
		char low = hexStr[x + 1];

		char value = 0;
		if (high >= '0' && high <= '9') {
			value = (high - '0') << 4;
		}
		if (high >= 'A' && high <= 'F') {
			value = (high - 'A' + 10) << 4;
		}
		if (low >= '0' && low <= '9') {
			value |= (low - '0');
		}
		if (low >= 'A' && low <= 'F') {
			value |= (low - 'A' + 10);
		}
		binStr[binPos] = value;
		binPos++;

	}
	return binPos;
}

int HexConverter::encodeToHex(const char* binStr, const unsigned int startPos, const unsigned int endPos, char* hexStr, const unsigned int hexStrLength, uint32_t hexPosStart) {
	if (startPos >= endPos)
		return 0;
	if ((endPos - startPos) > hexStrLength)
		return 0;

	unsigned int hexPos = hexPosStart;
	for (unsigned int x = startPos; x < endPos; x++) {
		char ch = binStr[x];
		// encode high nibble
		char high = (ch >> 4) & 15;
		if (high >= 0 && high <= 9) {
			hexStr[hexPos] = '0' + high;
		}
		if (high >= 10 && high <= 15) {
			hexStr[hexPos] = 'A' + high - 10;
		}
		char low = ch & 15;
		if (low >= 0 && low <= 9) {
			hexStr[hexPos + 1] = '0' + low;
		}
		if (low >= 10 && low <= 15) {
			hexStr[hexPos + 1] = 'A' + low - 10;
		}
		//std::cout << hexStr[hexPos] << hexStr[hexPos+1] << std::endl;
		hexPos += 2;
	}

	return hexPos;
}
int HexConverter::estimatedHexBufLen(unsigned int binStrlen) {
	return binStrlen * 2;
}
int HexConverter::estimatedBinBufLen(int hexStrlen) {
	return hexStrlen / 2 + hexStrlen % 2;
}

