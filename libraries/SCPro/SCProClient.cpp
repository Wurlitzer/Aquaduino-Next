#include <SCPro.h>
#include <HttpClient.h>
#include <Time.h>
#include <CountingStream.h>

extern int freeRam();
enum {
	SERVER_MANAGER = 0, API_KEY = 1
};

SCProClient::SCProClient(Client& aClient) :
		_client(aClient) {
	m_hexConverter = new HexConverter();
	m_binMessageParser = new BinMessageParser();
}
int SCProClient::init(char* server, uint16_t port, char* path, char* serial,
		char* key, char* SWVersion) {

	/*if (m_severURL != 0) {
	 free(m_severURL);
	 }
	 m_severURL = 0;*/
	m_SWVersion = (char*) malloc(16);
	strncpy(m_SWVersion, SWVersion, 16);

	m_Serial = serial;
	m_connectionKey = key;

	get(server, port, path, SERVER_MANAGER);
	Serial.print("..");
	get(m_severURL, m_severPORT, m_severPath, API_KEY);
	//Serial.print(m_apikey);
	Serial.print(F("Free Ram: "));
	Serial.println(freeRam());

}

uint8_t SCProClient::get(char* server, uint16_t port, char* path,
		int8_t myFunction) {

	Serial.print(F("SCProClient::get Free Ram: "));
	Serial.println(freeRam());
	char functionStr[16];

	HttpClient http(_client);

	http.beginRequest();

	switch (myFunction) {
	case 0:
		strncpy(functionStr, "server-manager/", 16);
		break;
	case 1:
		strncpy(functionStr, "apikey/", 16);

		break;
	}

	uint8_t counter = 0;
	uint8_t contentLength;

	char pathFunction[128];
	strncpy(pathFunction, path, 128);
	strncat(pathFunction, functionStr, 128);
	strncat(pathFunction, m_Serial, 128);

	int ret = http.get(server, port, pathFunction);
	if (myFunction == 1) {
		http.sendHeader("X-Key", m_connectionKey);
	}	//pathFunction[0] = (char) 0;
	Serial.println(pathFunction);
	char* httpResult = NULL;

	if (ret == 0) {
		CountingStream countingStream; // Used to work out how long that data will be

		http.sendHeader("Content-Length", 0);
		// Now we're done sending the request

		http.endRequest();

		ret = http.responseStatusCode();
		if ((ret < 200) || (ret > 299)) {
			// It wasn't a successful response, ensure it's -ve so the error is easy to spot
			if (ret > 0) {
				ret = ret * -1;
			}
		} else {
			http.skipResponseHeaders();

			contentLength = http.contentLength();

			httpResult = (char*) malloc(contentLength);

			char next;
			while ((http.available() || http.connected())
					&& counter < contentLength) {
				if (http.available()) {

					next = http.read();

					httpResult[counter] = next;
					counter++;
				}
			}
			httpResult[counter] = 0;

			http.flush();
			//Serial.print("RAW:");
			//Serial.println(httpResult);

		}
		http.stop();

	}

	uint8_t binStrLen;
	uint8_t httpResultLength = strlen(httpResult);
	binStrLen = m_hexConverter->estimatedBinBufLen(httpResultLength);
	//Serial.println(binStrLen);
	//Serial.println(".");
	m_hexConverter->decodeFromHex(httpResult, httpResultLength, httpResult, 0,
			binStrLen);

	switch (myFunction) {
	case 0:
		//URL
		uint8_t nextPos;
		nextPos = m_binMessageParser->fromBinToString8(m_severURL, 32,
				httpResult, binStrLen, 0);
		Serial.print(m_severURL);
		//Port

		m_severPORT = m_binMessageParser->fromBinToInt16(httpResult, binStrLen,
				nextPos);
		nextPos += 2;
		Serial.print(m_severPORT);
		//Path

		m_binMessageParser->fromBinToString8(m_severPath, 32, httpResult,
				binStrLen, nextPos);
		Serial.println(m_severPath);
		break;
	case 1:

		m_binMessageParser->fromBinToString8(m_apikey, 32, httpResult,
				binStrLen, 0);
		Serial.print("apikey: ");
		Serial.println(m_apikey);
		break;
	}
	free(httpResult);
	return contentLength;
}

int16_t SCProClient::put(CPutchannelRequest* request) {
	Serial.print(F("SCProClient::put Free Ram: "));
	Serial.println(freeRam());

	uint8_t len = m_hexConverter->estimatedHexBufLen(request->getSize());
	char* buffer = (char*) malloc(len);

	len = request->toHex(buffer, len, 0, m_hexConverter, m_binMessageParser);

	int16_t result = put(buffer, len);
	free(buffer);


	return result;
}
int16_t SCProClient::put(char* feed, uint8_t length) {


	HttpClient http(_client);

	http.beginRequest();

	char pathFunction[254];
	strncpy(pathFunction, m_severPath, 254);
	strncat(pathFunction, "channels/", 254);
	strncat(pathFunction, m_Serial, 254);

	int ret = http.post(m_severURL, m_severPORT, pathFunction);

	if (ret == 0) {
		http.sendHeader("X-ApiKey", m_apikey);
		http.sendHeader("X-SW", m_SWVersion);
		http.sendHeader("Content-Type", "text/plain");
		http.sendHeader("Content-Length", length);

		http.write((unsigned char*) feed, length);

		// Now we're done sending the request
		http.endRequest();

		ret = http.responseStatusCode();
		if ((ret < 200) || (ret > 299)) {
			// It wasn't a successful response, ensure it's -ve so the error is easy to spot
			if (ret > 0) {
				ret = ret * -1;
			}
		}
		http.flush();
		http.stop();

	}
	//free(sendFeed);

	return ret;
}
/*
 SCProClient::÷SCProClient() {
 free(m_SWVersion);

 }*/
