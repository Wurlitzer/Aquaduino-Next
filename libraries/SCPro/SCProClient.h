#include <Client.h>
#include <HexConverter.h>
#include <BinMessageParser.h>
#include <PutchannelRequest.h>

class SCProClient {
public:
	SCProClient(Client& aClient);

	uint8_t init(char* server, uint16_t port, char* path, char* serial, char* key,
			char* SWVersion);
	uint8_t get(char* server, uint16_t port, char* path, int8_t myFunction);
	int16_t put(char* feed, uint8_t length);
	int16_t put(CPutchannelRequest* request);

	uint8_t buildSingleFeed(uint8_t channel, uint32_t value, char* buffer,
			uint8_t maxBuffer);

	//÷SCProClient();

protected:
	static const int kCalculateDataLength = 0;
	static const int kSendData = 1;
	void buildPath(char* aDest, unsigned long aFeedId, const char* aFormat);

	Client& _client;
	HexConverter* m_hexConverter;
	BinMessageParser* m_binMessageParser;

	char* m_SWVersion;
	char* m_Serial;
	char* m_connectionKey;
	char m_severURL[33];
	uint16_t m_severPORT;
	char m_severPath[33];
	char m_apikey[65];
	uint8_t m_apikeySet;
};

