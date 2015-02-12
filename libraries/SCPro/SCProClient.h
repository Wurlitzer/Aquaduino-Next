#include <Client.h>

class SCProClient
{
public:
  SCProClient(Client& aClient);

  int init(char* server,uint16_t port,char* path,char* function,char* serial);
  //int get(XivelyFeed& aFeed, const char* aApiKey);
  // int put(XivelyFeed& aFeed, const char* aApiKey);

protected:
  static const int kCalculateDataLength =0;
  static const int kSendData =1;
  void buildPath(char* aDest, unsigned long aFeedId, const char* aFormat);

  Client& _client;
};


