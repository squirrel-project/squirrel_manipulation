#include "KCLTCPDataClient.h"


void KCLTCPDataClient::error(const char *msg)
{
  //ROSINFO(msg);
  //exit(0);
}

KCLTCPDataClient::KCLTCPDataClient(const char * addr,int port)
{
  connected = false;
  configureHandAddress(addr, port);
  connectToHand();
}

KCLTCPDataClient::~KCLTCPDataClient()
{
  disconnectFromHand();
}

int KCLTCPDataClient::configureHandAddress(const char * new_addr,int new_port)
{
  portno = new_port;
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
    error("ERROR opening socket");
  else
  {
    server = gethostbyaddr(new_addr, 4, AF_INET);
    if (server == NULL) {
      error("ERROR, no such host\n");
    }
    else
    {
      memset((char *) &serv_addr, 0, sizeof(serv_addr));
      serv_addr.sin_family = AF_INET;
      memcpy((char *)&serv_addr.sin_addr.s_addr,
          (char *)server->h_addr,
          server->h_length);
      serv_addr.sin_port = htons(portno);
    }
  }
}

int KCLTCPDataClient::connectToHand()
{
  if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
  {
    error("ERROR connecting");
    connected = false;
  }
  else
    connected = true;
}

int KCLTCPDataClient::disconnectFromHand()
{
  close(sockfd);
  connected = false;
}

int KCLTCPDataClient::sendData()
{
  if (connected)
  {
    int n;

    char buffer[256];
    memset(buffer, 0, sizeof(buffer));

    sprintf(buffer,"test message");
    n = write(sockfd,buffer,strlen(buffer));
    if (n < 0)
      error("ERROR writing to socket");
    //ROSINFO("%s\n",buffer);
  }
  return 0;
}
