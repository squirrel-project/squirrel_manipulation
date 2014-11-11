#ifndef KCLHANDTCPDATACLIENT_H
#define KCLHANDTCPDATACLIENT_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include "ros/ros.h"

class KCLTCPDataClient
{
  private:
    int sockfd;
    int portno;
    bool connected;

    struct sockaddr_in serv_addr;
    struct hostent *server;

    void error(const char *msg);

  public:
    KCLTCPDataClient(const char * address,int port);
    ~KCLTCPDataClient();

    int configureHandAddress(const char * new_address,int new_port);
    int connectToHand();
    int disconnectFromHand();
    int sendData();
};

#endif
