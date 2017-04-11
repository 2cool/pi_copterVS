#include <thread>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>

#include "define.h"
#include "Wi_Fi.h"
#include "Telemetry.h"
#include "commander.h"


//---------------------------------------------
void error(const char *msg)
{
    perror(msg);
    exit(1);
}

int wifi_connections=0;
	int sockfd, newsockfd, portno;
     socklen_t clilen;
     uint8_t buffer[TELEMETRY_BUF_SIZE];
     struct sockaddr_in serv_addr, cli_addr;
     int n;
     int connected=0;
     bool run=true;

void mclose(){
	close(newsockfd);
    close(sockfd);
    wifi_connections--;
	
}
CommanderClass *com;
TelemetryClass *tel;

bool wite_connection(){
	connected = 0;
	 newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
     if (newsockfd < 0) {
          printf("ERROR on accept\n");
          wifi_connections--;     
          return true;
	  }

	  
	  return false;
}

void server(){
	delay(5000);
     if (wite_connection())
		return;
	  printf("connected \n");
	  while(run){
		// bzero(buffer,256);
		 n = read(newsockfd,buffer, TELEMETRY_BUF_SIZE);
		 connected++;
		// printf("recived: %i byte\n",n); 
		// if (buffer[0]!='M')
		//	printf("%s\n",buffer);
		// if (n < 0) {
		//	printf("ERROR reading from socket\n");
		// printf("Here is the message: %s\n",buffer);
		//	return;
		//}
		if (n>0){
		//	printf("<- ");
			com->new_data(buffer,n);
			
			int buf_len=tel->read_buf(buffer);
		//	printf("-> ");
			
			n = write(newsockfd,buffer,buf_len);
		}else{
			printf("ERROR reading from socket\n");
			if (wite_connection())
				return;
			printf("connected \n");
		}
	}	
	
	mclose();
}

	
	
	


bool WiFiClass::connectedF() { return connected > 0; }

WiFiClass::~WiFiClass(){
	run = false;
	printf("WIFICLass distructor\n");
	//delay(1000);
	//mclose();
}




int WiFiClass::init()
{
#ifdef WORK_WITH_WIFI
	com=&Commander;
	tel=&Telemetry;
	blinkState = 0;


	

	commander_done = true;
	command_resived = false;
	
	if (wifi_connections>0)
		return 0;
	wifi_connections++;

	connected=0;
     sockfd = socket(AF_INET, SOCK_STREAM, 0);
     if (sockfd < 0) {
        error("ERROR opening socket/n");
        wifi_connections--;
        return -1;
	}
     bzero((char *) &serv_addr, sizeof(serv_addr));
    portno = 9876;
     serv_addr.sin_family = AF_INET;
     serv_addr.sin_addr.s_addr = INADDR_ANY;
     serv_addr.sin_port = htons(portno);
     if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
          printf("ERROR on binding/n");
          wifi_connections--;
          return -1;
	}
    listen(sockfd,5);
     clilen = sizeof(cli_addr);
     
     thread t(server);
     t.detach();
     //server();
	  printf("server started...\n");
	  return 0;
	
	
#endif
}

WiFiClass WiFi;

