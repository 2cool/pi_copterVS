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

ofstream logfile;



int wifi_connections=0;
	int sockfd, newsockfd, portno;
     socklen_t clilen;
     uint8_t inbuffer[TELEMETRY_BUF_SIZE];
	 uint8_t outbuffer[TELEMETRY_BUF_SIZE];
     struct sockaddr_in serv_addr, cli_addr;
     int n;
     int connected=0;
     bool run=true;

void mclose(){
	logfile.close();
    wifi_connections--;
	close(newsockfd);
    close(sockfd);
	printf("FIFI closed\n");
}
CommanderClass *com;
TelemetryClass *tel;

bool wite_connection(){
	 newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
     if (newsockfd < 0) {
          printf("ERROR on accept\n");
          wifi_connections--;  
          return true;
	  }

	  
	  return false;
}

void server(){
	//delay(5000);
     if (wite_connection())
		return;
	  while(run){

		// bzero(buffer,256);
		  usleep(33000);
		 n = read(newsockfd,inbuffer, TELEMETRY_BUF_SIZE);
		// if (Autopilot.motors_is_on())
			// logfile.write((char*)inbuffer, n);
		if (n>0){
			if (connected == 0)
				printf("connected \n");
			connected++;
			com->new_data(inbuffer,n);
		//	printf("K");	
			int buf_len=tel->read_buf(outbuffer);
		//	printf("T\n");
			
			n = write(newsockfd,outbuffer,buf_len);
		//	if (Autopilot.motors_is_on())
			//	logfile.write((char*)outbuffer, buf_len);
		}else{
			if (connected) {
				
				printf("ERROR reading from socket\n");
				connected = 0;
			}
			if (wite_connection())
				return;
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

	logfile.open("/home/igor/log4444.log", fstream::in | fstream::out | fstream::trunc);







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
        printf("ERROR opening socket/n");
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

