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
uint8_t inbuffer[TELEMETRY_BUF_SIZE];
uint8_t outbuffer[TELEMETRY_BUF_SIZE];
struct sockaddr_in serv_addr, cli_addr;
int n;
int connected=0;
#define uchar unsigned char


string WiFiClass::get_client_addres() {
	int adr = cli_addr.sin_addr.s_addr;
	string s = to_string((uchar)(adr & 255)) + "." + to_string((uchar)(255 & (adr >> 8))) + "." + to_string((uchar)(255 & (adr >> 16))) + "." + to_string((uchar)(adr >> 24));
	return s;



}

string log_fname;
	 
void mclose(){

	fprintf(Debug.out_stream, "server stoped\n");
    wifi_connections--;
	close(newsockfd);
    close(sockfd);
	fprintf(Debug.out_stream,"WIFI closed\n");
}
CommanderClass *com;
TelemetryClass *tel;

bool wite_connection(){
	 newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
     if (newsockfd < 0) {
          fprintf(Debug.out_stream,"ERROR on accept\n");
          wifi_connections--;  
          return true;
	  }

	  
	  return false;
}

bool run = true;
bool WiFiClass::stopServer() { 
	run = false; 
	mclose();

}
uint32_t wifiold_t = 0;
void server(){
	//delay(5000);
	
     if (wite_connection())
		return;
	  while(run){

		// bzero(buffer,256);
		  const uint32_t t = millis();
		  const uint32_t dt = t - wifiold_t;
		  wifiold_t = t;
		  if (dt < 33)
			  usleep(33 - dt);
		  if (run)
			n = read(newsockfd,inbuffer, TELEMETRY_BUF_SIZE);



		
		if (n>0){
			if (connected == 0)
				fprintf(Debug.out_stream,"connected \n");
			connected++;
			com->new_data(inbuffer,n);
			
		//	fprintf(Debug.out_stream,"K");	
			int buf_len=tel->read_buf(outbuffer);
		//	fprintf(Debug.out_stream,"T\n");

			

			if (true)
				n = write(newsockfd,outbuffer,buf_len);


			
		}else{
			if (connected) {
				
				fprintf(Debug.out_stream,"ERROR reading from socket\n");
				connected = 0;
			}
			if (wite_connection())
				return;
		}
	}	
	
	
}

	
	
	


bool WiFiClass::connectedF() { return connected > 0; }






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
        fprintf(Debug.out_stream,"ERROR opening socket/n");
        wifi_connections--;
        return -1;
	}
     bzero((char *) &serv_addr, sizeof(serv_addr));
    portno = 9876;
     serv_addr.sin_family = AF_INET;
     serv_addr.sin_addr.s_addr = INADDR_ANY;
     serv_addr.sin_port = htons(portno);
     if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
          fprintf(Debug.out_stream,"ERROR on binding/n");
          wifi_connections--;
          return -1;
	}
    listen(sockfd,5);
     clilen = sizeof(cli_addr);
     
     thread t(server);
     t.detach();
     //server();
	  fprintf(Debug.out_stream,"server started...\n");
	  return 0;
	
	
#endif
}

WiFiClass WiFi;

