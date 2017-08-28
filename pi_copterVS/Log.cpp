#include "Log.h"
#include "debug.h"

volatile  bool	run_loging = true;
volatile int log_index, log_bank_, log_bank, old_bank, error_bansk = 0;
volatile bool log_file_closed;
#define mask 255
uint8_t log_buffer[mask+1][1024];


ofstream logfile;
string this_log_fname;



void loger() {
	while (run_loging) {
		while (run_loging && log_bank_ == old_bank) {
			usleep(2000);
		}
		
		//printf("%i\n",log_bank_ - old_bank);
		bool print = true;
		while (log_bank_ - old_bank > mask) {
			if (print)
				fprintf(Debug.out_stream, "log sync error! %i\n",log_bank_);
			print = false;
			old_bank++;
			error_bansk++;
		}
		if (logfile.is_open()) {
			int len = *((uint16_t*)log_buffer[old_bank & mask]);
			logfile.write((char*)log_buffer[old_bank & mask], len);
			//logfile.flush();
			old_bank++;
		}else{
			fprintf(Debug.out_stream, "LOG ERROR\n");
		}
			
		
	}
	logfile.close();

	

	std::ifstream in(this_log_fname, std::ifstream::ate | std::ifstream::binary);
	int filesize = in.tellg();
	//printf("file size %i\n", filesize);
	if (filesize == 0) {
		remove(this_log_fname.c_str());
	}
	log_file_closed = true;


}
bool LogClass::close() {
	fprintf(Debug.out_stream, "close tel log\n");
	fprintf(Debug.out_stream, "banks: %i\twrited banks: %i\terrors:%i\n", log_bank_, old_bank, error_bansk);
	run_loging = false;
	while (log_file_closed == false)
		usleep(100000);
	return true;



}

bool LogClass::init(int counter_) {
	counter = counter_;
	run_counter = 0;
	if (writeTelemetry) {
		run_loging = true;
		ostringstream convert;
		convert << "/home/igor/logs/tel_log" << counter << ".log";
		this_log_fname = convert.str();
		fprintf(Debug.out_stream, "log 2 %s\n", this_log_fname.c_str());
		logfile.open(this_log_fname.c_str(), fstream::in | fstream::out | fstream::trunc);
		log_file_closed = false;
		log_bank_ = old_bank = log_bank = 0;
		log_index = 2;
		thread t(loger);
		t.detach();
	}
	return logfile.is_open();
}
void LogClass::loadFloat(float f) {
	uint8_t *fp = (uint8_t*)&f;
	log_buffer[log_bank][log_index++] = fp[0];
	log_buffer[log_bank][log_index++] = fp[1];
	log_buffer[log_bank][log_index++] = fp[2];
	log_buffer[log_bank][log_index++] = fp[3];
}
void LogClass::loaduint32t(uint32_t ui) {
	uint8_t *fp = (uint8_t*)&ui;
	log_buffer[log_bank][log_index++] = fp[0];
	log_buffer[log_bank][log_index++] = fp[1];
	log_buffer[log_bank][log_index++] = fp[2];
	log_buffer[log_bank][log_index++] = fp[3];
}
void LogClass::loadInt16t(int16_t i) {
	uint8_t *ip = (uint8_t*)&i;
	log_buffer[log_bank][log_index++] = ip[0];
	log_buffer[log_bank][log_index++] = ip[1];
}


void LogClass::loadGPS_full(NAV_POSLLH *gps) {
	memcpy((uint8_t*)&log_buffer[log_bank][log_index], gps, sizeof(NAV_POSLLH));
	log_index += sizeof(NAV_POSLLH);
}


void LogClass::loadGPS(long lat, long lon) {
	uint8_t *fp = (uint8_t*)&lat;
	log_buffer[log_bank][log_index++] = fp[0];
	log_buffer[log_bank][log_index++] = fp[1];
	log_buffer[log_bank][log_index++] = fp[2];
	log_buffer[log_bank][log_index++] = fp[3];
	fp = (uint8_t*)&lon;
	log_buffer[log_bank][log_index++] = fp[0];
	log_buffer[log_bank][log_index++] = fp[1];
	log_buffer[log_bank][log_index++] = fp[2];
	log_buffer[log_bank][log_index++] = fp[3];
	log_buffer[log_bank][log_index++] = 1;
	log_buffer[log_bank][log_index++] = 1;
}
void LogClass::loadGPS(NAV_POSLLH *gps) {
	uint8_t *fp = (uint8_t*)&gps->lat;
	log_buffer[log_bank][log_index++] = fp[0];
	log_buffer[log_bank][log_index++] = fp[1];
	log_buffer[log_bank][log_index++] = fp[2];
	log_buffer[log_bank][log_index++] = fp[3];
	fp = (uint8_t*)&gps->lon;
	log_buffer[log_bank][log_index++] = fp[0];
	log_buffer[log_bank][log_index++] = fp[1];
	log_buffer[log_bank][log_index++] = fp[2];
	log_buffer[log_bank][log_index++] = fp[3];
	log_buffer[log_bank][log_index++] = gps->hAcc;
	log_buffer[log_bank][log_index++] = gps->vAcc;
}

void LogClass::loadMem(uint8_t*src, int len) {
	uint8_t *fp = (uint8_t*)&len;
	log_buffer[log_bank][log_index++] = fp[0];
	log_buffer[log_bank][log_index++] = fp[1];
	memcpy(&(log_buffer[log_bank][log_index]), src, len);
	log_index += len;
}
void LogClass::loadByte(uint8_t b)
{
	log_buffer[log_bank][log_index++] = b;
}

int max_log_index = 0;
void LogClass::end() {
	//if (log_index > max_log_index)
	//{
	//	max_log_index = log_index;
	//	printf("max log index=%i\n", log_index);
	//}
	if (log_index > 2) {
		uint8_t *fp = (uint8_t*)&log_index;
		log_buffer[log_bank][0] = fp[0];
		log_buffer[log_bank][1] = fp[1];
		log_bank_++;
		log_bank = log_bank_ & mask;

		log_index = 2;
	}
}

LogClass Log;