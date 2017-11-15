#pragma once
class GsmClass
{
public:
	int init();
	int loop();
private:
	int getsim(char * str);
	int send2sim(char *str, int len);
};
extern GsmClass GSM;
