// FIND.h

#ifndef _FIND_h
#define _FIND_h

#include <string>
using namespace std;

class FINDClass
{
protected:
	string s;
	char i;

public:
	void init(string s);
	void init(){ i = 0; }
	bool find(const char ch);
};

extern FINDClass FIND;

#endif

