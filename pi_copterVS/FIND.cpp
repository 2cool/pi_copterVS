// 
// 
// 

#include "FIND.h"



bool FINDClass::find(const char ch){

	if (s[i] == ch){
		i++;
		if (i == s.length()){
			i = 0;
			return true;
		}
	}
	else
		i = 0;

	return false;
}

void FINDClass::init(string st)
{
	s = st;
	i = 0;

}


FINDClass FIND;

