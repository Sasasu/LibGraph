#ifndef EXCEPTION_H
#define EXCEPTION_H

#include <iostream>
using namespace std;

class Exception {
	string msg;
	public:
		Exception() {}
		Exception(string m){
			msg = m;
		}
		string getMessage(){
			return msg;
		}
};

#endif