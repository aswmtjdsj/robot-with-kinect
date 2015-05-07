#ifndef COMMON_H
#define COMMON_H


#define ASIO_STANDALONE
#define _WIN32_WINNT 0x0602
// #include <Windows.h>
#include "asio-1.10.6\include\asio.hpp"
#include <cstdlib>
#include <deque>
#include <iostream>
#include <thread>
#include <iostream>
#include <locale>
#include <codecvt>

template<class Interface>
inline void SafeRelease( Interface *& pInterfaceToRelease )
{
	if( pInterfaceToRelease != NULL ){
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

#endif
