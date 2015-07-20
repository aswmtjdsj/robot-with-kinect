#ifndef COMMON_H
#define COMMON_H


#define ASIO_STANDALONE
#define _WIN32_WINNT 0x0602
// #include <Windows.h>
#include "asio-1.10.6\include\asio.hpp"

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cstdio>
#include <iomanip>
#include <sstream>

#include <deque>
#include <vector>
#include <queue>
#include <algorithm>

#include <thread>

#include <locale>
#include <codecvt>
#include <comdef.h>

template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease )
{
	if( pInterfaceToRelease != nullptr ){
		pInterfaceToRelease->Release();
		pInterfaceToRelease = nullptr;
	}
}

inline std::string wConvToS(const std::wstring & ws) {
	std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
	std::string s = converter.to_bytes(ws);
	return s;
}

inline std::wstring sConvToW(const std::string & s) {
	std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
	std::wstring ws = converter.from_bytes(s);
	return ws;
}

#endif
