#pragma once  
#include "thread_variables.h"
#include "loadModel.h"
#include "openCV.h"
//DWORD WINAPI cvThreadFun(LPVOID lpParmeter);
//DWORD WINAPI glThreadFun(LPVOID lpParmeter);
HANDLE sentEvent;
HANDLE readEvent;//全局变量应在相应cpp下先声明，再在thread_variables.h中声明extern
int main() {
	HANDLE h_glThread = INVALID_HANDLE_VALUE;
	HANDLE h_cvTread = INVALID_HANDLE_VALUE;
	sentEvent = CreateEvent(NULL, false, false, (LPTSTR)"sendingEvent");
	readEvent = CreateEvent(NULL, false, false, (LPTSTR)"readingEvent");
	ResetEvent(sentEvent);
	ResetEvent(readEvent);
	SetEvent(readEvent);
	if ((!sentEvent) || (!readEvent))
	{
		std::cout << "Failed to CreateEvent !" << std::endl;
		return 0;
	}
	h_glThread = CreateThread(NULL, 0, glThreadFun, NULL, 0, NULL);
	h_cvTread = CreateThread(NULL, 0, cvThreadFun, NULL, 0, NULL);	
	Sleep(INFINITE);
	CloseHandle(h_cvTread);
	CloseHandle(h_glThread);
	system("PAUSE");
	return 0; 
}
