#pragma once  
#include "thread_variables.h"
#include "loadModel.h"
#include "openCV.h"
//DWORD WINAPI cvThreadFun(LPVOID lpParmeter);
//DWORD WINAPI glThreadFun(LPVOID lpParmeter);
//HANDLE sentImgEvent;
HANDLE sentModelEvent;
HANDLE readModelEvent;//全局变量应在相应cpp下先声明，再在thread_variables.h中声明extern
//HANDLE readImgEvent;
int main() {
	HANDLE h_glThread = INVALID_HANDLE_VALUE;
	HANDLE h_cvModelThread = INVALID_HANDLE_VALUE;
	//HANDLE h_cvImgThread = INVALID_HANDLE_VALUE;
	//sentImgEvent = CreateEvent(NULL, false, false, (LPTSTR)"sendingImgEvent");
	sentModelEvent = CreateEvent(NULL, false, false, (LPTSTR)"sendingModelEvent");
	readModelEvent = CreateEvent(NULL, false, false, (LPTSTR)"readingModelEvent");
	//readImgEvent = CreateEvent(NULL, false, false, (LPTSTR)"readingImgEvent");
	ResetEvent(sentModelEvent);
	//ResetEvent(sentImgEvent);
	ResetEvent(readModelEvent);
	//ResetEvent(readImgEvent);
	if ((!sentModelEvent) || (!readModelEvent))
	{
		std::cout << "Failed to CreateEvent !" << std::endl;
		return 0;
	}
	h_glThread = CreateThread(NULL, 0, glThreadFun, NULL, 0, NULL);
	h_cvModelThread = CreateThread(NULL, 0, cvModelThreadFun, NULL, 0, NULL);	
	//h_cvImgThread = CreateThread(NULL, 0, cvImgThreadFun, NULL, 0, NULL);
	Sleep(INFINITE);
	//CloseHandle(h_cvImgThread);
	CloseHandle(h_cvModelThread);
	CloseHandle(h_glThread);
	system("PAUSE");
	return 0; 
}
