
#include "thread_variables.h"
DWORD WINAPI cvThreadFun(LPVOID lpParmeter) {
	while (1) {
		WaitForSingleObject(sentEvent,INFINITE);
		Sleep(20);
		std::cout << "openCV processing" << std::endl;
		SetEvent(readEvent);

	}
	return 0;
}