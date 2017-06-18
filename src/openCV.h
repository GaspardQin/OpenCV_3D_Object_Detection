#pragma once
#include "thread_variables.h"
#include "detectionMethodWithoutBuffer.h"
#include "CaptureCameraImage.h"
#include "calibrate.h"
DWORD WINAPI cvModelThreadFun(LPVOID lpParmeter);