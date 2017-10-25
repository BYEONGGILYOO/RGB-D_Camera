#define _CRT_SECURE_NO_WARNINGS

#include <iostream>
#include <stdlib.h>
#include <thread>
#include <mutex>
#include <conio.h>
#include <windows.h>

#include <opencv2\opencv.hpp>

#include "functions.h"
#include "real_sense.h"

int main(void)
{
	IPC ipc("RealSense.exe");
	RGBDcamera *rgbdData = ipc.connect<RGBDcamera>("RealSense.exe");
	//ipc.start("RealSense.exe");

	RealSense realsense(rgbdData, &ipc);

	std::thread realsenseThread(&RealSense::threadRun, &realsense);

	realsenseThread.join();

	return 0;
}