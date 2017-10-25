#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <ctime>
#include <chrono>
#include "functions.h"
#include "camera_matrix.h"
#include "opencv2\core.hpp"
#include "opencv2\imgproc.hpp"
#include "opencv2\highgui.hpp"
#include "opencv2\imgcodecs.hpp"

#include "orbbec.h"

#include <mutex>

void commend_thread(Orbbec* orbbec, std::mutex* mtx);
void takePictureForCalibration(Orbbec* orbbec, std::mutex* mtx);
bool bTPC_flag;

int main(int argc, char** argv)
{
	IPC ipc("Orbbec.exe");
	RGBDcamera *rgbdData = ipc.connect<RGBDcamera>("Orbbec.exe");
	std::mutex mtx;

	Sleep(300);

	Orbbec* orbbec = new Orbbec(rgbdData, &ipc);
	try
	{
		orbbec->initialize("..\\data\\");
	}
	catch (const std::exception& ex)
	{
		std::cout << ex.what() << std::endl;
		//return 1;
	}
			
	int num_of_sensor = rgbdData->num_of_senseor;
	orbbec->enableRegistration(true);
	//orbbec->setRGBResolution(Orbbec::Resolution::SXGA);

	std::thread sensorThread(&Orbbec::threadRun, orbbec, &mtx);
	std::thread commendThread(commend_thread, orbbec, &mtx);	
	std::thread takePictureForCalibrationThread(takePictureForCalibration, orbbec, &mtx);

	sensorThread.join();
	commendThread.join();
	takePictureForCalibrationThread.join();

	orbbec->stop();
	delete orbbec;

	return 0;
}

void commend_thread(Orbbec* orbbec, std::mutex *m_mtx)
{
	bool tpThreadFlag = false;

	while (1)
	{
		std::cout << "->";
		std::string commend;
		std::cin >> commend;

		if (commend == "h" || commend == "help")
		{
			std::cout << "[img(i)]: toggle img on off [now: ";
			if (orbbec->drawImageEnabled()) std::cout << "enabled]" << std::endl;
			else std::cout << "disabled]" << std::endl;

			std::cout << "[rgb or ir]: toggle rgb mode or ir mode [now: "; 
			if (orbbec->IRenabled()) std::cout << "IR mode]" << std::endl; 
			else std::cout << "RGB mode]" << std::endl;

			std::cout << "[reg(r)]: toggle registraion mode [now: ";
			if (orbbec->regisrationEnabled()) std::cout << "enabled]" << std::endl; 
			else std::cout << "disabled]" << std::endl;

			std::cout << "[overlap(o)]: toggle draw rgb and depth overlap [now: ";
			if (orbbec->overlapEnabled())std::cout << "enabled]" << std::endl;
			else std::cout << "disabled]" << std::endl;

			std::cout << "[rgb resolution]: select resoltion rgb image [now: ";
			std::string res;
			switch (orbbec->getRGBResolution())
			{
			case Orbbec::Resolution::QVGA:
				res = "QVGA(320x240)";
				break;
			case Orbbec::Resolution::VGA:
				res = "VGA(640x480)";
				break;
			case Orbbec::Resolution::SXGA:
				res = "SXGA(1280x960)";
				break;
			default:
				res = "NONE";
				break;
			}
			std::cout << res << "]" << std::endl;
			std::cout << "r0: QVGA, r1: VGA, r2: SXGA" << std::endl;

			std::cout << "[TakePictureThread(t)]: toggle take picture [now: ";
			if (tpThreadFlag) std::cout << "enabled]" << std::endl;
			else std::cout << "disabled]" << std::endl;

		}

		if (commend == "img" || commend == "i")
		{
			orbbec->enableDrawImage(!orbbec->drawImageEnabled());
		}
		
		if (commend == "rgb" || commend == "ir")
		{
			m_mtx->lock();
			orbbec->enableIR(!orbbec->IRenabled());
			for (int i = 0; i < orbbec->getNumOfCameras(); i++)
				orbbec->openRGBorIR(i);
			m_mtx->unlock();
		}

		if (commend == "reg" || commend == "r")
		{
			orbbec->enableRegistration(!orbbec->regisrationEnabled());
		}

		if (commend == "overlap" || commend == "o")
		{
			orbbec->enableOverlap(!orbbec->overlapEnabled());
		}

		if (commend == "r0") {
			m_mtx->lock();
			orbbec->setRGBResolution(Orbbec::Resolution::QVGA);
			m_mtx->unlock();
		}
		if (commend == "r1") {
			m_mtx->lock();
			orbbec->setRGBResolution(Orbbec::Resolution::VGA);
			m_mtx->unlock();
		}
		if (commend == "r2") {
			m_mtx->lock();
			orbbec->setRGBResolution(Orbbec::Resolution::SXGA);
			m_mtx->unlock();
		}
		if (commend == "d0") {
			m_mtx->lock();
			orbbec->setDepthResolution(Orbbec::Resolution::QVGA);
			m_mtx->unlock();
		}
		if (commend == "d1") {
			m_mtx->lock();
			orbbec->setDepthResolution(Orbbec::Resolution::VGA);
			m_mtx->unlock();
		}
		
		
		if (commend == "t")
		{
			if (!bTPC_flag)
				bTPC_flag = true;
			else
				bTPC_flag = false;
		}

		if (commend == "q" || commend == "exit" || commend == "Exit")
		{
			orbbec->stop();
			break;
		}
	}
}

void takePictureForCalibration(Orbbec * orbbec, std::mutex * mtx)
{
	IPC ipc("takePictureForCalibration.exe");
	RGBDcamera *data2 = ipc.connect<RGBDcamera>("Orbbec.exe");

	cv::Mat canvas;

	char key = 0;
	double etime = 0.0;
	int waiting_time = 10;
	int frm_cnt = 0;
	std::string cam_pos = std::string(data2->camera_order[0]);
	std::string path = "../data/depth_calibration/";

	while(1)
	{
		if (key == 27)
			break;

		if (bTPC_flag)
		{			
			if (orbbec->IRenabled())
			{
				std::cout << "This mode allows RGB mode only, not IR mode." << std::endl;
				bTPC_flag = false;
			}
			cv::Mat img(data2->colorHeight, data2->colorWidth, CV_8UC3);
			cv::Mat depth(data2->depthHeight, data2->depthWidth, CV_16UC1);
			cv::Mat depth1_8U(data2->depthHeight, data2->depthWidth, CV_8UC1);

			int64 t1 = cv::getTickCount();

			memcpy(img.data, data2->colorData[0], sizeof(uchar)*data2->colorHeight*data2->colorWidth * 3);
			memcpy(depth.data, data2->depthData[0], sizeof(ushort)*data2->depthHeight*data2->depthWidth);

			depth.convertTo(depth1_8U, CV_8UC1, 0.025, -25);
			cv::cvtColor(depth1_8U, depth1_8U, CV_GRAY2BGR);

			//cv::hconcat(img, depth1_8U, canvas);
			cv::imshow("RGB", img);
			cv::imshow("Depth", depth1_8U);

			int64 t2 = cv::getTickCount();
			etime = (double)(t2 - t1) / cv::getTickFrequency()*1000.0;
			//std::cout << "Elapsed time: " << etime << std::endl;
			waiting_time = (int)std::round(180.0 - etime);
			waiting_time = waiting_time < 0 ? 1 : waiting_time;
			key = cv::waitKey(waiting_time);

			if (key == ' ')
			{
				char filename[256];
				sprintf(filename, "%s_rgb_%04d.png", cam_pos.c_str(), frm_cnt);
				cv::imwrite(path + filename, img);
				std::cout << filename << " written" << std::endl;
				sprintf(filename, "%s_depth_%04d.png", cam_pos.c_str(), frm_cnt);
				imwrite(path + filename, depth);
				std::cout << filename << " written" << std::endl;
				frm_cnt++;
			}
		}
		else
		{
			cv::destroyWindow("RGB");
			cv::destroyWindow("Depth");
			Sleep(1000);
		}
	}
}
