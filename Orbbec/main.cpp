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
	ipc.start("Orbbec.exe");

	Orbbec* orbbec = new Orbbec(rgbdData, &ipc);
	orbbec->initialize("..\\data\\");
		
	int num_of_sensor = rgbdData->num_of_senseor;
	orbbec->enableRegistration(true);
	//orbbec->setRGBResolution(Orbbec::Resolution::SXGA);

	std::thread sensorThread(&Orbbec::threadRun, orbbec, &mtx);
	std::thread commendThread(commend_thread, orbbec, &mtx);	
	std::thread takePictureForCalibrationThread(takePictureForCalibration, orbbec, &mtx);
	if (0)
	{
		std::thread registration([&]()
		{
			IPC ipc("takePictureForCalibration.exe");
			RGBDcamera *data2 = ipc.connect<RGBDcamera>("Orbbec.exe");

			int num_of_sensor = data2->num_of_senseor;
			cv::Mat* img = new cv::Mat[num_of_sensor];
			cv::Mat* depth = new cv::Mat[num_of_sensor];

			cv::Mat* newDepth = new cv::Mat[num_of_sensor];
			cv::Mat* depth2color = new cv::Mat[num_of_sensor];

			// init
			RGBD_Parameters* rgbd_param = new RGBD_Parameters[num_of_sensor];
			cv::Mat* rt = new cv::Mat[num_of_sensor];

			for (int i = 0; i < num_of_sensor; i++)
			{
				img[i] = cv::Mat(data2->colorHeight, data2->colorWidth, CV_8UC3);
				depth[i] = cv::Mat(data2->depthHeight, data2->depthWidth, CV_16UC1);
				newDepth[i] = cv::Mat(data2->colorHeight, data2->colorWidth, CV_16UC1, cv::Scalar(0));
				depth2color[i] = cv::Mat(data2->colorHeight, data2->colorWidth, CV_8UC3);

				rgbd_param[i].color_intrinsic.ppx = data2->colorK[i][0]; rgbd_param[i].color_intrinsic.ppy = data2->colorK[i][1];
				rgbd_param[i].color_intrinsic.fx = data2->colorK[i][2]; rgbd_param[i].color_intrinsic.fy = data2->colorK[i][3];
				memcpy(rgbd_param[i].color_intrinsic.coeffs, data2->colorCoeffs[i], sizeof(float) * 5);

				rgbd_param[i].depth_intrinsic.ppx = data2->depthK[i][0]; rgbd_param[i].depth_intrinsic.ppy = data2->depthK[i][1];
				rgbd_param[i].depth_intrinsic.fx = data2->depthK[i][2]; rgbd_param[i].depth_intrinsic.fy = data2->depthK[i][3];
				memcpy(rgbd_param[i].depth_intrinsic.coeffs, data2->depthCoeffs[i], sizeof(float) * 5);

				memcpy(rgbd_param[i].depth_to_color.rotation, data2->depth_to_color_R[i], sizeof(float) * 9);
				memcpy(rgbd_param[i].depth_to_color.translation, data2->depth_to_color_tvec[i], sizeof(float) * 3);

				std::cout << std::endl << std::string(data2->camera_order[i]) << std::endl;
				float tmpRT[16];
				rgbd_param[i].get_depth2color_all_matrix(tmpRT);
				rt[i] = cv::Mat(4, 4, CV_32FC1,tmpRT).clone();
			}

			while (1)
			{
				double time = 0.0;
				for (int i = 0; i < data2->num_of_senseor; i++)
				{
					memcpy(img[i].data, data2->colorData[i], sizeof(uchar) * data2->colorHeight * data2->colorWidth * 3);
					memcpy(depth[i].data, data2->depthData[i], sizeof(ushort) * data2->depthHeight * data2->depthWidth);

					auto start = std::chrono::high_resolution_clock::now();
					/*for (int y = 0; y < data2->depthHeight; y++)
					{
						for (int x = 0; x < data2->depthWidth; x++)
						{
							uint16_t depth_val = (uint16_t)depth[i].at<ushort>(y, x);
							float depth_val_float = (float)depth_val;

							//// deproject - transform - project --> 13~14 miliseconds
							//float2 depth_pixel = { (float)x, (float)y };
							//float3 depth_point = rgbd_param[i].depth_intrinsic.deproject(depth_pixel, depth_val_float);
							//float3 color_point = rgbd_param[i].depth_to_color.transform(depth_point);
							//float2 color_pixel = rgbd_param[i].color_intrinsic.project(color_point);

							//const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y);

							//// all_mat product 9~10 miliseconds
							float* mat = (float*)rt[i].data;
							int cx = (int)std::round(mat[0] * (double)x + mat[1] * (double)y + mat[2] + mat[3] / (double)depth_val_float);
							int cy = (int)std::round(mat[4] * (double)x + mat[5] * (double)y + mat[6] + mat[7] / (double)depth_val_float);

							if (cx < 0 || cy < 0 || cx >= data2->colorWidth || cy >= data2->colorHeight) {
								//std::cout << cx << ", " << cy << std::endl;
								depth2color[i].at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
								continue;
							}
							else {
								uint16_t* val = (uint16_t*)newDepth[i].ptr<uint16_t>(cy, cx);
								*val = (uint16_t)depth_val;
								depth2color[i].at<cv::Vec3b>(y, x) = img[i].at<cv::Vec3b>(cy, cx);
							}
						}
					}*/
					auto elapsed = std::chrono::high_resolution_clock::now() - start;
					time += std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();

					cv::Mat canvas;
					newDepth[i].convertTo(canvas, CV_8UC1, 0.05, -25);
					cv::cvtColor(canvas, canvas, CV_GRAY2BGR);
					cv::imshow("color" + std::string(data2->camera_order[i]), img[i]);
					cv::Mat canvas2;
					depth[i].convertTo(canvas2, CV_8UC1, 0.05, -25);
					cv::imshow("depth" + std::string(data2->camera_order[i]), canvas2);
					cv::imshow(data2->camera_order[i], img[i] * 0.5 + canvas * 0.5);
					cv::imshow("depth2color" + std::string(data2->camera_order[i]), depth2color[i]);

					cv::Mat registerMat;
					cv::Mat caliDepthHistogram(480, 640, CV_16UC1);
					orbbec->getDepthHistogram(depth[i], caliDepthHistogram);
					cv::addWeighted(caliDepthHistogram, (double)(5 / 10.0), img[i], (double)(5 / 10.0), 0.5, registerMat);
					cv::imshow("registerMat " + std::string(data2->camera_order[i]), registerMat);
				}
				cv::waitKey(10);
				time /= 3.0;
				//std::cout << time << std::endl;
			}

			delete[] rt;
			delete[] rgbd_param;
			delete[] depth2color;
			delete[] newDepth;
			delete[] depth;
			delete[] img;
			ipc.exit();
		});
		sensorThread.join();
		registration.join();
	}

	sensorThread.join();
	commendThread.join();

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
			orbbec->enableIR(!orbbec->IRenabled());
			for (int i = 0; i < orbbec->getNumOfCameras(); i++)
				orbbec->openRGBorIR(i);
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
	std::cout << cam_pos << std::endl;
	std::string path = "../data/depth_calibration/";

	while(1)
	{
		if (key == 27)
			break;

		if (bTPC_flag)
		{			
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
	}
}
