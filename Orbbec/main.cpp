#define _CRT_SECURE_NO_WARNINGS
#include <iostream>

#include "functions.h"
#include "camera_matrix.h"
#include "opencv2\core.hpp"
#include "opencv2\imgproc.hpp"
#include "opencv2\highgui.hpp"

#include "orbbec.h"

int main(int argc, char** argv)
{
	IPC ipc("Orbbec.exe");
	RGBDcamera *rgbdData = ipc.connect<RGBDcamera>("Orbbec.exe");

	Sleep(300);
	ipc.start("Orbbec.exe");

	Orbbec orbbec(rgbdData, &ipc);
	orbbec.initialize("..//data//");
	//Orbbec.readCalibrationData("..");
	std::thread sensorThread(&Orbbec::threadRun, &orbbec);
	std::thread forTest([&]() {
		IPC ipc("test.exe");
		RGBDcamera *nData = ipc.connect<RGBDcamera>("Orbbec.exe");
		
		int nCam = nData->num_of_senseor;

		cv::Mat tmp1(nData->colorHeight, nData->colorWidth, CV_8UC3);
		cv::Mat tmp2(nData->depthHeight, nData->depthWidth, CV_16UC1);
		cv::Mat tmp3(nData->colorHeight, nData->colorWidth, CV_8UC3, cv::Scalar(0, 0, 0));

		orbbec.readCalibrationData("..");

		RGBD_Intrinsics colorIntrin, depthIntrin;
		colorIntrin.ppx = nData->colorK[0][0]; colorIntrin.ppy = nData->colorK[0][1]; colorIntrin.fx = nData->colorK[0][2]; colorIntrin.fy = nData->colorK[0][3];
		memcpy(colorIntrin.coeffs, nData->colorCoeffs[0], sizeof(float) * 5);
		depthIntrin.ppx = nData->depthK[0][0]; depthIntrin.ppy = nData->depthK[0][1]; depthIntrin.fx = nData->depthK[0][2]; depthIntrin.fy = nData->depthK[0][3];
		memcpy(depthIntrin.coeffs, nData->depthCoeffs[0], sizeof(float) * 5);

		RGBD_Extrinsics extrinsic;
		memcpy(extrinsic.rotation, nData->depth_to_color_R[0], sizeof(float) * 9);
		memcpy(extrinsic.translation, nData->depth_to_color_tvec[0], sizeof(float) * 3);
		
		for(int i=0; i<9;i++)
		std::cout << extrinsic.rotation[i] << std::endl;
		
		int a = 0;
		while (1)
		{
			if (a++ < 100) {
				Sleep(10);
				continue;
			}
			memcpy(tmp1.data, nData->colorData[0], sizeof(uchar) * 640 * 480 * 3);
			memcpy(tmp2.data, nData->depthData[0], sizeof(ushort) * 640 * 480);
			//std::cout << tmp1 << std::endl;
			tmp3.setTo(0);
			for (int y = 0; y < tmp2.rows; y++)
				for (int x = 0; x < tmp2.cols; x++)
				{
					ushort depth_val = tmp2.at<ushort>(y, x);
					float depth_in_meters = depth_val * 1.f;
					if (depth_val == 0)
						continue;
					float2 depth_pixel = { (float)x, (float)y };
					float3 depth_point = depthIntrin.deproject(depth_pixel, depth_in_meters);
					float3 color_point = extrinsic.transform(depth_point);
					float2 color_pixel = colorIntrin.project(color_point);

					const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y);

					if (cx < 0 || cy < 0 || cx >= tmp1.cols || cy >= tmp1.rows) {
						//std::cout << cx << ", " << cy << std::endl;
					}
					else
					{
						//std::cout << cx << ", " << cy << std::endl;
						//std::cout << (cv::Vec3b)tmp1.at<cv::Vec3b>(cy, cx) << std::endl;
						tmp3.at<cv::Vec3b>(y, x) = tmp1.at<cv::Vec3b>(cy, cx);
					}
				}
			cv::imshow("color_2_depth", tmp3);
			cv::Mat tmp4;
			tmp2.convertTo(tmp4, CV_8U, 0.05, -25);
			cv::imshow("color", tmp1);
			cv::imshow("depth", tmp4);
			//cv::imshow("ir", Orbbec.ir);

			char key = cv::waitKey(10);
			if (key == 'q')
				break;
		}
		
	});
	forTest.join();
	sensorThread.join();

	return 0;
}