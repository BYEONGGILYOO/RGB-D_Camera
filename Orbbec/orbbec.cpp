#include "orbbec.h"

#include <iostream>
#include "opencv2\core.hpp"
#include "opencv2\imgproc.hpp"


Orbbec::Orbbec(RGBDcamera * data, IPC * ipc)
	:m_pData(data), m_pIpc(ipc), num_of_cameras(0),
	m_pDevice(nullptr), m_pStreamDepth(nullptr), m_pStreamRGBorIR(nullptr), m_pRGBDparam(nullptr), m_bIRon(false)
{
	
}

Orbbec::~Orbbec()
{
	if (m_pDevice != nullptr)
		delete[] m_pDevice;
	m_pDevice = nullptr;

	if (m_pStreamRGBorIR != nullptr)
		delete[] m_pStreamRGBorIR;
	m_pStreamRGBorIR = nullptr;

	if (m_pStreamDepth != nullptr)
		delete[] m_pStreamDepth;
	m_pStreamDepth = nullptr;

	if (m_pRGBDparam != nullptr)
		delete[] m_pRGBDparam;
	m_pRGBDparam = nullptr;
}

bool Orbbec::initialize(std::string cam_order_path)
{
	openni::OpenNI::initialize();
	openni::Array<openni::DeviceInfo> list_of_devices;
	openni::OpenNI::enumerateDevices(&list_of_devices);

	this->num_of_cameras = list_of_devices.getSize();

	if (num_of_cameras == 0) {
		throw std::runtime_error("No device detected!");
		return false;
	}

	// make object
	m_pDevice = new openni::Device[num_of_cameras];
	m_pStreamRGBorIR = new openni::VideoStream[num_of_cameras];
	m_pStreamDepth = new openni::VideoStream[num_of_cameras];

	m_pRGBDparam = new RGBD_Parameters[num_of_cameras];

	// object init
	if (num_of_cameras > 1) {
		readCalibrationData(cam_order_path);

		std::cout << "[Info] " << num_of_cameras << " sensors are detected" << std::endl;
		std::cout << "Camera order: ";
		for (int i = 0; i < num_of_cameras; i++)
			std::cout << cam_order[i] << ", ";
		std::cout << std::endl;
	}
	else if (num_of_cameras == 1)	// for test
	{
		cam_order.push_back("left");
		readParameterYaml(cam_order_path + "orbbec_calibration_" + cam_order[0] + ".yml", &this->m_pRGBDparam[0]);
		const char* tmp = cam_order[0].c_str();
		memcpy(m_pData->camera_order[0], tmp, strlen(tmp) + 1);
		m_pData->colorK[0][0] = m_pRGBDparam[0].color_intrinsic.ppx;
		m_pData->colorK[0][1] = m_pRGBDparam[0].color_intrinsic.ppy;
		m_pData->colorK[0][2] = m_pRGBDparam[0].color_intrinsic.fx;
		m_pData->colorK[0][3] = m_pRGBDparam[0].color_intrinsic.fy;
		memcpy(m_pData->colorCoeffs[0], m_pRGBDparam[0].color_intrinsic.coeffs, sizeof(float) * 5);

		m_pData->depthK[0][0] = m_pRGBDparam[0].depth_intrinsic.ppx;
		m_pData->depthK[0][1] = m_pRGBDparam[0].depth_intrinsic.ppy;
		m_pData->depthK[0][2] = m_pRGBDparam[0].depth_intrinsic.fx;
		m_pData->depthK[0][3] = m_pRGBDparam[0].depth_intrinsic.fy;
		memcpy(m_pData->depthCoeffs[0], m_pRGBDparam[0].depth_intrinsic.coeffs, sizeof(float) * 5);


		cv::Mat mat_tmp = cv::Mat(3, 3, CV_32FC1, m_pRGBDparam[0].depth_to_color.rotation).clone();
		//mat_tmp = mat_tmp.inv();
		memcpy(m_pData->depth_to_color_R[0], mat_tmp.data, sizeof(float) * 9);
		mat_tmp = cv::Mat(3, 1, CV_32FC1, m_pRGBDparam[0].depth_to_color.translation).clone();
		//mat_tmp = -mat_tmp;// *0.001f;
		memcpy(m_pData->depth_to_color_tvec[0], mat_tmp.data, sizeof(float) * 3);
	}
	//
	m_pData->num_of_senseor = num_of_cameras;
	m_pData->ref_cam_idx = ref_cam_idx;

	int nWidth = 640;
	int nHeight = 480;
	m_pData->colorHeight = nHeight;
	m_pData->colorWidth = nWidth;
	m_pData->depthHeight = nHeight;
	m_pData->depthWidth = nWidth;

	for (int i = 0; i < num_of_cameras; i++)
	{
		m_pDevice[i].open(list_of_devices[i].getUri());
		printf("%c. %s->%s (VID : %d | PID : %d) is connected""at %s\r\n", 'a' + i, list_of_devices[i].getVendor(),
			list_of_devices[i].getName(), list_of_devices[i].getUsbVendorId(), list_of_devices[i].getUsbProductId(), list_of_devices[i].getUri());

		openni::Status rc;

		if (!m_bIRon)
		{
			if (m_pDevice[i].getSensorInfo(openni::SENSOR_COLOR) != nullptr)
			{
				rc = m_pStreamRGBorIR[i].create(m_pDevice[i], openni::SENSOR_COLOR);
				if (rc != openni::STATUS_OK)
				{
					printf("Couldn't create color stream\n%s\n", openni::OpenNI::getExtendedError());
					return false;
				}
			}
			openni::VideoMode mModeColor = m_pStreamRGBorIR[i].getVideoMode();
			mModeColor.setResolution(nWidth, nHeight);
			mModeColor.setFps(30);
			mModeColor.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
			m_pStreamRGBorIR[i].setVideoMode(mModeColor);
		}
		else
		{
			if (m_pDevice[i].getSensorInfo(openni::SENSOR_IR) != nullptr)
			{
				rc = m_pStreamRGBorIR[i].create(m_pDevice[i], openni::SENSOR_IR);
				if (rc != openni::STATUS_OK)
				{
					printf("Couldn't create ir stream\n%s\n", openni::OpenNI::getExtendedError());
					return false;
				}
			}
			openni::VideoMode mModeIR = m_pStreamRGBorIR[i].getVideoMode();
			mModeIR.setResolution(nWidth, nHeight);
			mModeIR.setFps(30);
			mModeIR.setPixelFormat(openni::PIXEL_FORMAT_GRAY16);
			m_pStreamRGBorIR[i].setVideoMode(mModeIR);
		}
		
		if (m_pDevice[i].getSensorInfo(openni::SENSOR_DEPTH) != nullptr)
		{
			rc = m_pStreamDepth[i].create(m_pDevice[i], openni::SENSOR_DEPTH);
			if (rc != openni::STATUS_OK)
			{
				printf("Couldn't create depth stream\n%s\n", openni::OpenNI::getExtendedError());
				return false;
			}
		}		

		openni::VideoMode mModeDepth;
		mModeDepth.setResolution(nWidth, nHeight);
		mModeDepth.setFps(30);
		mModeDepth.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
		m_pStreamDepth[i].setVideoMode(mModeDepth);		

		//iMaxDepth[i] = m_pStreamDepth[i].getMaxPixelValue();

		m_pRGBDparam[i].color_intrinsic.width = nWidth;
		m_pRGBDparam[i].color_intrinsic.height = nHeight;
		m_pRGBDparam[i].depth_intrinsic.width = nWidth;
		m_pRGBDparam[i].depth_intrinsic.height = nHeight;
	}
	return true;
}

void Orbbec::getRGBorIR(int dev_idx)
{
	int changedStreamDummy;
	openni::VideoStream* pStream = &m_pStreamRGBorIR[dev_idx];
	openni::Status rc = openni::OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, 100); // 100ms timeout delay

	openni::VideoFrameRef frame_rgb_ir;
	m_pStreamRGBorIR[dev_idx].readFrame(&frame_rgb_ir);

	if (rc != openni::STATUS_OK || !frame_rgb_ir.isValid())
	{
		printf("Read failed!\n%s\n", openni::OpenNI::getExtendedError());
		return;
	}
	if(!m_bIRon)
	{
		const cv::Mat rgbImage(m_pData->colorHeight, m_pData->colorWidth, CV_8UC3,
			(uchar*)frame_rgb_ir.getData());
		cv::flip(rgbImage, rgbImage, 1);
		cv::Mat bgrImage(m_pData->colorHeight, m_pData->colorWidth, CV_8UC3,
			m_pData->colorData[dev_idx]);
		cv::cvtColor(rgbImage, bgrImage, CV_RGB2BGR);
	}
	else
	{
		const cv::Mat irImage(480, 640, CV_16UC1, (uint16_t*)frame_rgb_ir.getData());
		cv::Mat flipedIRimage(480, 640, CV_16UC1, m_pData->irData[dev_idx]);
		cv::flip(irImage, flipedIRimage, 1);
	}
}

void Orbbec::getDepth(int dev_idx)
{
	int changedStreamDummy;
	openni::VideoStream* pStream = &m_pStreamDepth[dev_idx];
	openni::Status rc = openni::OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, 100); // 100ms timeout delay

	openni::VideoFrameRef frame_depth;
	m_pStreamDepth[dev_idx].readFrame(&frame_depth);

	if (rc == openni::STATUS_OK && frame_depth.isValid())
	{
		if (frame_depth.getHeight() != m_pData->depthHeight || frame_depth.getWidth() != m_pData->depthWidth) {
			throw std::runtime_error("resolution is incorrect");
			return;
		}
		const cv::Mat depthImage(m_pData->depthHeight, m_pData->depthWidth, CV_16UC1,
			(ushort*)frame_depth.getData());
		cv::flip(depthImage, depthImage, 1);
		memcpy(m_pData->depthData[dev_idx], frame_depth.getData(), sizeof(ushort)*m_pData->depthHeight*m_pData->depthWidth);
	}
}


bool Orbbec::getData()
{
	bool stream_is_updated = false;

	for (int i = 0; i < num_of_cameras; i++)
	{
		stream_is_updated = true;
		getRGBorIR(i);
		getDepth(i);
	}
	return stream_is_updated;
}

bool Orbbec::startRGBorIRstream(const int dev_idx) const
{
	openni::Status rc = m_pStreamRGBorIR[dev_idx].start();
	if (rc != openni::STATUS_OK)
	{
		printf("Couldn't start the rgb stream\n%s\n", openni::OpenNI::getExtendedError());
		return false;
	}
	return true;
}

bool Orbbec::startDepthstream(const int dev_idx) const
{
	openni::Status rc = m_pStreamDepth[dev_idx].start();
	if (rc != openni::STATUS_OK)
	{
		printf("Couldn't start the depth stream\n%s\n", openni::OpenNI::getExtendedError());
		return false;
	}
	return true;
}

void Orbbec::stopRGBorIRstream(const int dev_idx) const
{
	m_pStreamRGBorIR[dev_idx].stop();
	m_pStreamRGBorIR[dev_idx].destroy();
}

void Orbbec::stopDepthstream(const int dev_idx) const
{
	m_pStreamDepth[dev_idx].stop();
	m_pStreamDepth[dev_idx].destroy();
}

void Orbbec::threadRun()
{
	function_cummunication *state = m_pIpc->get_state("Orbbec.exe");

	while (1)
	{
		getData();

		if (m_pIpc->exit())
		{
			Sleep(30);
			break;
		}
	}
}

void Orbbec::readCalibrationData(std::string path)
{
	readCameraOrder(path + "cam_order.yml", cam_order, ref_cam_idx);

	for (int i = 0; i < num_of_cameras; i++)
	{
		readParameterYaml(path + "orbbec_calibration_" + cam_order[i] + ".yml", &this->m_pRGBDparam[i]);
		const char* tmp = cam_order[i].c_str();
		memcpy(m_pData->camera_order[i], tmp, strlen(tmp) + 1);
		m_pData->colorK[i][0] = m_pRGBDparam[i].color_intrinsic.ppx;
		m_pData->colorK[i][1] = m_pRGBDparam[i].color_intrinsic.ppy;
		m_pData->colorK[i][2] = m_pRGBDparam[i].color_intrinsic.fx;
		m_pData->colorK[i][3] = m_pRGBDparam[i].color_intrinsic.fy;
		memcpy(m_pData->colorCoeffs[i], m_pRGBDparam[i].color_intrinsic.coeffs, sizeof(float) * 5);

		m_pData->depthK[i][0] = m_pRGBDparam[i].depth_intrinsic.ppx;
		m_pData->depthK[i][1] = m_pRGBDparam[i].depth_intrinsic.ppy;
		m_pData->depthK[i][2] = m_pRGBDparam[i].depth_intrinsic.fx;
		m_pData->depthK[i][3] = m_pRGBDparam[i].depth_intrinsic.fy;
		memcpy(m_pData->depthCoeffs[i], m_pRGBDparam[i].depth_intrinsic.coeffs, sizeof(float) * 5);

		
		cv::Mat mat_tmp = cv::Mat(3, 3, CV_32FC1, m_pRGBDparam[i].depth_to_color.rotation).clone();
		//mat_tmp = mat_tmp.inv();
		memcpy(m_pData->depth_to_color_R[i], mat_tmp.data, sizeof(float) * 9);
		mat_tmp = cv::Mat(3, 1, CV_32FC1, m_pRGBDparam[i].depth_to_color.translation).clone();
		//mat_tmp = -mat_tmp;// *0.001f;
		memcpy(m_pData->depth_to_color_tvec[i], mat_tmp.data, sizeof(float) * 3);
		
		//memcpy(m_pData->depth_to_color_R, m_pRGBDparam[i].depth_to_color.rotation, sizeof(float) * 9);
		//cv::Mat mat_tmp = cv::Mat(3, 1, CV_32FC1, m_pRGBDparam[i].depth_to_color.translation).clone();
		//mat_tmp = -mat_tmp;// *0.001f;
		//memcpy(m_pData->depth_to_color_tvec, mat_tmp.data, sizeof(float) * 3);
		////memcpy(m_pData->depth_to_color_tvec, m_pRGBDparam[i].depth_to_color.translation, sizeof(float) * 3);
	}
}

void Orbbec::writeCalibrationData(std::string path)
{
	for (int i = 0; i < num_of_cameras; i++)
	{
		cam_order.push_back(std::to_string(i));
		writeParametersYaml(std::string(path + "\\Orbbec_calibration_" + cam_order[i] + ".yml"), &this->m_pRGBDparam[i]);
	}
}
