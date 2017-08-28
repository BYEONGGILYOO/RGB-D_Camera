#include "orbbec.h"

#include <iostream>
#include <chrono>
#include "opencv2\core.hpp"
#include "opencv2\imgproc.hpp"
#include "opencv2\highgui.hpp"

Orbbec::Orbbec(RGBDcamera * data, IPC * ipc)
	:m_pData(data), m_pIpc(ipc), rgb_width(640), rgb_height(480), depth_width(640), depth_height(480), num_of_cameras(0),
	m_pDevice(nullptr), m_pStreamDepth(nullptr), m_pStreamRGBorIR(nullptr), m_pRGBDparam(nullptr),
	ref_cam_idx(0),
	m_pEnableRegistration(&data->registration_enable), m_bDrawImage(false), m_bIRon(false), m_bStopThread(false), m_bOverlap(false),
	m_depthResolution(Resolution::VGA), m_RGBResolution(Resolution::VGA)
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

	for (int i = 0; i < num_of_cameras; i++)
	{
		if (m_pRegistrationMatrix[i] != nullptr)
			delete[] m_pRegistrationMatrix[i];
	}
	delete[] m_pRegistrationMatrix;
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

	m_pRegistrationMatrix = new double*[num_of_cameras];

	// object init
	if (num_of_cameras > 1) {
		readCalibrationData(cam_order_path);

		std::cout << "[Info] " << num_of_cameras << " sensors are detected" << std::endl;
		std::cout << "Camera order: ";
		for (int i = 0; i < num_of_cameras; i++)
			std::cout << cam_order[i] << ", ";
		std::cout << std::endl;
	}
	else if (num_of_cameras == 1)	// for test only one camera
	{
		cam_order.push_back("left");
		readParameterYaml(cam_order_path + "calibration_orbbec_" + cam_order[0] + ".yml", &this->m_pRGBDparam[0]);
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

		memcpy(m_pData->depth_to_color_R[0], m_pRGBDparam[0].depth_to_color.rotation, sizeof(float) * 9);
		memcpy(m_pData->depth_to_color_tvec[0], m_pRGBDparam[0].depth_to_color.translation, sizeof(float) * 3);
	}

	// set the parameters
	m_pData->num_of_senseor = num_of_cameras;
	m_pData->ref_cam_idx = ref_cam_idx;

	m_pData->colorWidth = rgb_width;
	m_pData->colorHeight = rgb_height;
	m_pData->depthWidth = depth_width;
	m_pData->depthHeight = depth_height;

	// open the video
	for (int i = 0; i < num_of_cameras; i++)
	{
		m_pDevice[i].open(list_of_devices[i].getUri());
		printf("%c. %s->%s (VID : %d | PID : %d) is connected""at %s\r\n", 'a' + i, list_of_devices[i].getVendor(),
			list_of_devices[i].getName(), list_of_devices[i].getUsbVendorId(), list_of_devices[i].getUsbProductId(), list_of_devices[i].getUri());
				
		m_pDevice[i].setDepthColorSyncEnabled(false);
		openRGBorIR(i);
		openDepth(i);	

		//iMaxDepth[i] = m_pStreamDepth[i].getMaxPixelValue();

		m_pRGBDparam[i].color_intrinsic.width = rgb_width;
		m_pRGBDparam[i].color_intrinsic.height = rgb_height;
		m_pRGBDparam[i].depth_intrinsic.width = depth_width;
		m_pRGBDparam[i].depth_intrinsic.height = depth_height;

		m_pRegistrationMatrix[i] = new double[16];
		m_pRGBDparam[i].get_depth2color_all_matrix(m_pRegistrationMatrix[i]);
	}
	return true;
}

bool Orbbec::openRGBorIR(int dev_idx)
{
	this->stopRGBorIRstream(dev_idx);

	openni::Status rc;
	int nWidth = this->rgb_width, nHeight = this->rgb_height;
	if (m_RGBResolution == Resolution::SXGA)
	{
		nWidth = 1280; nHeight = 1024;
	}
	if (!m_bIRon)
	{
		if (m_pDevice[dev_idx].getSensorInfo(openni::SENSOR_COLOR) != nullptr)
		{
			rc = m_pStreamRGBorIR[dev_idx].create(m_pDevice[dev_idx], openni::SENSOR_COLOR);
			if (rc != openni::STATUS_OK)
			{
				printf("Couldn't create color stream\n%s\n", openni::OpenNI::getExtendedError());
				return false;
			}
		}
		openni::VideoMode mModeColor = m_pStreamRGBorIR[dev_idx].getVideoMode();
		mModeColor.setResolution(nWidth, nHeight);
		//mModeColor.setFps(30);
		//mModeColor.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
		m_pStreamRGBorIR[dev_idx].setVideoMode(mModeColor);
	}
	else
	{
		if (m_pDevice[dev_idx].getSensorInfo(openni::SENSOR_IR) != nullptr)
		{
			rc = m_pStreamRGBorIR[dev_idx].create(m_pDevice[dev_idx], openni::SENSOR_IR);
			if (rc != openni::STATUS_OK)
			{
				printf("Couldn't create ir stream\n%s\n", openni::OpenNI::getExtendedError());
				return false;
			}
		}
		openni::VideoMode mModeIR = m_pStreamRGBorIR[dev_idx].getVideoMode();
		mModeIR.setResolution(nWidth, nHeight);
		mModeIR.setFps(30);
		mModeIR.setPixelFormat(openni::PIXEL_FORMAT_GRAY16);
		m_pStreamRGBorIR[dev_idx].setVideoMode(mModeIR);
	}

	this->startRGBorIRstream(dev_idx);

	return true;
}

bool Orbbec::openDepth(int dev_idx)
{
	this->stopDepthstream(dev_idx);

	openni::Status rc;
	int nWidth = depth_width, nHeight = depth_height;

	if (m_pDevice[dev_idx].getSensorInfo(openni::SENSOR_DEPTH) != nullptr)
	{
		rc = m_pStreamDepth[dev_idx].create(m_pDevice[dev_idx], openni::SENSOR_DEPTH);
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
	m_pStreamDepth[dev_idx].setVideoMode(mModeDepth);

	this->startDepthstream(dev_idx);

	return true;
}

template<typename T>
void Orbbec::getRGBorIR(int dev_idx, T* output_data, double* time)
{
	int changedStreamDummy;
	openni::VideoStream* pStream = &m_pStreamRGBorIR[dev_idx];
	if (!m_pStreamRGBorIR[dev_idx].isValid())
		return;

	openni::Status rc = openni::OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, 1000); // 500ms timeout delay
	
	openni::VideoFrameRef frame_rgb_ir;
	if (rc != openni::STATUS_OK)
	{
		printf("Read failed!\n%s\n", openni::OpenNI::getExtendedError());
		return;
	}
	m_pStreamRGBorIR[dev_idx].readFrame(&frame_rgb_ir);

	int nHeight = rgb_height;
	int nWidth = rgb_width;
	if (m_RGBResolution == Resolution::SXGA)
	{
		nHeight = 1024;
		nWidth = 1280;
	}

	if(!m_bIRon)		// rgb mode
	{
		cv::Mat rgbImage(nHeight, nWidth, CV_8UC3, (T*)frame_rgb_ir.getData());
		// time stamp
		auto now = std::chrono::system_clock::now();
		*time = (double)std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count() / 1000000000.0;
		
		cv::flip(rgbImage, rgbImage, 1);
		cv::Mat bgrImage(rgb_height, rgb_width, CV_8UC3, (T*)output_data);

		if (m_RGBResolution != Resolution::SXGA)
		{
			cv::cvtColor(rgbImage, bgrImage, CV_RGB2BGR);
		}
		else
		{
			cv::Mat tmp = rgbImage(cv::Range(0, 960), cv::Range(0, 1280));
			cv::cvtColor(tmp, bgrImage, CV_RGB2BGR);				
		}		
	}
	else
	{
		// ir mode
		const cv::Mat irImage(m_pData->colorHeight, m_pData->colorWidth, CV_16UC1, (T*)frame_rgb_ir.getData());
		// time stamp
		auto now = std::chrono::system_clock::now();
		*time = (double)std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count() / 1000000000.0;
		
		cv::Mat flipedIRimage(m_pData->colorHeight, m_pData->colorWidth, CV_16UC1, (T*)output_data);
		cv::flip(irImage, flipedIRimage, 1);
	}
}

void Orbbec::getDepth(int dev_idx, unsigned short* output_data, double* time)
{
	int changedStreamDummy;
	openni::VideoStream* pStream = &m_pStreamDepth[dev_idx];
	openni::Status rc = openni::OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, 100); // 100ms timeout delay

	openni::VideoFrameRef frame_depth;
	m_pStreamDepth[dev_idx].readFrame(&frame_depth);

	if (rc == openni::STATUS_OK && frame_depth.isValid())
	{
		const cv::Mat depthImage(depth_height, depth_width, CV_16UC1,
			(ushort*)frame_depth.getData());
		// time stamp
		auto now = std::chrono::system_clock::now();
		*time = (double)std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count() / 1000000000.0;

		cv::flip(depthImage, depthImage, 1);

		cv::Mat newDepth(depth_height, depth_width, CV_16UC1, output_data);
		if (*m_pEnableRegistration)
		{
			for (int y = 0; y < depthImage.rows; y++)
			{
				for (int x = 0; x < depthImage.cols; x++)
				{
					uint16_t depth_val = (uint16_t)depthImage.at<ushort>(y, x);
					float depth_val_float = (float)depth_val;

					// method 1 : 13 ~ 14 milisec
					float2 depth_pixel = { (float)x, (float)y };
					float3 depth_point = m_pRGBDparam[dev_idx].depth_intrinsic.deproject(depth_pixel, depth_val_float, true);
					float3 color_point = m_pRGBDparam[dev_idx].depth_to_color.transform(depth_point);
					float2 color_pixel = m_pRGBDparam[dev_idx].color_intrinsic.project(color_point, true);

					const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y);

					// mehtod 2 : 9 ~ 10 milisec
					/*const int cx = (int)std::round(m_pRegistrationMatrix[dev_idx][0] * (double)x + m_pRegistrationMatrix[dev_idx][1] * (double)y + m_pRegistrationMatrix[dev_idx][2] + m_pRegistrationMatrix[dev_idx][3] / (double)depth_val_float);
					const int cy = (int)std::round(m_pRegistrationMatrix[dev_idx][4] * (double)x + m_pRegistrationMatrix[dev_idx][5] * (double)y + m_pRegistrationMatrix[dev_idx][6] + m_pRegistrationMatrix[dev_idx][7] / (double)depth_val_float);
					*/
					if (cx < 0 || cy < 0 || cx >= m_pRGBDparam[dev_idx].color_intrinsic.width || cy >= m_pRGBDparam[dev_idx].color_intrinsic.height) 
						continue;
					
					uint16_t * val = (uint16_t*)newDepth.ptr<uint16_t>(cy, cx);
					*val = depth_val;
				}
			}
		}
		else		// no registration
			memcpy(output_data, depthImage.data, sizeof(ushort)* depth_height * depth_width);
	}
}

bool Orbbec::getData()
{
	bool stream_is_updated = false;

	// buff init
	uchar** rgb_buff = nullptr;
	ushort** ir_buff = nullptr;
	if (!m_bIRon)
		rgb_buff = new uchar*[num_of_cameras];
	else
		ir_buff = new ushort*[num_of_cameras];

	ushort** depth_buff = new ushort*[num_of_cameras];

	double* color_time_buff = new double[num_of_cameras];
	double* depth_time_buff = new double[num_of_cameras];

	// get the data at buff memory
	for (int i = 0; i < num_of_cameras; i++)
	{
		stream_is_updated = true;
		if (!m_bIRon)
		{
			rgb_buff[i] = new uchar[this->rgb_width * this->rgb_height * 3];
			getRGBorIR(i, rgb_buff[i], &color_time_buff[i]);
		}
		else
		{
			ir_buff[i] = new ushort[this->depth_width * this->depth_width];
			getRGBorIR(i, ir_buff[i], &color_time_buff[i]);
		}

		depth_buff[i] = new ushort[this->depth_width * this->depth_height];
		memset(depth_buff[i], 0, sizeof(ushort) * this->depth_width * this->depth_height);
		getDepth(i, depth_buff[i], &depth_time_buff[i]);
	}

	// update buff to shared memory
	if (!m_bIRon)
		for (int i = 0; i < num_of_cameras; i++) {
			memcpy(m_pData->colorData[i], rgb_buff[i], sizeof(uchar)*this->rgb_width*this->rgb_height * 3);
			memcpy(&m_pData->colorTime[i], &color_time_buff[i], sizeof(double));
		}
	else
		for (int i = 0; i < num_of_cameras; i++) {
			memcpy(m_pData->irData[i], ir_buff[i], sizeof(ushort)*this->depth_width*this->depth_height * 1);
			memcpy(&m_pData->colorTime[i], &color_time_buff[i], sizeof(double));
		}
	for (int i = 0; i < num_of_cameras; i++) {
		memcpy(m_pData->depthData[i], depth_buff[i], sizeof(ushort)*this->depth_width*this->depth_height * 1);
		memcpy(&m_pData->depthTime[i], &depth_time_buff[i], sizeof(double));
	}

	// release buff memory
	for (int i = 0; i < num_of_cameras; i++)
	{
		if (rgb_buff != nullptr)
			delete[] rgb_buff[i];
		if (ir_buff != nullptr)
			delete[] ir_buff[i];
		delete[] depth_buff[i];
	}
	if(rgb_buff != nullptr)
		delete[] rgb_buff;
	if (ir_buff != nullptr)
		delete[] ir_buff;
	delete[] depth_buff;
	delete[] color_time_buff;
	delete[] depth_time_buff;

	
	if (m_bOverlap) {
			for (int i = 0; i < num_of_cameras; i++)
			{
				cv::Mat canvas;
				cv::Mat depth = cv::Mat(depth_height, depth_width, CV_16UC1, m_pData->depthData[i]).clone();
				cv::Mat color(rgb_height, rgb_width, CV_8UC3, m_pData->colorData[i]);
				
				if (m_depthResolution != m_RGBResolution)
					cv::resize(depth, depth, cv::Size(rgb_width, rgb_height));
				cv::Mat caliDepthHistogram(depth_height, depth_width, CV_16UC1);
				getDepthHistogram(depth, caliDepthHistogram);
				cv::addWeighted(caliDepthHistogram, (double)(5 / 10.0), color, (double)(5 / 10.0), 0.5, canvas);
				cv::imshow("overlap img " + std::string(m_pData->camera_order[i]), canvas);
			}
		cv::waitKey(10);
	}
	else
		for (int i = 0; i < num_of_cameras; i++)
			cv::destroyWindow("overlap img " + std::string(m_pData->camera_order[i]));


	if (m_bDrawImage)
	{
		if (!m_bIRon)
		{
			cv::Mat canvas;
			for (int i = 0; i < num_of_cameras; i++)
			{
				cv::Mat tmpRGB(this->rgb_height, this->rgb_width, CV_8UC3, m_pData->colorData[i]);
				if (i == 0)
					canvas = tmpRGB.clone();
				else
					cv::hconcat(canvas, tmpRGB, canvas);
			}
			cv::imshow("rgb", canvas);
			cv::destroyWindow("ir");
		}
		else
		{
			cv::Mat canvas;
			for (int i = 0; i < num_of_cameras; i++)
			{
				cv::Mat ir(this->depth_height, this->depth_width, CV_16UC1, m_pData->irData[i]);
				cv::Mat tmpIR;
				double min, max;
				cv::minMaxLoc(ir, &min, &max);
				ir.convertTo(tmpIR, CV_8UC1, 1 / max * 255, 0);
				if (i == 0)
					canvas = tmpIR.clone();
				else
					cv::hconcat(canvas, tmpIR, canvas);
			}
			cv::imshow("ir", canvas);
			cv::destroyWindow("rgb");
		}
		cv::Mat canvas;
		for (int i = 0; i < num_of_cameras; i++)
		{
			cv::Mat depth(this->depth_height, this->depth_width, CV_16UC1, m_pData->depthData[i]);
			cv::Mat tmpDepth;
			depth.convertTo(tmpDepth, CV_8UC1, 0.05, -25);
			if (i == 0)
				canvas = tmpDepth.clone();
			else
				cv::hconcat(canvas, tmpDepth, canvas);
		}
		cv::imshow("depth", canvas);
		cv::waitKey(10);
	}
	else
	{
		for (int i = 0; i < num_of_cameras; i++)
		{
			cv::destroyWindow("rgb");
			cv::destroyWindow("ir");
			cv::destroyWindow("depth");
		}
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

void Orbbec::setRGBResolution(const int nWidth, const  int nHeight)
{
	this->rgb_width = nWidth;
	this->rgb_height = nHeight;
}

void Orbbec::setRGBResolution(const Resolution res)
{
	this->m_RGBResolution = res;
	switch (res)
	{
	case Orbbec::Resolution::QVGA:
		this->rgb_width = 320;
		this->rgb_height = 240;
		break;
	case Orbbec::Resolution::VGA:
		this->rgb_width = 640;
		this->rgb_height = 480;
		break;
	case Orbbec::Resolution::SXGA:
		this->rgb_width = 1280;
		this->rgb_height = 960;
		//this->rgb_height = 1024;
		break;
	default:
		this->rgb_width = 640;
		this->rgb_height = 480;
		break;
	}

	for (int i = 0; i < num_of_cameras; i++) {
		m_pData[i].colorWidth = rgb_width;
		m_pData[i].colorHeight = rgb_height;
		//this->stopRGBorIRstream(i);
		openRGBorIR(i);
	}
}

void Orbbec::setDepthResolution(const int nWidth, const int nHeight)
{
	this->depth_width = nWidth;
	this->depth_height = nHeight;
}

void Orbbec::setDepthResolution(const Resolution res)
{
	m_depthResolution = res;
	switch (res)
	{
	case Orbbec::Resolution::SXGA:
		throw std::runtime_error("Depth Image Size does not support SXGA");
		break;
	case Orbbec::Resolution::VGA:
		this->depth_width = 640;
		this->depth_height = 480;
		break;
	case Orbbec::Resolution::QVGA:
		this->depth_width = 320;
		this->depth_height = 240;
		break;
	default:
		break;
	}
}

int Orbbec::getRGBResolution() const
{
	return m_RGBResolution;
}

int Orbbec::getDepthResolution() const
{
	return this->m_depthResolution;
}

void Orbbec::getDepthHistogram(cv::Mat & src, cv::Mat & dst)
{
	float depthHistogram[65536];
	int numberOfPoints = 0;
	cv::Mat depthHist(src.rows, src.cols, CV_8UC3);
	memset(depthHistogram, 0, sizeof(depthHistogram));
	for (int y = 0; y < src.rows; ++y)
	{
		ushort* depthCell = (ushort*)src.ptr<uchar>(y);
		for (int x = 0; x < src.cols; ++x)
		{
			if (*depthCell != 0)
			{
				depthHistogram[*depthCell]++;
				numberOfPoints++;
			}
			depthCell++;
		}
	}

	for (int nIndex = 1; nIndex < sizeof(depthHistogram) / sizeof(int); nIndex++)
	{
		depthHistogram[nIndex] += depthHistogram[nIndex - 1];
	}
	for (int nIndex = 1; nIndex < sizeof(depthHistogram) / sizeof(int); nIndex++)
	{
		depthHistogram[nIndex] = (numberOfPoints - depthHistogram[nIndex]) / numberOfPoints;
	}
	for (int y = 0; y < src.rows; ++y)
	{
		ushort* depthCell = (ushort*)src.ptr<uchar>(y);
		uchar * showcell = (uchar *)depthHist.ptr<uchar>(y);
		for (int x = 0; x < src.cols; ++x)
		{
			char depthValue = depthHistogram[*depthCell] * 255;
			*showcell++ = 0;
			*showcell++ = depthValue;
			*showcell++ = depthValue;

			depthCell++;
		}
	}
	dst = depthHist;
}

void Orbbec::enableRegistration(const bool flag)
{
	*m_pEnableRegistration = flag;
}

bool Orbbec::regisrationEnabled() const
{
	return *m_pEnableRegistration;
}

void Orbbec::enableIR(const bool flag)
{
	this->m_bIRon = flag;
}

bool Orbbec::IRenabled() const
{
	return this->m_bIRon;
}

void Orbbec::threadRun(std::mutex* mtx)
{
	function_cummunication *state = m_pIpc->get_state("Orbbec.exe");

	while (!m_bStopThread)
	{
		mtx->lock();
		getData();
		mtx->unlock();
		Sleep(70);
		if (m_pIpc->exit())
		{
			Sleep(30);
			break;
		}
	}
}

void Orbbec::readCalibrationData(std::string path)
{
	std::string cam_name;
	readCameraOrder(path + "cam_order.yml", cam_name, cam_order, ref_cam_idx);

	for (int i = 0; i < num_of_cameras; i++)
	{
		readParameterYaml(path + "calibration_orbbec_" + cam_order[i] + ".yml", &this->m_pRGBDparam[i]);
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

		
		memcpy(m_pData->depth_to_color_R[i], m_pRGBDparam[i].depth_to_color.rotation, sizeof(float) * 9);
		memcpy(m_pData->depth_to_color_tvec[i], m_pRGBDparam[i].depth_to_color.translation, sizeof(float) * 3);
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
void Orbbec::enableDrawImage(const bool flag)
{
	this->m_bDrawImage = flag;
}

bool Orbbec::drawImageEnabled() const
{
	return this->m_bDrawImage;
}

int Orbbec::getNumOfCameras() const
{
	return this->num_of_cameras;
}

void Orbbec::stop()
{
	m_bStopThread = true;
}

void Orbbec::start()
{
	m_bStopThread = false;
}

void Orbbec::enableOverlap(const bool flag)
{
	this->m_bOverlap = flag;
}

bool Orbbec::overlapEnabled() const
{
	return this->m_bOverlap;
}