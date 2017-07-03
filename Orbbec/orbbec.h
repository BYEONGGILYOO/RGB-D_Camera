#pragma once
#include "functions.h"
#include "OpenNI.h"
#include "camera_matrix.h"

class Orbbec
{
public:
	Orbbec(RGBDcamera *data, IPC *ipc);
	~Orbbec();

	bool initialize(std::string cam_order_path);
	void threadRun();

	// Enter only the data path, not the file name.
	void readCalibrationData(std::string path);
	void writeCalibrationData(std::string path);

	// for test
	bool bIR;
	int widthIR, heightIR;
	cv::Mat ir;
private:
	void getColor(int dev_idx);
	void getDepth(int dev_idx);
	bool getData();
private:
	RGBDcamera *m_pData;
	IPC *m_pIpc;
	int num_of_cameras;
	int ref_cam_idx;
	openni::Device *m_pDevice;
	openni::VideoStream *m_pStreamDepth, *m_pStreamColor, *m_pStreamIr;
	RGBD_Parameters *m_pRGBDparam;
	std::vector<std::string> cam_order;
};

