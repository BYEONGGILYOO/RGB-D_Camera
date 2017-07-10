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
	int widthIR, heightIR;
	bool m_bIRon;
private:
	void getRGBorIR(int dev_idx);
	void getDepth(int dev_idx);
	bool getData();

public:
	bool startRGBorIRstream(const int dev_idx) const;
	bool startDepthstream(const int dev_idx) const;
	void stopRGBorIRstream(const int dev_idx) const;
	void stopDepthstream(const int dev_idx) const;

	void getDepthHistogram(cv::Mat &src, cv::Mat &dst);
	void enableRegistration(const bool flag = false);
	bool regisrationEnabled() const;
private:
	RGBDcamera *m_pData;
	IPC *m_pIpc;
	int num_of_cameras;
	int ref_cam_idx;
	openni::Device *m_pDevice;
	openni::VideoStream *m_pStreamDepth, *m_pStreamRGBorIR;
	RGBD_Parameters *m_pRGBDparam;
	std::vector<std::string> cam_order;
	bool m_bEnableRegistration;
	float **m_pRegistrationMatrix;
};

