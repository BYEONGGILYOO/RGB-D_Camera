#pragma once
#include "functions.h"
#include "OpenNI.h"
#include "camera_matrix.h"

#include <iostream>
#include <mutex>
#include <opencv2/core.hpp>

class Orbbec
{
public:
	enum Resolution
	{
		// SXGA:	1280 x 960
		// VGA:		640 x 480
		// QVGA:	320 x 240
		SXGA, VGA, QVGA
	};
	Orbbec(RGBDcamera *data, IPC *ipc);
	~Orbbec();

	bool initialize(std::string cam_order_path);
	bool openRGBorIR(int dev_idx);
	bool openDepth(int dev_idx);
	void threadRun(std::mutex* mtx);

	// Enter only the data path, not the file name.
	void readCalibrationData(std::string path);
	void writeCalibrationData(std::string path);

private:
	template<typename T>
	void getRGBorIR(int dev_idx, T * output_data, double * time);
	void getDepth(int dev_idx, unsigned short * output_data, double * time);
	bool getData();

public:
	bool startRGBorIRstream(const int dev_idx) const;
	bool startDepthstream(const int dev_idx) const;
	void stopRGBorIRstream(const int dev_idx) const;
	void stopDepthstream(const int dev_idx) const;

	// input data
	void setRGBResolution(const int nWidth, const int nHeight);
	void setRGBResolution(const Resolution res);
	void setDepthResolution(const int nWidth, const int nHeight);
	void setDepthResolution(const Resolution res);
	int getRGBResolution() const;
	int getDepthResolution() const;
	// draw func
	void getDepthHistogram(cv::Mat &src, cv::Mat &dst);

	// flag func
	void enableRegistration(const bool flag = false);
	bool regisrationEnabled() const;
	void enableIR(const bool flag = false);
	bool IRenabled() const;
	void enableDrawImage(const bool flag = false);
	bool drawImageEnabled() const;
	int getNumOfCameras() const;
	void enableOverlap(const bool flag = false);
	bool overlapEnabled() const;

	void stop();
	void start();
private:
	// IPC object
	RGBDcamera *m_pData;
	IPC *m_pIpc;
	
	// input data
	int rgb_width, rgb_height;
	int depth_width, depth_height;

	// openni object
	openni::Device *m_pDevice;
	openni::VideoStream *m_pStreamDepth, *m_pStreamRGBorIR;

	// data
	RGBD_Parameters *m_pRGBDparam;
	std::vector<std::string> cam_order;

	int num_of_cameras;
	int ref_cam_idx;
	
	bool *m_pEnableRegistration;
	double **m_pRegistrationMatrix;

	// commend flag
	bool m_bDrawImage;
	bool m_bIRon;
	bool m_bStopThread;
	bool m_bOverlap;

	int m_depthResolution;
	int m_RGBResolution;
};

