#pragma once
#define _CRT_SECURE_NO_WARNINGS

#ifndef ORBBEC_H
#define ORBBEC_H

#include "functions.h"
#include "OpenNI.h"
#include "camera_matrix.h"

#include <iostream>
#include <mutex>
#include <opencv2/core.hpp>


class MouseInterface
{
private:
	int number_of_mouse;

public:
	MouseInterface::MouseInterface(int maxMouse_num = 1);
	MouseInterface::~MouseInterface();

	struct mouse_info
	{
		int x = 0, y = 0, click = 0;
		int change_event;
	};
	mouse_info* Mouse;
	void setWindow(char* name, int mouse_idx);
};

class PointCloudView
{
private:
	MouseInterface m;

	struct PointCloudInfo
	{
		std::vector<cv::Point3f> pc;
		cv::Scalar Color;
		int size;
	};
	struct LineCloudInfo
	{
		std::vector<std::pair<cv::Point3f, cv::Point3f>> lc;
		cv::Scalar Color;
		int size;
	};

	std::map<std::string, PointCloudInfo> pointclouds;
	std::map<std::string, LineCloudInfo> lineclouds;
	std::map<std::string, LineCloudInfo> frames;

	cv::Mat R, T;
	cv::Mat base_R, base_T;
	cv::Mat plot;
	int cx, cy;
	int m_x, m_y, m_change_event, m_update = false;
	std::vector<std::pair<cv::Point3f, cv::Point3f>> gridPoints;

	void makeXZGridPoints(cv::Rect& Range, float Size, std::vector<std::pair<cv::Point3f, cv::Point3f>>& Output)
	{
		float xMin = (float)Range.x;
		float yMin = (float)Range.y;

		float xMax = (float)Range.x + (float)Range.width;
		float yMax = (float)Range.y + (float)Range.height;

		for (float x = (float)Range.x; x < xMax; x += Size)
		{
			Output.push_back(std::pair<cv::Point3f, cv::Point3f>(cv::Point3f(), cv::Point3f()));
			Output.back().first.x = x;
			Output.back().first.y = 0.0f;
			Output.back().first.z = yMin;

			Output.back().second.x = x;
			Output.back().second.y = 0.0f;
			Output.back().second.z = yMax;
		}
		for (float y = (float)Range.y; y < yMax; y += Size)
		{
			Output.push_back(std::pair<cv::Point3f, cv::Point3f>(cv::Point3f(), cv::Point3f()));
			Output.back().first.x = xMin;
			Output.back().first.y = 0.0f;
			Output.back().first.z = y;

			Output.back().second.x = xMax;
			Output.back().second.y = 0.0f;
			Output.back().second.z = y;
		}
	};
	cv::Point project(cv::Point3f& point, float scale = 1.0f)
	{
		float x = R.at<float>(0, 0) * point.x + R.at<float>(0, 1) * point.y + R.at<float>(0, 2) * point.z + T.at<float>(0, 0);
		float y = R.at<float>(1, 0) * point.x + R.at<float>(1, 1) * point.y + R.at<float>(1, 2) * point.z + T.at<float>(1, 0);

		x *= scale;
		y *= scale;

		return cv::Point((int)x + cx, (int)y + cy);
	}
	void drawGrid(void)
	{
		for (int i = 0; i < (int)gridPoints.size(); i++)
		{
			cv::Point p1 = project(gridPoints[i].first, 0.1f);
			cv::Point p2 = project(gridPoints[i].second, 0.1f);
			cv::line(plot, p1, p2, cv::Scalar(125, 125, 125), 1, 8);
		}
	};
	void drawAxis(void)
	{
		float length = 2500.0f;

		cv::Point o = project(cv::Point3f(0, 0, 0), 0.1f);
		cv::Point x = project(cv::Point3f(length, 0, 0), 0.1f);
		cv::Point y = project(cv::Point3f(0, length, 0), 0.1f);
		cv::Point z = project(cv::Point3f(0, 0, length), 0.1f);

		cv::line(plot, o, x, cv::Scalar(0, 0, 255), 2, 8);
		cv::line(plot, o, y, cv::Scalar(0, 255, 0), 2, 8);
		cv::line(plot, o, z, cv::Scalar(255, 0, 0), 2, 8);

		cv::putText(plot, "X", x, 0, 0.5, cv::Scalar(0, 0, 255));
		cv::putText(plot, "Y", y, 0, 0.5, cv::Scalar(0, 255, 0));
		cv::putText(plot, "Z", z, 0, 0.5, cv::Scalar(255, 0, 0));
	}
	void drawPoints(void)
	{
		std::map<std::string, PointCloudInfo>::iterator i = pointclouds.begin();

		for (; i != pointclouds.end(); i++)
			for (int p = 0; p < (int)i->second.pc.size(); p++)
			{
				cv::Point p1 = project(i->second.pc[p], 0.1f);
				cv::circle(plot, p1, i->second.size, i->second.Color);
			}
	}
	void drawLines(void)
	{
		std::map<std::string, LineCloudInfo>::iterator i = lineclouds.begin();

		for (; i != lineclouds.end(); i++)
			for (int p = 0; p < (int)i->second.lc.size(); p++)
			{
				cv::Point p1 = project(i->second.lc[p].first, 0.1f);
				cv::Point p2 = project(i->second.lc[p].second, 0.1f);
				cv::line(plot, p1, p2, i->second.Color, i->second.size);
			}
	}
	void drawFrames(void)
	{
		std::map<std::string, LineCloudInfo>::iterator i = frames.begin();

		for (; i != frames.end(); i++)
			for (int p = 0; p < (int)i->second.lc.size(); p++)
			{
				cv::Point p1 = project(i->second.lc[p].first, 0.1f);
				cv::Point p2 = project(i->second.lc[p].second, 0.1f);
				cv::line(plot, p1, p2, i->second.Color, i->second.size);
			}
	}
	void projectionUpdate(void)
	{
		if (m.Mouse[0].change_event != 4 && m_change_event == 4)
		{//in
			m_x = m.Mouse[0].x;
			m_y = m.Mouse[0].y;

			base_R = R.clone();
			base_T = T.clone();

			m_update = true;
		}
		if (m_update)
		{
			int dx = m.Mouse[0].x - m_x;
			int dy = m.Mouse[0].y - m_y;

			float dx_radian = 0.5f * (float)dx / (float)cx * (float)CV_PI;
			float dy_radian = 0.5f *(float)dy / (float)cy * (float)CV_PI;

			cv::Mat temp_XZ_R = cv::Mat::eye(cv::Size(3, 3), CV_32FC1);
			temp_XZ_R.at<float>(0, 0) = cos(dx_radian);
			temp_XZ_R.at<float>(0, 2) = -sin(dx_radian);
			temp_XZ_R.at<float>(2, 0) = sin(dx_radian);
			temp_XZ_R.at<float>(2, 2) = cos(dx_radian);

			cv::Mat temp_YZ_R = cv::Mat::eye(cv::Size(3, 3), CV_32FC1);
			temp_YZ_R.at<float>(1, 1) = cos(dy_radian);
			temp_YZ_R.at<float>(1, 2) = -sin(dy_radian);
			temp_YZ_R.at<float>(2, 1) = sin(dy_radian);
			temp_YZ_R.at<float>(2, 2) = cos(dy_radian);

			cv::Mat temp_R = temp_XZ_R * temp_YZ_R;
			cv::Mat temp_T = cv::Mat::eye(cv::Size(1, 3), CV_32FC1);

			R = temp_R * base_R;
			T = temp_T + base_T;

			if (m.Mouse[0].change_event != 1 && m_change_event == 1)
			{//out
			 //if (!base_R.empty()) R = base_R * R;
			 //if (!base_T.empty()) T = base_T + T;

				m_update = false;
			}
		}

		m_change_event = m.Mouse[0].change_event;

		//printf("%d %d\n", m_c, m_t);
	}
public:
	PointCloudView::PointCloudView()
	{
		R = cv::Mat::eye(cv::Size(3, 3), CV_32FC1);
		T = cv::Mat::eye(cv::Size(1, 3), CV_32FC1);

		float gridSize = 500.0f;
		cv::Rect XZplane(-5000, -5000, 10000, 10000);
		makeXZGridPoints(XZplane, 500.0f, gridPoints);
	}

	void clear(std::string Name = std::string("points"))
	{
		std::map<std::string, PointCloudInfo>::iterator finder_P = pointclouds.find(Name);
		if (finder_P != pointclouds.end())
		{
			pointclouds.erase(finder_P);
		}

		std::map<std::string, LineCloudInfo>::iterator finder_L = lineclouds.find(Name);
		if (finder_L != lineclouds.end())
		{
			lineclouds.erase(finder_L);
		}
	}
	void insert(std::vector<cv::Point3f>& PointCloud, cv::Scalar Color, int Size, std::string Name = std::string("points"))
	{
		std::map<std::string, PointCloudInfo>::iterator finder = pointclouds.find(Name);

		if (finder == pointclouds.end())
		{
			PointCloudInfo insert;

			insert.Color = Color;
			insert.size = Size;
			insert.pc = PointCloud;

			pointclouds.insert(std::pair< std::string, PointCloudInfo >(Name, insert));
		}
		else
		{
			finder->second.Color = Color;
			finder->second.size = Size;

			for (int i = 0; i<(int)PointCloud.size(); i++)
				finder->second.pc.push_back(PointCloud[i]);
		}
	}
	void insert(std::vector<std::pair<cv::Point3f, cv::Point3f>>& LineCloud, cv::Scalar Color, int Size, std::string Name = std::string("lines"))
	{
		std::map<std::string, LineCloudInfo>::iterator finder = lineclouds.find(Name);

		if (finder == lineclouds.end())
		{
			LineCloudInfo insert;

			insert.Color = Color;
			insert.size = Size;
			insert.lc = LineCloud;

			lineclouds.insert(std::pair< std::string, LineCloudInfo >(Name, insert));
		}
		else
		{
			finder->second.Color = Color;
			finder->second.size = Size;

			for (int i = 0; i<(int)LineCloud.size(); i++)
				finder->second.lc.push_back(LineCloud[i]);
		}
	}
	void insert(cv::Mat& FramePose, cv::Scalar Color, int Size, std::string Name = std::string("Frame"))
	{
		if (FramePose.cols != 4 || FramePose.rows != 4) return;
		cv::Mat FramePose_clone = FramePose.clone();
		if (FramePose_clone.type() != CV_32F) FramePose_clone.convertTo(FramePose_clone, CV_32FC1);

		float length = 500.0f;
		float r = 3.0f / 4.0f;
		float frameVec[20 + 12] = { 0, 0, 0, 1.0f,
			length , length * r , length , 1.0f,
			length , -length * r , length , 1.0f,
			-length , -length  * r, length , 1.0f,
			-length , length  * r, length , 1.0f,

			100.0f , 0.0f , 0.0f , 1.0f,
			0.0f , 100.0f , 0.0f , 1.0f,
			0.0f , 0.0f , 100.0f , 1.0f };
		cv::Mat frameVecMat_t(8, 4, CV_32FC1, frameVec);
		cv::Mat frameVecMat = frameVecMat_t.t();
		cv::Mat transPose_frameVecMat = FramePose_clone * frameVecMat;

		std::map<std::string, LineCloudInfo>::iterator finder = frames.find(Name);
		if (finder == frames.end())
		{
			frames.insert(std::pair< std::string, LineCloudInfo >(Name, LineCloudInfo()));
			finder = frames.find(Name);
		}

		LineCloudInfo& target = finder->second;

		target.Color = Color;
		target.size = Size;
		target.lc.clear();

		cv::Point3f p[8];
		for (int i = 0; i < 8; i++)
		{
			p[i].x = transPose_frameVecMat.at<float>(0, i);
			p[i].y = transPose_frameVecMat.at<float>(1, i);
			p[i].z = transPose_frameVecMat.at<float>(2, i);
		}

		target.lc.push_back(std::pair<cv::Point3f, cv::Point3f>(p[0], p[1]));
		target.lc.push_back(std::pair<cv::Point3f, cv::Point3f>(p[0], p[2]));
		target.lc.push_back(std::pair<cv::Point3f, cv::Point3f>(p[0], p[3]));
		target.lc.push_back(std::pair<cv::Point3f, cv::Point3f>(p[0], p[4]));

		target.lc.push_back(std::pair<cv::Point3f, cv::Point3f>(p[1], p[2]));
		target.lc.push_back(std::pair<cv::Point3f, cv::Point3f>(p[2], p[3]));
		target.lc.push_back(std::pair<cv::Point3f, cv::Point3f>(p[3], p[4]));
		target.lc.push_back(std::pair<cv::Point3f, cv::Point3f>(p[4], p[1]));

		target.lc.push_back(std::pair<cv::Point3f, cv::Point3f>(p[0], p[5]));
		target.lc.push_back(std::pair<cv::Point3f, cv::Point3f>(p[0], p[6]));
		target.lc.push_back(std::pair<cv::Point3f, cv::Point3f>(p[0], p[7]));
	}
	void draw(cv::Size plotSize, cv::String WindowName = "Viewer", bool draw_axis = true, bool draw_grid = true)
	{
		if (plot.cols != plotSize.width || plot.rows != plotSize.height)
		{
			plot = cv::Mat(plotSize, CV_8UC3, cv::Scalar(0, 0, 0));
			cx = plot.cols / 2;
			cy = 3 * plot.rows / 4;
		}

		plot.setTo(0);

		projectionUpdate();

		if (draw_grid) drawGrid();

		if (draw_axis) drawAxis();

		drawPoints();

		drawLines();

		drawFrames();

		cv::imshow(WindowName, plot);
		m.setWindow((char*)WindowName.c_str(), 0);
		cv::waitKey(1);
	}
};

class GridMaker
{
private:
	struct GridData
	{
		//0:wait, 1:every iterator
		int get_grid_mode = 1;
		bool mamual_ground_detecte = false;

		double mm2grid = 100.0 / 1000.0;

		int cgridWidth = 500;
		int cgridHeight = 500;
		int cRobotCol = 250;
		int cRobotRow = 350;

		unsigned char gridData[500 * 500];
		unsigned char freeData[500 * 500];
		unsigned char occupyData[500 * 500];
	};
	struct Option
	{
		int sampling_gap = 20;
	};

	Option option;
	IPC* m_pIpc;
	RGBDcamera *m_pData;
	GridData* m_pGridData;

	PointCloudView* view;

	std::vector<RGBD_Extrinsics> cam2ground;
	std::vector<cv::Mat> temp_FreeGrid, temp_ObGrid;
	//std::vector<cv::Mat> DepthIdx2Coordinate;
	cv::Mat Grid, FreeGrid, ObGrid;
public:

	GridMaker::GridMaker(RGBDcamera *pData)
		: m_pData(pData)
	{
		m_pIpc = new IPC("GridData");
		m_pGridData = m_pIpc->connect<GridData>("GridData");

		pData->set_flag_on(Mode::draw2);
	};
	GridMaker::~GridMaker()
	{
		if (m_pIpc) delete m_pIpc;
	}
	void getGrid(unsigned short* input_data, int dev_idx, std::string data_path, int width, int height, RGBD_Parameters *m_pRGBDparam)
	{
		if (Grid.empty())
		{
			Grid = cv::Mat(m_pGridData->cgridHeight, m_pGridData->cgridWidth, CV_8UC1);
			FreeGrid = cv::Mat(m_pGridData->cgridHeight, m_pGridData->cgridWidth, CV_8UC1);
			ObGrid = cv::Mat(m_pGridData->cgridHeight, m_pGridData->cgridWidth, CV_8UC1);
		}

		while (dev_idx > (int)temp_FreeGrid.size() - 1)
		{
			temp_FreeGrid.push_back(cv::Mat(m_pGridData->cgridHeight, m_pGridData->cgridWidth, CV_8UC1));
			temp_ObGrid.push_back(cv::Mat(m_pGridData->cgridHeight, m_pGridData->cgridWidth, CV_8UC1));
			

			float m2mm = 1000.0f;
			cv::Mat c2g_R, c2g_T;
			readCam2GroundRt(data_path, c2g_R, c2g_T);
			if (c2g_R.type() != CV_32FC1) c2g_R.convertTo(c2g_R, CV_32FC1);
			if (c2g_T.type() != CV_32FC1) c2g_T.convertTo(c2g_T, CV_32FC1);
			cv::Mat Cam2GroundRt = cv::Mat::eye(cv::Size(4, 4), CV_32FC1);
			c2g_T *= m2mm;
			c2g_R.copyTo(Cam2GroundRt(cv::Rect(0, 0, 3, 3)));
			c2g_T.copyTo(Cam2GroundRt(cv::Rect(3, 0, 1, 3)));

			cv::Mat c2c_R, c2c_T;
			cv::Mat Cam2Cam = cv::Mat::eye(cv::Size(4, 4), CV_32FC1);
			readExtrinsicParametersYaml(data_path, c2c_R, c2c_T);
			if (c2c_R.type() != CV_32FC1) c2c_R.convertTo(c2c_R, CV_32FC1);
			if (c2c_T.type() != CV_32FC1) c2c_T.convertTo(c2c_T, CV_32FC1);
			c2c_T *= m2mm;
			c2c_R.copyTo(Cam2Cam(cv::Rect(0, 0, 3, 3)));
			c2c_T.copyTo(Cam2Cam(cv::Rect(3, 0, 1, 3)));

			std::cout << Cam2Cam << std::endl;
			std::cout << Cam2GroundRt << std::endl;

			cv::Mat extrin = Cam2GroundRt * Cam2Cam;
			cv::Mat R, T;
			R = extrin(cv::Rect(0, 0, 3, 3)).clone();
			T = extrin(cv::Rect(3, 0, 1, 3)).clone();

			cam2ground.push_back(RGBD_Extrinsics());
			std::memcpy(cam2ground.back().rotation, R.data, sizeof(float) * 9);
			std::memcpy(cam2ground.back().translation, T.data, sizeof(float) * 3);
		}

		temp_FreeGrid[dev_idx].setTo(0);
		temp_ObGrid[dev_idx].setTo(0);

		int size = width * height;
		int sampling_gap = option.sampling_gap;

		short val = 0;
		float real_height, col, row;
		std::vector<cv::Point3f> ground, obstacle;
		for (int i = 0; i < size; i += sampling_gap)
		{
			val = input_data[i];
			if (val < 200 || val > 5000) continue;

			col = i % width;
			row = i / width;

			float2 depth_pixel = { (float)col, (float)row };
			float3 depth_point = m_pRGBDparam[dev_idx].depth_intrinsic.deproject(depth_pixel, (float)val);
			float3 real_point = cam2ground[dev_idx].transform(depth_point);

			int x = m_pGridData->cRobotCol + (int)(m_pGridData->mm2grid * real_point.x);
			int z = m_pGridData->cRobotRow - (int)(m_pGridData->mm2grid * real_point.z);

			real_height = -real_point.y;

			if (real_height < 100.0f)
			{
				if (m_pData->get_flag(Mode::draw2)) 
					ground.push_back(cv::Point3f(real_point.x, real_point.y, real_point.z));

				cv::circle(temp_FreeGrid[dev_idx], cv::Point(x, z), 5, cv::Scalar(255), -1);
			}
			else if (real_height < 2000.0f)
			{
				if (m_pData->get_flag(Mode::draw2)) 
					obstacle.push_back(cv::Point3f(real_point.x, real_point.y, real_point.z));

				cv::circle(temp_ObGrid[dev_idx], cv::Point(x, z), 5, cv::Scalar(255), -1);
			}
		}

		if (m_pData->get_flag(Mode::draw2))
		{
			if (!view) view = new PointCloudView;

			if (dev_idx == 0)
			{
				view->clear("ground");
				view->clear("obstacle");
			}

			view->insert(ground, cv::Scalar(0, 255, 0), 1, "ground");
			view->insert(obstacle, cv::Scalar(0, 0, 255), 1, "obstacle");

			cv::Scalar frameColor(0, 0, 0);
			frameColor.val[dev_idx] = 255;

			cv::Mat RT = cv::Mat::eye(cv::Size(4, 4), CV_32FC1);
			cv::Mat R(3, 3, CV_32FC1, cam2ground[dev_idx].rotation);
			cv::Mat T(3, 1, CV_32FC1, cam2ground[dev_idx].translation);
			R.copyTo(RT(cv::Rect(0, 0, 3, 3)));
			T.copyTo(RT(cv::Rect(3, 0, 1, 3)));

			view->insert(RT, frameColor, 1, cv::format("frame_%d", dev_idx));

			if (dev_idx == (int)temp_FreeGrid.size() - 1)
				view->draw(cv::Size(500, 500), "3D Viewer");
		}
		else
		{
			cv::destroyWindow("3D Viewer");
			if (view)
			{
				delete view;
				view = nullptr;
			}
		}
	};
	void updateGrid(void)
	{
		FreeGrid.setTo(0);
		ObGrid.setTo(0);

		for (int i = 0; i < (int)temp_FreeGrid.size(); i++)
		{
			if (temp_FreeGrid[i].empty()) continue;

			FreeGrid = cv::max(temp_FreeGrid[i], FreeGrid);
			ObGrid = cv::max(temp_ObGrid[i], ObGrid);
		}

		Grid.setTo(125);
		Grid += FreeGrid;
		Grid -= ObGrid;
	}
};

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

	GridMaker* getGrid;

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

#endif