#pragma once

#include <iostream>
#include "realsense\rs.hpp"
#include "opencv2\opencv.hpp"

#define MAX_SENSOR_NUMBER 3


class MouseInterface
{
private:
	int number_of_mouse;

public:
	MouseInterface::MouseInterface(int maxMouse_num);
	MouseInterface::~MouseInterface();

	struct mouse_info
	{
		int x = 0, y = 0, click = 0;
		int change_event;


	};
	mouse_info* Mouse;
	void setWindow(char* name, int mouse_idx);
};
MouseInterface::MouseInterface(int maxMouse_num = 1)
{
	number_of_mouse = maxMouse_num;
	Mouse = new mouse_info[maxMouse_num];
}
MouseInterface::~MouseInterface()
{
	delete[] Mouse;
}
void onMouse(int event, int x, int y, int, void* param)
{
	MouseInterface::mouse_info *input_mouse = (MouseInterface::mouse_info *)param;

	input_mouse->x = x;
	input_mouse->y = y;
	input_mouse->click = event;

	if (event != 0 && (input_mouse->change_event != input_mouse->click))
		input_mouse->change_event = event;
};
void MouseInterface::setWindow(char* name, int mouse_idx)
{
	if (mouse_idx<0 || mouse_idx>this->number_of_mouse - 1)
	{
		printf("There is no %d mouse\n", mouse_idx);
		return;
	}

	cv::setMouseCallback(name, onMouse, &this->Mouse[mouse_idx]);
}

class GroundDetector
{
private:
	struct Box_Info
	{
		int state = 0;

		cv::Point start, end;
		cv::Rect box;
	};
	Box_Info Box;

	MouseInterface mouse;

	cv::Mat Depth_Img;
	bool readPram(void);

	cv::Mat ground_rotation_mat = cv::Mat::eye(cv::Size(3, 3), CV_64FC1);
	cv::Mat ground_translation_mat = cv::Mat(3, 1, CV_64FC1, cv::Scalar(0));
	cv::Mat grondRT = cv::Mat(4, 4, CV_64FC1, cv::Scalar(0));
	cv::Mat configRT = cv::Mat::eye(cv::Size(4, 4), CV_64FC1);

public:
	struct Depth_info
	{
		cv::Mat depth_img;
		cv::Mat depth_3dPoints;
	};
	struct Output
	{
		double meanval[3];
		double normal[3];
		double Configuration[3] = {};
		double Ground_Height = 300;
		double Ceiling_height = 2000;

		cv::Mat RT = cv::Mat(4, 4, CV_64FC1, cv::Scalar(0));
	};
	Depth_info input;
	Output output;

	GroundDetector::GroundDetector();
	GroundDetector::~GroundDetector();

	bool isrunning = 0;
	std::string Ground_file_name;

	void iniGroundPram(void);
	void getGroundPram(cv::Rect &range);
	void run();
};
GroundDetector::GroundDetector()
{
	//Ground_file_name = "..\\data\\Kinect2_Ground_nomal_vector.txt";

	output.normal[0] = 0.0;
	output.normal[1] = 1.0;
	output.normal[2] = 0.0;

	iniGroundPram();
}
GroundDetector::~GroundDetector()
{

}
bool GroundDetector::readPram(void)
{
	if (Ground_file_name.empty()) return 0;

	std::ifstream MyFile;
	MyFile.open(Ground_file_name, std::ios::in);
	if (!MyFile)
	{
		printf("Fail to read %s.\n", Ground_file_name.c_str());
		return 0;
	}


	double temp_config[3];

	MyFile >> output.meanval[0] >> output.meanval[1] >> output.meanval[2]
		>> output.normal[0] >> output.normal[1] >> output.normal[2]
		>> temp_config[0] >> temp_config[1] >> temp_config[2]
		>> output.Ground_Height >> output.Ceiling_height;
	MyFile.close();

	bool isUpdataed = false;
	if (temp_config[0] != output.Configuration[0]) isUpdataed = true;
	if (temp_config[1] != output.Configuration[1]) isUpdataed = true;
	if (temp_config[2] != output.Configuration[2]) isUpdataed = true;

	if (isUpdataed)
	{
		std::memcpy(output.Configuration, temp_config, sizeof(double) * 3);

		configRT = cv::Mat::eye(cv::Size(4, 4), CV_64FC1);

		cv::Mat temp_rot = cv::getRotationMatrix2D(cv::Point(0, 0), output.Configuration[2], 1.0);
		configRT.at<double>(0, 0) = temp_rot.at<double>(0, 0);
		configRT.at<double>(0, 2) = temp_rot.at<double>(0, 1);
		configRT.at<double>(2, 0) = temp_rot.at<double>(1, 0);
		configRT.at<double>(2, 2) = temp_rot.at<double>(1, 1);
		configRT.at<double>(0, 3) = output.Configuration[0];
		configRT.at<double>(2, 3) = output.Configuration[1];

		output.RT = configRT * grondRT;
	}

	return 1;
}
void GroundDetector::iniGroundPram(void)
{
	if (!readPram()) return;

	ground_rotation_mat = cv::Mat(3, 3, CV_64FC1, cv::Scalar(0));
	ground_translation_mat = cv::Mat(3, 1, CV_64FC1, cv::Scalar(0));
	grondRT = cv::Mat::eye(cv::Size(4, 4), CV_64FC1);

	double temp_z_val[3] = { 0.0, 0.0, 1.0 };
	cv::Mat temp_Zaixe(3, 1, CV_64FC1, temp_z_val);
	cv::Mat Ground_norm(3, 1, CV_64FC1, output.normal);
	cv::Mat Xaixe = temp_Zaixe.cross(Ground_norm);
	cv::Mat Zaixe = Ground_norm.cross(Xaixe);

	cv::transpose(Xaixe, ground_rotation_mat.row(0));
	cv::transpose(Ground_norm, ground_rotation_mat.row(1));
	cv::transpose(Zaixe, ground_rotation_mat.row(2));

	cv::Mat temp_ground_Trans = cv::Mat(3, 1, CV_64FC1);
	std::memcpy(temp_ground_Trans.data, output.meanval, sizeof(double) * 3);
	ground_translation_mat = ground_rotation_mat * temp_ground_Trans;
	ground_translation_mat.at<double>(0, 0) = 0.0;
	ground_translation_mat.at<double>(1, 0) = -ground_translation_mat.at<double>(1, 0);
	ground_translation_mat.at<double>(2, 0) = 0.0;

	ground_rotation_mat.copyTo(grondRT(cv::Rect(0, 0, 3, 3)));
	ground_translation_mat.copyTo(grondRT(cv::Rect(3, 0, 1, 3)));

	//std::string temp_name = Ground_file_name + ".txt";
	//std::ofstream file(temp_name, std::ios::out);
	//if (file.is_open())
	//{
	//	file << grondRT.at<double>(0, 0) << " " << grondRT.at<double>(0, 1) << " " << grondRT.at<double>(0, 2) << " " << grondRT.at<double>(0, 3) << std::endl;
	//	file << grondRT.at<double>(1, 0) << " " << grondRT.at<double>(1, 1) << " " << grondRT.at<double>(1, 2) << " " << grondRT.at<double>(1, 3) << std::endl;
	//	file << grondRT.at<double>(2, 0) << " " << grondRT.at<double>(2, 1) << " " << grondRT.at<double>(2, 2) << " " << grondRT.at<double>(2, 3) << std::endl;
	//	file << grondRT.at<double>(3, 0) << " " << grondRT.at<double>(3, 1) << " " << grondRT.at<double>(3, 2) << " " << grondRT.at<double>(3, 3) << std::endl;
	//}
	//file.close();

	output.RT = configRT * grondRT;
}
void GroundDetector::getGroundPram(cv::Rect &range)
{
	int points_counter = 0;
	cv::Mat temp_Point_mat(range.width*range.height, 3, CV_64FC1);
	for (int col_i = 0; col_i < range.width; col_i++)
		for (int row_i = 0; row_i < range.height; row_i++)
		{
			int idx = input.depth_img.cols * (range.y + row_i) + (range.x + col_i);

			if (!std::isfinite(input.depth_3dPoints.at<float>(idx, 0))
				|| !std::isfinite(input.depth_3dPoints.at<float>(idx, 1))
				|| !std::isfinite(input.depth_3dPoints.at<float>(idx, 2)))
				continue;

			if (input.depth_3dPoints.at<float>(idx, 0) == 0
				&& input.depth_3dPoints.at<float>(idx, 1) == 0
				&& input.depth_3dPoints.at<float>(idx, 2) == 0)
				continue;

			temp_Point_mat.at<double>(points_counter, 0) = (double)input.depth_3dPoints.at<float>(idx, 0);
			temp_Point_mat.at<double>(points_counter, 1) = (double)input.depth_3dPoints.at<float>(idx, 1);
			temp_Point_mat.at<double>(points_counter, 2) = (double)input.depth_3dPoints.at<float>(idx, 2);
			points_counter++;
		}

	if (points_counter < 100)
	{
		std::printf("There is no enough depth points\n");
		return;
	}

	cv::Mat Point_mat(points_counter, 3, CV_64FC1);
	temp_Point_mat(cv::Rect(0, 0, 3, points_counter)).copyTo(Point_mat);

	cv::Mat mean;
	cv::PCA pca(Point_mat, mean, CV_PCA_DATA_AS_ROW);
	cv::Vec3d nrm = pca.eigenvectors.row(2);
	nrm = nrm / norm(nrm);
	if (nrm[2]>0)
	{
		nrm[0] = -nrm[0];
		nrm[1] = -nrm[1];
		nrm[2] = -nrm[2];
	}

	cv::Vec3d x0 = pca.mean;

	std::ofstream file(Ground_file_name, std::ios::out);
	if (file.is_open())
	{
		file << x0[0] << " " << x0[1] << " " << x0[2] << " " << nrm[0] << " " << nrm[1] << " " << nrm[2] << std::endl;
		file << output.Configuration[0] << " " << output.Configuration[1] << " " << output.Configuration[2] << std::endl;
		file << output.Ground_Height << " " << output.Ceiling_height;

		file << std::endl << std::endl;
		file << "// ground vector information" << std::endl;
		file << "// x, y, th configuration(mm)" << std::endl;
		file << "// ground height, ceiling height (mm)" << std::endl;
	}

	file.close();

	output.meanval[0] = x0[0];
	output.meanval[1] = x0[1];
	output.meanval[2] = x0[2];
	output.normal[0] = nrm[0];
	output.normal[1] = nrm[1];
	output.normal[2] = nrm[2];

	iniGroundPram();
}
void GroundDetector::run()
{
	if (!isrunning)
	{
		readPram();
		return;
	}

	char window_name[32];
	sprintf_s(window_name, "Ground Detector");

	if (Box.state == 0)
	{
		if (mouse.Mouse->change_event == 1)
		{
			Box.state = 1;
			Box.start = cv::Point(mouse.Mouse->x, mouse.Mouse->y);
		}
	}
	if (Box.state == 1)
	{
		int Rect_start_x = std::min(Box.start.x, mouse.Mouse->x);
		int Rect_start_y = std::min(Box.start.y, mouse.Mouse->y);
		int Rect_size_x = std::abs(Box.start.x - mouse.Mouse->x);
		int Rect_size_y = std::abs(Box.start.y - mouse.Mouse->y);
		Box.box = cv::Rect(Rect_start_x, Rect_start_y, Rect_size_x, Rect_size_y);
		cv::rectangle(input.depth_img, Box.box, cv::Scalar(255));
	}

	cv::imshow(window_name, input.depth_img);
	cv::waitKey(10);
	mouse.setWindow(window_name, 0);

	if (Box.state == 1 && mouse.Mouse->change_event == 4)
	{
		bool box_info = 1;
		if (Box.box.x < 0 || Box.box.y < 0) box_info = 0;
		if (Box.box.x + Box.box.width > input.depth_img.cols) box_info = 0;
		if (Box.box.y + Box.box.height > input.depth_img.rows) box_info = 0;
		if (!box_info)
		{
			Box.state = 0;
			return;
		}

		Box.state = 0;
		getGroundPram(Box.box);
		cv::destroyWindow(window_name);
		isrunning = 0;
	}
}

class RealSense
{
private:
	struct Option
	{
		bool draw_color = false;
		bool draw_grid = false;

		int sampling_gap = 20;
	};

	GroundDetector* gd = nullptr;
	rs::intrinsics depth_intrin[MAX_SENSOR_NUMBER], color_intrin[MAX_SENSOR_NUMBER];
	cv::Mat temp_FreeGrid[MAX_SENSOR_NUMBER], temp_ObGrid[MAX_SENSOR_NUMBER];
	rs::extrinsics depth_to_color[MAX_SENSOR_NUMBER];
	float fScale[MAX_SENSOR_NUMBER];
	cv::Mat Grid, FreeGrid, ObGrid;

	int initialize(void);
	void depth2world(int x, int y, float depthValue, float *x_, float *y_, float *z_, int idx);
	void projection(float *x_, float *y_, float *z_, cv::Mat &_RT);
	void GroundFlagControl(void);
	bool deapthFilter(unsigned short *stream, int &idx, int &row, int &col, int &dev_idx);
	void getGrid(int dev_idx);
	void getColor(int dev_idx);
	void getDepth(int dev_idx);

public:
	RealSense::RealSense(RGBDcamera *_data, IPC *ipc_import);
	RealSense::~RealSense();

	rs::context ctx;

	struct Output
	{
		cv::Mat RT[MAX_SENSOR_NUMBER];
		cv::Mat K[MAX_SENSOR_NUMBER];
		float coeffs[MAX_SENSOR_NUMBER][5];

		cv::Mat groundMask[MAX_SENSOR_NUMBER];

		bool generateGroundMask = true;
	};

	RGBDcamera *data = nullptr;
	IPC *m_ipc = nullptr;
	Option option;
	Output output;

	int getGround = 0;
	int getData(void);
	void threadRun(void);
};
RealSense::RealSense(RGBDcamera *_data, IPC *ipc_import)
{
	data = _data;
	m_ipc = ipc_import;

	initialize();
}
RealSense::~RealSense()
{

}
int RealSense::initialize(void)
{
	rs_error * e = 0;
	rs::log_to_console(rs::log_severity::warn);
	//rs::log_to_file(rs::log_severity::debug, "librealsense.log");

	int SensorCount = ctx.get_device_count();

	data->num_of_senseor = SensorCount;

	if (SensorCount == 0)
	{
		throw new std::exception("No device detected. Is it plugged in?");
		return 0;
	}

	std::printf("%d Sensors are detected\n", ctx.get_device_count());
	for (int i = 0; i < SensorCount; i++)
	{
		/*ctx.get_device(i)->enable_stream(rs::stream::depth, rs::preset::best_quality);
		ctx.get_device(i)->enable_stream(rs::stream::color, rs::preset::best_quality);*/
		ctx.get_device(i)->set_option(rs::option::color_enable_auto_white_balance, 0);
		//ctx.get_device(i)->set_option(rs::option::color_white_balance, 0.5);
		ctx.get_device(i)->enable_stream(rs::stream::depth, rs::preset::highest_framerate);
		ctx.get_device(i)->enable_stream(rs::stream::color, rs::preset::highest_framerate);
		ctx.get_device(i)->enable_stream(rs::stream::infrared2, 0, 0, rs::format::y16, 60);
		ctx.get_device(i)->enable_stream(rs::stream::infrared, 0, 0, rs::format::y16, 60);
		ctx.get_device(i)->start();

		//rs::float2 depth_pixel = { (float)dx, (float)dy };
		//rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
		//rs::float3 color_point = depth_to_color.transform(depth_point);
		//rs::float2 color_pixel = color_intrin.project(color_point);

		//// Use the color from the nearest color pixel, or pure white if this point falls outside the color image
		//const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y);
		//if (cx < 0 || cy < 0 || cx >= color_intrin.width || cy >= color_intrin.height)
		//{
		//	glColor3ub(255, 255, 255);
		//}
		//else
		//{
		//	glColor3ubv(color_image + (cy * color_intrin.width + cx) * 3);
		//}
		depth_intrin[i] = ctx.get_device(i)->get_stream_intrinsics(rs::stream::depth);
		color_intrin[i] = ctx.get_device(i)->get_stream_intrinsics(rs::stream::color);

		output.K[i] = cv::Mat::eye(cv::Size(3, 3), CV_64FC1);
		output.K[i].at<double>(0, 0) = color_intrin[i].fx;
		output.K[i].at<double>(0, 2) = color_intrin[i].ppx;
		output.K[i].at<double>(1, 1) = color_intrin[i].fy;
		output.K[i].at<double>(1, 2) = color_intrin[i].ppy;

		std::memcpy(output.coeffs[i], color_intrin[i].coeffs, sizeof(float) * 5);

		//////////////////////////////////// bg
		data->colorWidth = color_intrin[i].width;
		data->colorHeight = color_intrin[i].height;
		data->colorK[i][0] = color_intrin[i].fx;
		data->colorK[i][1] = 0.f;
		data->colorK[i][2] = color_intrin[i].ppx;
		data->colorK[i][3] = 0.f;
		data->colorK[i][4] = color_intrin[i].fy;
		data->colorK[i][5] = color_intrin[i].ppy;
		data->colorK[i][6] = 0.f;
		data->colorK[i][7] = 0.f;
		data->colorK[i][8] = 1.f;
		std::memcpy(data->colorCoeffs[i], color_intrin[i].coeffs, sizeof(float) * 5);

		data->depthWidth = depth_intrin[i].width;
		data->depthHeight = depth_intrin[i].height;
		data->depthK[i][0] = depth_intrin[i].fx;
		data->depthK[i][1] = 0.f;
		data->depthK[i][2] = depth_intrin[i].ppx;
		data->depthK[i][3] = 0.f;
		data->depthK[i][4] = depth_intrin[i].fy;
		data->depthK[i][5] = depth_intrin[i].ppy;
		data->depthK[i][6] = 0.f;
		data->depthK[i][7] = 0.f;
		data->depthK[i][8] = 1.f;

		for (int x = 0; x < 9; x++) {
			std::cout << data->depthK[i][x] << ", ";
			if ((x + 1) % 3 == 0)
				std::cout << std::endl;
		}

		std::memcpy(data->depthCoeffs[i], depth_intrin[i].coeffs, sizeof(float) * 5);

		depth_to_color[i] = ctx.get_device(i)->get_extrinsics(rs::stream::depth, rs::stream::color);
		memcpy(data->depth_to_color_R[i], depth_to_color[i].rotation, sizeof(float) * 9);
		memcpy(data->depth_to_color_tvec[i], depth_to_color[i].translation, sizeof(float) * 3);
		std::cout << "Rotation" << std::endl;
		for (int x = 0; x < 9; x++) {
			std::cout << data->depth_to_color_R[i][x] << ", ";
			if ((x + 1) % 3 == 0)
				std::cout << std::endl;
		}
		std::cout << "tvec" << std::endl;
		for (int x = 0; x < 3; x++)
			std::cout << data->depth_to_color_tvec[i][x] << std::endl;
		fScale[i] = ctx.get_device(i)->get_depth_scale();
		//memcpy(&data->depthScale[i], &fScale[i], sizeof(float));

		//////////////////////////////////////

		if (output.generateGroundMask)
			output.groundMask[i] = cv::Mat(480, 640, CV_8UC1, cv::Scalar(0));

		printf("Sensor[%d] is done\n", i);
	}

	//gd = new GroundDetector[ctx.get_device_count()];
	//for (int i = 0; i < SensorCount; i++)
	//{
	//	gd[i].Ground_file_name = "..\\data\\RealSense(" + std::to_string(i) + ")_config.txt";
	//	gd[i].iniGroundPram();

	//	output.RT[i] = gd[i].output.RT;
	//	//std::memcpy(data->RT[i], gd[i].output.RT.data, sizeof(double) * 16);
	//}


	return 1;
}
int RealSense::getData(void)
{
	bool stream_is_updated = false;

	for (int i = 0; i < ctx.get_device_count(); i++)
	{
		if (!ctx.get_device(i)->poll_for_frames())	continue;

		stream_is_updated = true;
		getColor(i);
		getDepth(i);
		//if (data->get_grid) getGrid(i);
		//if (data->get_color) getColor(i);
		//if (data->get_depth) getDepth(i);
	}

	/*if (data->get_grid && stream_is_updated)
	{
		FreeGrid.setTo(0);
		ObGrid.setTo(0);

		for (int i = 0; i < ctx.get_device_count(); i++)
		{
			if (temp_FreeGrid[i].empty()) continue;

			FreeGrid = cv::max(temp_FreeGrid[i], FreeGrid);
			ObGrid = cv::max(temp_ObGrid[i], ObGrid);
		}

		Grid.setTo(125);
		Grid += FreeGrid;
		Grid -= ObGrid;
	}*/

	return stream_is_updated;
}
void RealSense::depth2world(int x, int y, float depthValue, float *x_, float *y_, float *z_, int idx)
{
	*x_ = ((float)x - depth_intrin[idx].ppx) * depthValue / depth_intrin[idx].fx;
	*y_ = ((float)y - depth_intrin[idx].ppy) * depthValue / depth_intrin[idx].fy;
	*z_ = depthValue;
}
void RealSense::projection(float *x_, float *y_, float *z_, cv::Mat &_RT)
{
	if (_RT.type() != CV_64F)
	{
		printf("_RT is no a double matrix in projection : %d\n", _RT.type());
		return;
	}

	float temp_x = (float)_RT.at<double>(0, 0) * *x_ +
		(float)_RT.at<double>(0, 1) * *y_ +
		(float)_RT.at<double>(0, 2) * *z_ +
		(float)_RT.at<double>(0, 3);

	float temp_y = (float)_RT.at<double>(1, 0) * *x_ +
		(float)_RT.at<double>(1, 1) * *y_ +
		(float)_RT.at<double>(1, 2) * *z_ +
		(float)_RT.at<double>(1, 3);

	float temp_z = (float)_RT.at<double>(2, 0) * *x_ +
		(float)_RT.at<double>(2, 1) * *y_ +
		(float)_RT.at<double>(2, 2) * *z_ +
		(float)_RT.at<double>(2, 3);

	*x_ = temp_x;
	*y_ = temp_y;
	*z_ = temp_z;
}
void RealSense::GroundFlagControl(void)
{
	if (getGround == 0) return;

	if (getGround == 1 && gd[0].isrunning == 0)
	{
		getGround = 2;
		gd[0].isrunning = 1;
	}

	if (getGround == 2 && gd[0].isrunning == 0)
	{
		getGround = 0;

		if (ctx.get_device_count() > 1)
		{
			getGround = 3;
			gd[1].isrunning = 1;
		}
	}

	if (getGround == 3) if (gd[1].isrunning == 0)
	{
		getGround = 0;

		if (ctx.get_device_count() > 2)
		{
			getGround = 4;
			gd[2].isrunning = 1;
		}
	}

	if (getGround == 4) if (gd[2].isrunning == 0)
	{
		getGround = 0;
	}

	//for (int i = 0; i < SensorCount; i++)
	//{
	//	std::memcpy(data->RT[i], gd[i].output.RT.data, sizeof(double) * 16);
	//}
}
bool RealSense::deapthFilter(unsigned short *stream, int &idx, int &row, int &col, int &dev_idx)
{
	int mode = 1;

	if (mode == 1)
	{
		int l = -1;
		int r = 1;
		int u = depth_intrin[dev_idx].width;
		int d = -depth_intrin[dev_idx].width;

		if (row == 0) d = 0;
		if (row == depth_intrin[dev_idx].height - 1) u = 0;
		if (col == 0) l = 0;
		if (col == depth_intrin[dev_idx].width - 1) r = 0;

		if (stream[idx + r] == 0) return false;
		if (stream[idx + l] == 0) return false;
		if (stream[idx + d] == 0) return false;
		if (stream[idx + u] == 0) return false;
	}

	return true;
}
void RealSense::getGrid(int dev_idx)
{
	/*GroundFlagControl();

	if (Grid.empty())
	{
		if (data != nullptr)
		{
			Grid = cv::Mat(data->cgridHeight, data->cgridWidth, CV_8UC1, data->gridData);
			FreeGrid = cv::Mat(data->cgridHeight, data->cgridWidth, CV_8UC1, data->freeData);
			ObGrid = cv::Mat(data->cgridHeight, data->cgridWidth, CV_8UC1, data->occupyData);
		}
		else
		{
			Grid = cv::Mat(data->cgridHeight, data->cgridWidth, CV_8UC1);
			FreeGrid = cv::Mat(data->cgridHeight, data->cgridWidth, CV_8UC1);
			ObGrid = cv::Mat(data->cgridHeight, data->cgridWidth, CV_8UC1);
		}
	}

	if (temp_FreeGrid[dev_idx].empty())
	{
		temp_FreeGrid[dev_idx] = cv::Mat(data->cgridHeight, data->cgridWidth, CV_8UC1);
		temp_ObGrid[dev_idx] = cv::Mat(data->cgridHeight, data->cgridWidth, CV_8UC1);
	}

	temp_FreeGrid[dev_idx].setTo(0);
	temp_ObGrid[dev_idx].setTo(0);

	int size = depth_intrin[dev_idx].height * depth_intrin[dev_idx].width;
	int sampling_gap = option.sampling_gap;

	bool isGettingGround = false;
	if (dev_idx + 2 == getGround)
	{
		isGettingGround = true;
		sampling_gap = 1;
	}

	if (gd[dev_idx].isrunning == 1)
	{
		gd[dev_idx].input.depth_3dPoints = cv::Mat((int)size, 3, CV_32FC1, cv::Scalar(0));
		gd[dev_idx].input.depth_img = cv::Mat(depth_intrin[dev_idx].height, depth_intrin[dev_idx].width, CV_8UC1, cv::Scalar(0));
	}

	if (output.generateGroundMask) output.groundMask[dev_idx].setTo(0);

	double fine_round_height = gd[dev_idx].output.Ground_Height / 3.0;
	for (int i = 0; i < size; i += sampling_gap)
	{
		float val = (float)((unsigned short*)ctx.get_device(dev_idx)->get_frame_data(rs::stream::depth))[i];

		if (val <= 0) continue;

		float point[3] = {}, ori_point[3];
		int col = i % depth_intrin[dev_idx].width;
		int row = i / depth_intrin[dev_idx].width;

		bool valid = deapthFilter((unsigned short*)ctx.get_device(dev_idx)->get_frame_data(rs::stream::depth), i, row, col, dev_idx);
		if (!valid) continue;

		depth2world(col, row, val, &point[0], &point[1], &point[2], dev_idx);

		ori_point[0] = point[0];
		ori_point[1] = point[1];
		ori_point[2] = point[2];

		point[1] = -point[1];

		if (gd[dev_idx].isrunning == 1)
		{
			gd[dev_idx].input.depth_3dPoints.at<float>(i, 0) = point[0];
			gd[dev_idx].input.depth_3dPoints.at<float>(i, 1) = point[1];
			gd[dev_idx].input.depth_3dPoints.at<float>(i, 2) = point[2];
			gd[dev_idx].input.depth_img.at<unsigned char>(row, col) = (int)val % 256;
		}

		if (!isGettingGround)
		{
			projection(&point[0], &point[1], &point[2], gd[dev_idx].output.RT);

			int x = data->cRobotCol - (int)(data->mm2grid * point[0]);
			int z = data->cRobotRow - (int)(data->mm2grid * point[2]);

			if (point[1] < gd[dev_idx].output.Ground_Height)
			{
				cv::circle(temp_FreeGrid[dev_idx], cv::Point(x, z), 5, cv::Scalar(255), -1);

				if (point[1] < fine_round_height && output.generateGroundMask)
				{
					float imgpoint[2];
					rs_project_point_to_pixel(imgpoint, (rs_intrinsics*)&color_intrin[dev_idx], ori_point);
					cv::circle(output.groundMask[dev_idx], cv::Point2f(imgpoint[0], imgpoint[1]), 20, cv::Scalar(1), -1);
				}
			}
			else if (point[1] < gd[dev_idx].output.Ceiling_height)
			{
				cv::circle(temp_ObGrid[dev_idx], cv::Point(x, z), 5, cv::Scalar(255), -1);
			}
		}
	}

	gd[dev_idx].run();*/
}
void RealSense::getColor(int dev_idx)
{
	if (data == nullptr)
	{
		printf("Data is not allocated in getColor\n");
		return;
	}

	int size = color_intrin[dev_idx].height * color_intrin[dev_idx].width * 3;

	cv::Mat BGRimg(color_intrin[dev_idx].height, color_intrin[dev_idx].width, CV_8UC3, (uchar*)ctx.get_device(dev_idx)->get_frame_data(rs::stream::color));
	cv::Mat RGBimg(color_intrin[dev_idx].height, color_intrin[dev_idx].width, CV_8UC3, data->colorData[dev_idx]);
	cv::cvtColor(BGRimg, RGBimg, CV_BGR2RGB);

	if (option.draw_color) cv::imshow(cv::format("RealSense Color(%d)", dev_idx), RGBimg);
	else cv::destroyWindow(cv::format("RealSense Color(%d)", dev_idx));
}
void RealSense::getDepth(int dev_idx)
{
	if (data == nullptr)
	{
		printf("Data is not allocated in getColor\n");
		return;
	}

	int size = depth_intrin[dev_idx].height * depth_intrin[dev_idx].width;
	std::cout << depth_intrin[dev_idx].height << ", " << depth_intrin[dev_idx].width << std::endl;
	cv::Mat depthImg(depth_intrin[dev_idx].height, depth_intrin[dev_idx].width, CV_16UC1, (ushort*)ctx.get_device(dev_idx)->get_frame_data(rs::stream::depth));
	std::memcpy(data->depthData[dev_idx], depthImg.data, sizeof(short) * size);
	//depthImg.convertTo(depthImg, CV_8U, 0.05, -25);

	//
	int height = ctx.get_device(dev_idx)->get_stream_intrinsics(rs::stream::infrared2).height;
	int width = ctx.get_device(dev_idx)->get_stream_intrinsics(rs::stream::infrared2).width;
	std::cout << height << ", " << width << std::endl;
	cv::Mat irImg1(height, width, CV_16S,
		(ushort*)ctx.get_device(dev_idx)->get_frame_data(rs::stream::infrared));
	cv::Mat irImg2(height, width, CV_16S,
		(ushort*)ctx.get_device(dev_idx)->get_frame_data(rs::stream::infrared2));

	//cv::imshow("de", depthImg);
	cv::imshow("ir", irImg1);
	cv::imshow("ir2", irImg2);

	//cv::resize(depthImg, depthImg, cv::Size(640, 480));
	cv::Mat rgb(480, 640, CV_8UC3,
		(uchar*)ctx.get_device(dev_idx)->get_frame_data(rs::stream::color));
	cv::cvtColor(rgb, rgb, CV_RGB2BGR);
	float scale = ctx.get_device(dev_idx)->get_depth_scale();

	rs::intrinsics depth_intrin = ctx.get_device(dev_idx)->get_stream_intrinsics(rs::stream::depth);
	rs::intrinsics color_intrin = ctx.get_device(dev_idx)->get_stream_intrinsics(rs::stream::color);
	rs::extrinsics depth_to_color = ctx.get_device(dev_idx)->get_extrinsics(rs::stream::depth, rs::stream::color);
	cv::Mat newDepth(240, 320, CV_8UC3, cv::Scalar(0, 0, 0));
	cv::Mat Color2DepthMapper(480, 640, CV_16SC2, cv::Scalar(0, 0));
	std::cout << "RS_DISTORTION_MODIFIED_BROWN_CONRADY: 1, " << color_intrin.model() << std::endl;

	for (int y = 0; y < depthImg.rows; y++)
	{
		for (int x = 0; x < depthImg.cols; x++)
		{
			uint16_t depth_val = depthImg.at<ushort>(y, x);
			float depth_in_meters = depth_val * scale;

			if (depth_val == 0) continue;

			rs::float2 depth_pixel = { (float)x, (float)y };
			rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
			rs::float3 color_point = depth_to_color.transform(depth_point);
			rs::float2 color_pixel = color_intrin.project(color_point);

			const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y);
			//Color2DepthMapper.at<cv::Vec2s>(cy, cx)[0] = x;
			//Color2DepthMapper.at<cv::Vec2s>(cy, cx)[1] = y;

			if (cx < 0 || cy < 0 || cx >= color_intrin.width || cy >= color_intrin.height) {

			}
			else
			{
				try
				{
					newDepth.at<cv::Vec3b>(y, x) = /*depth_val * 0.05 **/ rgb.at<cv::Vec3b>(cy, cx);
				}
				catch (const std::exception&)
				{
					std::cout << cx << ", " << cy << std::endl;
				}

			}
		}
	}
	cv::imshow("newDe", newDepth);
	cv::waitKey(10);


	//if (option.draw_color) cv::imshow(cv::format("RealSense Color(%d)", dev_idx), RGBimg);
	//else cv::destroyWindow(cv::format("RealSense Color(%d)", dev_idx));
}
void RealSense::threadRun(void)
{
	//function_cummunication* state = m_ipc->get_state("RealSense.exe");

	while (1)
	{
		getData();

		/*if (option.draw_grid)
		{
			cv::putText(Grid, cv::format("%dms", state->ms), cv::Point(15, 25),
				0, 0.8, cv::Scalar(0), 2);
			cv::imshow("Grid", Grid);
		}
		else cv::destroyWindow("Grid");
		
		if (option.draw_color || option.draw_grid) cv::waitKey(1);
		*/
		//if (GetAsyncKeyState(VK_ESCAPE)) state->trigger = 0;

		if (/*m_ipc->exit()*/0)
		{
			Sleep(30);
			break;
		}
	}
};