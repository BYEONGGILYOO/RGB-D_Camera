#pragma once

#include <direct.h>
#include "opencv2/aruco.hpp"
#include "opencv2/ximgproc.hpp"

//Occlusion time(second)
//#define occlusionTest_time 30
#define occlusionTest_open_time 10
#define MAX_SENSOR_NUMBER 3

class LineObservation
{
private:
	int Gx(cv::Mat & patch, int x, int y);
	int Gy(cv::Mat & patch, int x, int y);
	int bound_check(int x, int y, int w, int h);
	void BackwardWarping(cv::Mat & dst, cv::Mat &p_mat);

	void CalcMSLD(cv::Mat & patch, cv::Mat & msld);
	void GetImagePatch(cv::Mat &_output_patch, cv::Vec4f &seg);

public:

	cv::Mat gray_RGB;

	float getAngle(cv::Vec4f *seg);
	void line_compute(std::vector<cv::Vec4f> &_lines, cv::Mat &_disc);
};
int LineObservation::Gx(cv::Mat &patch, int x, int y)
{
	return (patch.at<unsigned char>(y + 1, x - 1) + 2 * patch.at<unsigned char>(y + 1, x) + patch.at<unsigned char>(y + 1, x + 1))
		- (patch.at<unsigned char>(y - 1, x - 1) + 2 * patch.at<unsigned char>(y - 1, x) + patch.at<unsigned char>(y - 1, x + 1));
}
int LineObservation::Gy(cv::Mat &patch, int x, int y)
{
	return (patch.at<unsigned char>(y - 1, x + 1) + 2 * patch.at<unsigned char>(y, x + 1) +
		patch.at<unsigned char>(y + 1, x + 1)) - (patch.at<unsigned char>(y - 1, x - 1) +
			2 * patch.at<unsigned char>(y, x - 1) + patch.at<unsigned char>(y + 1, x - 1));
}
int LineObservation::bound_check(int x, int y, int w, int h)
{
	if (x<0 || x >= w || y<0 || y >= h) return 0;
	else return 1;
}
void LineObservation::BackwardWarping(cv::Mat & dst, cv::Mat &p_mat)
{
	double w, pixel, ratio, px, py;

	double wx[2];
	double wy[2];

	int i, j, x, y;

	cv::Mat rot_inv = p_mat.inv();

	for (j = 0; j < dst.rows; j++)
	{
		for (i = 0; i < dst.cols; i++)
		{
			ratio = pixel = 0.0;
			w = rot_inv.at<double>(2, 0)*i + rot_inv.at<double>(2, 1)*j + rot_inv.at<double>(2, 2);

			px = rot_inv.at<double>(0, 0)*i + rot_inv.at<double>(0, 1)*j + rot_inv.at<double>(0, 2);
			py = rot_inv.at<double>(1, 0)*i + rot_inv.at<double>(1, 1)*j + rot_inv.at<double>(1, 2);

			wx[1] = px - floor(px);
			wx[0] = 1.0 - wx[1];

			wy[1] = py - floor(py);
			wy[0] = 1.0 - wy[1];

			x = (int)floor(px);
			y = (int)floor(py);

			if (bound_check(x, y, gray_RGB.cols, gray_RGB.rows)) {
				pixel += wx[0] * wy[0] * gray_RGB.at<unsigned char>(y, x);
				ratio += wx[0] * wy[0];
			}
			if (bound_check(x + 1, y, gray_RGB.cols, gray_RGB.rows)) {
				pixel += wx[1] * wy[0] * gray_RGB.at<unsigned char>(y, x + 1);
				ratio += wx[1] * wy[0];
			}
			if (bound_check(x, y + 1, gray_RGB.cols, gray_RGB.rows)) {
				pixel += wx[0] * wy[1] * gray_RGB.at<unsigned char>(y + 1, x);
				ratio += wx[0] * wy[1];
			}
			if (bound_check(x + 1, y + 1, gray_RGB.cols, gray_RGB.rows)) {
				pixel += wx[1] * wy[1] * gray_RGB.at<unsigned char>(y + 1, x + 1);
				ratio += wx[1] * wy[1];
			}
			dst.at<unsigned char>(j, i) = (unsigned char)floor(pixel / ratio + 0.5);
		}
	}
}
float LineObservation::getAngle(cv::Vec4f *seg)
{
	float fDx = (float)(seg->val[2] - seg->val[0]);
	float fDy = (float)(seg->val[3] - seg->val[1]);
	float fTemp = 0.0f;
	double dAngle = 0.0;

	if (fDx == 0.0f) {
		if (fDy > 0)
			dAngle = CV_PI / 2.0;
		else
			dAngle = -1.0 * CV_PI / 2.0;
	}
	else if (fDy == 0.0f) {
		if (fDx > 0)
			dAngle = 0.0;
		else
			dAngle = CV_PI;
	}
	else if (fDx < 0.0f && fDy > 0.0f)
		dAngle = CV_PI + atan(fDy / fDx);
	else if (fDx > 0.0f && fDy < 0.0f)
		dAngle = 2 * CV_PI + atan(fDy / fDx);
	else if (fDx < 0.0f && fDy < 0.0f)
		dAngle = CV_PI + atan(fDy / fDx);
	else
		dAngle = atan(fDy / fDx);

	if (dAngle > 2.0 * CV_PI)
		dAngle -= 2.0 * CV_PI;

	return (float)dAngle;
}
void LineObservation::CalcMSLD(cv::Mat & patch, cv::Mat & msld)
{
	cv::Mat GDM = cv::Mat::zeros(9, patch.cols - 6, CV_64FC4);
	cv::Mat G = cv::Mat::zeros(5 * 9, patch.cols - 2, CV_64FC2); // 위아래 그래이 값 차이를 가우스 분산모양의 w 를 주어 그 위치에 저장
																 //	cout << GDM.rows << " " << GDM.cols << endl;
	double sigma = 22.5f;
	double d_sq_sigma = 2.0*(sigma*sigma);
	double gaussian_n = 1.0 / (sqrt(2.0*CV_PI)*sigma);
	for (int i = 1; i<patch.rows - 1; i++)
	{
		for (int j = 1; j<patch.cols - 1; j++)
		{
			double gx = (double)Gx(patch, j, i); // j,i 포인트 기준 위아래 그래이값의 차이
			double gy = (double)Gy(patch, j, i); // j,i 포인트 기준 좌우 그래이값의 차이
			int d = abs(i - 23); //패치에서 선이 위치하는 중앙

			double w = gaussian_n * exp(-1.0*(double)(d*d) / d_sq_sigma);

			gx = w*gx;
			gy = w*gy;

			G.at<cv::Vec2d>(i - 1, j - 1)[0] = gx;
			G.at<cv::Vec2d>(i - 1, j - 1)[1] = gy;
		}
	}


	for (int i = 0; i<GDM.rows; i++)
	{
		for (int j = 0; j<GDM.cols; j++)
		{
			double gx = 0.0, gy = 0.0;
			double w1 = 0.0, w2 = 0.0;
			int d1 = 0, d2 = 0;

			for (int k = 0; k<5; k++)
			{
				d1 = abs(2 - k);
				d2 = (3 + k) % 5;
				w1 = (double)d2 / (double)(d1 + d2);
				w2 = (double)d1 / (double)(d1 + d2);

				for (int l = 0; l<5; l++)
				{
					gx = G.at<cv::Vec2d>(k + 5 * i, l + j)[0];
					gy = G.at<cv::Vec2d>(k + 5 * i, l + j)[1];

					if (k <= 1 && gy >= 0.0 && i != 0)
					{
						GDM.at<cv::Vec4d>(i - 1, j)[0] += w2*gy;
						GDM.at<cv::Vec4d>(i, j)[0] += w1*gy;
					}
					else if (k <= 1 && gy<0.0 && i != 0)
					{
						GDM.at<cv::Vec4d>(i - 1, j)[1] += -w2*gy;
						GDM.at<cv::Vec4d>(i, j)[1] += -w1*gy;
					}

					if (k <= 1 && gx >= 0.0 && i != 0)
					{
						GDM.at<cv::Vec4d>(i - 1, j)[2] += w2*gx;
						GDM.at<cv::Vec4d>(i, j)[2] += w1*gx;
					}
					else if (k <= 1 && gx<0.0 && i != 0)
					{
						GDM.at<cv::Vec4d>(i - 1, j)[3] += -w2*gx;
						GDM.at<cv::Vec4d>(i, j)[3] += -w1*gx;
					}

					if (k == 2 && gy >= 0.0) {
						GDM.at<cv::Vec4d>(i, j)[0] += gy;
					}
					else if (k == 2 && gy<0.0) {
						GDM.at<cv::Vec4d>(i, j)[1] += -gy;
					}
					if (k == 2 && gx >= 0.0) {
						GDM.at<cv::Vec4d>(i, j)[2] += gx;
					}
					else if (k == 2 && gx<0.0) {
						GDM.at<cv::Vec4d>(i, j)[3] += -gx;
					}

					if (k >= 3 && gy >= 0.0 && i != GDM.rows - 1) {
						GDM.at<cv::Vec4d>(i + 1, j)[0] += w2*gy;
						GDM.at<cv::Vec4d>(i, j)[0] += w1*gy;
					}
					else if (k >= 3 && gy<0.0 && i != GDM.rows - 1) {
						GDM.at<cv::Vec4d>(i + 1, j)[1] += -w2*gy;
						GDM.at<cv::Vec4d>(i, j)[1] += -w1*gy;
					}

					if (k >= 3 && gx >= 0.0 && i != GDM.rows - 1)
					{
						GDM.at<cv::Vec4d>(i + 1, j)[2] += w2*gx;
						GDM.at<cv::Vec4d>(i, j)[2] += w1*gx;
					}
					else if (k >= 3 && gx<0.0 && i != GDM.rows - 1)
					{
						GDM.at<cv::Vec4d>(i + 1, j)[3] += -w2*gx;
						GDM.at<cv::Vec4d>(i, j)[3] += -w1*gx;
					}
					//else if(k>=3 && gx>=0.0 && i==GDM.rows-1)
					//{
					//	GDM.at<Vec4d>(i,j)[2] += gx;
					//}
					//else if(k>=3 && gx<0.0 && i==GDM.rows-1)
					//{
					//	GDM.at<Vec4d>(i,j)[3] += -gx;
					//}
				}
			}
			//cout << iv1 << "\t" << iv2 << "\t" << iv3 << "\t" << iv4 << endl;
			//GDM.at<Vec4d>(i,j)[0] += iv1;
			//GDM.at<Vec4d>(i,j)[1] += iv2;
			//GDM.at<Vec4d>(i,j)[2] += iv3;
			//GDM.at<Vec4d>(i,j)[3] += iv4;
		}
	}
	//	print_matrix(GDM);

	cv::Mat mean = cv::Mat::zeros(4 * 9, 1, CV_64FC1);
	cv::Mat stddev = cv::Mat::zeros(4 * 9, 1, CV_64FC1);
	double mag_mean = 0.0, mag_stddev = 0.0;

	for (int i = 0; i<GDM.cols; i++) {
		for (int j = 0; j<GDM.rows; j++) {
			for (int k = 0; k<4; k++) {
				mean.at<double>(j * 4 + k, 0) += GDM.at<cv::Vec4d>(j, i)[k];
			}
		}
	}
	for (int i = 0; i<mean.rows; i++) {
		mean.at<double>(i, 0) /= (double)GDM.cols;
		mag_mean += mean.at<double>(i, 0)*mean.at<double>(i, 0);
	}
	mag_mean = sqrt(mag_mean);
	//	cout << "mag_mean: " << mag_mean << endl;

	for (int i = 0; i<GDM.cols; i++) {
		for (int j = 0; j<GDM.rows; j++) {
			for (int k = 0; k<4; k++) {
				stddev.at<double>(j * 4 + k, 0) +=
					(GDM.at<cv::Vec4d>(j, i)[k] - mean.at<double>(j * 4 + k, 0))*
					(GDM.at<cv::Vec4d>(j, i)[k] - mean.at<double>(j * 4 + k, 0));
			}
		}
	}

	for (int i = 0; i<stddev.rows; i++) {
		stddev.at<double>(i, 0) /= (double)GDM.cols;
		stddev.at<double>(i, 0) = sqrt(stddev.at<double>(i, 0));
		mag_stddev += stddev.at<double>(i, 0)*stddev.at<double>(i, 0);
	}
	mag_stddev = sqrt(mag_stddev);
	//	cout << "mag_stddev: " << mag_stddev << endl;
	//	print_matrix(stddev);

	for (int i = 0; i<8 * 9; i++)
	{
		if (i<8 * 9 / 2)
			msld.at<float>(0, i) = (float)(mean.at<double>(i, 0) / mag_mean) / 1.414213562f;
		else
			msld.at<float>(0, i) = (float)(stddev.at<double>(i - 8 * 9 / 2, 0) / mag_stddev) / 1.414213562f;
	}
	//	print_matrix(stddev);
	//	print_matrix(msld);
}
void LineObservation::GetImagePatch(cv::Mat &_output_patch, cv::Vec4f &seg)
{
	cv::Mat rot = cv::Mat::zeros(3, 3, CV_64FC1);

	double offset_x = 0.0;
	double offset_y = 0.0;

	cv::Mat l = cv::Mat::zeros(3, 1, CV_64FC1);
	l.at<double>(0, 0) = (double)(seg.val[0] - seg.val[2]);
	l.at<double>(1, 0) = (double)(seg.val[1] - seg.val[3]);
	l.at<double>(2, 0) = 1.0;


	double s = sqrt(l.at<double>(0, 0) * l.at<double>(0, 0) + l.at<double>(1, 0) *
		l.at<double>(1, 0));

	cv::Mat ln = cv::Mat::zeros(3, 1, CV_64FC1);
	ln.at<double>(0, 0) = l.at<double>(0, 0) / s;
	ln.at<double>(1, 0) = l.at<double>(1, 0) / s;
	ln.at<double>(2, 0) = 1.0;


	cv::Mat n = cv::Mat::zeros(3, 1, CV_64FC1);
	n.at<double>(0, 0) = (double)(cos(90.0*CV_PI / 180.0)*l.at<double>(0, 0) -
		sin(90.0*CV_PI / 180.0)*l.at<double>(1, 0));
	n.at<double>(1, 0) =
		(double)(sin(90.0*CV_PI / 180.0)*l.at<double>(0, 0) +
			cos(90.0*CV_PI / 180.0)*l.at<double>(1, 0));
	n.at<double>(2, 0) = 1.0;


	s = sqrt(n.at<double>(0, 0)*n.at<double>(0, 0) + n.at<double>(1, 0)
		*n.at<double>(1, 0));

	n.at<double>(0, 0) = n.at<double>(0, 0) / s;
	n.at<double>(1, 0) = n.at<double>(1, 0) / s;


	cv::Mat e1 = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat e2 = cv::Mat::zeros(3, 1, CV_64FC1);
	e1.at<double>(0, 0) = (double)seg.val[0];
	e1.at<double>(1, 0) = (double)seg.val[1];
	e1.at<double>(2, 0) = 0.0;
	e2.at<double>(0, 0) = (double)seg.val[2];
	e2.at<double>(1, 0) = (double)seg.val[3];
	e2.at<double>(2, 0) = 0.0;

	float angle = getAngle(&seg);

	rot.at<double>(0, 0) = cos(-angle);
	rot.at<double>(0, 1) = -sin(-angle);
	rot.at<double>(1, 0) = sin(-angle);
	rot.at<double>(1, 1) = cos(-angle);
	rot.at<double>(2, 2) = 1.0;
	rot.at<double>(0, 2) = 0.0;
	rot.at<double>(1, 2) = 0.0;


	cv::Mat v1 = 23.0*n + e1 - 3.0*ln;
	v1 = rot * v1;
	cv::Mat v2 = -23.0*n + e1 - 3.0*ln;
	v2 = rot * v2;
	cv::Mat v3 = 23.0*n + e2 + 3.0*ln;
	v3 = rot * v3;
	cv::Mat v4 = -23.0*n + e2 + 3.0*ln;
	v4 = rot * v4;

	offset_x = v1.at<double>(0, 0) < v2.at<double>(0, 0) ? v1.at<double>(0, 0) : v2.at<double>(0, 0);
	offset_x = offset_x < v3.at<double>(0, 0) ? offset_x : v3.at<double>(0, 0);
	offset_x = offset_x < v4.at<double>(0, 0) ? offset_x : v4.at<double>(0, 0);

	offset_y = v1.at<double>(1, 0) < v2.at<double>(1, 0) ? v1.at<double>(1, 0) : v2.at<double>(1, 0);
	offset_y = offset_y < v3.at<double>(1, 0) ? offset_y : v3.at<double>(1, 0);
	offset_y = offset_y < v4.at<double>(1, 0) ? offset_y : v4.at<double>(1, 0);

	rot.at<double>(0, 0) = cos(-angle);
	rot.at<double>(0, 1) = -sin(-angle);
	rot.at<double>(1, 0) = sin(-angle);
	rot.at<double>(1, 1) = cos(-angle);
	rot.at<double>(2, 2) = 1.0;
	rot.at<double>(0, 2) = -offset_x;
	rot.at<double>(1, 2) = -offset_y;

	BackwardWarping(_output_patch, rot);
}
void LineObservation::line_compute(std::vector<cv::Vec4f> &_lines, cv::Mat &_disc)
{
	for (int i = 0; i < (int)_lines.size(); i++)
	{
		cv::Vec4f seg = _lines.at(i);
		//cv::Mat msld = cv::Mat::zeros(pram.D, 1, CV_32FC1);
		double length = sqrt((seg.val[0] - seg.val[2])*(seg.val[0] - seg.val[2])
			+ (seg.val[1] - seg.val[3])*(seg.val[1] - seg.val[3]));
		cv::Mat patch = cv::Mat::zeros(45 + 2, cvRound(length) + 6, CV_8UC1);

		GetImagePatch(patch, seg);
		CalcMSLD(patch, _disc.row(i));
	}
}

class MarkerPoseEstimater
{
private:
	struct Option
	{
		bool showMarker = false;
		bool showRejected = false;
		bool refindStrategy = true;

		bool draw_RGB = true;
		bool draw_viewer3D = true;

		int markersX = 3;
		int markersY = 5;
		int markerType = cv::aruco::DICT_6X6_250;
		//float markerLength = 0.066f;
		//float markerSeparation = 0.006;
		float markerLength = 67.5f;
		float markerSeparation = 3.375;
		float axisLength = 1.0f * ((float)std::min(markersX, markersY) * (markerLength + markerSeparation) +
			markerSeparation);
	};

	Option option;

	std::string file_name = "..\\data\\color_2_world_mat_";

	PointCloudView viewer;

	RGBDcamera *m_camera_data = nullptr;
	cv::Ptr<cv::aruco::Dictionary> dictionary;
	cv::Ptr<cv::aruco::GridBoard> gridboard;
	cv::Ptr<cv::aruco::Board> board;
	cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
	std::vector<cv::Mat> RGB, cameraMatrix, distCoeffs;
	int center_camera_idx = -1;

	static bool readDetectorParameters(cv::String filename, cv::Ptr<cv::aruco::DetectorParameters> &params) {
		cv::FileStorage fs(filename, cv::FileStorage::READ);
		if (!fs.isOpened())
			return false;
		fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
		fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
		fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
		fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
		fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
		fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
		fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
		fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
		fs["minDistanceToBorder"] >> params->minDistanceToBorder;
		fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
		//fs["doCornerRefinement"] >> params->doCornerRefinement;
		fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
		fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
		fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
		fs["markerBorderBits"] >> params->markerBorderBits;
		fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
		fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
		fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
		fs["minOtsuStdDev"] >> params->minOtsuStdDev;
		fs["errorCorrectionRate"] >> params->errorCorrectionRate;
		return true;
	}
	cv::Mat transpose2World(cv::Mat& m)
	{
		if (m.cols != 4 || m.rows != 4) return cv::Mat();

		float base_z_axis_data[3] = { 0.0f, 0.0f, 1.0f };
		cv::Mat base_z_axis(3, 1, CV_32FC1, base_z_axis_data);

		cv::Mat result = cv::Mat::eye(cv::Size(4, 4), CV_32FC1);

		result.at<float>(0, 1) = m.at<float>(2, 0);
		result.at<float>(1, 1) = -m.at<float>(2, 1);
		result.at<float>(2, 1) = m.at<float>(2, 2);

		cv::Mat y_axis = result(cv::Rect(1, 0, 1, 3));
		y_axis /= cv::norm(y_axis);

		cv::Mat x_axis = y_axis.cross(base_z_axis);
		x_axis /= cv::norm(x_axis);

		cv::Mat z_axis = x_axis.cross(y_axis);
		z_axis /= cv::norm(z_axis);

		x_axis.copyTo(result(cv::Rect(0, 0, 1, 3)));
		y_axis.copyTo(result(cv::Rect(1, 0, 1, 3)));
		z_axis.copyTo(result(cv::Rect(2, 0, 1, 3)));

		result.at<float>(1, 3) = -m.at<float>(2, 3);

		return result;
	};
	void save(std::vector<cv::Mat>& inputMats)
	{
		for (int i = 0; i < (int)inputMats.size(); i++)
		{
			if (inputMats[i].empty()) continue;

			std::string file = file_name + std::to_string(i) + ".dat";

			std::ofstream MapDBsaver(file, std::ios_base::out | std::ios_base::trunc | std::ios_base::binary);
			if (!MapDBsaver.is_open()) continue;

			MapDBsaver.write((char*)inputMats[i].data, sizeof(float) * 16);

			MapDBsaver.close();
		}
	};
	void load(std::vector<cv::Mat>& outputMats)
	{
		outputMats.clear();
		outputMats.resize(3);

		float buf[16];
		for (int i = 0; i < 3; i++)
		{
			std::string file = file_name + std::to_string(i) + ".dat";
			std::ifstream MapDBloader(file, std::ios_base::in | std::ios_base::binary);
			if (!MapDBloader.is_open()) continue;

			MapDBloader.read((char*)buf, sizeof(float) * 16);

			outputMats[i] = cv::Mat(4, 4, CV_32FC1, buf).clone();

			MapDBloader.close();
		}
	};

public:

	bool running = false;
	std::vector<cv::Mat> Output_color_2_world;

	MarkerPoseEstimater::MarkerPoseEstimater(RGBDcamera *Camera_data)
		: m_camera_data(Camera_data)
	{
		dictionary = cv::aruco::getPredefinedDictionary(option.markerType);

		int markersX = option.markersX;
		int markersY = option.markersY;
		float markerLength = option.markerLength;
		float markerSeparation = option.markerSeparation;

		gridboard = cv::aruco::GridBoard::create(markersX, markersY, markerLength, markerSeparation, dictionary);
		board = gridboard.staticCast<cv::aruco::Board>();
		detectorParams = cv::aruco::DetectorParameters::create();
		bool readOk = readDetectorParameters("..\\data\\marker_detector_params.yml", detectorParams);
		if (!readOk) {
			std::cerr << "Invalid detector parameters file" << std::endl;
		}

		for (int i = 0; i < m_camera_data->num_of_senseor; i++)
		{
			RGB.push_back(cv::Mat(m_camera_data->colorHeight, m_camera_data->colorWidth,
				CV_8UC3, m_camera_data->colorData[i]));

			cameraMatrix.push_back(cv::Mat::eye(cv::Size(3, 3), CV_32FC1));
			cameraMatrix[i].at<float>(0, 0) = m_camera_data->colorK[i][2];
			//cameraMatrix[i].at<float>(0, 2) = 640.0f - m_camera_data->colorK[i][0];
			cameraMatrix[i].at<float>(0, 2) = m_camera_data->colorK[i][0];
			cameraMatrix[i].at<float>(1, 1) = m_camera_data->colorK[i][3];
			//cameraMatrix[i].at<float>(1, 2) = 480.0f - m_camera_data->colorK[i][1];
			cameraMatrix[i].at<float>(1, 2) = m_camera_data->colorK[i][1];

			distCoeffs.push_back(cv::Mat(1, 5, CV_32FC1, m_camera_data->colorCoeffs[i]));

			std::string name(m_camera_data->camera_order[i]);
			if(name == "front") center_camera_idx = i;
		}

		load(Output_color_2_world);
	}

	void run(void)
	{
		if (!running)
		{
			for (int i = 0; i < 3; i++)
				cv::destroyWindow(cv::format("Marker %d", i));

			cv::destroyWindow("Cameara pose");

			return;
		}

		if (!m_camera_data) return;

		bool draw_ori_frames = false;

		bool readOk = readDetectorParameters("..\\data\\marker_detector_params.yml", detectorParams);
		if (!readOk) {
			std::cerr << "Invalid detector parameters file" << std::endl;
		}

		cv::Scalar frameColor[3];
		frameColor[0] = cv::Scalar(0, 0, 255);
		frameColor[1] = cv::Scalar(0, 255, 0);
		frameColor[2] = cv::Scalar(255, 0, 0);

		cv::Mat RGB_plot;
		std::vector<cv::Mat> result_from_marker, result_from_robot;
		for (int i = 0; i < m_camera_data->num_of_senseor; i++)
		{
			RGB_plot = RGB[i].clone();

			std::vector< int > ids;
			std::vector< std::vector< cv::Point2f > > corners, rejected;
			cv::Vec3d rvec, tvec;

			// detect markers
			cv::aruco::detectMarkers(RGB_plot, dictionary, corners, ids, detectorParams, rejected);

			if (option.refindStrategy)
				cv::aruco::refineDetectedMarkers(RGB_plot, board, corners, ids, rejected, cameraMatrix[i], distCoeffs[i]);

			// estimate board pose
			int markersOfBoardDetected = 0;
			bool found = false;
			if (ids.size() > 0)
			{
				markersOfBoardDetected = (int)corners.size();

				std::vector<cv::Point3f> objPoints;
				std::vector<cv::Point2f> imgPoints;

				for (int i = 0; i < (int)corners.size(); i++)
					for (int j = 0; j < (int)corners[i].size(); j++)
						imgPoints.push_back(corners[i][j]);

				for (int i = 0; i < (int)ids.size(); i++)
					for (int j = 0; j < (int)board->objPoints[ids[i]].size(); j++)
						objPoints.push_back(board->objPoints[ids[i]][j]);

				found = cv::solvePnP(objPoints, imgPoints, cameraMatrix[i], distCoeffs[i], rvec, tvec);
				//cv::solvePnPRansac(objPoints, imgPoints, cameraMatrix[i], distCoeffs[i], rvec, tvec);

				//markersOfBoardDetected =
				//	cv::aruco::estimatePoseBoard(corners, ids, board, cameraMatrix[i], distCoeffs[i], rvec, tvec);

				if (found)
				{
					cv::Mat r_mat;
					cv::Rodrigues(rvec, r_mat);

					cv::Mat rt_mat = cv::Mat::eye(cv::Size(4, 4), CV_32FC1);
					if (r_mat.type() != CV_32F) r_mat.convertTo(r_mat, CV_32FC1);
					r_mat.copyTo(rt_mat(cv::Rect(0, 0, 3, 3)));
					rt_mat.at<float>(0, 3) = (float)tvec[0];
					rt_mat.at<float>(1, 3) = (float)tvec[1];
					rt_mat.at<float>(2, 3) = (float)tvec[2];

					cv::Mat result = rt_mat.inv();

					result_from_marker.push_back(result);
				}
			}

			if(!found) result_from_marker.push_back(cv::Mat());

			// draw results
			if (option.showMarker && ids.size() > 0)
				cv::aruco::drawDetectedMarkers(RGB_plot, corners, ids);

			if (option.showRejected && rejected.size() > 0)
				cv::aruco::drawDetectedMarkers(RGB_plot, rejected, cv::noArray(), cv::Scalar(100, 0, 255));

			if (markersOfBoardDetected > 0)
				cv::aruco::drawAxis(RGB_plot, cameraMatrix[i], distCoeffs[i], rvec, tvec, option.axisLength);			

			if(option.draw_viewer3D) if (draw_ori_frames) if(found)
				viewer.insert(result_from_marker.back(), frameColor[i], 1, cv::format("frame_%d", i));

			if(option.draw_RGB) cv::imshow(cv::format("Marker %d", i), RGB_plot);
		}
		 
		if (!result_from_marker[center_camera_idx].empty())
		{
			result_from_robot.resize(3);
			transpose2World(result_from_marker[center_camera_idx]).copyTo(result_from_robot[center_camera_idx]);

			for (int i = 0; i < m_camera_data->num_of_senseor; i++)
			{
				if (i == center_camera_idx) continue;

				cv::Mat side2center = result_from_marker[center_camera_idx].inv() * result_from_marker[i];

				result_from_robot[i] = result_from_robot[center_camera_idx] * side2center;

				if (option.draw_viewer3D) viewer.insert(result_from_robot[i],
					frameColor[i], 1, cv::format("result_frame_%d", i));
			}
			if (option.draw_viewer3D) viewer.insert(result_from_robot[center_camera_idx],
				frameColor[center_camera_idx], 1, cv::format("result_frame_%d", center_camera_idx));

			save(result_from_robot);
		}

		if (option.draw_viewer3D) viewer.draw(cv::Size(500, 500), "Cameara pose", true, false);

		if(option.draw_RGB) cv::waitKey(1);
	}
};

class GroundLine
{
private:
	struct GroundNode
	{
		std::vector<cv::Vec4f> lines;
		std::vector<cv::Vec4f> ground_lines;
		std::vector<cv::Point3d> line_param;

		cv::Mat desc;
		cv::Mat GroundImg;
	};
	struct Input
	{
		////cv::Mat depth_2_robot_R[MAX_SENSOR_NUMBER];
		//cv::Mat depth_2_robot_RT[MAX_SENSOR_NUMBER];
		cv::Mat color_K[MAX_SENSOR_NUMBER];
		//cv::Mat color_K_inv[MAX_SENSOR_NUMBER];
		cv::Mat color_coeffs[MAX_SENSOR_NUMBER];
		//cv::Mat color_2_depth_RT[MAX_SENSOR_NUMBER];
		////cv::Mat color_2_depth_T[MAX_SENSOR_NUMBER];
		//cv::Mat color_2_world_2_depth[MAX_SENSOR_NUMBER];

		//cv::Mat groundMask[MAX_SENSOR_NUMBER];

		cv::Mat allMat[MAX_SENSOR_NUMBER];
	};
	struct Option
	{
		double imgResizeRate = 0.5;
		bool undistortion = false;
		//bool using_Mask = false;
		bool usingColor2DepthRT = true;
		bool usingDepthCheck = true;

		//bool draw_line_Ground = true;
		//bool draw_matching = true;
	};

	std::string dataPath, MapdataName = "GroundNode.dat";

	IPC_v2* m_ipc = nullptr;
	RGBDcamera *m_camera_data = nullptr;
	GridData *m_grid_data = nullptr;
	MotionCalc *m_data = nullptr;

	cv::Mat Ground_line_plot_img, Ground_fit_line_plot;
	PointCloudView view;

	LineObservation lineComputer;
	cv::Ptr<cv::ximgproc::FastLineDetector> ld;
	cv::Ptr<cv::DescriptorMatcher> matcher;
	cv::Mat FreeGrid, ObstacleGrid;
	cv::Mat RGB[MAX_SENSOR_NUMBER], Gray[MAX_SENSOR_NUMBER];
	std::vector<cv::Mat> ground_line_plot;
	std::vector<GroundNode*> DB;

	//void getGroundLine(std::vector<cv::Vec4f> &_lines, std::vector<cv::Vec4f> &groundlines, std::vector<cv::Point3d> &lineparam, int dev_idx);
	void getGroundLine2(std::vector<cv::Vec4f> &_lines, std::vector<cv::Vec4f> &groundlines, std::vector<cv::Point3d> &lineparam, int dev_idx);
	void compareImgs(cv::Mat &img1, cv::Mat &img2);
	void getInliar(cv::Mat *data, std::vector<cv::DMatch> *match, cv::Mat &meanVal, cv::Mat &stdVal);
	void drawPoint(cv::Mat &output, cv::Point2d &point, cv::Scalar color);
	void drawLine_ABC(cv::Mat &output, cv::Point3d &line, cv::Scalar color);
	void motion2warp(cv::Mat &inputMotion, cv::Mat &outputWarp, int center_col, int center_row);
	void getLinePoint(cv::Point3d &lineParam, cv::Point2d &outputPoint);
	int getMotion(GroundNode *ref, GroundNode *target, cv::Mat &iniMotion, cv::Mat &outputMotion);
	void RGB_processing(cv::Mat &output_, int dev_idx);
	void getNowNode(GroundNode **tempBefore);
	void estimateQuery(GroundNode *tempNow);
	void clearDB(void);
	void save(void);
	void load(void);

	int test_img_idx = 0;

#ifdef occlusionTest_time
	_timeb time_checker, time_checker_now;
	bool occlusion_on = true;
#endif

public:
	GroundLine::GroundLine(MotionCalc *data, RGBDcamera *c_data, GridData *grid_data, IPC_v2* ipc);

	MarkerPoseEstimater *m_markerPoseEstimater = nullptr;
	GroundNode *tempNow = nullptr;
	Input input;
	Option option;

	//void getIntrinFrom(RGBDcamera &input, GroundDetector* gd_info);
	void getParamFromYML(void);
	void getGroundImg(cv::Mat &_output);
	void run(void);
	void threadRun(void);
};
GroundLine::GroundLine(MotionCalc *data, RGBDcamera *c_data, GridData *grid_data, IPC_v2* ipc)
{
	m_ipc = ipc;
	m_data = data;
	m_camera_data = c_data;
	m_grid_data = grid_data;

	dataPath = "../data";
	_mkdir(dataPath.c_str());

	for (int i = 0; i < m_camera_data->num_of_senseor; i++)
	{
		RGB[i] = cv::Mat(m_camera_data->colorHeight, m_camera_data->colorWidth, CV_8UC3, m_camera_data->colorData[i]);
		ground_line_plot.push_back(cv::Mat(m_grid_data->cgridHeight, m_grid_data->cgridWidth, CV_8UC1));
	}

	int length_threshold = (int)(40.0 * option.imgResizeRate);
	float distance_threshold = 1.41421356f;
	double canny_th1 = 50.0;
	double canny_th2 = 20.0;
	int canny_aperture_size = 3;
	bool do_merge = false;
	ld = cv::ximgproc::createFastLineDetector(length_threshold,
		distance_threshold, canny_th1, canny_th2, canny_aperture_size,
		do_merge);

	matcher = cv::DescriptorMatcher::create("BruteForce");
	FreeGrid = cv::Mat(m_grid_data->cgridHeight, m_grid_data->cgridWidth, CV_8UC1, m_grid_data->freeData);
	ObstacleGrid = cv::Mat(m_grid_data->cgridHeight, m_grid_data->cgridWidth, CV_8UC1, m_grid_data->occupyData);

	//m_markerPoseEstimater = new MarkerPoseEstimater(c_data);
}
//void GroundLine::getGroundLine(std::vector<cv::Vec4f> &_lines, std::vector<cv::Vec4f> &groundlines, std::vector<cv::Point3d> &lineparam, int dev_idx)
//{
//	bool depthCheck = option.usingDepthCheck;
//	bool lengthCheck = true;
//	bool rangeCheck = false;
//	bool draw3DView = true;
//	bool filped = true;
//
//	if (m_camera_data->get_flag(Mode::draw4))
//		ground_line_plot[dev_idx].setTo(0);
//
//	double& mm2grid = m_grid_data->mm2grid;
//
//	cv::Mat vec1(3, 1, CV_64FC1);
//	cv::Mat vec2(3, 1, CV_64FC1);
//	cv::Mat vec4_1(4, 1, CV_64FC1);
//	cv::Mat vec4_2(4, 1, CV_64FC1);
//
//	groundlines.reserve((int)_lines.size());
//	std::vector<cv::Vec4f> _lines_on_ground;
//	_lines_on_ground.reserve((int)_lines.size());
//
//	cv::Mat depth_2_robot_R = input.depth_2_robot_RT[dev_idx](cv::Rect(0, 0, 3, 3));
//	cv::Mat& RT = input.depth_2_robot_RT[dev_idx];
//
//	std::vector<std::pair<cv::Point3f, cv::Point3f>> ground1, ground2, ground3;
//
//	for (int i = 0; i < (int)_lines.size(); i++)
//	{
//		vec1.at<double>(0, 0) = (double)_lines.at(i).val[0] / option.imgResizeRate;
//		vec1.at<double>(1, 0) = (double)_lines.at(i).val[1] / option.imgResizeRate;
//		vec1.at<double>(2, 0) = 1.0;
//
//		vec2.at<double>(0, 0) = (double)_lines.at(i).val[2] / option.imgResizeRate;
//		vec2.at<double>(1, 0) = (double)_lines.at(i).val[3] / option.imgResizeRate;
//		vec2.at<double>(2, 0) = 1.0;
//
//		if (filped)
//		{
//			vec1.at<double>(0, 0) = 640.0 - vec1.at<double>(0, 0);
//			vec2.at<double>(0, 0) = 640.0 - vec2.at<double>(0, 0);
//		}
//
//		cv::Mat temp3Dvec1 = input.color_K_inv[dev_idx] * vec1;
//		cv::Mat temp3Dvec2 = input.color_K_inv[dev_idx] * vec2;
//
//		//temp3Dvec1.at<double>(0, 0) = -temp3Dvec1.at<double>(0, 0);
//		//temp3Dvec2.at<double>(0, 0) = -temp3Dvec2.at<double>(0, 0);
//
//		if (option.usingColor2DepthRT)
//		{
//			vec4_1.at<double>(0, 0) = temp3Dvec1.at<double>(0, 0);
//			vec4_1.at<double>(1, 0) = temp3Dvec1.at<double>(1, 0);
//			vec4_1.at<double>(2, 0) = temp3Dvec1.at<double>(2, 0);
//			vec4_1.at<double>(3, 0) = 1.0;
//
//			vec4_2.at<double>(0, 0) = temp3Dvec2.at<double>(0, 0);
//			vec4_2.at<double>(1, 0) = temp3Dvec2.at<double>(1, 0);
//			vec4_2.at<double>(2, 0) = temp3Dvec2.at<double>(2, 0);
//			vec4_2.at<double>(3, 0) = 1.0;
//
//			vec4_1 = input.color_2_depth_RT[dev_idx] * vec4_1;
//			vec4_2 = input.color_2_depth_RT[dev_idx] * vec4_2;
//
//			temp3Dvec1.at<double>(0, 0) = vec4_1.at<double>(0, 0);
//			temp3Dvec1.at<double>(1, 0) = vec4_1.at<double>(1, 0);
//			temp3Dvec1.at<double>(2, 0) = vec4_1.at<double>(2, 0);
//
//			temp3Dvec2.at<double>(0, 0) = vec4_2.at<double>(0, 0);
//			temp3Dvec2.at<double>(1, 0) = vec4_2.at<double>(1, 0);
//			temp3Dvec2.at<double>(2, 0) = vec4_2.at<double>(2, 0);
//		}
//
//		cv::Mat ground_line1 = depth_2_robot_R * temp3Dvec1;
//		cv::Mat ground_line2 = depth_2_robot_R * temp3Dvec2;
//
//		//std::cout << std::endl << vec1 << std::endl;
//		//std::cout << temp3Dvec1 << std::endl;
//		//std::cout << ground_line1 << std::endl;
//		//cv::Mat test(240, 320, CV_8UC3, cv::Scalar(255, 255, 255));
//		//cv::line(test, cv::Point2f(_lines.at(i).val[0], _lines.at(i).val[1]),
//		//	cv::Point2f(_lines.at(i).val[2], _lines.at(i).val[3]), cv::Scalar(0, 0, 0));
//		//cv::imshow("test", test);
//		//cv::waitKey(0);
//
//		ground_line1 = ground_line1 / abs(ground_line1.at<double>(1, 0)) * RT.at<double>(1, 3);
//		ground_line2 = ground_line2 / abs(ground_line2.at<double>(1, 0)) * RT.at<double>(1, 3);
//
//		ground_line1.at<double>(0, 0) += RT.at<double>(0, 3);
//		ground_line2.at<double>(0, 0) += RT.at<double>(0, 3);
//		ground_line1.at<double>(2, 0) += RT.at<double>(2, 3);
//		ground_line2.at<double>(2, 0) += RT.at<double>(2, 3);
//
//		if (draw3DView)
//		{
//			ground1.push_back(std::pair<cv::Point3f, cv::Point3f>(
//				cv::Point3f((float)ground_line1.at<double>(0, 0), (float)ground_line1.at<double>(1, 0), (float)ground_line1.at<double>(2, 0)),
//				cv::Point3f((float)ground_line2.at<double>(0, 0), (float)ground_line2.at<double>(1, 0), (float)ground_line2.at<double>(2, 0))));
//		
//			//if (dev_idx == 1)
//			//{
//			//	temp3Dvec1 = RT.at<double>(1, 3) * temp3Dvec1;
//			//	temp3Dvec2 = RT.at<double>(1, 3) * temp3Dvec2;
//
//			//	ground2.push_back(std::pair<cv::Point3f, cv::Point3f>(
//			//		cv::Point3f(temp3Dvec1.at<double>(0, 0), temp3Dvec1.at<double>(1, 0), temp3Dvec1.at<double>(2, 0)),
//			//		cv::Point3f(temp3Dvec2.at<double>(0, 0), temp3Dvec2.at<double>(1, 0), temp3Dvec2.at<double>(2, 0))));
//			//
//			//	ground3.push_back(std::pair<cv::Point3f, cv::Point3f>(
//			//		cv::Point3f(ground_line1.at<double>(0, 0), ground_line1.at<double>(1, 0), ground_line1.at<double>(2, 0)),
//			//		cv::Point3f(ground_line2.at<double>(0, 0), ground_line2.at<double>(1, 0), ground_line2.at<double>(2, 0))));
//			//}
//		}
//
//
//		double l_dist1 = std::sqrt(ground_line1.at<double>(0, 0) * ground_line1.at<double>(0, 0)
//			+ ground_line1.at<double>(2, 0) * ground_line1.at<double>(2, 0));
//		double l_dist2 = std::sqrt(ground_line2.at<double>(0, 0) * ground_line2.at<double>(0, 0)
//			+ ground_line2.at<double>(2, 0) * ground_line2.at<double>(2, 0));
//
//		if (l_dist1 > 3000.0 && lengthCheck) continue;
//		if (l_dist2 > 3000.0 && lengthCheck) continue;
//
//		double g_x = ground_line2.at<double>(0, 0) - ground_line1.at<double>(0, 0);
//		double g_y = ground_line2.at<double>(2, 0) - ground_line1.at<double>(2, 0);
//
//		double g_dist = std::sqrt(g_x*g_x + g_y*g_y);
//		
//		if (g_dist < 200.0 && lengthCheck) continue;
//
//		//ground_line1.at<double>(0, 0) = -ground_line1.at<double>(0, 0);
//		//ground_line2.at<double>(0, 0) = -ground_line2.at<double>(0, 0);
//
//		cv::Point2d g_line1 = cv::Point2d((double)m_grid_data->cRobotCol - mm2grid * ground_line1.at<double>(0, 0)
//			, (double)m_grid_data->cRobotRow - mm2grid * ground_line1.at<double>(2, 0));
//		cv::Point2d g_line2 = cv::Point2d((double)m_grid_data->cRobotCol - mm2grid * ground_line2.at<double>(0, 0)
//			, (double)m_grid_data->cRobotRow - mm2grid * ground_line2.at<double>(2, 0));
//
//		if (rangeCheck)
//		{
//			if (g_line1.x < 0.0 || g_line1.x >= (double)m_grid_data->cgridWidth) continue;
//			if (g_line1.y < 0.0 || g_line1.y >= (double)m_grid_data->cgridHeight) continue;
//			if (g_line2.x < 0.0 || g_line2.x >= (double)m_grid_data->cgridWidth) continue;
//			if (g_line2.y < 0.0 || g_line2.y >= (double)m_grid_data->cgridHeight) continue;
//		}
//
//		bool depth_valid = true;
//		if (depthCheck)
//		{
//			if (FreeGrid.at<unsigned char>((int)g_line1.y, (int)g_line1.x) == 0) depth_valid = false;
//			if (FreeGrid.at<unsigned char>((int)g_line2.y, (int)g_line2.x) == 0) depth_valid = false;
//		}
//
//		if (depth_valid)
//		{
//			groundlines.push_back(cv::Vec4f((float)g_line1.x, (float)g_line1.y, (float)g_line2.x, (float)g_line2.y));
//			_lines_on_ground.push_back(_lines.at(i));
//			lineparam.push_back(cv::Point3d(g_x, -g_y, (g_y * ground_line1.at<double>(0, 0) - g_x * ground_line1.at<double>(2, 0))));
//		}
//
//		if (m_camera_data->get_flag(Mode::draw4))
//		{
//			cv::Scalar color = cv::Scalar(255);
//			if (!depth_valid) color = cv::Scalar(30);
//			//drawLine_ABC(ground_line_plot[dev_idx], lineparam.back(), cv::Scalar(125));
//			cv::line(ground_line_plot[dev_idx], g_line1, g_line2, color, 2);
//		}		
//	}
//
//	if (draw3DView)
//	{
//		if (dev_idx == 0)
//		{
//			view.clear("ground0");
//			view.clear("ground1");
//			view.clear("ground2");
//		}
//
//		cv::Scalar color;
//		if (dev_idx == 0) color = cv::Scalar(255, 0, 0);
//		if (dev_idx == 1) color = cv::Scalar(0, 255, 0);
//		if (dev_idx == 2) color = cv::Scalar(0, 0, 255);
//
//		view.insert(ground1, color, 1, cv::format("ground%d", dev_idx));
//		//if (dev_idx == 1)
//		//{
//		//	view.clear("ground3");
//		//	view.insert(ground3, cv::Scalar(0, 0, 255), 1, "ground3");
//		//}
//
//
//		if (dev_idx == 2)
//			view.draw(cv::Size(500, 500), "Ground Line");
//	}
//
//	_lines_on_ground.swap(_lines);
//}
void GroundLine::getGroundLine2(std::vector<cv::Vec4f> &_lines, std::vector<cv::Vec4f> &groundlines, std::vector<cv::Point3d> &lineparam, int dev_idx)
{
	bool depthCheck = option.usingDepthCheck;

	if (m_camera_data->get_flag(Mode::draw4) || true)
		ground_line_plot[dev_idx].setTo(0);

	double& mm2grid = m_grid_data->mm2grid;

	groundlines.reserve((int)_lines.size());
	std::vector<cv::Vec4f> _lines_on_ground;
	_lines_on_ground.reserve((int)_lines.size());

	cv::Mat allMat = input.allMat[dev_idx];
	if (allMat.type() != CV_64F) allMat.convertTo(allMat, CV_64F);

	//if(1) allMat = input.depth_2_robot_RT[dev_idx] * input.color_2_world_2_depth[dev_idx];
	//else
	//{
	//	cv::Mat colorK_4x4 = cv::Mat::eye(cv::Size(4, 4), CV_64FC1);
	//	input.color_K[dev_idx].copyTo(colorK_4x4(cv::Rect(0, 0, 3, 3)));

	//	cv::Mat c2w_double;
	//	m_markerPoseEstimater->Output_color_2_world[dev_idx].convertTo(c2w_double, CV_64FC1);

	//	if (c2w_double.empty())
	//		allMat = input.depth_2_robot_RT[dev_idx] * input.color_2_world_2_depth[dev_idx];
	//	else
	//		allMat = c2w_double * colorK_4x4.inv();
	//}

	for (int i = 0; i < (int)_lines.size(); i++)
	{
		//cv::Mat temp = Gray[dev_idx].clone();
		//cv::line(temp, cv::Point(_lines.at(i).val[0], _lines.at(i).val[1]),
		//	cv::Point(_lines.at(i).val[2], _lines.at(i).val[3]), cv::Scalar(255));
		//cv::imshow("temp", temp);
		//cv::waitKey(10);

		double x1 = (double)_lines.at(i).val[0] / option.imgResizeRate;
		double y1 = (double)_lines.at(i).val[1] / option.imgResizeRate;

		double x2 = (double)_lines.at(i).val[2] / option.imgResizeRate;
		double y2 = (double)_lines.at(i).val[3] / option.imgResizeRate;

		double z1 = -allMat.at<double>(1, 3) 
			/ (allMat.at<double>(1, 0) * x1 + allMat.at<double>(1, 1) * y1 + allMat.at<double>(1, 2));
		double z2 = -allMat.at<double>(1, 3) 
			/ (allMat.at<double>(1, 0) * x2 + allMat.at<double>(1, 1) * y2 + allMat.at<double>(1, 2));

		double wx1 = z1*(allMat.at<double>(0, 0)*x1 + allMat.at<double>(0, 1)*y1 + 
			allMat.at<double>(0, 2) + allMat.at<double>(0, 3) / z1);
		double wy1 = z1*(allMat.at<double>(2, 0)*x1 + allMat.at<double>(2, 1)*y1 + 
			allMat.at<double>(2, 2) + allMat.at<double>(2, 3) / z1);

		double wx2 = z2*(allMat.at<double>(0, 0)*x2 + allMat.at<double>(0, 1)*y2 + 
			allMat.at<double>(0, 2) + allMat.at<double>(0, 3) / z2);
		double wy2 = z2*(allMat.at<double>(2, 0)*x2 + allMat.at<double>(2, 1)*y2 + 
			allMat.at<double>(2, 2) + allMat.at<double>(2, 3) / z2);

		wx1 = -wx1;
		wx2 = -wx2;

		cv::Point2d g_line1 = cv::Point2d((double)m_grid_data->cRobotCol - mm2grid * wx1
			, (double)m_grid_data->cRobotRow - mm2grid * wy1);
		cv::Point2d g_line2 = cv::Point2d((double)m_grid_data->cRobotCol - mm2grid * wx2
			, (double)m_grid_data->cRobotRow - mm2grid * wy2);

		double g_x = wx2 - wx1;
		double g_y = wy2 - wy1;

		double g_dist = std::sqrt(g_x*g_x + g_y*g_y);

		if (g_dist < 200.0 || g_dist > 3000.0) continue;
		if (g_line1.x < 0 || g_line1.x >= m_grid_data->cgridWidth) continue;
		if (g_line1.y < 0 || g_line1.y >= m_grid_data->cgridHeight) continue;
		if (g_line2.x < 0 || g_line2.x >= m_grid_data->cgridWidth) continue;
		if (g_line2.y < 0 || g_line2.y >= m_grid_data->cgridHeight) continue;


		bool depth_valid = true;
		if (depthCheck)
		{
			if (FreeGrid.at<unsigned char>((int)g_line1.y, (int)g_line1.x) == 0) depth_valid = false;
			if (FreeGrid.at<unsigned char>((int)g_line2.y, (int)g_line2.x) == 0) depth_valid = false;
		}

		if (depth_valid)
		{
			groundlines.push_back(cv::Vec4f((float)g_line1.x, (float)g_line1.y, (float)g_line2.x, (float)g_line2.y));
			_lines_on_ground.push_back(_lines.at(i));
			lineparam.push_back(cv::Point3d(g_x, -g_y, (g_y * wx1 - g_x * wy1)));
		}

		if (m_camera_data->get_flag(Mode::draw4) || true)
		{
			cv::Scalar color = cv::Scalar(255);
			if (!depth_valid) color = cv::Scalar(30);
			//drawLine_ABC(ground_line_plot[dev_idx], lineparam.back(), cv::Scalar(125));
			//cv::line(ground_line_plot[dev_idx], g_line1, g_line2, color, 2);
			cv::arrowedLine(ground_line_plot[dev_idx], g_line1, g_line2, color, 2);
		}
	}

	_lines_on_ground.swap(_lines);
}
void GroundLine::compareImgs(cv::Mat &img1, cv::Mat &img2)
{
	cv::Mat canny1, canny2;
	std::vector<cv::Mat> merging_img;

	for (int i = 0; i < 3; i++) merging_img.push_back(cv::Mat());

	cv::Canny(img1, merging_img[2], 50, 50);
	cv::Canny(img2, merging_img[0], 50, 50);

	merging_img[1] = cv::Mat(merging_img[0].rows, merging_img[0].cols, CV_8UC1, cv::Scalar(0));

	cv::Mat result;
	cv::merge(merging_img, result);

	cv::imshow("compare imgs", result);
}
//void GroundLine::getIntrinFrom(RGBDcamera &_input, GroundDetector* gd_info)
//{
//	//if (option.using_Mask) _input.generateGroundMask = true;
//	//else _input.generateGroundMask = false;
//
//	for (int i = 0; i < m_camera_data->num_of_senseor; i++)
//	{
//		input.depth_2_robot_RT[i] = gd_info[i].output.RT;
//
//		input.color_K[i] = cv::Mat::eye(cv::Size(3, 3), CV_64FC1);
//		input.color_K[i].at<double>(0, 0) = _input.colorK[i][2];
//		input.color_K[i].at<double>(0, 2) = _input.colorK[i][0];
//		input.color_K[i].at<double>(1, 1) = _input.colorK[i][3];
//		input.color_K[i].at<double>(1, 2) = _input.colorK[i][1];
//
//		input.color_K_inv[i] = input.color_K[i].inv();
//		input.color_coeffs[i] = cv::Mat(1, 5, CV_32FC1, cv::Scalar(0));
//		std::memcpy(input.color_coeffs[i].data, _input.colorCoeffs[i], sizeof(float) * 5);
//
//		cv::Mat temp_depth_to_color_RT = cv::Mat::eye(cv::Size(4, 4), CV_64FC1);
//		temp_depth_to_color_RT.at<double>(0, 0) = _input.depth_to_color_R[i][0];
//		temp_depth_to_color_RT.at<double>(0, 1) = _input.depth_to_color_R[i][1];
//		temp_depth_to_color_RT.at<double>(0, 2) = _input.depth_to_color_R[i][2];
//
//		temp_depth_to_color_RT.at<double>(1, 0) = _input.depth_to_color_R[i][3];
//		temp_depth_to_color_RT.at<double>(1, 1) = _input.depth_to_color_R[i][4];
//		temp_depth_to_color_RT.at<double>(1, 2) = _input.depth_to_color_R[i][5];
//
//		temp_depth_to_color_RT.at<double>(2, 0) = _input.depth_to_color_R[i][6];
//		temp_depth_to_color_RT.at<double>(2, 1) = _input.depth_to_color_R[i][7];
//		temp_depth_to_color_RT.at<double>(2, 2) = _input.depth_to_color_R[i][8];
//
//		temp_depth_to_color_RT.at<double>(0, 3) = _input.depth_to_color_tvec[i][0];
//		temp_depth_to_color_RT.at<double>(1, 3) = _input.depth_to_color_tvec[i][1];
//		temp_depth_to_color_RT.at<double>(2, 3) = _input.depth_to_color_tvec[i][2];
//
//		input.color_2_depth_RT[i] = temp_depth_to_color_RT.inv();
//
//		cv::Mat colorK_4x4 = cv::Mat::eye(cv::Size(4, 4), CV_64FC1);
//		input.color_K[i].copyTo(colorK_4x4(cv::Rect(0, 0, 3, 3)));
//
//		cv::Mat color_2_depth;
//		if(option.usingColor2DepthRT) color_2_depth = input.color_2_depth_RT[i].clone();
//		else color_2_depth = cv::Mat::eye(cv::Size(4, 4), CV_64FC1);
//
//		input.color_2_world_2_depth[i] = color_2_depth * colorK_4x4.inv();
//	}
//}
void GroundLine::getParamFromYML(void)
{
	for (int i = 0; i < m_camera_data->num_of_senseor; i++)
	{
		//input.depth_2_robot_RT[i] = gd_info[i].output.RT;

		input.color_K[i] = cv::Mat::eye(cv::Size(3, 3), CV_32FC1);
		input.color_K[i].at<float>(0, 0) = m_camera_data->colorK[i][2];
		input.color_K[i].at<float>(0, 2) = m_camera_data->colorK[i][0];
		input.color_K[i].at<float>(1, 1) = m_camera_data->colorK[i][3];
		input.color_K[i].at<float>(1, 2) = m_camera_data->colorK[i][1];

		//input.color_K_inv[i] = input.color_K[i].inv();
		input.color_coeffs[i] = cv::Mat(1, 4, CV_32FC1, cv::Scalar(0));
		std::memcpy(input.color_coeffs[i].data, m_camera_data->colorCoeffs[i], sizeof(float) * 4);

		cv::Mat color_k_inv = input.color_K[i].inv();
		cv::Mat color_k_inv_4_4 = cv::Mat::eye(cv::Size(4, 4), color_k_inv.type());
		color_k_inv.copyTo(color_k_inv_4_4(cv::Rect(0, 0, 3, 3)));

		std::string data_path = "..\\Data\\calibration_orbbec_"
			+ std::string(m_camera_data->camera_order[i]) + ".yml";
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

		cv::Mat extrin = Cam2GroundRt * Cam2Cam;

		input.allMat[i] = extrin * color_k_inv_4_4;
	}
}
void GroundLine::getInliar(cv::Mat *data, std::vector<cv::DMatch> *match, cv::Mat &meanVal, cv::Mat &stdVal)
{
	float std_val_rate = 2.0f;

	int counter = 0;
	while (1)
	{
		cv::meanStdDev(*data, meanVal, stdVal);

		cv::Mat new_angle;
		std::vector<cv::DMatch> new_inliar_match;

		double std_val_threashold = std_val_rate * stdVal.at<double>(0, 0);
		for (int a = 0; a < data->rows; a++)
		{
			float gap = abs(data->at<float>(a, 0) - (float)meanVal.at<double>(0, 0));
			if (gap <= std_val_threashold)
			{
				new_angle.push_back(data->at<float>(a, 0));
				new_inliar_match.push_back(match->at(a));
			}
		}

		if (counter++ > 10) break;

		if (new_angle.rows == data->rows) break;
		else
		{
			*data = new_angle.clone();
			match->swap(new_inliar_match);
		}
	}
}
void GroundLine::drawPoint(cv::Mat &output, cv::Point2d &point, cv::Scalar color)
{
	cv::Point2d plotPoint = cv::Point2d((double)m_grid_data->cRobotCol - m_grid_data->mm2grid * point.y,
		(double)m_grid_data->cRobotRow - m_grid_data->mm2grid * point.x);

	cv::circle(output, plotPoint, 10, color, -1);
}
void GroundLine::drawLine_ABC(cv::Mat &output, cv::Point3d &line, cv::Scalar color)
{
	cv::Point2d Points[4];

	double low_r = m_grid_data->cRobotRow / m_grid_data->mm2grid;
	double high_r = (m_grid_data->cRobotRow - output.rows) / m_grid_data->mm2grid;
	Points[0] = cv::Point2d(m_grid_data->cRobotCol - m_grid_data->mm2grid * (-(line.z + line.x * low_r) / line.y), 0);
	Points[1] = cv::Point2d(m_grid_data->cRobotCol - m_grid_data->mm2grid * (-(line.z + line.x * high_r) / line.y), output.rows);

	double low_c = m_grid_data->cRobotCol / m_grid_data->mm2grid;
	double high_c = (m_grid_data->cRobotCol - output.cols) / m_grid_data->mm2grid;
	Points[2] = cv::Point2d(0, m_grid_data->cRobotRow - m_grid_data->mm2grid * (-(line.z + line.y * low_c) / line.x));
	Points[3] = cv::Point2d(output.cols, m_grid_data->cRobotRow - m_grid_data->mm2grid * (-(line.z + line.y * high_c) / line.x));


	cv::Point2d inPoints[2];
	inPoints[0] = Points[0];
	inPoints[1] = Points[1];


	if (Points[0].x < 0) inPoints[0] = Points[2];
	if (Points[0].x >= output.cols) inPoints[0] = Points[3];

	if (Points[1].x < 0) inPoints[1] = Points[2];
	if (Points[1].x >= output.cols) inPoints[1] = Points[3];


	cv::line(output, inPoints[0], inPoints[1], color, 4);

}
void GroundLine::motion2warp(cv::Mat &inputMotion, cv::Mat &outputWarp, int center_col, int center_row)
{
	double x_vec_norm_checker = inputMotion.at<double>(0, 0)*inputMotion.at<double>(0, 0) + inputMotion.at<double>(0, 1)*inputMotion.at<double>(0, 1) - 1.0;
	double y_vec_norm_checker = inputMotion.at<double>(1, 0)*inputMotion.at<double>(1, 0) + inputMotion.at<double>(1, 1)*inputMotion.at<double>(1, 1) - 1.0;

	bool mat_validation = true;
	if (abs(x_vec_norm_checker) > 0.1) mat_validation = false;
	if (abs(y_vec_norm_checker) > 0.1) mat_validation = false;
	if (!mat_validation)
	{
		inputMotion.at<double>(0, 0) = 1.0;
		inputMotion.at<double>(0, 1) = 0.0;
		inputMotion.at<double>(0, 2) = 0.0;

		inputMotion.at<double>(1, 0) = 0.0;
		inputMotion.at<double>(1, 1) = 1.0;
		inputMotion.at<double>(1, 2) = 0.0;

		inputMotion.at<double>(2, 0) = 0.0;
		inputMotion.at<double>(2, 1) = 0.0;
		inputMotion.at<double>(2, 2) = 1.0;
	}

	double angle_radian = atan2(-inputMotion.at<double>(0, 1), inputMotion.at<double>(0, 0));
	double angle_degree = 180.0 / CV_PI * angle_radian;

	outputWarp = cv::getRotationMatrix2D(cv::Point2f((float)m_grid_data->cRobotCol, (float)m_grid_data->cRobotRow), angle_degree, 1);

	outputWarp.at<double>(0, 2) -= (m_grid_data->mm2grid * inputMotion.at<double>(1, 2));
	outputWarp.at<double>(1, 2) -= (m_grid_data->mm2grid * inputMotion.at<double>(0, 2));
}
void GroundLine::getLinePoint(cv::Point3d &lineParam, cv::Point2d &outputPoint)
{
	double absquar = lineParam.x*lineParam.x + lineParam.y*lineParam.y;
	outputPoint = cv::Point2d(-(lineParam.x * lineParam.z) / (absquar), -(lineParam.y * lineParam.z) / (absquar));
}
int GroundLine::getMotion(GroundNode *ref, GroundNode *target, cv::Mat &iniMotion, cv::Mat &outputMotion)
{
	/////////////////////////////////////////////////////////////////////
	bool draw_query = true;
	bool draw_result = true;
	bool draw_infinite_line_matching = true;

	//low=hard
	float nndr = 0.7f;
	//float nndr = 0.6f;
	double vertical_line_detector = 0.25;
	int number_of_matching_line = 3;
	/////////////////////////////////////////////////////////////////////

	if (ref->desc.rows == 0 || target->desc.rows == 0)
	{
		printf("There is no desc in getMotion %d %d\n", ref->desc.rows, target->desc.rows);
		return 0;
	}
	
	cv::Mat warpMat, warped_img, merge_warped_img;
	motion2warp(iniMotion, warpMat, m_grid_data->cRobotCol, m_grid_data->cRobotRow);
	if (draw_query && m_camera_data->get_flag(Mode::draw4))
	{
		if (!target->GroundImg.empty())
		{

			cv::warpAffine(target->GroundImg, warped_img, warpMat,
				cv::Size(target->GroundImg.cols, target->GroundImg.rows),
				1, 0, cv::Scalar(0, 0, 0));

			merge_warped_img = cv::max(warped_img, ref->GroundImg);
		}
		else
		{
			merge_warped_img = cv::Mat(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));
		}
	}

	cv::Mat result_img;
	if (draw_result && m_camera_data->get_flag(Mode::draw4))
	{
		if (!ref->GroundImg.empty())
			result_img = cv::Mat(ref->GroundImg.size(), CV_8UC3, cv::Scalar(0, 0, 0));
		else
			result_img = cv::Mat(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));
	}

	//printf("rows : %d %d\n", ref.desc.rows, target.desc.rows);
	std::vector< std::vector<cv::DMatch> > knmatches;
	matcher->knnMatch(ref->desc, target->desc, knmatches, 2);

	cv::Mat angle;
	std::vector<cv::DMatch> inliar_match;
	float pi2degree = 2.0f * (float)CV_PI;

	for (int i = 0; i<(int)knmatches.size(); ++i)
	{
		if (knmatches[i].size() == 2)
		{
			float dist1 = knmatches[i][0].distance;
			float dist2 = knmatches[i][1].distance;
			float distance = dist1 / dist2;
			if (distance > nndr) continue;
		}

		cv::DMatch &match = knmatches[i][0];

		int q = match.queryIdx;
		int t = match.trainIdx;

		float x1 = (float)warpMat.at<double>(0, 0) * target->ground_lines[t].val[0]
			+ (float)warpMat.at<double>(0, 1) * target->ground_lines[t].val[1] + (float)warpMat.at<double>(0, 2);
		float y1 = (float)warpMat.at<double>(1, 0) * target->ground_lines[t].val[0]
			+ (float)warpMat.at<double>(1, 1) * target->ground_lines[t].val[1] + (float)warpMat.at<double>(1, 2);
		float x2 = (float)warpMat.at<double>(0, 0) * target->ground_lines[t].val[2]
			+ (float)warpMat.at<double>(0, 1) * target->ground_lines[t].val[3] + (float)warpMat.at<double>(0, 2);
		float y2 = (float)warpMat.at<double>(1, 0) * target->ground_lines[t].val[2]
			+ (float)warpMat.at<double>(1, 1) * target->ground_lines[t].val[3] + (float)warpMat.at<double>(1, 2);

		cv::Vec4f iniPose(x1, y1, x2, y2);

		float ref_angle = lineComputer.getAngle(&ref->ground_lines[q]);
		float target_angle = lineComputer.getAngle(&iniPose);
		float gap_angle = target_angle - ref_angle;
		if (gap_angle >(float)CV_PI) gap_angle -= pi2degree;
		if (gap_angle <= -(float)CV_PI) gap_angle += pi2degree;
		if (abs(gap_angle) > (float)CV_PI / 3.0f) continue;

		double from_c_x = (double)(ref->ground_lines.at(q).val[0] + ref->ground_lines.at(q).val[2]) / 2.0;
		double from_c_y = (double)(ref->ground_lines.at(q).val[1] + ref->ground_lines.at(q).val[3]) / 2.0;

		int to_c_x = (int)((iniPose.val[0] + iniPose.val[2]) / 2.0f);
		int to_c_y = (int)((iniPose.val[1] + iniPose.val[3]) / 2.0f);

		if (from_c_x < 0 || from_c_x >= m_grid_data->cgridWidth) continue;
		if (from_c_y < 0 || from_c_y >= m_grid_data->cgridHeight) continue;
		if (to_c_x < 0 || to_c_x >= m_grid_data->cgridWidth) continue;
		if (to_c_y < 0 || to_c_y >= m_grid_data->cgridHeight) continue;

		double gap_x = (double)(to_c_x - from_c_x);
		double gap_y = (double)(to_c_y - from_c_y);
		double gap_dist = std::sqrt(gap_x*gap_x + gap_y*gap_y);

		bool inlier = true;

		// 100 pixel distance 
		if (gap_dist > 100) inlier = false;;

		if (inlier)
		{
			angle.push_back(gap_angle);
			inliar_match.push_back(match);
		}

		if (draw_query && m_camera_data->get_flag(Mode::draw4))
		{
			cv::Point2d from_point(from_c_x, from_c_y);
			cv::Point2d to_point(to_c_x, to_c_y);

			cv::Scalar color;
			if (inlier) color = cv::Scalar(255, 0, 0);
			else color = cv::Scalar(0, 0, 255);

			cv::circle(merge_warped_img, from_point, 5, cv::Scalar(0, 255, 0));
			cv::circle(merge_warped_img, to_point, 5, cv::Scalar(0, 255, 0));
			//cv::putText(merge_warped_img, cv::format("%d", angle.rows), from_point, 0, 0.5, cv::Scalar(0, 0, 0), 2);
			cv::arrowedLine(merge_warped_img, from_point, to_point, color, 2);
		}
	}

	if (draw_query && m_camera_data->get_flag(Mode::draw4))
	{
		cv::imshow("ground query", merge_warped_img);
		cv::waitKey(1);
	}

	//printf("inliar angle : %d\n", angle.rows);
	if (angle.empty() || angle.rows < number_of_matching_line)
	{
		if (draw_result && m_camera_data->get_flag(Mode::draw4))
		{
			cv::putText(result_img, cv::format("Not enough matching %d < %d", angle.rows, number_of_matching_line),
				cv::Point(15, 25), 0, 0.8, cv::Scalar(0, 255, 0));
			cv::imshow("ground result", result_img);
			cv::waitKey(1);
		}		
		return false;
	}

	if (draw_result && m_camera_data->get_flag(Mode::draw4))
	{
		cv::putText(result_img, cv::format("matching %d >= %d", angle.rows, number_of_matching_line),
			cv::Point(15, 25), 0, 0.8, cv::Scalar(0, 255, 0));
	}

	cv::Mat angle_meanVal, angle_stdVal;
	getInliar(&angle, &inliar_match, angle_meanVal, angle_stdVal);

	double stdVal_in_dgree = 180.0 / CV_PI * angle_stdVal.at<double>(0, 0);
	//printf("%lf, %d\n", stdVal_in_dgree, (int)inliar_match.size());
	if (stdVal_in_dgree > 1.5 || (int)inliar_match.size() == 0)
	{
		if (draw_result && m_camera_data->get_flag(Mode::draw4))
		{
			cv::putText(result_img, cv::format("Not enough iniler std:%lf, %d == 0", stdVal_in_dgree, (int)inliar_match.size()),
				cv::Point(15, 55), 0, 0.5, cv::Scalar(0, 255, 0));
			cv::imshow("ground result", result_img);
			cv::waitKey(1);
		}
		return false;
	}

	if (draw_result && m_camera_data->get_flag(Mode::draw4))
	{
		cv::putText(result_img, cv::format("iniler std:%lf, %d == 0", stdVal_in_dgree, (int)inliar_match.size()),
			cv::Point(15, 55), 0, 0.8, cv::Scalar(0, 255, 0));
	}

	double a = angle_meanVal.at<double>(0, 0);
	double a_dgree = 180.0 / CV_PI * a;

	std::vector<cv::Point3d> rotated_from_line;
	rotated_from_line.reserve((int)inliar_match.size());

	cv::Mat correctedRT = cv::Mat::eye(cv::Size(3, 3), CV_64FC1);
	correctedRT.at<double>(0, 0) = cos(a);
	correctedRT.at<double>(0, 1) = -sin(a);
	correctedRT.at<double>(1, 0) = -correctedRT.at<double>(0, 1);
	correctedRT.at<double>(1, 1) = correctedRT.at<double>(0, 0);

	std::vector<cv::Point2d> from_line_points;
	from_line_points.reserve((int)inliar_match.size());
	for (int i = 0; i < (int)inliar_match.size(); i++)
	{
		from_line_points.push_back(cv::Point2d());
		getLinePoint(ref->line_param.at(inliar_match.at(i).queryIdx), from_line_points.back());
	}
	int counter = 10;
	while (1)
	{
		cv::Mat solved_RT_inv = iniMotion.inv() * correctedRT.inv();
		double meanX[2] = {};
		double meanY[2] = {};
		for (int i = 0; i < (int)inliar_match.size(); i++)
		{
			cv::Point3d to_line = target->line_param.at(inliar_match.at(i).trainIdx);

			double new_x = solved_RT_inv.at<double>(0, 0) * to_line.x + solved_RT_inv.at<double>(1, 0) * to_line.y + solved_RT_inv.at<double>(2, 0) * to_line.z;
			double new_y = solved_RT_inv.at<double>(0, 1) * to_line.x + solved_RT_inv.at<double>(1, 1) * to_line.y + solved_RT_inv.at<double>(2, 1) * to_line.z;
			double new_z = solved_RT_inv.at<double>(0, 2) * to_line.x + solved_RT_inv.at<double>(1, 2) * to_line.y + solved_RT_inv.at<double>(2, 2) * to_line.z;

			cv::Point3d to_line_new(new_x, new_y, new_z);

			cv::Point2d to_line_point;
			getLinePoint(to_line_new, to_line_point);

			double x_gap = to_line_point.x - from_line_points[i].x;
			double y_gap = to_line_point.y - from_line_points[i].y;
			double weight = x_gap*x_gap + y_gap*y_gap;

			meanX[0] += (weight * (x_gap));
			meanX[1] += weight;
			meanY[0] += (weight * (y_gap));
			meanY[1] += weight;
		}
		double resultX = -meanX[0] / meanX[1];
		double resultY = -meanY[0] / meanY[1];
		correctedRT.at<double>(0, 2) += 0.5 * resultX;
		correctedRT.at<double>(1, 2) += 0.5 * resultY;

		//if (abs(resultX) <= 0.02 && abs(resultY) <= 0.02) break;
		//if (abs(resultX) <= 100.0 && abs(resultY) <= 100.0) break;
		if (abs(resultX) <= 50.0 && abs(resultY) <= 50.0) break;

		if (--counter <= 0)
		{
			if (draw_result && m_camera_data->get_flag(Mode::draw4))
			{
				cv::putText(result_img, cv::format("Not enough counter"),
					cv::Point(15, 85), 0, 0.8, cv::Scalar(0, 255, 0));
				cv::imshow("ground result", result_img);
				cv::waitKey(1);
			}
			return false;
		}
	}

	outputMotion = correctedRT * iniMotion;

	if (draw_infinite_line_matching && m_camera_data->get_flag(Mode::draw4))
	{
		cv::Mat match_line_plot_t, match_line_plot_q, merged_img;
		match_line_plot_t = cv::Mat(500, 500, CV_8UC1, cv::Scalar(0));
		match_line_plot_q = cv::Mat(500, 500, CV_8UC1, cv::Scalar(0));

		cv::Mat output_inv = outputMotion.inv();
		for (int i = 0; i < (int)inliar_match.size(); i++)
		{
			cv::Point3d to_line = target->line_param.at(inliar_match.at(i).trainIdx);
			cv::Point3d from_line = ref->line_param.at(inliar_match.at(i).queryIdx);

			double r_x = output_inv.at<double>(0, 0) * to_line.x + output_inv.at<double>(1, 0) * to_line.y + output_inv.at<double>(2, 0) * to_line.z;
			double r_y = output_inv.at<double>(0, 1) * to_line.x + output_inv.at<double>(1, 1) * to_line.y + output_inv.at<double>(2, 1) * to_line.z;
			double r_z = output_inv.at<double>(0, 2) * to_line.x + output_inv.at<double>(1, 2) * to_line.y + output_inv.at<double>(2, 2) * to_line.z;

			drawLine_ABC(match_line_plot_q, from_line, cv::Scalar(255));
			drawLine_ABC(match_line_plot_t, cv::Point3d(r_x, r_y, r_z), cv::Scalar(255));
		}

		std::vector<cv::Mat> merge;
		merge.push_back(match_line_plot_t);
		merge.push_back(cv::Mat(match_line_plot_t.rows, match_line_plot_t.cols, CV_8UC1, cv::Scalar(0)));
		merge.push_back(match_line_plot_q);

		cv::merge(merge, merged_img);
		cv::putText(merged_img, cv::format("a_v : %4.1lf, a : %4.1lf", stdVal_in_dgree, -a_dgree),
			cv::Point(15, 25), 0, 0.8, cv::Scalar(0, 255, 0), 2);
		cv::imshow("merged_img", merged_img);
	}

	if (draw_result && m_camera_data->get_flag(Mode::draw4))
	{
		cv::Mat result_warped_img;
		cv::Mat result_warp;
		motion2warp(outputMotion, result_warp, m_grid_data->cRobotCol, m_grid_data->cRobotRow);

		if (!target->GroundImg.empty())
		{
			cv::warpAffine(target->GroundImg, result_warped_img, result_warp,
				cv::Size(target->GroundImg.cols, target->GroundImg.rows),
				1, 0, cv::Scalar(0, 0, 0));

			result_img = cv::max(result_warped_img, ref->GroundImg);
		}
		else
		{
			result_img = cv::Mat(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));
		}

		cv::putText(result_img, cv::format("a:%lf x:%lf y:%lf", a, correctedRT.at<double>(0, 2), correctedRT.at<double>(1, 2)),
			cv::Point(15, 85), 0, 0.5, cv::Scalar(0, 255, 0));

		cv::imshow("ground result", result_img);
		cv::waitKey(1);
	}

	return 1;
}
void GroundLine::RGB_processing(cv::Mat &output_, int dev_idx)
{
#ifdef occlusionTest_time
	if (occlusionTest_time != 0)
	{
		_ftime64_s(&time_checker_now);
		int dt = (int)(time_checker_now.time - time_checker.time);
		int remain_t;
		if (occlusion_on)
		{
			remain_t = occlusionTest_time - dt;
			if (dt > occlusionTest_time)
			{
				occlusion_on = false;
				time_checker = time_checker_now;
			}
		}
		else if (!occlusion_on)
		{
			remain_t = occlusionTest_open_time - dt;
			if (dt > occlusionTest_open_time)
			{
				occlusion_on = true;
				time_checker = time_checker_now;
			}
		}

		if (occlusion_on)
			RGB[dev_idx].setTo(0);

		cv::putText(RGB[dev_idx], cv::format("%ds", remain_t), cv::Point(15, 25), 0, 0.8, cv::Scalar(255, 255, 255));
	}
#endif

	cv::Mat gray;
	cv::cvtColor(RGB[dev_idx], gray, CV_RGB2GRAY);

	//if (option.using_Mask && !input.groundMask[dev_idx].empty()) 
	//	gray = gray.mul(input.groundMask[dev_idx]);

	if (option.undistortion)
	{
		cv::undistort(gray, output_, input.color_K[dev_idx], input.color_coeffs[dev_idx]);
		if (option.imgResizeRate != 1.0) cv::resize(output_, output_,
			cv::Size((int)((double)output_.cols * option.imgResizeRate), (int)((double)output_.rows * option.imgResizeRate)));
	}	
	else if (option.imgResizeRate != 1.0)
	{
		cv::resize(gray, output_,
			cv::Size((int)((double)RGB[dev_idx].cols * option.imgResizeRate), (int)((double)RGB[dev_idx].rows * option.imgResizeRate)));
	}
	else
	{
		output_ = gray;
	}
}
void GroundLine::getGroundImg(cv::Mat &_output)
{
	if (ground_line_plot.empty()) return;

	//if (!ground_line_plot[0].empty() && m_camera_data->get_flag(Mode::draw4))
	if (!ground_line_plot[0].empty())
	{
		if (ground_line_plot.size() == 3)
		{
			cv::merge(ground_line_plot, _output);
		}
		else if (ground_line_plot.size() == 1)
		{
			cv::cvtColor(ground_line_plot[0], _output, CV_GRAY2RGB);
		}
		else if (ground_line_plot.size() == 2)
		{
			ground_line_plot.push_back(ground_line_plot[0]);
			cv::merge(ground_line_plot, _output);
		}

		cv::Mat freegrid = FreeGrid.clone();
		freegrid *= 0.3;
		cv::cvtColor(freegrid, freegrid, CV_GRAY2RGB);

		cv::line(_output, cv::Point(0, m_grid_data->cRobotRow), cv::Point(freegrid.cols, m_grid_data->cRobotRow), cv::Scalar(200, 200, 200), 1);
		cv::line(_output, cv::Point(m_grid_data->cRobotCol, 0), cv::Point(m_grid_data->cRobotCol, freegrid.rows), cv::Scalar(200, 200, 200), 1);

		_output = cv::max(_output, freegrid);
	}
}
void GroundLine::getNowNode(GroundNode **tempBefore)
{
	if (*tempBefore != nullptr)
	{
		bool deleteBeforeNode = false;
		if (DB.size() != 0) if (DB.back() != *tempBefore) deleteBeforeNode = true;

		if (DB.size() == 0) deleteBeforeNode = true;

		if (deleteBeforeNode)
		{
			delete *tempBefore;
			*tempBefore = nullptr;
		}
	}

	*tempBefore = new GroundNode;

	cv::Mat line_plot_img, line_plot_ROI;
	if (m_camera_data->get_flag(Mode::draw3))
	{
		if(Ground_line_plot_img.empty()) Ground_line_plot_img = cv::Mat(240, 320 * 3, CV_8UC3, cv::Scalar(0, 0, 0));
		Ground_line_plot_img.setTo(0);
	}

	for (int i = 0; i < m_camera_data->num_of_senseor; i++)
	{
		std::vector<cv::Vec4f> temp_lines;
		std::vector<cv::Vec4f> temp_ground_lines;
		std::vector<cv::Point3d> temp_lineparam;
		cv::Mat temp_desc;

		RGB_processing(Gray[i], i);

		//cv::imshow("Gray", Gray[i]);
		//cv::waitKey(1);

		ld->detect(Gray[i], temp_lines);

		if (m_camera_data->get_flag(Mode::draw3) && !Ground_line_plot_img.empty())
		{
			line_plot_ROI = Ground_line_plot_img(cv::Rect(320 * i, 0, 320, 240));
			line_plot_img = Gray[i].clone();			
			ld->drawSegments(line_plot_img, temp_lines);
		}

		//getGroundLine(temp_lines, temp_ground_lines, temp_lineparam, i);
		getGroundLine2(temp_lines, temp_ground_lines, temp_lineparam, i);

		temp_desc = cv::Mat((int)temp_lines.size(), 72, CV_32FC1, cv::Scalar(0));
		lineComputer.gray_RGB = Gray[i];
		lineComputer.line_compute(temp_lines, temp_desc);

		for (int j = 0; j < (int)temp_lines.size(); j++)
		{
			(*tempBefore)->lines.push_back(temp_lines[j]);
			(*tempBefore)->ground_lines.push_back(temp_ground_lines[j]);
			(*tempBefore)->line_param.push_back(temp_lineparam[j]);
		}

		(*tempBefore)->desc.push_back(temp_desc);

		if (m_camera_data->get_flag(Mode::draw3) && !Ground_line_plot_img.empty())
		{
			ld->drawSegments(line_plot_img, temp_lines);
			cv::resize(line_plot_img, line_plot_ROI, cv::Size(320, 240));
		}
	}

	getGroundImg((*tempBefore)->GroundImg);

	//cv::imshow("test", (*tempBefore)->GroundImg);
	//cv::waitKey(10);
}
void GroundLine::estimateQuery(GroundNode *tempNow)
{
	for (int i = 0; i < m_data->querySize; i++)
	{
		if (m_data->queryidx[i] < 0 || m_data->queryidx[i] >= (int)DB.size()) continue;

		cv::Mat iniPose(3, 3, CV_64FC1, m_data->queryIniPose[i]);
		cv::Mat outputPose;

		printf("querying %d\n", m_data->queryidx[i]);
		if (getMotion(tempNow, DB.at(m_data->queryidx[i]), iniPose, outputPose))
		{
			std::memcpy(m_data->matchingResult[i], outputPose.data, sizeof(double) * 9);
			m_data->matchingRate[i] = 1.0;
		}
		else
		{
			m_data->matchingRate[i] = 0.0;
		}
	}
}
void GroundLine::clearDB(void)
{
	for (int i = 0; i < (int)DB.size(); i++)
	{
		//DB[i]->pop();
		delete DB[i];
	}
	DB.clear();
}
void GroundLine::save(void)
{
	std::string path(m_camera_data->mapData_folder);

	_mkdir(path.c_str());

	std::string file = path + MapdataName;

	std::ofstream MapDBsaver(file, std::ios_base::out | std::ios_base::trunc | std::ios_base::binary);
	if (!MapDBsaver.is_open())
	{
		printf("\nCannot open %s\n", file.c_str());
		return;
	}

	int DB_size = (int)DB.size();
	MapDBsaver.write((char*)&DB_size, sizeof(int));

	for (int i = 0; i < (int)DB_size; i++)
	{
		int lines_size = (int)DB[i]->lines.size();
		MapDBsaver.write((char*)&lines_size, sizeof(int));
		for (int j = 0; j < lines_size; j++)
		{
			MapDBsaver.write((char*)&DB[i]->lines[j].val[0], sizeof(float));
			MapDBsaver.write((char*)&DB[i]->lines[j].val[1], sizeof(float));
			MapDBsaver.write((char*)&DB[i]->lines[j].val[2], sizeof(float));
			MapDBsaver.write((char*)&DB[i]->lines[j].val[3], sizeof(float));
		}

		int ground_lines_size = (int)DB[i]->ground_lines.size();
		MapDBsaver.write((char*)&ground_lines_size, sizeof(int));
		for (int j = 0; j < ground_lines_size; j++)
		{
			MapDBsaver.write((char*)&DB[i]->ground_lines[j].val[0], sizeof(float));
			MapDBsaver.write((char*)&DB[i]->ground_lines[j].val[1], sizeof(float));
			MapDBsaver.write((char*)&DB[i]->ground_lines[j].val[2], sizeof(float));
			MapDBsaver.write((char*)&DB[i]->ground_lines[j].val[3], sizeof(float));
		}

		int line_param_size = (int)DB[i]->line_param.size();
		MapDBsaver.write((char*)&line_param_size, sizeof(int));
		for (int j = 0; j < line_param_size; j++)
		{
			MapDBsaver.write((char*)&DB[i]->line_param[j].x, sizeof(double));
			MapDBsaver.write((char*)&DB[i]->line_param[j].y, sizeof(double));
			MapDBsaver.write((char*)&DB[i]->line_param[j].z, sizeof(double));
		}

		int d_row = DB[i]->desc.rows;
		int d_col = DB[i]->desc.cols;
		MapDBsaver.write((char*)&d_row, sizeof(int));
		MapDBsaver.write((char*)&d_col, sizeof(int));

		int d_size = d_row * d_col;
		if (d_size != 0)
		{
			int d_type = DB[i]->desc.type();
			MapDBsaver.write((char*)&d_type, sizeof(int));

			if (d_type == 6) MapDBsaver.write((char*)DB[i]->desc.data, sizeof(double) * d_size);
			if (d_type == 5) MapDBsaver.write((char*)DB[i]->desc.data, sizeof(float) * d_size);
		}


		int g_row = DB[i]->GroundImg.rows;
		int g_col = DB[i]->GroundImg.cols;
		MapDBsaver.write((char*)&g_row, sizeof(int));
		MapDBsaver.write((char*)&g_col, sizeof(int));

		int g_size = g_row * g_col;
		if (g_size != 0)
		{
			MapDBsaver.write((char*)DB[i]->GroundImg.data, sizeof(unsigned char) * g_size * 3);
		}

		printf("%d %d %d\n", i, (int)DB[i]->desc.rows, d_row);
	}

	MapDBsaver.close();

	std::cout << std::to_string(DB_size) << " nodes are saved" << std::endl;
}
void GroundLine::load(void)
{
	std::string path(m_camera_data->mapData_folder);
	std::string file = path + MapdataName;

	std::ifstream MapDBloader(file, std::ios_base::in | std::ios_base::binary);
	if (!MapDBloader.is_open())
	{
		printf("\nCannot open %s\n", file.c_str());
		return;
	}

	clearDB();

	int DB_size;
	MapDBloader.read((char*)&DB_size, sizeof(int));
	DB.reserve(DB_size);

	for (int i = 0; i < DB_size; i++)
	{
		GroundNode *newNode = new GroundNode;

		int lines_size;
		MapDBloader.read((char*)&lines_size, sizeof(int));
		for (int j = 0; j < lines_size; j++)
		{
			float val[4];
			MapDBloader.read((char*)&val[0], sizeof(float));
			MapDBloader.read((char*)&val[1], sizeof(float));
			MapDBloader.read((char*)&val[2], sizeof(float));
			MapDBloader.read((char*)&val[3], sizeof(float));
			newNode->lines.push_back(cv::Vec4f(val[0], val[1], val[2], val[3]));
		}

		int ground_lines_size;
		MapDBloader.read((char*)&ground_lines_size, sizeof(int));
		for (int j = 0; j < ground_lines_size; j++)
		{
			float val[4];
			MapDBloader.read((char*)&val[0], sizeof(float));
			MapDBloader.read((char*)&val[1], sizeof(float));
			MapDBloader.read((char*)&val[2], sizeof(float));
			MapDBloader.read((char*)&val[3], sizeof(float));
			newNode->ground_lines.push_back(cv::Vec4f(val[0], val[1], val[2], val[3]));
		}

		int line_param_size;
		MapDBloader.read((char*)&line_param_size, sizeof(int));
		for (int j = 0; j < line_param_size; j++)
		{
			double val[3];
			MapDBloader.read((char*)&val[0], sizeof(double));
			MapDBloader.read((char*)&val[1], sizeof(double));
			MapDBloader.read((char*)&val[2], sizeof(double));
			newNode->line_param.push_back(cv::Point3d(val[0], val[1], val[2]));
		}

		int d_row, d_col, d_type;
		MapDBloader.read((char*)&d_row, sizeof(int));
		MapDBloader.read((char*)&d_col, sizeof(int));

		int d_size = d_row * d_col;
		if (d_size != 0)
		{
			MapDBloader.read((char*)&d_type, sizeof(int));
			if (d_type == CV_32F)
			{
				newNode->desc = cv::Mat(d_row, d_col, CV_32FC1);
				MapDBloader.read((char*)newNode->desc.data, sizeof(float) * d_row * d_col);
			}
			if (d_type == CV_64F)
			{
				newNode->desc = cv::Mat(d_row, d_col, CV_64FC1);
				MapDBloader.read((char*)newNode->desc.data, sizeof(double) * d_row * d_col);
			}
		}

		int g_row, g_col;
		MapDBloader.read((char*)&g_row, sizeof(int));
		MapDBloader.read((char*)&g_col, sizeof(int));
		int g_size = g_row * g_col;
		if (g_size != 0)
		{
			newNode->GroundImg = cv::Mat(g_row, g_col, CV_8UC3);
			MapDBloader.read((char*)newNode->GroundImg.data, sizeof(unsigned char) * g_size * 3);
		}

		DB.push_back(newNode);

		printf("%d %d %d\n", i, DB.back()->desc.rows, d_row);
	}

	MapDBloader.close();

	std::cout << std::to_string(DB_size) << " nodes are loaded" << std::endl;

	bool showAllGroundImg = false;
	if (showAllGroundImg)
	{
		for (int i = 0; i < (int)DB.size(); i++)
		{
			cv::Mat temp_plot = DB[i]->GroundImg.clone();
			cv::putText(temp_plot, cv::format("%d", i), cv::Point(15, 25), 0, 0.8, cv::Scalar(0, 255, 0));
			cv::putText(temp_plot, cv::format("%d", DB[i]->desc.rows), cv::Point(65, 25), 0, 0.8, cv::Scalar(0, 255, 0));
			cv::imshow("Ground Img", temp_plot);
			cv::waitKey(0);
		}
	}
}
void GroundLine::run(void)
{
	if (m_data == nullptr) return;

	if(m_markerPoseEstimater) m_markerPoseEstimater->run();

	if (m_data->get_flag(Mode::querying))
	{
		getNowNode(&tempNow);
		if (!m_data->get_flag(Mode::always_on))
			m_data->set_flag_off(Mode::querying);
	}

	if (m_data->get_flag(Mode::inserting))
	{
		if (tempNow != nullptr)
		{
			printf("insert : %d %d\n", (int)DB.size(), (int)tempNow->desc.rows);
			DB.push_back(tempNow);
		}		
		m_data->set_flag_off(Mode::FLAG::inserting);
	}
	
	if (m_data->get_flag(Mode::saveDB))
	{
		if (DB.size() != 0) save();
		m_data->set_flag_off(Mode::FLAG::saveDB);
	}
	
	if (m_data->get_flag(Mode::loadDB))
	{
		load();
		m_data->set_flag_off(Mode::FLAG::loadDB);
	}
	
	if (m_data->get_flag(Mode::reset))
	{
		clearDB();
		m_data->set_flag_off(Mode::FLAG::reset);
	}

	if (m_data->querySize > 0)
	{
		estimateQuery(tempNow);
		m_data->querySize = 0;
	}

	if (m_camera_data->get_flag(Mode::draw3))
	{
		if (Ground_line_plot_img.empty()) Ground_line_plot_img = cv::Mat(240, 320 * 3, CV_8UC3, cv::Scalar(0, 0, 0));
		cv::imshow("Ground Node line", Ground_line_plot_img);
		cv::waitKey(1);
	}
	else cv::destroyWindow("Ground Node line");

	if (m_camera_data->get_flag(Mode::draw4))
	{
		if (tempNow != nullptr) if(!tempNow->GroundImg.empty())
			Ground_fit_line_plot = tempNow->GroundImg.clone();

		if (Ground_fit_line_plot.empty())
			Ground_fit_line_plot = cv::Mat(m_grid_data->cgridHeight, m_grid_data->cgridWidth, CV_8UC1, cv::Scalar(0, 0, 0));

		cv::Mat plot_now = Ground_fit_line_plot.clone();
		cv::putText(plot_now, cv::format("%dms", m_ipc->get_iteration_time()), cv::Point(15, 25),
			0, 0.8, cv::Scalar(0, 255, 0), 2);

		cv::imshow("Ground fitting line", plot_now);
		cv::waitKey(1);
	}
	else
	{
		cv::destroyWindow("Ground fitting line");
		cv::destroyWindow("ground query");
		cv::destroyWindow("merged_img");
		cv::destroyWindow("ground result");
	}
}
void GroundLine::threadRun(void)
{
	while (1)
	{
		run();
		m_ipc->wait_IPC();
	}
}