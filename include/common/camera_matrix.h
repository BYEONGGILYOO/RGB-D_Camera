/*
* =====================================================================================
*
*       Filename:  camera_matrix.h
*
*    Description:
*
*        Version:  1.0
*        Created:  2017 . 06 . 29 , 23 : 52 : 28
*       Revision:  none
*       Compiler:  VS2015
*
*         Author:  BYEONGGILYOO
*   Organization: Hanyang Univ.
*
* =====================================================================================
*/
#ifndef CAMERA_MATRIX_H
#define CAMERA_MATRIX_H

#include <iostream>
#include <string>
#include "opencv2/core.hpp"
#include <Eigen/LU>

using Eigen::MatrixXd;

// basic camera matrix functions
void project_point_to_pixel(float pixel[2], const struct _intrinsics_ * intrin, const float point[3], bool coeffs);
void deproject_pixel_to_point(float point[3], const struct _intrinsics_* intrin, const float pixel[2], float depth, bool coeffs);
void transform_point_to_point(float to_point[3], const struct _extrinsics_ * extrin, const float from_point[3]);
void transform_point_to_point(float to_point[3], float rotation[9], float translation[3], const float from_point[3]);
void calculate_depth_to_color_matrix(const struct _rgbd_parameters_* data, float* transform_matrix);
void calculate_depth_to_color_matrix(const struct _rgbd_parameters_* data, double* transform_matrix);

// utility fuction
bool makeParametersYaml(const std::string full_path);
bool printAllParametersYaml(const std::string full_path);
bool writeParametersYaml(const std::string full_path, const struct _rgbd_parameters_ * data);
bool readParameterYaml(const std::string full_path, struct _rgbd_parameters_ * data);
bool writeDepthCorrectionFactorsYaml(const std::string full_path, const double dmcf, const double dacf);
bool readDepthCorrectionFactorsYaml(const std::string full_path, const float *dmcf, const float *dacf);
bool writeExtrinsicParametersYaml(const std::string full_path, const cv::Mat& R, const cv::Mat& t);
bool readExtrinsicParametersYaml(const std::string full_path, cv::Mat& R, cv::Mat& t);
bool writeCam2GroundRt(const std::string full_path, const cv::Mat& R, const cv::Mat& t);
bool readCam2GroundRt(const std::string full_path, cv::Mat& R, cv::Mat& t);
void readCameraOrder(std::string full_path, std::string& camera_name, std::vector<std::string>& cam_order, int& ref_cam_idx);


// basic camera parameters
struct float2 { float x, y; };
struct float3 { float x, y, z; };

typedef struct _intrinsics_
{
	int width, height;
	float ppx, ppy, fx, fy;
	float coeffs[5];
	_intrinsics_();
	_intrinsics_(const _intrinsics_& intr);
}Intrinsics;

typedef struct _extrinsics_
{
	float rotation[9];
	float translation[3];
	_extrinsics_();
	_extrinsics_(const _extrinsics_& ext);
}Extrinsics;

// rgbd camera parameters
typedef struct _rgbd_intrinsics_ : Intrinsics
{
	inline float2 project(const float3& point, bool coeffs = false) const
	{
		float2 pixel = {};
		project_point_to_pixel(&pixel.x, this, &point.x, coeffs);
		return pixel;
	}
	inline float3 deproject(const float2& pixel, float depth, bool coeffs = false) const
	{
		float3 point = {};
		deproject_pixel_to_point(&point.x, this, &pixel.x, depth, coeffs);
		return point;
	}
}RGBD_Intrinsics;

typedef struct _rgbd_extrinsics_ : Extrinsics
{
	bool isIdentity() const;
	inline float3 transform(const float3& point) const
	{
		float3 p = {};
		transform_point_to_point(&p.x, this, &point.x);
		return p;
	}
}RGBD_Extrinsics;

typedef struct _rgbd_parameters_
{
	std::string cam_id;
	RGBD_Intrinsics color_intrinsic;
	RGBD_Intrinsics depth_intrinsic;
	RGBD_Extrinsics depth_to_color;
	float depth_slope, depth_offset;

	_rgbd_parameters_();
	_rgbd_parameters_(const _rgbd_parameters_& rparam);
	inline void get_depth2color_all_matrix(float* dst) const
	{
		calculate_depth_to_color_matrix(this, dst);
	}
	inline void get_depth2color_all_matrix(double* dst) const
	{
		calculate_depth_to_color_matrix(this, dst);
	}
}RGBD_Parameters;

typedef struct _camera_raw_data_
{
	std::vector<cv::Mat> colorData, depthData;
	std::vector<double> colorTime, depthTime;
	_rgbd_parameters_ rgbdParam;

	_camera_raw_data_();
	_camera_raw_data_(const _camera_raw_data_& crd);
}Data;

#endif /* COMMON_FUNCTIONS_H_ */