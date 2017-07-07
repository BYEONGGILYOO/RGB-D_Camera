#pragma once

#include <astra\astra.hpp>
#include <cstdio>
#include <chrono>
#include <iostream>
#include <iomanip>

#include "camera_matrix.h"

class Orbbec_astra : public astra::FrameListener
{
private:
	using buffer_ptr = std::unique_ptr<int16_t[]>;
	buffer_ptr buffer_;

	using duration_type = std::chrono::duration<double>;
	duration_type frameDuration_{ 0.0 };

	using clock_type = std::chrono::system_clock;
	std::chrono::time_point<clock_type> lastTimepoint_;

	RGBD_Parameters rgbd_param;
	
public:
	Orbbec_astra();
	~Orbbec_astra();

	virtual void on_frame_ready(astra::StreamReader& reader, astra::Frame& frame) override
	{
		const astra::DepthFrame depthFrame = frame.get<astra::DepthFrame>();

		if (depthFrame.is_valid())
		{
			rgbd_param.depth_intrinsic.width = depthFrame.width();
			rgbd_param.depth_intrinsic.height = depthFrame.height();

			depthFrame.copy_to(buffer_.get());

			
		}
	}

	void update_depth(astra::Frame& frame)
	{
		const astra::PointFrame point_frame = frame.get<astra::PointFrame>();

		if (!point_frame.is_valid())
		{
			
		}
	}

	void check_fps()
	{
		const double frameWeight = 0.2;

		auto newTimepoint = clock_type::now();
		auto frameDuration = std::chrono::duration_cast<duration_type>(newTimepoint - lastTimepoint_);

		frameDuration_ = frameDuration * frameWeight + frameDuration_ * (1 - frameWeight);
		lastTimepoint_ = newTimepoint;

		double fps = 1.0 / frameDuration_.count();

		auto precision = std::cout.precision();
		std::cout << std::fixed
			<< std::setprecision(1)
			<< fps << " fps ("
			<< std::setprecision(2)
			<< frameDuration.count() * 1000 << " ms)"
			<< std::setprecision(precision)
			<< std::endl;
	}
};

