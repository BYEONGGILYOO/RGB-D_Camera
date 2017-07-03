#pragma once
#include <astra\astra.hpp>
#include <cstdio>
#include <chrono>
#include <iostream>
#include <iomanip>

class DepthListner :
	public astra::FrameListener
{
private:
	using buffer_ptr = std::unique_ptr<int16_t[]>;
	buffer_ptr buffer_;
	unsigned int lastWidth_;
	unsigned int lastHeight_;

public:
	DepthListner();
	~DepthListner();
	virtual void on_frame_ready(astra::StreamReader& reader, astra::Frame& frame) override;

	void print_depth(const astra::DepthFrame& depthFrame, const astra::CoordinateMapper& mapper);
	void check_fps();

private:
	using duration_type = std::chrono::duration < double >;
	duration_type frameDuration_{ 0.0 };

	using clock_type = std::chrono::system_clock;
	std::chrono::time_point<clock_type> lastTimepoint_;
	
};
