#include "depth_listner.h"



DepthListner::DepthListner()
{
}


DepthListner::~DepthListner()
{
}

void DepthListner::on_frame_ready(astra::StreamReader & reader, astra::Frame & frame)
{
	const astra::DepthFrame depthFrame = frame.get<astra::DepthFrame>();

	if (depthFrame.is_valid())
	{
		print_depth(depthFrame,
			reader.stream<astra::DepthStream>().coordinateMapper());
		check_fps();
	}
}

void DepthListner::print_depth(const astra::DepthFrame & depthFrame, const astra::CoordinateMapper & mapper)
{
	if (depthFrame.is_valid())
	{
		int width = depthFrame.width();
		int height = depthFrame.height();
		int frameIndex = depthFrame.frame_index();

		//determine if buffer needs to be reallocated
		if (width != lastWidth_ || height != lastHeight_)
		{
			buffer_ = buffer_ptr(new int16_t[depthFrame.length()]);
			lastWidth_ = width;
			lastHeight_ = height;
		}
		depthFrame.copy_to(buffer_.get());

		size_t index = ((width * (height / 2.0f)) + (width / 2.0f));
		short middle = buffer_[index];

		float worldX, worldY, worldZ;
		float depthX, depthY, depthZ;
		mapper.convert_depth_to_world(width / 2.0f, height / 2.0f, middle, &worldX, &worldY, &worldZ);
		mapper.convert_world_to_depth(worldX, worldY, worldZ, &depthX, &depthY, &depthZ);

		std::cout << "depth frameIndex: " << frameIndex
			<< " value: " << middle
			<< " wX: " << worldX
			<< " wY: " << worldY
			<< " wZ: " << worldZ
			<< " dX: " << depthX
			<< " dY: " << depthY
			<< " dZ: " << depthZ
			<< std::endl;
	}
}

void DepthListner::check_fps()
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
