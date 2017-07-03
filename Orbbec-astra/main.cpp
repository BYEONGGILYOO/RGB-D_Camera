#define _CRT_SECURE_NO_WARNINGS

#include <astra\astra.hpp>
#include <opencv2\opencv.hpp>
#include <iostream>

#include "orbbec_astra.h"

int main(int argc, char** argv)
{
	cv::Mat tmp;
	astra::initialize();
	astra::StreamSet streamSet;
	astra::StreamReader reader = streamSet.create_reader();

	Orbbec_astra orbbec;
	reader.stream<astra::DepthStream>().start();

	reader.add_listener(orbbec);

	while (1)
	{
		//astra_temp_update();
	}
	// terminate
	astra::terminate();
	std::cout << "hit enter to exit program" << std::endl;
	std::cin.get();
	return 0;
}