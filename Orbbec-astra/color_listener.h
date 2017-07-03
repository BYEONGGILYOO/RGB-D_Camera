#pragma once
#include <astra/astra.hpp>
#include <cstdio>
#include <iostream>

class ColorListener :
	public astra::FrameListener
{
public:
	ColorListener();
	~ColorListener();
};

