/* Put any C/C++ code in here which you want as a separate library */
#pragma once

extern "C" {
	#define MAX_EXT_API_CONNECTION 255
	#include "extApi.h"
	#include "extApi.c"
	#include "extApiPlatform.h"
	#include "extApiPlatform.c"
}

#include <YouBotSIM.hpp>
