#pragma once

#include <cassert>
#include <cstdarg>
#include <stdio.h>

//Add actual implementation with an assertion handler
#define BP_ASSERT(expr) \
	do { \
		if (!(expr)) \
						{ \
			assert(expr); \
						} \
			} while (0) \

#define BP_ASSERTf(expr, ...) \
	do { \
		if (!(expr)) \
		{ \
			char formattedMsg[1024]; \
			sprintf_s(formattedMsg, sizeof(formattedMsg), __VA_ARGS__); \
			assert(expr && formattedMsg); \
		} \
	} while (0) 