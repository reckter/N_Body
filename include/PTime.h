/*
 * PTime.h
 *
 *  Created on: Dec 15, 2014
 *      Author: ahueck
 *
 *  Function "now" used for accurate performance measurements
 *
 */

#ifndef PTIME_H_
#define PTIME_H_

#include <time.h>

#define HAS_CLOCK_GETTIME (_POSIX_C_SOURCE >= 199309L)

#ifdef WIN32
#include <windows.h>
#else
#if !HAS_CLOCK_GETTIME
#include <sys/time.h>
#endif
#endif

namespace practical {
namespace nbody {

#ifdef WIN32
static double frequency = 1.0;

inline void init() {
	// Call before now on Windows...
	LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);
	frequency = static_cast<double>(freq.QuadPart);
}
#endif

/**
 * returns time in seconds
 */
inline double now() {
	double time = 0.0;
#ifdef WIN32
	LARGE_INTEGER t_now;
	QueryPerformanceCounter(&t_now);
	time = static_cast<double>(t_now.QuadPart) / frequency; // seconds
#else
#if HAS_CLOCK_GETTIME
	timespec t_now;
	clock_gettime(CLOCK_MONOTONIC, &t_now);
	time = static_cast<double>(t_now.tv_sec)
			+ 1e-9 * static_cast<double>(t_now.tv_nsec);
#else
	timeval t_now;
	gettimeofday(&t_now, NULL);
	time = static_cast<double>(t_now.tv_sec)
	+ 1e-6 * static_cast<double>(t_now.tv_usec);
#endif
#endif
	return time;
}

} /* namespace nbody */
} /* namespace practical */

#endif /* PTIME_H_ */
