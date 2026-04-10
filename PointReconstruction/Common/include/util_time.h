#pragma once
#ifndef UTIL_TIME_H
#define UTIL_TIME_H
//#include "log.h"
//#define TIMING_DEBUG_INFO

typedef struct
{
	int secs; /*seconds*/
	int nsecs;  /*nanoseconds*/
} time_count;

#ifndef M_PI
#define M_PI       3.14159265358979323846   // pi
#endif

#define TIMING_DECLARE(X)  time_count X;

#ifdef TIMING_DEBUG_INFO
#define TIMING_BEGIN(X)   X=timing_count();

#define TIMING_END_us(MODULE_NAME, X) printf("%40s :\t%f us \n", MODULE_NAME,getCurMicroSecs(X));

#define TIMING_END_ms(MODULE_NAME, X) printf("%40s :\t%f ms \n", MODULE_NAME,getCurMillSecs(X));

#define TIMING_END(MODULE_NAME, X) printf("%40s :\t%f s \n", MODULE_NAME,getCurSecs(X));
#else
#define TIMING_BEGIN(X)

#define TIMING_END_us(MODULE_NAME, X)

#define TIMING_END_ms(MODULE_NAME, X)

#define TIMING_END(MODULE_NAME, X)
#endif

#define OUT_TIMING_BEGIN(X)   X=timing_count();

#define OUT_TIMING_END_us(MODULE_NAME, X) printf("***%40s :\t	%f us ***\n", MODULE_NAME,getCurMicroSecs(X));

#define OUT_TIMING_END_ms(MODULE_NAME, X) printf("***%40s :\t%f ms ***\n", MODULE_NAME,getCurMillSecs(X));

#define OUT_TIMING_END(MODULE_NAME, X) printf("***%40s :\t%f s ***\n", MODULE_NAME,getCurSecs(X));

#define GET_HIGH_PRECISE_TIME

#ifdef __cplusplus
extern "C" {
#endif

	time_count timing_count(void);
	//int getCurNanoSecs(time_count tp1);
	float getCurMicroSecs(time_count tp1);
	float getCurMillSecs(time_count tp1);
	float getCurSecs(time_count tp1);

#if defined WIN32 || defined _WIN32 || defined __CYGWIN__
	/*
	* The IDs of the various system clocks (for POSIX.1b interval timers):
	*/
#define CLOCK_REALTIME				0
#define CLOCK_MONOTONIC				1
#define CLOCK_PROCESS_CPUTIME_ID	2
#define CLOCK_THREAD_CPUTIME_ID		3
#define CLOCK_MONOTONIC_RAW			4
#define CLOCK_REALTIME_COARSE		5
#define CLOCK_MONOTONIC_COARSE		6
#define CLOCK_BOOTTIME				7
#define CLOCK_REALTIME_ALARM		8
#define CLOCK_BOOTTIME_ALARM		9
	int gettimeofday(struct timeval * tp, struct timezone * tzp);
	int clock_gettime(int type, struct timespec *tp);
#endif

#ifdef __cplusplus
}  // extern "C"
#endif

#endif
