#ifndef _UTIL_TIME_HPP_
#define _UTIL_TIME_HPP_


#if defined WIN32 || defined _WIN32 || defined __CYGWIN__
#define NOMINMAX
#include <Windows.h>
#include <stdint.h>
#else  /* __linux__*/
//#include <bits/time.h>
#include <sys/time.h>
#endif

#include <time.h>
#include "../../log/log.h"

#define MS_PER_SEC      1000ULL     // MS = milliseconds
#define US_PER_MS       1000ULL     // US = microseconds
#define HNS_PER_US      10ULL       // HNS = hundred-nanoseconds (e.g., 1 hns = 100 ns)
#define NS_PER_US       1000ULL

#define HNS_PER_SEC     (MS_PER_SEC * US_PER_MS * HNS_PER_US)
#define NS_PER_HNS      (100ULL)    // NS = nanoseconds
#define NS_PER_SEC      (MS_PER_SEC * US_PER_MS * NS_PER_US)

 
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

		/*get from https://stackoverflow.com/questions/10905892/equivalent-of-gettimeday-for-windows*/
		static inline int gettimeofday(struct timeval* tp, struct timezone* tzp)
		{
			// Note: some broken versions only have 8 trailing zero's, the correct epoch has 9 trailing zero's
			// This magic number is the number of 100 nanosecond intervals since January 1, 1601 (UTC)
			// until 00:00:00 January 1, 1970 
			static const uint64_t EPOCH = ((uint64_t)116444736000000000ULL);

			SYSTEMTIME  system_time;
			FILETIME    file_time;
			uint64_t    time;

			GetSystemTime(&system_time);
			SystemTimeToFileTime(&system_time, &file_time);
			time = ((uint64_t)file_time.dwLowDateTime);
			time += ((uint64_t)file_time.dwHighDateTime) << 32;

			tp->tv_sec = (long)((time - EPOCH) / 10000000L);
			tp->tv_usec = (long)(system_time.wMilliseconds * 1000);
			return 0;
		}

		/*get from https://stackoverflow.com/questions/5404277/porting-clock-gettime-to-windows*/

		static inline int clock_gettime_monotonic(struct timespec* tv)
		{
			static LARGE_INTEGER ticksPerSec;
			LARGE_INTEGER ticks;
			double seconds;

			if (!ticksPerSec.QuadPart) {
				QueryPerformanceFrequency(&ticksPerSec);
				if (!ticksPerSec.QuadPart) {
					errno = ENOTSUP;
					return -1;
				}
			}
			QueryPerformanceCounter(&ticks);
			seconds = (double)ticks.QuadPart / (double)ticksPerSec.QuadPart;
			tv->tv_sec = (time_t)seconds;
			tv->tv_nsec = (long)((ULONGLONG)(seconds * NS_PER_SEC) % NS_PER_SEC);
			return 0;
		}

		static inline int clock_gettime_realtime(struct timespec* tv)
		{
			FILETIME ft;
			ULARGE_INTEGER hnsTime;

			GetSystemTimeAsFileTime(&ft);

			hnsTime.LowPart = ft.dwLowDateTime;
			hnsTime.HighPart = ft.dwHighDateTime;

			// To get POSIX Epoch as baseline, subtract the number of hns intervals from Jan 1, 1601 to Jan 1, 1970.
			hnsTime.QuadPart -= (11644473600ULL * HNS_PER_SEC);

			// modulus by hns intervals per second first, then convert to ns, as not to lose resolution
			tv->tv_nsec = (long)((hnsTime.QuadPart % HNS_PER_SEC) * NS_PER_HNS);
			tv->tv_sec = (long)(hnsTime.QuadPart / HNS_PER_SEC);

			return 0;
		}

		static inline int clock_gettime(int type, struct timespec* tp)
		{
			if (type == CLOCK_MONOTONIC)
			{
				return clock_gettime_monotonic(tp);
			}
			else if (type == CLOCK_REALTIME)
			{
				return clock_gettime_realtime(tp);
			}
			errno = ENOTSUP;
			return -1;
		}
#endif
		static inline void CurTime(time_count* ptime)
		{
#ifdef GET_HIGH_PRECISE_TIME
			struct timespec tval;
			if (clock_gettime(CLOCK_MONOTONIC, &tval) == -1)
			{
				log_fatal("CurTime(): clock_gettime return failed \n");
				return;
			}
			ptime->secs = (int)tval.tv_sec;
			ptime->nsecs = (int)tval.tv_nsec;

#else
			struct timeval tval;
			if (gettimeofday(&tval, NULL) == -1)
			{
				log_fatal("CurTime(): gettimeofday return failed \n");
				return;
			}
			ptime->secs = (int)tval.tv_sec;
			ptime->nsecs = (int)tval.tv_usec * NS_PER_US;
#endif
			return;
		}

		static inline time_count timing_count(void)
		{
			time_count TP1;
			CurTime(&TP1);
			return TP1;
		}

		static inline int getCurNanoSecs(time_count tp1)
		{
			time_count tvals;
			CurTime(&tvals);
			return (int)((tvals.secs - tp1.secs) * NS_PER_SEC + (tvals.nsecs - tp1.nsecs));
		}


		static inline float getCurMicroSecs(time_count tp1)
		{
			time_count tvals;
			CurTime(&tvals);
			return (float)(((tvals.secs - tp1.secs) * MS_PER_SEC * US_PER_MS) + ((tvals.nsecs - tp1.nsecs) / (NS_PER_US * 1.0f)));
		}

		static inline float getCurMillSecs(time_count tp1)
		{
			time_count tvals;
			CurTime(&tvals);
			return (float)(((tvals.secs - tp1.secs) * MS_PER_SEC) + ((tvals.nsecs - tp1.nsecs) / (NS_PER_US * US_PER_MS * 1.0f)));
		}

		static inline float getCurSecs(time_count tp1)
		{
			time_count tvals;
			CurTime(&tvals);
			return (float)((tvals.secs - tp1.secs) + ((tvals.nsecs - tp1.nsecs) / (NS_PER_SEC * 1.0f)));
		}

#ifdef __cplusplus
	}  // extern "C"
#endif

#endif
