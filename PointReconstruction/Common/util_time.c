/*

 add timer compute  23/02/2019 
*/
#if defined WIN32 || defined _WIN32 || defined __CYGWIN__
#include <Windows.h>
#include <stdint.h> // portable: uint64_t   MSVC: __int64 
#else  /* __linux__*/
//#include <bits/time.h>
#include <sys/time.h>
#endif
#include <time.h>
#include "util_time.h"
#include "log.h"


#define MS_PER_SEC      1000ULL     // MS = milliseconds
#define US_PER_MS       1000ULL     // US = microseconds
#define HNS_PER_US      10ULL       // HNS = hundred-nanoseconds (e.g., 1 hns = 100 ns)
#define NS_PER_US       1000ULL

#define HNS_PER_SEC     (MS_PER_SEC * US_PER_MS * HNS_PER_US)
#define NS_PER_HNS      (100ULL)    // NS = nanoseconds
#define NS_PER_SEC      (MS_PER_SEC * US_PER_MS * NS_PER_US)


#if defined WIN32 || defined _WIN32
/*get from https://stackoverflow.com/questions/10905892/equivalent-of-gettimeday-for-windows*/
int gettimeofday(struct timeval * tp, struct timezone * tzp)
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

int clock_gettime_monotonic(struct timespec *tv)
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

int clock_gettime_realtime(struct timespec *tv)
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

int clock_gettime(int type, struct timespec *tp)
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

void CurTime(time_count *ptime)
{
#ifdef GET_HIGH_PRECISE_TIME
	struct timespec tval;
	if (clock_gettime(CLOCK_MONOTONIC,&tval) == -1)
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
	ptime->nsecs = (int)tval.tv_usec*NS_PER_US;
#endif
	return;
}

time_count timing_count(void)
{
	time_count TP1;
    CurTime(&TP1);
	return TP1;
}

int getCurNanoSecs(time_count tp1)
{
	time_count tvals;
	CurTime(&tvals);
	return (int)((tvals.secs - tp1.secs) * NS_PER_SEC + (tvals.nsecs - tp1.nsecs));
}


float getCurMicroSecs(time_count tp1)
{
	time_count tvals;
	CurTime(&tvals);
	return (float)(((tvals.secs - tp1.secs)*MS_PER_SEC*US_PER_MS) + ((tvals.nsecs - tp1.nsecs) / (NS_PER_US*1.0f)));
}

float getCurMillSecs(time_count tp1)
{
	time_count tvals;
	CurTime(&tvals);
	return (float)(((tvals.secs - tp1.secs)*MS_PER_SEC) + ((tvals.nsecs - tp1.nsecs) / (NS_PER_US*US_PER_MS*1.0f)));
}

float getCurSecs(time_count tp1)
{
	time_count tvals;
	CurTime(&tvals);
	return (float)((tvals.secs - tp1.secs) + ((tvals.nsecs - tp1.nsecs) / (NS_PER_SEC*1.0f)));
}



