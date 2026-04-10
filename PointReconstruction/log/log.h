/**
 * Copyright (c) 2017 rxi
 *
 * This library is free software; you can redistribute it and/or modify it
 * under the terms of the MIT license. See `log.c` for details.
 */
 
 #if defined WIN32 || defined _WIN32 || defined __CYGWIN__
#define LOG_DLL_IMPORT __declspec(dllimport)
#define LOG_DLL_EXPORT __declspec(dllexport)
#define LOG_DLL_LOCAL
#else
#if __GNUC__ >= 4
#define LOG_DLL_IMPORT __attribute__ ((visibility ("default")))
#define LOG_DLL_EXPORT __attribute__ ((visibility ("default")))
#define DP_DLL_LOCAL  __attribute__ ((visibility ("hidden")))
#else
#define LOG_DLL_IMPORT
#define LOG_DLL_EXPORT
#define LOG_DLL_LOCAL
#endif
#endif

 

#ifndef LOG_H
#define LOG_H

#include <stdio.h>
#include <stdarg.h>

#define LOG_VERSION "0.1.0"

LOG_DLL_EXPORT typedef void (*log_LockFn)(void *udata, int lock);


enum { LOG_TRACE, LOG_DEBUG, LOG_INFO, LOG_WARN, LOG_ERROR, LOG_FATAL };
enum RECONSTRUCTION_ERROR_CODE
{
	RECONSTRUCTION_SUCCESS = 30001,
	RECONSTRUCTION_LOAD_ERROR = 30002,
	RECONSTRUCTION_PLANE_FIT_ERROR = 30003,
	RECONSTRUCTION_PLANE_MISMATCH = 30004,
	RECONSTRUCTION_NORMAL_ERROR= 30005,
	RECONSTRUCTION_FAKE_CEILING_ERROR = 30006,
	RECONSTRUCTION_BOTTOM_CEILING_ERROR = 30007,
	RECONSTRUCTION_CREATE_JSON_ERROR = 30008,
};


#define log_trace(...) log_log(LOG_TRACE, __FILE__, __LINE__, __VA_ARGS__)
#define log_debug(...) log_log(LOG_DEBUG, __FILE__, __LINE__, __VA_ARGS__)
#define log_info(...)  log_log(LOG_INFO,  __FILE__, __LINE__, __VA_ARGS__)
#define log_warn(...)  log_log(LOG_WARN,  __FILE__, __LINE__, __VA_ARGS__)
#define log_error(...) log_log(LOG_ERROR, __FILE__, __LINE__, __VA_ARGS__)
#define log_fatal(...) log_log(LOG_FATAL, __FILE__, __LINE__, __VA_ARGS__)

#ifdef __cplusplus
extern "C" {
#endif

LOG_DLL_EXPORT void log_set_udata(void *udata);
LOG_DLL_EXPORT void log_set_lock(log_LockFn fn);
LOG_DLL_EXPORT void log_set_fp(FILE *fp);
LOG_DLL_EXPORT void log_set_level(int level);
LOG_DLL_EXPORT void log_set_quiet(int enable);
LOG_DLL_EXPORT void log_file_close();
LOG_DLL_EXPORT void log_log(int level, const char *file, int line, const char *fmt, ...);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif
