#ifndef _UTIL_LOG_HPP_
#define _UTIL_LOG_HPP_

#include <string>
#include <iostream>
#include "log.h"

namespace util_log
{
	inline static void log_cfg_set(std::string file_name, FILE* log_file, int is_quiet, int level)
	{
#ifdef _WIN32
		errno_t err = fopen_s(&log_file, file_name.c_str(), "w+");
#else
		log_file = fopen(file_name.c_str(), "w+");
#endif
		if (!log_file)
		{
			std::cout << "Log file: cannot open file" << std::endl;
		}
		log_set_fp(log_file);
		log_set_quiet(is_quiet);
		log_set_level(level);
	}
}

#endif /*_UTIL_LOG_HPP_*/