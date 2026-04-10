#ifndef LINE_EXTRACT_MACRO_H
#define LINE_EXTRACT_MACRO_H

#if defined WIN32 || defined _WIN32 || defined __CYGWIN__
#define DATA_INPUT_PATH "../../../../data/input/"
#define DATA_OUTPUT_PATH "../../../../data/output/"
#define LINE_EXTRACT_INPUT_PATH "../../../../data/input/LineDetection/"
#define LINE_EXTRACT_OUTPUT_PATH "../../../../data/output/LineDetection/"
#elif __linux__
#define DATA_INPUT_PATH "../../../data/input/"
#define DATA_OUTPUT_PATH "../../../../data/output/"
#define LINE_EXTRACT_INPUT_PATH "../../../data/input/LineDetection/"
#define LINE_EXTRACT_OUTPUT_PATH "../../../../data/output/LineDetection/"
#else
#define DATA_INPUT_PATH "../../../../data/input/"
#define DATA_OUTPUT_PATH "../../../../data/output/"
#define LINE_EXTRACT_INPUT_PATH "../../../../data/input/LineDetection/"
#define LINE_EXTRACT_OUTPUT_PATH "../../../../data/output/LineDetection/"
#endif

//#define TIMING_DEBUG_INFO
#define SAVE_OUTPUT_FILE_DEBUG

#define LINE_EXTRACT_MODULE

#endif//LINE_EXTRACT_MACRO_H

