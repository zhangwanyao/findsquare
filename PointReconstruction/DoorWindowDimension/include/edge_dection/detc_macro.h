#ifndef DETC_MACRO_H
#define DETC_MACRO_H

#if defined WIN32 || defined _WIN32 || defined __CYGWIN__
#define DATA_INPUT_PATH "../../../../data/input/"
#define DATA_OUTPUT_PATH "../../../../data/output/"
#define DETC_EDGE2D_INPUT_PATH "../../../../data/input/detc_edge/"
#define DETC_EDGE2D_OUTPUT_PATH "../../../../data/output/detc_edge/"
#elif __linux__
#define DATA_INPUT_PATH "../../../data/input/"
#define DATA_OUTPUT_PATH "../../../../data/output/"
#define DETC_EDGE2D_INPUT_PATH "../../../data/input/detc_edge2d/"
#define DETC_EDGE2D_OUTPUT_PATH "../../../../data/output/detc_edge2d/"
#else
#define DATA_INPUT_PATH "../../../../data/input/"
#define DATA_OUTPUT_PATH "../../../../data/output/"
#define DETC_EDGE2D_INPUT_PATH "../../../../data/input/detc_edge2d/"
#define DETC_EDGE2D_OUTPUT_PATH "../../../../data/output/detc_edge2d/"
#endif

//#define TIMING_DEBUG_INFO
#define SAVE_OUTPUT_FILE_DEBUG

#define DETC_EDGE_MODULE

#endif//DETC_MACRO_H

