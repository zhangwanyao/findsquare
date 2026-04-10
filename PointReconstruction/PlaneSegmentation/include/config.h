#ifndef CONFIG_H
#define CONFIG_H

#define DEBUG_INFO

#define _USE_MATH_DEFINES

// Calculate average normal_diff and plane_dist and assign to smallest_normal_diff and smallest_plane_dist in PlaneMergeItem
#define USE_AVG_NORMAL_DIFF

// input data and outpu data file relative path with target file
#if defined WIN32 || defined _WIN32 || defined __CYGWIN__
#define DATA_INPUT_PATH "../../../../data/input/"
#define DATA_OUTPUT_PATH "../../../../data/output/PlaneSegmentation/"
#elif __linux__
#define DATA_INPUT_PATH "../../../data/input/"
#define DATA_OUTPUT_PATH "../../../../data/output/PlaneSegmentation/"
#else
#define DATA_INPUT_PATH "../../../../data/input/"
#define DATA_OUTPUT_PATH "../../../../data/output/PlaneSegmentation/"
#endif

#ifdef DEBUG_INFO
#define IO_DEBUG_INFO
#define TIMING_DEBUG_INFO
//#define SAVE_OUTPUT_FILE_DEBUG
#endif

#define INCLUDE_POINTS_GROUP_MERGING

// the max number of planes the one bad voxel's points can be merged in ,the value must >= 1
#define MAX_NUM_OF_PLANE_IN_BAD_VOXEL   5

// merging pseudo bad voxels connected wtih good plane, if plane points cloud is two thick , this macro should not be defined
//#define INCLUDE_MERGING_PB_CONNECTED_WITH_GOOD_PLANE

#if defined WIN32 || defined _WIN32 || defined __CYGWIN__
#define DP_DLL_IMPORT __declspec(dllimport)
#define DP_DLL_EXPORT __declspec(dllexport)
#define DP_DLL_LOCAL
#else
#if __GNUC__ >= 4
#define DP_DLL_IMPORT __attribute__ ((visibility ("default")))
#define DP_DLL_EXPORT __attribute__ ((visibility ("default")))
#define DP_DLL_LOCAL  __attribute__ ((visibility ("hidden")))
#else
#define DP_DLL_IMPORT
#define DP_DLL_EXPORT
#define DP_DLL_LOCAL
#endif
#endif

#endif//CONFIG_H
