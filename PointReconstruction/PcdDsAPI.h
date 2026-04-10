#pragma once
#ifndef PCDDSAPI_H
#define PCDDSAPI_H

#include <string>

#define WIN32

#ifdef WIN32
#define _PCDDS_API_EXPORT __declspec(dllexport)
#else
#define _PCDDS_API_EXPORT
#endif

/**
* @brief pcd downsampling
*
* @param inputPTSFileName pts file path in EXYZI encrypted format
* @param outputFileName output file path in EXYZI encrypted and downsampled format
* @param level downsample level, the higher means denser pointcloud
* @return true
* @return false
*/

extern "C" _PCDDS_API_EXPORT bool pcdIcoDownsample(
    const char* inputPTSFileName,
    const char* outputFileName,
    int level
);


#endif