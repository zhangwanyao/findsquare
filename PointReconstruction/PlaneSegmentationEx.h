#ifndef PLANE_SEGMENTATIONEX_H
#define PLANE_SEGMENTATIONEX_H
#include "PlaneSegmentation.h"
#include "PlaneSegmentationInterface.h"
#include <iostream>

#ifdef _WIN32
#ifdef SEG_DLL_API_EXPORTS
#define DLL_API __declspec(dllexport)
#else
#define DLL_API __declspec(dllimport)
#endif
#else
#define DLL_API
#endif

DLL_API bool FitPlaneExt(std::vector<cv::Point3f> &scene_data, std::vector<unsigned char>  &cene_reflect, PlaneSegResultInterface & plane_output);
#endif
