#ifndef PLANE_CUTTINGEX_H
#define PLANE_CUTTINGEX_H
#include "CutScanPlane/operations/script_roi.h"
#include "PlaneCuttingInterface.h"
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

DLL_API bool CutPlaneExt(MEASUREMENT_MODE mode, PlaneCutResultInterface & plane_info, bool withInclinedPlane,int station_size, bool cut_poly);
#endif
