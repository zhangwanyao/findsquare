#ifndef _PLANE_SEGMENTATION_API_
#define _PLANE_SEGMENTATION_API_

#if defined WIN32 || defined _WIN32 || defined __CYGWIN__
#define PLANE_SEG_DLL_IMPORT __declspec(dllimport)
#define PLANE_SEG_DLL_EXPORT __declspec(dllexport)
#else
#define PLANE_SEG_DLL_IMPORT
#define PLANE_SEG_DLL_EXPORT
#endif
#include "NormalStruct.hpp"
#include "common_struct.h"
#include "plane_seg_inf.h"

/**
* \brief  Interface of plane segmentation, create instance of plane segmentation
*/
PLANE_SEG_DLL_EXPORT void* Features_PlaneSegCreateHandle();

/**
* \brief  Interface of plane segmentation, distroy instance of plane segmentation
* @param [in]  void** handle, plane segmentation handle
*/
PLANE_SEG_DLL_EXPORT void Features_PlaneSegDestroyHandle(void** handle);

/**
* \brief  Interface of plane segmentation using default configure parameters
* @param [in]  NormalEstimationInterface *inputStruct,  get from NormalEstimation module output
* @param [out]  PlaneSegmentationOutput* plane_seg_out, plane output
* @return if success or not
* */
//PLANE_SEG_DLL_EXPORT bool Features_FitPlane(void* handle, const PointArray input_points, const VoxelParams voxel_param, NormalEstimationInterface *inputStruct, PlaneSegmentationOutput* plane_seg_out);
PLANE_SEG_DLL_EXPORT bool Features_FitPlane(void* handle, const Vector<Point3f>input_points, const VoxelParams voxel_param, NormalEstimationResults* inputStruct, \
	Vector<PlaneSegOutput>& out_planes);

/**
* \brief  Interface of plane segmentation using file configure
* @param [in]  void* handle, plane segmentation handle
* @param [in]  const Vector<Point3f>input_points, plane segmentation input data
* @param [in]  VoxelParams voxel_param plane segmentation voxel size
* @param [in]  NormalEstimationInterface *inputStruct,  get from NormalEstimation module output
* @param [out]  PlaneSegmentationOutput* plane_seg_out, plane output
* @param [in] std::string config_file, input configure file
* @return if success or not
* */
PLANE_SEG_DLL_EXPORT bool Features_FitPlane(void* handle, const Vector<Point3f>input_points, const VoxelParams voxel_param, \
	NormalEstimationResults* inputStruct, Vector<PlaneSegOutput>& out_planes, const std::string config_file);

/**
* \brief  Interface of plane segmentation with input configure parameters
* @param [in]  void* handle, plane segmentation handle
* @param [in]  const Vector<Point3f>input_points, plane segmentation input data
* @param [in]  VoxelParams voxel_param plane segmentation voxel size
* @param [in]  NormalEstimationInterface *inputStruct,  get from NormalEstimation module output
* @param [out]  PlaneSegmentationOutput* plane_seg_out, plane segmentation output
* @param [in]  SysCntrlParams seg_control_param,  system control parameters
* @param [in]  SegCntrlParams seg_control_param, plane segmentation control parameters
* @param [in]  PlaneFitThresholds plane_fit_thrshld,  plane segmentation threshold value
* @return if success or not
* */
//PLANE_SEG_DLL_EXPORT bool Features_FitPlane(void* handle, const PointArray input_points, NormalEstimationInterface* inputStruct, PlaneSegmentationOutput* plane_seg_out, SysCntrlParams sys_control_param, \
//	SegCntrlParams seg_control_param, PlaneFitThresholds plane_fit_thrshld, VoxelParams voxel_param);
PLANE_SEG_DLL_EXPORT bool Features_FitPlane(void* handle, const std::string output_path, const Vector<Point3f>input_points, VoxelParams voxel_param, \
	NormalEstimationResults* inputStruct, Vector<PlaneSegOutput>& out_planes, \
	const SysCntrlParams sys_control_param, const SegCntrlParams seg_control_param, const PlaneFitThresholds plane_fit_thrshld);

/**
* \brief debug interface of plane segmentation with input configure parameters
* @param [in]  void* handle, plane segmentation handle
* @param [in]  std::string output_path, debug information output path
* @param [in]  const Vector<Point3f>input_points, plane segmentation input data
* @param [in]  VoxelParams voxel_param plane segmentation voxel size
* @param [in]  NormalEstimationInterface *inputStruct,  get from NormalEstimation module output
* @param [out] PlaneSegmentationOutput* plane_seg_out, plane segmentation output
* @param [in]  SysCntrlParams seg_control_param,  system control parameters
* @param [in]  SegCntrlParams seg_control_param, plane segmentation control parameters
* @param [in]  PlaneFitThresholds plane_fit_thrshld,  plane segmentation threshold value
* @param [in]  DebugConfigParams dbg_param plane segmentation debug control parameters
* @return if success or not
* */
//PLANE_SEG_DLL_EXPORT bool Features_FitPlane(void* handle, const std::string output_path, const PointArray input_points, NormalEstimationInterface *inputStruct, PlaneSegmentationOutput *plane_seg_out, SysCntrlParams sys_control_param, \
//	SegCntrlParams seg_control_param, PlaneFitThresholds plane_fit_thrshld, VoxelParams voxel_param, DebugConfigParams dbg_param);
PLANE_SEG_DLL_EXPORT bool Features_FitPlane(void* handle, const std::string output_path, const Vector<Point3f>input_points, const VoxelParams voxel_param, \
	NormalEstimationResults* inputStruct, Vector<PlaneSegOutput>& out_planes, const SysCntrlParams sys_control_param, const SegCntrlParams seg_control_param, \
	const PlaneFitThresholds plane_fit_thrshld, const DebugConfigParams dbg_param);

#endif // !_PLANE_SEGMENTATION_API_
