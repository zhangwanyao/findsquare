#include "PlaneSegmentation.h"
#include "log.h"
#include "util_plane_seg.hpp"
#include "plane_seg_api.h"

using namespace planeSegmentationInLine;

void* Features_PlaneSegCreateHandle()
{
	return (new PlaneSegmentation());
}

void Features_PlaneSegDestroyHandle(void** handle)
{
	if (handle)
	{
		PlaneSegmentation* ptr = reinterpret_cast<PlaneSegmentation*>(*handle);
		delete ptr;
		ptr = nullptr;
	}
}

bool Features_FitPlane(void* handle, const Vector<Point3f>input_points, const VoxelParams voxel_param, NormalEstimationResults *inputStruct,\
	std::vector<PlaneSegOutput>& out_planes)
{
	PlaneSegmentation *plane_seg = reinterpret_cast<PlaneSegmentation*>(handle);
	NormalEstimationInterface normal_input;
	Features_PlaneSegInputConvert(input_points,voxel_param,inputStruct,&normal_input);
	PlaneSegmentationOutput plane_seg_out;
	plane_seg->FitPlane("\\", normal_input, &plane_seg_out);
	Features_PlaneSegFreeMemory(normal_input);
	Features_PlaneSegOutputConvert(input_points,&plane_seg_out,out_planes);
	Features_PlaneSegOutFreeMemory(&plane_seg_out);
	return true;
}

bool Features_FitPlane(void* handle, const std::vector<Point3f>input_points, const VoxelParams voxel_param,\
	NormalEstimationResults* inputStruct, std::vector<PlaneSegOutput>& out_planes, const std::string config_file)
{
	PlaneSegmentation* plane_seg = reinterpret_cast<PlaneSegmentation*>(handle);
	NormalEstimationInterface normal_input;
	Features_PlaneSegInputConvert(input_points, voxel_param, inputStruct, &normal_input);
	PlaneSegmentationOutput plane_seg_out;

	plane_seg->FitPlane("\\", config_file, normal_input, &plane_seg_out);
	Features_PlaneSegFreeMemory(normal_input);

	Features_PlaneSegOutputConvert(input_points,&plane_seg_out, out_planes);
	Features_PlaneSegOutFreeMemory(&plane_seg_out);

	return true;
}

 bool Features_FitPlane(void* handle, const std::string output_path, const std::vector<Point3f>input_points, VoxelParams voxel_param, \
	NormalEstimationResults * inputStruct, std::vector<PlaneSegOutput>& out_planes,\
	 const SysCntrlParams sys_control_param, const SegCntrlParams seg_control_param, const PlaneFitThresholds plane_fit_thrshld)
{
	PlaneSegmentation *plane_seg = reinterpret_cast<PlaneSegmentation*>(handle);
	plane_seg->SetConfigure(sys_control_param);
	plane_seg->SetConfigure(seg_control_param);
	plane_seg->SetConfigure(plane_fit_thrshld);
	plane_seg->SetConfigure(voxel_param);

	NormalEstimationInterface normal_input;
	Features_PlaneSegInputConvert(input_points, voxel_param, inputStruct, &normal_input);

	PlaneSegmentationOutput plane_seg_out;
	plane_seg->FitPlaneMethod(seg_control_param.filtering_strtgy_type, output_path, normal_input, &plane_seg_out);
	Features_PlaneSegFreeMemory(normal_input);

	Features_PlaneSegOutputConvert(input_points,&plane_seg_out, out_planes);
	Features_PlaneSegOutFreeMemory(&plane_seg_out);
	return true;
}


#ifdef SAVE_OUTPUT_FILE_DEBUG
 bool Features_FitPlane(void* handle, const std::string output_path, const std::vector<Point3f>input_points, const VoxelParams voxel_param, \
	 NormalEstimationResults* inputStruct, std::vector<PlaneSegOutput>& out_planes, const SysCntrlParams sys_control_param, const SegCntrlParams seg_control_param, \
	 const PlaneFitThresholds plane_fit_thrshld, const DebugConfigParams dbg_param)
 {
	 PlaneSegmentation* plane_seg = reinterpret_cast<PlaneSegmentation*>(handle);
	 plane_seg->SetConfigure(sys_control_param);
	 plane_seg->SetConfigure(seg_control_param);
	 plane_seg->SetConfigure(plane_fit_thrshld);
	 plane_seg->SetConfigure(voxel_param);
	 plane_seg->SetConfigure(dbg_param);

	 NormalEstimationInterface normal_input = {};
	 Features_PlaneSegInputConvert(input_points, voxel_param, inputStruct, &normal_input);

	 PlaneSegmentationOutput plane_seg_out;
	 plane_seg->FitPlaneMethod(seg_control_param.filtering_strtgy_type, output_path, normal_input, &plane_seg_out);

	 if (dbg_param.missing_point_output) plane_seg->MissingPointsOutput(output_path, plane_seg_out);
	 if (seg_control_param.filtering_strtgy_type != NO_NORMALEST) Features_PlaneSegFreeMemory(normal_input);

	 Features_PlaneSegOutputConvert(input_points ,&plane_seg_out, out_planes);
	 Features_PlaneSegOutFreeMemory(&plane_seg_out);
	 return true;
}

#endif
