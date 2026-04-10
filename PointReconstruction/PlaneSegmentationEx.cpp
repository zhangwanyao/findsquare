#pragma once
#include "PlaneSegmentationEx.h"
#include "PlaneSegmentationInterface.h"
#include "config.h"

#include <opencv2/core/core.hpp>
#include <string>
#include <ctime>
#include <sstream>
#include <omp.h>
#include "InOutData.h"
#include "log.h"
#include <iomanip>
#include "plane_seg_api.h"
#include <codecvt> //ut8
#include "MathOperation.hpp"
#include "DownSampleFilter.hpp"
#include "in_out_data.hpp"
#include "util_time.hpp"
//#include "Open3dIO.h"
#include "util_log.hpp"
#include "util_UNRE.hpp"
#include "util_normalEst.hpp"
#include "util_plane_fit.hpp"
bool FitPlaneExt(std::vector<cv::Point3f> &input_data, std::vector<unsigned char>  &cene_reflect, PlaneSegResultInterface & plane_output)
{
	try {

		std::string output_path, config_file_name;

		config_file_name = "config_room.ini";
		output_path = "dll_log//";

		TIMING_DECLARE(TP1)
		OUT_TIMING_BEGIN(TP1)
			//planeSegConfigParse config_it;
		planeSegCfgParse config_it;
		if (util_plane_seg::load_input_cfg(config_file_name, output_path, config_it) < 0)
			return false;

		SegCntrlParams* config_para = &config_it.getSegConfigParams();
		//FilterCntrlParams* filter_control_param = &config_it.getPlaneTestFilterControl();
		FilterCntrlParams* filter_control_param = &config_it.getFilterControl();
		OUT_TIMING_END_ms("GetConfig", TP1)

		OUT_TIMING_BEGIN(TP1)
		//Vector<cv::Point3f> input_data;
		std::vector<unsigned char> intensityVec;
		intensityVec.resize(cene_reflect.size());
		for (int i = 0; i < intensityVec.size(); i++)
			intensityVec[i] = cene_reflect[i];

		Vector<unsigned int> point_to_centriod_array;
		Vector<Point3f>input_data_filtered(0);

		Vector<Point3f> *input_data_ptr = &input_data;

		log_info("input data filtered size =%d", input_data_ptr->size());

		Vector<Point3f> downsample_points;
		std::vector<unsigned char> ds_intensityVec;
		SysCntrlParams sys_config_para = config_it.getSysConfigParams();

		OUT_TIMING_BEGIN(TP1)
		if ((filter_control_param->downsample_type == DOWNSAMPLE_WITH_ORG) || \
			(filter_control_param->downsample_type == DOWNSAMPLE_NO_ORG))
		{
			//TIMING_BEGIN(TP1)
			//std::string s_file_name = output_path + "input_src.txt";
			//IOData::SavePoint3fDataWithDelimiter(s_file_name, " ", *input_data_ptr);
			//TIMING_END_ms("Downsample file save", TP1)
			DownSampleFilter::FilterGridApply<Point3f, float, unsigned int>(*input_data_ptr, filter_control_param->downsample_leaf_size, downsample_points, point_to_centriod_array);
			log_info("input data downsample size =%d point_to_centriod_array size =%d", downsample_points.size(), point_to_centriod_array.size());

			ds_intensityVec.resize(downsample_points.size());
			log_info("ds_intensityVec size = %d intensityVec =%d", ds_intensityVec.size(), intensityVec.size());
			util_UNRE::SyncDownsampleIntensity(point_to_centriod_array, intensityVec, ds_intensityVec);

			//TIMING_BEGIN(TP1)
			//std::string file_name = output_path + "input_downsample.txt";
			//IOData::SavePoint3fDataWithDelimiter(file_name," ", downsample_points);
			//TIMING_END_ms("Downsample file save", TP1)
		}
		else
		{
			downsample_points = *input_data_ptr;
			ds_intensityVec.clear();
			ds_intensityVec.shrink_to_fit();
		}
		OUT_TIMING_END_ms("Downsample FilterGridApply", TP1)

		OUT_TIMING_BEGIN(TP1)
		NormalEstimationParamters normalEstimation_input;
		VoxelParams normal_voxel_size = {};
		NormalEstimationResults normal_result;
		//normalEstimation_input.points.assign(downsample_points.begin(), downsample_points.end());
		normalEstimation_input.points = &downsample_points;
		util_normalEst::get_normal_result(config_para->filtering_strtgy_type, config_it, normalEstimation_input, normal_voxel_size, normal_result);
		log_info("density from normal =%f ", normal_result.density);
		OUT_TIMING_END_ms("Get NormalEstimation", TP1)

			
		OUT_TIMING_BEGIN(TP1)
		Vector<PlaneSegOutput> plane_seg_out;
		Vector<PlaneSegOutput>plane_downsample_out;
		Vector<PlaneSegOutput> * fit_plane_seg_out = NULL;
		if ((filter_control_param->downsample_type == DOWNSAMPLE_WITH_ORG) || \
			(filter_control_param->downsample_type == DOWNSAMPLE_NO_ORG))
		{
			fit_plane_seg_out = &plane_downsample_out;
		}
		else
		{
			fit_plane_seg_out = &plane_seg_out;
		}

#ifdef SAVE_OUTPUT_FILE_DEBUG
		DebugConfigParams debug_config_para = config_it.getDebugConfigParams();
		void* plane_seg_handle = Features_PlaneSegCreateHandle();
		Features_FitPlane(plane_seg_handle, output_path, downsample_points, normal_voxel_size, &normal_result, \
			* fit_plane_seg_out, sys_config_para, *config_para, config_it.getPlaneSegThresholds(), debug_config_para);
#else
		void* plane_seg_handle = Features_PlaneSegCreateHandle();
		Features_FitPlane(plane_seg_handle, output_path, downsample_points, normal_voxel_size, &normal_result, \
			* fit_plane_seg_out, sys_config_para, *config_para, config_it.getPlaneSegThresholds());
#endif
		Features_PlaneSegDestroyHandle(&plane_seg_handle);
		OUT_TIMING_END_ms("FitPlane", TP1)

#ifdef PLANE_SEG_FOR_PLADE
			OUT_TIMING_BEGIN(TP1)
			util_plade_tool::SavePladeData(is_m_unit, output_path, input_data_file, downsample_points, fit_plane_seg_out);
		TIMING_END_ms("SavePladeData", TP1)
			log_file_close();
		return 1;
#endif

		OUT_TIMING_BEGIN(TP1)
		Vector<unsigned int> is_merged;
		if (filter_control_param->downsample_type == DOWNSAMPLE_WITH_ORG)
		{
			bool is_with_multiplane_pts = config_it.getSegConfigParams().filtering_strtgy_type & 8;
			log_info("is_with_multiplane_pts = %d", is_with_multiplane_pts);
			if (is_with_multiplane_pts)
			{
				util_plane_seg::SyncOrigPointToPlaneWithMPPts(*input_data_ptr, (unsigned int)downsample_points.size(), point_to_centriod_array, plane_downsample_out, plane_seg_out, is_merged);
			}
			else
			{
				util_plane_seg::SyncOrigPointToPlane(*input_data_ptr, (unsigned int)downsample_points.size(), point_to_centriod_array, plane_downsample_out, plane_seg_out, is_merged);
			}

		}
		else if (filter_control_param->downsample_type == DOWNSAMPLE_NO_ORG)
		{
			plane_seg_out = plane_downsample_out;
		}
		else
		{
		}
		//Add by simon
		//plane reflectance
		std::vector<std::vector<int>> plane_reflect;
		TIMING_END_ms("SyncOrigPointToPlane", TP1)

		plane_output.plane_xyz.resize(plane_seg_out.size());
		plane_output.plane_normals.resize(plane_seg_out.size());
		plane_output.plane_center.resize(plane_seg_out.size());
		plane_output.plane_reflect.resize(plane_seg_out.size());
		for (int i = 0; i < plane_seg_out.size(); i++)
		{
			PlaneSegOutput* in_p = &plane_seg_out[i];
			//PlaneSegOutput* out_p = &plane_seg_out[i];
			//plane_output[i].point_ids = in_p->point_ids;
			plane_output.plane_center[i] = in_p->plane_center;
			plane_output.plane_normals[i] = in_p->plane_normal;
			plane_output.plane_xyz[i].resize(in_p->points.size());
			plane_output.plane_reflect[i].resize(in_p->points.size());
			for (size_t j = 0; j < in_p->points.size(); j++)
			{
				size_t point_ids = in_p->point_ids[j];
				plane_output.plane_xyz[i][j].x = input_data[point_ids].x;
				plane_output.plane_xyz[i][j].y = input_data[point_ids].y;
				plane_output.plane_xyz[i][j].z = input_data[point_ids].z;
				plane_output.plane_reflect[i][j] = intensityVec[point_ids];
			}
		}
		/*OUT_TIMING_BEGIN(TP1)
		if (filter_control_param->downsample_type == DOWNSAMPLE_NO_ORG)
		{
			util_plane_seg::save_planes_file(filter_control_param->downsample_type, debug_config_para.missing_point_output, \
				output_path, downsample_points, plane_seg_out, is_merged);
		}
		else
		{
			util_plane_seg::save_planes_file(filter_control_param->downsample_type, debug_config_para.missing_point_output, \
				output_path, *input_data_ptr, plane_seg_out, is_merged);
		}
		OUT_TIMING_END("save_planes_file", TP1)

		OUT_TIMING_BEGIN(TP1)
		if (filter_control_param->downsample_type == DOWNSAMPLE_NO_ORG)
		{
			util_UNRE::save_planes_with_insten(output_path, downsample_points, plane_seg_out, ds_intensityVec);
		}
		else
		{
			util_UNRE::save_planes_with_insten(output_path, *input_data_ptr, plane_seg_out, intensityVec);
		}
		OUT_TIMING_END("save_planes_with_insten", TP1)
		*/

	}
	catch (const std::string err) {
		std::cerr << err << std::endl;
		log_fatal(err.c_str());
	}
	return true;
}