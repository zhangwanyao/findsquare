#ifndef _UTIL_NORMALEST_HPP_
#define _UTIL_NORMALEST_HPP_

#include "DataStruct.h"
#include "NormalEstimationAPI.hpp"
#include "CommonConfigParse.hpp"
#undef NORMALS_EXPORT
#define NORMALS_EXPORT __declspec(dllimport)

namespace util_normalEst
{
	static inline bool InputNormalEstimationParameters(NormalEstimationParamters* inputStruct, const NormalEstimationThresholds normal_thrsld_param, const VoxelParams voxel_params)
	{
		//inputStruct->threshold_max_normal_angle_of_two_voxel = static_cast<value_type>(plane_seg_thresholds.THRESHOLD_MAX_NORMAL_ANGLE_OF_TWO_VOXEL);
		//inputStruct->threshold_min_point_num_of_valid_normal_voxel = static_cast<value_type>(plane_seg_thresholds.THRESHOLD_MIN_POINT_NUM_OF_VALID_NORMAL_VOXEL);
		inputStruct->voxel_size_x = voxel_params.length_x_of_voxel;
		inputStruct->voxel_size_y = voxel_params.length_y_of_voxel;
		inputStruct->voxel_size_z = voxel_params.length_z_of_voxel;
		//inputStruct->threshold_min_mse_of_voxel = static_cast<value_type>(plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_VOXEL);
		inputStruct->threshold_max_normal_angle_of_two_voxel = normal_thrsld_param.max_normal_angle_of_2voxel;
		inputStruct->threshold_min_point_num_of_valid_normal_voxel = static_cast<size_t>(normal_thrsld_param.min_point_num_of_valid_normal_voxel);
		inputStruct->threshold_min_mse_of_voxel = normal_thrsld_param.max_mse_ratio_of_voxel * (voxel_params.length_x_of_voxel + voxel_params.length_y_of_voxel + voxel_params.length_z_of_voxel) / 3;
		log_info("threshold_max_normal_angle_of_two_voxel = %f threshold_min_point_num_of_valid_normal_voxel=%d threshold_min_mse_of_voxel=%f ", \
			inputStruct->threshold_max_normal_angle_of_two_voxel, inputStruct->threshold_min_point_num_of_valid_normal_voxel, inputStruct->threshold_min_mse_of_voxel);
		return true;
	}

	static inline void get_normal_result(int filter_type, FeatureCommonConfigParse config_it, NormalEstimationParamters &normalEstimation_input, VoxelParams &normal_voxel_size, NormalEstimationResults& normal_result)
	{
		if (filter_type != 2)
		{
			//NormalEstimationCntrlParam* normal_control_param = &config_it.getPlaneTestNormalEst();
			NormalEstimationCntrlParam* normal_control_param = &config_it.getNormalEstParams();
			InputNormalEstimationParameters(&normalEstimation_input, config_it.getNormalEstThrldParams(), config_it.getVoxelSize());
			//InputNormalEstimationParameters(&normalEstimation_input, config_it.getPlaneSegThresholds(), config_it.getVoxelSize());
			void* nomalsHandle = Features_NormalsCreateHandle();
			Features_NormalsSetParams(nomalsHandle, &normalEstimation_input);
			//normal_voxel_size.length_x_of_voxel = normalEstimation_input.voxel_size_x;
			//normal_voxel_size.length_y_of_voxel = normalEstimation_input.voxel_size_y;
			//normal_voxel_size.length_z_of_voxel = normalEstimation_input.voxel_size_z;
			if (normal_control_param->is_point_density_set)
			{
				normal_result = Features_NormalsCalculate(nomalsHandle, normal_control_param->is_voxel_size_detected, normal_control_param->point_density);
			}
			else
			{
				normal_result = Features_NormalsCalculate(nomalsHandle, normal_control_param->is_voxel_size_detected);
			}

			normal_voxel_size.length_x_of_voxel = normalEstimation_input.voxel_size_x;
			normal_voxel_size.length_y_of_voxel = normalEstimation_input.voxel_size_y;
			normal_voxel_size.length_z_of_voxel = normalEstimation_input.voxel_size_z;

			Features_NormalsDestroyHandle(&nomalsHandle);
		}
		else
		{
			normal_result.normals.clear();
			normal_result.normals.shrink_to_fit();
			normal_result.grid_to_occupied_voxel_idx.clear();
			normal_result.grid_to_occupied_voxel_idx.shrink_to_fit();
			normal_result.voxel_array.clear();
			normal_result.voxel_array.shrink_to_fit();
			normal_result.voxel_idx.clear();
			normal_result.voxel_idx.shrink_to_fit();
			normal_voxel_size = config_it.getVoxelSize();
		}
	}
}

#endif /*_UTIL_NORMALEST_HPP_*/