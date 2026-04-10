#ifndef _PLANE_CFG_HPP_
#define _PLANE_CFG_HPP_
#include "config.h"
#include "common_struct.h"
#include "plane_seg_inf.h"
#include "ConfigParse.hpp"
#include "CommonConfigParse.hpp"

class planeSegCfgParse : public FeatureCommonConfigParse
{
public:

	/**
	* \brief constructor with plane segmentation config
	*/
	planeSegCfgParse()
	{
		this->freeMemory();
		//sys_control_para_.is_cad = false;
		//sys_control_para_.scanner_type = INVALID_SCAN_TYPE;

		config_params_.identify_with_bridge = false;
		config_params_.filtering_strtgy_type = NO_FILTER;
		config_params_.seg_thrshld_section_valid = SEG_DEFAULT_CONFIG_PARAM;

		plane_seg_thresholds_.THRESHOLD_MIN_PLANE_DIST_OF_TWO_VOXEL = 8;
		plane_seg_thresholds_.THRESHOLD_MIN_DIST_OF_POINT2VOXEL = 20;
		plane_seg_thresholds_.THRESHOLD_MAX_NORMAL_ANGLE_OF_TWO_VOXEL = 10;
		plane_seg_thresholds_.THRESHOLD_MAX_MSE_OF_VOXEL = 6;
		plane_seg_thresholds_.THRESHOLD_MIN_POINT_NUM_OF_PLANE = 800;
		plane_seg_thresholds_.THRESHOLD_MIN_VOXEL_NUM_OF_PLANE = 5;
		plane_seg_thresholds_.THRESHOLD_MAX_MSE_OF_PLANE = 5;
		plane_seg_thresholds_.THRESHOLD_MIN_POINT_NUM_OF_VALID_NORMAL_VOXEL = 10;
		plane_seg_thresholds_.THRESHOLD_MIN_DIST_OF_2PLANE = 5;
		plane_seg_thresholds_.THRESHOLD_MAX_NORMAL_ANGEL_OF_2PLANE = 5;
		plane_seg_thresholds_.THRESHOLD_MAX_NORMAL_ANGEL_OF_EDGE_POINT = 45;
		plane_seg_thresholds_.THRESHOLD_MAX_HIGH_MSE_RATIO = 0.02f;
		plane_seg_thresholds_.THRESHOLD_MIN_NUM_OF_GOOD_GROUP_IN_PLANE = 3;
		plane_seg_thresholds_.THRESHOLD_MIN_POINT_NUM_OF_FLAT_VOXEL = 3;
		plane_seg_thresholds_.THRESHOLD_DIST_OF_2VOXEL_CONNECTED = 40;
		plane_seg_thresholds_.THRESHOLD_MIN_GOOD_PLANE_VOXEL_SIZE = 20;

		//voxel_params_.length_x_of_voxel = 120.f;
		//voxel_params_.length_y_of_voxel = 120.f;
		//voxel_params_.length_z_of_voxel = 120.f;
#ifdef SAVE_OUTPUT_FILE_DEBUG
		debug_config_params_.dbg_section_valid = false;
		debug_config_params_.reserved_test = 0;
		debug_config_params_.missing_point_output = false;
		debug_config_params_.io_debug_info = false;
		debug_config_params_.voxel_info_debug = false;
		debug_config_params_.neighbour_info_debug = false;
		debug_config_params_.no_plane_merge = false;
		debug_config_params_.plane_output_debug = false;
		debug_config_params_.merge_pseudobad_voxel_debug = false;
		debug_config_params_.merge_bad_voxel_debug = false;
		debug_config_params_.pseudo_bad_lost_plane_output_debug = false;
		debug_config_params_.is_dwnsmpl_output = false;
		debug_config_params_.points_group_neighbor_output_debug = false;
		debug_config_params_.merge_same_plane_output_debug = false;
		debug_config_params_.include_get_nm_by_file = false;
		debug_config_params_.nm_file_ouput_debug = false;
#endif
		this->init();
	}

	/**
	* \brief destructor
	*/
	~planeSegCfgParse()
	{
		this->freeMemory();
	}

	/**
	* \brief  Get  plane segmentation config parameters from config file(config.ini)
	* @param [in] const std::string config_ini, config.ini file
	*/
	inline bool GetPlaneSegConfigure(const std::string config_path)
	{
		if (!this->GetConfigMap(config_path))
		{
			log_error("GetConfigMap failed");
			return false;
		}

		//system control parameters from config
		ParseConfigure(sys_control_para_);

		//voxel size from config section VOXEL_SIZE_CONFIG
		ParseConfigure(voxel_params_);

		//get plane segmentation control parameters from config section SEG_CNTRL_PARA
		SyncPlaneSegConfigure(config_params_);

		float voxel_length = (voxel_params_.length_x_of_voxel + voxel_params_.length_y_of_voxel + voxel_params_.length_z_of_voxel) / 3;
		SyncPlaneSegConfigure(config_params_.seg_thrshld_section_valid, voxel_length, plane_seg_thresholds_);
		//get plane segmentation threshold parameters from section SEG_THRSHLD_CONFIG_DEBUG
		return true;
	}

	/**
	* \brief  Get  all the config parameters  for plane segmentation test  from config file(config.ini)
	* @param [in] const std::string config_path, config.ini file
	*/
	inline bool GetPlaneTestConfigure(const std::string config_path)
	{
		if (!this->GetConfigMap(config_path))
		{
			log_error("GetConfigMap failed");
			return false;
		}

		//system control parameters from config
		ParseConfigure(sys_control_para_);

		//voxel size from config section VOXEL_SIZE_CONFIG
		ParseConfigure(voxel_params_);

		ParseConfigure(sys_dbg_para);
		//get plane segmentation control parameters from config section SEG_CNTRL_PARA
		SyncPlaneSegConfigure(config_params_);

		SyncCommonConfigure(filter_control_para);

		SyncCommonConfigure(normalEst_control_para);

		SyncCommonConfigure(normalEst_thrld_para);

		//get plane segmentation threshold parameters from section SEG_THRSHLD_CONFIG or SEG_THRSHLD_CONFIG_DEBUG
		float voxel_length = (voxel_params_.length_x_of_voxel + voxel_params_.length_y_of_voxel + voxel_params_.length_z_of_voxel) / 3;
		SyncPlaneSegConfigure(config_params_.seg_thrshld_section_valid, voxel_length, plane_seg_thresholds_);

		//SyncPlaneTestConfigure(filter_control_para);

		//SyncPlaneTestConfigure(normalEst_control_para);

		return true;
	}

	/**
	* \brief sync  plane segmentation config parameters from config section SEG_CNTRL_PARA
	* @param [out] SegCntrlParams &config_params Plane segmentation parameters
	*/
	inline void SyncPlaneSegConfigure(SegCntrlParams& config_params)
	{
		int tmp_i = std::numeric_limits<int>::lowest();

		bool sync_con = ReadConfigInt("SEG_CNTRL_PARA", "identify_with_bridge", tmp_i);
		if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) config_params.identify_with_bridge = (tmp_i == 0) ? false : true;

		sync_con = ReadConfigInt("SEG_CNTRL_PARA", "filtering_strtgy_type", tmp_i);
		if ((sync_con) && (tmp_i < FILER_TYPE_MAX) && (tmp_i >= 0)) config_params.filtering_strtgy_type = (planeSegFilterType)tmp_i;

		sync_con = ReadConfigInt("SEG_CNTRL_PARA", "seg_thrshld_section_valid", tmp_i);
		if ((sync_con) && (tmp_i < SEG_CONFIG_PARAM_MAX) && (tmp_i >= 0)) config_params.seg_thrshld_section_valid = tmp_i;
	}

	/**
	* \brief  sync  plane segmentation threshold parameters from section SEG_THRSHLD_CONFIG or SEG_THRSHLD_CONFIG_DEBUG
	* @param [in] bool is_valid  if true show this section is valid otherwise is invalid
	* @param [in] PlaneFitThresholds &plane_seg_th plane segmentation threshold
	*/
	inline void SyncPlaneSegConfigure(unsigned int option_section, float voxel_length, PlaneFitThresholds& plane_seg_th)
	{
		int tmp_i = std::numeric_limits<int>::lowest();
		float tmp_f = std::numeric_limits<float>::infinity();

		//PlaneFitThresholds parse
		if (option_section == SEG_CONFIG_PARAM_VALID)
		{
			const char* section = "SEG_THRSHLD_CONFIG";
			bool sync_con = ReadConfigInt(section, "min_voxel_num_of_plane", tmp_i);
			if ((sync_con) && (tmp_i >= 0)) plane_seg_th.THRESHOLD_MIN_VOXEL_NUM_OF_PLANE = tmp_i;

			sync_con = ReadConfigInt(section, "min_point_num_of_plane", tmp_i);
			if ((sync_con) && (tmp_i >= 0)) plane_seg_th.THRESHOLD_MIN_POINT_NUM_OF_PLANE = tmp_i;

			sync_con = ReadConfigInt(section, "min_point_num_of_flat_voxel", tmp_i);
			if ((sync_con) && (tmp_i >= 0)) plane_seg_th.THRESHOLD_MIN_POINT_NUM_OF_FLAT_VOXEL = tmp_i;

			sync_con = ReadConfigFloat("SEG_THRSHLD_CONFIG", "max_mse_ratio_of_voxel", tmp_f);
			if ((sync_con) && (tmp_f <= 1))
			{
				plane_seg_th.THRESHOLD_MAX_MSE_OF_VOXEL = tmp_f * voxel_length;
			}

			sync_con = ReadConfigFloat(section, "max_mse_ratio_of_plane", tmp_f);
			if ((sync_con) && (tmp_f <= 1))
			{
				plane_seg_th.THRESHOLD_MAX_MSE_OF_PLANE = tmp_f * voxel_length;
			}

			sync_con = ReadConfigFloat(section, "min_dist_ratio_of_2plane", tmp_f);
			if ((sync_con) && (tmp_f <= 1))
			{
				plane_seg_th.THRESHOLD_MIN_DIST_OF_2PLANE = tmp_f * voxel_length;
			}

			sync_con = ReadConfigFloat(section, "max_dist_ratio_of_connected", tmp_f);
			if ((sync_con) && (tmp_f <= 1))
			{
				plane_seg_th.THRESHOLD_DIST_OF_2VOXEL_CONNECTED = tmp_f * voxel_length;
			}

			// some default parameter according to scanner
			if (sys_control_para_.scanner_type == UNRE_TYPE_0)
			{
				plane_seg_th.THRESHOLD_MIN_PLANE_DIST_OF_TWO_VOXEL = 0.25f * voxel_length;
				plane_seg_th.THRESHOLD_MIN_DIST_OF_POINT2VOXEL = 0.375f * voxel_length;
				plane_seg_th.THRESHOLD_MAX_NORMAL_ANGLE_OF_TWO_VOXEL = 10;
				plane_seg_th.THRESHOLD_MIN_POINT_NUM_OF_VALID_NORMAL_VOXEL = 5;
				plane_seg_th.THRESHOLD_MAX_NORMAL_ANGEL_OF_2PLANE = 10;
				plane_seg_th.THRESHOLD_MAX_NORMAL_ANGEL_OF_EDGE_POINT = 45;
				plane_seg_th.THRESHOLD_MAX_HIGH_MSE_RATIO = 0.02f;
				plane_seg_th.THRESHOLD_MIN_GOOD_PLANE_VOXEL_SIZE = 50;
			}
			else if (sys_control_para_.scanner_type == LEICA_TYPE_0)
			{
				plane_seg_th.THRESHOLD_MIN_PLANE_DIST_OF_TWO_VOXEL = 0.2f * voxel_length;
				plane_seg_th.THRESHOLD_MIN_DIST_OF_POINT2VOXEL = 0.4f * voxel_length;
				plane_seg_th.THRESHOLD_MAX_NORMAL_ANGLE_OF_TWO_VOXEL = 10;
				plane_seg_th.THRESHOLD_MIN_POINT_NUM_OF_VALID_NORMAL_VOXEL = 10;
				plane_seg_th.THRESHOLD_MAX_NORMAL_ANGEL_OF_2PLANE = 10;
				plane_seg_th.THRESHOLD_MAX_NORMAL_ANGEL_OF_EDGE_POINT = 45;
				plane_seg_th.THRESHOLD_MAX_HIGH_MSE_RATIO = 0.02f;
				plane_seg_th.THRESHOLD_MIN_GOOD_PLANE_VOXEL_SIZE = 50;
			}
			else if (sys_control_para_.scanner_type == INDUSTRY_TYPE_0)
			{
				plane_seg_th.THRESHOLD_MIN_PLANE_DIST_OF_TWO_VOXEL = 0.3333f * voxel_length;
				plane_seg_th.THRESHOLD_MIN_DIST_OF_POINT2VOXEL = 0.4166f * voxel_length;
				plane_seg_th.THRESHOLD_MAX_NORMAL_ANGLE_OF_TWO_VOXEL = 5;
				plane_seg_th.THRESHOLD_MIN_POINT_NUM_OF_VALID_NORMAL_VOXEL = 10;
				plane_seg_th.THRESHOLD_MAX_NORMAL_ANGEL_OF_2PLANE = 10;
				plane_seg_th.THRESHOLD_MAX_NORMAL_ANGEL_OF_EDGE_POINT = 45;
				plane_seg_th.THRESHOLD_MAX_HIGH_MSE_RATIO = 0.01f;
				plane_seg_th.THRESHOLD_MIN_GOOD_PLANE_VOXEL_SIZE = 20;
			}
			else if (sys_control_para_.scanner_type == INDUSTRY_TYPE_1)
			{
				plane_seg_th.THRESHOLD_MIN_PLANE_DIST_OF_TWO_VOXEL = 0.15f * voxel_length;
				plane_seg_th.THRESHOLD_MIN_DIST_OF_POINT2VOXEL = 0.2f * voxel_length;
				plane_seg_th.THRESHOLD_MAX_NORMAL_ANGLE_OF_TWO_VOXEL = 10;
				plane_seg_th.THRESHOLD_MIN_POINT_NUM_OF_VALID_NORMAL_VOXEL = 5;
				plane_seg_th.THRESHOLD_MAX_NORMAL_ANGEL_OF_2PLANE = 10;
				plane_seg_th.THRESHOLD_MAX_NORMAL_ANGEL_OF_EDGE_POINT = 45;
				plane_seg_th.THRESHOLD_MAX_HIGH_MSE_RATIO = 0.02f;
				plane_seg_th.THRESHOLD_MIN_GOOD_PLANE_VOXEL_SIZE = 10;
			}
		}
		else if (option_section == SEG_CONFIG_DEBUG_PARAM_VALID)
		{
			bool sync_con = ReadConfigFloat("SEG_THRSHLD_CONFIG_DEBUG", "min_plane_dist_of_2voxel", tmp_f);
			if ((sync_con) && (tmp_f >= 0)) plane_seg_th.THRESHOLD_MIN_PLANE_DIST_OF_TWO_VOXEL = tmp_f;

			sync_con = ReadConfigFloat("SEG_THRSHLD_CONFIG_DEBUG", "min_dist_of_point2voxel", tmp_f);
			if ((sync_con) && (tmp_f >= 0)) plane_seg_th.THRESHOLD_MIN_DIST_OF_POINT2VOXEL = tmp_f;

			sync_con = ReadConfigFloat("SEG_THRSHLD_CONFIG_DEBUG", "max_normal_angle_of_2voxel", tmp_f);
			if ((sync_con) && (tmp_f >= 0)) plane_seg_th.THRESHOLD_MAX_NORMAL_ANGLE_OF_TWO_VOXEL = tmp_f;

			sync_con = ReadConfigFloat("SEG_THRSHLD_CONFIG_DEBUG", "max_mse_of_voxel", tmp_f);
			if ((sync_con) && (tmp_f >= 0)) plane_seg_th.THRESHOLD_MAX_MSE_OF_VOXEL = tmp_f;

			sync_con = ReadConfigInt("SEG_THRSHLD_CONFIG_DEBUG", "min_point_num_of_plane", tmp_i);
			if ((sync_con) && (tmp_i >= 0)) plane_seg_th.THRESHOLD_MIN_POINT_NUM_OF_PLANE = tmp_i;

			sync_con = ReadConfigInt("SEG_THRSHLD_CONFIG_DEBUG", "min_voxel_num_of_plane", tmp_i);
			if ((sync_con) && (tmp_i >= 0)) plane_seg_th.THRESHOLD_MIN_VOXEL_NUM_OF_PLANE = tmp_i;

			sync_con = ReadConfigFloat("SEG_THRSHLD_CONFIG_DEBUG", "max_mse_of_plane", tmp_f);
			if ((sync_con) && (tmp_f >= 0)) plane_seg_th.THRESHOLD_MAX_MSE_OF_PLANE = tmp_f;

			sync_con = ReadConfigInt("SEG_THRSHLD_CONFIG_DEBUG", "min_point_num_of_voxel", tmp_i);
			if ((sync_con) && (tmp_i >= 0)) plane_seg_th.THRESHOLD_MIN_POINT_NUM_OF_VALID_NORMAL_VOXEL = tmp_i;

			sync_con = ReadConfigFloat("SEG_THRSHLD_CONFIG_DEBUG", "min_plane_dist_of_2plane", tmp_f);
			if ((sync_con) && (tmp_f >= 0)) plane_seg_th.THRESHOLD_MIN_DIST_OF_2PLANE = tmp_f;

			sync_con = ReadConfigFloat("SEG_THRSHLD_CONFIG_DEBUG", "max_normal_angle_of_2plane", tmp_f);
			if ((sync_con) && (tmp_f >= 0)) plane_seg_th.THRESHOLD_MAX_NORMAL_ANGEL_OF_2PLANE = tmp_f;

			sync_con = ReadConfigFloat("SEG_THRSHLD_CONFIG_DEBUG", "max_normal_angle_of_edge_point", tmp_f);
			if ((sync_con) && (tmp_f >= 0)) plane_seg_th.THRESHOLD_MAX_NORMAL_ANGEL_OF_EDGE_POINT = tmp_f;

			sync_con = ReadConfigFloat("SEG_THRSHLD_CONFIG_DEBUG", "max_high_mse_ratio", tmp_f);
			if ((sync_con) && (tmp_f >= 0)) plane_seg_th.THRESHOLD_MAX_HIGH_MSE_RATIO = tmp_f;

			sync_con = ReadConfigInt("SEG_THRSHLD_CONFIG_DEBUG", "min_good_group_in_plane", tmp_i);
			if ((sync_con) && (tmp_i >= 0)) plane_seg_th.THRESHOLD_MIN_NUM_OF_GOOD_GROUP_IN_PLANE = tmp_i;

			sync_con = ReadConfigInt("SEG_THRSHLD_CONFIG_DEBUG", "min_point_num_of_flat_voxel", tmp_i);
			if ((sync_con) && (tmp_i >= 0)) plane_seg_th.THRESHOLD_MIN_POINT_NUM_OF_FLAT_VOXEL = tmp_i;

			sync_con = ReadConfigFloat("SEG_THRSHLD_CONFIG_DEBUG", "max_dist_of_2voxel_connected", tmp_f);
			if ((sync_con) && (tmp_f >= 0)) plane_seg_th.THRESHOLD_DIST_OF_2VOXEL_CONNECTED = tmp_f;

			sync_con = ReadConfigInt("SEG_THRSHLD_CONFIG_DEBUG", "min_good_plane_voxel_size", tmp_i);
			if ((sync_con) && (tmp_i >= 0)) plane_seg_th.THRESHOLD_MIN_GOOD_PLANE_VOXEL_SIZE = tmp_i;
		}
		else
		{
			//nothing need to do
		}
	}

	///**
	//* \brief sync  plane test NormalEstimation parameters from config section NORMALEST_CNTRL_PARA
	//* @param [out] SegCntrlParams &normalEst_control_params,  NormalEstimation  control  parameters
	//*/
	//inline void SyncPlaneTestConfigure(NormalEstimationCntrlParam& normalEst_control_params)
	//{
	//	int tmp_i = std::numeric_limits<int>::lowest();
	//	float tmp_f = std::numeric_limits<float>::infinity();

	//	//SegCntrlParams parse
	//	bool sync_con = ReadConfigInt("NORMALEST_CNTRL_PARA", "is_voxel_size_detected", tmp_i);
	//	if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) normalEst_control_params.is_voxel_size_detected = (tmp_i == 0) ? false : true;

	//	sync_con = ReadConfigInt("NORMALEST_CNTRL_PARA", "is_point_density_set", tmp_i);
	//	if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) normalEst_control_params.is_point_density_set = (tmp_i == 0) ? false : true;

	//	sync_con = ReadConfigFloat("NORMALEST_CNTRL_PARA", "point_density", tmp_f);
	//	if ((sync_con) && (tmp_f >= 0)) normalEst_control_params.point_density = tmp_f;

	//}

	///**
	//* \brief sync  plane test filter control parameters from config section FILTER_CNTRL_PARA
	//* @param [out] SegCntrlParams &filter_control_params,   filter control parameters
	//*/
	//inline void SyncPlaneTestConfigure(FilterCntrlParams& filter_control_params)
	//{
	//	int tmp_i = std::numeric_limits<int>::lowest();
	//	float tmp_f = std::numeric_limits<float>::infinity();

	//	bool sync_con = ReadConfigInt("FILTER_CNTRL_PARA", "downsample_type", tmp_i);
	//	if ((sync_con) && (tmp_i < SCANNER_TYPE_MAX) && (tmp_i > INVALID_SCAN_TYPE)) filter_control_params.downsample_type = (DownsampleType)tmp_i;

	//	sync_con = ReadConfigFloat("FILTER_CNTRL_PARA", "downsample_leaf_size", tmp_f);
	//	if ((sync_con) && (tmp_f >= 0)) filter_control_params.downsample_leaf_size = tmp_f;

	//	sync_con = ReadConfigInt("FILTER_CNTRL_PARA", "is_input_filtered", tmp_i);
	//	if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) filter_control_params.is_input_filtered = (tmp_i == 0) ? false : true;

	//	sync_con = ReadConfigInt("FILTER_CNTRL_PARA", "is_input_unit_check", tmp_i);
	//	if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) filter_control_params.is_input_unit_check = (tmp_i == 0) ? false : true;

	//	sync_con = ReadConfigInt("FILTER_CNTRL_PARA", "origin_in_cloud_center", tmp_i);
	//	if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) filter_control_params.origin_in_cloud_center = (tmp_i == 0) ? false : true;
	//}

#ifdef SAVE_OUTPUT_FILE_DEBUG
	/**
	* \brief  sync plane segmentation debug control parameters from section SEG_DEBUG_CONFIG
	* @param [out]  DebugConfigParams &debug_config_param  plane segmentation debug parameters
	*/
	inline void SyncPlaneSegConfigure(const std::string file_name, DebugConfigParams& debug_config_param)
	{
		int tmp_i = std::numeric_limits<int>::lowest();
		bool sync_con = ReadConfigInt("SEG_DEBUG_CONFIG", "dbg_section_valid", tmp_i);
		if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) debug_config_param.dbg_section_valid = (tmp_i == 0) ? false : true;

		if (debug_config_param.dbg_section_valid)
		{
			sync_con = ReadConfigInt("SEG_DEBUG_CONFIG", "reserved_test", tmp_i);
			if ((sync_con) && (tmp_i >= 0)) debug_config_param.reserved_test = tmp_i;

			sync_con = ReadConfigInt("SEG_DEBUG_CONFIG", "missing_point_output", tmp_i);
			if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) debug_config_param.missing_point_output = (tmp_i == 0) ? false : true;

			sync_con = ReadConfigInt("SEG_DEBUG_CONFIG", "io_debug_info", tmp_i);
			if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) debug_config_param.io_debug_info = (tmp_i == 0) ? false : true;

			sync_con = ReadConfigInt("SEG_DEBUG_CONFIG", "voxel_info_debug", tmp_i);
			if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) debug_config_param.voxel_info_debug = (tmp_i == 0) ? false : true;

			sync_con = ReadConfigInt("SEG_DEBUG_CONFIG", "neighbour_info_debug", tmp_i);
			if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) debug_config_param.neighbour_info_debug = (tmp_i == 0) ? false : true;

			sync_con = ReadConfigInt("SEG_DEBUG_CONFIG", "no_plane_merge", tmp_i);
			if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) debug_config_param.no_plane_merge = (tmp_i == 0) ? false : true;

			sync_con = ReadConfigInt("SEG_DEBUG_CONFIG", "plane_output_debug", tmp_i);
			if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) debug_config_param.plane_output_debug = (tmp_i == 0) ? false : true;

			sync_con = ReadConfigInt("SEG_DEBUG_CONFIG", "merge_pseudobad_voxel_debug", tmp_i);
			if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) debug_config_param.merge_pseudobad_voxel_debug = (tmp_i == 0) ? false : true;

			sync_con = ReadConfigInt("SEG_DEBUG_CONFIG", "merge_bad_voxel_debug", tmp_i);
			if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) debug_config_param.merge_bad_voxel_debug = (tmp_i == 0) ? false : true;

			sync_con = ReadConfigInt("SEG_DEBUG_CONFIG", "pseudo_bad_lost_plane_output_debug", tmp_i);
			if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) debug_config_param.pseudo_bad_lost_plane_output_debug = (tmp_i == 0) ? false : true;

			sync_con = ReadConfigInt("SEG_DEBUG_CONFIG", "is_dwnsmpl_output", tmp_i);
			if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) debug_config_param.is_dwnsmpl_output = (tmp_i == 0) ? false : true;

			sync_con = ReadConfigInt("SEG_DEBUG_CONFIG", "points_group_neighbor_output_debug", tmp_i);
			if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) debug_config_param.points_group_neighbor_output_debug = (tmp_i == 0) ? false : true;

			sync_con = ReadConfigInt("SEG_DEBUG_CONFIG", "merge_same_plane_output_debug", tmp_i);
			if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) debug_config_param.merge_same_plane_output_debug = (tmp_i == 0) ? false : true;

			sync_con = ReadConfigInt("SEG_DEBUG_CONFIG", "include_get_nm_by_file", tmp_i);
			if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) debug_config_param.include_get_nm_by_file = (tmp_i == 0) ? false : true;

			sync_con = ReadConfigInt("SEG_DEBUG_CONFIG", "nm_file_ouput_debug", tmp_i);
			if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) debug_config_param.nm_file_ouput_debug = (tmp_i == 0) ? false : true;

			sync_con = ReadConfigInt("SEG_DEBUG_CONFIG", "io_debug_info", tmp_i);
			if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) debug_config_param.io_debug_info = (tmp_i == 0) ? false : true;

			sync_con = ReadConfigInt("SEG_DEBUG_CONFIG", "io_debug_info", tmp_i);
			if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) debug_config_param.io_debug_info = (tmp_i == 0) ? false : true;
		}
		debug_config_params_ = debug_config_param;
	}
#endif

	/**
	* \brief save plane segmentation config parameters ( into file configParseInfo.txt for debug )
	* @param [in] std::string config_path , output config file name
	*/
	inline void SavePlaneSegConfigParams(const std::string file_name)
	{
		std::ofstream config_outinfo(file_name, std::ios::app);

		//output config of SEG_CNTRL_PARA
		config_outinfo << "[" << "SEG_CNTRL_PARA" << "]" << std::endl;
		config_outinfo << "identify_with_bridge" << "=" << config_params_.identify_with_bridge << std::endl;
		config_outinfo << "filtering_strtgy_type" << "=" << config_params_.filtering_strtgy_type << std::endl;
		config_outinfo << "seg_thrshld_section_valid" << "=" << config_params_.seg_thrshld_section_valid << std::endl;
		config_outinfo << std::endl;

		//output config of section SEG_THRSHLD_CONFIG_DEBUG
		config_outinfo << "[" << "SEG_THRSHLD_CONFIG_DEBUG" << "]" << std::endl;
		config_outinfo << "min_plane_dist_of_2voxel" << "=" << plane_seg_thresholds_.THRESHOLD_MIN_PLANE_DIST_OF_TWO_VOXEL << std::endl;
		config_outinfo << "min_dist_of_point2voxel" << "=" << plane_seg_thresholds_.THRESHOLD_MIN_DIST_OF_POINT2VOXEL << std::endl;
		config_outinfo << "max_normal_angle_of_2voxel" << "=" << plane_seg_thresholds_.THRESHOLD_MAX_NORMAL_ANGLE_OF_TWO_VOXEL << std::endl;
		config_outinfo << "max_mse_of_voxel" << "=" << plane_seg_thresholds_.THRESHOLD_MAX_MSE_OF_VOXEL << std::endl;
		config_outinfo << "min_point_num_of_plane" << "=" << plane_seg_thresholds_.THRESHOLD_MIN_POINT_NUM_OF_PLANE << std::endl;
		config_outinfo << "min_voxel_num_of_plane" << "=" << plane_seg_thresholds_.THRESHOLD_MIN_VOXEL_NUM_OF_PLANE << std::endl;
		config_outinfo << "max_mse_of_plane" << "=" << plane_seg_thresholds_.THRESHOLD_MAX_MSE_OF_PLANE << std::endl;
		config_outinfo << "min_point_num_of_voxel" << "=" << plane_seg_thresholds_.THRESHOLD_MIN_POINT_NUM_OF_VALID_NORMAL_VOXEL << std::endl;
		config_outinfo << "min_plane_dist_of_2plane" << "=" << plane_seg_thresholds_.THRESHOLD_MIN_DIST_OF_2PLANE << std::endl;
		config_outinfo << "max_normal_angle_of_2plane" << "=" << plane_seg_thresholds_.THRESHOLD_MAX_NORMAL_ANGEL_OF_2PLANE << std::endl;
		config_outinfo << "max_normal_angle_of_edge_point" << "=" << plane_seg_thresholds_.THRESHOLD_MAX_NORMAL_ANGEL_OF_EDGE_POINT << std::endl;
		config_outinfo << "max_high_mse_ratio" << "=" << plane_seg_thresholds_.THRESHOLD_MAX_HIGH_MSE_RATIO << std::endl;
		config_outinfo << "min_good_group_in_plane" << "=" << plane_seg_thresholds_.THRESHOLD_MIN_NUM_OF_GOOD_GROUP_IN_PLANE << std::endl;
		config_outinfo << "min_point_num_of_flat_voxel" << "=" << plane_seg_thresholds_.THRESHOLD_MIN_POINT_NUM_OF_FLAT_VOXEL << std::endl;
		config_outinfo << "max_dist_of_2voxel_connected" << "=" << plane_seg_thresholds_.THRESHOLD_DIST_OF_2VOXEL_CONNECTED << std::endl;
		config_outinfo << "min_good_plane_voxel_size" << "=" << plane_seg_thresholds_.THRESHOLD_MIN_GOOD_PLANE_VOXEL_SIZE << std::endl;
		config_outinfo << std::endl;

		//output config of section SEG_DEBUG_CONFIG
		config_outinfo << "[" << "SEG_DEBUG_CONFIG" << "]" << std::endl;
		config_outinfo << "dbg_section_valid" << "=" << debug_config_params_.dbg_section_valid << std::endl;
		config_outinfo << "reserved_test" << "=" << debug_config_params_.reserved_test << std::endl;
		config_outinfo << "missing_point_output" << "=" << debug_config_params_.missing_point_output << std::endl;
		config_outinfo << "io_debug_info" << "=" << debug_config_params_.io_debug_info << std::endl;
		config_outinfo << "voxel_info_debug" << "=" << debug_config_params_.voxel_info_debug << std::endl;
		config_outinfo << "neighbour_info_debug" << "=" << debug_config_params_.neighbour_info_debug << std::endl;
		config_outinfo << "no_plane_merge" << "=" << debug_config_params_.no_plane_merge << std::endl;
		config_outinfo << "plane_output_debug" << "=" << debug_config_params_.plane_output_debug << std::endl;
		config_outinfo << "merge_pseudobad_voxel_debug" << "=" << debug_config_params_.merge_pseudobad_voxel_debug << std::endl;
		config_outinfo << "merge_bad_voxel_debug" << "=" << debug_config_params_.merge_bad_voxel_debug << std::endl;
		config_outinfo << "pseudo_bad_lost_plane_output_debug" << "=" << debug_config_params_.pseudo_bad_lost_plane_output_debug << std::endl;
		config_outinfo << "is_dwnsmpl_output" << "=" << debug_config_params_.is_dwnsmpl_output << std::endl;
		config_outinfo << "points_group_neighbor_output_debug" << "=" << debug_config_params_.points_group_neighbor_output_debug << std::endl;
		config_outinfo << "merge_same_plane_output_debug" << "=" << debug_config_params_.merge_same_plane_output_debug << std::endl;
		config_outinfo << "include_get_nm_by_file" << "=" << debug_config_params_.include_get_nm_by_file << std::endl;
		config_outinfo << "nm_file_ouput_debug" << "=" << debug_config_params_.nm_file_ouput_debug << std::endl;
		config_outinfo << std::endl;

		config_outinfo.close();
	}

	///**
	//* \brief save all the config parameters for plane segmentation test ( into file configParseInfo.txt for debug )
	//* @param [in] std::string config_path , output config file name
	//*/
	//inline void SavePlaneTestConfigParams(const std::string file_name)
	//{
	//	SavePlaneSegConfigParams(file_name);

	//	std::ofstream config_outinfo(file_name, std::ios::app);
	//	output config of FILTER_CNTRL_PARA
	//	config_outinfo << "[" << "FILTER_CNTRL_PARA" << "]" << std::endl;
	//	config_outinfo << "downsample_type" << "=" << filter_control_para.downsample_type << std::endl;
	//	config_outinfo << "downsample_leaf_size" << "=" << filter_control_para.downsample_leaf_size << std::endl;
	//	config_outinfo << "is_input_filtered" << "=" << filter_control_para.is_input_filtered << std::endl;
	//	config_outinfo << "is_input_unit_check" << "=" << filter_control_para.is_input_unit_check << std::endl;
	//	config_outinfo << "origin_in_cloud_center" << "=" << filter_control_para.origin_in_cloud_center << std::endl;
	//	config_outinfo << std::endl;

	//	//output config of NORMALEST_CNTRL_PARA
	//	config_outinfo << "[" << "NORMALEST_CNTRL_PARA" << "]" << std::endl;
	//	config_outinfo << "is_voxel_size_detected" << "=" << normalEst_control_para.is_voxel_size_detected << std::endl;
	//	config_outinfo << "is_point_density_set" << "=" << normalEst_control_para.is_point_density_set << std::endl;
	//	config_outinfo << "point_density" << "=" << normalEst_control_para.point_density << std::endl;
	//	config_outinfo << std::endl;
	//	config_outinfo.close();
	//}

	//inline VoxelParams& getPlaneSegVoxelSize() { return voxel_params_; };

	inline PlaneFitThresholds& getPlaneSegThresholds() { return plane_seg_thresholds_; }

	inline SegCntrlParams& getSegConfigParams() { return config_params_; }

	//inline SysCntrlParams& getSysConfigParams() { return sys_control_para_; }

	//inline NormalEstimationCntrlParam& getPlaneTestNormalEst() { return normalEst_control_para; };

	//inline FilterCntrlParams& getPlaneTestFilterControl() { return filter_control_para; };

#ifdef SAVE_OUTPUT_FILE_DEBUG
	inline DebugConfigParams & getDebugConfigParams() { return debug_config_params_; };
#endif

private:

	/**
	* \brief plane segmentation config parameter
	*/
	SegCntrlParams config_params_;

	/**
	* \brief plane segmentation threshold setting
	*/
	PlaneFitThresholds plane_seg_thresholds_;

	/**
	* \brief  plane segmentatio debug config parameter
	*/
	DebugConfigParams debug_config_params_;
};

#endif // _PLANE_CFG_HPP_