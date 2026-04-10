#ifndef _PLANE_CONFIG_PARSE_HPP_
#define _PLANE_CONFIG_PARSE_HPP_
#include "common_struct.h"
#include "ConfigParse.hpp"

class FeatureCommonConfigParse : public ConfigParse
{
public:

	/**
	* \brief constructor with plane segmentation config
	*/
	FeatureCommonConfigParse()
	{
		this->freeMemory();
		sys_control_para_.is_cad = false;
		sys_control_para_.scanner_type = INVALID_SCAN_TYPE;
		sys_control_para_.encryption_type = 0;

		voxel_params_.length_x_of_voxel = 1.f;
		voxel_params_.length_y_of_voxel = 1.f;
		voxel_params_.length_z_of_voxel = 1.f;

		this->init();
	}

	/**
	* \brief destructor
	*/
	~FeatureCommonConfigParse()
	{
		this->freeMemory();
	}

	/**
	* \brief sync   NormalEstimation parameters from config section NORMALEST_CNTRL_PARA
	* @param [out] SegCntrlParams &config_params,  NormalEstimation  control  parameters
	*/
	inline void SyncCommonConfigure(NormalEstimationThresholds& normalEst_thrld_params)
	{
		int tmp_i = std::numeric_limits<int>::lowest();
		float tmp_f = std::numeric_limits<float>::infinity();
		const char* section = "NORMALEST_THRSHLD_CFG";

		//SegCntrlParams parse
		bool sync_con = ReadConfigInt(section, "min_point_num_of_valid_normal_voxel", tmp_i);
		if ((sync_con) && (tmp_i >= 0)) normalEst_thrld_params.min_point_num_of_valid_normal_voxel = tmp_i;

		sync_con = ReadConfigFloat(section, "max_normal_angle_of_2voxel", tmp_f);
		if ((sync_con) && (tmp_f >= 0)) normalEst_thrld_params.max_normal_angle_of_2voxel = tmp_f;

		sync_con = ReadConfigFloat(section, "max_mse_ratio_of_voxel", tmp_f);
		if ((sync_con) && (tmp_f < 1) && (tmp_f > 0)) normalEst_thrld_params.max_mse_ratio_of_voxel = tmp_f;
	}

	/**
	* \brief sync   NormalEstimation parameters from config section NORMALEST_CNTRL_PARA
	* @param [out] SegCntrlParams &config_params,  NormalEstimation  control  parameters
	*/
	inline void SyncCommonConfigure(NormalEstimationCntrlParam& normalEst_control_para)
	{
		int tmp_i = std::numeric_limits<int>::lowest();
		float tmp_f = std::numeric_limits<float>::infinity();
		const char* section = "NORMALEST_CNTRL_PARA";
		//SegCntrlParams parse
		bool sync_con = ReadConfigInt(section, "is_voxel_size_detected", tmp_i);
		if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) normalEst_control_para.is_voxel_size_detected = (tmp_i == 0) ? false : true;

		sync_con = ReadConfigInt(section, "is_point_density_set", tmp_i);
		if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) normalEst_control_para.is_point_density_set = (tmp_i == 0) ? false : true;

		sync_con = ReadConfigFloat(section, "point_density", tmp_f);
		if ((sync_con) && (tmp_f >= 0)) normalEst_control_para.point_density = tmp_f;
	}

	/**
	* \brief sync  filter control parameters from config section FILTER_CNTRL_PARA
	* @param [out] SegCntrlParams &filter_control_params,   filter control parameters
	*/
	inline void SyncCommonConfigure(FilterCntrlParams& filter_control_params)
	{
		int tmp_i = std::numeric_limits<int>::lowest();
		float tmp_f = std::numeric_limits<float>::infinity();

		bool sync_con = ReadConfigInt("FILTER_CNTRL_PARA", "downsample_type", tmp_i);
		if ((sync_con) && (tmp_i < DOWNSAMPLE_TYPE_MAX) && (tmp_i > INVALID_DOWNSAMPLE_TYPE)) filter_control_params.downsample_type = (DownsampleType)tmp_i;

		sync_con = ReadConfigFloat("FILTER_CNTRL_PARA", "downsample_leaf_size", tmp_f);
		if ((sync_con) && (tmp_f >= 0)) filter_control_params.downsample_leaf_size = tmp_f;

		sync_con = ReadConfigInt("FILTER_CNTRL_PARA", "is_input_filtered", tmp_i);
		if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) filter_control_params.is_input_filtered = (tmp_i == 0) ? false : true;

		sync_con = ReadConfigInt("FILTER_CNTRL_PARA", "is_input_unit_check", tmp_i);
		if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) filter_control_params.is_input_unit_check = (tmp_i == 0) ? false : true;

		sync_con = ReadConfigInt("FILTER_CNTRL_PARA", "origin_in_cloud_center", tmp_i);
		if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) filter_control_params.origin_in_cloud_center = (tmp_i == 0) ? false : true;
	}

	/**
	* \brief  Get  all the Featrues common config parameters from config file(config.ini)
	* @param [in] const std::string config_file, config.ini file
	*/
	inline bool GetCommonConfigure(const std::string config_file)
	{
		if (!this->GetConfigMap(config_file))
		{
			log_error("GetConfigMap failed");
			return false;
		}

		//system control parameters from config
		ParseConfigure(sys_control_para_);

		//voxel size from config section VOXEL_SIZE_CONFIG
		ParseConfigure(voxel_params_);

		ParseConfigure(sys_dbg_para);

		SyncCommonConfigure(filter_control_para);

		SyncCommonConfigure(normalEst_control_para);

		SyncCommonConfigure(normalEst_thrld_para);
		return true;
	}

	/**
	* \brief save all the config parameters for plane segmentation test ( into file configParseInfo.txt for debug )
	* @param [in] std::string config_path , output config file name
	*/
	inline void SaveCommonConfigParams(const std::string config_file)
	{
		SaveConfigParams(config_file, sys_control_para_);
		SaveConfigParams(config_file, voxel_params_);
		SaveConfigParams(config_file, sys_dbg_para);
		std::ofstream config_outinfo(config_file, std::ios::app);
		//output config of FILTER_CNTRL_PARA
		config_outinfo << "[" << "FILTER_CNTRL_PARA" << "]" << std::endl;
		config_outinfo << "downsample_type" << "=" << filter_control_para.downsample_type << std::endl;
		config_outinfo << "downsample_leaf_size" << "=" << filter_control_para.downsample_leaf_size << std::endl;
		config_outinfo << "is_input_filtered" << "=" << filter_control_para.is_input_filtered << std::endl;
		config_outinfo << "is_input_unit_check" << "=" << filter_control_para.is_input_unit_check << std::endl;
		config_outinfo << "origin_in_cloud_center" << "=" << filter_control_para.origin_in_cloud_center << std::endl;
		config_outinfo << std::endl;

		//output config of NORMALEST_CNTRL_PARA
		config_outinfo << "[" << "NORMALEST_CNTRL_PARA" << "]" << std::endl;
		config_outinfo << "is_voxel_size_detected" << "=" << normalEst_control_para.is_voxel_size_detected << std::endl;
		config_outinfo << "is_point_density_set" << "=" << normalEst_control_para.is_point_density_set << std::endl;
		config_outinfo << "point_density" << "=" << normalEst_control_para.point_density << std::endl;
		config_outinfo << std::endl;

		//output config of NORMALEST_THRSHLD_CFG
		config_outinfo << "[" << "NORMALEST_THRSHLD_CFG" << "]" << std::endl;
		config_outinfo << "min_point_num_of_valid_normal_voxel" << "=" << normalEst_thrld_para.min_point_num_of_valid_normal_voxel << std::endl;
		config_outinfo << "max_normal_angle_of_2voxel" << "=" << normalEst_thrld_para.max_normal_angle_of_2voxel << std::endl;
		config_outinfo << "max_mse_ratio_of_voxel" << "=" << normalEst_thrld_para.max_mse_ratio_of_voxel << std::endl;
		config_outinfo << std::endl;

		config_outinfo.close();
	}

	virtual inline VoxelParams& getVoxelSize() { return voxel_params_; };

	virtual inline SysCntrlParams& getSysConfigParams() { return sys_control_para_; }

	inline NormalEstimationCntrlParam& getNormalEstParams() { return normalEst_control_para; };

	inline FilterCntrlParams& getFilterControl() { return filter_control_para; };

	inline NormalEstimationThresholds& getNormalEstThrldParams() { return normalEst_thrld_para; };
	virtual inline SysDebugParams& getSysDbgParams() { return sys_dbg_para; };

protected:
	/**
	* \brief voxel_params is the voxel size along x ,y ,z axis
	*/
	VoxelParams voxel_params_;

	/**
	* \brief system control parameter
	*/
	SysCntrlParams sys_control_para_;

	/**
	* \brief normalEst_control_para is the NormalEstimation parameters
	*/
	NormalEstimationCntrlParam normalEst_control_para;

	/**
	* \brief filter_control_para is the input data filter control parameters
	*/
	FilterCntrlParams filter_control_para;

	/**
	* \brief filter_control_para is the input data filter control parameters
	*/
	NormalEstimationThresholds normalEst_thrld_para;

	/**
	* \brief sys_dbg_para is the system debug  control parameters
	*/
	SysDebugParams sys_dbg_para;
};

#endif // _PLANE_CONFIG_PARSE_HPP_