#ifndef _PLANE_CONFIG_PARSE_H_
#define _PLANE_CONFIG_PARSE_H_
#include "config.h"
#include "DataStruct.h"
#include "plane_seg_inf.h"
#include "ConfigParse.hpp"

class planeSegConfigParse : public ConfigParse
{
public:

	/**
	* \brief constructor with plane segmentation config
	*/
	planeSegConfigParse();

	/**
	* \brief destructor
	*/
	~planeSegConfigParse();

	/**
	* \brief  Get  plane segmentation config parameters from config file(config.ini)
	* @param [in] const std::string config_ini, config.ini file
	*/
	bool GetPlaneSegConfigure(const std::string config_ini);

	/**
	* \brief  Get  all the config parameters  for plane segmentation test  from config file(config.ini)
	* @param [in] const std::string config_ini, config.ini file
	*/
	bool GetPlaneTestConfigure(const std::string config_ini);

	/**
	* \brief sync  plane segmentation config parameters from config section SEG_CNTRL_PARA
	* @param [out] SegCntrlParams &config_params Plane segmentation parameters
	*/
	void SyncPlaneSegConfigure(SegCntrlParams &config_params);

	/**
	* \brief  sync  plane segmentation threshold parameters from section SEG_THRSHLD_CONFIG or SEG_THRSHLD_CONFIG_DEBUG
	* @param [in] bool is_valid  if true show this section is valid otherwise is invalid
	* @param [in] PlaneFitThresholds &plane_seg_th plane segmentation threshold
	*/
	void SyncPlaneSegConfigure(unsigned int option_section, float voxel_length, PlaneFitThresholds &plane_seg_th);

	/**
	* \brief sync  plane test NormalEstimation parameters from config section NORMALEST_CNTRL_PARA
	* @param [out] SegCntrlParams &config_params,  NormalEstimation  control  parameters
	*/
	void SyncPlaneTestConfigure(NormalEstimationCntrlParam& normalEst_control_para);

	/**
	* \brief sync  plane test filter control parameters from config section FILTER_CNTRL_PARA
	* @param [out] SegCntrlParams &filter_control_params,   filter control parameters
	*/
	void SyncPlaneTestConfigure(FilterCntrlParams& filter_control_params);

#ifdef SAVE_OUTPUT_FILE_DEBUG
	/**
	* \brief  sync plane segmentation debug control parameters from section SEG_DEBUG_CONFIG
	* @param [out]  DebugConfigParams &debug_config_param  plane segmentation debug parameters
	*/
	void SyncPlaneSegConfigure(const std::string file_name, DebugConfigParams &debug_config_param);
#endif

	/**
	* \brief save plane segmentation config parameters ( into file configParseInfo.txt for debug )
	* @param [in] std::string config_path , output config file name
	*/
	void SavePlaneSegConfigParams(const std::string config_path);

	/**
	* \brief save all the config parameters for plane segmentation test ( into file configParseInfo.txt for debug )
	* @param [in] std::string config_path , output config file name
	*/
	void SavePlaneTestConfigParams(const std::string config_path);

	VoxelParams& getPlaneSegVoxelSize() { return voxel_params_; };

	PlaneFitThresholds& getPlaneSegThresholds() { return plane_seg_thresholds_; }

	SegCntrlParams& getSegConfigParams() { return config_params_; }

	SysCntrlParams& getSysConfigParams() { return sys_control_para_; }

	NormalEstimationCntrlParam& getPlaneTestNormalEst() { return normalEst_control_para; };

	FilterCntrlParams& getPlaneTestFilterControl() { return filter_control_para; };

#ifdef SAVE_OUTPUT_FILE_DEBUG
	DebugConfigParams & getDebugConfigParams() { return debug_config_params_; };
#endif

private:
	/**
	* \brief voxel_params is the voxel size along x ,y ,z axis
	*/
	VoxelParams voxel_params_;

	/**
	* \brief system control parameter
	*/
	SysCntrlParams sys_control_para_;

	/**
	* \brief plane segmentation config parameter
	*/
	SegCntrlParams config_params_;

	/**
	* \brief plane segmentation threshold setting
	*/
	PlaneFitThresholds plane_seg_thresholds_;

	/**
	* \brief normalEst_control_para is the NormalEstimation parameters
	*/
	NormalEstimationCntrlParam normalEst_control_para;

	/**
	* \brief filter_control_para is the input data filter control parameters
	*/
	FilterCntrlParams filter_control_para;

	/**
	* \brief  plane segmentatio debug config parameter
	*/
	DebugConfigParams debug_config_params_;
};

#endif // _PLANE_CONFIG_PARSE_H_
