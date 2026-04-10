#ifndef _LINE_EXTRACT2D_CONFIG_H_
#define _LINE_EXTRACT2D_CONFIG_H_
#include "line_extract_macro.h"
#include "pxl_struct.h"
#include "pxl_struct_inf.h"
#include "ConfigParse.hpp"


class line_extract2D_config : public ConfigParse {

public:

	/**
	* \brief constructor with line extract config
	*/
	line_extract2D_config();

	/**
	* \brief destructor
	*/
	~line_extract2D_config();


	/**
	* \brief  Get line extraction config parameters from config file(config.ini)
	* @param config_path  config file path
	*/
	void GetLineConfigure(const std::string config_path);
#ifdef SAVE_OUTPUT_FILE_DEBUG
	/**
	* \brief  get line extraction debug control parameters from section LINE_DBG_PARA
	* @param config_path  config file path
	*/
	void GetLineConfigure(const std::string config_path, Line2DDebugParams &line2d_debug_params);
#endif
	/**
	* \brief  sync line extraction threshold parameters from section LINE_THRSHLD_CONFIG
	* @bool is_valid  if true show this section is valid otherwise is invalid
	* @param  LineFitThresholds &line_seg_thrshld line extraction threshold
	*/
	void SyncLineConfigure(bool is_valid, LineFitThresholds &line_seg_thrshld);

	/**
	* \brief sync line extraction control parameters from config section LINE_CNTRL_PARA
	* @LineExtrCntrlParams &line_cntrl_params line extraction control parameters
	*/
	void SyncLineConfigure(LineExtrCntrlParams &line_cntrl_params);
#ifdef SAVE_OUTPUT_FILE_DEBUG
	/**
	* \brief  sync line extraction debug control parameters from section LINE_DBG_PARA
	* @Line2DDebugParams &line2d_debug_para line extraction debug parameters
	*/
	bool SyncLineConfigure(Line2DDebugParams &line2d_debug_params);
#endif
	/**
	* \brief save line extraction config parameters ( into file configParseInfo.txt for debug )
	*/
	void SaveLineConfigParams(const std::string config_path);

	/**
	* \brief get line threshold from config file
	*/
	LineFitThresholds &getLineThreshold();
	
	/**
	* \brief get line control parameters from config file
	*/
	LineExtrCntrlParams &getLineExtrConfig();	

	/**
	* \brief get line pixel size from config file
	*/
	PixelSize &getPixelConfig();
	
	/**
	* \brief get system control parameters from config file
	*/
	SysCntrlParams &getSysConfig();
	
#ifdef SAVE_OUTPUT_FILE_DEBUG

	/**
	* \brief get line debug parameters from config file
	*/
	Line2DDebugParams &getLineDebugConfig();
#endif
	
private:
	/**
	* \brief PixelSize is the pixel size along x ,y axis
	*/
	PixelSize pixel_size_;

	/**
	* \brief Line extraction config parameter
	*/
	SysCntrlParams sys_control_para_;


	/**
	* \brief Line extraction config parameter
	*/
	LineExtrCntrlParams config_params_;

	/**
	* \brief Line extraction threshold setting
	*/
	LineFitThresholds line_seg_thrshld_;

#ifdef SAVE_OUTPUT_FILE_DEBUG
	/**
	* \brief Line extraction debug config parameter
	*/
	Line2DDebugParams line2d_debug_params_;
#endif

}; 


#endif // _LINE_EXTRACT2D_CONFIG_H_
