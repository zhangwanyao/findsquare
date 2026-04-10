#include <fstream>
#include "log.h"
#include "line_extract/line_extract_macro.h"
#include "line_extract/line_extract2d_config.h"

line_extract2D_config::line_extract2D_config()
{
	this->freeMemory();
	sys_control_para_.is_cad = false;
	sys_control_para_.scanner_type = INVALID_SCAN_TYPE;

	config_params_.is_point_density_set = false;
	config_params_.point_density = 0.f;
	config_params_.line_thrshld_section_valid = false;

	line_seg_thrshld_.min_line_dist_2pixel = 0.f;
	line_seg_thrshld_.min_line_dist_pt2pixel = 0.f;
	line_seg_thrshld_.max_normal_angle_of_2pixel = 0.f;
	line_seg_thrshld_.max_mse_of_pixel = 0.f;
	line_seg_thrshld_.min_point_num_of_line = 0;
	line_seg_thrshld_.min_pixel_num_of_line = 0;
	line_seg_thrshld_.max_mse_of_line = 0.f;
	line_seg_thrshld_.min_dist_of_2line = 0.f;
	line_seg_thrshld_.max_angle_of_2line = 0.f;
	line_seg_thrshld_.max_high_mse_ratio = 0.f;
	line_seg_thrshld_.min_point_num_of_line_pixel = 0;

	pixel_size_.length_x_of_pixel = 0.f;
	pixel_size_.length_y_of_pixel = 0.f;

#ifdef SAVE_OUTPUT_FILE_DEBUG
	line2d_debug_params_.dbg_section_valid = false;
	line2d_debug_params_.reserved_test = 0;
	line2d_debug_params_.missing_point_output = false;
	line2d_debug_params_.neighbour_info_debug = false;
	line2d_debug_params_.line_output_debug = false;
	line2d_debug_params_.reserved_test1 = 0;
#endif
	this->init();
}

line_extract2D_config::~line_extract2D_config()
{
	this->freeMemory();
}

LineFitThresholds& line_extract2D_config::getLineThreshold()
{
	return line_seg_thrshld_;
}

LineExtrCntrlParams& line_extract2D_config::getLineExtrConfig()
{
	return config_params_;
}

PixelSize& line_extract2D_config::getPixelConfig()
{
	return pixel_size_;
}

SysCntrlParams& line_extract2D_config::getSysConfig()
{
	return sys_control_para_;
}

#ifdef SAVE_OUTPUT_FILE_DEBUG
Line2DDebugParams& line_extract2D_config::getLineDebugConfig()
{
	return line2d_debug_params_;
}
#endif

//get line extraction threshold parameters from section LINE_THRSHLD_CONFIG
void line_extract2D_config::SyncLineConfigure(bool is_valid, LineFitThresholds &line_seg_thrshld)
{
	//ConfigParse config_it;
	int tmp_i = std::numeric_limits<int>::lowest();
	float tmp_f = std::numeric_limits<float>::infinity();
	const char *section = "LINE_THRSHLD_CONFIG";
	if (is_valid)
	{
		bool sync_con = ReadConfigFloat(section, "min_line_dist_2pixel", tmp_f);
		if ((sync_con) && (tmp_f >= 0)) line_seg_thrshld_.min_line_dist_2pixel = tmp_f;

		sync_con = ReadConfigFloat(section, "min_line_dist_pt2pixel", tmp_f);
		if ((sync_con) && (tmp_f >= 0)) line_seg_thrshld_.min_line_dist_pt2pixel = tmp_f;

		sync_con = ReadConfigFloat(section, "max_normal_angle_of_2pixel", tmp_f);
		if ((sync_con) && (tmp_f >= 0)) line_seg_thrshld_.max_normal_angle_of_2pixel = tmp_f;

		sync_con = ReadConfigFloat(section, "max_mse_of_pixel", tmp_f);
		if ((sync_con) && (tmp_f >= 0)) line_seg_thrshld_.max_mse_of_pixel = tmp_f;

		sync_con = ReadConfigInt(section, "min_point_num_of_line", tmp_i);
		if ((sync_con) && (tmp_i >= 0)) line_seg_thrshld_.min_point_num_of_line = tmp_i;

		sync_con = ReadConfigInt(section, "min_pixel_num_of_line", tmp_i);
		if ((sync_con) && (tmp_i >= 0)) line_seg_thrshld_.min_pixel_num_of_line = tmp_i;

		sync_con = ReadConfigFloat(section, "max_mse_of_line", tmp_f);
		if ((sync_con) && (tmp_f >= 0)) line_seg_thrshld_.max_mse_of_line = tmp_f;

		sync_con = ReadConfigFloat(section, "min_dist_of_2line", tmp_f);
		if ((sync_con) && (tmp_f >= 0)) line_seg_thrshld_.min_dist_of_2line = tmp_f;

		sync_con = ReadConfigFloat(section, "max_angle_of_2line", tmp_f);
		if ((sync_con) && (tmp_f >= 0)) line_seg_thrshld_.max_angle_of_2line = tmp_f;

		sync_con = ReadConfigFloat(section, "max_high_mse_ratio", tmp_f);
		if ((sync_con) && (tmp_f >= 0)) line_seg_thrshld_.max_high_mse_ratio = tmp_f;

		sync_con = ReadConfigInt(section, "min_point_num_of_line_pixel", tmp_i);
		if ((sync_con) && (tmp_i >= 0)) line_seg_thrshld_.min_point_num_of_line_pixel = tmp_i;
	}

}

//get line extraction control parameters from config section LINE_CNTRL_PARA
void line_extract2D_config::SyncLineConfigure(LineExtrCntrlParams &line_cntrl_params)
{
	//ConfigParse config_it;
	int tmp_i = std::numeric_limits<int>::lowest();
	float tmp_f = std::numeric_limits<float>::infinity();
	const char *section = "LINE_CNTRL_PARA";
	bool sync_con = ReadConfigInt(section, "is_point_density_set", tmp_i);
	if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) config_params_.is_point_density_set = (tmp_i == 0) ? false : true;
	sync_con = ReadConfigFloat(section, "point_density", tmp_f);
	if ((sync_con) && (tmp_f >= 0)) config_params_.point_density = tmp_f;
	sync_con = ReadConfigInt(section, "line_thrshld_section_valid", tmp_i);
	if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) config_params_.line_thrshld_section_valid = (tmp_i == 0) ? false : true;

}

#ifdef SAVE_OUTPUT_FILE_DEBUG
//get line extraction debug control parameters from section LINE_DBG_PARA
bool line_extract2D_config::SyncLineConfigure(Line2DDebugParams &line2d_debug_params)
{
	//ConfigParse config_it;
	int tmp_i = std::numeric_limits<int>::lowest();
	const char* section = "LINE_DBG_PARA";
	bool sync_con = ReadConfigInt(section, "dbg_section_valid", tmp_i);
	if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) line2d_debug_params.dbg_section_valid = (tmp_i == 0) ? false : true;
	if (line2d_debug_params.dbg_section_valid)
	{
		sync_con = ReadConfigInt(section, "reserved_test", tmp_i);
		if ((sync_con) && (tmp_i >= 0)) line2d_debug_params.reserved_test = tmp_i;

		sync_con = ReadConfigInt(section, "missing_point_output", tmp_i);
		if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) line2d_debug_params.missing_point_output = (tmp_i == 0) ? false : true;

		sync_con = ReadConfigInt(section, "neighbour_info_debug", tmp_i);
		if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) line2d_debug_params.neighbour_info_debug = (tmp_i == 0) ? false : true;

		sync_con = ReadConfigInt(section, "line_output_debug", tmp_i);
		if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) line2d_debug_params.line_output_debug = (tmp_i == 0) ? false : true;

		sync_con = ReadConfigInt(section, "reserved_test1", tmp_i);
		if ((sync_con) && (tmp_i >= 0)) line2d_debug_params.reserved_test1 = tmp_i;

		return true;
	}
	else
	{
		return false;
	}

}

void line_extract2D_config::GetLineConfigure(const std::string config_path, Line2DDebugParams &line2d_debug_params)
{
	if (!this->GetConfigMap(config_path))
	{
		log_error("GetConfigMap failed");
	}	
	//get line extraction debug control parameters from section LINE_DBG_PARA
	if (!SyncLineConfigure(line2d_debug_params))
	{
		log_error("line2d_debug_params failed");
	}
}
#endif


void line_extract2D_config::GetLineConfigure(const std::string config_path)
{
	if (!this->GetConfigMap(config_path))
	{
		log_error("GetConfigMap failed");
	}

	//system control parameters from config
	ParseConfigure(sys_control_para_);

	//voxel size from config
	VoxelParams voxel_size;
	ParseConfigure(voxel_size);
	pixel_size_.length_x_of_pixel = voxel_size.length_x_of_voxel;
	pixel_size_.length_y_of_pixel = voxel_size.length_y_of_voxel;

	//get line extraction control parameters from config section LINE_CNTRL_PARA
	SyncLineConfigure(config_params_);
	//get line extraction threshold parameters from section LINE_THRSHLD_CONFIG
	SyncLineConfigure(config_params_.line_thrshld_section_valid,line_seg_thrshld_);

#ifdef SAVE_OUTPUT_FILE_DEBUG
	//get line extraction debug control parameters from section LINE_DBG_PARA
	SyncLineConfigure(line2d_debug_params_);

#endif

}


//save line extraction config parameters  into file configParseInfo.txt for debug 
void line_extract2D_config::SaveLineConfigParams(const std::string file_name)
{
	std::ofstream config_outinfo(file_name, std::ios::app);

	//output config of LINE_CNTRL_PARA  
	config_outinfo << "[" << "LINE_CNTRL_PARA" << "]" << std::endl;
	config_outinfo << "is_point_density_set" << "=" << config_params_.is_point_density_set << std::endl;
	config_outinfo << "point_density" << "=" << config_params_.point_density << std::endl;
	config_outinfo << "line_thrshld_section_valid" << "=" << config_params_.line_thrshld_section_valid << std::endl;
	config_outinfo << std::endl;

	//output config of section LINE_THRSHLD_CONFIG
	config_outinfo << "[" << "LINE_THRSHLD_CONFIG" << "]" << std::endl;
	config_outinfo << "min_line_dist_2pixel" << "=" << line_seg_thrshld_.min_line_dist_2pixel << std::endl;
	config_outinfo << "min_line_dist_pt2pixel" << "=" << line_seg_thrshld_.min_line_dist_pt2pixel << std::endl;
	config_outinfo << "max_normal_angle_of_2pixel" << "=" << line_seg_thrshld_.max_normal_angle_of_2pixel << std::endl;
	config_outinfo << "max_mse_of_pixel" << "=" << line_seg_thrshld_.max_mse_of_pixel << std::endl;
	config_outinfo << "min_point_num_of_line" << "=" << line_seg_thrshld_.min_point_num_of_line << std::endl;
	config_outinfo << "min_pixel_num_of_line" << "=" << line_seg_thrshld_.min_pixel_num_of_line << std::endl;
	config_outinfo << "max_mse_of_line" << "=" << line_seg_thrshld_.max_mse_of_line << std::endl;
	config_outinfo << "min_dist_of_2line" << "=" << line_seg_thrshld_.min_dist_of_2line << std::endl;
	config_outinfo << "max_angle_of_2line" << "=" << line_seg_thrshld_.max_angle_of_2line << std::endl;
	config_outinfo << "max_high_mse_ratio" << "=" << line_seg_thrshld_.max_high_mse_ratio << std::endl;
	config_outinfo << "min_point_num_of_line_pixel" << "=" << line_seg_thrshld_.min_point_num_of_line_pixel << std::endl;
	config_outinfo << std::endl;

	//output config of section LINE_DBG_PARA
	config_outinfo << "[" << "LINE_DBG_PARA" << "]" << std::endl;
	config_outinfo << "dbg_section_valid" << "=" << line2d_debug_params_.dbg_section_valid << std::endl;
	config_outinfo << "reserved_test" << "=" << line2d_debug_params_.reserved_test << std::endl;
	config_outinfo << "missing_point_output" << "=" << line2d_debug_params_.missing_point_output << std::endl;
	config_outinfo << "neighbour_info_debug" << "=" << line2d_debug_params_.neighbour_info_debug << std::endl;
	config_outinfo << "line_output_debug" << "=" << line2d_debug_params_.line_output_debug << std::endl;
	config_outinfo << "reserved_test1" << "=" << line2d_debug_params_.reserved_test1 << std::endl;
	config_outinfo << std::endl;

}
