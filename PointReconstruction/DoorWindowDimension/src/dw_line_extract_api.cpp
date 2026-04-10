#include <iostream>
#include "log.h"
#include "InOutData.h"
#include "line_extract/line_extract_macro.h"
#include "line_extract/line_extract_api.h"
//#include "line_extract2d_config.h"
#include "line_extract/line2d_cfg.hpp"
#include "line_extract/line_extract2d.h"
#include "line_extract/util_line_rot.hpp"

using namespace ModuleStruct;

void* Features_LineExtractCreateHandle()
{
	return (new DW_line_extract2D());
}

void Features_LineExtractDestroyHandle(void** handle)
{
	if (handle)
	{
		DW_line_extract2D* ptr = reinterpret_cast<DW_line_extract2D*>(*handle);
		delete ptr;
		ptr = nullptr;
	}
}

static inline void Features_convertLineOutput(const Line2DSegOut line_out, Vector<Line2DSegOutItem>& line_segout)
{
	line_segout.resize(line_out.size);
	for (unsigned int i = 0; i < line_out.size; i++)
	{
		line_segout[i].line_direction = line_out.line_segs[i].line_direction;
		line_segout[i].line_seg_start = line_out.line_segs[i].line_seg_start;
		line_segout[i].line_seg_end = line_out.line_segs[i].line_seg_end;
	}
}

static inline void Features_convertLineOutput(const Line2DSegOut line_out, Vector<Line2DSegOutItemDebug>& line_segout)
{
	line_segout.resize(line_out.size);
	for (unsigned int i = 0; i < line_out.size; i++)
	{
		line_segout[i].line_direction = line_out.line_segs[i].line_direction;
		line_segout[i].line_seg_start = line_out.line_segs[i].line_seg_start;
		line_segout[i].line_seg_end = line_out.line_segs[i].line_seg_end;
		line_segout[i].line_center = line_out.line_segs[i].line_center;
		line_segout[i].line_mse = line_out.line_segs[i].line_mse;
		line_segout[i].points.assign(line_out.line_segs[i].points.point_idx, line_out.line_segs[i].points.point_idx + line_out.line_segs[i].points.size);
	}
}

bool Features_GetLinePoints2D(void* handle, const std::string config_file, const Point2fArray&input_data, Line2DSegOut* line_seg)
{
	DW_line_extract2D* line_extract = reinterpret_cast<DW_line_extract2D*>(handle);

	//line_extract2D_config config_it;
	Line2DConfigParse config_it;
	config_it.GetLineConfigure(config_file);
	PixelSize pixel_size = config_it.getPixelConfig();
	line_extract->SetConfigure(config_it.getSysConfigParams(), config_it.getLineExtrConfig(), config_it.getLineThreshold(), pixel_size);

#ifdef SAVE_OUTPUT_FILE_DEBUG
	line_extract->SetConfigure(config_it.getSysConfigParams(), config_it.getLineExtrConfig(), config_it.getLineThreshold(), pixel_size, config_it.getLineDebugConfig());
	std::string output_path = line_extract->getOutputPath();
	std::string save_file = output_path +"configinfo.txt";
	config_it.SaveConfigInfo(save_file);
	save_file = output_path+"configParseinfo.txt";
	config_it.SaveConfigParams(save_file, config_it.getSysConfigParams());
	//config_it.SaveConfigParams(save_file, voxel_size);
	config_it.SaveLineConfigParams(save_file);
#endif
	log_info("output_path = %s", output_path.c_str());
	line_extract->setOutputPath(output_path);
	line_extract->FitLine(input_data, line_seg);
	return true;
}


static inline void Features_LineOutputRelease(Line2DSegOut& line_out)
{

	if (line_out.line_segs != NULL)
	{
		for (unsigned int i = 0; i < line_out.size; i++)
		{
			if (line_out.line_segs[i].points.point_idx != NULL)
			{
				delete[] line_out.line_segs[i].points.point_idx;
				line_out.line_segs[i].points.point_idx = NULL;
			}
		}
		delete[] line_out.line_segs;
		line_out.line_segs = NULL;
	}
}



bool Features_GetLinePoints2D(void* handle, const Point2fArray& input_data, Vector<Line2DSegOutItem>& line_seg)
{
	Line2DSegOut line_out;
	DW_line_extract2D* line_extract = reinterpret_cast<DW_line_extract2D*>(handle);
	line_extract->FitLine(input_data, &line_out);
	Features_convertLineOutput(line_out, line_seg);
	Features_LineOutputRelease(line_out);
	return true;
}


bool Features_GetLinePoints2D(void* handle, const std::string config_file, const Point2fArray &input_data, Vector<Line2DSegOutItem>& line_seg)
{
	Line2DSegOut line_out;
	Features_GetLinePoints2D(handle, config_file, input_data, &line_out);
	Features_convertLineOutput(line_out, line_seg);
	Features_LineOutputRelease(line_out);
	return true;
}

bool Features_GetLinePoints2D(void* handle, const std::string config_file, const Point2fArray &input_data, Vector<Line2DSegOutItemDebug>& line_seg)
{
	Line2DSegOut line_out;
	Features_GetLinePoints2D(handle, config_file, input_data, &line_out);
	Features_convertLineOutput(line_out, line_seg);
	Features_LineOutputRelease(line_out);
	return true;
}


bool Features_GetLinePoints2D(void* handle,///T
							  const Point2fArray&input_data, 
							  Vector<Line2DSegOutItemDebug>& line_seg)
{
	Line2DSegOut line_out;
	DW_line_extract2D* line_extract = reinterpret_cast<DW_line_extract2D*>(handle);

	line_extract->FitLine(input_data, &line_out);///T
	Features_convertLineOutput(line_out, line_seg);

	Features_LineOutputRelease(line_out);
	return true;
}


bool Features_GetLinePoints2D(void* handle, const std::string config_file, Vector<Point2fArray>& plane_input_data, Vector<Line2DSegOutItemDebug>& plane_line_seg)
{
	DW_line_extract2D* line_extract = reinterpret_cast<DW_line_extract2D*>(handle);
	//line_extract2D_config config_it;
	Line2DConfigParse config_it;
	config_it.GetLineConfigure(config_file);
	PixelSize pixel_size = config_it.getPixelConfig();

	line_extract->SetConfigure(config_it.getSysConfigParams(), config_it.getLineExtrConfig(), config_it.getLineThreshold(), pixel_size);

#ifdef SAVE_OUTPUT_FILE_DEBUG
	line_extract->SetConfigure(config_it.getSysConfigParams(), config_it.getLineExtrConfig(), config_it.getLineThreshold(), pixel_size, config_it.getLineDebugConfig());
	std::string output_path = line_extract->getOutputPath();
	std::string save_file = output_path + "configinfo.txt";
	config_it.SaveConfigInfo(save_file);
	save_file = output_path + "configParseinfo.txt";
	config_it.SaveConfigParams(config_file, config_it.getSysConfigParams());
	config_it.SaveLineConfigParams(save_file);
#endif
	line_extract->setOutputPath(output_path);

	//Vector<Line2DSegOutItemDebug>plane_line_seg;
	plane_line_seg.clear();

	for (int i = 0; i < plane_input_data.size(); i++)
	{
		Vector<Line2DSegOutItemDebug>line_seg;
		Line2DSegOut line_out;
		Point2fArray line_input_data = plane_input_data[i];
		line_extract->FitLine(line_input_data, &line_out);
		Features_convertLineOutput(line_out, line_seg);
		plane_line_seg.insert(plane_line_seg.end(), line_seg.begin(), line_seg.end());
		Features_LineOutputRelease(line_out);
	}

	return true;

}


void Features_SetLineConfig(void* handle, const SysCntrlParams sys_control_para, const LineExtrCntrlParams line_config_params, \
	const LineFitThresholds line_seg_thrshld, const PixelSize pixel_size)
{
	DW_line_extract2D* line_extract = reinterpret_cast<DW_line_extract2D*>(handle);
	
	line_extract->SetConfigure(sys_control_para, line_config_params, line_seg_thrshld, pixel_size);
}

bool Features_SetLineDbgPath(void* handle, const std::string output_path)
{
	DW_line_extract2D* line_extract = reinterpret_cast<DW_line_extract2D*>(handle);
	if ((IOData::createDirectory(output_path)) != 0)
	{
		std::cout << "createDirectory: " << output_path << " failed" << std::endl;
		return false;
	}
	line_extract->setOutputPath(output_path);
	return true;
}

LINE_EXTRACT_EXPORT void Features_SetLineConfig(void* handle, const SysCntrlParams sys_control_para, const LineExtrCntrlParams line_config_params, \
	const LineFitThresholds line_seg_thrshld, const PixelSize pixel_size, const Line2DDebugParams line_dbg_para)
{
	DW_line_extract2D* line_extract = reinterpret_cast<DW_line_extract2D*>(handle);

	line_extract->SetConfigure(sys_control_para, line_config_params, line_seg_thrshld, pixel_size, line_dbg_para);
}


