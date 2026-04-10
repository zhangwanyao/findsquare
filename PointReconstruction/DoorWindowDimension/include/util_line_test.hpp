#ifndef _UTIL_LINE_TEST_HPP_
#define _UTIL_LINE_TEST_HPP_

#include <string>
#include <iostream>
#include "log.h"
#include "in_out_data.hpp"
#include "pxl_struct_inf.h"
#include "util_math.hpp"
#include "line_extract/util_line_rot.hpp"
#include "line_extract/line2d_cfg.hpp"
#include "line_extract/line_extract_api.h"

namespace util_line_test
{
    /**
	* \brief save line segmention output xyz to file
	* @param [in] sub_path,  out put path, with endof "/" or"\\"
	* @param [in] sub_file_name, input file name
	* @param [in] delimiter, delimiter of save data
	* @param [in] file_type,  save file suffix
	* @param [in] input_data, input 2D point cloud
	* @param [in] line_seg_out,  segment out put  of all lines
	* return > = 0 success, else failed
	* */
	static inline int save_line_xyz2D(const std::string sub_path, const std::string sub_file_name, const std::string delimiter, \
		const std::string file_type, const ModuleStruct::Point2fArray input_data, ModuleStruct::Vector<Line2DSegOutItemDebug> line_seg_out)
	{
		if (IOData::createDirectory(sub_path) != 0)
		{
			std::cout << "can not create path: " << sub_path << std::endl;
			return -1;
		}

		for (unsigned int i = 0; i < line_seg_out.size(); i++)
		{
			Line2DSegOutItemDebug* line_it = &line_seg_out[i];
			std::stringstream line_id;
			line_id << i;
			//file_name = "line_xyz";
			//folder = output_path + "line_xyz\\";

			std::string test_output_path = sub_path + sub_file_name + "_" + line_id.str() + file_type;
			//std::string delimiter = ";";
			ModuleStruct::Point2fArray line_pt;
			line_pt.clear();
			line_pt.resize(line_it->points.size());

			for (int j = 0; j < line_pt.size(); j++)
			{
				unsigned int point_idx = line_it->points[j];
				line_pt[j] = input_data[point_idx];
			}
			// save line point cloud
			FeaturesIO::SavePoint2fDataWithDelimiter(test_output_path, delimiter, line_pt);
		}
		return 0;
	}

	 /**
	* \brief save line segmention output xyz to file
	* @param [in] sub_path,  out put path, with endof "/" or"\\"
	* @param [in] sub_file_name, input file name
	* @param [in] delimiter, delimiter of save data
	* @param [in] file_type,  save file suffix
	* @param [in] input_data, input 2D point cloud
	* @param [in] origin_normal, input 2D point cloud normal in 3d space
	* @param [in] origin_center, input 2D point cloud center in 3d space
	* @param [in] input_data, input 2D point cloud
	* @param [in] line_seg_out,  segment out put  of all lines
	* return > = 0 success, else failed
	* */
	static inline int save_line_xyz3D(const std::string sub_path, const std::string sub_file_name, const std::string delimiter, \
		const std::string file_type, const ModuleStruct::Point2fArray input_data, const ModuleStruct::Point3f origin_normal, const ModuleStruct::Point3f origin_center, \
		ModuleStruct::Vector<Line2DSegOutItemDebug> line_seg_out)
	{
		if (IOData::createDirectory(sub_path) != 0)
		{
			std::cout << "can not create path: " << sub_path << std::endl;
			return -1;
		}

		ModuleStruct::Point3f nz(0.0f, 0.0f, 1.0f);
		bool isNeedRot = !Util_Math::vec3_are_same(origin_normal, nz); // is need rotation
		ModuleStruct::CMat rot_mat(3, 3, CV_32F);

		if (isNeedRot)
		{
			rot_mat = Util_Math::CreateRotationMat4E(nz, origin_normal);
		}

		for (unsigned int i = 0; i < line_seg_out.size(); i++)
		{
			Line2DSegOutItemDebug* line_it = &line_seg_out[i];
			std::stringstream line_id;
			line_id << i;
			//file_name = "line_xyz";
			//folder = output_path + "line_xyz\\";

			std::string test_output_path = sub_path + sub_file_name + "_" + line_id.str() + file_type;
			//std::string delimiter = ";";
			ModuleStruct::Point2fArray line_pt;
			line_pt.clear();
			line_pt.resize(line_it->points.size());
			ModuleStruct::Point3fArray line_pt3d;
			line_pt3d.clear();
			line_pt3d.resize(line_it->points.size());

			for (int j = 0; j < line_pt.size(); j++)
			{
				unsigned int point_idx = line_it->points[j];
				line_pt[j] = input_data[point_idx];
				line_pt3d[j] = util_line_rot::point2d_to_3d(isNeedRot, line_pt[j], rot_mat, origin_center);
			}
			// save line point cloud
			IOData::SavePoint3fDataWithDelimiter(test_output_path, delimiter, line_pt3d);
		}
		return 0;
	}

	/**
	* \brief save line segmention output  line direction ,center , endpoint to file
	* @param [in] save_type,  0:center and end points, 1: direction, else, center and end points together
	* @param [in] sub_path,  out put path, with endof "/" or"\\"
	* @param [in] file_name, data  file  to save
	* @param [in] line_seg_out, all lines output
	* return = 0 success, else failed
	* */
	static inline int save_line_direction_end2D(int save_type, const std::string sub_path, const std::string file_name, ModuleStruct::Vector<Line2DSegOutItemDebug> line_seg_out)
	{
		if (IOData::createDirectory(sub_path))
		{
			log_error("createDirectory %s failed", sub_path.c_str());
			return -1;
		}
		std::string file_type;
		file_type = ".txt";

		for (unsigned int i = 0; i < line_seg_out.size(); i++) {
			Line2DSegOutItemDebug* line_it = &line_seg_out[i];
			std::stringstream line_id;
			line_id << i;
			//folder = DATA_OUTPUT_PATH;
			//folder = output_path + "plane_normals\\";
			std::string whole_path = sub_path + file_name + "_" + line_id.str() + file_type;
			if (save_type == 0)
			{
				ModuleStruct::Point2fArray line_Points;
				line_Points.clear();
				line_Points.resize(3);
				line_Points[0] = line_it->line_seg_start;
				line_Points[1] = line_it->line_center;
				line_Points[2] = line_it->line_seg_end;
				FeaturesIO::SavePoint2fDataWithDelimiter(whole_path, " ", line_Points);
			}
			else if (save_type == 1)
			{
				ModuleStruct::Point2fArray line_Points;
				line_Points.clear();
				line_Points.resize(1);
				line_Points[0] = line_it->line_direction;
				FeaturesIO::SavePoint2fDataWithDelimiter(whole_path, " ", line_Points);
			}
		}
		return 0;
	}

	/**
	* \brief save line segmention output  line direction ,center , endpoint to file
	* @param [in] save_type,  0:center and end points, 1: direction, else, center and end points together
	* @param [in] sub_path,  out put path, with endof "/" or"\\"
	* @param [in] file_name, data  file  to save
	* @param [in] origin_normal, input 2D point cloud normal in 3d space
	* @param [in] origin_center, input 2D point cloud center in 3d space
	* @param [in] line_seg_out, all lines output
	* return = 0 success, else failed
	* */
	static inline int save_line_direction_end3D(int save_type, const std::string sub_path, const std::string file_name, \
		const ModuleStruct::Point3f origin_normal, const ModuleStruct::Point3f origin_center, ModuleStruct::Vector<Line2DSegOutItemDebug> line_seg_out)
	{
		if (IOData::createDirectory(sub_path))
		{
			log_error("createDirectory %s failed", sub_path.c_str());
			return -1;
		}
		std::string file_type;
		file_type = ".txt";

		ModuleStruct::Point3f nz(0.0f, 0.0f, 1.0f);
		bool isNeedRot = !Util_Math::vec3_are_same(origin_normal, nz); // is need rotation
		ModuleStruct::CMat rot_mat(3, 3, CV_32F);

		if (isNeedRot)
		{
			rot_mat = Util_Math::CreateRotationMat4E(nz, origin_normal);
		}

		if (save_type == 0)
		{
			std::vector<ModuleStruct::Point3f> line_start(line_seg_out.size());
			//line_start.clear();
			std::vector<ModuleStruct::Point3f> line_end(line_seg_out.size());
			//line_end.clear();
			std::string whole_path = sub_path + file_name + file_type;
			for (unsigned int i = 0; i < line_seg_out.size(); i++) {
				Line2DSegOutItemDebug* line_it = &line_seg_out[i];
				line_start[i] = util_line_rot::point2d_to_3d(isNeedRot, line_it->line_seg_start, rot_mat, origin_center);
				line_end[i] = util_line_rot::point2d_to_3d(isNeedRot, line_it->line_seg_end, rot_mat, origin_center);
			}
			FeaturesIO::SavePoint3fDataNormalWithDelimiter(whole_path, " ", line_start, line_end);
			//log_info("line_start size =%d", line_start.size());
			return 0;
		}

		for (unsigned int i = 0; i < line_seg_out.size(); i++) {
			Line2DSegOutItemDebug* line_it = &line_seg_out[i];
			std::stringstream line_id;
			line_id << i;
			//folder = DATA_OUTPUT_PATH;
			//folder = output_path + "plane_normals\\";
			std::string whole_path = sub_path + file_name + "_" + line_id.str() + file_type;
			if (save_type == 1)
			{
				ModuleStruct::Point3fArray line_Points;
				line_Points.clear();
				line_Points.resize(3);
				line_Points[0] = util_line_rot::point2d_to_3d(isNeedRot, line_it->line_seg_start,rot_mat,origin_center);
				line_Points[1] = util_line_rot::point2d_to_3d(isNeedRot, line_it->line_center, rot_mat, origin_center);
				line_Points[2] = util_line_rot::point2d_to_3d(isNeedRot, line_it->line_seg_end, rot_mat, origin_center);
				IOData::SavePoint3fDataWithDelimiter(whole_path, " ", line_Points);
			}
			else if (save_type == 2)
			{
				ModuleStruct::Point3fArray line_Points;
				line_Points.clear();
				line_Points.resize(1);
				line_Points[0] = util_line_rot::point2d_to_3d(isNeedRot, line_it->line_direction, rot_mat, origin_center);
				IOData::SavePoint3fDataWithDelimiter(whole_path, " ", line_Points);
			}
		}
		return 0;
	}

	/**
	* \brief save line segmention output xyz to file
	* @param [in] sub_path,  out put path, with endof "/" or"\\"
	* @param [in] sub_file_name, input file name
	* @param [in] delimiter, delimiter of save data
	* @param [in] file_type,  save file suffix
	* @param [in] line_seg_out,  segment out put  of all lines in 3D space
	* return > = 0 success, else failed
	* */
	static inline int save_line3d_xyz(
		const std::string sub_path, 
		const std::string sub_file_name, 
		const std::string delimiter,
		const std::string file_type, 
		ModuleStruct::Vector<Line3DSegOutItemDebug> line_seg_out)
	{
		if (IOData::createDirectory(sub_path) != 0)
		{
			std::cout << "can not create path: " << sub_path << std::endl;
			return -1;
		}

		for (unsigned int i = 0; i < line_seg_out.size(); i++)
		{
			Line3DSegOutItemDebug* line_it = &line_seg_out[i];
			std::stringstream line_id;
			line_id << i;
			std::string test_output_path = sub_path + sub_file_name + "_" + line_id.str() + file_type;
			// save line3d point cloud
			IOData::SavePoint3fDataWithDelimiter(test_output_path, delimiter, line_seg_out[i].points);
		}
		return 0;
	}
	static inline int save_plane_line3d_xyz(const std::string sub_path, const std::string sub_file_name, const std::string delimiter, \
		const std::string file_type, const int plane_idx, ModuleStruct::Vector<Line3DSegOutItemDebug> line_seg_out)
	{
		if (IOData::createDirectory(sub_path) != 0)
		{
			std::cout << "can not create path: " << sub_path << std::endl;
			return -1;
		}
		std::stringstream plane_id;
		plane_id << plane_idx;
		for (unsigned int i = 0; i < line_seg_out.size(); i++)
		{
			Line3DSegOutItemDebug* line_it = &line_seg_out[i];
			std::stringstream line_id;
			line_id << i;
			std::string test_output_path = sub_path + sub_file_name + "_p"+ plane_id.str()+"_l" + line_id.str() + file_type;
			// save line3d point cloud
			IOData::SavePoint3fDataWithDelimiter(test_output_path, delimiter, line_seg_out[i].points);
		}
		return 0;
	}

	/**
	* \brief save line segmention output  line direction ,center , endpoint to file
	* @param [in] save_type,  0:center and end points, 1: direction, else, center and end points together
	* @param [in] sub_path,  out put path, with endof "/" or"\\"
	* @param [in] file_name, data  file  to save
	* @param [in] line_seg_out, all lines output
	* return = 0 success, else failed
	* */
	static inline int save_line3d_direction_end(
		int save_type, 
		const std::string sub_path, const std::string file_name, 
		ModuleStruct::Vector<Line3DSegOutItemDebug> line_seg_out)
	{
		if (IOData::createDirectory(sub_path))
		{
			log_error("createDirectory %s failed", sub_path.c_str());
			return -1;
		}
		std::string file_type;
		file_type = ".txt";

		if (save_type == 0)
		{
			std::vector<ModuleStruct::Point3f> line_start(line_seg_out.size());
			std::vector<ModuleStruct::Point3f> line_end(line_seg_out.size());
			std::string whole_path = sub_path + file_name + file_type;
			for (unsigned int i = 0; i < line_seg_out.size(); i++) {
				Line3DSegOutItemDebug* line_it = &line_seg_out[i];
				line_start[i] = line_it->line_seg_start;
				line_end[i] = line_it->line_seg_end;
			}
			FeaturesIO::SavePoint3fDataNormalWithDelimiter(whole_path, " ", line_start, line_end);
			//log_info("line_start size =%d", line_start.size());
			return 0;
		}

		for (unsigned int i = 0; i < line_seg_out.size(); i++) {
			Line3DSegOutItemDebug* line_it = &line_seg_out[i];
			std::stringstream line_id;
			line_id << i;
			//folder = DATA_OUTPUT_PATH;
			//folder = output_path + "plane_normals\\";
			std::string whole_path = sub_path + file_name + "_" + line_id.str() + file_type;
			if (save_type == 1)
			{
				ModuleStruct::Point3fArray line_Points;
				line_Points.clear();
				line_Points.resize(3);
				line_Points[0] = line_it->line_seg_start;
				line_Points[1] = line_it->line_center;
				line_Points[2] = line_it->line_seg_end;
				IOData::SavePoint3fDataWithDelimiter(whole_path, " ", line_Points);
			}
			else if (save_type == 2)
			{
				ModuleStruct::Point3fArray line_Points;
				line_Points.clear();
				line_Points.resize(1);
				line_Points[0] =line_it->line_direction;
				IOData::SavePoint3fDataWithDelimiter(whole_path, " ", line_Points);
			}
		}
		return 0;
	}

	/**
	* \brief   Output of line segmentation  in 3D space with  configure file and debug info
	* @param [in]  output_path, debug output path
	* @param [in]  config_file, input config file
	* @param [in]  Planes_normal, normal of multiple planes
	* @param [in]  Planes_normal, center of multiple planes
	* @param [in]  planes_input_data, input data in 2D space of multiple planes
	* @param [out] planes_line_seg, pointer of line extraction output in 3D space
	* @return bool operation success or not
	*/
	static inline bool Features_GetLinePoints3D(const std::string output_path, const std::string config_file, const ModuleStruct::Point3fArray& planes_normal, const ModuleStruct::Point3fArray& planes_center, \
		ModuleStruct::Vector<ModuleStruct::Vector<ModuleStruct::Point2fArray>>& planes_input_data, ModuleStruct::Vector<Line3DSegOutItemDebug>& planes_line_seg)
	{

		if ((planes_normal.size() != planes_center.size()) || (planes_normal.size() != planes_input_data.size()))
		{
			log_error("input normal size (%d),center size (%d), input_data size (%d) is not equeal ", planes_normal.size(), planes_center.size(), planes_input_data.size());
			return false;
		}

		Line2DConfigParse config_it;
		config_it.GetLineConfigure(config_file);
		PixelSize pixel_size = config_it.getPixelConfig();


#ifdef SAVE_OUTPUT_FILE_DEBUG
		std::string save_file = output_path + "LineCfgInfo.txt";
		config_it.SaveConfigInfo(save_file);
		save_file = output_path + "LineCfgParseinfo.txt";
		//config_it.SaveConfigParams(config_file, config_it.getSysConfigParams());
		config_it.SaveLineConfigParams(save_file);
#endif

		//ModuleStruct::Vector<Line2DSegOutItemDebug>plane_line_seg;
		planes_line_seg.clear();

		// ModuleStruct::Point2fArray line_input_data; To be optimised for parallel operations
		for (int i = 0; i < planes_input_data.size(); i++)
		{
			ModuleStruct::Vector<ModuleStruct::Point2fArray> plane_input_data = planes_input_data[i];
			ModuleStruct::Vector<Line3DSegOutItemDebug> plane_line3d_seg; // all lines of one plane
			plane_line3d_seg.clear();
			ModuleStruct::Point3f cur_plane_normal = planes_normal[i];
			ModuleStruct::Point3f cur_plane_center = planes_center[i];
			for (int j = 0; j < plane_input_data.size(); j++)
			{
				ModuleStruct::Vector<Line2DSegOutItemDebug>line2d_seg;
				ModuleStruct::Vector<Line3DSegOutItemDebug>line3d_seg;
				ModuleStruct::Point2fArray line_input_data = plane_input_data[j];
				//DW_line_extract2D* line_extract = new DW_line_extract2D();
				void* line_handle = Features_LineExtractCreateHandle();
				Features_SetLineConfig(line_handle, config_it.getSysConfigParams(), config_it.getLineExtrConfig(), \
					config_it.getLineThreshold(), pixel_size);

#ifdef SAVE_OUTPUT_FILE_DEBUG
				//line_extract->SetConfigure(config_it.getSysConfigParams(), config_it.getLineExtrConfig(), config_it.getLineThreshold(), pixel_size, config_it.getLineDebugConfig());
				Features_SetLineDbgPath(line_handle, output_path);
				Features_SetLineConfig(line_handle, config_it.getSysConfigParams(), config_it.getLineExtrConfig(), \
					config_it.getLineThreshold(), pixel_size, config_it.getLineDebugConfig());
#endif
				Features_GetLinePoints2D(line_handle, line_input_data, line2d_seg);
				if (!line2d_seg.empty())
				{
					util_line_rot::line2d_to_3d(line_input_data, cur_plane_normal, cur_plane_center, line2d_seg, line3d_seg);
					for (int k = 0; k < line3d_seg.size(); k++)
					{
						line3d_seg[k].line_plane_idx = i;
					}
					plane_line3d_seg.insert(plane_line3d_seg.end(), line3d_seg.begin(), line3d_seg.end());
					
				}
			}

			if (!plane_line3d_seg.empty())
			{
				planes_line_seg.insert(planes_line_seg.end(), plane_line3d_seg.begin(), plane_line3d_seg.end());
			}
		}
		return true;
	}

	static inline bool Features_GetLinePoints3D_Single(const std::string output_path, const std::string config_file, const ModuleStruct::Point3f& planes_normal, const ModuleStruct::Point3f& planes_center, \
		ModuleStruct::Vector<ModuleStruct::Point2fArray>& planes_input_data, ModuleStruct::Vector<Line3DSegOutItemDebug>& planes_line_seg)
	{

		Line2DConfigParse config_it;
		config_it.GetLineConfigure(config_file);
		PixelSize pixel_size = config_it.getPixelConfig();


#ifdef SAVE_OUTPUT_FILE_DEBUG
		std::string save_file = output_path + "LineCfgInfo.txt";
		config_it.SaveConfigInfo(save_file);
		save_file = output_path + "LineCfgParseinfo.txt";
		//config_it.SaveConfigParams(config_file, config_it.getSysConfigParams());
		config_it.SaveLineConfigParams(save_file);
#endif

		//ModuleStruct::Vector<Line2DSegOutItemDebug>plane_line_seg;
		planes_line_seg.clear();

		// ModuleStruct::Point2fArray line_input_data; To be optimised for parallel operations
		ModuleStruct::Vector<ModuleStruct::Point2fArray> plane_input_data = planes_input_data;
		ModuleStruct::Vector<Line3DSegOutItemDebug> plane_line3d_seg; // all lines of one plane
		plane_line3d_seg.clear();
		ModuleStruct::Point3f cur_plane_normal = planes_normal;
		ModuleStruct::Point3f cur_plane_center = planes_center;
		for (int j = 0; j < plane_input_data.size(); j++)
		{
			ModuleStruct::Vector<Line2DSegOutItemDebug>line2d_seg;
			ModuleStruct::Vector<Line3DSegOutItemDebug>line3d_seg;
			ModuleStruct::Point2fArray line_input_data = plane_input_data[j];
			//DW_line_extract2D* line_extract = new DW_line_extract2D();
			void* line_handle = Features_LineExtractCreateHandle();
			Features_SetLineConfig(line_handle, config_it.getSysConfigParams(), config_it.getLineExtrConfig(), \
				config_it.getLineThreshold(), pixel_size);

#ifdef SAVE_OUTPUT_FILE_DEBUG
			//line_extract->SetConfigure(config_it.getSysConfigParams(), config_it.getLineExtrConfig(), config_it.getLineThreshold(), pixel_size, config_it.getLineDebugConfig());
			Features_SetLineDbgPath(line_handle, output_path);
			Features_SetLineConfig(line_handle, config_it.getSysConfigParams(), config_it.getLineExtrConfig(), \
				config_it.getLineThreshold(), pixel_size, config_it.getLineDebugConfig());
#endif
			Features_GetLinePoints2D(line_handle, line_input_data, line2d_seg);
			if (!line2d_seg.empty())
			{
				util_line_rot::line2d_to_3d(line_input_data, cur_plane_normal, cur_plane_center, line2d_seg, line3d_seg);
				plane_line3d_seg.insert(plane_line3d_seg.end(), line3d_seg.begin(), line3d_seg.end());

			}
		}

		if (!plane_line3d_seg.empty())
		{
			planes_line_seg.insert(planes_line_seg.end(), plane_line3d_seg.begin(), plane_line3d_seg.end());
		}
		return true;
	}

	static inline bool Features_GetLinePoints3D_Single(///T
		const std::string config_file,
		const ModuleStruct::Point3f& planes_normal,
		const ModuleStruct::Point3f& planes_center,
		ModuleStruct::Vector<ModuleStruct::Point2fArray>& planes_input_data, //i
		ModuleStruct::Vector<Line3DSegOutItemDebug>& planes_line_seg)///o
	{
		Line2DConfigParse config_it;
		config_it.GetLineConfigure(config_file);///T
		PixelSize pixel_size = config_it.getPixelConfig();
		planes_line_seg.clear();
		// ModuleStruct::Point2fArray line_input_data; To be optimised for parallel operations
		ModuleStruct::Vector<ModuleStruct::Point2fArray> plane_input_data = planes_input_data;
		ModuleStruct::Vector<Line3DSegOutItemDebug> plane_line3d_seg; // all lines of one plane
		plane_line3d_seg.clear();

		ModuleStruct::Point3f cur_plane_normal = planes_normal;
		ModuleStruct::Point3f cur_plane_center = planes_center;
		for (int j = 0; j < plane_input_data.size(); j++)/// go through 2d contours
		{
			ModuleStruct::Vector<Line2DSegOutItemDebug>line2d_seg;
			ModuleStruct::Vector<Line3DSegOutItemDebug>line3d_seg;
			ModuleStruct::Point2fArray line_input_data = plane_input_data[j];
			//DW_line_extract2D* line_extract = new DW_line_extract2D();

			void* line_handle = Features_LineExtractCreateHandle();
#ifdef SAVE_OUTPUT_FILE_DEBUG
				Features_SetLineConfig(line_handle,
					config_it.getSysConfigParams(),
					config_it.getLineExtrConfig(),
					config_it.getLineThreshold(),
					pixel_size,
					config_it.getLineDebugConfig());
#else
			Features_SetLineConfig(line_handle,
				config_it.getSysConfigParams(),
				config_it.getLineExtrConfig(),
				config_it.getLineThreshold(),
				pixel_size);
#endif				

			Features_GetLinePoints2D(line_handle, line_input_data,  line2d_seg);///T

			if (!line2d_seg.empty())
			{
				util_line_rot::line2d_to_3d(line_input_data, cur_plane_normal, cur_plane_center, 
											line2d_seg, line3d_seg);
				plane_line3d_seg.insert(plane_line3d_seg.end(), line3d_seg.begin(), line3d_seg.end());
			}
		}

		if (!plane_line3d_seg.empty())
		{
			planes_line_seg.insert(planes_line_seg.end(), plane_line3d_seg.begin(), plane_line3d_seg.end());
		}
		return true;
	}


	/**
	* \brief   Output of line segmentations by plane ids in 3D space with  configure file and debug info
	* @param [in]  handle, line extraction handle
	* @param [in]  config_file, input config file
	* @param [in]  Planes_normal, normal of multiple planes
	* @param [in]  Planes_normal, center of multiple planes
	* @param [in]  planes_input_data, input data in 2D space of multiple planes
	* @param [out] planes_line_seg, pointer of line extraction output in 3D space
	* @return bool operation success or not
	*/
	static inline bool Features_GetPlaneLines3D(const std::string output_path, const std::string config_file, const ModuleStruct::Point3fArray& planes_normal, const ModuleStruct::Point3fArray& planes_center, \
		ModuleStruct::Vector<ModuleStruct::Vector<ModuleStruct::Point2fArray>>& planes_input_data, ModuleStruct::Vector<ModuleStruct::Vector<Line3DSegOutItemDebug>>&planes_lines)
	{

		if ((planes_normal.size() != planes_center.size()) || (planes_normal.size() != planes_input_data.size()))
		{
			log_error("input normal size (%d),center size (%d), input_data size (%d) is not equeal ", planes_normal.size(), planes_center.size(), planes_input_data.size());
			return false;
		}
		planes_lines.clear();
		planes_lines.resize(planes_input_data.size());
		Line2DConfigParse config_it;
		config_it.GetLineConfigure(config_file);
		PixelSize pixel_size = config_it.getPixelConfig();


#ifdef SAVE_OUTPUT_FILE_DEBUG
		std::string save_file = output_path + "LineCfgInfo.txt";
		config_it.SaveConfigInfo(save_file);
		save_file = output_path + "LineCfgParseinfo.txt";
		//config_it.SaveConfigParams(config_file, config_it.getSysConfigParams());
		config_it.SaveLineConfigParams(save_file);
#endif

		//ModuleStruct::Vector<Line2DSegOutItemDebug>plane_line_seg;
		//planes_line_seg.clear();

		// ModuleStruct::Point2fArray line_input_data; To be optimised for parallel operations
		for (int i = 0; i < planes_input_data.size(); i++)
		{
			ModuleStruct::Vector<ModuleStruct::Point2fArray> plane_input_data = planes_input_data[i];
			ModuleStruct::Vector<Line3DSegOutItemDebug> plane_line3d_seg; // all lines of one plane
			plane_line3d_seg.clear();
			ModuleStruct::Point3f cur_plane_normal = planes_normal[i];
			ModuleStruct::Point3f cur_plane_center = planes_center[i];
			for (int j = 0; j < plane_input_data.size(); j++)
			{
				ModuleStruct::Vector<Line2DSegOutItemDebug>line2d_seg;
				ModuleStruct::Vector<Line3DSegOutItemDebug>line3d_seg;
				ModuleStruct::Point2fArray line_input_data = plane_input_data[j];
				//DW_line_extract2D* line_extract = new DW_line_extract2D();
				void* line_handle = Features_LineExtractCreateHandle();
				Features_SetLineConfig(line_handle, config_it.getSysConfigParams(), config_it.getLineExtrConfig(), \
					config_it.getLineThreshold(), pixel_size);

#ifdef SAVE_OUTPUT_FILE_DEBUG
				//line_extract->SetConfigure(config_it.getSysConfigParams(), config_it.getLineExtrConfig(), config_it.getLineThreshold(), pixel_size, config_it.getLineDebugConfig());
				Features_SetLineDbgPath(line_handle, output_path);
				Features_SetLineConfig(line_handle, config_it.getSysConfigParams(), config_it.getLineExtrConfig(), \
					config_it.getLineThreshold(), pixel_size, config_it.getLineDebugConfig());
#endif
				Features_GetLinePoints2D(line_handle, line_input_data, line2d_seg);
				if (!line2d_seg.empty())
				{
					util_line_rot::line2d_to_3d(line_input_data, cur_plane_normal, cur_plane_center, line2d_seg, line3d_seg);

					plane_line3d_seg.insert(plane_line3d_seg.end(), line3d_seg.begin(), line3d_seg.end());
				}
			}
			planes_lines[i].assign(plane_line3d_seg.begin(), plane_line3d_seg.end());
		}
		return true;
	}

	/**
	* \brief   Output of line segmentations by plane ids in 3D space with  configure file and debug info
	* @param [in]  handle, line extraction handle
	* @param [in]  config_file, input config file
	* @param [in]  Planes_normal, normal of multiple planes
	* @param [in]  Planes_normal, center of multiple planes
	* @param [in]  planes_input_data, input data in 2D space of multiple planes
	* @param [out] planes_line_seg, pointer of line extraction output in 3D space
	* @return bool operation success or not
	*/
	static inline bool Features_Single_GetPlaneLines3D(
		const std::string output_path, 
		const std::string config_file,
		const ModuleStruct::Point3f& planes_normal, 
		const ModuleStruct::Point3f& planes_center, 
		ModuleStruct::Vector<ModuleStruct::Point2fArray>& planes_input_data, 
		ModuleStruct::Vector<Line3DSegOutItemDebug>& planes_lines)
	{
		Line2DConfigParse config_it;
		config_it.GetLineConfigure(config_file);
		PixelSize pixel_size = config_it.getPixelConfig();
		

#ifdef SAVE_OUTPUT_FILE_DEBUG
		std::string save_file = output_path + "LineCfgInfo.txt";
		config_it.SaveConfigInfo(save_file);
		save_file = output_path + "LineCfgParseinfo.txt";
		//config_it.SaveConfigParams(config_file, config_it.getSysConfigParams());
		config_it.SaveLineConfigParams(save_file);
#endif

		//ModuleStruct::Vector<Line2DSegOutItemDebug>plane_line_seg;
		//planes_line_seg.clear();

		// ModuleStruct::Point2fArray line_input_data; To be optimised for parallel operations
		ModuleStruct::Vector<ModuleStruct::Point2fArray> plane_input_data = planes_input_data;
		ModuleStruct::Vector<Line3DSegOutItemDebug> plane_line3d_seg; // all lines of one plane
		plane_line3d_seg.clear();
		ModuleStruct::Point3f cur_plane_normal = planes_normal;
		ModuleStruct::Point3f cur_plane_center = planes_center;

		for (int j = 0; j < plane_input_data.size(); j++)
		{
			
			ModuleStruct::Vector<Line2DSegOutItemDebug>line2d_seg;
			ModuleStruct::Vector<Line3DSegOutItemDebug>line3d_seg;
			ModuleStruct::Point2fArray line_input_data = plane_input_data[j];
			            //Vector<Point2f>
			//DW_line_extract2D* line_extract = new DW_line_extract2D();
			void* line_handle = Features_LineExtractCreateHandle();
			
			Features_SetLineConfig(line_handle, 
				                   config_it.getSysConfigParams(), 
				                   config_it.getLineExtrConfig(),
								   config_it.getLineThreshold(), 
								   pixel_size);
#ifdef SAVE_OUTPUT_FILE_DEBUG
			//line_extract->SetConfigure(config_it.getSysConfigParams(), config_it.getLineExtrConfig(), config_it.getLineThreshold(), pixel_size, config_it.getLineDebugConfig());
			Features_SetLineDbgPath(line_handle, output_path);
			Features_SetLineConfig(line_handle, config_it.getSysConfigParams(), config_it.getLineExtrConfig(), \
				config_it.getLineThreshold(), pixel_size, config_it.getLineDebugConfig());
#endif
			Features_GetLinePoints2D(line_handle, ///
									 line_input_data, 
									 line2d_seg);
			if (!line2d_seg.empty())
			{
				util_line_rot::line2d_to_3d(line_input_data, 
											cur_plane_normal, 
											cur_plane_center, 
					                        line2d_seg, 
											line3d_seg);
				plane_line3d_seg.insert(plane_line3d_seg.end(), line3d_seg.begin(), line3d_seg.end());
			}
		}
		planes_lines.assign(plane_line3d_seg.begin(), plane_line3d_seg.end());
		
		return true;
	}

}


#endif /*_UTIL_LINE_TEST_HPP_*/
