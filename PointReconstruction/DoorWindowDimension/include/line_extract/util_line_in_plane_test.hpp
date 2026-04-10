#ifndef _UTIL_LINE_IN_PLANE_TEST_HPP_
#define _UTIL_LINE_IN_PLANE_TEST_HPP_

#include <string>
#include <codecvt> //ut8
#include <iostream>
#include "config.h"
#include "InOutData.h"
#include "ModuleStruct.hpp"
#include "plane_seg_inf.h"
#include "edge_dection/detc_edge_api.h"
#include "line_extract/line_extract_api.h"
//#include "util_math.hpp"
#include "../../../Common/MathOperation.h"
#include "util_line_test.hpp"


namespace util_line_in_plane_test
{
	
	/**
	* \brief get plane segmentation input default file path (no input parameters)
	* @param [out] output_path,  default out put path
	* @param [out] input_file_name,default  input file path+name
	* @param [out] config_file_name, default configure file name
	* */
	static inline void tool_defaut_input_process(std::string &output_path,\
		std::string &input_file_name, std::string &config_file_name, std::string& line_config_file)
	{
		output_path = DATA_OUTPUT_PATH;
		std::string file_name, folder, xml_path;
		file_name = "load_path.xml";
		folder = DATA_INPUT_PATH;
		xml_path = folder + file_name;
		cv::FileStorage load_path_xml(xml_path, cv::FileStorage::READ);
		input_file_name = folder + (std::string)load_path_xml["scene_data"];
		config_file_name = folder + (std::string)load_path_xml["configure"];
		line_config_file = folder + (std::string)load_path_xml["line_cfg"];

	}

	/**
	* \brief get plane segmentation input file path by input parameters
	* @param [in] argc, number of input parameters
	* @param [in] argv, pointer of input data address
	* @param [out] output_path,  default out put path, with endof "/" or"\\"
	* @param [out] input_file_name,default  input file path+name
	* @param [out] config_file_name, configure file path+name
	* @param [out] line_config_file, line extract configure file path+name
	* return 0 is ok, <0 is error
	* */
	static inline int tool_arc_parse(const int argc, char** argv, std::string& output_path, \
		std::string& input_file_name, std::string& config_file_name, std::string& line_config_file)
	{
		if (IOData::parse_argument(argc, argv, "-i", input_file_name) <= 0)
		{
			printf("No Input File given\n");
			//printHelp();
			return -1;
		}

		if (IOData::parse_argument(argc, argv, "-o", output_path) <= 0)
		{
			printf("No Output Path given\n");
			//printHelp();
			return -1;
		}

		if (IOData::parse_argument(argc, argv, "-c", config_file_name) <= 0)
		{
			config_file_name = "config_plane.ini";
		}

		if (IOData::parse_argument(argc, argv, "-l", line_config_file) <= 0)
		{
			line_config_file = "config_line.ini";
		}

		return 0;
	}


	/**
	* \brief  get edge of all the planes in 2D space
	* @param [in] output_path, out put path
	* @param [in] pxl_size,  pxl_size  of  detc_edge2d
	* @param [in] plane_seg_out,  plane segmentation output
	* @param [in] [out] plane_2Dcontours_array,  planes edge in 2D space
	* */
	static inline void get_planes_edge2d(const std::string output_path, double pxl_size, Vector<PlaneSegOutput> plane_seg_out, Vector<Vector<Point2fArray>> &plane_2Dcontours_array)
	{
		//float edge2d_pxl_size = (voxel_size.length_x_of_voxel + voxel_size.length_y_of_voxel + voxel_size.length_z_of_voxel) / 3 / 4;
		plane_2Dcontours_array.clear();
		plane_2Dcontours_array.resize(plane_seg_out.size());
		//log_warn("edge2d_pxl_size =%f", pxl_size);
		for (int i = 0; i < plane_seg_out.size(); i++)
		{
			PlaneSegOutput* plane_it = &plane_seg_out[i];
			void* edge_handle = DW_Features_EdgeDetcCreateHandle();
			DW_Features_getOrigMapOfContours2D(output_path, edge_handle, plane_it->plane_normal, plane_it->plane_center, &plane_it->points, pxl_size, plane_2Dcontours_array[i]);
			DW_Features_EdgeDetcDestroyHandle(&edge_handle);
		}
	}

	static inline void get_planes_edge2d(double pxl_size, Vector<PlaneSegOutput> plane_seg_out, Vector<Vector<Point2fArray>>& plane_2Dcontours_array)
	{
		//float edge2d_pxl_size = (voxel_size.length_x_of_voxel + voxel_size.length_y_of_voxel + voxel_size.length_z_of_voxel) / 3 / 4;
		plane_2Dcontours_array.clear();
		plane_2Dcontours_array.resize(plane_seg_out.size());
		//log_warn("edge2d_pxl_size =%f", pxl_size);
		for (int i = 0; i < plane_seg_out.size(); i++)
		{
			PlaneSegOutput* plane_it = &plane_seg_out[i];
			void* edge_handle = DW_Features_EdgeDetcCreateHandle();
			DW_Features_getOrigMapOfContours2D(edge_handle, plane_it->plane_normal, plane_it->plane_center, &plane_it->points, pxl_size, plane_2Dcontours_array[i]);
			DW_Features_EdgeDetcDestroyHandle(&edge_handle);
		}
	}

	static inline void get_planes_edge2d(double pxl_size, const std::vector<ModuleStruct::Vector<ModuleStruct::Point3f>> input_plane_points, const std::vector<ModuleStruct::Point3f>& input_plane_normal, const std::vector<ModuleStruct::Point3f>& input_plane_center, Vector<Vector<Point2fArray>>& plane_2Dcontours_array)
	{
		//float edge2d_pxl_size = (voxel_size.length_x_of_voxel + voxel_size.length_y_of_voxel + voxel_size.length_z_of_voxel) / 3 / 4;
		plane_2Dcontours_array.clear();
		plane_2Dcontours_array.resize(input_plane_points.size());
		//log_warn("edge2d_pxl_size =%f", pxl_size);
		for (int i = 0; i < input_plane_points.size(); i++)
		{
			ModuleStruct::Vector<ModuleStruct::Point3f> points = input_plane_points[i];
			void* edge_handle = DW_Features_EdgeDetcCreateHandle();
			DW_Features_getOrigMapOfContours2D(edge_handle, input_plane_normal[i], input_plane_center[i], &points, pxl_size, plane_2Dcontours_array[i]);
			DW_Features_EdgeDetcDestroyHandle(&edge_handle);
		}
	}

	static inline void get_plane_edge2d(double pxl_size, 
		ModuleStruct::Vector<ModuleStruct::Point3f> input_plane_points,
		const ModuleStruct::Point3f& input_plane_normal, 
		const ModuleStruct::Point3f& input_plane_center, 
		Vector<Point2fArray>& plane_2Dcontours_array)
		//Vector<Vector<Point2f>>
	{
		//float edge2d_pxl_size = (voxel_size.length_x_of_voxel + voxel_size.length_y_of_voxel + voxel_size.length_z_of_voxel) / 3 / 4;
		plane_2Dcontours_array.clear();
		plane_2Dcontours_array.resize(input_plane_points.size());
		//log_warn("edge2d_pxl_size =%f", pxl_size);
		ModuleStruct::Vector<ModuleStruct::Point3f> points = input_plane_points;
		void* edge_handle = DW_Features_EdgeDetcCreateHandle();

		DW_Features_getOrigMapOfContours2D(edge_handle, 
										input_plane_normal, 
			                            input_plane_center,
			                            &points,///i 
			                            pxl_size, 
			                            plane_2Dcontours_array///o
										);
		
		DW_Features_EdgeDetcDestroyHandle(&edge_handle);
	}

	/**
	* \brief  get edge of all the planes in 2D space
	* @param [in] output_path, out put path
	* @param [in] pxl_size,  pxl_size  of  detc_edge2d
	* @param [in] plane_seg_out,  plane segmentation output
	* @param [in] [out] plane_2Dcontours_array,  planes edge in 2D space
	* */
	static inline void get_planes_edge2d_bn(const std::string output_path, const double pxl_size, Vector<PlaneSegOutput>&plane_seg_out, Vector<Vector<Point2fArray>>& plane_2Dcontours_array)
	{
		//float edge2d_pxl_size = (voxel_size.length_x_of_voxel + voxel_size.length_y_of_voxel + voxel_size.length_z_of_voxel) / 3 / 4;
		plane_2Dcontours_array.clear();
		plane_2Dcontours_array.resize(plane_seg_out.size());
		for (int i = 0; i < plane_seg_out.size(); i++)
		{
			PlaneSegOutput* plane_it = &plane_seg_out[i];
			void* edge_handle = DW_Features_EdgeDetcCreateHandle();
			plane_2Dcontours_array[i].clear();
			plane_2Dcontours_array[i].resize(1);
			DW_Features_getOrigMapOfContours2D_bn(output_path, edge_handle, plane_it->plane_normal, plane_it->plane_center, &plane_it->points, pxl_size, plane_2Dcontours_array[i][0]);
			DW_Features_EdgeDetcDestroyHandle(&edge_handle);
		}
	}

	/**
	* \brief  get edge of all the planes in 2D space
	* @param [in] output_path, out put path
	* @param [in] pxl_size,  pxl_size  of  detc_edge2d
	* @param [in] plane_seg_out,  plane segmentation output
	* @param [in] [out] plane_2Dcontours_array,  planes edge in 2D space
	* */
	static inline void get_planes_edge3d(const std::string output_path, const double pxl_size, Vector<PlaneSegOutput>&plane_seg_out, Vector<Vector<Point3fArray>> &plane_3Dcontours_array)
	{
		//float edge2d_pxl_size = (voxel_size.length_x_of_voxel + voxel_size.length_y_of_voxel + voxel_size.length_z_of_voxel) / 3 / 6;
		plane_3Dcontours_array.clear();
		plane_3Dcontours_array.resize(plane_seg_out.size());

		for (int i = 0; i < plane_seg_out.size(); i++)
		{
			PlaneSegOutput* plane_it = &plane_seg_out[i];
			void* edge_handle = DW_Features_EdgeDetcCreateHandle();
			DW_Features_getOrigMapOfContours3D(output_path, edge_handle, plane_it->plane_normal, plane_it->plane_center, &plane_it->points, pxl_size, plane_3Dcontours_array[i]);
			DW_Features_EdgeDetcDestroyHandle(&edge_handle);
		}

	}

	static inline void get_plane_edge3d(const double pxl_size, const std::vector<Point3f>& points, const Point3f& normal, const Point3f& center, std::vector<Point3fArray>& plane_3Dcontours_array)
	{
		void* edge_handle = DW_Features_EdgeDetcCreateHandle();
		
		DW_Features_getOrigMapOfContours3D(edge_handle,///T 
										normal, center, points, pxl_size, 
			                            plane_3Dcontours_array);
		
		DW_Features_EdgeDetcDestroyHandle(&edge_handle);
		
	}

	/**
* \brief  get edge of all the planes in 2D space
* @param [in] output_path, out put path
* @param [in] pxl_size,  pxl_size  of  detc_edge2d
* @param [in] plane_seg_out,  plane segmentation output
* @param [in] [out] plane_2Dcontours_array,  planes edge in 2D space
* */
	static inline void get_planes_edge3d_bn(const std::string output_path, const double pxl_size, Vector<PlaneSegOutput>&plane_seg_out, Vector<Vector<Point3fArray>>& plane_3Dcontours_array)
	{
		//float edge2d_pxl_size = (voxel_size.length_x_of_voxel + voxel_size.length_y_of_voxel + voxel_size.length_z_of_voxel) / 3 / 6;
		plane_3Dcontours_array.clear();
		plane_3Dcontours_array.resize(plane_seg_out.size());

		for (int i = 0; i < plane_seg_out.size(); i++)
		{
			PlaneSegOutput* plane_it = &plane_seg_out[i];
			void* edge_handle = DW_Features_EdgeDetcCreateHandle();
			plane_3Dcontours_array[i].clear();
			plane_3Dcontours_array[i].resize(1);
			DW_Features_getOrigMapOfContours3D(output_path, edge_handle, plane_it->plane_normal, plane_it->plane_center, &plane_it->points, pxl_size, plane_3Dcontours_array[i][0]);
			DW_Features_EdgeDetcDestroyHandle(&edge_handle);
		}

	}

	static inline bool line_large_cp(Line3DSegOutItemDebug L1, Line3DSegOutItemDebug L2)
	{
		return (L1.points.size() > L2.points.size());
	}

	/**
	* \brief  get lines of all the planes' edge in 2D space
	* @param [in] output_path, out put path
	* @param [in] line_cfg_file,  config file for line extration
	* @param [in] plane_2Dcontours_array, plane edge detction resulat in 2D space
	* @param [in] [out] plane_line_out,  planes lines in 2D space
	* */
	static inline void get_edge2d_lines(const std::string output_path, const std::string line_cfg_file, Vector<PlaneSegOutput>& plane_seg_out, \
		Vector<Vector<Point2fArray>> &plane_2Dcontours_array,Vector<Line3DSegOutItemDebug> &plane_line_out)
	{
		Point3fArray planes_normal, planes_center;
		planes_normal.clear(); planes_normal.resize(plane_seg_out.size());
		planes_center.clear(); planes_center.resize(plane_seg_out.size());
		for (int i = 0; i < plane_seg_out.size(); i++)
		{
			planes_normal[i] = plane_seg_out[i].plane_normal;
			planes_center[i] = plane_seg_out[i].plane_center;
		}

		util_line_test::Features_GetLinePoints3D(output_path, line_cfg_file, planes_normal, planes_center,plane_2Dcontours_array, plane_line_out);
		//std::sort(plane_line_out.begin(), plane_line_out.end(), line_large_cp);
	}

	static inline void get_edge2d_lines(const std::string output_path, const std::string line_cfg_file, const Vector<Point3f>& planes_normal, const Vector<Point3f>& planes_center, \
		Vector<Vector<Point2fArray>>& plane_2Dcontours_array, Vector<Line3DSegOutItemDebug>& plane_line_out)
	{
		util_line_test::Features_GetLinePoints3D(output_path, line_cfg_file, planes_normal, planes_center, plane_2Dcontours_array, plane_line_out);
		//std::sort(plane_line_out.begin(), plane_line_out.end(), line_large_cp);
	}

	static inline void get_edge2d_lines_single(const std::string output_path, const std::string line_cfg_file, const Point3f& planes_normal, const Point3f& planes_center, \
		Vector<Point2fArray>& plane_2Dcontours_array, Vector<Line3DSegOutItemDebug>& plane_line_out)
	{
		util_line_test::Features_GetLinePoints3D_Single(output_path, line_cfg_file, planes_normal, planes_center, plane_2Dcontours_array, plane_line_out);
		//std::sort(plane_line_out.begin(), plane_line_out.end(), line_large_cp);
	}

	static inline void get_edge2d_lines_single(///T
		const std::string line_cfg_file, 
		const Point3f& planes_normal, 
		const Point3f& planes_center,
		Vector<Point2fArray>& plane_2Dcontours_array, 
		Vector<Line3DSegOutItemDebug>& plane_line_out)///o
	{
		util_line_test::Features_GetLinePoints3D_Single(line_cfg_file, 
			                                            planes_normal, 
														planes_center,
			                                            plane_2Dcontours_array, 
			                                            plane_line_out);
		//std::sort(plane_line_out.begin(), plane_line_out.end(), line_large_cp);
	}

	/**
	* \brief  transfer input data to a new position and rotation an angle (Right-hand rule)
	* @param [in] trans_diff, Coordinate difference of output with input data
	* @param [in] axis_normal, Coordinate axis normal
	* @param [in] angle, rotation angle form axis
	* @param [in] input_data, input point cloud
	* @param [out] output_data, output point cloud
	* */
	static inline void RotTrans(const Point3f trans_diff, const Point3f axis_normal, const float angle, const Point3fArray&input_data, Point3fArray& output_data)
	{

		size_t nPts = input_data.size();
		output_data.clear(); output_data.resize(nPts);
#pragma omp parallel for
		for (int i = 0; i < nPts; i++)
		{
			output_data[i] = input_data[i] - trans_diff;
		}
		CMat rot_mat(3, 3, CV_32F);
		
		Util_Math::get_rot_mat3<CMat,Point3f, float>(axis_normal, angle, rot_mat);

		float dt = 0;
#pragma omp parallel for
		for (int i = 0; i < nPts; i++)
		{
			//output_data[i] = Point3f(
			//	rot_mat.at<float>(0, 0) * output_data[i].x + rot_mat.at<float>(0, 1) * output_data[i].y + rot_mat.at<float>(0, 2) * output_data[i].z,
			//	rot_mat.at<float>(1, 0) * output_data[i].x + rot_mat.at<float>(1, 1) * output_data[i].y + rot_mat.at<float>(1, 2) * output_data[i].z,
			//	rot_mat.at<float>(2, 0) * output_data[i].x + rot_mat.at<float>(2, 1) * output_data[i].y + rot_mat.at<float>(2, 2) * output_data[i].z);
			Util_Math::rot<Point3f, float, CMat>(output_data[i], dt, rot_mat);
		}	
	}

	/**
	* \brief  transfer input data to a new position and rotation an angle
	* @param [in] trans_diff, Coordinate difference of output with input data
	* @param [in] axis_normal, Coordinate axis normal
	* @param [in] angle, rotation angle form axis
	* @param [in] input_data, input point cloud
	* @param [out] output_data, output point cloud
	* */
	static inline void RotTrans_line(const Point3f trans_diff, const Point3f axis_normal, const float angle, const Vector<Line3DSegOutItemDebug> input_lines, Vector<Line3DSegOutItemDebug>&output_lines)
	{

		size_t nLs = input_lines.size();
		output_lines.clear(); output_lines.resize(nLs);
#pragma omp parallel for
		for (int i = 0; i < nLs; i++)
		{
			output_lines[i].line_center = input_lines[i].line_center - trans_diff;
			output_lines[i].line_direction = input_lines[i].line_direction;
			output_lines[i].line_seg_start = input_lines[i].line_seg_start - trans_diff;
			output_lines[i].line_seg_end = input_lines[i].line_seg_end - trans_diff;
			size_t nPts = input_lines[i].points.size();
			output_lines[i].points.clear();
			output_lines[i].points.resize(nPts);
			for (int j = 0; j < nPts; j++)
			{
				output_lines[i].points[j] = input_lines[i].points[j] - trans_diff;
			}
		}
		//CMat rot_mat(3, 3, CV_32F);

		CMat rot_mat = Util_CV::zeros<CMat>(3, 3, CV_32F);

		Util_Math::get_rot_mat3<CMat, Point3f, float>(axis_normal, angle, rot_mat);

		float dt = 0.f;
#pragma omp parallel for
		for (int i = 0; i < nLs; i++)
		{

			Util_Math::rot<Point3f, float, CMat>(output_lines[i].line_center, dt, rot_mat);
			Util_Math::rot<Point3f, float,  CMat>(output_lines[i].line_seg_start,dt, rot_mat);
			Util_Math::rot<Point3f, float, CMat>(output_lines[i].line_seg_end, dt,rot_mat);
			size_t nPts = output_lines[i].points.size();
			for (int j = 0; j < nPts; j++)
			{
				Util_Math::rot<Point3f, float, CMat>(output_lines[i].points[j],dt,rot_mat);
			}
		}
	}

	/**
	* \brief save input data after rotation
	* @param [in] file, save file name
	* @param [in] trans_diff, Coordinate difference of output with input data
	* @param [in] axis_normal, Coordinate axis normal
	* @param [in] angle, rotation angle form axis
	* @param [in] input_data, input point cloud
	* */
	static inline void save_input_rot(std::string file, const Point3f trans_diff, const Point3f axis_normal, const float angle, const Point3fArray &input_data)
	{
		Point3fArray output_data;
		RotTrans(trans_diff, axis_normal, angle, input_data, output_data);
		IOData::SavePoint3fDataWithDelimiter(file," ", output_data);
	}

	/**
	* \brief give a rotation example for input data and lines extracted
	* @param [in] output_path, out put path
	* @param [in] voxel_size, reference value for example difference distance
	* @param [in] input_data,  input data to be rotated
	* @param [in] plane_line_out,  lines from planes to be rotated 	
	* */
	static inline void rot_test_example(int example, std::string output_path, float voxel_size, Vector<Point3f>&input_data, Vector<Line3DSegOutItemDebug>&plane_line_out)
	{
		// save rotation 
		std::string rot_input_file_name = output_path + "rot_input.xyz";
		Point3f trans_diff;		
		float angle;
		if (example == 0)
		{
			trans_diff = { 600,600,600 };
			angle = static_cast<float>(10 * M_PI / 180);
		}
		else if(example == 1)
		{
			trans_diff = { 6000,6000,2000 };
			angle = static_cast<float>(45 * M_PI / 180);
		}
		else
		{
			trans_diff = { 6000,4000,0 };
			angle = static_cast<float>(45 * M_PI / 180);
		}

		Point3f nzPlane = { 0,0,1 };
		save_input_rot(rot_input_file_name, trans_diff, nzPlane, angle, input_data);
		Vector<Line3DSegOutItemDebug> rot_plane_line_out;
		RotTrans_line(trans_diff, nzPlane, angle, plane_line_out, rot_plane_line_out);
#if 0
		std::string folder = output_path + "rot_line_xyz3d\\";
		util_line_test::save_line3d_xyz(folder, "line_xyz3d", ";", ".txt", rot_plane_line_out);
		folder = output_path;
		util_line_test::save_line3d_direction_end(0, folder, "rot_line_start_end3d", rot_plane_line_out);	
#endif
	}

	/**
	* \brief give a rotation example for input data and save output (z oxis Right-hand rule)
	* @param [in] rotback_file_name, rotback data save file name
	* @param [in] angle,  rotback angle
	* @param [in] input_data,  input data to be rotated
	* */
	static inline void z_rotback_example(std::string rotback_file_name, float angle, Vector<Point3f>&input_data)
	{
		// save rotation 
		Point3f trans_diff = { 0.f,0.f,0.f };
		float angle_curve = static_cast<float>(-angle * M_PI / 180);
		Point3f nzPlane = { 0,0,1 };
		save_input_rot(rotback_file_name, trans_diff, nzPlane, angle_curve, input_data);
	}

	static inline void z_rotback_input(float angle, const std::vector<ModuleStruct::Point3f>& input_data, std::vector<ModuleStruct::Point3f>& output_data)
	{
		ModuleStruct::Point3f trans_diff = { 0.f,0.f,0.f };
		float angle_curve = static_cast<float>(-angle * M_PI / 180);
		ModuleStruct::Point3f nzPlane = { 0,0,1 };
		RotTrans(trans_diff, nzPlane, angle_curve, input_data, output_data);
	}

	/**
	* \brief  get lines of all the planes' lines in 2D space
	* @param [in] output_path, out put path
	* @param [in] line_cfg_file,  config file for line extration
	* @param [in] plane_2Dcontours_array, plane edge detction resulat in 2D space
	* @param [in] [out] plane_line_out,  planes lines in 2D space
	* */
	static void get_edge2d_lines_with_noisy(const std::string output_path, double pxl_size, const std::string line_cfg_file, const float sub_line_dist, \
		const std::vector<Point3f>& input_points, std::vector<PlaneSegOutput>& plane_seg_out,\
		const std::vector<unsigned char>& intensityVec,std::vector<std::vector<unsigned char>>& is_noisy, std::vector<util_UNRE::PlaneSegOutputWithInten>&out_planes)
	{
		Vector<Vector<Point2fArray>> plane_2Dcontours_array;
		util_line_in_plane_test::get_planes_edge2d(output_path, pxl_size, plane_seg_out, plane_2Dcontours_array);
		Point3fArray planes_normal, planes_center;
		planes_normal.clear(); planes_normal.resize(plane_seg_out.size());
		planes_center.clear(); planes_center.resize(plane_seg_out.size());
		for (int i = 0; i < plane_seg_out.size(); i++)
		{
			planes_normal[i] = plane_seg_out[i].plane_normal;
			planes_center[i] = plane_seg_out[i].plane_center;
		}
		std::vector<std::vector<Line3DSegOutItemDebug>> tmp_planes_lines;
		util_line_test::Features_GetPlaneLines3D(output_path, line_cfg_file, planes_normal, planes_center, plane_2Dcontours_array, tmp_planes_lines);

		std::vector<std::vector<Line3DSegOutItemDebug>> planes_lines;
		planes_lines.resize(tmp_planes_lines.size());
		for (int i = 0; i < tmp_planes_lines.size(); i++)
		{
			//std::sort(tmp_planes_lines.begin(), tmp_planes_lines.end(), line_large_cp);
			for (int j = 0; j < tmp_planes_lines[i].size(); j++)
			{
				if (tmp_planes_lines[i][j].points.size() < 20) continue;
				planes_lines[i].push_back(tmp_planes_lines[i][j]);
			}
		}

#if 0
		std::string folder = output_path + "plane_line_xyz3d\\";
		for (int i = 0; i < plane_seg_out.size(); i++)
		{
			util_line_test::save_plane_line3d_xyz(folder, "line_xyz3d", ";", ".txt", i, planes_lines[i]);
		}
#endif
		
		util_UNRE::get_plane_noisy(output_path, input_points, planes_lines, sub_line_dist,plane_seg_out,intensityVec, is_noisy, out_planes);
	}

	/**
	* \brief  get lines of all the planes' lines in 2D space
	* @param [in] output_path, out put path
	* @param [in] line_cfg_file,  config file for line extration
	* @param [in] plane_2Dcontours_array, plane edge detction resulat in 2D space
	* @param [in] [out] plane_line_out,  planes lines in 2D space
	* */
	static void get_edge2d_lines_with_noisy(
		const std::string output_path, double pxl_size, 
		const std::string line_cfg_file, const float sub_line_dist, 
		const std::vector<Point3f>& input_points,
		const ModuleStruct::Point3f& plane_center, 
		const ModuleStruct::Point3f& plane_normal, 
		const std::vector<unsigned char>& intensityVec,
		std::vector<unsigned char>& is_noisy, 
		util_UNRE::PlaneSegOutputWithInten& out_planes, 
		const std::vector<std::vector<cv::Point3f>>& holes_location)
	{
		clock_t t1 = clock();
		
#if 0	
		std::vector<Line3DSegOutItemDebug> planes_lines;
		std::cout << "holes_location.size(): " << holes_location.size() << std::endl;
		for (int i = 0; i < holes_location.size(); i++)
		{
			Line3DSegOutItemDebug line0;
			line0.line_seg_start = holes_location[i][0];
			line0.line_seg_end = holes_location[i][1];
			line0.points.push_back(line0.line_seg_start);
			line0.points.push_back(line0.line_seg_end);
			planes_lines.push_back(line0);
			Line3DSegOutItemDebug line1;
			line1.line_seg_start = holes_location[i][1];
			line1.line_seg_end = holes_location[i][2];
			line1.points.push_back(line1.line_seg_start);
			line1.points.push_back(line1.line_seg_end);
			planes_lines.push_back(line1);
			Line3DSegOutItemDebug line2;
			line2.line_seg_start = holes_location[i][2];
			line2.line_seg_end = holes_location[i][3];
			line2.points.push_back(line2.line_seg_start);
			line2.points.push_back(line2.line_seg_end);
			planes_lines.push_back(line2);
			Line3DSegOutItemDebug line3;
			line3.line_seg_start = holes_location[i][3];
			line3.line_seg_end = holes_location[i][0];
			line3.points.push_back(line3.line_seg_start);
			line3.points.push_back(line3.line_seg_end);
			planes_lines.push_back(line3);
		}
#endif
		std::vector<std::vector<cv::Point2f>> plane_2Dcontours_array;
		util_line_in_plane_test::get_plane_edge2d(16, 
			                                      input_points, 
			                                      plane_normal, 
			                                      plane_center, 
			                                      plane_2Dcontours_array///o cv::findContours()
												 );
		
		std::vector<Line3DSegOutItemDebug> tmp_planes_lines;
		util_line_test::Features_Single_GetPlaneLines3D(output_path, 
														line_cfg_file, 
			                                            plane_normal, 
			                                            plane_center,
			                                            plane_2Dcontours_array, ///i
													    tmp_planes_lines///o
														);
		
		std::vector<Line3DSegOutItemDebug> planes_lines;
		//std::sort(tmp_planes_lines.begin(), tmp_planes_lines.end(), line_large_cp);
		for (int j = 0; j < tmp_planes_lines.size(); j++)
		{
			if (tmp_planes_lines[j].points.size() < 20) 
				continue;
			planes_lines.push_back(tmp_planes_lines[j]);
		}
		//util_line_test::save_line3d_xyz("C:\\Users\\david\\Documents\\UNRE_line_tool
		//\\UNRE_plane_output\\401主卧\\rn_new\\", "line_rn_xyz3d", ";", ".txt", planes_lines);
		
		
		clock_t t2 = clock();
		//std::string folder = output_path + "plane_line_xyz3d\\";
		//util_line_test::save_plane_line3d_xyz(folder, "line_xyz3d", ";", ".txt", 0, planes_lines);
		util_UNRE::get_single_plane_noisy(output_path, 
										  input_points, plane_center, plane_normal, 
			                              planes_lines, ///i
										  sub_line_dist, 
										  intensityVec, 
										  is_noisy, ///o
										  out_planes);
		//std::cout << "get_single_plane_noisy time cost: " << (clock() - t2) / float(CLOCKS_PER_SEC) << std::endl;
		//std::cout << "get_edge2d_lines_with_noisy edge dect time cost: " << (clock() - t1) / float(CLOCKS_PER_SEC) << std::endl;
	}

	// save line index ,plane index,and plane mse for line dist tool to calibration the size door and window
	static bool save_lines_plane_mse(std::string output_path, Vector<Line3DSegOutItemDebug>& plane_line3d_seg, Vector<PlaneSegOutput>& plane_out)
	{
		if (plane_line3d_seg.empty() || plane_out.empty())
		{
			//log_error("save_lines_plane_mse input empty");
			return false;
		}

		std::string whole_path = output_path + "lines_plane_mse.txt";
		ofstream fout(whole_path);
		if (fout.fail()) {
			//log_error("save_lines_plane_mse cannot open %s", whole_path.c_str());
			return false;
		}

		std::string delimiter = " ";
		size_t plane_num = plane_out.size();
		for (int i = 0; i < plane_line3d_seg.size(); i++)
		{
			unsigned int plane_idx = plane_line3d_seg[i].line_plane_idx;
			if (plane_idx >= plane_num)
			{
				//log_error("save_lines_plane_mse input erro plane_idx =%d plane_num=%d ",plane_idx, plane_num);
				return false;
			}

			float plane_mse = plane_out[plane_idx].plane_mse;
			fout << i << delimiter << plane_idx << delimiter << plane_mse;
			if (i < plane_line3d_seg.size() - 1) {
				fout << std::endl;
			}
		}
		fout.close();
		return false;
	}

	static inline bool covert_edge2d_to3D(Vector<PlaneSegOutput>& planes, Vector<Vector<Point2fArray>>& plane_2Dcontours_array, Vector<Vector<Point3fArray>>& plane_3Dcontours_array)
	{
		if (planes.empty() || (plane_2Dcontours_array.empty()))
		{
			//log_error("covert_edge2d_to3D input error");
			return false;
		}
		size_t plane_size = planes.size();
		if (plane_size != plane_2Dcontours_array.size())
		{
			//log_error("covert_edge2d_to3D input error plane_size =%d plane_2Dcontours_array size =%d", plane_size, plane_2Dcontours_array.size());
			return false;
		}

		plane_3Dcontours_array.resize(plane_size);
		//#pragma omp parallel for
		for (int i = 0; i < plane_size; i++)
		{
			Point3f nz(0.0f, 0.0f, 1.0f);
			Point3f plane_normal = planes[i].plane_normal;
			Point3f plane_center = planes[i].plane_center;
			bool isNeedRot = !Util_Math::vec3_are_same(plane_normal, nz); // is need rotation
			CMat rot_mat(3, 3, CV_32F);
			bool debug = i < 2;

			if (plane_2Dcontours_array[i].empty())
			{
				//log_info("plane %d 2D contours empty", i);
				continue;
			}
			if (isNeedRot)
			{
				rot_mat = Util_Math::CreateRotationMat4E(nz, plane_normal);
			}
			plane_3Dcontours_array[i].resize(plane_2Dcontours_array[i].size());
			for (int j = 0; j < plane_2Dcontours_array[i].size(); j++)
			{
				if (plane_2Dcontours_array[i][j].empty())
				{
					//log_info("plane%d edge%d 2D contours empty", i, j);
					continue;
				}

				plane_3Dcontours_array[i][j].resize(plane_2Dcontours_array[i][j].size());
				for (int k = 0; k < plane_3Dcontours_array[i][j].size(); k++)
				{
					//debug = debug && (k < 20);
					Point3f point3D;// = plane_3Dcontours_array[i][j][k];
					if (isNeedRot)
					{
						point3D.x = plane_2Dcontours_array[i][j][k].x;
						point3D.y = plane_2Dcontours_array[i][j][k].y;
						point3D.z = 0.f;
						point3D = MathOperation::point_rot(rot_mat, point3D);
						plane_3Dcontours_array[i][j][k] = point3D + plane_center;
					}
					else
					{
						point3D.x = plane_2Dcontours_array[i][j][k].x;
						point3D.y = plane_2Dcontours_array[i][j][k].y;
						point3D.z = 0.f;
						plane_3Dcontours_array[i][j][k] = point3D + plane_center;
					}
				}
			}
		}
		return true;
	}


}


#endif /*_UTIL_LINE_IN_PLANE_TEST_HPP_*/
