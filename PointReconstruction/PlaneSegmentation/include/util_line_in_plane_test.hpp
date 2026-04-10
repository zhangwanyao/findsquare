#ifndef _UTIL_LINE_IN_PLANE_TEST_HPP_
#define _UTIL_LINE_IN_PLANE_TEST_HPP_

#include <string>
#include <codecvt> //ut8
#include <iostream>
#include "config.h"
#include "log.h"
#include "InOutData.h"
#include "ModuleStruct.hpp"
#include "plane_seg_inf.h"
#include "detc_edge_api.h"
#include "line_extract_api.h"
#include "util_math.hpp"
#include "util_line_test.hpp"

namespace util_line_in_plane_test
{
	/**
	* \brief get plane segmentation input default file path (no input parameters)
	* @param [out] output_path,  default out put path
	* @param [out] input_file_name,default  input file path+name
	* @param [out] config_file_name, default configure file name
	* */
	static inline void tool_defaut_input_process(std::string &output_path, \
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
		log_warn("edge2d_pxl_size =%f", pxl_size);

		for (int i = 0; i < plane_seg_out.size(); i++)
		{
			PlaneSegOutput* plane_it = &plane_seg_out[i];
			void* edge_handle = Features_EdgeDetcCreateHandle();
			Features_getOrigMapOfContours2D(output_path, edge_handle, plane_it->plane_normal, plane_it->plane_center, &plane_it->points, pxl_size, plane_2Dcontours_array[i]);
			Features_EdgeDetcDestroyHandle(&edge_handle);
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
			void* edge_handle = Features_EdgeDetcCreateHandle();
			Features_getOrigMapOfContours3D(output_path, edge_handle, plane_it->plane_normal, plane_it->plane_center, &plane_it->points, pxl_size, plane_3Dcontours_array[i]);
			Features_EdgeDetcDestroyHandle(&edge_handle);
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

		Util_Math::get_rot_mat3<CMat, Point3f, float>(axis_normal, angle, rot_mat);

		float mat_rot_v[3][3];
		for (size_t i = 0; i < 3; i++)
			for (size_t j = 0; j < 3; j++)
				mat_rot_v[i][j] = rot_mat.ptr<float>(i)[j];
		float dt = 0;
#pragma omp parallel for
		for (int i = 0; i < nPts; i++)
		{
			//output_data[i] = Point3f(
			//	rot_mat.at<float>(0, 0) * output_data[i].x + rot_mat.at<float>(0, 1) * output_data[i].y + rot_mat.at<float>(0, 2) * output_data[i].z,
			//	rot_mat.at<float>(1, 0) * output_data[i].x + rot_mat.at<float>(1, 1) * output_data[i].y + rot_mat.at<float>(1, 2) * output_data[i].z,
			//	rot_mat.at<float>(2, 0) * output_data[i].x + rot_mat.at<float>(2, 1) * output_data[i].y + rot_mat.at<float>(2, 2) * output_data[i].z);

			//Util_Math::rot<Point3f, float, CMat>(output_data[i], dt, rot_mat);
			Util_Math::rot<Point3f, float>(output_data[i], rot_mat_v);
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

		float mat_rot_v[3][3];
		for (size_t i = 0; i < 3; i++)
			for (size_t j = 0; j < 3; j++)
				mat_rot_v[i][j] = rot_mat.ptr<float>(i)[j];
		float dt = 0.f;
#pragma omp parallel for
		for (int i = 0; i < nLs; i++)
		{
			/*Util_Math::rot<Point3f, float, CMat>(output_lines[i].line_center, dt, rot_mat);
			Util_Math::rot<Point3f, float,  CMat>(output_lines[i].line_seg_start,dt, rot_mat);
			Util_Math::rot<Point3f, float, CMat>(output_lines[i].line_seg_end, dt,rot_mat);*/
			Util_Math::rot<Point3f, float>(output_lines[i].line_center, mat_rot_v);
			Util_Math::rot<Point3f, float>(output_lines[i].line_seg_start, mat_rot_v);
			Util_Math::rot<Point3f, float>(output_lines[i].line_seg_end, mat_rot_v);
			size_t nPts = output_lines[i].points.size();
			for (int j = 0; j < nPts; j++)
			{
				//Util_Math::rot<Point3f, float, CMat>(output_lines[i].points[j],dt,rot_mat);
				Util_Math::rot<Point3f, float>(output_lines[i].points[j], mat_rot_v);
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
		IOData::SavePoint3fDataWithDelimiter(file, " ", output_data);
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
		else if (example == 1)
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
		std::string folder = output_path + "rot_line_xyz3d\\";
		util_line_test::save_line3d_xyz(folder, "line_xyz3d", ";", ".txt", rot_plane_line_out);
		folder = output_path;
		util_line_test::save_line3d_direction_end(0, folder, "rot_line_start_end3d", rot_plane_line_out);
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
}

#endif /*_UTIL_LINE_IN_PLANE_TEST_HPP_*/