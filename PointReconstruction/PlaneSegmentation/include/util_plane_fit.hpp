#ifndef _UTIL_PLANE_FIT_HPP_
#define _UTIL_PLANE_FIT_HPP_

#include <string>
#include <codecvt> //ut8
#include <opencv2/core.hpp>
#include <iostream>
#include "config.h"
#include "log.h"
#include "InOutData.h"
//#include "Open3dIO.h"
#include "plane_seg_inf.h"
#include "Util_UNRE.hpp"
#include "plane_cfg.hpp"
#include "util_time.hpp"
#include "util_math.hpp"
#include "MathOperation.hpp"
#include "NormalStruct.hpp"
namespace util_plane_seg
{
	//check input unit is m or mm
#define CHECK_UNIT_POINT_SIZE	10000
#define CHECK_UNIT_POINT_THREADHOLD	0.8
#define CHECK_UNIT_POINT_LEN	50.f

#define		DETECT_RADIUS_VOXEL_STEP	1
#define		DETECT_RADIUS_NEAR_POINTS	5
//#define		OCCUPIED_VOXEL_POINTS 106

	/**
	* \brief input help
	* */
	static inline void printHelp()
	{
		std::cout << "Syntax is: target.exe -i input file -o output path -c config_file" << std::endl;
		std::cout << " for example use default config file: planeSegmentation.exe -i c:/mypath/inpu_data.pts -o c:/output/" << std::endl;
		std::cout << " for example use specifical config file: planeSegmentation.exe -i c:/mypath/inpu_data.pts -o c:/output/ -c c:/mypath/config_room.ini" << std::endl;
	}

	/**
	* \brief get plane segmentation input default file path (no input parameters)
	* @param [out] output_path,  default out put path
	* @param [out] input_file_name,default  input file path+name
	* @param [out] config_file_name, default configure file name
	* */
	static inline void plane_defaut_input_process(std::string &output_path,\
		std::string &input_file_name, std::string &config_file_name)
	{
		output_path = DATA_OUTPUT_PATH;
		std::string file_name, folder, xml_path;
		file_name = "load_path.xml";
		folder = DATA_INPUT_PATH;
		xml_path = folder + file_name;
		cv::FileStorage load_path_xml(xml_path, cv::FileStorage::READ);
		input_file_name = folder + (std::string)load_path_xml["scene_data"];
		config_file_name = folder + (std::string)load_path_xml["configure"];
	}

	/**
	* \brief get plane segmentation input file path by input parameters
	* @param [in] argc, number of input parameters
	* @param [in] argv, pointer of input data address
	* @param [out] output_path,  default out put path, with endof "/" or"\\"
	* @param [out] input_file_name,default  input file path+name
	* @param [out] config_file_name, configure file path+name  
	* return 0 is ok, <0 is error
	* */
	static inline int plane_arc_parse(const int argc, char** argv, std::string& output_path, \
		std::string& input_file_name, std::string& config_file_name)
	{
		if (IOData::parse_argument(argc, argv, "-i", input_file_name) <= 0)
		{
			printf("No Input File given\n");
			printHelp();
			return -1;
		}

		if (IOData::parse_argument(argc, argv, "-o", output_path) <= 0)
		{
			printf("No Output Path given\n");
			printHelp();
			return -1;
		}

		if (IOData::parse_argument(argc, argv, "-c", config_file_name) <= 0)
		{
#ifdef PLANE_SEG_FOR_PLADE
			config_file_name = "config_plade.ini";
#else
			config_file_name = "config_room.ini";
#endif
		}
		return 0;
	}

	/**
	* \brief  input file name process for ut_8  input compatible
	* @param [in] [out] output_path,  default out put path, with endof "/" or"\\"
	* @param [in] [out] input_file_name,default  input file path+name
	* @param [in] [out] config_file_name, configure file path+name
	* return 0 is ok, <0 is error
	* */
	static inline int file_cv_process(std::string& output_path, std::string& input_data_file, std::string& config_file_name)
	{
		std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
		std::wstring w_input_file_name = converter.from_bytes(input_data_file);
		std::wstring w_output_path = converter.from_bytes(output_path);
		output_path = converter.to_bytes(w_output_path);
		input_data_file = converter.to_bytes(w_input_file_name);

		std::cout << "scene_data_path: " << input_data_file << std::endl;
		std::wcout << "w_input_file_name: " << w_input_file_name << std::endl;

		if ((IOData::createDirectory(output_path)) != 0)
		{
			std::cout << "createDirectory: " << output_path << " failed" << std::endl;
			return -1;
		}
		std::cout << "createDirectory:" << output_path << " success" << std::endl;
		return 0;
	}

	/**
	* \brief  get plane segmentation configure
	* @param [in] config_file_name, configure file path+name
	* @param [in] output_path,  default out put path, with endof "/" or"\\"
	* @param [in] [out] config_it,  plane configures
	* return 0 is ok, <0 is error
	* */
	static inline int load_input_cfg(const std::string config_file_name, const std::string output_path, planeSegCfgParse &config_it)
	{	    
		bool rtn = config_it.GetPlaneTestConfigure(config_file_name);
		if (!rtn)
		{
			std::cout << "load config file: " << config_file_name << " failed" << std::endl;
			return -1;
		}
		std::cout << "load config file: " << config_file_name << " success" << std::endl;

#ifdef SAVE_OUTPUT_FILE_DEBUG
		std::string output_config_file = output_path + "PlaneCfginfo.txt";
		config_it.SaveConfigInfo(output_config_file);
		output_config_file = output_path + "PlaneCfgParseInfo.txt";
		//config_it.SaveConfigParams(output_config_file, config_it.getSysConfigParams());
		//config_it.SaveConfigParams(output_config_file, config_it.getVoxelSize());
		DebugConfigParams debug_config_para;
		config_it.SaveCommonConfigParams(output_config_file);
		config_it.SyncPlaneSegConfigure(output_config_file, debug_config_para);
		//config_it.SavePlaneTestConfigParams(output_config_file);
		config_it.SavePlaneSegConfigParams(output_config_file);
#endif
		return 0;
	}

	/**
	* \brief  load input data by file type , include 3 types: UNRE encryption, txt(same format with .xyz)  and open3d filetype
	* @param [in] encryption_type,   0 plain; 1 UNRE_ENCRYPTION 
	* @param [in] input_data_file,  file path+name of input data
	* @param [out] input_data,input data loaded from file
	* @param [out] intensity_vec,input data intensity loaded from file
	* return 0 is ok, <0 is error
	* */
	static inline int load_input_data(const int encryption_type, const std::string input_data_file, ModuleStruct::Point3fArray& input_data, \
		std::vector<unsigned char>& intensity_vec)
	{
		if (encryption_type == 1)
		{
			/*if (!util_UNRE::ReadEncryptionDataWithIntensity(input_data_file, input_data, intensity_vec))
			{
				std::cout << "can not open file: " << input_data_file << std::endl;
				return -1;
			}*/

			std::vector<float>* intensity = new std::vector<float>;

			input_data = IOData::readFromXyz(input_data_file, true, intensity);

			for (int i = 0; i < input_data.size(); i++)
			{
				//scene_xyz.emplace_back(pair_cloud_reflect[i].first);
				intensity_vec.emplace_back(static_cast<unsigned char>((*intensity)[i]));
			}
		}
		log_debug("input data size =%d", input_data.size());
		if (input_data.size() == 0)
		{
			return -1;
		}
		return 0;
	}
	
	/**
	* \brief  change the center of input data as the coordinate origin
	* @param [in] encryption_type,   0 plain; 1 UNRE_ENCRYPTION
	* @param [in] orig_in_center,  true: change, false: no change
	* @param [in] [out] input_data,input data loaded from file
	* */
	static inline void center2origin(bool orig_in_center, ModuleStruct::Point3fArray& input_data)
	{
		if (orig_in_center)
		{
			Point3f pt_center = { 0.f,0.f,0.f };
			double sumx, sumy, sumz;
			sumx = sumy = sumz = 0;

#pragma omp parallel for reduction(+:sumx,sumy,sumz)
			for (int i = 0; i < static_cast<int>(input_data.size()); i++) {
				sumx += input_data[i].x;
				sumy += input_data[i].y;
				sumz += input_data[i].z;
			}
			double invers_pt_size = 1.0 / input_data.size();
			pt_center.x = static_cast<float>(sumx * invers_pt_size);
			pt_center.y = static_cast<float>(sumy * invers_pt_size);
			pt_center.z = static_cast<float>(sumz * invers_pt_size);
			log_debug("pt_center = [%lf,%lf,%lf]", pt_center.x, pt_center.y, pt_center.z);

#pragma omp parallel for
			for (int i = 0; i < static_cast<int>(input_data.size()); i++) {
				input_data[i] = input_data[i] - pt_center;
			}
		}	
	}

	/**
	* \brief  check unit is m or mm
	* @param [in] encryption_type,   0 plain; 1 UNRE_ENCRYPTION
	* @param [in] orig_in_center,  true: change, false: no change
	* @param [in] [out] input_data,input data loaded from file
	* return true unit is m, else is mm
	* */
	static inline bool CheckInputDataUnit(const ModuleStruct::Point3fArray&data)
	{
		int64_t size = data.size() > CHECK_UNIT_POINT_SIZE ? CHECK_UNIT_POINT_SIZE : data.size();
		int out = 0;
		for (int i = 0; i < size; i++)
		{
			if (std::abs(data[i].x) > CHECK_UNIT_POINT_LEN || std::abs(data[i].y) > CHECK_UNIT_POINT_LEN || std::abs(data[i].z) > CHECK_UNIT_POINT_LEN)
			{
				out++;
			}
		}
		return out > (CHECK_UNIT_POINT_THREADHOLD * size) ? true : false;
	}


	/**
	* \brief  change unit mm to m 
	* @param [in] [out] input_data, input with m unit and out data with unit mm
	* return true unit is m, else is mm
	* */
	static inline void unit_m2mm(ModuleStruct::Point3fArray & input_data)
	{
		log_debug("unit is m");
#pragma omp parallel for
		for (int i = 0; i < input_data.size(); i++) {
			input_data[i].x *= 1000.f;
			input_data[i].y *= 1000.f;
			input_data[i].z *= 1000.f;
		}

	}

	/**
	* \brief save plane segmention output to file
	* @param [in] sub_path,  out put path, with endof "/" or"\\"
	* @param [in] sub_file_name, input file name
	* @param [in] delimiter, delimiter of save data
	* @param [in] file_type,  save file suffix
	* @param [in] input_data, input point cloud
	* @param [in] plane_seg_out,  segment out put  of all planes
	* return > = 0 success, else failed
	* */
	static inline int save_plane_xyz(const std::string sub_path, const std::string sub_file_name, const std::string delimiter, \
		const std::string file_type, ModuleStruct::Point3fArray&input_data, Vector<PlaneSegOutput>&plane_seg_out)
	{
	
		for (unsigned int i = 0; i < plane_seg_out.size(); i++) {
			std::stringstream plane_id;
			plane_id << i;
			if (IOData::createDirectory(sub_path)<0)
			{
				log_error(" save_plane_xyz createDirectory %s failed", sub_path.c_str());
				return -1;
			}
			std::string whole_path = sub_path + sub_file_name + plane_id.str() + file_type;
			bool rtn = IOData::SavePoint3fDataWithDelimiter(whole_path, delimiter, input_data, plane_seg_out[i].point_ids);
			if (!rtn)
			{
				log_error("whole_path %s save_plane_xyz plane %d return error", whole_path.c_str(),i);
				return -1;
			}
		}
		return 0;
	}
		
	/**
	* \brief save plane segmention output to file
	* @param [in] is_missing,  true save missing points  or else false save  merging points to file
	* @param [in] sub_path,  out put path, with endof "/" or"\\"
	* @param [in] file_name, data  file  to save
	* @param [in] delimiter, delimiter of save data
	* @param [in] input_data_ptr, address of save daa
	* @param [in] is_merged, point merging status array of input data
	* return = 0 success, else failed
	* */
	static inline int save_missing_points(bool is_missing, const std::string sub_path, const std::string file_name, const std::string delimiter,\
		ModuleStruct::Point3fArray * input_data_ptr, Vector<unsigned int> is_merged)
	{

		if (IOData::createDirectory(sub_path))
		{
			log_error("createDirectory %s failed", sub_path.c_str());
			return -1;
		}

		std::ofstream out_points(sub_path + file_name);
		out_points << std::setprecision(6) << std::fixed;
		if (is_missing)
		{
			// file_type == 0 missing  point outpt
			int missing_cnt = 0;
			for (size_t i = 0; i < input_data_ptr->size(); i++) {

				if (is_merged[i]==0) {

					out_points << (*input_data_ptr)[i].x << delimiter;
					out_points << (*input_data_ptr)[i].y << delimiter;
					out_points << (*input_data_ptr)[i].z << '\n';
					missing_cnt++;
				}
			}
			log_info("missing_cnt =%d", missing_cnt);
		}
		else
		{
			int merging_cnt = 0;
			for (size_t i = 0; i < input_data_ptr->size(); i++) {

				if (is_merged[i] == 1) {

					out_points << (*input_data_ptr)[i].x << delimiter;
					out_points << (*input_data_ptr)[i].y << delimiter;
					out_points << (*input_data_ptr)[i].z << '\n';
					merging_cnt++;
				}
			}
			log_info("merging_cnt =%d", merging_cnt);
		}
		out_points.close();
		return 0;
	}

	/**
	* \brief save plane segmention output to file
	* @param [in] save_type,  0:normal, 1:center, else, normal and center together
	* @param [in] sub_path,  out put path, with endof "/" or"\\"
	* @param [in] file_name, data  file  to save
	* @param [in] plane_seg_out, all fit planes output
	* return = 0 success, else failed
	* */
	static inline int save_plane_normal_center(int save_type, const std::string sub_path, const std::string file_name, Vector<PlaneSegOutput> plane_seg_out)
	{
		if (IOData::createDirectory(sub_path))
		{
			log_error("createDirectory %s failed", sub_path.c_str());
			return -1;
		}
		std::string file_type = ".txt";

		for (unsigned int i = 0; i < plane_seg_out.size(); i++) {
			std::stringstream plane_id;
			plane_id << i;
			//folder = DATA_OUTPUT_PATH;
			//folder = output_path + "plane_normals\\";
			std::string whole_path = sub_path + file_name + plane_id.str() + file_type;
			if (save_type == 0)
			{
				IOData::SavePoint3fData(whole_path, plane_seg_out[i].plane_normal);
			}
			else if(save_type == 1 )
			{
				IOData::SavePoint3fData(whole_path, plane_seg_out[i].plane_center);
			}
			else
			{
				std::vector<Point3f> input_data(1);
				std::vector<Point3f> input_normal(1);
				input_data[0] = plane_seg_out[i].plane_center;
				input_normal[0] = plane_seg_out[i].plane_normal;
				FeaturesIO::SavePoint3fDataNormalWithDelimiter(whole_path," ", input_data, input_normal);
			}
		}
		return 0;
	}

	/**
	* \brief sync original point cloud to  the plane with downsample point cloud
	* @param [in] input_points,  points cloud before downsample
	* @param [in] const unsigned int centroid_array_size, points cloud size after downsampling
	* @param [in] Point2CentriodIdxArray *point_to_centriod_array, map of original point index to filtered point index by downsample
	* @param [in]  PlaneSegmentationOutput *filtered_points_plane, pointer to planes with filtered point cloud by downsampling
	* @param [out]  PlaneSegmentationOutput *orig_points_planes, pointer to planes with point cloud before downsampling, new memory resources created
	* @param [out]  std::vector<bool> is_merged, output parameters,  show if the points is in the plane, new memory resources created
	* @return if success or not
	* */
	static inline bool SyncOrigPointToPlane(const Vector<Point3f>input_points, const unsigned int centroid_array_size, const std::vector<unsigned int> point_to_centriod_array, \
		std::vector<PlaneSegOutput> filtered_points_plane, std::vector<PlaneSegOutput>& orig_points_planes, std::vector<unsigned int>& is_merged)
	{
		const unsigned int invalid_plane_idx = -1;
		unsigned int orig_point_array_size = static_cast<unsigned int>(point_to_centriod_array.size());

		std::vector<unsigned int> centroid_plane_idx(centroid_array_size, invalid_plane_idx);
		is_merged.resize(orig_point_array_size, 0);
		// get plane idx for centroid_array
#pragma omp parallel for
		for (int i = 0; i < filtered_points_plane.size(); i++)
		{
			PlaneSegOutput* plane_it = &filtered_points_plane[i];
			for (int j = 0; j < plane_it->point_ids.size(); j++)
			{
				unsigned int centroid_point_idx = plane_it->point_ids[j];
				if (centroid_point_idx >= centroid_array_size)
				{
					// it is impossible, must have bug or input error
					log_error("centroid_point_idx =%d exceed size of centroid_array_size =%d", centroid_point_idx, centroid_array_size);
					continue;
				}
				centroid_plane_idx[centroid_point_idx] = i;
			}
		}
		// get plane cnt list for orig_point_array
		std::vector<unsigned int> point_cnt_list_of_plane(filtered_points_plane.size(), 0);
		//muiltple thread ?  #pragma omp parallel for
		for (int i = 0; i < static_cast<int>(orig_point_array_size); i++)
		{
			unsigned int centroid_point_idx = point_to_centriod_array[i];
			if (centroid_point_idx >= centroid_array_size)
			{
				// it is impossible, must have bug or input error
				log_error("centroid_point_idx =%d exceed size of centroid_array_size =%d", centroid_point_idx, centroid_array_size);
				continue;
			}
			unsigned int plane_idx = centroid_plane_idx[centroid_point_idx];
			if (plane_idx == invalid_plane_idx) continue;
			point_cnt_list_of_plane[plane_idx]++;
		}

		//orig_points_planes->size = filtered_points_plane->size;
		//orig_points_planes->planes = new PlaneItem[orig_points_planes->size];
		orig_points_planes.resize(filtered_points_plane.size());

#pragma omp parallel for
		for (int i = 0; i < orig_points_planes.size(); i++)
		{
			PlaneSegOutput* plane_it = &filtered_points_plane[i];
			PlaneSegOutput* orig_plane_it = &orig_points_planes[i];
			orig_plane_it->plane_normal = plane_it->plane_normal;
			orig_plane_it->plane_center = plane_it->plane_center;
			orig_plane_it->plane_mse = plane_it->plane_mse;
			orig_plane_it->point_ids.resize(point_cnt_list_of_plane[i]);
			orig_plane_it->points.resize(point_cnt_list_of_plane[i]);
			point_cnt_list_of_plane[i] = 0;
		}

		for (int i = 0; i < static_cast<int>(orig_point_array_size); i++)
		{
			unsigned int centroid_point_idx = point_to_centriod_array[i];
			if (centroid_point_idx >= centroid_array_size)
			{
				// it is impossible, must have bug or input error
				log_error("centroid_point_idx =%d exceed size of centroid_array_size =%d", centroid_point_idx, centroid_array_size);
				continue;
			}
			unsigned int plane_idx = centroid_plane_idx[centroid_point_idx];
			if (plane_idx == invalid_plane_idx) continue;

			is_merged[i] = 1;
			PlaneSegOutput* orig_plane_it = &orig_points_planes[plane_idx];
			unsigned int plane_point_idx = point_cnt_list_of_plane[plane_idx];
			if (plane_point_idx >= orig_point_array_size)
			{
				// it is impossible, must have bug or input error
				log_error("plane_point_idx =%d exceed size of orig_point_array_size =%d", plane_point_idx, orig_point_array_size);
				continue;
			}
			orig_plane_it->point_ids[plane_point_idx] = i;
			orig_plane_it->points[plane_point_idx] = input_points[i];
			point_cnt_list_of_plane[plane_idx]++;
		}
		return true;
	}

	/**
	* \brief sync original point cloud to  the plane with downsample point cloud while planes have multiple plane points
	* @param [in] input_points,  points cloud before downsample
	* @param [in] const unsigned int centroid_array_size, points cloud size after downsampling
	* @param [in] Point2CentriodIdxArray *point_to_centriod_array, map of original point index to filtered point index by downsample
	* @param [in]  PlaneSegmentationOutput *filtered_points_plane, pointer to planes with filtered point cloud by downsampling
	* @param [out]  PlaneSegmentationOutput *orig_points_planes, pointer to planes with point cloud before downsampling, new memory resources created
	* @param [out]  std::vector<bool> is_merged, output parameters,  show if the points is in the plane, new memory resources created
	* @return if success or not
	* */
	static inline bool SyncOrigPointToPlaneWithMPPts(const Vector<Point3f>input_points, const unsigned int centroid_array_size, const std::vector<unsigned int> point_to_centriod_array, \
		std::vector<PlaneSegOutput> filtered_points_plane, std::vector<PlaneSegOutput>& orig_points_planes, std::vector<unsigned int>& is_merged)
	{
		const unsigned int invalid_plane_idx = -1;  // flag for points who are not in planes
		const unsigned int multi_plane_idx = -2;    // flag for points who are in multiple planes
		unsigned int orig_point_array_size = static_cast<unsigned int>(point_to_centriod_array.size()); // original point array size
		is_merged.resize(orig_point_array_size, 0);  
		std::vector<unsigned int> centroid_plane_idx(centroid_array_size, invalid_plane_idx); // downsample  point  plane index 
		std::vector<std::vector<unsigned int>> point_plane_idx_ds(centroid_array_size);     // downsample points point multiple planes index vector array
		std::vector<unsigned int>centroid_plane_cnt(centroid_array_size,0);
#pragma omp parallel for
		for (int i = 0; i < static_cast<int>(centroid_array_size); i++)
		{
			point_plane_idx_ds[i].clear();
		}
		//find the multiplane points in downsample plane,and record plane idx
		for (size_t i = 0; i < filtered_points_plane.size(); i++)
		{
			PlaneSegOutput* plane_it = &filtered_points_plane[i];			
			for (size_t j = 0; j < plane_it->point_ids.size(); j++)
			{
				unsigned int centroid_point_idx = plane_it->point_ids[j];
				centroid_plane_cnt[centroid_point_idx]++;
			}
		}

		for (size_t i = 0; i < filtered_points_plane.size(); i++)
		{
			PlaneSegOutput* plane_it = &filtered_points_plane[i];
			for (size_t j = 0; j < plane_it->point_ids.size(); j++)
			{
				unsigned int centroid_point_idx = plane_it->point_ids[j];
				if (centroid_plane_cnt[centroid_point_idx] > 1)
				{
					point_plane_idx_ds[centroid_point_idx].push_back(static_cast<unsigned int>(i));
					centroid_plane_idx[centroid_point_idx] = multi_plane_idx;
				}
				else
				{
					centroid_plane_idx[centroid_point_idx] = static_cast<unsigned int>(i);
				}
			}
		}

		// get plane cnt list for orig_point_array
		std::vector<unsigned int> point_cnt_list_of_plane(filtered_points_plane.size(), 0);
		//muiltple thread ?  #pragma omp parallel for
		for (int i = 0; i < static_cast<int>(orig_point_array_size); i++)
		{
			unsigned int centroid_point_idx = point_to_centriod_array[i];
			if (centroid_point_idx >= centroid_array_size)
			{
				// it is impossible, must have bug or input error
				log_error("centroid_point_idx =%d exceed size of centroid_array_size =%d", centroid_point_idx, centroid_array_size);
				continue;
			}
			unsigned int plane_idx = centroid_plane_idx[centroid_point_idx];
			if (plane_idx == invalid_plane_idx) continue;
			if (plane_idx == multi_plane_idx)
			{
				for (size_t j = 0; j < point_plane_idx_ds[centroid_point_idx].size(); j++)
				{
					point_cnt_list_of_plane[point_plane_idx_ds[centroid_point_idx][j]]++;
				}
				continue;
			}
			point_cnt_list_of_plane[plane_idx]++;
		}

		orig_points_planes.resize(filtered_points_plane.size());
#pragma omp parallel for
		for (int i = 0; i < orig_points_planes.size(); i++)
		{
			PlaneSegOutput* plane_it = &filtered_points_plane[i];
			PlaneSegOutput* orig_plane_it = &orig_points_planes[i];
			orig_plane_it->plane_normal = plane_it->plane_normal;
			orig_plane_it->plane_center = plane_it->plane_center;
			orig_plane_it->plane_mse = plane_it->plane_mse;
			orig_plane_it->point_ids.resize(point_cnt_list_of_plane[i]);
			orig_plane_it->points.resize(point_cnt_list_of_plane[i]);
			point_cnt_list_of_plane[i] = 0;
		}

		//add all the plane original points
		for (int i = 0; i < static_cast<int>(orig_point_array_size); i++)
		{
			unsigned int centroid_point_idx = point_to_centriod_array[i];
			if (centroid_point_idx >= centroid_array_size)
			{
				// it is impossible, must have bug or input error
				log_error("centroid_point_idx =%d exceed size of centroid_array_size =%d", centroid_point_idx, centroid_array_size);
				continue;
			}
			unsigned int plane_idx = centroid_plane_idx[centroid_point_idx];
			if (plane_idx == invalid_plane_idx) continue;

			is_merged[i] = 1;

			if (plane_idx != multi_plane_idx)
			{
				PlaneSegOutput* orig_plane_it = &orig_points_planes[plane_idx];
				unsigned int plane_point_idx = point_cnt_list_of_plane[plane_idx];
				if (plane_point_idx >= orig_point_array_size)
				{
					// it is impossible, must have bug or input error
					log_error("plane_point_idx =%d exceed size of orig_point_array_size =%d", plane_point_idx, orig_point_array_size);
					continue;
				}
				orig_plane_it->point_ids[plane_point_idx] = i;
				orig_plane_it->points[plane_point_idx] = input_points[i];
				point_cnt_list_of_plane[plane_idx]++;
			}
			else
			{
				for (size_t j = 0; j < point_plane_idx_ds[centroid_point_idx].size(); j++)
				{
					unsigned int plane_idx = point_plane_idx_ds[centroid_point_idx][j];
					PlaneSegOutput* orig_plane_it = &orig_points_planes[plane_idx];
					unsigned int plane_point_idx = point_cnt_list_of_plane[plane_idx];
					orig_plane_it->point_ids[plane_point_idx] = i;
					orig_plane_it->points[plane_point_idx] = input_points[i];
					point_cnt_list_of_plane[plane_idx]++;
				}
			}
		}

		return true;
	}
	static inline bool save_planes_file(const DownsampleType ds_type, const bool is_missing_out, const std::string output_path, Vector<Point3f>&input_points, \
		 std::vector<PlaneSegOutput>&plane_seg_out, std::vector<unsigned int> is_merged)
	{
		std::string sub_path = output_path + "plane_xyz\\";
		int rtn = util_plane_seg::save_plane_xyz(sub_path, "plane_xyz", ";", ".txt", input_points, plane_seg_out);
		if (rtn)
		{
			log_error("save_plane_xyz return error");
			return rtn;
		}

#ifdef SAVE_OUTPUT_FILE_DEBUG
		if ((is_missing_out) && (ds_type == DOWNSAMPLE_WITH_ORG))
		{
			std::string missing_points_folder = output_path + "plane_xyz\\";
			int rtn = util_plane_seg::save_missing_points(true, missing_points_folder, "missing_points.txt", ";", &input_points, is_merged);
			if (rtn)
			{
				log_error("save_missing_points return error");
				return false;
			}
			is_merged.clear();
			is_merged.shrink_to_fit();
		}
#endif

		std::string file_name, folder;
		folder = output_path + "plane_normals\\";
		file_name = "plane_normals";
		rtn = util_plane_seg::save_plane_normal_center(0, folder, file_name, plane_seg_out);
		if (rtn)
		{
			log_error("save_plane_normal_center normal return error");
			return false;
		}


		folder = output_path + "plane_center\\";
		file_name = "plane_center";
		rtn = util_plane_seg::save_plane_normal_center(1, folder, file_name, plane_seg_out);
		if (rtn)
		{
			log_error("save_plane_normal_center center return error");
			return false;
		}

		folder = output_path + "plane_center_normal\\";
		file_name = "plane_center_normal";
		rtn = util_plane_seg::save_plane_normal_center(2, folder, file_name, plane_seg_out);
		if (rtn)
		{
			log_error("save_plane_normal_center normal and center return error");
			return false;
		}

		return true;
	}

	/**
	* \brief find the nearest  center index of voxel
	* @param [in] input_data, input points cloud
	* @param [in] vxl_pt_ids, voxel points indexs
	* @return the nearest center index of vxl_pt_ids
	* */
	static inline unsigned int findNearCenterPoint(Vector<Point3f>& input_data, Vector<unsigned int>& vxl_pt_ids)
	{
		Point3f max_p, min_p;
		int nPts = static_cast<int>(vxl_pt_ids.size());

		//Util_Math::GetMinMaxXYZ(input_data, vxl_pt_ids,max_p,min_p,dt);
		min_p.x = min_p.y = min_p.z = std::numeric_limits<float>::infinity();
		max_p.x = max_p.y = max_p.z = -std::numeric_limits<float>::infinity();
		for (int i = 0; i < nPts; i++) {

			min_p.x = (min_p.x > input_data[vxl_pt_ids[i]].x) ? input_data[vxl_pt_ids[i]].x : min_p.x;
			max_p.x = (max_p.x < input_data[vxl_pt_ids[i]].x) ? input_data[vxl_pt_ids[i]].x : max_p.x;
			min_p.y = (min_p.y > input_data[vxl_pt_ids[i]].y) ? input_data[vxl_pt_ids[i]].y : min_p.y;
			max_p.y = (max_p.y < input_data[vxl_pt_ids[i]].y) ? input_data[vxl_pt_ids[i]].y : max_p.y;
			min_p.z = (min_p.z > input_data[vxl_pt_ids[i]].z) ? input_data[vxl_pt_ids[i]].z : min_p.z;
			max_p.z = (max_p.z < input_data[vxl_pt_ids[i]].z) ? input_data[vxl_pt_ids[i]].z : max_p.z;
		}

		Point3f center_pos = (max_p + min_p) / 2;
		interim_value_type mini_distance = std::numeric_limits<interim_value_type>::infinity();
		unsigned int near_center_index = 0;
		for (int i = 0; i < nPts; i++)
		{
			Point3f Point = input_data[vxl_pt_ids[i]];
			interim_value_type dist = Util_Math::ComputePointToPointDist<interim_value_type, Point3f>(Point, center_pos);
			if (dist < mini_distance)
			{
				mini_distance = dist;
				near_center_index = i;
			}
		}
		return near_center_index;
	}

	/**
	* \brief detect the density of input data
	* @param [in] input_data, input points cloud
	* @param [in] occupied_voxel_array, 
	* @param [out]  point_density, point density of input data
	* */
	static inline void PtDensityDetc(Vector<Point3f>&input_data, Vector<Vector<unsigned int>> &occupied_voxel_array, interim_value_type& point_density, const int step)
	{
		std::vector<interim_value_type> voxels_distance;
		const interim_value_type invalid_dist = (std::numeric_limits<interim_value_type>::max)();
		//int nVxls = static_cast<int>(occupied_voxel_array.size()/ DETECT_RADIUS_VOXEL_STEP);
		if ((step <= 0) || (step >= occupied_voxel_array.size() / 2))
		{
			log_error("input step %d error",step);
			return;
		}

		int nVxls = static_cast<int>(occupied_voxel_array.size() / step);
		voxels_distance.resize(nVxls);
		int half_avg_points = static_cast<int>(input_data.size() / (2 * nVxls));
#pragma omp parallel for
		for (int i = 0; i < nVxls; i++)
		{
			int vxl_ids = i * step;
			int nPts = static_cast<int>(occupied_voxel_array[vxl_ids].size());
			bool vxl_sel_con = (nPts > half_avg_points) && (nPts<4* half_avg_points);
			if (vxl_sel_con)
			{
				//unsigned int near_center_index = findNearCenterPoint(input_data, occupied_voxel_array[i]);
				unsigned int near_center_index = findNearCenterPoint(input_data,occupied_voxel_array[vxl_ids]);
				unsigned int center_point_idx = occupied_voxel_array[vxl_ids][near_center_index];
				std::vector<interim_value_type> voxel_item_distance;
				voxel_item_distance.resize(nPts);
				Point3f center_point = (input_data)[center_point_idx];
				voxel_item_distance[near_center_index] = invalid_dist;  // set center as max to escape paticipating caclulation
				for (int j = 0; j < nPts; j++)
				{
					if (j != near_center_index)
					{
						Point3f Point = (input_data)[occupied_voxel_array[vxl_ids][j]];
						//voxel_item_distance.push_back(Util_Math::ComputePointToPointDist<interim_value_type, Point3f>(Point, center_point));
						voxel_item_distance[j] = Util_Math::ComputePointToPointDist<interim_value_type, Point3f>(Point, center_point);
					}
				}
				std::sort(voxel_item_distance.begin(), voxel_item_distance.end());
				//get average distance for current voxel
				unsigned int average_distance_size = DETECT_RADIUS_NEAR_POINTS;
				if (voxel_item_distance.size() <= DETECT_RADIUS_NEAR_POINTS)
				{
					average_distance_size = (unsigned int)voxel_item_distance.size() - 1; // must get rid of the center point
				}

				interim_value_type items_total_distance = 0;
				for (unsigned int j = 0; j < average_distance_size; j++)
				{
					items_total_distance += voxel_item_distance[j];
				}		
				if (average_distance_size > 0)
				{
					voxels_distance[i] = items_total_distance / average_distance_size;
				}
				else
				{
					voxels_distance[i] = invalid_dist;
				}
			}
			else
			{
				voxels_distance[i] = invalid_dist;
			}
		}	
		//get average distance for all detected voxels
		interim_value_type total_detected_distance = 0;
		int cnt = 0;
		for (interim_value_type i : voxels_distance)
		{
			if (i == invalid_dist) continue;
			total_detected_distance += i;
			cnt++;
		}

		point_density = total_detected_distance / cnt;
	}


	static inline bool OccupiedVoxelInit(ModuleStruct::Point3fArray &input_data, const VoxelParams voxel_size, vector<vector<unsigned int>>&occupied_voxel_array)
	{
		vector<PointGrid2PointIdx> point_to_subgrid_list(input_data.size());

		VoxelDimension voxel_dim;   // input data dimension by voxel size
		Point3f max_p, min_p;
		MathOperation::GetMinMaxXYZ(input_data, max_p, min_p);

		MathOperation::GetTotalNumOfVoxelGrid(max_p, min_p, voxel_size, voxel_dim);

#pragma omp parallel for
		for (int i = 0; i < static_cast<int>(input_data.size()); i++)
		{
			point_to_subgrid_list[i].cloud_point_index = i;
			ModuleStruct::Point3f point = input_data[i];
			MathOperation::ConvertXYZToVoxelID(min_p, point, voxel_size, voxel_dim, point_to_subgrid_list[i].grid_idx);
		}

		std::sort(point_to_subgrid_list.begin(), point_to_subgrid_list.end(), std::less<PointGrid2PointIdx>());

		unsigned int i = 0;
		size_t occupied_grid_cnt = 0;  // total occupied grid number of the voxel
		while (i < input_data.size())
		{
			unsigned int j = i + 1;
			while ((j < input_data.size()) && (point_to_subgrid_list[i].grid_idx == point_to_subgrid_list[j].grid_idx))
			{
				++j;
			}
			//grid_to_occupied[point_to_subgrid_list[i].grid_idx] = static_cast<unsigned int>(occupied_grid_cnt);
			occupied_grid_cnt++;
			i = j;
		}

		log_info("occupied_grid_cnt =%d", occupied_grid_cnt);
		occupied_voxel_array.clear();
		occupied_voxel_array.resize(occupied_grid_cnt);

		occupied_grid_cnt = 0;
		for (unsigned int i = 0; i < input_data.size();)
		{
			unsigned int j = i + 1;
			while ((j < input_data.size()) && (point_to_subgrid_list[i].grid_idx == point_to_subgrid_list[j].grid_idx))
			{
				//occupied_voxel_array[occupied_grid_cnt].push_back(point_to_subgrid_list[j].cloud_point_index);
				j++;
			}			
			//occupied_voxel_array[occupied_grid_cnt].grid_idx = static_cast<unsigned int>(point_to_subgrid_list[i].grid_idx);
			//occupied_voxel_array[occupied_grid_cnt].clear();
			occupied_voxel_array[occupied_grid_cnt].resize(j - i);
			i = j;
			occupied_grid_cnt++;
		}

		//assign input point index to the occupied voxel array
		occupied_grid_cnt = 0;
		for (unsigned int i = 0; i < input_data.size();)
		{
			unsigned int current_index = 0;
			occupied_voxel_array[occupied_grid_cnt][current_index] = point_to_subgrid_list[i].cloud_point_index;
			unsigned int j = i + 1;
			current_index = 1;
			while ((j < input_data.size()) && (point_to_subgrid_list[i].grid_idx == point_to_subgrid_list[j].grid_idx))
			{
				//voxel_array[occupied_grid_cnt].points.point_idx[current_index] = point_to_subgrid_list[j].cloud_point_index;
				occupied_voxel_array[occupied_grid_cnt][current_index] = point_to_subgrid_list[j].cloud_point_index;
				current_index++;
				j++;
			}
			i = j;
			occupied_grid_cnt++;
		}
		return  true;
	}

	/**
	* \brief compute the density of input data according to NormalEstimationParamters
	* @param [in] input_data, input points cloud
	* @param [in] occupied_voxel_array,
	* @param [out]  point_density, point density of input data
	* */
	static inline void get_density_test(vector<ModuleStruct::Point3f>& input_data, const VoxelParams voxel_size, interim_value_type& point_density, const int step)
	{
		TIMING_DECLARE(TP1)
		TIMING_BEGIN(TP1)
		vector<vector<unsigned int>> occupied_voxel_array;
		OccupiedVoxelInit(input_data, voxel_size, occupied_voxel_array);
		TIMING_END_ms("OccupiedVoxelInit",TP1)

		TIMING_BEGIN(TP1)

		PtDensityDetc(input_data, occupied_voxel_array, point_density, step);
		TIMING_END_ms("PtDensityDetc", TP1)
	}


	/**
	* \brief compute the density of input data according to NormalEstimationParamters
	* @param [in] input_data, input points cloud
	* @param [in] occupied_voxel_array,
	* @param [out]  point_density, point density of input data
	* */
	static inline void get_density_from_normalEst(vector<ModuleStruct::Point3f>&input_data, NormalEstimationResults &normal_result, interim_value_type&point_density, const int step)
	{
		Vector<Vector<unsigned int>> occupied_voxel_array;
		int nVxls = static_cast<int>(normal_result.voxel_array.size());
		occupied_voxel_array.clear(); occupied_voxel_array.resize(nVxls);
#pragma omp parallel for
		for (int i = 0; i < nVxls; i++)
		{
			occupied_voxel_array[i] = (normal_result.voxel_array[i].points_index);
		}

		PtDensityDetc(input_data, occupied_voxel_array, point_density, step);
	}	
}

#endif /*_UTIL_PLANE_FIT_HPP_*/
