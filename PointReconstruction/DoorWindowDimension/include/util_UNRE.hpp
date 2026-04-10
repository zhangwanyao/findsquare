#ifndef _UTIL_UNRE_HPP_
#define _UTIL_UNRE_HPP_

#include <vector>
#include <string>
#include <opencv2/core.hpp>
#include "plane_seg_inf.h"
#include "pxl_struct_inf.h"
#include "ModuleStruct.hpp"
#include "util_opencv.hpp"
using namespace std;
using namespace ModuleStruct;

#if defined WIN32 || defined _WIN32 || defined __CYGWIN__
#define UNRE_DLL_IMPORT __declspec(dllimport)
#define UNRE_DLL_EXPORT __declspec(dllexport)
#define UNRE_DLL_LOCAL
#else
#if __GNUC__ >= 4
#define UNRE_DLL_IMPORT __attribute__ ((visibility ("default")))
#define UNRE_DLL_EXPORT __attribute__ ((visibility ("default")))
#define UNRE_DLL_LOCAL  __attribute__ ((visibility ("hidden")))
#else
#define UNRE_DLL_IMPORT
#define UNRE_DLL_EXPORT
#define UNRE_DLL_LOCAL
#endif
#endif

namespace util_UNRE
{

#pragma pack(push, 1)
	typedef struct sPointInfo
	{
		float x;
		float y;
		float z;
		unsigned char i;
	}sPointInfo;
#pragma pack(pop)

	extern "C" bool UNRE_DLL_IMPORT WriteEncryFile(Vector<sPointInfo> * vecPtInfos, std::string strPath);
	extern "C" bool UNRE_DLL_IMPORT ReadEncryFile(std::vector<sPointInfo> * vecPtInfos, std::string strPath);
	static float sigma_ratio = 2.5;
	static float line_close_points_dist = 30.0f;
	static float line_points_dist = 50.0f;
	/**
	* \brief data struct of intensity distribution of plane
	*/
	struct IntensityDistribution
	{
		interim_value_type inten_mean;
		interim_value_type inten_sigma;
		value_type scaler_ratio;
		unsigned char inten_max;
		unsigned char inten_min;
		unsigned char reserved[2];
	};


	/**
	* \brief data struct of line close points and its properties of a plane
	*/
	struct LineClosePointsItem
	{
		/**
		* \brief intensity dsitribution of line close points
		*/
		IntensityDistribution inten_distrib;
	};

	struct PlaneLineClosePointsItem
	{

		/**
		* \brief point distance to one line segment of points in a plane
		*/
		float pts_ln_dist;

		bool  not_close;

		/**
		* \brief foot point to one line segment of points in a plane
		*/
		ModuleStruct::Point3f foot_pts;

		/**
		* \brief flag of inside points ,size is equal to subline size of one line in a plane
		*/
		std::vector<unsigned int> subline_inside;

		/**
		* \brief  flag of candidate points to compute sigma of subline , size is equal to subline size
		*/
		std::vector<unsigned int> sub_line_candidate;
	};

	struct LineSegmentType
	{
		ModuleStruct::Point3f line_seg_start;
		ModuleStruct::Point3f line_seg_end;
	};

	/**
	* \brief data struct of plane segmentation output with intensity
	*/
	struct PlaneSegOutputWithInten
	{
		ModuleStruct::Vector<sPointInfo> sPoints;
		/**
		* \brief all the points index  merged into this plane
		*/
		ModuleStruct::Vector<unsigned int> point_ids;
		/**
		* \brief center of all points in the plane
		*/
		ModuleStruct::Point3f plane_center;
		/**
		* \brief normal of the plane
		*/
		ModuleStruct::Point3f plane_normal;
		/**
		* \brief mean squred errors of all the points in the plane
		*/
		ModuleStruct::value_type plane_mse;

		/**
		* \brief area of the plane
		*/
		ModuleStruct::interim_value_type plane_area;

		/**
		* \brief intensity distribution of plane
		*/
		IntensityDistribution inten_distrib;

	};


	static void set_sigma_ratio(float f_sigma_ratio)
	{
		sigma_ratio = f_sigma_ratio;
	}
	inline static bool ReadEncryptionData(std::string path, std::vector<cv::Point3f>& data)
	{
		std::vector<sPointInfo> encryptData;
		bool rtn = ReadEncryFile(&encryptData, path);
		if (!rtn)
		{
			//log_error("ReadEncryFile error path =  %s", path.c_str());
			return false;
		}
		data.resize(encryptData.size());
		for (uint i = 0; i < encryptData.size(); i++)
		{
			data[i].x = encryptData[i].x, data[i].y = encryptData[i].y, data[i].z = encryptData[i].z;
		}
		return true;
	}

	static bool FilterInputDataWithInten(ModuleStruct::Point3Array& scene_xyz, ModuleStruct::Point3Array& filtered_scene_xyz, \
		std::vector<unsigned char>& src_intensityVec, std::vector<unsigned char>& out_intensityVec)
	{
		if (src_intensityVec.size() != scene_xyz.size())
		{
			//log_error("FilterInputDataWithInten input error src_intensityVec size =%d scene_xyz size= %d", src_intensityVec.size(), scene_xyz.size());
			return false;
		}
		const float max_size_x = 5000.0f;
		const float max_size_y = 5000.0f;
		const float rough_scale = 2.f;
		const float SceneDataRemoveNoiseFactor = 1.5f;

		ModuleStruct::Point3d scene_data_mean = { 0.0,0.0,0.0 };

		size_t size_of_input = scene_xyz.size();

		bool* is_inside = new bool[size_of_input];
		double thrld_size_1 = rough_scale * max_size_x;

		int size_of_filtered = 0;
		double sum_x = 0;
		double sum_y = 0;
#pragma omp parallel for reduction (+:size_of_filtered,sum_x,sum_y)
		for (int i = 0; i < scene_xyz.size(); i++)
		{
			is_inside[i] = false;
			bool condition = (fabs(scene_xyz[i].x) < thrld_size_1) && (fabs(scene_xyz[i].y) < thrld_size_1);
			if (!condition) continue;
			sum_x += scene_xyz[i].x;
			sum_y += scene_xyz[i].y;
			size_of_filtered++;
		}

		scene_data_mean.x = sum_x / size_of_filtered;
		scene_data_mean.y = sum_y / size_of_filtered;

		double thrld_size_2 = SceneDataRemoveNoiseFactor * max_size_x;
		size_of_filtered = 0;

#pragma omp parallel for reduction (+:size_of_filtered)
		for (int i = 0; i < scene_xyz.size(); i++)
		{
			bool condition = (fabs(scene_xyz[i].x - scene_data_mean.x) < thrld_size_2) && (fabs(scene_xyz[i].y - scene_data_mean.y) < thrld_size_2);
			if (condition)
			{
				is_inside[i] = true;
				size_of_filtered++;
			}
		}

		filtered_scene_xyz.resize(size_of_filtered);
		out_intensityVec.resize(size_of_filtered);
		for (int i = 0, j = 0; i < scene_xyz.size(); i++)
		{
			if (is_inside[i])
			{
				filtered_scene_xyz[j] = scene_xyz[i];
				out_intensityVec[j] = src_intensityVec[i];
				j++;
			}
		}
		delete[] is_inside;
		return true;
	}


	inline static bool ReadEncryptionDataWithIntensity(std::string path, std::vector<cv::Point3f>& data, std::vector<unsigned char>& intensityVec)
	{
		std::vector<sPointInfo> encryptData;
		bool rtn = ReadEncryFile(&encryptData, path);
		if (!rtn)
		{
			//log_error("ReadEncryFile error path =  %s", path.c_str());
			return false;
		}
		data.resize(encryptData.size());
		intensityVec.resize(encryptData.size());
		for (uint i = 0; i < encryptData.size(); i++)
		{
			data[i].x = encryptData[i].x, data[i].y = encryptData[i].y, data[i].z = encryptData[i].z;
			intensityVec[i] = encryptData[i].i;
		}
		return true;
	}

	/**
	* \brief  get angle value from direction file
	* @param [in] dire_file ,  UNRE direction file path
	* @param [in] delimiter,  UNRE direction angle delimiter, usually use","
	* @param [out] rot_angle, UNRE direction angle value
	* return true is ok,else is error
	* */

	inline static bool ReadDirection(std::string dire_file, std::string delimiter, float& rot_angle)
	{
		ifstream fin;
		std::string line;
		size_t begin_pos = 0;
		fin.open(dire_file, ios::in);
		if (!fin.is_open()) {
			cout << "ReadDirection open error, file name: " << dire_file << endl;
			return false;
		}
		int cnt = 0;
		while (!fin.eof())
		{
			std::getline(fin, line);
			if (!line.empty()) break;
		}

		if (line.empty())
		{
			return false;
		}
		int pos = static_cast<int>(line.find(delimiter, begin_pos));
		if (pos < 0)
		{
			return false;
		}
		std::string angle_num = line.substr(begin_pos, pos);
		cout << "ReadDirection angle_num: " << angle_num << endl;
		rot_angle = std::stof(angle_num);
		fin.close();
		return true;
	}

	/**
	* \brief  get angle process according to input pts name, direction file often lie on the same path of input load file name
	* @param [in] pts_file ,  input data pts file path
	* @param [out] rot_angle, UNRE direction angle value
	* return true is ok,else is error
	* */
	inline static bool GetDirectionAngle(std::string pts_file, float& rot_angle)
	{
		size_t iPos = pts_file.find_last_of('\\') + 1;
		std::string input_path = pts_file.substr(0, iPos);
		std::string dire_file = input_path + "dire.txt";
		//log_info("GetDirectionAngle dire_file =%s", dire_file.c_str());
		if (!ReadDirection(dire_file, ",", rot_angle))
		{
			//log_error("ReadDirection file %s error", dire_file.c_str());
			return false;
		}
		//log_info("rot angle =%f", rot_angle);
		return true;
	}

	static bool SyncDownsampleIntensity(std::vector<unsigned int> ds_point_to_centriod, std::vector<unsigned char>& intensityVec, \
		std::vector<unsigned char>& ds_intensityVec)
	{
		//check input
		if (ds_point_to_centriod.empty() || intensityVec.empty() || ds_intensityVec.empty() || ds_point_to_centriod.size() != intensityVec.size())
		{
			//log_error("input error ds_point_to_centriod size %d intensityVec size%d ds_intensityVec size%d", ds_point_to_centriod.size(), intensityVec.size(), ds_intensityVec.size());
			return false;
		}
#pragma omp parallel for
		for (int i = 0; i < ds_point_to_centriod.size(); i++)
		{
			unsigned int c_idx = ds_point_to_centriod[i];
			ds_intensityVec[c_idx] = intensityVec[i];
		}

	}

	static bool PlaneSegOutputConvert(const std::vector<Point3f>& input_data, std::vector<PlaneSegOutput>& seg_planes, \
		const std::vector<unsigned char>& intensityVec, std::vector<PlaneSegOutputWithInten>& out_planes)
	{
		//check input
		if ((input_data.empty()) || (seg_planes.size() == 0) || (input_data.size() != intensityVec.size()))
		{
			//log_info("input error seg_planes size =%d intensityVec size =%d input_data size =%d", seg_planes.size(), intensityVec.size(), input_data.size());
			return false;
		}

		out_planes.clear();
		out_planes.resize(seg_planes.size());
#pragma omp parallel for
		for (int i = 0; i < seg_planes.size(); i++)
		{
			PlaneSegOutput* in_p = &seg_planes[i];
			PlaneSegOutputWithInten* out_p = &out_planes[i];
			out_p->point_ids = in_p->point_ids;
			out_p->plane_center = in_p->plane_center;
			out_p->plane_normal = in_p->plane_normal;
			out_p->plane_mse = in_p->plane_mse;
			out_p->plane_area = in_p->plane_area;
			out_p->sPoints.resize(in_p->points.size());
			for (size_t j = 0; j < out_p->sPoints.size(); j++)
			{
				size_t point_ids = out_p->point_ids[j];
				out_p->sPoints[j].x = input_data[point_ids].x;
				out_p->sPoints[j].y = input_data[point_ids].y;
				out_p->sPoints[j].z = input_data[point_ids].z;
				out_p->sPoints[j].i = intensityVec[point_ids];
			}
			out_p->inten_distrib.inten_sigma = 0.0;
			out_p->inten_distrib.inten_mean = 0.0;
			out_p->inten_distrib.inten_max = 0;
			out_p->inten_distrib.inten_min = 255;
			out_p->inten_distrib.scaler_ratio = sigma_ratio;
		}
		//log_info("sigma_ratio =%f", sigma_ratio);
		return true;
	}

	static bool SinglePlaneSegOutputConvert(const std::vector<Point3f>& points, ModuleStruct::Point3f plane_center, ModuleStruct::Point3f plane_normal, \
		const std::vector<unsigned char>& intensityVec, PlaneSegOutputWithInten& out_planes)
	{
		//check input
		if ((points.empty()))
		{
			//log_info("input error seg_planes size =%d intensityVec size =%d input_data size =%d", seg_planes.size(), intensityVec.size(), input_data.size());
			return false;
		}
		PlaneSegOutputWithInten* out_p = &out_planes;
		ModuleStruct::Vector<unsigned int> point_ids(points.size());
		out_p->plane_center = plane_center;
		out_p->plane_normal = plane_normal;
		//out_p->plane_mse = in_p->plane_mse;
		//out_p->plane_area = in_p->plane_area;
		out_p->sPoints.resize(points.size());
		out_p->point_ids.resize(points.size());
#pragma omp parallel for
		for (int j = 0; j < out_p->sPoints.size(); j++)
		{
			out_p->point_ids[j] = j;
			//size_t point_ids = out_p->point_ids[j];
			out_p->sPoints[j].x = points[j].x;
			out_p->sPoints[j].y = points[j].y;
			out_p->sPoints[j].z = points[j].z;
			out_p->sPoints[j].i = intensityVec[j];
		}
		out_p->inten_distrib.inten_sigma = 0.0;
		out_p->inten_distrib.inten_mean = 0.0;
		out_p->inten_distrib.inten_max = 0;
		out_p->inten_distrib.inten_min = 255;
		out_p->inten_distrib.scaler_ratio = sigma_ratio;
		//log_info("sigma_ratio =%f", sigma_ratio);
		return true;
	}


	static bool save_oneplane_xyzi(std::string path_name, std::string delimiter, PlaneSegOutputWithInten& plane)
	{
		if (path_name.empty() || (plane.sPoints.empty())) {
			//log_error("save_plane_xyzi input error ");
			return false;
		}

		ofstream fout(path_name);
		if (fout.fail()) {
			//log_error("save_plane_xyzi cannot open %s", path_name.c_str());
			return false;
		}

		//ModuleStruct::Point3f point;
		for (unsigned int i = 0; i < plane.sPoints.size(); i++) {

			sPointInfo point = plane.sPoints[i];
			unsigned int idx = plane.point_ids[i];
			fout << point.x << delimiter << point.y << delimiter << point.z << delimiter << static_cast<int>(point.i) << delimiter << idx;
			if (i < plane.sPoints.size() - 1) {
				fout << std::endl;
			}
		}
		fout.close();
		return true;
	}




	static inline int save_plane_xyzi(const std::string sub_path, const std::string sub_file_name, const std::string delimiter, \
		const std::string file_type, ModuleStruct::Point3fArray& input_data, std::vector<PlaneSegOutputWithInten>& plane_seg_out)
	{

		if (IOData::createDirectory(sub_path) < 0)
		{
			//log_error(" save_plane_xyzi createDirectory %s failed", sub_path.c_str());
			return -1;
		}

		for (unsigned int i = 0; i < plane_seg_out.size(); i++)
		{
			std::stringstream plane_id;
			plane_id << i;
			std::string whole_path = sub_path + sub_file_name + plane_id.str() + file_type;
			bool rtn = save_oneplane_xyzi(whole_path, delimiter, plane_seg_out[i]);
			if (!rtn)
			{
				//log_error("save_plane_xyzi %s plane %d return error", whole_path.c_str(), i);
				return -1;
			}
		}
		return 0;
	}

	static inline bool save_planes_with_insten(const std::string output_path, std::vector<Point3f>& input_points, \
		std::vector<PlaneSegOutput>& plane_seg_out, std::vector<unsigned char>& intensityVec)
	{
		if (intensityVec.empty())
		{
			//log_warn("save_planes_with_insten input empty");
			return true;
		}

		std::vector<PlaneSegOutputWithInten>out_planes;
		if (!PlaneSegOutputConvert(input_points, plane_seg_out, intensityVec, out_planes))
		{
			//log_error("PlaneSegOutputConvert return error");
			return false;
		}

		std::string sub_path = output_path + "plane_xyzi\\";
		int rtn = save_plane_xyzi(sub_path, "plane_xyzi", ";", ".txt", input_points, out_planes);
		if (rtn)
		{
			//log_error("save_plane_xyzi return error");
			return false;
		}

		return true;
	}

	static inline int save_plane_xyzi_with_noisy(const std::string sub_path, const std::string sub_file_name, const std::string delimiter, \
		const std::string file_type, ModuleStruct::Point3fArray& input_data, std::vector<PlaneSegOutputWithInten>& plane_seg_out, std::vector<std::vector<unsigned char>>& is_noisy)
	{
		if (IOData::createDirectory(sub_path) < 0)
		{
			//log_error(" save_plane_xyzi_with_noisy createDirectory %s failed", sub_path.c_str());
			return -1;
		}

		//size_t iPos = sub_path.find_last_of('\\');

		//std::string noisy_sub_path = sub_path.substr(0, iPos)+"_noisy\\";
		//if (IOData::createDirectory(noisy_sub_path) < 0)
		//{
		//	log_error(" save_plane_xyzi_with_noisy createDirectory %s failed", noisy_sub_path.c_str());
		//	return -1;
		//}

		std::string noisy_sub_path = sub_path;
		for (unsigned int i = 0; i < plane_seg_out.size(); i++)
		{
			PlaneSegOutputWithInten* plane_it = &plane_seg_out[i];
			IntensityDistribution* inten_distr = &plane_it->inten_distrib;

			if (plane_it->sPoints.size() == 0)
			{
				//log_error("save_plane_xyzi_with_noisy error: input size is zero");
				continue;
			}

			std::stringstream plane_id;
			plane_id << i;
			std::string whole_path = sub_path + sub_file_name + plane_id.str() + file_type;
			//bool rtn = save_oneplane_xyzi(whole_path, delimiter, plane_seg_out[i]);
			ofstream fout(whole_path);
			if (fout.fail()) {
				//log_error("save_plane_xyzi_with_noisy cannot open %s", whole_path.c_str());
				return -1;
			}

			//first save non-noisy points
			for (unsigned int j = 0; j < plane_it->sPoints.size(); j++) {

				if (is_noisy[i][j] == 1) continue;
				unsigned int idx = plane_it->point_ids[j];
				sPointInfo point = plane_it->sPoints[j];
				fout << point.x << delimiter << point.y << delimiter << point.z << delimiter << static_cast<int>(point.i) << delimiter << idx;
				if (i < plane_it->sPoints.size() - 1) {
					fout << std::endl;
				}
			}
			fout.close();

			//second save noisy points in other path
			std::string noisy_sub_file_name = sub_file_name + "_noisy_";
			std::string noisy_whole_path = noisy_sub_path + noisy_sub_file_name + plane_id.str() + file_type;
			ofstream noisy_fout(noisy_whole_path);
			if (noisy_fout.fail()) {
				//log_error("save_plane_xyzi_with_noisy cannot open %s", noisy_whole_path.c_str());
				return -1;
			}

			for (unsigned int j = 0; j < plane_it->sPoints.size(); j++) {

				if (is_noisy[i][j] == 0) continue;
				unsigned int idx = plane_it->point_ids[j];
				sPointInfo point = plane_it->sPoints[j];
				noisy_fout << point.x << delimiter << point.y << delimiter << point.z << delimiter << static_cast<int>(point.i) << delimiter << idx;
				if (i < plane_it->sPoints.size() - 1) {
					noisy_fout << std::endl;
				}
			}
			noisy_fout.close();
		}
		return 0;
	}



	static bool save_input_xyzi(const std::string path_name, const std::string delimiter, const std::vector<Point3f>& input_data, std::vector<unsigned char>& input_intensity)
	{
		//check input
		if ((input_data.empty()) || (input_intensity.empty()) || (input_data.size() != input_intensity.size()))
		{
			//log_info("input input_intensity size =%d input_data size =%d", input_intensity.size(), input_data.size());
			return false;
		}

		ofstream fout(path_name);
		if (fout.fail()) {
			//log_error("save_plane_xyzi cannot open %s", path_name.c_str());
			return false;
		}

		for (unsigned int i = 0; i < input_data.size(); i++) {

			fout << input_data[i].x << delimiter << input_data[i].y << delimiter << input_data[i].z << delimiter << static_cast<int>(input_intensity[i]);
			if (i < input_data.size() - 1) {
				fout << std::endl;
			}
		}
		fout.close();
		return true;
	}



	static void sgm_compute(std::vector<PlaneSegOutputWithInten>& input_planes)
	{
		int n_pls = input_planes.size();
#pragma omp parallel for
		for (int i = 0; i < n_pls; i++)
		{
			PlaneSegOutputWithInten* plane_it = &input_planes[i];
			IntensityDistribution* inten_distr = &plane_it->inten_distrib;
			for (int j = 0; j < plane_it->sPoints.size(); j++)
			{
				inten_distr->inten_mean += plane_it->sPoints[j].i;
				//if ((plane_it->sPoints[j].i == 0) || (plane_it->sPoints[j].i == 255))
				//{
				//	log_info("plane %d idx%d intensity =%d",i, plane_it->point_ids[j], plane_it->sPoints[j].i);
				//}

				if (plane_it->sPoints[j].i < inten_distr->inten_min) inten_distr->inten_min = plane_it->sPoints[j].i;
				if (plane_it->sPoints[j].i > inten_distr->inten_max) inten_distr->inten_max = plane_it->sPoints[j].i;
			}
			inten_distr->inten_mean = inten_distr->inten_mean / plane_it->sPoints.size();
			//log_info("plane %d intensity mean =%f max=%d min=%d",i, inten_distr->inten_mean, inten_distr->inten_max, inten_distr->inten_min);
		}

		// sigma =sqrt(sum((X-E(X))^2)/(n-1));
#pragma omp parallel for
		for (int i = 0; i < n_pls; i++)
		{
			PlaneSegOutputWithInten* plane_it = &input_planes[i];
			IntensityDistribution* inten_distr = &plane_it->inten_distrib;
			for (int j = 0; j < plane_it->sPoints.size(); j++)
			{
				inten_distr->inten_sigma += (static_cast<double>(plane_it->sPoints[j].i) - inten_distr->inten_mean) * (static_cast<double>(plane_it->sPoints[j].i) - inten_distr->inten_mean);
			}

			if (plane_it->sPoints.size() < 2)
			{
				//log_error("plane %d have too few points size =%d",i, plane_it->sPoints.size());
				continue;
			}
			inten_distr->inten_sigma = inten_distr->inten_sigma / (plane_it->sPoints.size() - 1);
			inten_distr->inten_sigma = sqrt(inten_distr->inten_sigma);
			//log_info("plane %d intensity sigma =%f", i, inten_distr->inten_sigma);
		}
	}

	static void plane_noisy_compute(std::vector<PlaneSegOutputWithInten>& input_planes, std::vector<std::vector<unsigned char>>& is_noisy)
	{
		int n_pls = input_planes.size();

#pragma omp parallel for
		for (int i = 0; i < n_pls; i++)
		{
			PlaneSegOutputWithInten* plane_it = &input_planes[i];
			IntensityDistribution* inten_distr = &plane_it->inten_distrib;
			interim_value_type min_diff = inten_distr->scaler_ratio * inten_distr->inten_sigma;
			for (int j = 0; j < plane_it->sPoints.size(); j++)
			{
				is_noisy[i][j] = 0;
				interim_value_type inten_diff = std::abs(static_cast<interim_value_type>(plane_it->sPoints[j].i) - inten_distr->inten_mean);
				if (inten_diff > min_diff)
				{
					is_noisy[i][j] = 1;
				}
			}
		}
		//log_info("plane_noisy_compute end");
	}

	static inline bool plane_noisy_process(const std::string output_path, const std::vector<Point3f>& input_points, std::vector<PlaneSegOutput>& plane_seg_out, \
		std::vector<unsigned char>& intensityVec, std::vector<std::vector<unsigned char>>& is_noisy, std::vector<PlaneSegOutputWithInten>& out_planes)
	{
		if (intensityVec.empty())
		{
			//log_warn("save_planes_with_insten input empty");
			return true;
		}
		if (!PlaneSegOutputConvert(input_points, plane_seg_out, intensityVec, out_planes))
		{
			//log_error("PlaneSegOutputConvert return error");
			return false;
		}

		sgm_compute(out_planes);
		is_noisy.clear();
		is_noisy.resize(out_planes.size());
		for (int i = 0; i < out_planes.size(); i++)
		{
			is_noisy[i].resize(out_planes[i].sPoints.size());
		}
		plane_noisy_compute(out_planes, is_noisy);
		return true;
	}


	static inline bool save_planes_with_inten_noisy(const std::string output_path, std::vector<Point3f>& input_points, \
		std::vector<PlaneSegOutput>& plane_seg_out, std::vector<unsigned char>& intensityVec, std::vector<std::vector<unsigned char>>& is_noisy)
	{
		std::vector<PlaneSegOutputWithInten>out_planes;
		plane_noisy_process(output_path, input_points, plane_seg_out, intensityVec, is_noisy, out_planes);
		std::string sub_path = output_path + "plane_xyzi\\";
		int rtn = save_plane_xyzi_with_noisy(sub_path, "plane_xyzi", ";", ".txt", input_points, out_planes, is_noisy);
		if (rtn)
		{
			//log_error("save_plane_xyzi return error");
			return false;
		}

		//std::string folder = output_path + "plane_center_normal\\";
		//std::string file_name = "plane_center_normal";
		//rtn = util_plane_seg::save_plane_normal_center(2, folder, file_name, plane_seg_out);
		//if (rtn)
		//{
		//	log_error("save_plane_normal_center normal and center return error");
		//	return false;
		//}

		return true;
	}

	static bool plane_remove_noisy(const std::string output_path, std::vector<std::vector<unsigned char>>& is_noisy, std::vector<PlaneSegOutput>& input_planes, \
		std::vector<PlaneSegOutput>& output_planes)
	{

		output_planes.clear();
		output_planes.resize(input_planes.size());
		if (is_noisy.size() != input_planes.size())
		{
			//log_error("is_noisy error is_noisy size =%d not equal to input planes size =%d", is_noisy.size(), input_planes.size());
			return false;
		}
		//#pragma omp parallel for
		for (int i = 0; i < input_planes.size(); i++)
		{
			PlaneSegOutput* in_p = &input_planes[i];
			PlaneSegOutput* out_p = &output_planes[i];
			//out_p->point_ids = in_p->point_ids;
			out_p->plane_center = in_p->plane_center;
			out_p->plane_normal = in_p->plane_normal;
			out_p->plane_mse = in_p->plane_mse;
			out_p->plane_area = in_p->plane_area;
			if (is_noisy[i].size() != in_p->point_ids.size())
			{
				//log_error("is_noisy[%d] error size =%d not equal to plane%d size =%d", i,is_noisy[i].size(),i, in_p->point_ids.size());
				return false;
			}
			//get cnt of non noisy points size
			int tmp_cnt = 0;
			for (int j = 0; j < is_noisy[i].size(); j++)
			{
				if (is_noisy[i][j] == 0)
				{
					tmp_cnt++;
				}
			}
			out_p->point_ids.resize(tmp_cnt);
			out_p->points.resize(tmp_cnt);

			int tmp_idx = 0;
			for (int j = 0; j < is_noisy[i].size(); j++)
			{
				if (is_noisy[i][j] == 0)
				{
					out_p->point_ids[tmp_idx] = in_p->point_ids[j];
					out_p->points[tmp_idx] = in_p->points[j];
					tmp_idx++;
				}
			}
		}
		return true;
	}

	static bool single_plane_remove_noisy(
		const std::string output_path, std::vector<unsigned char>& is_noisy,
	    const std::vector<Point3f>& input_points, 
		const ModuleStruct::Point3f& plane_normal, 
		const ModuleStruct::Point3f& plane_center, 
		PlaneSegOutput& output_planes)
	{
		PlaneSegOutput* out_p = &output_planes;
		//out_p->point_ids = in_p->point_ids;
		out_p->plane_center = plane_center;
		out_p->plane_normal = plane_normal;
		if (is_noisy.size() != input_points.size())
		{
			std::cout << "is_noisy and input_points size not equal" << std::endl;
			return false;
		}
		//get cnt of non noisy points size
		int tmp_cnt = 0;
		for (int j = 0; j < is_noisy.size(); j++)
		{
			if (is_noisy[j] == 0)
			{
				tmp_cnt++;
			}
		}
		out_p->point_ids.resize(tmp_cnt);
		out_p->points.resize(tmp_cnt);

		int tmp_idx = 0;
		for (int j = 0; j < is_noisy.size(); j++)
		{
			if (is_noisy[j] == 0)
			{
				out_p->point_ids[tmp_idx] = j;
				out_p->points[tmp_idx] = input_points[j];
				tmp_idx++;
			}
		}
		return true;
	}
	/**
	* \brief compute  a point project on a line, perpendicular foot point to a line
	* reference to https://blog.csdn.net/zhouschina/article/details/14647587
	* @param pt point out of line
	* @param line_pt  point on the line
	* @param direction of the line
	* @return the project of the point out of line
	*/
	static ModuleStruct::Point3f GetFootOfPerpendicular(const ModuleStruct::Point3f pt, const ModuleStruct::Point3f P0, const ModuleStruct::Point3f P1)
	{
		float dx = P0.x - P1.x;
		float dy = P0.y - P1.y;
		float dz = P0.z - P1.z;
		if (std::fabs(dx) < std::numeric_limits<float>::epsilon()\
			&& std::fabs(dy) < std::numeric_limits<float>::epsilon()\
			&& std::fabs(dz) < std::numeric_limits<float>::epsilon())
		{
			return P0;
		}

		//double u = (pt.x - begin.x) * (begin.x - end.x) +
		//	(pt.y - begin.y) * (begin.y - end.y) + (pt.z - begin.z) * (begin.z - end.z);
		//u = u / ((dx * dx) + (dy * dy) + (dz * dz));

		//retVal.x = begin.x + u * dx;
		//retVal.y = begin.y + u * dy;
		//retVal.y = begin.z + u * dz;

		double u = (pt.x - P0.x) * dx + (pt.y - P0.y) * dy + (pt.z - P0.z) * dz;
		u = u / ((dx * dx) + (dy * dy) + (dz * dz));
		ModuleStruct::Point3f footPoint;
		footPoint.x = P0.x + u * dx;
		footPoint.y = P0.y + u * dy;
		footPoint.z = P0.z + u * dz;
		return footPoint;
	}


	/**
	* \brief determine if a point is inside a segment, reference: http://geomalgorithms.com/a05-_intersect-1.html#intersect2D_2Segments()
	* @param [in] P, point
	* @param [in] S, a collinear segment
	* return   1 = P is inside S, 0 = P is  not inside S
	* */
	static inline int inSegment(const ModuleStruct::Point3f P, const LineSegmentType& S)
	{
		if (S.line_seg_start.x != S.line_seg_end.x) {    // S is not  vertical
			if (S.line_seg_start.x <= P.x && P.x <= S.line_seg_end.x)
				return 1;
			if (S.line_seg_start.x >= P.x && P.x >= S.line_seg_end.x)
				return 1;
		}
		else {    // S is vertical, so test y  coordinate
			if (S.line_seg_start.y <= P.y && P.y <= S.line_seg_end.y)
				return 1;
			if (S.line_seg_start.y >= P.y && P.y >= S.line_seg_end.y)
				return 1;
		}
		return 0;
	}
	static bool save_oneplane_sublines_xyzi(const std::string sub_path, const std::string delimiter, const int plane_idx, \
		const int line_idx, const int subline_size, const std::vector<ModuleStruct::Point3f>& input_data, const PlaneSegOutputWithInten* plane, \
		std::vector<PlaneLineClosePointsItem>& line_close_pts, std::vector<unsigned char>& is_noisy_in_line)
	{
		//first save non-noisy points
		if (IOData::createDirectory(sub_path) != 0)
		{
			std::cout << "can not create path: " << sub_path << std::endl;
			return false;
		}

		std::stringstream plane_id;
		plane_id << plane_idx;
		std::stringstream line_id;
		line_id << line_idx;
		for (unsigned int i = 0; i < subline_size; i++)
		{
			std::stringstream sub_line_id;
			sub_line_id << i;
			std::string whole_path = sub_path + "xyzi_P" + plane_id.str() + "_L" + line_id.str() + "_subL" + sub_line_id.str() + ".xyz";
			ofstream fout(whole_path);
			if (fout.fail()) {
				//log_error("save_oneplane_lines_xyzi cannot open %s", whole_path.c_str());
				return false;
			}

			for (unsigned int j = 0; j < plane->sPoints.size(); j++) {

				if (line_close_pts[j].not_close) continue;
				if (line_close_pts[j].subline_inside[i] == 0) continue;
				if (is_noisy_in_line[j] == 1) continue;
				unsigned int idx = plane->point_ids[j];
				sPointInfo point = plane->sPoints[j];
				fout << point.x << delimiter << point.y << delimiter << point.z << delimiter << static_cast<int>(point.i) << delimiter << idx;
				if (j < plane->sPoints.size() - 1) {
					fout << std::endl;
				}
			}
			fout.close();

			//second save noisy points in other path
			std::string noisy_whole_path = sub_path + "xyzi_noisy_P" + plane_id.str() + "_L" + line_id.str() + "_subL" + sub_line_id.str() + ".xyz";
			ofstream noisy_fout(noisy_whole_path);
			if (noisy_fout.fail()) {
				//log_error("save_plane_xyzi_with_noisy cannot open %s", noisy_whole_path.c_str());
				return false;
			}

			for (unsigned int j = 0; j < plane->sPoints.size(); j++) {

				if (line_close_pts[j].not_close) continue;
				if (line_close_pts[j].subline_inside[i] == 0) continue;
				if (is_noisy_in_line[j] == 0) continue;
				unsigned int idx = plane->point_ids[j];
				sPointInfo point = plane->sPoints[j];
				noisy_fout << point.x << delimiter << point.y << delimiter << point.z << delimiter << static_cast<int>(point.i) << delimiter << idx;
				if (i < plane->sPoints.size() - 1) {
					noisy_fout << std::endl;
				}
			}
			noisy_fout.close();
		}
		return true;
	}

	/**
	* \brief get the distance of a point to a line,reference: http://geomalgorithms.com/a02-_lines.html
	* @param [in] P, a Point
	* @param [in] LP0, first Point of line
	* @param [in] LP1, second Point of line
	* return  dist of P to line (P0,P1)
	* */
#define dot_fc(u,v)   (u.x * v.x + u.y * v.y + u.z * v.z)
	static inline float dist_point_to_line(ModuleStruct::Point3f P, ModuleStruct::Point3f LP0, ModuleStruct::Point3f LP1)
	{
		ModuleStruct::Point3f v = LP1 - LP0;
		ModuleStruct::Point3f w = P - LP0;

		double c1 = dot_fc(w, v);
		double c2 = dot_fc(v, v);
		double b = c1 / c2;
		ModuleStruct::Point3f Pb = LP0 + b * v;
		ModuleStruct::Point3f delt_p = P - Pb;
		return static_cast<float>(sqrt(dot_fc(delt_p, delt_p)));
		//return static_cast<float>(Util_Math::ComputePointToPointDist<double, ModuleStruct::Point3f>(P, Pb));
		//return d(P, Pb);
	}

	static bool get_subline_bydist(const float dist, const LineSegmentType& line, std::vector<LineSegmentType>& sub_lines)
	{
		if (dist < std::numeric_limits<float>::epsilon())
		{
			//log_error("input error dist =%f", dist);
			std::cout << "input error dist = " << dist << std::endl;
			return false;
		}

		ModuleStruct::Point3f delt = line.line_seg_end - line.line_seg_start;
		double length = sqrt(dot_fc(delt, delt));
		int n_Ls = std::ceil(length / dist);
		if (n_Ls <= 0)
		{
			std::cout << "sub line size =" << n_Ls << std::endl;
			return false;
		}
		sub_lines.clear(); sub_lines.resize(n_Ls);
		delt /= n_Ls;
		for (int i = 0; i < n_Ls; i++)
		{
			sub_lines[i].line_seg_start = line.line_seg_start + i * delt;
			sub_lines[i].line_seg_end = line.line_seg_start + (i + 1) * delt;
		}
		return true;
	}

	static bool get_line_close_points(const float can_dist, const int sl_idx, const LineSegmentType& line, const PlaneSegOutputWithInten* plane, \
		const std::vector<ModuleStruct::Point3f>& input_points, std::vector<PlaneLineClosePointsItem>& plane_line_close_pts)
	{
		// get line candidate
		LineSegmentType subline_cand;
		ModuleStruct::Point3f delt_line = line.line_seg_end - line.line_seg_start;
		subline_cand.line_seg_start = line.line_seg_start - delt_line;
		subline_cand.line_seg_end = line.line_seg_end + delt_line;
#pragma omp parallel for
		for (int j = 0; j < plane->point_ids.size(); j++)
		{
			PlaneLineClosePointsItem* plane_line_close_it = &plane_line_close_pts[j];
			if (plane_line_close_it->not_close) continue;
			unsigned int point_ids = plane->point_ids[j];
			//ModuleStruct::Point3f point = input_points[point_ids];

			//compute point to line dist
			float c_dist = plane_line_close_it->pts_ln_dist;
			//if (dbg_con) log_info("plane%d line%d point%d  dist =%f", plane_idx, i, j, dist);
			ModuleStruct::Point3f foot_point = plane_line_close_it->foot_pts;
			if ((c_dist < can_dist + line_close_points_dist) && (c_dist > line_close_points_dist))
			{
				//check if footpoint is inside line segment
				plane_line_close_it->sub_line_candidate[sl_idx] = inSegment(foot_point, subline_cand);
			}

			if (c_dist < line_points_dist)
			{
				//check if footpoint is inside line segment
				plane_line_close_it->subline_inside[sl_idx] = inSegment(foot_point, line);
			}
		}
		return true;

	}

	static bool get_plane_line_close_points(const std::string output_path, const int plane_idx, 
		const std::vector<ModuleStruct::Point3f>input_points, \
		const PlaneSegOutputWithInten* plane, 
		std::vector<Line3DSegOutItemDebug>& lines, \
		const float sub_line_dist, std::vector<unsigned char>& is_noisy)
	{
		if (plane == NULL)
		{
			//log_error("input error");
			return false;
		}

		bool dbg_con = (plane_idx == 0);
		if (plane->point_ids.empty())
		{
			//log_error("input error plane->points empty");
			return false;
		}
		is_noisy.clear();
		is_noisy.resize(plane->sPoints.size());
		for (int i = 0; i < plane->sPoints.size(); i++)
		{
			is_noisy[i] = 0;
		}

		// if no line, assume plane have no noisy
		if (lines.empty())
		{
			return true;
		}

		//log_info("get_plane_line_close_points plane %d line size %d", plane_idx, lines.size());

		std::vector<std::vector<PlaneLineClosePointsItem>> plane_line_close_pts(lines.size());
		std::vector<LineSegmentType> ext_line_seg(lines.size());
		float candidate_dist = sub_line_dist * 2;

		// compute sub line and candidate points:input line and distance ,
		std::vector<std::vector<LineSegmentType>> plane_sub_lines;
		plane_sub_lines.clear(); plane_sub_lines.resize(lines.size());
		for (int k = 0; k < lines.size(); k++)
		{
			Line3DSegOutItemDebug* line_it = &lines[k];
			//std::vector<PlaneLineClosePointsItem> *l_close_pts = &plane_line_close_pts[k];
			ext_line_seg[k].line_seg_start = (line_it->line_seg_start - line_it->line_seg_end) * 0.1 + line_it->line_seg_start;
			ext_line_seg[k].line_seg_end = (line_it->line_seg_end - line_it->line_seg_start) * 0.1 + line_it->line_seg_end;
			if (!get_subline_bydist(sub_line_dist, ext_line_seg[k], plane_sub_lines[k]))
			{
				//log_error("get_subline_bydist error");
				return false;
			}
			size_t sub_line_size = plane_sub_lines[k].size();
			plane_line_close_pts[k].resize(plane->sPoints.size());
#pragma omp parallel for
			for (int i = 0; i < plane->sPoints.size(); i++)
			{
				PlaneLineClosePointsItem* line_close_pts_it = &plane_line_close_pts[k][i];
				unsigned int point_ids = plane->point_ids[i];
				ModuleStruct::Point3f point = input_points[point_ids];
				//compute point to line dist
				line_close_pts_it->pts_ln_dist = dist_point_to_line(point, line_it->line_seg_start, line_it->line_seg_end);
				if (line_close_pts_it->pts_ln_dist > candidate_dist + line_close_points_dist)
				{
					line_close_pts_it->not_close = true;
				}
				else {
					line_close_pts_it->not_close = false;
					line_close_pts_it->foot_pts = GetFootOfPerpendicular(point, line_it->line_seg_start, line_it->line_seg_end);
					line_close_pts_it->subline_inside.resize(sub_line_size);
					line_close_pts_it->sub_line_candidate.resize(sub_line_size);
				}
			}
			std::vector<LineClosePointsItem> line_close_points;
			line_close_points.clear();
			line_close_points.resize(sub_line_size);
#pragma omp parallel for
			for (int i = 0; i < sub_line_size; i++)
			{

				LineSegmentType* sub_line_it = &plane_sub_lines[k][i];
				LineClosePointsItem* line_close_pts_it = &line_close_points[i];
				IntensityDistribution* sub_inten_distr = &line_close_pts_it->inten_distrib;
				sub_inten_distr->inten_sigma = 0.0;
				sub_inten_distr->inten_mean = 0.0;
				sub_inten_distr->inten_max = 0;
				sub_inten_distr->inten_min = 255;
				sub_inten_distr->scaler_ratio = sigma_ratio;

				get_line_close_points(candidate_dist, i, *sub_line_it, plane, input_points, plane_line_close_pts[k]);

				int sub_line_close_point_num = 0;
				for (int j = 0; j < plane->sPoints.size(); j++)
				{
					if (!plane_line_close_pts[k][j].not_close && plane_line_close_pts[k][j].sub_line_candidate[i] > 0 && ((plane->sPoints[j].i >= 30) && (plane->sPoints[j].i <= 225)))
					{
						sub_inten_distr->inten_mean += plane->sPoints[j].i;
						//if ((plane_it->sPoints[j].i == 0) || (plane_it->sPoints[j].i == 255))
						//{
						//	log_info("plane %d idx%d intensity =%d",i, plane_it->point_ids[j], plane_it->sPoints[j].i);
						//}

						if (plane->sPoints[j].i < sub_inten_distr->inten_min) sub_inten_distr->inten_min = plane->sPoints[j].i;
						if (plane->sPoints[j].i > sub_inten_distr->inten_max) sub_inten_distr->inten_max = plane->sPoints[j].i;
						sub_line_close_point_num++;
					}
				}

				if (sub_line_close_point_num >= 2)
				{
					sub_inten_distr->inten_mean = sub_inten_distr->inten_mean / sub_line_close_point_num;
					for (int j = 0; j < plane->sPoints.size(); j++)
					{
						if (!plane_line_close_pts[k][j].not_close && plane_line_close_pts[k][j].sub_line_candidate[i] > 0)
						{
							sub_inten_distr->inten_sigma += (static_cast<double>(plane->sPoints[j].i) - sub_inten_distr->inten_mean) * (static_cast<double>(plane->sPoints[j].i) - sub_inten_distr->inten_mean);
						}
					}

					//log_error("plane %d line%d subline%d have too few points size =%d", plane_idx, k, i,sub_line_close_point_num);
					sub_inten_distr->inten_sigma = sub_inten_distr->inten_sigma / (sub_line_close_point_num - 1);
					sub_inten_distr->inten_sigma = sqrt(sub_inten_distr->inten_sigma);
					//log_info("plane %d line%d sline%d intensity mean =%f max=%d min =%d", plane_idx,k, i, sub_inten_distr->inten_mean, sub_inten_distr->inten_max, sub_inten_distr->inten_min);
					interim_value_type min_diff = sub_inten_distr->scaler_ratio * sub_inten_distr->inten_sigma;
					if (min_diff > 20.0)
					{
						min_diff = 20.0;
					}
					else if (min_diff < 5.0)
					{
						min_diff = 5.0;
					}

					//log_info("plane %d line%d sline%d  intensity sigma =%f min_diff=%f", plane_idx,k, i, sub_inten_distr->inten_sigma, min_diff);
					for (int j = 0; j < plane->sPoints.size(); j++)
					{
						//if (subline_inside[k][i][j] == 0) continue;
						if (!plane_line_close_pts[k][j].not_close && plane_line_close_pts[k][j].subline_inside[i] > 0)
						{
							interim_value_type inten_diff = std::abs(static_cast<interim_value_type>(plane->sPoints[j].i) - sub_inten_distr->inten_mean);
							if (inten_diff > min_diff)
							{
								is_noisy[j] = 1;
							}
						}
					}
				}

			}
		}

		//if (dbg_con) log_info("plane %d save_oneplane_sublines_xyzi begin", plane_idx);

		//std::string sub_path = output_path + "plane_line_close_xyzi\\";
		//bool dgb_con = (plane_idx == 5) || (plane_idx == 14) || (plane_idx == 22) || (plane_idx == 25);
		//if (dgb_con)
		//{
		//	for (int k = 0; k < lines.size(); k++)
		//	{
		//		size_t sub_line_size = plane_sub_lines[k].size();
		//		save_oneplane_sublines_xyzi(sub_path, " ", plane_idx, k, sub_line_size, input_points, plane, plane_line_close_pts[k], is_noisy);
		//	}
		//}

		return true;
	}

	static bool get_plane_noisy(const std::string output_path, const std::vector<Point3f>& input_points, std::vector<std::vector<Line3DSegOutItemDebug>>& planes_lines, \
		const float sub_line_dist, std::vector<PlaneSegOutput>& plane_seg_out, const std::vector<unsigned char>& intensityVec, std::vector<std::vector<unsigned char>>& is_noisy, \
		std::vector<PlaneSegOutputWithInten>& out_planes)
	{
		if (intensityVec.empty())
		{
			//log_warn("save_planes_with_insten input empty");
			return true;
		}

		if (!PlaneSegOutputConvert(input_points, plane_seg_out, intensityVec, out_planes))
		{
			//log_error("PlaneSegOutputConvert return error");
			return false;
		}

		is_noisy.clear();
		is_noisy.resize(out_planes.size());
		for (int i = 0; i < out_planes.size(); i++)
		{
			PlaneSegOutputWithInten* plane_it = &out_planes[i];
			get_plane_line_close_points(output_path, i, input_points, plane_it, planes_lines[i], sub_line_dist, is_noisy[i]);
		}
		return true;
	}

	static bool get_single_plane_noisy(
		const std::string output_path, 
		const std::vector<Point3f>& input_points, Point3f plane_center, Point3f plane_normal, 
		std::vector<Line3DSegOutItemDebug>& planes_lines, 
		const float sub_line_dist, 
		const std::vector<unsigned char>& intensityVec,
		std::vector<unsigned char>& is_noisy, 
		PlaneSegOutputWithInten& out_planes)
	{
		if (intensityVec.empty())
		{
			//log_warn("save_planes_with_insten input empty");
			return true;
		}
		if (!SinglePlaneSegOutputConvert(input_points, plane_center, plane_normal, intensityVec, out_planes))
		{
			//log_error("PlaneSegOutputConvert return error");
			return false;
		}
		PlaneSegOutputWithInten* plane_it = &out_planes;
		get_plane_line_close_points(output_path, 0, input_points, plane_it, planes_lines, sub_line_dist, is_noisy);
		return true;
	}

	static inline bool save_planes_with_noisy(const std::string output_path, std::vector<PlaneSegOutputWithInten>& plane_seg_out, \
		std::vector<std::vector<unsigned char>>& is_noisy)
	{
		if (plane_seg_out.empty() || (is_noisy.empty()) || output_path.empty())
		{
			//log_error("save_planes_with_noisy input empty");
			return false;
		}

		if (plane_seg_out.size() != is_noisy.size())
		{
			//log_error("save_planes_with_noisy input error plane_seg_out.size() =%d is_noisy.size() =%d", plane_seg_out.size(), is_noisy.size());
			return false;
		}

		std::string sub_path = output_path + "plane_xyzi\\";
		//first save non-noisy points
		if (IOData::createDirectory(sub_path) != 0)
		{
			std::cout << "can not create path: " << sub_path << std::endl;
			return false;
		}
		std::string delimiter = " ";

		for (unsigned int i = 0; i < plane_seg_out.size(); i++)
		{
			PlaneSegOutputWithInten* plane = &plane_seg_out[i];
			std::stringstream plane_id;
			plane_id << i;
			std::string whole_path = sub_path + "plane_xyzi_" + plane_id.str() + ".xyz";
			ofstream fout(whole_path);
			if (fout.fail()) {
				//log_error("save_oneplane_lines_xyzi cannot open %s", whole_path.c_str());
				return false;
			}

			for (unsigned int j = 0; j < plane->sPoints.size(); j++) {

				if (is_noisy[i][j] == 1) continue;
				sPointInfo point = plane->sPoints[j];
				unsigned int idx = plane->point_ids[j];
				fout << point.x << delimiter << point.y << delimiter << point.z << delimiter << static_cast<int>(point.i) << delimiter << idx;
				if (j < plane->sPoints.size() - 1) {
					fout << std::endl;
				}
			}
			fout.close();

			//second save noisy points in other path
			std::string noisy_whole_path = sub_path + "plane_noisy_xyzi_" + plane_id.str() + ".xyz";
			ofstream noisy_fout(noisy_whole_path);
			if (noisy_fout.fail()) {
				//log_error("save_plane_xyzi_with_noisy cannot open %s", noisy_whole_path.c_str());
				return false;
			}

			for (unsigned int j = 0; j < plane->sPoints.size(); j++) {
				if (is_noisy[i][j] == 0) continue;
				sPointInfo point = plane->sPoints[j];
				unsigned int idx = plane->point_ids[j];
				noisy_fout << point.x << delimiter << point.y << delimiter << point.z << delimiter << static_cast<int>(point.i) << delimiter << idx;
				if (j < plane->sPoints.size() - 1) {
					noisy_fout << std::endl;
				}
			}
			noisy_fout.close();
		}
		return true;
	}

	static inline bool save_single_plane_with_noisy(const std::string output_path, PlaneSegOutputWithInten& plane_seg_out, \
		std::vector<unsigned char>& is_noisy, int measure_index)
	{

		std::string sub_path = output_path + "plane_xyzi\\";
		//first save non-noisy points
		if (IOData::createDirectory(sub_path) != 0)
		{
			std::cout << "can not create path: " << sub_path << std::endl;
			return false;
		}
		std::string delimiter = " ";

		PlaneSegOutputWithInten* plane = &plane_seg_out;
		std::stringstream plane_id;
		plane_id << measure_index;
		std::string whole_path = sub_path + "plane_xyzi_" + plane_id.str() + ".txt";
		ofstream fout(whole_path);
		if (fout.fail()) {
			//log_error("save_oneplane_lines_xyzi cannot open %s", whole_path.c_str());
			return false;
		}

		for (unsigned int j = 0; j < plane->sPoints.size(); j++) {

			if (is_noisy[j] == 1) continue;
			sPointInfo point = plane->sPoints[j];
			unsigned int idx = plane->point_ids[j];
			fout << point.x << delimiter << point.y << delimiter << point.z << delimiter << static_cast<int>(point.i) << delimiter << idx;
			if (j < plane->sPoints.size() - 1) {
				fout << std::endl;
			}
		}
		fout.close();

		//second save noisy points in other path
		std::string noisy_whole_path = sub_path + "plane_noisy_xyzi_" + plane_id.str() + ".txt";
		ofstream noisy_fout(noisy_whole_path);
		if (noisy_fout.fail()) {
			//log_error("save_plane_xyzi_with_noisy cannot open %s", noisy_whole_path.c_str());
			return false;
		}

		for (unsigned int j = 0; j < plane->sPoints.size(); j++) {
			if (is_noisy[j] == 0) continue;
			sPointInfo point = plane->sPoints[j];
			unsigned int idx = plane->point_ids[j];
			noisy_fout << point.x << delimiter << point.y << delimiter << point.z << delimiter << static_cast<int>(point.i) << delimiter << idx;
			if (j < plane->sPoints.size() - 1) {
				noisy_fout << std::endl;
			}
		}
		noisy_fout.close();
		return true;
	}

}


#endif /*_UTIL_UNRE_HPP_*/
