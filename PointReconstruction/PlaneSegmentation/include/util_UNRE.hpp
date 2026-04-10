#ifndef _UTIL_UNRE_HPP_
#define _UTIL_UNRE_HPP_

#include <vector>
#include <string>
#include <opencv2/core.hpp>
#include "plane_seg_inf.h"
#include "ModuleStruct.hpp"
using namespace ModuleStruct;
//#include "sDef.h"
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

	//extern "C" bool UNRE_DLL_IMPORT WriteEncryFile(Vector<sPointInfo> * vecPtInfos, std::string strPath);
	//extern "C" bool UNRE_DLL_IMPORT ReadEncryFile(std::vector<sPointInfo> * vecPtInfos, std::string strPath);
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
	};

	static bool ReadEncryFile(vector<sPointInfo>* vecPtInfos, string strPath)
	{

		ifstream inFile(strPath, ios::in | ios::binary); //二进制读方式打开
		int nCount = 0;
		inFile.read((char*)&nCount, sizeof(int));
		char nSeed = nCount % 255;
		int count = nCount * sizeof(sPointInfo);
		char * buf = new char[count];
		inFile.read(buf, count);
		inFile.close();
		for (int n = 0; n < count; n++)
		{
			buf[n] = buf[n] ^ nSeed;
		}
		sPointInfo * t = (sPointInfo *)buf;
		vecPtInfos->insert(vecPtInfos->begin(), t, t + nCount);
		delete buf;
		return true;
	}

	inline static bool ReadEncryptionData(std::string path, std::vector<cv::Point3f>& data)
	{
		std::vector<sPointInfo> encryptData;
		bool rtn = ReadEncryFile(&encryptData, path);
		if (!rtn)
		{
			log_error("ReadEncryFile error path =  %s", path.c_str());
			return false;
		}
		data.resize(encryptData.size());
		for (uint i = 0; i < encryptData.size(); i++)
		{
			data[i].x = encryptData[i].x, data[i].y = encryptData[i].y, data[i].z = encryptData[i].z;
		}
		return true;
	}


	inline static bool ReadEncryptionDataWithIntensity(std::string path, std::vector<cv::Point3f>& data, std::vector<unsigned char>&intensityVec)
	{
		std::vector<sPointInfo> encryptData;
		bool rtn = ReadEncryFile(&encryptData, path);
		if (!rtn)
		{
			log_error("ReadEncryFile error path =  %s", path.c_str());
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
			cout << "ReadDirection open error, file name: " << dire_file <<endl;
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
		 log_info("GetDirectionAngle dire_file =%s", dire_file.c_str());
		 if (!ReadDirection(dire_file,",", rot_angle))
		 {
			 log_error("ReadDirection file %s error", dire_file.c_str());
			 return false;
		 }
		 log_info("rot angle =%f", rot_angle);
		 return true;	    
	}
	
	static bool SyncDownsampleIntensity(std::vector<unsigned int> ds_point_to_centriod, std::vector<unsigned char>&intensityVec,\
		std::vector<unsigned char>& ds_intensityVec)
	{
		//check input
		if (ds_point_to_centriod.empty() || intensityVec.empty() || ds_intensityVec.empty() || ds_point_to_centriod.size() != intensityVec.size())
		{
			log_error("input error ds_point_to_centriod size %d intensityVec size%d ds_intensityVec size%d", ds_point_to_centriod.size(), intensityVec.size(), ds_intensityVec.size());
			return false;
		}
#pragma omp parallel for
		for (int i = 0; i < ds_point_to_centriod.size(); i++)
		{
			unsigned int c_idx = ds_point_to_centriod[i];
			ds_intensityVec[c_idx] = intensityVec[i];
		}

	}

	static bool PlaneSegOutputConvert(const std::vector<Point3f>&input_data, std::vector<PlaneSegOutput>& seg_planes, \
		const std::vector<unsigned char>&intensityVec, std::vector<PlaneSegOutputWithInten>& out_planes)
	{
	    //check input
		if ((input_data.empty())||(seg_planes.size() == 0)||(input_data.size() != intensityVec.size()))
		{
			log_info("input error seg_planes size =%d intensityVec size =%d input_data size =%d", seg_planes.size(), intensityVec.size(), input_data.size());
			return false;
		}

		out_planes.clear();
		out_planes.resize(seg_planes.size());
//#pragma omp parallel for
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
		}
		return true;
	}


	static bool save_oneplane_xyzi(std::string path_name, std::string delimiter, PlaneSegOutputWithInten &plane)
	{
		if (path_name.empty()||(plane.sPoints.empty())) {
			log_error("save_plane_xyzi input error ");
			return false;
		}

		ofstream fout(path_name);
		if (fout.fail()) {
			log_error("save_plane_xyzi cannot open %s", path_name.c_str());
			return false;
		}

		ModuleStruct::Point3f point;
		for (unsigned int i = 0; i < plane.sPoints.size(); i++) {

			sPointInfo point = plane.sPoints[i];
			unsigned int idx = plane.point_ids[i];
			fout << point.x << delimiter << point.y << delimiter << point.z << delimiter << static_cast<int>(point.i)<< delimiter<< idx;
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
			log_error(" save_plane_xyzi createDirectory %s failed", sub_path.c_str());
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
				log_error("save_plane_xyzi %s plane %d return error", whole_path.c_str(), i);
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
			log_warn("save_planes_with_insten input empty");
			return true;
		}
		log_info("save_planes_with_insten 00");

		std::vector<PlaneSegOutputWithInten>out_planes;
		if(!PlaneSegOutputConvert(input_points,plane_seg_out,intensityVec,out_planes))
		{
			log_error("PlaneSegOutputConvert return error");
			return false;
		}

		log_info("save_planes_with_insten 01 out_planes size =%d", out_planes.size());
		std::string sub_path = output_path + "plane_xyzi\\";
		int rtn = save_plane_xyzi(sub_path, "plane_xyzi", ";", ".txt", input_points, out_planes);
		if (rtn)
		{
			log_error("save_plane_xyzi return error");
			return false;
		}
		log_info("save_planes_with_insten 02");
		return true;
	}

	static bool save_input_xyzi(const std::string path_name, const std::string delimiter,const std::vector<Point3f>&input_data, std::vector<unsigned char>&input_intensity)
	{
		//check input
		if ((input_data.empty()) || (input_intensity.empty()) || (input_data.size() != input_intensity.size()))
		{
			log_info("input input_intensity size =%d input_data size =%d", input_intensity.size(), input_data.size());
			return false;
		}

		ofstream fout(path_name);
		if (fout.fail()) {
			log_error("save_plane_xyzi cannot open %s", path_name.c_str());
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


}


#endif /*_UTIL_UNRE_HPP_*/
