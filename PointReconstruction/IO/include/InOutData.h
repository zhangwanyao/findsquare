#ifndef __INOUTDATA_H__
#define __INOUTDATA_H__

#include <iostream>
#include <fstream>
#include <string>
#include "ModuleStruct.hpp"

#include "sDef.h"

using namespace std;

#ifdef WIN32
#define FSCANF fscanf_s
#else
#define FSCANF fscanf
#endif
class IOData {
public:

	static bool SavePoint3fData(const string path_name, const ModuleStruct::Point3 &input_data);
	static bool SavePoint3fData(const string path_name, const ModuleStruct::Point3Array &input_data, const ModuleStruct::Point3Array &input_normal);
	static bool SavePoint3fData(const string path_name, const ModuleStruct::Point3Array &input_data);
	static bool SavePoint3fData(const string path_name, const ModuleStruct::Vector<ModuleStruct::Point3Array> &input_data);
	
	static bool SavePoint3fData(const std::string path_name, std::vector<cv::Point3f> &input_data, const std::vector<unsigned char> &input_reflect);

	static bool SavePoint3fDataWithDelimiter(const string path_name, const string delimiter, const ModuleStruct::Point3Array &point_array, const ModuleStruct::Vector<unsigned int> &point_in_voxel_array);
	static bool SavePoint3fDataWithDelimiter(const std::string path_name, const std::string delimiter, const ModuleStruct::Vector<ModuleStruct::Point3>& input_data);
	static int parse_argument(int argc, char** argv, const char* str, string &path);
	static int parse_argument(int argc, char** argv, const char* str, bool &is_include);

	static int createDirectory(string Dir);
	
	static wstring ReadUTF8FileString(string &filename);
	static wstring ReadUTF8FileWString(wstring &filename);

	//save point3d data
	//path_name(in): path name + file name
	//input_data(in): N x Point3f input data
	static bool SavePoint3dData(const string path_name, const ModuleStruct::Point3dArray &input_data);

	//get number of rows of 3d point cloud (N x 3)
	//path_name(in): path name + file name
	//rownum(out): N x 3 raw data from .txt file
	static bool Get3DPtCloudRowNum(const string path_name, int &row_num);

	//load 3d point cloud from .txt file
	//path_name(in): path name + file name
	//pointclouddata(out): N x 3 raw data from .txt file
	static bool Load3DPtCloudData(const string path_name, ModuleStruct::CMat &pt_cloud_xyz);

	//load 3d point cloud from .txt file
	//path_name(in): path name + file name
	//pointclouddata(out): N x 3 raw data from .txt file
	static bool Load3DPtCloudData(const string path_name, ModuleStruct::Point3Array &pt_cloud_xyz);

	static bool Load3DPtCloudData(const std::string path_name, std::vector<cv::Point3f>& pt_cloud_xyz, std::vector<int>& input_reflect, bool is_cad = true);
	
	//load 3d point cloud from .txt file
	//path_name(in): path name + file name
	//pointclouddata(out): N x 3 raw data from .txt file
	static bool Load3DPtCloudDataWithDelimiter(const string file_name, const string delimiter, ModuleStruct::CMat &pt_cloud_xyz);

	//load 3d point cloud with normals from .txt file
	//path_name(in): path name + file name
	//pointclouddata(out): N x 6 raw data from .txt file
	static bool Load3DPtCloudDataWithNormal(const string path_name, ModuleStruct::Point3Array &input_points, ModuleStruct::Point3Array &input_normals);

	//write error log
	static bool WriteErrorLog(const string err_msg);

	//load grid to occupied voxel index array from .txt file
	static bool LoadGridToOccupiedIdx(const string path_name, unsigned int* &grid_to_occupied_voxel_idx, unsigned int &grid_to_occupied_voxel_idx_size);

	// filter noise data of input data
	static bool FilterInputData(ModuleStruct::Point3Array& scene_xyz, ModuleStruct::Point3Array &filtered_scene_xyz);
	static bool IdentifySuffix(string filePath, const string suffix);
	
	static bool SavePlyFile(const string path_name, const ModuleStruct::Point3Array input_points, ModuleStruct::Point3Array input_normals);
	
	// load line segment from .txt file
	static bool LoadLineSegment(const string path_name, ModuleStruct::Vector<ModuleStruct::Line3> &lines);

	static bool LoadPLYPoints3f(const std::string path_name, std::vector<cv::Point3f> &pt_cloud_xyz);
	static bool SavePLYPoints3f(const std::string path_name, const std::vector<cv::Point3f> &input_data, bool isSaveBinary = false);

	static std::vector<cv::Point3f> readFromXyz(const std::string& path, const bool& useEncryptData = true, std::vector<float>* intensity = nullptr);
private:
	//load 3d point cloud from .ply file
	//path_name(in): path name + file name
	//pointclouddata(out): N x 3 raw data from .ply file
	//Ascii
	static bool LoadPLYPoints3fAsc(const std::string path_name, std::vector<cv::Point3f> &pt_cloud_xyz);
	//load 3d point cloud from .ply file
	//path_name(in): path name + file name
	//pointclouddata(out): N x 3 raw data from .ply file
	//Binary 
	static bool LoadPLYPoints3fBin(const std::string path_name, std::vector<cv::Point3f> &pt_cloud_xyz);
	//save point3f data
	//path_name(in): path name + file name
	//input_data(in): N x cv::point3f input data	
	//save as Ascii 
	static bool SavePLYPoints3fAsc(const std::string path_name, const std::vector<cv::Point3f> &input_data);
	//save point3f data
	//path_name(in): path name + file name
	//input_data(in): N x cv::point3f input data	
	//save as Binary 
	static bool SavePLYPoints3fBin(const std::string path_name, const std::vector<cv::Point3f> &input_data);
};

#endif // __INOUTDATA_H__
