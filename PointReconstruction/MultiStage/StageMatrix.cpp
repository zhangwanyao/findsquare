#include "StageMatrix.h"

#include <fstream>
#include <iostream>
#include <CGAL/Simple_cartesian.h> 
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/read_points.h>
#include <CGAL/property_map.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include "CGAL/Polygonal_surface_reconstruction.h"
#include <iostream>
#include <algorithm>
#include <string>
#include <sstream>
#include "../log/log.h"
#include "jansson.h"
using namespace std;

StageMatrix::StageMatrix()
{
}


StageMatrix::~StageMatrix()
{
}

cv::Point3f StageMatrix::RotatePoints(cv::Point3f &ori, int matrix_id)
{
	cv::Point3f  pt;
	pt.x = ori.x / 1000.0f;
	pt.y = ori.y / 1000.0f;
	pt.z = ori.z / 1000.0f;

	cv::Mat rot_mat = GetMatrix(matrix_id);
	cv::Point3f c_pt = cv::Point3f(
		rot_mat.at<float>(0, 0) * pt.x + rot_mat.at<float>(0, 1) * pt.y + rot_mat.at<float>(0, 2) * pt.z,
		rot_mat.at<float>(1, 0) * pt.x + rot_mat.at<float>(1, 1) * pt.y + rot_mat.at<float>(1, 2) * pt.z,
		rot_mat.at<float>(2, 0) * pt.x + rot_mat.at<float>(2, 1) * pt.y + rot_mat.at<float>(2, 2) * pt.z);

	c_pt.x += rot_mat.at<float>(0, 3);
	c_pt.y += rot_mat.at<float>(1, 3);
	c_pt.z += rot_mat.at<float>(2, 3);

	pt.x = c_pt.x * 1000.0f;
	pt.y = c_pt.y * 1000.0f;
	pt.z = c_pt.z * 1000.0f;

	return pt;
}
bool StageMatrix::LoadMatrix(std::string matrix_filename)
{
	std::ifstream is(matrix_filename.c_str(), std::ifstream::in | std::ifstream::binary);
	
	matrixs.clear();
	if (!is.good())
	{
		return false;
	}


	int mini(1), maxi(-1);
	std::string s;

	std::string line;
	bool tex_found(false), norm_found(false);

	int curid = 0;
	while (getline(is, s))
	{
		if (s.empty())
			continue;
		std::cout << s << std::endl;
		std::cout << s.find_first_of("cloud:") << std::endl;
		if (s.find_first_of("cloud:") != -1)
		{
			curid = atoi(s.substr(s.find_first_of(':') + 1, -1).c_str());
			matrixs[curid] = cv::Mat(4, 4, CV_32FC1);
		}
		else if (s.find_first_of("raw") >= 0)
		{
			int mid = 0;

			std::string matrix;

			int linecount = 0;
			while (linecount<4 && getline(is, matrix))
			{
				if (matrix.empty())
					continue;
				
				istringstream iss(matrix);
				float a, b, c, d;

				iss >> a >> b >> c >>d;

				matrixs[curid].at<float>(mid, 0) = a;
				matrixs[curid].at<float>(mid, 1) = b;
				matrixs[curid].at<float>(mid, 2) = c;
				matrixs[curid].at<float>(mid, 3) = d;
				linecount++;
				mid++;
			}
		}
		s.clear();
		s = "";
	}

	return !is.bad();

} // namespace internal


int  StageMatrix::GetCounts()
{
	return mStationInfos.size();
}

cv::Mat StageMatrix::GetMatrix(int id)
{
	return mStationInfos[id].station_matrixs;
}

#include "../Common/mesh_tool.h"

// 功能：读取server_info_xxx.json中的stations和matrixs信息
bool StageMatrix::LoadStationInfo(std::string station_json)
{
	std::string path = station_json.substr(0, station_json.find("Scanning") + std::string("Scanning").length());
	//std::cout << path << std::endl;
	json_error_t error;
	json_t *root = json_load_file(station_json.c_str(), 0, &error);
	if (!root)
	{
		json_decref(root);
		return false;
	}
	json_t* stations = json_object_get(root, "stations");
	if (!json_is_array(stations))
	{
		json_decref(root);
		return false;
	}

	int stations_size = json_array_size(stations);

	std::string file_dir = json_string_value(json_object_get(json_object_get(root, "mergeData"), "fileDir"));
	mergeDataDir = json_number_value(json_object_get(json_object_get(root, "mergeData"), "dire"));
	//log_info("mergeDataDir is: %f", mergeDataDir);

	for (int i = 0; i < stations_size; i++)
	{
		json_t* stations_info = json_array_get(stations, i);

		std::string station_name = json_string_value(json_object_get(stations_info, "name"));
		unsigned long long station_id = json_number_value(json_object_get(stations_info, "id"));
		std::string station_filename = json_string_value(json_object_get(stations_info, "fileDir"));

		//station_filename = path + station_filename.substr(file_dir.length()-1);

		char isSlash = station_filename.substr(file_dir.length())[0];

		if (isSlash != '/')
		{
			station_filename = path + station_filename.substr(file_dir.length() - 1);
		}
		else
		{
			station_filename = path + station_filename.substr(file_dir.length());
		}

		//std::string sub_filename = station_filename.substr(station_filename.find("Scanning") + std::string("Scanning").length(),-1);
		//std::cout << sub_filename << std::endl;
		//station_filename = path + sub_filename;

		float station_dire =  json_integer_value(json_object_get(stations_info, "dire"));


		//std::cout << station_name.substr(station_name.find("Station") + 7) << std::endl;
	 	int id= atoi(station_name.substr(station_name.find("Station")+7).c_str());
		
		json_t* pos_matrix = json_object_get(stations_info, "position");
		int pos_size = json_array_size(pos_matrix);
		

		StationInfo station;
		station.pts_id = station_id;
		station.station_id = id;
		station.pts_filename = station_filename;
		station.station_name = station_name;
		for (int j = 0; j < pos_size; j++)
		{
			station.position.push_back(json_number_value(json_array_get(pos_matrix, j)));
		}
		//cout << "station.position: " << station.position[0] <<" "<< station.position[1] << endl;
		mStationInfos[id]=station;
	}

	std::string version = json_string_value(json_object_get(root, "version"));
	if (version.find("sub") != std::string::npos) {
		log_info("version: %s", version.c_str());
		//std::cout << "version: " << version << std::endl;
		json_t* ref_rt_matrix = json_object_get(json_object_get(root, "ref_rt"), "matrixs");
		int matrix_value_size = json_array_size(ref_rt_matrix);
		cv::Mat ref_rt_m = cv::Mat(4, 4, CV_32FC1);
		for (int j = 0; j < matrix_value_size; j++)
		{
			ref_rt_m.at<float>(j / 4, j % 4) = json_number_value(json_array_get(ref_rt_matrix, j));
		}
		m_ref_matrix = ref_rt_m;
	}
	else {
		m_ref_matrix = cv::Mat::eye(4, 4, CV_32FC1);
	}

	json_t* rt_matrix = json_object_get(json_object_get(root, "mergeData"), "matrixs");
	if (!json_is_array(rt_matrix))
	{
		json_decref(root);
		return false;
	}

	int rt_matrix_size = json_array_size(rt_matrix);
	for (int i = 0; i < rt_matrix_size; i++)
	{
		json_t* rt_info = json_array_get(rt_matrix, i);
		unsigned long long pts_id = json_number_value(json_object_get(rt_info, "id"));
		json_t* matrix_value = json_object_get(rt_info, "matrix");

		int matrix_value_size = json_array_size(matrix_value);

		cv::Mat rt_m = cv::Mat(4, 4, CV_32FC1);
		for (int j = 0; j < matrix_value_size; j++)
		{
			rt_m.at<float>(j / 4, j % 4) = json_number_value(json_array_get(matrix_value, j));
		}
		
		//find id
		for (auto &state : mStationInfos)
		{
			if (pts_id == state.second.pts_id)
			{
				state.second.station_matrixs = m_ref_matrix * rt_m;
				break;
			}
		}
	}

	json_decref(root);

	return true;
}

std::map<int, StationInfo>   StageMatrix::GetStationInfos()
{
	return mStationInfos;
}


cv::Mat StageMatrix::GetRefMatrix()
{
	return m_ref_matrix;
}

float StageMatrix::GetmergeDataDir()
{
	return mergeDataDir;
}

void StageMatrix::SetStationInfos(std::map<int, StationInfo> infos)
{
	mStationInfos = infos;
}