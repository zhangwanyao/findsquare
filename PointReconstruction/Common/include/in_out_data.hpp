#ifndef _IN_OUT_DATA_H_
#define _IN_OUT_DATA_H_
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include "InOutData.h"
#include "DataStruct.h"
#include "ModuleStruct.hpp"
#include "log.h"

#ifndef FSCANF
#if defined WIN32 || defined _WIN32 || defined __CYGWIN__
#define FSCANF fscanf_s
#else
#define FSCANF fscanf
#endif
#endif

class FeaturesIO {
public:

	static inline bool ParseLinePoint3f(const std::string line, const std::string delim, ModuleStruct::Point3f& pointData)
	{
		if (line.empty()) return false;
		std::string sub_strings[4];
		int prev_pos = 0, cur_pos = 0;
		int cnt = 0;
		do
		{
			cur_pos = static_cast<int>(line.find(delim, prev_pos));
			if (cur_pos == std::string::npos) cur_pos = static_cast<int>(line.length());
			std::string tmp_substring = line.substr(prev_pos, cur_pos - prev_pos);
			if (!tmp_substring.empty())
			{
				sub_strings[cnt].assign(tmp_substring);
			}
			prev_pos = cur_pos + static_cast<int>(delim.length());
			cnt++;
		} while (cur_pos < line.length() && prev_pos < line.length() && (cnt < 4));

		if (cnt == 3)
		{
			pointData.x = strtof(sub_strings[0].c_str(), NULL);
			pointData.y = strtof(sub_strings[1].c_str(), NULL);
			pointData.z = strtof(sub_strings[2].c_str(), NULL);
			return true;
		}
		else
		{
			return false;
		}
	}

	static inline bool SavePoint3fData(const std::string path_name, const PointArray& point_array, const PointInVoxelArray& point_in_voxel_array)
	{
		if (point_array.size == 0) {
			std::cout << "IOData::SavePoint3fData: empty point_array" << std::endl;
			return false;
		}

		if (point_array.points == NULL) {
			std::cout << "IOData::SavePoint3fData: point_array pointer is NULL" << std::endl;
			return false;
		}

		if (point_in_voxel_array.size == 0) {
			std::cout << "IOData::SavePoint3fData: empty point_in_voxel_array" << std::endl;
			return false;
		}

		if (point_in_voxel_array.point_idx == NULL) {
			std::cout << "IOData::SavePoint3fData: point_in_voxel_array pointer is NULL" << std::endl;
			return false;
		}

		if (path_name.empty()) {
			std::cout << "IOData::SavePoint3fData: empty file path" << std::endl;
			return false;
		}

		std::ofstream fout(path_name);
		if (fout.fail()) {
			log_fatal("cannot open %s", path_name.c_str());
			return false;
		}

		unsigned int point_idx;
		ModuleStruct::Point3f* point;
		for (unsigned int i = 0; i < point_in_voxel_array.size; i++) {
			point_idx = point_in_voxel_array.point_idx[i];
			point = &point_array.points[point_idx];
			fout << point->x << ';' << point->y << ';' << point->z;
			if (i < point_in_voxel_array.size - 1) {
				fout << std::endl;
			}
		}
		fout.close();
		return true;
	}
	static inline bool SavePoint3fData(const std::string path_name, const PointArray& point_array, const ModuleStruct::Vector<unsigned int> input_data)
	{
		if (point_array.size == 0) {
			std::cout << "IOData::SavePoint3fData: empty point_array" << std::endl;
			return false;
		}

		if (point_array.points == NULL) {
			std::cout << "IOData::SavePoint3fData: point_array pointer is NULL" << std::endl;
			return false;
		}

		if (input_data.empty()) {
			std::cout << "IOData::SavePoint3fData: empty point_in_voxel_array" << std::endl;
			return false;
		}

		if (path_name.empty()) {
			std::cout << "IOData::SavePoint3fData: empty file path" << std::endl;
			return false;
		}

		std::ofstream fout(path_name);
		if (fout.fail()) {
			log_fatal("cannot open %s", path_name.c_str());
			return false;
		}

		unsigned int point_idx;
		ModuleStruct::Point3f* point;
		for (unsigned int i = 0; i < input_data.size(); i++) {
			point_idx = input_data[i];
			point = &point_array.points[point_idx];
			fout << point->x << ';' << point->y << ';' << point->z;
			if (i < input_data.size() - 1) {
				fout << std::endl;
			}
		}
		fout.close();
		return true;
	}

	static inline bool Ft_Get3DPtCloudRowNum(const std::string path_name, int& row_num) {
		FILE* pFile;
#ifdef WIN32
		errno_t err = fopen_s(&pFile, path_name.c_str(), "r");
#else
		pFile = fopen(path_name.c_str(), "r");
#endif
		if (!pFile)
		{
			std::cout << "IOData::Load3DPtCloudData: cannot open file" << std::endl;
			return false;
		}
		row_num = 0;
		//std::string line;
		char line[256] = { 0 };
		while (!feof(pFile)) {
			if (fgets(line, sizeof(line), pFile))
				row_num++;

			/* revised by Tang Xueyan @08/11/2018 */
			//size_t pos = 0;
			//while ((pos = line.find(' ') || (pos = line.find('\n'))) != std::string::npos) {
			//	pos++;
			//}
			//if (pos++ == 3)
			//	row_num++;
		}
		fclose(pFile);

		return true;
	}

	//static inline bool SavePoint3fData(const std::string path_name, const PointArray &point_array, const std::vector<unsigned int> input_data);
	static inline bool SavePoint3fData(const std::string path_name, const PointArray& point_array)
	{
		if (point_array.size == 0) {
			std::cout << "IOData::SavePoint3fData: empty point_array" << std::endl;
			return false;
		}

		if (point_array.points == NULL) {
			std::cout << "IOData::SavePoint3fData: point_array pointer is NULL" << std::endl;
			return false;
		}

		if (path_name.empty()) {
			std::cout << "IOData::SavePoint3fData: empty file path" << std::endl;
			return false;
		}

		std::ofstream fout(path_name);
		if (fout.fail()) {
			log_fatal("cannot open %s", path_name.c_str());
			return false;
		}

		ModuleStruct::Point3f* point;
		for (unsigned int i = 0; i < point_array.size; i++) {
			point = &point_array.points[i];
			fout << point->x << ';' << point->y << ';' << point->z;
			if (i < point_array.size - 1) {
				fout << std::endl;
			}
		}
		fout.close();
		return true;
	}

	static inline bool SavePoint3fDataWithDelimiter(const std::string path_name, const std::string delimiter, const PointArray& point_array, const PointInVoxelArray& point_in_voxel_array)
	{
		if (point_array.size == 0) {
			std::cout << "IOData::SavePoint3fData: empty point_array" << std::endl;
			return false;
		}

		if (point_array.points == NULL) {
			std::cout << "IOData::SavePoint3fData: point_array pointer is NULL" << std::endl;
			return false;
		}

		if (point_in_voxel_array.size == 0) {
			std::cout << "IOData::SavePoint3fData: empty point_in_voxel_array" << std::endl;
			return false;
		}

		if (point_in_voxel_array.point_idx == NULL) {
			std::cout << "IOData::SavePoint3fData: point_in_voxel_array pointer is NULL" << std::endl;
			return false;
		}

		if (path_name.empty()) {
			std::cout << "IOData::SavePoint3fData: empty file path" << std::endl;
			return false;
		}

		std::ofstream fout(path_name);
		if (fout.fail()) {
			log_fatal("cannot open %s", path_name.c_str());
			return false;
		}

		unsigned int point_idx;
		ModuleStruct::Point3f* point;
		for (unsigned int i = 0; i < point_in_voxel_array.size; i++) {
			point_idx = point_in_voxel_array.point_idx[i];
			point = &point_array.points[point_idx];
			fout << point->x << delimiter << point->y << delimiter << point->z;
			if (i < point_in_voxel_array.size - 1) {
				fout << std::endl;
			}
		}
		fout.close();
		return true;
	}
	//load 3d point cloud from .txt file
	//path_name(in): path name + file name
	//pointclouddata(out): N x 3 raw data from .txt file
	static inline bool Load3DPtCloudData(const std::string path_name, PointArray& point_array)
	{
		FILE* pFile;
		pFile = fopen(path_name.c_str(), "r");

		if (!pFile)
		{
			std::cout << "IOData::Load3DPtCloudData: cannot open file" << std::endl;
			log_fatal("cannot open %s", path_name.c_str());
			return false;
		}

		int raw_data_row_num = 0;
		Ft_Get3DPtCloudRowNum(path_name, raw_data_row_num);
		bool is_pts_file = IOData::IdentifySuffix(path_name, ".pts");
		if (is_pts_file)
		{
			raw_data_row_num = raw_data_row_num - 1;
		}

		if (raw_data_row_num <= 0) {
			std::cout << "IOData::Load3DPtCloudData: Get3DPtCloudRowNum() error" << std::endl;
			log_fatal("NO row number at %s", path_name.c_str());
			return false;
		}

		point_array.size = raw_data_row_num;
		point_array.points = new ModuleStruct::Point3f[raw_data_row_num];
		int row_idx = 0;
		while (!feof(pFile) && row_idx < raw_data_row_num)
		{
			if ((row_idx == 0) && is_pts_file)
			{
				int first_line;
				FSCANF(pFile, "%d%*[^\n]%*c", &first_line);
			}

			if (FSCANF(pFile, "%f%f%f%*[^\n]%*c", &point_array.points[row_idx].x, &point_array.points[row_idx].y, &point_array.points[row_idx].z) == 3)
			{
				//point_array.points[row_idx].voxel_idx = std::numeric_limits<unsigned int>::max();
				row_idx++;
			}
		}
		fclose(pFile);
		return true;
	}

	static inline bool Load3DPtCloudDataWithDelimiter(const std::string path_name, std::string Delimter, PointArray& point_array)
	{
		std::string line;
		std::ifstream input_file(path_name.c_str());
		if (!input_file)
		{
			log_error("can not open file %s", path_name.c_str());
			return false;
		}

		int raw_data_row_num = 0;
		Ft_Get3DPtCloudRowNum(path_name, raw_data_row_num);
		bool is_pts_file = IOData::IdentifySuffix(path_name, ".pts");
		if (is_pts_file)
		{
			raw_data_row_num = raw_data_row_num - 1;
		}

		if (raw_data_row_num <= 0) {
			std::cout << "IOData::Load3DPtCloudData: Get3DPtCloudRowNum() error" << std::endl;
			log_fatal("NO row number at %s", path_name.c_str());
			return false;
		}

		point_array.size = raw_data_row_num;
		point_array.points = new ModuleStruct::Point3f[raw_data_row_num];
		int row_idx = 0;
		//while (!feof(pFile) && row_idx < raw_data_row_num)
		while (std::getline(input_file, line) && (row_idx < raw_data_row_num))
		{
			ModuleStruct::Point3f point;
			if (ParseLinePoint3f(line, Delimter, point))
			{
				//point_array.points[row_idx].voxel_idx = std::numeric_limits<unsigned int>::max();
				point_array.points[row_idx] = point;
				row_idx++;
			}
		}
		input_file.close();
		return true;
	}

	static inline bool Load3DPtCloudDataWithDelimiter(const std::string path_name, std::string Delimter, ModuleStruct::Point3fArray& point_array)
	{
		std::string line;
		std::ifstream input_file(path_name.c_str());
		if (!input_file)
		{
			log_error("can not open file %s", path_name.c_str());
			return false;
		}

		int raw_data_row_num = 0;
		Ft_Get3DPtCloudRowNum(path_name, raw_data_row_num);
		bool is_pts_file = IOData::IdentifySuffix(path_name, ".pts");
		if (is_pts_file)
		{
			raw_data_row_num = raw_data_row_num - 1;
		}

		if (raw_data_row_num <= 0) {
			std::cout << "IOData::Load3DPtCloudData: Get3DPtCloudRowNum() error" << std::endl;
			log_fatal("NO row number at %s", path_name.c_str());
			return false;
		}

		point_array.resize(raw_data_row_num);
		int row_idx = 0;
		//while (!feof(pFile) && row_idx < raw_data_row_num)
		while (std::getline(input_file, line) && (row_idx < raw_data_row_num))
		{
			ModuleStruct::Point3f point;
			if (ParseLinePoint3f(line, Delimter, point))
			{
				//point_array.points[row_idx].voxel_idx = std::numeric_limits<unsigned int>::max();
				point_array[row_idx] = point;
				row_idx++;
			}
		}
		input_file.close();
		if (row_idx == 0) return false;
		return true;
	}
	static inline bool Load3DPtCloudDataWithDelimiter(const std::string path_name, std::string Delimter, ModuleStruct::Point2fArray& pt_cloud_xyz) {
		std::string line;
		std::ifstream input_file(path_name.c_str());
		if (!input_file)
		{
			log_error("can not open file %s", path_name.c_str());
			return false;
		}

		int raw_data_row_num = 0;
		Ft_Get3DPtCloudRowNum(path_name, raw_data_row_num);
		bool is_pts_file = IOData::IdentifySuffix(path_name, ".pts");
		if (is_pts_file)
		{
			raw_data_row_num = raw_data_row_num - 1;
		}

		if (raw_data_row_num <= 0) {
			std::cout << "IOData::Load3DPtCloudData: Get3DPtCloudRowNum() error" << std::endl;
			log_fatal("NO row number at %s", path_name.c_str());
			return false;
		}
		pt_cloud_xyz.clear();
		pt_cloud_xyz.resize(raw_data_row_num);
		int row_idx = 0;
		//while (!feof(pFile) && row_idx < raw_data_row_num)
		while (std::getline(input_file, line) && (row_idx < raw_data_row_num))
		{
			ModuleStruct::Point3f point;
			if (ParseLinePoint3f(line, Delimter, point))
			{
				pt_cloud_xyz[row_idx].x = point.x;
				pt_cloud_xyz[row_idx].y = point.y;
				row_idx++;
			}
		}
		input_file.close();
		return true;
	}

	static inline bool SavePoint2fDataWithDelimiter(const std::string path_name, const std::string delimiter, const ModuleStruct::Point2fArray input_data)
	{
		if (input_data.size() == 0) {
			std::cout << "IOData::SavePoint2fDataWithDelimiter: empty point_in_voxel_array" << std::endl;
			return false;
		}

		if (path_name.empty()) {
			std::cout << "IOData::SavePoint2fDataWithDelimiter: empty file path" << std::endl;
			return false;
		}

		std::ofstream fout(path_name);

		if (fout.fail()) {
			log_fatal("cannot open %s", path_name.c_str());
			return false;
		}

		cv::Point2f point;
		for (unsigned int i = 0; i < input_data.size(); i++) {
			//point_idx = input_data[i];
			point = input_data[i];
			fout << point.x << delimiter << point.y << delimiter << 0.0f;
			if (i < input_data.size() - 1) {
				fout << std::endl;
			}
		}
		fout.close();
		return true;
	}

	static inline bool LoadNormalFromFile(const std::string path_name, ModuleStruct::Point3f& normal)
	{
		FILE* pFile;
		pFile = fopen(path_name.c_str(), "r");

		if (!pFile)
		{
			log_fatal("cannot open %s", path_name.c_str());
			return false;
		}

		int raw_data_row_num = 0;
		Ft_Get3DPtCloudRowNum(path_name, raw_data_row_num);
		if (raw_data_row_num != 1) {
			log_fatal("row number at %s is %d", path_name.c_str(), raw_data_row_num);
			return false;
		}

		if (FSCANF(pFile, "%f%f%f%*[^\n]%*c", &normal.x, &normal.y, &normal.z) == 3)
		{
			fclose(pFile);
			return true;
		}
		else
		{
			fclose(pFile);
			log_fatal("load %s failed", path_name.c_str());
			return false;
		}
	}

	static bool SavePoint3fDataNormalWithDelimiter(const std::string path_name, const std::string delimiter, const std::vector<ModuleStruct::Point3f> input_data, const std::vector<ModuleStruct::Point3f> input_normal) {
		if (input_data.size() == 0) {
			std::cout << "SavePoint3fDataNormalWithDelimiter: empty input_data" << std::endl;
			return false;
		}

		if (input_normal.size() == 0) {
			std::cout << "SavePoint3fDataNormalWithDelimiter: empty input_normal" << std::endl;
			return false;
		}

		if (input_data.size() != input_normal.size())
		{
			std::cout << "input error input_data.size():" << input_data.size() << " is not equal to input_normal.size() :" << input_normal.size() << std::endl;
			return false;
		}

		if (path_name.empty()) {
			std::cout << "SavePoint3fDataNormalWithDelimiter: empty file path" << std::endl;
			return false;
		}

		std::ofstream fout(path_name);

		if (fout.fail()) {
			log_fatal("cannot open %s", path_name.c_str());
			return false;
		}
		ModuleStruct::Point3f point;
		ModuleStruct::Point3f point_normal;
		fout << std::fixed << std::setprecision(16);
		for (unsigned int i = 0; i < input_data.size(); i++) {
			//point_idx = input_data[i];
			point = input_data[i];
			point_normal = input_normal[i];
			fout << point.x << delimiter << point.y << delimiter << point.z\
				<< delimiter << point_normal.x << delimiter << point_normal.y << delimiter << point_normal.z;
			if (i < input_data.size() - 1) {
				fout << std::endl;
			}
		}
		fout.close();
		return true;
	}
};

#endif // __IN_OUT_DATA_H_