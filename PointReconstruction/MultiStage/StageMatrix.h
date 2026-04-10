#pragma once

#include <string>
#include <map>
#include <opencv2\core.hpp>
using namespace std;


typedef struct StationInfo_t
{
	unsigned long long pts_id;
	int			station_id;
	std::string station_name;
	std::string pts_filename;
	float       station_dire;
	std::vector<float> position;
	cv::Mat     station_matrixs;
}StationInfo;

class StageMatrix
{
public:
	StageMatrix();
	~StageMatrix();

public:
	bool LoadStationInfo(std::string station_json);
	bool LoadMatrix(std::string matrix_filename);
	cv::Point3f RotatePoints(cv::Point3f &ori, int matrix_id);

	int  GetCounts();
	cv::Mat GetMatrix(int id);
	float GetmergeDataDir();

	std::map<int, StationInfo>   GetStationInfos();
	cv::Mat GetRefMatrix();

	void SetStationInfos(std::map<int, StationInfo> infos);

private:
	std::map<int, cv::Mat>  matrixs;
	cv::Mat m_ref_matrix;
	std::map<int, StationInfo> mStationInfos;
	float mergeDataDir;
};

