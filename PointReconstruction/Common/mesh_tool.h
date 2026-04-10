#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

class CMeshTool
{
public:
	CMeshTool();
	~CMeshTool();

	bool SaveObjData(const int& meshId,
		const std::string& fileContent,
		const std::vector<cv::Point3f>& points,
		const std::vector<cv::Vec3i>& face,
		const std::vector<cv::Vec2f>& vt);

	bool SaveObjData(std::string mtl,
		const std::string& fileContent,
		const std::vector<cv::Point3f>& points,
		const std::vector<cv::Vec3i>& face,
		const std::vector<cv::Vec2f>& vt);

	bool GetObjData(const std::string& fileContent, std::vector<cv::Point3f> &points, std::vector<cv::Vec3i> &face, std::string* mtl = nullptr, std::vector<cv::Vec2f>* vt = nullptr);


	void SaveMtlfile(const int& meshId, std::string& mesh_str, const std::string& mtlPath);

	bool SaveObjDataNew(
		const std::string& fileContent,
		const std::string& mtllib_str,
		const std::vector<cv::Point3f>& points,
		const std::vector<cv::Vec3i>& face,
		const std::vector<cv::Vec2f>& vt);

	void SaveMtlfileNew(const std::string& mtlPath, const std::string& mapkd_str);

private:


	float stringToFloat(const std::string& str);

};

