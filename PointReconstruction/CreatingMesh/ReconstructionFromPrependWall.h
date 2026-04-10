#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>

class CreatingMesh
{
public:

	void buildMesh(std::string jsonPath, float ratio, std::string output_path);

private:

	cv::Point3d coordinateTransformation(cv::Point3d &plane_vertices, cv::Point3d &plane_normal, double room_depth);

	void Appendsufix(std::string &filedir);
};