#pragma once
#include <Eigen\Dense> 
#include "Mesh/mesher/ModuleStruct.hpp"
class concreteMesherDelauney {
public:
	using Point2f = cv::Point2f;
	using Point3f = cv::Point3f;
	concreteMesherDelauney() {};
	virtual ~concreteMesherDelauney() {};

	//void AutoTestPts(std::vector<std::wstring>& fileName);
	cv::Point3d getTriCentroid(const cv::Point3d& p0, const cv::Point3d& p1, const cv::Point3d& p2);
	double getTdAngle(cv::Point3d& wall_normal, cv::Point3d& ground_normal);
	void GetnormalBy3Pts(cv::Point3d& p1, cv::Point3d& p2, cv::Point3d& p3, cv::Point3d& normalByPlaneCormer);

	bool buildMesh(std::vector<cv::Point3f> inPointCloud, std::string plane_file_name);

	bool MeshPlane(std::vector<cv::Point3f>& plane_points, cv::Point3f plane_normal, const std::string plane_file_name);

	bool meshAllPlaneInFilePath(std::string planeFilePath);

	bool buildMesh(std::vector<ModuleStruct::Point3fArray>& edgePointsList,
		std::vector<std::vector<ModuleStruct::Point3fArray>>& holePointsList,
		Eigen::Matrix4d& rmMatrix, std::string plane_file_name);
private:

};

Eigen::Matrix4d projectionXY_(std::vector<cv::Point3f>& inPointCloud, std::vector<cv::Point3f>& outPointCloud);
Eigen::Matrix4d projectionXY_(std::vector<cv::Point3f>& inPointCloud, cv::Point3f inNormal, std::vector<cv::Point3f>& outPointCloud, cv::Point3f outNormal);