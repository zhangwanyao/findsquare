#pragma once
#include <vector>
#include <opencv2/core/core.hpp> 
#include <Eigen/Dense>

class CPlaneXyImg
{
public:
	CPlaneXyImg();
	~CPlaneXyImg();

	bool CheckOneGroundAxisLine(
		const int& ground_id,
		const cv::Point3f& inner_normal,
		const cv::Point3f& ground_center,
		const std::vector<cv::Point3f>& ground_plane,
		const std::vector<int>& planeReflectValues,
		std::vector<cv::Point3f>& circle_center_center3d);

	bool CheckAxisLineHelper(
		const int& wall_id,
		const std::vector<cv::Point3f>& xyCloudPoints,
		const std::vector<int> planeReflectValues,
		std::vector<cv::Point3f>& vec_circle_center3d);

	bool GetPlaneXYRefectImgTen(const std::vector<cv::Point3f>& xyCloudPoints,const std::vector<int> planeReflectValues,cv::Mat& imgTen);

	bool GetPlaneXYRefectImgOne(const std::vector<cv::Point3f>& xyCloudPoints, const std::vector<int> planeReflectValues, cv::Mat& imgOne);

	bool PlaneXYRefectImgComm(const std::vector<cv::Point3f>& xyCloudPoints,
		const std::vector<int> planeReflectValues,
		const int& pixelHeight,
		const int& pixelWidth,
		cv::Mat& reflectImg);

	bool CheckCircle(const int& wall_id,const cv::Mat& imgTen);


private:
	bool IsImgTenExistCircle(const int& wall_id, const cv::Mat& reflectImgTen, std::vector<cv::Vec3f>& cir_info);

	void GetRefineMoveDistance(float& radius, float& move_distance);

	float AbsMinPointTowDistance(const cv::Point& pt1, const cv::Point& pt2);
	bool SelectFitTwoCenter(const std::vector<cv::Point>&circle_center2d, std::vector<cv::Point>&circle_center2d_filter);



};
