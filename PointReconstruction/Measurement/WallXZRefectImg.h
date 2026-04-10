#pragma once

#include <opencv2/core/core.hpp> 
#include <Eigen/Dense>
class CWallXZRefectImg
{
public:
	CWallXZRefectImg();

	~CWallXZRefectImg();

	bool GetXzPlane(
		const std::vector<cv::Point3f>&ori_plane,
		const cv::Point3f& plane_normal,
		const cv::Point3f& plane_center,
		std::vector<cv::Point3f>& xz_plane);

	bool GetReflectImgPixelTen(
		const std::vector<cv::Point3f>& xzCloudPoints,
		const std::vector<int> planeReflectValues,
		cv::Mat& reflectRasterImgTen);

	bool GetReflectImgPixelOne(
		const std::vector<cv::Point3f>& xzCloudPoints,
		const std::vector<int> planeReflectValues,
		cv::Mat& reflectRasterImgTen);

	bool FindXzPlaneMinMax(const std::vector<cv::Point3f>& xzcloudPoints, cv::Point3f& pt_min, cv::Point3f& pt_max);


private:

	bool GetXZPlaneAndMatHelper(
		const std::vector<cv::Point3f>&plane,
		const cv::Point3f& plane_normal,
		const cv::Point3f& plane_center,
		std::vector<cv::Point3f>& xz_plane,
		cv::Mat& ori2XzMat,
		cv::Mat& xz2OriMat);

	bool RasterXZImgWithReflectionCommHelper(
		const std::vector<cv::Point3f>& xzCloudPoints,
		const std::vector<int> planeReflectValues,
		const int& pixelHeight,
		const int& pixelWidth,
		cv::Mat& reflectRasterImg);

	bool CmptRotateXZPlaneMatHelper(const float* plane_normal, cv::Mat&ori2Xz, cv::Mat& xz2Ori);

	//common raster_XZ_img,difference pixelHeight
	bool RasterXZImgCommHelper(
		const std::vector<cv::Point3f>& cloudPoints,
		const int& pixelHeight,
		const int& pixelWidth,
		Eigen::MatrixXi& countMat,
		Eigen::ArrayXf& coordX, Eigen::ArrayXf& coordZ,
		cv::Mat &rstImg,
		float minGlobalX = FLT_MAX, float maxGlobalX = -FLT_MAX,
		float minGlobalZ = FLT_MAX, float maxGlobalZ = -FLT_MAX);

private:
	// raster image
	const int pixelHeight; // 1 pixel mean x mm 
	const int pixelWidth;

	//float minCloudPoints_X; // min x coordinate of cloud point
	//float minCloudPoints_Y; // min y coordinate of cloud point
	//float minCloudPoints_Z; // min z coordinate of cloud point
	//float maxCloudPoints_X;
	//float maxCloudPoints_Y;
	//float maxCloudPoints_Z;
	//float meanCloudPoints_Y; // mean y coordinate of cloud point
	//float meanCloudPoints_Z; // mean z coordinate of cloud point

	//float deltaCloudPointsY;

	//int sparseDistanceToCameraThreshold1; // merge pixel in 1cm if less than this value
	//int sparseDistanceToCameraThreshold2; // merge pixel in 3cm if less than this value

};
