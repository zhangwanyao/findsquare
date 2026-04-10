#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

class CLookAt
{
public:
	CLookAt();
	~CLookAt();

	//normal and center  //defualt n=100
	static void GetLookAtMat(const cv::Point3f& plane_normal, const cv::Point3f& plane_center, const int& n, cv::Mat &lookAt);

	static void RotationPoints(const cv::Mat& defPoints, cv::Mat &finalPoints, const cv::Mat& lookAt);

	static void RotationPoints(const std::vector<cv::Point3f>& points, std::vector<cv::Point3f> &rpoints, const cv::Mat& lookAt);

	static void Rotation3dTo2dPoints(const std::vector<cv::Point3f>& points, std::vector<cv::Point2f> &rpoints, const cv::Mat& lookAt);

	static void RotationPointsPair(
		const std::vector<std::pair<float, cv::Point3f>>& vec_pair, 
		std::vector<std::pair<float, cv::Point3f>>& vec_pair_lookat, 
		const cv::Mat& lookAt);

private:

	//cv::Mat lookAt_inv;
	//cv::invert(lookAt, lookAt_inv, CV_SVD);
};

