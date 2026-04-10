#pragma once
#include <iostream>
#include <vector>
#include <opencv2\opencv.hpp>

class BuildRefect
{
public:
	BuildRefect();
	~BuildRefect();

	static void BuildRefectImageFuc(const std::vector<cv::Point3f>& points,
		const std::vector<unsigned char>& reflects,
		const cv::Point3f& pt_center,
		const cv::Point3f&pt_normal,
		cv::Mat & dstImg);

private:

};


