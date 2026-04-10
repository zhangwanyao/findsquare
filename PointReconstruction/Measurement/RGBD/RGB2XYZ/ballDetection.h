#if 1
#pragma once
#ifndef BALLDETECTION_H
#define BALLDETECTION_H

#include <vector>
#include <opencv2/core/core.hpp>
#include "net.h"

//#define YOLOFACE_INPUT_WIDTH  512
//#define YOLOFACE_INPUT_HEIGHT 288

#define YOLOFACE_INPUT_WIDTH  640
#define YOLOFACE_INPUT_HEIGHT 640
#define NUM_OUTPUTS 3
//#define NUM_KEYPOINTS 5

struct Point {
	float x;
	float y;
	float prob;
};

typedef struct Object
{
	float x1;
	float y1;
	float x2;
	float y2;
	float score;
	//Point landmark[NUM_KEYPOINTS];
} Object;


class BALLDETECTION
{
public:

	int init(const char* modeltype, int num_threads);

	int detect(const cv::Mat& rgb, std::vector<Object>& objects, float prob_threshold = 0.5f, float nms_threshold = 0.45f);

	std::vector<cv::Point2d> ballDetection(const cv::Mat& rgb);

	int destroy() { return 0; }

private:

	void decode(ncnn::Mat data, std::vector<int> anchor, std::vector<Object> &prebox, float threshold, int stride);
	std::vector<float>LetterboxImage(const cv::Mat& src, cv::Mat& dst, const cv::Size& out_size);

private:

	ncnn::Net yoloface;
	int num_threads = 4;

	float norm_vals[3] = { 1 / 255.f, 1 / 255.f, 1 / 255.f }; //·½²î
	std::vector<int> anchor0 = { 4,5,  6,8,  10,12 };
	std::vector<int> anchor1 = { 15,19,  23,30,  39,52 };
	std::vector<int> anchor2 = { 72,97,  123,164,  209,297 };
};

#endif // BALLDETECTION_H

#endif