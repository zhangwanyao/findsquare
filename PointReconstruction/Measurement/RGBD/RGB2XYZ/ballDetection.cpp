#if 1
#include "ballDetection.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>


int BALLDETECTION::init(const char* modeltype, int num_threads)
{
	char parampath[256];
	char modelpath[256];
	sprintf_s(parampath, "%s.param", modeltype);
	sprintf_s(modelpath, "%s.bin", modeltype);

	this->num_threads = num_threads;
	yoloface.load_param(parampath);
	yoloface.load_model(modelpath);

	return 0;
}

bool cmp(Object b1, Object b2) {
	return b1.score > b2.score;
}

static inline float sigmoid(float x) {
	return static_cast<float>(1.f / (1.f + exp(-x)));
}

void nms(std::vector<Object> &input_boxes, float NMS_THRESH)
{
	std::vector<float>vArea(input_boxes.size());
	for (int i = 0; i < int(input_boxes.size()); ++i)
	{
		vArea[i] = (input_boxes.at(i).x2 - input_boxes.at(i).x1 + 1)
			* (input_boxes.at(i).y2 - input_boxes.at(i).y1 + 1);
	}
	for (int i = 0; i < int(input_boxes.size()); ++i)
	{
		for (int j = i + 1; j < int(input_boxes.size());)
		{
			float xx1 = max(input_boxes[i].x1, input_boxes[j].x1);
			float yy1 = max(input_boxes[i].y1, input_boxes[j].y1);
			float xx2 = min(input_boxes[i].x2, input_boxes[j].x2);
			float yy2 = min(input_boxes[i].y2, input_boxes[j].y2);
			float w = max(float(0), xx2 - xx1 + 1);
			float h = max(float(0), yy2 - yy1 + 1);
			float inter = w * h;
			float ovr = inter / (vArea[i] + vArea[j] - inter);
			if (ovr >= NMS_THRESH)
			{
				input_boxes.erase(input_boxes.begin() + j);
				vArea.erase(vArea.begin() + j);
			}
			else
			{
				j++;
			}
		}
	}
}

std::vector<float> BALLDETECTION::LetterboxImage(const cv::Mat& src, cv::Mat& dst, const cv::Size& out_size)
{
	auto in_h = static_cast<float>(src.rows);
	auto in_w = static_cast<float>(src.cols);
	float out_h = out_size.height;
	float out_w = out_size.width;

	float scale = min(out_w / in_w, out_h / in_h);

	int mid_h = static_cast<int>(in_h * scale);
	int mid_w = static_cast<int>(in_w * scale);

	cv::resize(src, dst, cv::Size(mid_w, mid_h), 0, 0, cv::INTER_NEAREST);

	int top = (static_cast<int>(out_h) - mid_h) / 2;
	int down = (static_cast<int>(out_h) - mid_h + 1) / 2;
	int left = (static_cast<int>(out_w) - mid_w) / 2;
	int right = (static_cast<int>(out_w) - mid_w + 1) / 2;

	cv::copyMakeBorder(dst, dst, top, down, left, right, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

	std::vector<float> pad_info{ static_cast<float>(left), static_cast<float>(top), scale };
	return pad_info;
}

void BALLDETECTION::decode(ncnn::Mat data, std::vector<int> anchor, std::vector<Object> &prebox, float threshold, int stride)
{
	int fea_h = data.h;
	int fea_w = data.w;
	int spacial_size = fea_w * fea_h;
	int channels = 6; //3 x 5 + 6 = 21,点用三个值描述，框用六个值描述，总共是五个点一个框
	//int channels = NUM_KEYPOINTS * 3 + 6; //3 x 5 + 6 = 21,点用三个值描述，框用六个值描述，总共是五个点一个框
	float *ptr = (float*)(data.data);
	for (int c = 0; c < anchor.size() / 2; c++)
	{
		float anchor_w = float(anchor[c * 2 + 0]);
		float anchor_h = float(anchor[c * 2 + 1]);
		float *ptr_x = ptr + spacial_size * (c * channels + 0);
		float *ptr_y = ptr + spacial_size * (c * channels + 1);
		float *ptr_w = ptr + spacial_size * (c * channels + 2);
		float *ptr_h = ptr + spacial_size * (c * channels + 3);
		float *ptr_s = ptr + spacial_size * (c * channels + 4);
		float *ptr_c = ptr + spacial_size * (c * channels + 5);

		for (int i = 0; i < fea_h; i++)
		{
			for (int j = 0; j < fea_w; j++)
			{
				int index = i * fea_w + j;


				float confidence = sigmoid(ptr_s[index]) * sigmoid(ptr_c[index]);
				//printf("c=%d, i=%d, j=%d,index=%d,confidenc = %f, threshold = %f \n", c, i, j, index, confidence, threshold);
				if (confidence > threshold)
				{
					//printf("c=%d, i=%d, j=%d,index=%d,confidenc = %f, threshold = %f \n", c, i, j, index, confidence, threshold);
					//printf("c=%d, i=%d, j=%d,confidenc = %f, threshold = %f \n", c, i, j, confidence, threshold);
					Object temp_box;
					float dx = sigmoid(ptr_x[index]);
					float dy = sigmoid(ptr_y[index]);
					float dw = sigmoid(ptr_w[index]);
					float dh = sigmoid(ptr_h[index]);

					float pb_cx = (dx * 2.f - 0.5f + j) * stride;
					float pb_cy = (dy * 2.f - 0.5f + i) * stride;

					float pb_w = pow(dw * 2.f, 2) * anchor_w;
					float pb_h = pow(dh * 2.f, 2) * anchor_h;

					temp_box.score = confidence;
					temp_box.x1 = pb_cx - pb_w * 0.5f;
					temp_box.y1 = pb_cy - pb_h * 0.5f;
					temp_box.x2 = pb_cx + pb_w * 0.5f;
					temp_box.y2 = pb_cy + pb_h * 0.5f;

					prebox.push_back(temp_box);
				}
			}
		}
	}
}

int BALLDETECTION::detect(const cv::Mat& rgb, std::vector<Object>& objects, float prob_threshold, float nms_threshold)
{
	cv::Mat dst;
	std::vector<float> infos = LetterboxImage(rgb, dst, cv::Size(YOLOFACE_INPUT_WIDTH, YOLOFACE_INPUT_HEIGHT));
	ncnn::Mat in = ncnn::Mat::from_pixels(dst.data, ncnn::Mat::PIXEL_RGB, dst.cols, dst.rows);

	in.substract_mean_normalize(0, norm_vals);
	ncnn::Extractor ex = yoloface.create_extractor();
	ex.set_num_threads(num_threads);
	ex.set_light_mode(true);
	ex.input("images", in);

	// stride 8
	{
		ncnn::Mat out;
		ex.extract("480", out);
		decode(out, anchor0, objects, prob_threshold, 8);
	}

	// stride 16
	{
		ncnn::Mat out;
		ex.extract("489", out);
		decode(out, anchor1, objects, prob_threshold, 16);
	}

	// stride 32
	{
		ncnn::Mat out;
		ex.extract("498", out);
		decode(out, anchor2, objects, prob_threshold, 32);
	}

	std::sort(objects.begin(), objects.end(), cmp);
	std::cout << "1=== objects.size:" << objects.size() << std::endl;
	nms(objects, nms_threshold);
	std::cout << "2=== objects.size:" << objects.size() << std::endl;
	for (int i = 0; i < objects.size(); i++)
	{
		objects[i].x1 = (objects[i].x1 - infos[0]) / infos[2];
		objects[i].y1 = (objects[i].y1 - infos[1]) / infos[2];
		objects[i].x2 = (objects[i].x2 - infos[0]) / infos[2];
		objects[i].y2 = (objects[i].y2 - infos[1]) / infos[2];
	}

	return 0;
}

std::vector<cv::Point2d> BALLDETECTION::ballDetection(const cv::Mat& rgb)
{
	int num_threads = 1;
	std::vector<Object> results;
	std::vector<cv::Point2d> points;
	cv::Point2d temp_lt, temp_rb;
	
	int ret = init("detect_markerball_0620-sim-opt", num_threads);
	if (ret < 0)
	{
		std::cout << "yolov7face init failed!!!\n";
	}
	
	detect(rgb, results);
		
	for (int i = 0; i < results.size(); i++)
	{
		float x_lt = results[i].x1;
		float y_lt = results[i].y1;
		float x_rb = results[i].x2;
		float y_rb = results[i].y2;

		temp_lt.x = x_lt;
		temp_lt.y = y_lt;
		temp_rb.x = x_rb;
		temp_rb.y = y_rb;

		points.push_back(temp_lt);
		points.push_back(temp_rb);
	
		std::cout << "左上角坐标：(" << x_lt << ", " << y_lt << ")" << std::endl;
		std::cout << "右下角坐标：(" << x_rb << ", " << y_rb << ")" << std::endl;

		cv::rectangle(rgb, temp_lt, temp_rb, (0, 0, 255), 2, 8, 0);
	}
	return points;
}

#endif
