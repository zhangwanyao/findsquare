#ifndef _UTIL_IMG_H_
#define _UTIL_IMG_H_
#include <string>
#include <vector>
#include <opencv2\core.hpp>
#include <opencv2\imgproc.hpp>
#include <opencv2\highgui.hpp>
#include <opencv2\features2d.hpp>
//#include "../config_dbg.h"

/**
* \brief utility of image
*/
class Util_Img 
{
public:
	/**
	* \brief constructor
	*/
	Util_Img() {}
	/**
	* \brief destructor
	*/
	~Util_Img() {}
	/**
	* \brief clear vector
	*/
	template<class T>
	static void clear(std::vector<T> &data) {
		data.clear();
	}
	/**
	* \brief clear vector
	*/
	template<class T>
	static void clear(std::vector<std::vector<T>> &data) {
		for (size_t i = 0; i < data.size(); ++i) {
			data[i].clear();
		}
		data.clear();
	}
	/**
	* \brief draw image
	*/
	static void showImg(const cv::Mat &img) {
		using namespace cv;
		char* win_name = "img";
		namedWindow(win_name, CV_WINDOW_AUTOSIZE);
		imshow(win_name, img);
		while (true){
			int c;
			c = waitKey(20);
			if ((char)c == 13){
				break;
			}
		}
	}
	/**
	* \brief merge two images
	*/
	static bool merge(const cv::Mat &src_0, const cv::Mat &src_1, double alpha, 
		cv::Mat &dst) 
	{
		using namespace cv;
		// data validation
		if (src_0.type() != src_1.type()) return false; // check type
		if (src_0.cols != src_1.cols) return false;	// check size
		if (src_0.rows != src_1.rows) return false;	// check size
		// prepare blending coefficients
		if (alpha < 0.0 || alpha > 1.0) alpha = 0.5;
		double beta = 1.0 - alpha;
		// merge
		addWeighted(src_0, alpha, src_1, beta, 0.0, dst);
		return true;
	}
	/**
	* \brief merge two images
	* merge src image to dst image, customize merging strategy
	*/
	static bool merge_c(const cv::Mat &src, cv::Mat &dst) 
	{
		using namespace cv;
		// data validation
		if (src.type() != dst.type()) return false; // check type
		if (src.cols != dst.cols) return false;	// check size
		if (src.rows != dst.rows) return false;	// check size
		// merge
		for (int k = 0; k < src.cols; k++) {
			for (int j = 0; j < src.rows; j++) {
				if (src.at<Vec3b>(j, k)[0] != 0) {
					if (dst.at<Vec3b>(j, k)[0] == 0) {
						dst.at<Vec3b>(j, k) = src.at<Vec3b>(j, k);
					}
					else {
						dst.at<Vec3b>(j, k)[0] = MIN(static_cast<int>(dst.at<Vec3b>(j, k)[0] * 0.5 + src.at<Vec3b>(j, k)[0] * 0.5), 255);
						dst.at<Vec3b>(j, k)[1] = MIN(static_cast<int>(dst.at<Vec3b>(j, k)[1] * 0.5 + src.at<Vec3b>(j, k)[1] * 0.5), 255);
						dst.at<Vec3b>(j, k)[2] = MIN(static_cast<int>(dst.at<Vec3b>(j, k)[2] * 0.5 + src.at<Vec3b>(j, k)[2] * 0.5), 255);
					}
				}
			}
		}
		return true;
	}
	/**
	* \brief save image
	*/
	static bool save(const std::string filePath, const cv::Mat &img){
		return cv::imwrite(filePath, img);
	}
	/**
	* \brief find convex hull
	*/
	static void convexHull(const cv::Mat &img_rgb, cv::Mat &img_hull, const bool isRect) {
		// cvt to gray
		cv::Mat img_gray;
		Util_Img::cvtRGB2Gray(img_rgb, img_gray);
		// blur
		cv::Mat img_blur;
		cv::blur(img_gray, img_blur, cv::Size(20, 20));
		// cvt to binary
		cv::Mat img_bin;
		Util_Img::cvtGray2BIN(img_blur, 10, img_bin);
		//Util_Img::showImg(img_bin);

		// find contours
		std::vector<std::vector<cv::Point>> cnts;
		std::vector<cv::Vec4i> hrchy;
		cv::findContours(img_bin, cnts, hrchy, cv::RETR_TREE, 
			cv::CHAIN_APPROX_SIMPLE);
		// merge contour points
		size_t npts_cnt = 0;
		for (size_t i = 0; i < cnts.size(); ++i) {
			npts_cnt += cnts[i].size();
		}
		std::vector<std::vector<cv::Point>> cnts_all(1);
		cnts_all[0].resize(npts_cnt);
		size_t count = -1;
		for (size_t i = 0; i < cnts.size(); ++i) {
			for (size_t j = 0; j < cnts[i].size(); ++j) {
				cnts_all[0][++count] = cnts[i][j];
			}
		}
		// find convex hull
		std::vector<std::vector<cv::Point>> hull(1);
		cv::convexHull(cnts_all[0], hull[0]);
		// draw convex hull
		img_hull = cv::Mat::zeros(img_bin.size(), CV_8UC3);
		cv::Scalar c_hull = cv::Scalar(0, 255, 0);
		cv::Scalar c_rect = cv::Scalar(0, 255, 0);
		if (!isRect) {
			cv::drawContours(img_hull, hull, 0, c_hull, 5);
			cv::drawContours(img_hull, hull, 0, c_hull, -1);
		}
		else {
			// draw bbox
			cv::Rect bRect;
			Util_Img::boundRect(hull[0], bRect);
			cv::rectangle(img_hull, bRect.tl(), bRect.br(), c_rect, 5);
			cv::rectangle(img_hull, bRect.tl(), bRect.br(), c_rect, -1);

		}
		//Util_Img::showImg(img_hull);
		// clear
		Util_Img::clear(hull);
		Util_Img::clear(cnts_all);
		Util_Img::clear(cnts);
		img_bin.release();
		img_blur.release();
		img_gray.release();
	}
	/**
	* \brief find contours
	*/
	static void contours(const cv::Mat &img_rgb, cv::Mat &img_cnt) {
		// cvt to gray
		cv::Mat img_gray;
		Util_Img::cvtRGB2Gray(img_rgb, img_gray);
		// blur
		cv::Mat img_blur;
		cv::blur(img_gray, img_blur, cv::Size(20, 20));
		// cvt to binary
		cv::Mat img_bin;
		Util_Img::cvtGray2BIN(img_blur, 1, img_bin);
		//Util_Img::showImg(img_bin);
		// find contours
		std::vector<std::vector<cv::Point>> cnts;
		std::vector<cv::Vec4i> hrchy;
		cv::findContours(img_bin, cnts, hrchy, cv::RETR_TREE,
			cv::CHAIN_APPROX_SIMPLE);
		// draw contours
		img_cnt = cv::Mat::zeros(img_bin.size(), CV_8UC3);
		cv::Scalar c_cnt = cv::Scalar(0, 255, 0);
		cv::Scalar c_rect = cv::Scalar(255, 0, 0);
		for (size_t i = 0; i < cnts.size(); ++i) {
			cv::drawContours(img_cnt, cnts, i, c_cnt, 2);
			cv::drawContours(img_cnt, cnts, i, c_cnt, -1);
			// draw bbox
			//cv::Rect bRect;
			//Util_Img::boundRect(cnts[i], bRect);
			//cv::rectangle(img_cnt, bRect.tl(), bRect.br(), c_rect, 5);
		}
		// Util_Img::showImg(img_hull);
		// clear
		Util_Img::clear(cnts);
		img_bin.release();
		img_blur.release();
		img_gray.release();
	}

	/**
	* \brief detect feature: SIFT
	* \param img input image
	* \param kPts keypoints
	* need to recompile OpenCV lib with opencv_contrib, in xfeature2d
	*/
	static void detcFeat_SIFT(cv::Mat &img, std::vector<cv::KeyPoint> &kPts) 
	{
		kPts.clear();
		// data validation
		if (img.empty()) return;
	}
	/**
	* \brief detect feature: FAST
	* \param img input image
	* \param kPts keypoints
	*/
	static void detcFeat_FAST(cv::Mat &img, std::vector<cv::KeyPoint> &kPts)
	{
		kPts.clear();
		// data validation
		if (img.empty()) return;
		//cv::Ptr<cv::FastFeatureDetector> detc = cv::FastFeatureDetector::create(10, true);
		//detc->detect(img, kPts);
		//cv::drawKeypoints(img, kPts, img);
		//Util_Img::showImg(img);
	}
	/**
	* \brief detect feature: ORB
	* \param img input image
	* \param kPts keypoints
	*/
	static void detcFeat_ORB(cv::Mat &img, std::vector<cv::KeyPoint> &kPts)
	{
		kPts.clear();
		// data validation
		if (img.empty()) return;
	}
	/**
	* \brief find bounding rect of contour
	*/
	static void boundRect(const std::vector<cv::Point> &cnt, cv::Rect &bRect) {
		bRect = cv::boundingRect(cnt);
	}
	/**
	* \brief two image bitwise AND
	*/
	static bool bitAND(const cv::Mat &src_0, const cv::Mat &src_1, cv::Mat &dst) {
		cv::bitwise_and(src_0, src_1, dst);
		return true;
	}
	/**
	* \brief two image bitwise OR
	*/
	static bool bitOR(const cv::Mat &src_0, const cv::Mat &src_1, cv::Mat &dst) {
		cv::bitwise_or(src_0, src_1, dst);
		return true;
	}
	/**
	* \brief two image bitwise XOR
	*/
	static bool bitXOR(const cv::Mat &src_0, const cv::Mat &src_1, cv::Mat &dst) {
		cv::bitwise_xor(src_0, src_1, dst);
		return true;
	}
	/**
	* \brief two image bitwise NOT
	*/
	static bool bitNOT(const cv::Mat &src, cv::Mat &dst) {
		cv::bitwise_not(src, dst);
		return true;
	}
	/**
	* \brief two image bitwise AND
	*/
	static bool bitAND_RGB(const cv::Mat &src_0, const cv::Mat &src_1, cv::Mat &dst) {
		cv::Mat src_0_b, src_1_b;
		if (!Util_Img::cvtRGB2BIN(src_0, src_0_b)) return false;
		if (!Util_Img::cvtRGB2BIN(src_1, src_1_b)) return false;
		return Util_Img::bitAND(src_0_b, src_1_b, dst);
	}
	/**
	* \brief two image bitwise OR
	*/
	static bool bitOR_RGB(const cv::Mat &src_0, const cv::Mat &src_1, cv::Mat &dst) {
		cv::Mat src_0_b, src_1_b;
		if (!Util_Img::cvtRGB2BIN(src_0, src_0_b)) return false;
		if (!Util_Img::cvtRGB2BIN(src_1, src_1_b)) return false;
		return Util_Img::bitOR(src_0_b, src_1_b, dst);
	}
	/**
	* \brief convert rgb to gray
	*/
	static bool cvtRGB2Gray(const cv::Mat &img_rgb, cv::Mat &img_gray) {
		if (img_rgb.channels() == 3) {
			cv::cvtColor(img_rgb, img_gray, CV_RGB2GRAY);
			return true;
		}
		else if (img_rgb.channels() == 1) {
			img_gray = img_rgb.clone();
			return true;
		}
		return false;
	}
	/**
	* \brief convert gray to rgb
	*/
	static bool cvtGray2RGB(const cv::Mat &img_gray, cv::Mat &img_rgb) {
		if (img_gray.channels() == 1) {
			cv::cvtColor(img_gray, img_rgb, CV_GRAY2RGB);
			return true;
		}
		return false;
	}
	/**
	* \brief convert gray to binary
	*/
	static bool cvtGray2BIN(const cv::Mat &img_gray, const float thres, cv::Mat &img_bin) {
		if (img_gray.channels() != 1) return false;
		if (thres < 0.f || thres > 255.f) return false;
		cv::threshold(img_gray, img_bin, thres, 255.f, CV_THRESH_BINARY);
		return true;
	}
	/**
	* \brief convert rgb to binary
	*/
	static bool cvtRGB2BIN(const cv::Mat &img_rgb, cv::Mat &img_bin) {
		cv::Mat img_gray;
		if (Util_Img::cvtRGB2Gray(img_rgb, img_gray)) {
			if (Util_Img::cvtGray2BIN(img_gray, 5.f, img_bin)) {
				img_gray.release();
				return true;
			}
		}
		img_gray.release();
		return false;
	}
	/**
	* \brief convert rgb to rgb, but with one color
	*/
	static bool cvtRGB2RGB_1C(cv::Mat &img) {
		cv::Mat img_temp;
		if (!Util_Img::cvtRGB2BIN(img, img_temp)) return false;
		if (!Util_Img::cvtGray2RGB(img_temp, img)) return false;
		return true;
	}

private:

};

#endif // !_UTIL_IMG_H_
