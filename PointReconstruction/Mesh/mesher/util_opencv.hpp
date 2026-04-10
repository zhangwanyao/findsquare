#ifndef _UTIL_OPENCV_H_
#define _UTIL_OPENCV_H_

#include <opencv2/core.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/types_c.h>

namespace Util_CV {

	// keep same stucture with ModuleStruct.hpp

	/**
	* \brief wrapper function of cv::Mat::eye()
	* Tp -> cv::Mat for opencv
	*/
	template<typename Tp>
	static inline Tp eye(int rows, int cols, int type) {
		return Tp::eye(rows, cols, type);
	}
	 
	/**
	* \brief wrapper function of cv::Mat::zeros()
	* Tp -> cv::Mat for opencv
	*/
	template<typename Tp>
	static inline Tp zeros(int rows, int cols, int type) {
		return Tp::zeros(rows, cols, type);
	}

	/**
	* \brief wrapper function of cv::Rect
	* Tp -> data type
	*/
	template<typename Tp>
	static inline cv::Rect Rect(Tp x, Tp y, Tp width, Tp height) {
		return cv::Rect(x, y, width, height);
	}

	/**
	* \brief wrapper function of cv::eigen()
	* Tp -> cv::Mat for opencv
	*/
	template<typename Tp>
	static inline bool eigen(const Tp& input, Tp& eigenvalues, Tp& eigenvectors) {
		return cv::eigen(input, eigenvalues, eigenvectors);
	}

	/**
	* \brief wrapper function of cv::calcCovarMatrix()
	* Tp -> cv::Mat for opencv
	*/
	template<typename Tp>
	static inline void covariance_matrix(const Tp& input, Tp& covariance_matrix, Tp& mean, unsigned int rows) {
		cv::calcCovarMatrix(input, covariance_matrix, mean, cv::COVAR_NORMAL | cv::COVAR_ROWS);
		covariance_matrix = covariance_matrix / (rows - 1);
	}

	/**
	* \brief wrapper function of cv::imwrite()
	* Tp -> cv::Mat for opencv
	*/
	template<typename Tp>
	static inline bool imwrite(const std::string& filename, Tp img) {
		return(cv::imwrite(filename, img));
	}

	/**
	* \brief wrapper function of cv::cvtColorBgr2Gray()
	* Tp -> cv::Mat for opencv
	*/
	template<typename Tp>
	static inline void cvtColor(Tp src, Tp &dst, int code) {
		cv::cvtColor(src, dst, code);
	}

	/**
	* \brief wrapper function of cv::threshold()
	* Tp -> cv::Mat for opencv
	*/
	template<typename Tp, typename T>
	static inline T threshold(Tp src, Tp &dst, T thresh, T maxval, int type) {
		return(cv::threshold(src, dst, thresh, maxval,type));
	}

	/**
	* \brief wrapper function of cv::drawContours()
	* Tp -> cv::Mat for opencv
	*/
	template<typename Tp, typename Tp1, typename Tp2>
	static inline void findContours(Tp image, Tp1 &contours, Tp2&hierarchy, int mode, int method) {
		cv::findContours(image, contours, hierarchy, mode, method);
	}

	/**
	* \brief wrapper function of cv::drawContours()
	* Tp -> cv::Mat for opencv
	*/
	template<typename Tp, typename Tp1, typename Tp2>
	static inline void drawContours(Tp image, Tp1 contours, int contourIdx, const Tp2& color, int thickness) {
		cv::drawContours(image, contours, contourIdx, color, thickness);
	}

	/**
	* \brief wrapper function of cv::blur()
	* Tp -> cv::Mat for opencv
	*/
	template<typename Tp, typename Tp1>
	static inline void blur(Tp src, Tp &dst, Tp1 ksize) {
		cv::blur(src, dst, ksize);
	}

	/**
	* \brief wrapper function of cv::Canny()
	* Tp -> cv::Mat for opencv
	*/
	template<typename Tp, typename T>
	static inline void Canny(Tp image, Tp &edges, T threshold1, T threshold2, int apertureSize) {
		cv::Canny(image, edges, threshold1, threshold2, apertureSize);
	}

}

#endif // !_UTIL_OPENCV_H_
