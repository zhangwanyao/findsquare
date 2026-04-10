#ifndef _UTIL_PCA_H_
#define _UTIL_PCA_H_
#include <vector>
#include <Eigen\Geometry>
#include <Eigen\Eigenvalues> 
#include <opencv2\core\core.hpp>

/**
* \brief class of principal component analysis
* \author JING Bichen
*/
class Util_PCA {
public:
	/**
	* \brief constructor
	*/
	Util_PCA(){}
	/**
	* \brief destructor
	*/
	~Util_PCA() {}
	/**
	* \brief compute pca
	*/
	// template<class VEC>
	static void pca3(const std::vector<cv::Point3f> &pts, cv::PCA &pca) {
		// data validation
		// assert(!pts.empty());
		if (pts.empty()) return;
		// compute covariance matrix
		size_t npts = pts.size();
		cv::Point3f centroid(0.f, 0.f, 0.f);
		for (size_t i = 0; i < npts; ++i) {
			centroid += pts[i];
		}
		centroid *= 1.f / static_cast<float>(npts);
		cv::Mat_<float> covariance = cv::Mat::zeros(3, 3, CV_32F);
		for (size_t i = 0; i < npts; ++i) {
			cv::Point3f pt = pts[i] - centroid;
			covariance.at<float>(1, 1) += pt.y * pt.y;
			covariance.at<float>(1, 2) += pt.y * pt.z;
			covariance.at<float>(2, 2) += pt.z * pt.z;
			pt *= pt.x;
			covariance.at<float>(0, 0) += pt.x;
			covariance.at<float>(0, 1) += pt.y;
			covariance.at<float>(0, 2) += pt.z;
		}
		covariance.at<float>(1, 0) = covariance.at<float>(0, 1);
		covariance.at<float>(2, 0) = covariance.at<float>(0, 2);
		covariance.at<float>(2, 1) = covariance.at<float>(1, 2);
		// compute eigen vector of maximum eigen value
		pca = cv::PCA(covariance, cv::Mat_<float>(), cv::PCA::DATA_AS_ROW, 3);
	}
	/**
	* \brief compute pca
	* \param step_sample step of sampling of pts, used for large data set
	*/
	// template<class VEC>
	static void pca3(const std::vector<cv::Point3f> &pts, const unsigned int step_sample, cv::PCA &pca) {
		// data validation
		// assert(!pts.empty());
		if (pts.empty()) return;
		unsigned int step = MAX(1, step_sample);
		// compute covariance matrix
		size_t npts = pts.size();
		cv::Point3f centroid(0.f, 0.f, 0.f);
		for (size_t i = 0; i < npts; i += step) {
			centroid += pts[i];
		}
		centroid *= 1.f / static_cast<float>(floor(npts / step) + 1);
		cv::Mat_<float> covariance = cv::Mat::zeros(3, 3, CV_32F);
		for (size_t i = 0; i < npts; i += step) {
			cv::Point3f pt = pts[i] - centroid;
			covariance.at<float>(1, 1) += pt.y * pt.y;
			covariance.at<float>(1, 2) += pt.y * pt.z;
			covariance.at<float>(2, 2) += pt.z * pt.z;
			pt *= pt.x;
			covariance.at<float>(0, 0) += pt.x;
			covariance.at<float>(0, 1) += pt.y;
			covariance.at<float>(0, 2) += pt.z;
		}
		covariance.at<float>(1, 0) = covariance.at<float>(0, 1);
		covariance.at<float>(2, 0) = covariance.at<float>(0, 2);
		covariance.at<float>(2, 1) = covariance.at<float>(1, 2);
		// compute eigen vector of maximum eigen value
		pca = cv::PCA(covariance, cv::Mat_<float>(), cv::PCA::DATA_AS_ROW, 3);
	}
	/**
	* \brief compute pca
	*/
	static void pca3(const std::vector<cv::Point3f> &pts, 
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 3, 3>> &eigenSolver)
	{
		typedef float scalar;
		typedef cv::Point3f PT;
		typedef Eigen::SelfAdjointEigenSolver<Eigen::Matrix<scalar, 3, 3>> EigenSolver;
		// data validation
		// assert(!pts.empty());
		if (pts.empty()) return;
		// compute covariance matrix
		size_t npts = pts.size();
		float inv_npts = 1.f / float(npts);
		cv::Point3f pt_avg(0.f, 0.f, 0.f);
		for (size_t i = 0; i < npts; ++i) {
			pt_avg += pts[i];
		}
		pt_avg *= inv_npts;
		// compute covariance matrix
		Eigen::Matrix<scalar, 3, 3> cov = Eigen::Matrix<scalar, 3, 3>::Zero();
		scalar demean_xy, demean_xz, demean_yz;
		for (unsigned int i = 0; i < npts; ++i) {
			PT demean = pts[i] - pt_avg;
			demean_xy = demean.x * demean.y;
			demean_xz = demean.x * demean.z;
			demean_yz = demean.y * demean.z;

			cov(0, 0) += demean.x * demean.x;
			cov(0, 1) += demean_xy;
			cov(0, 2) += demean_xz;

			cov(1, 0) += demean_xy;
			cov(1, 1) += demean.y * demean.y;
			cov(1, 2) += demean_yz;

			cov(2, 0) += demean_xz;
			cov(2, 1) += demean_yz;
			cov(2, 2) += demean.z * demean.z;
		}
		for (unsigned int i = 0; i < 3; i++) {
			for (unsigned int j = 0; j < 3; j++) {
				cov(i, j) *= inv_npts * inv_npts;
			}
		}
		eigenSolver = EigenSolver(cov);
	}
};

#endif // !_UTIL_PCA_H_
