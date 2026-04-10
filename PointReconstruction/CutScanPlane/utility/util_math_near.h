#ifndef _UTIL_MATH_NEAR_H_
#define _UTIL_MATH_NEAR_H_

#include "util_math.hpp"
#include <cmath>
#include <vector>
#include <opencv2\core.hpp>

namespace Util_Math_Near {
	/**
	* \brief check if two scalar values near equal or not, under given threshold
	*/
	template<typename S0, typename S1, typename T>
	inline bool isNearEqual(const S0 a, const S1 b, const T thres) {
		return fabs(a - b) <= fabs(thres);
	}
	/**
	* \brief check if two vectors have near direction or not, under given threshold
	* check the angle between two vectors
	*/
	template<class VEC, typename T>
	inline bool isNearDir(const VEC &a, const VEC &b, const T thres_deg) {
		// if any vector is zero vector, return true
		if (Util_Math::vec3_is_zero(a) || Util_Math::vec3_is_zero(b)) return true;
		return Util_Math::vec3_angle_deg(a, b) < fabs(thres_deg);
	}
	/**
	* \brief check if two vectors near parallel or not, under given threshold
	*/
	template<class VEC, typename T>
	inline bool isNearParl(const VEC &a, const VEC &b, const T thres_deg) {
		return isNearDir(a, b, thres_deg) || isNearDir(a, -b, thres_deg);
	}
	/**
	* \brief clear std vector
	*/
	template<typename T>
	inline void clearStdVector(std::vector<std::vector<T>> &data) {
		if (data.empty()) return;
		for (size_t i = 0; i < data.size(); ++i) {
			data[i].clear();
		}
		data.clear();
	}
	/**
	* \brief compare two clusters size, if first > second, return true
	*/
	static bool cmpClusterSize(const std::vector<unsigned int> &a, const std::vector<unsigned int> &b) {
		return a.size() > b.size();
	}
	/**
	* \brief cluster vectors
	* \param vecs input vector list
	* \param thres_deg threshold of angle in format of degree to group vectors
	* \param output num of cluster<num of vectors>
	*/
	static void clusterVecs(const std::vector<cv::Point3f> &vecs, const float thres_deg, std::vector<std::vector<unsigned int>> &output) 
	{
		typedef std::vector<unsigned int> idList;
		clearStdVector(output);
		// data validation
		if (vecs.empty()) return;
		std::vector<idList> clusters;
		for (int i = 0; i < vecs.size(); ++i) {
			cv::Point3f qVec = vecs[i];
			bool isInCluster = false;
			// check query vector with first element of cluster
			for (int c = 0; c < clusters.size(); ++c) {
				if (isNearParl(qVec, vecs[clusters[c].front()], thres_deg)) {
					isInCluster = true;
					clusters[c].push_back(i);
					break;
				}
			}
			// if no current cluster found, create new cluster
			if (!isInCluster) {
				idList newCluster;
				newCluster.push_back(i);
				clusters.push_back(newCluster);
			}
		}
		// sort clusters
		std::sort(clusters.begin(), clusters.end(), cmpClusterSize);
		// output and clear
		output = clusters;
		clusters.clear();
	}

}

#endif // !_UTIL_MATH_NEAR_H_
