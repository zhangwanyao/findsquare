#ifndef _UTIL_OPEN3D_H_
#define _UTIL_OPEN3D_H_

#include "ModuleStruct.hpp"
#include "open3d/Open3D.h"

using namespace ModuleStruct;
using namespace open3d;

namespace Util_3D {
	typedef geometry::PointCloud PointCloud;
	typedef geometry::KDTreeFlann KDTreeFlann;
	typedef Eigen::Vector3d Vector3d;

	/**
	* \brief assign data to geometry::Geometry
	*/
	template<typename Tp1, typename Tp2>
	static inline void AssignDataToGeometry(Tp1& geometry, const Tp2& data) {
		geometry.points_.resize(data.size());

#pragma omp parallel for
		for (int i = 0; i < data.size(); i++)
		{
			geometry.points_[i](0) = data[i].x;
			geometry.points_[i](1) = data[i].y;
			geometry.points_[i](2) = data[i].z;
		}
	}

	/**
	* \brief assign data to geometry::Geometry
	*/
	template<typename Tp1, typename Tp2>
	static inline void AssignPairDataToGeometry(Tp1& geometry, const Tp2& data) {
		geometry.points_.resize(data.size());

#pragma omp parallel for
		for (int i = 0; i < data.size(); i++)
		{
			geometry.points_[i](0) = data[i].first.x;
			geometry.points_[i](1) = data[i].first.y;
			geometry.points_[i](2) = data[i].first.z;
		}
	}

	/**
	* \brief wrapper function of open3d::geometry::KDTreeFlann::SetGeometry()
	* Tp1 -> geometry::KDTreeFlann
	* Tp2 -> geometry::Geometry
	*/
	template<typename Tp1, typename Tp2>
	static inline bool SetGeometry(Tp1 &kdtree, const Tp2 &geometry) {
		return kdtree.SetGeometry(geometry);
	}

	/**
	* \brief wrapper function of open3d::geometry::KDTreeFlann::SearchKNN()
	* Tp1 -> geometry::KDTreeFlann
	* Tp2 -> Eigen::Vector3d
	*/
	template<typename Tp1, typename Tp2>
	static inline void SearchKNN(const Tp1 &kdtree, const Tp2 &query, int knn, Vector<int> &indices, Vector<double> &distance) {
		kdtree.SearchKNN(query, knn, indices, distance);
	}
}

#endif // !_UTIL_OPEN3D_H_