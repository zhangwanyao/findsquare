#ifndef _DOC_PC_H_
#define _DOC_PC_H_

#include "doc.h"
#include "bbox.h"
//#include "../../../include/DataStruct.h"
#include <opencv2\opencv.hpp>

/**
* \brief document of point cloud
* \author Bichen JING
*/
class Doc_PC : public Doc {
public:
	/**
	* \brief constructor
	*/
	Doc_PC()
		: m_idx(-1)
		, m_center(cv::Point3f(0.f, 0.f, 0.f))
	{
	}
	/**
	* \brief destructor
	*/
	virtual ~Doc_PC() {
		clear();
	}
	/**
	* \brief virtual function, clear data
	*/
	virtual void clear() {
	}
	/**
	* \brief virtual function, update model
	*/
	virtual void update() {
	}
	/**
	* \brief update bbox
	*/
	bool updateBBox(const std::vector<cv::Point3f> &pts) {
		return m_bbox.compute(pts);
	}
	///**
	//* \brief update bbox
	//*/
	//bool updateBBox(PointArray *ptArray) {
	//	return m_bbox.compute(ptArray);
	//}
	/**
	* \brief update center
	*/
	bool updateCenter(const std::vector<cv::Point3f> &pts) {
		if (pts.empty()) return false;
		size_t npts = pts.size();
		double fnpts = static_cast<double>(npts);
		cv::Point3d temp(0.0, 0.0, 0.0);
		for (size_t i = 0; i < npts; ++i) {
			temp += (cv::Point3d)pts[i];
		}
		m_center = temp /= fnpts;
		return true;
	}
	///**
	//* \brief update center
	//*/
	//bool updateCenter(PointArray *ptArray) {
	//	if (ptArray == nullptr) return false;
	//	size_t npts = ptArray->size;
	//	float fnpts = static_cast<float>(npts);
	//	PointItem *points = ptArray->points;
	//	cv::Point3f temp(0.f, 0.f, 0.f);
	//	for (size_t i = 0; i < npts; i++) {
	//		temp = temp + points[i].point;
	//	}
	//	m_center = cv::Point3f(temp.x / fnpts, temp.y / fnpts, temp.z / fnpts);
	//	return true;
	//}
	/**
	* \brief set index
	*/
	void setIdx(const int idx) {
		m_idx = idx;
	}
	/**
	* \brief get index
	*/
	int getIdx() const {
		return m_idx;
	}
	/**
	* \brief set bbox
	*/
	void setBBox(const BBox3D &bbox) {
		m_bbox = bbox;
	}
	/**
	* \brief get bbox
	*/
	BBox3D getBBox() const {
		return m_bbox;
	}
	/**
	* \brief set center
	*/
	void setCenter(const cv::Point3f center) {
		m_center = center;
	}
	/**
	* \brief get center
	*/
	cv::Point3f getCenter() const {
		return m_center;
	}
protected:
	int m_idx;				/**< index of point cloud */
	BBox3D m_bbox;			/**< bbox of point cloud */
	cv::Point3f m_center;	/**< center of point cloud */
};

#endif // !_DOC_PC_H_
