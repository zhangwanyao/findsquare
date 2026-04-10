#ifndef _BBOX_H_
#define _BBOX_H_

#include <limits>
#include <vector>
#include <opencv2\opencv.hpp>

/**
* \brief 2D bounding box
* \author JING Bichen
*/
class BBox2D {
public:
	/**
	* \brief enum class of coordinates for 3d point (as projection)
	*/
	enum class TYPE_COORD {
		COORD_XY,
		COORD_XZ,
		COORD_YZ
	};
	/**
	* \brief constructor
	*/
	BBox2D()
		: m_min(cv::Point2f(
			std::numeric_limits<float>::infinity(),
			std::numeric_limits<float>::infinity()))
		, m_max(cv::Point2f(
			-1 * std::numeric_limits<float>::infinity(),
			-1 * std::numeric_limits<float>::infinity()))
	{
	}
	/**
	* \brief constructor
	*/
	BBox2D(cv::Point2f bbox_min, cv::Point2f bbox_max)
	{
		m_min = bbox_min;
		m_max = bbox_max;
	}
	/**
	* \brief destructor
	*/
	virtual ~BBox2D() {
	}
	/**
	* \brief compute
	*/
	bool compute(const std::vector<cv::Point2f> &pts) {
		// data validation
		if (pts.empty()) return false;

		// bbox compare
		float x_min, x_max, y_min, y_max;
		x_min = std::numeric_limits<float>::infinity();
		x_max = -1 * std::numeric_limits<float>::infinity();
		y_min = std::numeric_limits<float>::infinity();
		y_max = -1 * std::numeric_limits<float>::infinity();

		size_t npts = pts.size();

		for (const auto& p : pts) {
			x_min = (std::min)(x_min, p.x);
			x_max = (std::max)(x_max, p.x);
			y_min = (std::min)(y_min, p.y);
			y_max = (std::max)(y_max, p.y);
		}

		m_min = cv::Point2f(x_min, y_min);
		m_max = cv::Point2f(x_max, y_max);
		return true;
	}

	/**
	* \brief compute
	*/
	bool compute(const std::vector<cv::Point3f> &pts, const TYPE_COORD type) {
		// data validation
		if (pts.empty()) return false;

		// bbox compare
		float x_min, x_max, y_min, y_max, z_min, z_max;
		x_min = std::numeric_limits<float>::infinity();
		x_max = -1 * std::numeric_limits<float>::infinity();
		y_min = std::numeric_limits<float>::infinity();
		y_max = -1 * std::numeric_limits<float>::infinity();
		z_min = std::numeric_limits<float>::infinity();
		z_max = -1 * std::numeric_limits<float>::infinity();

		size_t npts = pts.size();

		switch (type)
		{
		case TYPE_COORD::COORD_XY:
			for (size_t i = 0; i < npts - 1; i++) {
				if (pts[i].x < pts[i + 1].x) {
					if (pts[i + 1].x > x_max) x_max = pts[i + 1].x;
					if (pts[i].x < x_min) x_min = pts[i].x;
				}
				else {
					if (pts[i].x > x_max) x_max = pts[i].x;
					if (pts[i + 1].x < x_min) x_min = pts[i + 1].x;
				}
				if (pts[i].y < pts[i + 1].y) {
					if (pts[i + 1].y > y_max) y_max = pts[i + 1].y;
					if (pts[i].y < y_min) y_min = pts[i].y;
				}
				else {
					if (pts[i].y > y_max) y_max = pts[i].y;
					if (pts[i + 1].y < y_min) y_min = pts[i + 1].y;
				}
			}
			m_min = cv::Point2f(x_min, y_min);
			m_max = cv::Point2f(x_max, y_max);
			break;
		case TYPE_COORD::COORD_XZ:
			for (size_t i = 0; i < npts - 1; i++) {
				if (pts[i].x < pts[i + 1].x) {
					if (pts[i + 1].x > x_max) x_max = pts[i + 1].x;
					if (pts[i].x < x_min) x_min = pts[i].x;
				}
				else {
					if (pts[i].x > x_max) x_max = pts[i].x;
					if (pts[i + 1].x < x_min) x_min = pts[i + 1].x;
				}
				if (pts[i].z < pts[i + 1].z) {
					if (pts[i + 1].z > z_max) z_max = pts[i + 1].z;
					if (pts[i].z < z_min) z_min = pts[i].z;
				}
				else {
					if (pts[i].z > z_max) z_max = pts[i].z;
					if (pts[i + 1].z < z_min) z_min = pts[i + 1].z;
				}
			}
			m_min = cv::Point2f(x_min, z_min);
			m_max = cv::Point2f(x_max, z_max);
			break;
		case TYPE_COORD::COORD_YZ:
			for (size_t i = 0; i < npts - 1; i++) {
				if (pts[i].y < pts[i + 1].y) {
					if (pts[i + 1].y > y_max) y_max = pts[i + 1].y;
					if (pts[i].y < y_min) y_min = pts[i].y;
				}
				else {
					if (pts[i].y > y_max) y_max = pts[i].y;
					if (pts[i + 1].y < y_min) y_min = pts[i + 1].y;
				}
				if (pts[i].z < pts[i + 1].z) {
					if (pts[i + 1].z > z_max) z_max = pts[i + 1].z;
					if (pts[i].z < z_min) z_min = pts[i].z;
				}
				else {
					if (pts[i].z > z_max) z_max = pts[i].z;
					if (pts[i + 1].z < z_min) z_min = pts[i + 1].z;
				}
			}
			m_min = cv::Point2f(y_min, z_min);
			m_max = cv::Point2f(y_max, z_max);
			break;
		default:
			break;
		}
		return true;
	}

	/**
	* \brief overrided operator + for merge bounding box
	*/
	friend BBox2D operator + (const BBox2D &b0, const BBox2D &b1) {
		float x_min, x_max, y_min, y_max;
		x_min = MIN(b0.m_min.x, b1.m_min.x);
		y_min = MIN(b0.m_min.y, b1.m_min.y);
		x_max = MAX(b0.m_max.x, b1.m_max.x);
		y_max = MAX(b0.m_max.y, b1.m_max.y);
		BBox2D bbox;
		bbox.m_min = cv::Point2f(x_min, y_min);
		bbox.m_max = cv::Point2f(x_max, y_max);
		return bbox;
	}
public:
	cv::Point2f m_min;	/**< bbox min */
	cv::Point2f m_max;	/**< bbox max */
};

/**
* \brief 3D bounding box
* \author JING Bichen
*/
class BBox3D {
public:
	/**
	* \brief constructor
	*/
	BBox3D()
		: m_min(cv::Point3f(
			std::numeric_limits<float>::infinity(),
			std::numeric_limits<float>::infinity(),
			std::numeric_limits<float>::infinity()))
		, m_max(cv::Point3f(
			-1 * std::numeric_limits<float>::infinity(),
			-1 * std::numeric_limits<float>::infinity(),
			-1 * std::numeric_limits<float>::infinity()))
	{
	}
	/**
	* \brief constructor
	*/
	BBox3D(cv::Point3f bbox_min, cv::Point3f bbox_max)
	{
		m_min = bbox_min;
		m_max = bbox_max;
	}
	/**
	* \brief destructor
	*/
	virtual ~BBox3D() {
	}
	/**
	* \brief compute
	*/
	bool compute(const std::vector<cv::Point3f> &pts) {
		// data validation
		if (pts.empty()) return false;

		// bbox compare
		float x_min, x_max, y_min, y_max, z_min, z_max;
		x_min = std::numeric_limits<float>::infinity();
		x_max = -1 * std::numeric_limits<float>::infinity();
		y_min = std::numeric_limits<float>::infinity();
		y_max = -1 * std::numeric_limits<float>::infinity();
		z_min = std::numeric_limits<float>::infinity();
		z_max = -1 * std::numeric_limits<float>::infinity();

		size_t npts = pts.size();

		for (size_t i = 0; i < npts - 1; i++) {
			if (pts[i].x < pts[i + 1].x) {
				if (pts[i + 1].x > x_max) x_max = pts[i + 1].x;
				if (pts[i].x < x_min) x_min = pts[i].x;
			}
			else {
				if (pts[i].x > x_max) x_max = pts[i].x;
				if (pts[i + 1].x < x_min) x_min = pts[i + 1].x;
			}
			if (pts[i].y < pts[i + 1].y) {
				if (pts[i + 1].y > y_max) y_max = pts[i + 1].y;
				if (pts[i].y < y_min) y_min = pts[i].y;
			}
			else {
				if (pts[i].y > y_max) y_max = pts[i].y;
				if (pts[i + 1].y < y_min) y_min = pts[i + 1].y;
			}
			if (pts[i].z < pts[i + 1].z) {
				if (pts[i + 1].z > z_max) z_max = pts[i + 1].z;
				if (pts[i].z < z_min) z_min = pts[i].z;
			}
			else {
				if (pts[i].z > z_max) z_max = pts[i].z;
				if (pts[i + 1].z < z_min) z_min = pts[i + 1].z;
			}
		}
		m_min = cv::Point3f(x_min, y_min, z_min);
		m_max = cv::Point3f(x_max, y_max, z_max);
		return true;
	}

	float getAreaXOY()
	{
		return (m_max.x - m_min.x)*(m_max.y - m_min.y);
	}
	/**
	* \brief dilate, change original bounding box, careful
	*/
	void dilate(const float d) {
		float ex = fabsf(d);
		m_min -= cv::Point3f(ex, ex, ex);
		m_max += cv::Point3f(ex, ex, ex);
	}
	/**
	* \brief check if overlap with another bbox
	*/
	bool isOverlap(const BBox3D &b) const {
		if (m_min.x > b.m_max.x) return false;
		if (m_min.y > b.m_max.y) return false;
		if (m_min.z > b.m_max.z) return false;
		if (m_max.x < b.m_min.x) return false;
		if (m_max.y < b.m_min.y) return false;
		if (m_max.z < b.m_min.z) return false;
		return true;
	}

	/**
	* \brief check if overlap with another bbox
	*/
	bool isFather(const BBox3D &b) const {
		return  (m_min.x < b.m_min.x) &&
			(m_min.y < b.m_min.y) &&
			(m_max.x > b.m_max.x) &&
			(m_max.y > b.m_max.y);
	}

	/**
	* \brief check if two bbox overlap
	*/
	static bool isOverlap(const BBox3D &a, const BBox3D &b) {
		if (a.m_min.x > b.m_max.x) return false;
		if (a.m_min.y > b.m_max.y) return false;
		if (a.m_min.z > b.m_max.z) return false;
		if (a.m_max.x < b.m_min.x) return false;
		if (a.m_max.y < b.m_min.y) return false;
		if (a.m_max.z < b.m_min.z) return false;
		return true;
	}

	/**
	* \brief check if two bbox overlap
	*/
	static bool isOverlapXOY(const BBox3D &a, const BBox3D &b, float r) {
		r = fabsf(r);
		if ((a.m_min.x - r) > (b.m_max.x + r)) return false;
		if ((a.m_min.y - r) > (b.m_max.y + r)) return false;
		//if ((a.m_min.z - r) > (b.m_max.z + r)) return false;
		if ((a.m_max.x + r) < (b.m_min.x - r)) return false;
		if ((a.m_max.y + r) < (b.m_min.y - r)) return false;
		//if ((a.m_max.z + r) < (b.m_min.z - r)) return false;
		return true;
	}
	/**
	* \brief get a dilated bounding box
	*/
	BBox3D getDilateBBox(const float d) {
		float ex = fabsf(d);
		cv::Point3f bbox_min = m_min - cv::Point3f(ex, ex, ex);
		cv::Point3f bbox_max = m_max + cv::Point3f(ex, ex, ex);
		return BBox3D(bbox_min, bbox_max);
	}
	/**
	* \brief overrided operator + for merge bounding box
	*/
	friend BBox3D operator + (const BBox3D &b0, const BBox3D &b1) {
		float x_min, x_max, y_min, y_max, z_min, z_max;
		x_min = MIN(b0.m_min.x, b1.m_min.x);
		y_min = MIN(b0.m_min.y, b1.m_min.y);
		z_min = MIN(b0.m_min.z, b1.m_min.z);
		x_max = MAX(b0.m_max.x, b1.m_max.x);
		y_max = MAX(b0.m_max.y, b1.m_max.y);
		z_max = MAX(b0.m_max.z, b1.m_max.z);
		BBox3D bbox;
		bbox.m_min = cv::Point3f(x_min, y_min, z_min);
		bbox.m_max = cv::Point3f(x_max, y_max, z_max);
		return bbox;
	}
public:
	cv::Point3f m_min;	/**< bbox min */
	cv::Point3f m_max;	/**< bbox max */
};

#endif