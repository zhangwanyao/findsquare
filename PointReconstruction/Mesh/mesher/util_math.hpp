#ifndef _UTIL_MATH_H_
#define _UTIL_MATH_H_

#include "ModuleStruct.hpp"
#include "util_opencv.hpp"

using namespace std;

/**
* \author Bichen JING, Elaine Li
*/
namespace Util_Math {
	/**
	* \brief compare two vals equal or not
	* \tparam DT data type
	* \param[in] a first value
	* \param[in] b second value
	* \param[in] eps epsilon for equal judgement
	*/
	template<class DT>
	inline bool isEqual(DT a, DT b, DT eps) {
		if (static_cast<DT>(abs(a - b)) < eps) return true;
		return false;
	}
	 
	/**
	* \brief vector normalization
	*/
	template<class VEC>
	inline VEC vec3_normalize(VEC vec) {
		float len = sqrtf(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
		return (fabs(len) < numeric_limits<float>::epsilon()) ? vec
			: VEC(vec.x / len, vec.y / len, vec.z / len);
	}

	/**
	* \brief check if vector is zero
	*/
	template<class VEC>
	inline bool vec3_is_zero(VEC vec) {
		//return (fabs(vec.x) < numeric_limits<float>::epsilon()
		//	&&	fabs(vec.y) < numeric_limits<float>::epsilon() 
		//	&&	fabs(vec.z) < numeric_limits<float>::epsilon()) ? true : false;
		return (fabs(vec.x) < 1e-6
			&& fabs(vec.y) < 1e-6
			&& fabs(vec.z) < 1e-6) ? true : false;
	}

	/**
	* \brief check if two vectors same
	*/
	template<class VEC>
	inline bool vec3_are_same(VEC vec0, VEC vec1) {
		return (fabs(vec0.x - vec1.x) < numeric_limits<float>::epsilon()
			&& fabs(vec0.y - vec1.y) < numeric_limits<float>::epsilon()
			&& fabs(vec0.z - vec1.z) < numeric_limits<float>::epsilon()) ? true : false;
	}

	/**
	* \brief check if two vectors parallel
	*/
	template<class VEC3>
	inline bool vec3_are_parallel(VEC3 v0, VEC3 v1) {
		return vec3_is_zero(v0.cross(v1)) ? true : false;
	}

	/**
	* \brief compute angle between two vectors
	*/
	template<class VEC>
	inline float vec3_angle(VEC vec0, VEC vec1) {
		vec0 = vec3_normalize(vec0);
		vec1 = vec3_normalize(vec1);
		return acosf(vec0.x * vec1.x + vec0.y * vec1.y + vec0.z * vec1.z);
	}

	/**
	* \brief compute angle between two vectors, return degree
	*/
	template<class VEC>
	inline float vec3_angle_deg(VEC vec0, VEC vec1) {
		vec0 = vec3_normalize(vec0);
		vec1 = vec3_normalize(vec1);
		return acosf(vec0.x * vec1.x + vec0.y * vec1.y + vec0.z * vec1.z) / 3.1415926 * 180.0f;
	}

	/**
	* \brief compute vector length
	*/
	template<class VEC>
	inline double vec3_len(VEC vec) {
		return sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
	}
	/**
	* \brief project vector to plane
	* \param vec query vector
	* \param norm normal of given plane
	* \return projected vector
	*/
	template<class VEC>
	inline VEC proj_vec3ToPlane(const VEC& vec, const VEC& norm) {
		if (vec3_is_zero(vec) || vec3_is_zero(norm)) return vec;
		float ln = vec3_len(norm);
		return vec - (vec.dot(norm) / (ln * ln)) * norm;
	}
	/**
	* \brief compute matrix multiply point
	* \tparam MT matrix type
	* \tparam PT point type
	* \tparam DT data type, e.g. float, double
	* @param[in] m matrix3x3
	* @param[in] pt point3f
	* @param[out] mpt result point3f
	*/
	template<class MT, class PT, class DT>
	inline void mat3_mult_pt3(const MT m, const PT pt, PT& mpt) {
		MT src(3, 1);
		src.template at<DT>(0, 0) = pt.x; // src.at<DT>(0, 0) = pt.x; // visual studio ok
		src.template at<DT>(1, 0) = pt.y;
		src.template at<DT>(2, 0) = pt.z;
		MT dst = m * src;
		mpt.x = dst.template at<DT>(0, 0);
		mpt.y = dst.template at<DT>(1, 0);
		mpt.z = dst.template at<DT>(2, 0);
		return;
	}

	/**
	* \brief compute matrix multiply point
	* \tparam MT matrix type
	* \tparam PT point type
	* \tparam DT data type, e.g. float, double
	* @param[in] m matrix3x3
	* @param[in] pt point3f
	* @param[out] mpt result matrix3x1
	*/
	//template<class MT, class PT, class DT>
	//inline void mat3_mult_pt3(const MT m, const PT pt, MT& mpt) {
	//	MT src(3, 1);
	//	src.at<DT>(0, 0) = pt.x;
	//	src.at<DT>(1, 0) = pt.y;
	//	src.at<DT>(2, 0) = pt.z;
	//	mpt = m * src;
	//	return;
	//}

	/**
	* \brief compute matrix multiply point
	* \tparam MT matrix type
	* \tparam PT point type
	* \tparam DT data type, e.g. float, double
	* @param[in] m matrix3x3
	* @param[in] pts list of point3f
	*/
	template<class MT, class PT, class DT>
	inline void mat3_mult_pt3(const MT& m, vector<PT>& pts) {
		for (size_t i = 0; i < pts.size(); ++i) {
			MT src(3, 1);
			src.template at<DT>(0, 0) = pts[i].x;
			src.template at<DT>(1, 0) = pts[i].y;
			src.template at<DT>(2, 0) = pts[i].z;
			MT tgt = m * src;
			pts[i].x = tgt.template at<DT>(0, 0);
			pts[i].y = tgt.template at<DT>(1, 0);
			pts[i].z = tgt.template at<DT>(2, 0);
		}
	}
	// example
	// CMat M = Util_CV::eye<CMat>(3, 3, CV_32F);
	// Point3f pt(1.0, 2.0, -3.0);
	// Point3f mpt;
	// mat3_mult_pt3<CMat_<float>, Point3f, float>(M, pt, mpt);
	// DBG_OUT << mpt.x << " " << mpt.y << " " << mpt.z;

	/**
	* \brief compute rotation matrix using quaternion method
	* \param[in] u rotation axis
	* \param[in] theta rotation angle
	*/
	template<class MT, class PT, class DT>
	inline void get_rot_mat3(PT u, DT theta, MT& mat) {
		u = vec3_normalize(u);
		DT theta_half = static_cast<DT>(theta / 2.f);
		// calculate rotation quaternion (a, bi, cj, dk)
		DT a = cos(theta_half);
		DT b = sin(theta_half) * u.x;
		DT c = sin(theta_half) * u.y;
		DT d = sin(theta_half) * u.z;
		// calculate transformation matrix elements
		mat.template at<DT>(0, 0) = 1 - 2 * c * c - 2 * d * d;
		mat.template at<DT>(0, 1) = 2 * b * c - 2 * a * d;
		mat.template at<DT>(0, 2) = 2 * a * c + 2 * b * d;
		mat.template at<DT>(1, 0) = 2 * b * c + 2 * a * d;
		mat.template at<DT>(1, 1) = 1 - 2 * b * b - 2 * d * d;
		mat.template at<DT>(1, 2) = 2 * c * d - 2 * a * b;
		mat.template at<DT>(2, 0) = 2 * b * d - 2 * a * c;
		mat.template at<DT>(2, 1) = 2 * a * b + 2 * c * d;
		mat.template at<DT>(2, 2) = 1 - 2 * b * b - 2 * c * c;
	}

	/**
	* \brief find min and max value in a vector with given min and max threshold
	* \tparam T data type
	* @param[in] vals vector of data
	* @param[in] thres_min minimum value of type T
	* @param[in] thres_max maximum value of type T
	* @param[out] val_min minimum value of vector
	* @param[out] val_max maximum value of vector
	*/
	template<class T>
	inline void arr_min_max(const vector<T>& vals, const T thres_min, const T thres_max, T& val_min, T& val_max)
	{
		T cur_min = thres_max;
		T cur_max = thres_min;
		size_t nv = vals.size();
		if (nv == 0) {	// vector empty, return threshold
			val_min = thres_min;
			val_max = thres_max;
			return;
		}
		if (nv == 1) {	// vector has one element, return the only element as both min and max
			val_min = vals[0];
			val_max = vals[0];
			return;
		}
		for (size_t i = 0; i < nv - 1; i++) {
			if (vals[i] < vals[i + 1]) {
				if (vals[i + 1] > cur_max) cur_max = vals[i + 1];
				if (vals[i] < cur_min) cur_min = vals[i];
			}
			else {
				if (vals[i] > cur_max) cur_max = vals[i];
				if (vals[i + 1] < cur_min) cur_min = vals[i + 1];
			}
		}
		val_min = cur_min;
		val_max = cur_max;
	}

	/**
	* \brief find min and max value in a vector with given min and max threshold
	* \tparam T data type
	* @param[in] vals vector of data
	* @param[out] val_min minimum value of vector
	* @param[out] val_max maximum value of vector
	*/
	template<class T>
	inline void arr_min_max(const vector<T>& vals, T& val_min, T& val_max)
	{
		T cur_min = numeric_limits<T>::max();
		T cur_max = numeric_limits<T>::min();
		size_t nv = vals.size();
		if (nv == 0) {	// vector empty, return threshold
			val_min = numeric_limits<T>::min();
			val_max = numeric_limits<T>::max();
			return;
		}
		if (nv == 1) {	// vector has one element, return the only element as both min and max
			val_min = vals[0];
			val_max = vals[0];
			return;
		}
		for (size_t i = 0; i < nv - 1; i++) {
			if (vals[i] < vals[i + 1]) {
				if (vals[i + 1] > cur_max) cur_max = vals[i + 1];
				if (vals[i] < cur_min) cur_min = vals[i];
			}
			else {
				if (vals[i] > cur_max) cur_max = vals[i];
				if (vals[i + 1] < cur_min) cur_min = vals[i + 1];
			}
		}
		val_min = cur_min;
		val_max = cur_max;
	}

	/**
	* \brief check if two float values equal or not
	* \author Bichen JING
	* \tparam VAL type of value
	* \tparam CMP type of compared value
	* \param val input value
	* \param cmp input compared value
	* \return true if values are equal, otherwise, false
	*
	* need to include <limits>
	*/
	template<typename VAL, typename CMP>
	static inline bool IsValEqual(const VAL& val, const CMP& cmp) {
		return fabs(val - static_cast<VAL>(cmp)) <= numeric_limits<VAL>::epsilon();
	}

	/**
	* \brief check if a float values equals to zero or not
	* \author Bichen JING
	* \tparam T type of value
	* \param val input value
	* \return true if value equals to zero, otherwise, false
	*/
	template<typename T>
	static inline bool IsValZero(const T& val) {
		return IsValEqual(val, 0.0);
	}

	/**
	* \brief check if value in bounds
	* \author Bichen JING
	* \tparam T type of value
	* \param val input value
	* \param lower lower bound
	* \param upper upper bound
	* \return true if value in bounds, otherwise, false
	*/
	template<typename T>
	static inline bool IsInBounds(const T& val, const T& lower, const T& upper) {
		// data validation
		assert(lower <= upper);
		return !(val < lower) && !(val > upper);
	}

	/**
	* \brief check if value in bounds
	* \author Bichen JING
	* \tparam T type of value
	* \param val input value
	* \param bounds bounding values in pair
	* \return true if value in bounds, otherwise, false
	*/
	template<typename T>
	static inline bool IsInBounds(const T& val, const pair<T, T>& bounds)
	{
		// data validation
		assert(bounds.first <= bounds.second);
		return !(val < bounds.first) && !(val > bounds.second);
	}

	/**
	* \brief check if 2d point in bounds
	* \author Bichen JING
	* \tparam T type of value
	* \param x x-coord of point
	* \param y y-coord of point
	* \param xmin minmum x-coord bound
	* \param xmax maximum x-coord bound
	* \param ymin minmum y-coord bound
	* \param ymax maximum y-coord bound
	* \return true if value in bounds, otherwise, false
	*/
	template<typename T>
	static inline bool IsPtInBounds2D(const T& x, const T& y,
		const T& xmin, const T& xmax, const T& ymin, const T& ymax)
	{
		return IsInBounds(x, xmin, xmax) && IsInBounds(y, ymin, ymax);
	}


	//assign 3d rotation matrix
	//ang_value(in): value of rotation ang
	//axis (in): 0 = x - axis, 1 = y - axis, 2 = z - axis
	//rot_mat(out): 3 x 3 rotation matrix
	static inline ModuleStruct::CMat CreateRotationMat(const float ang_value, const int axis)
	{
		//error check
		if (axis < 0 || axis > 2) {
			assert(false);
		}

		ModuleStruct::CMat rot_mat = Util_CV::zeros<ModuleStruct::CMat>(3, 3, CV_32F);

		if (axis == 0) {
			rot_mat.at<float>(0, 0) = 1.f; rot_mat.at<float>(0, 1) = 0.f;            rot_mat.at<float>(0, 2) = 0.f;
			rot_mat.at<float>(1, 0) = 0.f; rot_mat.at<float>(1, 1) = cos(ang_value); rot_mat.at<float>(1, 2) = -sin(ang_value);
			rot_mat.at<float>(2, 0) = 0.f; rot_mat.at<float>(2, 1) = sin(ang_value); rot_mat.at<float>(2, 2) = cos(ang_value);
		}
		else if (axis == 1) {
			rot_mat.at<float>(0, 0) = cos(ang_value); rot_mat.at<float>(0, 1) = 0.f; rot_mat.at<float>(0, 2) = sin(ang_value);
			rot_mat.at<float>(1, 0) = 0.f;            rot_mat.at<float>(1, 1) = 1.f; rot_mat.at<float>(1, 2) = 0.f;
			rot_mat.at<float>(2, 0) = -sin(ang_value); rot_mat.at<float>(2, 1) = 0.f; rot_mat.at<float>(2, 2) = cos(ang_value);
		}
		else if (axis == 2) {
			rot_mat.at<float>(0, 0) = cos(ang_value); rot_mat.at<float>(0, 1) = -sin(ang_value); rot_mat.at<float>(0, 2) = 0.f;
			rot_mat.at<float>(1, 0) = sin(ang_value); rot_mat.at<float>(1, 1) = cos(ang_value); rot_mat.at<float>(1, 2) = 0.f;
			rot_mat.at<float>(2, 0) = 0.f;            rot_mat.at<float>(2, 1) = 0.f;             rot_mat.at<float>(2, 2) = 1.f;
		}

		return rot_mat;
	}

	/**
	* \brief Assign four-element rotation matrix
	*
	* ref: https://krasjet.github.io/quaternion/quaternion.pdf
	* @param source plane normal
	* @param destination plane normal,must axis of rotation of the unit vector
	* @return  3*3 rotation matrix  */

	static inline ModuleStruct::CMat CreateRotationMat4E(const ModuleStruct::Point3f src_plane_normal, const ModuleStruct::Point3f dst_plane_normal)
	{
		// find normalized rotation axis vector u: rotate nPlane to nz
		// crossproduct(nPlane, nz)
		ModuleStruct::Point3f u = src_plane_normal.cross(dst_plane_normal);
		u = Util_Math::vec3_normalize(u);
		// calculate rotation angle in radius, two vectors have unit length
		double theta_half = acosf(src_plane_normal.dot(dst_plane_normal)) / 2.0f;
		// calculate rotation quaternion (a, bi, cj, dk)
		double a = cos(theta_half);
		double b = sin(theta_half) * u.x;
		double c = sin(theta_half) * u.y;
		double d = sin(theta_half) * u.z;

		ModuleStruct::CMat rot_mat = Util_CV::zeros<ModuleStruct::CMat>(3, 3, CV_64FC1);
		rot_mat.at<float>(0, 0) = static_cast<float>(1 - 2 * c * c - 2 * d * d);
		rot_mat.at<float>(0, 1) = static_cast <float>(2 * b * c - 2 * a * d);
		rot_mat.at<float>(0, 2) = static_cast <float>(2 * a * c + 2 * b * d);
		rot_mat.at<float>(1, 0) = static_cast <float>(2 * b * c + 2 * a * d);
		rot_mat.at<float>(1, 1) = static_cast <float>(1 - 2 * b * b - 2 * d * d);
		rot_mat.at<float>(1, 2) = static_cast <float>(2 * c * d - 2 * a * b);
		rot_mat.at<float>(2, 0) = static_cast <float>(2 * b * d - 2 * a * c);
		rot_mat.at<float>(2, 1) = static_cast <float>(2 * a * b + 2 * c * d);
		rot_mat.at<float>(2, 2) = static_cast <float>(1 - 2 * b * b - 2 * c * c);

		return rot_mat;
	}

	//Eigen value , eigen vectors related
	/****** revised by TangXueyan @ 2018/11/21 *********/
	//static double ComputeDetermin(const double mat[6]);
	static inline double ComputeDetermin(const double* mat) {
		return (*mat) * ((*(mat + 3)) * (*(mat + 5)) - (*(mat + 4)) * (*(mat + 4)))
			- (*(mat + 1)) * ((*(mat + 1)) * (*(mat + 5)) - (*(mat + 2)) * (*(mat + 4)))
			+ (*(mat + 2)) * ((*(mat + 1)) * (*(mat + 4)) - (*(mat + 2)) * (*(mat + 3)));
	}

	//Distance related
	template<typename T>
	static inline T ComputePointToPointSquareDist(const T point_1[3], const T point_2[3]) {
		return static_cast<T>(pow(point_1[0] - point_2[0], 2.0)
			+ pow(point_1[1] - point_2[1], 2.0)
			+ pow(point_1[2] - point_2[2], 2.0));
	}

	template<typename OUT, typename T>
	static inline OUT ComputePointToPointSquareDist(const T point_1, const T point_2) {
		return static_cast<OUT>(pow(point_1.x - point_2.x, 2.0)
			+ pow(point_1.y - point_2.y, 2.0)
			+ pow(point_1.z - point_2.z, 2.0));
	}

	template<typename OUT, typename T>
	static inline OUT ComputePointToPointDist(const T point_1, const T point_2) {
		return static_cast<OUT>(sqrt(pow(point_1.x - point_2.x, 2.0)
			+ pow(point_1.y - point_2.y, 2.0)
			+ pow(point_1.z - point_2.z, 2.0)));
	}

	template<typename OUT, typename T>
	static inline OUT ComputePointToPointDist2D(const T point_1, const T point_2) {
		return static_cast<OUT>(sqrt(pow(point_1.x - point_2.x, 2.0)
			+ pow(point_1.y - point_2.y, 2.0)));
	}

	template<typename OUT, typename T>
	static inline OUT ComputePointToLineDist(const T pt, const T line_pt, const T line_dir) {

#if 1
		//TODO: make it better
		ModuleStruct::Point3f temp_pt, vec1, vec2, vec3;
		temp_pt = line_pt + line_dir;
		vec1 = temp_pt - line_pt;
		vec2 = pt - line_pt;

		OUT d = vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z;
		vec3 = d * line_dir;

		OUT dist = sqrt(pow(vec2.x - vec3.x, 2) + pow(vec2.y - vec3.y, 2) + pow(vec2.z - vec3.z, 2));
		return dist;
#else
		// Elaine Li: another method to compute point to line distance
		// Ref to: https://onlinemschool.com/math/library/analytic_geometry/p_line/
		Point3f vec1, vec2, vec3;
		vec1 = line_pt - pt;
		vec2.x = vec1.y * line_dir.z - vec1.z * line_dir.y;
		vec2.y = -(vec1.x * line_dir.z - vec1.z * line_dir.x);
		vec2.z = vec1.x * line_dir.y - vec1.y * line_dir.x;

		OUT d1 = sqrt(pow(vec2.x, 2) + pow(vec2.y, 2) + pow(vec2.z, 2));
		OUT d2 = sqrt(pow(line_dir.x, 2) + pow(line_dir.y, 2) + pow(line_dir.z, 2));
		OUT dist = d1 / d2;
		return dist;
#endif
	}

	template<typename T>
	static inline T ComputePointToPlaneDist(const T point[3], const T plane_center[3], const T plane_normals[3]) {
		return static_cast<T>(fabs(plane_normals[0] * (point[0] - plane_center[0])
			+ plane_normals[1] * (point[1] - plane_center[1])
			+ plane_normals[2] * (point[2] - plane_center[2])));
	}

	template<typename OUT, typename T>
	static inline OUT ComputePointToPlaneDist(const T pt, const T plane_normals, const T plane_center) {
		return static_cast<OUT>(fabs(plane_normals.x * (pt.x - plane_center.x) +
			plane_normals.y * (pt.y - plane_center.y) +
			plane_normals.z * (pt.z - plane_center.z)));
		return static_cast<OUT>(pt.x + plane_normals.x);
	}

	//vector related
	template<typename T>
	static inline T ComputeVectorDotProduct(const T vector_1[3], const T vector_2[3]) {
		float dot_product = vector_1[0] * vector_2[0] + vector_1[1] * vector_2[1] + vector_1[2] * vector_2[2];
		return dot_product;
	}

	template<typename OUT, typename T>
	static inline OUT ComputeVectorDotProduct(const T vector_1, const T vector_2) {
		return  static_cast<OUT>(vector_1.x * vector_2.x + vector_1.y * vector_2.y + vector_1.z * vector_2.z);
	}

	template<typename T>
	static inline T ComputeVectorCrossProduct(const T vector_1, const T vector_2) {
		T cross;
		cross.x = vector_1.y * vector_2.z - vector_1.z * vector_2.y;
		cross.y = vector_1.z * vector_2.x - vector_1.x * vector_2.z;
		cross.z = vector_1.x * vector_2.y - vector_1.y * vector_2.x;
		return cross;
	}

	template<typename PT, typename DT, typename MT>
	static inline void rot(PT &input, DT data_type, MT rot_mat) {
		input = PT(
			rot_mat.template at<DT>(0, 0) * input.x + rot_mat.template at<DT>(0, 1) * input.y + rot_mat.template at<DT>(0, 2) * input.z,
			rot_mat.template at<DT>(1, 0) * input.x + rot_mat.template at<DT>(1, 1) * input.y + rot_mat.template at<DT>(1, 2) * input.z,
			rot_mat.template at<DT>(2, 0) * input.x + rot_mat.template at<DT>(2, 1) * input.y + rot_mat.template at<DT>(2, 2) * input.z);
	}

	template<typename PT, typename DT>
	static inline bool GetMinMaxXYZ(const vector<PT>&input_data, PT& max_p, PT &min_p, DT dt)
	{
		if (input_data.size() == 0) {
			//log_error("empty input");
			return false;
		}
		min_p.x = min_p.y = min_p.z = std::numeric_limits<DT>::infinity();
		max_p.x = max_p.y = max_p.z = -std::numeric_limits<DT>::infinity();

		for (unsigned int i = 0; i < input_data.size(); i++) {

			min_p.x = (min_p.x > input_data[i].x) ? input_data[i].x : min_p.x;
			max_p.x = (max_p.x < input_data[i].x) ? input_data[i].x : max_p.x;
			min_p.y = (min_p.y > input_data[i].y) ? input_data[i].y : min_p.y;
			max_p.y = (max_p.y < input_data[i].y) ? input_data[i].y : max_p.y;
			min_p.z = (min_p.z > input_data[i].z) ? input_data[i].z : min_p.z;
			max_p.z = (max_p.z < input_data[i].z) ? input_data[i].z : max_p.z;
		}
		return true;
	}

	template<typename PT, typename DT, typename  T>
	static inline bool GetMinMaxXYZ(const vector<PT>&input_data , const vector<T>&input_ids, PT& max_p, PT& min_p, DT dt)
	{
		if ((input_ids).size() == 0) {
			//log_error("empty input");
			return false;
		}
		min_p.x = min_p.y = min_p.z = std::numeric_limits<DT>::infinity();
		max_p.x = max_p.y = max_p.z = -std::numeric_limits<DT>::infinity();

		for (size_t i = 0; i < (input_ids).size(); i++) {

			min_p.x = (min_p.x > (input_data)[(input_ids)[i]].x) ? (input_data)[(input_ids)[i]].x : min_p.x;
			max_p.x = (max_p.x < (input_data)[(input_ids)[i]].x) ? (input_data)[(input_ids)[i]].x : max_p.x;
			min_p.y = (min_p.y > (input_data)[(input_ids)[i]].y) ? (input_data)[(input_ids)[i]].y : min_p.y;
			max_p.y = (max_p.y < (input_data)[(input_ids)[i]].y) ? (input_data)[(input_ids)[i]].y : max_p.y;
			min_p.z = (min_p.z > (input_data)[(input_ids)[i]].z) ? (input_data)[(input_ids)[i]].z : min_p.z;
			max_p.z = (max_p.z < (input_data)[(input_ids)[i]].z) ? (input_data)[(input_ids)[i]].z : max_p.z;
		}
		return true;
	}

}

#endif // !_UTIL_MATH_H_
