#ifndef MATHOPERATION_H
#define MATHOPERATION_H
#define _USE_MATH_DEFINES

//dependency
//suggested opencv version: 3.4.1
#include <opencv/cv.hpp>
#include <math.h>
#include "bbox.h"

class MathOperation {
public:

	//assign 3d rotation matrix
	//ang_value(in): value of rotation ang
	//axis (in): 0 = x - axis, 1 = y - axis, 2 = z - axis
	//rot_mat(out): 3 x 3 rotation matrix
	static cv::Mat CreateRotationMat(const float ang_value, const int axis);

	//Eigen value , eigen vectors related
	/****** revised by TangXueyan @ 2018/11/21 *********/
	//static double ComputeDetermin(const double mat[6]);
	static double ComputeDetermin(const double *mat);

	//static void ComputeEigenValue(const double cov_mat[6], double *eig_val);
	static void ComputeEigenValue(const double *cov_mat, double *eig_val);

	//static void ComputeEigenVector(const double cov_mat[6], const double eig_val[3], double *eig_vec);
	static void ComputeEigenVector(const double *cov_mat, const double *eig_val, double *eig_vec);

	//static void ComputeFastEigen(const double cov_mat[6], double *eig_val, double *eig_vec);
	static void ComputeFastEigen(const double *cov_mat, double *eig_val, double *eig_vec);

	//static void ComputeFastEigenParallel(const double cov_mat[6], double *eig_val, double *eig_vec);
	static void ComputeFastEigenParallel(const double *cov_mat, double *eig_val, double *eig_vec);

	//Distance related
	static float ComputePointToPointSquareDist(const float point_1[3], const float point_2[3]);

	static float ComputePointToPointDist(const cv::Point3f point_1, const cv::Point3f point_2);

	static float ComputePointToPointSquareDist(const cv::Point3f point_1, const cv::Point3f point_2);

	static float ComputePointToPointSquareDist2D(const cv::Point point_1, const cv::Point point_2);

	static float ComputePointToPointDist2D(const cv::Point point_1, const cv::Point point_2);

	static float ComputePointToLineDist(const cv::Point3f pt, const cv::Point3f line_pt, const cv::Point3f line_dir);

	static float ComputePointToPlaneDist(const float point[3], const float plane_center[3], const float plane_normal[3]);

	static float ComputePointToPlaneDist(const cv::Point3f pt, const cv::Point3f plane_normals, const cv::Point3f plane_center);

	static float ComputeTriangleHypotenuse(float x, float y);

	//Add by Simon.jin
	static float PointToLinesegDist(cv::Point3f pt3, cv::Point3f pt3A, cv::Point3f pt3B, bool onLineOnly = true);
	static float PointToLinesegDist(cv::Point P, cv::Point A, cv::Point B);

	//vector related
	static float ComputeVectorDotProduct(const float vector_1[3], const float vector_2[3]);

	static float ComputeVectorDotProduct(const cv::Point3f vector_1, const cv::Point3f vector_2);

	static float ComputeVectorDotProduct(const cv::Point  vector_1, const cv::Point  vector_2);
	//cross product
	static std::vector<float> ComputeVectorCrossProduct(const float vector_1[3], const float vector_2[3]);

	static std::vector<float> ComputeVectorCrossProduct(const cv::Point3f vector_1, const cv::Point3f vector_2);

	static float  ComputeVectorCrossProduct(const cv::Point  vector_1, const cv::Point  vector_2);
	//rotate matrix as normals
	static cv::Mat CreateRotationMatAsNormals(const float normalSrc[3], const float normalDst[3]);

	static cv::Mat CreateRotationMatAsNormals(const cv::Point3f normalSrc, const cv::Point3f normalDst);
	//normalize
	static float VectorNormalize(const float v[3]);
	static float VectorNormalize(const cv::Point3f v);
	static float VectorNormalize(const cv::Point v);
	//find intersect plane point of one pts outside along plane normal in plane
	static cv::Point3f  CalPointIntersectPlanePoint(const cv::Point3f pointOutPlane,
		const float planeNormal[3],
		const cv::Point3f planePoint);

	//find intersect point p of one point ouside along line normal on Line, and distance to line dist, so return <p, dist>
	static std::pair<cv::Point3f, float> CalPointIntersectLine(const cv::Point3f pointOutLine,
		const cv::Point3f pointALine,
		const cv::Point3f pointBLine);

	//if point inside the circle
	static bool IsPointInCircle(const  cv::Point3f p, const  cv::Point3f center, const  float radius);

	//added by yu.liang
	static cv::Point3f CalPlaneLineIntersectPoint(const cv::Point3f pt1, const cv::Point3f pt2,
		const cv::Point3f planeNormal, const cv::Point3f planePoint);

	static bool CalPlaneLineIntersectPoint_LineNoraml(const cv::Point3f lineNormal, const cv::Point3f linePoint,
		const cv::Point3f planeNormal, const cv::Point3f planePoint, cv::Point3f &resultPoint);

	//if point inside the box at XOY plane
	static bool IsPointInBoxXOY(const cv::Point3f & p, const BBox3D & box, float dilateRadius);

	//static void getNewNormalAndCenter(const std::vector<cv::Point3f> &pts, cv::Point3f& center, cv::Point3f& normal);
	static void getNewNormalAndCenter(std::vector<cv::Point3f> &pts,
		cv::Point3f& center, cv::Point3f& normal,
		std::vector<cv::Point3f> &fitplane);

	static void CalPointOnPlaneProjection(cv::Point3f& pt, const cv::Point3f plane_normals, const cv::Point3f plane_center);
	static float CalLine2LineAngle(const cv::Point L1_A, const cv::Point L1_B, const cv::Point L2_A, const cv::Point L2_B);

	//astri
	/**
	* \brief  compare according to multiple conditions, large_result and equal_result are the result of  five same pairs elements
	* only when the first pair element of five equal_results is true, the second elements of large_result is considered, and only if large_result
	* is true , the update_flag is set.  If all the equal_result elements is true,  return false
	* @param bool large_result[5],input bool type,show the compare result of whether  five  pairs elements are larger than the second five parameters or not
	* @param bool equal_result[5],input bool parameters,show the compare result of whether first five  parameters are equal to the second five parameters or not
	* @param update_flag  output, if the second
	* @return if success or failed for compare result
	*/
	static inline bool CompareByMultipleConditions(bool large_result[5], bool equal_result[5], unsigned char& update_flag)
	{
		if (large_result[0] == true) {

			update_flag = 1;
			return true;
		}
		else if (equal_result[0] == false) {

			return true;
		}

		if (large_result[1] == true) {

			update_flag = 1;
			return true;
		}
		else if (equal_result[1] == false) {

			return true;
		}

		if (large_result[2] == true) {

			update_flag = 1;
			return true;
		}
		else if (equal_result[2] == false) {

			return true;
		}

		if (large_result[3] == true) {

			update_flag = 1;
			return true;
		}
		else if (equal_result[3] == false) {

			return true;
		}


		if (large_result[4] == true) {

			update_flag = 1;
			return true;
		}
		else if (equal_result[4] == false) {

			return true;
		}
		else
		{
			return false;
		}
	}

	static inline cv::Point3f point_rot(cv::Mat rot_mat, cv::Point3f& pt)
	{
		cv::Point3f c_pt = cv::Point3f(
			rot_mat.at<float>(0, 0) * pt.x + rot_mat.at<float>(0, 1) * pt.y + rot_mat.at<float>(0, 2) * pt.z,
			rot_mat.at<float>(1, 0) * pt.x + rot_mat.at<float>(1, 1) * pt.y + rot_mat.at<float>(1, 2) * pt.z,
			rot_mat.at<float>(2, 0) * pt.x + rot_mat.at<float>(2, 1) * pt.y + rot_mat.at<float>(2, 2) * pt.z);
		return c_pt;
	}

	static inline std::vector<cv::Point3f> plane_rot(cv::Mat mat_r, const std::vector<cv::Point3f> &pts) {
		// data validation
		if (mat_r.rows != 3 || mat_r.cols != 3 || pts.empty()) return std::vector<cv::Point3f>();

		std::vector<std::vector<float>> mat_clbr_v(3, std::vector<float>(3));
		for (size_t i = 0; i < 3; i++)
			for (size_t j = 0; j < 3; j++)
				mat_clbr_v[i][j] = mat_r.ptr<float>(i)[j];

		size_t npts = pts.size();
		std::vector<cv::Point3f> rotated_pts(npts);
#pragma omp parallel for
		for (int i = 0; i < npts; i++) {
			rotated_pts[i].x = mat_clbr_v[0][0] * pts[i].x + mat_clbr_v[0][1] * pts[i].y + mat_clbr_v[0][2] * pts[i].z;
			rotated_pts[i].y = mat_clbr_v[1][0] * pts[i].x + mat_clbr_v[1][1] * pts[i].y + mat_clbr_v[1][2] * pts[i].z;
			rotated_pts[i].z = mat_clbr_v[2][0] * pts[i].x + mat_clbr_v[2][1] * pts[i].y + mat_clbr_v[2][2] * pts[i].z;
		}
		return rotated_pts;
	}

	static inline std::vector<std::vector<cv::Point3f>> planes_rot(cv::Mat rot_mat, std::vector<std::vector<cv::Point3f>>& pts)
	{
		if (pts.empty())
			return  std::vector<std::vector<cv::Point3f>>();

		std::vector<std::vector<cv::Point3f>> rotated_pts(pts.size());

		for (int i = 0; i < rotated_pts.size(); i++)
		{
			rotated_pts[i] = plane_rot(rot_mat, pts[i]);
		}

		return rotated_pts;
	}
};

#endif //MATHOPERATION_H
