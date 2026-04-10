#pragma once
#include "MathOperation.h"
#include "../CutScanPlane/utility/util_pca.h"
#include "../Measurement/Measurebase.h"
//#include "../Alignment/CloudSampling.h"
#include "Voxel.h"
#include <time.h>
//assign 3d rotation matrix
//ang_value(in): value of rotation ang
//axis (in): 0 = x - axis, 1 = y - axis, 2 = z - axis
//rot_mat(out): 3 x 3 rotation matrix
cv::Mat MathOperation::CreateRotationMat(const float ang_value, const int axis)
{
	std::string err_message = "error free";

	//error check
	if (axis < 0 || axis > 2)
		throw err_message = "MathOperation::CreateRotationMat: wrong axis input";

	cv::Mat rot_mat = cv::Mat::zeros(3, 3, CV_32F);

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

double MathOperation::ComputeDetermin(const double *mat) {
	double det_mat = (*mat) * ((*(mat + 3)) * (*(mat + 5)) - (*(mat + 4)) * (*(mat + 4)))
		- (*(mat + 1)) * ((*(mat + 1)) * (*(mat + 5)) - (*(mat + 2)) * (*(mat + 4)))
		+ (*(mat + 2)) * ((*(mat + 1)) * (*(mat + 4)) - (*(mat + 2)) * (*(mat + 3)));

	return det_mat;
}

void MathOperation::ComputeEigenValue(const double *cov_mat, double *eig_val) {
	double p1 = std::pow(*(cov_mat + 1), 2) + std::pow(*(cov_mat + 2), 2) + std::pow(*(cov_mat + 4), 2);

	if (p1 == 0) {
		*(eig_val + 2) = (*(cov_mat + 3) < *(cov_mat + 5)) ? *(cov_mat + 3) : *(cov_mat + 5);
		*(eig_val + 2) = (*cov_mat < *(eig_val + 2)) ? *(cov_mat + 3) : *(cov_mat + 5);

		*eig_val = (*(cov_mat + 3) > *(cov_mat + 5)) ? *(cov_mat + 3) : *(cov_mat + 5);
		*eig_val = (*cov_mat > *eig_val) ? *(cov_mat + 3) : *(cov_mat + 5);

		double q = (*cov_mat + *(cov_mat + 3) + *(cov_mat + 5));
		*(eig_val + 1) = q - *eig_val - *(eig_val + 2);
	}
	else {
		double q = (*cov_mat + *(cov_mat + 3) + *(cov_mat + 5)) / 3.0;
		double p2 = std::pow(*cov_mat - q, 2)
			+ std::pow(*(cov_mat + 3) - q, 2)
			+ std::pow(*(cov_mat + 5) - q, 2)
			+ 2 * p1;
		double p = std::sqrt(p2 / 6.0);

		double cov_mat_copy[6];// (6, CV_64F);
		std::memcpy(&cov_mat_copy[0], cov_mat, sizeof(cov_mat_copy));

		cov_mat_copy[0] -= q; cov_mat_copy[3] -= q; cov_mat_copy[5] -= q;
		double det_cov_mat_copy = ComputeDetermin(&cov_mat_copy[0]);//determinant(B);
		det_cov_mat_copy /= std::pow(p, 3);

		double r = det_cov_mat_copy / 2.0;//double r = cv::determinant(B) / 2.0;

		double phi;
		if (r <= -1.0) {
			phi = M_PI / 3.0;
		}
		else if (r >= 1.0) {
			phi = 0.0;
		}
		else {
			phi = std::acos(r) / 3.0;
		}
		*eig_val = q + 2.0 * p * std::cos(phi);
		*(eig_val + 2) = q + 2.0 * p * std::cos(phi + 2.0 * M_PI / 3.0);
		*(eig_val + 1) = q * 3.0 - *eig_val - *(eig_val + 2);
	}
}

void MathOperation::ComputeEigenVector(const double *cov_mat, const double *eig_val, double *eig_vec) {
	double temp_eig = *cov_mat - *(eig_val + 1);
	*eig_vec = (*cov_mat - *eig_val) * temp_eig + (*(cov_mat + 1)) * (*(cov_mat + 1)) + (*(cov_mat + 2)) * (*(cov_mat + 2));
	*(eig_vec + 1) = (temp_eig + *(cov_mat + 3) - *eig_val) * (*(cov_mat + 1)) + (*(cov_mat + 4)) * (*(cov_mat + 2));
	*(eig_vec + 2) = (*(cov_mat + 4)) * (*(cov_mat + 1)) + (temp_eig + *(cov_mat + 5) - *eig_val) * (*(cov_mat + 2));

	double len = (std::sqrt(std::pow(*eig_vec, 2) + std::pow(*(eig_vec + 1), 2) + std::pow(*(eig_vec + 2), 2)));

	if (len > 0) {
		for (int i = 0; i < 3; i++)
			*(eig_vec + i) /= len;
	}
	else {
		//throw err_message = "MathOperation::ComputeEigenVector: Not able to calculate";
	}
	return;
}

void MathOperation::ComputeFastEigen(const double *cov_mat, double *eig_val, double *eig_vec) {
	// Based on:

	// https://en.wikipedia.org/wiki/Eigenvalue_algorithm#3.C3.973_matrices
	double min_val_threshold = 1e-3;

	for (int i = 0; i < 3; i++)
		*(eig_vec + i) = 0;

	double a0sq = std::pow(*cov_mat, 2);
	double a1sq = std::pow(*(cov_mat + 1), 2);
	double a2sq = std::pow(*(cov_mat + 2), 2);
	double a3sq = std::pow(*(cov_mat + 3), 2);
	double a4sq = std::pow(*(cov_mat + 4), 2);
	double a5sq = std::pow(*(cov_mat + 5), 2);
	double cov_mat_col0 = std::sqrt(a0sq + a1sq + a2sq);
	double cov_mat_col1 = std::sqrt(a1sq + a3sq + a4sq);
	double cov_mat_col2 = std::sqrt(a2sq + a4sq + a5sq);

	if (cov_mat_col0 > min_val_threshold && cov_mat_col1 > min_val_threshold && cov_mat_col2 > min_val_threshold) {
		ComputeEigenValue(cov_mat, eig_val);

		ComputeEigenVector(cov_mat, eig_val, eig_vec);
	}
	else {
		if (cov_mat_col2 <= min_val_threshold)
			*(eig_vec + 2) = 1.0;
		else {
			if (cov_mat_col1 <= min_val_threshold)
				*(eig_vec + 1) = 1.0;
			else {
				if (cov_mat_col0 <= min_val_threshold)
					*eig_vec = 1.0;
			}
		}
	}
	return;
}

void MathOperation::ComputeFastEigenParallel(const double *cov_mat, double *eig_val, double *eig_vec) {
	// Based on:
	// https://en.wikipedia.org/wiki/Eigenvalue_algorithm#3.C3.973_matrices
	double min_val_threshold = 1e-3;

	for (int i = 0; i < 3; i++)
		*(eig_vec + i) = 0;

	double a0sq = std::pow(*cov_mat, 2);
	double a1sq = std::pow(*(cov_mat + 1), 2);
	double a2sq = std::pow(*(cov_mat + 2), 2);
	double a3sq = std::pow(*(cov_mat + 3), 2);
	double a4sq = std::pow(*(cov_mat + 4), 2);
	double a5sq = std::pow(*(cov_mat + 5), 2);
	double cov_mat_col0 = std::sqrt(a0sq + a1sq + a2sq);
	double cov_mat_col1 = std::sqrt(a1sq + a3sq + a4sq);
	double cov_mat_col2 = std::sqrt(a2sq + a4sq + a5sq);

	if (cov_mat_col0 > min_val_threshold && cov_mat_col1 > min_val_threshold && cov_mat_col2 > min_val_threshold) {
		/********** compute eigen values *************/
		double p1 = a1sq + a2sq + a4sq;
		if (p1 == 0) {
			*(eig_val + 2) = (*(cov_mat + 3) < *(cov_mat + 5)) ? *(cov_mat + 3) : *(cov_mat + 5);
			*(eig_val + 2) = (*cov_mat < *(eig_val + 2)) ? *(cov_mat + 3) : *(cov_mat + 5);

			*eig_val = (*(cov_mat + 3) > *(cov_mat + 5)) ? *(cov_mat + 3) : *(cov_mat + 5);
			*eig_val = (*cov_mat > *eig_val) ? *(cov_mat + 3) : *(cov_mat + 5);

			double q = *cov_mat + *(cov_mat + 3) + *(cov_mat + 5);
			*(eig_val + 1) = q - *eig_val - *(eig_val + 2);
		}
		else {
			double q = (*cov_mat + *(cov_mat + 3) + *(cov_mat + 5)) / 3.0;
			double p2 = std::pow(*cov_mat - q, 2)
				+ std::pow(*(cov_mat + 3) - q, 2)
				+ std::pow(*(cov_mat + 5) - q, 2)
				+ 2 * p1;
			double p = std::sqrt(p2 / 6.0);

			double cov_mat_copy[6];// (6, CV_64F);
			std::memcpy(&cov_mat_copy[0], cov_mat, sizeof(cov_mat_copy));

			cov_mat_copy[0] -= q; cov_mat_copy[3] -= q; cov_mat_copy[5] -= q;
			double det_cov_mat_copy = cov_mat_copy[0] * (cov_mat_copy[3] * cov_mat_copy[5] - cov_mat_copy[4] * cov_mat_copy[4])
				- cov_mat_copy[1] * (cov_mat_copy[1] * cov_mat_copy[5] - cov_mat_copy[2] * cov_mat_copy[4])
				+ cov_mat_copy[2] * (cov_mat_copy[1] * cov_mat_copy[4] - cov_mat_copy[2] * cov_mat_copy[3]);//determinant(B);
			det_cov_mat_copy /= std::pow(p, 3);

			double r = det_cov_mat_copy / 2.0;//double r = cv::determinant(B) / 2.0;

			double phi;
			if (r <= -1.0) {
				phi = M_PI / 3.0;
			}
			else if (r >= 1.0) {
				phi = 0.0;
			}
			else {
				phi = std::acos(r) / 3.0;
			}
			*eig_val = q + 2.0 * p * std::cos(phi);
			*(eig_val + 2) = q + 2.0 * p * std::cos(phi + 2.0 * M_PI / 3.0);
			*(eig_val + 1) = q * 3.0 - *eig_val - *(eig_val + 2);
		}
		/********** End of compute eigen values *************/

		/********** compute eigen vectors ***********/
		double temp_eig = *cov_mat - *(eig_val + 1);
		*eig_vec = (*cov_mat - *eig_val) * temp_eig + (*(cov_mat + 1)) * (*(cov_mat + 1)) + (*(cov_mat + 2)) * (*(cov_mat + 2));
		*(eig_vec + 1) = (temp_eig + *(cov_mat + 3) - *eig_val) * (*(cov_mat + 1)) + (*(cov_mat + 4)) * (*(cov_mat + 2));
		*(eig_vec + 2) = (*(cov_mat + 4)) * (*(cov_mat + 1)) + (temp_eig + *(cov_mat + 5) - *eig_val) * (*(cov_mat + 2));

		double len = (std::sqrt(std::pow(*eig_vec, 2) + std::pow(*(eig_vec + 1), 2) + std::pow(*(eig_vec + 2), 2)));

		if (len > 0) {
			for (int i = 0; i < 3; i++)
				*(eig_vec + i) /= len;
		}
		else {
			//throw err_message = "MathOperation::ComputeFastEigen: EigenVector can not able to calculate";
		}
		/********** End of compute eigen vectors ***********/
	}
	else {
		if (cov_mat_col2 <= min_val_threshold)
			*(eig_vec + 2) = 1.0;
		else {
			if (cov_mat_col1 <= min_val_threshold)
				*(eig_vec + 1) = 1.0;
			else {
				if (cov_mat_col0 <= min_val_threshold)
					*eig_vec = 1.0;
			}
		}
	}

	return;
}

float MathOperation::ComputePointToPointSquareDist(const float point_1[3], const float point_2[3]) {
	float point_to_point_dist;

	point_to_point_dist = std::pow(point_1[0] - point_2[0], 2.0)
		+ std::pow(point_1[1] - point_2[1], 2.0)
		+ std::pow(point_1[2] - point_2[2], 2.0);

	return point_to_point_dist;
}

float MathOperation::ComputePointToPointSquareDist(const cv::Point3f point_1, const cv::Point3f point_2) {
	float point_to_point_dist;

	point_to_point_dist = std::pow(point_1.x - point_2.x, 2.0)
		+ std::pow(point_1.y - point_2.y, 2.0)
		+ std::pow(point_1.z - point_2.z, 2.0);

	return point_to_point_dist;
}

float MathOperation::ComputePointToPointSquareDist2D(const cv::Point point_1, const cv::Point point_2) {
	float point_to_point_dist;

	point_to_point_dist = std::pow(point_1.x - point_2.x, 2.0)
		+ std::pow(point_1.y - point_2.y, 2.0);

	return point_to_point_dist;
}

float MathOperation::ComputePointToPointDist(const cv::Point3f point_1, const cv::Point3f point_2) {
	float point_to_point_dist;

	point_to_point_dist = std::pow(point_1.x - point_2.x, 2.0)
		+ std::pow(point_1.y - point_2.y, 2.0)
		+ std::pow(point_1.z - point_2.z, 2.0);

	return sqrt(point_to_point_dist);
}

float MathOperation::ComputePointToPointDist2D(const cv::Point point_1, const cv::Point point_2) {
	float point_to_point_dist;

	point_to_point_dist = std::pow(point_1.x - point_2.x, 2.0)
		+ std::pow(point_1.y - point_2.y, 2.0);

	return sqrt(point_to_point_dist);
}
float MathOperation::ComputePointToLineDist(const cv::Point3f pt, const cv::Point3f line_pt, const cv::Point3f line_dir) {
	//TODO: make it better
	cv::Point3f temp_pt, vec1, vec2, vec3;
	temp_pt = line_pt + line_dir;
	vec1 = temp_pt - line_pt;
	vec2 = pt - line_pt;

	float d = vec1.x*vec2.x + vec1.y*vec2.y + vec1.z*vec2.z;
	vec3 = d * line_dir;

	float dist = std::sqrt(std::pow(vec2.x - vec3.x, 2) + std::pow(vec2.y - vec3.y, 2) + std::pow(vec2.z - vec3.z, 2));

	return dist;
}

//Add by simon.jin start
float MathOperation::PointToLinesegDist(cv::Point3f pt3, cv::Point3f pt3A, cv::Point3f pt3B, bool onLineOnly)
{
	cv::Point2f pt(pt3.x, pt3.y);
	cv::Point2f ptA(pt3A.x, pt3A.y);
	cv::Point2f ptB(pt3B.x, pt3B.y);

	cv::Point2f ptAP = ptA - pt;
	cv::Point2f ptAB = ptA - ptB;
	double dot = ptAP.dot(ptAB);
	if (dot < 0)
	{
		if (onLineOnly)
			return INFINITY;
		else
			return cv::norm(ptAP);
	}

	double len_AB = ptAB.x * ptAB.x + ptAB.y * ptAB.y;

	if (dot > len_AB)
	{
		if (onLineOnly)
			return INFINITY;
		else
			return cv::norm(ptB - pt);
	}

	double r = dot / len_AB;
	double px = ptA.x + (ptB.x - ptA.x) * r;
	double py = ptA.y + (ptB.y - ptA.y) * r;
	return std::sqrt((pt.x - px) * (pt.x - px) + (py - pt.y) * (py - pt.y));
}
//add by simon end

float MathOperation::PointToLinesegDist(cv::Point P, cv::Point A, cv::Point B)
{
	//计算r |AB| |AP| |BP| |PC|

	double ab = sqrt(pow((B.x - A.x), 2) + pow((B.y - A.y), 2)); // |AB|
	double ap = sqrt(pow((P.x - A.x), 2) + pow((P.y - A.y), 2)); // |AP|
	double bp = sqrt(pow((P.x - B.x), 2) + pow((P.y - B.y), 2)); // |BP|
	double r = 0;
	if (ab > 0)
	{
		r = ((P.x - A.x)*(B.x - A.x) + (P.y - A.y)*(B.y - A.y)) / pow(ab, 2);
	} //r
	else
	{
		cout << "no lines" << endl;
	}

	//double distance = 0;
	double distance = 0;
	if (ab > 0)
	{
		if (r >= 1)
			distance = bp;
		else if (r > 0 && r < 1)
			distance = sqrt(pow(ap, 2) - r * r*pow(ab, 2));
		else
			distance = ap;
	}

	return distance;
}

float MathOperation::ComputePointToPlaneDist(const float point[3], const float plane_center[3], const float plane_normals[3]) {
	float point_to_plane_dist;

	point_to_plane_dist = fabs(plane_normals[0] * (point[0] - plane_center[0])
		+ plane_normals[1] * (point[1] - plane_center[1])
		+ plane_normals[2] * (point[2] - plane_center[2]));

	return point_to_plane_dist;
}

float MathOperation::ComputePointToPlaneDist(const cv::Point3f pt, const cv::Point3f plane_normals, const cv::Point3f plane_center) {
	return std::fabs(plane_normals.x * (pt.x - plane_center.x) +
		plane_normals.y * (pt.y - plane_center.y) +
		plane_normals.z * (pt.z - plane_center.z));
}

float MathOperation::ComputeTriangleHypotenuse(float x, float y)
{
	return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
}

float MathOperation::ComputeVectorDotProduct(const float vector_1[3], const float vector_2[3]) {
	float dot_product = vector_1[0] * vector_2[0] + vector_1[1] * vector_2[1] + vector_1[2] * vector_2[2];
	return dot_product;
}

float MathOperation::ComputeVectorDotProduct(const cv::Point3f vector_1, const cv::Point3f vector_2) {
	float dot_product = vector_1.x * vector_2.x + vector_1.y * vector_2.y + vector_1.z * vector_2.z;
	return dot_product;
}
float MathOperation::ComputeVectorDotProduct(const cv::Point  vector_1, const cv::Point  vector_2) {
	return (vector_1.x * vector_2.x + vector_1.y * vector_2.y);
}
std::vector<float> MathOperation::ComputeVectorCrossProduct(const float vector_1[3], const float vector_2[3])
{
	std::vector<float> out(3);
	out[0] = vector_1[1] * vector_2[2] - vector_1[2] * vector_2[1];
	out[1] = vector_1[2] * vector_2[0] - vector_1[0] * vector_2[2];
	out[2] = vector_1[0] * vector_2[1] - vector_1[1] * vector_2[0];

	return out;
}

std::vector<float> MathOperation::ComputeVectorCrossProduct(const cv::Point3f vector_1, const cv::Point3f vector_2)
{
	std::vector<float> out(3);
	out[0] = vector_1.y * vector_2.z - vector_1.z * vector_2.y;
	out[1] = vector_1.z * vector_2.x - vector_1.x * vector_2.z;
	out[2] = vector_1.x * vector_2.y - vector_1.y * vector_2.x;

	return out;
}

float MathOperation::ComputeVectorCrossProduct(const cv::Point  vector_1, const cv::Point  vector_2)
{
	return (vector_1.x * vector_2.y - vector_1.y * vector_2.x);
}
float MathOperation::VectorNormalize(const float v[3])
{
	return std::sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

float MathOperation::VectorNormalize(const cv::Point3f v)
{
	return std::sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

float MathOperation::VectorNormalize(const cv::Point v)
{
	return std::sqrtf(v.x * v.x + v.y * v.y);
}
cv::Mat MathOperation::CreateRotationMatAsNormals(const float normalSrc[3], const float normalDst[3])
{
	cv::Mat rot_mat = cv::Mat::zeros(3, 3, CV_32F);
	std::vector<float> rotationAxis = MathOperation::ComputeVectorCrossProduct(normalSrc, normalDst);
	float angle = std::acosf(MathOperation::ComputeVectorDotProduct(normalSrc, normalDst) / MathOperation::VectorNormalize(normalSrc) / MathOperation::VectorNormalize(normalDst));

	double norm = MathOperation::VectorNormalize(rotationAxis.data());
	if (norm > 0.00001f)
	{
		float inv_norm = 1.f / norm;
		rotationAxis[0] = rotationAxis[0] * inv_norm;
		rotationAxis[1] = rotationAxis[1] * inv_norm;
		rotationAxis[2] = rotationAxis[2] * inv_norm;
	}
	else {
		angle = 0.f;
	}

	float sin_value = std::sinf(angle);
	float cos_value = std::cosf(angle);
	rot_mat.at<float>(0, 0) = cos_value + rotationAxis[0] * rotationAxis[0] * (1 - cos_value);
	rot_mat.at<float>(0, 1) = rotationAxis[0] * rotationAxis[1] * (1 - std::cosf(angle)) - rotationAxis[2] * sin_value;
	rot_mat.at<float>(0, 2) = rotationAxis[1] * sin_value + rotationAxis[0] * rotationAxis[2] * (1 - cos_value);

	rot_mat.at<float>(1, 0) = rotationAxis[2] * sin_value + rotationAxis[0] * rotationAxis[1] * (1 - cos_value);
	rot_mat.at<float>(1, 1) = std::cosf(angle) + rotationAxis[1] * rotationAxis[1] * (1 - cos_value);
	rot_mat.at<float>(1, 2) = rotationAxis[1] * rotationAxis[2] * (1 - cos_value) - rotationAxis[0] * sin_value;

	rot_mat.at<float>(2, 0) = rotationAxis[0] * rotationAxis[2] * (1 - cos_value) - rotationAxis[1] * sin_value;
	rot_mat.at<float>(2, 1) = rotationAxis[0] * sin_value + rotationAxis[1] * rotationAxis[2] * (1 - cos_value);
	rot_mat.at<float>(2, 2) = cos_value + rotationAxis[2] * rotationAxis[2] * (1 - cos_value);

	return rot_mat;
}

cv::Mat MathOperation::CreateRotationMatAsNormals(const cv::Point3f normalSrc, const cv::Point3f normalDst)
{
	cv::Mat rot_mat = cv::Mat::zeros(3, 3, CV_32F);
	std::vector<float> rotationAxis = MathOperation::ComputeVectorCrossProduct(normalSrc, normalDst);
	float angle = std::acosf(MathOperation::ComputeVectorDotProduct(normalSrc, normalDst) / MathOperation::VectorNormalize(normalSrc) / MathOperation::VectorNormalize(normalDst));

	double norm = MathOperation::VectorNormalize(rotationAxis.data());
	if (norm > 0.00001f)
	{
		float inv_norm = 1.f / norm;
		rotationAxis[0] = rotationAxis[0] * inv_norm;
		rotationAxis[1] = rotationAxis[1] * inv_norm;
		rotationAxis[2] = rotationAxis[2] * inv_norm;
	}
	else {
		angle = 0.f;
	}

	float sin_value = std::sinf(angle);
	float cos_value = std::cosf(angle);
	rot_mat.at<float>(0, 0) = cos_value + rotationAxis[0] * rotationAxis[0] * (1 - cos_value);
	rot_mat.at<float>(0, 1) = rotationAxis[0] * rotationAxis[1] * (1 - std::cosf(angle)) - rotationAxis[2] * sin_value;
	rot_mat.at<float>(0, 2) = rotationAxis[1] * sin_value + rotationAxis[0] * rotationAxis[2] * (1 - cos_value);

	rot_mat.at<float>(1, 0) = rotationAxis[2] * sin_value + rotationAxis[0] * rotationAxis[1] * (1 - cos_value);
	rot_mat.at<float>(1, 1) = std::cosf(angle) + rotationAxis[1] * rotationAxis[1] * (1 - cos_value);
	rot_mat.at<float>(1, 2) = rotationAxis[1] * rotationAxis[2] * (1 - cos_value) - rotationAxis[0] * sin_value;

	rot_mat.at<float>(2, 0) = rotationAxis[0] * rotationAxis[2] * (1 - cos_value) - rotationAxis[1] * sin_value;
	rot_mat.at<float>(2, 1) = rotationAxis[0] * sin_value + rotationAxis[1] * rotationAxis[2] * (1 - cos_value);
	rot_mat.at<float>(2, 2) = cos_value + rotationAxis[2] * rotationAxis[2] * (1 - cos_value);

	return rot_mat;
}

cv::Point3f MathOperation::CalPointIntersectPlanePoint(
	const cv::Point3f pointOutPlane, //rule center
	const float planeNormal[3],
	const cv::Point3f planePoint)  //vertice
{
	float  t, line[3];// vpt, v[3],
	line[0] = planePoint.x - pointOutPlane.x;
	line[1] = planePoint.y - pointOutPlane.y;
	line[2] = planePoint.z - pointOutPlane.z;

	//distance of two points along plane normal
	t = line[0] * planeNormal[0] + line[1] * planeNormal[1] + line[2] * planeNormal[2];

	//if parallel
	if (std::abs(t) < 1e-3)
	{
		return pointOutPlane;
	}
	else
	{
		cv::Point3f returnResult;
		returnResult.x = pointOutPlane.x + planeNormal[0] * t;
		returnResult.y = pointOutPlane.y + planeNormal[1] * t;
		returnResult.z = pointOutPlane.z + planeNormal[2] * t;
		return returnResult;
	}
}

std::pair<cv::Point3f, float> MathOperation::CalPointIntersectLine(const cv::Point3f pointOutLine,
	const cv::Point3f pointALine,
	const cv::Point3f pointBLine)
{
	//see https://www.cnblogs.com/mazhenyu/p/3508735.html
	cv::Point3f line = pointALine - pointBLine;
	float length_line = cv::norm(line);

	if (length_line < 0.1)
	{
		return(std::pair<cv::Point3f, float>((pointALine - pointBLine)*0.5f, 0.f));
	}
	else {
		float k = -(pointBLine - pointOutLine).dot(line) / std::pow(length_line, 2);

		cv::Point3f point = k * line + pointBLine;

		float dist_to_line = cv::norm(pointOutLine - point);

		/*printf("line: %f, %f, %f",  line.x,line.y,line.z );
		printf("k: %f", k);
		printf("pt: %f, %f, %f", point.x , point.y , point.z);*/

		return(std::pair<cv::Point3f, float>(point, dist_to_line));
	}
}

bool MathOperation::IsPointInCircle(const cv::Point3f p, const cv::Point3f center, const float radius)
{
	if ((std::pow(p.x - center.x, 2) + std::pow(p.y - center.y, 2) + std::pow(p.z - center.z, 2) <= radius * radius))
	{
		return true; //inside
	}
	else
	{
		return false;  //outside
	}
}

//求解参考 https://blog.csdn.net/abcjennifer/article/details/6688080#
cv::Point3f MathOperation::CalPlaneLineIntersectPoint(const cv::Point3f pt1, const cv::Point3f pt2,
	const cv::Point3f planeNormal, const cv::Point3f planePoint)
{
	float v1, v2, v3, t, vpt;

	float v_len = sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2) + pow(pt1.z - pt2.z, 2));
	v1 = (pt1.x - pt2.x) / v_len;
	v2 = (pt1.y - pt2.y) / v_len;
	v3 = (pt1.z - pt2.z) / v_len;

	vpt = v1 * planeNormal.x + v2 * planeNormal.y + v3 * planeNormal.z;

	if (abs(vpt) < 0.00001f)
		return cv::Point3f(0.f, 0.f, 0.f);

	t = ((planePoint.x - pt1.x) * planeNormal.x + (planePoint.y - pt1.y) * planeNormal.y + (planePoint.z - pt1.z) * planeNormal.z) / vpt;
	cv::Point3f Result;
	Result.x = pt1.x + v1 * t;
	Result.y = pt1.y + v2 * t;
	Result.z = pt1.z + v3 * t;

	return Result;
}


bool MathOperation::CalPlaneLineIntersectPoint_LineNoraml(const cv::Point3f lineNormal, const cv::Point3f linePoint,
	const cv::Point3f planeNormal, const cv::Point3f planePoint, cv::Point3f &resultPoint)
{
	float t, vpt;

	vpt = lineNormal.x * planeNormal.x + lineNormal.y * planeNormal.y + lineNormal.z * planeNormal.z;

	if (abs(vpt) < 0.00001f)
		return false;

	t = ((planePoint.x - linePoint.x) * planeNormal.x + (planePoint.y - linePoint.y) * planeNormal.y + (planePoint.z - linePoint.z) * planeNormal.z) / vpt;
	resultPoint.x = linePoint.x + lineNormal.x * t;
	resultPoint.y = linePoint.y + lineNormal.y * t;
	resultPoint.z = linePoint.z + lineNormal.z * t;

	return true;
}


bool MathOperation::IsPointInBoxXOY(const cv::Point3f & p, const BBox3D & box, float dilateRadius)
{
	BBox3D detectBox = box;
	detectBox.dilate(dilateRadius);
	return (detectBox.m_min.x < p.x && p.x < detectBox.m_max.x
		&& detectBox.m_min.y < p.y && p.y < detectBox.m_max.y);
}

//void MathOperation::getNewNormalAndCenter(const std::vector<cv::Point3f> &pts, cv::Point3f& center, cv::Point3f& normal)
//{
//	if (pts.size() < 100) return;
//	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//	//inliers表示误差能容忍的点 记录的是点云的序号
//	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//	// 创建一个分割器
//	pcl::SACSegmentation<pcl::PointXYZ> seg;
//	// Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
//	seg.setOptimizeCoefficients(true);
//	// Mandatory-设置目标几何形状
//	seg.setModelType(pcl::SACMODEL_PLANE);
//	//分割方法：随机采样法
//	seg.setMethodType(pcl::SAC_RANSAC);
//	//设置误差容忍范围，也就是我说过的阈值
//	seg.setDistanceThreshold(0.5);
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
//	cloud->points.resize(pts.size());
//	for (int i = 0; i < pts.size(); i++) {
//		cloud->points[i].x = pts[i].x;
//		cloud->points[i].y = pts[i].y;
//		cloud->points[i].z = pts[i].z;
//	}
//	pcl::VoxelGrid<pcl::PointXYZ> filter;
//	filter.setInputCloud(cloud);
//	filter.setLeafSize(10.f, 10.0f, 10.f);// 设置体素栅格的大小
//	filter.filter(*filteredCloud);
//	//printf("before: %d,  after: %d\n", cloud->points.size(), filteredCloud->points.size());
//	seg.setInputCloud(filteredCloud);
//	//分割点云
//	seg.segment(*inliers, *coefficients);
//
//	std::vector<cv::Point3f> RANSAC_pts(inliers->indices.size());
//	for (size_t i = 0; i < inliers->indices.size(); i++)
//	{
//		RANSAC_pts[i].x = filteredCloud->points[inliers->indices[i]].x;
//		RANSAC_pts[i].y = filteredCloud->points[inliers->indices[i]].y;
//		RANSAC_pts[i].z = filteredCloud->points[inliers->indices[i]].z;
//	}
//
//
//	unsigned int step_sample = 1; // to accelerate speed
//	cv::PCA pca;
//	Util_PCA::pca3(RANSAC_pts, step_sample, pca);
//	// Util_PCA::pca3(pts, pca);
//	cv::Mat_<float> eig_vals = pca.eigenvalues;
//	// pick minimum eigen value
//	float min_eig = eig_vals.at<float>(0, 0);
//	unsigned int id_min_eig = 0;
//	if (eig_vals.at<float>(1, 0) < min_eig) {
//		min_eig = eig_vals.at<float>(1, 0);
//		id_min_eig = 1;
//	}
//	if (eig_vals.at<float>(2, 0) < min_eig) {
//		min_eig = eig_vals.at<float>(2, 0);
//		id_min_eig = 2;
//	}
//	// compute plane function
//	float a = pca.eigenvectors.at<float>(id_min_eig, 0);
//	float b = pca.eigenvectors.at<float>(id_min_eig, 1);
//	float c = pca.eigenvectors.at<float>(id_min_eig, 2);
//	float len_n = sqrt(a * a + b * b + c * c);
//	a = a / len_n; b = b / len_n; c = c / len_n;
//
//	float plane_normal[3] = {a, b, c};
//	float zdirection[3] = { 0.f,0.f,1.f };
//
//	cv::Mat rotation_matrix = MeasureBase::CalRotationMatrixFromVectors(plane_normal, zdirection);
//	std::vector<cv::Point3f> rotated_pts, rotated_RANSAC_pts;
//	MeasureBase::RotatePoints(pts, rotation_matrix, rotated_pts);
//	MeasureBase::RotatePoints(RANSAC_pts, rotation_matrix, rotated_RANSAC_pts);
//	/*IOData::SavePoint3fData("rotated_pts.txt", rotated_pts);
//	IOData::SavePoint3fData("rotated_RANSAC_pts.txt", rotated_RANSAC_pts);*/
//	//find ransac box
//	float x_min, x_max, y_min, y_max, z_min, z_max;
//	x_min = std::numeric_limits<float>::infinity();
//	x_max = -1 * std::numeric_limits<float>::infinity();
//	y_min = std::numeric_limits<float>::infinity();
//	y_max = -1 * std::numeric_limits<float>::infinity();
//	z_min = std::numeric_limits<float>::infinity();
//	z_max = -1 * std::numeric_limits<float>::infinity();
//
//	size_t npts = rotated_RANSAC_pts.size();
//	for (size_t i = 0; i < npts - 1; i++) {
//		if (rotated_RANSAC_pts[i].x < rotated_RANSAC_pts[i + 1].x) {
//			if (rotated_RANSAC_pts[i + 1].x > x_max) x_max = rotated_RANSAC_pts[i + 1].x;
//			if (rotated_RANSAC_pts[i].x < x_min) x_min = rotated_RANSAC_pts[i].x;
//		}
//		else {
//			if (rotated_RANSAC_pts[i].x > x_max) x_max = rotated_RANSAC_pts[i].x;
//			if (rotated_RANSAC_pts[i + 1].x < x_min) x_min = rotated_RANSAC_pts[i + 1].x;
//		}
//		if (rotated_RANSAC_pts[i].y < rotated_RANSAC_pts[i + 1].y) {
//			if (rotated_RANSAC_pts[i + 1].y > y_max) y_max = rotated_RANSAC_pts[i + 1].y;
//			if (rotated_RANSAC_pts[i].y < y_min) y_min = rotated_RANSAC_pts[i].y;
//		}
//		else {
//			if (rotated_RANSAC_pts[i].y > y_max) y_max = rotated_RANSAC_pts[i].y;
//			if (rotated_RANSAC_pts[i + 1].y < y_min) y_min = rotated_RANSAC_pts[i + 1].y;
//		}
//		if (rotated_RANSAC_pts[i].z < rotated_RANSAC_pts[i + 1].z) {
//			if (rotated_RANSAC_pts[i + 1].z > z_max) z_max = rotated_RANSAC_pts[i + 1].z;
//			if (rotated_RANSAC_pts[i].z < z_min) z_min = rotated_RANSAC_pts[i].z;
//		}
//		else {
//			if (rotated_RANSAC_pts[i].z > z_max) z_max = rotated_RANSAC_pts[i].z;
//			if (rotated_RANSAC_pts[i + 1].z < z_min) z_min = rotated_RANSAC_pts[i + 1].z;
//		}
//	}
//
//	cv::Point3f expent = cv::Point3f(0.f, 0.f, 25.f);
//	cv::Point3f m_min = cv::Point3f(x_min, y_min, z_min) - expent;
//	cv::Point3f m_max = cv::Point3f(x_max, y_max, z_max) + expent;
//	//cout << m_min << m_max;
//
//	std::vector<int> select_id;
//	for (size_t i = 0; i < rotated_pts.size(); i++)
//	{
//		if (m_min.x < rotated_pts[i].x && rotated_pts[i].x < m_max.x &&
//			m_min.y < rotated_pts[i].y && rotated_pts[i].y < m_max.y &&
//			m_min.z < rotated_pts[i].z && rotated_pts[i].z < m_max.z)
//		{
//			select_id.push_back(i);
//		}
//	}
//	std::vector<cv::Point3f> output(select_id.size());
//	cv::Point3d centerPT(0.0,0.0,0.0);
//	for (size_t i = 0; i < select_id.size(); i++)
//	{
//		output[i] = pts[select_id[i]];
//		centerPT.x += pts[select_id[i]].x;
//		centerPT.y += pts[select_id[i]].y;
//		centerPT.z += pts[select_id[i]].z;
//	}
//	/*IOData::SavePoint3fData("output.txt", output);
//	IOData::SavePoint3fData("pts.txt", pts);*/
//	unsigned int step_sample2 = output.size() / 20000; // to accelerate speed
//	cv::PCA pca2;
//	Util_PCA::pca3(output, step_sample2, pca2);
//	// Util_PCA::pca3(pts, pca);
//
//	cv::Mat_<float> eig_vals2 = pca2.eigenvalues;
//	// pick minimum eigen value
//	float min_eig2 = eig_vals2.at<float>(0, 0);
//	unsigned int id_min_eig2 = 0;
//	if (eig_vals2.at<float>(1, 0) < min_eig2) {
//		min_eig2 = eig_vals2.at<float>(1, 0);
//		id_min_eig2 = 1;
//	}
//	if (eig_vals2.at<float>(2, 0) < min_eig2) {
//		min_eig2 = eig_vals2.at<float>(2, 0);
//		id_min_eig2 = 2;
//	}
//	// compute plane function
//	float a2 = pca2.eigenvectors.at<float>(id_min_eig2, 0);
//	float b2 = pca2.eigenvectors.at<float>(id_min_eig2, 1);
//	float c2 = pca2.eigenvectors.at<float>(id_min_eig2, 2);
//	float len_n2 = sqrt(a2 * a2 + b2 * b2 + c2 * c2);
//	a2 = a2 / len_n2; b2 = b2 / len_n2; c2 = c2 / len_n2;
//
//	// update normal
//	normal = cv::Point3f(a2, b2, c2);
//	center = cv::Point3f(centerPT.x / select_id.size(), centerPT.y / select_id.size(), centerPT.z / select_id.size());
//}

void MathOperation::getNewNormalAndCenter(std::vector<cv::Point3f> &pts_src,
	cv::Point3f& center, cv::Point3f& normal,
	std::vector<cv::Point3f> &fitplane)
{
	/*
	if (pts_src.size() < 100) return;
	std::vector<cv::Point3f> pts;
	CloudSampling::DownSample(pts_src, 10, 10, 10, pts);
	//cout << pts_src.size() << "   " << pts.size() << endl;
	size_t iter = 100;
	cv::RNG rng((unsigned)time(NULL));
	float sigma = 25.f;
	size_t pretotal = 0;     //符合拟合模型的数据的个数
	std::vector<float> bestplane(4);
	std::vector<float> bestmask(pts.size());
	for (size_t i = 0; i < iter; i++)
	{
		int idx1, idx2, idx3;
		idx1 = rng.uniform((int)0, (int)pts.size());
		do { idx2 = rng.uniform((int)0, (int)pts.size()); } while (idx2 == idx1);
		do { idx3 = rng.uniform((int)0, (int)pts.size()); } while (idx3 == idx1 || idx3 == idx2);
		cv::Point3f pt1 = pts[idx1], pt2 = pts[idx2], pt3 = pts[idx3];

		// plane function z=ax+by+c
		float a = ((pt1.z - pt2.z)*(pt1.y - pt3.y) - (pt1.z - pt3.z)*(pt1.y - pt2.y)) /
			((pt1.x - pt2.x)*(pt1.y - pt3.y) - (pt1.x - pt3.x)*(pt1.y - pt2.y));
		float b = ((pt1.z - pt3.z) - a * (pt1.x - pt3.x)) / (pt1.y - pt3.y);
		float c = pt1.z - a * pt1.x - b * pt1.y;
		float len = sqrt(a * a + b * b + 1.f);
		std::vector<float> plane = { a,b,-1.f,c };
		std::vector<float> mask(pts.size());
		size_t total = 0;
		for (size_t j = 0; j < pts.size(); j++)
		{
			mask[j] = abs(plane[0] * pts[j].x + plane[1] * pts[j].y + plane[2] * pts[j].z + plane[3]) / len;
			if (mask[j] < sigma) total++;
		}
		if (total > pretotal)
		{
			pretotal = total;
			bestplane = plane;
			bestmask = mask;
		}
	}
	fitplane.clear();
	VoxelDataStruct::VoxelGrid_Base fitplane2;
	for (size_t i = 0; i < bestmask.size(); i++)
	{
		if (bestmask[i] < sigma) {
			fitplane.push_back(pts[i]);
			fitplane2.Push(pts[i]);
		}
	}
	fitplane2.Compute();
	float zdirection[3] = { 0.f,0.f,1.f };

	cv::Mat rotation_matrix = MeasureBase::CalRotationMatrixFromVectors(fitplane2.plane_normals, zdirection);
	std::vector<cv::Point3f> rotated_pts, rotated_RANSAC_pts;
	MeasureBase::RotatePoints(pts, rotation_matrix, rotated_pts);
	MeasureBase::RotatePoints(fitplane, rotation_matrix, rotated_RANSAC_pts);
	//IOData::SavePoint3fData("rotated_pts.txt", rotated_pts);
	//IOData::SavePoint3fData("rotated_RANSAC_pts.txt", rotated_RANSAC_pts);
	//find ransac box
	float x_min, x_max, y_min, y_max, z_min, z_max;
	x_min = std::numeric_limits<float>::infinity();
	x_max = -1 * std::numeric_limits<float>::infinity();
	y_min = std::numeric_limits<float>::infinity();
	y_max = -1 * std::numeric_limits<float>::infinity();
	z_min = std::numeric_limits<float>::infinity();
	z_max = -1 * std::numeric_limits<float>::infinity();

	size_t npts = rotated_RANSAC_pts.size();
	for (size_t i = 0; i < npts - 1; i++) {
		if (rotated_RANSAC_pts[i].x < rotated_RANSAC_pts[i + 1].x) {
			if (rotated_RANSAC_pts[i + 1].x > x_max) x_max = rotated_RANSAC_pts[i + 1].x;
			if (rotated_RANSAC_pts[i].x < x_min) x_min = rotated_RANSAC_pts[i].x;
		}
		else {
			if (rotated_RANSAC_pts[i].x > x_max) x_max = rotated_RANSAC_pts[i].x;
			if (rotated_RANSAC_pts[i + 1].x < x_min) x_min = rotated_RANSAC_pts[i + 1].x;
		}
		if (rotated_RANSAC_pts[i].y < rotated_RANSAC_pts[i + 1].y) {
			if (rotated_RANSAC_pts[i + 1].y > y_max) y_max = rotated_RANSAC_pts[i + 1].y;
			if (rotated_RANSAC_pts[i].y < y_min) y_min = rotated_RANSAC_pts[i].y;
		}
		else {
			if (rotated_RANSAC_pts[i].y > y_max) y_max = rotated_RANSAC_pts[i].y;
			if (rotated_RANSAC_pts[i + 1].y < y_min) y_min = rotated_RANSAC_pts[i + 1].y;
		}
		if (rotated_RANSAC_pts[i].z < rotated_RANSAC_pts[i + 1].z) {
			if (rotated_RANSAC_pts[i + 1].z > z_max) z_max = rotated_RANSAC_pts[i + 1].z;
			if (rotated_RANSAC_pts[i].z < z_min) z_min = rotated_RANSAC_pts[i].z;
		}
		else {
			if (rotated_RANSAC_pts[i].z > z_max) z_max = rotated_RANSAC_pts[i].z;
			if (rotated_RANSAC_pts[i + 1].z < z_min) z_min = rotated_RANSAC_pts[i + 1].z;
		}
	}

	cv::Point3f expent = cv::Point3f(0.f, 0.f, 25.f);
	cv::Point3f m_min = cv::Point3f(x_min, y_min, z_min) - expent;
	cv::Point3f m_max = cv::Point3f(x_max, y_max, z_max) + expent;
	//cout << m_min << m_max;

	std::vector<int> select_id;
	for (size_t i = 0; i < rotated_pts.size(); i++)
	{
		if (m_min.x < rotated_pts[i].x && rotated_pts[i].x < m_max.x &&
			m_min.y < rotated_pts[i].y && rotated_pts[i].y < m_max.y &&
			m_min.z < rotated_pts[i].z && rotated_pts[i].z < m_max.z)
		{
			select_id.push_back(i);
		}
	}
	VoxelDataStruct::VoxelGrid_Base fitplane2_new;
	for (size_t i = 0; i < select_id.size(); i++)
	{
		fitplane2_new.Push(pts[select_id[i]]);
	}
	//IOData::SavePoint3fData("output.txt", output);
	//IOData::SavePoint3fData("pts.txt", pts);
	fitplane2_new.Compute();
	fitplane2_new.CalPlaneCenter();

	// update normal
	normal = cv::Point3f(fitplane2_new.plane_normals[0], fitplane2_new.plane_normals[1], fitplane2_new.plane_normals[2]);
	center = cv::Point3f(fitplane2_new.plane_center[0], fitplane2_new.plane_center[1], fitplane2_new.plane_center[2]);
	*/

    if (pts_src.size() < 100) return;
	//cout << "getNewNormalAndCenter test begin" << endl;
	//comptue normal first
	VoxelDataStruct::VoxelGrid_Base fitplane2_ori;
	for (size_t i = 0; i < pts_src.size(); i++)
	{
		fitplane2_ori.Push(pts_src[i]);
	}
	fitplane2_ori.Compute();

	//std::vector<cv::Point3f> rot_plane_first_points;
	std::vector<cv::Point3f> rot_plane_points;
	//float rotation_angle_to_y_coarse;
	//float rot_plane_normal_to_y_coarse[3];
	//MeasureBase::CalcAngleVectorXY2YAxis(fitplane2_ori.plane_normals, &rotation_angle_to_y_coarse); // plane_normal -> refined_plane_normal modified by wei.fan 
	//cv::Mat rotation_matrix_to_y_coarse = MeasureBase::TranslateAngleAroundZ2RotationMatrix(rotation_angle_to_y_coarse);
	//MeasureBase::RotateVector(fitplane2_ori.plane_normals, rotation_matrix_to_y_coarse, rot_plane_normal_to_y_coarse); // plane_normal -> refined_plane_normal modified by wei.fan	MeasureBase::RotatePoints(pts_src, rotation_matrix_to_y_coarse, rot_plane_first_points);
	
	float zdirection_test[3] = { 0.f,0.f,1.f };
	cv::Mat rotation_matrix_origin = MeasureBase::CalRotationMatrixFromVectors(fitplane2_ori.plane_normals, zdirection_test);


	//std::vector<cv::Point3f> rot_plane_points_to_y_fine, rot_centers_to_y_fine;
//	float rotation_angle_to_y_fine;
	//MeasureBase::CalcAngleVectorYZ2YAxis(rot_plane_normal_to_y_coarse, &rotation_angle_to_y_fine);
	//if (abs(fitplane2_ori.plane_normals[2]) < 0.2)
	//	rotation_angle_to_y_fine = 0;
	//else
	//	std::cout << "xxxx" << std::endl;
	//cv::Mat rotation_matrix_to_y_fine = MeasureBase::TranslateAngleAroundX2RotationMatrix(rotation_angle_to_y_fine);
	MeasureBase::RotatePoints(pts_src, rotation_matrix_origin, rot_plane_points);


	std::cout << rot_plane_points.size() << std::endl;
	float plane_minmax_xy[6];
	MeasureBase::FindPointsMinMaxXYZ(rot_plane_points, plane_minmax_xy);


	int voxel_witdh = 30;
	unsigned int voxel_num_in_x = std::floor((plane_minmax_xy[1] - plane_minmax_xy[0]) / voxel_witdh) + 1;
	unsigned int voxel_num_in_y = std::floor((plane_minmax_xy[3] - plane_minmax_xy[2]) / voxel_witdh) + 1;

	std::vector<cv::Point3f>  voxel_pts;
	std::vector<int>  voxel_pt_couts;
	std::cout << voxel_num_in_x << "," << voxel_num_in_y << std::endl;
	voxel_pts.resize(voxel_num_in_x * voxel_num_in_y);
	voxel_pt_couts.resize(voxel_num_in_x * voxel_num_in_y);
	unsigned int x_idx, y_idx, voxel_idx;

	float delta_y;

	for (int i = 0; i < voxel_pts.size(); i++)
		voxel_pt_couts[i] = 0;

	for (int i = 0; i < rot_plane_points.size(); i++)
	{
		if (abs(rot_plane_points[i].x - plane_minmax_xy[0]) < 30.0 ||
			abs(rot_plane_points[i].x - plane_minmax_xy[1]) < 30.0 ||
			abs(rot_plane_points[i].y - plane_minmax_xy[2]) < 30.0 ||
			abs(rot_plane_points[i].y - plane_minmax_xy[3]) < 30.0)
			continue;
		//assign to voxel
		x_idx = std::floor((rot_plane_points[i].x - plane_minmax_xy[0]) / voxel_witdh);
		y_idx = std::floor((rot_plane_points[i].y - plane_minmax_xy[2]) / voxel_witdh);
		voxel_idx = y_idx * voxel_num_in_x + x_idx;

		voxel_pts[voxel_idx] += rot_plane_points[i];
		voxel_pt_couts[voxel_idx]++;

	}

	std::vector<int>  voxel_idxs;
	std::vector<cv::Point3f> new_pts;
	for (int i = 0; i < voxel_pts.size(); i++)
	{
		if (voxel_pt_couts[i] > 0)
		{
			voxel_pts[i] /= voxel_pt_couts[i];
			new_pts.push_back(voxel_pts[i]);
			voxel_idxs.push_back(i);
		}
	}



	std::vector<cv::Point3f>  voxel_pts_ori;
	///cv::Mat backward_rotation_matrix_from_y_coarse(3, 3, CV_32FC1);
	cv::Mat backward_rotation_matrix(3, 3, CV_32FC1);
//	cv::transpose(rotation_matrix_to_y_coarse, backward_rotation_matrix_from_y_coarse);
	//cv::transpose(rotation_matrix_origin, backward_rotation_matrix);
	backward_rotation_matrix = rotation_matrix_origin.inv();


	std::vector<cv::Point3f> pts;
	if (new_pts.size() > 0)
		pts = new_pts;
	else
		;///Tao CloudSampling::DownSample(rot_plane_points, 50, 50, 50, pts);
	
	
	//cout << pts_src.size() << "   " << pts.size() << endl;
	size_t iter = 200;
	cv::RNG rng((unsigned)time(NULL));
	float sigma = 5.f;
	size_t pretotal = 0;     //符合拟合模型的数据的个数
	std::vector<float> bestplane(4);
	std::vector<float> bestmask(pts.size());
	for (size_t i = 0; i < iter; i++)
	{
		int idx1, idx2, idx3;
		idx1 = rng.uniform((int)0, (int)pts.size());
		do { idx2 = rng.uniform((int)0, (int)pts.size()); } while (idx2 == idx1);
		do { idx3 = rng.uniform((int)0, (int)pts.size()); } while (idx3 == idx1 || idx3 == idx2);
		cv::Point3f pt1 = pts[idx1], pt2 = pts[idx2], pt3 = pts[idx3];

		// plane function z=ax+by+c
		float a = ((pt1.z - pt2.z)*(pt1.y - pt3.y) - (pt1.z - pt3.z)*(pt1.y - pt2.y)) /
			((pt1.x - pt2.x)*(pt1.y - pt3.y) - (pt1.x - pt3.x)*(pt1.y - pt2.y));
		float b = ((pt1.z - pt3.z) - a * (pt1.x - pt3.x)) / (pt1.y - pt3.y);
		float c = pt1.z - a * pt1.x - b * pt1.y;
		float len = sqrt(a * a + b * b + 1.f);
		std::vector<float> plane = { a,b,-1.f,c };
		std::vector<float> mask(pts.size());
		size_t total = 0;
		for (size_t j = 0; j < pts.size(); j++)
		{
			mask[j] = abs(plane[0] * pts[j].x + plane[1] * pts[j].y + plane[2] * pts[j].z + plane[3]) / len;
			if (mask[j] < sigma) total++;
		}
		if (total > pretotal)
		{
			pretotal = total;
			bestplane = plane;
			bestmask = mask;
		}
	}
	fitplane.clear();
	VoxelDataStruct::VoxelGrid_Base fitplane2;

	
	for (size_t i = 0; i < bestmask.size(); i++)
	{
		if (bestmask[i] < sigma) {
			fitplane.push_back(pts[i]);
			fitplane2.Push(pts[i]);
		
		}
	}
	fitplane2.Compute();
	float zdirection[3] = { 0.f,0.f,1.f };

	fill(voxel_pt_couts.begin(), voxel_pt_couts.end() , 0);
	for (int i = 0; i < fitplane.size(); i++)
	{
		x_idx = std::floor((fitplane[i].x - plane_minmax_xy[0]) / voxel_witdh);
		y_idx = std::floor((fitplane[i].y - plane_minmax_xy[2]) / voxel_witdh);
		voxel_idx = y_idx * voxel_num_in_x + x_idx;
		voxel_pt_couts[voxel_idx] = 1;
	}


	std::vector<int> select_id;
	for (int i = 0; i < rot_plane_points.size(); i++)
	{
		if (abs(rot_plane_points[i].x - plane_minmax_xy[0]) < 30.0 ||
			abs(rot_plane_points[i].x - plane_minmax_xy[1]) < 30.0 ||
			abs(rot_plane_points[i].y - plane_minmax_xy[2]) < 30.0 ||
			abs(rot_plane_points[i].y - plane_minmax_xy[3]) < 30.0)
			continue;
		//assign to voxel
		x_idx = std::floor((rot_plane_points[i].x - plane_minmax_xy[0]) / voxel_witdh);
		y_idx = std::floor((rot_plane_points[i].y - plane_minmax_xy[2]) / voxel_witdh);
		voxel_idx = y_idx * voxel_num_in_x + x_idx;

		if (voxel_pt_couts[voxel_idx])
			select_id.push_back(i);

	}
	VoxelDataStruct::VoxelGrid_Base fitplane2_new;

	std::vector<cv::Point3f>  savept;
	for (size_t i = 0; i < select_id.size(); i++)
	{
		fitplane2_new.Push(rot_plane_points[select_id[i]]);
		savept.push_back(rot_plane_points[select_id[i]]);
	}
	//if(g_saveid == 6)
	//	IOData::SavePoint3fData("output.txt", savept);
	/*IOData::SavePoint3fData("output.txt", output);
	IOData::SavePoint3fData("pts.txt", pts);*/
	fitplane2_new.Compute();
	fitplane2_new.CalPlaneCenter();

	float rot_plane_normal[3];
	float rot_plane_center[3];
	//MeasureBase::RotateVector(fitplane2_new.plane_normals, backward_rotation_matrix_from_y_coarse, rot_plane_normal); 
	//MeasureBase::RotateVector(fitplane2_new.plane_center, backward_rotation_matrix_from_y_coarse, rot_plane_center);

	MeasureBase::RotateVector(fitplane2_new.plane_normals, backward_rotation_matrix, rot_plane_normal);
	MeasureBase::RotateVector(fitplane2_new.plane_center, backward_rotation_matrix, rot_plane_center);

	
	// update normal
	//normal = { cv::Point3f(fitplane2_new.plane_normals[0],fitplane2_new.plane_normals[1],fitplane2_new.plane_normals[2]) };
	//center = { cv::Point3f(fitplane2_new.plane_center[0],fitplane2_new.plane_center[1],fitplane2_new.plane_center[2]) };

	normal = { cv::Point3f(rot_plane_normal[0],rot_plane_normal[1],rot_plane_normal[2]) };
	center = { cv::Point3f(rot_plane_center[0],rot_plane_center[1],rot_plane_center[2]) };
	//cout << "getNewNormalAndCenter test end;" << endl;
	
}

// url: https://www.cnblogs.com/nobodyzhou/p/6145030.html
void MathOperation::CalPointOnPlaneProjection(cv::Point3f& pt, const cv::Point3f plane_normals, const cv::Point3f plane_center)
{
	float a = plane_normals.x;
	float b = plane_normals.y;
	float c = plane_normals.z;

	float x0 = plane_center.x;
	float y0 = plane_center.y;
	float z0 = plane_center.z;

	float x = pt.x;
	float y = pt.y;
	float z = pt.z;

	double t = (a*x0 + b*y0 + c*z0 - (a*x + b*y + c*z)) / (a*a + b*b + c*c);

	pt = cv::Point3f(x + a * t, y + b * t, z + c * t);
}


float MathOperation::CalLine2LineAngle(const cv::Point L1_A, const cv::Point L1_B, const cv::Point L2_A, const cv::Point L2_B)
{
	cv::Point vector1 = L1_B - L1_A;
	cv::Point vector2 = L2_B - L2_A;

	float angle = std::acosf(MathOperation::ComputeVectorDotProduct(vector1, vector2) / MathOperation::VectorNormalize(vector1) / MathOperation::VectorNormalize(vector2));

	angle = angle * 180.f / CV_PI;	
	angle = angle > 90 ? (180.f - angle) : angle;

	return angle;
}