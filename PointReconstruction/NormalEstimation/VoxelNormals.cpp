#include "VoxelNormals.h"
#include "util_math.hpp"
#include "util_opencv.hpp"

VoxelNormals::VoxelNormals()
{
	cols = rows = 0;
}

VoxelNormals::~VoxelNormals()
{
	points_matrix.release();
}

void VoxelNormals::Init(ns_uint32 input_rows, ns_uint32 input_cols)
{
	//assert(input_rows > 3);
	cols = input_cols;
	rows = input_rows;
	covariance_matrix.release();
	points_matrix.release();
	int type = CV_64FC1;
	if(sizeof(interim_value_type) == 4)
	{
		type = CV_32FC1;
	}
	points_matrix.create(rows, cols, type);
	plane_mse = plane_curvature = 0;
	memset(plane_center, 0, sizeof(plane_center));
	memset(plane_normals, 0, sizeof(plane_normals));
}

void VoxelNormals::_BuildCovarianceMatrix()
{
	cv::Mat mean;
	Util_CV::covariance_matrix(points_matrix, covariance_matrix, mean, rows);

	//IntermediateType sum_x = 0, sum_y = 0, sum_z = 0;
	//for (int i = 0; i < rows; i++)
	//{
	//	sum_x += points_matrix.at<IntermediateType>(i, 0);
	//	sum_y += points_matrix.at<IntermediateType>(i, 1);
	//	sum_z += points_matrix.at<IntermediateType>(i, 2);
	//}
	//plane_center[0] = sum_x / rows;
	//plane_center[1] = sum_y / rows;
	//plane_center[2] = sum_z / rows;

	//for (int i = 0; i < cols; i++)
	//{
	//	plane_center[i] = mean.at<IntermediateType>(0, i);
	//}
	memcpy(plane_center, mean.ptr<interim_value_type>(0), sizeof(interim_value_type) * PLANE_NORMALS_CENTER_ARRAY);
}

void VoxelNormals::FastProcess()
{
	_BuildCovarianceMatrix();
	//TODO: shift all the eigen code written by chen meng to math operation
	//IntermediateType eig_val[PLANE_NORMALS_CENTER_ARRAY], eig_vec[PLANE_NORMALS_CENTER_ARRAY];
	//std::vector<IntermediateType> cov_mat_element(cols * cols);
	//for (ns_uint32 i = 0; i < cols; i++)
	//{
	//	for (ns_uint32 j = 0; j < cols; j++)
	//		cov_mat_element[j + cols * i] = covariance_matrix.at<IntermediateType>(i, j);
	//}

	////TODO: waiting for test
	//MathOperation::ComputeFastEigenParallel(cov_mat_element.data(), eig_val, eig_vec);
	//for (int i = 0; i < PLANE_NORMALS_CENTER_ARRAY; i++)
	//{
	//	plane_normals[i] = eig_vec[i];
	//}

	//IntermediateType average_distance = 0;
	////IntermediateType point[3];
	//for (ns_uint32 i = 0; i < rows; i++)
	//{
	//	//point[0] = points_matrix.at<IntermediateType>(i, 0);
	//	//point[1] = points_matrix.at<IntermediateType>(i, 1);
	//	//point[2] = points_matrix.at<IntermediateType>(i, 2);
	//	average_distance += MathOperation::ComputePointToPlaneDist(points_matrix.ptr<IntermediateType>(i), plane_center, plane_normals);
	//}
	//plane_mse = average_distance / rows;
}

void VoxelNormals::cvProcess()
{
	_BuildCovarianceMatrix();

	CMat eig_val_mat, eig_vec_mat;
	Util_CV::eigen(covariance_matrix, eig_val_mat, eig_vec_mat);
	//for (int i = 0; i < PLANE_NORMALS_CENTER_ARRAY; i++)
	//{
	//	plane_normals[i] = eig_vec_mat.at<IntermediateType>(2, i);
	//}
	memcpy(plane_normals, eig_vec_mat.ptr<interim_value_type>(2), sizeof(interim_value_type) * PLANE_NORMALS_CENTER_ARRAY);

	//curvature
	//plane_curvature = eig_val_mat.at<IntermediateType>(2) / (eig_val_mat.at<IntermediateType>(0) + eig_val_mat.at<IntermediateType>(1) + eig_val_mat.at<IntermediateType>(2));
}

interim_value_type VoxelNormals::GetMse()
{
	interim_value_type average_distance = 0.f;
	//IntermediateType point[3];
	for (ns_uint32 i = 0; i < rows; i++)
	{
		//point[0] = points_matrix.at<IntermediateType>(i, 0);
		//point[1] = points_matrix.at<IntermediateType>(i, 1);
		//point[2] = points_matrix.at<IntermediateType>(i, 2);
		interim_value_type* ptr = points_matrix.ptr<interim_value_type>(i);
		average_distance += Util_Math::ComputePointToPlaneDist(ptr, plane_center, plane_normals);
		//average_distance += MathOperation::ComputePointToPlaneDist(Point3f(point[0], point[1], point[2])
		//	, Point3f(plane_normals[0], plane_normals[1], plane_normals[2]), Point3f(plane_center[0], plane_center[1], plane_center[2]));
	}
	return plane_mse = average_distance / rows;
}

interim_value_type VoxelNormals::GetWeightedMse()
{
	interim_value_type center_normal_product = 0;
	interim_value_type total_center2point_distance = 0;
	interim_value_type points2center_distance = 0;
	for (ns_uint32 i = 0; i < rows; i++)
	{
		interim_value_type* ptr = points_matrix.ptr<interim_value_type>(i);
		total_center2point_distance += (points2center_distance = cv::norm(Point3f(ptr[0], ptr[1], ptr[2]) - Point3f(plane_center[0], plane_center[1], plane_center[2])));
		center_normal_product += Util_Math::ComputePointToPlaneDist(ptr, plane_center, plane_normals) * points2center_distance;
	}
	return total_center2point_distance > 0 ? center_normal_product / total_center2point_distance : 0;
}
