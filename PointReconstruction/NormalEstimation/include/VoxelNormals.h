#ifndef VOXEL_NORMAL_H
#define VOXEL_NORMAL_H

#include "ConstantDefines.h"
#include "NormalStruct.hpp"

class VoxelNormals
{
public:

	VoxelNormals();
	~VoxelNormals();

	void Init(ns_uint32 input_rows, ns_uint32 input_cols);
	void cvProcess();
	void FastProcess();
	inline void Push(const Point3f& point, ns_uint32 row_index)
	{
		//if (row_index < rows)
		//{
		//IntermediateType data[] = {point.x, point.y, point.z};
		//cv::Mat(1, 3, CV_64FC1, data).copyTo(points_matrix.row(row_index));
		//cv::Mat buf = (cv::Mat_<IntermediateType>(1, cols) << point.x, point.y, point.z);
		//buf.copyTo(points_matrix.row(row_index));
		interim_value_type* ptr = points_matrix.ptr<interim_value_type>(row_index);
		//memcpy(ptr, data, sizeof(data));
		ptr[0] = point.x, ptr[1] = point.y, ptr[2] = point.z;
		//points_matrix.at<IntermediateType>(row_index, 0) = point.x;
		//points_matrix.at<IntermediateType>(row_index, 1) = point.y;
		//points_matrix.at<IntermediateType>(row_index, 2) = point.z;
		//}
	}

	inline void Push(const Vector<Point3f>& points)
	{
		for (ns_uint32 t = 0; t < points.size(); t++)
		{
			interim_value_type* ptr = points_matrix.ptr<interim_value_type>(t);
			ptr[0] = points[t].x, ptr[1] = points[t].y, ptr[2] = points[t].z;
		}
	}

	inline Point3f GetNormals()
	{
		//Point3f_ex normal;
		//normal.x = plane_normals[0];
		//normal.y = plane_normals[1];
		//normal.z = plane_normals[2];
		return Point3f(plane_normals[0], plane_normals[1], plane_normals[2]);
	}

	inline Point3f GetCenters()
	{
		return Point3f(plane_center[0], plane_center[1], plane_center[2]);
	}
	interim_value_type GetMse();
	interim_value_type GetWeightedMse();

private:
	void _BuildCovarianceMatrix();

private:
	CMat points_matrix; // row * col = n * 3
	CMat covariance_matrix; //3 * 3
	ns_uint32 cols, rows;

	//plane mean squared error and curvature
	interim_value_type plane_mse{}, plane_curvature{};

	//plane center
	interim_value_type plane_center[PLANE_NORMALS_CENTER_ARRAY]{};

	//plane normals
	interim_value_type plane_normals[PLANE_NORMALS_CENTER_ARRAY]{};
};
#endif
