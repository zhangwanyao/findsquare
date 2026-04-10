/* Copyright (c) ASTRI, 2019. All rights reserved.
* This software is proprietary to and embodies the confidential technology
* of Hong Kong Applied Science and Technology Research Institute Company
* Limited (ASTRI).
*
* Possession, use, or copying of this software and media is authorized
* only pursuant to a valid written license from ASTRI or an authorized
* sublicensor.
*
* Author:mounty
* Date: 20190826
* Description: Plane Segmentation Class Definitions
*/

#include "MathOperation.hpp"
#include "GeometryFilter.h"
#include "log.h"
#include <math.h>

using namespace ModuleStruct;

GeometryFilter::GeometryFilter() {
	this->Init();
	plane_normals = Point3f(0, 0, 0);
	plane_center = Point3f(0, 0, 0);
}

GeometryFilter::~GeometryFilter() {
	FreeMemory();
};

void GeometryFilter::FreeMemory()
{
	delete[] plane_xyz_2D.points;
	return;
}

void GeometryFilter::Reset() {
	this->Init();
	FreeMemory();
}

void GeometryFilter::Rotate2XYPlane(const PointArray & pt_cloud_xyz)
{
	CMat rot_mat_z(3, 3, CV_32F), rot_mat_y1(3, 3, CV_32F), rot_mat_y2(3, 3, CV_32F);
	CMat rot_mat_zy(3, 3, CV_32F), rot_mat_zy_transpose(3, 3, CV_32F);
	CMat fnl_rot_mat(3, 3, CV_32F), fnl_rot_mat_transpose(3, 3, CV_32F);

	float length_xy = std::sqrtf(std::pow(plane_normals.x, 2) + std::pow(plane_normals.y, 2));
	float ang_y1 = (float)(-M_PI / 2);
	float ang_y2 = (float)(atan2f(-plane_normals.z, length_xy + 1e-10f));
	float ang_z = (float)(atan2f(plane_normals.y, plane_normals.x + 1e-10f));

	rot_mat_y1 = CreateRotationMat(ang_y1, 1);
	rot_mat_y2 = CreateRotationMat(ang_y2, 1);
	rot_mat_z = CreateRotationMat(ang_z, 2);

	rot_mat_zy = rot_mat_z * rot_mat_y2;
	cv::transpose(rot_mat_zy, rot_mat_zy_transpose);
	fnl_rot_mat = rot_mat_y1 * rot_mat_zy_transpose;

	plane_xyz_2D.size = pt_cloud_xyz.size;

	plane_xyz_2D.points = new Point3f[sizeof(Point3f)*plane_xyz_2D.size];

	for (unsigned int i = 0; i < plane_xyz_2D.size; i++) {
		plane_xyz_2D.points[i].x = fnl_rot_mat.at<float>(0, 0)*pt_cloud_xyz.points[i].x +
			fnl_rot_mat.at<float>(0, 1)*pt_cloud_xyz.points[i].y +
			fnl_rot_mat.at<float>(0, 2)*pt_cloud_xyz.points[i].z;

		plane_xyz_2D.points[i].y = fnl_rot_mat.at<float>(1, 0)*pt_cloud_xyz.points[i].x +
			fnl_rot_mat.at<float>(1, 1)*pt_cloud_xyz.points[i].y +
			fnl_rot_mat.at<float>(1, 2)*pt_cloud_xyz.points[i].z;

		plane_xyz_2D.points[i].z = fnl_rot_mat.at<float>(2, 0)*pt_cloud_xyz.points[i].x +
			fnl_rot_mat.at<float>(2, 1)*pt_cloud_xyz.points[i].y +
			fnl_rot_mat.at<float>(2, 2)*pt_cloud_xyz.points[i].z;
	}

	//get mean value of z
	float mean_z = 0;

	for (unsigned int i = 0; i < plane_xyz_2D.size; i++)
		mean_z += plane_xyz_2D.points[i].z;
	mean_z = mean_z / float(plane_xyz_2D.size);

	for (unsigned int i = 0; i < plane_xyz_2D.size; i++)
		plane_xyz_2D.points[i].z = mean_z;

	return;
}

// create voxels for each plane
void GeometryFilter::CreateVoxel()
{
	//is_occupied = new unsigned int[sizeof(unsigned int)*total_num_of_voxel_grid];
	//memset(is_occupied, 0, sizeof(unsigned int)*total_num_of_voxel_grid);
	for (unsigned int i = 0; i < plane_xyz_2D.size; i++) {
		int bin_idx_x = (int)(std::floor((plane_xyz_2D.points[i].x - min_x) / length_x_of_voxel) + 1);
		int bin_idx_y = (int)(std::floor((plane_xyz_2D.points[i].y - min_y) / length_y_of_voxel) + 1);
		int bin_idx_z = (int)(std::floor((plane_xyz_2D.points[i].z - min_z) / length_z_of_voxel));
		int voxel_grid_idx = (int)(bin_idx_z*(cols_of_voxel*rows_of_voxel) + bin_idx_y * cols_of_voxel + bin_idx_x);
		//is_occupied[voxel_grid_idx] = true;
	}

	// compute num_of_occupied_voxel
	//MathOperation::ComputeSumsByParallel(is_occupied, (unsigned int)total_num_of_voxel_grid, (unsigned int)num_of_occupied_voxel);
	//delete[] is_occupied;
	return;
}

bool GeometryFilter::GetPlaneArea(const PointArray& pt_plane_xyz, double &area)
{
	//rotate the plane to x - y plane
	Rotate2XYPlane(pt_plane_xyz);

	//get min, max x, y, z
	GetMinMaxXYZ(plane_xyz_2D);

	//get total voxel size
	GetTotalNumOfVoxelGrid(2);

	//create voxel and get num_of_occupied_voxel
	CreateVoxel();
	area = (double)(num_of_occupied_voxel) * (length_x_of_voxel * length_y_of_voxel);
	return true;
}

void GeometryFilter::GetPlaneArea(GeometryFilterParams &Geometry_filter_params, const PointArray& pt_plane_xyz, float &area)
{
	length_x_of_voxel = Geometry_filter_params.voxel_params.length_x_of_voxel;
	length_y_of_voxel = Geometry_filter_params.voxel_params.length_y_of_voxel;
	length_z_of_voxel = Geometry_filter_params.voxel_params.length_z_of_voxel;
	//plane_normal = plane_normal;

	//rotate the plane to x - y plane
	Rotate2XYPlane(pt_plane_xyz);

	//get min, max x, y, z
	GetMinMaxXYZ(plane_xyz_2D);

	//get total voxel size
	GetTotalNumOfVoxelGrid(2);

	//create voxel and get num_of_occupied_voxel
	CreateVoxel();

	area = (float)(num_of_occupied_voxel)*(length_x_of_voxel*length_y_of_voxel);

	return;
}

bool GeometryFilter::Rotate2XYPlane2D(const PointArray& pt_plane_xyz)
{
	CMat rot_mat(3, 3, CV_64F);
	Point3f nz = { 0,0,1 };
	plane_normals = Util_Math::vec3_normalize(plane_normals);
	nz = Util_Math::vec3_normalize(nz);
	rot_mat = CreateRotationMat4E(plane_normals, nz);

	Point3f* pts = new Point3f[pt_plane_xyz.size];

	plane_xy_2D.clear();
	plane_xy_2D.resize(pt_plane_xyz.size);
	float data_type = 0.f;

	float mat_rot_v[3][3];
	for (size_t i = 0; i < 3; i++)
		for (size_t j = 0; j < 3; j++)
			mat_rot_v[i][j] = rot_mat.ptr<float>(i)[j];

#pragma omp parallel for
	for (int i = 0; i < pt_plane_xyz.size; i++) {
		pts[i] = pt_plane_xyz.points[i] - plane_center;
		//Util_Math::rot<Point3f,float,CMat>(pts[i], data_type, rot_mat);
		Util_Math::rot<Point3f, float>(pts[i], mat_rot_v);
		plane_xy_2D[i].x = pts[i].x;
		plane_xy_2D[i].y = pts[i].y;
	}
	plane_xyz_2D.points = pts;
	plane_xyz_2D.size = pt_plane_xyz.size;
	return true;
}

int GeometryFilter::GetNumOfOccupiedPixels(const VoxelParams voxel_size)
{
	unsigned int input_point_size = static_cast<unsigned int>(plane_xy_2D.size());

	PointGrid2PointIdx* point_to_grid_list = new PointGrid2PointIdx[input_point_size];
	for (unsigned int i = 0; i < input_point_size; i++)
	{
		point_to_grid_list[i].cloud_point_index = i;
		ConvertPointToPixelID(min_p, plane_xy_2D[i], voxel_size, point_to_grid_list[i].grid_idx);
	}
	std::sort(point_to_grid_list, point_to_grid_list + input_point_size, std::less<PointGrid2PointIdx>());

	unsigned int occupied_cnt = 0;  // total occupied number of pixels
	unsigned int i = 0;
	while (i < input_point_size)
	{
		unsigned int j = i + 1;
		while ((j < input_point_size) && (point_to_grid_list[i].grid_idx == point_to_grid_list[j].grid_idx))
		{
			++j;
		}
		occupied_cnt++;
		i = j;
	}
	delete[] point_to_grid_list;
	return occupied_cnt;
}

bool GeometryFilter::GetPlaneArea2D(const PointArray& pt_plane_xyz, const VoxelParams voxel_size, double &area)
{
	//input rotation
	Rotate2XYPlane2D(pt_plane_xyz);

	//pixel initializeation
	GetMinMaxXY(plane_xy_2D, max_p, min_p);

	// get number of pixel
	unsigned long long pixel_num = GetTotalNumOfPixelGrid(max_p, min_p, voxel_size);

	// creat pixels
	int num_of_occupied = GetNumOfOccupiedPixels(voxel_size);
	// Area compute

	area = num_of_occupied * voxel_size.length_x_of_voxel * voxel_size.length_y_of_voxel;
	return true;
}