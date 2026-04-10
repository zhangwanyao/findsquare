/* Copyright (c) ASTRI, 2019. All rights reserved.
* This software is proprietary to and embodies the confidential technology
* of Hong Kong Applied Science and Technology Research Institute Company
* Limited (ASTRI).
*
* Possession, use, or copying of this software and media is authorized
* only pursuant to a valid written license from ASTRI or an authorized
* sublicensor.
*
* Author: mounty
* Date: 20190826
* Description: Plane Segmentation Class Definitions
*/

#ifndef GEOMETRY_FILTER_H
#define GEOMETRY_FILTER_H

#include "DataStruct.h"
#include "VoxelBaseClass.h"

//geometry filter params
struct GeometryFilterParams
{
	GeometryFilterParams(float length_x_of_voxel = 1.0f,
		float length_y_of_voxel = 1.0f,
		float length_z_of_voxel = 1.0f)
	{
		voxel_params.length_x_of_voxel = length_x_of_voxel;
		voxel_params.length_y_of_voxel = length_y_of_voxel;
		voxel_params.length_z_of_voxel = length_z_of_voxel;
	}
	// voxel length in x, y and z directions
	VoxelParams voxel_params;
};

class GeometryFilter :public VoxelBaseClass
{
private:

	/**
	* \brief  input plane center
	* */
	ModuleStruct::Point3f plane_normals;

	/**
	* \brief  input plane normal
	* */
	ModuleStruct::Point3f plane_center;

	/**
	* \brief plane x,y,z data on x - y plane
	* */
	PointArray plane_xyz_2D;

	/**
	* \brief plane x,y data on x - y plane
	* */
	ModuleStruct::Point2fArray plane_xy_2D;

	int cols_of_pixel;

	int rows_of_pixel;

	ModuleStruct::Point2f max_p, min_p;

	///**
	//* \brief array size is the plane voxel size , identify if the voxel of index in this plane is occpupied
	//* */
	//unsigned int* is_occupied;

private:

	void Reset();

	void FreeMemory();

	/**
	* \brief  rotate the input data into x - y plane
	* @param [in] pt_plane_xyz,   input plane x,y,z
	* */
	void Rotate2XYPlane(const PointArray& pt_plane_xyz);

	/**
	* \brief  create voxels
	* */
	void CreateVoxel();

	/**
	* \brief get plane area
	* @param [out] area, plane area
	* return true success, else failed
	* */
	bool GetPlaneArea(const PointArray& pt_plane_xyz, double &area);

	// compute plane area in 2D space
	bool GetPlaneArea2D(const PointArray& pt_plane_xyz, double& area);

	bool Rotate2XYPlane2D(const PointArray& pt_plane_xyz);

	inline bool GetMinMaxXY(const std::vector<ModuleStruct::Point2f> input_data, ModuleStruct::Point2f& max_p, ModuleStruct::Point2f& min_p)
	{
		if (input_data.size() == 0)
		{
			log_error("empty input");
			return false;
		}

		min_p.x = min_p.y = std::numeric_limits<float>::infinity();
		max_p.x = max_p.y = -std::numeric_limits<float>::infinity();

		for (unsigned int i = 0; i < input_data.size(); i++) {
			ModuleStruct::Point2f point = input_data[i];
			min_p.x = (min_p.x > point.x) ? point.x : min_p.x;
			max_p.x = (max_p.x < point.x) ? point.x : max_p.x;
			min_p.y = (min_p.y > point.y) ? point.y : min_p.y;
			max_p.y = (max_p.y < point.y) ? point.y : max_p.y;
		}
		return true;
	}

	inline unsigned long long GetTotalNumOfPixelGrid(const ModuleStruct::Point2f max_p, const ModuleStruct::Point2f min_p, const VoxelParams voxle_param)
	{
		//int cols_of_pixel, rows_of_pixel;
		if ((max_p.x - min_p.x) != 0 && std::remainder((max_p.x - min_p.x), voxle_param.length_x_of_voxel) == 0)
			cols_of_pixel = (unsigned int)(std::floor((max_p.x - min_p.x) / voxle_param.length_x_of_voxel));
		else
			cols_of_pixel = (unsigned int)(std::floor((max_p.x - min_p.x) / voxle_param.length_x_of_voxel) + 1);

		if ((max_p.y - min_p.y) != 0 && std::remainder((max_p.y - min_p.y), voxle_param.length_y_of_voxel) == 0)
			rows_of_pixel = (unsigned int)(std::floor((max_p.y - min_p.y) / voxle_param.length_y_of_voxel));
		else
			rows_of_pixel = (unsigned int)(std::floor((max_p.y - min_p.y) / voxle_param.length_y_of_voxel) + 1);

		unsigned long long total_num_of_pxl_grid = cols_of_pixel * rows_of_pixel;
		return total_num_of_pxl_grid;
	}

	bool ConvertPointToPixelID(const ModuleStruct::Point2f min_p, const ModuleStruct::Point2f point, const VoxelParams voxel_size, unsigned long long& grid_id)
	{
		unsigned int col_idx = static_cast<unsigned int>(std::floor((point.x - min_p.x) / voxel_size.length_x_of_voxel));
		unsigned int row_idx = static_cast<unsigned int>(std::floor((point.y - min_p.y) / voxel_size.length_y_of_voxel));
		if (col_idx == cols_of_pixel)
			col_idx--;
		if (row_idx == rows_of_pixel)
			row_idx--;
		grid_id = row_idx * cols_of_pixel + col_idx;
		return true;
	}

	int GetNumOfOccupiedPixels(const VoxelParams voxel_size);
public:
	GeometryFilter();

	~GeometryFilter();

	//get plane area
	void GetPlaneArea(GeometryFilterParams &Geometry_filter_params, const PointArray& pt_plane_xyz, float &area);
	bool GetPlaneArea2D(const PointArray& pt_plane_xyz, const VoxelParams voxel_size, double& area);

	/**
	* \brief  set  the input plane normal
	* @param [in] input_normal,   input plane normal
	* */
	inline void SetInputNormal(const ModuleStruct::Point3f input_normal)
	{
		plane_normals = input_normal;
	}

	/**
	* \brief  set  the input plane center
	* @param [in] input_center, input_center
	* */
	inline void SetInputCenter(const ModuleStruct::Point3f input_center)
	{
		plane_center = input_center;
	}
};

#endif //GEOMETRY_FILTER_H
