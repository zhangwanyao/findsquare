#ifndef _NORMAL_ESTIMATION_STRUCTURE_
#define _NORMAL_ESTIMATION_STRUCTURE_
#include "ModuleStruct.hpp"
#include <unordered_map>

using namespace ModuleStruct;

// Each occupied voxel with its element
struct VoxelItemStruct
{
	// for single voxel
	// True - Good Voxel, False - Bad Voxel
	bool is_good_voxel;

	// grid index: index to 3d space, contains both empty and occupied voxels
	unsigned int grid_idx;

	// point index in voxel
	Vector<unsigned int> points_index;
};

// Interface of Normal Estimation Module
struct NormalEstimationParamters
{
	// Data Type: InOut
	Point3Array* points;

	// Data Type: Input
	value_type threshold_max_normal_angle_of_two_voxel;
	size_t threshold_min_point_num_of_valid_normal_voxel;
	value_type threshold_min_mse_of_voxel;

	// voxel size: suggest same size for x, y, z
	value_type voxel_size_x, voxel_size_y, voxel_size_z;
};

struct NormalEstimationResults {
	Vector<Point3f> normals;

	// belongs to which voxel
	Vector<unsigned int> voxel_idx;			// index to occupied voxel array

	// Data Type: Output, voxel array which is occupied
	Vector<VoxelItemStruct> voxel_array;

	// Data Type: Output, it is used to find out voxel_idx in occupied voxel array from grid_idx in 3D Space
	Vector<unsigned int> grid_to_occupied_voxel_idx;

	// Points density
	value_type density;
};

/**
* \save bad points neighbour for normal estimation
*/
struct NormalEstimationBadPoints
{
	// int -> point index, std::vector<int> -> the neighbour points index
	std::unordered_map<unsigned int, std::vector<unsigned int>>  badPoint_map;
};

/**
* \brief data struct of normalEstimation contrl parameters
*/
struct NormalEstimationCtrParam
{
	/**
	* \brief if true  voxel size is auto detected; else fixed value is set
	*/
	bool is_voxel_size_detected;

	/**
	* \brief if false, point density is auto detected else fixed value is set
	*/
	bool is_point_density_set;

	/**
	* \brief point density for normalEstimation, invalid while is_point_density_set is false
	*/
	float point_density;
};

#endif