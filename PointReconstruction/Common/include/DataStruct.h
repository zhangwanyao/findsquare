/* Copyright (c) ASTRI, 2019. All rights reserved.
 * This software is proprietary to and embodies the confidential technology
 * of Hong Kong Applied Science and Technology Research Institute Company
 * Limited (ASTRI).
 *
 * Possession, use, or copying of this software and media is authorized
 * only pursuant to a valid written license from ASTRI or an authorized
 * sublicensor.
 *
 * Author: Elaine Li, Chongshan Liu
 * Date: 20190807
 * Description: Data Structure Definitions
 */

#ifndef DATASTRUCT_H_
#define DATASTRUCT_H_

 //suggested opencv version: 4.3.0
 //#include <opencv2/core.hpp>
#include <unordered_map>
#include "ModuleStruct.hpp"
#include "common_struct.h"

typedef double IntermediateType;		// for intermediate result of computing Covariance

// Each occupied voxel with its element
struct VoxelItem
{
	// for single voxel
	// True - Good Voxel, False - Bad Voxel
	bool is_good_voxel;

	// grid index: index to 3d space, contains both empty and occupied voxels
	unsigned int grid_idx;

	// point index in voxel
	PointInVoxelArray points;
};

enum PlaneVoxelType
{
	VOXEL_INVALID_TYPE = 0,
	GOOD_VOXEL = 1,
	PSEUDO_BAD_VOXEL = 2,
	REAL_BAD_VOXEL = 3
};

// The middle value to compute Covariance
struct SumforCovariance
{
	IntermediateType sum_x;
	IntermediateType sum_y;
	IntermediateType sum_z;
	IntermediateType sum_xx;
	IntermediateType sum_yy;
	IntermediateType sum_zz;
	IntermediateType sum_xy;
	IntermediateType sum_xz;
	IntermediateType sum_yz;
};

// Each neighbor voxel with its element
struct NeighborItem
{
	// occupancy, false - Empty Voxel, true - Occupied Voxel
	bool is_occupied;

	// neighbor voxel index
	unsigned int voxel_idx;			// index to occupied voxel array

	// voxel type: 0 - Good Voxel, 1 - Pseduo Bad Voxel, 2 - Bad Voxel
	//PlaneVoxelType voxel_type;

	// normal difference between current voxel and the neighbour
	float normal_diff;

	// plane distance  between current voxel and the neighbour
	float plane_dist;

	// neighbor indication: true - ((normal_diff < Nt) && (plane_dist < Dt)) where Nt and Dt is pre-defined thresholds, otherwise equals to false
	bool neighbor_flag;

	//balaneced neighbor idication ,which normal diff is from all the neighbor_flag voxel info and recompute normal
	//bool balanced_neighbor_flag;

	// if<13 show  this voxel is a bridge , and this direction neighbour of this bridge voxel is in a bridge plane ,and the value is the plane index  of this voxel in the bridge
	unsigned int plane_idx_of_bridge;

	// if <13 show this voxel is a neighbour of a bridge voxel, and this value is the plane index of this neighbor direction's bridge (a voxel maybe have multiple bridges)
	unsigned int bridge_plane_idx_of_plane_pair;

	// true show neighbour voxel distance is close and connected with this voxel
	bool is_connected;
};

// Each occupied voxel with its element in plane segmentation
struct PlaneVoxelItem
{
	// voxel type: 1 - Good Voxel, 2 - Pseduo Bad Voxel, 3 - Bad Voxel
	PlaneVoxelType voxel_type;

	// grid index: index to 3d space, contains both empty and occupied voxels
	unsigned int grid_idx;

	// plane mean squared error
	float plane_mse;

	// the ratio of points whose mse is more than threshold
	float plane_high_mse_ratio;

	// plane mean squared error by eigen
	float avg_mse;

	// the ratio of points whose mse is more than threshold according to average normal
	float avg_high_mse_ratio;

	// plane center
	ModuleStruct::Point3f plane_center;

	// plane normal
	ModuleStruct::Point3f plane_normal;

	// plane avg normal (if not flat voxel, avg normal is equal to plane_normal)
	ModuleStruct::Point3f avg_normal;

	// covariance of all the points in current voxel
	SumforCovariance sums;

	// point index in voxel
	PointInVoxelArray points;

	// the flag for being merged in a plane, true is being merged
	bool is_being_merged;

	// the flag show this voxel can be merged overall into  a plane
	bool is_overall_merged;

	// the flag show this voxel points normal is all the same, sync from the normalEstimation module
	bool is_good_normal;

	// the flag show this voxel is flat
	bool is_flat;

	/*if voxel is tiny, it can only be used a bad voxel to be merged by point to voxel, it is neither good neighbor nor bad neighbor*/
	bool is_tiny;

	// flag if voxel is same with reference plane, for merging pseudobad voxels connected with good voxels
	bool *is_same_with_ref_plane;

	unsigned int ref_plane_cnt;
	// neighbor voxel elements
	// 0  - (X-1, Y-1, Z+1), 1  - (X  , Y-1, Z+1), 2  - (X+1, Y-1, Z+1),
	// 3  - (X-1, Y  , Z+1), 4  - (X  , Y  , Z+1), 5  - (X+1, Y  , Z+1),
	// 6  - (X-1, Y+1, Z+1), 7  - (X  , Y+1, Z+1), 8  - (X+1, Y+1, Z+1),
	// 9  - (X-1, Y-1, Z  ), 10 - (X  , Y-1, Z  ), 11 - (X+1, Y-1, Z  ),
	// 12 - (X-1, Y  , Z  ), C  - (X  , Y  , Z  ), 13 - (X+1, Y  , Z  ),
	// 14 - (X-1, Y+1, Z  ), 15 - (X  , Y+1, Z  ), 16 - (X+1, Y+1, Z  ),
	// 17 - (X-1, Y-1, Z-1), 18 - (X  , Y-1, Z-1), 19 - (X+1, Y-1, Z-1),
	// 20 - (X-1, Y  , Z-1), 21 - (X  , Y  , Z-1), 22 - (X+1, Y  , Z-1),
	// 23 - (X-1, Y+1, Z-1), 24 - (X  , Y+1, Z-1), 25 - (X+1, Y+1, Z-1)
	//
	// define movement
	// int movement_x[26] = {-1,  0,  1, -1,  0,  1, -1,  0,  1, -1,  0,  1, -1,  1, -1,  0,  1, -1,  0,  1, -1,  0,  1, -1,  0,  1};
	// int movement_y[26] = {-1, -1, -1,  0,  0,  0,  1,  1,  1, -1, -1, -1,  0,  0,  1,  1,  1, -1, -1, -1,  0,  0,  0,  1,  1,  1};
	// int movement_z[26] = { 1,  1,  1,  1,  1,  1,  1,  1,  1,  0,  0,  0,  0,  0,  0,  0,  0, -1, -1, -1, -1, -1, -1, -1, -1, -1};
	NeighborItem neighbors[26];
};

// Occpuied Voxel Array
struct PlaneVoxelArray {
	PlaneVoxelItem* voxels;
	unsigned int size;
};

// Bridge is psudo bad or real bad voxel, it connects two pseudo bad voxels or good voxels  of a same plane but no fitted-plane neigbour
struct BridgeVoxelItem
{
	unsigned int num_of_planes;             // how many neighbour planes of this bridge
	unsigned int(*parent_voxel_idxs)[2];   // the ping pong parent voxel index of the neighbour planes of this bridge
};

// Elements to be used in plane merging
struct PlaneMergeItem
{
	// smallest normal difference value between current voxel and its neighbors
	float smallest_normal_diff;

	// smallest plane distance value between current voxel and its neighbors
	float smallest_plane_dist;

	// use in plane merge process, two parent index buffer is ping-pong buffer to escape self voxel writing conflict with neighbor's reading
	unsigned int parent_voxel_idx[2];

	// count the neighbor voxels which is a good voxel, normal difference and the plane to plane distance are within pre-defined threshold
	// only use in good voxel growing, init to 0 before plane merge process
	unsigned int good_neighbor_count;
	unsigned int bad_neighbor_count;

	// the distance of plane to center of all 3D space points
	//float center_dist;

	//// the flag show this is the connected bridge voxel for two voxels who are in the same plane but  no  fitted-plane neigbour to connect
	//bool is_bridge;

	//// the flag show this is the  bridge voxel who only one fitted plane;
	//bool is_single_plane_bridge;

	//BridgeVoxelItem bridge_voxel_elements;
};

///**
//* \brief Predefined threshold from normal estimation configure file
//*/
//struct NormalEstimationThresholds
//{
//	/**
//	* \brief  point number threshold of a voxel points which have passibility to be a good normal voxel;
//	*/
//	unsigned int min_point_num_of_valid_normal_voxel;
//
//	/**
//	* \brief  0~90 normal difference threshold of two voxels who are the good neighbours
//	*/
//	float max_normal_angle_of_2voxel;
//
//	/**
//	* \brief  0.0~1.0,  mean squred error threshold  ratio to voxel size, if mse ratio > value, voxel is  not a flat voxel
//	*/
//	float max_mse_ratio_of_voxel;
//};

//dimensions of voxels
struct VoxelDimension
{
	int cols_of_voxel;
	int rows_of_voxel;
	int depths_of_voxel;
};

enum PlaneType
{
	PLANE_WALL = 0,
	PLANE_GROUND = 1,
	PLANE_CEILING = 2,
	PLANE_UNDEFINED = 3
};

// plane elements after merging
struct PlaneItem
{
	// voxel root id of a plane, it is the final parent voxel id after plane merging
	unsigned int parent_voxel_idx;		// index to occupied voxel array

	// plane type after plane classification
	PlaneType plane_type;

	// all the points who is merging into this plane
	PointInVoxelArray points;

	// center of all points in the plane
	ModuleStruct::Point3f plane_center;

	// plane normal of the plane
	ModuleStruct::Point3f plane_normal;

	// mean squred errors of all the points in the plane
	float plane_mse;

	// covariance of all the points in the plane
	SumforCovariance sums;

	/**
	* \brief area of the plane
	*/
	ModuleStruct::interim_value_type plane_area;
};

// Interface of Normal Estimation Module
struct NormalEstimationInterface
{
	// Data Type: InOut
	PointArray points;

	// Data Type: InOut
	PointArray normals;

	// Data Type: InPut
	//unsigned int* point_voxel_idx;

	// Data Type: Input
	NormalEstimationThresholds threshold;

	// Data Type: Input and output
	VoxelParams voxel_para;

	// Data Type: Output, voxel array which is occupied
	VoxelItem* voxel_array;

	// Data Type: Output, occupied voxel array size
	unsigned int voxel_array_size;

	// Data Type: Output, it is used to find out voxel_idx in occupied voxel array from grid_idx in 3D Space
	unsigned int* grid_to_occupied_voxel_idx;

	// Data Type: Output, total number of voxel grids in 3D Space
	unsigned int grid_to_occupied_voxel_idx_size;

	float density;
};

// Output of plane segmentation
struct PlaneSegmentationOutput
{
	// plane array after plane segmentation
	PlaneItem* planes;

	// plane number after plane segmentation
	unsigned int size;
};

//from point index  to  centriod index array with specifical leaf size in downsample process
struct Point2CentriodIdxArray
{
	unsigned int* centriod_idx;  //  centriod index of point index under specifical leaf size
	unsigned int size;
};

struct PointGrid2PointIdx
{
	unsigned long long grid_idx;
	unsigned int cloud_point_index;
	//PointGrid2PointIdx(unsigned int idx_, unsigned int cloud_point_index_) : grid_idx(idx_), cloud_point_index(cloud_point_index_) {}
	bool operator < (const PointGrid2PointIdx& p) const { return (grid_idx < p.grid_idx); }
};

///**
//* \brief data struct of normalEstimation contrl parameters
//*/
//struct NormalEstimationCntrlParam
//{
//	/**
//	* \brief if true  voxel size is auto detected; else fixed value is set
//	*/
//	bool is_voxel_size_detected;
//
//	/**
//	* \brief if false, point density is auto detected else fixed value is set
//	*/
//	bool is_point_density_set;
//
//	/**
//	* \brief point density for normalEstimation, invalid while is_point_density_set is false
//	*/
//	float point_density;
//};

// Fixed array of voxels in a plane
struct VoxelArray
{
	unsigned int* voxel_idx;			// index to occupied voxel array
	unsigned int size;
};

///**
//* \brief data struct of point index and delta distance
//*/
//struct PointIdx2Dist
//{
//	float delta_dist;
//	unsigned int cloud_point_idx;
//	bool operator < (const PointIdx2Dist& p) const { return (delta_dist < p.delta_dist); }
//};

/**
* \save bad points neighbour for normal estimation
*/
struct NormalEstimationPointsMap
{
	// int -> point index, std::vector<int> -> the neighbour points index
	std::unordered_map<unsigned int, std::vector<unsigned int>>  badPoint_map;
};

#endif	// DATASTRUCT_H_
