/* Copyright (c) ASTRI, 2020. All rights reserved.
 * This software is proprietary to and embodies the confidential technology
 * of Hong Kong Applied Science and Technology Research Institute Company
 * Limited (ASTRI).
 *
 * Possession, use, or copying of this software and media is authorized
 * only pursuant to a valid written license from ASTRI or an authorized
 * sublicensor.
 *
 * Author:mounty
 * Date: 20200730
 * Description: Points Group Plane Class Definitions
 */

#ifndef PARENT_VOXEL_STRUCT_H
#define PARENT_VOXEL_STRUCT_H

 /**
 * \brief  data struct of updated elements for parent voxel identify
 */
struct ParentVoxelIdentifyUpdateItem
{
	/**
	* \brief largest good_neighbor_count of current 's parent voxel
	*/
	unsigned char largest_good_neighbor_count;
	/**
	* \brief  smallest bad_neighbor_count of current 's parent voxel
	*/
	unsigned char smallest_bad_neighbor_count;
	/**
	* \brief good_neighbor_count of neighbor's parent voxel
	*/
	unsigned char neighbor_good_neighbor_count;
	/**
	* \brief  bad_neighbor_count of neighbor's parent voxel
	*/
	unsigned char neighbor_bad_neighbor_count;
	/**
	* \brief smallest mean square error of current's parent voxel
	*/
	float smallest_mse;
	/**
	* \brief the grid index of current voxel
	*/
	unsigned int  grid_idx;
	/**
	* \brief current's parent voxel index to occupied voxel array
	*/
	unsigned int current_parent_voxel_idx;
	/**
	* \brief neighbor's parent voxel index to occupied voxel array
	*/
	unsigned int neighbor_parent_voxel_idx;
	/**
	* \brief  mean square error of neighbor's parent voxel
	*/
	float neighbor_mse;
	/**
	* \brief the grid index of neighbor's parent voxel
	*/
	unsigned int  neighbor_grid_idx;
	/**
	* \brief the parent points group index of current's parent voxel
	*/
	unsigned int current_parent_group_idx;
	/**
	* \brief the parent points group index of neighbor's parent voxel
	*/
	unsigned int  neighbor_parent_group_idx;
	/**
	* \brief the ratio of points whose mse > threshold VS all the points
	*/
	//float high_mse_ratio;
	/**
	* \brief the ratio of points whose mse > threshold VS all the points
	*/
	//float neighbor_high_mse_ratio;
};

/**
* \brief  parent voxel identify condition enum type
*/
enum ParentIDCondType
{
	/**
	* \brief good neighbour count
	*/
	GOOD_NEIGHBOR_CNT = 0,
	/**
	* \brief smallest mormal difference between current voxels and its neighbours
	*/
	SMALLEST_NORMAL_DIFF = 1,
	/**
	* \brief smallest plane distance of the neigbours to the current voxel
	*/
	SMALLEST_PLANE_DIST = 2,
	/**
	* \brief plane mean squre errors of the current voxel
	*/
	PLANE_MSE = 3,
	/**
	* \brief number of points of the current voxel
	*/
	NUM_OF_POINT = 4,
	/**
	* \brief the  distance  to the center of current voxel
	*/
	CENTER_DIST = 5,
	/**
	* \brief the grid index of current voxels
	*/
	GRID_IDX = 6,
	/**
	* \brief the ratio of points whose mse > threshold  vs all the points  of current voxel
	*/
	HIGH_MSE_RATIO = 7,
	/**
	* \brief bad neighbour count
	*/
	BAD_NEIGHBOR_CNT = 8,
	/**
	* \brief group index for good points group indentify
	*/
	GROUP_IDX = 9,
	/**
	* \briefinvalid parameters
	*/
	INVALID_PARA
};

/**
* \brief  point array type in planes, same as point array type in voxels
*/
typedef PointInVoxelArray PointInPlaneArray;

/**
* \brief  plane item type distingushed by parent voxel
*/
enum PlaneItemType
{
	/**
	* \brief plane is in valid, just for initial
	*/
	INVALID_PLANE = 0,
	/**
	* \brief plane whose parent voxel type is good and is_overall_merged is true
	*/
	GOOD_PLANE = 1,
	/**
	* \brief plane whose parent voxel type is pseudobad and is_overall_merged is true
	*/
	PSEUDO_BAD_PLANE = 2,
	/**
	* \brief plane whose parent voxel is merged by point-to-plane mode
	*/
	REAL_BAD_PLANE = 3
};

/**
* \brief  Each plane merge output item
*/
struct PlaneMergeOutputItem
{
	/**
	* \brief plane item type distingushed by parent voxel
	*/
	PlaneItemType plane_type;

	/**
	* \brief voxel root id of a plane, it is the final parent voxel id after plane merging
	*/
	unsigned int parent_voxel_idx;

	/**
	* \brief total number of points in this plane
	*/
	unsigned int total_point_cnt;

	/**
	* \brief line mean square error
	*/
	float plane_mse;

	/**
	* \brief center point of all the points in the plane
	*/
	ModuleStruct::Point3f plane_center;

	/**
	* \brief normal of all the points in the plane
	*/
	ModuleStruct::Point3f plane_normal;

	/**
	* \brief covariance of all the points in the plane
	*/
	SumforCovariance sums;

	/**
	* \brief all the good and pseudo bad voxels who are merging into this plane
	*/
	VoxelArray voxels;

	/**
	* \brief  the real bad points who are merging into this plane in MergeBadVoxels
	*/
	PointInPlaneArray points;

	/**
	* \brief extended parts  points of plane  who are merging into this plane in MergeExtendedPartWithRefPlane
	*/
	PointInPlaneArray extended_part_points;

	/**
	* \brief edge points of plane  whose is_in_multiplane is true  merging in MergeMultiplaneEdgePoints
	*/
	PointInPlaneArray multiplane_points;
};

/**
* \brief  plane merge output item array struct
*/
struct PlaneMergeOutput
{
	PlaneMergeOutputItem* planes;
	unsigned int size;
};

#endif // PARENT_VOXEL_STRUCT_H
