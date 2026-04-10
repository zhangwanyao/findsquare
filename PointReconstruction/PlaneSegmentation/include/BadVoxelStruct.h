#ifndef BAD_VOXEL_STRUCT_H
#define BAD_VOXEL_STRUCT_H

#include "config.h"
#include "DataStruct.h"

/**
* \brief data struct of points group neighbor item
*/
struct NeighborPointsGroupItem
{
	/**
	* \brief if this value < top_k show current group have good neighbor group and the value is  good neighbor group index of this neighbor voxel
	*/
	unsigned int points_group_idx;

	/**
	* \brief if is_good_group is ture show current group is good neighbor group , if point cloud is thick, may be have multiple good group;
	*/
	bool is_good_neighbor[MAX_NUM_OF_PLANE_IN_BAD_VOXEL];
};

/**
* \brief the bool array of whether in same plane for points group  and reference planes
*/
struct IsSameWithRefPlaneArray
{
	/**
	* \brief the bool array elements of whether in same plane for points group  and reference planes
	*/
	bool * is_same_with_ref_plane;

	/**
	* \brief size of bool array elements
	*/
	int ref_plane_size;
};
/**
* \brief data struct of points group item
*/
struct PointsGroupsItem
{
	/**
	* \brief points group parent voxel , ping pong buffer
	*/
	unsigned int parent_voxel_idx[2];
	/**
	* \brief  points group parent group index of parent voxel, ping pong buffer
	*/
	unsigned int parent_group_idx[2];
	/**
	* \brief good neighbor count
	*/
	unsigned int good_neighbor_cnt;
	/**
	* \brief mean square error of the plane
	*/
	float plane_mse;
	/**
	* \brief the ratio of points whose mse is higher than mse threshold
	*/
	float plane_high_mse_ratio;

	/**
	* \brief points group fitted plane normal
	*/
	ModuleStruct::Point3f  plane_normal;
	/**
	* \brief points group fitted plane center
	*/
	ModuleStruct::Point3f plane_center;

	/**
	* \brief sums for  points Covariance matrix in group
	*/
	SumforCovariance sums;

	/**
	* \brief all the points index of current group
	*/
	PointInVoxelArray points;

	/**
	* \brief all the neighbor voxel group info
	*/
	NeighborPointsGroupItem neighbour_points_group_item[26];

	/**
	* \brief all the points index in bad voxel of current group
	*/
	unsigned int* bad_voxel_point_idx;

	/**
	* \brief the bool array of reference plane relationshape of points group,  array size is equal to reference plane number
	*/
	IsSameWithRefPlaneArray same_with_ref_plane_array;

	/**
	* \brief  replace the current voxels is a good group and can be taken part in the identifying parent of good points groups
	*/
	bool  is_good_group;

	/**
	* \brief  the closest reference plane index of this group
	*/
	unsigned int closest_ref_plane_idx;
};

/**
* \brief  data struct of bad voxel points group
*/
struct BadVoxelPointsGroup
{
	PointsGroupsItem similar_points_group_item[MAX_NUM_OF_PLANE_IN_BAD_VOXEL];
};

/**
* \brief  data struct of merge output item  for voxels whose is_overall_merged are false
*/
struct BadVoxelMergeOutpuItem
{
	/**
	* \brief real bad voxel index to occupied voxel array
	*/
	unsigned int bad_voxel_idx;

	/**
	* \brief point size of this real bad voxel
	*/
	unsigned int voxel_point_size;

	/**
	* \brief true show this real bad voxel has points to be merged to a plane
	*/
	bool is_being_merged;

	/**
	* \brief index to PlaneMergeOutput planes, record the closest plane of this real bad voxel points
	*/
	unsigned int* closest_plane_idx;

	/**
	* \brief smallest distance to all the planes of this real bad voxel points
	*/
	float* smallest_dist;

	/**
	* \brief  point merged  flag  of this real bad voxel points
	*/
	bool* point_merged_flag;

	BadVoxelMergeOutpuItem** best_merge_plane_voxel;
	unsigned int* best_merge_point_id;
	unsigned int* best_merge_plane_id;

	/**
	* \brief size is equel to number of all the previous planes generated before
	*if it is true show  that this voxel have remaining points being merged  in reference planes by plane dist < threshold
	* for example, if remaining_points_merged_plane[0] == true,show this voxel have points in the plane whose plane index is 0
	* if remaining_points_merged_plane[1] = false ,show this voxel have no point  in the plane whose plane index is 1
	*/
	bool *remaining_points_merged_plane;

	/**
	* \brief the number of the Previously  found planes who have the same normal with this voxel
	*/
	unsigned int same_normal_plane_cnt;

	/**
	* \brief 	the number of the  points who have being merged in the plane
	*/
	unsigned int num_of_points_merged;

	/**
	* \brief flag show if it still  have points not being merged
	*/
	bool is_remainer_occupied;

	/**
	* \brief record the group index  of points whose normal is similar with the current point,size is equal to points size of the voxel;
	*/
	unsigned int* remainer_similar_group_idx;

	/**
	* \brief record the size of points group  whose normal is similar
	*/
	unsigned int remainer_similar_group_size;

	/**
	* \brief size is equel to points_group_plane_merge_out.size flag for all the remaining group plane, if it is true show
	*that this voxel have remaining points being merged  as corresponding planes by plane dist < threshold while based_plane is false;
	*/
	bool *being_in_group_plane;

	/**
	* \brief size is equel to points_group_plane_merge_out.size flag for all the group plane who is extension of the previous plane, if it is true show
	*that this voxel have remaining points being merged  as corresponding planes by plane dist < threshold while based_plane is true;
	*/
	//bool *being_in_extended_plane;

	/**
	* \brief the points group of bad voxel  if  is_remainer_occupied is true
	*/
	BadVoxelPointsGroup* points_group;

	/**
	* \brief true show it is connected with its neighbor, otherwise false, note: this flag in only for points group clustering in remaining points
	*/
	bool neighbor_connected[26];

	/**
	* \brief true show its remaining points are connected with reference planes, otherwise false ,size is equal to total plane size
	*/
	bool *is_plane_connected;

	/**
	* \brief true show  its merging points are connected with the reference plane, otherwise false ,size is equal to total plane size
	*/
	bool* is_multi_plane_connected;
};

#endif // BAD_VOXEL_STRUCT_H
