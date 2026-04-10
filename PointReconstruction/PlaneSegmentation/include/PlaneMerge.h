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
 * Date: 20200911
 * Description: PlaneMerge Class Definitions
 */

#ifndef PLANE_MERGE_H
#define PLANE_MERGE_H

#include "config.h"
#include "DataStruct.h"
#include "BadVoxelStruct.h"
#include "VoxelDbgStruct.h"
#include "ParentVoxelStruct.h"
#include "planeConfigParse.h"

class PlaneSegmentation;
#define project_point2plane(p, n, o) (p - (n.dot(p - o)) * n)

/**
* \brief the same planes  group index array
*/
struct SamePlaneGroupArray
{
	/**
	* \brief  record the same planes  group index of all merging-out  planes , if -1 show have no same plane
	*/
	unsigned int* same_plane_group_idx;
	/**
	* \brief   size of all the planes to be checked if  have same  planes (both  having same normal and being connected)
	*/
	unsigned int  plane_size;
};

/**
* \brief the same and connected  relationshape  array
*/
struct SameAndConnectedArray
{
	/**
	* \brief  record the connected  relationship for each pair of planes
	*/
	bool **is_connected;

	/**
	* \brief  record the same  relationship for each pair of planes
	*/
	bool **is_same_plane;
};

/**
* \brief class of points group plane
*/
class PlaneMerge {
public:

	/**
	* \brief  constructor class of points group plane
	*/
	PlaneMerge(PlaneSegmentation *plane_segmentation);

	/**
	* \brief destructor
	*/
	~PlaneMerge();

	/**
	* \brief main process of plane merge
	* @ PlaneMergeOutput *plane_array, input parameters, plane array to be merged
	* @SamePlaneGroupArray *same_plane_array, input and output parameters, pointer to same plane array
	* @ return true if process succes otherwise return false
	*/
	bool MergePlanes(const bool no_plane_merge, PlaneMergeOutputItem* planes, const unsigned int plane_size, SamePlaneGroupArray* same_plane_array, SameAndConnectedArray* same_connected, bool last_merge = false);

	/**
	* \brief pointer of instance of class PlaneSegmentation
	*/
	PlaneSegmentation *plane_seg_ptr;

private:

	/**
	* \brief Plane array to be merged
	*/
	PlaneMergeOutput plane_merge_array;

	/**
	* \brief system control parameters
	*/
	SysCntrlParams sys_control_para;

	/**
	* \brief Plane segmentation parameter configure
	*/
	SegCntrlParams config_params;

	/**
	* \brief voxel length in x, y and z directions
	*/
	VoxelParams voxel_params;

	/**
	* \brief Predefined threshold for plane segmentation
	*/
	PlaneFitThresholds plane_seg_thresholds;

	/**
	* \brief Occupied Voxel Array
	*/
	PlaneVoxelArray plane_voxel_array;

	/**
	* \brief Plane merge elements, the size is the same as plane_voxel_array
	*/
	PlaneMergeItem* plane_merge_element;

	/**
	* \brief input point cloud x,y,z
	*/
	PointArray pt_cloud_xyz;

	/**
	* \brief input point normal
	*/
	PointArray pt_cloud_normal;

	/**
	* \brief Total number of  voxels who may be merged by point to plane in occupied voxel array
	* include parts of pseudobad voxels will  before IdentifyParentOfPseudoBadVoxels
	*/
	unsigned int point_merged_voxel_size;

	/**
	* \brief real bad voxel merge output info
	*/
	BadVoxelMergeOutpuItem* bad_voxel_merge_item;

	/**
	* \brief voxel index of occupied voxel array to plane index
	*/
	unsigned int *voxel_to_plane_idx;

	/**
	* \brief occupied voxel to bad voxel index
	*/
	unsigned int* voxel_idx_to_bad_voxel;

	SamePlaneGroupArray same_plane_group_array; // the same planes  group index array

	/**
	* \brief Total number of planes whose parent voxel  is good voxel or pseudobad voxel
	*/
	unsigned int total_flat_plane_size;

	/**
	* \brief Total number of planes which taking part in merging planes
	*/
	unsigned int total_plane_size;

	/**
	* \brief max number of existing planes
	*/
	unsigned int max_plane_size;

	/**
	* \brief debug control parammeters
	*/
	DebugConfigParams debug_config_params;

	/**
	* \brief the output path
	*/
	std::string data_output_path;

private:

	/**
	* \brief initilize function of elements
	*/
	void init();
	/**
	* \brief release memory
	*/
	void FreeMemory();

	//void AddPlaneToSamePlaneGroup(unsigned int plane_in_group, const unsigned int total_plane_size, bool**is_same_plane, bool**is_connected);
	void AddPlaneToSamePlaneGroup(const unsigned int plane_in_group, const unsigned int same_group_idx, const unsigned int total_plane_size, bool** is_same_plane, bool** is_connected);

	// find the planes  with both  same normal  and  plane distance < threshold
	void IdentifySameNormalPlanes(bool **is_same_plane, unsigned int plane_size, bool last_merge = false);

	void IdentifyConnectedPlanes(bool ** is_connected, unsigned int plane_size);

	/**
	* \brief identify the connected relationship for  bad voxel and found planes, thread-safe is required
	*/
	void IdentifyConnectedPlanesFromBadVoxel(bool **is_connected, const unsigned int bad_voxel_idx);
	void IdentifyConnectedPlanesFromNeighbors(bool **is_connected, const unsigned int bad_voxel_idx, const unsigned int first_plane_idx\
		, const std::vector<unsigned int> first_points);

	void IdentifyConnectedPlanesBypoints(bool** is_same_plane, bool** is_connected);

};

#endif// PLANE_MERGE_H
