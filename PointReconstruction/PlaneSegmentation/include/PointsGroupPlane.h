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
 * Date: 20200801
 * Description: Points Group Plane Class Definitions
 */

#ifndef POINTS_GROUP_PLANE_H
#define POINTS_GROUP_PLANE_H

#include "config.h"
#include "log.h"
#include "DataStruct.h"
#include "BadVoxelStruct.h"
#include "VoxelDbgStruct.h"
#include "ParentVoxelStruct.h"
#include "plane_seg_inf.h"
 /**
 * \brief  data struct of  points group array
 */
struct PointsGroupArray
{
	/**
	* \brief voxel index and group index of a pionts group,  first  is voxel index ,second is group index
	*/
	std::pair<unsigned int, unsigned int> *points_group_id;

	/**
	* \brief size of the points_voxel_group
	*/
	unsigned int size;
};

/**
* \brief  data struct of  elements for parent points group identify
*/
struct PointsGroupParentPlaneItem
{
	/**
	* \brief voxel root id of a plane, it is the final parent voxel id after plane merging
	*/
	unsigned int parent_voxel_idx;

	/**
	* \brief parent voxel's parent group index of a ponts group plane
	*/
	unsigned int parent_group_idx;

	/**
	* \brief points group fitted plane center
	*/
	ModuleStruct::Point3f plane_center;

	/**
	* \brief points group fitted plane normal
	*/
	ModuleStruct::Point3f plane_normal;

	/**
	* \brief mean square errors of points group fitted plane
	*/
	float plane_mse;

	/**
	* \brief eigen mean square errors of points group fitted plane
	*/
	float eigen_mse;

	/**
	* \brief the ratio of points whose mse is higher than mse threshold
	*/
	float high_mse_ratio;

	/**
	* \brief sums for  points Covariance matrix of plane
	*/
	SumforCovariance sums;

	/**
	* \brief all the voxels index of current points group plane
	*/
	//VoxelArray voxels;
	PointsGroupArray groups;

	/**
	* \brief all the points index of current plane's good groups
	*/
	PointInVoxelArray points;

	/**
	* \brief all the points index in bad group of current plane
	*/
	//PointInVoxelArray bad_group_points;

	/**
	* \brief  if points group plane is extended part of a refence plane ,record the plane idx here(note: it is not global plane idx)
	*/
	unsigned int ref_plane_idx;
};

/**
* \brief data struct of points group plane array
*/
struct PointsGroupPlaneArray
{
	/**
	* \brief data struct of points group plane
	*/
	PointsGroupParentPlaneItem* planes;
	/**
	* \brief size of points group plane
	*/
	unsigned int size;
};

class PlaneSegmentation;

/**
* \brief class of points group plane
*/
class PointsGroupPlane {
public:

	/**
	* \brief  constructor class of points group plane
	*/
	PointsGroupPlane(PlaneSegmentation *plane_segmentation);

	/**
	* \brief destructor
	*/
	~PointsGroupPlane();

	/**
	* \brief main process of points group plane identifying
	* @const bool based_on_plane, input parameters, jf true show the process based on the reference plane
	* @const PlaneMergeOutput * base_planes, input parameters, reference plane  for points group plane identifying
	* @PointsGroupPlaneArray *points_group_planes, output parameters , found planes in remaining points
	*/
	void PointsGroupPlaneMergeFromRemainer(const bool based_on_plane, const PlaneMergeOutput * base_planes, PointsGroupPlaneArray *points_group_planes);

	/**
	* \brief pointer of instance of class PlaneSegmentation
	*/
	PlaneSegmentation * plane_seg_ptr;

private:

	/**
	* \brief system control parameter setting
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
	* \brief input point cloud x,y,z
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

	/**
	* \brief found planes in points groups of bad voxel
	*/
	PointsGroupPlaneArray points_group_plane_merge_out;

	/**
	* \brief record the plane index from parent voxel idx and parent group index
	*/
	unsigned int(*parent_group_to_plane_idx)[MAX_NUM_OF_PLANE_IN_BAD_VOXEL];

	/**
	* \brief total found planes for good voxel planes and pseudobad voxel planes
	*/
	unsigned int total_flat_plane_size;

	/**
	* \brief Total number of planes whose parent voxel  is good voxel
	*/
	unsigned int good_voxel_plane_size;

	/**
	* \brief Total number of planes whose parent voxel  is good voxel
	*/
	unsigned int pseudobad_voxel_plane_size;

	/**
	* \brief global reference plane index vector,  pseudobad planes index must add the good_voxel_plane_size
	*/
	std::vector<unsigned int> global_ref_plane_idx;

	/**
	* \brief the mse threhhold of mini plane whose flat voxels is few
	*/
	float max_mse_of_mini_plane;

	/**
	* \brief debug control parammeters
	*/
	DebugConfigParams debug_config_params;

	/**
	* \brief the ratio threshold of points whose mse is higher than mse threshold
	*/
	float min_high_mse_ratio;

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
	void FreeMemory(bool is_init);

	/**
	* \brief get mse of points group plane
	*/
	bool GetPlaneDistMse(PointsGroupParentPlaneItem* plane, float& plane_mse);
	/**
	* \brief condition compare function for parent group identifying
	*/
	bool NeighborParentCompare(ParentIDCondType *condition, ParentVoxelIdentifyUpdateItem * parent_voxel_id_update_it, unsigned char &update_flag);

	/**
	* \brief compute normal difference for each bad voxel who have points not merged
	*/
	bool GetRemainerMaxSimilarPointsIdx(unsigned int bad_voxel_idx, unsigned int* similar_point_cnt_list, unsigned int &bad_point_idx);

	/**
	* \brief find all the points based on the input reference planes, and assign them points group index.
	* Note : here same reference  planes are allowed, so
	* a points group may have multiple based planes, so it should not need to record the reference plane for each group ,and
	* must check all the refernce planes to identify the connection relationshap. if input planes have no same planes,  it only need to
	* check the current same planes to identify the connection relationshap
	* @param const PlaneMergeOutput *exist_planes , input parameter, refence planes based for find points groups
	*/
	void GetRemainerNormalDiffByPlanes(const PlaneMergeOutput *exist_planes);

	/**
	* \brief points group initial
	*/
	void BadVoxelGroupInit();

	/**
	* \briefcompute normal difference for each bad voxel who have points not merged
	*/
	void GetRemainerNormalDiff();

	/**
	* \brief group the top k normal similary points for each voxel
	*/
	void GroupRemainerNormalDiffTopk();

	/**
	* \brief compute the neighbours info of points group
	*/
	void AssignRemainerNeighbors();

	/**
	* \brief compute the neighbours info of points group with reference planes
	*/
	void AssignRemainerNeighborsWithRef(const PlaneMergeOutput *exist_planes);

	/**
	* \brief get exist_planes same plane information
	*/
	bool getPlaneSameInfo(const PlaneMergeOutput* exist_planes, bool ** &is_same_plane);

	/**
	* \brief Identify parent for all the good points groups of bad voxels
	*/
	bool IdentifyRemainerGoodGroupParent();

	/**
	* \brief Identify parent for all the good points groups  with good points group neighbor
	*/
	bool IdentifyRemainerGoodGroupParentByGoodneighbor();

	/**
	* \brief plane merge from the groups of bad voxels whose group mse < threshold
	*/
	void MergeRemainerGoodGroup(const bool based_on_plane, const PlaneMergeOutput *exist_planes);

	/**
	* \brief Identify parent for all the bad points groups of bad voxels
	*/
	//bool IdentifyRemainerBadGroupParent(bool is_first);

	/**
	* \brief points merge from the groups of of bad voxels whose group mse >= threshold
	*/
	//void MergeRemainerBadGroup();

	/**
	* \brief debug info for points group neighbors
	*/
	bool GetPointsGroupNeigbourDebugInfo(const bool based_on_plane);

	/**
	* \brief compute normal and mse of points group and reshape it if based_on_plane is false
	*/
	void ReshapeGroupByPlaneDist(const bool based_on_plane);

	/**
	* \brief points group debug information saving by output path "group_xyz" and "group_xyz_info"
	*/
	void PointsGroupDebug();

	/**
	* \brief debug for save points group plane points by voxel name and plane_idx
	*/
	bool SaveGroupPlanePointsDebug(const std::string output_path, unsigned int plane_idx);

	/**
	* \brief Identify connected relationshap with reference planes of the points group plane specified parent group.
	* @param bool &is_same_connected,  out parameter, is same and connected  with exist_planes of the points group plane
	* @param const unsigned int bad_voxel_idx,  input parameter, bad voxel index
	* @param const unsigned int group_idx,  input parameter, group index
	* @param const PlaneMergeOutput *exist_planes , input parameter, refence planes based for find points groups
	* @ return true if process succes otherwise return false
	*/
	bool IdentifyParentPointsGroupConnectedWithRef(bool &is_same_connected, unsigned int &plane_idx, const PointsGroupParentPlaneItem* group_plane, const PlaneMergeOutput *exist_planes);

	bool IdentifyConnectedPlaneFromNeighbor(bool &is_same_connected, unsigned int &plane_idx, const PointsGroupsItem *points_group, const unsigned int bad_voxel_idx, const PlaneMergeOutput *exist_planes);

	bool IdentifyConnectedPlaneFromBadVoxel(bool &is_same_connected, unsigned int &plane_idx, const PointsGroupsItem *points_group, const unsigned int bad_voxel_idx, const PlaneMergeOutput *exist_planes);

	bool FilteredGroupPlaneWithRef(const PlaneMergeOutput *exist_planes, const unsigned int group_plane_size, PointsGroupParentPlaneItem* group_planes, bool * is_fitted_plane);

	bool ComputeAvgNormal(const unsigned int parent_voxel_idx, const unsigned int parent_group_idx, ModuleStruct::Point3f &avg_normal);

#ifdef SAVE_OUTPUT_FILE_DEBUG
	/**
	* \brief merging points group debug info in merge_points_group.txt
	*/
	void MergePointsGroupDebugOutput(std::string output_path, PlaneItemType ref_plane_type);
#endif

	void pt_info(unsigned int point_ids, unsigned int voxel_idx, const char* file, int line)
	{
		if (voxel_idx > plane_voxel_array.size)
		{
			log_error("input voxel_idx =%d exceed size of array =%d", voxel_idx, plane_voxel_array.size);
			return;
		}
		if (point_ids > pt_cloud_xyz.size)
		{
			log_error("input voxel_idx =%d exceed size of array =%d", point_ids, pt_cloud_xyz.size);
			return;
		}

		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[voxel_idx];

		if (plane_voxel->is_overall_merged)
		{
			log_log(LOG_DEBUG, file, line, "point idx%d voxel idx(overall) %d is_being_merged =%d", point_ids, voxel_idx, plane_voxel->is_being_merged);
		}
		else
		{
			if (bad_voxel_merge_item != NULL)
			{
				unsigned int bad_voxel_ids = voxel_idx_to_bad_voxel[voxel_idx];

				if (bad_voxel_ids > point_merged_voxel_size)
				{
					log_error("input voxel_idx %d bad_voxel_ids =%d error", voxel_idx, bad_voxel_ids);
				}
				else
				{
					BadVoxelMergeOutpuItem* bad_vxl_it = &bad_voxel_merge_item[bad_voxel_ids];
					for (unsigned int i = 0; i < plane_voxel->points.size; i++)
					{
						if (plane_voxel->points.point_idx[i] == point_ids)
						{
							log_log(LOG_DEBUG, file, line, "point idx%d voxel idx %d bad voxel%d is_being_merged =%d plane_idx =%d", point_ids, voxel_idx, bad_voxel_ids, bad_vxl_it->point_merged_flag[i], bad_vxl_it->closest_plane_idx[i]);
						}
					}
				}
			}
			else
			{
				log_error("bad_voxel_merge_item is NULL");
			}
		}
	}
#define PT_DEBUG(point_ids,vxl_ids,file,line) pt_info(point_ids,vxl_ids,file,line)
	//PT_DEBUG(1517971,21087,__FILE__,__LINE__);
};

#endif// POINTS_GROUP_PLANE_H
