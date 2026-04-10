#include "config.h"
#include <fstream>
#include <iostream>
#include "PointsGroupPlane.h"
#include "log.h"
#include "MathOperation.hpp"
#include <iomanip>
#include "PlaneSegmentation.h"
#include "InOutData.h"
#include "in_out_data.hpp"
#include "util_time.hpp"

using namespace ModuleStruct;

PointsGroupPlane::PointsGroupPlane(PlaneSegmentation *plane_segmentation) {
	plane_seg_ptr = plane_segmentation;
	init();
	FreeMemory(true);
}

PointsGroupPlane::~PointsGroupPlane() {
	FreeMemory(false);
}


void PointsGroupPlane::init() {

	sys_control_para = plane_seg_ptr->sys_control_para;
	config_params = plane_seg_ptr->config_params;
	voxel_params = plane_seg_ptr->plane_seg_params.voxel_params;
	plane_seg_thresholds = plane_seg_ptr->plane_seg_params.plane_seg_thresholds;
	plane_voxel_array = plane_seg_ptr->plane_voxel_array;
	plane_merge_element = plane_seg_ptr->plane_merge_element;
	pt_cloud_xyz = plane_seg_ptr->pt_cloud_xyz;
	pt_cloud_normal = plane_seg_ptr->pt_cloud_normal;
	point_merged_voxel_size = plane_seg_ptr->point_merged_voxel_size;
	bad_voxel_merge_item = plane_seg_ptr->bad_voxel_merge_item;
	voxel_to_plane_idx = plane_seg_ptr->voxel_to_plane_idx;
	voxel_idx_to_bad_voxel = plane_seg_ptr->voxel_idx_to_bad_voxel;
	debug_config_params = plane_seg_ptr->debug_config_params;
	min_high_mse_ratio = plane_seg_ptr->min_high_mse_ratio;
	good_voxel_plane_size = plane_seg_ptr->good_voxel_plane_size;
	pseudobad_voxel_plane_size = plane_seg_ptr->pseudobad_voxel_plane_size;
	total_flat_plane_size = good_voxel_plane_size + pseudobad_voxel_plane_size;
	//data_output_path.clear();
	data_output_path.assign(plane_seg_ptr->current_out_path.c_str());
	global_ref_plane_idx.clear();
	global_ref_plane_idx.shrink_to_fit();
	points_group_plane_merge_out.size = 0;
	points_group_plane_merge_out.planes = NULL;
	parent_group_to_plane_idx = NULL;
	bad_voxel_merge_item->remainer_similar_group_idx = NULL;
}
void PointsGroupPlane::FreeMemory(bool is_init) {

	if (parent_group_to_plane_idx != NULL)
	{
		delete[] parent_group_to_plane_idx;
		parent_group_to_plane_idx = NULL;
	}

	if (bad_voxel_merge_item != NULL)
	{
		for (unsigned int i = 0; i < point_merged_voxel_size; i++)
		{
			BadVoxelMergeOutpuItem* bad_voxel_it = &bad_voxel_merge_item[i];
			if (bad_voxel_it->points_group != NULL)
			{
				for (unsigned int j = 0; j < MAX_NUM_OF_PLANE_IN_BAD_VOXEL; j++)
				{
					PointsGroupsItem* points_group_it = &bad_voxel_it->points_group->similar_points_group_item[j];
					//points_group_it->points.point_idx has been assigned to the points_group_plane_merge_out ,and not need to be released here

					if (is_init)
					{
						if (points_group_it->points.point_idx != NULL)
						{
							delete[] points_group_it->points.point_idx;
							points_group_it->points.point_idx = NULL;
						}

						if (points_group_it->bad_voxel_point_idx != NULL)
						{
							delete[] points_group_it->bad_voxel_point_idx;
							points_group_it->bad_voxel_point_idx = NULL;
						}
					}

					if (points_group_it->same_with_ref_plane_array.is_same_with_ref_plane != NULL)
					{
						delete[] points_group_it->same_with_ref_plane_array.is_same_with_ref_plane;
						points_group_it->same_with_ref_plane_array.is_same_with_ref_plane = NULL;
					}

				}


				delete bad_voxel_it->points_group;
				bad_voxel_it->points_group = NULL;
			}
		}
	}

	if (bad_voxel_merge_item->remainer_similar_group_idx != NULL)
	{
		delete[] bad_voxel_merge_item->remainer_similar_group_idx;
		bad_voxel_merge_item->remainer_similar_group_idx = NULL;
	}

	global_ref_plane_idx.clear();
	global_ref_plane_idx.shrink_to_fit();
}

bool PointsGroupPlane::GetPlaneDistMse(PointsGroupParentPlaneItem* plane, float& plane_mse)
{
	unsigned int point_cnt = 0;
	IntermediateType average_distance = 0.0;
	Point3f point;
	float plane_dist;

	for (unsigned int i = 0; i < plane->points.size; i++)
	{
		point = pt_cloud_xyz.points[plane->points.point_idx[i]];
		plane_dist = ComputePointToPlaneDist<float>(point, plane->plane_normal, plane->plane_center);
		average_distance += plane_dist;
		point_cnt++;
	}

	/*for (unsigned int i = 0; i < plane->bad_group_points.size; i++) {
		point = pt_cloud_xyz.points[plane->bad_group_points.point_idx[i]];
		plane_dist = MathOperation::ComputePointToPlaneDist<float>(point, plane->plane_normal, plane->plane_center);
		average_distance += plane_dist;
		point_cnt++;
	}*/

	plane_mse = (float)(average_distance / point_cnt);
	return true;
}
bool PointsGroupPlane::NeighborParentCompare(ParentIDCondType *condition, ParentVoxelIdentifyUpdateItem * parent_voxel_id_update_it, unsigned char &update_flag)
{
	bool neighbor_good_than_current[5] = { false };
	bool neighbor_equal_current[5] = { false };

	for (int i = 0; i < 5; i++)
	{
		switch (condition[i])
		{
		case GOOD_NEIGHBOR_CNT:
			neighbor_good_than_current[i] = (parent_voxel_id_update_it->neighbor_good_neighbor_count > parent_voxel_id_update_it->largest_good_neighbor_count) ? true : false;
			neighbor_equal_current[i] = (parent_voxel_id_update_it->neighbor_good_neighbor_count == parent_voxel_id_update_it->largest_good_neighbor_count) ? true : false;
			break;
			/*case SMALLEST_NORMAL_DIFF:
				neighbor_good_than_current[i] = (parent_voxel_id_update_it->neighbor_smallest_normal_diff < parent_voxel_id_update_it->smallest_normal_diff) ? true : false;
				neighbor_equal_current[i] = (parent_voxel_id_update_it->neighbor_smallest_normal_diff == parent_voxel_id_update_it->smallest_normal_diff) ? true : false;
				break;
			case SMALLEST_PLANE_DIST:
				neighbor_good_than_current[i] = (parent_voxel_id_update_it->neighbor_smallest_plane_dist < parent_voxel_id_update_it->smallest_plane_dist) ? true : false;
				neighbor_equal_current[i] = (parent_voxel_id_update_it->neighbor_smallest_plane_dist == parent_voxel_id_update_it->smallest_plane_dist) ? true : false;
				break;*/
		case PLANE_MSE:
			neighbor_good_than_current[i] = (parent_voxel_id_update_it->neighbor_mse < parent_voxel_id_update_it->smallest_mse) ? true : false;
			neighbor_equal_current[i] = (parent_voxel_id_update_it->neighbor_mse == parent_voxel_id_update_it->smallest_mse) ? true : false;
			break;
			/*case NUM_OF_POINT:
				neighbor_good_than_current[i] = (parent_voxel_id_update_it->neighbor_largest_point_cnt > parent_voxel_id_update_it->largest_point_cnt) ? true : false;
				neighbor_equal_current[i] = (parent_voxel_id_update_it->neighbor_largest_point_cnt == parent_voxel_id_update_it->largest_point_cnt) ? true : false;
				break;*/
				/*case CENTER_DIST:
					neighbor_good_than_current[i] = (parent_voxel_id_update_it->neighbor_center_dist < parent_voxel_id_update_it->center_dist) ? true : false;
					neighbor_equal_current[i] = (parent_voxel_id_update_it->neighbor_center_dist == parent_voxel_id_update_it->center_dist) ? true : false;
					break;*/

		case GRID_IDX:
			neighbor_good_than_current[i] = (parent_voxel_id_update_it->neighbor_grid_idx < parent_voxel_id_update_it->grid_idx) ? true : false;
			neighbor_equal_current[i] = (parent_voxel_id_update_it->neighbor_grid_idx == parent_voxel_id_update_it->grid_idx) ? true : false;
			break;

		case GROUP_IDX:
			neighbor_good_than_current[i] = (parent_voxel_id_update_it->neighbor_parent_group_idx < parent_voxel_id_update_it->current_parent_group_idx) ? true : false;
			neighbor_equal_current[i] = (parent_voxel_id_update_it->neighbor_parent_group_idx == parent_voxel_id_update_it->current_parent_group_idx) ? true : false;
			break;
			/*case HIGH_MSE_RATIO:
			neighbor_good_than_current[i] = (parent_voxel_id_update_it->neighbor_high_mse_ratio < parent_voxel_id_update_it->high_mse_ratio) ? true : false;
			neighbor_equal_current[i] = (parent_voxel_id_update_it->neighbor_high_mse_ratio == parent_voxel_id_update_it->high_mse_ratio) ? true : false;
			break;*/

		case BAD_NEIGHBOR_CNT:
			neighbor_good_than_current[i] = (parent_voxel_id_update_it->neighbor_bad_neighbor_count < parent_voxel_id_update_it->smallest_bad_neighbor_count) ? true : false;
			neighbor_equal_current[i] = (parent_voxel_id_update_it->neighbor_bad_neighbor_count == parent_voxel_id_update_it->smallest_bad_neighbor_count) ? true : false;
			break;

		case INVALID_PARA:
			neighbor_good_than_current[i] = false;
			neighbor_equal_current[i] = true;
			break;


		default:
			break;
		}
	}

	if (neighbor_good_than_current[0] == true) {

		update_flag = 1;
		goto update_check_finish;
	}
	else if (neighbor_equal_current[0] == false) {

		return false;
	}

	if (neighbor_good_than_current[1] == true) {

		update_flag = 1;
		goto update_check_finish;
	}
	else if (neighbor_equal_current[1] == false) {

		return false;
	}

	if (neighbor_good_than_current[2] == true) {

		update_flag = 1;
		goto update_check_finish;
	}
	else if (neighbor_equal_current[2] == false) {

		return false;
	}

	if (neighbor_good_than_current[3] == true) {

		update_flag = 1;
		goto update_check_finish;
	}
	else if (neighbor_equal_current[3] == false) {

		return false;
	}


	if (neighbor_good_than_current[4] == true) {

		update_flag = 1;
		goto update_check_finish;
	}
	else if (neighbor_equal_current[4] == false) {

		return false;
	}
	else {

		log_info("all conditions are the same");
		log_info("neighbor_parent_voxel_idx = %d current_parent_voxel_idx=%d", parent_voxel_id_update_it->neighbor_parent_voxel_idx, parent_voxel_id_update_it->current_parent_voxel_idx);
		log_info("neighbor_good_neighbor_count = %d, largest_good_neighbor_count = %d", parent_voxel_id_update_it->neighbor_good_neighbor_count, parent_voxel_id_update_it->largest_good_neighbor_count);
		log_info("neighbor_bad_neighbor_count = %d, smallest_bad_neighbor_count = %d", parent_voxel_id_update_it->neighbor_bad_neighbor_count, parent_voxel_id_update_it->smallest_bad_neighbor_count);

		log_info("neighbor_mse = %f, smallest_mse = %f", parent_voxel_id_update_it->neighbor_mse, parent_voxel_id_update_it->smallest_mse);
		//log_info("neighbor_smallest_normal_diff = %f, smallest_normal_diff = %f", parent_voxel_id_update_it->neighbor_smallest_normal_diff, parent_voxel_id_update_it->smallest_normal_diff);
		//log_info("neighbor_smallest_plane_dist = %f, smallest_plane_dist = %f", parent_voxel_id_update_it->neighbor_smallest_plane_dist, parent_voxel_id_update_it->smallest_plane_dist);
		//log_info("neighbor_largest_point_cnt = %d, largest_point_cnt = %d", parent_voxel_id_update_it->neighbor_largest_point_cnt, parent_voxel_id_update_it->largest_point_cnt);
		//log_info("neighbor_center_dist = %f, center_dist = %f", parent_voxel_id_update_it->neighbor_center_dist, parent_voxel_id_update_it->center_dist);
		//log_info("neighbor_high_mse_ratio = %f, high_mse_ratio = %f", parent_voxel_id_update_it->neighbor_high_mse_ratio, parent_voxel_id_update_it->high_mse_ratio);
		log_info("neighbor_grid_idx = %d, grid_idx = %d", parent_voxel_id_update_it->neighbor_grid_idx, parent_voxel_id_update_it->grid_idx);
		log_info("neighbor_parent_group_idx = %d, current_parent_group_idx = %d", parent_voxel_id_update_it->neighbor_parent_group_idx, parent_voxel_id_update_it->current_parent_group_idx);
		return false;
	}

update_check_finish:

	// update current parent voxel index
	if (update_flag == 1)
	{
		return true;
	}
	else
	{
		return false;
	}

}

// find the point who have the max similar points in each bad voxel
bool PointsGroupPlane::GetRemainerMaxSimilarPointsIdx(unsigned int bad_voxel_idx, unsigned int* similar_point_cnt_list, unsigned int &bad_point_idx)
{
	const unsigned int top_k = MAX_NUM_OF_PLANE_IN_BAD_VOXEL;
	float min_normal_diff = static_cast<float>(std::cos(plane_seg_thresholds.THRESHOLD_MAX_NORMAL_ANGEL_OF_2PLANE * M_PI / 180));
	unsigned int max_cnt = 0;
	BadVoxelMergeOutpuItem *bad_voxel_it = &bad_voxel_merge_item[bad_voxel_idx];
	PlaneVoxelItem *plane_voxel = &plane_voxel_array.voxels[bad_voxel_it->bad_voxel_idx];

	for (unsigned int j = 0; j < plane_voxel->points.size; j++)
	{
		if (bad_voxel_it->point_merged_flag[j]) continue; // if second point has been merged , go to next point
		if (bad_voxel_it->remainer_similar_group_idx[j] != std::numeric_limits<unsigned int>::max()) continue;
		Point3f fist_point_normal = pt_cloud_normal.points[plane_voxel->points.point_idx[j]];
		for (unsigned int k = 0; k < plane_voxel->points.size; k++)
		{
			if (bad_voxel_it->point_merged_flag[k]) continue; // if second point has been merged , go to next point
			if (bad_voxel_it->remainer_similar_group_idx[k] != std::numeric_limits<unsigned int>::max()) continue;
			if (k == j) continue;
			Point3f second_point_normal = pt_cloud_normal.points[plane_voxel->points.point_idx[k]];
			float normal_diff = std::fabs( ComputeVectorDotProduct<float>(fist_point_normal, second_point_normal));
			if (normal_diff > min_normal_diff)
			{
				similar_point_cnt_list[j]++;
			}
		}

		if (similar_point_cnt_list[j] > max_cnt)
		{
			max_cnt = similar_point_cnt_list[j];
			bad_point_idx = j;
		}
	}

	if (max_cnt != 0)
	{
		return true;
	}
	else
	{
		return false;
	}

}

void PointsGroupPlane::GetRemainerNormalDiffByPlanes(const PlaneMergeOutput * exist_planes)
{
	float min_normal_diff = static_cast<float>(std::cos((plane_seg_thresholds.THRESHOLD_MAX_NORMAL_ANGEL_OF_EDGE_POINT) * M_PI / 180));
	float min_plane_dist = plane_seg_thresholds.THRESHOLD_MIN_DIST_OF_2PLANE;
	float max_plane_mse = plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_PLANE;
	if (sys_control_para.scanner_type == UNRE_TYPE_0)
	{
		min_plane_dist = static_cast<float>(min_plane_dist * 0.8);
		max_plane_mse = static_cast<float>(max_plane_mse * 0.8);
	}
	log_info("GetRemainerNormalDiffByPlanes min_normal_diff =%f min_plane_dist =%f", std::acos(min_normal_diff)*180 / M_PI, min_plane_dist);
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(point_merged_voxel_size); i++)
	{
		unsigned point_group_idx = 0;
		BadVoxelMergeOutpuItem *bad_voxel_it = &bad_voxel_merge_item[i];
		PlaneVoxelItem *plane_voxel = &plane_voxel_array.voxels[bad_voxel_it->bad_voxel_idx];
		if (!bad_voxel_it->is_remainer_occupied) continue;   // no remainer points , go to next voxel

		for (unsigned int j = 0; j < exist_planes->size; j++)
		{
			int plane_points_cnt = 0;
			Point3f base_plane_normal = exist_planes->planes[j].plane_normal;
			Point3f base_plane_center = exist_planes->planes[j].plane_center;
			for (unsigned int k = 0; k < plane_voxel->points.size; k++)
			{
				unsigned int point_idx = plane_voxel->points.point_idx[k];
				bool debug_con = false;// (point_idx == 670961) || (point_idx == 670777) || (point_idx == 670708) || \
					//(point_idx == 670789) || (point_idx == 670632)|| (point_idx == 670353);

				if (bad_voxel_it->point_merged_flag[k]) continue; // if first point has been merged , go to next point
				if (bad_voxel_it->remainer_similar_group_idx[k] != std::numeric_limits<unsigned int>::max()) continue;
				//Point3f point_normal = pt_cloud_normal.points[plane_voxel->points.point_idx[k]];
				//float normal_diff = std::fabs(ComputeVectorDotProduct<float>(base_plane_normal, point_normal));
				Point3f point = pt_cloud_xyz.points[plane_voxel->points.point_idx[k]];
				float plane_dist = ComputePointToPlaneDist<float>(point, base_plane_normal, base_plane_center);
				//bool merge_contion = (plane_dist < max_plane_mse) || ((normal_diff > min_normal_diff) && (plane_dist < min_plane_dist));
				bool merge_contion = plane_dist < exist_planes->planes[j].plane_mse * 2;
				if (merge_contion)
				{
					if (debug_con) log_info("voxel idx =%d point_idx=%d is same with  plane =%d point_group_idx", bad_voxel_it->bad_voxel_idx,point_idx, global_ref_plane_idx[j]);
					bad_voxel_it->remainer_similar_group_idx[k] = point_group_idx;
					plane_points_cnt ++;
				}
			}

			if(plane_points_cnt != 0) point_group_idx++;			
		}
		bad_voxel_it->remainer_similar_group_size = point_group_idx;
	}
}

//initialize the elments of Points Group in bad voxels or pseudo bad voxels
void PointsGroupPlane::BadVoxelGroupInit()
{
	const unsigned int top_k = MAX_NUM_OF_PLANE_IN_BAD_VOXEL;

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(point_merged_voxel_size); i++)
	{
		BadVoxelMergeOutpuItem *bad_voxel_it = &bad_voxel_merge_item[i];
		if (!bad_voxel_it->is_remainer_occupied) continue;
		bad_voxel_it->remainer_similar_group_idx = new unsigned int[bad_voxel_it->voxel_point_size];
		for (unsigned int j = 0; j < bad_voxel_it->voxel_point_size; j++)
		{
			bad_voxel_it->remainer_similar_group_idx[j] = std::numeric_limits<unsigned int>::max();
		}
		bad_voxel_it->remainer_similar_group_size = 0;

		bad_voxel_it->points_group = new BadVoxelPointsGroup;
		for (unsigned int j = 0; j < top_k; j++)
		{

			PointsGroupsItem *points_group_it = &bad_voxel_it->points_group->similar_points_group_item[j];
			points_group_it->parent_voxel_idx[0] = points_group_it->parent_voxel_idx[1] = bad_voxel_it->bad_voxel_idx;
			points_group_it->parent_group_idx[0] = points_group_it->parent_group_idx[1] = j;
			points_group_it->good_neighbor_cnt = 0;
			points_group_it->plane_mse = std::numeric_limits<float>::infinity();
			points_group_it->plane_high_mse_ratio = 1.0f;
			points_group_it->plane_normal = Point3f{ 0.0f };
			points_group_it->plane_center = Point3f{ 0.0f };
			MathOperation::ClearSums(&points_group_it->sums);
			points_group_it->points.size = 0;
			points_group_it->points.point_idx = NULL;
			points_group_it->bad_voxel_point_idx = NULL;
			for (unsigned int k = 0; k < 26; k++)
			{
				NeighborPointsGroupItem *neighbor_group = &points_group_it->neighbour_points_group_item[k];
				neighbor_group->points_group_idx = std::numeric_limits<unsigned int>::max();
				for (unsigned int m = 0; m < top_k; m++)
				{
					//neighbor_group->normal_diff[m] = std::numeric_limits<float>::infinity();
					//neighbor_group->plane_dist[m] = std::numeric_limits<float>::infinity();
					neighbor_group->is_good_neighbor[m] = false;
				}
			}
			points_group_it->same_with_ref_plane_array.ref_plane_size = 0;
			points_group_it->same_with_ref_plane_array.is_same_with_ref_plane = NULL;
			points_group_it->is_good_group = false;
			points_group_it->closest_ref_plane_idx = std::numeric_limits<unsigned int>::max();
		}
	}


}

void PointsGroupPlane::GetRemainerNormalDiff()
{
	float min_normal_diff = static_cast<float>(std::cos((plane_seg_thresholds.THRESHOLD_MAX_NORMAL_ANGEL_OF_2PLANE) * M_PI / 180));
	float min_plane_dist = plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_VOXEL;

	unsigned int min_point_cnt_remain_voxel = plane_seg_thresholds.THRESHOLD_MIN_POINT_NUM_OF_FLAT_VOXEL;
	// find the max similar points in each bad voxel ,  select  average  normal of these points as the poings group normal
#if 0
	for (unsigned int i = 0; i < point_merged_voxel_size; i++)
	{
		unsigned point_group_idx = 0;
		BadVoxelMergeOutpuItem *bad_voxel_it = &bad_voxel_merge_item[i];
		PlaneVoxelItem *plane_voxel = &plane_voxel_array.voxels[bad_voxel_it->bad_voxel_idx];

		if (!bad_voxel_it->is_remainer_occupied) continue;   // no remainer points , go to next voxel

		unsigned max_cnt_idx;
		unsigned int* similar_point_cnt_list = new unsigned int[plane_voxel->points.size];
		memset(similar_point_cnt_list, 0, sizeof(unsigned int) *plane_voxel->points.size);
		Point3f avg_nomal;
		while (GetRemainerMaxSimilarPointsIdx(i, similar_point_cnt_list, max_cnt_idx) && point_group_idx < top_k)
		{
			bad_voxel_it->remainer_similar_group_idx[max_cnt_idx] = point_group_idx;
			PointsGroupsItem *points_group_it = &bad_voxel_it->points_group->similar_points_group_item[point_group_idx];
			Point3f fist_point_normal = pt_cloud_xyz.points[plane_voxel->points.point_idx[max_cnt_idx]].normal;
			avg_nomal = fist_point_normal;

			for (unsigned int j = 0; j < plane_voxel->points.size; j++)
			{
				if (bad_voxel_it->point_merged_flag[j]) continue; // if second point has been merged , go to next point
				if (bad_voxel_it->remainer_similar_group_idx[j] != std::numeric_limits<unsigned int>::max()) continue; // if second point have been assigned group index, go to next point
				similar_point_cnt_list[j] = 0;
				Point3f second_point_normal = pt_cloud_normal.points[plane_voxel->points.point_idx[j]];
				float normal_diff = std::fabs(MathOperation::ComputeVectorDotProduct<float>(fist_point_normal, second_point_normal));
				if (normal_diff <= min_normal_diff) continue;
				bad_voxel_it->remainer_similar_group_idx[j] = point_group_idx;
				avg_nomal += second_point_normal;
			}
			MathOperation::VectorNormalized(avg_nomal, points_group_it->plane_normal);
			point_group_idx++;
		}
		bad_voxel_it->remainer_similar_group_size = point_group_idx;
		delete[] similar_point_cnt_list;

	}
#endif


	/*int comput_normal_times = 0;
	int comput_dist_times = 0;
	int compute_point_size = 0;
	int total_point_size = 0;*/

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(point_merged_voxel_size); i++)
	{
		unsigned point_group_idx = 0;
		BadVoxelMergeOutpuItem *bad_voxel_it = &bad_voxel_merge_item[i];
		PlaneVoxelItem *plane_voxel = &plane_voxel_array.voxels[bad_voxel_it->bad_voxel_idx];
		if (!bad_voxel_it->is_remainer_occupied) continue;   // no remainer points , go to next voxel
		if ((bad_voxel_it->voxel_point_size - bad_voxel_it->num_of_points_merged) < min_point_cnt_remain_voxel) continue;
		//unsigned int *remainer_similar_group_idx = new unsigned int[plane_voxel->points.size];
		//memset(remainer_similar_group_idx,-1,sizeof(unsigned int)*plane_voxel->points.size);

		//total_point_size +=(bad_voxel_it->voxel_point_size - bad_voxel_it->num_of_points_merged);
		for (unsigned int j = 0; j < plane_voxel->points.size; j++)
		{
			if (bad_voxel_it->point_merged_flag[j]) continue; // if first point has been merged , go to next point
			//compute_point_size++;
			//if (bad_voxel_it->remainer_similar_group_idx[j] != std::numeric_limits<unsigned int>::max()) continue;  //if first point have been assigned group index, go to next point
			Point3f fist_point_normal = pt_cloud_normal.points[plane_voxel->points.point_idx[j]];
			Point3f fist_point = pt_cloud_xyz.points[plane_voxel->points.point_idx[j]];

			for (unsigned int k = j + 1; k < plane_voxel->points.size; k++)
			{
				if (bad_voxel_it->point_merged_flag[k]) continue; // if second point has been merged , go to next point
				//if (bad_voxel_it->remainer_similar_group_idx[k] != std::numeric_limits<unsigned int>::max()) continue; if second point have been assigned group index, go to next point
				if ((bad_voxel_it->remainer_similar_group_idx[j] != std::numeric_limits<unsigned int>::max())\
					&& (bad_voxel_it->remainer_similar_group_idx[k] != std::numeric_limits<unsigned int>::max())) continue;

				Point3f second_point_normal = pt_cloud_normal.points[plane_voxel->points.point_idx[k]];
				float normal_diff = std::fabs( ComputeVectorDotProduct<float>(fist_point_normal, second_point_normal));
				//comput_normal_times++;
				if (normal_diff < min_normal_diff) continue;

				//normal_diff =  ComputeVectorDotProduct<float>(fist_point_normal, second_point_normal);
				Point3f second_point = pt_cloud_xyz.points[plane_voxel->points.point_idx[k]];
				float normal_dist =  ComputePointToPlaneDist<float>(second_point, fist_point_normal, fist_point);
				//comput_dist_times++;
				//if ((normal_diff > min_normal_diff) && normal_dist < min_plane_dist)
				if (normal_dist < min_plane_dist)
				{
					if (bad_voxel_it->remainer_similar_group_idx[j] != std::numeric_limits<unsigned int>::max()) // if  first point have been assigned group index ,just add the second point to the same group
					{
						bad_voxel_it->remainer_similar_group_idx[k] = bad_voxel_it->remainer_similar_group_idx[j];
					}
					else if (bad_voxel_it->remainer_similar_group_idx[k] != std::numeric_limits<unsigned int>::max()) // if  second point have been assigned group index ,just add the first point to the same group
					{
						bad_voxel_it->remainer_similar_group_idx[j] = bad_voxel_it->remainer_similar_group_idx[k];
					}
					else
					{
						bad_voxel_it->remainer_similar_group_idx[j] = point_group_idx;
						bad_voxel_it->remainer_similar_group_idx[k] = point_group_idx;
						point_group_idx++;
					}
				}
			}
		}
		bad_voxel_it->remainer_similar_group_size = point_group_idx;
	}

	/*log_info("total point size = %d ", total_point_size);
	log_info("comput_normal_times = %d ", comput_normal_times);
	log_info("comput_dist_times = %d ", comput_dist_times);
	log_info("compute_point_size = %d ", compute_point_size);*/

}

void PointsGroupPlane::GroupRemainerNormalDiffTopk()
{
	const unsigned int top_k = MAX_NUM_OF_PLANE_IN_BAD_VOXEL;

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(point_merged_voxel_size); i++)
	{
		unsigned int topk_array[top_k];
		unsigned int topk_group_array[top_k];
		unsigned int current_group;     // current group index of each voxel
		unsigned int top_idx;           // top size array  index of each voxel

		BadVoxelMergeOutpuItem *bad_voxel_it = &bad_voxel_merge_item[i];
		PlaneVoxelItem *plane_voxel = &plane_voxel_array.voxels[bad_voxel_it->bad_voxel_idx];

		if (!bad_voxel_it->is_remainer_occupied) continue;

		top_idx = 0;
		for (unsigned int j = 0; j < top_k; j++)
		{
			topk_array[j] = std::numeric_limits<unsigned int>::max();
			topk_group_array[j] = std::numeric_limits<unsigned int>::max();
		}

		if (bad_voxel_it->remainer_similar_group_size == 0) continue;

		unsigned int* similar_point_cnt_list = new unsigned int[bad_voxel_it->remainer_similar_group_size];
		memset(similar_point_cnt_list, 0, sizeof(unsigned int) * bad_voxel_it->remainer_similar_group_size);

		for (unsigned int j = 0; j < plane_voxel->points.size; j++)
		{
			current_group = bad_voxel_it->remainer_similar_group_idx[j];
			if (current_group != std::numeric_limits<unsigned int> ::max())
			{
				similar_point_cnt_list[current_group]++;
			}
		}


		if (bad_voxel_it->remainer_similar_group_size > top_k)
		{
			bool rtn = false;
			if (top_k >= 2)
			{
				//rtn = MathOperation::FindTopK(bad_voxel_it->remainer_similar_points_cnt, bad_voxel_it->point_merged_flag, plane_voxel->points.size, topk_array, top_k, top_idx);
				rtn = MathOperation::FindTopK(similar_point_cnt_list, bad_voxel_it->remainer_similar_group_size, topk_array, top_k, top_idx);
				if (!rtn)
				{
					top_idx = 0;
				}
			}

			else if (top_k == 1)
			{
				//rtn = MathOperation::FindMax(bad_voxel_it->remainer_similar_points_cnt, bad_voxel_it->point_merged_flag, plane_voxel->points.size, topk_array[0]);
				rtn = MathOperation::FindMax(similar_point_cnt_list, bad_voxel_it->remainer_similar_group_size, topk_array[0]);
				if (!rtn)
				{
					top_idx = 0;
				}
			}

			if(top_idx != 0)
			std::sort(topk_array, topk_array + top_idx, MathOperation::CompareReverseOrderFunc);

			//find the top_k group index
			unsigned int current_group_cnt = 0;
			for (unsigned int j = 0; j < bad_voxel_it->remainer_similar_group_size; j++)
			{
				for (unsigned int k = 0; (k < top_idx) && (current_group_cnt < top_idx); k++)
				{
					if ((topk_array[k] == std::numeric_limits<unsigned int> ::max()) || (topk_array[k] < 3)) continue;
					if (similar_point_cnt_list[j] == topk_array[k])
					{
						topk_group_array[current_group_cnt] = j;
						topk_array[k] = std::numeric_limits<unsigned int> ::max();  // if topk array is found, invalid this topk array
						current_group_cnt++;
						break;
					}
				}
			}
			top_idx = current_group_cnt;
		}
		else
		{
			top_idx = bad_voxel_it->remainer_similar_group_size;
			for (unsigned int j = 0; j < bad_voxel_it->remainer_similar_group_size; j++)
			{
				topk_group_array[j] = j;
			}

		}

		//get the points size of top k array and get the top k point index array
		for (unsigned int j = 0; j < top_idx; j++)
		{
			PointsGroupsItem* points_group_it = &bad_voxel_it->points_group->similar_points_group_item[j];
			current_group = topk_group_array[j];   // get current top points group index
			points_group_it->points.size = similar_point_cnt_list[current_group];
			points_group_it->points.point_idx = new unsigned int[points_group_it->points.size];
			points_group_it->bad_voxel_point_idx = new unsigned int[points_group_it->points.size];
			unsigned int current_idx = 0;
			for (unsigned int k = 0; k < plane_voxel->points.size; k++)
			{
				if (bad_voxel_it->remainer_similar_group_idx[k] == current_group)
				{
					points_group_it->points.point_idx[current_idx] = plane_voxel->points.point_idx[k];
					points_group_it->bad_voxel_point_idx[current_idx] = k;
					current_idx++;
				}
			}

			if (current_idx != similar_point_cnt_list[current_group]) log_debug("group %d current_idx %d is not equal to similar_point_cnt_list[%d] %d ", j, current_idx, current_group, similar_point_cnt_list[current_group]);
		}
		delete[] similar_point_cnt_list;
		similar_point_cnt_list = NULL;

	}
	//PointsGroupDebug();
}


void PointsGroupPlane::ReshapeGroupByPlaneDist(const bool based_on_plane)
{
	const unsigned int top_k = MAX_NUM_OF_PLANE_IN_BAD_VOXEL;
	float points_group_mse_threshold = plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_VOXEL / 2;
	if (sys_control_para.scanner_type == UNRE_TYPE_0)
	{
		points_group_mse_threshold = static_cast<float>(plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_VOXEL *0.8);
	}
	if (based_on_plane) points_group_mse_threshold = plane_seg_thresholds.THRESHOLD_MIN_DIST_OF_2PLANE;
	//float min_point_dist = plane_seg_params.plane_seg_thresholds.THRESHOLD_MIN_DIST_OF_POINT2VOXEL;
	// compute the points group normal,mse and center
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(point_merged_voxel_size); i++)
	{
		BadVoxelMergeOutpuItem *bad_voxel_it = &bad_voxel_merge_item[i];
		if (!bad_voxel_it->is_remainer_occupied) continue;
		// get sums of each group and compute  normal and mse
		for (unsigned int j = 0; j < top_k; j++)
		{
			PointsGroupsItem *points_group_it = &bad_voxel_it->points_group->similar_points_group_item[j];
			if (points_group_it->points.size > 0)
			{
				for (unsigned int k = 0; k < points_group_it->points.size; k++)
				{
					unsigned int point_idx = points_group_it->points.point_idx[k];
					bool debug_con = false;// (point_idx == 670961) || (point_idx == 670777) || (point_idx == 670708) || \
						//(point_idx == 670789) || (point_idx == 670632) || (point_idx == 670353);
					//debug_con = debug_con || (bad_voxel_it->bad_voxel_idx == 18600) || (bad_voxel_it->bad_voxel_idx == 18599);
					//if (debug_con) log_info("voxel  idx =%d point_idx=%d in group =%d", bad_voxel_it->bad_voxel_idx, point_idx, j);
					Point3f Point = pt_cloud_xyz.points[points_group_it->points.point_idx[k]];
					MathOperation::PushPoint(&points_group_it->sums, Point);
				}
				float eigen_mse;
				MathOperation::Compute(points_group_it->points.size, points_group_it->sums, points_group_it->plane_normal, points_group_it->plane_center, eigen_mse);
				//MathOperation::GetPointsCenter(&points_group_it->sums, points_group_it->points.size, points_group_it->plane_center);
				MathOperation::GetPointsArrayMse(pt_cloud_xyz, points_group_it->points, points_group_it->plane_normal, points_group_it->plane_center, points_group_mse_threshold, points_group_it->plane_mse, points_group_it->plane_high_mse_ratio);
			}
		}
	}

	//PointsGroupDebug();
	// find and remove the points whose plane distance > theshold from groups
	if (based_on_plane) return;
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(point_merged_voxel_size); i++)
	{
		BadVoxelMergeOutpuItem *bad_voxel_it = &bad_voxel_merge_item[i];
		PlaneVoxelItem *plane_voxel = &plane_voxel_array.voxels[bad_voxel_it->bad_voxel_idx];
		if (!bad_voxel_it->is_remainer_occupied) continue;
		// get sums of each group and compute  normal and mse
		for (unsigned int j = 0; j < top_k; j++)
		{
			PointsGroupsItem *points_group_it = &bad_voxel_it->points_group->similar_points_group_item[j];
			if (points_group_it->points.size == 0) continue;
			if (points_group_it->plane_mse > points_group_mse_threshold) continue; // if the group mse is too high , it can not be as a good neighbour group, so need not reshape it

			unsigned int *tmp_point_idx = new unsigned int[points_group_it->points.size];
			unsigned int point_cnt = 0;
			for (unsigned int k = 0; k < points_group_it->points.size; k++)
			{
				Point3f Point = pt_cloud_xyz.points[points_group_it->points.point_idx[k]];
				float point_dist =  ComputePointToPlaneDist<float>(Point, points_group_it->plane_normal, points_group_it->plane_center);
				if (point_dist >= points_group_mse_threshold)
				{
					MathOperation::PopPoint(&points_group_it->sums, Point);
					continue;
				}
				tmp_point_idx[point_cnt] = points_group_it->points.point_idx[k];
				point_cnt++;
			}

			if (point_cnt == 0)
			{
				delete[] tmp_point_idx;
				tmp_point_idx = NULL;
				//log_debug("voxel %d group %d point cnt is zero! ", bad_voxel_it->bad_voxel_idx, j);
			}
			delete[] points_group_it->points.point_idx;
			points_group_it->points.size = point_cnt;
			points_group_it->points.point_idx = tmp_point_idx;
			if (point_cnt)
			{
				float eigen_mse;
				MathOperation::Compute(points_group_it->points.size, points_group_it->sums, points_group_it->plane_normal, points_group_it->plane_center, eigen_mse);
				//MathOperation::GetPointsCenter(&points_group_it->sums, points_group_it->points.size, points_group_it->plane_center);
				MathOperation::GetPointsArrayMse(pt_cloud_xyz, points_group_it->points, points_group_it->plane_normal, points_group_it->plane_center, points_group_mse_threshold, points_group_it->plane_mse, points_group_it->plane_high_mse_ratio);
			}
		}
	}
	//PointsGroupDebug();
}

void PointsGroupPlane::AssignRemainerNeighbors()
{
	//unsigned int bad_voxel_idx;   // bad voxel index
	//float normal_diff, plane_dist;
	const unsigned int top_k = MAX_NUM_OF_PLANE_IN_BAD_VOXEL;
	float points_group_mse_threshold = plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_VOXEL;
	/*if (sys_control_para.scanner_type == INDUSTRY_TYPE_0)
	{
		// to solve the problem of discrete planes genarated in edge of good voxel planes
		points_group_mse_threshold = plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_VOXEL / 2;
	}*/
	float min_normal_diff = static_cast<float>(std::cos(plane_seg_thresholds.THRESHOLD_MAX_NORMAL_ANGLE_OF_TWO_VOXEL * M_PI / 180));
	float min_plane_dist = plane_seg_thresholds.THRESHOLD_MIN_PLANE_DIST_OF_TWO_VOXEL;


	// find the sampe groups in each voxel whose normal difference and plane distance of the same voxel  is < theshold ,merge  them into a same group
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(point_merged_voxel_size); i++)
	{
		BadVoxelMergeOutpuItem* bad_voxel_it = &bad_voxel_merge_item[i];
		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[bad_voxel_it->bad_voxel_idx];
		if (!bad_voxel_it->is_remainer_occupied) continue;

		unsigned int same_group_cnt = 0;
		unsigned int same_group_idx[top_k];   // record the same group index  of all the voxel groups whose normal difference and plane distance;
		memset(same_group_idx, -1, sizeof(unsigned int)*top_k);
		for (unsigned int j = 0; j < top_k; j++)
		{
			PointsGroupsItem* points_group_it = &bad_voxel_it->points_group->similar_points_group_item[j];
			if (points_group_it->points.size == 0) continue;
			if ((points_group_it->plane_mse > points_group_mse_threshold) || (points_group_it->plane_high_mse_ratio > min_high_mse_ratio)) continue;
			points_group_it->is_good_group = true;
			//if (same_group_idx[j] != -1) continue;
			for (unsigned int k = j + 1; k < top_k; k++)
			{
				PointsGroupsItem* second_points_group_it = &bad_voxel_it->points_group->similar_points_group_item[k];
				if (second_points_group_it->points.size == 0) continue;
				//if (same_group_idx[k] != -1) continue;
				if ((second_points_group_it->plane_mse > points_group_mse_threshold) || (second_points_group_it->plane_high_mse_ratio > min_high_mse_ratio)) continue;
				float normal_diff = std::fabs( ComputeVectorDotProduct<float>(second_points_group_it->plane_normal, points_group_it->plane_normal));
				if (normal_diff < min_normal_diff) continue;
				float plane_dist =  ComputePointToPlaneDist<float>(second_points_group_it->plane_center, points_group_it->plane_normal, points_group_it->plane_center);
				if (plane_dist < min_plane_dist)
				{
					if (same_group_idx[j] != -1)
					{
						same_group_idx[k] = same_group_idx[j];
					}
					else if (same_group_idx[k] != -1)
					{
						same_group_idx[j] = same_group_idx[k];
					}
					else
					{
						same_group_idx[j] = same_group_cnt;
						same_group_idx[k] = same_group_cnt;
						same_group_cnt++;
					}
					//log_info("voxel %d group %d group%d is in same group same_group_idx[j] =%d [k] =%d ", bad_voxel_it->bad_voxel_idx,j,k, same_group_idx[j], same_group_idx[k]);
				}
			}
		}

		if (same_group_cnt != 0)
		{
			PointsGroupsItem* tmp_points_group_item = new PointsGroupsItem[same_group_cnt];  // only apply new resource for same groups 

			// initialize the elements of tmp_points_group_item;
			for (unsigned int j = 0; j < same_group_cnt; j++)
			{
				PointsGroupsItem* tmp_points_group_it = &tmp_points_group_item[j];
				tmp_points_group_it->parent_voxel_idx[0] = tmp_points_group_it->parent_voxel_idx[1] = std::numeric_limits<unsigned int>::max();
				tmp_points_group_it->parent_group_idx[0] = tmp_points_group_it->parent_group_idx[1] = std::numeric_limits<unsigned int>::max();
				tmp_points_group_it->good_neighbor_cnt = 0;
				tmp_points_group_it->plane_mse = std::numeric_limits<float>::infinity();
				tmp_points_group_it->plane_high_mse_ratio = 1.0f;
				tmp_points_group_it->plane_normal = Point3f{ 0.f, 0.f, 0.f };
				tmp_points_group_it->plane_center = Point3f{ 0.f, 0.f, 0.f };
				MathOperation::ClearSums(&tmp_points_group_it->sums);
				tmp_points_group_it->points.size = 0;
				tmp_points_group_it->points.point_idx = NULL;
				tmp_points_group_it->bad_voxel_point_idx = NULL;
				for (unsigned int k = 0; k < 26; k++)
				{
					NeighborPointsGroupItem *neighbor_group = &tmp_points_group_it->neighbour_points_group_item[k];
					for (unsigned int m = 0; m < top_k; m++)
					{
						//neighbor_group->normal_diff[m] = std::numeric_limits<float>::infinity();
						//neighbor_group->plane_dist[m] = std::numeric_limits<float>::infinity();
						neighbor_group->is_good_neighbor[m] = false;
					}
					neighbor_group->points_group_idx = std::numeric_limits<unsigned int>::max();
				}
			}

			// merge all the same groups with same group cnt into one tmp_points_group_item;

			for (unsigned int j = 0; j < same_group_cnt; j++)
			{
				// get points size and  parent voxel index , parent group index
				PointsGroupsItem* tmp_points_group_it = &tmp_points_group_item[j];
				for (unsigned int k = 0; k < top_k; k++)
				{
					PointsGroupsItem* points_group_it = &bad_voxel_it->points_group->similar_points_group_item[k];
					if (same_group_idx[k] == j)
					{
						tmp_points_group_it->points.size += points_group_it->points.size;
						MathOperation::PushSums(&tmp_points_group_it->sums, &points_group_it->sums);
						if (tmp_points_group_it->parent_voxel_idx[0] == std::numeric_limits<unsigned int>::max())
						{
							tmp_points_group_it->parent_voxel_idx[0] = tmp_points_group_it->parent_voxel_idx[1] = points_group_it->parent_voxel_idx[0];
							tmp_points_group_it->parent_group_idx[0] = tmp_points_group_it->parent_group_idx[1] = points_group_it->parent_group_idx[0];
						}
					}
				}

				//  assgined the point index and bad voxel point index
				tmp_points_group_it->points.point_idx = new unsigned int[tmp_points_group_it->points.size];
				tmp_points_group_it->bad_voxel_point_idx = new unsigned int[tmp_points_group_it->points.size];
				unsigned int point_cnt_in_group = 0;
				for (unsigned int k = 0; k < top_k; k++)
				{
					PointsGroupsItem* points_group_it = &bad_voxel_it->points_group->similar_points_group_item[k];
					if (same_group_idx[k] == j)
					{
						for (unsigned int m = 0; m < points_group_it->points.size; m++)
						{
							tmp_points_group_it->points.point_idx[point_cnt_in_group] = points_group_it->points.point_idx[m];
							tmp_points_group_it->bad_voxel_point_idx[point_cnt_in_group] = points_group_it->bad_voxel_point_idx[m];
							point_cnt_in_group++;
						}
					}
				}

				//recompute  plane normal , plane center , and mse of new merged group
				float eigen_mse;
				MathOperation::Compute(tmp_points_group_it->points.size, tmp_points_group_it->sums, tmp_points_group_it->plane_normal, tmp_points_group_it->plane_center, eigen_mse);
				MathOperation::GetPointsArrayMse(pt_cloud_xyz, tmp_points_group_it->points, tmp_points_group_it->plane_normal, tmp_points_group_it->plane_center, points_group_mse_threshold, tmp_points_group_it->plane_mse, tmp_points_group_it->plane_high_mse_ratio);
				//bool condition = (bad_voxel_it->bad_voxel_idx == 2582)|| (bad_voxel_it->bad_voxel_idx == 2598)|| (bad_voxel_it->bad_voxel_idx == 2580)|| (bad_voxel_it->bad_voxel_idx == 2599)|| (bad_voxel_it->bad_voxel_idx == 2600);
				//condition = condition || (bad_voxel_it->bad_voxel_idx == 2583) || (bad_voxel_it->bad_voxel_idx == 2578) || (bad_voxel_it->bad_voxel_idx == 2579);
				//if (condition) log_info("voxel %d parent group %d  mse = %f mse ratio =%f ", bad_voxel_it->bad_voxel_idx, tmp_points_group_it->parent_group_idx[0],tmp_points_group_it->plane_mse, tmp_points_group_it->plane_high_mse_ratio);
			}

			unsigned group_cnt = 0;
			unsigned non_same_group_cnt = 0;
			for (unsigned int j = 0; j < top_k; j++)
			{
				if (same_group_idx[j] == -1)
				{
					non_same_group_cnt++;
				}
			}
			group_cnt = same_group_cnt + non_same_group_cnt;
			//log_debug("idx% voxel%d have  group cnt %d same group cnt %d  non same group cnt %d ",i,bad_voxel_it->bad_voxel_idx,group_cnt,same_group_cnt,non_same_group_cnt);
			for (unsigned int j = 0; j < top_k; j++)
			{
				PointsGroupsItem* points_group_it = &bad_voxel_it->points_group->similar_points_group_item[j];
				if (same_group_idx[j] == -1) continue;

				unsigned int k = 0;
				for (k = 0; k < same_group_cnt; k++)
				{
					PointsGroupsItem* tmp_points_group_it = &tmp_points_group_item[k];
					if (tmp_points_group_it->parent_group_idx[0] == points_group_it->parent_group_idx[0])
					{
						break;
					}
				}

				if (k < same_group_cnt)
				{
					PointsGroupsItem* tmp_points_group_it = &tmp_points_group_item[k];
					points_group_it->points.size = tmp_points_group_it->points.size;
					if (points_group_it->points.point_idx != NULL)
					{
						delete[] points_group_it->points.point_idx;
						points_group_it->points.point_idx = NULL;
					}

					if (points_group_it->bad_voxel_point_idx != NULL)
					{
						delete[] points_group_it->bad_voxel_point_idx;
						points_group_it->bad_voxel_point_idx = NULL;
					}

					points_group_it->points.point_idx = tmp_points_group_it->points.point_idx;
					points_group_it->bad_voxel_point_idx = tmp_points_group_it->bad_voxel_point_idx;
					MathOperation::AssignSums(&points_group_it->sums, &tmp_points_group_it->sums);
					points_group_it->plane_normal = tmp_points_group_it->plane_normal;
					points_group_it->plane_center = tmp_points_group_it->plane_center;
					points_group_it->plane_mse = tmp_points_group_it->plane_mse;
					points_group_it->plane_high_mse_ratio = tmp_points_group_it->plane_high_mse_ratio;

				}
				else
				{
					points_group_it->points.size = 0;
					if (points_group_it->points.point_idx != NULL)
					{
						delete[] points_group_it->points.point_idx;
						points_group_it->points.point_idx = NULL;
					}

					if (points_group_it->bad_voxel_point_idx != NULL)
					{
						delete[] points_group_it->bad_voxel_point_idx;
						points_group_it->bad_voxel_point_idx = NULL;
					}
				}
			}
			delete[] tmp_points_group_item;
			tmp_points_group_item = NULL;
		}
	}

	// compute the points group neighbor info
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(point_merged_voxel_size); i++)
	{
		BadVoxelMergeOutpuItem* bad_voxel_it = &bad_voxel_merge_item[i];
		//PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[bad_voxel_it->bad_voxel_idx];

		if (!bad_voxel_merge_item[i].is_remainer_occupied) continue;
		for (unsigned int j = 0; j < top_k; j++)
		{
			PointsGroupsItem* points_group_it = &bad_voxel_merge_item[i].points_group->similar_points_group_item[j];
			//if ((points_group_it->plane_mse > points_group_mse_threshold) || (points_group_it->plane_high_mse_ratio > min_high_mse_ratio)) continue;
			if (!points_group_it->is_good_group) continue;
			if (points_group_it->points.size > 0)
			{
				/*for (unsigned int k = j+1; k < top_k; k++)
				{
					PointsGroupsItem* second_points_group_it = &bad_voxel_it->points_group->similar_points_group_item[k];
					if ((second_points_group_it->plane_mse > points_group_mse_threshold) || (second_points_group_it->plane_high_mse_ratio > min_high_mse_ratio)) continue;

					if (second_points_group_it->points.size > 0)
					{
						normal_diff = MathOperation::ComputeVectorDotProduct<float>(second_points_group_it->plane_normal, points_group_it->plane_normal);
						if (normal_diff < min_normal_diff) continue;
						plane_dist = MathOperation::ComputePointToPlaneDist<float>(second_points_group_it->plane_center, points_group_it->plane_normal, points_group_it->plane_center);
						if (plane_dist < min_plane_dist)
						{
							log_debug("voxel%d group %d second group %d plane_dist =%f  normal_diff = %f ", bad_voxel_it->bad_voxel_idx, j, k, plane_dist, std::acos(normal_diff) * 180 / M_PI);
						}
					}
				}*/

				for (unsigned int k = 0; k < 26; k++)
				{
					PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[bad_voxel_it->bad_voxel_idx];
					NeighborItem* neighbor_it = &plane_voxel->neighbors[k];
					NeighborPointsGroupItem* points_group_neighbor_it = &points_group_it->neighbour_points_group_item[k];
					//if (!neighbor_it->is_occupied) continue;
					if (!neighbor_it->is_connected) continue;
					unsigned int neighbor_bad_voxel_idx = voxel_idx_to_bad_voxel[neighbor_it->voxel_idx];
					if (neighbor_bad_voxel_idx == std::numeric_limits<unsigned int>::max()) continue;
					BadVoxelMergeOutpuItem *neighbor_bad_voxel_it = &bad_voxel_merge_item[neighbor_bad_voxel_idx];
					if (!neighbor_bad_voxel_it->is_remainer_occupied) continue;
					//if (!neighbor_bad_voxel_it->neighbor_connected[k]) continue;
					if (!bad_voxel_it->neighbor_connected[k]) continue;
					if (points_group_neighbor_it->points_group_idx < top_k) continue;  // if points_group_idx < top_k show this group already has neighbour group
					// find the smallest normal difference in the neighbour points groups.

					float smallest_normal_diff = 0.0f;
					unsigned smallest_group_idx = -1;
					for (unsigned int m = 0; m < top_k; m++)
					{
						PointsGroupsItem* neighbor_points_group_it = &neighbor_bad_voxel_it->points_group->similar_points_group_item[m];

						if (neighbor_points_group_it->points.size == 0) continue;
						unsigned int voxel_idx = bad_voxel_merge_item[i].bad_voxel_idx;
						//bool condition = (voxel_idx == 2582) || (voxel_idx == 2598) || (voxel_idx == 2580) || (voxel_idx == 2599) || (voxel_idx == 2600);
						//condition = condition || (voxel_idx == 2583) || (voxel_idx == 2578) || (bad_voxel_idx == 2579);
						//if (condition) log_info("voxel %d group %d neighbour voxel %d group %d neighbor mse = %f neighbour mse ratio =%f ", voxel_idx, j, neighbor_it->voxel_idx, m,neighbor_points_group_it->plane_mse, neighbor_points_group_it->plane_high_mse_ratio);

						//if ((neighbor_points_group_it->plane_mse > points_group_mse_threshold) || (neighbor_points_group_it->plane_high_mse_ratio > min_high_mse_ratio)) continue;
						if (!neighbor_points_group_it->is_good_group) continue;


						if (neighbor_points_group_it->points.size > 0)
						{
							float normal_diff = std::fabs( ComputeVectorDotProduct<float>(neighbor_points_group_it->plane_normal, points_group_it->plane_normal));
							//points_group_neighbor_it->normal_diff[m] = normal_diff;
							if (normal_diff < min_normal_diff) continue;
							float plane_dist =  ComputePointToPlaneDist<float>(neighbor_points_group_it->plane_center, points_group_it->plane_normal, points_group_it->plane_center);

							//points_group_neighbor_it->plane_dist[m] = plane_dist;
							//if (normal_diff < min_normal_diff) continue;

							//unsigned int neighbor_voxel_idx = neighbor_it->voxel_idx;
							/*bool voxel_condition =((voxel_idx == 6186)&&(neighbor_voxel_idx==6196)) || ((voxel_idx == 61898)&& (neighbor_voxel_idx == 6201));
							if (voxel_condition) log_debug("voxel %d group %d  neigbor voxel %d group %d plane_dist =%f normal diff =%f ", bad_voxel_it->bad_voxel_idx, j, neighbor_it->voxel_idx, m, plane_dist, std::acos(normal_diff) * 180 / M_PI);
							*/
							if (plane_dist < min_plane_dist)
							{
								/*if (points_group_neighbor_it->points_group_idx < top_k)
								{
									log_info("i=%d,j=%d,k=%d,m=%d neighbor points_group_idx is %d < topk  ",i,j,k,m, points_group_neighbor_it->points_group_idx);

									continue;
								}*/
								//points_group_neighbor_it->points_group_idx = m;
								//smallest_group_idx = m;
								// also record the neighbor voxel group item info
								if (smallest_normal_diff < normal_diff)
								{
									smallest_normal_diff = normal_diff;
									smallest_group_idx = m;
								}
							}
						}
					}
					// It is probably that one group maybe have more than one good neighbour in the neighbour voxel, so we only get the smallest normal difference group as good neighbor
					if (smallest_group_idx != -1)
					{
						points_group_neighbor_it->points_group_idx = smallest_group_idx;
					}

				}
			}
		}
	}
	// get  the points group  good neighbour count
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(point_merged_voxel_size); i++)
	{
		BadVoxelMergeOutpuItem* bad_voxel_it = &bad_voxel_merge_item[i];
		//PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[bad_voxel_it->bad_voxel_idx];
		if (!bad_voxel_it->is_remainer_occupied) continue;
		for (unsigned int j = 0; j < top_k; j++)
		{
			PointsGroupsItem* points_group_it = &bad_voxel_it->points_group->similar_points_group_item[j];
			//if ((points_group_it->plane_mse > points_group_mse_threshold) || (points_group_it->plane_high_mse_ratio > min_high_mse_ratio)) continue;
			if (!points_group_it->is_good_group) continue;
			if (points_group_it->points.size > 0)
			{
				for (unsigned int k = 0; k < 26; k++)
				{
					//NeighborItem* neighbor_it = &plane_voxel->neighbors[k];
					NeighborPointsGroupItem* points_group_neighbor_it = &points_group_it->neighbour_points_group_item[k];
					if (points_group_neighbor_it->points_group_idx < top_k) points_group_it->good_neighbor_cnt++;
				}
			}
		}
	}

}

bool PointsGroupPlane::getPlaneSameInfo(const PlaneMergeOutput* exist_planes, bool **&is_same_plane)
{
	PlaneMerge plane_merge(plane_seg_ptr);
	
	SameAndConnectedArray same_connected;
	SamePlaneGroupArray same_plane_group_array;
	same_plane_group_array.same_plane_group_idx = NULL;
	same_connected.is_connected = NULL;
	same_connected.is_same_plane = NULL;
	is_same_plane = NULL;
	plane_merge.MergePlanes(true, exist_planes->planes, exist_planes->size, &same_plane_group_array, &same_connected);
	is_same_plane = same_connected.is_same_plane;
	if (is_same_plane == NULL)
	{
		log_error("getPlaneSameInfo is_same_plane return NULL");
		return false;
	}
	//if (same_connected.is_same_plane != NULL)
	//{
	//	for (unsigned int i = 0; i < exist_planes->size; i++)
	//	{
	//		if (same_connected.is_same_plane[i] != NULL)
	//		{
	//			delete[] same_connected.is_same_plane[i];
	//			same_connected.is_same_plane[i] = NULL;
	//		}
	//	}
	//	delete[] same_connected.is_same_plane;
	//	same_connected.is_same_plane = NULL;
	//}
	if (same_plane_group_array.same_plane_group_idx !=NULL )
	{
		delete[] same_plane_group_array.same_plane_group_idx;
		same_plane_group_array.same_plane_group_idx = NULL;
	}
	if (same_connected.is_connected != NULL)
	{
		for (unsigned int i = 0; i < exist_planes->size; i++)
		{
			if (same_connected.is_connected[i] != NULL)
			{
				delete[] same_connected.is_connected[i];
				same_connected.is_connected[i] = NULL;
			}
		}
		delete[] same_connected.is_connected;
		same_connected.is_connected = NULL;
	}
	return true;
}

void PointsGroupPlane::AssignRemainerNeighborsWithRef(const PlaneMergeOutput *exist_planes)
{
	const unsigned int top_k = MAX_NUM_OF_PLANE_IN_BAD_VOXEL;

	float points_group_mse_threshold = plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_PLANE;
	// min normal diff must relax the threshold 
	float min_normal_diff = static_cast<float>(std::cos(plane_seg_thresholds.THRESHOLD_MAX_NORMAL_ANGEL_OF_2PLANE *2* M_PI / 180));
	float min_plane_dist = plane_seg_thresholds.THRESHOLD_MIN_DIST_OF_2PLANE;

	// first get plane same info
	bool** is_same_plane = NULL;;
	if (!getPlaneSameInfo(exist_planes, is_same_plane))
	{
		log_error("getPlaneSameInfo return error");
	}

	//check each points group whether being same plane with referene planes
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(point_merged_voxel_size); i++)
	{
		BadVoxelMergeOutpuItem* bad_voxel_it = &bad_voxel_merge_item[i];
		if (!bad_voxel_it->is_remainer_occupied) continue;

		for (unsigned int j = 0; j < top_k; j++)
		{
			PointsGroupsItem* points_group_it = &bad_voxel_it->points_group->similar_points_group_item[j];
			points_group_it->same_with_ref_plane_array.is_same_with_ref_plane = new bool[exist_planes->size];
			float closest_plane_dist = std::numeric_limits<float>::infinity();
			for (unsigned int k = 0; k < exist_planes->size; k++)
			{
				points_group_it->same_with_ref_plane_array.is_same_with_ref_plane[k] = false;
			}

			points_group_it->same_with_ref_plane_array.ref_plane_size = exist_planes->size;
			//if ((points_group_it->plane_mse > points_group_mse_threshold) || (points_group_it->plane_high_mse_ratio > min_high_mse_ratio)) continue;
			if (points_group_it->points.size == 0) continue;
			for (unsigned int k = 0; k < exist_planes->size; k++)
			{
				bool debug_con = false;// (bad_voxel_it->bad_voxel_idx == 18601) && (j == 1) || (bad_voxel_it->bad_voxel_idx == 18626) && (j == 0) || (bad_voxel_it->bad_voxel_idx == 18628) && (j == 0) || \
					//(bad_voxel_it->bad_voxel_idx == 18630) && (j == 0) || (bad_voxel_it->bad_voxel_idx == 18633) && (j == 0);

				PlaneMergeOutputItem *plane_it = &exist_planes->planes[k];
				float normal_diff = std::fabs( ComputeVectorDotProduct<float>(points_group_it->plane_normal, plane_it->plane_normal));
				//if (debug_con) log_info("voxel  idx =%d group =%d normal_diff =%f with plane %d", bad_voxel_it->bad_voxel_idx, j, std::acos(normal_diff) * 180 / M_PI, global_ref_plane_idx[k]);
				if (normal_diff < min_normal_diff)  continue;				
				float plane_dist =  ComputePointToPlaneDist<float>(points_group_it->plane_center, plane_it->plane_normal, plane_it->plane_center);
				//if (debug_con) log_info("voxel  idx =%d group =%d plane_dist =%f with plane %d", bad_voxel_it->bad_voxel_idx, j, plane_dist, global_ref_plane_idx[k]);
				//if (normal_diff < min_normal_diff)  continue;
				if (plane_dist < min_plane_dist)
				{
					if (debug_con) log_info("voxel  idx =%d group =%d is good with plane %d", bad_voxel_it->bad_voxel_idx, j, global_ref_plane_idx[k]);
					points_group_it->same_with_ref_plane_array.is_same_with_ref_plane[k] = true;
					points_group_it->is_good_group = true;
					if (plane_dist < closest_plane_dist)
					{
						points_group_it->closest_ref_plane_idx = global_ref_plane_idx[k];
						closest_plane_dist = plane_dist;
					}
				
				}
			}
		}
	}
	// if have multiple reference plane and reference plane is not same plane , only selected most colsest plane
	for (int i = 0; i < static_cast<int>(point_merged_voxel_size); i++)
	{
		BadVoxelMergeOutpuItem* bad_voxel_it = &bad_voxel_merge_item[i];
		if (!bad_voxel_it->is_remainer_occupied) continue;
		for (unsigned int j = 0; j < top_k; j++)
		{
			PointsGroupsItem* points_group_it = &bad_voxel_it->points_group->similar_points_group_item[j];
			for (unsigned int k = 0; k < exist_planes->size; k++)
			{
				for (unsigned int m = k+1; m < exist_planes->size; m++)
				{
					if (m == k) continue;
					if ((points_group_it->same_with_ref_plane_array.is_same_with_ref_plane[k]) && (points_group_it->same_with_ref_plane_array.is_same_with_ref_plane[m]))
					{
						if (!is_same_plane[k][m])
						{
							if (points_group_it->closest_ref_plane_idx != global_ref_plane_idx[k])
							{
								points_group_it->same_with_ref_plane_array.is_same_with_ref_plane[k] = false;
							}
							if (points_group_it->closest_ref_plane_idx != global_ref_plane_idx[m])
							{
								points_group_it->same_with_ref_plane_array.is_same_with_ref_plane[m] = false;
							}
						}
					}
				}
			}
		}
	}

	if (is_same_plane != NULL)
	{
		for (unsigned int i = 0; i < exist_planes->size; i++)
		{
			if (is_same_plane[i] != NULL)
			{
				delete[] is_same_plane[i];
				is_same_plane[i] = NULL;
			}
		}
		delete[] is_same_plane;
		is_same_plane = NULL;
	}


	//check each points group whether is good neighbor identified by referecne planes
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(point_merged_voxel_size); i++)
	{
		BadVoxelMergeOutpuItem* bad_voxel_it = &bad_voxel_merge_item[i];
		if (!bad_voxel_it->is_remainer_occupied) continue;
		for (unsigned int j = 0; j < top_k; j++)
		{
			PointsGroupsItem* points_group_it = &bad_voxel_it->points_group->similar_points_group_item[j];

			//bool debug_con = false;//(bad_voxel_it->bad_voxel_idx == 18601) && (j == 1) || (bad_voxel_it->bad_voxel_idx == 18626) && (j == 0) || (bad_voxel_it->bad_voxel_idx == 18628) && (j == 0) || \
				//(bad_voxel_it->bad_voxel_idx == 18630) && (j == 0) || (bad_voxel_it->bad_voxel_idx == 18633) && (j == 0);
			//if (points_group_it->points.size == 0) continue;
			if (!points_group_it->is_good_group) continue;
			for (unsigned int k = 0; k < 26; k++)
			{
				PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[bad_voxel_it->bad_voxel_idx];
				NeighborItem* neighbor_it = &plane_voxel->neighbors[k];
				if (!neighbor_it->is_connected) continue;
				//PlaneVoxelItem* neighbor_voxel = &plane_voxel_array.voxels[neighbor_it->voxel_idx];

				unsigned int neighbor_bad_voxel_idx = voxel_idx_to_bad_voxel[neighbor_it->voxel_idx];
				if (neighbor_bad_voxel_idx == std::numeric_limits<unsigned int>::max()) continue;
				BadVoxelMergeOutpuItem *neighbor_bad_voxel_it = &bad_voxel_merge_item[neighbor_bad_voxel_idx];
				if (!neighbor_bad_voxel_it->is_remainer_occupied) continue;
				//if (!neighbor_bad_voxel_it->neighbor_connected[k]) continue;
				if (!bad_voxel_it->neighbor_connected[k]) continue;
				for (unsigned int m = 0; m < top_k; m++)
				{
					PointsGroupsItem* neighbor_points_group_it = &neighbor_bad_voxel_it->points_group->similar_points_group_item[m];
					int same_ref_cnt = 0;
					for (unsigned int n = 0; n < exist_planes->size; n++)
					{
						if ((points_group_it->same_with_ref_plane_array.is_same_with_ref_plane[n])&&(neighbor_points_group_it->same_with_ref_plane_array.is_same_with_ref_plane[n]))
						{
							NeighborPointsGroupItem* points_group_neighbor_it = &points_group_it->neighbour_points_group_item[k];
							//if (debug_con) log_info("voxel%d group%d is good with neigbor voxel %d group %d plane %d",bad_voxel_it->bad_voxel_idx, j, neighbor_it->voxel_idx,m, global_ref_plane_idx[n]);
							points_group_neighbor_it->is_good_neighbor[m] = true;
							same_ref_cnt++;
						}
					}
					if (same_ref_cnt != 0)
					{
						points_group_it->good_neighbor_cnt++;
					}
				}
			}
		}
	}
}
// Identify parent for all the points groups of bad voxels
bool PointsGroupPlane::IdentifyRemainerGoodGroupParent()
{
	const unsigned int top_k = MAX_NUM_OF_PLANE_IN_BAD_VOXEL;
	float points_group_mse_threshold = plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_VOXEL;
	//unsigned int parent_bad_voxel_idx;            // parent bad voxel index of bad voxel points group

	unsigned char update_flag;					// global update flag for each iterations
	unsigned char **update_flag_array;			// update flag for top_k points group of all bad voxels
	unsigned char curr_ping_pong_buf_idx;				// index to parent_voxel_idx[2] in current iteration
	unsigned char next_ping_pong_buf_idx;				// index to parent_voxel_idx[2] in next iteration
	unsigned int iteration_count;

	update_flag_array = new unsigned char*[top_k];
	for (int i = 0; i < top_k; i++)
	{
		update_flag_array[i] = new unsigned char[point_merged_voxel_size];
	}
	iteration_count = 0;

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(point_merged_voxel_size); i++)
	{
		for (int j = 0; j < top_k; j++)
		{
			update_flag_array[j][i] = 0;
		}
	}

	log_info(" IdentifyRemainerGoodGroupParent mse threshold %f  mse ratio threshold =%f ", points_group_mse_threshold, min_high_mse_ratio);
	do
	{
		curr_ping_pong_buf_idx = iteration_count % 2;
		next_ping_pong_buf_idx = (iteration_count + 1) % 2;

#pragma omp parallel for
		for (int i = 0; i < static_cast<int>(point_merged_voxel_size); i++)
		{
			BadVoxelMergeOutpuItem* bad_voxel_it = &bad_voxel_merge_item[i];
			PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[bad_voxel_it->bad_voxel_idx];
			//PlaneMergeItem* plane_merge_it = &plane_merge_element[bad_voxel_it->bad_voxel_idx];

			if (!bad_voxel_it->is_remainer_occupied) continue;

			//ParentVoxelIdentifyUpdateItem* points_group_identify_item = new ParentVoxelIdentifyUpdateItem[top_k]; points_group_identify_item is not thread-safe if save the last parent voxel info
			for (int j = 0; j < top_k; j++)
			{
				//update the current voxel parent voxel index ,  current voxel parent group index  and the all the current voxel elements  for parent Identifying
				update_flag_array[j][i] = 0;
				PointsGroupsItem* points_group_it = &bad_voxel_it->points_group->similar_points_group_item[j];
				//if ((points_group_it->plane_mse > points_group_mse_threshold) || (points_group_it->plane_high_mse_ratio > min_high_mse_ratio)) continue; // if mse > threhold , must give up
				if (!points_group_it->is_good_group) continue;  
				if (points_group_it->good_neighbor_cnt == 0) continue;   // if have no good neighbor must give up

				//ParentVoxelIdentifyUpdateItem *points_group_identify_it = &points_group_identify_item[j];
				unsigned int current_parent_voxel_idx = points_group_it->parent_voxel_idx[curr_ping_pong_buf_idx];
				unsigned int current_parent_group_idx = points_group_it->parent_group_idx[curr_ping_pong_buf_idx];
				unsigned int parent_bad_voxel_idx = voxel_idx_to_bad_voxel[current_parent_voxel_idx];
				if (parent_bad_voxel_idx > point_merged_voxel_size)
				{
					log_debug("voxel %d group %d parent_bad_voxel_idx %d exceed the size of bad voxels %d", i, j, parent_bad_voxel_idx, point_merged_voxel_size);
					continue;
				}
				//points_group_identify_it->largest_good_neighbor_count = parent_points_group_it->good_neighbor_cnt;
				//points_group_identify_it->smallest_mse = parent_points_group_it->plane_mse;
				//points_group_identify_it->largest_point_cnt = parent_points_group_it->points.size;
				//points_group_identify_it->high_mse_ratio = parent_points_group_it->plane_high_mse_ratio;
				//points_group_identify_it->grid_idx = group_parent_voxel->grid_idx;
				//points_group_it->parent_voxel_idx[next_ping_pong_buf_idx] = points_group_identify_it->current_parent_voxel_idx;
				//points_group_it->parent_group_idx[next_ping_pong_buf_idx] = points_group_identify_it->current_parent_group_idx;

				//update the neighbor  parent voxel index ,  neighbor voxel parent group index  and the all the neighbor voxel  group elements  for parent Identifying
				for (unsigned int k = 0; k < 26; k++)
				{
					ParentVoxelIdentifyUpdateItem points_group_identify_item;
					ParentVoxelIdentifyUpdateItem *points_group_identify_it = &points_group_identify_item;

					NeighborItem* neighbor_it = &plane_voxel->neighbors[k];
					NeighborPointsGroupItem* points_group_neighbor_it = &points_group_it->neighbour_points_group_item[k];
					if (points_group_neighbor_it->points_group_idx >= top_k) continue; // if points_group_idx > top_k show there is no good neighbor group in this neighbor voxel

					unsigned int bad_voxel_idx = voxel_idx_to_bad_voxel[neighbor_it->voxel_idx];
					if (bad_voxel_idx == std::numeric_limits<unsigned int>::max()) continue;      // not need ,condition should be same as points_group_idx > top_k
					BadVoxelMergeOutpuItem *neighbor_bad_voxel_it = &bad_voxel_merge_item[bad_voxel_idx];
					PointsGroupsItem* neighbor_points_group_it = &neighbor_bad_voxel_it->points_group->similar_points_group_item[points_group_neighbor_it->points_group_idx];
					//if ((neighbor_points_group_it->plane_mse > points_group_mse_threshold) || (neighbor_points_group_it->plane_high_mse_ratio > min_high_mse_ratio)) continue; // if mse > threhold , must give up
					if (!neighbor_points_group_it->is_good_group) continue;
					//if (!neighbor_bad_voxel_it->neighbor_connected[k]) continue;
					if (!bad_voxel_it->neighbor_connected[k]) continue;
					unsigned int neighbor_parent_voxel_idx = neighbor_points_group_it->parent_voxel_idx[next_ping_pong_buf_idx];
					unsigned int neighbor_parent_group_idx = neighbor_points_group_it->parent_group_idx[next_ping_pong_buf_idx];

					unsigned int neighbor_parent_bad_voxel_idx = voxel_idx_to_bad_voxel[neighbor_parent_voxel_idx];
					if (neighbor_parent_bad_voxel_idx > point_merged_voxel_size)
					{
						log_debug("voxel %d group %d neighbor_parent_bad_voxel_idx %d exceed the size of bad voxels %d", i, j, neighbor_parent_bad_voxel_idx, point_merged_voxel_size);
						continue;
					}
					BadVoxelMergeOutpuItem *group_parent_bad_Voxel_it = &bad_voxel_merge_item[parent_bad_voxel_idx];
					PointsGroupsItem* parent_points_group_it = &group_parent_bad_Voxel_it->points_group->similar_points_group_item[current_parent_group_idx];

					points_group_identify_it->largest_good_neighbor_count = parent_points_group_it->good_neighbor_cnt;
					points_group_identify_it->smallest_mse = parent_points_group_it->plane_mse;
					points_group_identify_it->grid_idx = plane_voxel_array.voxels[current_parent_voxel_idx].grid_idx;
					points_group_identify_it->current_parent_voxel_idx = current_parent_voxel_idx;
					points_group_identify_it->current_parent_group_idx = current_parent_group_idx;

					BadVoxelMergeOutpuItem *neighbor_group_parent_bad_Voxel_it = &bad_voxel_merge_item[neighbor_parent_bad_voxel_idx];
					PlaneVoxelItem *neighbor_group_parent_voxel = &plane_voxel_array.voxels[neighbor_parent_voxel_idx];
					PointsGroupsItem* neighbor_parent_points_group_it = &neighbor_group_parent_bad_Voxel_it->points_group->similar_points_group_item[neighbor_parent_group_idx];

					points_group_identify_it->neighbor_good_neighbor_count = neighbor_parent_points_group_it->good_neighbor_cnt;
					points_group_identify_it->neighbor_mse = neighbor_parent_points_group_it->plane_mse;
					//points_group_identify_it->neighbor_largest_point_cnt = neighbor_parent_points_group_it->points.size;
					//points_group_identify_it->neighbor_high_mse_ratio = neighbor_parent_points_group_it->plane_high_mse_ratio;
					points_group_identify_it->neighbor_grid_idx = neighbor_group_parent_voxel->grid_idx;
					points_group_identify_it->neighbor_parent_voxel_idx = neighbor_parent_voxel_idx;
					points_group_identify_it->neighbor_parent_group_idx = neighbor_parent_group_idx;
					ParentIDCondType condition[5] = { GOOD_NEIGHBOR_CNT,PLANE_MSE,GRID_IDX,GROUP_IDX,INVALID_PARA };
					if ((neighbor_parent_voxel_idx == current_parent_voxel_idx) && (neighbor_parent_group_idx == current_parent_group_idx)) continue;
					bool check_condition = NeighborParentCompare(condition, points_group_identify_it, update_flag_array[j][i]);
					if (!check_condition) continue;
					if (check_condition)
					{
						if (update_flag_array[j][i] == 1)
						{
							points_group_it->parent_voxel_idx[curr_ping_pong_buf_idx] = neighbor_parent_voxel_idx;
							points_group_it->parent_group_idx[curr_ping_pong_buf_idx] = neighbor_parent_group_idx;
						}
					}
				}

			}

			//delete[] points_group_identify_item;
		}

		// Check if no more updates
		update_flag = 0;
		for (unsigned int i = 0; i < point_merged_voxel_size; i++) {

			for (int j = 0; j < top_k; j++)
			{
				update_flag |= update_flag_array[j][i];
			}
		}

		// Increment the iteration number
		iteration_count++;

	} while (update_flag > 0);

	// Release memory

	for (int i = 0; i < top_k; i++)
	{
		if (update_flag_array[i] != NULL)
		{
			delete[] update_flag_array[i];
			update_flag_array[i] = NULL;
		}
	}
	if (update_flag_array != NULL)
	{
		delete[] update_flag_array;
		update_flag_array = NULL;
	}
	log_info("IdentifyRemainerGoodGroupParent iteration %d ", iteration_count);

	return true;
}


bool PointsGroupPlane::IdentifyRemainerGoodGroupParentByGoodneighbor()
{

	const unsigned int top_k = MAX_NUM_OF_PLANE_IN_BAD_VOXEL;
	//float points_group_mse_threshold = plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_PLANE;
	//unsigned int parent_bad_voxel_idx;            // parent bad voxel index of bad voxel points group

	unsigned char update_flag;					// global update flag for each iterations
	unsigned char **update_flag_array;			// update flag for top_k points group of all bad voxels
	unsigned char curr_ping_pong_buf_idx;				// index to parent_voxel_idx[2] in current iteration
	unsigned char next_ping_pong_buf_idx;				// index to parent_voxel_idx[2] in next iteration
	unsigned int iteration_count;
	bool(*is_good_neighbor)[top_k][top_k] = NULL;

	update_flag_array = new unsigned char*[top_k];
	for (int i = 0; i < top_k; i++)
	{
		update_flag_array[i] = new unsigned char[point_merged_voxel_size];
	}
	iteration_count = 0;

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(point_merged_voxel_size); i++)
	{
		for (int j = 0; j < top_k; j++)
		{
			update_flag_array[j][i] = 0;
		}
	}

	// check the points group  have the good neighbors in self voxel
	is_good_neighbor = new bool[point_merged_voxel_size][top_k][top_k];

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(point_merged_voxel_size); i++)
	{
		for (int j = 0; j < top_k; j++)
		{
			for (int k = 0; k < top_k; k++)
			{
				if (k == j)
				{
					is_good_neighbor[i][j][k] = true;
				}
				else
				{
					is_good_neighbor[i][j][k] = false;
				}

			}
		}
	}
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(point_merged_voxel_size); i++)
	{
		BadVoxelMergeOutpuItem* bad_voxel_it = &bad_voxel_merge_item[i];
		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[bad_voxel_it->bad_voxel_idx];
		if (!bad_voxel_it->is_remainer_occupied) continue;
		//ParentVoxelIdentifyUpdateItem* points_group_identify_item = new ParentVoxelIdentifyUpdateItem[top_k]; points_group_identify_item is not thread-safe if save the last parent voxel info
		for (int j = 0; j < top_k; j++)
		{
			int self_good_neigbor_cnt = 0;
			PointsGroupsItem* points_group_it = &bad_voxel_it->points_group->similar_points_group_item[j];
			if (!points_group_it->is_good_group) continue;
			for (int k = j + 1; k < top_k; k++)
			{
				if (is_good_neighbor[i][j][k]) continue;
				PointsGroupsItem* self_neighbor_points_group_it = &bad_voxel_it->points_group->similar_points_group_item[k];
				if (!self_neighbor_points_group_it->is_good_group) continue;
				for (int m = 0; m < points_group_it->same_with_ref_plane_array.ref_plane_size; m++)
				{
					if (points_group_it->same_with_ref_plane_array.is_same_with_ref_plane[m] && \
						self_neighbor_points_group_it->same_with_ref_plane_array.is_same_with_ref_plane[m])
					{
						self_good_neigbor_cnt++;
						is_good_neighbor[i][j][k] = true;
						is_good_neighbor[i][k][j] = true;
					}
				}
			}
		}
	}


	do
	{
		curr_ping_pong_buf_idx = iteration_count % 2;
		next_ping_pong_buf_idx = (iteration_count + 1) % 2;

#pragma omp parallel for
		for (int i = 0; i < static_cast<int>(point_merged_voxel_size); i++)
		{
			BadVoxelMergeOutpuItem* bad_voxel_it = &bad_voxel_merge_item[i];
			PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[bad_voxel_it->bad_voxel_idx];
			//PlaneMergeItem* plane_merge_it = &plane_merge_element[bad_voxel_it->bad_voxel_idx];

			if (!bad_voxel_it->is_remainer_occupied) continue;

			//ParentVoxelIdentifyUpdateItem* points_group_identify_item = new ParentVoxelIdentifyUpdateItem[top_k]; points_group_identify_item is not thread-safe if save the last parent voxel info
			for (int j = 0; j < top_k; j++)
			{
				update_flag_array[j][i] = 0;
				//update the current voxel parent voxel index ,  current voxel parent group index  and the all the current voxel elements  for parent Identifying
				PointsGroupsItem* points_group_it = &bad_voxel_it->points_group->similar_points_group_item[j];
				//if ((points_group_it->plane_mse > points_group_mse_threshold) || (points_group_it->plane_high_mse_ratio > min_high_mse_ratio)) continue; // if mse > threhold , must give up
				if (!points_group_it->is_good_group) continue;
				if (points_group_it->good_neighbor_cnt == 0) continue;   // if have no good neighbor must give up

				//ParentVoxelIdentifyUpdateItem *points_group_identify_it = &points_group_identify_item[j];
				unsigned int current_parent_voxel_idx = points_group_it->parent_voxel_idx[curr_ping_pong_buf_idx];
				unsigned int current_parent_group_idx = points_group_it->parent_group_idx[curr_ping_pong_buf_idx];
				unsigned int parent_bad_voxel_idx = voxel_idx_to_bad_voxel[current_parent_voxel_idx];
				if (parent_bad_voxel_idx > point_merged_voxel_size)
				{
					log_debug("voxel %d group %d parent_bad_voxel_idx %d exceed the size of bad voxels %d", i, j, parent_bad_voxel_idx, point_merged_voxel_size);
					continue;
				}

				//first must check neighbor group in self voxel, update the neigbhor  group index
				for (int k = 0; k < top_k; k++)
				{
					if (k == j) continue;
					if (!is_good_neighbor[i][j][k]) continue;

					PointsGroupsItem* self_neighbor_points_group_it = &bad_voxel_it->points_group->similar_points_group_item[k];
					unsigned int self_neighbor_parent_voxel_idx = self_neighbor_points_group_it->parent_voxel_idx[next_ping_pong_buf_idx];
					unsigned int self_neighbor_parent_group_idx = self_neighbor_points_group_it->parent_group_idx[next_ping_pong_buf_idx];
					if ((self_neighbor_parent_voxel_idx == current_parent_voxel_idx) && (self_neighbor_parent_group_idx == current_parent_group_idx)) continue;
					unsigned int self_neighbor_parent_bad_voxel_idx = voxel_idx_to_bad_voxel[self_neighbor_parent_voxel_idx];
					if (self_neighbor_parent_bad_voxel_idx > point_merged_voxel_size)
					{
						log_debug("voxel %d group %d neighbor_parent_bad_voxel_idx %d exceed the size of bad voxels %d", i, j, self_neighbor_parent_bad_voxel_idx, point_merged_voxel_size);
						continue;
					}
					BadVoxelMergeOutpuItem *self_group_parent_bad_voxel_it = &bad_voxel_merge_item[parent_bad_voxel_idx];
					PointsGroupsItem* self_parent_points_group_it = &self_group_parent_bad_voxel_it->points_group->similar_points_group_item[current_parent_group_idx];
					BadVoxelMergeOutpuItem *self_neighbor_group_parent_bad_voxel_it = &bad_voxel_merge_item[self_neighbor_parent_bad_voxel_idx];
					PointsGroupsItem* self_neighbor_parent_points_group_it = &self_neighbor_group_parent_bad_voxel_it->points_group->similar_points_group_item[self_neighbor_parent_group_idx];

					bool large_result[5], equal_result[5];
					large_result[0] = (self_neighbor_parent_points_group_it->good_neighbor_cnt > self_parent_points_group_it->good_neighbor_cnt);
					equal_result[0] = (self_neighbor_parent_points_group_it->good_neighbor_cnt == self_parent_points_group_it->good_neighbor_cnt);
					large_result[1] = (self_neighbor_parent_points_group_it->plane_mse < self_parent_points_group_it->plane_mse);
					equal_result[1] = (self_neighbor_parent_points_group_it->plane_mse == self_parent_points_group_it->plane_mse);
					large_result[2] = (plane_voxel_array.voxels[self_neighbor_parent_voxel_idx].grid_idx < plane_voxel_array.voxels[current_parent_voxel_idx].grid_idx);
					equal_result[2] = (plane_voxel_array.voxels[self_neighbor_parent_voxel_idx].grid_idx == plane_voxel_array.voxels[current_parent_voxel_idx].grid_idx);
					large_result[3] = (self_neighbor_parent_group_idx < current_parent_group_idx);
					equal_result[3] = (self_neighbor_parent_group_idx == current_parent_group_idx);
					large_result[4] = false;
					equal_result[4] = true;
					unsigned char tmp_parent_update_flag = 0;
					bool comp_result = MathOperation::CompareByMultipleConditions(large_result, equal_result, tmp_parent_update_flag);
					if (!comp_result)
					{
						log_info("all conditions are the same");
						log_info("self_neighbor_parent_voxel_idx = %d current_parent_voxel_idx=%d", self_neighbor_parent_voxel_idx, current_parent_voxel_idx);
						log_info("neighbor good neighbor cnt = %d, current  good neighbor cnt  = %d", self_neighbor_parent_points_group_it->good_neighbor_cnt, self_parent_points_group_it->good_neighbor_cnt);
						log_info("neighbor mse = %d, current mse = %d", self_neighbor_parent_points_group_it->plane_mse, self_parent_points_group_it->plane_mse);
						log_info("neighbor grid idx = %d, current grid idx = %d", plane_voxel_array.voxels[self_neighbor_parent_voxel_idx].grid_idx,\
							plane_voxel_array.voxels[current_parent_voxel_idx].grid_idx);
						log_info("neighbor group idx = %d, current group = %d", self_neighbor_parent_group_idx, current_parent_group_idx);
						continue;
					}
					else
					{
						if (tmp_parent_update_flag == 1)
						{
							points_group_it->parent_voxel_idx[curr_ping_pong_buf_idx] = self_neighbor_parent_voxel_idx;
							points_group_it->parent_group_idx[curr_ping_pong_buf_idx] = self_neighbor_parent_group_idx;
							update_flag_array[j][i] |= 1;
						}
					}
				}

				//update the neighbor  parent voxel index ,  neighbor voxel parent group index  and the all the neighbor voxel  group elements  for parent Identifying
				for (unsigned int k = 0; k < 26; k++)
				{
					NeighborItem* neighbor_it = &plane_voxel->neighbors[k];
					if (!neighbor_it->is_connected) continue;
					PlaneVoxelItem* neighbor_voxel = &plane_voxel_array.voxels[neighbor_it->voxel_idx];
					NeighborPointsGroupItem* points_group_neighbor_it = &points_group_it->neighbour_points_group_item[k];
					unsigned int bad_voxel_idx = voxel_idx_to_bad_voxel[neighbor_it->voxel_idx];
					if (bad_voxel_idx == std::numeric_limits<unsigned int>::max()) continue; 

					BadVoxelMergeOutpuItem *neighbor_bad_voxel_it = &bad_voxel_merge_item[bad_voxel_idx];
					//if (points_group_neighbor_it->points_group_idx > top_k) continue; // if points_group_idx > top_k show there is no good neighbor group in this neighbor voxel
					//if(points_group_neighbor_it->is_good_group)

					for (int m = 0; m < top_k; m++)
					{
						ParentVoxelIdentifyUpdateItem points_group_identify_item;
						ParentVoxelIdentifyUpdateItem *points_group_identify_it = &points_group_identify_item;

						if (!points_group_neighbor_it->is_good_neighbor[m]) continue;

						PointsGroupsItem* neighbor_points_group_it = &neighbor_bad_voxel_it->points_group->similar_points_group_item[m];

						//if ((neighbor_points_group_it->plane_mse > points_group_mse_threshold) || (neighbor_points_group_it->plane_high_mse_ratio > min_high_mse_ratio)) continue; // if mse > threhold , must give up

						unsigned int neighbor_parent_voxel_idx = neighbor_points_group_it->parent_voxel_idx[next_ping_pong_buf_idx];
						unsigned int neighbor_parent_group_idx = neighbor_points_group_it->parent_group_idx[next_ping_pong_buf_idx];
						if ((neighbor_parent_voxel_idx == current_parent_voxel_idx) && (neighbor_parent_group_idx == current_parent_group_idx)) continue;

						unsigned int neighbor_parent_bad_voxel_idx = voxel_idx_to_bad_voxel[neighbor_parent_voxel_idx];
						if (neighbor_parent_bad_voxel_idx >= point_merged_voxel_size)
						{
							log_debug("voxel %d group %d neighbor_parent_bad_voxel_idx %d exceed the size of bad voxels %d", i, j, neighbor_parent_bad_voxel_idx, point_merged_voxel_size);
							continue;
						}
						BadVoxelMergeOutpuItem *group_parent_bad_voxel_it = &bad_voxel_merge_item[parent_bad_voxel_idx];
						PointsGroupsItem* parent_points_group_it = &group_parent_bad_voxel_it->points_group->similar_points_group_item[current_parent_group_idx];

						points_group_identify_it->largest_good_neighbor_count = parent_points_group_it->good_neighbor_cnt;
						points_group_identify_it->smallest_mse = parent_points_group_it->plane_mse;
						points_group_identify_it->grid_idx = plane_voxel_array.voxels[current_parent_voxel_idx].grid_idx;
						points_group_identify_it->current_parent_voxel_idx = current_parent_voxel_idx;
						points_group_identify_it->current_parent_group_idx = current_parent_group_idx;

						BadVoxelMergeOutpuItem *neighbor_group_parent_bad_Voxel_it = &bad_voxel_merge_item[neighbor_parent_bad_voxel_idx];
						PlaneVoxelItem *neighbor_group_parent_voxel = &plane_voxel_array.voxels[neighbor_parent_voxel_idx];
						PointsGroupsItem* neighbor_parent_points_group_it = &neighbor_group_parent_bad_Voxel_it->points_group->similar_points_group_item[neighbor_parent_group_idx];

						points_group_identify_it->neighbor_good_neighbor_count = neighbor_parent_points_group_it->good_neighbor_cnt;
						points_group_identify_it->neighbor_mse = neighbor_parent_points_group_it->plane_mse;
					    points_group_identify_it->neighbor_grid_idx = neighbor_group_parent_voxel->grid_idx;
						points_group_identify_it->neighbor_parent_voxel_idx = neighbor_parent_voxel_idx;
						points_group_identify_it->neighbor_parent_group_idx = neighbor_parent_group_idx;
						ParentIDCondType condition[5] = { GOOD_NEIGHBOR_CNT,PLANE_MSE,GRID_IDX,GROUP_IDX,INVALID_PARA };
						unsigned char parent_update_flag = 0;
						bool check_condition = NeighborParentCompare(condition, points_group_identify_it, parent_update_flag);
						if (!check_condition) continue;
						if (check_condition)
						{
							if (parent_update_flag == 1)
							{
								points_group_it->parent_voxel_idx[curr_ping_pong_buf_idx] = neighbor_parent_voxel_idx;
								points_group_it->parent_group_idx[curr_ping_pong_buf_idx] = neighbor_parent_group_idx;
								update_flag_array[j][i] |= 1;
							}
						}
					}
				}

			}

			//delete[] points_group_identify_item;
		}

		// Check if no more updates
		update_flag = 0;
		for (unsigned int i = 0; i < point_merged_voxel_size; i++) {

			for (int j = 0; j < top_k; j++)
			{
				update_flag |= update_flag_array[j][i];
			}
		}

		// Increment the iteration number
		iteration_count++;

	} while (update_flag > 0);

	// Release memory
	if (is_good_neighbor != NULL)
	{
		delete[]is_good_neighbor;
		is_good_neighbor = NULL;
	}

	for (int i = 0; i < top_k; i++)
	{
		if (update_flag_array[i] != NULL)
		{
			delete[] update_flag_array[i];
			update_flag_array[i] = NULL;
		}
	}
	if (update_flag_array != NULL)
	{
		delete[] update_flag_array;
		update_flag_array = NULL;
	}
	log_info("IdentifyRemainerGoodGroupParentByGoodneighbor iteration %d ", iteration_count);

	return true;


}

void PointsGroupPlane::MergeRemainerGoodGroup(const bool based_on_plane, const PlaneMergeOutput *exist_planes) {

	const unsigned int top_k = MAX_NUM_OF_PLANE_IN_BAD_VOXEL;
	float plane_mse_threshold = plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_PLANE;
	float max_normal_diff = static_cast<float>(std::cos(plane_seg_thresholds.THRESHOLD_MAX_NORMAL_ANGLE_OF_TWO_VOXEL*M_PI / 180));

	if (sys_control_para.scanner_type == UNRE_TYPE_0)
	{
		// to solve the problem of discrete planes genarated in edge of good voxel planes
		//max_normal_diff = std::cos(plane_seg_thresholds.THRESHOLD_MAX_NORMAL_ANGLE_OF_TWO_VOXEL*M_PI*1.5/ 180);
	}	
	
	//unsigned int min_voxel_num_of_plane = plane_seg_params.plane_seg_thresholds.THRESHOLD_MIN_VOXEL_NUM_OF_PLANE;
	unsigned int min_voxel_num_of_plane = plane_seg_thresholds.THRESHOLD_MIN_NUM_OF_GOOD_GROUP_IN_PLANE;
	unsigned int min_points_num_of_plane = plane_seg_thresholds.THRESHOLD_MIN_POINT_NUM_OF_PLANE / 3;
	//log_debug("points group plane_mse_threshold =%f min_voxel_num_of_plane =%d min_points_num_of_plane=%d", plane_mse_threshold, min_voxel_num_of_plane, min_points_num_of_plane);

	unsigned int(*group_cnt_list)[top_k];			// list of total number of groups in plane
	unsigned int(*point_cnt_list)[top_k];			// list of total number of points in plane

	//unsigned int voxel_cnt;							// total number of voxels in plane
	//unsigned int point_cnt;							// total number of points in plane
	//unsigned int parent_voxel_idx;					// parent voxel index to occupied voxel array
	//unsigned int parent_group_idx;					// parent points group index of parent voxel

	//unsigned int group_plane_index;					// plane index
	unsigned int group_plane_cnt;					// total number of planes
	unsigned int(*plane_list)[2];				    // list of parent voxel indexes and parent group index

	//unsigned int(*ref_plane_list)[top_k] = NULL;			 list of reference plane 

	//unsigned int(*parent_group_to_plane_idx)[top_k]; // record the plane index from parent voxel idx and parent group index

	//	is_parent_idx = new bool[point_merged_voxel_size][top_k];
	group_cnt_list = new unsigned int[point_merged_voxel_size][top_k];
	point_cnt_list = new unsigned int[point_merged_voxel_size][top_k];
	parent_group_to_plane_idx = new unsigned int[point_merged_voxel_size][top_k];

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(point_merged_voxel_size); i++) {

		for (unsigned int j = 0; j < top_k; j++)
		{
			//is_parent_idx[i][j] = false;
			group_cnt_list[i][j] = 0;
			point_cnt_list[i][j] = 0;
			parent_group_to_plane_idx[i][j] = -1;
		}
	}
	// Find out the parent voxel index list
	for (unsigned int i = 0; i < point_merged_voxel_size; i++) {

		// Find out the parent points group of all bad voxels and record the total number of voxels belong to it
		BadVoxelMergeOutpuItem* bad_voxel_it = &bad_voxel_merge_item[i];
		if (!bad_voxel_it->is_remainer_occupied) continue;   // bad_voxel_it->points_group == NULL include is_remainer_occupied
		//if (bad_voxel_it->points_group == NULL) continue;
		for (unsigned int j = 0; j < top_k; j++)
		{
			bool debug_con = false;// (bad_voxel_it->bad_voxel_idx == 18601) && (j == 1) || (bad_voxel_it->bad_voxel_idx == 18626) && (j == 0) || (bad_voxel_it->bad_voxel_idx == 18628) && (j == 0) || \
			//	(bad_voxel_it->bad_voxel_idx == 18630) && (j == 0) || (bad_voxel_it->bad_voxel_idx == 18633) && (j == 0);

			PointsGroupsItem* points_group_it = &bad_voxel_it->points_group->similar_points_group_item[j];
			if (points_group_it->points.size == 0) continue; // if have no point, to to next group
			if (points_group_it->good_neighbor_cnt == 0) continue; // if have no good neighbor, not include it
			if (!points_group_it->is_good_group) continue;
			unsigned int parent_voxel_idx = points_group_it->parent_voxel_idx[0];
			unsigned int parent_group_idx = points_group_it->parent_group_idx[0];
			unsigned int parent_bad_voxel_idx = voxel_idx_to_bad_voxel[parent_voxel_idx];
			if (parent_bad_voxel_idx > point_merged_voxel_size) log_info("voxel %d parent voxel idx  %d exceed size of bad voxel %d", bad_voxel_it->bad_voxel_idx, parent_voxel_idx, point_merged_voxel_size);
			if (parent_group_idx > top_k) log_info("voxel%d parent group idx %d exceed max ", bad_voxel_it->bad_voxel_idx, parent_group_idx);
			//if (debug_con) log_info("voxel%d group%d group  parent bad voxel = %d parent group =%d", bad_voxel_it->bad_voxel_idx,j, parent_bad_voxel_idx, parent_group_idx);
			//if (debug_con) log_info("voxel%d group%d group  parent voxel = %d", bad_voxel_it->bad_voxel_idx, j, parent_voxel_idx);
			group_cnt_list[parent_bad_voxel_idx][parent_group_idx] ++;
			point_cnt_list[parent_bad_voxel_idx][parent_group_idx] += points_group_it->points.size;
		}
	}
	// Calculate the total number of group planes
	group_plane_cnt = 0;

#pragma omp parallel for reduction(+:group_plane_cnt)
	for (int i = 0; i < static_cast<int>(point_merged_voxel_size); i++)
	{
		for (unsigned int j = 0; j < top_k; j++)
		{
			if ((group_cnt_list[i][j] > min_voxel_num_of_plane) && (point_cnt_list[i][j] > min_points_num_of_plane))
			{
				group_plane_cnt++;
			}
			else
			{
				group_cnt_list[i][j] = 0;
				point_cnt_list[i][j] = 0;
			}
		}
	}

	// Create the plane list to store all the parent voxel indexes and parent group indexes
	plane_list = new unsigned int[group_plane_cnt][2];
	unsigned int plane_cnt = 0;
	for (unsigned int i = 0, j = 0; i < point_merged_voxel_size; i++)
	{
		for (unsigned int j = 0; j < top_k; j++) {

			if (group_cnt_list[i][j] != 0)
			{
				plane_list[plane_cnt][0] = i;
				plane_list[plane_cnt][1] = j;
				plane_cnt++;
			}
		}
	}

	// log is possible , if log show some group plane has been given up 
	if (plane_cnt != group_plane_cnt) log_debug("plane_cnt %d is not equal to group_plane_cnt %d ", plane_cnt, group_plane_cnt);


	int tmp_planes_size = plane_cnt;

	std::vector<Point3f> plane_parent_avg_normal(tmp_planes_size);

	PointsGroupParentPlaneItem *tmp_planes = NULL;
	bool *is_fitted_plane = NULL;
	int group_plane_idx = 0;
	if (tmp_planes_size == 0)
	{
		points_group_plane_merge_out.size = 0;
		goto end_process;
	}
	
	tmp_planes = new PointsGroupParentPlaneItem[plane_cnt];
	is_fitted_plane = new bool[plane_cnt];
	//#pragma omp parallel for
	
	for (int i = 0; i < tmp_planes_size; i++) {

		PointsGroupParentPlaneItem *group_plane_it = &tmp_planes[i];
		unsigned int  parent_bad_voxel_idx = plane_list[i][0];
		unsigned int  parent_group_idx = plane_list[i][1];
		BadVoxelMergeOutpuItem* parent_bad_voxel_it = &bad_voxel_merge_item[parent_bad_voxel_idx];
		unsigned int parent_voxel_idx = parent_bad_voxel_it->bad_voxel_idx;
		parent_group_to_plane_idx[parent_bad_voxel_idx][parent_group_idx] = i;
		unsigned int group_cnt = group_cnt_list[parent_bad_voxel_idx][parent_group_idx];
		unsigned int point_cnt = point_cnt_list[parent_bad_voxel_idx][parent_group_idx];
		group_plane_it->parent_voxel_idx = parent_voxel_idx;
		group_plane_it->parent_group_idx = parent_group_idx;
		group_plane_it->groups.points_group_id = new std::pair<unsigned int, unsigned int> [group_cnt];
		group_plane_it->groups.size = group_cnt;
		group_plane_it->points.point_idx = new unsigned int[point_cnt];
		group_plane_it->points.size = point_cnt;
		/*group_plane_it->bad_group_points.point_idx = NULL;
		group_plane_it->bad_group_points.size = 0;*/
		MathOperation::ClearSums(&group_plane_it->sums);
		group_plane_it->plane_normal = { 0.f,0.f,0.f };
		group_plane_it->plane_center = { 0.f,0.f,0.f };
		group_plane_it->eigen_mse = std::numeric_limits<float>::infinity();
		group_plane_it->plane_mse = std::numeric_limits<float>::infinity();
		group_plane_it->high_mse_ratio = std::numeric_limits<float>::infinity();
		is_fitted_plane[i] = true;
		if (!based_on_plane)
		{
			if (!ComputeAvgNormal(parent_voxel_idx, parent_group_idx, plane_parent_avg_normal[i]))
			{
				log_error("compute plane %d parent_voxel_idx =%d parent_group_idx%d average normal failed", i, parent_voxel_idx, parent_group_idx);
				BadVoxelMergeOutpuItem * parent_bad_voxel_it = &bad_voxel_merge_item[parent_voxel_idx];
				plane_parent_avg_normal[i] = parent_bad_voxel_it->points_group->similar_points_group_item[parent_group_idx].plane_normal;				
			}
		}

	}

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(point_merged_voxel_size); i++) {

		for (unsigned int j = 0; j < top_k; j++)
		{
			group_cnt_list[i][j] = 0;
			point_cnt_list[i][j] = 0;
		}
	}

	unsigned int group_cnt;							// total number of groups in plane
	unsigned int point_cnt;							// total number of points in plane
	unsigned int parent_voxel_idx;					// parent voxel index to occupied voxel array
	unsigned int parent_group_idx;					// parent points group index of parent voxel
	unsigned int group_plane_index;					// plane index
	unsigned int parent_bad_voxel_idx;
	//float threshold_of_mse_of_plane = plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_PLANE;
	// Assign voxel indexes and points index to points_group_plane_merge_out
	for (unsigned int i = 0; i < point_merged_voxel_size; i++)
	{

		BadVoxelMergeOutpuItem* bad_voxel_it = &bad_voxel_merge_item[i];

		if (!bad_voxel_it->is_remainer_occupied) continue;

		for (unsigned int j = 0; j < top_k; j++)
		{
			//bool debug_con = (bad_voxel_it->bad_voxel_idx == 26224) && (j == 1) || (bad_voxel_it->bad_voxel_idx == 22450) && (j == 0);
			PointsGroupsItem* points_group_it = &bad_voxel_it->points_group->similar_points_group_item[j];
			//if ((points_group_it->plane_mse > voxel_mse_threshold) || (points_group_it->plane_high_mse_ratio > min_high_mse_ratio)) continue; // group mse>threshold  not include it
			if (points_group_it->good_neighbor_cnt == 0) continue; // if have no good neighbor, not include it
			if (points_group_it->points.size == 0) continue;
			if (!points_group_it->is_good_group) continue;
			parent_voxel_idx = points_group_it->parent_voxel_idx[0];
			parent_group_idx = points_group_it->parent_group_idx[0];
			parent_bad_voxel_idx = voxel_idx_to_bad_voxel[parent_voxel_idx];
			group_cnt = group_cnt_list[parent_bad_voxel_idx][parent_group_idx];
			point_cnt = point_cnt_list[parent_bad_voxel_idx][parent_group_idx];
			group_plane_index = parent_group_to_plane_idx[parent_bad_voxel_idx][parent_group_idx];
			if (group_plane_index == -1) continue;
			if (!based_on_plane)
			{
				// check if points group normal  difference with parent group
				float norm_diff = std::fabs( ComputeVectorDotProduct<float>(plane_parent_avg_normal[group_plane_index], points_group_it->plane_normal));
				if (norm_diff < max_normal_diff)
				{
					//log_info("temp plane %d voxel%d group%d norm_diff =%f is given up", group_plane_index, bad_voxel_it->bad_voxel_idx, j, std::acos(norm_diff) *180 / M_PI);
					// If the group is removed, the parent flag must be restored to itself voxel index and group index
					points_group_it->parent_voxel_idx[0] = bad_voxel_it->bad_voxel_idx;
					points_group_it->parent_group_idx[0] = j;
					continue;
				}
			}

			PointsGroupParentPlaneItem *group_plane_it = &tmp_planes[group_plane_index];
			//group_plane_it->voxels.voxel_idx[voxel_cnt] = bad_voxel_it->bad_voxel_idx;
			group_plane_it->groups.points_group_id[group_cnt].first = bad_voxel_it->bad_voxel_idx;
			group_plane_it->groups.points_group_id[group_cnt].second = j;
			group_cnt_list[parent_bad_voxel_idx][parent_group_idx]++;
			//if (debug_con) log_info("voxel%d group%d parent_bad_voxel_idx =%d parent_group_idx =%d group_plane_index = %d",bad_voxel_it->bad_voxel_idx,j, parent_bad_voxel_idx, parent_group_idx, group_plane_index);
			for (unsigned int k = 0; k < points_group_it->points.size; k++)
			{
				unsigned int point_idx = points_group_it->points.point_idx[k];
				group_plane_it->points.point_idx[point_cnt + k] = point_idx;
			}
			point_cnt_list[parent_bad_voxel_idx][parent_group_idx] += points_group_it->points.size;
			MathOperation::PushSums(&group_plane_it->sums, &points_group_it->sums);
		}
	}

	if (!based_on_plane)
	{
		//because points group whose normal difference > max_normal_diff  is removed, here must update the point number of plane
		for (int i = 0; i < tmp_planes_size; i++)
		{
			PointsGroupParentPlaneItem *group_plane_it = &tmp_planes[i];	
			unsigned int parent_bad_voxel_idx = voxel_idx_to_bad_voxel[group_plane_it->parent_voxel_idx];
			group_plane_it->points.size = point_cnt_list[parent_bad_voxel_idx][group_plane_it->parent_group_idx];
		}
	}

#pragma omp parallel for
	for (int i = 0; i < tmp_planes_size; i++)
	{
		PointsGroupParentPlaneItem *group_plane_it = &tmp_planes[i];
		MathOperation::Compute(group_plane_it->points.size, group_plane_it->sums, group_plane_it->plane_normal, group_plane_it->plane_center, group_plane_it->eigen_mse);
		GetPlaneDistMse(group_plane_it, group_plane_it->plane_mse);
		// check if mse of planes is within threshold, give up this plane by set is_fitted_plane[i]= false;
	}


	/*if based on reference planes, since same reference planes are allowed , so must check all the reference planes connected relationsip*/
	if (based_on_plane)
	{
		FilteredGroupPlaneWithRef(exist_planes, tmp_planes_size, tmp_planes, is_fitted_plane);
	}

	for (int i = 0; i < tmp_planes_size; i++)
	{
		PointsGroupParentPlaneItem *group_plane_it = &tmp_planes[i];
		if (group_plane_it->plane_mse > plane_mse_threshold)
		{
			is_fitted_plane[i] = false;		
		}
	}



	plane_cnt = 0;
	for (int i = 0; i < tmp_planes_size; i++)
	{
		if (is_fitted_plane[i])
		{
			plane_cnt++;
		}
	}
	// Create instant of all points_group_plane_merge_out planes

	points_group_plane_merge_out.size = plane_cnt;

	if (plane_cnt == 0)
	{
		goto end_process1;
	}
	points_group_plane_merge_out.planes = new PointsGroupParentPlaneItem[plane_cnt];

	log_debug("group plane size =%d", plane_cnt);

	for (int i = 0; i < static_cast<int>(point_merged_voxel_size); i++) {

		for (unsigned int j = 0; j < top_k; j++)
		{
			parent_group_to_plane_idx[i][j] = -1;
		}
	}

	
	for (int i = 0; i < tmp_planes_size; i++) {

		PointsGroupParentPlaneItem *tmp_group_plane_it = &tmp_planes[i];
		// if is_fitted_plane[i]  true, tmp_group_plane_it->voxels and tmp_group_plane_it->point_idx will release in the end of class with points_group_plane_merge_out
		// if is_fitted_plane[i]  false,  must release here  or in this function
		if (!is_fitted_plane[i])
		{
			if (tmp_group_plane_it->groups.points_group_id != NULL)
			{
				delete[] tmp_group_plane_it->groups.points_group_id;
				tmp_group_plane_it->groups.points_group_id = NULL;
			}

			if (tmp_group_plane_it->points.point_idx != NULL)
			{
				delete[] tmp_group_plane_it->points.point_idx;
				tmp_group_plane_it->points.point_idx = NULL;
			}
			continue;
		}

		unsigned int  parent_bad_voxel_idx = plane_list[i][0];
		unsigned int  parent_group_idx = plane_list[i][1];
		parent_group_to_plane_idx[parent_bad_voxel_idx][parent_group_idx] = group_plane_idx;

		PointsGroupParentPlaneItem *group_plane_it = &points_group_plane_merge_out.planes[group_plane_idx];
		points_group_plane_merge_out.planes[group_plane_idx] = tmp_planes[i];
		group_plane_idx++;
	}

	if (tmp_planes != NULL)
	{
		//tmp_planes voxels or point_idx have been released or assign to the points_group_plane_merge_out
		delete[] tmp_planes;
		tmp_planes = NULL;
	}

	// marked merged flag for each group and each point in merged group
//#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(point_merged_voxel_size); i++)
	{
		BadVoxelMergeOutpuItem* bad_voxel_it = &bad_voxel_merge_item[i];
		if (based_on_plane)
		{
			/*bad_voxel_it->being_in_extended_plane = new bool[points_group_plane_merge_out.size];
			for (unsigned int j = 0; j < points_group_plane_merge_out.size; j++)
			{
				bad_voxel_it->being_in_extended_plane[j] = false;
			}*/
		}
		else
		{
			bad_voxel_it->being_in_group_plane = new bool[points_group_plane_merge_out.size];
			for (unsigned int j = 0; j < points_group_plane_merge_out.size; j++)
			{
				bad_voxel_it->being_in_group_plane[j] = false;
			}
		}

		if (!bad_voxel_it->is_remainer_occupied) continue;

		for (unsigned int j = 0; j < top_k; j++)
		{
			PointsGroupsItem* points_group_it = &bad_voxel_it->points_group->similar_points_group_item[j];
			//if ((points_group_it->plane_mse > voxel_mse_threshold) || (points_group_it->plane_high_mse_ratio > min_high_mse_ratio)) continue; // group mse>threshold  not include it
			if (points_group_it->good_neighbor_cnt == 0) continue; // if have no good neighbor, not include it
			if (points_group_it->points.size == 0) continue;
			if (!points_group_it->is_good_group) continue;

			unsigned int parent_voxel_idx = points_group_it->parent_voxel_idx[0];
			unsigned int parent_group_idx = points_group_it->parent_group_idx[0];
			unsigned int parent_bad_voxel_idx = voxel_idx_to_bad_voxel[parent_voxel_idx];
			unsigned int group_plane_index = parent_group_to_plane_idx[parent_bad_voxel_idx][parent_group_idx];
			if (group_plane_index == -1) continue;
			PointsGroupParentPlaneItem *group_plane_it = &points_group_plane_merge_out.planes[group_plane_index];

			for (unsigned int k = 0; k < points_group_it->points.size; k++)
			{
				unsigned int point_idx = points_group_it->points.point_idx[k];
				unsigned int bad_voxel_point_idx = points_group_it->bad_voxel_point_idx[k];
				bad_voxel_it->point_merged_flag[bad_voxel_point_idx] = true;    // mark the point merge flag in bad voxel item
				if (based_on_plane)
				{
					if (group_plane_it->ref_plane_idx >= exist_planes->size)
					{
						// log is close due to maybe large errors info, but if group plane have correct  same and connected reference plane , this condition should not exist , must have bug
						//log_info("group_plane_index=%d ref_plane_idx exceed size of ref planes =%d", group_plane_index, group_plane_it->ref_plane_idx, exist_planes->size);
						continue;
					}
					unsigned int cur_global_ref_plane_idx = global_ref_plane_idx[group_plane_it->ref_plane_idx];
					if (cur_global_ref_plane_idx < total_flat_plane_size)
					{
						bad_voxel_it->remaining_points_merged_plane[cur_global_ref_plane_idx] = true;
					}
					else
					{
						bad_voxel_it->being_in_group_plane[group_plane_it->ref_plane_idx] = true;
					}
					bad_voxel_it->closest_plane_idx[bad_voxel_point_idx] = global_ref_plane_idx[group_plane_it->ref_plane_idx];
				}
				else
				{
					bad_voxel_it->closest_plane_idx[bad_voxel_point_idx] = group_plane_index + total_flat_plane_size;
					bad_voxel_it->being_in_group_plane[group_plane_index] = true;
				}
				bad_voxel_it->num_of_points_merged++;
			}
		}

		PlaneVoxelItem * cur_voxel = &plane_voxel_array.voxels[bad_voxel_it->bad_voxel_idx];

		if (bad_voxel_it->num_of_points_merged != 0)
		{
			//while finding extended part of plane , if the voxel's have points merged and its is_overall_merged is true, must mark it false
			cur_voxel->is_overall_merged = false;
			bad_voxel_it->is_being_merged = true;
			cur_voxel->is_being_merged = true;
		}

		if (bad_voxel_it->voxel_point_size == bad_voxel_it->num_of_points_merged) bad_voxel_it->is_remainer_occupied = false;

		if (bad_voxel_it->voxel_point_size < bad_voxel_it->num_of_points_merged)
		{
			log_debug("voxel %d bad voxel %d num_of_points_merged %d exceed size of bad voxel points = %d", bad_voxel_it->bad_voxel_idx, i, bad_voxel_it->num_of_points_merged, bad_voxel_it->voxel_point_size);
		}
	}

end_process1:
	{
		if (tmp_planes != NULL)
		{
			for (int i = 0; i < tmp_planes_size; i++)
			{
				PointsGroupParentPlaneItem *group_plane_it = &tmp_planes[i];
				if (group_plane_it->groups.points_group_id != NULL)
				{
					delete[] group_plane_it->groups.points_group_id;
					group_plane_it->groups.points_group_id = NULL;
				}
			}
			delete[] tmp_planes;
			tmp_planes = NULL;
		}
	}

	if (is_fitted_plane != NULL)
	{
		delete[] is_fitted_plane;
		is_fitted_plane = NULL;
	}

end_process:

	plane_parent_avg_normal.clear();
	plane_parent_avg_normal.shrink_to_fit();

	// Release memory
	/*if (ref_plane_list != NULL)
	{
		delete[] ref_plane_list;
		ref_plane_list = NULL;
	}*/

	if (point_cnt_list != NULL)
	{
		delete[] point_cnt_list;
		point_cnt_list = NULL;
	}

	if (plane_list != NULL)
	{
		delete[] plane_list;
		plane_list = NULL;
	}

	if (group_cnt_list != NULL)
	{
		delete[] group_cnt_list;
		group_cnt_list = NULL;
	}

	return;
}

#ifdef SAVE_OUTPUT_FILE_DEBUG
void PointsGroupPlane::MergePointsGroupDebugOutput(std::string output_path, PlaneItemType ref_plane_type)
{

	std::string merge_points_group_path = output_path;
	std::string merge_points_group_file;
	if (ref_plane_type == GOOD_PLANE)
	{
		merge_points_group_file = "good_extend_group_neighbor.txt";
	}
	else if(ref_plane_type == PSEUDO_BAD_PLANE)
	{
		merge_points_group_file = "pseudobad_extend_group_neighbor.txt";
	}
	else
	{
		merge_points_group_file = "realbad_plane_group_neighbor.txt";
	}
	

	std::ofstream merge_points_group_voxel(merge_points_group_path + merge_points_group_file);


	merge_points_group_voxel << "Total number of planes: " << points_group_plane_merge_out.size << '\n';
	unsigned int total_points_cnt_all_plane = 0;
	unsigned int total_voxels_cnt_all_plane = 0;
	/*for (unsigned int i = 0; i < points_group_plane_merge_out.size; i++)
	{
		total_voxels_cnt_all_plane += points_group_plane_merge_out.planes[i].voxels.size;
		total_points_cnt_all_plane += points_group_plane_merge_out.planes[i].points.size + points_group_plane_merge_out.planes[i].bad_group_points.size;
	}*/
	merge_points_group_voxel << "Total number of voxels: " << total_voxels_cnt_all_plane << '\n';
	merge_points_group_voxel << "Total number of points: " << total_points_cnt_all_plane << '\n';

	for (unsigned int i = 0; i < points_group_plane_merge_out.size; i++)
	{

		PointsGroupParentPlaneItem *group_plane_it = &points_group_plane_merge_out.planes[i];

		unsigned int parent_voxel_idx = group_plane_it->parent_voxel_idx;
		unsigned int parent_group_idx = group_plane_it->parent_group_idx;

		merge_points_group_voxel << "Group Plane Index = " << i << ", Parent Voxel Index = " << group_plane_it->parent_voxel_idx \
			<< ", Parent group Index = " << group_plane_it->parent_group_idx << ", Voxel Count = " << group_plane_it->groups.size \
			<< ", Point Count = " << group_plane_it->points.size/* + group_plane_it->bad_group_points.size */<< '\n';

		merge_points_group_voxel << std::setprecision(10) << std::fixed;
		merge_points_group_voxel << i << '\t' << "plane_mse:      " << group_plane_it->plane_mse << '\n';
		merge_points_group_voxel << i << '\t' << "eigen_mse:      " << group_plane_it->eigen_mse << '\n';
		merge_points_group_voxel << i << '\t' << "high_mse_ratio: " << group_plane_it->high_mse_ratio << '\n';
		merge_points_group_voxel << i << '\t' << "plane_normal:   " << group_plane_it->plane_normal << '\n';
		merge_points_group_voxel << i << '\t' << "plane_center:   " << group_plane_it->plane_center << '\n';

		merge_points_group_voxel << "plane  " << i << std::setw(20) << "bad voxel idx" << std::setw(16) << "grid idx\t\n";
		for (unsigned int j = 0; j < group_plane_it->groups.size; j++)
		{
			unsigned int bad_voxel_idx = group_plane_it->groups.points_group_id[j].first;
			//unsigned int point_size = group_plane_it->points.size + group_plane_it->bad_group_points.size;
			unsigned int grid_idx = plane_voxel_array.voxels[bad_voxel_idx].grid_idx;
			merge_points_group_voxel << std::setw(8) << j << std::setw(20) << bad_voxel_idx << std::setw(16) << grid_idx << '\n';
		}

	}
	merge_points_group_voxel.close();
}
#endif

#if 0

bool PointsGroupPlane::IdentifyRemainerBadGroupParent(bool is_first)
{
	const unsigned int top_k = MAX_NUM_OF_PLANE_IN_BAD_VOXEL;
	//float min_point_plane_dist = plane_seg_params.plane_seg_thresholds.THRESHOLD_MIN_DIST_OF_POINT2VOXEL;
	float min_point_plane_dist = plane_seg_thresholds.THRESHOLD_MIN_DIST_OF_POINT2VOXEL;
	//if(based_on_plane)  min_point_plane_dist = plane_seg_thresholds.THRESHOLD_MIN_DIST_OF_2PLANE;
	if (is_first)
	{
		/*if (based_on_plane)
		{
			min_point_plane_dist = plane_seg_thresholds.THRESHOLD_MIN_DIST_OF_2PLANE / 2;
			min_point_plane_dist = plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_PLANE < min_point_plane_dist ? plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_PLANE : min_point_plane_dist;
		}
		else*/
		{
			min_point_plane_dist = plane_seg_thresholds.THRESHOLD_MIN_DIST_OF_POINT2VOXEL / 2;
			min_point_plane_dist = plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_VOXEL < min_point_plane_dist ? plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_VOXEL : min_point_plane_dist;
		}
	}

	float min_normal_diff = std::cos(plane_seg_thresholds.THRESHOLD_MAX_NORMAL_ANGEL_OF_EDGE_POINT*M_PI / 180);

	// Identify parent voxel and parent group idx in all bad groups: and find points in these real bad voxels to be merged, then add to the bad_voxel_merge_item
#pragma omp parallel for
	for (int i = 0; i < point_merged_voxel_size; i++)
	{
		BadVoxelMergeOutpuItem *bad_voxel_merge_it = &bad_voxel_merge_item[i];
		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[bad_voxel_merge_it->bad_voxel_idx];

		if (!bad_voxel_merge_it->is_remainer_occupied) continue;

		for (unsigned int k = 0; k < plane_voxel->points.size; k++)
		{
			//if (bad_voxel_merge_it->point_merged_flag[k]) continue;
			if (bad_voxel_merge_it->closest_plane_idx[k] != std::numeric_limits<unsigned int>::max()) continue; //if this point is not identified  to be of a plane
			Point3f Point = pt_cloud_xyz.points[plane_voxel->points.point_idx[k]];
			Point3f Point_normal = pt_cloud_normal.points[plane_voxel->points.point_idx[k]];

			// first step to find points group plane in self voxel
			bool points_group_plane_found = false;

			for (unsigned int j = 0; j < top_k; j++)
			{
				PointsGroupsItem* points_group = &bad_voxel_merge_it->points_group->similar_points_group_item[j];
				if (points_group->points.size == 0) continue;
				unsigned int parent_voxel_idx = points_group->parent_voxel_idx[0];
				unsigned int parent_group_idx = points_group->parent_group_idx[0];

				unsigned int parent_bad_voxel_idx = voxel_idx_to_bad_voxel[parent_voxel_idx];

				unsigned int group_plane_idx = parent_group_to_plane_idx[parent_bad_voxel_idx][parent_group_idx];


				if (group_plane_idx == -1) continue;

				//if (group_plane_idx >= points_group_plane_merge_out.size) log_debug("group_plane_idx %d exceed size of max planes %d", group_plane_idx, points_group_plane_merge_out.size);

				PointsGroupParentPlaneItem* group_plane_it = &points_group_plane_merge_out.planes[group_plane_idx];
				Point3f plane_normal = group_plane_it->plane_normal;
				Point3f plane_center = group_plane_it->plane_center;
				if (!is_first)
				{
					float normal_diff = std::fabs(MathOperation::ComputeVectorDotProduct<float>(Point_normal, plane_normal));
					if (normal_diff < min_normal_diff) continue;
				}

				float new_dist = MathOperation::ComputePointToPlaneDist<float>(Point, plane_normal, plane_center);
				if ((new_dist < bad_voxel_merge_it->smallest_dist[k]) && (new_dist < min_point_plane_dist))
				{
					bad_voxel_merge_it->is_being_merged = true;
					bad_voxel_merge_it->smallest_dist[k] = new_dist;
					bad_voxel_merge_it->closest_plane_idx[k] = group_plane_idx + total_previous_plane_size;
					plane_voxel->is_being_merged = true;
					points_group_plane_found = true;
				}
			}

			// if not found , do second step:  find points group plane in neighbour
			if (!points_group_plane_found)
			{
				for (unsigned int j = 0; j < 26; j++)
				{
					NeighborItem* neighbor_item = &plane_voxel->neighbors[j];
					if (!neighbor_item->is_occupied) continue;
					if (!neighbor_item->is_connected) continue;
					PlaneVoxelItem* neighbor_plane_voxel = &plane_voxel_array.voxels[neighbor_item->voxel_idx];
					PlaneMergeItem* neighbor_plane_merge_item = &plane_merge_element[neighbor_item->voxel_idx];
					if (neighbor_plane_voxel->is_overall_merged) continue;
					unsigned int neighbor_bad_voxel_idx = voxel_idx_to_bad_voxel[neighbor_item->voxel_idx];
					BadVoxelMergeOutpuItem* neighbor_bad_voxel_merge_it = &bad_voxel_merge_item[neighbor_bad_voxel_idx];
					if (!neighbor_bad_voxel_merge_it->is_remainer_occupied) continue;

					for (unsigned int m = 0; m < top_k; m++)
					{
						PointsGroupsItem* neighbor_points_group = &neighbor_bad_voxel_merge_it->points_group->similar_points_group_item[m];
						if (neighbor_points_group->points.size == 0) continue;

						unsigned int parent_voxel_idx = neighbor_points_group->parent_voxel_idx[0];
						unsigned int parent_group_idx = neighbor_points_group->parent_group_idx[0];

						unsigned int parent_bad_voxel_idx = voxel_idx_to_bad_voxel[parent_voxel_idx];

						if (parent_bad_voxel_idx == std::numeric_limits<unsigned int>::max())
						{
							log_fatal("bad voxel %d group %d neighbor voxel %d group%d parent voxel idx = %d  bad voxel idx =%d ", bad_voxel_merge_it->bad_voxel_idx, k, neighbor_item->voxel_idx, m, parent_voxel_idx, parent_bad_voxel_idx);
						}

						unsigned int group_plane_idx = parent_group_to_plane_idx[parent_bad_voxel_idx][parent_group_idx];

						if (group_plane_idx == -1) continue;
						if (group_plane_idx >= points_group_plane_merge_out.size) log_debug("neighbor group_plane_idx %d exceed size of max planes %d", group_plane_idx, points_group_plane_merge_out.size);
						PointsGroupParentPlaneItem* neighbor_group_plane_it = &points_group_plane_merge_out.planes[group_plane_idx];

						Point3f plane_normal = neighbor_group_plane_it->plane_normal;
						Point3f plane_center = neighbor_group_plane_it->plane_center;
						if (!is_first)
						{
							float normal_diff = std::fabs(MathOperation::ComputeVectorDotProduct<float>(Point_normal, plane_normal));
							if (normal_diff < min_normal_diff) continue;
						}
		
						float new_dist = MathOperation::ComputePointToPlaneDist<float>(Point, plane_normal, plane_center);
						if ((new_dist < bad_voxel_merge_it->smallest_dist[k]) && (new_dist < min_point_plane_dist))
						{
							bad_voxel_merge_it->is_being_merged = true;
							bad_voxel_merge_it->smallest_dist[k] = new_dist;
							bad_voxel_merge_it->closest_plane_idx[k] = group_plane_idx + total_previous_plane_size;
							plane_voxel->is_being_merged = true;
							points_group_plane_found = true;
						}
					}
				}
			}
		}
	}

	return true;

}
void PointsGroupPlane::MergeRemainerBadGroup()
{
	//const unsigned int top_k = MAX_NUM_OF_PLANE_IN_BAD_VOXEL;
	//unsigned int *num_of_points_list;		// list of points number in plane for each points group 

	//float point2plane_dist_threshold = plane_seg_params.plane_seg_thresholds.THRESHOLD_MIN_DIST_OF_2PLANE;	

	//num_of_points_list = new unsigned int[points_group_plane_merge_out.size];  // record number of points being merged in plane

	unsigned int *num_of_points_list;				// list of points number in plane for each points group 

	num_of_points_list = new unsigned int[points_group_plane_merge_out.size];  // record number of points being merged in plane

	memset(num_of_points_list, 0, sizeof(unsigned int)*points_group_plane_merge_out.size);

	for (unsigned int i = 0; i < point_merged_voxel_size; i++)
	{
		BadVoxelMergeOutpuItem* bad_voxel_merge_it = &bad_voxel_merge_item[i];
		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[bad_voxel_merge_it->bad_voxel_idx];

		//if (plane_voxel->is_overall_merged) continue;
		if (!bad_voxel_merge_it->is_remainer_occupied) continue;
		// get number of all the bad voxel points for each plane
		for (unsigned int j = 0; j < points_group_plane_merge_out.size; j++)
		{
			for (unsigned int k = 0; k < plane_voxel->points.size; k++)
			{
				if (bad_voxel_merge_it->point_merged_flag[k]) continue;
				if (bad_voxel_merge_it->closest_plane_idx[k] == j + total_previous_plane_size)
				{
					num_of_points_list[j]++;
				}
			}
		}
	}

	/*for (unsigned int j = 0; j < points_group_plane_merge_out.size; j++)
	{
		log_debug("plane %d points size = %d", j, num_of_points_list[j]);
	}*/

	/*for (int i = 0; i < points_group_plane_merge_out.size; i++)
	{
		points_group_plane_merge_out.planes[i].bad_group_points.size = num_of_points_list[i];
	}*/

	if (num_of_points_list != NULL)
	{
		delete[] num_of_points_list;
		num_of_points_list = NULL;
	}
	

#pragma omp parallel for
	for (int i = 0; i < points_group_plane_merge_out.size; i++)
	{
		PointsGroupParentPlaneItem* group_plane_it = &points_group_plane_merge_out.planes[i];
		//num_of_points_list[i] = 0;
		if (group_plane_it->bad_group_points.size == 0) continue;
		group_plane_it->bad_group_points.point_idx = new unsigned int[group_plane_it->bad_group_points.size];
		memset(group_plane_it->bad_group_points.point_idx, -1, sizeof(unsigned int)* group_plane_it->bad_group_points.size);

		// current point index pointer
		unsigned int* current_plane_point_ptr = group_plane_it->bad_group_points.point_idx;
		for (unsigned int j = 0; j < point_merged_voxel_size; j++)
		{
			BadVoxelMergeOutpuItem* bad_voxel_it = &bad_voxel_merge_item[j];

			unsigned int bad_voxel_index = bad_voxel_it->bad_voxel_idx;

			for (unsigned int k = 0; k < bad_voxel_it->voxel_point_size; k++)
			{
				if (bad_voxel_it->point_merged_flag[k]) continue;

				if (bad_voxel_it->closest_plane_idx[k] == i + total_previous_plane_size)
				{
					unsigned int point_idx = plane_voxel_array.voxels[bad_voxel_index].points.point_idx[k];

					// merge the points of this bad voxel to plane
					*current_plane_point_ptr = point_idx;
					current_plane_point_ptr++;

					//num_of_points_list[i]++;
					/*if (based_on_plane)
					{
						bad_voxel_it->being_in_extended_plane[i] = true;
					}
					else*/
					{
						bad_voxel_it->being_in_group_plane[i] = true;
					}
					bad_voxel_it->num_of_points_merged++;
					Point3f Point = pt_cloud_xyz.points[point_idx].point;
					MathOperation::PushPoint(&group_plane_it->sums, Point);
					bad_voxel_it->point_merged_flag[k] = true;
				}
			}
		}
		//if (num_of_points_list[i] != group_plane_it->bad_group_points.size) log_debug("plane %d bad voxel merge size =%d is not equal to identify size =%d ", i, num_of_points_list[i], group_plane_it->bad_group_points.size);
		//update normals mse and plane center of all the planes
		unsigned total_group_plane_cnt = group_plane_it->bad_group_points.size + group_plane_it->points.size;
		//MathOperation::Compute(total_group_plane_cnt, group_plane_it->sums, group_plane_it->plane_normal, group_plane_it->plane_center,eigen_mse);
		GetPlaneDistMse(group_plane_it, group_plane_it->plane_mse);
	}

	//delete[] num_of_points_list;
	return;

}
#endif

bool PointsGroupPlane::GetPointsGroupNeigbourDebugInfo(const bool based_on_plane)
{
	std::string file_name;
	std::string folder_path = data_output_path;
	file_name = folder_path + "points_group_neighbour.txt";
	std::ofstream points_group_neigbour(file_name);
	const unsigned int top_k = MAX_NUM_OF_PLANE_IN_BAD_VOXEL;

	unsigned int num_of_points_group = 0;
	unsigned int num_of_group_with_neighbours = 0;
	unsigned int num_of_good_groups = 0;

	for (unsigned int i = 0; i < point_merged_voxel_size; i++)
	{
		BadVoxelMergeOutpuItem *bad_voxel_it = &bad_voxel_merge_item[i];
		PlaneVoxelItem *plane_voxel = &plane_voxel_array.voxels[bad_voxel_it->bad_voxel_idx];

		// find all the points group

		if (!bad_voxel_it->is_remainer_occupied)  continue;
		if (plane_voxel->is_being_merged&&plane_voxel->is_overall_merged) continue;

		for (unsigned int j = 0; j < top_k; j++)
		{
			if (bad_voxel_it->points_group == NULL) continue;

			PointsGroupsItem * points_group_it = &bad_voxel_it->points_group->similar_points_group_item[j];
			if (points_group_it->points.size == 0) continue;
			num_of_points_group++;

			bool have_neighbour = false;
			for (unsigned int k = 0; k < 26; k++)
			{
				NeighborItem * neighbor_it = &plane_voxel->neighbors[k];
				if (!neighbor_it->is_connected) continue;
				PlaneVoxelItem *neighbor_voxel = &plane_voxel_array.voxels[neighbor_it->voxel_idx];
				if (neighbor_voxel->is_being_merged&&neighbor_voxel->is_overall_merged) continue;
				unsigned int bad_voxel_idx = voxel_idx_to_bad_voxel[neighbor_it->voxel_idx];
				if (bad_voxel_idx == std::numeric_limits<unsigned int>::max()) continue;
				BadVoxelMergeOutpuItem * neighbor_bad_voxel_it = &bad_voxel_merge_item[bad_voxel_idx];
				if (!neighbor_bad_voxel_it->is_remainer_occupied) continue;
				have_neighbour = true;
				break;
			}

			if (have_neighbour)
			{
				num_of_group_with_neighbours++;
			}

			if (points_group_it->good_neighbor_cnt != 0)
			{
				num_of_good_groups++;
			}

		}
	}

	points_group_neigbour << "num of points group:							" << std::setw(8) << num_of_points_group << '\n';
	points_group_neigbour << "num of group with neighbors:					" << std::setw(8) << num_of_group_with_neighbours << '\n';
	points_group_neigbour << "num of groups with good neighbour:            " << std::setw(8) << num_of_good_groups << '\n';

	int good_neigbour_idx[top_k][26], bad_neigbour_idx[top_k][26]; // record the current points group's good neighbour  index and bad neighbour  index 
	int good_neighbor_voxel_count[top_k];
	int bad_neighbor_voxel_count[top_k];

	for (unsigned int i = 0; i < point_merged_voxel_size; i++)
	{
		BadVoxelMergeOutpuItem *bad_voxel_it = &bad_voxel_merge_item[i];
		PlaneVoxelItem * plane_voxel = &plane_voxel_array.voxels[bad_voxel_it->bad_voxel_idx];
		if (!bad_voxel_it->is_remainer_occupied) continue;
		if (plane_voxel->is_being_merged&&plane_voxel->is_overall_merged) continue;
		BadVoxelPointsGroup *points_group = bad_voxel_it->points_group;
		if (points_group == NULL) continue;
		for (unsigned int j = 0; j < top_k; j++)
		{
			good_neighbor_voxel_count[j] = 0;
			bad_neighbor_voxel_count[j] = 0;
			for (int k = 0; k < 26; k++)
			{
				good_neigbour_idx[j][k] = -1;
				bad_neigbour_idx[j][k] = -1;
			}
		}

		for (unsigned int j = 0; j < top_k; j++)
		{
			PointsGroupsItem *points_group_it = &points_group->similar_points_group_item[j];
			if (points_group_it->points.size == 0) continue;

			for (int k = 0; k < 26; k++)
			{

				NeighborItem* neighbor_it = &plane_voxel->neighbors[k];
				NeighborPointsGroupItem* points_group_neighbor_it = &points_group_it->neighbour_points_group_item[k];
				if (!neighbor_it->is_connected) continue;
				PlaneVoxelItem *neighbor_voxel = &plane_voxel_array.voxels[neighbor_it->voxel_idx];

				unsigned int neighbor_bad_voxel_idx = voxel_idx_to_bad_voxel[neighbor_it->voxel_idx];
				if (neighbor_bad_voxel_idx == std::numeric_limits<unsigned int>::max()) continue;
				BadVoxelMergeOutpuItem *neighbor_bad_voxel_it = &bad_voxel_merge_item[neighbor_bad_voxel_idx];
				if (!neighbor_bad_voxel_it->is_remainer_occupied) continue;
				if (neighbor_voxel->is_being_merged&&neighbor_voxel->is_overall_merged) continue;
				bool have_good_points_group = false;
				if (based_on_plane)
				{
					int good_group_cnt = 0;
					for (int m = 0; m < top_k; m++)
					{
						good_group_cnt = 0;
						if (points_group_neighbor_it->is_good_neighbor[m]) good_group_cnt++;
					}
					if (good_group_cnt > 0) have_good_points_group = true;
				}
				else
				{
					have_good_points_group = points_group_neighbor_it->points_group_idx < top_k;

				}
				if (have_good_points_group)  // if points_group_idx < top_k show this group has good neighbour group
				{
					good_neigbour_idx[j][good_neighbor_voxel_count[j]] = k;
					good_neighbor_voxel_count[j]++;
				}
				else
				{
					BadVoxelPointsGroup* neighbor_points_group = neighbor_bad_voxel_it->points_group;
					if (neighbor_points_group == NULL) continue;
					int bad_group_cnt = 0;
					for (unsigned int m = 0; m < top_k; m++)
					{
						PointsGroupsItem* neighbor_points_group_it = &neighbor_points_group->similar_points_group_item[m];
						if (neighbor_points_group_it->points.size != 0)
						{
							bad_neigbour_idx[j][bad_neighbor_voxel_count[j]] = k;
							bad_group_cnt++;
							break;
						}
					}
					if (bad_group_cnt > 0) bad_neighbor_voxel_count[j]++;
				}
			}
		}

		unsigned int* sorted_good_neighbor_voxel_idx[top_k] = { NULL };
		unsigned int *sorted_bad_neighbor_voxel_idx[top_k] = { NULL };

		for (unsigned int j = 0; j < top_k; j++)
		{
			PointsGroupsItem *points_group_it = &points_group->similar_points_group_item[j];
			if (points_group_it->points.size == 0) continue;
			sorted_good_neighbor_voxel_idx[j] = NULL;
			sorted_bad_neighbor_voxel_idx[j] = NULL;
			if (good_neighbor_voxel_count[j] != 0)
			{
				// Sort good neighbor group according to neighbor voxel index in ascending order
				sorted_good_neighbor_voxel_idx[j] = new unsigned int[good_neighbor_voxel_count[j]];
				for (int k = 0; k < good_neighbor_voxel_count[j]; k++)
				{
					NeighborItem* neighbour_item = &plane_voxel->neighbors[good_neigbour_idx[j][k]];
					sorted_good_neighbor_voxel_idx[j][k] = neighbour_item->voxel_idx;
				}
				std::sort(sorted_good_neighbor_voxel_idx[j], sorted_good_neighbor_voxel_idx[j] + good_neighbor_voxel_count[j]);
			}
			// Sort bad neighbor group according to neighbor voxel index in ascending order
			if (bad_neighbor_voxel_count[j] != 0)
			{
				sorted_bad_neighbor_voxel_idx[j] = new unsigned int[bad_neighbor_voxel_count[j]];
				for (int k = 0; k < bad_neighbor_voxel_count[j]; k++)
				{
					NeighborItem* neighbour_item = &plane_voxel->neighbors[bad_neigbour_idx[j][k]];
					sorted_bad_neighbor_voxel_idx[j][k] = neighbour_item->voxel_idx;

				}
				std::sort(sorted_bad_neighbor_voxel_idx[j], sorted_bad_neighbor_voxel_idx[j] + bad_neighbor_voxel_count[j]);
			}

		}

		points_group_neigbour << "bad voxel " << i << "  voxel idx " << bad_voxel_it->bad_voxel_idx << "  points size " << bad_voxel_it->voxel_point_size << "   merged point size " << bad_voxel_it->num_of_points_merged \
			<< "  similar_group_size " << bad_voxel_it->remainer_similar_group_size << '\n';

		for (unsigned int j = 0; j < top_k; j++)
		{
			PointsGroupsItem *points_group_it = &points_group->similar_points_group_item[j];
			if (points_group_it->points.size == 0) continue;
			for (int k = 0; k < 26; k++)
			{
				NeighborItem* neighbor_it = &plane_voxel->neighbors[k];
				if (!neighbor_it->is_connected) continue;				

				if (good_neighbor_voxel_count[j] != 0)
				{
					for (int m = 0; m < good_neighbor_voxel_count[j]; m++)
					{
						if (neighbor_it->voxel_idx == sorted_good_neighbor_voxel_idx[j][m])
						{
							good_neigbour_idx[j][m] = k;
						}
					}
				}

				if (bad_neighbor_voxel_count[j] != 0)
				{
					for (int m = 0; m < bad_neighbor_voxel_count[j]; m++)
					{
						if (neighbor_it->voxel_idx == sorted_bad_neighbor_voxel_idx[j][m])
						{
							bad_neigbour_idx[j][m] = k;
						}
					}
				}
			}

			points_group_neigbour << "\tvoxel " << bad_voxel_it->bad_voxel_idx << std::setw(12) << "group " << j << "\tgood  neighbours:" << std::setw(8) << good_neighbor_voxel_count[j]\
				<< "\t plane_mse:" << std::setw(8) << points_group_it->plane_mse\
				<< "\t plane_high_mse_ratio:" << std::setw(8) << points_group_it->plane_high_mse_ratio \
				<< "\t parent_voxel_idx:" << std::setw(4) << points_group_it->parent_voxel_idx[0] \
				<< "\tparent_group_idx:" << std::setw(4) << points_group_it->parent_group_idx[0] << "\t\n";

			if (good_neighbor_voxel_count[j] != 0)
			{
				points_group_neigbour << "\tvoxel idx  " << std::setw(12) << "   group idx   " << std::setw(16) << "   normal_diff   " << std::setw(16) << "   plane_dist   " << "   plane_mse   " << "  num_of_point  "\
					<< " high_mse_ratio " << " parent_voxel_idx " << " parent_group_idx " << "\n";

				for (int k = 0; k < good_neighbor_voxel_count[j]; k++)
				{
					unsigned int neighbour_idx = good_neigbour_idx[j][k];

					if( (neighbour_idx == -1)||(neighbour_idx>=26))
					{
						log_info("voxel%d good_neigbour_idx[%d][%d]=%d", bad_voxel_it->bad_voxel_idx,j,k, neighbour_idx);
						continue;
					}
					NeighborItem* neighbour_item = &plane_voxel->neighbors[neighbour_idx];
					unsigned int neighbor_bad_voxel_idx = voxel_idx_to_bad_voxel[neighbour_item->voxel_idx];
					if (neighbor_bad_voxel_idx == std::numeric_limits<unsigned int>::max()) continue;
					NeighborPointsGroupItem *neighbour_group_it = &points_group_it->neighbour_points_group_item[neighbour_idx];
					if (based_on_plane)
					{
						float normal_diff, plane_dist, plane_mse, high_mse_ratio;
						unsigned int num_of_point;
						for (unsigned int m = 0; m < top_k; m++)
						{
							PointsGroupsItem* neighbour_points_group_it = &bad_voxel_merge_item[neighbor_bad_voxel_idx].points_group->similar_points_group_item[m];
							if (!neighbour_group_it->is_good_neighbor[m]) continue;
							if (neighbour_points_group_it->points.size != 0)
							{
								plane_mse = neighbour_points_group_it->plane_mse;

								//normal_diff = neighbour_group_it->normal_diff[m];
								normal_diff = std::fabs( ComputeVectorDotProduct<float>(neighbour_points_group_it->plane_normal, points_group_it->plane_normal));
								float normal_diff_angle = (float)((std::acos(normal_diff) * 180.0) / M_PI);
								//plane_dist = neighbour_group_it->plane_dist[m];
								plane_dist =  ComputePointToPlaneDist<float>(neighbour_points_group_it->plane_center, points_group_it->plane_normal, points_group_it->plane_center);
								num_of_point = neighbour_points_group_it->points.size;
								unsigned int parent_voxel_idx = neighbour_points_group_it->parent_voxel_idx[0];
								unsigned int parent_group_idx = neighbour_points_group_it->parent_group_idx[0];
								high_mse_ratio = neighbour_points_group_it->plane_high_mse_ratio;
								points_group_neigbour << ' ' << std::setw(5) << k << ' ' << std::setw(10) << neighbour_item->voxel_idx << std::setw(14) << m << std::setw(16) << normal_diff_angle << std::setw(18) << plane_dist \
									<< std::setw(18) << plane_mse << std::setw(10) << num_of_point \
									<< std::setw(18) << high_mse_ratio << std::setw(18) << parent_voxel_idx << std::setw(18) << parent_group_idx << "\t\n";
							}
						}

					}
					else
					{
						unsigned int neighbour_group_idx = neighbour_group_it->points_group_idx;
						if (neighbour_group_idx > top_k)
						{
							log_debug("voxel %d group %d neighbor voxel=%d  good neighbour group %d exceed the top_k %d", bad_voxel_it->bad_voxel_idx, j, neighbour_item->voxel_idx, neighbour_group_idx, top_k);
							continue;
						}
						PointsGroupsItem * neighbour_points_group_it = &bad_voxel_merge_item[neighbor_bad_voxel_idx].points_group->similar_points_group_item[neighbour_group_idx];
						//float normal_diff = neighbour_group_it->normal_diff[neighbour_group_idx];
						float normal_diff = std::fabs( ComputeVectorDotProduct<float>(neighbour_points_group_it->plane_normal, points_group_it->plane_normal));

						float normal_diff_angle = (float)((std::acos(normal_diff) * 180.0) / M_PI);

						//float plane_dist = neighbour_group_it->plane_dist[neighbour_group_idx];
						float plane_dist =  ComputePointToPlaneDist<float>(neighbour_points_group_it->plane_center, points_group_it->plane_normal, points_group_it->plane_center);

						float plane_mse = neighbour_points_group_it->plane_mse;
						unsigned int num_of_point = neighbour_points_group_it->points.size;

						unsigned int parent_voxel_idx = neighbour_points_group_it->parent_voxel_idx[0];
						unsigned int parent_group_idx = neighbour_points_group_it->parent_group_idx[0];
						float high_mse_ratio = neighbour_points_group_it->plane_high_mse_ratio;

						points_group_neigbour << ' ' << std::setw(10) << neighbour_item->voxel_idx << std::setw(14) << neighbour_group_idx << std::setw(16) << normal_diff_angle << std::setw(18) << plane_dist \
							<< std::setw(18) << plane_mse << std::setw(10) << num_of_point \
							<< std::setw(18) << high_mse_ratio << std::setw(18) << parent_voxel_idx << std::setw(18) << parent_group_idx << "\t\n";

					}
				}
			}
			points_group_neigbour << "\tvoxel " << bad_voxel_it->bad_voxel_idx << std::setw(12) << "group " << j << "\tbad  neighbours:" << std::setw(8) << bad_neighbor_voxel_count[j]\
				<< "\t plane_mse:" << std::setw(8) << points_group_it->plane_mse\
				<< "\t plane_high_mse_ratio:" << std::setw(8) << points_group_it->plane_high_mse_ratio \
				<< "\t parent_voxel_idx:" << std::setw(8) << points_group_it->parent_voxel_idx[0] \
				<< "\t parent_group_idx:" << std::setw(8) << points_group_it->parent_group_idx[0] << "\t\n";

#if 1
			if (bad_neighbor_voxel_count[j] != 0)
			{

				points_group_neigbour << "\tidx  " << "\tvoxel idx  " << std::setw(12) << "   group idx   " << std::setw(16) << "   normal_diff   " << std::setw(16) << "   plane_dist   " << "   plane_mse   " << "  num_of_point  "\
					<< " high_mse_ratio " << " parent_voxel_idx " << " parent_group_idx " << "\n";
				/*if (based_on_plane)
				{
					for (int k = 0; k < good_neighbor_voxel_count[j]; k++)
					{
						unsigned int neighbour_idx = good_neigbour_idx[j][k];
						NeighborItem* neighbour_item = &plane_voxel->neighbors[neighbour_idx];
						NeighborPointsGroupItem *neighbour_group_it = &points_group_it->neighbour_points_group_item[neighbour_idx];
						unsigned int bad_voxel_idx = voxel_idx_to_bad_voxel[neighbour_item->voxel_idx];
						float normal_diff, plane_dist, plane_mse, high_mse_ratio;
						unsigned int num_of_point;
						for (unsigned int m = 0; m < top_k; m++)
						{
							PointsGroupsItem* neighbour_points_group_it = &bad_voxel_merge_item[bad_voxel_idx].points_group->similar_points_group_item[m];
							if (neighbour_group_it->is_good_neighbor[m]) continue;
							if (neighbour_points_group_it->points.size != 0)
							{
								plane_mse = neighbour_points_group_it->plane_mse;

								normal_diff = std::fabs( ComputeVectorDotProduct<float>(neighbour_points_group_it->plane_normal, points_group_it->plane_normal));
								float normal_diff_angle = (float)((std::acos(normal_diff) * 180.0) / M_PI);
								plane_dist =  ComputePointToPlaneDist<float>(neighbour_points_group_it->plane_center, points_group_it->plane_normal, points_group_it->plane_center);
								num_of_point = neighbour_points_group_it->points.size;
								unsigned int parent_voxel_idx = neighbour_points_group_it->parent_voxel_idx[0];
								unsigned int parent_group_idx = neighbour_points_group_it->parent_group_idx[0];
								high_mse_ratio = neighbour_points_group_it->plane_high_mse_ratio;
								points_group_neigbour << ' ' << std::setw(5) << k << ' ' << std::setw(10) << neighbour_item->voxel_idx << std::setw(14) << m << std::setw(16) << normal_diff_angle << std::setw(18) << plane_dist \
									<< std::setw(18) << plane_mse << std::setw(10) << num_of_point \
									<< std::setw(18) << high_mse_ratio << std::setw(18) << parent_voxel_idx << std::setw(18) << parent_group_idx << "\t\n";
							}
						}
					}
				}*/

				for (int k = 0; k < bad_neighbor_voxel_count[j]; k++)
				{
					unsigned int neighbour_idx = bad_neigbour_idx[j][k];
					NeighborItem* neighbour_item = &plane_voxel->neighbors[neighbour_idx];
					NeighborPointsGroupItem *neighbour_group_it = &points_group_it->neighbour_points_group_item[neighbour_idx];
					unsigned int bad_voxel_idx = voxel_idx_to_bad_voxel[neighbour_item->voxel_idx];
					//unsigned int neighbour_group_idx = neighbour_group_it->points_group_idx;
					//if (neighbour_group_idx > top_k) log_debug("neighbour_group_idx%d exceed the top_k %d", neighbour_group_idx, top_k);
					float normal_diff, plane_dist, plane_mse, high_mse_ratio;
					unsigned int num_of_point;
					for (unsigned int m = 0; m < top_k; m++)
					{
						PointsGroupsItem* neighbour_points_group_it = &bad_voxel_merge_item[bad_voxel_idx].points_group->similar_points_group_item[m];
						if (neighbour_points_group_it->points.size != 0)
						{
							plane_mse = neighbour_points_group_it->plane_mse;

							//normal_diff = neighbour_group_it->normal_diff[m];
							normal_diff = std::fabs( ComputeVectorDotProduct<float>(neighbour_points_group_it->plane_normal, points_group_it->plane_normal));
							float normal_diff_angle = (float)((std::acos(normal_diff) * 180.0) / M_PI);
							//plane_dist = neighbour_group_it->plane_dist[m];
							plane_dist =  ComputePointToPlaneDist<float>(neighbour_points_group_it->plane_center, points_group_it->plane_normal, points_group_it->plane_center);
							num_of_point = neighbour_points_group_it->points.size;
							unsigned int parent_voxel_idx = neighbour_points_group_it->parent_voxel_idx[0];
							unsigned int parent_group_idx = neighbour_points_group_it->parent_group_idx[0];
							high_mse_ratio = neighbour_points_group_it->plane_high_mse_ratio;
							points_group_neigbour << ' ' << std::setw(5) << k << ' ' << std::setw(10) << neighbour_item->voxel_idx << std::setw(14) << m << std::setw(16) << normal_diff_angle << std::setw(18) << plane_dist \
								<< std::setw(18) << plane_mse << std::setw(10) << num_of_point \
								<< std::setw(18) << high_mse_ratio << std::setw(18) << parent_voxel_idx << std::setw(18) << parent_group_idx << "\t\n";
						}
					}
				}
			}
#endif
		}

		for (unsigned int j = 0; j < top_k; j++)
		{
			if (sorted_good_neighbor_voxel_idx[j] != NULL)
			{
				delete[] sorted_good_neighbor_voxel_idx[j];
				sorted_good_neighbor_voxel_idx[j] = NULL;
			}
			if (sorted_bad_neighbor_voxel_idx[j] != NULL)
			{
				delete[] sorted_bad_neighbor_voxel_idx[j];
				sorted_bad_neighbor_voxel_idx[j] = NULL;
			}
		}

	}

	points_group_neigbour.close();
	return true;
}


void PointsGroupPlane::PointsGroupDebug()
{
	const unsigned int top_k = MAX_NUM_OF_PLANE_IN_BAD_VOXEL;
	std::string group_xyz_path = data_output_path + "group_xyz\\";
	std::string group_info_path = data_output_path + "group_xyz_info\\";

	if (IOData::createDirectory(group_xyz_path))
	{
		log_error("createDirectory %s failed", group_xyz_path.c_str());
		return;
	}

	if (IOData::createDirectory(group_info_path))
	{
		log_error("createDirectory %s failed", group_info_path.c_str());
		return;
	}

	std::string file_type;
	std::string group_file_name = "group_xyz";
	file_type = ".txt";

	std::string group_info_file_name = "group_info";

	float mse_threhold = plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_VOXEL;

	for (unsigned int i = 0; i < point_merged_voxel_size; i++)
	{
		BadVoxelMergeOutpuItem* bad_voxel_it = &bad_voxel_merge_item[i];
		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[bad_voxel_it->bad_voxel_idx];
		//if (!bad_voxel_it->is_remainer_occupied) continue;
		BadVoxelPointsGroup* points_group = bad_voxel_it->points_group;
		unsigned int voxel_idx = bad_voxel_it->bad_voxel_idx;
		bool voxel_condition = false;// (voxel_idx == 164) || (voxel_idx == 101) || (voxel_idx == 102) || (voxel_idx == 163);// || (voxel_idx == 7585) || (voxel_idx == 7600) || (voxel_idx == 7615);
		for (unsigned int j = 0; j < top_k; j++)
		{
			if (bad_voxel_it->points_group == NULL) continue;
			PointsGroupsItem* points_group_it = &bad_voxel_it->points_group->similar_points_group_item[j];
			/*if (voxel_condition)
			{
				std::cout << "voxel_idx" << voxel_idx << '_' << j << "   points size::\t" << points_group_it->points.size << std::endl;
			}*/
			if (points_group_it->points.size == 0) continue;
			if (voxel_condition)
			{
				std::stringstream voxel_group_id;
				voxel_group_id << voxel_idx << '_' << j;
				std::string whole_path = group_xyz_path + group_file_name + voxel_group_id.str() + file_type;
				FeaturesIO::SavePoint3fData(whole_path, pt_cloud_xyz, points_group_it->points);
			}
		}

		if (voxel_condition)
		{
			std::stringstream voxel_id;
			voxel_id << voxel_idx;
			std::string group_points_info_file = group_info_path + group_info_file_name + voxel_id.str() + file_type;
			std::ofstream group_plane_points_info(group_points_info_file);
			group_plane_points_info << "group: idx \t" << std::setw(10) << "mse: " << std::setw(25) << "normal: " << std::setw(40) << "center: " << std::setw(35) << "high_mse_ratio:" << std::endl;
			for (unsigned int j = 0; j < top_k; j++)
			{
				PointsGroupsItem* points_group_it = &bad_voxel_it->points_group->similar_points_group_item[j];
				if (points_group_it->points.size == 0) continue;
				group_plane_points_info << std::setprecision(6) << std::fixed;
				group_plane_points_info << std::setw(6) << j << std::setw(16) << points_group_it->plane_mse << std::setw(5) << points_group_it->plane_normal << std::setw(5) << points_group_it->plane_center << std::setw(12) << points_group_it->plane_high_mse_ratio << std::endl;

			}
			group_plane_points_info.close();
		}
	}
}

bool PointsGroupPlane::IdentifyConnectedPlaneFromBadVoxel(bool &is_same_connected, unsigned int &plane_idx, const PointsGroupsItem *points_group_it, const unsigned int bad_voxel_idx, const PlaneMergeOutput *exist_planes)
{
	float close_dist_2plane = plane_seg_thresholds.THRESHOLD_DIST_OF_2VOXEL_CONNECTED;
	//float min_normal_diff_2plane = std::cos(plane_seg_thresholds.THRESHOLD_MAX_NORMAL_ANGEL_OF_2PLANE*M_PI / 180.0);
	//float min_dist_2plane = plane_seg_thresholds.THRESHOLD_MIN_DIST_OF_2PLANE;

	BadVoxelMergeOutpuItem *bad_voxel_it = &bad_voxel_merge_item[bad_voxel_idx];
	PlaneVoxelItem *plane_voxel = &plane_voxel_array.voxels[bad_voxel_it->bad_voxel_idx];
	is_same_connected = false;
	for (unsigned int k = 0; k < exist_planes->size; k++)
	{
		if (global_ref_plane_idx[k] < total_flat_plane_size)
		{
			if (!bad_voxel_it->remaining_points_merged_plane[global_ref_plane_idx[k]]) continue;
		}
		else
		{
			if (!bad_voxel_it->being_in_group_plane[k]) continue;
		}
		//if ((!bad_voxel_it->is_same_with_pseudobad_parent_plane[k]) && (!bad_voxel_it->remaining_points_merged_plane[k])) continue;

		if (!points_group_it->same_with_ref_plane_array.is_same_with_ref_plane[k]) continue;
		std::vector<unsigned int> plane_points(0);
		for (unsigned int m = 0; m < plane_voxel->points.size; m++)
		{
			if (bad_voxel_it->closest_plane_idx[m] != global_ref_plane_idx[k]) continue;
			plane_points.push_back(plane_voxel->points.point_idx[m]);
		}

		if (plane_points.size() == 0) continue;
		bool is_close = MathOperation::Identify2PointsGroupCLoseByDistance(pt_cloud_xyz, plane_points, points_group_it->points, close_dist_2plane);
		if (is_close)
		{
			is_same_connected = true;
			plane_idx = k;
			break;//if find the connected plane, goto next seach, otherwise, must go on 
		}
	}

	return true;
}


bool PointsGroupPlane::IdentifyConnectedPlaneFromNeighbor(bool &is_same_connected, unsigned int &plane_idx, const PointsGroupsItem *points_group_it, const unsigned int bad_voxel_idx, const PlaneMergeOutput *exist_planes)
{
	float close_dist_2plane = plane_seg_thresholds.THRESHOLD_DIST_OF_2VOXEL_CONNECTED;
	/*float min_normal_diff_2plane = std::cos(plane_seg_thresholds.THRESHOLD_MAX_NORMAL_ANGEL_OF_2PLANE*M_PI / 180.0);
	float min_dist_2plane = plane_seg_thresholds.THRESHOLD_MIN_DIST_OF_2PLANE;*/
	BadVoxelMergeOutpuItem *bad_voxel_it = &bad_voxel_merge_item[bad_voxel_idx];
	PlaneVoxelItem *plane_voxel = &plane_voxel_array.voxels[bad_voxel_it->bad_voxel_idx];
	for (int i = 0; i < 26; i++)
	{
		NeighborItem *neighbor_it = &plane_voxel->neighbors[i];
		if (!neighbor_it->is_connected) continue;
		PlaneVoxelItem *neighbor_plane_voxel = &plane_voxel_array.voxels[neighbor_it->voxel_idx];
		if (neighbor_plane_voxel->is_overall_merged)// if neighbour is_overall_merged is true , it have only one plane
		{
			if(!neighbor_plane_voxel->is_being_merged) continue;
			unsigned int neighbor_parent_voxel_idx = plane_merge_element[neighbor_it->voxel_idx].parent_voxel_idx[0];
			unsigned int neighbor_plane_idx = voxel_to_plane_idx[neighbor_parent_voxel_idx];
			//unsigned int group_parent_bad_voxel_idx = voxel_idx_to_bad_voxel[points_group_it->parent_voxel_idx[0]];
			if ((neighbor_plane_idx < global_ref_plane_idx[0]) || (neighbor_plane_idx > global_ref_plane_idx[exist_planes->size-1])) continue;
			unsigned int neighbor_ref_plane_idx = neighbor_plane_idx - global_ref_plane_idx[0];
			if (!points_group_it->same_with_ref_plane_array.is_same_with_ref_plane[neighbor_ref_plane_idx]) continue;
			bool is_close = MathOperation::Identify2PointsGroupCLoseByDistance(pt_cloud_xyz, neighbor_plane_voxel->points, points_group_it->points, close_dist_2plane);
			if (is_close)
			{
				is_same_connected = true;
				plane_idx = neighbor_ref_plane_idx;
				break;//if find the connected plane, goto next seach, otherwise, must go on 
			}
		}
		else
		{
			unsigned int neighbor_bad_voxel_idx = voxel_idx_to_bad_voxel[neighbor_it->voxel_idx];
			if (std::numeric_limits<unsigned int>::max() != neighbor_bad_voxel_idx)
			{
				//BadVoxelMergeOutpuItem* neighbor_bad_voxel_merge_it = &bad_voxel_merge_item[neighbor_bad_voxel_idx];
				IdentifyConnectedPlaneFromBadVoxel(is_same_connected, plane_idx, points_group_it, neighbor_bad_voxel_idx, exist_planes);
				if (is_same_connected)
				{
					break;
				}
			}
		}
	}

	return true;
}

bool PointsGroupPlane::IdentifyParentPointsGroupConnectedWithRef(bool &is_same_connected, unsigned int &plane_idx, const PointsGroupParentPlaneItem* group_plane, const PlaneMergeOutput *exist_planes)
{
	const unsigned int top_k = MAX_NUM_OF_PLANE_IN_BAD_VOXEL;
	float min_normal_diff_2plane = static_cast<float>(std::cos(plane_seg_thresholds.THRESHOLD_MAX_NORMAL_ANGEL_OF_2PLANE*M_PI / 180.0));
	float min_plane_dist = plane_seg_thresholds.THRESHOLD_MIN_DIST_OF_2PLANE;
	float max_plane_mse = plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_PLANE;
	is_same_connected = false;	
	unsigned int parent_bad_voxel_idx = voxel_idx_to_bad_voxel[group_plane->parent_voxel_idx];

//#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(point_merged_voxel_size); i++)
	{
		BadVoxelMergeOutpuItem *bad_voxel_it = &bad_voxel_merge_item[i];
		PlaneVoxelItem *plane_voxel = &plane_voxel_array.voxels[bad_voxel_it->bad_voxel_idx];
		if (!bad_voxel_it->is_remainer_occupied) continue;
		//if (bad_voxel_it->points_group == NULL) continue;
		for (unsigned int j = 0; j < top_k; j++)
		{
			PointsGroupsItem* points_group_it = &bad_voxel_it->points_group->similar_points_group_item[j];
			if (!points_group_it->is_good_group) continue;
			unsigned int cur_parent_bad_voxel_idx = voxel_idx_to_bad_voxel[points_group_it->parent_voxel_idx[0]];
			if ((cur_parent_bad_voxel_idx != parent_bad_voxel_idx) || (points_group_it->parent_group_idx[0] != group_plane->parent_group_idx)) continue;
			//bool debug_condition =(bad_voxel_it->bad_voxel_idx == 26224)||((parent_bad_voxel_idx == 1758)&&(group_plane->parent_group_idx==1));
			//if (debug_condition)
			//{
			//	for (int k = 0; k < exist_planes->size; k++)
			//	{
			//		if (points_group_it->same_with_ref_plane_array.is_same_with_ref_plane[k])
			//			log_info("voxel%d have ref plane %d global plane =%d parent_bad_voxel_idx =%d parent_group_idx=%d", bad_voxel_it->bad_voxel_idx, k, global_ref_plane_idx[k], parent_bad_voxel_idx, group_plane->parent_group_idx);
			//	}
			//}
			bool tmp_same_connected = false;
			IdentifyConnectedPlaneFromNeighbor(tmp_same_connected, plane_idx, points_group_it, i, exist_planes);
			if (tmp_same_connected)
			{
				// since each points-group can be added into several reference plane, here must check group plane is still same with the reference plane to escape find error reference plane
				PlaneMergeOutputItem* ref_plane = &exist_planes->planes[plane_idx];
				Point3f ref_plane_normal = exist_planes->planes[plane_idx].plane_normal;
				float normal_diff = std::fabs( ComputeVectorDotProduct<float>(group_plane->plane_normal, ref_plane->plane_normal));
				float plane_dist =  ComputePointToPlaneDist<float>(group_plane->plane_center, ref_plane->plane_normal, ref_plane->plane_center);
				//if (debug_condition) log_info("bad_voxel_it->bad_voxel_idx %d group%d  with plane %d  normal_diff =%f, plane_dist =%f",\
				//	bad_voxel_it->bad_voxel_idx,j, plane_idx, normal_diff, plane_dist);
				if ((plane_dist < min_plane_dist) && (normal_diff > min_normal_diff_2plane))
				{
					is_same_connected = true;
				}

			}

			if (is_same_connected)
			{
				break;
			}
			//else
			//{
			//	// second step check if self voxel have the plane same and connected 
			//	IdentifyConnectedPlaneFromBadVoxel(is_same_connected, plane_idx,points_group_it, i, exist_planes);
			//	if (is_same_connected) break;
			//}
		}

		if (is_same_connected&&plane_idx < exist_planes->size) break;
	}
	return true;	
}

//debug for save plane points by voxel name and plane_idx
bool PointsGroupPlane::SaveGroupPlanePointsDebug(const std::string output_path, unsigned int plane_idx)
{

	std::string file_type;
	std::string plane_file_name = "group_plane_idx_xyz";
	std::string voxel_ouput_path = output_path + "voxel_xyz\\";

	if (IOData::createDirectory(voxel_ouput_path))
	{
		log_error("createDirectory %s failed", voxel_ouput_path.c_str());
		return false;
	}
	file_type = ".txt";
	std::stringstream plane_id;
	if (plane_idx > points_group_plane_merge_out.size)
	{
		log_error("plane_idx exceed size of group planes = %d ", plane_idx, points_group_plane_merge_out.size);
		return false;
	}
	PointsGroupParentPlaneItem* group_plane_it = &points_group_plane_merge_out.planes[plane_idx];
	std::string voxel_file_name = "voxel_xyz";
	for (unsigned int j = 0; j < group_plane_it->groups.size; j++)
	{
		std::stringstream voxel_id;
		voxel_id.str("");
		unsigned int voxel_idx = group_plane_it->groups.points_group_id[j].first;
		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[voxel_idx];
		voxel_id << voxel_idx;

		std::string voxel_file = voxel_ouput_path + voxel_file_name + voxel_id.str() + file_type;
		FeaturesIO::SavePoint3fData(voxel_file, pt_cloud_xyz, plane_voxel->points);
	}
	plane_id << plane_idx;
	std::string plane_file = voxel_ouput_path + plane_file_name + plane_id.str() + file_type;
	FeaturesIO::SavePoint3fData(plane_file, pt_cloud_xyz, group_plane_it->points);

	return true;
}



bool PointsGroupPlane::FilteredGroupPlaneWithRef(const PlaneMergeOutput *exist_planes, const unsigned int group_plane_size, PointsGroupParentPlaneItem* group_planes, bool * is_fitted_plane)
{
	float min_normal_diff = static_cast<float>(std::cos((plane_seg_thresholds.THRESHOLD_MAX_NORMAL_ANGEL_OF_EDGE_POINT) * M_PI / 180));
	float min_plane_dist = plane_seg_thresholds.THRESHOLD_MIN_DIST_OF_2PLANE;
	float max_plane_mse = plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_PLANE;

	const unsigned int top_k = MAX_NUM_OF_PLANE_IN_BAD_VOXEL;

	for (int i = 0; i < static_cast<int>(group_plane_size); i++)
	{
		bool is_same_connected = false; // here is_connected represent the new plane is connected and in same plane with one of exist_planes
		unsigned int parent_voxel_idx = group_planes[i].parent_voxel_idx;
		unsigned int parent_group_idx = group_planes[i].parent_group_idx;
		unsigned int plane_idx = -1;
		IdentifyParentPointsGroupConnectedWithRef(is_same_connected, plane_idx, &group_planes[i], exist_planes);
		if (!is_same_connected)
		{
			//log_info("tmp plane = %d parent_bad_voxel_idx %d parent_group_idx%d  with plane %d is_same_connected =%d ", i, parent_bad_voxel_idx, parent_group_idx, plane_idx, is_same_connected);
			is_fitted_plane[i] = false;
		}
		else
		{
			bool debug_con = false;// (exist_planes->planes[0].plane_type == GOOD_PLANE) && (plane_idx == 38);

			group_planes[i].ref_plane_idx = plane_idx;
			if (debug_con) log_info("group plane%d parent_voxel_idx=%d parent_group_idx =%d connected plane %d", i, parent_voxel_idx, parent_group_idx, plane_idx);
			if (plane_idx >= exist_planes->size)
			{
				//it is impossible , must have bug here
				log_info("group plane%d parent_voxel_idx=%d parent_group_idx =%d connected plane %d error", i, parent_voxel_idx, parent_group_idx, plane_idx);
			}
		}
	}

	return true;

}

bool PointsGroupPlane::ComputeAvgNormal(const unsigned int parent_voxel_idx, const unsigned int parent_group_idx, Point3f &avg_normal)
{
	const unsigned int top_k = MAX_NUM_OF_PLANE_IN_BAD_VOXEL;

	if ((parent_voxel_idx >= plane_voxel_array.size) || (parent_group_idx >= top_k))
	{
		log_error("input error parent_voxel_idx =%d parent_group_idx =%d", parent_voxel_idx, parent_group_idx);
		return false;
	}

	unsigned int bad_voxel_idx = voxel_idx_to_bad_voxel[parent_voxel_idx];
	if (bad_voxel_idx >= point_merged_voxel_size)
	{
		log_error("input error parent_voxel_idx =%d bad_voxel_idx =%d", parent_voxel_idx, bad_voxel_idx);
		return false;
	}

	BadVoxelMergeOutpuItem *bad_voxel_it = &bad_voxel_merge_item[bad_voxel_idx];
	PlaneVoxelItem *plane_voxel = &plane_voxel_array.voxels[bad_voxel_it->bad_voxel_idx];
	PlaneMergeItem* plane_merge_it = &plane_merge_element[parent_voxel_idx];
	
	if (bad_voxel_it->points_group == NULL)
	{
		log_error("input error parent_voxel_idx =%d points_group is NULL", parent_voxel_idx);
		return false;
	}

	PointsGroupsItem* points_group_it = &bad_voxel_it->points_group->similar_points_group_item[parent_group_idx];
	SumforCovariance sums;
	PointInVoxelArray point_array;
	point_array.point_idx = NULL;
	point_array.size = 0;

	MathOperation::AssignSums(&sums, &points_group_it->sums);
	unsigned int point_cnt = points_group_it->points.size;

	for (int k = 0; k < 26; k++)
	{
		NeighborItem* neighbor_it = &plane_voxel->neighbors[k];
		NeighborPointsGroupItem *neighbor_group_it = &points_group_it->neighbour_points_group_item[k];
		if (!neighbor_it->is_connected) continue;
		unsigned int neighbor_bad_voxel_idx = voxel_idx_to_bad_voxel[neighbor_it->voxel_idx];
		if (neighbor_bad_voxel_idx == std::numeric_limits<unsigned int>::max()) continue;
		BadVoxelMergeOutpuItem *neighbor_bad_voxel_it = &bad_voxel_merge_item[neighbor_bad_voxel_idx];
		//if (!neighbor_bad_voxel_it->neighbor_connected[k]) continue;
		if (neighbor_group_it->points_group_idx >= top_k) continue;
		PointsGroupsItem* neighbor_points_group_it = &neighbor_bad_voxel_it->points_group->similar_points_group_item[neighbor_group_it->points_group_idx];
		point_cnt += neighbor_points_group_it->points.size;
	}
	point_array.size = point_cnt;
	point_array.point_idx = new unsigned int[point_array.size];
	point_cnt = 0;
	std::memcpy(point_array.point_idx, points_group_it->points.point_idx, sizeof(unsigned int) * points_group_it->points.size);
	point_cnt = points_group_it->points.size;
	for (int k = 0; k < 26; k++)
	{
		NeighborItem* neighbor_it = &plane_voxel->neighbors[k];
		NeighborPointsGroupItem *neighbor_group_it = &points_group_it->neighbour_points_group_item[k];
		if (!neighbor_it->is_connected) continue;
		unsigned int neighbor_bad_voxel_idx = voxel_idx_to_bad_voxel[neighbor_it->voxel_idx];
		if (neighbor_bad_voxel_idx == std::numeric_limits<unsigned int>::max()) continue;
		BadVoxelMergeOutpuItem *neighbor_bad_voxel_it = &bad_voxel_merge_item[neighbor_bad_voxel_idx];
		if (neighbor_group_it->points_group_idx >= top_k) continue;
		PointsGroupsItem* neighbor_points_group_it = &neighbor_bad_voxel_it->points_group->similar_points_group_item[neighbor_group_it->points_group_idx];
		std::memcpy(&point_array.point_idx[point_cnt], neighbor_points_group_it->points.point_idx, sizeof(unsigned int)* neighbor_points_group_it->points.size);
		MathOperation::PushSums(&sums, &neighbor_points_group_it->sums);
		point_cnt += neighbor_points_group_it->points.size;
	}

	Point3f group_center;
	float eigen_mse;
	MathOperation::Compute(point_cnt, sums, avg_normal, group_center, eigen_mse);
	if (point_array.point_idx != NULL)
	{
		delete[] point_array.point_idx;
		point_array.point_idx = NULL;
	}

	return true;
}


void PointsGroupPlane::PointsGroupPlaneMergeFromRemainer(const bool based_on_plane, const PlaneMergeOutput * base_planes, PointsGroupPlaneArray *points_group_planes)
{
	TIMING_DECLARE(TP1)

	//check  if the remainer voxel is occupied after MergeBadVoxels
	/*#pragma omp parallel for
	for (int i = 0; i < point_merged_voxel_size; i++)
	{
		BadVoxelMergeOutpuItem *bad_voxel_merge_it = &bad_voxel_merge_item[i];
		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[bad_voxel_merge_it->bad_voxel_idx];
		if (plane_voxel->points.size <= bad_voxel_merge_it->num_of_points_merged) bad_voxel_merge_it->is_remainer_occupied = false;
	}*/

	TIMING_BEGIN(TP1)
	BadVoxelGroupInit();
	TIMING_END_ms("BadVoxelGroupInit", TP1)

	// compute normal difference for each bad voxel who have points not merged
	TIMING_BEGIN(TP1)
	if(based_on_plane)
	{
		global_ref_plane_idx.resize(base_planes->size);
		for (int i = 0; i < static_cast<int>(base_planes->size); i++)
		{
			if (base_planes->planes[i].plane_type == GOOD_PLANE)
			{
				global_ref_plane_idx[i] = i;
			}
			else if(base_planes->planes[i].plane_type == PSEUDO_BAD_PLANE)
			{
				global_ref_plane_idx[i] = i + good_voxel_plane_size;
			}
			else
			{
				global_ref_plane_idx[i] = i + good_voxel_plane_size + pseudobad_voxel_plane_size;
			}
		}
		GetRemainerNormalDiffByPlanes(base_planes);
	}
	else
	{
		GetRemainerNormalDiff();
	}
	TIMING_END_ms("GetRemainerNormalDiff", TP1)

	// group the top k normal similary points for each voxel
	TIMING_BEGIN(TP1)
	GroupRemainerNormalDiffTopk();
	TIMING_END_ms("GroupRemainerNormalDiffTopk", TP1)


	//reshape the groups by remove the points whose plane distance > threshold
	TIMING_BEGIN(TP1)
	ReshapeGroupByPlaneDist(based_on_plane);
	TIMING_END_ms("ReshapeGroupByPlaneDist", TP1)
	// compute the neighbours info of points group
	TIMING_BEGIN(TP1)
	if (based_on_plane)
	{
		AssignRemainerNeighborsWithRef(base_planes);
	}
	else
	{
		AssignRemainerNeighbors();
	}
	TIMING_END_ms("AssignRemainerNeighbors", TP1)

	// Identify parent for all the points groups of bad voxels
	TIMING_BEGIN(TP1)
	if (based_on_plane)
	{
		IdentifyRemainerGoodGroupParentByGoodneighbor();
	}
	else
	{
		IdentifyRemainerGoodGroupParent();
	}
	TIMING_END_ms("IdentifyRemainerGoodGroupParent", TP1)

#ifdef SAVE_OUTPUT_FILE_DEBUG
		TIMING_BEGIN(TP1)
	if (debug_config_params.points_group_neighbor_output_debug&&!based_on_plane)
		GetPointsGroupNeigbourDebugInfo(based_on_plane);
	TIMING_END_ms("GetPointsGroupNeigbourDebugInfo", TP1)
#endif

	// plane merge from the groups of bad voxels
	TIMING_BEGIN(TP1)
	MergeRemainerGoodGroup(based_on_plane, base_planes);
	TIMING_END_ms("MergeRemainerGoodGroup", TP1)


	/*if (!based_on_plane)
	SaveGroupPlanePointsDebug(data_output_path, 8);*/
 //   if(based_on_plane)
	//{
	//	for (int i = 0; i < points_group_plane_merge_out.size; i++)
	//	{
	//		PointsGroupParentPlaneItem* cur_group_plane = &points_group_plane_merge_out.planes[i];
	//		if (global_ref_plane_idx[cur_group_plane->ref_plane_idx] == 38)
	//		{
	//			if (cur_group_plane->parent_voxel_idx != 5280) continue;
	//			SaveGroupPlanePointsDebug(data_output_path, i);
	//		}
	//	}		
	//}

#ifdef SAVE_OUTPUT_FILE_DEBUG
	if (debug_config_params.neighbour_info_debug)
	{
		PlaneItemType ref_plane_type;
		TIMING_BEGIN(TP1)
		if (based_on_plane)
		{
			ref_plane_type = base_planes->planes[0].plane_type;
		}
		else
		{
			ref_plane_type = REAL_BAD_PLANE;
		}
		MergePointsGroupDebugOutput(data_output_path, ref_plane_type);
		TIMING_END_ms("MergePointsGroupDebugOutput", TP1)
	}
#endif
	points_group_planes->planes = points_group_plane_merge_out.planes;
	points_group_planes->size = points_group_plane_merge_out.size;
}

