#include "config.h"
#include <fstream>
#include <iostream>
#include "PlaneMerge.h"
#include "log.h"
#include "MathOperation.hpp"
#include <iomanip>
#include "PlaneSegmentation.h"
#include "ParentVoxelStruct.h"
#include "InOutData.h"
#include "util_time.hpp"

using namespace ModuleStruct;

PlaneMerge::PlaneMerge(PlaneSegmentation *plane_segmentation) {
	plane_seg_ptr = plane_segmentation;
	init();
}

PlaneMerge::~PlaneMerge() {
	FreeMemory();
}


void PlaneMerge::init() 
{
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
	data_output_path.clear();
	data_output_path.assign(plane_seg_ptr->current_out_path.c_str());
	max_plane_size = plane_seg_ptr->plane_merge_out.size;
	total_flat_plane_size = plane_seg_ptr->good_voxel_plane_size + plane_seg_ptr->pseudobad_voxel_plane_size;
	sys_control_para = plane_seg_ptr->sys_control_para;
	plane_merge_array.planes = NULL;
	plane_merge_array.size = 0;
}
void PlaneMerge::FreeMemory() {

}



// find same plane
void PlaneMerge::IdentifySameNormalPlanes(bool**is_same_plane, unsigned int plane_size, bool last_merge)
{
	bool is_cad = sys_control_para.is_cad;
	float min_normal_diff = static_cast<float>(std::cos(plane_seg_thresholds.THRESHOLD_MAX_NORMAL_ANGEL_OF_2PLANE * M_PI / 180));
	float min_plane_dist = plane_seg_thresholds.THRESHOLD_MIN_DIST_OF_2PLANE;
	
	// if the parent voxel type is real bad voxel, the threshod may be select more than good or pesuod bad voxel plane
	float min_bad_voxel_plane_dist;
	float min_bad_voxel_plane_normal_diff;
	if (!is_cad)
	{
		min_bad_voxel_plane_normal_diff = static_cast<float>(std::cos(plane_seg_thresholds.THRESHOLD_MAX_NORMAL_ANGEL_OF_2PLANE * 1.5 * M_PI / 180));
		min_bad_voxel_plane_dist = plane_seg_thresholds.THRESHOLD_MIN_DIST_OF_2PLANE * 1.5;
	}
	else
	{
		min_bad_voxel_plane_normal_diff = static_cast<float>(std::cos(plane_seg_thresholds.THRESHOLD_MAX_NORMAL_ANGEL_OF_2PLANE * M_PI / 180));
		min_bad_voxel_plane_dist = plane_seg_thresholds.THRESHOLD_MIN_DIST_OF_2PLANE;
	}

	// min number if a good plane whose normal is accurate
	unsigned int threshold_of_good_plane_voxel_size = plane_seg_thresholds.THRESHOLD_MIN_GOOD_PLANE_VOXEL_SIZE;
	log_info("min_normal_diff =%f min_plane_dist=%f", std::acos(min_normal_diff)*180/M_PI, min_plane_dist);
	log_info("min_bad_voxel_plane_normal_diff =%f min_bad_voxel_plane_dist=%f", std::acos(min_bad_voxel_plane_normal_diff) * 180 / M_PI, min_bad_voxel_plane_dist);

	// if the plane center is large , and all the plane is good , use this distance scale to judge the same plane
	float large_plane_dist_scale = static_cast<float>(std::sin(plane_seg_thresholds.THRESHOLD_MAX_NORMAL_ANGEL_OF_2PLANE * M_PI / 180 / 5));


//#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(plane_size); i++)
	{
		Point3f plane_normal, plane_center, other_plane_normal, other_plane_center;
		unsigned int point_cnt_of_plane, point_cnt_of_other_plane;
		unsigned int parent_voxel_idx, other_parent_voxel_idx;
		unsigned int cur_good_voxel_size, cur_other_good_voxel_size;
		float compare_dist;

		PlaneMergeOutputItem * output_plane_it = &plane_merge_array.planes[i];
		plane_normal = output_plane_it->plane_normal;
		plane_center = output_plane_it->plane_center;
		point_cnt_of_plane = output_plane_it->total_point_cnt;
		parent_voxel_idx = output_plane_it->parent_voxel_idx;
		cur_good_voxel_size = output_plane_it->voxels.size;

		for (unsigned int j = 0; j < plane_size; j++)
		{
			if (j == i) continue;
			if (is_same_plane[i][j]) continue;
			PlaneMergeOutputItem* other_output_plane_it = &plane_merge_array.planes[j];
			other_plane_normal = other_output_plane_it->plane_normal;
			other_plane_center = other_output_plane_it->plane_center;
			point_cnt_of_other_plane = other_output_plane_it->total_point_cnt;
			other_parent_voxel_idx = other_output_plane_it->parent_voxel_idx;
			cur_other_good_voxel_size = other_output_plane_it->voxels.size;

			float normal_sim = std::fabs( ComputeVectorDotProduct<float,Point3f>(plane_normal, other_plane_normal));
			if (normal_sim > 1.0f) normal_sim = 1.0f;

			float other_plane_dist =  ComputePointToPlaneDist<float, Point3f>(plane_center, other_plane_normal, other_plane_center);

			float plane_dist =  ComputePointToPlaneDist<float, Point3f>(other_plane_center, plane_normal, plane_center);

			bool same_plane_condition = false;
			bool debug_condition = false;// ((i == 41) && (j == 43)) || ((i == 41) && (j == 53)) || ((i == 53) && (j == 103)) || ((i == 103) && (j == 53));
			float plane_center_dist = 0.f;
			if ((output_plane_it->plane_type == GOOD_PLANE) && (other_output_plane_it->plane_type == GOOD_PLANE))
			{
				/* Generally escape the different planes distance  probably have small distance from center to plane  due to normal difference , we check the both distances of two planes */
				plane_center_dist =  ComputePointToPointDist<float, Point3f>(plane_center, other_plane_center);
				float scale_min_plane_dist = min_plane_dist;

				if ((plane_center_dist > (voxel_params.length_x_of_voxel + voxel_params.length_y_of_voxel + voxel_params.length_z_of_voxel)*5)&&!is_cad)
				{
					//find the small distance , project the center to calculate other distance
					if (plane_dist < other_plane_dist)
					{
						Point3f project_pt = project_point2plane(other_plane_center, plane_normal, plane_center);
						other_plane_dist = ComputePointToPlaneDist<float, Point3f>(project_pt, other_plane_normal, other_plane_center);
					}
					else
					{
						Point3f project_pt = project_point2plane(plane_center, other_plane_normal, other_plane_center);
						plane_dist = ComputePointToPlaneDist<float, Point3f>(project_pt, plane_normal, plane_center);
					}

					//	float cur_plane_dist_scale = static_cast<float>(std::sin(std::acos(normal_sim) * M_PI / 180));
					//	cur_plane_dist_scale = large_plane_dist_scale > cur_plane_dist_scale ? cur_plane_dist_scale : large_plane_dist_scale;
					//	scale_min_plane_dist = cur_plane_dist_scale * plane_center_dist;
					//	scale_min_plane_dist = scale_min_plane_dist > min_plane_dist ? scale_min_plane_dist : min_plane_dist;
					if(debug_condition) log_info("plane[%d,%d] plane_center_dist =%f plane distance threshold change into %f", i, j, plane_center_dist, scale_min_plane_dist);
				}

				//if ((cur_good_voxel_size < threshold_of_good_plane_voxel_size)&& (cur_other_good_voxel_size > threshold_of_good_plane_voxel_size) ||\
				//	(cur_other_good_voxel_size < threshold_of_good_plane_voxel_size)&& (cur_good_voxel_size > threshold_of_good_plane_voxel_size))
				//{
				//	compare_dist = cur_good_voxel_size > cur_other_good_voxel_size ? plane_dist : other_plane_dist;
				//	same_plane_condition = (normal_sim > min_normal_diff) && (compare_dist < scale_min_plane_dist);
				//}
				//else
				//{
				//	same_plane_condition = (normal_sim > min_normal_diff) && (plane_dist < scale_min_plane_dist) && (other_plane_dist < scale_min_plane_dist);
				//}
				bool check_mse_can_merge = false;
				if (last_merge)
				{
					PlaneMergeOutputItem* base_refer_plane = output_plane_it;
					PlaneMergeOutputItem* other_base_refer_plane = other_output_plane_it;
					float thres = base_refer_plane->plane_mse * 1;
					for (unsigned int m = 0; m < other_base_refer_plane->voxels.size; m++) 
					{
						if (check_mse_can_merge)
							break;
						unsigned int voxel_in_plane_idx = other_base_refer_plane->voxels.voxel_idx[m];
						unsigned int point_in_voxel_cnt = plane_voxel_array.voxels[voxel_in_plane_idx].points.size;

						for (unsigned int n = 0; n < point_in_voxel_cnt; n++) 
						{
							unsigned int point_in_plane_idx = plane_voxel_array.voxels[voxel_in_plane_idx].points.point_idx[n];
							if (ComputePointToPlaneDist<float, Point3f>(pt_cloud_xyz.points[point_in_plane_idx], base_refer_plane->plane_normal, base_refer_plane->plane_center) < thres)
							{
								check_mse_can_merge = true;
								break;
							}
						}
					}
				}
				if ((cur_good_voxel_size > threshold_of_good_plane_voxel_size) && (cur_other_good_voxel_size > threshold_of_good_plane_voxel_size))
				{
					if (last_merge)
						same_plane_condition = (normal_sim > min_normal_diff) && check_mse_can_merge;
					else
						same_plane_condition = (normal_sim > min_normal_diff) && (plane_dist < scale_min_plane_dist) && (other_plane_dist < scale_min_plane_dist);
				}
				else
				{
					//if no plane good voxel number is more than threshold, just confirm compare dist according to the point count of plane
					float compare_normal_diff = min_normal_diff;
					if ((cur_good_voxel_size < threshold_of_good_plane_voxel_size) && (cur_other_good_voxel_size < threshold_of_good_plane_voxel_size))
					{
						compare_dist = point_cnt_of_plane > point_cnt_of_other_plane ? plane_dist : other_plane_dist;
						if ((point_cnt_of_plane > 10 * point_cnt_of_other_plane) || (point_cnt_of_other_plane > 10 * point_cnt_of_plane))
						{
							compare_normal_diff = min_bad_voxel_plane_normal_diff;
						}
						else
						{
							compare_normal_diff = min_normal_diff;
						}

					}
					else
					{
						compare_dist = cur_good_voxel_size > cur_other_good_voxel_size ? plane_dist : other_plane_dist;
						compare_normal_diff = min_normal_diff;
					}
					if (last_merge)
						same_plane_condition = (normal_sim > compare_normal_diff) && check_mse_can_merge;
					else
						same_plane_condition = (normal_sim > compare_normal_diff) && (compare_dist < scale_min_plane_dist);
				}
			}
			else
			{
				if ((output_plane_it->plane_type > GOOD_PLANE) && (other_output_plane_it->plane_type > GOOD_PLANE))
				{
					/* if both two planes are not good voxel plane ,check the psudobad voxel size if have enough size ,assume its normal is accurate*/
					if ((cur_good_voxel_size < threshold_of_good_plane_voxel_size) || (cur_other_good_voxel_size < threshold_of_good_plane_voxel_size))
					{
						compare_dist = cur_good_voxel_size > cur_other_good_voxel_size ? plane_dist : other_plane_dist;
					}
					else
					{
						// if both planes have enough pseudobad voxels ,check the average dist.
						compare_dist = (plane_dist + other_plane_dist) / 2;
					}

					bool first_plane_condition = (normal_sim > min_bad_voxel_plane_normal_diff) && (compare_dist < min_plane_dist);
					//if (last_merge)
					//	first_plane_condition = (normal_sim > min_bad_voxel_plane_normal_diff);
					bool second_plane_condition = (normal_sim > min_bad_voxel_plane_normal_diff) && (plane_dist < min_bad_voxel_plane_dist) && (other_plane_dist < min_bad_voxel_plane_dist);
					//if (last_merge)
					//	second_plane_condition = (normal_sim > min_bad_voxel_plane_normal_diff);
					same_plane_condition = first_plane_condition || second_plane_condition;

				}
				else
				{
					compare_dist = output_plane_it->plane_type <= other_output_plane_it->plane_type ? plane_dist : other_plane_dist;
					if ((cur_good_voxel_size < threshold_of_good_plane_voxel_size) && (cur_other_good_voxel_size < threshold_of_good_plane_voxel_size))
					{
						//if (last_merge)
						//	same_plane_condition = (normal_sim > min_normal_diff);
						//else
							same_plane_condition = (normal_sim > min_normal_diff) && (compare_dist < min_bad_voxel_plane_dist);
					}
					else
					{
						plane_center_dist =  ComputePointToPointDist<float>(plane_center, other_plane_center);
						float scale_min_plane_dist = min_plane_dist;
						if ((plane_center_dist > (voxel_params.length_x_of_voxel + voxel_params.length_y_of_voxel + voxel_params.length_z_of_voxel) * 5)&&!is_cad)
						{
							float cur_plane_dist_scale = static_cast<float>(std::sin(std::acos(normal_sim) * M_PI / 180));
							cur_plane_dist_scale = large_plane_dist_scale > cur_plane_dist_scale ? cur_plane_dist_scale : large_plane_dist_scale;
							scale_min_plane_dist = cur_plane_dist_scale * plane_center_dist;
							scale_min_plane_dist = scale_min_plane_dist > min_plane_dist ? scale_min_plane_dist : min_plane_dist;
							if (debug_condition) log_info("plane[%d,%d] plane_center_dist =%f plane distance threshold change into %f", i, j, plane_center_dist, scale_min_plane_dist);
						}

						//if (last_merge)
						//	same_plane_condition = (normal_sim > min_normal_diff);
						//else
							same_plane_condition = (normal_sim > min_normal_diff) && (compare_dist < scale_min_plane_dist);
						if ((output_plane_it->plane_type == REAL_BAD_PLANE) || (other_output_plane_it->plane_type == REAL_BAD_PLANE))
						{
							//if (last_merge)
							//	same_plane_condition = (normal_sim > min_bad_voxel_plane_normal_diff);
							//else
								same_plane_condition = (normal_sim > min_bad_voxel_plane_normal_diff) && (compare_dist < scale_min_plane_dist);
						}
					}
				}
			}

#ifdef SAVE_OUTPUT_FILE_DEBUG
			if (debug_condition)
			{
				log_info("plane %d and %d parent_voxel_type=%d, other_parent_voxel_type=%d plane_center_dist =%f", i, j, output_plane_it->plane_type, other_output_plane_it->plane_type, plane_center_dist);
				log_info("plane %d and %d normal_sim=%f, plane_dist=%.10f other_plane_dist =%f compare_dist =%f", i, j, std::acos(normal_sim) * 180 / M_PI, plane_dist, other_plane_dist, compare_dist);
				log_info("plane %d and %d point_cnt_of_plane=%d, point_cnt_of_other_plane=%d", i, j, point_cnt_of_plane, point_cnt_of_other_plane);
				log_info("plane %d and %d cur_good_voxel_size=%d, other_good_voxel_size=%d", i, j, cur_good_voxel_size, cur_other_good_voxel_size);
			}
#endif
			if (same_plane_condition)
			{
				is_same_plane[i][j] = true;
				is_same_plane[j][i] = true;
#ifdef SAVE_OUTPUT_FILE_DEBUG
				if (debug_config_params.merge_same_plane_output_debug)
					log_info("plane[%d,%d] is same normal", i, j);
#endif
			}
		}
	}

}

void PlaneMerge::AddPlaneToSamePlaneGroup(const unsigned int plane_in_group, const unsigned int same_group_idx, const unsigned int plane_size, bool**is_same_plane, bool**is_connected)
{
	// if it is the new group , must find again in all the other plane to add the same group plane
	//unsigned int same_group_idx = same_plane_group_array.same_plane_group_idx[plane_in_group];
	//find same plane again in all the planes 
	for (unsigned int i = 0; i < plane_size; i++)
	{
		//if (i == plane_in_group) continue;
		unsigned int sameplane_idx_of_plane = same_plane_group_array.same_plane_group_idx[i];
		if (sameplane_idx_of_plane != -1) continue;
		if ((is_connected[i][plane_in_group]) && (is_same_plane[i][plane_in_group]))
		{
			//if find one same and connected plane with current plane, must check it is same normal with all the good plane in the group
			bool is_same_with_all_planes_in_group = true;
			int un_fit_plane_cnt = 0;
			int same_plane_cnt = 0;
			for (unsigned int j = 0; j < plane_size; j++)
			{
				unsigned int cur_tmp_same_group_idx = same_plane_group_array.same_plane_group_idx[j];
				if (cur_tmp_same_group_idx == same_group_idx)
				{
					PlaneMergeOutputItem *plane_it = &plane_merge_array.planes[j];
					//only check with good plane(good voxel>threshold)
					same_plane_cnt++;
					if((plane_it->voxels.size < plane_seg_thresholds.THRESHOLD_MIN_GOOD_PLANE_VOXEL_SIZE)) continue;
					if (!is_same_plane[j][i])
					{
						//is_same_with_all_planes_in_group = false;
						un_fit_plane_cnt++;
					}
				}
			}

			if (un_fit_plane_cnt > same_plane_cnt*2/3)
			{
				log_info("plane [%d] un_fit_plane_cnt =%d same_plane_cnt =%d in same_group_idx =%d", i, un_fit_plane_cnt, same_plane_cnt, same_group_idx);
				is_same_with_all_planes_in_group = false;
			}
			// new same plane can be added to the group  only when it is same with all good plane in group 
			if (is_same_with_all_planes_in_group)
			{
				same_plane_group_array.same_plane_group_idx[i] = same_group_idx;
				AddPlaneToSamePlaneGroup(i, same_group_idx, plane_size, is_same_plane, is_connected);
			}
		}
	}


}

bool IfClosedPointsInPointsArray(const PointArray& pt_cloud_xyz, const PointInPlaneArray& first, const Point3f& first_normal, const Point3f& first_center, const float& first_band,
	const PointInPlaneArray& second, const Point3f& second_normal, const Point3f& second_center, const float& second_band, float distance)
{
	for (int i = 0; i < first.size; i++)
	{
		ModuleStruct::Point3f out_point = pt_cloud_xyz.points[first.point_idx[i]];
		//remove the noise points > band
		if (ComputePointToPlaneDist<float, Point3f>(out_point, first_normal, first_center) > first_band)
			continue;
		for (int j = 0; j < second.size; j++)
		{
			ModuleStruct::Point3f in_point = pt_cloud_xyz.points[second.point_idx[j]];
			//remove the noise points > band
			if (ComputePointToPlaneDist<float, Point3f>(in_point, second_normal, second_center) > second_band)
				continue;
			ModuleStruct::Point3f delta_point = out_point - in_point;
			float dist = std::fabs(delta_point.x) + std::fabs(delta_point.y) + std::fabs(delta_point.z);
			if (/*ComputePointToPointDist<float>(out_point, in_point)*//*cv::norm(out_point - in_point)*/dist < distance)
			{
				//std::cout << "out: " << out_point.x << "\t" << out_point.y << "\t" << out_point.z << std::endl;
				//std::cout << "in: " << in_point.x << "\t" << in_point.y << "\t" << in_point.z << std::endl;
				return true;
			}
		}
	}
	return false;
}

void PlaneMerge::IdentifyConnectedPlanesBypoints(bool** is_same_plane, bool** is_connected)
{
#pragma omp parallel for
	for (int i = 0; i < total_plane_size; i++)
	{
		PlaneMergeOutputItem* outside_plane_item = &plane_merge_array.planes[i];
		if (outside_plane_item->points.size <= 0 && outside_plane_item->extended_part_points.size <= 0 && outside_plane_item->multiplane_points.size <= 0)
			continue;
		for (int j = 0; j < total_plane_size; j++)
		{
			if (j == i) continue;
			if (is_connected[i][j]) continue;
			PlaneMergeOutputItem* inside_plane_item = &plane_merge_array.planes[j];
			if (inside_plane_item->points.size <= 0 && inside_plane_item->extended_part_points.size <= 0 && inside_plane_item->multiplane_points.size <= 0)
				continue;
			if (is_same_plane[i][j])
			{
				bool isConnected = false;
				PointInPlaneArray* first_parray=NULL;
				PointInPlaneArray* second_parray=NULL;
				for (int m = 0; m < 2; m++)
				{
					if (isConnected)
						break;
					if (m == 0)
						first_parray = &outside_plane_item->multiplane_points;
					else if (m == 1)
						first_parray = &outside_plane_item->extended_part_points;
					else if (m == 2)
						first_parray = &outside_plane_item->points;
					for (int n = 0; n < 2; n++)
					{
						if (n == 0)
							second_parray = &inside_plane_item->multiplane_points;
						else if (n == 1)
							second_parray = &inside_plane_item->extended_part_points;
						else if (n == 2)
							second_parray = &inside_plane_item->points;
						if (IfClosedPointsInPointsArray(pt_cloud_xyz, *first_parray, outside_plane_item->plane_normal, outside_plane_item->plane_center, outside_plane_item->plane_mse * 2, *second_parray, inside_plane_item->plane_normal, inside_plane_item->plane_center, inside_plane_item->plane_mse * 2, plane_seg_ptr->point_density * 2))
						{
							isConnected = true;
							break;
						}
					}
				}

				if (isConnected)
				{
					is_connected[i][j] = true;
					is_connected[j][i] = true;
				}
			}
		}
	}
}

void PlaneMerge::IdentifyConnectedPlanesFromBadVoxel(bool **is_connected, const unsigned int bad_voxel_idx)
{
	BadVoxelMergeOutpuItem* bad_voxel_merge_it = &bad_voxel_merge_item[bad_voxel_idx];
	PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[bad_voxel_merge_it->bad_voxel_idx];
	float max_connected_distance = plane_seg_thresholds.THRESHOLD_DIST_OF_2VOXEL_CONNECTED;
	// may be have  voxels in pseudobad planes after generating planes by pseudobad voxes
	if (plane_voxel->is_being_merged&&plane_voxel->is_overall_merged) return;

	for (unsigned int i = 0; i < total_plane_size; i++)
	{
		PlaneMergeOutputItem * plane_it = &plane_merge_array.planes[i];
		unsigned int first_plane_idx = i;
		if (first_plane_idx < total_flat_plane_size)
		{
			if (!bad_voxel_merge_it->remaining_points_merged_plane[first_plane_idx]) continue;
		}
		else
		{
			unsigned int points_group_plane_idx = first_plane_idx - total_flat_plane_size;
			if (!bad_voxel_merge_it->being_in_group_plane[points_group_plane_idx]) continue;
		}
		std::vector<unsigned int> first_plane_points;
		first_plane_points.clear();
		first_plane_points.shrink_to_fit();
		for (unsigned int k = 0; k < plane_voxel->points.size; k++)
		{
			unsigned int another_plane_idx = bad_voxel_merge_it->closest_plane_idx[k];
			if (another_plane_idx == first_plane_idx) first_plane_points.push_back(plane_voxel->points.point_idx[k]);
		}

		if (first_plane_points.size() > 1) //Assume a voxel at least have two points in a plane if connected with other plane
		{
			// first step find connected plane in neighbor voxel 
			IdentifyConnectedPlanesFromNeighbors(is_connected, bad_voxel_idx, first_plane_idx, first_plane_points);
			// second step find connected plane in self bad voxel
			std::vector<unsigned int> sec_plane_points;
			sec_plane_points.clear();
			sec_plane_points.shrink_to_fit();
			for (unsigned int j = i + 1; j < total_plane_size; j++)
			{
				unsigned int sec_plane_idx = j;
				//if (sec_plane_idx == first_plane_idx) continue; not need 
				if (is_connected[first_plane_idx][sec_plane_idx]) continue;
				if (sec_plane_idx < total_flat_plane_size)
				{
					if (!bad_voxel_merge_it->remaining_points_merged_plane[sec_plane_idx]) continue;
				}
				else
				{
					unsigned int points_group_plane_idx = sec_plane_idx - total_flat_plane_size;
					if (!bad_voxel_merge_it->being_in_group_plane[points_group_plane_idx]) continue;
				}
				//if (!bad_voxel_merge_it->remaining_points_merged_plane[sec_plane_idx]) continue;
				for (unsigned int k = 0; k < bad_voxel_merge_it->voxel_point_size; k++)
				{
					unsigned int sec_another_plane_idx = bad_voxel_merge_it->closest_plane_idx[k];
					if (sec_another_plane_idx == sec_plane_idx) sec_plane_points.push_back(plane_voxel->points.point_idx[k]);
				}

				if (sec_plane_points.size() > 1)
				{
					bool is_close = MathOperation::Identify2PointsGroupCLoseByDistance(pt_cloud_xyz, first_plane_points, sec_plane_points, max_connected_distance);
					if (is_close)
					{
						is_connected[first_plane_idx][sec_plane_idx] = true;
						is_connected[sec_plane_idx][first_plane_idx] = true;
					}
				}
			}

			sec_plane_points.clear();
			sec_plane_points.shrink_to_fit();
		}

		first_plane_points.clear();
		first_plane_points.shrink_to_fit();
	}

	return;
}

void PlaneMerge::IdentifyConnectedPlanesFromNeighbors(bool **is_connected, const unsigned int bad_voxel_idx, \
	const unsigned int first_plane_idx, const std::vector<unsigned int> first_plane_points)
{
	BadVoxelMergeOutpuItem* bad_voxel_merge_it = &bad_voxel_merge_item[bad_voxel_idx];
	PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[bad_voxel_merge_it->bad_voxel_idx];
	float max_connected_distance = plane_seg_thresholds.THRESHOLD_DIST_OF_2VOXEL_CONNECTED;

	for (int i = 0; i < 26; i++)
	{
		NeighborItem *neighbor_it = &plane_voxel->neighbors[i];
		if (!neighbor_it->is_connected) continue;
		PlaneVoxelItem *neighbor_plane_voxel = &plane_voxel_array.voxels[neighbor_it->voxel_idx];
		if ((neighbor_plane_voxel->is_overall_merged) && (neighbor_plane_voxel->is_being_merged))// if neighbour is_overall_merged is true , it have only one plane
		{
			unsigned int parent_voxel_idx = plane_merge_element[neighbor_it->voxel_idx].parent_voxel_idx[0];
			unsigned int neighbor_plane_idx = voxel_to_plane_idx[parent_voxel_idx];
			if (neighbor_plane_idx >= total_plane_size) continue;
			if (is_connected[first_plane_idx][neighbor_plane_idx]) continue;
			if (is_connected[neighbor_plane_idx][first_plane_idx]) continue;			
			bool is_close = MathOperation::Identify2PointsGroupCLoseByDistance(pt_cloud_xyz, first_plane_points, neighbor_plane_voxel->points, max_connected_distance);
			if (is_close)
			{
				is_connected[first_plane_idx][neighbor_plane_idx] = true;
				is_connected[neighbor_plane_idx][first_plane_idx] = true;
			}
		}
		else
		{
			unsigned int neighbor_bad_voxel_idx = voxel_idx_to_bad_voxel[neighbor_it->voxel_idx];
			if (std::numeric_limits<unsigned int>::max() == neighbor_bad_voxel_idx) continue;

			BadVoxelMergeOutpuItem *neighbor_bad_voxel_merge_it = &bad_voxel_merge_item[neighbor_bad_voxel_idx];
			for (unsigned int j = 0; j < total_plane_size; j++)
			{
				unsigned int sec_plane_idx = j;
				//if (sec_plane_idx == first_plane_idx) continue; not need ?
				if (is_connected[first_plane_idx][sec_plane_idx]) continue;
				if (sec_plane_idx < total_flat_plane_size)
				{
					if (!neighbor_bad_voxel_merge_it->remaining_points_merged_plane[sec_plane_idx]) continue;
				}
				else
				{
					unsigned int points_group_plane_idx = sec_plane_idx - total_flat_plane_size;
					if (!neighbor_bad_voxel_merge_it->being_in_group_plane[points_group_plane_idx]) continue;
				}
				//if (!neighbor_bad_voxel_merge_it->remaining_points_merged_plane[sec_plane_idx]) continue;

				std::vector<unsigned int> sec_plane_points;
				sec_plane_points.clear();
				sec_plane_points.shrink_to_fit();
				for (unsigned int k = 0; k < neighbor_bad_voxel_merge_it->voxel_point_size; k++)
				{
					unsigned int sec_another_plane_idx = neighbor_bad_voxel_merge_it->closest_plane_idx[k];
					if (sec_another_plane_idx == sec_plane_idx) sec_plane_points.push_back(neighbor_plane_voxel->points.point_idx[k]);
				}

				if (sec_plane_points.size() > 2)
				{
					bool is_close = MathOperation::Identify2PointsGroupCLoseByDistance(pt_cloud_xyz, first_plane_points, sec_plane_points, max_connected_distance);
					if (is_close)
					{
						is_connected[first_plane_idx][sec_plane_idx] = true;
						is_connected[sec_plane_idx][first_plane_idx] = true;
					}

				}
			}
		}
	}
}

void PlaneMerge::IdentifyConnectedPlanes(bool**is_connected, unsigned int plane_size)
{
	if (bad_voxel_merge_item == NULL) return;  // confirm the bad voxels  are initialized

	// first check if have connected by pseudobad voxels
	for (unsigned int i = 0; i < plane_voxel_array.size; i++)
	{
		PlaneVoxelItem *plane_voxel = &plane_voxel_array.voxels[i];
		if ((plane_voxel->voxel_type != PSEUDO_BAD_VOXEL) || (!plane_voxel->is_overall_merged))  continue;
		if (!plane_voxel->is_being_merged) continue;
		for (unsigned int k = 0; k < 26; k++)
		{
			NeighborItem *neighbor_it = &plane_voxel->neighbors[k];
			if (!neighbor_it->is_connected) continue;
			PlaneVoxelItem *neighbor_plane_voxel = &plane_voxel_array.voxels[neighbor_it->voxel_idx];
			if (!neighbor_plane_voxel->is_being_merged) continue;
			if (neighbor_plane_voxel->is_overall_merged) // if neighbour is_overall_merged is true , it have only one plane
			{
				unsigned int parent_voxel_idx = plane_merge_element[i].parent_voxel_idx[0];
				unsigned int plane_idx = voxel_to_plane_idx[parent_voxel_idx];
				if (plane_idx >= plane_size) continue;
				unsigned int neighbor_parent_voxel_idx = plane_merge_element[neighbor_it->voxel_idx].parent_voxel_idx[0];
				unsigned int neighbor_plane_idx = voxel_to_plane_idx[neighbor_parent_voxel_idx];
				if (neighbor_plane_idx >= plane_size) continue;
				if (is_connected[plane_idx][neighbor_plane_idx]) continue;
				is_connected[plane_idx][neighbor_plane_idx] = true;
				is_connected[neighbor_plane_idx][plane_idx] = true;
			}
		}
	}

	// check if have connected plane in point_merged_voxel_size 
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(point_merged_voxel_size); i++)
	{
		IdentifyConnectedPlanesFromBadVoxel(is_connected, i);
	}
}


// merge the same  planes in all the planes include generated by good voxels , pseudo bad voxels or remaining voxels
bool PlaneMerge::MergePlanes(const bool no_plane_merge, PlaneMergeOutputItem *merge_planes, const unsigned int plane_size, SamePlaneGroupArray *same_plane, SameAndConnectedArray *same_connected, bool last_merge)
{
	if (plane_size <= 1)
	{
		log_debug("toatal plane size =%d, no planes merging required", plane_size);
		return true;
	}

	bool **is_same_plane = NULL; //the other planes  same plane flag  array of all planes, true show the plane is  same with the other plane
	bool **is_connected = NULL;  //the other planes  connected flag  array of all planes, true show the plane is  connected with the other plane

	plane_merge_array.planes = merge_planes;
	plane_merge_array.size = plane_size;
	total_plane_size = plane_merge_array.size;

	if (total_plane_size > max_plane_size)
	{
		log_debug("input plane_size =%d  exceed max_plane_size =%d", plane_size, max_plane_size);
	}

	log_debug("plane_size =%d max_plane_size =%d", plane_size, max_plane_size);

	same_plane_group_array.plane_size = total_plane_size;

	is_same_plane = new bool *[total_plane_size];
	is_connected = new bool *[total_plane_size];
	same_plane_group_array.same_plane_group_idx = new unsigned int[total_plane_size]; // record the same planes  group index of all planes , if -1 show have no same plane
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(total_plane_size); i++)
	{
		same_plane_group_array.same_plane_group_idx[i] = -1;
		is_same_plane[i] = new bool[total_plane_size];
		is_connected[i] = new bool[total_plane_size];
		for (unsigned int j = 0; j < total_plane_size; j++)
		{
			if (j == i)
			{
				is_same_plane[i][j] = true;
				is_connected[i][j] = true;
			}
			else
			{
				is_same_plane[i][j] = false;
				is_connected[i][j] = false;
			}
		}
	}
	IdentifySameNormalPlanes(is_same_plane, total_plane_size, last_merge);

	if (last_merge) {
		TIMING_DECLARE(TP1)
		TIMING_BEGIN(TP1)
		IdentifyConnectedPlanesBypoints(is_same_plane, is_connected);
		TIMING_END_ms("IdentifyConnectedPlanesBypoints", TP1)
	}
	else
		IdentifyConnectedPlanes(is_connected, total_plane_size);

#ifdef SAVE_OUTPUT_FILE_DEBUG
	if (debug_config_params.merge_same_plane_output_debug)
	{
		for (unsigned int i = 0; i < total_plane_size; i++)
		{
			for (unsigned int j = i + 1; j < total_plane_size; j++)
			{
				if (is_same_plane[i][j])
				{
					log_info("same plane %d plane %d  is_connected = %d", i, j, is_connected[i][j]);
				}
			}
		}
	}

	if(no_plane_merge)// (debug_config_params.no_plane_merge)
	{
		*same_plane = same_plane_group_array; 
		same_connected->is_same_plane = is_same_plane;
		same_connected->is_connected = is_connected;
		return true;
	}

#endif /*SAVE_OUTPUT_FILE_DEBUG*/

	// find the same plane by confirm both  is_same_plane and is_connected are true
	unsigned int same_plane_idx = 0;
	//#pragma omp parallel for reduction(+:same_plane_idx)
	for (int i = 0; i < static_cast<int>(total_plane_size); i++)
	{
		for (unsigned int j = i + 1; j < total_plane_size; j++)
		{
			unsigned int sameplane_idx_of_first_plane = same_plane_group_array.same_plane_group_idx[i];
			unsigned int sameplane_idx_of_second_plane = same_plane_group_array.same_plane_group_idx[j];
			if ((sameplane_idx_of_second_plane != -1) && (sameplane_idx_of_first_plane != -1)) continue; // if first and second plane have already the same plane idx , to check next plane
			if ((is_connected[i][j]) && (is_same_plane[i][j]))
			{
				//unsigned int sameplane_idx_of_first_plane = same_plane_group_array.same_plane_group_idx[i];
				if (sameplane_idx_of_first_plane != -1)     // if first plane have already the same plane idx , add second plane to same idx and then find again for this same group in all the left planes
				{
					//same_plane_group_array.same_plane_group_idx[j] = sameplane_idx_of_first_plane;
					AddPlaneToSamePlaneGroup(j, sameplane_idx_of_first_plane, total_plane_size, is_same_plane, is_connected);
					continue;
				}

				if (sameplane_idx_of_second_plane != -1)     //  if second plane have already the same plane idx ,  just add second plane to same idx and  check next plane
				{
					//same_plane_group_array.same_plane_group_idx[i] = sameplane_idx_of_second_plane;
					AddPlaneToSamePlaneGroup(i, sameplane_idx_of_second_plane,total_plane_size, is_same_plane, is_connected);
					continue;
				}

				same_plane_group_array.same_plane_group_idx[i] = same_plane_idx;
				same_plane_group_array.same_plane_group_idx[j] = same_plane_idx;
				log_info("new group%d add plane [%d,%d]", same_plane_idx,i,j);
				AddPlaneToSamePlaneGroup(i, same_plane_idx, total_plane_size, is_same_plane, is_connected);
				AddPlaneToSamePlaneGroup(j, same_plane_idx,total_plane_size, is_same_plane, is_connected);
				// if it is the new group , must find again in all the other plane to add the same group plane
				same_plane_idx++;

			}
		}
	}

	*same_plane = same_plane_group_array;
	same_connected->is_same_plane = is_same_plane;
	same_connected->is_connected = is_connected;

	return true;
}


