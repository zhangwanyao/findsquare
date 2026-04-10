/* Copyright (c) ASTRI, 2019. All rights reserved.
 * This software is proprietary to and embodies the confidential technology
 * of Hong Kong Applied Science and Technology Research Institute Company
 * Limited (ASTRI).
 *
 * Possession, use, or copying of this software and media is authorized
 * only pursuant to a valid written license from ASTRI or an authorized
 * sublicensor.
 *
 * Author: Elaine Li, mounty
 * Date: 20190812
 * Description: Plane Segmentation Class Definitions
 */

#include "PlaneSegmentation.h"
#include "MathOperation.hpp"
#include "GeometryFilter.h"
#include "log.h"
#include "math.h"
#include <iomanip>
#include <fstream>
#include <iostream>
#include "InOutData.h"
#include "ConfigParse.hpp"
//#include "planeConfigParse.h"
#include "plane_cfg.hpp"
#include "util_math.hpp"
#include "in_out_data.hpp"
#include "util_time.hpp"
//#include "detc_edge_api.h"
#include "util_plane_seg.hpp"
using namespace Util_Math;
using namespace ModuleStruct;

#define VOXEL_NEAREST_APPROACH_MERGE

PlaneSegmentation::PlaneSegmentation() {
	this->Reset();
};

PlaneSegmentation::~PlaneSegmentation() {
	FreeMemory();
};

void PlaneSegmentation::Reset(){
	plane_voxel_array.size = 0;
	plane_voxel_array.voxels = NULL;
	plane_merge_element = NULL;
	plane_merge_out.size = 0;
	plane_merge_out.planes = NULL;
	plane_seg_out.size = 0;
	plane_seg_out.planes = NULL;
	good_voxel_size = 0;
	pseudo_bad_voxel_size = 0;
	point_merged_voxel_size = 0;
	bad_voxel_size = 0;
	good_voxel_plane_size = 0;
	pseudobad_voxel_plane_size = 0;
	grid_to_occupied = NULL;
	grid_to_occupied_size = 0;
	point_to_voxel_idx_nm = NULL;
	//is_good_voxel_by_mse = NULL;
	voxel_idx_to_bad_voxel = NULL;
	voxel_to_plane_idx = NULL;
	same_plane_group_array.plane_size = 0;
	same_plane_group_array.same_plane_group_idx = NULL;
	points_group_plane_merge_out.size = 0;
	points_group_plane_merge_out.planes = NULL;
	points_group_remaning_planes.size = 0;
	points_group_remaning_planes.planes = NULL;
	//parent_group_to_plane_idx = NULL;
	bad_voxel_merge_item = NULL;

	sys_control_para.is_cad = false;
	sys_control_para.scanner_type = LEICA_TYPE_0;
	sys_control_para.encryption_type = 0;

	config_params.identify_with_bridge = false;
	config_params.filtering_strtgy_type = NO_FILTER;
	config_params.seg_thrshld_section_valid = SEG_DEFAULT_CONFIG_PARAM;

	debug_config_params.dbg_section_valid = false;
	debug_config_params.reserved_test = 0;
	debug_config_params.missing_point_output = false;
	debug_config_params.io_debug_info = false;
	debug_config_params.voxel_info_debug = false;
	debug_config_params.neighbour_info_debug = false;
	debug_config_params.no_plane_merge = false;
	debug_config_params.plane_output_debug = false;
	debug_config_params.merge_pseudobad_voxel_debug = false;
	debug_config_params.merge_bad_voxel_debug = false;
	debug_config_params.pseudo_bad_lost_plane_output_debug = false;
	debug_config_params.is_dwnsmpl_output = false;
	debug_config_params.points_group_neighbor_output_debug = false;
	debug_config_params.merge_same_plane_output_debug = false;
	debug_config_params.include_get_nm_by_file = false;
	debug_config_params.nm_file_ouput_debug = false;

	this->Init();
	FreeMemory();
	return;
}

void PlaneSegmentation::FreeMemory() {

	// free memory of all PlaneVoxelItem in PlaneVoxelArray
	log_info("planeSegmentation free begin");
	if (plane_voxel_array.voxels != NULL) {

		for (unsigned int i = 0; i < plane_voxel_array.size; i++) {

			// free memory of storing all point indexes in each occupied voxels
			if (plane_voxel_array.voxels[i].points.point_idx != NULL) {

				delete[] plane_voxel_array.voxels[i].points.point_idx;
				plane_voxel_array.voxels[i].points.point_idx = NULL;
				plane_voxel_array.voxels[i].points.size = 0;
			}

			if (plane_voxel_array.voxels[i].is_same_with_ref_plane != NULL) {

				delete[] plane_voxel_array.voxels[i].is_same_with_ref_plane;
				plane_voxel_array.voxels[i].is_same_with_ref_plane = NULL;
			}
		}

		delete[] plane_voxel_array.voxels;
		plane_voxel_array.voxels = NULL;
		plane_voxel_array.size = 0;
	}

	// free memory of all PlaneMergeItem
	if (plane_merge_element != NULL) {

		//for (unsigned int i = 0; i < plane_voxel_array.size; i++) {

		//	// free memory of storing bridge voxel infos
		//	if (plane_merge_element[i].bridge_voxel_elements.parent_voxel_idxs != NULL) {

		//		delete[] plane_merge_element[i].bridge_voxel_elements.parent_voxel_idxs;
		//		plane_merge_element[i].bridge_voxel_elements.parent_voxel_idxs = NULL;
		//		plane_merge_element[i].bridge_voxel_elements.num_of_planes = 0;
		//	}
		//}

		delete[] plane_merge_element;
		plane_merge_element = NULL;
	}


	if (plane_merge_out.planes != NULL) {

		for (unsigned int i = 0; i < plane_merge_out.size; i++) {

			PlaneMergeOutputItem *plane_it = &plane_merge_out.planes[i];
			// free memory of storing all voxel indexes in each plane merge output
			if (plane_it->voxels.voxel_idx != NULL) {

				delete[] plane_it->voxels.voxel_idx;
				plane_it->voxels.voxel_idx = NULL;
				plane_it->voxels.size = 0;
			}
			// free memory of storing all point indexes in each plane merge output
			if (plane_it->extended_part_points.point_idx != NULL) {

				delete[] plane_it->extended_part_points.point_idx;
				plane_it->extended_part_points.point_idx = NULL;
				plane_it->extended_part_points.size = 0;
			}

			// free memory of storing all point indexes in each plane merge output
			if (plane_it->points.point_idx != NULL) {

				delete[] plane_it->points.point_idx;
				plane_it->points.point_idx = NULL;
				plane_it->points.size = 0;
			}
			// free memory of storing all point indexes in each plane merge output
			if (plane_it->multiplane_points.point_idx != NULL) {

				delete[] plane_it->multiplane_points.point_idx;
				plane_it->multiplane_points.point_idx = NULL;
				plane_it->multiplane_points.size = 0;
			}
		}

		delete[] plane_merge_out.planes;
		plane_merge_out.planes = NULL;
		plane_merge_out.size = 0;
	}

	if (points_group_plane_merge_out.planes != NULL) {

		for (unsigned int i = 0; i < points_group_plane_merge_out.size; i++) {

			// free memory of storing all point indexes in each plane merge output
			if (points_group_plane_merge_out.planes[i].points.point_idx != NULL) {

				delete[] points_group_plane_merge_out.planes[i].points.point_idx;
				points_group_plane_merge_out.planes[i].points.point_idx = NULL;
				points_group_plane_merge_out.planes[i].points.size = 0;
			}

			if (points_group_plane_merge_out.planes[i].voxels.voxel_idx != NULL) {

				delete[] points_group_plane_merge_out.planes[i].voxels.voxel_idx;
				points_group_plane_merge_out.planes[i].voxels.voxel_idx = NULL;
				points_group_plane_merge_out.planes[i].voxels.size = 0;
			}

			if (points_group_plane_merge_out.planes[i].extended_part_points.point_idx != NULL) {

				delete[] points_group_plane_merge_out.planes[i].extended_part_points.point_idx;
				points_group_plane_merge_out.planes[i].extended_part_points.point_idx = NULL;
				points_group_plane_merge_out.planes[i].extended_part_points.size = 0;
			}

			// free memory of storing all point indexes in each plane merge output
			if (points_group_plane_merge_out.planes[i].multiplane_points.point_idx != NULL) {

				delete[] points_group_plane_merge_out.planes[i].multiplane_points.point_idx;
				points_group_plane_merge_out.planes[i].multiplane_points.point_idx = NULL;
				points_group_plane_merge_out.planes[i].multiplane_points.size = 0;
			}
		}

		delete[] points_group_plane_merge_out.planes;
		points_group_plane_merge_out.planes = NULL;
		points_group_plane_merge_out.size = 0;
	}


	// points_group_remaning_planes. planes's voxels, points and bad_group_points have already deleted in points_group_plane_merge_out ,so here only need delete planes
	if (points_group_remaning_planes.planes != NULL) 
	{
			delete[] points_group_remaning_planes.planes;
			points_group_remaning_planes.planes = NULL;
	}

	if (same_plane_group_array.same_plane_group_idx != NULL)
	{
		delete[] same_plane_group_array.same_plane_group_idx;
		same_plane_group_array.same_plane_group_idx = NULL;
	}


	// free memory of all PlaneItem in PlaneSegmentationOutput
#if 0
	if (plane_seg_out.planes != NULL) {

		for (unsigned int i = 0; i < plane_seg_out.size; i++) {
	 
			// free memory of storing all point indexes in each planes
			if (plane_seg_out.planes[i].points.point_idx != NULL) {

				delete[] plane_seg_out.planes[i].points.point_idx;
				plane_seg_out.planes[i].points.point_idx = NULL;
				plane_seg_out.planes[i].points.size = 0;
			}
	 	}

		delete[] plane_seg_out.planes;
		plane_seg_out.planes = NULL;
		plane_seg_out.size = 0;
	}
#endif
	if (voxel_to_plane_idx != NULL) {
		free(voxel_to_plane_idx);
		voxel_to_plane_idx = NULL;
	}


	if (grid_to_occupied != NULL) {
		delete [] grid_to_occupied;
		grid_to_occupied = NULL;
	}

	if (voxel_idx_to_bad_voxel != NULL) {
		delete[] voxel_idx_to_bad_voxel;
		voxel_idx_to_bad_voxel = NULL;
	}

	/*if (is_good_voxel_by_mse != NULL) {
		delete[] is_good_voxel_by_mse;
		is_good_voxel_by_mse = NULL;
	}*/

#ifdef INCLUDE_GET_NM_BY_FILE
	if (point_to_voxel_idx_nm != NULL) {
		delete [] point_to_voxel_idx_nm;
		point_to_voxel_idx_nm = NULL;
	}
#endif
	log_info("planeSegmentation free end");
	return;
}

void PlaneSegmentation::VerifyPlaneSegParameters() {

	if (plane_seg_params.voxel_params.length_x_of_voxel <= 0.0f ||
		plane_seg_params.voxel_params.length_y_of_voxel <= 0.0f ||
		plane_seg_params.voxel_params.length_z_of_voxel <= 0.0f ||
		plane_seg_params.plane_seg_thresholds.THRESHOLD_MIN_PLANE_DIST_OF_TWO_VOXEL < 0.0f ||
		plane_seg_params.plane_seg_thresholds.THRESHOLD_MIN_DIST_OF_POINT2VOXEL < 0.0f ||
		plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_NORMAL_ANGLE_OF_TWO_VOXEL < 0.0f ||
		plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_NORMAL_ANGLE_OF_TWO_VOXEL > 90.0f ||
		plane_seg_params.plane_seg_thresholds.THRESHOLD_MIN_POINT_NUM_OF_PLANE <= 5)
		// ||plane_seg_params.plane_classify_thresholds.THRESHOLD_MIN_AREA_OF_A_VALID_PLANE <= 0.0f ||
		//plane_seg_params.plane_classify_thresholds.THRESHOLD_HEIGHT_DIFF_CONNECTED_TO_GROUND < 0.0f) 
	{
		log_warn("wrong parmeters");
	}

	return;
}

void PlaneSegmentation::InitializeParams(const NormalEstimationInterface inputStruct)
{
	// Set values to parent class VoxelBaseClass
	point_density = inputStruct.density;
	pt_cloud_xyz = inputStruct.points;
	log_info("pt_cloud_xyz size =%d", pt_cloud_xyz.size);
	pt_cloud_normal = inputStruct.normals;
	//if (config_params.is_voxel_size_detected)
	{
		plane_seg_params.voxel_params.length_x_of_voxel = inputStruct.voxel_para.length_x_of_voxel;
		plane_seg_params.voxel_params.length_y_of_voxel = inputStruct.voxel_para.length_y_of_voxel;
		plane_seg_params.voxel_params.length_z_of_voxel = inputStruct.voxel_para.length_z_of_voxel;
	}

	length_x_of_voxel = plane_seg_params.voxel_params.length_x_of_voxel;
	length_y_of_voxel = plane_seg_params.voxel_params.length_y_of_voxel;
	length_z_of_voxel = plane_seg_params.voxel_params.length_z_of_voxel;
	length_x_of_voxel_inverse = 1 / length_x_of_voxel;
	length_y_of_voxel_inverse = 1 / length_y_of_voxel;
	length_z_of_voxel_inverse = 1 / length_z_of_voxel;
	num_of_occupied_voxel = inputStruct.voxel_array_size;
	grid_to_occupied = inputStruct.grid_to_occupied_voxel_idx;
	grid_to_occupied_size = inputStruct.grid_to_occupied_voxel_idx_size;
	min_high_mse_ratio = plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_HIGH_MSE_RATIO;
	max_mse_of_mini_plane = plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_PLANE;
	if (sys_control_para.scanner_type == UNRE_TYPE_0)
	{
		max_mse_of_mini_plane = plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_PLANE / 2;
	}

	log_debug("length_x_of_voxel=%f num_of_occupied_voxel=%d", length_x_of_voxel, num_of_occupied_voxel);
	return;
}

bool PlaneSegmentation::CheckInputParameters(const PointArray point_array, NormalEstimationInterface *inputStruct) {

	// Check the input point array size
	if (point_array.size == 0) {

		log_error("Empty Input Point Array");
		return false;
	}

	// Check the input point array pointer
	if (point_array.points == NULL) {

		log_error("Input Point Array Pointer is NULL");
		return false;
	}

	// Check the input voxel array size
	if (inputStruct->voxel_array_size == 0) {

		log_error("Empty Input Voxel Array");
		return false;
	}

	// Check the input voxel array pointer
	if (inputStruct->voxel_array == NULL) {

		log_error("Input Voxel Array Pointer is NULL");
		return false;
	}

	// Check the input grid index to occupied voxel index array size
	if (inputStruct->grid_to_occupied_voxel_idx_size == 0) {

		log_error("Empty Input Grid Index to Occupied Voxel Index Array");
		return false;
	}

	// Check the input grid index to occupied voxel index array pointer
	if (inputStruct->grid_to_occupied_voxel_idx == NULL) {

		log_error("Input Grid Index to Occupied Voxel Index Array Pointer is NULL");
		return false;
	}

	return true;
}

// Create voxel and assign its data for all occupied voxels
void PlaneSegmentation::CreateVoxels(VoxelItem* &voxel_array) {

	 plane_voxel_array.size = num_of_occupied_voxel;

	 log_debug("num_of_occupied_voxel =%d", num_of_occupied_voxel);
	 plane_voxel_array.voxels = new PlaneVoxelItem[plane_voxel_array.size];
	
	 PlaneFitThresholds* plane_fit_thresholds = &plane_seg_params.plane_seg_thresholds;
	 float max_normal_diff = static_cast<float>(std::cos(plane_fit_thresholds->THRESHOLD_MAX_NORMAL_ANGLE_OF_TWO_VOXEL * M_PI / 180.0));
	 float max_close_dist_2voxel = plane_fit_thresholds->THRESHOLD_DIST_OF_2VOXEL_CONNECTED;
	//TIMING_DECLARE(TP1)
	// current voxel grid compute
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(plane_voxel_array.size); i++)
	{
		PlaneVoxelItem *plane_voxel = &plane_voxel_array.voxels[i];
		VoxelMergeBaseClass occupied_voxel_grid(plane_voxel, plane_fit_thresholds);
		
		PointInVoxelArray *points_voxel_array = &voxel_array[i].points;

		//voxel grid init 
		occupied_voxel_grid.ResetVoxel();

		// Assign grid index
		plane_voxel->grid_idx = voxel_array[i].grid_idx;

		//voxel grid point array input
		occupied_voxel_grid.InputVoxelPoints(points_voxel_array);

		//voxel grid sums 
		occupied_voxel_grid.GetVoxelSums(&pt_cloud_xyz);

		//compute voxel center, normal, mse.
		//occupied_voxel_grid.Compute(plane_voxel->points.size);

		plane_voxel->is_good_normal = voxel_array[i].is_good_voxel;
	}

#pragma omp parallel for
	 for (int i = 0; i < static_cast<int>(plane_voxel_array.size); i++)
	 {
		 PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[i];

		 VoxelMergeBaseClass occupied_voxel_grid(plane_voxel, plane_fit_thresholds);
		 unsigned int point_index = voxel_array[i].points.point_idx[0];
		 Point3f point = pt_cloud_xyz.points[point_index];
		 int col_idx, row_idx, depth_idx;
		 ConvertVoxelIDTo3dID(point, row_idx, col_idx, depth_idx);
		 for (int k = 0; k < 26; k++)
		 {
			 if (depth_idx + movement_z[k] >= depths_of_voxel || depth_idx + movement_z[k] < 0)
				 continue;
			 if (row_idx + movement_y[k] >= rows_of_voxel || row_idx + movement_y[k] < 0)
				 continue;
			 if (col_idx + movement_x[k] >= cols_of_voxel || col_idx + movement_x[k] < 0)
				 continue;

			 long long neighbor_voxel_id = (long long)((depth_idx + movement_z[k]) * (cols_of_voxel * rows_of_voxel) + (row_idx + movement_y[k]) * cols_of_voxel + (col_idx + movement_x[k]));

			 if (neighbor_voxel_id < 0 || neighbor_voxel_id >= static_cast<long long>(total_num_of_voxel_grid)) continue;

			 //check if this voxel is initialized
			 if (grid_to_occupied[neighbor_voxel_id] == std::numeric_limits<unsigned int>::max()) continue;

			 /* grid_to_occupied_voxel_idx is the output of NormalEstimationInterface */
			 unsigned int neighbour_voxel_index = grid_to_occupied[neighbor_voxel_id];
			 NeighborItem* neighbour_item = &plane_voxel->neighbors[k];
			 neighbour_item->is_occupied = true;
			 neighbour_item->voxel_idx = neighbour_voxel_index;
			// PointInVoxelArray neighbor_points = plane_voxel_array.voxels[neighbour_item->voxel_idx].points;
			// neighbour_item->is_close = MathOperation::Identify2PointsGroupCLoseByDistance(pt_cloud_xyz, plane_voxel->points, neighbor_points, max_close_dist_2voxel);
		 }
	 }

	GetVoxelNomalByMse();

	/*float min_number_of_2close_voxel = 200;
	if (sys_control_para.scanner_type == UNRE_TYPE_0)
	{
		if (config_params.downsample_type == ALL_DOWNSAMPLE)
		{
			min_number_of_2close_voxel = 20;
		}
	}*/
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(plane_voxel_array.size); i++)
	{	
	   // must compute mse again according to new normal
		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[i];
		GetVoxelDistMse(i, plane_voxel->plane_mse);

		//check is_flat flag for each voxel
		plane_voxel->is_flat = (plane_voxel->plane_mse < plane_fit_thresholds->THRESHOLD_MAX_MSE_OF_VOXEL) && \
			(plane_voxel->plane_high_mse_ratio < min_high_mse_ratio);
		//if (!sys_control_para.is_cad) 
		//	plane_voxel->is_tiny = plane_voxel->is_flat&&(plane_voxel->points.size >= plane_fit_thresholds->THRESHOLD_MIN_POINT_NUM_OF_FLAT_VOXEL);
		plane_voxel->is_tiny = (plane_voxel->points.size < plane_fit_thresholds->THRESHOLD_MIN_POINT_NUM_OF_FLAT_VOXEL);
	   for (int k = 0; k < 26; k++)
	   {
		   NeighborItem* neighbour_item = &plane_voxel->neighbors[k];
		   //Identify2VoxelCLoseByDistance  must call after voxel center being compute 
		   if (!neighbour_item->is_occupied) continue;
		   PlaneVoxelItem* neighbor_voxel = &plane_voxel_array.voxels[neighbour_item->voxel_idx];

		 /*  if ((plane_voxel->points.size >= min_number_of_2close_voxel) && (neighbor_voxel->points.size >= min_number_of_2close_voxel))
		   {
			   neighbour_item->is_connected = true;
		   }
		   else*/
		   {
			   neighbour_item->is_connected = Identify2VoxelCLoseByDistance(i, neighbour_item->voxel_idx, max_close_dist_2voxel);
		   }
	   }
	}
   

	//get neighbour item of current voxel
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(plane_voxel_array.size); i++)
	{
		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[i];
		VoxelMergeBaseClass occupied_voxel_grid(plane_voxel, plane_fit_thresholds);
		if (!plane_voxel->is_flat) continue;
		if (plane_voxel->is_tiny) continue;

		for (int k = 0; k < 26; k++)
		{
			NeighborItem* neighbour_item = &plane_voxel->neighbors[k];
			if (!neighbour_item->is_connected) continue;
			PlaneVoxelItem* neighbor_voxel = &plane_voxel_array.voxels[neighbour_item->voxel_idx];

			//if (neighbor_voxel->points.size < plane_fit_thresholds->THRESHOLD_MIN_POINT_NUM_OF_FLAT_VOXEL) continue;
			if (!neighbor_voxel->is_flat) continue;
			if (neighbor_voxel->is_tiny) continue;
			neighbour_item->normal_diff = std::fabs(ComputeVectorDotProduct<float>(neighbor_voxel->plane_normal, plane_voxel->plane_normal));

			if (neighbour_item->normal_diff < max_normal_diff) continue;
			neighbour_item->plane_dist = ComputePointToPlaneDist<float>(neighbor_voxel->plane_center, plane_voxel->plane_normal, plane_voxel->plane_center);
			if (neighbour_item->plane_dist <= plane_fit_thresholds->THRESHOLD_MIN_PLANE_DIST_OF_TWO_VOXEL)
			{
				neighbour_item->neighbor_flag = true;
			}
		}
	}  

	//compute voxel average normal,mse and high_mse_ratio if it is flat,  these three parameters will be used in some scenarios such as plane parent_voxel noral etc.
	ComputeVoxelAvgNormalMse();

	plane_merge_element = new PlaneMergeItem[plane_voxel_array.size];
	int tmp_bad_voxel_cnt = 0;
	int tmp_pseudobad_voxel_cnt = 0;
	int tmp_good_voxel_cnt = 0;
#pragma omp parallel for reduction (+:tmp_bad_voxel_cnt,tmp_pseudobad_voxel_cnt,tmp_good_voxel_cnt)
	for (int i = 0; i < static_cast<int>(plane_voxel_array.size); i++)
	{
		int bad_neighbour_count = 0;          //number of neigbours whose normal difference and plane distance exceed threshold 
		int neighbor_flag_count = 0;          //number of neigbours whose plane distance and normal difference are less than thresholds

#ifdef USE_AVG_NORMAL_DIFF
		float smallest_normal_diff = 0;
		float smallest_plane_dist = 0;
#else
		float smallest_normal_diff = std::numeric_limits<float>::infinity();
		float smallest_plane_dist = std::numeric_limits<float>::infinity();
#endif
		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[i];
		PlaneMergeItem* plane_merge_it = &plane_merge_element[i];
		// init  parent_voxel_idx smallest_normal_diff and smallest_plane_dist of the plane_merge_element
		plane_merge_it->parent_voxel_idx[0] = plane_merge_it->parent_voxel_idx[1] = i;
		plane_merge_it->good_neighbor_count = 0;
		plane_merge_it->bad_neighbor_count = 0;
		plane_merge_it->smallest_normal_diff = smallest_normal_diff;
		plane_merge_it->smallest_plane_dist = smallest_plane_dist;

		//plane_merge_it->is_bridge = false;
		//plane_merge_it->is_single_plane_bridge = false;
		//plane_merge_it->bridge_voxel_elements.num_of_planes = 0;
		//plane_merge_it->bridge_voxel_elements.parent_voxel_idxs = NULL;
		//plane_merge_it->center_dist = 1/(plane_voxel->grid_idx+1.0f);  // temprary compute
		double delta_x = plane_voxel->plane_center.x - (max_x - min_x) / 2;
		double delta_y = plane_voxel->plane_center.y - (max_y - min_y) / 2;
		double delta_z = plane_voxel->plane_center.z - (max_z - min_z) / 2;
		//plane_merge_it->center_dist = std::sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z);

		/*if ((plane_voxel->plane_mse >= plane_fit_thresholds->THRESHOLD_MAX_MSE_OF_VOXEL)|| \
			(plane_voxel->plane_high_mse_ratio>min_high_mse_ratio)||\
			((!sys_control_para.is_cad)&&(plane_voxel->points.size < min_points_num_of_flat_voxel)))*/
		if(!plane_voxel->is_flat)
		{
			tmp_bad_voxel_cnt++;// point_merged_voxel_size++;
			plane_merge_it->good_neighbor_count = 0;
			for (int j = 0; j < 26; j++)
			{
				NeighborItem* neighbor_it = &plane_voxel->neighbors[j];
				PlaneVoxelItem* neighbor_voxel = &plane_voxel_array.voxels[neighbor_it->voxel_idx];
				if (!neighbor_it->is_connected) continue;
				if (!neighbor_voxel->is_tiny) bad_neighbour_count++;
			}
			plane_merge_it->bad_neighbor_count = bad_neighbour_count;
			continue;
		}

		for (int k = 0; k < 26; k++)
		{
			NeighborItem* neighbour_item = &plane_voxel->neighbors[k];

			if (!neighbour_item->is_connected) continue;
			PlaneVoxelItem* neighbor_voxel = &plane_voxel_array.voxels[neighbour_item->voxel_idx];

			if (neighbour_item->neighbor_flag)
			{
				neighbor_flag_count++;
			}
			else
			{
				if(!neighbor_voxel->is_tiny)
				bad_neighbour_count++;
			}

			// init  parent_voxel_idx smallest_normal_diff and smallest_plane_dist of the plane_merge_element

			if (neighbor_voxel->is_tiny) continue;
#ifdef USE_AVG_NORMAL_DIFF
			smallest_normal_diff += neighbour_item->normal_diff;
			smallest_plane_dist += neighbour_item->plane_dist;
#else
			if (neighbour_item->normal_diff < smallest_normal_diff)  smallest_normal_diff = neighbour_item->normal_diff;
			if (neighbour_item->plane_dist < smallest_plane_dist)  smallest_plane_dist = neighbour_item->plane_dist;
#endif
		}


		plane_merge_it->good_neighbor_count = neighbor_flag_count;
		plane_merge_it->bad_neighbor_count = bad_neighbour_count;


#ifdef USE_AVG_NORMAL_DIFF
		if ((neighbor_flag_count + bad_neighbour_count) != 0)
		{
			smallest_normal_diff /= neighbor_flag_count + bad_neighbour_count;
			smallest_plane_dist /= neighbor_flag_count + bad_neighbour_count;
		}
#endif

		/*check if it is good voxel or pseudo bad voxels for plane merge in all occpupied voxels

		Criteria of "Check if voxel is Pseudo bad voxel or good voxels ":
		Check if plane mse is less than the predefined threshold.
		Check if  the  normal difference between the neighbour and this voxel is less than threshold.
		Check if the distance between fitted plane and this voxel is less than the predefined threshold;
		Pseudo bad voxel or good voxel is "1)" and "2)" and "3)"  otherwise is  bad voxels

		Criteria of "Check if voxel is good voxel ":
		In all the non-bad voxels
		1) Check if number of neighbour voxels whose normal difference with current voxel exeed predefined threshold <=3
		2) Check if number of neighbour voxels whose plane distance with current voxel exeed predefined threshold <=3
		good voxel  for plane merge is "1)" and "2)"   */

		if (neighbor_flag_count!=0)
		{
            if ((bad_neighbour_count >= 3)|| (neighbor_flag_count==1))
			{
				plane_voxel->voxel_type = PSEUDO_BAD_VOXEL;
				plane_voxel->is_overall_merged = true;
				//pseudo_bad_voxel_size++;
				tmp_pseudobad_voxel_cnt++;
			}
			else
			{
				plane_voxel->voxel_type = GOOD_VOXEL;
				//plane_voxel->is_being_merged = true;
				plane_voxel->is_overall_merged = true;
				//good_voxel_size++;
				tmp_good_voxel_cnt++;
			}
		}
		else
		{
			//point_merged_voxel_size++;
			tmp_bad_voxel_cnt++;
		}

		plane_merge_it->smallest_normal_diff = smallest_normal_diff;
		plane_merge_it->smallest_plane_dist = smallest_plane_dist;
	}

	bad_voxel_size += tmp_bad_voxel_cnt;
	good_voxel_size = tmp_good_voxel_cnt;
	pseudo_bad_voxel_size = tmp_pseudobad_voxel_cnt;	// Identify the voxel is plane bridge voxel

#if 0
	unsigned int max_bridge_plane_cnt = 0;// max planes in all the bridge voxels
	int bridge_cnt = 0; //number of all the bridge voxels
	int single_bridge_cnt = 0; //number of all the bridge voxels who have only one single plane
	int good_bridge_cnt = 0; //number of all the bridge good voxels
	int pseudobad_bridge_cnt = 0; //number of all the bridge pesudobad voxels
	int realbad_bridge_cnt = 0; //number of all the bridge real bad voxels
   #pragma omp parallel for reduction(+:bridge_cnt,single_bridge_cnt,good_bridge_cnt,pseudobad_bridge_cnt,realbad_bridge_cnt)
	for (int i = 0; i < static_cast<int>(plane_voxel_array.size); i++)
	{
		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[i];
		PlaneMergeItem* plane_merge_it = &plane_merge_element[i];

		float normal_diff = std::numeric_limits<float>::infinity();
		float plane_dist = std::numeric_limits<float>::infinity();

		unsigned int bridge_plane_cnt = 0;  // if this voxel is a bridge, record the planes number of this bridge
		for (int j = 0; j < 26;j++)
		{
			NeighborItem* first_neighbour_item = &plane_voxel->neighbors[j];

			if (!first_neighbour_item->is_connected) continue;
			if (first_neighbour_item->neighbor_flag) continue; //find in non good neighbour

			PlaneVoxelType first_neighbour_voxeltype = plane_voxel_array.voxels[first_neighbour_item->voxel_idx].voxel_type;

			if (first_neighbour_voxeltype == REAL_BAD_VOXEL) continue;// the two neighbour must be good or pesudo bad voxel


     		// if first neighbour has already in a bridge plane, to the next neighbour
			if (first_neighbour_item->plane_idx_of_bridge < 13) continue;

			for (int k = j + 1; k < 26;k++)
			{
				NeighborItem* second_neighbour_item = &plane_voxel->neighbors[k];
				if (!second_neighbour_item->is_connected) continue;

				if (second_neighbour_item->neighbor_flag) continue; //find in non good neighbour

				PlaneVoxelType second_neighbour_voxeltype = plane_voxel_array.voxels[second_neighbour_item->voxel_idx].voxel_type;
				if (second_neighbour_voxeltype == REAL_BAD_VOXEL) continue;

				VoxelMergeBaseClass occupied_voxel_grid(&plane_voxel_array.voxels[first_neighbour_item->voxel_idx], plane_fit_thresholds);

				Point3f plane_normal = plane_voxel_array.voxels[second_neighbour_item->voxel_idx].plane_normal;
				Point3f plane_center = plane_voxel_array.voxels[second_neighbour_item->voxel_idx].plane_center;
#ifdef COMPARE_NORMAL_DIFF_IN_ANGLE
				float normal_sim = std::fabs(occupied_voxel_grid.ComputeNormalsSimilarity(plane_normal));
				if (normal_sim > 1.0f) normal_sim = 1.0f;
				normal_diff = (float)((std::acos(normal_sim) * 180.0) / M_PI);
				//normal_diff = (std::acos(std::fabs(occupied_voxel_grid.ComputeNormalsSimilarity(plane_normal))) * 180.0) / M_PI;
#else
				normal_diff = std::fabs(occupied_voxel_grid.ComputeNormalsSimilarity(plane_normal));
#endif

				//compute neighbour item plane distance 
				plane_dist = occupied_voxel_grid.GetPointPlaneDist(plane_center);
#ifdef COMPARE_NORMAL_DIFF_IN_ANGLE				
				if ((normal_diff <= (plane_fit_thresholds->THRESHOLD_MAX_NORMAL_ANGLE_OF_TWO_VOXEL)) && (plane_dist <= (plane_fit_thresholds->THRESHOLD_MIN_PLANE_DIST_OF_TWO_VOXEL)))
#else
				if ((normal_diff > max_normal_diff) && (plane_dist <= plane_fit_thresholds->THRESHOLD_MIN_PLANE_DIST_OF_TWO_VOXEL))
#endif
				{
					
					int neighbour_flag_cnt = 0;// check if the second neighbour is one of the good neighbours of first neighbour

					int the_another_neighbour_cnt = 0;// check if second neighbour is already connectd  with  first neighbour by a good or pesudo bad  voxel

					PlaneVoxelItem * first_neighbour_voxel = &plane_voxel_array.voxels[first_neighbour_item->voxel_idx];
					PlaneVoxelItem * second_neighbour_voxel = &plane_voxel_array.voxels[second_neighbour_item->voxel_idx];
					for (int m = 0; m < 26; m++)
					{

						NeighborItem* neighbour_of_first_neighbour_item = &first_neighbour_voxel->neighbors[m];
						//if (!neighbour_of_first_neighbour_item->is_occupied) continue;
						if (!neighbour_of_first_neighbour_item->neighbor_flag) continue;  // find in the good neighbours
						for (int n = 0; n < 26; n++)
						{
							NeighborItem* neighbour_of_second_neighbour_item = &second_neighbour_voxel->neighbors[n];
							//if (!neighbour_of_second_neighbour_item->is_occupied) continue;
							if (!neighbour_of_second_neighbour_item->neighbor_flag) continue; // find in the good neighbours
							if (neighbour_of_first_neighbour_item->voxel_idx != neighbour_of_second_neighbour_item->voxel_idx) continue;
							//if (neighbour_of_first_neighbour_item->voxel_idx == i)  continue;
							the_another_neighbour_cnt++;
						}
						if (neighbour_of_first_neighbour_item->voxel_idx != second_neighbour_item->voxel_idx) continue;
						neighbour_flag_cnt++;						
					}
					// if two voxels is already have neighbour flag , go to next pair voxels
					if (neighbour_flag_cnt != 0) continue;
					
					// if two voxels is already connected by other voxels (not seperated) , go to next pair voxels
					if (the_another_neighbour_cnt != 0) continue;				

					// if second neighbour has already in a bridge plane ,to the next
					if (second_neighbour_item->plane_idx_of_bridge < 13)
					{
						first_neighbour_item->plane_idx_of_bridge = second_neighbour_item->plane_idx_of_bridge;
						unsigned int first_neighbor_bridge_idx = -1;
						for (int m = 0; m < 26; m++)
						{
							NeighborItem *neighbour_of_first_neighbour = &first_neighbour_voxel->neighbors[m];
							if (!neighbour_of_first_neighbour->is_connected) continue;

							if (neighbour_of_first_neighbour->voxel_idx == i)
							{
								first_neighbor_bridge_idx = m;
								break;
							}
						}
						//impossible things , maybe redundant, delete this after debug
						if (first_neighbor_bridge_idx == -1)
						{
							log_info("bridge voxel %d add second index =%d  \n", i, first_neighbor_bridge_idx);
							continue;
						}
						first_neighbour_voxel->neighbors[first_neighbor_bridge_idx].bridge_plane_idx_of_plane_pair = first_neighbour_item->plane_idx_of_bridge;
						continue;

					}

					// if first neighbour has already in a bridge plane ,just add it into the second neighbour
					if (first_neighbour_item->plane_idx_of_bridge < 13)
					{
						second_neighbour_item->plane_idx_of_bridge = first_neighbour_item->plane_idx_of_bridge;
						unsigned int second_neighbor_bridge_idx = -1;
						for (int m = 0; m < 26; m++)
						{
							NeighborItem *neighbour_of_second_neighbour = &second_neighbour_voxel->neighbors[m];
							if (!neighbour_of_second_neighbour->is_connected) continue;

							if (neighbour_of_second_neighbour->voxel_idx == i)
							{
								second_neighbor_bridge_idx = m;
								break;
							}
						}
						//impossible things , maybe redundant, delete this after debug
						if (second_neighbor_bridge_idx == -1) 
						{
							log_info("bridge voxel %d add second index =%d  \n", i, second_neighbor_bridge_idx);
							continue;
						}
						second_neighbour_voxel->neighbors[second_neighbor_bridge_idx].bridge_plane_idx_of_plane_pair = second_neighbour_item->plane_idx_of_bridge;
						continue;
					}

					// get current bridge voxel neighbour direction in its two plane pair voxel
					unsigned int first_neighbor_bridge_idx = -1;
					unsigned int second_neighbor_bridge_idx = -1;
					for (int m = 0; m < 26; m++)
					{
						NeighborItem *neighbour_of_first_neighbour = &first_neighbour_voxel->neighbors[m];
						if (!neighbour_of_first_neighbour->is_connected) continue;
						if (neighbour_of_first_neighbour->voxel_idx == i)
						{
							first_neighbor_bridge_idx = m;
							break;
						}
					}

					for (int m = 0; m < 26; m++)
					{
						NeighborItem *neighbour_of_second_neighbour = &second_neighbour_voxel->neighbors[m];
						if (!neighbour_of_second_neighbour->is_connected) continue;
						if (neighbour_of_second_neighbour->voxel_idx == i)
						{
							second_neighbor_bridge_idx = m;
							break;
						}
					}
					//impossible things , maybe redundant, delete this after debug
					if ((second_neighbor_bridge_idx == -1) || (first_neighbor_bridge_idx == -1))
					{
						log_info("bridge voxel %d first index =%d second index =%d  \n", i, first_neighbor_bridge_idx, second_neighbor_bridge_idx);
						continue;
					}

					// update  neighbor item bridge plane index of the current bridge voxel 
					first_neighbour_item->plane_idx_of_bridge = bridge_plane_cnt;
					second_neighbour_item->plane_idx_of_bridge = bridge_plane_cnt;

					// update the  bridge plane index  info of the neighbour voxel which is the current bridge voxel's plane pair
					first_neighbour_voxel->neighbors[first_neighbor_bridge_idx].bridge_plane_idx_of_plane_pair = bridge_plane_cnt;
					second_neighbour_voxel->neighbors[second_neighbor_bridge_idx].bridge_plane_idx_of_plane_pair = bridge_plane_cnt;
					bridge_plane_cnt++;
					//parent_voxe_idx_of_plane_connected[j] = plane_merge_element[first_neighbour_item->voxel_idx].parent_voxel_idx[0];						
					//printf("i=%d bridge_plane_cnt =%d first_neighbour_item=%d,second_neighbour_item =%d\n\r", i, bridge_plane_cnt,first_neighbour_item->voxel_idx, second_neighbour_item->voxel_idx);
					//if(first_neighbour_voxeltype == GOOD_VOXEL) printf("i=%d bridge_plane_cnt =%d first_neighbour_item=%d,second_neighbour_item =%d\n\r", i, bridge_plane_cnt, first_neighbour_item->voxel_idx, second_neighbour_item->voxel_idx);
					//if (second_neighbour_voxeltype == GOOD_VOXEL) printf("i=%d bridge_plane_cnt =%d first_neighbour_item=%d,second_neighbour_item =%d\n\r", i, bridge_plane_cnt, first_neighbour_item->voxel_idx, second_neighbour_item->voxel_idx);
					//if (bridge_plane_cnt >1) printf("i=%d bridge_plane_cnt =%d first_neighbour_item=%d,second_neighbour_item =%d\n\r", i, bridge_plane_cnt, first_neighbour_item->voxel_idx, second_neighbour_item->voxel_idx);
				}
			}
		}

		if (bridge_plane_cnt != 0)
		{
			plane_merge_it->is_bridge = true;
			bridge_cnt++;
			plane_merge_it->bridge_voxel_elements.num_of_planes = bridge_plane_cnt;
			plane_merge_it->bridge_voxel_elements.parent_voxel_idxs = new unsigned int[bridge_plane_cnt][2];
			if (max_bridge_plane_cnt < bridge_plane_cnt)  max_bridge_plane_cnt = bridge_plane_cnt;

			if (bridge_plane_cnt == 1)
			{
				plane_merge_it->is_single_plane_bridge = true;
				single_bridge_cnt++;
			}
			if (plane_voxel->voxel_type == GOOD_VOXEL) good_bridge_cnt++;
			if (plane_voxel->voxel_type == PSEUDO_BAD_VOXEL) pseudobad_bridge_cnt++;
			if (plane_voxel->voxel_type == REAL_BAD_VOXEL) realbad_bridge_cnt++;
			if (bridge_plane_cnt > 13) log_fatal("bridge_plane_cnt = %d > 13 \n\r", bridge_plane_cnt);
		}

		for (int j = 0; j < 26; j++)
		{
			NeighborItem* neighbour_item = &plane_voxel->neighbors[j];

			if (!neighbour_item->is_occupied) continue;

			//get the neighbour parent voxel if neighbour is the voxel in the bridge planes , this is just the init value from one neighbour,and  will update in the Identify good or pesudo bad voxels
			if (neighbour_item->plane_idx_of_bridge < bridge_plane_cnt)
			{
                unsigned int neighbour_parent_voxel_idx = plane_merge_element[neighbour_item->voxel_idx].parent_voxel_idx[0];
				if(neighbour_parent_voxel_idx>plane_voxel_array.size) log_fatal("voxel %d  neighbour= %d parent_voxel_idx =%d excceed max %d \n\r", i, j, neighbour_parent_voxel_idx, plane_voxel_array.size);
				plane_merge_it->bridge_voxel_elements.parent_voxel_idxs[neighbour_item->plane_idx_of_bridge][0] = neighbour_parent_voxel_idx;
				plane_merge_it->bridge_voxel_elements.parent_voxel_idxs[neighbour_item->plane_idx_of_bridge][1] = neighbour_parent_voxel_idx;
			}
		}
	}

	num_of_bridge = bridge_cnt;
	num_of_good_bridge = good_bridge_cnt;
	num_of_pseudo_bad_bridge = pseudobad_bridge_cnt;
	num_of_real_bad_bridge = realbad_bridge_cnt;
	max_num_of_bridge_planes = max_bridge_plane_cnt;
	num_of_single_bridge = single_bridge_cnt;
#endif
	return;
}

// Find out the parent of all good voxels
void PlaneSegmentation::IdentifyParentOfGoodVoxels() {

#if 0
	unsigned int good_neighbor_count;				// total number of good neighbor
	unsigned int bad_neighbor_count;        // total number of bad neighbor
	bool neighbor_flag;						// neighbor_flag: indicate that it is a valid neighbor of current voxel
	unsigned int neighbor_voxel_idx;				// neighbor voxel index to occupied voxel array
	PlaneVoxelType neighbor_voxel_type;				// voxel_type: Good, Pseudo Bad or Real Bad
#pragma omp parallel for
	for (int i = 0; i < plane_voxel_array.size; i++) {

		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[i];
		if (plane_voxel->voxel_type != REAL_BAD_VOXEL) {

			good_neighbor_count = 0;
			bad_neighbor_count = 0;
			for (int j = 0; j < 26; j++) {

				NeighborItem* neighbor_it = &plane_voxel->neighbors[j];
				if (!neighbor_it->is_occupied) continue;
				neighbor_flag = neighbor_it->neighbor_flag;
				if (neighbor_flag == true) {

					neighbor_voxel_idx = neighbor_it->voxel_idx;
					neighbor_voxel_type = plane_voxel_array.voxels[neighbor_voxel_idx].voxel_type;
					if (neighbor_voxel_type != REAL_BAD_VOXEL) {

						good_neighbor_count++;
					}
					else
					{
						bad_neighbor_count++;
					}
				}
				else
				{
					bad_neighbor_count++;
				}
			}
			plane_merge_element[i].good_neighbor_count = good_neighbor_count;
			plane_merge_element[i].bad_neighbor_count = bad_neighbor_count;

		}
		else {

			plane_merge_element[i].good_neighbor_count = 0;
			for (int j = 0; j < 26; j++)
			{
				NeighborItem* neighbor_it = &plane_voxel->neighbors[j];
				if (!neighbor_it->is_occupied) continue;
				bad_neighbor_count++;
			}
			plane_merge_element[i].bad_neighbor_count = bad_neighbor_count;
		}
	}

#endif
	//if (config_params.identify_with_bridge)
	//{
	//	IdentifyParentVoxelsMethodWithBridge(GOOD_VOXEL, GOOD_VOXEL, 1);

	//}
	//else
	//{
	//	IdentifyParentOfVoxelsMethodNoBridge(GOOD_VOXEL, GOOD_VOXEL);
	//}
	IdentifyParentOfVoxelsMethodNoBridge(GOOD_VOXEL, GOOD_VOXEL);

	return;
}

// Merge all  voxels whose parent voxel is good into its similar plane
void PlaneSegmentation::MergeGoodVoxels() {

	bool* is_parent_idx = NULL;					// indicates whether the voxel in occupied voxel array is a parent voxel or not
	unsigned int* child_voxel_cnt_list = NULL;				// list of total number of voxels in plane
	unsigned int voxel_cnt;					// total number of voxels in plane
	unsigned int voxel_cnt_in_pow2;				// round up total number of voxels in plane to power of 2
	unsigned int parent_voxel_idx;				// parent voxel index to occupied voxel array
	unsigned int plane_index;				// plane index
	unsigned int plane_cnt;					// total number of planes
	unsigned int* plane_list = NULL;				// list of parent voxel indexes
	//unsigned int voxel_in_plane_idx;			// voxel index to occupied voxel array
	SumforCovariance* sums = NULL;					// Temporary storage for SumforCovariance in voxel
	unsigned int* point_cnt = NULL;				// Temporary storage for total number of points in voxel

	unsigned int* child_voxel_point_cnt_list = NULL;				// list of total number of points in plane

	float max_normal_diff = static_cast<float>(std::cos(plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_NORMAL_ANGLE_OF_TWO_VOXEL*M_PI / 180));

	// Initialize the flag indicating parent voxel in occupied voxel array
	// Initialize the voxel count in plane
	is_parent_idx = new bool[plane_voxel_array.size];
	child_voxel_cnt_list = new unsigned int[plane_voxel_array.size];
	child_voxel_point_cnt_list = new unsigned int[plane_voxel_array.size];
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(plane_voxel_array.size); i++) {

		is_parent_idx[i] = false;
		child_voxel_cnt_list[i] = 0;
		child_voxel_point_cnt_list[i] = 0;
	}

	// Find out the parent voxel index list
	for (unsigned int i = 0; i < plane_voxel_array.size; i++) {

		// Find out the parent voxel of all good voxels and record the total number of voxels belong to it
		PlaneVoxelItem *plane_voxel = &plane_voxel_array.voxels[i];

		if (plane_voxel->is_overall_merged&&plane_voxel->voxel_type == GOOD_VOXEL) {
			parent_voxel_idx = plane_merge_element[i].parent_voxel_idx[0];
			PlaneVoxelItem *parent_voxel = &plane_voxel_array.voxels[parent_voxel_idx];
			bool voxel_type_condition = (parent_voxel->voxel_type != GOOD_VOXEL);
			if (voxel_type_condition) continue;

			is_parent_idx[parent_voxel_idx] = true;
			//plane_voxel_array.voxels[i].is_being_merged = true;
			child_voxel_cnt_list[parent_voxel_idx]++;
			child_voxel_point_cnt_list[parent_voxel_idx] += plane_voxel->points.size;
		}
	}

	// Calculate the total number of planes
	plane_cnt = 0;
#pragma omp parallel for reduction( +:plane_cnt)
	for (int i = 0; i < static_cast<int>(plane_voxel_array.size); i++) {

		if (is_parent_idx[i] == true) {

			plane_cnt++;
		}
	}

	// Create the plane list to store all the parent voxel indexes to the occupied voxel array
	plane_list = new unsigned int[plane_cnt];
	voxel_to_plane_idx = new unsigned int[plane_voxel_array.size];
	for (unsigned int i = 0, j = 0; i < plane_voxel_array.size; i++) {

		if (is_parent_idx[i] == true) {

			plane_list[j] = i;
			j++;
		}
	}

	unsigned int min_voxel_num_of_plane = plane_seg_params.plane_seg_thresholds.THRESHOLD_MIN_VOXEL_NUM_OF_PLANE;
	unsigned int min_point_num_of_plane = plane_seg_params.plane_seg_thresholds.THRESHOLD_MIN_POINT_NUM_OF_PLANE/2;

	for (unsigned int i = 0; i < plane_cnt; i++) {

		parent_voxel_idx = plane_list[i];		
		//PlaneVoxelType voxel_type = plane_voxel_array.voxels[parent_voxel_idx].voxel_type;
		PlaneVoxelItem *parent_voxel = &plane_voxel_array.voxels[parent_voxel_idx];
		bool voxel_type_condition = (parent_voxel->voxel_type == GOOD_VOXEL);

		if (voxel_type_condition)
		{

			if ((child_voxel_cnt_list[parent_voxel_idx] <= min_voxel_num_of_plane) || (child_voxel_point_cnt_list[parent_voxel_idx] < min_point_num_of_plane))
			{
				child_voxel_cnt_list[parent_voxel_idx] = 0;
				is_parent_idx[parent_voxel_idx] = false;
				parent_voxel->is_overall_merged = false;
				bad_voxel_size++;
				//parent_voxel->is_being_merged = false;
			}
		}
	}

	if (child_voxel_point_cnt_list != NULL)
	{
		delete[] child_voxel_point_cnt_list;
		child_voxel_point_cnt_list = NULL;
	}

	int tmp_bad_voxel_cnt = 0;
#pragma omp parallel for reduction( +:tmp_bad_voxel_cnt)
	for (int i = 0; i < static_cast<int>(plane_voxel_array.size); i++) {

		PlaneVoxelItem *plane_voxel = &plane_voxel_array.voxels[i];
		unsigned int current_parent_voxel_idx = plane_merge_element[i].parent_voxel_idx[0];
		PlaneVoxelItem *parent_voxel = &plane_voxel_array.voxels[current_parent_voxel_idx];
		if ((parent_voxel->voxel_type == GOOD_VOXEL) && (!parent_voxel->is_overall_merged))
		{
			if (plane_voxel->is_overall_merged)
			{
				plane_voxel->is_overall_merged = false;
				//point_merged_voxel_size++;
				tmp_bad_voxel_cnt++;
				//plane_voxel->is_being_merged = false;
			}
			/*else
			{
				if (plane_voxel->is_being_merged)
					plane_voxel->is_being_merged = false;
			}*/
		}
	}
	bad_voxel_size += tmp_bad_voxel_cnt;

	if (plane_list != NULL)
	{
		delete[] plane_list;
		plane_list = NULL;
	}

	// reCalculate the total number of planes
	plane_cnt = 0;
#pragma omp parallel for reduction(+:plane_cnt)
	for (int i = 0; i < static_cast<int>(plane_voxel_array.size); i++) {

		if (is_parent_idx[i] == true) {

			plane_cnt++;
		}
	}
	log_info("good plane size =%d", plane_cnt);
	good_voxel_plane_size = plane_cnt;


	// Create the plane list to store all the parent voxel indexes to the occupied voxel array
	plane_list = new unsigned int[plane_cnt];
	for (unsigned int i = 0, j = 0; i < plane_voxel_array.size; i++) {

		if (is_parent_idx[i] == true) {

			plane_list[j] = i;
			voxel_to_plane_idx[i] = j;
			j++;
		}
		else
		{
			voxel_to_plane_idx[i] = -1;
		}
	}


	// Create instant of all PlaneMergeOutputItem in PlaneMergeOutput
	plane_merge_out.size = plane_cnt;
	plane_merge_out.planes = new PlaneMergeOutputItem[plane_cnt];

	//std::vector<Point3f> plane_parent_avg_normal(plane_cnt);
	for (unsigned int i = 0; i < plane_merge_out.size; i++) {

		PlaneMergeOutputItem *plane_it = &plane_merge_out.planes[i];
		parent_voxel_idx = plane_list[i];
		voxel_cnt = child_voxel_cnt_list[parent_voxel_idx];
		plane_it->plane_type = GOOD_PLANE;
		plane_it->total_point_cnt = 0;
		plane_it->parent_voxel_idx = parent_voxel_idx;
		plane_it->voxels.voxel_idx = new unsigned int[voxel_cnt];
		plane_it->voxels.size = voxel_cnt;
		plane_it->points.point_idx = NULL;
		plane_it->points.size = 0;
		//plane_it->good_voxel_size = voxel_cnt;
		//plane_it->pseudobad_voxel_size = 0;
		ClearSums(&plane_it->sums);
		plane_it->plane_mse = std::numeric_limits<float> ::infinity();
		plane_it->plane_normal = { 0.f,0.f,0.f };
		plane_it->plane_center = { 0.f,0.f,0.f };
		plane_it->extended_part_points.size = 0;
		plane_it->extended_part_points.point_idx = NULL;
		plane_it->multiplane_points.size = 0;
		plane_it->multiplane_points.point_idx = NULL;
		//if (!ComputeAvgNormal(parent_voxel_idx, plane_parent_avg_normal[i]))
		//{
		//	log_error("compute plane %d parent_voxel_idx =%d average normal failed", i, parent_voxel_idx);
		//	plane_parent_avg_normal[i] = plane_voxel_array.voxels[parent_voxel_idx].plane_normal;
		//}
	}

	// Initialize the voxel count in plane
#pragma omp parallel for 
	for (int i = 0; i < static_cast<int>(plane_voxel_array.size); i++) {

		child_voxel_cnt_list[i] = 0;
	}


	// Assign voxel indexes to PlaneMergeOutputItem
	for (unsigned int i = 0; i < plane_voxel_array.size; i++) {

		PlaneVoxelItem *plane_voxel = &plane_voxel_array.voxels[i];

		if (plane_voxel->voxel_type == GOOD_VOXEL) {

			parent_voxel_idx = plane_merge_element[i].parent_voxel_idx[0];
			PlaneVoxelItem *parent_voxel = &plane_voxel_array.voxels[parent_voxel_idx];
#ifdef IDENTIFY_GOOD_PSEUDOBAD_VOXEL_TOGETHER
			bool voxel_type_condition = (parent_voxel->voxel_type == REAL_BAD_VOXEL);
#else
			bool voxel_type_condition = (parent_voxel->voxel_type != GOOD_VOXEL);
#endif
			if (voxel_type_condition) continue;

			if (!parent_voxel->is_overall_merged) continue;

			voxel_cnt = child_voxel_cnt_list[parent_voxel_idx];
			plane_index = voxel_to_plane_idx[parent_voxel_idx];

			//check if current voxel normal diff with parent average normal
			float norm_diff = std::fabs( ComputeVectorDotProduct<float>(parent_voxel->avg_normal, plane_voxel->plane_normal));
			if (norm_diff < max_normal_diff)
			{
				//must set this voxel as point-to-plane mode 
				plane_voxel->is_overall_merged = false;
				//log_info("plane%d voxel idx%d normal diff =%f with parent voxel %d", plane_index, i, std::fabs(std::acos(norm_diff) * 180 / M_PI), parent_voxel_idx);
				continue;
			}
			plane_merge_out.planes[plane_index].voxels.voxel_idx[voxel_cnt] = i;
			plane_voxel->is_being_merged = true;
			child_voxel_cnt_list[parent_voxel_idx]++;
		}
	}

	// Merge all voxels in the same plane
	for (unsigned int i = 0; i < plane_merge_out.size; i++) {

		parent_voxel_idx = plane_merge_out.planes[i].parent_voxel_idx;
		voxel_cnt =child_voxel_cnt_list[parent_voxel_idx];
		plane_merge_out.planes[i].voxels.size = voxel_cnt;
		// Round up the voxel count to power of 2
		if (voxel_cnt > 1)
		{
			voxel_cnt_in_pow2 = (unsigned int)(std::pow(2, std::ceil(std::log(voxel_cnt) / std::log(2))));
		}
		else
		{
			voxel_cnt_in_pow2 = 2;	// if voxel == 1 , only self voxel, if voxel == 0 is not possible
		}

		// Temporary storage for SumforCovariance and total number of points in voxel
		sums = new SumforCovariance[voxel_cnt_in_pow2];
		point_cnt = new unsigned int[voxel_cnt_in_pow2];

		// Copy voxel data from PlaneVoxelArray to temporary storage
#pragma omp parallel for
		for (int j = 0; j < static_cast<int>(voxel_cnt); j++) {

			unsigned int voxel_in_plane_idx = plane_merge_out.planes[i].voxels.voxel_idx[j];
			sums[j].sum_x = plane_voxel_array.voxels[voxel_in_plane_idx].sums.sum_x;
			sums[j].sum_y = plane_voxel_array.voxels[voxel_in_plane_idx].sums.sum_y;
			sums[j].sum_z = plane_voxel_array.voxels[voxel_in_plane_idx].sums.sum_z;
			sums[j].sum_xx = plane_voxel_array.voxels[voxel_in_plane_idx].sums.sum_xx;
			sums[j].sum_yy = plane_voxel_array.voxels[voxel_in_plane_idx].sums.sum_yy;
			sums[j].sum_zz = plane_voxel_array.voxels[voxel_in_plane_idx].sums.sum_zz;
			sums[j].sum_xy = plane_voxel_array.voxels[voxel_in_plane_idx].sums.sum_xy;
			sums[j].sum_yz = plane_voxel_array.voxels[voxel_in_plane_idx].sums.sum_yz;
			sums[j].sum_xz = plane_voxel_array.voxels[voxel_in_plane_idx].sums.sum_xz;

			point_cnt[j] = plane_voxel_array.voxels[voxel_in_plane_idx].points.size;
		}

		// Initialize the useless memory
#pragma omp parallel for
		for (int j = voxel_cnt; j < static_cast<int>(voxel_cnt_in_pow2); j++) {

			sums[j].sum_x = 0.0;
			sums[j].sum_y = 0.0;
			sums[j].sum_z = 0.0;
			sums[j].sum_xx = 0.0;
			sums[j].sum_yy = 0.0;
			sums[j].sum_zz = 0.0;
			sums[j].sum_xy = 0.0;
			sums[j].sum_yz = 0.0;
			sums[j].sum_xz = 0.0;

			point_cnt[j] = 0;
		}

		// Update SumforCovariance in temporary storage
		unsigned int k = 0;
		unsigned int k_pow2, k_plus_1_pow2;
		do
		{
			k_pow2 = (unsigned int)(std::pow(2, k));
			k_plus_1_pow2 = (unsigned int)(std::pow(2, k+1));
#pragma omp parallel for 
			for (int j = 0; j < static_cast<int>(voxel_cnt_in_pow2); j = j + k_plus_1_pow2) {

				sums[j].sum_x = sums[j].sum_x + sums[j+k_pow2].sum_x;
				sums[j].sum_y = sums[j].sum_y + sums[j+k_pow2].sum_y;
				sums[j].sum_z = sums[j].sum_z + sums[j+k_pow2].sum_z;
				sums[j].sum_xx = sums[j].sum_xx + sums[j+k_pow2].sum_xx;
				sums[j].sum_yy = sums[j].sum_yy + sums[j+k_pow2].sum_yy;
				sums[j].sum_zz = sums[j].sum_zz + sums[j+k_pow2].sum_zz;
				sums[j].sum_xy = sums[j].sum_xy + sums[j+k_pow2].sum_xy;
				sums[j].sum_yz = sums[j].sum_yz + sums[j+k_pow2].sum_yz;
				sums[j].sum_xz = sums[j].sum_xz + sums[j+k_pow2].sum_xz;

				point_cnt[j] = point_cnt[j] + point_cnt[j+k_pow2];
			}

			k++;
		} while (k_plus_1_pow2 < voxel_cnt_in_pow2);

		// Assign the updated value of SumforCovariance to PlaneVoxelArray
		/*plane_voxel_array.voxels[parent_voxel_idx].sums.sum_x = sums[0].sum_x;
		plane_voxel_array.voxels[parent_voxel_idx].sums.sum_y = sums[0].sum_y;
		plane_voxel_array.voxels[parent_voxel_idx].sums.sum_z = sums[0].sum_z;
		plane_voxel_array.voxels[parent_voxel_idx].sums.sum_xx = sums[0].sum_xx;
		plane_voxel_array.voxels[parent_voxel_idx].sums.sum_yy = sums[0].sum_yy;
		plane_voxel_array.voxels[parent_voxel_idx].sums.sum_zz = sums[0].sum_zz;
		plane_voxel_array.voxels[parent_voxel_idx].sums.sum_xy = sums[0].sum_xy;
		plane_voxel_array.voxels[parent_voxel_idx].sums.sum_yz = sums[0].sum_yz;
		plane_voxel_array.voxels[parent_voxel_idx].sums.sum_xz = sums[0].sum_xz;*/

		// *********************************IMPORTANT***************************
		// The point_idx and size in PointInVoxelArray will not be updated at this moment,
		// only total_point_cnt in PlaneMergeOutputItem will be updated for calcuating the plane_mse,
		// plane_normal and plane_center.
		plane_merge_out.planes[i].total_point_cnt = point_cnt[0];

		// Compute new plane mse, normal and center
		VoxelMergeBaseClass occupied_voxel_grid(&plane_voxel_array.voxels[parent_voxel_idx], &plane_seg_params.plane_seg_thresholds);
		//occupied_voxel_grid.Compute(point_cnt[0]);
		occupied_voxel_grid.Compute(point_cnt[0], sums[0], plane_merge_out.planes[i].plane_normal, plane_merge_out.planes[i].plane_center);
		PushSums(&plane_merge_out.planes[i].sums, &sums[0]);
		// Release temporay memory for each plane
		delete[] sums;
		delete[] point_cnt;
	}

#ifdef UPDATE_NEIGHBOR_FLAG_AFTER_MERGE

	// Update normal_diff & neighbor_flag according to neighbor's parent after good voxel merging
	UpdateNeighborFlag();

	GetVoxelNeigbourDebugInfo(ALL_VOXEL_DEBUG);

#endif

	// Release memory
	if (is_parent_idx != NULL)
	{
		delete[] is_parent_idx;
		is_parent_idx = NULL;
	}

	if (child_voxel_cnt_list != NULL)
	{
		delete[] child_voxel_cnt_list;
		child_voxel_cnt_list = NULL;
	}
	if (plane_list != NULL)
	{
		delete[] plane_list;
		plane_list = NULL;
	}
	return;
}

/* Find out the parent of all pseudo bad voxels
Identify Parent voxel Criteria (note : only find parent in the good voxel for plane merging according to IdentifyParentOfGoodVoxels ): 
1)  have the smallest normal difference  in all its neighbors
2)  have the smallest distance  in all its neighbors
3)  have the best mse  in all its neighbors if more than one winner
Current Parent voxel is the parent voxel of winner selected by "1) " and "2)" and "3)" */

void PlaneSegmentation::IdentifyParentOfPseudoBadVoxels() 
{
	//if (config_params.identify_with_bridge)
	//{
	//	IdentifyParentVoxelsMethodWithBridge(PSEUDO_BAD_VOXEL, PSEUDO_BAD_VOXEL, 1);

	//}
	//else
	//{
	//	IdentifyParentOfVoxelsMethodNoBridge(PSEUDO_BAD_VOXEL, PSEUDO_BAD_VOXEL);
	//}
	IdentifyParentOfVoxelsMethodNoBridge(PSEUDO_BAD_VOXEL, PSEUDO_BAD_VOXEL);
	return;
}

bool PlaneSegmentation::IdentifyParentOfVoxelsMethodNoBridge(PlaneVoxelType voxeltype, PlaneVoxelType parent_voxel_type)
{
	//float max_normal_diff = static_cast<float>(std::cos(plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_NORMAL_ANGLE_OF_TWO_VOXEL * M_PI / 180));
	unsigned char update_flag;					// global update flag for each iterations
	unsigned char* update_flag_array = NULL;				// update flag for all occupied voxels
	unsigned char curr_ping_pong_buf_idx;				// index to parent_voxel_idx[2] in current iteration
	unsigned char next_ping_pong_buf_idx;				// index to parent_voxel_idx[2] in next iteration
	unsigned int iteration_count;
	iteration_count = 0;
	update_flag_array = new unsigned char[plane_voxel_array.size];
	do
	{
		// Update current parent voxel index from neighbor's parent voxel index
		curr_ping_pong_buf_idx = iteration_count % 2;
		next_ping_pong_buf_idx = (iteration_count + 1) % 2;
		
#pragma omp parallel for
		for (int i = 0; i < static_cast<int>(plane_voxel_array.size); i++) {

			update_flag_array[i] = 0;

			if (parent_voxel_type == VOXEL_INVALID_TYPE)
			{
				if ((plane_voxel_array.voxels[i].voxel_type == REAL_BAD_VOXEL) || (plane_voxel_array.voxels[i].is_being_merged)) continue;
			}
			else
			{
				if ((plane_voxel_array.voxels[i].voxel_type != voxeltype) || (plane_voxel_array.voxels[i].is_being_merged)) continue;
			}

			unsigned int current_parent_voxel_idx = plane_merge_element[i].parent_voxel_idx[curr_ping_pong_buf_idx];

			for (int j = 0; j < 26; j++) {

				bool neighbor_flag = plane_voxel_array.voxels[i].neighbors[j].neighbor_flag;
				if (!neighbor_flag) continue;

				unsigned int neighbor_voxel_idx = plane_voxel_array.voxels[i].neighbors[j].voxel_idx;
				PlaneVoxelType neighbor_voxel_type = plane_voxel_array.voxels[neighbor_voxel_idx].voxel_type;
				if (parent_voxel_type == VOXEL_INVALID_TYPE)
				{
					if (neighbor_voxel_type == REAL_BAD_VOXEL) continue;
				}
				else
				{
					if (neighbor_voxel_type != voxeltype) continue;
				}


				//parent_identify_update_item->neighbor_parent_voxel_idx = plane_merge_element[neighbor_voxel_idx].parent_voxel_idx[next_ping_pong_buf_idx];
				unsigned int neighbor_parent_voxel_idx = plane_merge_element[neighbor_voxel_idx].parent_voxel_idx[next_ping_pong_buf_idx];
				if(neighbor_parent_voxel_idx == current_parent_voxel_idx) continue;

				PlaneVoxelType neighbor_parent_voxel_type = plane_voxel_array.voxels[neighbor_parent_voxel_idx].voxel_type;

				if (parent_voxel_type == VOXEL_INVALID_TYPE)
				{
					if (neighbor_parent_voxel_type == REAL_BAD_VOXEL) continue;
				}
				else
				{
					if (neighbor_parent_voxel_type != parent_voxel_type) continue;
				}

				
				ParentIDCondType condition[5] = { GOOD_NEIGHBOR_CNT,BAD_NEIGHBOR_CNT, PLANE_MSE,GRID_IDX,INVALID_PARA};
				bool check_condition = ParentVoxelIdentifyCondition(condition, current_parent_voxel_idx, neighbor_parent_voxel_idx, update_flag_array[i]);
				if (!check_condition) continue;
				//float norm_diff = std::fabs(ComputeVectorDotProduct<float>(plane_voxel_array.voxels[i].avg_normal, plane_voxel_array.voxels[neighbor_parent_voxel_idx].plane_normal));
				//if (norm_diff < max_normal_diff)
				//	update_flag_array[i] = 0;
				if (update_flag_array[i] == 1)
				{
					plane_merge_element[i].parent_voxel_idx[curr_ping_pong_buf_idx] = neighbor_parent_voxel_idx;
				}
			}
		}

		// Check if no more updates
		update_flag = 0;
		for (unsigned int i = 0; i < plane_voxel_array.size; i++) {

			update_flag |= update_flag_array[i];
		}

		// Increment the iteration number
		iteration_count++;

	} while (update_flag > 0);

	// Release memory
	if (update_flag_array != NULL)
	{
		delete[] update_flag_array;
		update_flag_array = NULL;
	}
	log_debug("identify voxeltype %d parent_voxel_type %d iteration_count=%d", voxeltype, parent_voxel_type, iteration_count);
	return true;
}

//bool PlaneSegmentation::IdentifyParentVoxelsMethodWithBridge(PlaneVoxelType voxeltype, PlaneVoxelType parent_voxel_type, int max_bridge_plane_cnt)
//{
//	unsigned char update_flag;					// global update flag for each iterations
//	unsigned char* update_flag_array = NULL;				// update flag for all occupied voxels
//	unsigned char curr_ping_pong_buf_idx;				// index to parent_voxel_idx[2] in current iteration
//	unsigned char next_ping_pong_buf_idx;				// index to parent_voxel_idx[2] in next iteration
//	unsigned int iteration_count;
//	unsigned char **bridge_update_flag_array = NULL;           // update  flag array for voxels of the bridges 
//
//	iteration_count = 0;
//	update_flag_array = new unsigned char[plane_voxel_array.size];
//	bridge_update_flag_array = new unsigned char*[max_bridge_plane_cnt];
//	for (int i = 0; i < max_bridge_plane_cnt; i++)
//	{
//		bridge_update_flag_array[i] = new unsigned char[plane_voxel_array.size];
//	}	
//
//	do
//	{
//		curr_ping_pong_buf_idx = iteration_count % 2;
//		next_ping_pong_buf_idx = (iteration_count + 1) % 2;
//
//		// Update current parent voxel index from neighbor's parent voxel index
//#pragma omp parallel for
//		for (int i = 0; i < static_cast<int>(plane_voxel_array.size); i++) {
//
//			update_flag_array[i] = 0;
//			for (int j = 0; j < max_bridge_plane_cnt; j++)
//			{
//				bridge_update_flag_array[j][i] = 0;
//			}
//			// the condition of parent identify if current voxel is bridge
//			bool bridge_identify_condition = plane_merge_element[i].is_single_plane_bridge;			
//
//			//if (((plane_voxel_array.voxels[i].voxel_type != voxeltype) && !bridge_identify_condition) || (plane_voxel_array.voxels[i].is_being_merged)) continue;
//			// bridge must always update bridge plane parent info even  if it is being merged
//			if (((plane_voxel_array.voxels[i].voxel_type == REAL_BAD_VOXEL) && !bridge_identify_condition) || ((plane_voxel_array.voxels[i].is_being_merged) && !bridge_identify_condition)) continue;
//			unsigned current_parent_voxel_idx = plane_merge_element[i].parent_voxel_idx[curr_ping_pong_buf_idx];
//
//			for (int j = 0; j < 26; j++) 
//			{
//				NeighborItem *neighbor_item = &plane_voxel_array.voxels[i].neighbors[j];
//				if (!neighbor_item->is_occupied) continue;
//
//				bool neighbor_flag = neighbor_item->neighbor_flag;
//
//				if ((!bridge_identify_condition&&!neighbor_flag) || (bridge_identify_condition&&!neighbor_flag &&!(neighbor_item->bridge_plane_idx_of_plane_pair < 13))) continue;
//
//				/*if current voxel is not bridge or  is bridge and this neighbour is good neighbour ,  only update the currennt fitted plane info;
//				 but this case  must identify  parent from neighbour both if neighbor_flag is true  and ifbridge_neighbour_condition is true*/	
//				// if current voxel is bridge and the current neighbor is its good neighbor(neighbor_flag os true), identify parent as same as other non-bridge voxels
//				if ((!bridge_identify_condition) || (bridge_identify_condition && neighbor_flag))
//				{
//					if (bridge_identify_condition && neighbor_flag && plane_voxel_array.voxels[i].is_being_merged) continue;
//					//bool bridge_neighbour_condition = &plane_merge_element[neighbor_item->voxel_idx].is_single_plane_bridge; // bridge_neighbour_condition check if the current neighbour is  a bridge voxel
//					// to  good or pseudo bad voxel , its neighbours maybe have multiple bridge , so must check bridge_plane_idx_of_plane_pair <13  to make sure the current neighbour is self bridge
//					bool bridge_neighbour_condition = (neighbor_item->bridge_plane_idx_of_plane_pair < 13);
//					if ((!neighbor_flag) && !bridge_neighbour_condition) continue;   // if neighbor is not fitted plane voxel , it must be a bridge voxel, so must check if this neighbour is a bridge
//					unsigned int neighbor_voxel_idx = neighbor_item->voxel_idx;
//					PlaneVoxelType neighbor_voxel_type = plane_voxel_array.voxels[neighbor_voxel_idx].voxel_type;
//					if ((neighbor_voxel_type == REAL_BAD_VOXEL) && !bridge_neighbour_condition) continue; // if it is not bridge neighbour , check if voxel type is  real bad voxel
//
//					unsigned int neighbor_parent_voxel_idx;
//					if (neighbor_flag)    
//					{
//						neighbor_parent_voxel_idx = plane_merge_element[neighbor_voxel_idx].parent_voxel_idx[next_ping_pong_buf_idx];
//					}
//					else // if it is not bridge voxel but have a bridge  neighbour,  must check if update parent index from the fitted plane in this bridge neighbor
//					{
//						neighbor_parent_voxel_idx = plane_merge_element[neighbor_voxel_idx].bridge_voxel_elements.parent_voxel_idxs[neighbor_item->bridge_plane_idx_of_plane_pair][next_ping_pong_buf_idx];
//					}
//
//					if (neighbor_parent_voxel_idx == current_parent_voxel_idx) continue;
//
//					PlaneVoxelType neighbor_parent_voxel_type = plane_voxel_array.voxels[neighbor_parent_voxel_idx].voxel_type;
//					
//					if (parent_voxel_type == VOXEL_INVALID_TYPE)
//					{
//						if (neighbor_parent_voxel_type == REAL_BAD_VOXEL) continue;
//					}
//					else
//					{
//						if (neighbor_parent_voxel_type != parent_voxel_type) continue;
//					}					
//
//					ParentIDCondType condition[5] = { GOOD_NEIGHBOR_CNT,BAD_NEIGHBOR_CNT,PLANE_MSE,GRID_IDX,INVALID_PARA};
//					bool check_condition = ParentVoxelIdentifyCondition(condition, current_parent_voxel_idx, neighbor_parent_voxel_idx, update_flag_array[i]);
//					if (!check_condition) continue;
//					if (update_flag_array[i] == 1)
//					{
//						plane_merge_element[i].parent_voxel_idx[curr_ping_pong_buf_idx] = neighbor_parent_voxel_idx;
//					}
//				}
//				else  //  if bridge_identify_condition and neighbor_flag is false
//				{
//					bool bridge_neighbour_condition = (neighbor_item->plane_idx_of_bridge < 13);
//					if (!bridge_neighbour_condition) continue;
//					unsigned int bridge_current_parent_voxel_idx = plane_merge_element[i].bridge_voxel_elements.parent_voxel_idxs[neighbor_item->plane_idx_of_bridge][curr_ping_pong_buf_idx];
//					unsigned int neighbor_voxel_idx = neighbor_item->voxel_idx;
//					unsigned int bridge_neighbor_parent_voxel_idx = plane_merge_element[neighbor_voxel_idx].parent_voxel_idx[next_ping_pong_buf_idx];
//
//					if (bridge_neighbor_parent_voxel_idx == bridge_current_parent_voxel_idx) continue;
//					PlaneVoxelType neighbor_parent_voxel_type = plane_voxel_array.voxels[bridge_neighbor_parent_voxel_idx].voxel_type;
//					if (neighbor_parent_voxel_type != parent_voxel_type) continue;
//
//					if (parent_voxel_type == VOXEL_INVALID_TYPE)
//					{
//						if (neighbor_parent_voxel_type == REAL_BAD_VOXEL) continue;
//					}
//					else
//					{
//						if (neighbor_parent_voxel_type != parent_voxel_type) continue;
//					}
//
//					ParentIDCondType condition[5] = { GOOD_NEIGHBOR_CNT,BAD_NEIGHBOR_CNT,PLANE_MSE,GRID_IDX,INVALID_PARA};
//					//bool check_condition = ParentVoxelIdentifyCondition(condition, bridge_identify_it, bridge_update_flag_array[neighbor_item->plane_idx_of_bridge][i]);
//					bool check_condition = ParentVoxelIdentifyCondition(condition, bridge_current_parent_voxel_idx, bridge_neighbor_parent_voxel_idx, bridge_update_flag_array[neighbor_item->plane_idx_of_bridge][i]);
//					if (!check_condition) continue;
//					if (bridge_update_flag_array[neighbor_item->plane_idx_of_bridge][i] == 1)
//					{
//						plane_merge_element[i].bridge_voxel_elements.parent_voxel_idxs[neighbor_item->plane_idx_of_bridge][curr_ping_pong_buf_idx] = bridge_neighbor_parent_voxel_idx;
//					}
//				} // endif  ((!bridge_identify_condition) || (bridge_identify_condition && neighbor_flag))
//			}// endif j
//		}  // endif i
//
//		// Check if no more updates
//		update_flag = 0;
//		for (unsigned int i = 0; i < plane_voxel_array.size; i++) {
//
//			update_flag |= update_flag_array[i];
//			for (int j = 0; j < max_bridge_plane_cnt; j++)
//			{
//				update_flag |= bridge_update_flag_array[j][i];
//			}
//		}
//		
//		// Increment the iteration number
//		iteration_count++;		
//	} while (update_flag > 0);
//		
//	// Release memory
//	if (update_flag_array != NULL)
//	{
//		delete[] update_flag_array;
//		update_flag_array = NULL;
//	}
//
//	//log_debug("identify voxeltype %d parent_voxel_type %d iteration_count=%d", voxeltype,parent_voxel_type,iteration_count);
//	if (bridge_update_flag_array != NULL)
//	{
//		for (int i = 0; i < max_bridge_plane_cnt; i++)
//		{
//			if (bridge_update_flag_array[i] != NULL)
//			{
//				delete[] bridge_update_flag_array[i];
//				bridge_update_flag_array[i] = NULL;
//			}
//		}
//		delete[] bridge_update_flag_array;
//		bridge_update_flag_array = NULL;
//	}
//	return true;
//}

bool PlaneSegmentation::NeighborParentCompare(ParentIDCondType *condition, ParentVoxelIdentifyUpdateItem * parent_voxel_id_update_it, unsigned char &update_flag)
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
			neighbor_good_than_current[i] =false;
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

// ParentVoxelIdentifyCondition must be thread-safe function if do parallel computing
bool PlaneSegmentation::ParentVoxelIdentifyCondition(ParentIDCondType *condition, const unsigned int current_parent_voxel_idx, const unsigned int neighbour_parent_voxel_idx, unsigned char &update_flag)
{
	ParentVoxelIdentifyUpdateItem parent_voxel_id_update_item;
	ParentVoxelIdentifyUpdateItem* parent_voxel_id_update_it = &parent_voxel_id_update_item;

	parent_voxel_id_update_it->neighbor_good_neighbor_count = plane_merge_element[neighbour_parent_voxel_idx].good_neighbor_count;
	parent_voxel_id_update_it->neighbor_mse = plane_voxel_array.voxels[neighbour_parent_voxel_idx].plane_mse;
	parent_voxel_id_update_it->neighbor_grid_idx = plane_voxel_array.voxels[neighbour_parent_voxel_idx].grid_idx;
	parent_voxel_id_update_it->neighbor_bad_neighbor_count = plane_merge_element[neighbour_parent_voxel_idx].bad_neighbor_count;

	parent_voxel_id_update_it->largest_good_neighbor_count = plane_merge_element[current_parent_voxel_idx].good_neighbor_count;
	parent_voxel_id_update_it->smallest_mse = plane_voxel_array.voxels[current_parent_voxel_idx].plane_mse;
	parent_voxel_id_update_it->grid_idx = plane_voxel_array.voxels[current_parent_voxel_idx].grid_idx;
	parent_voxel_id_update_it->smallest_bad_neighbor_count = plane_merge_element[current_parent_voxel_idx].bad_neighbor_count;

	bool rtn = NeighborParentCompare(condition, parent_voxel_id_update_it, update_flag);
	// update current parent voxel index

	if (rtn)
	{
		if (update_flag == 1)
		{
			return true;
		}
	}
	return false;
}

// Merge all pseudo bad voxels into its similar plane
void PlaneSegmentation::MergePseudoBadVoxels(PlaneMergeOutput *pseudobad_plane_merge_out) 
{

	unsigned int* child_voxel_cnt_list = NULL;				// list of total number of  pseudo bad voxels  in plane (a parent voxel )
	unsigned int* pseudobad_voxel_cnt_list = NULL;				// list of total number of  pseudo bad voxels  in plane (a parent voxel )

	unsigned int voxel_cnt;					    // total number of voxels in plane
	unsigned int parent_voxel_idx;				// parent voxel index to occupied voxel array
	//unsigned int plane_cnt;					    // total number of planes 
	//unsigned int* plane_list;				    // list of parent voxel indexes

	unsigned int* pseudobad_point_cnt_list = NULL;				// list of total number of  points  in plane (a parent voxel )
	float max_normal_diff = static_cast<float>(std::cos(plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_NORMAL_ANGLE_OF_TWO_VOXEL*M_PI / 180));

	// Initialize the voxel count in plane
	child_voxel_cnt_list = new unsigned int[plane_voxel_array.size];
	pseudobad_voxel_cnt_list = new unsigned int[plane_voxel_array.size];
	pseudobad_point_cnt_list = new unsigned int[plane_voxel_array.size];

#pragma omp parallel for
	for(int i = 0; i < static_cast<int>(plane_voxel_array.size); i++) {

		child_voxel_cnt_list[i] = 0;
		pseudobad_voxel_cnt_list[i] = 0;
		//voxel_to_plane_idx[i] = -1;
		pseudobad_point_cnt_list[i] = 0;
	}
	
	//step1:  Find out all  the parent voxel index of pseudo bad voxels
	for (unsigned int i = 0; i < plane_voxel_array.size; i++) 
	{
		PlaneVoxelItem *plane_voxel = &plane_voxel_array.voxels[i];
		// Find out the parent voxel of all pseudo bad voxels and record the total number of voxels belong to it
		if (plane_voxel->voxel_type == PSEUDO_BAD_VOXEL) {
			if (plane_voxel->is_being_merged) continue;
			if (!plane_voxel->is_overall_merged) continue;
			parent_voxel_idx = plane_merge_element[i].parent_voxel_idx[0];
			PlaneVoxelItem *parent_voxel = &plane_voxel_array.voxels[parent_voxel_idx];
			
			if (parent_voxel->voxel_type != PSEUDO_BAD_VOXEL) continue;
			if (parent_voxel->is_overall_merged)
			{
				child_voxel_cnt_list[parent_voxel_idx]++;
				//pseudobad_point_cnt_list[parent_voxel_idx] += plane_voxel_array.voxels[i].points.size;
				//plane_voxel_array.voxels[i].is_being_merged = true;
			}
			else
			{
				plane_voxel->is_overall_merged = false;
				bad_voxel_size++;
			}
		}
	}

	// Calculate the total number of planes
	unsigned int tmp_plane_cnt = 0;
#pragma omp parallel for reduction( +:tmp_plane_cnt)
	for (int i = 0; i < static_cast<int>(plane_voxel_array.size); i++) {

		if (child_voxel_cnt_list[i] != 0) {

			tmp_plane_cnt++;
		}
	}

	//step2  Create the plane list to store all the parent voxel indexes to the occupied voxel array
	unsigned int * tmp_plane_list = new unsigned int[tmp_plane_cnt];
	for (unsigned int i = 0, j = 0; i < plane_voxel_array.size; i++) {

		if (child_voxel_cnt_list[i] != 0) {

			tmp_plane_list[j] = i;
			j++;
		}
	}

	log_info("tmp_plane_cnt =%d", tmp_plane_cnt);

	//step3 Create all PlaneMergeOutputItem for pseudo bad voxels	
	//std::vector<Point3f> plane_parent_avg_normal(tmp_plane_cnt);
	PlaneMergeOutputItem *tmp_pseudobad_planes = new PlaneMergeOutputItem[tmp_plane_cnt];
	for (unsigned int i = 0; i < tmp_plane_cnt; i++) {
		PlaneMergeOutputItem* pseudobad_plane_it = &tmp_pseudobad_planes[i];
		parent_voxel_idx = tmp_plane_list[i];
		voxel_cnt = child_voxel_cnt_list[parent_voxel_idx];
		pseudobad_plane_it->plane_type = PSEUDO_BAD_PLANE;
		pseudobad_plane_it->parent_voxel_idx = parent_voxel_idx;
		pseudobad_plane_it->total_point_cnt = 0;// pseudobad_point_cnt_list[parent_voxel_idx];
		pseudobad_plane_it->voxels.voxel_idx = new unsigned int[voxel_cnt];
		pseudobad_plane_it->voxels.size = voxel_cnt;
		voxel_to_plane_idx[parent_voxel_idx] = i + good_voxel_plane_size;	
		//pseudobad_plane_it->good_voxel_size = 0;
		//pseudobad_plane_it->pseudobad_voxel_size = 0;
		pseudobad_plane_it->plane_mse = std::numeric_limits<float> ::infinity();
		pseudobad_plane_it->plane_normal = { 0.f,0.f,0.f }; 
		pseudobad_plane_it->plane_center = { 0.f,0.f,0.f };
		pseudobad_plane_it->points.size = 0;
		pseudobad_plane_it->points.point_idx = NULL;
		pseudobad_plane_it->extended_part_points.size = 0;
		pseudobad_plane_it->extended_part_points.point_idx = NULL;
		pseudobad_plane_it->multiplane_points.size = 0;
		pseudobad_plane_it->multiplane_points.point_idx = NULL;
		ClearSums(&pseudobad_plane_it->sums);
		//if (!ComputeAvgNormal(parent_voxel_idx, plane_parent_avg_normal[i]))
		//{
		//	log_error("compute plane %d parent_voxel_idx =%d average normal failed",i, parent_voxel_idx);			
		//	plane_parent_avg_normal[i] = plane_voxel_array.voxels[parent_voxel_idx].plane_normal;
		//}
	}


	//step4  Assign voxel indexes to pseudobad_voxel_merge_planes and chek normal difference with parent average normal
	for (unsigned int i = 0; i < plane_voxel_array.size; i++) 
	{
		PlaneVoxelItem *plane_voxel = &plane_voxel_array.voxels[i];
		if (plane_voxel->voxel_type == PSEUDO_BAD_VOXEL) {

			if (plane_voxel->is_being_merged) continue;
			if (!plane_voxel->is_overall_merged) continue;
			parent_voxel_idx = plane_merge_element[i].parent_voxel_idx[0];
			PlaneVoxelItem *parent_voxel = &plane_voxel_array.voxels[parent_voxel_idx];
			if (parent_voxel->voxel_type != PSEUDO_BAD_VOXEL) continue;
			if (parent_voxel->is_overall_merged)
			{
				voxel_cnt = pseudobad_voxel_cnt_list[parent_voxel_idx];
				unsigned int  pseudobad_plane_indx = voxel_to_plane_idx[parent_voxel_idx] - good_voxel_plane_size;	
				if (pseudobad_plane_indx > tmp_plane_cnt)
				{
					// it is impossible , should have bug
					log_error("pseudobad_plane_indx exceed size of pseudobad_plane_merge_out", pseudobad_plane_indx, pseudobad_plane_merge_out->size);
					continue;
				}
				//check if current voxel normal diff with parent average normal
				float norm_diff = std::fabs( ComputeVectorDotProduct<float>(parent_voxel->avg_normal, plane_voxel->plane_normal));
				if (norm_diff < max_normal_diff)
				{
					//must set this voxel as point-to-plane mode 
					plane_voxel->is_overall_merged = false;
					//log_info("pseudobad plane%d voxel idx%d normal diff =%f with parent voxel %d", pseudobad_plane_indx, i, std::fabs(std::acos(norm_diff) * 180 / M_PI), parent_voxel_idx);					
					continue;
				}
				tmp_pseudobad_planes[pseudobad_plane_indx].voxels.voxel_idx[voxel_cnt] = i;
				pseudobad_point_cnt_list[parent_voxel_idx] += plane_voxel->points.size;
				pseudobad_voxel_cnt_list[parent_voxel_idx]++;
			}
		}
	}

	// step5 check the plane whose parent voxel is pseudo bad voxel, here add more condition than good voxel to check if plane can be generated 
	float max_plane_mse = plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_PLANE;
	unsigned int min_voxel_num_of_plane = plane_seg_params.plane_seg_thresholds.THRESHOLD_MIN_VOXEL_NUM_OF_PLANE;	
	unsigned int min_point_num_of_plane = plane_seg_params.plane_seg_thresholds.THRESHOLD_MIN_POINT_NUM_OF_PLANE;
	log_debug("pseudo bad plane max_plane_mse =%f min_voxel_num_of_plane =%d min_point_num_of_plane=%d", max_plane_mse, min_voxel_num_of_plane, min_point_num_of_plane);
	log_debug("max_mse_of_mini_plane =%f ", max_mse_of_mini_plane);
	for (unsigned int i = 0; i < tmp_plane_cnt; i++)
	{
		//bool dbg_con = (i == 159);
		PlaneMergeOutputItem *tmp_merge_plane_it = &tmp_pseudobad_planes[i];
		if ((tmp_merge_plane_it->voxels.size <= 3 * min_voxel_num_of_plane))
		{
			max_plane_mse = max_mse_of_mini_plane;
		}
		else
		{
			max_plane_mse = plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_PLANE;
		}
		//here must update voxel size and total point size because voxel is less than initial for check normal difference in step4
		parent_voxel_idx = tmp_merge_plane_it->parent_voxel_idx;
		tmp_merge_plane_it->voxels.size = pseudobad_voxel_cnt_list[parent_voxel_idx];
		tmp_merge_plane_it->total_point_cnt = pseudobad_point_cnt_list[parent_voxel_idx];
		PlaneVoxelType voxel_type = plane_voxel_array.voxels[parent_voxel_idx].voxel_type;
		if (voxel_type == PSEUDO_BAD_VOXEL)
		{
			float plane_mse;
			//GetPlaneDistMseByVoxelAvg(tmp_pseudobad_voxel_merge_planes[i].voxels, plane_mse);
			GetPlaneDistMseByPointAvg(&plane_voxel_array.voxels[parent_voxel_idx], tmp_merge_plane_it->voxels, plane_mse);
			//if (dbg_con) log_info("PB plane %d plane_mse =%f voxel size =%d parent_voxel_idx=%d point size =%d",i, plane_mse, tmp_merge_plane_it->voxels.size, parent_voxel_idx,pseudobad_point_cnt_list[parent_voxel_idx]);
			// filtered condition of pseudobad plane generated can be added in here
			if ((plane_mse > max_plane_mse) || (tmp_merge_plane_it->voxels.size <= min_voxel_num_of_plane)\
				|| pseudobad_point_cnt_list[parent_voxel_idx] < min_point_num_of_plane)
			{
				child_voxel_cnt_list[parent_voxel_idx] = 0;
				voxel_to_plane_idx[parent_voxel_idx] = -1;  // if paret voxel is canceled , must upadate the voxel_to_plane_idx
				for (unsigned int j = 0; j < tmp_merge_plane_it->voxels.size; j++)
				{
					unsigned int voxel_idx = tmp_merge_plane_it->voxels.voxel_idx[j];
					PlaneVoxelItem *plane_voxel = &plane_voxel_array.voxels[voxel_idx];
					if (plane_voxel->is_overall_merged)
					{
						plane_voxel->is_overall_merged = false;
						bad_voxel_size++;
					}
				}
			}
		}
	}
	log_debug("bad_voxel_size =%d", bad_voxel_size);


#ifdef SAVE_OUTPUT_FILE_DEBUG
	if(debug_config_params.pseudo_bad_lost_plane_output_debug)
	OutputPseudoBadMergeLostPlaneInfo(current_out_path, tmp_pseudobad_planes, tmp_plane_cnt);
#endif

	//assign remaining pseudobad planes  to pseudobad_plane_merge_out, and delete the lost pseduobad planes		
	int pseudobad_plane_cnt = 0;
	for (unsigned int i = 0; i < plane_voxel_array.size; i++) {
		if (child_voxel_cnt_list[i] != 0) {

			pseudobad_plane_cnt++;
		}
	}
	log_info("pseudobad plane count =%d", pseudobad_plane_cnt);

	pseudobad_voxel_plane_size = pseudobad_plane_cnt;
	pseudobad_plane_merge_out->size = pseudobad_plane_cnt;
	pseudobad_plane_merge_out->planes = new PlaneMergeOutputItem[pseudobad_plane_cnt];
	pseudobad_plane_cnt = 0;
	for (unsigned int i = 0; i < tmp_plane_cnt; i++)
	{
		PlaneMergeOutputItem* tmp_pseudobad_plane_it = &tmp_pseudobad_planes[i];
		if ((child_voxel_cnt_list[tmp_pseudobad_plane_it->parent_voxel_idx]) == 0)
		{
			// now tmp_pseudobad_planes[i].points and extended_part_points is still NULL, is not need to delete
			delete[] tmp_pseudobad_planes[i].voxels.voxel_idx;
			tmp_pseudobad_planes[i].voxels.voxel_idx = NULL;
			continue;
		}
		pseudobad_plane_merge_out->planes[pseudobad_plane_cnt] = tmp_pseudobad_planes[i];
		// must update voxel_to_plane_idx for pseduobad parent voxel index 
		unsigned int pseduobad_parent_voxel_idx = pseudobad_plane_merge_out->planes[pseudobad_plane_cnt].parent_voxel_idx;
		voxel_to_plane_idx[pseduobad_parent_voxel_idx] = pseudobad_plane_cnt + good_voxel_plane_size;
		//must upate the size of voxel and points
		//pseudobad_plane_merge_out->planes[pseudobad_plane_cnt].voxels.size = pseudobad_voxel_cnt_list[pseduobad_parent_voxel_idx];
		//pseudobad_plane_merge_out->planes[pseudobad_plane_cnt].total_point_cnt = pseudobad_point_cnt_list[pseduobad_parent_voxel_idx];
		pseudobad_plane_cnt++;
	}
#pragma omp parallel for reduction( +:tmp_plane_cnt)
	for (int i = 0; i < static_cast<int>(pseudobad_plane_merge_out->size); i++)
	{
		PlaneMergeOutputItem *pseudobad_plane_it = &pseudobad_plane_merge_out->planes[i];
		for (unsigned int k = 0; k< pseudobad_plane_it->voxels.size; k++)
		{
			unsigned int voxel_idx = pseudobad_plane_it->voxels.voxel_idx[k];
			PlaneVoxelItem *plane_voxel = &plane_voxel_array.voxels[voxel_idx];
			//pseudobad_plane_it->total_point_cnt += plane_voxel->points.size;
			PushSums(&pseudobad_plane_it->sums, &plane_voxel->sums);
			plane_voxel->is_being_merged = true;

			//here must renew point-to-plane merging information of psuedobad voxels in reference planes, escape to be merged into other planes 
			unsigned int bad_voxel_idx = voxel_idx_to_bad_voxel[voxel_idx];
			if (bad_voxel_idx > point_merged_voxel_size)
			{
				// it is impossible ,should have bug
				log_error("pseudobad plane %d voxel%d voxel type%d bad_voxel_idx=%d exceed point_merged_voxel_size =%d", i, voxel_idx, plane_voxel->voxel_type, bad_voxel_idx,  point_merged_voxel_size);
				continue;
			}
			BadVoxelMergeOutpuItem *bad_voxel_it = &bad_voxel_merge_item[bad_voxel_idx];
			bad_voxel_it->is_being_merged = true;
			bad_voxel_it->is_remainer_occupied = false;
			bad_voxel_it->num_of_points_merged = bad_voxel_it->voxel_point_size;
		}
		//log_info("pseudo bad voxel plane idx %d total_point_cnt =%d", i, pseudobad_plane_it->total_point_cnt);
		float eigen_mse;
		MathOperation::Compute(pseudobad_plane_it->total_point_cnt, pseudobad_plane_it->sums, pseudobad_plane_it->plane_normal, pseudobad_plane_it->plane_center, eigen_mse);
		//MathOperation::GetPointsArrayMse(pt_cloud_xyz,pseudobad_plane_it->points, pseudobad_plane_it->plane_normal, pseudobad_plane_it->plane_center, pseudobad_plane_it->plane_mse, high_mse_ratio);
		GetPlaneDistMse(pseudobad_plane_it, pseudobad_plane_it->plane_mse);
	}

#ifdef SAVE_OUTPUT_FILE_DEBUG
		if (debug_config_params.merge_pseudobad_voxel_debug)
		{
			std::string merge_pseudobad_path = current_out_path;
			std::string merge_pseudobad_file = "merge_pseudobad_voxel.txt";
			std::ofstream merge_pseudobad_voxel(merge_pseudobad_path + merge_pseudobad_file);

			merge_pseudobad_voxel << "Total number of planes: " << plane_merge_out.size << '\n';
			unsigned int total_points_cnt_all_plane = 0;
			unsigned int total_voxels_cnt_all_plane = 0;
			for (unsigned int i = 0; i < plane_merge_out.size; i++)
			{
				total_voxels_cnt_all_plane += plane_merge_out.planes[i].voxels.size;
				total_points_cnt_all_plane += plane_merge_out.planes[i].total_point_cnt;
			}
			merge_pseudobad_voxel << "Total number of voxels: " << total_voxels_cnt_all_plane << '\n';
			merge_pseudobad_voxel << "Total number of points: " << total_points_cnt_all_plane << '\n';

			for (unsigned int i = 0; i < plane_merge_out.size; i++) {


				unsigned int parent_voxel_idx = plane_merge_out.planes[i].parent_voxel_idx;

				if (parent_voxel_idx > plane_voxel_array.size)
				{
					log_warn("plane %d parent_voxel_idx = %d is exceed sizeof plane_voxel_array.size %d\n", i, parent_voxel_idx, plane_voxel_array.size);
					continue;
				}


				merge_pseudobad_voxel << "Plane Index = " << i << ", Parent Voxel Index = " << plane_merge_out.planes[i].parent_voxel_idx << ", Voxel Count = " << plane_merge_out.planes[i].voxels.size << ", Point Count = " << plane_merge_out.planes[i].total_point_cnt << '\n';

				merge_pseudobad_voxel << std::setprecision(10) << std::fixed;
				merge_pseudobad_voxel << i << '\t' << plane_voxel_array.voxels[parent_voxel_idx].plane_mse << '\n';
				merge_pseudobad_voxel << i << '\t' << plane_voxel_array.voxels[parent_voxel_idx].plane_center.x << '\t' << plane_voxel_array.voxels[parent_voxel_idx].plane_center.y << '\t' << plane_voxel_array.voxels[parent_voxel_idx].plane_center.z << '\n';
				merge_pseudobad_voxel << i << '\t' << plane_voxel_array.voxels[parent_voxel_idx].plane_normal.x << '\t' << plane_voxel_array.voxels[parent_voxel_idx].plane_normal.y << '\t' << plane_voxel_array.voxels[parent_voxel_idx].plane_normal.z << '\n';
				merge_pseudobad_voxel << i << '\t' << plane_voxel_array.voxels[parent_voxel_idx].sums.sum_x << '\n';
				merge_pseudobad_voxel << i << '\t' << plane_voxel_array.voxels[parent_voxel_idx].sums.sum_y << '\n';
				merge_pseudobad_voxel << i << '\t' << plane_voxel_array.voxels[parent_voxel_idx].sums.sum_z << '\n';
				merge_pseudobad_voxel << i << '\t' << plane_voxel_array.voxels[parent_voxel_idx].sums.sum_xx << '\n';
				merge_pseudobad_voxel << i << '\t' << plane_voxel_array.voxels[parent_voxel_idx].sums.sum_yy << '\n';
				merge_pseudobad_voxel << i << '\t' << plane_voxel_array.voxels[parent_voxel_idx].sums.sum_zz << '\n';
				merge_pseudobad_voxel << i << '\t' << plane_voxel_array.voxels[parent_voxel_idx].sums.sum_xy << '\n';
				merge_pseudobad_voxel << i << '\t' << plane_voxel_array.voxels[parent_voxel_idx].sums.sum_xz << '\n';
				merge_pseudobad_voxel << i << '\t' << plane_voxel_array.voxels[parent_voxel_idx].sums.sum_yz << '\n';
			}
			merge_pseudobad_voxel.close();
		}
#endif
	delete[] tmp_plane_list;
	delete[] tmp_pseudobad_planes;
	delete[] child_voxel_cnt_list;
	delete[] pseudobad_voxel_cnt_list;
	return;
}


// apply the resources of bad_voxel_merge_item, include all the pseudo bad voxels and bad voxels, and point-to-plane merging
void PlaneSegmentation::CreateBadVoxelItems(bool is_good_voxel_plane)
{
	if (!is_good_voxel_plane)
	{
		// check if bad_voxel_merge_item has already initialized, is_good_voxel_plane  false must be called after true;
		if (bad_voxel_merge_item == NULL)
		{
			log_error("bad_voxel_merge_item is NULL");
			return;
		}

		if (pseudobad_voxel_plane_size == 0) return;

		unsigned int total_plane_size = good_voxel_plane_size + pseudobad_voxel_plane_size;
		bool **new_remaining_points_merged_plane = new bool*[point_merged_voxel_size];

		for (unsigned int i = 0; i < point_merged_voxel_size; i++)
		{
			BadVoxelMergeOutpuItem* bad_voxel_it = &bad_voxel_merge_item[i];
			new_remaining_points_merged_plane[i] = new bool[total_plane_size];
			for (unsigned int k = 0; k < total_plane_size; k++)
			{
				// copy good plane information to new 
				if (k < good_voxel_plane_size)
				{
					new_remaining_points_merged_plane[i][k] = bad_voxel_it->remaining_points_merged_plane[k];
				}
				else
				{
					new_remaining_points_merged_plane[i][k] = false;
				}
			}
			delete[] bad_voxel_it->remaining_points_merged_plane;
			bad_voxel_it->remaining_points_merged_plane = new_remaining_points_merged_plane[i];
		}

		delete[] new_remaining_points_merged_plane;
		new_remaining_points_merged_plane = NULL;
		return;
	}

	//find all the voxels who are not merged into planes
	int unmerged_voxel_cnt = 0;
	for (unsigned int i = 0; i < plane_voxel_array.size; i++)
	{
		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[i];
		if (!plane_voxel->is_being_merged) unmerged_voxel_cnt++;
	}
    
	log_info("all the unmerged voxel count =%d bad_voxel_size=%d", unmerged_voxel_cnt, bad_voxel_size);
	point_merged_voxel_size = unmerged_voxel_cnt;
	//bad_voxel_cnt need to be optimized to  reduce memory usage for scan data if have large amount of noise data,for example to discard the voxels who have few occupied neighbours
	bad_voxel_merge_item = new BadVoxelMergeOutpuItem[unmerged_voxel_cnt];

	voxel_idx_to_bad_voxel = new unsigned int[plane_voxel_array.size];
	unsigned int bad_voxel_cnt = 0;
	for (unsigned int i = 0; i < plane_voxel_array.size; i++)
	{
		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[i];

		if (plane_voxel->is_being_merged)   //  bad_voxel_merge_item is only for all of non-overall-merged voxels
		{
			voxel_idx_to_bad_voxel[i] = std::numeric_limits<unsigned int>::max();
			continue;
		}	
		bad_voxel_merge_item[bad_voxel_cnt].bad_voxel_idx = i;   // first init the bad_voxel_idx
		voxel_idx_to_bad_voxel[i] = bad_voxel_cnt;
		if (bad_voxel_cnt >= point_merged_voxel_size) log_error("bad_voxel_cnt=%d point_merged_voxel_size =%d \n\r", bad_voxel_cnt, point_merged_voxel_size);
		bad_voxel_cnt++;
	}

	// here bad_voxel_cnt include point_merged_voxel_size and unmerged voxels whose is_overall_merged is true(good or pseudobad voxels)
	log_info("bad_voxel_cnt=%d", bad_voxel_cnt);
	// initialize the elements of  bad_voxel_merge_item
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(bad_voxel_cnt); i++)
	{
		BadVoxelMergeOutpuItem* bad_voxel_merge_it = &bad_voxel_merge_item[i];
		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[bad_voxel_merge_it->bad_voxel_idx];

		bad_voxel_merge_it->voxel_point_size = plane_voxel->points.size;
		bad_voxel_merge_it->closest_plane_idx = new unsigned int[plane_voxel->points.size];
#ifdef VOXEL_NEAREST_APPROACH_MERGE
		bad_voxel_merge_it->best_merge_point_id = new unsigned int[plane_voxel->points.size];
		bad_voxel_merge_it->best_merge_plane_voxel = new BadVoxelMergeOutpuItem*[plane_voxel->points.size];
		bad_voxel_merge_it->best_merge_plane_id = new unsigned int[plane_voxel->points.size];
#endif
		bad_voxel_merge_it->smallest_dist = new float[plane_voxel->points.size];
		bad_voxel_merge_it->point_merged_flag = new bool[plane_voxel->points.size];
		// remaining_points_merged_plane size is equel to all the plane generated, will update resources after identify parent of pseudobad voxels
		bad_voxel_merge_it->remaining_points_merged_plane = new bool[good_voxel_plane_size]; 
		bad_voxel_merge_it->num_of_points_merged = 0;
		bad_voxel_merge_it->same_normal_plane_cnt = 0;   
		bad_voxel_merge_it->points_group = NULL;      
		bad_voxel_merge_it->being_in_group_plane = NULL; 
		//bad_voxel_merge_it->being_in_extended_plane = NULL; 
		bad_voxel_merge_it->is_being_merged = false;
		bad_voxel_merge_it->is_plane_connected = NULL;
		bad_voxel_merge_it->is_multi_plane_connected = NULL;

		for (unsigned int k = 0; k < good_voxel_plane_size; k++)
		{
			//bad_voxel_merge_it->remaining_with_same_normal[k] = false;
			bad_voxel_merge_it->remaining_points_merged_plane[k] = false;
		}

		for (unsigned int k = 0; k < plane_voxel->points.size; k++)
		{
			bad_voxel_merge_it->closest_plane_idx[k] = std::numeric_limits<unsigned int>::max();
#ifdef VOXEL_NEAREST_APPROACH_MERGE
			bad_voxel_merge_it->best_merge_plane_id[k] = std::numeric_limits<unsigned int>::max();
			bad_voxel_merge_it->best_merge_point_id[k] = std::numeric_limits<unsigned int>::max();
			bad_voxel_merge_it->best_merge_plane_voxel[k] = nullptr;
#endif
			bad_voxel_merge_it->smallest_dist[k] = std::numeric_limits<float>::infinity();
			bad_voxel_merge_it->point_merged_flag[k] = false;
		}

		for (unsigned int k = 0; k < 26; k++)
		{
			bad_voxel_merge_it->neighbor_connected[k] = false;
		}

		bad_voxel_merge_it->is_remainer_occupied = true; // true show this voxel has still have points not being merged
	}
	return;
}

// Find out the parent of all bad voxels and pseudobad voxels
void PlaneSegmentation::IdentifyParentOfBadVoxelsWithRef(bool is_first)
{
	//unsigned int parent_voxel_idx;				// parent voxel index to occupied voxel array

	float min_point_plane_dist = plane_seg_params.plane_seg_thresholds.THRESHOLD_MIN_DIST_OF_POINT2VOXEL;
	
	if (is_first)
	{
		min_point_plane_dist = plane_seg_params.plane_seg_thresholds.THRESHOLD_MIN_DIST_OF_POINT2VOXEL / 2;
		min_point_plane_dist = plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_VOXEL < min_point_plane_dist ? plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_VOXEL : min_point_plane_dist;
	}

	float min_normal_diff = static_cast<float>(std::cos(plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_NORMAL_ANGEL_OF_EDGE_POINT * M_PI / 180));

	log_debug("min_point_plane_dist is %f in IdentifyParentOfBadVoxels", min_point_plane_dist);
#ifndef VOXEL_NEAREST_APPROACH_MERGE
	unsigned int flat_voxel_plane_size = good_voxel_plane_size + pseudobad_voxel_plane_size;
#endif
	// Identify parent voxel in all real bad voxels
//#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(point_merged_voxel_size); i++)
	{
		BadVoxelMergeOutpuItem* bad_voxel_merge_it = &bad_voxel_merge_item[i];
		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[bad_voxel_merge_it->bad_voxel_idx];

		if (plane_voxel->is_being_merged&&plane_voxel->is_overall_merged) continue;
		if (!bad_voxel_merge_it->is_remainer_occupied) continue;
#ifndef VOXEL_NEAREST_APPROACH_MERGE
		unsigned int voxel_identify_plane_cnt = 0; // all points found in current voxels
		std::vector<unsigned int> voxel_identify_plane_cnt_list(plane_merge_out.size,0);// all points found in current voxels by plane list
#endif
		for (unsigned int k = 0; k < plane_voxel->points.size; k++)
		{
			unsigned int point_idx = plane_voxel->points.point_idx[k];
			//if (bad_voxel_merge_it->point_merged_flag[k]) continue;
			if (bad_voxel_merge_it->closest_plane_idx[k] != std::numeric_limits<unsigned int>::max()) continue; //if this point is not identified  to be of a plane
			Point3f Point = pt_cloud_xyz.points[point_idx];
			Point3f Point_normal = pt_cloud_normal.points[point_idx];
			unsigned int plane_idx = -1;

			for (unsigned int j = 0; j < 26; j++)
			{
				NeighborItem* neighbor_item = &plane_voxel->neighbors[j];
				//if (!neighbor_item->is_occupied) continue;
				if (!neighbor_item->is_connected) continue;
				PlaneVoxelItem* neighbor_plane_voxel = &plane_voxel_array.voxels[neighbor_item->voxel_idx];
				PlaneMergeItem* neighbor_plane_merge_item = &plane_merge_element[neighbor_item->voxel_idx];
				plane_idx = -1;
				//check if neighbour is good or psudobad voxel who have parent voxel
				if ((neighbor_plane_voxel->is_overall_merged)&&(neighbor_plane_voxel->is_being_merged))
				{
					unsigned int cur_parent_voxel_idx = neighbor_plane_merge_item->parent_voxel_idx[0];
					plane_idx = voxel_to_plane_idx[cur_parent_voxel_idx];
					if (plane_idx >= plane_merge_out.size)
					{
						// log for debug, it is possible, while in multilple-thread environment, because have overlap for voxel's is_overall_merged 
						log_debug("plane_idx =%d exceed the size of total plane =%d ", plane_idx, plane_merge_out.size);
						continue;
					}
					//check if have points is connected with reference plane
					if (!bad_voxel_merge_it->is_plane_connected[plane_idx]) continue;
					unsigned int start_plane_idx = 0;
					/*if (!is_good_voxel_plane)
					{
						start_plane_idx = good_voxel_plane_size;
						// for speed, do not need find again in planes generated by good voxels;						
					}*/

					if ((plane_idx >= start_plane_idx) && (plane_idx < plane_merge_out.size))
					{						
						// check if point distance to neighbours parent plane is less than predefined ,and find the smalles distance plane in all the neighbors ,record the plane merge flag				
						Point3f plane_normal = plane_merge_out.planes[plane_idx].plane_normal;
						Point3f plane_center = plane_merge_out.planes[plane_idx].plane_center;

						if (!is_first)
						{
							float point_normal_diff = std::fabs(ComputeVectorDotProduct<float>(Point_normal, plane_normal));
							if (point_normal_diff < min_normal_diff) continue;
						}
#ifdef VOXEL_NEAREST_APPROACH_MERGE
						float p2plane_dist = ComputePointToPlaneDist<float>(Point, plane_normal, plane_center);
						float new_dist = std::numeric_limits<float>::infinity();
						//for (int mk = 0; mk < plane_merge_out.planes[plane_idx].extended_part_points.size; mk++)
						//{
						//	Point3f delta_point = Point - pt_cloud_xyz.points[plane_merge_out.planes[plane_idx].extended_part_points.point_idx[mk]];
						//	float tmp_dist = std::fabs(delta_point.x) + std::fabs(delta_point.y) + std::fabs(delta_point.z);
						//	if (tmp_dist < new_dist)
						//		new_dist = tmp_dist;
						//}

						PlaneVoxelItem* tmp_voxel = &plane_voxel_array.voxels[neighbor_item->voxel_idx];
						for (int mk = 0; mk < tmp_voxel->points.size; mk++)
						{
							Point3f delta_point = Point - pt_cloud_xyz.points[tmp_voxel->points.point_idx[mk]];
							float tmp_dist = std::fabs(delta_point.x) + std::fabs(delta_point.y) + std::fabs(delta_point.z);
							if (tmp_dist < new_dist)
								new_dist = tmp_dist;
						}
						if ((new_dist < bad_voxel_merge_it->smallest_dist[k]) && (p2plane_dist < min_point_plane_dist))
#else
						float new_dist = ComputePointToPlaneDist<float>(Point, plane_normal, plane_center);
						if ((new_dist < bad_voxel_merge_it->smallest_dist[k]) && (new_dist < min_point_plane_dist))
#endif
						{
							bad_voxel_merge_it->smallest_dist[k] = new_dist;
							bad_voxel_merge_it->closest_plane_idx[k] = plane_idx;
#ifndef VOXEL_NEAREST_APPROACH_MERGE
							voxel_identify_plane_cnt_list[plane_idx]++;
							voxel_identify_plane_cnt++;
#endif
						}
					}
				}
				else
				{
#ifdef VOXEL_NEAREST_APPROACH_MERGE
					if (voxel_idx_to_bad_voxel[neighbor_item->voxel_idx] != std::numeric_limits<unsigned int>::max())
					{
						float new_dist = std::numeric_limits<float>::infinity();
						PlaneVoxelItem* tmp_voxel = &plane_voxel_array.voxels[bad_voxel_merge_item[voxel_idx_to_bad_voxel[neighbor_item->voxel_idx]].bad_voxel_idx];
						unsigned int merge_plane_idx = std::numeric_limits<unsigned int>::max();
						for (int mk = 0; mk < tmp_voxel->points.size; mk++)
						{
							Point3f delta_point = Point - pt_cloud_xyz.points[tmp_voxel->points.point_idx[mk]];
							float tmp_dist = std::fabs(delta_point.x) + std::fabs(delta_point.y) + std::fabs(delta_point.z);
							if (tmp_dist < new_dist)
							{
								new_dist = tmp_dist;
								merge_plane_idx = mk;
							}
						}

						auto& map_plane_idx = bad_voxel_merge_item[voxel_idx_to_bad_voxel[neighbor_item->voxel_idx]].best_merge_plane_id[merge_plane_idx];
						if (new_dist < bad_voxel_merge_it->smallest_dist[k] && ((map_plane_idx == std::numeric_limits<unsigned int>::max() && bad_voxel_merge_it->closest_plane_idx[k] == std::numeric_limits<unsigned int>::max()) || map_plane_idx != bad_voxel_merge_it->closest_plane_idx[k]))
						{
							//if (point_idx == 897780)
							//{
							//	std::cout << "## 897780 : " << map_plane_idx << "\t" << bad_voxel_merge_it->closest_plane_idx[k] << "\t" 
							//		<< bad_voxel_merge_it->closest_plane_idx[merge_plane_idx] << "\t"
							//		<< bad_voxel_merge_item[voxel_idx_to_bad_voxel[neighbor_item->voxel_idx]].closest_plane_idx[k] << "\t"
							//		<< j << std::endl;
							//}
								
							bad_voxel_merge_it->smallest_dist[k] = new_dist;
							//bad_voxel_merge_it->closest_plane_idx[k] = plane_idx;
							bad_voxel_merge_it->best_merge_plane_voxel[k] = &bad_voxel_merge_item[voxel_idx_to_bad_voxel[neighbor_item->voxel_idx]];
							bad_voxel_merge_it->best_merge_point_id[k] = merge_plane_idx;
							bad_voxel_merge_it->best_merge_plane_id[k] = bad_voxel_merge_item[voxel_idx_to_bad_voxel[neighbor_item->voxel_idx]].closest_plane_idx[merge_plane_idx];
						}
					}
#else
					// when neighbor voxel is also point-to-plane mode, check its connected 					
					//if (!bad_voxel_merge_it->neighbor_connected[j]) continue; 
					// if neigbour voxel is not is_overall_merged, the points in the neighbour maybe the 2 kind of case:
					//1) directly be merged the in the plane by good or pseudo-bad voxel growing 
					//2) not being merged 
					unsigned int neighbor_bad_voxel_idx = voxel_idx_to_bad_voxel[neighbor_item->voxel_idx];
					BadVoxelMergeOutpuItem* neighbor_bad_voxel_merge_it = &bad_voxel_merge_item[neighbor_bad_voxel_idx];
					//if (neighbor_bad_voxel_merge_it->same_normal_plane_cnt == 0) continue; //skip  to next if neighbour have no points in plane
					unsigned int start_plane_idx = 0;
					//if (!is_good_voxel_plane) start_plane_idx = good_voxel_plane_size; // if identify in pseudobad planes
					for (unsigned int m = start_plane_idx; m < plane_merge_out.size; m++)
					{
						//check if have points is connected with reference plane
						if (!bad_voxel_merge_it->is_plane_connected[m]) continue;

						if (m < flat_voxel_plane_size)
						{
							//if (is_first) continue;
							if (!neighbor_bad_voxel_merge_it->remaining_points_merged_plane[m]) continue;
						}
						else if (neighbor_bad_voxel_merge_it->being_in_group_plane != NULL)
						{
							unsigned int group_plane_idx = m - flat_voxel_plane_size;
							if (!neighbor_bad_voxel_merge_it->being_in_group_plane[group_plane_idx]) continue;
						}
						plane_idx = m;
						Point3f plane_normal = plane_merge_out.planes[plane_idx].plane_normal;
						Point3f plane_center = plane_merge_out.planes[plane_idx].plane_center;
						if (!is_first)
						{
							float point_normal_diff = std::fabs(ComputeVectorDotProduct<float>(Point_normal, plane_normal));
							if (point_normal_diff < min_normal_diff) continue;
						}
						float new_dist = ComputePointToPlaneDist<float>(Point, plane_normal, plane_center);
						if ((new_dist < bad_voxel_merge_it->smallest_dist[k]) && (new_dist < min_point_plane_dist))
						{
							bad_voxel_merge_it->smallest_dist[k] = new_dist;
							bad_voxel_merge_it->closest_plane_idx[k] = plane_idx;
							voxel_identify_plane_cnt_list[plane_idx]++;
							voxel_identify_plane_cnt++;
						}
					}
#endif
				}
			}
		}
#ifndef VOXEL_NEAREST_APPROACH_MERGE
		if (voxel_identify_plane_cnt != 0)
		{
			for (unsigned int i = 0; i < plane_merge_out.size; i++)
			{
				if (voxel_identify_plane_cnt_list[i] > 0)
				{
					if (i < flat_voxel_plane_size)
					{
						bad_voxel_merge_it->remaining_points_merged_plane[i] = true;
					}
					else if (bad_voxel_merge_it->being_in_group_plane != NULL)
					{
						unsigned int group_plane_idx = i - flat_voxel_plane_size;
						bad_voxel_merge_it->being_in_group_plane[group_plane_idx] = true;
					}
					bad_voxel_merge_it->same_normal_plane_cnt++;

					/*if (!is_good_voxel_plane && (i < good_voxel_plane_size))
					{
						// it is impossible , should have bug
						log_error("voxel %d find point idx in good plane %d while is_good_voxel_plane %d ", bad_voxel_merge_it->bad_voxel_idx, i, is_good_voxel_plane);
					}*/
				}
			}
		}

		voxel_identify_plane_cnt_list.clear();
		voxel_identify_plane_cnt_list.shrink_to_fit();
#endif
	}

#if 0
	//refine plane index with self voxel points
	//for (int i = 0; i < static_cast<int>(point_merged_voxel_size); i++)
	//{
	//	BadVoxelMergeOutpuItem* bad_voxel_merge_it = &bad_voxel_merge_item[i];
	//	PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[bad_voxel_merge_it->bad_voxel_idx];

	//	bool is_plane_2 = false;
	//	bool is_plane_6 = false;
	//	for (unsigned int k = 0; k < plane_voxel->points.size; k++)
	//	{
			//if (bad_voxel_merge_it->closest_plane_idx[k] == 2)
			//	is_plane_2 = true;
			//else if (bad_voxel_merge_it->closest_plane_idx[k] == 6)
			//	is_plane_6 = true;
			//if (bad_voxel_merge_it->closest_plane_idx[k] != std::numeric_limits<unsigned int>::max() && bad_voxel_merge_it->smallest_dist[k] > point_density * 2)
			//{
			//	unsigned int point_idx = plane_voxel->points.point_idx[k];
			//	Point3f Point = pt_cloud_xyz.points[point_idx];
			//	for (unsigned int t = 0; t < plane_voxel->points.size; t++)
			//	{
			//		if (t != k)
			//		{
			//			if (bad_voxel_merge_it->closest_plane_idx[t] != std::numeric_limits<unsigned int>::max())
			//			{
			//				Point3f delta_point = Point - pt_cloud_xyz.points[plane_voxel->points.point_idx[t]];
			//				float tmp_dist = std::fabs(delta_point.x) + std::fabs(delta_point.y) + std::fabs(delta_point.z);
			//				if (tmp_dist < bad_voxel_merge_it->smallest_dist[t])
			//				{
			//					bad_voxel_merge_it->smallest_dist[k] = tmp_dist;
			//					bad_voxel_merge_it->closest_plane_idx[k] = bad_voxel_merge_it->closest_plane_idx[t];
			//				}
			//			}
			//		}
			//	}
			//}
		//}
		//if (is_plane_2 && is_plane_6)
		//{
		//	ofstream fout("points_merge.xyz");
		//	for (int p = 0; p < plane_voxel->points.size; p++)
		//	{
		//		std::cout << "pppt index: " << plane_voxel->points.point_idx[p] << std::endl;
		//		Point3f pt = pt_cloud_xyz.points[plane_voxel->points.point_idx[p]];
		//		fout << pt.x << '\t' << pt.y << '\t' << pt.z << std::endl;
		//	}
		//	fout.close();
		//	std::cout << "merge plane 2&6: " << plane_voxel->points.size << "\t" << bad_voxel_merge_it->bad_voxel_idx << std::endl;
		//}
	//}
#endif
	return;
}

unsigned int PlaneSegmentation::FindBestPlaneIDToMerge(BadVoxelMergeOutpuItem* voxel, unsigned int k)
{
	BadVoxelMergeOutpuItem* tmp_voxel = voxel;
	std::set<BadVoxelMergeOutpuItem*> voxel_list;
	unsigned int id = k;
	unsigned int last_plane = std::numeric_limits<unsigned int>::max();
	Point3f point = pt_cloud_xyz.points[plane_voxel_array.voxels[tmp_voxel->bad_voxel_idx].points.point_idx[id]];

	while (tmp_voxel)
	{
		if (voxel_list.find(tmp_voxel) != voxel_list.end())
		{
			return std::numeric_limits<unsigned int>::max();
		}
		voxel_list.insert(tmp_voxel);
		if (tmp_voxel->closest_plane_idx[id] == std::numeric_limits<unsigned int>::max())
		{
			if (tmp_voxel->best_merge_plane_id[id] != std::numeric_limits<unsigned int>::max())
			{
				last_plane = tmp_voxel->best_merge_plane_id[id];
				break;
			}
			else
			{
				unsigned int dd = tmp_voxel->best_merge_point_id[id];
				tmp_voxel = tmp_voxel->best_merge_plane_voxel[id];
				id = dd;
			}
		}
		else
		{
			last_plane = tmp_voxel->closest_plane_idx[id];
			break;
		}
	}
	if (last_plane != std::numeric_limits<unsigned int>::max())
	{
		Point3f plane_normal = plane_merge_out.planes[last_plane].plane_normal;
		Point3f plane_center = plane_merge_out.planes[last_plane].plane_center;
		float p2plane_dist1 = ComputePointToPlaneDist<float>(point, plane_normal, plane_center);
		Point3f that_point = pt_cloud_xyz.points[plane_voxel_array.voxels[tmp_voxel->bad_voxel_idx].points.point_idx[id]];
		float p2plane_dist2 = ComputePointToPlaneDist<float>(that_point, plane_normal, plane_center);
		float threshold = plane_seg_params.plane_seg_thresholds.THRESHOLD_MIN_DIST_OF_POINT2VOXEL * 0.5;
		if (p2plane_dist1 < threshold && p2plane_dist2 < threshold)
			return last_plane;
	}
	return std::numeric_limits<unsigned int>::max();
}

// Merge all edge points of planes generated by good or pseudobad voxels  into its similar plane
void PlaneSegmentation::MergeBadVoxels()
{
	//unsigned int parent_voxel_idx;				// parent voxel index to occupied voxel array

	unsigned int *num_of_points_list_by_plane = new unsigned int[plane_merge_out.size];  // record number of points being merged in plane
	memset(num_of_points_list_by_plane, 0, sizeof(unsigned int) * plane_merge_out.size);
	for (unsigned int i = 0; i < point_merged_voxel_size; i++)
	{
		BadVoxelMergeOutpuItem* bad_voxel_merge_it = &bad_voxel_merge_item[i];
		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[bad_voxel_merge_it->bad_voxel_idx];

		//if (plane_voxel->is_overall_merged) continue;
		if (!bad_voxel_merge_it->is_remainer_occupied) continue;
		if (plane_voxel->is_overall_merged&&plane_voxel->is_being_merged) continue;

		// get number of all the bad voxel points for each plane
		for (unsigned int k = 0; k < plane_voxel->points.size; k++)
		{
			if (bad_voxel_merge_it->point_merged_flag[k]) continue;
#ifdef VOXEL_NEAREST_APPROACH_MERGE				
			unsigned int plane_idx = FindBestPlaneIDToMerge(bad_voxel_merge_it, k);
#else
			unsigned int plane_idx = bad_voxel_merge_it->closest_plane_idx[k];
#endif
			if (plane_idx < plane_merge_out.size)
			{
				num_of_points_list_by_plane[plane_idx]++;
#ifdef VOXEL_NEAREST_APPROACH_MERGE
				unsigned int flat_size = (good_voxel_plane_size + pseudobad_voxel_plane_size);
				if (plane_idx < flat_size)
				{
					bad_voxel_merge_it->remaining_points_merged_plane[plane_idx] = true;
				}
				else if (bad_voxel_merge_it->being_in_group_plane != NULL)
				{
					unsigned int group_plane_idx = plane_idx - flat_size;
					bad_voxel_merge_it->being_in_group_plane[group_plane_idx] = true;
				}
				bad_voxel_merge_it->same_normal_plane_cnt++;
#endif
			}

		}
	}
	int start_plane_idx = 0;
	//if (!is_good_voxel_plane) start_plane_idx = good_voxel_plane_size;
#pragma omp parallel for
	for (int i = start_plane_idx; i < static_cast<int>(plane_merge_out.size); i++)
	{
		PlaneMergeOutputItem* plane_it = &plane_merge_out.planes[i];
		plane_it->points.size = num_of_points_list_by_plane[i];
		plane_it->total_point_cnt += plane_it->points.size;
	}

	// merge the points of all the bad voxels to plane and update sums of all the planes
#pragma omp parallel for
	for (int i = start_plane_idx; i < static_cast<int>(plane_merge_out.size); i++)
	{
		PlaneMergeOutputItem* plane_it = &plane_merge_out.planes[i];
		num_of_points_list_by_plane[i] = 0;
		if (plane_it->points.size == 0) continue;
		plane_it->points.point_idx = new unsigned int[plane_it->points.size];
		memset(plane_it->points.point_idx, 0, sizeof(unsigned int)*plane_it->points.size);
		unsigned int parent_voxel_idx = plane_it->parent_voxel_idx;

		// current point index pointer
		unsigned int* current_plane_point_ptr = plane_it->points.point_idx;
		for (unsigned int j = 0; j < point_merged_voxel_size; j++)
		{
			BadVoxelMergeOutpuItem* bad_voxel_merge_it = &bad_voxel_merge_item[j];
			PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[bad_voxel_merge_it->bad_voxel_idx];

			if (!bad_voxel_merge_it->is_remainer_occupied) continue;
			if (plane_voxel->is_overall_merged&&plane_voxel->is_being_merged) continue;

			for (unsigned int k = 0; k < bad_voxel_merge_it->voxel_point_size; k++)
			{
				if (bad_voxel_merge_it->point_merged_flag[k]) continue;
#ifdef VOXEL_NEAREST_APPROACH_MERGE
				bad_voxel_merge_it->closest_plane_idx[k] = FindBestPlaneIDToMerge(bad_voxel_merge_it, k);
				if (bad_voxel_merge_it->closest_plane_idx[k] == i && num_of_points_list_by_plane[i] < plane_it->points.size)
#else
				if (bad_voxel_merge_it->closest_plane_idx[k] == i)
#endif
				{
					//unsigned int point_idx = plane_voxel_array.voxels[bad_voxel_merge_it->bad_voxel_idx].points.point_idx[k];

					// merge the points of this bad voxel to plane
					*current_plane_point_ptr = plane_voxel->points.point_idx[k];
					current_plane_point_ptr++;

					num_of_points_list_by_plane[i]++;
					//bad_voxel_merge_item[j].remaining_points_merged_plane[i] = true;
					bad_voxel_merge_it->num_of_points_merged++;
					// update sums of this plane
					//VoxelMergeBaseClass occupied_voxel_grid(&plane_voxel_array.voxels[parent_voxel_idx], &plane_seg_params.plane_seg_thresholds);
					Point3f Point = pt_cloud_xyz.points[plane_voxel->points.point_idx[k]];
					//occupied_voxel_grid.Push(Point);
					PushPoint(&plane_it->sums,Point);
					bad_voxel_merge_it->point_merged_flag[k] = true;
				}
			}

			if (bad_voxel_merge_it->num_of_points_merged > 0)
			{
				plane_voxel->is_being_merged = true;
				bad_voxel_merge_it->is_being_merged = true;
				// if have points merged in plane, mark voxel's is_overall_merged false
				plane_voxel->is_overall_merged = false;
			}
		}

		if (num_of_points_list_by_plane[i] != plane_it->points.size) log_debug("plane %d bad voxel merge size =%d is not equal to identify size =%d ",i, num_of_points_list_by_plane[i], plane_it->points.size);
		//update normals mse and plane center of all the planes
		//VoxelMergeBaseClass occupied_voxel_grid(&plane_voxel_array.voxels[parent_voxel_idx], &plane_seg_params.plane_seg_thresholds);
		//occupied_voxel_grid.Compute(plane_it->total_point_cnt, plane_it->sums, plane_it->plane_normal, plane_it->plane_center);
		GetPlaneDistMse(&plane_merge_out.planes[i], plane_it->plane_mse);
	}

	if (num_of_points_list_by_plane != NULL){
		delete[] num_of_points_list_by_plane;
		num_of_points_list_by_plane = NULL;
	}
	return;
}

#if 0
void PlaneSegmentation::MergeBadVoxelsByNormalDiff()
{
	for (unsigned int i = 0; i < plane_merge_out.size; i++)
	{
		PlaneMergeOutputItem *plane_it = &plane_merge_out.planes[i];
		//unsigned int plane_merge_out.planes[i].points.size = num_of_points_list_by_normal[i];
		if (plane_it->same_normal_points.size == 0) continue;
		plane_it->same_normal_points.point_idx = new unsigned int[plane_it->same_normal_points.size];

		unsigned int current_point_idx = 0;  // bad voxel point index whose is being merged into current plane
		for (unsigned int j = 0; j < point_merged_voxel_size; j++)
		{
			BadVoxelMergeOutpuItem *bad_voxel_merge_it = &bad_voxel_merge_item[j];
			PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[bad_voxel_merge_it->bad_voxel_idx];
			if (bad_voxel_merge_it->remaining_with_same_normal[i][0])
			{
				for (unsigned int k = 0; k < plane_voxel->points.size; k++)
				{
					if (bad_voxel_merge_it->point_merged_flag[k]) continue;
					if (bad_voxel_merge_it->closest_plane_idx[k] == i)
					{
						unsigned int point_idx = plane_voxel->points.point_idx[k];
						Point3f Point = pt_cloud_xyz.points[point_idx].point;
						PushPoint(&plane_it->sums, Point);
						plane_it->same_normal_points.point_idx[current_point_idx] = point_idx;
						bad_voxel_merge_it->point_merged_flag[k] = true;
						bad_voxel_merge_it->num_of_points_merged++;
						current_point_idx++;
					}
				}

				if (current_point_idx > plane_it->same_normal_points.size) log_error("plane %d bad voxel %d current_points_idx %d is large than num of same_normal_points found before =%d", i, j, current_point_idx, plane_it->same_normal_points.size);
			}
		}
		plane_it->total_point_cnt += plane_it->same_normal_points.size;
		float eigen_mse;
		MathOperation::Compute(plane_it->total_point_cnt,plane_it->sums,plane_it->plane_normal,plane_it->plane_center,eigen_mse);
		GetPlaneDistMse(plane_it, plane_it->plane_mse);
		if (current_point_idx != plane_it->same_normal_points.size) log_error("plane %d current_points_idx %d is not equal to num of same_normal_points found before =%d", i, current_point_idx, plane_it->same_normal_points.size);
	}

	return;
}
#endif

// find same plane
void PlaneSegmentation::IdentifySameNormalPlanes(bool**is_same_plane, unsigned int plane_size)
{
	float min_normal_diff = static_cast<float>(std::cos(plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_NORMAL_ANGEL_OF_2PLANE * M_PI / 180));
	float min_plane_dist = plane_seg_params.plane_seg_thresholds.THRESHOLD_MIN_DIST_OF_2PLANE; 
	// if the parent voxel type is real bad voxel, the threshod may be select more than good or pesuod bad voxel plane
	float min_bad_voxel_plane_normal_diff = static_cast<float > (std::cos(plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_NORMAL_ANGEL_OF_2PLANE * 2 * M_PI / 180));
	float min_bad_voxel_plane_dist = plane_seg_params.plane_seg_thresholds.THRESHOLD_MIN_DIST_OF_2PLANE*2;

	// min number if a good plane whose normal is accurate
	float threshold_of_good_plane_voxel_size = 20;
	//log_info("min_normal_diff =%f min_plane_dist=%f", std::acos(min_normal_diff)*180/M_PI, min_plane_dist);
	//log_info("min_bad_voxel_plane_normal_diff =%f min_bad_voxel_plane_dist=%f", std::acos(min_bad_voxel_plane_normal_diff) * 180 / M_PI, min_bad_voxel_plane_dist);

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(plane_size); i++)
	{
		Point3f plane_normal, plane_center, other_plane_normal, other_plane_center;
		SumforCovariance sums, other_sums;
		unsigned int point_cnt_of_plane, point_cnt_of_other_plane;
		unsigned int parent_voxel_idx, other_parent_voxel_idx;
		unsigned int cur_good_voxel_size, cur_other_good_voxel_size;
		PlaneVoxelType parent_voxel_type, other_parent_voxel_type;
		float compare_dist;

		if (i < static_cast<int>(plane_merge_out.size))
		{
			PlaneMergeOutputItem * output_plane_it = &plane_merge_out.planes[i];
			plane_normal = output_plane_it->plane_normal;
			plane_center = output_plane_it->plane_center;
			AssignSums(&sums, &output_plane_it->sums);
			point_cnt_of_plane = output_plane_it->total_point_cnt;
			parent_voxel_idx = output_plane_it->parent_voxel_idx;
			parent_voxel_type = plane_voxel_array.voxels[parent_voxel_idx].voxel_type;	
			cur_good_voxel_size = output_plane_it->voxels.size;
		}
#ifdef INCLUDE_POINTS_GROUP_MERGING
		else if (points_group_plane_merge_out.size != 0)
		{

			unsigned int group_plane_idx = i - plane_merge_out.size;
			PlaneMergeOutputItem* output_plane_it = &points_group_plane_merge_out.planes[group_plane_idx];
			plane_normal = output_plane_it->plane_normal;
			plane_center = output_plane_it->plane_center;
			AssignSums(&sums, &output_plane_it->sums);
			point_cnt_of_plane = output_plane_it->points.size+ output_plane_it->extended_part_points.size;
			parent_voxel_type = REAL_BAD_VOXEL;
			cur_good_voxel_size = output_plane_it->voxels.size;
		}

#endif

		for (unsigned int j = 0; j < plane_size; j++)
		{
			if (j == i) continue;
			if (is_same_plane[i][j]) continue;
			if (j < plane_merge_out.size)
			{
				PlaneMergeOutputItem* other_output_plane_it = &plane_merge_out.planes[j];
				other_plane_normal = other_output_plane_it->plane_normal;
				other_plane_center = other_output_plane_it->plane_center;
				AssignSums(&other_sums, &other_output_plane_it->sums);
				point_cnt_of_other_plane = other_output_plane_it->total_point_cnt;
				other_parent_voxel_idx = other_output_plane_it->parent_voxel_idx;
				other_parent_voxel_type = plane_voxel_array.voxels[other_parent_voxel_idx].voxel_type;
				cur_other_good_voxel_size = other_output_plane_it->voxels.size;

			}
#ifdef INCLUDE_POINTS_GROUP_MERGING
			else if (points_group_plane_merge_out.size != 0)
			{
				unsigned int other_group_plane_idx = j - plane_merge_out.size;
				PlaneMergeOutputItem* other_output_plane_it = &points_group_plane_merge_out.planes[other_group_plane_idx];
				other_plane_normal = other_output_plane_it->plane_normal;
				other_plane_center = other_output_plane_it->plane_center;
				AssignSums(&other_sums, &other_output_plane_it->sums);
				point_cnt_of_other_plane = other_output_plane_it->points.size+ other_output_plane_it->extended_part_points.size;
				other_parent_voxel_type = REAL_BAD_VOXEL;
				cur_other_good_voxel_size =0;
			}
#endif


			float normal_sim = std::fabs(ComputeVectorDotProduct<float>(plane_normal, other_plane_normal));
			if (normal_sim > 1.0f) normal_sim = 1.0f;

			float other_plane_dist = ComputePointToPlaneDist<float>(plane_center, other_plane_normal, other_plane_center);
			float plane_dist = ComputePointToPlaneDist<float>(other_plane_center, plane_normal, plane_center);

			bool same_plane_condition = false;
			bool debug_condition = false;//((i == 1) && (j == 2)) || ((i == 6) && (j == 38))|| ((i == 2) && (j == 35))|| ((i == 2) && (j == 37))|| ((i == 2) && (j == 40));
			if ((parent_voxel_type == GOOD_VOXEL) && (parent_voxel_type == other_parent_voxel_type))
			{
				/* Generally escape the different planes distance  probably have small distance from center to plane  due to normal difference , we check the both distances of two planes */

				float plane_center_dist = ComputePointToPointDist<float>(plane_center, other_plane_center);
				if(plane_center_dist < (length_x_of_voxel+length_y_of_voxel+length_z_of_voxel)*2) 
				{
					same_plane_condition = (normal_sim > min_normal_diff) && (plane_dist < min_plane_dist) && (other_plane_dist < min_plane_dist);
				}
				else
				{
					same_plane_condition = (normal_sim > min_normal_diff) && ((plane_dist+ other_plane_dist)/2< min_plane_dist);
				}				
			}
			else
			{
				if ((parent_voxel_type > GOOD_VOXEL) && (other_parent_voxel_type > GOOD_VOXEL))
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
					bool second_plane_condition = (normal_sim > min_bad_voxel_plane_normal_diff) && (plane_dist < min_bad_voxel_plane_dist) && (other_plane_dist < min_bad_voxel_plane_dist);
					same_plane_condition = first_plane_condition||second_plane_condition;

				}
				else
				{
					compare_dist = parent_voxel_type <= other_parent_voxel_type ? plane_dist : other_plane_dist;
					if ((cur_good_voxel_size < threshold_of_good_plane_voxel_size) && (cur_other_good_voxel_size < threshold_of_good_plane_voxel_size))
					{
						same_plane_condition = (normal_sim > min_normal_diff) && (compare_dist < min_bad_voxel_plane_dist);
					}
					else
					{
						same_plane_condition = (normal_sim > min_normal_diff) && (compare_dist < min_plane_dist);
						if ((parent_voxel_type == REAL_BAD_VOXEL) || (other_parent_voxel_type == REAL_BAD_VOXEL))
						{
							same_plane_condition = (normal_sim > min_bad_voxel_plane_normal_diff) && (compare_dist < min_plane_dist);
						}
					}
				}
			}
			
			if (debug_condition)
			{
				log_info("plane %d and %d parent_voxel_type=%d, other_parent_voxel_type=%d ", i, j, parent_voxel_type, other_parent_voxel_type);
				log_info("plane %d and %d normal_sim=%f, plane_dist=%.10f other_plane_dist =%f compare_dist =%f", i, j, std::acos(normal_sim) * 180 / M_PI, plane_dist, other_plane_dist,compare_dist);
				log_info("plane %d and %d point_cnt_of_plane=%d, point_cnt_of_other_plane=%d", i, j, point_cnt_of_plane, point_cnt_of_other_plane);
				log_info("plane %d and %d cur_good_voxel_size=%d, other_good_voxel_size=%d", i, j, cur_good_voxel_size, cur_other_good_voxel_size);
			}

			if (same_plane_condition)
			{
				is_same_plane[i][j] = true;
				is_same_plane[j][i] = true;
#ifdef SAVE_OUTPUT_FILE_DEBUG
				if(debug_config_params.merge_same_plane_output_debug)
				log_info("plane[%d,%d] is same normal", i, j);
#endif
			}
		}
	}

}

//check if have connected plane for plane_merge_out
//there are three variables from that the connected plane can be find:
//1: remaining_points_merged_plane ,member of BadVoxelMergeOutpuItem record the plane informaiton while identify parent voxel of plane generated by good voxels
//2:is_same_with_pseudobad_parent_plane, member of BadVoxelMergeOutpuItem record the plane informaiton while identify parent voxel of plane generated by pseudobad voxels
//3: being_in_group_plane, member of BadVoxelMergeOutpuItem record the plane informaiton while identify  points group
void PlaneSegmentation::IdentifyConnectedPlanesFromNeigbors(bool**is_connected, unsigned int bad_voxel_idx, unsigned int plane_idx)
{
	BadVoxelMergeOutpuItem *bad_voxel_merge_it = &bad_voxel_merge_item[bad_voxel_idx];
	PlaneVoxelItem *plane_voxel = &plane_voxel_array.voxels[bad_voxel_merge_it->bad_voxel_idx];
	for (unsigned int k = 0; k < 26; k++)
	{
		NeighborItem *neighbor_it = &plane_voxel->neighbors[k];
		if (!neighbor_it->is_occupied) continue;
		PlaneVoxelItem *neighbor_plane_voxel = &plane_voxel_array.voxels[neighbor_it->voxel_idx];

		if ((neighbor_plane_voxel->is_overall_merged)&&(neighbor_plane_voxel->is_being_merged)) // if neighbour is_overall_merged is true , it have only one plane
		{
			unsigned int parent_voxel_idx = plane_merge_element[neighbor_it->voxel_idx].parent_voxel_idx[0];
			unsigned int neighbor_plane_idx = voxel_to_plane_idx[parent_voxel_idx];
			if (is_connected[plane_idx][neighbor_plane_idx]) continue;
			is_connected[plane_idx][neighbor_plane_idx] = true;
			is_connected[neighbor_plane_idx][plane_idx] = true;

			//bool debug_condition = ((plane_idx == 16) && (neighbor_plane_idx == 17)) || ((plane_idx == 12) && (neighbor_plane_idx == 18)) || ((plane_idx == 21) && (neighbor_plane_idx == 4)) || ((plane_idx == 12) && (neighbor_plane_idx == 20)) || ((plane_idx == 24) && (neighbor_plane_idx == 13)) || ((plane_idx == 24) && (neighbor_plane_idx == 15)) || ((plane_idx == 12) && (neighbor_plane_idx == 19));
			//if (debug_condition)  log_info("plane[%d,%d] is connected", plane_idx, neighbor_plane_idx);
		}
		else // if neighbour is_overall_merged is false , it may have more than one plane , must check all the points
		{
			unsigned int bad_voxel_idx = voxel_idx_to_bad_voxel[neighbor_it->voxel_idx];
			BadVoxelMergeOutpuItem *neighbor_bad_voxel_merge_it = &bad_voxel_merge_item[bad_voxel_idx];
			for (unsigned int m = 0; m < plane_merge_out.size; m++)
			{
				if (neighbor_bad_voxel_merge_it->remaining_points_merged_plane[m])
				{
					if (is_connected[plane_idx][m]) continue;
					is_connected[plane_idx][m] = true;
					is_connected[m][plane_idx] = true;
					//unsigned int j = plane_idx;
					//bool debug_condition = ((j == 16) && (m == 17)) || ((j == 12) && (m == 18)) || ((j == 21) && (m == 4)) || ((j == 20) && (m == 12)) || ((j == 24) && (m == 15))|| ((j == 24) && (m == 13));
					//if (debug_condition)  log_info("plane[%d,%d] is connected", plane_idx, m);					
				}
			}

#ifdef INCLUDE_POINTS_GROUP_MERGING
			for (unsigned int m = 0; m < points_group_plane_merge_out.size; m++)
			{
				unsigned int current_plane_idx = plane_merge_out.size + m;
				if (neighbor_bad_voxel_merge_it->being_in_group_plane[m])
				{
					if (is_connected[plane_idx][current_plane_idx]) continue;
					is_connected[plane_idx][current_plane_idx] = true;
					is_connected[current_plane_idx][plane_idx] = true;
				}
			}
#endif
		}
	}

}
void PlaneSegmentation::IdentifyConnectedPlanes(bool**is_connected, unsigned int plane_size)
{

	if (bad_voxel_merge_item == NULL) return;  // confirm the bad voxels  are initialized
	// check if have connected plane for plane_merge_out 
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(point_merged_voxel_size); i++)
	{
		BadVoxelMergeOutpuItem* bad_voxel_merge_it = &bad_voxel_merge_item[i];

		//if (bad_voxel_merge_it->same_normal_plane_cnt == 0) continue; 

		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[bad_voxel_merge_it->bad_voxel_idx];

		for (unsigned int j = 0; j < plane_merge_out.size; j++)
		{
			if (!bad_voxel_merge_it->remaining_points_merged_plane[j]) continue;
			//if ((!bad_voxel_merge_it->is_same_with_pseudobad_parent_plane[j]) && (!bad_voxel_merge_it->remaining_points_merged_plane[j])) continue;
			// if have points in plane , check if the neighbour's plane, if have the other plane ,then these plane should be connected
			IdentifyConnectedPlanesFromNeigbors(is_connected, i, j);
		}
#ifdef INCLUDE_POINTS_GROUP_MERGING
		for (unsigned int j = 0; j < points_group_plane_merge_out.size; j++)
		{
			unsigned plane_idx = j + plane_merge_out.size;
			if (!bad_voxel_merge_it->being_in_group_plane[j]) continue;
			// if have points in group plane , check if the neighbour's plane, if have the other plane ,then these plane should be connected
			IdentifyConnectedPlanesFromNeigbors(is_connected, i, plane_idx);
		}
#endif
	}

	// check if have connected plane for plane_merge_out planes with points_group_plane_merge_out planes
#if 0//def INCLUDE_POINTS_GROUP_MERGING
	for (unsigned int i = 0; i < points_group_plane_merge_out.size; i++)
	{
		PointsGroupParentPlaneItem* group_plane = &points_group_plane_merge_out.planes[i];
		unsigned int plane_idx = i + plane_merge_out.size;
		for (unsigned int j = 0; j < group_plane->voxels.size; j++)
		{
			unsigned int voxel_idx = group_plane->voxels.voxel_idx[j];
			unsigned int bad_voxel_idx = voxel_idx_to_bad_voxel[voxel_idx];
			if (bad_voxel_idx > plane_voxel_array.size)
			{
				log_debug("bad_voxel_idx %d exceed size of occupied voxels", bad_voxel_idx, plane_voxel_array.size);
				continue;
			}

			IdentifyConnectedPlanesFromNeigbors(is_connected, bad_voxel_idx, plane_idx);

		}
	}
#endif
	// check if have connected plane in the pseudo bad voxels 
	/*for (int i = 0; i < plane_voxel_array.size; i++)
	{
		PlaneVoxelItem *plane_voxel = &plane_voxel_array.voxels[i];
		if ((plane_voxel->voxel_type != PSEUDO_BAD_VOXEL)|| (!plane_voxel->is_overall_merged))  continue;

		for (unsigned int k = 0; k < 26; k++)
		{
			NeighborItem *neighbor_it = &plane_voxel->neighbors[k];
			if (!neighbor_it->is_occupied) continue;
			PlaneVoxelItem *neighbor_plane_voxel = &plane_voxel_array.voxels[neighbor_it->voxel_idx];
			unsigned int parent_voxel_idx = plane_merge_element[i].parent_voxel_idx[0];
			unsigned int plane_idx = voxel_to_plane_idx[parent_voxel_idx];
			bool dbg_con = (plane_idx == 1) || (plane_idx == 2);
			if (neighbor_plane_voxel->is_overall_merged) // if neighbour is_overall_merged is true , it have only one plane
			{
				unsigned int neighbor_parent_voxel_idx = plane_merge_element[neighbor_it->voxel_idx].parent_voxel_idx[0];
				unsigned int neighbor_plane_idx = voxel_to_plane_idx[neighbor_parent_voxel_idx];
				if (is_connected[plane_idx][neighbor_plane_idx]) continue;
				is_connected[plane_idx][neighbor_plane_idx] = true;
				is_connected[neighbor_plane_idx][plane_idx] = true;
			}
		}
	}*/
}

// merge the same  planes in all the planes include generated by good voxels , pseudo bad voxels or remaining voxels
void PlaneSegmentation::IdentifySamePlanes()
{
	PlaneMerge plane_merge(this);
	//same_plane_group_array.plane_size = plane_merge_out.size;
	SameAndConnectedArray same_connected;
	same_connected.is_connected = NULL;
	same_connected.is_same_plane = NULL;
	bool no_plane_merge = debug_config_params.no_plane_merge;
#pragma omp parallel for
	for (int i = 0; i < plane_merge_out.size; i++)
	{
		PlaneMergeOutputItem* ref_plane_it = &plane_merge_out.planes[i];
		MathOperation::Compute(ref_plane_it->total_point_cnt, ref_plane_it->sums, ref_plane_it->plane_normal, ref_plane_it->plane_center, ref_plane_it->plane_mse);
		GetPlaneDistMse(ref_plane_it, ref_plane_it->plane_mse);
	}
	plane_merge.MergePlanes(no_plane_merge,plane_merge_out.planes, plane_merge_out.size, &same_plane_group_array, &same_connected, true);

	if (same_connected.is_same_plane != NULL)
	{
		for (unsigned int i = 0; i < plane_merge_out.size; i++)
		{
			if (same_connected.is_same_plane[i] != NULL)
			{
				delete[] same_connected.is_same_plane[i];
				same_connected.is_same_plane[i] = NULL;
			}
		}
		delete[] same_connected.is_same_plane;
		same_connected.is_same_plane = NULL;
	}

	if (same_connected.is_connected != NULL)
	{
		for (unsigned int i = 0; i < plane_merge_out.size; i++)
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

}


#ifdef SAVE_OUTPUT_FILE_DEBUG

void PlaneSegmentation::OutputBadVoxelMergeOutInfo(const std::string output_path)
{

	GetVoxelNeigbourDebugInfo(output_path,BAD_VOXEL_DEBUG);
	GetVoxelNeigbourDebugInfo(output_path,UNMERGED_BAD_VOXEL_DEBUG);
	GetVoxelNeigbourDebugInfo(output_path,UNMERGED_PSEUDOBAD_VOXEL_DEBUG);
	GetVoxelNeigbourDebugInfo(output_path,NON_OVERALL_MERGED_VOXEL_DEBUG);
	// find missing merged pseudo bad voxel number and missing merged real bad voxel number
	unsigned int missing_pseudobad_cnt = 0;
	unsigned int missing_realbad_cnt = 0;
	unsigned int missing_good_cnt = 0;
	for (unsigned int i = 0; i < plane_voxel_array.size; i++)
	{
		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[i];
		if ((plane_voxel->voxel_type == PSEUDO_BAD_VOXEL) && (!plane_voxel->is_being_merged))
		{
			missing_pseudobad_cnt++;
		}
		if ((plane_voxel->voxel_type == REAL_BAD_VOXEL) && (!plane_voxel->is_being_merged))
		{
			missing_realbad_cnt++;
		}
		if ((plane_voxel->voxel_type == GOOD_VOXEL) && (!plane_voxel->is_being_merged))
		{
			missing_good_cnt++;
		}
	}

	// compute point size from voxels 
	unsigned int good_voxel_points_size = 0;
	unsigned int pseudobad_voxel_points_size = 0;
	unsigned int realbad_voxel_points_size = 0;
	for (unsigned int i = 0; i < plane_voxel_array.size; i++)
	{
		unsigned int  voxel_type = plane_voxel_array.voxels[i].voxel_type;
		unsigned int  voxel_points_size = plane_voxel_array.voxels[i].points.size;
		if (voxel_type == GOOD_VOXEL)
		{
			good_voxel_points_size += voxel_points_size;
		}
		else if (voxel_type == PSEUDO_BAD_VOXEL)
		{
			pseudobad_voxel_points_size += voxel_points_size;
		}
		else
		{
			realbad_voxel_points_size += voxel_points_size;
		}
	}

	std::string folder_path = output_path;
	std::string bad_voxel_plane_path = folder_path + "voxel_plane_debug_info.txt";
	std::ofstream bad_voxel_plane_output(bad_voxel_plane_path);

	//unsigned int good_voxel_cnt = 0;
	//unsigned int good_voxel_cnt_from_pseudobad_mergeplane = 0;
	unsigned int pseduobad_voxel_cnt = 0;
	unsigned int total_point_cnt = 0;             // all the points in good voxel merge plane
	unsigned int bad_voxel_point_cnt = 0;         // all the points from bad merged voxels
	unsigned int merged_bad_voxel_cnt = 0;		  // all the bad merged voxel in the real bad voxel 
											             
	for (unsigned int i = 0; i < plane_merge_out.size; i++)
	{
		total_point_cnt += plane_merge_out.planes[i].total_point_cnt;
	}

	bad_voxel_plane_output << "voxel size length_x,lenth_y,length_z:                    " << "[" << length_x_of_voxel << "," << length_y_of_voxel << "," << length_z_of_voxel << "]" << '\n';
	bad_voxel_plane_output << "min distance of two voxels in same plane:                " << std::setw(8) << plane_seg_params.plane_seg_thresholds.THRESHOLD_MIN_PLANE_DIST_OF_TWO_VOXEL << '\n';
	bad_voxel_plane_output << "min distance of point who is in a plane:                  " << std::setw(8) << plane_seg_params.plane_seg_thresholds.THRESHOLD_MIN_DIST_OF_POINT2VOXEL << '\n';
	bad_voxel_plane_output << "min angle of two voxels in same plane:                   " << std::setw(8) << plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_NORMAL_ANGLE_OF_TWO_VOXEL << '\n';
	bad_voxel_plane_output << "max mse of a voxel whose points in a same plane:         " << std::setw(8) << plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_VOXEL << '\n';
	bad_voxel_plane_output << "max mse of all points who is in a same plane :           " << std::setw(8) << plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_PLANE << '\n';
	bad_voxel_plane_output << "min  point num of a valid normal voxel:                  " << std::setw(8) << plane_seg_params.plane_seg_thresholds.THRESHOLD_MIN_POINT_NUM_OF_VALID_NORMAL_VOXEL << '\n';
	bad_voxel_plane_output << "min  voxel num of a a plane:                             " << std::setw(8) << plane_seg_params.plane_seg_thresholds.THRESHOLD_MIN_VOXEL_NUM_OF_PLANE << '\n';
	bad_voxel_plane_output << "min point num of a plane:                                " << std::setw(8) << plane_seg_params.plane_seg_thresholds.THRESHOLD_MIN_POINT_NUM_OF_PLANE << '\n';

	bad_voxel_plane_output << "num of good voxel:                                       " << std::setw(8) << good_voxel_size << '\n';
	bad_voxel_plane_output << "num of good voxel being merged:                          " << std::setw(8) << good_voxel_size - missing_good_cnt << '\n';
	bad_voxel_plane_output << "num of good voxel missing merged:                        " << std::setw(8) << missing_good_cnt << '\n';
	//bad_voxel_plane_output << "num of good voxel being merged from pseudo plane:        " << std::setw(8) << good_voxel_cnt_from_pseudobad_mergeplane << '\n';
	bad_voxel_plane_output << "num of pesudo bad voxel:                                 " << std::setw(8) << pseudo_bad_voxel_size << '\n';
	bad_voxel_plane_output << "num of pesudo bad voxel being merged:                    " << std::setw(8) << pseudo_bad_voxel_size - missing_pseudobad_cnt << '\n';
	bad_voxel_plane_output << "num of pesudo bad voxel missing merged:                  " << std::setw(8) << missing_pseudobad_cnt << '\n';
	bad_voxel_plane_output << "num of bad voxel:                                        " << std::setw(8) << plane_voxel_array.size - good_voxel_size - pseudo_bad_voxel_size << '\n';
	bad_voxel_plane_output << "num of bad voxel being merged:                           " << std::setw(8) << plane_voxel_array.size - good_voxel_size - pseudo_bad_voxel_size - missing_realbad_cnt << '\n';
	bad_voxel_plane_output << "num of bad voxel missing merged:                         " << std::setw(8) << missing_realbad_cnt << '\n';
	bad_voxel_plane_output << "num of voxels merging by point_to_plane:                 " << std::setw(8) << bad_voxel_size << '\n';
	bad_voxel_plane_output << "num of points:                                           " << std::setw(8) << pt_cloud_xyz.size << '\n';
	bad_voxel_plane_output << "num of points from good voxels:                          " << std::setw(8) << good_voxel_points_size << '\n';
	bad_voxel_plane_output << "num of points from pseudo bad voxels:                    " << std::setw(8) << pseudobad_voxel_points_size << '\n';
	bad_voxel_plane_output << "num of points from real bad voxels:                      " << std::setw(8) << realbad_voxel_points_size << '\n';
	bad_voxel_plane_output << "num of points being merged all plane:                    " << std::setw(8) << total_point_cnt << '\n';
	bad_voxel_plane_output << "num of points being merged by point_to_plane:            " << std::setw(8) << bad_voxel_point_cnt << '\n';

	if (missing_good_cnt != 0)
	{
		bad_voxel_plane_output << "   missing good voxel index " << std::setw(28) << "grid_idx" << "\t\n";
		for (unsigned int i = 0; i < plane_voxel_array.size; i++)
		{
			PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[i];
			if ((plane_voxel->voxel_type == GOOD_VOXEL) && (!plane_voxel->is_being_merged))
			{
				bad_voxel_plane_output << std::setw(28) << i << std::setw(28) << plane_voxel->grid_idx << '\n';
			}
		}
	}
	else
	{
		bad_voxel_plane_output << "no missing good voxel" << "\t\n";
	}

	if (missing_pseudobad_cnt != 0)
	{
		bad_voxel_plane_output << "missing pesudo bad voxel index " << std::setw(28) << "grid_idx" << "\t\n";
		for (unsigned int i = 0; i < plane_voxel_array.size; i++)
		{
			PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[i];
			if ((plane_voxel->voxel_type == PSEUDO_BAD_VOXEL) && (!plane_voxel->is_being_merged))
			{
				bad_voxel_plane_output << std::setw(28) << i << std::setw(28) << plane_voxel->grid_idx << '\n';
			}
		}
	}
	else
	{
		bad_voxel_plane_output << "no missing pesudo bad voxel" << "\t\n";
	}

	if (missing_realbad_cnt != 0)
	{
		bad_voxel_plane_output << "missing real bad voxel index " << std::setw(28) << "grid_idx" << "\t\n";
		for (unsigned int i = 0; i < plane_voxel_array.size; i++)
		{
			PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[i];
			if ((plane_voxel->voxel_type == REAL_BAD_VOXEL) && (!plane_voxel->is_being_merged))
			{
				bad_voxel_plane_output << std::setw(28) << i << std::setw(28) << plane_voxel->grid_idx << '\n';
			}
		}
	}
	else
	{
		bad_voxel_plane_output << "no missing real bad voxel" << "\t\n";
	}

	bad_voxel_plane_output << "\t\n**************\n";

	bad_voxel_plane_output.close();

	//GetVoxelNeigbourDebugInfo(GOOD_VOXEL_DEBUG);
	GetVoxelNeigbourDebugInfo(output_path,PSEUDOBAD_VOXEL_DEBUG);
	GetVoxelNeigbourDebugInfo(output_path,ALL_VOXEL_DEBUG);
	//GetVoxelNeigbourDebugInfo(BAD_VOXEL_DEBUG);
	//GetVoxelNeigbourDebugInfo(UNMERGED_PSEUDOBAD_VOXEL_DEBUG);
	//GetVoxelNeigbourDebugInfo(UNMERGED_BAD_VOXEL_DEBUG);	

}

#endif // SAVE_OUTPUT_FILE_DEBUG

void PlaneSegmentation::FreeBadVoxelItem()
{
	log_info("FreeBadVoxelItem begin");
	if (bad_voxel_merge_item != NULL)
	{
		for (unsigned int i = 0; i < point_merged_voxel_size; i++)
		{
			BadVoxelMergeOutpuItem* bad_voxel_it = &bad_voxel_merge_item[i];
			if (bad_voxel_it->closest_plane_idx != NULL)
			{
				delete[] bad_voxel_it->closest_plane_idx;
				bad_voxel_it->closest_plane_idx = NULL;
			}
#ifdef VOXEL_NEAREST_APPROACH_MERGE
			if (bad_voxel_it->best_merge_plane_id != NULL)
			{
				delete[] bad_voxel_it->best_merge_plane_id;
				bad_voxel_it->best_merge_plane_id = NULL;
			}
			if (bad_voxel_it->best_merge_plane_voxel != NULL)
			{
				delete[] bad_voxel_it->best_merge_plane_voxel;
				bad_voxel_it->best_merge_plane_voxel = NULL;
			}
			if (bad_voxel_it->best_merge_point_id != NULL)
			{
				delete[] bad_voxel_it->best_merge_point_id;
				bad_voxel_it->best_merge_point_id = NULL;
			}
#endif

			if (bad_voxel_it->smallest_dist != NULL)
			{
				delete[] bad_voxel_it->smallest_dist;
				bad_voxel_it->smallest_dist = NULL;
			}

			if (bad_voxel_it->point_merged_flag != NULL)
			{
				delete[] bad_voxel_it->point_merged_flag;
				bad_voxel_it->point_merged_flag = NULL;
			}

			if (bad_voxel_it->remaining_points_merged_plane != NULL)
			{
				delete[] bad_voxel_it->remaining_points_merged_plane;
				bad_voxel_it->remaining_points_merged_plane = NULL;
			}

			if (bad_voxel_it->being_in_group_plane != NULL)
			{
				delete[] bad_voxel_it->being_in_group_plane;
				bad_voxel_it->being_in_group_plane = NULL;
			}

			/*if (bad_voxel_it->being_in_extended_plane != NULL)
			{
				delete[] bad_voxel_it->being_in_extended_plane;
				bad_voxel_it->being_in_extended_plane = NULL;
			}*/

			if (bad_voxel_it->is_plane_connected != NULL)
			{
				delete[] bad_voxel_it->is_plane_connected;
				bad_voxel_it->is_plane_connected = NULL;
			}

			if (bad_voxel_it->is_multi_plane_connected != NULL)
			{
				delete[] bad_voxel_it->is_multi_plane_connected;
				bad_voxel_it->is_multi_plane_connected = NULL;
			}

		}
		delete[] bad_voxel_merge_item;
		bad_voxel_merge_item = NULL;
	}
	log_info("FreeBadVoxelItem end");
}

	


// Assign plane outputs when have no same planes
void PlaneSegmentation::AssignPlaneOutputsNoSamePlane() {

	// Create plane segmentation output
	if (plane_merge_out.size == 0) return;

	plane_seg_out.size = plane_merge_out.size;

	plane_seg_out.planes = new PlaneItem[plane_seg_out.size];
	// First if not ransac plane, assign parent voxel index to PointArray and assign data to PlaneItem
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(plane_merge_out.size); i++)
	{
		unsigned int voxel_in_plane_idx;			// voxel index to occupied voxel array
		unsigned int point_in_voxel_cnt;			// total number of points in voxel
		unsigned int point_in_plane_idx;			// point index to input point array
		unsigned int point_in_plane_cnt;			// total number of points in plane
		unsigned int parent_voxel_idx;				// current's parent voxel index to occupied voxel array

		// Assign parent voxel index for the points in good and pseudo bad voxels
		point_in_plane_cnt = 0;
		PlaneMergeOutputItem *plane_it = &plane_merge_out.planes[i];
		for (unsigned int j = 0; j < plane_it->voxels.size; j++) {

			voxel_in_plane_idx = plane_it->voxels.voxel_idx[j];
			point_in_voxel_cnt = plane_voxel_array.voxels[voxel_in_plane_idx].points.size;
			point_in_plane_cnt += point_in_voxel_cnt;
		}

		point_in_plane_cnt += plane_it->extended_part_points.size;
		
		point_in_plane_cnt += plane_it->points.size;
		point_in_plane_cnt += plane_it->multiplane_points.size;

		// Assign parent_voxel_idx, plane_type and points to PlaneItem
		plane_seg_out.planes[i].parent_voxel_idx = plane_it->parent_voxel_idx;
		//plane_seg_out.planes[i].plane_type = PLANE_UNDEFINED;
		plane_seg_out.planes[i].points.size = point_in_plane_cnt;
		plane_seg_out.planes[i].points.point_idx = new unsigned int[point_in_plane_cnt];

		// Assign point indexes to PlaneItem from the points in good and pseudo bad voxels
		point_in_plane_cnt = 0;
		for (unsigned int j = 0; j < plane_it->voxels.size; j++) {

			voxel_in_plane_idx = plane_it->voxels.voxel_idx[j];
			point_in_voxel_cnt = plane_voxel_array.voxels[voxel_in_plane_idx].points.size;

			for (unsigned int k = 0; k < point_in_voxel_cnt; k++) {

				point_in_plane_idx = plane_voxel_array.voxels[voxel_in_plane_idx].points.point_idx[k];
				plane_seg_out.planes[i].points.point_idx[point_in_plane_cnt] = point_in_plane_idx;
				point_in_plane_cnt++;
			}
		}

		for (unsigned int j = 0; j < plane_it->extended_part_points.size; j++) {

			point_in_plane_idx = plane_it->extended_part_points.point_idx[j];
			plane_seg_out.planes[i].points.point_idx[point_in_plane_cnt] = point_in_plane_idx;
			point_in_plane_cnt++;
		}

		// Assign point indexes to PlaneItem from the real bad points
		for (unsigned int j = 0; j < plane_it->points.size; j++) {

			point_in_plane_idx = plane_it->points.point_idx[j];
			plane_seg_out.planes[i].points.point_idx[point_in_plane_cnt] = point_in_plane_idx;
			point_in_plane_cnt++;
		}

		// Assign point indexes to PlaneItem from the edge points
		for (unsigned int j = 0; j < plane_it->multiplane_points.size; j++) {

			point_in_plane_idx = plane_it->multiplane_points.point_idx[j];
			plane_seg_out.planes[i].points.point_idx[point_in_plane_cnt] = point_in_plane_idx;
			point_in_plane_cnt++;
		}

		// it is impossible, must have bug
		if (point_in_plane_cnt != plane_it->total_point_cnt) log_debug("plane %d point_in_plane_cnt %d is not equel to total_point_cnt =%d", i, point_in_plane_cnt, plane_it->total_point_cnt);

		// Assign plane_center, plane_normal, plane_mse and sums to PlaneItem
		parent_voxel_idx = plane_seg_out.planes[i].parent_voxel_idx;
		plane_seg_out.planes[i].plane_center = plane_it->plane_center;
		plane_seg_out.planes[i].plane_normal = plane_it->plane_normal;
		GetPlaneDistMse(&plane_seg_out.planes[i], plane_seg_out.planes[i].plane_mse);
		AssignSums(&plane_seg_out.planes[i].sums,&plane_it->sums);
	}

	return;
}

#ifdef SAVE_OUTPUT_FILE_DEBUG

void PlaneSegmentation::PlaneMergeOutInfo(const std::string output_path)
{
	// unsigned int voxel_in_plane_idx;			voxel index to occupied voxel array
	unsigned int point_in_plane_idx;			// point index to input point array
	unsigned int parent_voxel_idx;				// current's parent voxel index to occupied voxel array

	std::string plane_output_path = output_path;
	std::string plane_output_file = "plane_output.txt";
	std::ofstream plane_output(plane_output_path + plane_output_file);
	plane_output << "Total number of planes: " << plane_seg_out.size << '\n';

	bool *point_being_merged = new bool[pt_cloud_xyz.size];
	for (unsigned int i = 0; i < pt_cloud_xyz.size; i++)
	{
		point_being_merged[i] = false;
	}

	int tatal_merged_point_cnt = 0;
	for (unsigned int i = 0; i < plane_seg_out.size; i++) {

		//point_in_plane_cnt = 0;
		for (unsigned int j = 0; j < plane_seg_out.planes[i].points.size; j++) {

			point_in_plane_idx = plane_seg_out.planes[i].points.point_idx[j];
			point_being_merged[point_in_plane_idx] = true;
		}
		tatal_merged_point_cnt += plane_seg_out.planes[i].points.size;
	}

	plane_output << "Total number of merged points: " << tatal_merged_point_cnt << '\n';
	plane_output << "Total number of unmerged points: " << pt_cloud_xyz.size - tatal_merged_point_cnt << '\n';
	
	// get the  conversion of all  seg out plane idx to merge out plane idx
	std::vector<unsigned int> seg_to_merge_no_same_plane_idx;  // for conversion of seg out planes who have no same plane 
	std::vector<std::vector<unsigned int>> all_seg_to_merge_same_plane_idx;  // for conversion of seg out planes who have several same plane 
	std::vector<unsigned int> seg_out_same_plane_voxel_size; // for record the voxel size  of seg out planes who have several same plane
	std::vector<std::vector<PlaneItemType>> seg_out_same_plane_voxel_type;// for record the voxel type  of seg out planes who have several same plane
	seg_to_merge_no_same_plane_idx.clear();
	seg_to_merge_no_same_plane_idx.shrink_to_fit();
	for (unsigned int i = 0; i < same_plane_group_array.plane_size; i++)
	{  
		unsigned int same_plane_idx = same_plane_group_array.same_plane_group_idx[i];
		if (same_plane_idx == -1)
		{
			seg_to_merge_no_same_plane_idx.push_back(i);
		}			
	}

	unsigned int no_same_plane_cnt = static_cast<unsigned int>(seg_to_merge_no_same_plane_idx.size());
	unsigned int same_plane_cnt = plane_seg_out.size - no_same_plane_cnt;
	all_seg_to_merge_same_plane_idx.clear();
	all_seg_to_merge_same_plane_idx.resize(same_plane_cnt);
	seg_out_same_plane_voxel_size.clear();
	seg_out_same_plane_voxel_size.resize(same_plane_cnt);
	seg_out_same_plane_voxel_type.clear();
	seg_out_same_plane_voxel_type.resize(same_plane_cnt);
	for (unsigned int i = 0; i < same_plane_cnt; i++)
	{
		seg_out_same_plane_voxel_size[i] = 0;
		for (unsigned int j = 0; j < same_plane_group_array.plane_size; j++)
		{
			
			unsigned int same_plane_idx = same_plane_group_array.same_plane_group_idx[j];
			if (same_plane_idx == -1) continue;
			if (same_plane_idx == i)
			{
				all_seg_to_merge_same_plane_idx[i].push_back(j);
				unsigned int voxel_size = 0;
				//PlaneVoxelType voxel_type = VOXEL_INVALID_TYPE;
				PlaneMergeOutputItem* plane_merge_it = &plane_merge_out.planes[j];
				voxel_size = plane_merge_it->voxels.size;
				//voxel_type = plane_voxel_array.voxels[plane_merge_it->parent_voxel_idx].voxel_type;
				seg_out_same_plane_voxel_type[i].push_back(plane_merge_it->plane_type);
				seg_out_same_plane_voxel_size[i] += voxel_size;
			}
		}
	}

	plane_output << "seg out plane idx: " << "merge out plane idx:   " << '\n';

	for (unsigned int i = 0; i < seg_to_merge_no_same_plane_idx.size(); i++)
	{
		plane_output << i << " \t\t\t\t\t"<< seg_to_merge_no_same_plane_idx[i] << std::endl;
	}
	for (unsigned int i = 0; i < all_seg_to_merge_same_plane_idx.size(); i++)
	{
		plane_output << i+ no_same_plane_cnt << " \t\t\t\t\t";
		for (unsigned int j = 0; j < all_seg_to_merge_same_plane_idx[i].size(); j++)
		{
			plane_output << all_seg_to_merge_same_plane_idx[i][j];
			if (j < all_seg_to_merge_same_plane_idx[i].size() - 1)
			{
				plane_output << ", ";
			}
		}
		plane_output << std::endl;

	}

	plane_output << "seg out plane idx: " << "voxel size:   " << "voxel type:   "<<'\n';

	for (unsigned int i = 0; i < all_seg_to_merge_same_plane_idx.size(); i++)
	{
		plane_output << i + no_same_plane_cnt <<" \t\t\t\t\t"<< seg_out_same_plane_voxel_size[i]<< " \t\t\t";
		for (unsigned int j = 0; j < seg_out_same_plane_voxel_type[i].size(); j++)
		{
			plane_output << seg_out_same_plane_voxel_type[i][j];
			if (j < seg_out_same_plane_voxel_type[i].size() - 1)
			{
				plane_output << ", ";
			}
		}
		plane_output << std::endl;
	}
	for (unsigned int i = 0; i < plane_seg_out.size; i++) {

		parent_voxel_idx = plane_seg_out.planes[i].parent_voxel_idx;
		float point_mse;
		//GetPlaneDistMseByPointAvg(&plane_voxel_array.voxels[parent_voxel_idx], plane_merge_out.planes[i].voxels,point_mse);
		GetPlaneDistMse(&plane_seg_out.planes[i], point_mse);

		unsigned int voxel_size = 0;
		//PlaneVoxelType voxel_type = VOXEL_INVALID_TYPE;
		PlaneItemType plane_type = REAL_BAD_PLANE;
		if (i < no_same_plane_cnt)
		{
			unsigned int merge_out_plane_idx = seg_to_merge_no_same_plane_idx[i];
			PlaneMergeOutputItem* plane_merge_it = &plane_merge_out.planes[merge_out_plane_idx];
			voxel_size = plane_merge_it->voxels.size;
			//parent_voxel_idx = plane_merge_it->parent_voxel_idx;
			plane_type = plane_merge_it->plane_type;
		}
		else if (same_plane_cnt != 0)
		{
			unsigned int same_plane_idx = i - no_same_plane_cnt;
			voxel_size = seg_out_same_plane_voxel_size[same_plane_idx];
			for (unsigned int j = 0; j < seg_out_same_plane_voxel_type[same_plane_idx].size(); j++)
			{
				plane_type = plane_type < seg_out_same_plane_voxel_type[same_plane_idx][j] ? plane_type : seg_out_same_plane_voxel_type[same_plane_idx][j];
			}
		}

		plane_output << "Plane Index = " << i << ", Parent Voxel Index = " << plane_seg_out.planes[i].parent_voxel_idx << ", Plane Type = " << plane_type << ", Voxel Count = " << voxel_size << ", Point Count = " << plane_seg_out.planes[i].points.size << '\n';
		if (i < no_same_plane_cnt)
		{
			unsigned int merge_out_plane_idx = seg_to_merge_no_same_plane_idx[i];
			plane_output << "plane idx before merge plane: " << merge_out_plane_idx << std::endl;
		}
		else if(same_plane_cnt != 0)
		{
			unsigned int same_plane_idx = i - no_same_plane_cnt;
			plane_output << "plane idx before merge plane:  ";
			for (unsigned int j = 0; j < all_seg_to_merge_same_plane_idx[same_plane_idx].size(); j++)
			{
				unsigned int merge_out_plane_idx = all_seg_to_merge_same_plane_idx[same_plane_idx][j];
				plane_output << merge_out_plane_idx;
				if (j < all_seg_to_merge_same_plane_idx[same_plane_idx].size() - 1)
				{
					plane_output << ", ";
				}
			}
			plane_output << std::endl;
			plane_output << "plane type before merge plane:  ";
			for (unsigned int j = 0; j < seg_out_same_plane_voxel_type[same_plane_idx].size(); j++)
			{
				PlaneItemType plane_type = seg_out_same_plane_voxel_type[same_plane_idx][j];
				plane_output << plane_type;
				if (j < seg_out_same_plane_voxel_type[same_plane_idx].size() - 1)
				{
					plane_output << ", ";
				}
			}
			plane_output << std::endl;
		}

		plane_output << std::setprecision(10) << std::fixed;
		plane_output << i << '\t' << plane_seg_out.planes[i].plane_mse << '\n';
		plane_output << i << '\t' << point_mse << '\n';
		plane_output << i << '\t' << plane_seg_out.planes[i].plane_center.x << '\t' << plane_seg_out.planes[i].plane_center.y << '\t' << plane_seg_out.planes[i].plane_center.z << '\n';
		plane_output << i << '\t' << plane_seg_out.planes[i].plane_normal.x << '\t' << plane_seg_out.planes[i].plane_normal.y << '\t' << plane_seg_out.planes[i].plane_normal.z << '\n';
		plane_output << i << '\t' << plane_seg_out.planes[i].sums.sum_x << '\n';
		plane_output << i << '\t' << plane_seg_out.planes[i].sums.sum_y << '\n';
		plane_output << i << '\t' << plane_seg_out.planes[i].sums.sum_z << '\n';
		plane_output << i << '\t' << plane_seg_out.planes[i].sums.sum_xx << '\n';
		plane_output << i << '\t' << plane_seg_out.planes[i].sums.sum_yy << '\n';
		plane_output << i << '\t' << plane_seg_out.planes[i].sums.sum_zz << '\n';
		plane_output << i << '\t' << plane_seg_out.planes[i].sums.sum_xy << '\n';
		plane_output << i << '\t' << plane_seg_out.planes[i].sums.sum_xz << '\n';
		plane_output << i << '\t' << plane_seg_out.planes[i].sums.sum_yz << '\n';
	}
	plane_output.close();
	//Point3f point = {1566.170044f,-1912.069946f,-205.190994f};
	//GetPointDebugInfo(point);
	//point = { 1566.550049f,-1910.569946f,-356.669006f};
	//GetPointDebugInfo(point);
	//point = { -756.000000f,-48.500000f,1660.000000f };
	//GetPointDebugInfo(point);
	
	delete[] point_being_merged;
	point_being_merged = NULL;
	seg_to_merge_no_same_plane_idx.clear();
	seg_to_merge_no_same_plane_idx.shrink_to_fit();
	for (unsigned int i = 0; i < all_seg_to_merge_same_plane_idx.size(); i++)
	{
		all_seg_to_merge_same_plane_idx[i].clear();
		all_seg_to_merge_same_plane_idx[i].shrink_to_fit();
	}
	all_seg_to_merge_same_plane_idx.clear();
	all_seg_to_merge_same_plane_idx.shrink_to_fit();
	seg_out_same_plane_voxel_size.clear();
	seg_out_same_plane_voxel_size.shrink_to_fit();
	for (unsigned int i = 0; i < seg_out_same_plane_voxel_type.size(); i++)
	{
		seg_out_same_plane_voxel_type[i].clear();
		seg_out_same_plane_voxel_type[i].shrink_to_fit();
	}
	seg_out_same_plane_voxel_type.clear();
	seg_out_same_plane_voxel_type.shrink_to_fit();

}
#endif//SAVE_OUTPUT_FILE_DEBUG

void PlaneSegmentation::MissingPointsOutput(const std::string output_path, const PlaneSegmentationOutput& plane_seg)
{
	int missing_cnt = 0;
	bool *point_being_merged = new bool[pt_cloud_xyz.size];
	for (unsigned int i = 0; i < pt_cloud_xyz.size; i++)
	{
		point_being_merged[i] = false;
	}

	for (unsigned int i = 0; i < plane_seg.size; i++) {

		//point_in_plane_cnt = 0;
		for (unsigned int j = 0; j < plane_seg.planes[i].points.size; j++) {

			unsigned int point_in_plane_idx = plane_seg.planes[i].points.point_idx[j];
			point_being_merged[point_in_plane_idx] = true;
		}
	}

	std::string missing_points_path = output_path;
	std::string missing_points_info_file = "missing_points_info.txt";
	std::ofstream missing_points_info(missing_points_path + missing_points_info_file);

	missing_points_info << std::setprecision(6) << std::fixed;
	for (unsigned int i = 0; i < pt_cloud_xyz.size; i++) {

		if (!point_being_merged[i]) {

			unsigned long long grid_index = ConvertXYZToVoxelID(pt_cloud_xyz.points[i].x, pt_cloud_xyz.points[i].y, pt_cloud_xyz.points[i].z);
			unsigned int voxel_index = grid_to_occupied[grid_index];
			missing_points_info << "Voxel Index = " << voxel_index << ", Point Index = " << i << ", [" << pt_cloud_xyz.points[i].x;
			missing_points_info << ";" << pt_cloud_xyz.points[i].y;
			missing_points_info << ";" << pt_cloud_xyz.points[i].z << "]\n";
			missing_cnt++;
		}
	}

	if (missing_cnt == 0) missing_points_info << "no missing point" << std::endl;

	missing_points_info.close();

	if (missing_cnt != 0)
	{
		std::string missing_points_file = "missing_points.txt";
		missing_points_path = output_path + "plane_xyz\\";
		if (IOData::createDirectory(missing_points_path))
		{
			log_error("createDirectory %s failed", missing_points_path.c_str());
			return;
		}

		std::ofstream missing_points(missing_points_path + missing_points_file);

		missing_points << std::setprecision(6) << std::fixed;
		for (unsigned int i = 0; i < pt_cloud_xyz.size; i++) {

			if (!point_being_merged[i]) {

				missing_points << pt_cloud_xyz.points[i].x << ";";
				missing_points << pt_cloud_xyz.points[i].y << ";";
				missing_points << pt_cloud_xyz.points[i].z << '\n';
			}
		}
		missing_points.close();
	}	
	delete[] point_being_merged;
	point_being_merged = NULL;
}

void PlaneSegmentation::AssignPlaneOutputs() {

	unsigned int seg_out_plane_idx = 0;
	// get the same plane group number
	unsigned int same_plane_group_cnt = 0; // the num of planes which are the result of same planes merging
	unsigned int max_same_plane_idx = -1; // record the max same plane  idx
	unsigned int num_of_same_plane = 0;   // the number of planes which have same plane
	int num_of_planes_with_no_same_plane_0 = 0;// the number of planes with no same plane in plane_merge_out
	//int num_of_planes_with_no_same_plane_1 = 0;// the number of planes with no same plane in points_group_plane_merge_out
	int* no_same_plane_to_plane_idx_0 = NULL; //record index  of planes  with no same plane to orignal index of planes in  plane_merge_out
	//int* no_same_plane_to_plane_idx_1 = NULL; //record index  of planes  with no same plane to orignal index of planes in  points_group_plane_merge_out

	unsigned int cur_total_plane_size = plane_merge_out.size;
	if (cur_total_plane_size <= 1)
	{
		log_debug("plane_seg_out size =%d", cur_total_plane_size);
		return AssignPlaneOutputsNoSamePlane();
	}

	for (unsigned int i = 0; i < cur_total_plane_size; i++)
	{
		if (i < same_plane_group_array.plane_size)
		{
			unsigned int same_plane_idx = same_plane_group_array.same_plane_group_idx[i];
			if (same_plane_idx != -1)
			{
				num_of_same_plane++;
				if (max_same_plane_idx == -1) max_same_plane_idx = 0;
				if (same_plane_idx > max_same_plane_idx) max_same_plane_idx = same_plane_idx;
			}
			else
			{
				if (i < plane_merge_out.size) num_of_planes_with_no_same_plane_0++;
			}
		}
		else
		{
			if (i < plane_merge_out.size) num_of_planes_with_no_same_plane_0++;
		}
	}

	if (max_same_plane_idx != -1) same_plane_group_cnt = max_same_plane_idx+1;
	
	//log_debug("same_plane_group_cnt =%d num_of_same_plane =%d", same_plane_group_cnt, num_of_same_plane);

	if (same_plane_group_cnt == 0)
	{
		log_debug("plane_seg_out size =%d", cur_total_plane_size);
		AssignPlaneOutputsNoSamePlane();
		return;
	}

	plane_seg_out.size = cur_total_plane_size - num_of_same_plane+ same_plane_group_cnt;
	log_debug("plane_seg_out size =%d", plane_seg_out.size);

	plane_seg_out.planes = new PlaneItem[plane_seg_out.size];

	if (num_of_planes_with_no_same_plane_0 != 0)
	{
		no_same_plane_to_plane_idx_0 = new int[num_of_planes_with_no_same_plane_0];
		for (int i = 0; i < num_of_planes_with_no_same_plane_0; i++)
		{
			no_same_plane_to_plane_idx_0[i] = -1;
		}
		int j = 0;
		for (unsigned int i = 0; i < plane_merge_out.size; i++)
		{
			if (i < same_plane_group_array.plane_size){
				if (same_plane_group_array.same_plane_group_idx[i] != -1) continue;
				no_same_plane_to_plane_idx_0[j] = i;
			}
			else{
				no_same_plane_to_plane_idx_0[j] = i;
			}
			j++;
		}
	}

	/*
	if (num_of_planes_with_no_same_plane_1 != 0)
	{
		 no_same_plane_to_plane_idx_1 = new int[num_of_planes_with_no_same_plane_1];
		for (int i = 0; i < num_of_planes_with_no_same_plane_1; i++)
		{
			no_same_plane_to_plane_idx_1[i] = -1;
		}
		int j = 0;
		for (int i = 0; i < points_group_plane_merge_out.size; i++)
		{
			if (i < same_plane_group_array.plane_size){
				if (same_plane_group_array.same_plane_group_idx[plane_merge_out.size + i] != -1) continue;
				no_same_plane_to_plane_idx_1[j] = i;
			}
			else {
				no_same_plane_to_plane_idx_1[j] = i;
			}
			j++;
		}
	}*/


	// First  merge out all the planes which  have no same plane
#pragma omp parallel for
	for (int i = 0 ; i < num_of_planes_with_no_same_plane_0; i++)
	{
		//unsigned int voxel_in_plane_idx;			// voxel index to occupied voxel array
		//unsigned int point_in_voxel_cnt;			// total number of points in voxel
		//unsigned int point_in_plane_idx;			// point index to input point array
		unsigned int point_in_plane_cnt;			// total number of points in plane
		//unsigned int parent_voxel_idx;				 current's parent voxel index to occupied voxel array

		//if (same_plane_group_array.same_plane_group_idx[i] != -1) continue;
		// Assign parent voxel index for the points in good and pseudo bad voxels
		int orign_plane_idx = no_same_plane_to_plane_idx_0[i];
		PlaneMergeOutputItem* orgin_plane_it = &plane_merge_out.planes[orign_plane_idx];
		point_in_plane_cnt = 0;
		for (unsigned int j = 0; j < orgin_plane_it->voxels.size; j++) {

			unsigned int voxel_in_plane_idx = orgin_plane_it->voxels.voxel_idx[j];
			unsigned int point_in_voxel_cnt = plane_voxel_array.voxels[voxel_in_plane_idx].points.size;
			point_in_plane_cnt += point_in_voxel_cnt;
		}

		PlaneItem* out_plane_it = &plane_seg_out.planes[i];

		point_in_plane_cnt += orgin_plane_it->extended_part_points.size;

		point_in_plane_cnt += orgin_plane_it->points.size;
		point_in_plane_cnt += orgin_plane_it->multiplane_points.size;

		// Assign parent_voxel_idx, plane_type and points to PlaneItem
		out_plane_it->parent_voxel_idx = orgin_plane_it->parent_voxel_idx;
		//out_plane_it->plane_type = PLANE_UNDEFINED;
		out_plane_it->points.size = point_in_plane_cnt;
		out_plane_it->points.point_idx = new unsigned int[point_in_plane_cnt];

		// Assign point indexes to PlaneItem from the points in good and pseudo bad voxels
		point_in_plane_cnt = 0;
		for (unsigned int j = 0; j < orgin_plane_it->voxels.size; j++) {

			unsigned int voxel_in_plane_idx = orgin_plane_it->voxels.voxel_idx[j];
			unsigned int point_in_voxel_cnt = plane_voxel_array.voxels[voxel_in_plane_idx].points.size;

			for (unsigned int k = 0; k < point_in_voxel_cnt; k++) {

				unsigned int point_in_plane_idx = plane_voxel_array.voxels[voxel_in_plane_idx].points.point_idx[k];
				out_plane_it->points.point_idx[point_in_plane_cnt] = point_in_plane_idx;
				point_in_plane_cnt++;
			}
		}

		for (unsigned int j = 0; j < orgin_plane_it->extended_part_points.size; j++) {

			unsigned int point_in_plane_idx = orgin_plane_it->extended_part_points.point_idx[j];
			out_plane_it->points.point_idx[point_in_plane_cnt] = point_in_plane_idx;
			point_in_plane_cnt++;
		}


		// Assign point indexes to PlaneItem from the real bad points
		for (unsigned int j = 0; j < orgin_plane_it->points.size; j++) {

			unsigned int point_in_plane_idx = orgin_plane_it->points.point_idx[j];
			out_plane_it->points.point_idx[point_in_plane_cnt] = point_in_plane_idx;
			point_in_plane_cnt++;
		}

		// Assign point indexes to PlaneItem from the real bad points
		for (unsigned int j = 0; j < orgin_plane_it->multiplane_points.size; j++) {

			unsigned int point_in_plane_idx = orgin_plane_it->multiplane_points.point_idx[j];
			out_plane_it->points.point_idx[point_in_plane_cnt] = point_in_plane_idx;
			point_in_plane_cnt++;
		}

		// it is impossible, must have bug
		if (point_in_plane_cnt != orgin_plane_it->total_point_cnt)
			log_debug("plane %d point_in_plane_cnt %d is not equel to total_point_cnt =%d", i, point_in_plane_cnt, orgin_plane_it->total_point_cnt);

		// Assign plane_center, plane_normal, plane_mse and sums to PlaneItem
		out_plane_it->parent_voxel_idx = orgin_plane_it->parent_voxel_idx;
		out_plane_it->plane_center = orgin_plane_it->plane_center;
		out_plane_it->plane_normal = orgin_plane_it->plane_normal;
		//GetPlaneDistMseByVoxelAvg(plane_merge_out.planes[i].voxels, out_plane_it->plane_mse);
		GetPlaneDistMse(out_plane_it, out_plane_it->plane_mse);
		AssignSums(&out_plane_it->sums, &orgin_plane_it->sums);
		//float eigen_mse;
		//MathOperation::Compute(out_plane_it->points.size, out_plane_it->sums, out_plane_it->plane_normal, out_plane_it->plane_center, eigen_mse);

	}
	seg_out_plane_idx = num_of_planes_with_no_same_plane_0/* + num_of_planes_with_no_same_plane_1*/;

	//release
	if (no_same_plane_to_plane_idx_0 != NULL)
	{
		delete[] no_same_plane_to_plane_idx_0;
		no_same_plane_to_plane_idx_0 = NULL;
	}
	/*if (no_same_plane_to_plane_idx_1 != NULL)
	{
		delete[] no_same_plane_to_plane_idx_1;
		no_same_plane_to_plane_idx_1 = NULL;
	}*/

	// merge all the same plane to the new plane	
	unsigned int same_plane_idx = -1;
	unsigned int *point_in_same_plane_cnt_list = new unsigned int[same_plane_group_cnt];
	for (unsigned int i = 0; i < same_plane_group_cnt; i++)
	{
		point_in_same_plane_cnt_list[i] = 0;
	}

	unsigned int voxel_in_plane_idx;			// voxel index to occupied voxel array
	unsigned int point_in_voxel_cnt;			// total number of points in voxel
	unsigned int point_in_plane_idx;			// point index to input point array
	unsigned int point_in_plane_cnt;			// total number of points in plane
	//unsigned int parent_voxel_idx;				// current's parent voxel index to occupied voxel array

	// first get the points in all the same plane
	for (unsigned int i = 0; i < same_plane_group_array.plane_size; i++)
	{
		same_plane_idx = same_plane_group_array.same_plane_group_idx[i];
		if (same_plane_idx == -1) continue;
		if (same_plane_idx >= same_plane_group_cnt)
		{
			log_debug("same_plane_idx =%d is larger than same_plane_group_cnt =%d", same_plane_idx, same_plane_group_cnt);
			continue;
		}

		PlaneMergeOutputItem* plane_merge_it = &plane_merge_out.planes[i];
		for (unsigned int j = 0; j < plane_merge_it->voxels.size; j++) {

			voxel_in_plane_idx = plane_merge_it->voxels.voxel_idx[j];
			point_in_voxel_cnt = plane_voxel_array.voxels[voxel_in_plane_idx].points.size;
			point_in_same_plane_cnt_list[same_plane_idx] += point_in_voxel_cnt;
		}
		point_in_same_plane_cnt_list[same_plane_idx] += plane_merge_it->extended_part_points.size;
		point_in_same_plane_cnt_list[same_plane_idx] += plane_merge_it->points.size;
		point_in_same_plane_cnt_list[same_plane_idx] += plane_merge_it->multiplane_points.size;
	}

	for (unsigned int i = 0; i < same_plane_group_cnt; i++)
	{
		unsigned int tmp_seg_out_plane_idx = seg_out_plane_idx+i;
		PlaneItem* out_plane_it = &plane_seg_out.planes[tmp_seg_out_plane_idx];
		out_plane_it->parent_voxel_idx = std::numeric_limits<unsigned int >::max();
		out_plane_it->points.size = point_in_same_plane_cnt_list[i];
		out_plane_it->points.point_idx = new unsigned int[out_plane_it->points.size];
		//out_plane_it->plane_type = PLANE_UNDEFINED;
		ClearSums(&out_plane_it->sums);
		point_in_same_plane_cnt_list[i] = 0;
	}

	// put all the points of the same planes ,and recompute normal and center
	for (unsigned int i = 0; i < same_plane_group_array.plane_size; i++)
	{
		same_plane_idx = same_plane_group_array.same_plane_group_idx[i];
		if (same_plane_idx == -1) continue;
		unsigned int current_seg_out_plane_idx = seg_out_plane_idx + same_plane_idx;
		PlaneItem* out_plane_it = &plane_seg_out.planes[current_seg_out_plane_idx];  // the new merged plane is put on the end of plane_seg_out
		PlaneItemType old_plane_type = REAL_BAD_PLANE;
		if (i < plane_merge_out.size)
		{
			PlaneMergeOutputItem* plane_merge_it = &plane_merge_out.planes[i];
			if ((old_plane_type > plane_merge_it->plane_type) || out_plane_it->parent_voxel_idx == std::numeric_limits<unsigned int >::max())
			{
				old_plane_type = plane_merge_it->plane_type;
				out_plane_it->parent_voxel_idx = plane_merge_it->parent_voxel_idx;
			}

			// add points in the is_overall_merged voxels
			for (unsigned int j = 0; j < plane_merge_it->voxels.size; j++) {

				voxel_in_plane_idx = plane_merge_it->voxels.voxel_idx[j];
				point_in_voxel_cnt = plane_voxel_array.voxels[voxel_in_plane_idx].points.size;

				for (unsigned int k = 0; k < point_in_voxel_cnt; k++) {
					point_in_plane_cnt = point_in_same_plane_cnt_list[same_plane_idx];
					point_in_plane_idx = plane_voxel_array.voxels[voxel_in_plane_idx].points.point_idx[k];
					out_plane_it->points.point_idx[point_in_plane_cnt] = point_in_plane_idx;
					point_in_same_plane_cnt_list[same_plane_idx]++;
				}
			}
			// add points of same_normal_points in plane
			for (unsigned int j = 0; j < plane_merge_it->extended_part_points.size; j++) {
				point_in_plane_cnt = point_in_same_plane_cnt_list[same_plane_idx];
				point_in_plane_idx = plane_merge_it->extended_part_points.point_idx[j];
				out_plane_it->points.point_idx[point_in_plane_cnt] = point_in_plane_idx;
				point_in_same_plane_cnt_list[same_plane_idx]++;
			}
			// add points of points  in plane by MergeBadVoxels
			for (unsigned int j = 0; j < plane_merge_it->points.size; j++) {
				point_in_plane_cnt = point_in_same_plane_cnt_list[same_plane_idx];
				point_in_plane_idx = plane_merge_it->points.point_idx[j];
				out_plane_it->points.point_idx[point_in_plane_cnt] = point_in_plane_idx;
				point_in_same_plane_cnt_list[same_plane_idx]++;
			}

			// add points of points  in plane by 
			for (unsigned int j = 0; j < plane_merge_it->multiplane_points.size; j++) {
				point_in_plane_cnt = point_in_same_plane_cnt_list[same_plane_idx];
				point_in_plane_idx = plane_merge_it->multiplane_points.point_idx[j];
				out_plane_it->points.point_idx[point_in_plane_cnt] = point_in_plane_idx;
				point_in_same_plane_cnt_list[same_plane_idx]++;
			}
			// add sums  of this plane
			PushSums(&out_plane_it->sums, &plane_merge_it->sums);
		}
	}

	// it is impossible, must have bug
	for (unsigned int i = 0; i < same_plane_group_cnt; i++)
	{
		unsigned int tmp_seg_out_plane_idx = seg_out_plane_idx + i;
		PlaneItem* out_plane_it = &plane_seg_out.planes[tmp_seg_out_plane_idx];
		if(out_plane_it->points.size != point_in_same_plane_cnt_list[i]) 
			log_debug(" point_in_plane_cnt[%d] of same planes  %d is not equel to total_point_cnt =%d", i, point_in_same_plane_cnt_list[i], out_plane_it->points.size);
	}

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(same_plane_group_cnt); i++)
	{
		unsigned int current_seg_out_plane_idx = seg_out_plane_idx + i;
		PlaneItem* out_plane_it = &plane_seg_out.planes[current_seg_out_plane_idx];
		float eigen_mse = 0.0f;
		MathOperation::Compute(out_plane_it->points.size, out_plane_it->sums, out_plane_it->plane_normal, out_plane_it->plane_center, eigen_mse);
		GetPlaneDistMse(out_plane_it, out_plane_it->plane_mse);
	}

	if (point_in_same_plane_cnt_list != NULL)
	{
		delete[] point_in_same_plane_cnt_list;
		point_in_same_plane_cnt_list = NULL;
	}
	return;
}

void PlaneSegmentation::GetOnePlaneArea(PlaneItem* plane)
{
	VoxelParams area_voxel;
	float scale = 0.3f;
	area_voxel.length_x_of_voxel = plane_seg_params.voxel_params.length_x_of_voxel * scale;
	area_voxel.length_y_of_voxel = plane_seg_params.voxel_params.length_y_of_voxel * scale;
	area_voxel.length_z_of_voxel = plane_seg_params.voxel_params.length_z_of_voxel * scale;

	GeometryFilter Geometry_it;
	Geometry_it.SetInputNormal(plane->plane_normal);
	Geometry_it.SetInputCenter(plane->plane_center);
	PointArray pt_plane_xyz;
	pt_plane_xyz.size = plane->points.size;
	pt_plane_xyz.points = new Point3f[pt_plane_xyz.size];
	for (unsigned int j = 0; j < plane->points.size; j++)
	{
		pt_plane_xyz.points[j] = pt_cloud_xyz.points[plane->points.point_idx[j]];
	}

	Geometry_it.GetPlaneArea2D(pt_plane_xyz, area_voxel, plane->plane_area);
	delete[] pt_plane_xyz.points;
	//log_debug("plane %d plane area = %f", i, out_plane->plane_area);
}

void PlaneSegmentation::GetPlaneArea() 
{
	for (int i = 0; i < static_cast<int>(plane_seg_out.size); i++)
	{
		GetOnePlaneArea(&plane_seg_out.planes[i]);
	}

}
void PlaneSegmentation::GetPointDebugInfo(Point3f &point)
{
	log_info("GetPointDebugInfo: Point(%.10f, %.10f, %.10f)", point.x, point.y, point.z);
	unsigned long long grid_idx = ConvertXYZToVoxelID(point.x, point.y, point.z);
	if (grid_idx > grid_to_occupied_size)
	{
		log_info("\t\t GetPointDebugInfo input error grid_idx = %ld is exceed the size of grid_to_occupied =%ld", grid_idx, grid_to_occupied_size);
		return;
	}
	unsigned int voxel_idx = grid_to_occupied[grid_idx];
	if (voxel_idx > plane_voxel_array.size)
	{
		log_info("\t\t GetPointDebugInfo input error voxel_idx  = %d is exceed the size of plane_voxel_array", voxel_idx, plane_voxel_array.size);
		return;
	}
	PlaneVoxelType voxel_type = plane_voxel_array.voxels[voxel_idx].voxel_type;
	unsigned int point_size = plane_voxel_array.voxels[voxel_idx].points.size;
	bool is_being_merged = plane_voxel_array.voxels[voxel_idx].is_being_merged;
	unsigned int parent_voxel_idx = plane_merge_element[voxel_idx].parent_voxel_idx[0];
	if (parent_voxel_idx > plane_voxel_array.size)
	{
		log_info("\t\t GetPointDebugInfo input error parent_voxel_idx  = %d is exceed the size of plane_voxel_array", parent_voxel_idx, plane_voxel_array.size);
		return;
	}
	log_info("");
	log_info("Grid Index = %d, Voxel Index = %d, Parent Voxel Index = %d", grid_idx, voxel_idx, parent_voxel_idx);
	log_info("Voxel Type = %d, Point Size = %d, is_being_merged = %d", voxel_type, point_size, is_being_merged);
	float dist_mse;
	GetVoxelDistMse(voxel_idx,dist_mse);
	float avg_mse = plane_voxel_array.voxels[voxel_idx].avg_mse;
	log_info("Voxel dist_mse = %f, old_mse = %f, high_mse_ratio =%f ", dist_mse, avg_mse, plane_voxel_array.voxels[voxel_idx].plane_high_mse_ratio);

	Point3f normal = plane_voxel_array.voxels[voxel_idx].plane_normal;	
	Point3f center = plane_voxel_array.voxels[voxel_idx].plane_center;
	//log_info("Voxel plane_normal [x,y,z] =[%.10f,%.10f, %.10f]", normal.x, normal.y, normal.z);
	log_info("plane_normal[x,y,z] is[%f,%f,%f]", normal.x, normal.y, normal.z);
	log_info("plane_center[x,y,z] is[%f,%f,%f]", center.x, center.y, center.z);

	unsigned int i;
	double closest_dist = length_x_of_voxel;
	unsigned int closest_index = -1;
	for (i = 0; i < point_size; i++)
	{
		unsigned int point_idx = plane_voxel_array.voxels[voxel_idx].points.point_idx[i];
		const float min_diff = 0.01f;	
		Point3f current_point = pt_cloud_xyz.points[point_idx];
		Point3f delta_point  = current_point- point;
		float current_dist = std::fabs(delta_point.x) + std::fabs(delta_point.y) + std::fabs(delta_point.z);

		if (current_dist < closest_dist)
		{
			closest_dist = current_dist;
			closest_index = point_idx;
		}

		bool condition = (std::fabs(delta_point.x) < min_diff) && (std::fabs(delta_point.y) < min_diff) && (std::fabs(delta_point.z) < min_diff);
		if (condition)
		{
			Point3f point = pt_cloud_xyz.points[point_idx];
			Point3f normal = pt_cloud_normal.points[point_idx];
			log_info("Input Point Index = %d  point [%.10f,%.10f,%.10f]", point_idx, point.x, point.y, point.z);
			log_info("\t\tInput Point point_normal [%.10f,%.10f,%.10f]", normal.x, normal.y, normal.z);
			break;
		}

		if (i >= point_size-1)
		{
			Point3f point = pt_cloud_xyz.points[closest_index];
			Point3f normal = pt_cloud_normal.points[closest_index];
			log_info("Can't find the point in the voxel with diff %f" , min_diff);
			log_info("\t\tClosest from input Point Index = %d  point [%.10f,%.10f,%.10f]", closest_index , point.x, point.y, point.z);
			log_info("\t\tClosest Point point_normal [%.10f,%.10f,%.10f]", normal.x, normal.y, normal.z);
		}
	}

	/*for (unsigned int i = 0; i < point_size; i++)
	{
		log_info("\t\tPoint Index = %d", plane_voxel_array.voxels[voxel_idx].points.point_idx[i]);
	}*/

	if (voxel_idx != parent_voxel_idx)
	{
		PlaneVoxelType parent_voxel_type = plane_voxel_array.voxels[parent_voxel_idx].voxel_type;
		unsigned int parent_point_size = plane_voxel_array.voxels[parent_voxel_idx].points.size;
		bool parent_is_being_merged = plane_voxel_array.voxels[parent_voxel_idx].is_being_merged;

		log_info("Parent Voxel Type = %d, Point Size = %d, is_being_merged = %d", parent_voxel_type, parent_point_size, parent_is_being_merged);
		/*for (unsigned int i = 0; i < parent_point_size; i++)
		{
			log_info("\t\tPoint Index = %d", plane_voxel_array.voxels[parent_voxel_idx].points.point_idx[i]);
		}*/
	}

	log_info("NeighborVoxelDebugInfo:");
	for (int j = 0; j < 26; j++)
	{
		NeighborItem *neighbor_item = &plane_voxel_array.voxels[voxel_idx].neighbors[j];
		if (neighbor_item->neighbor_flag)
		{
			unsigned int neighbor_voxel_idx = neighbor_item->voxel_idx;
			unsigned int neighbor_voxel_type = plane_voxel_array.voxels[neighbor_voxel_idx].voxel_type;
			bool neighbor_is_being_merged = plane_voxel_array.voxels[neighbor_voxel_idx].is_being_merged;
			unsigned int neighbor_parent_voxel_idx = plane_merge_element[neighbor_voxel_idx].parent_voxel_idx[0];
			log_info("\t\tNeighbor Voxel Index = %d, Voxel Type = %d, is_being_merged = %d, Parent Voxel Index = %d", \
				neighbor_voxel_idx, neighbor_voxel_type, neighbor_is_being_merged, neighbor_parent_voxel_idx);
		}
	}
	log_info("");

	return;
}

// get the plane index in plane_merge_out from a parent voxel index,if can't find return false
bool PlaneSegmentation::GetPlaneIdxfromParentVoxel(unsigned int parent_voxel_idx, unsigned int plane_idx)
{
	unsigned int i = 0;
	for (i = 0; i < plane_merge_out.size; i++)
	{
		unsigned int curent_parent_idx = plane_merge_out.planes[i].parent_voxel_idx;
		if (parent_voxel_idx == curent_parent_idx)
		{
			plane_idx = i;
			break;
		}		
	}

	if (i < plane_merge_out.size)
	{
		return true;
	}
	else
	{
		return false;
	}
}


bool PlaneSegmentation::GetVoxelNeigbourDebugInfo(const std::string output_path,VoxelNeighborDebugType type)
{
	std::string file_name;
	std::string folder_path = output_path;
	VoxelNeighborDebugType debug_type = type;
	switch (debug_type)
	{
	case GOOD_VOXEL_DEBUG:
		file_name = folder_path + "good_voxel_neighbour.txt";
		break;
	case PSEUDOBAD_VOXEL_DEBUG:
		file_name = folder_path + "pseudobad_voxel_neighbour.txt";
		break;
	case BAD_VOXEL_DEBUG:
		file_name = folder_path + "bad_voxel_neighbour.txt";
		break;
	case UNMERGED_PSEUDOBAD_VOXEL_DEBUG:
		file_name = folder_path + "unmerged_pseudobad_voxel_neighbour.txt";
		break;
	case UNMERGED_BAD_VOXEL_DEBUG:
		file_name = folder_path +  "unmerged_bad_voxel_neighbour.txt";
		break;
	case ALL_VOXEL_DEBUG:
		file_name = folder_path + "voxel_neighbour.txt";
		break;
	//case BRIDGE_VOXEL_DEBUG:
	//	file_name = folder_path + "bridge_voxel_neighbour.txt";
	//	break;
	case NON_OVERALL_MERGED_VOXEL_DEBUG:
		file_name = folder_path + "non_overall_merged_voxel_neighbour.txt";
		break;
	default:
		file_name = folder_path + "voxel_neighbour.txt";
		break;
	}

	std::ofstream voxel_neigbour(file_name);

	if (debug_type == ALL_VOXEL_DEBUG)
	{
		int num_of_good_voxel = good_voxel_size;
		int num_of_psudobad_voxel = pseudo_bad_voxel_size;
		int num_of_bad_voxel = num_of_occupied_voxel - num_of_good_voxel - num_of_psudobad_voxel;

		voxel_neigbour << "num of occpupied voxel:     " << std::setw(8) << num_of_occupied_voxel << '\n';
		voxel_neigbour << "num of voxel with neighbors:" << std::setw(8) << num_of_occupied_voxel - num_of_bad_voxel << '\n';
		voxel_neigbour << "num of good voxel:          " << std::setw(8) << num_of_good_voxel << '\n';
		voxel_neigbour << "num of pesudo bad voxel:    " << std::setw(8) << num_of_psudobad_voxel << '\n';
		voxel_neigbour << "num of bad voxel:           " << std::setw(8) << num_of_bad_voxel << '\n';
		voxel_neigbour << "num of bridge voxel:        " << std::setw(8) << num_of_bridge << '\n';
		voxel_neigbour << "num of single plane bridge: " << std::setw(8) << num_of_single_bridge << '\n';
		voxel_neigbour << "num of good bridge:		   " << std::setw(8) << num_of_good_bridge << '\n';
		voxel_neigbour << "num of real bad bridge:     " << std::setw(8) << num_of_real_bad_bridge << '\n';
		voxel_neigbour << "num of pesudo bad bridge:   " << std::setw(8) << num_of_pseudo_bad_bridge << '\n';
		voxel_neigbour << "max num of bridge planes :  " << std::setw(8) << max_num_of_bridge_planes << '\n';
	}

	PlaneFitThresholds* plane_fit_thresholds = &plane_seg_params.plane_seg_thresholds;

	for (unsigned int i = 0; i < plane_voxel_array.size; i++)
	{
		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[i];
		PlaneVoxelType  tmp_voxel_type = plane_voxel->voxel_type;
		PlaneMergeItem * plane_merge_item = &plane_merge_element[i];

		bool condition = false;

		//if (plane_voxel->plane_mse >= plane_fit_thresholds->THRESHOLD_MAX_MSE_OF_VOXEL) continue;

		switch (debug_type)
		{
		case GOOD_VOXEL_DEBUG:
			condition = !(tmp_voxel_type == GOOD_VOXEL);
			break;
		case PSEUDOBAD_VOXEL_DEBUG:
			condition = !(tmp_voxel_type == PSEUDO_BAD_VOXEL);
			break;

		case BAD_VOXEL_DEBUG:
			condition = !(tmp_voxel_type == REAL_BAD_VOXEL);
			break;

		case UNMERGED_PSEUDOBAD_VOXEL_DEBUG:
			condition = !((tmp_voxel_type == PSEUDO_BAD_VOXEL) && (!plane_voxel->is_being_merged));
			break;

		case UNMERGED_BAD_VOXEL_DEBUG:
			condition = !((tmp_voxel_type == REAL_BAD_VOXEL) && (!plane_voxel->is_being_merged));
			break;

		//case BRIDGE_VOXEL_DEBUG:
		//	condition = !(plane_merge_item->is_bridge);
		//	break;

		case NON_OVERALL_MERGED_VOXEL_DEBUG:
			condition = plane_voxel->is_overall_merged;
			break;

		case ALL_VOXEL_DEBUG:
			condition =false;
			break;

		default:
			condition = false;
			break;
		}

		if (condition) continue;

		int neighbor_flag_count = 0;
		int badneighbor_flag_count = 0;

		int good_neigbour_idx[26], bad_neigbour_idx[26];
		for (int k = 0; k < 26; k++)
		{
			good_neigbour_idx[k] = -1;
			bad_neigbour_idx[k] = -1;
		}

		for (int k = 0; k < 26; k++)
		{
			NeighborItem* neighbour_item = &plane_voxel->neighbors[k];
			if (!neighbour_item->is_connected) continue;

			if (neighbour_item->neighbor_flag)
			{
				good_neigbour_idx[neighbor_flag_count] = k;
				neighbor_flag_count++;
			}
			else
			{
				bad_neigbour_idx[badneighbor_flag_count] = k;
				badneighbor_flag_count++;
			}
		}

		unsigned int *sorted_good_neighbor_voxel_idx = NULL;
		unsigned int *sorted_bad_neighbor_voxel_idx = NULL;
		// Sort good neighbor according to neighbor voxel index in ascending order
		if (neighbor_flag_count != 0)
		{
			sorted_good_neighbor_voxel_idx = new unsigned int[neighbor_flag_count];
			for (int k = 0; k<neighbor_flag_count; k++)
			{
				NeighborItem* neighbour_item = &plane_voxel->neighbors[good_neigbour_idx[k]];
				sorted_good_neighbor_voxel_idx[k] = neighbour_item->voxel_idx;
			}
			std::sort(sorted_good_neighbor_voxel_idx, sorted_good_neighbor_voxel_idx + neighbor_flag_count);
		}

		// Sort bad neighbor according to neighbor voxel index in ascending order
		if (badneighbor_flag_count != 0)
		{
			sorted_bad_neighbor_voxel_idx = new unsigned int[badneighbor_flag_count];
			for (int k = 0; k<badneighbor_flag_count; k++)
			{
				NeighborItem* neighbour_item = &plane_voxel->neighbors[bad_neigbour_idx[k]];
				sorted_bad_neighbor_voxel_idx[k] = neighbour_item->voxel_idx;
			}
			std::sort(sorted_bad_neighbor_voxel_idx, sorted_bad_neighbor_voxel_idx + badneighbor_flag_count);
		}

		// Update good_neigbour_idx[] and bad_neigbour_idx[] according to neighbor voxel index in ascending order
		for (int k = 0; k < 26; k++)
		{
			NeighborItem* neighbour_item = &plane_voxel->neighbors[k];
			if (!neighbour_item->is_occupied) continue;

			if (neighbour_item->neighbor_flag)
			{
				for (int j = 0; j<neighbor_flag_count; j++)
				{
					if (neighbour_item->voxel_idx == sorted_good_neighbor_voxel_idx[j])
					{
						good_neigbour_idx[j] = k;
					}
				}
			}
			else
			{
				for (int j = 0; j<badneighbor_flag_count; j++)
				{
					if (neighbour_item->voxel_idx == sorted_bad_neighbor_voxel_idx[j])
					{
						bad_neigbour_idx[j] = k;
					}
				}
			}
		}

		voxel_neigbour << "voxel " << i << std::setw(20) << "\tgood neighbours:" << std::setw(8) << neighbor_flag_count << "\t mergeflag:" << std::setw(8) << plane_voxel->is_being_merged \
			<< "\t\t grid_idx:" << std::setw(8) << plane_voxel_array.voxels[i].grid_idx \
			<< "\tvoxel type:" << std::setw(8) << plane_voxel->voxel_type \
			<< "\tis_overall:" << std::setw(8) << plane_voxel->is_overall_merged << "\t\n";

		if (neighbor_flag_count != 0)
		{
			voxel_neigbour << "    idx  " << std::setw(12) << "   normal_diff   " << std::setw(16) << "   plane_dist   " << "   plane_mse   " << "  num_of_point  " << " mergeflag " \
				<< "      grid_idx " << " plane_high_mse_ratio " << " is_connected " <<" is_tiny " <<  " avg_high_mse_ratio " <<"\n";

			for (int k = 0; k<neighbor_flag_count; k++)
			{
				//if (good_neigbour_idx[k] == -1) continue;
				NeighborItem* neighbour_item = &plane_voxel->neighbors[good_neigbour_idx[k]];
				PlaneVoxelItem* neighbor_voxel = &plane_voxel_array.voxels[neighbour_item->voxel_idx];

#ifdef COMPARE_NORMAL_DIFF_IN_ANGLE
				float normal_diff = neighbour_item->normal_diff;
#else
				float normal_diff = float(std::acos(neighbour_item->normal_diff) * 180 / M_PI);
#endif

				//float normal_diff = float(std::acos(neighbour_item->normal_diff)*180/M_PI);
				float plane_dist = neighbour_item->plane_dist;
				float plane_mse = neighbor_voxel->plane_mse;
				unsigned int num_of_point = neighbor_voxel->points.size;
				float smallest_normal_diff = plane_merge_element[neighbour_item->voxel_idx].smallest_normal_diff;
				//float smallest_plane_dist = plane_merge_element[neighbour_item->voxel_idx].smallest_plane_dist;
				float high_mse_ratio = neighbor_voxel->plane_high_mse_ratio;
				//bool is_bridge = plane_merge_element[neighbour_item->voxel_idx].is_bridge;
				bool is_tiny = neighbor_voxel->is_tiny;
				unsigned int grid_idx = neighbor_voxel->grid_idx;
				bool is_connected = neighbour_item->is_connected;
				bool is_being_merged = neighbor_voxel->is_being_merged;
				bool avg_high_mse_ratio = neighbor_voxel->avg_high_mse_ratio;
				voxel_neigbour << ' ' << std::setw(6) << neighbour_item->voxel_idx << std::setw(12) << normal_diff << std::setw(18) << plane_dist \
					<< std::setw(18) << plane_mse << std::setw(10) << num_of_point << std::setw(14) << is_being_merged \
					<< std::setw(18) << grid_idx << std::setw(18) << high_mse_ratio << std::setw(9) << is_connected \
					<< std::setw(14) << is_tiny << std::setw(18) << avg_high_mse_ratio << "\t\n";
			}
		}

		voxel_neigbour << "voxel " << i << std::setw(20) << "\tbad neighbours:\t" << std::setw(8) << badneighbor_flag_count << "\t mergeflag:" << std::setw(8) << plane_voxel->is_being_merged \
			<< "\t grid_idx:" << std::setw(8) << plane_voxel_array.voxels[i].grid_idx \
			<< "\tvoxel type:" << std::setw(8) << plane_voxel->voxel_type \
			<< "\tis_overall:" << std::setw(8) << plane_voxel->is_overall_merged << "\t\n";

		if (badneighbor_flag_count != 0)
		{
			voxel_neigbour << "    idx  " << std::setw(12) << "   normal_diff   " << std::setw(16) << "   plane_dist   " << "   plane_mse   " << "  num_of_point  " << " mergeflag " \
				<< "      grid_idx " << " plane_high_mse_ratio " << " is_connected " << " is_tiny " << " avg_high_mse_ratio " << "\n";

			for (int k = 0; k<badneighbor_flag_count; k++)
			{
				//if (bad_neigbour_idx[k] == -1) continue;
				NeighborItem* neighbour_item = &plane_voxel->neighbors[bad_neigbour_idx[k]];
				PlaneVoxelItem* neighbor_voxel = &plane_voxel_array.voxels[neighbour_item->voxel_idx];
				//float normal_diff = neighbour_item->normal_diff;
#ifdef COMPARE_NORMAL_DIFF_IN_ANGLE
				float normal_diff = neighbour_item->normal_diff;
#else
				float normal_diff = float(std::acos(neighbour_item->normal_diff) * 180 / M_PI);
#endif

				//float normal_diff = float(std::acos(neighbour_item->normal_diff) * 180 / M_PI);
				float plane_dist = neighbour_item->plane_dist;
				float plane_mse = neighbor_voxel->plane_mse;
				unsigned int num_of_point = neighbor_voxel->points.size;
				float smallest_normal_diff = plane_merge_element[neighbour_item->voxel_idx].smallest_normal_diff;
				//float smallest_plane_dist = plane_merge_element[neighbour_item->voxel_idx].smallest_plane_dist;
				float high_mse_ratio = neighbor_voxel->plane_high_mse_ratio;
				//bool is_bridge = plane_merge_element[neighbour_item->voxel_idx].is_bridge;
				bool is_tiny = neighbor_voxel->is_tiny;
				unsigned int grid_idx = neighbor_voxel->grid_idx;
				bool is_connected = neighbour_item->is_connected;
				bool is_being_merged = neighbor_voxel->is_being_merged;
				bool avg_high_mse_ratio = neighbor_voxel->avg_high_mse_ratio;
				voxel_neigbour << ' ' << std::setw(6) << neighbour_item->voxel_idx << std::setw(12) << normal_diff << std::setw(18) << plane_dist \
					<< std::setw(18) << plane_mse << std::setw(10) << num_of_point << std::setw(14) << is_being_merged \
					<< std::setw(18) << grid_idx << std::setw(18) << high_mse_ratio << std::setw(9) << is_connected \
					<< std::setw(14) << is_tiny << std::setw(18) << avg_high_mse_ratio << "\t\n";
			}
		}

		if (neighbor_flag_count != 0)
			delete[] sorted_good_neighbor_voxel_idx;
		if (badneighbor_flag_count != 0)
			delete[] sorted_bad_neighbor_voxel_idx;

	}
	voxel_neigbour.close();
	return true;
}

bool PlaneSegmentation::GetVoxelNomalByMse(void)
{
	unsigned int compute_normal_min_points = plane_seg_params.plane_seg_thresholds.THRESHOLD_MIN_POINT_NUM_OF_VALID_NORMAL_VOXEL;
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(plane_voxel_array.size); i++)
	{
		PlaneVoxelItem *plane_voxel = &plane_voxel_array.voxels[i];
		unsigned int point_center = plane_voxel->points.size / 2;		
		//if (plane_voxel->plane_mse < plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_VOXEL)

		// three cases are processed separetely ( to be improved, we also can get normal from NormaEstimation module escape recomputing)   
		// 1. if the voxel is good normal voxel just sync the normalEstimation module
		// 2 .if the voxel is too few points , compute the average normalized normal as voxel normal
		// 3. if the voxel is not in the above two cases , call egien function to compute the normal 
		
		if(plane_voxel->is_good_normal)
		{
			//get normal from the center point
			unsigned int first_point_idx = plane_voxel->points.point_idx[point_center];
			plane_voxel->plane_normal = pt_cloud_normal.points[first_point_idx];
			float point_size_inverse = 1.0f / plane_voxel->points.size;
			plane_voxel->plane_center.x = static_cast<float>(plane_voxel->sums.sum_x * point_size_inverse);
			plane_voxel->plane_center.y = static_cast<float>(plane_voxel->sums.sum_y * point_size_inverse);
			plane_voxel->plane_center.z = static_cast<float>(plane_voxel->sums.sum_z * point_size_inverse);
		}
		/*else if (plane_voxel->points.size < compute_normal_min_points)
		{
			float point_size_inverse = 1.0f / plane_voxel->points.size;
			plane_voxel->plane_center.x = static_cast<float>(plane_voxel->sums.sum_x * point_size_inverse);
			plane_voxel->plane_center.y = static_cast<float>(plane_voxel->sums.sum_y * point_size_inverse);
			plane_voxel->plane_center.z = static_cast<float>(plane_voxel->sums.sum_z * point_size_inverse);
			Point3f avg_normal = {0.f,0.f,0.f};
			for (unsigned int j = 0; j < plane_voxel->points.size; j++)
			{

				unsigned int point_idx = plane_voxel->points.point_idx[j];
				avg_normal += pt_cloud_normal.points[point_idx];
			}
			MathOperation::VectorNormalized(avg_normal, plane_voxel->plane_normal);
		}*/
		else
		{
			float eigen_mse;
			MathOperation::Compute(plane_voxel->points.size, plane_voxel->sums, plane_voxel->plane_normal, plane_voxel->plane_center, eigen_mse);
		}
	}
	return true;
}

bool PlaneSegmentation::ComputeVoxelAvgNormalMse(void)
{

	#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(plane_voxel_array.size); i++)
	{
		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[i];		
		if (!plane_voxel->is_flat)
		{
			plane_voxel->avg_normal = plane_voxel->plane_normal;
			GetVoxelAvgMse(i);
			continue;
		}
		int point_cnt = plane_voxel->points.size;
		SumforCovariance sums = plane_voxel->sums;
		for (int k = 0; k < 26; k++)
		{
			NeighborItem* neighbor_it = &plane_voxel->neighbors[k];
			if (!neighbor_it->neighbor_flag) continue;
			PlaneVoxelItem* neighbor_voxel = &plane_voxel_array.voxels[neighbor_it->voxel_idx];
			PushSums(&sums, &neighbor_voxel->sums);
			point_cnt += neighbor_voxel->points.size;
		}
		float eigen_mse;
		Point3f center;
		MathOperation::Compute(point_cnt, sums, plane_voxel->avg_normal, center, eigen_mse); // only update normal
		GetVoxelAvgMse(i);
	}
	return true;
}

// Update normal_diff & neighbor_flag according to neighbor's parent after merging
void PlaneSegmentation::UpdateNeighborFlag() {

	PlaneVoxelItem *current_voxel, *neighbour_parent_voxel;
	NeighborItem *neighbour_item;
	unsigned int neighbour_voxel_index, neighbour_parent_voxel_index;
	bool occupy_flag;
	PlaneFitThresholds* plane_fit_thresholds = &plane_seg_params.plane_seg_thresholds;
	for (unsigned int i = 0; i < plane_voxel_array.size; i++)
	{
		current_voxel = &plane_voxel_array.voxels[i];
		VoxelMergeBaseClass occupied_voxel_grid(current_voxel, plane_fit_thresholds);
		for (int j = 0; j < 26; j++)
		{
			neighbour_item = &current_voxel->neighbors[j];
			occupy_flag = neighbour_item->is_occupied;
			if (!occupy_flag) { continue; }

			neighbour_voxel_index = neighbour_item->voxel_idx;
			neighbour_parent_voxel_index = plane_merge_element[neighbour_voxel_index].parent_voxel_idx[0];
			neighbour_parent_voxel = &plane_voxel_array.voxels[neighbour_parent_voxel_index];
			occupied_voxel_grid.UpdateVoxelNeighborItem(neighbour_item, neighbour_parent_voxel);
		}
	}

	return;
}


bool PlaneSegmentation::ClearSums(SumforCovariance *sums)
{
	sums->sum_x = 0.0;
	sums->sum_y = 0.0;
	sums->sum_z = 0.0;
	sums->sum_xx = 0.0;
	sums->sum_yy = 0.0;
	sums->sum_zz = 0.0;
	sums->sum_xy = 0.0;
	sums->sum_xz = 0.0;
	sums->sum_yz = 0.0;
	return true;
}

bool PlaneSegmentation::AssignSums(SumforCovariance *sums_to, SumforCovariance *sums_from)
{
	sums_to->sum_x = sums_from->sum_x; sums_to->sum_y = sums_from->sum_y; sums_to->sum_z = sums_from->sum_z;
	sums_to->sum_xx = sums_from->sum_xx; sums_to->sum_yy = sums_from->sum_yy; sums_to->sum_zz = sums_from->sum_zz;
	sums_to->sum_xy = sums_from->sum_xy; sums_to->sum_xz = sums_from->sum_xz; sums_to->sum_yz = sums_from->sum_yz;
	return true;
}

bool PlaneSegmentation::PushSums(SumforCovariance *sums_to, SumforCovariance *sums_from)
{
	sums_to->sum_x += sums_from->sum_x; sums_to->sum_y += sums_from->sum_y; sums_to->sum_z += sums_from->sum_z;
	sums_to->sum_xx += sums_from->sum_xx; sums_to->sum_yy += sums_from->sum_yy; sums_to->sum_zz += sums_from->sum_zz;
	sums_to->sum_xy += sums_from->sum_xy; sums_to->sum_xz += sums_from->sum_xz; sums_to->sum_yz += sums_from->sum_yz;
	return true;
}

bool PlaneSegmentation::PushPoint(SumforCovariance *sums_to, Point3f point)
{
	sums_to->sum_x += point.x; sums_to->sum_y += point.y; sums_to->sum_z += point.z;
	sums_to->sum_xx += point.x*point.x; sums_to->sum_yy += point.y*point.y; sums_to->sum_zz += point.z*point.z;
	sums_to->sum_xy += point.x*point.y; sums_to->sum_xz += point.x*point.z; sums_to->sum_yz += point.y*point.z;
	return true;
}

bool PlaneSegmentation::PopPoint(SumforCovariance *sums_to, Point3f point)
{
	sums_to->sum_x -= point.x; sums_to->sum_y -= point.y; sums_to->sum_z -= point.z;
	sums_to->sum_xx -= point.x*point.x; sums_to->sum_yy -= point.y*point.y; sums_to->sum_zz -= point.z*point.z;
	sums_to->sum_xy -= point.x*point.y; sums_to->sum_xz -= point.x*point.z; sums_to->sum_yz -= point.y*point.z;
	return true;
}


// get mse of the voxel by compute avg distance with plane 
bool PlaneSegmentation::GetVoxelDistMse(unsigned int voxel_idx, float& dist_mse)
{
	unsigned int plane_high_mse_points_cnt = 0;	
	PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[voxel_idx];
	IntermediateType average_distance = 0.0;
	float high_threshold = plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_VOXEL;
	float plane_dist = 0.f;
	Point3f point;
	for (unsigned int i = 0; i < plane_voxel->points.size; i++)
	{
		point = pt_cloud_xyz.points[plane_voxel->points.point_idx[i]];
		plane_dist =  ComputePointToPlaneDist<float>(point, plane_voxel->plane_normal, plane_voxel->plane_center);
		if (plane_dist > high_threshold) plane_high_mse_points_cnt++;
		average_distance += plane_dist;
	}
	dist_mse =(float)(average_distance/plane_voxel->points.size);
	plane_voxel->plane_high_mse_ratio = float (plane_high_mse_points_cnt*1.0f / plane_voxel->points.size);
	return true;
}

// get mse of the voxel by compute avg distance with plane 
bool PlaneSegmentation::GetVoxelAvgMse(unsigned int voxel_idx)
{
	unsigned int plane_high_mse_points_cnt = 0;
	PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[voxel_idx];
	if (!plane_voxel->is_flat)
	{
		plane_voxel->avg_mse = plane_voxel->plane_mse;
		plane_voxel->avg_high_mse_ratio = plane_voxel->plane_high_mse_ratio;
		return true;
	}
	IntermediateType average_distance = 0.0;
	float high_threshold = plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_VOXEL;
	float plane_dist = 0.f;
	Point3f point;
	for (unsigned int i = 0; i < plane_voxel->points.size; i++)
	{
		point = pt_cloud_xyz.points[plane_voxel->points.point_idx[i]];
		plane_dist = ComputePointToPlaneDist<float>(point, plane_voxel->avg_normal, plane_voxel->plane_center);
		if (plane_dist > high_threshold) plane_high_mse_points_cnt++;
		average_distance += plane_dist;
	}
	plane_voxel->avg_mse = (float)(average_distance / plane_voxel->points.size);
	plane_voxel->avg_high_mse_ratio = float(plane_high_mse_points_cnt * 1.0f / plane_voxel->points.size);
	return true;
}

// get mse of the plane by compute avg distance with plane , mse is get by avg of all voxels mse
bool PlaneSegmentation::GetPlaneDistMseByVoxelAvg(VoxelArray voxels, float& plane_mse)
{
	unsigned int point_cnt = 0;
	IntermediateType average_distance = 0.0;
	Point3f point;
	float dist_mse = 0.f;
	for (unsigned int i = 0; i < voxels.size; i++)
	{
		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[voxels.voxel_idx[i]];
		GetVoxelDistMse(voxels.voxel_idx[i], dist_mse);
		average_distance += dist_mse* plane_voxel->points.size;
		point_cnt += plane_voxel->points.size;
	}
	average_distance = average_distance / point_cnt;
	plane_mse = (float)average_distance;
	return true;
}


// get mse of the plane by compute avg distance with plane , mse is get by avg of all the points in all voxels
bool PlaneSegmentation::GetPlaneDistMseByPointAvg(PlaneVoxelItem *parent_voxel,VoxelArray voxels, float& plane_mse)
{
	unsigned int point_cnt = 0;
	IntermediateType average_distance = 0.0;
	Point3f point;
	float dist_mse = 0.f;
	SumforCovariance sums;

	ClearSums(&sums);
	for (unsigned int i = 0; i < voxels.size; i++)
	{
		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[voxels.voxel_idx[i]];
		point_cnt += plane_voxel->points.size;
		PushSums(&sums,&plane_voxel->sums);
	}

	VoxelMergeBaseClass voxel_grid(parent_voxel, &plane_seg_params.plane_seg_thresholds);
	Point3f plane_normal, plane_center;
	voxel_grid.Compute(point_cnt, sums, plane_normal, plane_center);
	for (unsigned int i = 0; i < voxels.size; i++)
	{
		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[voxels.voxel_idx[i]];
		Point3f point;
		for (unsigned int j = 0; j < plane_voxel->points.size; j++)
		{
			point = pt_cloud_xyz.points[plane_voxel->points.point_idx[j]];
			average_distance +=  ComputePointToPlaneDist<float>(point, plane_normal, plane_center);
		}
	}

	dist_mse = (float)(average_distance / point_cnt);
	plane_mse = (float)dist_mse;

	return true;
}

bool PlaneSegmentation::GetPlaneDistMse(PlaneItem* plane, float& plane_mse)
{
	Point3f point;
	IntermediateType average_distance = 0.0;
	/*for (unsigned int j = 0; j < plane->points.size; j++)
	{
		if ((plane->points.point_idx[j] == 0) || (plane->points.point_idx[j] >= pt_cloud_xyz.size))
		{
			log_info(" points.point_idx[%d] =%d  is zero or exceed point size =%d ",  j, plane->points.point_idx[j], pt_cloud_xyz.size);
		}
	}*/

	for (unsigned int i = 0; i < plane->points.size; i++)
	{
		point = pt_cloud_xyz.points[plane->points.point_idx[i]];
		average_distance +=  ComputePointToPlaneDist<float>(point, plane->plane_normal, plane->plane_center);
	}
	plane_mse = (float)(average_distance / plane->points.size);
	return true;
}

bool PlaneSegmentation::GetPlaneDistMse(PlaneMergeOutputItem * plane, float& plane_mse)
{
	unsigned int point_cnt = 0;
	IntermediateType average_distance = 0.0;
	Point3f point;
	float plane_dist;
	for (unsigned int i = 0; i < plane->voxels.size; i++)
	{
		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[plane->voxels.voxel_idx[i]];
		for (unsigned int i = 0; i < plane_voxel->points.size; i++)
		{
			point = pt_cloud_xyz.points[plane_voxel->points.point_idx[i]];
			plane_dist =  ComputePointToPlaneDist<float>(point, plane->plane_normal, plane->plane_center);
			average_distance += plane_dist;
		}
		point_cnt += plane_voxel->points.size;
	}
	for (unsigned int i = 0; i < plane->extended_part_points.size; i++)
	{
		point = pt_cloud_xyz.points[plane->extended_part_points.point_idx[i]];
		plane_dist =  ComputePointToPlaneDist<float>(point, plane->plane_normal, plane->plane_center);
		average_distance += plane_dist;
		point_cnt++;
	}

	for (unsigned int i = 0; i < plane->points.size; i++)
	{
		point = pt_cloud_xyz.points[plane->points.point_idx[i]];
		plane_dist =  ComputePointToPlaneDist<float>(point, plane->plane_normal, plane->plane_center);
		average_distance += plane_dist;
		point_cnt++;
	}

	//for (unsigned int i = 0; i < plane->multiplane_points.size; i++)
	//{
	//	point = pt_cloud_xyz.points[plane->multiplane_points.point_idx[i]];
	//	plane_dist = ComputePointToPlaneDist<float>(point, plane->plane_normal, plane->plane_center);
	//	average_distance += plane_dist;
	//	point_cnt++;
	//}

	plane_mse = (float)(average_distance / point_cnt);
	return true;
}


bool PlaneSegmentation::OutputPseudoBadMergeLostPlaneInfo(const std::string output_path,PlaneMergeOutputItem *pseudo_bad_planes, unsigned int  pseudo_bad_plane_size)
{

	std::string lost_plane_file = "lost_plane_info.txt";
	std::ofstream lost_plane_info(output_path + lost_plane_file);
	unsigned int tmp_plane_cnt = pseudo_bad_plane_size;

	std::string lost_file_name, file_type;
	std::string  Lost_plane_path = output_path + "lost_plane_xyz\\";
	if (IOData::createDirectory(Lost_plane_path))
	{
		log_error("createDirectory %s failed", Lost_plane_path.c_str());
		return false;
	}
	lost_file_name = "plane_xyz";
	file_type = ".txt";

	std::string lost_points_info_file_name;
	std::string  Lost_points_info_path = output_path + "lost_plane_points_info\\";
	lost_points_info_file_name = "plane_points_info";
	if (IOData::createDirectory(Lost_points_info_path))
	{
		log_error("createDirectory %s failed", Lost_points_info_path.c_str());
		return false;
	}

	float max_voxel_mse = plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_VOXEL;
	unsigned int min_voxel_num_of_plane = plane_seg_params.plane_seg_thresholds.THRESHOLD_MIN_VOXEL_NUM_OF_PLANE;
	unsigned int min_point_num_of_plane = plane_seg_params.plane_seg_thresholds.THRESHOLD_MIN_POINT_NUM_OF_PLANE;

	unsigned int total_lost_planes = 0;
	for (unsigned int i = 0; i < tmp_plane_cnt; i++)
	{
		unsigned int parent_voxel_idx = pseudo_bad_planes[i].parent_voxel_idx;
		PlaneVoxelType voxel_type = plane_voxel_array.voxels[parent_voxel_idx].voxel_type;
		if (voxel_type != GOOD_VOXEL)
		{
			float plane_mse;
			//GetPlaneDistMseByVoxelAvg(pseudo_bad_planes[i].voxels, plane_mse);
			GetPlaneDistMseByPointAvg(&plane_voxel_array.voxels[parent_voxel_idx], pseudo_bad_planes[i].voxels, plane_mse);
			if ((plane_mse > max_voxel_mse) || (pseudo_bad_planes[i].voxels.size <= min_voxel_num_of_plane)\
				|| pseudo_bad_planes[i].total_point_cnt < min_point_num_of_plane)
			{
				total_lost_planes++;
			}
		}
	}
	lost_plane_info << "Total number of lost planes: " << total_lost_planes << '\n';

	unsigned int lost_plane_idx = 0;
	for (unsigned int i = 0; i < tmp_plane_cnt; i++)
	{
		unsigned int parent_voxel_idx = pseudo_bad_planes[i].parent_voxel_idx;
		PlaneVoxelType voxel_type = plane_voxel_array.voxels[parent_voxel_idx].voxel_type;
		if (voxel_type != GOOD_VOXEL)
		{
			float plane_mse, avg_mse,point_avg_mse;
			float max_voxel_mse = plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_VOXEL;
			unsigned int min_voxel_num_of_plane = plane_seg_params.plane_seg_thresholds.THRESHOLD_MIN_VOXEL_NUM_OF_PLANE;
			GetPlaneDistMseByVoxelAvg(pseudo_bad_planes[i].voxels, plane_mse);
			GetPlaneDistMseByPointAvg(&plane_voxel_array.voxels[parent_voxel_idx], pseudo_bad_planes[i].voxels, point_avg_mse);
			avg_mse = plane_voxel_array.voxels[parent_voxel_idx].avg_mse;
			
			if ((point_avg_mse > max_voxel_mse) || (pseudo_bad_planes[i].voxels.size <= min_voxel_num_of_plane)||pseudo_bad_planes[i].total_point_cnt < min_point_num_of_plane)
			{
				//lost_plane_info << "Plane parent_voxel_idx=" << parent_voxel_idx << ", plane mse:" << plane_mse << ", voxels.size:" << pseudo_bad_planes[i].voxels.size << "will give up" << std::endl;
				lost_plane_info << "Plane Index =" << lost_plane_idx << ", Parent Voxel Index =" << parent_voxel_idx << ", Parent Voxel Type =" << voxel_type << ", Voxel Count ="\
					<< pseudo_bad_planes[i].voxels.size << ", Plane Mse =" << plane_mse <<", eigen Mse ="<< avg_mse <<",Plane mse2 ="<< point_avg_mse << ", Plane normal =" << plane_voxel_array.voxels[parent_voxel_idx].plane_normal<<'\n';

				lost_plane_info<< "\t  voxel idx \t" <<"voxel mse\t"<<"eigen mse\t"<<"voxel normal\t"<<std::endl;
				for (unsigned int j = 0; j<pseudo_bad_planes[i].voxels.size; j++)
				{
					unsigned int voxel_idx = pseudo_bad_planes[i].voxels.voxel_idx[j];
					float voxel_mse;
					GetVoxelDistMse(voxel_idx, voxel_mse);
					Point3f normal = plane_voxel_array.voxels[voxel_idx].plane_normal;
					lost_plane_info <<j<<"\t\t"<< voxel_idx << "\t\t" << voxel_mse << "\t\t" <<plane_voxel_array.voxels[voxel_idx].avg_mse << "\t\t" << normal << std::endl;
				}

				PointInVoxelArray plane_points;
				// Assign parent voxel index for the points in good and pseudo bad voxels
				unsigned int point_in_plane_cnt = 0;
				for (unsigned int j = 0; j < pseudo_bad_planes[i].voxels.size; j++) {

					unsigned int voxel_in_plane_idx = pseudo_bad_planes[i].voxels.voxel_idx[j];
					unsigned int point_in_voxel_cnt = plane_voxel_array.voxels[voxel_in_plane_idx].points.size;
				    point_in_plane_cnt += point_in_voxel_cnt;
				}

				std::stringstream lost_plane_id;
				lost_plane_id << lost_plane_idx;
				std::string whole_path = Lost_plane_path + lost_file_name + lost_plane_id.str() + file_type;
				std::string lost_points_info_file = Lost_points_info_path + lost_points_info_file_name + lost_plane_id.str() + file_type;
				std::ofstream lost_plane_points_info(lost_points_info_file);

				plane_points.size = point_in_plane_cnt;
				plane_points.point_idx = new unsigned int[point_in_plane_cnt];

				point_in_plane_cnt = 0;
				for (unsigned int j = 0; j < pseudo_bad_planes[i].voxels.size; j++) 
				{
					unsigned int voxel_in_plane_idx = pseudo_bad_planes[i].voxels.voxel_idx[j];
					unsigned int point_in_voxel_cnt = plane_voxel_array.voxels[voxel_in_plane_idx].points.size;
					for (unsigned int k = 0; k < point_in_voxel_cnt; k++)
					{
						plane_points.point_idx[point_in_plane_cnt] = plane_voxel_array.voxels[voxel_in_plane_idx].points.point_idx[k];
						unsigned int point_idx = plane_points.point_idx[point_in_plane_cnt];
						lost_plane_points_info<<"Voxel Index = " << voxel_in_plane_idx << ", Point Index = " << point_idx << ", [" << pt_cloud_xyz.points[point_idx].x;
						lost_plane_points_info << ", " << pt_cloud_xyz.points[point_idx].y;
						lost_plane_points_info << ", " << pt_cloud_xyz.points[point_idx].z << "]\n";
						point_in_plane_cnt++;
					}
				}
				FeaturesIO::SavePoint3fData(whole_path, pt_cloud_xyz, plane_points);

				delete[] plane_points.point_idx;
				plane_points.point_idx = NULL;
				lost_plane_points_info.close();
				lost_plane_idx++;
			}
		}

	}

	lost_plane_info.close();
	return true;
}

void PlaneSegmentation::VoxelPoint(unsigned int voxel_idx) 
{

		unsigned int point_size = plane_voxel_array.voxels[voxel_idx].points.size;
		std::cout << "voxel idx: " << voxel_idx  <<"   voxel points size= "<<point_size<< std::endl;
		for (unsigned int k = 0; k < point_size; k++)
		{
			unsigned int point_idx = plane_voxel_array.voxels[voxel_idx].points.point_idx[k];
			Point3f Point = pt_cloud_xyz.points[point_idx];
			std::cout << "point idx: " << point_idx << "Point =" << Point << std::endl;
		}		
}


// debug for save voxel points by voxel name 
bool PlaneSegmentation::SaveVoxelPointsDebug(const std::string output_path,unsigned int voxel_idx)
{
	std::string file_type;
	std::string voxel_file_name = "voxel_xyz";
	std::stringstream voxel_id;
	std::string voxel_ouput_path = output_path+"voxel_xyz\\";
	if (IOData::createDirectory(voxel_ouput_path))
	{
		log_error("createDirectory %s failed", voxel_ouput_path.c_str());
		return false;
	}
	file_type = ".txt";
	PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[voxel_idx];
	voxel_id << voxel_idx;
	std::string voxel_file = voxel_ouput_path + voxel_file_name + voxel_id.str() + file_type;
	FeaturesIO::SavePoint3fData(voxel_file, pt_cloud_xyz, plane_voxel->points);
	return true;
}

bool PlaneSegmentation::SaveBadVoxelPointsClassify(const std::string output_path, unsigned int voxel_idx)
{
	if (voxel_idx > plane_voxel_array.size)
	{
		log_error("voxel_idx = %d is exceed size of voxel array = %d", voxel_idx, plane_voxel_array.size);
		return false;
	}

	unsigned int bad_voxel_idx = voxel_idx_to_bad_voxel[voxel_idx];
	if (bad_voxel_idx > point_merged_voxel_size)
	{
		log_error("bad_voxel_idx = %d is exceed size of voxel array = %d", bad_voxel_idx, point_merged_voxel_size);
		return false;
	}

	std::string file_type;
	std::string voxel_xyz_name = "voxel_xyz_";
	std::stringstream voxel_id;
	std::stringstream plane_id;
	std::string voxel_xyz_path = output_path + "voxel_xyz_by_plane\\";
	std::string voxel_info_path = output_path + "voxel_xyz_info\\";

	if (IOData::createDirectory(voxel_info_path))
	{
		log_error("createDirectory %s failed", voxel_info_path.c_str());
		return false;
	}
	if (IOData::createDirectory(voxel_xyz_path))
	{
		log_error("createDirectory %s failed", voxel_xyz_path.c_str());
		return false;
	}
	file_type = ".txt";
	voxel_id << voxel_idx;

	std::string voxel_info_name = voxel_info_path + "voxel_" + voxel_id.str() + "info"+file_type;

	std::ofstream voxel_debug_info(voxel_info_name);

	PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[voxel_idx];
	BadVoxelMergeOutpuItem* bad_voxel_it = &bad_voxel_merge_item[bad_voxel_idx];


	voxel_debug_info << "points size: " << plane_voxel->points.size << std::endl;
	voxel_debug_info << "voxel_type: " << plane_voxel->voxel_type << std::endl;
	voxel_debug_info << "grid_idx: " << plane_voxel->grid_idx << std::endl;
	voxel_debug_info << "plane_mse: " << plane_voxel->plane_mse << std::endl;
	voxel_debug_info << "high_mse_ratio: " << plane_voxel->plane_high_mse_ratio << std::endl;
	voxel_debug_info << "bad_voxel_idx: " << bad_voxel_idx << std::endl;
	voxel_debug_info << "num_of_points_merged: " << bad_voxel_it->num_of_points_merged << std::endl;
	for (int i = 0; i < 26; i++)
	{ 
		NeighborItem* neighbor_it = &plane_voxel->neighbors[i];
		if (neighbor_it->is_connected)
		{
			PlaneVoxelItem* neighbor_voxel = &plane_voxel_array.voxels[neighbor_it->voxel_idx];
			PlaneMergeItem* neighbor_merge_it = &plane_merge_element[neighbor_it->voxel_idx];

			std::vector<unsigned int> plane_idx;
			if((neighbor_voxel->is_overall_merged)&&(neighbor_voxel->is_being_merged))
			{
				unsigned int plane_idx = voxel_to_plane_idx[neighbor_merge_it->parent_voxel_idx[0]];
			}

			voxel_debug_info << "connected voxel: " << neighbor_it->voxel_idx\
			<< "  voxel_type: " << neighbor_voxel->voxel_type;

			if ((neighbor_voxel->is_overall_merged) && (neighbor_voxel->is_being_merged))
			{
				unsigned int plane_idx = voxel_to_plane_idx[neighbor_merge_it->parent_voxel_idx[0]];
				voxel_debug_info << "  plane_idx: " << plane_idx;
			}
			voxel_debug_info<< std::endl;
		}
	
	}
	unsigned int plane_size = plane_merge_out.size + points_group_plane_merge_out.size;
	std::vector<std::vector<unsigned int>> plane_point_idx_vec(plane_size);
	std::vector<unsigned int> voxel_missing_points(0);
	for (unsigned int i = 0; i < plane_voxel->points.size;i++)
	{
		unsigned int point_idx = plane_voxel->points.point_idx[i];
		if (bad_voxel_it->point_merged_flag[i])
		{
			unsigned int plane_idx = bad_voxel_it->closest_plane_idx[i];
			if (plane_idx < plane_size)
			{
				plane_point_idx_vec[plane_idx].push_back(point_idx);
			}
			else
			{
				log_error("plane_idx % exceed size of total plane =%d", plane_idx,plane_size);
			}
		}
		else
		{
			voxel_missing_points.push_back(point_idx);
		}
	}

	for (int i = 0; i < plane_point_idx_vec.size(); i++)
	{
		if (plane_point_idx_vec[i].size() != 0)
		{
			voxel_debug_info << "plane idx: " << i << "   points size: " << plane_point_idx_vec[i].size() << std::endl;
			plane_id << i;
			std::string voxel_xyz_byplane_file = voxel_xyz_path + voxel_xyz_name + voxel_id.str()+"_plane_"+ plane_id.str() + file_type;
			FeaturesIO::SavePoint3fData(voxel_xyz_byplane_file, pt_cloud_xyz, plane_point_idx_vec[i]);
			plane_id.clear();
			plane_id.str("");
		}
	}

	voxel_debug_info << "missing points size: " << voxel_missing_points.size() << std::endl;
	voxel_debug_info.close();
	if (voxel_missing_points.size() != 0)
	{
		std::string voxel_xyz_byplane_file = voxel_xyz_path + voxel_xyz_name + voxel_id.str() + "_missingpoints" + file_type;
		FeaturesIO::SavePoint3fData(voxel_xyz_byplane_file, pt_cloud_xyz, voxel_missing_points);
	}
	return true;
}

//debug for save plane points by voxel name and plane_idx
bool PlaneSegmentation::SavePlanePointsDebug(const std::string output_path, unsigned int plane_idx, bool is_discete_combined)
{

	std::string file_type;
	std::string plane_file_name = "plane_idx_xyz";
	std::string voxel_ouput_path = output_path + "voxel_xyz\\";
	if (IOData::createDirectory(voxel_ouput_path))
	{
		log_error("createDirectory %s failed", voxel_ouput_path.c_str());
		return false;
	}
	file_type = ".txt";
	std::stringstream voxel_id, plane_id;
	if (!is_discete_combined)
	{
		if (plane_idx >= plane_merge_out.size)
		{

			unsigned int bad_plane_idx = plane_idx - plane_merge_out.size;
			PlaneMergeOutputItem* plane_it = &points_group_plane_merge_out.planes[bad_plane_idx];
			for (unsigned int j = 0; j < plane_it->voxels.size; j++)
			{
				unsigned int voxel_idx = plane_it->voxels.voxel_idx[j];
				SaveVoxelPointsDebug(output_path, voxel_idx);
			}
			return true;
		}
	}
	PlaneMergeOutputItem* plane_it = &plane_merge_out.planes[plane_idx];
	plane_id << plane_idx;
	std::string plane_file = voxel_ouput_path + plane_file_name + plane_id.str() + file_type;

	for (unsigned int j = 0; j < plane_it->voxels.size; j++)
	{
		unsigned int voxel_idx = plane_it->voxels.voxel_idx[j];
		SaveVoxelPointsDebug(output_path, voxel_idx);
	}
	FeaturesIO::SavePoint3fData(plane_file, pt_cloud_xyz, plane_it->points);
	return true;
}

bool PlaneSegmentation::SavePlanePointsByVoxelType(const std::string output_path, const unsigned int plane_idx, const PlaneVoxelType voxel_type)
{
	std::string file_type;
	
	std::string voxel_ouput_path = output_path + "plane_xyz\\";
	if (IOData::createDirectory(voxel_ouput_path))
	{
		log_error("createDirectory %s failed", voxel_ouput_path.c_str());
		return false;
	}

	if (plane_idx >= plane_merge_out.size)
	{
		log_error("SavePlanePointsByVoxelType input plane_idx%d exceed size of generated planes =%d", plane_idx, plane_merge_out.size);
		return false;
	}

	//log_info("SavePlanePointsByVoxelType 00");
	std::string plane_file_name;
	if (voxel_type == GOOD_VOXEL)
	{
		plane_file_name = "plane_good_voxel_xyz";
	}
	else
	{
		plane_file_name = "plane_pseudobad_voxel_xyz";

	}
	file_type = ".txt";
	std::stringstream plane_id;
	plane_id << plane_idx;
	std::string plane_file = voxel_ouput_path + plane_file_name + plane_id.str() + file_type;

	PointInVoxelArray points;
	points.point_idx = NULL;
	points.size = 0;
	PlaneMergeOutputItem* plane_it = &plane_merge_out.planes[plane_idx];
	for (unsigned int i = 0; i < plane_it->voxels.size; i++)
	{
		int voxel_idx = plane_it->voxels.voxel_idx[i];
		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[voxel_idx];
		if (plane_voxel->voxel_type != voxel_type) continue;
		points.size += plane_voxel->points.size;
	}
	if (points.size > 0)
	{
		points.point_idx = new unsigned int[points.size];
		int point_cnt = 0;
		for (unsigned int i = 0; i < plane_it->voxels.size; i++)
		{
			int voxel_idx = plane_it->voxels.voxel_idx[i];
			PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[voxel_idx];
			if (plane_voxel->voxel_type != voxel_type) continue;
			memcpy(&points.point_idx[point_cnt], plane_voxel->points.point_idx, sizeof(unsigned int)*plane_voxel->points.size);
			point_cnt += plane_voxel->points.size;
		}
		FeaturesIO::SavePoint3fData(plane_file, pt_cloud_xyz, points);
	}

	if (points.point_idx != NULL)
	{
		delete[]points.point_idx;
		points.point_idx = NULL;
	}
	
	return true;

}


//debug for save plane extended and edge points by  plane_idx
bool PlaneSegmentation::SavePlaneEdgePointsDebug(const std::string output_path, unsigned int plane_idx)
{

	std::string file_type;
	std::string plane_ouput_path = output_path + "plane_xyz\\";
	if (IOData::createDirectory(plane_ouput_path))
	{
		log_error("createDirectory %s failed", plane_ouput_path.c_str());
		return false;
	}
	file_type = ".txt";
	PlaneMergeOutputItem* plane_it = &plane_merge_out.planes[plane_idx];
	if (plane_it->points.size > 0)
	{
		std::stringstream plane_id;
		plane_id << plane_idx;
		std::string plane_file_name = "plane_edg_xyz_";
		std::string plane_file = plane_ouput_path + plane_file_name + plane_id.str() + file_type;
		FeaturesIO::SavePoint3fData(plane_file, pt_cloud_xyz, plane_it->points);
	}
	if (plane_it->extended_part_points.size > 0)
	{
		std::stringstream plane_id;
		plane_id << plane_idx;
		std::string plane_file_name = "plane_extended_xyz_";
		std::string plane_file = plane_ouput_path + plane_file_name + plane_id.str() + file_type;
		FeaturesIO::SavePoint3fData(plane_file, pt_cloud_xyz, plane_it->extended_part_points);
	}

	if (plane_it->multiplane_points.size > 0)
	{
		std::string plane_file_name = "multi_plane_edge_xyz_";
		std::stringstream plane_id;
		plane_id << plane_idx;
		std::string plane_file = plane_ouput_path + plane_file_name + plane_id.str() + file_type;
		FeaturesIO::SavePoint3fData(plane_file, pt_cloud_xyz, plane_it->multiplane_points);
	}

	if (plane_it->plane_type == GOOD_PLANE)
	{
		SavePlanePointsByVoxelType(output_path, plane_idx, PSEUDO_BAD_VOXEL);
	}
	return true;
}

bool PlaneSegmentation::GetVoxelNeighbourinfo(unsigned int voxel_idx)
{
	for (unsigned int i = 0; i < plane_voxel_array.size; i++)
	{
		bool debug_con = i == voxel_idx;
		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[i];
		if (debug_con)
		{
			log_info("voxel %d voxel_type %d is_overall_merged=%d", i, plane_voxel->voxel_type, plane_voxel->is_overall_merged);

			unsigned int bad_voxel_idx = voxel_idx_to_bad_voxel[i];
			if (bad_voxel_idx != std::numeric_limits<unsigned int>::max())
			{
				BadVoxelMergeOutpuItem * bad_voxel_it = &bad_voxel_merge_item[bad_voxel_idx];
				log_info("voxel %d bad_voxel_idx =%d bad_voxel_it-voxel %d", i, bad_voxel_idx,bad_voxel_it->bad_voxel_idx);

				for (unsigned int j = 0; j < plane_merge_out.size; j++)
				{
					log_info("remaining_points_merged_plane[%d] = %d", j, bad_voxel_it->remaining_points_merged_plane[j]);
				}
				
			}
			else
			{
				log_info("voxel %d not in bad voxels");
			}
			
			
			for (unsigned int j = 0; j < 26; j++)
			{
				NeighborItem*neighbor_it = &plane_voxel->neighbors[j];
				if (!neighbor_it->is_occupied) continue;
				PlaneVoxelItem* neighbour_plane_voxel = &plane_voxel_array.voxels[neighbor_it->voxel_idx];
				log_info("voxel %d neigbour-voxel %d is_being_merged=%d is_overall_merged=%d", i, neighbor_it->voxel_idx, neighbour_plane_voxel->is_being_merged, neighbour_plane_voxel->is_overall_merged);
			}
		}

		unsigned int parent_voxel_idx = plane_merge_element[i].parent_voxel_idx[0];
		if (debug_con) log_info("05voxel %d is_overall_merged %d parent_voxel_idx=%d  is_being_merged=%d", i, plane_voxel->is_overall_merged, parent_voxel_idx, plane_voxel->is_being_merged);
	}
	return true;
}

bool PlaneSegmentation::SetConfigure(const SysCntrlParams sys_cntrl_param)
{
	sys_control_para = sys_cntrl_param;
	return true;
}
bool PlaneSegmentation::SetConfigure(const SegCntrlParams config_para)
{
	config_params = config_para;
	return true;
}
bool PlaneSegmentation::SetConfigure(const PlaneFitThresholds plane_seg_thrshld)
{
	plane_seg_params.plane_seg_thresholds = plane_seg_thrshld;
	return true;
}

bool PlaneSegmentation::SetConfigure(const VoxelParams voxel_para)
{
	plane_seg_params.voxel_params = voxel_para;
	return true;
}

bool PlaneSegmentation::SetInputPoints(const PointArray input_points)
{
	pt_cloud_xyz = input_points;
	return true;
}

bool PlaneSegmentation::SetInputPointsNormal(const PointArray input_normals)
{
	pt_cloud_normal = input_normals;
	return true;
}

bool PlaneSegmentation::SetGridToOccupiedIdx(const std::vector<unsigned int> grid_to_occupied_voxel_idx)
{
	grid_to_occupied_size = static_cast<unsigned int>(grid_to_occupied_voxel_idx.size());
	grid_to_occupied = new unsigned int[grid_to_occupied_size];
	for (unsigned int i = 0; i < grid_to_occupied_size; i++)
	{
		grid_to_occupied[i] = grid_to_occupied_voxel_idx[i];
	}
	return true;
}



#ifdef SAVE_OUTPUT_FILE_DEBUG
bool PlaneSegmentation::SetConfigure(const DebugConfigParams debug_config_para)
{
	debug_config_params = debug_config_para;
	return true;
}
#endif

bool PlaneSegmentation::Identify2VoxelCLoseByDistance(const unsigned int first_voxel, const unsigned int sec_voxel, const float distance)
{
	// step 1: get points center of 2 points group
	if ((first_voxel >= plane_voxel_array.size) || (sec_voxel >= plane_voxel_array.size))
	{
		log_error("input error first_voxel = %d sec_voxel =%d", first_voxel, sec_voxel);
		return false;
	}

	PlaneVoxelItem *first_plane_voxel = &plane_voxel_array.voxels[first_voxel];
	PlaneVoxelItem * sec_plane_voxel = &plane_voxel_array.voxels[sec_voxel];

	// step 2:  sort 2 points by distance to the center of 2 points group
	std::vector <PointIdx2Dist> first_idx_to_dist(first_plane_voxel->points.size);
	for (size_t i = 0; i < first_plane_voxel->points.size; i++)
	{
		Point3f point = pt_cloud_xyz.points[first_plane_voxel->points.point_idx[i]];
		first_idx_to_dist[i].cloud_point_idx = first_plane_voxel->points.point_idx[i];
		Point3f delta_point = point - sec_plane_voxel->plane_center;
		float distance = std::fabs(delta_point.x) + std::fabs(delta_point.y) + std::fabs(delta_point.z);
		first_idx_to_dist[i].delta_dist = distance;
	}
	std::sort(first_idx_to_dist.begin(), first_idx_to_dist.end(), std::less<PointIdx2Dist>{});

	std::vector <PointIdx2Dist>sec_idx_to_dist(sec_plane_voxel->points.size);
	for (size_t i = 0; i < sec_plane_voxel->points.size; i++)
	{
		Point3f point = pt_cloud_xyz.points[sec_plane_voxel->points.point_idx[i]];
		sec_idx_to_dist[i].cloud_point_idx = sec_plane_voxel->points.point_idx[i];
		Point3f delta_point = point - first_plane_voxel->plane_center;
		float distance = std::fabs(delta_point.x) + std::fabs(delta_point.y) + std::fabs(delta_point.z);
		sec_idx_to_dist[i].delta_dist = distance;
	}
	std::sort(sec_idx_to_dist.begin(), sec_idx_to_dist.end(), std::less<PointIdx2Dist>{});

	// step 3:   each group use 3 closest points  to  compute center distance, in 3D space only need 3  points to identify close of 2 points group 
	int first_comp_cnt = 2;
	int sec_comp_cnt = 2;
	if (first_comp_cnt > static_cast<int>(first_plane_voxel->points.size))  first_comp_cnt = static_cast<int>(first_plane_voxel->points.size);
	if (sec_comp_cnt > static_cast<int>(sec_plane_voxel->points.size))  sec_comp_cnt = static_cast<int>(sec_plane_voxel->points.size);

	Point3f first_comp_center = { 0.f,0.f,0.f };
	//std::vector <unsigned int> first_close_points(first_comp_cnt);
	for (int i = 0; i < first_comp_cnt; i++)
	{
		first_comp_center += pt_cloud_xyz.points[first_idx_to_dist[i].cloud_point_idx];
		//first_close_points[i] = first_idx_to_dist[i].cloud_point_idx;

	}
	first_comp_center = first_comp_center / first_comp_cnt;

	Point3f sec_comp_center = { 0.f,0.f,0.f };
	//std::vector <unsigned int> sec_close_points(sec_comp_cnt);

	for (int i = 0; i < sec_comp_cnt; i++)
	{
		sec_comp_center += pt_cloud_xyz.points[sec_idx_to_dist[i].cloud_point_idx];
		//sec_close_points[i] = sec_idx_to_dist[i].cloud_point_idx;
	}
	sec_comp_center = sec_comp_center / sec_comp_cnt;

	float dist =  ComputePointToPointDist<float>(first_comp_center, sec_comp_center);
	bool rtn = false;
	if (dist < distance) rtn = true;
	return rtn;
}


void PlaneSegmentation::PointsGroupPlaneMergeFromRemainer()
{
	BadVoxelConnectedUpdate();
	PointsGroupPlane points_group_plane(this);
	//points_group_plane.PointsGroupPlaneMergeFromRemainer(true, &plane_merge_out, &points_group_extended_planes);
	//log_info("points_group_extended_planes size = %d", points_group_extended_planes.size);
	points_group_plane.PointsGroupPlaneMergeFromRemainer(false, NULL, &points_group_remaning_planes);
	log_info("points_group_remaning_planes size = %d", points_group_remaning_planes.size);
	points_group_plane_merge_out.size = points_group_remaning_planes.size;
	log_info("points_group_plane_size =%d", points_group_plane_merge_out.size);
	if (points_group_plane_merge_out.size > 0)
	{
		points_group_plane_merge_out.planes = new PlaneMergeOutputItem[points_group_plane_merge_out.size];		
		for (unsigned int i = 0; i < points_group_plane_merge_out.size; i++)
		{
			PlaneMergeOutputItem* group_merge_out_it = &points_group_plane_merge_out.planes[i];
			group_merge_out_it->plane_type = REAL_BAD_PLANE;
			group_merge_out_it->voxels.size = 0;
			group_merge_out_it->points.size = 0;
			group_merge_out_it->voxels.voxel_idx = NULL;
			group_merge_out_it->points.point_idx = NULL;

			PointsGroupParentPlaneItem * points_group_org_it;
			points_group_org_it = &points_group_remaning_planes.planes[i];
			group_merge_out_it->parent_voxel_idx = points_group_org_it->parent_voxel_idx;
			group_merge_out_it->extended_part_points = points_group_org_it->points;
			//group_merge_out_it->points = points_group_org_it->bad_group_points;
			group_merge_out_it->total_point_cnt = points_group_org_it->points.size;
			MathOperation::AssignSums(&group_merge_out_it->sums, &points_group_org_it->sums);
			group_merge_out_it->plane_mse = points_group_org_it->plane_mse;
			group_merge_out_it->plane_normal = points_group_org_it->plane_normal;
			group_merge_out_it->plane_center = points_group_org_it->plane_center;
			group_merge_out_it->multiplane_points.size = 0;
			group_merge_out_it->multiplane_points.point_idx = NULL;
		}
	}
}



bool PlaneSegmentation::IdentifyParentOfVoxelsWithRefPlane(PlaneVoxelType voxeltype)
{

	unsigned char update_flag;					// global update flag for each iterations
	unsigned char* update_flag_array = NULL;				// update flag for all occupied voxels
	unsigned char curr_ping_pong_buf_idx;				// index to parent_voxel_idx[2] in current iteration
	unsigned char next_ping_pong_buf_idx;				// index to parent_voxel_idx[2] in next iteration
	unsigned int iteration_count;
	iteration_count = 0;
	update_flag_array = new unsigned char[plane_voxel_array.size];
	do
	{
		// Update current parent voxel index from neighbor's parent voxel index
		curr_ping_pong_buf_idx = iteration_count % 2;
		next_ping_pong_buf_idx = (iteration_count + 1) % 2;

//#pragma omp parallel for
		for (int i = 0; i < static_cast<int>(plane_voxel_array.size); i++) {

			update_flag_array[i] = 0;
			PlaneVoxelItem *plane_voxel = &plane_voxel_array.voxels[i];
			PlaneMergeItem *plane_merge_it = &plane_merge_element[i];
			if (plane_voxel->ref_plane_cnt == 0) continue;
			if ((plane_voxel->voxel_type != voxeltype) || (plane_voxel->is_being_merged)) continue;

			unsigned int current_parent_voxel_idx = plane_merge_it->parent_voxel_idx[curr_ping_pong_buf_idx];
			for (int j = 0; j < 26; j++) {

				NeighborItem * neighbor_it = &plane_voxel->neighbors[j];

				if (!neighbor_it->is_connected) continue;

				PlaneVoxelItem *neighbor_voxel = &plane_voxel_array.voxels[neighbor_it->voxel_idx];
				PlaneMergeItem *neighbor_merge_it = &plane_merge_element[neighbor_it->voxel_idx];

				if ((neighbor_voxel->voxel_type == voxeltype) && (neighbor_voxel->ref_plane_cnt == 0)) continue;
				if ((neighbor_voxel->voxel_type == voxeltype) && (neighbor_voxel->is_being_merged)) continue;
				//if ((neighbor_voxel->voxel_type == voxeltype) && (!neighbor_voxel->is_same_with_ref_plane[plane_idx])) continue;
				if (neighbor_voxel->voxel_type != voxeltype)
				{
					// if the neighbor is good voxels ,if its plane is not same with current voxel reference plane, neighbor parent voxel will not participate passing on parent voxel
					unsigned int plane_idx = voxel_to_plane_idx[neighbor_merge_it->parent_voxel_idx[0]];
					if (plane_idx >= plane_merge_out.size) continue;
					if (!plane_voxel->is_same_with_ref_plane[plane_idx]) continue;
				}
				else
				{
					// if the neighbor is pseudobad voxels ,if has no same reference plane, neighbor parent voxel will not participate passing on parent voxel
					unsigned int same_plane_idx_cnt = 0;
					for (unsigned int k = 0; k < plane_merge_out.size; k++)
					{
						if (plane_voxel->is_same_with_ref_plane[k] && neighbor_voxel->is_same_with_ref_plane[k]) same_plane_idx_cnt++;
					}
					if (same_plane_idx_cnt == 0) continue;
				}

				//parent_identify_update_item->neighbor_parent_voxel_idx = plane_merge_element[neighbor_voxel_idx].parent_voxel_idx[next_ping_pong_buf_idx];
				unsigned int neighbor_parent_voxel_idx = neighbor_merge_it->parent_voxel_idx[next_ping_pong_buf_idx];
				if (neighbor_parent_voxel_idx == current_parent_voxel_idx) continue;

				PlaneVoxelType neighbor_parent_voxel_type = plane_voxel_array.voxels[neighbor_parent_voxel_idx].voxel_type;

				//if (neighbor_parent_voxel_type != voxeltype) continue;

				ParentIDCondType condition[5] = { GOOD_NEIGHBOR_CNT,BAD_NEIGHBOR_CNT, PLANE_MSE,GRID_IDX,INVALID_PARA };
				bool check_condition = ParentVoxelIdentifyCondition(condition, current_parent_voxel_idx, neighbor_parent_voxel_idx, update_flag_array[i]);
				if (!check_condition) continue;
				if (update_flag_array[i] == 1)
				{
					plane_merge_element[i].parent_voxel_idx[curr_ping_pong_buf_idx] = neighbor_parent_voxel_idx;
				}
			}
		}

		// Check if no more updates
		update_flag = 0;
		for (unsigned int i = 0; i < plane_voxel_array.size; i++) {

			update_flag |= update_flag_array[i];
		}

		// Increment the iteration number
		iteration_count++;

	} while (update_flag > 0);

	// Release memory
	if (update_flag_array != NULL){
		delete[] update_flag_array;
		update_flag_array = NULL;
	}
	if (iteration_count > 1) log_debug("IdentifyParentOfVoxelsWithRefPlane voxeltype %d  iteration_count=%d", voxeltype,  iteration_count);
	return true;
}

void PlaneSegmentation::IdentifyParentOfPseudoBadConnectWithGood()
{
	float min_normal_diff = static_cast<float>(std::cos((plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_NORMAL_ANGLE_OF_TWO_VOXEL) * M_PI / 180));
	float min_plane_dist = plane_seg_params.plane_seg_thresholds.THRESHOLD_MIN_PLANE_DIST_OF_TWO_VOXEL;

    //Identify extended plane connected with reference planes
//#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(plane_voxel_array.size); i++)
	{
		PlaneVoxelItem *plane_voxel = &plane_voxel_array.voxels[i];
		if ((plane_voxel->voxel_type != PSEUDO_BAD_VOXEL) || (!plane_voxel->is_overall_merged))  continue;
		plane_voxel->is_same_with_ref_plane = new bool[plane_merge_out.size];
		for (int j = 0; j < static_cast<int>(plane_merge_out.size); j++)
		{
			plane_voxel->is_same_with_ref_plane[j] = false;
			PlaneMergeOutputItem *plane_it =&plane_merge_out.planes[j];
			float normal_diff = std::fabs( ComputeVectorDotProduct<float>(plane_voxel->plane_normal, plane_it->plane_normal));
			if (normal_diff < min_normal_diff)  continue;
			float plane_dist =  ComputePointToPlaneDist<float>(plane_voxel->plane_center, plane_it->plane_normal, plane_it->plane_center);
			if (plane_dist < min_plane_dist)
			{
				plane_voxel->is_same_with_ref_plane[j] = true;
				plane_voxel->ref_plane_cnt++;
			}
		}
	}  

	IdentifyParentOfVoxelsWithRefPlane(PSEUDO_BAD_VOXEL);
}

void PlaneSegmentation::MergeExtendPartWithRef(PlaneMergeOutput *ref_planes)
{
	BadVoxelConnectedUpdate();
	PointsGroupPlane *points_group_plane = new PointsGroupPlane(this);
	PointsGroupPlaneArray extended_planes;
	extended_planes.size = 0;
	extended_planes.planes = NULL;
#pragma omp parallel for
	for (int i = 0; i < ref_planes->size; i++)
	{
		PlaneMergeOutputItem* ref_plane_it = &ref_planes->planes[i];
		GetPlaneDistMse(ref_plane_it, ref_plane_it->plane_mse);
	}

	points_group_plane->PointsGroupPlaneMergeFromRemainer(true, ref_planes, &extended_planes);
	log_info("extended_planes size= %d", extended_planes.size);

	std::vector<bool> is_to_be_reserved(extended_planes.size, false);
	//record flag if extendted resource is to be kept for reference plane , only true in case of zero copy 
	if (extended_planes.size > 0)
	{
		//merging extended plane points into reference planes
		for (unsigned int i = 0; i < extended_planes.size; i++)
		{
			PointsGroupParentPlaneItem *extend_plane_it = &extended_planes.planes[i];
			if ((extend_plane_it->ref_plane_idx == std::numeric_limits<unsigned int> ::max()) || (extend_plane_it->ref_plane_idx >= ref_planes->size))
			{
				//it is impossible ,must have a bug
				log_error("extended plane %d get reference plane error plane idx =%d", i, extend_plane_it->ref_plane_idx);
				continue;
			}
			PlaneMergeOutputItem *ref_plane_it = &ref_planes->planes[extend_plane_it->ref_plane_idx];
			if (ref_plane_it->extended_part_points.size == 0)
			{
				// if a reference plane only one extended part, here is zero copy
				is_to_be_reserved[i] = true;
				ref_plane_it->extended_part_points = extend_plane_it->points;
				MathOperation::PushSums(&ref_plane_it->sums, &extend_plane_it->sums);
				ref_plane_it->total_point_cnt += ref_plane_it->extended_part_points.size;
			}
			else
			{
				unsigned int old_point_cnt = ref_plane_it->extended_part_points.size;
				ref_plane_it->extended_part_points.size += extend_plane_it->points.size;
				unsigned int *tmp_point_idx = new unsigned int[ref_plane_it->extended_part_points.size];
				// first copy the old point idx
				memcpy(tmp_point_idx, ref_plane_it->extended_part_points.point_idx, sizeof(unsigned int)*old_point_cnt);

				memcpy(tmp_point_idx + old_point_cnt, extend_plane_it->points.point_idx, sizeof(unsigned int)*extend_plane_it->points.size);
				delete[] ref_plane_it->extended_part_points.point_idx;
				ref_plane_it->extended_part_points.point_idx = tmp_point_idx;
				MathOperation::PushSums(&ref_plane_it->sums, &extend_plane_it->sums);
				ref_plane_it->total_point_cnt +=  extend_plane_it->points.size;
			}
		}

		//recompute the normal and mse of reference planes if have extended parts;
		for (unsigned int i = 0; i < ref_planes->size; i++)
		{
			PlaneMergeOutputItem *ref_plane_it = &ref_planes->planes[i];
			if (ref_plane_it->extended_part_points.size != 0)
			{
				float eigen_mse;
				MathOperation::Compute(ref_plane_it->total_point_cnt, ref_plane_it->sums, ref_plane_it->plane_normal, ref_plane_it->plane_center, eigen_mse);
				GetPlaneDistMse(ref_plane_it, ref_plane_it->plane_mse);
			}
		}


	}
	//release sources
	if (extended_planes.planes != NULL)
	{
		for (unsigned int i = 0; i < extended_planes.size; i++)
		{
			PointsGroupParentPlaneItem *extend_plane_it = &extended_planes.planes[i];
			if (extend_plane_it->groups.points_group_id != NULL)
			{
				delete[] extend_plane_it->groups.points_group_id;
				extend_plane_it->groups.points_group_id = NULL;
			}

			if (is_to_be_reserved[i]) continue; // resources has been assigned to extended part of reference plane for zero copy

			if (extend_plane_it->points.point_idx != NULL)
			{
				delete[] extend_plane_it->points.point_idx;
				extend_plane_it->points.point_idx = NULL;
			}

			/*if (extend_plane_it->bad_group_points.point_idx != NULL)
			{
				delete[] extend_plane_it->bad_group_points.point_idx;
				extend_plane_it->bad_group_points.point_idx = NULL;
			}*/

		}
		delete[] extended_planes.planes;
		extended_planes.planes = NULL;

	}
	delete points_group_plane;
	points_group_plane = NULL;
}

void PlaneSegmentation::MergePseudoBadConnectWithGood()
{
	float min_normal_diff = static_cast<float > (std::cos((plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_NORMAL_ANGLE_OF_TWO_VOXEL) * M_PI / 180));
	float min_plane_dist = plane_seg_params.plane_seg_thresholds.THRESHOLD_MIN_PLANE_DIST_OF_TWO_VOXEL;

	/*to reduce the pseudobad voxels normal difference threshold with good plane because the plane normal now is accurate */
	min_normal_diff = min_normal_diff * 0.9f;
	//for UNRE must give less threshold , reason: because UNRE points cloud is too thick, 
	// so  pseduobad voxels's mse threshold must be  given higher,  pseudobad voxels  in edges have high probility of being bad voxels
	if (sys_control_para.scanner_type == UNRE_TYPE_0)
	{
		min_normal_diff = min_normal_diff * 0.8f;   
	}
	// step 1: find all the pseudobad voxels whose parent voxel  are parent voxel of reference plane and marked with merged flag, and change parent voxel 
	std::vector<std::vector<unsigned int> > pseduobad_idx_of_mergeout_plane;
	pseduobad_idx_of_mergeout_plane.resize(plane_merge_out.size);

	for (unsigned int i = 0; i < plane_voxel_array.size; i++) {
		PlaneVoxelItem *plane_voxel = &plane_voxel_array.voxels[i];
		PlaneMergeItem *plane_merge_it = &plane_merge_element[i];
		if (plane_voxel->voxel_type != PSEUDO_BAD_VOXEL) continue;
		if (plane_voxel->ref_plane_cnt == 0) continue;
		unsigned int ref_plane_idx = voxel_to_plane_idx[plane_merge_it->parent_voxel_idx[0]];

		//bool debug_con = (i == 25251) || (i == 25136) || (i == 25213) || (i == 25098) || (i == 24982);
		if (ref_plane_idx < plane_merge_out.size)
		{
			PlaneMergeOutputItem* plane_it = &plane_merge_out.planes[ref_plane_idx];
			//compare reference plane normal difference and plane distance, if less predefined threshold then add this voxels
			float normal_diff = std::fabs(ComputeVectorDotProduct<float>(plane_it->plane_normal, plane_voxel->plane_normal));
			float plane_dist = Util_Math::ComputePointToPlaneDist<float,Point3f>(plane_voxel->plane_center,plane_it->plane_normal, plane_it->plane_center);
			//if (debug_con) log_info("voxel %d plane%d normal_diff =%f plane_dist =%f ", i, ref_plane_idx,(std::acos(normal_diff)) * 180 / M_PI, plane_dist);
			if (normal_diff < min_normal_diff) continue;
			if (plane_dist > min_plane_dist) continue;
			pseduobad_idx_of_mergeout_plane[ref_plane_idx].push_back(i);
			plane_voxel->is_being_merged = true;
		}
	}

	// step2: add merged voxels to plane_merge_out
	for (unsigned int i = 0; i < plane_merge_out.size; i++)
	{
		if (pseduobad_idx_of_mergeout_plane[i].size() != 0)
		{
			PlaneMergeOutputItem *plane_it = &plane_merge_out.planes[i];
			unsigned int total_voxel_size = plane_it->voxels.size + static_cast<unsigned int>(pseduobad_idx_of_mergeout_plane[i].size());
			unsigned int *tmp_voxel_idx_ptr = new unsigned int[total_voxel_size];
			memcpy(tmp_voxel_idx_ptr, plane_it->voxels.voxel_idx, sizeof(unsigned int)*plane_it->voxels.size);
			for (int j = 0; j < pseduobad_idx_of_mergeout_plane[i].size(); j++)
			{
				unsigned int voxel_idx = pseduobad_idx_of_mergeout_plane[i][j];
				tmp_voxel_idx_ptr[plane_it->voxels.size + j] = voxel_idx;
				PlaneVoxelItem *plane_voxel = &plane_voxel_array.voxels[voxel_idx];
				MathOperation::PushSums(&plane_it->sums, &plane_voxel->sums);
				plane_it->total_point_cnt += plane_voxel->points.size;
			}
			delete[] plane_it->voxels.voxel_idx;
			plane_it->voxels.voxel_idx = tmp_voxel_idx_ptr;
			plane_it->voxels.size = total_voxel_size;
			float eigen_mse;
			MathOperation::Compute(plane_it->total_point_cnt, plane_it->sums,plane_it->plane_normal, plane_it->plane_center,eigen_mse);
			GetPlaneDistMse(plane_it, plane_it->plane_mse);
		}
	}	

	pseduobad_idx_of_mergeout_plane.clear();
	pseduobad_idx_of_mergeout_plane.shrink_to_fit();
}

void PlaneSegmentation::InvalidOverallForPBWithGoodPlane()
{
	// find all the pseudobad voxels connected with reference plane ,marked is_overall_merged is false, and restore the parent of psuedobad voxels who is not merged
	for (unsigned int i = 0; i < plane_voxel_array.size; i++)
	{
		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[i];
		PlaneMergeItem* plane_merge_it = &plane_merge_element[i];
		if (plane_voxel->voxel_type != PSEUDO_BAD_VOXEL) continue;

		if (!plane_voxel->is_being_merged)
		{
			//if not being merged , restore the other pseudobad voxels parent voxel 
			plane_merge_it->parent_voxel_idx[0] = plane_merge_it->parent_voxel_idx[1] = i;
			// if pseudobad voxel is not merged, and its is_overall_merged is true and connected with reference plane, mark it as bad voxels
			for (int j = 0; j < 26; j++) {

				NeighborItem* neighbor_it = &plane_voxel->neighbors[j];
				if (!neighbor_it->is_connected) continue;
				PlaneVoxelItem* neighbor_voxel = &plane_voxel_array.voxels[neighbor_it->voxel_idx];
				PlaneMergeItem* neighbor_merge_it = &plane_merge_element[neighbor_it->voxel_idx];
				if (!neighbor_voxel->is_overall_merged) continue;
				unsigned int plane_idx = voxel_to_plane_idx[neighbor_merge_it->parent_voxel_idx[0]];
				// if the voxel is connected with a reference plane , mark its is_overall_merged is false
				if (plane_idx < plane_merge_out.size)
				{
					//if a pesudo bad voxel is connected with good voxel's plane ,treat it as bad voxel.
					if (plane_voxel->is_overall_merged)
					{
						plane_voxel->is_overall_merged = false;
						bad_voxel_size++;
					}
				}
			}
		}
	}

}
//combine the planes together with plane_merge_out
void PlaneSegmentation::CombinePlanes(PlaneMergeOutput *planes)
{
	// combine new planes with plane_merge_out and use planes array replace old plane_merge_out
	if (planes->size != 0)
	{
		unsigned int combine_plane_cnt = plane_merge_out.size + planes->size;
		
		PlaneMergeOutputItem *new_planes = new PlaneMergeOutputItem[combine_plane_cnt];
		for (unsigned int i = 0; i < combine_plane_cnt; i++)
		{
			if (i < plane_merge_out.size)
			{
				new_planes[i] = plane_merge_out.planes[i];
			}
			else
			{
				int cur_plane_idx = i- plane_merge_out.size;
				new_planes[i] = planes->planes[cur_plane_idx];
			}
		}
		delete[] plane_merge_out.planes;
		plane_merge_out.planes = new_planes;
		plane_merge_out.size = combine_plane_cnt;
		delete[] planes->planes;
		planes->planes = NULL;
	}	 
}


bool PlaneSegmentation::ComputeAvgNormal(unsigned int voxel_idx, Point3f &avg_normal)
{
	if (voxel_idx >= plane_voxel_array.size)
	{
		log_error("voxel idx is exceed the size of voxel array =%d", voxel_idx, plane_voxel_array.size);
		return false;
	}
	PlaneMergeItem* plane_merge_it = &plane_merge_element[voxel_idx];
	if (plane_merge_it->good_neighbor_count == 0)
	{
		log_error("voxel idx %d have no good neighbor", voxel_idx);
	}
	PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[voxel_idx];
	SumforCovariance sums;
	MathOperation::AssignSums(&sums, &plane_voxel->sums);
	unsigned int point_cnt = plane_voxel->points.size;
	for (int k = 0; k < 26; k++)
	{
		NeighborItem* neighbour_item = &plane_voxel->neighbors[k];
		if (!neighbour_item->neighbor_flag) continue;
		PlaneVoxelItem* neighbor_voxel_it = &plane_voxel_array.voxels[neighbour_item->voxel_idx];
		MathOperation::PushSums(&sums, &neighbor_voxel_it->sums);
		point_cnt += neighbor_voxel_it->points.size;
	}
	Point3f voxel_center;
	float eigen_mse;
	MathOperation::Compute(point_cnt, sums, avg_normal, voxel_center, eigen_mse);	
	return true;
}

bool checkNeighboursVoxelPointsConnected(const PointArray pt_cloud_xyz, const std::vector<unsigned int> first_points, const std::vector<unsigned int> second_points, const float distance)
{
	if (second_points.size() == 0)
		return false;

	std::vector<float> dists_set(first_points.size() * second_points.size());
#pragma omp parallel for
	for (int i = 0; i < first_points.size(); i++)
	{
		for (int j = 0; j < second_points.size(); j++)
		{
			Point3f delta_point = pt_cloud_xyz.points[first_points[i]] - pt_cloud_xyz.points[second_points[j]];
			dists_set[i * second_points.size() + j] = std::fabs(delta_point.x) + std::fabs(delta_point.y) + std::fabs(delta_point.z);
		}
	}
	std::sort(dists_set.begin(), dists_set.end());

	if (dists_set.size() > 0 && dists_set[0] < distance)
		return true;
	return false;
}

typedef struct VoxelsClassifygroup
{
	std::vector<unsigned int> points_index;
	int group_id;
	std::vector<VoxelsClassifygroup*> neighbours;

	VoxelsClassifygroup()
	{
		points_index.clear();
		neighbours.clear();
		group_id = -1;
	}
};
#define SUB_VOXEL_GRID_RATIO 0.45
//#define VOXELS_CONNECT_CHECKED_POINTS
//limited by PROJECT->Properties->Configuration Properties->Linker->System->Stack Reserve Size
static inline void VoxelsGroupGrowing(VoxelsClassifygroup* voxel)
{
	for (auto& nbr : voxel->neighbours)
	{
		if (nbr->group_id < 0 && nbr->points_index.size() > 0)
		{
			nbr->group_id = voxel->group_id;
			VoxelsGroupGrowing(nbr);
		}
	}
}

void PlaneSegmentation::ClassifyVoxelPoints(const PointArray& pt_cloud_xyz, const std::vector<unsigned int>& points, std::vector<std::vector<unsigned int>>& out_points, float voxel_ratio)
{
	double min_x, min_y, min_z, max_x, max_y, max_z;
	min_x = min_y = min_z = std::numeric_limits<float>::infinity();
	max_x = max_y = max_z = -std::numeric_limits<float>::infinity();
	float voxel_size = length_x_of_voxel * voxel_ratio;
	int movement_x[26] = { -1, -1, -1,  1,  1,  1, 0, 0,  0,  0,  0,  0, 0,  0, -1, -1, -1, -1,  -1, -1, 1, 1,  1,  1,  1,  1 };
	int movement_y[26] = { 0,  0,  0,  0,  0,  0, 1, 1,  1, -1, -1, -1, 0,  0,  1,  1,  1, -1,  -1, -1, 1, 1,  1, -1, -1, -1 };
	int movement_z[26] = { 0,  1, -1,  0,  1, -1, 0, 1, -1,  0,  1, -1, 1, -1,  0,  1, -1,  0,   1, -1, 0, 1, -1,  0,  1, -1 };

	for (unsigned int i = 0; i < points.size(); i++) 
	{
		min_x = (min_x > pt_cloud_xyz.points[points[i]].x) ? pt_cloud_xyz.points[points[i]].x : min_x;
		max_x = (max_x < pt_cloud_xyz.points[points[i]].x) ? pt_cloud_xyz.points[points[i]].x : max_x;
		min_y = (min_y > pt_cloud_xyz.points[points[i]].y) ? pt_cloud_xyz.points[points[i]].y : min_y;
		max_y = (max_y < pt_cloud_xyz.points[points[i]].y) ? pt_cloud_xyz.points[points[i]].y : max_y;
		min_z = (min_z > pt_cloud_xyz.points[points[i]].z) ? pt_cloud_xyz.points[points[i]].z : min_z;
		max_z = (max_z < pt_cloud_xyz.points[points[i]].z) ? pt_cloud_xyz.points[points[i]].z : max_z;
	}

	unsigned int cols_of_voxel, rows_of_voxel, depths_of_voxel, numbers_voxel_grid;
	cols_of_voxel = (unsigned int)(std::floor((max_x - min_x) / voxel_size) + 1);
	rows_of_voxel = (unsigned int)(std::floor((max_y - min_y) / voxel_size) + 1);
	depths_of_voxel = (unsigned int)(std::floor((max_z - min_z) / voxel_size) + 1);
	numbers_voxel_grid = cols_of_voxel * rows_of_voxel * depths_of_voxel;

	std::vector<VoxelsClassifygroup> voxels(numbers_voxel_grid);
	unsigned int col_idx, row_idx, height_idx, voxel_id;
	for (unsigned int i = 0; i < points.size(); i++)
	{
		col_idx = (unsigned int)(std::floor((pt_cloud_xyz.points[points[i]].x - min_x) / voxel_size));
		row_idx = (unsigned int)(std::floor((pt_cloud_xyz.points[points[i]].y - min_y) / voxel_size));
		height_idx = (unsigned int)(std::floor((pt_cloud_xyz.points[points[i]].z - min_z) / voxel_size));

		voxel_id = height_idx * (cols_of_voxel * rows_of_voxel) + row_idx * cols_of_voxel + col_idx;
		voxels[voxel_id].points_index.push_back(points[i]);
		if (voxels[voxel_id].neighbours.size() == 0)
		{
			for (int i = 0; i < 26; i++)
			{
				if (height_idx + movement_z[i] >= (int)depths_of_voxel || height_idx + movement_z[i] < 0)
					continue;
				if (row_idx + movement_y[i] >= (int)rows_of_voxel || row_idx + movement_y[i] < 0)
					continue;
				if (col_idx + movement_x[i] >= (int)cols_of_voxel || col_idx + movement_x[i] < 0)
					continue;

				int neighbor_voxel_rid = (int)((height_idx + movement_z[i]) * (cols_of_voxel * rows_of_voxel) + (row_idx + movement_y[i]) * cols_of_voxel + (col_idx + movement_x[i]));

				if (neighbor_voxel_rid < 0 || neighbor_voxel_rid >= (int)numbers_voxel_grid) continue;

				voxels[voxel_id].neighbours.push_back(&voxels[neighbor_voxel_rid]);
			}
		}
	}

	unsigned int group_index = 0;
	//neighbours growing group
	for (unsigned int i = 0; i < voxels.size();i++)
	{
		if (voxels[i].group_id < 0 && voxels[i].points_index.size() > 0)
		{
			voxels[i].group_id = ++group_index;
			VoxelsGroupGrowing(&voxels[i]);
		}
	}

	out_points.resize(group_index);
	for (unsigned int i = 0; i < voxels.size(); i++)
	{
		if (voxels[i].group_id > 0)
		{
			unsigned int gid = voxels[i].group_id - 1;
			out_points[gid].insert(out_points[gid].end(), voxels[i].points_index.begin(), voxels[i].points_index.end());
		}
	}
}

bool PlaneSegmentation::BadVoxelConnectedUpdate()
{
	float close_dist_2points = plane_seg_params.plane_seg_thresholds.THRESHOLD_DIST_OF_2VOXEL_CONNECTED;

	//check  if the remainer voxel is occupied before init points group
	int point_cnt = 0;	
#pragma omp parallel for reduction(+:point_cnt)
	for (int i = 0; i < static_cast<int>(point_merged_voxel_size); i++)
	{
		BadVoxelMergeOutpuItem *bad_voxel_merge_it = &bad_voxel_merge_item[i];
		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[bad_voxel_merge_it->bad_voxel_idx];
		if (!bad_voxel_merge_it->is_remainer_occupied) continue;
		if (plane_voxel->points.size <= bad_voxel_merge_it->num_of_points_merged) bad_voxel_merge_it->is_remainer_occupied = false;

		point_cnt += (plane_voxel->points.size - bad_voxel_merge_it->num_of_points_merged);
	}

	std::vector<std::vector<unsigned int>> remain_points(point_merged_voxel_size);
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(point_merged_voxel_size); i++)
	{
		BadVoxelMergeOutpuItem* bad_voxel_it = &bad_voxel_merge_item[i];
		PlaneVoxelItem * plane_voxel = &plane_voxel_array.voxels[bad_voxel_it->bad_voxel_idx];
		if (plane_voxel->is_being_merged&&plane_voxel->is_overall_merged) continue;
		if (!bad_voxel_it->is_remainer_occupied) continue;
		
		for (int j = 0; j < static_cast<int>(bad_voxel_it->voxel_point_size); j++) {

			if (bad_voxel_it->point_merged_flag[j]) continue;
			remain_points[i].push_back(plane_voxel->points.point_idx[j]);
		}
	}

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(point_merged_voxel_size); i++)
	{
		BadVoxelMergeOutpuItem* bad_voxel_it = &bad_voxel_merge_item[i];
		PlaneVoxelItem * plane_voxel = &plane_voxel_array.voxels[bad_voxel_it->bad_voxel_idx];
		if (!bad_voxel_it->is_remainer_occupied) continue;
		if (remain_points[i].empty()) continue;

		for (int j = 0; j < 26; j++)
		{

			bad_voxel_it->neighbor_connected[j] = false;
			NeighborItem* neighbor_it = &plane_voxel->neighbors[j];
			if (!neighbor_it->is_connected) continue;
			unsigned int neighbor_bad_voxel_idx = voxel_idx_to_bad_voxel[neighbor_it->voxel_idx];
			if (neighbor_bad_voxel_idx >= point_merged_voxel_size) continue;
			BadVoxelMergeOutpuItem* neighbor_bad_voxel_it = &bad_voxel_merge_item[neighbor_bad_voxel_idx];
			PlaneVoxelItem * neighbor_voxel = &plane_voxel_array.voxels[neighbor_it->voxel_idx];
			if (!neighbor_bad_voxel_it->is_remainer_occupied) continue;
			if (remain_points[neighbor_bad_voxel_idx].empty()) continue;
#ifdef VOXELS_CONNECT_CHECKED_POINTS
			std::vector<std::vector<unsigned int>> remain_points_group, remain_neighbor_points_group;
			ClassifyVoxelPoints(pt_cloud_xyz, remain_points[i], remain_points_group, SUB_VOXEL_GRID_RATIO);
			ClassifyVoxelPoints(pt_cloud_xyz, remain_points[neighbor_bad_voxel_idx], remain_neighbor_points_group, SUB_VOXEL_GRID_RATIO);
			//int rg_count = 0;
			//for (auto& rg : remain_points_group)
			//	rg_count += rg.size();
			//if (rg_count != remain_points[i].size())
			//	std::cout << "111-miss " << rg_count << "\t" << remain_points[i].size() << std::endl;

			//int rng_count = 0;
			//for (auto& rng : remain_neighbor_points_group)
			//	rng_count += rng.size();
			//if (rng_count != remain_points[neighbor_bad_voxel_idx].size())
			//	std::cout << "222-miss " << rng_count << "\t" << remain_points[neighbor_bad_voxel_idx].size() << std::endl;

			bad_voxel_it->neighbor_connected[j] = true;
			for (int m = 0; m < remain_points_group.size(); m++)
			{
				if (!bad_voxel_it->neighbor_connected[j]) break;
				for (int n = 0; n < remain_neighbor_points_group.size(); n++)
				{
					if (!checkNeighboursVoxelPointsConnected(pt_cloud_xyz, remain_points_group[m], remain_neighbor_points_group[n], close_dist_2points))
					{
						bad_voxel_it->neighbor_connected[j] = false;
						break;
					}
				}
			}
#else
			bad_voxel_it->neighbor_connected[j] = MathOperation::Identify2PointsGroupCLoseByDistance(pt_cloud_xyz, remain_points[i], remain_points[neighbor_bad_voxel_idx], close_dist_2points);
#endif
		}		
	}

	for (unsigned int i = 0; i < point_merged_voxel_size; i++)
	{
		remain_points[i].clear();
		remain_points[i].shrink_to_fit();
	}
	remain_points.clear();
	remain_points.shrink_to_fit();
	return true;
}


bool PlaneSegmentation::IdentifyBadVoxelConnectedWithPlanePoints(const unsigned int bad_voxel_idx, const unsigned int plane_idx, const std::vector<unsigned int> plane_points, bool &is_connected)
{
	float min_point_plane_dist = plane_seg_params.plane_seg_thresholds.THRESHOLD_MIN_DIST_OF_POINT2VOXEL;

	//float second_min_point_plane_dist = min_point_plane_dist = plane_seg_params.plane_seg_thresholds.THRESHOLD_MIN_DIST_OF_POINT2VOXEL / 2;
	//second_min_point_plane_dist = plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_VOXEL < second_min_point_plane_dist ? \
	//	plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_VOXEL : second_min_point_plane_dist;
	float close_dist_2points = plane_seg_params.plane_seg_thresholds.THRESHOLD_DIST_OF_2VOXEL_CONNECTED;

	//float min_normal_diff = static_cast<float>(std::cos(plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_NORMAL_ANGEL_OF_EDGE_POINT * M_PI / 180));
	BadVoxelMergeOutpuItem *bad_voxel_it = &bad_voxel_merge_item[bad_voxel_idx];
	PlaneMergeOutputItem *plane_it = &plane_merge_out.planes[plane_idx];
	PlaneVoxelItem *plane_voxel = &plane_voxel_array.voxels[bad_voxel_it->bad_voxel_idx];
	std::vector<unsigned int> points_in_plane(0);


	// bool debug = false;((bad_voxel_it->bad_voxel_idx == 1003) && (plane_idx == 22)) || ((bad_voxel_it->bad_voxel_idx == 9998) && (plane_idx == 22))\
		//|| ((bad_voxel_it->bad_voxel_idx == 10002) && (plane_idx == 22)) || ((bad_voxel_it->bad_voxel_idx == 10040) && (plane_idx == 22));

	for (unsigned int i = 0; i < bad_voxel_it->voxel_point_size; i++)
	{
		if (bad_voxel_it->point_merged_flag[i]) continue;
		unsigned int point_idx = plane_voxel->points.point_idx[i];
		Point3f Point = pt_cloud_xyz.points[point_idx];
		//Point3f Point_normal = pt_cloud_normal.points[point_idx];
		//float normal_diff = std::fabs( ComputeVectorDotProduct<float>(Point_normal, plane_it->plane_normal));
		float plane_dist =  ComputePointToPlaneDist<float>(Point, plane_it->plane_normal, plane_it->plane_center);
		//if (debug) log_info("voxel %d have point idx=%d normal diff =%f plane_dist =%f  with plane %d", bad_voxel_it->bad_voxel_idx, point_idx, std::acos(normal_diff) * 180 / M_PI, plane_dist, plane_idx);
		//if ((plane_dist < second_min_point_plane_dist) || (normal_diff > min_normal_diff) && (plane_dist < min_point_plane_dist))
		if (plane_dist < min_point_plane_dist)
		{
			//if (debug) log_info("voxel %d have point idx=%d normal diff =%f plane_dist =%f  with plane %d added", bad_voxel_it->bad_voxel_idx, point_idx, std::acos(normal_diff) * 180 / M_PI, plane_dist, plane_idx);
			points_in_plane.push_back(point_idx);
		}
	}

	if (points_in_plane.size() >= 1)
	{
		is_connected = MathOperation::Identify2PointsGroupCLoseByDistance(pt_cloud_xyz, points_in_plane, plane_points, close_dist_2points);
	}
	else
	{
		is_connected = false;
	}
	return true;
}

bool PlaneSegmentation::IdentifyBadVoxelConnectedWithPlanePoints(const unsigned int bad_voxel_idx, const unsigned int plane_idx, const unsigned int voxel_idx, bool &is_connected)
{
	PlaneVoxelItem *plane_voxel = &plane_voxel_array.voxels[voxel_idx];

	std::vector<unsigned int> plane_points(plane_voxel->points.size);

	for (unsigned int i = 0; i < plane_voxel->points.size; i++)
	{
		plane_points[i] = plane_voxel->points.point_idx[i];
	}

	IdentifyBadVoxelConnectedWithPlanePoints(bad_voxel_idx, plane_idx, plane_points, is_connected);
	plane_points.clear();
	plane_points.shrink_to_fit();
	return true;
}


bool PlaneSegmentation::BadVoxelConnectedWithRef(const unsigned int bad_voxel_idx, const unsigned int plane_idx, bool &is_connected)
{
	if ((bad_voxel_idx >= point_merged_voxel_size) || plane_idx >= plane_merge_out.size)
	{
		log_error("input error bad_voxel_idx=%d plane_idx =%d", bad_voxel_idx, plane_idx);
	}

	unsigned int flat_voxel_plane_size = good_voxel_plane_size + pseudobad_voxel_plane_size;

	is_connected = false;
	BadVoxelMergeOutpuItem *bad_voxel_it = &bad_voxel_merge_item[bad_voxel_idx];
	PlaneMergeOutputItem *plane_it = &plane_merge_out.planes[plane_idx];
	PlaneVoxelItem *plane_voxel = &plane_voxel_array.voxels[bad_voxel_it->bad_voxel_idx];

	// first find plane in neighbors
	for (unsigned int i = 0; i < 26; i++)
	{
		NeighborItem *neighbor_it = &plane_voxel->neighbors[i];
		if (!neighbor_it->is_connected) continue;
		PlaneVoxelItem *neighbor_voxel = &plane_voxel_array.voxels[neighbor_it->voxel_idx];
		PlaneMergeItem *neighbor_merge_it = &plane_merge_element[neighbor_it->voxel_idx];
		if ((neighbor_voxel->is_overall_merged)&& neighbor_voxel->is_being_merged)
		{
			//if (!neighbor_voxel->is_being_merged) continue;
			unsigned int neighbor_plane_idx = voxel_to_plane_idx[neighbor_merge_it->parent_voxel_idx[0]];
			if (neighbor_plane_idx != plane_idx) continue;
			//find all the points who is same in the plane and check if close to the plane
			IdentifyBadVoxelConnectedWithPlanePoints(bad_voxel_idx, plane_idx, neighbor_it->voxel_idx, is_connected);	
			if (is_connected) break;
		}
		else
		{
			//if (!bad_voxel_it->neighbor_connected[i]) continue;
			unsigned int neighbor_bad_voxel_idx = voxel_idx_to_bad_voxel[neighbor_it->voxel_idx];
			if (neighbor_bad_voxel_idx >= point_merged_voxel_size) continue;
			BadVoxelMergeOutpuItem *neighbor_bad_voxel_it = &bad_voxel_merge_item[neighbor_bad_voxel_idx];

			if (plane_idx < flat_voxel_plane_size)
			{
				if (!neighbor_bad_voxel_it->remaining_points_merged_plane[plane_idx]) continue;
			}
			else
			{
				unsigned int group_plane_idx = plane_idx - flat_voxel_plane_size;
				if (!neighbor_bad_voxel_it->being_in_group_plane[group_plane_idx]) continue;
			}

			//find all the points who is same in the plane and check if close to the plane
			std::vector<unsigned int> points_in_plane(0);			
			for (unsigned int j = 0; j < neighbor_voxel->points.size; j++)
			{
				if ((neighbor_bad_voxel_it->closest_plane_idx[j] == plane_idx) /*&& (neighbor_bad_voxel_it->point_merged_flag[j])*/)
				{
					points_in_plane.push_back(neighbor_voxel->points.point_idx[j]);
				}
			}
			if (points_in_plane.size() > 0)
			{
				IdentifyBadVoxelConnectedWithPlanePoints(bad_voxel_idx, plane_idx, points_in_plane, is_connected);
			}
			if (is_connected) break;
		}

	}

	if (is_connected) return true;
	
	// second find plane in self voxel
	if (plane_idx < flat_voxel_plane_size)	{

		if (!bad_voxel_it->remaining_points_merged_plane[plane_idx])
		{
			return true;
		}
	}
	else
	{
		unsigned int group_plane_idx = plane_idx - flat_voxel_plane_size;
		if (!bad_voxel_it->being_in_group_plane[group_plane_idx])
		{
			return true;
		}
	}

	//if (debug) log_info("03 bad_voxel_idx =%d plane_idx=%d ", bad_voxel_idx, plane_idx);

	std::vector<unsigned int> points_in_plane(0);
	for (unsigned int i = 0; i < bad_voxel_it->voxel_point_size; i++)
	{
		if ((bad_voxel_it->closest_plane_idx[i] == plane_idx) && (bad_voxel_it->point_merged_flag[i]))
		{
			points_in_plane.push_back(plane_voxel->points.point_idx[i]);
		}
	}

	//if (debug) log_info("04 bad_voxel_idx =%d plane_idx=%d", bad_voxel_idx, plane_idx);
	if (points_in_plane.size() > 0)
	{
		IdentifyBadVoxelConnectedWithPlanePoints(bad_voxel_idx, plane_idx, points_in_plane, is_connected);
	}
	return true;
}

bool PlaneSegmentation::BadVoxelConnectedWithRef(bool is_first)
{
//#pragma omp parallel for // reduction( +:tmp_plane_cnt)
	for (int i = 0; i < static_cast<int>(point_merged_voxel_size); i++)
	{
		BadVoxelMergeOutpuItem *bad_voxel_merge_it = &bad_voxel_merge_item[i];
		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[bad_voxel_merge_it->bad_voxel_idx];
		if (!bad_voxel_merge_it->is_remainer_occupied) continue;
		if (plane_voxel->points.size <= bad_voxel_merge_it->num_of_points_merged) bad_voxel_merge_it->is_remainer_occupied = false;
		if (!bad_voxel_merge_it->is_remainer_occupied) continue;
		/*bool debug_con = bad_voxel_merge_it->bad_voxel_idx == 10035 || bad_voxel_merge_it->bad_voxel_idx == 10034;
		if (debug_con)
		{
			for (unsigned int j = 0; j < plane_voxel->points.size; j++)
			{
				unsigned int tmp_plane_idx = bad_voxel_merge_it->closest_plane_idx[j];
				if (tmp_plane_idx <= plane_merge_out.size)
				{
					log_info("voxel%d point %d plane index =%d", bad_voxel_merge_it->bad_voxel_idx, plane_voxel->points.point_idx[j], tmp_plane_idx);
				}
				
			}
		}*/

		if(is_first) bad_voxel_merge_it->is_plane_connected = new bool[plane_merge_out.size];
		for (unsigned int j = 0; j < plane_merge_out.size; j++)
		{
			bad_voxel_merge_it->is_plane_connected[j] = false;
			BadVoxelConnectedWithRef(i, j, bad_voxel_merge_it->is_plane_connected[j]);
		}
	}
	return true;
}



bool PlaneSegmentation::IdentifyBadVoxelConnectedWithMultiPlanePoints(const unsigned int bad_voxel_idx, const unsigned int plane_idx, const std::vector<unsigned int> plane_points, bool& is_connected)
{
	float min_point_plane_dist = plane_seg_params.plane_seg_thresholds.THRESHOLD_MIN_DIST_OF_POINT2VOXEL;
	float close_dist_2points = plane_seg_params.plane_seg_thresholds.THRESHOLD_DIST_OF_2VOXEL_CONNECTED;

	BadVoxelMergeOutpuItem* bad_voxel_it = &bad_voxel_merge_item[bad_voxel_idx];
	PlaneMergeOutputItem* plane_it = &plane_merge_out.planes[plane_idx];
	PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[bad_voxel_it->bad_voxel_idx];
	std::vector<unsigned int> points_in_plane(0);

	for (unsigned int i = 0; i < bad_voxel_it->voxel_point_size; i++)
	{
		if (!bad_voxel_it->point_merged_flag[i]) continue;
		//if (bad_voxel_it->closest_plane_idx[i] == plane_idx) continue;
		unsigned int point_idx = plane_voxel->points.point_idx[i];
		Point3f Point = pt_cloud_xyz.points[point_idx];
		float plane_dist = ComputePointToPlaneDist<float>(Point, plane_it->plane_normal, plane_it->plane_center);
		if (plane_dist < min_point_plane_dist)
		{
			points_in_plane.push_back(point_idx);
		}
	}

	if (points_in_plane.size() >= 2)
	{
#ifdef VOXELS_CONNECT_CHECKED_POINTS
		std::vector<std::vector<unsigned int>> remain_points_group, remain_neighbor_points_group;
		ClassifyVoxelPoints(pt_cloud_xyz, points_in_plane, remain_points_group, SUB_VOXEL_GRID_RATIO);
		ClassifyVoxelPoints(pt_cloud_xyz, plane_points, remain_neighbor_points_group, SUB_VOXEL_GRID_RATIO);

		is_connected = true;
		for (int m = 0; m < remain_points_group.size(); m++)
		{
			if (!is_connected) break;
			for (int n = 0; n < remain_neighbor_points_group.size(); n++)
			{
				if (!checkNeighboursVoxelPointsConnected(pt_cloud_xyz, remain_points_group[m], remain_neighbor_points_group[n], close_dist_2points))
				{
					is_connected = false;
					break;
				}
			}
		}
#else
		is_connected = MathOperation::Identify2PointsGroupCLoseByDistance(pt_cloud_xyz, points_in_plane, plane_points, close_dist_2points);
#endif
	}
	else
	{
		is_connected = false;
	}
	return true;
}

bool PlaneSegmentation::IdentifyPseudoBadVoxelConnectedWithMultiPlanePoints(const unsigned int voxel_idx, const unsigned int plane_idx, const std::vector<unsigned int> plane_points, bool& is_connected)
{
	//float min_point_plane_dist = plane_seg_params.plane_seg_thresholds.THRESHOLD_MIN_DIST_OF_POINT2VOXEL;
	float close_dist_2points = plane_seg_params.plane_seg_thresholds.THRESHOLD_DIST_OF_2VOXEL_CONNECTED;

	//PlaneMergeOutputItem* plane_it = &plane_merge_out.planes[plane_idx];
	PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[voxel_idx];
	std::vector<unsigned int> points_in_plane(plane_voxel->points.size);

	for (unsigned int i = 0; i < plane_voxel->points.size; i++)
	{
		points_in_plane[i] = plane_voxel->points.point_idx[i];
	}

	if (points_in_plane.size() >= 2)
	{
#ifdef VOXELS_CONNECT_CHECKED_POINTS
		std::vector<std::vector<unsigned int>> remain_points_group, remain_neighbor_points_group;
		ClassifyVoxelPoints(pt_cloud_xyz, points_in_plane, remain_points_group, SUB_VOXEL_GRID_RATIO);
		ClassifyVoxelPoints(pt_cloud_xyz, plane_points, remain_neighbor_points_group, SUB_VOXEL_GRID_RATIO);

		is_connected = true;
		for (int m = 0; m < remain_points_group.size(); m++)
		{
			if (!is_connected) break;
			for (int n = 0; n < remain_neighbor_points_group.size(); n++)
			{
				if (!checkNeighboursVoxelPointsConnected(pt_cloud_xyz, remain_points_group[m], remain_neighbor_points_group[n], close_dist_2points))
				{
					is_connected = false;
					break;
				}
			}
		}
#else
		is_connected = MathOperation::Identify2PointsGroupCLoseByDistance(pt_cloud_xyz, points_in_plane, plane_points, close_dist_2points);
#endif
	}
	else
	{
		is_connected = false;
	}
	return true;
}

bool PlaneSegmentation::BadVoxelConnectedWithMultiRef(const unsigned int bad_voxel_idx, const unsigned int plane_idx, bool& is_connected)
{
	if ((bad_voxel_idx >= point_merged_voxel_size) || plane_idx >= plane_merge_out.size)
	{
		log_error("input error bad_voxel_idx=%d plane_idx =%d", bad_voxel_idx, plane_idx);
	}

	unsigned int flat_voxel_plane_size = good_voxel_plane_size + pseudobad_voxel_plane_size;

	is_connected = false;
	BadVoxelMergeOutpuItem* bad_voxel_it = &bad_voxel_merge_item[bad_voxel_idx];
	PlaneMergeOutputItem* plane_it = &plane_merge_out.planes[plane_idx];
	PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[bad_voxel_it->bad_voxel_idx];
	// first find plane in neighbors
	for (unsigned int i = 0; i < 26; i++)
	{
		NeighborItem* neighbor_it = &plane_voxel->neighbors[i];
		if (!neighbor_it->is_connected) continue;
		PlaneVoxelItem* neighbor_voxel = &plane_voxel_array.voxels[neighbor_it->voxel_idx];
		PlaneMergeItem* neighbor_merge_it = &plane_merge_element[neighbor_it->voxel_idx];
		if (neighbor_voxel->is_overall_merged)
		{

			if (!neighbor_voxel->is_being_merged) continue;
			unsigned int neighbor_plane_idx = voxel_to_plane_idx[neighbor_merge_it->parent_voxel_idx[0]];
			if (neighbor_plane_idx != plane_idx) continue;
			std::vector<unsigned int> plane_points(neighbor_voxel->points.size);

			for (unsigned int i = 0; i < neighbor_voxel->points.size; i++)
			{
				plane_points[i] = neighbor_voxel->points.point_idx[i];
			}

			if ((plane_voxel->is_overall_merged) && (plane_voxel->is_being_merged))
			{
				IdentifyPseudoBadVoxelConnectedWithMultiPlanePoints(bad_voxel_it->bad_voxel_idx, plane_idx, plane_points, is_connected);
			}
			else
			{
				IdentifyBadVoxelConnectedWithMultiPlanePoints(bad_voxel_idx, plane_idx, plane_points, is_connected);
			}
			if (is_connected) break;
		}
		else
		{
			unsigned int neighbor_bad_voxel_idx = voxel_idx_to_bad_voxel[neighbor_it->voxel_idx];
			if (neighbor_bad_voxel_idx >= point_merged_voxel_size) continue;
			BadVoxelMergeOutpuItem* neighbor_bad_voxel_it = &bad_voxel_merge_item[neighbor_bad_voxel_idx];
			if (plane_idx < flat_voxel_plane_size)
			{
				if (!neighbor_bad_voxel_it->remaining_points_merged_plane[plane_idx]) continue;
			}
			else
			{
				unsigned int group_plane_idx = plane_idx - flat_voxel_plane_size;
				if (!neighbor_bad_voxel_it->being_in_group_plane[group_plane_idx]) continue;
			}
			//find all the points who is same in the plane and check if close to the plane
			std::vector<unsigned int> points_in_plane(0);
			for (unsigned int j = 0; j < neighbor_voxel->points.size; j++)
			{
				if ((neighbor_bad_voxel_it->closest_plane_idx[j] == plane_idx) /*&& (neighbor_bad_voxel_it->point_merged_flag[j])*/)
				{
					points_in_plane.push_back(neighbor_voxel->points.point_idx[j]);
				}
			}
			if (points_in_plane.size() > 0)
			{
				if ((plane_voxel->is_overall_merged) && (plane_voxel->is_being_merged))
				{
					IdentifyPseudoBadVoxelConnectedWithMultiPlanePoints(bad_voxel_it->bad_voxel_idx, plane_idx, points_in_plane, is_connected);
				}
				else
				{
					IdentifyBadVoxelConnectedWithMultiPlanePoints(bad_voxel_idx, plane_idx, points_in_plane, is_connected);
				}
			}
			if (is_connected) break;
		}
	}
	return true;
}

// Perform plane segmentation process from point array and voxel array
bool PlaneSegmentation::FitPlaneMethod(const unsigned int method_type, const std::string output_path, NormalEstimationInterface &inputStruct, PlaneSegmentationOutput *plane_segmentation_out) {

	unsigned int filtering_strtgy_type = method_type & 7;
	bool with_multiplane_points = method_type & 8; // if true show the seg out planes wtih multiplane points, else with no multiplane points

	bool is_valid_inputs = false;
	TIMING_DECLARE(TP1)
	current_out_path.assign(output_path.c_str());

	if (filtering_strtgy_type != NO_NORMALEST)
	{
		TIMING_BEGIN(TP1)
		log_info("current_out_path = %s", current_out_path.c_str());
		InitializeParams(inputStruct);
		TIMING_END_ms("InitializeParams", TP1)

		TIMING_BEGIN(TP1)
		// Check the input data structure size and whether the pointer is NULL or not
		is_valid_inputs = CheckInputParameters(inputStruct.points, &inputStruct);
		if (is_valid_inputs == false) {
			log_error("Invalid input parameters");
			return false;
		}
		TIMING_END_ms("CheckInputParameters", TP1)
	}
	else
	{
		TIMING_BEGIN(TP1)
		InitializeParamsNoNm(inputStruct);
		TIMING_END_ms("InitializeParamsNoNm", TP1)
	}
	
	TIMING_BEGIN(TP1)
	// Calculate min and max x, y and z of point cloud
	GetMinMaxXYZ(pt_cloud_xyz);
	TIMING_END_ms("GetMinMaxXYZ", TP1)

	TIMING_BEGIN(TP1)
	// Calculate total number of voxel grid
	GetTotalNumOfVoxelGrid();
	TIMING_END_ms("GetTotalNumOfVoxelGrid", TP1)

	VoxelItem* voxel_array = NULL;
	if (filtering_strtgy_type != NO_NORMALEST)
	{
		TIMING_BEGIN(TP1)
		// Create voxel and assign its data for all occupied voxels
		CreateVoxels(inputStruct.voxel_array);
		TIMING_END_ms("CreateVoxels", TP1)
	}
	else
	{
		TIMING_BEGIN(TP1)
		OccupiedVoxelInit(inputStruct.points, voxel_array, num_of_occupied_voxel);
		TIMING_END_ms("OccupiedVoxelInit", TP1)

		TIMING_BEGIN(TP1)
		for (int i = 0; i < num_of_occupied_voxel; i++)
		{
			unsigned int point_size = voxel_array[i].points.size;
			//log_info("i =%d point_size =%d", i, point_size);
			for (unsigned int j = 0; j < point_size; j++)
			{
				unsigned int point_idx = voxel_array[i].points.point_idx[j];
				if (point_idx >= pt_cloud_xyz.size) log_info("i=%d j=%d point idx =%d", i, j, point_idx);
			}
		}
		CreateVoxels(voxel_array);
		if (voxel_array != NULL)
		{
			delete[]voxel_array;
		}
		TIMING_END_ms("CreateVoxels", TP1)
	}

#ifdef SAVE_OUTPUT_FILE_DEBUG
	if (debug_config_params.neighbour_info_debug)
	{
		TIMING_BEGIN(TP1)
		GetVoxelNeigbourDebugInfo(output_path, ALL_VOXEL_DEBUG);
		TIMING_END_ms("GetVoxelNeigbourDebugInfo", TP1)
	}
#endif // SAVE_OUTPUT_FILE_DEBUG

/*---------------generating planes by good voxels begin---------------------*/

	TIMING_BEGIN(TP1)
	// Find out the parent of voxels whose voxel type are good
	IdentifyParentOfGoodVoxels();
	TIMING_END_ms("IdentifyParentOfGoodVoxels", TP1)

	TIMING_BEGIN(TP1)
	// Merge all good voxels into its similar plane
	MergeGoodVoxels();
	TIMING_END_ms("MergeGoodVoxels", TP1)

#ifdef INCLUDE_MERGING_PB_CONNECTED_WITH_GOOD_PLANE
	TIMING_BEGIN(TP1)
	// Find out the parent of voxels whose voxel type is pseudo bad but parent is good (connected with good)
	IdentifyParentOfPseudoBadConnectWithGood();
	TIMING_END_ms("IdentifyParentOfPseudoBadConnectWithGood", TP1)

	TIMING_BEGIN(TP1)
	// Merge all pseudo voxels  whose parent is good into its similar plane
	MergePseudoBadConnectWithGood();
	TIMING_END_ms("MergePseudoBadConnectWithGood", TP1)
#endif

	InvalidOverallForPBWithGoodPlane();
	TIMING_BEGIN(TP1)
	// create point-to-plane item to prepare for extended part of plane generated by good voxels
	CreateBadVoxelItems(true);
	TIMING_END_ms("CreateBadVoxelItems good", TP1)


	
	TIMING_BEGIN(TP1)
	// find extended parts of good planes
	MergeExtendPartWithRef(&plane_merge_out);
	TIMING_END_ms("MergeExtendPartWithRef good", TP1)


#ifdef SAVE_OUTPUT_FILE_DEBUG
	if (debug_config_params.voxel_info_debug)
	{
		TIMING_BEGIN(TP1)
		for (unsigned int i = 0; i < good_voxel_plane_size; i++)
		{
			SavePlanePointsByVoxelType(output_path, i, GOOD_VOXEL);
		}
		TIMING_END_ms("save good plane voxels", TP1)
	}
#endif


/*---------------generating planes by good voxels end---------------------*/


/*---------------generating planes by pseudo bad voxels begin---------------------*/

	TIMING_BEGIN(TP1)
	// Find out the parent of all pseudo bad voxels
	IdentifyParentOfPseudoBadVoxels();
	TIMING_END_ms("IdentifyParentOfPseudoBadVoxels", TP1)

	TIMING_BEGIN(TP1)
	// Merge all pseudo bad voxels into its similar plane
	PlaneMergeOutput pseudobad_plane_merge_out;
	pseudobad_plane_merge_out.planes = NULL;
	pseudobad_plane_merge_out.size = 0;
	MergePseudoBadVoxels(&pseudobad_plane_merge_out);
	TIMING_END_ms("MergePseudoBadVoxels", TP1)

	TIMING_BEGIN(TP1)
	// create point-to-plane item to prepare for extended part of plane generated by pseudobad voxels
	CreateBadVoxelItems(false);
	TIMING_END_ms("CreateBadVoxelItems pseudobad", TP1)

	TIMING_BEGIN(TP1)
	// find extended parts of pseudobad planes
	MergeExtendPartWithRef(&pseudobad_plane_merge_out);
	TIMING_END_ms("MergeExtendPartWithRef pseduobad", TP1)

	TIMING_BEGIN(TP1)
	//Combine pseudobad planes and good planes together 
	CombinePlanes(&pseudobad_plane_merge_out);
	TIMING_END_ms("CombinePlanes psedobad", TP1)	

#ifdef SAVE_OUTPUT_FILE_DEBUG
	if (debug_config_params.voxel_info_debug)
	{
		TIMING_BEGIN(TP1)
		for (unsigned int i = 0; i < pseudobad_plane_merge_out.size; i++)
		{
			int pseudobad_plane_idx = i + good_voxel_plane_size;			
			SavePlanePointsByVoxelType(output_path, pseudobad_plane_idx, PSEUDO_BAD_VOXEL);
		}
		TIMING_END_ms("save pseudobad plane voxels", TP1)
	}
#endif

/*---------------generating planes by pseudo bad voxels end---------------------*/


/*---------------generating discrete planes by all remaining voxels begin---------------------*/

	if (filtering_strtgy_type == NO_FILTER)
	{
#ifdef	INCLUDE_POINTS_GROUP_MERGING
	TIMING_BEGIN(TP1)
	PointsGroupPlaneMergeFromRemainer();
	TIMING_END_ms("PointsGroupPlaneMergeFromRemainer", TP1)

	TIMING_BEGIN(TP1)
	MergeExtendPartWithRef(&points_group_plane_merge_out);
	TIMING_END_ms("MergeExtendPartWithRef realbad", TP1)

	TIMING_BEGIN(TP1)
	//Combine discrete planes and good planes together 
	CombinePlanes(&points_group_plane_merge_out);
	TIMING_END_ms("CombinePlanes discrete", TP1)
#endif
	}
	/*---------------generating discrete planes by all remaining voxels End---------------------*/

	TIMING_BEGIN(TP1)
	BadVoxelConnectedWithRef(true);
	TIMING_END_ms("BadVoxelConnectedWithRef true", TP1)

	TIMING_BEGIN(TP1)	
	if (filtering_strtgy_type != NO_NORMALEST)
	{
	IdentifyParentOfBadVoxelsWithRef(true);
	}
	else
	{
	IdentifyParentOfBadVoxelsWithRefNm(true);
	}
	TIMING_END_ms("IdentifyParentOfBadVoxels true", TP1)	
#ifndef VOXEL_NEAREST_APPROACH_MERGE
	TIMING_BEGIN(TP1)
	BadVoxelConnectedWithRef(false);
	if (filtering_strtgy_type != NO_NORMALEST)
	{
	IdentifyParentOfBadVoxelsWithRef(false);
	}
	else
	{
	IdentifyParentOfBadVoxelsWithRefNm(false);
	}
	TIMING_END_ms("BadVoxelConnectedWithRef false", TP1)
#endif
	TIMING_BEGIN(TP1)
	// Merge all points in bad voxels into its similar plane
	MergeBadVoxels();
	TIMING_END_ms("MergeBadVoxels", TP1)

	if(with_multiplane_points)
	{
		TIMING_BEGIN(TP1)
		MergeMultiplaneEdgePoints();
		TIMING_END_ms("MergeMultiplaneEdgePoints", TP1)
	}

	TIMING_BEGIN(TP1)
#ifdef SAVE_OUTPUT_FILE_DEBUG
	if(debug_config_params.voxel_info_debug)
	{
		for (unsigned int i = 0; i < plane_merge_out.size; i++)
		{
			SavePlaneEdgePointsDebug(current_out_path, i);
		}
	}
	TIMING_END_ms("save edgepoints", TP1)
#endif

	//SavePlanePointsDebug(output_path, 43, true); //SavePlanePointsDebug(output_path, 69, true);// SavePlanePointsDebug(output_path, 97);
	//SaveVoxelPointsDebug(output_path,10285);	SaveVoxelPointsDebug(output_path, 10286);
	//SaveBadVoxelPointsClassify(output_path, 23693);
	//PointsGroupDebug();
	TIMING_BEGIN(TP1)
	IdentifySamePlanes();
	TIMING_END_ms("MergeSamePlanes", TP1)

#ifdef SAVE_OUTPUT_FILE_DEBUG
	if(debug_config_params.merge_bad_voxel_debug)
	{
		TIMING_BEGIN(TP1)
		OutputBadVoxelMergeOutInfo(output_path);
		TIMING_END_ms("OutputBadVoxelMergeOutInfo", TP1)
	}
#endif

	
	TIMING_BEGIN(TP1)
	FreeBadVoxelItem();
	TIMING_END_ms("FreeBadVoxelItem", TP1)

	// Assign plane outputs
	TIMING_BEGIN(TP1)
	AssignPlaneOutputs();
	TIMING_END_ms("AssignPlaneOutputs", TP1)

	TIMING_BEGIN(TP1)
	GetPlaneArea();
	TIMING_END_ms("GetPlaneArea", TP1)

#ifdef SAVE_OUTPUT_FILE_DEBUG
	if(debug_config_params.plane_output_debug)
	{
		TIMING_BEGIN(TP1)
		PlaneMergeOutInfo(output_path);
		TIMING_END_ms("PlaneMergeOutInfo", TP1)
	}
#endif
	TIMING_BEGIN(TP1)
	RefinePlanesOutput(plane_segmentation_out);
	TIMING_END_ms("RefinePlanesOutput", TP1)
	return true;
}

void PlaneSegmentation::RefinePlanesOutput(PlaneSegmentationOutput* plane_segmentation_out)
{
	//refine the merged plane
	std::map<unsigned int, std::vector<std::vector<unsigned int>>> split_planes_map;
	split_planes_map.clear();
	int merged_plane_min_points = 40;
	size_t refine_plane_numbers = 0;
	for (unsigned int i = 0; i < this->plane_seg_out.size; i++)
	{
		std::vector<unsigned int> points_ind(plane_seg_out.planes[i].points.size);
		std::vector<std::vector<unsigned int>> points_group;
		for (size_t j = 0; j < plane_seg_out.planes[i].points.size; j++)
			points_ind[j] = plane_seg_out.planes[i].points.point_idx[j];
		ClassifyVoxelPoints(pt_cloud_xyz, points_ind, points_group, 0.3);
		if (points_group.size() > 1)
		{
			std::vector<std::vector<unsigned int>> refine_plane_idx;
			for (unsigned int m = 0; m < points_group.size(); m++)
			{
				if (points_group[m].size() > merged_plane_min_points)
				{
					refine_plane_idx.push_back(points_group[m]);
				}
			}
			if (refine_plane_idx.size() > 1)
			{
				split_planes_map[i] = refine_plane_idx;
				refine_plane_numbers += refine_plane_idx.size();
			}
			else
				refine_plane_numbers++;
		}
		else
			refine_plane_numbers++;

	}

	//refine_plane_numbers = plane_seg_out.size;
	// Assign the plane segmentation output
	if (static_cast<int> (plane_seg_out.size) < refine_plane_numbers)
	{
		PlaneSegmentationOutput refine_plane_output;
		refine_plane_output.size = refine_plane_numbers;
		refine_plane_output.planes = new PlaneItem[refine_plane_output.size];

		int j = 0;
		for (size_t i = 0; i < plane_seg_out.size; i++)
		{
			if (split_planes_map.find(i) != split_planes_map.end())
			{
				for (auto& pm : split_planes_map[i])
				{
					ClearSums(&refine_plane_output.planes[j].sums);
					for (auto& pp : pm)
						PushPoint(&refine_plane_output.planes[j].sums, pt_cloud_xyz.points[pp]);
					MathOperation::Compute(pm.size(), refine_plane_output.planes[j].sums, refine_plane_output.planes[j].plane_normal,
						refine_plane_output.planes[j].plane_center, refine_plane_output.planes[j].plane_mse);
					refine_plane_output.planes[j].plane_type = plane_seg_out.planes[i].plane_type;
					refine_plane_output.planes[j].points.size = pm.size();
					refine_plane_output.planes[j].points.point_idx = new unsigned int[refine_plane_output.planes[j].points.size];
					for (int m = 0; m < pm.size(); m++)
						refine_plane_output.planes[j].points.point_idx[m] = pm[m];
					GetOnePlaneArea(&refine_plane_output.planes[j]);
					//std::cout << "save split plane " << i << "\t" << j << std::endl;
					j++;
				}
			}
			else
			{
				refine_plane_output.planes[j].parent_voxel_idx = plane_seg_out.planes[i].parent_voxel_idx;
				refine_plane_output.planes[j].plane_area = plane_seg_out.planes[i].plane_area;
				refine_plane_output.planes[j].plane_center = plane_seg_out.planes[i].plane_center;
				refine_plane_output.planes[j].plane_mse = plane_seg_out.planes[i].plane_mse;
				refine_plane_output.planes[j].plane_normal = plane_seg_out.planes[i].plane_normal;
				refine_plane_output.planes[j].sums = plane_seg_out.planes[i].sums;
				refine_plane_output.planes[j].plane_type = plane_seg_out.planes[i].plane_type;
				refine_plane_output.planes[j].points.size = plane_seg_out.planes[i].points.size;
				refine_plane_output.planes[j].points.point_idx = new unsigned int[refine_plane_output.planes[j].points.size];
				for (size_t m = 0; m < refine_plane_output.planes[j].points.size; m++)
					refine_plane_output.planes[j].points.point_idx[m] = plane_seg_out.planes[i].points.point_idx[m];
				j++;
			}
		}
		plane_segmentation_out->size = refine_plane_output.size;
		plane_segmentation_out->planes = refine_plane_output.planes;
		planeSegmentationInLine::Features_PlaneSegOutFreeMemory(&plane_seg_out);
	}
	else
	{
		plane_segmentation_out->size = this->plane_seg_out.size;
		plane_segmentation_out->planes = this->plane_seg_out.planes;
	}
}

void PlaneSegmentation::IdentifyParentOfBadVoxelsWithRefNm(bool is_first)
{
	//unsigned int parent_voxel_idx;				// parent voxel index to occupied voxel array

	float min_point_plane_dist = plane_seg_params.plane_seg_thresholds.THRESHOLD_MIN_DIST_OF_POINT2VOXEL;

	if (is_first)
	{
		min_point_plane_dist = plane_seg_params.plane_seg_thresholds.THRESHOLD_MIN_DIST_OF_POINT2VOXEL / 2;
		min_point_plane_dist = plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_VOXEL < min_point_plane_dist ? plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_VOXEL : min_point_plane_dist;
	}

	float min_normal_diff = static_cast<float>(std::cos(plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_NORMAL_ANGEL_OF_EDGE_POINT * M_PI / 180));

	log_debug("min_point_plane_dist is %f in IdentifyParentOfBadVoxels", min_point_plane_dist);

	unsigned int flat_voxel_plane_size = good_voxel_plane_size + pseudobad_voxel_plane_size;
	// Identify parent voxel in all real bad voxels
//#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(point_merged_voxel_size); i++)
	{
		BadVoxelMergeOutpuItem* bad_voxel_merge_it = &bad_voxel_merge_item[i];
		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[bad_voxel_merge_it->bad_voxel_idx];

		if (plane_voxel->is_being_merged && plane_voxel->is_overall_merged) continue;
		if (!bad_voxel_merge_it->is_remainer_occupied) continue;

		unsigned int voxel_identify_plane_cnt = 0; // all points found in current voxels
		std::vector<unsigned int> voxel_identify_plane_cnt_list(plane_merge_out.size, 0);// all points found in current voxels by plane list

		for (unsigned int k = 0; k < plane_voxel->points.size; k++)
		{
			unsigned int point_idx = plane_voxel->points.point_idx[k];
			//if (bad_voxel_merge_it->point_merged_flag[k]) continue;
			if (bad_voxel_merge_it->closest_plane_idx[k] != std::numeric_limits<unsigned int>::max()) continue; //if this point is not identified  to be of a plane
			Point3f Point = pt_cloud_xyz.points[point_idx];
			//Point3f Point_normal = pt_cloud_normal.points[point_idx];
			unsigned int plane_idx = -1;

			for (unsigned int j = 0; j < 26; j++)
			{
				NeighborItem* neighbor_item = &plane_voxel->neighbors[j];
				//if (!neighbor_item->is_occupied) continue;
				if (!neighbor_item->is_connected) continue;
				PlaneVoxelItem* neighbor_plane_voxel = &plane_voxel_array.voxels[neighbor_item->voxel_idx];
				PlaneMergeItem* neighbor_plane_merge_item = &plane_merge_element[neighbor_item->voxel_idx];
				plane_idx = -1;
				//check if neighbour is good or psudobad voxel who have parent voxel
				if ((neighbor_plane_voxel->is_overall_merged) && (neighbor_plane_voxel->is_being_merged))
				{
					unsigned int cur_parent_voxel_idx = neighbor_plane_merge_item->parent_voxel_idx[0];
					plane_idx = voxel_to_plane_idx[cur_parent_voxel_idx];
					if (plane_idx >= plane_merge_out.size)
					{
						// log for debug, it is possible, while in multilple-thread environment, because have overlap for voxel's is_overall_merged 
						log_debug("plane_idx =%d exceed the size of total plane =%d ", plane_idx, plane_merge_out.size);
						continue;
					}
					//check if have points is connected with reference plane
					if (!bad_voxel_merge_it->is_plane_connected[plane_idx]) continue;
					unsigned int start_plane_idx = 0;
					/*if (!is_good_voxel_plane)
					{
						start_plane_idx = good_voxel_plane_size;
						// for speed, do not need find again in planes generated by good voxels;
					}*/

					if ((plane_idx >= start_plane_idx) && (plane_idx < plane_merge_out.size))
					{
						// check if point distance to neighbours parent plane is less than predefined ,and find the smalles distance plane in all the neighbors ,record the plane merge flag				
						Point3f plane_normal = plane_merge_out.planes[plane_idx].plane_normal;
						Point3f plane_center = plane_merge_out.planes[plane_idx].plane_center;

						float new_dist = ComputePointToPlaneDist<float>(Point, plane_normal, plane_center);
						if ((new_dist < bad_voxel_merge_it->smallest_dist[k]) && (new_dist < min_point_plane_dist))
						{
							bad_voxel_merge_it->smallest_dist[k] = new_dist;
							bad_voxel_merge_it->closest_plane_idx[k] = plane_idx;
							voxel_identify_plane_cnt_list[plane_idx]++;
							voxel_identify_plane_cnt++;
						}
					}
				}
				else
				{
					// when neighbor voxel is also point-to-plane mode, check its connected 
					//if (!bad_voxel_merge_it->neighbor_connected[j]) continue;
					// if neigbour voxel is not is_overall_merged, the points in the neighbour maybe the 2 kind of case:
					//1) directly be merged the in the plane by good or pseudo-bad voxel growing 
					//2) not being merged 
					unsigned int neighbor_bad_voxel_idx = voxel_idx_to_bad_voxel[neighbor_item->voxel_idx];
					BadVoxelMergeOutpuItem* neighbor_bad_voxel_merge_it = &bad_voxel_merge_item[neighbor_bad_voxel_idx];
					//if (neighbor_bad_voxel_merge_it->same_normal_plane_cnt == 0) continue; //skip  to next if neighbour have no points in plane
					unsigned int start_plane_idx = 0;
					//if (!is_good_voxel_plane) start_plane_idx = good_voxel_plane_size; // if identify in pseudobad planes
					for (unsigned int m = start_plane_idx; m < plane_merge_out.size; m++)
					{
						//check if have points is connected with reference plane
						if (!bad_voxel_merge_it->is_plane_connected[m]) continue;

						if (m < flat_voxel_plane_size)
						{
							if (is_first) continue;
							if (!neighbor_bad_voxel_merge_it->remaining_points_merged_plane[m]) continue;
						}
						else if (neighbor_bad_voxel_merge_it->being_in_group_plane != NULL)
						{
							unsigned int group_plane_idx = m - flat_voxel_plane_size;
							if (!neighbor_bad_voxel_merge_it->being_in_group_plane[group_plane_idx]) continue;
						}
						plane_idx = m;
						Point3f plane_normal = plane_merge_out.planes[plane_idx].plane_normal;
						Point3f plane_center = plane_merge_out.planes[plane_idx].plane_center;
						float new_dist = ComputePointToPlaneDist<float>(Point, plane_normal, plane_center);
						if ((new_dist < bad_voxel_merge_it->smallest_dist[k]) && (new_dist < min_point_plane_dist))
						{
							bad_voxel_merge_it->smallest_dist[k] = new_dist;
							bad_voxel_merge_it->closest_plane_idx[k] = plane_idx;
							voxel_identify_plane_cnt_list[plane_idx]++;
							voxel_identify_plane_cnt++;
						}
					}
				}
			}
		}

		if (voxel_identify_plane_cnt != 0)
		{
			for (unsigned int i = 0; i < plane_merge_out.size; i++)
			{
				if (voxel_identify_plane_cnt_list[i] > 0)
				{
					if (i < flat_voxel_plane_size)
					{
						bad_voxel_merge_it->remaining_points_merged_plane[i] = true;
					}
					else if (bad_voxel_merge_it->being_in_group_plane != NULL)
					{
						unsigned int group_plane_idx = i - flat_voxel_plane_size;
						bad_voxel_merge_it->being_in_group_plane[group_plane_idx] = true;
					}
					bad_voxel_merge_it->same_normal_plane_cnt++;

					/*if (!is_good_voxel_plane && (i < good_voxel_plane_size))
					{
						// it is impossible , should have bug
						log_error("voxel %d find point idx in good plane %d while is_good_voxel_plane %d ", bad_voxel_merge_it->bad_voxel_idx, i, is_good_voxel_plane);
					}*/
				}
			}
		}

		voxel_identify_plane_cnt_list.clear();
		voxel_identify_plane_cnt_list.shrink_to_fit();
	}
	return;
}


bool PlaneSegmentation::OccupiedVoxelInit(PointArray input_data, VoxelItem*& voxel_array, int& Occupied_voxel_size)
{
	PointGrid2PointIdx* point_to_subgrid_list = new PointGrid2PointIdx[input_data.size];
	grid_to_occupied = new unsigned int[total_num_of_voxel_grid];

	
	grid_to_occupied_size = total_num_of_voxel_grid;
	/*Point3f max_p, min_p;
	max_p.x = max_x; max_p.y = max_y; max_p.z = max_z;
	min_p.x = min_x; max_p.y = min_y; max_p.z = min_z;*/
#pragma omp parallel for
	for (long long i = 0; i < static_cast<long long>(total_num_of_voxel_grid); i++)
	{
		grid_to_occupied[i] = std::numeric_limits<unsigned int>::max();
	}

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(input_data.size); i++)
	{
		point_to_subgrid_list[i].cloud_point_index = i;
		Point3f point = input_data.points[i];
		point_to_subgrid_list[i].grid_idx = ConvertXYZToVoxelID(point.x, point.y, point.z);
	}

	std::sort(point_to_subgrid_list, point_to_subgrid_list + input_data.size, std::less<PointGrid2PointIdx>());

	unsigned int i = 0;
	size_t occupied_grid_cnt = 0;  // total occupied grid number of the voxel
	while (i < input_data.size)
	{
		unsigned int j = i + 1;
		while ((j < input_data.size) && (point_to_subgrid_list[i].grid_idx == point_to_subgrid_list[j].grid_idx))
		{
			++j;
		}
		grid_to_occupied[point_to_subgrid_list[i].grid_idx] = static_cast<unsigned int>(occupied_grid_cnt);
		occupied_grid_cnt++;
		i = j;
	}

	log_info("occupied_grid_cnt =%d", occupied_grid_cnt);
	Occupied_voxel_size = static_cast<int>(occupied_grid_cnt);

	voxel_array = new VoxelItem[occupied_grid_cnt];

	occupied_grid_cnt = 0;
	for (unsigned int i = 0; i < input_data.size;)
	{
		unsigned int j = i + 1;
		while ((j < input_data.size) && (point_to_subgrid_list[i].grid_idx == point_to_subgrid_list[j].grid_idx))
		{
			j++;
		}
		voxel_array[occupied_grid_cnt].is_good_voxel = false;
		voxel_array[occupied_grid_cnt].grid_idx = static_cast<unsigned int>(point_to_subgrid_list[i].grid_idx);
		voxel_array[occupied_grid_cnt].points.size = j - i;
		voxel_array[occupied_grid_cnt].points.point_idx = new unsigned int[j - i];
		i = j;
		occupied_grid_cnt++;
	}

	//assign input point index to the occupied voxel array
	occupied_grid_cnt = 0;
	for (unsigned int i = 0; i < input_data.size;)
	{
		//unsigned int first_point_idx = point_to_subgrid_list[i].cloud_point_index;
		unsigned int current_index = 0;
		voxel_array[occupied_grid_cnt].points.point_idx[0] = point_to_subgrid_list[i].cloud_point_index;
		unsigned int j = i + 1;
		current_index = 1;
		while ((j < input_data.size) && (point_to_subgrid_list[i].grid_idx == point_to_subgrid_list[j].grid_idx))
		{

			voxel_array[occupied_grid_cnt].points.point_idx[current_index] = point_to_subgrid_list[j].cloud_point_index;
			current_index++;
			j++;
		}
		i = j;
		occupied_grid_cnt++;
	}
	delete[] point_to_subgrid_list;
	return  true;
}


bool PlaneSegmentation::MergeMultiplaneEdgePoints()
{
	float min_point_plane_dist = plane_seg_params.plane_seg_thresholds.THRESHOLD_MIN_DIST_OF_POINT2VOXEL / 2;
	min_point_plane_dist = plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_VOXEL < min_point_plane_dist ? plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_VOXEL : min_point_plane_dist;

	unsigned int total_plane_size = plane_merge_out.size;
	//#pragma omp parallel for
	for (int j = 0; j < static_cast<int>(point_merged_voxel_size); j++)
	{
		BadVoxelMergeOutpuItem* bad_voxel_it = &bad_voxel_merge_item[j];
		bad_voxel_it->is_multi_plane_connected = new bool[total_plane_size];
	}

	//#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(total_plane_size); i++)
	{
		for (unsigned int j = 0; j < point_merged_voxel_size; j++)
		{
			BadVoxelMergeOutpuItem* bad_voxel_it = &bad_voxel_merge_item[j];
			bad_voxel_it->is_multi_plane_connected[i] = false;
			BadVoxelConnectedWithMultiRef(j,i,bad_voxel_it->is_multi_plane_connected[i]);
		}		
	}

//#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(total_plane_size); i++)
	{
		//to find multiplanes points for all planes in remaining voxel
		PlaneMergeOutputItem* plane_it = &plane_merge_out.planes[i];
		int edge_point_num = 0; // sum of points in multiplanes
		// step 1: identify the multiplane points is in the plane
		std::vector<std::vector<unsigned int>>closest_plane_idx;// record all the pseud bad voxel and bad voxel points closest plane idx
		closest_plane_idx.clear();
		closest_plane_idx.resize(point_merged_voxel_size);
		for (unsigned int j = 0; j < point_merged_voxel_size; j++)
		{
			BadVoxelMergeOutpuItem* bad_voxel_it = &bad_voxel_merge_item[j];
			PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[bad_voxel_it->bad_voxel_idx];

			closest_plane_idx[j].resize(bad_voxel_it->voxel_point_size, std::numeric_limits<unsigned int>::max());

			//if (debug_con) log_debug("voxel%d  is_multi_plane_connected[%d] =%d", bad_voxel_it->bad_voxel_idx,i, bad_voxel_it->is_multi_plane_connected[i]);
			if (!bad_voxel_it->is_multi_plane_connected[i]) continue; // must connected to this plane
			//PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[bad_voxel_it->bad_voxel_idx];
			if ((plane_voxel->is_overall_merged) && (plane_voxel->is_being_merged))
			{
				unsigned int plane_idx = voxel_to_plane_idx[bad_voxel_it->bad_voxel_idx];
				if (plane_idx == i) continue;
				for (unsigned int k = 0; k < bad_voxel_it->voxel_point_size; k++)
				{
					//unsigned int tmp_point_idx = plane_voxel->points.point_idx[k];
					Point3f Point = pt_cloud_xyz.points[plane_voxel->points.point_idx[k]];
					float point_dist = ComputePointToPlaneDist<float>(Point, plane_it->plane_normal, plane_it->plane_center);
					if (point_dist < min_point_plane_dist)
					{
						closest_plane_idx[j][k] = i;
						edge_point_num++;
					}
				}
			}
			else
			{
				for (unsigned int k = 0; k < bad_voxel_it->voxel_point_size; k++)
				{
					//unsigned int tmp_point_idx = plane_voxel->points.point_idx[k];
					if ((bad_voxel_it->closest_plane_idx[k] != std::numeric_limits<unsigned int>::max()) && (bad_voxel_it->closest_plane_idx[k] != i))
					{
						Point3f Point = pt_cloud_xyz.points[plane_voxel->points.point_idx[k]];
						float point_dist = ComputePointToPlaneDist<float>(Point, plane_it->plane_normal, plane_it->plane_center);
						if (point_dist < min_point_plane_dist)
						{
							closest_plane_idx[j][k] = i;
							edge_point_num++;
						}
					}
				}
			}
		}

		//log_info("plane i =%d edge_point_num = %d", i,edge_point_num);
		// step 2: add points to the plane
		plane_it->multiplane_points.size = edge_point_num;
		plane_it->total_point_cnt += edge_point_num;
		plane_it->multiplane_points.point_idx = new unsigned int[edge_point_num];
		edge_point_num = 0;
		for (unsigned int j = 0; j < point_merged_voxel_size; j++)
		{
			BadVoxelMergeOutpuItem* bad_voxel_it = &bad_voxel_merge_item[j];
			//if (!bad_voxel_it->is_remainer_occupied) continue;
			if (!bad_voxel_it->is_multi_plane_connected[i]) continue; // must connected to this plane
			PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[bad_voxel_it->bad_voxel_idx];

			//if ((plane_voxel->is_overall_merged) && (plane_voxel->is_being_merged)) continue;
			for (unsigned int k = 0; k < bad_voxel_it->voxel_point_size; k++)
			{
				if (closest_plane_idx[j][k] == i)
				{
					plane_it->multiplane_points.point_idx[edge_point_num] = plane_voxel->points.point_idx[k];
					Point3f point = pt_cloud_xyz.points[plane_voxel->points.point_idx[k]];
					PushPoint(&plane_it->sums, point);
					edge_point_num++;
				}
			}
		}
	}	
	return true;

}


bool PlaneSegmentation::InitializeParamsNoNm(const NormalEstimationInterface inputStruct)
{
	if ((inputStruct.points.size == 0) || (inputStruct.voxel_para.length_x_of_voxel <= 0))
	{
		log_error("input error");
		return false;
	}

	point_density = inputStruct.density;
	pt_cloud_xyz = inputStruct.points;
	plane_seg_params.voxel_params = inputStruct.voxel_para;
	length_x_of_voxel = plane_seg_params.voxel_params.length_x_of_voxel;
	length_y_of_voxel = plane_seg_params.voxel_params.length_y_of_voxel;
	length_z_of_voxel = plane_seg_params.voxel_params.length_z_of_voxel;
	length_x_of_voxel_inverse = 1 / length_x_of_voxel;
	length_y_of_voxel_inverse = 1 / length_y_of_voxel;
	length_z_of_voxel_inverse = 1 / length_z_of_voxel;
	//grid_to_occupied_size = total_num_of_voxel_grid;
	min_high_mse_ratio = plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_HIGH_MSE_RATIO;
	max_mse_of_mini_plane = plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_PLANE;
	if (sys_control_para.scanner_type == UNRE_TYPE_0)
	{
		max_mse_of_mini_plane = plane_seg_params.plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_PLANE / 2;
	}
	log_debug("length_x_of_voxel=%f", length_x_of_voxel);
	return true;
}


bool PlaneSegmentation::FitPlane(const std::string output_path, const std::string config_file, NormalEstimationInterface& inputStruct, PlaneSegmentationOutput* plane_segmentation_out)
{
	//planeSegConfigParse config_it;
	planeSegCfgParse config_it;
	bool rtn = config_it.GetPlaneSegConfigure(config_file);
	if (!rtn)
	{
		log_error("load config file %s failed", config_file.c_str());
		return false;
	}
	sys_control_para = config_it.getSysConfigParams();
	config_params = config_it.getSegConfigParams();
	//plane_seg_params.voxel_params = config_it.getPlaneSegVoxelSize();
	plane_seg_params.voxel_params = config_it.getVoxelSize();
	plane_seg_params.plane_seg_thresholds = config_it.getPlaneSegThresholds();
	FitPlaneMethod(config_params.filtering_strtgy_type, output_path, inputStruct, plane_segmentation_out);
	return true;
}

bool PlaneSegmentation::FitPlane(const std::string output_path, NormalEstimationInterface& inputStruct, PlaneSegmentationOutput* plane_segmentation_out)
{
	return (FitPlaneMethod(NO_FILTER, output_path, inputStruct, plane_segmentation_out));
}



