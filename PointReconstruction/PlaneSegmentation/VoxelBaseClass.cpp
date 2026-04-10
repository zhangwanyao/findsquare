#include "VoxelBaseClass.h"
#include <iostream>
#include "log.h"
#include "MathOperation.hpp"
#include <stdlib.h>

using namespace ModuleStruct;

VoxelBaseClass::VoxelBaseClass() {

	this->Init();
}

VoxelBaseClass::~VoxelBaseClass() {
	FreeMemory();
}

void VoxelBaseClass::Init() {

	num_of_occupied_voxel = 0;
	cols_of_voxel = rows_of_voxel = depths_of_voxel = 0;
	total_num_of_voxel_grid = 0;
	min_x = min_y = min_z = std::numeric_limits<float>::infinity();
	max_x = max_y = max_z = -std::numeric_limits<float>::infinity();
	num_of_bridge = num_of_good_bridge = num_of_pseudo_bad_bridge = num_of_real_bad_bridge = num_of_single_bridge = max_num_of_bridge_planes = 0;
	pt_cloud_xyz.points = NULL;
	return;
}

void VoxelBaseClass::FreeMemory() {
	/*
	if (pt_cloud_xyz.points != NULL) {
		free(pt_cloud_xyz.points);
		pt_cloud_xyz.points = NULL;	
	}*/
}

/*
void VoxelBaseClass::Reset() {
	
	this->Init();

	FreeMemory();

	return;
}*/

bool VoxelBaseClass::GetMinMaxXYZ (PointArray input_data){

	if (input_data.size==0){
		log_error("empty input");
		return false;
	}

	for (unsigned int i = 0; i < pt_cloud_xyz.size; i++){
		min_x = (min_x > input_data.points[i].x) ? input_data.points[i].x : min_x;
		max_x = (max_x < input_data.points[i].x) ? input_data.points[i].x : max_x;
		min_y = (min_y > input_data.points[i].y) ? input_data.points[i].y : min_y;
		max_y = (max_y < input_data.points[i].y) ? input_data.points[i].y : max_y;
		min_z = (min_z > input_data.points[i].z) ? input_data.points[i].z : min_z;
		max_z = (max_z < input_data.points[i].z) ? input_data.points[i].z : max_z;
	}

	return true;
}

void VoxelBaseClass::GetTotalNumOfVoxelGrid(){

	//if ((max_x - min_x) != 0 && std::remainder((max_x - min_x), length_x_of_voxel) == 0)
	//	cols_of_voxel = (unsigned long long)(std::floor((max_x - min_x) * length_x_of_voxel_inverse));
	//else
	//	cols_of_voxel = (unsigned long long)(std::floor((max_x - min_x) * length_x_of_voxel_inverse) + 1);

	//if ((max_y - min_y) != 0 && std::remainder((max_y - min_y), length_y_of_voxel) == 0)
	//	rows_of_voxel = (unsigned long long)( std::floor((max_y - min_y) * length_y_of_voxel_inverse));
	//else
	//	rows_of_voxel = (unsigned long long)(std::floor((max_y - min_y) * length_y_of_voxel_inverse) + 1);

	//if ((max_z - min_z) != 0 && std::remainder((max_z - min_z), length_z_of_voxel) == 0)
	//	depths_of_voxel = (unsigned long long)(std::floor((max_z - min_z) * length_z_of_voxel_inverse));
	//else
	//	depths_of_voxel = (unsigned long long)( std::floor((max_z - min_z) * length_z_of_voxel_inverse) + 1);

	cols_of_voxel = int(std::floor((max_x - min_x) / length_x_of_voxel) + 1);

	rows_of_voxel = int(std::floor((max_y - min_y) / length_y_of_voxel) + 1);

	depths_of_voxel = int(std::floor((max_z - min_z) / length_z_of_voxel) + 1);

	total_num_of_voxel_grid = (unsigned long long )(cols_of_voxel*rows_of_voxel*depths_of_voxel);

	return;
}

void VoxelBaseClass::GetTotalNumOfVoxelGrid(const int buffer){

	if ((max_x - min_x) != 0 && std::remainder((max_x - min_x), length_x_of_voxel) == 0)
		cols_of_voxel = (unsigned int)(std::floor((max_x - min_x) * length_x_of_voxel_inverse));
	else
		cols_of_voxel = (unsigned int)(std::floor((max_x - min_x) * length_x_of_voxel_inverse) + 1);

	if ((max_y - min_y) != 0 && std::remainder((max_y - min_y), length_y_of_voxel) == 0)
		rows_of_voxel = (unsigned int)(std::floor((max_y - min_y) * length_y_of_voxel_inverse));
	else
		rows_of_voxel = (unsigned int)(std::floor((max_y - min_y) * length_y_of_voxel_inverse) + 1);

	if ((max_z - min_z) != 0 && std::remainder((max_z - min_z), length_z_of_voxel) == 0)
		depths_of_voxel = (unsigned int)(std::floor((max_z - min_z) * length_z_of_voxel_inverse));
	else
		depths_of_voxel = (unsigned int)(std::floor((max_z - min_z) * length_z_of_voxel_inverse) + 1);

	cols_of_voxel += buffer;
	rows_of_voxel += buffer;
	depths_of_voxel += buffer;

	total_num_of_voxel_grid = cols_of_voxel*rows_of_voxel*depths_of_voxel;

	return;
}

unsigned long long VoxelBaseClass::ConvertXYZToVoxelID(const float x, const float y, const float z){

	//unsigned int col_idx = (unsigned int)(std::floor((x - min_x) * length_x_of_voxel_inverse));
	//unsigned int row_idx = (unsigned int)( std::floor((y - min_y) * length_y_of_voxel_inverse));
	//unsigned int height_idx = (unsigned int)(std::floor((z - min_z) * length_z_of_voxel_inverse));
	unsigned int col_idx = (unsigned int)(std::floor((x - min_x) / length_x_of_voxel));
	unsigned int row_idx = (unsigned int)( std::floor((y - min_y) / length_y_of_voxel));
	unsigned int height_idx = (unsigned int)(std::floor((z - min_z) / length_z_of_voxel));

	//if (col_idx == cols_of_voxel)
	//	col_idx--;
	//if (row_idx == rows_of_voxel)
	//	row_idx--;
	//if (height_idx == depths_of_voxel)
	//	height_idx--;

	return height_idx*(cols_of_voxel*rows_of_voxel) + row_idx*cols_of_voxel + col_idx;
}


void VoxelBaseClass::ConvertVoxelIDTo3dID(const Point3f &pt_xyz, int &row_idx, int &col_idx, int &depth_idx)
{

	col_idx = (int)(std::floor((pt_xyz.x - min_x) / length_x_of_voxel));
	row_idx =(int)( std::floor((pt_xyz.y - min_y) / length_y_of_voxel));
	depth_idx = (int)(std::floor((pt_xyz.z - min_z) / length_z_of_voxel));

	//if (col_idx == cols_of_voxel)
	//	col_idx--;
	//if (row_idx == rows_of_voxel)
	//	row_idx--;
	//if (depth_idx == depths_of_voxel)
	//	depth_idx--;
	return;
}


void VoxelMergeBaseClass::Init()
{
	this->ResetVoxel();
}

void VoxelMergeBaseClass::FreeMemory() 
{
	// free memory of storing all point indexes in each occupied voxels
	if (voxel_grid->points.point_idx != NULL)
	{
		free(voxel_grid->points.point_idx);
		voxel_grid->points.point_idx = NULL;
	}	
}

void VoxelMergeBaseClass::ResetVoxel() 
{
	    voxel_grid->voxel_type = REAL_BAD_VOXEL; // occupied voxel init value is REAL_BAD_VOXEL default
		voxel_grid->grid_idx = std::numeric_limits<unsigned int>::max();
		voxel_grid->plane_center.x = 0.f;
		voxel_grid->plane_center.y = 0.f;
		voxel_grid->plane_center.z = 0.f;
		voxel_grid->plane_normal.x = 0.f;
		voxel_grid->plane_normal.y = 0.f;
		voxel_grid->plane_normal.z = 0.f;
		voxel_grid->avg_normal.x = 0.f;
		voxel_grid->avg_normal.y = 0.f;
		voxel_grid->avg_normal.z = 0.f;
		voxel_grid->plane_mse = std::numeric_limits<float>::infinity();
		voxel_grid->avg_mse = std::numeric_limits<float>::infinity();
		voxel_grid->plane_high_mse_ratio = 0.0f;
		voxel_grid->avg_high_mse_ratio = 0.0f;
		// free memory of all PlaneVoxelItem in PlaneVoxelArray
		voxel_grid->points.size = 0;
		voxel_grid->points.point_idx = NULL;		
		voxel_grid->is_being_merged = false;
		voxel_grid->is_overall_merged = false;
		voxel_grid->is_good_normal = false;
		voxel_grid->is_flat = false;
		voxel_grid->is_tiny = false;

		voxel_grid->sums.sum_xx = voxel_grid->sums.sum_yy = voxel_grid->sums.sum_zz = 0.0;
		voxel_grid->sums.sum_xy = voxel_grid->sums.sum_xz = voxel_grid->sums.sum_yz = 0.0;
		voxel_grid->sums.sum_x = voxel_grid->sums.sum_y = voxel_grid->sums.sum_z = 0.0;
		voxel_grid->is_same_with_ref_plane = NULL;
		voxel_grid->ref_plane_cnt = 0;
		for (int i = 0; i<26; i++)
		{
			voxel_grid->neighbors[i].is_occupied = false;
			voxel_grid->neighbors[i].voxel_idx = std::numeric_limits<unsigned int>::max();
			voxel_grid->neighbors[i].normal_diff = std::numeric_limits<float>::infinity();
			voxel_grid->neighbors[i].plane_dist = std::numeric_limits<float>::infinity();
			voxel_grid->neighbors[i].neighbor_flag = false;
			//voxel_grid->neighbors[i].balanced_neighbor_flag = false;
			voxel_grid->neighbors[i].plane_idx_of_bridge = std::numeric_limits<unsigned int>::max();
			voxel_grid->neighbors[i].bridge_plane_idx_of_plane_pair = std::numeric_limits<unsigned int>::max();
			voxel_grid->neighbors[i].is_connected = false;

		}

}


VoxelMergeBaseClass::VoxelMergeBaseClass(PlaneVoxelItem* voxel_grid_input, PlaneFitThresholds *plane_seg_thresholds)
{
	voxel_grid = voxel_grid_input;
	plane_fit_thresholds = plane_seg_thresholds;	
}

VoxelMergeBaseClass::~VoxelMergeBaseClass() 
{
	voxel_grid = NULL;
	plane_fit_thresholds = NULL;
}

//compute plane properties
void VoxelMergeBaseClass::Compute(unsigned int point_cnt)
{
		if (point_cnt <= 0) return;

		double point_num_inverse = (double)1.0 / point_cnt;

		//plane center
		voxel_grid->plane_center.x = (float)(voxel_grid->sums.sum_x*point_num_inverse);
		voxel_grid->plane_center.y = (float)(voxel_grid->sums.sum_y*point_num_inverse);
		voxel_grid->plane_center.z = (float)(voxel_grid->sums.sum_z*point_num_inverse);

		//calculate covariance matrix
		double cov_mat_element[6] = { voxel_grid->sums.sum_xx - voxel_grid->sums.sum_x*voxel_grid->plane_center.x,voxel_grid->sums.sum_xy - voxel_grid->sums.sum_x*voxel_grid->plane_center.y,
			voxel_grid->sums.sum_xz - voxel_grid->sums.sum_x*voxel_grid->plane_center.z,voxel_grid->sums.sum_yy - voxel_grid->sums.sum_y*voxel_grid->plane_center.y,
			voxel_grid->sums.sum_yz - voxel_grid->sums.sum_y*voxel_grid->plane_center.z,voxel_grid->sums.sum_zz - voxel_grid->sums.sum_z*voxel_grid->plane_center.z };


#ifdef FAST_COMPUTE_EIGEN
		double eig_val[3], eig_vec[3];

		MathOperation::ComputeFastEigenParallel(&cov_mat_element[0], &eig_val[0], &eig_vec[0]);
		voxel_grid->plane_normal.x = eig_vec[0];
		voxel_grid->plane_normal.y = eig_vec[1];
		voxel_grid->plane_normal.z = eig_vec[2];

		//plane mean squared error
		voxel_grid->plane_mse = eig_val[2] * point_num_inverse;
#else
		double cov_mat_arr[3][3] = { { cov_mat_element[0],cov_mat_element[1],cov_mat_element[2] },
		{ cov_mat_element[1], cov_mat_element[3], cov_mat_element[4] },
		{ cov_mat_element[2], cov_mat_element[4], cov_mat_element[5] } };

		CMat cov_mat(3, 3, CV_64F, cov_mat_arr);
		cov_mat *= point_num_inverse;

		CMat eig_val_mat, eig_vec_mat;

		cv::eigen(cov_mat, eig_val_mat, eig_vec_mat);

		voxel_grid->plane_normal.x = (float)(eig_vec_mat.at<double>(2, 0));
		voxel_grid->plane_normal.y = (float)(eig_vec_mat.at<double>(2, 1));
		voxel_grid->plane_normal.z = (float)(eig_vec_mat.at<double>(2, 2));

		//plane mean squared error
		//voxel_grid->plane_mse = std::fabs((float)(eig_val_mat.at<double>(2) * point_num_inverse));
		//voxel_grid->eigen_mse = std::fabs((float)(eig_val_mat.at<double>(2) * point_num_inverse));
#endif

		//curvature
		//plane_curvature = eig_val[2] / (eig_val[0] + eig_val[1] + eig_val[2]);
}

void VoxelMergeBaseClass::Compute(unsigned int point_cnt, SumforCovariance sums, Point3f &plane_normal, Point3f &plane_center)
{
	if (point_cnt <= 0) return;

	double point_num_inverse = (double)1.0 / point_cnt;

	//plane center
	plane_center.x = (float)(sums.sum_x*point_num_inverse);
	plane_center.y = (float)(sums.sum_y*point_num_inverse);
	plane_center.z = (float)(sums.sum_z*point_num_inverse);

	//calculate covariance matrix
	double cov_mat_element[6] = { sums.sum_xx - sums.sum_x*plane_center.x,sums.sum_xy - sums.sum_x*plane_center.y,
		sums.sum_xz - sums.sum_x*plane_center.z,sums.sum_yy - sums.sum_y*plane_center.y,
		sums.sum_yz - sums.sum_y*plane_center.z,sums.sum_zz - sums.sum_z*plane_center.z };


#ifdef FAST_COMPUTE_EIGEN
	double eig_val[3], eig_vec[3];

	MathOperation::ComputeFastEigenParallel(&cov_mat_element[0], &eig_val[0], &eig_vec[0]);
	voxel_grid->plane_normal.x = eig_vec[0];
	voxel_grid->plane_normal.y = eig_vec[1];
	voxel_grid->plane_normal.z = eig_vec[2];

	//plane mean squared error
	voxel_grid->plane_mse = eig_val[2] * point_num_inverse;
#else
	double cov_mat_arr[3][3] = { { cov_mat_element[0],cov_mat_element[1],cov_mat_element[2] },
	{ cov_mat_element[1], cov_mat_element[3], cov_mat_element[4] },
	{ cov_mat_element[2], cov_mat_element[4], cov_mat_element[5] } };

	CMat cov_mat(3, 3, CV_64F, cov_mat_arr);
	cov_mat *= point_num_inverse;

	CMat eig_val_mat, eig_vec_mat;

	eigen(cov_mat, eig_val_mat, eig_vec_mat);

	plane_normal.x = (float)(eig_vec_mat.at<double>(2, 0));
	plane_normal.y = (float)(eig_vec_mat.at<double>(2, 1));
	plane_normal.z = (float)(eig_vec_mat.at<double>(2, 2));
	//plane mean squared error
	//voxel_grid->plane_mse = std::fabs((float)(eig_val_mat.at<double>(2) * point_num_inverse));
	//voxel_grid->eigen_mse = std::fabs((float)(eig_val_mat.at<double>(2) * point_num_inverse));
#endif

}

 // get SumforCovariance sums of all points in current voxel
void VoxelMergeBaseClass::GetVoxelSums(PointArray* point_array)
{
	//unsigned int *index = voxel_grid->points.point_idx;
	for (size_t i = 0; i < voxel_grid->points.size; i++)
	{
		unsigned int index = voxel_grid->points.point_idx[i];

		// Elaine Li - comment out for performance issue
		// (index < point_array->size)
		{
			Point3f point = point_array->points[index];
			Push(point.x, point.y, point.z);
		}
	}
	
}

// get a neighbour item of current voxel except voxel_type 
void VoxelMergeBaseClass::GetVoxelNeighborItem(unsigned int neigbour_index, unsigned int neighbour_voxel_occupied_index, PlaneVoxelItem* plane_voxel_array)
{

	NeighborItem* neighbour_item = &voxel_grid->neighbors[neigbour_index];
	
	PlaneVoxelItem*  neighbour_voxel = plane_voxel_array;
	
	neighbour_item->is_occupied = true;

	neighbour_item->voxel_idx = neighbour_voxel_occupied_index;
	
	//compute neighbour item normal difference
#ifdef COMPARE_NORMAL_DIFF_IN_ANGLE
	float normal_sim = std::fabs(ComputeNormalsSimilarity(neighbour_voxel->plane_normal));
	if (normal_sim > 1.0f) normal_sim = 1.0f;
	neighbour_item->normal_diff = (float)((std::acos(normal_sim) * 180.0) / M_PI);
	//neighbour_item->normal_diff = (std::acos(std::fabs(ComputeNormalsSimilarity(neighbour_voxel->plane_normal))) * 180.0) / M_PI;
#else
	neighbour_item->normal_diff = std::fabs(ComputeNormalsSimilarity(neighbour_voxel->plane_normal));
#endif

	//compute neighbour item plane distance 
	neighbour_item->plane_dist = GetPointPlaneDist(neighbour_voxel->plane_center);

#ifdef COMPARE_NORMAL_DIFF_IN_ANGLE
	if ((neighbour_item->normal_diff <= plane_fit_thresholds->THRESHOLD_MAX_NORMAL_ANGLE_OF_TWO_VOXEL) && (neighbour_item->plane_dist <= plane_fit_thresholds->THRESHOLD_MIN_PLANE_DIST_OF_TWO_VOXEL))
#else
	if ((neighbour_item->normal_diff > std::cos(plane_fit_thresholds->THRESHOLD_MAX_NORMAL_ANGLE_OF_TWO_VOXEL*M_PI / 180.0)) && (neighbour_item->plane_dist <= plane_fit_thresholds->THRESHOLD_MIN_PLANE_DIST_OF_TWO_VOXEL))
#endif
	{
		neighbour_item->neighbor_flag = true;
	}

	return;
}

// Update normal_diff & neighbor_flag according to neighbor's parent after merging
void VoxelMergeBaseClass::UpdateVoxelNeighborItem(NeighborItem* neighbour_item, PlaneVoxelItem* neighbour_parent_voxel)
{
	//compute neighbor item normal difference
#ifdef COMPARE_NORMAL_DIFF_IN_ANGLE
	float normal_sim = std::fabs(ComputeNormalsSimilarity(neighbour_parent_voxel->plane_normal));
	if (normal_sim > 1.0f) normal_sim = 1.0f;
	neighbour_item->normal_diff = (float)((std::acos(normal_sim) * 180.0) / M_PI);
	//neighbour_item->normal_diff = (std::acos(std::fabs(ComputeNormalsSimilarity(neighbour_parent_voxel->plane_normal))) * 180.0) / M_PI;
	if ((neighbour_item->normal_diff <= plane_fit_thresholds->THRESHOLD_MAX_NORMAL_ANGLE_OF_TWO_VOXEL) && (neighbour_item->plane_dist <= plane_fit_thresholds->THRESHOLD_MIN_PLANE_DIST_OF_TWO_VOXEL))
#else
	neighbour_item->normal_diff = std::fabs(ComputeNormalsSimilarity(neighbour_parent_voxel->plane_normal));
	if ((neighbour_item->normal_diff > std::cos(plane_fit_thresholds->THRESHOLD_MAX_NORMAL_ANGLE_OF_TWO_VOXEL*M_PI / 180.0)) && (neighbour_item->plane_dist <= plane_fit_thresholds->THRESHOLD_MIN_PLANE_DIST_OF_TWO_VOXEL))
#endif
	{
		neighbour_item->neighbor_flag = true;
	}

	return;
}

