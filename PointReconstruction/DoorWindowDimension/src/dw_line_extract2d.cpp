#include <fstream>
#include <iomanip> //std::stew
#include "log.h"
#include "in_out_data.hpp"
#include "ModuleStruct.hpp"
#include "../../Common/MathOperation.h"
//#include "common/in_out_data.hpp"
#include "util_math.hpp"
#include "line_extract/line_extract2d.h"
#include "common_struct.h"

#ifndef M_PI
#define M_PI       3.14159265358979323846   // pi
#endif

using namespace Util_Math;
using namespace ModuleStruct;

/**
* \brief Identify the two points group are close or connected by pre-specified distance in 2D space
* @param const std::vector<Point2f> pt_cloud_xyz,input parameter, input data cloud data in 2D space
* @param const std::vector<unsigned int> first_points, input parameter, the point indices of first points group
* @param const PointInVoxelArray second_points , input parameter, the point indices of second points group
* @param const float distance , input parameter, pre-specified distance
* @return true if the two points group distance is lower thean pre-specified distance ,otherwise return false
*/
static inline bool Identify2PointsGroupCLoseByDistance(const std::vector<cv::Point2f> pt_cloud_xyz, 
														const std::vector<unsigned int> first_points, 
														const PointInVoxelArray second_points, const float distance) {
	if ((first_points.size() == 0) || (second_points.size == 0))
	{
		//log_error(" Input first_points size = %d second_points size =%d error ", first_points.size(), second_points.size);
		return false;
	}
	// step 1: get points center of 2 points group
	cv::Point2f first_points_center = { 0.f,0.f };
	for (int i = 0; i < first_points.size(); i++)
	{
		cv::Point2f point = pt_cloud_xyz[first_points[i]];
		first_points_center += point;
	}
	int first_point_cnt = static_cast<int>(first_points.size());
	first_points_center /= first_point_cnt;

	cv::Point2f sec_points_center = { 0.f,0.f };
	for (unsigned int i = 0; i < second_points.size; i++)
	{
		cv::Point2f point = pt_cloud_xyz[second_points.point_idx[i]];
		sec_points_center += point;
	}
	int sec_point_cnt = second_points.size;
	sec_points_center = sec_points_center / sec_point_cnt;

	// step 2:  sort 2 points by distance to the center of 2 points group
	std::vector <PointIdx2Dist> first_idx_to_dist(first_points.size());
	for (size_t i = 0; i < first_points.size(); i++)
	{
		cv::Point2f point = pt_cloud_xyz[first_points[i]];
		first_idx_to_dist[i].cloud_point_idx = first_points[i];
		//ModuleStruct::Point2f delta_point = point - sec_points_center;
		//float distance = std::fabs(delta_point.x) + std::fabs(delta_point.y);
		float distance = Util_Math::DWComputePointToPointDist2D<cv::Point2f>(point, sec_points_center);
		first_idx_to_dist[i].delta_dist = distance;
	}
	std::sort(first_idx_to_dist.begin(), first_idx_to_dist.end(), std::less<PointIdx2Dist>{});

	std::vector <PointIdx2Dist>sec_idx_to_dist(second_points.size);
	for (size_t i = 0; i < second_points.size; i++)
	{
		cv::Point2f point = pt_cloud_xyz[second_points.point_idx[i]];
		sec_idx_to_dist[i].cloud_point_idx = second_points.point_idx[i];
		//ModuleStruct::Point2f delta_point = point - first_points_center;
		//float distance = std::fabs(delta_point.x) + std::fabs(delta_point.y);
		float distance = Util_Math::DWComputePointToPointDist2D<cv::Point2f>(point, first_points_center);
		sec_idx_to_dist[i].delta_dist = distance;
	}
	std::sort(sec_idx_to_dist.begin(), sec_idx_to_dist.end(), std::less<PointIdx2Dist>{});

	// step 3:   each group use 2 closest points  to  compute center distance, in 2D space only need 2  points to identify close of 2 points group 
	float dist1 = Util_Math::DWComputePointToPointDist2D<cv::Point2f>(pt_cloud_xyz[first_idx_to_dist[0].cloud_point_idx], pt_cloud_xyz[sec_idx_to_dist[0].cloud_point_idx]);
	float min_dist = dist1;
	if ((first_points.size() == 1) || (second_points.size == 1))
	{
		if (min_dist < distance)  return true;
		return false;
	}

	int first_comp_cnt = 2;
	int sec_comp_cnt = 2;
	if (first_comp_cnt > static_cast<int>(first_points.size()))  first_comp_cnt = static_cast<int>(first_points.size());
	if (sec_comp_cnt > static_cast<int>(second_points.size))  sec_comp_cnt = static_cast<int>(second_points.size);

	cv::Point2f first_comp_center = { 0.f,0.f };
	for (int i = 0; i < first_comp_cnt; i++)
	{
		first_comp_center += pt_cloud_xyz[first_idx_to_dist[i].cloud_point_idx];
	}
	first_comp_center = first_comp_center / first_comp_cnt;

	cv::Point2f sec_comp_center = { 0.f,0.f };
	for (int i = 0; i < sec_comp_cnt; i++)
	{
		sec_comp_center += pt_cloud_xyz[sec_idx_to_dist[i].cloud_point_idx];
	}
	sec_comp_center = sec_comp_center / sec_comp_cnt;

	float dist = Util_Math::DWComputePointToPointDist2D<cv::Point2f>(first_comp_center, sec_comp_center);
	min_dist = min_dist < dist ? min_dist : dist;
	bool rtn = false;
	if (min_dist < distance) rtn = true;
	return rtn;

}

/**
* \brief Identify the two points group are close or connected by pre-specified distance in 2D space
* @param const std::vector<ModuleStruct::Point2f> pt_cloud_xyz,input parameter, input data cloud data in 2D space
* @param const std::vector<unsigned int> first_points, input parameter, the point indices of first points group
* @param const std::vector<unsigned int>  second_points , input parameter, the point indices of second points group
* @param const float distance , input parameter, pre-specified distance
* @return true if the two points group distance is lower thean pre-specified distance ,otherwise return false
*/
static inline bool Identify2PointsGroupCLoseByDistance1(const std::vector<cv::Point2f> pt_cloud_xyz, const std::vector<unsigned int> first_points, const std::vector<unsigned int> second_points, const float distance) {
	// step 1: get points center of 2 points group

	if ((first_points.size() == 0) || (second_points.size() == 0))
	{
		//log_error(" Input first_points size = %d second_points size =%d error ", first_points.size(), second_points.size());
		return false;
	}

	cv::Point2f first_points_center = { 0.f,0.f };
	for (int i = 0; i < first_points.size(); i++)
	{
		cv::Point2f point = pt_cloud_xyz[first_points[i]];
		first_points_center += point;
	}
	int first_point_cnt = static_cast<int>(first_points.size());
	first_points_center /= first_point_cnt;

	cv::Point2f sec_points_center = { 0.f,0.f };
	for (unsigned int i = 0; i < second_points.size(); i++)
	{
		cv::Point2f point = pt_cloud_xyz[second_points[i]];
		sec_points_center += point;
	}
	int sec_point_cnt = static_cast<int>(second_points.size());
	sec_points_center = sec_points_center / sec_point_cnt;

	// step 2:  sort 2 points by distance to the center of 2 points group
	std::vector <PointIdx2Dist> first_idx_to_dist(first_points.size());
	for (size_t i = 0; i < first_points.size(); i++)
	{
		cv::Point2f point = pt_cloud_xyz[first_points[i]];
		first_idx_to_dist[i].cloud_point_idx = first_points[i];
		//ModuleStruct::Point2f delta_point = point - sec_points_center;
		//float distance = std::fabs(delta_point.x) + std::fabs(delta_point.y);
		float distance = Util_Math::DWComputePointToPointDist2D<cv::Point2f>(point, sec_points_center);
		first_idx_to_dist[i].delta_dist = distance;
	}
	std::sort(first_idx_to_dist.begin(), first_idx_to_dist.end(), std::less<PointIdx2Dist>{});

	std::vector <PointIdx2Dist>sec_idx_to_dist(second_points.size());
	for (size_t i = 0; i < second_points.size(); i++)
	{
		cv::Point2f point = pt_cloud_xyz[second_points[i]];
		sec_idx_to_dist[i].cloud_point_idx = second_points[i];
		//ModuleStruct::Point2f delta_point = point - first_points_center;
		//float distance = std::fabs(delta_point.x) + std::fabs(delta_point.y);
		float distance = Util_Math::DWComputePointToPointDist2D<cv::Point2f>(point, first_points_center);
		sec_idx_to_dist[i].delta_dist = distance;
	}
	std::sort(sec_idx_to_dist.begin(), sec_idx_to_dist.end(), std::less<PointIdx2Dist>{});

	// step 3:   each group use 2 closest points  to  compute center distance, in 2D space only need 2  points to identify close of 2 points group 
	float dist1 = Util_Math::DWComputePointToPointDist2D<cv::Point2f>(pt_cloud_xyz[first_idx_to_dist[0].cloud_point_idx], pt_cloud_xyz[sec_idx_to_dist[0].cloud_point_idx]);
	float min_dist = dist1;
	if ((first_points.size() == 1) || (second_points.size() == 1))
	{
		if (min_dist < distance)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	int first_comp_cnt = 2;
	int sec_comp_cnt = 2;
	if (first_comp_cnt > static_cast<int>(first_points.size()))  first_comp_cnt = static_cast<int>(first_points.size());
	if (sec_comp_cnt > static_cast<int>(second_points.size()))  sec_comp_cnt = static_cast<int>(second_points.size());

	cv::Point2f first_comp_center = { 0.f,0.f };
	for (int i = 0; i < first_comp_cnt; i++)
	{
		first_comp_center += pt_cloud_xyz[first_idx_to_dist[i].cloud_point_idx];
	}
	first_comp_center = first_comp_center / first_comp_cnt;

	cv::Point2f sec_comp_center = { 0.f,0.f };
	for (int i = 0; i < sec_comp_cnt; i++)
	{
		sec_comp_center += pt_cloud_xyz[sec_idx_to_dist[i].cloud_point_idx];
	}
	sec_comp_center = sec_comp_center / sec_comp_cnt;

	float dist = Util_Math::DWComputePointToPointDist2D<cv::Point2f>(first_comp_center, sec_comp_center);
	min_dist = min_dist < dist ? min_dist : dist;
	bool rtn = false;
	if (min_dist < distance) rtn = true;
	return rtn;

}

DW_line_extract2D::DW_line_extract2D()
{
	sys_control_para_.is_cad = false;
	sys_control_para_.scanner_type = LEICA_TYPE_0;

	config_params_.is_point_density_set = false;
	config_params_.point_density = 0.5f;
	config_params_.line_thrshld_section_valid = false;

	line_seg_thrshld_.min_line_dist_2pixel = 0.5f;
	line_seg_thrshld_.min_line_dist_pt2pixel = 0.5f;
	line_seg_thrshld_.max_normal_angle_of_2pixel = 10.f;
	line_seg_thrshld_.max_mse_of_pixel = 0.5f;
	line_seg_thrshld_.min_point_num_of_line = 50;
	line_seg_thrshld_.min_pixel_num_of_line = 3;
	line_seg_thrshld_.max_mse_of_line = 0.5f;
	line_seg_thrshld_.min_dist_of_2line = 0.5f;
	line_seg_thrshld_.max_angle_of_2line = 10.f;
	line_seg_thrshld_.max_high_mse_ratio = 0.f;
	line_seg_thrshld_.min_point_num_of_line_pixel = 3;

	pixel_size_.length_x_of_pixel = 10.f;
	pixel_size_.length_y_of_pixel = 10.f;

	line_seg_thrshld_.min_connected_dist_2pxl = pixel_size_.length_x_of_pixel / 2;

#ifdef SAVE_OUTPUT_FILE_DEBUG
	line2d_debug_params_.dbg_section_valid = false;
	line2d_debug_params_.reserved_test = 0;
	line2d_debug_params_.missing_point_output = false;
	line2d_debug_params_.neighbour_info_debug = false;
	line2d_debug_params_.line_output_debug = false;
	line2d_debug_params_.reserved_test1 = 0;
#endif
	this->init();
}

DW_line_extract2D::~DW_line_extract2D() {

	this->FreeMemroy();
}

bool DW_line_extract2D::init() {

	grid_to_occupied_pxl_idx = NULL;
	pixel_to_line_idx = NULL;
	line_merge_out.lines = NULL;
	line_merge_out.size = 0;
	bad_pxl_merge_array.size = 0;
	bad_pxl_merge_array.bad_pxl_merge_it = NULL;
	occupied_idx_to_bad_pxl = NULL;
	same_line_group_array.line_size = 0;
	same_line_group_array.same_line_group_idx = NULL;
	same_line_group_array.group_number = 0;
	return true;
}



void DW_line_extract2D::FreeMemroy() 
{
	// release memory in pxl_occupied_array
	if (pxl_occupied_array.pxl_item != NULL) 
	{

		for (unsigned int i = 0; i < pxl_occupied_array.size; i++) {

			// free memory of storing all point indexes in each occupied pixels
			if (pxl_occupied_array.pxl_item[i].points.point_idx != NULL) {

				delete[] pxl_occupied_array.pxl_item[i].points.point_idx;
				pxl_occupied_array.pxl_item[i].points.point_idx = NULL;
				pxl_occupied_array.pxl_item[i].points.size = 0;
			}
		}

		delete[] pxl_occupied_array.pxl_item;
		pxl_occupied_array.pxl_item = NULL;
		pxl_occupied_array.size = 0;
	}
	
	if (grid_to_occupied_pxl_idx != NULL)
	{
		delete[] grid_to_occupied_pxl_idx;
		grid_to_occupied_pxl_idx = NULL;
	}


	if (pixel_to_line_idx != NULL)
	{
		delete[] pixel_to_line_idx;
		pixel_to_line_idx = NULL;
	}

	if (line_merge_out.lines != NULL)
	{

		for (unsigned int i = 0; i < line_merge_out.size; i++) {

			if (line_merge_out.lines[i].points.point_idx != NULL) {

				delete[] line_merge_out.lines[i].points.point_idx;
				line_merge_out.lines[i].points.point_idx = NULL;
				line_merge_out.lines[i].points.size = 0;
			}

			if (line_merge_out.lines[i].pixels.pxl_idx != NULL) {

				delete[] line_merge_out.lines[i].pixels.pxl_idx;
				line_merge_out.lines[i].pixels.pxl_idx = NULL;
				line_merge_out.lines[i].pixels.size = 0;
			}
		}
		delete[] line_merge_out.lines;
		line_merge_out.lines = NULL;
		line_merge_out.size = 0;
	}

	if (bad_pxl_merge_array.bad_pxl_merge_it != NULL)
	{
		for (unsigned int i = 0; i < bad_pxl_merge_array.size; i++) {

			if (bad_pxl_merge_array.bad_pxl_merge_it[i].closest_line_idx!= NULL) {

				delete[] bad_pxl_merge_array.bad_pxl_merge_it[i].closest_line_idx;
				bad_pxl_merge_array.bad_pxl_merge_it[i].closest_line_idx = NULL;
			}

			if (bad_pxl_merge_array.bad_pxl_merge_it[i].point_merged_flag != NULL) {

				delete[] bad_pxl_merge_array.bad_pxl_merge_it[i].point_merged_flag;
				bad_pxl_merge_array.bad_pxl_merge_it[i].point_merged_flag = NULL;
			}

			if (bad_pxl_merge_array.bad_pxl_merge_it[i].is_of_line_list != NULL) {

				delete[] bad_pxl_merge_array.bad_pxl_merge_it[i].is_of_line_list;
				bad_pxl_merge_array.bad_pxl_merge_it[i].is_of_line_list = NULL;
			}

		}

		delete[] bad_pxl_merge_array.bad_pxl_merge_it;
		bad_pxl_merge_array.bad_pxl_merge_it = NULL;
		bad_pxl_merge_array.size = 0;
	}

	if (occupied_idx_to_bad_pxl != NULL)
	{
		delete[] occupied_idx_to_bad_pxl;
		occupied_idx_to_bad_pxl = NULL;
	}

	if (same_line_group_array.same_line_group_idx != NULL)
	{
		delete[] same_line_group_array.same_line_group_idx;
		same_line_group_array.same_line_group_idx = NULL;
	}

	
	pt_points.clear();
	pt_points.shrink_to_fit();
}

bool DW_line_extract2D::GetMinMaxXY(const std::vector<Point2f> input_data, Point2f &max_p, Point2f &min_p)
{
	if (input_data.size() == 0)
	{
		log_error("empty input");
		return false;
	}

	min_p.x = min_p.y = std::numeric_limits<float>::infinity();
	max_p.x = max_p.y = -std::numeric_limits<float>::infinity();

	for (unsigned int i = 0; i < input_data.size(); i++) {
		Point2f point = input_data[i];
		min_p.x = (min_p.x > point.x) ? point.x : min_p.x;
		max_p.x = (max_p.x < point.x) ? point.x : max_p.x;
		min_p.y = (min_p.y > point.y) ? point.y : min_p.y;
		max_p.y = (max_p.y < point.y) ? point.y : max_p.y;
	}
	return true;
}

bool DW_line_extract2D::GetMinMaxXPoint(const std::vector<Point2f> input_data, const PointInPixelArray point_array, Point2f &max_p, Point2f &min_p)
{
	if ((input_data.size() == 0)||(point_array.size==0))
	{
		log_error("empty input");
		return false;
	}

	min_p.x = std::numeric_limits<float>::infinity();
	max_p.x = -std::numeric_limits<float>::infinity();

	for (unsigned int i = 0; i < point_array.size; i++) {
		unsigned int point_idx = point_array.point_idx[i];

		Point2f point = input_data[point_idx];
		min_p = (min_p.x > point.x) ? point : min_p;
		max_p = (max_p.x < point.x) ? point : max_p;
	}
	return true;
}


bool DW_line_extract2D::GetMinMaxYPoint(const std::vector<Point2f> input_data, const PointInPixelArray point_array, Point2f &max_p, Point2f &min_p)
{
	if ((input_data.size() == 0) || (point_array.size == 0))
	{
		log_error("empty input");
		return false;
	}

	min_p.y = std::numeric_limits<float>::infinity();
	max_p.y = -std::numeric_limits<float>::infinity();

	for (unsigned int i = 0; i < point_array.size; i++) {
		unsigned int point_idx = point_array.point_idx[i];

		Point2f point = input_data[point_idx];
		min_p = (min_p.y > point.y) ? point : min_p;
		max_p = (max_p.y < point.y) ? point : max_p;
	}
	return true;
}
bool DW_line_extract2D::GetMinMaxYPoint(const std::vector<Point2f> input_data, const Vector<unsigned int> point_array, Point2f& max_p, Point2f& min_p)
{
	if ((input_data.size() == 0) || (point_array.size() == 0))
	{
		log_error("empty input");
		return false;
	}

	min_p.y = std::numeric_limits<float>::infinity();
	max_p.y = -std::numeric_limits<float>::infinity();

	for (unsigned int i = 0; i < point_array.size(); i++) {
		unsigned int point_idx = point_array[i];

		Point2f point = input_data[point_idx];
		min_p = (min_p.y > point.y) ? point : min_p;
		max_p = (max_p.y < point.y) ? point : max_p;
	}
	return true;
}

bool DW_line_extract2D::GetStartEndOfLines(LineSegItem *line_seg)
{
	Point2f max_p, min_p;
	if (std::fabs(line_seg->line_direction.x) < std::numeric_limits<float>::epsilon()
		&& std::fabs(line_seg->line_direction.y) < std::numeric_limits<float>::epsilon()) 
		return false;
	
	// here 0.7 show the angle with x axis is < about 45 , no need to be very precise
	if (std::fabs(line_seg->line_direction.x) < 0.7)
	{
		GetMinMaxYPoint(pt_points, line_seg->points, max_p, min_p);
	}
	else
	{
		GetMinMaxXPoint(pt_points, line_seg->points, max_p, min_p);
	}

	GetFootOfPerpendicular(min_p, line_seg->line_center, line_seg->line_direction, line_seg->line_seg_start);
	GetFootOfPerpendicular(max_p, line_seg->line_center, line_seg->line_direction, line_seg->line_seg_end);
	return true;
}



bool DW_line_extract2D::GetTotalNumOfPixelGrid( const Point2f max_p, const Point2f min_p, 
												const PixelSize pixel_param, PixelDimension& pixel_dim)
{
	if ((max_p.x - min_p.x) != 0 && std::remainder((max_p.x - min_p.x), pixel_param.length_x_of_pixel) == 0)
		pixel_dim.cols_of_pixel = (unsigned int)(std::floor((max_p.x - min_p.x) * inverse_length_x_of_pixel));
	else
		pixel_dim.cols_of_pixel = (unsigned int)(std::floor((max_p.x - min_p.x) * inverse_length_x_of_pixel) + 1);


	if ((max_p.y - min_p.y) != 0 && std::remainder((max_p.y - min_p.y), pixel_param.length_y_of_pixel) == 0)
		pixel_dim.rows_of_pixel = (unsigned int)(std::floor((max_p.y - min_p.y) * inverse_length_y_of_pixel));
	else
		pixel_dim.rows_of_pixel = (unsigned int)(std::floor((max_p.y - min_p.y) * inverse_length_y_of_pixel) + 1);

	total_num_of_pxl_grid = pixel_dim.cols_of_pixel * pixel_dim.rows_of_pixel;
	return true;
}

// code refence to https://blog.csdn.net/fengbingchun/article/details/73558370
	
bool DW_line_extract2D::Compute2d(unsigned int point_cnt, SumforCovariance2d sums, Point2f &line_direction, Point2f &center, float &line_mse)
{
	if (point_cnt <= 0) return false;

	double point_num_inverse = (double)1.0 / point_cnt;

	//line center
	center.x = (float)(sums.sum_x*point_num_inverse);
	center.y = (float)(sums.sum_y*point_num_inverse);

	//calculate covariance matrix
	double cov_mat_element[3] = { (sums.sum_xx - sums.sum_x*center.x),(sums.sum_xy - sums.sum_x*center.y),(sums.sum_yy - sums.sum_y*center.y)};


	double cov_mat_arr[2][2] = { { cov_mat_element[0],cov_mat_element[1]},
	{ cov_mat_element[1], cov_mat_element[2]} };

	cv::Mat cov_mat(2, 2, CV_64F, cov_mat_arr);

	cov_mat *= point_num_inverse;

	cv::Mat eig_val_mat, eig_vec_mat;
	cv::eigen(cov_mat, eig_val_mat, eig_vec_mat);

	line_mse  = static_cast<float>(eig_val_mat.at<IntermediateType>(0));

	line_direction.x = static_cast<float>(eig_vec_mat.at<IntermediateType>(0, 0));
	line_direction.y = static_cast<float>(eig_vec_mat.at<IntermediateType>(0, 1));
	return true;
}

bool DW_line_extract2D::Compute2dS(
	unsigned int point_cnt, 
	SumforCovariance2d sums, 
	Point2f &line_direction, 
	Point2f &center, 
	float &line_mse)
{
	//if (point_cnt <= 0) return false;

	double point_num_inverse = (double)1.0 / point_cnt;

	//line center
	center.x = (float)(sums.sum_x*point_num_inverse);
	center.y = (float)(sums.sum_y*point_num_inverse);
	float center_z = 0.f;
	double sum_xz, sum_yz, sum_zz,sum_z;
	sum_xz = sum_yz = sum_zz = sum_z = 0;

	//calculate covariance matrix
	//double cov_mat_element[6] = { (sums.sum_xx - sums.sum_x*center.x),(sums.sum_xy - sums.sum_x*center.y),(sums.sum_yy - sums.sum_y*center.y)};
	double cov_mat_element[6] = { sums.sum_xx - sums.sum_x*center.x,
		                          sums.sum_xy - sums.sum_x*center.y,
								  sum_xz - sums.sum_x*center_z,
		                          sums.sum_yy - sums.sum_y*center.y,
								  sum_yz - sums.sum_y*center_z,
		                          sum_zz - sum_z*center_z };

	/*double cov_mat_arr[2][2] = { { cov_mat_element[0],cov_mat_element[1]},
	{ cov_mat_element[1], cov_mat_element[2]} };*/

	double cov_mat_arr[3][3] = { 
		{ cov_mat_element[0],cov_mat_element[1],cov_mat_element[2] },
		{ cov_mat_element[1], cov_mat_element[3], cov_mat_element[4] },
		{ cov_mat_element[2], cov_mat_element[4], cov_mat_element[5] } };


	/*cv::Mat cov_mat(2, 2, CV_64F, cov_mat_arr);*/
	cv::Mat cov_mat(3, 3, CV_64F, cov_mat_arr);

	cov_mat *= point_num_inverse;

	cv::Mat eig_val_mat, eig_vec_mat;
	cv::eigen(cov_mat, eig_val_mat, eig_vec_mat);

	line_mse = static_cast<float>(eig_val_mat.at<IntermediateType>(0));

	line_direction.x = static_cast<float>(eig_vec_mat.at<IntermediateType>(0, 0));
	line_direction.y = static_cast<float>(eig_vec_mat.at<IntermediateType>(0, 1));
	return true;
}

bool DW_line_extract2D::ConvertPointToPixelID(
	const Point2f min_p, const Point2f point, const PixelSize pixel_size, 
	const PixelDimension pixel_dim, unsigned long long &grid_id)
{

	unsigned int col_idx = static_cast<unsigned int>(std::floor((point.x - min_p.x) / pixel_size.length_x_of_pixel));
	unsigned int row_idx = static_cast<unsigned int>(std::floor((point.y - min_p.y) / pixel_size.length_y_of_pixel));
	if (col_idx == pixel_dim.cols_of_pixel)
		col_idx--;
	if (row_idx == pixel_dim.rows_of_pixel)
		row_idx--;
	grid_id = row_idx * pixel_dim.cols_of_pixel + col_idx;
	return true;
}


////============================================================================
bool DW_line_extract2D::CreateOccupiedPixels()
{
	unsigned int input_point_size = static_cast<unsigned int>(pt_points.size());

	PointGrid2PointIdx* point_to_grid_list = new PointGrid2PointIdx[input_point_size];

	for (unsigned int i = 0; i < input_point_size; i++)
	{
		point_to_grid_list[i].cloud_point_index = i;
		ConvertPointToPixelID(min_p,  pt_points[i],
							  pixel_size_,  pixel_dim, 
							  point_to_grid_list[i].grid_idx);
	}
	std::sort(point_to_grid_list, point_to_grid_list + input_point_size, std::less<PointGrid2PointIdx>());
	
	unsigned int occupied_cnt = 0;  // total occupied number of pixels
	unsigned int i = 0;

	while (i < input_point_size)//==========
	{
		unsigned int j = i + 1;
		while ((j < input_point_size) && 
			   (point_to_grid_list[i].grid_idx == point_to_grid_list[j].grid_idx))
		{
			++j;/// belong to one grid
		}
		occupied_cnt++; ///total grid occupied
		i = j;
	}

	pxl_occupied_array.size = occupied_cnt;
	pxl_occupied_array.pxl_item = new PixelGridItem[occupied_cnt];

	// initialized pixels item
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(pxl_occupied_array.size);i++)
	{
		PixelGridItem *pxl_it = &pxl_occupied_array.pxl_item[i];
		pxl_it->is_good_pixel = false;
		pxl_it->in_line_pixel = false;
		pxl_it->grid_idx = std::numeric_limits<unsigned int>::max();
		pxl_it->pxl_mse = std::numeric_limits<float>::infinity();
		pxl_it->pxl_eigen_mse = std::numeric_limits<float>::infinity();
		pxl_it->pxl_weighted_mse = std::numeric_limits<float>::infinity();
		pxl_it->high_weighted_mse_cnt = 0;

		pxl_it->pxl_high_mse_ratio = 1.0f;
		pxl_it->center = {0.f,0.f};
		pxl_it->line_direction = { 0.f,0.f };
		pxl_it->weighted_line_direction = { 0.f,0.f };
		pxl_it->high_weighted_mse_ratio = 1.0f;


		SumsClear(&pxl_it->sums);
		pxl_it->points.size = 0;
		pxl_it->points.point_idx = NULL;
		pxl_it->is_being_merged = false;
		pxl_it->is_overall_merged = false;
		pxl_it->good_neighbor_cnt = 0;
		pxl_it->bad_neighbor_cnt = 0;
		pxl_it->parent_pixel_idx[0] = pxl_it->parent_pixel_idx[1] = i;

		for (unsigned int j = 0; j < 8; j++)
		{
			PixelNeighborItem *pxl_neighbor_it = &pxl_it->neighbors[j];
			pxl_neighbor_it->is_occupied = false;
			pxl_neighbor_it->pixel_idx = std::numeric_limits<unsigned int>::max();
			pxl_neighbor_it->direcion_diff = std::numeric_limits<float>::infinity();
			pxl_neighbor_it->line_dist = std::numeric_limits<float>::infinity();
			pxl_neighbor_it->neighbor_flag = false;
			pxl_neighbor_it->in_line_flag = false;
			//pxl_neighbor_it->high_weighted_mse_cnt = 0;
			pxl_neighbor_it->is_connected = false;
		}
	}

	grid_to_occupied_pxl_idx = new unsigned int[total_num_of_pxl_grid];
	for (unsigned int i = 0; i < total_num_of_pxl_grid; i++)
	{
		grid_to_occupied_pxl_idx[i] = std::numeric_limits<unsigned int>::max();
	}

	i = 0;
	occupied_cnt = 0;
	while (i < input_point_size)//==========
	{
		unsigned int j = i + 1;
		while ((j < input_point_size) && (point_to_grid_list[i].grid_idx == point_to_grid_list[j].grid_idx))
		{
			++j;
		}
		
		pxl_occupied_array.pxl_item[occupied_cnt].points.size = j - i;
		pxl_occupied_array.pxl_item[occupied_cnt].points.point_idx = new unsigned int[j - i];
		grid_to_occupied_pxl_idx[point_to_grid_list[i].grid_idx] = occupied_cnt;
		occupied_cnt++;
		i = j;
	}

	unsigned int pxl_index = 0;
	for (unsigned int i = 0; i < input_point_size;)//==========
	{
		PixelGridItem *pxl_it = &pxl_occupied_array.pxl_item[pxl_index];
		unsigned int first_point_idx = point_to_grid_list[i].cloud_point_index;
		Point2f first_point = pt_points[first_point_idx];
		unsigned int point_cnt_in_pxl = 0;
		pxl_it->grid_idx = static_cast<unsigned int>(point_to_grid_list[i].grid_idx);
		pxl_it->points.point_idx[point_cnt_in_pxl] = first_point_idx;
		PushPoint(&pxl_it->sums, first_point);
		unsigned int j = i + 1;
		point_cnt_in_pxl++;
		while ((j < input_point_size)
			    && (point_to_grid_list[i].grid_idx == point_to_grid_list[j].grid_idx))
		{
			unsigned int current_point_idx = point_to_grid_list[j].cloud_point_index;
			Point2f current_point = pt_points[current_point_idx];
			PushPoint(&pxl_it->sums, current_point);
			pxl_it->points.point_idx[point_cnt_in_pxl] = current_point_idx;
			point_cnt_in_pxl++;
			j++;
		}
		if (point_cnt_in_pxl != pxl_it->points.size) 
			log_debug("pixel %d point_cnt_in_pxl = %d is not equal to point size =%d ", pxl_index,point_cnt_in_pxl, pxl_it->points.size);
		i = j;
		pxl_index++;
	}
	delete[] point_to_grid_list;	
	return true;
}

// 
bool DW_line_extract2D::GetOccupiedPixelsNeighbor()
{
	float max_line_direction_diff = static_cast<float>(std::cos(line_seg_thrshld_.max_normal_angle_of_2pixel * M_PI / 180.0));//20
	float min_dist_of_2pixel = line_seg_thrshld_.min_line_dist_2pixel;//10mm
	float max_mse_pixel = line_seg_thrshld_.max_mse_of_pixel; //20mm
	//printf("max_normal_angle_of_2pixel=%f  max_mse_pixel=%f  min_dist_of_2pixel=%f\n", line_seg_thrshld_.max_normal_angle_of_2pixel, max_mse_pixel, min_dist_of_2pixel);

	//#pragma omp parallel for
	for (unsigned int i = 0; i < pxl_occupied_array.size; i++)
	{
		PixelGridItem *pxl_it = &pxl_occupied_array.pxl_item[i];
		unsigned int point_cnt = pxl_it->points.size;		
		Compute2dS(point_cnt, pxl_it->sums, pxl_it->line_direction, pxl_it->center, pxl_it->pxl_eigen_mse);
		ComputePixelMse(pxl_it);
		if ((pxl_it->pxl_mse < max_mse_pixel) && 
			(pxl_it->points.size > line_seg_thrshld_.min_point_num_of_line_pixel)&& 
			(pxl_it->pxl_high_mse_ratio < line_seg_thrshld_.max_high_mse_ratio))
		{
			pxl_it->is_good_pixel = true;
		}
	}

	// check if neighbour is occupied and get pixel index if true
	//#pragma omp parallel for
	for (unsigned int i = 0; i < pxl_occupied_array.size; i++)
	{
		PixelGridItem *pxl_it = &pxl_occupied_array.pxl_item[i];
		unsigned int point_index = pxl_it->points.point_idx[0];
		Point2f point =  pt_points[point_index];
		int col_idx, row_idx;

		ConvertPixelIDTo2dID(point, row_idx, col_idx);
		for (int k = 0; k < 8; k++)
		{
			if (row_idx + movement_y[k] >= (int)pixel_dim.rows_of_pixel || row_idx + movement_y[k] < 0)
				continue;
			if (col_idx + movement_x[k] >= (int)pixel_dim.cols_of_pixel || col_idx + movement_x[k] < 0)
				continue;

			unsigned int neighbor_pixel_id = static_cast<unsigned int>(row_idx + movement_y[k]) * pixel_dim.cols_of_pixel +
				                                                      (col_idx + movement_x[k]);

			if (neighbor_pixel_id < 0 || neighbor_pixel_id >= total_num_of_pxl_grid) 
				continue;

			//check if this pixel is initialized
			if (grid_to_occupied_pxl_idx[neighbor_pixel_id] == std::numeric_limits<unsigned int>::max()) 
				continue;

			/* grid_to_occupied_pixel_idx is the output of NormalEstimationInterface */
			unsigned int neighbour_pixel_index = grid_to_occupied_pxl_idx[neighbor_pixel_id];
			PixelNeighborItem *pxl_neighbor_it = &pxl_it->neighbors[k];
			pxl_neighbor_it->is_occupied = true;
			pxl_neighbor_it->pixel_idx = neighbour_pixel_index;
		}       
	}
	

	for (unsigned int i = 0; i < pxl_occupied_array.size; i++)
	{
		// check pixel neighbor connected by distance
		//float max_connected_distance = line_seg_thrshld_.min_line_dist_pt2pixel*2;
		float max_connected_distance = line_seg_thrshld_.min_connected_dist_2pxl;
		PixelGridItem *pxl_it = &pxl_occupied_array.pxl_item[i];
		for (int k = 0; k < 8; k++)
		{
			PixelNeighborItem *pxl_neighbor_it = &pxl_it->neighbors[k];
			if (!pxl_neighbor_it->is_occupied) 
				continue;
			PixelGridItem *neighbor_pxl_it = &pxl_occupied_array.pxl_item[pxl_neighbor_it->pixel_idx];
			//bool debug_condition = (i == 26);// && (pxl_neighbor_it->pixel_idx == 74);
			//debug_condition = debug_condition || (i == 75) && (pxl_neighbor_it->pixel_idx == 74);
			//debug_condition = debug_condition || (i == 62) && (pxl_neighbor_it->pixel_idx == 74);
			pxl_neighbor_it->is_connected = Identify2PixelCLoseByDistance(pt_points, i, 
				                                                          pxl_neighbor_it->pixel_idx, 
				                                                          max_connected_distance);
			//if(debug_condition) 
			  //log_debug("pxl =%d neighbor %d is connected = %d threshold =%f", i, pxl_neighbor_it->pixel_idx, pxl_neighbor_it->is_connected, max_connected_distance);
		}
	}


	for (unsigned int i = 0; i < pxl_occupied_array.size; i++)
	{
		// get pixel weighted elements
		ComputePixeWeightedlElements(i);///T
	}

	// get in line flag
//#pragma omp parallel for
	for (unsigned int i = 0; i < pxl_occupied_array.size; i++)
	{
		PixelGridItem *pxl_it = &pxl_occupied_array.pxl_item[i];
		if (!pxl_it->in_line_pixel) continue;

		for (int k = 0; k < 8; k++)
		{
			PixelNeighborItem *pxl_neighbor_it = &pxl_it->neighbors[k];
			if (!pxl_neighbor_it->is_occupied) continue;
			PixelGridItem *neighbor_pxl_it = &pxl_occupied_array.pxl_item[pxl_neighbor_it->pixel_idx];
			if (!neighbor_pxl_it->in_line_pixel) continue;
			float direction_diff = std::fabsf(pxl_it->weighted_line_direction.dot(neighbor_pxl_it->weighted_line_direction));
			if (direction_diff <= max_line_direction_diff) continue;
			float line_dist = ComputePointToLineDist(neighbor_pxl_it->weighted_center, pxl_it->weighted_center, pxl_it->weighted_line_direction);
			if (line_dist < min_dist_of_2pixel)
			{
				pxl_neighbor_it->in_line_flag = true;///
			}
		}
	}

	// get neighbor flag 
//#pragma omp parallel for
	for (unsigned int i = 0; i < pxl_occupied_array.size; i++)
	{
		PixelGridItem *pxl_it = &pxl_occupied_array.pxl_item[i];
		if (!pxl_it->is_good_pixel) continue;

		for (int k = 0; k < 8; k++)
		{
			PixelNeighborItem *pxl_neighbor_it = &pxl_it->neighbors[k];
			if (!pxl_neighbor_it->is_connected) continue;
			PixelGridItem *neighbor_pxl_it = &pxl_occupied_array.pxl_item[pxl_neighbor_it->pixel_idx];
			if (!neighbor_pxl_it->is_good_pixel) continue;
			float direcion_diff = std::fabsf(pxl_it->line_direction.dot(neighbor_pxl_it->line_direction));
			if (direcion_diff <= max_line_direction_diff) continue;
			float line_dist = ComputePointToLineDist(neighbor_pxl_it->center, pxl_it->center, pxl_it->line_direction);
			if (line_dist < min_dist_of_2pixel)
			{
				pxl_neighbor_it->neighbor_flag = true;///
				//if ((pxl_it->in_line_pixel)&&(pxl_neighbor_it->high_weighted_mse_cnt == 0))
				//{
				//	neighbor_pxl_it->is_overall_merged = true;
				//}
			}
		}
	}

	// get neighbor count
	for (unsigned int i = 0; i < pxl_occupied_array.size; i++)
	{
		PixelGridItem *pxl_it = &pxl_occupied_array.pxl_item[i];
		if (!pxl_it->in_line_pixel) continue;
		for (int k = 0; k < 8; k++)
		{
			PixelNeighborItem *pxl_neighbor_it = &pxl_it->neighbors[k];
			if (!pxl_neighbor_it->is_occupied) continue;
			PixelGridItem *neighbor_pxl_it = &pxl_occupied_array.pxl_item[pxl_neighbor_it->pixel_idx];
			//if (!neighbor_pxl_it->is_good_pixel) continue;
			if (pxl_neighbor_it->in_line_flag)
			{
				pxl_it->good_neighbor_cnt++;///
				//pxl_it->is_overall_merged = true;
			}
			else
			{
				pxl_it->bad_neighbor_cnt++;///
			}
		}
	}

	for (unsigned int i = 0; i < pxl_occupied_array.size; i++)
	{
		PixelGridItem* pxl_it = &pxl_occupied_array.pxl_item[i];
		if (!pxl_it->in_line_pixel) continue;
		if ((pxl_it->good_neighbor_cnt >= 1) && (pxl_it->high_weighted_mse_cnt == 0))
		{
			pxl_it->is_overall_merged = true;
		}
	}

	return true;
}

//for pixel neighbor info file output 
void DW_line_extract2D::GetpixelNeigbourDebugInfo(PxlNeighborDbgType output_type)
{
	std::string file_name;
	std::string folder_path = output_path;

	switch (output_type)
	{
	case GOOD_PIXEL_DEBUG:
		file_name = folder_path + "good_pixel_neighbour.txt";
		break;
	case BAD_PIXEL_DEBUG:
		file_name = folder_path + "bad_pixel_neighbour.txt";
		break;
	case ALL_PIXEL_DEBUG:
		file_name = folder_path + "pixel_neighbour.txt";
		break;
	default:
		file_name = folder_path + "pixel_neighbour.txt";
		break;
	}
		
	std::ofstream pixel_neigbour(file_name);
	float max_line_direction_diff = static_cast<float>(std::cos(line_seg_thrshld_.max_normal_angle_of_2pixel * M_PI / 180.0));
	float min_dist_of_2pixel = line_seg_thrshld_.min_line_dist_2pixel;

	for (unsigned int i = 0; i < pxl_occupied_array.size; i++)
	{
		PixelGridItem *pxl_it = &pxl_occupied_array.pxl_item[i];
		//if (!pxl_it->is_good_pixel) continue;

		for (int k = 0; k < 8; k++)
		{
			PixelNeighborItem *pxl_neighbor_it = &pxl_it->neighbors[k];
			if (!pxl_neighbor_it->is_occupied) continue;
			PixelGridItem *neighbor_pxl_it = &pxl_occupied_array.pxl_item[pxl_neighbor_it->pixel_idx];
			//if (!neighbor_pxl_it->is_good_pixel) continue;
			bool condition = false;// i == 4;
			pxl_neighbor_it->direcion_diff = std::fabsf(pxl_it->weighted_line_direction.dot(neighbor_pxl_it->weighted_line_direction));
			pxl_neighbor_it->line_dist = ComputePointToLineDist(neighbor_pxl_it->weighted_center, pxl_it->weighted_center, pxl_it->weighted_line_direction);
			if (condition) log_debug("neighbor pxl %d pxl_it->center = [%f,%f]", pxl_neighbor_it->pixel_idx,neighbor_pxl_it->weighted_center.x, neighbor_pxl_it->weighted_center.y);
			if (condition) log_debug("i%d pxl_it->is_being_merged = %d", i, pxl_it->is_being_merged);
			if (condition) log_debug("pxl_neighbor_it->direcion_diff =%f dist =%f", pxl_neighbor_it->direcion_diff, pxl_neighbor_it->line_dist);

		}
	}

	for (unsigned int i = 0; i < pxl_occupied_array.size; i++)
	{
		PixelGridItem *pxl_it = &pxl_occupied_array.pxl_item[i];

		bool condition = false;

		//if (pxl_pixel->pxl_mse >= pxl_fit_thresholds->THRESHOLD_MAX_MSE_OF_pixel) continue;

		switch (output_type)
		{
		case GOOD_PIXEL_DEBUG:
			condition = !(pxl_it->is_good_pixel);
			break;

		case BAD_PIXEL_DEBUG:
			condition = (pxl_it->is_good_pixel);
			break;

		case ALL_PIXEL_DEBUG:
			condition = false;
			break;

		default:
			condition = false;
			break;
		}

		if (condition) continue;


		int neighbor_flag_count = 0;
		int badneighbor_flag_count = 0;
		int good_neigbour_idx[8], bad_neigbour_idx[8];
		for (int k = 0; k < 8; k++)
		{
			good_neigbour_idx[k] = INVALID_LINE_IDX;
			bad_neigbour_idx[k] = -1;
		}

		for (int k = 0; k < 8; k++)
		{
			PixelNeighborItem *pxl_neighbor_it = &pxl_it->neighbors[k];
			if (!pxl_neighbor_it->is_occupied) continue;

			if (pxl_neighbor_it->in_line_flag)
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

		unsigned int *sorted_good_neighbor_pixel_idx = NULL;
		unsigned int *sorted_bad_neighbor_pixel_idx = NULL;
		// Sort good neighbor according to neighbor pixel index in ascending order
		if (neighbor_flag_count != 0)
		{
			sorted_good_neighbor_pixel_idx = new unsigned int[neighbor_flag_count];
			for (int k = 0; k < neighbor_flag_count; k++)
			{
				PixelNeighborItem *pxl_neighbor_it = &pxl_it->neighbors[good_neigbour_idx[k]];
				sorted_good_neighbor_pixel_idx[k] = pxl_neighbor_it->pixel_idx;
			}
			std::sort(sorted_good_neighbor_pixel_idx, sorted_good_neighbor_pixel_idx + neighbor_flag_count);
		}

		// Sort bad neighbor according to neighbor pixel index in ascending order
		if (badneighbor_flag_count != 0)
		{
			sorted_bad_neighbor_pixel_idx = new unsigned int[badneighbor_flag_count];
			for (int k = 0; k < badneighbor_flag_count; k++)
			{
				PixelNeighborItem *pxl_neighbor_it = &pxl_it->neighbors[bad_neigbour_idx[k]];
				sorted_bad_neighbor_pixel_idx[k] = pxl_neighbor_it->pixel_idx;
			}
			std::sort(sorted_bad_neighbor_pixel_idx, sorted_bad_neighbor_pixel_idx + badneighbor_flag_count);
		}

		// Update good_neigbour_idx[] and bad_neigbour_idx[] according to neighbor pixel index in ascending order
		for (int k = 0; k < 8; k++)
		{
			PixelNeighborItem *pxl_neighbor_it = &pxl_it->neighbors[k];

			if (!pxl_neighbor_it->is_occupied) continue;

			if (pxl_neighbor_it->in_line_flag)
			{
				for (int j = 0; j < neighbor_flag_count; j++)
				{
					if (pxl_neighbor_it->pixel_idx == sorted_good_neighbor_pixel_idx[j])
					{
						good_neigbour_idx[j] = k;
					}
				}
			}
			else
			{
				for (int j = 0; j < badneighbor_flag_count; j++)
				{
					if (pxl_neighbor_it->pixel_idx == sorted_bad_neighbor_pixel_idx[j])
					{
						bad_neigbour_idx[j] = k;
					}
				}
			}
		}

		pixel_neigbour << "pixel " << i << std::setw(18) << "\tgood neighbours:" << std::setw(8) << neighbor_flag_count << "\t mergeflag:" << std::setw(8) << pxl_it->is_being_merged \
			<< "\t\t grid_idx:" << std::setw(8) << pxl_it->grid_idx \
			<< "\t mse:" << std::setw(8) << pxl_it->pxl_mse <<"\t is_good_pixel:"<< std::setw(8) <<pxl_it->is_good_pixel\
			<< "\t is_overall_merged:" << std::setw(8) << pxl_it->is_overall_merged \
			<< "\t high_weighted_mse_cnt:" << std::setw(8) << pxl_it->high_weighted_mse_cnt <<"\t\n";

		if (neighbor_flag_count != 0)
		{
			pixel_neigbour << "    idx  " << std::setw(12) << "   direction_diff   " << std::setw(16) << "   line_dist   " << "   pxl_mse   " << "  num_of_point  " << "mergeflag" \
				<< "       grid_idx " << "  high_mse_ratio " << " is_overall_merged " << " high_weighted_mse_cnt " << " pxl_weighted_mse " << " is_connected " \
				<< " high_weighted_mse_ratio " << "\n";

			for (int k = 0; k < neighbor_flag_count; k++)
			{
				PixelNeighborItem *pxl_neighbor_it = &pxl_it->neighbors[good_neigbour_idx[k]];
				PixelGridItem *neighbor_pxl_it = &pxl_occupied_array.pxl_item[pxl_neighbor_it->pixel_idx];
				float direction_diff = static_cast<float>(std::acos(pxl_neighbor_it->direcion_diff)*180/M_PI);
				float line_dist = pxl_neighbor_it->line_dist;
				float pxl_mse = neighbor_pxl_it->pxl_mse;
				int num_of_point = neighbor_pxl_it->points.size;
				bool mergeflag = neighbor_pxl_it->is_being_merged;
				int grid_idx = neighbor_pxl_it->grid_idx;
				float high_mse_ratio = neighbor_pxl_it->pxl_high_mse_ratio;
				float eigen_mse = neighbor_pxl_it->pxl_eigen_mse;
				bool is_overall_merged = neighbor_pxl_it->is_overall_merged;
				int high_weighted_mse_cnt = neighbor_pxl_it->high_weighted_mse_cnt;
				float pxl_weighted_mse = neighbor_pxl_it->pxl_weighted_mse;
				bool is_connected = pxl_neighbor_it->is_connected;
				float  high_weighted_mse_ratio = neighbor_pxl_it->high_weighted_mse_ratio;
				pixel_neigbour << ' ' << std::setw(6) << pxl_neighbor_it->pixel_idx << std::setw(12) << direction_diff << std::setw(20) << line_dist \
					<< std::setw(18) << pxl_mse << std::setw(10) << num_of_point << std::setw(14) << mergeflag \
					<< std::setw(16) << grid_idx << std::setw(14) << high_mse_ratio << std::setw(18) << is_overall_merged \
					<< std::setw(18) << high_weighted_mse_cnt << std::setw(18) << pxl_weighted_mse\
					<< std::setw(16) << is_connected << std::setw(16) << high_weighted_mse_ratio << "\t\n";
			}

		}
		pixel_neigbour << "pixel " << i << std::setw(18) << "\tbad neighbours:" << std::setw(8) << badneighbor_flag_count << "\t mergeflag:" << std::setw(8) << pxl_it->is_being_merged \
			<< "\t\t grid_idx:" << std::setw(8) << pxl_it->grid_idx \
			<< "\t mse:" << std::setw(8) << pxl_it->pxl_mse << "\t is_good_pixel:" << std::setw(8) << pxl_it->is_good_pixel\
			<< "\t is_overall_merged:" << std::setw(8) << pxl_it->is_overall_merged\
			<< "\t high_weighted_mse_cnt:" << std::setw(8) << pxl_it->high_weighted_mse_cnt << "\t\n";
		if (badneighbor_flag_count != 0)
		{
			pixel_neigbour << "    idx  " << std::setw(12) << "   direction_diff   " << std::setw(16) << "   line_dist   " << "   pxl_mse   " << "  num_of_point  " << "mergeflag" \
				<< "       grid_idx " << "  high_mse_ratio " << " is_overall_merged " << " high_weighted_mse_cnt " << " pxl_weighted_mse " << " is_connected " \
				<<" high_weighted_mse_ratio "<< "\n";

			for (int k = 0; k < badneighbor_flag_count; k++)
			{
				PixelNeighborItem *pxl_neighbor_it = &pxl_it->neighbors[bad_neigbour_idx[k]];
				PixelGridItem *neighbor_pxl_it = &pxl_occupied_array.pxl_item[pxl_neighbor_it->pixel_idx];
				float direction_diff = static_cast<float>(std::acos(pxl_neighbor_it->direcion_diff) * 180 / M_PI);
				float line_dist = pxl_neighbor_it->line_dist;
				float pxl_mse = neighbor_pxl_it->pxl_mse;
				int num_of_point = neighbor_pxl_it->points.size;
				bool mergeflag = neighbor_pxl_it->is_being_merged;
				int grid_idx = neighbor_pxl_it->grid_idx;
				float high_mse_ratio = neighbor_pxl_it->pxl_high_mse_ratio;
				float eigen_mse = neighbor_pxl_it->pxl_eigen_mse;
				bool is_overall_merged = neighbor_pxl_it->is_overall_merged;
				int high_weighted_mse_cnt = neighbor_pxl_it->high_weighted_mse_cnt;
				float pxl_weighted_mse = neighbor_pxl_it->pxl_weighted_mse;
				bool is_connected = pxl_neighbor_it->is_connected;
				float  high_weighted_mse_ratio = neighbor_pxl_it->high_weighted_mse_ratio;

				pixel_neigbour << ' ' << std::setw(6) << pxl_neighbor_it->pixel_idx << std::setw(12) << direction_diff << std::setw(20) << line_dist \
					<< std::setw(18) << pxl_mse << std::setw(10) << num_of_point << std::setw(14) << mergeflag \
					<< std::setw(16) << grid_idx << std::setw(14) << high_mse_ratio << std::setw(18) << is_overall_merged \
					<< std::setw(18) << high_weighted_mse_cnt << std::setw(18) << pxl_weighted_mse\
					<< std::setw(16) << is_connected << std::setw(16) << high_weighted_mse_ratio << "\t\n";
			}

		}
		if (neighbor_flag_count != 0)
			delete[] sorted_good_neighbor_pixel_idx;
		if (badneighbor_flag_count != 0)
			delete[] sorted_bad_neighbor_pixel_idx;
	}
	pixel_neigbour.close();
	return;

}



bool DW_line_extract2D::IdentifyParentOfGoodPixels()///T
{
	unsigned char update_flag;					    // global update flag for each iterations
	unsigned char* update_flag_array;				// update flag for all occupied pixels
	unsigned char curr_ping_pong_buf_idx;			// index to parent_pixel_idx[2] in current iteration
	unsigned char next_ping_pong_buf_idx;			// index to parent_pixel_idx[2] in next iteration
	unsigned int iteration_count;
	iteration_count = 0;
#if 0
	for (unsigned int i = 0; i < pxl_occupied_array.size; i++) {
		cout <<"i:"<<i <<"  "<< pxl_occupied_array.pxl_item[i].parent_pixel_idx[0] << " " << pxl_occupied_array.pxl_item[i].parent_pixel_idx[1] << endl;
	}
#endif
	update_flag_array = new unsigned char[pxl_occupied_array.size];
	do
	{
		// Update current parent pixel index from neighbor's parent pixel index
		curr_ping_pong_buf_idx = iteration_count % 2;
		next_ping_pong_buf_idx = (iteration_count + 1) % 2;

//#pragma omp parallel for
		for (unsigned int i = 0; i < pxl_occupied_array.size; i++) {
			PixelGridItem *pxl_it = &pxl_occupied_array.pxl_item[i];
			update_flag_array[i] = 0;
			if (!pxl_it->is_overall_merged) continue;

			unsigned int current_parent_pixel_idx = pxl_it->parent_pixel_idx[curr_ping_pong_buf_idx];

			bool large_result[5], equal_result[5];

			unsigned best_parent_pixel_idx = current_parent_pixel_idx;
			for (int j = 0; j < 8; j++) 
			{
				PixelNeighborItem *pxl_neighbor_it = &pxl_it->neighbors[j];
				//if (!pxl_neighbor_it->high_weighted_mse_cnt > 0) continue;
				if ((!pxl_neighbor_it->in_line_flag)&&(!pxl_neighbor_it->neighbor_flag)) continue;
				unsigned int neighbor_pixel_idx = pxl_neighbor_it->pixel_idx;
				PixelGridItem *neighbor_pxl_it = &pxl_occupied_array.pxl_item[neighbor_pixel_idx];
				
				if (!neighbor_pxl_it->is_overall_merged) continue;
				unsigned int neighbor_parent_pixel_idx = neighbor_pxl_it->parent_pixel_idx[next_ping_pong_buf_idx];
				if (neighbor_parent_pixel_idx == best_parent_pixel_idx) continue;

				PixelGridItem * current_parent_pxl_it = &pxl_occupied_array.pxl_item[best_parent_pixel_idx];
				PixelGridItem * neighbor_parent_pxl_it = &pxl_occupied_array.pxl_item[neighbor_parent_pixel_idx];

				large_result[0] = neighbor_parent_pxl_it->pxl_weighted_mse < current_parent_pxl_it->pxl_weighted_mse;
				equal_result[0] = neighbor_parent_pxl_it->pxl_weighted_mse == current_parent_pxl_it->pxl_weighted_mse;
				large_result[1] = neighbor_parent_pxl_it->high_weighted_mse_ratio < current_parent_pxl_it->high_weighted_mse_ratio;
				equal_result[1] = neighbor_parent_pxl_it->high_weighted_mse_ratio == current_parent_pxl_it->high_weighted_mse_ratio;
				large_result[2] = neighbor_parent_pxl_it->points.size > current_parent_pxl_it->points.size;
				equal_result[2] = neighbor_parent_pxl_it->points.size == current_parent_pxl_it->points.size;
				large_result[3] = neighbor_parent_pxl_it->grid_idx < current_parent_pxl_it->grid_idx;
				equal_result[3] = neighbor_parent_pxl_it->grid_idx == current_parent_pxl_it->grid_idx;
				large_result[4] = false;
				equal_result[4] = true;

				unsigned char neighbor_update_flag = 0;
				bool comp_result = MathOperation::CompareByMultipleConditions(large_result, equal_result, neighbor_update_flag);

				if (!comp_result)
				{
					log_info("all conditions are the same");
					log_info("neighbor_parent_pixel_idx = %d current_parent_pixel_idx=%d", neighbor_parent_pixel_idx, current_parent_pixel_idx);
					log_info("neighbor mse = %d, current mse = %d", neighbor_parent_pxl_it->pxl_mse, current_parent_pxl_it->pxl_mse);
					log_info("neighbor pxl_high_mse_ratio = %d, current pxl_high_mse_ratio = %d", neighbor_parent_pxl_it->pxl_high_mse_ratio, current_parent_pxl_it->pxl_high_mse_ratio);
					log_info("neighbor point cnt = %d, current point cnt = %d", neighbor_parent_pxl_it->points.size, current_parent_pxl_it->points.size);
					log_info("neighbor grid_idx = %d, current grid_idx = %d", neighbor_parent_pxl_it->grid_idx, current_parent_pxl_it->grid_idx);
					return false;
				}
				else
				{
					if (neighbor_update_flag == 1)
					{
						update_flag_array[i] = 1;
						best_parent_pixel_idx = neighbor_parent_pixel_idx;
						//pxl_occupied_array.pxl_item[i].parent_pixel_idx[curr_ping_pong_buf_idx] = neighbor_parent_pixel_idx;
					}
				}
			}

			if (update_flag_array[i])
			{
				pxl_occupied_array.pxl_item[i].parent_pixel_idx[curr_ping_pong_buf_idx] = best_parent_pixel_idx;
			}
		}


		// Check if no more updates
		update_flag = 0;
		for (unsigned int i = 0; i < pxl_occupied_array.size; i++) {

			update_flag |= update_flag_array[i];
		}

		// Increment the iteration number
		iteration_count++;

	} while (update_flag > 0);
	// Release memory
	delete[] update_flag_array;
	//printf("identify good  parent iteration_count=%d\n", iteration_count);
#if 0
	for (unsigned int i = 0; i < pxl_occupied_array.size; i++) {
		cout << "i:" << i << "  " << pxl_occupied_array.pxl_item[i].parent_pixel_idx[0] << " " << pxl_occupied_array.pxl_item[i].parent_pixel_idx[1] << endl;
	}
#endif
	return true;

}


bool DW_line_extract2D::MergeGoodPixels()
{
	int* pixel_cnt_list;				// list of total number of  good  pixels  in a line
	int* line_list;				    // list of parent pixel indexes
	int* point_cnt_list;				// list of total number of  points  in line (a parent pixel )

	pixel_cnt_list = new int[pxl_occupied_array.size];
	point_cnt_list = new int[pxl_occupied_array.size];
//#pragma omp parallel for
	for (unsigned int i = 0; i < pxl_occupied_array.size; i++) {

		pixel_cnt_list[i] = 0;
		point_cnt_list[i] = 0;
	}
	int tmp_bad_pixel_cnt = 0;

	//step1:  Find out all pixels who have the same parent pixel index and count out number, points number
	for (unsigned int i = 0; i < pxl_occupied_array.size; i++) {
		PixelGridItem *pxl_it = &pxl_occupied_array.pxl_item[i];
		if (!pxl_it->is_overall_merged) 
			continue;
		int parent_pixel_idx = pxl_it->parent_pixel_idx[0];
		PixelGridItem *parent_pxl_it = &pxl_occupied_array.pxl_item[parent_pixel_idx];
		if (!parent_pxl_it->is_overall_merged) 
			continue;
		pixel_cnt_list[parent_pixel_idx]++;
		point_cnt_list[parent_pixel_idx] += pxl_it->points.size;
		pxl_it->is_being_merged = true;
	}

	// Calculate the total number of lines
	int tmp_line_cnt = 0;
	for (unsigned int i = 0; i < pxl_occupied_array.size; i++) {

		if (pixel_cnt_list[i] != 0) 
		{
			tmp_line_cnt++;
		}
	}
	//step2  Create the line list to store all the parent pixel indexes to the occupied pixel array
	line_list = new int[tmp_line_cnt];
	for (unsigned int i = 0, j = 0; i < pxl_occupied_array.size; i++) {

		if (pixel_cnt_list[i] != 0) {

			line_list[j] = i;
			j++;
		}
	}

	for (int i = 0; i < tmp_line_cnt; i++) {

		unsigned int parent_pixel_idx = line_list[i];
		PixelGridItem *parent_pxl_it = &pxl_occupied_array.pxl_item[parent_pixel_idx];
		//if (parent_pxl_it->is_overall_merged)
		{
			if ((pixel_cnt_list[parent_pixel_idx] < line_seg_thrshld_.min_pixel_num_of_line) || 
				(point_cnt_list[parent_pixel_idx] < line_seg_thrshld_.min_point_num_of_line))
			{
				//log_debug("parent pixel =%d pixel cnt =%d point cnt =%d ",parent_pixel_idx, pixel_cnt_list[parent_pixel_idx], point_cnt_list[parent_pixel_idx]);
				pixel_cnt_list[parent_pixel_idx] = 0;
				parent_pxl_it->is_overall_merged = false;
				parent_pxl_it->is_being_merged = false;
			}
		}
	}

	for (unsigned int i = 0; i < pxl_occupied_array.size; i++) {

		PixelGridItem *pxl_it = &pxl_occupied_array.pxl_item[i];
		int parent_pixel_idx = pxl_it->parent_pixel_idx[0];
		PixelGridItem *parent_pxl_it = &pxl_occupied_array.pxl_item[parent_pixel_idx];

		if ((parent_pxl_it->in_line_pixel) && (!parent_pxl_it->is_overall_merged))
		{
			if (pxl_it->is_overall_merged)
			{
				pxl_it->is_being_merged = false;
				pxl_it->is_overall_merged = false;
			}
			else
			{
				if (pxl_it->is_being_merged)
					pxl_it->is_being_merged = false;
			}
		}
	}
	delete[] line_list;
	line_list = NULL;
	int line_cnt = 0;
	for (unsigned int i = 0; i < pxl_occupied_array.size; i++) {

		if (pixel_cnt_list[i] != 0) 
		{
			line_cnt++;
		}
	}

	line_list = new int[line_cnt];	
	pixel_to_line_idx = new unsigned int[pxl_occupied_array.size];
	for (unsigned int i = 0, j = 0; i < pxl_occupied_array.size; i++) {

		if (pixel_cnt_list[i] != 0) {

			line_list[j] = i;
			pixel_to_line_idx[i] = j;
			j++;
		}
		else
		{
			pixel_to_line_idx[i] = INVALID_LINE_IDX;
		}
	}


	// Create instant of all LineMergeOutputItem in LineMergeOutput
	line_merge_out.size = line_cnt;
	line_merge_out.lines = new LineMergeOutputItem[line_cnt];

	for (unsigned int i = 0; i < line_merge_out.size; i++) {

		LineMergeOutputItem *line_item = &line_merge_out.lines[i];
		unsigned int parent_pixel_idx = line_list[i];
		unsigned int pixel_cnt = pixel_cnt_list[parent_pixel_idx];
		line_item->parent_pixel_idx = parent_pixel_idx;
		line_item->pixels.pxl_idx = new unsigned int[pixel_cnt];
		line_item->pixels.size = pixel_cnt;
		line_item->points.point_idx = NULL;
		line_item->points.size = 0;
		line_item->good_pixel_size = pixel_cnt;
		SumsClear(&line_item->sums);
		line_item->line_mse = std::numeric_limits<float> ::infinity();
		line_item->line_direction = { 0.f,0.f};
		line_item->line_center = { 0.f,0.f};
		line_item->total_point_cnt = point_cnt_list[parent_pixel_idx];
	}

	for (unsigned int i = 0; i < pxl_occupied_array.size; i++) {

		pixel_cnt_list[i] = 0;
	}

	// Assign pixel indexes to LineMergeOutputItem
	for (unsigned int i = 0; i < pxl_occupied_array.size; i++) {

		PixelGridItem *pxl_it = &pxl_occupied_array.pxl_item[i];
		if (pxl_it->is_overall_merged)
		{
			unsigned int parent_pixel_idx = pxl_it->parent_pixel_idx[0];
			PixelGridItem *parent_pxl_it = &pxl_occupied_array.pxl_item[parent_pixel_idx];
			if (!parent_pxl_it->is_overall_merged) 
				continue;
			unsigned int pixel_cnt = pixel_cnt_list[parent_pixel_idx];
			int line_index  = pixel_to_line_idx[parent_pixel_idx];
			if (line_index == INVALID_LINE_IDX) 
				continue;
			line_merge_out.lines[line_index].pixels.pxl_idx[pixel_cnt] = i;
			pixel_cnt_list[parent_pixel_idx]++;
		}
	}
	// add pixels' covariance matrix sums to line , compute the line direction and line center , and compute the line mse
//#pragma omp parallel for
	for (unsigned int i = 0; i < line_merge_out.size; i++)
	{
		LineMergeOutputItem *line_item = &line_merge_out.lines[i];
		for (int k = 0; k < line_item->pixels.size; k++)
		{
			unsigned int pixel_idx = line_item->pixels.pxl_idx[k];
			PixelGridItem *pxl_it = &pxl_occupied_array.pxl_item[pixel_idx];
			PushSums(&line_item->sums, &pxl_it->sums);			
		}
		float eigen_mse;
		Compute2dS(line_item->total_point_cnt, line_item->sums, line_item->line_direction, line_item->line_center, eigen_mse);
		GetLineDistMse(line_item, line_item->line_mse);
	}
	delete[] pixel_cnt_list;
	delete[] point_cnt_list;
	delete[] line_list;
	return true;
}


bool DW_line_extract2D::IdentifyParentOfbadPixels()
{
	int cnt = 0;
	// record the bad pixel index of each occupied pixel
	occupied_idx_to_bad_pxl = new unsigned int[pxl_occupied_array.size];
	//float max_connected_distance = line_seg_thrshld_.min_line_dist_pt2pixel * 2;
	float max_connected_distance = line_seg_thrshld_.min_connected_dist_2pxl;

	for (unsigned int i = 0; i < pxl_occupied_array.size; i++)
	{
		PixelGridItem *pxl_it = &pxl_occupied_array.pxl_item[i];
		if (!pxl_it->is_overall_merged)
		{
			occupied_idx_to_bad_pxl[i] = cnt;
			cnt++;
		}
		else
		{
			occupied_idx_to_bad_pxl[i] = std::numeric_limits<unsigned int>::max();
		}
	}
	
	// create unmerged pixels array
	bad_pxl_merge_array.size = cnt;
	bad_pxl_merge_array.bad_pxl_merge_it = new BadPixelMergeOutpuItem[cnt];

	//unmerged pixels array initialized
//#pragma omp parallel for
	for (unsigned int i = 0; i < pxl_occupied_array.size; i++)
	{
		PixelGridItem *pxl_it = &pxl_occupied_array.pxl_item[i];
		if (!pxl_it->is_overall_merged)
		{
			int bad_pxl_idx = occupied_idx_to_bad_pxl[i];
			BadPixelMergeOutpuItem *bad_pxl_merge_item = &bad_pxl_merge_array.bad_pxl_merge_it[bad_pxl_idx];

			bad_pxl_merge_item->pxl_idx = i;
			bad_pxl_merge_item->pixel_point_size = pxl_it->points.size;
			bad_pxl_merge_item->is_being_merged = false;
			bad_pxl_merge_item->closest_line_idx = new unsigned int[pxl_it->points.size];
			bad_pxl_merge_item->point_merged_flag = new bool[pxl_it->points.size];
			for (unsigned int j = 0; j < pxl_it->points.size; j++)
			{
				bad_pxl_merge_item->closest_line_idx[j] = std::numeric_limits<unsigned int>::max();
				bad_pxl_merge_item->point_merged_flag[j] = false;
			}
			bad_pxl_merge_item->num_of_points_merged = 0;
			bad_pxl_merge_item->is_neighbor_of_line = false;
			bad_pxl_merge_item->is_of_line_list = new bool[line_merge_out.size];
			for (unsigned int j = 0; j < line_merge_out.size; j++)
			{
				bad_pxl_merge_item->is_of_line_list[j] = false;
			}
		}

	}

	// Identify parent pixel in all unmerged pixels
//#pragma omp parallel for
	std::vector <bool> upadate_flag_array(bad_pxl_merge_array.size,0); // record flag if new points found in all the bad pixels
	bool update_flag = true;
	int iteration_count = 0;

	while (update_flag)
	{
		update_flag = false;
		for (unsigned int i = 0; i < bad_pxl_merge_array.size; i++)
		{
			upadate_flag_array[i] = false;
			BadPixelMergeOutpuItem * bad_pxl_merge_it = &bad_pxl_merge_array.bad_pxl_merge_it[i];
			PixelGridItem *pxl_it = &pxl_occupied_array.pxl_item[bad_pxl_merge_it->pxl_idx];
			std::vector<unsigned int> is_neighbor_of_line_list(line_merge_out.size, 0);
			//check if the pixel neighbors have line,record the flag
			for (int j = 0; j < 8; j++)
			{
				PixelNeighborItem *pxl_neighbor_it = &pxl_it->neighbors[j];
				if (!pxl_neighbor_it->is_connected) 
					continue;
				unsigned int neighbor_pixel_idx = pxl_neighbor_it->pixel_idx;
				PixelGridItem *neighbor_pxl_it = &pxl_occupied_array.pxl_item[neighbor_pixel_idx];
				if (neighbor_pxl_it->is_overall_merged)
				{
					unsigned int neighbor_parent_pixel_idx = neighbor_pxl_it->parent_pixel_idx[0];
					unsigned int line_idx = pixel_to_line_idx[neighbor_parent_pixel_idx];
					if ((line_idx < line_merge_out.size) && (line_idx != INVALID_LINE_IDX))
					{
						is_neighbor_of_line_list[line_idx] = 1;
						bad_pxl_merge_it->is_neighbor_of_line = true;
					}
				}
				else
				{
					unsigned int neighbor_bad_pxl_idx = occupied_idx_to_bad_pxl[neighbor_pixel_idx];
					if (neighbor_bad_pxl_idx >= bad_pxl_merge_array.size)
					{
						log_debug("neighbor_bad_pxl_idx is exceed size of bad pixels =%d", neighbor_bad_pxl_idx, bad_pxl_merge_array.size);
						continue;
					}
					BadPixelMergeOutpuItem * neighbor_bad_pxl_merge_it = &bad_pxl_merge_array.bad_pxl_merge_it[neighbor_bad_pxl_idx];
					for (unsigned int k = 0; k < line_merge_out.size; k++)
					{
						if (!neighbor_bad_pxl_merge_it->is_of_line_list[k]) 
							continue;
						std::vector<unsigned int> neighbor_pxl_points(0);
						for (unsigned int m = 0; m < neighbor_pxl_it->points.size; m++)
						{
							if (neighbor_bad_pxl_merge_it->closest_line_idx[m] == k)
							{
								neighbor_pxl_points.push_back(neighbor_pxl_it->points.point_idx[m]);
							}
						}
						bool is_close = Identify2PointsGroupCLoseByDistance(pt_points, neighbor_pxl_points,
																			pxl_it->points, max_connected_distance);
						if (is_close)
						{
							is_neighbor_of_line_list[k] = 1;
							bad_pxl_merge_it->is_neighbor_of_line = true;
						}
					}
				}
			}

			if (!bad_pxl_merge_it->is_neighbor_of_line)
				continue;

			// if have neighbor in line, compute the point to line distance and find the closest line index
			for (unsigned int j = 0; j < pxl_it->points.size; j++)
			{
				//if this point has already found , continue next;
				if (bad_pxl_merge_it->closest_line_idx[j] != std::numeric_limits<unsigned int>::max()) 
					continue;

				Point2f current_point = pt_points[pxl_it->points.point_idx[j]];
				float max_dist = std::numeric_limits<float>::infinity();
				for (unsigned int k = 0; k < line_merge_out.size; k++)
				{
					if (is_neighbor_of_line_list[k] == 0)
						continue;
					LineMergeOutputItem *line_item = &line_merge_out.lines[k];
					float line_dist = ComputePointToLineDist(current_point, line_item->line_center, line_item->line_direction);
					if ((line_dist < max_dist) && (line_dist < line_seg_thrshld_.min_line_dist_pt2pixel))
					{
						max_dist = line_dist;
						bad_pxl_merge_it->is_being_merged = true;
						pxl_it->is_being_merged = true;
						bad_pxl_merge_it->closest_line_idx[j] = k;
						bad_pxl_merge_it->is_of_line_list[k] = true;
						update_flag = true; // if new point found, set update flag to go on next iteration
					}
				}
			}
		}
		iteration_count++;
	}

	//log_info("IdentifyParentOfbadPixels iteration_count =%d", iteration_count);
	return true;
}


bool DW_line_extract2D::MergeBadPixels()
{
	//unsigned int *points_in_line_list = new unsigned int[line_merge_out.size];
	//memset(points_in_line_list, 0, sizeof(unsigned int)*line_merge_out.size);
	std::vector<unsigned int>points_in_line_list(line_merge_out.size,0);
	for (unsigned int i = 0; i < bad_pxl_merge_array.size; i++)
	{
		BadPixelMergeOutpuItem * bad_pxl_merge_it = &bad_pxl_merge_array.bad_pxl_merge_it[i];
		PixelGridItem *pxl_it = &pxl_occupied_array.pxl_item[bad_pxl_merge_it->pxl_idx];
		if (!bad_pxl_merge_it->is_being_merged) 
			continue;
		// get number of all the unmerged pixels' points for each line
		for (unsigned int j = 0; j < line_merge_out.size; j++)
		{

			for (unsigned int k = 0; k < pxl_it->points.size; k++)
			{
				//if (bad_pixel_merge_it->point_merged_flag[k]) continue;
				if (bad_pxl_merge_it->closest_line_idx[k] == j) 
					points_in_line_list[j]++; //??j??¨®D¦Ì?points ¨ºy¨¢?
			}
		}
	}

//#pragma omp parallel for
	for (unsigned int i = 0; i < line_merge_out.size; i++)
	{
		LineMergeOutputItem* line_it = &line_merge_out.lines[i];
		line_it->points.size = points_in_line_list[i];
		if (line_it->points.size == 0) 
			continue;
		line_it->points.point_idx = new unsigned int[line_it->points.size];
		memset(line_it->points.point_idx, 0, sizeof(unsigned int)*line_it->points.size);
		line_it->total_point_cnt += line_it->points.size;
	}

	for (unsigned int i = 0; i < line_merge_out.size; i++)
	{
		LineMergeOutputItem* line_it = &line_merge_out.lines[i];
		if (line_it->points.size == 0) 
			continue;
		unsigned int parent_pixel_idx = line_it->parent_pixel_idx;
		points_in_line_list[i] = 0;
		//assign point index  and compute Covariance sums for each line
		for (unsigned int j = 0; j < bad_pxl_merge_array.size; j++)
		{
			BadPixelMergeOutpuItem * bad_pxl_merge_it = &bad_pxl_merge_array.bad_pxl_merge_it[j];
			PixelGridItem *pxl_it = &pxl_occupied_array.pxl_item[bad_pxl_merge_it->pxl_idx];
			if (!bad_pxl_merge_it->is_neighbor_of_line) //¦Ì¡À?¡ãpxl ??¨®Dneighbor?¨²??¨¦?
				continue;
			for (unsigned int k = 0; k < pxl_it->points.size; k++)
			{
				if (bad_pxl_merge_it->closest_line_idx[k] ==i)
				{
					unsigned int point_idx = pxl_it->points.point_idx[k];
					// merge the points of this pixel to line
					line_it->points.point_idx[points_in_line_list[i]] = point_idx;
					points_in_line_list[i]++;
					bad_pxl_merge_it->num_of_points_merged++;
					Point2f Point =pt_points[point_idx];
					PushPoint(&line_it->sums, Point);
					bad_pxl_merge_it->point_merged_flag[k] = true;
				}
			}

		}
		if (points_in_line_list[i] != line_it->points.size) 
			log_debug("line %d pixel merge size =%d is not equal to identify size =%d ", i, points_in_line_list[i], line_it->points.size);
		float eigen_mse;
		Compute2dS(line_it->total_point_cnt, line_it->sums, line_it->line_direction, line_it->line_center, eigen_mse);
		GetLineDistMse(&line_merge_out.lines[i], line_it->line_mse);
	}
	return true;
}



bool DW_line_extract2D::IdentifySameLines(bool **is_same_line, unsigned int line_size)
{
	float min_direction_diff = static_cast<float>(std::cos(line_seg_thrshld_.max_angle_of_2line * M_PI / 180));
	//float large_line_dist_scale = static_cast<float>(std::sin(line_seg_thrshld_.max_angle_of_2line * M_PI / 180 / 2));
	float max_dist_2line_same = line_seg_thrshld_.min_dist_of_2line;	
	float large_line_center_dist = 5 * (pixel_size_.length_x_of_pixel + pixel_size_.length_y_of_pixel) / 2;
	for (unsigned int i = 0; i < line_size; i++)
	{
		//Point2f line_direction, line_center, other_line_direction, other_line_center;
		LineMergeOutputItem* line_it = &line_merge_out.lines[i];
		for (unsigned int j = i + 1; j < line_size; j++)
		{
			if (is_same_line[i][j])
				continue;
			LineMergeOutputItem* other_line_it = &line_merge_out.lines[j];
			float direction_diff = std::fabsf(line_it->line_direction.dot(other_line_it->line_direction));
			if (direction_diff < min_direction_diff) //???????¨¨¡ä¨®¨®¨²10?¨¨¡ê?2?¨º?¨ª?¨°?¨¬???
				continue;
			float line_dist = ComputePointToLineDist(line_it->line_center, other_line_it->line_center, other_line_it->line_direction);
			float other_line_dist = ComputePointToLineDist(other_line_it->line_center, line_it->line_center, line_it->line_direction);
			float center_dist = Util_Math::DWComputePointToPointDist2D<Point2f>(line_it->line_center, other_line_it->line_center);
			if (center_dist > large_line_center_dist)
			{
				float large_line_dist_scale = static_cast<float>(std::sin(std::acos(direction_diff)));
				max_dist_2line_same = line_seg_thrshld_.min_dist_of_2line + large_line_dist_scale * center_dist;
			}
			else
			{
				max_dist_2line_same = line_seg_thrshld_.min_dist_of_2line;
			}
			float avg_line_dist = (line_dist + other_line_dist) / 2;
			if (avg_line_dist < max_dist_2line_same)
			{
				is_same_line[i][j] = true;
				is_same_line[j][i] = true;
			}
		}
	}
	return true;
}


bool DW_line_extract2D::IdentifyConnectedLines(bool **is_connected, unsigned int line_size)
{
	//float max_connected_distance = line_seg_thrshld_.min_line_dist_pt2pixel * 2;
	float max_connected_distance = line_seg_thrshld_.min_connected_dist_2pxl;

	for (unsigned int i = 0; i < pxl_occupied_array.size; i++)
	{
		PixelGridItem *pxl_it = &pxl_occupied_array.pxl_item[i];
		// if pixel is overall merged ,  find the connected line  int the bad neighbors( the other line must be in bad neighbors)
		if (pxl_it->is_overall_merged)
		{
			unsigned int line_idx = pixel_to_line_idx[pxl_it->parent_pixel_idx[0]];
			if (line_idx> line_size)// not need, if log there must be a bug
			{
				log_debug("line_idx %d pxl% is exceed  line_size", line_idx,i, line_size);
				continue;
			}
			for (int j = 0; j < 8; j++)
			{
				PixelNeighborItem *pxl_neighbor_it = &pxl_it->neighbors[j];
				if (!pxl_neighbor_it->is_connected) continue;
				//if (!pxl_neighbor_it->in_line_flag) continue;
				//if (!pxl_neighbor_it->neighbor_flag) continue;
				PixelGridItem *neighbor_pxl_it = &pxl_occupied_array.pxl_item[pxl_neighbor_it->pixel_idx];

				if (neighbor_pxl_it->is_overall_merged)
				{
					unsigned int neighbor_line_idx = pixel_to_line_idx[neighbor_pxl_it->parent_pixel_idx[0]];

					if (is_connected[line_idx][neighbor_line_idx]) continue;
					is_connected[line_idx][neighbor_line_idx] = true;
					is_connected[neighbor_line_idx][line_idx] = true;
				}
				else
				{
					unsigned int bad_pixel_idx = occupied_idx_to_bad_pxl[pxl_neighbor_it->pixel_idx];
					if (bad_pixel_idx >= pxl_occupied_array.size)// not need, if log there must be a bug
					{
						log_info("pxl_neighbor_it->pixel_idx %d bad_pixel_idx %d is exceed the size of pxl_occupied_array = %d ", pxl_neighbor_it->pixel_idx, bad_pixel_idx, pxl_occupied_array.size);
						continue;
					}
					BadPixelMergeOutpuItem * bad_pxl_it = &bad_pxl_merge_array.bad_pxl_merge_it[bad_pixel_idx];

					for (unsigned int k = 0; k < line_merge_out.size; k++)
					{
						if (is_connected[line_idx][k]) continue;
						if (!bad_pxl_it->is_of_line_list[k]) continue;
						std::vector<unsigned int> neigbor_pxl_points(0);
						for (unsigned int m = 0; m < neighbor_pxl_it->points.size; m++)
						{
							//if (!bad_pxl_it->point_merged_flag[m]) continue;
							if (bad_pxl_it->closest_line_idx[m] == k) 
								neigbor_pxl_points.push_back(neighbor_pxl_it->points.point_idx[m]);
						}
						bool is_close = Identify2PointsGroupCLoseByDistance(pt_points, neigbor_pxl_points, pxl_it->points, max_connected_distance);
						if (is_close)
						{
							is_connected[line_idx][k] = true;
							is_connected[k][line_idx] = true;
							//log_debug("line[%d,%d] is connected", line_idx, k);
						}
					}
				}
			}

		}
		else  // if current pixel is bad pixel , find connected line  in self pixel  and its neighbors
		{
			unsigned int bad_pixel_idx = occupied_idx_to_bad_pxl[i];
			if (bad_pixel_idx >= pxl_occupied_array.size)  // not need, if log there must be a bug
			{
				log_info("pixel %d bad_pixel_idx %d is exceed the size of pxl_occupied_array = %d ", i, bad_pixel_idx, pxl_occupied_array.size);
				continue;
			}
			BadPixelMergeOutpuItem * bad_pxl_merge_it = &bad_pxl_merge_array.bad_pxl_merge_it[bad_pixel_idx];
			if (!bad_pxl_merge_it->is_being_merged)
				continue;

			for (unsigned int j = 0; j < line_size; j++)
			{
				if (!bad_pxl_merge_it->is_of_line_list[j]) 
					continue;
				// first step: find line in self pixel
				for (unsigned int k = j + 1; k < line_size; k++)
				{
					if (is_connected[j][k])
						continue;
					if (!bad_pxl_merge_it->is_of_line_list[k]) 
						continue;
					std::vector<unsigned int> first_points(0);
					std::vector<unsigned int> sec_points(0);
					for (unsigned int m = 0; m < pxl_it->points.size; m++)
					{
						//if (!bad_pxl_merge_it->point_merged_flag[m]) continue;
						if (bad_pxl_merge_it->closest_line_idx[m] == j) 
							first_points.push_back(pxl_it->points.point_idx[m]);
						if (bad_pxl_merge_it->closest_line_idx[m] == k) 
							sec_points.push_back(pxl_it->points.point_idx[m]);
					}
					bool is_close = Identify2PointsGroupCLoseByDistance1(pt_points, first_points, sec_points, max_connected_distance);
					if (is_close)
					{
						is_connected[j][k] = true;
						is_connected[k][j] = true;
					}
				}
				// second step: find connected line in neigbors
				for (unsigned int k = j + 1; k < line_size; k++)
				{
					if (is_connected[j][k]) 
						continue;
					for (unsigned int m = 0; m < 8; m++)
					{
						PixelNeighborItem *pxl_neighbor_it = &pxl_it->neighbors[m];
						if (!pxl_neighbor_it->is_connected) 
							continue;
						PixelGridItem *neighbor_pxl_it = &pxl_occupied_array.pxl_item[pxl_neighbor_it->pixel_idx];
						if (neighbor_pxl_it->is_overall_merged) 
							continue; // has already found when  pixel is is_overall_merged
						unsigned int neighbor_bad_pxl_idx = occupied_idx_to_bad_pxl[pxl_neighbor_it->pixel_idx];
						if (neighbor_bad_pxl_idx >= bad_pxl_merge_array.size)// not need , if log , there must be a bug;
						{
							log_debug("neighbor_bad_pxl_idx %d is exceed size of bad pixels =%d", neighbor_bad_pxl_idx, bad_pxl_merge_array.size);
							continue; 
						}
						BadPixelMergeOutpuItem *neighbor_bad_pxl_it = &bad_pxl_merge_array.bad_pxl_merge_it[neighbor_bad_pxl_idx];
						if (!neighbor_bad_pxl_it->is_of_line_list[k]) 
							continue;
						std::vector<unsigned int> pxl_points(0);
						std::vector<unsigned int> neighbor_points(0);

						for (unsigned int n = 0; n < pxl_it->points.size; n++)
						{
							if (!bad_pxl_merge_it->point_merged_flag[n]) 
								continue;
							if (bad_pxl_merge_it->closest_line_idx[n] == j) 
								pxl_points.push_back(pxl_it->points.point_idx[n]);
						}

						for (unsigned int n = 0; n < neighbor_pxl_it->points.size; n++)
						{
							if (!neighbor_bad_pxl_it->point_merged_flag[n]) 
								continue;
							if (neighbor_bad_pxl_it->closest_line_idx[n] == k) 
								neighbor_points.push_back(neighbor_pxl_it->points.point_idx[n]);
						}
						bool is_close = Identify2PointsGroupCLoseByDistance1(pt_points, pxl_points, neighbor_points, max_connected_distance);
						if (is_close)
						{
							is_connected[j][k] = true;
							is_connected[k][j] = true;
							//log_debug("line[%d,%d] is connected", j, k);
						}
					}

				}
			}

			

		}
	}

#if 0
	// check if a pixel have several lines , these line is connected
	for (int i = 0; i < bad_pxl_merge_array.size; i++)
	{
		BadPixelMergeOutpuItem * bad_pxl_merge_it = &bad_pxl_merge_array.bad_pxl_merge_it[i];
		if (!bad_pxl_merge_it->is_being_merged) continue;  // if pixel in line should have no point in a line ,continue next
		for (int j = 0; j < line_size; j++)
		{
			if (!bad_pxl_merge_it->is_of_line_list[j]) continue;
			for (int k = j+1; k < line_size; k++)
			{
				if (is_connected[j][k]) continue;
				if (!bad_pxl_merge_it->is_of_line_list[k]) continue;
				log_debug("line[%d,%d] is connected", j, k);
				is_connected[j][k] = true;
				is_connected[k][j] = true;
			}
		}

	}

	//if a pixel in current line is not a overall merged and whose center to a neighbor's line distance < predefined threshold, 
	// this neighbour's line are connected with the current line
	for (int i = 0; i < bad_pxl_merge_array.size; i++)
	{
		BadPixelMergeOutpuItem * bad_pxl_merge_it = &bad_pxl_merge_array.bad_pxl_merge_it[i];
		PixelGridItem *pxl_it = &pxl_occupied_array.pxl_item[bad_pxl_merge_it->pxl_idx];
		if (!bad_pxl_merge_it->is_being_merged) continue;  // if pixel have no point in a line ,continue next

		for (int j = 0; j < line_merge_out.size; j++)
		{
			if (!bad_pxl_merge_it->is_of_line_list[j]) continue; // if pixel have no point in current line ,continue next line
			LineMergeOutputItem* line_it = &line_merge_out.lines[j];
			for (int k = 0; k < 8; k++)
			{
				PixelNeighborItem *pxl_neighbor_it = &pxl_it->neighbors[j];
				if (!pxl_neighbor_it->is_occupied) continue;
				PixelGridItem *neighbor_pxl_it = &pxl_occupied_array.pxl_item[pxl_neighbor_it->pixel_idx];
				if (neighbor_pxl_it->is_overall_merged)
				{
					unsigned int neighbor_line_idx = pixel_to_line_idx[neighbor_pxl_it->parent_pixel_idx[0]];
					if (is_connected[j][neighbor_line_idx]) continue;
					//if ((neighbor_line_idx >= line_merge_out.size) || (neighbor_line_idx == INVALID_LINE_IDX)) continue;
					LineMergeOutputItem* neighbor_line_it = &line_merge_out.lines[neighbor_line_idx];
					float center_dist = ComputePointToLineDist(pxl_it->center, neighbor_line_it->line_center, neighbor_line_it->line_direction);
					if (center_dist < line_seg_thrshld_.min_dist_of_2line)
					{
						is_connected[j][neighbor_line_idx] = true;
						is_connected[neighbor_line_idx][j] = true;
						log_debug("line[%d,%d] is connected", j, neighbor_line_idx);
					}
				}
				else
				{
					for (int m = 0; m < line_merge_out.size; m++)
					{
						if (is_connected[j][m]) continue;
						if (!bad_pxl_merge_it->is_of_line_list[m]) continue;
						LineMergeOutputItem* neighbor_line_it = &line_merge_out.lines[m];
						float center_dist = ComputePointToLineDist(pxl_it->center, neighbor_line_it->line_center, neighbor_line_it->line_direction);
						if (center_dist < line_seg_thrshld_.min_dist_of_2line)
						{
							is_connected[j][m] = true;
							is_connected[m][j] = true;
							log_debug("line[%d,%d] is connected", j, m);
						}
					}
				}
			}
		}
	}
#endif

	return true;
}

void DW_line_extract2D::AddLineToSameLineGroup(const unsigned int line_in, const unsigned int group_idx, 
	                                           const unsigned int line_size, bool** is_same_line, bool** is_connected)
{
	// if it is the new group , must find again in all the other line to add the same group line
	//unsigned int same_group_idx = same_line_group_array.same_line_group_idx[line_in_group];
	//find same line again in all the lines 
	
	for (unsigned int i = 0; i < line_size; i++)
	{
		unsigned int sameline_idx_of_line = same_line_group_array.same_line_group_idx[i];
		if (sameline_idx_of_line != INVALID_LINE_IDX) 
			continue;
		if ((is_connected[i][line_in]) && (is_same_line[i][line_in]))//io¨ª?¡ä¡¤?¡Á¨¦¦Ì?line_in ?¨¤¨¢??¨®?¨°¨ª???
		{ 
			//if find one same and connected line with current line, must check it is same direction with all the lines in the group
			bool is_same_with_all_lines_in_group = true;

			for (unsigned int j = 0; j < line_size; j++)
			{
				unsigned int cur_tmp_same_group_idx = same_line_group_array.same_line_group_idx[j];
				if (cur_tmp_same_group_idx == group_idx)//??j¦Ì?¡¤?¡Á¨¦o¨ªgroup_idx ?¨¤¨ª?
				{
					LineMergeOutputItem* line_it = &line_merge_out.lines[j];
					if(line_it->pixels.size < line_seg_thrshld_.min_pixel_num_of_line * 2)//j??¨®D¡Á?1?¦Ì?pixel
						continue;
					if (!is_same_line[j][i]) //jo¨ªi2?¨ª???
						is_same_with_all_lines_in_group = false;//¨¨?1?group_idx¡Á¨¦?¨²¦Ì?¨¨?o?¨°?¨¬???o¨ªi2?¨ª????¨°false;
				}
			}

			// new same plane can be added to the group  only when it is same with all good plane in group 
			if (is_same_with_all_lines_in_group)//o¨ª line_in?¨¤¨¢??¨°¨ª???¦Ì?i ¨®?¡Á¨¦group_idx ?¨²¦Ì??¨´¨®D????¨ª???
			{
				same_line_group_array.same_line_group_idx[i] = group_idx; //??i??¡¤?¦Ì?¡Á¨¦group_idx
				log_debug("line[%d] is add same line idx =%d!", i, group_idx);
				AddLineToSameLineGroup(i, group_idx, line_size, is_same_line, is_connected);
			}
		}
	}


}

bool DW_line_extract2D::MergeLines()
{
	bool **is_same_line; //the lines'  same line flag  array to all the lines, true show the line is  same with the other line
	bool **is_connected;  //the lines'  connected flag  array of all lines, true show the line is  connected with the other line
	int total_line_size = line_merge_out.size;

	// apply resource and initialized
	same_line_group_array.line_size = total_line_size;
	same_line_group_array.same_line_group_idx = new unsigned int[total_line_size];
	is_same_line = new bool*[total_line_size];
	is_connected = new bool*[total_line_size];

//#pragma omp parallel for
	for (int i = 0; i < total_line_size; i++)
	{
		same_line_group_array.same_line_group_idx[i] = INVALID_LINE_IDX;
		is_same_line[i] = new bool[total_line_size];
		is_connected[i] = new bool[total_line_size];
		for (int j = 0; j < total_line_size; j++)
		{
			if (j == i)
			{
				is_same_line[i][j] = true;
				is_connected[i][j] = true;
			}
			else
			{
				is_same_line[i][j] = false;
				is_connected[i][j] = false;
			}
		}
	}

	//check if line to line direction and distance is same direction 
	IdentifySameLines(is_same_line, total_line_size);

	// check if line is connected the line
	IdentifyConnectedLines(is_connected, total_line_size);

#if 0
	for (int i = 0; i < total_line_size; i++)
	{
		for (int j = i + 1; j < total_line_size; j++)
		{
			if (is_same_line[i][j])
			{
				printf("same line %d line %d  is_connected = %d", i, j, is_connected[i][j]);
			}
		}
	}
#endif

	//return true;
	unsigned int same_line_group_idx = 0;

	for (int i = 0; i < total_line_size; i++)
	{
		for (int j = i + 1; j < total_line_size; j++)
		{
			unsigned int sameline_idx_of_first_line = same_line_group_array.same_line_group_idx[i]; 
			unsigned int sameline_idx_of_second_line = same_line_group_array.same_line_group_idx[j];
			if ((sameline_idx_of_second_line != INVALID_LINE_IDX) && (sameline_idx_of_first_line != INVALID_LINE_IDX)) continue; // if first and second line have already the same line idx , to check next line
			if ((is_connected[i][j]) && (is_same_line[i][j]))
			{
				if (sameline_idx_of_first_line != INVALID_LINE_IDX)     // if first line have already the same line idx , just add second line to same idx and  check next line
				{
					//same_line_group_array.same_line_group_idx[j] = sameline_idx_of_first_line;
					AddLineToSameLineGroup(j, sameline_idx_of_first_line, total_line_size,is_same_line,is_connected);
					//log_debug("line[%d] is add same line idx =%d!", j, sameline_idx_of_first_line);
					continue;
				}

				if (sameline_idx_of_second_line != INVALID_LINE_IDX)     //  if second line have already the same line idx ,  just add first line to same idx and  check next line
				{
					//same_line_group_array.same_line_group_idx[i] = sameline_idx_of_second_line;
					AddLineToSameLineGroup(i, sameline_idx_of_second_line, total_line_size, is_same_line, is_connected);
					//log_debug("line[%d] is add same line idx =%d!", i, sameline_idx_of_second_line);
					continue;
				}

				same_line_group_array.same_line_group_idx[i] = same_line_group_idx;
				same_line_group_array.same_line_group_idx[j] = same_line_group_idx;
				AddLineToSameLineGroup(i, same_line_group_idx, total_line_size, is_same_line, is_connected);
				AddLineToSameLineGroup(j, same_line_group_idx, total_line_size, is_same_line, is_connected);
				log_debug("line[%d,%d] is same line idx =%d!", i, j, same_line_group_idx);
				same_line_group_idx++;
			}
		}
	}
	same_line_group_array.group_number = same_line_group_idx;


	for ( int i = 0; i < total_line_size; i++)
	{
		delete[] is_same_line[i];
		delete[] is_connected[i];
	}
	delete[] is_same_line;
	delete[] is_connected;

	return true;
}


bool DW_line_extract2D::AssignLine(LineMergeOutputItem *orgin_line_it, LineSegItem *out_line_seg_it)
{
	int point_in_line_cnt = 0;
	for (int j = 0; j < orgin_line_it->pixels.size; j++) {

		unsigned int pixel_idx_in_line = orgin_line_it->pixels.pxl_idx[j];
		unsigned int point_in_pixel_cnt = pxl_occupied_array.pxl_item[pixel_idx_in_line].points.size;
		point_in_line_cnt += point_in_pixel_cnt;
	}
	point_in_line_cnt += orgin_line_it->points.size;
	//out_line_seg_it->parent_pixel_idx = orgin_line_it->parent_pixel_idx;
	out_line_seg_it->points.size = point_in_line_cnt;
	out_line_seg_it->points.point_idx = new unsigned int[point_in_line_cnt];
	// Assign point indexes to LineSegItem from the points in overall pixels
	point_in_line_cnt = 0;
	for (int j = 0; j < orgin_line_it->pixels.size; j++) {

		unsigned int pixel_in_line_idx = orgin_line_it->pixels.pxl_idx[j];
		unsigned int point_in_pixel_cnt = pxl_occupied_array.pxl_item[pixel_in_line_idx].points.size;

		for (unsigned int k = 0; k < point_in_pixel_cnt; k++) {

			unsigned int point_in_line_idx = pxl_occupied_array.pxl_item[pixel_in_line_idx].points.point_idx[k];
			out_line_seg_it->points.point_idx[point_in_line_cnt] = point_in_line_idx;
			point_in_line_cnt++;
		}
	}

	for (unsigned int j = 0; j < orgin_line_it->points.size; j++) {

		unsigned int point_in_line_idx = orgin_line_it->points.point_idx[j];
		out_line_seg_it->points.point_idx[point_in_line_cnt] = point_in_line_idx;
		point_in_line_cnt++;
	}

	// Assign line, line direction, line_mse and sums to lineItem
	out_line_seg_it->line_center = orgin_line_it->line_center;
	out_line_seg_it->line_direction = orgin_line_it->line_direction;
	GetLineDistMse(out_line_seg_it, out_line_seg_it->line_mse);
	//AssignSums(&out_line_seg_it->sums, &orgin_line_it->sums);
	return true;
}


bool DW_line_extract2D::AssignLinesOutput(Line2DSegOut * line_seg_out)
{
	int* non_same_line_to_line_idx = NULL; //record index  of lines  with no same line to orignal index of lines in  line_merge_out
	int *same_line_list = NULL;           //record number of lines  of each same line group

	line_seg_out->line_segs = NULL;
	line_seg_out->size = 0;
	if (same_line_group_array.group_number == 0)
	{
		//log_debug("same group is zero line_seg_out size =%d", same_line_group_array.line_size);
		line_seg_out->size = same_line_group_array.line_size;
		line_seg_out->line_segs = new LineSegItem[line_seg_out->size];

		for (unsigned int i = 0; i < line_merge_out.size; i++)
		{
			LineMergeOutputItem* orgin_line_it = &line_merge_out.lines[i];
			LineSegItem *out_line_seg_it = &line_seg_out->line_segs[i];
			AssignLine(orgin_line_it, out_line_seg_it);
			GetStartEndOfLines(&line_seg_out->line_segs[i]);
		}
		return true;
	}

	log_debug("all found line size =%d before merging", same_line_group_array.line_size);
	// find all the same line index in the same line group array
	int num_of_same_line = 0;
	same_line_list = new int[same_line_group_array.group_number];
	for (unsigned int i = 0; i < same_line_group_array.group_number; i++)
	{
		same_line_list[i] = 0;
	}
	for (unsigned int i = 0; i < same_line_group_array.line_size; i++)
	{
		unsigned int same_line_group_idx = same_line_group_array.same_line_group_idx[i];
		if (same_line_group_idx != INVALID_LINE_IDX)
		{
			same_line_list[same_line_group_idx]++;
			num_of_same_line++;
		}
	}
	// find all line who have no same line in the same line group array
	unsigned int num_of_non_same_line = same_line_group_array.line_size- num_of_same_line;
	non_same_line_to_line_idx = new int[num_of_non_same_line];
	for (unsigned int i = 0; i < num_of_non_same_line; i++)
	{
		non_same_line_to_line_idx[i] = INVALID_LINE_IDX;
	}

	//for lines who have no same line, assign conversion array  from index  of lines  to orignal index of lines
	unsigned int cnt = 0;
	for (unsigned int i = 0; i < same_line_group_array.line_size; i++)
	{
		unsigned int same_line_group_idx = same_line_group_array.same_line_group_idx[i];
		if (same_line_group_idx == INVALID_LINE_IDX)
		{
			non_same_line_to_line_idx[cnt] = i;
			cnt++;
		}
	}

	line_seg_out->size =  num_of_non_same_line + same_line_group_array.group_number;
	line_seg_out->line_segs = new LineSegItem[line_seg_out->size];

	log_debug("line size =%d after merging", line_seg_out->size);

	//initialize the line_seg_out
	for (unsigned int i = 0; i < line_seg_out->size; i++)
	{
		LineSegItem *out_line_seg_it = &line_seg_out->line_segs[i];
		out_line_seg_it->points.point_idx = NULL;
		out_line_seg_it->points.size = 0;
		out_line_seg_it->line_center = {0.f,0.f};
		out_line_seg_it->line_direction = { 0.f,0.f };
		out_line_seg_it->line_direction = { 0.f,0.f };
		out_line_seg_it->line_seg_start = { 0.f,0.f };
		out_line_seg_it->line_seg_end = { 0.f,0.f };
	}
	//step 1 assign the line who have no same line index
	for (unsigned int i = 0; i < num_of_non_same_line; i++)
	{
		int orign_line_idx = non_same_line_to_line_idx[i];
		LineMergeOutputItem* orgin_line_it = &line_merge_out.lines[orign_line_idx];
		LineSegItem *out_line_seg_it = &line_seg_out->line_segs[i];
		AssignLine(orgin_line_it, out_line_seg_it);
	}

	////step 2 merge all the same line to the new line	
	unsigned int same_line_group_idx = INVALID_LINE_IDX;
	unsigned int *point_in_same_line_cnt_list = new unsigned int[same_line_group_array.group_number];
	for (unsigned int i = 0; i < same_line_group_array.group_number; i++)
	{
		point_in_same_line_cnt_list[i] = 0;
	}

	// get the points  size in all the same line
	for (unsigned int i = 0; i < same_line_group_array.line_size; i++)
	{
		same_line_group_idx = same_line_group_array.same_line_group_idx[i];
		if (same_line_group_idx == INVALID_LINE_IDX)
			continue;
		LineMergeOutputItem* line_merge_it = &line_merge_out.lines[i];
		for (int j = 0; j < line_merge_it->pixels.size; j++) 
		{
			unsigned int pixel_in_line_idx = line_merge_it->pixels.pxl_idx[j];
			unsigned int point_in_pixel_cnt = pxl_occupied_array.pxl_item[pixel_in_line_idx].points.size;
			point_in_same_line_cnt_list[same_line_group_idx] += point_in_pixel_cnt;
		}
		point_in_same_line_cnt_list[same_line_group_idx] += line_merge_it->points.size;
	}

	for (unsigned int i = 0; i < same_line_group_array.group_number; i++)
	{
		unsigned int tmp_seg_out_line_idx = num_of_non_same_line + i;
		LineSegItem* out_line_it = &line_seg_out->line_segs[tmp_seg_out_line_idx];
		out_line_it->points.size = point_in_same_line_cnt_list[i];
		out_line_it->points.point_idx = new unsigned int[out_line_it->points.size];
		point_in_same_line_cnt_list[i] = 0;
	}

	// put all the points of the same lines ,and recompute direction and center
	std::vector <SumforCovariance2d> sums(same_line_group_array.group_number);
	for (unsigned int i = 0; i < same_line_group_array.group_number; i++)
	{
		SumsClear(&sums[i]);
	}
	//int point_in_line_idx = 0;
	for (unsigned int i = 0; i < same_line_group_array.line_size; i++)
	{
		same_line_group_idx = same_line_group_array.same_line_group_idx[i];
		if (same_line_group_idx == INVALID_LINE_IDX) continue;
		int current_line_idx = num_of_non_same_line+ same_line_group_idx; 
		LineSegItem* out_line_it = &line_seg_out->line_segs[current_line_idx]; //the new merged line is put on the end of line_seg_out
		LineMergeOutputItem* line_merge_it = &line_merge_out.lines[i];
		PushSums(&sums[same_line_group_idx], &line_merge_it->sums);
		// add points in the is_overall_merged pixels
		for (int j = 0; j < line_merge_it->pixels.size; j++) {

			unsigned int pixel_in_line_idx = line_merge_it->pixels.pxl_idx[j];
			unsigned int point_in_pixel_cnt = pxl_occupied_array.pxl_item[pixel_in_line_idx].points.size;
			for (unsigned int k = 0; k < point_in_pixel_cnt; k++) {
				unsigned int point_in_line_cnt = point_in_same_line_cnt_list[same_line_group_idx];
				unsigned int point_in_line_idx = pxl_occupied_array.pxl_item[pixel_in_line_idx].points.point_idx[k];
				out_line_it->points.point_idx[point_in_line_cnt] = point_in_line_idx;
				point_in_same_line_cnt_list[same_line_group_idx]++;
			}
		}
		// add points in non is_overall_merged pixels
		for (unsigned int j = 0; j < line_merge_it->points.size; j++) {
			unsigned int point_in_line_cnt = point_in_same_line_cnt_list[same_line_group_idx];
			unsigned int point_in_line_idx = line_merge_it->points.point_idx[j];
			out_line_it->points.point_idx[point_in_line_cnt] = point_in_line_idx;
			point_in_same_line_cnt_list[same_line_group_idx]++;
		}
	}
	// compute the start and end point of line
	for (unsigned int i = 0; i < line_seg_out->size; i++)
	{
		LineSegItem* out_line_it = &line_seg_out->line_segs[i];
		if (i >= num_of_non_same_line)
		{
			unsigned int same_line_idx = i - num_of_non_same_line;
			float eigen_mse;
			Compute2dS(out_line_it->points.size,sums[same_line_idx], out_line_it->line_direction, out_line_it->line_center, eigen_mse);
			GetLineDistMse(out_line_it, out_line_it->line_mse);
		}

		if (!GetStartEndOfLines(out_line_it))
		{
			log_error("GetStartEndOfLines fail");
		}
	}


	//release
	if (non_same_line_to_line_idx != NULL)
	{
		delete[] non_same_line_to_line_idx;
		non_same_line_to_line_idx = NULL;
	}

	if (same_line_list != NULL)
	{
		delete[] same_line_list;
		same_line_list = NULL;
	}
	if (point_in_same_line_cnt_list != NULL)
	{
		delete[] point_in_same_line_cnt_list;
		point_in_same_line_cnt_list = NULL;
	}	
	return true;
}



void DW_line_extract2D::LineMergeOutInfo(Line2DSegOut *line_seg_out)
{
	unsigned int point_in_line_idx;			// point index to input point array
	unsigned int parent_pixel_idx;				// current's parent pixel index to occupied pixel array

	std::string line_output_path = output_path;
	std::string line_output_file = "line_output.txt";
	std::ofstream line_output(line_output_path + line_output_file);
	line_output << "Total number of lines: " << line_seg_out->size << '\n';

	bool *point_being_merged = new bool[pt_points.size()];
	for (unsigned int i = 0; i < pt_points.size(); i++)
	{
		point_being_merged[i] = false;
	}

	int tatal_merged_point_cnt = 0;
	for (unsigned int i = 0; i < pxl_occupied_array.size; i++)
	{
		PixelGridItem *pxl_it = &pxl_occupied_array.pxl_item[i];

		if (pxl_it->is_overall_merged)
		{
			for (unsigned int j = 0; j < pxl_it->points.size; j++)
			{
				tatal_merged_point_cnt++;
				point_being_merged[pxl_it->points.point_idx[j]] = true;
			}
		}
		else
		{
			unsigned int bad_pixel_idx = occupied_idx_to_bad_pxl[i];
			BadPixelMergeOutpuItem * bad_pxl_it = &bad_pxl_merge_array.bad_pxl_merge_it[bad_pixel_idx];
			if (bad_pixel_idx >= pxl_occupied_array.size)
			{
				log_info("bad_pixel_idx %d is exceed the size of pxl_occupied_array = %d ", bad_pixel_idx, pxl_occupied_array.size);
				continue;
			}
			for (unsigned int j = 0; j < bad_pxl_it->pixel_point_size; j++)
			{
				if (bad_pxl_it->point_merged_flag[j])
				{
					tatal_merged_point_cnt++;
					point_being_merged[pxl_it->points.point_idx[j]] = true;
				}
			}
		}
	}

	for (unsigned int i = 0; i < line_seg_out->size; i++) {
		LineMergeOutputItem * line_merge_it = &line_merge_out.lines[i];
		//point_in_line_cnt = 0;
		for (unsigned int j = 0; j < line_merge_it->points.size; j++) {

			point_in_line_idx = line_merge_it->points.point_idx[j];
			point_being_merged[point_in_line_idx] = true;
		}
		//tatal_merged_point_cnt += line_merge_it->points.size;
	}

	line_output << "Total number of merged points: " << tatal_merged_point_cnt << '\n';
	line_output << "Total number of unmerged points: " << static_cast<int>(pt_points.size()) - tatal_merged_point_cnt << '\n';
	// get the  conversion of all  seg out line idx to merge out line idx
	std::vector<unsigned int> seg_to_merge_no_same_line_idx;  // for conversion of extactedt lines who have no same line 
	std::vector<std::vector<unsigned int>> all_seg_to_merge_same_line_idx;  // for conversion of extacted lines who have several same lines 
	std::vector<unsigned int> seg_out_same_line_pixel_size; // for record the pixel size  of seg out lines who have several same lines
	seg_to_merge_no_same_line_idx.clear();
	seg_to_merge_no_same_line_idx.shrink_to_fit();
	for (unsigned int i = 0; i < same_line_group_array.line_size; i++)
	{
		unsigned int same_line_idx = same_line_group_array.same_line_group_idx[i];
		if (same_line_idx == INVALID_LINE_IDX)
		{
			seg_to_merge_no_same_line_idx.push_back(i);
		}
	}

	unsigned int no_same_line_cnt = (unsigned int)seg_to_merge_no_same_line_idx.size();
	unsigned int same_line_cnt = line_seg_out->size - no_same_line_cnt;
	all_seg_to_merge_same_line_idx.clear();
	all_seg_to_merge_same_line_idx.resize(same_line_cnt);
	seg_out_same_line_pixel_size.clear();
	seg_out_same_line_pixel_size.resize(same_line_cnt);
	for (unsigned int i = 0; i < same_line_cnt; i++)
	{
		seg_out_same_line_pixel_size[i] = 0;
		for (unsigned int j = 0; j < same_line_group_array.line_size; j++)
		{

			unsigned int same_line_idx = same_line_group_array.same_line_group_idx[j];
			if (same_line_idx == i)
			{
				all_seg_to_merge_same_line_idx[i].push_back(j);
				unsigned int pixel_size = 0;
				LineMergeOutputItem * line_merge_it = &line_merge_out.lines[j];
				pixel_size = line_merge_it->pixels.size;
				seg_out_same_line_pixel_size[i] += pixel_size;
			}
		}
	}

	line_output << "seg out line idx: " << "merge out line idx:   " << '\n';

	for (unsigned int i = 0; i < seg_to_merge_no_same_line_idx.size(); i++)
	{
		line_output << i << " \t\t\t\t\t" << seg_to_merge_no_same_line_idx[i] << std::endl;
	}
	for (unsigned int i = 0; i < all_seg_to_merge_same_line_idx.size(); i++)
	{
		line_output << i + no_same_line_cnt << " \t\t\t\t\t";
		for (unsigned int j = 0; j < all_seg_to_merge_same_line_idx[i].size(); j++)
		{
			line_output << all_seg_to_merge_same_line_idx[i][j];
			if (j < all_seg_to_merge_same_line_idx[i].size() - 1)
			{
				line_output << ", ";
			}
		}
		line_output << std::endl;

	}

	line_output << "seg out line idx: " << "pixel size:   " <<'\n';

	for (unsigned int i = 0; i < all_seg_to_merge_same_line_idx.size(); i++)
	{
		line_output << i + no_same_line_cnt << " \t\t\t\t\t" << seg_out_same_line_pixel_size[i] << " \t\t\t";
		line_output << std::endl;
	}
	for (unsigned int i = 0; i < line_seg_out->size; i++) {

		LineSegItem* line_seg_item = &line_seg_out->line_segs[i];
		//parent_pixel_idx = line_seg_item->parent_pixel_idx;

		unsigned int pixel_size = 0;
		if (i < no_same_line_cnt)
		{
			unsigned int merge_out_line_idx = seg_to_merge_no_same_line_idx[i];
			if (merge_out_line_idx < (unsigned int)line_merge_out.size)
			{
				LineMergeOutputItem* line_merge_it = &line_merge_out.lines[merge_out_line_idx];
				pixel_size = line_merge_it->pixels.size;
				parent_pixel_idx = line_merge_it->parent_pixel_idx;
			}

		}
		else if (same_line_cnt != 0)
		{
			unsigned int same_line_idx = i - no_same_line_cnt;
			pixel_size = seg_out_same_line_pixel_size[same_line_idx];
		}

		line_output << "line Index = " << i << ", Parent pixel Index = " << parent_pixel_idx << ", pixel Count = " << pixel_size << ", Point Count = " << line_seg_item->points.size << '\n';
		if (i < no_same_line_cnt)
		{
			unsigned int merge_out_line_idx = seg_to_merge_no_same_line_idx[i];
			line_output << "line idx before merge line: " << merge_out_line_idx << std::endl;
		}
		else if (same_line_cnt != 0)
		{
			unsigned int same_line_idx = i - no_same_line_cnt;
			line_output << "line idx before merge line:  ";
			for (unsigned int j = 0; j < all_seg_to_merge_same_line_idx[same_line_idx].size(); j++)
			{
				unsigned int merge_out_line_idx = all_seg_to_merge_same_line_idx[same_line_idx][j];
				line_output << merge_out_line_idx;
				if (j < all_seg_to_merge_same_line_idx[same_line_idx].size() - 1)
				{
					line_output << ", ";
				}
			}
			line_output << std::endl;
			line_output << std::endl;
		}

		line_output << std::setprecision(10) << std::fixed;
		line_output << i << "  line mse       \t" << line_seg_item->line_mse << '\n';
		line_output << i << "  line center    \t"  << line_seg_item->line_center.x << '\t' << line_seg_item->line_center.y << '\t' << '\n';
		line_output << i << "  line direction \t" << line_seg_item->line_direction.x << '\t' << line_seg_item->line_direction.y << '\t' << '\n';
		line_output << i << "  line start     \t" << line_seg_item->line_seg_start.x << '\t' << line_seg_item->line_seg_start.y << '\t' << '\n';
		line_output << i << "  line end		  \t" << line_seg_item->line_seg_end.x << '\t' << line_seg_item->line_seg_end.y << '\t' << '\n';
	}
	line_output.close();
	delete[] point_being_merged;
	seg_to_merge_no_same_line_idx.clear();
	seg_to_merge_no_same_line_idx.shrink_to_fit();
	for (unsigned int i = 0; i < all_seg_to_merge_same_line_idx.size(); i++)
	{
		all_seg_to_merge_same_line_idx[i].clear();
		all_seg_to_merge_same_line_idx[i].shrink_to_fit();
	}
	all_seg_to_merge_same_line_idx.clear();
	all_seg_to_merge_same_line_idx.shrink_to_fit();
	seg_out_same_line_pixel_size.clear();
	seg_out_same_line_pixel_size.shrink_to_fit();
}

void DW_line_extract2D::MissingPointsOutput(Line2DSegOut *line_seg_out)
{
	bool *point_being_merged = new bool[pt_points.size()];
	for (unsigned int i = 0; i < pt_points.size(); i++)
	{
		point_being_merged[i] = false;
	}

	int cnt = 0;
	for (unsigned int i = 0; i < line_seg_out->size; i++)
	{

		//point_in_plane_cnt = 0;
		for (unsigned int j = 0; j < line_seg_out->line_segs[i].points.size; j++) {

			unsigned int point_in_line_idx = line_seg_out->line_segs[i].points.point_idx[j];
			point_being_merged[point_in_line_idx] = true;
			cnt++;
		}
	}

	log_info("pt_points size =%d merging point cnt = %d", pt_points.size(),cnt);
	std::string missing_points_path = output_path;
	std::string missing_points_info_file = "line2d_missing_points_info.txt";
	std::ofstream missing_points_info(missing_points_path + missing_points_info_file);

	if (cnt == 0)
	{
		missing_points_info << "no missing points" << std::endl;
		missing_points_info.close();
		return;
	}

	missing_points_info << std::setprecision(6) << std::fixed;
	for (unsigned int i = 0; i < pt_points.size(); i++)
	{

		if (!point_being_merged[i]) 
		{
			unsigned long long grid_index;
			ConvertPointToPixelID(min_p, pt_points[i], pixel_size_, pixel_dim, grid_index);
			unsigned int pixel_index = grid_to_occupied_pxl_idx[grid_index];
			missing_points_info << "Pixel Index = " << pixel_index << ", Point Index = " << i << ", [" << pt_points[i].x;
			missing_points_info << ", " << pt_points[i].y;
			missing_points_info << "]"<<std::endl;
		}
	}
	missing_points_info.close();

	missing_points_path = output_path + "line_xyz\\";
	if (IOData::createDirectory(missing_points_path))
	{
		log_error("createDirectory %s failed", missing_points_path.c_str());
		return;
	}
	log_debug("createDirectory %s success", missing_points_path.c_str());

	std::string missing_points_file = "missing_points_2d.txt";

	std::ofstream missing_points(missing_points_path + missing_points_file);

	cnt = 0;
	missing_points << std::setprecision(6) << std::fixed;
	for (unsigned int i = 0; i < pt_points.size(); i++)
	{
		if (!point_being_merged[i]) {

			missing_points << pt_points[i].x << ";";
			missing_points << pt_points[i].y << ";";
			missing_points << 0 << '\n';
			cnt++;
		}
	}
	log_info(" 00 pt_points size =%d missing point cnt = %d", pt_points.size(), cnt);

	missing_points.close();
	delete[] point_being_merged;
}

bool DW_line_extract2D::SetConfigure(const SysCntrlParams sys_control_para, const LineExtrCntrlParams config_params,\
	const LineFitThresholds line_seg_thrshld, const PixelSize pixel_size)
{
	sys_control_para_ = sys_control_para;
	config_params_ = config_params;
	line_seg_thrshld_ = line_seg_thrshld;
	pixel_size_ = pixel_size;
	return true;
}


#ifdef SAVE_OUTPUT_FILE_DEBUG
bool DW_line_extract2D::SetConfigure(const SysCntrlParams sys_control_para, const LineExtrCntrlParams config_params, \
	const LineFitThresholds line_seg_thrshld, const PixelSize pixel_size, const Line2DDebugParams getLineDebugConfig)
{
	sys_control_para_ = sys_control_para;
	config_params_ = config_params;
	line_seg_thrshld_ = line_seg_thrshld;
	pixel_size_ = pixel_size;
	line2d_debug_params_ = getLineDebugConfig;
	return true;
}
#endif
bool DW_line_extract2D::SavePixel(std::string path, int pixel_idx)
{
	std::string folder = path;
	std::stringstream pixel_id;
	pixel_id << pixel_idx;
	std::string file_name = folder+"pxl_xyz_"+ pixel_id.str()+".txt";
	std::vector<cv::Point2f> points;
	PixelGridItem *pxl_it = &pxl_occupied_array.pxl_item[pixel_idx];
	points.resize(pxl_it->points.size);
	for (unsigned int i = 0; i < pxl_it->points.size; i++)
	{
		points[i] = pt_points[pxl_it->points.point_idx[i]];
	}
	FeaturesIO::SavePoint2fDataWithDelimiter(file_name,";", points);
	return true;
}

bool DW_line_extract2D::SaveLines(std::string path, int line_idx)
{
	if (line_idx > static_cast<int>(line_merge_out.size))
	{
		log_error("SaveLines input line_idx =%d exceed sizeof line =%d", line_idx, line_merge_out.size);
		return false;
	}

	if (IOData::createDirectory(path))
	{
		log_error("SaveLines create path =%d return error", path.c_str());
		return false;
	}

	LineMergeOutputItem* line_it = &line_merge_out.lines[line_idx];
	for (int i = 0; i < line_it->pixels.size; i++)
	{
		SavePixel(path,line_it->pixels.pxl_idx[i]);
	}
	std::stringstream line_id;
	line_id << line_idx;
	std::string file_name = path + "line_xyz_" + line_id.str() + ".txt";
	Point2fArray lines_points;
	lines_points.resize(line_it->points.size);
	for (unsigned int i = 0; i < line_it->points.size; i++)
	{
		lines_points[i] = pt_points[line_it->points.point_idx[i]];
	}
	FeaturesIO::SavePoint2fDataWithDelimiter(file_name," ", lines_points);
	return true;
}

bool DW_line_extract2D::SaveLinesInfo(void)
{
	for (unsigned int i = 0; i < line_merge_out.size; i++)
	{
		std::stringstream line_id;
		LineMergeOutputItem* line_it = &line_merge_out.lines[i];
		line_id << i;
		std::string folder = output_path + "t_line_dbg_" + line_id.str() + "\\";
		SaveLines(folder, i);
	}
	return true;

}



bool DW_line_extract2D::Identify2PixelCLoseByDistance(
	const std::vector<Point2f> input_data, 
	const unsigned int first_pixel, 
	const unsigned int second_pixel, 
	const float distance)
{
	PixelGridItem *first_pxl_it = &pxl_occupied_array.pxl_item[first_pixel];
	PixelGridItem *sec_pxl_it = &pxl_occupied_array.pxl_item[second_pixel];

	if ((first_pxl_it->points.size < 1) || (sec_pxl_it->points.size < 1))
		return false;
	std::vector<PointIdx2Dist> first_idx_to_dist(first_pxl_it->points.size);
	for (size_t i = 0; i < first_pxl_it->points.size; i++)
	{
		Point2f point = input_data[first_pxl_it->points.point_idx[i]];
		first_idx_to_dist[i].cloud_point_idx = first_pxl_it->points.point_idx[i];
		//Point2f delta_point = point - sec_pxl_it->center;
		//float distance = std::fabs(delta_point.x) + std::fabs(delta_point.y);
		float distance = Util_Math::DWComputePointToPointDist2D<Point2f>(point, sec_pxl_it->center);
		first_idx_to_dist[i].delta_dist = distance;
	}

	std::sort(first_idx_to_dist.begin(), first_idx_to_dist.end(), std::less<PointIdx2Dist>{});
	std::vector<PointIdx2Dist> sec_idx_to_dist(sec_pxl_it->points.size);
	for (size_t i = 0; i < sec_pxl_it->points.size; i++)
	{
		Point2f point = input_data[sec_pxl_it->points.point_idx[i]];
		sec_idx_to_dist[i].cloud_point_idx = sec_pxl_it->points.point_idx[i];
		//Point2f delta_point = point - first_pxl_it->center;
		//float distance = std::fabs(delta_point.x) + std::fabs(delta_point.y);
		float distance = Util_Math::DWComputePointToPointDist2D<Point2f>(point, first_pxl_it->center);
		sec_idx_to_dist[i].delta_dist = distance;
		
	}
	std::sort(sec_idx_to_dist.begin(), sec_idx_to_dist.end(), std::less<PointIdx2Dist>{});

	float dist1 = DWComputePointToPointDist2D<Point2f>(input_data[first_idx_to_dist[0].cloud_point_idx], 
		                                               input_data[sec_idx_to_dist[0].cloud_point_idx]);
	float min_dist = 0;
	if ((first_pxl_it->points.size < 2) || (sec_pxl_it->points.size < 2))
	{
		min_dist = dist1;
	}
	else
	{
		int first_comp_cnt = 2;
		int sec_comp_cnt = 2;
		Point2f first_comp_center, sec_comp_center;

		first_comp_center = { 0.f,0.f };
		for (unsigned int i = 0; i < static_cast<unsigned int>(first_comp_cnt); i++)
		{
			first_comp_center += input_data[first_idx_to_dist[i].cloud_point_idx];
		}

		first_comp_center = first_comp_center / first_comp_cnt;
		sec_comp_center = { 0.f,0.f };
		for (unsigned int i = 0; i < static_cast<unsigned int>(sec_comp_cnt); i++)
		{
			sec_comp_center += input_data[sec_idx_to_dist[i].cloud_point_idx];
		}

		sec_comp_center = sec_comp_center / sec_comp_cnt;
		float dist2 = DWComputePointToPointDist2D<Point2f>(first_comp_center, sec_comp_center);
		min_dist = dist1 < dist2 ? dist1 : dist2;
	}
	//if (first_pixel == 12) log_info("first_pixel %d second_pixel =%d dist1 =%f dist2= %f ", first_pixel, second_pixel, dist1, dist2);
	bool rtn = false;
	if (min_dist < distance) 
		rtn = true;

	return rtn;
}

void DW_line_extract2D::debug_point_idx()
{
	for (unsigned int i = 0; i < pxl_occupied_array.size; i++)
	{
		PixelGridItem *pxl_it = &pxl_occupied_array.pxl_item[i];
		for (unsigned int j = 0; j < pxl_it->points.size; j++)
		{
			unsigned int point_idx = pxl_it->points.point_idx[j];
			if (point_idx >= pt_points.size())
			{
				log_debug("pxl %d idx %d point indx%d is exceed max =%d",i,j, point_idx, pt_points.size());
			}

		}
	}
}


//( compare result with ransac by time and result shape?)
bool DW_line_extract2D::FitLine(std::vector<Point2f> input_data, Line2DSegOut *line_out)
{
	pt_points = input_data;
	
	inverse_length_x_of_pixel = 1 /pixel_size_.length_x_of_pixel;
	inverse_length_y_of_pixel = 1 / pixel_size_.length_y_of_pixel;

	line_seg_thrshld_.min_connected_dist_2pxl = (pixel_size_.length_x_of_pixel + pixel_size_.length_y_of_pixel) / 4;	
	//line_seg_thrshld_.min_connected_dist_2pxl = line_seg_thrshld_.min_line_dist_pt2pixel*2;

	//get maxmin value of input data	
	GetMinMaxXY(input_data, max_p, min_p);
	//log_info("max_p[x,y] = [%f,%f]", max_p.x, max_p.y);
	//log_info("min_p[x,y] = [%f,%f]", min_p.x, min_p.y);
	
	// pixels grid initlized
	GetTotalNumOfPixelGrid(max_p, min_p, pixel_size_, pixel_dim);
	//log_info("total_num_of_pxl_grid =%d ",total_num_of_pxl_grid);
	//log_info("pixel_dim[cols_of_pixel,rows_of_pixel] = [%d,%d]", pixel_dim.cols_of_pixel, pixel_dim.rows_of_pixel);

	// create occupied grid pixels in 2d space
	CreateOccupiedPixels();
	//log_info("pxl_occupied_array size=%d", pxl_occupied_array.size);

	// compute pixels line direction, mse, is_good_pixel and neighbour info such as normal differnce ,line dist ,neighbour flag etc.
	GetOccupiedPixelsNeighbor();
#if 0
	TIMING_BEGIN(TP1);
	GetpixelNeigbourDebugInfo(ALL_PIXEL_DEBUG);
	TIMING_END_ms("GetpixelNeigbourDebugInfo", TP1);

	for (int i = 0; i < pxl_occupied_array.size; i++)
	{
		SavePixel(output_path,i);
	}
#endif
	// identify parent pixels in good pixels
	IdentifyParentOfGoodPixels();

	// merge Good pixels
	MergeGoodPixels();
	//log_info("line_merge_out size =%d", line_merge_out.size);

	IdentifyParentOfbadPixels();

	// merge bad pixels
	MergeBadPixels();	

	if (line2d_debug_params_.neighbour_info_debug)
	{
		GetpixelNeigbourDebugInfo(ALL_PIXEL_DEBUG);
	}
#ifdef SAVE_OUTPUT_FILE_DEBUG
	//SaveLinesInfo(); ///Tadd
#endif

	//merge all the same lines into one line
	MergeLines();
	//assign all lines output, line xyz , line center , line direction, line start point, line end point
	AssignLinesOutput(line_out);

	//log_info("dbg_section_valid =%d line_output_debug =%d missing_point_output= %d", line2d_debug_params_.dbg_section_valid, line2d_debug_params_.line_output_debug, line2d_debug_params_.missing_point_output);
	if (line2d_debug_params_.line_output_debug)
	{
		LineMergeOutInfo(line_out);
	}

	if (line2d_debug_params_.missing_point_output)
	{
		MissingPointsOutput(line_out);
	}

	return true;
}

