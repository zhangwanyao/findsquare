#ifndef MATHOPERATION_HPP
#define MATHOPERATION_HPP

#include "DataStruct.h"
#include "log.h"
#include "util_math.hpp"
#include "DownSampleFilter.hpp"
#include "VoxelNormals.h"
#include "../Voxel.h"


using namespace Util_Math;

namespace MathOperation {
	// get mse of the points array
	static inline bool GetPointsArrayMse(const PointArray pt_cloud_xyz, const PointInVoxelArray point_array, const ModuleStruct::Point3f plane_normal, const ModuleStruct::Point3f plane_center, float mse_threshold, float& dist_mse, float& plane_high_mse_ratio) {
		unsigned int plane_high_mse_points_cnt = 0;
		IntermediateType average_distance = 0.0;
		float high_threshold = mse_threshold;
		float plane_dist = 0.f;
		ModuleStruct::Point3f point, point1, point2;
		//float tmp = sum<float>(point1, point2,point2);
		for (unsigned int i = 0; i < point_array.size; i++)
		{
			point = pt_cloud_xyz.points[point_array.point_idx[i]];
			plane_dist = ComputePointToPlaneDist<float, ModuleStruct::Point3f>(point, plane_normal, plane_center);
			if (plane_dist > high_threshold) plane_high_mse_points_cnt++;
			average_distance += plane_dist;
		}
		dist_mse = (float)(average_distance / point_array.size);
		plane_high_mse_ratio = float(plane_high_mse_points_cnt * 1.0f / point_array.size);
		return true;
	}

	static inline bool GetPointsArrayMse(const std::vector<ModuleStruct::Point3f> point_array, const ModuleStruct::Point3f plane_normal, const ModuleStruct::Point3f plane_center, const float mse_threshold, float& dist_mse, float& plane_high_mse_ratio) {
		unsigned int plane_high_mse_points_cnt = 0;
		IntermediateType average_distance = 0.0;
		float high_threshold = mse_threshold;
		float plane_dist = 0.f;
		ModuleStruct::Point3f point;
		for (unsigned int i = 0; i < point_array.size(); i++)
		{
			point = point_array[i];
			plane_dist = ComputePointToPlaneDist<float, ModuleStruct::Point3f>(point, plane_normal, plane_center);
			if (plane_dist > high_threshold) plane_high_mse_points_cnt++;
			average_distance += plane_dist;
		}
		dist_mse = (float)(average_distance / point_array.size());
		plane_high_mse_ratio = float(plane_high_mse_points_cnt * 1.0f / point_array.size());
		return true;
	}

	static inline bool GetPointsArrayWeightedMse(const std::vector<ModuleStruct::Point3f> point_array, const ModuleStruct::Point3f plane_normal, ModuleStruct::Point3f const plane_center, const float mse_threshold, float& dist_mse, float& plane_high_mse_ratio) {
		unsigned int plane_high_mse_points_cnt = 0;
		IntermediateType average_distance = 0.0;
		IntermediateType total_center2point_distance = 0.0;
		float center_dist = 0.0f;
		float high_threshold = mse_threshold;
		float plane_dist = 0.f;
		ModuleStruct::Point3f point;
		for (unsigned int i = 0; i < point_array.size(); i++)
		{
			point = point_array[i];
			plane_dist = ComputePointToPlaneDist<float, ModuleStruct::Point3f>(point, plane_normal, plane_center);
			center_dist = ComputePointToPointDist<float, ModuleStruct::Point3f>(point, plane_center);
			total_center2point_distance += center_dist;
			if (plane_dist > high_threshold) plane_high_mse_points_cnt++;
			average_distance += plane_dist * center_dist;
		}
		dist_mse = (float)(average_distance / total_center2point_distance);
		plane_high_mse_ratio = float(plane_high_mse_points_cnt * 1.0f / point_array.size());
		return true;
	}

	static inline bool GetPointsArrayWeightedMse(const PointArray pt_cloud_xyz, const PointInVoxelArray point_array, const ModuleStruct::Point3f plane_normal, const ModuleStruct::Point3f plane_center, const float mse_threshold, float& dist_mse, float& plane_high_mse_ratio) {
		unsigned int plane_high_mse_points_cnt = 0;
		IntermediateType average_distance = 0.0;
		IntermediateType total_center2point_distance = 0.0;
		float center_dist = 0.0f;
		float high_threshold = mse_threshold;
		float plane_dist = 0.f;
		ModuleStruct::Point3f point;
		for (unsigned int i = 0; i < point_array.size; i++)
		{
			point = pt_cloud_xyz.points[point_array.point_idx[i]];
			plane_dist = ComputePointToPlaneDist<float, ModuleStruct::Point3f>(point, plane_normal, plane_center);
			center_dist = ComputePointToPointDist<float, ModuleStruct::Point3f>(point, plane_center);
			total_center2point_distance += center_dist;
			if (plane_dist > high_threshold) plane_high_mse_points_cnt++;
			average_distance += plane_dist * center_dist;
		}
		dist_mse = (float)(average_distance / total_center2point_distance);
		plane_high_mse_ratio = float(plane_high_mse_points_cnt * 1.0f / point_array.size);
		return true;
	}

	static inline void BinarySortDown(unsigned int topk_array[], unsigned int topk, unsigned int root) {
		unsigned int left_leaf;   // left leaf of the current root
		unsigned int right_leaf;  // right leaf of the current root
		unsigned int min_value_idx;      // the index of elments whose value in min

		while (1)
		{
			left_leaf = 2 * root + 1;
			// check if have the left leaf in the tree
			min_value_idx = left_leaf;
			if (left_leaf >= topk)
			{
				return;
			}

			right_leaf = 2 * root + 2;
			if (right_leaf < topk)
			{
				// find the min in left and right leaf
				min_value_idx = topk_array[right_leaf] < topk_array[left_leaf] ? right_leaf : left_leaf;
			}

			if (topk_array[root] <= topk_array[min_value_idx])
			{
				return;
			}

			//swap
			int t = topk_array[root];
			topk_array[root] = topk_array[min_value_idx];
			topk_array[min_value_idx] = t;
			root = min_value_idx;
		}
	}

	// find the top k elements  in the array , k must > = 2 , top_idx <= topk is real top array
	static inline bool FindTopK(unsigned int* data_array, bool* is_labeled, unsigned int array_size, unsigned int* topk_array, unsigned int topk, unsigned int& top_idx) {
		if (topk < 2)
		{
			return false;
		}

		unsigned int array_idx = 0;
		unsigned int tmp_top_idx = 0;
		for (tmp_top_idx = 0, array_idx = 0; (tmp_top_idx < topk) && (array_idx < array_size); array_idx++)
		{
			if (!is_labeled[array_idx])
			{
				topk_array[tmp_top_idx] = data_array[array_idx];
				tmp_top_idx++;
			}
		}

		if (tmp_top_idx < 2)
		{
			top_idx = 0;
			//log_debug(" too few elements %d in array", tmp_top_idx);
			return false;
		}

		for (int i = (tmp_top_idx - 2) / 2; i >= 0; i--)
		{
			BinarySortDown(topk_array, tmp_top_idx, i);   // sort the topk array by binary sort tree
		}

		for (unsigned int i = array_idx; i < array_size; i++)
		{
			if (is_labeled[i]) continue;
			if (data_array[i] > topk_array[0])
			{
				topk_array[0] = data_array[i];
				BinarySortDown(topk_array, tmp_top_idx, 0);
			}
		}

		top_idx = tmp_top_idx;
		if (top_idx < topk)
		{
			for (unsigned int i = top_idx; i < topk; i++)
			{
				topk_array[i] = (std::numeric_limits<unsigned int>::max)();
			}
		}
		return true;
	}

	static inline bool FindTopK(unsigned int* data_array, unsigned int array_size, unsigned int* topk_array, unsigned int topk, unsigned int& top_idx) {
		if (topk < 2)
		{
			log_fatal("input error top k  =%d array_size =%d ", topk, array_size);
			return false;
		}

		unsigned int array_idx = 0;
		unsigned int tmp_top_idx = 0;
		for (tmp_top_idx = 0, array_idx = 0; (tmp_top_idx < topk) && (array_idx < array_size); array_idx++)
		{
			topk_array[tmp_top_idx] = data_array[array_idx];
			tmp_top_idx++;
		}

		if (tmp_top_idx < 1)
		{
			top_idx = 0;
			//log_debug(" too few elements %d in array", tmp_top_idx);
			return false;
		}

		for (int i = (tmp_top_idx - 2) / 2; i >= 0; i--)
		{
			BinarySortDown(topk_array, tmp_top_idx, i);   // sort the topk array by binary sort tree
		}

		for (unsigned int i = array_idx; i < array_size; i++)
		{
			if (data_array[i] > topk_array[0])
			{
				topk_array[0] = data_array[i];
				BinarySortDown(topk_array, tmp_top_idx, 0);
			}
		}

		top_idx = tmp_top_idx;
		if (top_idx < topk)
		{
			for (unsigned int i = top_idx; i < topk; i++)
			{
				topk_array[i] = (std::numeric_limits<unsigned int>::max)();
			}
		}
		return true;
	}

	static inline bool FindMax(unsigned int* data_array, bool* is_labeled, unsigned int array_size, unsigned int& max) {
		unsigned tmp_max = 0;
		for (unsigned int i = 0; i < array_size; i++)
		{
			if (!is_labeled[i])
			{
				if (data_array[i] > tmp_max)
				{
					tmp_max = data_array[i];
				}
			}
		}
		max = tmp_max;
		return  true;
	}

	static inline bool FindMax(unsigned int* data_array, unsigned int array_size, unsigned int& max) {
		unsigned tmp_max = 0;
		for (unsigned int i = 0; i < array_size; i++)
		{
			if (data_array[i] > tmp_max)
			{
				tmp_max = data_array[i];
			}
		}
		max = tmp_max;
		return  true;
	}

	static inline bool CompareReverseOrderFunc(int i, int j) { return (i > j); }

	static inline bool VectorNormalized(ModuleStruct::Point3f vecrtor, ModuleStruct::Point3f& normalized_vector) {
		double sqrt_value = std::sqrt(vecrtor.x * vecrtor.x + vecrtor.y * vecrtor.y + vecrtor.z * vecrtor.z);

		if (sqrt_value == 0) return false;
		normalized_vector.x = float(vecrtor.x / sqrt_value);
		normalized_vector.y = float(vecrtor.y / sqrt_value);
		normalized_vector.z = float(vecrtor.z / sqrt_value);

		return true;
	}

	static inline bool GetPointsCenter(SumforCovariance* points_sums, unsigned int points_size, ModuleStruct::Point3f& points_center) {
		if (points_size <= 0) return false;

		double point_num_inverse = (double)1.0 / points_size;

		//plane center
		points_center.x = (float)(points_sums->sum_x * point_num_inverse);
		points_center.y = (float)(points_sums->sum_y * point_num_inverse);
		points_center.z = (float)(points_sums->sum_z * point_num_inverse);
		return true;
	}

	static inline bool ClearSums(SumforCovariance* sums) {
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

	static inline bool AssignSums(SumforCovariance* sums_to, SumforCovariance* sums_from) {
		sums_to->sum_x = sums_from->sum_x; sums_to->sum_y = sums_from->sum_y; sums_to->sum_z = sums_from->sum_z;
		sums_to->sum_xx = sums_from->sum_xx; sums_to->sum_yy = sums_from->sum_yy; sums_to->sum_zz = sums_from->sum_zz;
		sums_to->sum_xy = sums_from->sum_xy; sums_to->sum_xz = sums_from->sum_xz; sums_to->sum_yz = sums_from->sum_yz;
		return true;
	}

	static inline bool PushSums(SumforCovariance* sums_to, SumforCovariance* sums_from) {
		sums_to->sum_x += sums_from->sum_x; sums_to->sum_y += sums_from->sum_y; sums_to->sum_z += sums_from->sum_z;
		sums_to->sum_xx += sums_from->sum_xx; sums_to->sum_yy += sums_from->sum_yy; sums_to->sum_zz += sums_from->sum_zz;
		sums_to->sum_xy += sums_from->sum_xy; sums_to->sum_xz += sums_from->sum_xz; sums_to->sum_yz += sums_from->sum_yz;
		return true;
	}

	template<typename T>
	static inline bool PushPoint(SumforCovariance* sums_to, T point) {
		sums_to->sum_x += point.x; sums_to->sum_y += point.y; sums_to->sum_z += point.z;
		sums_to->sum_xx += point.x * point.x; sums_to->sum_yy += point.y * point.y; sums_to->sum_zz += point.z * point.z;
		sums_to->sum_xy += point.x * point.y; sums_to->sum_xz += point.x * point.z; sums_to->sum_yz += point.y * point.z;
		return true;
	}

	template<typename T>
	static inline bool PopPoint(SumforCovariance* sums_to, T point) {
		sums_to->sum_x -= point.x; sums_to->sum_y -= point.y; sums_to->sum_z -= point.z;
		sums_to->sum_xx -= point.x * point.x; sums_to->sum_yy -= point.y * point.y; sums_to->sum_zz -= point.z * point.z;
		sums_to->sum_xy -= point.x * point.y; sums_to->sum_xz -= point.x * point.z; sums_to->sum_yz -= point.y * point.z;
		return true;
	}

	static inline bool GetMinMaxXYZ(const PointArray input_data, const PointInVoxelArray point_idx_array, ModuleStruct::Point3f& max_p, ModuleStruct::Point3f& min_p) {
		if (input_data.size == 0) {
			return false;
		}
		min_p.x = min_p.y = min_p.z = std::numeric_limits<float>::infinity();
		max_p.x = max_p.y = max_p.z = -std::numeric_limits<float>::infinity();

		for (unsigned int i = 0; i < point_idx_array.size; i++) {
			ModuleStruct::Point3f point = input_data.points[point_idx_array.point_idx[i]];
			min_p.x = (min_p.x > point.x) ? point.x : min_p.x;
			max_p.x = (max_p.x < point.x) ? point.x : max_p.x;
			min_p.y = (min_p.y > point.y) ? point.y : min_p.y;
			max_p.y = (max_p.y < point.y) ? point.y : max_p.y;
			min_p.z = (min_p.z > point.z) ? point.z : min_p.z;
			max_p.z = (max_p.z < point.z) ? point.z : max_p.z;
		}
		return true;
	}

	static inline bool GetMinMaxXYZ(const PointArray input_data, ModuleStruct::Point3f& max_p, ModuleStruct::Point3f& min_p) {
		if (input_data.size == 0) {
			return false;
		}
		min_p.x = min_p.y = min_p.z = std::numeric_limits<float>::infinity();
		max_p.x = max_p.y = max_p.z = -std::numeric_limits<float>::infinity();

		for (unsigned int i = 0; i < input_data.size; i++) {
			ModuleStruct::Point3f point = input_data.points[i];
			min_p.x = (min_p.x > point.x) ? point.x : min_p.x;
			max_p.x = (max_p.x < point.x) ? point.x : max_p.x;
			min_p.y = (min_p.y > point.y) ? point.y : min_p.y;
			max_p.y = (max_p.y < point.y) ? point.y : max_p.y;
			min_p.z = (min_p.z > point.z) ? point.z : min_p.z;
			max_p.z = (max_p.z < point.z) ? point.z : max_p.z;
		}
		return true;
	}

	static inline bool GetMinMaxXYZ(const std::vector<ModuleStruct::Point3f>input_data, ModuleStruct::Point3f& max_p, ModuleStruct::Point3f& min_p)
	{
		if (input_data.size() == 0) {
			log_error("empty input");
			return false;
		}
		min_p.x = min_p.y = min_p.z = std::numeric_limits<float>::infinity();
		max_p.x = max_p.y = max_p.z = -std::numeric_limits<float>::infinity();

		for (unsigned int i = 0; i < input_data.size(); i++) {
			ModuleStruct::Point3f point = input_data[i];
			min_p.x = (min_p.x > point.x) ? point.x : min_p.x;
			max_p.x = (max_p.x < point.x) ? point.x : max_p.x;
			min_p.y = (min_p.y > point.y) ? point.y : min_p.y;
			max_p.y = (max_p.y < point.y) ? point.y : max_p.y;
			min_p.z = (min_p.z > point.z) ? point.z : min_p.z;
			max_p.z = (max_p.z < point.z) ? point.z : max_p.z;
		}
		return true;
	}

	static inline bool GetMinMaxXYZ(const std::vector<ModuleStruct::Point3f>input_data, PointInVoxelArray input_ids, ModuleStruct::Point3f& max_p, ModuleStruct::Point3f& min_p)
	{
		if ((input_data.size() == 0) || (input_ids.size == 0)) {
			log_error("empty input");
			return false;
		}

		min_p.x = min_p.y = min_p.z = std::numeric_limits<float>::infinity();
		max_p.x = max_p.y = max_p.z = -std::numeric_limits<float>::infinity();

		for (unsigned int i = 0; i < input_ids.size; i++) {
			ModuleStruct::Point3f point = input_data[input_ids.point_idx[i]];
			min_p.x = (min_p.x > point.x) ? point.x : min_p.x;
			max_p.x = (max_p.x < point.x) ? point.x : max_p.x;
			min_p.y = (min_p.y > point.y) ? point.y : min_p.y;
			max_p.y = (max_p.y < point.y) ? point.y : max_p.y;
			min_p.z = (min_p.z > point.z) ? point.z : min_p.z;
			max_p.z = (max_p.z < point.z) ? point.z : max_p.z;
		}
		return true;
	}

	static inline bool GetMinMaxXYZ(const std::vector<ModuleStruct::Point3f>& points, float* minmax_xyz)
	{
		if (points.empty())
			return false;

		//float output_minmax_xyz[6];
		minmax_xyz[0] = std::numeric_limits<float>::infinity();		minmax_xyz[1] = -std::numeric_limits<float>::infinity();
		minmax_xyz[2] = std::numeric_limits<float>::infinity();		minmax_xyz[3] = -std::numeric_limits<float>::infinity();
		minmax_xyz[4] = std::numeric_limits<float>::infinity();		minmax_xyz[5] = -std::numeric_limits<float>::infinity();

		for (int i = 0; i < points.size(); i++)
		{
			minmax_xyz[0] = (points[i].x < minmax_xyz[0]) ? points[i].x : minmax_xyz[0];
			minmax_xyz[1] = (points[i].x > minmax_xyz[1]) ? points[i].x : minmax_xyz[1];

			minmax_xyz[2] = (points[i].y < minmax_xyz[2]) ? points[i].y : minmax_xyz[2];
			minmax_xyz[3] = (points[i].y > minmax_xyz[3]) ? points[i].y : minmax_xyz[3];

			minmax_xyz[4] = (points[i].z < minmax_xyz[4]) ? points[i].z : minmax_xyz[4];
			minmax_xyz[5] = (points[i].z > minmax_xyz[5]) ? points[i].z : minmax_xyz[5];
		}

		return true;
	}

	static inline bool GetTotalNumOfVoxelGrid(const ModuleStruct::Point3f max_p, const ModuleStruct::Point3f min_p, const VoxelParams voxel_param, VoxelDimension& voxel_dim)
	{
		float max_x = max_p.x;
		float max_y = max_p.y;
		float max_z = max_p.z;

		float min_x = min_p.x;
		float min_y = min_p.y;
		float min_z = min_p.z;

		float length_x_of_voxel_inverse = 1.0f / voxel_param.length_x_of_voxel;
		float length_y_of_voxel_inverse = 1.0f / voxel_param.length_y_of_voxel;
		float length_z_of_voxel_inverse = 1.0f / voxel_param.length_z_of_voxel;

		if ((max_x - min_x) != 0 && std::remainder((max_x - min_x), voxel_param.length_x_of_voxel) == 0)
			voxel_dim.cols_of_voxel = (unsigned int)(std::floor((max_x - min_x) * length_x_of_voxel_inverse));
		else
			voxel_dim.cols_of_voxel = (unsigned int)(std::floor((max_x - min_x) * length_x_of_voxel_inverse) + 1);

		if ((max_y - min_y) != 0 && std::remainder((max_y - min_y), voxel_param.length_y_of_voxel) == 0)
			voxel_dim.rows_of_voxel = (unsigned int)(std::floor((max_y - min_y) * length_y_of_voxel_inverse));
		else
			voxel_dim.rows_of_voxel = (unsigned int)(std::floor((max_y - min_y) * length_y_of_voxel_inverse) + 1);

		if ((max_z - min_z) != 0 && std::remainder((max_z - min_z), voxel_param.length_z_of_voxel) == 0)
			voxel_dim.depths_of_voxel = (unsigned int)(std::floor((max_z - min_z) * length_z_of_voxel_inverse));
		else
			voxel_dim.depths_of_voxel = (unsigned int)(std::floor((max_z - min_z) * length_z_of_voxel_inverse) + 1);
		return true;
	}

	static inline bool ConvertXYZToVoxelID(const ModuleStruct::Point3f min_p, const ModuleStruct::Point3f Point, const VoxelParams voxel_param, const VoxelDimension voxel_dim, unsigned long long& grid_id) {
		float length_x_of_voxel_inverse = 1.0f / voxel_param.length_x_of_voxel;
		float length_y_of_voxel_inverse = 1.0f / voxel_param.length_y_of_voxel;
		float length_z_of_voxel_inverse = 1.0f / voxel_param.length_z_of_voxel;

		unsigned int col_idx = (unsigned int)(std::floor((Point.x - min_p.x) * length_x_of_voxel_inverse));
		unsigned int row_idx = (unsigned int)(std::floor((Point.y - min_p.y) * length_y_of_voxel_inverse));
		unsigned int height_idx = (unsigned int)(std::floor((Point.z - min_p.z) * length_z_of_voxel_inverse));

		if (col_idx == voxel_dim.cols_of_voxel)
			col_idx--;
		if (row_idx == voxel_dim.rows_of_voxel)
			row_idx--;
		if (height_idx == voxel_dim.depths_of_voxel)
			height_idx--;

		grid_id = height_idx * (voxel_dim.cols_of_voxel * voxel_dim.rows_of_voxel) + row_idx * voxel_dim.cols_of_voxel + col_idx;
		return true;
	}

	static inline void ConvertVoxelIDTo3dID(const ModuleStruct::Point3f min_p, const ModuleStruct::Point3f& pt_xyz, const VoxelParams voxel_param, const VoxelDimension voxel_dim, int& row_idx, int& col_idx, int& depth_idx) {
		float length_x_of_voxel_inverse = 1.0f / voxel_param.length_x_of_voxel;
		float length_y_of_voxel_inverse = 1.0f / voxel_param.length_y_of_voxel;
		float length_z_of_voxel_inverse = 1.0f / voxel_param.length_z_of_voxel;

		col_idx = (int)(std::floor((pt_xyz.x - min_p.x) * length_x_of_voxel_inverse));
		row_idx = (int)(std::floor((pt_xyz.y - min_p.y) * length_y_of_voxel_inverse));
		depth_idx = (int)(std::floor((pt_xyz.z - min_p.z) * length_z_of_voxel_inverse));

		if (col_idx == voxel_dim.cols_of_voxel)
			col_idx--;
		if (row_idx == voxel_dim.rows_of_voxel)
			row_idx--;
		if (depth_idx == voxel_dim.depths_of_voxel)
			depth_idx--;
		return;
	}

	static inline void Compute(const PointArray points_cloud, const PointInVoxelArray poins_index, ModuleStruct::Point3f& plane_normal, ModuleStruct::Point3f& plane_center) {
		ModuleStruct::CMat points_matrix; // row * col = n * 3
		ModuleStruct::CMat covariance_matrix; //3 * 3
		int type = CV_64FC1;
		if (sizeof(IntermediateType) == 4)
		{
			type = CV_32FC1;
		}
		points_matrix.release();
		covariance_matrix.release();
		points_matrix.create(poins_index.size, 3, type);
		plane_normal.x = plane_normal.y = plane_normal.z = 0.f;
		plane_center.x = plane_center.y = plane_center.z = 0.f;

		for (unsigned int i = 0; i < poins_index.size; i++)
		{
			unsigned int point_idx = poins_index.point_idx[i];
			ModuleStruct::Point3f Point = points_cloud.points[point_idx];
			IntermediateType* ptr = points_matrix.ptr<IntermediateType>(i);
			ptr[0] = Point.x;
			ptr[1] = Point.y;
			ptr[2] = Point.z;
		}
		ModuleStruct::CMat mean;
		cv::calcCovarMatrix(points_matrix, covariance_matrix, mean, cv::COVAR_NORMAL | cv::COVAR_ROWS);
		covariance_matrix = covariance_matrix / (poins_index.size - 1);
		plane_center.x = static_cast<float>(mean.at<IntermediateType>(0, 0));
		plane_center.y = static_cast<float>(mean.at<IntermediateType>(0, 1));
		plane_center.z = static_cast<float>(mean.at<IntermediateType>(0, 2));

		ModuleStruct::CMat eig_val_mat, eig_vec_mat;
		cv::eigen(covariance_matrix, eig_val_mat, eig_vec_mat);
		plane_normal.x = static_cast<float>(eig_vec_mat.at<IntermediateType>(2, 0));
		plane_normal.y = static_cast<float>(eig_vec_mat.at<IntermediateType>(2, 1));
		plane_normal.z = static_cast<float>(eig_vec_mat.at<IntermediateType>(2, 2));
	}

	static inline void Compute(const std::vector<ModuleStruct::Point3f> poins_index, ModuleStruct::Point3f& plane_normal, ModuleStruct::Point3f& plane_center) {
		ModuleStruct::CMat points_matrix; // row * col = n * 3
		ModuleStruct::CMat covariance_matrix; //3 * 3
		int type = CV_64FC1;
		if (sizeof(IntermediateType) == 4)
		{
			type = CV_32FC1;
		}
		points_matrix.release();
		covariance_matrix.release();
		points_matrix.create((int)poins_index.size(), 3, type);
		plane_normal.x = plane_normal.y = plane_normal.z = 0.f;
		plane_center.x = plane_center.y = plane_center.z = 0.f;

		for (unsigned int i = 0; i < poins_index.size(); i++)
		{
			//unsigned int point_idx = poins_index[i];
			ModuleStruct::Point3f Point = poins_index[i];
			IntermediateType* ptr = points_matrix.ptr<IntermediateType>(i);
			ptr[0] = Point.x;
			ptr[1] = Point.y;
			ptr[2] = Point.z;
		}
		ModuleStruct::CMat mean;
		cv::calcCovarMatrix(points_matrix, covariance_matrix, mean, cv::COVAR_NORMAL | cv::COVAR_ROWS);
		covariance_matrix = covariance_matrix / static_cast<double>(poins_index.size() - 1);
		plane_center.x = static_cast<float>(mean.at<IntermediateType>(0, 0));
		plane_center.y = static_cast<float>(mean.at<IntermediateType>(0, 1));
		plane_center.z = static_cast<float>(mean.at<IntermediateType>(0, 2));

		ModuleStruct::CMat eig_val_mat, eig_vec_mat;

		cv::eigen(covariance_matrix, eig_val_mat, eig_vec_mat);
		plane_normal.x = static_cast<float>(eig_vec_mat.at<IntermediateType>(2, 0));
		plane_normal.y = static_cast<float>(eig_vec_mat.at<IntermediateType>(2, 1));
		plane_normal.z = static_cast<float>(eig_vec_mat.at<IntermediateType>(2, 2));
	}

	static inline void Compute(unsigned int point_cnt, SumforCovariance sums, ModuleStruct::Point3f& plane_normal, ModuleStruct::Point3f& plane_center, float& eigen_mse)
	{
		if (point_cnt <= 0) return;

		double point_num_inverse = (double)1.0 / point_cnt;

		//plane center
		plane_center.x = (float)(sums.sum_x * point_num_inverse);
		plane_center.y = (float)(sums.sum_y * point_num_inverse);
		plane_center.z = (float)(sums.sum_z * point_num_inverse);

		//calculate covariance matrix
		double cov_mat_element[6] = { sums.sum_xx - sums.sum_x * plane_center.x,sums.sum_xy - sums.sum_x * plane_center.y,
			sums.sum_xz - sums.sum_x * plane_center.z,sums.sum_yy - sums.sum_y * plane_center.y,
			sums.sum_yz - sums.sum_y * plane_center.z,sums.sum_zz - sums.sum_z * plane_center.z };

#ifdef FAST_COMPUTE_EIGEN
		double eig_val[3], eig_vec[3];

		MathOperation::ComputeFastEigenParallel(&cov_mat_element[0], &eig_val[0], &eig_vec[0]);
		plane_normal.x = eig_vec[0];
		plane_normal.y = eig_vec[1];
		plane_normal.z = eig_vec[2];

		//plane mean squared error
		eigen_mse = std::fabs((float)(eig_val[2] * point_num_inverse));
#else
		double cov_mat_arr[3][3] = { { cov_mat_element[0],cov_mat_element[1],cov_mat_element[2] },
		{ cov_mat_element[1], cov_mat_element[3], cov_mat_element[4] },
		{ cov_mat_element[2], cov_mat_element[4], cov_mat_element[5] } };

		ModuleStruct::CMat cov_mat(3, 3, CV_64F, cov_mat_arr);
		cov_mat *= point_num_inverse;

		ModuleStruct::CMat eig_val_mat, eig_vec_mat;

		cv::eigen(cov_mat, eig_val_mat, eig_vec_mat);

		plane_normal.x = (float)(eig_vec_mat.at<double>(2, 0));
		plane_normal.y = (float)(eig_vec_mat.at<double>(2, 1));
		plane_normal.z = (float)(eig_vec_mat.at<double>(2, 2));
		//plane mean squared error
		//voxel_grid->plane_mse = std::fabs((float)(eig_val_mat.at<double>(2) * point_num_inverse));
		eigen_mse = std::fabs((float)(eig_val_mat.at<double>(2) * point_num_inverse));
#endif
	}

	/**
	* \brief  compare according to multiple conditions, large_result and equal_result are the result of  five same pairs elements
	* only when the first pair element of five equal_results is true, the second elements of large_result is considered, and only if large_result
	* is true , the update_flag is set.  If all the equal_result elements is true,  return false
	* @param bool large_result[5],input bool type,show the compare result of whether  five  pairs elements are larger than the second five parameters or not
	* @param bool equal_result[5],input bool parameters,show the compare result of whether first five  parameters are equal to the second five parameters or not
	* @param update_flag  output, if the second
	* @return if success or failed for compare result
	*/
	static inline bool CompareByMultipleConditions(bool large_result[5], bool equal_result[5], unsigned char& update_flag) {
		if (large_result[0] == true) {
			update_flag = 1;
			return true;
		}
		else if (equal_result[0] == false) {
			return true;
		}

		if (large_result[1] == true) {
			update_flag = 1;
			return true;
		}
		else if (equal_result[1] == false) {
			return true;
		}

		if (large_result[2] == true) {
			update_flag = 1;
			return true;
		}
		else if (equal_result[2] == false) {
			return true;
		}

		if (large_result[3] == true) {
			update_flag = 1;
			return true;
		}
		else if (equal_result[3] == false) {
			return true;
		}

		if (large_result[4] == true) {
			update_flag = 1;
			return true;
		}
		else if (equal_result[4] == false) {
			return true;
		}
		else
		{
			return false;
		}
	}

	/**
	* \brief Identify the two points group are close or connected by pre-specified distance
	* @param const PointArray pt_cloud_xyz,input parameter, input data cloud data
	* @param const std::vector<unsigned int> first_points , input parameter, the point indices of first points group
	* @param const std::vector<unsigned int> second_points , input parameter, the point indices of second points group
	* @param const float distance , input parameter, pre-specified distance
	* @return true if the two points group distance is lower thean pre-specified distance ,otherwise return false
	*/
	static inline bool Identify2PointsGroupCLoseByDistance(const PointArray pt_cloud_xyz, const std::vector<unsigned int> first_points, const std::vector<unsigned int> second_points, const float distance) {
		if ((first_points.size() == 0) || (second_points.size() == 0))
		{
			log_error(" Input first_points size = %d second_points size =%d error ", first_points.size(), second_points.size());
			return false;
		}

		// step 1: get points center of 2 points group
		ModuleStruct::Point3f first_points_center = { 0.f,0.f,0.f };
		for (size_t i = 0; i < first_points.size(); i++)
		{
			ModuleStruct::Point3f point = pt_cloud_xyz.points[first_points[i]];
			first_points_center += point;
		}
		int first_point_cnt = static_cast<int>(first_points.size());
		first_points_center /= first_point_cnt;

		ModuleStruct::Point3f sec_points_center;
		for (size_t i = 0; i < second_points.size(); i++)
		{
			ModuleStruct::Point3f point = pt_cloud_xyz.points[second_points[i]];
			sec_points_center += point;
		}
		int sec_point_cnt = static_cast<int>(second_points.size());
		sec_points_center /= sec_point_cnt;

		// step 2:  sort 2 points by distance to the center of each other
		std::vector <PointIdx2Dist> first_idx_to_dist(first_points.size());
		for (size_t i = 0; i < first_points.size(); i++)
		{
			ModuleStruct::Point3f point = pt_cloud_xyz.points[first_points[i]];
			first_idx_to_dist[i].cloud_point_idx = first_points[i];
			ModuleStruct::Point3f delta_point = point - sec_points_center;
			float distance = std::fabs(delta_point.x) + std::fabs(delta_point.y) + std::fabs(delta_point.z);
			first_idx_to_dist[i].delta_dist = distance;
		}
		std::sort(first_idx_to_dist.begin(), first_idx_to_dist.end(), std::less<PointIdx2Dist>{});

		std::vector <PointIdx2Dist>sec_idx_to_dist(second_points.size());
		for (size_t i = 0; i < second_points.size(); i++)
		{
			ModuleStruct::Point3f point = pt_cloud_xyz.points[second_points[i]];
			sec_idx_to_dist[i].cloud_point_idx = second_points[i];
			ModuleStruct::Point3f delta_point = point - first_points_center;
			float distance = std::fabs(delta_point.x) + std::fabs(delta_point.y) + std::fabs(delta_point.z);
			sec_idx_to_dist[i].delta_dist = distance;
		}
		std::sort(sec_idx_to_dist.begin(), sec_idx_to_dist.end(), std::less<PointIdx2Dist>{});

		int first_comp_cnt = 2;
		int sec_comp_cnt = 2;
		if (first_comp_cnt > static_cast<int>(first_points.size()))  first_comp_cnt = static_cast<int>(first_points.size());
		if (sec_comp_cnt > static_cast<int>(second_points.size()))  sec_comp_cnt = static_cast<int>(second_points.size());
		ModuleStruct::Point3f first_comp_center, sec_comp_center;

		first_comp_center = { 0.f,0.f,0.f };
		for (int i = 0; i < first_comp_cnt; i++)
		{
			first_comp_center += pt_cloud_xyz.points[first_idx_to_dist[i].cloud_point_idx];
		}
		first_comp_center = first_comp_center / first_comp_cnt;

		sec_comp_center = { 0.f,0.f,0.f };
		for (int i = 0; i < sec_comp_cnt; i++)
		{
			sec_comp_center += pt_cloud_xyz.points[sec_idx_to_dist[i].cloud_point_idx];
		}
		sec_comp_center = sec_comp_center / sec_comp_cnt;

		float dist = ComputePointToPointDist<float, ModuleStruct::Point3f>(first_comp_center, sec_comp_center);
		bool rtn = false;
		if (dist < distance) rtn = true;

		return rtn;
	}

	/**
	* \brief Identify the two points group are close or connected by pre-specified distance
	* @param const PointArray pt_cloud_xyz,input parameter, input data cloud data
	* @param const std::vector<unsigned int> first_points , input parameter, the point indices of first points group
	* @param const PointInVoxelArray second_points , input parameter, the point indices of second points group
	* @param const float distance , input parameter, pre-specified distance
	* @return true if the two points group distance is lower thean pre-specified distance ,otherwise return false
	*/
	static inline bool Identify2PointsGroupCLoseByDistance(const PointArray pt_cloud_xyz, const std::vector<unsigned int> first_points, const PointInVoxelArray second_points, const float distance) {
		if ((first_points.size() == 0) || (second_points.size == 0))
		{
			log_error(" Input first_points size = %d second_points size =%d error ", first_points.size(), second_points.size);
			return false;
		}

		// step 1: get points center of 2 points group
		ModuleStruct::Point3f first_points_center = { 0.f,0.f,0.f };
		for (int i = 0; i < first_points.size(); i++)
		{
			ModuleStruct::Point3f point = pt_cloud_xyz.points[first_points[i]];
			first_points_center += point;
		}
		int first_point_cnt = static_cast<int>(first_points.size());
		first_points_center /= first_point_cnt;

		ModuleStruct::Point3f sec_points_center;
		for (unsigned int i = 0; i < second_points.size; i++)
		{
			ModuleStruct::Point3f point = pt_cloud_xyz.points[second_points.point_idx[i]];
			sec_points_center += point;
		}
		int sec_point_cnt = second_points.size;
		sec_points_center /= sec_point_cnt;

		// step 2:  sort 2 points by distance to the center of 2 points group
		std::vector <PointIdx2Dist> first_idx_to_dist(first_points.size());
		for (size_t i = 0; i < first_points.size(); i++)
		{
			ModuleStruct::Point3f point = pt_cloud_xyz.points[first_points[i]];
			first_idx_to_dist[i].cloud_point_idx = first_points[i];
			ModuleStruct::Point3f delta_point = point - sec_points_center;
			float distance = std::fabs(delta_point.x) + std::fabs(delta_point.y) + std::fabs(delta_point.z);
			first_idx_to_dist[i].delta_dist = distance;
		}
		std::sort(first_idx_to_dist.begin(), first_idx_to_dist.end(), std::less<PointIdx2Dist>{});

		std::vector <PointIdx2Dist>sec_idx_to_dist(second_points.size);
		for (size_t i = 0; i < second_points.size; i++)
		{
			ModuleStruct::Point3f point = pt_cloud_xyz.points[second_points.point_idx[i]];
			sec_idx_to_dist[i].cloud_point_idx = second_points.point_idx[i];
			ModuleStruct::Point3f delta_point = point - first_points_center;
			float distance = std::fabs(delta_point.x) + std::fabs(delta_point.y) + std::fabs(delta_point.z);
			sec_idx_to_dist[i].delta_dist = distance;
		}
		std::sort(sec_idx_to_dist.begin(), sec_idx_to_dist.end(), std::less<PointIdx2Dist>{});

		int first_comp_cnt = 2;
		int sec_comp_cnt = 2;
		if (first_comp_cnt > static_cast<int>(first_points.size()))  first_comp_cnt = static_cast<int>(first_points.size());
		if (sec_comp_cnt > static_cast<int>(second_points.size))  sec_comp_cnt = static_cast<int>(second_points.size);

		ModuleStruct::Point3f first_comp_center = { 0.f,0.f,0.f };
		for (int i = 0; i < first_comp_cnt; i++)
		{
			first_comp_center += pt_cloud_xyz.points[first_idx_to_dist[i].cloud_point_idx];
		}
		first_comp_center = first_comp_center / first_comp_cnt;

		ModuleStruct::Point3f sec_comp_center = { 0.f,0.f,0.f };
		for (int i = 0; i < sec_comp_cnt; i++)
		{
			sec_comp_center += pt_cloud_xyz.points[sec_idx_to_dist[i].cloud_point_idx];
		}
		sec_comp_center = sec_comp_center / sec_comp_cnt;

		float dist = ComputePointToPointDist<float, ModuleStruct::Point3f>(first_comp_center, sec_comp_center);
		bool rtn = false;
		if (dist < distance) rtn = true;
		return rtn;
	}

	/**
	* \brief Identify the two points group are close or connected by pre-specified distance
	* @param const PointArray pt_cloud_xyz,input parameter, input data cloud data
	* @param const const PointInVoxelArray first_points , input parameter, the point indices of first points group
	* @param const PointInVoxelArray second_points , input parameter, the point indices of second points group
	* @param const float distance , input parameter, pre-specified distance
	* @return true if the two points group distance is lower thean pre-specified distance ,otherwise return false
	*/
	static inline bool Identify2PointsGroupCLoseByDistance(const PointArray pt_cloud_xyz, const PointInVoxelArray first_points, const PointInVoxelArray second_points, const float distance) {
		if ((first_points.size == 0) || (second_points.size == 0))
		{
			log_error(" Input first_points size = %d second_points size =%d error ", first_points.size, second_points.size);
			return false;
		}

		// step 1: get points center of 2 points group
		ModuleStruct::Point3f first_points_center = { 0.f,0.f,0.f };
		for (unsigned int i = 0; i < first_points.size; i++)
		{
			ModuleStruct::Point3f point = pt_cloud_xyz.points[first_points.point_idx[i]];
			first_points_center += point;
		}
		int first_point_cnt = first_points.size;
		first_points_center /= first_point_cnt;

		ModuleStruct::Point3f sec_points_center;
		for (unsigned int i = 0; i < second_points.size; i++)
		{
			ModuleStruct::Point3f point = pt_cloud_xyz.points[second_points.point_idx[i]];
			sec_points_center += point;
		}
		int sec_point_cnt = second_points.size;
		sec_points_center /= sec_point_cnt;

		// step 2:  sort 2 points by distance to the center of 2 points group
		std::vector <PointIdx2Dist> first_idx_to_dist(first_points.size);
		for (size_t i = 0; i < first_points.size; i++)
		{
			ModuleStruct::Point3f point = pt_cloud_xyz.points[first_points.point_idx[i]];
			first_idx_to_dist[i].cloud_point_idx = first_points.point_idx[i];
			ModuleStruct::Point3f delta_point = point - sec_points_center;
			float distance = std::fabs(delta_point.x) + std::fabs(delta_point.y) + std::fabs(delta_point.z);
			first_idx_to_dist[i].delta_dist = distance;
		}
		std::sort(first_idx_to_dist.begin(), first_idx_to_dist.end(), std::less<PointIdx2Dist>{});

		std::vector <PointIdx2Dist>sec_idx_to_dist(second_points.size);
		for (size_t i = 0; i < second_points.size; i++)
		{
			ModuleStruct::Point3f point = pt_cloud_xyz.points[second_points.point_idx[i]];
			sec_idx_to_dist[i].cloud_point_idx = second_points.point_idx[i];
			ModuleStruct::Point3f delta_point = point - first_points_center;
			float distance = std::fabs(delta_point.x) + std::fabs(delta_point.y) + std::fabs(delta_point.z);
			sec_idx_to_dist[i].delta_dist = distance;
		}
		std::sort(sec_idx_to_dist.begin(), sec_idx_to_dist.end(), std::less<PointIdx2Dist>{});

		// step 3:   each group use 3 closest points  to  compute center distance, in 3D space only need 3  points to identify close of 2 points group
		int first_comp_cnt = 2;
		int sec_comp_cnt = 2;
		if (first_comp_cnt > static_cast<int>(first_points.size))  first_comp_cnt = static_cast<int>(first_points.size);
		if (sec_comp_cnt > static_cast<int>(second_points.size))  sec_comp_cnt = static_cast<int>(second_points.size);

		ModuleStruct::Point3f first_comp_center = { 0.f,0.f,0.f };
		for (int i = 0; i < first_comp_cnt; i++)
		{
			first_comp_center += pt_cloud_xyz.points[first_idx_to_dist[i].cloud_point_idx];
		}
		first_comp_center = first_comp_center / first_comp_cnt;

		ModuleStruct::Point3f sec_comp_center = { 0.f,0.f,0.f };
		for (int i = 0; i < sec_comp_cnt; i++)
		{
			sec_comp_center += pt_cloud_xyz.points[sec_idx_to_dist[i].cloud_point_idx];
		}
		sec_comp_center = sec_comp_center / sec_comp_cnt;

		float dist = ComputePointToPointDist<float, ModuleStruct::Point3f>(first_comp_center, sec_comp_center);
		bool rtn = false;
		if (dist < distance) rtn = true;
		return rtn;
	}

	/**
	* \brief Identify the two points group are close or connected by pre-specified distance
	* @param const PointArray pt_cloud_xyz,input parameter, input data cloud data
	* @param const const PointInVoxelArray first_points , input parameter, the point indices of first points group
	* @param const PointInVoxelArray second_points , input parameter, the point indices of second points group
	* @param const float distance , input parameter, pre-specified distance
	* @return true if the two points group distance is lower thean pre-specified distance ,otherwise return false
	*/
	static inline bool Identify2PointsGroupCLoseByDistance(const ModuleStruct::Point3fArray pt_cloud_xyz, const PointInVoxelArray first_points, const PointInVoxelArray second_points, const float distance) {
		if ((first_points.size == 0) || (second_points.size == 0))
		{
			log_error(" Input first_points size = %d second_points size =%d error ", first_points.size, second_points.size);
			return false;
		}

		// step 1: get points center of 2 points group
		ModuleStruct::Point3f first_points_center = { 0.f,0.f,0.f };
		for (unsigned int i = 0; i < first_points.size; i++)
		{
			ModuleStruct::Point3f point = pt_cloud_xyz[first_points.point_idx[i]];
			first_points_center += point;
		}
		int first_point_cnt = first_points.size;
		first_points_center /= first_point_cnt;

		ModuleStruct::Point3f sec_points_center;
		for (unsigned int i = 0; i < second_points.size; i++)
		{
			ModuleStruct::Point3f point = pt_cloud_xyz[second_points.point_idx[i]];
			sec_points_center += point;
		}
		int sec_point_cnt = second_points.size;
		sec_points_center /= sec_point_cnt;

		// step 2:  sort 2 points by distance to the center of 2 points group
		std::vector <PointIdx2Dist> first_idx_to_dist(first_points.size);
		for (size_t i = 0; i < first_points.size; i++)
		{
			ModuleStruct::Point3f point = pt_cloud_xyz[first_points.point_idx[i]];
			first_idx_to_dist[i].cloud_point_idx = first_points.point_idx[i];
			ModuleStruct::Point3f delta_point = point - sec_points_center;
			float distance = std::fabs(delta_point.x) + std::fabs(delta_point.y) + std::fabs(delta_point.z);
			first_idx_to_dist[i].delta_dist = distance;
		}
		std::sort(first_idx_to_dist.begin(), first_idx_to_dist.end(), std::less<PointIdx2Dist>{});

		std::vector <PointIdx2Dist>sec_idx_to_dist(second_points.size);
		for (size_t i = 0; i < second_points.size; i++)
		{
			ModuleStruct::Point3f point = pt_cloud_xyz[second_points.point_idx[i]];
			sec_idx_to_dist[i].cloud_point_idx = second_points.point_idx[i];
			ModuleStruct::Point3f delta_point = point - first_points_center;
			float distance = std::fabs(delta_point.x) + std::fabs(delta_point.y) + std::fabs(delta_point.z);
			sec_idx_to_dist[i].delta_dist = distance;
		}
		std::sort(sec_idx_to_dist.begin(), sec_idx_to_dist.end(), std::less<PointIdx2Dist>{});

		// step 3:   each group use 3 closest points  to  compute center distance, in 3D space only need 3  points to identify close of 2 points group
		int first_comp_cnt = 2;
		int sec_comp_cnt = 2;
		if (first_comp_cnt > static_cast<int>(first_points.size))  first_comp_cnt = static_cast<int>(first_points.size);
		if (sec_comp_cnt > static_cast<int>(second_points.size))  sec_comp_cnt = static_cast<int>(second_points.size);

		ModuleStruct::Point3f first_comp_center = { 0.f,0.f,0.f };
		for (int i = 0; i < first_comp_cnt; i++)
		{
			first_comp_center += pt_cloud_xyz[first_idx_to_dist[i].cloud_point_idx];
		}
		first_comp_center = first_comp_center / first_comp_cnt;

		ModuleStruct::Point3f sec_comp_center = { 0.f,0.f,0.f };
		for (int i = 0; i < sec_comp_cnt; i++)
		{
			sec_comp_center += pt_cloud_xyz[sec_idx_to_dist[i].cloud_point_idx];
		}
		sec_comp_center = sec_comp_center / sec_comp_cnt;

		float dist = ComputePointToPointDist<float, ModuleStruct::Point3f>(first_comp_center, sec_comp_center);
		bool rtn = false;
		if (dist < distance) rtn = true;
		return rtn;
	}

	/**
	* \brief Identify the two points group are close or connected by pre-specified distance in 2D space
	* @param const std::vector<Point2f> pt_cloud_xyz,input parameter, input data cloud data in 2D space
	* @param const std::vector<unsigned int> first_points, input parameter, the point indices of first points group
	* @param const PointInVoxelArray second_points , input parameter, the point indices of second points group
	* @param const float distance , input parameter, pre-specified distance
	* @return true if the two points group distance is lower thean pre-specified distance ,otherwise return false
	*/
	static inline bool Identify2PointsGroupCLoseByDistance(const std::vector<ModuleStruct::Point2f> pt_cloud_xyz, const std::vector<unsigned int> first_points, const PointInVoxelArray second_points, const float distance) {
		if ((first_points.size() == 0) || (second_points.size == 0))
		{
			//log_error(" Input first_points size = %d second_points size =%d error ", first_points.size(), second_points.size);
			return false;
		}
		// step 1: get points center of 2 points group
		ModuleStruct::Point2f first_points_center = { 0.f,0.f };
		for (int i = 0; i < first_points.size(); i++)
		{
			ModuleStruct::Point2f point = pt_cloud_xyz[first_points[i]];
			first_points_center += point;
		}
		int first_point_cnt = static_cast<int>(first_points.size());
		first_points_center /= first_point_cnt;

		ModuleStruct::Point2f sec_points_center = { 0.f,0.f };
		for (unsigned int i = 0; i < second_points.size; i++)
		{
			ModuleStruct::Point2f point = pt_cloud_xyz[second_points.point_idx[i]];
			sec_points_center += point;
		}
		int sec_point_cnt = second_points.size;
		sec_points_center = sec_points_center / sec_point_cnt;

		// step 2:  sort 2 points by distance to the center of 2 points group
		std::vector <PointIdx2Dist> first_idx_to_dist(first_points.size());
		for (size_t i = 0; i < first_points.size(); i++)
		{
			ModuleStruct::Point2f point = pt_cloud_xyz[first_points[i]];
			first_idx_to_dist[i].cloud_point_idx = first_points[i];
			//ModuleStruct::Point2f delta_point = point - sec_points_center;
			//float distance = std::fabs(delta_point.x) + std::fabs(delta_point.y);
			float distance = Util_Math::ComputePointToPointDist2D<float, ModuleStruct::Point2f>(point, sec_points_center);
			first_idx_to_dist[i].delta_dist = distance;
		}
		std::sort(first_idx_to_dist.begin(), first_idx_to_dist.end(), std::less<PointIdx2Dist>{});

		std::vector <PointIdx2Dist>sec_idx_to_dist(second_points.size);
		for (size_t i = 0; i < second_points.size; i++)
		{
			ModuleStruct::Point2f point = pt_cloud_xyz[second_points.point_idx[i]];
			sec_idx_to_dist[i].cloud_point_idx = second_points.point_idx[i];
			//ModuleStruct::Point2f delta_point = point - first_points_center;
			//float distance = std::fabs(delta_point.x) + std::fabs(delta_point.y);
			float distance = Util_Math::ComputePointToPointDist2D<float, ModuleStruct::Point2f>(point, first_points_center);
			sec_idx_to_dist[i].delta_dist = distance;
		}
		std::sort(sec_idx_to_dist.begin(), sec_idx_to_dist.end(), std::less<PointIdx2Dist>{});

		// step 3:   each group use 2 closest points  to  compute center distance, in 2D space only need 2  points to identify close of 2 points group
		float dist1 = ComputePointToPointDist2D<float, ModuleStruct::Point2f>(pt_cloud_xyz[first_idx_to_dist[0].cloud_point_idx], pt_cloud_xyz[sec_idx_to_dist[0].cloud_point_idx]);
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

		ModuleStruct::Point2f first_comp_center = { 0.f,0.f };
		for (int i = 0; i < first_comp_cnt; i++)
		{
			first_comp_center += pt_cloud_xyz[first_idx_to_dist[i].cloud_point_idx];
		}
		first_comp_center = first_comp_center / first_comp_cnt;

		ModuleStruct::Point2f sec_comp_center = { 0.f,0.f };
		for (int i = 0; i < sec_comp_cnt; i++)
		{
			sec_comp_center += pt_cloud_xyz[sec_idx_to_dist[i].cloud_point_idx];
		}
		sec_comp_center = sec_comp_center / sec_comp_cnt;

		float dist = ComputePointToPointDist2D<float, ModuleStruct::Point2f>(first_comp_center, sec_comp_center);
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
	static inline bool Identify2PointsGroupCLoseByDistance(const std::vector<ModuleStruct::Point2f> pt_cloud_xyz, const std::vector<unsigned int> first_points, const std::vector<unsigned int> second_points, const float distance) {
		// step 1: get points center of 2 points group

		if ((first_points.size() == 0) || (second_points.size() == 0))
		{
			//log_error(" Input first_points size = %d second_points size =%d error ", first_points.size(), second_points.size());
			return false;
		}

		ModuleStruct::Point2f first_points_center = { 0.f,0.f };
		for (int i = 0; i < first_points.size(); i++)
		{
			ModuleStruct::Point2f point = pt_cloud_xyz[first_points[i]];
			first_points_center += point;
		}
		int first_point_cnt = static_cast<int>(first_points.size());
		first_points_center /= first_point_cnt;

		ModuleStruct::Point2f sec_points_center = { 0.f,0.f };
		for (unsigned int i = 0; i < second_points.size(); i++)
		{
			ModuleStruct::Point2f point = pt_cloud_xyz[second_points[i]];
			sec_points_center += point;
		}
		int sec_point_cnt = static_cast<int>(second_points.size());
		sec_points_center = sec_points_center / sec_point_cnt;

		// step 2:  sort 2 points by distance to the center of 2 points group
		std::vector <PointIdx2Dist> first_idx_to_dist(first_points.size());
		for (size_t i = 0; i < first_points.size(); i++)
		{
			ModuleStruct::Point2f point = pt_cloud_xyz[first_points[i]];
			first_idx_to_dist[i].cloud_point_idx = first_points[i];
			//ModuleStruct::Point2f delta_point = point - sec_points_center;
			//float distance = std::fabs(delta_point.x) + std::fabs(delta_point.y);
			float distance = Util_Math::ComputePointToPointDist2D<float, ModuleStruct::Point2f>(point, sec_points_center);
			first_idx_to_dist[i].delta_dist = distance;
		}
		std::sort(first_idx_to_dist.begin(), first_idx_to_dist.end(), std::less<PointIdx2Dist>{});

		std::vector <PointIdx2Dist>sec_idx_to_dist(second_points.size());
		for (size_t i = 0; i < second_points.size(); i++)
		{
			ModuleStruct::Point2f point = pt_cloud_xyz[second_points[i]];
			sec_idx_to_dist[i].cloud_point_idx = second_points[i];
			//ModuleStruct::Point2f delta_point = point - first_points_center;
			//float distance = std::fabs(delta_point.x) + std::fabs(delta_point.y);
			float distance = Util_Math::ComputePointToPointDist2D<float, ModuleStruct::Point2f>(point, first_points_center);
			sec_idx_to_dist[i].delta_dist = distance;
		}
		std::sort(sec_idx_to_dist.begin(), sec_idx_to_dist.end(), std::less<PointIdx2Dist>{});

		// step 3:   each group use 2 closest points  to  compute center distance, in 2D space only need 2  points to identify close of 2 points group
		float dist1 = ComputePointToPointDist2D<float, ModuleStruct::Point2f>(pt_cloud_xyz[first_idx_to_dist[0].cloud_point_idx], pt_cloud_xyz[sec_idx_to_dist[0].cloud_point_idx]);
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

		ModuleStruct::Point2f first_comp_center = { 0.f,0.f };
		for (int i = 0; i < first_comp_cnt; i++)
		{
			first_comp_center += pt_cloud_xyz[first_idx_to_dist[i].cloud_point_idx];
		}
		first_comp_center = first_comp_center / first_comp_cnt;

		ModuleStruct::Point2f sec_comp_center = { 0.f,0.f };
		for (int i = 0; i < sec_comp_cnt; i++)
		{
			sec_comp_center += pt_cloud_xyz[sec_idx_to_dist[i].cloud_point_idx];
		}
		sec_comp_center = sec_comp_center / sec_comp_cnt;

		float dist = ComputePointToPointDist2D<float, ModuleStruct::Point2f>(first_comp_center, sec_comp_center);
		min_dist = min_dist < dist ? min_dist : dist;
		bool rtn = false;
		if (min_dist < distance) rtn = true;
		return rtn;
	}

	static inline ModuleStruct::Point3f point_rot(ModuleStruct::CMat rot_mat, ModuleStruct::Point3f& pt)
	{
		ModuleStruct::Point3f c_pt = ModuleStruct::Point3f(
			rot_mat.at<float>(0, 0) * pt.x + rot_mat.at<float>(0, 1) * pt.y + rot_mat.at<float>(0, 2) * pt.z,
			rot_mat.at<float>(1, 0) * pt.x + rot_mat.at<float>(1, 1) * pt.y + rot_mat.at<float>(1, 2) * pt.z,
			rot_mat.at<float>(2, 0) * pt.x + rot_mat.at<float>(2, 1) * pt.y + rot_mat.at<float>(2, 2) * pt.z);
		return c_pt;
	}

	static inline Vector<ModuleStruct::Point3f> plane_rot(ModuleStruct::CMat mat_r, const std::vector<cv::Point3f> &pts) {
		// data validation
		if (mat_r.rows != 3 || mat_r.cols != 3 || pts.empty()) return Vector<ModuleStruct::Point3f>();

		std::vector<std::vector<float>> mat_clbr_v(3, std::vector<float>(3));
		for (size_t i = 0; i < 3; i++)
			for (size_t j = 0; j < 3; j++)
				mat_clbr_v[i][j] = mat_r.ptr<float>(i)[j];

		size_t npts = pts.size();
		Vector<ModuleStruct::Point3f> rotated_pts(npts);
#pragma omp parallel for
		for (size_t i = 0; i < npts; i++) {
			rotated_pts[i].x = mat_clbr_v[0][0] * pts[i].x + mat_clbr_v[0][1] * pts[i].y + mat_clbr_v[0][2] * pts[i].z;
			rotated_pts[i].y = mat_clbr_v[1][0] * pts[i].x + mat_clbr_v[1][1] * pts[i].y + mat_clbr_v[1][2] * pts[i].z;
			rotated_pts[i].z = mat_clbr_v[2][0] * pts[i].x + mat_clbr_v[2][1] * pts[i].y + mat_clbr_v[2][2] * pts[i].z;
		}
		return rotated_pts;
	}

	static inline Vector<Vector<ModuleStruct::Point3f>> planes_rot(ModuleStruct::CMat rot_mat, Vector<Vector<ModuleStruct::Point3f>>& pts)
	{
		if (pts.empty())
			return  Vector<Vector<ModuleStruct::Point3f>>();

		Vector<Vector<ModuleStruct::Point3f>> rotated_pts(pts.size());

		for (int i = 0; i < rotated_pts.size(); i++)
		{
			rotated_pts[i] = plane_rot(rot_mat, pts[i]);
		}

		return rotated_pts;
	}

	static inline bool GetNewNormalAndCenter(std::vector<cv::Point3f> &pts_src,
		cv::Point3f& center, cv::Point3f& normal,
		std::vector<cv::Point3f> &fitplane)
	{
		if (pts_src.size() < 100) {
			return false;
		}
		//cout << "getNewNormalAndCenter test begin" << endl;
		//comptue normal first
		VoxelDataStruct::VoxelGrid_Base fitplane2_ori;
		for (size_t i = 0; i < pts_src.size(); i++)
		{
			fitplane2_ori.Push(pts_src[i]);
		}
		fitplane2_ori.Compute();

		std::vector<cv::Point3f> rot_plane_points;

		ModuleStruct::Point3f zdirection = { 0.f,0.f,1.f };
		//cv::Mat rotation_matrix_origin = MeasureBase::CalRotationMatrixFromVectors(fitplane2_ori.plane_normals, zdirection_test);
		//CMat rotation_matrix = CreateRotationMat4E(fitplane2.GetNormals(), zdirection);
		ModuleStruct::Point3f planeNormal = { fitplane2_ori.plane_normals[0],fitplane2_ori.plane_normals[1],fitplane2_ori.plane_normals[2] };
		cv::Mat rotation_matrix_origin = CreateRotationMat4E(planeNormal, zdirection);

		//MeasureBase::RotatePoints(pts_src, rotation_matrix_origin, rot_plane_points);
		rot_plane_points = plane_rot(rotation_matrix_origin, pts_src);

		//std::cout << rot_plane_points.size() << std::endl;
		//float plane_minmax_xy[6];
		//MeasureBase::FindPointsMinMaxXYZ(rot_plane_points, plane_minmax_xy);
		ModuleStruct::Point3f m_max, m_min;
		GetMinMaxXYZ(rot_plane_points, m_max, m_min);

		int voxel_witdh = 30;
		unsigned int voxel_num_in_x = std::floor((m_max.x - m_min.x) / voxel_witdh) + 1;
		unsigned int voxel_num_in_y = std::floor((m_max.y - m_min.y) / voxel_witdh) + 1;


		std::vector<cv::Point3f>  voxel_pts;
		std::vector<int>  voxel_pt_couts;
		//std::cout << voxel_num_in_x << "," << voxel_num_in_y << std::endl;
		voxel_pts.resize(voxel_num_in_x * voxel_num_in_y);
		voxel_pt_couts.resize(voxel_num_in_x * voxel_num_in_y);
		unsigned int x_idx, y_idx, voxel_idx;

		float delta_y;

		for (int i = 0; i < voxel_pts.size(); i++)
			voxel_pt_couts[i] = 0;

		for (int i = 0; i < rot_plane_points.size(); i++)
		{
			if (abs(rot_plane_points[i].x - m_min.x) < 30.0 ||
				abs(rot_plane_points[i].x - m_max.x) < 30.0 ||
				abs(rot_plane_points[i].y - m_min.y) < 30.0 ||
				abs(rot_plane_points[i].y - m_max.y) < 30.0)
				continue;
			//assign to voxel
			x_idx = std::floor((rot_plane_points[i].x - m_min.x) / voxel_witdh);
			y_idx = std::floor((rot_plane_points[i].y - m_min.y) / voxel_witdh);
			voxel_idx = y_idx * voxel_num_in_x + x_idx;

			voxel_pts[voxel_idx] += rot_plane_points[i];
			voxel_pt_couts[voxel_idx]++;

		}

		std::vector<int>  voxel_idxs;
		std::vector<cv::Point3f> new_pts;
		for (int i = 0; i < voxel_pts.size(); i++)
		{
			if (voxel_pt_couts[i] > 0)
			{
				voxel_pts[i] /= voxel_pt_couts[i];
				new_pts.push_back(voxel_pts[i]);
				voxel_idxs.push_back(i);
			}
		}

		std::vector<cv::Point3f>  voxel_pts_ori;
		///cv::Mat backward_rotation_matrix_from_y_coarse(3, 3, CV_32FC1);
		cv::Mat backward_rotation_matrix(3, 3, CV_32FC1);
		//	cv::transpose(rotation_matrix_to_y_coarse, backward_rotation_matrix_from_y_coarse);
		//cv::transpose(rotation_matrix_origin, backward_rotation_matrix);
		backward_rotation_matrix = rotation_matrix_origin.inv();

		std::vector<cv::Point3f> pts;
		Vector<unsigned int> point_to_centriod_array;
		if (new_pts.size() > 3)
			pts = new_pts;
		else
		   //CloudSampling::DownSample(rot_plane_points, 50, 50, 50, pts);
		   DownSampleFilter::FilterGridApply<Point3f, float, unsigned int>
			(rot_plane_points, 50, pts, point_to_centriod_array);
		size_t iter = 200;
		cv::RNG rng((unsigned)time(NULL));
		float sigma = 5.f;
		size_t pretotal = 0;     //?∴?o??ao???┷D：a|━?：oy?Y|━???：oy
		std::vector<float> bestplane(4);
		std::vector<float> bestmask(pts.size());
		for (size_t i = 0; i < iter; i++)
		{
			int idx1, idx2, idx3;
			idx1 = rng.uniform((int)0, (int)pts.size());
			do { idx2 = rng.uniform((int)0, (int)pts.size()); } while (idx2 == idx1);
			do { idx3 = rng.uniform((int)0, (int)pts.size()); } while (idx3 == idx1 || idx3 == idx2);
			cv::Point3f pt1 = pts[idx1], pt2 = pts[idx2], pt3 = pts[idx3];
			// plane function z=ax+by+c
			float a = ((pt1.z - pt2.z)*(pt1.y - pt3.y) - (pt1.z - pt3.z)*(pt1.y - pt2.y)) /
				((pt1.x - pt2.x)*(pt1.y - pt3.y) - (pt1.x - pt3.x)*(pt1.y - pt2.y));
			float b = ((pt1.z - pt3.z) - a * (pt1.x - pt3.x)) / (pt1.y - pt3.y);
			float c = pt1.z - a * pt1.x - b * pt1.y;
			float len = sqrt(a * a + b * b + 1.f);
			std::vector<float> plane = { a,b,-1.f,c };
			std::vector<float> mask(pts.size());
			size_t total = 0;
			for (size_t j = 0; j < pts.size(); j++)
			{
				mask[j] = abs(plane[0] * pts[j].x + plane[1] * pts[j].y + plane[2] * pts[j].z + plane[3]) / len;
				if (mask[j] < sigma) total++;
			}
			if (total > pretotal)
			{
				pretotal = total;
				bestplane = plane;
				bestmask = mask;
			}
		}
		fitplane.clear();
		for (size_t i = 0; i < bestmask.size(); i++)
		{
			if (bestmask[i] < sigma) {
				fitplane.push_back(pts[i]);
			}
		}

		fill(voxel_pt_couts.begin(), voxel_pt_couts.end(), 0);
		for (int i = 0; i < fitplane.size(); i++)
		{
			x_idx = std::floor((fitplane[i].x - m_min.x) / voxel_witdh);
			y_idx = std::floor((fitplane[i].y - m_min.y) / voxel_witdh);
			voxel_idx = y_idx * voxel_num_in_x + x_idx;
			voxel_pt_couts[voxel_idx] = 1;
		}

		std::vector<int> select_id;
		for (int i = 0; i < rot_plane_points.size(); i++)
		{
			if (abs(rot_plane_points[i].x - m_min.x) < 30.0 ||
				abs(rot_plane_points[i].x - m_max.x) < 30.0 ||
				abs(rot_plane_points[i].y - m_min.y) < 30.0 ||
				abs(rot_plane_points[i].y - m_max.y) < 30.0)
				continue;
			//assign to voxel
			x_idx = std::floor((rot_plane_points[i].x - m_min.x) / voxel_witdh);
			y_idx = std::floor((rot_plane_points[i].y - m_min.y) / voxel_witdh);
			voxel_idx = y_idx * voxel_num_in_x + x_idx;

			if (voxel_pt_couts[voxel_idx])
				select_id.push_back(i);

		}
		VoxelDataStruct::VoxelGrid_Base fitplane2_new;

		std::vector<cv::Point3f>  savept;
		for (size_t i = 0; i < select_id.size(); i++)
		{
			fitplane2_new.Push(pts_src[select_id[i]]);
			savept.push_back(pts_src[select_id[i]]);
		}
		fitplane2_new.Compute();
		fitplane2_new.CalPlaneCenter();

		// update normal
		normal = cv::Point3f(fitplane2_new.plane_normals[0], fitplane2_new.plane_normals[1], fitplane2_new.plane_normals[2]);
		center = cv::Point3f(fitplane2_new.plane_center[0], fitplane2_new.plane_center[1], fitplane2_new.plane_center[2]);
		//cout << "getNewNormalAndCenter test end;" << endl;
		return true;
	}

	static float ComputeVectorDotProduct(const cv::Point  vector_1, const cv::Point  vector_2) {
		return (vector_1.x * vector_2.x + vector_1.y * vector_2.y);
	}

	static float VectorNormalize(const cv::Point v)
	{
		return std::sqrtf(v.x * v.x + v.y * v.y);
	}

	static float CalLine2LineAngle(const cv::Point L1_A, const cv::Point L1_B, const cv::Point L2_A, const cv::Point L2_B) {
		cv::Point vector1 = L1_B - L1_A;
		cv::Point vector2 = L2_B - L2_A;

		float angle = std::acosf(MathOperation::ComputeVectorDotProduct(vector1, vector2) / MathOperation::VectorNormalize(vector1) / MathOperation::VectorNormalize(vector2));

		angle = angle * 180.f / CV_PI;
		angle = angle > 90 ? (180.f - angle) : angle;

		return angle;
	}

	static float PointToLinesegDist(cv::Point P, cv::Point A, cv::Point B)
	{
		double ab = sqrt(pow((B.x - A.x), 2) + pow((B.y - A.y), 2)); // |AB|
		double ap = sqrt(pow((P.x - A.x), 2) + pow((P.y - A.y), 2)); // |AP|
		double bp = sqrt(pow((P.x - B.x), 2) + pow((P.y - B.y), 2)); // |BP|
		double r = 0;
		if (ab > 0)
		{
			r = ((P.x - A.x)*(B.x - A.x) + (P.y - A.y)*(B.y - A.y)) / pow(ab, 2);
		} //r
		else
		{
			cout << "no lines" << endl;
		}

		double distance = 0;
		if (ab > 0)
		{
			if (r >= 1)
				distance = bp;
			else if (r > 0 && r < 1)
				distance = sqrt(pow(ap, 2) - r * r*pow(ab, 2));
			else
				distance = ap;
		}

		return distance;
	}

	static float ComputeTriangleHypotenuse(float x, float y)
	{
		return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
	}

	static bool FindPlaneMinMaxXYZ(const std::vector<cv::Point3f>& input_points,
		const int normal_axis,
		const float length_plane_threshold1,
		const float length_plane_threshold2,
		float* output_minmax_xyz)
	{
		if (input_points.empty())
			return false;

		if (!GetMinMaxXYZ(input_points, output_minmax_xyz))
			return false;

		if (normal_axis == 1) //for vertical planes along y-axis
		{
			if (output_minmax_xyz[1] - output_minmax_xyz[0] < length_plane_threshold1 || output_minmax_xyz[5] - output_minmax_xyz[4] < length_plane_threshold2)
			{
				log_error("plane not available along y-axis: %f, %f", output_minmax_xyz[1] - output_minmax_xyz[0], output_minmax_xyz[5] - output_minmax_xyz[4]);

				return false;
			}
		}
		else if (normal_axis == 2) // for top/bottom planes
		{
			if (output_minmax_xyz[1] - output_minmax_xyz[0] < length_plane_threshold1 || output_minmax_xyz[3] - output_minmax_xyz[2] < length_plane_threshold2)
			{
				log_error("plane not available top/bottom: %f, %f", output_minmax_xyz[1] - output_minmax_xyz[0], output_minmax_xyz[3] - output_minmax_xyz[2]);

				return false;
			}
		}
		else if (normal_axis == 0)//for vertical planes, normal_axis == 0 along x-axis
		{
			if (output_minmax_xyz[3] - output_minmax_xyz[2] < length_plane_threshold1 || output_minmax_xyz[5] - output_minmax_xyz[4] < length_plane_threshold2)
			{
				log_error("plane not available along x-axis: %f, %f", output_minmax_xyz[3] - output_minmax_xyz[2], output_minmax_xyz[5] - output_minmax_xyz[4]);

				return false;
			}
		}
		return true;
	}
};

#endif// MATHOPERATION_H