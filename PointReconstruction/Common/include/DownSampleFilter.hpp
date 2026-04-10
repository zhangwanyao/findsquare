/* Copyright (c) ASTRI, 2019. All rights reserved.
* This software is proprietary to and embodies the confidential technology
* of Hong Kong Applied Science and Technology Research Institute Company
* Limited (ASTRI).
*
* Possession, use, or copying of this software and media is authorized
* only pursuant to a valid written license from ASTRI or an authorized
* sublicensor.
*
* Author: mounty
* Date: 20190826
* Description: Downsample filter Class Definitions
*/

#include "ModuleStruct.hpp"
#include "util_opencv.hpp"
using namespace std;
using namespace ModuleStruct;

#ifndef DOWNSAMPLE_FILTER_HPP
#define DOWNSAMPLE_FILTER_HPP

namespace DownSampleFilter
{
	/**
	* \brief  Predefined voxel size,unit depends on application
	*/
	struct VoxelSizeParams
	{
		/**
		* \brief  voxel length in x direction
		*/
		float length_x_of_voxel;

		/**
		* \brief  voxel length in y direction
		*/
		float length_y_of_voxel;

		/**
		* \brief  voxel length in z direction
		*/
		float length_z_of_voxel;
	};

	/**
	* \brief  dimenshion struct of voxels
	*/
	struct VoxelDimension
	{
		int cols_of_voxel;
		int rows_of_voxel;
		int depths_of_voxel;
	};

	/**
	* \brief  struct of cloud point index for sorting by voxel grid index
	*/
	struct PointGrid2PointIdx
	{
		unsigned long long grid_idx;
		unsigned int cloud_point_index;
		bool operator < (const PointGrid2PointIdx& p) const { return (grid_idx < p.grid_idx); }
	};

	template<typename PT, typename DT>
	static inline bool GetMinMaxXYZ(const Vector<PT>& input_data, PT& max_p, PT& min_p, DT dt)
	{
		if (input_data.size() == 0) {
			log_error("empty input");
			return false;
		}
		min_p.x = min_p.y = min_p.z = std::numeric_limits<DT>::infinity();
		max_p.x = max_p.y = max_p.z = -std::numeric_limits<DT>::infinity();

		for (unsigned int i = 0; i < input_data.size(); i++) {
			min_p.x = (min_p.x > input_data[i].x) ? input_data[i].x : min_p.x;
			max_p.x = (max_p.x < input_data[i].x) ? input_data[i].x : max_p.x;
			min_p.y = (min_p.y > input_data[i].y) ? input_data[i].y : min_p.y;
			max_p.y = (max_p.y < input_data[i].y) ? input_data[i].y : max_p.y;
			min_p.z = (min_p.z > input_data[i].z) ? input_data[i].z : min_p.z;
			max_p.z = (max_p.z < input_data[i].z) ? input_data[i].z : max_p.z;
		}
		return true;
	}

	/**
	* \brief  compute the dimension in x,y,z of all the voxels according to voxel size and min , max point
	* @param [in] max_p: max x,y,z of point cloud
	* @param [in] min_p: min x,y,z of point cloud
	* @param [in] voxel_param, voxel size
	* @param [out] the dimension in x,y,z of voxels
	* */
	template<typename PT>
	static inline void GetTotalNumOfVoxelGrid(const PT max_p, const PT min_p, const VoxelSizeParams voxel_param, VoxelDimension& voxel_dim)
	{
		if ((max_p.x - min_p.x) != 0 && std::remainder((max_p.x - min_p.x), voxel_param.length_x_of_voxel) == 0)
			voxel_dim.cols_of_voxel = (unsigned int)(std::floor((max_p.x - min_p.x) / voxel_param.length_x_of_voxel));
		else
			voxel_dim.cols_of_voxel = (unsigned int)(std::floor((max_p.x - min_p.x) / voxel_param.length_x_of_voxel) + 1);

		if ((max_p.y - min_p.y) != 0 && std::remainder((max_p.y - min_p.y), voxel_param.length_y_of_voxel) == 0)
			voxel_dim.rows_of_voxel = (unsigned int)(std::floor((max_p.y - min_p.y) / voxel_param.length_x_of_voxel));
		else
			voxel_dim.rows_of_voxel = (unsigned int)(std::floor((max_p.y - min_p.y) / voxel_param.length_x_of_voxel) + 1);

		if ((max_p.z - min_p.z) != 0 && std::remainder((max_p.z - min_p.z), voxel_param.length_z_of_voxel) == 0)
			voxel_dim.depths_of_voxel = (unsigned int)(std::floor((max_p.z - min_p.z) / voxel_param.length_x_of_voxel));
		else
			voxel_dim.depths_of_voxel = (unsigned int)(std::floor((max_p.z - min_p.z) / voxel_param.length_x_of_voxel) + 1);
	}

	/**
	* \brief  compute the voxel grid index according to min x,y,z and a point in voxel
	* @param [in] min_p: min x,y,z of point cloud
	* @param [in] Point: any point of a voxel
	* @param [in] voxel_param, voxel size
	* @param [in] the dimension in x,y,z of voxels
	* @param [out] grid_id , grid index of the voxel which Point lies in
	* */
	template<typename PT>
	static inline void ConvertXYZToVoxelID(const PT min_p, const PT Point, const VoxelSizeParams voxel_param, const VoxelDimension voxel_dim, unsigned long long& grid_id) {
		unsigned int col_idx = (unsigned int)(std::floor((Point.x - min_p.x) / voxel_param.length_x_of_voxel));
		unsigned int row_idx = (unsigned int)(std::floor((Point.y - min_p.y) / voxel_param.length_x_of_voxel));
		unsigned int height_idx = (unsigned int)(std::floor((Point.z - min_p.z) / voxel_param.length_x_of_voxel));

		if (col_idx == voxel_dim.cols_of_voxel)
			col_idx--;
		if (row_idx == voxel_dim.rows_of_voxel)
			row_idx--;
		if (height_idx == voxel_dim.depths_of_voxel)
			height_idx--;

		grid_id = height_idx * (voxel_dim.cols_of_voxel * voxel_dim.rows_of_voxel) + row_idx * voxel_dim.cols_of_voxel + col_idx;
	}

	/**
	* \brief  Interface of downsample filter
	* @param [in] const std::vector<T1> input_data, input point cloud
	* @param [in] leaf_size, downsample leaf size
	* @param [out]  centroid_array, point cloud after donwsample
	* @param [out]  point_to_centriod_array,  point index to centroid index map information
	* @return if success or not
	* */
	template<typename PT, typename DT, typename T>
	static inline bool FilterGridApply(Vector<PT> &input_data, const DT leaf_size, Vector<PT> &centroid_array, Vector<T> &point_to_centriod_array)
	{
		VoxelSizeParams voxel_params;
		voxel_params.length_x_of_voxel = static_cast<float>(leaf_size);
		voxel_params.length_y_of_voxel = static_cast<float>(leaf_size);
		voxel_params.length_z_of_voxel = static_cast<float>(leaf_size);

		//float inverse_leaf_size = 1.0f/leaf_size;
		VoxelDimension voxel_dim;   // dimension of voxel to the leaf size
		PT max_p, min_p;        // max x,y,z and min x, y,z

		// get the max x,y,z of voxel
		DT dt = max_p.x;  // DT must consistent with PT
		GetMinMaxXYZ<PT, DT>(input_data, max_p, min_p, dt);

		GetTotalNumOfVoxelGrid<PT>(max_p, min_p, voxel_params, voxel_dim);

		std::int64_t  total_num_of_grid = static_cast<std::int64_t>(voxel_dim.cols_of_voxel) * static_cast<std::int64_t>(voxel_dim.rows_of_voxel) * static_cast<std::int64_t>(voxel_dim.depths_of_voxel);

		// Check that the leaf size is not too small, given the size of the data
		if (total_num_of_grid > static_cast<std::int64_t>((std::numeric_limits<std::int32_t>::max)()))
		{
			log_info("FilterGridApply Leaf size is too small for the input dataset. Integer indices would overflow.");
			log_info("cols_of_voxel =%d rows_of_voxel =%d depths_of_voxel =%d", voxel_dim.cols_of_voxel, voxel_dim.rows_of_voxel, voxel_dim.depths_of_voxel);
		}
		PointGrid2PointIdx* point_to_subgrid_list = new PointGrid2PointIdx[input_data.size()];

#pragma omp parallel for
		for (int i = 0; i < input_data.size(); i++)
		{
			point_to_subgrid_list[i].cloud_point_index = i;
			PT point = input_data[i];
			ConvertXYZToVoxelID(min_p, point, voxel_params, voxel_dim, point_to_subgrid_list[i].grid_idx);
		}

		std::sort(point_to_subgrid_list, point_to_subgrid_list + input_data.size(), std::less<PointGrid2PointIdx>());

		unsigned int i = 0;
		unsigned int occupied_centriod_cnt = 0;  // total occupied centriod number of the voxel
		while (i < input_data.size())
		{
			unsigned int j = i + 1;
			while ((j < input_data.size()) && (point_to_subgrid_list[i].grid_idx == point_to_subgrid_list[j].grid_idx))
			{
				++j;
			}
			occupied_centriod_cnt++;
			i = j;
		}
		centroid_array.resize(occupied_centriod_cnt);
		point_to_centriod_array.resize(input_data.size());
		T centriod_index = 0;
		PT centroid;
		for (unsigned int i = 0; i < input_data.size();)
		{
			unsigned int point_idx = point_to_subgrid_list[i].cloud_point_index;
			PT first_point = input_data[point_idx];
			centroid = first_point;
			point_to_centriod_array[point_idx] = centriod_index;
			unsigned int j = i + 1;
			while ((j < input_data.size()) && (point_to_subgrid_list[i].grid_idx == point_to_subgrid_list[j].grid_idx))
			{
				unsigned int current_point_idx = point_to_subgrid_list[j].cloud_point_index;
				PT current_point = input_data[current_point_idx];
				centroid.x += current_point.x;
				centroid.y += current_point.y;
				centroid.z += current_point.z;
				point_to_centriod_array[current_point_idx] = centriod_index;
				j++;
			}
			centroid /= static_cast<float> (j - i);
			centroid_array[centriod_index].x = centroid.x;
			centroid_array[centriod_index].y = centroid.y;
			centroid_array[centriod_index].z = centroid.z;
			//centroid_array.points[centriod_index].voxel_idx = point_to_subgrid_list[i].grid_idx;

			i = j;
			centriod_index++;
		}
		delete[] point_to_subgrid_list;
		return true;
	}
};

#endif //DOWNSAMPLE_FILTER_H