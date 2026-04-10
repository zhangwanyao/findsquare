#ifndef _UTIL_PLANE_SEG_HPP_
#define _UTIL_PLANE_SEG_HPP_

#include "NormalStruct.hpp"
#include "plane_seg_inf.h"

namespace planeSegmentationInLine
{
	static inline bool Features_PlaneSegInputConvert(const std::vector<Point3f>input_points, const VoxelParams voxel_para, NormalEstimationResults* inputStruct, NormalEstimationInterface* normal_inputStruct)
	{
		if (input_points.empty())
		{
			return false;
		}

		normal_inputStruct->density = inputStruct->density;
		normal_inputStruct->points.size = static_cast<unsigned int>(input_points.size());
		normal_inputStruct->points.points = new Point3f[input_points.size()];
		for (unsigned int i = 0; i < normal_inputStruct->points.size; i++)
		{
			normal_inputStruct->points.points[i] = input_points[i];
		}

		normal_inputStruct->voxel_para = voxel_para;
		normal_inputStruct->normals.size = static_cast<unsigned int>(inputStruct->normals.size());
		if (!inputStruct->normals.empty())
		{
			normal_inputStruct->normals.points = new Point3f[inputStruct->normals.size()];
			for (unsigned int i = 0; i < normal_inputStruct->normals.size; i++)
			{
				normal_inputStruct->normals.points[i] = inputStruct->normals[i];
			}
		}

		normal_inputStruct->voxel_array_size = static_cast<unsigned int>(inputStruct->voxel_array.size());
		normal_inputStruct->voxel_array = new VoxelItem[inputStruct->voxel_array.size()];	
		for (int i = 0; i < static_cast<int>(inputStruct->voxel_array.size()); i++)
		{
			normal_inputStruct->voxel_array[i].is_good_voxel = inputStruct->voxel_array[i].is_good_voxel;
			normal_inputStruct->voxel_array[i].grid_idx = inputStruct->voxel_array[i].grid_idx;
			normal_inputStruct->voxel_array[i].points.size = static_cast<unsigned int>(inputStruct->voxel_array[i].points_index.size());
			normal_inputStruct->voxel_array[i].points.point_idx = new unsigned int[inputStruct->voxel_array[i].points_index.size()];

			for (int j = 0; j < static_cast<int>(inputStruct->voxel_array[i].points_index.size()); j++)
			{
				normal_inputStruct->voxel_array[i].points.point_idx[j] = inputStruct->voxel_array[i].points_index[j];
			}
		}

		normal_inputStruct->grid_to_occupied_voxel_idx_size = static_cast<unsigned int>(inputStruct->grid_to_occupied_voxel_idx.size());
		normal_inputStruct->grid_to_occupied_voxel_idx = new unsigned int[normal_inputStruct->grid_to_occupied_voxel_idx_size];
		for (unsigned int i = 0; i < normal_inputStruct->grid_to_occupied_voxel_idx_size; i++)
		{
			normal_inputStruct->grid_to_occupied_voxel_idx[i] = inputStruct->grid_to_occupied_voxel_idx[i];
		}

		//release
		inputStruct->grid_to_occupied_voxel_idx.clear();
		inputStruct->grid_to_occupied_voxel_idx.shrink_to_fit();
		inputStruct->normals.clear();
		inputStruct->normals.shrink_to_fit();
		for (int i = 0; i < inputStruct->voxel_array.size(); i++)
		{
			inputStruct->voxel_array[i].points_index.clear();
			inputStruct->voxel_array[i].points_index.shrink_to_fit();
		}
		inputStruct->voxel_array.clear();
		inputStruct->voxel_array.shrink_to_fit();
		inputStruct->voxel_idx.clear();
		inputStruct->voxel_idx.shrink_to_fit();
		return true;
	}

	static inline bool Features_PlaneSegOutputConvert(const std::vector<Point3f>input_points, PlaneSegmentationOutput* plane_seg, std::vector<PlaneSegOutput>& out_planes)
	{
		unsigned int total_size = 0;
		for (unsigned int i = 0; i < plane_seg->size; i++)
		{
			if (plane_seg->planes[i].points.size > 0)
				total_size++;
		}
		out_planes.resize(total_size);

		unsigned int n = 0;
		for (unsigned int i = 0; i < plane_seg->size; i++)
		{
			if (plane_seg->planes[i].points.size > 0)
			{
				out_planes[n].plane_center = plane_seg->planes[i].plane_center;
				out_planes[n].plane_normal = plane_seg->planes[i].plane_normal;
				out_planes[n].plane_mse = plane_seg->planes[i].plane_mse;
				out_planes[n].point_ids.resize(plane_seg->planes[i].points.size);
				out_planes[n].points.resize(plane_seg->planes[i].points.size);
				for (int j = 0; j < static_cast<int>(out_planes[n].point_ids.size()); j++)
				{
					out_planes[n].point_ids[j] = plane_seg->planes[i].points.point_idx[j];
					out_planes[n].points[j] = input_points[out_planes[n].point_ids[j]];
				}
				out_planes[n].plane_area = plane_seg->planes[i].plane_area;
				n++;
				//log_debug("plane %d plane area = %f", i, out_planes[i].plane_area);
			}
		}
		return true;
	}

	static inline bool Features_PlaneSegFreeMemory(NormalEstimationInterface normal_input)
	{
		/*if (normal_input.grid_to_occupied_voxel_idx != NULL)
		{
			delete[] normal_input.grid_to_occupied_voxel_idx;
			normal_input.grid_to_occupied_voxel_idx = NULL;
		}*/

		if (normal_input.points.points != NULL)
		{
			delete[] normal_input.points.points;
			normal_input.points.points = NULL;
		}

		if (normal_input.normals.points != NULL)
		{
			delete[] normal_input.normals.points;
			normal_input.normals.points = NULL;
		}

		/*if (normal_input.voxel_array != NULL)
		{
			for (int i = 0; i < normal_input.voxel_array_size; i++)
			{
				if (normal_input.voxel_array[i].points.point_idx != NULL);
				{
					delete[] normal_input.voxel_array[i].points.point_idx;
					normal_input.voxel_array[i].points.point_idx = NULL;
				}
			}
			delete[] normal_input.voxel_array;
			normal_input.voxel_array = NULL;
		}*/
		return true;
	}

	static inline bool Features_PlaneSegOutFreeMemory(PlaneSegmentationOutput* plane_seg_out)
	{
		if (plane_seg_out->planes != NULL)
		{
			for (unsigned int i = 0; i < plane_seg_out->size; i++)
			{
				if (plane_seg_out->planes[i].points.point_idx != NULL)
				{
					delete[] plane_seg_out->planes[i].points.point_idx;
					plane_seg_out->planes[i].points.point_idx = NULL;
				}
			}
			delete[] plane_seg_out->planes;
			plane_seg_out->planes = NULL;
		}
		return true;
	}
}

#endif // _PLANE_SEG_INF_H_