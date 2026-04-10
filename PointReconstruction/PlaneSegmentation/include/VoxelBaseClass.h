#ifndef VOXEL_BASE_CLASS_H
#define VOXEL_BASE_CLASS_H

#include "config.h"
#include "plane_seg_inf.h"
#include "DataStruct.h"


class VoxelBaseClass {

protected:

	//number of occupied voxel (occupied = at least contain predifined points)
	int num_of_occupied_voxel;

	//pt cloud x,y,z
	PointArray pt_cloud_xyz;

	//pt cloud normal
	PointArray pt_cloud_normal;

	//dimensions of voxels
	int cols_of_voxel, rows_of_voxel, depths_of_voxel;

	//total number of voxel grids
	unsigned long long total_num_of_voxel_grid;

	//length of voxel in -x, -y and -z direction
	double length_x_of_voxel, length_y_of_voxel, length_z_of_voxel;

	double length_x_of_voxel_inverse, length_y_of_voxel_inverse, length_z_of_voxel_inverse;

	//min and max x, y and z of point cloud
	double min_x, max_x, min_y, max_y, min_z, max_z;

	int num_of_bridge, num_of_good_bridge, num_of_pseudo_bad_bridge, num_of_real_bad_bridge, num_of_single_bridge, max_num_of_bridge_planes;

protected:

	//initialize variables 
	virtual void Init();

	virtual void FreeMemory();

	//reset class status
	//virtual void Reset();

	//get min and max x, y, z
	virtual bool GetMinMaxXYZ(PointArray input_data);

	//get total number of voxel grids
	virtual void GetTotalNumOfVoxelGrid();

	//this func is mainly used for voxel based edge detection
	virtual void GetTotalNumOfVoxelGrid(const int buffer);

	//convert x,y,z coordinate to voxel id
	virtual unsigned long long ConvertXYZToVoxelID(const float x, const float y, const float z);

	virtual void ConvertVoxelIDTo3dID(const ModuleStruct::Point3f &pt_xyz, int &row_idx, int &col_idx, int &depth_idx);

public:

	VoxelBaseClass();

	virtual ~VoxelBaseClass();

	float point_density;
};




class VoxelMergeBaseClass
{
	public:
		VoxelMergeBaseClass(PlaneVoxelItem* voxel_grid_input, PlaneFitThresholds* plane_seg_thresholds);
		~VoxelMergeBaseClass();

		//initialize variables 
		virtual void Init();

		virtual void FreeMemory();

		void ResetVoxel();

		void Compute(unsigned int point_cnt);

		//push data for calculating covariance matrix
		inline void Push(const float x, const float y, const float z) {
			voxel_grid->sums.sum_x += x; voxel_grid->sums.sum_y += y; voxel_grid->sums.sum_z += z;
			voxel_grid->sums.sum_xx += x*x; voxel_grid->sums.sum_yy += y*y; voxel_grid->sums.sum_zz += z*z;
			voxel_grid->sums.sum_xy += x*y; voxel_grid->sums.sum_xz += x*z; voxel_grid->sums.sum_yz += y*z;
			//voxel_grid->points.size++;	// Elaine Li - remove it, it has been assigned by the caller
		}

		//push data by point count for calculating covariance matrix
		inline void Push(const SumforCovariance* sums) {
			voxel_grid->sums.sum_x += sums->sum_x; voxel_grid->sums.sum_y += sums->sum_y; voxel_grid->sums.sum_z += sums->sum_z;
			voxel_grid->sums.sum_xx += sums->sum_xx; voxel_grid->sums.sum_yy += sums->sum_yy; voxel_grid->sums.sum_zz += sums->sum_zz;
			voxel_grid->sums.sum_xy += sums->sum_xy; voxel_grid->sums.sum_xz += sums->sum_xz; voxel_grid->sums.sum_yz += sums->sum_yz;
			//voxel_grid->points.size += point_cnt;		// Elaine Li - assign it by the caller
		}

		inline void Push(const ModuleStruct::Point3f pt) {
			Push(pt.x, pt.y, pt.z);
		}

		//get plane center
		inline void CalPlaneCenter(int voxel_index) {
				if (voxel_grid->points.size <= 0) return;
				double point_num_inverse = (double)1.0 / voxel_grid->points.size;
				//plane center
				voxel_grid->plane_center.x = (float)(voxel_grid->sums.sum_x*point_num_inverse);
				voxel_grid->plane_center.y = (float)(voxel_grid->sums.sum_y*point_num_inverse);
				voxel_grid->plane_center.z = (float)(voxel_grid->sums.sum_z*point_num_inverse);
		}		

		inline float ComputeNormalsSimilarity(ModuleStruct::Point3f plane_normal) 
		{
			return voxel_grid->plane_normal.x * plane_normal.x + voxel_grid->plane_normal.y * plane_normal.y + voxel_grid->plane_normal.z * plane_normal.z;
		}

		inline float GetPointPlaneDist(ModuleStruct::Point3f point)
		{
			return std::fabs(voxel_grid->plane_normal.x * (point.x - voxel_grid->plane_center.x) +
				voxel_grid->plane_normal.y * (point.y - voxel_grid->plane_center.y)+
					voxel_grid->plane_normal.z * (point.z - voxel_grid->plane_center.z));
		}
		

		void GetVoxelSums(PointArray* point_array);


		void InputVoxelPoints(PointInVoxelArray *point_array) 
		{
			voxel_grid->points.point_idx = point_array->point_idx;
			voxel_grid->points.size = point_array->size;
		}
		
		void GetVoxelNeighborItem(unsigned int neigbour_index, unsigned int neighbour_voxel_occupied_index, PlaneVoxelItem* plane_voxel_array);
		
		void UpdateVoxelNeighborItem(NeighborItem* neighbour_item, PlaneVoxelItem* neighbour_parent_voxel);

		void OutputVoxelPoints(PointInVoxelArray *point_array)
		{
			point_array->size = voxel_grid->points.size;
			point_array->point_idx = voxel_grid->points.point_idx;
		}
		// compute do not change the voxel normal , voxel center and sums
		void Compute(unsigned int point_cnt, SumforCovariance sums, ModuleStruct::Point3f &plane_normal, ModuleStruct::Point3f &plane_center);

	protected:

		PlaneVoxelItem* voxel_grid;
		PlaneFitThresholds* plane_fit_thresholds;
};


#endif// VOXEL_BASE_CLASS_H
