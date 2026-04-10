#ifndef MEASUREDEFECT_H
#define MEASUREDEFECT_H

#include "std_header.h"


#include "MeasureBase.h"

//#define DEBUG_DEFECT

#define BOOL_FUNCTION_CHECK(val) {if(!val) return false;}

struct Voxel_in_Z
{
	bool is_occupied;
	unsigned int pt_num;
	unsigned int pt_num_plus, pt_num_minus;
	float defect_sum;
	float defect_sum_plus, defect_sum_minus;
	cv::Point3f left_top;

#ifdef DEBUG_DEFECT
	std::vector<cv::Point3f> points;
#endif // DEBUG_DEFECT

	Voxel_in_Z()
	{
		is_occupied = false;
		pt_num = 0;
		pt_num_plus = 0;
		pt_num_minus = 0;
		defect_sum = 0.f;
		defect_sum_plus = 0.f;
		defect_sum_minus = 0.f;
	}
};


class MeasureDefect {

public:

	MeasureDefect(bool defect_3d=false);
	~MeasureDefect();


	// compute height
	bool MeasureDefectFcn(const float* plane_normal, 
		const float* plane_center, 
		const std::vector<cv::Point3f>& plane_points, 
		std::vector<std::pair<float, cv::Point3f>>& defect, 
		ObstacleInfo* obstacleInfoPtr = nullptr);

	//Parameter settings
	void SetRulerSize(float ruler_width, float ruler_height);
	void GetRulerSize(float& ruler_width, float& ruler_height);

	void SetDefectThreshold(float defect_threshold); //in mm
	void GetDefectThreshold(float& defect_threshold); //in mm


private:
	
	float voxel_width, voxel_height;
	float defect_threshold_in_mm;
	unsigned int min_pt_num_in_voxel;
	unsigned int pt2pt_distance; //scanner property

	bool VoxelizePlane(const std::vector<cv::Point3f>& plane_points, const float y_mean, const float* plane_minmax_xy, std::vector<Voxel_in_Z>& voxels);

	bool AssignDefectResult(const std::vector<Voxel_in_Z>& voxels, const cv::Mat backward_rotation_matrix1, const cv::Mat backward_rotation_matrix2, std::vector<std::pair<float, cv::Point3f>>& defect);

	bool mDefect3D;
};

#endif MEASUREDEFECT_H