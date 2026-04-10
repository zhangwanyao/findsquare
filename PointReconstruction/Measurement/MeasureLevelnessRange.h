#ifndef MEASURELEVELNESSRANGE_H
#define MEASURELEVELNESSRANGE_H

#include "std_header.h"
//#include "..\Common\Voxel.h"

#include "MeasureBase.h" 
//#define DEBUG_MEASURELEVELNESSRANGE
#include "..\Common\VariableDefine.h"


#define BOOL_FUNCTION_CHECK(val) {if(!val) return false;}
#define BOOL_FUNCTION_CHECK_CONTINUE(val) {if(!val) continue;}
#define BOOL_FUNCTION_CHECK_BREAK(val) {if(!val) break;}
#define PRINT_FUNCTION_TIME(val) do { \
	time_t start = clock(); \
	val; \
	std::string output = #val; \
	cout << output.substr(0, output.find_first_of("(")) + " time = " <<float(clock()-start)/CLOCKS_PER_SEC<<" s"<<endl; \
}while (0)


struct Voxel_LNR
{
	bool is_occupied;
	unsigned int pt_num;
	float z_sum;
	cv::Point3f left_top;

	Voxel_LNR()
	{
		is_occupied = false;
		pt_num = 0;
		z_sum = 0.f;
	}
};


class MeasureLevelnessRange {

public:
	MeasureLevelnessRange();
	~MeasureLevelnessRange();

	bool MeasureLevelnessRangeFcn(const float* plane_normal,
		const float* ground_normal,
		const std::vector<cv::Point3f>& plane_points,
		MeasurementResultValueValuesPoints& levelness,
		std::vector<cv::Point3f>& actual_vertices,
		std::vector<std::pair<float, cv::Point3f>> * p_full_levelness=NULL);

	//Parameter settings
	void SetSetRulerDistanceToPlaneEdge(float distance_from_edge);
	void GetRulerDistanceToPlaneEdge(float& distance_from_edge);

	void SetRulerSize(float ruler_square_length);
	void GetRulerSize(float& ruler_square_length);
	///added by bailing.li@unre.com
	///putIn the pos=300(measure_ROI_length=300),but the outPos!=300, pos has changed by some reasons,so i change it back outside.
	///for the private value, i had to add this method.
	float  get_measure_ROI_length() { return measure_ROI_length; }
	/*added by Tao*/
	void ReArrangeRuleWithCompass(std::vector<cv::Point2f>& markerPoint, int rules,float CompassAng,
		MeasurementResultValueValuesPointsT& levelnessTemp, std::vector<cv::Point3f>& actual_verticesTemp,
		MeasurementResultValueValuesPoints& levelness, std::vector<cv::Point3f>& actual_vertices);
	/*end adding by Tao*/
	bool GetLevelnessLocalRulersFromPlaneCore(const std::vector<cv::Point3f>& plane_points, const std::vector<cv::Point3f>& levelness_pts, std::vector<MeasurementRulerverticeStruct>& levelness_local_ruler_vertice_intersect_pts_dis);

protected:
	
	//parameters
	int min_num_pts_inrulerband;
	float measure_ROI_length;
	float normal_zero_dir_threshold;
	float plane_width_threshold;
	float measure_ROI_dist_from_plane_edge;
	bool has_imu;
	float offset_ROI_actualPts;

	cv::Point3f origin, rotated_origin;

	/*rotate horizontal planes & ground normal to ensure one edge of plane aligned with X-AXIS*/
	bool RotateHorizontalPlanes2XAxis(const std::vector<cv::Point3f>& plane1_points, const float* plane1_normal,
		const std::vector<cv::Point3f>& plane2_points, const float* ground_normal,
		float* rotation_angle_around_z,
		std::vector<cv::Point3f>& rot_plane1_points,
		std::vector<cv::Point3f>& rot_plane2_points,
		float* rot_ground_normal);

	/*rotate horizontal plane & ground normal to ensure one edge of plane aligned with X-AXIS*/
	bool RotateHorizontalPlane2XAxis(const std::vector<cv::Point3f>& plane_points, const float* plane_normal, 
		const float* ground_normal,
		float* rotation_angle_around_z, 
		std::vector<cv::Point3f>& rot_plane_points,
		float* rot_ground_normal);

	/*ruler 1~4 @ corners & ruler 5 @ center*/
	void AssignMeasureVirtualCorner(int Rulers,const float* minmax_xyz, const float edgesize,
		const int times_ifrerun_x, const int times_ifrerun_y,
		const float movesize_x, const float movesize_y,
		const int ruler,
		float* virtual_corner);

	int ComputeFixLocationZMean(const std::vector<cv::Point3f>& plane1_points, const std::vector<cv::Point3f>& plane2_points, 
		 const float * minmax_xyz, const cv::Mat rotation_matrix, float * mean_z_1, float * mean_z_2, float * virtual_corner, cv::Point2f & single_actual_vertex);

	std::pair<int, int> ComputeCornersCenterZMean(int Rules,const std::vector<cv::Point3f>& plane1_points, const std::vector<cv::Point3f>& plane2_points,
		const float* minmax_xyz, const int ruler, const cv::Mat rotation_matrix,
		float* mean_z_1, float* mean_z_2, float* virtual_corner, cv::Point2f& single_actual_vertex);

	std::pair<int, int> ComputeCornersCenterZMeanAll(cv::Point2f &Pt, float &mLen, const std::vector<cv::Point3f>& plane1_points, const std::vector<cv::Point3f>& plane2_points,
		const float* minmax_xyz, const cv::Mat rotation_matrix,
		float* mean_z_1, float* mean_z_2, float* virtual_corner, cv::Point2f& single_actual_vertex);
	std::pair<int, int> ComputeCornersCenterZMeanFix(cv::Point2f &Pt, float &mLen, const std::vector<cv::Point3f>& plane1_points, const std::vector<cv::Point3f>& plane2_points,
		const float* minmax_xyz, const cv::Mat rotation_matrix,
		float* mean_z_1, float* mean_z_2, float* virtual_corner, cv::Point2f& single_actual_vertex);


private:
	
	// compute level range
	bool ComputeLevelnessRange(const std::vector<cv::Point3f> &plane_points,
		const float* minmax_xyz,
		const  cv::Mat rotation_matrix_to_Z,
		const float rotation_angle_around_z,
		MeasurementResultValueValuesPoints& levelness,
		std::vector<cv::Point3f>& actual_vertices);

	bool ComputeLevelnessRangeAll(const std::vector<cv::Point3f>& plane_points, const float * minmax_xyz, const cv::Mat rotation_matrix_to_z, const float rotation_angle_around_z, MeasurementResultValueValuesPoints & levelness, std::vector<cv::Point3f>& actual_vertices);

	bool SaveROIPoints(const std::vector<cv::Point3f>& plane_points, const float* virtual_corner, const int ruler);

	///Tao VoxelDataStruct::VoxelGrid_Base local_rulerplane_pts;


	//added by simon.jin@unre.com  start
	/**
	* \brief voxelize plane
	* \param plane_points: the source points
	* \param plane_minmax_xy: input cloud point
	* \param voxels:in and output voxels points
	* \return bool
	*/
	bool VoxelizePlane(const std::vector<cv::Point3f>& plane_points, const float* plane_minmax_xy, std::vector<Voxel_LNR>& voxels);
	/**
	* \brief voxelize plane
	* \param voxels:voxels points
	* \param levelness: output the voxelized levelness
	* \return bool
	*/
	bool ComputeLevelnessResult(const std::vector<Voxel_LNR>& voxels, float rotation_angle_around_z, std::vector<std::pair<float, cv::Point3f>>& levelness);

private:
	float voxel_width;
	float voxel_height;
	float levelness_threshold;
	int  min_pt_num_in_voxel;
	//bool  
	//added by simon.jin@unre.com end
};
#endif MEASURELEVELNESSRANGE