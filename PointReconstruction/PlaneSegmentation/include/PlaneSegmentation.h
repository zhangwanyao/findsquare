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

#ifndef PLANE_SEGMENTATION_H_
#define PLANE_SEGMENTATION_H_

#include "DataStruct.h"
#include "VoxelBaseClass.h"
#include "BadVoxelStruct.h"
#include "VoxelDbgStruct.h"
#include "ParentVoxelStruct.h"
#include "PointsGroupPlane.h"
#include "PlaneMerge.h"
#include "in_out_data.hpp"

 // Plane segmentation parameter data structure
struct PlaneSegParams
{
	PlaneSegParams(
		float length_x_of_voxel = 50.f,
		float length_y_of_voxel = 50.f,
		float length_z_of_voxel = 50.f,
		float min_plane_dist_of_two_voxel_threshold = 10.0f,
		float min_dist_of_point2voxel_threshold = 2.5f,
		float max_normal_angle_of_two_voxel_threshold = 10.0f,
		float max_mse_of_voxel_threshold = 2.5f,
		int min_point_num_of_plane = 500,
		//float horizontal_or_vertical_plane_normals_threshold = 0.9f,
		//float ground_height_threshold = 0.0f,
		//float ceiling_height_threshold = 0.0f,
		//float min_width_of_a_valid_plane_threshold = 200.0f,
		//float min_area_of_a_valid_plane_threshold = 500.0f * 500.0f,
		//float height_diff_connected_to_ground_threshold = 300.0f,
		//float min_threshold_of_inter_angle = 85.0f,
		//float max_threshold_of_inter_angle = 95.0f,
		int min_voxel_num_of_plane = 5,
		float max_mse_of_plane = 2.5f,
		int min_point_num_of_voxel = 10,
		float min_plane_dist_of_2plane = 5.0f,
		float max_normal_angle_of_2plane = 5.0f,
		float max_normal_angle_of_edge_point = 45.0f,
		float max_high_mse_ratio = 0.02f,
		//unsigned int compute_normal_min_points = 5,
		unsigned int min_good_group_in_plane = 3,
		unsigned int min_point_num_of_flat_voxel = 3,
		float max_dist_of_2voxel_connected = 50.f,
		unsigned int min_good_plane_voxel_size = 20
	) {
		voxel_params.length_x_of_voxel = length_x_of_voxel;
		voxel_params.length_y_of_voxel = length_y_of_voxel;
		voxel_params.length_z_of_voxel = length_z_of_voxel;

		plane_seg_thresholds.THRESHOLD_MIN_PLANE_DIST_OF_TWO_VOXEL = min_plane_dist_of_two_voxel_threshold;
		plane_seg_thresholds.THRESHOLD_MIN_DIST_OF_POINT2VOXEL = min_dist_of_point2voxel_threshold;
		plane_seg_thresholds.THRESHOLD_MAX_NORMAL_ANGLE_OF_TWO_VOXEL = max_normal_angle_of_two_voxel_threshold;
		plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_VOXEL = max_mse_of_voxel_threshold;
		plane_seg_thresholds.THRESHOLD_MIN_POINT_NUM_OF_PLANE = min_point_num_of_plane;
		plane_seg_thresholds.THRESHOLD_MIN_VOXEL_NUM_OF_PLANE = min_voxel_num_of_plane;
		plane_seg_thresholds.THRESHOLD_MAX_MSE_OF_PLANE = max_mse_of_plane;
		plane_seg_thresholds.THRESHOLD_MIN_POINT_NUM_OF_VALID_NORMAL_VOXEL = min_point_num_of_voxel;
		plane_seg_thresholds.THRESHOLD_MIN_DIST_OF_2PLANE = min_plane_dist_of_2plane;
		plane_seg_thresholds.THRESHOLD_MAX_NORMAL_ANGEL_OF_2PLANE = max_normal_angle_of_2plane;
		plane_seg_thresholds.THRESHOLD_MAX_NORMAL_ANGEL_OF_EDGE_POINT = max_normal_angle_of_edge_point;
		plane_seg_thresholds.THRESHOLD_MAX_HIGH_MSE_RATIO = max_high_mse_ratio;
		//plane_seg_thresholds.THRESHOLD_COMPUTE_NORMALS_MIN_POINTS = compute_normal_min_points;
		plane_seg_thresholds.THRESHOLD_MIN_NUM_OF_GOOD_GROUP_IN_PLANE = min_good_group_in_plane;
		plane_seg_thresholds.THRESHOLD_MIN_POINT_NUM_OF_FLAT_VOXEL = min_point_num_of_flat_voxel;
		plane_seg_thresholds.THRESHOLD_DIST_OF_2VOXEL_CONNECTED = max_dist_of_2voxel_connected;
		plane_seg_thresholds.THRESHOLD_MIN_GOOD_PLANE_VOXEL_SIZE = min_good_plane_voxel_size;

		/*plane_classify_thresholds.THRESHOLD_HORIZONTAL_OR_VERTICAL_PLANE_NORMALS = horizontal_or_vertical_plane_normals_threshold;
		plane_classify_thresholds.THRESHOLD_GROUND_HEIGHT = ground_height_threshold;
		plane_classify_thresholds.THRESHOLD_CEILING_HEIGHT = ceiling_height_threshold;
		plane_classify_thresholds.THRESHOLD_MIN_WIDTH_OF_A_VALID_PLANE = min_width_of_a_valid_plane_threshold;
		plane_classify_thresholds.THRESHOLD_MIN_AREA_OF_A_VALID_PLANE = min_area_of_a_valid_plane_threshold;
		plane_classify_thresholds.THRESHOLD_HEIGHT_DIFF_CONNECTED_TO_GROUND = height_diff_connected_to_ground_threshold;
		plane_classify_thresholds.THRESHOLD_MIN_L_SHAPE_ANGLE = min_threshold_of_inter_angle;
		plane_classify_thresholds.THRESHOLD_MAX_L_SHAPE_ANGLE = max_threshold_of_inter_angle;*/
	}

	// voxel length in x, y and z directions
	VoxelParams voxel_params;

	// Predefined threshold for plane segmentation
	PlaneFitThresholds plane_seg_thresholds;

	// Predefined threshold for plane classification
	//PlaneClassifyThresholds plane_classify_thresholds;
};

// Plane segmentation class definition
class PlaneSegmentation :public VoxelBaseClass
{
protected:

	//define movement
	const int movement_x[26] = { -1,  0,  1, -1,  0,  1, -1,  0,  1, -1,  0,  1, -1,  1, -1,  0,  1, -1,  0,  1, -1,  0,  1, -1,  0,  1 };
	const int movement_y[26] = { -1, -1, -1,  0,  0,  0,  1,  1,  1, -1, -1, -1,  0,  0,  1,  1,  1, -1, -1, -1,  0,  0,  0,  1,  1,  1 };
	const int movement_z[26] = { 1,  1,  1,  1,  1,  1,  1,  1,  1,  0,  0,  0,  0,  0,  0,  0,  0, -1, -1, -1, -1, -1, -1, -1, -1, -1 };

	SysCntrlParams sys_control_para;

	SegCntrlParams config_params;

	// Plane segmentation parameter setting
	PlaneSegParams plane_seg_params;

	// Occupied Voxel Array
	PlaneVoxelArray plane_voxel_array;

	// Plane merge elements, the size is the same as plane_voxel_array
	PlaneMergeItem* plane_merge_element;

	// Plane Merge output
	PlaneMergeOutput plane_merge_out;

	// Plane segmentation output
	PlaneSegmentationOutput plane_seg_out;

	// Total number of good voxels in occupied voxel array
	unsigned int good_voxel_size;

	// Total number of pseudo bad voxels in occupied voxel array
	unsigned int pseudo_bad_voxel_size;

	/**
	* \brief Total number of  voxels who may be merged by point-to-plane in occupied voxel array
	* this number will not change after generating reference planes by good voxels
	*/
	unsigned int point_merged_voxel_size;

	// Total number of  voxels who must be noise or merged by point-to-plane mode in occupied voxel array
	/**
	* \brief Total number of  voxels who may be noise or merged by point-to-plane mode in occupied voxel array
	* this number is less than point_merged_voxel_size, will be change as new point added
	*/
	unsigned int bad_voxel_size;

	// Total number of plane which parent voxel  is good voxel
	unsigned int good_voxel_plane_size;

	// Total number of plane which parent voxel  is pseudobad voxel
	unsigned int pseudobad_voxel_plane_size;

	// real bad voxel merge output info
	BadVoxelMergeOutpuItem* bad_voxel_merge_item;

	// grid index to occupied voxel index array for finding out voxel_idx in occupied voxel array from grid_idx in 3D Space
	unsigned int* grid_to_occupied;

	// the size of grid_to_occupied, is equal to max grid number
	unsigned long long  grid_to_occupied_size;

	// voxel index of occupied voxel array to plane index
	unsigned int *voxel_to_plane_idx;

	// occupied voxel idx of each point voxel from normal estimation module
	unsigned int *point_to_voxel_idx_nm;

	//if is good  voxel  according to its mean squre error from normal estimation module
	//bool *is_good_voxel_by_mse;

	unsigned int* voxel_idx_to_bad_voxel; // occupied voxel to bad voxel index

	//RemainerGroupArray groups_array;  all the group in ramining voxels who are the neighbors

	SamePlaneGroupArray same_plane_group_array; // the same planes  group index array

	//PointsGroupPlaneArray points_group_extended_planes;  find extension of the existing planes  in points groups of bad voxel

	PointsGroupPlaneArray points_group_remaning_planes; // find remaining planes  in points groups of bad voxel

	PlaneMergeOutput points_group_plane_merge_out; //total array for points_group_extended_planes + points_group_remaning_planes

	float min_high_mse_ratio;
	/**
	* \brief the mse threhhold of mini plane whose flat voxels is few
	*/
	float max_mse_of_mini_plane;

	std::string current_out_path;

	//PlaneMergeDebugArray plane_merge_debug_array;
	DebugConfigParams debug_config_params;

	void Reset();

	void FreeMemory();

	void VerifyPlaneSegParameters();

	void InitializeParams(const NormalEstimationInterface inputStruct);

	bool CheckInputParameters(const PointArray point_array, NormalEstimationInterface * inputStruct);

	// Create voxel and assign its data for all occupied voxels
	void CreateVoxels(VoxelItem* &voxel_array);

	// Find out the parent of all good voxels
	void IdentifyParentOfGoodVoxels();

	// Merge all good voxels into its similar plane
	void MergeGoodVoxels();

	// Find out the parent of all pseudo bad voxels
	void IdentifyParentOfPseudoBadVoxels();

	// Merge all pseudo bad voxels into its similar plane
	void MergePseudoBadVoxels(PlaneMergeOutput *pseudobad_plane_merge_out);

	// apply the resources of bad_voxel_merge_item
	void CreateBadVoxelItems(bool is_good_voxel_plane);

	// Find out the parent of points of all bad voxels and pseudo bad voxels by plane's parent voxel type
	void IdentifyParentOfBadVoxelsWithRef(bool is_first);
	void IdentifyParentOfBadVoxelsWithRefNm(bool is_first);


	// Find best plane id to merge for bad voxel points
	unsigned int FindBestPlaneIDToMerge(BadVoxelMergeOutpuItem* voxel, unsigned int k);

	// Merge all points in bad voxels into its similar plane
	void MergeBadVoxels();

	// Assign plane outputs when have no same planes
	void AssignPlaneOutputsNoSamePlane();

	// Assign plane outputs
	void AssignPlaneOutputs();

	//void get one plane area
	void GetOnePlaneArea(PlaneItem* plane);

	// get plane area
	void GetPlaneArea();

	// get the plane index in plane_merge_out from a parent voxel index,if can't find return false
	bool GetPlaneIdxfromParentVoxel(unsigned int parent_voxel_idx, unsigned int plane_idx);

	//for voxel neighbor info file output
	bool GetVoxelNeigbourDebugInfo(const std::string output_path, VoxelNeighborDebugType debug_type);

	bool IdentifyParentOfVoxelsMethodNoBridge(PlaneVoxelType voxeltype, PlaneVoxelType parent_voxel_type);

	//// Find out the parent of in all voxels with participation of bridges
	//bool IdentifyParentVoxelsMethodWithBridge(PlaneVoxelType voxeltype, PlaneVoxelType parent_voxel_type, int max_bridge_plane_cnt);

	//  point debug info include voxel index, parent index, etc
	void GetPointDebugInfo(ModuleStruct::Point3f &point);

	//  update the voxel's neighbour normal and neighbour flag by the plane normal after plane merging completed
	void UpdateNeighborFlag();

	//  recompute all the occpupied voxels's neighbour normal and neighbour flag  by their banlanced normal
	//bool GetNeighborFlag();

	// get average mse of all the voxels of a plane
	//bool GetPlaneMse(VoxelArray voxels, float &plane_mse);

	bool ClearSums(SumforCovariance *sums);

	// add  sums_from to sums_to
	bool PushSums(SumforCovariance *sums_to, SumforCovariance *sums_from);

	// assign sums_from to sums_to
	bool AssignSums(SumforCovariance *sums_to, SumforCovariance *sums_from);

	bool PushPoint(SumforCovariance *sums_to, ModuleStruct::Point3f point);
	bool PopPoint(SumforCovariance *sums_to, ModuleStruct::Point3f point);

	// get the specificed voxel's mse by computing point to plane distance
	bool GetVoxelDistMse(unsigned int voxel_idx, float& dist_mse);

	// get the specificed voxel's average mse by computing point to plane distance
	bool GetVoxelAvgMse(unsigned int voxel_idx);

	// get average mse of all the voxels of a plane  by computing point to plane distance, mse is get by avg of all voxels mse
	bool GetPlaneDistMseByVoxelAvg(VoxelArray voxels, float& plane_mse);

	// get average mse of all the voxels of a plane  by computing point to plane distance, mse is get by avg distance of all the points in all voxels
	bool GetPlaneDistMseByPointAvg(PlaneVoxelItem *parent_voxel, VoxelArray voxels, float& plane_mse);

	bool NeighborParentCompare(ParentIDCondType *condition, ParentVoxelIdentifyUpdateItem * it, unsigned char &update_flag);

	// Identify parent voxel according to input 5 conditions
	bool ParentVoxelIdentifyCondition(ParentIDCondType *condition, const unsigned int parent_voxel_idx, const unsigned int neighbour_parent_voxel_idx, unsigned char &update_flag);
	//get the voxel normal whose mse is <threshold (THRESHOLD_MAX_MSE_OF_VOXEL)
	bool GetVoxelNomalByMse(void);

	//if flat voxel have no good normal , compute it normal with weighted normal
	bool ComputeVoxelAvgNormalMse(void);

	// for debug , output lost plane info in process of merging pseudo bad voxels
	bool OutputPseudoBadMergeLostPlaneInfo(const std::string output_path, PlaneMergeOutputItem *pseudo_bad_planes, unsigned int  pseudo_bad_plane_size);

	// merge the remaining points whose normal is similar with the plane found before
	void MergeBadVoxelsByNormalDiff();

	bool GetPlaneDistMse(PlaneItem *plane, float& plane_mse);

	bool GetPlaneDistMse(PlaneMergeOutputItem* plane, float& plane_mse);

	void FreeBadVoxelItem();
	void AddPlaneToSamePlaneGroup(unsigned int plane_in_group, const unsigned int total_plane_size, bool**is_same_plane, bool**is_connected);

	// mergeing the planes  with both  same planes and  connected
	void IdentifySamePlanes();

	// find the planes  with both  same normal  and  plane distance < threshold
	void IdentifySameNormalPlanes(bool **is_same_plane, unsigned int plane_size);

	// find the planes  who are connected
	void IdentifyConnectedPlanes(bool ** is_connected, unsigned int plane_size);
	void IdentifyConnectedPlanesNew(bool ** is_connected, unsigned int plane_size);
	void IdentifyConnectedPlanesFromNeigbors(bool **is_connected, unsigned int bad_voxel_idx, unsigned int plane_idx);
	void IdentifyConnectedPlanesFromBadVoxel(bool **is_connected, const unsigned int bad_voxel_idx);
	void IdentifyConnectedPlanesFromNeighbors(bool **is_connected, const unsigned int bad_voxel_idx, const unsigned int first_plane_idx\
		, const std::vector<unsigned int> first_points);
	// Identfiy and Group all the planes in points group of all the bad voxels
	void PointsGroupPlaneMergeFromRemainer();

	void VoxelPoint(unsigned int voxel_idx);

	bool Identify2VoxelCLoseByDistance(const unsigned int first_voxel, const unsigned int sec_voxel, const float distance);

	/**
	* \brief  identify parent of voxels with refence planes
	*/
	bool IdentifyParentOfVoxelsWithRefPlane(PlaneVoxelType voxeltype);

	/**
	* \brief find out the parent of pseudobad voxels who are connected with all good voxels
	*/
	void IdentifyParentOfPseudoBadConnectWithGood();

	/**
	* \brief merge pseudobad voxels who are connected with all good voxels
	*/
	void MergePseudoBadConnectWithGood();

	/**
	* \brief Change the pseudobad voxels connected from goodplane into non-overall voxels
	*/
	void InvalidOverallForPBWithGoodPlane();

	/**
	* \brief merge extended parts according to  reference planes 
	*/
	void MergeExtendPartWithRef(PlaneMergeOutput *ref_planes);

	/**
	* \brief combine the planes together with plane_merge_out
	*/
	void CombinePlanes(PlaneMergeOutput *planes);

	/**
	* \brief compute good or pseudobad average normal acording to its good neighbors
	*/
	bool ComputeAvgNormal(unsigned int voxel_idx, ModuleStruct::Point3f &avg_normal);

	/**
	* \brief update the connected relationshap with the neighbor for remainning voxels(connected relationship is changed after points being merged)
	*/
	bool BadVoxelConnectedUpdate();

	void ClassifyVoxelPoints(const PointArray& pt_cloud_xyz, const std::vector<unsigned int>& points, std::vector<std::vector<unsigned int>>& out_points, float voxel_ratio);

	bool IdentifyBadVoxelConnectedWithPlanePoints(const unsigned int bad_voxel_idx, const unsigned int plane_idx, const unsigned int voxel_idx, bool &is_connected);
	/**
	* \brief update points group of all the unmerged remaining points of a bad voxel  connected relationshap with points of reference planes
	* @param [in]  bad_voxel_idx,  bad voxel index
	* @param [in]  plane_idx,  the plane index of plane_merge_out array
	* @param [in]  plane_points,  the point indexes of plane(plane_idx)
	* @param [out]  is_connected,  if treu show the bad voxel is connected with the plane
	* @return if success or not
	*/
	bool IdentifyBadVoxelConnectedWithPlanePoints(const unsigned int bad_voxel_idx, const unsigned int plane_idx, const std::vector<unsigned int> plane_points, bool &is_connected);

	/**
	* \brief update points group of all the unmerged remaining points of a bad voxel  connected relationshap with reference planes
	* @param [in]  bad_voxel_idx,  bad voxel index
	* @param [in]  plane_idx,  the plane index of plane_merge_out array
	* @param [out]  is_connected,  if treu show the bad voxel is connected with the plane
	* @return if success or not
	*/
	bool BadVoxelConnectedWithRef(const unsigned int bad_voxel_idx, const unsigned int plane_idx, bool &is_connected);

	/**
	* \brief update  all the bad voxel remaining points connected relationshap with reference planes ( connected relationship is changed after points being merged)
	* @param [in]  is_first,   if true show this is the first time to update for plane segmentation
	* @return if success or not
	*/
	bool BadVoxelConnectedWithRef(bool is_first);

	/**
	* \brief get points group of all the merged points(maybe lying in multiple planes) of a bad voxel  connected relationshap with points in reference planes
	* @param [in]  bad_voxel_idx,  bad voxel index
	* @param [in]  plane_idx,  the plane index of plane_merge_out array
	* @param [in]  plane_points,  the point indexes of plane(plane_idx)
	* @param [out]  is_connected,  if treu show the bad voxel is connected with the plane
	* @return if success or not
	* */
	bool IdentifyBadVoxelConnectedWithMultiPlanePoints(const unsigned int bad_voxel_idx, const unsigned int plane_idx, const std::vector<unsigned int> plane_points, bool& is_connected);
	bool IdentifyPseudoBadVoxelConnectedWithMultiPlanePoints(const unsigned int voxel_idx, const unsigned int plane_idx, const std::vector<unsigned int> plane_points, bool& is_connected);

	/**
	* \brief get points group of all the merged points(maybe lying in multiple planes) of a bad voxel  connected relationshap with reference planes
	* @param [in]  bad_voxel_idx,  bad voxel index
	* @param [in]  plane_idx,  the plane index of plane_merge_out array
	* @param [out]  is_connected,  if treu show the bad voxel is connected with the plane
	* @return if success or not
	* */
	bool BadVoxelConnectedWithMultiRef(const unsigned int bad_voxel_idx, const unsigned int plane_idx, bool& is_connected);

	/**
	* \brief Init Occupeid voxel from input data, being used with no normalEstimation input
	*/
	bool OccupiedVoxelInit(PointArray input_data, VoxelItem* &voxel_array, int &Occupied_voxel_size);

	/**
	* \brief Init params for planeSegmentation with no normalEstimation input
	*/
	bool InitializeParamsNoNm(const NormalEstimationInterface inputStruct);

#ifdef SAVE_OUTPUT_FILE_DEBUG
	void OutputBadVoxelMergeOutInfo(const std::string output_path);
#endif

#ifdef SAVE_OUTPUT_FILE_DEBUG
	//plane merge out info after assigning   merging same planes
	void PlaneMergeOutInfo(const std::string output_path);
#endif

public:

	friend class PointsGroupPlane;
	friend class PlaneMerge;

	PlaneSegmentation();

	~PlaneSegmentation();

	/**
	* \brief Perform plane segmentation with default config parameters,filtering strategy is no filter
	* @param [in]  output_path,  output path of debug info
	* @param [in]  config_file,   configure file name
	* @param [in]  NormalEstimationInterface inputStruct,  get from NormalEstimation module output
	* @param [out]  PlaneSegmentationOutput* plane_seg_out, plane segmentation output
	* @return if success or not
	* */
	bool FitPlane(const std::string output_path, NormalEstimationInterface &inputStruct, PlaneSegmentationOutput* plane_segmentation_out);

	/**
	* \brief Perform plane segmentation process from point array and voxel array according config file
	* @param [in]  output_path,  output path of debug info
	* @param [in]  config_file,   configure file name
	* @param [in]  NormalEstimationInterface inputStruct,  get from NormalEstimation module output
	* @param [out]  PlaneSegmentationOutput* plane_seg_out, plane segmentation output
	* @return if success or not
	* */
	bool FitPlane(const std::string output_path, const std::string config_file, NormalEstimationInterface &inputStruct, PlaneSegmentationOutput* plane_segmentation_out);

	/**
	* \brief Perform plane segmentation process from point array and voxel array according config file, filtering strategy is that only segmenting planes by good voxels
	* @param [in]  method_type,  plane segmentation strategy, 0 : no filter 1: Only good voxel 2: no normalEstimation
	*                                                         8 : no filter with multiplane points 9:Only good voxel with multiplane points 10:2: no normalEstimation with multiplane points
	* @param [in]  output_path,  output path of debug info
	* @param [in]  NormalEstimationInterface inputStruct,  get from NormalEstimation module output
	* @param [out]  PlaneSegmentationOutput* plane_seg_out, plane segmentation output
	* @return if success or not
	* */
	bool FitPlaneMethod(const unsigned int method_type, const std::string output_path, NormalEstimationInterface& inputStruct, PlaneSegmentationOutput* plane_segmentation_out);

	bool SaveVoxelPointsDebug(const std::string output_path, unsigned int voxel_idx);

	bool SaveBadVoxelPointsClassify(const std::string output_path, unsigned int voxel_idx);

	/**
	* \brief debug for save plane points by voxel name and plane_idx
	*/
	bool SavePlanePointsDebug(const std::string output_path, unsigned int plane_idx, bool is_discete_combined);

	/**
	* \brief debug for save plane points by plane_idx and voxel type
	*/
	bool SavePlanePointsByVoxelType(const std::string output_path, const unsigned int plane_idx, const PlaneVoxelType voxel_tpe);

	/**
	* \brief debug for save plane extended and edge points by  plane_idx
	*/
	bool SavePlaneEdgePointsDebug(const std::string output_path, unsigned int plane_idx);

	// missing points info and xyz output
	void MissingPointsOutput(const std::string output_path, const PlaneSegmentationOutput& plane_seg);
	
	void RefinePlanesOutput(PlaneSegmentationOutput* plane_seg);

	PlaneSegParams& getPlaneSegParams() { return plane_seg_params; }

	SegCntrlParams& getSegConfigParams() { return config_params; }

	SysCntrlParams& getSysConfigParams() { return sys_control_para; }
#ifdef SAVE_OUTPUT_FILE_DEBUG
	DebugConfigParams & getDebugConfigParams() { return debug_config_params; };
#endif
	bool GetVoxelNeighbourinfo(unsigned int voxel_idx);
	/**
	* \brief set config parameters before plane segmentation
	* @param [in] SysCntrlParams sys_control_para system control parameters
	* @return if success or failed
	*/
	bool SetConfigure(const SysCntrlParams sys_cntrl_param);
	/**
	* \brief set config parameters before plane segmentation
	* @param [in] SegCntrlParams &config_para plane segmentation control parameters
	* @return if success or failed
	*/
	bool SetConfigure(const SegCntrlParams config_para);
	/**
	* \brief set config parameters before plane segmentation
	* @param [in] PlaneFitThresholds plane_seg_thrshld plane segmentation threshold value
	* @return if success or failed
	*/
	bool SetConfigure(const PlaneFitThresholds plane_seg_thrshld);
	/**
	* \brief set config parameters before plane segmentation
	* @param [in] VoxelParams &voxel_para plane segmentation voxel size
	* @return if success or failed
	*/
	bool SetConfigure(const VoxelParams voxel_para);

	/**
	* \brief set  plane segmentation input points
	* @param [in] PointArray input_points plane segmentation input points cloud
	* @return if success or failed
	*/
	bool SetInputPoints(const PointArray input_points);

	/**
	* \brief set  plane segmentation input points normal
	* @param [in] PointArray input_normals plane segmentation input points cloud normals
	* @return if success or failed
	*/
	bool SetInputPointsNormal(const PointArray input_normals);

	/**
	* \brief set  plane segmentation input,grid index to occupied index map
	* @param [in] const std::vector<unsigned int> grid_to_occupied_voxel_idx
	* @return if success or failed
	*/
	bool SetGridToOccupiedIdx(const std::vector<unsigned int> grid_to_occupied_voxel_idx);

	/**
	* \brief identifying and add all the points lying on the edge of multiple planes to all the planes that meet predefined threshold
	* @return if success or failed
	*/
	bool MergeMultiplaneEdgePoints();

#ifdef SAVE_OUTPUT_FILE_DEBUG

	/**
	* \brief set config parameters before plane segmentation
	* @param [in] DebugConfigParams debug_config_par of plane segmentation
	* @return if success or failed
	*/
	bool SetConfigure(const DebugConfigParams debug_config_para);
#endif

	void vxl_info(unsigned int voxel_idx, const char* file, int line)
	{
		if (voxel_idx > plane_voxel_array.size)
		{
			log_error("input voxel_idx =%d exceed size of array =%d", voxel_idx, plane_voxel_array.size);
			return;
		}
		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[voxel_idx];
		int neighbor_flag_count = 0;
		int badneighbor_flag_count = 0;
		for (int k = 0; k < 26; k++)
		{
			NeighborItem* neighbour_item = &plane_voxel->neighbors[k];
			if (!neighbour_item->is_occupied) continue;

			if (!neighbour_item->is_connected) continue;

			if (neighbour_item->neighbor_flag)
			{
				neighbor_flag_count++;
			}
			else
			{
				badneighbor_flag_count++;
			}
		}
		log_log(LOG_DEBUG, file, line, "voxel %d good neighbours: %d bad neighbours: %d", voxel_idx, neighbor_flag_count, badneighbor_flag_count);
		log_log(LOG_DEBUG, file, line, "grid idx %d voxel type %d is_overall: %d", plane_voxel->grid_idx, plane_voxel->voxel_type, plane_voxel->is_overall_merged);
		log_log(LOG_DEBUG, file, line, "plane_mse %f is_flat %d is_good_normal: %d", plane_voxel->plane_mse, plane_voxel->is_flat, plane_voxel->is_good_normal);
		log_log(LOG_DEBUG, file, line, "is_being_merged %d num_of_point %d high_mse_ratio: %f", plane_voxel->is_being_merged, plane_voxel->points.size, plane_voxel->plane_high_mse_ratio);
	}

#define VXL_DEBUG(vxl_ids,file,line) vxl_info(vxl_ids,file,line)
	//VXL_DEBUG(18816,__FILE__,__LINE__);

	/**
	* \brief  save good voxel and its good neighbors
	* @param [in] output_path, output path
	* @param [in] voxel_idx, occupied voxel index
	* @return if success or failed
	*/

	inline bool SaveGoodVoxelPointsDebug(const std::string output_path, unsigned int voxel_idx)
	{
		std::string file_type;
		std::string voxel_file_name = "voxel_xyz";
		std::stringstream voxel_id;
		voxel_id << voxel_idx;
		std::string voxel_ouput_path = output_path + "good_voxel_xyz" + voxel_id.str() + "\\";
		if (IOData::createDirectory(voxel_ouput_path))
		{
			log_error("createDirectory %s failed", voxel_ouput_path.c_str());
			return false;
		}
		file_type = ".txt";
		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[voxel_idx];
		std::string voxel_file = voxel_ouput_path + voxel_file_name + voxel_id.str() + file_type;
		FeaturesIO::SavePoint3fData(voxel_file, pt_cloud_xyz, plane_voxel->points);
		for (int i = 0; i < 26; i++)
		{
			NeighborItem* neighbor_it = &plane_voxel->neighbors[i];
			if (!neighbor_it->neighbor_flag) continue;
			PlaneVoxelItem* neighbor_voxel = &plane_voxel_array.voxels[neighbor_it->voxel_idx];
			std::stringstream neighbor_voxel_id;
			neighbor_voxel_id << neighbor_it->voxel_idx;
			std::string neighbor_voxel_file = voxel_ouput_path + voxel_file_name + neighbor_voxel_id.str() + file_type;
			FeaturesIO::SavePoint3fData(neighbor_voxel_file, pt_cloud_xyz, neighbor_voxel->points);
		}
		return true;
	}

	/**
	* \brief  debug for point closest plane
	* @param [in] pt_idx, point index of input points cloud
	*/
	inline void getPointClosestPlane(unsigned int point_idx, const char* file, int line)
	{
		bool is_being_merging_in_bad_voxel = false;
		unsigned int closest_plane_idx = (std::numeric_limits<unsigned int>::max)();
		for (unsigned int j = 0; j < point_merged_voxel_size; j++)
		{
			BadVoxelMergeOutpuItem* bad_voxel_it = &bad_voxel_merge_item[j];
			PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[bad_voxel_it->bad_voxel_idx];
			for (unsigned int k = 0; k < bad_voxel_it->voxel_point_size; k++)
			{
				if ((plane_voxel->points.point_idx[k] == point_idx) && (bad_voxel_it->closest_plane_idx[k] != (std::numeric_limits<unsigned int>::max)()))
				{
					is_being_merging_in_bad_voxel = true;
					closest_plane_idx = bad_voxel_it->closest_plane_idx[k];
				}
			}
		}
		if (is_being_merging_in_bad_voxel)
		{
			log_log(LOG_DEBUG, file, line, "point idx %d closest plane = %d", point_idx, closest_plane_idx);
		}
		else
		{
			log_log(LOG_DEBUG, file, line, "point idx %d closest plane = %d", point_idx, closest_plane_idx);
		}
	}
#define PT_PLANE(pt_idx) getPointClosestPlane(pt_idx,__FILE__,__LINE__)

	/**
	* \brief  debug for showing voxel neighbor index
	* @param [in] voxel_idx, voxel index of occupied voxels
	*/
	inline void getVoxelNeighborInfo(unsigned int voxel_idx, const char* file, int line)
	{
		PlaneVoxelItem* plane_voxel = &plane_voxel_array.voxels[voxel_idx];
		for (unsigned int i = 0; i < 26; i++)
		{
			NeighborItem* neighbor_it = &plane_voxel->neighbors[i];
			if (neighbor_it->voxel_idx == -1) continue;
			log_log(LOG_DEBUG, file, line, "voxel %d neighbor[%d]= %d ", voxel_idx, i, neighbor_it->voxel_idx);
		}
	}

#define VXL_NGHBR(voxel_idx) getVoxelNeighborInfo(voxel_idx,__FILE__,__LINE__)
};

#endif	// PLANE_SEGMENTATION_H_
