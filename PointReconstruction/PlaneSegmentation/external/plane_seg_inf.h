#ifndef _PLANE_SEG_INF_H_
#define _PLANE_SEG_INF_H_
#include "ModuleStruct.hpp"

enum planeSegParamType
{
	SEG_DEFAULT_CONFIG_PARAM = 0,
	SEG_CONFIG_PARAM_VALID = 1,
	SEG_CONFIG_DEBUG_PARAM_VALID = 2,
	SEG_CONFIG_PARAM_MAX
};
enum planeSegFilterType
{
	NO_FILTER = 0,
	ONLY_GOOD_VOXEL = 1,
	NO_NORMALEST = 2,
	NO_FILTER_MP_PTS = 8,         // with multiplane points
	ONLY_GOOD_VOXEL_MP_PTS = 9,   // with multiplane points
	NO_NORMALEST_MP_PTS = 10,     // with multiplane points
	FILER_TYPE_MAX
};

/**
* \brief data struct of plane segmentation configure parameters from config.ini file
*/
struct SegCntrlParams
{
	/**
	* \brief true identify with  bridge voxel  else with bridge voxel
	*/
	bool identify_with_bridge;

	/**
	* \brief segmenting planes  filtering strategy, 0: no filter 1: Only good voxel
	*/
	planeSegFilterType filtering_strtgy_type;

	/**
	* \brief  if true configure of seciton SEG_THRESHOLD_CONFIG is valid  else  configure is invalid
	*/
	unsigned int seg_thrshld_section_valid;
};

// Predefined threshold for plane segmentation
struct PlaneFitThresholds
{
	// plane distance threshold of two voxels who are the good neighbours
	float THRESHOLD_MIN_PLANE_DIST_OF_TWO_VOXEL;

	// point to plane distance threshold of a point who can be merging to a plane
	float THRESHOLD_MIN_DIST_OF_POINT2VOXEL;

	// normal difference threshold of two voxels who are the good neighbours
	float THRESHOLD_MAX_NORMAL_ANGLE_OF_TWO_VOXEL;

	// mean squred error threshold of a voxel who is good voxel
	float THRESHOLD_MAX_MSE_OF_VOXEL;

	// point number threshold of a plane
	unsigned int THRESHOLD_MIN_POINT_NUM_OF_PLANE;

	// voxel number threshold of a plane
	int THRESHOLD_MIN_VOXEL_NUM_OF_PLANE;

	// mean squred error threshold of a plane
	float THRESHOLD_MAX_MSE_OF_PLANE;

	// point number threshold of a voxel which is valid to estimate voxel normal ,  the voxel length size  must meet this condition: number voxels <5% ,  whose point number < threshold  voxels
	unsigned int THRESHOLD_MIN_POINT_NUM_OF_VALID_NORMAL_VOXEL;

	// plane to plane distance threshold of two planes who can be merging to a plane
	// for the large planes  whose plane distance always more accurate  , so  same planes  must fit smaller distance than two voxels
	float THRESHOLD_MIN_DIST_OF_2PLANE;

	// normal difference angle threshold of two planes who can be merging to a plane
	float THRESHOLD_MAX_NORMAL_ANGEL_OF_2PLANE;

	// normal difference angle threshold of edge points who can be merging to a plane
	float THRESHOLD_MAX_NORMAL_ANGEL_OF_EDGE_POINT;

	// the ratio threhold of the voxel points  whose distance to the voxel plane exceed the mse threshold in all the voxel points
	float THRESHOLD_MAX_HIGH_MSE_RATIO;

	// the min number of candidate points which can compute right normal for a point
	//unsigned int THRESHOLD_COMPUTE_NORMALS_MIN_POINTS;

	// the min number of good points group in a points plane
	unsigned int THRESHOLD_MIN_NUM_OF_GOOD_GROUP_IN_PLANE;

	// the min point number of a voxel who is flat (good or pseduo bad voxel)
	unsigned int THRESHOLD_MIN_POINT_NUM_OF_FLAT_VOXEL;

	// the max distance of 2 close voxels who are confirmed to be connected
	float THRESHOLD_DIST_OF_2VOXEL_CONNECTED;

	// if less, the plane normal is not accurate
	unsigned int THRESHOLD_MIN_GOOD_PLANE_VOXEL_SIZE;
};

/**
* \brief data struct of debug configure info
*/
struct DebugConfigParams
{
	/**
	* \brief represents all the flowing value of KEY_DEBUG_INT  are invvalid, 1 represents they are valid
	*/
	bool dbg_section_valid;
	/**
	* \briefreserved test parameters
	*/
	int reserved_test;
	/**
	* \brief output missing_points.txt file and missing_points_info.txt file
	*/
	bool missing_point_output;
	/**
	* \briefInOutData log control
	*/
	bool io_debug_info;
	/**
	* \brief output voxel_create.txt file for voxel create debug info
	*/
	bool voxel_info_debug;
	/**
	* \briefoutput voxel_neighbour.txt file for voxel neighbour debug info
	*/
	bool neighbour_info_debug;
	/**
	* \brief is true, will output planes without plane mering
	*/
	bool no_plane_merge;
	/**
	* \brief plane_output.txt file for plane output debug info
	*/
	bool plane_output_debug;
	/**
	* \brief  merge_pseudobad_output.txt file for good voxel merge debug info
	*/
	bool merge_pseudobad_voxel_debug;
	/**
	* \brief output debug file for real bad voxels merge
	*/
	bool merge_bad_voxel_debug;
	/**
	* \brief debug get lost plane info in process of merging pseduo bad  plane
	*/
	bool pseudo_bad_lost_plane_output_debug;
	/**
	* \brief true  if 1 will output plane xyz after downsample ,otherwise false
	*/
	bool is_dwnsmpl_output;
	/**
	* \brief output points_group_neighbour.txt file for group merge debug info
	*/
	bool points_group_neighbor_output_debug;
	/**
	* \brief merge same plane debug info ouput
	*/
	bool merge_same_plane_output_debug;
	/**
	* \brief debug get result of normal estimation module by saved file before
	*/
	bool include_get_nm_by_file;
	/**
	* \brief debug info for output result of normalEstimation
	*/
	bool nm_file_ouput_debug;
};

/**
* \brief data struct of plane segmentation output
*/
struct PlaneSegOutput
{
	/**
	* \brief all the points merged into this plane
	*/
	ModuleStruct::Vector<ModuleStruct::Point3f> points;

	/**
	* \brief all the points index  merged into this plane
	*/
	ModuleStruct::Vector<unsigned int> point_ids;
	/**
	* \brief center of all points in the plane
	*/
	ModuleStruct::Point3f plane_center;
	/**
	* \brief normal of the plane
	*/
	ModuleStruct::Point3f plane_normal;
	/**
	* \brief mean squred errors of all the points in the plane
	*/
	ModuleStruct::value_type plane_mse;

	/**
	* \brief area of the plane
	*/
	ModuleStruct::interim_value_type plane_area;
};

#endif // _PLANE_SEG_INF_H_
