/* Copyright (c) ASTRI, 2021. All rights reserved.
 * This software is proprietary to and embodies the confidential technology
 * of Hong Kong Applied Science and Technology Research Institute Company
 * Limited (ASTRI).
 *
 * Possession, use, or copying of this software and media is authorized
 * only pursuant to a valid written license from ASTRI or an authorized
 * sublicensor.
 *
 * Author: mounty
 * Date: 20210219
 * Description: common data struct definitions
 */

#ifndef _COMMON_STRUCT_H_
#define _COMMON_STRUCT_H_

#include <unordered_map>
#include "ModuleStruct.hpp"

 /**
 * \brief  Predefined voxel size,unit depends on application
 */
struct VoxelParams
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

enum ScannerType
{
	INVALID_SCAN_TYPE = 0,
	LEICA_TYPE_0 = 1,
	UNRE_TYPE_0 = 2,
	INDUSTRY_TYPE_0 = 3,
	INDUSTRY_TYPE_1 = 4,
	SCANNER_TYPE_MAX
};

/**
* \brief syetem control parameters
*/
struct SysCntrlParams
{
	/**
	* \brief  if true  point cloud is CAD else is scanned scene data
	*/
	bool is_cad;
	/**
	* \brief scanner type
	*/
	ScannerType scanner_type;

	/**
	* \brief  0 plain; 1 UNRE_ENCRYPTION
	*/
	int encryption_type;
};

/**
* \brief  data struct for system debug parameters
*/
struct SysDebugParams
{
	int reserved_int0;
	int reserved_int1;
	float reserved_float0;
	float reserved_float1;
};

/**
* \brief Fixed point array with all input points
*/
struct PointArray
{
	/**
	* \brief  point cloud array
	*/
	ModuleStruct::Point3f* points;

	/**
	* \brief  point cloud array size
	*/
	unsigned int size;
};

/**
* \brief Fixed point index array in each voxel and its size
*/
struct PointInVoxelArray
{
	/**
	* \brief  point index to PointArray
	*/
	unsigned int* point_idx;

	/**
	* \brief  point index array size
	*/
	unsigned int size;
};

/**
* \brief data struct of point index and delta distance
*/
struct PointIdx2Dist
{
	float delta_dist;
	unsigned int cloud_point_idx;
	bool operator < (const PointIdx2Dist& p) const { return (delta_dist < p.delta_dist); }
};

/**
* \brief downsample type
*/
enum DownsampleType
{
	INVALID_DOWNSAMPLE_TYPE = 0,
	NO_DOWNSAMPLE_TYPE = 1,         // no downsample
	DOWNSAMPLE_NO_ORG = 2,         // downsample and output with points after downsample
	DOWNSAMPLE_WITH_ORG = 3,       // downsample and output with orignal points
	DOWNSAMPLE_TYPE_MAX
};

/**
* \brief data struct of plane test Filter configure parameters from section "FILTER_CNTRL_PARA" in config.ini file
*/

struct FilterCntrlParams
{
	/**
	* \brief  downsample type ,default is no downsample
	*/
	DownsampleType downsample_type;

	/**
	* \brief downsample leaf size , if leaf size is zero ,  no downsample
	*/
	float downsample_leaf_size;

	/**
	* \brief if true  have input data filtering else no input data filtering
	*/
	bool is_input_filtered;

	/**
	* \brief if true have unit check else no unit check
	*/
	bool is_input_unit_check;

	/**
	* \briefif true , select the center as origin in 3D space otherwise not change
	*/
	bool origin_in_cloud_center;
};

/**
* \brief data struct of normalEstimation contrl parameters
*/
struct NormalEstimationCntrlParam
{
	/**
	* \brief if true  voxel size is auto detected; else fixed value is set
	*/
	bool is_voxel_size_detected;

	/**
	* \brief if false, point density is auto detected else fixed value is set
	*/
	bool is_point_density_set;

	/**
	* \brief point density for normalEstimation, invalid while is_point_density_set is false
	*/
	float point_density;
};

/**
* \brief Predefined threshold from normal estimation configure file
*/
struct NormalEstimationThresholds
{
	/**
	* \brief  point number threshold of a voxel points which have passibility to be a good normal voxel;
	*/
	unsigned int min_point_num_of_valid_normal_voxel;

	/**
	* \brief  0~90 normal difference threshold of two voxels who are the good neighbours
	*/
	float max_normal_angle_of_2voxel;

	/**
	* \brief  0.0~1.0,  mean squred error threshold  ratio to voxel size, if mse ratio > value, voxel is  not a flat voxel
	*/
	float max_mse_ratio_of_voxel;
};

#endif	// _COMMON_STRUCT_H_
