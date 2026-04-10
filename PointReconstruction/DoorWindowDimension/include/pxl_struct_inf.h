#ifndef _PXL_STRUCT__INF_H_
#define _PXL_STRUCT__INF_H_
#include "ModuleStruct.hpp"

/**
* \brief line seg elements after merging in 2D space
*/
struct Line2DSegOutItem
{
	/**
	* \brief line direction
	*/
	ModuleStruct::Point2f line_direction;

	/**
	* \brief start point of line
	*/
	ModuleStruct::Point2f line_seg_start;


	/**
	* \brief end point of line
	*/
	ModuleStruct::Point2f line_seg_end;

};



struct Line2DSegOutItemDebug
{
	/**
	* \brief all the points who is merging into this line
	*/
	std::vector<unsigned int> points;

	/**
	* \brief line mean square error
	*/
	float line_mse;

	/**
	* \brief line center
	*/
	ModuleStruct::Point2f line_center;

	/**
	* \brief line direction
	*/
	ModuleStruct::Point2f line_direction;


	/**
	* \brief start point of line
	*/
	ModuleStruct::Point2f line_seg_start;


	/**
	* \brief end point of line
	*/
	ModuleStruct::Point2f line_seg_end;

};




/**
* \brief line seg elements after merging in 3D space
*/
struct Line3DSegOutItem
{
	/**
	* \brief line direction
	*/
	ModuleStruct::Point3f line_direction;


	/**
	* \brief start point of line
	*/
	ModuleStruct::Point3f line_seg_start;


	/**
	* \brief end point of line
	*/
	ModuleStruct::Point3f line_seg_end;
};


/**
* \brief line seg elements after merging in 3D space
*/
struct Line3DSegOutItemDebug
{
	/**
	* \brief all the points index who is merging into this line
	*/
	ModuleStruct::Vector<ModuleStruct::Point3f> points;

	/**
	* \brief line center
	*/
	ModuleStruct::Point3f line_center;

	/**
	* \brief line direction
	*/
	ModuleStruct::Point3f line_direction;


	/**
	* \brief start point of line
	*/
	ModuleStruct::Point3f line_seg_start;


	/**
	* \brief end point of line
	*/
	ModuleStruct::Point3f line_seg_end;

	/**
	* \brief input plane index of lines
	*/
	unsigned int line_plane_idx;
};

/**
* \brief line segmentation configure parameters from config.ini file
*/
struct LineExtrCntrlParams
{
	//#0~1 point density is auto detected else fixed value is set;
	bool is_point_density_set;
	//point density, invalid while is_point_density_set is false
	float point_density;
	// if true configure of seciton LINE_THRESHOLD_CONFIG is valid  else  configure is invalid
	bool line_thrshld_section_valid;
};



/**
* \brief Predefined threshold for line segmentation
*/
struct LineFitThresholds
{
	/**
	* \brief line distance threshold of two pixels who are the good neighbours
	*/
	float min_line_dist_2pixel;

	/**
	* \brief point to line of pixel distance threshold of a point who can be merging to a line
	*/
	float min_line_dist_pt2pixel;

	/**
	* \brief normal difference threshold of two pixels who are the good neighbours
	*/
	float max_normal_angle_of_2pixel;

	/**
	* \brief mean squred error threshold of a pixel who is good pixel
	*/
	float max_mse_of_pixel;

	/**
	* \brief point number threshold of a line
	*/
	int min_point_num_of_line;

	/**
	* \brief pixel number threshold of a line
	*/
	int min_pixel_num_of_line;

	/**
	* \brief mean squred error threshold of a line
	*/
	float max_mse_of_line;

	/**
	* \brief line to line distance threshold of two lines who can be merging to a line
	*/
	float min_dist_of_2line; //5mm

	/**
	* \briefnormal difference angle threshold of two lines who can be merging to a line
	*/
	float max_angle_of_2line; //10mm

	/**
	* \brief the ratio threhold of the pixel points  whose distance to the pixel fitted line exceed the mse threshold in all the pixel points
	*/
	float max_high_mse_ratio;


	/**
	* \brief point number threshold of a pixel fitted a line
	*/
	unsigned int min_point_num_of_line_pixel;

	/**
	* \brief  min distance of two voxel is connected
	*/
	float min_connected_dist_2pxl;
};


/**
* \brief line extraction module debug configure info
*/
struct Line2DDebugParams
{
	/**
	* \brief occpuied pixel index to bad pixels, is used to find index in the bad pixels array from occupied index
	*/
	bool dbg_section_valid;
	/**
	* \brief reserved test parameters
	*/
	int reserved_test;
	/**
	* \brief output missing_points_2d.txt file and line2d_missing_points_info.txt file
	*/
	bool missing_point_output;
	/**
	* \brief output line_output.txt file for plane output debug info
	*/
	bool neighbour_info_debug;
	/**
	* \brief output line_output.txt file for plane output debug info
	*/
	bool line_output_debug;
	/**
	* \brief reserved test parameters
	*/
	int reserved_test1;
};


/**
* \brief  Predefined pixel size
*/
struct PixelSize
{
	/**
	* \brief pixel length in x direction
	*/
	float length_x_of_pixel;

	/**
	* \brief pixel length in y direction
	*/
	float length_y_of_pixel;

};

#endif // _PXL_STRUCT__INF_H_
