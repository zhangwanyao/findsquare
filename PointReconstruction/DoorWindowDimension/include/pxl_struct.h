#ifndef _PXL_STRUCT_H_
#define _PXL_STRUCT_H_
#include "line_extract/line_extract_macro.h"
#include "DataStruct.h"
#include "ModuleStruct.hpp"

/**
* \brief  point array  in 2D space
*/
/*struct Point2fArray
{
	Point2f * points;
	unsigned int size;
};*/



/**
* \brief dimensions of pixels in 2D space
*/
struct PixelDimension
{
	unsigned int cols_of_pixel;
	unsigned int rows_of_pixel;
};

/**
* \brief  Fixed point array in each pixel and its size
*/
typedef PointInVoxelArray PointInPixelArray;
/*struct PointInPixelArray
{*/
	/**
	* \brief index of points
	*/
	//unsigned int* point_idx;

	/**
	* \brief number of points of array 
	*/
	//int size;
//};


/**
* \brief The covariance sums for pixels covariance matrix 
*/
struct SumforCovariance2d
{
	IntermediateType sum_x;
	IntermediateType sum_y;
	IntermediateType sum_xx;
	IntermediateType sum_yy;
	IntermediateType sum_xy;
};


/**
* \brief  pixel neighbour output debug type
*/
enum PxlNeighborDbgType
{
	ALL_PIXEL_DEBUG = 0,
	GOOD_PIXEL_DEBUG = 1,
	BAD_PIXEL_DEBUG = 2,
};



/**
* \brief Each neighbor pixel with its element
*/
struct PixelNeighborItem
{
	/**
	* \brief occupancy, false - Empty pixel, true - Occupied pixel
	*/
	bool is_occupied;

	/**
	* \brief index to occupied pixel array
	*/
	unsigned int pixel_idx;

	/**
	* \brief normal difference between current pixel and the neighbour
	*/
	float direcion_diff;

	/**
	* \brief line distance  between current pixel and the neighbour
	*/
	float line_dist;

	/**
	* \brief 
	neighbor indication: true - ((direcion_diff < Nt) && (pxl_dist < Dt)) 
	where Nt and Dt is pre-defined thresholds, otherwise equals to false
	*/
	bool neighbor_flag;

	/**
	* \brief 
	neighbor indication: true - ((weighted direcion diff < Nt) && (weighted pxl dist < Dt)) 
	where Nt and Dt is pre-defined thresholds, otherwise equals to false
	*/
	bool in_line_flag;

	/**
	* \brief the count of points whose distance are more than  weighted mse threshold 
	*/
	//unsigned int high_weighted_mse_cnt;

	/**
	* \brief true  indicate this neighbor is connected with current pixel
	*/
	bool is_connected;

};

struct PixelGridItem
{
	/**
	* \brief true- pixel being fitted a line, false -  pixel not being fitted a line
	*/
	bool is_good_pixel;

	/**
	* \brief true-  pixel and all its neighbors being fitted a line, otherwise false, 
	* if a pixel is in_line_pixel,its is_good_pixel must be true
	*/
	bool in_line_pixel;


	/**
	* \brief true- index to 2d space, contains both empty and occupied pixels
	*/
	unsigned int grid_idx;

	/**
	* \brief pixel mean squared error
	*/
	float pxl_mse;


	/**
	* \brief pixel eigen mean squared error
	*/
	float pxl_eigen_mse;

	/**
	* \brief pixel weighted mse is equal to the sum of mse of pixel and its neighbourss
	*/
	float pxl_weighted_mse;

	/**
	* \brief the ratio of points whose mse is more than threshold
	*/
	float pxl_high_mse_ratio;
	

	/**
	* \brief  center of all points in pixel
	*/
	ModuleStruct::Point2f center;

	/**
	* \brief  center of all points in pixel
	*/
	ModuleStruct::Point2f weighted_center;

	/**
	* \brief pixel fitted line direction in 2D space
	*/
	ModuleStruct::Point2f line_direction;

	/**
	* \brief line direction of points group constructed by pixel and its neighbours  
	*/
	ModuleStruct::Point2f weighted_line_direction;

	/**
	* \brief covariance of all the points in current pixel
	*/
	SumforCovariance2d sums;

	/**
	* \brief point index pointer and size per pixel 
	*/
	PointInPixelArray points;
		
	/**
	* \brief the flag for being merged in a line, true is being merged
	*/
	bool is_being_merged;


	/**
	* \brief the flag show this pixel can be merged overall into  a line
	*/
	bool is_overall_merged;


	/**
	* \brief  8 neigbour pixels information of current pixel
	*/
	PixelNeighborItem neighbors[8];

	/**
	* \brief parent pixel index, two parent index buffer is ping-pong buffer to escape  
	   self pixel writing  conflict with neighbor's reading
	*/
	unsigned int parent_pixel_idx[2];

	/**
	* \brief number of good neighbor pixels who have fitted line with current pixel
	*/
	unsigned int good_neighbor_cnt;

	/**
	* \brief number of bad neighbor pixels who have no fitted line with current pixel
	*/
	unsigned int bad_neighbor_cnt;

	/**
	* \brief the count of points whose distance are more than  weighted mse threshold
	*/
	unsigned int high_weighted_mse_cnt;


	/**
	* \brief the ratio of points whose mse is more than threshold
	*/
	float high_weighted_mse_ratio;


};



/**
* \brief pixel grid item array stuct
*/
struct PixelGridArray
{
	PixelGridItem *pxl_item;
	unsigned int size;
};


/**
* \brief Fixed array of pixels in a line
*/
struct PixelArray
{
	unsigned int* pxl_idx;
	int size;
};

/**
* \brief Each line merge output item
*/
struct LineMergeOutputItem
{
	/**
	* \brief pixel root id of a line, it is the final parent pixel id after line merging
	*/
	unsigned int parent_pixel_idx;
	
	/**
	* \brief number of points in this line;
	*/
	unsigned int total_point_cnt;
	
	/**
	* \brief total number of good pixel in this plane;
	*/
	unsigned int good_pixel_size;
	
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
	* \brief covariance of all the points in the line
	*/
	SumforCovariance2d sums;

	/**
	* \brief all the good pixels who is merging into this line
	*/
	PixelArray pixels;
	
	/**
	* \brief edge points in bad pixels being merged into this line
	*/
	PointInPixelArray points;
};

/**
* \brief Fixed line merge output item array
*/
struct LineMergeOutput
{
	LineMergeOutputItem* lines;
	unsigned int size;
};

/**
* \brief merge output item  for pixels whose is_overall_merged are false
*/
struct BadPixelMergeOutpuItem
{
	/**
	* \brief pixel index to occupied pixel array
	*/
	unsigned int pxl_idx;

	/**
	* \brief point size of this bad pixel 
	*/
	unsigned int pixel_point_size;

	/**
	* \brief true show this bad pixel has points to be merged to a line
	*/
	bool is_being_merged;

	/**
	* \brief index to lineMergeOutput Lines, record the closest line of this bad pixel point
	*/
	unsigned int* closest_line_idx;
	
	/**
	* \brief point merged  flag  of this bad pixel points
	*/
	bool* point_merged_flag;
		
	/**
	* \brief the number of the  points who have being merged in the line
	*/
	unsigned int num_of_points_merged;

	/**
	* \brief  show if this pixel have neighbor in a line
	*/
	bool is_neighbor_of_line;

	/**
	* \brief  show list of true or false for the line have point in this pixel
	*/
	bool *is_of_line_list;

};

/** 
* \brief  merge out item array of pixels whose is_overall_merged are false
*/
struct BadPxlMergeArray
{
	BadPixelMergeOutpuItem* bad_pxl_merge_it;
	unsigned int size;
};


/**
* \brief the same lines  group index array
*/
struct SameLineGroupArray
{
	/**
	* \brief  record the same lines  group index of all merging-out  lines , if -1 show have no same line
	*/
	unsigned int* same_line_group_idx;

	/**
	* \brief  the same line  group idex size , is equal to sum of line_merge_out.size 
	*/
	unsigned int  line_size;


	/**
	* \brief  the same line  group number, is equal to max group index
	*/
	unsigned int  group_number;

};


/**
* \brief line seg elements after merging
*/
struct LineSegItem
{
	/**
	* \brief parent pixel index
	*/
	//unsigned int parent_pixel_idx;

	/**
	* \brief all the points who is merging into this line
	*/
	PointInPixelArray points;

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


	/**
	* \brief covariance of all the points in the line
	*/
	//SumforCovariance2d sums;

};


/**
* \brief  Output of line segmentation  extraction
*/
struct Line2DSegOut
{
	/**
	* \brief line array after line segmentation extration
	*/
	LineSegItem* line_segs;

	/**
	* \brief line seg number after line segmentation extraction
	*/
	unsigned int size;
};




#endif // _PXL_STRUCT_H_
