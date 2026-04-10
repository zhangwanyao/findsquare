#ifndef _LINE_EXTRACT2D_H_
#define _LINE_EXTRACT2D_H_
#include "line_extract_macro.h"
//#include "DataStruct.h"

#include "pxl_struct.h"
#include "pxl_struct_inf.h"
#include "log.h"

/**
* \brief class for line extrcation in 2D space
*
* \author: mounty
*/

class DW_line_extract2D {

public:

	void debug_point_idx();

	/**
	* \brief constructor with input data
	* @param pt_input_data point-set of
	*/
	DW_line_extract2D();

	/**
	* \brief destructor
	*/
	~DW_line_extract2D();

	/**
	* \brief line extraction main flow
	*
	*  @return if success or fail
	*/
	bool FitLine(std::vector<ModuleStruct::Point2f> input_data, Line2DSegOut *line_out);


	/**
	* \brief    missing points info and xy output
	*/
	void MissingPointsOutput(Line2DSegOut *line_seg_out);

	/**
	* \brief set config parameters before line extraction
	* @SysCntrlParams &sys_control_para system control parameters
	* @LineExtrCntrlParams &config_params line extraction control parameters
	* @LineFitThresholds &line_seg_thrshld line extraction threshold value
	* @PixelSize &pixel_size line extraction pixel size
	* @return if success or failed
	*/
	bool SetConfigure(const SysCntrlParams sys_control_para, const LineExtrCntrlParams config_params, \
		const LineFitThresholds line_seg_thrshld, const PixelSize pixel_size);
	
	/**
	* \brief set config parameters before line extraction
	* @SysCntrlParams &sys_control_para system control parameters
	* @LineExtrCntrlParams &config_params line extraction control parameters
	* @LineFitThresholds &line_seg_thrshld line extraction threshold value
	* @PixelSize &pixel_size line extraction pixel size
	* @Line2DDebugParams &line2d_debug_para pointer of line extraction threshold value
	* @return if success or failed
	*/
#ifdef SAVE_OUTPUT_FILE_DEBUG
	bool SetConfigure(const SysCntrlParams sys_control_para, const LineExtrCntrlParams config_params, \
		const LineFitThresholds line_seg_thrshld, const PixelSize pixel_size, const Line2DDebugParams getLineDebugConfig);
#endif

	/**
	* \brief set output path (for debug information)
	*/
	inline void setOutputPath(const std::string path)
	{
		output_path = path;
	}

	/**
	* \brief get output path (for debug information)
	*/
	inline std::string getOutputPath()
	{
		return output_path;
	}



private:

	/**
	* \brief variable initialization
	* @return if class member are initialized or not
	*/
	bool init();

	/**
	* \brief FreeMemroy
	*
	* release resources
	*/
	void FreeMemroy();


	/**
	* \brief get maxmin value of point index array in input data 
	* @param input_data : input data
	* @param max_p output max point
	* @param min_p output min point
	* @return if success or failed
	*/
	bool GetMinMaxXY(const std::vector<ModuleStruct::Point2f> input_data, ModuleStruct::Point2f &max_p, ModuleStruct::Point2f &min_p);

	/**
	* \brief get point with max min of X coordinates of input data
	* @param input_data : all the input data
	* @param point_array: point index array   in input data
	* @param max_p output point with max x coordinates
	* @param min_p output point with min x coordinates
	* @return if success or failed
	*/
	bool GetMinMaxXPoint(const std::vector<ModuleStruct::Point2f> input_data, const PointInPixelArray point_array, ModuleStruct::Point2f &max_p, ModuleStruct::Point2f &min_p);

	/**
	* \brief get point with max min of Y coordinates of input data
	* @param input_data : all the input data
	* @param point_array: point index array   in input data
	* @param max_p output point with max Y coordinates
	* @param min_p output point with min Y coordinates
	* @return if success or failed
	*/
	bool GetMinMaxYPoint(const std::vector<ModuleStruct::Point2f> input_data, const PointInPixelArray point_array, ModuleStruct::Point2f &max_p, ModuleStruct::Point2f &min_p);
	bool GetMinMaxYPoint(const std::vector<ModuleStruct::Point2f> input_data, const ModuleStruct::Vector<unsigned int> point_array, ModuleStruct::Point2f &max_p, ModuleStruct::Point2f &min_p);
	/**
	* \brief get pixels number along x , y axis , include all th empty and occupied pixels
	*/
	bool GetTotalNumOfPixelGrid(const ModuleStruct::Point2f max_p, const ModuleStruct::Point2f min_p, const PixelSize pixel_param, PixelDimension& pixel_dim);

	/**
	* \brief compute the line_direction and line mse by eigen 
	*/
	bool Compute2d(unsigned int point_cnt, SumforCovariance2d sums, ModuleStruct::Point2f &line_direction, ModuleStruct::Point2f &center, float &eigen_mse);
	bool Compute2dS(unsigned int point_cnt, SumforCovariance2d sums, ModuleStruct::Point2f &line_direction, ModuleStruct::Point2f &center, float &eigen_mse);

	/**
	* \brief get maxmin value of input data
	*/
	bool ConvertPointToPixelID(const ModuleStruct::Point2f min_p, const ModuleStruct::Point2f point, const PixelSize pixel_size, const PixelDimension pixel_dim, unsigned long long&grid_id);

	/**
	* \brief create occupied grid pixels in 2d space
	*/
	bool CreateOccupiedPixels();

	/**
	* \brief get pixels line direction, mse, is_good_pixel and  then get neighbour info such as normal differnce ,line dist ,neighbour flag etc.
	*/
	bool GetOccupiedPixelsNeighbor();


	/**
	* \brief compute point to line dist
	* @param pt point out of line
	* @param line_pt  point on the line
	* @param direction of the line
	* @return distance of point to line
	*/
	float ComputePointToLineDist(const ModuleStruct::Point2f pt, const ModuleStruct::Point2f line_pt, const ModuleStruct::Point2f line_dir)
	{
		ModuleStruct::Point2f vec, vec_proj;
		//ModuleStruct::Point2f line_dir_norm = Util_Math::vec2_normalize(line_dir);
		vec = pt - line_pt;
		double d = vec.dot(line_dir);
		vec_proj = d * line_dir;
		float dist = std::sqrtf(std::pow(vec.x - vec_proj.x, 2) + std::pow(vec.y - vec_proj.y, 2));
		return dist;
	}


	/**
	* \brief compute  a point project on a line, perpendicular foot point to a line
	* reference to https://www.jianshu.com/p/c77367f8e2d8 
	*  assume line have a slope k and a point(x1,y1) , point0(x0,y0) out of the line and the project of point0 :
	*   x = ( k^2 * x1 + k * ( y0 - y1 ) + x0 ) / ( k^2 + 1); y = k(x-x1)+y1;
	* @param pt point out of line
	* @param line_pt  point on the line
	* @param direction of the line
	* @return the project of the point out of line
	*/
	bool GetFootOfPerpendicular(const ModuleStruct::Point2f pt, const ModuleStruct::Point2f line_pt, const ModuleStruct::Point2f line_direction, ModuleStruct::Point2f &footPoint)
	{
		if (std::fabs(line_direction.x) < std::numeric_limits<float>::epsilon()\
			&& std::fabs(line_direction.y) < std::numeric_limits<float>::epsilon()) return false;

		if (std::fabs(line_direction.x) < std::numeric_limits<float>::epsilon())
		{
			// if line is paralle on the y-axis
			footPoint.y = pt.y;
			footPoint.x = line_pt.x;
			return true;
		}

		if (std::fabs(line_direction.y) < std::numeric_limits<float>::epsilon())
		{
			// if line is paralle on the x-axis
			footPoint.x = pt.x;
			footPoint.y = line_pt.y;
			return true;
		}

		float k = line_direction.y / line_direction.x;
		footPoint.x = (k*k*line_pt.x + k * (pt.y - line_pt.y) + pt.x) / ((k*k) + 1);
		footPoint.y = k * (footPoint.x - line_pt.x) + line_pt.y;
		return true;
	}

	/**
	* \brief compute mean squere error of pixel
	* @param PixelGridItem *pxl_it, input parameter,pointer to a pixel item of the PixelGridItem array of a input
	* @return success or failed
	*/

	bool ComputePixelMse(PixelGridItem *pxl_it)
	{
		double sum = 0;
		unsigned int high_mse_cnt = 0;
		for (unsigned int i = 0; i < pxl_it->points.size; i++)
		{
			unsigned int point_idx = pxl_it->points.point_idx[i];
			ModuleStruct::Point2f point = pt_points[point_idx];

			float dist = ComputePointToLineDist(point, pxl_it->center, pxl_it->line_direction);
			if (dist > line_seg_thrshld_.min_line_dist_pt2pixel) 
				high_mse_cnt++;
			sum += dist;
		}
		pxl_it->pxl_high_mse_ratio = static_cast<float>(high_mse_cnt*1.0f/pxl_it->points.size);
		pxl_it->pxl_mse = static_cast<float>(sum / pxl_it->points.size);
		return true;
	}

	/**
	* \brief compute weighted elements of pixel, include the mse, line direction , line center,  
	    equal to the elements of all the points of current pixel and its neighbors
	* @param PixelGridItem *pxl_it, input parameter, pointer to  pixel item of the PixelGridItem array 
	* @return success or failed
	*/
	bool ComputePixeWeightedlElements(unsigned int pxl_idx)
	{
		PixelGridItem *pxl_it = &pxl_occupied_array.pxl_item[pxl_idx];
		unsigned int point_cnt = pxl_it->points.size;
		SumforCovariance2d sums;
		AssignSums(&sums,&pxl_it->sums);
		// compute the weighted_line_direction  and  weighted_center
		for (int i = 0; i < 8; i++)
		{
			PixelNeighborItem *pxl_neighbor = &pxl_it->neighbors[i];
			if (!pxl_neighbor->is_connected) continue;
			PixelGridItem *neighbor_pxl_it = &pxl_occupied_array.pxl_item[pxl_neighbor->pixel_idx];
			PushSums(&sums, &neighbor_pxl_it->sums);
			point_cnt += neighbor_pxl_it->points.size;
		}

		float eigen_mse;
		Compute2dS(point_cnt,sums, pxl_it->weighted_line_direction, pxl_it->weighted_center, eigen_mse);

		double sum_dist = 0;
		unsigned int high_mse_cnt = 0;
		pxl_it->high_weighted_mse_cnt = 0;

		// compute the weighted mse , high_weighted_mse_cnt  and  high_weighted_mse_ratio
		// first get sum of distance of points to line of current pixel
		for (unsigned int i = 0; i < pxl_it->points.size; i++)
		{
			unsigned int point_idx = pxl_it->points.point_idx[i];
			ModuleStruct::Point2f point = pt_points[point_idx];

			float dist = ComputePointToLineDist(point, pxl_it->weighted_center, pxl_it->weighted_line_direction);
			if (dist > line_seg_thrshld_.min_line_dist_pt2pixel) pxl_it->high_weighted_mse_cnt++;
			sum_dist += dist;
		}
		high_mse_cnt = pxl_it->high_weighted_mse_cnt;
		// get sum of distance of points to line  of the neigbhors
		for (int i = 0; i < 8; i++)
		{
			PixelNeighborItem *pxl_neighbor = &pxl_it->neighbors[i];
			if (!pxl_neighbor->is_connected) 
				continue;
			PixelGridItem *neighbor_pxl_it = &pxl_occupied_array.pxl_item[pxl_neighbor->pixel_idx];
			//pxl_neighbor->high_weighted_mse_cnt = 0;
			int neighbor_high_weighted_mse_cnt = 0;
			for (unsigned int j = 0; j < neighbor_pxl_it->points.size; j++)
			{
				unsigned int point_idx = neighbor_pxl_it->points.point_idx[j];
				ModuleStruct::Point2f point = pt_points[point_idx];
				float dist = ComputePointToLineDist(point, pxl_it->weighted_center, pxl_it->weighted_line_direction);
				if (dist > line_seg_thrshld_.min_line_dist_pt2pixel) 
					neighbor_high_weighted_mse_cnt++;
				
				sum_dist += dist;
			}
			high_mse_cnt += neighbor_high_weighted_mse_cnt;

		}

		pxl_it->high_weighted_mse_ratio = static_cast<float>(high_mse_cnt*1.0f / point_cnt);
		pxl_it->pxl_weighted_mse = static_cast<float>(sum_dist / point_cnt);

		pxl_it->in_line_pixel = false;
		if (pxl_it->pxl_weighted_mse < line_seg_thrshld_.max_mse_of_line)
		{
			pxl_it->in_line_pixel = true;			
			pxl_it->is_overall_merged = true;
		}

		return true;
	}


	
	/**
	* \brief  clear a covariance sums
	* @param pointer to a covariance sums	
	*/
	void SumsClear(SumforCovariance2d *sums)
	{
		sums->sum_x = sums->sum_y = sums->sum_xx = sums->sum_yy = sums->sum_xy = 0;
	}


	/**
	* \brief give all the value of a sums from another sums
	* @param pointer to target covariance sums
	* @param pointer to source covariance sums
	*/
	void AssignSums(SumforCovariance2d *sums_to, SumforCovariance2d *sums_from)
	{
		sums_to->sum_x = sums_from->sum_x;
		sums_to->sum_y = sums_from->sum_y;
		sums_to->sum_xx = sums_from->sum_xx;
		sums_to->sum_yy = sums_from->sum_yy;
		sums_to->sum_xy = sums_from->sum_xy;
	}


	/**
	* \brief add a point into a covariance sums
	* @param pointer to target covariance sums
	* @param point
	*/
	void PushPoint(SumforCovariance2d *sums, ModuleStruct::Point2f point)
	{
		sums->sum_x += point.x;
		sums->sum_y += point.y;
		sums->sum_xx += point.x*point.x;
		sums->sum_yy += point.y*point.y;
		sums->sum_xy += point.x*point.y;
	}


	/**
	* \brief add all the value ofcovariance sums to another covariance sums
	* @param pointer to target covariance sums
	* @param pointer to source covariance sums
	*/

	void  PushSums(SumforCovariance2d *sums_to, SumforCovariance2d *sums_from)
	{
		sums_to->sum_x += sums_from->sum_x;
		sums_to->sum_y += sums_from->sum_y;
		sums_to->sum_xx += sums_from->sum_xx;
		sums_to->sum_yy += sums_from->sum_yy;
		sums_to->sum_xy += sums_from->sum_xy;
	}


	/**
	* \brief convert pixel grid index into the cols and rows of the pixel in the 2D space
	* @param pointer to a pixel item of the PixelGridItem array of a input
	* @return mean squere error of pixel
	*/
	void ConvertPixelIDTo2dID(const ModuleStruct::Point2f& pt_xyz, int& row_idx, int& col_idx)
	{

		col_idx = (int)(std::floor((pt_xyz.x - min_p.x)* inverse_length_x_of_pixel));
		row_idx = (int)(std::floor((pt_xyz.y - min_p.y)* inverse_length_x_of_pixel));

		if (col_idx == pixel_dim.cols_of_pixel)
			col_idx--;
		if (row_idx == pixel_dim.rows_of_pixel)
			row_idx--;
		return;
	}


	/**
	* \brief  Find out the parent pixel of all good pixels, implement unique identifier parent pixel passed in all the good pixels of each line	
	* @return if success or failed for identify parent pixel
	*/
	bool IdentifyParentOfGoodPixels();


	/**
	* \brief  merge all the good pixels on the same line into line
	* @return if success or failed for identifying parent pixel
	*/
	bool MergeGoodPixels();

	/**
	* \brief  Find out the parent pixel of all bad pixels in the edge of each line 
	* @return if success or failed for identify parent pixel in bad pixels
	*/
	bool IdentifyParentOfbadPixels();


	/**
	* \brief  merge all the edge points of  line in the bad pixels into a line 
	* @return if success or failed for merging bad pixels
	*/
	bool MergeBadPixels();



	/**
	* \brief find the lines  with both  same direction  and  line distance < threshold
	* @return if success or failed to find
	*/
	bool IdentifySameLines(bool **is_same_line, unsigned int line_size);


	/**
	* \brief  find the lines  who are connected
	* @return if success or failed to find
	*/
	bool IdentifyConnectedLines(bool **is_connected, unsigned int line_size);


	/**
	* \brief  merge all the same lines into one line
	* @return if success or failed for merging lines
	*/
	bool MergeLines();


	/**
	* \brief  get mean squared errors of a line
	* @return if success or failed
	*/
	bool GetLineDistMse(LineMergeOutputItem * line, float& line_mse)
	{
		unsigned int point_cnt = 0;
		IntermediateType average_distance = 0.0;
		ModuleStruct::Point2f point;
		for (int i = 0; i < line->pixels.size; i++)
		{
			unsigned int pxl_idx = line->pixels.pxl_idx[i];
			PixelGridItem *pxl_it = &pxl_occupied_array.pxl_item[pxl_idx];
			for (unsigned int j = 0; j < pxl_it->points.size; j++)
			{
				unsigned int point_idx = pxl_it->points.point_idx[j];
				point = pt_points[point_idx];
				float line_dist = ComputePointToLineDist(point, line->line_center,line->line_direction);
				average_distance += line_dist;
			}
			point_cnt += pxl_it->points.size;
		}

		for (unsigned int i = 0; i < line->points.size; i++)
		{
			point = pt_points[line->points.point_idx[i]];
			float line_dist = ComputePointToLineDist(point, line->line_center, line->line_direction);
			average_distance += line_dist;
			point_cnt++;
		}

		line_mse = (float)(average_distance / point_cnt);
		return true;
	}

	/**
	* \brief  get mean squared errors of a line
	* @return if success or failed
	*/
	bool GetLineDistMse(LineSegItem * line, float& line_mse)
	{
		unsigned int point_cnt = 0;
		IntermediateType average_distance = 0.0;
		ModuleStruct::Point2f point;

		for (unsigned int i = 0; i < line->points.size; i++)
		{
			point = pt_points[line->points.point_idx[i]];
			float line_dist = ComputePointToLineDist(point, line->line_center, line->line_direction);
			average_distance += line_dist;
			point_cnt++;
		}

		line_mse = (float)(average_distance / point_cnt);
		return true;
	}

	/**
	* \brief compute start and end point of a line
	* @return if success or failed
	*/
	bool GetStartEndOfLines(LineSegItem *line_seg);

	/**
	* \brief  Assign all the lines segment output, include line direction, line center, line start, line end ,and line xyz in 3d coordinates axis( z is fixed to zero)
	* @return if success or failed for assign lines
	*/
	bool AssignLinesOutput(Line2DSegOut * line_out);

	/**
	* \brief  Assign the merge out lines into output line segment
	* @param orgin_line_it  orignal line merge out pointer info to be assigned
	* @param out_line_seg_it  output lineinfo  pointer
	* @return if success or failed for assign
	*/
	bool AssignLine(LineMergeOutputItem *orgin_line_it, LineSegItem *out_line_seg_it);

	/**
	* \brief  Line merge out info  after assigning  lines , output in line_output.txt
	*/
	void LineMergeOutInfo(Line2DSegOut *line_seg_out);


	/**
	* \brief   pixel neighbor info file output , file name is output_type+ pixel_neighbour.txt
	* @param output_type  determine which kind of pixel show in output file 	
	*/
	void GetpixelNeigbourDebugInfo(PxlNeighborDbgType output_type);

	/**
	* \brief Identify the two points group are close or connected by pre-specified distance
	* @param const std::vector<ModuleStruct::Point2f> input_data,input parameter, input data cloud data
	* @param const unsigned int first_pixel, input parameter, first pixel
	* @param onst unsigned int second_pixel, input parameter, second pixel
	* @param const float distance , input parameter, pre-specified distance
	* @return true if the two points group distance is lower thean pre-specified distance ,otherwise return false
	*/
	bool Identify2PixelCLoseByDistance(const std::vector<ModuleStruct::Point2f> input_data, const unsigned int first_pixel, const unsigned int second_pixel, const float distance);


	/**
	* \brief  save pixel  information for debug
	*/
	bool SavePixel(const std::string path, int pixel_idx);


	/**
	* \brief  save line pixel information for debug, include good pixels and bad pixels
	*/
	bool SaveLines(const std::string path, int line_idx);
	bool SaveLinesInfo(void);

	void AddLineToSameLineGroup(const unsigned int line_in_group, const unsigned int same_line_idx, const unsigned int line_size, bool** is_same_line, bool** is_connected);

private:
	
	/**
	* \brief movement reprensent neighbours position of pixels
	*/
	const int movement_x[8] = { -1, -1, -1, 0, 1, 1, 1, 0};
	const int movement_y[8] = { -1, 0, 1, 1, 1, 0, -1, -1};

	/**
	* \brief input data and parameters path
	*/
	std::string input_path;

	/**
	* \brief max_p is the max coordinates along x ,y axis, min_p is the min coordinates along x ,y axis
	*/
	ModuleStruct::Point2f max_p, min_p;

	/**
	* \brief PixelSize is the pixel size along x ,y axis
	*/
	PixelSize pixel_size_;///T
	
	float inverse_length_x_of_pixel;///T
	float inverse_length_y_of_pixel; ///T

	/**
	* \brief column numbers along x axis,  and  row numbers along y axis 
	*/
	PixelDimension pixel_dim;

	/**
	* \brief Line extraction config parameter
	*/
	SysCntrlParams sys_control_para_;


	/**
	* \brief Line extraction config parameter
	*/
	LineExtrCntrlParams config_params_;

	/**
	* \brief Line extraction threshold setting
	*/
	LineFitThresholds line_seg_thrshld_;


	/**
	* \brief total number of pixel grids
	*/
	unsigned int total_num_of_pxl_grid;///T
	
	/**
	* \brief input points in 2d space
	*/
	std::vector<ModuleStruct::Point2f>pt_points;

	/**
	* \brief  PixelGridItem array
	*/
	PixelGridArray pxl_occupied_array;///T CreateOccupiedPixels

	/**
	* \brief it is used to find out pixel_idx in occupied pixel array from grid_idx in 2D Space
	*/
	unsigned int *grid_to_occupied_pxl_idx; ///T CreateOccupiedPixels

	/**
	* \brief pixel index of occupied pixel array to line index
	*/
	unsigned int *pixel_to_line_idx;

	/**
	* \brief line merge out by good pixels
	*/
	LineMergeOutput line_merge_out;

	/**
	* \brief bad pixel merge output info
	*/
	BadPxlMergeArray bad_pxl_merge_array;

	/**
	* \brief occpuied pixel index to bad pixels, is used to find index in the bad pixels array from occupied index
	*/
	unsigned int *occupied_idx_to_bad_pxl;

	/**
	* \brief the same lines  group index array
	*/
	SameLineGroupArray same_line_group_array; // the same planes  group index array

#ifdef SAVE_OUTPUT_FILE_DEBUG
	Line2DDebugParams line2d_debug_params_;
#endif

	const unsigned int INVALID_LINE_IDX = std::numeric_limits<unsigned int>::max();

	std::string output_path;
};


#endif // _LINE_EXTRACT2D_H_
