#ifndef _LineToolHandle_HPP_
#define _LineToolHandle_HPP_

#include <vector>
#include <string>
#include "plane_seg_inf.h"
#include "pxl_struct_inf.h"
#include <map>

struct LinePlaneMse
{
	unsigned int plane_idx;
	float plane_mse;
};

#ifndef M_PI
#define M_PI       3.14159265358979323846   // pi
#endif

class LineToolHandle
{
protected:
	std::string plane_seg_output_path;
	std::string output_path;
	std::string line_cfg_file;
	const float sub_line_dist = 100.f;
	float edge2d_pxl_size = 16;
	float pxl_size = 50;
	const float check_line_angle = 8;///
	const float check_line_dist = 150;///
	const float check_line_length = 300;///
	const float expand_shrink_const = 2.0;
	const unsigned int max_check_line_times = 10;
	const float window_calibrate_const = 1.5;
	const float door_calibrate_const = 0.2;
	const float close_to_plane_height_threshold = 100;
	const float close_to_plane_width_threshold = 50;
	const float check_line_method3_dist = 150;
	const float check_line_method3_in_hole_dist = 60;
	const float check_line_method3_length = 1500;

	//thresholds
	float plane_to_plane_distance_thr = 200; //unit = mm
	float normal_direction_threshold = 0.3;
	float min_normal_diff = static_cast<float> (std::cos((30) * M_PI / 180));
	std::vector<LinePlaneMse> line_plane_mse_arr;

public:
	/**
	* \measurement width and height of plane
	* @param [in] input_plane_points, input plane segmentation result
	* @param [in] input_plane_normal, input plane normal
	* @param [in] input_plane_center, input plane center
	* @param [in] measure__index, the measurement plane index
	* @param [in] measurement_type, measurement type, door or window
	* @param [in] holes_location, input 8 points of each hole
	* @param [out] result, measurement result, std::pair<float, float> is the width and height measurement result
	•* return true is ok, false is error
	* */
	bool measurement(const std::vector<std::vector<cv::Point3f>>& input_plane_points, const std::vector<cv::Point3f>& input_plane_normal,
		const std::vector<cv::Point3f>& input_plane_center, int measure_index, std::vector<int> measurement_type, std::vector<unsigned char> intensityVec,
		std::vector<std::vector<cv::Point3f>>& holes_location, std::vector<std::pair<float, float>>& result);

	void init_params(string plane_seg_output_path, string output_path, string line_cfg_file);

private:
	/**
	* \measurement width and height of plane
	* @param [in] holes_location, intput holes location
	* @param [in] plane_line_out, line extraction result
	* @param [out] group_lines, output check line result
	* */
	// check line 
	void check_line(const std::vector <std::vector<cv::Point3f>>& holes_location, std::vector<Line3DSegOutItemDebug>& plane_line_out, std::vector<std::pair<std::vector<int>, std::vector<int>>>& group_lines, std::vector<std::pair<std::vector<int>, std::vector<int>>>& group_holes, const cv::Point3f plane_normal);

	void check_line_with_break(const std::vector<cv::Point3f>& hole_location, std::vector<Line3DSegOutItemDebug>& plane_line_out, std::pair<std::vector<int>, std::vector<int>>& group_lines, std::pair<std::vector<int>, std::vector<int>>& group_holes, const cv::Point3f plane_normal);

	int check_line_single(const std::vector<cv::Point3f>& holes_location, int hole_id, std::vector<Line3DSegOutItemDebug>& plane_line_out);

	bool check_line_match(cv::Point3f p1, cv::Point3f p2, Line3DSegOutItemDebug& line, float check_line_angle, float check_line_dist, float check_line_length, bool output_info);
	bool check_line_match_method3(cv::Point3f p1, cv::Point3f p2, Line3DSegOutItemDebug& line, float check_line_angle, float check_line_dist, float check_line_length, bool output_info);

	std::vector<unsigned int> filter_line_match_result(cv::Point3f p1, cv::Point3f p2, std::vector<Line3DSegOutItemDebug>& plane_line_out, std::vector<unsigned int>& line_matches, bool is_method3);

	float method0(std::vector<int> lines, std::vector<Line3DSegOutItemDebug>& plane_line_out);

	float method1(std::vector<int> line, std::vector<Line3DSegOutItemDebug>& plane_line_out);

	float method3(int line_index_0, int line_index_1, std::vector<Line3DSegOutItemDebug>& plane_line_out);

	float GetBestReferencePlaneDistance(unsigned int lines, const std::vector<Line3DSegOutItemDebug>& plane_line_out, unsigned int reference_line,
		const std::vector<std::vector<cv::Point3f>>& input_plane_points, const std::vector<cv::Point3f>& input_plane_normal,
		const std::vector<cv::Point3f>& input_plane_center, int measure_index, int& reference_plane_index);

	void calibrate(std::vector<std::pair<float, float>>& result, std::vector<std::pair<int, int>>& group_method, std::vector<int> measurement_type, int measure_index, float plane_mse);
	float GetReferencePlaneForLine(unsigned int lines, const std::vector<Line3DSegOutItemDebug>& plane_line_out, const std::vector<cv::Point3f>& ref_pts, const std::vector<std::vector<cv::Point3f>>& input_plane_points, const std::vector<cv::Point3f>& input_plane_normal, const std::vector<cv::Point3f>& input_plane_center, int measure_index, int & reference_plane_index);
	float LineToPlaneDistance(const cv::Point3f& plane_center, const cv::Point3f& plane_normal, const Line3DSegOutItemDebug& line);

	float LineToPlaneDistance(const cv::Point3f & plane_center, const cv::Point3f & plane_normal, const std::vector<cv::Point3f>& pts);

	bool IsTwoPlanesConnected(const std::vector<cv::Point3f>& plane1_vertices, const std::vector<cv::Point3f>& plane2_vertices);

	bool get_lines_plane_mse(std::string output_path, std::vector<LinePlaneMse>& line_plane_mse_arr);
};


#endif /*_LineToolHandle_HPP_*/
