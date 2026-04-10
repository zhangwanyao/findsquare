#include "config.h"
#include "../../Common/MathOperation.h"
//#include "common/in_out_data.hpp"
#include "in_out_data.hpp"
#include "util_UNRE.hpp"
#include "util_line_tool.hpp"
#include "edge_dection/util_edge2d_test.hpp"
#include <assert.h>  
#include "line_extract/util_line_in_plane_test.hpp"
#include "../../Measurement/MeasureBase.h"
//#include "../../CutScanPlane/utility/util_math.h"
#include "util_math.hpp"
#include <atomic>
#include "util_UNRE.hpp"
#include "../../CutScanPlane/utility/util_pca.h"
#include "../PointReconstruction.h"

//#define OUTPUT_DEBUG_INFO
//#define OUTPUT_DEBUG_INFO_SV
//#define TESTPA
#define REMOVE_NOISE

#ifdef  OUTPUT_DEBUG_INFO
extern std::ofstream resultA;
#endif

void SaveSceneData(std::string path, int scene_idx, const std::vector<cv::Point3f>& filter_scene_plane_xyz)
{
	std::stringstream file_num;
	file_num << path.c_str() << scene_idx << "_xyz.txt";
	std::string file_name = file_num.str();
	IOData::SavePoint3fData(file_name, filter_scene_plane_xyz);
}

float GetPlaneMse(Point3f plane_center, Point3f plane_normal, Point3fArray points)
{
	IntermediateType average_distance = 0.0;
	float dist_mse = 0.f;
	for (unsigned int i = 0; i < points.size(); i++)
	{
		average_distance += Util_Math::DWComputePointToPlaneDist<Point3f>(points[i], plane_normal, plane_center);
	}
	dist_mse = (float)(average_distance / points.size());
	//std::cout << "dist_mse: " << dist_mse << std::endl;
	return dist_mse;
}

void DWgetLinePara(cv::Point2f p1, cv::Point2f p2, cv::Point2f &Pa)
{
	if (abs(p1.x - p2.x) < 0.01)
		//if (p1.x == p2.x)
	{
		Pa.x = 0;
		Pa.y = 0;
	}
	else {
		Pa.x = (p2.y - p1.y) / (p2.x - p1.x);
		Pa.y = p1.y - Pa.x * p1.x;
	}
}

void get2LineCross(cv::Point2f p0, cv::Point2f p1, cv::Point2f &Pa)
{
	Pa.x = (p1.y - p0.y) / (p0.x - p1.x);
	Pa.y = p0.x * Pa.x + p0.y;
}

bool getCorssedPoint(cv::Point2f pt0, cv::Point2f pt1, cv::Point2f pt2, cv::Point2f pt3, cv::Point2f& cp)
{
	bool isCorssed = false;
	cv::Point2f pa, pb;
	DWgetLinePara(pt0, pt1, pa);
	DWgetLinePara(pt2, pt3, pb);
	if ((pa.x == 0) && (pa.y == 0) && (pb.x == 0) && (pb.y == 0)) //a//b//y
	{
		isCorssed = false;
	}
	else if ((pa.x == 0) && (pa.y == 0)) // a//y, b!//y
	{
		cp.x = pt1.x;
		cp.y = pb.x * cp.x + pb.y;
		isCorssed = true;
	}
	else if ((pb.x == 0) && (pb.y == 0)) // b//y, a !//y
	{
		cp.x = pt2.x;
		cp.y = pa.x * cp.x + pa.y;
		isCorssed = true;
	}
	else { //none //y
		if (pa.x == pb.x) {  //a//b
			isCorssed = false;
		}
		else {
			cp.x = (pa.y - pb.y) / (pb.x - pa.x);
			cp.y = pa.x *cp.x + pa.y;
			isCorssed = true;
		}
	}
	return isCorssed;
}

bool LessSortX(cv::Point3f a, cv::Point3f b) { return (a.x > b.x); };
bool LessSortY(cv::Point3f a, cv::Point3f b) { return (a.y > b.y); };


bool intersectionLinePlane(cv::Point3f& P0, cv::Point3f& P1, cv::Point3f& P2, float coeffi[4])
{
	cv::Point3f P1P2 = { P2.x - P1.x, P2.y - P1.y, P2.z - P1.z };
	float num, den, n;
	num = coeffi[0] * P1.x + coeffi[1] * P1.y + coeffi[2] * P1.z + coeffi[3];
	den = coeffi[0] * P1P2.x + coeffi[1] * P1P2.y + coeffi[2] * P1P2.z;
	if (fabs(den)<1e-5)
	{
		return false;
	}
	n = abs(num / den);

	P0.x = P1.x + n * P1P2.x;
	P0.y = P1.y + n * P1P2.y;
	P0.z = P1.z + n * P1P2.z;

	return true;
}

bool LineToolHandle::measurement(
	const std::vector<std::vector<cv::Point3f>>& input_plane_points,
	const std::vector<cv::Point3f>& input_plane_normal, 
	const std::vector<cv::Point3f>& input_plane_center,
	int measure_index, 
	std::vector<int> measurement_type,
	std::vector<unsigned char> intensityVec,
	std::vector<std::vector<cv::Point3f>>& holes_location, 
	std::vector<std::pair<float, float>>& result)

{
	bool return_value = true;
	result.resize(holes_location.size());
	std::vector<std::vector<cv::Point2f>> plane_2Dcontours_array;
	//get_lines_plane_mse(plane_seg_output_path, line_plane_mse_arr);

	//===Remove Noise
#ifdef REMOVE_NOISE
	clock_t t1 = clock();
	PlaneSegOutput plane_out_no_noisy;
	std::vector<unsigned char>is_noisy;
	double pxl_size = 0.2 * 80;
	util_UNRE::PlaneSegOutputWithInten with_inten_planes;
	util_UNRE::set_sigma_ratio(2.0f);
#ifdef  OUTPUT_DEBUG_INFO_SV	
	SaveSceneData(output_path, measure_index*1000+66, input_plane_points[measure_index]);
#endif
	util_line_in_plane_test::get_edge2d_lines_with_noisy(   output_path,
															pxl_size, ///TBC
															line_cfg_file, sub_line_dist, 
															input_plane_points[measure_index], 
															input_plane_center[measure_index], 
															input_plane_normal[measure_index], 
															intensityVec, 
															is_noisy, ///o 
															with_inten_planes, ///o PlaneSegOutputWithInten
															holes_location
														);
	
	util_UNRE::single_plane_remove_noisy(	output_path, is_noisy, 
											input_plane_points[measure_index], 
											input_plane_center[measure_index], 
											input_plane_normal[measure_index], 
											plane_out_no_noisy ///o 3D PlaneSegOutput
										);
#ifdef  OUTPUT_DEBUG_INFO	
	std::cout << "remove noisy time cost: " << (clock() - t1) / float(CLOCKS_PER_SEC) << std::endl;
	util_UNRE::save_single_plane_with_noisy(output_path, with_inten_planes, is_noisy, measure_index);
#endif

	//===Edge Dectetion
	clock_t t2 = clock();
	util_line_in_plane_test::get_plane_edge2d(///detector->get2DContours
										//	  output_path,
											  edge2d_pxl_size, //TBC
		                                      plane_out_no_noisy.points, ///i 3DPlane
											  input_plane_normal[measure_index], 
											  input_plane_center[measure_index], 
											  plane_2Dcontours_array///o 2Dcontours
											 );
#ifdef  OUTPUT_DEBUG_INFO	
	std::cout << "edge dectection time cost: " << (clock() - t2) / float(CLOCKS_PER_SEC) << std::endl;
#endif
#endif // REMOVE_NOISE

#ifndef REMOVE_NOISE
	util_line_in_plane_test::get_plane_edge2d(edge2d_pxl_size, input_plane_points[measure_index], input_plane_normal[measure_index], input_plane_center[measure_index], plane_2Dcontours_array);
#endif
	//save edge dection result for debug
#ifdef  OUTPUT_DEBUG_INFO_SV
	std::string folder;
	folder = output_path + "edge2d_xy_" + to_string(measure_index) + "\\";
	std::vector < std::vector<std::vector<cv::Point2f>>> plane_2Dcontours_array_temp;
	plane_2Dcontours_array_temp.push_back(plane_2Dcontours_array);
	util_edge2d_test::save_edge2d_array(folder, "plane_edge2d_", ";", ".txt", plane_2Dcontours_array_temp);
#endif

#if 0
	clock_t t2 = clock();
	// edge 2d to edge 3d
	std::vector<Point3fArray> plane_3Dcontours_array;
	util_line_in_plane_test::get_plane_edge3d(edge2d_pxl_size, input_plane_points[measure_index], input_plane_normal[measure_index], 
											 input_plane_center[measure_index], plane_3Dcontours_array);
	std::cout << "edge dectection time cost: " << (clock() - t2) / float(CLOCKS_PER_SEC) << std::endl;
	std::vector <std::vector<Point3fArray>> plane_3Dcontours_array_temp;
	plane_3Dcontours_array_temp.push_back(plane_3Dcontours_array);
	folder = output_path + "edge3d_xy_" + to_string(measure_index) + "\\";
	util_edge2d_test::save_edge3d_array(folder, "plane_edge3d_", ";", ".txt", plane_3Dcontours_array_temp);
#endif
	
//===Line Extraction;
	clock_t t3 = clock();
	std::vector<Line3DSegOutItemDebug> plane_line_out;
	//=================4=================================
	util_line_in_plane_test::get_edge2d_lines_single(
														line_cfg_file, 
														input_plane_normal[measure_index], 
														input_plane_center[measure_index], 
														plane_2Dcontours_array,///i
														plane_line_out ///o  Line3DSegOutItemDebug
													);
#ifdef  OUTPUT_DEBUG_INFO_SV
	std::cout << "line extraction time cost: " << (clock() - t3) / float(CLOCKS_PER_SEC) << std::endl;

	//save line extraction result for debug
	folder = output_path + "line_xyz3d_" + to_string(measure_index) + "\\";
	util_line_test::save_line3d_xyz(folder, "line_xyz3d", ";", ".txt", plane_line_out);

	folder = output_path;
	util_line_test::save_line3d_direction_end(0, folder, "line_start_end3d", plane_line_out);

	folder = output_path + "line_end3d_" + to_string(measure_index) + "\\";
	util_line_test::save_line3d_direction_end(1, folder, "line_point3d", plane_line_out);

	//folder = output_path + "line_direction3d_" + to_string(measure_index) + "\\";
	//util_line_test::save_line3d_direction_end(2, folder, "line_direction3d", plane_line_out);
#endif
#if 1
	//===check lines
	// first is width, second is height
	std::vector<std::pair<std::vector<int>, std::vector<int>>> group_lines;
	std::vector<std::pair<std::vector<int>, std::vector<int>>> group_holes;
	std::vector<std::pair<int, int>> group_method(holes_location.size());
	clock_t t4 = clock();
	//==================5================================
#ifdef  OUTPUT_DEBUG_INFO
	cout << "=== plane_line_out.size: " << plane_line_out.size() << " holes_location.size():" << holes_location.size() << endl;
#endif
	this->check_line(holes_location, 
		             plane_line_out, ///i Line3DSegOutItemDebug
		             group_lines,
					 group_holes,
		             input_plane_normal[measure_index]);
#ifdef  OUTPUT_DEBUG_INFO
	cout << "===group_lines.size: " << group_lines.size() 	 << " group_holes.size: " << group_holes.size() << endl;
	std::cout << "check_line time cost: " << (clock() - t4) / float(CLOCKS_PER_SEC) << std::endl;
#endif
	//==========do measurement
	clock_t t5 = clock();
	//cout <<"======= "<< __FUNCTION__ <<"  "<< __LINE__ << endl;
	for (unsigned int i = 0; i < group_lines.size(); i++)
	{
		float width_value = 0.0, height_value = 0.0;
		int first_method = 0, second_method = 0;
		int reference_w_index = -1;
		int reference_h_index = -1;
#ifdef  OUTPUT_DEBUG_INFO
		cout << "=  group_lines.first[0] : " << group_lines[i].first[0] << " group_lines.first[1]: " << group_lines[i].first[1] << endl;
		cout << "=  group_lines.second[0] : " << group_lines[i].second[0] << " group_lines.second[1]: " << group_lines[i].second[1] << endl;
#endif
		// calculete width
		{
			if (group_lines[i].first[0] != -1 && group_lines[i].first[1] != -1)
			{
				
				width_value = this->method1(group_lines[i].first, plane_line_out);
				first_method = 1;
				
			}
			else
			{
				if ((group_lines[i].first[0] == -1 && group_lines[i].first[1] == -1)
					|| (group_lines[i].second[0] == -1 && group_lines[i].second[1] == -1))	
				{	
					this->check_line_with_break(holes_location[i],
						plane_line_out,
						group_lines[i],
						group_holes[i],
						input_plane_normal[measure_index]);
#ifdef  OUTPUT_DEBUG_INFO
					cout << "===group_lines.first[0] : " << group_lines[i].first[0] << " group_lines.first[1]: " << group_lines[i].first[1] << endl;
					cout << "===group_lines.second[0] : " << group_lines[i].second[0] << " group_lines.second[1]: " << group_lines[i].second[1] << endl;
#endif
					if (group_lines[i].first[0] == -1 && group_lines[i].first[1] == -1) 
					{
#ifdef  OUTPUT_DEBUG_INFO
						resultA << "====width_value = -1;" << endl;
#endif
						width_value = -1;
					}
				}

				if ((group_lines[i].first[0] != -1 || group_lines[i].first[1] != -1) && //宽和高各至少各检测到一条边
					(group_lines[i].second[0] != -1 || group_lines[i].second[1] != -1))
				{ // close to edge
					int valid_line = group_lines[i].first[0] != -1 ? group_lines[i].first[0] : group_lines[i].first[1];
					int refer_line = group_lines[i].second[0] != -1 ? group_lines[i].second[0] : group_lines[i].second[1];
					int reference_plane_index = -1;
					
					width_value = this->GetBestReferencePlaneDistance(valid_line, plane_line_out, refer_line,
						input_plane_points,
						input_plane_normal,
						input_plane_center,
						measure_index,
						reference_plane_index);
					
					first_method = 2;
					reference_w_index = reference_plane_index;
#ifdef  OUTPUT_DEBUG_INFO_SV
					cout << "=== width reference_plane_index: " << reference_plane_index << endl;
					if (reference_plane_index > -1)
						SaveSceneData(output_path, reference_plane_index*1000+66, input_plane_points[reference_plane_index]);
#endif
					if (reference_plane_index > -1) // check reference plane distance
					{
						
						float dist1 = Util_Math::DWComputePointToPlaneDist<cv::Point3f>(plane_line_out[refer_line].line_seg_start,
							input_plane_normal[reference_plane_index],
							input_plane_center[reference_plane_index]);
						float dist2 = Util_Math::DWComputePointToPlaneDist<cv::Point3f>(plane_line_out[refer_line].line_seg_end,
							input_plane_normal[reference_plane_index],
							input_plane_center[reference_plane_index]);
						float dist = dist1 < dist2 ? dist1 : dist2;
#ifdef  OUTPUT_DEBUG_INFO						
						cout << "dis of nearest refline endpoint to refer plan is:" << dist	<< "  samller than threshold: " 
							 << close_to_plane_height_threshold << endl;
#endif
						if (dist > close_to_plane_width_threshold)//参考线段的近端到参考平面的距离大于 50毫米时不适用线到面的距离
						{			
							int hole_id = group_lines[i].first[0] != -1 ? group_holes[i].first[0] : group_holes[i].first[1];

							int match_line = check_line_single(holes_location[i], hole_id, plane_line_out);
							if (match_line != -1) {
#ifdef  OUTPUT_DEBUG_INFO
								cout << "width use method 3， valid line: " << valid_line << " valid line: " << match_line << endl;
#endif
								width_value = this->method3(valid_line, match_line, plane_line_out);
								first_method = 3;
							}
						}
					}
				}
			}
			if (width_value < 0) {
				return_value = false;
			}
		}
		// calculete height
		{
			if (group_lines[i].second[0] != -1 && group_lines[i].second[1] != -1)
			{
				height_value = this->method1(group_lines[i].second, plane_line_out);
				second_method = 1;
			}
			else {
				if (group_lines[i].second[0] == -1 && group_lines[i].second[1] == -1) // no height board found
				{
					
					this->check_line_with_break(holes_location[i],
						plane_line_out,
						group_lines[i],
						group_holes[i],
						input_plane_normal[measure_index]);
					
					if (group_lines[i].second[0] == -1 && group_lines[i].second[1] == -1) {
#ifdef  OUTPUT_DEBUG_INFO
						resultA << "== height_value = -1;" << endl;
#endif
						height_value = -1;
					}
				}
				if ((group_lines[i].second[0] != -1 || group_lines[i].second[1] != -1) &&
					(group_lines[i].first[0] != -1 || group_lines[i].first[1] != -1))
				{ // close to edge
					int valid_line = group_lines[i].second[0] != -1 ? group_lines[i].second[0] : group_lines[i].second[1];
					int refer_line = group_lines[i].first[0] != -1 ? group_lines[i].first[0] : group_lines[i].first[1];
#ifdef  OUTPUT_DEBUG_INFO
					cout << "height valid_line: " << valid_line << " refer_line:" << refer_line << endl;
					
#endif
					int reference_plane_index;
					height_value = this->GetBestReferencePlaneDistance(valid_line, plane_line_out,
						refer_line, input_plane_points,
						input_plane_normal, input_plane_center, measure_index,
						reference_plane_index);
					
					reference_h_index = reference_plane_index;
					second_method = 2;
#ifdef  OUTPUT_DEBUG_INFO_SV
					cout << "=== height reference_plane_index: " << reference_plane_index << endl;
					if (reference_plane_index > -1)
						SaveSceneData(output_path, reference_plane_index*1000+66, input_plane_points[reference_plane_index]);
#endif
					

					if (reference_plane_index > -1) {
						
						float dist1 = Util_Math::DWComputePointToPlaneDist<cv::Point3f>(plane_line_out[refer_line].line_seg_start,
							input_plane_normal[reference_plane_index],
							input_plane_center[reference_plane_index]);
						float dist2 = Util_Math::DWComputePointToPlaneDist<cv::Point3f>(plane_line_out[refer_line].line_seg_end,
							input_plane_normal[reference_plane_index],
							input_plane_center[reference_plane_index]);
						
						float dist = dist1 < dist2 ? dist1 : dist2;
#ifdef  OUTPUT_DEBUG_INFO
						cout << "dis of nearest refline endpoint to refer plan is:" << dist << "  samller than threshold: " << close_to_plane_height_threshold << endl;
#endif
						if (dist > close_to_plane_height_threshold) 
						{							
							int hole_id = group_lines[i].second[0] != -1 ? group_holes[i].second[0] : group_holes[i].second[1];
							int match_line = check_line_single(holes_location[i], hole_id, plane_line_out);
							if (match_line != -1) {
#ifdef  OUTPUT_DEBUG_INFO
								cout << "height use method 3， valid line: " << valid_line << " valid line: " << match_line << endl;
#endif
								height_value = this->method3(valid_line, match_line, plane_line_out);
								second_method = 3;
							}
						}
					}
				}
			}
			if (height_value < 0) {
				return_value = false;
			}
		}
		int reference_plane_index1 = -1, reference_plane_index2 = -1;
#ifdef TESTPA
		{
			std::vector<cv::Point3f> ref_pts;
			ref_pts.resize(2);
			if ((width_value = -1) && (group_lines[i].second[0] != -1 && group_lines[i].second[1] != -1)) 
			{
				if (group_holes[i].second[0] == 1) { ref_pts[0] = holes_location[i][1]; ref_pts[1] = holes_location[i][2]; }
				if (group_holes[i].second[0] == 2) { ref_pts[0] = holes_location[i][2]; ref_pts[1] = holes_location[i][3]; }
				if (group_holes[i].second[0] == 3) { ref_pts[0] = holes_location[i][3]; ref_pts[1] = holes_location[i][0]; }
				if (group_holes[i].second[0] == 4) { ref_pts[0] = holes_location[i][0]; ref_pts[1] = holes_location[i][1]; }
				cout << ref_pts[0] <<"  "<< ref_pts[0] << endl;
				cout << plane_line_out[group_holes[i].second[0]].line_seg_start<<" " << plane_line_out[group_holes[i].second[0]].line_seg_end << endl;
				GetReferencePlaneForLine(
					group_lines[i].second[0],
					plane_line_out,
					ref_pts,
					input_plane_points,
					input_plane_normal,
					input_plane_center,
					measure_index,
					reference_plane_index1);
				if (group_holes[i].second[0] == 1) { ref_pts[0] = holes_location[i][0]; ref_pts[1] = holes_location[i][3]; }
				if (group_holes[i].second[0] == 2) { ref_pts[0] = holes_location[i][1]; ref_pts[1] = holes_location[i][0]; }
				if (group_holes[i].second[0] == 3) { ref_pts[0] = holes_location[i][2]; ref_pts[1] = holes_location[i][1]; }
				if (group_holes[i].second[0] == 4) { ref_pts[0] = holes_location[i][3]; ref_pts[1] = holes_location[i][3]; }
				cout << ref_pts[0] << "  " << ref_pts[0] << endl;
				cout << plane_line_out[group_lines[i].second[0]].line_seg_start << " " << plane_line_out[group_lines[i].second[0]].line_seg_end << endl;
				GetReferencePlaneForLine(///T
					group_lines[i].second[0],
					plane_line_out,
					ref_pts,
					input_plane_points,
					input_plane_normal,
					input_plane_center,
					measure_index,
					reference_plane_index2);
			}
			if ((height_value = -1) && (group_lines[i].first[0] != -1 && group_lines[i].first[1] != -1))
			{
				if (group_holes[i].first[0] == 1) { ref_pts[0] = holes_location[i][1]; ref_pts[0] = holes_location[i][2]; }
				if (group_holes[i].first[0] == 2) { ref_pts[0] = holes_location[i][2]; ref_pts[0] = holes_location[i][3]; }
				if (group_holes[i].first[0] == 3) { ref_pts[0] = holes_location[i][3]; ref_pts[0] = holes_location[i][0]; }
				if (group_holes[i].first[0] == 4) { ref_pts[0] = holes_location[i][0]; ref_pts[0] = holes_location[i][1]; }
				cout << ref_pts[0] << "  " << ref_pts[0] << endl;
				cout << plane_line_out[group_lines[i].first[0]].line_seg_start << " " << plane_line_out[group_lines[i].first[0]].line_seg_end << endl;
				GetReferencePlaneForLine(///T
					group_lines[i].first[0],
					plane_line_out,
					ref_pts,
					input_plane_points,
					input_plane_normal,
					input_plane_center,
					measure_index,
					reference_plane_index1);
				if (group_holes[i].first[0] == 1) { ref_pts[0] = holes_location[i][0]; ref_pts[0] = holes_location[i][3]; }
				if (group_holes[i].first[0] == 2) { ref_pts[0] = holes_location[i][1]; ref_pts[0] = holes_location[i][0]; }
				if (group_holes[i].first[0] == 3) { ref_pts[0] = holes_location[i][2]; ref_pts[0] = holes_location[i][1]; }
				if (group_holes[i].first[0] == 4) { ref_pts[0] = holes_location[i][3]; ref_pts[0] = holes_location[i][3]; }
				GetReferencePlaneForLine(///T
					group_lines[i].first[0],
					plane_line_out,
					ref_pts,
					input_plane_points,
					input_plane_normal,
					input_plane_center,
					measure_index,
					reference_plane_index2);
			}
			cout << "reference_plane_index1: " << reference_plane_index1 << " reference_plane_index2: " << reference_plane_index2 << endl;
		}
#endif
		// calculate vertices
		{
#ifdef  OUTPUT_DEBUG_INFO
			resultA << "=========measure_index: " << measure_index <<"  hole: " << i << endl;
			resultA << "first_method: " << first_method << " second_method: " << second_method << endl;
			cout << "=========measure_index: " << measure_index << "  hole: " << i << endl;
			cout << "first_method: " << first_method << " second_method: " << second_method << endl;
			resultA << "		====F0: " << group_lines[i].first[0] << " F1: " << group_lines[i].first[1] << endl;
			resultA << "		====S0: " << group_lines[i].second[0] << " S1: " << group_lines[i].second[1] << endl;
			cout << "		====F0: " << group_lines[i].first[0] << " F1: " << group_lines[i].first[1] << endl;
			cout << "		====S0: " << group_lines[i].second[0] << " S1: " << group_lines[i].second[1] << endl;
#endif
			cv::Mat rotation_matrix_R;
			float AXIS_X_DIRECTION[3] = { 1.f,0.f,0.f };
			float vector_before[3] = { input_plane_normal[measure_index].x,
									   input_plane_normal[measure_index].y,
										input_plane_normal[measure_index].z };
			cv::Mat rotation_matrix = MeasureBase::CalRotationMatrixFromVectors(vector_before, AXIS_X_DIRECTION);
			cv::transpose(rotation_matrix, rotation_matrix_R);

#if 0 //def  OUTPUT_DEBUG_INFO
			PointReconstruction PR("log.txt");
			cout << "0 original holes:\n" << holes_location[i] << endl;
			holes_location[i] = PR.SortVertices(holes_location[i], input_plane_normal[measure_index]);
			cout << "0 sorted holes:\n" << holes_location[i] << endl;
#endif
			std::vector<cv::Point3f> holes_calx(4);
			std::vector<cv::Point3f> holes_ori_yz(4);
			for (int h = 0; h < holes_location[i].size(); h++) {
				MeasureBase::RotatePoint(holes_location[i][h], rotation_matrix, holes_ori_yz[h]);
			}

			cv::Point2f L0Ps, L0Pe, L1Ps, L1Pe, L2Ps, L2Pe, L3Ps, L3Pe;
			float Lx[4];
			std::vector<std::pair<bool, std::vector<cv::Point2f>>> Par(4);
			for (int j = 0; j < Par.size(); j++)
			{
				Par[j].first = false;
			}

			if (group_lines[i].first[0] >= 0)
			{
				ModuleStruct::Point3f line_seg_start, line_seg_end;
				MeasureBase::RotatePoint(plane_line_out[group_lines[i].first[0]].line_seg_start, rotation_matrix, line_seg_start);
				MeasureBase::RotatePoint(plane_line_out[group_lines[i].first[0]].line_seg_end, rotation_matrix, line_seg_end);
				Lx[0] = line_seg_start.x;
				L0Ps.x = line_seg_start.y;
				L0Ps.y = line_seg_start.z;
				L0Pe.x = line_seg_end.y;
				L0Pe.y = line_seg_end.z;
				Par[0].second.push_back(L0Ps);
				Par[0].second.push_back(L0Pe);
				Par[0].first = true;
			}
			if (group_lines[i].first[1] >= 0)
			{
				ModuleStruct::Point3f line_seg_start, line_seg_end;
				MeasureBase::RotatePoint(plane_line_out[group_lines[i].first[1]].line_seg_start, rotation_matrix, line_seg_start);
				MeasureBase::RotatePoint(plane_line_out[group_lines[i].first[1]].line_seg_end, rotation_matrix, line_seg_end);
				Lx[1] = line_seg_start.x;
				L1Ps.x = line_seg_start.y;
				L1Ps.y = line_seg_start.z;
				L1Pe.x = line_seg_end.y;
				L1Pe.y = line_seg_end.z;
				Par[1].second.push_back(L1Ps);
				Par[1].second.push_back(L1Pe);
				Par[1].first = true;
			}
			if (group_lines[i].second[0] >= 0)
			{
				ModuleStruct::Point3f line_seg_start, line_seg_end;
				MeasureBase::RotatePoint(plane_line_out[group_lines[i].second[0]].line_seg_start, rotation_matrix, line_seg_start);
				MeasureBase::RotatePoint(plane_line_out[group_lines[i].second[0]].line_seg_end, rotation_matrix, line_seg_end);
				Lx[2] = line_seg_start.x;
				L2Ps.x = line_seg_start.y;
				L2Ps.y = line_seg_start.z;
				L2Pe.x = line_seg_end.y;
				L2Pe.y = line_seg_end.z;
				Par[2].second.push_back(L2Ps);
				Par[2].second.push_back(L2Pe);
				Par[2].first = true;
			}
			if (group_lines[i].second[1] >= 0)
			{
				ModuleStruct::Point3f line_seg_start, line_seg_end;
				MeasureBase::RotatePoint(plane_line_out[group_lines[i].second[1]].line_seg_start, rotation_matrix, line_seg_start);
				MeasureBase::RotatePoint(plane_line_out[group_lines[i].second[1]].line_seg_end, rotation_matrix, line_seg_end);
				Lx[3] = line_seg_start.x;
				L3Ps.x = line_seg_start.y;
				L3Ps.y = line_seg_start.z;
				L3Pe.x = line_seg_end.y;
				L3Pe.y = line_seg_end.z;
				Par[3].second.push_back(L3Ps);
				Par[3].second.push_back(L3Pe);
				Par[3].first = true;
			}

			int cnt = 0;
			for (int j = 0; j < Par.size(); j++) {
				if (Par[j].first) {
					cnt++;
				}
			}
			float P0[4], P1[4], P2[4];
			cv::Point3f CP1, CP2, CP3, CP4, LP0, LP1, cpr31, cpr32;
			cv::Point2f cpr21, cpr22;

#ifdef  OUTPUT_DEBUG_INFO
			cout << "##cnt == "<<cnt<< endl;
			cout << "=== width reference_plane_index: " << reference_w_index << endl;
			cout << "=== height reference_plane_index: " << reference_h_index << endl;
			resultA << "##cnt == " << cnt << endl;
			resultA << "=== width reference_plane_index: " << reference_w_index << endl;
			resultA << "=== height reference_plane_index: " << reference_h_index << endl;
#endif
			if (cnt == 4) {
				std::vector<cv::Point2f> holes_cro_yz(4);
				getCorssedPoint(Par[0].second[0], Par[0].second[1], Par[2].second[0], Par[2].second[1], holes_cro_yz[0]);
				getCorssedPoint(Par[0].second[0], Par[0].second[1], Par[3].second[0], Par[3].second[1], holes_cro_yz[1]);
				getCorssedPoint(Par[1].second[0], Par[1].second[1], Par[2].second[0], Par[2].second[1], holes_cro_yz[2]);
				getCorssedPoint(Par[1].second[0], Par[1].second[1], Par[3].second[0], Par[3].second[1], holes_cro_yz[3]);
				for (int m = 0; m < holes_ori_yz.size(); m++)
				{
					int idx = -1;
					float dis = INFINITY;
					for (int n = 0; n < holes_cro_yz.size(); n++) {
						float temp = std::sqrt(std::pow(holes_ori_yz[m].y - holes_cro_yz[n].x, 2.f)
							+ std::pow(holes_ori_yz[m].z - holes_cro_yz[n].y, 2.f));
						if (temp < dis) {
							dis = temp;
							idx = n;
						}
					}
					//cout << "idx:" << idx << endl;;
					holes_ori_yz[m].y = holes_cro_yz[idx].x;
					holes_ori_yz[m].z = holes_cro_yz[idx].y;
				}
				for (int h = 0; h < holes_ori_yz.size(); h++) {
					MeasureBase::RotatePoint(holes_ori_yz[h], rotation_matrix_R, holes_calx[h]);
				}
				CP1 = holes_calx[0]; CP2 = holes_calx[1];	CP3 = holes_calx[2];	CP4 = holes_calx[3];
#ifdef  OUTPUT_DEBUG_INFO_SV
				std::vector<cv::Point3f> temp2;
				temp2.push_back(holes_calx[0]);
				SaveSceneData(output_path,  measure_index * 100000 + i*100 + 1, temp2);
				temp2.clear();
				temp2.push_back(holes_calx[1]);
				SaveSceneData(output_path, measure_index * 100000 + i * 100 + 2, temp2);
				temp2.clear();
				temp2.push_back(holes_calx[2]);
				SaveSceneData(output_path, measure_index * 100000 + i * 100 + 3, temp2);
				temp2.clear();
				temp2.push_back(holes_calx[3]);
				SaveSceneData(output_path, measure_index * 100000 + i * 100 + 4, temp2);
#endif
			}
			else if (cnt == 3) {
				if ((Par[0].first) && (Par[1].first))///w
				{
					if (Par[2].first) {
						getCorssedPoint(Par[0].second[0], Par[0].second[1], Par[2].second[0], Par[2].second[1], cpr21);
						getCorssedPoint(Par[1].second[0], Par[1].second[1], Par[2].second[0], Par[2].second[1], cpr22);
					}
					else {
						getCorssedPoint(Par[0].second[0], Par[0].second[1], Par[3].second[0], Par[3].second[1], cpr21);
						getCorssedPoint(Par[1].second[0], Par[1].second[1], Par[3].second[0], Par[3].second[1], cpr22);
					}
					cpr31.x = Lx[0];
					cpr32.x = Lx[1];
					cv::Point3f nb = input_plane_normal[reference_h_index];
					cv::Point3f cb = input_plane_center[reference_h_index];
#ifdef  OUTPUT_DEBUG_INFO
					cout << "######## w2 h1" << endl;
#endif
					int valid_line = group_lines[i].first[0];
					LP0 = plane_line_out[valid_line].line_seg_start;
					LP1 = plane_line_out[valid_line].line_seg_end;
					P0[0] = nb.x; P0[1] = nb.y;
					P0[2] = nb.z; P0[3] = -(P0[0] * cb.x + P0[1] * cb.y + P0[2] * cb.z);
					intersectionLinePlane(CP3, LP0, LP1, P0);

					valid_line = group_lines[i].first[1];
					LP0 = plane_line_out[valid_line].line_seg_start;
					LP1 = plane_line_out[valid_line].line_seg_end;
					intersectionLinePlane(CP4, LP0, LP1, P0);
				}
				else {
					if (Par[0].first) {
						getCorssedPoint(Par[0].second[0], Par[0].second[1], Par[2].second[0], Par[2].second[1], cpr21);
						getCorssedPoint(Par[0].second[0], Par[0].second[1], Par[3].second[0], Par[3].second[1], cpr22);

					}
					else {
						getCorssedPoint(Par[1].second[0], Par[1].second[1], Par[2].second[0], Par[2].second[1], cpr21);
						getCorssedPoint(Par[1].second[0], Par[1].second[1], Par[3].second[0], Par[3].second[1], cpr22);
					}
					cpr31.x = Lx[2];
					cpr32.x = Lx[3];
					cv::Point3f nb = input_plane_normal[reference_w_index];
					cv::Point3f cb = input_plane_center[reference_w_index];
#ifdef  OUTPUT_DEBUG_INFO
					cout << "######## h2 w1" << endl;
#endif
					int valid_line = group_lines[i].second[0];
					//cout << "valid_line for CP3:" << valid_line << endl;
					LP0 = plane_line_out[valid_line].line_seg_start;
					LP1 = plane_line_out[valid_line].line_seg_end;
					P0[0] = nb.x; P0[1] = nb.y;
					P0[2] = nb.z; P0[3] = -(P0[0] * cb.x + P0[1] * cb.y + P0[2] * cb.z);
					intersectionLinePlane(CP3, LP0, LP1, P0);

					valid_line = group_lines[i].second[1];
					//cout << "valid_line for CP4:" << valid_line << endl;
					LP0 = plane_line_out[valid_line].line_seg_start;
					LP1 = plane_line_out[valid_line].line_seg_end;
					intersectionLinePlane(CP4, LP0, LP1, P0);
				}
				cpr31.y = cpr21.x; cpr31.z = cpr21.y;
				cpr32.y = cpr22.x; cpr32.z = cpr22.y;
				MeasureBase::RotatePoint(cpr31, rotation_matrix_R, CP1);
				MeasureBase::RotatePoint(cpr32, rotation_matrix_R, CP2);
#ifdef OUTPUT_DEBUG_INFO_SV
				cout << "CP1: " << CP1 << endl;
				cout << "CP2: " << CP2 << endl;
				cout << "CP3: " << CP3 << endl;
				cout << "CP4: " << CP4 << endl;
				std::vector<cv::Point3f> temp2;
				temp2.push_back(CP1);
				SaveSceneData(output_path, measure_index * 100000 + i * 100 + 1, temp2);
				temp2.clear();
				temp2.push_back(CP2);
				SaveSceneData(output_path, measure_index * 100000 + i * 100 + 2, temp2);
				temp2.clear();
				temp2.push_back(CP3);
				SaveSceneData(output_path, measure_index * 100000 + i * 100 + 3, temp2);
				temp2.clear();
				temp2.push_back(CP4);
				SaveSceneData(output_path, measure_index * 100000 + i * 100 + 4, temp2);
#endif
			}
			//===================================================
			else if (cnt == 2) {
				if ((Par[0].first || Par[1].first) && (Par[2].first || Par[3].first))
				{
					int idx[2];
					int cn = 0;
					for (int j = 0; j < Par.size(); j++) {
						if (Par[j].first) {
							idx[cn] = j;
							cn++;
						}
					}
					getCorssedPoint(Par[idx[0]].second[0], Par[idx[0]].second[1], Par[idx[1]].second[0], Par[idx[1]].second[1], cpr22);
					cpr32.x = Lx[idx[0]];
					cpr32.y = cpr22.x;
					cpr32.z = cpr22.y;
					MeasureBase::RotatePoint(cpr32, rotation_matrix_R, CP1);
					cv::Point3f na = input_plane_normal[reference_w_index];
					cv::Point3f nb = input_plane_normal[reference_h_index];
					cv::Point3f nc = input_plane_normal[measure_index];
					cv::Point3f ca = input_plane_center[reference_w_index];
					cv::Point3f cb = input_plane_center[reference_h_index];
					cv::Point3f cc = input_plane_center[measure_index];
				//	if (first_method == 2) 
					{
						int valid_line = group_lines[i].first[0] != -1 ? group_lines[i].first[0] : group_lines[i].first[1];
#ifdef  OUTPUT_DEBUG_INFO_SV
						cout << "width valid_line:" << valid_line <<" reference_h_index:"<< reference_h_index<< endl;
#endif
						LP0 = plane_line_out[valid_line].line_seg_start;
						LP1 = plane_line_out[valid_line].line_seg_end;

						P0[0] = nb.x; P0[1] = nb.y;
						P0[2] = nb.z; P0[3] = -(P0[0] * cb.x + P0[1] * cb.y + P0[2] * cb.z);
						intersectionLinePlane(CP2, LP0, LP1, P0);
					}
					//if (second_method == 2) 
					{
						int valid_line = group_lines[i].second[0] != -1 ? group_lines[i].second[0] : group_lines[i].second[1];
#ifdef  OUTPUT_DEBUG_INFO_SV
						cout << "height valid_line:" << valid_line << " reference_w_index:" << reference_w_index << endl;
#endif
						LP0 = plane_line_out[valid_line].line_seg_start;
						LP1 = plane_line_out[valid_line].line_seg_end;
						P1[0] = na.x; P1[1] = na.y;
						P1[2] = na.z; P1[3] = -(P1[0] * ca.x + P1[1] * ca.y + P1[2] * ca.z);
						intersectionLinePlane(CP3, LP0, LP1, P1);
					}
					std::vector<std::vector<float>> param = { { na.x, na.y, na.z },
															  { nb.x, nb.y, nb.z },
															  { nc.x, nc.y, nc.z } };
					std::vector<float> Cresult = { na.x*ca.x + na.y*ca.y + na.z*ca.z,
												   nb.x*cb.x + nb.y*cb.y + nb.z*cb.z,
												   nc.x*cc.x + nc.y*cc.y + nc.z*cc.z };
					int nResult = Util_Math::LUDecomposition(param, Cresult);
					if (nResult == 1)
					{
						CP4.x = Cresult[0];
						CP4.y = Cresult[1];
						CP4.z = Cresult[2];
					}
#ifdef  OUTPUT_DEBUG_INFO_SV
					cout << "CP1: " << CP1 << endl;
					cout << "CP2: " << CP2 << endl;
					cout << "CP3: " << CP3 << endl;
					cout << "CP4: " << CP4 << endl;
					std::vector<cv::Point3f> temp2;
					temp2.push_back(CP1);
					SaveSceneData(output_path, measure_index * 100000 + i * 100 + 1, temp2);
					temp2.clear();
					temp2.push_back(CP2);
					SaveSceneData(output_path, measure_index * 100000 + i * 100 + 2, temp2);
					temp2.clear();
					temp2.push_back(CP3);
					SaveSceneData(output_path, measure_index * 100000 + i * 100 + 3, temp2);
					temp2.clear();
					temp2.push_back(CP4);
					SaveSceneData(output_path, measure_index * 100000 + i * 100 + 4, temp2);
#endif
				}
				else { // detected two parallel line
#ifdef OUTPUT_DEBUG_INFO
					cout << "Detected two parallel line !!!!!" << endl;
					resultA << "Detected two parallel line !!!!!" << endl;
#endif	
#ifdef TESTPA
					cv::Point3f nb, cb;
					if (group_lines[i].first[0] != -1 && group_lines[i].first[1] != -1 
						&& reference_plane_index1 != -1 && reference_plane_index2 != -1) 
					{					
						LP0 = plane_line_out[group_lines[i].first[0]].line_seg_start;
						LP1 = plane_line_out[group_lines[i].first[0]].line_seg_end;
						P0[0] = input_plane_normal[reference_plane_index1].x;
						P0[1] = input_plane_normal[reference_plane_index1].y;
						P0[2] = input_plane_normal[reference_plane_index1].z;
						P0[3] = -(P0[0] * input_plane_center[reference_plane_index1].x + P0[1] * 
							              input_plane_center[reference_plane_index1].y + P0[2] * 
							              input_plane_center[reference_plane_index1].z);
						intersectionLinePlane(CP1, LP0, LP1, P0);
						P0[0] = input_plane_normal[reference_plane_index2].x;
						P0[1] = input_plane_normal[reference_plane_index2].y;
						P0[2] = input_plane_normal[reference_plane_index2].z;
						P0[3] = -(P0[0] * input_plane_center[reference_plane_index2].x + P0[1] *
							input_plane_center[reference_plane_index2].y + P0[2] *
							input_plane_center[reference_plane_index2].z);
						intersectionLinePlane(CP2, LP0, LP1, P0);

						LP0 = plane_line_out[group_lines[i].first[1]].line_seg_start;
						LP1 = plane_line_out[group_lines[i].first[1]].line_seg_end;
						P0[0] = input_plane_normal[reference_plane_index1].x;
						P0[1] = input_plane_normal[reference_plane_index1].y;
						P0[2] = input_plane_normal[reference_plane_index1].z;
						P0[3] = -(P0[0] * input_plane_center[reference_plane_index1].x + P0[1] *
							input_plane_center[reference_plane_index1].y + P0[2] *
							input_plane_center[reference_plane_index1].z);
						intersectionLinePlane(CP3, LP0, LP1, P0);
						P0[0] = input_plane_normal[reference_plane_index2].x;
						P0[1] = input_plane_normal[reference_plane_index2].y;
						P0[2] = input_plane_normal[reference_plane_index2].z;
						P0[3] = -(P0[0] * input_plane_center[reference_plane_index2].x + P0[1] *
							input_plane_center[reference_plane_index2].y + P0[2] *
							input_plane_center[reference_plane_index2].z);
						intersectionLinePlane(CP4, LP0, LP1, P0);	
					}
					else if (group_lines[i].second[0] != -1 && group_lines[i].second[1] != -1
						&& reference_plane_index1 != -1 && reference_plane_index2 != -1) 
					{
						LP0 = plane_line_out[group_lines[i].second[0]].line_seg_start;
						LP1 = plane_line_out[group_lines[i].second[0]].line_seg_end;
						P0[0] = input_plane_normal[reference_plane_index1].x;
						P0[1] = input_plane_normal[reference_plane_index1].y;
						P0[2] = input_plane_normal[reference_plane_index1].z;
						P0[3] = -(P0[0] * input_plane_center[reference_plane_index1].x + P0[1] *
							input_plane_center[reference_plane_index1].y + P0[2] *
							input_plane_center[reference_plane_index1].z);
						intersectionLinePlane(CP1, LP0, LP1, P0);
						P0[0] = input_plane_normal[reference_plane_index2].x;
						P0[1] = input_plane_normal[reference_plane_index2].y;
						P0[2] = input_plane_normal[reference_plane_index2].z;
						P0[3] = -(P0[0] * input_plane_center[reference_plane_index2].x + P0[1] *
							input_plane_center[reference_plane_index2].y + P0[2] *
							input_plane_center[reference_plane_index2].z);
						intersectionLinePlane(CP2, LP0, LP1, P0);

						LP0 = plane_line_out[group_lines[i].second[1]].line_seg_start;
						LP1 = plane_line_out[group_lines[i].second[1]].line_seg_end;
						P0[0] = input_plane_normal[reference_plane_index1].x;
						P0[1] = input_plane_normal[reference_plane_index1].y;
						P0[2] = input_plane_normal[reference_plane_index1].z;
						P0[3] = -(P0[0] * input_plane_center[reference_plane_index1].x + P0[1] *
							input_plane_center[reference_plane_index1].y + P0[2] *
							input_plane_center[reference_plane_index1].z);
						intersectionLinePlane(CP3, LP0, LP1, P0);
						P0[0] = input_plane_normal[reference_plane_index2].x;
						P0[1] = input_plane_normal[reference_plane_index2].y;
						P0[2] = input_plane_normal[reference_plane_index2].z;
						P0[3] = -(P0[0] * input_plane_center[reference_plane_index2].x + P0[1] *
							input_plane_center[reference_plane_index2].y + P0[2] *
							input_plane_center[reference_plane_index2].z);
						intersectionLinePlane(CP4, LP0, LP1, P0);
#ifdef  OUTPUT_DEBUG_INFO
						cout << "p CP1: " << CP1 << endl;
						cout << "p CP2: " << CP2 << endl;
						cout << "p CP3: " << CP3 << endl;
						cout << "p CP4: " << CP4 << endl;
#endif

					}
					else {
						CP1 = holes_location[i][0];
						CP2 = holes_location[i][1];
						CP3 = holes_location[i][2];
						CP4 = holes_location[i][3];
					}
#else
					CP1 = holes_location[i][0];
					CP2 = holes_location[i][1];
					CP3 = holes_location[i][2];
					CP4 = holes_location[i][3];
#endif
				}
			}
			else {
#ifdef OUTPUT_DEBUG_INFO
				cout << "Valid board count: " <<cnt<<" !!!!!" << endl;
				resultA << "Valid board count: " << cnt << " !!!!!" << endl;
#endif		
				CP1 = holes_location[i][0];
				CP2 = holes_location[i][1];
				CP3 = holes_location[i][2];
				CP4 = holes_location[i][3];
			}
#ifdef  OUTPUT_DEBUG_INFO	
			cout << "orginal holes:\n" << holes_location[i] << endl;
			resultA << "orginal holes:\n" << holes_location[i] << endl;
#endif
#ifdef	OUTPUT_DEBUG_INFO_SV
			std::vector<cv::Point3f> temp2;
			temp2.push_back(holes_location[i][0]);
			
			SaveSceneData(output_path, 88 * 1000000 + measure_index * 10000 + i * 100 + 1, temp2);
			temp2.clear();
			temp2.push_back(holes_location[i][1]);
			SaveSceneData(output_path, 88 * 1000000 + measure_index * 10000 + i * 100 + 2, temp2);
			temp2.clear();
			temp2.push_back(holes_location[i][2]);
			SaveSceneData(output_path, 88 * 1000000 + measure_index * 10000 + i * 100 + 3, temp2);
			temp2.clear();
			temp2.push_back(holes_location[i][3]);
			SaveSceneData(output_path, 88 * 1000000 + measure_index * 10000 + i * 100 + 4, temp2);
#endif
			holes_location[i][0] = CP1;
			holes_location[i][1] = CP2;
			holes_location[i][2] = CP3;
			holes_location[i][3] = CP4;
#ifdef  OUTPUT_DEBUG_INFO
			//	holes_location[i] = PR.SortVertices(holes_location[i], input_plane_normal[measure_index]);
			cout << "remeasured holes:\n" << holes_location[i] << endl;
			resultA << "remeasured holes:\n" << holes_location[i] << endl;
#endif
	}

		group_method[i] = std::pair<int, int>(first_method, second_method);
		result[i] = std::pair<float, float>(width_value, height_value);
#ifdef  OUTPUT_DEBUG_INFO
		cout << "width_value: " << width_value << " height_value: " << height_value << endl;
		resultA << "width_value: " << width_value << " height_value: " << height_value << endl;
#endif
	}///for holes_location[i]
//	std::cout << " do measurement time cost: " << (clock() - t5) / float(CLOCKS_PER_SEC) << std::endl;



	//calibrate
	float plane_mse = GetPlaneMse(input_plane_center[measure_index], 
								  input_plane_normal[measure_index], 
								  input_plane_points[measure_index]);
	this->calibrate(result, group_method, 
					measurement_type, 
					measure_index, 
					plane_mse);
#endif
	return return_value;
}

bool LineToolHandle::check_line_match(///T
	cv::Point3f p1, cv::Point3f p2, 
	Line3DSegOutItemDebug& line, 
	float line_angle_threshold, //8
	float line_dist_threshold, 
	float line_length_threshold, bool output_info)
{
	bool angle_match = false, dist_match = false, length_match = false;
	// check angle match
	float max_direction_diff = static_cast<float>(std::cos(line_angle_threshold * M_PI / 180));
	Point3f line_direction = Util_Math::vec3_normalize(p2 - p1);
	float direction_diff = std::fabsf(Util_Math::vec3_normalize(line.line_direction).dot(line_direction));
	angle_match = direction_diff >= max_direction_diff;

	// check dist match
	Point3f line_center_tmp((p1.x + p2.x) / 2, (p1.y + p2.y) / 2, (p1.z + p2.z) / 2);
	float center_dist = Util_Math::DWComputePointToPointDist<Point3f>(line_center_tmp, line.line_center);
	//cout << line_center_tmp <<" , "<< line.line_center << endl;
	dist_match = center_dist <= line_dist_threshold;//100mm
	
	// check length
	float inlength = Util_Math::DWComputePointToPointDist<Point3f>(p1, p2);
	float linelength = Util_Math::DWComputePointToPointDist<Point3f>(line.line_seg_start, line.line_seg_end);
	//length_match = std::fabs(inlength - linelength) <= line_length_threshold;///200mm
	length_match = std::fabs(inlength - linelength) <= inlength / 2.0;
	if (output_info) {
		std::cout << "angle_match: " << angle_match << "  direction_diff:" << direction_diff << std::endl;
		std::cout << "dist_match: " << dist_match <<"  center_dist:" << center_dist << std::endl;
		std::cout << "length_match: "<< length_match <<"   "<< std::fabs(inlength - linelength) << std::endl;
	}
	return angle_match && dist_match && length_match;
}

bool LineToolHandle::check_line_match_method3(
	cv::Point3f p1, cv::Point3f p2, Line3DSegOutItemDebug& line, 
	float line_angle_threshold, 
	float line_dist_threshold, 
	float line_length_threshold, 
	bool output_info)
{
	bool angle_match = false, dist_match = false, in_hole_match = false, length_match = false;
	// check angle match
	float max_direction_diff = static_cast<float>(std::cos(line_angle_threshold * M_PI / 180));
	Point3f line_direction = Util_Math::vec3_normalize(p2 - p1);
	float direction_diff = std::fabsf(Util_Math::vec3_normalize(line.line_direction).dot(line_direction));
	angle_match = direction_diff >= max_direction_diff;
	// check dist match
	float dist1 = util_UNRE::dist_point_to_line(p1, line.line_seg_start, line.line_seg_end);
	float dist2 = util_UNRE::dist_point_to_line(p2, line.line_seg_start, line.line_seg_end);
	float end_dist = dist1 < dist2 ? dist1 : dist2;
	dist_match = end_dist <= line_dist_threshold;
	//check point in hole
	float start_dist1 = Util_Math::DWComputePointToPointDist<Point3f>(line.line_seg_start, p1);
	float start_dist2 = Util_Math::DWComputePointToPointDist<Point3f>(line.line_seg_start, p2);
	float end_dist1 = Util_Math::DWComputePointToPointDist<Point3f>(line.line_seg_end, p1);
	float end_dist2 = Util_Math::DWComputePointToPointDist<Point3f>(line.line_seg_end, p2);
	float hole_length = Util_Math::DWComputePointToPointDist<Point3f>(p1, p2);
	in_hole_match = ((start_dist1 + start_dist2 - hole_length) < check_line_method3_in_hole_dist) && 
		            ((end_dist1   + end_dist2   - hole_length) < check_line_method3_in_hole_dist);
	// check length
	float inlength = Util_Math::DWComputePointToPointDist<Point3f>(p1, p2);
	float linelength = Util_Math::DWComputePointToPointDist<Point3f>(line.line_seg_start, line.line_seg_end);
	length_match = std::fabs(inlength - linelength) <= line_length_threshold;
	if (output_info) {
		std::cout << "angle_match: " << angle_match << std::endl;
		std::cout << "end_dist: " << end_dist << std::endl;
		std::cout << "length_diff: " << std::fabs(inlength - linelength) << std::endl;
		std::cout << "in_hole_diff1: " << (start_dist1 + start_dist2 - hole_length) << std::endl;
		std::cout << "in_hole_diff2: " << (end_dist1 + end_dist2 - hole_length) << std::endl;
	}
	return angle_match && dist_match && length_match && in_hole_match;
}

std::vector<unsigned int> LineToolHandle::filter_line_match_result(
	cv::Point3f p1, cv::Point3f p2, 
	std::vector<Line3DSegOutItemDebug>& plane_line_out, 
	std::vector<unsigned int>& line_matches, 
	bool is_method3)
{
	float check_line_dist_tmp, check_line_length_tmp;
	float high = 1.0, low = 0.0, mid = 1.0;
	unsigned int t = 0;
	while (t < max_check_line_times)///10
	{
		mid = low + (high - low) / expand_shrink_const;//2.0
		if (!is_method3) {
			check_line_dist_tmp = this->check_line_dist * mid;//100*mid
			check_line_length_tmp = this->check_line_length * mid; //200*mid
		}
		else {
			check_line_dist_tmp = this->check_line_method3_dist * mid;//150*mid
			check_line_length_tmp = this->check_line_method3_length * mid;//1500*mid
		}

		unsigned int matchs_num = line_matches.size();
		unsigned int matchs = 0;
		for (unsigned int n = 0; n < line_matches.size(); n++)
		{
			if (is_method3) {
				if (!this->check_line_match_method3(p1, p2, plane_line_out[line_matches[n]], 
													this->check_line_angle, 
					                                check_line_dist_tmp, 
					                                check_line_length_tmp, false)) 
				{
					matchs_num--;
				}
				else {
					matchs = line_matches[n];
				}
			}
			else {//
				if (!this->check_line_match(p1, p2, plane_line_out[line_matches[n]], 
											this->check_line_angle, 
											check_line_dist_tmp, 
											check_line_length_tmp, false)) 
				{
					matchs_num--;
				}
				else {
					matchs = line_matches[n];
				}
			}
		}//line_matches

		if (matchs_num > 1)
		{
			high = mid;
		}
		else if (matchs_num == 0)
		{
			low = mid;
		}
		else // filter to only one record
		{
			line_matches.resize(1);
			line_matches[0] = matchs;
			break;
		}
		t++;
	}
	if (line_matches.size() != 1) {
		std::cout << "filter line match time exceeded" << std::endl;
	}
	return line_matches;
}

// for method 3
int LineToolHandle::check_line_single(
	const std::vector<cv::Point3f>& hole_location, 
	int hole_id, 
	std::vector<Line3DSegOutItemDebug>& plane_line_out)
{
	int result = -1;
	unsigned int opposite_hole_id = hole_id - 3 >= 0 ? hole_id - 3 : hole_id + 1;
	unsigned int opposite_hole_id_next = opposite_hole_id + 1 < 4 ? opposite_hole_id + 1 : 0;
//	std::cout << "opposite_hole_id: " << opposite_hole_id << std::endl;
//	std::cout << "opposite_hole_id_next: " << opposite_hole_id_next << std::endl;
	std::vector<unsigned int> line1_matches;
	for (unsigned int n = 0; n < plane_line_out.size(); n++)
	{
		if (this->check_line_match_method3(hole_location[opposite_hole_id], 
			                               hole_location[opposite_hole_id_next], 
			                               plane_line_out[n], this->check_line_angle, 
										   this->check_line_method3_dist, //150
										   this->check_line_method3_length, //1500
											false))
		{
			line1_matches.push_back(n);
		}
	}
	if (line1_matches.size() > 1) {
		filter_line_match_result(hole_location[opposite_hole_id], 
			                     hole_location[opposite_hole_id_next], 
			                     plane_line_out, line1_matches, true);
	}
	if (line1_matches.size() == 1) {
		result = line1_matches[0];
	}
	//std::cout << "methold 3 match result: " << result << std::endl;
	return result;
}

void LineToolHandle::check_line(
	const std::vector <std::vector<cv::Point3f>>& holes_location, 
	std::vector<Line3DSegOutItemDebug>& plane_line_out,
	std::vector<std::pair<std::vector<int>, std::vector<int>>>& group_lines, 
	std::vector<std::pair<std::vector<int>, std::vector<int>>>& group_holes, 
	const cv::Point3f plane_normal)
{
	group_lines.resize(holes_location.size());
	group_holes.resize(holes_location.size());
	bool log = false;
	for (unsigned int i = 0; i < holes_location.size(); i++)//loop for each hole
	{
		assert(holes_location[i].size() == 4);///
		std::vector<unsigned int> line1_matches, line2_matches, line3_matches, line4_matches;
		if (log)
		cout << "====================================holes: " << i << endl;
		for (unsigned int n = 0; n < plane_line_out.size(); n++)/// loop and try match hole with each line
		{
			//line 1
			if (log)
			cout << "====================================line: " << n << endl;
			if (this->check_line_match(holes_location[i][0], holes_location[i][1], 
									   plane_line_out[n], 
				                       this->check_line_angle, //8
				                       this->check_line_dist,//100
				                       this->check_line_length,//20 
										log))
			{
				line1_matches.push_back(n);
			}
			//line 2
			if (this->check_line_match(holes_location[i][1], holes_location[i][2], plane_line_out[n], 
									   this->check_line_angle, this->check_line_dist, this->check_line_length, log))
			{
				line2_matches.push_back(n);
			}
			//line 3
			if (this->check_line_match(holes_location[i][2], holes_location[i][3], plane_line_out[n], 
				                       this->check_line_angle, this->check_line_dist, this->check_line_length, log))
			{
				line3_matches.push_back(n);
			}
			//line 4
			if (this->check_line_match(holes_location[i][3], holes_location[i][0], plane_line_out[n], 
				                       this->check_line_angle, this->check_line_dist, this->check_line_length, log))
			{
				line4_matches.push_back(n);
			}

		}
		if (line1_matches.size() > 1) 
		{
			filter_line_match_result(   holes_location[i][0], 
										holes_location[i][1], 
										plane_line_out, 
										line1_matches, 
										false);
		}
		if (line2_matches.size() > 1) {
			filter_line_match_result(holes_location[i][1], holes_location[i][2], plane_line_out, line2_matches, false);
		}
		if (line3_matches.size() > 1) {
			filter_line_match_result(holes_location[i][2], holes_location[i][3], plane_line_out, line3_matches, false);
		}
		if (line4_matches.size() > 1) {
			filter_line_match_result(holes_location[i][3], holes_location[i][0], plane_line_out, line4_matches, false);
		}
		int arr1[] = { line1_matches.size() > 0 ? (int)line1_matches[0] : -1, line3_matches.size() > 0 ? (int)line3_matches[0] : -1 };
		int arr2[] = { line2_matches.size() > 0 ? (int)line2_matches[0] : -1, line4_matches.size() > 0 ? (int)line4_matches[0] : -1 };

		//check width and height line
		vector<int> height_vec = { -1, -1 }, width_vec = { -1, -1 };
		vector<int> hole_vec_1 = { -1, -1 }, hole_vec_2 = { -1, -1 };
		Point3f height_normal(0.0, 0.0, 1.0);
		if (arr1[0] != -1 || arr1[1] != -1)// 13 matched.
		{
			Point3f arr1_direction = Util_Math::vec3_normalize(
									  plane_line_out[arr1[0] != -1 ? arr1[0] : arr1[1]].line_direction);
			float angle1 = std::fabsf(height_normal.dot(arr1_direction));
			if (angle1 > 0.5) {
				height_vec = { arr1[0], arr1[1] };
				hole_vec_1 = { 1, 3 };
			}else {
				width_vec = { arr1[0], arr1[1] };
				hole_vec_2 = { 1, 3 };
			}
		}
		if (arr2[0] != -1 || arr2[1] != -1)// 24 matched.
		{
			Point3f arr2_direction = Util_Math::vec3_normalize(plane_line_out[arr2[0] != -1 ? arr2[0] : arr2[1]].line_direction);
			float angle2 = std::fabsf(height_normal.dot(arr2_direction));
			if (angle2 > 0.5) {
				height_vec = { arr2[0], arr2[1] };
				hole_vec_1 = { 2, 4 };
			}
			else {
				width_vec = { arr2[0], arr2[1] };
				hole_vec_2 = { 2, 4 };
			}
		}
	//	std::cout << "check_line height_vec0: " << height_vec[0] << std::endl;
	//	std::cout << "check_line height_vec1: " << height_vec[1] << std::endl;
	//	std::cout << "check_line width_vec0: " << width_vec[0] << std::endl;
	//	std::cout << "check_line width_vec1: " << width_vec[1] << std::endl;
		group_lines[i] = std::make_pair(height_vec, width_vec);
		group_holes[i] = std::make_pair(hole_vec_1, hole_vec_2);
	}
}

void LineToolHandle::check_line_with_break(
	const std::vector<cv::Point3f>& hole_location, 
	std::vector<Line3DSegOutItemDebug>& plane_line_out,
	std::pair<std::vector<int>, std::vector<int>>& group_lines, 
	std::pair<std::vector<int>, std::vector<int>>& group_holes, 
	const cv::Point3f plane_normal)
{
	assert(hole_location.size() == 4);
	std::vector<unsigned int> line1_matches, line2_matches, line3_matches, line4_matches;
	for (unsigned int n = 0; n < plane_line_out.size(); n++)
	{
		//line 1
		if (this->check_line_match_method3(hole_location[0], hole_location[1], plane_line_out[n], 
			                              this->check_line_angle, 
			                              this->check_line_method3_dist, 
			                              this->check_line_method3_length, false))
		{
			line1_matches.push_back(n);
		}
		//line 2
		if (this->check_line_match_method3(hole_location[1], hole_location[2], plane_line_out[n], this->check_line_angle, this->check_line_method3_dist, this->check_line_method3_length, false))
		{
			line2_matches.push_back(n);
		}
		//line 3
		if (this->check_line_match_method3(hole_location[2], hole_location[3], plane_line_out[n], this->check_line_angle, this->check_line_method3_dist, this->check_line_method3_length, false))
		{
			line3_matches.push_back(n);
		}
		//line 4
		if (this->check_line_match_method3(hole_location[3], hole_location[0], plane_line_out[n], this->check_line_angle, this->check_line_method3_dist, this->check_line_method3_length, false))
		{
			line4_matches.push_back(n);
		}

	}
	if (line1_matches.size() > 1) {
		filter_line_match_result(hole_location[0], hole_location[1], plane_line_out, line1_matches, true);
	}
	if (line2_matches.size() > 1) {
		filter_line_match_result(hole_location[1], hole_location[2], plane_line_out, line2_matches, true);
	}
	if (line3_matches.size() > 1) {
		filter_line_match_result(hole_location[2], hole_location[3], plane_line_out, line3_matches, true);
	}
	if (line4_matches.size() > 1) {
		filter_line_match_result(hole_location[3], hole_location[0], plane_line_out, line4_matches, true);
	}
	int arr1[] = { line1_matches.size() > 0 ? (int)line1_matches[0] : -1,
		           line3_matches.size() > 0 ? (int)line3_matches[0] : -1 };
	int arr2[] = { line2_matches.size() > 0 ? (int)line2_matches[0] : -1,
		           line4_matches.size() > 0 ? (int)line4_matches[0] : -1 };
	//check width and height line
	vector<int> height_vec = { -1, -1 }, width_vec = { -1, -1 };
	vector<int> hole_vec_1 = { -1, -1 }, hole_vec_2 = { -1, -1 };
	Point3f height_normal(0.0, 0.0, 1.0);
	if (arr1[0] != -1 || arr1[1] != -1)
	{
		Point3f arr1_direction = Util_Math::vec3_normalize(plane_line_out[arr1[0] != -1 ? arr1[0] : arr1[1]].line_direction);
		float angle1 = std::fabsf(height_normal.dot(arr1_direction));
		if (angle1 > 0.5) {
			height_vec = { arr1[0], arr1[1] };
			hole_vec_1 = { 1, 3 };
		}
		else {
			width_vec = { arr1[0], arr1[1] };
			hole_vec_2 = { 1, 3 };
		}
	}
	if (arr2[0] != -1 || arr2[1] != -1)
	{
		Point3f arr2_direction = Util_Math::vec3_normalize(plane_line_out[arr2[0] != -1 ? arr2[0] : arr2[1]].line_direction);
		float angle2 = std::fabsf(height_normal.dot(arr2_direction));
		if (angle2 > 0.5) {
			height_vec = { arr2[0], arr2[1] };
			hole_vec_1 = { 2, 4 };
		}
		else {
			width_vec = { arr2[0], arr2[1] };
			hole_vec_2 = { 2, 4 };
		}
	}
	//cout << "height_vec.size(): " << height_vec.size() << endl;
	//cout << "width_vec.size(): " << width_vec.size() << endl;

	//std::cout << "check_line_with_break height_vec0: " << height_vec[0] << std::endl;
	//std::cout << "check_line_with_break height_vec1: " << height_vec[1] << std::endl;
	//std::cout << "check_line_with_break width_vec0: " << width_vec[0] << std::endl;
	//std::cout << "check_line_with_break width_vec1: " << width_vec[1] << std::endl;
	group_lines = std::make_pair(height_vec, width_vec);
	group_holes = std::make_pair(hole_vec_1, hole_vec_2);
}

void LineToolHandle::init_params(string plane_seg_output_path, string output_path, string line_cfg_file)
{
	this->plane_seg_output_path = plane_seg_output_path;
	this->output_path = output_path;
	this->line_cfg_file = line_cfg_file;
}

float LineToolHandle::method0(std::vector<int> lines, std::vector<Line3DSegOutItemDebug>& plane_line_out)
{
	return 0.0;
}

float LineToolHandle::method1(
	std::vector<int> lines, 
	std::vector<Line3DSegOutItemDebug>& plane_line_out)
{
	Line3DSegOutItemDebug line0 = plane_line_out[lines[0]];
	Line3DSegOutItemDebug line1 = plane_line_out[lines[1]];
	float cur_diff0_0 = Util_Math::DWComputePointToPointDist<ModuleStruct::Point3f>(line0.line_seg_start, line1.line_seg_start);
	float cur_diff0_1 = Util_Math::DWComputePointToPointDist<ModuleStruct::Point3f>(line0.line_seg_start, line1.line_seg_end);
	float cur_diff1_0 = Util_Math::DWComputePointToPointDist<ModuleStruct::Point3f>(line0.line_seg_end, line1.line_seg_end);
	float cur_diff1_1 = Util_Math::DWComputePointToPointDist<ModuleStruct::Point3f>(line0.line_seg_end, line1.line_seg_start);
	// 防止交叉情况
	float cur_diff0 = cur_diff0_0 < cur_diff0_1 ? cur_diff0_0 : cur_diff0_1;
	float cur_diff1 = cur_diff1_0 < cur_diff1_1 ? cur_diff1_0 : cur_diff1_1;
	float cur_diff2 = Util_Math::DWComputePointToPointDist<ModuleStruct::Point3f>(line0.line_center, line1.line_center);
	float cur_width = (cur_diff0 + cur_diff1 + cur_diff2) / 3;
	return cur_width;
}


static float dist_2lines(Line3DSegOutItemDebug& line0, Line3DSegOutItemDebug& line1)
{
	float dist0 = util_UNRE::dist_point_to_line(line0.line_seg_start, line1.line_seg_start, line1.line_seg_end);
	float dist1 = util_UNRE::dist_point_to_line(line0.line_seg_end, line1.line_seg_start, line1.line_seg_end);
	float dist2 = util_UNRE::dist_point_to_line(line0.line_center, line1.line_seg_start, line1.line_seg_end);

	float dist3 = util_UNRE::dist_point_to_line(line1.line_seg_start, line0.line_seg_start, line0.line_seg_end);
	float dist4 = util_UNRE::dist_point_to_line(line1.line_seg_end, line0.line_seg_start, line0.line_seg_end);
	float dist5 = util_UNRE::dist_point_to_line(line1.line_center, line0.line_seg_start, line0.line_seg_end);
	return (dist0 + dist1 + dist2 + dist3 + dist4 + dist5) / 6;
}

float LineToolHandle::method3(int line_index_0, int line_index_1, 
							  std::vector<Line3DSegOutItemDebug>& plane_line_out)
{
	Line3DSegOutItemDebug line0 = plane_line_out[line_index_0];
	Line3DSegOutItemDebug line1 = plane_line_out[line_index_1];
	float cur_width = dist_2lines(line0, line1);
	return cur_width;
}

inline void rotateAngle_XY(const cv::Point3f& plane_points_rotated, const float& rotated_xz_plane_angle, cv::Point3f& rot_plane_points_to_xz)
{
	rot_plane_points_to_xz.x = plane_points_rotated.x * cos(rotated_xz_plane_angle) - plane_points_rotated.y * sin(rotated_xz_plane_angle);
	rot_plane_points_to_xz.y = plane_points_rotated.x * sin(rotated_xz_plane_angle) + plane_points_rotated.y * cos(rotated_xz_plane_angle);
	rot_plane_points_to_xz.z = plane_points_rotated.z;
}

void RecoverPointRotate(const cv::Mat& matrix, cv::Point3f& point)
{
	cv::Point3f in = point;
	MeasureBase::RotatePoint(in, matrix, point);
}

bool LineToolHandle::IsTwoPlanesConnected(const std::vector<cv::Point3f>& plane1_vertices, 
	                                      const std::vector<cv::Point3f>& plane2_vertices)
{
	int count = 0;

	for (int i = 0; i < plane1_vertices.size(); i++)
	{
		for (int j = 0; j < plane2_vertices.size(); j++)
		{
			if (cv::norm(plane1_vertices[i] - plane2_vertices[j]) < plane_to_plane_distance_thr)///200mm
				count++;
		}
	}
	
	if (count > 0) {

		return true; // find the closed two vertices
	}
	
	return false;
}

int getMaxAreaOfContourIndex(const std::vector<std::vector<cv::Point3f>>& edges, 
							 const float& rotated_xz_plane_angle)
{
	float area = 0;
	int index = 0;
	std::vector < std::vector<cv::Point>> points_2d(edges.size());
#pragma omp parallel for
	for (int i = 0; i < edges.size(); i++)
	{
		points_2d[i].resize(edges[i].size());
		for (int j = 0; j < edges[i].size(); j++)
		{
			cv::Point3f tmp;
			rotateAngle_XY(edges[i][j], rotated_xz_plane_angle, tmp);
			points_2d[i][j].x = tmp.x;
			points_2d[i][j].y = tmp.z;
		}
	}
	for (int i = 0; i < points_2d.size(); i++)
	{
		float cur_area = cv::contourArea(points_2d[i]);
		if (cur_area > area)
		{
			area = cur_area;
			index = i;
		}
	}
	return index;
}

bool overlapSizeCheck(std::vector<std::vector<cv::Point3f>>& current_plane_edges, 
					  const float& rotated_xz_plane_angle,
	                  const cv::Mat& rotation_matrix_adjustment, 
					  const std::pair<cv::Point3f, cv::Point3f>& project_lines_coordinate, bool x_aixs)
{
	float max_value = -std::numeric_limits<float>::infinity(), min_value = std::numeric_limits<float>::infinity();
	for (int m = 0; m < current_plane_edges.size(); m++)
	{
		for (int k = 0; k < current_plane_edges[m].size(); k++)
		{
			cv::Point3f tmp = current_plane_edges[m][k];
			rotateAngle_XY(tmp, rotated_xz_plane_angle, current_plane_edges[m][k]);
			RecoverPointRotate(rotation_matrix_adjustment, current_plane_edges[m][k]);
			if (x_aixs)
			{
				if (current_plane_edges[m][k].x > max_value) 
					max_value = current_plane_edges[m][k].x;
				if (current_plane_edges[m][k].x < min_value) 
					min_value = current_plane_edges[m][k].x;
			}
			else
			{
				if (current_plane_edges[m][k].z > max_value) 
					max_value = current_plane_edges[m][k].z;
				if (current_plane_edges[m][k].z < min_value) 
					min_value = current_plane_edges[m][k].z;
			}
		}
	}
	float measure_line_max_value, measure_line_min_value;
	if (x_aixs)
	{
		measure_line_max_value = project_lines_coordinate.first.x > project_lines_coordinate.second.x ? project_lines_coordinate.first.x : project_lines_coordinate.second.x;
		measure_line_min_value = project_lines_coordinate.first.x < project_lines_coordinate.second.x ? project_lines_coordinate.first.x : project_lines_coordinate.second.x;
	}
	else
	{
		measure_line_max_value = project_lines_coordinate.first.z > project_lines_coordinate.second.z ? project_lines_coordinate.first.z : project_lines_coordinate.second.z;
		measure_line_min_value = project_lines_coordinate.first.z < project_lines_coordinate.second.z ? project_lines_coordinate.first.z : project_lines_coordinate.second.z;
	}
	if (measure_line_max_value < min_value || max_value < measure_line_min_value)
		return false;
	//find p2, p3 p1--------p2---------p3--------p4
	float p2 = min_value > measure_line_min_value ? min_value : measure_line_min_value;
	float p3 = max_value < measure_line_max_value ? max_value : measure_line_max_value;
	if (fabs(p3 - p2) < (measure_line_max_value - measure_line_min_value) * 0.5)
		return false;
	return true;
}

float LineToolHandle::GetBestReferencePlaneDistance(///T
	unsigned int lines, 
	const std::vector<Line3DSegOutItemDebug>& plane_line_out, 
	unsigned int reference_line,
	const std::vector<std::vector<cv::Point3f>>& input_plane_points, 
	const std::vector<cv::Point3f>& input_plane_normal,
	const std::vector<cv::Point3f>& input_plane_center, 
	int measure_index, ///wall id
	int& reference_plane_index)
{
	///reference_plane_index = measure_index;
	//p' = p - (n ⋅ (p - o)) * n -> project point to plane
#define project_point2plane(p, n, o) (p - (n.dot(p - o)) * n)
	if (input_plane_center.size() != input_plane_normal.size()
		|| input_plane_center.size() != input_plane_points.size()
		|| lines < 0
		|| reference_line < 0)///T add
	{
		

		return -1;
	}
	
	reference_plane_index = measure_index;

	//rotate plane to xz-plane 
	std::pair<cv::Point3f, cv::Point3f> project_lines_coordinate;// first ->start, second ->end
	cv::Point3f project_reference_point;
	float rotate_to_xz_normal[3] = { input_plane_normal[measure_index].x, 
		                             input_plane_normal[measure_index].y, 
									 input_plane_normal[measure_index].z };
	float corsspro[3];
	float length_normal_on_xyplane = MathOperation::ComputeTriangleHypotenuse(rotate_to_xz_normal[0], rotate_to_xz_normal[1]);
	MeasureBase::CrossProduct(rotate_to_xz_normal, AXIS_Y_DIRECTION, corsspro);
	float rotated_xz_plane_angle = (corsspro[2] > 0.f) ? acosf(rotate_to_xz_normal[1] / length_normal_on_xyplane) :
		                                                -acosf(rotate_to_xz_normal[1] / length_normal_on_xyplane);
	
	//for project lines
	cv::Point3f start = plane_line_out[lines].line_seg_start;
	cv::Point3f end = plane_line_out[lines].line_seg_end;
	rotateAngle_XY(start, rotated_xz_plane_angle, project_lines_coordinate.first);
	rotateAngle_XY(end, rotated_xz_plane_angle, project_lines_coordinate.second);

	//for referenc point
	rotateAngle_XY(plane_line_out[reference_line].line_center, rotated_xz_plane_angle, project_reference_point);
	
	float plane_normal_rotate_x = input_plane_normal[measure_index].x * cos(rotated_xz_plane_angle) - input_plane_normal[measure_index].y * sin(rotated_xz_plane_angle);
	float plane_normal_rotate_y = input_plane_normal[measure_index].x * sin(rotated_xz_plane_angle) + input_plane_normal[measure_index].y * cos(rotated_xz_plane_angle);
	float plane_normal_rotate[3] = { plane_normal_rotate_x, plane_normal_rotate_y, input_plane_normal[measure_index].z };
	//cout << "plane_normal_rotate: " << plane_normal_rotate[0] << " " << plane_normal_rotate[1] << " " << plane_normal_rotate[2] << endl;
	// rotate normal to Y
	cv::Mat rotation_matrix_adjustment = MeasureBase::CalRotationMatrixFromVectors(plane_normal_rotate, AXIS_Y_DIRECTION);
	//for project lines
	RecoverPointRotate(rotation_matrix_adjustment, project_lines_coordinate.first);
	RecoverPointRotate(rotation_matrix_adjustment, project_lines_coordinate.second);
	//for referenc point
	RecoverPointRotate(rotation_matrix_adjustment, project_reference_point);
	//*************************************************************************************************
	
	//if line is vertical or horizontal ******************************************************************
	float line_x_diff = abs(project_lines_coordinate.first.x - project_lines_coordinate.second.x);
	float line_z_diff = abs(project_lines_coordinate.first.z - project_lines_coordinate.second.z);
	bool vertical_line = false;
	if (line_x_diff < line_z_diff)
		vertical_line = true;
	
	std::vector<std::pair<int, cv::Point3f>> project_planes_centerP;
	std::vector<std::vector<cv::Point3f>> measure_plane_edges;
	util_line_in_plane_test::get_plane_edge3d(edge2d_pxl_size,///T 
											  input_plane_points[measure_index], 
											  input_plane_normal[measure_index], 
											  input_plane_center[measure_index], 
											  measure_plane_edges);

	int measure_outside_index = getMaxAreaOfContourIndex(measure_plane_edges, rotated_xz_plane_angle);
	for (int i = 0; i < input_plane_center.size(); i++)
	{
		if (measure_index != i)
		{
#if 0
			//for walls
			remove parallel plane and connected in two planes
			float plane2_normal[3] = { input_plane_normal[i].x, input_plane_normal[i].y, input_plane_normal[i].z };
			if ((abs(input_plane_normal[i].z) > 1 - normal_direction_threshold) &&
				(abs(input_plane_normal[i].z) < 1 + normal_direction_threshold))
			{
				plane2_normal[0] = measure_normal[0];
		 		plane2_normal[1] = measure_normal[1];
			}
			std::vector<cv::Point3f> cur_plane_vertices;
			if (!measure_vertices_c.MeasureFindVerticeFcn(plane2_normal, input_plane_points[i], cur_plane_vertices))
			continue;
#endif
			if (vertical_line)
			{
				if (abs(input_plane_normal[i].z) < normal_direction_threshold &&
					fabs(input_plane_normal[measure_index].dot(input_plane_normal[i])) < min_normal_diff)
				{
					//clock_t t1 = clock();
					

					std::vector<std::vector<cv::Point3f>> current_plane_edges;
					util_line_in_plane_test::get_plane_edge3d(edge2d_pxl_size, 
															  input_plane_points[i], 
															  input_plane_normal[i], 
															  input_plane_center[i], 
															  current_plane_edges);
					if ((measure_outside_index < measure_plane_edges.size()) &&
						(getMaxAreaOfContourIndex(current_plane_edges, rotated_xz_plane_angle) < current_plane_edges.size()))
					{
						if (IsTwoPlanesConnected(measure_plane_edges[measure_outside_index],
							                     current_plane_edges[getMaxAreaOfContourIndex(current_plane_edges, rotated_xz_plane_angle)]))
						{	
							if (overlapSizeCheck(current_plane_edges,
												 rotated_xz_plane_angle,
												 rotation_matrix_adjustment,
												 project_lines_coordinate, false))
							{
								

								project_planes_centerP.push_back(std::make_pair(i,
									//#define project_point2plane(p, n, o) (p - (n.dot(p - o)) * n)
									project_point2plane(input_plane_center[i],input_plane_normal[measure_index],input_plane_center[measure_index])));
							}
						}
					}	
					//std::cout << "plane: " << i << ", elapsed time: " << (clock() - t1) / float(CLOCKS_PER_SEC) << std::endl;
				}
			} 
			else if ((abs(input_plane_normal[i].z) > 1 - normal_direction_threshold) && //for ground or ceil
				     (abs(input_plane_normal[i].z) < 1 + normal_direction_threshold))
			{
				//0 is outside contour edges points		
				std::vector<std::vector<cv::Point3f>> current_plane_edges;
				util_line_in_plane_test::get_plane_edge3d(edge2d_pxl_size, 
														  input_plane_points[i], 
														  input_plane_normal[i], 
														  input_plane_center[i], 
														  current_plane_edges);		
				if ((measure_outside_index < measure_plane_edges.size()) &&
					(getMaxAreaOfContourIndex(current_plane_edges, rotated_xz_plane_angle) < current_plane_edges.size()))
				{
					if (IsTwoPlanesConnected(measure_plane_edges[measure_outside_index],
						                     current_plane_edges[getMaxAreaOfContourIndex(current_plane_edges, rotated_xz_plane_angle)]))
					{	
						if (overlapSizeCheck(current_plane_edges,
							rotated_xz_plane_angle,
							rotation_matrix_adjustment,
							project_lines_coordinate, true))
						{
							project_planes_centerP.push_back(std::make_pair(i,
								project_point2plane(input_plane_center[i],
									input_plane_normal[measure_index],
									input_plane_center[measure_index])));
						}
					}
				}
			}
		}
	}
	//for project center points rotate to xz-plane
	

#pragma omp parallel for
	for (int i = 0; i < project_planes_centerP.size(); i++)
	{
		cv::Point3f tmp = project_planes_centerP[i].second;
		rotateAngle_XY(tmp, rotated_xz_plane_angle, project_planes_centerP[i].second);
	}
#pragma omp parallel for
	for (int i = 0; i < project_planes_centerP.size(); i++)
	{
		RecoverPointRotate(rotation_matrix_adjustment, project_planes_centerP[i].second);
	}
	//**************************************************************************************************

	//find candidates ***********************************************************************************
	if (vertical_line)
	{
		float line_x = (project_lines_coordinate.first.x + project_lines_coordinate.second.x) * 0.5;
		if (project_reference_point.x > line_x)
		{
			for (std::vector<std::pair<int, cv::Point3f>>::iterator iter = project_planes_centerP.begin(); iter != project_planes_centerP.end();)
			{
				if (iter->second.x < project_reference_point.x)
				{
					iter = project_planes_centerP.erase(iter);
				}
				else
					iter++;
			}
		}
		else
		{
			for (std::vector<std::pair<int, cv::Point3f>>::iterator iter = project_planes_centerP.begin(); iter != project_planes_centerP.end();)
			{
				if (iter->second.x > project_reference_point.x)
				{
					iter = project_planes_centerP.erase(iter);
				}
				else
					iter++;
			}
		}

	}
	else
	{
		float line_z = (project_lines_coordinate.first.z + project_lines_coordinate.second.z) * 0.5;
		if (project_reference_point.z > line_z)
		{
			for (std::vector<std::pair<int, cv::Point3f>>::iterator iter = project_planes_centerP.begin(); iter != project_planes_centerP.end();)
			{
				if (iter->second.z < project_reference_point.z)
				{
					iter = project_planes_centerP.erase(iter);
				}
				else
					iter++;
			}
		}
		else
		{
			for (std::vector<std::pair<int, cv::Point3f>>::iterator iter = project_planes_centerP.begin(); iter != project_planes_centerP.end();)
			{
				if (iter->second.z > project_reference_point.z)
				{
					iter = project_planes_centerP.erase(iter);
				}
				else
					iter++;
			}
		}
	}
	//get nearest plane index and calculate line to plane distance, sort by map
	std::map<float, int> final_planes; ///dis, pId
	for (int i = 0; i < project_planes_centerP.size(); i++)
	{
		float dist = LineToPlaneDistance(input_plane_center[project_planes_centerP[i].first], 
										 input_plane_normal[project_planes_centerP[i].first],
										 plane_line_out[lines]
										);
		final_planes[dist] = project_planes_centerP[i].first;///pId
	}	
	if (final_planes.size() > 0)
	{
		//std::cout << "find reference plane index " << final_planes.begin()->second << std::endl;
		reference_plane_index = final_planes.begin()->second;
		return final_planes.begin()->first;
	}
	return -1;
}

float LineToolHandle::GetReferencePlaneForLine(
	unsigned int lines,
	const std::vector<Line3DSegOutItemDebug>& plane_line_out,
	const std::vector<cv::Point3f>& ref_pts,
	const std::vector<std::vector<cv::Point3f>>& input_plane_points,
	const std::vector<cv::Point3f>& input_plane_normal,
	const std::vector<cv::Point3f>& input_plane_center,
	int measure_index, ///wall id
	int& reference_plane_index)
{
	///reference_plane_index = measure_index;
	//p' = p - (n ⋅ (p - o)) * n -> project point to plane
#define project_point2plane(p, n, o) (p - (n.dot(p - o)) * n)
	if (input_plane_center.size() != input_plane_normal.size()
		|| input_plane_center.size() != input_plane_points.size()
		|| lines < 0 ){
		return -1;
	}
	//rotate plane to xz-plane 
	std::pair<cv::Point3f, cv::Point3f> project_lines_coordinate;// first ->start, second ->end
	cv::Point3f project_reference_point;
	float rotate_to_xz_normal[3] = { input_plane_normal[measure_index].x,
									 input_plane_normal[measure_index].y,
									 input_plane_normal[measure_index].z };
	float corsspro[3];
	float length_normal_on_xyplane = MathOperation::ComputeTriangleHypotenuse(rotate_to_xz_normal[0], rotate_to_xz_normal[1]);
	MeasureBase::CrossProduct(rotate_to_xz_normal, AXIS_Y_DIRECTION, corsspro);
	float rotated_xz_plane_angle = (corsspro[2] > 0.f) ? acosf(rotate_to_xz_normal[1] / length_normal_on_xyplane) :
		-acosf(rotate_to_xz_normal[1] / length_normal_on_xyplane);

	//for project lines
	cv::Point3f start = ref_pts[0];// plane_line_out[lines].line_seg_start;
	cv::Point3f end = ref_pts[1];// plane_line_out[lines].line_seg_end;
	rotateAngle_XY(start, rotated_xz_plane_angle, project_lines_coordinate.first);
	rotateAngle_XY(end, rotated_xz_plane_angle, project_lines_coordinate.second);
	//for referenc point
	rotateAngle_XY(plane_line_out[lines].line_center, rotated_xz_plane_angle, project_reference_point);
	float plane_normal_rotate_x = input_plane_normal[measure_index].x * cos(rotated_xz_plane_angle) - input_plane_normal[measure_index].y * sin(rotated_xz_plane_angle);
	float plane_normal_rotate_y = input_plane_normal[measure_index].x * sin(rotated_xz_plane_angle) + input_plane_normal[measure_index].y * cos(rotated_xz_plane_angle);
	float plane_normal_rotate[3] = { plane_normal_rotate_x, plane_normal_rotate_y, input_plane_normal[measure_index].z };
	cout << "plane_normal_rotate: " << plane_normal_rotate[0] << " " << plane_normal_rotate[1] << " " << plane_normal_rotate[2] << endl;
	// rotate normal to Y
	cv::Mat rotation_matrix_adjustment = MeasureBase::CalRotationMatrixFromVectors(plane_normal_rotate, AXIS_Y_DIRECTION);
	//for project lines
	RecoverPointRotate(rotation_matrix_adjustment, project_lines_coordinate.first);
	RecoverPointRotate(rotation_matrix_adjustment, project_lines_coordinate.second);
	//for referenc point
	RecoverPointRotate(rotation_matrix_adjustment, project_reference_point);

	//*************************************************************************************************

	//if line is vertical or horizontal ******************************************************************
	float line_x_diff = abs(project_lines_coordinate.first.x - project_lines_coordinate.second.x);
	float line_z_diff = abs(project_lines_coordinate.first.z - project_lines_coordinate.second.z);
	bool vertical_line = false;
	if (line_x_diff < line_z_diff)
		vertical_line = true;

	std::vector<std::pair<int, cv::Point3f>> project_planes_centerP;
	std::vector<std::vector<cv::Point3f>> measure_plane_edges;
	util_line_in_plane_test::get_plane_edge3d(edge2d_pxl_size,///T 
		input_plane_points[measure_index],
		input_plane_normal[measure_index],
		input_plane_center[measure_index],
		measure_plane_edges);

	int measure_outside_index = getMaxAreaOfContourIndex(measure_plane_edges, rotated_xz_plane_angle);
	for (int i = 0; i < input_plane_center.size(); i++)
	{
		if (measure_index != i)
		{
			if (vertical_line)// valid line direction
			{
				if (abs(input_plane_normal[i].z) < normal_direction_threshold &&
					fabs(input_plane_normal[measure_index].dot(input_plane_normal[i])) < min_normal_diff)
				{

					std::vector<std::vector<cv::Point3f>> current_plane_edges;
					util_line_in_plane_test::get_plane_edge3d(edge2d_pxl_size,
						input_plane_points[i],
						input_plane_normal[i],
						input_plane_center[i],
						current_plane_edges);

					if ((measure_outside_index < measure_plane_edges.size()) &&
						(getMaxAreaOfContourIndex(current_plane_edges, rotated_xz_plane_angle) < current_plane_edges.size()))
					{
						if (IsTwoPlanesConnected(measure_plane_edges[measure_outside_index],
							current_plane_edges[getMaxAreaOfContourIndex(current_plane_edges, rotated_xz_plane_angle)]))
						{
							if (overlapSizeCheck(current_plane_edges,
								rotated_xz_plane_angle,
								rotation_matrix_adjustment,
								project_lines_coordinate, false))
							{
								project_planes_centerP.push_back(std::make_pair(i,
									//#define project_point2plane(p, n, o) (p - (n.dot(p - o)) * n)
									project_point2plane(input_plane_center[i], input_plane_normal[measure_index], input_plane_center[measure_index])));
							}
						}
					}
					//std::cout << "plane: " << i << ", elapsed time: " << (clock() - t1) / float(CLOCKS_PER_SEC) << std::endl;
				}
			}
			else if ((abs(input_plane_normal[i].z) > 1 - normal_direction_threshold) && //for ground or ceil
				(abs(input_plane_normal[i].z) < 1 + normal_direction_threshold))
			{
				//0 is outside contour edges points
				std::vector<std::vector<cv::Point3f>> current_plane_edges;
				util_line_in_plane_test::get_plane_edge3d(edge2d_pxl_size,
					input_plane_points[i],
					input_plane_normal[i],
					input_plane_center[i],
					current_plane_edges);
				if ((measure_outside_index < measure_plane_edges.size()) &&
					(getMaxAreaOfContourIndex(current_plane_edges, rotated_xz_plane_angle) < current_plane_edges.size()))
				{
					if (IsTwoPlanesConnected(measure_plane_edges[measure_outside_index],
						current_plane_edges[getMaxAreaOfContourIndex(current_plane_edges, rotated_xz_plane_angle)]))
					{
						if (overlapSizeCheck(current_plane_edges,
							rotated_xz_plane_angle,
							rotation_matrix_adjustment,
							project_lines_coordinate, true))
						{
							project_planes_centerP.push_back(std::make_pair(i,
								project_point2plane(input_plane_center[i],
									input_plane_normal[measure_index],
									input_plane_center[measure_index])));
						}
					}
				}
			}
		}
	}
	//for project center points rotate to xz-plane
#pragma omp parallel for
	for (int i = 0; i < project_planes_centerP.size(); i++)
	{
		cv::Point3f tmp = project_planes_centerP[i].second;
		rotateAngle_XY(tmp, rotated_xz_plane_angle, project_planes_centerP[i].second);
	}
#pragma omp parallel for
	for (int i = 0; i < project_planes_centerP.size(); i++)
	{
		RecoverPointRotate(rotation_matrix_adjustment, project_planes_centerP[i].second);
	}
	//find candidates 
	if (vertical_line)
	{
		float line_x = (project_lines_coordinate.first.x + project_lines_coordinate.second.x) * 0.5;
		if (project_reference_point.x > line_x)
		{
			for (std::vector<std::pair<int, cv::Point3f>>::iterator iter = project_planes_centerP.begin(); iter != project_planes_centerP.end();)
			{
				if (iter->second.x < project_reference_point.x)
				{
					iter = project_planes_centerP.erase(iter);
				}
				else
					iter++;
			}
		}
		else
		{
			for (std::vector<std::pair<int, cv::Point3f>>::iterator iter = project_planes_centerP.begin(); iter != project_planes_centerP.end();)
			{
				if (iter->second.x > project_reference_point.x)
				{
					iter = project_planes_centerP.erase(iter);
				}
				else
					iter++;
			}
		}
	}
	else
	{
		float line_z = (project_lines_coordinate.first.z + project_lines_coordinate.second.z) * 0.5;
		if (project_reference_point.z > line_z)
		{
			for (std::vector<std::pair<int, cv::Point3f>>::iterator iter = project_planes_centerP.begin(); iter != project_planes_centerP.end();)
			{
				if (iter->second.z < project_reference_point.z)
				{
					iter = project_planes_centerP.erase(iter);
				}
				else
					iter++;
			}
		}
		else
		{
			for (std::vector<std::pair<int, cv::Point3f>>::iterator iter = project_planes_centerP.begin(); iter != project_planes_centerP.end();)
			{
				if (iter->second.z > project_reference_point.z)
				{
					iter = project_planes_centerP.erase(iter);
				}
				else
					iter++;
			}
		}
	}

	//get nearest plane index and calculate line to plane distance, sort by map
	std::map<float, int> final_planes; ///dis, pId
	for (int i = 0; i < project_planes_centerP.size(); i++)
	{
		float dist = LineToPlaneDistance(input_plane_center[project_planes_centerP[i].first],
										 input_plane_normal[project_planes_centerP[i].first],
										  ref_pts// plane_line_out[lines]
										 );
		final_planes[dist] = project_planes_centerP[i].first;///pId
	}

	if (final_planes.size() > 0)
	{
		//std::cout << "find reference plane index " << final_planes.begin()->second << std::endl;
		reference_plane_index = final_planes.begin()->second;
		return final_planes.begin()->first;
	}
	return -1;
}

float LineToolHandle::LineToPlaneDistance(const cv::Point3f& plane_center, const cv::Point3f& plane_normal, const Line3DSegOutItemDebug& line)
{
	float dist0 = Util_Math::DWComputePointToPlaneDist<cv::Point3f>(line.line_seg_start, plane_normal, plane_center);
	float dist1 = Util_Math::DWComputePointToPlaneDist<cv::Point3f>(line.line_seg_end, plane_normal, plane_center);
	float dist2 = Util_Math::DWComputePointToPlaneDist<cv::Point3f>(line.line_center, plane_normal, plane_center);
	return (dist0 + dist1 + dist2) / 3;
}

float LineToolHandle::LineToPlaneDistance(const cv::Point3f& plane_center, const cv::Point3f& plane_normal, const  std::vector<cv::Point3f>& pts)
{
	float dist0 = Util_Math::DWComputePointToPlaneDist<cv::Point3f>(pts[0], plane_normal, plane_center);
	float dist1 = Util_Math::DWComputePointToPlaneDist<cv::Point3f>(pts[1], plane_normal, plane_center);
	float dist2 = Util_Math::DWComputePointToPlaneDist<cv::Point3f>((pts[0]+ pts[1])/2, plane_normal, plane_center);
	return (dist0 + dist1 + dist2) / 3;
}

void LineToolHandle::calibrate(
				std::vector<std::pair<float, float>>& result, 
				std::vector<std::pair<int, int>>& group_method, 
				std::vector<int> measurement_type, 
				int measure_index, 
				float plane_mse)
{
	assert(result.size() == measurement_type.size());
	for (int i = 0; i < result.size(); i++) {
		float calibrate_val = 0.0;
		if (measurement_type[i] == 0) { // window
			calibrate_val = plane_mse * window_calibrate_const;
		}
		else { // door
			calibrate_val = plane_mse * door_calibrate_const;
		}
	//	std::cout << "hole " << i << ", width: " << result[i].first << std::endl;
	//	std::cout << "hole " << i << ", height: " << result[i].second << std::endl;
		// only method 1 need 
		//if (group_method[i].first == 1) {
		if (result[i].first > 0) {
			result[i].first += calibrate_val;
		}
		if (result[i].second > 0) {
			result[i].second += calibrate_val;
		}
		//}
		//if (group_method[i].second == 1) {
		
		//}
	//	std::cout << "calibrate value: " << calibrate_val << std::endl;
	//	std::cout << "hole " << i << ", calibrated width: " << result[i].first << std::endl;
	//	std::cout << "hole " << i << ", calibrated height: " << result[i].second << std::endl;
	}
}

bool LineToolHandle::get_lines_plane_mse(std::string output_path, std::vector<LinePlaneMse>& line_plane_mse_arr)
{
	std::string filename = output_path + "lines_plane_mse.txt";
	FILE* pFile;
#ifdef WIN32
	errno_t err = fopen_s(&pFile, filename.c_str(), "r");
#else
	pFile = fopen(filename.c_str(), "r");
#endif

	if (!pFile)
	{
		cout << "IOData::get_lines_plane_mse: cannot open file" << endl;
		log_fatal("cannot open %s", filename.c_str());
		return false;
	}

	int raw_data_row_num = 0;
	IOData::Get3DPtCloudRowNum(filename, raw_data_row_num);
	int row_idx = 0;
	unsigned int line_idx;
	line_plane_mse_arr.clear();
	line_plane_mse_arr.resize(raw_data_row_num);
	while (!feof(pFile) && row_idx < raw_data_row_num)
	{
		if (FSCANF(pFile, "%d%d%f%*[^\n]%*c", &line_idx, &line_plane_mse_arr[row_idx].plane_idx, &\
			line_plane_mse_arr[row_idx].plane_mse) == 3)
		{
			row_idx++;
		}
	}
	fclose(pFile);
	return true;
}

