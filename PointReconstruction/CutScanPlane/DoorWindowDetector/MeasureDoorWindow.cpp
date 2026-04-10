#include "MeasureDoorWindow.h"
#include "util_math.hpp"
#include "MathOperation.hpp"
#include "../Measurement/MeasureBase.h"
#include "bbox.h"
#include "log.h"
#define RULER_VERTICE_SIZE	4
#define PLANE_MIN_MAX_SIZE	6
/*static const cv::Point3f AXIS_X_DIRECTION = { 1.f,0.f,0.f };
static const cv::Point3f AXIS_Y_DIRECTION = { 0.f,1.f,0.f };
static const cv::Point3f AXIS_Z_DIRECTION = { 0.f,0.f,1.f };
*/
extern std::ofstream resultA;
float m_thres_offset_isSamePlane = 60.f;
bool MeasureDoorWindow::DoorWindowDetector(const Vector<Vector<cv::Point3f>>& pts, 
	const Vector<cv::Point3f>& normals,
	const Vector<int>& detect_plane_idx,
	float walltopZ, 
	Vector<Vector<DoorWindowInfo>>& door_window_info)
{
	DetectionHoleClass detect_hole;
	MeasureDoorWindow::Params_DetcHole detect_hole_params;
	detect_hole.SetWidthRule(detect_hole_params.width_ruler);
	detect_hole.SetThresMinLenHole(detect_hole_params.thres_minLen_hole);
	detect_hole.SetThresMaxLenHole(detect_hole_params.thres_maxLen_hole);
	detect_hole.SetThresMinWidthDoor(detect_hole_params.thres_minWidth_door);
	detect_hole.SetThresMaxWidthDoor(detect_hole_params.thres_maxWidth_door);
	detect_hole.SetThresMinHeightDoor(detect_hole_params.thres_minHeight_door);
	detect_hole.SetThresMaxHeightDoor(detect_hole_params.thres_maxHeight_door);
	detect_hole.SetSampleCstrLen(detect_hole_params.sample_cstr_len);
	detect_hole.SetSampleMargin(detect_hole_params.sample_margin);

	Vector<Vector<Vector<cv::Point3f>>> corners_hole(pts.size());
	Vector<Vector<std::array<float, 2>>>raw_w_h(pts.size());
	Vector<Vector<std::array<float, 4>>> hole_w_h(pts.size());
	Vector<Vector<int>> type_hole(pts.size());

	door_window_info.resize(pts.size());

	//added by yu.liang 2021/1/13 start

	//for (int i = 0; i < pts.size(); i++)
	for (auto i : detect_plane_idx)
	{
		std::vector<cv::Point3f> corners_plane(4);

		if (!MeasureFindVerticeFcn(normals[i], pts[i], corners_plane))
			continue;

		float plane_normal[3] = { normals[i].x ,normals[i].y ,normals[i].z };
		door_window_info[i].clear();
		detect_hole.DetectionHoleClassFcn(	i,
			pts[i], plane_normal, corners_plane, walltopZ, corners_hole[i],
			raw_w_h[i], type_hole[i], true, hole_w_h[i]);
	}
	//door_window_info.clear();
	//
	std::vector<std::vector<cv::Point3f>> window_pos;
	std::vector<int> plane_with_window_idx;
	for (auto i = 0; i < type_hole.size(); i++)
	{
		for (auto j = 0; j < type_hole[i].size(); j++)
		{
			door_window_info[i].push_back({ type_hole[i][j], corners_hole[i][j], i });
		}
	}
	return true;
}

bool MeasureDoorWindow::MeasureFindVerticeFcn(const cv::Point3f plane_normal, const std::vector<cv::Point3f> &plane_points,
	std::vector<cv::Point3f> &real_vertices)
{
	if (plane_points.empty())
	{
		log_warn("err10-1: empty input");

		return false;
	}
	real_vertices.resize(RULER_VERTICE_SIZE);

	float normal_zero_dir_threshold = 0.3f;
	int normal_axis = -1;
	float length_plane_threshold = 1.f;
	float ang_value;
	bool ifRotate = false;

	//rotated points
	std::vector<cv::Point3f> rot_plane_points(plane_points.size());

	if (Util_Math::IsValZero(plane_normal.x))
	{
		normal_axis = 1;
		ang_value = 0.f;
		rot_plane_points = plane_points;
	}
	else if (Util_Math::IsValZero(plane_normal.y))
	{
		normal_axis = 0;
		ang_value = 0.f;
		rot_plane_points = plane_points;
	}
	else //rotate around Z-axis  & project to XZ-plane
	{
		ifRotate = true;
		normal_axis = 1;
		cv::Point3f corsspro;
		corsspro = Util_Math::ComputeVectorCrossProduct(plane_normal, cv::Point3f(0,1.0,0));
		float length_normal_on_xyplane = MathOperation::ComputeTriangleHypotenuse(plane_normal.x, plane_normal.y);
		ang_value = (corsspro.z > 0.f) ? acosf(plane_normal.y / length_normal_on_xyplane) : -acosf(plane_normal.y / length_normal_on_xyplane);
		for (int i = 0; i < plane_points.size(); i++)
		{
			rot_plane_points[i].x = plane_points[i].x* cos(ang_value) - plane_points[i].y*sin(ang_value);
			rot_plane_points[i].y = plane_points[i].x* sin(ang_value) + plane_points[i].y*cos(ang_value);
			rot_plane_points[i].z = plane_points[i].z;
		}
	}

	// top or bottom plane
	if (std::abs(plane_normal.z) > (1.f - normal_zero_dir_threshold))
	{
		normal_axis = 2;
		if (max(std::abs(plane_normal.x), std::abs(plane_normal.y)) < (1 - normal_zero_dir_threshold))
		{
			/*std::cout << "error/warning: Fail to inpute available normal for ceiling/ground roattaion!" << std::endl;
			std::cout << "      Note that normal=[wall_normal[0],  wall_normal[1],  ceiling/ground_normal[2] ]!" << std::endl;*/
			log_warn("error/warning: Fail to find availble walls for ceiling/ground roattaion! Note that normal=[wall_normal[0],  wall_normal[1],  ceiling/ground_normal[2] ]!");
			return false;
		}
	}
	//wall
	else if (std::abs(plane_normal.z) <= normal_zero_dir_threshold)
		normal_axis = normal_axis;
	else {
		//throw err_message = "err10-1: no well-fitted plane within a threshold for normal";
		std::cout << "error/warning 10-1: no well-fitted plane within a threshold for normal" << std::endl;
		log_warn("error/warning 10-1: no well-fitted plane within a threshold for normal");

		return false;
	}

	float plane_minmax_xyz[PLANE_MIN_MAX_SIZE];
	MathOperation::FindPlaneMinMaxXYZ(rot_plane_points, normal_axis, length_plane_threshold, length_plane_threshold, plane_minmax_xyz);

	MeasureDoorWindow::FindVertice(normal_axis, plane_normal, ifRotate, ang_value, plane_minmax_xyz, real_vertices);
	for (auto& p : real_vertices)
	{
		if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
		{
			std::cerr << "[UB DETECT] invalid corner\n";
			return false;
		}
	}

	return true;
}

//Find virtual vertices for all planes
void MeasureDoorWindow::FindVertice(const int normal_axis,
	const cv::Point3f plane_normal,
	const bool ifRotate,
	const float ang_value,
	const float* data_minmax_xyz,
	std::vector<cv::Point3f>& vertice)
{
	if (normal_axis == 1 || normal_axis == 0)
	{
		float real_vertice[RULER_VERTICE_SIZE];

		if (ifRotate)
		{
			vertice[0].x = data_minmax_xyz[0] * cos(ang_value) + data_minmax_xyz[2] * sin(ang_value);
			vertice[0].y = -data_minmax_xyz[0] * sin(ang_value) + data_minmax_xyz[2] * cos(ang_value);

			vertice[1].x = data_minmax_xyz[1] * cos(ang_value) + data_minmax_xyz[2] * sin(ang_value);
			vertice[1].y = -data_minmax_xyz[1] * sin(ang_value) + data_minmax_xyz[2] * cos(ang_value);

			vertice[2].x = data_minmax_xyz[1] * cos(ang_value) + data_minmax_xyz[3] * sin(ang_value);
			vertice[2].y = -data_minmax_xyz[1] * sin(ang_value) + data_minmax_xyz[3] * cos(ang_value);

			vertice[3].x = data_minmax_xyz[0] * cos(ang_value) + data_minmax_xyz[3] * sin(ang_value);
			vertice[3].y = -data_minmax_xyz[0] * sin(ang_value) + data_minmax_xyz[3] * cos(ang_value);
		}
		else
		{
			int vertice_indx[4];
			FindVerticeXY(data_minmax_xyz, plane_normal, vertice_indx);
			real_vertice[0] = data_minmax_xyz[vertice_indx[0]];
			real_vertice[1] = data_minmax_xyz[vertice_indx[1]];
			real_vertice[2] = data_minmax_xyz[vertice_indx[2]];
			real_vertice[3] = data_minmax_xyz[vertice_indx[3]];

			vertice[0].x = real_vertice[0];
			vertice[0].y = real_vertice[2];

			vertice[1].x = real_vertice[1];
			vertice[1].y = real_vertice[2];

			vertice[2].x = real_vertice[1];
			vertice[2].y = real_vertice[3];

			vertice[3].x = real_vertice[0];
			vertice[3].y = real_vertice[3];
		}
		vertice[0].z = data_minmax_xyz[4];
		vertice[1].z = data_minmax_xyz[4];
		vertice[2].z = data_minmax_xyz[5];
		vertice[3].z = data_minmax_xyz[5];
	}
	else
	{
		if (ifRotate)
		{
			vertice[0].x = data_minmax_xyz[0] * cos(ang_value) + data_minmax_xyz[2] * sin(ang_value);
			vertice[0].y = -data_minmax_xyz[0] * sin(ang_value) + data_minmax_xyz[2] * cos(ang_value);

			vertice[1].x = data_minmax_xyz[1] * cos(ang_value) + data_minmax_xyz[2] * sin(ang_value);
			vertice[1].y = -data_minmax_xyz[1] * sin(ang_value) + data_minmax_xyz[2] * cos(ang_value);

			vertice[2].x = data_minmax_xyz[1] * cos(ang_value) + data_minmax_xyz[3] * sin(ang_value);
			vertice[2].y = -data_minmax_xyz[1] * sin(ang_value) + data_minmax_xyz[3] * cos(ang_value);

			vertice[3].x = data_minmax_xyz[0] * cos(ang_value) + data_minmax_xyz[3] * sin(ang_value);
			vertice[3].y = -data_minmax_xyz[0] * sin(ang_value) + data_minmax_xyz[3] * cos(ang_value);
		}
		else
		{
			vertice[0].x = data_minmax_xyz[0]; vertice[0].y = data_minmax_xyz[2];
			vertice[1].x = data_minmax_xyz[1]; vertice[1].y = data_minmax_xyz[2];
			vertice[2].x = data_minmax_xyz[1]; vertice[2].y = data_minmax_xyz[3];
			vertice[3].x = data_minmax_xyz[0]; vertice[3].y = data_minmax_xyz[3];
		}

		vertice[0].z = (data_minmax_xyz[4] + data_minmax_xyz[5]) * 0.5f;
		vertice[1].z = vertice[0].z;
		vertice[3].z = vertice[0].z;
		vertice[2].z = vertice[0].z;
	}
}

void MeasureDoorWindow::FindVerticeXY(const float* data_minmax_xyz, const cv::Point3f plane_normal, int* vertice_indx)
{
	// find two vertices (x1,y1) & (x2,y2) of two planes on x-y plane, vertice_indx=[x1,x2,y1,y2]
	float normal_zero_dir_threshold = 0.3f;
	for (int i = 0; i < 4; i++)
		vertice_indx[i] = i;

	if (abs(plane_normal.x) > normal_zero_dir_threshold  //plane not parallels to x-axis
		&& abs(plane_normal.y) > normal_zero_dir_threshold)   //plane not parallels to y-axis
	{
		float temp_vector[2];
		temp_vector[0] = data_minmax_xyz[1] - data_minmax_xyz[0];
		temp_vector[1] = data_minmax_xyz[2] - data_minmax_xyz[3];

		float length = std::sqrt(std::pow(temp_vector[0], 2.f) + std::pow(temp_vector[1], 2.f));

		// line between two vertices is vertical to normal
		if (abs(temp_vector[0] * plane_normal.x + temp_vector[1] * plane_normal.y) / length < normal_zero_dir_threshold)
		{
			vertice_indx[2] = 3;
			vertice_indx[3] = 2;
		}
	}
}

bool MeasureDoorWindow::FindWindowCornerMinMax(
	const Vector<Vector<MeasureDoorWindow::DoorWindowInfo>>& door_window_info,
	const Vector<cv::Point3f>& scene_plane_normals,
	const Vector<cv::Point3f>& scene_plane_center,
	Vector<Vector<float>>& window_corner_minmax_xyz)
{
	float length_ext_band = 30.f;
	float length_ext_window = 300.f;

	for (int i = 0; i < door_window_info.size(); i++)
	{
		if (door_window_info[i].size() == 0) continue;
		for (size_t j = 0; j < door_window_info[i].size(); j++)
		{
			auto corners = door_window_info[i][j].corners;
			float min_x = min(min(corners[0].x, corners[1].x), min(corners[2].x, corners[3].x));
			float max_x = max(max(corners[0].x, corners[1].x), max(corners[2].x, corners[3].x));

			float min_y = min(min(corners[0].y, corners[1].y), min(corners[2].y, corners[3].y));
			float max_y = max(max(corners[0].y, corners[1].y), max(corners[2].y, corners[3].y));

			float min_z = min(corners[0].z, corners[1].z) - length_ext_band;
			float max_z = max(corners[2].z, corners[3].z) + length_ext_band;

			if (abs(scene_plane_normals[door_window_info[i][j].wallId].y) > 0.75f)
			{
				min_x -= length_ext_band;
				max_x += length_ext_band;

				if (scene_plane_center[door_window_info[i][j].wallId].y > 0.f)
				{
					min_y -= length_ext_band;
					max_y += length_ext_window;
				}
				else
				{
					min_y -= length_ext_window;
					max_y += length_ext_band;
				}

			}
			else if (abs(scene_plane_normals[door_window_info[i][j].wallId].x) > 0.75f)
			{
				min_y -= length_ext_band;
				max_y += length_ext_band;

				if (scene_plane_center[door_window_info[i][j].wallId].x > 0.f)
				{
					min_x -= length_ext_band;
					max_x += length_ext_window;
				}
				else
				{
					min_x -= length_ext_window;
					max_x += length_ext_band;
				}
			}

			window_corner_minmax_xyz.push_back({ min_x, max_x, min_y, max_y, min_z, max_z });
		}
	}
	return true;
}

bool MeasureDoorWindow::FindWindowCornerMinMax(
	const MeasureDoorWindow::DoorWindowInfo& door_window_info,
	const Vector<cv::Point3f>& scene_plane_normals,
	const Vector<cv::Point3f>& scene_plane_center,
	Vector<Vector<float>>& window_corner_minmax_xyz)
{
	float length_ext_band = 30.f;
	float length_ext_window = 300.f;
	
	auto corners = door_window_info.corners;
	float min_x = min(min(corners[0].x, corners[1].x), min(corners[2].x, corners[3].x));
	float max_x = max(max(corners[0].x, corners[1].x), max(corners[2].x, corners[3].x));

	float min_y = min(min(corners[0].y, corners[1].y), min(corners[2].y, corners[3].y));
	float max_y = max(max(corners[0].y, corners[1].y), max(corners[2].y, corners[3].y));

	float min_z = min(corners[0].z, corners[1].z) - length_ext_band;
	float max_z = max(corners[2].z, corners[3].z) + length_ext_band;

	if (abs(scene_plane_normals[door_window_info.wallId].y) > 0.75f)
	{
		min_x -= length_ext_band;
		max_x += length_ext_band;

		if (scene_plane_center[door_window_info.wallId].y > 0.f)
		{
			min_y -= length_ext_band;
			max_y += length_ext_window;
		}
		else
		{
			min_y -= length_ext_window;
			max_y += length_ext_band;
		}

	}
	else if (abs(scene_plane_normals[door_window_info.wallId].x) > 0.75f)
	{
		min_y -= length_ext_band;
		max_y += length_ext_band;

		if (scene_plane_center[door_window_info.wallId].x > 0.f)
		{
			min_x -= length_ext_band;
			max_x += length_ext_window;
		}
		else
		{
			min_x -= length_ext_window;
			max_x += length_ext_band;
		}
	}

	window_corner_minmax_xyz.push_back({ min_x, max_x, min_y, max_y, min_z, max_z });

	return true;
}

bool MeasureDoorWindow::MergePlanes(Vector<Vector<cv::Point3f>>& pts_planes, std::vector<std::vector<MeasureDoorWindow::DoorWindowInfo>> &door_window_info, std::vector<cv::Point3f> &mid_planes, std::vector<cv::Point3f> &norm_planes, Vector<Vector<unsigned char>>& scene_plane_reflect,
	std::vector<std::pair<int, std::vector<float>>> & wall_list)
{
	for (auto &normal : norm_planes)
		normal = Util_Math::vec3_normalize(normal);

	std::vector<int>  validplane;
	for (int i = 0; i < pts_planes.size(); i++)
		if (pts_planes.size() != 0)
			validplane.push_back(i);

	size_t num_planes = pts_planes.size();
	std::vector<std::vector<cv::Point3f>> merged_pts_planes(num_planes);
	std::vector<std::vector<unsigned char>> merged_reflects_planes(num_planes);
	std::vector<cv::Point3f>  merged_mid_planes(num_planes);
	std::vector<std::vector<cv::Point3f>> raw_corners_plane(num_planes);
	std::vector<std::vector<cv::Point3f>> corners_plane(num_planes);
	std::vector<std::vector<unsigned int>> idx_h_v_plane;
	std::vector<std::vector<std::vector<cv::Point3f>>> corners_hole;

	std::vector<std::vector<unsigned int>> indx_same_plane;
	merged_mid_planes = mid_planes;
	if (LocatePlane_HV(pts_planes, ///T
		door_window_info,
		norm_planes,
		scene_plane_reflect,
		merged_reflects_planes,
		merged_mid_planes,
		merged_pts_planes,
		idx_h_v_plane,
		raw_corners_plane,
		indx_same_plane,
		validplane,
		wall_list) == false
		)
	{
		return false;
	}

	pts_planes.swap(merged_pts_planes);
	scene_plane_reflect.swap(merged_reflects_planes);
}

bool MeasureDoorWindow::LocatePlane_HV(
	const std::vector<std::vector<cv::Point3f>> & pts_planes,
	std::vector<std::vector<MeasureDoorWindow::DoorWindowInfo>> &door_window_info,
	const std::vector<cv::Point3f> & norm_planes,
	const std::vector<std::vector<unsigned char>>& scene_plane_reflect,
	std::vector<std::vector<unsigned char>>& merged_reflects_planes,
	std::vector<cv::Point3f> & mid_planes,
	std::vector<std::vector<cv::Point3f>> & merged_pts_planes,
	std::vector<std::vector<unsigned int>> &idx_h_v_plane,
	std::vector<std::vector<cv::Point3f>> &corners_plane,
	std::vector<std::vector<unsigned int>> &indx_same_plane,
	const std::vector<int>  validplane,
	const std::vector<std::pair<int, std::vector<float>>> &wall_angle_list)
{
	// data validation
	if (pts_planes.size() == 0)
		return false;

	merged_pts_planes.resize(pts_planes.size());
	std::vector<std::vector<unsigned int>> orig_idx_h_v_plane(2);
	idx_h_v_plane.resize(2);
	corners_plane.resize(norm_planes.size());
	unsigned int max_wall_idx;
	float max_wall_size = 0.f;


	//for wall
	for (int i = 0; i < norm_planes.size(); i++)
	{
		if (std::abs(std::abs(norm_planes[i].z) - 1) < 0.3)
			continue;  //skip horizonal plane

		float plane_normal[3];
		corners_plane[i].resize(4);

		//find plane vertices
		plane_normal[0] = norm_planes[i].x;
		plane_normal[1] = norm_planes[i].y;
		plane_normal[2] = norm_planes[i].z;
		if (!MeasureFindVerticeFcn(norm_planes[i], pts_planes[i], corners_plane[i]))
			continue;
		float temp_size = norm(corners_plane[i][0] - corners_plane[i][2]);
		if (temp_size > 100.f && std::find(validplane.begin(), validplane.end(), i) != validplane.end()) {
			orig_idx_h_v_plane[1].push_back(i);
		}

		if (max_wall_size < temp_size) {
			max_wall_size = temp_size;
			max_wall_idx = i;
		}
	}

	std::vector<float> wall_widths(pts_planes.size());
	std::fill(wall_widths.begin(), wall_widths.end(), -1);
	for (auto wall_angle : wall_angle_list)
	{
		cv::Mat mat_to_y_c;
		float angle_to_y_c;
		float norm_wall[3];
		norm_wall[0] = norm_planes[wall_angle.first].x;
		norm_wall[1] = norm_planes[wall_angle.first].y;
		norm_wall[2] = norm_planes[wall_angle.first].z;

		MeasureBase::CalcAngleVectorXY2YAxis(&norm_wall[0], &angle_to_y_c);
		mat_to_y_c = MeasureBase::TranslateAngleAroundZ2RotationMatrix(angle_to_y_c);

		std::vector<cv::Point3f> wall_corners;
		if (!MeasureBase::RotatePoints(corners_plane[wall_angle.first], mat_to_y_c, wall_corners))
			continue;

		wall_widths[wall_angle.first] = abs(wall_corners[2].x - wall_corners[0].x);
	}

	//for horizontal planes
	for (int i = 0; i < norm_planes.size(); i++) {
		if (std::abs(std::abs(norm_planes[i].z) - 1) >= 0.3)
			continue;  //skip verticle plane

		cv::Point3f plane_normal;
		corners_plane[i].resize(4);
		plane_normal.x = norm_planes[max_wall_idx].x;
		plane_normal.y = norm_planes[max_wall_idx].y;
		plane_normal.z = norm_planes[i].z;

		if (!MeasureFindVerticeFcn(
			plane_normal, pts_planes[i], corners_plane[i]))
		    continue;

		if (std::find(validplane.begin(), validplane.end(), i) != validplane.end())
			orig_idx_h_v_plane[0].push_back(i);
	}
	/***************** merge plane & refind vertices **********************/
	//level planes
	//std::vector<std::vector<unsigned int>> indx_same_plane;
	//Not merge level planes
	idx_h_v_plane[0].resize(orig_idx_h_v_plane[0].size());
	for (int i = 0; i < orig_idx_h_v_plane[0].size(); i++)
	{
		merged_pts_planes[orig_idx_h_v_plane[0][i]].resize(pts_planes[orig_idx_h_v_plane[0][i]].size());
		merged_pts_planes[orig_idx_h_v_plane[0][i]] = pts_planes[orig_idx_h_v_plane[0][i]];
		merged_reflects_planes[orig_idx_h_v_plane[0][i]].resize(scene_plane_reflect[orig_idx_h_v_plane[0][i]].size());
		merged_reflects_planes[orig_idx_h_v_plane[0][i]] = scene_plane_reflect[orig_idx_h_v_plane[0][i]];
		idx_h_v_plane[0][i] = orig_idx_h_v_plane[0][i];
	} //H

	indx_same_plane.clear();
	indx_same_plane.resize(orig_idx_h_v_plane[1].size());

	for (int i = 0; i < orig_idx_h_v_plane[1].size(); i++)
	{
		for (int k = 0; k < i; k++) {
			for (int kj = 0; kj < indx_same_plane[k].size(); kj++) {
				if (orig_idx_h_v_plane[1][i] == indx_same_plane[k][kj])
				{
					goto loop;
				}
			}
		}

		if (std::abs(corners_plane[orig_idx_h_v_plane[1][i]][2].z -
			corners_plane[orig_idx_h_v_plane[1][i]][0].z) < 150.f)
			continue;

		float norm_wall[3];
		norm_wall[0] = norm_planes[orig_idx_h_v_plane[1][i]].x;
		norm_wall[1] = norm_planes[orig_idx_h_v_plane[1][i]].y;
		norm_wall[2] = norm_planes[orig_idx_h_v_plane[1][i]].z;

		for (int j = i + 1; j < orig_idx_h_v_plane[1].size(); j++)
		{
			if (i == j ||
				(std::abs(corners_plane[orig_idx_h_v_plane[1][j]][2].z
					- corners_plane[orig_idx_h_v_plane[1][j]][0].z) < 150.f))
				continue;

		
			float j_plane_normal[3];
			j_plane_normal[0] = norm_planes[orig_idx_h_v_plane[1][j]].x;
			j_plane_normal[1] = norm_planes[orig_idx_h_v_plane[1][j]].y;
			j_plane_normal[2] = norm_planes[orig_idx_h_v_plane[1][j]].z;

			float min_distance = 1.0E10;
			int  id_min_i = 0;
			int  id_min_j = 0;
			for (int ki = 0; ki < corners_plane[orig_idx_h_v_plane[1][i]].size(); ki++)
			{
				for (int kj = 0; kj < corners_plane[orig_idx_h_v_plane[1][j]].size(); kj++)
				{
					if (cv::norm(corners_plane[orig_idx_h_v_plane[1][i]][ki] - corners_plane[orig_idx_h_v_plane[1][j]][kj]) < min_distance)
					{
						id_min_i = ki;
						id_min_j = kj;
						min_distance = cv::norm(corners_plane[orig_idx_h_v_plane[1][i]][ki] - corners_plane[orig_idx_h_v_plane[1][j]][kj]);
					}
				}
			}
			if (std::abs(MeasureBase::DotProduct(norm_wall, j_plane_normal)) > 0.96 &&
				MeasureBase::DistTwoPlane(corners_plane[orig_idx_h_v_plane[1][i]][id_min_i], norm_wall,
					corners_plane[orig_idx_h_v_plane[1][j]][id_min_j], j_plane_normal) <= m_thres_offset_isSamePlane/* &&
				IsPtInRect(corners_plane[orig_idx_h_v_plane[1][i]],
					corners_plane[orig_idx_h_v_plane[1][j]]) == false*/)
			{
				std::vector<cv::Point3f> T_corners_plane, R_corners_plane;
				cv::Mat mat_to_y_c;
				float angle_to_y_c;
				MeasureBase::CalcAngleVectorXY2YAxis(&norm_wall[0], &angle_to_y_c);
				mat_to_y_c = MeasureBase::TranslateAngleAroundZ2RotationMatrix(angle_to_y_c);

				for (int n = 0; n < corners_plane[orig_idx_h_v_plane[1][i]].size(); n++) {
					T_corners_plane.push_back(corners_plane[orig_idx_h_v_plane[1][i]][n]);
				}
				for (int n = 0; n < corners_plane[orig_idx_h_v_plane[1][j]].size(); n++) {
					T_corners_plane.push_back(corners_plane[orig_idx_h_v_plane[1][j]][n]);
				}
				if (!MeasureBase::RotatePoints(T_corners_plane, mat_to_y_c, R_corners_plane))
					continue;

				
				std::vector<cv::Point2f> L_Corners, R_Corners;
				for (int n = 0; n < 4; n++)
					L_Corners.push_back(cv::Point2f(R_corners_plane[n].x, R_corners_plane[n].z));
				for (int n = 4; n < 8; n++)
					R_Corners.push_back(cv::Point2f(R_corners_plane[n].x, R_corners_plane[n].z));

				cv::Rect rc_i = cv::boundingRect(L_Corners);
				cv::Rect rc_j = cv::boundingRect(R_Corners);

				cv::Rect rect_or = rc_i | rc_j;
				cv::Rect rect_and = rc_i & rc_j;
				//compute iou

				double IOU = rect_and.area() *1.0 / rect_or.area();

				float min_width = min(rc_i.width, rc_j.width);
				if (IOU > 0.5 || (rect_and.area()*1.0 / rc_i.area() > 0.8 && rc_i.width > 600.0f) ||
					(rect_and.area()*1.0 / rc_j.area() > 0.8 && rc_j.width > 600.0f))
					continue;

				bool fInHole = false;
				for (auto door_window : door_window_info[orig_idx_h_v_plane[1][i]])
				{
					std::vector<cv::Point3f>  door_window_corners;
					if (!MeasureBase::RotatePoints(door_window.corners, mat_to_y_c, door_window_corners))
						continue;

					std::vector<cv::Point2f> Corners;
					for (int n = 0; n < 4; n++)
						Corners.push_back(cv::Point2f(door_window_corners[n].x, door_window_corners[n].z));
					cv::Rect rc_i_dw = cv::boundingRect(Corners);
					

					cv::Rect rect_or = rc_i_dw | rc_j;
					cv::Rect rect_and = rc_i_dw & rc_j;
					//compute iou

					double IOU = rect_and.area() *1.0 / rect_or.area();

					if (IOU > 0.5 || (rect_and.area()*1.0 / rc_i_dw.area() > 0.8) ||
						(rect_and.area()*1.0 / rc_j.area() > 0.8))
						fInHole = true;
				}

				for (auto door_window : door_window_info[orig_idx_h_v_plane[1][j]])
				{
					std::vector<cv::Point3f>  door_window_corners;
					if (!MeasureBase::RotatePoints(door_window.corners, mat_to_y_c, door_window_corners))
						continue;

					std::vector<cv::Point2f> Corners;
					for (int n = 0; n < 4; n++)
						Corners.push_back(cv::Point2f(door_window_corners[n].x, door_window_corners[n].z));

					cv::Rect rc_j_dw = cv::boundingRect(Corners);

					cv::Rect rect_or = rc_j_dw | rc_i;
					cv::Rect rect_and = rc_j_dw & rc_i;
					//compute iou

					double IOU = rect_and.area() *1.0 / rect_or.area();

					if (IOU > 0.5 || (rect_and.area()*1.0 / rc_i.area() > 0.8) ||
						(rect_and.area()*1.0 / rc_j_dw.area() > 0.8))
						fInHole = true;
				}
				
				if(fInHole)
					continue;

				std::vector<int>  wall_link_list;
				if (IOU == 0.0f)
				{
					//
					int wall_id_start = orig_idx_h_v_plane[1][i];
					int wall_id_end = orig_idx_h_v_plane[1][j];
					auto iter_i = std::find_if(wall_angle_list.begin(), wall_angle_list.end(), [wall_id_start](auto v)->bool {return v.first == wall_id_start; });
					auto iter_j = std::find_if(wall_angle_list.begin(), wall_angle_list.end(), [wall_id_end](auto v)->bool {return v.first == wall_id_end; });
					
					if (iter_i != wall_angle_list.end() && iter_j != wall_angle_list.end())
					{
						wall_id_start = iter_i - wall_angle_list.begin();
						wall_id_end = iter_j - wall_angle_list.begin();
						if (wall_id_end < wall_id_start)
						{
							wall_id_start = iter_j - wall_angle_list.begin();
							wall_id_end = iter_i - wall_angle_list.begin();
						}

						int wall_link_count = wall_id_end - wall_id_start - 1;
						if ((wall_id_start + wall_angle_list.size() - wall_id_end - 1) < wall_link_count)
						{
							wall_link_count = wall_id_start + wall_angle_list.size() - wall_id_end - 1;
							wall_id_start = wall_id_end;
						}

						for (int wid = 0; wid < wall_link_count; wid++)
						{
							wall_link_list.push_back(wall_angle_list[(wall_id_start + 1 + wid)% wall_angle_list.size()].first);
						}

					}

				}

				//std::cout << "i==" << orig_idx_h_v_plane[1][i] << ",j==" << orig_idx_h_v_plane[1][j] << std::endl;
				bool bNeedMerge = true;

				if (wall_link_list.size() >= 3)
				{
					for (auto wid : wall_link_list)
					{
						float w_plane_normal[3];
						w_plane_normal[0] = norm_planes[wid].x;
						w_plane_normal[1] = norm_planes[wid].y;
						w_plane_normal[2] = norm_planes[wid].z;

						float wall_distance = rc_j.x > (rc_i.width + rc_i.x) ? rc_j.x - (rc_i.width + rc_i.x) : rc_i.x - (rc_j.width + rc_j.x);
						if (std::abs(MeasureBase::DotProduct(norm_wall, w_plane_normal)) > 0.8 &&
							MeasureBase::DistTwoPlane(mid_planes[orig_idx_h_v_plane[1][i]], norm_wall,
								mid_planes[wid], w_plane_normal) >= 500.0f &&
							wall_distance > 1500.0f &&
							wall_widths[wid] * 2 > wall_distance)
						{
							bNeedMerge = false;
							break;
							
						}
					}
				}
				if(bNeedMerge)
					indx_same_plane[i].push_back(orig_idx_h_v_plane[1][j]);
#if 0
				if (min(R_corners_plane[0].z, R_corners_plane[1].z) > max(R_corners_plane[6].z, R_corners_plane[7].z) ||
					min(R_corners_plane[4].z, R_corners_plane[5].z) > max(R_corners_plane[2].z, R_corners_plane[3].z) ||
					min(R_corners_plane[0].x, R_corners_plane[3].x) > max(R_corners_plane[5].x, R_corners_plane[6].x) ||
					min(R_corners_plane[4].x, R_corners_plane[7].x) > max(R_corners_plane[1].x, R_corners_plane[2].x))

				{
					indx_same_plane[i].push_back(orig_idx_h_v_plane[1][j]);
				}
#endif
			}
		}

		//  merge, then refind max plane vertice  
		if (indx_same_plane[i].size() > 0)
		{
			int size = pts_planes[orig_idx_h_v_plane[1][i]].size();
			for (int n = 0; n < indx_same_plane[i].size(); n++)
				size += pts_planes[indx_same_plane[i][n]].size();

			//cope all into one plane & find max plane
			std::vector<cv::Point3f> temp_plane_xyz(size);
			std::vector<unsigned char> temp_plane_reflect(size);
			int count = 0;

			int indx_maxplane = orig_idx_h_v_plane[1][i];
			float max_diag = cv::norm(corners_plane[orig_idx_h_v_plane[1][i]][0] - corners_plane[orig_idx_h_v_plane[1][i]][2]);

			for (int m = 0; m < pts_planes[orig_idx_h_v_plane[1][i]].size(); m++)
			{
				temp_plane_xyz[count] = pts_planes[orig_idx_h_v_plane[1][i]][m];
				count++;
			}
			for (int n = 0; n < indx_same_plane[i].size(); n++)
			{
				for (int m = 0; m < pts_planes[indx_same_plane[i][n]].size(); m++)
				{
					temp_plane_xyz[count] = pts_planes[indx_same_plane[i][n]][m];
					temp_plane_reflect[count] = scene_plane_reflect[indx_same_plane[i][n]][m];
					count++;

					float temp_diag = cv::norm(corners_plane[indx_same_plane[i][n]][0]
						- corners_plane[indx_same_plane[i][n]][2]);
					if (temp_diag > max_diag)
					{
						max_diag = temp_diag;
						indx_maxplane = indx_same_plane[i][n];
					}
				}
			}

			//find large plane
			merged_pts_planes[indx_maxplane].resize(temp_plane_xyz.size());
			merged_pts_planes[indx_maxplane] = temp_plane_xyz;
			merged_reflects_planes[indx_maxplane].resize(temp_plane_reflect.size());
			merged_reflects_planes[indx_maxplane] = temp_plane_reflect;
			if (!MeasureFindVerticeFcn(cv::Point3f(norm_wall[0], norm_wall[1], norm_wall[2]),
				temp_plane_xyz,
				corners_plane[indx_maxplane]))
				continue;
			idx_h_v_plane[1].push_back(indx_maxplane);

			mid_planes[indx_maxplane] = (corners_plane[indx_maxplane][0]
				+ corners_plane[indx_maxplane][1]
				+ corners_plane[indx_maxplane][2]
				+ corners_plane[indx_maxplane][3])*0.25f;
		}
		else {
			merged_pts_planes[orig_idx_h_v_plane[1][i]].resize(pts_planes[orig_idx_h_v_plane[1][i]].size());
			merged_pts_planes[orig_idx_h_v_plane[1][i]] = pts_planes[orig_idx_h_v_plane[1][i]];
			merged_reflects_planes[orig_idx_h_v_plane[1][i]].resize(scene_plane_reflect[orig_idx_h_v_plane[1][i]].size());
			merged_reflects_planes[orig_idx_h_v_plane[1][i]] = scene_plane_reflect[orig_idx_h_v_plane[1][i]];
			mid_planes[orig_idx_h_v_plane[1][i]] = (corners_plane[orig_idx_h_v_plane[1][i]][0]
				+ corners_plane[orig_idx_h_v_plane[1][i]][1]
				+ corners_plane[orig_idx_h_v_plane[1][i]][2]
				+ corners_plane[orig_idx_h_v_plane[1][i]][3])*0.25f;

			idx_h_v_plane[1].push_back(orig_idx_h_v_plane[1][i]);
		}
	loop:;
	}

	orig_idx_h_v_plane.clear();
	return true;
}

bool MeasureDoorWindow::LocatePlane_corners(
	const std::vector<std::vector<cv::Point3f>> &pts_planes,
	const std::vector<cv::Point3f> &norm_planes,
	const std::vector<cv::Point3f> &mid_planes,
	const std::vector<std::vector<unsigned int>> &idx_h_v_plane,
	const std::vector<std::vector<cv::Point3f>> &raw_corners,
	std::vector<std::vector<cv::Point3f>> &corners_plane)
{
	if (pts_planes.size() == 0)
		return false;

	corners_plane.resize(raw_corners.size());
	corners_plane = raw_corners;
	// for vertical/wall planes
	for (int i = 0; i < idx_h_v_plane[1].size(); i++) 
	{
		unsigned int idx_cur = idx_h_v_plane[1][i];
		if ((min(cv::norm(raw_corners[idx_cur][0] - raw_corners[idx_cur][1]), 
				 cv::norm(raw_corners[idx_cur][2] - raw_corners[idx_cur][1])) < 600.f)
			|| (raw_corners[idx_cur][0].z > 0))
		{
			continue;
		}

		float norm_wall[3];
		norm_wall[0] = norm_planes[idx_cur].x;
		norm_wall[1] = norm_planes[idx_cur].y;
		norm_wall[2] = norm_planes[idx_cur].z;
		if (norm_planes[idx_cur].dot(mid_planes[idx_cur]) < 0)
		{
			norm_wall[0] = -norm_wall[0];
			norm_wall[1] = -norm_wall[1];
			norm_wall[2] = -norm_wall[2];
		}
		/****************************** For adjacent walls ******************************/
		/*1 select perpendicular and heigh walls (not samll vertical walls for ceilings) to this one*/
		///*2 cut connected walls outside the room*/
		/*2 classify selected walls on left and right sides of this wall center*/
		float min_dist_1 = INFINITE, min_dist_2 = INFINITE;
		std::array<int, 2> idx_nbrs = { -1,-1 };
		std::array<int, 2> corner_idx = { -1, -1 };//get indx for two closest vertices 

		for (int k = 0; k < idx_h_v_plane[1].size(); k++) 
		{
			unsigned int idx_cmp = idx_h_v_plane[1][k];
			if (i != k &&
				(mid_planes[idx_cmp].z
					<= max(raw_corners[idx_cur][0].z, raw_corners[idx_cur][2].z)
					|| std::abs(raw_corners[idx_cmp][2].z - raw_corners[idx_cmp][0].z) >= 1000.f))
			{
				float wallk_plane_normal[3];
				wallk_plane_normal[0] = norm_planes[idx_cmp].x;
				wallk_plane_normal[1] = norm_planes[idx_cmp].y;
				wallk_plane_normal[2] = norm_planes[idx_cmp].z;

				float dot_value = std::abs(MeasureBase::DotProduct(norm_wall, wallk_plane_normal));
				//1 perpendicular
				if (dot_value < 0.25)
				{
					//cut this wall outside room 
					if (IsWallOutsideRoom(mid_planes[idx_cur],
						norm_wall,
						mid_planes[idx_cmp],
						wallk_plane_normal)) {
						// goto loop;
						continue;
					}
						
					// dist between perpendicular wall corners and wall along wall normal
					float dist_A = INFINITY, dist_B = INFINITY;
					for (int i_k = 0; i_k < 4; i_k++)
					{
						dist_A = min(dist_A, cv::norm(raw_corners[idx_cmp][0] - raw_corners[idx_cur][i_k]));
						dist_A = min(dist_A, cv::norm(raw_corners[idx_cmp][3] - raw_corners[idx_cur][i_k]));
						/*MeasureBase::DistTwoPlane(raw_corners[idx_cmp][0], norm_wall, mid_planes[idx_cur], norm_wall)*/

						dist_B = min(dist_B, cv::norm(raw_corners[idx_cmp][1] - raw_corners[idx_cur][i_k]));
						dist_B = min(dist_B, cv::norm(raw_corners[idx_cmp][2] - raw_corners[idx_cur][i_k]));
						/*float dist_B = MeasureBase::DistTwoPlane(raw_corners[idx_cmp][2],norm_wall,mid_planes[idx_cur],norm_wall);*/
					}

					float min_AB = min(dist_A, dist_B);
					//see if two walls projected to xy-plane intersect
					cv::Point SegA_p1, SegA_p2, SegB_p1, SegB_p2;
					SegA_p1.x = raw_corners[idx_cmp][0].x; SegA_p1.y = raw_corners[idx_cmp][0].y;
					SegA_p2.x = raw_corners[idx_cmp][1].x; SegA_p2.y = raw_corners[idx_cmp][1].y;
					SegB_p1.x = raw_corners[idx_cur][0].x; SegB_p1.y = raw_corners[idx_cur][0].y;
					SegB_p2.x = raw_corners[idx_cur][1].x; SegB_p2.y = raw_corners[idx_cur][1].y;
					min_AB = min(min_AB,IsLineSegIntersect2D(SegA_p1, SegA_p2, SegB_p1, SegB_p2));

					//2 classify by cross product value: z>0 or <0
					float vector[3];
					float u[3];
					vector[0] = mid_planes[idx_cur].x - mid_planes[idx_cmp].x;
					vector[1] = mid_planes[idx_cur].y - mid_planes[idx_cmp].y;
					vector[2] = mid_planes[idx_cur].z - mid_planes[idx_cmp].z;

					MeasureBase::CrossProduct(vector, norm_wall, u);

					if (u[2] > 0){
						//dist_A = min(dist_A, dist_B);
						if (min_dist_1 > min_AB){
							//get indx for two closest vertices  
							if (dist_A < dist_B)
								corner_idx[0] = 3;
							else
								corner_idx[0] = 1;

							min_dist_1 = min_AB;
							idx_nbrs[0] = k;
						}
					}else{
						if (min_dist_2 > min_AB){
							//get indx for two closest vertices  
							if (dist_A < dist_B)
								corner_idx[1] = 3;
							else
								corner_idx[1] = 1;

							min_dist_2 = min_AB;
							idx_nbrs[1] = k;
						}
					}
				}
			}
		//loop:;
		}

		//reset vertices projected on this wall
		//	std::cout << idx_h_v_plane[1][idx_nbrs[0]] ;
		std::vector<cv::Point3f> temp_vertice(4);

		float cent_z = 0.f;
		int count = 0;
		for (int n = 0; n < 2; n++)
		{
			if (idx_nbrs[n] >= 0)
			{
				for (int m = 0; m < 2; m++)
				{
					int l = m + corner_idx[n];
					if (l > 3)
						l = 0;
					//std::cout << norm_wall[0] << ", " << norm_wall[1] << ", " << norm_wall[2] << std::endl;
					float  k_plane_normal[3];
					k_plane_normal[0] = norm_planes[idx_h_v_plane[1][idx_nbrs[n]]].x;
					k_plane_normal[1] = norm_planes[idx_h_v_plane[1][idx_nbrs[n]]].y;
					k_plane_normal[2] = norm_planes[idx_h_v_plane[1][idx_nbrs[n]]].z;
					cv::Point3f p;
					p = CalPointIntersectPlanePoint(raw_corners[idx_h_v_plane[1][idx_nbrs[n]]][l],
						k_plane_normal,
						mid_planes[idx_h_v_plane[1][idx_nbrs[n]]]);
					temp_vertice[count] = CalPointIntersectPlanePoint(p,
						norm_wall,
						mid_planes[idx_cur]);
					cent_z += temp_vertice[count].z;
					count++;
				}
			}
		}

		if (count == 2) {
			cv::Point3f center_k = (temp_vertice[0] + temp_vertice[1])*0.5f;
			float dist_i1 = cv::norm(center_k - (raw_corners[idx_cur][0] + raw_corners[idx_cur][3])*0.5f);
			float dist_i2 = cv::norm(center_k - (raw_corners[idx_cur][1] + raw_corners[idx_cur][2])*0.5f);
			if (dist_i1>dist_i2) {
				temp_vertice[2] = raw_corners[idx_cur][0];
				temp_vertice[3] = raw_corners[idx_cur][3];
			}else {
				temp_vertice[2] = raw_corners[idx_cur][1];
				temp_vertice[3] = raw_corners[idx_cur][2];
			}
			cent_z += (temp_vertice[2].z + temp_vertice[3].z);
			count = 4;
		}

		if (count > 0) {
			cent_z *= 0.25f;
			if (temp_vertice[0].z < temp_vertice[3].z)
			{
				corners_plane[idx_cur][0] = temp_vertice[0];
				corners_plane[idx_cur][3] = temp_vertice[3];
			}
			else {
				corners_plane[idx_cur][0] = temp_vertice[3];
				corners_plane[idx_cur][3] = temp_vertice[0];
			}

			if (temp_vertice[1].z < temp_vertice[2].z)
			{
				corners_plane[idx_cur][1] = temp_vertice[1];
				corners_plane[idx_cur][2] = temp_vertice[2];
			}
			else {
				corners_plane[idx_cur][1] = temp_vertice[2];
				corners_plane[idx_cur][2] = temp_vertice[1];
			}
		}
		// continue;
		/*********************************************************************/

		/****************************** For upper and bottom boundness ******************************/
		float min_dist_up = INFINITE, min_dist_btm = INFINITE;
		int idx_up = -1, idx_low = -1;
		for (int j = 0; j < idx_h_v_plane[0].size(); j++) {
			if ((cv::norm(raw_corners[idx_h_v_plane[0][j]][0]
				- raw_corners[idx_h_v_plane[0][j]][2]) < 1000.f))
				continue;

			// dist between level plane vertice and wall along wall normal 
			float dist = INFINITE;
			for (int j_k = 0; j_k < 4; j_k++)
			{
				for (int i_k = 0; i_k < 4; i_k++)
				{
					float distA = cv::norm(raw_corners[idx_h_v_plane[0][j]][j_k]
						- raw_corners[idx_cur][i_k]);
					dist = min(dist, distA);
				}
				//dist = min(dist,distA);
			}
			float  j_plane_normal[3];
			j_plane_normal[0] = norm_planes[idx_h_v_plane[0][j]].x;
			j_plane_normal[1] = norm_planes[idx_h_v_plane[0][j]].y;
			j_plane_normal[2] = norm_planes[idx_h_v_plane[0][j]].z;
			float u[3];
			MeasureBase::CrossProduct(norm_wall, j_plane_normal, u);
			//float v[3] = { 0.f,1.f,0.f }; 
			//see if two walls projected to xy-plane intersect
			cv::Mat rotMat(3, 3, CV_32F); 
			rotMat = MeasureBase::CalRotationMatrixFromVectors(j_plane_normal, u);
			cv::Point SegA_p1, SegA_p2, SegB_p1, SegB_p2;
			SegA_p1.x = rotMat.at<float>(0, 0)  *raw_corners[idx_h_v_plane[0][j]][0].x
				+ rotMat.at<float>(0, 1)  *raw_corners[idx_h_v_plane[0][j]][0].y
				+ rotMat.at<float>(0, 2)  *raw_corners[idx_h_v_plane[0][j]][0].z;
			SegA_p1.y = rotMat.at<float>(1, 0)  *raw_corners[idx_h_v_plane[0][j]][0].x
				+ rotMat.at<float>(1, 1)  *raw_corners[idx_h_v_plane[0][j]][0].y
				+ rotMat.at<float>(1, 2)  *raw_corners[idx_h_v_plane[0][j]][0].z;
			SegA_p2.x = rotMat.at<float>(0, 0)  *raw_corners[idx_h_v_plane[0][j]][2].x
				+ rotMat.at<float>(0, 1)  *raw_corners[idx_h_v_plane[0][j]][2].y
				+ rotMat.at<float>(0, 2)  *raw_corners[idx_h_v_plane[0][j]][2].z;
			SegA_p2.y = rotMat.at<float>(1, 0)  *raw_corners[idx_h_v_plane[0][j]][2].x
				+ rotMat.at<float>(1, 1)  *raw_corners[idx_h_v_plane[0][j]][2].y
				+ rotMat.at<float>(1, 2)  *raw_corners[idx_h_v_plane[0][j]][2].z;
			SegB_p1.x = rotMat.at<float>(0, 0)  *raw_corners[idx_cur][0].x
				+ rotMat.at<float>(0, 1)  *raw_corners[idx_cur][0].y
				+ rotMat.at<float>(0, 2)  *raw_corners[idx_cur][0].z;
			SegB_p1.y = rotMat.at<float>(1, 0)  *raw_corners[idx_cur][0].x
				+ rotMat.at<float>(1, 1)  *raw_corners[idx_cur][0].y
				+ rotMat.at<float>(1, 2)  *raw_corners[idx_cur][0].z;
			SegB_p2.x = rotMat.at<float>(0, 0)  *raw_corners[idx_cur][2].x
				+ rotMat.at<float>(0, 1)  *raw_corners[idx_cur][2].y
				+ rotMat.at<float>(0, 2)  *raw_corners[idx_cur][2].z;
			SegB_p2.y = rotMat.at<float>(1, 0)  *raw_corners[idx_cur][2].x
				+ rotMat.at<float>(1, 1)  *raw_corners[idx_cur][2].y
				+ rotMat.at<float>(1, 2)  *raw_corners[idx_cur][2].z;
			rotMat.release();
			dist = min(dist,IsLineSegIntersect2D(SegA_p1, SegA_p2, SegB_p1, SegB_p2));

			//find upper level planes
			if (mid_planes[idx_cur].z < mid_planes[idx_h_v_plane[0][j]].z) {
				if (min_dist_up > dist){
					min_dist_up = dist;
					idx_up = j;
				}
			}else {//grounds
				if (min_dist_btm > dist){
					min_dist_btm = dist;
					idx_low = j;
				}
			}
		}

#ifdef OUTPUT_DEBUG_INFO 
		std::cout << idx_cur << ": ";
		std::cout << idx_h_v_plane[1][idx_nbrs[0]] << ", " << idx_h_v_plane[1][idx_nbrs[1]] << ";     ";
		std::cout << idx_h_v_plane[0][idx_up] << "," << idx_h_v_plane[0][idx_low] << std::endl;
#endif
		if (idx_up > -1)
		{
			float bound = min(raw_corners[idx_h_v_plane[0][idx_up]][2].z,
				mid_planes[idx_h_v_plane[0][idx_up]].z);
			corners_plane[idx_cur][2].z = (bound > mid_planes[idx_cur].z) ? bound : mid_planes[idx_cur].z;
			corners_plane[idx_cur][3].z = corners_plane[idx_cur][2].z;
		}
		if (idx_low > -1)
		{
			float bound = max(raw_corners[idx_h_v_plane[0][idx_low]][0].z,
				raw_corners[idx_h_v_plane[0][idx_low]][1].z);
			corners_plane[idx_cur][0].z = (bound < mid_planes[idx_cur].z) ? bound : mid_planes[idx_cur].z;
			corners_plane[idx_cur][1].z = corners_plane[idx_cur][0].z;
		}
		/*********************************************************************/
		//float diag = max( norm(corners_plane[i][0]- corners_plane[i][2]), norm(corners_plane[i][0] - corners_plane[i][3]));
		/*if ( IsPtInRect(corners_plane[idx_cur], raw_corners[idx_cur]))
		corners_plane[idx_cur]=raw_corners[idx_cur];*/

		// sorting corners
		std::vector<cv::Point3f> sorted_corners;
		SortCorners(corners_plane[idx_cur], sorted_corners);
		corners_plane[idx_cur] = sorted_corners;
		sorted_corners.clear();
	}
	return true;
}

void MeasureDoorWindow::SortCorners(const std::vector<cv::Point3f> &corners, std::vector<cv::Point3f> &sorted_corners)
{
	// assume first two corners are at bottom-left and bottom-right, then reorder to counter-clockwise
	sorted_corners.resize(corners.size());
	sorted_corners[0] = corners[0];
	sorted_corners[1] = corners[1];
	float d12 = cv::norm(corners[1] - corners[2]);
	float d13 = cv::norm(corners[1] - corners[3]);
	if (d12 <= d13) {
		sorted_corners[2] = corners[2];
		sorted_corners[3] = corners[3];
	}
	else {
		sorted_corners[2] = corners[3];
		sorted_corners[3] = corners[2];
	}
}

bool MeasureDoorWindow::IsPtInRect(
	const std::vector<cv::Point3f> &realbound_vertices,
	const std::vector<cv::Point3f> &plane_vertices)
{
	// data validation
	if (plane_vertices.size() != 4 || realbound_vertices.size() != 4) return false;

	cv::Point3f realbound_center = (realbound_vertices[0] + realbound_vertices[1] + realbound_vertices[2] + realbound_vertices[3])*0.25f;
	cv::Point3f plane_center = (plane_vertices[0] + plane_vertices[1] + plane_vertices[2] + plane_vertices[3])*0.25f;
	float L = 0.f;
	float Lz = std::abs(realbound_center.z - plane_center.z);

	float realbound_w;
	float realbound_h = max(std::abs(realbound_vertices[0].z - realbound_vertices[1].z), std::abs(realbound_vertices[0].z - realbound_vertices[2].z));
	float plane_w;
	float plane_h = max(std::abs(plane_vertices[0].z - plane_vertices[1].z), std::abs(plane_vertices[0].z - plane_vertices[2].z));
	if (std::abs(plane_vertices[0].x - plane_vertices[2].x) > std::abs(plane_vertices[0].y - plane_vertices[2].y))
	{
		L = std::abs(realbound_center.x - plane_center.x);
		realbound_w = std::abs(realbound_vertices[0].x - realbound_vertices[2].x);
		plane_w = std::abs(plane_vertices[0].x - plane_vertices[2].x);
	}
	else {
		L = std::abs(realbound_center.y - plane_center.y);
		realbound_w = std::abs(realbound_vertices[0].y - realbound_vertices[2].y);
		plane_w = std::abs(plane_vertices[0].y - plane_vertices[2].y);
	}

	if (L <= (realbound_w + plane_w)*0.5f  &&
		Lz <= (realbound_h + plane_h)*0.25f)
		return true;// intersect
	else
		return false;
}

bool MeasureDoorWindow::IsWallOutsideRoom(const cv::Point3f or_plane_center,
	const float plane_normal[3],
	const cv::Point3f or_wall_center,
	const float wall_normal[3]) {
	//plane_normal: point to outside room
	// all point projected on xy-plane
	cv::Point2f plane_center, wall_center;

	plane_center.x = or_plane_center.x;
	plane_center.y = or_plane_center.y;

	wall_center.x = or_wall_center.x;
	wall_center.y = or_wall_center.y;
	//1: get intersection point 
	float D = plane_normal[0] * wall_normal[1] - plane_normal[1] * wall_normal[0];

	//parallel
	if (std::abs(D) < 0.01)
		return false;

	D = 1 / D;
	float c0 = -(plane_normal[0] * plane_center.x + plane_normal[1] * plane_center.y);
	float c1 = -(wall_normal[0] * wall_center.x + wall_normal[1] * wall_center.y);

	cv::Point2f t;
	t.x = (plane_normal[1] * c1 - wall_normal[1] * c0)*D;
	t.y = (wall_normal[0] * c0 - plane_normal[0] * c1)*D;

	//see if ouside room	
	float u = wall_center.cross(cv::Point2f{ plane_normal[0],plane_normal[1] }); 
	float v = wall_center.cross(t);// MathOperation::ComputeVectorCrossProduct(wall_center, t);

	return((u*v) < 0);
}

float MeasureDoorWindow::IsLineSegIntersect2D(const cv::Point2f SegA_p1, const cv::Point2f SegA_p2, const cv::Point2f SegB_p1, const cv::Point2f SegB_p2) {
	//parallel
	cv::Point2f segA = SegA_p1 - SegA_p2;
	cv::Point2f segB = SegB_p1 - SegB_p2;
	if (norm(segA)<0.01 || cv::norm(segB)<0.01) return INFINITY;

	segA /= cv::norm(segA); segB /= cv::norm(segB);

	if (std::abs(std::abs(MathOperation::ComputeVectorDotProduct(segA, segB)) - 1) < 0.001) return INFINITY;

	float u = (SegB_p1 - SegA_p1).cross(SegB_p1 - SegB_p2);// MathOperation::ComputeVectorCrossProduct(SegB_p1 - SegA_p1, SegB_p1 - SegB_p2);
	float v = (SegB_p1 - SegA_p2).cross(SegB_p1 - SegB_p2);// MathOperation::ComputeVectorCrossProduct(SegB_p1 - SegA_p2, SegB_p1 - SegB_p2);
	float w = (SegA_p1 - SegB_p1).cross(SegA_p1 - SegA_p2);// MathOperation::ComputeVectorCrossProduct(SegA_p1 - SegB_p1, SegA_p1 - SegA_p2);
	float t = (SegA_p1 - SegB_p2).cross(SegA_p1 - SegA_p2);// MathOperation::ComputeVectorCrossProduct(SegA_p1 - SegB_p2, SegA_p1 - SegA_p2);

	if ((u * v) < (1e-6) || (w * t) < (1e-6))
	{
		if ((u * v) < (1e-6) && (w * t) < (1e-6)) {
			return 0.f;
		}
		else if ((u * v) < (1e-6)) {
			float a, b, c;
			a = SegA_p2.y - SegA_p1.y;
			b = SegA_p1.x - SegA_p2.x;
			c = -(b*SegA_p1.y + a*SegA_p1.x);
			float t = std::sqrt(a*a + b*b);
			float dist1 = std::abs(b*SegB_p1.y + a*SegB_p1.x + c) / t;
			float dist2 = std::abs(b*SegB_p2.y + a*SegB_p2.x + c) / t;
			return (min(dist1, dist2));
		}
		else {
			float a, b, c;
			a = SegB_p2.y - SegB_p1.y;
			b = SegB_p1.x - SegB_p2.x;
			c = -(b*SegB_p1.y + a*SegB_p1.x);
			float t = std::sqrt(a*a + b*b);
			float dist1 = std::abs(b*SegA_p1.y + a*SegA_p1.x + c) / t;
			float dist2 = std::abs(b*SegA_p2.y + a*SegA_p2.x + c) / t;
			return (min(dist1, dist2));
		}
	}
	else {
		return INFINITY;
	}
	return INFINITY;
}

cv::Point3f MeasureDoorWindow::CalPointIntersectPlanePoint(
	const cv::Point3f pointOutPlane, //rule center
	const float planeNormal[3],
	const cv::Point3f planePoint)  //vertice
{
	float  t, line[3];// vpt, v[3],
	line[0] = planePoint.x - pointOutPlane.x;
	line[1] = planePoint.y - pointOutPlane.y;
	line[2] = planePoint.z - pointOutPlane.z;

	//distance of two points along plane normal
	t = line[0] * planeNormal[0] + line[1] * planeNormal[1] + line[2] * planeNormal[2];

	//if parallel
	if (std::abs(t) < 1e-3)
	{
		return pointOutPlane;
	}
	else
	{
		cv::Point3f returnResult;
		returnResult.x = pointOutPlane.x + planeNormal[0] * t;
		returnResult.y = pointOutPlane.y + planeNormal[1] * t;
		returnResult.z = pointOutPlane.z + planeNormal[2] * t;
		return returnResult;
	}
}
