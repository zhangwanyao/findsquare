#pragma once

#include "DetectionHoleClass.h"
#include "DataStruct.h"

using namespace ModuleStruct;


class MeasureDoorWindow
{
	/**
	* \brief parameters of detection hole for external setting
	*/
	struct Params_DetcHole {
		float width_ruler = 40.f;
		float thres_minLen_hole = 500.f;
		float thres_maxLen_hole = 4000.f;
		float thres_minWidth_door = 500.f;
		float thres_maxWidth_door = 10000.f;
		float thres_minHeight_door = 1800.f;
		float thres_maxHeight_door = 4000.f;
		float sample_cstr_len = 400.f;
		float sample_margin = 200.f;
	};

	typedef struct holes_param
	{
		Vector<Vector<cv::Point3f>> corners_hole;
		Vector<std::array<float, 4>> hole_w_h;
		Vector<int> type_hole;
		Vector<int> wallid_hole;
	}holes_param;
public:
	typedef struct DoorWindowInfo
	{
		int type;  //0_window, 1_door
		Vector<cv::Point3f> corners;
		int wallId;
	}DoorWindowInfo;


public:
	static bool DoorWindowDetector(const Vector<Vector<cv::Point3f>>& pts, 
		const Vector<cv::Point3f>& normals,
		const Vector<int>& detect_plane_idx,
		float walltopZ, 
		Vector<Vector<DoorWindowInfo>>& door_window_info);
	static bool FindWindowCornerMinMax(const Vector<Vector<MeasureDoorWindow::DoorWindowInfo>>& door_window_info, const Vector<cv::Point3f>& scene_plane_normals, const Vector<cv::Point3f>& scene_plane_center, Vector<Vector<float>>& window_corner_minmax_xyz);

	static bool FindWindowCornerMinMax(const MeasureDoorWindow::DoorWindowInfo & door_window_info, const Vector<cv::Point3f>& scene_plane_normals, const Vector<cv::Point3f>& scene_plane_center, Vector<Vector<float>>& window_corner_minmax_xyz);

	static bool MergePlanes(Vector<Vector<cv::Point3f>>& pts, std::vector<std::vector<MeasureDoorWindow::DoorWindowInfo>> &door_window_info, std::vector<cv::Point3f> &center_plane_scene, std::vector<cv::Point3f> &normal_plane_scene, Vector<Vector<unsigned char>>& reflects, std::vector<std::pair<int, std::vector<float>>> &wall_list);
	static bool MeasureFindVerticeFcn(const cv::Point3f plane_normal, const std::vector<cv::Point3f>& plane_points, std::vector<cv::Point3f>& real_vertices);
private:
	
	static void FindVertice(const int normal_axis, const cv::Point3f plane_normal, const bool ifRotate, const float ang_value, const float * data_minmax_xyz, std::vector<cv::Point3f>& vertice);
	static void FindVerticeXY(const float * data_minmax_xyz, const cv::Point3f plane_normal, int * vertice_indx);
	
	static bool LocatePlane_HV(
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
		const std::vector<std::pair<int, std::vector<float>>> &wall_list);

	static bool LocatePlane_corners(
		const std::vector<std::vector<cv::Point3f>> &pts_planes,
		const std::vector<cv::Point3f> &norm_planes,
		const std::vector<cv::Point3f> &mid_planes,
		const std::vector<std::vector<unsigned int>> &idx_h_v_plane,
		const std::vector<std::vector<cv::Point3f>> &raw_corners,
		std::vector<std::vector<cv::Point3f>> &corners_plane);

	static void SortCorners(const std::vector<cv::Point3f> &corners, std::vector<cv::Point3f> &sorted_corners);

	static bool IsPtInRect(
		const std::vector<cv::Point3f> &realbound_vertices,
		const std::vector<cv::Point3f> &plane_vertices);

	static bool IsWallOutsideRoom(const cv::Point3f or_plane_center,
		const float plane_normal[3],
		const cv::Point3f or_wall_center,
		const float wall_normal[3]);
	static float IsLineSegIntersect2D(const cv::Point2f SegA_p1, const cv::Point2f SegA_p2, const cv::Point2f SegB_p1, const cv::Point2f SegB_p2);
	static cv::Point3f CalPointIntersectPlanePoint(
		const cv::Point3f pointOutPlane, //rule center
		const float planeNormal[3],
		const cv::Point3f planePoint);
};
