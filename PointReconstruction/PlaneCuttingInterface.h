#ifndef PLANE_CUTTING_INTERFACE_H
#define PLANE_CUTTING_INTERFACE_H

#include <vector>

//suggested opencv version: 3.4.1
#include <opencv2/core/core.hpp>
#include "CutScanPlane/DoorWindowDetector/MeasureDoorWindow.h"
enum MEASUREMENT_MODE
{
	WALL_BASE = 0,
	WALL_LESS = 1,
	BEAM_BASE = 2,
	FLATNESS_ONLY = 3,
	MANUAL = 4
};

//plane cutting result interface
struct PlaneCutResultInterface {
public:
	//plane x,y,z
	std::vector<std::vector<cv::Point3f>> plane_xyz;

	//plane reflectance
	std::vector<std::vector<unsigned char>> plane_reflect;

	//plane normals
	std::vector<cv::Point3f> plane_normals;

	//plane center
	std::vector<cv::Point3f> plane_center;

	//plane ground index
	std::vector<int> plane_ground_idx;

	//plane ceiling
	std::vector<int> plane_ceiling_idx;

	//plane wall
	std::vector<int> plane_wall_idx;

	//plane beam
	std::vector<int> plane_beam_idx;

	//connected and perpendicular wall index
	std::vector<std::pair<int, int>> L_shape_plane_idx;

	//connected and perpendicular wall index
	std::vector<std::pair<int, int>> parallel_plane_idx;

	//door window corner
	std::vector<std::vector<MeasureDoorWindow::DoorWindowInfo>> door_window_info;

	//plane corner points
	std::vector<std::vector<cv::Point3f>> plane_corners;//add 20230714

	//plane flateness_defect
	std::vector<std::pair<int, std::vector<std::pair<float, cv::Point3f>>>> flateness_defect;
	std::vector<std::pair<int, std::vector<std::pair<float, cv::Point3f>>>> removeCritical_flateness_defect;

	//plane area
	std::vector<float> plane_area;
public:
	PlaneCutResultInterface() {};
};

//plane cutting interface class
class PlaneCutInterface {
protected:
	PlaneCutInterface() {};
};

#endif