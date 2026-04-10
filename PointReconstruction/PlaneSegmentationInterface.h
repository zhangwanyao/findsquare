#ifndef PLANE_SEGMENTATION_INTERFACE_H
#define PLANE_SEGMENTATION_INTERFACE_H

#include <vector>

//suggested opencv version: 3.4.1
#include <opencv2/core/core.hpp>

//#ifdef DLL_API_EXPORTS
//#define DLL_API __declspec(dllexport)
//#else
//#define DLL_API __declspec(dllimport)
//#endif

//plane segmentation result interface
struct PlaneSegResultInterface {

public:


	//plane normals
	std::vector<cv::Point3f> plane_normals;

	//plane center
	std::vector<cv::Point3f> plane_center;

	//plane x,y,z
	std::vector<std::vector<cv::Point3f>> plane_xyz;

	//Add by simon
	//plane reflectance
	std::vector<std::vector<unsigned char>> plane_reflect;

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

	//plane corner points
	std::vector<std::vector<cv::Point3f>> plane_corners;

public:
	PlaneSegResultInterface() {};

};

//plane segmentation interface class
class PlaneSegInterface {
private:

protected:

	PlaneSegInterface() {};

public:

	//interface
	//input_data(in): input cloud x,y,z (in mm)
	virtual bool FitPlane(const std::vector<cv::Point3f> &input_data, const std::vector<int> &input_reflect, const bool is_cad, PlaneSegResultInterface &plane_seg_result) = 0;

};

#endif