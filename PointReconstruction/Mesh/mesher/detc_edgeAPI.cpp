
#include "detc_macro.h"
#include "../../log/log.h"
#include <iostream>
#include "detc_edge_api.h"
#include "detc_edge2d.h"
#include "util_time.hpp"
 
using namespace ModuleStruct;

void* Features_EdgeDetcCreateHandle()
{
	return (new Detc_Edge2D());
}

void Features_EdgeDetcDestroyHandle(void** handle)
{
	if (handle)
	{
		Detc_Edge2D* ptr = reinterpret_cast<Detc_Edge2D*>(*handle);
		delete ptr;
		ptr = nullptr;
	}
}

bool Features_getOrigMapOfContours2D(bool have_center_input, std::string output_path, void* handle, const Point3f plane_normal, const Point3f center, \
	Point3fArray* input_data, const double voxel_size, std::vector<Point2fArray>& contours,
	std::unordered_map<int,std::vector<int>>& tree)
{
	TIMING_DECLARE(TP1)
		TIMING_BEGIN(TP1)
		Detc_Edge2D* detector = reinterpret_cast<Detc_Edge2D*>(handle);
	//	new Detc_Edge2D(input_data, plane_normal, voxel_size);
	detector->setPtArray(input_data);
	detector->setNPlane(plane_normal);
	detector->initImgVoxelGrid(1, 1);
	detector->setVoxelSize(voxel_size);
	detector->setOutputPath(output_path);
	if (have_center_input)
	{
		detector->setPointsCenter(center);
	}


	bool isDetcInit = detector->init();
	if (!isDetcInit) {
		detector->clear();
		return false;
	}
	TIMING_END_ms("isDetcInit", TP1)

		TIMING_BEGIN(TP1)
		detector->detect();
	TIMING_END_ms("detect", TP1)

		TIMING_BEGIN(TP1)
		detector->get2DContours(contours, detector->m_hierarchy);
	tree=((detector->mapEdgeHoleList));
	TIMING_END_ms("getOrigMapOfContours", TP1)
		return true;
}

///
bool Features_getOrigMapOfContours2D(std::string output_path, void* handle, const Point3f plane_normal, \
	Point3fArray* input_data, const double voxel_size, std::vector<Point2fArray>& contours,
	std::unordered_map<int, std::vector<int>>& tree)
{
	return (Features_getOrigMapOfContours2D(false, output_path, handle, plane_normal, {}, input_data, voxel_size, contours,tree));
}

bool Features_getOrigMapOfContours2D(bool have_center_input, std::string output_path, void *handle, const Point3f plane_normal,const Point3f center, \
	Point3fArray *input_data, const double voxel_size, std::vector<Point2fArray> &contours)
{
	TIMING_DECLARE(TP1)
	TIMING_BEGIN(TP1)
	Detc_Edge2D* detector = reinterpret_cast<Detc_Edge2D*>(handle);
	//	new Detc_Edge2D(input_data, plane_normal, voxel_size);
	detector->setPtArray(input_data);
	detector->setNPlane(plane_normal);
	detector->initImgVoxelGrid(1, 1);
	detector->setVoxelSize(voxel_size);
	detector->setOutputPath(output_path);
	if (have_center_input)
	{
		detector->setPointsCenter(center);
	}


	bool isDetcInit = detector->init();
	if (!isDetcInit) {
		detector->clear();
		return false;
	}
	TIMING_END_ms("isDetcInit", TP1)

	TIMING_BEGIN(TP1)
	detector->detect();
	TIMING_END_ms("detect", TP1)

	TIMING_BEGIN(TP1)
	detector->get2DContours(contours,detector->m_hierarchy);
	TIMING_END_ms("getOrigMapOfContours", TP1)
	return true;
}

bool Features_getOrigMapOfContours2D(std::string output_path, void* handle, const Point3f plane_normal, \
	Point3fArray* input_data, const double voxel_size, std::vector<Point2fArray>& contours)
{
	return (Features_getOrigMapOfContours2D(false, output_path, handle, plane_normal, {}, input_data, voxel_size, contours));
}
bool Features_getOrigMapOfContours2D(void* handle, const Point3f plane_normal, Point3fArray* input_data, const double voxel_size, std::vector<Point2fArray>& contours)
{
	return (Features_getOrigMapOfContours2D(false, "\\", handle, plane_normal, {}, input_data, voxel_size, contours));
}

bool Features_getOrigMapOfContours2D(void* handle, const Point3f plane_normal, const Point3f center, \
	Point3fArray* input_data, const double voxel_size, std::vector<Point2fArray>& contours)
{
	return (Features_getOrigMapOfContours2D(true, "\\", handle, plane_normal, center, input_data, voxel_size, contours));

}

bool Features_getOrigMapOfContours2D(std::string output_path, void* handle, const Point3f plane_normal, const Point3f center, \
	Point3fArray* input_data, const double voxel_size, std::vector<Point2fArray>& contours)
{
	return (Features_getOrigMapOfContours2D(true, output_path, handle, plane_normal, center, input_data, voxel_size, contours));

}

bool Features_getOrigMapOfContours3D(bool have_center_input, const std::string output_path, void* handle, const Point3f plane_normal, const Point3f center, \
	Point3fArray *input_data, const double voxel_size, Vector<Point3fArray> &orig_map)
{

	TIMING_DECLARE(TP1)
	TIMING_BEGIN(TP1)
	Detc_Edge2D* detector = reinterpret_cast<Detc_Edge2D*>(handle);
	detector->setPtArray(input_data);
	detector->setNPlane(plane_normal);
	detector->initImgVoxelGrid(1, 1);
	detector->setVoxelSize(voxel_size);
	detector->setOutputPath(output_path);
	if (have_center_input)
	{
		detector->setPointsCenter(center);
	}

	bool isDetcInit = detector->init();
	if (!isDetcInit) {
		detector->clear();
		return false;
	}
	TIMING_END_ms("isDetcInit", TP1)

	TIMING_BEGIN(TP1)
	detector->detect();
	TIMING_END_ms("detect", TP1)

	TIMING_BEGIN(TP1)
	detector->getOrigMapOfContours(orig_map);
	TIMING_END_ms("getOrigMapOfContours", TP1)
	return true;
}

bool Features_getOrigMapOfContours3D(bool have_center_input, const std::string output_path, void* handle, const Point3f plane_normal, const Point3f center, \
	Point3fArray* input_data, const double voxel_size, Point3fArray& orig_map)
{

	TIMING_DECLARE(TP1)
	TIMING_BEGIN(TP1)
	Detc_Edge2D* detector = reinterpret_cast<Detc_Edge2D*>(handle);
	detector->setPtArray(input_data);
	detector->setNPlane(plane_normal);
	detector->initImgVoxelGrid(1, 1);
	detector->setVoxelSize(voxel_size);
	detector->setOutputPath(output_path);
	if (have_center_input)
	{
		detector->setPointsCenter(center);
	}

	bool isDetcInit = detector->init();
	if (!isDetcInit) {
		detector->clear();
		return false;
	}
	TIMING_END_ms("isDetcInit", TP1)

	TIMING_BEGIN(TP1)
	detector->detect_bn();
	TIMING_END_ms("detect", TP1)

	TIMING_BEGIN(TP1)
	detector->getOrigMapOfContours(orig_map);
	TIMING_END_ms("getOrigMapOfContours", TP1)
	return true;
}

bool Features_getOrigMapOfContours3D(const std::string output_path, void* handle, const Point3f plane_normal, Point3fArray* input_data, const double voxel_size, Vector<Point3fArray>& orig_map)
{
	return (Features_getOrigMapOfContours3D(false, output_path, handle, plane_normal, {}, input_data, voxel_size, orig_map));
}

bool Features_getOrigMapOfContours3D(void* handle, const Point3f plane_normal, Point3fArray* input_data, const double voxel_size, Vector<Point3fArray>& orig_map)
{
	return (Features_getOrigMapOfContours3D(false, "\\", handle, plane_normal, {},input_data, voxel_size, orig_map));
}


bool Features_getOrigMapOfContours3D(void* handle, const Point3f plane_normal, const Point3f center, \
	Point3fArray* input_data, const double voxel_size, Vector<Point3fArray>& orig_map)
{
	return (Features_getOrigMapOfContours3D(true, "\\", handle, plane_normal, center, input_data, voxel_size, orig_map));
}

bool Features_getOrigMapOfContours3D(const std::string output_path, void* handle, const Point3f plane_normal, const Point3f center, \
	Point3fArray* input_data, const double voxel_size, Vector<Point3fArray>& orig_map)
{
	return (Features_getOrigMapOfContours3D(true, output_path, handle, plane_normal, center, input_data, voxel_size, orig_map));
}

bool Features_getOrigMapOfContours3D(const std::string output_path, void* handle, const Point3f plane_normal, Point3fArray* input_data, const double voxel_size, Point3fArray& orig_map)
{
	return (Features_getOrigMapOfContours3D(false, output_path, handle, plane_normal, {}, input_data, voxel_size, orig_map));
}

bool Features_getOrigMapOfContours3D(void* handle, const Point3f plane_normal, Point3fArray* input_data, const double voxel_size, Point3fArray& orig_map)
{
	return (Features_getOrigMapOfContours3D(false, "\\", handle, plane_normal, {}, input_data, voxel_size, orig_map));
}


bool Features_getOrigMapOfContours3D(void* handle, const Point3f plane_normal, const Point3f center, \
	Point3fArray* input_data, const double voxel_size, Point3fArray& orig_map)
{
	return (Features_getOrigMapOfContours3D(true, "\\", handle, plane_normal, center, input_data, voxel_size, orig_map));
}

bool Features_getOrigMapOfContours3D(const std::string output_path, void* handle, const Point3f plane_normal, const Point3f center, \
	Point3fArray* input_data, const double voxel_size, Point3fArray& orig_map)
{
	return (Features_getOrigMapOfContours3D(true, output_path, handle, plane_normal, center, input_data, voxel_size, orig_map));
}


bool Features_getOrigMapOfContours2D_bn(bool have_center_input, std::string output_path, void* handle, const Point3f plane_normal, const Point3f center, \
	Point3fArray* input_data, const double voxel_size, Point2fArray& contours)
{
	TIMING_DECLARE(TP1)
	TIMING_BEGIN(TP1)
	Detc_Edge2D* detector = reinterpret_cast<Detc_Edge2D*>(handle);
	//	new Detc_Edge2D(input_data, plane_normal, voxel_size);
	detector->setPtArray(input_data);
	detector->setNPlane(plane_normal);
	detector->initImgVoxelGrid(1, 1);
	detector->setVoxelSize(voxel_size);
	detector->setOutputPath(output_path);
	if (have_center_input)
	{
		detector->setPointsCenter(center);
	}
	bool isDetcInit = detector->init();
	if (!isDetcInit) {
		detector->clear();
		return false;
	}
	TIMING_END_ms("isDetcInit", TP1)

	TIMING_BEGIN(TP1)
	detector->detect_bn();
	TIMING_END_ms("detect", TP1)

	TIMING_BEGIN(TP1)
	detector->get2DContours_bn(contours);
	TIMING_END_ms("getOrigMapOfContours", TP1)
	return true;
}


bool Features_getOrigMapOfContours2D_bn(std::string output_path, void* handle, const Point3f plane_normal, \
	Point3fArray* input_data, const double voxel_size, Point2fArray& contours)
{
	return (Features_getOrigMapOfContours2D_bn(false, output_path, handle, plane_normal, {}, input_data, voxel_size, contours));
}
bool Features_getOrigMapOfContours2D(void* handle, const Point3f plane_normal, Point3fArray* input_data, const double voxel_size, \
	Point2fArray& contours)
{
	return (Features_getOrigMapOfContours2D_bn(false, "\\", handle, plane_normal, {}, input_data, voxel_size, contours));
}

bool Features_getOrigMapOfContours2D_bn(void* handle, const Point3f plane_normal, const Point3f center, \
	Point3fArray* input_data, const double voxel_size, Point2fArray& contours)
{
	return (Features_getOrigMapOfContours2D_bn(true, "\\", handle, plane_normal, center, input_data, voxel_size, contours));

}

bool Features_getOrigMapOfContours2D_bn(std::string output_path, void* handle, const Point3f plane_normal, const Point3f center, \
	Point3fArray* input_data, const double voxel_size, Point2fArray& contours)
{
	return (Features_getOrigMapOfContours2D_bn(true, output_path, handle, plane_normal, center, input_data, voxel_size, contours));

}
