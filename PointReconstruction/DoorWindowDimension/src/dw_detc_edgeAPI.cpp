
#include "edge_dection/detc_macro.h"
#include "log.h"
#include <iostream>
#include "edge_dection/detc_edge_api.h"
#include "edge_dection/detc_edge2d.h"

using namespace ModuleStruct;

void* DW_Features_EdgeDetcCreateHandle()
{
	return (new DW_Detc_Edge2D());
}

void DW_Features_EdgeDetcDestroyHandle(void** handle)
{
	if (handle)
	{
		DW_Detc_Edge2D* ptr = reinterpret_cast<DW_Detc_Edge2D*>(*handle);
		delete ptr;
		ptr = nullptr;
	}
}


bool DW_Features_getOrigMapOfContours2D(
	bool have_center_input, 
	std::string output_path, 
	void* handle, 
	const Point3f plane_normal, 
	const Point3f center,
	Point3fArray* input_data,///i 
	const double voxel_size, //16
	std::vector<Point2fArray>& contours
	)
{
	DW_Detc_Edge2D* detector = reinterpret_cast<DW_Detc_Edge2D*>(handle);
	//	new DW_Detc_Edge2D(input_data, plane_normal, voxel_size);
	detector->setPtArray(input_data);
	detector->setNPlane(plane_normal);
	detector->initImgVoxelGrid(1, 1);
	detector->setVoxelSize(voxel_size);//16
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
	detector->detect();
	detector->get2DContours(contours);
	return true;
}

bool DW_Features_getOrigMapOfContours2D(std::string output_path, void* handle, const Point3f plane_normal, \
	Point3fArray* input_data, const double voxel_size, std::vector<Point2fArray>& contours)
{
	return (DW_Features_getOrigMapOfContours2D(false, output_path, handle, plane_normal, {}, input_data, voxel_size, contours));
}
bool DW_Features_getOrigMapOfContours2D(void* handle, const Point3f plane_normal, Point3fArray* input_data, const double voxel_size, std::vector<Point2fArray>& contours)
{
	return (DW_Features_getOrigMapOfContours2D(false, "\\", handle, plane_normal, {}, input_data, voxel_size, contours));
}

bool DW_Features_getOrigMapOfContours2D(void* handle, const Point3f plane_normal, const Point3f center, \
	Point3fArray* input_data, const double voxel_size, std::vector<Point2fArray>& contours)
{
	return (DW_Features_getOrigMapOfContours2D(true, "\\", handle, plane_normal, center, input_data, voxel_size, contours));

}

bool DW_Features_getOrigMapOfContours2D(std::string output_path, void* handle, const Point3f plane_normal, const Point3f center, \
	Point3fArray* input_data, const double voxel_size, std::vector<Point2fArray>& contours)
{
	return (DW_Features_getOrigMapOfContours2D(true, output_path, handle, plane_normal, center, input_data, voxel_size, contours));

}

bool DW_Features_getOrigMapOfContours3D(bool have_center_input, const std::string output_path, void* handle, const Point3f plane_normal, const Point3f center, \
	Point3fArray* input_data, const double voxel_size, Vector<Point3fArray>& orig_map)
{

	DW_Detc_Edge2D* detector = reinterpret_cast<DW_Detc_Edge2D*>(handle);
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
	detector->detect();
	detector->getOrigMapOfContours(orig_map);
	return true;
}

bool DW_Features_getOrigMapOfContours3D(bool have_center_input,
									 const std::string output_path, 
									 void* handle, const Point3f plane_normal, const Point3f center,
									 Point3fArray* input_data, const double voxel_size, Point3fArray& orig_map)
{

	DW_Detc_Edge2D* detector = reinterpret_cast<DW_Detc_Edge2D*>(handle);
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
	detector->detect_bn();
	detector->getOrigMapOfContours(orig_map);
	return true;
}

bool DW_Features_getOrigMapOfContours3D(bool have_center_input, const std::string output_path, void* handle, const Point3f& plane_normal, const Point3f& center, \
	const Point3fArray& input_data, const double voxel_size, Vector<Point3fArray>& orig_map)
{
	DW_Detc_Edge2D* detector = reinterpret_cast<DW_Detc_Edge2D*>(handle);
	Point3fArray* pj = const_cast<Point3fArray*>(&input_data);
	detector->setPtArray(pj);
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
	detector->detect();
	detector->getOrigMapOfContours(orig_map);
	
	return true;
}

bool DW_Features_getOrigMapOfContours3D(const std::string output_path, void* handle, const Point3f plane_normal, Point3fArray* input_data, const double voxel_size, Vector<Point3fArray>& orig_map)
{
	return (DW_Features_getOrigMapOfContours3D(false, output_path, handle, plane_normal, {}, input_data, voxel_size, orig_map));
}

bool DW_Features_getOrigMapOfContours3D(void* handle, const Point3f plane_normal, Point3fArray* input_data, const double voxel_size, Vector<Point3fArray>& orig_map)
{
	return (DW_Features_getOrigMapOfContours3D(false, "\\", handle, plane_normal, {}, input_data, voxel_size, orig_map));
}


bool DW_Features_getOrigMapOfContours3D(void* handle, const Point3f plane_normal, const Point3f center, \
	Point3fArray* input_data, const double voxel_size, Vector<Point3fArray>& orig_map)
{
	return (DW_Features_getOrigMapOfContours3D(true, "\\", handle, plane_normal, center, input_data, voxel_size, orig_map));
}

bool DW_Features_getOrigMapOfContours3D(void* handle, const ModuleStruct::Point3f& plane_normal, const ModuleStruct::Point3f& plane_center, const ModuleStruct::Point3fArray& input_data, const double voxel_size, \
	Vector<Point3fArray>& orig_map)
{
	return (DW_Features_getOrigMapOfContours3D(true, "\\", handle, plane_normal, plane_center, input_data, voxel_size, orig_map));
}

bool DW_Features_getOrigMapOfContours3D(const std::string output_path, void* handle, const Point3f plane_normal, const Point3f center, \
	Point3fArray* input_data, const double voxel_size, Vector<Point3fArray>& orig_map)
{
	return (DW_Features_getOrigMapOfContours3D(true, output_path, handle, plane_normal, center, input_data, voxel_size, orig_map));
}

bool DW_Features_getOrigMapOfContours3D(const std::string output_path, void* handle, const Point3f plane_normal, Point3fArray* input_data, const double voxel_size, Point3fArray& orig_map)
{
	return (DW_Features_getOrigMapOfContours3D(false, output_path, handle, plane_normal, {}, input_data, voxel_size, orig_map));
}

bool DW_Features_getOrigMapOfContours3D(void* handle, const Point3f plane_normal, Point3fArray* input_data, const double voxel_size, Point3fArray& orig_map)
{
	return (DW_Features_getOrigMapOfContours3D(false, "\\", handle, plane_normal, {}, input_data, voxel_size, orig_map));
}


bool DW_Features_getOrigMapOfContours3D(void* handle, const Point3f plane_normal, const Point3f center, \
	Point3fArray* input_data, const double voxel_size, Point3fArray& orig_map)
{
	return (DW_Features_getOrigMapOfContours3D(true, "\\", handle, plane_normal, center, input_data, voxel_size, orig_map));
}

bool DW_Features_getOrigMapOfContours3D(const std::string output_path, void* handle, const Point3f plane_normal, const Point3f center, \
	Point3fArray* input_data, const double voxel_size, Point3fArray& orig_map)
{
	return (DW_Features_getOrigMapOfContours3D(true, output_path, handle, plane_normal, center, input_data, voxel_size, orig_map));
}


bool DW_Features_getOrigMapOfContours2D_bn(bool have_center_input, std::string output_path, void* handle, const Point3f plane_normal, const Point3f center, \
	Point3fArray* input_data, const double voxel_size, Point2fArray& contours)
{
	DW_Detc_Edge2D* detector = reinterpret_cast<DW_Detc_Edge2D*>(handle);
	//	new DW_Detc_Edge2D(input_data, plane_normal, voxel_size);
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
	detector->detect_bn();
	detector->get2DContours_bn(contours);
	return true;
}


bool DW_Features_getOrigMapOfContours2D_bn(std::string output_path, void* handle, const Point3f plane_normal, \
	Point3fArray* input_data, const double voxel_size, Point2fArray& contours)
{
	return (DW_Features_getOrigMapOfContours2D_bn(false, output_path, handle, plane_normal, {}, input_data, voxel_size, contours));
}
bool DW_Features_getOrigMapOfContours2D(void* handle, const Point3f plane_normal, Point3fArray* input_data, const double voxel_size, \
	Point2fArray& contours)
{
	return (DW_Features_getOrigMapOfContours2D_bn(false, "\\", handle, plane_normal, {}, input_data, voxel_size, contours));
}

bool DW_Features_getOrigMapOfContours2D_bn(void* handle, const Point3f plane_normal, const Point3f center, \
	Point3fArray* input_data, const double voxel_size, Point2fArray& contours)
{
	return (DW_Features_getOrigMapOfContours2D_bn(true, "\\", handle, plane_normal, center, input_data, voxel_size, contours));

}

bool DW_Features_getOrigMapOfContours2D_bn(std::string output_path, void* handle, const Point3f plane_normal, const Point3f center, \
	Point3fArray* input_data, const double voxel_size, Point2fArray& contours)
{
	return (DW_Features_getOrigMapOfContours2D_bn(true, output_path, handle, plane_normal, center, input_data, voxel_size, contours));

}
