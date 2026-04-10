#ifndef _DETC_EDGE_API_H_
#define _DETC_EDGE_API_H_

#include "ModuleStruct.hpp"

#if defined WIN32 || defined _WIN32 || defined __CYGWIN__
#ifdef DETC_EDGE_MODULE
#define DETC_EDGE_EXPORT __declspec(dllexport)
#else
#define DETC_EDGE_EXPORT __declspec(dllimport)
#endif
#else
#define DETC_EDGE_EXPORT
#endif
 
/**
* \brief  Interface of edge_detc , create instance of edge_detc
*/
DETC_EDGE_EXPORT void* Features_EdgeDetcCreateHandle();

/**
* \brief  Interface of edge_detc, distroy instance of edge_detc
* @param [in]  void** handle, edge_detc handle
*/
DETC_EDGE_EXPORT void Features_EdgeDetcDestroyHandle(void** handle);

/**
* \brief API of detc_edge2d output, get original map of contours in 3D space
* @param [in]  void** handle, edge_detc handle
* @param [in] const Point3f plane_normal, input point cloud normal 
* @param [in] Point3fArray *input_data, input point cloud object
* @param [in] const double pixel_size,, voxel size ,if pixel_size == 0, use default value 
* @param [out] orig_map, contours in 3D space
*/
DETC_EDGE_EXPORT bool Features_getOrigMapOfContours3D(void* handle, const ModuleStruct::Point3f plane_normal, const ModuleStruct::Point3fArray *input_data, const double pixel_size, \
	ModuleStruct::Vector<ModuleStruct::Point3fArray> &orig_map);
DETC_EDGE_EXPORT bool Features_getOrigMapOfContours3D(void* handle, const ModuleStruct::Point3f plane_normal, const ModuleStruct::Point3fArray* input_data, const double pixel_size, \
	ModuleStruct::Point3fArray& orig_map);

/**
* \brief API of detc_edge2d output, get original map of contours in 3D space
* @param [in]  output_path,  output path for log or debug information
* @param [in]  void** handle, edge_detc handle
* @param [in] const Point3f plane_normal, input point cloud normal
* @param [in] Point3fArray *input_data, input point cloud object
* @param [in] const double pixel_size,, voxel size ,if pixel_size == 0, use default value
* @param [out] orig_map, contours in 3D space
*/
DETC_EDGE_EXPORT bool Features_getOrigMapOfContours3D(const std::string output_path, void* handle, const ModuleStruct::Point3f plane_normal, ModuleStruct::Point3fArray* input_data, \
	const double pixel_size, ModuleStruct::Vector<ModuleStruct::Point3fArray>& orig_map);
DETC_EDGE_EXPORT bool Features_getOrigMapOfContours3D(const std::string output_path, void* handle, const ModuleStruct::Point3f plane_normal, ModuleStruct::Point3fArray* input_data, \
	const double pixel_size, ModuleStruct::Point3fArray& orig_map);

/**
* \brief API of detc_edge2d output, get original map of contours in 3D space
* @param [in]  void** handle, edge_detc handle
* @param [in] const Point3f plane_normal, input point cloud normal
* @param [in] const Point3f center, input point cloud center
* @param [in] Point3fArray *input_data, input point cloud object
* @param [in] const double pixel_size,, voxel size ,if pixel_size == 0, use default value
* @param [out] orig_map, contours in 3D space
*/
DETC_EDGE_EXPORT bool Features_getOrigMapOfContours3D(void* handle, const ModuleStruct::Point3f plane_normal, const ModuleStruct::Point3f center, \
	const ModuleStruct::Point3fArray* input_data, const double pixel_size, ModuleStruct::Vector<ModuleStruct::Point3fArray>& orig_map);
DETC_EDGE_EXPORT bool Features_getOrigMapOfContours3D(void* handle, const ModuleStruct::Point3f plane_normal, const ModuleStruct::Point3f center, \
	const ModuleStruct::Point3fArray* input_data, const double pixel_size, ModuleStruct::Point3fArray& orig_map);

/**
* \brief API of detc_edge2d output, get original map of contours in 3D space
* @param [in]  output_path,  output path for log or debug information
* @param [in]  void** handle, edge_detc handle
* @param [in] const Point3f plane_normal, input point cloud normal
* @param [in] const Point3f center, input point cloud center
* @param [in] Point3fArray *input_data, input point cloud object
* @param [in] const double pxl_size,, pixel size ,if pixel_size == 0, use default value
* @param [out] orig_map, contours in 3D space
*/
DETC_EDGE_EXPORT bool Features_getOrigMapOfContours3D(const std::string output_path, void* handle, const ModuleStruct::Point3f plane_normal, const ModuleStruct::Point3f center, \
	ModuleStruct::Point3fArray* input_data, const double pxl_size, ModuleStruct::Vector<ModuleStruct::Point3fArray>& orig_map);
DETC_EDGE_EXPORT bool Features_getOrigMapOfContours3D(const std::string output_path, void* handle, const ModuleStruct::Point3f plane_normal, const ModuleStruct::Point3f center, \
	ModuleStruct::Point3fArray* input_data, const double pxl_size, ModuleStruct::Point3fArray& orig_map);

/**
* \brief API of detc_edge2d output, get original map of contours in 2D space
* @param [in]  void** handle, edge_detc handle
* @param [in] const Point3f plane_normal, input point cloud normal
* @param [in] Point3fArray *input_data, input point cloud object
* @param [in] const double pixel_size,, voxel size ,if pixel_size == 0, use default value
* @param [out] contours, contours in 2D space
*/
DETC_EDGE_EXPORT bool Features_getOrigMapOfContours2D(void* handle, const ModuleStruct::Point3f plane_normal, ModuleStruct::Point3fArray*input_data, const double pixel_size, \
	std::vector<ModuleStruct::Point2fArray> &contours);
DETC_EDGE_EXPORT bool Features_getOrigMapOfContours2D_bn(void* handle, const ModuleStruct::Point3f plane_normal, ModuleStruct::Point3fArray* input_data, const double pixel_size, \
	ModuleStruct::Point2fArray& contours);


/**
* \brief API of detc_edge2d output, get original map of contours in 2D space
* @param [in]  output_path,  output path for log or debug information
* @param [in]  void** handle, edge_detc handle
* @param [in] const Point3f plane_normal, input point cloud normal
* @param [in] Point3fArray *input_data, input point cloud object
* @param [in] const double pixel_size,, voxel size ,if pixel_size == 0, use default value
* @param [out] contours, contours in 2D space
*/
DETC_EDGE_EXPORT bool Features_getOrigMapOfContours2D(const std::string output_path, void* handle, const ModuleStruct::Point3f plane_normal, ModuleStruct::Point3fArray* input_data, \
	const double pixel_size, std::vector<ModuleStruct::Point2fArray>& contours);
DETC_EDGE_EXPORT bool Features_getOrigMapOfContours2D_bn(const std::string output_path, void* handle, const ModuleStruct::Point3f plane_normal, ModuleStruct::Point3fArray* input_data, \
	const double pixel_size, ModuleStruct::Point2fArray& contours);
DETC_EDGE_EXPORT bool Features_getOrigMapOfContours2D(const std::string output_path, void* handle, const ModuleStruct::Point3f plane_normal, ModuleStruct::Point3fArray* input_data, \
	const double pixel_size, std::vector<ModuleStruct::Point2fArray>& contours,
	std::unordered_map<int, std::vector<int>>& tree);
/**
* \brief API of detc_edge2d output, get original map of contours in 2D space
* @param [in]  void** handle, edge_detc handle
* @param [in] const Point3f plane_normal, input point cloud normal
* @param [in] const Point3f center, input point cloud center
* @param [in] Point3fArray *input_data, input point cloud object
* @param [in] const double pixel_size,, voxel size ,if pixel_size == 0, use default value
* @param [out] contours, contours in 2D space
*/
DETC_EDGE_EXPORT bool Features_getOrigMapOfContours2D(void* handle, const ModuleStruct::Point3f plane_normal, const ModuleStruct::Point3f center,\
	ModuleStruct::Point3fArray* input_data, const double pixel_size, std::vector<ModuleStruct::Point2fArray>& contours);

DETC_EDGE_EXPORT bool Features_getOrigMapOfContours2D_bn(void* handle, const ModuleStruct::Point3f plane_normal, const ModuleStruct::Point3f center, \
	ModuleStruct::Point3fArray* input_data, const double pixel_size, ModuleStruct::Point2fArray& contours);

/**
* \brief API of detc_edge2d output, get original map of contours in 2D space
* @param [in]  output_path,  output path for log or debug information
* @param [in]  void** handle, edge_detc handle
* @param [in] const Point3f plane_normal, input point cloud normal
* @param [in] const Point3f center, input point cloud center
* @param [in] Point3fArray *input_data, input point cloud object
* @param [in] const double pixel_size,, voxel size ,if pixel_size == 0, use default value
* @param [out] contours, contours in 2D space
*/
DETC_EDGE_EXPORT bool Features_getOrigMapOfContours2D(const std::string output_path, void* handle, const ModuleStruct::Point3f plane_normal, const ModuleStruct::Point3f center, \
	ModuleStruct::Point3fArray* input_data, const double pixel_size, std::vector<ModuleStruct::Point2fArray>& contours);
DETC_EDGE_EXPORT bool Features_getOrigMapOfContours2D_bn(const std::string output_path, void* handle, const ModuleStruct::Point3f plane_normal, const ModuleStruct::Point3f center, \
	ModuleStruct::Point3fArray* input_data, const double pixel_size, ModuleStruct::Point2fArray& contours);
//

#endif // _DETC_EDGE_API_H_
