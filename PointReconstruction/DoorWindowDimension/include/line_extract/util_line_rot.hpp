#ifndef _UTIL_LINE_ROT_HPP_
#define _UTIL_LINE_ROT_HPP_

#include <string>
#include <iostream>
#include "log.h"
#include "pxl_struct_inf.h"
#include "util_math.hpp"

namespace util_line_rot
{
	/**
	* \brief transfer 2D point to 3D point according to original center and four-element rotation matrix
	* @param [in] isNeedRot,  true will do rotation
	* @param [in] point_2d,  point in 2D space
	* @param [in] rot_mat_4e,  four-element rotation matrix
	* @param [in] orig_center,  orignal rotation center
	* return = 0 success, else failed
	* */
	static inline ModuleStruct::Point3f point2d_to_3d(bool isNeedRot, const ModuleStruct::Point2f point_2d, ModuleStruct::CMat rot_mat, const ModuleStruct::Point3f orig_center)
	{
		ModuleStruct::Point3f orig_point;
		ModuleStruct::Point3f point;
		point.x = point_2d.x;
		point.y = point_2d.y;
		point.z = 0.f;

		if (isNeedRot)
		{
			orig_point = ModuleStruct::Point3f(
				rot_mat.at<float>(0, 0) * point.x + rot_mat.at<float>(0, 1) * point.y + rot_mat.at<float>(0, 2) * point.z,
				rot_mat.at<float>(1, 0) * point.x + rot_mat.at<float>(1, 1) * point.y + rot_mat.at<float>(1, 2) * point.z,
				rot_mat.at<float>(2, 0) * point.x + rot_mat.at<float>(2, 1) * point.y + rot_mat.at<float>(2, 2) * point.z);

			orig_point = orig_point + orig_center;
		}
		else
		{
			orig_point = point + orig_center;
		}
		return orig_point;
	}

	static inline bool line2d_to_3d(     ///T
		const ModuleStruct::Point2fArray input_data, 
		const ModuleStruct::Point3f origin_normal, 
		const ModuleStruct::Point3f origin_center, 
		std::vector<Line2DSegOutItemDebug> line_seg2D, 
		ModuleStruct::Vector<Line3DSegOutItemDebug> &line_seg3D)
	{
		ModuleStruct::Point3f nz(0.0f, 0.0f, 1.0f);
		bool isNeedRot = !Util_Math::vec3_are_same(origin_normal, nz); // is need rotation
		ModuleStruct::CMat rot_mat(3, 3, CV_32F);

		if (isNeedRot)
		{
			rot_mat = Util_Math::CreateRotationMat4E(nz, origin_normal);
		}
		line_seg3D.clear();
		line_seg3D.resize(line_seg2D.size());
		for (unsigned int i = 0; i < line_seg2D.size(); i++) 
		{
			Line2DSegOutItemDebug* line2d_it = &line_seg2D[i];
			Line3DSegOutItemDebug* line3d_it = &line_seg3D[i];

			line3d_it->line_seg_start = point2d_to_3d(isNeedRot, line2d_it->line_seg_start, rot_mat, origin_center);
			line3d_it->line_seg_end = point2d_to_3d(isNeedRot, line2d_it->line_seg_end, rot_mat, origin_center);
			line3d_it->line_center = point2d_to_3d(isNeedRot, line2d_it->line_center, rot_mat, origin_center);
			line3d_it->line_direction =  line3d_it->line_seg_start - line3d_it->line_seg_end;
			Util_Math::vec3_normalize(line3d_it->line_direction);

			ModuleStruct::Point2fArray line_pt;
			line_pt.clear();
			line_pt.resize(line2d_it->points.size());
			line3d_it->points.clear();
			line3d_it->points.resize(line2d_it->points.size());
			
			for (int j = 0; j < line_pt.size(); j++)
			{
				unsigned int point_idx = line2d_it->points[j];
				line_pt[j] = input_data[point_idx];
				line3d_it->points[j] = point2d_to_3d(isNeedRot, line_pt[j], rot_mat, origin_center);
			}
		}

		return false;
	
	}

}


#endif /*_UTIL_LINE_TEST_HPP_*/
