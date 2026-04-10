#ifndef _UTIL_EDGE2D_TEST_HPP_
#define _UTIL_EDGE2D_TEST_HPP_

#include <string>
#include <iostream>
#include "in_out_data.hpp"
#include "util_opencv.hpp"
#include "util_math.hpp"
#include "DataStruct.h"

static inline bool ClearSumsAst(SumforCovariance* sums) {
	sums->sum_x = 0.0;
	sums->sum_y = 0.0;
	sums->sum_z = 0.0;
	sums->sum_xx = 0.0;
	sums->sum_yy = 0.0;
	sums->sum_zz = 0.0;
	sums->sum_xy = 0.0;
	sums->sum_xz = 0.0;
	sums->sum_yz = 0.0;
	return true;
}

template<typename T>
static inline bool PushPointAst(SumforCovariance* sums_to, T point) {
	sums_to->sum_x += point.x; sums_to->sum_y += point.y; sums_to->sum_z += point.z;
	sums_to->sum_xx += point.x * point.x; sums_to->sum_yy += point.y * point.y; sums_to->sum_zz += point.z * point.z;
	sums_to->sum_xy += point.x * point.y; sums_to->sum_xz += point.x * point.z; sums_to->sum_yz += point.y * point.z;
	return true;
}

static inline void ComputeAst(unsigned int point_cnt, SumforCovariance sums, 
	ModuleStruct::Point3f& plane_normal, ModuleStruct::Point3f& plane_center, float& eigen_mse)
{
	if (point_cnt <= 0) return;

	double point_num_inverse = (double)1.0 / point_cnt;

	//plane center
	plane_center.x = (float)(sums.sum_x * point_num_inverse);
	plane_center.y = (float)(sums.sum_y * point_num_inverse);
	plane_center.z = (float)(sums.sum_z * point_num_inverse);

	//calculate covariance matrix
	double cov_mat_element[6] = { sums.sum_xx - sums.sum_x * plane_center.x,sums.sum_xy - sums.sum_x * plane_center.y,
		sums.sum_xz - sums.sum_x * plane_center.z,sums.sum_yy - sums.sum_y * plane_center.y,
		sums.sum_yz - sums.sum_y * plane_center.z,sums.sum_zz - sums.sum_z * plane_center.z };


#ifdef FAST_COMPUTE_EIGEN
	double eig_val[3], eig_vec[3];

	MathOperation::ComputeFastEigenParallel(&cov_mat_element[0], &eig_val[0], &eig_vec[0]);
	plane_normal.x = eig_vec[0];
	plane_normal.y = eig_vec[1];
	plane_normal.z = eig_vec[2];

	//plane mean squared error
	eigen_mse = std::fabs((float)(eig_val[2] * point_num_inverse));
#else
	double cov_mat_arr[3][3] = { { cov_mat_element[0],cov_mat_element[1],cov_mat_element[2] },
	{ cov_mat_element[1], cov_mat_element[3], cov_mat_element[4] },
	{ cov_mat_element[2], cov_mat_element[4], cov_mat_element[5] } };

	ModuleStruct::CMat cov_mat(3, 3, CV_64F, cov_mat_arr);
	cov_mat *= point_num_inverse;

	ModuleStruct::CMat eig_val_mat, eig_vec_mat;

	cv::eigen(cov_mat, eig_val_mat, eig_vec_mat);

	plane_normal.x = (float)(eig_vec_mat.at<double>(2, 0));
	plane_normal.y = (float)(eig_vec_mat.at<double>(2, 1));
	plane_normal.z = (float)(eig_vec_mat.at<double>(2, 2));
	//plane mean squared error
	//voxel_grid->plane_mse = std::fabs((float)(eig_val_mat.at<double>(2) * point_num_inverse));
	eigen_mse = std::fabs((float)(eig_val_mat.at<double>(2) * point_num_inverse));
#endif

}

namespace util_edge2d_test
{
	/**
	* \brief save edge xyz output to file
	* @param [in] sub_path,  out put path, with endof "/" or"\\"
	* @param [in] sub_file_name, input file name
	* @param [in] delimiter, delimiter of save data
	* @param [in] file_type,  save file suffix
	* @param [in] contours_2d, contours  in 2d space
	* return > = 0 success, else failed
	* */
	static inline int save_edge2d_xy(const std::string sub_path, const std::string sub_file_name, const std::string delimiter, \
		const std::string file_type, ModuleStruct::Vector<ModuleStruct::Point2fArray>&contours_2d)
	{
		if (IOData::createDirectory(sub_path))
		{
			//log_error("createDirectory %s failed", sub_path.c_str());
			return -1;
		}

		for (unsigned int i = 0; i < contours_2d.size(); i++)
		{
			std::stringstream edge_id;
			edge_id << i;
			std::string whole_path = sub_path + sub_file_name + "_" + edge_id.str() + file_type;
			bool rtn = FeaturesIO::SavePoint2fDataWithDelimiter(whole_path, delimiter, contours_2d[i]);
			if (!rtn)
			{
				//log_error("whole_path %s save_edge2d_xyz edge %d return error", whole_path.c_str(), i);
				return -1;
			}
		}
		return 0;
	}


	/**
	* \brief save edge xyz output to file with no rotation
	* @param [in] sub_path,  out put path, with endof "/" or"\\"
	* @param [in] sub_file_name, input file name
	* @param [in] delimiter, delimiter of save data
	* @param [in] file_type,  save file suffix
	* @param [in] contours_2d, contours  in 2d space
	* @param [in] center, contours  origin center
	* return > = 0 success, else failed
	* */
	static inline int save_edge2d_xyz(const std::string sub_path, const std::string sub_file_name, const std::string delimiter, \
		const std::string file_type, ModuleStruct::Vector<ModuleStruct::Point2fArray> contours_2d, ModuleStruct::Point3f center)
	{
		if (IOData::createDirectory(sub_path))
		{
			//log_error("createDirectory %s failed", sub_path.c_str());
			return -1;
		}

		for (unsigned int i = 0; i < contours_2d.size(); i++)
		{
			ModuleStruct::Point3fArray contour_3d;
			contour_3d.clear(); contour_3d.resize(contours_2d[i].size());
			for (int j = 0; j < contour_3d.size(); j++)
			{
				contour_3d[j].x = contours_2d[i][j].x + center.x;
				contour_3d[j].y = contours_2d[i][j].y + center.y;
				contour_3d[j].z = center.z;
			}
			std::stringstream edge_id;
			edge_id << i;
			std::string whole_path = sub_path + sub_file_name + "_" + edge_id.str() + file_type;
			bool rtn = IOData::SavePoint3fDataWithDelimiter(whole_path, delimiter, contour_3d);
			if (!rtn)
			{
				//log_error("whole_path %s save_edge2d_xyz edge %d return error", whole_path.c_str(), i);
				return -1;
			}
		}
		return 0;
	}

	/**
	* \brief save edge xyz array output to file
	* @param [in] sub_path,  out put path, with endof "/" or"\\"
	* @param [in] sub_file_name, input file name
	* @param [in] delimiter, delimiter of save data
	* @param [in] file_type,  save file suffix
	* @param [in] contours2d_array, contours array in 2d space
	* return > = 0 success, else failed
	* */
	static inline int save_edge2d_array(const std::string sub_path, const std::string sub_file_name, const std::string delimiter, \
		const std::string file_type, ModuleStruct::Vector<ModuleStruct::Vector<ModuleStruct::Point2fArray>> contours2d_array)
	{
		if (IOData::createDirectory(sub_path))
		{
			//log_error("createDirectory %s failed", sub_path.c_str());
			return -1;
		}
		for (unsigned int i = 0; i < contours2d_array.size(); i++)
		{
			ModuleStruct::Vector<ModuleStruct::Point2fArray> contours_2d =  contours2d_array[i];
			std::stringstream contour_id;
			contour_id << i;
			for (unsigned int j = 0; j < contours_2d.size(); j++)
			{
				std::stringstream edge_id;
				edge_id << j;
				std::string whole_path = sub_path + sub_file_name + contour_id.str() +"_" + edge_id.str() + file_type;
				bool rtn = FeaturesIO::SavePoint2fDataWithDelimiter(whole_path, delimiter, contours_2d[j]);
				if (!rtn)
				{
					//log_error("whole_path %s save_edge2d_array edge %d return error", whole_path.c_str(), j);
					return -1;
				}
			}
		}
		return 0;
	}


	/**
	* \brief save edge xyz array output to file
	* @param [in] sub_path,  out put path, with endof "/" or"\\"
	* @param [in] sub_file_name, input file name
	* @param [in] delimiter, delimiter of save data
	* @param [in] file_type,  save file suffix
	* @param [in] contours2d_array, contours array in 2d space
	* return > = 0 success, else failed
	* */
	static inline int save_edge3d_array(const std::string sub_path, const std::string sub_file_name, const std::string delimiter, \
		const std::string file_type, ModuleStruct::Vector<ModuleStruct::Vector<ModuleStruct::Point3fArray>> contours3d_array)
	{
		if (IOData::createDirectory(sub_path))
		{
			//log_error("createDirectory %s failed", sub_path.c_str());
			return -1;
		}
		for (unsigned int i = 0; i < contours3d_array.size(); i++)
		{
			ModuleStruct::Vector<ModuleStruct::Point3fArray> contours_3d = contours3d_array[i];
			std::stringstream contour_id;
			contour_id << i;
			for (unsigned int j = 0; j < contours_3d.size(); j++)
			{
				std::stringstream edge_id;
				edge_id << j;
				std::string whole_path = sub_path + sub_file_name + contour_id.str() +"_" + edge_id.str() + file_type;
				bool rtn = IOData::SavePoint3fDataWithDelimiter(whole_path, delimiter, contours_3d[j]);
				if (!rtn)
				{
					//log_error("whole_path %s save_edge3d_array edge %d return error", whole_path.c_str(), j);
					return -1;
				}
			}
		}
		return 0;
	}

	static inline ModuleStruct::Point3f point_rot(ModuleStruct::CMat rot_mat, ModuleStruct::Point3f &pts)
	{
		ModuleStruct::Point3f c_pts = ModuleStruct::Point3f(
			rot_mat.at<float>(0, 0) * pts.x + rot_mat.at<float>(0, 1) * pts.y + rot_mat.at<float>(0, 2) * pts.z,
			rot_mat.at<float>(1, 0) * pts.x + rot_mat.at<float>(1, 1) * pts.y + rot_mat.at<float>(1, 2) * pts.z,
			rot_mat.at<float>(2, 0) * pts.x + rot_mat.at<float>(2, 1) * pts.y + rot_mat.at<float>(2, 2) * pts.z);
		return c_pts;
	}



	static inline void rot_test(bool return_back, ModuleStruct::Point3fArray input_data, ModuleStruct::Point3f normal, ModuleStruct::Point3f center, ModuleStruct::Point3fArray& pts)
	{
	   //parallevel to nz
		ModuleStruct::Point3f m_nPlane = static_cast<ModuleStruct::Point3f>(Util_Math::vec3_normalize(normal));
		ModuleStruct::Point3f nz(0.0f, 0.0f, 1.0f);
		// check if nPlane is parallel to nz
		// if parallel, no need
		bool isNeedRot = !Util_Math::vec3_are_same(m_nPlane, nz); // is need rotation
		//Point3fArray pts;
		pts.clear(); pts.resize(input_data.size());
		int nPts = static_cast<int>(pts.size());
		// translate and rotate
		if (isNeedRot) {

			ModuleStruct::CMat rot_mat(3, 3, CV_32F);
			rot_mat = Util_Math::CreateRotationMat4E(m_nPlane, nz);
#pragma omp parallel for
			for (int i = 0; i < pts.size(); i++) {
				pts[i] = input_data[i];
				pts[i] -= center;
			}
#pragma omp parallel for
			for (int i = 0; i < pts.size(); i++) {
				pts[i] = point_rot(rot_mat, pts[i]);
			}

		}

		
		if (isNeedRot)
		{
			ModuleStruct::CMat rot_mat(3, 3, CV_32F);
			rot_mat = Util_Math::CreateRotationMat4E(nz, m_nPlane);
			if (return_back)
			{
#pragma omp parallel for
			 for (int i = 0; i < nPts; i++) {
					pts[i] = point_rot(rot_mat, pts[i]);
				}
			}

#pragma omp parallel for
			for (int i = 0; i < pts.size(); i++) {
				pts[i] += center;
			}

		}
	}


	static inline void compute_points_normal(ModuleStruct::Point3fArray input_data, ModuleStruct::Point3f &normal, ModuleStruct::Point3f &center)
	{
		SumforCovariance sums;
		ClearSumsAst(&sums);
		for (int i = 0; i < input_data.size(); i++)
		{
			PushPointAst(&sums,input_data[i]);
		}
		float eigen_mse;
		ComputeAst(input_data.size(), sums, normal, center, eigen_mse);
		
	}

	static inline void cv_compute_points_normal(ModuleStruct::Point3fArray input_data, ModuleStruct::Point3f& normal, ModuleStruct::Point3f& center)
	{
		ModuleStruct::CMat pointMatrix(input_data.size(),3, CV_64FC1);

		for (int i = 0; i < input_data.size(); i++)
		{
			ModuleStruct::interim_value_type* ptr = pointMatrix.ptr<ModuleStruct::interim_value_type>(i);
			ptr[0] = input_data[i].x, ptr[1] = input_data[i].y, ptr[2] = input_data[i].z;
		}

		cv::Mat mean;
		cv::Mat points_covariance_matrix;
		Util_CV::covariance_matrix(pointMatrix, points_covariance_matrix, mean, input_data.size());
		ModuleStruct::Point3f plane_center;
		center.x = (float)(mean.at<double>(0, 0));
		center.y = (float)(mean.at<double>(0, 1));
		center.z = (float)(mean.at<double>(0, 2));

		ModuleStruct::CMat eig_val_mat, eig_vec_mat;
		Util_CV::eigen(points_covariance_matrix, eig_val_mat, eig_vec_mat);
		normal.x = (float)(eig_vec_mat.at<double>(2, 0));
		normal.y = (float)(eig_vec_mat.at<double>(2, 1));
		normal.z = (float)(eig_vec_mat.at<double>(2, 2));
	}

}


#endif /*_UTIL_EDGE2D_TEST_HPP_*/
