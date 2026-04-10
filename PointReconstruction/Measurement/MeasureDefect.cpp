#pragma once
#include "MeasureDefect.h"
#include <omp.h>

MeasureDefect::MeasureDefect(bool defect_3d)
{
	mDefect3D = defect_3d;
	voxel_width = 25.f;
	voxel_height = 25.f;

	defect_threshold_in_mm = 5.f; //5mm

	pt2pt_distance = 10;
	min_pt_num_in_voxel = 10;// 0.01 * voxel_width * voxel_height / (pt2pt_distance * pt2pt_distance); // 1/100 of fully occupieed voxel
}

MeasureDefect::~MeasureDefect()
{}

void MeasureDefect::SetRulerSize(float ruler_width, float ruler_height)
{
	voxel_width = ruler_width;
	voxel_height = ruler_height;
}

void MeasureDefect::GetRulerSize(float& ruler_width, float& ruler_height)
{
	ruler_width = voxel_width;
	ruler_height = voxel_height;
}

void MeasureDefect::SetDefectThreshold(float defect_threshold)
{
	defect_threshold_in_mm = defect_threshold;
}

void MeasureDefect::GetDefectThreshold(float& defect_threshold)
{
	defect_threshold = defect_threshold_in_mm;
}


bool MeasureDefect::MeasureDefectFcn(const float* plane_normal, const float* plane_center, const std::vector<cv::Point3f>& plane_points,
	std::vector<std::pair<float, cv::Point3f>>& defect, ObstacleInfo* obstacleInfoPtr)
{
	/*Check if empty planes */
	if (plane_points.empty())
	{
#ifdef DEVELOPER_MODE
		std::cout << "MeasureDefect::MeasureDefectFcn(): Empty plane input" << std::endl;
#else
		//log_error("MeasureDefect: Empty plane input");
#endif
		return false;
	}

	// use refined plane_normal // added by wei.fan
	float refined_plane_normal[3];
	//if (obstacleInfoPtr != nullptr && typeid(*(obstacleInfoPtr->getClassType())) == typeid(WallObstacleInfo)) {
	//	WallObstacleInfo* wallObstacleInfo = static_cast<WallObstacleInfo*>(obstacleInfoPtr);
	//	if ((wallObstacleInfo->refinedPlaneNormalVector[wallObstacleInfo->origin_wall_id]).sum() > -1.5) {
	//		refined_plane_normal[0] = wallObstacleInfo->refinedPlaneNormalVector[wallObstacleInfo->origin_wall_id](0);
	//		refined_plane_normal[1] = wallObstacleInfo->refinedPlaneNormalVector[wallObstacleInfo->origin_wall_id](1);
	//		refined_plane_normal[2] = wallObstacleInfo->refinedPlaneNormalVector[wallObstacleInfo->origin_wall_id](2); 
	//	}
	//	else {
	//		refined_plane_normal[0] = plane_normal[0];
	//		refined_plane_normal[1] = plane_normal[1];
	//		refined_plane_normal[2] = plane_normal[2];
	//	}
	//}
	//else if (obstacleInfoPtr != nullptr && typeid(*(obstacleInfoPtr->getClassType())) == typeid(GroundObstacleInfo)) {
	//	GroundObstacleInfo* groundObstacleInfo = static_cast<GroundObstacleInfo*>(obstacleInfoPtr);
	//	if ((groundObstacleInfo->refinedPlaneNormal).sum() > -1.5) {
	//		refined_plane_normal[0] = groundObstacleInfo->refinedPlaneNormal(0);
	//		refined_plane_normal[1] = groundObstacleInfo->refinedPlaneNormal(1);
	//		refined_plane_normal[2] = groundObstacleInfo->refinedPlaneNormal(2);
	//	}
	//	else {
	//		refined_plane_normal[0] = plane_normal[0];
	//		refined_plane_normal[1] = plane_normal[1];
	//		refined_plane_normal[2] = plane_normal[2];
	//	}
	//}
	//else
	//{
	//	refined_plane_normal[0] = plane_normal[0];
	//	refined_plane_normal[1] = plane_normal[1];
	//	refined_plane_normal[2] = plane_normal[2];
	//}
	//cv::Point3f center, normals;
	//MathOperation::getNewNormalAndCenter(plane_points, center, normals);
	refined_plane_normal[0] = plane_normal[0];
	refined_plane_normal[1] = plane_normal[1];
	refined_plane_normal[2] = plane_normal[2];
	//refined_plane_normal[0] = normals.x;
	//refined_plane_normal[1] = normals.y;
	//refined_plane_normal[2] = normals.z;

	std::vector<cv::Point3f> centers = { cv::Point3f(plane_center[0],plane_center[1],plane_center[2])};
	//std::vector<cv::Point3f> centers = { center };
	/*Assume defect only for vertical walls*/
	/*1st rotation: rotate plane normal XY projection to Y axis */
	/*Ensure wall's bottom lateral is along X axis*/
	std::vector<cv::Point3f> rot_plane_points_to_y_coarse, rot_centers_to_y_coarse;
	float rot_plane_normal_to_y_coarse[3];
	float rotation_angle_to_y_coarse;
	MeasureBase::CalcAngleVectorXY2YAxis(refined_plane_normal, &rotation_angle_to_y_coarse); // plane_normal -> refined_plane_normal modified by wei.fan 
	cv::Mat rotation_matrix_to_y_coarse = MeasureBase::TranslateAngleAroundZ2RotationMatrix(rotation_angle_to_y_coarse);
	MeasureBase::RotateVector(refined_plane_normal, rotation_matrix_to_y_coarse, rot_plane_normal_to_y_coarse); // plane_normal -> refined_plane_normal modified by wei.fan 
	BOOL_FUNCTION_CHECK(MeasureBase::RotatePoints(plane_points, rotation_matrix_to_y_coarse, rot_plane_points_to_y_coarse));
	BOOL_FUNCTION_CHECK(MeasureBase::RotatePoints(centers, rotation_matrix_to_y_coarse, rot_centers_to_y_coarse));

	/*2nd rotation: refine normal aligning with Y axis*/
	std::vector<cv::Point3f> rot_plane_points_to_y_fine, rot_centers_to_y_fine;
	float rotation_angle_to_y_fine;
	MeasureBase::CalcAngleVectorYZ2YAxis(rot_plane_normal_to_y_coarse, &rotation_angle_to_y_fine);
	if(abs(refined_plane_normal[2]) < 0.2)
		rotation_angle_to_y_fine = 0;
	cv::Mat rotation_matrix_to_y_fine = MeasureBase::TranslateAngleAroundX2RotationMatrix(rotation_angle_to_y_fine);
	//cv::Mat rotation_matrix_to_y_fine = MeasureBase::CalRotationMatrixFromVectors(rot_plane_normal_to_y_coarse, AXIS_Y_DIRECTION);
	MeasureBase::RotatePoints(rot_plane_points_to_y_coarse, rotation_matrix_to_y_fine, rot_plane_points_to_y_fine);
	MeasureBase::RotatePoints(rot_centers_to_y_coarse, rotation_matrix_to_y_fine, rot_centers_to_y_fine);


#ifdef DEBUG_DEFECT
	//for debug
	std::cout << "plane normal coarsely aligned to Y axis: ( " << rot_plane_normal_to_y_coarse[0] << " , " << rot_plane_normal_to_y_coarse[1] << " , " << rot_plane_normal_to_y_coarse[2] << " )" << std::endl;
	float rot_plane_normal_to_y_fine[3];
	MeasureBase::RotateVector(rot_plane_normal_to_y_coarse, rotation_matrix_to_y_fine, rot_plane_normal_to_y_fine);
	std::cout << "plane normal finely aligned to Y axis: ( " << rot_plane_normal_to_y_fine[0] << " , " << rot_plane_normal_to_y_fine[1] << " , " << rot_plane_normal_to_y_fine[2] << " )" << std::endl;
#endif


	/*Backward rotation matrix*/
	/*Rotation matrix property: tranpose = inverse*/
	cv::Mat backward_rotation_matrix_from_y_coarse(3, 3, CV_32FC1);
	cv::Mat backward_rotation_matrix_from_y_fine(3, 3, CV_32FC1);
	cv::transpose(rotation_matrix_to_y_coarse, backward_rotation_matrix_from_y_coarse);
	cv::transpose(rotation_matrix_to_y_fine, backward_rotation_matrix_from_y_fine);


#ifdef DEBUG_DEFECT
	//for debug
	MeasureBase::SavePoints(rot_plane_points_to_y_coarse, "RotatedPlane2YCoarse.txt");;
	MeasureBase::SavePoints(rot_plane_points_to_y_fine, "RotatedPlane2YFine.txt");
#endif

	/*calculate mean along Y axis*/
	//double sum_y = 0.f;
	//for (int i = 0; i < rot_plane_points_to_y_fine.size(); i++)
	//	sum_y += rot_plane_points_to_y_fine[i].y;
	//double y_mean = sum_y / ((double)(rot_plane_points_to_y_fine.size()));
	// 
	//// use refined mean y  // added by wei.fan
	//if (obstacleInfoPtr != nullptr && typeid(*(obstacleInfoPtr->getClassType())) == typeid(WallObstacleInfo)) {
	//	WallObstacleInfo* wallObstacleInfo = static_cast<WallObstacleInfo*>(obstacleInfoPtr);
	//	
	//	/*std::cout << "origin_wall_id:" << wallObstacleInfo->origin_wall_id << std::endl;
	//	std::cout << "origin wall normal:" << plane_normal[0] << "," << plane_normal[1] << "," << plane_normal[2] << std::endl;
	//	std::cout << "refined wall normal:" << wallObstacleInfo->refinedPlaneNormalVector[wallObstacleInfo->origin_wall_id] << std::endl;
	//	std::cout << "origin mean:" << y_mean << std::endl;
	//	std::cout << "refined mean:" << wallObstacleInfo->refinedMeanCloudPointsYVector[wallObstacleInfo->origin_wall_id] << std::endl;*/
	//	
	//	if ((wallObstacleInfo->refinedPlaneNormalVector[wallObstacleInfo->origin_wall_id]).sum() > -1.5)
	//		y_mean = wallObstacleInfo->refinedMeanCloudPointsYVector[wallObstacleInfo->origin_wall_id];
	//}
	//else if (obstacleInfoPtr != nullptr && typeid(*(obstacleInfoPtr->getClassType())) == typeid(GroundObstacleInfo)) {
	//	GroundObstacleInfo* groundObstacleInfo = static_cast<GroundObstacleInfo*>(obstacleInfoPtr);
	//	
	//	/*std::cout << "origin_wall_id:" << groundObstacleInfo->origin_wall_id << std::endl;
	//	std::cout << "origin wall normal:" << plane_normal[0] << "," << plane_normal[1] << "," << plane_normal[2] << std::endl;
	//	std::cout << "refined wall normal:" << groundObstacleInfo->refinedPlaneNormal << std::endl;
	//	std::cout << "origin mean:" << y_mean << std::endl;
	//	std::cout << "refined mean:" << groundObstacleInfo->refinedMeanCloudPointsZ << std::endl;*/

	//	if ((groundObstacleInfo->refinedPlaneNormal).sum() > -1.5)
	//		y_mean = groundObstacleInfo->refinedMeanCloudPointsZ;
	//}
	double y_mean = rot_centers_to_y_fine[0].y;
	/*Find plane boundaries in XY*/
	float plane_minmax_xy[6];
	MeasureBase::FindPointsMinMaxXYZ(rot_plane_points_to_y_fine, plane_minmax_xy);

#ifdef DEBUG_DEFECT
	//for debug
	std::cout << "y_mean: " << y_mean << std::endl;
	std::cout << "Ymin: " << plane_minmax_xy[2] << " ; " << "Ymax: " << plane_minmax_xy[3] << std::endl;
#endif


	/*Voxelize*/
	/*Voxels are arranged row by row in XOZ plane*/
	std::vector<Voxel_in_Z> voxels;
	this->VoxelizePlane(rot_plane_points_to_y_fine, y_mean, plane_minmax_xy, voxels);


#ifdef DEBUG_DEFECT
	//for debug
	//to show all voxels' left top corner
	std::vector<cv::Point3f> defect_points(voxels.size());
	for (int i = 0; i < voxels.size(); i++)
	{
		if (voxels[i].is_occupied)
			defect_points[i] = voxels[i].left_top;
	}
	MeasureBase::SavePoints(defect_points, "DefectCorners1.txt");
#endif


	/*Find defect in original coordinate*/
	this->AssignDefectResult(voxels, backward_rotation_matrix_from_y_coarse, backward_rotation_matrix_from_y_fine, defect);

#ifdef DEBUG_DEFECT
	//for debug
	defect_points.clear();
	defect_points.resize(defect.size());
	//std::vector<cv::Point3f> defect_points(defect.size());
	for (int i = 0; i < defect.size(); i++)	
		defect_points[i] = defect[i].second;
	MeasureBase::SavePoints(defect_points, "DefectCorners2.txt");
#endif

	return true;
}



bool MeasureDefect::VoxelizePlane(const std::vector<cv::Point3f>& plane_points, const float y_mean, const float* plane_minmax_xy, std::vector<Voxel_in_Z>& voxels)
{
	if (plane_points.empty())	return false;
	
	unsigned int voxel_num_in_x = std::floor((plane_minmax_xy[1] - plane_minmax_xy[0]) / voxel_width) + 1;
	unsigned int voxel_num_in_z = std::floor((plane_minmax_xy[5] - plane_minmax_xy[4]) / voxel_height) + 1;

	voxels.resize(voxel_num_in_x * voxel_num_in_z);
	unsigned int x_idx, z_idx, voxel_idx;

	float delta_y;

	for (int i = 0; i < plane_points.size(); i++)
	{

		if (abs(plane_points[i].x - plane_minmax_xy[0]) < 30.0 ||
			abs(plane_points[i].x - plane_minmax_xy[1]) < 30.0 ||
			abs(plane_points[i].z - plane_minmax_xy[4]) < 30.0 ||
			abs(plane_points[i].z - plane_minmax_xy[5]) < 30.0)
			continue;
		//assign to voxel
		x_idx = std::floor((plane_points[i].x - plane_minmax_xy[0]) / voxel_width);
		z_idx = std::floor((plane_points[i].z - plane_minmax_xy[4]) / voxel_height);
		voxel_idx = z_idx * voxel_num_in_x + x_idx;

		if (!voxels[voxel_idx].is_occupied)
		{
			//conduct once for each voxel to be occupied
			voxels[voxel_idx].is_occupied = true;
			voxels[voxel_idx].left_top.x = plane_minmax_xy[0] + x_idx * voxel_width + voxel_width / 2;
			voxels[voxel_idx].left_top.y = y_mean;
			voxels[voxel_idx].left_top.z = plane_minmax_xy[4] + z_idx * voxel_height + voxel_height / 2;
		}

		delta_y = plane_points[i].y - y_mean;
		voxels[voxel_idx].defect_sum += delta_y;
		voxels[voxel_idx].pt_num++;

		////for debug
		//if (delta_y > 10.f || delta_y < -10.f)
		//	std::cout << delta_y << std::endl;


		if (delta_y >= 0.f)
		{
			voxels[voxel_idx].defect_sum_plus += delta_y;
			voxels[voxel_idx].pt_num_plus++;
		}
		else
		{
			voxels[voxel_idx].defect_sum_minus += delta_y;
			voxels[voxel_idx].pt_num_minus++;
		}

#ifdef DEBUG_DEFECT
		voxels[voxel_idx].points.push_back(plane_points[i]);
#endif
		
	}

	//for debug
#ifdef DEBUG_DEFECT
	std::vector<cv::Point3f> pts_in_voxels;
	for (int i = 0; i < voxels.size(); i++)
	{
		for (int j = 0; j < voxels[i].points.size(); j++)
			pts_in_voxels.push_back(voxels[i].points[j]);
	}

	MeasureBase::SavePoints(pts_in_voxels, "pts_in_voxels.txt");
#endif

	////for debug
	//for (int i = 0; i < voxels.size(); i++)
	//{
	//	//if (MeasureBase::IsValZero(voxels[i].left_top.x) && MeasureBase::IsValZero(voxels[i].left_top.y) && MeasureBase::IsValZero(voxels[i].left_top.z))
	//	if (voxels[i].is_occupied && voxels[i].left_top.x == 0.f && voxels[i].left_top.y == 0.f && voxels[i].left_top.z == 0.f)
	//	{
	//		std::cout << "zero: " << i << std::endl;
	//	}
	//}

	return true;
}


bool MeasureDefect::AssignDefectResult(const std::vector<Voxel_in_Z>& voxels, const cv::Mat backward_rotation_matrix1, const cv::Mat backward_rotation_matrix2, std::vector<std::pair<float, cv::Point3f>>& defect)
{
	if (voxels.empty())
		return false;

	defect.resize(voxels.size());
	unsigned int valid_voxel_counter = 0;
	float defect_value;

	cv::Point3f temp_pt;

	////for debug
	//std::cout << "defect_threshold_in_mm: " << defect_threshold_in_mm << std::endl;

#ifdef DEBUG_DEFECT
	std::vector<cv::Point3f> defect_pts_plus, defect_pts_minus;
#endif // DEBUG_DEFECT

	for (int i = 0; i < voxels.size(); i++)
	{
		if (voxels[i].is_occupied && voxels[i].pt_num >= min_pt_num_in_voxel)
		{
			defect_value = voxels[i].defect_sum/ (float)(voxels[i].pt_num);
			
			//if (voxels[i].defect_sum_plus >= -voxels[i].defect_sum_minus)
			//	defect_value = voxels[i].defect_sum_plus / (float)(voxels[i].pt_num);
			//else
			//	defect_value = voxels[i].defect_sum_minus / (float)(voxels[i].pt_num);

#ifdef DEBUG_DEFECT
			if (defect_value > defect_threshold_in_mm)
			{
				for (int j = 0; j < voxels[i].points.size(); j++)
					defect_pts_plus.push_back(voxels[i].points[j]);
			}
			else if (defect_value < -defect_threshold_in_mm)
			{
				for (int j = 0; j < voxels[i].points.size(); j++)
					defect_pts_minus.push_back(voxels[i].points[j]);
			}
#endif
			
			////for debug
			//std::cout << "defect value: " << defect_value << std::endl;

			//if (defect_value >= defect_threshold_in_mm || defect_value <= -defect_threshold_in_mm)
			{
				////for debug
				//std::cout << defect_value << std::endl;

				defect[valid_voxel_counter].first = defect_value;

				cv::Point3f left_top = voxels[i].left_top;
				if(mDefect3D)
					left_top.y += defect_value;
				//voxel left top vertex as output
				temp_pt.x = backward_rotation_matrix2.at<float>(0, 0) * left_top.x
					+ backward_rotation_matrix2.at<float>(0, 1) * left_top.y
					+ backward_rotation_matrix2.at<float>(0, 2) * left_top.z;
				temp_pt.y = backward_rotation_matrix2.at<float>(1, 0) * left_top.x
					+ backward_rotation_matrix2.at<float>(1, 1) * left_top.y
					+ backward_rotation_matrix2.at<float>(1, 2) * left_top.z;
				temp_pt.z = backward_rotation_matrix2.at<float>(2, 0) * left_top.x
					+ backward_rotation_matrix2.at<float>(2, 1) * left_top.y
					+ backward_rotation_matrix2.at<float>(2, 2) * left_top.z;

				defect[valid_voxel_counter].second.x = backward_rotation_matrix1.at<float>(0, 0) * temp_pt.x
					+ backward_rotation_matrix1.at<float>(0, 1) * temp_pt.y
					+ backward_rotation_matrix1.at<float>(0, 2) * temp_pt.z;
				defect[valid_voxel_counter].second.y = backward_rotation_matrix1.at<float>(1, 0) * temp_pt.x
					+ backward_rotation_matrix1.at<float>(1, 1) * temp_pt.y
					+ backward_rotation_matrix1.at<float>(1, 2) * temp_pt.z;
				defect[valid_voxel_counter].second.z = backward_rotation_matrix1.at<float>(2, 0) * temp_pt.x
					+ backward_rotation_matrix1.at<float>(2, 1) * temp_pt.y
					+ backward_rotation_matrix1.at<float>(2, 2) * temp_pt.z;

				valid_voxel_counter++;
			}
		}
	}

	defect.resize(valid_voxel_counter);

#ifdef DEBUG_DEFECT
	std::cout << "defect_pts_plus: " << defect_pts_plus.size() << std::endl;
	std::cout << "defect_pts_minus: " << defect_pts_minus.size() << std::endl;
	MeasureBase::SavePoints(defect_pts_plus, "defect_plus.txt");
	MeasureBase::SavePoints(defect_pts_minus, "defect_minus.txt");
#endif // DEBUG

	return true;
}


