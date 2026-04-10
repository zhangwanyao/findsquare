#pragma once
#include "MeasureLevelnessRange.h"
#include "../log/log.h"
#include <omp.h> 
//#include <../RoomCompass.h>
#include "MeasureBase.h"
#include <opencv2\opencv.hpp>
#define DEVELOPER_MODE
//#define DBG_L
//#define DBG_RL

#define MEASURE_POSITION_ALL            0x0777
#define MEASURE_POSITION_CORNER_AND_MIDDLE 0x0777
bool GetHeightLevelnessAll()
{
	return false;
}
int GetCeilingLevelnessMeasurePosition()
{
	return false;
}

MeasureLevelnessRange::MeasureLevelnessRange(){
	measure_ROI_dist_from_plane_edge = 600.f;
	measure_ROI_length = 300.f;

	plane_width_threshold = 800.f;
	min_num_pts_inrulerband = 100;
	normal_zero_dir_threshold = 0.3f;

	offset_ROI_actualPts = measure_ROI_length * 0.5f * 0.01f;
	
	has_imu = false;

	origin = cv::Point3f(0.f, 0.f, 0.f);

	//added by simon.jin@unre.com start
	voxel_width = 100.0f;
	voxel_height = 100.0f;
	min_pt_num_in_voxel = 10;
	levelness_threshold = 3;
	//added by simon.jin@unre.com end
}

MeasureLevelnessRange::~MeasureLevelnessRange(){}


void MeasureLevelnessRange::SetSetRulerDistanceToPlaneEdge(float distance_from_edge)
{
	measure_ROI_dist_from_plane_edge = distance_from_edge;
}

void MeasureLevelnessRange::GetRulerDistanceToPlaneEdge(float& distance_from_edge)
{
	distance_from_edge = measure_ROI_dist_from_plane_edge;
}

void MeasureLevelnessRange::SetRulerSize(float ruler_square_length)
{
	measure_ROI_length = ruler_square_length;
}

void MeasureLevelnessRange::GetRulerSize(float& ruler_square_length)
{
	 ruler_square_length = measure_ROI_length;
}



bool MeasureLevelnessRange::MeasureLevelnessRangeFcn(const float* plane_normal, 
	const float* ground_normal, 
	const std::vector<cv::Point3f>& plane_points, 
	MeasurementResultValueValuesPoints& levelness,
	std::vector<cv::Point3f>& actual_vertices,
	std::vector<std::pair<float, cv::Point3f>> * p_full_levelness)
{
	/* Check if empty plane */
	if (plane_points.empty())
	{
#ifdef DEVELOPER_MODE
		std::cout << "MeasureLevelnessRange::MeasureLevelnessRangeFcn(): Empty plane input" << std::endl;
#endif
		return false;
	}

#if 0
	/*Rotate ceiling normal by aligning ground normal to Z*/
	float grot_plane_normal[3];
	cv::Mat rotation_matrix_to_z(3, 3, CV_32F);
	if (has_imu)
	{
		rotation_matrix_to_z = cv::Mat::eye(3, 3, CV_32FC1);
		std::memcpy(&grot_plane_normal[0], &plane_normal[0], sizeof(plane_normal));
	}
	else
	{
		rotation_matrix_to_z = MeasureBase::CalRotationMatrixFromVectors(&ground_normal[0], &AXIS_Z_DIRECTION[0]);
		MeasureBase::RotateVector(&plane_normal[0], rotation_matrix_to_z, &grot_plane_normal[0]);
	}


	/* Ensure horizontal plane */
	if (abs(abs(grot_plane_normal[2]) - 1.f) > normal_zero_dir_threshold)
	{
#ifdef DEVELOPER_MODE
		std::cout << "MeasureLevelnessRange::MeasureLevelnessRangeFcn(): Ceiling is not horizontal" <<std::endl;
#endif
		return false;
	}	


	/* 1st rotation */
	/* Rotate horizontal plane so that one edge of plane parallel to x axis */
	/* Note that x=plane_normal[0], y=plane_normal[1], and z=plane_normal[2]:
	vector (x,y,0) presents normal of some plane edge; vector (0,0,z) is plane normal*/
	float rot_ground_normal[3];
	std::vector<cv::Point3f> rot_plane_points(plane_points.size());
	float rotation_angle_around_z;
	BOOL_FUNCTION_CHECK(this->RotateHorizontalPlane2XAxis(plane_points, &plane_normal[0], &ground_normal[0], &rotation_angle_around_z, rot_plane_points, &rot_ground_normal[0]));

	//Rotate coordinate origin
	MeasureBase::RotatePointAroundZ(origin, rotation_angle_around_z, rotated_origin);

	/* 2nd rotation (only rotation matrix, not rotating all points)*/
	/*Rigid rotation matrix for this room, with which ground normal is aligned with Z_AXIS */
	if (!has_imu)
		rotation_matrix_to_z = MeasureBase::CalRotationMatrixFromVectors(rot_ground_normal, &AXIS_Z_DIRECTION[0]);
	else
		rotation_matrix_to_z = cv::Mat::eye(3, 3, CV_32FC1);

	////for debug 
	//std::cout << "rotation_matrix_z" << rotation_matrix_to_z << std::endl;
	//std::vector<cv::Point3f> rot_rot_plane_points;
	//MeasureBase::RotatePoints(rot_plane_points, rotation_matrix_to_z, rot_rot_plane_points);
	//MeasureBase::SavePoints(rot_plane_points, "rot_ceiling.txt");
	//MeasureBase::SavePoints(rot_rot_plane_points, "rot_rot_ceiling.txt");
	
#else
	cv::Mat rotation_matrix_to_z(3, 3, CV_32F);
	std::vector<cv::Point3f> rot_plane_points(plane_points.size());
	rot_plane_points = plane_points;
	float rotation_angle_around_z = 0;

#endif	
	/*Find horizontal plane boundary by min/max x, y & z*/
	float rot_plane_minmax_xyz[6];
	int normal_axis = 2; //to indicate horizontal plane

	if (!MeasureBase::FindPlaneMinMaxXYZ2(rot_plane_points, normal_axis, plane_width_threshold, plane_width_threshold, &rot_plane_minmax_xyz[0]))
	{
#ifdef DEVELOPER_MODE
		std::cout << "MeasureLevelnessRange::MeasureLevelnessRangeFcn(): Ceiling is too small" << std::endl;
#endif
		return false;
	}

	//added by simon.jin@unre.com start
	if (p_full_levelness != NULL)
	{
		std::vector<Voxel_LNR> voxel_lnr;

		if (VoxelizePlane(rot_plane_points, &rot_plane_minmax_xyz[0], voxel_lnr))
		{
			ComputeLevelnessResult(voxel_lnr, rotation_angle_around_z, *p_full_levelness);
			std::vector<std::pair<float, cv::Point3f>> & full_levelness = *p_full_levelness;
			if (plane_normal[2] < 0.f)
			{
				for (int i = 0; i < full_levelness.size(); i++)
				{
					full_levelness[i].first = -full_levelness[i].first;
				}
			}
		}
	} 
	//added by simon.jin@unre.com end

	/*Check if reasonable parameter settings of measure_ROI_dist_from_plane_edge & measure_ROI_length*/
	float min_plane_length = 2.f * (measure_ROI_dist_from_plane_edge + measure_ROI_length * 0.5f); 
	if (MeasureBase::IsValZero(measure_ROI_length)
		|| measure_ROI_dist_from_plane_edge < measure_ROI_length * 0.5f
		|| rot_plane_minmax_xyz[1] - rot_plane_minmax_xyz[0] < min_plane_length 
		|| rot_plane_minmax_xyz[3] - rot_plane_minmax_xyz[2] < min_plane_length)
	{
#ifdef DEVELOPER_MODE
		std::cout << "MeasureLevelnessRange::wrong parameter settings of measure_ROI_dist_from_plane_edge & measure_ROI_length" << std::endl;;
#endif
		return false;
	}


	/* Measure levelness at five locations with four at corners & one in center */
	/* ruler 1~4 @ corner & ruler 5 @ center */

	if (GetHeightLevelnessAll()) {
		bool ret = this->ComputeLevelnessRangeAll(rot_plane_points, rot_plane_minmax_xyz, rotation_matrix_to_z, rotation_angle_around_z,
			levelness, actual_vertices);
		if (!ret){
			std::cout << "MeasureLevelnessRange::MeasureLevelnessRangeFcn(): Cannot find enough points to compute levelness" << std::endl;
			return false;
		}
	}
	else {
		if (!this->ComputeLevelnessRange(rot_plane_points, rot_plane_minmax_xyz, rotation_matrix_to_z, rotation_angle_around_z, levelness, actual_vertices))
		{
			std::cout << "MeasureLevelnessRange::MeasureLevelnessRangeFcn(): Cannot find enough points to compute levelness" << std::endl;
			return false;
		}
	}

	return true;
}

std::pair<cv::Point3f, float> CalPointIntersectLine(const cv::Point3f pointOutLine,
	const cv::Point3f pointALine,
	const cv::Point3f pointBLine)
{
	//see https://www.cnblogs.com/mazhenyu/p/3508735.html
	cv::Point3f line = pointALine - pointBLine;
	float length_line = cv::norm(line);

	if (length_line < 0.1)
	{
		return(std::pair<cv::Point3f, float>((pointALine - pointBLine)*0.5f, 0.f));
	}
	else {
		float k = -(pointBLine - pointOutLine).dot(line) / std::pow(length_line, 2);

		cv::Point3f point = k * line + pointBLine;

		float dist_to_line = cv::norm(pointOutLine - point);

		/*printf("line: %f, %f, %f",  line.x,line.y,line.z );
		printf("k: %f", k);
		printf("pt: %f, %f, %f", point.x , point.y , point.z);*/

		return(std::pair<cv::Point3f, float>(point, dist_to_line));
	}
}

void getLinePara(cv::Point2f p1, cv::Point2f p2, cv::Point2f &Pa)
{
	if (0 == (p1.x - p2.x))
	{
		Pa.x = 100000.0;
		Pa.y = p1.y - Pa.x * p1.x;
	}
	else {
		Pa.x = (p2.y - p1.y) / (p2.x - p1.x);
		Pa.y = p1.y - Pa.x * p1.x;
	}
}

bool MeasureLevelnessRange::GetLevelnessLocalRulersFromPlaneCore(
	const std::vector<cv::Point3f> & ver_pts,
	const std::vector<cv::Point3f> & levelness_pts,
	std::vector<MeasurementRulerverticeStruct> & levelness_local_ruler_vertice_intersect_pts_dis)
{
	if (ver_pts.size() == 0)
		return false;
#ifdef	DBG_RL
	std::cout << "ver_pts.size():  " << ver_pts.size() << endl;
#endif
	int id1[3];

	for (unsigned int k = 0; k < levelness_pts.size(); k++)
	{
#ifdef	DBG_RL
		std::cout << "k==== :  " << k << endl;
#endif
		float dist_temp = INFINITY;
		unsigned int indx = 0;
		int rd = ver_pts.size();
		float STest0 = INFINITY;
		float STest1 = INFINITY;
		int STestI0 = 0;
		int STestI1 = 1;
		levelness_local_ruler_vertice_intersect_pts_dis.resize(levelness_pts.size());
		levelness_local_ruler_vertice_intersect_pts_dis[k].ruler_endpts.resize(1);
		levelness_local_ruler_vertice_intersect_pts_dis[k].endpt_intersect_pts.resize(1);
		levelness_local_ruler_vertice_intersect_pts_dis[k].endpt_intersect_dist.resize(1);
		levelness_local_ruler_vertice_intersect_pts_dis[k].ruler_endpts[0] = levelness_pts[k];
		
		cv::Point3f ruler_center = levelness_pts[k];

		for (int i = 0; i < ver_pts.size(); i++) 
		{
			//cout << "ruler_center:" << ruler_center << endl;
			//cout << "ver_pts[0]:" << ver_pts[i] << endl;
			//cout << "ver_pts[1]:" << ver_pts[(i + 1) % rd] << endl;
			
			if (abs(ver_pts[i].x - ver_pts[(i + 1) % rd].x) > abs(ver_pts[i].y - ver_pts[(i + 1) % rd].y)) {
				if (ruler_center.x < min(ver_pts[i].x, ver_pts[(i + 1) % rd].x) ||
					ruler_center.x > max(ver_pts[i].x, ver_pts[(i + 1) % rd].x)) {
					continue;
				}
			}else {
				if (ruler_center.y < min(ver_pts[i].y, ver_pts[(i + 1) % rd].y) ||
					ruler_center.y > max(ver_pts[i].y, ver_pts[(i + 1) % rd].y)) {
					continue;
				}
			}
			std::pair<cv::Point3f, float> tmpd = CalPointIntersectLine(ruler_center,ver_pts[i], ver_pts[(i+1)%rd]);
#ifdef	DBG_RL
			std::cout << "=========i:"<<i <<" tmpd.second < STest0:   " << tmpd.second << " ,  " << STest0 << endl;
#endif
			if (tmpd.second < STest0) {
				levelness_local_ruler_vertice_intersect_pts_dis[k].endpt_intersect_pts[0].first = tmpd.first;
				levelness_local_ruler_vertice_intersect_pts_dis[k].endpt_intersect_dist[0].first = tmpd.second;
				STest0 = tmpd.second;
				STestI0 = i;
#ifdef	DBG_RL
				std::cout << "##### STestI0 =:  " << STestI0 << endl;
#endif
			}
	
		}
		for (int i = 0; i < ver_pts.size(); i++) 
		{
			if (i == STestI0) 
				continue;

			if (abs(ver_pts[i].x - ver_pts[(i + 1) % rd].x) > abs(ver_pts[i].y - ver_pts[(i + 1) % rd].y)) {
				if (ruler_center.x < min(ver_pts[i].x, ver_pts[(i + 1) % rd].x) ||
					ruler_center.x > max(ver_pts[i].x, ver_pts[(i + 1) % rd].x)) {
					continue;
				}
			}else {
				if (ruler_center.y < min(ver_pts[i].y, ver_pts[(i + 1) % rd].y) ||
					ruler_center.y > max(ver_pts[i].y, ver_pts[(i + 1) % rd].y)) {
					continue;
				}
			}
			std::pair<cv::Point3f, float> tmpd = CalPointIntersectLine(ruler_center,ver_pts[i], ver_pts[(i + 1) % rd]);
#ifdef	DBG_RL
			std::cout << "=========i:" << i << " tmpd.second < STest1: " << tmpd.second << " ,  " << STest1 << endl;
#endif
			if (tmpd.second < STest1) {
				levelness_local_ruler_vertice_intersect_pts_dis[k].endpt_intersect_pts[0].second = tmpd.first;
				levelness_local_ruler_vertice_intersect_pts_dis[k].endpt_intersect_dist[0].second = tmpd.second;
				STest1 = tmpd.second;
				STestI1 = i;
#ifdef	DBG_RL
				std::cout << "############E STestI1 =:  " << STestI1 << endl;
#endif
			}
		}
/*
		for (unsigned int n = 0; n < ver_pts.size(); n++) {
			float dist = norm(ruler_center - ver_pts[n]);
			if (dist < dist_temp)
			{
				indx = n;
				dist_temp = dist;
			}
		}

		levelness_local_ruler_vertice_intersect_pts_dis[k].ruler_endpts.resize(1);
		levelness_local_ruler_vertice_intersect_pts_dis[k].endpt_intersect_pts.resize(1);
		levelness_local_ruler_vertice_intersect_pts_dis[k].endpt_intersect_dist.resize(1);
		levelness_local_ruler_vertice_intersect_pts_dis[k].ruler_endpts[0] = levelness_pts[0];

		std::vector<std::pair<cv::Point3f, float>> two_intersect_pts_dis(2);

		int count = 0;
		for (int n = -1; n < 2; n += 2)
		{
			int indx2 = indx + n;
			if (indx2 < 0)
			{
				indx2 = 3;
			}
			else if (indx2 > 3)
			{
				indx2 = 0;
			}

			two_intersect_pts_dis[count] = CalPointIntersectLine(
				levelness_local_ruler_vertice_intersect_pts_dis[k].ruler_endpts[0],
				ver_pts[indx], ver_pts[indx2]);

			if (count == 0)
			{
				levelness_local_ruler_vertice_intersect_pts_dis[k].endpt_intersect_pts[0].first = two_intersect_pts_dis[count].first;
				levelness_local_ruler_vertice_intersect_pts_dis[k].endpt_intersect_dist[0].first = two_intersect_pts_dis[count].second;
			}
			else {
				levelness_local_ruler_vertice_intersect_pts_dis[k].endpt_intersect_pts[0].second = two_intersect_pts_dis[count].first;
				levelness_local_ruler_vertice_intersect_pts_dis[k].endpt_intersect_dist[0].second = two_intersect_pts_dis[count].second;
			}
			count += 1;
		}
		*/
	}
	return true;
}

bool MeasureLevelnessRange::RotateHorizontalPlanes2XAxis(const std::vector<cv::Point3f>& plane1_points, const float* plane1_normal,
	const std::vector<cv::Point3f>& plane2_points, const float* ground_normal,
	float* rotation_angle_around_z,
	std::vector<cv::Point3f>& rot_plane1_points,
	std::vector<cv::Point3f>& rot_plane2_points,
	float* rot_ground_normal)
{
	if (plane1_points.empty()) 
		return false;

	if (MeasureBase::IsValZero(plane1_normal[0]))
	{
		*rotation_angle_around_z = 0.f;
		rot_plane1_points = plane1_points;
		if (!plane2_points.empty())
			rot_plane2_points = plane2_points;
	
		rot_ground_normal[0] = ground_normal[0];
		rot_ground_normal[1] = ground_normal[1];
		rot_ground_normal[2] = ground_normal[2];
	}
	else //rotate around Z-axis & one edge will be aligned with Y-axis
	{
		MeasureBase::CalcAngleVectorXY2YAxis(plane1_normal, rotation_angle_around_z);
		BOOL_FUNCTION_CHECK(MeasureBase::RotatePointsAroundZ(plane1_points, *rotation_angle_around_z, rot_plane1_points));
		if (!plane2_points.empty())
			BOOL_FUNCTION_CHECK(MeasureBase::RotatePointsAroundZ(plane2_points, *rotation_angle_around_z, rot_plane2_points));
		MeasureBase::RotateVectorAroundZ(ground_normal, *rotation_angle_around_z, rot_ground_normal);
	}

	return true;
}



bool MeasureLevelnessRange::RotateHorizontalPlane2XAxis(const std::vector<cv::Point3f>& plane_points, const float* plane_normal, 
	const float* ground_normal,
	float* rotation_angle_around_z,
	std::vector<cv::Point3f>& rot_plane_points,
	float* rot_ground_normal)
{
	if (plane_points.empty())
		return false;

	std::vector<cv::Point3f> empty_plane_points, rot_empty_plane_points;
	BOOL_FUNCTION_CHECK(RotateHorizontalPlanes2XAxis(plane_points, plane_normal, empty_plane_points, ground_normal, rotation_angle_around_z,
		rot_plane_points, rot_empty_plane_points, rot_ground_normal));

	return true;
}


void MeasureLevelnessRange::AssignMeasureVirtualCorner(int Rulers, const float* minmax_xyz, const float edgesize,
	const int times_ifrerun_x, const int times_ifrerun_y,
	const float movesize_x, const float movesize_y,
	const int ruler,
	float* virtual_corner)
{
	if (Rulers == 5) {
		if (ruler == 0) 
		{
			virtual_corner[0] = minmax_xyz[0] + edgesize + times_ifrerun_x * movesize_x;
			virtual_corner[1] = minmax_xyz[2] + edgesize + times_ifrerun_y * movesize_y;
		}
		else if (ruler == 1)
		{
			virtual_corner[0] = minmax_xyz[1] - edgesize - times_ifrerun_x * movesize_x;
			virtual_corner[1] = minmax_xyz[2] + edgesize + times_ifrerun_y * movesize_y;
		}
		else if (ruler == 2) 
		{
			virtual_corner[0] = minmax_xyz[1] - edgesize - times_ifrerun_x * movesize_x;
			virtual_corner[1] = minmax_xyz[3] - edgesize - times_ifrerun_y * movesize_y;
		}
		else if (ruler == 3) 
		{
			virtual_corner[0] = minmax_xyz[0] + edgesize + times_ifrerun_x * movesize_x;
			virtual_corner[1] = minmax_xyz[3] - edgesize - times_ifrerun_y * movesize_y;
		}
		else {
			float plane_center[2];
			plane_center[0] = (minmax_xyz[0] + minmax_xyz[1]) * 0.5f;
			plane_center[1] = (minmax_xyz[2] + minmax_xyz[3]) * 0.5f;
			float plane_center_origin_offset[2];
			plane_center_origin_offset[0] = plane_center[0] - rotated_origin.x;
			plane_center_origin_offset[1] = plane_center[1] - rotated_origin.y;
		//	cout << " c.x:" << plane_center[0] << " c.y:" << plane_center[1] << " r.x:" << rotated_origin.x << " r.y:" << rotated_origin.y << endl;
			if (plane_center_origin_offset[0] > 0.f && plane_center_origin_offset[1] > 0.f)
			{
				virtual_corner[0] = plane_center[0] + times_ifrerun_x * movesize_x;
				virtual_corner[1] = plane_center[1] + times_ifrerun_y * movesize_y;
			}
			else if (plane_center_origin_offset[0] < 0.f && plane_center_origin_offset[1] > 0.f)
			{
				virtual_corner[0] = plane_center[0] - times_ifrerun_x * movesize_x;
				virtual_corner[1] = plane_center[1] + times_ifrerun_y * movesize_y;
			}
			else if (plane_center_origin_offset[0] < 0.f && plane_center_origin_offset[1] < 0.f)
			{
				virtual_corner[0] = plane_center[0] - times_ifrerun_x * movesize_x;
				virtual_corner[1] = plane_center[1] - times_ifrerun_y * movesize_y;
			}
			else if (plane_center_origin_offset[0] > 0.f && plane_center_origin_offset[1] < 0.f)
			{
				virtual_corner[0] = plane_center[0] + times_ifrerun_x * movesize_x;
				virtual_corner[1] = plane_center[1] - times_ifrerun_y * movesize_y;
			}
		}
	}
	
	if (Rulers == 9) {
		if (ruler == 0)
		{
			virtual_corner[0] = minmax_xyz[0] + edgesize + times_ifrerun_x * movesize_x;
			virtual_corner[1] = minmax_xyz[2] + edgesize + times_ifrerun_y * movesize_y;
		}
		else if (ruler == 1)
		{
			virtual_corner[0] = minmax_xyz[1] - edgesize - times_ifrerun_x * movesize_x;
			virtual_corner[1] = minmax_xyz[2] + edgesize + times_ifrerun_y * movesize_y;
		}
		else if (ruler == 2) 
		{
			virtual_corner[0] = minmax_xyz[1] - edgesize - times_ifrerun_x * movesize_x;
			virtual_corner[1] = minmax_xyz[3] - edgesize - times_ifrerun_y * movesize_y;
		}
		else if (ruler == 3)
		{
			virtual_corner[0] = minmax_xyz[0] + edgesize + times_ifrerun_x * movesize_x;
			virtual_corner[1] = minmax_xyz[3] - edgesize - times_ifrerun_y * movesize_y;
		}
		else if (ruler == 4) 
		{
			virtual_corner[0] = minmax_xyz[0] + edgesize + times_ifrerun_x * movesize_x;
			virtual_corner[1] = (minmax_xyz[2] + minmax_xyz[3]) / 2 + times_ifrerun_y * movesize_y;
		}
		else if (ruler == 5) 
		{
			virtual_corner[0] = (minmax_xyz[0] + minmax_xyz[1]) / 2 - times_ifrerun_x * movesize_x;
			virtual_corner[1] = minmax_xyz[2] + edgesize + times_ifrerun_y * movesize_y;
		}
		else if (ruler == 6) 
		{
			virtual_corner[0] = minmax_xyz[1] - edgesize - times_ifrerun_x * movesize_x;
			virtual_corner[1] = (minmax_xyz[2] + minmax_xyz[3]) / 2 - times_ifrerun_y * movesize_y;
		}
		else if (ruler == 7) 
		{
			virtual_corner[0] = (minmax_xyz[0] + minmax_xyz[1]) / 2 + times_ifrerun_x * movesize_x;
			virtual_corner[1] = minmax_xyz[3] - edgesize - times_ifrerun_y * movesize_y;
		}
		else //ruler 9 @ center, moving away from the origin (LiDar center)
		{
			float plane_center[2];
			plane_center[0] = (minmax_xyz[0] + minmax_xyz[1]) * 0.5f;
			plane_center[1] = (minmax_xyz[2] + minmax_xyz[3]) * 0.5f;
			float plane_center_origin_offset[2];
			plane_center_origin_offset[0] = plane_center[0] - rotated_origin.x;
			plane_center_origin_offset[1] = plane_center[1] - rotated_origin.y;
			//cout << " c.x:" << plane_center[0] << " c.y:" << plane_center[1] << " r.x:" << rotated_origin.x << " r.y:" << rotated_origin.y;
			if (plane_center_origin_offset[0] > 0.f && plane_center_origin_offset[1] > 0.f)
			{
				virtual_corner[0] = plane_center[0] + times_ifrerun_x * movesize_x;
				virtual_corner[1] = plane_center[1] + times_ifrerun_y * movesize_y;
			}
			else if (plane_center_origin_offset[0] < 0.f && plane_center_origin_offset[1] > 0.f)
			{
				virtual_corner[0] = plane_center[0] - times_ifrerun_x * movesize_x;
				virtual_corner[1] = plane_center[1] + times_ifrerun_y * movesize_y;
			}
			else if (plane_center_origin_offset[0] < 0.f && plane_center_origin_offset[1] < 0.f)
			{
				virtual_corner[0] = plane_center[0] - times_ifrerun_x * movesize_x;
				virtual_corner[1] = plane_center[1] - times_ifrerun_y * movesize_y;
			}
			else if (plane_center_origin_offset[0] > 0.f && plane_center_origin_offset[1] < 0.f)
			{
				virtual_corner[0] = plane_center[0] + times_ifrerun_x * movesize_x;
				virtual_corner[1] = plane_center[1] - times_ifrerun_y * movesize_y;
			}
		}
	}

	if (Rulers == 13) {
	
		float offsetX = (abs(minmax_xyz[0]) + abs(minmax_xyz[1])) / 3.0;
		float offsetY = (abs(minmax_xyz[2]) + abs(minmax_xyz[3])) / 3.0;

		if (ruler == 0)
		{
			virtual_corner[0] = minmax_xyz[0] + edgesize + times_ifrerun_x * movesize_x;
			virtual_corner[1] = minmax_xyz[2] + edgesize + times_ifrerun_y * movesize_y;
		}
		else if (ruler == 1)
		{
			virtual_corner[0] = minmax_xyz[1] - edgesize - times_ifrerun_x * movesize_x;
			virtual_corner[1] = minmax_xyz[2] + edgesize + times_ifrerun_y * movesize_y;
		}
		else if (ruler == 2)
		{
			virtual_corner[0] = minmax_xyz[1] - edgesize - times_ifrerun_x * movesize_x;
			virtual_corner[1] = minmax_xyz[3] - edgesize - times_ifrerun_y * movesize_y;
		}
		else if (ruler == 3)
		{
			virtual_corner[0] = minmax_xyz[0] + edgesize + times_ifrerun_x * movesize_x;
			virtual_corner[1] = minmax_xyz[3] - edgesize - times_ifrerun_y * movesize_y;
		}
		else if (ruler == 4)
		{
			virtual_corner[0] = minmax_xyz[0] + edgesize + times_ifrerun_x * movesize_x;
			virtual_corner[1] = (minmax_xyz[2] + minmax_xyz[3]) / 2 + times_ifrerun_y * movesize_y;
		}
		else if (ruler == 5)
		{
			virtual_corner[0] = (minmax_xyz[0] + minmax_xyz[1]) / 2 - times_ifrerun_x * movesize_x;
			virtual_corner[1] = minmax_xyz[2] + edgesize + times_ifrerun_y * movesize_y;
		}
		else if (ruler == 6)
		{
			virtual_corner[0] = minmax_xyz[1] - edgesize - times_ifrerun_x * movesize_x;
			virtual_corner[1] = (minmax_xyz[2] + minmax_xyz[3]) / 2 - times_ifrerun_y * movesize_y;
		}
		else if (ruler == 7)
		{
			virtual_corner[0] = (minmax_xyz[0] + minmax_xyz[1]) / 2 + times_ifrerun_x * movesize_x;
			virtual_corner[1] = minmax_xyz[3] - edgesize - times_ifrerun_y * movesize_y;
		}
		else if (ruler == 8)
		{
			virtual_corner[0] = minmax_xyz[0] + 1 * offsetX;
			virtual_corner[1] = minmax_xyz[2] + 2 * offsetY;
		}
		else if (ruler == 9)
		{
			virtual_corner[0] = minmax_xyz[0] + 1 * offsetX;
			virtual_corner[1] = minmax_xyz[2] + 1 * offsetY;
		}
		else if (ruler == 10)
		{
			virtual_corner[0] = minmax_xyz[0] + 2 * offsetX;
			virtual_corner[1] = minmax_xyz[2] + 1 * offsetY;
		}
		else if (ruler == 11)
		{
			virtual_corner[0] = minmax_xyz[0] + 2 * offsetX;
			virtual_corner[1] = minmax_xyz[2] + 2 * offsetY;
		}
		else 
		{
			float plane_center[2];
			plane_center[0] = (minmax_xyz[0] + minmax_xyz[1]) * 0.5f;
			plane_center[1] = (minmax_xyz[2] + minmax_xyz[3]) * 0.5f;
			float plane_center_origin_offset[2];
			plane_center_origin_offset[0] = plane_center[0] - rotated_origin.x;
			plane_center_origin_offset[1] = plane_center[1] - rotated_origin.y;
		
			if (plane_center_origin_offset[0] > 0.f && plane_center_origin_offset[1] > 0.f)
			{
				virtual_corner[0] = plane_center[0] + times_ifrerun_x * movesize_x;
				virtual_corner[1] = plane_center[1] + times_ifrerun_y * movesize_y;
			}
			else if (plane_center_origin_offset[0] < 0.f && plane_center_origin_offset[1] > 0.f)
			{
				virtual_corner[0] = plane_center[0] - times_ifrerun_x * movesize_x;
				virtual_corner[1] = plane_center[1] + times_ifrerun_y * movesize_y;
			}
			else if (plane_center_origin_offset[0] < 0.f && plane_center_origin_offset[1] < 0.f)
			{
				virtual_corner[0] = plane_center[0] - times_ifrerun_x * movesize_x;
				virtual_corner[1] = plane_center[1] - times_ifrerun_y * movesize_y;
			}
			else if (plane_center_origin_offset[0] > 0.f && plane_center_origin_offset[1] < 0.f)
			{
				virtual_corner[0] = plane_center[0] + times_ifrerun_x * movesize_x;
				virtual_corner[1] = plane_center[1] - times_ifrerun_y * movesize_y;
			}
		}
	//	cout <<"=======ruler: "<< ruler << "  virtual_corner: " << virtual_corner[0] << "  ,  " << virtual_corner[1] << endl;
	}
}

int MeasureLevelnessRange::ComputeFixLocationZMean(
	const std::vector<cv::Point3f>& plane1_points,
	const std::vector<cv::Point3f>& plane2_points,
	const float* minmax_xyz, const cv::Mat rotation_matrix,
	float* mean_z_1, float* mean_z_2,
	float* virtual_corner, cv::Point2f& single_actual_vertex)
{
	if (plane1_points.empty())
		return 0;

	single_actual_vertex = cv::Point2f(-1.f, -1.f);
	
	int pt_num_1, pt_num_2;

	pt_num_1 = MeasureBase::FindCornerPts(plane1_points, virtual_corner,
										  measure_ROI_length, offset_ROI_actualPts,
										  (unsigned int)(min_num_pts_inrulerband * 0.25f),
										  rotation_matrix, mean_z_1);
	pt_num_2 = MeasureBase::FindCornerPts(plane2_points, virtual_corner,
										  measure_ROI_length, offset_ROI_actualPts,
										  (unsigned int)(min_num_pts_inrulerband * 0.25f),
										  rotation_matrix, mean_z_2);
	if (pt_num_1 >= min_num_pts_inrulerband && pt_num_2 >= min_num_pts_inrulerband)
	{
		single_actual_vertex = cv::Point2f(virtual_corner[0], virtual_corner[1] + measure_ROI_dist_from_plane_edge);
		return 1;
	}
	return -1;
}


std::pair<int,int> MeasureLevelnessRange::ComputeCornersCenterZMean(
	int Rulers,
	const std::vector<cv::Point3f>& plane1_points,
	const std::vector<cv::Point3f>& plane2_points,
	const float* minmax_xyz, const int ruler, const cv::Mat rotation_matrix,
	float* mean_z_1, float* mean_z_2, 
	float* virtual_corner, cv::Point2f& single_actual_vertex)
{
	if (plane1_points.empty())
		return{0,0};
	single_actual_vertex = cv::Point2f(-1.f, -1.f);
	//initial distance between measurement ROI CENTER and plane edge
	float edgesize = measure_ROI_dist_from_plane_edge;
	int total_times_ifrerun = min(std::floor(min(-minmax_xyz[0] + minmax_xyz[1], -minmax_xyz[2] + minmax_xyz[3]) / measure_ROI_length * 2.f), 100);
	float movesize_x, movesize_y;
	if (!MeasureBase::IsValZero(total_times_ifrerun))
	{
		movesize_x = std::floor((-minmax_xyz[0] + minmax_xyz[1] - 2.f * edgesize) / total_times_ifrerun * 0.4f);
		movesize_y = std::floor((-minmax_xyz[2] + minmax_xyz[3] - 2.f * edgesize) / total_times_ifrerun * 0.4f);
	}
	else
	{
		return{0,0};
	}
	int moved_cnt = 0;
	int pt_num_1, pt_num_2;
	for (int times_ifrerun_x = 0; times_ifrerun_x < total_times_ifrerun; times_ifrerun_x++)
	{
		for (int times_ifrerun_y = 0; times_ifrerun_y < total_times_ifrerun; times_ifrerun_y++)
		{
			moved_cnt++;
			//virtual_corner is ROI CENTER, vertex sequence of (x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max)
			this->AssignMeasureVirtualCorner(Rulers, &minmax_xyz[0], edgesize, times_ifrerun_x, times_ifrerun_y, movesize_x, movesize_y, ruler, &virtual_corner[0]);

			if (plane2_points.empty())///for levelness
			{
				pt_num_1 = MeasureBase::FindCornerPts(plane1_points, virtual_corner, 
					                                  measure_ROI_length, offset_ROI_actualPts, 
					                                  (unsigned int)(min_num_pts_inrulerband * 0.25f), 
					                                  rotation_matrix, mean_z_1);
				if (pt_num_1 >= min_num_pts_inrulerband)
				{
#ifdef OUTPUT_DEBUG_LEVEL_INFO
					SaveROIPoints(plane1_points, &virtual_corner[0], ruler);
#endif
					//assign vertex
					if (ruler == 0)	single_actual_vertex = cv::Point2f(virtual_corner[0] - edgesize, virtual_corner[1] - edgesize);
					if (ruler == 1)	single_actual_vertex = cv::Point2f(virtual_corner[0] + edgesize, virtual_corner[1] - edgesize);
					if (ruler == 2)	single_actual_vertex = cv::Point2f(virtual_corner[0] + edgesize, virtual_corner[1] + edgesize);
					if (ruler == 3)	single_actual_vertex = cv::Point2f(virtual_corner[0] - edgesize, virtual_corner[1] + edgesize);
					if (ruler == 4)	single_actual_vertex = cv::Point2f(virtual_corner[0] - edgesize, virtual_corner[1]);
					if (ruler == 5)	single_actual_vertex = cv::Point2f(virtual_corner[0], virtual_corner[1] - edgesize);
					if (ruler == 6)	single_actual_vertex = cv::Point2f(virtual_corner[0] + edgesize, virtual_corner[1]);
					if (ruler == 7)	single_actual_vertex = cv::Point2f(virtual_corner[0], virtual_corner[1] + edgesize);
					return{ moved_cnt,total_times_ifrerun};
				}
			}
			else  ///for height
			{
				pt_num_1 = MeasureBase::FindCornerPts(plane1_points, virtual_corner, 
					                                  measure_ROI_length, offset_ROI_actualPts, 
					                                  (unsigned int)(min_num_pts_inrulerband * 0.25f), 
					                                  rotation_matrix, mean_z_1);
				pt_num_2 = MeasureBase::FindCornerPts(plane2_points, virtual_corner, 
					                                  measure_ROI_length, offset_ROI_actualPts, 
													  (unsigned int)(min_num_pts_inrulerband * 0.25f), 
					                                  rotation_matrix, mean_z_2);
				if (pt_num_1 >= min_num_pts_inrulerband && pt_num_2 >= min_num_pts_inrulerband)
				{
					if (ruler == 0)	single_actual_vertex = cv::Point2f(virtual_corner[0] - edgesize, virtual_corner[1] - edgesize);
					if (ruler == 1)	single_actual_vertex = cv::Point2f(virtual_corner[0] + edgesize, virtual_corner[1] - edgesize);
					if (ruler == 2)	single_actual_vertex = cv::Point2f(virtual_corner[0] + edgesize, virtual_corner[1] + edgesize);
					if (ruler == 3)	single_actual_vertex = cv::Point2f(virtual_corner[0] - edgesize, virtual_corner[1] + edgesize);
					if (ruler == 4)	single_actual_vertex = cv::Point2f(virtual_corner[0] - edgesize, virtual_corner[1]);
					if (ruler == 5)	single_actual_vertex = cv::Point2f(virtual_corner[0], virtual_corner[1] - edgesize);
					if (ruler == 6)	single_actual_vertex = cv::Point2f(virtual_corner[0] + edgesize, virtual_corner[1]);
					if (ruler == 7)	single_actual_vertex = cv::Point2f(virtual_corner[0], virtual_corner[1] + edgesize);
					return { moved_cnt,total_times_ifrerun };
				}
			}
		}
	}
	return{ -1,total_times_ifrerun };
}

std::pair<int, int> MeasureLevelnessRange::ComputeCornersCenterZMeanAll(
	cv::Point2f &Pt,
	float &mLen,
	const std::vector<cv::Point3f>& plane1_points,
	const std::vector<cv::Point3f>& plane2_points,
	const float* minmax_xyz, const cv::Mat rotation_matrix,
	float* mean_z_1, float* mean_z_2,
	float* virtual_corner, cv::Point2f& single_actual_vertex)
{
	if (plane1_points.empty())
		return{ 0,0 };
	single_actual_vertex = cv::Point2f(-1.f, -1.f);

	int pt_num_1, pt_num_2;
	float ori_x, ori_y;

	int i;
	int total_times_ifrerun = 2;
	float moveSz = mLen/(2*total_times_ifrerun);
	//cout << "Total_times_ifrerun:" << total_times_ifrerun << " moveSz:" << moveSz << endl;
	for (i = 0; i < total_times_ifrerun ; i++)
	{
		virtual_corner[0] = ori_x = Pt.x - i * moveSz;
		virtual_corner[1] = ori_y = Pt.y - i * moveSz;
		if (plane2_points.empty())///for levelness
		{
			pt_num_1 = MeasureBase::FindCornerPts(plane1_points, virtual_corner,
				measure_ROI_length, offset_ROI_actualPts,
				(unsigned int)(min_num_pts_inrulerband * 0.25f),
				rotation_matrix, mean_z_1);
			if (pt_num_1 >= min_num_pts_inrulerband)
			{
				return{ 1,1 };
			}
		}
		else  ///for height
		{
			pt_num_1 = MeasureBase::FindCornerPts(plane1_points, virtual_corner,
				measure_ROI_length, offset_ROI_actualPts,
				(unsigned int)(min_num_pts_inrulerband * 0.25f),
				rotation_matrix, mean_z_1);
			pt_num_2 = MeasureBase::FindCornerPts(plane2_points, virtual_corner,
				measure_ROI_length, offset_ROI_actualPts,
				(unsigned int)(min_num_pts_inrulerband * 0.25f),
				rotation_matrix, mean_z_2);
			if (pt_num_1 >= min_num_pts_inrulerband && pt_num_2 >= min_num_pts_inrulerband)
			{
				return{ 1,1 };
			}
		}

		int x = 1;
		for (; x <= i * 2; x++) {
			virtual_corner[0] = ori_x + x * moveSz;
			if (plane2_points.empty())///for levelness
			{
				pt_num_1 = MeasureBase::FindCornerPts(plane1_points, virtual_corner,
					measure_ROI_length, offset_ROI_actualPts,
					(unsigned int)(min_num_pts_inrulerband * 0.25f),
					rotation_matrix, mean_z_1);
				if (pt_num_1 >= min_num_pts_inrulerband)
				{
					return{ 1,1 };
				}
			}
			else  ///for height
			{
				pt_num_1 = MeasureBase::FindCornerPts(plane1_points, virtual_corner,
					measure_ROI_length, offset_ROI_actualPts,
					(unsigned int)(min_num_pts_inrulerband * 0.25f),
					rotation_matrix, mean_z_1);
				pt_num_2 = MeasureBase::FindCornerPts(plane2_points, virtual_corner,
					measure_ROI_length, offset_ROI_actualPts,
					(unsigned int)(min_num_pts_inrulerband * 0.25f),
					rotation_matrix, mean_z_2);
				if (pt_num_1 >= min_num_pts_inrulerband && pt_num_2 >= min_num_pts_inrulerband)
				{
					return{ 1,1 };
				}
			}
		}
	
		x--;
		int y = 1;
		if (x == i * 2) {
			for (; y <= i * 2; y++) {
				virtual_corner[1] = ori_y + y * moveSz;
				if (plane2_points.empty())///for levelness
				{
					pt_num_1 = MeasureBase::FindCornerPts(plane1_points, virtual_corner,
						measure_ROI_length, offset_ROI_actualPts,
						(unsigned int)(min_num_pts_inrulerband * 0.25f),
						rotation_matrix, mean_z_1);
					if (pt_num_1 >= min_num_pts_inrulerband)
					{
						return{ 1,1 };
					}
				}
				else  ///for height
				{
					pt_num_1 = MeasureBase::FindCornerPts(plane1_points, virtual_corner,
						measure_ROI_length, offset_ROI_actualPts,
						(unsigned int)(min_num_pts_inrulerband * 0.25f),
						rotation_matrix, mean_z_1);
					pt_num_2 = MeasureBase::FindCornerPts(plane2_points, virtual_corner,
						measure_ROI_length, offset_ROI_actualPts,
						(unsigned int)(min_num_pts_inrulerband * 0.25f),
						rotation_matrix, mean_z_2);
					if (pt_num_1 >= min_num_pts_inrulerband && pt_num_2 >= min_num_pts_inrulerband)
					{
						return{ 1,1 };
					}
				}
			}
		}
	
		y--;
		if (y == i * 2) {
			for (x--; x >= 0; x--) {
				virtual_corner[0] = ori_x + x * moveSz;
				if (plane2_points.empty())///for levelness
				{
					pt_num_1 = MeasureBase::FindCornerPts(plane1_points, virtual_corner,
						measure_ROI_length, offset_ROI_actualPts,
						(unsigned int)(min_num_pts_inrulerband * 0.25f),
						rotation_matrix, mean_z_1);
					if (pt_num_1 >= min_num_pts_inrulerband)
					{
						return{ 1,1 };
					}
				}
				else  ///for height
				{
					pt_num_1 = MeasureBase::FindCornerPts(plane1_points, virtual_corner,
						measure_ROI_length, offset_ROI_actualPts,
						(unsigned int)(min_num_pts_inrulerband * 0.25f),
						rotation_matrix, mean_z_1);
					pt_num_2 = MeasureBase::FindCornerPts(plane2_points, virtual_corner,
						measure_ROI_length, offset_ROI_actualPts,
						(unsigned int)(min_num_pts_inrulerband * 0.25f),
						rotation_matrix, mean_z_2);
					if (pt_num_1 >= min_num_pts_inrulerband && pt_num_2 >= min_num_pts_inrulerband)
					{
						return{ 1,1 };
					}
				}
			}
		}

		x++;
		if (x == 0) {
			for (y--; y > 0; y--) {
				virtual_corner[1] = ori_y + y * moveSz;
				if (plane2_points.empty())///for levelness
				{
					pt_num_1 = MeasureBase::FindCornerPts(plane1_points, virtual_corner,
						measure_ROI_length, offset_ROI_actualPts,
						(unsigned int)(min_num_pts_inrulerband * 0.25f),
						rotation_matrix, mean_z_1);
					if (pt_num_1 >= min_num_pts_inrulerband)
					{
						return{ 1,1 };
					}
				}
				else  ///for height
				{
					pt_num_1 = MeasureBase::FindCornerPts(plane1_points, virtual_corner,
						measure_ROI_length, offset_ROI_actualPts,
						(unsigned int)(min_num_pts_inrulerband * 0.25f),
						rotation_matrix, mean_z_1);
					pt_num_2 = MeasureBase::FindCornerPts(plane2_points, virtual_corner,
						measure_ROI_length, offset_ROI_actualPts,
						(unsigned int)(min_num_pts_inrulerband * 0.25f),
						rotation_matrix, mean_z_2);
					if (pt_num_1 >= min_num_pts_inrulerband && pt_num_2 >= min_num_pts_inrulerband)
					{
						return{ 1,1 };
					}
				}
			}
		}
	}
			return{ -1,0 };
}


std::pair<int, int> MeasureLevelnessRange::ComputeCornersCenterZMeanFix(
	cv::Point2f &Pt,
	float &mLen,
	const std::vector<cv::Point3f>& plane1_points,
	const std::vector<cv::Point3f>& plane2_points,
	const float* minmax_xyz, const cv::Mat rotation_matrix,
	float* mean_z_1, float* mean_z_2,
	float* virtual_corner, cv::Point2f& single_actual_vertex)
{
	if (plane1_points.empty())
		return{ 0,0 };
	single_actual_vertex = cv::Point2f(-1.f, -1.f);
	int pt_num_1, pt_num_2;
	float measure_ROI_Fix_length = 40; //T
	float offset_ROI_Fix_actualPts = measure_ROI_Fix_length;//T
	unsigned int min_pt_num_fix_in_quarter = 5;//T
	virtual_corner[0] = Pt.x;
	virtual_corner[1] = Pt.y;
	pt_num_1 = MeasureBase::FindCornerPts(plane1_points, virtual_corner,
		measure_ROI_Fix_length, offset_ROI_Fix_actualPts,
		min_pt_num_fix_in_quarter,
		rotation_matrix, mean_z_1);
	pt_num_2 = MeasureBase::FindCornerPts(plane2_points, virtual_corner,
		measure_ROI_Fix_length, offset_ROI_Fix_actualPts,
		min_pt_num_fix_in_quarter,
		rotation_matrix, mean_z_2);
	if (pt_num_1 >= min_pt_num_fix_in_quarter*4 && pt_num_2 >= min_pt_num_fix_in_quarter*4)
	{
		return{ 1,1 };
	}
	return{ -1,0 };
}


void MeasureLevelnessRange::ReArrangeRuleWithCompass(
	std::vector<cv::Point2f>& markerPoint, 
	int rulers, float CompassAng,
	MeasurementResultValueValuesPointsT& levelnessTemp,
	std::vector<cv::Point3f>& actual_verticesTemp,
	MeasurementResultValueValuesPoints& levelness,
	std::vector<cv::Point3f>& actual_vertices)
{
	int RulerNorthIdx = -1;
	float CompassNorthAng = CompassAng;
	if (CompassNorthAng < 0)
		CompassNorthAng += 360;
#ifdef DBG_L
	for (int i = 0; i < markerPoint.size(); i++) {
		cout << "i: " << i << " markerPoint:" << markerPoint[i] << endl;
	}
#endif
	float a = 0;
	float b = -1;
	float c = 0;
	cv::Point2f DirPoint;
	std::pair<int, float> DirIdxAng = { 0,0 };
	std::vector<std::pair<int, float>> DirIdxCalAng;
	std::vector<float> Dis;
	DirIdxCalAng.resize(4);
	Dis.resize(4);
	int DirNorAng = -1;
	float DirNorAngF = -1.0;
	for (int i = 0; i < DirIdxCalAng.size(); i++) {
		DirIdxCalAng[i].first = 0;
		DirIdxCalAng[i].second = 0;
	}

{
	if (markerPoint[0].x == markerPoint[1].x) {
		DirPoint.x = markerPoint[0].x;
		DirPoint.y = 0;
	}
	else {
		a = (markerPoint[0].y - markerPoint[1].y) /
			(markerPoint[0].x - markerPoint[1].x);
		c = markerPoint[0].y - a*markerPoint[0].x;
		DirPoint.x = (-a*c) / (a*a + b*b);
		DirPoint.y = (-b*c) / (a*a + b*b);
	}
	DirIdxCalAng[1].first = 5;
	DirIdxCalAng[1].second = atan2(DirPoint.y, DirPoint.x) * 180 / M_PI;
	Dis[1] = std::sqrt(std::pow(DirPoint.x, 2) + std::pow(DirPoint.y, 2));
#ifdef DBG_L
	cout << markerPoint[0] << " ," << markerPoint[1] << endl;
	cout << "DirPoint.x:" << DirPoint.x << " DirPoint.y:" << DirPoint.y << endl;
	cout << "DirIdxCalAng[1].first:" << DirIdxCalAng[1].first << " DirIdxCalAng[1].second: " << DirIdxCalAng[1].second << endl;
#endif
}

{
	if (markerPoint[2].x == markerPoint[3].x) {
		DirPoint.x = markerPoint[2].x;
		DirPoint.y = 0;
	}
	else {
		a = (markerPoint[2].y - markerPoint[3].y) /
			(markerPoint[2].x - markerPoint[3].x);
		c = markerPoint[2].y - a*markerPoint[2].x;
		DirPoint.x = (-a*c) / (a*a + b*b);
		DirPoint.y = (-b*c) / (a*a + b*b);
	}
	DirIdxCalAng[3].first = 7;
	DirIdxCalAng[3].second = atan2(DirPoint.y, DirPoint.x) * 180 / M_PI;
	Dis[3] = std::sqrt(std::pow(DirPoint.x, 2) + std::pow(DirPoint.y, 2));

#ifdef DBG_L
	cout << markerPoint[2] << " ," << markerPoint[3] << endl;
	cout << "DirPoint.x:" << DirPoint.x << " DirPoint.y:" << DirPoint.y << endl;
	cout << "DirIdxCalAng[3].first:" << DirIdxCalAng[3].first << " DirIdxCalAng[3].second: " << DirIdxCalAng[3].second << endl;
#endif
}
{
	if (std::floor(DirIdxCalAng[1].second) == std::floor(DirIdxCalAng[3].second))
	{
		if (Dis[3] > Dis[1]) {
			DirIdxAng.first = 7;
			if (DirIdxCalAng[3].second < 0)
				DirIdxAng.second = DirIdxCalAng[3].second + 360.0;
			else
				DirIdxAng.second = DirIdxCalAng[3].second;
		}
		else {
			DirIdxAng.first = 5;
			if (DirIdxCalAng[1].second < 0)
				DirIdxAng.second = DirIdxCalAng[1].second + 360.0;
			else
				DirIdxAng.second = DirIdxCalAng[1].second;
		}
	}
	else {
		DirIdxAng.first = 5;
		if (DirIdxCalAng[1].second < 0)
			DirIdxAng.second = DirIdxCalAng[1].second + 360.0;
		else
			DirIdxAng.second = DirIdxCalAng[1].second;
	}
}
	DirNorAng = std::floor((DirIdxAng.second + CompassNorthAng) * 10000);
	DirNorAng %= 3600000;
	DirNorAngF = DirNorAng / 10000.0;
#ifdef DBG_L
	cout << "CompassNorthAng:" << CompassNorthAng << " DirNorAng :" << DirNorAng << " DirNorAngF:" << DirNorAngF << endl;
#endif
	if ((DirNorAngF > 45.0) && (DirNorAngF <= 135.0)) { //E
		if (DirIdxAng.first == 5) RulerNorthIdx = 4;
		if (DirIdxAng.first == 7) RulerNorthIdx = 6;
	}
	else if ((DirNorAngF > 135.0) && (DirNorAngF <= 225.0)) { //S
		if (DirIdxAng.first == 5) RulerNorthIdx = 7;
		if (DirIdxAng.first == 7) RulerNorthIdx = 5;
	}
	else if ((DirNorAngF > 225.0) && (DirNorAngF <= 315.0)) { //W
		if (DirIdxAng.first == 5) RulerNorthIdx = 6;
		if (DirIdxAng.first == 7) RulerNorthIdx = 4;
	}
	else {
		if ((DirIdxAng.first == 5) || (DirIdxAng.first == 7))
		{
			RulerNorthIdx = DirIdxAng.first;
		}
		else {

		}
	}
	if (DirIdxAng.first == 0)
	{
		RulerNorthIdx = 4;
		std::cout << "Can't find valid points pair !!!!" << std::endl;
	}
#ifdef DBG_L
	cout << "RulerNorthIdx: " << RulerNorthIdx << endl;
#endif

	MeasurementResultValueValuesPoints levelnessP;
	levelnessP.is_valid = levelnessTemp.is_valid;
	levelnessP.value = levelnessTemp.value;
	for (int i = 0; i < levelnessTemp.points.size(); i++)
	{
		levelnessP.points.push_back(std::tuple<bool, float, cv::Point3f>
						{std::get<0>(levelnessTemp.points[i]),
						 std::get<1>(levelnessTemp.points[i]),
						 std::get<2>(levelnessTemp.points[i])});
	}

	if (RulerNorthIdx == 4) {
		levelness.points.push_back(levelnessP.points[0]);
		actual_vertices.push_back(actual_verticesTemp[0]);
		if (rulers == 9){
			levelness.points.push_back(levelnessP.points[5]);
			actual_vertices.push_back(actual_verticesTemp[5]);
		}
		levelness.points.push_back(levelnessP.points[1]);
		actual_vertices.push_back(actual_verticesTemp[1]);
		if (rulers == 9){
			levelness.points.push_back(levelnessP.points[4]);
			actual_vertices.push_back(actual_verticesTemp[4]);
			levelness.points.push_back(levelnessP.points[8]);
			//actual_vertices.push_back(actual_verticesTemp[8]);
			levelness.points.push_back(levelnessP.points[6]);
			actual_vertices.push_back(actual_verticesTemp[6]);
		}
		if (rulers == 5) {
			levelness.points.push_back(levelnessP.points[4]);
		//	actual_vertices.push_back(actual_verticesTemp[4]);
		}
		levelness.points.push_back(levelnessP.points[3]);
		actual_vertices.push_back(actual_verticesTemp[3]);
		if (rulers == 9) {
			levelness.points.push_back(levelnessP.points[7]);
			actual_vertices.push_back(actual_verticesTemp[7]);
		}
		levelness.points.push_back(levelnessP.points[2]);
		actual_vertices.push_back(actual_verticesTemp[2]);
	}
	else if (RulerNorthIdx == 5) {
		levelness.points.push_back(levelnessP.points[1]);
		actual_vertices.push_back(actual_verticesTemp[1]);
		if (rulers == 9) {
			levelness.points.push_back(levelnessP.points[6]);
			actual_vertices.push_back(actual_verticesTemp[6]);
		}
		levelness.points.push_back(levelnessP.points[2]);
		actual_vertices.push_back(actual_verticesTemp[2]);
		if (rulers == 9) {
			levelness.points.push_back(levelnessP.points[5]);
			actual_vertices.push_back(actual_verticesTemp[5]);
			levelness.points.push_back(levelnessP.points[8]);
	//		actual_vertices.push_back(actual_verticesTemp[8]);
			levelness.points.push_back(levelnessP.points[7]);
			actual_vertices.push_back(actual_verticesTemp[7]);
		}
		if (rulers == 5) {
			levelness.points.push_back(levelnessP.points[4]);
		//	actual_vertices.push_back(actual_verticesTemp[4]);
		}
		levelness.points.push_back(levelnessP.points[0]);
		actual_vertices.push_back(actual_verticesTemp[0]);
		if (rulers == 9) {
			levelness.points.push_back(levelnessP.points[4]);
			actual_vertices.push_back(actual_verticesTemp[4]);
		}
		levelness.points.push_back(levelnessP.points[3]);
		actual_vertices.push_back(actual_verticesTemp[3]);

	}
	else if (RulerNorthIdx == 6) {
		levelness.points.push_back(levelnessP.points[2]);
		actual_vertices.push_back(actual_verticesTemp[2]);
		if (rulers == 9) {
			levelness.points.push_back(levelnessP.points[7]);
			actual_vertices.push_back(actual_verticesTemp[7]);
		}
		levelness.points.push_back(levelnessP.points[3]);
		actual_vertices.push_back(actual_verticesTemp[3]);
		if (rulers == 9) {
			levelness.points.push_back(levelnessP.points[6]);
			actual_vertices.push_back(actual_verticesTemp[6]);
			levelness.points.push_back(levelnessP.points[8]);
		//	actual_vertices.push_back(actual_verticesTemp[8]);
			levelness.points.push_back(levelnessP.points[4]);
			actual_vertices.push_back(actual_verticesTemp[4]);
		}
		if (rulers == 5) {
			levelness.points.push_back(levelnessP.points[4]);
		//	actual_vertices.push_back(actual_verticesTemp[4]);
		}
		levelness.points.push_back(levelnessP.points[1]);
		actual_vertices.push_back(actual_verticesTemp[1]);
		if (rulers == 9) {
			levelness.points.push_back(levelnessP.points[5]);
			actual_vertices.push_back(actual_verticesTemp[5]);
		}
		levelness.points.push_back(levelnessP.points[0]);
		actual_vertices.push_back(actual_verticesTemp[0]);
	}
	else if (RulerNorthIdx == 7) {
		levelness.points.push_back(levelnessP.points[3]);
		actual_vertices.push_back(actual_verticesTemp[3]);
		if (rulers == 9) {
			levelness.points.push_back(levelnessP.points[4]);
			actual_vertices.push_back(actual_verticesTemp[4]);
		}
		levelness.points.push_back(levelnessP.points[0]);
		actual_vertices.push_back(actual_verticesTemp[0]);
		if (rulers == 9) {
			levelness.points.push_back(levelnessP.points[7]);
			actual_vertices.push_back(actual_verticesTemp[7]);
			levelness.points.push_back(levelnessP.points[8]);
		//	actual_vertices.push_back(actual_verticesTemp[8]);
			levelness.points.push_back(levelnessP.points[5]);
			actual_vertices.push_back(actual_verticesTemp[5]);
		}
		if (rulers == 5) {
			levelness.points.push_back(levelnessP.points[4]);
		//	actual_vertices.push_back(actual_verticesTemp[4]);
		}
		levelness.points.push_back(levelnessP.points[2]);
		actual_vertices.push_back(actual_verticesTemp[2]);
		if (rulers == 9) {
			levelness.points.push_back(levelnessP.points[6]);
			actual_vertices.push_back(actual_verticesTemp[6]);
		}
		levelness.points.push_back(levelnessP.points[1]);
		actual_vertices.push_back(actual_verticesTemp[1]);
	}
}

bool MeasureLevelnessRange::ComputeLevelnessRange(const std::vector<cv::Point3f> &plane_points,
	const float *minmax_xyz,
	const cv::Mat rotation_matrix_to_z,
	const float rotation_angle_around_z,
	MeasurementResultValueValuesPoints& levelness,
	std::vector<cv::Point3f>& actual_vertices)
{ 
	if (plane_points.empty())
		return false;

	float mean_z_1;
	std::vector<cv::Point3f> empty_plane_points;
	float mean_z_2;
	float virtual_corner[2];

	float cornerpt_z_max = -INFINITY;
	float cornerpt_z_min = INFINITY;
	float temp_measure_pt_x, temp_measure_pt_y;
	cv::Point3f backward_rotated_pt;
	cv::Point2f single_actual_vertex;
	std::vector<bool> is_valid;

	MeasurementResultValueValuesPointsT levelnessTemp;
	std::vector<cv::Point3f> actual_verticesTemp;
	int Rulers = 5;
	if (MEASURE_POSITION_ALL == GetCeilingLevelnessMeasurePosition()) {
		Rulers = 9;
	}
	if (MEASURE_POSITION_CORNER_AND_MIDDLE == GetCeilingLevelnessMeasurePosition()) {
		Rulers = 5;
	}
	//Rulers = 5;
	for (int ruler = 0; ruler < Rulers; ruler++)
	{
		std::pair<int, int> ret = this->ComputeCornersCenterZMean(Rulers, plane_points, empty_plane_points, minmax_xyz, ruler, rotation_matrix_to_z,
			&mean_z_1, &mean_z_2, virtual_corner, single_actual_vertex);
		
		if (ret.first > 0)
		{
			mean_z_1 = std::floor(mean_z_1 + 0.5);
			is_valid.push_back(true);
			cornerpt_z_max = max(cornerpt_z_max, mean_z_1);
			cornerpt_z_min = min(cornerpt_z_min, mean_z_1);

			//actual measurement pts
			temp_measure_pt_x = virtual_corner[0];
			temp_measure_pt_y = virtual_corner[1];
			//backward rotation by -rotation_angle_around_z
			backward_rotated_pt.x = temp_measure_pt_x * cos(rotation_angle_around_z) + temp_measure_pt_y * sin(rotation_angle_around_z);
			backward_rotated_pt.y = -temp_measure_pt_x * sin(rotation_angle_around_z) + temp_measure_pt_y * cos(rotation_angle_around_z);
			backward_rotated_pt.z = mean_z_1;
#ifdef DBG_L
			cout << "leveness rule: "  << backward_rotated_pt  <<"  mean_z_1:"<< mean_z_1 << endl;
#endif
			levelnessTemp.points.push_back(std::tuple<bool, float, cv::Point3f,int,int> {true, mean_z_1, backward_rotated_pt,ret.first,ret.second});
			if (ruler < Rulers -1)
			{
				backward_rotated_pt.x = single_actual_vertex.x * cos(rotation_angle_around_z) + single_actual_vertex.y * sin(rotation_angle_around_z);
				backward_rotated_pt.y = -single_actual_vertex.x * sin(rotation_angle_around_z) + single_actual_vertex.y * cos(rotation_angle_around_z);
				backward_rotated_pt.z = mean_z_1;
				actual_verticesTemp.push_back(backward_rotated_pt);
			}
		}
		else if(ret.first == -1){
#ifdef DBG_L
			//cout << "leveness rule:" << ruler << "  move_cnt:" << ret.first << endl;
#endif
			levelnessTemp.points.push_back(std::tuple<bool, float, cv::Point3f, int,int> {true, 0, cv::Point3f(-1.f, -1.f, -1.f), ret.first, ret.second});
			if (ruler < Rulers - 1) {
				actual_verticesTemp.push_back(cv::Point3f(-1.f, -1.f, -1.f));
			}
		}
		else 
		{
			levelnessTemp.points.push_back(std::tuple<bool, float, cv::Point3f,int,int> {false, -1.f, cv::Point3f(-1.f, -1.f, -1.f), ret.first, ret.second});
			if (ruler < Rulers -1) {
				actual_verticesTemp.push_back(cv::Point3f(-1.f, -1.f, -1.f));
			}
		}
	}

//	CCompass compass;
//	float CompassNorthAng = compass.getCompassVal();
	float CompassNorthAng = 0;
	std::vector<cv::Point2f> MarkerPoint;
	cv::Point2f temp;
	float actual_top_minmax_xy[4];
	temp.x = minmax_xyz[0] * cos(rotation_angle_around_z) + minmax_xyz[2] * sin(rotation_angle_around_z);
	temp.y = -minmax_xyz[0] * sin(rotation_angle_around_z) + minmax_xyz[2] * cos(rotation_angle_around_z);
	MarkerPoint.push_back(temp);
	temp.x = minmax_xyz[1] * cos(rotation_angle_around_z) + minmax_xyz[2] * sin(rotation_angle_around_z);
	temp.y = -minmax_xyz[1] * sin(rotation_angle_around_z) + minmax_xyz[2] * cos(rotation_angle_around_z);
	MarkerPoint.push_back(temp);
	temp.x = minmax_xyz[1] * cos(rotation_angle_around_z) + minmax_xyz[3] * sin(rotation_angle_around_z);
	temp.y = -minmax_xyz[1] * sin(rotation_angle_around_z) + minmax_xyz[3] * cos(rotation_angle_around_z);
	MarkerPoint.push_back(temp);
	temp.x = minmax_xyz[0] * cos(rotation_angle_around_z) + minmax_xyz[3] * sin(rotation_angle_around_z);
	temp.y = -minmax_xyz[0] * sin(rotation_angle_around_z) + minmax_xyz[3] * cos(rotation_angle_around_z);
	MarkerPoint.push_back(temp);
	ReArrangeRuleWithCompass(MarkerPoint, Rulers,CompassNorthAng,levelnessTemp,actual_verticesTemp,levelness, actual_vertices);
	
	if (is_valid.size() < 2)
	{
		levelness.is_valid = false;
		levelness.value = -1.f;
		return false;
	}


	levelness.is_valid = true;
	levelness.value = cornerpt_z_max - cornerpt_z_min;


#ifdef DBG_L

	for (int j = 0; j < levelness.points.size(); j++) {
		std::pair<float, cv::Point3f> temp;
		if (std::get<0>(levelness.points[j])) {
			temp.first = std::get<1>(levelness.points[j]);
			temp.second = std::get<2>(levelness.points[j]);
		}
	}
	cout << "ComputeLevelnessRange" << " levelness.value:" << levelness.value << endl;
#endif
#ifdef OUTPUT_DEBUG_LEVEL_INFO
	local_rulerplane_pts.Compute();
	std::cout << "ceiling local ruler normal: " << local_rulerplane_pts.plane_normals[0] << ", " << local_rulerplane_pts.plane_normals[1] << ", " << local_rulerplane_pts.plane_normals[2] << std::endl;
#endif


	////if hole in middle, value in middle is the average of valid corners
	//if (!std::get<0>(levelness.points[levelness.points.size() - 1]))
	//{
	//	float levelness_avg = 0.f;
	//	int valid_count = 0;
	//	cv::Point3f temp_point = cv::Point3f(0.f, 0.f, 0.f);
	//	for (int i = 0; i < levelness.points.size() - 1; i++)
	//	{
	//		if (std::get<0>(levelness.points[i]))
	//		{
	//			valid_count++;
	//			levelness_avg += std::get<1>(levelness.points[i]);
	//			temp_point += std::get<2>(levelness.points[i]);
	//		}
	//	}
	//	std::get<0>(levelness.points[levelness.points.size() - 1]) = true;
	//	std::get<1>(levelness.points[levelness.points.size() - 1]) = levelness_avg / (float)valid_count;
	//	std::get<2>(levelness.points[levelness.points.size() - 1]) = temp_point / (float)valid_count;
	//}

	return true;
}


bool MeasureLevelnessRange::ComputeLevelnessRangeAll(const std::vector<cv::Point3f> &plane_points,
	const float *minmax_xyz,
	const cv::Mat rotation_matrix_to_z,
	const float rotation_angle_around_z,
	MeasurementResultValueValuesPoints& levelness,
	std::vector<cv::Point3f>& actual_vertices)
{
	if (plane_points.empty())
		return false;

	float mean_z_1;
	std::vector<cv::Point3f> empty_plane_points;
	float mean_z_2;
	float virtual_corner[2];

	float cornerpt_z_max = -INFINITY;
	float cornerpt_z_min = INFINITY;
	float temp_measure_pt_x, temp_measure_pt_y;
	cv::Point3f backward_rotated_pt;
	cv::Point2f single_actual_vertex;
	std::vector<bool> is_valid;

	MeasurementResultValueValuesPointsT levelnessTemp;
	std::vector<cv::Point3f> actual_verticesTemp;

	cv::Point2f Pt;
	float mLen = 1000.0;
	float lineWideth = 0;
	int xCnt, yCnt;
	float xLen, yLen;

	if ((minmax_xyz[1] - minmax_xyz[0]) / mLen > 3.0) {
		xCnt = std::floor((minmax_xyz[1] - minmax_xyz[0]) / mLen);
	}
	else {
		xCnt = 3;
	}
	xLen = (minmax_xyz[1] - minmax_xyz[0]) / xCnt;

	if ((minmax_xyz[3] - minmax_xyz[2]) / mLen > 3.0) {
		yCnt = std::floor((minmax_xyz[3] - minmax_xyz[2]) / mLen);
	}
	else {
		yCnt = 3;
	}
	yLen = (minmax_xyz[3] - minmax_xyz[2]) / yCnt;
	if (xLen < yLen)
		lineWideth = xLen;
	else
		lineWideth = yLen;



	for (int i = 0; i < xCnt; i++)
	{
		Pt.x = minmax_xyz[0] + xLen / 2.0 + i * xLen;
		for (int j = 0; j < yCnt; j++)
		{
			Pt.y = minmax_xyz[3] - yLen / 2.0 - j * yLen;

			std::pair<int, int> ret = this->ComputeCornersCenterZMeanAll(Pt, lineWideth, plane_points, empty_plane_points,
				                           minmax_xyz, rotation_matrix_to_z,
			&mean_z_1, &mean_z_2, virtual_corner, single_actual_vertex);

		if (ret.first > 0)
		{
			mean_z_1 = std::floor(mean_z_1 + 0.5);
			is_valid.push_back(true);
			cornerpt_z_max = max(cornerpt_z_max, mean_z_1);
			cornerpt_z_min = min(cornerpt_z_min, mean_z_1);

			//actual measurement pts
			temp_measure_pt_x = virtual_corner[0];
			temp_measure_pt_y = virtual_corner[1];
			//backward rotation by -rotation_angle_around_z
			backward_rotated_pt.x = temp_measure_pt_x * cos(rotation_angle_around_z) + temp_measure_pt_y * sin(rotation_angle_around_z);
			backward_rotated_pt.y = -temp_measure_pt_x * sin(rotation_angle_around_z) + temp_measure_pt_y * cos(rotation_angle_around_z);
			backward_rotated_pt.z = mean_z_1;
#ifdef DBG_L
			cout << "ComputeLevelnessRangeAll"  << "  move_cnt:" << ret.first << endl;
#endif
			levelness.points.push_back(std::tuple<bool, float, cv::Point3f> {true, mean_z_1, backward_rotated_pt});
			backward_rotated_pt.x = single_actual_vertex.x * cos(rotation_angle_around_z) + single_actual_vertex.y * sin(rotation_angle_around_z);
			backward_rotated_pt.y = -single_actual_vertex.x * sin(rotation_angle_around_z) + single_actual_vertex.y * cos(rotation_angle_around_z);
			backward_rotated_pt.z = mean_z_1;
			actual_vertices.push_back(backward_rotated_pt);
		
		}
		else if (ret.first == -1) {
#ifdef DBG_L
			cout << "ComputeLevelnessRangeAll" << "  move_cnt:" << ret.first << endl;
#endif
			//levelness.points.push_back(std::tuple<bool, float, cv::Point3f> {true, 0, cv::Point3f(-1.f, -1.f, -1.f)});
			//actual_vertices.push_back(cv::Point3f(-1.f, -1.f, -1.f));

		}
		else
		{
			//levelness.points.push_back(std::tuple<bool, float, cv::Point3f> {false, -1.f, cv::Point3f(-1.f, -1.f, -1.f)});
			//actual_vertices.push_back(cv::Point3f(-1.f, -1.f, -1.f));
			}
		}
	}
	if (is_valid.size() < 2)
	{
		levelness.is_valid = false;
		levelness.value = -1.f;
		return false;
	}


	levelness.is_valid = true;
	levelness.value = cornerpt_z_max - cornerpt_z_min;
#ifdef DBG_L
	cout << "ComputeLevelnessRangeAll" << " levelness.value:" << levelness.value << endl;
#endif
	return true;
}



bool MeasureLevelnessRange::SaveROIPoints(const std::vector<cv::Point3f>& plane_points, const float* virtual_corner, const int ruler)
{
	if (plane_points.empty())
		return false;
#if 0
	std::string file_name; std::stringstream file_num; file_num << ruler;
	std::string file_type = ".txt";
	file_name = "levelness_xyz";
	file_name += file_num.str();
	file_name += file_type;
	ofstream fout3(file_name, 'w');
	
	if (!fout3.good())
		return false;

	

	for (int i = 0; i < plane_points.size(); i++)
	{
		if ((plane_points[i].x >= virtual_corner[0] - measure_ROI_length * 0.5f)
			&& (plane_points[i].x <= virtual_corner[0] + measure_ROI_length * 0.5f)
			&& (plane_points[i].y >= virtual_corner[1] - measure_ROI_length * 0.5f)
			&& (plane_points[i].y <= virtual_corner[1] + measure_ROI_length * 0.5f))
		{
			fout3 << plane_points[i].x << '\t';
			fout3 << plane_points[i].y << '\t';
			fout3 << plane_points[i].z << std::endl;

			local_rulerplane_pts.Push(plane_points[i]);
		}
	}

	fout3.close();
#endif
	
	return true;
}




bool MeasureLevelnessRange::VoxelizePlane(const std::vector<cv::Point3f>& plane_points, const float* plane_minmax_xy, std::vector<Voxel_LNR>& voxels)
{
	if (plane_points.empty())	return false;

	unsigned int voxel_num_in_x = std::floor((plane_minmax_xy[1] - plane_minmax_xy[0]) / voxel_width) + 1;
	unsigned int voxel_num_in_y = std::floor((plane_minmax_xy[3] - plane_minmax_xy[2]) / voxel_height) + 1;

	voxels.resize(voxel_num_in_x * voxel_num_in_y);
	unsigned int x_idx, y_idx, voxel_idx;

	double sum_z = 0.f;
	for (int i = 0; i < plane_points.size(); i++)
		sum_z += plane_points[i].z;
	double z_mean = sum_z / ((double)(plane_points.size()));

	float delta_z;

	for (int i = 0; i < plane_points.size(); i++)
	{
		//assign to voxel
		x_idx = std::floor((plane_points[i].x - plane_minmax_xy[0]) / voxel_width);
		y_idx = std::floor((plane_points[i].y - plane_minmax_xy[2]) / voxel_height);
		voxel_idx = y_idx * voxel_num_in_x + x_idx;

		if (!voxels[voxel_idx].is_occupied)
		{
			//conduct once for each voxel to be occupied
			voxels[voxel_idx].is_occupied = true;
			voxels[voxel_idx].left_top.x = plane_minmax_xy[0] + x_idx * voxel_width + voxel_width/2;
			voxels[voxel_idx].left_top.y = plane_minmax_xy[2] + y_idx * voxel_height + voxel_height/2;
			voxels[voxel_idx].left_top.z = z_mean;
		}

		delta_z = plane_points[i].z - z_mean;
		voxels[voxel_idx].z_sum += delta_z;
		voxels[voxel_idx].pt_num++;
	}

	//added by simon.jin@unre.com start
	std::vector<Voxel_LNR> voxelsFilter;
	voxelsFilter.resize(voxels.size());
	//voxelsFilter.clear();
	for (int row = 0; row < voxel_num_in_y; row++)
	{
		for (int col = 0; col < voxel_num_in_x; col++)
		{
			int voxel_id = row  * voxel_num_in_x + col;
			if (voxels[voxel_id].is_occupied && voxels[voxel_id].pt_num > min_pt_num_in_voxel)
			{
				voxelsFilter[voxel_id].is_occupied = true;
				voxelsFilter[voxel_id].left_top = voxels[voxel_id].left_top;
				voxelsFilter[voxel_id].pt_num = 0;
				voxelsFilter[voxel_id].z_sum = 0;
				for (int k1 = -1; k1 <= 1; k1++)
				{
					for (int k2 = -1; k2 <= 1; k2++)
					{
						if ((row + k1) >= 0 && (row + k1) < voxel_num_in_y && (col + k2) < voxel_num_in_x && (col + k2) >= 0)
						{
							int voxel_idx = (row + k1) * voxel_num_in_x + (col + k2);

							if (voxels[voxel_idx].is_occupied && voxels[voxel_idx].pt_num > min_pt_num_in_voxel)
							{
								voxelsFilter[voxel_id].z_sum += voxels[voxel_idx].z_sum;
								voxelsFilter[voxel_id].pt_num += voxels[voxel_idx].pt_num;

							}
						}
					}
				}
			}
		}
	}
	voxels = voxelsFilter;
	//added by simon.jin@unre.com end

	return true;
}

bool MeasureLevelnessRange::ComputeLevelnessResult(const std::vector<Voxel_LNR>& voxels, float rotation_angle_around_z, std::vector<std::pair<float, cv::Point3f>>& levelness)
{
	if (voxels.empty())
		return false;

	levelness.resize(voxels.size());

	float z_value = 0.0;
	int valid_voxel_counter = 0;
	for (auto voxel:voxels)
	{
		if (voxel.is_occupied && voxel.pt_num > min_pt_num_in_voxel)
		{
			z_value = voxel.z_sum / (float)(voxel.pt_num);


			//if (abs(z_value) > levelness_threshold)
			{
				////for debug
				//std::cout << defect_value << std::endl;

				levelness[valid_voxel_counter].first = z_value;

				//voxel left top vertex as output
				//backward rotation by -rotation_angle_around_z
				cv::Point3f backward_rotated_pt;
				backward_rotated_pt.x = voxel.left_top.x * cos(rotation_angle_around_z) + voxel.left_top.y * sin(rotation_angle_around_z);
				backward_rotated_pt.y = -voxel.left_top.x * sin(rotation_angle_around_z) + voxel.left_top.y * cos(rotation_angle_around_z);
				backward_rotated_pt.z = voxel.left_top.z;

				levelness[valid_voxel_counter].second = backward_rotated_pt;
				valid_voxel_counter++;
			}
		}
	}
	levelness.resize(valid_voxel_counter);
	//for(int i=0; i < levelness.size)

	return true;
}