#include "AxisLineCheck.h"
#include "../Common/CheckPreOp.h"
#include "../Common/LookatOperation.h"
#include "../Common/MathOperation.h"
#include "../IO/include/InOutData.h"


#define DEBUG_GROUND_AXIS_LINE false

CAxisLine::CAxisLine()
{

	mXYRefectImg = new CPlaneXyImg();
}

CAxisLine::~CAxisLine()
{
	if (mXYRefectImg != nullptr)
	{
		delete mXYRefectImg;
		mXYRefectImg = nullptr;
	}
}

bool CAxisLine::CheckGroundAixsLine(
	const std::vector<int>& plane_ground_idx,
	const std::vector<cv::Point3f> plane_normals,
	const std::vector<cv::Point3f>& plane_centers,
	const std::vector<std::vector<cv::Point3f>>& plane_xyz,
	const std::vector<std::vector<unsigned char>>& plane_reflect,
	std::vector<std::pair<cv::Point3f, cv::Point3f>>& ground_axis_line)
{
	return CheckGroundAixsLineHelper(plane_ground_idx, plane_normals, plane_centers, plane_xyz, plane_reflect, ground_axis_line);
}



bool CAxisLine::CheckGroundAixsLineHelper(
	const std::vector<int>& plane_ground_idx,
	const std::vector<cv::Point3f> plane_normals,
	const std::vector<cv::Point3f>& plane_centers,
	const std::vector<std::vector<cv::Point3f>>& plane_xyz,
	const std::vector<std::vector<unsigned char>>& plane_reflect,
	std::vector<std::pair<cv::Point3f, cv::Point3f>>& ground_axis_line)
{
	bool isExistAxis = false;
	ground_axis_line.resize(plane_ground_idx.size());
	for (int i = 0; i<plane_ground_idx.size(); i++)
	{
		int ground_id = plane_ground_idx[i];
		//std::cout << "i:" << i << " ground_id:" << ground_id << std::endl;

		//std::cout << "ori_id:" << ori_id << " wall_id:" << wall_id << std::endl;
		//IOData::SavePLYPoints3f("origin_meshes\\wall_ori_" + std::to_string(wall_id) + "_.ply", plane_xyz[wall_id], true);

		cv::Point3f planeNormal = plane_normals[ground_id];
		cv::Point3f planeCenter = plane_centers[ground_id];
		std::vector<cv::Point3f> ground_plane = plane_xyz[ground_id];

		cv::Point3f normal_inner = planeNormal;
		if (MathOperation::ComputeVectorDotProduct(planeNormal, planeCenter) > 0.f)
		{
			//point to center
			normal_inner *= -1;
		}

		std::vector<int> planeReflectValues;
		for (int i = 0; i<plane_reflect[ground_id].size(); i++)
		{
			planeReflectValues.push_back(int(plane_reflect[ground_id][i]));
		}

		std::vector<cv::Point3f> circle_center_center3d;
		bool isOk = mXYRefectImg->CheckOneGroundAxisLine(ground_id,normal_inner,planeCenter,ground_plane,planeReflectValues,circle_center_center3d);
		if (isOk)
		{
			//std::cout << "====test===ok===" << std::endl;
			if (DEBUG_GROUND_AXIS_LINE)
			{
				IOData::SavePLYPoints3f("rst_axis\\ground_ori_" + std::to_string(ground_id) + "_.ply", ground_plane, true);
				IOData::SavePLYPoints3f("rst_axis\\center_ori_" + std::to_string(ground_id) + "_.ply", circle_center_center3d, false);
			}
			ground_axis_line[i] = std::make_pair(circle_center_center3d[0], circle_center_center3d[1]);
			isExistAxis = true;
		}
	}

	return isExistAxis;
}