#include "OneMeterCheckHelper.h"
#include <opencv2/core/core.hpp> 
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "InOutData.h"

COneMeterHelper::COneMeterHelper()
{
	mWallXZRefectImg =new CWallXZRefectImg();
	mOneMeter=new COneMeter();
}

COneMeterHelper::~COneMeterHelper()
{
	if (mWallXZRefectImg !=nullptr)
	{
		delete mWallXZRefectImg;
		mWallXZRefectImg = nullptr;
	}

	if (mOneMeter != nullptr)
	{
		delete mOneMeter;
		mOneMeter = nullptr;
	}
}


bool COneMeterHelper::CheckAllWallOneMeterAve(
	const std::vector<int>& plane_wall_idx,
	const std::vector<cv::Point3f> plane_normals,
	const std::vector<cv::Point3f>& plane_centers,
	const std::vector<std::vector<cv::Point3f>>& plane_xyz,
	const std::vector<std::vector<unsigned char>>& plane_reflect,
	float& mean_one_meter_z)
{
	return CheckAllWallOneMeterAveHelper(plane_wall_idx,plane_normals,plane_centers,plane_xyz,plane_reflect,mean_one_meter_z);
}

bool COneMeterHelper::CheckAllWallOneMeterAveHelper(
	const std::vector<int>& plane_wall_idx,
	const std::vector<cv::Point3f> plane_normals,
	const std::vector<cv::Point3f>& plane_centers,
	const std::vector<std::vector<cv::Point3f>>& plane_xyz,
	const std::vector<std::vector<unsigned char>>& plane_reflect,
	float& mean_one_meter_z)
{
	int wall_one_meter_count = 0;
	float wall_one_meter_mean = 0;
	for (int ori_id = 0; ori_id<plane_wall_idx.size(); ori_id++)
	{
		int wall_id = plane_wall_idx[ori_id];
		//std::cout << "ori_id:" << ori_id << " wall_id:" << wall_id << std::endl;
		//IOData::SavePLYPoints3f("origin_meshes\\wall_ori_" + std::to_string(wall_id) + "_.ply", plane_xyz[wall_id], true);

		cv::Point3f planeNormal = plane_normals[wall_id];
		cv::Point3f planeCenter = plane_centers[wall_id];
		std::vector<cv::Point3f> ori_plane = plane_xyz[wall_id];

		std::vector<int> planeReflectValues;
		for (int i = 0; i<plane_reflect[wall_id].size(); i++)
		{
			planeReflectValues.push_back(int(plane_reflect[wall_id][i]));
		}

		float  one_meter_z = 0;
		bool isOneMeterZ = CheckWallIdOneMeter(wall_id,
			planeNormal,
			planeCenter,
			ori_plane,
			planeReflectValues,
			one_meter_z);
		if (isOneMeterZ)
		{
			wall_one_meter_count++;
			wall_one_meter_mean += one_meter_z;
		}
	}

	if (wall_one_meter_count != 0)
	{
		wall_one_meter_mean /= wall_one_meter_count;
		mean_one_meter_z = wall_one_meter_mean;
		//std::cout << "===in COneMeterHelper::CheckAllWallOneMeterAve=have check one_meter wall_one_meter_mean_z_value:" << wall_one_meter_mean << std::endl;
		return true;
	}
	std::cout << "===no check one_meter_z value!===" << std::endl;
	return false;
}

bool COneMeterHelper::CheckWallIdOneMeter(const int& wall_id,
	const cv::Point3f& plane_normal,
	const cv::Point3f& plane_center,
	const std::vector<cv::Point3f>& ori_plane,
	const std::vector<int>& planeReflectValues,
	float& one_meter_z)
{
	//rotate to XZ plane
	std::vector<cv::Point3f> xzPlane;
	bool isGet = mWallXZRefectImg->GetXzPlane(ori_plane, plane_normal, plane_center, xzPlane);
	if (!isGet)
	{
		//std::cout << "in COneMeterHelper::CheckWallIdOneMeter get xzPlane false !" << std::endl;
		return false;
	}
	//IOData::SavePLYPoints3f("origin_meshes\\wall_xz_" + std::to_string(wall_id) + "_.ply", xzPlane, true);

	cv::Mat reflectRasterImgTen;
	bool isImg10 = mWallXZRefectImg->GetReflectImgPixelTen(xzPlane, planeReflectValues, reflectRasterImgTen);
	if (!isImg10)
	{
		//std::cout << "in COneMeterHelper::CheckWallIdOneMeter get img10  false !" << std::endl;
		return false;
	}
	//cv::imwrite("origin_meshes\\refect_10_img_" + std::to_string(wall_id) + "_.jpg", reflectRasterImgTen);

	int pixelHeighTen = 10;
	std::vector<cv::Vec3f> vec_circle_big_ten;
	bool isCheckCircle = mOneMeter->StepOneCheckImgTenPixleCircles(wall_id, pixelHeighTen, reflectRasterImgTen, vec_circle_big_ten);
	if (!isCheckCircle)
	{
		std::cout << "in COneMeterHelper::CheckWallIdOneMeter is not check circle false !" << std::endl;
		return false;
	}

	cv::Mat reflectRasterImgOne;
	bool isImg1 = mWallXZRefectImg->GetReflectImgPixelOne(xzPlane, planeReflectValues, reflectRasterImgOne);
	if (!isImg1)
	{
		//std::cout << "in COneMeterHelper::CheckWallIdOneMeter get imgOne  false !" << std::endl;
		return false;
	}

	cv::Point3f pt_min;
	cv::Point3f pt_max;
	bool isFindMinMax = mWallXZRefectImg->FindXzPlaneMinMax(xzPlane, pt_min, pt_max);
	if (!isFindMinMax)
	{
		//std::cout << "in COneMeterHelper::CheckWallIdOneMeter FindMinMax  false !" << std::endl;
		return false;
	}
	bool isMeterZ = mOneMeter->StepTwoRefineMoveCircleFindMostFit(wall_id,
		reflectRasterImgOne,
		vec_circle_big_ten,
		pt_min,
		pt_max,
		one_meter_z);
	if (!isMeterZ)
	{
		std::cout << "in COneMeterHelper::CheckWallIdOneMeter StepTwoRefine MeterZ false !" << std::endl;
		return false;
	}
	return true;
}
