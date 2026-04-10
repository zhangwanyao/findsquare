#include "PlaneXyRefectImg.h"
#include <iostream>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "../Common/CheckPreOp.h"
#include "../Common/FindTool.h"
#include "../Common/LookatOperation.h"

#define DEBUG_AXIS_LINE false

CPlaneXyImg::CPlaneXyImg()
{
}

CPlaneXyImg::~CPlaneXyImg()
{
}

bool CPlaneXyImg::PlaneXYRefectImgComm(
	const std::vector<cv::Point3f>& xyCloudPoints,
	const std::vector<int> planeReflectValues,
	const int& pixelHeight,
	const int& pixelWidth,
	cv::Mat& reflectImg)
{
	if (xyCloudPoints.size()==0 )
	{
		std::cout << "CPlaneXyImg::PlaneXYRefectImgComm xyPlane points is zero!" << std::endl;
		return false;
	}

	int numPoints = xyCloudPoints.size();

	Eigen::ArrayXf coordX;
	Eigen::ArrayXf coordY;

	std::vector<float> cloudPoints_X(numPoints);
	std::vector<float> cloudPoints_Y(numPoints);
	std::vector<float> cloudPoints_Z(numPoints);
	for (auto iter = xyCloudPoints.begin(); iter < xyCloudPoints.end(); iter++) {
		int id = iter - xyCloudPoints.begin();
		cloudPoints_X[id] = xyCloudPoints[id].x;
		cloudPoints_Y[id] = xyCloudPoints[id].y;
		cloudPoints_Z[id] = xyCloudPoints[id].z;
	}

	Eigen::Map<Eigen::ArrayXf> cloudPoints_mapX(cloudPoints_X.data(), numPoints);
	Eigen::Map<Eigen::ArrayXf> cloudPoints_mapY(cloudPoints_Y.data(), numPoints);
	Eigen::Map<Eigen::ArrayXf> cloudPoints_mapZ(cloudPoints_Z.data(), numPoints);

	float minCloudPoints_X = cloudPoints_mapX.minCoeff();
	float minCloudPoints_Y = cloudPoints_mapY.minCoeff();
	float minCloudPoints_Z = cloudPoints_mapZ.minCoeff();

	float maxCloudPoints_X = cloudPoints_mapX.maxCoeff();
	float maxCloudPoints_Y = cloudPoints_mapY.maxCoeff();
	float maxCloudPoints_Z = cloudPoints_mapZ.maxCoeff();

	//minCloudPoints_X = min(minCloudPoints_X, minGlobalX);
	//minCloudPoints_Z = min(minCloudPoints_Z, minGlobalZ);
	//maxCloudPoints_X = max(maxCloudPoints_X, maxGlobalX);
	//maxCloudPoints_Z = max(maxCloudPoints_Z, maxGlobalZ);

	float meanCloudPoints_Z = cloudPoints_mapZ.mean();

	float deltaCloudPointsZ = maxCloudPoints_Z - minCloudPoints_Z;

	float width = maxCloudPoints_X - minCloudPoints_X;
	float height = maxCloudPoints_Y - minCloudPoints_Y;

	coordX = (cloudPoints_mapX - minCloudPoints_X);
	coordY = (cloudPoints_mapY - minCloudPoints_Y);

	int rasterWidth = int(width / pixelWidth) + 1;
	int rasterHeight = int(height / pixelHeight) + 1;

	std::vector<float> floatPlaneReflectValues(planeReflectValues.begin(), planeReflectValues.end());
	Eigen::Map<Eigen::ArrayXf> ReflectValues(floatPlaneReflectValues.data(), numPoints);
	//std::cout<<"ReflectValues:" << ReflectValues << std::endl;
	Eigen::ArrayXi reflectPixelValue = (255 * (ReflectValues - ReflectValues.minCoeff()) / (ReflectValues.maxCoeff() - ReflectValues.minCoeff())).cast<int>();


	Eigen::MatrixXi reflectValueMat(Eigen::MatrixXi::Zero(rasterHeight, rasterWidth));
	Eigen::MatrixXi countMat = Eigen::MatrixXi::Zero(rasterHeight, rasterWidth);
	for (int id = 0; id < numPoints; id++)
	{
		//std::cout << pixelValue(id) << std::endl;
		int axisY = int(coordY(id) / pixelHeight);
		int axisX = int(coordX(id) / pixelWidth);
		countMat(axisY, axisX) += 1;
		reflectValueMat(axisY, axisX) += reflectPixelValue(id);
	}

	cv::Mat reflectRasterImg_tmp = cv::Mat::zeros(rasterHeight, rasterWidth, CV_8UC1);
	for (int i = 0; i < reflectRasterImg_tmp.rows; ++i) {
		for (int j = 0; j < reflectRasterImg_tmp.cols; ++j) {
			if (countMat(i, j) != 0) {
				reflectRasterImg_tmp.at<uchar>(i, j) = int(reflectValueMat(i, j) / countMat(i, j));
			}
		}
	}

	reflectImg = reflectRasterImg_tmp.clone();

	////flip
	//cv::flip(reflectRasterImg_tmp, reflectRasterImg_tmp, 0);
	//reflectImg = reflectRasterImg_tmp.clone();
	//reflectRasterImg_tmp.release();
	return true;
}


bool CPlaneXyImg::CheckCircle(const int& wall_id, const cv::Mat& reflectImgTen)
{
	cv::Mat reflectImgTen_stretch;
	bool isStretch = CheckPreOp::ContrastStretchBetweenMinMax(reflectImgTen, reflectImgTen_stretch);
	if (!isStretch)
	{
		return false;
	}

	std::cout << "COneMeter::CheckReflectImg10PixleCirclesWithStretch img_ten stretch false!" << std::endl;
	cv::imwrite("rst_axis\\groud10_stretch_" + std::to_string(wall_id) + "_.jpg", reflectImgTen_stretch);

	cv::Mat refect_img_gradxy1 = CheckPreOp::GetReflectGradXY(reflectImgTen);
	cv::imwrite("rst_axis\\groud10_stretch_gradxy1_" + std::to_string(wall_id) + "_.jpg", refect_img_gradxy1);

	cv::Mat refect_img_gradxy = CheckPreOp::GetReflectGradXY(reflectImgTen_stretch);
	cv::imwrite("rst_axis\\groud10_stretch_gradxy2_" + std::to_string(wall_id) + "_.jpg", refect_img_gradxy);


	std::vector<cv::Vec3f> cir;
	cv::HoughCircles(refect_img_gradxy, cir, CV_HOUGH_GRADIENT, 1, 20, 100, 17, 10, 12);//10_size ok

	//for (size_t i = 0; i < cir.size(); i++)
	//{
	//	cv::Point center(cvRound(cir[i][0]), cvRound(cir[i][1]));
	//	int radius = cvRound(cir[i][2]);
	//	//std::cout << " ===" << i << " radius:" << radius << " center.y:" << center.y << std::endl;
	//}

	cv::Mat print_ldmk = CheckPreOp::PrintCirclesForAxis(refect_img_gradxy, cir);
	cv::imwrite("rst_axis\\print_circle_10_" + std::to_string(wall_id) + "_.jpg", print_ldmk);

	return true;
}

bool CPlaneXyImg::IsImgTenExistCircle(const int& wall_id, const cv::Mat& reflectImgTen, std::vector<cv::Vec3f>& cir_info)
{
	cv::Mat reflectImgTen_stretch;
	bool isStretch = CheckPreOp::ContrastStretchBetweenMinMax(reflectImgTen, reflectImgTen_stretch);
	if (!isStretch)
	{
		std::cout << "IsImgTenExistCircle img_ten stretch false!" << std::endl;
		return false;
	}


	cv::imwrite("rst_axis\\groud10_stretch_" + std::to_string(wall_id) + "_.jpg", reflectImgTen_stretch);

	cv::Mat refect_img_gradxy1 = CheckPreOp::GetReflectGradXY(reflectImgTen);
	cv::imwrite("rst_axis\\groud10_stretch_gradxy1_" + std::to_string(wall_id) + "_.jpg", refect_img_gradxy1);

	cv::Mat refect_img_gradxy = CheckPreOp::GetReflectGradXY(reflectImgTen_stretch);
	cv::imwrite("rst_axis\\groud10_stretch_gradxy2_" + std::to_string(wall_id) + "_.jpg", refect_img_gradxy);


	std::vector<cv::Vec3f> cir;
	cv::HoughCircles(refect_img_gradxy, cir, CV_HOUGH_GRADIENT, 1, 20, 100, 17, 10, 12);//10_size ok

	//for (size_t i = 0; i < cir.size(); i++)
	//{
	//	cv::Point center(cvRound(cir[i][0]), cvRound(cir[i][1]));
	//	int radius = cvRound(cir[i][2]);
	//	//std::cout << " ===" << i << " radius:" << radius << " center.y:" << center.y << std::endl;
	//}
	if (cir.size()<2)
	{
		std::cout << "IsImgTenExistCircle circle num small two,false!" << std::endl;
		return false;
	}
	cv::Mat print_ldmk = CheckPreOp::PrintCirclesForAxis(refect_img_gradxy, cir);
	cv::imwrite("rst_axis\\print_circle_10_" + std::to_string(wall_id) + "_.jpg", print_ldmk);
	cir_info = cir;
	return true;
}

bool CPlaneXyImg::GetPlaneXYRefectImgTen(const std::vector<cv::Point3f>& xyCloudPoints, const std::vector<int> planeReflectValues, cv::Mat& imgTen)
{
	const int pixelHeight = 10;
	const int pixelWidth = 10;
	return PlaneXYRefectImgComm(xyCloudPoints, planeReflectValues, pixelHeight, pixelWidth, imgTen);
}

bool CPlaneXyImg::GetPlaneXYRefectImgOne(const std::vector<cv::Point3f>& xyCloudPoints, const std::vector<int> planeReflectValues, cv::Mat& imgOne)
{
	const int pixelHeight = 1;
	const int pixelWidth = 1;
	return PlaneXYRefectImgComm(xyCloudPoints, planeReflectValues, pixelHeight, pixelWidth, imgOne);
}


void CPlaneXyImg::GetRefineMoveDistance(float& radius, float& move_distance)
{
	//default set fixed circle radius is 100mm
	radius = 100;//1== pixelSize
	move_distance = 24;//1== pixelSize
	//if (ONE_METER_CIRCLE_RADIUS_IS_FIVE)
	//{
	//	//fixed circle radius is 50mm
	//	radius = 50;//1== pixelSize
	//	move_distance = 12;//1== pixelSize
	//}
}


//void Points2dTo3dHelper(const std::vector<cv::Point>& vc_pt2d,
//	const float&min_x3d, const int& width_2d, const float&width_3d,
//	const float&min_y3d, const int& height_2d, const float&height_3d,
//	const float& meanZ, std::vector<cv::Point3f>& vc_pt3d)
//{
//	vc_pt3d.resize(0);
//	for (int i = 0; i<vc_pt2d.size(); i++)
//	{
//		cv::Point3f pt;
//		pt.x = vc_pt2d[i].x*(width_3d) / width_2d + min_x3d;
//		pt.y = vc_pt2d[i].y*(height_3d) / height_2d + min_y3d;;
//		pt.z = meanZ;
//		vc_pt3d.push_back(pt);
//	}
//}
float CPlaneXyImg::AbsMinPointTowDistance(const cv::Point& pt1, const cv::Point& pt2)
{
	float dis_x = std::abs(pt1.x-pt2.x);
	float dis_y = std::abs(pt1.y - pt2.y);
	float min_dis = std::min(dis_x,dis_y);
	return min_dis;
}

bool CPlaneXyImg::SelectFitTwoCenter(const std::vector<cv::Point>&circle_center2d, std::vector<cv::Point>&circle_center2d_filter)
{
	int circle_num = circle_center2d.size();
	if (circle_num < 2)
	{
		return false;
	}

	if (circle_num==2)
	{
		circle_center2d_filter=circle_center2d;
		return true;
	}

	int min_i = -1;
	int min_j = -1;
	float min_value= std::numeric_limits<float>::infinity();
	for (int i=0;i<circle_center2d.size()-1;i++)
	{
		cv::Point pt_i = circle_center2d[i];
		for (int j=i+1;j<circle_center2d.size();j++)
		{
			cv::Point pt_j = circle_center2d[j];
			float dist = AbsMinPointTowDistance(pt_i, pt_j);
			if (dist<min_value)
			{
				min_value = dist;
				min_i = i;
				min_j = j;
			}
		}
	}
	 

	if (min_i==-1 || min_j==-1)
	{
		return false;
	}
	circle_center2d_filter.resize(2);
	circle_center2d_filter[0] = circle_center2d[min_i];
	circle_center2d_filter[1] = circle_center2d[min_j];
	return true;

}

bool CPlaneXyImg::CheckAxisLineHelper(
	const int& wall_id,
	const std::vector<cv::Point3f>& xyCloudPoints, 
	const std::vector<int> planeReflectValues,
	std::vector<cv::Point3f>& vec_circle_center3d)
{
	cv::Mat reflectImgTen;
	bool isImgTen = GetPlaneXYRefectImgTen(xyCloudPoints, planeReflectValues, reflectImgTen);
	if (!isImgTen)
	{
		std::cout << "CheckAxisLine isImgTen false!" << std::endl;
		return false;
	}

	std::vector<cv::Vec3f> cir_info;
	bool isExistCircle = IsImgTenExistCircle(wall_id, reflectImgTen, cir_info);
	if (!isExistCircle)
	{
		std::cout << "CheckAxisLine isExistCircle false!" << std::endl;
		return false;
	}

	cv::Mat reflectImgOne;
	bool isImgOne = GetPlaneXYRefectImgOne(xyCloudPoints, planeReflectValues, reflectImgOne);
	if (!isImgOne)
	{
		std::cout << "CheckAxisLine isImgOne false!" << std::endl;
		return false;
	}

	std::vector<cv::Vec3f> vec_circle_fit;
	CheckPreOp::ChaneTenCircleToFitPixelOneCircle(cir_info, vec_circle_fit);


	float radius;
	float move_distance;
	GetRefineMoveDistance(radius, move_distance);
	std::vector<cv::Vec3f> vec_circle_fit_after;
	bool isFitOk = CheckPreOp::FindFitCircleCentersHelper(reflectImgOne, vec_circle_fit, radius, move_distance, vec_circle_fit_after);
	if (!isFitOk)
	{
		std::cout << "CheckAxisLine::FindFitCircleCentersHelper false!" << std::endl;
		return false;
	}

	if (DEBUG_AXIS_LINE)
	{
		cv::Mat reflectImgOneAve = CheckPreOp::ReflectRasterImgAveInterpolation(reflectImgOne);
		cv::imwrite("rst_axis\\refect_ave_" + std::to_string(wall_id) + "_.jpg", reflectImgOneAve);

		cv::Mat reflectImgOneAveLdmk= CheckPreOp::PrintCirclesForAxis(reflectImgOneAve, vec_circle_fit_after);
		cv::imwrite("rst_axis\\refect_ave_ldmk_" + std::to_string(wall_id) + "_.jpg", reflectImgOneAveLdmk);
	}

	cv::Point3f minPt, maxPt;
	FindTool::FindMinMaxHelper(xyCloudPoints, minPt, maxPt);
	float min_3dx = minPt.x;
	float min_3dy = minPt.y;
	float width3d = maxPt.x - minPt.x;
	float height3d = maxPt.y - minPt.y;

	float meanZ = minPt.z+(maxPt.z - minPt.z) / 2;

	int width_2d = reflectImgOne.cols;
	int height_2d = reflectImgOne.rows;
	std::vector<cv::Point>circle_center2d;
	CheckPreOp::ChangeVecCirce2PointCenter(vec_circle_fit_after, circle_center2d);

	std::vector<cv::Point>circle_center2d_filter;
	bool isSelect = SelectFitTwoCenter(circle_center2d, circle_center2d_filter);
	if (!isSelect)
	{
		std::cout << "CheckAxisLine SelectFitTwoCenter false!" << std::endl;
		return false;
	}

	std::vector<cv::Point3f> vc_pt3d;
	for (int i=0;i<circle_center2d_filter.size();i++)
	{
		cv::Point3f pt;
		pt.x = circle_center2d_filter[i].x*(width3d) / width_2d + min_3dx;
		pt.y = circle_center2d_filter[i].y*(height3d) / height_2d + min_3dy;
		pt.z = meanZ;
		vc_pt3d.push_back(pt);
	}

	vec_circle_center3d = vc_pt3d;

	return true;

}


bool CPlaneXyImg::CheckOneGroundAxisLine(
	const int& ground_id,
	const cv::Point3f& inner_normal,
	const cv::Point3f& ground_center,
	const std::vector<cv::Point3f>& ground_plane,
	const std::vector<int>& planeReflectValues,
	std::vector<cv::Point3f>& circle_center_center3d)
{
	if (ground_plane.size()==0|| planeReflectValues.size()==0)
	{
		std::cout << "CheckOneGroundAxisLine input cloud size or refect size is zero!" << std::endl;
		return false;
	}
	cv::Mat lookAt;
	CLookAt::GetLookAtMat(inner_normal, ground_center, 100, lookAt);

	std::vector<cv::Point3f> ground_lookat_xyplane;
	CLookAt::RotationPoints(ground_plane, ground_lookat_xyplane, lookAt);

	std::vector<cv::Point3f> circle_xyplane_center3d;
	bool isCircleCenterOk = CheckAxisLineHelper(ground_id, ground_lookat_xyplane,planeReflectValues,circle_xyplane_center3d);
	if (!isCircleCenterOk)
	{
		std::cout << "CheckOneGroundAxisLine isCircleCenterOk is false!" << std::endl;
		return false;
	}

	cv::Mat lookAt_inv;
	cv::invert(lookAt, lookAt_inv, CV_SVD);

	std::vector<cv::Point3f> circle_ori_center3d;
	CLookAt::RotationPoints(circle_xyplane_center3d, circle_ori_center3d, lookAt_inv);

	circle_center_center3d = circle_ori_center3d;
	return true;

}