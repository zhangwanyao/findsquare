#include "RoomMeasurement.h"
#include "PointReconstruction.h"
#include "Common/MathOperation.h"
#include "Measurement/MeasureDefect.h"
#include "IO/include/InOutData.h"
#include "Measurement/OneMeterCheckHelper.h"
#include "Common/LookatOperation.h"
#include  "Measurement/PlaneXyRefectImg.h"
#include "../Common/CheckPreOp.h"
#include  "Measurement/AxisLineCheck.h"
#include "../DoorWindowDimension/include/util_line_tool.hpp"
#include "util_math.hpp"
#include "PointReconstructionGetConfig.h"
#include "jansson.h"
#include <opencv2/opencv.hpp>

//#define DBG_HOLE_ALGO
#define HOLES_DIRECTORY  (std::string("Holes\\"))

//#define Debug_L
//#define Debug_R
//#define Debug_RW
//#define Debug_M
//#define Debug_RC
//#define Debug_COV
//#define Debug_MH
//#define Debug_MHS
//#define Debug_Hols_P
//#define Debug_Hols_PS
//#define Debug_05

#if 1//def Debug_R
extern std::string filePath;
extern std::ofstream resultA;
#endif
RoomMeasurement::RoomMeasurement(PointReconstruction *pRecon)
	: mHasOneMeterLine(false),
	mHasGroundAxisLine(false),
	mPRecon(pRecon)
{
	if (pRecon) {
		mPRecon = pRecon;
	}
}

RoomMeasurement::~RoomMeasurement()
{
}

#ifdef Debug_MHS
bool SavePoint3fDataT(const string path_name, const ModuleStruct::Point3Array& input_data) {

	if (input_data.empty()) {
		cout << "IOData::SavePoint3fData: empty data" << endl;
		return false;
	}

	if (path_name.empty()) {
		cout << "IOData::SavePoint3fData: empty file path" << endl;
		return false;
	}

	ofstream fout(path_name);
	if (fout.fail()) {
		log_fatal("cannot open %s", path_name.c_str());
		return false;
	}

	for (int i = 0; i < input_data.size(); i++) {
		fout << input_data[i].x<< '\t' << input_data[i].y << '\t' << input_data[i].z << '\t' <<  100;
		if (i < input_data.size() - 1) { fout << endl; }
	}
	fout.close();

	return true;
}
#endif

void RoomMeasurement::DetectGroundAxisLine(void)
{

	std::vector<cv::Point3f> plane_normals = mPRecon->GetPlanes().plane_normals;
	std::vector<cv::Point3f> plane_centers = mPRecon->GetPlanes().plane_center;
	std::vector<std::vector<cv::Point3f>> plane_xyz = mPRecon->GetPlanes().plane_xyz;
	std::vector<std::vector<unsigned char>> plane_reflect = mPRecon->GetPlanes().plane_reflect;
	std::vector<int> plane_ground_idx = mPRecon->GetPlanes().plane_ground_idx;

	CAxisLine axisLine;
	std::vector<std::pair<cv::Point3f, cv::Point3f>> ground_axis_line;
	bool isSuc=axisLine.CheckGroundAixsLine( plane_ground_idx, plane_normals, plane_centers,plane_xyz,plane_reflect, ground_axis_line);
	if (isSuc)
	{
		mHasGroundAxisLine = true;
		m_ground_axis_line = ground_axis_line;
		//for (int i=0;i<plane_ground_idx.size();i++)
		//{
		//	int ground_id = plane_ground_idx[i];
		//	std::cout << "i:" << i << " ground_id:" << ground_id << " first:" << m_ground_axis_line[i].first << " second:" << m_ground_axis_line[i].second << std::endl;
		//}
		//std::cout << "==================test================" << std::endl;
	}
}

void RoomMeasurement::DetectOneMeterLine(void)
{
	std::vector<int> plane_wall_idx = mPRecon->GetPlanes().plane_wall_idx;
	std::vector<cv::Point3f> plane_normals = mPRecon->GetPlanes().plane_normals;
	std::vector<cv::Point3f> plane_centers = mPRecon->GetPlanes().plane_center;
	//std::vector<StructuredPlane> vec_StrctPlane = mPRecon->GetStructuredPlanes();
	std::vector<std::vector<cv::Point3f>> plane_xyz = mPRecon->GetPlanes().plane_xyz;
	std::vector<std::vector<unsigned char>> plane_reflect = mPRecon->GetPlanes().plane_reflect;

	////just for test ground Axis line
   // DetectGroundAxisLine();

	COneMeterHelper oneMeterHelper;
	float mean_one_meter_z = 0;
	bool isMeanOneMeterZ = oneMeterHelper.CheckAllWallOneMeterAve(
		plane_wall_idx,
		plane_normals,
		plane_centers,
		plane_xyz,
		plane_reflect,
		mean_one_meter_z);
	if (isMeanOneMeterZ)
	{
		std::cout << "==check one meter success===mean_one_meter_z=====:" << mean_one_meter_z << std::endl;
		mOneMerterLineZ = mean_one_meter_z;
		mHasOneMeterLine = true;
	}
}

bool LessSort(std::pair<float, cv::Point3f> a, std::pair<float, cv::Point3f> b) { return (a.first > b.first); };

float DisOf2Points2f(cv::Point2f p0, cv::Point2f p1) {
	return  sqrt(std::pow(p0.x - p1.x, 2.0) + std::pow(p0.y - p1.y, 2.0));
}
bool RoomMeasurement::MeasureStraightness(void)
{
	std::vector<cv::Point3f> ground_corner_points;
	std::vector<std::pair<int, int>> L_shape_plane_idx = mPRecon->GetPlanes().L_shape_plane_idx;
	std::vector<cv::Point3f> plane_normals = mPRecon->GetPlanes().plane_normals;
	std::vector<cv::Point3f> plane_center = mPRecon->GetPlanes().plane_center;
	std::vector<int> plane_ground_idx = mPRecon->GetPlanes().plane_ground_idx;
	std::vector<std::vector<cv::Point3f>> plane_xyz = mPRecon->GetPlanes().plane_xyz;
	std::vector<std::pair<std::pair<int, int>, float>> straightness;
	for (int i = 0; i < L_shape_plane_idx.size(); i++)
	{
		//cout << "zhujunq filtered_L_shape_idx = " << i << "  id=  " << scene_filter_output.angle_squareness_Lshape_idx[i].first << endl;
		//cout << "zhujunq filtered_L_shape_idx = " << i << "  id=  " << scene_filter_output.angle_squareness_Lshape_idx[i].second << endl;

		float plane_normal1[3];
		plane_normal1[0] = plane_normals[L_shape_plane_idx[i].first].x;
		plane_normal1[1] = plane_normals[L_shape_plane_idx[i].first].y;
		plane_normal1[2] = plane_normals[L_shape_plane_idx[i].first].z;

		float plane_center_normal1[3];
		plane_center_normal1[0] = plane_center[L_shape_plane_idx[i].first].x;
		plane_center_normal1[1] = plane_center[L_shape_plane_idx[i].first].y;
		plane_center_normal1[2] = plane_center[L_shape_plane_idx[i].first].z;

		float plane_normal2[3];
		plane_normal2[0] = plane_normals[L_shape_plane_idx[i].second].x;
		plane_normal2[1] = plane_normals[L_shape_plane_idx[i].second].y;
		plane_normal2[2] = plane_normals[L_shape_plane_idx[i].second].z;

		float plane_center_normal2[3];
		plane_center_normal2[0] = plane_center[L_shape_plane_idx[i].second].x;
		plane_center_normal2[1] = plane_center[L_shape_plane_idx[i].second].y;
		plane_center_normal2[2] = plane_center[L_shape_plane_idx[i].second].z;


		float x, y, z;

		float line_normal[3];
		MeasureBase::CrossProduct(plane_normal1, plane_normal2, line_normal);

		//cout << "zhujunqing show cross line normal = " << line_normal[0] << " " << line_normal[1] << " " << line_normal[2] << endl;

		0 == plane_normal1[0] * (x - plane_center_normal1[0]) + plane_normal1[1] * (y - plane_center_normal1[1]) + plane_normal1[2] * (z - plane_center_normal1[2]);
		0 == plane_normal2[0] * (x - plane_center_normal2[0]) + plane_normal2[1] * (y - plane_center_normal2[1]) + plane_normal2[2] * (z - plane_center_normal2[2]);
		z = 0.0f;
		x = (plane_normal2[1] * (plane_normal1[0] * plane_center_normal1[0] + plane_normal1[1] * plane_center_normal1[1] + plane_normal1[2] * plane_center_normal1[2]) - plane_normal1[1] * (plane_normal2[0] * plane_center_normal2[0] + plane_normal2[1] * plane_center_normal2[1] + plane_normal2[2] * plane_center_normal2[2])) / (plane_normal2[1] * plane_normal1[0] - plane_normal1[1] * plane_normal2[0]);
		y = (plane_normal2[0] * (plane_normal1[0] * plane_center_normal1[0] + plane_normal1[1] * plane_center_normal1[1] + plane_normal1[2] * plane_center_normal1[2]) - plane_normal1[0] * (plane_normal2[0] * plane_center_normal2[0] + plane_normal2[1] * plane_center_normal2[1] + plane_normal2[2] * plane_center_normal2[2])) / (plane_normal2[0] * plane_normal1[1] - plane_normal1[0] * plane_normal2[1]);
		//cout << "zhujunqing show cross line point = " << x << " " << y << " " << z << endl;

		cv::Point3f groundCenterPoint, groundNormalPoint, lineNormalPoint, linePoint, resultPoint;
		lineNormalPoint.x = line_normal[0];
		lineNormalPoint.y = line_normal[1];
		lineNormalPoint.z = line_normal[2];
		linePoint.x = x;
		linePoint.y = y;
		linePoint.z = z;
		groundCenterPoint = plane_center[plane_ground_idx[0]];
		groundNormalPoint = plane_normals[plane_ground_idx[0]];


		cv::Point3f twoPlaneCenterPoint;
		twoPlaneCenterPoint.x = (plane_center_normal1[0] + plane_center_normal2[0]) / 2;
		twoPlaneCenterPoint.y = (plane_center_normal1[1] + plane_center_normal2[1]) / 2;
		twoPlaneCenterPoint.z = (plane_center_normal1[2] + plane_center_normal2[2]) / 2;

		if (MathOperation::CalPlaneLineIntersectPoint_LineNoraml(lineNormalPoint, linePoint, groundNormalPoint, groundCenterPoint, resultPoint)) {

		}
		//cout << "show resultPoint == " << resultPoint << endl;

		cv::Point3f topPoint;
		float tmpZ_first = -10000;
		float tmpZ_second = -10000;
		cv::Mat twoPlanePoint;
		//cout << "show twoPlanePoint first size = " << plane_xyz[L_shape_plane_idx[i].first].size() << endl;
		for (int j = 0; j < plane_xyz[L_shape_plane_idx[i].first].size(); j++)
		{

			cv::Mat point_tmp = cv::Mat(1, 3, CV_32F);
			point_tmp.at<float>(0, 0) = plane_xyz[L_shape_plane_idx[i].first][j].x;
			point_tmp.at<float>(0, 1) = plane_xyz[L_shape_plane_idx[i].first][j].y;
			point_tmp.at<float>(0, 2) = plane_xyz[L_shape_plane_idx[i].first][j].z;
			twoPlanePoint.push_back(point_tmp);
			if (plane_xyz[L_shape_plane_idx[i].first][j].z > tmpZ_first)
			{
				tmpZ_first = plane_xyz[L_shape_plane_idx[i].first][j].z;
			}
		}
		//cout << " show twoPlanePoint second size = " << plane_xyz[L_shape_plane_idx[i].second].size() << endl;
		for (int k = 0; k < plane_xyz[L_shape_plane_idx[i].second].size(); k++)
		{
			if (plane_xyz[L_shape_plane_idx[i].second][k].z > tmpZ_second)
			{
				tmpZ_second = plane_xyz[L_shape_plane_idx[i].second][k].z;
			}
			cv::Mat point_tmp = cv::Mat(1, 3, CV_32F);
			point_tmp.at<float>(0, 0) = plane_xyz[L_shape_plane_idx[i].second][k].x;
			point_tmp.at<float>(0, 1) = plane_xyz[L_shape_plane_idx[i].second][k].y;
			point_tmp.at<float>(0, 2) = plane_xyz[L_shape_plane_idx[i].second][k].z;
			twoPlanePoint.push_back(point_tmp);
		}

		//cout << " show twoPlanePoint .row = " << twoPlanePoint.rows << endl;


		float chooseZ = min(tmpZ_first, tmpZ_second);
		//cout << " show choose z === " << chooseZ << endl;
		float Z_length = chooseZ - resultPoint.z;
		//cout << " show Z_length  === " << Z_length << endl;


		cv::flann::Index  kdtree;
		kdtree.build(twoPlanePoint, cv::flann::KDTreeIndexParams(1), cvflann::FLANN_DIST_EUCLIDEAN);
		cv::Mat indices, distance;
		//kdtree.knnSearch(selectwallcenter, indices, distance, 1, cv::flann::SearchParams(-1));
		std::vector<float> distanceVector;
		for (size_t z_value = 20; z_value < Z_length; z_value += 20)
		{
			cv::Point3f  pickPoint;
			pickPoint.z = resultPoint.z + z_value;
			pickPoint.x = (plane_normal2[1] * (plane_normal1[0] * plane_center_normal1[0] + plane_normal1[1] * plane_center_normal1[1] + plane_normal1[2] * plane_center_normal1[2]) - plane_normal1[1] * (plane_normal2[0] * plane_center_normal2[0] + plane_normal2[1] * plane_center_normal2[1] + plane_normal2[2] * plane_center_normal2[2]) + (plane_normal2[2] * plane_normal1[1] - plane_normal2[1] * plane_normal1[2])*pickPoint.z) / (plane_normal2[1] * plane_normal1[0] - plane_normal1[1] * plane_normal2[0]);
			pickPoint.y = (plane_normal2[0] * (plane_normal1[0] * plane_center_normal1[0] + plane_normal1[1] * plane_center_normal1[1] + plane_normal1[2] * plane_center_normal1[2]) - plane_normal1[0] * (plane_normal2[0] * plane_center_normal2[0] + plane_normal2[1] * plane_center_normal2[1] + plane_normal2[2] * plane_center_normal2[2]) + (plane_normal2[0] * plane_normal1[2] - plane_normal2[2] * plane_normal1[0])*pickPoint.z) / (plane_normal2[0] * plane_normal1[1] - plane_normal1[0] * plane_normal2[1]);
			//cout << " show pickPoint num == " << pickPoint << endl;
			cv::Mat pickPointMat = cv::Mat(1, 3, CV_32F);
			pickPointMat.at<float>(0, 0) = pickPoint.x;
			pickPointMat.at<float>(0, 1) = pickPoint.y;
			pickPointMat.at<float>(0, 2) = pickPoint.z;
			kdtree.knnSearch(pickPointMat, indices, distance, 10, cv::flann::SearchParams(-1));
			//cout << " show kdtree indices = " << indices << endl;
			//cout << "indices .rows = " << indices.rows << " cols = " << indices.cols << endl;
			float total_x = 0.f;
			float total_y = 0.f;
			float total_z = 0.f;
			for (int index = 0; index < indices.cols; index++)
			{
				float tmpx = twoPlanePoint.at<float>(indices.at<int>(0, index), 0);
				float tmpy = twoPlanePoint.at<float>(indices.at<int>(0, index), 1);
				float tmpz = twoPlanePoint.at<float>(indices.at<int>(0, index), 2);

				//cout << "tmpx == " << tmpx << " tmpy == " << tmpy << " tmpz == " << tmpz << endl;
				total_x += tmpx;
				total_y += tmpy;
				total_z += tmpz;
			}
			//cout << "total_x == " << total_x << " total_y == " << total_y << " total_z == " << total_z << endl;
			pickPoint.x = total_x / 10;
			pickPoint.y = total_y / 10;
			pickPoint.z = total_z / 10;
			//cout << " show pickPoint last last == " << pickPoint << endl;

			//(x-x0)/m = (y-y0)/n = (z-z0)/p;
			cv::Point3f CenterAxisPoint;
			CenterAxisPoint.z = 0.f;
			CenterAxisPoint.x = groundCenterPoint.x - groundNormalPoint.x * groundCenterPoint.z / groundNormalPoint.z;
			CenterAxisPoint.y = groundCenterPoint.y - groundNormalPoint.y*groundCenterPoint.z / groundNormalPoint.z;

			std::pair<cv::Point3f, float> distancePoint = MathOperation::CalPointIntersectLine(pickPoint, groundCenterPoint, CenterAxisPoint);
			//cout << " distancePoint ======================== " << distancePoint.second << endl;
			distanceVector.push_back(distancePoint.second);
		}
		/*float tmpMax = -10000;
		for (auto v : distanceVector)
		{
			if (tmpMax < v) tmpMax = v;
		}
		float tmpMin = 10000;
		for (auto v : distanceVector)
		{
			if (tmpMin > v) tmpMin = v;
		}*/

		float tmpSum = std::accumulate(std::begin(distanceVector),std::end(distanceVector),0.0);
		float tmpMean = tmpSum / distanceVector.size();

		float tmpVariance = 0.0;
		for (int v = 0; v < distanceVector.size(); v++)
		{
			tmpVariance += pow(distanceVector[v] - tmpMean, 2);
		}
		tmpVariance = tmpVariance / distanceVector.size();

		float standard_deviation = sqrt(tmpVariance);

		//cout << " test tmpVariance == " << tmpVariance << " standard_deviation " << standard_deviation << endl;

		straightness.push_back(std::pair<std::pair<int, int>, float>(L_shape_plane_idx[i], tmpVariance));

	}
	return true;
}

ResultData RoomMeasurement::MeasureGroundLevelness(void)
{
	std::vector<std::pair<float, cv::Point3f>>  levelness;

	ResultData  result;
	MeasureLevelnessRange measure_levelness_range;
	std::vector<cv::Point3f> pts = mPRecon->GetRoomContour();
	//std::vector<int> GrdIds = mPRecon->GetPlaneList(ePLANE_GROUND);
	//std::vector<StructuredPlane> planes = mPRecon->GetStructuredPlanes();
	std::vector<int> plane_ground_idx = mPRecon->GetPlanes().plane_ground_idx;
	std::vector<int> plane_wall_idx = mPRecon->GetPlanes().plane_wall_idx;
	std::vector<cv::Point3f> plane_normals = mPRecon->GetPlanes().plane_normals;
	std::vector<cv::Point3f> plane_centers = mPRecon->GetPlanes().plane_center;

	for (int i = 0; i < plane_ground_idx.size(); i++)
	{
		std::vector<cv::Point3f> plane_points = mPRecon->GetPlanes().plane_xyz[plane_ground_idx[i]];
		cv::Point3f normal = plane_normals[plane_ground_idx[i]];
		float ground_normal[3];
		ground_normal[0] = normal.x;
		ground_normal[1] = normal.y;
		ground_normal[2] = normal.z;
		float plane_normal[3];
		plane_normal[0] = plane_normals[plane_wall_idx[0]].x;
		plane_normal[1] = plane_normals[plane_wall_idx[0]].y;
		plane_normal[2] = std::abs(ground_normal[2]);
		float plane_center[3];
		plane_center[0] = plane_centers[plane_wall_idx[0]].x;
		plane_center[1] = plane_centers[plane_wall_idx[0]].y;
		plane_center[2] = plane_centers[plane_wall_idx[0]].z;
		if (MathOperation::ComputeVectorDotProduct(plane_normal, plane_center) > 0.f)
		{
			plane_normal[0] *= -1.f;
			plane_normal[1] *= -1.f;
			plane_normal[2] *= -1.f;
		}
		MeasurementResultValueValuesPoints levelnessOld; ////T
		std::vector<cv::Point3f> actual_vertices;
		measure_levelness_range.MeasureLevelnessRangeFcn(plane_normal, ground_normal, plane_points,
			levelnessOld, actual_vertices, NULL);

		std::vector<cv::Point3f> m_pts;
		if (levelnessOld.is_valid) {
			for (int j = 0; j < levelnessOld.points.size(); j++) {
				std::pair<float, cv::Point3f> temp;
				if (std::get<0>(levelnessOld.points[j])) {
					temp.first = std::get<1>(levelnessOld.points[j]);//val
					temp.second = std::get<2>(levelnessOld.points[j]);//pts
					m_pts.push_back(temp.second);
				}
				levelness.push_back(temp);
			}
		}

		std::vector<MeasurementRulerverticeStruct> ruler_vertice_intersect_pts_dis;
		ruler_vertice_intersect_pts_dis.resize(levelnessOld.points.size());

		for (unsigned int i = 0; i < levelnessOld.points.size(); i++)
		{
			ruler_vertice_intersect_pts_dis[i].is_valid = std::get<0>(levelnessOld.points[i]);
			ruler_vertice_intersect_pts_dis[i].value = std::get<1>(levelnessOld.points[i]);
		}

		measure_levelness_range.GetLevelnessLocalRulersFromPlaneCore(pts, m_pts, ruler_vertice_intersect_pts_dis);
#ifdef Debug_L
		std::cout << "levelnessOld.points.size:  " << levelnessOld.points.size() << endl;
		std::cout << "ruler_vertice_intersect_pts_dis.size:  " << ruler_vertice_intersect_pts_dis.size() << endl;
		std::cout << "GroundLeveness: \n";
		for (int i = 0; i < ruler_vertice_intersect_pts_dis.size(); i++)
		{
#if 1
			if (ruler_vertice_intersect_pts_dis[i].is_valid) {
				std::cout << "\n" << ruler_vertice_intersect_pts_dis[i].value << " , " << ruler_vertice_intersect_pts_dis[i].ruler_endpts << std::endl;
				std::cout << ruler_vertice_intersect_pts_dis[i].endpt_intersect_dist[0].first << " , " << ruler_vertice_intersect_pts_dis[i].endpt_intersect_dist[0].second << std::endl;
				std::cout << ruler_vertice_intersect_pts_dis[i].endpt_intersect_pts[0].first << " ," << ruler_vertice_intersect_pts_dis[i].endpt_intersect_pts[0].second << std::endl;
			}
#endif
		}
#endif
		result.ground_levelness = levelnessOld;
		result.ground_levelness_local_ruler_vertice_intersect_pts_dis = ruler_vertice_intersect_pts_dis;
	}

	return result;
}

double getLinesAngle(cv::Point2d Pt1, cv::Point2d Pt2, cv::Point2d Pt3, cv::Point2d Pt4)
{
	long double x, y;
	x = (Pt1.x - Pt2.x)*(Pt1.x - Pt2.x) + (Pt1.y - Pt2.y)*(Pt1.y - Pt2.y);
	y = (Pt3.x - Pt4.x)*(Pt3.x - Pt4.x) + (Pt3.y - Pt4.y)*(Pt3.y - Pt4.y);

	if ((Pt1 == Pt3) && (Pt2 == Pt4) ||(Pt1 == Pt2) && (Pt3 == Pt4))
	{	return 0; }

	long double s1, s2;
	s1 = sqrt(x);
	s2 = sqrt(y);
	long double s3;
	s3 = (Pt2.x - Pt1.x)*(Pt4.x - Pt3.x) + (Pt2.y - Pt1.y)*(Pt4.y - Pt3.y);
	long double s4;
	s4 = abs(s3);
	long double ans, ans2;
	ans2 = s4 / (s1*s2);
	ans = acos(ans2);
	//cout << "ans2: " << ans2 << " ans: " << ans << endl;
	long double ans3;
	ans3 = (180 * ans) / M_PI;
	return (double)ans3;
}

cv::Point2f RoomMeasurement::GetFootOfPerpendicular(const cv::Point2f &pt, const cv::Point2f &begin, const cv::Point2f &end)
{
	cv::Point2f retVal;
	double dx = begin.x - end.x;
	double dy = begin.y - end.y;
	if (abs(dx) < 0.00000001 && abs(dy) < 0.00000001)
	{
		retVal = begin;
		return retVal;
	}

	double u = (pt.x - begin.x)*(begin.x - end.x) +
		(pt.y - begin.y)*(begin.y - end.y);
	u = u / ((dx*dx) + (dy*dy));

	retVal.x = begin.x + u*dx;
	retVal.y = begin.y + u*dy;

	return retVal;
}

cv::Point3f RoomMeasurement::GetFootOfPerpendicular3f(const cv::Point3f &pt, const cv::Point3f &begin, const cv::Point3f &end)
{
	cv::Point3f retVal;
	double dx = begin.x - end.x;
	double dy = begin.y - end.y;
	if (abs(dx) < 0.00000001 && abs(dy) < 0.00000001)
	{
		retVal = begin;
		return retVal;
	}

	double u = (pt.x - begin.x)*(begin.x - end.x) +
		(pt.y - begin.y)*(begin.y - end.y);
	u = u / ((dx*dx) + (dy*dy));

	retVal.x = begin.x + u * dx;
	retVal.y = begin.y + u * dy;
	retVal.z = pt.z;

	return retVal;
}

void RoomMeasurement::getLinePara(cv::Point2f p1, cv::Point2f p2, cv::Point2f &Pa)
{
	if (abs(p1.x - p2.x) < 0.01)
	//if (p1.x == p2.x)
	{
		Pa.x = 0;
		Pa.y = 0;
	}else{
		Pa.x = (p2.y - p1.y) / (p2.x - p1.x);
		Pa.y = p1.y - Pa.x * p1.x;
	}
}

void Rotate2(Point2f pt0, double alpha, Point2f pt1)
{
//	pt1.x = pt0.x * cos(alpha) - pt0.y * sin(alpha);
//	pt1.y = pt0.x * sin(alpha) + pt0.y * cos(alpha);
	pt1.x = pt0.x * cos(alpha) + pt0.y * sin(alpha);
	pt1.y = -pt0.x * sin(alpha) + pt0.y * cos(alpha);

}

double RoomMeasurement::GetAngle(double x1, double y1, double x2, double y2, double x3, double y3)
{
	double theta = atan2(x1 - x2, y1 - y2) - atan2(x3 - x2, y3 - y2);
	if (theta > M_PI)
		theta -= 2 * M_PI;
	if (theta < -M_PI)
		theta += 2 * M_PI;

	theta = abs(theta * 180.0 / M_PI);
	return theta;
}

double RoomMeasurement::GetAngleRel(double x1, double y1, double x2, double y2, double x3, double y3)
{
	double theta = atan2(x1 - x2, y1 - y2) - atan2(x3 - x2, y3 - y2);
	if (theta > M_PI)
		theta -= 2 * M_PI;
	if (theta < -M_PI)
		theta += 2 * M_PI;

	theta = theta * 180.0 / M_PI;
	return theta;
}

bool RoomMeasurement::isPositive(cv::Point2f p0, cv::Point2f p1, cv::Point2f p2)
{
#if 1
	double ag = GetAngleRel((double)p1.x, (double)p1.y, (double)p0.x, (double)p0.y,(double)p2.x,(double)p2.y);
	//double ag = GetAngleRel((double)p0.x, (double)p0.y, (double)p1.x, (double)p1.y, (double)p2.x, (double)p2.y);
	if (((abs(p0.x - p1.x) > abs(p0.y - p1.y)) && (p0.x > p1.x)) ||
		((abs(p0.y - p1.y) > abs(p0.x - p1.x)) && (p0.y < p1.y))) {
		//cout << "===normal" << endl;
		if (ag > 0) { return true;}
		else { return false;}
	}else {
		//cout << "===reversed" << endl;
		if (ag > 0) { return true; }
		else { return false; }
	}
#else
	cv::Point2f ori, p02a;
	ori.x = ori.y = 0;
	cv::Point2f pm = (p0 + p2) / 2;
	float dist1 = sqrt(std::pow(p1.x - ori.x, 2.0) + std::pow(p1.y - ori.y, 2.0));
	float dist2 = sqrt(std::pow(pm.x - ori.x, 2.0) + std::pow(pm.y - ori.y, 2.0));
	if (dist1 < dist2) {
		return true;
	}
	else {
		return false;
	}
#endif
}

cv::Point3f UniformNormals(const cv::Point3f normal, const cv::Point3f center)
{
	if (normal.dot(center) > 0.f)
	{
		return -normal;
	}
	return normal;
}

bool RoomMeasurement::getCorssedPoint(cv::Point2f pt0, cv::Point2f pt1, cv::Point2f pt2, cv::Point2f pt3, cv::Point2f& cp)
{
	bool isCorssed = false;
	cv::Point2f pa, pb;
	getLinePara(pt0, pt1, pa);
	getLinePara(pt2, pt3, pb);
	 if ((pa.x ==0)&&(pa.y==0) && (pb.x == 0) && (pb.y == 0)) //a//b//y
	 {
		 isCorssed = false;
	 }
	 else if ((pa.x == 0) && (pa.y == 0)) // a//y, b!//y
	 {
		 cp.x = pt1.x;
		 cp.y = pb.x * cp.x + pb.y;
		 if (((pt2.x < pt3.x) && (cp.x >= pt2.x) && (cp.x <= pt3.x)) ||
			 ((pt2.x > pt3.x) && (cp.x <= pt2.x) && (cp.x >= pt3.x))) {
			 isCorssed = true;
		 }
	 }
	 else if ((pb.x == 0) && (pb.y == 0)) // b//y, a !//y
	 {
		 cp.x = pt2.x;
		 cp.y = pa.x * cp.x + pa.y;
		 if (((pt2.y < pt3.y) && (cp.y >= pt2.y) && (cp.y <= pt3.y)) ||
			 ((pt2.y > pt3.y) && (cp.y <= pt2.y) && (cp.y >= pt3.y))) {
			 isCorssed = true;
		 }
	 }
	 else { //none //y
		 if (pa.x == pb.x) {  //a//b
			 isCorssed = false;
		 }
		 else {
			 cp.x = (pa.y - pb.y) / (pb.x - pa.x);
			 cp.y = pa.x *cp.x + pa.y;

			 if (((pt2.y < pt3.y) && (cp.y >= pt2.y) && (cp.y <= pt3.y)) ||
				 ((pt2.y > pt3.y) && (cp.y <= pt2.y) && (cp.y >= pt3.y)))
				 isCorssed = true;
			 }
		}
	return isCorssed;
}

void RoomMeasurement::getTempPoint(cv::Point2f pt0, cv::Point2f pt1, cv::Point2f pt2, cv::Point2f& cp)
{
#if 0
	bool isCorssed = false;
	cv::Point2f pa,pa1,pa2,ppa, pb;
	getLinePara(pt0, pt1, pa);

	if ((pa.x == 0) && (pa.y == 0))
	{
		cp.x = pt2.x;
		cp.y = pt1.y;
	}
	else {
		pa2.x = pa.x;
		pa2.y = pt2.y - pt2.x * pa2.x;

		if (pa.x != 0) {
			pa1.x = -1 / pa.x;
			pa1.y = pt1.y - pt1.x * pa1.x;
			cp.x = (pa2.y - pa1.y) / (pa1.x - pa2.x);
			cp.y = pa1.x * cp.x + pa1.y;
			//cp.y = pa2.x * cp.x + pa2.y;
		}
		else {
			cp.x = pt1.x;
			cp.y = pt2.y;
		}
	}
#else
	cv::Mat rotation_matrix_R;
	float AXIS_X_DIRECTION[3] = { 1.f,0.f,0.f };
	float vector_before[3] = { pt0.x - pt1.x, pt0.y - pt1.y, 0 };
	cv::Mat rotation_matrix = MeasureBase::CalRotationMatrixFromVectors(vector_before, AXIS_X_DIRECTION);
	cv::transpose(rotation_matrix, rotation_matrix_R);
	cv::Point3f p2,p22, p1, p11, temp, temp2;
	p2.x = pt2.x; p2.y = pt2.y; p2.z = 0;
	p1.x = pt1.x; p1.y = pt1.y; p1.z = 0;

	//cout << rotation_matrix << endl;
	MeasureBase::RotatePoint(p1, rotation_matrix, p11);
	MeasureBase::RotatePoint(p2, rotation_matrix, p22);
	temp.x = p11.x;temp.y = p22.y; temp.z = 0;
	MeasureBase::RotatePoint(p11, rotation_matrix_R, p1);
	MeasureBase::RotatePoint(p22, rotation_matrix_R, p2);
	MeasureBase::RotatePoint(temp, rotation_matrix_R, temp2);
	cp.x = temp2.x;
	cp.y = temp2.y;
#endif
}

std::vector<cv::Point2f>RoomMeasurement::ContourSquare(std::vector<cv::Point3f>& pts0, std::vector<cv::Point3f>& pts, float &LAera,
	                                                   int ref_wall_contour_id,
	                                                   std::vector<int>& wall_list )
{
	log_info("ContourSquare 2D start!");
	std::vector<cv::Point2f> PtsForRoomSquared;

#ifdef Debug_RW
	cout<< "=========ref_wall_contour_id:" << ref_wall_contour_id << endl;
	float minX = 10000, maxX = -10000, minY = 10000, maxY = -10000;
	for (int i = 0; i < pts0.size(); i++)
	{
		float X = pts0[i].x;
		float Y = pts0[i].y;
		if (X > maxX) { maxX = X;}
		if (X < minX) {minX = X;}
		if (Y > maxY) {maxY = Y;}
		if (Y < minY) {minY = Y;}
	}
	int imgw = (maxX - minX) / 10 + 1;
	int imgh = (maxY - minY) / 10 + 1;
	int pending = 10;
	cv::Mat contourMap = cv::Mat(imgh + pending * 2, imgw + pending * 2, CV_8UC1);
#endif

	float wallThre = 50.0;
	float angThre =  179.0;

	for (int i = 0; i < pts0.size(); i++)
	{
		cv::Point2f pt;
		pt.x = pts0[i].x; pt.y = pts0[i].y;
		PtsForRoomSquared.push_back(pt);
	}

	for (int i = 0; i < PtsForRoomSquared.size(); i++)
	{
		int round = PtsForRoomSquared.size();
		int h = ref_wall_contour_id + i;
#ifdef Debug_R
		cout << "\n\n============== PtsForRoomSquared" << PtsForRoomSquared.size() << " \nh: " << (h) % round << "\n" << PtsForRoomSquared << endl;
		cout << "------------------\n" << PtsForRoomSquared[(h) % round] << "\n"
			<< PtsForRoomSquared[(h + 1) % round] << "\n"
			<< PtsForRoomSquared[(h + 2) % round] << "\n"
			<< PtsForRoomSquared[(h + 3) % round] << "\n";
#endif
		cv::Point2f foot, pa, pa1, pa2, pa3, ppa;
		cv::Point2f pt0, pt1, pt2, pt3;
		pt0.x = PtsForRoomSquared[(h) % round].x;     pt0.y = PtsForRoomSquared[(h) % round].y;
		pt1.x = PtsForRoomSquared[(h + 1) % round].x; pt1.y = PtsForRoomSquared[(h + 1) % round].y;
		pt2.x = PtsForRoomSquared[(h + 2) % round].x; pt2.y = PtsForRoomSquared[(h + 2) % round].y;
		pt3.x = PtsForRoomSquared[(h + 3) % round].x; pt3.y = PtsForRoomSquared[(h + 3) % round].y;

		auto Angle = GetAngle(pt0.x, pt0.y, pt1.x, pt1.y, pt2.x, pt2.y);

		if (!isPositive(pt0, pt1, pt2))
		{
#ifdef Debug_R
			cout << "#############Negative, angle: " << Angle << endl;
#endif
			if (Angle < 90.0) {
				foot = GetFootOfPerpendicular(pt2, pt0, pt1);
#ifdef Debug_R
				cout << "N < 90 update h+1 foot: " << foot << endl;
#endif
				//// eg1
				if (abs(PtsForRoomSquared[(h + 1) % round].x - foot.x) > abs(PtsForRoomSquared[(h + 1) % round].y - foot.y)) {
					LAera += (abs(PtsForRoomSquared[(h + 1) % round].x - foot.x) *abs(PtsForRoomSquared[(h+2) % round].y - foot.y)) / 2.0;
				}else {
					LAera += (abs(PtsForRoomSquared[(h + 1) % round].y - foot.y) *abs(PtsForRoomSquared[(h+2) % round].x - foot.x)) / 2.0;
				}

				PtsForRoomSquared[(h + 1) % round] = foot;

			}
			else if (Angle > 90.0) {
				cv::Point2f temp, cp;
				getTempPoint(pt0, pt1, pt2, temp);
				if (getCorssedPoint(pt1, temp, pt2, pt3, cp)) { // pt1 temp 和  pt2pt3 lineseg corssed
#ifdef Debug_R
					cout << "N > 90, update h_+2 cp: " << cp << endl;
#endif
					//// eg2
					if (abs(PtsForRoomSquared[(h + 1) % round].x - cp.x) > abs(PtsForRoomSquared[(h + 1) % round].y - cp.y)) {
						LAera += (abs(PtsForRoomSquared[(h + 1) % round].x - cp.x) *abs(PtsForRoomSquared[(h+2) % round].y - cp.y)) / 2.0;
					}
					else {
						LAera += (abs(PtsForRoomSquared[(h + 1) % round].y - cp.y) *abs(PtsForRoomSquared[(h+2) % round].x - cp.x)) / 2.0;
					}

					PtsForRoomSquared[(h + 2) % round] = cp;

				}
				else {
					float dist1 = sqrt(std::pow(temp.x - pt2.x, 2.0) + std::pow(temp.y - pt2.y, 2.0));
					if (dist1 < wallThre) {
#ifdef Debug_R
						cout << "N > 90, update h_+2 temp: " << temp << endl;
#endif
						////eg3
						if (abs(PtsForRoomSquared[(h + 1) % round].x - temp.x) > abs(PtsForRoomSquared[(h + 1) % round].y - temp.y)) {
							LAera += (abs(PtsForRoomSquared[(h + 1) % round].x - temp.x) *abs(PtsForRoomSquared[(h + 2) % round].y - temp.y)) / 2.0;
						}
						else {
							LAera += (abs(PtsForRoomSquared[(h + 1) % round].y - temp.y) *abs(PtsForRoomSquared[(h + 2) % round].x - temp.x)) / 2.0;
						}

						if (abs(PtsForRoomSquared[(h + 3) % round].x - temp.x) > abs(PtsForRoomSquared[(h + 3) % round].y - temp.y)) {
							LAera += (abs(PtsForRoomSquared[(h + 3) % round].x - temp.x) *abs(PtsForRoomSquared[(h + 2) % round].y - temp.y)) / 2.0;
						}
						else {
							LAera += (abs(PtsForRoomSquared[(h + 3) % round].y - temp.y) *abs(PtsForRoomSquared[(h + 2) % round].x - temp.x)) / 2.0;
						}

						PtsForRoomSquared[(h + 2) % round] = temp;

					}
					else {
						auto Angle = GetAngle(temp.x, temp.y, pt2.x, pt2.y, pt3.x, pt3.y);
						if (Angle > angThre) {
#ifdef Debug_R
							cout << "N > 90, Angle < angThre:"<< Angle << angThre<<" temp: " << temp << endl;
#endif
							////eg4 TBD
							if (abs(PtsForRoomSquared[(h + 1) % round].x - temp.x) > abs(PtsForRoomSquared[(h + 1) % round].y - temp.y)) {
								LAera += (abs(PtsForRoomSquared[(h + 1) % round].x - temp.x) *abs(PtsForRoomSquared[(h + 2) % round].y - temp.y)) / 2.0;
							}
							else {
								LAera += (abs(PtsForRoomSquared[(h + 1) % round].y - temp.y) *abs(PtsForRoomSquared[(h + 2) % round].x - temp.x)) / 2.0;
							}

							if (abs(PtsForRoomSquared[(h + 2) % round].x - temp.x) > abs(PtsForRoomSquared[(h + 2) % round].y - temp.y)) {
								LAera += (abs(PtsForRoomSquared[(h + 2) % round].x - temp.x) *abs(PtsForRoomSquared[(h + 3) % round].y - temp.y)) / 2.0;
							}
							else {
								LAera += (abs(PtsForRoomSquared[(h + 2) % round].y - temp.y) *abs(PtsForRoomSquared[(h + 3) % round].x - temp.x)) / 2.0;
							}

							PtsForRoomSquared[(h + 2) % round] = temp;

						}
						else {
#ifdef Debug_R
							cout << "\n\n N > 90, insert temp: " << temp << endl;
#endif

							////eg5
							if (abs(PtsForRoomSquared[(h + 1) % round].x - temp.x) > abs(PtsForRoomSquared[(h + 1) % round].y - temp.y)) {
								LAera += (abs(PtsForRoomSquared[(h + 1) % round].x - temp.x) *abs(PtsForRoomSquared[(h + 2) % round].y - temp.y)) / 2.0;
							}
							else {
								LAera += (abs(PtsForRoomSquared[(h + 1) % round].y - temp.y) *abs(PtsForRoomSquared[(h + 2) % round].x - temp.x)) / 2.0;
							}
							PtsForRoomSquared.insert(PtsForRoomSquared.begin() + ((h + 2) % round), temp);
							pts0.insert(pts0.begin() + ((h + 2) % round), pts0[(h + 1) % round]);
							pts.insert(pts.begin() + ((h + 2) % round), pts[(h + 1) % round]);
							wall_list.insert(wall_list.begin() + ((h + 2) % round), -1);
							insertMap.push_back((h + 2) % round);
							for (int m = 0; m < insertMap.size(); m++)
							{
								if (((h + 2) % round) < insertMap[m])
								{
									insertMap[m] += 1;
								}
							}


#ifdef Debug_R
							cout << "#### inserted position: " << (h + 2) % round <<"\n"<< endl;
							resultA << "#### inserted position: " << (h + 2) % round << endl;
#endif
						}
					}
				}
			}
		}
		else {
#ifdef Debug_R
			cout << "#############Positive, angle: " << Angle << endl;
#endif
			getLinePara(pt0, pt1, pa);
			getLinePara(pt2, pt3, ppa);

			if (Angle < 90.0) {
				cv::Point2f temp, cp;
				getTempPoint(pt0, pt1, pt2, temp);
				if (getCorssedPoint( pt1,temp, pt2, pt3, cp)) {
#ifdef Debug_R
					cout << "P < 90, update h_+2 cp: " << cp << endl;
#endif
					//// eg1
					if (abs(PtsForRoomSquared[(h + 1) % round].x - cp.x) > abs(PtsForRoomSquared[(h + 1) % round].y - cp.y)) {
						LAera += (abs(PtsForRoomSquared[(h + 1) % round].x - cp.x) *abs(PtsForRoomSquared[(h + 2) % round].y - cp.y)) / 2.0;
					}
					else {
						LAera += (abs(PtsForRoomSquared[(h + 1) % round].y - cp.y) *abs(PtsForRoomSquared[(h + 2) % round].x - cp.x)) / 2.0;
					}

					PtsForRoomSquared[(h + 2) % round] = cp;
				}
				else {
					float dist1 = sqrt(std::pow(temp.x - pt2.x, 2.0) + std::pow(temp.y - pt2.y, 2.0));
					if (dist1 < wallThre)
					{
						////eg2
						if (abs(PtsForRoomSquared[(h + 1) % round].x - temp.x) > abs(PtsForRoomSquared[(h + 1) % round].y - temp.y)) {
							LAera += (abs(PtsForRoomSquared[(h + 1) % round].x - temp.x) *abs(PtsForRoomSquared[(h + 2) % round].y - temp.y)) / 2.0;
						}
						else {
							LAera += (abs(PtsForRoomSquared[(h + 1) % round].y - temp.y) *abs(PtsForRoomSquared[(h + 2) % round].x - temp.x)) / 2.0;
						}

						if (abs(PtsForRoomSquared[(h + 3) % round].x - temp.x) > abs(PtsForRoomSquared[(h + 3) % round].y - temp.y)) {
							LAera += (abs(PtsForRoomSquared[(h + 3) % round].x - temp.x) *abs(PtsForRoomSquared[(h + 2) % round].y - temp.y)) / 2.0;
						}
						else {
							LAera += (abs(PtsForRoomSquared[(h + 3) % round].y - temp.y) *abs(PtsForRoomSquared[(h + 2) % round].x - temp.x)) / 2.0;
						}

						PtsForRoomSquared[(h + 2) % round] = temp;
					}
					else {
						auto Angle = GetAngle(temp.x, temp.y, pt2.x, pt2.y, pt3.x, pt3.y);
						if (Angle > angThre) {
#ifdef Debug_R
							cout << "P < 90, Angle < angThre:" << Angle << angThre << " temp: " << temp << endl;
#endif
                         ////eg3
							if (abs(PtsForRoomSquared[(h + 1) % round].x - temp.x) > abs(PtsForRoomSquared[(h + 1) % round].y - temp.y)) {
								LAera += (abs(PtsForRoomSquared[(h + 1) % round].x - temp.x) *abs(PtsForRoomSquared[(h + 2) % round].y - temp.y)) / 2.0;
							}
							else {
								LAera += (abs(PtsForRoomSquared[(h + 1) % round].y - temp.y) *abs(PtsForRoomSquared[(h + 2) % round].x - temp.x)) / 2.0;
							}

							if (abs(PtsForRoomSquared[(h + 2) % round].x - temp.x) > abs(PtsForRoomSquared[(h + 2) % round].y - temp.y)) {
								LAera += (abs(PtsForRoomSquared[(h + 2) % round].x - temp.x) *abs(PtsForRoomSquared[(h + 3) % round].y - temp.y)) / 2.0;
							}
							else {
								LAera += (abs(PtsForRoomSquared[(h + 2) % round].y - temp.y) *abs(PtsForRoomSquared[(h + 3) % round].x - temp.x)) / 2.0;
							}
							PtsForRoomSquared[(h + 2) % round] = temp;
						}
						else {
#ifdef Debug_R
							cout << "\n\n P < 90, insert temp: " << temp << endl;
#endif
							////eg4
							if (abs(PtsForRoomSquared[(h + 1) % round].x - temp.x) > abs(PtsForRoomSquared[(h + 1) % round].y - temp.y)) {
								LAera += (abs(PtsForRoomSquared[(h + 1) % round].x - temp.x) *abs(PtsForRoomSquared[(h + 2) % round].y - temp.y)) / 2.0;
							}
							else {
								LAera += (abs(PtsForRoomSquared[(h + 1) % round].y - temp.y) *abs(PtsForRoomSquared[(h + 2) % round].x - temp.x)) / 2.0;
							}
							PtsForRoomSquared.insert(PtsForRoomSquared.begin() + ((h + 2) % round), temp);
							pts0.insert(pts0.begin() + ((h + 2) % round), pts0[(h + 1) % round]);
							pts.insert(pts.begin() + ((h + 2) % round), pts[(h + 1) % round]);
							wall_list.insert(wall_list.begin() + ((h + 2) % round), -1);
							insertMap.push_back((h + 2) % round);
							for (int m = 0; m < insertMap.size(); m++)
							{
								if (((h + 2) % round) < insertMap[m])
{
									insertMap[m] += 1;
								}
							}
#ifdef Debug_R
							cout << "#### inserted position: " << (h + 2) % round << "\n" << endl;
							resultA << "#### inserted position: " << (h + 2) % round << endl;
#endif
						}
					}
				}
			}
			else if (Angle > 90.0) {

				cv::Point2f cp;
				if (getCorssedPoint(pt0, pt1, pt2, pt3, cp)) { // pt0 pt1 和  pt2pt3 lineseg corssed
#ifdef Debug_R
					cout << "P > 90, update h_+2 cp: " << cp << endl;
#endif
					////eg5
					if (abs(PtsForRoomSquared[(h + 1) % round].x - cp.x) > abs(PtsForRoomSquared[(h + 1) % round].y - cp.y)) {
						LAera += (abs(PtsForRoomSquared[(h + 1) % round].x - cp.x) *abs(PtsForRoomSquared[(h + 2) % round].y - cp.y)) / 2.0;
					}
					else {
						LAera += (abs(PtsForRoomSquared[(h + 1) % round].y - cp.y) *abs(PtsForRoomSquared[(h + 2) % round].x - cp.x)) / 2.0;
					}

					PtsForRoomSquared[(h + 2) % round] = cp;
				}
				else {
					foot = GetFootOfPerpendicular(pt2, pt0, pt1);
#ifdef Debug_R
					cout << "P >90, update h_+1 foot: " << foot << endl;
#endif
					////eg6
					if (abs(PtsForRoomSquared[(h + 1) % round].x - foot.x) > abs(PtsForRoomSquared[(h + 1) % round].y - foot.y)) {
						LAera += (abs(PtsForRoomSquared[(h + 1) % round].x - foot.x) *abs(PtsForRoomSquared[(h + 2) % round].y - foot.y)) / 2.0;
					}
					else {
						LAera += (abs(PtsForRoomSquared[(h + 1) % round].y - foot.y) *abs(PtsForRoomSquared[(h + 2) % round].x - foot.x)) / 2.0;
					}
					PtsForRoomSquared[(h + 1) % round] = foot;
				}
			}
		}

#if 0 //def Debug_R
		contourMap = 0;
#if 1
		for (int aa = 0; aa < pts0.size(); aa++)
		{
			cv::Point2f pt_s((pts0[aa].x - minX) / 10 + pending, (pts0[aa].y - minY) / 10 + pending);
			cv::Point2f pt_end((pts0[(aa + 1) % pts0.size()].x - minX) / 10 + pending,
				(pts0[(aa + 1) % pts0.size()].y - minY) / 10 + pending);
			cv::line(contourMap, pt_s, pt_end, cv::Scalar(255, 255, 255), 1);
		}
#endif
		cv::imshow("ROOMCONTOUR", contourMap);
		cv::waitKey(0);
#if 1
		for (int aa = 0; aa < PtsForRoomSquared.size(); aa++)
		{
			cv::Point2f pt_s((PtsForRoomSquared[aa].x - minX) / 10 + pending, (PtsForRoomSquared[aa].y - minY) / 10 + pending);
			cv::Point2f pt_end((PtsForRoomSquared[(aa + 1) % PtsForRoomSquared.size()].x - minX) / 10 + pending,
				(PtsForRoomSquared[(aa + 1) % PtsForRoomSquared.size()].y - minY) / 10 + pending);
			//cv::line(contourMap, pt_s, pt_end, cv::Scalar(255, 255, 255), 1, CV_AA);
			cv::line(contourMap, pt_s, pt_end, cv::Scalar(255, 255, 255), 1);
		}
		cv::imshow("ROOMCONTOUR", contourMap);
		cv::waitKey(0);
#endif
#endif
	}///for

//cout << "============LAera:" << LAera << endl;
#ifdef Debug_RW
	cout << " is Showing RoomContourSquared !!!!\n";
	contourMap = 0;
#if 1
	for (int aa = 0; aa < PtsForRoomSquared.size(); aa++)
	{
		cv::Point2f pt_s((PtsForRoomSquared[aa].x - minX) / 10 + pending, (PtsForRoomSquared[aa].y - minY) / 10 + pending);
		cv::Point2f pt_end((PtsForRoomSquared[(aa + 1) % PtsForRoomSquared.size()].x - minX) / 10 + pending,
			(PtsForRoomSquared[(aa + 1) % PtsForRoomSquared.size()].y - minY) / 10 + pending);
		//cv::line(contourMap, pt_s, pt_end, cv::Scalar(255, 255, 255), 1, CV_AA);
		cv::line(contourMap, pt_s, pt_end, cv::Scalar(255, 255, 255), 1);
	}

	cv::imwrite(filePath + std::to_string(countA++)+"_00PtsForRoomSquared.jpg", contourMap);
	//cv::imshow("PtsForRoomSquared", contourMap);
	//cv::waitKey(0);
#endif
	//contourMap = 0;
#if 1
	contourMap = 0;
	for (int aa = 0; aa < pts.size(); aa++)
	{
		cv::Point2f pt_s((pts[aa].x - minX) / 10 + pending, (pts[aa].y - minY) / 10 + pending);
		cv::Point2f pt_end((pts[(aa + 1) % pts.size()].x - minX) / 10 + pending,
			(pts[(aa + 1) % pts.size()].y - minY) / 10 + pending);
		cv::line(contourMap, pt_s, pt_end, cv::Scalar(255, 255, 255), 1);
	}
	cv::imwrite(filePath + std::to_string(countB++) + "_00Pts.jpg", contourMap);
	//cv::imshow("PtsForRoomSquared", contourMap);
	//cv::waitKey(0);
#endif

#if 0
	contourMap = 0;
	for (int aa = 0; aa < pts.size(); aa++)
	{
		cv::Point2f pt_s((pts[aa].x - minX) / 10 + pending, (pts[aa].y - minY) / 10 + pending);
		cv::Point2f pt_end((pts[(aa + 1) % pts.size()].x - minX) / 10 + pending,
			(pts[(aa + 1) % pts.size()].y - minY) / 10 + pending);
		cv::line(contourMap, pt_s, pt_end, cv::Scalar(255, 255, 255), 1);
	}

	for (int aa = 0; aa < PtsForRoomSquared.size(); aa++)
	{
		cv::Point2f pt_s((PtsForRoomSquared[aa].x - minX) / 10 + pending, (PtsForRoomSquared[aa].y - minY) / 10 + pending);
		cv::Point2f pt_end((PtsForRoomSquared[(aa + 1) % PtsForRoomSquared.size()].x - minX) / 10 + pending,
			(PtsForRoomSquared[(aa + 1) % PtsForRoomSquared.size()].y - minY) / 10 + pending);
		cv::line(contourMap, pt_s, pt_end, cv::Scalar(255, 255, 255), 1);
	}
	cv::imshow("PtsForRoomSquared", contourMap);
	cv::waitKey(0);
#endif

#if 1
	contourMap = 0;
	for (int aa = 0; aa < pts.size(); aa++)
	{
		cv::Point2f pt_s((pts[aa].x - minX) / 10 + pending, (pts[aa].y - minY) / 10 + pending);
		cv::Point2f pt_end((pts[(aa + 1) % pts.size()].x - minX) / 10 + pending,
			(pts[(aa + 1) % pts.size()].y - minY) / 10 + pending);
		cv::line(contourMap, pt_s, pt_end, cv::Scalar(255, 255, 255), 1);
	}

	for (int aa = 0; aa < PtsForRoomSquared.size(); aa++)
	{
		cv::Point2f pt_s((PtsForRoomSquared[aa].x - minX) / 10 + pending, (PtsForRoomSquared[aa].y - minY) / 10 + pending);
		cv::Point2f pt_end((PtsForRoomSquared[(aa + 1) % PtsForRoomSquared.size()].x - minX) / 10 + pending,
			(PtsForRoomSquared[(aa + 1) % PtsForRoomSquared.size()].y - minY) / 10 + pending);
		cv::line(contourMap, pt_s, pt_end, cv::Scalar(255, 255, 255), 1);
	}
	cv::imwrite(filePath + std::to_string(countC++) + "_00PRT.jpg", contourMap);
#endif


	assert(resultA.is_open());
	resultA << "ref_wall_contour_id:" << ref_wall_contour_id << endl;

	if (pts.size() == PtsForRoomSquared.size()) {
		resultA << "####No addtional point added!" << endl;
	}
	else {
		resultA << "####Need add Points:" << PtsForRoomSquared.size() - pts.size()
			<< "   pts:" << pts.size() << " PtsForRoomSquared£º" << PtsForRoomSquared.size() << endl;
	}
	resultA << "pts:" << pts.size() << "\n" << pts << endl;
	resultA << "PtsForRoomSquared£º" << PtsForRoomSquared.size() << "\n" << PtsForRoomSquared << endl;
	resultA << "wall_list.size() after:" << wall_list.size() << endl;

	cout << "pts: " << pts.size() << "\n" << pts << endl;
	cout << "PtsForRoomSquared£º" << PtsForRoomSquared.size() << "\n" << PtsForRoomSquared << endl;
#endif
	log_info("ContourSquare 2D end!");
	return PtsForRoomSquared;
}

Eigen::Vector4d equationOfPlane(double x1, double y1, double z1,
	double x2, double y2, double z2,
	double x3, double y3, double z3) {
	// Finding the direction cosines for the normals
	Eigen::Vector3d lineInPlane1(x2 - x1, y2 - y1, z2 - z1);
	Eigen::Vector3d lineInPlane2(x3 - x1, y3 - y1, z3 - z1);
	Eigen::Vector3d normal = lineInPlane1.cross(lineInPlane2);
	normal.normalize();
	// Finding the constant in the lines equation ax + by + cz + d = 0
	Eigen::Vector3d pt0(x1, y1, z1);
	double d = 0 - pt0.dot(normal);
	double a = normal(0), b = normal(1), c = normal(2);
	Eigen::Vector4d v(a, b, c, d);
	return v;
}
bool planeWithPlaneIntersection(Eigen::Vector4d &plane_a, Eigen::Vector4d &plane_b,
	Eigen::VectorXd &line, double angular_tolerance = 0.1) {
	// Calculating and normalizing normal of both the planes
	Eigen::Vector3d n0(plane_a(0), plane_a(1), plane_a(2));
	Eigen::Vector3d n1(plane_b(0), plane_b(1), plane_b(2));
	n0.normalize();
	n1.normalize();

	// Finding angle between the planes
	// If is almost 1, then the planes are parallel
	// Hence, no intersection
	double dotProduct = n0.dot(n1);
	if (std::abs(dotProduct) > 1 - sin(std::abs(angular_tolerance))) {
		return false;
	}

	// Direction cosines of line of intersection is cross product
	Eigen::Vector3d line_direction = n0.cross(n1);
	line_direction.normalized();
	// Modifying the argument meant for result
	line.resize(6);
	line(3) = line_direction(0),
	line(4) = line_direction(1),
	line(5) = line_direction(2);

	// Finding the point
	Eigen::Matrix<double, 2, 2> A;
	Eigen::Matrix<double, 2, 1> B, p;
	B << -plane_a(3), -plane_b(3);
	if (line(3) > line(4) && line(3) > line(5)) {
		A << plane_a(1), plane_a(2),
			plane_b(1), plane_b(2);
		p = A.inverse() * B;
		line(0) = 0;
		line(1) = p(0);
		line(2) = p(1);
	}
	else if (line(4) > line(3) && line(4) > line(5)) {
		A << plane_a(0), plane_a(2),
			plane_b(0), plane_b(2);
		p = A.inverse() * B;
		line(0) = p(0);
		line(1) = 0;
		line(2) = p(1);
	}
	else {
		A << plane_a(0), plane_a(1),
			plane_b(0), plane_b(1);
		p = A.inverse() * B;
		line(0) = p(0);
		line(1) = p(1);
		line(2) = 0;
	}
	// Intersection exists
	return true;
}

bool compareFloatPointPair( const std::pair<float, cv::Point3f> &a, const std::pair<float, cv::Point3f> &b)
{
	return a.first < b.first;
}
bool compln(const std::pair<float, int> &a, const std::pair<float, int> &b)
{
	return a.first > b.first;
}
double ComputePolygonArea(const vector<Point2f> &points)
{
	int point_num = points.size();
	if (point_num < 3)return 0.0;
	double s = points[0].y * (points[point_num - 1].x - points[1].x);
	for (int i = 1; i < point_num; ++i)
		s += points[i].y * (points[i - 1].x - points[(i + 1) % point_num].x);
	return fabs(s / 2.0);
}

std::vector <cv::Point3f> tempr, tempbr;
#define DBRT
float SBH = 2000;
float SBB = 200;
cv::Point3f SBNormal, SBPoint1, SBPoint2;

// -------------------------------------
// SaveDebugImages
// -------------------------------------
void SaveDebugImages(
	const vector<cv::Point3f>& orig3d,
	const vector<cv::Point3f>& squared3d,
	const string& dir)
{
	vector<cv::Point2f> orig, sq;

	orig.reserve(orig3d.size());
	sq.reserve(squared3d.size());

	// drop Z
	for (auto& p : orig3d) orig.emplace_back(p.x, p.y);
	for (auto& p : squared3d) sq.emplace_back(p.x, p.y);

	// compute bounds
	float minx = 1e9, miny = 1e9, maxx = -1e9, maxy = -1e9;
	auto acc = [&](const cv::Point2f& p) {
		minx = min(minx, p.x); miny = min(miny, p.y);
		maxx = max(maxx, p.x); maxy = max(maxy, p.y);
		};
	for (auto& p : orig) acc(p);
	for (auto& p : sq)   acc(p);

	float pad = 50;
	int W = int(maxx - minx + 2 * pad);
	int H = int(maxy - miny + 2 * pad);
	if (W < 400) W = 400;
	if (H < 400) H = 400;

	auto toImg = [&](const cv::Point2f& p) {
		float x = p.x - minx + pad;
		float y = maxy - p.y + pad;  // flip y
		return cv::Point(int(x + 0.5f), int(y + 0.5f));
		};

	// 1. original
	cv::Mat imgO(H, W, CV_8UC3, cv::Scalar(255, 255, 255));
	for (size_t i = 0; i < orig.size(); ++i)
	{
		cv::Point a = toImg(orig[i]);
		cv::Point b = toImg(orig[(i + 1) % orig.size()]);
		cv::line(imgO, a, b, cv::Scalar(255, 0, 0), 5, cv::LINE_AA);
	}
	for (size_t i = 0; i < orig.size(); ++i)
	{
		cv::Point p = toImg(orig[i]);
		cv::circle(imgO, p, 7, cv::Scalar(200, 0, 0), -1);
		cv::putText(imgO, to_string(i), p + cv::Point(5, -5),
			cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
	}

	// 2. squared
	cv::Mat imgS(H, W, CV_8UC3, cv::Scalar(255, 255, 255));
	for (size_t i = 0; i < sq.size(); ++i)
	{
		cv::Point a = toImg(sq[i]);
		cv::Point b = toImg(sq[(i + 1) % sq.size()]);
		cv::line(imgS, a, b, cv::Scalar(0, 0, 255), 5, cv::LINE_AA);
	}
	for (size_t i = 0; i < sq.size(); ++i)
	{
		cv::Point p = toImg(sq[i]);
		cv::circle(imgS, p, 7, cv::Scalar(0, 200, 0), -1);
		cv::putText(imgS, to_string(i), p + cv::Point(5, -5),
			cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
	}

	// 3. overlay
	cv::Mat imgC;
	cv::addWeighted(imgO, 0.5, imgS, 0.5, 0.0, imgC);

	// save
	cv::imwrite(dir + "original.png", imgO);
	cv::imwrite(dir + "squared.png", imgS);
	cv::imwrite(dir + "overlay.png", imgC);
	log_info("save img: %s", (dir + "original.png").c_str());
	log_info("[MakeRoomSquare] Debug images saved.");
	//printf("[MakeRoomSquare] Debug images saved \n");
}

std::vector<std::pair<cv::Point3f, cv::Point3f>> RoomMeasurement::MakeRoomSquare(
	cv::Mat &rotation_matrix0,
	std::vector <std::pair< cv::Point3f, cv::Point3f>> &contour_squared1,
	std::vector<std::vector<std::vector<cv::Point3f>>>& holes_projected,
	std::vector<std::vector<StructuredHole>>& holes_sq,
	std::vector<std::vector<StructuredHole>>& holes_sq1,
	int square_height, int square_width)
{
	log_info("[MakeRoomSquare]:start");
	eRoomSquare_type type = SQUARE_BY_CONVEXITY;
	const int square_mode_raw = GetSquareMode();
	const int square_strategy_mode = GetSquareStrategyMode();
	if (square_strategy_mode == SQUARE_STRATEGY_FIT_MAX_AREA) {
		type = SQUARE_BY_MIN_LOSS;
	}
	else {
		type = SQUARE_BY_CONVEXITY;
	}
	log_info("RoomMeasurement::MakeRoomSquare, square_mode_raw: %d, square_strategy_mode: %d, internal_type: %d",
		square_mode_raw, square_strategy_mode, type);
	std::cout << "RoomMeasurement::MakeRoomSquare, square_mode_raw: " << square_mode_raw
		<< " square_strategy_mode: " << square_strategy_mode
		<< " internal_type: " << type << endl;
	contour_squared1.clear();
	bool custom_square = false;

#ifdef DBRT
	std::vector<cv::Point3f> pts2 = mPRecon->GetRoomContour();
	std::vector<cv::Point3f> pts2b = mPRecon->GetRoomContour();
	std::vector<StructuredPlane> mSPlane2 = mPRecon->GetStructuredPlanes();
	std::vector<StructuredPlane> mSPlane;
	std::vector<cv::Point3f> pts, pts_int;
	std::vector<cv::Point3f> CuboidUpdatedContour(pts2.size()), pts00, pts0, pts1;
	walls_squareness.resize(pts2.size());
	std::vector<std::vector<std::vector<cv::Point3f>>> nHoleVer(pts2.size());
	int round = pts2.size();
#else
	std::vector<cv::Point3f> pts = mPRecon->GetRoomContour();
	std::vector<StructuredPlane> mSPlane = mPRecon->GetStructuredPlanes();
	std::vector<cv::Point3f> CuboidUpdatedContour(pts.size()), pts0, pts1;
	std::vector<std::vector<std::vector<cv::Point3f>>> nHoleVer(pts.size());
	int round = pts.size();
#endif

	if (IsCustomizeSuqare())
	{
		std::cout << "IsCustomizeSuqare(): " << IsCustomizeSuqare() << endl;
		custom_square = true;
		SBH = GetSquareHeight();
		SBB = GetSquareWidth();
	}
	else {
		std::cout << "IsCustomizeSuqare(): " << IsCustomizeSuqare() << endl;
	}

	//custom_square = true;
	std::vector<std::vector<cv::Point3f>> ptsF;

	std::vector<int> wallList0 = mPRecon->GetPlaneList(ePLANE_WALL);
	std::vector<int> wallList = mPRecon->GetWallList();
	std::vector<cv::Point3f> plane_normals = mPRecon->GetPlanes().plane_normals;

#ifdef Debug_R
	cout << "G cornor n: " << pts.size() << endl;
	//cout << pts << endl;
	cout << "wallList.size() before: " << wallList.size() << endl;
	resultA << "wallList.size() before: " << wallList.size() << endl;
	cout << "mSPlane2 n: " << mSPlane.size() << endl;
#endif

	std::vector <std::vector<cv::Point2f>> PtsForRoomSquared;
	std::vector<cv::Point2f> CuboidUpdatedContourSquared;
	std::vector <std::pair< cv::Point2f, cv::Point2f>> RoomContourSquaredJson;
	std::vector <std::pair< cv::Point2f, cv::Point2f>> CuboidContourSquaredJson;
	std::vector <std::pair< cv::Point2f, cv::Point2f>> ConvexityContourSquaredJson;

	std::vector <std::pair< cv::Point3f, cv::Point3f>> RoomContourSquaredJson3d;
	std::vector <std::pair< cv::Point3f, cv::Point3f>> CuboidContourSquaredJson3d;
	std::vector <std::pair< cv::Point3f, cv::Point3f>> ConvexityContourSquaredJson3d;

	std::vector <std::pair< cv::Point2f, cv::Point2f>> ContourSquaredForHoles;
	std::vector <std::pair< cv::Point2f, cv::Point2f>> ContourSquaredForConvexity;
	std::vector <std::pair< cv::Point2f, cv::Point2f>> ContourSquaredFConvexityUpdated;

	//获取参考墙
	int ref_wall_id = mPRecon->GetReferenceWall();
	if (ref_wall_id == -1) {
		std::cout << " Terrible, get no ref wall, use first wall." << endl;
			for (auto wall : wallList) {
			if(wall != -1)
				ref_wall_id = wall;
			}
	}


	std::vector<int> wall_list = mPRecon->GetWallList();
	int ref_wall_contour_id = 0; //shift
	auto ref_wall_it = std::find(wall_list.begin(), wall_list.end(), ref_wall_id);
	if (ref_wall_it != wall_list.end()) {
		ref_wall_contour_id = ref_wall_it - wall_list.begin();
	}

	bool ref_wall_chg = false;
	double angle = 180.0;
	cv::Mat Brotation_matrix = cv::Mat::zeros(3, 3, CV_32FC1);
	cv::Mat b_mat_to_y;
	int nref_idx = 0;
	std::pair<int, cv::Point3f> axis_p;
	bool axis_p_update = false;

	if (IsSquareByAxis())
	{
		std::cout << "IsSquareByAxis(): " << IsSquareByAxis() << endl;
		std::vector<Eigen::Vector4d> axisPlane;
		Eigen::Vector4d groundPlane;
		Eigen::Vector4d meterPlane;
		cv::Point3f p0, p1, p2;
		Eigen::VectorXd myLine;
		bool myIntersect;
		const std::string config_json = GetAxisEqnConfig();
		json_error_t error;
		json_t *root = json_load_file(config_json.c_str(), 0, &error);
		if (root)
		{
			//obtain transformation matrix
			json_t* type = json_object_get(root, "type");
			std::string axisEqnType = (!type) ? "" : json_string_value(type);
			std::cout << "axisEqnType: " << axisEqnType << std::endl;
			if ((axisEqnType == "single") || (axisEqnType == "2D") || (!type)) {

				//axis line
				int axis_line_pl_eqn_size = json_array_size(json_object_get(root, "axis_line_pl_eqn"));
				axisPlane.clear();
				AxisLines.clear();
				for (int i = 0; i < axis_line_pl_eqn_size; i++) {
					auto axis_line_item = json_array_get(json_object_get(root, "axis_line_pl_eqn"), i);
					bool is_axis_pl_fitted = json_is_true(json_object_get(axis_line_item, "is_axis_pl_fitted"));
					std::cout << "[AxisDebug] axis_line_pl_eqn[" << i << "].is_axis_pl_fitted: " << is_axis_pl_fitted << std::endl;
					if (!is_axis_pl_fitted) {
						std::cout << "[AxisDebug] skip unfitted axis line index: " << i << std::endl;
						continue;
					}
					axisPlane.emplace_back(Eigen::Vector4d::Zero());
					AxisLines.emplace_back(Eigen::Vector4d::Zero());
					auto cofe_arr = json_object_get(axis_line_item, "axis_pl_eqn_coef");
					std::cout << "cofe_arr:\n" << json_number_value(json_array_get(cofe_arr, 0)) << "\n" << json_number_value(json_array_get(cofe_arr, 1)) << "\n" << json_number_value(json_array_get(cofe_arr, 2)) << "\n" << json_number_value(json_array_get(cofe_arr, 3)) << endl;
					axisPlane.back()[0] = json_number_value(json_array_get(cofe_arr, 0));
					axisPlane.back()[1] = json_number_value(json_array_get(cofe_arr, 1));
					axisPlane.back()[2] = json_number_value(json_array_get(cofe_arr, 2));
					axisPlane.back()[3] = json_number_value(json_array_get(cofe_arr, 3));

					cv::Mat axisPlaneMat = cv::Mat(4, 1, CV_32FC1);
					axisPlaneMat.at<float>(0, 0) = axisPlane.back()[0];
					axisPlaneMat.at<float>(1, 0) = axisPlane.back()[1];
					axisPlaneMat.at<float>(2, 0) = axisPlane.back()[2];
					axisPlaneMat.at<float>(3, 0) = axisPlane.back()[3];
				}
				std::cout << "[AxisDebug] fitted axis plane count: " << axisPlane.size() << ", total axis_line_pl_eqn count: " << axis_line_pl_eqn_size << std::endl;
				//markers
				if (axisEqnType == "single") {
					int markers_size = json_array_size(json_object_get(root, "markers"));
					std::cout << "markers_size: " << markers_size << std::endl;
					markers_pairs.resize(markers_size);
					for (int i = 0; i < markers_size; i++) {
						cv::Point3f v, vn;
						auto ptsPos = json_object_get(json_array_get(json_object_get(root, "markers"), i), "ptsPos");
						v.x = json_number_value(json_array_get(ptsPos, 0));
						v.y = json_number_value(json_array_get(ptsPos, 1));
						v.z = json_number_value(json_array_get(ptsPos, 2));
						auto normal = json_object_get(json_array_get(json_object_get(root, "markers"), i), "normal");
						vn.x = json_number_value(json_array_get(normal, 0));
						vn.y = json_number_value(json_array_get(normal, 1));
						vn.z = json_number_value(json_array_get(normal, 2));
						markers_pairs[i] = std::make_pair(v, vn);
						std::vector<StructuredPlane> planes = mPRecon->GetStructuredPlanes();
						std::pair<int, std::pair<cv::Point3f, cv::Point3f>> candidate;
						float min_dist = INFINITY;
						for (int j = 0; j < planes.size(); j++) {
							cv::Point3f pt2center_norm = (v - planes[j].center / 1000) / MathOperation::ComputePointToPointDist(planes[j].center / 1000, v);
							//std::cout << v << "," << planes[j].center / 1000 << "," << MathOperation::ComputePointToPointDist(planes[j].center / 1000, v) << std::endl;
							if ((abs(planes[j].normal.dot(vn)) > 0.8) && (abs(pt2center_norm.dot(planes[j].normal)) < 0.01)) {
								if (MathOperation::ComputePointToPointDist(planes[j].center / 1000, v) < min_dist) {
									min_dist = MathOperation::ComputePointToPointDist(planes[j].center / 1000, v);
									candidate = std::make_pair(j, std::make_pair(v, vn));
								}
							}
						}
						if (min_dist < INFINITE) {
							markers_pairs_with_pid.push_back(candidate);
							std::cout << "makers = " << v << ", lies on plane " << candidate.first << ": center diff " << min_dist << std::endl;;
						}
					}
				}
			}
			else if (axisEqnType == "global") {

				//axis line
				int axis_line_pl_eqn_size = json_array_size(json_object_get(root, "axis_line_pl_eqn"));
				axisPlane.clear();
				for (int i = 0; i < axis_line_pl_eqn_size; i++) {
					auto axis_line_item = json_array_get(json_object_get(root, "axis_line_pl_eqn"), i);
					bool is_axis_pl_fitted = json_is_true(json_object_get(axis_line_item, "is_axis_pl_fitted"));
					std::cout << "[AxisDebug] axis_line_pl_eqn[" << i << "].is_axis_pl_fitted: " << is_axis_pl_fitted << std::endl;
					if (!is_axis_pl_fitted) {
						std::cout << "[AxisDebug] skip unfitted axis line index: " << i << std::endl;
						continue;
					}
					axisPlane.emplace_back(Eigen::Vector4d::Zero());
					auto cofe_arr = json_object_get(axis_line_item, "axis_pl_eqn_coef");
					std::cout << "cofe_arr:\n" << json_number_value(json_array_get(cofe_arr, 0)) << "\n" << json_number_value(json_array_get(cofe_arr, 1)) << "\n" << json_number_value(json_array_get(cofe_arr, 2)) << "\n" << json_number_value(json_array_get(cofe_arr, 3)) << endl;
					axisPlane.back()[0] = json_number_value(json_array_get(cofe_arr, 0));
					axisPlane.back()[1] = json_number_value(json_array_get(cofe_arr, 1));
					axisPlane.back()[2] = json_number_value(json_array_get(cofe_arr, 2));
					axisPlane.back()[3] = json_number_value(json_array_get(cofe_arr, 3));
				}
				std::cout << "[AxisDebug] fitted axis plane count: " << axisPlane.size() << ", total axis_line_pl_eqn count: " << axis_line_pl_eqn_size << std::endl;

				//markers
				int markers_size = json_array_size(json_object_get(root, "markers"));
				std::cout << "markers_size: " << markers_size << std::endl;
				markers_pairs.resize(markers_size);
				for (int i = 0; i < markers_size; i++) {
					cv::Point3f v, vn;
					auto ptsPos = json_object_get(json_array_get(json_object_get(root, "markers"), i), "ptsPos");
					v.x = json_number_value(json_array_get(ptsPos, 0));
					v.y = json_number_value(json_array_get(ptsPos, 1));
					v.z = json_number_value(json_array_get(ptsPos, 2));
					auto normal = json_object_get(json_array_get(json_object_get(root, "markers"), i), "normal");
					vn.x = json_number_value(json_array_get(normal, 0));
					vn.y = json_number_value(json_array_get(normal, 1));
					vn.z = json_number_value(json_array_get(normal, 2));
					markers_pairs[i] = std::make_pair(v, vn);
					std::vector<StructuredPlane> planes = mPRecon->GetStructuredPlanes();
					std::pair<int, std::pair<cv::Point3f, cv::Point3f>> candidate;
					float min_dist = INFINITY;
					for (int j = 0; j < planes.size(); j++) {
						cv::Point3f pt2center_norm = (v - planes[j].center / 1000) / MathOperation::ComputePointToPointDist(planes[j].center / 1000, v);
						//std::cout << v << "," << planes[j].center / 1000 << "," << MathOperation::ComputePointToPointDist(planes[j].center / 1000, v) << std::endl;
						if ((abs(planes[j].normal.dot(vn)) > 0.8) && (abs(pt2center_norm.dot(planes[j].normal)) < 0.1)) {
							if (MathOperation::ComputePointToPointDist(planes[j].center / 1000, v) < min_dist) {
								min_dist = MathOperation::ComputePointToPointDist(planes[j].center / 1000, v);
								candidate = std::make_pair(j, std::make_pair(v, vn));
							}
						}
					}
					if (min_dist < INFINITE) {
						markers_pairs_with_pid.push_back(candidate);
						std::cout << "makers = " << v << ", lies on plane " << candidate.first << ": center diff " << min_dist << std::endl;;
					}
				}
			}
			else {
				std::cout << "undefined axis marker equation type" << std::endl;
			}
		}
		else {
			axisPlane.resize(1);
			std::cout << "[AxisDebug] Can not load :" << config_json.c_str() << " use default plane !!!" << endl;
			axisPlane[0][0] = 0; 	axisPlane[0][1] = 1; axisPlane[0][2] = 0; axisPlane[0][3] = 0;
		}
		if (axisPlane.empty()) {
			std::cout << "[AxisDebug] No fitted axis plane from config, fallback to default y=0 plane." << std::endl;
			axisPlane.resize(1);
			axisPlane[0][0] = 0; axisPlane[0][1] = 1; axisPlane[0][2] = 0; axisPlane[0][3] = 0;
		}
		std::cout << "axis plane Eqn:" << axisPlane[0][0] << "," << axisPlane[0][1] << "," << axisPlane[0][2] << endl;
		groundPlane = equationOfPlane(pts2[0].x, pts2[0].y, pts2[0].z, pts2[1].x, pts2[1].y, pts2[1].z, pts2[2].x, pts2[2].y, pts2[2].z);
		if (planeWithPlaneIntersection(axisPlane[0], groundPlane, myLine))
		{
			std::cout << "\n myLine: \n" << myLine << endl;
			std::cout << "plane0:\n" << axisPlane[0] << "\n plane1:\n" << groundPlane << endl;
		}
		else {
			std::cout << "[AxisDebug] planeWithPlaneIntersection failed, axis plane may be parallel/invalid. fallback to reference wall mode." << std::endl;
		}
		cv::Point3f lnor(myLine[3], myLine[4], myLine[5]);
		if (std::abs(lnor.x) < 1e-6f && std::abs(lnor.y) < 1e-6f) {
			lnor = cv::Point3f(axisPlane[0][1], -axisPlane[0][0], 0.f);
			std::cout << "[AxisDebug] use axis plane normal fallback to build line direction: " << lnor << std::endl;
		}
		std::vector<Point3f> lnors;
		lnors.push_back(lnor);
		for (int k = 0; k < pts2.size(); k++)
		{
			cv::Point3f ptsnor(pts2[(k + 1) % pts2.size()].x - pts2[k].x, pts2[(k + 1) % pts2.size()].y - pts2[k].y, 0);
			float t = (lnors[0].x * ptsnor.x + lnors[0].y * ptsnor.y) /
					    (sqrt(pow(lnors[0].x, 2) + pow(lnors[0].y, 2)) * sqrt(pow(ptsnor.x, 2) + pow(ptsnor.y, 2)));
			float t_clamped = std::max(-1.0f, std::min(1.0f, t));
			if (t != t_clamped) {
				std::cout << "[AxisDebug] acos input out of range, clamp from " << t << " to " << t_clamped << std::endl;
			}
			float theta = acos(t_clamped) * (180 / M_PI);

			if (theta > 90.0) {
				if (angle > 180.0 - theta) {
					angle = 180.0 - theta; nref_idx = k;
				}
			}
			else {
				if (angle > theta) {
					angle = theta;	nref_idx = k;
				}
			}
			//cout << "k:" << k << " theta: " << theta << " angle: " << angle << " nref_idx:" << nref_idx << endl;
		}
		//cout << " ================== minang: " << angle << " nref_idx: " << nref_idx << endl;
		if (angle != 0) {
			int rd = pts2.size();
			cv::Point3f pm11, p00, p11, p22, cp, cpb;
			cv::Point2f pm1, p0, p1, p2, pa;
			float AXIS_Y_DIRECTION[3] = { 0.f,1.f,0.f };
			float vector_before[3] = { lnors[0].x, lnors[0].y, 0 };
			Brotation_matrix = MeasureBase::CalRotationMatrixFromVectors(vector_before, AXIS_Y_DIRECTION);
			cv::transpose(Brotation_matrix, b_mat_to_y);
			MeasureBase::RotatePoint(pts2[(nref_idx - 1 + rd) % rd], Brotation_matrix, pm11);
			MeasureBase::RotatePoint(pts2[nref_idx], Brotation_matrix, p00);
			MeasureBase::RotatePoint(pts2[(nref_idx + 1) % rd], Brotation_matrix, p11);
			MeasureBase::RotatePoint(pts2[(nref_idx + 2) % rd], Brotation_matrix, p22);
			//cout << p00 << " " << p11 << " " << p22 << " " << pm11 << endl;
			pm1.x = pm11.x; pm1.y = pm11.y;
			p0.x = p00.x; p0.y = p00.y;
			p1.x = p11.x; p1.y = p11.y;
			p2.x = p22.x; p2.y = p22.y;
			if (abs(p0.y - p1.y) > abs(p0.x - p1.x)) {
				axis_p_update = true;
				if (abs(p0.x) < abs(p1.x)) {
					cp.x = p0.x;
					getLinePara(p1, p2, pa);
					if ((pa.x + pa.y) == 0) cp.y = p1.y;
					else cp.y = pa.x * cp.x + pa.y;
					//cout << p00 <<" "<< cp << endl;
					MeasureBase::RotatePoint(cp, b_mat_to_y, cpb);
					axis_p.first = (nref_idx + 1) % rd; axis_p.second = pts2[(nref_idx + 1) % rd];

					pts2[(nref_idx + 1) % rd].x = cpb.x;
					pts2[(nref_idx + 1) % rd].y = cpb.y;
				}
				else {
					cp.x = p1.x;
					getLinePara(pm1, p0, pa);
					if ((pa.x + pa.y) == 0)	cp.y = p0.y;
					else cp.y = pa.x * cp.x + pa.y;
					//cout << cp <<" "<< p11 << endl;
					MeasureBase::RotatePoint(cp, b_mat_to_y, cpb);
					axis_p.first = nref_idx; axis_p.second = pts2[nref_idx];
					pts2[nref_idx].x = cpb.x;
					pts2[nref_idx].y = cpb.y;
				}
			}
			if (nref_idx != ref_wall_contour_id) {
				ref_wall_chg = true;
				std::cout << "ref_wall_chg = true !!! ori_ref_wall_contour_id:" << ref_wall_contour_id;
				ref_wall_contour_id = nref_idx;
				std::cout << "   new ref_wall_contour_id:" << ref_wall_contour_id << endl;
			}
		}
		else {
			//need deault Brotation_matrix
			std::cout << "Using deault Brotation_matrix." << endl;
			Brotation_matrix.at<float>(0, 0) = cos(0);
			Brotation_matrix.at<float>(0, 1) = -sin(0);
			Brotation_matrix.at<float>(1, 0) = sin(0);
			Brotation_matrix.at<float>(1, 1) = cos(0);
			Brotation_matrix.at<float>(2, 2) = 1.f;
			cv::transpose(Brotation_matrix, b_mat_to_y);
			if (nref_idx != ref_wall_contour_id) {
				ref_wall_chg = true;
				std::cout << "ref_wall_chg = true !!! ori_ref_wall_contour_id:" << ref_wall_contour_id;
				ref_wall_contour_id = nref_idx;
				std::cout << "   new ref_wall_contour_id:" << ref_wall_contour_id << endl;
			}
		}
	}
	else {
		PlaneCutResultInterface  mPlane = mPRecon->GetPlaneCutResultInterface();
		std::vector<ePlane_Direction>  mPlaneDire = mPRecon->GetPlaneDire();
		cv::Point3f normal_plane = mPlane.plane_normals[ref_wall_id];
		std::cout << "IsSquareByAxis(): " << IsSquareByAxis() << " squared with ref_wall: " << ref_wall_id << endl;
		normal_plane = -UniformNormals(normal_plane, mPlane.plane_center[ref_wall_id]);
		cv::Point3f rot_axis = { 0.f, 0.f, 0.f };

		if (mPlaneDire[ref_wall_id] == ePLANE_SOUTH)
			rot_axis.y = -1.0f;
		else if (mPlaneDire[ref_wall_id] == ePLANE_EAST)
			rot_axis.x = 1.0f;
		else if (mPlaneDire[ref_wall_id] == ePLANE_WEST)
			rot_axis.x = -1.0f;
		else
			rot_axis.y = 1.0f;

		cv::Point3f cross = Util_Math::ComputeVectorCrossProduct(normal_plane, rot_axis);
		float  angle_rot = 0;
		angle_rot = (cross.z > 0.f) ? acosf(normal_plane.dot(rot_axis) / std::sqrt(std::pow(normal_plane.x, 2) + std::pow(normal_plane.y, 2))) :
			                          -acosf(normal_plane.dot(rot_axis) / std::sqrt(std::pow(normal_plane.x, 2) + std::pow(normal_plane.y, 2)));
		Brotation_matrix.at<float>(0, 0) = cos(angle_rot);
		Brotation_matrix.at<float>(0, 1) = -sin(angle_rot);
		Brotation_matrix.at<float>(1, 0) = sin(angle_rot);
		Brotation_matrix.at<float>(1, 1) = cos(angle_rot);
		Brotation_matrix.at<float>(2, 2) = 1.f;
		cv::transpose(Brotation_matrix, b_mat_to_y);
	}

	rotation_matrix0 = Brotation_matrix;
	std::cout << "Brotation_matrix:\n" << Brotation_matrix << endl;


	pts = pts_int = MathOperation::plane_rot(Brotation_matrix, pts2b);
	pts0 = MathOperation::plane_rot(Brotation_matrix, pts2);
	for (int i = 0; i < pts0.size(); i++)
	{
		pts1.push_back(pts0[i]);
	}
	//SavePoint3fDataT("pts.txt", pts);
	plane_normals= MathOperation::plane_rot(Brotation_matrix, plane_normals);
	mSPlane.resize(mSPlane2.size());

	if (custom_square) {
		std::vector <float> floo;
		for (auto wall : wallList)
		{
			if (wall == -1)
				continue;
			if (mSPlane2[wall].vertices.size() == 4)
			{
				for (int x = 0; x < mSPlane2[wall].vertices.size(); x++)
				{
					if (mSPlane2[wall].vertices[x].z < 0)
						floo.push_back(mSPlane2[wall].vertices[x].z);
				}
			}
		}
		double SBF = std::accumulate(std::begin(floo), std::end(floo), 0.0);
		SBF = SBF / floo.size();
		//cv::Point3f SBNormal, SBPoint1, SBPoint2;
		SBNormal.x = 0.0;
		SBNormal.y = 0.0;
		SBNormal.z = 1.0;
		SBPoint1.x = 0;
		SBPoint1.y = 0;
		SBPoint1.z = SBF + SBH + SBB / 2;
		SBPoint2.x = 0;
		SBPoint2.y = 0;
		SBPoint2.z = SBF + SBH - SBB / 2;
		log_info("custom_square,  SBF:%f SBH:%f SBB:%f ", SBF, SBH, SBB);
	}

	std::vector<cv::Point3f> SBSquare1;
	std::vector<cv::Point3f> SBSquare2;
	for (auto wall : wallList)
	{
		if (wall == -1)
			continue;

		mSPlane[wall].vertices = MathOperation::plane_rot(Brotation_matrix, mSPlane2[wall].vertices);
		if (custom_square) {
			if (mSPlane[wall].vertices.size() == 4)
			{

				cv::Point3f intSBP1 = MathOperation::CalPlaneLineIntersectPoint(mSPlane[wall].vertices[0], mSPlane[wall].vertices[3], SBNormal, SBPoint1);
				cv::Point3f intSBP2 = MathOperation::CalPlaneLineIntersectPoint(mSPlane[wall].vertices[0], mSPlane[wall].vertices[3], SBNormal, SBPoint2);
				//cout << "wall: " << wall << " intSBP: " << intSBP << endl;
				SBSquare1.push_back(intSBP1);
				SBSquare2.push_back(intSBP2);
			}
			else {
				std::cout << "wall: " << wall << "mSPlane[wall].vertices.size() != 4 !!!!!!!" << endl;
			}
		}

		mSPlane[wall].holes.resize(mSPlane2[wall].holes.size());
		for (int n = 0; n < mSPlane2[wall].holes.size(); n++)
		{
#ifdef Debug_MHS
			cout << "\n====original holes =============wall:" << wall << " holes:" << n << " vertice:\n" << mSPlane2[wall].holes[n].vertice << endl;
			std::stringstream ss;
			ss << n;
			SavePoint3fDataT("original_holes_" + ss.str() + ".txt", mSPlane2[wall].holes[n].vertice);
#endif
			mSPlane[wall].holes[n].vertice = MathOperation::plane_rot(Brotation_matrix, mSPlane2[wall].holes[n].vertice);
#ifdef Debug_MHS
			cout << "====original holes rotated to axis==========wall:" << wall << " holes: " << n << " vertice: \n" << mSPlane[wall].holes[n].vertice << endl;
			SavePoint3fDataT("hole_rot2axis_" + ss.str() + ".txt", mSPlane2[wall].holes[n].vertice);
#endif
		}
	}

	if (custom_square) {
		int i = 0;
		for (auto wall : wallList)
		{
			if (wall == -1)
				continue;
			if (mSPlane[wall].vertices.size() == 4)
			{
				//cout << mSPlane[wall].vertices[0] <<" , "<<SBSquare2[(i + 0) % SBSquare2.size()] << endl;
				mSPlane[wall].vertices[0] = SBSquare2[(i + 0) % SBSquare2.size()];
				mSPlane[wall].vertices[1] = SBSquare2[(i + 1) % SBSquare2.size()];
				mSPlane[wall].vertices[2] = SBSquare1[(i + 1) % SBSquare1.size()];
				mSPlane[wall].vertices[3] = SBSquare1[(i + 0) % SBSquare1.size()];
				i++;
			}
		}
		if (pts.size() == SBSquare2.size())
		{
			pts = SBSquare2;
			std::cout << "\n------- now using SBSquare make roomsquare." << endl;
		}
	}

	std::vector<std::pair<int, std::vector<std::pair<float, cv::Point3f>>>> flateness_defect2 = mPRecon->GetPlanes().removeCritical_flateness_defect;
#if 1
	if (HasOneMeterLine())
	{
		std::vector<cv::Point2f> cont;
		std::vector<int> cIds = mPRecon->GetPlaneList(ePLANE_CEILING);
		for (int i = 0; i < cIds.size(); i++)
			std::cout << "cids: " << cIds[i] << endl;
		std::vector<std::vector<cv::Point3f>> ceiling_int;
		std::vector<std::vector<cv::Point3f>> ground_int;
		float margin = 100;
		for (int i = 0; i < flateness_defect2.size(); i++)
		{
			ceiling_int.resize(mPRecon->GetPlanes().plane_ceiling_idx.size());
			for (int j = 0; j < mPRecon->GetPlanes().plane_ceiling_idx.size(); j++) {
				if ((flateness_defect2[i].first == mPRecon->GetPlanes().plane_ceiling_idx[j])&&(flateness_defect2[i].second.size()>0))
				{
					sort(flateness_defect2[i].second.begin(), flateness_defect2[i].second.end(), compareFloatPointPair);
					CvMat* myPolyc = cvCreateMat(1, pts2b.size(), CV_32FC2);
					for (int t = 0; t < pts2b.size(); t++)
					{
						cvSet1D(myPolyc, t, CvScalar(pts2b[t].x, pts2b[t].y));
					}
					for (int f = 0; f < flateness_defect2[i].second.size(); f++) {
						//CVAPI(double) cvPointPolygonTest(const CvArr* contour,CvPoint2D32f pt, int measure_dist);
						cv::Point2f testp = Point2f(flateness_defect2[i].second[f].second.x, flateness_defect2[i].second[f].second.y);
						if (cvPointPolygonTest(myPolyc, testp, true) > margin) {
							/*std::cout << "plane_ceiling_idx[" << j << "]: " << mPRecon->GetPlanes().plane_ceiling_idx[j]
								<< " cf0: "<< f<<" "<< flateness_defect2[i].second[f].first << " " << flateness_defect2[i].second[f].second << endl;*/
							ceiling_int[j].push_back(flateness_defect2[i].second[f].second);
							break;
						}
					}
					for (int f = flateness_defect2[i].second.size()-1 ; f >=0; f--) {
						cv::Point2f testp = Point2f(flateness_defect2[i].second[f].second.x, flateness_defect2[i].second[f].second.y);
						if (cvPointPolygonTest(myPolyc, testp, true) > margin) {
							/*std::cout << "plane_ceiling_idx[" << j << "]: " << mPRecon->GetPlanes().plane_ceiling_idx[j]
								<< " cf1: " << f << " "<< flateness_defect2[i].second[f].first << " " << flateness_defect2[i].second[f].second << endl;*/
							ceiling_int[j].push_back(flateness_defect2[i].second[f].second);
							break;
						}
					}

					if (ceiling_int[j].size() == 2) {
						std::vector <std::pair< float, cv::Point3f>> temp;
						if (HasOneMeterLine()) {
							temp.push_back(std::make_pair(ceiling_int[j][0].z - GetOneMerterLinePos(), ceiling_int[j][0]));
							temp.push_back(std::make_pair(ceiling_int[j][1].z - GetOneMerterLinePos(), ceiling_int[j][1]));
						}
						else {
							temp.push_back(std::make_pair(ceiling_int[j][0].z, ceiling_int[j][0]));
							temp.push_back(std::make_pair(ceiling_int[j][1].z, ceiling_int[j][1]));
						}
						ceiling_h_onemeter.push_back(std::make_pair(flateness_defect2[i].first, temp));
					}
					else {
						std::cout << "plane_ceiling_idx[" << j << "]: " << mPRecon->GetPlanes().plane_ceiling_idx[j] << " Don't have any valid defect point!!" << endl;
					}

				}
			}
			ground_int.resize(mPRecon->GetPlanes().plane_ground_idx.size());
			for (int j = 0; j < mPRecon->GetPlanes().plane_ground_idx.size(); j++) {
				if ((flateness_defect2[i].first == mPRecon->GetPlanes().plane_ground_idx[j])&&(flateness_defect2[i].second.size()>0))
				{
					sort(flateness_defect2[i].second.begin(), flateness_defect2[i].second.end(), compareFloatPointPair);
					CvMat* myPoly = cvCreateMat(1, pts2b.size(), CV_32FC2);
					for (int t = 0; t < pts2b.size(); t++)
					{
						cvSet1D(myPoly, t, CvScalar(pts2b[t].x, pts2b[t].y));
					}
					for (int f = 0; f < flateness_defect2[i].second.size(); f++) {
						//CVAPI(double) cvPointPolygonTest(const CvArr* contour,CvPoint2D32f pt, int measure_dist);
						cv::Point2f testp = Point2f(flateness_defect2[i].second[f].second.x,
							                        flateness_defect2[i].second[f].second.y);
						if (cvPointPolygonTest(myPoly, testp, true) > margin) {
							std::cout << "plane_ground_idx[" << j << "]: " << mPRecon->GetPlanes().plane_ground_idx[j]
								<< " gf0:" << f << " "
								<< flateness_defect2[i].second[f].first << " " << flateness_defect2[i].second[f].second << endl;

							ground_int[j].push_back(flateness_defect2[i].second[f].second);
							break;
						}
					}
					for (int f = flateness_defect2[i].second.size() - 1; f >= 0; f--) {
						cv::Point2f testp = Point2f(flateness_defect2[i].second[f].second.x,
							                        flateness_defect2[i].second[f].second.y);
						if (cvPointPolygonTest(myPoly, testp, true) > margin) {
							std::cout << "plane_ground_idx[" << j << "]: " << mPRecon->GetPlanes().plane_ground_idx[j]
								<< " gf1:" << f << " "<< flateness_defect2[i].second[f].first << " " << flateness_defect2[i].second[f].second << endl;
							ground_int[j].push_back(flateness_defect2[i].second[f].second);
							break;
						}
					}
					if (ground_int[j].size() == 2) {
						std::vector <std::pair< float, cv::Point3f>> temp;
						if (HasOneMeterLine()) {
							temp.push_back(std::make_pair(GetOneMerterLinePos() - ground_int[j][0].z , ground_int[j][0]));
							temp.push_back(std::make_pair(GetOneMerterLinePos() - ground_int[j][1].z , ground_int[j][1]));
						}
						else {
							temp.push_back(std::make_pair(ground_int[j][0].z, ground_int[j][0]));
							temp.push_back(std::make_pair(ground_int[j][1].z, ground_int[j][1]));
						}
						ground_h_onemeter.push_back(std::make_pair(flateness_defect2[i].first, temp));
					}
					else {
						std::cout << "====plane_ground_idx[" << j << "]: " << mPRecon->GetPlanes().plane_ground_idx[j] << " Don't have any valid defect point!!" << endl;
					}
				}
			}
		}
#if 1
		ceiling_h_onemeter_int.resize(ceiling_int.size());
		for (int j = 0; j < ceiling_int.size(); j++)
		{
			if (ceiling_int[j].size() != 2)
				continue;
			ceiling_int[j] = MathOperation::plane_rot(Brotation_matrix, ceiling_int[j]);
			ceiling_h_onemeter_int[j].resize(ceiling_int[j].size());

			for (int n = 0; n < ceiling_int[j].size(); n++)
			{
				int rd = pts_int.size();
				float min_dis = INFINITE;
				for (int k = 0; k < rd; k++)
				{
					if (std::abs(pts_int[k].x - pts_int[(k + 1) % rd].x) > std::abs(pts_int[k].y - pts_int[(k + 1) % rd].y))
					{   //xy
						if ((((ceiling_int[j][n].x >= pts_int[k].x) && (ceiling_int[j][n].x <= pts_int[(k + 1) % rd].x)) ||
							((ceiling_int[j][n].x <= pts_int[k].x) && (ceiling_int[j][n].x >= pts_int[(k + 1) % rd].x))) &&
							(((ceiling_int[j][n].y >= pts_int[(k + 1) % rd].y) && (ceiling_int[j][0].y <= pts_int[(k + 2) % rd].y)) ||
							((ceiling_int[j][n].y <= pts_int[(k + 1) % rd].y) && (ceiling_int[j][0].y >= pts_int[(k + 2) % rd].y))))
						{
							cv::Point3f cur_int0 = GetFootOfPerpendicular3f(ceiling_int[j][n], pts_int[k], pts_int[(k + 1) % rd]);
							cv::Point3f cur_int1 = GetFootOfPerpendicular3f(ceiling_int[j][n], pts_int[(k + 1) % rd], pts_int[(k + 2) % rd]);
							float cur_dis = sqrt(std::pow(cur_int0.x - cur_int1.x, 2.0) + std::pow(cur_int0.y - cur_int1.y, 2.0));
							if (cur_dis < min_dis) {
								min_dis = cur_dis;
								MeasureBase::RotatePoint(cur_int0, b_mat_to_y, ceiling_h_onemeter_int[j][n].first);
								MeasureBase::RotatePoint(cur_int1, b_mat_to_y, ceiling_h_onemeter_int[j][n].second);
								//cout << "xyceiling_h_onemeter_int n: " << n << " first: " << ceiling_h_onemeter_int[j][n].first << " second: " << ceiling_h_onemeter_int[j][n].second << endl;
							}
						//	break;
						}
					}
					else {//yx
						if ((((ceiling_int[j][n].y >= pts_int[k].y) && (ceiling_int[j][n].y <= pts_int[(k + 1) % rd].y)) ||
							((ceiling_int[j][n].y <= pts_int[k].y) && (ceiling_int[j][n].y >= pts_int[(k + 1) % rd].y))) &&
							(((ceiling_int[j][n].x >= pts_int[(k + 1) % rd].x) && (ceiling_int[j][0].x <= pts_int[(k + 2) % rd].x)) ||
							((ceiling_int[j][n].x <= pts_int[(k + 1) % rd].x) && (ceiling_int[j][0].x >= pts_int[(k + 2) % rd].x))))
						{

							cv::Point3f cur_int0 = GetFootOfPerpendicular3f(ceiling_int[j][n], pts_int[k], pts_int[(k + 1) % rd]);
							cv::Point3f cur_int1 = GetFootOfPerpendicular3f(ceiling_int[j][n], pts_int[(k + 1) % rd], pts_int[(k + 2) % rd]);
							float cur_dis = sqrt(std::pow(cur_int0.x - cur_int1.x, 2.0) + std::pow(cur_int0.y - cur_int1.y, 2.0));
							if (cur_dis < min_dis) {
								min_dis = cur_dis;
								MeasureBase::RotatePoint(cur_int0, b_mat_to_y, ceiling_h_onemeter_int[j][n].first);
								MeasureBase::RotatePoint(cur_int1, b_mat_to_y, ceiling_h_onemeter_int[j][n].second);
								//cout << "yxceiling_h_onemeter_int n: " << n << " first: " << ceiling_h_onemeter_int[j][n].first << " second: " << ceiling_h_onemeter_int[j][n].second << endl;
							}
							//	break;
						}
					}
				}
			}
		}
		ground_h_onemeter_int.resize(ground_int.size());
		for (int j = 0; j < ground_int.size(); j++)
		{
			if (ground_int[j].size() != 2)
				continue;
			ground_int[j] = MathOperation::plane_rot(Brotation_matrix, ground_int[j]);
			ground_h_onemeter_int[j].resize(ground_int[j].size());

			for (int n = 0; n < ground_int[j].size(); n++)
			{
				int rd = pts_int.size();
				float min_dis = INFINITE;
				for (int k = 0; k < rd; k++)
				{
					if (std::abs(pts_int[k].x - pts_int[(k + 1) % rd].x) > std::abs(pts_int[k].y - pts_int[(k + 1) % rd].y))
					{   //xy
						if ((((ground_int[j][n].x > pts_int[k].x) && (ground_int[j][n].x < pts_int[(k + 1) % rd].x)) ||
							((ground_int[j][n].x < pts_int[k].x) && (ground_int[j][n].x > pts_int[(k + 1) % rd].x))) &&
							(((ground_int[j][n].y > pts_int[(k + 1) % rd].y) && (ground_int[j][0].y < pts_int[(k + 2) % rd].y)) ||
							((ground_int[j][n].y < pts_int[(k + 1) % rd].y) && (ground_int[j][0].y > pts_int[(k + 2) % rd].y))))
						{

							cv::Point3f cur_int0 = GetFootOfPerpendicular3f(ground_int[j][n], pts_int[k], pts_int[(k + 1) % rd]);
							cv::Point3f cur_int1 = GetFootOfPerpendicular3f(ground_int[j][n], pts_int[(k + 1) % rd], pts_int[(k + 2) % rd]);
							float cur_dis = sqrt(std::pow(cur_int0.x - cur_int1.x, 2.0) + std::pow(cur_int0.y - cur_int1.y, 2.0));
							if (cur_dis < min_dis) {
								min_dis = cur_dis;
								MeasureBase::RotatePoint(cur_int0, b_mat_to_y, ground_h_onemeter_int[j][n].first);
								MeasureBase::RotatePoint(cur_int1, b_mat_to_y, ground_h_onemeter_int[j][n].second);
								//cout << "xyground_h_onemeter_int n: " << n << " first: " << ground_h_onemeter_int[j][n].first << " second: " << ground_h_onemeter_int[j][n].second << endl;
							}
							//break;
						}
					}
					else {//yx
						if ((((ground_int[j][n].y > pts_int[k].y) && (ground_int[j][n].y < pts_int[(k + 1) % rd].y)) ||
							((ground_int[j][n].y < pts_int[k].y) && (ground_int[j][n].y > pts_int[(k + 1) % rd].y))) &&
							(((ground_int[j][n].x > pts_int[(k + 1) % rd].x) && (ground_int[j][0].x < pts_int[(k + 2) % rd].x)) ||
							((ground_int[j][n].x < pts_int[(k + 1) % rd].x) && (ground_int[j][0].x > pts_int[(k + 2) % rd].x))))
						{
							cv::Point3f cur_int0 = GetFootOfPerpendicular3f(ground_int[j][n], pts_int[k], pts_int[(k + 1) % rd]);
							cv::Point3f cur_int1 = GetFootOfPerpendicular3f(ground_int[j][n], pts_int[(k + 1) % rd], pts_int[(k + 2) % rd]);
							float cur_dis = sqrt(std::pow(cur_int0.x - cur_int1.x, 2.0) + std::pow(cur_int0.y - cur_int1.y, 2.0));
							if (cur_dis < min_dis) {
								min_dis = cur_dis;
								MeasureBase::RotatePoint(cur_int0, b_mat_to_y, ground_h_onemeter_int[j][n].first);
								MeasureBase::RotatePoint(cur_int1, b_mat_to_y, ground_h_onemeter_int[j][n].second);
								//cout << "yxground_h_onemeter_int n: " << n << " first: " << ground_h_onemeter_int[j][n].first << " second: " << ground_h_onemeter_int[j][n].second << endl;
							}
							//	break;
						}
					}
				}
			}
		}
#endif
	}
#endif
#if WALL_CONVPT
	//wallConvPt
	wallConvPt.resize(wallList.size());
	for (int i = 0; i < flateness_defect2.size(); i++){
		for (auto wall : wallList) {
			if (wall == -1)
				continue;
			if (flateness_defect2[i].first == wall)
			{
				sort(flateness_defect2[i].second.begin(), flateness_defect2[i].second.end(), compl);
				cout << "====wallList: " << wall << " "
					<< flateness_defect2[i].second[0].first << " " << flateness_defect2[i].second[0].second << " "
					<< flateness_defect2[i].second[flateness_defect2[i].second.size() - 1].first << " " << flateness_defect2[i].second[flateness_defect2[i].second.size() - 1].second << endl;
				wallConvPt[wall] = flateness_defect2[i].second[flateness_defect2[i].second.size() - 1].second;
			}
		}
	}
#endif

	std::vector<std::pair<int, std::vector<cv::Point3f>>> flateness_defect;
	for (int i = 0; i < flateness_defect2.size(); i++)
	{
		std::vector<cv::Point3f> temp;
		for (int m = 0; m < flateness_defect2[i].second.size(); m++)
		{
			if (custom_square) {
				if ((flateness_defect2[i].second[m].second.z < SBPoint1.z) &&
					(flateness_defect2[i].second[m].second.z > SBPoint2.z))
					{
						temp.push_back(flateness_defect2[i].second[m].second);
					}
			}
			else {
				temp.push_back(flateness_defect2[i].second[m].second);
			}
		}
		temp = MathOperation::plane_rot(Brotation_matrix, temp);
		flateness_defect.push_back(std::make_pair(flateness_defect2[i].first, temp));
	}



	std::cout << "final ref_wall_contour_id:" << ref_wall_contour_id << endl;
	float LAera = 0;
	float shrink = GetMinSquareOffset();
	if ((type == SQUARE_BY_ROOMCONTOUR)||(type == SQUARE_BY_MIN_LOSS))
	{
#ifdef Debug_M
		cout << "\n =========SQUARE_BY_ROOMCONTOUR ref_wall_contour_id:" << ref_wall_contour_id << endl;
#endif
		int minIdx = 0;
		float minLAera = INFINITY;
		if (type == SQUARE_BY_MIN_LOSS) {
			std::vector <float> LAeraS;
			int tempId=0;
			for (int p = 0; p < pts0.size(); p++)
			{
				pts0.clear(); pts1.clear();
				for (int i = 0; i < pts.size(); i++)
				{
					pts0.push_back(pts[i]);
					pts1.push_back(pts[i]);
				}
				std::vector<cv::Point2f > PtsForRoomSquaredTemp;
			    LAera = 0;
				log_info("[ContourSquare]:1");
				PtsForRoomSquaredTemp = ContourSquare(pts0, pts1, LAera, ref_wall_contour_id + p, wall_list);
				if (LAera < minLAera) {
					minLAera = LAera;
					minIdx = p;
					std::cout << "minLAera: " << minLAera << " minIdx:" << minIdx << endl;
				}
				PtsForRoomSquared.push_back(PtsForRoomSquaredTemp);
			}
		}
		else {
			std::vector<cv::Point2f > PtsForRoomSquaredTemp;
			log_info("[ContourSquare]:2");
			PtsForRoomSquaredTemp = ContourSquare(pts0, pts1, LAera, ref_wall_contour_id, wall_list);
			PtsForRoomSquared.push_back(PtsForRoomSquaredTemp);
		}

		if (pts0.size() != PtsForRoomSquared[minIdx].size())
		{
#ifdef Debug_M
			cout << "##### ==> SQUARE_BY_ROOMCONTOUR  pts0.size() != PtsForRoomSquared[minIdx].size()" << endl;
#endif;
			for (int k = 0; k < pts0.size(); k++)
			{
				cv::Point2f pt_s, diff;
				pt_s.x = pts0[k].x;
				pt_s.y = pts0[k].y;
				diff.x = diff.y = 0;
				RoomContourSquaredJson.push_back(std::make_pair(pt_s, diff));
				ContourSquaredForHoles.push_back(std::make_pair(pt_s, diff));
				cv::Point3f diff3;
				diff3.x = diff3.y = diff3.z = 0;
				RoomContourSquaredJson3d.push_back(std::make_pair(pts0[k], diff3));
			}
		}
		else {
#ifdef DBRT
			std::vector<cv::Point3f> temp;
			std::vector<cv::Point3f> PtsForRoomSquared_sh;
			float mx = 0;
			int md = 0;
			//PtsForRoomSquared_sh.resize(PtsForRoomSquared[minIdx].size());
			for (int k = 0; k < PtsForRoomSquared[minIdx].size(); k++)
			{
				cv::Point3f tt;
				tt.x = PtsForRoomSquared[minIdx][k].x;
				tt.y = PtsForRoomSquared[minIdx][k].y;
				tt.z = pts0[k].z;
				temp.push_back(tt);
				PtsForRoomSquared_sh.push_back(tt);
			   // cout << "txt k: " << k << " " << tt << endl;
				if (mx < tt.x) {
					mx = tt.x;	md = k;
					//cout << "md :" << k << endl;
				}
			}
			int rd = PtsForRoomSquared_sh.size();
			//cout << PtsForRoomSquared[minIdx][(md + rd - 1) % rd] << "  " << PtsForRoomSquared[minIdx][md] << " " << PtsForRoomSquared[minIdx][(md + 1) % rd] << endl;
			if ((PtsForRoomSquared[minIdx][(md + rd - 1) % rd].x - PtsForRoomSquared[minIdx][(md + 1) % rd].x)*(PtsForRoomSquared[minIdx][md].y - PtsForRoomSquared[minIdx][(md + 1) % rd].y) -
				(PtsForRoomSquared[minIdx][(md + rd - 1) % rd].y - PtsForRoomSquared[minIdx][(md + 1) % rd].y)*(PtsForRoomSquared[minIdx][md].x - PtsForRoomSquared[minIdx][(md + 1) % rd].x) > 0)
			{
				//cout << "===========counterclockwise." << endl;
				for (int k = 0; k < PtsForRoomSquared_sh.size(); k++)
				{
					if (abs(PtsForRoomSquared_sh[k].x - PtsForRoomSquared_sh[(k + 1) % rd].x) >
						abs(PtsForRoomSquared_sh[k].y - PtsForRoomSquared_sh[(k + 1) % rd].y))
					{
						if (PtsForRoomSquared_sh[k].x < PtsForRoomSquared_sh[(k + 1) % rd].x) {
							PtsForRoomSquared_sh[k].y += shrink;
							PtsForRoomSquared_sh[(k + 1) % rd].y += shrink;
						}
						else {
							PtsForRoomSquared_sh[k].y -= shrink;
							PtsForRoomSquared_sh[(k + 1) % rd].y -= shrink;
						}
					}
					else {
						if (PtsForRoomSquared_sh[k].y < PtsForRoomSquared_sh[(k + 1) % rd].y) {
							PtsForRoomSquared_sh[k].x -= shrink;
							PtsForRoomSquared_sh[(k + 1) % rd].x -= shrink;
						}
						else {
							PtsForRoomSquared_sh[k].x += shrink;
							PtsForRoomSquared_sh[(k + 1) % rd].x += shrink;
						}
					}
				}
			}
			else {
				//cout << "===========clockwise." << endl;
				for (int k = 0; k < PtsForRoomSquared_sh.size(); k++)
				{
					if (abs(PtsForRoomSquared_sh[k].x - PtsForRoomSquared_sh[(k + 1) % rd].x) >
						abs(PtsForRoomSquared_sh[k].y - PtsForRoomSquared_sh[(k + 1) % rd].y))
					{
						if (PtsForRoomSquared_sh[k].x < PtsForRoomSquared_sh[(k + 1) % rd].x) {
							PtsForRoomSquared_sh[k].y -= shrink;
							PtsForRoomSquared_sh[(k + 1) % rd].y -= shrink;
						}
						else {
							PtsForRoomSquared_sh[k].y += shrink;
							PtsForRoomSquared_sh[(k + 1) % rd].y += shrink;
						}
					}
					else {
						if (PtsForRoomSquared_sh[k].y < PtsForRoomSquared_sh[(k + 1) % rd].y) {
							PtsForRoomSquared_sh[k].x += shrink;
							PtsForRoomSquared_sh[(k + 1) % rd].x += shrink;
						}
						else {
							PtsForRoomSquared_sh[k].x -= shrink;
							PtsForRoomSquared_sh[(k + 1) % rd].x -= shrink;
						}

					}
				}
			}
			temp = MathOperation::plane_rot(b_mat_to_y, temp);
			PtsForRoomSquared_sh = MathOperation::plane_rot(b_mat_to_y, PtsForRoomSquared_sh);
			for (int k = 0; k < PtsForRoomSquared[minIdx].size(); k++)
			{
				cv::Point2f tempPts, tempDis;
				tempPts.x = temp[k].x;
				tempPts.y = temp[k].y;
				//cout << "squared rback tempPts i: " << k << " " << tempPts << endl;
				tempDis.x = abs(PtsForRoomSquared[minIdx][k].x - pts[k].x);
				tempDis.y = abs(PtsForRoomSquared[minIdx][k].y - pts[k].y);
				RoomContourSquaredJson.push_back(std::make_pair(tempPts, tempDis));

				cv::Point3f tempPts3, tempDis3;
				tempPts3.x = temp[k].x;
				tempPts3.y = temp[k].y;
				tempPts3.z = temp[k].z;
				//cout << "squared rback tempPts i: " << k << " " << tempPts << endl;
				tempDis3.x = abs(PtsForRoomSquared[minIdx][k].x - pts[k].x);
				tempDis3.y = abs(PtsForRoomSquared[minIdx][k].y - pts[k].y);
				tempDis3.z = 0;
				//cout << "tempDis3: " << tempDis3 << endl;
				RoomContourSquaredJson3d.push_back(std::make_pair(tempPts3, tempDis3));
			}
			if (IsSquareByAxis()) {
				tempbr.push_back(temp[ref_wall_contour_id]);
				tempbr.push_back(temp[(ref_wall_contour_id + 1) % temp.size()]);
			}
			for (int k = 0; k < PtsForRoomSquared_sh.size(); k++)
			{
				cv::Point3f tempPts3, tempDis3;
				tempPts3.x = PtsForRoomSquared_sh[k].x;
				tempPts3.y = PtsForRoomSquared_sh[k].y;
				tempPts3.z = PtsForRoomSquared_sh[k].z;
				//cout << "squared rback tempPts i: " << k << " " << tempPts << endl;
				tempDis3.x = abs(PtsForRoomSquared_sh[k].x - pts[k].x);
				tempDis3.y = abs(PtsForRoomSquared_sh[k].y - pts[k].y);
				tempDis3.z = 0;
				//cout << "tempDis31: " << tempDis3 << endl;
				contour_squared1.push_back(std::make_pair(tempPts3, tempDis3));
			}
#if 1
			for (int k = 0; k < PtsForRoomSquared[minIdx].size(); k++)
			{
				cv::Point2f tempDis;
				tempDis.x = abs(PtsForRoomSquared[minIdx][k].x - pts[k].x);
				tempDis.y = abs(PtsForRoomSquared[minIdx][k].y - pts[k].y);
				ContourSquaredForHoles.push_back(std::make_pair(PtsForRoomSquared[minIdx][k], tempDis));
			}
#endif
#else

			for (int k = 0; k < PtsForRoomSquared[minIdx].size(); k++)
			{
				cv::Point2f tempDis;
				tempDis.x = abs(PtsForRoomSquared[minIdx][k].x - pts0[k].x);
				tempDis.y = abs(PtsForRoomSquared[minIdx][k].y - pts0[k].y);
				RoomContourSquaredJson.push_back(std::make_pair(PtsForRoomSquared[minIdx][k], tempDis));
				ContourSquaredForHoles.push_back(std::make_pair(PtsForRoomSquared[minIdx][k], tempDis));
#ifdef Debug_M
				cout << "x: " << tempDis.x << " y:" << tempDis.y << endl;
#endif
			}
#endif
		}
		insertedPts = pts0;
		if (axis_p_update) {
			MeasureBase::RotatePoint(axis_p.second, Brotation_matrix, insertedPts[axis_p.first]);
		}
		insertedWallList = wall_list;
	}///SQUARE_BY_ROOMCONTOUR

	if ((type == SQUARE_BY_CUBOID) || (type == SQUARE_BY_CONVEXITY))
	{
		std::cout << "[SquareMode] CUBOID/CONVEXITY branch: build cuboid-like contour from matched wall vertices first, then ContourSquare."
			<< std::endl;
#ifdef Debug_RC
		cout << "\n ====================SQUARE_BY_CUBOID||SQUARE_BY_CONVEXITY£¬ ref_wall_contour_id:" << ref_wall_contour_id << endl;
#endif
		for (int k = 0; k < pts.size(); k++)
		{

#ifdef Debug_RC
			cout << "\n\n==========k:" << k << "  " << pts[k] << " " << pts[(k + 1) % round] << endl;
			cout << "roomcontour pts size:" << pts.size() << "  wallList.size(): " << wallList.size() << endl;
#endif
			bool Exit0 = false;
			std::vector<cv::Point3f> WallVer;
			int w = -2;
			if (custom_square) {
				for (auto wall : wallList)
				{
					if (wall == -1)
						continue;
#ifdef Debug_RC
					cout << "Wall verticles: " << mSPlane[wall].vertices.size() << endl;
					cout << mSPlane[wall].vertices << endl;
#endif
					w = wall;
					bool Fidx = false;
					bool Sidx = false;

					for (int i = 0; i < mSPlane[wall].vertices.size(); i++)
					{
						//cout <<wall<<": "<< mSPlane[wall].vertices[i] << " "<<pts[k] << endl;
						if ((abs((int)mSPlane[wall].vertices[i].x - (int)pts[k].x) < 50) &&
							(abs((int)mSPlane[wall].vertices[i].y - (int)pts[k].y) < 50))
						{
							Fidx = true;
						}

						//cout << wall << ": " << mSPlane[wall].vertices[i] << " " << pts[(k + 1) % round] << endl;
						if ((abs((int)mSPlane[wall].vertices[i].x - (int)pts[(k + 1) % round].x) < 50) &&
							(abs((int)mSPlane[wall].vertices[i].y - (int)pts[(k + 1) % round].y) < 50))
						{
							Sidx = true;
						}

						if (Fidx && Sidx)
						{
#ifdef Debug_RC
							cout << "=====Fidx Sidx = true;0;====" << endl;
#endif
							WallVer.push_back(mSPlane[wall].vertices[3]);
							WallVer.push_back(mSPlane[wall].vertices[2]);
							Exit0 = true;
							break;
						}
					}
					if (Exit0) {
						break;
					}
				}
			}
			else {
				for (auto wall : wallList)
				{
					if (wall == -1)
						continue;
#ifdef Debug_RC
					cout << "Wall verticles: " << mSPlane[wall].vertices.size() << endl;
					cout << mSPlane[wall].vertices << endl;
#endif
					w = wall;
					bool Fidx = false;
					bool Sidx = false;
					for (int i = 0; i < mSPlane[wall].vertices.size(); i++)
					{
						if (mSPlane[wall].vertices[i].z < 0) {
							if (((int)mSPlane[wall].vertices[i].x == (int)pts[k].x) &&
								((int)mSPlane[wall].vertices[i].y == (int)pts[k].y))
							{
								Fidx = true;
							}

							if (((int)mSPlane[wall].vertices[i].x == (int)pts[(k + 1) % round].x) &&
								((int)mSPlane[wall].vertices[i].y == (int)pts[(k + 1) % round].y))
							{
								Sidx = true;
							}
						}
						if (Fidx && Sidx)
						{
#ifdef Debug_RC
							cout << "=====Fidx Sidx = true;0;====" << endl;
#endif
							for (int j = 0; j < mSPlane[wall].vertices.size(); j++)
							{
								if (mSPlane[wall].vertices[j].z > 0) {
									WallVer.push_back(mSPlane[wall].vertices[j]);
								}
							}
							Exit0 = true;
							break;
						}
					}
					if (Exit0) {
						break;
					}
				}
			}
			if (WallVer.size() > 0) {
#ifdef Debug_RC
				cout << "WallVer n: " << WallVer.size() << endl;
#endif
				if (norm(WallVer[0] - pts0[k]) < norm(WallVer[1] - pts0[k])) {

					if (abs(WallVer[0].x) < abs(pts0[k].x)) {
						CuboidUpdatedContour[k].x = WallVer[0].x;
					}
					else {
						CuboidUpdatedContour[k].x = pts0[k].x;
					}
					if (abs(WallVer[0].y) < abs(pts0[k].y)) {
						CuboidUpdatedContour[k].y = WallVer[0].y;
					}
					else {
						CuboidUpdatedContour[k].y = pts0[k].y;
					}
					CuboidUpdatedContour[k].z = WallVer[0].z;
				}
				else {
					if (abs(WallVer[1].x) < abs(pts0[k].x)) {
						CuboidUpdatedContour[k].x = WallVer[1].x;
					}
					else {
						CuboidUpdatedContour[k].x = pts0[k].x;
					}
					if (abs(WallVer[1].y) < abs(pts0[k].y)) {
						CuboidUpdatedContour[k].y = WallVer[1].y;
					}
					else {
						CuboidUpdatedContour[k].y = pts0[k].y;
					}
					CuboidUpdatedContour[k].z = WallVer[1].z;
				}

				cv::Point3f nor_squ = { 0.f,0.f,1.f };
				//cout << CuboidUpdatedContour[k] << endl;
				//cout << pts0[k] << endl;
				cv::Point3f nor_ori = { CuboidUpdatedContour[k].x - pts0[k].x,   CuboidUpdatedContour[k].y - pts0[k].y,  CuboidUpdatedContour[k].z - pts0[k].z };
#if 0
				cout << nor_ori << endl;
				float inv_norm = 1.f / std::sqrtf(std::pow(nor_ori.x, 2.f) + std::pow(nor_ori.y, 2.f) + std::pow(nor_ori.z, 2.f));
				nor_ori.x *= inv_norm;
				nor_ori.y *= inv_norm;
				nor_ori.z *= inv_norm;
				cout << nor_ori << endl;
#endif
				walls_squareness[k].first = Util_Math::vec3_angle_deg(nor_squ, nor_ori);
				walls_squareness[k].second = GetAngle(pts0[(k+ pts0.size()-1)% pts0.size()].x, pts0[(k + pts0.size() - 1) % pts0.size()].y, pts0[k].x, pts0[k].y,
					                                 pts0[(k + 1) % pts0.size()].x, pts0[(k + 1) % pts0.size()].y);
				//cout << "walls_squareness[" << k << "].first: " << walls_squareness[k].first << endl;
				//cout << "walls_squareness[" << k << "].second: " << walls_squareness[k].second << endl;
			}
			else {
				CuboidUpdatedContour[k] = pts[k];
				walls_squareness[k].first = 0;
				std::cout << "\n\n Abnormal wall found. " << " wall:" << w << " line: "<<__LINE__<< endl;
			}
		}
		log_info("[ContourSquare]:3");
		CuboidUpdatedContourSquared = ContourSquare(CuboidUpdatedContour, pts1, LAera, ref_wall_contour_id, wall_list);
		if (CuboidUpdatedContour.size() != CuboidUpdatedContourSquared.size())
		{
			std::cout << "##### ==> SQUARE_BY_CUBOID  pts0.size() != CuboidUpdatedContourSquared.size()" << endl;
			for (int k = 0; k < CuboidUpdatedContour.size(); k++)
			{
				cv::Point2f pt_s, tmep;
				pt_s.x = CuboidUpdatedContour[k].x;
				pt_s.y = CuboidUpdatedContour[k].y;
				tmep.x = 0;tmep.x = 0;
				CuboidContourSquaredJson.push_back(std::make_pair(pt_s, tmep));
				ContourSquaredForHoles.push_back(std::make_pair(pt_s, tmep));
				ContourSquaredForConvexity.push_back(std::make_pair(pt_s, tmep));
				ContourSquaredFConvexityUpdated.push_back(std::make_pair(pt_s, tmep));

				cv::Point3f pt_s3, tmep3;
				pt_s3.x = CuboidUpdatedContour[k].x;
				pt_s3.y = CuboidUpdatedContour[k].y;
				pt_s3.z = CuboidUpdatedContour[k].z;
				tmep3.x = 0; tmep3.x = 0; tmep3.z = 0;
				CuboidContourSquaredJson3d.push_back(std::make_pair(pt_s3, tmep3));
			}
		}
		else {
#ifdef DBRT
			std::vector<cv::Point3f> temp;
			std::vector<cv::Point3f> PtsForRoomSquared_sh;
			float mx = 0;
			int md = 0;
			for (int k = 0; k < CuboidUpdatedContourSquared.size(); k++)
			{
				cv::Point3f tt;
				tt.x = CuboidUpdatedContourSquared[k].x;
				tt.y = CuboidUpdatedContourSquared[k].y;
				tt.z = pts0[k].z;
				temp.push_back(tt);
				PtsForRoomSquared_sh.push_back(tt);
				//cout << "squared tt i: " << k << " " << tt << endl;
				if (mx < tt.x) {
					mx = tt.x;	md = k;
				}
			}
			if (type == SQUARE_BY_CUBOID) {
				int rd = PtsForRoomSquared_sh.size();
				if ((CuboidUpdatedContourSquared[(md + rd - 1) % rd].x - CuboidUpdatedContourSquared[(md + 1) % rd].x)*(CuboidUpdatedContourSquared[md].y - CuboidUpdatedContourSquared[(md + 1) % rd].y) -
					(CuboidUpdatedContourSquared[(md + rd - 1) % rd].y - CuboidUpdatedContourSquared[(md + 1) % rd].y)*(CuboidUpdatedContourSquared[md].x - CuboidUpdatedContourSquared[(md + 1) % rd].x) > 0)
				{
					//cout << "===========counterclock." << endl;
					for (int k = 0; k < PtsForRoomSquared_sh.size(); k++)
					{
						if (abs(PtsForRoomSquared_sh[k].x - PtsForRoomSquared_sh[(k + 1) % rd].x) >
							abs(PtsForRoomSquared_sh[k].y - PtsForRoomSquared_sh[(k + 1) % rd].y))
						{
							if (PtsForRoomSquared_sh[k].x < PtsForRoomSquared_sh[(k + 1) % rd].x) {
								PtsForRoomSquared_sh[k].y += shrink;
								PtsForRoomSquared_sh[(k + 1) % rd].y += shrink;
							}
							else {
								PtsForRoomSquared_sh[k].y -= shrink;
								PtsForRoomSquared_sh[(k + 1) % rd].y -= shrink;
							}
						}
						else {
							if (PtsForRoomSquared_sh[k].y < PtsForRoomSquared_sh[(k + 1) % rd].y) {
								PtsForRoomSquared_sh[k].x -= shrink;
								PtsForRoomSquared_sh[(k + 1) % rd].x -= shrink;
							}
							else {
								PtsForRoomSquared_sh[k].x += shrink;
								PtsForRoomSquared_sh[(k + 1) % rd].x += shrink;
							}
						}
					}
				}
				else {
					for (int k = 0; k < PtsForRoomSquared_sh.size(); k++)
					{
						if (abs(PtsForRoomSquared_sh[k].x - PtsForRoomSquared_sh[(k + 1) % rd].x) >
							abs(PtsForRoomSquared_sh[k].y - PtsForRoomSquared_sh[(k + 1) % rd].y))
						{
							if (PtsForRoomSquared_sh[k].x < PtsForRoomSquared_sh[(k + 1) % rd].x) {
								PtsForRoomSquared_sh[k].y -= shrink;
								PtsForRoomSquared_sh[(k + 1) % rd].y -= shrink;
							}
							else {
								PtsForRoomSquared_sh[k].y += shrink;
								PtsForRoomSquared_sh[(k + 1) % rd].y += shrink;
							}
						}
						else {
							if (PtsForRoomSquared_sh[k].y < PtsForRoomSquared_sh[(k + 1) % rd].y) {
								PtsForRoomSquared_sh[k].x += shrink;
								PtsForRoomSquared_sh[(k + 1) % rd].x += shrink;
							}
							else {
								PtsForRoomSquared_sh[k].x -= shrink;
								PtsForRoomSquared_sh[(k + 1) % rd].x -= shrink;
							}
						}
					}
				}
			}
			temp = MathOperation::plane_rot(b_mat_to_y, temp);
			if (type == SQUARE_BY_CUBOID) {
				PtsForRoomSquared_sh = MathOperation::plane_rot(b_mat_to_y, PtsForRoomSquared_sh);
			}
			for (int k = 0; k < pts.size(); k++)
			{
				cv::Point2f tempPts, tempDis;
				tempPts.x = temp[k].x;
				tempPts.y = temp[k].y;
				//cout << "squared rback tempPts i: " << k << " " << tempPts << endl;
				tempDis.x = abs(CuboidUpdatedContourSquared[k].x - CuboidUpdatedContour[k].x);
				tempDis.y = abs(CuboidUpdatedContourSquared[k].y - CuboidUpdatedContour[k].y);
				CuboidContourSquaredJson.push_back(std::make_pair(tempPts, tempDis));
				cv::Point3f  tempDis3;

				//cout << "squared rback tempPts i: " << k << " " << tempPts << endl;
				tempDis3.x = abs(CuboidUpdatedContourSquared[k].x - CuboidUpdatedContour[k].x);
				tempDis3.y = abs(CuboidUpdatedContourSquared[k].y - CuboidUpdatedContour[k].y);
				tempDis3.z = 0;
				CuboidContourSquaredJson3d.push_back(std::make_pair(temp[k], tempDis3));
			}
			if (IsSquareByAxis()) {
				tempbr.push_back(temp[ref_wall_contour_id]);
				tempbr.push_back(temp[(ref_wall_contour_id + 1) % temp.size()]);
			}
			if (type == SQUARE_BY_CUBOID) {
				for (int k = 0; k < PtsForRoomSquared_sh.size(); k++)
				{
					cv::Point3f tempPts3, tempDis3;
					tempPts3.x = PtsForRoomSquared_sh[k].x;
					tempPts3.y = PtsForRoomSquared_sh[k].y;
					tempPts3.z = PtsForRoomSquared_sh[k].z;
					//cout << "squared rback tempPts i: " << k << " " << tempPts << endl;
					tempDis3.x = abs(PtsForRoomSquared_sh[k].x - pts[k].x);
					tempDis3.y = abs(PtsForRoomSquared_sh[k].y - pts[k].y);
					tempDis3.z = 0;
					//cout << "tempDis31: " << tempDis3 << endl;
					contour_squared1.push_back(std::make_pair(tempPts3, tempDis3));
				}
			}
#if 1
			for (int k = 0; k < CuboidUpdatedContourSquared.size(); k++)
			{
				cv::Point2f tempDis;
				tempDis.x = abs(CuboidUpdatedContourSquared[k].x - CuboidUpdatedContour[k].x);
				tempDis.y = abs(CuboidUpdatedContourSquared[k].y - CuboidUpdatedContour[k].y);
				ContourSquaredForHoles.push_back(std::make_pair(CuboidUpdatedContourSquared[k], tempDis));
				ContourSquaredForConvexity.push_back(std::make_pair(CuboidUpdatedContourSquared[k], tempDis));
				ContourSquaredFConvexityUpdated.push_back(std::make_pair(CuboidUpdatedContourSquared[k], tempDis));
			}
#endif
#else

			for (int k = 0; k < CuboidUpdatedContourSquared.size(); k++)
			{
				cv::Point2f tempDis;
				tempDis.x = abs(CuboidUpdatedContourSquared[k].x - CuboidUpdatedContour[k].x);
				tempDis.y = abs(CuboidUpdatedContourSquared[k].y - CuboidUpdatedContour[k].y);
				CuboidContourSquaredJson.push_back(std::make_pair(CuboidUpdatedContourSquared[k], tempDis));
				ContourSquaredForHoles.push_back(std::make_pair(CuboidUpdatedContourSquared[k], tempDis));
				ContourSquaredForConvexity.push_back(std::make_pair(CuboidUpdatedContourSquared[k], tempDis));
				ContourSquaredFConvexityUpdated.push_back(std::make_pair(CuboidUpdatedContourSquared[k], tempDis));
#ifdef Debug_RC
				cout << "x: " << tempDis.x << " y:" << tempDis.y << endl;
#endif
			}
#endif
		}
		insertedPts = pts0;
		if (axis_p_update) {
			MeasureBase::RotatePoint(axis_p.second, Brotation_matrix, insertedPts[axis_p.first]);
		}
		insertedWallList = wall_list;
	}///SQUARE_BY_CUBOID

	squared_defect.resize(wall_list.size());
	filling_vol.resize(wall_list.size());
	square_defect_max.resize(wall_list.size());
	std::vector<std::vector<std::pair<float, cv::Point3f>>> squared_defect_r;
	squared_defect_r.resize(ContourSquaredFConvexityUpdated.size());

	if (SQUARE_BY_CONVEXITY == type)
	{
#ifdef Debug_COV
		cout << "\n\n====================SQUARE_BY_CONVEXITY" << endl;
		cout << pts.size() << " " << ContourSquaredForConvexity.size() << endl;
		cout << "=========ref_wall_contour_id:" << ref_wall_contour_id << endl;
#endif

#ifdef Debug_COV
		for (int j = 0; j < wallList.size(); j++)
		{
			cout << wallList[j] << "  ";
		}
		cout << endl;
#endif
		round = pts.size();
		convPts.resize(wallList.size());
		convPtsF.resize(wallList.size());
		MeasureDefect measure_defect(true);
		float ruler_width; float ruler_height;
		measure_defect.GetRulerSize(ruler_width, ruler_height);
		std::vector<float> filling_vol_opt_temp;
		filling_vol_opt_temp.resize(wallList.size());

		float total_filing = 0;
		for (int j = 0; j < convPtsF.size(); j++)
		{
			convPtsF[j]=false;
		}
		for (int k = 0; k < ContourSquaredFConvexityUpdated.size(); k++)
		{
#ifdef Debug_COV
			cout << "\n==========k:" << k <<"  "<< pts[k]<<" "<< pts[(k + 1) % round]<< " " << pts[(k + 2) % round] <<  endl;
#endif
			bool  nextIsInserted = false;
			bool  skipThis = false;

			for (int x = 0; x < insertMap.size(); x++)
			{
				if (k == insertMap[x] -1) {
					nextIsInserted = true;
					std::cout << "nextIsInserted = true"<< endl;
				}
				if (k == insertMap[x]) {
					skipThis = true;
					std::cout << "skipThis = true" << endl;
				}
			}
			if (skipThis) continue;
			bool Exit0 = false;
			for (int j = 0; j< wallList.size(); j++)
			{
				bool Fidx = false;
				bool Sidx = false;
				if (wallList[j] == -1)
					continue;

#ifdef Debug_COV
				cout << "wallList :" << wallList[j] << endl;
#endif
				for (int i = 0; i < mSPlane[wallList[j]].vertices.size(); i++)
				{
					//cout << "vertices :" << mSPlane[wallList[j]].vertices << endl;
					if (custom_square) {
						{
							//cout << wallList[j] << ": " << mSPlane[wallList[j]].vertices[i] << " " << pts1[k] << endl;
							if ((abs((int)mSPlane[wallList[j]].vertices[i].x - (int)pts1[k].x) < 50) &&
								(abs((int)mSPlane[wallList[j]].vertices[i].y - (int)pts1[k].y) < 50))

							{
								Fidx = true;
								//cout << "Sidx = true;  line:" << __LINE__ << endl;
							}
							if (nextIsInserted) {
								//	cout << wallList[j] << ": " << mSPlane[wallList[j]].vertices[i] << " " << pts1[(k + 2) % round] << endl;
								if ((abs((int)mSPlane[wallList[j]].vertices[i].x - (int)pts1[(k + 2) % round].x) < 50) &&
									(abs((int)mSPlane[wallList[j]].vertices[i].y - (int)pts1[(k + 2) % round].y) < 50))

								{
									Sidx = true;
									//cout << "Sidx = true;  line:" << __LINE__ << endl;
								}
							}
							else {
								//	cout << wallList[j] << ": " << mSPlane[wallList[j]].vertices[i] << " " << pts1[(k + 1) % round] << endl;
								if ((abs((int)mSPlane[wallList[j]].vertices[i].x - (int)pts1[(k + 1) % round].x) < 50) &&
									(abs((int)mSPlane[wallList[j]].vertices[i].y - (int)pts1[(k + 1) % round].y) < 50))

								{
									Sidx = true;
								}
							}
						}
					}
					else {
						if (mSPlane[wallList[j]].vertices[i].z < 0)
						{
							if (((int)mSPlane[wallList[j]].vertices[i].x == (int)pts[k].x) &&
								((int)mSPlane[wallList[j]].vertices[i].y == (int)pts[k].y))
							{
								Fidx = true;
								//cout << "Sidx = true;  line:" << __LINE__ << endl;
							}
							if (nextIsInserted) {

								if (((int)mSPlane[wallList[j]].vertices[i].x == (int)pts[(k + 2) % round].x) &&
									((int)mSPlane[wallList[j]].vertices[i].y == (int)pts[(k + 2) % round].y))
								{
									Sidx = true;
									//cout << "Sidx = true;  line:" << __LINE__ << endl;
								}
							}
							else {
								if (((int)mSPlane[wallList[j]].vertices[i].x == (int)pts[(k + 1) % round].x) &&
									((int)mSPlane[wallList[j]].vertices[i].y == (int)pts[(k + 1) % round].y))
								{
									Sidx = true;
								}
							}
						}
					}
					if (Fidx && Sidx)
					{
						float  filling_volume = 0;
						cv::Point3f p1, p2;
						cv::Point3f p10, p12, p11, p22, pd;
						p1.x = ContourSquaredFConvexityUpdated[(k) % round].first.x;
						p1.y = ContourSquaredFConvexityUpdated[(k) % round].first.y;
						p1.z = p2.z = 0;

						//cout <<"wall:" << wallList[j] << endl;
						//cout <<"original contour: " << insertedPts[(k) % round] << "  " << insertedPts[(k + 1) % round] << endl;
						//cout <<"original squared: "<< CuboidUpdatedContourSquared[(k) % round] << "  " << CuboidUpdatedContourSquared[(k+1) % round] << endl;

						if (nextIsInserted) {
							p2.x = ContourSquaredFConvexityUpdated[(k + 1) % round].first.x;
							p2.y = ContourSquaredFConvexityUpdated[(k + 1) % round].first.y;
						}
						else {
							p2.x = ContourSquaredFConvexityUpdated[(k + 1) % round].first.x;
							p2.y = ContourSquaredFConvexityUpdated[(k + 1) % round].first.y;
						}

#ifdef Debug_COV
						cout << "===COV==Fidx Sidx = true;1;====" << endl;
					//	cout << mSPlane[wallList[j]].vertices[i] << endl;
						cout << " j:" << j << "[wallList[j]: " << wallList[j]  << "  xxxu: " << p1 << " , " << ContourSquaredFConvexityUpdated[(k + 1) % round].first << endl;
#endif


						if (abs(p1.x - p2.x) > abs(p1.y - p2.y))
						{
							//cout << "delt x > delt y" << endl;
#if 1
							std::vector<cv::Point2f> wall_v, hole_v;
							for (int v = 0; v < mSPlane[wallList[j]].vertices.size(); v++)
							{
								wall_v.push_back(cv::Point2f(mSPlane[wallList[j]].vertices[v].x, mSPlane[wallList[j]].vertices[v].z));
							}
							square_defect_max[k].second = ComputePolygonArea(wall_v);
							for (int v = 0; v < mSPlane[wallList[j]].holes.size(); v++)
							{
								for (int h = 0; h < mSPlane[wallList[j]].holes[v].vertice.size(); h++)
								{
									hole_v.push_back(cv::Point2f(mSPlane[wallList[j]].holes[v].vertice[h].x, mSPlane[wallList[j]].holes[v].vertice[h].z));
								}
								square_defect_max[k].second -= ComputePolygonArea(hole_v);
							}
							square_defect_max[k].second /= 1000000000;
#endif
							bool find_wallflatness = false;
							for (int f = 0; f < flateness_defect.size(); f++)
							{
								if ((flateness_defect[f].first == wallList[j]))
								{
									find_wallflatness = true;
#ifdef Debug_COV
									cout << "yA plane_normals:" << plane_normals[wallList[j]] << endl;
#endif
									//sort(flateness_defect[f].second.begin(), flateness_defect[f].second.end(), LessSortY);
									float diffmax = 0;
									float dis = 0;
									for (int m = 0; m < flateness_defect[f].second.size(); m++)
									{
										//if (flateness_defect[f].second[m].first > 0)
										{
#ifdef DBRT
											pd.x = flateness_defect[f].second[m].x;//second.x;
											pd.y = flateness_defect[f].second[m].y;//second.y;
											pd.z = flateness_defect[f].second[m].z;
#else
											pd.x = flateness_defect[f].second[m].second.x;
											pd.y = flateness_defect[f].second[m].second.y;
#endif
#if 1
											if (abs(p1.y) > abs(pd.y)) //need update
											{
												dis = abs(p1.y) - abs(pd.y);
												cv::Point3f pdb;
												MeasureBase::RotatePoint(pd, b_mat_to_y, pdb);
												squared_defect[k].push_back(make_pair(dis, pdb));
												squared_defect_r[k].push_back(make_pair(dis, pd));

												if ((diffmax < dis))
												{
													diffmax = dis;
													convPts[wallList[j]] = flateness_defect[f].second[m];
													convPtsF[wallList[j]] = true;
#ifdef Debug_COV
													//cout << "y axis, diffmax: " << diffmax << endl;
#endif
												}
											}
											else {
											//	cout << "ruler_width: " << ruler_width << " ruler_height:" << ruler_height << endl;
												if ((abs(flateness_defect[f].second[m].x) - abs(p1.x)) != 0)
													filling_volume += (ruler_width * ruler_height*(abs(pd.y) - abs(p1.y)));
											}
#else
											float y1 = Pa.x*flateness_defect[f].second[m].second.x + Pa.y;
											float diff = 0;
											cv::Point2f P0, P1;
											P0.x = pts[k].x; P0.y = pts[k].y;
											P1.x = pts[(k + 1) % round].x; P1.y = pts[(k + 1) % round].y;
											double theta = getLinesAngle(ContourSquaredForConvexity[k].first,
												ContourSquaredForConvexity[(k + 1) % round].first, P0, P1);
											if ((theta <= 90) && (theta >= 0)) {// (180 * ans) / M_PI;
												diff = flateness_defect[f].second[m].first*cos(theta*M_PI/180.0) - abs(abs(y1) - abs(flateness_defect[f].second[m].second.y));
												//diff = flateness_defect[f].second[m].first - abs(abs(y1) - abs(flateness_defect[f].second[m].second.y));
											}else{
												cout << "y !!!!!!!!! theta: " << theta << endl;
												diff = flateness_defect[f].second[m].first - abs(abs(y1) - abs(flateness_defect[f].second[m].second.y));
											}
											if ((diff > 0) && (diffmax < diff))
											{
												diffmax = diff;
#ifdef Debug_COV
												cout << "y theta: " << theta << endl;
												cout << "===cos(theta*M_PI/180.0): " << cos(theta*M_PI / 180.0) << endl;
												cout << "diffmax: " << diffmax << endl;
#endif
											}
#endif
										}
									}
									square_defect_max[k].first = diffmax;
#if 1
									if (p1.x  >  p2.x)
									{
#ifdef Debug_COV
										cout << " ======== y + diffmax " << diffmax << endl;
#endif
										ContourSquaredFConvexityUpdated[(k) % round].first.y += diffmax;
										ContourSquaredFConvexityUpdated[(k + 1) % round].first.y += diffmax;

									}
									else {
#ifdef Debug_COV
										cout << " ======== y - diffmax " << diffmax << endl;
#endif
										ContourSquaredFConvexityUpdated[(k) % round].first.y -= diffmax;
										ContourSquaredFConvexityUpdated[(k + 1) % round].first.y -= diffmax;
									}
#else
									int sign = ContourSquaredFConvexityUpdated[k].first.y / abs(ContourSquaredFConvexityUpdated[k].first.y);
#ifdef Debug_COV
									cout << "y sign:" << sign << endl;
#endif
									if (abs(ContourSquaredFConvexityUpdated[k].first.y) > diffmax) {
										ContourSquaredFConvexityUpdated[k].first.y = sign* (abs(ContourSquaredFConvexityUpdated[k].first.y) - diffmax);
									}
									else {
										ContourSquaredFConvexityUpdated[k].first.y = -sign* (abs(ContourSquaredFConvexityUpdated[k].first.y) - diffmax);
									}
									ContourSquaredFConvexityUpdated[(k + 1) % round].first.y = sign* (abs(ContourSquaredFConvexityUpdated[(k + 1) % round].first.y) - diffmax);
#ifdef Debug_COV
									cout << k << "," << (k + 1) % round << " y-diffmax " << diffmax << endl;
#endif
#endif
								}
							}
							if (!find_wallflatness) {
								std::cout << " ######### Do not Find wall flatness!" << endl;
							}
						}
						else {
#if 1
						std::vector<cv::Point2f> wall_v, hole_v;
						for (int v = 0; v < mSPlane[wallList[j]].vertices.size(); v++)
						{
							wall_v.push_back(cv::Point2f(mSPlane[wallList[j]].vertices[v].y, mSPlane[wallList[j]].vertices[v].z));
						}
						square_defect_max[k].second = ComputePolygonArea(wall_v);
						for (int v = 0; v < mSPlane[wallList[j]].holes.size(); v++)
						{
							for (int h = 0; h < mSPlane[wallList[j]].holes[v].vertice.size(); h++)
							{
								hole_v.push_back(cv::Point2f(mSPlane[wallList[j]].holes[v].vertice[h].y, mSPlane[wallList[j]].holes[v].vertice[h].z));
							}
							square_defect_max[k].second -= ComputePolygonArea(hole_v);
						}
						square_defect_max[k].second /= 1000000000;
#endif
							//cout << "delt x > delt y" << endl;
							bool find_wallflatness = false;
							for (int f = 0; f < flateness_defect.size(); f++)
							{
								if ((flateness_defect[f].first == wallList[j]))
								{
#ifdef Debug_COV
									cout << "xA plane_normals::" << plane_normals[wallList[j]] << endl;
#endif
									find_wallflatness = true;
								//	sort(flateness_defect[f].second.begin(), flateness_defect[f].second.end(), LessSortX);


									float diffmax = 0;
									float dis = 0;
									for (int m = 0; m < flateness_defect[f].second.size(); m++)
									{
								//		if (flateness_defect[f].second[m].first > 0)
										{
#if 1
#ifdef DBRT
											if (abs(p1.x) > abs(flateness_defect[f].second[m]./*second.*/x) )
											{
												dis = abs(p1.x) - abs(flateness_defect[f].second[m]./*second.*/x);
											    cv::Point3f pdb;
												MeasureBase::RotatePoint(flateness_defect[f].second[m], b_mat_to_y, pdb);
												squared_defect[k].push_back(make_pair(dis, flateness_defect[f].second[m]));
												squared_defect_r[k].push_back(make_pair(dis, pd));

#else
											if (abs(p1.x) > abs(flateness_defect[f].second[m].second.x))
											{
												float dis = abs(p1.x) - abs(flateness_defect[f].second[m].second.x);
#endif
												if ((diffmax < dis))
												{
													diffmax = dis;
													convPts[wallList[j]] = flateness_defect[f].second[m];
													convPtsF[wallList[j]] = true;
#ifdef Debug_COV
													//cout << " x axis, diffmax: " << diffmax << endl;
#endif
												}
											}
											else {

												//cout << "ruler_width: " << ruler_width << " ruler_height:" << ruler_height << endl;
												if((abs(flateness_defect[f].second[m].x) - abs(p1.x)) !=0)
													filling_volume += (ruler_width * ruler_height*(abs(flateness_defect[f].second[m].x) - abs(p1.x)));
											}
#else
											float x1 = 0;
											if ((Pa.x == 0) && (Pa.y == 0)) {

												x1 = ContourSquaredForConvexity[k].first.x;
											}
											else {
												x1 = (flateness_defect[f].second[m].second.y - Pa.y) / Pa.x;
											}
											float diff = 0;
											cv::Point2f P0, P1;
											P0.x = pts[k].x; P0.y = pts[k].y;
											P1.x = pts[(k + 1) % round].x; P1.y = pts[(k + 1) % round].y;
											double theta = getLinesAngle(ContourSquaredForConvexity[k].first,
												ContourSquaredForConvexity[(k + 1) % round].first,P0, P1);
											if ((theta <= 90) && (theta >= 0)) {

												diff = flateness_defect[f].second[m].first * cos(theta * M_PI/180.0) - abs(abs(x1) - abs(flateness_defect[f].second[m].second.x));
												//diff = flateness_defect[f].second[m].first  - abs(abs(x1) - abs(flateness_defect[f].second[m].second.x));
											}
											else {
												cout << "x !!!!!!!!! theta: " << theta << endl;
												diff = flateness_defect[f].second[m].first - abs(abs(x1) - abs(flateness_defect[f].second[m].second.x));

											}
											if ((diff > 0) && (diffmax < diff))
											{
												diffmax = diff;
#ifdef Debug_COV
												cout << "y theta: " << theta << endl;
												cout << "==cos(theta*M_PI/180.0): " << cos(theta*M_PI / 180.0) << endl;
												cout << "diffmax: " << diffmax << endl;
#endif
											}
#endif
										}
									}
									square_defect_max[k].first = diffmax;
#if 1
									if (p1.y > p2.y)
									{
#ifdef Debug_COV
										cout << " ======== x - diffmax " << diffmax << endl;
#endif
										ContourSquaredFConvexityUpdated[(k) % round].first.x -= diffmax;
										ContourSquaredFConvexityUpdated[(k + 1) % round].first.x -= diffmax;
									}
									else {
#ifdef Debug_COV
										cout << " ======== x + diffmax " << diffmax << endl;
#endif
										ContourSquaredFConvexityUpdated[(k) % round].first.x += diffmax;
										ContourSquaredFConvexityUpdated[(k + 1) % round].first.x += diffmax;

									}
#else
									int sign = ContourSquaredFConvexityUpdated[k].first.x / abs(ContourSquaredFConvexityUpdated[k].first.x);
									//cout << "x sign:" << sign <<endl;
									if (abs(ContourSquaredFConvexityUpdated[k].first.x) > diffmax) {
										ContourSquaredFConvexityUpdated[k].first.x = sign* (abs(ContourSquaredFConvexityUpdated[k].first.x) - diffmax);
									}
									else {
										//cout << "Cross cord sign:" << sign << endl;
										ContourSquaredFConvexityUpdated[k].first.x = - sign* (abs(ContourSquaredFConvexityUpdated[k].first.x) - diffmax);
									}
									ContourSquaredFConvexityUpdated[(k + 1) % round].first.x= sign* (abs(ContourSquaredFConvexityUpdated[(k + 1) % round].first.x) - diffmax);
#ifdef Debug_COV
									cout << k <<","<< (k + 1) % round << " x-diffmax " << diffmax << endl;
#endif
#endif
								}
							}
							if (! find_wallflatness) {
								std::cout << " ######### Do not Find wall flatness!" << endl;
							}
						}
#ifdef Debug_COV
						cout << "done: " << ContourSquaredFConvexityUpdated[(k) % round].first
							<< " , " << ContourSquaredFConvexityUpdated[(k + 1) % round].first << endl;
#endif
						std::cout << "======wallList :" << wallList[j] << " filling_volume:" << filling_volume/1000000000 ;
						total_filing += (filling_volume / 1000000000);

						filling_vol[k].first = filling_volume / 1000000000;
						float temp_v = 0;
						for (int p = 0; p < squared_defect_r[k].size(); p++)
						{
							temp_v += squared_defect_r[k][p].first * ruler_width * ruler_height;
						}
						filling_vol_opt_temp[k] = filling_vol[k].first + square_defect_max[k].first * square_defect_max[k].second;// / 1000000000;

						filling_vol[k].second = filling_vol_opt_temp[k] - temp_v / 1000000000;
						Exit0 = true;
						break;
					}
				}
				if (Exit0) {
					break;
				}
			}
			std::cout << " k : " << k << " defect_max:" << square_defect_max[k].first <<" area:"<<square_defect_max[k].second << endl;
		}//loop

		convPts = MathOperation::plane_rot(b_mat_to_y, convPts);
		std::cout << "total_filing: " << total_filing << endl;

		for (int i = 0; i < wall_list.size(); i++) {
			std::cout << "wall_list[" << i << "]: " << wall_list[i] << endl;
		}

		for (int i = 0; i < ContourSquaredFConvexityUpdated.size(); i++)
		{
		}


	std::vector<float> wall_width_in;
	int rnd = ContourSquaredFConvexityUpdated.size();
	wall_width_in.resize(rnd);
     /*	squared wall width : 3490.98
		squared wall width : 3712.63
		squared wall width : 3490.98
		squared wall width : 3712.63*/
	std::vector<bool> defect_state;
	defect_state.resize(rnd);
	std::vector<float> convexity_mod;
	convexity_mod.resize(rnd);
	squared_defect_info.resize(rnd);
	filling_vol_opt.resize(rnd);

	for (int i = 0; i < defect_state.size(); i++) {
		defect_state[i] = true;
	}
	std::vector<cv::Point2f> ContourSquaredFConvexityUpdatedNew;
	ContourSquaredFConvexityUpdatedNew.resize(rnd);
	for (int i = 0; i < rnd; i++)
	{
		ContourSquaredFConvexityUpdatedNew[i] = ContourSquaredFConvexityUpdated[i].first;
		std::cout << "ContourSquaredFConvexityUpdatedNew[" << i << "]0: " << ContourSquaredFConvexityUpdatedNew[i] << endl;
	}
	for (int i = 0; i < rnd; i++)
	{
		std::cout << "squared wall width0: " << DisOf2Points2f(ContourSquaredFConvexityUpdatedNew[i], ContourSquaredFConvexityUpdatedNew[(i + 1) % rnd]) << endl;
	}

	wall_width_input.resize(wallList.size());

	wall_width_input[0] = std::make_pair(3498, 1);
	wall_width_input[1] = std::make_pair(3710, 2);
	wall_width_input[2] = std::make_pair(3498, 3);
	wall_width_input[3] = std::make_pair(3710, 0);

	std::vector<std::pair<float, int>> convexity_diff; // 凸点找方后墙宽大于设计值的差值， 当前墙面在contour中的索引
	wall_width_info.resize(wallList.size());
	std::cout << "wallList.size():" << wallList.size() << " wall_list.size():" << wall_list.size() << endl;

	// diss 的wall id 怎么确定？
	for (int j = 0; j < wall_width_input.size(); j++)
	{
		for (int w = 0; w < wall_list.size(); w++)
		{
			if (wall_list[w] == wall_width_input[j].second)
			{
				float diss = DisOf2Points2f(ContourSquaredFConvexityUpdatedNew[w], ContourSquaredFConvexityUpdatedNew[(w + 1) % rnd]);
				convexity_diff.push_back(std::make_pair(diss - wall_width_input[j].first, w));
				wall_width_in[w] = wall_width_input[j].first;
				std::cout << "convexity_diff, first: "<< diss - wall_width_input[j].first << " second:" << w << " squared wall width1:"<< diss << endl;
			}
		}
	}
	sort(convexity_diff.begin(), convexity_diff.end(), compln);
	for (int i = 0; i < convexity_diff.size(); i++)
	{
		std::cout << "sorted convexity_diff, first: " << convexity_diff[i].first << " second:" << i << endl;
	}
#if 0
	for (int cidx = 0; cidx < convexity_diff.size(); cidx++) //
	{ //按误差大小顺序遍历所有墙面
		int i = convexity_diff[cidx].second; //wall_list
		float square2_dis = DisOf2Points2f(ContourSquaredFConvexityUpdatedNew[i], ContourSquaredFConvexityUpdatedNew[(i +1) % rnd]);
		wall_width_info[i].first = square2_dis;
		if (convexity_diff[cidx].first >= 0)
		{ //凸点找方后墙宽大于设计值
			defect_state[(i - 1 + rnd) % rnd] = false;
			defect_state[(i + 1) % rnd] = false;
			convexity_mod[(i - 1 + rnd) % rnd] =0;
			convexity_mod[(i + 1) % rnd] = 0;
			cout <<"cidx:"<< cidx << " squared wall width > wall_width_input" << endl;
			continue;
		}
		else
		{//凸点找方后墙宽小于设计值
			if (abs(ContourSquaredFConvexityUpdatedNew[i].x - ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].x) > abs(ContourSquaredFConvexityUpdatedNew[i ].y - ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].y))
			{
				if (ContourSquaredFConvexityUpdatedNew[i].x > ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].x)
				{//d
					if ((ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].y > ContourSquaredFConvexityUpdatedNew[i ].y) && (ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].y > ContourSquaredFConvexityUpdatedNew[i ].y))
					{//d1
						if (defect_state[(i - 1 + rnd) % rnd] && defect_state[(i + 1) % rnd])
						{ //两面墙都未处理过
							if ((square_defect_max[(i - 1 + rnd) % rnd].first + square_defect_max[(i + 1) % rnd].first + square2_dis) < wall_width_in[i])
							{ // 凸点找方前墙宽小于设计值，去除两面墙所有凸点
								cout << "d d1 b2   line:" << __LINE__ << endl;
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].x += square_defect_max[(i - 1 + rnd) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[i ].x += square_defect_max[(i - 1 + rnd) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].x -= square_defect_max[(i + 1) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].x -= square_defect_max[(i + 1) % rnd].first;
								convexity_mod[(i - 1 + rnd) % rnd] = square_defect_max[(i - 1 + rnd) % rnd].first;
								convexity_mod[(i + 1 ) % rnd] = square_defect_max[(i + 1) % rnd].first;
							}
							if ((square_defect_max[(i - 1 + rnd) % rnd].first + square_defect_max[(i + 1) % rnd].first + square2_dis) > wall_width_in[i])
							{// 凸点找方前墙宽大于设计值，根据凸点比例分配去除的凸点量
								cout << "d d1 b2  line:" << __LINE__ << endl;
								float diff =  - square2_dis + wall_width_in[i];
								float denominator = square_defect_max[(i - 1 + rnd) % rnd].first + square_defect_max[(i + 1 + rnd) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].x += (square_defect_max[(i - 1 + rnd) % rnd].first / denominator * diff);
								ContourSquaredFConvexityUpdatedNew[i ].x += (square_defect_max[(i - 1 + rnd) % rnd].first / denominator * diff);
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].x -= (square_defect_max[(i + 1 + rnd) % rnd].first / denominator * diff);
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].x -= (square_defect_max[(i + 1 + rnd) % rnd].first / denominator * diff);
								convexity_mod[(i - 1 + rnd) % rnd] = (square_defect_max[(i - 1 + rnd) % rnd].first / denominator * diff);
								convexity_mod[(i + 1) % rnd] = (square_defect_max[(i - 1 + rnd) % rnd].first / denominator * diff);
							}
							defect_state[(i - 1 + rnd) % rnd] = false;
							defect_state[(i + 1) % rnd] = false;
						}
						if (defect_state[(i - 1 + rnd) % rnd] && !defect_state[(i + 1) % rnd])
						{//上一面墙还未处理过凸点
							if ((square_defect_max[(i - 1 + rnd) % rnd].first + square2_dis) < wall_width_in[i])
							{ // 上一面墙去除凸点墙宽小于设计值，去除上一面墙所有凸点
								cout << "d d1 s1  line:" << __LINE__ << endl;
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].x += (square_defect_max[(i - 1 + rnd) % rnd].first);
								ContourSquaredFConvexityUpdatedNew[i ].x += (square_defect_max[(i - 1 + rnd) % rnd].first);
								convexity_mod[(i - 1 + rnd) % rnd] = (square_defect_max[(i - 1 + rnd) % rnd].first);
							}
							if ((square_defect_max[(i - 1 + rnd) % rnd].first + square2_dis) > wall_width_in[i])
							{// 上一面墙去除凸点墙宽大于设计值，去除多余凸点
								cout << "d d1 s1  line:" << __LINE__ << endl;
								float diff =  - square2_dis + wall_width_in[i];
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].x += diff;
								ContourSquaredFConvexityUpdatedNew[i ].x += diff;
								convexity_mod[(i - 1 + rnd) % rnd] = diff;
							}
							defect_state[(i - 1 + rnd) % rnd] = false;
						}
						if (!defect_state[(i - 1 + rnd) % rnd] && defect_state[(i + 1) % rnd])
						{ //下一面墙还未处理过凸点
							if ((square_defect_max[(i + 1) % rnd].first + square2_dis) < wall_width_in[i])
							{ // 凸点找方前墙宽小于设计值，去除两面墙所有凸点
								cout << "d d1 x1  line:" << __LINE__ << endl;
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].x -= square_defect_max[(i + 1 + rnd) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].x -= square_defect_max[(i + 1 + rnd) % rnd].first;
								convexity_mod[(i + 1) % rnd] = square_defect_max[(i + 1 + rnd) % rnd].first;
							}
							if ((square_defect_max[(i + 1) % rnd].first + square2_dis) > wall_width_in[i])
							{// 凸点找方前墙宽大于设计值，根据凸点比例分配去除的凸点量
								cout << "d d1 x1  line:" << __LINE__ << endl;
								float diff =  -square2_dis + wall_width_in[i];
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].x -= diff;
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].x -= diff;
								convexity_mod[(i + 1) % rnd] = diff;
							}
							defect_state[(i + 1) % rnd] = false;
						}
					}
					if ((ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].y > ContourSquaredFConvexityUpdatedNew[i ].y) && (ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].y < ContourSquaredFConvexityUpdatedNew[i ].y))
					{//d2
						if (defect_state[(i - 1 + rnd) % rnd] && defect_state[(i + 1) % rnd])
						{//两面墙都未处理过
							if ((square_defect_max[(i - 1 + rnd) % rnd].first + square2_dis) < wall_width_in[i])
							{ // 凸点找方前墙宽小于设计值，去除两面墙所有凸点
								cout << "d d2 b1  line:" << __LINE__ << endl;
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].x += square_defect_max[(i - 1 + rnd) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[i ].x += square_defect_max[(i - 1 + rnd) % rnd].first;
								convexity_mod[(i - 1 + rnd) % rnd] = square_defect_max[(i - 1 + rnd) % rnd].first;
								convexity_mod[(i + 1) % rnd] = 0;
							}
							if ((square_defect_max[(i - 1 + rnd) % rnd].first + square2_dis) > wall_width_in[i])
							{// 凸点找方前墙宽大于设计值，根据凸点比例分配去除的凸点量
								cout << "d d2 b1  line:" << __LINE__ << endl;
								float difft = square_defect_max[(i - 1 + rnd) % rnd].first + square2_dis - wall_width_in[i];
								float difft0 = square_defect_max[(i - 1 + rnd) % rnd].first / square_defect_max[(i - 1 + rnd) % rnd].first + square_defect_max[(i + 1 + rnd) % rnd].first * difft;
								float difft1 = square_defect_max[(i + 1 + rnd) % rnd].first / square_defect_max[(i - 1 + rnd) % rnd].first + square_defect_max[(i + 1 + rnd) % rnd].first * difft;
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].x += (square_defect_max[(i - 1 + rnd) % rnd].first - difft0);
								ContourSquaredFConvexityUpdatedNew[i ].x += (square_defect_max[(i - 1 + rnd) % rnd].first - difft0);
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].x += (square_defect_max[(i + 1 + rnd) % rnd].first - difft1);
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].x += (square_defect_max[(i + 1 + rnd) % rnd].first - difft1);
								convexity_mod[(i - 1 + rnd) % rnd] = square_defect_max[(i - 1 + rnd) % rnd].first - difft0;
								convexity_mod[(i + 1) % rnd] = square_defect_max[(i + 1 + rnd) % rnd].first - difft1;
							}
							defect_state[(i - 1 + rnd) % rnd] = false;
							defect_state[(i + 1) % rnd] = false;
						}
						if ( defect_state[(i - 1 + rnd) % rnd] && !(defect_state[(i + 1) % rnd]))
						{//上一面墙还未处理过凸点
							if ((square_defect_max[(i - 1 + rnd) % rnd].first + square2_dis) < wall_width_in[i])
							{ // 凸点找方前墙宽小于设计值，去除上一面墙所有凸点
								cout << "d d2 s1  line:" << __LINE__ << endl;
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].x += square_defect_max[(i - 1 + rnd) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[i ].x += square_defect_max[(i - 1 + rnd) % rnd].first;
								convexity_mod[(i - 1 + rnd) % rnd] = square_defect_max[(i - 1 + rnd) % rnd].first;
							}
							if ((square_defect_max[(i - 1 + rnd) % rnd].first + square2_dis) > wall_width_in[i])
							{// 凸点找方前墙宽大于设计值，根据凸点比例分配去除的凸点量
								cout << "d d2 s1  line:" << __LINE__ << endl;
								float diff = -square2_dis + wall_width_in[i];
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].x += diff;
								ContourSquaredFConvexityUpdatedNew[i ].x += diff;
								convexity_mod[(i - 1 + rnd) % rnd] = diff;
							}
							defect_state[(i - 1 + rnd) % rnd] = false;
						}
						if (!(defect_state[(i - 1 + rnd) % rnd]) && defect_state[(i + 1) % rnd])
						{ //下一面墙还未处理过凸点
							cout << "d d2 x1  line:" << __LINE__ << endl;
							convexity_mod[(i +1) % rnd] = 0;
							defect_state[(i + 1) % rnd] = false;
						}
					}
					if ((ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].y < ContourSquaredFConvexityUpdatedNew[i ].y) && (ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].y > ContourSquaredFConvexityUpdatedNew[i ].y))
					{//d3
						if (defect_state[(i - 1 + rnd) % rnd] && defect_state[(i + 1) % rnd])
						{//两面墙都未处理过
							if ((square_defect_max[(i + 1) % rnd].first + square2_dis) < wall_width_in[i])
							{ // 凸点找方前墙宽小于设计值，去除后一面墙所有凸点
								cout << "d d3 b2  line:" << __LINE__ << endl;
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].x -= square_defect_max[(i + 1) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].x -= square_defect_max[(i + 1) % rnd].first;
								convexity_mod[(i - 1 + rnd) % rnd] = 0;
								convexity_mod[(i + 1) % rnd] = square_defect_max[(i + 1) % rnd].first;
							}
							if ((square_defect_max[(i + 1) % rnd].first + square2_dis) > wall_width_in[i])
							{// 凸点找方前墙宽大于设计值，根据凸点比例分配去除的凸点量
								cout << "d d3 b2  line:" << __LINE__ << endl;
								float difft = square_defect_max[(i + 1 + rnd) % rnd].first + square2_dis - wall_width_in[i];
								float difft0 = square_defect_max[(i - 1 + rnd) % rnd].first / square_defect_max[(i - 1 + rnd) % rnd].first + square_defect_max[(i + 1 + rnd) % rnd].first * difft;
								float difft1 = square_defect_max[(i + 1 + rnd) % rnd].first / square_defect_max[(i - 1 + rnd) % rnd].first + square_defect_max[(i + 1 + rnd) % rnd].first * difft;
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].x -= (square_defect_max[(i - 1 + rnd) % rnd].first - difft0);
								ContourSquaredFConvexityUpdatedNew[i ].x -= (square_defect_max[(i - 1 + rnd) % rnd].first - difft0);
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].x -= (square_defect_max[(i + 1 + rnd) % rnd].first - difft1);
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].x -= (square_defect_max[(i + 1 + rnd) % rnd].first - difft1);
								convexity_mod[(i - 1 + rnd) % rnd] = square_defect_max[(i - 1 + rnd) % rnd].first - difft0;
								convexity_mod[(i + 1) % rnd] = square_defect_max[(i + 1 + rnd) % rnd].first - difft1;
							}
							defect_state[(i - 1 + rnd) % rnd] = false;
							defect_state[(i + 1) % rnd] = false;
						}
						if ( defect_state[(i - 1 + rnd) % rnd] && !(defect_state[(i + 1) % rnd]))
						{//上一面墙还未处理过凸点
							cout << "d d3 s1  line:" << __LINE__ << endl;
							convexity_mod[(i - 1 + rnd) % rnd] = 0;
							defect_state[(i - 1 + rnd) % rnd] = false;
						}
						if (!defect_state[(i - 1 + rnd) % rnd] && defect_state[(i + 1) % rnd])
						{ //下一面墙还未处理过凸点
							if ((square_defect_max[(i + 1) % rnd].first + square2_dis) < wall_width_in[i])
							{
								cout << "d d3 x1  line:" << __LINE__ << endl;
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].x -= square_defect_max[(i + 1) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].x -= square_defect_max[(i + 1) % rnd].first;
								convexity_mod[(i + 1) % rnd] = square_defect_max[(i + 1) % rnd].first;
							}
							if ((square_defect_max[(i + 1) % rnd].first + square2_dis) > wall_width_in[i])
							{
								cout << "d d3 x1  line:" << __LINE__ << endl;
								float diff = - square2_dis + wall_width_in[i];
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].x -= diff;
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].x -= diff;
								convexity_mod[(i + 1) % rnd] = diff;
							}
							defect_state[(i + 1) % rnd] = false;
						}
					}
				}
				else { // u
					if ((ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].y < ContourSquaredFConvexityUpdatedNew[i ].y) && (ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].y < ContourSquaredFConvexityUpdatedNew[i ].y))
					{// u1
						if (defect_state[(i - 1 + rnd) % rnd] && defect_state[(i + 1) % rnd])
						{ //两面墙都未处理过
							if ((square_defect_max[(i - 1 + rnd) % rnd].first + square_defect_max[(i + 1) % rnd].first + square2_dis) < wall_width_in[i])
							{ // 凸点找方前墙宽小于设计值，去除两面墙所有凸点
								cout << "u u1 b2  line:" << __LINE__ << endl;
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].x -= square_defect_max[(i - 1 + rnd) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[i ].x -= square_defect_max[(i - 1 + rnd) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].x += square_defect_max[(i + 1 + rnd) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].x += square_defect_max[(i + 1 + rnd) % rnd].first;
								convexity_mod[(i - 1 + rnd) % rnd] = square_defect_max[(i - 1 + rnd) % rnd].first;
								convexity_mod[(i + 1) % rnd] = square_defect_max[(i + 1 + rnd) % rnd].first;
							}
							if ((square_defect_max[(i - 1 + rnd) % rnd].first + square_defect_max[(i + 1) % rnd].first + square2_dis) > wall_width_in[i])
							{// 凸点找方前墙宽大于设计值，根据凸点比例分配去除的凸点量
								cout << "u u1 b2  line:" << __LINE__ << endl;
								float diff = -square2_dis + wall_width_in[i];
								float denominator = square_defect_max[(i - 1 + rnd) % rnd].first + square_defect_max[(i + 1 + rnd) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].x -= (square_defect_max[(i - 1 + rnd) % rnd].first / denominator * diff);
								ContourSquaredFConvexityUpdatedNew[i ].x -= (square_defect_max[(i - 1 + rnd) % rnd].first / denominator * diff);
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].x += (square_defect_max[(i + 1 + rnd) % rnd].first / denominator * diff);
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].x += (square_defect_max[(i + 1 + rnd) % rnd].first / denominator * diff);
								convexity_mod[(i - 1 + rnd) % rnd] = square_defect_max[(i - 1 + rnd) % rnd].first / denominator * diff;
								convexity_mod[(i + 1) % rnd] = square_defect_max[(i + 1 + rnd) % rnd].first / denominator * diff;
							}
							defect_state[(i - 1 + rnd) % rnd] = false;
							defect_state[(i + 1) % rnd] = false;
						}
						if (defect_state[(i - 1 + rnd) % rnd] && !defect_state[(i + 1) % rnd])
						{//上一面墙还未处理过凸点
							if ((square_defect_max[(i - 1 + rnd) % rnd].first + square2_dis) < wall_width_in[i])
							{ // 上一面墙去除凸点墙宽小于设计值，去除上一面墙所有凸点
								cout << "u u1 s1  line:" << __LINE__ << endl;
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].x -= square_defect_max[(i - 1 + rnd) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[i ].x -= square_defect_max[(i - 1 + rnd) % rnd].first;
								convexity_mod[(i - 1 + rnd) % rnd] = square_defect_max[(i - 1 + rnd) % rnd].first;
							}
							if ((square_defect_max[(i - 1 + rnd) % rnd].first + square2_dis) > wall_width_in[i])
							{// 上一面墙去除凸点墙宽大于设计值，去除多余凸点
								cout << "u u1 s1  line:" << __LINE__ << endl;
								float diff = -square2_dis + wall_width_in[i];
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].x -= diff;
								ContourSquaredFConvexityUpdatedNew[i ].x -= diff;
								convexity_mod[(i - 1 + rnd) % rnd] = diff;
							}
							defect_state[(i - 1 + rnd) % rnd] = false;
						}
						if (!defect_state[(i - 1 + rnd) % rnd] && defect_state[(i + 1) % rnd])
						{ //下一面墙还未处理过凸点
							if ((square_defect_max[(i + 1) % rnd].first + square2_dis) < wall_width_in[i])
							{ // 凸点找方前墙宽小于设计值，去除两面墙所有凸点
								cout << "u u1 x1  line:" << __LINE__ << endl;
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].x += square_defect_max[(i + 1 + rnd) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].x += square_defect_max[(i + 1 + rnd) % rnd].first;
								convexity_mod[(i + 1) % rnd] = square_defect_max[(i + 1 + rnd) % rnd].first;
							}
							if ((square_defect_max[(i + 1) % rnd].first + square2_dis) > wall_width_in[i])
							{// 凸点找方前墙宽大于设计值，根据凸点比例分配去除的凸点量
								cout << "u u1 s1  line:" << __LINE__ << endl;
								float diff = -square2_dis + wall_width_in[i];
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].x += diff;
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].x += diff;
								convexity_mod[(i + 1) % rnd] = diff;
							}
							defect_state[(i + 1) % rnd] = false;
						}
					}
					if ((ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].y < ContourSquaredFConvexityUpdatedNew[i ].y) && (ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].y > ContourSquaredFConvexityUpdatedNew[i ].y))
					{// u2
						if (defect_state[(i - 1 + rnd) % rnd] && defect_state[(i + 1) % rnd])
						{//两面墙都未处理过
							if ((square_defect_max[(i - 1 + rnd) % rnd].first + square2_dis) < wall_width_in[i])
							{ // 凸点找方前墙宽小于设计值，去除两面墙所有凸点
								cout << "u u2 b2  line:" << __LINE__ << endl;
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].x -= square_defect_max[(i - 1 + rnd) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[i ].x -= square_defect_max[(i - 1 + rnd) % rnd].first;
								convexity_mod[(i - 1 + rnd) % rnd] = square_defect_max[(i - 1 + rnd) % rnd].first;
								convexity_mod[(i + 1) % rnd] = 0;
							}
							if ((square_defect_max[(i - 1 + rnd) % rnd].first + square2_dis) > wall_width_in[i])
							{// 凸点找方前墙宽大于设计值，根据凸点比例分配去除的凸点量
								cout << "u u2 b2  line:" << __LINE__ << endl;
								float difft = square_defect_max[(i - 1 + rnd) % rnd].first + square2_dis - wall_width_in[i];
								float difft0 = square_defect_max[(i - 1 + rnd) % rnd].first / square_defect_max[(i - 1 + rnd) % rnd].first + square_defect_max[(i + 1 + rnd) % rnd].first * difft;
								float difft1 = square_defect_max[(i + 1 + rnd) % rnd].first / square_defect_max[(i - 1 + rnd) % rnd].first + square_defect_max[(i + 1 + rnd) % rnd].first * difft;
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].x -= (square_defect_max[(i - 1 + rnd) % rnd].first - difft0);
								ContourSquaredFConvexityUpdatedNew[i ].x -= (square_defect_max[(i - 1 + rnd) % rnd].first - difft0);
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].x -= (square_defect_max[(i + 1 + rnd) % rnd].first - difft1);
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].x -= (square_defect_max[(i + 1 + rnd) % rnd].first - difft1);
								convexity_mod[(i - 1 + rnd) % rnd] = square_defect_max[(i - 1 + rnd) % rnd].first - difft0;
								convexity_mod[(i + 1) % rnd] = square_defect_max[(i + 1 + rnd) % rnd].first - difft1;
							}
							defect_state[(i - 1 + rnd) % rnd] = false;
							defect_state[(i + 1) % rnd] = false;
						}
						if (defect_state[(i - 1 + rnd) % rnd] && !(defect_state[(i + 1) % rnd]))
						{//上一面墙还未处理过凸点
							if ((square_defect_max[(i - 1 + rnd) % rnd].first + square2_dis) < wall_width_in[i])
							{ // 凸点找方前墙宽小于设计值，去除上一面墙所有凸点
								cout << "u u2 s1  line:" << __LINE__ << endl;
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].x -= square_defect_max[(i - 1 + rnd) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[i ].x -= square_defect_max[(i - 1 + rnd) % rnd].first;
								convexity_mod[(i - 1 + rnd) % rnd] = square_defect_max[(i - 1 + rnd) % rnd].first;
							}
							if ((square_defect_max[(i - 1 + rnd) % rnd].first + square2_dis) > wall_width_in[i])
							{// 凸点找方前墙宽大于设计值，根据凸点比例分配去除的凸点量
								cout << "u u2 s1  line:" << __LINE__ << endl;
								float diff = -square2_dis + wall_width_in[i];
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].x -= diff;
								ContourSquaredFConvexityUpdatedNew[i ].x -= diff;
								convexity_mod[(i - 1 + rnd) % rnd] = diff;
							}
							defect_state[(i - 1 + rnd) % rnd] = false;
						}
						if (!(defect_state[(i - 1 + rnd) % rnd]) && defect_state[(i + 1) % rnd])
						{ //下一面墙还未处理过凸点
							cout << "u u2 x1  line:" << __LINE__ << endl;
							convexity_mod[(i + 1) % rnd] = 0;
							defect_state[(i + 1) % rnd] = false;
						}
					}
					if ((ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].y > ContourSquaredFConvexityUpdatedNew[i ].y) && (ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].y < ContourSquaredFConvexityUpdatedNew[i ].y))
					{// u3
						if (defect_state[(i - 1 + rnd) % rnd] && defect_state[(i + 1) % rnd])
						{//两面墙都未处理过
							if ((square_defect_max[(i + 1) % rnd].first + square2_dis) < wall_width_in[i])
							{ // 凸点找方前墙宽小于设计值，去除后一面墙所有凸点
								cout << "u u3 b2  line:" << __LINE__ << endl;
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].x += square_defect_max[(i + 1) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].x += square_defect_max[(i + 1) % rnd].first;
								convexity_mod[(i - 1 + rnd) % rnd] = 0;
								convexity_mod[(i + 1) % rnd] = square_defect_max[(i + 1) % rnd].first;
							}
							if ((square_defect_max[(i + 1) % rnd].first + square2_dis) > wall_width_in[i])
							{// 凸点找方前墙宽大于设计值，根据凸点比例分配去除的凸点量
								cout << "u u3 b2  line:" << __LINE__ << endl;
								float difft = square_defect_max[(i + 1 + rnd) % rnd].first + square2_dis - wall_width_in[i];
								float difft0 = square_defect_max[(i - 1 + rnd) % rnd].first / square_defect_max[(i - 1 + rnd) % rnd].first + square_defect_max[(i + 1 + rnd) % rnd].first * difft;
								float difft1 = square_defect_max[(i + 1 + rnd) % rnd].first / square_defect_max[(i - 1 + rnd) % rnd].first + square_defect_max[(i + 1 + rnd) % rnd].first * difft;
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].x += (square_defect_max[(i - 1 + rnd) % rnd].first - difft0);
								ContourSquaredFConvexityUpdatedNew[i ].x += (square_defect_max[(i - 1 + rnd) % rnd].first - difft0);
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].x += (square_defect_max[(i + 1 + rnd) % rnd].first - difft1);
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].x += (square_defect_max[(i + 1 + rnd) % rnd].first - difft1);
								convexity_mod[(i - 1 + rnd) % rnd] = square_defect_max[(i - 1 + rnd) % rnd].first - difft0;
								convexity_mod[(i + 1) % rnd] = square_defect_max[(i + 1 + rnd) % rnd].first - difft1;
							}
							defect_state[(i - 1 + rnd) % rnd] = false;
							defect_state[(i + 1) % rnd] = false;
						}
						if (defect_state[(i - 1 + rnd) % rnd] && !(defect_state[(i + 1) % rnd]))
						{//上一面墙还未处理过凸点
							cout << "u u2 s1  line:" << __LINE__ << endl;
							convexity_mod[(i - 1 + rnd) % rnd] = 0;
							defect_state[(i - 1 + rnd) % rnd] = false;
						}
						if (!defect_state[(i - 1 + rnd) % rnd] && defect_state[(i + 1) % rnd])
						{ //下一面墙还未处理过凸点
							if ((square_defect_max[(i + 1) % rnd].first + square2_dis) < wall_width_in[i])
							{
								cout << "u u2 s1  line:" << __LINE__ << endl;
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].x -= square_defect_max[(i + 1) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].x -= square_defect_max[(i + 1) % rnd].first;
								convexity_mod[(i + 1) % rnd] = square_defect_max[(i + 1) % rnd].first;
							}
							if ((square_defect_max[(i + 1) % rnd].first + square2_dis) > wall_width_in[i])
							{
								cout << "u u2 s1  line:" << __LINE__ << endl;
								float diff = -square2_dis + wall_width_in[i];
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].x -= diff;
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].x -= diff;
								convexity_mod[(i + 1) % rnd] = diff;
							}
							defect_state[(i + 1) % rnd] = false;
						}
					}
				}
			}
			else {
				if (ContourSquaredFConvexityUpdatedNew[i ].y < ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].y) //l
				{
					if ((ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].x > ContourSquaredFConvexityUpdatedNew[i ].x) && (ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].x > ContourSquaredFConvexityUpdatedNew[i ].x))
					{//l1
						if (defect_state[(i - 1 + rnd) % rnd] && defect_state[(i + 1) % rnd])
						{ //两面墙都未处理过
							if ((square_defect_max[(i - 1 + rnd) % rnd].first + square_defect_max[(i + 1) % rnd].first + square2_dis) < wall_width_in[i])
							{ // 凸点找方前墙宽小于设计值，去除两面墙所有凸点
								cout << "l l1 b2  line:" << __LINE__ << endl;
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].y -= square_defect_max[(i - 1 + rnd) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[i ].y -= square_defect_max[(i - 1 + rnd) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].y += square_defect_max[(i + 1 + rnd) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].y += square_defect_max[(i + 1 + rnd) % rnd].first;
								convexity_mod[(i - 1 + rnd) % rnd] = square_defect_max[(i - 1 + rnd) % rnd].first;
								convexity_mod[(i + 1) % rnd] = square_defect_max[(i + 1 + rnd) % rnd].first;
							}
							if ((square_defect_max[(i - 1 + rnd) % rnd].first + square_defect_max[(i + 1) % rnd].first + square2_dis) > wall_width_in[i])
							{// 凸点找方前墙宽大于设计值，根据凸点比例分配去除的凸点量
								cout << "l l1 b2  line:" << __LINE__ << endl;
								float diff = - square2_dis + wall_width_in[i];
								float denominator = square_defect_max[(i - 1 + rnd) % rnd].first + square_defect_max[(i + 1 + rnd) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].y -= (square_defect_max[(i - 1 + rnd) % rnd].first / denominator * diff);
								ContourSquaredFConvexityUpdatedNew[i].y -= (square_defect_max[(i - 1 + rnd) % rnd].first / denominator * diff);
								ContourSquaredFConvexityUpdatedNew[(i + 1) % rnd].y += (square_defect_max[(i + 1 + rnd) % rnd].first / denominator * diff);
								ContourSquaredFConvexityUpdatedNew[(i + 2) % rnd].y += (square_defect_max[(i + 1 + rnd) % rnd].first / denominator * diff);
								convexity_mod[(i - 1 + rnd) % rnd] = square_defect_max[(i - 1 + rnd) % rnd].first / denominator * diff;
								convexity_mod[(i + 1) % rnd] = square_defect_max[(i + 1 + rnd) % rnd].first / denominator * diff;
							}
							defect_state[(i - 1 + rnd) % rnd] = false;
							defect_state[(i + 1) % rnd] = false;
						}
						if (defect_state[(i - 1 + rnd) % rnd] && !defect_state[(i + 1) % rnd])
						{//上一面墙还未处理过凸点
							if ((square_defect_max[(i - 1 + rnd) % rnd].first + square2_dis) < wall_width_in[i])
							{ // 上一面墙去除凸点墙宽小于设计值，去除上一面墙所有凸点
								cout << "l l1 s1  line:" << __LINE__ << endl;
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].y -= square_defect_max[(i - 1 + rnd) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[i ].y -= square_defect_max[(i - 1 + rnd) % rnd].first;
								convexity_mod[(i - 1 + rnd) % rnd] = square_defect_max[(i - 1 + rnd) % rnd].first;
							}
							if ((square_defect_max[(i - 1 + rnd) % rnd].first + square2_dis) > wall_width_in[i])
							{// 上一面墙去除凸点墙宽大于设计值，去除多余凸点
								cout << "l l1 s1  line:" << __LINE__ << endl;
								float diff = -square2_dis + wall_width_in[i];
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].y -= diff;
								ContourSquaredFConvexityUpdatedNew[i ].y -= diff;
								convexity_mod[(i - 1 + rnd) % rnd] = diff;
							}
							defect_state[(i - 1 + rnd) % rnd] = false;
						}
						if (!defect_state[(i - 1 + rnd) % rnd] && defect_state[(i + 1) % rnd])
						{ //下一面墙还未处理过凸点
							if ((square_defect_max[(i + 1) % rnd].first + square2_dis) < wall_width_in[i])
							{ // 凸点找方前墙宽小于设计值，去除两面墙所有凸点
								cout << "l l1 s1  line:" << __LINE__ << endl;
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].y += square_defect_max[(i + 1 + rnd) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].y += square_defect_max[(i + 1 + rnd) % rnd].first;
								convexity_mod[(i + 1) % rnd] = square_defect_max[(i + 1 + rnd) % rnd].first;
							}
							if ((square_defect_max[(i + 1) % rnd].first + square2_dis) > wall_width_in[i])
							{// 凸点找方前墙宽大于设计值，根据凸点比例分配去除的凸点量
								cout << "l l1 s1  line:" << __LINE__ << endl;
								float diff = -square2_dis + wall_width_in[i];
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].y += diff;
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].y += diff;
								convexity_mod[(i + 1) % rnd] = diff;
							}
							defect_state[(i + 1) % rnd] = false;
						}
					}
					if ((ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].x > ContourSquaredFConvexityUpdatedNew[i ].x) && (ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].x < ContourSquaredFConvexityUpdatedNew[i ].x))
					{//l2
						if (defect_state[(i - 1 + rnd) % rnd] && defect_state[(i + 1) % rnd])
						{//两面墙都未处理过
							if ((square_defect_max[(i - 1 + rnd) % rnd].first + square2_dis) < wall_width_in[i])
							{ // 凸点找方前墙宽小于设计值，去除两面墙所有凸点
								cout << "l l2 b2  line:" << __LINE__ << endl;
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].y -= square_defect_max[(i - 1 + rnd) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[i ].y -= square_defect_max[(i - 1 + rnd) % rnd].first;
								convexity_mod[(i - 1 + rnd) % rnd] = square_defect_max[(i - 1 + rnd) % rnd].first;
								convexity_mod[(i + 1) % rnd] = square_defect_max[(i - 1 + rnd) % rnd].first;
							}
							if ((square_defect_max[(i - 1 + rnd) % rnd].first + square2_dis) > wall_width_in[i])
							{// 凸点找方前墙宽大于设计值，根据凸点比例分配去除的凸点量
								cout << "l l2 b2  line:" << __LINE__ << endl;
								float difft = square_defect_max[(i - 1 + rnd) % rnd].first + square2_dis - wall_width_in[i];
								float difft0 = square_defect_max[(i - 1 + rnd) % rnd].first / square_defect_max[(i - 1 + rnd) % rnd].first + square_defect_max[(i + 1 + rnd) % rnd].first * difft;
								float difft1 = square_defect_max[(i + 1 + rnd) % rnd].first / square_defect_max[(i - 1 + rnd) % rnd].first + square_defect_max[(i + 1 + rnd) % rnd].first * difft;
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].y -= (square_defect_max[(i - 1 + rnd) % rnd].first - difft0);
								ContourSquaredFConvexityUpdatedNew[i ].y -= (square_defect_max[(i - 1 + rnd) % rnd].first - difft0);
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].y -= (square_defect_max[(i + 1 + rnd) % rnd].first - difft1);
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].y -= (square_defect_max[(i + 1 + rnd) % rnd].first - difft1);
								convexity_mod[(i - 1 + rnd) % rnd] = square_defect_max[(i - 1 + rnd) % rnd].first - difft0;
								convexity_mod[(i + 1) % rnd] = square_defect_max[(i + 1 + rnd) % rnd].first - difft1;
							}
							defect_state[(i - 1 + rnd) % rnd] = false;
							defect_state[(i + 1) % rnd] = false;
						}
						if (defect_state[(i - 1 + rnd) % rnd] && !(defect_state[(i + 1) % rnd]))
						{//上一面墙还未处理过凸点
							if ((square_defect_max[(i - 1 + rnd) % rnd].first + square2_dis) < wall_width_in[i])
							{ // 凸点找方前墙宽小于设计值，去除上一面墙所有凸点
								cout << "l l1 s1  line:" << __LINE__ << endl;
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].y -= square_defect_max[(i - 1 + rnd) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[i ].y -= square_defect_max[(i - 1 + rnd) % rnd].first;
								convexity_mod[(i - 1 + rnd) % rnd] = square_defect_max[(i - 1 + rnd) % rnd].first;
							}
							if ((square_defect_max[(i - 1 + rnd) % rnd].first + square2_dis) > wall_width_in[i])
							{// 凸点找方前墙宽大于设计值，根据凸点比例分配去除的凸点量
								cout << "l l1 s1  line:" << __LINE__ << endl;
								float diff = -square2_dis + wall_width_in[i];
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].y -= diff;
								ContourSquaredFConvexityUpdatedNew[i ].y -= diff;
								convexity_mod[(i - 1 + rnd) % rnd] = diff;
							}
							defect_state[(i - 1 + rnd) % rnd] = false;
						}
						if (!(defect_state[(i - 1 + rnd) % rnd]) && defect_state[(i + 1) % rnd])
						{ //下一面墙还未处理过凸点
							cout << "l l1 s1  line:" << __LINE__ << endl;
							convexity_mod[(i + 1) % rnd] = 0;
							defect_state[(i + 1) % rnd] = false;
						}
					}
					if ((ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].x < ContourSquaredFConvexityUpdatedNew[i ].x) && (ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].x > ContourSquaredFConvexityUpdatedNew[i ].x))
					{//l3
						if (defect_state[(i - 1 + rnd) % rnd] && defect_state[(i + 1) % rnd])
						{//两面墙都未处理过
							if ((square_defect_max[(i + 1) % rnd].first + square2_dis) < wall_width_in[i])
							{ // 凸点找方前墙宽小于设计值，去除后一面墙所有凸点
								cout << "l l3 b2  line:" << __LINE__ << endl;
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].y += square_defect_max[(i + 1) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].y += square_defect_max[(i + 1) % rnd].first;
								convexity_mod[(i - 1 + rnd) % rnd] = square_defect_max[(i + 1) % rnd].first;
								convexity_mod[(i + 1) % rnd] = square_defect_max[(i + 1) % rnd].first;
							}
							if ((square_defect_max[(i + 1) % rnd].first + square2_dis) > wall_width_in[i])
							{// 凸点找方前墙宽大于设计值，根据凸点比例分配去除的凸点量
								cout << "l l2 b2  line:" << __LINE__ << endl;
								float difft = square_defect_max[(i + 1 + rnd) % rnd].first + square2_dis - wall_width_in[i];
								float difft0 = square_defect_max[(i - 1 + rnd) % rnd].first / square_defect_max[(i - 1 + rnd) % rnd].first + square_defect_max[(i + 1 + rnd) % rnd].first * difft;
								float difft1 = square_defect_max[(i + 1 + rnd) % rnd].first / square_defect_max[(i - 1 + rnd) % rnd].first + square_defect_max[(i + 1 + rnd) % rnd].first * difft;
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].y += (square_defect_max[(i - 1 + rnd) % rnd].first - difft0);
								ContourSquaredFConvexityUpdatedNew[i ].y += (square_defect_max[(i - 1 + rnd) % rnd].first - difft0);
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].y += (square_defect_max[(i + 1 + rnd) % rnd].first - difft1);
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].y += (square_defect_max[(i + 1 + rnd) % rnd].first - difft1);
								convexity_mod[(i - 1 + rnd) % rnd] = square_defect_max[(i - 1 + rnd) % rnd].first - difft0;
								convexity_mod[(i + 1) % rnd] = square_defect_max[(i + 1 + rnd) % rnd].first - difft1;
							}
							defect_state[(i - 1 + rnd) % rnd] = false;
							defect_state[(i + 1) % rnd] = false;
						}
						if (defect_state[(i - 1 + rnd) % rnd] && !(defect_state[(i + 1) % rnd]))
						{//上一面墙还未处理过凸点
							cout << "l l1 s1  line:" << __LINE__ << endl;
							convexity_mod[(i - 1 + rnd) % rnd] = 0;
							defect_state[(i - 1 + rnd) % rnd] = false;
						}
						if (!defect_state[(i - 1 + rnd) % rnd] && defect_state[(i + 1) % rnd])
						{ //下一面墙还未处理过凸点
							if ((square_defect_max[(i + 1) % rnd].first + square2_dis) < wall_width_in[i])
							{
								cout << "l l1 s1  line:" << __LINE__ << endl;
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].y += square_defect_max[(i + 1) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].y += square_defect_max[(i + 1) % rnd].first;
								convexity_mod[(i + 1) % rnd] = square_defect_max[(i + 1) % rnd].first;
							}
							if ((square_defect_max[(i + 1) % rnd].first + square2_dis) > wall_width_in[i])
							{
								cout << "l l1 s1  line:" << __LINE__ << endl;
								float diff = -square2_dis + wall_width_in[i];
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].y += diff;
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].y += diff;
								convexity_mod[(i + 1) % rnd] = diff;
							}
							defect_state[(i + 1) % rnd] = false;
						}
					}
				}
				else { //r
;					if ((ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].x < ContourSquaredFConvexityUpdatedNew[i ].x) && (ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].x < ContourSquaredFConvexityUpdatedNew[i ].x))
					{//r1
						if (defect_state[(i - 1 + rnd) % rnd] && defect_state[(i + 1) % rnd])
						{ //两面墙都未处理过
							if ((square_defect_max[(i - 1 + rnd) % rnd].first + square_defect_max[(i + 1) % rnd].first + square2_dis) < wall_width_in[i])
							{ // 凸点找方前墙宽小于设计值，去除两面墙所有凸点
								cout << "r r1 b2  line:" << __LINE__ << endl;
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].y += square_defect_max[(i - 1 + rnd) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[i ].y += square_defect_max[(i - 1 + rnd) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].y -= square_defect_max[(i + 1 + rnd) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].y -= square_defect_max[(i + 1 + rnd) % rnd].first;
								convexity_mod[(i - 1 + rnd) % rnd] = square_defect_max[(i - 1 + rnd) % rnd].first;
								convexity_mod[(i + 1) % rnd] = square_defect_max[(i + 1 + rnd) % rnd].first;
							}
							if ((square_defect_max[(i - 1 + rnd) % rnd].first + square_defect_max[(i + 1) % rnd].first + square2_dis) > wall_width_in[i])
							{// 凸点找方前墙宽大于设计值，根据凸点比例分配去除的凸点量
								cout << "r r1 b2  line:" << __LINE__ << endl;
								float diff = -square2_dis + wall_width_in[i];
								float denominator = square_defect_max[(i - 1 + rnd) % rnd].first + square_defect_max[(i + 1 + rnd) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].y += (square_defect_max[(i - 1 + rnd) % rnd].first / denominator * diff);
								ContourSquaredFConvexityUpdatedNew[i ].y += (square_defect_max[(i - 1 + rnd) % rnd].first / denominator * diff);
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].y -= (square_defect_max[(i + 1 + rnd) % rnd].first / denominator * diff);
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].y -= (square_defect_max[(i + 1 + rnd) % rnd].first / denominator * diff);
								convexity_mod[(i - 1 + rnd) % rnd] = square_defect_max[(i - 1 + rnd) % rnd].first / denominator * diff;
								convexity_mod[(i + 1) % rnd] = square_defect_max[(i + 1 + rnd) % rnd].first / denominator * diff;
							}
							defect_state[(i - 1 + rnd) % rnd] = false;
							defect_state[(i + 1) % rnd] = false;
						}
						if (defect_state[(i - 1 + rnd) % rnd] && !defect_state[(i + 1) % rnd])
						{//上一面墙还未处理过凸点
							if ((square_defect_max[(i - 1 + rnd) % rnd].first + square2_dis) < wall_width_in[i])
							{ // 上一面墙去除凸点墙宽小于设计值，去除上一面墙所有凸点
								cout << "l l1 s1  line:" << __LINE__ << endl;
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].y += square_defect_max[(i - 1 + rnd) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[i ].y += square_defect_max[(i - 1 + rnd) % rnd].first;
								convexity_mod[(i - 1 + rnd) % rnd] = square_defect_max[(i - 1 + rnd) % rnd].first;
							}
							if ((square_defect_max[(i - 1 + rnd) % rnd].first + square2_dis) > wall_width_in[i])
							{// 上一面墙去除凸点墙宽大于设计值，去除多余凸点
								cout << "l l1 s1  line:" << __LINE__ << endl;
								float diff = -square2_dis + wall_width_in[i];
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].y += diff;
								ContourSquaredFConvexityUpdatedNew[i ].y += diff;
								convexity_mod[(i - 1 + rnd) % rnd] = diff;
							}
							ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd] = ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd];
							ContourSquaredFConvexityUpdatedNew[i] = ContourSquaredFConvexityUpdatedNew[i ];
							defect_state[(i - 1 + rnd) % rnd] = false;
						}
						if (!defect_state[(i - 1 + rnd) % rnd] && defect_state[(i + 1) % rnd])
						{ //下一面墙还未处理过凸点
							if ((square_defect_max[(i + 1) % rnd].first + square2_dis) < wall_width_in[i])
							{ // 凸点找方前墙宽小于设计值，去除两面墙所有凸点
								cout << "l l1 s1  line:" << __LINE__ << endl;
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].y -= square_defect_max[(i + 1 + rnd) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].y -= square_defect_max[(i + 1 + rnd) % rnd].first;
								convexity_mod[(i + 1) % rnd] = square_defect_max[(i + 1 + rnd) % rnd].first;
							}
							if ((square_defect_max[(i + 1) % rnd].first + square2_dis) > wall_width_in[i])
							{// 凸点找方前墙宽大于设计值，根据凸点比例分配去除的凸点量
								cout << "l l1 s1  line:" << __LINE__ << endl;
								float diff = -square2_dis + wall_width_in[i];
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].y -= diff;
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].y -= diff;
								convexity_mod[(i + 1) % rnd] = diff;
							}
							defect_state[(i + 1) % rnd] = false;
						}
					}
					if ((ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].x < ContourSquaredFConvexityUpdatedNew[i ].x) && (ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].x > ContourSquaredFConvexityUpdatedNew[i ].x))
					{//r2
						if (defect_state[(i - 1 + rnd) % rnd] && defect_state[(i + 1) % rnd])
						{//两面墙都未处理过
							if ((square_defect_max[(i - 1 + rnd) % rnd].first + square2_dis) < wall_width_in[i])
							{ // 凸点找方前墙宽小于设计值，去除两面墙所有凸点
								cout << "r r2 b2  line:" << __LINE__ << endl;
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].y += square_defect_max[(i - 1 + rnd) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[i ].y += square_defect_max[(i - 1 + rnd) % rnd].first;
								convexity_mod[(i - 1 + rnd) % rnd] = square_defect_max[(i - 1 + rnd) % rnd].first;
								convexity_mod[(i + 1) % rnd] = square_defect_max[(i - 1 + rnd) % rnd].first;
							}
							if ((square_defect_max[(i - 1 + rnd) % rnd].first + square2_dis) > wall_width_in[i])
							{// 凸点找方前墙宽大于设计值，根据凸点比例分配去除的凸点量
								cout << "r r1 b2  line:" << __LINE__ << endl;
								float difft = square_defect_max[(i - 1 + rnd) % rnd].first + square2_dis - wall_width_in[i];
								float difft0 = square_defect_max[(i - 1 + rnd) % rnd].first / square_defect_max[(i - 1 + rnd) % rnd].first + square_defect_max[(i + 1 + rnd) % rnd].first * difft;
								float difft1 = square_defect_max[(i + 1 + rnd) % rnd].first / square_defect_max[(i - 1 + rnd) % rnd].first + square_defect_max[(i + 1 + rnd) % rnd].first * difft;
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].y += (square_defect_max[(i - 1 + rnd) % rnd].first - difft0);
								ContourSquaredFConvexityUpdatedNew[i ].y += (square_defect_max[(i - 1 + rnd) % rnd].first - difft0);
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].y += (square_defect_max[(i + 1 + rnd) % rnd].first - difft1);
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].y += (square_defect_max[(i + 1 + rnd) % rnd].first - difft1);
								convexity_mod[(i - 1 + rnd) % rnd] = square_defect_max[(i - 1 + rnd) % rnd].first - difft0;
								convexity_mod[(i + 1) % rnd] = square_defect_max[(i + 1 + rnd) % rnd].first - difft1;
							}
							defect_state[(i - 1 + rnd) % rnd] = false;
							defect_state[(i + 1) % rnd] = false;
						}
						if (defect_state[(i - 1 + rnd) % rnd] && !(defect_state[(i + 1) % rnd]))
						{//上一面墙还未处理过凸点
							if ((square_defect_max[(i - 1 + rnd) % rnd].first + square2_dis) < wall_width_in[i])
							{ // 凸点找方前墙宽小于设计值，去除上一面墙所有凸点
								cout << "l l1 s1  line:" << __LINE__ << endl;
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].y += square_defect_max[(i - 1 + rnd) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[i ].y += square_defect_max[(i - 1 + rnd) % rnd].first;
								convexity_mod[(i - 1 + rnd) % rnd] = square_defect_max[(i - 1 + rnd) % rnd].first;
							}
							if ((square_defect_max[(i - 1 + rnd) % rnd].first + square2_dis) > wall_width_in[i])
							{// 凸点找方前墙宽大于设计值，根据凸点比例分配去除的凸点量
								cout << "l l1 s1  line:" << __LINE__ << endl;
								float diff = -square2_dis + wall_width_in[i];
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].y += diff;
								ContourSquaredFConvexityUpdatedNew[i ].y += diff;
								convexity_mod[(i - 1 + rnd) % rnd] = diff;
							}
							ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd] = ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd];
							ContourSquaredFConvexityUpdatedNew[i] = ContourSquaredFConvexityUpdatedNew[i ];
							defect_state[(i - 1 + rnd) % rnd] = false;
						}
						if (!(defect_state[(i - 1 + rnd) % rnd]) && defect_state[(i + 1) % rnd])
						{ //下一面墙还未处理过凸点
							cout << "l l1 s1  line:" << __LINE__ << endl;
							convexity_mod[(i + 1) % rnd] = 0;
							defect_state[(i + 1) % rnd] = false;
						}
					}
					if ((ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].x > ContourSquaredFConvexityUpdatedNew[i ].x) && (ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].y < ContourSquaredFConvexityUpdatedNew[i ].y))
					{//r3
						if (defect_state[(i - 1 + rnd) % rnd] && defect_state[(i + 1) % rnd])
						{//两面墙都未处理过
							if ((square_defect_max[(i + 1) % rnd].first + square2_dis) < wall_width_in[i])
							{ // 凸点找方前墙宽小于设计值，去除后一面墙所有凸点
								cout << "r r3 b2  line:" << __LINE__ << endl;
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].y -= square_defect_max[(i + 1) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].y -= square_defect_max[(i + 1) % rnd].first;
								convexity_mod[(i - 1 + rnd) % rnd] = square_defect_max[(i + 1) % rnd].first;
								convexity_mod[(i + 1) % rnd] = square_defect_max[(i + 1) % rnd].first;
							}
							if ((square_defect_max[(i + 1) % rnd].first + square2_dis) > wall_width_in[i])
							{// 凸点找方前墙宽大于设计值，根据凸点比例分配去除的凸点量
								cout << "r r3 b2  line:" << __LINE__ << endl;
								float difft = square_defect_max[(i + 1 + rnd) % rnd].first + square2_dis - wall_width_in[i];
								float difft0 = square_defect_max[(i - 1 + rnd) % rnd].first / square_defect_max[(i - 1 + rnd) % rnd].first + square_defect_max[(i + 1 + rnd) % rnd].first * difft;
								float difft1 = square_defect_max[(i + 1 + rnd) % rnd].first / square_defect_max[(i - 1 + rnd) % rnd].first + square_defect_max[(i + 1 + rnd) % rnd].first * difft;
								ContourSquaredFConvexityUpdatedNew[(i - 1 + rnd) % rnd].y -= (square_defect_max[(i - 1 + rnd) % rnd].first - difft0);
								ContourSquaredFConvexityUpdatedNew[i ].y -= (square_defect_max[(i - 1 + rnd) % rnd].first - difft0);
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].y -= (square_defect_max[(i + 1 + rnd) % rnd].first - difft1);
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].y -= (square_defect_max[(i + 1 + rnd) % rnd].first - difft1);
								convexity_mod[(i - 1 + rnd) % rnd] = square_defect_max[(i - 1 + rnd) % rnd].first - difft0;
								convexity_mod[(i + 1) % rnd] = square_defect_max[(i + 1 + rnd) % rnd].first - difft1;
							}
							defect_state[(i - 1 + rnd) % rnd] = false;
							defect_state[(i + 1) % rnd] = false;
						}
						if (defect_state[(i - 1 + rnd) % rnd] && !(defect_state[(i + 1) % rnd]))
						{//上一面墙还未处理过凸点
							cout << "l l1 s1  line:" << __LINE__ << endl;
							convexity_mod[(i - 1 + rnd) % rnd] = 0;
							defect_state[(i - 1 + rnd) % rnd] = false;
						}
						if (!defect_state[(i - 1 + rnd) % rnd] && defect_state[(i + 1) % rnd])
						{ //下一面墙还未处理过凸点
							if ((square_defect_max[(i + 1) % rnd].first + square2_dis) < wall_width_in[i])
							{
								cout << "l l1 s1  line:" << __LINE__ << endl;
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].y -= square_defect_max[(i + 1) % rnd].first;
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].y -= square_defect_max[(i + 1) % rnd].first;
								convexity_mod[(i + 1) % rnd] = square_defect_max[(i + 1) % rnd].first;
							}
							if ((square_defect_max[(i + 1) % rnd].first + square2_dis) > wall_width_in[i])
							{
								cout << "l l1 s1  line:" << __LINE__ << endl;
								float diff = -square2_dis + wall_width_in[i];
								ContourSquaredFConvexityUpdatedNew[(i +1) % rnd].y -= diff;
								ContourSquaredFConvexityUpdatedNew[(i +2) % rnd].y -= diff;
								convexity_mod[(i + 1) % rnd] = diff;
							}
							ContourSquaredFConvexityUpdatedNew[(i + 1) % rnd] = ContourSquaredFConvexityUpdatedNew[(i +1) % rnd];
							ContourSquaredFConvexityUpdatedNew[(i + 2) % rnd] = ContourSquaredFConvexityUpdatedNew[(i +2) % rnd];
							defect_state[(i + 1) % rnd] = false;
						}
					}
				}
			}
		}
		//wall_width_in[i] = DisOf2Points2f(ContourSquaredFConvexityUpdatedNew[i], ContourSquaredFConvexityUpdatedNew[(i + 1) % rnd]);
	}


	for (int i = 0; i < rnd; i++)
	{
		wall_width_info[i].second = DisOf2Points2f(ContourSquaredFConvexityUpdatedNew[i], ContourSquaredFConvexityUpdatedNew[(i + 1) % rnd]);
		cout << "optimized squared wall width: " << wall_width_info[i].second << endl;
	}

	for (int i = 0; i < rnd; i++)
	{
		//cout << "squared wall width: " << DisOf2Points2f(ContourSquaredFConvexityUpdatedNew[i], ContourSquaredFConvexityUpdatedNew[(i + 1) % rnd]) << endl;;
		cout << "optimized ContourSquaredFConvexityUpdatedNew[" << i << "]: " << ContourSquaredFConvexityUpdatedNew[i] << endl;
		cout << "======k : " << i << " convexity_mod: " << convexity_mod[i] << endl;
	}

	for (int k = 0; k < rnd; k++)
	{
		float rm_vol = 0;
		for(int j= 0; j < squared_defect_r[k].size();j++)
		{
			if (squared_defect_r[k][j].first > (square_defect_max[k].first - convexity_mod[k]))
			{
				cv:Point3f pdb;
				MeasureBase::RotatePoint(squared_defect_r[k][j].second, b_mat_to_y, pdb);
				squared_defect_info[k].push_back(std::make_pair(squared_defect_r[k][j].first -(square_defect_max[k].first - convexity_mod[k]), pdb));

			//	cout << "k:" << k << " j:" << j << " mod:" << squared_defect_r[k][j].first - (square_defect_max[k].first - convexity_mod[k])
				//                                << " point:" << squared_defect_r[k][j].second << endl;
				rm_vol += (squared_defect_r[k][j].first - (square_defect_max[k].first - convexity_mod[k])) * ruler_width * ruler_height;
			}
		}
		filling_vol_opt[k] = filling_vol[k].second - (rm_vol / 1000000000);
		cout <<"squared_defect_info[k].size():"<< squared_defect_info[k].size() << endl;
	}
#endif

		if (pts0.size() != CuboidUpdatedContourSquared.size())
		{
			std::cout << "##### ==>  SQUARE_BY_CONVEXITY  pts0.size() != CuboidUpdatedContourSquared.size()" << endl;
			const int fallback_size = static_cast<int>(pts0.size());
			std::vector<cv::Point3f> pts0_back = MathOperation::plane_rot(b_mat_to_y, pts0);
			for (int k = 0; k < fallback_size; k++)
			{
				cv::Point2f pt_s, tmep;
				pt_s.x = pts0_back[k].x;
				pt_s.y = pts0_back[k].y;
				tmep.x = 0;
				tmep.y = 0;
				ConvexityContourSquaredJson.push_back(std::make_pair(pt_s, tmep));
				ContourSquaredForHoles.push_back(std::make_pair(pt_s, tmep));

				cv::Point3f tmep3;
				tmep3.x = tmep3.y = tmep3.z = 0;
				ConvexityContourSquaredJson3d.push_back(std::make_pair(pts0_back[k], tmep3));

				// Fallback: keep contour_squared1 non-empty so downstream wall/plane mapping
				// does not receive an empty RoomContour.
				contour_squared1.push_back(std::make_pair(pts0_back[k], tmep3));
			}
		}
		else {
#ifdef DBRT
			std::vector<cv::Point3f> temp;
			std::vector<cv::Point3f> PtsForRoomSquared_sh;
			float mx = 0;
			int md = 0;
			for (int k = 0; k < ContourSquaredFConvexityUpdated.size(); k++)
			{
				cv::Point3f tt;
				tt.x = ContourSquaredFConvexityUpdated[k].first.x;
				tt.y = ContourSquaredFConvexityUpdated[k].first.y;
				tt.z = pts0[k].z;
				temp.push_back(tt);
				PtsForRoomSquared_sh.push_back(tt);
				//cout << "squared tt i: " << k << " " << tt << endl;
				if (mx < tt.x) {
					mx = tt.x;	md = k;
				}
			}
			int rd = PtsForRoomSquared_sh.size();
			if ((CuboidUpdatedContourSquared[(md + rd - 1) % rd].x - CuboidUpdatedContourSquared[(md + 1) % rd].x)*(CuboidUpdatedContourSquared[md].y - CuboidUpdatedContourSquared[(md + 1) % rd].y) -
				(CuboidUpdatedContourSquared[(md + rd - 1) % rd].y - CuboidUpdatedContourSquared[(md + 1) % rd].y)*(CuboidUpdatedContourSquared[md].x - CuboidUpdatedContourSquared[(md + 1) % rd].x) > 0) {
				//cout << "===========counterclock." << endl;

				for (int k = 0; k < PtsForRoomSquared_sh.size(); k++)
				{
					if (abs(PtsForRoomSquared_sh[k].x - PtsForRoomSquared_sh[(k + 1) % rd].x) >
						abs(PtsForRoomSquared_sh[k].y - PtsForRoomSquared_sh[(k + 1) % rd].y))
					{
						if (PtsForRoomSquared_sh[k].x < PtsForRoomSquared_sh[(k + 1) % rd].x) {
							PtsForRoomSquared_sh[k].y += shrink;
							PtsForRoomSquared_sh[(k + 1) % rd].y += shrink;
						}
						else {
							PtsForRoomSquared_sh[k].y -= shrink;
							PtsForRoomSquared_sh[(k + 1) % rd].y -= shrink;
						}
					}
					else {
						if (PtsForRoomSquared_sh[k].y < PtsForRoomSquared_sh[(k + 1) % rd].y) {
							PtsForRoomSquared_sh[k].x -= shrink;
							PtsForRoomSquared_sh[(k + 1) % rd].x -= shrink;
						}
						else {
							PtsForRoomSquared_sh[k].x += shrink;
							PtsForRoomSquared_sh[(k + 1) % rd].x += shrink;
						}

					}
				}
			}
			else {
				for (int k = 0; k < PtsForRoomSquared_sh.size(); k++)
				{
					if (abs(PtsForRoomSquared_sh[k].x - PtsForRoomSquared_sh[(k + 1) % rd].x) >
						abs(PtsForRoomSquared_sh[k].y - PtsForRoomSquared_sh[(k + 1) % rd].y))
					{
						if (PtsForRoomSquared_sh[k].x < PtsForRoomSquared_sh[(k + 1) % rd].x) {
							PtsForRoomSquared_sh[k].y -= shrink;
							PtsForRoomSquared_sh[(k + 1) % rd].y -= shrink;
						}
						else {
							PtsForRoomSquared_sh[k].y += shrink;
							PtsForRoomSquared_sh[(k + 1) % rd].y += shrink;
						}
					}
					else {
						if (PtsForRoomSquared_sh[k].y < PtsForRoomSquared_sh[(k + 1) % rd].y) {
							PtsForRoomSquared_sh[k].x += shrink;
							PtsForRoomSquared_sh[(k + 1) % rd].x += shrink;
						}
						else {
							PtsForRoomSquared_sh[k].x -= shrink;
							PtsForRoomSquared_sh[(k + 1) % rd].x -= shrink;
						}

					}
				}
			}

			temp = MathOperation::plane_rot(b_mat_to_y, temp);
			PtsForRoomSquared_sh = MathOperation::plane_rot(b_mat_to_y, PtsForRoomSquared_sh);

			for (int k = 0; k < ContourSquaredFConvexityUpdated.size(); k++)
			{
				cv::Point2f tempPts, tempDis;
				tempPts.x = temp[k].x;
				tempPts.y = temp[k].y;
				//cout << "squared rback tempPts i: " << k << " " << tempPts << endl;
				tempDis.x = abs(ContourSquaredFConvexityUpdated[k].first.x - pts0[k].x);
				tempDis.y = abs(ContourSquaredFConvexityUpdated[k].first.y - pts0[k].y);
				ConvexityContourSquaredJson.push_back(std::make_pair(tempPts, tempDis));

				cv::Point3f tempDis3;
				tempDis3.x = abs(ContourSquaredFConvexityUpdated[k].first.x - pts0[k].x);
				tempDis3.y = abs(ContourSquaredFConvexityUpdated[k].first.y - pts0[k].y);
				tempDis3.z = 0;
				ConvexityContourSquaredJson3d.push_back(std::make_pair(temp[k], tempDis3));
			}
			if (IsSquareByAxis()) {
				tempbr.push_back(temp[ref_wall_contour_id]);
				tempbr.push_back(temp[(ref_wall_contour_id + 1) % temp.size()]);
			}

			for (int k = 0; k < PtsForRoomSquared_sh.size(); k++)
			{
				cv::Point3f tempPts3, tempDis3;
				tempPts3.x = PtsForRoomSquared_sh[k].x;
				tempPts3.y = PtsForRoomSquared_sh[k].y;
				tempPts3.z = PtsForRoomSquared_sh[k].z;
				//cout << "squared rback tempPts i: " << k << " " << tempPts << endl;
				tempDis3.x = abs(PtsForRoomSquared_sh[k].x - pts0[k].x);
				tempDis3.y = abs(PtsForRoomSquared_sh[k].y - pts0[k].y);
				tempDis3.z = 0;
				//cout << "tempDis31: " << tempDis3 << endl;
				contour_squared1.push_back(std::make_pair(tempPts3, tempDis3));
			}

#if 1
			for (int k = 0; k < pts0.size(); k++)
			{
				cv::Point2f tempPts, tempDis;
				tempPts.x = ContourSquaredFConvexityUpdated[k].first.x;
				tempPts.y = ContourSquaredFConvexityUpdated[k].first.y;
				//cout << " tempPts i: " << k << " " << tempPts << endl;
				tempDis.x = abs(ContourSquaredFConvexityUpdated[k].first.x - pts0[k].x);
				tempDis.y = abs(ContourSquaredFConvexityUpdated[k].first.y - pts0[k].y);
				ContourSquaredForHoles.push_back(std::make_pair(tempPts, tempDis));
			}
#endif
#else
			for (int k = 0; k < pts0.size(); k++)
			{
				cv::Point2f tempPts, tempDis;
				tempPts.x = ContourSquaredFConvexityUpdated[k].first.x;
				tempPts.y = ContourSquaredFConvexityUpdated[k].first.y;
				//cout << " tempPts i: " << k << " " << tempPts << endl;
				tempDis.x = abs(ContourSquaredFConvexityUpdated[k].first.x - pts0[k].x);
				tempDis.y = abs(ContourSquaredFConvexityUpdated[k].first.y - pts0[k].y);
				ConvexityContourSquaredJson.push_back(std::make_pair(tempPts, tempDis));
				ContourSquaredForHoles.push_back(std::make_pair(tempPts, tempDis));
			}
#endif
		}
	}///SQUARE_BY_CONVEXITY

#if 1 ///=============
	holes_projected.resize(mSPlane.size());
	for (int k = 0; k < pts.size(); k++)
	{
#ifdef Debug_Hols_P
		cout << "\n\n============k:" << k << "  " << pts[k] << " " << pts[(k + 1) % round] << endl;
#endif
		bool Exit0 = false;
		for (auto wall : wallList)
		{
			if (wall == -1)
				continue;
			bool Fidx = false;
			bool Sidx = false;

			for (int i = 0; i < mSPlane[wall].vertices.size(); i++)
			{
				if (mSPlane[wall].vertices[i].z < 0) {
#ifdef Debug_Hols_P
					cout << "wall:" << wall << " vetices: " << mSPlane[wall].vertices[i] << endl;
#endif
					if ((abs(mSPlane[wall].vertices[i].x - pts[k].x) <= 5.0f) &&
						(abs(mSPlane[wall].vertices[i].y - pts[k].y) <= 5.0f))
					{
						Fidx = true;
					}

					if ((abs(mSPlane[wall].vertices[i].x - pts[(k + 1) % round].x) <= 5.0f) &&
						(abs(mSPlane[wall].vertices[i].y - pts[(k + 1) % round].y) <= 5.0f))
					{
						Sidx = true;
					}
				}
				if (Fidx && Sidx)
				{
#ifdef Debug_Hols_P
					cout << "=====Fidx Sidx = true;=====wall:"<<wall << endl;
#endif
					cv::Point2f Pa;
					int round = ContourSquaredForHoles.size();
					getLinePara(ContourSquaredForHoles[k].first, ContourSquaredForHoles[(k+1)% round].first, Pa);
#ifdef Debug_Hols_P
					cout << "line0: " << ContourSquaredForHoles[k].first << " , " << ContourSquaredForHoles[(k + 1) % round].first << endl;
					cout << "Pa: " << Pa.x << " , " << Pa.y << endl;
#endif
					holes_projected[wall].resize(mSPlane[wall].holes.size());
					if (abs(ContourSquaredForHoles[k].first.x - ContourSquaredForHoles[(k + 1) % round].first.x) >  // wall // xoz
						abs(ContourSquaredForHoles[k].first.y - ContourSquaredForHoles[(k + 1) % round].first.y))
					{
						for (int n = 0; n < mSPlane[wall].holes.size(); n++)
						{
							if (mSPlane[wall].holes[n].vertice.size() != 4)
								continue;
							mSPlane[wall].holes[n].vertice_s = mSPlane[wall].holes[n].vertice;

							for (int m = 0; m < mSPlane[wall].holes[n].vertice.size(); m++)
							{
									mSPlane[wall].holes[n].vertice_s[m].y = Pa.x*mSPlane[wall].holes[n].vertice[m].x + Pa.y;
							}
							holes_projected[wall][n] = MathOperation::plane_rot(b_mat_to_y, mSPlane[wall].holes[n].vertice_s);
#ifdef Debug_Hols_PS
							cout <<"xozProject: holes_vertice_s0:\n" << mSPlane[wall].holes[n].vertice_s << endl;
							SavePoint3fDataT("xozProject_" + to_string(wall) + "_" + to_string(n) + ".txt", mSPlane[wall].holes[n].vertice_s);
#endif

							for (int m = 0; m < mSPlane[wall].holes[n].vertice_s.size(); m++)
							{
								if (abs(mSPlane[wall].holes[n].vertice_s[m].x - mSPlane[wall].holes[n].vertice_s[(m + 1)%4].x) >
									abs(mSPlane[wall].holes[n].vertice_s[m].z - mSPlane[wall].holes[n].vertice_s[(m + 1)%4].z))
								{  //x
									if (mSPlane[wall].holes[n].vertice_s[m].z < mSPlane[wall].holes[n].vertice_s[(m + 2) % 4].z)
									{ //bot
										if (mSPlane[wall].holes[n].vertice_s[m].z > mSPlane[wall].holes[n].vertice_s[(m + 1) % 4].z)
										{
											mSPlane[wall].holes[n].vertice_s[m].z = mSPlane[wall].holes[n].vertice_s[(m + 1) % 4].z;
										}
										else {
											mSPlane[wall].holes[n].vertice_s[(m + 1) % 4].z = mSPlane[wall].holes[n].vertice_s[m].z;
										}
#ifdef Debug_Hols_P
										cout << "bottom: " << mSPlane[wall].holes[n].vertice_s[m] << " , " << mSPlane[wall].holes[n].vertice_s[(m + 1) % 4] << endl;
#endif
									}
									else
									{//up
										if (mSPlane[wall].holes[n].vertice_s[m].z < mSPlane[wall].holes[n].vertice_s[(m + 1) % 4].z)
										{
											mSPlane[wall].holes[n].vertice_s[m].z = mSPlane[wall].holes[n].vertice_s[(m + 1) % 4].z;
										}
										else {
											mSPlane[wall].holes[n].vertice_s[(m + 1) % 4].z = mSPlane[wall].holes[n].vertice_s[m].z;

										}
#ifdef Debug_Hols_P
										cout << "up: " << mSPlane[wall].holes[n].vertice_s[m] << " , " << mSPlane[wall].holes[n].vertice_s[(m + 1) % 4] << endl;
#endif
									}
								}
								else
								{ //z
									if (mSPlane[wall].holes[n].vertice_s[m].x < mSPlane[wall].holes[n].vertice_s[(m + 2) % 4].x)
									{ //left
										if (mSPlane[wall].holes[n].vertice_s[m].x > mSPlane[wall].holes[n].vertice_s[(m + 1) % 4].x)
										{
											mSPlane[wall].holes[n].vertice_s[m].x = mSPlane[wall].holes[n].vertice_s[(m + 1) % 4].x;
										}
										else {
											mSPlane[wall].holes[n].vertice_s[(m + 1) % 4].x = mSPlane[wall].holes[n].vertice_s[m].x;

										}
#ifdef Debug_Hols_P
										cout << "left: " << mSPlane[wall].holes[n].vertice_s[m] << " , " << mSPlane[wall].holes[n].vertice_s[(m + 1) % 4] << endl;
#endif
									}
									else
									{//right
										if (mSPlane[wall].holes[n].vertice_s[m].x < mSPlane[wall].holes[n].vertice_s[(m + 1) % 4].x)
										{
											mSPlane[wall].holes[n].vertice_s[m].x = mSPlane[wall].holes[n].vertice_s[(m + 1) % 4].x;
										}
										else {
											mSPlane[wall].holes[n].vertice_s[(m + 1) % 4].x = mSPlane[wall].holes[n].vertice_s[m].x;
										}
#ifdef Debug_Hols_P
										cout << "right: " << mSPlane[wall].holes[n].vertice_s[m] << " , " << mSPlane[wall].holes[n].vertice_s[(m + 1) % 4] << endl;
#endif
									}
								}
							}
						}
					}
					else
					{// wall // yoz
						for (int n = 0; n < mSPlane[wall].holes.size(); n++)
						{
							if (mSPlane[wall].holes[n].vertice.size() != 4)
								continue;
							mSPlane[wall].holes[n].vertice_s = mSPlane[wall].holes[n].vertice;
							for (int m = 0; m < mSPlane[wall].holes[n].vertice.size(); m++) {
								if (abs(Pa.x) < 1e-6f) {
									mSPlane[wall].holes[n].vertice_s[m].x = ContourSquaredForHoles[k].first.x;
								}
								else {
									mSPlane[wall].holes[n].vertice_s[m].x = (mSPlane[wall].holes[n].vertice[m].y - Pa.y) / Pa.x;
								}
							}
							holes_projected[wall][n] = MathOperation::plane_rot(b_mat_to_y, mSPlane[wall].holes[n].vertice_s);
#ifdef Debug_Hols_PS
							cout << "yozProject: holes_vertice_s0:\n" << mSPlane[wall].holes[n].vertice_s << endl;
							SavePoint3fDataT("yozProject_" + to_string(wall) + "_" + to_string(n) + ".txt", mSPlane[wall].holes[n].vertice_s);
#endif
							for (int m = 0; m < mSPlane[wall].holes[n].vertice_s.size(); m++)
							{
								if (abs(mSPlane[wall].holes[n].vertice_s[m].y - mSPlane[wall].holes[n].vertice_s[(m + 1) % 4].y) > //y
									abs(mSPlane[wall].holes[n].vertice_s[m].z - mSPlane[wall].holes[n].vertice_s[(m + 1) % 4].z))
								{
									if (mSPlane[wall].holes[n].vertice_s[m].z < mSPlane[wall].holes[n].vertice_s[(m + 2) % 4].z)
									{ //bot
										if (mSPlane[wall].holes[n].vertice_s[m].z > mSPlane[wall].holes[n].vertice_s[(m + 1) % 4].z)
										{
											mSPlane[wall].holes[n].vertice_s[m].z = mSPlane[wall].holes[n].vertice_s[(m + 1) % 4].z;
										}
										else {
											mSPlane[wall].holes[n].vertice_s[(m + 1) % 4].z = mSPlane[wall].holes[n].vertice_s[m].z;
										}
#ifdef Debug_Hols_P
										cout << "bottom: " << mSPlane[wall].holes[n].vertice_s[m] << " , " << mSPlane[wall].holes[n].vertice_s[(m + 1) % 4] << endl;
#endif
									}
									else
									{//up
										if (mSPlane[wall].holes[n].vertice_s[m].z < mSPlane[wall].holes[n].vertice_s[(m + 1) % 4].z)
										{
											mSPlane[wall].holes[n].vertice_s[m].z = mSPlane[wall].holes[n].vertice_s[(m + 1) % 4].z;
										}
										else {
											mSPlane[wall].holes[n].vertice_s[(m + 1) % 4].z = mSPlane[wall].holes[n].vertice_s[m].z;
										}
#ifdef Debug_Hols_P
										cout << "up: " << mSPlane[wall].holes[n].vertice_s[m] << " , " << mSPlane[wall].holes[n].vertice_s[(m + 1) % 4] << endl;
#endif
									}
								}
								else
								{ //z
									if (mSPlane[wall].holes[n].vertice_s[m].y < mSPlane[wall].holes[n].vertice_s[(m + 2) % 4].y)
									{ //left
										if (mSPlane[wall].holes[n].vertice_s[m].y > mSPlane[wall].holes[n].vertice_s[(m + 1) % 4].y)
										{
											mSPlane[wall].holes[n].vertice_s[m].y = mSPlane[wall].holes[n].vertice_s[(m + 1) % 4].y;
										}
										else {
											mSPlane[wall].holes[n].vertice_s[(m + 1) % 4].y = mSPlane[wall].holes[n].vertice_s[m].y;
										}
#ifdef Debug_Hols_P
										cout << "left: " << mSPlane[wall].holes[n].vertice_s[m] << " , " << mSPlane[wall].holes[n].vertice_s[(m + 1) % 4] << endl;
#endif
									}
									else
									{//right
										if (mSPlane[wall].holes[n].vertice_s[m].y < mSPlane[wall].holes[n].vertice_s[(m + 1) % 4].y)
										{
											mSPlane[wall].holes[n].vertice_s[m].y = mSPlane[wall].holes[n].vertice_s[(m + 1) % 4].y;
										}
										else {
											mSPlane[wall].holes[n].vertice_s[(m + 1) % 4].y = mSPlane[wall].holes[n].vertice_s[m].y;
										}
#ifdef Debug_Hols_P
										cout << "right: " << mSPlane[wall].holes[n].vertice_s[m] << " , " << mSPlane[wall].holes[n].vertice_s[(m + 1) % 4] << endl;
#endif
									}
								}
							}
						}
					}
					Exit0 = true;
					break;
				}
			}
			if (Exit0) {
				break;
			}
		}
	}
#endif

	holes_sq.resize(mSPlane.size());
	holes_sq1.resize(mSPlane.size());
	for (auto wall : wallList)
	{
		if (wall == -1)
			continue;
		holes_sq[wall].resize(mSPlane[wall].holes.size());
		holes_sq1[wall].resize(mSPlane[wall].holes.size());
		for (int n = 0; n < mSPlane[wall].holes.size(); n++)
		{
			if (mSPlane[wall].holes[n].vertice_s.size() != 4)
				continue;
			holes_sq[wall][n].id = mSPlane2[wall].holes[n].id;
			holes_sq[wall][n].type = mSPlane2[wall].holes[n].type;
			holes_sq[wall][n].height = abs(mSPlane[wall].holes[n].vertice_s[0].z - mSPlane[wall].holes[n].vertice_s[2].z);
			holes_sq1[wall][n].id = holes_sq[wall][n].id;
			holes_sq1[wall][n].type = holes_sq[wall][n].type;
			holes_sq1[wall][n].height = holes_sq[wall][n].height;
			if (abs(mSPlane[wall].holes[n].vertice_s[0].x - mSPlane[wall].holes[n].vertice_s[1].x) >
				abs(mSPlane[wall].holes[n].vertice_s[0].y - mSPlane[wall].holes[n].vertice_s[1].y)) {
				holes_sq[wall][n].width = abs(mSPlane[wall].holes[n].vertice_s[0].x - mSPlane[wall].holes[n].vertice_s[2].x);
				holes_sq1[wall][n].width = holes_sq[wall][n].width;
			}
			else {
				holes_sq[wall][n].width = abs(mSPlane[wall].holes[n].vertice_s[0].y - mSPlane[wall].holes[n].vertice_s[2].y);
				holes_sq1[wall][n].width = holes_sq[wall][n].width;
			}
#ifdef Debug_MHS
			cout << "squared holes wall:" << wall << " holes:" << n << " vertice:\n" << mSPlane[wall].holes[n].vertice << endl;
			cout << "squared holes wall:" << wall << " holes:" << n << " width:" << holes_sq[wall][n].width << " height:" << holes_sq[wall][n].height << endl;
			std::stringstream ss;
			ss << n;
			SavePoint3fDataT("squared_holes_" + ss.str() + ".txt", mSPlane[wall].holes[n].vertice_s);
#endif
#ifdef DBRT
			mSPlane2[wall].holes[n].vertice = MathOperation::plane_rot(b_mat_to_y, mSPlane[wall].holes[n].vertice);
			mSPlane2[wall].holes[n].vertice_s = MathOperation::plane_rot(b_mat_to_y, mSPlane[wall].holes[n].vertice_s);


			holes_sq[wall][n].vertice = mSPlane2[wall].holes[n].vertice_s;
			holes_sq1[wall][n].vertice = holes_sq[wall][n].vertice;
#else
			holes_sq[wall][n].vertice = mSPlane[wall].holes[n].vertice_s;
			holes_sq1[wall][n].vertice = holes_sq[wall][n].vertice
#endif


#ifdef Debug_MHS
			cout << "holes_sq[wall].size(): " << holes_sq[wall].size() << " holes_projected[wall].size(): " << holes_projected[wall].size() << endl;
			cout << "squared holes rotate back, wall:" << wall << " holes:" << n << " holes_sq[wall][n].vertice:\n" << holes_sq[wall][n].vertice << endl;
			SavePoint3fDataT("squared_holes_rb_" + ss.str() + ".txt", holes_sq[wall][n].vertice);
#endif
		}
	}

#if 1
	for (int k = 0; k < pts.size(); k++)
	{
		//cout << "\n\n==========k:" << k << "  " << pts[k] << " " << pts[(k + 1) % round] << endl;
		bool Exit0 = false;
		for (auto wall : wallList)
		{
			if (wall == -1)
				continue;
			bool Fidx = false;
			bool Sidx = false;
			for (int i = 0; i < mSPlane[wall].vertices.size(); i++)
			{
				if (mSPlane[wall].vertices[i].z < 0) {

					if ((abs(mSPlane[wall].vertices[i].x - pts[k].x) <= 5.0f) &&
						(abs(mSPlane[wall].vertices[i].y - pts[k].y) <= 5.0f))
					{
						Fidx = true;
					}
					if ((abs(mSPlane[wall].vertices[i].x - pts[(k + 1) % round].x) <= 5.0f) &&
						(abs(mSPlane[wall].vertices[i].y - pts[(k + 1) % round].y) <= 5.0f))
					{
						Sidx = true;
					}
				}
				if (Fidx && Sidx)
				{
					if (holes_sq[wall].size() > 0) {
						int rd = pts.size();
						float mx = 0;
						int md = 0;
						for (int tt = 0; tt < pts.size(); tt++) {
							if (mx < pts[tt].x) {
								mx = pts[tt].x;	md = tt;
							}
						}
						if ((pts[(md + rd - 1) % rd].x - pts[(md + 1) % rd].x)*(pts[md].y - pts[(md + 1) % rd].y) -
							(pts[(md + rd - 1) % rd].y - pts[(md + 1) % rd].y)*(pts[md].x - pts[(md + 1) % rd].x) > 0)
						{
							//cout << "===========counter.  holes_sq1[wall].size():" << holes_sq1[wall].size() << endl;
							for (int n = 0; n < holes_sq1[wall].size(); n++)
							{
								holes_sq1[wall][n].vertice = MathOperation::plane_rot(Brotation_matrix, holes_sq1[wall][n].vertice);
							//	cout << "==========k:" << k << " holes_sq1[wall][n].vertice:\n" <<holes_sq1[wall][n].vertice << endl;
								if (abs(pts[k].x - pts[(k + 1) % rd].x) >
									abs(pts[k].y - pts[(k + 1) % rd].y))
								{
									if (pts[k].x < pts[(k + 1) % rd].x) {
										for (int v = 0; v < holes_sq1[wall][n].vertice.size(); v++) {
											holes_sq1[wall][n].vertice[v].y += shrink;
										}
									}
									else {
										for (int v = 0; v < holes_sq1[wall][n].vertice.size(); v++) {
											holes_sq1[wall][n].vertice[v].y -= shrink;
										}
									}
								}
								else {
									if (pts[k].y < pts[(k + 1) % rd].y) {
										for (int v = 0; v < holes_sq1[wall][n].vertice.size(); v++) {
											holes_sq1[wall][n].vertice[v].x -= shrink;
										}
									}
									else {
										for (int v = 0; v < holes_sq1[wall][n].vertice.size(); v++) {
											holes_sq1[wall][n].vertice[v].x += shrink;
										}
									}

								}
							//	cout << "==========k:" << k << " holes_sq1[wall][n].vertice:\n" << holes_sq1[wall][n].vertice << endl;
								holes_sq1[wall][n].vertice = MathOperation::plane_rot(b_mat_to_y, holes_sq1[wall][n].vertice);
							}
						}
						else {
						//	cout << "===========wise.  holes_sq1[wall].size():" << holes_sq1[wall].size() << endl;
							for (int n = 0; n < holes_sq1[wall].size(); n++)
							{
								holes_sq1[wall][n].vertice = MathOperation::plane_rot(Brotation_matrix, holes_sq1[wall][n].vertice);
						//		cout << "==========k:" << k << " holes_sq1[wall][n].vertice:\n" << holes_sq1[wall][n].vertice << endl;
								if (abs(pts[k].x - pts[(k + 1) % rd].x) >
									abs(pts[k].y - pts[(k + 1) % rd].y))
								{
									if (pts[k].x < pts[(k + 1) % rd].x) {

										for (int v = 0; v < holes_sq1[wall][n].vertice.size(); v++) {
											holes_sq1[wall][n].vertice[v].y -= shrink;
										}
									}
									else {
										for (int v = 0; v < holes_sq1[wall][n].vertice.size(); v++) {
											holes_sq1[wall][n].vertice[v].y += shrink;
										}
									}
								}
								else {
									if (pts[k].y < pts[(k + 1) % rd].y) {
										for (int v = 0; v < holes_sq1[wall][n].vertice.size(); v++) {
											holes_sq1[wall][n].vertice[v].x += shrink;
										}
									}
									else {
										for (int v = 0; v < holes_sq1[wall][n].vertice.size(); v++) {
											holes_sq1[wall][n].vertice[v].x -= shrink;
										}
									}
								}
							//	cout << "==========k:" << k << " holes_sq1[wall][n].vertice:\n" << holes_sq1[wall][n].vertice << endl;
								holes_sq1[wall][n].vertice = MathOperation::plane_rot(b_mat_to_y, holes_sq1[wall][n].vertice);
							}
						}
					}
					Exit0 = true;
					break;
				}
			}
			if (Exit0) {
				break;
			}
		}
	}
#endif

	//mPRecon->InsertFakePoint(insertMap);
#ifdef DBRT
	insertedPts = MathOperation::plane_rot(b_mat_to_y, insertedPts);
#endif
	mPRecon->InsertFakePoint(insertedPts, insertedWallList);

	std::vector <std::pair< cv::Point3f, cv::Point3f>> RoomSquaredJson3d;
	GetSquaredHolePoint3dHlper( mSPlane, m_hole_vertice_squared);//added by hgj
	if ((type == SQUARE_BY_ROOMCONTOUR)|| (type == SQUARE_BY_MIN_LOSS)){
		    std::cout << "@RoomContourSquaredJson.size():" << RoomContourSquaredJson.size() << " pts0.size():" << pts0.size() << endl;
			GetSquaredFloorContourPoint3dHlper(RoomContourSquaredJson, pts0, m_floor_contour_squared);//added by hgj
			RoomSquaredJson3d = RoomContourSquaredJson3d;
	}else if(type == SQUARE_BY_CUBOID) {
		    std::cout << "@CuboidContourSquaredJson.size():" << CuboidContourSquaredJson.size() << " pts0.size():" << pts0.size() << endl;
			GetSquaredFloorContourPoint3dHlper(CuboidContourSquaredJson, pts0, m_floor_contour_squared);//added by hgj
			RoomSquaredJson3d =  CuboidContourSquaredJson3d;
	}else if (type == SQUARE_BY_CONVEXITY) {
		    std::cout << "@ConvexityContourSquaredJson.size():" << ConvexityContourSquaredJson.size() << " pts0.size():" << pts0.size() << endl;
			GetSquaredFloorContourPoint3dHlper(ConvexityContourSquaredJson, pts0, m_floor_contour_squared);//added by hgj
			RoomSquaredJson3d = ConvexityContourSquaredJson3d;
	}

#if 1
	std::vector <cv::Point3f> RoomSquaredContourFloat;
	std::vector <cv::Point3i> RoomSquaredContour;
	for (int i = 0; i < contour_squared1.size(); i++)
	{
		RoomSquaredContourFloat.push_back(contour_squared1[i].first);
	}
	RoomSquaredContourFloat = MathOperation::plane_rot(Brotation_matrix, RoomSquaredContourFloat);
	//IOData::SavePoint3fData("RoomSquaredContourRotate.txt", RoomSquaredContour);

	int rdd = static_cast<int>(RoomSquaredContourFloat.size());
	for (int i = 0; i < RoomSquaredContourFloat.size(); i++)
	{
		cv::Point3i temp;
		temp.x = (RoomSquaredContourFloat[i].x > 0.0) ? (RoomSquaredContourFloat[i].x + 0.5) : (RoomSquaredContourFloat[i].x - 0.5);
		temp.y = (RoomSquaredContourFloat[i].y > 0.0) ? (RoomSquaredContourFloat[i].y + 0.5) : (RoomSquaredContourFloat[i].y - 0.5);
		temp.z = (RoomSquaredContourFloat[i].z > 0.0) ? (RoomSquaredContourFloat[i].z + 0.5) : (RoomSquaredContourFloat[i].z - 0.5);
		RoomSquaredContour.push_back(temp);
#ifdef Debug_05
		cout <<"RoomSquaredContour["<<i<<"]:"<< RoomSquaredContourFloat[i]<<"  "<<temp<<endl;
#endif
	}
	//(x1 - x3)*(y2 - y3) -
	//(y1 - y3)*(x2 - x3)
	int da = 0;
	int dm = 0;
	if (rdd >= 3) {
		for (int k = 0; k < rdd - 2; k++)
		{
			bool first_adj = false;
			if (k == 0) first_adj = true;
			if (abs(RoomSquaredContour[k].x - RoomSquaredContour[(k + 1) % rdd].x) > abs(RoomSquaredContour[k].y - RoomSquaredContour[(k + 1) % rdd].y))
			{
				if (RoomSquaredContour[k].x < RoomSquaredContour[(k + 1) % rdd].x)
				{ //u
					int re = (RoomSquaredContour[(k + 1) % rdd].x - RoomSquaredContour[k].x) % 5;
					if (re != 0) {
						if (RoomSquaredContour[(k + 1) % rdd].y < RoomSquaredContour[(k + 2) % rdd].y)
						{
#ifdef Debug_05
							cout << "k:" << k  <<" "<< RoomSquaredContour[k]  <<" UU  x+" << (5 - re) << " " << RoomSquaredContour[(k + 1) % rdd].x - RoomSquaredContour[k].x  << endl;
#endif
							if (first_adj && ((5 - re) != 1) &&(RoomSquaredContour[k].y < RoomSquaredContour[rdd - 1].y))
							{
								if ((5 - re) == 4) { da = dm = 2; }
								if ((5 - re) == 3) { da = 2; dm = 1; }
								if ((5 - re) == 2) { da = 1; dm = 1; }
								RoomSquaredContour[(k + 1) % rdd].x += da;
								RoomSquaredContour[(k + 2) % rdd].x += da;
								RoomSquaredContour[k].x -= dm;
								RoomSquaredContour[rdd - 1].x -= dm;
							}
							else {
								RoomSquaredContour[(k + 1) % rdd].x += (5 - re);
								RoomSquaredContour[(k + 2) % rdd].x += (5 - re);
							}
						}
						else {
#ifdef Debug_05
							cout << "k:" << k << " " << RoomSquaredContour[k] << " UD  x-" << re << " " << RoomSquaredContour[(k + 1) % rdd].x - RoomSquaredContour[k].x << endl;
#endif
							if (first_adj && (re != 1) && (RoomSquaredContour[k].y > RoomSquaredContour[rdd - 1].y))
							{
								if (re == 4) { da = dm = 2; }
								if (re == 3) { da = 2; dm = 1; }
								if (re == 2) { da = 1; dm = 1; }
								RoomSquaredContour[(k + 1) % rdd].x -= da;
								RoomSquaredContour[(k + 2) % rdd].x -= da;
								RoomSquaredContour[k].x += dm;
								RoomSquaredContour[rdd - 1].x += dm;
							}
							else {
								RoomSquaredContour[(k + 1) % rdd].x -= re;
								RoomSquaredContour[(k + 2) % rdd].x -= re;
							}
						}
					}
					else {
#ifdef Debug_05
						cout << "k:" << k << " " << RoomSquaredContour[k] << " :" << RoomSquaredContour[(k + 1) % rdd].x - RoomSquaredContour[k].x << endl;
#endif
					}
				}
				else {
					int re = (RoomSquaredContour[k].x - RoomSquaredContour[(k + 1) % rdd].x) % 5;
					if (re != 0) {
						if (RoomSquaredContour[(k + 1) % rdd].y < RoomSquaredContour[(k + 2) % rdd].y) {
#ifdef Debug_05
							cout <<"k:" << k << " " << RoomSquaredContour[k] << " DU  x-" << re << " " << RoomSquaredContour[k].x - RoomSquaredContour[(k + 1) % rdd].x << endl;
#endif
							if (first_adj && (re != 1) && (RoomSquaredContour[k].y < RoomSquaredContour[rdd - 1].y))
							{
								if (re == 4) { da = dm = 2; }
								if (re == 3) { da = 2; dm = 1; }
								if (re == 2) { da = 1; dm = 1; }
								RoomSquaredContour[(k + 1) % rdd].x += da;
								RoomSquaredContour[(k + 2) % rdd].x += da;
								RoomSquaredContour[k].x -= dm;
								RoomSquaredContour[rdd - 1].x -= dm;
							}
							else {
								RoomSquaredContour[(k + 1) % rdd].x += re;
								RoomSquaredContour[(k + 2) % rdd].x += re;
							}
						}
						else {
#ifdef Debug_05
							cout <<"k:" << k << " " << RoomSquaredContour[k] << " DD  x+" << (5 - re) << " " << RoomSquaredContour[k].x - RoomSquaredContour[(k + 1) % rdd].x  << endl;
#endif
							if (first_adj && ((5 - re) != 1) && (RoomSquaredContour[k].y > RoomSquaredContour[rdd - 1].y))
							{
								if ((5 - re) == 4) { da = dm = 2; }
								if ((5 - re) == 3) { da = 2; dm = 1; }
								if ((5 - re) == 2) { da = 1; dm = 1; }
								RoomSquaredContour[(k + 1) % rdd].x -= da;
								RoomSquaredContour[(k + 2) % rdd].x -= da;
								RoomSquaredContour[k].x += dm;
								RoomSquaredContour[rdd - 1].x += dm;
							}
							else {
								RoomSquaredContour[(k + 1) % rdd].x -= (5 - re);
								RoomSquaredContour[(k + 2) % rdd].x -= (5 - re);
							}
						}
					}
					else {
#ifdef Debug_05
						cout << "k:" << k << " " << RoomSquaredContour[k] << ":" << RoomSquaredContour[k].x - RoomSquaredContour[(k + 1) % rdd].x << endl;
#endif
					}
				}
			}
			else {
				if (RoomSquaredContour[k].y < RoomSquaredContour[(k + 1) % rdd].y)
				{//l
					int re = (RoomSquaredContour[(k + 1) % rdd].y - RoomSquaredContour[k].y) % 5;
					if (re != 0) {
						if (RoomSquaredContour[(k + 1) % rdd].x < RoomSquaredContour[(k + 2) % rdd].x) {
#ifdef Debug_05
							cout << "k:" << k << " " << RoomSquaredContour[k] << " LR  y-" << re << " " << RoomSquaredContour[(k + 1) % rdd].y - RoomSquaredContour[k].y << endl;
#endif
							if (first_adj && (re != 1) && (RoomSquaredContour[k].x < RoomSquaredContour[rdd - 1].x))
							{
								if (re == 4) { da = dm = 2; }
								if (re == 3) { da = 2; dm = 1; }
								if (re == 2) { da = 1; dm = 1; }
								RoomSquaredContour[(k + 1) % rdd].y -= da;
								RoomSquaredContour[(k + 2) % rdd].y -= da;
								RoomSquaredContour[k].y += dm;
								RoomSquaredContour[rdd - 1].y += dm;
							}
							else {
								RoomSquaredContour[(k + 1) % rdd].y -= re;
								RoomSquaredContour[(k + 2) % rdd].y -= re;
							}
						}
						else {
#ifdef Debug_05
							cout << "k:" << k << " " << RoomSquaredContour[k] << " LL  y+" << (5 - re) << " " << RoomSquaredContour[(k + 1) % rdd].y - RoomSquaredContour[k].y  << endl;
#endif
							if (first_adj && ((5 - re) != 1) && (RoomSquaredContour[k].x > RoomSquaredContour[rdd - 1].x))
							{
								if ((5 - re) == 4) { da = dm = 2; }
								if ((5 - re) == 3) { da = 2; dm = 1; }
								if ((5 - re) == 2) { da = 1; dm = 1; }
								RoomSquaredContour[(k + 1) % rdd].y += da;
								RoomSquaredContour[(k + 2) % rdd].y += da;
								RoomSquaredContour[k].y -= dm;
								RoomSquaredContour[rdd - 1].y -= dm;
							}
							else {
								RoomSquaredContour[(k + 1) % rdd].y += (5 - re);
								RoomSquaredContour[(k + 2) % rdd].y += (5 - re);
							}
						}
					}
					else {
#ifdef Debug_05
						cout << "k:" << k << " " << RoomSquaredContour[k] << " :" << RoomSquaredContour[(k + 1) % rdd].y - RoomSquaredContour[k].y << endl;
#endif
					}
				}
				else {//r
					int re = (RoomSquaredContour[k].y - RoomSquaredContour[(k + 1) % rdd].y) % 5;
					if (re != 0) {
						if (RoomSquaredContour[(k + 1) % rdd].x < RoomSquaredContour[(k + 2) % rdd].x) {
#ifdef Debug_05
							cout << "k:" << k << " " << RoomSquaredContour[k] << " RR  y+" << (5 - re) << " " << RoomSquaredContour[k].y - RoomSquaredContour[(k + 1) % rdd].y  << endl;
#endif
							if (first_adj && ((5 - re) != 1) && (RoomSquaredContour[k].x < RoomSquaredContour[rdd - 1].x))
							{
								if ((5 - re) == 4) { da = dm = 2; }
								if ((5 - re) == 3) { da = 2; dm = 1; }
								if ((5 - re) == 2) { da = 1; dm = 1; }
								RoomSquaredContour[(k + 1) % rdd].y -= da;
								RoomSquaredContour[(k + 2) % rdd].y -= da;
								RoomSquaredContour[k].y += dm;
								RoomSquaredContour[rdd - 1].y += dm;
							}
							else {
								RoomSquaredContour[(k + 1) % rdd].y -= (5 - re);
								RoomSquaredContour[(k + 2) % rdd].y -= (5 - re);
							}
						}
						else {
#ifdef Debug_05
							cout << "k:" << k << " " << RoomSquaredContour[k] << " RL  y-" << re << " " << RoomSquaredContour[k].y - RoomSquaredContour[(k + 1) % rdd].y  << endl;
#endif
							if (first_adj && (re != 1) && (RoomSquaredContour[k].x > RoomSquaredContour[rdd - 1].x))
							{
								if (re == 4) { da = dm = 2; }
								if (re == 3) { da = 2; dm = 1; }
								if (re == 2) { da = 1; dm = 1; }
								RoomSquaredContour[(k + 1) % rdd].y += da;
								RoomSquaredContour[(k + 2) % rdd].y += da;
								RoomSquaredContour[k].y -= dm;
								RoomSquaredContour[rdd - 1].y -= dm;
							}
							else {
								RoomSquaredContour[(k + 1) % rdd].y += re;
								RoomSquaredContour[(k + 2) % rdd].y += re;
							}
						}
					}
					else {
#ifdef Debug_05
						cout << "k:" << k << " " << RoomSquaredContour[k] << " :" << RoomSquaredContour[k].y - RoomSquaredContour[(k + 1) % rdd].y << endl;
#endif
					}
				}
			}
		}
	}
	else {
		std::cout << "[SquareMode] skip 5-grid snapping: RoomSquaredContour vertex count is " << rdd << std::endl;
	}

#if 0//def Debug_05
	for (int k = 0; k < RoomSquaredContour.size(); k++)
	{
		if (abs(RoomSquaredContour[k].x - RoomSquaredContour[(k + 1) % rdd].x) > abs(RoomSquaredContour[k].y - RoomSquaredContour[(k + 1) % rdd].y))
		{
			cout << RoomSquaredContour[k];
			if (RoomSquaredContour[k].x < RoomSquaredContour[(k + 1) % rdd].x)
			{
				int len = RoomSquaredContour[(k + 1) % rdd].x - RoomSquaredContour[k].x;
				cout << "k: " << k << " U " << len << " " << len % 5 << endl;
			}
			else {
				int len = RoomSquaredContour[k].x - RoomSquaredContour[(k + 1) % rdd].x;
				cout << "k: " << k << " D " <<  len << " " << len % 5 << endl;
			}
		}
		else {
			cout << RoomSquaredContour[k];
			if (RoomSquaredContour[k].y < RoomSquaredContour[(k + 1) % rdd].y)
			{
				int len = RoomSquaredContour[(k + 1) % rdd].y - RoomSquaredContour[k].y;
				cout << "k: " << k << " L " << len << " " << len % 5 << endl;
			}
			else {
				int len = RoomSquaredContour[k].y - RoomSquaredContour[(k + 1) % rdd].y;
				cout << "k:" << k << " R "  << len  <<" "<< len % 5 << endl;
			}
		}
	}
#endif
	RoomSquaredContourFloat.clear();
	for (int i = 0; i < RoomSquaredContour.size(); i++){
		RoomSquaredContourFloat.push_back((cv::Point3f)RoomSquaredContour[i]);
	}
#ifdef Debug_05
	for (int i = 0; i < RoomSquaredContourFloat.size(); i++) {
		cout << "RoomSquaredContourFloat["<<i<<"]: " << RoomSquaredContourFloat[i] << endl;
	}
#endif
	contour_squared05 =  MathOperation::plane_rot(b_mat_to_y, RoomSquaredContourFloat);
#if 1 //def Debug_05
	if (!contour_squared05.empty()) {
		const int contour_squared05_size = static_cast<int>(contour_squared05.size());
		for (int i = 0; i < contour_squared05_size; i++) {
			contour_squared05_len.push_back(sqrt(std::pow(contour_squared05[i].x - contour_squared05[(i + 1) % contour_squared05_size].x, 2.0) +
				std::pow(contour_squared05[i].y - contour_squared05[(i + 1) % contour_squared05_size].y, 2.0)));
			std::cout << "i: " << i << " len:" << sqrt(std::pow(contour_squared05[i].x - contour_squared05[(i + 1) % contour_squared05_size].x, 2.0) +
				                                   std::pow(contour_squared05[i].y - contour_squared05[(i + 1) % contour_squared05_size].y, 2.0)) << endl;
		}
	}
#endif
#endif

	return RoomSquaredJson3d;
}

bool RoomMeasurement::HasOneMeterLine(void)
{
	return mHasOneMeterLine;
}

bool RoomMeasurement::HasGroundAxisLine(void)
{
	return mHasGroundAxisLine;
}

//=========added by hgj========start========//
void RoomMeasurement::GetContourAndHolesSquared(std::vector<cv::Point3f>&floor_contour_squared, std::vector<std::vector<std::vector<cv::Point3f>>>&hole_vertice_squared)
{
	floor_contour_squared = m_floor_contour_squared;
	hole_vertice_squared = m_hole_vertice_squared;
}

bool RoomMeasurement::GetSquaredFloorContourPoint3dHlper(const std::vector <std::pair< cv::Point2f, cv::Point2f>>& CuboidContourSquaredJson,
	const std::vector<cv::Point3f>& coutour_ori,
	std::vector<cv::Point3f>& countour_squared)
{
	if (CuboidContourSquaredJson.size() == 0 || coutour_ori.size() == 0)
	{
		std::cout << " GetSquaredPoint3dHlper CuboidContourSquaredJson.size() or coutour_ori.size() is Zero!" << std::endl;
		return false;
	}
	if (CuboidContourSquaredJson.size() != coutour_ori.size())
	{
		std::cout << " GetSquaredPoint3dHlper CuboidContourSquaredJson.size()!= coutour_ori.size() false!" << std::endl;
		return false;
	}

	countour_squared = coutour_ori;
	for (int i = 0; i<CuboidContourSquaredJson.size(); i++)
	{
		countour_squared[i].x = CuboidContourSquaredJson[i].first.x;
		countour_squared[i].y = CuboidContourSquaredJson[i].first.y;
	}
	return true;
}

bool RoomMeasurement::GetSquaredHolePoint3dHlper(const std::vector<StructuredPlane>& mSPlane,
	std::vector<std::vector<std::vector<cv::Point3f>>>& vec_vec_vertice_s)
{
	if (mSPlane.size() == 0)
	{
		std::cout << " GetSquaredHolePoint3dHlper mSPlane.size() is Zero!" << std::endl;
		return false;
	}

	vec_vec_vertice_s.resize(mSPlane.size());
	for (int i = 0; i<mSPlane.size(); i++)
	{
		vec_vec_vertice_s[i].resize(mSPlane[i].holes.size());
		for (int j = 0; j<mSPlane[i].holes.size(); j++)
		{
			vec_vec_vertice_s[i][j] = mSPlane[i].holes[j].vertice_s;
		}
	}
	return true;
}
