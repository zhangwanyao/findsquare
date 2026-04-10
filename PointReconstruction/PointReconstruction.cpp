#include "PointReconstruction.h"
#include "PlaneSegmentationEx.h"
#include "util_plane_fit.hpp"
#include "util_time.hpp"
#include "DownSampleFilter.hpp"
#include "in_out_data.hpp"
#include "util_log.hpp"
#include "util_UNRE.hpp"
#include "util_normalEst.hpp"
#include <vector>
#include <cmath>
#include <limits>
#include <iostream>
#include <opencv2/core.hpp>
#include "InOutData.h"
#include "PlaneCuttingEx.h"
#include <CGAL/Simple_cartesian.h> 
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include "util_math.hpp"
#include "codec.h"
#include <opencv2/core/eigen.hpp>
#include "Measurement/MeasureDefect.h"
#include "../Common/buildRefectImgFuc.h"
#include "Measurement/MeasureBase.h"
#include "DetectObstacle.h"
#include "../DoorWindowDimension/include/util_line_tool.hpp"
#include "../MultiStage/StageMatrix.h"
#include "PointReconstructionGetConfig.h"


//#define USE_NEW_HOLE_ALGO
//#define DBG_HOLE_ALGO
//#define DBG_PRE_SEG
#define DEBUG_CORNERS_TEST false
#ifdef DBG_HOLE_ALGO
extern std::string filePath;
extern std::ofstream resultA;
#endif

using namespace Eigen;
std::string cloudPointsFileName;

PointReconstruction::PointReconstruction():
	mpReconstruction(NULL),
	mFakeCeiling(false)
{
	
}


PointReconstruction::~PointReconstruction()
{
}


/**
 * @brief 裁剪结果可视化验证
 * @param original_points 裁剪前的原始3D点云（完整数据）
 * @param filtered_points 裁剪后的筛选3D点云（有效数据）
 * @param polygon_xy      裁剪所用的多边形顶点（IntPoint2D格式）
 * @param save_path       可视化图片保存路径（默认当前目录）
 * @param canvas_size     画布尺寸（默认2000x2000高清尺寸）
 */
void PointReconstruction::VisualizeClippingResult(
	const ModuleStruct::Point3fArray& original_points,
	const ModuleStruct::Point3fArray& filtered_points,
	const std::vector<FloatPoint2D>& polygon_xy,
	const std::string& save_path,
	const cv::Size& canvas_size)
{
	if (original_points.empty() && filtered_points.empty()) {
		std::cerr << "[VisualizeError] 原始点云和裁剪点云均为空，无法可视化！" << std::endl;
		return;
	}
	if (polygon_xy.size() < 3) {
		std::cerr << "[VisualizeError] 多边形顶点数不足3个，无法绘制裁剪轮廓！" << std::endl;
		return;
	}

	float minX = FLT_MAX, minY = FLT_MAX;
	float maxX = -FLT_MAX, maxY = -FLT_MAX;

	auto updateMinMax = [&](float x, float y) {
		minX = (std::min)(minX, x);
		maxY = (std::max)(maxY, y);
		maxX = (std::max)(maxX, x);
		minY = (std::min)(minY, y);
		};

	for (const auto& p : original_points) { updateMinMax(p.x, p.y); }
	for (const auto& p : polygon_xy) { updateMinMax(p.x, p.y); }


	float rangeX = maxX - minX;
	float rangeY = maxY - minY;
	float pad = 0.05f * (std::max)(rangeX, rangeY);
	minX -= pad; maxX += pad;
	minY -= pad; maxY += pad;

	if (rangeX < 1e-6) { minX -= 1.0f; maxX += 1.0f; }
	if (rangeY < 1e-6) { minY -= 1.0f; maxY += 1.0f; }


	auto world2Canvas = [&](float x, float y) -> cv::Point {
		float u = (x - minX) / (maxX - minX) * canvas_size.width;
		float v = canvas_size.height - (y - minY) / (maxY - minY) * canvas_size.height;
		return cv::Point(static_cast<int>(u), static_cast<int>(v));
		};


	cv::Mat vis_img(canvas_size, CV_8UC3, cv::Scalar(255, 255, 255)); 

	cv::Scalar color_original = cv::Scalar(180, 180, 180); // 灰色
	for (const auto& p : original_points) {
		cv::Point canvas_p = world2Canvas(p.x, p.y);
		cv::circle(vis_img, canvas_p, 2, color_original, -1); 
	}

	cv::Scalar color_filtered = cv::Scalar(0, 200, 0); // 深绿色，醒目
	for (const auto& p : filtered_points) {
		cv::Point canvas_p = world2Canvas(p.x, p.y);
		cv::circle(vis_img, canvas_p, 3, color_filtered, -1); // 实心圆，半径3（比原始点大）
	}

	cv::Scalar color_polygon = cv::Scalar(255, 0, 0); // 蓝色
	std::vector<cv::Point> polygon_canvas;
	for (const auto& p : polygon_xy) {
		polygon_canvas.push_back(world2Canvas(p.x, p.y));
	}

	if (!polygon_canvas.empty() && polygon_canvas.front() != polygon_canvas.back()) {
		polygon_canvas.push_back(polygon_canvas.front());
	}

	cv::polylines(vis_img, polygon_canvas, true, color_polygon, 3, cv::LINE_AA);


	bool save_ok = cv::imwrite(save_path, vis_img);
	if (save_ok) {
		std::cout << "[裁剪可视化成功] 验证图已保存至：" << save_path << std::endl;
		std::cout << "[裁剪统计] 原始点数量：" << original_points.size()
			<< " | 裁剪后点数量：" << filtered_points.size() << std::endl;
	}
	else {
		std::cerr << "[裁剪可视化失败] 图片保存路径无效，请检查权限！" << std::endl;
	}
}

/**
 * @brief 判断单个Point3f点是否在二维闭合多边形内
 * @param point 待判断的点（OpenCV Point3f）
 * @param polygon 闭合多边形顶点列表（OpenCV Point2f组成的vector，按顺时针/逆时针排列）
 * @return true-点在多边形内，false-点在多边形外
 */
bool PointReconstruction::isPointInPolygon(const cv::Point3f& point, const std::vector<cv::Point2f>& polygon) {
	int intersectionCount = 0; 
	size_t vertexNum = polygon.size();
	if (vertexNum < 3) { 
		std::cerr << "Error: 多边形顶点数不能少于3个！" << std::endl;
		return false;
	}

	for (size_t i = 0; i < vertexNum; ++i) {
		const cv::Point2f& start = polygon[i];
		const cv::Point2f& end = polygon[(i + 1) % vertexNum];

		cv::Point2f low = start;
		cv::Point2f high = end;
		if (low.y > high.y) {
			std::swap(low, high);
		}

		if (point.y < low.y || point.y > high.y) {
			continue;
		}

		if (cv::abs(high.y - low.y) < 1e-6) { 
			continue;
		}

		float t = (point.y - low.y) / (high.y - low.y);
		float xIntersection = low.x + t * (high.x - low.x);

		if (xIntersection <= point.x + 1e-6) { 
			intersectionCount++;
		}
	}
	return (intersectionCount % 2) == 1;
}

bool PointReconstruction::ReadPoints(std::string file_path, ModuleStruct::Point3fArray &points, std::vector<unsigned char> &  reflectance)
{

#ifdef DBG_HOLE_ALGO
		system(("rd /S /Q " + std::string("dll_log")).c_str());
	IOData::createDirectory(filePath + std::string("DetectHole3\\").c_str());
#endif
	if (file_path.find("\\") != string::npos)
		cloudPointsFileName = file_path.substr(file_path.rfind("\\") + 1);
	else
		cloudPointsFileName = file_path.substr(file_path.rfind("/") + 1); // added by zhujunqing

	std::string file_ext = file_path.substr(file_path.find_first_of('.'));

	//std::cout << "[station info path]: " << file_ext << "\n";
	if (file_ext == ".ptsen")
	{
		CloudPointCodec dodec;
		std::vector<int> reflectance_int;
		dodec.Decode(file_path, points, reflectance_int);
		if (points.size() == 0)
		{
			s_ReconstructionErrorCode = RECONSTRUCTION_LOAD_ERROR;
			return false;
		}
		reflectance.resize(reflectance_int.size());
		for (int i = 0; i < reflectance_int.size(); i++)
			reflectance[i] = reflectance_int[i];
	}
	else if(file_ext == ".pts")
	{
		std::vector<float>* intensity = new std::vector<float>;

		points = IOData::readFromXyz(file_path, true, intensity);
		for (int i = 0; i < points.size(); i++)
		{
			//scene_xyz.emplace_back(pair_cloud_reflect[i].first);
			reflectance.emplace_back(int((*intensity)[i]));
		}
		delete intensity;
		intensity = nullptr;
	}
	else
		if (util_plane_seg::load_input_data(1, file_path, points, reflectance) < 0)
		{
			s_ReconstructionErrorCode = RECONSTRUCTION_LOAD_ERROR;
			return false;
		}

	//remove zero
	ModuleStruct::Point3fArray points_tmp;
	std::vector<unsigned char>     reflectance_tmp;
	for (int i = 0; i < points.size(); i++)
	{
		if (reflectance[i] > 0)
		{
			points_tmp.push_back(points[i]);
			reflectance_tmp.push_back(reflectance[i]);
		}
	}
	points.swap(points_tmp);
	reflectance.swap(reflectance_tmp);

	points_tmp.clear();
	points_tmp.shrink_to_fit();

	reflectance_tmp.clear();
	reflectance_tmp.shrink_to_fit();
}

bool PointReconstruction::PrePlaneSegmentation(ModuleStruct::Point3fArray &points_cut, std::vector<unsigned char> &reflectance_cut, std::vector<cv::Mat> RTs, std::vector<std::vector<float>> station_pos, float mergeDataDir)
{

	ModuleStruct::Point3fArray points = points_cut;
	std::vector<unsigned char> reflectance = reflectance_cut;
	std::vector<IntPoint2D> polypt_uv = GetPolypt_uv();
	float totalDir = mergeDataDir;
	if (std::fabs(totalDir) < 0.1) totalDir = 180.0f;
	int roomIdx = 0;
	cv::Mat ref_mat_rt = RTs[roomIdx];
	double mapXy2dFactor = 0.02;

	VectorXd anchor_vec(station_pos[roomIdx].size());
	for (size_t i = 0; i < station_pos[roomIdx].size(); ++i) {
		anchor_vec(i) = station_pos[roomIdx][i];
	}
#ifdef DBG_PRE_SEG
	cout << "mergeDataDir: " << totalDir << endl;
	cout << "station.position: " << anchor_vec[0] << " " << anchor_vec[1] << endl;
	cout << "RT: " << ref_mat_rt << endl;
#endif
	Eigen::MatrixXd Rt = Eigen::MatrixXd::Zero(4, 4);
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			Rt(i, j) = ref_mat_rt.at<float>(i, j);
		}
	}

	std::vector<cv::Point2f> polypt_maped;
	double cosCompass = std::cos(M_PI + totalDir * M_PI / 180);
	double sinCompass = std::sin(M_PI + totalDir * M_PI / 180);
	VectorXd XYZ0(4);
	XYZ0 << 0, 0, 0, 1;
	VectorXd guideWithRt0 = Rt * XYZ0 / mapXy2dFactor;
	double guideWithCompass_x0 = sinCompass * guideWithRt0[0] + cosCompass * guideWithRt0[1];
	double guideWithCompass_y0 = cosCompass * guideWithRt0[0] - sinCompass * guideWithRt0[1];
	double shiftX = anchor_vec[0] - guideWithCompass_x0;
	double shiftY = anchor_vec[1] - guideWithCompass_y0;

	for (int i = 0; i < polypt_uv.size(); i++) {
		double x = polypt_uv[i].x;
		double y = polypt_uv[i].y;
		double x0 = mapXy2dFactor * (sinCompass * (x - shiftX) + cosCompass * (y - shiftY));
		double y0 = mapXy2dFactor * (cosCompass * (x - shiftX) - sinCompass * (y - shiftY));
		VectorXd XY(4);
		XY << x0, y0, 0, 1;
		//VectorXd pt3d = Rt.inverse() * XY;
		VectorXd pt3d = XY;
		polypt_maped.push_back(cv::Point2f(pt3d(0), pt3d(1)));
	}

	CvMat* myPolyc = cvCreateMat(1, polypt_maped.size(), CV_32FC2);
	for (int t = 0; t < polypt_maped.size(); t++) {
		cvSet1D(myPolyc, t, CvScalar(polypt_maped[t].x, polypt_maped[t].y));
	}

	points_cut.clear();
	reflectance_cut.clear();
	for (int f = 0; f < points.size(); f++) {
		//	cout << f << " ----------------points:" << points[f] << endl;
		cv::Point2f testp = Point2f(points[f].x, points[f].y);
		if (cvPointPolygonTest(myPolyc, testp, true) > 0) {
			points_cut.push_back(points[f]);
			reflectance_cut.push_back(reflectance[f]);
		}
	}
#ifdef DBG_PRE_SEG
	std::cout << "points_cut time = " << GetTickCount() - dwStart << "  points.size():" << points.size()
		<< "  points_cut.size():" << points_cut.size() << std::endl;
	IOData::SavePoint3fData(mDebugDir + "points_cut" + to_string(index_cut) + ".txt", points_cut);
#endif
	std::string save_path = "dll_log/Clipping.png";
	float minX = FLT_MAX, minY = FLT_MAX;
	float maxX = -FLT_MAX, maxY = -FLT_MAX;

	auto updateMinMax = [&](float x, float y) {
		if (x < minX) minX = x;
		if (x > maxX) maxX = x;
		if (y < minY) minY = y;
		if (y > maxY) maxY = y;
	};

	for (auto& p : polypt_maped) {
		updateMinMax(p.x, p.y);
	}
	for (auto& p : points)
	{
		updateMinMax(p.x, p.y);
	}

	float rangeX = maxX - minX;
	float rangeY = maxY - minY;
	float pad = 0.05f * (std::max)(rangeX, rangeY);
	minX -= pad;
	maxX += pad;
	minY -= pad;
	maxY += pad;

	int imgW = 2000;
	int imgH = 2000;
	cv::Mat img(imgH, imgW, CV_8UC3, cv::Scalar(255, 255, 255));
	
	auto worldToImg = [&](float x, float y) -> cv::Point {
		float u = (x - minX) / (maxX - minX) * imgW;
		float v = imgH - (y - minY) / (maxY - minY) * imgH;
		return cv::Point(static_cast<int>(u), static_cast<int>(v));
	};

	// 绘制所有点（灰色）
	for (auto& p : points)
	{
		cv::Point ip = worldToImg(p.x, p.y);
		cv::circle(img, ip, 2, cv::Scalar(180, 180, 180), -1);
	}

	// 绘制裁剪点（绿色）
	for (auto& p : points_cut) {
		cv::Point pt = worldToImg(p.x, p.y);
		cv::circle(img, pt, 3, cv::Scalar(0, 200, 0), -1);
	}

	// 绘制多边形（蓝色）
	std::vector<cv::Point> polyImg;
	for (auto& p : polypt_maped)
		polyImg.push_back(worldToImg(p.x, p.y));

	// 自动闭合
	if (polyImg.front() != polyImg.back())
		polyImg.push_back(polyImg.front());

	cv::polylines(img, polyImg, true, cv::Scalar(255, 0, 0), 3);

	// 保存
	cv::imwrite(save_path, img);
	log_info("[VisualizeClipping] PNG 图像已保存到: %s", save_path.c_str());
	//std::cout << "[VisualizeClipping] PNG 图像已保存到: " << save_path << std::endl;
	return true;
}


bool PointReconstruction::RestorePlaneToOriginalCoordinate(PlaneCutResultInterface& mPlane, const cv::Mat& relaRT)
{
	std::cout << "relaRT type: " << relaRT.type() << " (CV_32F=" << CV_32F << ", CV_64F=" << CV_64F << ")" << std::endl;
	cv::Mat mat_identity = (relaRT.rows == 3) ? cv::Mat::eye(3, 3, relaRT.type()) : cv::Mat::eye(4, 4, relaRT.type());
	double diff = cv::norm(relaRT - mat_identity);
	if (diff < 1e-6) {
		std::cout << "警告：relaRT是单位矩阵，变换后坐标不变！" << std::endl;
	}

	if (!mPlane.plane_center.empty())
	{
		std::vector<cv::Point3f> tempCenter;
		if (!MeasureBase::RotatePoints(mPlane.plane_center, relaRT, tempCenter))
		{
			std::cerr << "Failed to restore plane_center to original coordinate!" << std::endl;
			return false;
		}
		mPlane.plane_center = std::move(tempCenter);
	}

	if (!mPlane.plane_normals.empty())
	{
		std::vector<cv::Point3f> tempNormals;
		cv::Mat rotationMatrixForNormal; 

		if (relaRT.rows == 3 && relaRT.cols == 3)
		{
			rotationMatrixForNormal = relaRT.clone();
		}
		else if (relaRT.rows == 4 && relaRT.cols == 4)
		{
			rotationMatrixForNormal = relaRT(cv::Rect(0, 0, 3, 3)).clone();
		}
		else
		{
			std::cerr << "Invalid matrix size for normal transformation!" << std::endl;
			return false;
		}

		if (!MeasureBase::RotatePoints(mPlane.plane_normals, rotationMatrixForNormal, tempNormals))
		{
			std::cerr << "Failed to restore plane_normals to original coordinate!" << std::endl;
			return false;
		}
		mPlane.plane_normals = std::move(tempNormals);
	}

	for (size_t pid = 0; pid < mPlane.plane_xyz.size(); ++pid)
	{
		auto& src = mPlane.plane_xyz[pid];
		if (src.empty())
			continue;

		/*std::cout << "pid=" << pid << " 变换前：(" << src[0].x << "," << src[0].y << "," << src[0].z << ")" << std::endl;
		std::cout << "pid=" << pid << " 变换前：(" << src[1].x << "," << src[1].y << "," << src[1].z << ")" << std::endl;
		std::cout << "pid=" << pid << " 变换前：(" << src[2].x << "," << src[2].y << "," << src[2].z << ")" << std::endl;*/

		std::vector<cv::Point3f> dst;
		if (!MeasureBase::RotatePoints(src, relaRT, dst))
		{
			std::cerr << "RotatePoints failed at pid=" << pid << std::endl;
			return false;
		}
		/*std::cout << "pid=" << pid << " 变换后：(" << dst[0].x << "," << dst[0].y << "," << dst[0].z << ")" << std::endl;
		std::cout << "pid=" << pid << " 变换后：(" << dst[1].x << "," << dst[1].y << "," << dst[1].z << ")" << std::endl;
		std::cout << "pid=" << pid << " 变换后：(" << dst[2].x << "," << dst[2].y << "," << dst[2].z << ")" << std::endl;*/
		src = std::move(dst);
	}

	for (size_t i = 0; i < mPlane.door_window_info.size(); ++i)
	{
		for (size_t j = 0; j < mPlane.door_window_info[i].size(); ++j)
		{
			MeasureDoorWindow::DoorWindowInfo& dwInfo = mPlane.door_window_info[i][j];
			if (!dwInfo.corners.empty())
			{
				std::vector<cv::Point3f> tempDwCorners; 
				if (!MeasureBase::RotatePoints(dwInfo.corners, relaRT, tempDwCorners))
				{
					std::cerr << "Failed to restore door_window_info[" << i << "][" << j << "] corners to original coordinate!" << std::endl;
					return false;
				}
				dwInfo.corners = std::move(tempDwCorners);
			}
		}
	}
	std::cout << "mPlane has been restored to original coordinate successfully!" << std::endl;
	return true;
}



bool PointReconstruction::PlaneSegmentation(std::vector<std::string> file_path, std::vector<cv::Mat> RTs, cv::Mat relaRT, MEASUREMENT_MODE seg_mode,
	std::vector<std::vector<float>> station_pos, float mergeDataDir, std::string compass_file, float compass_value, bool generate_contour)
{
	// 点云读取旋转合并
	ModuleStruct::Point3fArray points;
	std::vector<unsigned char>     reflectance;
	log_info("[file_path size]: %d", file_path.size());
	DWORD dwStartTotal = GetTickCount();
	for (int i = 0; i < file_path.size(); i++)
	{		
		ModuleStruct::Point3fArray points_t, points_t_r;
		std::vector<unsigned char>     reflectance_t;
		log_info("[readpoints]: %s", file_path[i].c_str());
		ReadPoints(file_path[i], points_t, reflectance_t);
		//rotate points
		std::cout << "Read points size = " << points_t.size() << "\n";
		cv::Mat mat_rt = RTs[i];
		
		std::cout << "mat_rt" << i << ": "  << mat_rt << "\n";
		MeasureBase::RotatePoints(points_t, mat_rt, points_t_r);

		points.insert(points.end(), points_t_r.begin(), points_t_r.end());
		reflectance.insert(reflectance.end(), reflectance_t.begin(), reflectance_t.end());		
	}


	DWORD dwEndTotal = GetTickCount();
	log_info("Read and transform points time = %lu ms. \n points.size():%zu", dwEndTotal - dwStartTotal, points.size());
	
	if (GetPolypt_xy().size() > 3)
	{
		cut_poly = true;
		std::vector<FloatPoint2D> polypt_xy = GetPolypt_xy();
		std::vector<cv::Point2f> polygon_points;

		ModuleStruct::Point3fArray filtered_points;
		std::vector<unsigned char> filtered_reflectance;

		for (auto polypt : polypt_xy)
		{
			std::cout << "polypt_xy: " << polypt.x << " , " << polypt.y << std::endl;
			polygon_points.emplace_back(cv::Point2f(polypt.x, polypt.y));
		}

		filtered_points.reserve(points.size());
		filtered_reflectance.reserve(reflectance.size());
		for (size_t i = 0; i < points.size(); ++i) {
			if (isPointInPolygon(points[i], polygon_points)) {
				filtered_points.push_back(points[i]);
				filtered_reflectance.push_back(reflectance[i]);
			}
		}

		ModuleStruct::Point3fArray original_points_copy = points;

		points = std::move(filtered_points);
		points.shrink_to_fit();

		reflectance = std::move(filtered_reflectance);
		reflectance.shrink_to_fit();

		VisualizeClippingResult(original_points_copy, points, polypt_xy, "Clipping_Verify.png");
	}
	else if (GetPolypt_uv().size() > 3) {
		log_info("GetPolypt_uv().size(): %zu", GetPolypt_uv().size());
		PrePlaneSegmentation(points, reflectance, RTs, station_pos, mergeDataDir);
		log_info("After PrePlaneSegmentation points.size(): %zu", points.size());
	}
	else {
		log_info("GetPolypt_uv().size(): %zu No PrePlaneSegmentation.", GetPolypt_uv().size());
	}

	


	// 点云单位转换
	DWORD start_prepare = GetTickCount();
	bool ret = util_plane_seg::CheckInputDataUnit(points);
	if (!ret)
	{
		util_plane_seg::unit_m2mm(points);
	}

	IOData::SavePoint3fData("original.pts", points);

	cv::Mat relaRt_inv = relaRT.inv();
	std::cout << "relaRt_inv: " << relaRt_inv << "\n";

	ModuleStruct::Point3fArray points_rt;
	MeasureBase::RotatePoints(points, relaRt_inv, points_rt);
	points = std::move(points_rt);
	points.shrink_to_fit();

	//IOData::SavePoint3fData("relaRt_inv.pts", points);

	if (generate_contour)
	{
		std::string path_dir = file_path[0].substr(0, file_path[0].find("Scanning") + std::string("Scanning").length());
		log_info("========= points.size(): %zu", points.size());
		//std::cout << "========= points.size(): " << points.size() << endl;
		ModuleStruct::Point3fArray points_tmpt;		

		points_tmpt = points;
		std::sort(points_tmpt.begin(), points_tmpt.end(), [](const cv::Point3f& v1, const cv::Point3f& v2) {return (v1.z < v2.z); });

		MatrixXf ground(points_tmpt.size(), 2);
		for (int i = 0; i < points_tmpt.size(); i++) {
			ground(i, 0) = points_tmpt[i].x / 10.0;
			ground(i, 1) = points_tmpt[i].y / 10.0;
		}
		ground.col(1) *= -1;
		mMinx = ground.col(0).minCoeff();
		mMiny = ground.col(1).minCoeff();
		ground.col(0) = (ground.col(0).array() - mMinx).matrix();
		ground.col(1) = (ground.col(1).array() - mMiny).matrix();
		float cols = ground.col(0).maxCoeff();
		float rows = ground.col(1).maxCoeff();

		ground = (ground.array() + 32).matrix();
		log_info("mMinx: %f  mMiny:%f  cols:%f  rows:%f", mMinx, mMiny, cols, rows);
		//std::cout << "mMinx: " << mMinx << "  mMiny:" << mMiny << " cols:" << cols << " rows:" << rows << endl;

		cv::Mat draw_img = cv::Mat(rows + 64, cols + 64, CV_32FC3);
		cv::Mat intensity_img = cv::Mat::zeros(rows + 64, cols + 64, CV_32FC1);
		cv::Mat startZ = cv::Mat(rows + 64, cols + 64, CV_32FC1);
		cv::Mat lastZ = cv::Mat(rows + 64, cols + 64, CV_32FC1);
		cv::Mat countZ = cv::Mat::zeros(rows + 64, cols + 64, CV_32FC1);
		cv::Mat countZ2 = cv::Mat::zeros(rows + 64, cols + 64, CV_32FC3);
		lastZ *= -100000;
		float maxCount = 0;
		for (int i = 0; i < points_tmpt.size(); i++)
		{									
			if (points_tmpt[i].z - lastZ.at<float>(ground(i, 1), ground(i, 0)) > 10.0f) {
				intensity_img.at<float>(ground(i, 1), ground(i, 0)) = std::fmax(intensity_img.at<float>(ground(i, 1), ground(i, 0)), (lastZ.at<float>(ground(i, 1), ground(i, 0)) - startZ.at<float>(ground(i, 1), ground(i, 0))));
				startZ.at<float>(ground(i, 1), ground(i, 0)) = points_tmpt[i].z;
				lastZ.at<float>(ground(i, 1), ground(i, 0)) = points_tmpt[i].z;
				countZ.at<float>(ground(i, 1), ground(i, 0)) = countZ.at<float>(ground(i, 1), ground(i, 0)) + 1;
				maxCount = (countZ.at<float>(ground(i, 1), ground(i, 0)) > maxCount) ? countZ.at<float>(ground(i, 1), ground(i, 0)) : maxCount;
			}
			else {
				lastZ.at<float>(ground(i, 1), ground(i, 0)) = points_tmpt[i].z;
			}			
		}
		cv::Mat diffZ = lastZ - startZ;
		
		//std::cout << draw_img.rows << "," << draw_img.cols << std::endl;
		for (int r = 0; r < draw_img.cols; r++) {
			for (int c = 0; c < draw_img.rows; c++) {			
				float grayValue;
				grayValue = (std::fmax(intensity_img.at<float>(c, r), diffZ.at<float>(c, r))) * 255 / maxCount;
				draw_img.at<cv::Vec3f>(c, r) = cv::Vec3f(grayValue, grayValue, grayValue);
				grayValue = countZ.at<float>(c, r) / maxCount * 255;
				countZ2.at<cv::Vec3f>(c, r) = (countZ.at<float>(c, r) / maxCount > 1) ? cv::Vec3f(0, 255, 0) : cv::Vec3f(grayValue, grayValue, grayValue);
			}
		}
		log_info("maxCount: %f", maxCount);
		//std::cout << "maxCount = " << maxCount << std::endl;
		cv::imwrite(path_dir + "\\generated_contour_count.jpg", countZ2);
		return cv::imwrite(path_dir + "\\generated_contour.jpg", draw_img);
	}
	log_info("0points.size(): %zu,reflectance.size(): %zu", points.size(), reflectance.size());
	DWORD end_prepare = GetTickCount();
	log_info("unit_m2mm and generated_contour time = %lu ms.", end_prepare - start_prepare);

	TIMING_DECLARE(TP1)
	OUT_TIMING_BEGIN(TP1)


	OUT_TIMING_END("Load3DPtCloudData", TP1)

	PlaneSegResultInterface segResult;

	if (!FitPlaneExt(points, reflectance, segResult))
	{
		s_ReconstructionErrorCode = RECONSTRUCTION_PLANE_FIT_ERROR;
		return false;
	}

	/*for (int i = 0; i < segResult.plane_xyz.size(); i++) {
		if (true)
		{
			int wall_id = i;
			std::string save_plane_ply2 = "plane_xyz_" + std::to_string(wall_id) + ".ply";
			IOData::SavePLYPoints3f(save_plane_ply2, segResult.plane_xyz[i], false);
		}
	}*/
	
	int n = 0;
	std::vector<int> save_pts_idx;
	for (size_t i = 0; i < segResult.plane_xyz.size();i++)
	{
		if (segResult.plane_xyz[i].size() > 800) {
			n++;
			save_pts_idx.push_back(i);
		}
	}
	mPlane.plane_xyz.reserve(n);
	mPlane.plane_reflect.reserve(n);
	mPlane.plane_normals.reserve(n);
	mPlane.plane_center.reserve(n);
	mPlane.plane_wall_idx.reserve(n);
	mPlane.plane_ceiling_idx.reserve(n);
	mPlane.plane_ground_idx.reserve(n);
	mPlane.plane_beam_idx.reserve(n);
	mPlane.L_shape_plane_idx.reserve(n);
	mPlane.parallel_plane_idx.reserve(n);
	for (size_t i = 0; i < save_pts_idx.size(); i++) { 
		int idx = save_pts_idx[i]; 

		if (!segResult.plane_xyz[idx].empty()) {
			mPlane.plane_xyz.push_back(segResult.plane_xyz[idx]);
		}

		if (!segResult.plane_reflect[idx].empty()) {
			mPlane.plane_reflect.push_back(segResult.plane_reflect[idx]);
		}

		if (idx < segResult.plane_normals.size()) { 
			cv::Point3f normal = segResult.plane_normals[idx];
			if (cv::norm(normal) > 1e-6) { 
				mPlane.plane_normals.push_back(normal);
			}
		}

		if (idx < segResult.plane_center.size()) {
			cv::Point3f center = segResult.plane_center[idx];
			if (std::isfinite(center.x) && std::isfinite(center.y) && std::isfinite(center.z)) { 
				mPlane.plane_center.push_back(center);
			}
		}

		if (idx < segResult.plane_wall_idx.size() && segResult.plane_wall_idx[idx] >= 0) {
			mPlane.plane_wall_idx.push_back(segResult.plane_wall_idx[idx]);
		}

		if (idx < segResult.plane_ceiling_idx.size() && segResult.plane_ceiling_idx[idx] >= 0) {
			mPlane.plane_ceiling_idx.push_back(segResult.plane_ceiling_idx[idx]);
		}

		if (idx < segResult.plane_ground_idx.size() && segResult.plane_ground_idx[idx] >= 0) {
			mPlane.plane_ground_idx.push_back(segResult.plane_ground_idx[idx]);
		}

		if (idx < segResult.plane_beam_idx.size() && segResult.plane_beam_idx[idx] >= 0) {
			mPlane.plane_beam_idx.push_back(segResult.plane_beam_idx[idx]);
		}

		if (idx < segResult.L_shape_plane_idx.size()) {
			std::pair<int, int> l_pair = segResult.L_shape_plane_idx[idx];
			if (l_pair.first >= 0 && l_pair.second >= 0) {
				mPlane.L_shape_plane_idx.push_back(l_pair);
			}
		}

		if (idx < segResult.parallel_plane_idx.size()) {
			std::pair<int, int> p_pair = segResult.parallel_plane_idx[idx];
			if (p_pair.first >= 0 && p_pair.second >= 0) {
				mPlane.parallel_plane_idx.push_back(p_pair);
			}
		}
	}
	
	
	bool isSuccess = CutPlaneExt(seg_mode, mPlane, true, file_path.size(), cut_poly);
	

	/*for (int i = 0; i < mPlane.plane_xyz.size(); i++)
	{
		std::cout
			<< "plane[" << i << "] "
			<< "pts=" << mPlane.plane_xyz[i].size()
			<< " isWall=" << (
				std::find(mPlane.plane_wall_idx.begin(),
					mPlane.plane_wall_idx.end(),
					i) != mPlane.plane_wall_idx.end()
				)
			<< " isGround=" << (
				std::find(mPlane.plane_ground_idx.begin(),
					mPlane.plane_ground_idx.end(),
					i) != mPlane.plane_ground_idx.end()
				)
			<< " isCeiling=" << (
				std::find(mPlane.plane_ceiling_idx.begin(),
					mPlane.plane_ceiling_idx.end(),
					i) != mPlane.plane_ceiling_idx.end()
				)
			<< std::endl;
	}*/
	if (!isSuccess)
	{
		std::cerr << __FUNCTION__ << " CutPlaneExt failed!" << std::endl;
		return false;
	}

	log_info("plane_wall_idx size: %d", (int)mPlane.plane_wall_idx.size());
	log_info("plane_ceiling_idx size: %d", (int)mPlane.plane_ceiling_idx.size());

	if (!RestorePlaneToOriginalCoordinate(mPlane, relaRT))
	{
		std::cerr << "RestorePlaneToOriginalCoordinate failed!" << std::endl;
		return false;
	}
	
	CalculateCeilFloorExtremes();

	std::vector<std::vector<cv::Point3f>> plane_xyz = mPlane.plane_xyz;
	std::vector<cv::Point3f> plane_normals = mPlane.plane_normals;

	mPlane.plane_corners.clear();
	mPlane.plane_corners.resize(mPlane.plane_xyz.size());

	for (size_t pid = 0; pid < mPlane.plane_xyz.size(); ++pid)
	{
		if (mPlane.plane_xyz[pid].size() < 500)
			continue;

		std::vector<cv::Point3f> real_corner;
		if (MeasureDoorWindow::MeasureFindVerticeFcn(
			plane_normals[pid],
			plane_xyz[pid],
			real_corner))
		{
			mPlane.plane_corners[pid] = std::move(real_corner);
		}
	}

	if (DEBUG_CORNERS_TEST)
	{
		std::string rst_file = "rst_file";
		for (int i = 0; i < plane_xyz.size(); i++)
		{
			if (true)
			{
				int wall_id = i;
				std::string save_plane_ply = rst_file + "/plane_corner_" + std::to_string(wall_id) + ".ply";
				IOData::SavePLYPoints3f(save_plane_ply, mPlane.plane_corners[i], false);

				std::string save_plane_ply2 = rst_file + "/plane_xyz_" + std::to_string(wall_id) + ".ply";
				IOData::SavePLYPoints3f(save_plane_ply2, mPlane.plane_xyz[i], false);
			}

		}

		std::vector<std::vector<MeasureDoorWindow::DoorWindowInfo>> door_window_info = mPlane.door_window_info;
		for (int i = 0; i < door_window_info.size(); i++)
		{
			std::vector<MeasureDoorWindow::DoorWindowInfo> vec_dw = door_window_info[i];
			if (vec_dw.size() > 0)
			{
				for (int j = 0; j < vec_dw.size(); j++)
				{
					MeasureDoorWindow::DoorWindowInfo dw = vec_dw[j];

					int wall_id = dw.wallId;
					int sub_id = j;
					std::string save_plane_ply3 = rst_file + "/plane_dw_" + std::to_string(wall_id) + "_" + std::to_string(sub_id) + ".ply";
					IOData::SavePLYPoints3f(save_plane_ply3, dw.corners, false);
				}

			}

		}
	}

	
#if 0
	string mDebugDir = "dll_log/Plan
		
		eSegmentation_Debug/";
	cout << "mPlane.plane_wall_idx.size():" << mPlane.plane_wall_idx.size() << endl;
	for (int i = 0; i < mPlane.plane_wall_idx.size(); i++)
	{
		IOData::SavePoint3fData(mDebugDir + "wall_" + to_string(mPlane.plane_wall_idx[i]) + ".txt", mPlane.plane_xyz[mPlane.plane_wall_idx[i]]);
	}
	cout << "mPlane.plane_ceiling_idx.size():" << mPlane.plane_ceiling_idx.size() << endl;
	for (int i = 0; i < mPlane.plane_ceiling_idx.size(); i++)
	{
		IOData::SavePoint3fData(mDebugDir + "ceiling_" + to_string(mPlane.plane_ceiling_idx[i]) + ".txt", mPlane.plane_xyz[mPlane.plane_ceiling_idx[i]]);
	}
	cout << "mPlane.plane_beam_idx.size():" << mPlane.plane_beam_idx.size() << endl;
	for (int i = 0; i < mPlane.plane_beam_idx.size(); i++)
	{
		IOData::SavePoint3fData(mDebugDir + "beam_" + to_string(mPlane.plane_beam_idx[i]) + ".txt", mPlane.plane_xyz[mPlane.plane_beam_idx[i]]);
	}
	cout << "mPlane.plane_ground_idx.size():" << mPlane.plane_ground_idx.size() << endl;
	for (int i = 0; i < mPlane.plane_ground_idx.size(); i++)
	{
		IOData::SavePoint3fData(mDebugDir + "ground_" + to_string(mPlane.plane_ground_idx[i]) + ".txt", mPlane.plane_xyz[mPlane.plane_ground_idx[i]]);
	}
#endif


#ifdef	USE_NEW_HOLE_ALGO
	RemeasureHoles();
#endif

	mPlaneDire.resize(mPlane.plane_xyz.size());
	std::fill(mPlaneDire.begin(), mPlaneDire.end(), ePLANE_DIR_UNKOWN);

	if (compass_value != 0)
	{
		mCompassValue = compass_value;
	}
	else
	{
		if (ReadCompass(compass_file))
			mPlaneDire = ComputeDirection();
	}

	ComputeReferenceWall();

	//// 检测一些缺陷 注释
	//if (isSuccess) {
	//	MeasureFlatenessDefectFcn(true);
	//}

	//ComputeReferenceWall();
	
	//Re compute compass again.
	mPlaneDire.resize(mPlane.plane_xyz.size());
	std::fill(mPlaneDire.begin(), mPlaneDire.end(), ePLANE_DIR_UNKOWN);
	std::cout << "compass_file: " << compass_file << "\n";
	if (ReadCompass(compass_file))
		mPlaneDire = ComputeDirection();

	//remove all none ground,beam, ceiling,wall plane
	for (int i = 0; i < mPlane.plane_xyz.size(); i++)
	{
		if (!(std::find(mPlane.plane_ground_idx.begin(), mPlane.plane_ground_idx.end(), i ) != mPlane.plane_ground_idx.end() ||
			std::find(mPlane.plane_wall_idx.begin(), mPlane.plane_wall_idx.end(), i) != mPlane.plane_wall_idx.end() ||
			std::find(mPlane.plane_ceiling_idx.begin(), mPlane.plane_ceiling_idx.end(), i) != mPlane.plane_ceiling_idx.end() ||
			std::find(mPlane.plane_beam_idx.begin(), mPlane.plane_beam_idx.end(), i) != mPlane.plane_beam_idx.end() ||
			(abs(mPlane.plane_normals[i].z) > 0.9 && mPlane.plane_center[i].z > 0.5)))
		{
			mPlane.plane_xyz[i].resize(0);
			mPlane.plane_reflect[i].resize(0);
			mPlane.plane_normals[i] = cv::Point3f(0, 0, 0);
			mPlane.plane_center[i] = cv::Point3f(0, 0, 0);
		}
	}

	if (isSuccess)
	{
		s_ReconstructionErrorCode = RECONSTRUCTION_SUCCESS;
		log_info("Success!");
	}
	else {
		s_ReconstructionErrorCode = RECONSTRUCTION_PLANE_MISMATCH;
		log_info("Fail!");
	}
	return isSuccess;
}

std::vector<std::vector<std::vector<cv::Point3f>>> PointReconstruction::RemeasureHoles(void)
{
#ifdef DBG_HOLE_ALGO
	cout << "RemeasureHoles===" << endl;
	cout << mPlane.plane_xyz.size() << "  " << mPlane.plane_wall_idx.size() << " " << mPlane.door_window_info.size() << endl;
	for (int p = 0; p < mPlane.plane_wall_idx.size(); p++) {
		cout << mPlane.plane_wall_idx[p] << " ; ";
	}
	cout << endl;


	for (int p = 0; p < mPlane.door_window_info.size(); p++) {
		for (int s = 0; s < mPlane.door_window_info[p].size(); s++) {
			cout << mPlane.door_window_info[p][s].wallId << " ; ";
		}
	}
	cout << endl;
#endif

	std::vector<std::vector<int>> type_hole;
	std::vector<std::vector<std::vector<cv::Point3f>>> holes_location;
	std::vector<int> hole_wall_idx;
	holes_location.resize(mPlane.door_window_info.size());
	type_hole.resize(mPlane.door_window_info.size());

	for (int p = 0; p < mPlane.door_window_info.size(); p++) {
		type_hole[p].resize(mPlane.door_window_info[p].size());
		holes_location[p].resize(mPlane.door_window_info[p].size());
#ifdef DBG_HOLE_ALGO
		if (mPlane.door_window_info[p].size() > 0) {
			cout << "=========== op:" << p << " size:" << mPlane.door_window_info[p].size() << endl;
		}
#endif
		for (int s = 0; s < mPlane.door_window_info[p].size(); s++) {
			holes_location[p][s] = mPlane.door_window_info[p][s].corners;
#ifdef DBG_HOLE_ALGO
			cout << mPlane.door_window_info[p][s].corners << endl;
#endif
			type_hole[p][s] = mPlane.door_window_info[p][s].type;
		}
	}
	//return holes_location;
	LineToolHandle windowDoorDimensionHandle;
#ifdef DBG_HOLE_ALGO
	windowDoorDimensionHandle.init_params("", filePath + std::string("Holes\\").c_str(), "config_line_room_UNRE.ini");
	IOData::createDirectory(filePath + std::string("Holes\\").c_str());
#else
	windowDoorDimensionHandle.init_params("", std::string("Holes\\").c_str(), "config_line_room_UNRE.ini");
	//windowDoorDimensionHandle.init_params("","", "config_line_room_UNRE.ini");
#endif
	for (auto measure_index : mPlane.plane_wall_idx)
	{
		if (measure_index < 0) continue;
#ifdef DBG_HOLE_ALGO
		cout << "\n============> wall: " << measure_index << " holes_location size: " << holes_location[measure_index].size() << endl;
#endif


		if (holes_location[measure_index].size() > 0)
		{
			clock_t t1 = clock();
			std::vector<std::pair<float, float>> wd_result;	//frist -> width , second -> height
#ifdef DBG_HOLE_ALGO
															//	std::cout << "windowDoorDimensionHandle.measurement..." << std::endl;
#endif
			windowDoorDimensionHandle.measurement(
				mPlane.plane_xyz,
				mPlane.plane_normals,
				mPlane.plane_center,
				measure_index,
				type_hole[measure_index],
				mPlane.plane_reflect[measure_index],
				holes_location[measure_index],
				wd_result);
#ifdef DBG_HOLE_ALGO
			std::cout << "windowDoorDimensionHandle.measurement  ======>>>>>time : " << (clock() - t1) / float(CLOCKS_PER_SEC)<< std::endl;
#endif
#if 0
			auto& wh = raw_w_h[measure_index];
			for (size_t j = 0; j < wd_result.size(); j++)
			{
				std::cout << "<" << measure_index << ">\tbefore <" << wh[j][0] << ", " << wh[j][1] << ">\t"
					<< "after <" << wd_result[j].first << ", " << wd_result[j].second << ">" << std::endl;
				//	std::cout << "hole 4 vertice (x, y, z): " << std::endl;
				//	for (auto& vertice : corners_hole[measure_index][j])
				//		std::cout << vertice.x << ", " << vertice.y << ", " << vertice.z << std::endl;
				//wh[j][0] = wd_result[j].first;
				//wh[j][1] = wd_result[j].second;
				//only update door width
				//if (type_hole[measure_index][j] == 1)
				//	wh[j][0] = wd_result[j].first;
			}
#endif
		}
	}


	for (int p = 0; p < holes_location.size(); p++) {
		for (int s = 0; s < holes_location[p].size(); s++) {
			mPlane.door_window_info[p][s].corners = holes_location[p][s];
		}
	}
#ifdef DBG_HOLE_ALGOR
	for (int p = 0; p < mPlane.door_window_info.size(); p++) {
		if (mPlane.door_window_info[p].size() > 0) {
			cout << "=========== ap:" << p << " size:" << mPlane.door_window_info[p].size() << endl;
		}
		for (int s = 0; s < mPlane.door_window_info[p].size(); s++) {
			cout << mPlane.door_window_info[p][s].corners << endl;
		}
	}
#endif	
	return  holes_location;
}

bool PointReconstruction::MeasureFlatenessDefectFcn(bool defect3d)
{
    WallObstacleInfo * wallObstacleInfo = new WallObstacleInfo(mPlane.plane_xyz);
	DetectObstacle obstacle;
	obstacle.DetectWallObstacleFcn(mPlane, *wallObstacleInfo, defect3d);

	MeasureDefect measure_defect(defect3d);
	std::vector<int> plane_ground_idx = mPlane.plane_ground_idx;
	std::vector<int> plane_wall_idx = mPlane.plane_wall_idx;
	std::vector<int> plane_celing_idx = mPlane.plane_ceiling_idx;
	std::vector<cv::Point3f> plane_normals = mPlane.plane_normals;
	std::vector<cv::Point3f> plane_centers = mPlane.plane_center;
	
	flateness_defect.clear();
	if (nullptr != wallObstacleInfo)
	{
		removeCritical_flateness_defect.clear();
	}
	for (int i = 0; i < plane_wall_idx.size(); i++)
	{
		//std::cout << "zhujunqing show plane_wall_idx =  " << plane_wall_idx[i] << endl;
		std::vector<cv::Point3f> plane_points = mPlane.plane_xyz[plane_wall_idx[i]];
		//std::vector<cv::Point3f> plane_points = wallObstacleInfo->removeCritical_filtered_wall_xyz[plane_wall_idx[i]];
		cv::Point3f normal = plane_normals[plane_wall_idx[i]];
		cv::Point3f center = plane_centers[plane_wall_idx[i]];
		std::vector<std::pair<float, cv::Point3f>> defect;
		float plane_normal[3];
		plane_normal[0] = normal.x;
		plane_normal[1] = normal.y;
		plane_normal[2] = normal.z;
		float plane_center[3];
		plane_center[0] = center.x;
		plane_center[1] = center.y;
		plane_center[2] = center.z;
		float plane_normal_uniformed[3];

		MeasureBase::UniformNormals(plane_normal, plane_center, plane_normal_uniformed);

		
		measure_defect.MeasureDefectFcn(plane_normal_uniformed, plane_center, plane_points, defect);
		flateness_defect.push_back(std::make_pair(plane_wall_idx[i], defect));

		if (nullptr != wallObstacleInfo)
		{
			std::vector<std::pair<float, cv::Point3f>> defect_removeCritical;
			std::vector<cv::Point3f> removeCritcal_plane_points = wallObstacleInfo->removeCritical_filtered_wall_xyz[i];
			measure_defect.MeasureDefectFcn(plane_normal_uniformed, plane_center, removeCritcal_plane_points, defect_removeCritical);
			removeCritical_flateness_defect.push_back(std::make_pair(plane_wall_idx[i], defect_removeCritical));
		}
	}

	for (int i = 0; i < plane_ground_idx.size(); i++)
	{
		//std::cout << "zhujunqing show plane_ground_idx =  " << plane_ground_idx[i] << endl;
		std::vector<cv::Point3f> plane_points = mPlane.plane_xyz[plane_ground_idx[i]];
		cv::Point3f normal = plane_normals[plane_ground_idx[i]];
		cv::Point3f center = plane_centers[plane_ground_idx[i]];
		std::vector<std::pair<float, cv::Point3f>> defect;
		float plane_normal[3];
		plane_normal[0] = normal.x;
		plane_normal[1] = normal.y;
		plane_normal[2] = normal.z;
		float plane_center[3];
		plane_center[0] = center.x;
		plane_center[1] = center.y;
		plane_center[2] = center.z;
		measure_defect.MeasureDefectFcn(plane_normal, plane_center, plane_points, defect);
		flateness_defect.push_back(std::make_pair(plane_ground_idx[i], defect));
		if (nullptr != wallObstacleInfo)
			removeCritical_flateness_defect.push_back(std::make_pair(plane_ground_idx[i], defect));
	}

	for (int i = 0; i < plane_celing_idx.size(); i++)
	{
		//std::cout << "zhujunqing show plane_celing_idx =  " << plane_celing_idx[i] << endl;
		std::vector<cv::Point3f> plane_points = mPlane.plane_xyz[plane_celing_idx[i]];
		cv::Point3f normal = plane_normals[plane_celing_idx[i]];
		cv::Point3f center = plane_centers[plane_celing_idx[i]];
		std::vector<std::pair<float, cv::Point3f>> defect;
		float plane_normal[3];
		plane_normal[0] = normal.x;
		plane_normal[1] = normal.y;
		plane_normal[2] = normal.z;
		float plane_center[3];
		plane_center[0] = center.x;
		plane_center[1] = center.y;
		plane_center[2] = center.z;
		measure_defect.MeasureDefectFcn(plane_normal, plane_center, plane_points, defect);
		flateness_defect.push_back(std::make_pair(plane_celing_idx[i], defect));
		if (nullptr != wallObstacleInfo)
			removeCritical_flateness_defect.push_back(std::make_pair(plane_celing_idx[i], defect));
	}

	mPlane.flateness_defect = flateness_defect;
	if (nullptr != wallObstacleInfo)
		mPlane.removeCritical_flateness_defect = removeCritical_flateness_defect;

	if (wallObstacleInfo != nullptr)
		delete wallObstacleInfo;
	wallObstacleInfo = nullptr;

	return true;
}


bool PointReconstruction::StructureReconstruction(void)
{
	log_info("start StructureReconstruction");

	mpReconstruction = make_shared<Reconstructer>(/*idxs,*/GetPlanes(), "out");

	mFakeCeiling = false;
	bool bRes = mpReconstruction->ProcessingModel();
	if (bRes)
		bRes = BuildStructuredData();
	log_info("end StructureReconstruction, status %d", bRes);
	if (!bRes)
		s_ReconstructionErrorCode = RECONSTRUCTION_NORMAL_ERROR;
	return bRes;
}

bool PointReconstruction::StructureReconstruction(float fake_ceiling_z)
{
	log_info("start StructureReconstruction by fake ceiling %f", fake_ceiling_z);

	mpReconstruction = make_shared<Reconstructer>(/*idxs,*/GetPlanes(), fake_ceiling_z, "out");

	mFakeCeiling = true;
	bool bRes = mpReconstruction->ProcessingModel();
	if (bRes)
		bRes = BuildStructuredData();
	log_info("end StructureReconstruction, status %d", bRes);
	if (!bRes)
		s_ReconstructionErrorCode = RECONSTRUCTION_FAKE_CEILING_ERROR;
	return bRes;
}

bool PointReconstruction::StructureReconstructionByBottomCeil(void)
{
	/*std::cout << "plane size: " << mPlane.plane_xyz.size() << "\n";
	std::cout << "plane_ground_idx size: " << mPlane.plane_ground_idx.size() << "\n";
	std::cout << "plane_ceiling_idx size: " << mPlane.plane_ceiling_idx.size() << "\n";
	std::cout << "plane_wall_idx size: " << mPlane.plane_wall_idx.size() << "\n";
	std::cout << "plane_beam_idx size: " << mPlane.plane_beam_idx.size() << "\n";*/
	constexpr float kMinCeilAreaM2 = 1.0e6f;
	log_info("start StructureReconstructionByBottomCeil");
	std::vector<int> ceiling_idx = mPlane.plane_ceiling_idx;
	if (ceiling_idx.size() == 0)
		return false;

	if (ceiling_idx.size() == 1)
	{
		return StructureReconstruction();
	}
	int bottomceiling = ceiling_idx[0];
	for (auto id : ceiling_idx)
	{
		//std::cout << "ceiling id: " << id << ", area: " << mPlane.plane_area[id] << ", center z: " << mPlane.plane_center[id].z << "\n";
		if (mPlane.plane_area[id] > kMinCeilAreaM2) {
			if (mPlane.plane_center[bottomceiling].z > mPlane.plane_center[id].z)
				bottomceiling = id;
		}
	}
	log_info("bottom ceiling id: %d", bottomceiling);
	log_info("ceiling z: %f", mPlane.plane_center[bottomceiling].z);
	//cout << "ceiling z: " << mPlane.plane_center[bottomceiling].z << "\n";
	mFakeCeiling = true;
	mpReconstruction = make_shared<Reconstructer>(mPlane, mPlane.plane_center[bottomceiling].z, "out");
	bool bRes = mpReconstruction->ProcessingModel();
	if (bRes)
		bRes = BuildStructuredData();
	log_info("end StructureReconstruction, status %d", bRes);
	if (!bRes)
		s_ReconstructionErrorCode = RECONSTRUCTION_FAKE_CEILING_ERROR;
	return bRes;
}

bool PointReconstruction::CalculateCeilFloorExtremes()
{
	const float voxelSize = 50.0f;   // mm

	//==============================
	// 1. 找最大面积天花
	//==============================
	int ceilPlaneIdx = -1;
	float maxArea = 0;

	for (int id : mPlane.plane_ceiling_idx)
	{
		if (mPlane.plane_area[id] > maxArea)
		{
			maxArea = mPlane.plane_area[id];
			ceilPlaneIdx = id;
		}
	}

	if (ceilPlaneIdx < 0)
	{
		std::cout << "No ceiling plane\n";
		return false;
	}

	//==============================
	// 2. 找最大面积地板
	//==============================
	int floorPlaneIdx = -1;
	maxArea = 0;

	for (int id : mPlane.plane_ground_idx)
	{
		if (mPlane.plane_area[id] > maxArea)
		{
			maxArea = mPlane.plane_area[id];
			floorPlaneIdx = id;
		}
	}

	if (floorPlaneIdx < 0)
	{
		std::cout << "No floor plane\n";
		return false;
	}

	auto& ceilPts = mPlane.plane_xyz[ceilPlaneIdx];
	auto& floorPts = mPlane.plane_xyz[floorPlaneIdx];

	//==============================
	// 3. 建立 Grid
	//==============================
	std::unordered_map<GridKey, GridCell, GridKeyHash> ceilGrid;
	std::unordered_map<GridKey, GridCell, GridKeyHash> floorGrid;

	auto buildGrid = [&](const std::vector<cv::Point3f>& pts,
		auto& grid)
		{
			for (auto& p : pts)
			{
				GridKey k;
				k.x = (int)std::floor(p.x / voxelSize);
				k.y = (int)std::floor(p.y / voxelSize);
				grid[k].add(p);
			}
		};

	buildGrid(ceilPts, ceilGrid);
	buildGrid(floorPts, floorGrid);

	//==============================
	// 4. 初始化
	//==============================
	double ceilMinZ = 1e12, ceilMaxZ = -1e12;
	double floorMinZ = 1e12, floorMaxZ = -1e12;
	double minGap = 1e12, maxGap = -1e12;

	GridKey ceilMinKey{}, ceilMaxKey{};
	GridKey floorMinKey{}, floorMaxKey{};

	//==============================
	// 5. 天花极值
	//==============================
	for (auto& kv : ceilGrid)
	{
		double z = kv.second.avgZ();

		if (z < ceilMinZ)
		{
			ceilMinZ = z;
			ceilMinKey = kv.first;
		}
		if (z > ceilMaxZ)
		{
			ceilMaxZ = z;
			ceilMaxKey = kv.first;
		}
	}

	//==============================
	// 6. 地板极值
	//==============================
	for (auto& kv : floorGrid)
	{
		double z = kv.second.avgZ();

		if (z < floorMinZ)
		{
			floorMinZ = z;
			floorMinKey = kv.first;
		}
		if (z > floorMaxZ)
		{
			floorMaxZ = z;
			floorMaxKey = kv.first;
		}
	}

	//==============================
	// 7. 4组极值配对
	//==============================
	auto tryMakePair = [&](GridKey k, GapPairResult& out)
		{
			bool hasCeil = ceilGrid.count(k);
			bool hasFloor = floorGrid.count(k);

			if (!hasCeil && !hasFloor)
				return; 

			out.valid = true;

			if (hasCeil)
			{
				double ceilZ = ceilGrid[k].avgZ();
				out.ceilPt = voxelCenterPoint(
					k.x, k.y, voxelSize, ceilZ);
			}
			else
			{
				out.ceilPt = cv::Point3f(0, 0, 0);
			}

			if (hasFloor)
			{
				double floorZ = floorGrid[k].avgZ();
				out.floorPt = voxelCenterPoint(
					k.x, k.y, voxelSize, floorZ);
			}
			else
			{
				out.floorPt = cv::Point3f(0, 0, 0);
			}

			if (hasCeil && hasFloor)
				out.gap = out.ceilPt.z - out.floorPt.z;
			else
				out.gap = 0.0;
		};


	tryMakePair(ceilMinKey, ceilMinPair);
	tryMakePair(ceilMaxKey, ceilMaxPair);
	tryMakePair(floorMinKey, floorMinPair);
	tryMakePair(floorMaxKey, floorMaxPair);

	//==============================
	// 8. 扫描 Min / Max Gap
	//==============================
	for (auto& kv : ceilGrid)
	{
		GridKey k = kv.first;
		if (!floorGrid.count(k))
			continue;

		double ceilZ = kv.second.avgZ();
		double floorZ = floorGrid[k].avgZ();
		double gap = ceilZ - floorZ;

		if (gap < minGap)
		{
			minGap = gap;
			minGapPair.valid = true;
			minGapPair.ceilPt = voxelCenterPoint(k.x, k.y, voxelSize, ceilZ);
			minGapPair.floorPt = voxelCenterPoint(k.x, k.y, voxelSize, floorZ);
			minGapPair.gap = gap;
		}

		if (gap > maxGap)
		{
			maxGap = gap;
			maxGapPair.valid = true;
			maxGapPair.ceilPt = voxelCenterPoint(k.x, k.y, voxelSize, ceilZ);
			maxGapPair.floorPt = voxelCenterPoint(k.x, k.y, voxelSize, floorZ);
			maxGapPair.gap = gap;
		}
	}

#ifdef DEBUG
	auto printPair = [&](const std::string& name,
		const GapPairResult& r)
		{
			if (!r.valid)
			{
				std::cout << name << ": empty\n";
				return;
			}

			std::cout << name << "\n";
			std::cout << "  Ceil : " << r.ceilPt << "\n";
			std::cout << "  Floor: " << r.floorPt << "\n";
			std::cout << "  Gap  : " << r.gap << " mm\n";

			std::string file_name = name + ".txt";
			std::ofstream single_file(file_name);
			single_file <<  r.ceilPt.x << " " << r.ceilPt.y << " " << r.ceilPt.z << "\n";
			single_file <<  r.floorPt.x << " " << r.floorPt.y << " " << r.floorPt.z << "\n";
			single_file.close();
		};

	printPair("CeilMinZ_Pair", ceilMinPair);
	printPair("CeilMaxZ_Pair", ceilMaxPair);
	printPair("FloorMinZ_Pair", floorMinPair);
	printPair("FloorMaxZ_Pair", floorMaxPair);
	printPair("MinGap_Pair", minGapPair);
	printPair("MaxGap_Pair", maxGapPair);
#endif // DEBUG

	return true;
}



PlaneCutResultInterface &PointReconstruction::GetPlanes(void)
{
	return mPlane;
}


void PointReconstruction::ExportAsObj(std::string filename, bool hasMapping)
{
	mpReconstruction->SaveAsObj(GetPlanes().door_window_info, filename);
}

void PointReconstruction::ExportOriPointAsObj(std::string filename)
{
	mpReconstruction->SaveModelAsObj(filename);
}


std::vector<cv::Point3f> PointReconstruction::GetRoomContour(bool rebuild)
{
	log_info("GetRoomContour rebuild: %d", rebuild);
	//std::cout << "rebuild: " << rebuild << "\n";
	if (!rebuild)
		return mContour;

	std::vector<cv::Point3f> contour;
	std::vector<Tree<Point3fArray>> ground_contour;
	std::vector<std::vector<Tree<Point3fArray>>> roomContours = mpReconstruction->getContourTreeList();

	
	ground_contour = roomContours[GetPlanes().plane_ground_idx[0]];

	contour = ground_contour[0].get()->_val;

	int wisecount = 0;
	for (int i = 0; i < contour.size(); i++)
	{
		if (contour[i].cross((contour[i] + contour[(i + 1) % contour.size()]) / 2).z < 0)
			wisecount++;
	}

	std::vector<cv::Point3f> contour_tmp;
	if ((float)wisecount / contour.size() < 0.3)
	{
		std::vector<cv::Point3f>::reverse_iterator riter;
		for (riter = contour.rbegin(); riter != contour.rend(); riter++)
		{
			contour_tmp.push_back(*riter);
		}
		contour.swap(contour_tmp);
	}
	mContour = contour;
	return  mContour;
}

double PointReconstruction::GetAngle(double x1, double y1, double x2, double y2, double x3, double y3)
{
	double theta = atan2(x1 - x2, y1 - y2) - atan2(x3 - x2, y3 - y2);
	if (theta > M_PI)
		theta -= 2 * M_PI;
	if (theta < -M_PI)
		theta += 2 * M_PI;

	theta = abs(theta * 180.0 / M_PI);
	return theta;
}


cv::Point2f PointReconstruction::GetFootOfPerpendicular(const cv::Point2f &pt, const cv::Point2f &begin,const cv::Point2f &end)
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


void PointReconstruction::DisplayRoomContour(void)
{

	std::vector<cv::Point3f> ground_contour = GetRoomContour();
	float minX = 10000, maxX = -10000, minY = 10000, maxY = -10000;

	for (int i = 0; i < ground_contour.size(); i++)
	{
		float X = ground_contour[i].x;
		float Y = ground_contour[i].y;
		if (X > maxX) {
			maxX = X;
		}
		if (X < minX) {
			minX = X;
		}
		if (Y > maxY) {
			maxY = Y;
		}
		if (Y < minY) {
			minY = Y;
		}
		//std::cout << ground_contour[i] << std::endl;
	}

	int imgw = (maxX - minX) / 10 + 1;
	int imgh = (maxY - minY) / 10 + 1;
	int pending = 10;

	cv::Mat contourMap = cv::Mat(imgh + pending * 2, imgw + pending * 2, CV_8UC1);
	contourMap = 0;
	
	//auto pts = contour;
	for (int i = 0; i < ground_contour.size(); i++)
	{
		cv::Point2f pt_s((ground_contour[i].x - minX) / 10 + pending, (ground_contour[i].y - minY) / 10 + pending);
		cv::Point2f pt_end((ground_contour[(i + 1) % ground_contour.size()].x - minX) / 10 + pending, (ground_contour[(i + 1) % ground_contour.size()].y - minY) / 10 + pending);
		cv::line(contourMap, pt_s, pt_end, cv::Scalar(255, 255, 255), 1);
	}

	cv::imshow("contourMap", contourMap);
	cv::waitKey(0);
}
void PointReconstruction::MakeRoomSquare(void)
{
	std::vector<std::vector<Tree<Point3fArray>>> roomContours = mpReconstruction->getContourTreeList();
	for (auto ground : GetPlanes().plane_ground_idx)
	{
		log_info("roomContours.size() == %zu", roomContours.size());
		log_info("ground id == %zu", ground);
		//std::cout << "roomContours.size() == " << roomContours.size() << std::endl;
		//std::cout << "ground id == " << ground << std::endl;
		std::vector<Tree<Point3fArray>> ground_contour = roomContours[ground];
		int max_edge_idx = -1;
		for (auto contour : ground_contour)
		{
			log_info("ground_contour.size(): == %zu", ground_contour.size());
			//std::cout << "ground_contour.size(): == " << ground_contour.size() << std::endl;
			auto pt_tree = contour.get();
			float max_edge = 0;
			for (int i = 0; i < pt_tree->_val.size(); i++)
			{
				float temp = std::sqrt(std::pow(pt_tree->_val[i].x - pt_tree->_val[(i + 1) % pt_tree->_val.size()].x, 2.f)
					+ std::pow(pt_tree->_val[i].y - pt_tree->_val[(i + 1) % pt_tree->_val.size()].y, 2.f));
				if (max_edge < temp) {
					max_edge = temp;
					max_edge_idx = i;
				}
			}

			for (int i = 0; i < pt_tree->_val.size(); i++)
			{
				int round = pt_tree->_val.size();
				int h = max_edge_idx + i;

				auto Angle = GetAngle(pt_tree->_val[h % round].x, pt_tree->_val[h % round].y,
					pt_tree->_val[(h + 1) % round].x, pt_tree->_val[(h + 1) % round].y,
					pt_tree->_val[(h + 2) % round].x, pt_tree->_val[(h + 2) % round].y);

				cv::Point2f pt, begin, end, foot;
				if (Angle < 90.0) {
					pt.x = pt_tree->_val[(h + 2) % round].x;
					pt.y = pt_tree->_val[(h + 2) % round].y;
					begin.x = pt_tree->_val[h % round].x;
					begin.y = pt_tree->_val[h % round].y;
					end.x = pt_tree->_val[(h + 1) % round].x;
					end.y = pt_tree->_val[(h + 1) % round].y;
					foot = GetFootOfPerpendicular(pt, begin, end);
					pt_tree->_val[(h + 1) % round].x = foot.x;
					pt_tree->_val[(h + 1) % round].y = foot.y;
				}
				else if (Angle > 90.0) {
					pt.x = pt_tree->_val[(h + 1) % round].x;
					pt.y = pt_tree->_val[(h + 1) % round].y;
					begin.x = pt_tree->_val[(h + 2) % round].x;
					begin.y = pt_tree->_val[(h + 2) % round].y;
					end.x = pt_tree->_val[(h + 3) % round].x;
					end.y = pt_tree->_val[(h + 3) % round].y;
					foot = GetFootOfPerpendicular(pt, begin, end);
					pt_tree->_val[(h + 2) % round].x = foot.x;
					pt_tree->_val[(h + 2) % round].y = foot.y;
				}
			}
		}
	}
}


std::vector<StructuredPlane>  PointReconstruction::GetStructuredPlanes(void)
{
	return mStructuredPlane;
}

void PointReconstruction::UpdateStructuredPlanesSquared(std::vector<StructuredPlane> updatePlanes)
{
	mStructuredPlaneSquared = updatePlanes;
}

void PointReconstruction::UpdateStructuredPlanesSquaredMin(std::vector<StructuredPlane> updatePlanes)
{
	mStructuredPlaneSquaredMin = updatePlanes;
}

void PointReconstruction::UpdateStructuredPlanesSquaredMin05(std::vector<StructuredPlane> updatePlanes)
{
	mStructuredPlaneSquaredMin05 = updatePlanes;
}

std::vector<StructuredPlane>  PointReconstruction::GetStructuredPlanesSquared(void)
{
	return mStructuredPlaneSquared;
}

std::vector<StructuredPlane> PointReconstruction::GetStructuredPlanesSquaredMin(void)
{
	return	mStructuredPlaneSquaredMin;
}

std::vector<StructuredPlane> PointReconstruction::GetStructuredPlanesSquaredMin05(void)
{
	return	mStructuredPlaneSquaredMin05;
}

void PointReconstruction::UpdateRoomContourSquared(std::vector<cv::Point3f> contour)
{
	mContourSquared = contour;
}

void PointReconstruction::UpdateRoomContourSquaredMin(std::vector<cv::Point3f> contour)
{
	mContourSquaredMin = contour;
}

void PointReconstruction::UpdateRoomContourSquaredMin05(std::vector<cv::Point3f> contour)
{
	mContourSquaredMin05 = contour;
}

std::vector<cv::Point3f> PointReconstruction::GetRoomContourSquared(void)
{
	return	mContourSquared;
}

std::vector<cv::Point3f> PointReconstruction::GetRoomContourSquaredMin(void)
{
	return	mContourSquaredMin;
}

std::vector<cv::Point3f> PointReconstruction::GetRoomContourSquaredMin05(void)
{
	return	mContourSquaredMin05;
}

cv::Point3f PointReconstruction::verticesNormal(std::vector<cv::Point3f> pts)
{
	cv::Point3f normal;
	if (pts.size() < 3)
		return normal;

	cv::Point3f v0 = pts[0];
	cv::Point3f v1 = pts[1];
	cv::Point3f v2 = pts[2];

	cv::Point3f e1 = v1 - v0;
	cv::Point3f e2 = v2 - v1;

	normal = e1.cross(e2);

	if (cv::norm(normal) < 1e-6) {
		return cv::Point3f(0, 0, 0);
	}
	normal /= cv::norm(normal);
	return normal;
}

ePlane_Type PointReconstruction::getWallType(int planeid)
{
	ePlane_Type type = ePLANE_TYPE_UNKOWN;

	if (std::find(mPlane.plane_wall_idx.begin(), mPlane.plane_wall_idx.end(), planeid) != mPlane.plane_wall_idx.end())
		type = ePLANE_WALL;
	else if(std::find(mPlane.plane_ground_idx.begin(), mPlane.plane_ground_idx.end(), planeid) != mPlane.plane_ground_idx.end())
		type = ePLANE_GROUND;
	else if (std::find(mPlane.plane_ceiling_idx.begin(), mPlane.plane_ceiling_idx.end(), planeid) != mPlane.plane_ceiling_idx.end())
		type = ePLANE_CEILING;
	else if (std::find(mPlane.plane_beam_idx.begin(), mPlane.plane_beam_idx.end(), planeid) != mPlane.plane_beam_idx.end())
		type = ePLANE_BEAM;
	return type;
}

#define USE_FRIST_CONTOUR 0
std::vector<Tree<Point3fArray>> PointReconstruction::DevidePlanes(void)
{
#if USE_FRIST_CONTOUR
	std::vector<std::vector<Tree<Point3fArray>>> roomContours = mpReconstruction->getContourTreeList();
	std::vector<Tree<Point3fArray>>  roomContours_new;
	roomContours_new.resize(roomContours.size());
	for (int i = 0; i < roomContours.size(); i++)
	{
		if (roomContours[i].size() == 0)
			continue;

		roomContours_new[i] = roomContours[i][0];

	}

	return roomContours_new;
#else

	std::vector<std::vector<Tree<Point3fArray>>> roomContours = mpReconstruction->getContourTreeList();


	std::vector<Tree<Point3fArray>>  roomContours_new;
	std::vector<Tree<Point3fArray>>  roomContours_append;
	std::vector<ePlane_Direction>  PlaneDire_append;
	PlaneCutResultInterface   mOriginPlane_append;
	roomContours_new.resize(roomContours.size());
	for (int i = 0; i < roomContours.size(); i++)
	{
		if (roomContours[i].size() == 0)
			continue;

		roomContours_new[i] = roomContours[i][0];
		if (roomContours[i].size() == 1 || i >= mPlane.plane_xyz.size())
			continue;


		std::vector<cv::Point3f>  point_rot;
		std::vector<std::vector<cv::Point3f>> vertices_rot;
		std::vector<std::vector<int>> vertices_idx;

		int door_id = -1;
		std::vector<std::vector<cv::Point3f>>  windows_vertices;
		//Parallel pair
		if (abs(GetPlanes().plane_normals[i].z) < 0.3)
		{
			cv::Point3f normal_plane = -GetPlanes().plane_normals[i];
			cv::Point3f rot_axis = { 0.f, 1.f, 0.f };

			cv::Point3f cross = Util_Math::ComputeVectorCrossProduct(normal_plane, rot_axis);
			float  angle_rot = 0;

			angle_rot = (cross.z > 0.f) ?
				acosf(normal_plane.dot(rot_axis) / std::sqrt(std::pow(normal_plane.x, 2) + std::pow(normal_plane.y, 2))) :
				-acosf(normal_plane.dot(rot_axis) / std::sqrt(std::pow(normal_plane.x, 2) + std::pow(normal_plane.y, 2)));

			cv::Mat rotation_matrix = cv::Mat::zeros(3, 3, CV_32FC1);
			rotation_matrix.at<float>(0, 0) = cos(angle_rot);
			rotation_matrix.at<float>(0, 1) = -sin(angle_rot);
			rotation_matrix.at<float>(1, 0) = sin(angle_rot);
			rotation_matrix.at<float>(1, 1) = cos(angle_rot);
			rotation_matrix.at<float>(2, 2) = 1.f;

			
			point_rot = MathOperation::plane_rot(rotation_matrix, mPlane.plane_xyz[i]);

			//windows info
			if (std::find(mPlane.plane_wall_idx.begin(), mPlane.plane_wall_idx.end(), i) != mPlane.plane_wall_idx.end())
			{
				door_id = std::find(mPlane.plane_wall_idx.begin(), mPlane.plane_wall_idx.end(), i) - mPlane.plane_wall_idx.begin();
				windows_vertices.resize(mPlane.door_window_info[door_id].size());
				for (int wid = 0; wid < mPlane.door_window_info[door_id].size(); wid++)
				{
					windows_vertices[wid] = MathOperation::plane_rot(rotation_matrix, mPlane.door_window_info[door_id][wid].corners);
				}
			}
			
			for (int subwall = 0; subwall < roomContours[i].size(); subwall++)
			{
				vertices_rot.push_back(MathOperation::plane_rot(rotation_matrix, roomContours[i][subwall].get()->_val));
			}
		}
		else
		{
			point_rot = mPlane.plane_xyz[i];
			for (int subwall = 0; subwall < roomContours[i].size(); subwall++)
			{
				vertices_rot.push_back(roomContours[i][subwall].get()->_val);
			}
		}

		std::vector<std::vector<MeasureDoorWindow::DoorWindowInfo>> windows_info_new;

		for (auto contour : vertices_rot)
		{
			std::vector<int> pt_id;
			cv::Point3f wall_max, wall_min;
			MathOperation::GetMinMaxXYZ(contour, wall_max, wall_min);

			for (int id = 0; id < point_rot.size(); id++)
			{
				if (abs(GetPlanes().plane_normals[i].z) < 0.3 &&
					point_rot[id].x >= wall_min.x && point_rot[id].x <= wall_max.x &&
					point_rot[id].z >= wall_min.z && point_rot[id].z <= wall_max.z)
				{
					pt_id.push_back(id);
				}
				else if(point_rot[id].x >= wall_min.x && point_rot[id].x <= wall_max.x &&
					point_rot[id].y >= wall_min.y && point_rot[id].y <= wall_max.y)
				{
					pt_id.push_back(id);
				}
			}

			if (pt_id.size() > 100)
			{
				vertices_idx.push_back(pt_id);
				windows_info_new.push_back(std::vector<MeasureDoorWindow::DoorWindowInfo>());
				for (int wid =0; wid < windows_vertices.size();wid++)
				{
					cv::Point3f win_max, win_min;
					MathOperation::GetMinMaxXYZ(windows_vertices[wid], win_max, win_min);

					if (min(win_max.x, wall_max.x) - max(win_min.x, wall_min.x)> 0)
					{
						windows_info_new[windows_info_new.size() - 1].push_back(mPlane.door_window_info[door_id][wid]);
					}

				}
			}
		}

		int cur_append_id = roomContours_append.size();

		std::vector<cv::Point3f> plane_pts = mPlane.plane_xyz[i];
		std::vector<unsigned char> plane_reflect= mPlane.plane_reflect[i];
		for (int id = 0; id < vertices_idx.size(); id++)
		{
			//
			if (id > 0)
				roomContours_append.push_back(roomContours[i][id]);

			std::vector<cv::Point3f> pts_tmp;
			std::vector<unsigned char> reflect_tmp;
			cv::Point3f ptCounts(0, 0, 0);
			for (auto pid : vertices_idx[id])
			{
				ptCounts += plane_pts[pid];
				pts_tmp.push_back(plane_pts[pid]);
				reflect_tmp.push_back(plane_reflect[pid]);
			}
			

			ptCounts.x /= vertices_idx[id].size();
			ptCounts.y /= vertices_idx[id].size();
			ptCounts.z /= vertices_idx[id].size();
			
			if (id == 0)
			{
				mPlane.plane_xyz[i].swap(pts_tmp);
				mPlane.plane_reflect[i].swap(reflect_tmp);
				mPlane.plane_center[i] = ptCounts;
				if(door_id != -1)
					mPlane.door_window_info[door_id] = windows_info_new[0];
			}
			else
			{
				mOriginPlane_append.plane_normals.push_back(mPlane.plane_normals[i]);
				mOriginPlane_append.plane_xyz.push_back(pts_tmp);
				mOriginPlane_append.plane_reflect.push_back(reflect_tmp);
				mOriginPlane_append.plane_center.push_back(ptCounts);
				mOriginPlane_append.door_window_info.push_back(windows_info_new[id]);
				PlaneDire_append.push_back(mPlaneDire[i]);
				if (std::find(mPlane.plane_wall_idx.begin(), mPlane.plane_wall_idx.end(), i) != mPlane.plane_wall_idx.end())
					mOriginPlane_append.plane_wall_idx.push_back(mPlane.plane_xyz.size() + mOriginPlane_append.plane_xyz.size() - 1);

				if (std::find(mPlane.plane_ground_idx.begin(), mPlane.plane_ground_idx.end(), i) != mPlane.plane_ground_idx.end())
					mOriginPlane_append.plane_ground_idx.push_back(mPlane.plane_xyz.size() + mOriginPlane_append.plane_xyz.size() - 1);

				if (std::find(mPlane.plane_ceiling_idx.begin(), mPlane.plane_ceiling_idx.end(), i) != mPlane.plane_ceiling_idx.end())
					mOriginPlane_append.plane_ceiling_idx.push_back(mPlane.plane_xyz.size() + mOriginPlane_append.plane_xyz.size() - 1);

				if (std::find(mPlane.plane_beam_idx.begin(), mPlane.plane_beam_idx.end(), i) != mPlane.plane_beam_idx.end())
					mOriginPlane_append.plane_beam_idx.push_back(mPlane.plane_xyz.size() + mOriginPlane_append.plane_xyz.size() - 1);
			}
		}

	}

	int origin_plane_count = mPlane.plane_xyz.size();

	for (int id = 0; id < mOriginPlane_append.plane_xyz.size(); id++)
	{
		mPlane.plane_normals.push_back(mOriginPlane_append.plane_normals[id]);
		mPlane.plane_xyz.push_back(mOriginPlane_append.plane_xyz[id]);
		mPlane.plane_reflect.push_back(mOriginPlane_append.plane_reflect[id]);
		mPlane.plane_center.push_back(mOriginPlane_append.plane_center[id]);
		mPlane.door_window_info.push_back(mOriginPlane_append.door_window_info[id]);
	}
	mPlane.plane_beam_idx.insert(mPlane.plane_beam_idx.end(), mOriginPlane_append.plane_beam_idx.begin(), mOriginPlane_append.plane_beam_idx.end());
	mPlane.plane_wall_idx.insert(mPlane.plane_wall_idx.end(), mOriginPlane_append.plane_wall_idx.begin(), mOriginPlane_append.plane_wall_idx.end());
	mPlane.plane_ground_idx.insert(mPlane.plane_ground_idx.end(), mOriginPlane_append.plane_ground_idx.begin(), mOriginPlane_append.plane_ground_idx.end());
	mPlane.plane_ceiling_idx.insert(mPlane.plane_ceiling_idx.end(), mOriginPlane_append.plane_ceiling_idx.begin(), mOriginPlane_append.plane_ceiling_idx.end());

	roomContours_new.insert(roomContours_new.begin() + origin_plane_count, roomContours_append.begin(), roomContours_append.end());
	mPlaneDire.insert(mPlaneDire.end(), PlaneDire_append.begin(), PlaneDire_append.end());
	
	std::vector<std::vector<Tree<Point3fArray>>> roomContours_update;
	roomContours_update.resize(roomContours_new.size());
	for (int i=0;i < roomContours_new.size(); i++)
	{
		if (!roomContours_new[i].empty())
		{
			roomContours_update[i].push_back(roomContours_new[i]);
		}
	}
	mpReconstruction->UpdateContourTreeList(roomContours_update);

	//call first
	GetRoomContour(true);
	return roomContours_new;
#endif
}


bool PointReconstruction::BuildStructuredData(void)
{
	std::vector<std::vector<Tree<Point3fArray>>> contour_tree = mpReconstruction->getContourTreeList();
	log_info("contour_tree'size: %zu", contour_tree.size());
	//std::cout << "contour_tree'size: " << contour_tree.size() << "\n";
	int plane_count = 0;
	for (auto contour : contour_tree)
		plane_count += contour.size();
	if (plane_count < 5)
		return false;
	//std::vector<std::vector<Tree<Point3fArray>>> roomContours = DevidePlanes();// mpReconstruction->getContourTreeList();
	std::vector<Tree<Point3fArray>> roomContours = DevidePlanes();// mpReconstruction->getContourTreeList();
	log_info("roomContours'size: %zu", roomContours.size());
	//std::cout << roomContours.size() << std::endl;
	int holeid = 0;//new	
	mStructuredPlane.resize(roomContours.size());
	for (int i=0; i < roomContours.size(); i++)
	{
		
		if (roomContours[i].size() == 0)
			continue;

		StructuredPlane stuPlane;

		//stuPlane.vertices.resize(roomContours[i].size());
		//stuPlane.holes.resize(roomContours[i].size());

		if(i < mPlaneDire.size())
			stuPlane.direction = mPlaneDire[i];
		stuPlane.normal = verticesNormal(roomContours[i].get()->_val);

		stuPlane.type = getWallType(i);
		

		cv::Point3f center(0,0,0);
		int ptcount=0;

		cv::Point3f m_max, m_min;
		
		
		//int holeid = 0; //old
		int localholeid = 0;
		auto pt_tree = roomContours[i].get();
		stuPlane.vertices = pt_tree->_val;
		ptcount += stuPlane.vertices.size();
		for (auto vertice : stuPlane.vertices)
			center += vertice;
			
		if (stuPlane.type == ePLANE_WALL)
		{
			cv::Point3f wall_max, wall_min;
			MathOperation::GetMinMaxXYZ(stuPlane.vertices, wall_max, wall_min);

			for (auto child : pt_tree->_children)
			{
				StructuredHole hole;
				hole.vertice = child->_val;
				cv::Point3f hole_max, hole_min;
				MathOperation::GetMinMaxXYZ(hole.vertice, hole_max, hole_min);
				if (hole_min.z - wall_min.z > 200.0f)
					hole.type = eHOLE_WINDOWS;
				else
					hole.type = eHOLE_DOOR;

				hole.id = holeid++;
				hole.local_id = localholeid++;
				stuPlane.holes.push_back(hole);
			}
		}

		stuPlane.center = center / ptcount;
		stuPlane.normal = UniformNormals(stuPlane.normal, stuPlane.center);

		//fake ceiling
		if (mFakeCeiling && i >= mPlane.plane_xyz.size() && stuPlane.normal.z < -0.9)
			stuPlane.type = ePLANE_CEILING;

		if (i < mPlane.plane_xyz.size())
		{
			BuildReflectImage(i, stuPlane.reflectImg);
			//BuildNormalImage(mPlane.plane_xyz[i], stuPlane.reflectImg);
		}
		mStructuredPlane[i]= stuPlane;
		
	}

	for (int i = 0; i < mStructuredPlane.size(); i++)
	{
		if (mStructuredPlane[i].vertices.size() == 0)
			continue;
		mStructuredPlane[i].vertices = SortVertices(mStructuredPlane[i].vertices, mStructuredPlane[i].normal);
		for (int j = 0; j < mStructuredPlane[i].holes.size(); j++)
		{
			
			mStructuredPlane[i].holes[j].vertice = SortVertices(mStructuredPlane[i].holes[j].vertice, mStructuredPlane[i].normal);
		}
	}

	CreateWallList();

	//compute wall/window/door width and height
	std::vector<cv::Point3f> ground_contour = GetRoomContour();
	for (auto &plane : mStructuredPlane)
		if (plane.type == ePLANE_GROUND)
			plane.vertices = ground_contour;

	for (auto &plane: mStructuredPlane)
	{
		if (plane.type != ePLANE_WALL)
			continue;
		//Find pt in Ground contour
		std::vector<cv::Point3f> pt_ground;
		for (auto pt : plane.vertices)
		{
			if (std::find(ground_contour.begin(), ground_contour.end(), pt) != ground_contour.end())
				pt_ground.push_back(pt);
		}
		float wall_width = 0.0f;
		float wall_height = 0.0f;
		
		if(plane.vertices.size() > 0)
		{ 
			cv::Point3f wall_start = plane.vertices[0];
			cv::Point3f wall_end = plane.vertices[0];
			for (auto pt : pt_ground)
			{
				if (cv::norm(pt - wall_start) > wall_width)
				{
					wall_width = cv::norm(pt - plane.vertices[0]);
					wall_end = pt;
				}
			}
			//std::cout <<"dir: "<< plane.direction <<" wid: "<< wall_width << endl;

		
			//find highest line
			cv::Point3f top_center(0,0,0);
			for (int i = 0; i < plane.vertices.size(); i++)
			{
				cv::Point3f p_center = (plane.vertices[i] + plane.vertices[(i + 1) % plane.vertices.size()]) / 2;
				if (p_center.z > top_center.z)
					top_center = p_center;
			}
			wall_height = top_center.z - (wall_start.z + wall_end.z) / 2;
		}
		plane.wall_height = wall_height;
		plane.wall_width = wall_width;
		std::vector<StructuredHole>::iterator iter;
		for (iter = plane.holes.begin(); iter != plane.holes.end(); ) {
			StructuredHole& door_window = *iter;
			log_info("door_window.vertice.size(): %zu", door_window.vertice.size());
			//std::cout << "door_window.vertice.size():" << door_window.vertice.size() << std::endl;
			if (door_window.vertice.size() == 4)
			{
				float dw_width = (cv::norm(door_window.vertice[0] - door_window.vertice[1]) +
					cv::norm(door_window.vertice[2] - door_window.vertice[3])) / 2.0f;
				float dw_height = (cv::norm(door_window.vertice[0] - door_window.vertice[3]) +
					cv::norm(door_window.vertice[1] - door_window.vertice[2])) / 2.0f;
				door_window.width = dw_width;
				door_window.height = dw_height;
				iter++;
			}
			else {
				log_info("Erased");
				//std::cout << "Erased" << std::endl;
				plane.holes.erase(iter);
			}
		}
		/*for (auto &door_window : plane.holes)
		{
			if (door_window.vertice.size() > 0)
			{
				float dw_width = (cv::norm(door_window.vertice[0] - door_window.vertice[1]) +
					cv::norm(door_window.vertice[2] - door_window.vertice[3])) / 2.0f;
				float dw_height = (cv::norm(door_window.vertice[0] - door_window.vertice[3]) +
					cv::norm(door_window.vertice[1] - door_window.vertice[2])) / 2.0f;
				door_window.width = dw_width;
				door_window.height = dw_height;
			}

		}*/
	}

	int bwall = GetBigestPlane(ePLANE_WALL);

	mPlane.plane_wall_idx = GetPlaneList(ePLANE_WALL);
	mPlane.plane_beam_idx = GetPlaneList(ePLANE_BEAM);

	int bwall_pair = bwall;
	int nextwall = bwall_pair;

	return true;
	/*do
	{
		std::cout << "wall = " << nextwall << std::endl;
	} while ((nextwall = GetNextWallId(nextwall)) != bwall_pair);*/
}

std::vector<cv::Point3f>  PointReconstruction::SortVertices(std::vector<cv::Point3f> vertices, cv::Point3f normal)
{
	std::vector<cv::Point3f>  new_vertices = vertices;
	if (abs(normal.z) < 0.3)
	{
		cv::Point3f normal_plane = -normal;
		cv::Point3f rot_axis = { 0.f, 1.f, 0.f };

		cv::Point3f cross = Util_Math::ComputeVectorCrossProduct(normal_plane, rot_axis);
		float  angle_rot = 0;

		angle_rot = (cross.z > 0.f) ?
			acosf(normal_plane.dot(rot_axis) / std::sqrt(std::pow(normal_plane.x, 2) + std::pow(normal_plane.y, 2))) :
			-acosf(normal_plane.dot(rot_axis) / std::sqrt(std::pow(normal_plane.x, 2) + std::pow(normal_plane.y, 2)));

		cv::Mat rotation_matrix = cv::Mat::zeros(3, 3, CV_32FC1);
		rotation_matrix.at<float>(0, 0) = cos(angle_rot);
		rotation_matrix.at<float>(0, 1) = -sin(angle_rot);
		rotation_matrix.at<float>(1, 0) = sin(angle_rot);
		rotation_matrix.at<float>(1, 1) = cos(angle_rot);
		rotation_matrix.at<float>(2, 2) = 1.f;


		new_vertices = MathOperation::plane_rot(rotation_matrix, vertices);
	}

	cv::Point3f wall_max, wall_min;
	MathOperation::GetMinMaxXYZ(new_vertices, wall_max, wall_min);


	for (auto it = new_vertices.begin(); it != new_vertices.end(); it++)
		*it -= wall_min;

	//find min x,y,z
	auto it_min = new_vertices.begin();
	for (auto it = new_vertices.begin(); it != new_vertices.end(); it++)
	{
		if ((it->x + it->y + it->z) < (it_min->x + it_min->y + it_min->z))
			it_min = it;
	}

	//
	int min_id = it_min - new_vertices.begin();

	int sign = 1;
	if (new_vertices[(min_id + 1) % new_vertices.size()].x < new_vertices[(min_id - 1 + new_vertices.size()) % new_vertices.size()].x)
		sign = -1;

	std::vector<cv::Point3f> vertice_tmp;

	for (int time = 0; time < vertices.size(); time++)
	{
		int curid = (min_id + vertices.size()) % vertices.size();
		vertice_tmp.push_back(vertices[(min_id + vertices.size()) % vertices.size()]);
		min_id += sign;
	}
	vertices.swap(vertice_tmp);
	return vertices;
}

cv::Point3f PointReconstruction::UniformNormals(const cv::Point3f normal, const cv::Point3f center)
{
	if (normal.dot(center) > 0.f)
	{
		return -normal;
	}
	return normal;
}


void PointReconstruction::BuildReflectImage(int id, cv::Mat & dstImg)
{
	//code struct changed by hgj
	std::vector<cv::Point3f>& points = mPlane.plane_xyz[id];
	std::vector<unsigned char>& reflects = mPlane.plane_reflect[id];
	cv::Point3f center = mPlane.plane_center[id];
	cv::Point3f normal_inner = mPlane.plane_normals[id];

	if (ComputeVectorDotProduct<float, Point3f>(normal_inner, center) > 0.f)
	{
		//point to center
		normal_inner *= -1;
	}
	BuildRefect::BuildRefectImageFuc(points, reflects, center, normal_inner, dstImg);
}



int PointReconstruction::GetBigestPlane(ePlane_Type type)
{
	std::vector<int>  planelist = GetPlaneList(type);
	std::vector<StructuredPlane> planes = GetStructuredPlanes();


	float maxdiag = -1;
	int curid = -1;
	for (auto planeid:planelist)
	{
		cv::Point3f wall_max, wall_min;
		MathOperation::GetMinMaxXYZ(planes[planeid].vertices, wall_max, wall_min);
		
		float diag = cv::norm(wall_max - wall_min);
		if (maxdiag < diag)
		{
			maxdiag = diag;
			curid = planeid;
		}
		
	}

	return curid;
}

std::vector<int> PointReconstruction::GetPlaneList(ePlane_Type type)
{
	std::vector<int>  planelist;
	std::vector<StructuredPlane> planes = GetStructuredPlanes();

	for (int i = 0; i < planes.size(); i++)
	{
		if (planes[i].vertices.size() > 0 && planes[i].type == type)
			planelist.push_back(i);
	}

	return planelist;

}

bool PointReconstruction::ReadCompass(std::string compass_path)
{
	//fill
	//mCompassValue == what?
	//std::cout << "ReadCompass TO BE implementing" << std::endl;
	if (compass_path.c_str() == nullptr)
	{
		return false;
	}
	//get stream
	std::ifstream myfile(compass_path);
	//modified by simon.jin@unre.com start
	if (!myfile.good())
		return false;
	std::string compassValue;
	std::string line;
	int read_counts = 10;
	while (!myfile.eof() && --read_counts)
	{
		getline(myfile, line);
		std::istringstream iss(line);
		compassValue = "";
		if (line.length() > 0) {

			if (line[0] == 'e')
			{
				return false;
			}
			else
			{
				iss >> compassValue;
				if (compassValue.find("ready") != -1)
					break;
				else
					continue;
			}
		}
		else
		{
			continue;
		}
	}
	//modified by simon.jin@unre.com end
	myfile.close();
	int id = compassValue.find(',');
	if (compassValue[0] == 'e')
	{
		return false;
	}
	std::string sVal = compassValue.substr(0, id);
	mCompassValue = atof(sVal.c_str());

	return true;
}

int  PointReconstruction::GetReferenceWall()
{
	return mRefWall;
}

PlaneCutResultInterface PointReconstruction::GetPlaneCutResultInterface()
{
	return mPlane;
}

std::vector<ePlane_Direction> PointReconstruction::GetPlaneDire()
{
	return mPlaneDire;
}

void PointReconstruction::ComputeReferenceWall(void)
{
	float max_score = 0;
	int   ref_wall_id = -1;
	for (auto wall_id : mPlane.plane_wall_idx)
	{
		Point3f wall_min, wall_max;
		MathOperation::GetMinMaxXYZ(mPlane.plane_xyz[wall_id], wall_max, wall_min);
		float wall_len = std::sqrtf(std::pow((wall_max.y - wall_min.y), 2.0f) + std::pow((wall_max.x - wall_min.x), 2.0f));

		float minAngle = 180.f;

		for (size_t j = 0; j < mPlane.L_shape_plane_idx.size(); j++)
		{
			if (mPlane.L_shape_plane_idx[j].first == wall_id || mPlane.L_shape_plane_idx[j].second == wall_id)
			{
				float angle = abs(90.f - Util_Math::vec3_angle_deg(mPlane.plane_normals[mPlane.L_shape_plane_idx[j].first], mPlane.plane_normals[mPlane.L_shape_plane_idx[j].second]));
				minAngle = angle < minAngle ? angle : minAngle;
			}
		}

		float door_window_lens = 0.0f;
		if (mPlane.door_window_info[wall_id].size() > 0)
		{
			for (auto door_window : mPlane.door_window_info[wall_id])
			{
				Point3f wall_min, wall_max;
				MathOperation::GetMinMaxXYZ(door_window.corners, wall_max, wall_min);
				float door_window_len = std::sqrtf(std::pow((wall_max.y - wall_min.y), 2.0f) + std::pow((wall_max.x - wall_min.x), 2.0f));
				door_window_lens += door_window_len;
			}
		}

		float angleScore = 180.f - minAngle;
		float wall_score = (wall_len - door_window_lens - 1000.f) * 0.5f + angleScore * 10;
		//cout << "Wall id is: "<< wall_idx << " score is: " << WallScore << endl;
		if (wall_score > max_score)
		{
			max_score = wall_score;
			ref_wall_id = wall_id;
		}
	}

	mRefWall = ref_wall_id;
}

float PointReconstruction::RotateByNorth(void)
{
	
	int max_wall_id = GetReferenceWall();
	cv::Point3f normal_plane = mPlane.plane_normals[max_wall_id];
	normal_plane = -UniformNormals(normal_plane, mPlane.plane_center[max_wall_id]);
	cv::Point3f rot_axis = { 0.f, 0.f, 0.f };

	std::string dir_str[] = { "UK",
		"east",
		"south",
		"west",
		"north" };
	log_info("max len wall is %d dir=%s", max_wall_id, dir_str[mPlaneDire[max_wall_id]].c_str());
	//std::cout << "max len wall is " << max_wall_id << " dir=" << dir_str[mPlaneDire[max_wall_id]] << std::endl;

	if (mPlaneDire[max_wall_id] == ePLANE_SOUTH)
		rot_axis.y = -1.0f;
	else if (mPlaneDire[max_wall_id] == ePLANE_EAST)
		rot_axis.x = 1.0f;
	else if (mPlaneDire[max_wall_id] == ePLANE_WEST)
		rot_axis.x = -1.0f;
	else
		rot_axis.y = 1.0f;

	cv::Point3f cross = Util_Math::ComputeVectorCrossProduct(normal_plane, rot_axis);
	float  angle_rot = 0;
	
	angle_rot = (cross.z > 0.f) ?
		acosf(normal_plane.dot(rot_axis) / std::sqrt(std::pow(normal_plane.x, 2) + std::pow(normal_plane.y, 2))) :
		-acosf(normal_plane.dot(rot_axis) / std::sqrt(std::pow(normal_plane.x, 2) + std::pow(normal_plane.y, 2)));
	log_info("Rotate angle is %f radian, %f degree", angle_rot, angle_rot * 180 / 3.1415926);
	/*std::cout << angle_rot << std::endl;
	std::cout << angle_rot * 180 / 3.1415926 << std::endl;*/

	cv::Mat rotation_matrix = cv::Mat::zeros(3, 3, CV_32FC1);
	rotation_matrix.at<float>(0, 0) = cos(angle_rot);
	rotation_matrix.at<float>(0, 1) = -sin(angle_rot);
	rotation_matrix.at<float>(1, 0) = sin(angle_rot);
	rotation_matrix.at<float>(1, 1) = cos(angle_rot);
	rotation_matrix.at<float>(2, 2) = 1.f;

	for(int i=0; i < mPlane.plane_xyz.size(); i++)
		mPlane.plane_xyz[i] = MathOperation::plane_rot(rotation_matrix, mPlane.plane_xyz[i]);

	//for (int i = 0; i < mPlane.plane_normals.size(); i++)
	mPlane.plane_normals = MathOperation::plane_rot(rotation_matrix, mPlane.plane_normals);
	mPlane.plane_center = MathOperation::plane_rot(rotation_matrix, mPlane.plane_center);

	for (int i = 0; i < mPlane.door_window_info.size(); i++)
		for(int j=0; j < mPlane.door_window_info[i].size(); j++)
		mPlane.door_window_info[i][j].corners = MathOperation::plane_rot(rotation_matrix, mPlane.door_window_info[i][j].corners);

	return mRorateAngle * 180/3.1415926;
}


std::vector<std::pair<int, int>> PointReconstruction::GetLShapeWallIdx(void) 
{
	return mPlane.L_shape_plane_idx;
};

std::vector<std::pair<int, int>> PointReconstruction::GetParallelWallIdx(void)
{
	return mPlane.parallel_plane_idx;
};


int PointReconstruction::GetPreWallId(int curId)
{
	int wall_id = -1;
	for (int i = 0; i < mWallClockWiseList.size(); i++)
	{
		if (mWallClockWiseList[i] == curId)
		{
			do
			{
				wall_id = mWallClockWiseList[(i - 1 + mWallClockWiseList.size()) % mWallClockWiseList.size()];
				i--;
			} while (wall_id == -1);
			break;
		}
	}
	return wall_id;
}


int PointReconstruction::GetNextWallId(int curId)
{
	int wall_id = -1;
	for (int i = 0; i < mWallClockWiseList.size(); i++)
	{
		if (mWallClockWiseList[i] == curId)
		{
			do
			{
				wall_id = mWallClockWiseList[(i + 1) % mWallClockWiseList.size()];
				i++;
			} while (wall_id == -1);
			break;
		}
	}
	return wall_id;
}

void PointReconstruction::CreateWallList(void)
{
	mWallClockWiseList.clear();
	mWallClockWiseList.resize(0);

	std::vector<cv::Point3f> ground_contour = GetRoomContour();

	//std::vector<std::vector<Tree<Point3fArray>>> roomContours = mpReconstruction->getContourTreeList();

	std::vector<int> wall_idx = GetPlaneList(ePLANE_WALL);

	mWallClockWiseList.resize(ground_contour.size());
	std::fill(mWallClockWiseList.begin(), mWallClockWiseList.end(), -1);
	for (int i =0;i < ground_contour.size(); i++)
	{
		std::vector<int> wall_pair;
		for (auto wid:wall_idx)
		{
			auto pts = mStructuredPlane[wid].vertices;
			if (std::find(pts.begin(), pts.end(), ground_contour[i]) != pts.end() &&
				std::find(pts.begin(), pts.end(), ground_contour[(i + 1) % ground_contour.size()]) != pts.end())
			{
				mWallClockWiseList[i] = wid;
				break;
			}

		}
	}
}

std::vector<int>  PointReconstruction::GetWallList(void)
{
	return mWallClockWiseList;
}

std::vector<ePlane_Direction> PointReconstruction::ComputeDirection(void)
{
	float deg = mCompassValue;
	deg = fmod((deg + 270), 360) - 180;
	float angleVal = deg*M_PI / 180;
	cv::Point3f yAxis(0, 1, 0);
	if (deg >= 0 && deg < 90)
	{
		yAxis.y = 1;
		yAxis.x = yAxis.y*tan(angleVal);
	}
	else if (deg == 90)
	{
		yAxis.x = 1;
		yAxis.y = 0;
	}
	else if (deg > 90 && deg < 180)
	{
		angleVal = angleVal - M_PI_2;
		yAxis.x = 1;
		yAxis.y = -yAxis.x*tan(angleVal);
	}
	else if (deg == 180 || deg == -180)
	{
		yAxis.x = 0;
		yAxis.y = -1;
	}
	else if (deg < 0 && deg > -90)
	{
		yAxis.y = 1;
		yAxis.x = yAxis.y*tan(angleVal);
	}
	else if (deg == -90)
	{
		yAxis.x = -1;
		yAxis.y = 0;
	}
	else if (deg < -90 && deg > -180)
	{
		angleVal = angleVal + M_PI_2;
		yAxis.x = -1;
		yAxis.y = -yAxis.x*tan(angleVal);
	}

	std::vector<cv::Point3f> plane_normals = mPlane.plane_normals;
	for (int i = 0; i < plane_normals.size(); i++)
		plane_normals[i] = UniformNormals(mPlane.plane_normals[i], mPlane.plane_center[i]);


	std::vector<int> plane_idx;
	plane_idx.insert(plane_idx.end(), mPlane.plane_wall_idx.begin(), mPlane.plane_wall_idx.end());
	plane_idx.insert(plane_idx.end(), mPlane.plane_beam_idx.begin(), mPlane.plane_beam_idx.end());

	std::vector<std::pair<float, int>> angle_idx;
	for (auto i : plane_idx)
	{
		//compute project angle:
		cv::Point3f N1(plane_normals[i].x, plane_normals[i].y, 0.0f);
		cv::Point3f nProject(-N1.x, -N1.y, 0);

		float n1 = cv::norm(nProject)*cv::norm(yAxis);
		float dotRes = nProject.dot(yAxis);
		cv::Point3f outCross = yAxis.cross(nProject);

		std::pair<float, int> deg;
		if (n1 != 0.0f)
		{
			float cosAngle = dotRes / n1;
			float angle1 = std::acos(cosAngle);
			if (outCross.z < 0)
			{
				angle1 = angle1;
			}
			else if (outCross.z > 0)
			{
				angle1 = 2 * M_PI - angle1;
			}
			else
			{
				//angle1 
			}
			//std::cout << "angle:" << angle1 * 180 / M_PI << endl;
			deg.first = angle1;
			deg.second = i;
		}
		angle_idx.push_back(deg);
	}
	
	std::sort(angle_idx.begin(), angle_idx.end());


	std::vector<ePlane_Direction>  directions;
	directions.resize(mPlane.plane_xyz.size());
	std::fill(directions.begin(), directions.end(), ePLANE_DIR_UNKOWN);

	for (int i = 0; i < angle_idx.size(); ++i)
	{
		float angle = 180 * angle_idx[i].first / M_PI;
		int id = angle_idx[i].second;
		//wall id and direction
		std::pair<int, string> wallDirection;
		//North
		wallDirection.first = id;
		if ((angle >= 0 && angle <= 45) || (angle > 315 && angle <= 360))
			directions[id] = ePLANE_NORTH;
		//East
		else if (angle > 45 && angle <= 135)
			directions[id] = ePLANE_EAST;
		//South
		else if (angle > 135 && angle <= 225)
			directions[id] = ePLANE_SOUTH;
		else if (angle > 225 && angle <= 315)
			directions[id] = ePLANE_WEST;
		//std::cout << "wall id =" << id << "wall direction = " << directions[id] << endl;
	}

	return directions;
}

void PointReconstruction::MovetoOneMeterLine(float oneMeterPos)
{
	float move_z = 1.0f - oneMeterPos;

	for (int i = 0; i < mPlane.plane_xyz.size(); i++)
	{
		for (auto &plane_xyz : mPlane.plane_xyz[i])
			plane_xyz.z += move_z;
		
		mPlane.plane_center[i].z += move_z;
		for (auto &door_window : mPlane.door_window_info[i])
			for(auto &corner_pt:door_window.corners)
				corner_pt.z += move_z;
	}
}

void PointReconstruction::UpdateDoorWindowInfo(std::vector<StructuredPlane> updatePlanes)
{
	std::vector<std::vector<Tree<Point3fArray>>> roomContours = mpReconstruction->getContourTreeList();

	for (auto updateplane : updatePlanes)
	{
		if (updateplane.vertices.size() > 0)
		{
			int id = updateplane.id;
			mStructuredPlane[id].holes = updateplane.holes;

			if (roomContours[id].size())
			{
				roomContours[id][0].Clear_Node();

				for (auto hole : mStructuredPlane[updateplane.id].holes)
				{
					roomContours[id][0].Add_Node(roomContours[id][0].get()->_val, hole.vertice);
				}
			}
		}
	}

	mpReconstruction->UpdateContourTreeList(roomContours);
}

void PointReconstruction::InsertFakePoint(std::vector<cv::Point3f> contour_new, std::vector<int>  wallList)
{
	mContour = contour_new;
	mWallClockWiseList = wallList;

	for (auto &plane : mStructuredPlane)
		if (plane.type == ePLANE_GROUND)
			plane.vertices = mContour;

}
void PointReconstruction::InsertFakePoint(std::vector<int> insertMap)
{
	std::sort(insertMap.begin(), insertMap.end());
	std::vector<int> insertidx;
	for (int i = 1; i <= insertMap.size(); i++)
		insertidx.push_back(insertMap[i] - i);

	std::vector<cv::Point3f> contour_new;
	std::vector<int>   wallList;

	for (int i = 0; i < mContour.size(); i++)
	{
		auto iter_map = std::find(insertMap.begin(), insertMap.end(), i);
		if (iter_map != insertMap.end())
		{
			contour_new.push_back(mContour[i]);
			contour_new.push_back((mContour[i] + mContour[(i-1)% mContour.size()])/2);

			wallList.push_back(mWallClockWiseList[i]);
			wallList.push_back(mWallClockWiseList[i]);
		}
		else
		{
			contour_new.push_back(mContour[i]);
			wallList.push_back(mWallClockWiseList[i]);
		}
	}
	mContour = contour_new;
	mWallClockWiseList = wallList;

	for (auto &plane : mStructuredPlane)
		if (plane.type == ePLANE_GROUND)
			plane.vertices = mContour;
	
	//contour_new.insert(contour_new.begin() + insertMap)
}