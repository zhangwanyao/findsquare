#include "bim_loc_roi.h"
#include <opencv2\core.hpp>
#include "../utility/util_img.h"
#include "../utility/util_line_intersection.h"
#include "bim_intxn.h"
#include "bim_proj.h"
#include "InOutData.h"
#include "../DoorWindowDetector/MeasureDoorWindow.h"
#include "util_math.hpp"
#include "GeometryFilter.h"
#include "log.h"
#include "cmath"
#include "bim_filter_plane.h"

//#define IMG_ROI_DEBUG

/**
* \brief constructor
*/
BIM_LOC_ROI::BIM_LOC_ROI() :
	m_type_roi(0)
{
}
BIM_LOC_ROI::BIM_LOC_ROI(int type_roi) :
	m_type_roi(type_roi)
{

}
/**
*
* \brief destructor
*/
BIM_LOC_ROI::~BIM_LOC_ROI()
{
	clear();
}
/**
* \ brief clear
*/
void BIM_LOC_ROI::clear() {
	m_idx_all.clear();
	m_idx_valid.clear();
	m_idx_wall.clear();
	m_idx_floor.clear();
	m_idx_hCeil.clear(); 
	if (m_bbox_g != nullptr)
	{
		delete m_bbox_g;
		m_bbox_g = nullptr;
	}
}
/**
* \ brief update plane index list
*/
void BIM_LOC_ROI::updateIdxList(Doc_BIM &doc_bim) {
	// set idx_proc according to conditions
	std::vector<ATTR_BIM_PLANE> type_ceiling;
	type_ceiling.push_back(ATTR_BIM_PLANE::CEILING);
	//type_ceiling.push_back(ATTR_BIM_PLANE::HORIZ);
	doc_bim.getIdxAll(m_idx_all);
	doc_bim.getIdxList(ATTR_BIM_PLANE::VALID, m_idx_valid);
	doc_bim.getIdxList(ATTR_BIM_PLANE::WALL, m_idx_wall);
	doc_bim.getIdxList(ATTR_BIM_PLANE::FLOOR, m_idx_floor);
	doc_bim.getIdxList(ATTR_BIM_PLANE::BEAM, m_idx_beam);
	doc_bim.getIdxList(type_ceiling, m_idx_hCeil);
	type_ceiling.clear();
}
/**
* \brief locate roi on image
*/
void BIM_LOC_ROI::createIntxnGraph(
	const std::vector<Doc_BIM_Plane *> &docs,
	const std::vector<unsigned int> &idx_proc,
	Topo_Graph *graph)
{
	// compute intersection
	std::vector<std::pair<unsigned int, unsigned int>> pairIntxn;
	BIM_Intxn::compute(docs, idx_proc, 50, pairIntxn);
	// build graph of walls
	std::vector<Topo_Node *> nodes;
	nodes.resize(docs.size());
	for (size_t i = 0; i < docs.size(); i++) {
		// init node
		nodes[i] = new Topo_Node();
		nodes[i]->setID(docs[i]->getIdxPlane());
	}
	graph->create(nodes, pairIntxn);
	nodes.clear();
	pairIntxn.clear();
}

/**
* \brief locate roi on image
*/
bool BIM_LOC_ROI::locROI_img_0(Doc_BIM &doc_bim, std::vector<std::vector<cv::Point3f>> &pts, cv::Mat &img_roi)
{
	// data validation
	bool isValid = true;
	std::vector<Doc_BIM_Plane *> docs; // all docs
	doc_bim.getDocPlane(docs);
	if (docs.empty()) isValid = false; // check if docs init
	if (!isValid) {
		docs.clear();
		return false;
	}
	// set idx_proc according to conditions
	updateIdxList(doc_bim);
	// projection
	bool isLocROI = false;
	if (!m_idx_valid.empty() && !m_idx_hCeil.empty()) {
		cv::Mat img_valid;
		cv::Mat img_ceiling;

		BIM_Proj* proj_optimize = new BIM_Proj(pts, m_idx_valid, m_type_roi);
		proj_optimize->proj_global(doc_bim, pts, m_idx_valid, m_idx_valid, img_valid);

		m_bbox_g = new BBox3D();
		m_bbox_g->m_min = proj_optimize->m_bbox_g->m_min;
		m_bbox_g->m_max = proj_optimize->m_bbox_g->m_max;

		isLocROI = proj_optimize->proj_global_multiceiling_optimize(doc_bim, pts, m_idx_valid, m_idx_hCeil, img_ceiling);
		Util_Img::cvtRGB2BIN(img_ceiling, img_roi);
		delete proj_optimize;
		proj_optimize = nullptr;
		cv::dilate(img_roi, img_roi, cv::Mat::ones(9, 9, CV_8U));
		/*if (!img_ceiling.empty()) {
			cv::imwrite("proj_img_ceiling.png", img_ceiling);
			std::cout << "[投影保存] 已保存 img_ceiling 到本地" << std::endl;
		}*/
	}
	
	// clear
	docs.clear();
	return isLocROI;
}

/**
* \brief locate roi on image
*/
//#include "../Common/InOutData.h"
bool BIM_LOC_ROI::locROI_img_1(Doc_BIM &doc_bim, std::vector<std::vector<cv::Point3f>> &pts, cv::Mat &img_roi)
{
	// data validation
	std::vector<Doc_BIM_Plane *> docs; // all docs
	doc_bim.getDocPlane(docs);
	if (docs.empty()) m_isValid = false; // check if docs init
	if (!m_isValid) {
		docs.clear();
		return false;
	}
	bool isLocROI = false;

	if (!m_withInclinedPlane) {
		for (int idx = 0; idx < pts.size(); idx++)
		{
			cv::Point3f ori = cv::Point3f(0.f, 0.f, 0.f);
			float angle = Util_Math::CalLine2PlaneAngle<float, cv::Point3f>(ori, docs[idx]->getCenter(), docs[idx]->getNormal());
			if (abs(angle) < 15.f &&
				abs(docs[idx]->getNormal().x < 0.98f) && abs(docs[idx]->getNormal().y < 0.98f))// 78.5 degree
			{
				State_BIM_Plane type = docs[idx]->getType();
				type.clearState();
				docs[idx]->setType(type);
				pts[idx].clear();
			}
		}
	}

	// set idx_proc according to conditions
	updateIdxList(doc_bim);
	//std::vector<unsigned int> m_idx_wall = m_idx_wall;

	// projection
	if (!m_idx_valid.empty() && !m_idx_wall.empty()) {
		cv::Mat img_valid;
		cv::Mat img_wall;
		cv::Mat img_floor;
		cv::Mat img_hCeil;
		//changed by yu.liang@unre.com start
		//2020/12/15
		std::vector<unsigned int> m_idx_hCeil_bak = m_idx_hCeil;
		std::vector<unsigned int> m_idx_hFloor_bak = m_idx_floor;
		BIM_Proj* proj_optimize = new BIM_Proj(pts, m_idx_valid, m_type_roi);
		proj_optimize->proj_global(doc_bim, pts, m_idx_valid, m_idx_valid, img_valid);
		m_bbox_g = new BBox3D();
		m_bbox_g->m_min = proj_optimize->m_bbox_g->m_min;
		m_bbox_g->m_max = proj_optimize->m_bbox_g->m_max;
		isLocROI = proj_optimize->proj_global_wall_optimize(doc_bim, pts, m_idx_valid, m_idx_wall, m_idx_roomwall, m_idx_beam, img_wall);
		if (!isLocROI)
		{
			docs.clear();
			delete proj_optimize;
			proj_optimize = nullptr;
			return false;
		}
		if (m_idx_hCeil.size() > 30)
		{
			proj_optimize->proj_global_floor_optimize(doc_bim, pts, m_idx_valid, m_idx_floor, img_wall, img_floor);

			if (m_idx_floor.size() > 0) {
				Util_Img::bitOR_RGB(img_floor, img_wall, img_roi);
			}
			else {
				m_idx_floor = m_idx_hFloor_bak;
				//Util_Img::bitAND_RGB(img_hCeil, img_wall, img_roi);
				Util_Img::cvtRGB2BIN(img_wall, img_roi);
			}
		}
		else
		{ 
			proj_optimize->proj_global_ceil_optimize(doc_bim, pts, m_idx_valid, m_idx_hCeil, img_wall, img_hCeil);

			if (m_idx_hCeil.size() > 0){
				Util_Img::bitOR_RGB(img_hCeil, img_wall, img_roi);
			}
			else {
				m_idx_hCeil = m_idx_hCeil_bak;
				//Util_Img::bitAND_RGB(img_hCeil, img_wall, img_roi);
				Util_Img::cvtRGB2BIN(img_wall, img_roi);
			}
		}
		delete proj_optimize;
		proj_optimize = nullptr;		
		//cv::dilate(img_roi, img_roi, cv::Mat::ones(9, 9, CV_8U));
		//std::string debug_path = "F:\\Unre\\BIM\\test\\3";
		//imwrite(debug_path + "\\img_wall_.jpg", img_wall);
		//imwrite(debug_path + "\\img_hCeil_.jpg", img_hCeil);
		//imwrite(debug_path + "\\img_valid_.jpg", img_valid);
		//imwrite(debug_path + "\\img_wORc_g_.jpg", img_roi);
		//changed by yu.liang@unre.com end	
	}

	// clear
	//m_idx_wall.clear();
	docs.clear();
	return isLocROI;
}
/**
* \brief locate roi on image
*/
bool BIM_LOC_ROI::locROI_img_5(Doc_BIM &doc_bim, std::vector<std::vector<cv::Point3f>> &pts, cv::Mat &img_roi)
{
	// data validation
	bool isValid = true;
	std::vector<Doc_BIM_Plane *> docs; // all docs
	doc_bim.getDocPlane(docs);
	if (docs.empty()) isValid = false; // check if docs init
	if (!isValid) {
		docs.clear();
		return false;
	}
	// set idx_proc according to conditions
	updateIdxList(doc_bim);
	bool isLocROI = false;

	// projection
	if (!m_idx_valid.empty() && !m_idx_hCeil.empty())
	{
		cv::Mat img_valid;
		cv::Mat img_wall;
		cv::Mat img_ceiling;
		BIM_Proj* proj_optimize = new BIM_Proj(pts, m_idx_valid, m_type_roi);
		proj_optimize->proj_global(doc_bim, pts, m_idx_valid, m_idx_valid, img_valid);
		m_bbox_g = new BBox3D();
		m_bbox_g->m_min = proj_optimize->m_bbox_g->m_min;
		m_bbox_g->m_max = proj_optimize->m_bbox_g->m_max;
		isLocROI = proj_optimize->proj_global_wall_beam_optimize(doc_bim, pts, m_idx_valid, m_idx_wall, m_idx_beam, img_wall);
		if (!isLocROI)
		{
			docs.clear();
			delete proj_optimize;
			proj_optimize = nullptr;
			return false;
		}
		std::vector<unsigned int> m_idx_hCeil_bak = m_idx_hCeil;
		isLocROI = proj_optimize->proj_global_ceil_beam_optimize(doc_bim, pts, m_idx_valid, m_idx_hCeil, m_idx_beam, img_ceiling);

		cv::Mat img_wORc_g, img_wORc;
		if (m_idx_hCeil.size() > 0) {
			Util_Img::bitOR_RGB(img_ceiling, img_wall, img_roi);
		}
		else {
			m_idx_hCeil = m_idx_hCeil_bak;
			//Util_Img::bitAND_RGB(img_hCeil, img_wall, img_roi);
			Util_Img::cvtRGB2BIN(img_wall, img_roi);
		}
		delete proj_optimize;
		proj_optimize = nullptr;
		cv::dilate(img_roi, img_roi, cv::Mat::ones(3, 3, CV_8U));
	}
	docs.clear();
	return isLocROI;
}


bool BIM_LOC_ROI::locROI_img_6(Doc_BIM& doc_bim, std::vector<std::vector<cv::Point3f>>& pts, cv::Mat& img_roi,int station_size, bool cut_poly)
{
	// data validation
	std::vector<Doc_BIM_Plane*> docs; // all docs
	doc_bim.getDocPlane(docs);
	if (docs.empty()) m_isValid = false; // check if docs init
	if (!m_isValid) {
		docs.clear();
		return false;
	}
	bool isLocROI = false;
	for (int idx = 0; idx < pts.size(); idx++)
	{
		cv::Point3f ori = cv::Point3f(0.f, 0.f, 0.f);
		float angle = Util_Math::CalLine2PlaneAngle<float, cv::Point3f>(ori, docs[idx]->getCenter(), docs[idx]->getNormal());

		State_BIM_Plane curr_type = docs[idx]->getType();
		bool is_floor = curr_type.isFloor();

		if (abs(angle) < 10.f &&
			abs(docs[idx]->getNormal().x < 0.98f) && abs(docs[idx]->getNormal().y < 0.98f) && !docs[idx]->getType().isFloor() && !docs[idx]->getType().isCeiling())// 78.5 degree, (not wall)
		{
			std::cout << "[警告]平面" << idx << "被误判，清空状态和点云！该平面是否为地面=" << (is_floor ? "Y" : "N") << std::endl;
			State_BIM_Plane type = docs[idx]->getType();
			type.clearState();
			docs[idx]->setType(type);
			pts[idx].clear();
		}
	}
	updateIdxList(doc_bim);

#if 0
	auto dumpWallCount = [&](const char* tag) {
		int cnt = 0;
		std::vector<Doc_BIM_Plane*> docs;
		doc_bim.getDocPlane(docs);
		for (auto* d : docs)
			if (d->getType().isWall()) cnt++;
		std::cout << tag << " wall count = " << cnt << std::endl;
		};
#endif

	// projection
	if (!m_idx_valid.empty() && !m_idx_wall.empty()) {
		cv::Mat img_valid;
		cv::Mat img_wall;
		cv::Mat img_floor;
		cv::Mat img_hCeil;

		std::vector<unsigned int> m_idx_hCeil_bak = m_idx_hCeil;
		std::vector<unsigned int> m_idx_hfloor_bak = m_idx_floor;
		BIM_Proj* proj_optimize = new BIM_Proj(pts, m_idx_valid, m_type_roi);
		proj_optimize->proj_global(doc_bim, pts, m_idx_valid, m_idx_valid, img_valid);
#if 0
		if (!img_valid.empty()) {
			cv::imwrite("proj_img_valid.png", img_valid);
			std::cout << "[投影保存] 已保存 img_valid 到本地" << std::endl;
		}
		else {
			std::cout << "[投影保存] img_valid 为空，无法保存" << std::endl;
		}
#endif

		m_bbox_g = new BBox3D();
		m_bbox_g->m_min = proj_optimize->m_bbox_g->m_min;
		m_bbox_g->m_max = proj_optimize->m_bbox_g->m_max;

		RemoveNoiseWall(doc_bim, pts);

		updateIdxList(doc_bim);

		isLocROI = proj_optimize->proj_global_wall_optimize(doc_bim, pts, m_idx_valid, m_idx_wall, m_idx_roomwall, m_idx_beam, img_wall);

		proj_optimize->proj_global_floor_optimize(doc_bim, pts, m_idx_valid, m_idx_floor, img_wall, img_floor);;
		bool isLocROI_ceil = proj_optimize->proj_global_multiceiling_optimize(doc_bim, pts, m_idx_valid, m_idx_hCeil, img_hCeil);

#ifdef IMG_ROI_DEBUG
		if (!img_hCeil.empty()) {
			cv::imwrite("proj_img_hCeil_origin.png", img_hCeil);
			std::cout << "[投影保存] 已保存 img_hCeil 到本地" << std::endl;
		}
#endif
		if (station_size == 1 && !cut_poly)
		{
			cv::Mat img_hCeil_bin;
			Util_Img::cvtRGB2BIN(img_hCeil, img_hCeil_bin);

			std::vector<std::vector<cv::Point>> contours;
			std::vector<cv::Vec4i> hierarchy;
			cv::findContours(img_hCeil_bin, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
			int best_contour_idx = 0;
			float best_score = -FLT_MAX;
			if (!contours.empty()) {
				for (size_t i = 0; i < contours.size(); ++i) {
					cv::Moments moments = cv::moments(contours[i]);
					cv::Point2f bottom_left(0.f, img_hCeil_bin.rows - 1.f);
					double area = fabs(cv::contourArea(contours[i]));
					float min_dist = FLT_MAX;
					for (const auto& pt : contours[i]) {
						float dist = cv::norm(cv::Point2f(pt) - bottom_left);
						if (dist < min_dist) {
							min_dist = dist;
						}
					}
#ifdef IMG_ROI_DEBUG
					cout << "轮廓索引=" << i << " 面积=" << area
						<< "距离=" << min_dist << std::endl;
#endif
					if (area > 2000) {
						float score = (500 - min_dist) * 40 + area;
						if (score > best_score) {
							best_score = score;
							best_contour_idx = static_cast<int>(i);
						}
					}
				}
			}

			cv::Mat img_hCeil_filtered = cv::Mat::zeros(img_hCeil_bin.size(), CV_8UC1);
			cv::drawContours(img_hCeil_filtered, contours, best_contour_idx, cv::Scalar(255), -1);
			cv::cvtColor(img_hCeil_filtered, img_hCeil, cv::COLOR_GRAY2BGR);

#ifdef IMG_ROI_DEBUG
			double best_area = fabs(cv::contourArea(contours[best_contour_idx]));
			std::cout << "[天花板聚类] 保留距离(0,0)最远的"
				<< "聚类，索引=" << best_contour_idx << " 面积=" << best_area
				<< "，剔除其他聚类数量：" << contours.size() - 1 << std::endl;
#endif
		}
		
		bool roi_or = Util_Img::bitOR_RGB(img_hCeil, img_wall, img_roi);

#ifdef IMG_ROI_DEBUG
		if (roi_or) {
			std::cout << "[投影融合] img_hCeil 和 img_wall 融合成功" << std::endl;
		}
		else {
			std::cout << "[投影融合] img_hCeil 和 img_wall 融合失败" << std::endl;
		}
		if (!img_wall.empty()) {
			cv::imwrite("proj_img_wall.png", img_wall);
			std::cout << "[投影保存] 已保存 img_wall 到本地" << std::endl;
		}
		else {
			std::cout << "[投影保存] img_wall 为空，无法保存" << std::endl;
		}
		if (!img_floor.empty()) {
			cv::imwrite("proj_img_floor.png", img_floor);
			std::cout << "[投影保存] 已保存 img_floor 到本地" << std::endl;
		}
		else {
			std::cout << "[投影保存] img_floor 为空，无法保存" << std::endl;
		}
		if (!img_hCeil.empty()) {
			cv::imwrite("proj_img_hCeil.png", img_hCeil);
			std::cout << "[投影保存] 已保存 img_hCeil 到本地" << std::endl;
		}
		else {
			std::cout << "[投影保存] img_hCeil 为空，无法保存" << std::endl;
		}
#endif

		if ((0 < m_idx_floor.size() && m_idx_floor.size() == 1) || img_hCeil.empty()) {
			cv::Mat img_floor_bin;
			Util_Img::cvtRGB2BIN(img_floor, img_floor_bin);
			Util_Img::bitOR(img_roi, img_floor_bin, img_roi);
		}

		if (!isLocROI_ceil || !isLocROI)
		{
			img_roi = 255;
		}

		if (station_size == 1 && !cut_poly) {
			cv::Mat img_roi_gray, img_roi_bin;
			if (img_roi.channels() == 3) {
				cv::cvtColor(img_roi, img_roi_gray, cv::COLOR_BGR2GRAY);
			}
			else {
				img_roi_gray = img_roi.clone();
			}
			cv::threshold(img_roi_gray, img_roi_bin, 1, 255, cv::THRESH_BINARY);

			std::vector<std::vector<cv::Point>> contours_roi;
			std::vector<cv::Vec4i> hierarchy_roi;
			cv::findContours(img_roi_bin, contours_roi, hierarchy_roi, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

			if (!contours_roi.empty()) {
				int max_area_idx = 0;
				double max_area = fabs(cv::contourArea(contours_roi[0]));
				for (size_t i = 1; i < contours_roi.size(); ++i) {
					double area = fabs(cv::contourArea(contours_roi[i]));
					if (area > max_area) {
						max_area = area;
						max_area_idx = i;
					}
				}

				cv::Mat img_roi_filtered = cv::Mat::zeros(img_roi_bin.size(), CV_8UC1);
				cv::drawContours(img_roi_filtered, contours_roi, max_area_idx, cv::Scalar(255), -1);

				if (img_roi.channels() == 3) {
					cv::cvtColor(img_roi_filtered, img_roi, cv::COLOR_GRAY2BGR);
				}
				else {
					img_roi = img_roi_filtered.clone();
				}

				std::cout << "[img_roi过滤] 总轮廓数：" << contours_roi.size()
					<< " 保留最大轮廓面积：" << max_area << std::endl;
			}
			else {
				std::cout << "[img_roi过滤] 未提取到任何轮廓，img_roi保持不变" << std::endl;
			}
		}

#ifdef IMG_ROI_DEBUG
		if (!img_roi.empty()) {
			cv::imwrite("proj_img_roi.png", img_roi);
			std::cout << "[投影保存] 已保存 img_roi 到本地" << std::endl;
		}
		else {
			std::cout << "[投影保存] img_roi 为空，无法保存" << std::endl;
		}
#endif
		delete proj_optimize;
		proj_optimize = nullptr;
		isLocROI = true; //set to true anyway

	}

#if 0
	int after_floor_count_final = 0;
	int after_ceiling_count_final = 0;
	int after_wall_count_final = 0;
	int after_beam_count_final = 0;
	for (size_t i = 0; i < docs.size(); ++i) {
		State_BIM_Plane after_type = docs[i]->getType();
		if (after_type.isFloor()) after_floor_count_final++;
		if (after_type.isCeiling()) after_ceiling_count_final++;
		if (after_type.isWall()) after_wall_count_final++;
		if (after_type.isBeam()) after_beam_count_final++;
	}

	std::cout << "===== RemoveNoiseWall后结果验证 =====" << std::endl;
	std::cout << "地面数量：" << after_floor_count_final << std::endl;
	std::cout << "天花板数量：" << after_ceiling_count_final << std::endl;
	std::cout << "墙面数量：" << after_wall_count_final << std::endl;
	std::cout << "梁数量：" << after_beam_count_final << std::endl;
	std::cout << "====================================" << std::endl;
#endif

	// clear
	//m_idx_wall.clear();
	docs.clear();
	return isLocROI;
}


bool IsPointInBox(const std::vector<std::vector<float>> window_corner_minmax_xyz,
	cv::Point3f point)
{
	for (int i = 0; i < window_corner_minmax_xyz.size(); i++)
	{
		if (window_corner_minmax_xyz[i][0] <= point.x && point.x <= window_corner_minmax_xyz[i][1] &&
			window_corner_minmax_xyz[i][2] <= point.y && point.y <= window_corner_minmax_xyz[i][3] &&
			window_corner_minmax_xyz[i][4] <= point.z && point.z <= window_corner_minmax_xyz[i][5])
		{
			return true;
		}
	}
	return false;
}

bool FindBoxMinMax(const std::vector<BBox3D>& box_corner_xyz,
	const std::vector<int>& plane_idx,
	const std::vector<cv::Point3f>& scene_plane_normals,
	const std::vector<cv::Point3f>& scene_plane_center,
	std::vector<std::vector<float>>& window_corner_minmax_xyz,
	float dilate_radius, float inner_ext, float outer_ext)
{
	for (int i = 0; i < box_corner_xyz.size(); i++)
	{
		float min_x = box_corner_xyz[i].m_min.x;
		float max_x = box_corner_xyz[i].m_max.x;

		float min_y = box_corner_xyz[i].m_min.y;
		float max_y = box_corner_xyz[i].m_max.y;

		float min_z = box_corner_xyz[i].m_min.z - dilate_radius;
		float max_z = box_corner_xyz[i].m_max.z + dilate_radius;

		if (i < plane_idx.size())
		{
			if (abs(scene_plane_normals[plane_idx[i]].y) > 0.75f)
			{
				min_x -= dilate_radius;
				max_x += dilate_radius;

				if (scene_plane_center[plane_idx[i]].y > 0.f)
				{
					min_y -= inner_ext;
					max_y += outer_ext;
				}
				else
				{
					min_y -= outer_ext;
					max_y += inner_ext;
				}
			}
			else if (abs(scene_plane_normals[plane_idx[i]].x) > 0.75f)
			{
				min_y -= dilate_radius;
				max_y += dilate_radius;

				if (scene_plane_center[plane_idx[i]].x > 0.f)
				{
					min_x -= inner_ext;
					max_x += outer_ext;
				}
				else
				{
					max_x += inner_ext;
					min_x -= outer_ext;
				}
			}
		}

		window_corner_minmax_xyz.push_back({ min_x, max_x, min_y, max_y, min_z, max_z });
	}
	return true;
}

void RemoveIntersectWall(Doc_BIM &doc_bim,
	std::vector<std::vector<cv::Point3f>> &pts,
	std::vector<std::vector<uchar>> & scene_plane_reflect,
	std::vector<Point3f> normals,
	std::vector<unsigned int> &idx_wall,
	std::vector<unsigned int> &wall_need_to_keep,
	BBox3D* m_bbox_g)
{
	std::vector<Doc_BIM_Plane *> docs; // all docs
	doc_bim.getDocPlane(docs);

	// compute projection parameters
	cv::Point3f bbox_diag = m_bbox_g->m_max - m_bbox_g->m_min;
	float vl = MAX(bbox_diag.x, bbox_diag.y) / 512.f;  //why devided by 512?
	unsigned int extent_x = bbox_diag.x / vl + 10;
	unsigned int extent_y = bbox_diag.y / vl + 10;

	size_t n_model = pts.size();
	std::vector<std::vector<cv::Point2i>> line_endpoints(n_model, std::vector<cv::Point2i>(3, cv::Point2i(0, 0)));
	cv::Point2i ori = cv::Point2i(int(floor((-m_bbox_g->m_min.x + 1) / vl)), int(floor((-m_bbox_g->m_min.y + 1) / vl)));

	for (auto idx : idx_wall)
	{
		cv::Mat img_temp;
		docs[idx]->getImgProjGlobal(img_temp);
		cvtColor(img_temp, img_temp, CV_RGB2GRAY);
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Point> contours_merge;
		std::vector<cv::Vec4i> hierarchy;
		findContours(img_temp, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point());

		for (auto contour : contours)
		{
			contours_merge.insert(contours_merge.end(), contour.begin(), contour.end());
		}

		cv::RotatedRect rrt = cv::minAreaRect(contours_merge);
		//changed by yu.liang 2021/1/6 end
		cv::Point2i cpt = rrt.center;
		cv::Point2f ppts[4];
		rrt.points(ppts);

		cv::Point2i endpoint_a, endpoint_b;

		std::vector<std::vector<cv::Point> > contour_lines;

		if (rrt.size.width > rrt.size.height) {
			contour_lines = { {ppts[1],ppts[2]} ,{ppts[0],ppts[3]} };
		}
		else {
			contour_lines = { {ppts[2],ppts[3]} ,{ppts[0],ppts[1]} };
		}
		float distA = MathOperation::PointToLinesegDist(ori, contour_lines[0][0], contour_lines[0][1]);
		float distB = MathOperation::PointToLinesegDist(ori, contour_lines[1][0], contour_lines[1][1]);
		if (distA > distB) {
			endpoint_a = contour_lines[0][0];
			endpoint_b = contour_lines[0][1];
		}
		else {
			endpoint_a = contour_lines[1][0];
			endpoint_b = contour_lines[1][1];
		}

		line_endpoints[idx] = { endpoint_a ,endpoint_b,  (endpoint_a + endpoint_b) / 2 };
	}

	//remove the wrong wall idx
	// changed by yu.liang@unre.com 2021/1/4 start
	std::vector<unsigned int> idx_wall_bak = idx_wall;

	std::vector<bool> wallIdx_need_to_be_deleted(n_model, false);
	for (int i = 0; i < idx_wall.size(); i++)
	{
		if (pts[idx_wall[i]].size() == 0) continue;
		if (!wall_need_to_keep.empty() && Util_Math::isIdxInList(idx_wall[i], wall_need_to_keep)) continue;

		cv::Point2i dir_a = line_endpoints[idx_wall[i]][1] - line_endpoints[idx_wall[i]][0];
		cv::Point2i dir_b = line_endpoints[idx_wall[i]][2] - ori;
		float cos_v = dir_a.dot(dir_b) / (cv::norm(dir_a)*cv::norm(dir_b));
		if (abs(cos_v) > 0.99)
		{
			int cur_id = idx_wall[i];

			int  wall_p_l_counts = 0;
			for (auto id : idx_wall)
			{
				if (cur_id == id)
					continue;

				float angle = acos(normals[cur_id].dot(normals[id]))*180.f / 3.1415926f;

				if (angle < 15 || (180.0f - angle) < 15 || abs(90 - angle) < 15)
					wall_p_l_counts++;
			}
			if(wall_p_l_counts/(float)idx_wall.size() < 0.4)
				wallIdx_need_to_be_deleted[idx_wall[i]] = true;
		}
	}


	idx_wall.clear();
	for (auto idx : idx_wall_bak)
	{
		if (wallIdx_need_to_be_deleted[idx])
		{
			pts[idx].clear();
			scene_plane_reflect[idx].clear();
		}
		else
		{
			idx_wall.push_back(idx);
		}
	}
}
//add by liang yu

/**
*\
*\changed by yu.liang@unre.com
*\2020/12/15
*/
void BIM_LOC_ROI::locROI_cull(Doc_BIM &doc_bim,
	PlaneCutResultInterface& cutResult,
	const std::vector<cv::Point3f> & scene_plane_normals,
	const std::vector<cv::Point3f> & scene_plane_centers,
	const cv::Mat &img_roi)
{
	// data validation
	if (img_roi.cols < 1 || img_roi.rows < 1)
	{
		m_isValid = false;
		return;
	}

	cv::Mat mask;
	if (img_roi.channels() == 1) {
		mask = img_roi.clone();
	}
	else {
		cv::cvtColor(img_roi, mask, cv::COLOR_BGR2GRAY);
		std::cout << "[locROI_cull] img_roi转为单通道灰度掩码" << std::endl;
	}

	cv::Mat mask3 = mask.clone();
	cv::dilate(mask3, mask3, cv::Mat::ones(3, 3, CV_8U));
	cv::Mat mask9 = mask.clone();
	cv::dilate(mask9, mask9, cv::Mat::ones(9, 9, CV_8U));
	cv::Mat mask12 = mask.clone();
	cv::dilate(mask12, mask12, cv::Mat::ones(12, 12, CV_8U));


	std::vector<Doc_BIM_Plane *> docs; // all docs
	doc_bim.getDocPlane(docs);
	if (docs.empty()) m_isValid = false; // check if docs init
	if (docs.size() != cutResult.plane_xyz.size()) m_isValid = false; // check if doc and model match
	if (!m_isValid) {
		docs.clear();
		return;
	}

#if 0
	std::cout << "\n===== [locROI_cull] 初始墙列表 =====\n";
	for (auto w : m_idx_wall)
	{
		std::cout << "Wall " << w
			<< " pts=" << cutResult.plane_xyz[w].size()
			<< " center=" << docs[w]->getCenter()
			<< "\n";
	}
	std::cout << "=================================\n";
#endif

	// =======keep the biggest floor========begin
	size_t max_floor_size = 0;
	int max_floor_idx = -1;
	for (auto idx : m_idx_floor)
	{
		if (max_floor_size < cutResult.plane_xyz[idx].size())
		{
			max_floor_size = cutResult.plane_xyz[idx].size();
			max_floor_idx = idx;
		}
	}



	for (auto idx : m_idx_floor)
	{
		if (idx == max_floor_idx) continue;
		cutResult.plane_xyz[idx].clear();
		cutResult.plane_reflect[idx].clear();
	}
	// =======keep the biggest floor========end



	// get valid indices
	std::vector<unsigned int> idx_valid;
	for (size_t i = 0; i < docs.size(); ++i) {
		auto type = docs[i]->getType();
		if (type.isValid()) {
			int idx = docs[i]->getIdxPlane();
			idx_valid.push_back(static_cast<unsigned int>(idx));
		}
	}
	// find pts in roi
	std::vector<std::vector<cv::Point3f>> pts_cut(docs.size());
	std::vector<std::vector<int>>scene_plane_reflect_cut(docs.size());
	std::vector<cv::Point3f> centers(docs.size(), cv::Point3f(0.f, 0.f, 0.f));
	//std::vector<int> restore_wall_idx;

	for (size_t i = 0; i < docs.size(); ++i) {
		int idx = docs[i]->getIdxPlane();
		// if not valid, clear model points
		size_t npts = cutResult.plane_xyz[i].size();
		if (!Util_Math::isIdxInList(idx, idx_valid) || npts < 1) {
			cutResult.plane_xyz[i].clear();
			cutResult.plane_reflect[i].clear();
			continue;
		}

#if 0
	if (Util_Math::isIdxInList(i, m_idx_wall))
	{
		cv::Mat debug_img;
		cv::cvtColor(mask3, debug_img, cv::COLOR_GRAY2BGR);
		for (auto& vx : docs[i]->getPtVx2())
		{
			if (vx.x >= 0 && vx.x < mask3.cols && vx.y >= 0 && vx.y < mask3.rows)
				cv::circle(debug_img, vx, 1, cv::Scalar(0, 0, 255), -1);
		}
		std::string filename = "debug_mask_wall_" + std::to_string(i) + ".png";
		cv::imwrite(filename, debug_img);
		printf("[debug] saved wall projection image: %s\n", filename.c_str());
	}
#endif

		const std::vector<cv::Point2i> &ptVx = docs[i]->getPtVx2();

		// init pts and norms
		std::vector<cv::Point3f> pts_new;
		pts_new.reserve(npts);
		std::vector<cv::Point3f> pts_new2;
		pts_new2.reserve(npts);
		std::vector<unsigned char> scene_plane_reflect_new;
		scene_plane_reflect_new.reserve(npts);
		std::vector<unsigned char> scene_plane_reflect_new2;
		scene_plane_reflect_new2.reserve(npts);

		unsigned int count = 0;
		unsigned int count2 = 0;
		for (size_t pid = 0; pid < npts; ++pid)
		{
			const int px = ptVx[pid].x;
			const int py = ptVx[pid].y;
			if ((unsigned)px >= (unsigned)mask3.cols || (unsigned)py >= (unsigned)mask3.rows) {
				if (pid == 0) {
					std::cout << "[locROI_cull] 平面" << i << "第1个点越界：(" << px << "," << py << ")，掩码尺寸：" << mask3.cols << "x" << mask3.rows << std::endl;
				}
				continue;
			}

			const bool in3 = mask3.at<uchar>(py, px) != 0;
			const bool in9 = mask9.at<uchar>(py, px) != 0;
			const bool in12 = mask12.at<uchar>(py, px) != 0;
			const bool noCeiling = m_idx_hCeil.empty();

			if (in3 || (noCeiling && in12)) {
				pts_new.push_back(cutResult.plane_xyz[i][pid]);
				scene_plane_reflect_new.push_back(cutResult.plane_reflect[i][pid]);
				centers[i] += cutResult.plane_xyz[i][pid];
				++count;
			}
			else if (!noCeiling &&
				(Util_Math::isIdxInList(i, m_idx_roomwall) &&
					!Util_Math::isIdxInList(i, m_idx_wall)) &&
				in9)
			{
				pts_new2.push_back(cutResult.plane_xyz[i][pid]);
				scene_plane_reflect_new2.push_back(cutResult.plane_reflect[i][pid]);
				++count2;
			}
			else {
				pts_cut[i].push_back(cutResult.plane_xyz[i][pid]);
				scene_plane_reflect_cut[i].push_back(cutResult.plane_reflect[i][pid]);
			}
		}

		// resize
		pts_new.resize(count);
		scene_plane_reflect_new.resize(count);
		pts_new2.resize(count2);
		scene_plane_reflect_new2.resize(count2);

		if (count2 > pts_cut[i].size() //保证加进来的是一面大面积的墙而不是飘出去的墙的一小部分
			&& count2 > count) //防止一堵墙被切割的部分再加回来
		{
			pts_new.insert(pts_new.end(), pts_new2.begin(), pts_new2.end());
			scene_plane_reflect_new.insert(scene_plane_reflect_new.end(), scene_plane_reflect_new2.begin(), scene_plane_reflect_new2.end());

			for (size_t j = 0; j < count2; j++)
			{
				centers[i] += pts_new2[j];
			}
			count += count2;
		}
		else {
			pts_cut[i].insert(pts_cut[i].end(), pts_new2.begin(), pts_new2.end());
			scene_plane_reflect_cut[i].insert(scene_plane_reflect_cut[i].end(), scene_plane_reflect_new2.begin(), scene_plane_reflect_new2.end());
		}

		// replace model points
		if (count == 0) {
			if (docs[i]->getType().isWall())
			{
				std::cout << "[KILL][ROI_MASK] Wall " << i
					<< " removed: no point inside ROI mask\n";
			}
			cutResult.plane_xyz[i].clear();
			cutResult.plane_reflect[i].clear();
		}
		else {
			cutResult.plane_xyz[i] = pts_new;
			cutResult.plane_reflect[i] = scene_plane_reflect_new;
			centers[i] = centers[i] / static_cast<int>(count);
		}
	}

	// ========DoorWindowDetector begin==========
	float walltopZ = -10000.0f;
	for (auto wall_id : m_idx_wall)
	{
		float boxZ = docs[wall_id]->getBBox().m_max.z;
		if (walltopZ < boxZ)
			walltopZ = boxZ;
	}

	Vector<Vector<float>> window_corner_minmax_xyz;
	MeasureDoorWindow::DoorWindowDetector(cutResult.plane_xyz, scene_plane_normals, cutResult.plane_wall_idx, walltopZ, cutResult.door_window_info);
	MeasureDoorWindow::FindWindowCornerMinMax(cutResult.door_window_info, scene_plane_normals, scene_plane_centers, window_corner_minmax_xyz);
	// ========DoorWindowDetector end==========
	
	//=========== nearby wall to idx_wall ========start======
	std::vector<uint> father_wall_idx;
	std::vector<std::vector<float>> box_minmax_xyz;
	std::vector<BBox3D> nearby_walls_box;
	std::vector<int> nearby_walls_idx;
	std::vector<std::pair<int, int>> nearby_pairs;
	for (auto wall : cutResult.plane_wall_idx) //wall------child wall
	{
		if (Util_Math::isIdxInList(wall, m_idx_wall)) continue;
		for (auto wall_neighbor : m_idx_wall)  //wallneighbor---------father wall
		{
			//if (wall == wall_neighbor) continue;
			float angle = acos(docs[wall]->getNormal().dot(docs[wall_neighbor]->getNormal()))*180.f / 3.1415926f;
			float dist = Util_Math::ComputePointToPlaneDist<float, cv::Point3f>(docs[wall]->getCenter(), docs[wall_neighbor]->getNormal(), docs[wall_neighbor]->getCenter());

			BBox3D box_wall, box_neighbor;
			if (pts_cut[wall].size() != 0)
				box_wall.compute(pts_cut[wall]);
			else
				continue;
			box_neighbor.compute(cutResult.plane_xyz[wall_neighbor]);
			box_neighbor.dilate(50.f);
			bool is_smaller_than_neighbor = false;
			//cout << wall << "  " << wall_neighbor << "  " << angle << "   " << dist << endl;
			if ((docs[wall]->getNormal().x > 0.7f && box_wall.m_min.y > box_neighbor.m_min.y && box_wall.m_max.y < box_neighbor.m_max.y) ||
				(docs[wall]->getNormal().y > 0.7f && box_wall.m_min.x > box_neighbor.m_min.x && box_wall.m_max.x < box_neighbor.m_max.x))
			{
				is_smaller_than_neighbor = true;
			}

			if ((angle < 5.0f || (180.f - angle) < 5.f) && dist < 300.f && is_smaller_than_neighbor)
			{
				std::cout << "[NEARBY_PAIR] father wall " << wall_neighbor
					<< " <--- child wall " << wall
					<< " angle=" << angle
					<< " dist=" << dist
					<< "\n";

				cutResult.plane_xyz[wall].insert(cutResult.plane_xyz[wall].end(), pts_cut[wall].begin(), pts_cut[wall].end());
				cutResult.plane_reflect[wall].insert(cutResult.plane_reflect[wall].end(), scene_plane_reflect_cut[wall].begin(), scene_plane_reflect_cut[wall].end());

				nearby_walls_box.push_back(docs[wall]->getBBox());
				//nearby_walls_idx.push_back(wall);
				//m_idx_wall.push_back(wall);
				nearby_pairs.push_back(std::make_pair(wall_neighbor, wall)); // father wall <-----> child wall
				father_wall_idx.push_back(wall_neighbor);
			}
		}
	}

	for (auto wall_pair : nearby_pairs)
	{
		auto sonId = wall_pair.second;
		auto fatherId = wall_pair.first;
		auto sonCenter = docs[sonId]->getCenter();
		auto sonBox = docs[sonId]->getBBox();
		float sonArea = MAX(sonBox.m_max.x - sonBox.m_min.x, sonBox.m_max.y - sonBox.m_min.y) * (sonBox.m_max.z - sonBox.m_min.z);

		auto doorInfos = cutResult.door_window_info[fatherId];
		std::vector<int> doorIdx_need_to_delete;
		for (size_t i = 0; i < doorInfos.size(); i++)
		{
			Vector<Vector<float>> d_corner;
			MeasureDoorWindow::FindWindowCornerMinMax(doorInfos[i], scene_plane_normals, scene_plane_centers, d_corner);
			if (IsPointInBox(d_corner, sonCenter))
			{
				float doorArea = MAX(d_corner[0][1] - d_corner[0][0], d_corner[0][3] - d_corner[0][2]) * (d_corner[0][5] - d_corner[0][4]);
				if (sonArea > doorArea * 0.85f)
				{
					doorIdx_need_to_delete.push_back(i);
					nearby_walls_idx.push_back(sonId);
				}
				else
				{
					std::cout <<
						"[KILL][NEARBY_DOOR] Wall "
						<< sonId
						<<
						" removed as child wall of "
						<< fatherId
						<<
						" (sonArea < doorArea)\n"
						;
					cutResult.plane_xyz[sonId].clear();
					cutResult.plane_reflect[sonId].clear();
				}
			}
		}
		std::vector<MeasureDoorWindow::DoorWindowInfo> newInfo;
		for (size_t i = 0; i < doorInfos.size(); i++)
		{
			if (!isIdxInList(i, doorIdx_need_to_delete))
			{
				for (size_t i = 0; i < doorInfos.size(); i++)
					newInfo.push_back(doorInfos[i]);
			}
		}
		cutResult.door_window_info[fatherId] = newInfo;
	}

	std::vector<Point3f> normals = scene_plane_normals;
	std::vector<Point3f> plane_centers = scene_plane_centers;

	//std::vector<std::pair<int, std::vector<float>>> wall_list;
	//computeWallList(doc_bim, cutResult.plane_xyz, scene_plane_normals, wall_list);
	//MeasureDoorWindow::MergePlanes(cutResult.plane_xyz, plane_centers, normals, cutResult.plane_reflect, wall_list);

	std::vector<uint> plane_to_skip;
	//=========== nearby wall to idx_wall ========end======	
	if (!nearby_walls_idx.empty())
	{
		MeasureDoorWindow::DoorWindowDetector(cutResult.plane_xyz, scene_plane_normals, nearby_walls_idx, walltopZ, cutResult.door_window_info);
		window_corner_minmax_xyz.resize(0);
		MeasureDoorWindow::FindWindowCornerMinMax(cutResult.door_window_info, scene_plane_normals, scene_plane_centers, window_corner_minmax_xyz);

		FindBoxMinMax(nearby_walls_box, nearby_walls_idx, scene_plane_normals, scene_plane_centers,
			box_minmax_xyz, 300.f, 50.f, 0.f);

		for (int j = 0; j < pts_cut.size(); j++)
		{
			if (pts_cut[j].size() == 0 || Util_Math::isIdxInList(j, m_idx_floor))	continue;
			for (auto k = 0; k < pts_cut[j].size(); k++)
			{
				if (IsPointInBox(box_minmax_xyz, pts_cut[j][k]))
				{
					cutResult.plane_xyz[j].push_back(pts_cut[j][k]);
					cutResult.plane_reflect[j].push_back(scene_plane_reflect_cut[j][k]);
				}
			}
		}
		plane_to_skip = father_wall_idx;

		for (size_t i = 0; i < cutResult.plane_xyz.size(); i++)
		{
			if (cutResult.plane_xyz[i].size() == 0)	continue;

			if (IsPointInBox(box_minmax_xyz, docs[i]->getCenter()))
			{
				plane_to_skip.push_back(i);
				nearby_walls_idx.push_back(i);
			}

			if (IsPointInBox(window_corner_minmax_xyz, docs[i]->getCenter())
				&& cutResult.door_window_info[i].size() == 0
				&& i != max_floor_idx
				&& !isIdxInList(i, father_wall_idx))
			{
				if (docs[i]->getType().isWall())
				{
					std::cout << "[KILL][WINDOW_BOX] Wall " << i
						<< " removed: inside window box but no door/window\n";
				}

				cutResult.plane_xyz[i].clear();
				cutResult.plane_reflect[i].clear();
			}
		}
	}

	std::vector<std::pair<int, std::vector<float>>> wall_list;
	computeWallList(doc_bim, cutResult.plane_xyz, scene_plane_normals, wall_list);
	/*for (int i = 0; i < cutResult.plane_xyz.size(); i++)
	{
		if (cutResult.plane_xyz[i].size() == 0)
			continue;

		std::stringstream filename;
		filename << "output_mesh\\" << i << ".obj";
		concreteMesherDelauney mesher;
		mesher.MeshPlane(cutResult.plane_xyz[i], normals[i], filename.str());
	}*/
	MeasureDoorWindow::MergePlanes(cutResult.plane_xyz, cutResult.door_window_info, plane_centers, normals, cutResult.plane_reflect, wall_list);
	MeasureDoorWindow::DoorWindowDetector(cutResult.plane_xyz, scene_plane_normals, cutResult.plane_wall_idx, walltopZ, cutResult.door_window_info);
	for(int i=0; i < docs.size(); i++)
		docs[i]->updateBBox(cutResult.plane_xyz[i]);
	
	BIM_Filter_Plane::filter_wall(doc_bim);

	std::cout << "===== Before removeTinyPlane =====\n";
	for (auto w : m_idx_wall)
		std::cout << "Wall " << w << " pts=" << cutResult.plane_xyz[w].size() << "\n";
	removeTinyPlane(doc_bim, cutResult.plane_xyz, scene_plane_normals, cutResult.plane_reflect, plane_to_skip);
	std::cout << "===== After removeTinyPlane =====\n";
	for (auto w : m_idx_wall)
	{
		if (cutResult.plane_xyz[w].empty())
			std::cout << "[KILL][TINY] Wall " << w << "\n";
	}

	//
	//ofstream file("E://test//pts_cut_windows.txt");
	/*for (int j = 0; j < pts_cut.size(); j++)
	{
		if (pts_cut[j].size() == 0)	continue; 

		for (auto k = 0; k < pts_cut[j].size(); k++)
		{
			if (IsPointInBox(window_corner_minmax_xyz, pts_cut[j][k]))
			{
				//file << pts_cut[j][k].x << " " << pts_cut[j][k].y << " " << pts_cut[j][k].z << endl;
				cutResult.plane_xyz[j].push_back(pts_cut[j][k]);
				cutResult.plane_reflect[j].push_back(scene_plane_reflect_cut[j][k]);
			}
		}
		//IOData::SavePoint3fData("E://test//pts_cut" + to_string(j)+".txt", pts_cut[j]);
		
	}*/
	//file.close();
	// 
	if (!isCompleteRoom(doc_bim, cutResult.plane_xyz, cutResult.plane_wall_idx, nearby_walls_idx))
	{
		m_isValid = false;
		return;
	}


	m_idx_wall.clear();
	m_idx_hCeil.clear();
	m_idx_floor.clear();
	m_idx_beam.clear();
	for (int i = 0; i < cutResult.plane_xyz.size(); i++)
	{
		if (cutResult.plane_xyz[i].size() == 0) continue;
		if (docs[i]->getType().isWall()) m_idx_wall.push_back(i);
		else if (docs[i]->getType().isCeiling()) m_idx_hCeil.push_back(i);
		else if (docs[i]->getType().isFloor())
		{
			m_idx_floor.push_back(i);
		}
		else if (docs[i]->getType().isBeam()) m_idx_beam.push_back(i);
	}


	std::vector<unsigned int> wall_need_keep;

#if 0
	std::cout << "===== Before RemoveIntersectWall =====\n";
	for (auto w : m_idx_wall)
		std::cout << "Wall " << w << " pts=" << cutResult.plane_xyz[w].size() << "\n";
#endif

	RemoveIntersectWall(doc_bim, cutResult.plane_xyz, cutResult.plane_reflect, scene_plane_normals, m_idx_wall, wall_need_keep, m_bbox_g);

#if 0
	std::cout << "===== After RemoveIntersectWall =====\n";
	for (auto w : m_idx_wall)
	{
		if (cutResult.plane_xyz[w].empty())
			std::cout << "[KILL][INTERSECT] Wall " << w << "\n";
	}

	std::cout << "\n===== [locROI_cull] 最终墙存活情况 =====\n";
	for (int i = 0; i < docs.size(); ++i)
	{
		if (!docs[i]->getType().isWall()) continue;

		if (cutResult.plane_xyz[i].empty())
			std::cout << "Wall " << i << " => DEAD\n";
		else
			std::cout << "Wall " << i << " => ALIVE (pts="
			<< cutResult.plane_xyz[i].size() << ")\n";
	}
	std::cout << "====================================\n";
#endif

	docs.clear();

}


void BIM_LOC_ROI::computeWallList(Doc_BIM &doc_bim,
	std::vector<std::vector<cv::Point3f>> &pts,
	const std::vector<cv::Point3f> &normals,
	std::vector<std::pair<int, std::vector<float>>> &angle_wall_list)
{
	std::vector<Doc_BIM_Plane *> docs; // all docs
	doc_bim.getDocPlane(docs);

	// compute projection parameters
	cv::Point3f bbox_diag = m_bbox_g->m_max - m_bbox_g->m_min;
	float vl = MAX(bbox_diag.x, bbox_diag.y) / 512.f;  //why devided by 512?
	unsigned int extent_x = bbox_diag.x / vl + 10;
	unsigned int extent_y = bbox_diag.y / vl + 10;


	
	std::vector<int> idx_wall;
	//for wall
	for (int i = 0; i < normals.size(); i++)
	{
		if (std::abs(std::abs(normals[i].z) - 1) < 0.3 || pts.size() == 0)
			continue;  //skip horizonal plane
		idx_wall.push_back(i);
	}
	size_t n_model = pts.size();
	std::vector<std::vector<cv::Point2i>> line_endpoints(n_model, std::vector<cv::Point2i>(3, cv::Point2i(0, 0)));
	cv::Point2i ori = cv::Point2i(int(floor((-m_bbox_g->m_min.x + 1) / vl)), int(floor((-m_bbox_g->m_min.y + 1) / vl)));

	auto calRotateAngle = [](cv::Point2i v)->double {
		double theta = atan2(v.y, v.x) / 3.1415926 * 180;
		if (theta < 0)
			theta += 360;
		return theta;
	};

	for (auto idx : idx_wall)
	{
		if (pts[idx].size() == 0)
			continue;
		cv::Mat img_temp;
		docs[idx]->getImgProjGlobal(img_temp);
		cvtColor(img_temp, img_temp, CV_RGB2GRAY);
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Point> contours_merge;
		std::vector<cv::Vec4i> hierarchy;
		findContours(img_temp, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point());

		for (auto contour : contours)
		{
			contours_merge.insert(contours_merge.end(), contour.begin(), contour.end());
		}

		cv::RotatedRect rrt = cv::minAreaRect(contours_merge);
		//changed by yu.liang 2021/1/6 end
		cv::Point2i cpt = rrt.center;
		cv::Point2f ppts[4];
		rrt.points(ppts);

		cv::Point2i endpoint_a, endpoint_b;

		std::vector<cv::Point>  contour_lines;

		if (rrt.size.width > rrt.size.height) {
			contour_lines = { (ppts[0] + ppts[1])/2.0f ,(ppts[2] + ppts[3]) / 2.0f };
		}
		else {
			contour_lines = { (ppts[0] + ppts[3]) / 2.0f ,(ppts[1] + ppts[2]) / 2.0f };
		}

		float angleA = calRotateAngle(contour_lines[0] - ori);
		float angleB = calRotateAngle(contour_lines[1] - ori);
		float angleMiddle = calRotateAngle((contour_lines[1] + contour_lines[0] )/2.0f- ori);
		std::pair<int, std::vector<float>> wall_angle_info;
		wall_angle_info.first = idx;
		if ((angleA > 270 && angleB < 90) || (angleB > 270 && angleA < 90))
		{
			if((angleA > 270 && angleB < 90))
				wall_angle_info.second = { angleA ,angleB,angleMiddle };
			else
				wall_angle_info.second = { angleB ,angleA,angleMiddle };
		}
		else if (angleA < angleB) {
			wall_angle_info.second = { angleA ,angleB,angleMiddle };
		}
		else {
			wall_angle_info.second = { angleB ,angleA,angleMiddle };
		}

		angle_wall_list.push_back(wall_angle_info);
	}

	std::sort(angle_wall_list.begin(), angle_wall_list.end(), [](auto a, auto b)->bool {return a.second[0] < b.second[0];});

	/*for (auto wall_angle : angle_wall_list)
	{
		std::cout << "wall_angle id = " << wall_angle.first << ",";
	}
	std::cout << std::endl;*/
}

// added by yu.liang@unre.com 2021/1/4 start
void BIM_LOC_ROI::updatePlaneType(Doc_BIM &doc_bim, PlaneCutResultInterface& cutResult)
{

	std::cout << "赋值前m_idx_floor：大小=" << m_idx_floor.size() << "，索引列表=";
	for (auto idx : m_idx_floor) std::cout << idx << " ";
	std::cout << std::endl;
	std::cout << "[updatePlaneType] 赋值前cutResult.plane_ground_idx：大小=" << cutResult.plane_ground_idx.size() << std::endl;

	//idx_valid.resize(m_idx_valid.size());
	cutResult.plane_wall_idx.resize(m_idx_wall.size());
	cutResult.plane_ground_idx.resize(m_idx_floor.size());
	cutResult.plane_ceiling_idx.resize(m_idx_hCeil.size());
	cutResult.plane_beam_idx.resize(m_idx_beam.size());
	//for (size_t i = 0; i < m_idx_valid.size(); i++)  cutResult.idx_valid[i] = m_idx_valid[i];
	for (size_t i = 0; i < m_idx_wall.size(); i++)   cutResult.plane_wall_idx[i] = m_idx_wall[i];
	for (size_t i = 0; i < m_idx_floor.size(); i++)  cutResult.plane_ground_idx[i] = m_idx_floor[i];
	for (size_t i = 0; i < m_idx_hCeil.size(); i++)  cutResult.plane_ceiling_idx[i] = m_idx_hCeil[i];
	for (size_t i = 0; i < m_idx_beam.size(); i++)   cutResult.plane_beam_idx[i] = m_idx_beam[i];
	//update  L_shape_plane_idx and parallel_plane_idx
	//added by yu.liang 2020/12/23 start
	cutResult.parallel_plane_idx.clear();
	cutResult.L_shape_plane_idx.clear();
	std::vector<Doc_BIM_Plane *> docs; // all docs
	doc_bim.getDocPlane(docs);

	if (docs.empty()) m_isValid = false; // check if docs init
	if (!m_isValid) {
		docs.clear();
		return;
	}
	for (int i = 0; i < m_idx_wall.size(); i++)
	{
		cv::Point3f normal_i = docs[m_idx_wall[i]]->getNormal();
		normal_i.z = 0.f;  //Currently we don't consider the z-direction component
		normal_i = Util_Math::vec3_normalize(normal_i);
		for (int j = i + 1; j < m_idx_wall.size(); j++)
		{
			cv::Point3f normal_j = docs[m_idx_wall[j]]->getNormal();
			normal_j.z = 0.f;//Currently we don't consider the z-direction component
			normal_j = Util_Math::vec3_normalize(normal_j);
			//cout << m_idx_wall[i] << "---" << m_idx_wall[j] << "===="<<abs(normal_i.dot(normal_j)) << endl;
			if (abs(normal_i.dot(normal_j)) > cos(5 / 180.f*3.1415926)) //-5~5 degree; cos(5 / 180.f*3.1415926)=0.996
			{
				cutResult.parallel_plane_idx.push_back(std::make_pair(m_idx_wall[i], m_idx_wall[j]));
			}
			else if (abs(normal_i.dot(normal_j)) < cos(85 / 180.f*3.1415926) && //85~95degree; cos(85 / 180.f*3.1415926)=0.088
				BIM_Intxn::isIntxn(docs[m_idx_wall[i]], docs[m_idx_wall[j]], 50))
			{
				cutResult.L_shape_plane_idx.push_back(std::make_pair(m_idx_wall[i], m_idx_wall[j]));
			}
		}
	}

	std::cout << "赋值后cutResult.plane_ground_idx：大小=" << cutResult.plane_ground_idx.size() << "，索引列表=";
	for (auto idx : cutResult.plane_ground_idx) std::cout << idx << " ";
	std::cout << std::endl;

	docs.clear();
}
// added by yu.liang@unre.com 2021/1/4 end

/**
* \brief filter beam
* yu.liang@unre.com
* find plane intxn with union of ceiling and vertical planes
* candidate planes: vertical, not wall
*/
void BIM_LOC_ROI::removeTinyPlane(Doc_BIM &doc_bim,
	std::vector<std::vector<cv::Point3f>> &pts,
	const std::vector<cv::Point3f> &normals,
	std::vector<std::vector<unsigned char>> & scene_plane_reflect,
	std::vector<uint>& wall_idx_need_to_keep) {
	// data validation
	std::vector<Doc_BIM_Plane *> docs;
	doc_bim.getDocPlane(docs);
	if (docs.empty()) return;

	//std::vector<unsigned int> idx_proc;
	//doc_bim.getIdxProc(idx_proc);
	// init states of all planes
	unsigned int min_pts_filter = 800;
	for (int i = 0; i < pts.size(); i++)
	{
		if (pts[i].size() == 0 || Util_Math::isIdxInList(i, wall_idx_need_to_keep))
			continue;
		
		if (pts[i].size() < min_pts_filter)
		{			
			pts[i].clear();
			scene_plane_reflect[i].clear();
		}
		else
		{
			double plane_area;
			GeometryFilter geometry_filter;
			VoxelParams area_voxel;
			float scale = 1.0f;
			area_voxel.length_x_of_voxel = 50.f * scale;
			area_voxel.length_y_of_voxel = 50.f * scale;
			area_voxel.length_z_of_voxel = 50.f * scale;
			PointArray pts_array;
			pts_array.points = new Point3f[pts[i].size()];
			for (unsigned int j = 0; j < pts[i].size(); j++)
			{
				pts_array.points[j] = pts[i][j];
			}
			pts_array.size = pts[i].size();
			geometry_filter.SetInputNormal(normals[i]);
			geometry_filter.GetPlaneArea2D(pts_array, area_voxel, plane_area);
			delete[] pts_array.points;			
			if (plane_area < 300.0 * 300.0) //13/3/2024 Bony: change from 600*600 to 300*300
			{
				pts[i].clear();
				scene_plane_reflect[i].clear();
			}
			else
			{
				docs[i]->updateBBox(pts[i]);
				BBox3D box = docs[i]->getBBox();
				if ((box.m_max.x - box.m_min.x < 30.f && box.m_max.y - box.m_min.y < 100.f) ||
					(box.m_max.x - box.m_min.x < 100.f && box.m_max.y - box.m_min.y < 30.f) ||
					(box.m_max.x - box.m_min.x > 60.f && box.m_max.y - box.m_min.y > 60.f && sqrt(pow(box.m_max.x - box.m_min.x, 2) + pow(box.m_max.y - box.m_min.y, 2)) < 170.f) ||
					box.m_max.z - box.m_min.z < 10.f && (box.m_max.y - box.m_min.y < 30.f || box.m_max.x - box.m_min.x < 30.f))
				{
					pts[i].clear();
					scene_plane_reflect[i].clear();
				}
			}
		}
	}

	for (size_t i = 0; i < pts.size(); ++i) {
		if (pts[i].size() == 0)
		{
			State_BIM_Plane states; // state of all planes
			states = docs[i]->getType();
			states.clearState();
			docs[i]->setType(states);
		}
	}
	docs.clear();
}

void BIM_LOC_ROI::printPlaneType(Doc_BIM &doc_bim)
{
	std::vector<Doc_BIM_Plane *> docs; // all docs
	doc_bim.getDocPlane(docs);
	if (docs.empty()) return;
	for (size_t i = 0; i < docs.size(); i++)
	{
		int type = docs[i]->getType().getState();
		cout << i << " --- ";
		if ((type >> 1) & 1) cout << "HOR ";
		if ((type >> 2) & 1) cout << "VER ";
		if ((type >> 3) & 1) cout << "FLR ";
		if ((type >> 4) & 1) cout << "CEI ";
		if ((type >> 5) & 1) cout << "WAL ";
		if ((type >> 6) & 1) cout << "GIR ";
		if ((type >> 0) & 1) cout << "VAL";
		else cout << "X";
		cout << endl;
	}
}

bool BIM_LOC_ROI::isCompleteRoom(Doc_BIM &doc_bim, std::vector<std::vector<cv::Point3f>> &pts, 
	std::vector<int>& wall_idx, std::vector<int>& wall_idx_to_skip)
{
	std::vector<Doc_BIM_Plane *> docs;
	doc_bim.getDocPlane(docs);
	if (docs.empty()) return false;

	bool has_wall_S = false, has_wall_N = false, has_wall_E = false, has_wall_W = false;
	std::vector<int> consideration_wall_idx;
	float total_angle = 0.0f;
	for (auto idx : wall_idx)
	{
		if (pts[idx].size() < 1000 || isIdxInList(idx, wall_idx_to_skip)) continue;
		
		auto normal = docs[idx]->getNormal();
		auto center = docs[idx]->getCenter();
		BBox3D wall_box = docs[idx]->getBBox();
		cv::Point3f A, B;
		float angle = 0.f;

		if (abs(normal.x) > 0.99f)  // 82 degree
		{
			if (center.x > 0)
			{
				has_wall_E = true;				
			}
			else
			{
				has_wall_W = true;				
			}
			A = cv::Point3f(center.x, wall_box.m_max.y, 0.f);
			B = cv::Point3f(center.x, wall_box.m_min.y, 0.f);
			angle = Util_Math::vec3_angle_deg<cv::Point3f>(A, B);
		}
		else if (abs(normal.y) > 0.99f)  // 82 degree
		{
			if (center.y > 0)
			{
				has_wall_N = true;
			}
			else
			{
				has_wall_S = true;
			}
			A = cv::Point3f(wall_box.m_max.x, center.y, 0.f);
			B = cv::Point3f(wall_box.m_min.x, center.y, 0.f);
			angle = Util_Math::vec3_angle_deg<cv::Point3f>(A, B);
			
		}
		//cout <<A<<"  "<<B<<"  "<< angle << endl;
		if (angle > 0.f)
			total_angle += angle;		
	}
	if (!has_wall_S) cout << "don't have south wall" << endl;
	if (!has_wall_N) cout << "don't have north wall" << endl;
	if (!has_wall_E) cout << "don't have east wall" << endl;
	if (!has_wall_W) cout << "don't have west wall" << endl;
	if (total_angle / 360.f < 0.5f) cout << "total_angle is " << total_angle << endl;
#if 0
	return has_wall_S && has_wall_N && has_wall_E && has_wall_W && (total_angle / 360.f > 0.5f);
#else
	std::cout << "ignore the Complete Room assumption if there is any, relax for robustness..." << std::endl;
	return true;
#endif
}

void BIM_LOC_ROI::RemoveNoiseWall(
	Doc_BIM& doc_bim,
	std::vector<std::vector<cv::Point3f>>& pts)
{
	std::vector<Doc_BIM_Plane*> docs;
	doc_bim.getDocPlane(docs);

	const int MIN_WALL_POINTS = 3000;
	const float PARALLEL_ANGLE = 10.f;
	const float ORTHO_ANGLE = 10.f;

	int n = docs.size();

	// 1. 收集有效墙索引
	std::vector<int> wall_ids;
	for (int i = 0; i < n; ++i)
	{
		if (pts[i].empty()) continue;
		if (std::abs(docs[i]->getNormal().z) < 0.2f)
			wall_ids.push_back(i);
	}

	if (wall_ids.size() < 3)
		return; // 墙太少，绝不删除

	// 2. 统计主方向（dominant normals）
	std::vector<cv::Point3f> dominant_normals;

	for (int id : wall_ids)
	{
		cv::Point3f n0 = docs[id]->getNormal();
		bool merged = false;

		for (auto& dn : dominant_normals)
		{
			float angle = acos(n0.dot(dn)) * 180.f / CV_PI;
			if (angle < PARALLEL_ANGLE || angle > 180 - PARALLEL_ANGLE)
			{
				merged = true;
				break;
			}
		}

		if (!merged)
			dominant_normals.push_back(n0);
	}

	std::vector<bool> need_delete(n, false);

	// 3. 判定每一堵墙
	for (int id : wall_ids)
	{
		// 3.1 点数足够，直接保留
		if (pts[id].size() > MIN_WALL_POINTS)
			continue;

		cv::Point3f n0 = docs[id]->getNormal();

		// 3.2 主方向保护
		bool is_dominant = false;
		for (auto& dn : dominant_normals)
		{
			float angle = acos(n0.dot(dn)) * 180.f / CV_PI;
			if (angle < PARALLEL_ANGLE || angle > 180 - PARALLEL_ANGLE)
			{
				is_dominant = true;
				break;
			}
		}
		if (is_dominant)
			continue;

		// 3.3 统计结构关系
		int parallel_cnt = 0;
		int ortho_cnt = 0;

		for (int other : wall_ids)
		{
			if (other == id) continue;

			float angle = acos(n0.dot(docs[other]->getNormal())) * 180.f / CV_PI;

			if (angle < PARALLEL_ANGLE || angle > 180 - PARALLEL_ANGLE)
				parallel_cnt++;
			else if (std::abs(angle - 90.f) < ORTHO_ANGLE)
				ortho_cnt++;
		}

		// 3.4 孤立噪声判定
		if (parallel_cnt < 2 && ortho_cnt < 1)
		{
			need_delete[id] = true;
		}
	}

	// 4. 执行删除
	for (int i = 0; i < n; ++i)
	{
		if (!need_delete[i]) continue;

		pts[i].clear();

		State_BIM_Plane type = docs[i]->getType();
		type.clearState();
		docs[i]->setType(type);

		std::cout << "[RemoveNoiseWall_Safe] delete wall id = " << i << std::endl;
	}
}

