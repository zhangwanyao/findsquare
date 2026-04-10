#include "DetectionHoleClass.h"
#include <cmath>
#include <omp.h>
#include <string>
#include <algorithm>
#include "util_math.hpp"
#include "util_sampler_1d.h"
#include "MathOperation.hpp"
#include "in_out_data.hpp"
#include "../../PointReconstruction/Measurement/MeasureBase.h"


extern struct rmInfo rmdata;
extern std::ofstream resultA;
extern std::string filePath;
using namespace std;

//#define DBG_HOLES
//#define DBG_HOLES_SV

/**
* \brief check if two float values equal or not
* \author Bichen JING
* \tparam VAL type of value
* \tparam CMP type of compared value
* \param val input value
* \param cmp input compared value
* \return true if values are equal, otherwise, false
*/
template<typename VAL, typename CMP>
inline bool IsValEqual(const VAL &val, const CMP &cmp) {
	return std::fabs(val - static_cast<VAL>(cmp)) <= std::numeric_limits<VAL>::epsilon();
}

/**
* \brief check if a float values equals to zero or not
* \author Bichen JING
* \tparam T type of value
* \param val input value
* \return true if value equals to zero, otherwise, false
*/
template<typename T>
inline bool IsValZero(const T &val) {
	return IsValEqual(val, 0.0);
}

/**
* \brief check if value in bounds
* \author Bichen JING
* \tparam T type of value
* \param val input value
* \param lower lower bound
* \param upper upper bound
* \return true if value in bounds, otherwise, false
*/
template<typename T>
inline bool IsInBounds(const T &val, const T &lower, const T &upper) {
	// data validation
	return !(val < lower) && !(val > upper);
}

/**
* \brief check if value in bounds
* \author Bichen JING
* \tparam T type of value
* \param val input value
* \param bounds bounding values in pair
* \return true if value in bounds, otherwise, false
*/
template<typename T>
inline bool IsInBounds(const T &val, const std::pair<T, T> &bounds)
{
	// data validation
	return !(val < bounds.first) && !(val > bounds.second);
}

/**
* \brief check if 2d point in bounds
* \author Bichen JING
* \tparam T type of value
* \param x x-coord of point
* \param y y-coord of point
* \param xmin minmum x-coord bound
* \param xmax maximum x-coord bound
* \param ymin minmum y-coord bound
* \param ymax maximum y-coord bound
* \return true if value in bounds, otherwise, false
*/
template<typename T>
inline bool IsPtInBounds2D(const T &x, const T &y,
	const T &xmin, const T &xmax, const T &ymin, const T &ymax)
{
	return IsInBounds(x, xmin, xmax) && IsInBounds(y, ymin, ymax);
}

/**
* \brief clear std vector
*/
template<class T>
inline void ClearStdVector(std::vector<T> &data) {
	data.clear();
}
/**
* \brief clear std vector
*/
template<class T>
inline void ClearStdVector(std::vector<std::vector<T>> &data) {
	if (data.empty()) return;
	size_t n_data = data.size();
	for (size_t i = 0; i < n_data; i++) {
		data[i].clear();
	}
	data.clear();
}
/**
* \brief clear std vector
*/
template<class T>
inline void ClearStdVector(std::vector<std::vector<std::vector<T>>> &data) {
	if (data.empty()) return;
	size_t n_data = data.size();
	for (size_t i = 0; i < n_data; i++) {
		if (data[i].empty()) continue;
		size_t n_data_i = data[i].size();
		for (size_t j = 0; j < n_data_i; j++) {
			data[i][j].clear();
		}
		data[i].clear();
	}
	data.clear();
}
/**
* \brief get minimum and maximum value from a vector
*/
template<class T>
inline void getMinMaxfromVector(const std::vector<T> &data, T &val_min, T &val_max)
{
	if (data.empty()) return;
	size_t n_data = data.size();
	val_min = data[0];
	val_max = data[0];
	for (size_t i = 1; i < n_data; i++) {
		if (data[i] < val_min) val_min = data[i];
		if (data[i] > val_max) val_max = data[i];
	}
}

DetectionHoleClass::DetectionHoleClass()
	: err_msg("error free")
	, m_width_ruler(40.f)
	, m_thres_minLen_hole(500.f)
	, m_thres_maxLen_hole(4000.f)
	, m_thres_minWidth_door(500.f)
	, m_thres_maxWidth_door(4000.f)
	, m_thres_minHeight_door(1800.f)
	, m_thres_maxHeight_door(2400.f)
	, m_thres_dis_isPtOverlap(50.f)
	, m_sample_cstr_len(400.f)
	, m_sample_margin(200.f)
	, m_decn_tree(nullptr)
{
	Init();
}

DetectionHoleClass::~DetectionHoleClass()
{
	Clear();
}

void DetectionHoleClass::Init()
{
	m_halfWidth_ruler = m_width_ruler * 0.5f;
	return;
}

/**
* \brief init decision tree
*/
void DetectionHoleClass::Init_DecnTree()
{
	// clear previous decision tree
	if (m_decn_tree != nullptr) {
		Decn_Tree *temp = m_decn_tree;
		m_decn_tree = nullptr;
		delete temp;
	}
	// init decision tree
	m_decn_tree = new Decn_Tree();
	std::vector<Decn_Node *> nodes;
	nodes.resize(3);
	for (size_t i = 0; i < nodes.size(); i++) {
		nodes[i] = new Decn_Node(i);
	}
	nodes[0]->setLeft(nodes[1]);
	nodes[0]->setRight(nodes[2]);
	nodes[0]->init(0, Decn_Cond::TYPE_UNARY::U_N_LESS);
	nodes[0]->appendCSTR(m_sample_cstr_len);
	nodes[1]->setIdxResult(0); // >= 400
	nodes[2]->setIdxResult(1); // < 400
	m_decn_tree->setRoot(nodes[0]);
	nodes.clear();
}

void DetectionHoleClass::Clear()
{
	if (m_decn_tree != nullptr) {
		delete m_decn_tree;
	}
}

void SaveSceneData(std::string path, int scene_idx, std::vector<cv::Point3f>& filter_scene_plane_xyz)
{
	std::stringstream file_num;
	file_num << path.c_str() << scene_idx << "_xyz.txt";
	std::string file_name = file_num.str();
	IOData::SavePoint3fData(file_name, filter_scene_plane_xyz);
}
bool DetectionHoleClass::DetectionHoleClassFcn(
	int pidx,
	const std::vector<cv::Point3f> &pts_plane,
	const float normal_plane[3],
	const std::vector<cv::Point3f> &corners_plane,
	const float walltopZ,
	std::vector<std::vector<cv::Point3f>> &corners_hole,
	std::vector<std::array<float, 2>> &raw_w_h,
	std::vector<int> &type_hole,
	const bool isRemeasure,
	std::vector<array<float, 4>> &output_w_h) {
	// data validation
	if (pts_plane.empty()) {
		PrintMsg("err0-1: empty input", "DetcHole::Detect(): empty input");
		return false;
	}
	// check plane normal
	if (std::abs(normal_plane[2]) > 0.1) {
		corners_hole.resize(0);
		raw_w_h.resize(0);
		type_hole.resize(0);
		return false;
	}
	// init decision and sampler
	Init_DecnTree();
#if 0
	// step 1: find holes
	DetectHole( pts_plane, ///T
				cv::Point3f(normal_plane[0], normal_plane[1], normal_plane[2]),
				corners_plane, 
				corners_hole,
				raw_w_h, 
				isRemeasure, 
				output_w_h);
	for (int i = 0; i < corners_hole.size(); i++) 
	{
		//cout << "DetectHole i:" << i << endl;
		for (int j = 0; j < corners_hole[i].size(); j++) 
		{
			cout << corners_hole[i][j] << endl;;
		}
	}
#else
	corners_hole.clear();
	raw_w_h.clear();
	output_w_h.clear();
#ifdef DBG_HOLES
	resultA <<"======= pidx: " <<pidx<< endl;
#endif
	//cout << "===================== DetectHole3 In. " << __FUNCTION__ << "  " << __LINE__ << endl;
	DetectHole3(pts_plane, ///T
		cv::Point3f(normal_plane[0], normal_plane[1], normal_plane[2]),
		corners_plane,
		corners_hole,
		raw_w_h, 
		pidx,
		output_w_h);
	//cout << "=================== DetectHole3 Out. " << __FUNCTION__ << "  " << __LINE__ << endl;
#ifdef DBG_HOLES
	for (int i = 0; i < corners_hole.size(); i++)
	{
		cout << "DetectHole3 i:" << i << endl;
		for (int j = 0; j < corners_hole[i].size(); j++)
		{
			cout << corners_hole[i][j] << endl;
		}
	}
#endif
#endif
#if 0
	std::vector<array<float, 4>> tempCornors;
#if 1
	std::vector<cv::Point3f> temp0 = pts_plane;
	SaveSceneData("dll_log\\", 66*10000+i, temp0);//================
#endif

	std::vector<cv::Point3f> temp = DetectHole2(pts_plane, ///T
									cv::Point3f(normal_plane[0], normal_plane[1], normal_plane[2]),
									corners_plane,
									corners_hole,
									raw_w_h, 
									isRemeasure,
									tempCornors);
#if 1
	SaveSceneData("dll_log\\", 99*10000+i, temp);//================
#endif
#endif
	// step 2: classify holes
	// detect upper and lower border of plane
	float z_min = std::numeric_limits<float>::infinity();
	float z_max = walltopZ;
	//float z_max = -std::numeric_limits<float>::infinity();
	for (size_t i = 0; i < pts_plane.size(); ++i) {
		if (pts_plane[i].z < z_min) z_min = pts_plane[i].z;
		//if (pts_plane[i].z > z_max) z_max = pts_plane[i].z;
	}

	if (raw_w_h.size() > 0) {
		//type_hole.resize(raw_w_h.size());
		//for (unsigned int i = 0; i < raw_w_h.size(); i++) {
		//	// type_hole[i] = ClassifyHole(raw_w_h[i][0], raw_w_h[i][1], corners_hole[i], corners_plane);
		//	type_hole[i] = ClassifyHole(raw_w_h[i][0], raw_w_h[i][1], corners_hole[i], z_max, z_min);

		//}

		std::vector<std::array<float, 2>> temp_raw_w_h;
		std::vector<std::vector<cv::Point3f>> tmp_corners_hole;
		std::vector<int> temp_type_hole;
		std::vector<array<float, 4>> temp_output_w_h;

		for (unsigned int i = 0; i < raw_w_h.size(); i++) {
			//cout << z_max << endl;

			//	type_hole[i] = ClassifyHole(raw_w_h[i][0], raw_w_h[i][1], corners_hole[i], z_max, z_min);
			int typeholes = ClassifyHole(raw_w_h[i][0], raw_w_h[i][1], corners_hole[i], z_max, z_min);
			//cout << typeholes << endl;
			//log_info("typeholes:%d  zmin: %f",typeholes, z_min );
			if (typeholes == 0 || typeholes == 1)
			{
				temp_raw_w_h.push_back(raw_w_h[i]);
				tmp_corners_hole.push_back(corners_hole[i]);
				temp_type_hole.push_back(typeholes);
				temp_output_w_h.push_back(output_w_h[i]);
			}
		}
		raw_w_h.clear();
		corners_hole.clear();
		type_hole.clear();
		output_w_h.clear();
		for (unsigned int i = 0; i < temp_raw_w_h.size(); i++) {
			raw_w_h.push_back(temp_raw_w_h[i]);
			corners_hole.push_back(tmp_corners_hole[i]);
			type_hole.push_back(temp_type_hole[i]);
			output_w_h.push_back(temp_output_w_h[i]);
		}
		temp_raw_w_h.clear();
		tmp_corners_hole.clear();
		temp_type_hole.clear();
		temp_output_w_h.clear();
		if (raw_w_h.size() > 0) {
			return true;
		}
		else {
			type_hole.resize(0);
			return false;
		}
	}
	else {
		type_hole.resize(0);
		return false;
	}
}

//bool DetectionHoleClass::DetectionHoleClassFcn(
//	const std::vector <std::vector<cv::Point3f>>& pts_planes,
//	const std::vector<std::vector<int>>& scene_plane_reflect,
//	const float normal_plane[3],
//	const std::vector<cv::Point3f>& corners_plane,
//	const float walltopZ,
//	const std::vector<cv::Point3f>& ground_points,
//	const std::vector<double>& planeMeanStd,
//	const  std::vector<cv::Point3f>& windowPoints,
//	std::vector<std::vector<cv::Point3f>>& corners_hole,
//	std::vector<std::array<float, 2>>& raw_w_h,
//	std::vector<int>& type_hole,
//	const bool isRemeasure,
//	std::vector<array<float, 4>>& output_w_h, const int curid, const map<int, vector<cv::Point3f>>& curWithNeigh, const vector<cv::Point3f>& wallBottomedges,const int k) {
//	std::vector<cv::Point3f> pts_plane = pts_planes[curid];
//	std::vector<int> reflect_plane = scene_plane_reflect[curid];
//	cout << pts_plane.size() << " " << reflect_plane.size() << endl;
//	// data validation
//	if (pts_plane.empty()) {
//		PrintMsg("err0-1: empty input", "DetcHole::Detect(): empty input");
//		return false;
//	}
//	// check plane normal
//	if (std::abs(normal_plane[2]) > 0.1) {
//		corners_hole.resize(0);
//		raw_w_h.resize(0);
//		type_hole.resize(0);
//		return false;
//	}
//	vector<cv::Point3f> neighpoints;
//
//	auto Find = curWithNeigh.find(curid);
//	if (Find != curWithNeigh.end()) {
//		neighpoints.push_back(curWithNeigh.at(curid)[0]);
//		neighpoints.push_back(curWithNeigh.at(curid)[1]);
//	}
//
//	// init decision and sampler
//	Init_DecnTree();
//
//	// step 1: find holes
//	DetectHole(pts_plane, cv::Point3f(normal_plane[0], normal_plane[1], normal_plane[2]), reflect_plane,
//		corners_plane, ground_points, planeMeanStd, windowPoints, corners_hole, raw_w_h, isRemeasure, output_w_h, neighpoints, wallBottomedges[k]);
//
//
//
//	// step 2: classify holes
//	// detect upper and lower border of plane
//	float z_min = 0;
//	float z_max = walltopZ;
//
//
//
//
//	if (raw_w_h.size() > 0) {
//
//		std::vector<std::array<float, 2>> temp_raw_w_h;
//		std::vector<std::vector<cv::Point3f>> tmp_corners_hole;
//		std::vector<int> temp_type_hole;
//		std::vector<array<float, 4>> temp_output_w_h;
//
//
//		for (unsigned int i = 0; i < raw_w_h.size(); i++) {
//
//			z_min = std::numeric_limits<float>::infinity();
//			for (int j = 0; j < pts_plane.size();j++) {
//				if (pts_plane[j].x> min(corners_hole[i][0].x, corners_hole[i][2].x) -5 && pts_plane[j].x < max(corners_hole[i][0].x, corners_hole[i][2].x) +5)
//
//					if (pts_plane[j].y > min(corners_hole[i][0].y, corners_hole[i][2].y) -5 && pts_plane[j].y < max(corners_hole[i][0].y, corners_hole[i][2].y)+5)
//
//						if (pts_plane[j].z < corners_hole[i][0].z )
//						{
//							if ( z_min > pts_plane[j].z)
//								z_min = pts_plane[j].z;
//
//						}
//			}
//
//			if (z_min > 0) {
//				z_min = std::numeric_limits<float>::infinity();
//				for (size_t i = 0; i < pts_plane.size(); ++i) {
//					if (pts_plane[i].z < z_min) z_min = pts_plane[i].z;
//				}
//			}
//
//
//			//	type_hole[i] = ClassifyHole(raw_w_h[i][0], raw_w_h[i][1], corners_hole[i], z_max, z_min);
//			int typeholes = ClassifyHole(raw_w_h[i][0], raw_w_h[i][1], corners_hole[i], z_max, z_min);
//			//cout << typeholes << endl;
//			//log_info("typeholes:%d  zmin: %f",typeholes, z_min );
//			if (typeholes == 0 || typeholes == 1)
//			{
//				temp_raw_w_h.push_back(raw_w_h[i]);
//				tmp_corners_hole.push_back(corners_hole[i]);
//				temp_type_hole.push_back(typeholes);
//				temp_output_w_h.push_back(output_w_h[i]);
//			}
//		}
//		raw_w_h.clear();
//		corners_hole.clear();
//		type_hole.clear();
//		output_w_h.clear();
//		for (unsigned int i = 0; i < temp_raw_w_h.size(); i++) {
//			raw_w_h.push_back(temp_raw_w_h[i]);
//			corners_hole.push_back(tmp_corners_hole[i]);
//			type_hole.push_back(temp_type_hole[i]);
//			output_w_h.push_back(temp_output_w_h[i]);
//		}
//		temp_raw_w_h.clear();
//		tmp_corners_hole.clear();
//		temp_type_hole.clear();
//		temp_output_w_h.clear();
//		if (raw_w_h.size() > 0) {
//			return true;
//		}
//		else {
//			type_hole.resize(0);
//			return false;
//		}
//	}
//	else {
//		type_hole.resize(0);
//		return false;
//	}
//
//}
//
//bool DetectionHoleClass::DetectionHoleClassFcn(
//	const std::vector<cv::Point3f> &pts_plane,
//	const float normal_plane[3],
//	const std::vector<cv::Point3f> & corners_plane,
//	std::vector<std::vector<cv::Point3f>> & corners_hole,
//	std::vector<std::array<float, 2>> &raw_w_h,
//	std::vector<int> &type_hole,
//	const bool isRemeasure,
//	std::vector<array<float, 4>> & output_w_h)
//{
//	// data validation
//	if (pts_plane.empty()) {
//		PrintMsg("err0-1: empty input", "DetcHole::Detect(): empty input");
//		return false;
//	}
//	// check plane normal
//	if (std::abs(normal_plane[2]) > 0.1) {
//		corners_hole.resize(0);
//		raw_w_h.resize(0);
//		type_hole.resize(0);
//		return false;
//	}
//	// init decision and sampler
//	Init_DecnTree();
//
//
//	float center[3] = { 0,0,0 };
//	float uniform_normal[3] = { 0,0,0 };
//	MeasureBase::UniformNormals(normal_plane, center, uniform_normal);
//	uniform_normal[0] = -uniform_normal[0];
//	uniform_normal[1] = -uniform_normal[1];
//	uniform_normal[2] = -uniform_normal[2];
//
//	// step 1: find holes
//	DetectHole(pts_plane, cv::Point3f(uniform_normal[0], uniform_normal[1], uniform_normal[2]),
//		corners_plane, corners_hole, raw_w_h, isRemeasure, output_w_h);
//	/*DetectHole(pts_plane, cv::Point3f(normal_plane[0], normal_plane[1], normal_plane[2]),
//		corners_plane, corners_hole, raw_w_h, isRemeasure, output_w_h);*/
//	// step 2: classify holes
//	// detect upper and lower border of plane
//	float z_min = std::numeric_limits<float>::infinity();
//	float z_max = -std::numeric_limits<float>::infinity();
//	for (size_t i = 0; i < pts_plane.size(); ++i) {
//		if (pts_plane[i].z < z_min) z_min = pts_plane[i].z;
//		if (pts_plane[i].z > z_max) z_max = pts_plane[i].z;
//	}
//
//	if (raw_w_h.size() > 0) {
//
//		std::vector<std::array<float, 2>> temp_raw_w_h;
//		std::vector<std::vector<cv::Point3f>> tmp_corners_hole;
//		std::vector<int> temp_type_hole;
//		std::vector<array<float, 4>> temp_output_w_h;
//
//
//		for (unsigned int i = 0; i < raw_w_h.size(); i++) {
//			//cout << z_max << endl;
//
//			//	type_hole[i] = ClassifyHole(raw_w_h[i][0], raw_w_h[i][1], corners_hole[i], z_max, z_min);
//			int typeholes = ClassifyHole(raw_w_h[i][0], raw_w_h[i][1], corners_hole[i], z_max, z_min);
//			//cout << typeholes << endl;
//			//log_info("typeholes:%d  zmin: %f",typeholes, z_min );
//			if (typeholes == 0|| typeholes==1)
//			{
//				temp_raw_w_h.push_back(raw_w_h[i]);
//				tmp_corners_hole.push_back(corners_hole[i]);
//				temp_type_hole.push_back(typeholes);
//				temp_output_w_h.push_back(output_w_h[i]);
//			}
//		}
//		raw_w_h.clear();
//		corners_hole.clear();
//		type_hole.clear();
//		output_w_h.clear();
//		for (unsigned int i = 0; i < temp_raw_w_h.size(); i++) {
//			raw_w_h.push_back(temp_raw_w_h[i]);
//			corners_hole.push_back(tmp_corners_hole[i]);
//			type_hole.push_back(temp_type_hole[i]);
//			output_w_h.push_back(temp_output_w_h[i]);
//		}
//		temp_raw_w_h.clear();
//		tmp_corners_hole.clear();
//		temp_type_hole.clear();
//		temp_output_w_h.clear();
//		if (raw_w_h.size() > 0){
//			return true;
//		}else {
//			type_hole.resize(0);
//			return false;
//		}
//	}
//	else {
//		type_hole.resize(0);
//		return false;
//	}
//}

bool RotatePointsAroundZ(const std::vector<cv::Point3f>& points, const float angle, std::vector < cv::Point3f>& rotated_points)
{
	if (points.empty())
		return false;

	rotated_points.resize(points.size());
	if (angle == 0.f)
		rotated_points = points;
	else {
		cv::Mat rotation_matrix = cv::Mat::zeros(3, 3, CV_32FC1);
		rotation_matrix.at<float>(0, 0) = cos(angle);
		rotation_matrix.at<float>(0, 1) = -sin(angle);
		rotation_matrix.at<float>(1, 0) = sin(angle);
		rotation_matrix.at<float>(1, 1) = cos(angle);
		rotation_matrix.at<float>(2, 2) = 1.f;
		rotated_points = MathOperation::plane_rot(rotation_matrix, points);
	}

	return true;
}

bool DetectionHoleClass::DetectHole(
	const std::vector<cv::Point3f>  &pts_plane,
	const cv::Point3f normal_plane,
	const std::vector<cv::Point3f> &corners_plane,
	std::vector<std::vector<cv::Point3f>> &corners_hole,
	std::vector<array<float, 2>> &raw_w_h,
	const bool isRemeasure,
	std::vector<array<float, 4>> &output_w_h)
{
	bool isSucceed = true;
	if (pts_plane.empty()) {
		PrintMsg("err4-1: empty input", "DetcHole::FindHole(): empty input");
		isSucceed = false;
		return isSucceed;
	}
	//rotate data
	std::vector<cv::Point3f> pts_plane_rot(pts_plane.size());
	//std::vector<cv::Point3f> corners_plane_rot;
	//corners_plane_rot = corners_plane;
	cv::Point3f normal_plane_rot;
	float angle_rot;
	if (Util_Math::IsValZero(normal_plane.x) || Util_Math::IsValZero(normal_plane.y)) {
		angle_rot = 0.f;
		pts_plane_rot = pts_plane;
		normal_plane_rot = normal_plane;
	}
	else // rotate around y-axis  & project to XZ-plane
	{
		cv::Point3f rot_axis = { 0.f, 1.f, 0.f };
		//float p_normal_plane[3] = { normal_plane.x, normal_plane.y, normal_plane.z };
		cv::Point3f cross = Util_Math::ComputeVectorCrossProduct<cv::Point3f>(normal_plane, rot_axis);
		angle_rot = (cross.z > 0.f) ?
			acosf(normal_plane.y / std::sqrt(std::pow(normal_plane.x, 2) + std::pow(normal_plane.y, 2))) :
			-acosf(normal_plane.y / std::sqrt(std::pow(normal_plane.x, 2) + std::pow(normal_plane.y, 2)));
		RotatePointsAroundZ(pts_plane, angle_rot, pts_plane_rot);

		normal_plane_rot = cv::Point3f(0.f, 1.f, 0.f);
	}
	//ExportPts("D:\\data\\plydata\\rot_plane.txt", pts_plane_rot);

	//find bounding box of plane
	float bbox_plane[6];
	bool isFindBBoxPlane = false;

	if (MathOperation::FindPlaneMinMaxXYZ(pts_plane_rot, 1, 0.f, 0.f, bbox_plane)) {
		isFindBBoxPlane = true;
	}
	// std::vector<std::vector<cv::Point3f>> corners_hole_rot;
	std::vector<array<float, 2>> raw_loc_w;	// search along horizontal direction first
	std::vector<array<float, 2>> raw_loc_h;	// search along vertical direction second

	bool isFindGap_w = false;
	if (isFindBBoxPlane) {
		isFindGap_w = LocateHole_w(pts_plane_rot, normal_plane_rot, bbox_plane, raw_loc_w);
	}

	if (isFindGap_w) {
		bool isFindGap_h = false;
		isFindGap_h = LocateHole_h(pts_plane_rot, normal_plane_rot, bbox_plane, raw_loc_w, raw_loc_h);
		if (isFindGap_h) {
			for (size_t i = 0; i < raw_loc_w.size(); i++)
			{
				std::vector<cv::Point3f> corners;
				bool isSucceed = false;
				if (Util_Math::IsValZero(normal_plane.x)) {
					isSucceed = LocateHole_corner(bbox_plane, CCSPlaneType::PLANE_XZ, std::make_pair(raw_loc_w[i][0],
						raw_loc_w[i][1]), std::make_pair(raw_loc_h[i][0], raw_loc_h[i][1]), 0.f, corners);
				}
				else if (Util_Math::IsValZero(normal_plane.y)) {
					isSucceed = LocateHole_corner(bbox_plane, CCSPlaneType::PLANE_YZ, std::make_pair(raw_loc_w[i][0],
						raw_loc_w[i][1]), std::make_pair(raw_loc_h[i][0], raw_loc_h[i][1]), 0.f, corners);
				}
				else {
					isSucceed = LocateHole_corner(bbox_plane, CCSPlaneType::PLANE_ARB, std::make_pair(raw_loc_w[i][0],
						raw_loc_w[i][1]), std::make_pair(raw_loc_h[i][0], raw_loc_h[i][1]), angle_rot, corners);
				}

				if (isSucceed) {
					raw_w_h.push_back({ raw_loc_w[i][1] - raw_loc_w[i][0], raw_loc_h[i][1] - raw_loc_h[i][0] });
					corners_hole.push_back(corners);
					output_w_h.push_back({ raw_loc_w[i][1] - raw_loc_w[i][0], raw_loc_w[i][1] - raw_loc_w[i][0], raw_loc_h[i][1] - raw_loc_h[i][0] , raw_loc_h[i][1] - raw_loc_h[i][0] });
				}
			}
			//Then remeasure doors & windows by 4 rulers
			/*output_w_h.resize(0);
			if (isRemeasure) {
				if (!Measure(pts_plane_rot, normal_plane_rot, bbox_plane, raw_loc_w, raw_loc_h, output_w_h)) {
					isSucceed = false;
				}
			}*/
		}
	}
	//release memory
	ClearStdVector(pts_plane_rot);
	raw_loc_w.clear();
	raw_loc_h.clear();
	return isSucceed;
}

bool DetectionHoleClass::DetectHole3(const std::vector<cv::Point3f>  &pts_plane,
	const cv::Point3f normal_plane,
	const std::vector<cv::Point3f> &corners_plane,
	std::vector<std::vector<cv::Point3f>> &corners_hole,
	std::vector<array<float, 2>> &raw_w_h,
	const int pidx,
	std::vector<array<float, 4>> &output_w_h)
{
	bool isSucceed = true;
	if (pts_plane.empty()) {
		PrintMsg("err4-1: empty input", "DetcHole::FindHole(): empty input");
		isSucceed = false;
		return isSucceed;
	}
#ifdef DBG_HOLES
	resultA << __FUNCTION__ << "  " << __LINE__ << endl;
#endif
	std::vector<std::vector<cv::Point3f>> rot_corners;
	std::vector<cv::Point3f> verticles_rot(corners_plane.size());
	std::vector<cv::Point3f> pts_plane_rot(pts_plane.size());
	cv::Point3f normal_plane_rot;
	float angle_rot;
	if (Util_Math::IsValZero(normal_plane.x) || Util_Math::IsValZero(normal_plane.y)) {
		angle_rot = 0.f;
		pts_plane_rot = pts_plane;
		normal_plane_rot = normal_plane;
		verticles_rot = corners_plane;
	}
	else // rotate around y-axis  & project to XZ-plane
	{
		cv::Point3f rot_axis = { 0.f, 1.f, 0.f };
		cv::Point3f cross = Util_Math::ComputeVectorCrossProduct<cv::Point3f>(normal_plane, rot_axis);
		angle_rot = (cross.z > 0.f) ?
			acosf(normal_plane.y / std::sqrt(std::pow(normal_plane.x, 2) + std::pow(normal_plane.y, 2))) :
			-acosf(normal_plane.y / std::sqrt(std::pow(normal_plane.x, 2) + std::pow(normal_plane.y, 2)));
		RotatePointsAroundZ(pts_plane, angle_rot, pts_plane_rot);
		RotatePointsAroundZ(corners_plane, angle_rot, verticles_rot);
		normal_plane_rot = cv::Point3f(0.f, 1.f, 0.f);
	}
#ifdef DBG_HOLES_SV
	SaveSceneData(filePath + std::string("DetectHole3\\"), 11110000 + pidx, pts_plane_rot);//原始平面
#endif
	float bbox_plane[6];
	bool isFindBBoxPlane = false;

	if (MathOperation::FindPlaneMinMaxXYZ(pts_plane_rot, 1, 0.f, 0.f, bbox_plane))
	{
		isFindBBoxPlane = true;
	}
	std::vector<array<float, 2>> raw_loc_w;
	std::vector<array<float, 2>> raw_loc_h;

	bool isFindGap_w = false;
	if (isFindBBoxPlane)
	{
		isFindGap_w = LocateHole_w(pts_plane_rot,
			normal_plane_rot,
			bbox_plane,
			raw_loc_w);
	}
	if (isFindGap_w)
	{
		bool isFindGap_h = false;
		isFindGap_h = LocateHole_h(pts_plane_rot,
			normal_plane_rot,
			bbox_plane,
			raw_loc_w, //i
			raw_loc_h);

		if (isFindGap_h) {
			for (size_t i = 0; i < raw_loc_w.size(); i++)
			{
				std::vector<cv::Point3f> corners;
				std::vector<cv::Point3f> corners_rot;
				if (Util_Math::IsValZero(normal_plane.x)) {
					isSucceed = LocateHoleRot_corner(bbox_plane,
						CCSPlaneType::PLANE_XZ,
						std::make_pair(raw_loc_w[i][0], raw_loc_w[i][1]),
						std::make_pair(raw_loc_h[i][0], raw_loc_h[i][1]),
						0.f, corners, corners_rot);
				}
				else if (Util_Math::IsValZero(normal_plane.y)) {
					isSucceed = LocateHoleRot_corner(bbox_plane, CCSPlaneType::PLANE_YZ, std::make_pair(raw_loc_w[i][0],
						raw_loc_w[i][1]), std::make_pair(raw_loc_h[i][0], raw_loc_h[i][1]), 0.f, corners, corners_rot);
				}
				else {
					isSucceed = LocateHoleRot_corner(bbox_plane, CCSPlaneType::PLANE_ARB, std::make_pair(raw_loc_w[i][0],
						raw_loc_w[i][1]), std::make_pair(raw_loc_h[i][0], raw_loc_h[i][1]), angle_rot, corners, corners_rot);
				}

				if (isSucceed)
				{
					raw_w_h.push_back({ raw_loc_w[i][1] - raw_loc_w[i][0], raw_loc_h[i][1] - raw_loc_h[i][0] });
					corners_hole.push_back(corners);
					rot_corners.push_back(corners_rot); //四个顶点
					output_w_h.push_back({ raw_loc_w[i][1] - raw_loc_w[i][0], raw_loc_w[i][1] - raw_loc_w[i][0], raw_loc_h[i][1] - raw_loc_h[i][0] ,  raw_loc_h[i][1] - raw_loc_h[i][0] });
				}
			}
		}
	}

#ifdef DBG_HOLES_SV
	std::vector<cv::Point3f> temp;
	for (int i = 0; i < rot_corners.size(); i++)
	{
		for (int w = 0; w < rot_corners[i].size(); w++)
		{
			temp.push_back(rot_corners[i][w]);
		}
	}
	SaveSceneData(filePath + std::string("DetectHole3\\"), 7 * 10000 + pidx, temp);//第一遍检测的墙面pidx上的洞
	temp.clear();
	for (int w = 0; w < verticles_rot.size(); w++)
	{
		temp.push_back(verticles_rot[w]);
	}
	SaveSceneData(filePath + std::string("DetectHole3\\"), 6 * 10000 + pidx, temp);//第一遍检测的墙面pidx上的洞
#endif

																				   //==============================================================
	std::vector<std::vector<cv::Point3f>> new_corners;
	std::vector<bool> need_rs;
	new_corners.resize(rot_corners.size());
	need_rs.resize(rot_corners.size());
	for (int i = 0; i < rot_corners.size(); i++)
	{
		new_corners[i].resize(rot_corners[i].size());
		new_corners[i] = rot_corners[i];
		need_rs[i] = false;
#ifdef DBG_HOLES
		resultA << "rot_corners, pidx: " << pidx << " holes: " << i << " \n"
			<< rot_corners[i][0] << " \n" << rot_corners[i][1] << "\n "
			<< rot_corners[i][2] << " \n" << rot_corners[i][3] << endl;
#endif
	}
#ifdef DBG_HOLES
	resultA << "verticles_rot, pidx: " << pidx << " \n"
		<< verticles_rot[0] << " \n" << verticles_rot[1] << "\n "
		<< verticles_rot[2] << " \n" << verticles_rot[3] << endl;
#endif
	std::vector<cv::Point3f> temp3;
	for (int i = 0; i < rot_corners.size(); i++)
	{
		float minx = (rot_corners[i][0].x + rot_corners[i][3].x) / 2.0;
		float maxx = (rot_corners[i][1].x + rot_corners[i][2].x) / 2.0;
		float minz = (rot_corners[i][0].z + rot_corners[i][1].z) / 2.0;
		float maxz = (rot_corners[i][3].z + rot_corners[i][2].z) / 2.0;
#ifdef DBG_HOLES
		cout << "minx: " << minx << " maxx: " << maxx << " minz: " << minz << " maxz: " << maxz << endl;
#endif
		int grids = 10;
		float grids_len = 0;
		for (int j = 0; j < rot_corners[i].size(); j++)
		{

			if (0 == j) {
				if ((int)rot_corners[i][j].x == (int)verticles_rot[j].x) {
#ifdef DBG_HOLES
					resultA << "Board line hole, j:" << j << endl;
#endif
					continue;
				}
				grids_len = abs(rot_corners[i][j].z - rot_corners[i][3].z) / grids;
			}
			if (1 == j) {
				if ((int)rot_corners[i][j].z == (int)verticles_rot[j].z) {
#ifdef DBG_HOLES
					resultA << "Board line hole, j:" << j << endl;
#endif
					continue;
				}
				grids_len = abs(rot_corners[i][j].x - rot_corners[i][0].x) / grids;
			}
			if (2 == j) {
				if ((int)rot_corners[i][j].x == (int)verticles_rot[j].x) {
#ifdef DBG_HOLES
					resultA << "Board line hole, j:" << j << endl;
#endif
					continue;
				}
				grids_len = abs(rot_corners[i][j].z - rot_corners[i][1].z) / grids;
			}
			if (3 == j) {
				if ((int)rot_corners[i][j].z == (int)verticles_rot[j].z) {
#ifdef DBG_HOLES
					resultA << "Board line hole, j:" << j << endl;
#endif
					continue;
				}
				grids_len = abs(rot_corners[i][j].x - rot_corners[i][2].x) / grids;
			}
#ifdef DBG_HOLES
			cout << "\n grids_len: " << grids_len << endl;
#endif
			std::vector<bool> grid_status(grids);
			std::vector<std::pair<float, float>> grid_range(grids);
			for (int k = 0; k < grid_status.size(); k++)
			{
				if (0 == j)
				{
					grid_range[k].first = rot_corners[i][0].z + grids_len * k;
					grid_range[k].second = rot_corners[i][0].z + grids_len * (k + 1);
				}
				if (1 == j)
				{
					grid_range[k].first = rot_corners[i][0].x + grids_len * k;
					grid_range[k].second = rot_corners[i][0].x + grids_len * (k + 1);
				}
				if (2 == j) {
					grid_range[k].first = rot_corners[i][1].z + grids_len * k;
					grid_range[k].second = rot_corners[i][1].z + grids_len * (k + 1);
				}
				if (3 == j) {
					grid_range[k].first = rot_corners[i][3].x + grids_len * k;
					grid_range[k].second = rot_corners[i][3].x + grids_len * (k + 1);
				}
#ifdef DBG_HOLES
				cout << "grid_range[" << k << "]: " << grid_range[k].first << " , " << grid_range[k].second << endl;
#endif
			}
			float ruler_c = 0;
			float step = 50;
			if (0 == j) ruler_c = rot_corners[i][j].x + step;
			if (1 == j) ruler_c = rot_corners[i][j].z + step;
			if (2 == j)	ruler_c = rot_corners[i][j].x - step;
			if (3 == j) ruler_c = rot_corners[i][j].z - step;
			bool move_ruler = true;
			int move_cnt = 0;

			while (move_cnt < 5)
			{
				if (0 == j) {
					ruler_c -= step;
#ifdef DBG_HOLES
					cout << "##while: i:" << i << " ruler_c: " << ruler_c << " [" << verticles_rot[0].x << " , " << verticles_rot[1].x << "]" << endl;
#endif
				}
				if (1 == j) {
					ruler_c -= step;
#ifdef DBG_HOLES
					cout << "##while: i:" << i << " ruler_c: " << ruler_c << " [" << verticles_rot[0].z << " , " << verticles_rot[3].z << "]" << endl;
#endif
				}
				if (2 == j) {
					ruler_c += step;
#ifdef DBG_HOLES
					cout << "##while: i:" << i << " ruler_c: " << ruler_c << " [" << verticles_rot[0].x << " , " << verticles_rot[1].x << "]" << endl;
#endif
				}
				if (3 == j) {
					ruler_c += step;
#ifdef DBG_HOLES
					cout << "##while: i:" << i << " ruler_c: " << ruler_c << " [" << verticles_rot[0].z << " , " << verticles_rot[3].z << "]" << endl;
#endif
				}

				std::pair<float, float> ruler_v;
				//if (0 == move_cnt) 
				//	ruler_v = std::make_pair(ruler_c - step / 1.5, ruler_c + step / 1.5);
				//else
				ruler_v = std::make_pair(ruler_c - step / 2.0, ruler_c + step / 2.0);
				std::vector<unsigned int>rulers_idx;
				if ((0 == j) || (2 == j)) {
					AppendPt2Ruler1D(pts_plane_rot, ruler_v, AxisType::Axis_X, rulers_idx);
#ifdef DBG_HOLES
					cout << "rulers_idx.size(): " << rulers_idx.size() << endl;
#endif
				}
				if ((1 == j) || (3 == j)) {
					AppendPt2Ruler1D(pts_plane_rot, ruler_v, AxisType::Axis_Z, rulers_idx);
#ifdef DBG_HOLES
					cout << "rulers_idx.size(): " << rulers_idx.size() << endl;
#endif
				}
#ifdef DBG_HOLES_SV
				std::vector<cv::Point3f> temp; ///sv ruler
				for (int w = 0; w < rulers_idx.size(); w++)
				{
					temp.push_back(pts_plane_rot[rulers_idx[w]]);
				}
				SaveSceneData(filePath + std::string("DetectHole3\\"), pidx * 1000000 + i * 10000 + j * 100 + move_cnt, temp);
#endif
				for (int m = 0; m < rulers_idx.size(); m++)
				{
					if (((0 == j) || (2 == j)) &&
						(pts_plane_rot[rulers_idx[m]].z > minz) && (pts_plane_rot[rulers_idx[m]].z < maxz))
					{
						for (int n = 0; n < grid_range.size(); n++)
						{
							if ((pts_plane_rot[rulers_idx[m]].z > grid_range[n].first) &&
								(pts_plane_rot[rulers_idx[m]].z < grid_range[n].second))
							{
								grid_status[n] = true;
							}
						}
					}
					if (((1 == j) || (3 == j)) &&
						(pts_plane_rot[rulers_idx[m]].x > minx) && (pts_plane_rot[rulers_idx[m]].x < maxx))
					{
						for (int n = 0; n < grid_range.size(); n++)
						{
							if ((pts_plane_rot[rulers_idx[m]].x > grid_range[n].first) &&
								(pts_plane_rot[rulers_idx[m]].x < grid_range[n].second))
							{
								grid_status[n] = true;
							}
						}
					}
				}
				int cnt = 0;
#ifdef DBG_HOLES
				cout << "==grid_status, board: " << j << "  move_cnt:" << move_cnt << endl;
				resultA << "==grid_status, board: " << j << "  move_cnt:" << move_cnt << endl;
#endif
				for (int k = 0; k < grid_status.size(); k++)
				{
					if (grid_status[k])
						cnt++;
#ifdef DBG_HOLES
					cout << grid_status[k] << "  ";
					resultA << grid_status[k] << "  ";
#endif
				}
#ifdef DBG_HOLES
				cout << endl;
				resultA << endl;
#endif
				if (cnt == grid_status.size())
				{
					move_ruler = false;
#ifdef DBG_HOLES
					resultA << "while break, move_cnt:" << move_cnt << endl;
#endif
					break;
				}
				else {
					move_ruler = true;//
					move_cnt++;
#ifdef DBG_HOLES
					cout << "Invailid board, need move ruler move_cnt:" << move_cnt << endl;
#endif
				}
			}//while

			if (move_cnt != 0)
			{
				need_rs[i] = true; /// one change make rs true;
				if (0 == j)
					new_corners[i][j].x = new_corners[i][3].x = ruler_c + step / 2.0;
				if (1 == j)
					new_corners[i][j].z = new_corners[i][0].z = ruler_c + step / 2.0;
				if (2 == j)
					new_corners[i][j].x = new_corners[i][1].x = ruler_c - step / 2.0;
				if (3 == j)
					new_corners[i][j].z = new_corners[i][2].z = ruler_c - step / 2.0;
			}
			else {
				if (0 == j)
					new_corners[i][j].x = new_corners[i][3].x = new_corners[i][j].x + step / 2.0;
				if (1 == j)
					new_corners[i][j].z = new_corners[i][0].z = new_corners[i][j].z + step / 2.0;
				if (2 == j)
					new_corners[i][j].x = new_corners[i][1].x = new_corners[i][j].x - step / 2.0;
				if (3 == j)
					new_corners[i][j].z = new_corners[i][2].z = new_corners[i][j].z - step / 2.0;

			}
		}//rot_corners[i].size j

#ifdef DBG_HOLES_SV

		for (int w = 0; w < new_corners[i].size(); w++)//move_cnt 后的corners
		{
			temp3.push_back(new_corners[i][w]);
		}

#endif
	}//for  rot_corners  i
#ifdef DBG_HOLES_SV
	SaveSceneData(filePath + std::string("DetectHole3\\"), 8 * 10000 + pidx, temp3);//重新检测pidx上所有洞的轮廓
#endif
	bool Erased = false;
	for (int d = 0; d < new_corners.size(); d++)
	{
		if (need_rs[d])
		{
#ifdef DBG_HOLES_SV
			cout << "erasing....." << endl;
			std::vector<cv::Point3f> temp_e;
			resultA << "##Erasing rubish..." << endl;
#endif		
			Erased = true;
			for (int i = 0; i < pts_plane_rot.size(); i++)
			{
				if ((pts_plane_rot[i].x > new_corners[d][0].x) &&
					(pts_plane_rot[i].x < new_corners[d][1].x) &&
					(pts_plane_rot[i].z > new_corners[d][0].z) &&
					(pts_plane_rot[i].z < new_corners[d][3].z))
				{
#ifdef DBG_HOLES_SV
					temp_e.push_back(pts_plane_rot[i]);
#endif
					pts_plane_rot.erase(pts_plane_rot.begin() + i);
					i--;
				}
			}
#ifdef DBG_HOLES_SV
			SaveSceneData(filePath + std::string("DetectHole3\\"), 999 * 10000 + pidx * 100 + d, temp_e);
#endif
		}
	}
	if (!Erased)
	{
		ClearStdVector(pts_plane_rot);
		raw_loc_w.clear();
		raw_loc_h.clear();
		return isSucceed;
	}
#ifdef DBG_HOLES_SV
	SaveSceneData(filePath + std::string("DetectHole3\\"), 22220000 + pidx, pts_plane_rot);
#endif
	//===================================
	isFindGap_w = false;
	raw_loc_w.clear();
	raw_loc_h.clear();
	raw_w_h.clear();
	output_w_h.clear();
	corners_hole.clear();
	rot_corners.clear();

	if (isFindBBoxPlane)
	{
		isFindGap_w = LocateHole_w(pts_plane_rot,
			normal_plane_rot,
			bbox_plane,
			raw_loc_w);
#ifdef DBG_HOLES
		for (int n = 0; n < raw_loc_w.size(); n++) {
			std::cout << "w_rule: " << n << "  " << raw_loc_w[n][0] << " , " << raw_loc_w[n][1] << endl;
		}
#endif
	}
	if (isFindGap_w)
	{
		bool isFindGap_h = false;
		isFindGap_h = LocateHole_h(pts_plane_rot,
			normal_plane_rot,
			bbox_plane,
			raw_loc_w, //i
			raw_loc_h);

		if (isFindGap_h) {
			for (size_t i = 0; i < raw_loc_w.size(); i++)
			{
				std::vector<cv::Point3f> corners;
				std::vector<cv::Point3f> corners_rot;
				if (Util_Math::IsValZero(normal_plane.x)) {
					isSucceed = LocateHoleRot_corner(bbox_plane,
						CCSPlaneType::PLANE_XZ,
						std::make_pair(raw_loc_w[i][0], raw_loc_w[i][1]),
						std::make_pair(raw_loc_h[i][0], raw_loc_h[i][1]),
						0.f, corners, corners_rot);
				}
				else if (Util_Math::IsValZero(normal_plane.y)) {
					isSucceed = LocateHoleRot_corner(bbox_plane, CCSPlaneType::PLANE_YZ, std::make_pair(raw_loc_w[i][0],
						raw_loc_w[i][1]), std::make_pair(raw_loc_h[i][0], raw_loc_h[i][1]), 0.f, corners, corners_rot);
				}
				else {
					isSucceed = LocateHoleRot_corner(bbox_plane, CCSPlaneType::PLANE_ARB, std::make_pair(raw_loc_w[i][0],
						raw_loc_w[i][1]), std::make_pair(raw_loc_h[i][0], raw_loc_h[i][1]), angle_rot, corners, corners_rot);
				}

				if (isSucceed)
				{
					raw_w_h.push_back({ raw_loc_w[i][1] - raw_loc_w[i][0], raw_loc_h[i][1] - raw_loc_h[i][0] });
					corners_hole.push_back(corners); //四个顶点
					rot_corners.push_back(corners_rot);
					output_w_h.push_back({ raw_loc_w[i][1] - raw_loc_w[i][0],raw_loc_w[i][1] - raw_loc_w[i][0],
						raw_loc_h[i][1] - raw_loc_h[i][0] ,raw_loc_h[i][1] - raw_loc_h[i][0] });
				}
			}
		}
	}

#ifdef DBG_HOLES
	std::vector<cv::Point3f> temp2;
	for (int i = 0; i < rot_corners.size(); i++)
	{
		for (int w = 0; w < rot_corners[i].size(); w++)
		{
			temp2.push_back(rot_corners[i][w]);
		}
	}
	SaveSceneData("dll_log\\", 9 * 10000 + pidx, temp2);
#endif
	//release memory
	ClearStdVector(pts_plane_rot);
	raw_loc_w.clear();
	raw_loc_h.clear();
	return isSucceed;
}

//
//bool DetectionHoleClass::DetectHole(
//	const std::vector<cv::Point3f>& pts_plane,
//	const cv::Point3f normal_plane,
//	const std::vector<int>& reflect_plane,
//	const std::vector<cv::Point3f>& corners_plane,
//	const std::vector<cv::Point3f>& ground_points,
//	const std::vector<double>& planeMeanStd,
//	const  std::vector<cv::Point3f>& windowPoints,
//	std::vector<std::vector<cv::Point3f>>& corners_hole,
//	std::vector<array<float, 2>>& raw_w_h,
//	const bool isRemeasure,
//	std::vector<array<float, 4>>& output_w_h, const vector<cv::Point3f>& neighpoints,const cv::Point3f& bottompt)
//{
//	bool isSucceed = true;
//	if (pts_plane.empty()) {
//		PrintMsg("err4-1: empty input", "DetcHole::FindHole(): empty input");
//		isSucceed = false;
//		return isSucceed;
//	}
//	vector<cv::Point3f> allRemovedPts;
//	vector<int> allRemovedReflect;
//	for (int i = 0; i < rmdata.removePts.size(); i++) {
//		//if (std::abs(rmdata.removeNormal[i].x * normal_plane.x + rmdata.removeNormal[i].y * normal_plane.y + rmdata.removeNormal[i].z * normal_plane.z)<0.3 ) {
//			for (int j = 0; j < rmdata.removePts[i].size(); j++) {
//				allRemovedPts.push_back(rmdata.removePts[i][j]);
//				allRemovedReflect.push_back(rmdata.removeReflect[i][j]);
//
//			}
//		//}
//	}
//	// ExportPts("D:\\data\\plydata\\allRemovedPts.obj", allRemovedPts, allRemovedReflect);
//	//ExportPts("D:\\data\\plydata\\pts_plane.txt", pts_plane);
//	//rotate data
//	std::vector<cv::Point3f> pts_plane_rot(pts_plane.size());
//	std::vector<cv::Point3f> neighpoints_rot(neighpoints.size());
//	std::vector<cv::Point3f> ground_rot(ground_points.size());
//	std::vector<cv::Point3f> allRemovedPts_rot(allRemovedPts.size());
//	std::vector<cv::Point3f> windowPoints_rot(windowPoints.size());
//	//corners_plane_rot = corners_plane;
//	cv::Point3f normal_plane_rot;
//	float angle_rot;
//	if (IsValZero(normal_plane.x) || IsValZero(normal_plane.y)) {
//		angle_rot = 0.f;
//		pts_plane_rot = pts_plane;
//		normal_plane_rot = normal_plane;
//		neighpoints_rot = neighpoints;
//		ground_rot = ground_points;
//		allRemovedPts_rot = allRemovedPts;
//		windowPoints_rot = windowPoints;
//	}
//	else // rotate around y-axis  & project to XZ-plane
//	{
//		float rot_axis[3] = { 0.f, 1.f, 0.f };
//		float cross[3];
//		float p_normal_plane[3] = { normal_plane.x, normal_plane.y, normal_plane.z };
//		MeasureBase::CrossProduct(p_normal_plane, rot_axis, cross);
//		angle_rot = (cross[2] > 0.f) ?
//			acosf(normal_plane.y / std::sqrt(std::pow(normal_plane.x, 2) + std::pow(normal_plane.y, 2))) :
//			-acosf(normal_plane.y / std::sqrt(std::pow(normal_plane.x, 2) + std::pow(normal_plane.y, 2)));
//		MeasureBase::RotatePointsAroundZ(pts_plane, angle_rot, pts_plane_rot);
//		MeasureBase::RotatePointsAroundZ(neighpoints, angle_rot, neighpoints_rot);
//		MeasureBase::RotatePointsAroundZ(ground_points, angle_rot, ground_rot);
//		MeasureBase::RotatePointsAroundZ(allRemovedPts, angle_rot, allRemovedPts_rot);
//		MeasureBase::RotatePointsAroundZ(windowPoints, angle_rot, windowPoints_rot);
//		normal_plane_rot = cv::Point3f(0.f, 1.f, 0.f);
//	}
//	//ExportPts("D:\\data\\plydata\\rot_plane.txt", pts_plane_rot);
//
//	//find bounding box of plane
//	float bbox_plane[6];
//	bool isFindBBoxPlane = false;
//
//	if (MeasureBase::FindPlaneMinMaxXYZ(pts_plane_rot, 1, 0.f, 0.f, bbox_plane)) {
//		isFindBBoxPlane = true;
//	}
//	// std::vector<std::vector<cv::Point3f>> corners_hole_rot;
//	std::vector<array<float, 2>> raw_loc_w;	// search along horizontal direction first
//	std::vector<array<float, 2>> raw_loc_h;	// search along vertical direction second
//
//	bool isFindGap_w = false;
//	if (isFindBBoxPlane) {
//		isFindGap_w = LocateHole_w(pts_plane_rot, normal_plane_rot, bbox_plane, raw_loc_w);
//	}
//
//	if (isFindGap_w) {
//		bool isFindGap_h = false;
//		isFindGap_h = LocateHole_h(pts_plane_rot, normal_plane_rot, bbox_plane, raw_loc_w, raw_loc_h);
//
//		if (isFindGap_h) {
//			//cout << rmdata.removePts.size() << endl;
//
//			edgeFitting(pts_plane_rot, reflect_plane, windowPoints_rot,  allRemovedPts_rot, allRemovedReflect, ground_rot, normal_plane_rot, bbox_plane, planeMeanStd, raw_loc_w, raw_loc_h, neighpoints_rot, bottompt);
//			for (size_t i = 0; i < raw_loc_w.size(); i++)
//			{
//				std::vector<cv::Point3f> corners;
//				bool isSucceed = false;
//				if (IsValZero(normal_plane.x)) {
//					isSucceed = LocateHole_corner(bbox_plane, CCSPlaneType::PLANE_XZ, std::make_pair(raw_loc_w[i][0],
//						raw_loc_w[i][1]), std::make_pair(raw_loc_h[i][0], raw_loc_h[i][1]), 0.f, corners);
//				}
//				else if (IsValZero(normal_plane.y)) {
//					isSucceed = LocateHole_corner(bbox_plane, CCSPlaneType::PLANE_YZ, std::make_pair(raw_loc_w[i][0],
//						raw_loc_w[i][1]), std::make_pair(raw_loc_h[i][0], raw_loc_h[i][1]), 0.f, corners);
//				}
//				else {
//					isSucceed = LocateHole_corner(bbox_plane, CCSPlaneType::PLANE_ARB, std::make_pair(raw_loc_w[i][0],
//						raw_loc_w[i][1]), std::make_pair(raw_loc_h[i][0], raw_loc_h[i][1]), angle_rot, corners);
//				}
//
//				if (isSucceed) {
//					raw_w_h.push_back({ raw_loc_w[i][1] - raw_loc_w[i][0], raw_loc_h[i][1] - raw_loc_h[i][0] });
//					corners_hole.push_back(corners);
//					output_w_h.push_back({ raw_loc_w[i][1] - raw_loc_w[i][0], raw_loc_w[i][1] - raw_loc_w[i][0], raw_loc_h[i][1] - raw_loc_h[i][0] , raw_loc_h[i][1] - raw_loc_h[i][0] });
//				}
//			}
//			//Then remeasure doors & windows by 4 rulers
//			/*output_w_h.resize(0);
//			if (isRemeasure) {
//				if (!Measure(pts_plane_rot, normal_plane_rot, bbox_plane, raw_loc_w, raw_loc_h, output_w_h)) {
//					isSucceed = false;
//				}
//			}*/
//		}
//	}
//	//release memory
//	ClearStdVector(pts_plane_rot);
//	raw_loc_w.clear();
//	raw_loc_h.clear();
//	return isSucceed;
//}
//
//
bool DetectionHoleClass::LocateHoleRot_corner(
	const float *bbox_plane,
	const CCSPlaneType &planeType,
	const std::pair<float, float> &bound_w,
	const std::pair<float, float> &bound_h,
	const float angle_rotZ,
	std::vector<cv::Point3f> &corners,
	std::vector<cv::Point3f> &rot_corners)
{
	// data validation
	if (bound_h.first > bound_h.second)
		return false;
	std::vector<cv::Point3f> corners_hole_rot;
	corners_hole_rot.resize(4);

	float fixed_coord = 0.f;
	if (planeType == CCSPlaneType::PLANE_XZ) {
		fixed_coord = (bbox_plane[2] + bbox_plane[3]) * 0.5f;
		corners_hole_rot[0] = cv::Point3f(bound_w.first, fixed_coord, bound_h.first);
		corners_hole_rot[1] = cv::Point3f(bound_w.second, fixed_coord, bound_h.first);
		corners_hole_rot[2] = cv::Point3f(bound_w.second, fixed_coord, bound_h.second);
		corners_hole_rot[3] = cv::Point3f(bound_w.first, fixed_coord, bound_h.second);
	}
	else if (planeType == CCSPlaneType::PLANE_YZ) {
		fixed_coord = (bbox_plane[0] + bbox_plane[1]) * 0.5f;
		corners_hole_rot[0] = cv::Point3f(fixed_coord, bound_w.first, bound_h.first);
		corners_hole_rot[1] = cv::Point3f(fixed_coord, bound_w.second, bound_h.first);
		corners_hole_rot[2] = cv::Point3f(fixed_coord, bound_w.second, bound_h.second);
		corners_hole_rot[3] = cv::Point3f(fixed_coord, bound_w.first, bound_h.second);
	}
	else {
		fixed_coord = (bbox_plane[2] + bbox_plane[3]) * 0.5f;
		corners_hole_rot[0] = cv::Point3f(bound_w.first, fixed_coord, bound_h.first);
		corners_hole_rot[1] = cv::Point3f(bound_w.second, fixed_coord, bound_h.first);
		corners_hole_rot[2] = cv::Point3f(bound_w.second, fixed_coord, bound_h.second);
		corners_hole_rot[3] = cv::Point3f(bound_w.first, fixed_coord, bound_h.second);
	}
	float len_w = fabs(bound_w.second - bound_w.first);
	float len_h = fabs(bound_h.second - bound_h.first);
	rot_corners = corners_hole_rot;
#if 1
	if (planeType == CCSPlaneType::PLANE_ARB) {
		std::vector<cv::Point3f> temp;
		temp.resize(4);
		RotatePointsAroundZ(corners_hole_rot, -angle_rotZ, temp);
		corners_hole_rot = temp;
		temp.clear();
	}
#endif
	if ((len_w > m_thres_minLen_hole) && (len_w < m_thres_maxLen_hole) && (len_h > m_thres_minLen_hole)) {
		corners = corners_hole_rot;
		corners_hole_rot.clear();
		return true;
	}
	corners_hole_rot.clear();
	return false;
}
bool DetectionHoleClass::LocateHole_corner(
	const float *bbox_plane,
	const CCSPlaneType &planeType,
	const std::pair<float, float> &bound_w,
	const std::pair<float, float> &bound_h,
	const float angle_rotZ,
	std::vector<cv::Point3f> &corners)
{
	// data validation
	if (bound_h.first > bound_h.second) return false;
	std::vector<cv::Point3f> corners_hole_rot;
	corners_hole_rot.resize(4);

	float fixed_coord = 0.f;
	if (planeType == CCSPlaneType::PLANE_XZ) {
		fixed_coord = (bbox_plane[2] + bbox_plane[3]) * 0.5f;
		corners_hole_rot[0] = cv::Point3f(bound_w.first, fixed_coord, bound_h.first);
		corners_hole_rot[1] = cv::Point3f(bound_w.second, fixed_coord, bound_h.first);
		corners_hole_rot[2] = cv::Point3f(bound_w.second, fixed_coord, bound_h.second);
		corners_hole_rot[3] = cv::Point3f(bound_w.first, fixed_coord, bound_h.second);
	}
	else if (planeType == CCSPlaneType::PLANE_YZ) {
		fixed_coord = (bbox_plane[0] + bbox_plane[1]) * 0.5f;
		corners_hole_rot[0] = cv::Point3f(fixed_coord, bound_w.first, bound_h.first);
		corners_hole_rot[1] = cv::Point3f(fixed_coord, bound_w.second, bound_h.first);
		corners_hole_rot[2] = cv::Point3f(fixed_coord, bound_w.second, bound_h.second);
		corners_hole_rot[3] = cv::Point3f(fixed_coord, bound_w.first, bound_h.second);
	}
	else {
		fixed_coord = (bbox_plane[2] + bbox_plane[3]) * 0.5f;
		corners_hole_rot[0] = cv::Point3f(bound_w.first, fixed_coord, bound_h.first);
		corners_hole_rot[1] = cv::Point3f(bound_w.second, fixed_coord, bound_h.first);
		corners_hole_rot[2] = cv::Point3f(bound_w.second, fixed_coord, bound_h.second);
		corners_hole_rot[3] = cv::Point3f(bound_w.first, fixed_coord, bound_h.second);
	}
	float len_w = fabs(bound_w.second - bound_w.first);
	float len_h = fabs(bound_h.second - bound_h.first);
	if (planeType == CCSPlaneType::PLANE_ARB) {
		std::vector<cv::Point3f> temp;
		temp.resize(4);
		RotatePointsAroundZ(corners_hole_rot, -angle_rotZ, temp);
		corners_hole_rot = temp;
		temp.clear();
	}
	//cout << len_w <<" " << len_h <<" "<< m_thres_maxLen_hole <<" "<< m_thres_minLen_hole << endl;
	if ((len_w > m_thres_minLen_hole) && (len_w < m_thres_maxLen_hole) && (len_h > m_thres_minLen_hole)) {
		corners = corners_hole_rot;
		corners_hole_rot.clear();
		return true;
	}
	corners_hole_rot.clear();
	return false;
}

bool DetectionHoleClass::checkHoleEdge(const std::vector<cv::Point3f> &pts_plane, const cv::Point3f normal_plane, const float* bbox_plane, Bbox &hole) {
	std::vector<array<float, 2>> gaps_loc_ruler;
	std::vector<std::vector<array<float, 2>>> gaps_loc_hole;
	float center_v = hole.y + hole.h * 0.5f;
	unsigned int n_ruler = 1; // number of ruler
	std::vector<std::vector<unsigned int>> ptID_rulers;
	ptID_rulers.resize(n_ruler);
	// compute ruler bounds at v direction
	std::vector<std::pair<float, float>> bbox_ruler_v;
	bbox_ruler_v.resize(n_ruler);
	unsigned int count = 0;
	//k = -1: 1st ruler at (center_v - spacing_ruler);
	//k = 0:  2nd ruler at center_v;
	//k = 1:  3rd ruler at (center_v + spacing_ruler)

	bbox_ruler_v[0] = std::make_pair(
		center_v - m_halfWidth_ruler * 1.5,
		center_v + m_halfWidth_ruler * 1.5);

	// append points in ruler
	AppendPt2Ruler1D(pts_plane, bbox_ruler_v[0], AxisType::Axis_Z, ptID_rulers[0]);
	cv::Mat vals_ruler(static_cast<int>(ptID_rulers[0].size()), 1, CV_32F);
	cv::Mat sorted_ptID_ruler(static_cast<int>(ptID_rulers[0].size()), 1, CV_32S);
	if (Util_Math::IsValZero(normal_plane.x)) {
		for (int i = 0; i < ptID_rulers[0].size(); i++) {
			vals_ruler.at<float>(i, 0) = pts_plane[ptID_rulers[0][i]].x;
		}
	}
	else if (Util_Math::IsValZero(normal_plane.y)) {
		for (int i = 0; i < ptID_rulers[0].size(); i++) {
			vals_ruler.at<float>(i, 0) = pts_plane[ptID_rulers[0][i]].y;
		}
	}
	else {
		//PrintMsg("err2-2: without rotation of wall", "DetcHole::GetHoleWidth(): without rotation for wall");
		return false;
	}
	if (vals_ruler.rows == 0) {
		return true;
	}
	std::pair<float, float> bbox_u;
	//parallel to x-axis
	if (Util_Math::IsValZero(normal_plane.x)) {
		bbox_u.first = bbox_plane[0];
		bbox_u.second = bbox_plane[1];
	}
	//parallel to y-axis
	else if (Util_Math::IsValZero(normal_plane.y)) {
		bbox_u.first = bbox_plane[2];
		bbox_u.second = bbox_plane[3];
	}
	cv::sortIdx(vals_ruler, sorted_ptID_ruler, CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);
	bool isGapInRuler = FindGapsInRuler(vals_ruler, sorted_ptID_ruler, bbox_u, gaps_loc_ruler);
	//ExportPts("D:\\data\\plydata\\rulerdata.txt", pts_plane, ptID_rulers[0], sorted_ptID_ruler);
	//ExportPts("D:\\data\\plydata\\rulerdata2.txt", pts_plane);
	if (isGapInRuler &&gaps_loc_ruler.size() == 1) {
		hole.x = gaps_loc_ruler[0][0];
		hole.w = gaps_loc_ruler[0][1] - gaps_loc_ruler[0][0];
	}

	return true;
}

bool clearLongtailPt(const std::vector<cv::Point3f>& pts_plane, std::vector<cv::Point3f>& cleared_plane) {
	float miny = std::numeric_limits<float>::infinity();
	float maxy = -std::numeric_limits<float>::infinity();

	float meany = 0.0f;
	for (int i = 0; i < pts_plane.size(); i++) {
		meany += pts_plane[i].y;
		if (pts_plane[i].y > maxy) {
			maxy = pts_plane[i].y;
		}
		if (pts_plane[i].y < miny) {
			miny = pts_plane[i].y;
		}
	}
	meany = meany / pts_plane.size();
	cout << maxy - miny << " " << meany - miny << endl;
	if (abs(miny) < abs(maxy)) {
		for (int i = 0; i < pts_plane.size(); i++) {
			if (pts_plane[i].y < miny + (maxy - miny) * 0.4) {
				cleared_plane.push_back(pts_plane[i]);
			}
		}
	}
	else {
		for (int i = 0; i < pts_plane.size(); i++) {
			if (pts_plane[i].y > maxy - (maxy - miny) * 0.4) {
				cleared_plane.push_back(pts_plane[i]);
			}
		}
	}

	return true;
}

bool DetectionHoleClass::getRefTopY(const std::vector<cv::Point3f>& pts_plane, const std::vector<float> & corner, float & refY)
{
	//ExportPts("D:\\data\\plydata\\pts_plane.txt", pts_plane);
	float minz = (corner[2] > corner[3]) ? corner[3] : corner[2];
	float maxz = (corner[2] > corner[3]) ? corner[2] : corner[3];
	float maxx = (corner[0] > corner[1]) ? corner[0] : corner[1];
	float minx = (corner[0] > corner[1]) ? corner[1] : corner[0];
	vector <float> cornerplaneY;
	vector<cv::Point3f> pt_temp1;
	for (int i = 0; i < pts_plane.size(); i++) {
		if (pts_plane[i].z > minz && pts_plane[i].z< maxz && pts_plane[i].x > minx && pts_plane[i].x < maxx)
		{
			cornerplaneY.push_back(abs(pts_plane[i].y));
			pt_temp1.push_back(pts_plane[i]);
		}
	}
	refY = 0;
	//ExportPts("D:\\data\\plydata\\pt_temp1.txt", pt_temp1);
	if (cornerplaneY.size() < 50) {
		return false;
	}
	sort(cornerplaneY.begin(), cornerplaneY.end());

	int numtop = 10;
	for (int i = cornerplaneY.size() - 1; i > cornerplaneY.size() - numtop; --i) {
		//cout << cornerplaneY[i] << endl;
		refY += cornerplaneY[i];
	}

	refY = refY / numtop;
	//cout << refY << endl;
	return true;
}

bool DetectionHoleClass::sparseOrDenseAlongLineDirection(const std::vector<cv::Point3f>& pts_plane, const float setPrecent, const float refline) {
	int samplenum = 99;
	float onestep;
	float* minmax_xyz = new float[6];
	int sampleHasPt_num = 0;
	int validsample = 0;
	float minDist = 0;
	float minDist_real_more0 = 0;
	float minDist_real_less0 = 0;
	int sample_more0 = 0;
	int sample_less0 = 0;
	float minDist_real = 0;
	float avarage_dist = 0;
	float Width_ruler = 20.f;
	std::vector<unsigned int> ptID_rulers;
	std::pair<float, float> bbox_ruler_v = std::make_pair(
		refline - Width_ruler * 1.5,
		refline + Width_ruler * 1.5);
	AppendPt2Ruler1D(pts_plane, bbox_ruler_v, AxisType::Axis_Y, ptID_rulers);
	vector<cv::Point3f> pt_temp;
	std::vector<unsigned int> pt_sample;
	for (int j = 0; j < ptID_rulers.size(); j++) {
		pt_temp.push_back(pts_plane[ptID_rulers[j]]);
	}

	// ExportPts("D:\\data\\plydata\\pt_temp.txt", pt_temp);
	if (MathOperation::GetMinMaxXYZ(pt_temp, minmax_xyz)) {
		//ExportPts("D:\\data\\plydata\\pt_temp.txt", pt_temp);
		///cout << minmax_xyz[3] << " " << minmax_xyz[2] << endl;
		///cout << minmax_xyz[4] << " "<< minmax_xyz[5] << endl;
		if (minmax_xyz[5] - minmax_xyz[4] > 300) {
			// 100 分抽样点云
			for (int k = 1; k < samplenum + 1; k++) {
				onestep = minmax_xyz[4] + k * (minmax_xyz[5] - minmax_xyz[4]) / 100;

				AppendPt2Ruler1D(pt_temp, std::make_pair(onestep - Width_ruler * 2.5, onestep + Width_ruler * 2.5), AxisType::Axis_Z, pt_sample);
				// cout << "minmax_xyz[1] " << minmax_xyz[1] << "  " << minmax_xyz[0] << "onestep ::" << onestep << "pt_sample.size() :: " << pt_sample.size() << endl;
				// 找到离虚拟线最小的距离
				/*vector<cv::Point3f> pt_sample2;
				for (int j = 0; j < pt_sample.size(); j++) {
					pt_sample2.push_back(pt_temp[pt_sample[j]]);
				}
				ExportPts("D:\\data\\plydata\\pt_sample2.txt", pt_sample2);*/
				if (pt_sample.size() > 0) {
					sampleHasPt_num++;
					minDist = std::numeric_limits<float>::infinity();
					for (int m = 0; m < pt_sample.size(); m++) {
						if (minDist > abs(pt_temp[pt_sample[m]].y - refline)) {
							minDist = abs(pt_temp[pt_sample[m]].y - refline);
						}
					}

					if (minDist < Width_ruler / 6) {
						validsample++;
					}
				}
			}
		}
		else {
			return false;
		}
	}
	else {
		return false;
	}

	if (validsample < sampleHasPt_num * setPrecent) {
		return false;
	}
	else {
		return true;
	}
}
bool DetectionHoleClass::edgeFitting(const std::vector<cv::Point3f>& pts_plane, const cv::Point3f normal_plane, const float* bbox_plane, Bbox& hole) {
	std::vector<cv::Point3f> cleared_plane;
	//ExportPts("D:\\data\\plydata\\pts_plane.txt", pts_plane);

	float bottomz = std::numeric_limits<float>::infinity();
	float topz = -std::numeric_limits<float>::infinity();
	float leftx = pts_plane[0].x;
	float rightx = pts_plane[0].x;
	for (int i = 0; i < pts_plane.size(); i++) {
		if (pts_plane[i].z < bottomz) bottomz = pts_plane[i].z;
		if (pts_plane[i].z > topz) topz = pts_plane[i].z;
		if (pts_plane[i].x > rightx) rightx = pts_plane[i].x;
		if (pts_plane[i].x < leftx) leftx = pts_plane[i].x;
		cleared_plane.push_back(pts_plane[i]);
	}
	//ExportPts("D:\\data\\plydata\\cleared_plane.txt", cleared_plane);
	std::vector<array<float, 2>> gaps_loc_ruler;
	std::vector<std::vector<array<float, 2>>> gaps_loc_hole;
	int mask[4] = { 0,0,0,0 };
	unsigned int n_ruler = 4; // number of ruler
	std::vector<std::vector<unsigned int>> ptID_rulers;
	ptID_rulers.resize(n_ruler);
	// compute ruler bounds at v direction
	std::vector<std::pair<float, float>> bbox_ruler_v;
	bbox_ruler_v.resize(n_ruler);
	unsigned int count = 0;
	//float corner[4] = { hole.y, hole.y + hole.h, hole.x, hole.x + hole.w };
	// 0: 横最低，1 : 横最高，2: 竖最左，3 ：竖最右

	float frontY = 0;
	float center_v[4] = { hole.y, hole.y + hole.h,0,0 };
	for (int i = 0; i < pts_plane.size(); i++) {
		frontY += pts_plane[i].y;
	}
	if (frontY > 0) {
		center_v[2] = hole.x + hole.w;
		center_v[3] = hole.x;
		swap(leftx, rightx);
	}
	else {
		center_v[2] = hole.x;
		center_v[3] = hole.x + hole.w;
	}

	//float center_v[4] = { hole.y, hole.y + hole.h, max(hole.x,hole.x + hole.w), min(hole.x + hole.w,hole.x) };
	int iter_num = 0;
	int direction[4] = { 1,1,-1,1 };
	float Width_ruler = 20.f;
	int num_first[4] = { 0, 0, 0, 0 };
	while ((mask[0] == 0 || mask[1] == 0 || mask[2] == 0 || mask[3] == 0) && iter_num < 100) {
		//ExportPts("D:\\data\\plydata\\pts_plane.txt", pts_plane);
		for (int i = 0; i < 4; i++) {
			// 上下边
			iter_num = iter_num + 1;
			//cout << mask[0] << " " << mask[1] << " " << mask[2] << " " << mask[3] << "iter_num :: " << iter_num << endl;
			if (mask[0] == 0) {
				bbox_ruler_v[0] = std::make_pair(
					center_v[0] - Width_ruler * 1.5,
					center_v[0] + Width_ruler * 1.5);
				AppendPt2Ruler1D(cleared_plane, bbox_ruler_v[0], AxisType::Axis_Z, ptID_rulers[0]);
				if (iter_num == 1) {
					std::vector<unsigned int> ptID1, ptID2;
					AppendPt2Ruler1D(cleared_plane, std::make_pair(
						center_v[0],
						center_v[0] + Width_ruler * 3), AxisType::Axis_Z, ptID1);
					AppendPt2Ruler1D(cleared_plane, std::make_pair(
						center_v[0] - Width_ruler * 3,
						center_v[0]), AxisType::Axis_Z, ptID2);
					int ptid1_num = 0;
					int ptid2_num = 0;
					for (int m = 0; m < ptID1.size(); m++) {
						if (cleared_plane[ptID1[m]].x < center_v[3] + Width_ruler * 3 && cleared_plane[ptID1[m]].x > center_v[2] - Width_ruler * 3) {
							ptid1_num++;
						}
					}
					for (int m = 0; m < ptID2.size(); m++) {
						if (cleared_plane[ptID2[m]].x < center_v[3] + Width_ruler * 3 && cleared_plane[ptID2[m]].x > center_v[2] - Width_ruler * 3) {
							ptid2_num++;
						}
					}
					if (!(ptid1_num == 0 && ptid2_num == 0)) {
						if (ptid1_num > ptid2_num) {
							direction[0] = 1;
						}
						else {
							direction[0] = -1;
						}
					}
					ptID1.clear();
					ptID2.clear();
				}
			}
			if (mask[1] == 0) {
				bbox_ruler_v[1] = std::make_pair(
					center_v[1] - Width_ruler * 1.5,
					center_v[1] + Width_ruler * 1.5);
				AppendPt2Ruler1D(cleared_plane, bbox_ruler_v[1], AxisType::Axis_Z, ptID_rulers[1]);
			}
			// 左右边
			if (mask[2] == 0) {
				bbox_ruler_v[2] = std::make_pair(
					center_v[2] - Width_ruler * 1.5,
					center_v[2] + Width_ruler * 1.5);

				AppendPt2Ruler1D(cleared_plane, bbox_ruler_v[2], AxisType::Axis_X, ptID_rulers[2]);
				if (iter_num == 1) {
					std::vector<unsigned int> ptID1, ptID2;
					AppendPt2Ruler1D(cleared_plane, std::make_pair(
						center_v[2],
						center_v[2] + Width_ruler * 3), AxisType::Axis_X, ptID1);
					AppendPt2Ruler1D(cleared_plane, std::make_pair(
						center_v[2] - Width_ruler * 3,
						center_v[2]), AxisType::Axis_X, ptID2);
					int ptid1_num = 0;
					int ptid2_num = 0;
					for (int m = 0; m < ptID1.size(); m++) {
						if (cleared_plane[ptID1[m]].z < center_v[1] + Width_ruler * 3 && cleared_plane[ptID1[m]].z > center_v[0] - Width_ruler * 3) {
							ptid1_num++;
						}
					}
					for (int m = 0; m < ptID2.size(); m++) {
						if (cleared_plane[ptID2[m]].z < center_v[1] + Width_ruler * 3 && cleared_plane[ptID2[m]].z > center_v[0] - Width_ruler * 3) {
							ptid2_num++;
						}
					}
					//ExportPts("D:\\data\\plydata\\cleared_plane.txt", cleared_plane);
					//cout << center_v[0] << " " << center_v[1] << " " << center_v[2] <<" "<< center_v[3] << endl;
					//cout <<"ptid2_num : "<< ptid2_num <<" ptid1_num :" << ptid1_num << endl;
					/*vector<cv::Point3f> t;
					for (int i = 0; i < ptID1.size(); i++) {
						t.push_back(cleared_plane[ptID1[i]]);
					}
					ExportPts("D:\\data\\plydata\\ptID1.txt", t);
					t.clear();
					for (int i = 0; i < ptID2.size(); i++) {
						t.push_back(cleared_plane[ptID2[i]]);
					}

					ExportPts("D:\\data\\plydata\\ptID2.txt", t);
					t.clear();*/
					if (!(ptid1_num == 0 && ptid2_num == 0)) {
						if (ptid1_num > ptid2_num) {
							direction[2] = 1;
						}
						else {
							direction[2] = -1;
						}
					}
					ptID1.clear();
					ptID2.clear();
				}
			}
			if (mask[3] == 0) {
				bbox_ruler_v[3] = std::make_pair(
					center_v[3] - Width_ruler * 1.5,
					center_v[3] + Width_ruler * 1.5);
				AppendPt2Ruler1D(cleared_plane, bbox_ruler_v[3], AxisType::Axis_X, ptID_rulers[3]);
				if (iter_num == 1) {
					std::vector<unsigned int> ptID1, ptID2;
					AppendPt2Ruler1D(cleared_plane, std::make_pair(
						center_v[3],
						center_v[3] + Width_ruler * 3), AxisType::Axis_X, ptID1);
					AppendPt2Ruler1D(cleared_plane, std::make_pair(
						center_v[3] - Width_ruler * 3,
						center_v[3]), AxisType::Axis_X, ptID2);
					int ptid1_num = 0;
					int ptid2_num = 0;
					for (int m = 0; m < ptID1.size(); m++) {
						if (cleared_plane[ptID1[m]].z < center_v[1] + Width_ruler * 3 && cleared_plane[ptID1[m]].z > center_v[0] - Width_ruler * 3) {
							ptid1_num++;
						}
					}
					for (int m = 0; m < ptID2.size(); m++) {
						if (cleared_plane[ptID2[m]].z < center_v[1] + Width_ruler * 3 && cleared_plane[ptID2[m]].z > center_v[0] - Width_ruler * 3) {
							ptid2_num++;
						}
					}
					if (!(ptid1_num == 0 && ptid2_num == 0)) {
						if (ptid1_num > ptid2_num) {
							direction[3] = 1;
						}
						else {
							direction[3] = -1;
						}
					}
					ptID1.clear();
					ptID2.clear();
				}
			}
			if (mask[i] == 0) {
				vector<cv::Point3f> pt_temp;
				for (int j = 0; j < ptID_rulers[i].size(); j++) {
					pt_temp.push_back(cleared_plane[ptID_rulers[i][j]]);
				}
				float* minmax_xyz = new float[6];
				float onestep = 0;
				int validsample = 0;
				int samplenum = 99;
				float minDist = 0;
				float minDist_real_more0 = 0;
				float minDist_real_less0 = 0;
				int sample_more0 = 0;
				int sample_less0 = 0;
				float minDist_real = 0;
				float avarage_dist = 0;
				//ExportPts("D:\\data\\plydata\\pt_temp.txt", pt_temp);
				std::vector<unsigned int> pt_sample;
				int sampleHasPt_num = 0;
				if (i < 2) {
					if (MathOperation::GetMinMaxXYZ(pt_temp, minmax_xyz)) {
						//ExportPts("D:\\data\\plydata\\pt_temp.txt", pt_temp);
						if (minmax_xyz[1] - minmax_xyz[0] > 300) {
							for (int k = 1; k < samplenum + 1; k++) {
								onestep = minmax_xyz[0] + k * (minmax_xyz[1] - minmax_xyz[0]) / 100;

								AppendPt2Ruler1D(pt_temp, std::make_pair(onestep - Width_ruler * 2.5, onestep + Width_ruler * 2.5), AxisType::Axis_X, pt_sample);
								//  for show  //

								/*vector<cv::Point3f> pt_temp1;
								for (int jj = 0; jj < pt_sample.size(); jj++) {
									pt_temp1.push_back(pt_temp[pt_sample[jj]]);
								}

								ExportPts("D:\\data\\plydata\\pt_temp1.txt", pt_temp1);
								pt_temp1.clear();*/

								//cout << "minmax_xyz[1] " << minmax_xyz[1] << "  " << minmax_xyz[0] << "onestep ::" << onestep << "pt_sample.size() :: " << pt_sample.size() << endl;
								if (pt_sample.size() > 0) {
									sampleHasPt_num++;
									minDist = std::numeric_limits<float>::infinity();
									for (int m = 0; m < pt_sample.size(); m++) {
										if (minDist > abs(pt_temp[pt_sample[m]].z - center_v[i])) {
											minDist = abs(pt_temp[pt_sample[m]].z - center_v[i]);
											minDist_real = pt_temp[pt_sample[m]].z - center_v[i];
										}
									}
									if (minDist < Width_ruler / 4) {
										validsample++;
										avarage_dist = avarage_dist + minDist_real;
										if (minDist_real >= 0) {
											minDist_real_more0 += minDist_real;
											sample_more0++;
										}
										else {
											minDist_real_less0 += minDist_real;
											sample_less0++;
										}
									}
								}
								pt_sample.clear();
							}
						}
						else {
							validsample = 90;
							sampleHasPt_num = 99;
						}
						num_first[i]++;
					}

					//cout << " i :: " << i << " percent :: " << (double)validsample / (double)sampleHasPt_num << " avarage_dist : " << avarage_dist << " sample_more0 " << sample_more0<< " sample_less0 "<< sample_less0<<  endl;
					if (validsample == 0) {
						mask[i] = 1;
					}
					else if (validsample < sampleHasPt_num * 0.95) {
						center_v[i] = center_v[i] + direction[i] * Width_ruler / 6;
					}
					else {
						/*if (num_first[i] == 1) {
							center_v[i] = center_v[i] - direction[i] * Width_ruler / 4;
							continue;
						}*/

						if (avarage_dist >= 0) {
							if (sample_more0 != 0) {
								avarage_dist = minDist_real_more0 / sample_more0;
							}
							else {
								avarage_dist = 0;
							}
						}
						else {
							if (sample_less0 != 0) {
								avarage_dist = minDist_real_less0 / sample_less0;
							}
							else {
								avarage_dist = 0;
							}
						}
						center_v[i] = center_v[i] + avarage_dist;
						//cout << "avarage_dist : " << avarage_dist << endl;
						mask[i] = 1;
						/*if (abs(avarage_dist) < Width_ruler / 4) {
							mask[i] = 1;
						}*/
					}
				}

				if (i >= 2) {
					if (MathOperation::GetMinMaxXYZ(pt_temp, minmax_xyz)) {
						if (minmax_xyz[5] - minmax_xyz[4] > 300) {
							if (center_v[0] < bottomz + 10 * Width_ruler) {
								int midnum = 0;
								//vector<cv::Point3f> ot;
								for (int jj = 0; jj < pt_temp.size(); jj++) {
									if (pt_temp[jj].z < center_v[1] - Width_ruler * 30 && pt_temp[jj].z > center_v[0] + Width_ruler * 30)
									{
										midnum++;
										//ot.push_back(pt_temp[jj]);
									}
								}
								//ExportPts("D:\\data\\plydata\\pt_ot.txt", ot);
								if (midnum != 0) {
									if (minmax_xyz[5] > center_v[1] - Width_ruler * 30) minmax_xyz[5] = center_v[1] - Width_ruler * 30;
									if (minmax_xyz[4] < center_v[0] + Width_ruler * 30) minmax_xyz[4] = center_v[0] + Width_ruler * 30;
								}
							}

							for (int k = 1; k < samplenum + 1; k++) {
								onestep = minmax_xyz[4] + k * (minmax_xyz[5] - minmax_xyz[4]) / 100;
								AppendPt2Ruler1D(pt_temp, std::make_pair(onestep - (minmax_xyz[5] - minmax_xyz[4]) / 200, onestep + (minmax_xyz[5] - minmax_xyz[4]) / 200), AxisType::Axis_Z, pt_sample);

								//pt_temp1.clear();
								if (pt_sample.size() > 0) {
									sampleHasPt_num++;
									minDist = std::numeric_limits<float>::infinity();
									for (int m = 0; m < pt_sample.size(); m++) {
										if (minDist > abs(pt_temp[pt_sample[m]].x - center_v[i])) {
											minDist = abs(pt_temp[pt_sample[m]].x - center_v[i]);
											minDist_real = pt_temp[pt_sample[m]].x - center_v[i];
										}
									}
									if (minDist < Width_ruler / 4) {
										validsample++;
										avarage_dist = avarage_dist + minDist_real;
										if (minDist_real >= 0) {
											minDist_real_more0 += minDist_real;
											sample_more0++;
										}
										else {
											minDist_real_less0 += minDist_real;
											sample_less0++;
										}
									}
								}
								pt_sample.clear();
							}
						}
						else {
							validsample = 90;
							sampleHasPt_num = 99;
						}
						num_first[i]++;
					}

					//cout << " i :: " << i << " percent :: " << (double)validsample/ (double)sampleHasPt_num<< " "<<(double)validsample << " "<<  (double)sampleHasPt_num << " avarage_dist : " << avarage_dist <<" sample_more0 " << sample_more0 << " sample_less0 " << sample_less0 << endl;
					if (validsample == 0) {
						mask[i] = 1;
						num_first[i] = 1;
					}
					else if (validsample < sampleHasPt_num * 0.97) {
						center_v[i] = center_v[i] + direction[i] * Width_ruler / 4;
					}
					else {
						if (num_first[i] == 1) {
							center_v[i] = center_v[i] - direction[i] * Width_ruler / 2;

							continue;
						}
						if (avarage_dist >= 0) {
							if (sample_more0 != 0) {
								avarage_dist = minDist_real_more0 / sample_more0;
							}
							else {
								avarage_dist = 0;
							}
						}
						else {
							if (sample_less0 != 0) {
								avarage_dist = minDist_real_less0 / sample_less0;
							}
							else {
								avarage_dist = 0;
							}
						}
						center_v[i] = center_v[i] + avarage_dist;
						mask[i] = 1;
						//cout << "avarage_dist : " << avarage_dist << endl;
						/*if (abs(avarage_dist) < Width_ruler / 4) {
							mask[i] = 1;
						}*/
					}
				}
				pt_temp.clear();
			}
		}
	}

	//cout << num_first[2] << "  " <<num_first[3] << endl;

	float diffmaxY = 10;

	float refY = -1;
	std::vector<float> corner(4, 0.0f); // 左右下上
	for (int k = 0; k < 2; k++) {
		if (k == 0 && center_v[0] < bottomz + 10 * Width_ruler) {
			//cout << "this is a door " << endl;
			continue;
		}
		if (k == 0 && center_v[0] > bottomz + 10 * Width_ruler) {
			//cout << center_v[0] << " " << bbox_plane[0] << " "<< bottomz  << endl;
			//cout << "this is a window " << endl;
			direction[k] = -1;
		}
		//cout << "K : "<< k << " direction: " << direction[k] << endl;

		corner[0] = (center_v[2] + center_v[3]) / 2 - 3 * Width_ruler;
		corner[1] = (center_v[2] + center_v[3]) / 2 + 3 * Width_ruler;
		corner[2] = center_v[k] + 2 * direction[k] * Width_ruler;
		corner[3] = center_v[k] + 3 * direction[k] * Width_ruler;
		if (!getRefTopY(pts_plane, corner, refY)) {
			corner[0] = (center_v[2] + center_v[3]) / 2 - 3 * Width_ruler;
			corner[1] = (center_v[2] + center_v[3]) / 2 + 3 * Width_ruler;
			corner[2] = center_v[k] + 1.5 * direction[k] * Width_ruler;
			corner[3] = center_v[k] + 2.5 * direction[k] * Width_ruler;
			getRefTopY(pts_plane, corner, refY);
		}
		else {
			//cout << corner[0] <<" "<< corner[1] << " " << bbox_plane[0] << " " << bbox_plane[1] << endl;
			if (corner[1] > bbox_plane[1] - 3 * Width_ruler) {
				corner[0] = (center_v[2] + center_v[3]) / 2 - 3 * Width_ruler;
				corner[1] = (center_v[2] + center_v[3]) / 2 + 3 * Width_ruler;
				corner[2] = center_v[k] + 2 * direction[k] * Width_ruler;
				corner[3] = center_v[k] + 3 * direction[k] * Width_ruler;
				getRefTopY(pts_plane, corner, refY);
				//refY = refY - 10;
			}
		}

		if (refY > 0) {
			float difY = 20;
			float compY = 0;
			int num = 0;

			while (difY > diffmaxY && num < 4) {
				compY = 0;
				corner[0] = (center_v[2] + center_v[3]) / 2 - 3 * Width_ruler;
				corner[1] = (center_v[2] + center_v[3]) / 2 + 3 * Width_ruler;
				corner[2] = center_v[k];
				corner[3] = center_v[k] + direction[k] * Width_ruler;
				getRefTopY(pts_plane, corner, compY);
				difY = compY - refY;
				//cout << "K :" << k << " dif :" << difY << endl;
				if (difY > diffmaxY) {
					center_v[k] = center_v[k] + direction[k] * Width_ruler / 6;
				}
				num++;
			}
			if (num == 1) {
				if (k == 0 && center_v[0] > bottomz + 10 * Width_ruler) {
					if (difY > 5) {
						center_v[k] = center_v[k] + direction[k] * Width_ruler / 2;
					}
				}
			}
			if (num == 4) {
				if (k == 1 && center_v[0] < bottomz + 10 * Width_ruler) {
					if (difY > 10) {
						center_v[k] = center_v[k] - direction[k] * Width_ruler / 2;
					}
				}
			}
		}
	}

	diffmaxY = 10;
	for (int k = 2; k < 4; k++) {
		if (k == 2) {
			if ((center_v[2] - leftx) < Width_ruler * 10 && (center_v[3] - rightx) > Width_ruler * 10) {
				corner[0] = center_v[3] + 3 * direction[3] * Width_ruler;
				corner[1] = center_v[3] + 6 * direction[3] * Width_ruler;
				corner[2] = (center_v[1] + center_v[0]) / 2 - 3 * Width_ruler; // center_v[1] + 2 * Width_ruler;
				corner[3] = (center_v[1] + center_v[0]) / 2 + 3 * Width_ruler; //center_v[1] + 5 * Width_ruler;
			}
			else {
				corner[0] = center_v[k] + 3 * direction[k] * Width_ruler;
				corner[1] = center_v[k] + 6 * direction[k] * Width_ruler;
				corner[2] = (center_v[1] + center_v[0]) / 2 - 3 * Width_ruler; // center_v[1] + 2 * Width_ruler;
				corner[3] = (center_v[1] + center_v[0]) / 2 + 3 * Width_ruler; //center_v[1] + 5 * Width_ruler;
			}
		}
		if (k == 3) {
			if ((center_v[3] - rightx) < Width_ruler * 10 && (center_v[2] - leftx) > Width_ruler * 10) {
				corner[0] = center_v[2] + 3 * direction[2] * Width_ruler;
				corner[1] = center_v[2] + 6 * direction[2] * Width_ruler;
				corner[2] = (center_v[1] + center_v[0]) / 2 - 3 * Width_ruler; // center_v[1] + 2 * Width_ruler;
				corner[3] = (center_v[1] + center_v[0]) / 2 + 3 * Width_ruler; //center_v[1] + 5 * Width_ruler;
			}
			else {
				corner[0] = center_v[k] + 3 * direction[k] * Width_ruler;
				corner[1] = center_v[k] + 6 * direction[k] * Width_ruler;
				corner[2] = (center_v[1] + center_v[0]) / 2 - 3 * Width_ruler; // center_v[1] + 2 * Width_ruler;
				corner[3] = (center_v[1] + center_v[0]) / 2 + 3 * Width_ruler; //center_v[1] + 5 * Width_ruler;
			}
		}

		if (!getRefTopY(pts_plane, corner, refY)) {
			corner[0] = center_v[k] + 2 * direction[k] * Width_ruler;
			corner[1] = center_v[k] + 4 * direction[k] * Width_ruler;
			corner[2] = (center_v[1] + center_v[0]) / 2 - 3 * Width_ruler; // center_v[1] + 2 * Width_ruler;
			corner[3] = (center_v[1] + center_v[0]) / 2 + 3 * Width_ruler; //center_v[1] + 5 * Width_ruler;
			getRefTopY(pts_plane, corner, refY);
		}
		else {
			//cout << corner[0] <<" "<< corner[1] << " " << bbox_plane[0] << " " << bbox_plane[1] << endl;
			if ((corner[0] < bottomz + 3 * Width_ruler || corner[1] < bottomz + 3 * Width_ruler) || (corner[0] > topz - 3 * Width_ruler || corner[1] > topz - 3 * Width_ruler)) {
				corner[0] = center_v[k] + 2 * direction[k] * Width_ruler;
				corner[1] = center_v[k] + 4 * direction[k] * Width_ruler;
				corner[2] = (center_v[1] + center_v[0]) / 2 - 3 * Width_ruler; // center_v[1] + 2 * Width_ruler;
				corner[3] = (center_v[1] + center_v[0]) / 2 + 3 * Width_ruler;
				getRefTopY(pts_plane, corner, refY);
				//refY = refY - 10;
			}
		}

		vector<float> notedif;

		if (refY > 0) {
			float difY = 20;
			float compY = 0;
			int num = 0;
			if (center_v[0] < bottomz + 10 * Width_ruler && num_first[k] == 1) {
				center_v[k] = center_v[k] + direction[k] * Width_ruler / 2;
			}
			while (difY > diffmaxY && num < 4) {
				compY = 0;
				corner[0] = center_v[k];
				corner[1] = center_v[k] + 2 * direction[k] * Width_ruler;
				corner[2] = (center_v[1] + center_v[0]) / 2 - 3 * Width_ruler;
				corner[3] = (center_v[1] + center_v[0]) / 2 + 3 * Width_ruler;
				getRefTopY(pts_plane, corner, compY);
				difY = compY - refY;
				notedif.push_back(difY);
				//cout << "K :" << k << " dif :" << difY << endl;
				if (difY > diffmaxY) {
					center_v[k] = center_v[k] + direction[k] * Width_ruler / 6;
				}
				num++;
			}
			if (num == 1) {
				if (center_v[0] > bottomz + 10 * Width_ruler) {
					if (difY < diffmaxY) {
						center_v[k] = center_v[k] + direction[k] * Width_ruler / 2;
					}
				}
				else {
					if (difY > 5 && difY < diffmaxY)
					{
						corner[0] = center_v[k] + direction[k] * Width_ruler / 6;
						corner[1] = center_v[k] + direction[k] * Width_ruler / 2;
						corner[2] = (center_v[1] + center_v[0]) / 2 - 3 * Width_ruler;
						corner[3] = (center_v[1] + center_v[0]) / 2 + 3 * Width_ruler;
						getRefTopY(pts_plane, corner, compY);
						difY = compY - refY;
						if (difY < 3) {
							//cout << center_v[k] << " " << difY << "  " << compY << " " << refY << endl;
							center_v[k] = center_v[k] + direction[k] * Width_ruler / 10;
						}
						else {
							//cout << center_v[k] << " " << difY << "  " << compY << " " << refY << endl;
							center_v[k] = center_v[k] + direction[k] * Width_ruler / 2;
						}
					}
					else if (difY > 0 && ((k == 2 && abs(center_v[k] - leftx) < Width_ruler * 10) || (k == 3 && abs(center_v[k] - rightx) < Width_ruler * 10))) {
						center_v[k] = center_v[k] + direction[k] * Width_ruler / 2;
					}
				}
			}

			if (num == 4 && center_v[0] < bottomz + 10 * Width_ruler) {
				//cout << center_v[k]<< "  " << leftx << "  " << center_v[k] - leftx << endl;

				if (k == 2 && abs(center_v[k] - leftx) < Width_ruler * 10 && abs(center_v[k] - leftx) > Width_ruler * 3)
				{
					center_v[k] = center_v[k] - direction[k] * Width_ruler / 2;
					continue;
				}
				if (k == 3 && abs(center_v[k] - rightx) < Width_ruler * 10 && abs(center_v[k] - rightx) > Width_ruler * 3)
				{
					center_v[k] = center_v[k] - direction[k] * Width_ruler / 2;
					continue;
				}
				//if (abs(notedif[3] - notedif[0]) < 1) {
				//	//cout << notedif[3] << "  " << notedif[0] << endl;
				//	//cout << center_v[k] << endl;
				//	center_v[k] = center_v[k] + direction[k] * Width_ruler/2 ;
				//	//cout << center_v[k] << endl;
				//	continue;
				//}
			}
		}
		notedif.clear();
	}
	//cout << center_v[2] << " "<< center_v[3] << endl;
	cleared_plane.clear();
	cout << hole.x << " " << hole.w + hole.x << " " << hole.y << " " << hole.y + hole.h << " " << hole.w << " " << hole.h << endl;
	//cout << mask[0] << " " << mask[1] << " " << mask[2] << " " << mask[3] << endl;
	// 如果始终没找到，则保持原有不变
	if (mask[0] == 0) {
		center_v[0] = hole.y;
	}
	if (mask[1] == 0) {
		center_v[1] = hole.y + hole.h;
	}
	if (mask[2] == 0) {
		if (frontY > 0) {
			center_v[2] = hole.x + hole.w;
		}
		else {
			center_v[2] = hole.x;
		}
	}
	if (mask[3] == 0) {
		if (frontY > 0) {
			center_v[3] = hole.x;
		}
		else {
			center_v[3] = hole.x + hole.w;
		}
	}

	if (frontY > 0) {
		hole.x = center_v[3];
	}
	else {
		hole.x = center_v[2];
	}
	//hole.x = min(center_v[2],center_v[3]);
	hole.w = abs(center_v[2] - center_v[3]);
	hole.y = center_v[0];
	hole.h = abs(center_v[1] - center_v[0]);
	cout << hole.x << " " << hole.w + hole.x << " " << hole.y << " " << hole.y + hole.h << " " << hole.w << " " << hole.h << endl;
	return true;
}

void DetectionHoleClass::lineApproaching(const std::vector<cv::Point3f>& pts_plane, const float bottomz,
	const float topz, const float leftx, const float rightx, float* center_v, int* direction, int* num_first) {
	int mask[4] = { 0,0,0,0 };

	unsigned int n_ruler = 4; // number of ruler
	std::vector<std::vector<unsigned int>> ptID_rulers;
	ptID_rulers.resize(n_ruler);
	// compute ruler bounds at v direction
	std::vector<std::pair<float, float>> bbox_ruler_v;
	bbox_ruler_v.resize(n_ruler);
	unsigned int count = 0;
	//float corner[4] = { hole.y, hole.y + hole.h, hole.x, hole.x + hole.w };
	// 0: 横最低，1 : 横最高，2: 竖最左，3 ：竖最右
	// 保证center_v 的顺序，同上。

	int iter_num = 0;
	// 初始化虚拟线的移动方向

	// 设置gap的阙值
	float Width_ruler = 20.f;

	while ((mask[0] == 0 || mask[1] == 0 || mask[2] == 0 || mask[3] == 0) && iter_num < 100) {
		//ExportPts("D:\\data\\plydata\\pts_plane.txt", pts_plane);
		for (int i = 0; i < 4; i++) {
			// 确定上下边的移动方向 ，并截取虚拟线附件的点。
			iter_num = iter_num + 1;
			//cout << mask[0] << " " << mask[1] << " " << mask[2] << " " << mask[3] << "iter_num :: " << iter_num << endl;
			if (mask[0] == 0) {
				bbox_ruler_v[0] = std::make_pair(
					center_v[0] - Width_ruler * 1.5,
					center_v[0] + Width_ruler * 1.5);
				AppendPt2Ruler1D(pts_plane, bbox_ruler_v[0], AxisType::Axis_Z, ptID_rulers[0]);
				if (iter_num == 1) {
					std::vector<unsigned int> ptID1, ptID2;
					AppendPt2Ruler1D(pts_plane, std::make_pair(
						center_v[0],
						center_v[0] + Width_ruler * 3), AxisType::Axis_Z, ptID1);
					AppendPt2Ruler1D(pts_plane, std::make_pair(
						center_v[0] - Width_ruler * 3,
						center_v[0]), AxisType::Axis_Z, ptID2);
					int ptid1_num = 0;
					int ptid2_num = 0;
					for (int m = 0; m < ptID1.size(); m++) {
						if (pts_plane[ptID1[m]].x < center_v[3] + Width_ruler * 3 && pts_plane[ptID1[m]].x > center_v[2] - Width_ruler * 3) {
							ptid1_num++;
						}
					}
					for (int m = 0; m < ptID2.size(); m++) {
						if (pts_plane[ptID2[m]].x < center_v[3] + Width_ruler * 3 && pts_plane[ptID2[m]].x > center_v[2] - Width_ruler * 3) {
							ptid2_num++;
						}
					}
					if (!(ptid1_num == 0 && ptid2_num == 0)) {
						if (ptid1_num > ptid2_num) {
							direction[0] = 1;
						}
						else {
							direction[0] = -1;
						}
					}
					else {
						if (center_v[0] < bottomz + 10 * Width_ruler) {
							direction[0] = 1;
						}
						else {
							direction[0] = -1;
						}
					}
					// cout << ptid1_num << " " << ptid2_num << " " << direction[0] << endl;
					ptID1.clear();
					ptID2.clear();
				}
			}
			if (mask[1] == 0) {
				bbox_ruler_v[1] = std::make_pair(
					center_v[1] - Width_ruler * 1.5,
					center_v[1] + Width_ruler * 1.5);
				AppendPt2Ruler1D(pts_plane, bbox_ruler_v[1], AxisType::Axis_Z, ptID_rulers[1]);
			}
			// 确定左右边的移动方向
			if (mask[2] == 0) {
				bbox_ruler_v[2] = std::make_pair(
					center_v[2] - Width_ruler * 1.5,
					center_v[2] + Width_ruler * 1.5);

				AppendPt2Ruler1D(pts_plane, bbox_ruler_v[2], AxisType::Axis_X, ptID_rulers[2]);
				if (iter_num == 1) {
					std::vector<unsigned int> ptID1, ptID2;
					AppendPt2Ruler1D(pts_plane, std::make_pair(
						center_v[2],
						center_v[2] + Width_ruler * 3), AxisType::Axis_X, ptID1);
					AppendPt2Ruler1D(pts_plane, std::make_pair(
						center_v[2] - Width_ruler * 3,
						center_v[2]), AxisType::Axis_X, ptID2);
					int ptid1_num = 0;
					int ptid2_num = 0;
					for (int m = 0; m < ptID1.size(); m++) {
						if (pts_plane[ptID1[m]].z < center_v[1] + Width_ruler * 3 && pts_plane[ptID1[m]].z > center_v[0] - Width_ruler * 3) {
							ptid1_num++;
						}
					}
					for (int m = 0; m < ptID2.size(); m++) {
						if (pts_plane[ptID2[m]].z < center_v[1] + Width_ruler * 3 && pts_plane[ptID2[m]].z > center_v[0] - Width_ruler * 3) {
							ptid2_num++;
						}
					}
					//ExportPts("D:\\data\\plydata\\cleared_plane.txt", cleared_plane);
					//cout << center_v[0] << " " << center_v[1] << " " << center_v[2] <<" "<< center_v[3] << endl;
					//cout <<"ptid2_num : "<< ptid2_num <<" ptid1_num :" << ptid1_num << endl;
					/*vector<cv::Point3f> t;
					for (int i = 0; i < ptID1.size(); i++) {
						t.push_back(cleared_plane[ptID1[i]]);
					}
					ExportPts("D:\\data\\plydata\\ptID1.txt", t);
					t.clear();
					for (int i = 0; i < ptID2.size(); i++) {
						t.push_back(cleared_plane[ptID2[i]]);
					}

					ExportPts("D:\\data\\plydata\\ptID2.txt", t);
					t.clear();*/
					if (!(ptid1_num == 0 && ptid2_num == 0)) {
						if (ptid1_num > ptid2_num) {
							direction[2] = 1;
						}
						else {
							direction[2] = -1;
						}
					}
					ptID1.clear();
					ptID2.clear();
				}
			}
			if (mask[3] == 0) {
				bbox_ruler_v[3] = std::make_pair(
					center_v[3] - Width_ruler * 1.5,
					center_v[3] + Width_ruler * 1.5);
				AppendPt2Ruler1D(pts_plane, bbox_ruler_v[3], AxisType::Axis_X, ptID_rulers[3]);
				if (iter_num == 1) {
					std::vector<unsigned int> ptID1, ptID2;
					AppendPt2Ruler1D(pts_plane, std::make_pair(
						center_v[3],
						center_v[3] + Width_ruler * 3), AxisType::Axis_X, ptID1);
					AppendPt2Ruler1D(pts_plane, std::make_pair(
						center_v[3] - Width_ruler * 3,
						center_v[3]), AxisType::Axis_X, ptID2);
					int ptid1_num = 0;
					int ptid2_num = 0;
					for (int m = 0; m < ptID1.size(); m++) {
						if (pts_plane[ptID1[m]].z < center_v[1] + Width_ruler * 3 && pts_plane[ptID1[m]].z > center_v[0] - Width_ruler * 3) {
							ptid1_num++;
						}
					}
					for (int m = 0; m < ptID2.size(); m++) {
						if (pts_plane[ptID2[m]].z < center_v[1] + Width_ruler * 3 && pts_plane[ptID2[m]].z > center_v[0] - Width_ruler * 3) {
							ptid2_num++;
						}
					}
					if (!(ptid1_num == 0 && ptid2_num == 0)) {
						if (ptid1_num > ptid2_num) {
							direction[3] = 1;
						}
						else {
							direction[3] = -1;
						}
					}

					ptID1.clear();
					ptID2.clear();
				}
			}
			// 只有标记为1，代表线逼近结束。
			if (mask[i] == 0) {
				vector<cv::Point3f> pt_temp;
				for (int j = 0; j < ptID_rulers[i].size(); j++) {
					pt_temp.push_back(pts_plane[ptID_rulers[i][j]]);
				}
				float* minmax_xyz = new float[6];
				float onestep = 0;
				int validsample = 0;
				int samplenum = 99;
				float minDist = 0;
				float minDist_real_more0 = 0;
				float minDist_real_less0 = 0;
				int sample_more0 = 0;
				int sample_less0 = 0;
				float minDist_real = 0;
				float avarage_dist = 0;
				//ExportPts("D:\\data\\plydata\\pt_temp.txt", pt_temp);
				std::vector<unsigned int> pt_sample;
				int sampleHasPt_num = 0;
				// 小于2 代表 上下边
				if (i < 2) {
					// 找到截取的点云的bounding box
					if (MathOperation::GetMinMaxXYZ(pt_temp, minmax_xyz)) {
						//ExportPts("D:\\data\\plydata\\pt_temp.txt", pt_temp);
						//cout << minmax_xyz[1] - minmax_xyz[0] << endl;
						if (minmax_xyz[1] - minmax_xyz[0] > 300) {
							// 100 分抽样点云
							for (int k = 1; k < samplenum + 1; k++) {
								onestep = minmax_xyz[0] + k * (minmax_xyz[1] - minmax_xyz[0]) / 100;

								AppendPt2Ruler1D(pt_temp, std::make_pair(onestep - Width_ruler * 2.5, onestep + Width_ruler * 2.5), AxisType::Axis_X, pt_sample);
								//  for show  //

								/*vector<cv::Point3f> pt_temp1;
								for (int jj = 0; jj < pt_sample.size(); jj++) {
									pt_temp1.push_back(pt_temp[pt_sample[jj]]);
								}

								ExportPts("D:\\data\\plydata\\pt_temp1.txt", pt_temp1);
								pt_temp1.clear();*/

								// cout << "minmax_xyz[1] " << minmax_xyz[1] << "  " << minmax_xyz[0] << "onestep ::" << onestep << "pt_sample.size() :: " << pt_sample.size() << endl;
								// 找到离虚拟线最小的距离
								if (pt_sample.size() > 0) {
									sampleHasPt_num++;
									minDist = std::numeric_limits<float>::infinity();
									for (int m = 0; m < pt_sample.size(); m++) {
										if (minDist > abs(pt_temp[pt_sample[m]].z - center_v[i])) {
											minDist = abs(pt_temp[pt_sample[m]].z - center_v[i]);
											minDist_real = pt_temp[pt_sample[m]].z - center_v[i];
										}
									}
									// 记录正向距离累积和负向距离累计
									if (minDist < Width_ruler / 4) {
										validsample++;
										avarage_dist = avarage_dist + minDist_real;
										if (minDist_real >= 0) {
											minDist_real_more0 += minDist_real;
											sample_more0++;
										}
										else {
											minDist_real_less0 += minDist_real;
											sample_less0++;
										}
									}
								}

								pt_sample.clear();
							}
						}
						else {
							validsample = 90;
							sampleHasPt_num = 99;
						}
						// 记录同一个方向查找了多少次
						num_first[i]++;
					}

					// cout << " i :: " << i << " " << validsample <<" "<< sampleHasPt_num <<  " percent :: " << (double)validsample / (double)sampleHasPt_num << " avarage_dist : " << avarage_dist << " sample_more0 " << sample_more0 << " sample_less0 " << sample_less0 << endl;
					// 如果没有找到，就不找了
					if (validsample == 0 || num_first[i] > 4) {
						mask[i] = 1;
					}
					else if (validsample < sampleHasPt_num * 0.94) { // 小于0.95比例的需要迭代
						center_v[i] = center_v[i] + direction[i] * Width_ruler / 6;
					}
					else {// 大于0.95比例的补偿后结束
						if (avarage_dist >= 0) {
							if (sample_more0 != 0) {
								avarage_dist = minDist_real_more0 / sample_more0;
							}
							else {
								avarage_dist = 0;
							}
						}
						else {
							if (sample_less0 != 0) {
								avarage_dist = minDist_real_less0 / sample_less0;
							}
							else {
								avarage_dist = 0;
							}
						}
						//cout << i  <<" "<<"avarage_dist : " << avarage_dist <<" "<< center_v[1]<< endl;
						center_v[i] = center_v[i] + avarage_dist;
						//cout << "center_v[1] : " << center_v[1] << endl;
						mask[i] = 1;
					}
				}
				// 大于2 代表左右边
				if (i >= 2) {
					// 找到截取的点云的bounding box
					if (MathOperation::GetMinMaxXYZ(pt_temp, minmax_xyz)) {
						if (minmax_xyz[5] - minmax_xyz[4] > 300) {
							if (center_v[0] < bottomz + 10 * Width_ruler) {
								// 截取中间段的点云
								int midnum = 0;
								//vector<cv::Point3f> ot;
								for (int jj = 0; jj < pt_temp.size(); jj++) {
									if (pt_temp[jj].z < center_v[1] - Width_ruler * 30 && pt_temp[jj].z > center_v[0] + Width_ruler * 30)
									{
										midnum++;
										//ot.push_back(pt_temp[jj]);
									}
								}
								//ExportPts("D:\\data\\plydata\\pt_ot.txt", ot);

								if (midnum != 0) {
									if (minmax_xyz[5] > center_v[1] - Width_ruler * 30) minmax_xyz[5] = center_v[1] - Width_ruler * 30;
									if (minmax_xyz[4] < center_v[0] + Width_ruler * 30) minmax_xyz[4] = center_v[0] + Width_ruler * 30;
								}
							}
							// 100 次点云采样 ，选择每次最近的点，计算累计误差。
							for (int k = 1; k < samplenum + 1; k++) {
								onestep = minmax_xyz[4] + k * (minmax_xyz[5] - minmax_xyz[4]) / 100;
								AppendPt2Ruler1D(pt_temp, std::make_pair(onestep - (minmax_xyz[5] - minmax_xyz[4]) / 200, onestep + (minmax_xyz[5] - minmax_xyz[4]) / 200), AxisType::Axis_Z, pt_sample);

								//pt_temp1.clear();
								if (pt_sample.size() > 0) {
									sampleHasPt_num++;
									minDist = std::numeric_limits<float>::infinity();
									for (int m = 0; m < pt_sample.size(); m++) {
										if (minDist > abs(pt_temp[pt_sample[m]].x - center_v[i])) {
											minDist = abs(pt_temp[pt_sample[m]].x - center_v[i]);
											minDist_real = pt_temp[pt_sample[m]].x - center_v[i];
										}
									}
									if (minDist < Width_ruler / 4) { //
										validsample++;
										avarage_dist = avarage_dist + minDist_real;
										if (minDist_real >= 0) {
											minDist_real_more0 += minDist_real;
											sample_more0++;
										}
										else {
											minDist_real_less0 += minDist_real;
											sample_less0++;
										}
									}
								}
								pt_sample.clear();
							}
						}
						else {
							validsample = 90;
							sampleHasPt_num = 99;
						}
						num_first[i]++;
					}
					if (num_first[i] > 4) {
						mask[i] = 1;
						continue;
					}
					// cout << " i :: " << i << " percent :: " << (double)validsample / (double)sampleHasPt_num << " " << (double)validsample << " " << (double)sampleHasPt_num << " avarage_dist : " << avarage_dist << " sample_more0 " << sample_more0 << " sample_less0 " << sample_less0 << endl;
					if (validsample == 0) {
						//continue;
						mask[i] = 1;
						cout << i << ": this is a problem" << endl;
						num_first[i] = 1;
					}
					else if (validsample < sampleHasPt_num * 0.93) {
						center_v[i] = center_v[i] + direction[i] * Width_ruler / 4;
					}
					else {
						if (avarage_dist >= 0) {
							if (sample_more0 != 0) {
								avarage_dist = minDist_real_more0 / sample_more0;
							}
							else {
								avarage_dist = 0;
							}
						}
						else {
							if (sample_less0 != 0) {
								avarage_dist = minDist_real_less0 / sample_less0;
							}
							else {
								avarage_dist = 0;
							}
						}
						center_v[i] = center_v[i] + avarage_dist;
						mask[i] = 1;
					}
				}
				pt_temp.clear();
			}
		}
	}
}

void DetectionHoleClass::adjustHoleY(const std::vector<cv::Point3f>& pts_plane, const float bottomz, const float topz, const float leftx,
	const float rightx, float* center_v, int* direction) {
	float diffmaxY = 10;
	float Width_ruler = 20;
	float refY = -1;
	std::vector<float> corner(4, 0.0f); // 左右下上
	for (int k = 0; k < 2; k++) {
		// 如果是门且是下边缘，跳过
		if (k == 0 && center_v[0] < bottomz + 10 * Width_ruler) {
			//cout << "this is a door " << endl;
			continue;
		}
		// 如果是窗， 下边缘朝向向下。
		if (k == 0 && center_v[0] > bottomz + 10 * Width_ruler) {
			//cout << center_v[0] << " " << bbox_plane[0] << " "<< bottomz  << endl;
			//cout << "this is a window " << endl;
			direction[k] = -1;
		}
		//cout << "K : "<< k << " direction: " << direction[k] << endl;

		// 附近采样 得到参考值
		corner[0] = (center_v[2] + center_v[3]) / 2 - 3 * Width_ruler;
		corner[1] = (center_v[2] + center_v[3]) / 2 + 3 * Width_ruler;
		corner[2] = center_v[k] + 2 * direction[k] * Width_ruler;
		corner[3] = center_v[k] + 3 * direction[k] * Width_ruler;
		if (!getRefTopY(pts_plane, corner, refY)) {
			corner[0] = (center_v[2] + center_v[3]) / 2 - 3 * Width_ruler;
			corner[1] = (center_v[2] + center_v[3]) / 2 + 3 * Width_ruler;
			corner[2] = center_v[k] + 1.5 * direction[k] * Width_ruler;
			corner[3] = center_v[k] + 2.5 * direction[k] * Width_ruler;
			getRefTopY(pts_plane, corner, refY);
		}
		else {
			//cout << corner[0] <<" "<< corner[1] << " " << bbox_plane[0] << " " << bbox_plane[1] << endl;
			if (corner[1] > rightx - 3 * Width_ruler) {
				corner[0] = (center_v[2] + center_v[3]) / 2 - 3 * Width_ruler;
				corner[1] = (center_v[2] + center_v[3]) / 2 + 3 * Width_ruler;
				corner[2] = center_v[k] + 2 * direction[k] * Width_ruler;
				corner[3] = center_v[k] + 3 * direction[k] * Width_ruler;
				getRefTopY(pts_plane, corner, refY);
				//refY = refY - 10;
			}
		}

		if (refY > 0) {
			float difY = 20;
			float compY = 0;
			int num = 0;

			while (difY > diffmaxY && num < 4) {
				// 计算当前值，计算与参考值的差值
				compY = 0;
				corner[0] = (center_v[2] + center_v[3]) / 2 - 3 * Width_ruler;
				corner[1] = (center_v[2] + center_v[3]) / 2 + 3 * Width_ruler;
				corner[2] = center_v[k];
				corner[3] = center_v[k] + direction[k] * Width_ruler;
				getRefTopY(pts_plane, corner, compY);
				difY = compY - refY;
				cout << "K :" << k << " dif :" << difY << endl;
				if (difY > diffmaxY) {
					center_v[k] = center_v[k] + direction[k] * Width_ruler / 6;
				}
				num++;
			}
			//if (num == 1) {
			//	// 如果 是窗，下边缘只移动一次，且差距还大于5，则向下补偿固定步长10 .
			//	if (k == 0 && center_v[0] > bottomz + 10 * Width_ruler) {
			//		if (difY > 5) {
			//			center_v[k] = center_v[k] + direction[k] * Width_ruler / 2;
			//		}

			//	}
			//}
			if (num == 4) {
				// 如果是门， 下边缘连续移动的4次，差距还大于10，则向下补偿固定补偿10 .
				if (k == 1 && center_v[0] < bottomz + 10 * Width_ruler) {
					if (difY > 10) {
						center_v[k] = center_v[k] - direction[k] * Width_ruler / 2;
					}
				}
			}
		}
	}
}

void DetectionHoleClass::adjustHoleX(const std::vector<cv::Point3f>& pts_plane, const float bottomz, const float topz, const float leftx,
	const float rightx, float* center_v, int* direction, const int* num_first) {
	float diffmaxY = 10;
	float Width_ruler = 20;
	float refY = -1;
	std::vector<float> corner(4, 0.0f); // 左右下上
	// 左右边寻找gap
	for (int k = 2; k < 4; k++) {
		if (k == 2) {
			// 如果左边或者右边靠近墙边缘，选择另一侧的作为参考值
			if ((center_v[2] - leftx) < Width_ruler * 10 && (center_v[3] - rightx) > Width_ruler * 10) {
				corner[0] = center_v[3] + 3 * direction[3] * Width_ruler;
				corner[1] = center_v[3] + 6 * direction[3] * Width_ruler;
				corner[2] = (center_v[1] + center_v[0]) / 2 - 3 * Width_ruler; // center_v[1] + 2 * Width_ruler;
				corner[3] = (center_v[1] + center_v[0]) / 2 + 3 * Width_ruler; //center_v[1] + 5 * Width_ruler;
			}
			else {
				corner[0] = center_v[k] + 3 * direction[k] * Width_ruler;
				corner[1] = center_v[k] + 6 * direction[k] * Width_ruler;
				corner[2] = (center_v[1] + center_v[0]) / 2 - 3 * Width_ruler; // center_v[1] + 2 * Width_ruler;
				corner[3] = (center_v[1] + center_v[0]) / 2 + 3 * Width_ruler; //center_v[1] + 5 * Width_ruler;
			}
		}
		if (k == 3) {
			if ((center_v[3] - rightx) < Width_ruler * 10 && (center_v[2] - leftx) > Width_ruler * 10) {
				corner[0] = center_v[2] + 3 * direction[2] * Width_ruler;
				corner[1] = center_v[2] + 6 * direction[2] * Width_ruler;
				corner[2] = (center_v[1] + center_v[0]) / 2 - 3 * Width_ruler; // center_v[1] + 2 * Width_ruler;
				corner[3] = (center_v[1] + center_v[0]) / 2 + 3 * Width_ruler; //center_v[1] + 5 * Width_ruler;
			}
			else {
				corner[0] = center_v[k] + 3 * direction[k] * Width_ruler;
				corner[1] = center_v[k] + 6 * direction[k] * Width_ruler;
				corner[2] = (center_v[1] + center_v[0]) / 2 - 3 * Width_ruler; // center_v[1] + 2 * Width_ruler;
				corner[3] = (center_v[1] + center_v[0]) / 2 + 3 * Width_ruler; //center_v[1] + 5 * Width_ruler;
			}
		}

		// 计算参考值
		if (!getRefTopY(pts_plane, corner, refY)) {
			corner[0] = center_v[k] + 2 * direction[k] * Width_ruler;
			corner[1] = center_v[k] + 4 * direction[k] * Width_ruler;
			corner[2] = (center_v[1] + center_v[0]) / 2 - 3 * Width_ruler; // center_v[1] + 2 * Width_ruler;
			corner[3] = (center_v[1] + center_v[0]) / 2 + 3 * Width_ruler; //center_v[1] + 5 * Width_ruler;
			getRefTopY(pts_plane, corner, refY);
		}
		else {
			//cout << corner[0] <<" "<< corner[1] << " " << bbox_plane[0] << " " << bbox_plane[1] << endl;
			if ((corner[0] < bottomz + 3 * Width_ruler || corner[1] < bottomz + 3 * Width_ruler) || (corner[0] > topz - 3 * Width_ruler || corner[1] > topz - 3 * Width_ruler)) {
				corner[0] = center_v[k] + 2 * direction[k] * Width_ruler;
				corner[1] = center_v[k] + 4 * direction[k] * Width_ruler;
				corner[2] = (center_v[1] + center_v[0]) / 2 - 3 * Width_ruler; // center_v[1] + 2 * Width_ruler;
				corner[3] = (center_v[1] + center_v[0]) / 2 + 3 * Width_ruler;
				getRefTopY(pts_plane, corner, refY);
				//refY = refY - 10;
			}
		}

		vector<float> notedif;

		if (refY > 0) {
			float difY = 20;
			float compY = 0;
			int num = 0;
			//// 如果是门，且只有一次逼近线，补偿步长10
			//if (center_v[0] < bottomz + 10 * Width_ruler && num_first[k] == 1) {
			//	center_v[k] = center_v[k] + direction[k] * Width_ruler / 2;
			//}
			// 迭代查找gap
			while (difY > diffmaxY && num < 4) {
				compY = 0;
				corner[0] = center_v[k];
				corner[1] = center_v[k] + 2 * direction[k] * Width_ruler;
				corner[2] = (center_v[1] + center_v[0]) / 2 - 3 * Width_ruler;
				corner[3] = (center_v[1] + center_v[0]) / 2 + 3 * Width_ruler;
				getRefTopY(pts_plane, corner, compY);
				difY = compY - refY;
				notedif.push_back(difY);
				// cout << "K :" << k << " dif :" << difY << endl;
				if (difY > diffmaxY) {
					center_v[k] = center_v[k] + direction[k] * Width_ruler / 6;
				}
				num++;
			}

			if (num == 1) {
				// 如果是窗，找一次就找到gap ，补偿10
				/*if (center_v[0] > bottomz + 10 * Width_ruler) {
					if (difY < diffmaxY) {
						center_v[k] = center_v[k] + direction[k] * Width_ruler / 2;
					}
				}
				else*/ {
				//if (difY > 0 && ((k == 2 && abs(center_v[k] - leftx) < Width_ruler * 10) || (k == 3 && abs(center_v[k] - rightx) < Width_ruler * 10))) {
				//	// 如果靠边，且一次就找到gap,补偿10
				//	center_v[k] = center_v[k] + direction[k] * Width_ruler / 2;
				//	continue;
				//}
				// 如果门一次就找gap,但差距大于5 ，则在计算一次，如果还大于3，则补偿5，小于3，补偿2.
				//if (difY > 5 && difY < diffmaxY)
				//{
				//	corner[0] = center_v[k] + direction[k] * Width_ruler / 6;
				//	corner[1] = center_v[k] + direction[k] * Width_ruler / 2;
				//	corner[2] = (center_v[1] + center_v[0]) / 2 - 3 * Width_ruler;
				//	corner[3] = (center_v[1] + center_v[0]) / 2 + 3 * Width_ruler;
				//	getRefTopY(pts_plane, corner, compY);
				//	difY = compY - refY;
				//	if (difY < 3) {
				//		//cout << center_v[k] << " " << difY << "  " << compY << " " << refY << endl;
				//		center_v[k] = center_v[k] + direction[k] * Width_ruler / 10;
				//	}
				//	else {
				//		//cout << center_v[k] << " " << difY << "  " << compY << " " << refY << endl;
				//		center_v[k] = center_v[k] + direction[k] * Width_ruler / 2;
				//	}

				//}
				}
			}

			if (num == 4 && center_v[0] < bottomz + 10 * Width_ruler) {
				//cout << center_v[k]<< "  " << leftx << "  " << center_v[k] - leftx << endl;
				// 如果门洞且靠边，移动超过4次，减小补偿10.
				if (k == 2 && abs(center_v[k] - leftx) < Width_ruler * 10 && abs(center_v[k] - leftx) > Width_ruler * 3)
				{
					if (notedif[3] > 15) {
						continue;
					}
					else {
						center_v[k] = center_v[k] - direction[k] * Width_ruler / 2;
						continue;
					}
				}
				if (k == 3 && abs(center_v[k] - rightx) < Width_ruler * 10 && abs(center_v[k] - rightx) > Width_ruler * 3)
				{
					if (notedif[3] > 15) {
						continue;
					}
					else {
						center_v[k] = center_v[k] - direction[k] * Width_ruler / 2;
						continue;
					}
				}
			}
		}
		notedif.clear();
	}
}

void frequencemaxnum(const vector<float>& input, float & maxfreq) {
	map<int, int> statis;
	for (int i = 0; i < input.size(); i++) {
		int v = int(input[i]);
		statis[v]++;
	}
	int tmp = 0;
	int id = 0;
	for (auto it = statis.begin(); it != statis.end(); it++) {
		cout << it->second << " " << it->first << endl;
		if (it->second > tmp) {
			tmp = it->second;
			id = it->first;
		}
	}
	maxfreq = static_cast<float>(id);
}
bool ComputeNormal(const std::vector<cv::Point3f>& pts_plane, double * plane_normals) {
	int num_of_pt = pts_plane.size();
	//if (!this->is_occupied || num_of_pt < 4) return;
	if (num_of_pt < 4) return false;
	double point_num_inverse = (double)1.0 / num_of_pt;

	double sum_x = 0;
	double sum_y = 0;
	double sum_z = 0;
	double sum_xx = 0;
	double sum_yy = 0;
	double  sum_zz = 0;
	double  sum_xy = 0;
	double  sum_xz = 0;
	double  sum_yz = 0;
	float plane_center[3] = { 0,0,0 };

	for (int i = 0; i < num_of_pt; i++) {
		sum_x += pts_plane[i].x;
		sum_y += pts_plane[i].y;
		sum_z += pts_plane[i].z;
		sum_xx += pts_plane[i].x * pts_plane[i].x;
		sum_yy += pts_plane[i].y * pts_plane[i].y;
		sum_zz += pts_plane[i].z * pts_plane[i].z;
		sum_xy += pts_plane[i].x * pts_plane[i].y;
		sum_xz += pts_plane[i].x * pts_plane[i].z;
		sum_yz += pts_plane[i].y * pts_plane[i].z;
	}

	//plane center
	plane_center[0] = sum_x * point_num_inverse;
	plane_center[1] = sum_y * point_num_inverse;
	plane_center[2] = sum_z * point_num_inverse;

	//calculate covariance matrix
	double cov_mat_element[6] = { sum_xx - sum_x * plane_center[0],sum_xy - sum_x * plane_center[1],
									sum_xz - sum_x * plane_center[2],sum_yy - sum_y * plane_center[1],
									sum_yz - sum_y * plane_center[2],sum_zz - sum_z * plane_center[2] };

	//double cov_mat_element2[6];
	//cov_mat_element2[0] = (sum_xx - 2 * plane_center[0] * sum_x)*point_num_inverse + std::pow(plane_center[0], 2);
	//std::cout << cov_mat_element2[0] << std::endl;

	double cov_mat_arr[3][3] = { { cov_mat_element[0],cov_mat_element[1],cov_mat_element[2] },
	{ cov_mat_element[1], cov_mat_element[3], cov_mat_element[4] },
	{ cov_mat_element[2], cov_mat_element[4], cov_mat_element[5] } };

	cv::Mat cov_mat(3, 3, CV_64F, cov_mat_arr);
	cov_mat *= point_num_inverse;

	//std::cout << cov_mat << std::endl;

	cv::Mat eig_val_mat, eig_vec_mat;

	cv::eigen(cov_mat, eig_val_mat, eig_vec_mat);

	//std::cout << eig_val_mat << std::endl;
	//std::cout << eig_vec_mat << std::endl;
	for (int i = 0; i < 3; i++)
		plane_normals[i] = eig_vec_mat.at<double>(2, i);

	return true;
}
bool isDensityOfWall(const int pts_plane_size, const float* corners_plane, const float setvalue) {
	float area_x = abs(corners_plane[0] - corners_plane[1]);
	float area_y = abs(corners_plane[2] - corners_plane[3]);
	float area_z = abs(corners_plane[4] - corners_plane[5]);
	float volume = area_x * area_y * area_z;

	if (volume == 0) return false;
	cout << "area : " << volume << " density : " << pts_plane_size / volume << endl;
	if (1.0 / volume * pts_plane_size > setvalue) {
		return true;
	}
	else {
		return false;
	}
}

void DetectionHoleClass::updateLeftRightInitPos(const std::vector<cv::Point3f>& pts_plane, const float rightx, const float leftx, const bool frontY, float *center_v) {
	std::vector<array<float, 2>> gaps_loc_ruler;
	std::vector<std::vector<array<float, 2>>> gaps_loc_hole;
	float center_z = (center_v[0] + center_v[1])* 0.5f;
	unsigned int n_ruler = 1; // number of ruler
	std::vector<std::vector<unsigned int>> ptID_rulers;
	ptID_rulers.resize(n_ruler);
	std::pair<float, float> bbox_u;
	//parallel to x-axis
	if (frontY) {
		bbox_u.first = max(center_v[3] - 100, rightx);
		bbox_u.second = min(center_v[2] + 100, leftx);
	}
	else {
		bbox_u.first = max(center_v[2] - 100, leftx);
		bbox_u.second = min(center_v[3] + 100, rightx);
	}

	vector<cv::Point3f> partPts;
	//cout << " " << bbox_u.first <<" "<< bbox_u.second << endl;
	for (int i = 0; i < pts_plane.size(); i++) {
		if (pts_plane[i].x > bbox_u.first && pts_plane[i].x < bbox_u.second)
			if (pts_plane[i].z > center_z - 100 && pts_plane[i].z < center_z + 100) {
				partPts.push_back(pts_plane[i]);
			}
	}
	if (partPts.size() > 0) {
		//cout << "has points " << endl;
		//ExportPts("D:\\data\\plydata\\partPts.txt", partPts);
		cv::Mat vals_ruler(static_cast<int>(partPts.size()), 1, CV_32F);
		cv::Mat sorted_ptID_ruler(static_cast<int>(partPts.size()), 1, CV_32S);
		for (int i = 0; i < partPts.size(); i++) {
			vals_ruler.at<float>(i, 0) = partPts[i].x;
		}

		cv::sortIdx(vals_ruler, sorted_ptID_ruler, CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);
		bool isGapInRuler = FindGapsInRuler(vals_ruler, sorted_ptID_ruler, bbox_u, gaps_loc_ruler);
		if (isGapInRuler && gaps_loc_ruler.size() == 1) {
			//cout << frontY <<" "<< center_v[2] << " " << center_v[3] << " " << leftx << " " << rightx << " " << bbox_u.first << " " << bbox_u.second << endl;
			if (frontY) {
				center_v[2] = gaps_loc_ruler[0][1];
				center_v[3] = gaps_loc_ruler[0][0];
			}
			else {
				center_v[2] = gaps_loc_ruler[0][0];
				center_v[3] = gaps_loc_ruler[0][1];
			}
		}
	}
}

void denoiseAroundHole(const std::vector<cv::Point3f>& pts_plane, const std::vector<int>& reflect_plane, const array<float, 2>& raw_loc_w, const array<float, 2>& raw_loc_h, const std::vector<double>& planeMeanStd, const float leftx, const float rightx, std::vector<cv::Point3f>& pts_plane_copy) {
	for (int i = 0; i < pts_plane.size(); i++) {
		if (((pts_plane[i].x > min(raw_loc_w[0], raw_loc_w[1]) - 100 && pts_plane[i].x < max(raw_loc_w[0], raw_loc_w[1]) + 100) && (pts_plane[i].z > min(raw_loc_h[0], raw_loc_h[1]) - 100 && pts_plane[i].z < max(raw_loc_h[0], raw_loc_h[1]) + 100)) || (pts_plane[i].x > leftx - 100 && pts_plane[i].x < leftx + 100) || (pts_plane[i].x > rightx - 100 && pts_plane[i].x < rightx + 100))
		{
			if (reflect_plane[i] > planeMeanStd[0] - 2.5 * planeMeanStd[1] && reflect_plane[i] < planeMeanStd[0] + 2.5 * planeMeanStd[1])
			{
				pts_plane_copy.push_back(pts_plane[i]);
			}
		}
		else {
			pts_plane_copy.push_back(pts_plane[i]);
		}
	}
}

void DetectionHoleClass::checkHoleDepthPts(const std::vector<cv::Point3f>& pts_plane_copy, const cv::Point3f &normal_plane, const  std::vector<cv::Point3f>& windowPoints, const std::vector<int>& allRemovedReflect, const std::vector<cv::Point3f>& allRemovedPts_rot,
	const std::vector<int>& reflect_plane, const array<float, 2>& raw_loc_w, const array<float, 2>& raw_loc_h, const float bottomz, const float topz, const double frontY,
	const std::vector<double>& planeMeanStd, const float leftx, const float rightx, bool &calCornerWall2, bool &calCornerWall3, float &meanx_2, float &meanx_3) {
	float refPlaneMaxY2 = 0;

	float refPlaneMaxY3 = 0;

	float compPlaneMinY2 = numeric_limits<float>::infinity();

	float compPlaneMinY3 = numeric_limits<float>::infinity();

	std::vector<cv::Point3f> pts_plane_copy2;

	std::vector<cv::Point3f> pts_plane_copy3;

	vector<float> vecx_2;
	vector<float> vecx_3;

	float adetay = 20;
	float gamma = 80;

	for (int i = 0; i < pts_plane_copy.size(); i++) {
		if (pts_plane_copy[i].x > raw_loc_w[1] - 30 && pts_plane_copy[i].x < raw_loc_w[1] + 30)
			if (pts_plane_copy[i].z > min(raw_loc_h[0], raw_loc_h[1]) + gamma && pts_plane_copy[i].z < max(raw_loc_h[0], raw_loc_h[1]) - gamma)
				if (refPlaneMaxY2 < abs(pts_plane_copy[i].y)) {
					refPlaneMaxY2 = abs(pts_plane_copy[i].y);
				}

		if (pts_plane_copy[i].x > raw_loc_w[0] - 30 && pts_plane_copy[i].x < raw_loc_w[0] + 30)
			if (pts_plane_copy[i].z > min(raw_loc_h[0], raw_loc_h[1]) + gamma && pts_plane_copy[i].z < max(raw_loc_h[0], raw_loc_h[1]) - gamma)
				if (refPlaneMaxY3 < abs(pts_plane_copy[i].y)) {
					refPlaneMaxY3 = abs(pts_plane_copy[i].y);
				}
	}
	// rmoved Points
	if (raw_loc_h[0] < bottomz + 200) {
		for (int i = 0; i < allRemovedPts_rot.size(); i++) {
			if (allRemovedReflect[i] > planeMeanStd[0] - 3 * planeMeanStd[1] && allRemovedReflect[i] < planeMeanStd[0] + 3 * planeMeanStd[1]) {
				if (allRemovedPts_rot[i].x > raw_loc_w[1] - 30 && allRemovedPts_rot[i].x < raw_loc_w[1] + 30)
					if (allRemovedPts_rot[i].z > min(raw_loc_h[0], raw_loc_h[1]) + gamma && allRemovedPts_rot[i].z < max(raw_loc_h[0], raw_loc_h[1]) - gamma)
						if (allRemovedPts_rot[i].y > (frontY > 0 ? frontY + adetay : frontY - gamma) && allRemovedPts_rot[i].y < (frontY > 0 ? frontY + gamma : frontY - adetay)) {
							pts_plane_copy2.push_back(allRemovedPts_rot[i]);
							vecx_2.push_back(allRemovedPts_rot[i].x);
							meanx_2 += allRemovedPts_rot[i].x;
							if (compPlaneMinY2 > abs(allRemovedPts_rot[i].y)) {
								compPlaneMinY2 = abs(allRemovedPts_rot[i].y);
							}
						}
				if (allRemovedPts_rot[i].x > raw_loc_w[0] - 30 && allRemovedPts_rot[i].x < raw_loc_w[0] + 30)
					if (allRemovedPts_rot[i].z > min(raw_loc_h[0], raw_loc_h[1]) + gamma && allRemovedPts_rot[i].z < max(raw_loc_h[0], raw_loc_h[1]) - gamma)
						if (allRemovedPts_rot[i].y > (frontY > 0 ? frontY + adetay : frontY - gamma) && allRemovedPts_rot[i].y < (frontY > 0 ? frontY + gamma : frontY - adetay)) {
							pts_plane_copy3.push_back(allRemovedPts_rot[i]);
							vecx_3.push_back(allRemovedPts_rot[i].x);
							meanx_3 += allRemovedPts_rot[i].x;
							if (compPlaneMinY3 > abs(allRemovedPts_rot[i].y)) {
								compPlaneMinY3 = abs(allRemovedPts_rot[i].y);
							}
						}
			}
		}
	}
	else {
		//window Points
		for (int i = 0; i < windowPoints.size(); i++) {
			if (windowPoints[i].x > raw_loc_w[1] - 30 && windowPoints[i].x < raw_loc_w[1] + 30)
				if (windowPoints[i].z > min(raw_loc_h[0], raw_loc_h[1]) + gamma && windowPoints[i].z < max(raw_loc_h[0], raw_loc_h[1]) - gamma)
					if (windowPoints[i].y > (frontY > 0 ? frontY + adetay : frontY - 2 * gamma) && windowPoints[i].y < (frontY > 0 ? frontY + 2 * gamma : frontY - adetay)) {
						pts_plane_copy2.push_back(windowPoints[i]);
						vecx_2.push_back(windowPoints[i].x);
						meanx_2 += windowPoints[i].x;
						if (compPlaneMinY2 > abs(windowPoints[i].y)) {
							compPlaneMinY2 = abs(windowPoints[i].y);
						}
					}
			if (windowPoints[i].x > raw_loc_w[0] - 30 && windowPoints[i].x < raw_loc_w[0] + 30)
				if (windowPoints[i].z > min(raw_loc_h[0], raw_loc_h[1]) + gamma && windowPoints[i].z < max(raw_loc_h[0], raw_loc_h[1]) - gamma)
					if (windowPoints[i].y > (frontY > 0 ? frontY + adetay : frontY - gamma) && windowPoints[i].y < (frontY > 0 ? frontY + gamma : frontY - adetay)) {
						pts_plane_copy3.push_back(windowPoints[i]);
						vecx_3.push_back(windowPoints[i].x);
						meanx_3 += windowPoints[i].x;
						if (compPlaneMinY3 > abs(windowPoints[i].y)) {
							compPlaneMinY3 = abs(windowPoints[i].y);
						}
					}
		}
	}

	//ExportPts("D:\\data\\plydata\\pts_plane_copy2.txt", pts_plane_copy2);
	//ExportPts("D:\\data\\plydata\\pts_plane_copy3.txt", pts_plane_copy3);
	if (pts_plane_copy2.size() != 0) {
		meanx_2 = meanx_2 / pts_plane_copy2.size();
		sort(vecx_2.begin(), vecx_2.end());

		double plane_normals[3] = { 0.0,0.0,0.0 };
		if (ComputeNormal(pts_plane_copy2, plane_normals)) {
			//cout <<"Normal :"<< std::abs(normal_plane.x * plane_normals[0] + normal_plane.y * plane_normals[1] + normal_plane.z * plane_normals[2]) << endl;
			//cout << *vecx_2.begin() << " : " << *(vecx_2.end() - 1) << endl;
			if (std::abs(normal_plane.x * plane_normals[0] + normal_plane.y * plane_normals[1] + normal_plane.z * plane_normals[2]) < 0.05) {
				float refline = frontY > 0 ? frontY + gamma : frontY - gamma;
				if (sparseOrDenseAlongLineDirection(pts_plane_copy2, 0.95, refline)) {
					//cout << " raw_loc_w[m][1]  the neighbor wall is dense wall " << endl;
					//cout << meanx_2 << " - " << raw_loc_w[m][1] << " =" << raw_loc_w[m][1] - meanx_2 << endl;
					calCornerWall2 = true;
				}
				float bbox_plane[6];
				if (MathOperation::FindPlaneMinMaxXYZ(pts_plane_copy2, 1, 0.f, 0.f, bbox_plane)) {
					//cout << "refPlaneMaxY2 : " << refPlaneMaxY2 << " compPlaneMinY2 :" << compPlaneMinY2 << endl;
					if (refPlaneMaxY2 < compPlaneMinY2 + 15 && !isDensityOfWall(pts_plane_copy2.size(), bbox_plane, 0.001)) {
						calCornerWall2 = false;
					}
				}
			}
		}
	}
	if (pts_plane_copy3.size() != 0) {
		meanx_3 = meanx_3 / pts_plane_copy3.size();
		sort(vecx_3.begin(), vecx_3.end());

		double plane_normals[3] = { 0.0,0.0,0.0 };
		if (ComputeNormal(pts_plane_copy3, plane_normals)) {
			//cout << "Normal :" << std::abs(normal_plane.x * plane_normals[0] + normal_plane.y * plane_normals[1] + normal_plane.z * plane_normals[2]) << endl;
			//cout << *vecx_3.begin() << " : " << *(vecx_3.end() - 1) << endl;
			if (std::abs(normal_plane.x * plane_normals[0] + normal_plane.y * plane_normals[1] + normal_plane.z * plane_normals[2]) < 0.05)
			{
				float refline = frontY > 0 ? frontY + gamma : frontY - gamma;
				if (sparseOrDenseAlongLineDirection(pts_plane_copy3, 0.95, refline)) {
					//cout << "raw_loc_w[m][0] the neighbor wall is dense wall " << endl;
					//cout << meanx_3 << " - " << raw_loc_w[m][0] << " =" << raw_loc_w[m][0] - meanx_3 << endl;
					calCornerWall3 = true;
				}
				float bbox_plane[6];
				if (MathOperation::FindPlaneMinMaxXYZ(pts_plane_copy3, 1, 0.f, 0.f, bbox_plane)) {
					//cout << "refPlaneMaxY3 : " << refPlaneMaxY3 << " compPlaneMinY3 :" << compPlaneMinY3 << endl;
					if (refPlaneMaxY3 < compPlaneMinY3 + 15 && !isDensityOfWall(pts_plane_copy3.size(), bbox_plane, 0.001)) {
						calCornerWall3 = false;
					}
				}
			}
		}
	}

	pts_plane_copy2.clear();
	pts_plane_copy3.clear();
}
void checkHoleAgaistWall(const std::vector<cv::Point3f> pts_plane_copy, const std::vector<cv::Point3f> neighpoints, const double frontY, const float leftx, const float rightx, bool &leftAgaistWall, bool & rightAgaistWall, float *center_v) {
	if (neighpoints.size() == 2) {
		int sideEdgePointNum = 0;
		for (int i = 0; i < pts_plane_copy.size(); i++) {
			if (pts_plane_copy[i].z > -50 && pts_plane_copy[i].z< 50 && pts_plane_copy[i].x > center_v[2] - 100 && pts_plane_copy[i].x < center_v[2] + 100)
			{
				sideEdgePointNum++;
			}
		}
		// 判断门是否靠墙
		if (sideEdgePointNum == 0) {
			leftAgaistWall = true;
			if (frontY > 0) {
				float tmp = max(neighpoints[0].x, neighpoints[1].x);
				//cout << "tmp :"<< tmp << " " << leftx << endl;
				if (abs(tmp - leftx) < 50 && abs(center_v[2] - leftx) < 100) {
					center_v[2] = tmp;
				}
			}
			else {
				float tmp = min(neighpoints[0].x, neighpoints[1].x);
				//cout << "tmp :" <<  tmp << " " << leftx << endl;
				if (abs(tmp - leftx) < 50 && abs(center_v[2] - leftx) < 100) {
					center_v[2] = tmp;
				}
			}
		}
		sideEdgePointNum = 0;
		for (int i = 0; i < pts_plane_copy.size(); i++) {
			if (pts_plane_copy[i].z > -50 && pts_plane_copy[i].z< 50 && pts_plane_copy[i].x > center_v[3] - 50 && pts_plane_copy[i].x < center_v[3] + 50)
			{
				sideEdgePointNum++;
			}
		}
		if (sideEdgePointNum == 0) {
			rightAgaistWall = true;
			if (frontY > 0) {
				float tmp = min(neighpoints[0].x, neighpoints[1].x);
				//cout << "tmp :" << tmp << " " << rightx << endl;
				if (abs(tmp - rightx) < 50 && abs(center_v[3] - rightx) < 100) {
					center_v[3] = tmp;
				}
			}
			else {
				float tmp = max(neighpoints[0].x, neighpoints[1].x);
				//cout << "tmp :" <<  tmp << " " << rightx << endl;
				if (abs(tmp - rightx) < 50 && abs(center_v[3] - rightx) < 100) {
					center_v[3] = tmp;
				}
			}
		}
	}
}

void adjustbottomedge(const std::vector<cv::Point3f>& ground_points, const cv::Point3f& bottompt, const float Width_ruler, const float bottomz, const double frontY, float *center_v) {
	if (center_v[0] < bottomz + 5 * Width_ruler && bottompt.z != 0) {
		center_v[0] = bottompt.z;
	}
	if (center_v[0] > bottomz + 5 * Width_ruler) {
		vector<cv::Point3f> bottom_window_edge_pts;
		float bottom_window_edge = 0;
		vector<cv::Point3f> top_window_edge_pts;
		float top_window_edge = 0;
		for (int i = 0; i < ground_points.size(); i++) {
			if (frontY > 0) {
				if (ground_points[i].x > min(center_v[2], center_v[3]) + 50 && ground_points[i].x < max(center_v[2], center_v[3]) - 50 && ground_points[i].z < center_v[0] + 50 && ground_points[i].z > center_v[0] - 50 && ground_points[i].y > frontY + 70 && ground_points[i].y < frontY + 90)
				{
					bottom_window_edge_pts.push_back(ground_points[i]);
					bottom_window_edge += ground_points[i].z;
				}
				if (ground_points[i].x > min(center_v[2], center_v[3]) + 50 && ground_points[i].x < max(center_v[2], center_v[3]) - 50 && ground_points[i].z < center_v[1] + 50 && ground_points[i].z > center_v[1] - 50 && ground_points[i].y > frontY + 70 && ground_points[i].y < frontY + 90)
				{
					top_window_edge_pts.push_back(ground_points[i]);
					top_window_edge += ground_points[i].z;
				}
			}
			else {
				if (ground_points[i].x > min(center_v[2], center_v[3]) + 50 && ground_points[i].x < max(center_v[2], center_v[3]) - 50 && ground_points[i].z < center_v[0] + 50 && ground_points[i].z > center_v[0] - 50 && ground_points[i].y > frontY - 90 && ground_points[i].y < frontY - 70)
				{
					bottom_window_edge_pts.push_back(ground_points[i]);
					bottom_window_edge += ground_points[i].z;
				}
				if (ground_points[i].x > min(center_v[2], center_v[3]) + 50 && ground_points[i].x < max(center_v[2], center_v[3]) - 50 && ground_points[i].z < center_v[1] + 50 && ground_points[i].z > center_v[1] - 50 && ground_points[i].y > frontY - 90 && ground_points[i].y < frontY - 70)
				{
					top_window_edge_pts.push_back(ground_points[i]);
					top_window_edge += ground_points[i].z;
				}
			}
		}

		if (bottom_window_edge_pts.size() > 0) {
			//cout << "------- this case use lower plane ---------- " << endl;
			center_v[0] = bottom_window_edge / bottom_window_edge_pts.size();
		}
		if (top_window_edge_pts.size() > 0) {
			//cout << "------- this case use lower plane ---------- " << endl;
			center_v[1] = top_window_edge / top_window_edge_pts.size();
		}
		//ExportPts("D:\\data\\plydata\\ground_rotate.txt", ground_points);
		//ExportPts("D:\\data\\plydata\\bottom_window_edge_pts.txt", bottom_window_edge_pts);
	}
}

bool DetectionHoleClass::edgeFitting(const std::vector<cv::Point3f>& pts_plane, const std::vector<int>& reflect_plane, const  std::vector<cv::Point3f>& windowPoints, const std::vector<cv::Point3f>& allRemovedPts_rot, const std::vector<int>& allRemovedReflect, const std::vector<cv::Point3f>& ground_points, const cv::Point3f normal_plane, const float* bbox_plane,
	const std::vector<double>& planeMeanStd, std::vector<array<float, 2>>& raw_loc_w, std::vector<array<float, 2>>& raw_loc_h, const std::vector<cv::Point3f> neighpoints, const cv::Point3f& bottompt) {
	float bottomz = std::numeric_limits<float>::infinity();
	float topz = -std::numeric_limits<float>::infinity();
	float leftx = pts_plane[0].x;
	float rightx = pts_plane[0].x;
	double frontY = 0;
	for (int i = 0; i < pts_plane.size(); i++) {
		if (pts_plane[i].z < bottomz) bottomz = pts_plane[i].z;
		if (pts_plane[i].z > topz) topz = pts_plane[i].z;
		if (pts_plane[i].x > rightx) rightx = pts_plane[i].x;
		if (pts_plane[i].x < leftx) leftx = pts_plane[i].x;
		frontY += pts_plane[i].y;
		//cleared_plane.push_back(pts_plane[i]);
	}
	frontY = frontY / pts_plane.size();
	//ExportPts("D:\\data\\plydata\\cleared_plane.txt", pts_plane);

	std::vector<cv::Point3f> pts_plane_copy;// = pts_plane;

	for (int m = 0; m < raw_loc_w.size(); m++) {
		denoiseAroundHole(pts_plane, reflect_plane, raw_loc_w[m], raw_loc_h[m], planeMeanStd, leftx, rightx, pts_plane_copy);
		//ExportPts("D:\\data\\plydata\\pts_plane_copy.txt", pts_plane_copy);

		//cout << raw_loc_w[m][0] << " " << raw_loc_w[m][1] << endl;
		bool calCornerWall2 = false;
		bool calCornerWall3 = false;

		float meanx_2 = 0;
		float meanx_3 = 0;

		checkHoleDepthPts(pts_plane_copy, normal_plane, windowPoints, allRemovedReflect, allRemovedPts_rot,
			reflect_plane, raw_loc_w[m], raw_loc_h[m], bottomz, topz, frontY,
			planeMeanStd, leftx, rightx, calCornerWall2, calCornerWall3, meanx_2, meanx_3);

		Bbox hole;
		hole.x = raw_loc_w[m][0];
		hole.w = abs(raw_loc_w[m][0] - raw_loc_w[m][1]);
		hole.y = raw_loc_h[m][0];
		hole.h = abs(raw_loc_h[m][0] - raw_loc_h[m][1]);
		cout << hole.x << " " << hole.w + hole.x << " " << hole.y << " " << hole.y + hole.h << " " << hole.w << " " << hole.h << endl;// " " << neighpoints[0].x << " " << neighpoints[1].x << endl;
		// 0: 横最低，1: 横最高，2: 竖最左，3 ：竖最右
		float center_v[4] = { hole.y, hole.y + hole.h,0,0 };
		float Width_ruler = 20.f;
		int direction[4] = { 1,1,-1,1 };
		int num_first[4] = { 0, 0, 0, 0 };
		/*for (int i = 0; i < pts_plane.size(); i++) {
			frontY += pts_plane[i].y;
		}*/
		//cout << " right : " << rightx << " left :" << leftx << endl;
		if (frontY > 0) {
			center_v[2] = hole.x + hole.w;
			center_v[3] = hole.x;
			swap(leftx, rightx);
		}
		else {
			center_v[2] = hole.x;
			center_v[3] = hole.x + hole.w;
		}
		//cout <<" right : "<< rightx <<" left :" <<leftx<< endl;
		updateLeftRightInitPos(pts_plane_copy, rightx, leftx, frontY > 0, center_v);
		//cout << center_v[2] << " " << center_v[3] << endl;
		//cout << center_v[0] << "  " <<center_v[1] <<" "<< center_v[2] << "  " <<center_v[3] << endl;
		lineApproaching(pts_plane_copy, bottomz, topz, leftx, rightx, center_v, direction, num_first);
		//cout << center_v[0] << "  " << center_v[1] << " " << center_v[2] << "  " << center_v[3] << endl;

		adjustHoleY(pts_plane_copy, bottomz, topz, leftx, rightx, center_v, direction);

		adjustHoleX(pts_plane_copy, bottomz, topz, leftx, rightx, center_v, direction, num_first);
		//cout << center_v[0] << "  " << center_v[1] << " " << center_v[2] << "  " << center_v[3] << endl;
		bool leftAgaistWall = false;
		bool rightAgaistWall = false;

		// 更新洞的坐标信息。

		if (frontY > 0) {
			raw_loc_w[m][0] = center_v[3];
			raw_loc_w[m][1] = center_v[2];
		}
		else {
			raw_loc_w[m][0] = center_v[2];
			raw_loc_w[m][1] = center_v[3];
		}
		//cout << leftAgaistWall <<" " << rightAgaistWall << " " << calCornerWall2 << " " << calCornerWall3 << endl;
		//cout << raw_loc_w[m][0] << " " << raw_loc_w[m][1] << " " << raw_loc_h[m][0] << " " << raw_loc_h[m][1] << " " << abs(raw_loc_w[m][1] - raw_loc_w[m][0]) << " " << abs(raw_loc_h[m][1] - raw_loc_h[m][0]) << endl;
		if (calCornerWall2) {
			//cout << raw_loc_w[m][1] << " " << *vecx_2.begin() << " " << *(vecx_2.end() - 1) << " " << meanx_2 << endl;
			//cout << raw_loc_w[m][1]<<" "<< meanx_2 << endl;
			if (abs(raw_loc_w[m][1] - meanx_2) > 7)
				raw_loc_w[m][1] = meanx_2;
		}
		if (calCornerWall3) {
			//cout << raw_loc_w[m][0] << " " << *vecx_3.begin() << " " << *(vecx_3.end() - 1) << " : " << meanx_3 << endl;
			//cout << raw_loc_w[m][0] << " " << meanx_3 << endl;
			if (abs(raw_loc_w[m][0] - meanx_3) > 7)
				raw_loc_w[m][0] = meanx_3;
		}

		checkHoleAgaistWall(pts_plane_copy, neighpoints, frontY, leftx, rightx, leftAgaistWall, rightAgaistWall, center_v);

		adjustbottomedge(ground_points, bottompt, Width_ruler, bottomz, frontY, center_v);

		raw_loc_h[m][0] = center_v[0];
		raw_loc_h[m][1] = center_v[1];

		pts_plane_copy.clear();

		if (frontY > 0) {
			swap(leftx, rightx);
		}
		cout << raw_loc_w[m][0] << " " << raw_loc_w[m][1] << " " << raw_loc_h[m][0] << " " << raw_loc_h[m][1] << " " << abs(raw_loc_w[m][1] - raw_loc_w[m][0]) << " " << abs(raw_loc_h[m][1] - raw_loc_h[m][0]) << endl;
	}

	return true;
}

bool DetectionHoleClass::LocateHole_w(
	const std::vector<cv::Point3f> &pts_plane,
	const cv::Point3f normal_plane,
	const float *bbox_plane,
	std::vector<array<float, 2>> &raw_loc_w)
{
	if (pts_plane.empty()) {
		PrintMsg("err2-0: empty input for one wall", "DetcHole::GetHoleWidth(): empty input for one wall");
		return false;
	}
	unsigned int npts = static_cast<unsigned int>(pts_plane.size());
	float center_v = (bbox_plane[4] + bbox_plane[5]) * 0.5f;
	float spacing_ruler = (bbox_plane[5] - bbox_plane[4]) * 0.3f;

	//std::array<float, 2> bbox_u;
	std::pair<float, float> bbox_u;
	//parallel to x-axis
	if (Util_Math::IsValZero(normal_plane.x)) {
		bbox_u.first = bbox_plane[0];
		bbox_u.second = bbox_plane[1];
	}
	//parallel to y-axis
	else if (Util_Math::IsValZero(normal_plane.y)) {
		bbox_u.first = bbox_plane[2];
		bbox_u.second = bbox_plane[3];
	}
	else {
		PrintMsg("err2-1: without rotation of wall", "DetcHole::GetHoleWidth(): without rotation for wall");
		return false;
	}
	// compute bounds of each ruler
	unsigned int n_ruler = 3; // number of ruler
	std::vector<std::vector<unsigned int>> ptID_rulers;
	ptID_rulers.resize(n_ruler);
	// compute ruler bounds at v direction
	std::vector<std::pair<float, float>> bbox_ruler_v;
	bbox_ruler_v.resize(n_ruler);
	unsigned int count = 0;
	//k = -1: 1st ruler at (center_v - spacing_ruler);
	//k = 0:  2nd ruler at center_v;
	//k = 1:  3rd ruler at (center_v + spacing_ruler)
	for (int k = -1; k <= 1; k++) {
		bbox_ruler_v[count] = std::make_pair(
			center_v + k * spacing_ruler - m_halfWidth_ruler * 1.5,
			center_v + k * spacing_ruler + m_halfWidth_ruler * 1.5);
		count++;
	}
	// append points in ruler
	for (unsigned int i = 0; i < n_ruler; i++) {
		AppendPt2Ruler1D(pts_plane, bbox_ruler_v[i], AxisType::Axis_Z, ptID_rulers[i]);
	}

	unsigned int npts_in_ruler = 0;
	for (unsigned int i = 0; i < n_ruler; i++) {
		npts_in_ruler += ptID_rulers[i].size();
	}
	// check if all ruler empty or not, false: a big hole then use hole raw location to compute width
	if (npts_in_ruler) {
		//contain all ruler gaps
		std::vector<std::vector<array<float, 2>>> gaps_loc_hole;

		for (int k = 0; k < 3; k++)
		{
			std::vector<array<float, 2>> gaps_loc_ruler;
			cv::Mat sorted_ptID_ruler(static_cast<int>(ptID_rulers[k].size()), 1, CV_32S);
			//put pts coordinates along x-axis or y-axis
			cv::Mat vals_ruler(static_cast<int>(ptID_rulers[k].size()), 1, CV_32F);

			if (ptID_rulers[k].size() == 0) { // use bounds as gap
				std::vector<array<float, 2>> gap;
				gap.resize(1);
				gap[0][0] = bbox_u.first; gap[0][1] = bbox_u.second;
				gaps_loc_hole.push_back(gap);
				raw_loc_w.push_back(gap[0]);
				continue;
			}
			if (Util_Math::IsValZero(normal_plane.x)) {
				for (int i = 0; i < ptID_rulers[k].size(); i++) {
					vals_ruler.at<float>(i, 0) = pts_plane[ptID_rulers[k][i]].x;
				}
			}
			else if (Util_Math::IsValZero(normal_plane.y)) {
				for (int i = 0; i < ptID_rulers[k].size(); i++) {
					vals_ruler.at<float>(i, 0) = pts_plane[ptID_rulers[k][i]].y;
				}
			}
			else {
				PrintMsg("err2-2: without rotation of wall", "DetcHole::GetHoleWidth(): without rotation for wall");
				return false;
			}

			//sort pts and save sorted indx
			cv::sortIdx(vals_ruler, sorted_ptID_ruler, CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);

			// debug output sorted points
			//ExportPts("ruler data", pts_plane, ptID_rulers[k], sorted_ptID_ruler);

			bool isGapInRuler = FindGapsInRuler(vals_ruler, sorted_ptID_ruler, bbox_u, gaps_loc_ruler);
			if (isGapInRuler) {
				gaps_loc_hole.push_back(gaps_loc_ruler);
				for (int m = 0; m < gaps_loc_ruler.size(); m++) //add by lpc
					raw_loc_w.push_back(gaps_loc_ruler[m]);
			}
		}
		// find ruler with maximum number of gaps
		// if number equal, compare summation of gap length
		if (gaps_loc_hole.size() > 0) {
			ClearStdVector(ptID_rulers);
			ClearStdVector(gaps_loc_hole);
			return true;
		}
		else {
			ClearStdVector(ptID_rulers);
			ClearStdVector(gaps_loc_hole);
			return false;
		}
	}
	else {
		//parallel to x-axis
		if (Util_Math::IsValZero(normal_plane.x))
			raw_loc_w.push_back({ bbox_plane[0],bbox_plane[1] });
		//parallel to y-axis
		else if (Util_Math::IsValZero(normal_plane.y))
			raw_loc_w.push_back({ bbox_plane[2],bbox_plane[3] });
		else {
			PrintMsg("err2-3: without rotation of wall", "DetcHole::GetHoleWidth(): without rotation of wall");
			return false;
		}
		//RELEASE MEMORY
		ClearStdVector(ptID_rulers);
		return true;
	}
}

float DetectionHoleClass::iou(Bbox box1, Bbox box2)
{
	int x1 = (std::max)(box1.x, box2.x);
	int y1 = (std::max)(box1.y, box2.y);
	int x2 = (std::min)((box1.x + box1.w), (box2.x + box2.w));
	int y2 = (std::min)((box1.y + box1.h), (box2.y + box2.h));
	float over_area = (x2 - x1) * (y2 - y1);
	float iou = over_area / (box1.w * box1.h + box2.w * box2.h - over_area);
	return iou;
}

bool sort_score(const DetectionHoleClass::Bbox &box1, const DetectionHoleClass::Bbox &box2)
{
	return (box1.score > box2.score);
}

void DetectionHoleClass::nms(vector<Bbox>&vec_boxs, float threshold, vector<Bbox>& results)
{
	std::sort(vec_boxs.begin(), vec_boxs.end(), sort_score);
	while (vec_boxs.size() > 0)
	{
		results.push_back(vec_boxs.front());
		//int i = 0;

		for (vector<Bbox>::iterator iter = vec_boxs.begin() + 1; iter != vec_boxs.end(); )
		{
			//cout << vec_boxs.size() << endl;
			float iou_value = iou(vec_boxs[0], *(iter));
			//cout << iou_value << " "<< 0 <<" "<< vec_boxs[0].x << " "<< vec_boxs[0].y << " "<< vec_boxs[0].w << " " << vec_boxs[0].h<< endl;
			//cout << iou_value << " "<< i+1 << " " << vec_boxs[i + 1].x << " " << vec_boxs[i + 1].y << " " << vec_boxs[i + 1].w << " " << vec_boxs[ i+1].h << endl;
			if (iou_value > threshold)
			{
				iter = vec_boxs.erase(iter);
			}
			else {
				iter++;
			}
		}
		vec_boxs.erase(vec_boxs.begin());
	}
}

bool DetectionHoleClass::calMinZFromRulerPointCloud(const std::vector<cv::Point3f> &pts_plane,
	const cv::Point3f normal_plane, const std::vector<cv::Point3f> &ruler_corner, float & minZ) {
	if (pts_plane.size() < 3 || ruler_corner.size() != 4) {
		return false;
	}
	std::vector<cv::Point3f> pts_plane_rot;
	cv::Point3f rot_axis = { 0.f, 1.f, 0.f };

	float p_normal_plane[3] = { normal_plane.x, normal_plane.y, normal_plane.z };
	cv::Point3f cross = Util_Math::ComputeVectorCrossProduct(normal_plane, rot_axis);
	float  angle_rot = (cross.z > 0.f) ?
		acosf(normal_plane.y / std::sqrt(std::pow(normal_plane.x, 2) + std::pow(normal_plane.y, 2))) :
		-acosf(normal_plane.y / std::sqrt(std::pow(normal_plane.x, 2) + std::pow(normal_plane.y, 2)));
	RotatePointsAroundZ(pts_plane, angle_rot, pts_plane_rot);

	std::vector<cv::Point3f> temp;
	temp.resize(4);
	RotatePointsAroundZ(ruler_corner, angle_rot, temp);
	float ruler_center_w = (temp[0].x + temp[1].x + temp[2].x + temp[3].x) / 4;

	std::pair<float, float> bounds = std::make_pair(ruler_center_w - 40, ruler_center_w + 40);
	//append pts to ruler
	std::vector<unsigned int> ptID_ruler;
	if (Util_Math::IsValZero(rot_axis.x)) {
		AppendPt2Ruler1D(pts_plane, bounds, AxisType::Axis_X, ptID_ruler);
	}
	else if (Util_Math::IsValZero(rot_axis.y)) {
		AppendPt2Ruler1D(pts_plane, bounds, AxisType::Axis_Y, ptID_ruler);
	}
	else {
		PrintMsg("err3-2: no roatation for wall", "DetcHole::GetHoleHeight(): no roatation for wall");
		return false;
	}
	minZ = std::numeric_limits<float>::infinity();
	for (int i = 0; i < ptID_ruler.size(); i++) {
		if (pts_plane[ptID_ruler[i]].z < minZ) {
			minZ = pts_plane[ptID_ruler[i]].z;
		}
	}
	return true;
}

bool DetectionHoleClass::LocateHole_h(
	const std::vector<cv::Point3f> &pts_plane,
	const cv::Point3f normal_plane,
	const float *bbox_plane,
	std::vector<array<float, 2>> &raw_loc_w,
	std::vector<array<float, 2>> &raw_loc_h)
{
	// data validation
	size_t n_gap_w = raw_loc_w.size();
	if (n_gap_w == 0) return false;

	//init size of raw_loc_h
	//raw_loc_h.resize(n_gap_w); delete by lpc
	int count_avail_hole = 0;

	//set min max of z
	std::pair<float, float> bbox_v;
	bbox_v.first = bbox_plane[4];
	bbox_v.second = bbox_plane[5];
	vector<Bbox>vec_boxs;
	for (int id_gap_w = 0; id_gap_w < n_gap_w; id_gap_w++)
	{
		// vertical ruler is set at mid position of horizontal ruler
		float ruler_center_w = (raw_loc_w[id_gap_w][0] + raw_loc_w[id_gap_w][1]) * 0.5f;
		std::pair<float, float> bounds =
			std::make_pair(ruler_center_w - m_halfWidth_ruler, ruler_center_w + m_halfWidth_ruler);
		//append pts to ruler
		std::vector<unsigned int> ptID_ruler;
		if (Util_Math::IsValZero(normal_plane.x)) {
			AppendPt2Ruler1D(pts_plane, bounds, AxisType::Axis_X, ptID_ruler);
		}
		else if (Util_Math::IsValZero(normal_plane.y)) {
			AppendPt2Ruler1D(pts_plane, bounds, AxisType::Axis_Y, ptID_ruler);
		}
		else {
			PrintMsg("err3-2: no roatation for wall", "DetcHole::GetHoleHeight(): no roatation for wall");
			return false;
		}

		size_t npts_ruler = ptID_ruler.size();

		if (npts_ruler != 0) {
			cv::Mat vals_ruler(static_cast<int>(ptID_ruler.size()), 1, CV_32F);
			//vector<cv::Point3f> tmppts;
			for (int i = 0; i < ptID_ruler.size(); i++)
			{
				vals_ruler.at<float>(i, 0) = pts_plane[ptID_ruler[i]].z;
				//tmppts.push_back(pts_plane[ptID_ruler[i]]);
			}
			//sort pts and save sorted indx
			cv::Mat sorted_ptID_ruler(static_cast<int>(ptID_ruler.size()), 1, CV_32S);
			cv::sortIdx(vals_ruler, sorted_ptID_ruler, CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);
			//ExportPts("D:\\data\\plydata\\ruler data.txt", vals_ruler, ptID_ruler[i], sorted_ptID_ruler);
			std::vector<array<float, 2>> gaps_loc;
			bool isGapInRuler = FindGapsInRuler(vals_ruler, sorted_ptID_ruler, bbox_v, gaps_loc);

			// init location of hole_h   // deleted by lpc
			/*	raw_loc_h[id_gap_w][0] = std::numeric_limits<float>::infinity();
			raw_loc_h[id_gap_w][1] = -std::numeric_limits<float>::infinity();*/
			if (isGapInRuler)
			{   // find only one pair of max gap
				/*ExportPts("D:\\data\\plydata\\pts_plane.txt", pts_plane);

				ExportPts("D:\\data\\plydata\\ruler_data.txt", tmppts);*/

				for (int i = 0; i < gaps_loc.size(); i++) {
					Bbox box1;
					box1.x = raw_loc_w[id_gap_w][0];
					box1.y = gaps_loc[i][0];
					box1.w = abs(raw_loc_w[id_gap_w][1] - raw_loc_w[id_gap_w][0]);
					box1.h = abs(gaps_loc[i][1] - gaps_loc[i][0]);
					box1.score = box1.w*box1.h / 1000.0;
					vec_boxs.push_back(box1);
				}
			}
		}
		else {
			Bbox box1;
			box1.x = raw_loc_w[id_gap_w][0];
			box1.y = bbox_plane[4];
			box1.w = abs(raw_loc_w[id_gap_w][1] - raw_loc_w[id_gap_w][0]);
			box1.h = abs(bbox_plane[5] - bbox_plane[4]);
			box1.score = box1.w * box1.h / 1000.0;
			vec_boxs.push_back(box1);
		}

		ClearStdVector(ptID_ruler);
	}

	vector<Bbox> result;
	DetectionHoleClass::nms(vec_boxs, 0.1, result);
	// improve hole edge precison
	for (int i = 0; i < result.size(); i++) {
		Bbox hole = result[i];
		checkHoleEdge(pts_plane, normal_plane, bbox_plane, hole);
		//edgeFitting(pts_plane, normal_plane, bbox_plane,hole);
		result[i] = hole;
	}

	raw_loc_w.clear();
	raw_loc_h.clear();
	//cout <<" result.size() " << result.size() << endl;
	for (int i = 0; i < result.size(); i++) {
		raw_loc_w.push_back({ result.at(i).x ,result.at(i).x + result.at(i).w });
		raw_loc_h.push_back({ result.at(i).y ,result.at(i).y + result.at(i).h });
		count_avail_hole++;
		//cout << result.at(i).x << " "<<result.at(i).y<< endl;
	}

	result.clear();
	return true;
}

bool DetectionHoleClass::Measure(
	const std::vector<cv::Point3f> & pts_plane,
	const cv::Point3f normal_plane,
	const float *bbox_plane,
	const std::vector<array<float, 2>> &raw_loc_w,
	const std::vector<array<float, 2>> &raw_loc_h,
	std::vector<array<float, 4>> &output_w_h)
{
	// number of hole
	size_t n_hole = raw_loc_w.size();
	// data validation
	if (n_hole == 0) return false;

	output_w_h.resize(n_hole);

	for (size_t i = 0; i < n_hole; i++)
	{
		array<float, 4> output;
		bool isSucceed = MeasureHole(pts_plane, normal_plane, bbox_plane, raw_loc_w[i], raw_loc_h[i], output);
		if (isSucceed) {
			output_w_h[i] = output;
		}
		else {
			std::fill(output.begin(), output.end(), nanf(""));
			output_w_h[i] = output;
		}
	}
	return true;
}

bool DetectionHoleClass::Sample1D(const float lower, const float upper, const float margin_lower,
	const float margin_upper, std::vector<float> &output)
{
	// data validation
	if (m_decn_tree == nullptr) {
		return false;
	}
	bool isSucceed = false;
	float range = std::fabs(upper - lower - margin_upper - margin_lower);
	Decn_Doc<float> *data = new Decn_Doc<float>();
	data->appendUData(range);
	int result = m_decn_tree->decision(*data);
	unsigned int nSample = 5;
	switch (result)
	{
	case 0:
		isSucceed = Sample1D(lower, upper, margin_lower, margin_upper, nSample, output);
		break;
	case 1:
		isSucceed = Sample1D(lower, upper, 0.f, 0.f, nSample, output);
		break;
	default:
		break;
	}
	return isSucceed;
}

bool DetectionHoleClass::Sample1D(
	const float lower, const float upper, const float margin_lower,
	const float margin_upper, const unsigned int nSample, std::vector<float> &output)
{
	Util_Sampler_1D<float> *sampler = new Util_Sampler_1D<float>();
	bool isSucceed = sampler->sample_uniform(std::make_pair(lower, upper),
		margin_lower, margin_upper, nSample, output);
	delete sampler;
	return isSucceed;
}

bool DetectionHoleClass::Sample1D(const float lower, const float upper, const float margin_lower,
	const float margin_upper, const bool isSymmetric, const std::vector<float> &intvl_len,
	std::vector<float> &output)
{
	Util_Sampler_1D<float> *sampler = new Util_Sampler_1D<float>();
	bool isSucceed = false;
	if (isSymmetric) {
		isSucceed = sampler->sample_nonunf_len(std::make_pair(lower, upper), margin_lower, margin_upper,
			Util_Sampler_1D<float>::COND_SYM::SYMMETRIC_ODD, intvl_len, output);
	}
	else {
		isSucceed = sampler->sample_nonunf_len(std::make_pair(lower, upper), margin_lower, margin_upper,
			Util_Sampler_1D<float>::COND_SYM::ASYMMETRIC, intvl_len, output);
	}
	return isSucceed;
}

bool DetectionHoleClass::MeasureHole(
	const std::vector<cv::Point3f> & pts_plane,
	const cv::Point3f normal_plane,
	const float *bbox_plane,
	const array<float, 2> &raw_loc_w,
	const array<float, 2> &raw_loc_h,
	array<float, 4> &output_w_h)
{
	//cout << "bbox_plane: x(" << bbox_plane[0] << ", " << bbox_plane[1]
	//	<< ") y(" << bbox_plane[2] << ", " << bbox_plane[3]
	//	<< ") z(" << bbox_plane[4] << ", " << bbox_plane[5] << ")\n";

	// in this function, redefine coordinate system
	// horizontal direction is u
	// vertical direction is v
	// w is width, measured along u direction, perpendicular to v direction
	// h is height, measured along v direction, perpendicular to u direction
	// use u instead of xy, use v instead of z

	// const unsigned int n_sample = 5; // param: number of measure position
	float offset_bound = 50.f; // param: extend raw bound for measuring purpose

	std::vector<float> sample_pos_u;
	std::vector<float> sample_pos_v;

	Sample1D(raw_loc_w[0], raw_loc_w[1], m_sample_margin, m_sample_margin, sample_pos_u);
	Sample1D(raw_loc_h[0], raw_loc_h[1], m_sample_margin, m_sample_margin, sample_pos_v);

	size_t n_sample_u = sample_pos_u.size();
	size_t n_sample_v = sample_pos_v.size();

	// find bounding box of each hole
	std::pair<float, float> bbox_hole_u;
	std::pair<float, float> bbox_hole_v;

	// get local bound for hole
	float diff = 2.f;
	bbox_hole_v.first = (bbox_plane[4] >= raw_loc_h[0] || abs(bbox_plane[4] - raw_loc_h[0]) < diff)
		? bbox_plane[4] : raw_loc_h[0] - offset_bound;
	bbox_hole_v.second = (bbox_plane[5] <= raw_loc_h[1] || abs(bbox_plane[5] - raw_loc_h[1]) < diff)
		? bbox_plane[5] : raw_loc_h[1] + offset_bound;

	if (Util_Math::IsValZero(normal_plane.x)) // rotate to y-z plane, measure x
	{
		std::vector<float> vals;
		vals.push_back(bbox_plane[0]);
		vals.push_back(bbox_plane[1]);
		vals.push_back(raw_loc_w[0] - offset_bound);
		vals.push_back(raw_loc_w[1] + offset_bound);
		float val_min, val_max;
		getMinMaxfromVector(vals, val_min, val_max);

		bbox_hole_u.first = (bbox_plane[0] >= raw_loc_w[0] || abs(bbox_plane[0] - raw_loc_w[0]) < diff)
			? bbox_plane[0] : raw_loc_w[0] - offset_bound;
		bbox_hole_u.second = (bbox_plane[1] <= raw_loc_w[1] || abs(bbox_plane[1] - raw_loc_w[1]) < diff)
			? bbox_plane[1] : raw_loc_w[1] + offset_bound;

		if (abs(bbox_hole_u.first - bbox_hole_u.second) < m_thres_minLen_hole) {
			bbox_hole_u.first = val_min;
			bbox_hole_u.second = val_max;
		}
	}
	//parallel to y-axis
	else if (Util_Math::IsValZero(normal_plane.y)) // rotate to x-z plane, measure y
	{
		std::vector<float> vals;
		vals.push_back(bbox_plane[2]);
		vals.push_back(bbox_plane[3]);
		vals.push_back(raw_loc_w[0] - offset_bound);
		vals.push_back(raw_loc_w[1] + offset_bound);
		float val_min, val_max;
		getMinMaxfromVector(vals, val_min, val_max);

		bbox_hole_u.first = (bbox_plane[2] >= raw_loc_w[0] || abs(bbox_plane[2] - raw_loc_w[0]) < diff)
			? bbox_plane[2] : raw_loc_w[0] - offset_bound;
		bbox_hole_u.second = (bbox_plane[3] <= raw_loc_w[1] || abs(bbox_plane[3] - raw_loc_w[1]) < diff)
			? bbox_plane[3] : raw_loc_w[1] + offset_bound;

		if (abs(bbox_hole_u.first - bbox_hole_u.second) < m_thres_minLen_hole) {
			bbox_hole_u.first = val_min;
			bbox_hole_u.second = val_max;
		}
	}
	else
		return false;

	std::vector<float> cands_w; // candidates of width
	std::vector<float> cands_h; // candidates of height

	for (size_t i = 0; i < n_sample_v; i++) {
		cands_w.push_back(
			MeasureRuler(pts_plane, normal_plane, bbox_hole_u, bbox_hole_v, OrienType::ORIEN_HORIZ, sample_pos_v[i]));
		// cout << "cand_w: " << cands_w[cands_w.size() - 1] << "\n";
	}
	for (size_t i = 0; i < n_sample_u; i++) {
		cands_h.push_back(
			MeasureRuler(pts_plane, normal_plane, bbox_hole_u, bbox_hole_v, OrienType::ORIEN_VERT, sample_pos_u[i]));
	}

	// compute width and height
	// init value, width-width-height-height
	unsigned int n_output = 4; // each hole has four output, 2 width, 2 height
	for (unsigned int id_output = 0; id_output < n_output; id_output++) {
		output_w_h[id_output] = 0.f;
	}

	// determine from candidates for output
	std::sort(cands_w.begin(), cands_w.end());
	std::sort(cands_h.begin(), cands_h.end());
	unsigned int n_w = static_cast<unsigned int>(cands_w.size());
	unsigned int n_h = static_cast<unsigned int>(cands_h.size());
	if (n_w < 2) {
		output_w_h[0] = bbox_hole_u.second - bbox_hole_u.first;
		output_w_h[1] = output_w_h[0];
	}
	else {
		output_w_h[0] = cands_w[n_w - 1];
		output_w_h[1] = cands_w[n_w - 2];
	}
	if (n_h < 2) {
		output_w_h[2] = bbox_hole_v.second - bbox_hole_v.first;
		output_w_h[3] = output_w_h[2];
	}
	else {
		output_w_h[2] = cands_h[n_h - 1];
		output_w_h[3] = cands_h[n_h - 2];
	}

	ClearStdVector(cands_w);
	ClearStdVector(cands_h);
	return true;
}

//bool DetectionHoleClass::MeasureHole(
//	const std::vector<cv::Point3f> & pts_plane,
//	const cv::Point3f normal_plane,
//	const float *bbox_plane,
//	const array<float, 2> &raw_loc_w,
//	const array<float, 2> &raw_loc_h,
//	array<float, 4> &output_w_h)
//{
//	// in this function, redefine coordinate system
//	// horizontal direction is u
//	// vertical direction is v
//	// w is width, measured along u direction, perpendicular to v direction
//	// h is height, measured along v direction, perpendicular to u direction
//	// use u instead of xy, use v instead of z
//	const unsigned int n_sample = 5; // param: number of measure position
//	float offset_bound = 50.f; // param: extend raw bound for measuring purpose
//
//	std::vector<float> sample_pos_u;
//	std::vector<float> sample_pos_v;
//
//	sample_pos_u.resize(n_sample);
//	sample_pos_v.resize(n_sample);
//
//	float spacing_u = (raw_loc_w[1] - raw_loc_w[0] + 2.f * offset_bound) / (n_sample + 1);
//	float spacing_v = (raw_loc_h[1] - raw_loc_h[0] + 2.f * offset_bound) / (n_sample + 1);
//
//	// determine each test position
//	for (int j = 0; j < n_sample; j++)
//	{
//		sample_pos_u[j] = raw_loc_w[0] - offset_bound + (j + 1) * spacing_u;
//		if (sample_pos_u[j] < raw_loc_w[0])
//			sample_pos_u[j] = raw_loc_w[0] + offset_bound * 0.5f;
//		else if (sample_pos_u[j] > raw_loc_w[1])
//			sample_pos_u[j] = raw_loc_w[1] - offset_bound * 0.5f;
//
//		sample_pos_v[j] = raw_loc_h[0] - offset_bound + (j + 1) * spacing_v;
//		if (sample_pos_v[j] < raw_loc_h[0])
//			sample_pos_v[j] = raw_loc_h[0] + offset_bound * 0.5f;
//		else if (sample_pos_v[j] > raw_loc_h[1])
//			sample_pos_v[j] = raw_loc_h[1] - offset_bound * 0.5f;
//	}
//	// find bounding box of each hole
//	std::pair<float, float> bbox_hole_u;
//	std::pair<float, float> bbox_hole_v;
//
//	// get local bound for hole
//	float diff = 2.f;
//	bbox_hole_v.first = (bbox_plane[4] >= raw_loc_h[0] || abs(bbox_plane[4] - raw_loc_h[0]) < diff)
//		? bbox_plane[4] : raw_loc_h[0] - offset_bound;
//	bbox_hole_v.second = (bbox_plane[5] <= raw_loc_h[1] || abs(bbox_plane[5] - raw_loc_h[1]) < diff)
//		? bbox_plane[5] : raw_loc_h[1] + offset_bound;
//
//	if (IsValZero(normal_plane.x)) // rotate to y-z plane, measure x
//	{
//		bbox_hole_u.first = (bbox_plane[0] >= raw_loc_w[0] || abs(bbox_plane[0] - raw_loc_h[0]) < diff)
//			? bbox_plane[0] : raw_loc_w[0] - offset_bound;
//		bbox_hole_u.second = (bbox_plane[1] <= raw_loc_w[1] || abs(bbox_plane[1] - raw_loc_h[1]) < diff)
//			? bbox_plane[1] : raw_loc_w[1] + offset_bound;
//	}
//	//parallel to y-axis
//	else if (IsValZero(normal_plane.y)) // rotate to x-z plane, measure y
//	{
//		bbox_hole_u.first = (bbox_plane[2] >= raw_loc_w[0] || abs(bbox_plane[2] - raw_loc_h[0]) < diff)
//			? bbox_plane[2] : raw_loc_w[0] - offset_bound;
//		bbox_hole_u.second = (bbox_plane[3] <= raw_loc_w[1] || abs(bbox_plane[3] - raw_loc_h[1]) < diff)
//			? bbox_plane[3] : raw_loc_w[1] + offset_bound;
//	}
//	else
//		return false;
//
//	std::vector<float> cands_w; // candidates of width
//	std::vector<float> cands_h; // candidates of height
//
//	for (size_t i = 0; i < n_sample; i++) {
//		cands_w.push_back(
//			MeasureRuler(pts_plane, normal_plane, bbox_hole_u, bbox_hole_v, OrienType::ORIEN_HORIZ, sample_pos_v[i]));
//	}
//	for (size_t i = 0; i < n_sample; i++) {
//		cands_h.push_back(
//			MeasureRuler(pts_plane, normal_plane, bbox_hole_u, bbox_hole_v, OrienType::ORIEN_VERT, sample_pos_u[i]));
//	}
//
//	// compute width and height
//	// init value, width-width-height-height
//	unsigned int n_output = 4; // each hole has four output, 2 width, 2 height
//	for (unsigned int id_output = 0; id_output < n_output; id_output++) {
//		output_w_h[id_output] = 0.f;
//	}
//
//	// determine from candidates for output
//	std::sort(cands_w.begin(), cands_w.end());
//	std::sort(cands_h.begin(), cands_h.end());
//	unsigned int n_w = static_cast<unsigned int>(cands_w.size());
//	unsigned int n_h = static_cast<unsigned int>(cands_h.size());
//	if (n_w < 2) {
//		output_w_h[0] = bbox_hole_u.second - bbox_hole_u.first;
//		output_w_h[1] = output_w_h[0];
//	}
//	else {
//		output_w_h[0] = cands_w[n_w - 1];
//		output_w_h[1] = cands_w[n_w - 2];
//	}
//	if (n_h < 2) {
//		output_w_h[2] = bbox_hole_v.second - bbox_hole_v.first;
//		output_w_h[3] = output_w_h[2];
//	}
//	else {
//		output_w_h[2] = cands_h[n_h - 1];
//		output_w_h[3] = cands_h[n_h - 2];
//	}
//
//	ClearStdVector(cands_w);
//	ClearStdVector(cands_h);
//	return true;
//}

float DetectionHoleClass::MeasureRuler(
	const std::vector<cv::Point3f> &pts_plane,
	const cv::Point3f &normal_plane,
	const std::pair<float, float> &bbox_hole_u,
	const std::pair<float, float> &bbox_hole_v,
	const OrienType orien,
	const float sample_pos)
{
	float output_gap = 0.f;
	std::vector<unsigned int> ptID_ruler;
	// compute ruler bounds based on orientation
	cv::Point2f bbox_min, bbox_max;
	if (orien == OrienType::ORIEN_HORIZ) {	// u dir
		bbox_min.x = bbox_hole_u.first - m_halfWidth_ruler;
		bbox_max.x = bbox_hole_u.second + m_halfWidth_ruler;
		bbox_min.y = sample_pos - m_halfWidth_ruler;
		bbox_max.y = sample_pos + m_halfWidth_ruler;
	}
	else if (orien == OrienType::ORIEN_VERT) {	//v_dir
		bbox_min.x = sample_pos - m_halfWidth_ruler;
		bbox_max.x = sample_pos + m_halfWidth_ruler;
		bbox_min.y = bbox_hole_v.first - m_halfWidth_ruler;
		bbox_max.y = bbox_hole_v.second + m_halfWidth_ruler;
	}
	else {
		return output_gap;
	}
	// append point to ruler
	if (Util_Math::IsValZero(normal_plane.x)) { // xz plane
		AppendPt2Ruler2D(pts_plane, std::make_pair(bbox_min, bbox_max), CCSPlaneType::PLANE_XZ, ptID_ruler);
	}
	if (Util_Math::IsValZero(normal_plane.y)) { // yz plane
		AppendPt2Ruler2D(pts_plane, std::make_pair(bbox_min, bbox_max), CCSPlaneType::PLANE_YZ, ptID_ruler);
	}
	if (orien == OrienType::ORIEN_HORIZ)
	{
		if (ptID_ruler.size() == 0) {
			return bbox_hole_u.second - bbox_hole_u.first;
		}
		else {
			float max_gap = 0.f;
			bool isFound = false;
			if (Util_Math::IsValZero(normal_plane.x)) {
				isFound = FindMaxGapInRuler2D(pts_plane, ptID_ruler, bbox_hole_u, AxisType::Axis_X, max_gap);
			}
			if (Util_Math::IsValZero(normal_plane.y)) {
				isFound = FindMaxGapInRuler2D(pts_plane, ptID_ruler, bbox_hole_u, AxisType::Axis_Y, max_gap);
			}
			if (isFound) {
				output_gap = max_gap;
			}
		}
	}
	else if (orien == OrienType::ORIEN_VERT)
	{
		if (ptID_ruler.size() == 0) {
			return bbox_hole_v.second - bbox_hole_v.first;
		}
		else {
			float max_gap = 0.f;
			bool isFound = false;
			isFound = FindMaxGapInRuler2D(pts_plane, ptID_ruler, bbox_hole_v, AxisType::Axis_Z, max_gap);
			if (isFound) {
				output_gap = max_gap;
			}
		}
	}
	else {
		return output_gap;
	}
	ClearStdVector(ptID_ruler);
	return output_gap;
}

bool DetectionHoleClass::FindMaxGapInRuler2D(
	const std::vector<cv::Point3f> &pts,
	const std::vector<unsigned int> &pt_idx,
	const std::pair<float, float> &bounds_ruler,
	const AxisType axisType,
	float &max_gap)
{
	bool isFound = false;
	// data validation
	if (pts.size() < 1 || pt_idx.size() < 1) return isFound;
	unsigned int nidx = static_cast<unsigned int>(pt_idx.size());

	// copy coordinates along x-axis or y-axis to opencv matrix for sorting
	cv::Mat coords(nidx, 1, CV_32F);
	if (axisType == AxisType::Axis_X) {
		for (unsigned int m = 0; m < nidx; m++) {
			coords.at<float>(m, 0) = pts[pt_idx[m]].x;
		}
	}
	else if (axisType == AxisType::Axis_Y) {
		for (unsigned int m = 0; m < nidx; m++) {
			coords.at<float>(m, 0) = pts[pt_idx[m]].y;
		}
	}
	else if (axisType == AxisType::Axis_Z) {
		for (unsigned int m = 0; m < nidx; m++) {
			coords.at<float>(m, 0) = pts[pt_idx[m]].z;
		}
	}
	else {
		return isFound;
	}

	//sort coordinates and save sorted indx
	cv::Mat sorted_idx(nidx, 1, CV_32S);
	cv::sortIdx(coords, sorted_idx, CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);
	/*std::cout << "coords "<<coords << std::endl;
	std::cout << "sorted_idx " << sorted_idx << std::endl;*/

	std::vector<array<float, 2>> gaps_loc;

	if (FindGapsInRuler(coords, sorted_idx, bounds_ruler, gaps_loc))
	{
		max_gap = 0.f;
		for (int id_gap = 0; id_gap < gaps_loc.size(); id_gap++) {
			max_gap = max(max_gap, gaps_loc[id_gap][1] - gaps_loc[id_gap][0]);
		}
		isFound = true;
	}
	//RELEASE MEMORY
	sorted_idx.release();
	coords.release();
	gaps_loc.clear();
	return isFound;
}

//coords, sorted_idx, bounds_ruler, gaps_loc
bool DetectionHoleClass::FindGapsInRuler(
	const cv::Mat &coords,
	const cv::Mat &sorted_idx,
	const std::pair<float, float> &bounds_ruler,
	std::vector<array<float, 2>> &gaps_loc)
{
	gaps_loc.resize(0);
	if (coords.rows == 0) {
		gaps_loc.push_back({ bounds_ruler.first, bounds_ruler.second });
		return true;
	}
	unsigned int n_gaps = 0;
	//have gap at the begin
	m_thres_maxLen_hole = 10000;
	if ((coords.at<float>(sorted_idx.at<int>(0, 0), 0) - bounds_ruler.first) > m_thres_minLen_hole
		&& (coords.at<float>(sorted_idx.at<int>(0, 0), 0) - bounds_ruler.first) < m_thres_maxLen_hole)
	{
		gaps_loc.push_back({ bounds_ruler.first, coords.at<float>(sorted_idx.at<int>(0, 0), 0) });
		n_gaps++;
	}
	for (int i = 0; i < sorted_idx.rows - 1; i++) {
		float p1 = coords.at<float>(sorted_idx.at<int>(i, 0), 0);
		float p2 = coords.at<float>(sorted_idx.at<int>(i + 1, 0), 0);

		if (Util_Math::IsInBounds(p2 - p1, m_thres_minLen_hole, m_thres_maxLen_hole))
		{
			float temp_center = (p2 + p1)*0.5f;
			if (n_gaps == 0) {
				gaps_loc.push_back({ p1, p2 });
				n_gaps++;
			}
			else {
				if ((abs((gaps_loc[n_gaps - 1][0] + gaps_loc[n_gaps - 1][1]) * 0.5f - temp_center) > m_thres_dis_isPtOverlap)
					&& (abs(gaps_loc[n_gaps - 1][1] - p2) > m_thres_dis_isPtOverlap))
				{
					gaps_loc.push_back({ p1, p2 });
					n_gaps++;
				}
			}
			//cout << n_gaps << endl;
		}
	}
	//have hole at the end
	//have hole at the begin
	if ((-coords.at<float>(sorted_idx.at<int>(sorted_idx.rows - 1, 0), 0) + bounds_ruler.second) > m_thres_minLen_hole &&
		(-coords.at<float>(sorted_idx.at<int>(sorted_idx.rows - 1, 0), 0) + bounds_ruler.second) < m_thres_maxLen_hole)
	{
		gaps_loc.push_back({ coords.at<float>(sorted_idx.at<int>(sorted_idx.rows - 1, 0), 0), bounds_ruler.second });
		n_gaps++;
	}
	return (n_gaps > 0);
}

bool DetectionHoleClass::AppendPt2Ruler1D(
	const std::vector<cv::Point3f> &pts,
	const std::pair<float, float> &bounds,
	const AxisType axisType,
	std::vector<unsigned int> &ptID_ruler)
{
	// data validation
	if (pts.empty()) return false;
	float lower = bounds.first;
	float upper = bounds.second;
	if (lower > upper) return false;

	unsigned int npts = pts.size();
	ptID_ruler.resize(npts);
	unsigned int count = 0;

	if (axisType == AxisType::Axis_X) {
		for (unsigned int i = 0; i < npts; i++) {
			if (Util_Math::IsInBounds(pts[i].x, lower, upper)) {
				ptID_ruler[count++] = i;
			}
		}
	}
	else if (axisType == AxisType::Axis_Y) {
		for (unsigned int i = 0; i < npts; i++) {
			if (Util_Math::IsInBounds(pts[i].y, lower, upper)) {
				ptID_ruler[count++] = i;
			}
		}
	}
	else if (axisType == AxisType::Axis_Z) {
		for (unsigned int i = 0; i < npts; i++) {
			if (Util_Math::IsInBounds(pts[i].z, lower, upper)) {
				ptID_ruler[count++] = i;
			}
		}
	}
	else {}
	ptID_ruler.resize(count);
	return false;
}

bool DetectionHoleClass::AppendPt2Ruler2D(
	const std::vector<cv::Point3f> &pts,
	const std::pair<cv::Point2f, cv::Point2f> &bounds,
	const CCSPlaneType planeType,
	std::vector<unsigned int> &ptID_ruler)
{
	ptID_ruler.clear();
	// data validation
	// points number
	unsigned int npts = static_cast<unsigned int>(pts.size());
	if (npts < 1) return false;
	// bounds value
	float u_min, u_max, v_min, v_max;
	u_min = bounds.first.x;
	v_min = bounds.first.y;
	u_max = bounds.second.x;
	v_max = bounds.second.y;
	if (u_min >= u_max || v_min >= v_max) return false;
	// resize ptID_ruler
	ptID_ruler.resize(npts);
	unsigned int count = 0;
	// check plane type, deterine pts component as u or v
	// xz plane, u = x, v = z;
	// yz plane, u = y, v = z;
	// xy plane, u = x, v = y
	if (planeType == CCSPlaneType::PLANE_XZ) {
		for (unsigned int i = 0; i < npts; i++) {
			if (Util_Math::IsPtInBounds2D(pts[i].x, pts[i].z, u_min, u_max, v_min, v_max)) {
				ptID_ruler[count++] = i;
			}
		}
	}
	else if (planeType == CCSPlaneType::PLANE_YZ) {
		for (unsigned int i = 0; i < npts; i++) {
			if (Util_Math::IsPtInBounds2D(pts[i].y, pts[i].z, u_min, u_max, v_min, v_max)) {
				ptID_ruler[count++] = i;
			}
		}
	}
	else if (planeType == CCSPlaneType::PLANE_XY) {
		for (unsigned int i = 0; i < npts; i++) {
			if (Util_Math::IsPtInBounds2D(pts[i].x, pts[i].y, u_min, u_max, v_min, v_max)) {
				ptID_ruler[count++] = i;
			}
		}
	}
	else {}

	ptID_ruler.resize(count);
	return true;
}

/**
* \brief classify hole type
*/
int DetectionHoleClass::ClassifyHole(const float width, const float height)
{
	if (width <= m_thres_maxWidth_door && height >= m_thres_minHeight_door) return 1;
	return 0;
}

/**
* \brief classify hole type
*/
int DetectionHoleClass::ClassifyHole(
	const float width, const float height,
	const std::vector<cv::Point3f> &corners_hole,
	const std::vector<cv::Point3f> &corners_plane)
{
	// compare hole height and wall height
	float hole_h_low = corners_hole[0].z;
	float hole_h_high = corners_hole[2].z;
	float wall_h_low = corners_plane[0].z;
	float wall_h_high = corners_plane[2].z;
	bool isLinkLow = false, isLinkHigh = false;
	float thres = 20.f; // threshold of compare
	//cout << "hole_h_low: " << hole_h_low << "\n";
	//cout << "wall_h_low: " << wall_h_low << "\n";
	//cout << "hole_h_high: " << hole_h_high << "\n";
	//cout << "wall_h_high: " << wall_h_high << "\n";
	if (hole_h_low < wall_h_low + thres) isLinkLow = true;
	if (hole_h_high > wall_h_high - thres) isLinkHigh = true;
	// window
	if (!isLinkLow) return 0;
	// if (!isLinkLow && !isLinkHigh) return 0;
	// door
	if (isLinkLow && !isLinkHigh) {
		if (height > m_thres_minHeight_door && height < m_thres_maxHeight_door
			&&	width > m_thres_minWidth_door && width < m_thres_maxWidth_door) {
			return 1;
		}
	}
	return -1;
}

/**
* \brief classify hole type
*/
int DetectionHoleClass::ClassifyHole(
	const float width, const float height,
	const std::vector<cv::Point3f> &corners_hole,
	const float bound_up, const float bound_low)
{
	// compare hole height and wall height
	float hole_h_low = corners_hole[0].z;
	float hole_h_high = corners_hole[2].z;
	float wall_h_low = bound_low;
	float wall_h_high = bound_up;
	bool isLinkLow = false, isLinkHigh = false;
	float thres = 80.f; // threshold of compare
	//cout << "hole_h_low: " << hole_h_low << "\n";
	//cout << "wall_h_low: " << wall_h_low << "\n";
	//cout << "hole_h_high: " << hole_h_high << "\n";
	//cout << "wall_h_high: " << wall_h_high << "\n";
	if (hole_h_low < wall_h_low + thres * 2) isLinkLow = true;
	if (hole_h_high > wall_h_high - thres) isLinkHigh = true;
	// window
	if (bound_low > -600.f || (!isLinkLow && !isLinkHigh)) return 0;
	// if (!isLinkLow && !isLinkHigh) return 0;
	// door
	//if (isLinkLow && !isLinkHigh)  //change door definition
	//cout << m_thres_minHeight_door << ":1800  "<< m_thres_maxWidth_door<<": 10000 "<< endl;
	if (isLinkLow) {
		if (height > m_thres_minHeight_door && height < m_thres_maxHeight_door
			&&	width > m_thres_minWidth_door && width < m_thres_maxWidth_door) {
			return 1;
		}
	}
	//return 1;
	return -1;
}

/**
* \brief set m_width_ruler
*/
bool DetectionHoleClass::SetWidthRule(const float val) {
	if (val <= 0.f) {
		std::string msg("error: set ruler width for hole detection, val <= 0.f");
		PrintMsg(msg, msg);
		return false;
	}
	m_width_ruler = val;
	m_halfWidth_ruler = m_width_ruler * 0.5f;
	return true;
}
/**
* \brief set m_thres_minLen_hole
*/
bool DetectionHoleClass::SetThresMinLenHole(const float val) {
	if (val <= 0.f) {
		std::string msg("error: set threshold of minimum length of hole, val <= 0.f");
		PrintMsg(msg, msg);
		return false;
	}
	m_thres_minLen_hole = val;
	return true;
}
/**
* \brief set m_thres_maxLen_hole
*/
bool DetectionHoleClass::SetThresMaxLenHole(const float val) {
	if (val <= 0.f) {
		std::string msg("error: set threshold of maximum length of hole, val <= 0.f");
		PrintMsg(msg, msg);
		return false;
	}
	m_thres_maxLen_hole = val;
	return true;
}
/**
* \brief set m_thres_minWidth_door
*/
bool DetectionHoleClass::SetThresMinWidthDoor(const float val)
{
	if (val <= 0.f) {
		std::string msg("error: set threshold of minimum width of door, val <= 0.f");
		PrintMsg(msg, msg);
		return false;
	}
	m_thres_minWidth_door = val;
	return true;
}
/**
* \brief set m_thres_maxWidth_door
*/
bool DetectionHoleClass::SetThresMaxWidthDoor(const float val) {
	if (val <= 0.f) {
		std::string msg("error: set threshold of maximum width of door, val <= 0.f");
		PrintMsg(msg, msg);
		return false;
	}
	m_thres_maxWidth_door = val;
	return true;
}
/**
* \brief set m_thres_minHeight_door
*/
bool DetectionHoleClass::SetThresMinHeightDoor(const float val) {
	if (val <= 0.f) {
		std::string msg("error: set threshold of minimum height of door, val <= 0.f");
		PrintMsg(msg, msg);
		return false;
	}
	m_thres_minHeight_door = val;
	return true;
}
/**
* \brief set m_thres_maxHeight_door
*/
bool DetectionHoleClass::SetThresMaxHeightDoor(const float val) {
	if (val <= 0.f) {
		std::string msg("error: set threshold of maximum height of door, val <= 0.f");
		PrintMsg(msg, msg);
		return false;
	}
	m_thres_maxHeight_door = val;
	return true;
}
/**
* \brief set m_thres_dis_isPtOverlap
*/
bool DetectionHoleClass::SetThresDisIsPtOverlap(const float val) {
	if (val <= 0.f) return false;
	m_thres_dis_isPtOverlap = val;
	return true;
}

/**
* \brief set m_sample_cstr_len
*/
bool DetectionHoleClass::SetSampleCstrLen(const float val) {
	if (val <= 0.f) return false;
	m_sample_cstr_len = val;
	return true;
}
/**
* \brief set m_sample_margin
*/
bool DetectionHoleClass::SetSampleMargin(const float val) {
	if (val <= 0.f) return false;
	m_sample_margin = val;
	return true;
}
/**
* \brief set m_thres_dis_isPtOverlap
*/
bool DetectionHoleClass::SetSampleInfo(const float cstr_len, const float margin) {
	// data validation
	if (cstr_len < 0.f || margin < 0.f) {
		std::string msg("error: set sampling info | cstr_len or margin < 0.f\n");
		PrintMsg(msg, msg);
		return false;
	}
	if (cstr_len < (1.8 * margin)) {
		std::string msg("error: set sampling info | cstr_len < 2 * margin");
		PrintMsg(msg, msg);
		return false;
	}
	m_sample_cstr_len = cstr_len;
	m_sample_margin = margin;
	return true;
}

/**
* \brief get m_width_ruler
*/
float DetectionHoleClass::GetWidthRuler() const {
	return m_width_ruler;
}
/**
* \brief get m_thres_minLen_hole
*/
float DetectionHoleClass::GetThresMinLenHole() const {
	return m_thres_minLen_hole;
}
/**
* \brief get m_thres_maxLen_hole
*/
float DetectionHoleClass::GetThresMaxLenHole()const {
	return m_thres_maxLen_hole;
}
/**
* \brief get m_thres_minWidth_door
*/
float DetectionHoleClass::GetThresMinWidthDoor() const {
	return m_thres_minWidth_door;
}
/**
* \brief get m_thres_maxWidth_door
*/
float DetectionHoleClass::GetThresMaxWidthDoor()const {
	return m_thres_maxWidth_door;
}
/**
* \brief get m_thres_minHeight_door
*/
float DetectionHoleClass::GetThresMinHeightDoor()const {
	return m_thres_minHeight_door;
}
/**
* \brief get m_thres_maxHeight_door
*/
float DetectionHoleClass::GetThresMaxHeightDoor() const {
	return m_thres_maxHeight_door;
}
/**
* \brief get m_thres_dis_isPtOverlap
*/
float DetectionHoleClass::GetThresDisIsPtOverlap() const {
	return m_thres_dis_isPtOverlap;
}