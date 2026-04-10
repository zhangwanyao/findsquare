#ifndef  DETECTIONHOLE_H

#include <iostream>
#include <vector>
#include <opencv/cv.hpp> //current opencv version: 3.4.1
#include "decn_tree.h"

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <map>

#define  DETECTIONHOLE_H
#define _USE_MATH_DEFINES
//#define DELIVER_TO_CLIENTS
using namespace std;
/**
* \brief class of detect hole
*
* \author Meng Chen, Bichen JING
*/
class  DetectionHoleClass {
protected:
	/**
	* \brief Cartesian coordinate plane type
	*/
	enum class CCSPlaneType
	{
		PLANE_XY,	/**< xy plane */
		PLANE_XZ,	/**< xz plane */
		PLANE_YZ,	/**< yz plane */
		PLANE_ARB	/**< arbitrary plane */
	};
	/**
	* \brief axis type
	*/
	enum class AxisType
	{
		Axis_X,
		Axis_Y,
		Axis_Z
	};
	/**
	* \brief orientation type
	*/
	enum class OrienType
	{
		ORIEN_HORIZ,
		ORIEN_VERT
	};
	typedef struct Bbox
	{
		float x;
		float y;
		float w;
		float h;
		float score;
	}Bbox;

public:
	/**
	* \brief constructor
	*/
	DetectionHoleClass();
	/**
	* \brief destructor
	*/
	virtual ~DetectionHoleClass();
	/**
	* \brief initialize variables
	*/
	virtual void Init();
	/**
	* \brief clear instance data
	*/
	virtual void Clear();
	/**
	* \brief main function of detecting holes on a plane, find positions of multiple window/door
	* \param[in] pts_plane plane point cloud, xyz coordinates
	* \param[in] normal_plane[3] normal of plane
	* \param[in] corners_plane four bounding vertices of plane, plane containing hole may divided, find real plane by considering nearby plane, >= original plane bounding
	* \param[out] corners_hole a vector of hole positions (4 vertices, namely vector<vector>)
	* \param[out] raw_w_h rough value of hole, it may be from CAD or scene, used for allocation and flatness compute
	* \param[out] type_hole vector has same size as pos_hole, value 1 means door 0 means window, -1 means no doors or windows
	* \param[in] isRemeasure this function is a rough estimation, if true, need to precisely compute height and width
	* \param[out] output_w_h isRemeasure true, output precise data, come from scene data, different from width_height
	*/
	/*bool DetectionHoleClassFcn(
		const std::vector<cv::Point3f> &pts_plane,
		const float normal_plane[3],
		const std::vector<cv::Point3f> &corners_plane,
		std::vector<std::vector<cv::Point3f>> &corners_hole,
		std::vector<std::array<float, 2>> &raw_w_h,
		std::vector<int> &type_hole,
		const bool isRemeasure,
		std::vector<array<float, 4>> &output_w_h);*/

	bool DetectionHoleClassFcn(
		int i,
		const std::vector<cv::Point3f> &pts_plane,
		const float normal_plane[3],
		const std::vector<cv::Point3f> &corners_plane,
		const float walltopZ,
		std::vector<std::vector<cv::Point3f>> &corners_hole,
		std::vector<std::array<float, 2>> &raw_w_h,
		std::vector<int> &type_hole,
		const bool isRemeasure,
		std::vector<array<float, 4>> &output_w_h);

	bool DetectionHoleClassFcn(
		const std::vector <std::vector<cv::Point3f>>& pts_planes,
		const std::vector<std::vector<int>>& scene_plane_reflect,
		const float normal_plane[3],
		const std::vector<cv::Point3f>& corners_plane,
		const float walltopZ,
		const std::vector<cv::Point3f> &ground_points,
		const std::vector<double>& planesMeanStd,
		const  std::vector<cv::Point3f> & windowPoints,
		std::vector<std::vector<cv::Point3f>>& corners_hole,
		std::vector<std::array<float, 2>>& raw_w_h,
		std::vector<int>& type_hole,
		const bool isRemeasure,
		std::vector<array<float, 4>>& output_w_h, const int curid, const map<int, vector<cv::Point3f>> &curWithNeigh, const vector<cv::Point3f> & wallBottomedges, const int k);

	bool checkHoleEdge(const std::vector<cv::Point3f> &pts_plane,
		const cv::Point3f normal_plane, const float* bbox_plane, Bbox &hole);

	bool calMinZFromRulerPointCloud(const std::vector<cv::Point3f> &pts_plane,
		const cv::Point3f normal_plane, const std::vector<cv::Point3f> &ruler_corner, float & minZ);

	bool getRefTopY(const std::vector<cv::Point3f>& pts_plane, const std::vector<float>& corner, float& refY);

	void adjustHoleY(const std::vector<cv::Point3f>& pts_plane, const float bottomz, const float topz, const float leftx,
		const float rightx, float* center_v, int* direction);
	void adjustHoleX(const std::vector<cv::Point3f>& pts_plane, const float bottomz, const float topz, const float leftx,
		const float rightx, float* center_v, int* direction, const int* num_first);
	void lineApproaching(const std::vector<cv::Point3f>& pts_plane, const float bottomz, const float topz, const float leftx,
		const float rightx, float* center_v, int* direction, int* num_first);
	bool sparseOrDenseAlongLineDirection(const std::vector<cv::Point3f>& pts_plane, const float setPrecent, const float refline);
	void updateLeftRightInitPos(const std::vector<cv::Point3f>& pts_plane, const float rightx, const float leftx, const bool frontY, float *center_v);
	void checkHoleDepthPts(const std::vector<cv::Point3f>& pts_plane_copy, const cv::Point3f& normal_plane, const  std::vector<cv::Point3f>& windowPoints, const std::vector<int>& allRemovedReflect, const std::vector<cv::Point3f>& allRemovedPts_rot,
		const std::vector<int>& reflect_plane, const array<float, 2>& raw_loc_w, const array<float, 2>& raw_loc_h, const float bottomz, const float topz, const double frontY,
		const std::vector<double>& planeMeanStd, const float leftx, const float rightx, bool& calCornerWall2, bool& calCornerWall3, float& meanx_2, float& meanx_3);
	bool edgeFitting(const std::vector<cv::Point3f>& pts_plane, const std::vector<int>& reflect_plane, const  std::vector<cv::Point3f>& windowPoints, const std::vector<cv::Point3f>& allRemovedPts_rot, const std::vector<int>& removedReflect, const std::vector<cv::Point3f>& ground_points, const cv::Point3f normal_plane, const float* bbox_plane, const std::vector<double>& planeMeanStd, std::vector<array<float, 2>>& raw_loc_w,
		std::vector<array<float, 2>>& raw_loc_h, const std::vector<cv::Point3f> neighpoints, const cv::Point3f& bottompt);
	bool edgeFitting(const std::vector<cv::Point3f>& pts_plane, const cv::Point3f normal_plane, const float* bbox_plane, Bbox& hole);
protected:
	/**
	* \brief init decision tree
	*/
	void Init_DecnTree();
	bool DetectHole3(const std::vector<cv::Point3f>& pts_plane, const cv::Point3f normal_plane,
		const std::vector<cv::Point3f>& corners_plane, std::vector<std::vector<cv::Point3f>>& corners_hole,
		std::vector<array<float, 2>>& raw_w_h, const int pidx, std::vector<array<float, 4>>& output_w_h);
	/**
	* \brief find holes in each plane
	*/
	bool DetectHole(
		const std::vector<cv::Point3f>  &pts_plane,
		const cv::Point3f normal_plane,
		const std::vector<cv::Point3f> &corners_plane,
		std::vector<std::vector<cv::Point3f>> &corners_hole,
		std::vector<array<float, 2>> &raw_w_h,
		const bool isRemeasure,
		std::vector<array<float, 4>> &output_w_h);
	/**
	* \brief find holes in each plane
	*/
	bool DetectHole(
		const std::vector<cv::Point3f>& pts_plane,
		const cv::Point3f normal_plane,
		const std::vector<int> &reflect_plane,
		const std::vector<cv::Point3f>& corners_plane,
		const std::vector<cv::Point3f>& ground_points,
		const std::vector<double>& planeMeanStd,
		const  std::vector<cv::Point3f>& windowPoints,
		std::vector<std::vector<cv::Point3f>>& corners_hole,
		std::vector<array<float, 2>>& raw_w_h,
		const bool isRemeasure,
		std::vector<array<float, 4>>& output_w_h, const vector<cv::Point3f>& neighpoints, const cv::Point3f& bottompt);
	bool LocateHoleRot_corner(const float * bbox_plane, const CCSPlaneType & planeType, 
		const std::pair<float, float>& bound_w, const std::pair<float, float>& bound_h, 
		const float angle_rotZ, std::vector<cv::Point3f>& corners, std::vector<cv::Point3f> &rot_corners);
	/**
	* \brief locate hole corner in original coord system
	*/
	bool LocateHole_corner(
		const float *bbox_plane,
		const CCSPlaneType &planeType,
		const std::pair<float, float> &bound_w,
		const std::pair<float, float> &bound_h,
		const float angle_rotZ,
		std::vector<cv::Point3f> &corners);

	/**
	* \brief get hole width by seaching in three bands of each plane
	*
	* call find several big gap
	* call get hole width before get hole height
	*/
	virtual bool LocateHole_w(
		const std::vector<cv::Point3f> &pts_plane,
		const cv::Point3f normal_plane,
		const float *bbox_plane,
		std::vector<array<float, 2>> &raw_loc_w);
	/**
	* \brief Get hole height by seaching in one band of each plane
	*
	* call find several big gap
	* call this func after get hole, namely find horizontal gap firstly
	*/
	virtual bool LocateHole_h(
		const std::vector<cv::Point3f> &pts_plane,
		const cv::Point3f normal_plane,
		const float *bbox_plane,
		std::vector<array<float, 2>> &raw_loc_w,
		std::vector<array<float, 2>> &raw_loc_h);

	/**
	* \brief remeasure holes
	* \param[in] pts_plane plane point cloud after rotation
	* \param[in] normal_plane[3] normal after rotation to a plane parallel to a coordinate frame plane, x-z or y-z
	* \param[in] bbox_plane bounding box of plane, 6 elements: in order of min and max of x, y, z
	* \param[in] raw_loc_w raw location of hole width (x or y according to rotation to which plane), [0] is minimum value, [1] is maximum value
	* \param[in] raw_loc_h raw location of hole height (z value), [0] is minimum value, [1] is maximum value
	*/
	bool Measure(
		const std::vector<cv::Point3f> &pts_plane,
		const cv::Point3f normal_plane,
		const float *bbox_plane,
		const std::vector<array<float, 2>> &raw_loc_w,
		const std::vector<array<float, 2>> &raw_loc_h,
		std::vector<array<float, 4>> &output_w_h);

	/**
	* \brief remeasure single hole
	*/
	bool MeasureHole(
		const std::vector<cv::Point3f> &pts_plane,
		const cv::Point3f normal_plane,
		const float *bbox_plane,
		const array<float, 2> &raw_loc_w,
		const array<float, 2> &raw_loc_h,
		array<float, 4> &output_w_h
	);

	/**
	* \brief remeasure single ruler
	*/
	float MeasureRuler(
		const std::vector<cv::Point3f> &pts_plane,
		const cv::Point3f &normal_plane,
		const std::pair<float, float> &bbox_hole_u,
		const std::pair<float, float> &bbox_hole_v,
		const OrienType orien,
		const float sample_pos
	);
	/**
	* \brief sample in 1d
	*/
	bool Sample1D(const float lower, const float upper, const float margin_lower,
		const float margin_upper, std::vector<float> &output);
	/**
	* \brief sample in 1d
	*/
	bool Sample1D(const float lower, const float upper, const float margin_lower,
		const float margin_upper, const unsigned int nSample, std::vector<float> &output);
	/**
	* \brief sample in 1d
	*/
	bool Sample1D(const float lower, const float upper, const float margin_lower,
		const float margin_upper, const bool isSymmetric, const std::vector<float> &intv_len,
		std::vector<float> &output);
	/**
	* \brief find points in ruler and append indices
	*
	* points defined in 3D, using 1D constraints, namely need to project points to 1D,
	*
	* \param[in] pts point coordinates
	* \param[in] bounds bounds of ruler, pair<bbox_min, bbox_max>, pair<(u_min, v_min), (u_max, v_max)>
	* \param[in] axisType use which Cartesian coordinate to check point in bounds or not
	* \param[out] ptID_ruler point indices in ruler
	*/
	bool AppendPt2Ruler1D(const std::vector<cv::Point3f> &pts,
		const std::pair<float, float> &bounds,
		const AxisType axisType,
		std::vector<unsigned int> &ptID_ruler);

	/**
	* \brief find points in ruler and append indices
	*
	* points defined in 3D, using 2D constraints, namely need to project points to 2D,
	* 2D coordinate system u-v, u is horizontal direction, v is vertical direction,
	* for xz plane, u = x, v = z;
	* for yz plane, u = y, v = z;
	* for xy plane, u = x, v = y
	*
	* \param[in] pts point coordinates
	* \param[in] bounds bounds of ruler, pair<bbox_min, bbox_max>, pair<(u_min, v_min), (u_max, v_max)>
	* \param[in] planeType use which Cartesian coordinate plane as projection plane
	* \param[out] ptID_ruler point indices in ruler
	*/
	bool AppendPt2Ruler2D(const std::vector<cv::Point3f> &pts,
		const std::pair<cv::Point2f, cv::Point2f> &bounds,
		const CCSPlaneType planeType,
		std::vector<unsigned int> &ptID_ruler);
	/**
	* \brief compute largest gap in a ruler (2D)
	* \param[in] pts all points of point cloud
	* \param[in] pt_idx indices of points in ruler
	* \param[in] bounds_ruler bounds value of ruler
	* \param[in] axisType use which coordinate of points for computing
	* \param[out] max_gap output of max_gap if return true
	*/
	bool FindMaxGapInRuler2D(const std::vector<cv::Point3f> &pts,
		const std::vector<unsigned int> &pt_idx,
		const std::pair<float, float> &bounds_ruler,
		const AxisType axisType,
		float &max_gap);
	/**
	* \brief find big holes
	*
	* if there are multiple hole in one plane, cut horizontally, there will be several holes (gap: width of hole)
	*
	* \param[out] gaps_loc the min and max value of each gap location(after rotating to xoz plane)
	*/
	virtual bool FindGapsInRuler(
		const cv::Mat &coords,
		const cv::Mat &sorted_idx,
		const std::pair<float, float> &bounds_ruler,
		std::vector<array<float, 2>> & gaps_loc);
	/**
	* \brief sum intervals/gaps
	*/
	template<typename T>
	inline T SumIntvls(const std::vector<array<T, 2>> &intvls) {
		T sum = 0.0;
		size_t n_intvl = intvls.size();
		for (size_t i = 0; i < n_intvl; i++) {
			sum += std::fabs(intvls[i][1] - intvls[i][0]);
		}
		return sum;
	}
	/*
	*\brief compare corner box size
	*/
	friend bool sort_score(const Bbox &box1, const Bbox & box2);
	/*
	* brief intersection/union rate between two boxes.
	*/
	float iou(Bbox box1, Bbox box2);
	/*
	* brief Non Maximum Suppression remove duplicate box
	*/
	void nms(vector<Bbox>&vec_boxs, float threshold, vector<Bbox>& results);
	/**
	* \brief classify hole type
	*/
	int ClassifyHole(const float width, const float height);
	/**
	* \brief classify hole type
	*/
	int ClassifyHole(const float width, const float height,
		const std::vector<cv::Point3f> &corners_hole, const std::vector<cv::Point3f> &corners_plane);
	/**
	* \brief classify hole type
	*/
	int ClassifyHole(
		const float width, const float height,
		const std::vector<cv::Point3f> &corners_hole,
		const float bound_up, const float bound_low);
	/**
	* \brief print message
	*/
	inline void PrintMsg(const std::string &msg2client, const std::string &msg2dbg)
	{
#ifdef DELIVER_TO_CLIENTS
		//throw err_msg = "err0-1: empty input";
		std::cout << msg2client << std::endl;
		log_error(msg2client.c_str());
#else
		throw err_msg = msg2dbg;
#endif
	}
	/**
	* \brief export points for debug
	*/
	inline void ExportPts(const std::string &url, const std::vector<cv::Point3f> &pts) {
		ofstream file(url, 'w');
		size_t npts = pts.size();
		for (size_t k = 0; k < npts; k++) {
			file << pts[k].x << "\t" << pts[k].y << "\t" << pts[k].z << std::endl;
		}
	}
	/**
	* \brief export points and reflect for debug
	*/
	inline void ExportPts(const std::string& url, const std::vector<cv::Point3f>& pts, const std::vector<int>& reflects) {
		ofstream file(url, 'w');
		size_t npts = pts.size();
		for (size_t k = 0; k < npts; k++) {
			file << "v " << pts[k].x << "\t" << pts[k].y << "\t" << pts[k].z << "\t" << static_cast<float>(reflects[k]) << "\t" << static_cast<float>(reflects[k]) << "\t" << static_cast<float>(reflects[k]) << std::endl;
		}
	}
	/**
	* \brief export points for debug
	*/
	inline void ExportPts(const std::string &url, const std::vector<cv::Point3f> &pts,
		const std::vector<unsigned int> &ptIDs, const cv::Mat &sorted_ptID_map)
	{
		if (ptIDs.size() != static_cast<size_t>(sorted_ptID_map.rows)) return;
		size_t npts = ptIDs.size();
		std::vector<cv::Point3f> output;
		output.resize(ptIDs.size());
		for (size_t i = 0; i < ptIDs.size(); i++) {
			output[i] = pts[ptIDs[sorted_ptID_map.at<int>(i, 0)]];
		}
		ExportPts(url, output);
		output.clear();
	}

public:
	/**
	* \brief set m_width_ruler
	*/
	bool SetWidthRule(const float val);
	/**
	* \brief set m_thres_minLen_hole
	*/
	bool SetThresMinLenHole(const float val);
	/**
	* \brief set m_thres_maxLen_hole
	*/
	bool SetThresMaxLenHole(const float val);
	/**
	* \brief set m_thres_minWidth_door
	*/
	bool SetThresMinWidthDoor(const float val);
	/**
	* \brief set m_thres_maxWidth_door
	*/
	bool SetThresMaxWidthDoor(const float val);
	/**
	* \brief set m_thres_minHeight_door
	*/
	bool SetThresMinHeightDoor(const float val);
	/**
	* \brief set m_thres_maxHeight_door
	*/
	bool SetThresMaxHeightDoor(const float val);
	/**
	* \brief set m_thres_dis_isPtOverlap
	*/
	bool SetThresDisIsPtOverlap(const float val);
	/**
	* \brief set m_sample_cstr_len
	*/
	bool SetSampleCstrLen(const float val);
	/**
	* \brief set m_sample_margin
	*/
	bool SetSampleMargin(const float val);
	/**
	* \brief set m_thres_dis_isPtOverlap
	*/
	bool SetSampleInfo(const float cstr_len, const float margin);
	/**
	* \brief get m_width_ruler
	*/
	float GetWidthRuler() const;
	/**
	* \brief get m_thres_minLen_hole
	*/
	float GetThresMinLenHole() const;
	/**
	* \brief get m_thres_maxLen_hole
	*/
	float GetThresMaxLenHole() const;
	/**
	* \brief get m_thres_minWidth_door
	*/
	float GetThresMinWidthDoor() const;
	/**
	* \brief get m_thres_maxWidth_door
	*/
	float GetThresMaxWidthDoor() const;
	/**
	* \brief get m_thres_minHeight_door
	*/
	float GetThresMinHeightDoor() const;
	/**
	* \brief get m_thres_maxHeight_door
	*/
	float GetThresMaxHeightDoor() const;
	/**
	* \brief get m_thres_dis_isPtOverlap
	*/
	float GetThresDisIsPtOverlap() const;

protected:
	std::string err_msg;			/**< error message */
	float m_width_ruler;			/**< width of ruler for remeasure width and height */
	float m_halfWidth_ruler;		/**< half width of ruler for remeasure width and height */
	float m_thres_minLen_hole;		/**< threshold of minimum length of hole for judgement */
	float m_thres_maxLen_hole;		/**< threshold of maximum length of hole for judgement */
	float m_thres_minWidth_door;	/**< threshold of minimum width of door to distinguish door and window */
	float m_thres_maxWidth_door;	/**< threshold of maximum width of door to distinguish door and window */
	float m_thres_minHeight_door;	/**< threshold of minimum height of door to distinguish door and window */
	float m_thres_maxHeight_door;	/**< threshold of maximum height of door to distinguish door and window */
	float m_thres_dis_isPtOverlap;	/**< threshold of distance to judge if points overlap */
	float m_sample_cstr_len;		/**< sample constraint length */
	float m_sample_margin;			/**< sample margin */
	Decn_Tree *m_decn_tree;			/**< decision tree */
};
#endif  DETECTIONHOLE_H
