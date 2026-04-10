#ifndef _BIM_LOC_ROI_H_
#define _BIM_LOC_ROI_H_
#include <vector>
#include <utility>
#include <opencv2\core.hpp>
#include "../doc/doc_bim.h"
#include "../topology/topo_graph.h"
#include "bbox.h"
/**
* \brief class of BIM locate ROI
*/
class BIM_LOC_ROI
{
public:
	/**
	* \brief constructor
	*/
	BIM_LOC_ROI();

	BIM_LOC_ROI(int type_roi);
	/**
	* \brief destructor
	*/
	~BIM_LOC_ROI();
	/**
	* \brief clear
	*/
	void clear();
	/**
	* \brief update plane index list
	*/
	void updateIdxList(Doc_BIM &doc_bim);
	/**
	* \brief locate roi on image
	*/
	void createIntxnGraph(
		const std::vector<Doc_BIM_Plane *> &docs,
		const std::vector<unsigned int> &idx_proc,
		Topo_Graph *graph);
	//added by yu.liang@unre.com
	//2020/12/15
// 	void createIntxnGraph_Wall(
// 		const std::vector<Doc_BIM_Plane *> &docs,
// 		Topo_Graph *graph,
// 		std::vector<std::pair<unsigned int, unsigned int>>& pairIntxn);
	/**
	* \brief locate roi on image by ceiling
	*/
	bool locROI_img_0(Doc_BIM &doc_bim, std::vector<std::vector<cv::Point3f>> &pts, cv::Mat &img_roi);
	/**
	* \brief locate roi on image only by wall
	// changed by yu.liang@unre.com 2021/1/4
	*/
	bool locROI_img_1(Doc_BIM &doc_bim, std::vector<std::vector<cv::Point3f>> &pts, cv::Mat &img_roi);
	/**
	* \brief locate roi on image by single ceiling and beam
	*/
	bool locROI_img_5(Doc_BIM &doc_bim, std::vector<std::vector<cv::Point3f>> &pts, cv::Mat &img_roi);

	bool locROI_img_6(Doc_BIM& doc_bim, std::vector<std::vector<cv::Point3f>>& pts, cv::Mat& img_roi,int station_size, bool cut_poly);

	/**
	* \brief cull models based on img_roi
	* \ yu.liang
	*/
	void locROI_cull(Doc_BIM &doc_bim,
		PlaneCutResultInterface& cutResult,
		const std::vector<cv::Point3f> & scene_plane_normals,
		const std::vector<cv::Point3f> & scene_plane_centers,
		const cv::Mat &img_roi);

	bool get_valid() { return m_isValid; }
	/**
	* \brief update the type of planes
	* \ yu.liang
	* \ 2021/1/4
	*/
	void updatePlaneType(Doc_BIM &doc_bim, PlaneCutResultInterface& cutResult);
	/**
	* \brief filter tiny plane
	* \added by yu.liang@unre.com
	* \2021/1/11
	* find tiny plane and remove it
	*/
	void removeTinyPlane(Doc_BIM &doc_bim,
		std::vector<std::vector<cv::Point3f>> &pts,
		const std::vector<cv::Point3f> &normals,
		std::vector<std::vector<unsigned char>> & scene_plane_reflect,
		std::vector<uint>& wall_idx_need_to_keep);
	void printPlaneType(Doc_BIM & doc_bim);

	void computeWallList(Doc_BIM &doc_bim,
		std::vector<std::vector<cv::Point3f>> &pts,
		const std::vector<cv::Point3f> &normals,
		std::vector<std::pair<int, std::vector<float>>> &wall_list);

	std::vector<unsigned int> getWallIdx() { return m_idx_wall; };

	void setWithInclinedPlane(bool withInclinedPlane) { m_withInclinedPlane = withInclinedPlane; };

	bool isCompleteRoom(Doc_BIM &doc_bim, std::vector<std::vector<cv::Point3f>> &pts, 
		std::vector<int>& wall_idx, std::vector<int>& wall_idx_to_skip);

	void RemoveNoiseWall(Doc_BIM& doc_bim, std::vector<std::vector<cv::Point3f>>& pts);

private:
	std::vector<unsigned int> m_idx_all;	/**< indices of all planes */
	std::vector<unsigned int> m_idx_valid;	/**< indices of valid planes */
	std::vector<unsigned int> m_idx_wall;	/**< indices of wall planes */
	std::vector<unsigned int> m_idx_floor;	/**< indices of floor planes */
	std::vector<unsigned int> m_idx_hCeil;	/**< indices of horizontal ceiling planes */
	std::vector<unsigned int> m_idx_beam;	/**< indices of beam planes */
	std::vector<unsigned int> m_idx_roomwall; /**< indices of room wall planes which can be seen direct at ori point*/
	bool m_isValid = true;
	bool m_withInclinedPlane = true;
	int	m_type_roi;
public:
	BBox3D *m_bbox_g;
};

#endif // !_BIM_LOC_ROI_H_
