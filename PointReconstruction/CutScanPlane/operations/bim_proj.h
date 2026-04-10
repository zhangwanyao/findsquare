#ifndef _BIM_PROJ_H_
#define _BIM_PROJ_H_
#include<vector>
#include<opencv2\core.hpp>
#include "../doc/doc_bim.h"
#include"bbox.h"
/**
* \brief class of BIM projection
*/

class BIM_Proj
{
public:
	/**
	* \brief constructor
	*/
	BIM_Proj();
	/**
	* \brief constructor
	*/
	BIM_Proj(const std::vector<std::vector<cv::Point3f>> &pts,
		const std::vector<unsigned int>& m_idx_valid, int type_roi);
	/**
	* \brief destructor
	*/
	~BIM_Proj();
	/**
	* \brief project locally
	*/
	void proj_local(Doc_BIM &doc_bim, std::vector<std::vector<cv::Point3f>> &pts);
	/**
	* \brief project globally
	*/
	void proj_global(Doc_BIM &doc_bim, std::vector<std::vector<cv::Point3f>> &pts, cv::Mat &img_g);
	/**
	* \brief project globally
	* \param idx_area index list of models to get area/bbox for projection
	* \param idx_proj index list of models to project
	*/
	void proj_global(
		Doc_BIM &doc_bim, std::vector<std::vector<cv::Point3f>> &pts,
		const std::vector<unsigned int> &idx_area,
		const std::vector<unsigned int> &idx_proj, cv::Mat &img_g);
	/**
	* \brief project globally
	* \param idx_area index list of models to get area/bbox for projection
	* \param idx_proj index list of models to project
	//added by yu.liang@unre.com
		//2020/12/15
	*/
	bool proj_global_wall_optimize(
		Doc_BIM &doc_bim, std::vector<std::vector<cv::Point3f>> &pts,
		const std::vector<unsigned int> &idx_area,
		std::vector<unsigned int> &idx_wall,
		std::vector<unsigned int> &idx_roomwall,
		const std::vector<unsigned int> &idx_beam,
		cv::Mat &img_g);

	bool proj_global_wall_beam_optimize(
		Doc_BIM &doc_bim, std::vector<std::vector<cv::Point3f>> &pts,
		const std::vector<unsigned int> &idx_area,
		std::vector<unsigned int> &idx_wall,
		const std::vector<unsigned int> &idx_beam,
		cv::Mat &img_g);

	/**
	* \brief project globally
	* \param idx_area index list of models to get area/bbox for projection
	* \param idx_proj index list of models to project
	//added by yu.liang@unre.com
		//2020/12/15
	*/
	void proj_global_ceil_optimize(
		Doc_BIM &doc_bim, std::vector<std::vector<cv::Point3f>> &pts,
		const std::vector<unsigned int> &idx_area,
		std::vector<unsigned int> &idx_ceiling, const cv::Mat &img_wall, cv::Mat &img_g);

	bool proj_global_multiceiling_optimize(Doc_BIM & doc_bim, std::vector<std::vector<cv::Point3f>>& pts, const std::vector<unsigned int>& idx_area, std::vector<unsigned int>& idx_ceil, cv::Mat & img_g);

	/**
	* \brief project globally
	* \param idx_area index list of models to get area/bbox for projection
	* \param idx_proj index list of models to project
	//added by yu.liang@unre.com
		//2020/12/15
	*/
	void proj_global_floor_optimize(
		Doc_BIM &doc_bim, std::vector<std::vector<cv::Point3f>> &pts,
		const std::vector<unsigned int> &idx_area,
		std::vector<unsigned int> &idx_floor, const cv::Mat &img_floor, cv::Mat &img_g);

	/**
	* \brief project globally of single model
	*/
	void proj_global_single(const std::vector<cv::Point3f> &pts, const BBox3D &bbox_g,
		const unsigned int extent_x, const unsigned int extent_y, const float vl,
		std::vector<cv::Point2i> &ptVx, cv::Mat &img);

	bool proj_global_ceil_beam_optimize(
		Doc_BIM &doc_bim, std::vector<std::vector<cv::Point3f>> &pts,
		const std::vector<unsigned int> &idx_area,
		const std::vector<unsigned int> &idx_ceiling,
		const std::vector<unsigned int> &idx_beam,
		cv::Mat &img_g);

	int m_type_roi;
public:
	BBox3D *m_bbox_g;
};

#endif // !_BIM_PROJ_H_
