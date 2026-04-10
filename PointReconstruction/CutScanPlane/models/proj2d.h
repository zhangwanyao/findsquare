#ifndef _PROJ2D_H_
#define _PROJ2D_H_

#include "voxel2d.h"
#include <vector>
#include <opencv2\opencv.hpp>
//#include "../config_dbg.h"

/**
* \brief projection on 2d
* \author Bichen JING
*/
class Proj2D {
public:
	/**
	* \brief constructor
	*/
	Proj2D();
	/**
	* \brief destructor
	*/
	~Proj2D();
	/**
	* \brief init
	*/
	bool init();
	/**
	* \brief clear
	*/
	void clear();
	/**
	* \brief clear voxelgrid
	*/
	void clearVoxelGrid();
	/**
	* \brief process
	*/
	void process(const std::vector<cv::Point3f> &pts_orig, cv::Mat &img);
	/**
	* \brief process
	* \param[out] ptVx 2d voxel index of point
	*/
	void process(const std::vector<cv::Point3f> &pts_orig, std::vector<cv::Point2i> &ptVx, cv::Mat &img);
	/**
	* \brief process
	* \param isLocal if true, need to compute projection matrix, translation vectors
	*/
	void process(const std::vector<cv::Point3f> &pts_orig, const bool isLocal, cv::Mat &img);
	/**
	* \brief set trans 3d
	*/
	void setTrans3D(const cv::Point3f trans_3d);
	/**
	* \brief get trans 3d
	*/
	cv::Point3f getTrans3D() const;
	/**
	* \brief set trans 2d
	*/
	void setTrans2D(const cv::Point2f trans_2d);
	/**
	* \brief get trans 2d
	*/
	cv::Point2f getTrans2D() const;
	/**
	* \brief set projection matrix
	*/
	void setProj(const cv::Mat proj);
	/**
	* \brief get projection matrix
	*/
	cv::Mat getProj() const;
	/**
	* \brief set inverse projection matrix
	*/
	void setInvProj(const cv::Mat inv_proj);
	/**
	* \brief get inverse matrix
	*/
	cv::Mat getInvProj() const;
	/**
	* \brief set length of voxel
	*/
	bool setLenVoxel(const float len_voxel);
	/**
	* \brief get length of voxel
	*/
	float getLenVoxel() const;
	/**
	* \brief set voxelgrid extent of x direction
	*/
	bool setExtentX(const unsigned int extent_x);
	/**
	* \brief get voxelgrid extent of x direction
	*/
	float getExtentX() const;
	/**
	* \brief set voxelgrid extent of y direction
	*/
	bool setExtentY(const unsigned int extent_y);
	/**
	* \brief get voxelgrid extent of y direction
	*/
	float getExtentY() const;
	/**
	* \brief get area of occupied region, rough
	*/
	float getAreaOccupied() const;
	/**
	* \brief compute PCA normal, direction along the eigen vector of minimal eigen value
	*/
	static cv::Point3f getPCANormal(const std::vector<cv::Point3f> &pts);
private:
	cv::Point3f m_trans_3d;		/**< translation vector of 3d */
	cv::Mat m_proj;				/**< projection matrix from 3d to 2d */
	cv::Point2f m_trans_2d;		/**< translation vector of 2d */
	cv::Mat m_inv_proj;			/**< projection matrix from 2d to 3d */
	
	VoxelGrid2D* m_voxelgrid;	/**< 2d voxelgrid */
	float m_len_voxel;			/**< voxel length */
	unsigned int m_extent_x;	/**< extent x */
	unsigned int m_extent_y;	/**< extent y */
};

#endif // !_PROJ2D_H_
