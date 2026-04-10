#ifndef _VOXEL2D_H_
#define _VOXEL2D_H_
#include <vector>
//#include <opencv2/opencv.hpp>
//#include "DataStruct.h" // include datastructure
#include "ModuleStruct.hpp"
/**
* \brief voxel class (physicially single voxel) with dimension of two
* 
* The voxel class with dimension of two, in 2D plane, similar to pixel.
* 
*/ 
class Voxel2D_mesh {
public:
	/**
	* \brief constructor
	*/
	Voxel2D_mesh();
	/**
	* \brief destructor
	*/
	~Voxel2D_mesh();
	/**
	* \brief clear class memeber
	*/
	void clear();
	/**
	* \brief reset class memeber
	*/
	void reset();
	/**
	* \brief append contained point index into member point indices
	* @param pt_id point index in PointArray of original model
	*/
	void appendPtID(const unsigned int pt_id);
	/**
	* \brief get contained point indices
	* @return a vector of inside point indices
	*/
	std::vector<unsigned int> getPtIDs();
	/**
	* \brief get number of contained point indices
	*/
	unsigned int getNumPtIds() const;
	/**
	* \brief append contained point index into member point indices
	* @param pt_id point index in PointArray of original model
	*/
	void appendPt(const cv::Point2f point2f);
	/**
	* \brief get contained points
	* @return a vector of inside points
	*/
	std::vector<cv::Point2f> getPts();
	/**
	* \brief check voxel occupation status
	* @return True: if voxel occupied; False: no point inside
	*/
	bool isOccupied() const;
	/**
	* \brief set voxel occupation status
	* @param occupation status
	*/
	void setOccupied(const bool isOccupied);
	/**
	* \brief get voxel-point intensity
	* @return point intensity of voxel
	*/
	float getIntensity() const;
	/**
	* \brief set voxel-point density
	* @return if setting successful
	*/
	bool setIntensity(const float intensity);

private:
	bool m_is_occupied;				/**< occupation status */
	float m_intensity;				/**< point intensity of one voxel, normalized over entire grid */
	std::vector<unsigned int> m_pt_ids;	/**< contained point-set indices from model's PointArray */
	std::vector<cv::Point2f> m_pts; 
};

/**
* \brief voxelgrid class (physically a series of voxels) with dimension of two
*
* The voxelgrid class with dimension of two, in 2D plane, similar to iamge or pixel matrix.
* 
*/
class VoxelGrid2D_mesh {
public:
	/**
	* \brief constructor
	*/
	VoxelGrid2D_mesh();
	/**
	* \brief destructor
	*/
	~VoxelGrid2D_mesh();
	/**
	* \brief reset class member
	*/
	void reset();
	/**
	* \brief clear class memeber
	*/
	void clear();
	/**
	* \brief clear voxels
	*/
	void clearVoxels();
	/**
	* \brief init voxels based on extent_x, extent_y
	*/
	void initVoxels();

	/**
	* \brief create voxelgrid from point array meeting different requirement
	* 
	* for 3D point-set, voxelization into 2d grid by projection onto x-y plane, z value is omitted
	* for a 3D plane, rotate it parallel to x-y plane before calling this function
	* otherwise, projected plane is a distortion of original plane
	* 
	* @param pts points of point-set model
	* @param npts number of points
	* @param intensity setting: is set intensity in binary format, 0 v.s. 255
	* @return True: successfully created
	*/
	bool create(ModuleStruct::Point3f* pts, const unsigned int npts, const bool isBinary, ModuleStruct::CMat& img);
	/**
	* \brief create voxelgrid from point array meeting different requirement
	*/
	bool create(const ModuleStruct::Point3fArray &pts, const bool isBinary, ModuleStruct::CMat &img);
	/**
	* \brief set voxelgrid extent
	* @param x number of voxels in a row, or column number
	* @param y number of voxels in a col, or row number
	*/
	bool setExtent(const unsigned int x, const unsigned int y);

	/**
	* \brief set voxel length if voxel is square
	* @param l length of voxel
	*/
	bool setLenVoxel(const double l);
	/**
	* \brief set voxel length if voxel is non-square
	* @param lx voxel length along x-axis
	* @param ly voxel length along y-axis
	*/
	bool setLenVoxel(const double lx, const double ly);
	/**
	* \brief set absolute coordinate of origin
	* @param ox x coordinate of origin in world space
	* @param oy y coordinate of origin in world space
	*/
	void setAbsOrigin(const double ox, const double oy);

	/**
	* \brief get voxels
	* @return voxels
	*/
	Voxel2D_mesh** getVoxels();
	/**
	* \brief extent of voxelgrid
	*/
	void getExtent(unsigned int& x, unsigned int& y);
	/**
	* \brief get x extent of voxelgrid
	*/
	unsigned int getExtentX() const;
	/**
	* \brief get y extent of voxelgrid
	*/
	unsigned int getExtentY() const;
	/**
	* \brief get column number of voxelgrid
	*
	* return same value of getExtentX
	* \sa getExtentX()
	*/
	unsigned int getNumCol() const;
	/**
	* \brief get row number of voxelgrid
	*
	* return same value of getExtentY
	* \sa getExtentY()
	*/
	unsigned int getNumRow() const;
	/**
	* \brief get voxel extent along x-axis
	*/
	double getLenVoxelX() const;
	/**
	* \brief get voxel extent along y-axis
	*/
	double getLenVoxelY() const;
	/**
	* \brief get absolute x coordinate of origin in world space
	*/
	double getAbsOriginX() const;
	/**
	* \brief get absolute y coordinate of origin in world space
	*/
	double getAbsOriginY() const;
	/**
	* \brief get absolute coordinate of origin in world space
	*/
	void getAbsOrigin(double& x, double& y);
	/**
	* \brief return if voxel is isometric on x and y axis
	*/
	bool isVoxelSquare() const;

private:
	Voxel2D_mesh** m_voxels;			/**< voxels forming 2D array */
	unsigned int m_extent_x;	/**< extent along x axis, corresponding to column number */
	unsigned int m_extent_y;	/**< extent along y axis, corresponding to row number */
	
	double m_len_vx;				/**< length of voxel along x-axis */
	double m_len_vy;				/**< length of voxel along y-axis */

	double m_abs_ox;				/**< absolute x coordinate value in world space */
	double m_abs_oy;				/**< absolute y coordinate value in world space */

	bool m_is_voxelSquare;		/**< True: voxel is squre; False: voxel is non-square */
};

#endif // !_VOXEL2D_H_
