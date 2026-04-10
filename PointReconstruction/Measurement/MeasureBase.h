#ifndef MEASUREBASE_H
#define MEASUREBASE_H

#include "std_header.h"
#include "../Common/VariableDefine.h"
#include <map>
#include <Eigen/Dense>

#define PLANE_MIN_MAX_SIZE	6
#define RULER_VERTICE_SIZE	4
#define RULER_BAND_SIZE	2
#define RULER_BAND_SIZE_MORE 4 //add by zhujunqing 20201218

enum AXIS_DIRECTION_XYZ
{
	AXIS_DIRECTION_X,
	AXIS_DIRECTION_Y,
	AXIS_DIRECTION_Z
};

typedef enum
{
	K_FIXED_RULER,
	K_FULL_WALL,
	K_RECT_VERTICALITY
}VERTICALITY_TYPE;
static const float AXIS_X_DIRECTION[3] = { 1.f,0.f,0.f };
static const float AXIS_Y_DIRECTION[3] = { 0.f,1.f,0.f };
static const float AXIS_Z_DIRECTION[3] = { 0.f,0.f,1.f };

struct MeasurementResultValueValuesPoints
{
	bool is_valid;
	float value;
	std::vector<std::tuple<bool, float, cv::Point3f>> points;

	MeasurementResultValueValuesPoints()
	{
		is_valid = false;
		value = -1.f;
	};
};

struct MeasurementResultValueValuesPointsT
{
	bool is_valid;
	float value;
	std::vector<std::tuple<bool, float, cv::Point3f, int, int>> points;

	MeasurementResultValueValuesPointsT()
	{
		is_valid = false;
		value = -1.f;
	};
};

struct MeasurementResultValuesDualPoints
{
	bool is_valid;
	std::vector<std::tuple<bool, float, cv::Point3f, cv::Point3f, int,int>> value_points;

	MeasurementResultValuesDualPoints()
	{
		is_valid = false;
	};
};

struct MeasurementResultValueValuesThreePoints
{
	bool is_valid;
	float value;
	std::vector<std::tuple<bool, float, cv::Point3f, cv::Point3f, cv::Point3f>> points;

	MeasurementResultValueValuesThreePoints()
	{
		is_valid = false;
		value = -1.f;
	};
};

//struct AngleSquarenessResult
//{
//	bool is_valid;
//	float angle_squareness;
//	cv::Point3f point00, point01; //point00 & point01 on the same plane
//	cv::Point3f point02, point03; //point02 && point03 on the same plane
//};

/**
* \brief structure of BIM information used for pass data and export
*
* \author Pengcheng LI
*/

//struct BIM_Info_Hole
//{
//	int   type;			/**< type of hole, window 0, door 1 */
//	int   wallid;       /**< 所在墙的编号 */
//	cv::Point3f   pos;	/**< 以内视角左下点坐标 */
//	float  length;		/**< 门窗宽度 */
//	float  height;		/**< 门窗高度 */
//	float depth;		/**< 门窗深度 */
//	float rotate;		/**内视角左下到右下连成的线，转到X轴正方向的角度。[-π, π] */
//};


/**
* \brief structure of hole information used for pass data and export
*
* \author Bichen JING
*/
struct Info_Hole
{
	typedef std::pair<cv::Point3f, cv::Point3f> pos_ruler;	/**< position of ruler */
	typedef MeasurementRulerverticeStruct Info_Ruler;		/**< information of ruler of ruler */
															// basic information
	int m_idx_plane = -1;					/**< index of segmented plane that hole belongs to */
	int m_type = -1;						/**< type of hole, window 0, door 1 */
	std::vector<cv::Point3f> m_corners;		/**< vector of hole corners, bottom-left->bottom->right->top-right->top->left */

											// hole and ruler information
	std::vector<float> m_width;				/**< vector of measured width */
	std::vector<float> m_height;			/**< vector of measured height */
	std::vector<pos_ruler> m_ruler_width;	/**< vector of ruler position for width, corresp to m_width */
	std::vector<pos_ruler> m_ruler_height;	/**< vector of ruler position for height, corresp to m_height */

											// unified data structure for converting to json format
	std::vector<Info_Ruler> m_info_ruler_width;	/**< vector of width ruler information */
	std::vector<Info_Ruler> m_info_ruler_height;/**< vector of height ruler information */

												/**
												* \brief destructor
												*/
	~Info_Hole();
	/**
	* \brief find nearest neighbor point
	* \param qPt query point
	* \param pts searching space
	* \param nn_idx index of nearest neighbor
	* \param nn_dist distance to nearest neighbor
	*/
	void findNNbrCorner(const cv::Point3f& qPt, size_t& nn_idx, float& nn_dist);
	/**
	* \brief update ruler information for export
	*/
	bool updateInfoRuler();
	/**
	* \brief print info
	*/
	void log();
};

struct HoleInfo {
	// hole: <wall_id: <hole num <lefttopPoint,righttopPoint,rightbottomPoint,leftbottomPoint> > xz_plane
	std::vector<std::tuple<int, std::vector<std::vector<cv::Point3f>>>> hole_infoVector_xz;
	// hole: <wall_id: <hole num <lefttopPoint,righttopPoint,rightbottomPoint,leftbottomPoint> > origin_plane
	//...
	std::vector<std::tuple<int, std::vector<std::vector<cv::Point3f>>>> hole_infoVector_origin;
};


struct BeamInfo {
	// beam: <wall_id: <height(mm),topPoint> >
	std::map<int, std::tuple<float, cv::Point3f>> beamMap;
	// beamHeight (raster)
	std::vector<int> rasterBeamHeightVector;
};

struct BaseBoardInfo {
	//  wall_id: height(mm)
	std::map<int, float> baseBoardMap;
	// baseboardHeight (raster)
	std::vector<int> rasterBaseBoardHeightVector;
};

struct DoorFrameInfo {
	// doorframe position in rasterImg ( raster xmin,ymin,xmax,ymax)
	std::vector<Eigen::ArrayX4i> rasterDoorFrameArrayVector;

	// 2D XZplane (x,z,x,z)
	std::vector<Eigen::ArrayX4i> doorFrameArrayXZVector;

	// MeasurementResultData::pos_hole (origin 3D )
	std::vector<std::vector<std::vector<cv::Point3f>>> wall_door_window_pos_3D;
};

struct CabinetInfo {
	// raster position (xmin, ymin, xmax, ymax)
	std::vector<Eigen::ArrayX4i> rasterCabinetBoxArrayVector;

	// 2D XZplane (xmin,zmax,xmax,zmin)
	std::vector<Eigen::ArrayX4i> cabinetBoxArrayXZVector;
};

struct SubWallInfo {
	// type
	std::vector<std::vector<int>> subWallType;
	std::vector<std::vector<float>> confidence;
	std::vector<std::vector<cv::Rect2i>> subimg_xz_rect;
	// (origin 3D subwall rectangle points )
	std::vector<std::vector<std::vector<cv::Point3f>>> sub_wall_pos_3D;
	std::vector<AXIS_DIRECTION_XYZ> ori_wall_plane_axis;
	std::vector<int> wallTrueId;
	std::vector<cv::Mat> refect_img_xz;
	std::vector<cv::Point3f> xz_plane_normal;
	std::vector<std::vector<float>> subWallBeam;
	std::vector<std::vector<float>> subWallBaseBoard;
	std::vector<std::vector<std::tuple<float, cv::Point3f>>> beam_tuples;
	std::vector<std::vector<std::tuple<float, cv::Point3f>>> baseboard_tuples;
	std::vector<std::vector<std::vector<cv::Point3f>>> subWallBrickFourVertic;
};
class ObstacleInfo {

public:

	// related to cloud points
	int pixelHeight;
	int pixelWidth;

	//current measure wall_id
	int origin_wall_id; // processing sequence
	int wall_id; // id in NUC

	//float diagonalRotationAngle; // set this value before each measure
	//cv::Point2f diagonalRotationCenter; // (coord_x, coord_z) if rotate

	virtual ObstacleInfo* getClassType() = 0;
};

class WallObstacleInfo : public ObstacleInfo {

public:
	WallObstacleInfo(std::vector<std::vector<cv::Point3f>>& filtered_wall_xyz) :filtered_wall_xyz(filtered_wall_xyz) {}

	std::vector<bool> isGlassWallVector;
	std::vector<int> rasterHeightVector;
	std::vector<int> rasterWidthVector;
	std::vector<cv::Mat> rasterImgVector;
	std::vector<cv::Mat> rasterDiagonalImgVector; // for diagonal ruler
	std::vector<cv::Mat> coarseRotationMatVector; 
	std::vector<cv::Mat> fineRotationMatVector;
	std::vector<cv::Mat> backwardCoarseRotationMatVector;
	std::vector<cv::Mat> backwardFineRotationMatVector;
	std::vector<Eigen::Matrix3f> coarseRotationEigenVector; // corresponding to coarseRotationMatVector
	std::vector<Eigen::Matrix3f> fineRotationEigenVector; // corresponding to fineRotationMatVector
	std::vector<Eigen::Matrix3f> backwardCoarseRotationEigenVector; // corresponding to backwardCoarseRotationMatVector
	std::vector<Eigen::Matrix3f> backwardFineRotationEigenVector; // corresponding to backwardFineRotationMatVector
	std::vector<float> minCloudPointsXVector;
	std::vector<float> minCloudPointsYVector;
	std::vector<float> minCloudPointsZVector;
	std::vector<float> deltaCloudPointsYVector; // maxY-minY vector
	std::vector<float> meanCloudPointsYVector; // meanY vector

	std::vector<Eigen::Vector3f> planeNormalVector;

	std::vector<float> refinedMeanCloudPointsYVector; // refined Y vector
	std::vector<Eigen::Vector3f> refinedPlaneNormalVector;

	std::vector<cv::Mat> obstacleMaskVector; // 1:obstacle 0:None ( without hole )
	std::vector<cv::Mat> doorFrameMaskVector; // 1:obstacle 0:None
	std::vector<cv::Mat> minRasterYMatVector; //min y in raster image
	std::vector<cv::Mat> maxRasterYMatVector; //max y in raster image
	//std::vector<cv::Mat> meanRasterBiasYMatVector;
	std::vector<cv::Mat> holeMaskImgVector;
	std::vector<cv::Mat> pinHoleMaskVector; // deprecated

	std::vector<std::vector<cv::Point3f>>& filtered_wall_xyz;

	// index correspond to filtered_wall_xyz
	std::vector<std::vector<std::vector<std::vector<int>>>> indexMapMatVector; // originWallID, row,col,indexList

	// cropped wall cloud points (xz_plane)
	std::vector<std::vector<cv::Point3f>> cropped_filtered_wall_xyz;
	std::vector<std::vector<cv::Point3f>> removeCritical_filtered_wall_xyz;

	// index correspond to cropped_filtered_wall_xyz
	std::vector<std::vector<std::vector<std::vector<int>>>> croppedWallIndexMapMatVector; // originWallID, row,col,indexList
	std::vector<std::vector<std::vector<Eigen::Vector3f>>> meanCloudPointsMatVector; // mean cloudPoints: row, col, meanValue

	std::pair<bool, float> oneMeterMeanZ=std::make_pair(false, 0);// added by hgj
	// roof beam
	BeamInfo beamInfo;
	// baseBoard
	BaseBoardInfo baseBoardInfo;
	// door frame
	DoorFrameInfo doorFrameInfo;
	// cabinet
	CabinetInfo cabinetInfo;

	//hole 
	HoleInfo holeInfo;


	SubWallInfo subWallInfo; //added by hgj 20210723
	WallObstacleInfo* getClassType() { return this; }

};

class CeilingObstacleInfo : public ObstacleInfo {

public:

	std::vector<int> rasterHeightVector;
	std::vector<int> rasterWidthVector;
	std::vector<cv::Mat> rasterImgVector;

	CeilingObstacleInfo* getClassType() { return this; }

};

class GroundObstacleInfo : public ObstacleInfo {

public:
	cv::Mat rasterImg;
	cv::Mat obstacleMask; // 1:obstacle 0:None
	cv::Mat holeMaskImg;

	int rasterHeight;
	int rasterWidth;

	float minCloudPointsX;
	float minCloudPointsY;
	float minCloudPointsZ;
	float meanCloudPointsZ;
	float deltaCloudPointsZ;

	float refinedMeanCloudPointsZ;
	Eigen::Vector3f refinedPlaneNormal;
	std::vector<std::vector<Eigen::Vector3f>> meanGroundCloudPointsMatVector; // mean cloudPoints: row, col, meanValue
	Eigen::Matrix3f backwardCoarseRotationEigen;
	Eigen::Matrix3f backwardFineRotationEigen;
	//backwardCoarseRotationEigen = castObstacleInfoPtr->backwardCoarseRotationEigen;
	//backwardFineRotationEigen = castObstacleInfoPtr->backwardFineRotationEigen;

	GroundObstacleInfo* getClassType() { return this; }

};

class MeasureBase {

public:
	/**
	* \brief check if two float values equal or not
	* \author Bichen JING
	* \tparam VAL type of value
	* \tparam CMP type of compared value
	* \param val input value
	* \param cmp input compared value
	* \return true if values are equal, otherwise, false
	*
	* need to include <limits>
	*/
	template<typename VAL, typename CMP>
	static bool IsValEqual(const VAL& val, const CMP& cmp) {
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
	static bool IsValZero(const T& val) {
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
	static bool IsInBounds(const T& val, const T& lower, const T& upper) {
		// data validation
		assert(lower <= upper);
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
	static bool IsInBounds(const T& val, const std::pair<T, T>& bounds)
	{
		// data validation
		assert(bounds.first <= bounds.second);
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
	static bool IsPtInBounds2D(const T& x, const T& y,
		const T& xmin, const T& xmax, const T& ymin, const T& ymax)
	{
		return IsInBounds(x, xmin, xmax) && IsInBounds(y, ymin, ymax);
	}

public:
	//int min_num_pts_inrulerband =100;

	static bool FindPlaneMinMaxXYZ(const std::vector<cv::Point3f>& input_points,
		const int normal_axis,
		const float length_plane_threshold1,
		const float length_plane_threshold2,
		float* output_minmax_xyz);
	/*added by Tang Xueyan*/
	static bool FindPlaneMinMaxXYZ2(const std::vector<cv::Point3f>& input_points,
		const int normal_axis,
		const float length_plane_threshold1,
		const float length_plane_threshold2,
		float* output_minmax_xyz);

	static bool FindPointsMinMaxXYZ(const std::vector<cv::Point3f>& points, float* minmax_xyz);
	static bool Cut_FindBigestValidMinMaxXYZ(const std::vector<cv::Point3f>& input_points, const array<float, 4>& hole_min_max,
		const int normal_axis, float* in_output_minmax_xyz);
	//static bool FindBigestValidMinMaxXYZ(const std::vector<cv::Point3f>& input_points, std::vector<array<float, 4>> &hole_min_max, const int normal_axis, float* in_output_minmax_xyz);
	static bool FindBigestValidMinMaxXYZ(const std::vector<cv::Point3f>& input_points, std::vector<array<float, 4>>& hole_min_max, const int normal_axis, float* in_output_minmax_xyz, const int wall_id = -1, ObstacleInfo* obstacleInfoPtr = NULL);

	//copy from Vertility & revised return value
	static int FindMaxPlaneInHolePlane(const std::vector<std::vector<cv::Point3f>>& pos_hole, AXIS_DIRECTION_XYZ axis, float* data_minmax_xyz);
	/*end adding by Tang Xueyan*/

	//find pts in ruler for vertical planes without rotation
	static bool FindRulerBandPts(const std::vector<cv::Point3f>& input_points,
		const float ang_value,
		const float* ruler_band,
		std::vector <int>& num_data_in_rulerband);

	// find two vertices (x1,y1) & (x2,y2) of vertical plane on x-y plane, vertice_indx=[x1,x2,y1,y2]
	static void FindVerticeXY(const float* data_minmax_xyz, const float plane_normal[3], int* vertice_indx);

	// find real vertice of plane
	static void FindVertice(const int normal_axis,
		const float plane_normal[3],
		const bool ifRotate,
		const float ang_value,
		const float* data_minmax_xyz,
		std::vector<cv::Point3f>& vertice);

	static void ReArrangeVerticeWithCompass(float CompassAng, std::vector<cv::Point3f>& vertice, std::vector<cv::Point3f>& verticeRot);

	// find z value of real corner of top/bottom plane
	static int FindCornerPts(const std::vector<cv::Point3f>& plane_points,
		const float* virtual_corner,
		const cv::Mat rotationMatrix,
		float* cornerpt_z); //replace
	/*added by Tang Xueyan*/
	static int FindCornerPts(const std::vector<cv::Point3f>& plane_points,
		const float* virtual_corner,
		const float measure_ROI_length,
		const cv::Mat rotation_matrix,
		float* cornerpt_z);

	static int FindCornerPts(const std::vector<cv::Point3f>& plane_points,
		const float* virtual_corner,
		const float measure_ROI_length,
		const float offset_ROI_actualPts,
		unsigned int min_pt_num_in_quarter,
		const cv::Mat rotation_matrix,
		float* cornerpt_z);
	/*end adding by Tang Xueyan*/

	/*For 3D rotation matrix*/
	//Rodrigues' rotation formula 


	static void NormalizeVector(const float* vector, float* normalized_vector); //convert to unit vector
	static void AlignVectorSgn(const float* vector, float* aligned_vector); //uniform vector sign
	static void UniformNormals(const float* normal, const float* center, float* uniform_normal); //align normal which will point to inside 

	static void CrossProduct(const float* a, const float* b, float* c);
	static double DotProduct(const float a[3], const float b[3]); //replace
	static void DotProduct(const float* a, const float* b, float* value);


	/*added by Tang Xueyan*/
	static cv::Mat CalRotationMatrixFromVectors(const float* vector_before, const float* vector_after);
	static void CalcAngleVectorXY2YAxis(const float* vector, float* angle); //rotation angle around Z
	static void CalcAngleVectorXY2XAxis(const float* vector, float* angle); //rotation angle around Z
	static void CalcAngleVectorYZ2YAxis(const float* vector, float* angle); //rotation angle around X
	static void CalcAngleVectorYZ2ZAxis(const float* vector, float* angle); //rotation angle around X
	static void CalcAngleVectorXZ2XAxis(const float* vector, float* angle); //rotation angle around Y
	static void CalcAngleVectorXZ2ZAxis(const float* vector, float* angle); //rotation angle around Y
	static cv::Mat TranslateAngleAroundX2RotationMatrix(const float angle);
	static cv::Mat TranslateAngleAroundY2RotationMatrix(const float angle);
	static cv::Mat TranslateAngleAroundZ2RotationMatrix(const float angle);
	static void RotateVector(const float* vector, const cv::Mat rotation_matrix, float* rotated_vector);
	static void RotateVectorAroundZ(const float* vector, const float angle, float* rotated_vector);
	static void RotatePoint(const cv::Point3f point, const cv::Mat rotation_matrix, cv::Point3f& rotated_point);
	static void RotatePoint(const float* point, const cv::Mat rotation_matrix, float* rotatePoint);
	static bool RotatePoints(const std::vector<cv::Point3f>& points, const cv::Mat& rotation_matrix, std::vector<cv::Point3f>& rotated_points);
	static bool RotatePoints(const cv::Mat& points, const cv::Mat& rotation_matrix, cv::Mat& rotated_points);
	static void RotatePointAroundZ(cv::Point3f& point, const float angle, cv::Point3f& rotated_point);
	static bool RotatePointsAroundZ(const std::vector<cv::Point3f>& points, const float angle, std::vector <cv::Point3f>& rotated_points);
	static bool RotatePointsWithSequence(const std::vector<cv::Point3f>& plane_points, const std::vector<cv::Mat>& rotation_matrices, std::vector<cv::Point3f>& rotated_plane_points);
	/*end adding by Tang Xueyan*/

	//find distance between two planes along normals
	static float DistTwoPlane(const cv::Point3f& plane_pointsA_center, const float plane_normalA[3], const cv::Point3f& plane_pointsB_center, const float plane_normalB[3]);

	static cv::Point ConvertVoxelIDToPts(const cv::Point pts_idx, const float* min_max_xyz, const float length_of_voxel);

	static void ConvertXYToVoxelID(const float x, const float y, unsigned int cols_of_voxel, unsigned int rows_of_voxel, const float* min_max_xyz, const float length_of_voxel,
		unsigned int& col_idx, unsigned int& row_idx);

	static void Generate2DImage(const std::vector<cv::Point3f>& input_data, const float* min_max_xyz, const float length_of_voxel, cv::Mat& image);

	//verify if pt in rectangle only for walls or level planes
	static bool IfPointInsideRect(const cv::Point3f pt, const std::vector<cv::Point3f>& plane_vertices);


	/*added by Tang Xueyan*/
	//filter holes which are connected with the ground
	static bool FilterHolesbyVertical(const std::vector<std::vector<cv::Point3f>>& holes, const float Z_threshold, std::vector<std::vector<cv::Point3f>>& filtered_holes);
	/*end adding by Tang Xueyan*/

	/*added by lichun*/
	//generate matrix of project to x-y direction
	//parameters
	//plane_points : input data
	//plane_normal : the normal of plane
	//mat : the project matrix
	//project_points: the points of project
	static void ProjectToXYMat(const std::vector<cv::Point3f>& plane_points, const cv::Point3f& plane_normal, cv::Mat& mat, cv::Mat& project_points);

	//Find the x, y limits form 3D points
	//plane_points_mat : the matrix of 3D points
	//min_x, max_x : the output of minmium x and maxmium x
	//min_y, max_y : the output of minmium y and maxmium y
	static void FindMinMaxForXY(const cv::Mat& plane_points_mat, float& min_x, float& max_x, float& min_y, float& max_y);
	/*end adding*/

	static bool SavePoints(const std::vector<cv::Point3f>& plane_points, const string file_name);
	static bool SavePoints(const cv::Mat plane_points, const string file_name);


	static void GetLargestRect(cv::Mat& img, cv::Rect& rc);
	static int largestRectangleArea(std::vector<int>& height, int& start, int& end, int& hi);
};

#endif MEASUREBASE_H
