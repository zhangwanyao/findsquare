#pragma once


#include <vector>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp> 
#include "Measurement/MeasureBase.h"
#include "CutScanPlane/DoorWindowDetector/MeasureDoorWindow.h"
#include "PlaneCuttingInterface.h"
class DetectObstacle
{
public:
	DetectObstacle();
	~DetectObstacle();

	/**
	* \brief union set to merge detected box
	*/
	struct Union_set {
		Union_set(Eigen::ArrayX4i boxList);
		int find(int id);
		void unionElement(int id1, int id2);

		int n;
		Eigen::ArrayXi father;
		Eigen::ArrayXi rank;
		Eigen::ArrayX4i _detectBoxArray;
	};

	/**
     * \brief detect wall obstacle (beam, baseboard and cabinet)
     * \param scene_filter_output: input cloud points
     * \param scene_plane_center: center of cloud points
     * \param scene_plane_normals: normal of cloud points
     * \param wall_door_window_pos: hole information
     * \param ObstacleInfo: result to saved
     * \return bool
     */
	bool DetectWallObstacleFcn(PlaneCutResultInterface  &mPlane,
		WallObstacleInfo& wallObstacleInfo, bool onlyDetect);




	/**
	* \brief raster image with depth in xz-plane
	* \param cloudPoints: input cloud point
	* \param rasterImg: output raster image
	* \param minGlobalX: min coordinate x for reference
	* \param maxGlobalX: max coordinate x for reference
	* \param minGlobalZ: min coordinate z for reference
	* \param maxGlobalZ: max coordinate z for reference
	* \return bool
	*/
	bool raster_XZ_img(const std::vector<cv::Point3f>& cloudPoints, float minGlobalX = FLT_MAX, float maxGlobalX = -FLT_MAX, float minGlobalZ = FLT_MAX, float maxGlobalZ = -FLT_MAX);

	/**
	* \brief rotate wall to xz_plane
	* \param plane_points: input cloud point
	* \param plane_normal: normal of cloud points
	* \param outputCloud: rotated plane
	* \return bool
	*/
	bool Rotate2XZPlane(const std::vector<cv::Point3f>& plane_points, const float* plane_normal, std::vector<cv::Point3f>& outputCloud);

	/**
	* \brief raster image with reflection
	* \param cloudPoints: input cloud point
	* \param planeReflectValues: reflection
	* \return bool
	*/
	bool raster_XZ_img_with_reflection(const std::vector<cv::Point3f>& cloudPoints, const std::vector<unsigned char> planeReflectValues);

	/**
	* \brief detect beam
	* \param cloudPoints: input cloud point
	* \param beamResult: beam height and a corner point to saved
	* \return bool
	*/
	bool DetectBeam(const std::vector<cv::Point3f>& cloudPoints, std::vector<float>& beamResult);

	/**
	* \brief detect baseboard
	* \param cloudPoints: input cloud point
	* \param baseBoardHeight: baseboard height to saved
	* \return bool
	*/
	bool DetectBaseBoard(const std::vector<cv::Point3f>& cloudPoints, float& baseBoardHeight);

	/**
	* \brief detect door frame
	* \param cloudPoints: input cloud point
	* \param origin_wall_id: wall_door_window_pos id
	* \param wall_id: wall id
	* \param wall_door_window_pos: position of hole
	* \param wall_door_window_pos: hole information
	* \param ObstacleInfo: result to saved
	* \return bool
	*/
	bool DetectDoorFrame(const std::vector<cv::Point3f>& cloudPoints, const std::vector<std::vector<std::vector<cv::Point3f>>>& wall_door_window_pos,int wallId, WallObstacleInfo& obstacleInfo);

	/**
	* \brief detect cabinet with depth and reflection image
	* \param cloudPoints: input cloud point
	* \param ObstacleInfo: result to saved
	* \return bool
	*/
	bool DetectCabinet(const std::vector<cv::Point3f>& cloudPoints, WallObstacleInfo& obstacleInfo);


	/**
	* \brief detect cabinet with depth image
	* \param cloudPoints: input cloud point
	* \param ObstacleInfo: result to saved
	* \return bool
	*/
	bool DetectCabinetWithDepth(const std::vector<cv::Point3f>& cloudPoints, const int wall_id, ObstacleInfo& obstacleInfo);


	/**
	* \brief detect beam and baseboard
	* \param cloudPoints: input cloud points
	* \param wall_id: wall id
	* \param ObstacleInfo: result to saved
	* \return bool
	*/
	bool DetectBeamAndBaseboard(const std::vector<cv::Point3f>& cloudPoints, WallObstacleInfo& obstacleInfo);
	/**
	* \brief calculate mode number
	* \param data: data to be calculated
	* \param resolution: width of bins
	* \param result: mode number
	* \return bool
	*/
	void statsMode(const Eigen::ArrayXf& data, const float resolution, float* result);

	bool DetectWallBoundaryBoard(const std::vector<cv::Point3f>& cloudPoints);

	bool JudgeDoorInGlassWall(const std::vector<std::vector<cv::Point3f>> plane_door_window_pos);



private:
	int wall_id;
	int origin_wall_id;

	// raster image
	const int pixelHeight; // 1 pixel mean x mm 
	const int pixelWidth;

	int rasterHeight; // raster image height
	int rasterWidth;

	//wall size
	float wallWidth;
	float wallHeight;

	int numPoints; // cloud point 
	Eigen::ArrayXf coordX;
	Eigen::ArrayXf coordY;
	Eigen::ArrayXf coordZ;
	Eigen::ArrayXf biasY; // y - min_y

	Eigen::MatrixXi countMat;
	cv::Mat rasterImg;
	cv::Mat reflectRasterImg;
	cv::Mat minRasterYMat; //min y in raster image
	cv::Mat maxRasterYMat; //max y in raster image
	cv::Mat meanRasterBiasYMat;
	cv::Mat pinHoleMask; // deprecated

	int sparseDistanceToCameraThreshold1; // merge pixel in 1cm if less than this value
	int sparseDistanceToCameraThreshold2; // merge pixel in 3cm if less than this value
	//int sparseDistanceToCameraThreshold3;

	const int MIN_RASTER_IMG_HEIGHT_THRESHOLD;
	const int MIN_RASTER_IMG_WIDTH_THRESHOLD;

	float minCloudPoints_X; // min x coordinate of cloud point
	float minCloudPoints_Y; // min y coordinate of cloud point
	float minCloudPoints_Z; // min z coordinate of cloud point
	float maxCloudPoints_X;
	float maxCloudPoints_Y;
	float maxCloudPoints_Z;
	float meanCloudPoints_Y; // mean y coordinate of cloud point
	float meanCloudPoints_Z; // mean z coordinate of cloud point

	float refinedMeanCloudPoints_Y;
	float refinedMeanCloudPoints_Z;
	float deltaCloudPointsY;
	float deltaCloudPointsZ;

	std::vector<std::vector<std::vector<int>>> indexMapMat; // row,col,indexList
	std::vector<std::vector<std::vector<int>>> croppedWallIndexMapMat;
	std::vector<std::vector<Eigen::Matrix3Xf>> meanCloudPointsMat;

	cv::Mat obstacleMask; // cabinet and doorFrame mask    1:obstacle 0:None

	// rotation Matrix
	Eigen::MatrixXi idxMat; // relation between cloud point and pixel coordinate  // will be deprecated
	cv::Mat coarseRotationMat;
	cv::Mat fineRotationMat;
	cv::Mat backwardCoarseRotationMat;
	cv::Mat backwardFineRotationMat;
	Eigen::Matrix3f backwardCoarseRotationEigen;
	Eigen::Matrix3f backwardFineRotationEigen;

	// canny 
	int cannyMinThreshold;
	int cannyMaxThreshold;

	// beam
	int beamExpandingValue;
	int rasterBeamHeight;

	// detect baseboard
	int clippedBaseboardHeight;
	int baseBoardExpandingValue;
	int rasterBaseBoardHeight;

	// detect door frame
	float expandRatio; // expanding threshold  
	int minExpandPixelValueThreshold; // min pixel value in expanding;
	int clipWidth; // width of roi
	int clipUpHeight; // height of roi
	int clipBottomHeight; // height of roi
	int correctWidth; // width of correction
	int doorFrameMergeGap; // union gap
	int doorFrameExpandingValue;
	Eigen::ArrayX4i	 rasterDoorFrameArray;

	// detect cabinet
	int cabinetMergeGap; // union gap
	int cabinetExpandingValue;
	Eigen::ArrayX4i detectDepthCabinetArray; //  result with depth image
	Eigen::ArrayX4i detectReflectionCabinetArray; //  result with reflection image
	Eigen::ArrayX4i	 rasterCabinetBoxArray; // final result

	// detect hole
	int minHoleAreaThreshold;
	cv::Mat holeResultImg;

	// for debug
	static bool clearRawDir;
	static bool clearObstaclesDir;
	std::string SAVE_OBSTACLES_DIR_PREFIX;
	std::string SAVE_OBSTACLES_DIR;
	bool SAVE_OBSTACLES;

	static bool clearObstaclesDir_baseBoard;
	static bool clearObstaclesDir_topBoard;
	static bool clearObstaclesDir_leftBoard;
	static bool clearObstaclesDir_rightBoard;
};

