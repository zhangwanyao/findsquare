#include "DetectObstacle.h"
#ifdef _WIN32
#include <io.h>
#include <direct.h>
#else
#include<unistd.h>
#endif

#include <opencv2/core/eigen.hpp>


#define VERTICALITY_ANGLE_MARK_CHECK (M_PI * 0.25f)
#define DEBUG_SAVE_ORIGIN_CLOUDPOINTS false
#define DEBUG_SAVE_CROPPED_CLOUDPOINTS false
#define DEBUG_DOOR_FRAME false
#define DEBUG_CABINET false
#define DEBUG_BEAM false
#define DEBUG_GLASS false
#define DEBUG_MASK false
#define DEBUG_HOLE false
#define DEBUG_REFINE_NORMAL_AND_MEAN_VALUE false

#define SMOOTH_RASTER_IMG true
#define DEBUG_ONE_METER false
#define SUB_WALL_CHECK true
#define DEBUG_DRAW_SUBWALL_IMG false
#define DEBUG_CHECK_SUBWALL_BEAM false
#define DEBUG_BASE_BOARD true
// if true set circle radius 5cm ,false default is 10cm
#define ONE_METER_CIRCLE_RADIUS_IS_FIVE false 
#define DEBUG_CHECK_SUBWALL_BRICK_RECT false

#define TOP_BOARD true
#define LEFT_BOARD true
#define RIGHT_BOARD true

extern std::string cloudPointsFileName;

bool DetectObstacle::clearRawDir = false;
bool DetectObstacle::clearObstaclesDir = false;
bool DetectObstacle::clearObstaclesDir_baseBoard = false;
bool DetectObstacle::clearObstaclesDir_topBoard = false;
bool DetectObstacle::clearObstaclesDir_leftBoard = false;
bool DetectObstacle::clearObstaclesDir_rightBoard = false;


DetectObstacle::DetectObstacle() :pixelHeight(10), pixelWidth(10), MIN_RASTER_IMG_HEIGHT_THRESHOLD(50), MIN_RASTER_IMG_WIDTH_THRESHOLD(15)
{

	sparseDistanceToCameraThreshold1 = 300; // cm
	sparseDistanceToCameraThreshold2 = 400; // cm
	//sparseDistanceToCameraThreshold3 = 500; // cm
	// canny 
	cannyMinThreshold = 50;
	cannyMaxThreshold = 150;

	// beam
	beamExpandingValue = 1;

	// detect baseboard
	clippedBaseboardHeight = 35;//old 10; 30,changed by hgj
	baseBoardExpandingValue = 1;

	// detect door frame
	expandRatio = 0.6;
	minExpandPixelValueThreshold = 20;
	clipWidth = 15;//tao 30->15
	clipUpHeight = 15;//modify by zhujunqing 20210518
	clipBottomHeight = 10;
	correctWidth = 8;
	doorFrameMergeGap = 5;
	doorFrameExpandingValue = 1;

	// detect cabinet
	cabinetMergeGap = 1;
	cabinetExpandingValue = 1;

	// detect hole
	minHoleAreaThreshold = 10;


	// DEBUG save obstacles images
	SAVE_OBSTACLES_DIR_PREFIX = "D:/DEBUG";
	SAVE_OBSTACLES = _access(SAVE_OBSTACLES_DIR_PREFIX.c_str(), 0) == 0 ? true : false;
	SAVE_OBSTACLES_DIR = SAVE_OBSTACLES_DIR_PREFIX + "/" + cloudPointsFileName + "_obstacles";

	if (SAVE_OBSTACLES && _access(SAVE_OBSTACLES_DIR.c_str(), 0) != 0) {
		_mkdir(SAVE_OBSTACLES_DIR.c_str());
	}
	else if (SAVE_OBSTACLES && !clearObstaclesDir) {
		std::string delFile = "del /a /f /q " + SAVE_OBSTACLES_DIR + "/*.jpg";
		replace(delFile.begin() + 11, delFile.end(), '/', '\\');
		//std::cout << delFile << std::endl;
		system(delFile.c_str());
		clearObstaclesDir = true;
	}


}
DetectObstacle::~DetectObstacle()
{

}


DetectObstacle::Union_set::Union_set(Eigen::ArrayX4i boxList)
{
	_detectBoxArray = boxList;
	n = _detectBoxArray.rows();
	father.conservativeResize(n);
	rank.conservativeResize(n);
	for (int i = 0; i < n; i++) {
		father(i) = i;
		rank(i) = 1;
	}
	//std::cout << "Union_set::n:" << n << std::endl;
	//std::cout << "Union_set::father:" << father << std::endl;
	//std::cout << "Union_set::rank:" << rank << std::endl;
}

int DetectObstacle::Union_set::find(int id) {
	if (father(id) == id) return id;

	father(id) = find(father(id));
	return father(id);
}

void DetectObstacle::Union_set::unionElement(int id1, int id2) {
	int root1 = find(id1);
	int root2 = find(id2);
	int union_min_x, union_min_y, union_max_x, union_max_y;
	if (root1 == root2)	return;
	if (rank(root1) > rank(root2)) {
		father(root2) = root1;
		rank(root1) += root2;

		union_min_x = std::min<int>(_detectBoxArray(root1, 0), _detectBoxArray(root2, 0));
		union_min_y = std::min<int>(_detectBoxArray(root1, 1), _detectBoxArray(root2, 1));
		union_max_x = std::max<int>(_detectBoxArray(root1, 2), _detectBoxArray(root2, 2));
		union_max_y = std::max<int>(_detectBoxArray(root1, 3), _detectBoxArray(root2, 3));
		//Eigen::Array4i mergedBox(union_min_x, union_min_y, union_max_x, union_max_y);
		_detectBoxArray.row(root1) = Eigen::Array4i(union_min_x, union_min_y, union_max_x, union_max_y);
	}
	else {
		father(root1) = root2;
		rank(root2) += root1;
		union_min_x = std::min<int>(_detectBoxArray(root1, 0), _detectBoxArray(root2, 0));
		union_min_y = std::min<int>(_detectBoxArray(root1, 1), _detectBoxArray(root2, 1));
		union_max_x = std::max<int>(_detectBoxArray(root1, 2), _detectBoxArray(root2, 2));
		union_max_y = std::max<int>(_detectBoxArray(root1, 3), _detectBoxArray(root2, 3));
		_detectBoxArray.row(root2) = Eigen::Array4i(union_min_x, union_min_y, union_max_x, union_max_y);
	}

	//std::cout << "******" << to_string(root1)<<"******"<<to_string(root2)<<"******" << std::endl;
	//std::cout << "union_min_x:" << union_min_x << std::endl;
	//std::cout << "union_min_y:" << union_min_y << std::endl;
	//std::cout << "union_max_x:" << union_max_x << std::endl;
	//std::cout << "union_max_y:" << union_max_y << std::endl;
	//std::cout << "father:" << father << std::endl;
	//std::cout << "rank:" << rank << std::endl;
	//std::cout << "union::_detectBoxArray" << _detectBoxArray << std::endl;
}

static bool compVec4iX(const cv::Vec4i& a, const cv::Vec4i& b) {
	return a[0] < b[0];
}

static bool compVec4iY(const cv::Vec4i& a, const cv::Vec4i& b) {
	return a[1] < b[1];
}

bool DetectObstacle::Rotate2XZPlane(const std::vector<cv::Point3f>& plane_points, const float* plane_normal, std::vector<cv::Point3f>& outputCloud)
{
	// rotation 1
	std::vector<cv::Point3f> rot_plane_points_to_y_coarse;
	float rot_plane_normal_to_y_coarse[3];
	float rotation_angle_to_y_coarse;
	//std::cout << "plane_normal:" << plane_normal[0] << "," << plane_normal[1] << "," << plane_normal[2] << std::endl;
	MeasureBase::CalcAngleVectorXY2YAxis(plane_normal, &rotation_angle_to_y_coarse);
	//this->CalcAngleVectorXY2YAxis(plane_normal, &rotation_angle_to_y_coarse);

	cv::Mat rotation_matrix_to_y_coarse = MeasureBase::TranslateAngleAroundZ2RotationMatrix(rotation_angle_to_y_coarse);
	MeasureBase::RotateVector(plane_normal, rotation_matrix_to_y_coarse, rot_plane_normal_to_y_coarse);
	MeasureBase::RotatePoints(plane_points, rotation_matrix_to_y_coarse, rot_plane_points_to_y_coarse);
	// rotation 2
	std::vector<cv::Point3f> rot_plane_points_to_y_fine;
	float rotation_angle_to_y_fine;
	MeasureBase::CalcAngleVectorYZ2YAxis(rot_plane_normal_to_y_coarse, &rotation_angle_to_y_fine);
	cv::Mat rotation_matrix_to_y_fine = MeasureBase::TranslateAngleAroundX2RotationMatrix(rotation_angle_to_y_fine);
	MeasureBase::RotatePoints(rot_plane_points_to_y_coarse, rotation_matrix_to_y_fine, rot_plane_points_to_y_fine);

	/*Backward rotation matrix*/
	cv::Mat backward_rotation_matrix_from_y_coarse(3, 3, CV_32FC1);
	cv::Mat backward_rotation_matrix_from_y_fine(3, 3, CV_32FC1);
	cv::transpose(rotation_matrix_to_y_coarse, backward_rotation_matrix_from_y_coarse);
	cv::transpose(rotation_matrix_to_y_fine, backward_rotation_matrix_from_y_fine);

	// save
	outputCloud = rot_plane_points_to_y_fine;
	coarseRotationMat = rotation_matrix_to_y_coarse;
	fineRotationMat = rotation_matrix_to_y_fine;
	backwardCoarseRotationMat = backward_rotation_matrix_from_y_coarse;
	backwardFineRotationMat = backward_rotation_matrix_from_y_fine;
	cv::cv2eigen(backwardCoarseRotationMat, backwardCoarseRotationEigen);
	cv::cv2eigen(backwardFineRotationMat, backwardFineRotationEigen);

	return true;
}
bool DetectObstacle::raster_XZ_img_with_reflection(const std::vector<cv::Point3f>& cloudPoints, const std::vector<unsigned char> planeReflectValues)
{
	// NOTE: share value with rasterImg !!!

	raster_XZ_img(cloudPoints);

	std::vector<float> floatPlaneReflectValues(planeReflectValues.begin(), planeReflectValues.end());
	Eigen::Map<Eigen::ArrayXf> ReflectValues(floatPlaneReflectValues.data(), numPoints);
	//std::cout << ReflectValues << std::endl;
	//Eigen::ArrayXi reflectPixelValue = (255 * (ReflectValues - ReflectValues.minCoeff()) / (ReflectValues.maxCoeff() - ReflectValues.minCoeff())).cast<int>();
	Eigen::ArrayXi reflectPixelValue = ReflectValues.cast<int>();
	//int pixelHeight = 10;
	//int pixelWidth = 10;

	Eigen::MatrixXi reflectValueMat(Eigen::MatrixXi::Zero(rasterHeight, rasterWidth));

	for (int id = 0; id < numPoints; id++) {
		//std::cout << pixelValue(id) << std::endl;
		int axisY = int(coordZ(id) / pixelHeight);
		int axisX = int(coordX(id) / pixelWidth);
		reflectValueMat(axisY, axisX) += reflectPixelValue(id);
	}

	reflectRasterImg = cv::Mat::zeros(rasterHeight, rasterWidth, CV_8UC1);
	for (int i = 0; i < rasterImg.rows; ++i) {
		for (int j = 0; j < rasterImg.cols; ++j) {
			if (countMat(i, j) != 0) {
				reflectRasterImg.at<uchar>(i, j) = int(reflectValueMat(i, j) / countMat(i, j));
			}
		}
	}

	//flip
	cv::flip(reflectRasterImg, reflectRasterImg, 0);

	return true;
}

bool DetectObstacle::raster_XZ_img(const std::vector<cv::Point3f>& cloudPoints, float minGlobalX /*= FLT_MAX*/, float maxGlobalX /*= -FLT_MAX*/, float minGlobalZ /*= FLT_MAX*/, float maxGlobalZ /*= -FLT_MAX*/)
{
	numPoints = cloudPoints.size();

	std::vector<float> cloudPoints_X(numPoints);
	std::vector<float> cloudPoints_Y(numPoints);
	std::vector<float> cloudPoints_Z(numPoints);
	for (auto iter = cloudPoints.begin(); iter < cloudPoints.end(); iter++) {
		int id = iter - cloudPoints.begin();
		cloudPoints_X[id] = cloudPoints[id].x;
		cloudPoints_Y[id] = cloudPoints[id].y;
		cloudPoints_Z[id] = cloudPoints[id].z;
	}

	Eigen::Map<Eigen::ArrayXf> cloudPoints_mapX(cloudPoints_X.data(), numPoints);
	Eigen::Map<Eigen::ArrayXf> cloudPoints_mapY(cloudPoints_Y.data(), numPoints);
	Eigen::Map<Eigen::ArrayXf> cloudPoints_mapZ(cloudPoints_Z.data(), numPoints);

	minCloudPoints_X = cloudPoints_mapX.minCoeff();
	minCloudPoints_Y = cloudPoints_mapY.minCoeff();
	minCloudPoints_Z = cloudPoints_mapZ.minCoeff();

	maxCloudPoints_X = cloudPoints_mapX.maxCoeff();
	maxCloudPoints_Y = cloudPoints_mapY.maxCoeff();
	maxCloudPoints_Z = cloudPoints_mapZ.maxCoeff();

	minCloudPoints_X = min(minCloudPoints_X, minGlobalX);
	minCloudPoints_Z = min(minCloudPoints_Z, minGlobalZ);
	maxCloudPoints_X = max(maxCloudPoints_X, maxGlobalX);
	maxCloudPoints_Z = max(maxCloudPoints_Z, maxGlobalZ);

	meanCloudPoints_Y = cloudPoints_mapY.mean();

	deltaCloudPointsY = maxCloudPoints_Y - minCloudPoints_Y;

	wallWidth = maxCloudPoints_X - minCloudPoints_X;
	wallHeight = maxCloudPoints_Z - minCloudPoints_Z;

	coordX = (cloudPoints_mapX - minCloudPoints_X);
	coordZ = (cloudPoints_mapZ - minCloudPoints_Z);
	biasY = (cloudPoints_mapY - minCloudPoints_Y);

	Eigen::ArrayXi pixelValue = (255 * (cloudPoints_mapY - cloudPoints_mapY.minCoeff()) / (cloudPoints_mapY.maxCoeff() - cloudPoints_mapY.minCoeff())).cast<int>();

	rasterWidth = int(wallWidth / pixelWidth) + 1;
	rasterHeight = int(wallHeight / pixelHeight) + 1;

	// calc Distance To Camera
	std::vector<int> distanceToCamera; // cm
	distanceToCamera.resize(rasterWidth);
	float spanPerPixel = (maxCloudPoints_X - minCloudPoints_X) / rasterWidth;
	float squareMeanY = pow(meanCloudPoints_Y, 2);
	for (int DistanceToCameraID = 0; DistanceToCameraID < rasterWidth; DistanceToCameraID++) {
		distanceToCamera[DistanceToCameraID] = sqrt(squareMeanY + pow(minCloudPoints_X + spanPerPixel * DistanceToCameraID, 2)) / pixelWidth;
	}

	Eigen::MatrixXi valueMat(Eigen::MatrixXi::Zero(rasterHeight, rasterWidth));
	Eigen::MatrixXf accBiasYValueMat(Eigen::MatrixXf::Zero(rasterHeight, rasterWidth));
	countMat = Eigen::MatrixXi::Zero(rasterHeight, rasterWidth);
	idxMat = Eigen::MatrixXi::Constant(rasterHeight, rasterWidth, -1);
	indexMapMat.resize(rasterHeight);
	croppedWallIndexMapMat.resize(rasterHeight);
	meanCloudPointsMat.resize(rasterHeight);

	minRasterYMat = cv::Mat(rasterHeight, rasterWidth, CV_32FC1, cv::Scalar(maxCloudPoints_Y));
	maxRasterYMat = cv::Mat(rasterHeight, rasterWidth, CV_32FC1, cv::Scalar(minCloudPoints_Y));
	meanRasterBiasYMat = cv::Mat(rasterHeight, rasterWidth, CV_32FC1, cv::Scalar(0));

	for (int indexMapRow = 0; indexMapRow < rasterHeight; indexMapRow++) {
		indexMapMat[indexMapRow].resize(rasterWidth);
		croppedWallIndexMapMat[indexMapRow].resize(rasterWidth);
		meanCloudPointsMat[indexMapRow].resize(rasterWidth);
		for (int indexMapMatCol = 0; indexMapMatCol < rasterWidth; indexMapMatCol++) {
			indexMapMat[indexMapRow][indexMapMatCol].resize(0);
			croppedWallIndexMapMat[indexMapRow][indexMapMatCol].resize(0);
			meanCloudPointsMat[indexMapRow][indexMapMatCol].resize(3, 0);
		}
	}
	for (int id = 0; id < numPoints; id++) {
		//std::cout << pixelValue(id) << std::endl;
		int axisY = int(coordZ(id) / pixelHeight);
		int axisX = int(coordX(id) / pixelWidth);
		valueMat(axisY, axisX) += pixelValue(id);
		accBiasYValueMat(axisY, axisX) += biasY(id);
		countMat(axisY, axisX) += 1;
		idxMat(axisY, axisX) = id;
		indexMapMat[rasterHeight - axisY - 1][axisX].push_back(id);
		(meanCloudPointsMat[rasterHeight - axisY - 1][axisX]).conservativeResize(3, (meanCloudPointsMat[rasterHeight - axisY - 1][axisX]).cols() + 1);
		meanCloudPointsMat[rasterHeight - axisY - 1][axisX].col(meanCloudPointsMat[rasterHeight - axisY - 1][axisX].cols() - 1) =
			Eigen::Vector3f(cloudPoints[id].x, cloudPoints[id].y, cloudPoints[id].z);


		minRasterYMat.at<float>(axisY, axisX) = std::min<float>(minRasterYMat.at<float>(axisY, axisX), cloudPoints_mapY(id));
		maxRasterYMat.at<float>(axisY, axisX) = std::max<float>(maxRasterYMat.at<float>(axisY, axisX), cloudPoints_mapY(id));
		/*std::cout << "cloudPoints_mapY(id):" << cloudPoints_mapY(id) << std::endl;
		std::cout << "minRasterYMat.at<float>(axisY, axisX):" << minRasterYMat.at<float>(axisY, axisX) << std::endl;
		std::cout << "maxRasterYMat.at<float>(axisY, axisX):" << maxRasterYMat.at<float>(axisY, axisX) << std::endl;*/
	}

	rasterImg = cv::Mat::zeros(rasterHeight, rasterWidth, CV_8UC1);
	//pinHoleMask = cv::Mat::zeros(rasterHeight, rasterWidth, CV_8UC1);
	//std::cout << "countMat.rows()" << countMat.rows() << std::endl;
	//std::cout << "countMat.cols()" << countMat.cols() << std::endl;
	for (int i = 0; i < rasterImg.rows; ++i) {
		for (int j = 0; j < rasterImg.cols; ++j) {
			int startRow = max(0, i - 1);
			int startCol = max(0, j - 1);

			if (SMOOTH_RASTER_IMG) {
				if (distanceToCamera[j] < sparseDistanceToCameraThreshold1) {
					if (countMat(i, j) != 0) {
						rasterImg.at<uchar>(i, j) = int(valueMat(i, j) / countMat(i, j));
					}
				}
				else if (distanceToCamera[j] < sparseDistanceToCameraThreshold2) { // sparse cloud points
					int roiRows = min(2, rasterHeight - startRow);
					int roiCols = min(2, rasterWidth - startCol);
					if (countMat.block(startRow, startCol, roiRows, roiCols).sum() != 0) {
						rasterImg.at<uchar>(i, j) = int(valueMat.block(startRow, startCol, roiRows, roiCols).sum()
							/ countMat.block(startRow, startCol, roiRows, roiCols).sum());
					}
				}
				else {
					int roiRows = min(3, rasterHeight - startRow);
					int roiCols = min(3, rasterWidth - startCol);
					if (countMat.block(startRow, startCol, roiRows, roiCols).sum() != 0) {
						rasterImg.at<uchar>(i, j) = int(valueMat.block(startRow, startCol, roiRows, roiCols).sum()
							/ countMat.block(startRow, startCol, roiRows, roiCols).sum());
					}
				}
			}
			else { // normal raster
				if (countMat(i, j) != 0) {
					rasterImg.at<uchar>(i, j) = int(valueMat(i, j) / countMat(i, j));
					meanRasterBiasYMat.at<float>(i, j) = accBiasYValueMat(i, j) / countMat(i, j);
				}
				/*else
					pinHoleMask.at<uchar>(i, j) = 1;*/
			}
		}
	}

	//cv::Mat equalizedRasterImg;
	//cv::equalizeHist(rasterImg, equalizedRasterImg);
	//rasterImg = equalizedRasterImg.clone();
	//cv::flip(equalizedRasterImg, equalizedRasterImg, 0);

	//flip
	cv::flip(rasterImg, rasterImg, 0);
	//cv::flip(pinHoleMask, pinHoleMask, 0);
	cv::flip(minRasterYMat, minRasterYMat, 0);
	cv::flip(maxRasterYMat, maxRasterYMat, 0);
	cv::flip(meanRasterBiasYMat, meanRasterBiasYMat, 0);

	idxMat.rowwise().reverseInPlace();

	return true;
}

bool DetectObstacle::DetectWallObstacleFcn(PlaneCutResultInterface  &mPlane,
	WallObstacleInfo& wallObstacleInfo, bool onlyDetect)
{
	int origin_wall_nums = mPlane.plane_wall_idx.size();

	std::vector<std::vector<std::vector<cv::Point3f>>> wall_door_window_pos;

	
	for (int i = 0; i < mPlane.door_window_info.size(); i++)
	{
		std::vector<std::vector<cv::Point3f>> plane_door_window_pos;
		for (int j = 0; j < mPlane.door_window_info[i].size(); j++)
		{
			plane_door_window_pos.push_back(mPlane.door_window_info[i][j].corners);
		}
		wall_door_window_pos.push_back(plane_door_window_pos);
	}


	//INIT
	wallObstacleInfo.holeInfo.hole_infoVector_xz.resize(origin_wall_nums);
	wallObstacleInfo.holeInfo.hole_infoVector_origin.resize(origin_wall_nums);
	wallObstacleInfo.doorFrameInfo.wall_door_window_pos_3D.resize(origin_wall_nums);
	wallObstacleInfo.doorFrameInfo.rasterDoorFrameArrayVector.resize(origin_wall_nums);
	wallObstacleInfo.cropped_filtered_wall_xyz.resize(origin_wall_nums);
	wallObstacleInfo.removeCritical_filtered_wall_xyz.resize(origin_wall_nums);
	wallObstacleInfo.rasterImgVector.resize(origin_wall_nums);
	wallObstacleInfo.rasterDiagonalImgVector.resize(origin_wall_nums);
	wallObstacleInfo.coarseRotationMatVector.resize(origin_wall_nums);
	wallObstacleInfo.fineRotationMatVector.resize(origin_wall_nums);
	wallObstacleInfo.coarseRotationMatVector.resize(origin_wall_nums);
	wallObstacleInfo.fineRotationMatVector.resize(origin_wall_nums);
	wallObstacleInfo.backwardCoarseRotationMatVector.resize(origin_wall_nums);
	wallObstacleInfo.backwardFineRotationMatVector.resize(origin_wall_nums);
	wallObstacleInfo.coarseRotationEigenVector.resize(origin_wall_nums);
	wallObstacleInfo.fineRotationEigenVector.resize(origin_wall_nums);
	wallObstacleInfo.backwardCoarseRotationEigenVector.resize(origin_wall_nums);
	wallObstacleInfo.backwardFineRotationEigenVector.resize(origin_wall_nums);
	wallObstacleInfo.minCloudPointsXVector.resize(origin_wall_nums);
	wallObstacleInfo.minCloudPointsYVector.resize(origin_wall_nums);
	wallObstacleInfo.minCloudPointsZVector.resize(origin_wall_nums);
	wallObstacleInfo.deltaCloudPointsYVector.resize(origin_wall_nums);
	wallObstacleInfo.meanCloudPointsYVector.resize(origin_wall_nums);
	wallObstacleInfo.planeNormalVector.resize(origin_wall_nums);
	wallObstacleInfo.refinedMeanCloudPointsYVector.resize(origin_wall_nums);
	wallObstacleInfo.rasterHeightVector.resize(origin_wall_nums);
	wallObstacleInfo.rasterWidthVector.resize(origin_wall_nums);
	wallObstacleInfo.cabinetInfo.cabinetBoxArrayXZVector.resize(origin_wall_nums);
	wallObstacleInfo.cabinetInfo.rasterCabinetBoxArrayVector.resize(origin_wall_nums);
	wallObstacleInfo.doorFrameInfo.doorFrameArrayXZVector.resize(origin_wall_nums);
	wallObstacleInfo.obstacleMaskVector.resize(origin_wall_nums);
	wallObstacleInfo.doorFrameMaskVector.resize(origin_wall_nums);
	wallObstacleInfo.pixelHeight = pixelHeight;
	wallObstacleInfo.pixelWidth = pixelWidth;
	wallObstacleInfo.baseBoardInfo.rasterBaseBoardHeightVector = std::vector<int>(origin_wall_nums);
	wallObstacleInfo.minRasterYMatVector.resize(origin_wall_nums);
	wallObstacleInfo.maxRasterYMatVector.resize(origin_wall_nums);
	wallObstacleInfo.pinHoleMaskVector.resize(origin_wall_nums);
	wallObstacleInfo.holeMaskImgVector.resize(origin_wall_nums);
	wallObstacleInfo.indexMapMatVector.resize(origin_wall_nums);
	wallObstacleInfo.croppedWallIndexMapMatVector.resize(origin_wall_nums);
	wallObstacleInfo.meanCloudPointsMatVector.resize(origin_wall_nums);
	wallObstacleInfo.refinedPlaneNormalVector.resize(origin_wall_nums);
	wallObstacleInfo.isGlassWallVector.resize(origin_wall_nums);

	for (int i = 0; i < origin_wall_nums; i++) {
		wallObstacleInfo.refinedPlaneNormalVector[i] = Eigen::Vector3f::Constant(-1);
	}

	int wall_one_meter_count = 0;
	float wall_one_meter_mean = 0;
	for (origin_wall_id = 0; origin_wall_id < origin_wall_nums; origin_wall_id++) {
		if (mPlane.plane_xyz[mPlane.plane_wall_idx[origin_wall_id]].empty()) continue;

		// NOTE: wall_id != i
		wall_id = origin_wall_id;

		//set normal
		float plane_normal[3];
		plane_normal[0] = mPlane.plane_normals[mPlane.plane_wall_idx[origin_wall_id]].x;
		plane_normal[1] = mPlane.plane_normals[mPlane.plane_wall_idx[origin_wall_id]].y;
		plane_normal[2] = mPlane.plane_normals[mPlane.plane_wall_idx[origin_wall_id]].z;
		float wall_normal_uniform[3];
		float wall_center[3] = { mPlane.plane_center[mPlane.plane_wall_idx[origin_wall_id]].x, mPlane.plane_center[mPlane.plane_wall_idx[origin_wall_id]].y, mPlane.plane_center[mPlane.plane_wall_idx[origin_wall_id]].z };
		MeasureBase::UniformNormals(plane_normal, wall_center, wall_normal_uniform);

		//rotate to XZ plane
		std::vector<cv::Point3f> xzPlane;

		Rotate2XZPlane(mPlane.plane_xyz[mPlane.plane_wall_idx[origin_wall_id]], wall_normal_uniform, xzPlane);

		// raster		
		// raster_XZ_img(xzPlane); 
		std::vector<unsigned char> planeReflectValues = mPlane.plane_reflect[mPlane.plane_wall_idx[origin_wall_id]];
		raster_XZ_img_with_reflection(xzPlane, planeReflectValues);

		// DEBUG
		//cv::imwrite("depth_" + to_string(wall_id) + ".jpg", rasterImg);
		//cv::imwrite("reflect_" + to_string(wall_id) + ".jpg", reflectRasterImg);

		if (DEBUG_SAVE_ORIGIN_CLOUDPOINTS) {
			std::string cloudPointsFileName = "F:\\tmp\\reflect_" + to_string(wall_id) + ".txt";
			ofstream fileOutput(cloudPointsFileName, 'w');
			for (int i = 0; i < xzPlane.size(); i++)
			{
				fileOutput << xzPlane[i].x << '\t';
				fileOutput << xzPlane[i].y << '\t';
				fileOutput << xzPlane[i].z << '\t';
				fileOutput << mPlane.plane_xyz[mPlane.plane_wall_idx[origin_wall_id]][i] << std::endl;
			}
			fileOutput.close();
		}

		//std::cout << " zhujuniqng show rasterHeight == " << rasterHeight << endl;
		//std::cout << " zhujuniqng show rasterWidth == " << rasterWidth << endl;

		if (rasterHeight < MIN_RASTER_IMG_HEIGHT_THRESHOLD || rasterWidth < MIN_RASTER_IMG_WIDTH_THRESHOLD)
			continue;

		//DetectBeamAndBaseboard(xzPlane, wallObstacleInfo);
		bool isGlassWall = false;
		isGlassWall = DetectWallBoundaryBoard(xzPlane);


		bool inGlassWall = false;
		if (wall_door_window_pos.size() > mPlane.plane_wall_idx[origin_wall_id]) {
			inGlassWall = JudgeDoorInGlassWall(wall_door_window_pos[mPlane.plane_wall_idx[origin_wall_id]]);
		}
		isGlassWall = isGlassWall || inGlassWall;

		DetectDoorFrame(xzPlane, wall_door_window_pos, mPlane.plane_wall_idx[origin_wall_id], wallObstacleInfo);
		DetectCabinet(xzPlane, wallObstacleInfo);
		//DetectWallHole(xzPlane, wallObstacleInfo);
		//cout << "zhujunqing detect  end" << endl;
		// ***************  START : save to mask ****************
		// save obstacle mask
		wallObstacleInfo.obstacleMaskVector[origin_wall_id] = cv::Mat::zeros(rasterHeight, rasterWidth, CV_8UC1);
		wallObstacleInfo.doorFrameMaskVector[origin_wall_id] = cv::Mat::zeros(rasterHeight, rasterWidth, CV_8UC1);
		// save cabinet info to mask
		/*for (int box_id = 0; box_id < rasterCabinetBoxArray.rows(); box_id++) {
			Eigen::ArrayX4i box(rasterCabinetBoxArray.row(box_id));
			int boxMinX = min(box(0), box(2));
			int boxMinY = min(box(1), box(3));
			int boxMaxX = max(box(0), box(2));
			int boxMaxY = max(box(1), box(3));
			//std::cout << "cabinet box:" << box << std::endl;
			wallObstacleInfo.obstacleMaskVector[origin_wall_id](cv::Range(boxMinY, boxMaxY), cv::Range(boxMinX, boxMaxX)).setTo(1);
		}*/

		// save doorFrame info to mask
		//ParameterController &parameter_config = *ParameterController::GetInstance();
		float doorLeftRightDist_float;
		//parameter_config.Get_flatness_distance_to_plane_boundary_topbottom(doorLeftRightDist_float);
		doorLeftRightDist_float = 0.;
		int doorLeftRightDist = int(doorLeftRightDist_float / pixelWidth);
		for (int box_id = 0; box_id < rasterDoorFrameArray.rows(); box_id++) {
			Eigen::ArrayX4i box(rasterDoorFrameArray.row(box_id));
			int boxMinX = max(0, min(box(0), box(2)) - doorLeftRightDist);
			int boxMinY = min(box(1), box(3));
			int boxMaxX = min(rasterWidth, max(box(0), box(2)) + doorLeftRightDist);
			int boxMaxY = max(box(1), box(3));
			//std::cout << "rasterDoorFrame box:" << box << std::endl;
			wallObstacleInfo.obstacleMaskVector[origin_wall_id](cv::Range(boxMinY, boxMaxY), cv::Range(boxMinX, boxMaxX)).setTo(1);
			wallObstacleInfo.doorFrameMaskVector[origin_wall_id](cv::Range(boxMinY, boxMaxY), cv::Range(boxMinX, boxMaxX)).setTo(1);
		}
		// save beam info to mask
		if (rasterBeamHeight > 0) {
			//std::cout << "rasterDoorFrame box:" << box << std::endl;
			//modify by zhujunqing 20210508 begin
			if (wallObstacleInfo.obstacleMaskVector[origin_wall_id].rows > rasterBeamHeight) {
				wallObstacleInfo.obstacleMaskVector[origin_wall_id].rowRange(0, rasterBeamHeight).setTo(1);
			}
			//modify by zhujunqing 20210508 end
		}
		// save baseboard info to mask
		if (rasterBaseBoardHeight > 0) {
			//std::cout << "rasterDoorFrame box:" << box << std::endl;
			wallObstacleInfo.obstacleMaskVector[origin_wall_id].rowRange(rasterHeight - rasterBaseBoardHeight, rasterHeight).setTo(1);
		}
		// save hole to mask
		//cv::bitwise_or(holeResultImg, wallObstacleInfo.obstacleMaskVector[origin_wall_id], wallObstacleInfo.obstacleMaskVector[origin_wall_id]);

		// ***************  END : save to mask ****************
		// *************** START : refine meanY value and normal *******************
		Eigen::Matrix3Xf cloudPointsWithoutObstacles;
		// count refined cloud points number to be push
		int totalCloudPointsWithoutObstaclesCount = 0;
		for (int rowOfMask = 0; rowOfMask < wallObstacleInfo.obstacleMaskVector[origin_wall_id].rows; rowOfMask++) {
			for (int colOfMask = 0; colOfMask < wallObstacleInfo.obstacleMaskVector[origin_wall_id].cols; colOfMask++) {
				if (wallObstacleInfo.obstacleMaskVector[origin_wall_id].at<uchar>(rowOfMask, colOfMask) != 0)
					continue;
				totalCloudPointsWithoutObstaclesCount += indexMapMat[rowOfMask][colOfMask].size();
			}
		}
		cloudPointsWithoutObstacles.resize(3, totalCloudPointsWithoutObstaclesCount);
		// fill refined cloud points 
		int currentCloudPointsWithoutObstaclesCount = 0;
		for (int rowOfMask = 0; rowOfMask < wallObstacleInfo.obstacleMaskVector[origin_wall_id].rows; rowOfMask++) {
			for (int colOfMask = 0; colOfMask < wallObstacleInfo.obstacleMaskVector[origin_wall_id].cols; colOfMask++) {
				if (wallObstacleInfo.obstacleMaskVector[origin_wall_id].at<uchar>(rowOfMask, colOfMask) != 0)
					continue;
				for (int pushID = 0; pushID < indexMapMat[rowOfMask][colOfMask].size(); pushID++) {
					cv::Point3f& pushCloudPoint = xzPlane[indexMapMat[rowOfMask][colOfMask][pushID]];
					cloudPointsWithoutObstacles.col(currentCloudPointsWithoutObstaclesCount) = Eigen::Vector3f(pushCloudPoint.x, pushCloudPoint.y, pushCloudPoint.z);
					currentCloudPointsWithoutObstaclesCount += 1;
				}
			}
		}

		// calc normal
		//std::cout << "cloudPointsWithoutObstacles.rows:" << cloudPointsWithoutObstacles.rows() << std::endl;
		//std::cout << "cloudPointsWithoutObstacles.cols:" << cloudPointsWithoutObstacles.cols() << std::endl;
		if (cloudPointsWithoutObstacles.cols() > 4) { // number of remaining points must greater than ...
			Eigen::Vector3f cloudPointsWithoutObstaclesCenter = cloudPointsWithoutObstacles.rowwise().mean();
			Eigen::Matrix3Xf cloudPointsBiasWithoutObstacles = cloudPointsWithoutObstacles.colwise() - cloudPointsWithoutObstaclesCenter;
			auto svd = (cloudPointsBiasWithoutObstacles).jacobiSvd(Eigen::ComputeThinU);
			Eigen::Vector3f XZPlaneWithoutObstaclesNormal = svd.matrixU().rightCols<1>();
			if (XZPlaneWithoutObstaclesNormal.dot(cloudPointsWithoutObstaclesCenter) > 0)
				XZPlaneWithoutObstaclesNormal = -XZPlaneWithoutObstaclesNormal; // point to origin 
			//std::cout << "xz_plane normal: " << XZPlaneWithoutObstaclesNormal << std::endl;
			Eigen::Vector3f planeWithoutObstaclesNormal = backwardCoarseRotationEigen * backwardFineRotationEigen * XZPlaneWithoutObstaclesNormal;
			wallObstacleInfo.refinedPlaneNormalVector[origin_wall_id] = planeWithoutObstaclesNormal;

			// save refined meanZ
			// calc mode rather than mean
			Eigen::ArrayXf yValue = cloudPointsWithoutObstacles.row(1).array();
			//std::cout << "yValue" << yValue << std::endl;
			float modeValue;
			statsMode(yValue, 0.1, &modeValue);
			wallObstacleInfo.refinedMeanCloudPointsYVector[origin_wall_id] = modeValue;

			//std::cout << typeid(cloudPointsWithoutObstacles.row(2).cast<int>()).name() << std::endl;

			//DEBUG
			if (DEBUG_REFINE_NORMAL_AND_MEAN_VALUE) {
				std::cout << "origin_wall_id: " << origin_wall_id << std::endl;
				std::cout << "XZPlaneWithoutObstaclesNormal: " << XZPlaneWithoutObstaclesNormal.transpose() << std::endl;
				std::cout << "planeWithoutObstaclesNormal: " << planeWithoutObstaclesNormal.transpose() << std::endl;

				cv::Mat defectImg = rasterImg.clone();
				cv::cvtColor(defectImg, defectImg, cv::COLOR_GRAY2RGB);
				std::cout << "modeValue" << modeValue << std::endl;
				std::cout << "meanCloudPoints_Y" << meanCloudPoints_Y << std::endl;
				std::cout << "delta Y" << cloudPointsBiasWithoutObstacles.row(1).maxCoeff() - cloudPointsBiasWithoutObstacles.row(1).minCoeff() << std::endl;
				int modePixel = int(255 * (modeValue - minCloudPoints_Y) / (maxCloudPoints_Y - minCloudPoints_Y));
				std::cout << "modePixel" << modePixel << std::endl;
				defectImg.setTo(cv::Scalar(255, 0, 0), rasterImg == modePixel);
			}
		}
		// *************** END : refine meanY value and normal *******************

			// save rotate 45 obstacle mask
		//cv::Point2f imgCenter(rasterHeight/2.0, rasterWidth/2.0);
		//cv::Mat rotationMat = cv::getRotationMatrix2D(imgCenter, 45, 1.0);
		//cv::Mat rotatedMaskImg;
		//cv::warpAffine(obstacleInfo.obstacleMaskVector[origin_wall_id], rotatedMaskImg, );
		//if (wall_id != 5)
		//	continue;
		if (DEBUG_MASK) {
			cv::namedWindow("maskImg_" + to_string(wall_id));
			cv::imshow("maskImg", wallObstacleInfo.obstacleMaskVector[origin_wall_id] * 255);
			cv::waitKey(0);
		}

		// crop cloud points in raster
		//int wallRasterBeamHeight = obstacleInfo.beamInfo.rasterBeamHeightList[origin_wall_id];
		//int wallRasterBaseBoardHeight = obstacleInfo.baseBoardInfo.rasterBaseBoardHeightList[origin_wall_id];
		//for (int indexMapRow = 0; indexMapRow < rasterHeight; indexMapRow++) {
		//	for (int indexMapCol = 0; indexMapCol < rasterWidth; indexMapCol++) {
		//		// crop beam
		//		if (wallRasterBeamHeight > 0 && indexMapRow < wallRasterBeamHeight) {
		//			indexMapMat[indexMapRow][indexMapCol].clear();
		//			continue;
		//		}
		//		// crop baseboard
		//		if (wallRasterBaseBoardHeight > 0 && indexMapRow > (rasterHeight - wallRasterBaseBoardHeight - 1)) {
		//			indexMapMat[indexMapRow][indexMapCol].clear();
		//			continue;
		//		}
		//		// crop cabinet
		//		for (int box_id = 0; box_id < obstacleInfo.cabinetInfo.rasterCabinetBoxArray.rows(); box_id++) {
		//			Eigen::Array4i box(obstacleInfo.cabinetInfo.rasterCabinetBoxArray.row(box_id));
		//			if (min(box(0), box(2)) < indexMapRow && indexMapRow < max(box(0), box(2)) && min(box(1), box(3)) < indexMapCol && indexMapCol < max(box(1), box(3))) {
		//				indexMapMat[indexMapRow][indexMapCol].clear();
		//				continue;
		//			}
		//		}

		//	}
		//}

		// crop cloud points (origin_wall_id) in xz plane
		std::vector<cv::Point3f> croppedCloudPoints;
		int croppedCloudPointNum = 0;
		for (int point_id = 0; point_id < xzPlane.size(); point_id++) {
			cv::Point3f point = xzPlane[point_id];
			bool isSave = true;
			// crop cabinet
			//for (int box_id = 0; box_id < wallObstacleInfo.cabinetInfo.cabinetBoxArrayXZVector[origin_wall_id].rows(); box_id++) {
			//	Eigen::Array4i box(wallObstacleInfo.cabinetInfo.cabinetBoxArrayXZVector[origin_wall_id].row(box_id));
			////	if (min(box(0), box(2)) < point.x && point.x < max(box(0), box(2)) && min(box(1), box(3)) < point.z && point.z < max(box(1), box(3))) {
			//		isSave = false;
			//		break;
			//	}
			//}
			//modify by zhujunqing 20210518 begin
			// crop door frame
			/*for (int box_id = 0; box_id < wallObstacleInfo.doorFrameInfo.doorFrameArrayXZVector[origin_wall_id].rows(); box_id++) {
				Eigen::Array4i box(wallObstacleInfo.doorFrameInfo.doorFrameArrayXZVector[origin_wall_id].row(box_id));
				if (min(box(0), box(2)) < point.x && point.x < max(box(0), box(2)) && min(box(1), box(3)) < point.z && point.z < max(box(1), box(3))) {
					isSave = false;
					break;
				}
			}*/
			// if crop beam and baseboard, flatness and verticality may be wrong (crop beam and baseboard twice)
			//// crop beam
			//int curbeamHeight = wallObstacleInfo.beamInfo.rasterBeamHeightVector[origin_wall_id];
			//if ( curbeamHeight  > 0) {
			//	if (((rasterHeight - curbeamHeight - 1) * pixelHeight + minCloudPoints_Z) < point.z)
			//		isSave = false;
			//}
			//// crop baseboard
			//int curbaseBoardHeight = wallObstacleInfo.baseBoardInfo.rasterBaseBoardHeightVector[origin_wall_id];
			//if (curbaseBoardHeight > 0) {
			//	if ( point.z < (curbaseBoardHeight * pixelHeight + minCloudPoints_Z))
			//		isSave = false;
			//}
			 //modify by zhujunqing 20210518 end
			if (isSave) {
				croppedCloudPoints.push_back(point);
				int axisX = int((point.x - minCloudPoints_X) / pixelWidth);
				int axisY = int((point.z - minCloudPoints_Z) / pixelHeight);
				croppedWallIndexMapMat[rasterHeight - axisY - 1][axisX].push_back(croppedCloudPointNum);
				croppedCloudPointNum += 1;
			}
		}
		// copy croppedWallIndexMapMat data
		wallObstacleInfo.croppedWallIndexMapMatVector[origin_wall_id].resize(indexMapMat.size());
		for (int indexMapMatRow = 0; indexMapMatRow < indexMapMat.size(); indexMapMatRow++) {
			wallObstacleInfo.croppedWallIndexMapMatVector[origin_wall_id][indexMapMatRow].resize(indexMapMat[indexMapMatRow].size());
			for (int indexMapMatCol = 0; indexMapMatCol < indexMapMat[indexMapMatRow].size(); indexMapMatCol++) {
				wallObstacleInfo.croppedWallIndexMapMatVector[origin_wall_id][indexMapMatRow][indexMapMatCol].assign(croppedWallIndexMapMat[indexMapMatRow][indexMapMatCol].begin(), croppedWallIndexMapMat[indexMapMatRow][indexMapMatCol].end());
			}
		}
		//IOData::SavePoint3fData("zhujunqing_croppedCloudPoints_"+to_string(origin_wall_id)+"_.txt", croppedCloudPoints);
		// rotate cropped cloud points back to origin(3d) space
		std::vector<cv::Point3f> coarseCroppedCloudPoints;
		MeasureBase::RotatePoints(croppedCloudPoints, backwardFineRotationMat, coarseCroppedCloudPoints);
		MeasureBase::RotatePoints(coarseCroppedCloudPoints, backwardCoarseRotationMat, wallObstacleInfo.cropped_filtered_wall_xyz[origin_wall_id]);



		///////////// remove cabinet  door frame and wall boundary  for find room square
		std::vector<cv::Point3f> removeCriticalCloudPoints;
		float zxPlane_minmax_xyz[6];
		MeasureBase::FindPointsMinMaxXYZ(xzPlane, zxPlane_minmax_xyz);
		for (int point_id = 0; point_id < xzPlane.size(); point_id++) {
			cv::Point3f point = xzPlane[point_id];
			bool isSave = true;
			// crop cabinet
			for (int box_id = 0; box_id < wallObstacleInfo.cabinetInfo.cabinetBoxArrayXZVector[origin_wall_id].rows(); box_id++) {
				Eigen::Array4i box(wallObstacleInfo.cabinetInfo.cabinetBoxArrayXZVector[origin_wall_id].row(box_id));
				if (min(box(0), box(2)) < point.x && point.x < max(box(0), box(2)) && min(box(1), box(3)) < point.z && point.z < max(box(1), box(3))) {
					isSave = false;
					break;
				}
			}
			// crop door frame
			for (int box_id = 0; box_id < wallObstacleInfo.doorFrameInfo.doorFrameArrayXZVector[origin_wall_id].rows(); box_id++) {
				Eigen::Array4i box(wallObstacleInfo.doorFrameInfo.doorFrameArrayXZVector[origin_wall_id].row(box_id));
				if (min(box(0), box(2)) < point.x && point.x < max(box(0), box(2)) && min(box(1), box(3)) < point.z && point.z < max(box(1), box(3))) {
					isSave = false;
					break;
				}
			}
		    //remove wall boundary
			if (point.x< zxPlane_minmax_xyz[0] + 30 || point.x>zxPlane_minmax_xyz[1] - 30 || point.z< zxPlane_minmax_xyz[4] + 30 || point.z>zxPlane_minmax_xyz[5] - 30)
			{
				isSave = false;
			}

			if (isSave) {
				removeCriticalCloudPoints.push_back(point);
			}
		}
		std::vector<cv::Point3f> coarseRemoveCriticalCloudPoints;
		MeasureBase::RotatePoints(removeCriticalCloudPoints, backwardFineRotationMat, coarseRemoveCriticalCloudPoints);
		MeasureBase::RotatePoints(coarseRemoveCriticalCloudPoints, backwardCoarseRotationMat, wallObstacleInfo.removeCritical_filtered_wall_xyz[origin_wall_id]);

		//////end



		if (DEBUG_SAVE_CROPPED_CLOUDPOINTS) {
			std::string cloudPointsFileName = "F:\\tmp\\cropped_" + to_string(wall_id) + ".txt";
			ofstream fileOutput(cloudPointsFileName, 'w');
			for (int i = 0; i < croppedCloudPoints.size(); i++)
			{
				fileOutput << wallObstacleInfo.cropped_filtered_wall_xyz[origin_wall_id][i].x << '\t';
				fileOutput << wallObstacleInfo.cropped_filtered_wall_xyz[origin_wall_id][i].y << '\t';
				fileOutput << wallObstacleInfo.cropped_filtered_wall_xyz[origin_wall_id][i].z << std::endl;
			}
			fileOutput.close();
		}
	
		// copy data to obstacleInfo
		wallObstacleInfo.isGlassWallVector[origin_wall_id] = isGlassWall;
		wallObstacleInfo.rasterImgVector[origin_wall_id] = rasterImg.clone();
		wallObstacleInfo.coarseRotationMatVector[origin_wall_id] = coarseRotationMat.clone();
		wallObstacleInfo.fineRotationMatVector[origin_wall_id] = fineRotationMat.clone();
		wallObstacleInfo.backwardCoarseRotationMatVector[origin_wall_id] = backwardCoarseRotationMat.clone();
		wallObstacleInfo.backwardFineRotationMatVector[origin_wall_id] = backwardFineRotationMat.clone();
		wallObstacleInfo.coarseRotationMatVector[origin_wall_id] = coarseRotationMat.clone();
		wallObstacleInfo.fineRotationMatVector[origin_wall_id] = fineRotationMat.clone();
		cv::cv2eigen(coarseRotationMat, wallObstacleInfo.coarseRotationEigenVector[origin_wall_id]);
		cv::cv2eigen(fineRotationMat, wallObstacleInfo.fineRotationEigenVector[origin_wall_id]);
		cv::cv2eigen(backwardCoarseRotationMat, wallObstacleInfo.backwardCoarseRotationEigenVector[origin_wall_id]);
		cv::cv2eigen(backwardFineRotationMat, wallObstacleInfo.backwardFineRotationEigenVector[origin_wall_id]);
		wallObstacleInfo.minRasterYMatVector[origin_wall_id] = minRasterYMat.clone();
		wallObstacleInfo.maxRasterYMatVector[origin_wall_id] = maxRasterYMat.clone();
		//wallObstacleInfo.meanRasterBiasYMatVector[origin_wall_id] = meanRasterBiasYMat.clone();
		wallObstacleInfo.holeMaskImgVector[origin_wall_id] = holeResultImg.clone();
		wallObstacleInfo.pinHoleMaskVector[origin_wall_id] = pinHoleMask.clone();
		wallObstacleInfo.minCloudPointsXVector[origin_wall_id] = minCloudPoints_X;
		wallObstacleInfo.minCloudPointsZVector[origin_wall_id] = minCloudPoints_Z;
		wallObstacleInfo.meanCloudPointsYVector[origin_wall_id] = meanCloudPoints_Y;
		wallObstacleInfo.rasterHeightVector[origin_wall_id] = rasterHeight;
		wallObstacleInfo.rasterWidthVector[origin_wall_id] = rasterWidth;
		wallObstacleInfo.indexMapMatVector[origin_wall_id].resize(indexMapMat.size());
		wallObstacleInfo.planeNormalVector[origin_wall_id] = Eigen::Vector3f(plane_normal[0], plane_normal[1], plane_normal[2]);
		for (int indexMapMatRow = 0; indexMapMatRow < indexMapMat.size(); indexMapMatRow++) {
			wallObstacleInfo.indexMapMatVector[origin_wall_id][indexMapMatRow].resize(indexMapMat[indexMapMatRow].size());
			for (int indexMapMatCol = 0; indexMapMatCol < indexMapMat[indexMapMatRow].size(); indexMapMatCol++) {
				wallObstacleInfo.indexMapMatVector[origin_wall_id][indexMapMatRow][indexMapMatCol].assign(indexMapMat[indexMapMatRow][indexMapMatCol].begin(), indexMapMat[indexMapMatRow][indexMapMatCol].end());
			}
		}
		wallObstacleInfo.meanCloudPointsMatVector[origin_wall_id].resize(rasterHeight);
		for (int meanCloudPointsRow = 0; meanCloudPointsRow < rasterHeight; meanCloudPointsRow++) {
			wallObstacleInfo.meanCloudPointsMatVector[origin_wall_id][meanCloudPointsRow].resize(rasterWidth);
			for (int meanCloudPointsCol = 0; meanCloudPointsCol < rasterWidth; meanCloudPointsCol++) {
				if (meanCloudPointsMat[meanCloudPointsRow][meanCloudPointsCol].cols() < 1) {
					wallObstacleInfo.meanCloudPointsMatVector[origin_wall_id][meanCloudPointsRow][meanCloudPointsCol] = Eigen::Vector3f(0, 0, 0); // set wrong value
				}
				else {
					wallObstacleInfo.meanCloudPointsMatVector[origin_wall_id][meanCloudPointsRow][meanCloudPointsCol] = meanCloudPointsMat[meanCloudPointsRow][meanCloudPointsCol].rowwise().mean();
				}
			}
		}
	}
	//cout << "zhujunqing start update mPlane" << endl;
	//update mPlane 
	for (int i = 0; i < origin_wall_nums; i++)
	{
		if (wallObstacleInfo.cropped_filtered_wall_xyz[i].size() != 0)
			mPlane.plane_xyz[mPlane.plane_wall_idx[i]] = wallObstacleInfo.cropped_filtered_wall_xyz[i];
	}


#if 0
	for (int i = 0; i < origin_wall_nums; i++)
	{
		for (int j = 0; j < wallObstacleInfo.doorFrameInfo.wall_door_window_pos_3D[i].size(); j++)
		{
			mPlane.door_window_info[mPlane.plane_wall_idx[i]][j].corners = wallObstacleInfo.doorFrameInfo.wall_door_window_pos_3D[i][j];
		}
	}
#endif

	if (onlyDetect) {
		//plane x,y,z
		std::vector<std::vector<cv::Point3f>> plane_xyz;

		//plane reflectance
		std::vector<std::vector<unsigned char>> plane_reflect;

		//plane normals
		std::vector<cv::Point3f> plane_normals;

		//plane center
		std::vector<cv::Point3f> plane_center;

		//plane ground index
		std::vector<int> plane_ground_idx;

		//plane ceiling
		std::vector<int> plane_ceiling_idx;

		//plane wall
		std::vector<int> plane_wall_idx;

		//plane beam
		std::vector<int> plane_beam_idx;

		//connected and perpendicular wall index
		std::vector<std::pair<int, int>> L_shape_plane_idx;

		//connected and perpendicular wall index
		std::vector<std::pair<int, int>> parallel_plane_idx;

		//door window corner
		std::vector<std::vector<MeasureDoorWindow::DoorWindowInfo>> door_window_info;

		//plane flateness_defect
		std::vector<std::pair<int, std::vector<std::pair<float, cv::Point3f>>>> flateness_defect;


		int newWllid = 0;
		std::vector<int> glassWallId;
		std::vector<std::pair<int, int>> oldAndNewIds;

		///
		std::vector<std::vector<cv::Point3f>> removeCritical_filtered_wall_xyz;

		for (int i = 0; i < wallObstacleInfo.isGlassWallVector.size(); i++)
		{
			if (!wallObstacleInfo.isGlassWallVector[i])
			{
				plane_xyz.push_back(mPlane.plane_xyz[i]);
				plane_reflect.push_back(mPlane.plane_reflect[i]);
				plane_normals.push_back(mPlane.plane_normals[i]);
				plane_center.push_back(mPlane.plane_center[i]);
				plane_wall_idx.push_back(newWllid);
				door_window_info.push_back(mPlane.door_window_info[i]);
				oldAndNewIds.push_back(std::pair<int, int>(i, newWllid));
				////need to update more wallObstacleInfo information
				removeCritical_filtered_wall_xyz.push_back(wallObstacleInfo.removeCritical_filtered_wall_xyz[i]);

				newWllid++;
			}
			else
			{
				glassWallId.push_back(i);
			}
		}
		if (glassWallId.size() > 0) {
			for (int j = wallObstacleInfo.isGlassWallVector.size(); j < mPlane.plane_xyz.size(); j++)
			{
				plane_xyz.push_back(mPlane.plane_xyz[j]);
				plane_reflect.push_back(mPlane.plane_reflect[j]);
				plane_normals.push_back(mPlane.plane_normals[j]);
				plane_center.push_back(mPlane.plane_center[j]);
				newWllid++;
			}
			for (size_t i = 0; i < mPlane.plane_ground_idx.size(); i++)
			{
				plane_ground_idx.push_back(mPlane.plane_ground_idx[i] - (mPlane.plane_xyz.size() - newWllid));
			}
			for (size_t i = 0; i < mPlane.plane_ceiling_idx.size(); i++)
			{
				plane_ceiling_idx.push_back(mPlane.plane_ceiling_idx[i] - (mPlane.plane_xyz.size() - newWllid));
			}

			for (size_t i = 0; i < mPlane.plane_beam_idx.size(); i++)
			{
				bool isGlass = false;
				int glassNum = 0;
				for (int j = 0; j < glassWallId.size(); j++)
				{
					if (mPlane.plane_beam_idx[i] == glassWallId[j])
					{
						isGlass = true;
						glassNum++;
						break;
					}
				}
				if (!isGlass)
				{
					plane_beam_idx.push_back(mPlane.plane_beam_idx[i] - glassNum);
				}
			}

			for (size_t i = 0; i < mPlane.L_shape_plane_idx.size(); i++)
			{
				bool isGlass = false;
				for (int j = 0; j < glassWallId.size(); j++)
				{
					if (mPlane.L_shape_plane_idx[i].first == glassWallId[j] || mPlane.L_shape_plane_idx[i].second == glassWallId[j])
					{
						isGlass = true;
						break;
					}
				}
				if (!isGlass)
				{
					int newFirstId = -1;
					int newSecondId = -1;
					for (int id = 0; id < oldAndNewIds.size(); id++)
					{
						if (mPlane.L_shape_plane_idx[i].first == oldAndNewIds[id].first)
						{
							newFirstId = oldAndNewIds[id].second;
						}
						if (mPlane.L_shape_plane_idx[i].second == oldAndNewIds[id].first)
						{
							newSecondId = oldAndNewIds[id].second;
						}
					}
					if (newFirstId != -1 && newSecondId != -1)
					{
						L_shape_plane_idx.push_back(std::pair<int, int>{newFirstId, newSecondId});
					}
				}
			}

			for (size_t i = 0; i < mPlane.parallel_plane_idx.size(); i++)
			{
				bool isGlass = false;
				for (int j = 0; j < glassWallId.size(); j++)
				{
					if (mPlane.parallel_plane_idx[i].first == glassWallId[j] || mPlane.parallel_plane_idx[i].second == glassWallId[j])
					{
						isGlass = true;
						break;
					}
				}
				if (!isGlass)
				{
					int newFirstId = -1;
					int newSecondId = -1;
					for (int id = 0; id < oldAndNewIds.size(); id++)
					{
						if (mPlane.parallel_plane_idx[i].first == oldAndNewIds[id].first)
						{
							newFirstId = oldAndNewIds[id].second;
						}
						if (mPlane.parallel_plane_idx[i].second == oldAndNewIds[id].first)
						{
							newSecondId = oldAndNewIds[id].second;
						}
					}
					if (newFirstId != -1 && newSecondId != -1)
					{
						parallel_plane_idx.push_back(std::pair<int, int>{newFirstId, newSecondId});
					}
				}
			}

			mPlane.plane_xyz = plane_xyz;
			mPlane.plane_reflect = plane_reflect;
			mPlane.plane_normals = plane_normals;
			mPlane.plane_center = plane_center;
			mPlane.plane_ground_idx = plane_ground_idx;
			mPlane.plane_ceiling_idx = plane_ceiling_idx;
			mPlane.plane_wall_idx = plane_wall_idx;
			mPlane.plane_beam_idx = plane_beam_idx;
			mPlane.L_shape_plane_idx = L_shape_plane_idx;
			mPlane.parallel_plane_idx = parallel_plane_idx;
			mPlane.door_window_info = door_window_info;
			//need to update more wallObstacleInfo information
			wallObstacleInfo.removeCritical_filtered_wall_xyz = removeCritical_filtered_wall_xyz;
		}
	}
	return true;
}
bool DetectObstacle::DetectBeamAndBaseboard(const std::vector<cv::Point3f>& cloudPoints, WallObstacleInfo& obstacleInfo) {
	//DEBUG: detection wall_id wall
	//if (!(wall_id == 19)) 
	//	return false;

	//std::cout << "wall_id:" << wall_id << std::endl;

	// beamResult: x,y,z,w,h
	std::vector<float> beamResult(5);
	if (DetectBeam(cloudPoints, beamResult)) {
		// rotate beam point to origin space
		cv::Point3f Pointxz{ beamResult[0],beamResult[1],beamResult[2] };
		cv::Point3f beamCoarsePoint3d, beamOriginPoint;
		MeasureBase::RotatePoint(Pointxz, backwardFineRotationMat, beamCoarsePoint3d);
		MeasureBase::RotatePoint(beamCoarsePoint3d, backwardCoarseRotationMat, beamOriginPoint);
		// save result (wall_id : height, point)
		obstacleInfo.beamInfo.beamMap.insert({ wall_id, std::pair<float, cv::Point3f>{beamResult[4] * pixelHeight, beamOriginPoint} });
		obstacleInfo.beamInfo.rasterBeamHeightVector[origin_wall_id] = beamResult[4];
	}

	//baseBoardResult: x,y,z,w,h
	float baseBoardHeight;
	if (DetectBaseBoard(cloudPoints, baseBoardHeight)) {
		// save result (wall_id : height, point)
		obstacleInfo.baseBoardInfo.baseBoardMap.insert({ wall_id, baseBoardHeight * pixelHeight });
		obstacleInfo.baseBoardInfo.rasterBaseBoardHeightVector[origin_wall_id] = baseBoardHeight;
	}

	return true;
}

void DetectObstacle::statsMode(const Eigen::ArrayXf& data, const float resolution, float* result)
{
	float minValue = data.minCoeff();
	float maxValue = data.maxCoeff();
	Eigen::ArrayXf sequenceData = data - minValue;
	//std::cout << "sequenceData:" << sequenceData << std::endl;

	int bin = int((maxValue - minValue) / resolution) + 1;
	//std::cout << "bin:" << bin << std::endl;
	std::vector<int> modeMap(bin, 0);
	for (int i = 0; i < sequenceData.size(); i++) {
		modeMap[int(sequenceData(i) / resolution)] += 1;
	}
	int maxIdx = std::max_element(modeMap.begin(), modeMap.end()) - modeMap.begin();
	/*std::cout << "*std::max_element(modeMap.begin(), modeMap.end()):" << *std::max_element(modeMap.begin(), modeMap.end()) << std::endl;
	std::cout << "*modeMap.begin():" << *modeMap.begin() << std::endl;
	std::cout << "maxIdx:" << maxIdx << std::endl;*/
	*result = minValue + maxIdx * resolution;
}

bool DetectObstacle::DetectBeam(const std::vector<cv::Point3f>& cloudPoints, std::vector<float>& beamResult) {
	//read cloud points file 
	/*std::string filename = "F:\\BoxDetection\\chengduzhuwo\\BoxDetectionchengduzhuwo_2.txt";
	std::ifstream file(filename);
	std::string fileX;
	std::string fileY;
	std::string fileZ;
	std::string dataLine;
	std::vector<float> cloudPoints_X;
	std::vector<float> cloudPoints_Y;
	std::vector<float> cloudPoints_Z;
	while (std::getline(file, dataLine)) {
		std::vector<std::string> tokens;
		split(dataLine, tokens, "\t");
		cloudPoints_X.push_back(std::stof(tokens[0]));
		cloudPoints_Y.push_back(std::stof(tokens[1]));
		cloudPoints_Z.push_back(std::stof(tokens[2]));
	}*/

	// clear history value
	rasterBeamHeight = 0;

	cv::Mat filterImg = rasterImg.clone();
	for (int i = 0; i < filterImg.rows; ++i) {
		for (int j = 0; j < filterImg.cols; ++j) {
			if (filterImg.at<uchar>(i, j) == 0) {
				filterImg.at<uchar>(i, j) = 110;//125 to 110 by zhujunqing for beam detect
			}
		}
	}
	cv::namedWindow("filterImg");
	cv::imshow("filterImg", filterImg);
	cv::waitKey(0);


	//cv::Mat blurImg;
	//cv::medianBlur(filterImg, blurImg, 3);
	//cv::namedWindow("medianBlur");
	//cv::imshow("medianBlur", blurImg);
	//cv::waitKey(0);




	/************ dilate & erode projected 2D image **********/

	cv::Mat projected_2Dimage_dilated;
	cv::dilate(filterImg, projected_2Dimage_dilated, cv::Mat::ones(15, 15, CV_8U));

	cv::namedWindow("projected_2Dimage_dilated");
	cv::imshow("projected_2Dimage_dilated", projected_2Dimage_dilated);
	cv::waitKey(0);

	cv::Mat projected_2Dimage_eroded;
	cv::erode(projected_2Dimage_dilated, projected_2Dimage_eroded, cv::Mat::ones(5, 5, CV_8U));

	cv::namedWindow("projected_2Dimage_eroded");
	cv::imshow("projected_2Dimage_eroded", projected_2Dimage_eroded);
	cv::waitKey(0);
	// erode dilate
	/*
	cv::Mat projected_2Dimage_eroded;
	cv::erode(GblurImg, projected_2Dimage_eroded, cv::Mat::ones(5, 5, CV_8U));

	cv::namedWindow("projected_2Dimage_eroded");
	cv::imshow("projected_2Dimage_eroded", projected_2Dimage_eroded);
	cv::waitKey(0);

	cv::Mat projected_2Dimage_dilated;
	cv::dilate(projected_2Dimage_eroded, projected_2Dimage_dilated, cv::Mat::ones(5, 5, CV_8U));

	cv::namedWindow("projected_2Dimage_dilated");
	cv::imshow("projected_2Dimage_dilated", projected_2Dimage_dilated);
	cv::waitKey(0);
	*/
	//cv::waitKey(0);
	//cv::Mat convertScale;
	//cv::convertScaleAbs(GblurImg, convertScale,1.5,10);


	//cv::namedWindow("convertScale");
	//cv::imshow("convertScale", convertScale);
	//cv::waitKey(0);

	//cv::Mat lPMat;
	//cv::Laplacian(GblurImg, lPMat, CV_8U);
	//cv::namedWindow("lPMat");
	//cv::imshow("lPMat", lPMat);
	//cv::waitKey(0);


	//cv::Mat binaryImg = blurImg.clone();
	//cv::threshold(convertScale, binaryImg, 0, 255, cv::THRESH_OTSU);
	//binaryImg = GblurImg.clone();
	//cv::namedWindow("testbeamImg");
	//cv::imshow("testbeamImg", binaryImg);
	//cv::waitKey(0);


	//cv::Mat grad_x, grad_y;
	//cv::Mat abs_grad_x, abs_grad_y, dst;
	//x 
	//cv::Sobel(projected_2Dimage_eroded, grad_x,CV_16S,1,0,3,1,1,cv::BORDER_DEFAULT);
	//cv::convertScaleAbs(grad_x, abs_grad_x);

	//cv::Sobel(projected_2Dimage_eroded, grad_y, CV_16S, 0, 1, 3, 1, 1, cv::BORDER_DEFAULT);
	//cv::convertScaleAbs(grad_y, abs_grad_y);

	//cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, dst);
	//dst.convertTo(dst, CV_8UC1);


	//cv::namedWindow("Sobel");
	//cv::imshow("Sobel", dst);
	//cv::waitKey(0);

	cv::Mat GblurImg;
	cv::GaussianBlur(projected_2Dimage_eroded, GblurImg, cv::Size(1, 5), 5);
	cv::namedWindow("GblurImg");
	cv::imshow("GblurImg", GblurImg);
	cv::waitKey(0);

	cv::Mat binaryImg;
	cv::threshold(GblurImg, binaryImg, 0, 255, cv::THRESH_OTSU);
	cv::namedWindow("threshold");
	cv::imshow("threshold", binaryImg);
	cv::waitKey(0);

	//test 
	cv::Mat cannyImg;
	std::vector<cv::Vec4i> lines;
	cv::Canny(binaryImg, cannyImg, 50, 150);
	cv::HoughLinesP(cannyImg, lines, 1, CV_PI / 180, 50, 0.5 * cannyImg.cols, 0.03 * cannyImg.cols);


	cv::namedWindow("cannyImg");
	cv::imshow("cannyImg", cannyImg);
	cv::waitKey(0);
	cv::Mat compHoughImg;
	cv::cvtColor(cannyImg, compHoughImg, cv::COLOR_GRAY2RGB);
	for (size_t i = 0; i < lines.size(); i++)
	{
		cv::line(compHoughImg, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 0, 255), 3, 8);
	}

	cv::namedWindow("compHoughImg");
	cv::imshow("compHoughImg", compHoughImg);
	cv::waitKey(0);



	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(cannyImg, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	if (contours.size() == 0) {
		return false;
	}

	//cout << "zhujunqing find counters === " << contours.size() << endl;
	//binaryImg = 0;
	//cv::drawContours(binaryImg, contours,-1,cv::Scalar(125),1);

	//cv::namedWindow("contours");
	//cv::imshow("contours", binaryImg);
	//cv::waitKey(0);


	for (auto cnt : contours) {
		cv::Rect bbox = cv::boundingRect(cnt);
		int x(bbox.x), y(bbox.y), w(bbox.width), h(bbox.height);
		//filter
		if (y + h > 80 || w < h || w < 30)	continue;
		cv::Mat roi = rasterImg.row(int(y + h / 2));
		//std::cout << "roi" << roi << std::endl;
		int lenHole = 0;
		for (int j = 0; j < roi.cols; j++) {
			//std::cout << "roi.at<uchar>(0, j)" << roi.at<uchar>(0, j) << std::endl;
			if (roi.at<uchar>(0, j) == 0 && (j < x || j >(x + w))) lenHole += 1;
		}
		if (float(lenHole + w) / float(rasterImg.cols) < 0.85) continue;

		// save result
		float top_PointID_x = cloudPoints[idxMat(y, x)].x;
		float top_PointID_y = cloudPoints[idxMat(y, x)].y;
		float top_PointID_z = cloudPoints[idxMat(y, x)].z;
		float beamWidth = w;
		float beamHeight = h + beamExpandingValue; // return value
		rasterBeamHeight = h + beamExpandingValue; // member value
		beamResult = std::vector<float>{ top_PointID_x,top_PointID_y, top_PointID_z, beamWidth, beamHeight };

		if (DEBUG_BEAM || SAVE_OBSTACLES) {
			/*std::cout << "top_PointID_x:" << top_PointID_x << std::endl;
			std::cout << "top_PointID_y:" << top_PointID_y << std::endl;
			std::cout << "top_PointID_z:" << top_PointID_z << std::endl;
			std::cout << "beamHeight:" << beamHeight << "cm" << std::endl;*/
			cv::Mat beamImg;
			cv::cvtColor(rasterImg, beamImg, cv::COLOR_GRAY2RGB);
			cv::rectangle(beamImg, cv::Point(x, y), cv::Point(x + w, y + h), cv::Scalar(0, 255, 0), 1, 16);
			if (DEBUG_BEAM) {
				cv::namedWindow("beamImg");
				cv::imshow("beamImg", beamImg);
				cv::waitKey(0);
			}
			if (SAVE_OBSTACLES) {
				cv::imwrite(SAVE_OBSTACLES_DIR + "/beam_" + to_string(wall_id) + ".jpg", beamImg);
			}
		}

		return true;
	}
	//if (DEBUG_BEAM) {
	//	cv::namedWindow("rasterImg");
	//	cv::imshow("rasterImg", rasterImg);
	//	cv::waitKey(0);
	//}

	return false;
}

bool DetectObstacle::DetectWallBoundaryBoard(const std::vector<cv::Point3f>& cloudPoints)
{
	string SAVE_BOUNDARY_DIR;

	// clear history value
	rasterBaseBoardHeight = 0;
	cv::Mat clipedImg = rasterImg.rowRange(rasterImg.rows - clippedBaseboardHeight, rasterImg.rows);
	cv::Mat blurImg;
	cv::GaussianBlur(clipedImg, blurImg, cv::Size(5, 1), 5);

	// transpose
	cv::transpose(blurImg, blurImg);

	cv::Mat statMat;
	// add where(cols !=0)
	// where(==0) + delete(axis=1)
	for (int i = 0; i < blurImg.rows; i++) {
		bool isColNonZero = true;
		for (int j = 0; j < blurImg.cols; j++) {
			if (blurImg.at<uchar>(i, j) == 0) {
				isColNonZero = false;
				break;
			}
		}
		if (isColNonZero) {
			statMat.push_back(blurImg.row(i));
		}
	}
	cv::transpose(statMat, statMat);

	cv::Mat compStatMat;
	// add where(cols == 0)
	// where(==0) + unique + sort
	for (int i = 0; i < blurImg.rows; i++) {
		bool isColExistZero = false;
		for (int j = 0; j < blurImg.cols; j++) {
			if (blurImg.at<uchar>(i, j) == 0) {
				isColExistZero = true;
				break;
			}
		}
		if (isColExistZero) {
			compStatMat.push_back(blurImg.row(i));
		}
	}
	cv::transpose(compStatMat, compStatMat);

	//transpose
	cv::transpose(blurImg, blurImg);

	SAVE_OBSTACLES_DIR_PREFIX = "D:/DEBUG";
	SAVE_OBSTACLES = _access(SAVE_OBSTACLES_DIR_PREFIX.c_str(), 0) == 0 ? true : false;
	SAVE_BOUNDARY_DIR = "rst_file_baseBoard";

	if (SAVE_OBSTACLES && _access(SAVE_BOUNDARY_DIR.c_str(), 0) != 0) {
		_mkdir(SAVE_BOUNDARY_DIR.c_str());
	}
	else if (SAVE_OBSTACLES && !clearObstaclesDir_baseBoard) {
		std::string delFile = "del /a /f /q " + SAVE_BOUNDARY_DIR + "/*.jpg";
		replace(delFile.begin() + 11, delFile.end(), '/', '\\');
		//std::cout << delFile << std::endl;
		system(delFile.c_str());
		clearObstaclesDir_baseBoard = true;
	}

	//detect
	std::vector<cv::Vec4i> lines;
	if (statMat.cols > 0) {
		cv::Mat cannyImg;
		cv::Canny(statMat, cannyImg, cannyMinThreshold, cannyMaxThreshold);
		cv::HoughLinesP(cannyImg, lines, 1, CV_PI / 180, 25, statMat.rows, 0.4*statMat.rows);
		if (DEBUG_BASE_BOARD)
		{

			cv::Mat houghImg;
			cv::cvtColor(cannyImg, houghImg, cv::COLOR_GRAY2RGB);
			for (size_t i = 0; i < lines.size(); i++)
			{
				cv::line(houghImg, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 0, 255), 3, 8);
			}


			cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_blurImg_.jpg", blurImg);
			cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_statMat_.jpg", statMat);
			cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_cannyImg_.jpg", cannyImg);
			cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_houghImg_.jpg", houghImg);
		}

	}
	std::vector<cv::Vec4i> compLines;
	if (compStatMat.cols > 0) {
		cv::Mat compCannyImg;
		cv::Canny(compStatMat, compCannyImg, 50, 150);
		cv::HoughLinesP(compCannyImg, compLines, 1, CV_PI / 180, 25, compStatMat.rows, 0.4*compStatMat.rows);
		if (DEBUG_BASE_BOARD)
		{
			//// DEBUG
			//cv::namedWindow("compStatMat");
			//cv::imshow("compStatMat", compStatMat);
			//cv::namedWindow("cannyStatMat");
			//cv::imshow("cannyStatMat", compCannyImg);
			cv::Mat compHoughImg;
			cv::cvtColor(compCannyImg, compHoughImg, cv::COLOR_GRAY2RGB);
			for (size_t i = 0; i < compLines.size(); i++)
			{
				cv::line(compHoughImg, cv::Point(compLines[i][0], compLines[i][1]), cv::Point(compLines[i][2], compLines[i][3]), cv::Scalar(0, 0, 255), 3, 8);
			}
			//cv::namedWindow("compHoughImg");
			//cv::imshow("compHoughImg", compHoughImg);




			cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_compStatMat_.jpg", compStatMat);
			cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_compCannyImg_.jpg", compCannyImg);
			cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_compHoughImg_.jpg", compHoughImg);
		}
	}

	int h_value = 0;
	if ((statMat.cols > 0.5 * blurImg.cols) && lines.size() > 0) {
		std::vector<int> heightList(lines.size());
		for (int i = 0; i < lines.size(); i++)
			heightList[i] = lines[i][1];
		h_value = *std::min_element(heightList.begin(), heightList.end());
	}
	else if (compLines.size() > 0) {
		std::vector<int> heightList(compLines.size());
		for (int i = 0; i < compLines.size(); i++)
			heightList[i] = compLines[i][1];
		h_value = *std::min_element(heightList.begin(), heightList.end());
	}
	else {
		//DEBUG
		//cv::namedWindow("rasterImg");
		//cv::imshow("rasterImg", rasterImg);
		//cv::waitKey(0);
		//return false;
	}

	if (h_value > 20 || h_value < 2)
		return false;

	//cout << "zhujunqing show wall id ==== " << wall_id << " baseBoard h_value =  " << h_value << endl;


	if (TOP_BOARD)
	{
		cv::Mat clipedImg = rasterImg.rowRange(0, clippedBaseboardHeight);
		cv::Mat blurImg;
		cv::GaussianBlur(clipedImg, blurImg, cv::Size(5, 1), 5);
		//cv::medianBlur(clipedImg, blurImg, 3);


		// transpose
		cv::transpose(blurImg, blurImg);

		cv::Mat statMat;
		// add where(cols !=0)
		// where(==0) + delete(axis=1)
		for (int i = 0; i < blurImg.rows; i++) {
			bool isColNonZero = true;
			for (int j = 0; j < blurImg.cols; j++) {
				if (blurImg.at<uchar>(i, j) == 0) {
					isColNonZero = false;
					break;
				}
			}
			if (isColNonZero) {
				statMat.push_back(blurImg.row(i));
			}
		}
		cv::transpose(statMat, statMat);

		cv::Mat compStatMat;
		// add where(cols == 0)
		// where(==0) + unique + sort
		for (int i = 0; i < blurImg.rows; i++) {
			bool isColExistZero = false;
			for (int j = 0; j < blurImg.cols; j++) {
				if (blurImg.at<uchar>(i, j) == 0) {
					isColExistZero = true;
					break;
				}
			}
			if (isColExistZero) {
				compStatMat.push_back(blurImg.row(i));
			}
		}
		cv::transpose(compStatMat, compStatMat);

		//transpose
		cv::transpose(blurImg, blurImg);


		SAVE_OBSTACLES_DIR_PREFIX = "D:/DEBUG";
		SAVE_OBSTACLES = _access(SAVE_OBSTACLES_DIR_PREFIX.c_str(), 0) == 0 ? true : false;
		SAVE_BOUNDARY_DIR = "rst_file_topBoard";

		if (SAVE_OBSTACLES && _access(SAVE_BOUNDARY_DIR.c_str(), 0) != 0) {
			_mkdir(SAVE_BOUNDARY_DIR.c_str());
		}
		else if (SAVE_OBSTACLES && !clearObstaclesDir_topBoard) {
			std::string delFile = "del /a /f /q " + SAVE_BOUNDARY_DIR + "/*.jpg";
			replace(delFile.begin() + 11, delFile.end(), '/', '\\');
			//std::cout << delFile << std::endl;
			system(delFile.c_str());
			clearObstaclesDir_topBoard = true;
		}

		//detect
		std::vector<cv::Vec4i> lines;
		if (statMat.cols > 0) {
			cv::Mat cannyImg;
			cv::Canny(statMat, cannyImg, cannyMinThreshold, cannyMaxThreshold);
			cv::HoughLinesP(cannyImg, lines, 1, CV_PI / 180, 25, statMat.rows, 0.4*statMat.rows);
			if (DEBUG_BASE_BOARD)
			{

				cv::Mat houghImg;
				cv::cvtColor(cannyImg, houghImg, cv::COLOR_GRAY2RGB);
				for (size_t i = 0; i < lines.size(); i++)
				{
					cv::line(houghImg, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 0, 255), 3, 8);
				}
				//cv::namedWindow("houghImg");
				//cv::imshow("houghImg", houghImg);

				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_blurImg_.jpg", blurImg);
				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_statMat_.jpg", statMat);
				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_cannyImg_.jpg", cannyImg);
				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_houghImg_.jpg", houghImg);
			}

		}
		std::vector<cv::Vec4i> compLines;
		if (compStatMat.cols > 0) {
			cv::Mat compCannyImg;
			cv::Canny(compStatMat, compCannyImg, 50, 150);
			cv::HoughLinesP(compCannyImg, compLines, 1, CV_PI / 180, 25, compStatMat.rows, 0.4*compStatMat.rows);
			if (DEBUG_BASE_BOARD)
			{
				//// DEBUG
				//cv::namedWindow("compStatMat");
				//cv::imshow("compStatMat", compStatMat);
				//cv::namedWindow("cannyStatMat");
				//cv::imshow("cannyStatMat", compCannyImg);
				cv::Mat compHoughImg;
				cv::cvtColor(compCannyImg, compHoughImg, cv::COLOR_GRAY2RGB);
				for (size_t i = 0; i < compLines.size(); i++)
				{
					cv::line(compHoughImg, cv::Point(compLines[i][0], compLines[i][1]), cv::Point(compLines[i][2], compLines[i][3]), cv::Scalar(0, 0, 255), 3, 8);
				}
				//cv::namedWindow("compHoughImg");
				//cv::imshow("compHoughImg", compHoughImg);

				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_compStatMat_.jpg", compStatMat);
				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_compCannyImg_.jpg", compCannyImg);
				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_compHoughImg_.jpg", compHoughImg);
			}
		}

		int h_value = 0;
		if ((statMat.cols > 0.4 * blurImg.cols) && lines.size() > 0) {
			std::vector<int> heightList(lines.size());
			for (int i = 0; i < lines.size(); i++)
				heightList[i] = lines[i][1];
			h_value = *std::max_element(heightList.begin(), heightList.end());
		}
		else if (compLines.size() > 0) {
			std::vector<int> heightList(compLines.size());
			for (int i = 0; i < compLines.size(); i++)
				heightList[i] = compLines[i][1];
			h_value = *std::max_element(heightList.begin(), heightList.end());
		}
		else {
			//DEBUG
			//cv::namedWindow("rasterImg");
			//cv::imshow("rasterImg", rasterImg);
			//cv::waitKey(0);
			//return false;
		}
		if (h_value > 28 || h_value <10 )
			return false;
		//cout << "zhujunqing show top wall id == " << wall_id << "  h_value == " << h_value << endl;
	}

	if (LEFT_BOARD)
	{
		//cv::Mat clipedImg = rasterImg.rowRange(0, clippedBaseboardHeight);
		int colRange = min(rasterImg.cols, clippedBaseboardHeight);
		cv::Mat clipedImg = rasterImg.colRange(0, colRange);
		cv::Mat blurImg;
		cv::GaussianBlur(clipedImg, blurImg, cv::Size(1, 5), 5);

		// transpose
		//cv::transpose(blurImg, blurImg);

		cv::Mat statMat;
		// add where(cols !=0)
		// where(==0) + delete(axis=1)
		for (int i = 0; i < blurImg.rows; i++) {
			bool isColNonZero = true;
			for (int j = 0; j < blurImg.cols; j++) {
				if (blurImg.at<uchar>(i, j) == 0) {
					isColNonZero = false;
					break;
				}
			}
			if (isColNonZero) {
				statMat.push_back(blurImg.row(i));
			}
		}
		//cv::transpose(statMat, statMat);

		cv::Mat compStatMat;
		// add where(cols == 0)
		// where(==0) + unique + sort
		for (int i = 0; i < blurImg.rows; i++) {
			bool isColExistZero = false;
			for (int j = 0; j < blurImg.cols; j++) {
				if (blurImg.at<uchar>(i, j) == 0) {
					isColExistZero = true;
					break;
				}
			}
			if (isColExistZero) {
				compStatMat.push_back(blurImg.row(i));
			}
		}
		//cv::transpose(compStatMat, compStatMat);

		//transpose
		//cv::transpose(blurImg, blurImg);

		SAVE_OBSTACLES_DIR_PREFIX = "D:/DEBUG";
		SAVE_OBSTACLES = _access(SAVE_OBSTACLES_DIR_PREFIX.c_str(), 0) == 0 ? true : false;
		SAVE_BOUNDARY_DIR = "rst_file_leftBoard";

		if (SAVE_OBSTACLES && _access(SAVE_BOUNDARY_DIR.c_str(), 0) != 0) {
			_mkdir(SAVE_BOUNDARY_DIR.c_str());
		}
		else if (SAVE_OBSTACLES && !clearObstaclesDir_leftBoard) {
			std::string delFile = "del /a /f /q " + SAVE_BOUNDARY_DIR + "/*.jpg";
			replace(delFile.begin() + 11, delFile.end(), '/', '\\');
			//std::cout << delFile << std::endl;
			system(delFile.c_str());
			clearObstaclesDir_leftBoard = true;
		}


		//detect
		std::vector<cv::Vec4i> lines;
		if (statMat.rows > 0) {
			cv::Mat cannyImg;
			cv::Canny(statMat, cannyImg, cannyMinThreshold, cannyMaxThreshold);
			cv::HoughLinesP(cannyImg, lines, 1, CV_PI / 180, 25, statMat.cols, 0.4*statMat.cols);
			if (DEBUG_BASE_BOARD)
			{

				cv::Mat houghImg;
				cv::cvtColor(cannyImg, houghImg, cv::COLOR_GRAY2RGB);
				for (size_t i = 0; i < lines.size(); i++)
				{
					cv::line(houghImg, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 0, 255), 3, 8);
				}
				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_blurImg_.jpg", blurImg);
				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_statMat_.jpg", statMat);
				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_cannyImg_.jpg", cannyImg);
				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_houghImg_.jpg", houghImg);
			}

		}
		std::vector<cv::Vec4i> compLines;
		if (compStatMat.rows > 0) {
			cv::Mat compCannyImg;
			cv::Canny(compStatMat, compCannyImg, 50, 150);
			cv::HoughLinesP(compCannyImg, compLines, 1, CV_PI / 180, 25, statMat.cols, 0.4*statMat.cols);
			if (DEBUG_BASE_BOARD)
			{
				cv::Mat compHoughImg;
				cv::cvtColor(compCannyImg, compHoughImg, cv::COLOR_GRAY2RGB);
				for (size_t i = 0; i < compLines.size(); i++)
				{
					cv::line(compHoughImg, cv::Point(compLines[i][0], compLines[i][1]), cv::Point(compLines[i][2], compLines[i][3]), cv::Scalar(0, 0, 255), 3, 8);
				}
				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_compStatMat_.jpg", compStatMat);
				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_compCannyImg_.jpg", compCannyImg);
				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_compHoughImg_.jpg", compHoughImg);
			}
		}

		int h_value = 0;
		if ((statMat.rows > 0.4 * blurImg.rows) && lines.size() > 0) {
			std::vector<int> heightList(lines.size());
			for (int i = 0; i < lines.size(); i++)
				heightList[i] = lines[i][0];
			h_value = *std::max_element(heightList.begin(), heightList.end());
		}
		else if (compLines.size() > 0) {
			std::vector<int> heightList(compLines.size());
			for (int i = 0; i < compLines.size(); i++)
				heightList[i] = compLines[i][0];
			h_value = *std::max_element(heightList.begin(), heightList.end());
		}
		else {
			//DEBUG
			//cv::namedWindow("rasterImg");
			//cv::imshow("rasterImg", rasterImg);
			//cv::waitKey(0);
			//return false;
		}
		if (h_value > 28 || h_value < 10)
			return false;
		//cout << "zhujunqing show left wall id == " << wall_id << "  h_value == " << h_value << endl;
	}

	if (RIGHT_BOARD)
	{
		//cv::Mat clipedImg = rasterImg.rowRange(0, clippedBaseboardHeight);
		cv::Mat clipedImg = rasterImg.colRange(rasterImg.cols - clippedBaseboardHeight < 0 ? 0 : rasterImg.cols - clippedBaseboardHeight, rasterImg.cols);
		cv::Mat blurImg;
		cv::GaussianBlur(clipedImg, blurImg, cv::Size(1, 5), 5);

		// transpose
		//cv::transpose(blurImg, blurImg);

		cv::Mat statMat;
		// add where(cols !=0)
		// where(==0) + delete(axis=1)
		for (int i = 0; i < blurImg.rows; i++) {
			bool isColNonZero = true;
			for (int j = 0; j < blurImg.cols; j++) {
				if (blurImg.at<uchar>(i, j) == 0) {
					isColNonZero = false;
					break;
				}
			}
			if (isColNonZero) {
				statMat.push_back(blurImg.row(i));
			}
		}
		//cv::transpose(statMat, statMat);

		cv::Mat compStatMat;
		// add where(cols == 0)
		// where(==0) + unique + sort
		for (int i = 0; i < blurImg.rows; i++) {
			bool isColExistZero = false;
			for (int j = 0; j < blurImg.cols; j++) {
				if (blurImg.at<uchar>(i, j) == 0) {
					isColExistZero = true;
					break;
				}
			}
			if (isColExistZero) {
				compStatMat.push_back(blurImg.row(i));
			}
		}
		//cv::transpose(compStatMat, compStatMat);

		//transpose
		//cv::transpose(blurImg, blurImg);


		SAVE_OBSTACLES_DIR_PREFIX = "D:/DEBUG";
		SAVE_OBSTACLES = _access(SAVE_OBSTACLES_DIR_PREFIX.c_str(), 0) == 0 ? true : false;
		SAVE_BOUNDARY_DIR = "rst_file_rightBoard";

		if (SAVE_OBSTACLES && _access(SAVE_BOUNDARY_DIR.c_str(), 0) != 0) {
			_mkdir(SAVE_BOUNDARY_DIR.c_str());
		}
		else if (SAVE_OBSTACLES && !clearObstaclesDir_rightBoard) {
			std::string delFile = "del /a /f /q " + SAVE_BOUNDARY_DIR + "/*.jpg";
			replace(delFile.begin() + 11, delFile.end(), '/', '\\');
			//std::cout << delFile << std::endl;
			system(delFile.c_str());
			clearObstaclesDir_rightBoard = true;
		}


		//detect
		std::vector<cv::Vec4i> lines;
		if (statMat.rows > 0) {
			cv::Mat cannyImg;
			cv::Canny(statMat, cannyImg, cannyMinThreshold, cannyMaxThreshold);
			cv::HoughLinesP(cannyImg, lines, 1, CV_PI / 180, 25, 0.2*statMat.rows, 0.05*statMat.rows);
			if (DEBUG_BASE_BOARD)
			{
				cv::Mat houghImg;
				cv::cvtColor(cannyImg, houghImg, cv::COLOR_GRAY2RGB);
				for (size_t i = 0; i < lines.size(); i++)
				{
					cv::line(houghImg, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 0, 255), 3, 8);
				}

				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_blurImg_.jpg", blurImg);
				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_statMat_.jpg", statMat);
				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_cannyImg_.jpg", cannyImg);
				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_houghImg_.jpg", houghImg);
			}

		}
		std::vector<cv::Vec4i> compLines;
		if (compStatMat.rows > 0) {
			cv::Mat compCannyImg;
			cv::Canny(compStatMat, compCannyImg, 50, 150);
			cv::HoughLinesP(compCannyImg, compLines, 1, CV_PI / 180, 25, statMat.cols, 0.4*statMat.cols);
			if (DEBUG_BASE_BOARD)
			{

				cv::Mat compHoughImg;
				cv::cvtColor(compCannyImg, compHoughImg, cv::COLOR_GRAY2RGB);
				for (size_t i = 0; i < compLines.size(); i++)
				{
					cv::line(compHoughImg, cv::Point(compLines[i][0], compLines[i][1]), cv::Point(compLines[i][2], compLines[i][3]), cv::Scalar(0, 0, 255), 3, 8);
				}

				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_compStatMat_.jpg", compStatMat);
				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_compCannyImg_.jpg", compCannyImg);
				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_compHoughImg_.jpg", compHoughImg);
			}
		}

		int h_value = 0;
		if ((statMat.rows > 0.4 * blurImg.rows) && lines.size() > 0) {
			std::vector<int> heightList(lines.size());
			for (int i = 0; i < lines.size(); i++)
				heightList[i] = lines[i][0];
			h_value = *std::min_element(heightList.begin(), heightList.end());
		}
		else if (compLines.size() > 0) {
			std::vector<int> heightList(compLines.size());
			for (int i = 0; i < compLines.size(); i++)
				heightList[i] = compLines[i][0];
			h_value = *std::min_element(heightList.begin(), heightList.end());
		}
		else {
			//DEBUG
			//cv::namedWindow("rasterImg");
			//cv::imshow("rasterImg", rasterImg);
			//cv::waitKey(0);
			//return false;
		}
		if (h_value > 20 || h_value < 2)
			return false;
		//cout << "zhujunqing show right wall id == " << wall_id << "  h_value == " << h_value << endl;
	}
	return true;
}

bool DetectObstacle::DetectBaseBoard(const std::vector<cv::Point3f>& cloudPoints, float& baseBoardHeight) {

	string SAVE_BOUNDARY_DIR;

	// clear history value
	rasterBaseBoardHeight = 0;
	cv::Mat clipedImg = rasterImg.rowRange(rasterImg.rows - clippedBaseboardHeight, rasterImg.rows);
	cv::Mat blurImg;
	cv::GaussianBlur(clipedImg, blurImg, cv::Size(5, 1), 5);
	//cv::medianBlur(clipedImg, blurImg, 3);


	// transpose
	cv::transpose(blurImg, blurImg);

	cv::Mat statMat;
	// add where(cols !=0)
	// where(==0) + delete(axis=1)
	for (int i = 0; i < blurImg.rows; i++) {
		bool isColNonZero = true;
		for (int j = 0; j < blurImg.cols; j++) {
			if (blurImg.at<uchar>(i, j) == 0) {
				isColNonZero = false;
				break;
			}
		}
		if (isColNonZero) {
			statMat.push_back(blurImg.row(i));
		}
	}
	cv::transpose(statMat, statMat);

	cv::Mat compStatMat;
	// add where(cols == 0)
	// where(==0) + unique + sort
	for (int i = 0; i < blurImg.rows; i++) {
		bool isColExistZero = false;
		for (int j = 0; j < blurImg.cols; j++) {
			if (blurImg.at<uchar>(i, j) == 0) {
				isColExistZero = true;
				break;
			}
		}
		if (isColExistZero) {
			compStatMat.push_back(blurImg.row(i));
		}
	}
	cv::transpose(compStatMat, compStatMat);

	//transpose
	cv::transpose(blurImg, blurImg);

	SAVE_OBSTACLES_DIR_PREFIX = "D:/DEBUG";
	SAVE_OBSTACLES = _access(SAVE_OBSTACLES_DIR_PREFIX.c_str(), 0) == 0 ? true : false;
	SAVE_BOUNDARY_DIR = "rst_file_baseBoard";

	if (SAVE_OBSTACLES && _access(SAVE_BOUNDARY_DIR.c_str(), 0) != 0) {
		_mkdir(SAVE_BOUNDARY_DIR.c_str());
	}
	else if (SAVE_OBSTACLES && !clearObstaclesDir_baseBoard) {
		std::string delFile = "del /a /f /q " + SAVE_BOUNDARY_DIR + "/*.jpg";
		replace(delFile.begin() + 11, delFile.end(), '/', '\\');
		//std::cout << delFile << std::endl;
		system(delFile.c_str());
		clearObstaclesDir_baseBoard = true;
	}

	//detect
	std::vector<cv::Vec4i> lines;
	if (statMat.cols > 0) {
		cv::Mat cannyImg;
		cv::Canny(statMat, cannyImg, cannyMinThreshold, cannyMaxThreshold);
		cv::HoughLinesP(cannyImg, lines, 1, CV_PI / 180, 25, statMat.rows, 0.4*statMat.rows);
		if (DEBUG_BASE_BOARD)
		{

			cv::Mat houghImg;
			cv::cvtColor(cannyImg, houghImg, cv::COLOR_GRAY2RGB);
			for (size_t i = 0; i < lines.size(); i++)
			{
				cv::line(houghImg, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 0, 255), 3, 8);
			}


			cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_blurImg_.jpg", blurImg);
			cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_statMat_.jpg", statMat);
			cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_cannyImg_.jpg", cannyImg);
			cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_houghImg_.jpg", houghImg);
		}

	}
	std::vector<cv::Vec4i> compLines;
	if (compStatMat.cols > 0) {
		cv::Mat compCannyImg;
		cv::Canny(compStatMat, compCannyImg, 50, 150);
		cv::HoughLinesP(compCannyImg, compLines, 1, CV_PI / 180, 25, compStatMat.rows, 0.4*compStatMat.rows);
		if (DEBUG_BASE_BOARD)
		{
			//// DEBUG
			//cv::namedWindow("compStatMat");
			//cv::imshow("compStatMat", compStatMat);
			//cv::namedWindow("cannyStatMat");
			//cv::imshow("cannyStatMat", compCannyImg);
			cv::Mat compHoughImg;
			cv::cvtColor(compCannyImg, compHoughImg, cv::COLOR_GRAY2RGB);
			for (size_t i = 0; i < compLines.size(); i++)
			{
				cv::line(compHoughImg, cv::Point(compLines[i][0], compLines[i][1]), cv::Point(compLines[i][2], compLines[i][3]), cv::Scalar(0, 0, 255), 3, 8);
			}
			//cv::namedWindow("compHoughImg");
			//cv::imshow("compHoughImg", compHoughImg);




			cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_compStatMat_.jpg", compStatMat);
			cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_compCannyImg_.jpg", compCannyImg);
			cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_compHoughImg_.jpg", compHoughImg);
		}
	}

	int h_value = 0;
	if ((statMat.cols > 0.4 * blurImg.cols) && lines.size() > 0) {
		std::vector<int> heightList(lines.size());
		for (int i = 0; i < lines.size(); i++)
			heightList[i] = lines[i][1];
		h_value = *std::min_element(heightList.begin(), heightList.end());
	}
	else if (compLines.size() > 0) {
		std::vector<int> heightList(compLines.size());
		for (int i = 0; i < compLines.size(); i++)
			heightList[i] = compLines[i][1];
		h_value = *std::min_element(heightList.begin(), heightList.end());
	}
	else {
		//DEBUG
		//cv::namedWindow("rasterImg");
		//cv::imshow("rasterImg", rasterImg);
		//cv::waitKey(0);
		//return false;
	}

	cout << "zhujunqing show wall id ==== " << wall_id << " baseBoard h_value =  " << h_value << endl;

	h_value = clippedBaseboardHeight - h_value;

	//filter
	if (h_value <= 1) return false;
	if (h_value > 35) return false;//old 8;29,changed by hgj ,35

	// save result (height)
	baseBoardHeight = h_value + baseBoardExpandingValue;  // return value
	rasterBaseBoardHeight = h_value + baseBoardExpandingValue; // member value


	if (TOP_BOARD)
	{
		cv::Mat clipedImg = rasterImg.rowRange(0, clippedBaseboardHeight);
		cv::Mat blurImg;
		cv::GaussianBlur(clipedImg, blurImg, cv::Size(5, 1), 5);
		//cv::medianBlur(clipedImg, blurImg, 3);


		// transpose
		cv::transpose(blurImg, blurImg);

		cv::Mat statMat;
		// add where(cols !=0)
		// where(==0) + delete(axis=1)
		for (int i = 0; i < blurImg.rows; i++) {
			bool isColNonZero = true;
			for (int j = 0; j < blurImg.cols; j++) {
				if (blurImg.at<uchar>(i, j) == 0) {
					isColNonZero = false;
					break;
				}
			}
			if (isColNonZero) {
				statMat.push_back(blurImg.row(i));
			}
		}
		cv::transpose(statMat, statMat);

		cv::Mat compStatMat;
		// add where(cols == 0)
		// where(==0) + unique + sort
		for (int i = 0; i < blurImg.rows; i++) {
			bool isColExistZero = false;
			for (int j = 0; j < blurImg.cols; j++) {
				if (blurImg.at<uchar>(i, j) == 0) {
					isColExistZero = true;
					break;
				}
			}
			if (isColExistZero) {
				compStatMat.push_back(blurImg.row(i));
			}
		}
		cv::transpose(compStatMat, compStatMat);

		//transpose
		cv::transpose(blurImg, blurImg);


		SAVE_OBSTACLES_DIR_PREFIX = "D:/DEBUG";
		SAVE_OBSTACLES = _access(SAVE_OBSTACLES_DIR_PREFIX.c_str(), 0) == 0 ? true : false;
		SAVE_BOUNDARY_DIR = "rst_file_topBoard";

		if (SAVE_OBSTACLES && _access(SAVE_BOUNDARY_DIR.c_str(), 0) != 0) {
			_mkdir(SAVE_BOUNDARY_DIR.c_str());
		}
		else if (SAVE_OBSTACLES && !clearObstaclesDir_topBoard) {
			std::string delFile = "del /a /f /q " + SAVE_BOUNDARY_DIR + "/*.jpg";
			replace(delFile.begin() + 11, delFile.end(), '/', '\\');
			//std::cout << delFile << std::endl;
			system(delFile.c_str());
			clearObstaclesDir_topBoard = true;
		}

		//detect
		std::vector<cv::Vec4i> lines;
		if (statMat.cols > 0) {
			cv::Mat cannyImg;
			cv::Canny(statMat, cannyImg, cannyMinThreshold, cannyMaxThreshold);
			cv::HoughLinesP(cannyImg, lines, 1, CV_PI / 180, 25, statMat.rows, 0.4*statMat.rows);
			if (DEBUG_BASE_BOARD)
			{

				cv::Mat houghImg;
				cv::cvtColor(cannyImg, houghImg, cv::COLOR_GRAY2RGB);
				for (size_t i = 0; i < lines.size(); i++)
				{
					cv::line(houghImg, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 0, 255), 3, 8);
				}
				//cv::namedWindow("houghImg");
				//cv::imshow("houghImg", houghImg);

				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_blurImg_.jpg", blurImg);
				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_statMat_.jpg", statMat);
				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_cannyImg_.jpg", cannyImg);
				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_houghImg_.jpg", houghImg);
			}

		}
		std::vector<cv::Vec4i> compLines;
		if (compStatMat.cols > 0) {
			cv::Mat compCannyImg;
			cv::Canny(compStatMat, compCannyImg, 50, 150);
			cv::HoughLinesP(compCannyImg, compLines, 1, CV_PI / 180, 25, compStatMat.rows, 0.4*compStatMat.rows);
			if (DEBUG_BASE_BOARD)
			{
				//// DEBUG
				//cv::namedWindow("compStatMat");
				//cv::imshow("compStatMat", compStatMat);
				//cv::namedWindow("cannyStatMat");
				//cv::imshow("cannyStatMat", compCannyImg);
				cv::Mat compHoughImg;
				cv::cvtColor(compCannyImg, compHoughImg, cv::COLOR_GRAY2RGB);
				for (size_t i = 0; i < compLines.size(); i++)
				{
					cv::line(compHoughImg, cv::Point(compLines[i][0], compLines[i][1]), cv::Point(compLines[i][2], compLines[i][3]), cv::Scalar(0, 0, 255), 3, 8);
				}
				//cv::namedWindow("compHoughImg");
				//cv::imshow("compHoughImg", compHoughImg);

				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_compStatMat_.jpg", compStatMat);
				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_compCannyImg_.jpg", compCannyImg);
				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_compHoughImg_.jpg", compHoughImg);
			}
		}

		int h_value = 0;
		if ((statMat.cols > 0.4 * blurImg.cols) && lines.size() > 0) {
			std::vector<int> heightList(lines.size());
			for (int i = 0; i < lines.size(); i++)
				heightList[i] = lines[i][1];
			h_value = *std::max_element(heightList.begin(), heightList.end());
		}
		else if (compLines.size() > 0) {
			std::vector<int> heightList(compLines.size());
			for (int i = 0; i < compLines.size(); i++)
				heightList[i] = compLines[i][1];
			h_value = *std::max_element(heightList.begin(), heightList.end());
		}
		else {
			//DEBUG
			//cv::namedWindow("rasterImg");
			//cv::imshow("rasterImg", rasterImg);
			//cv::waitKey(0);
			//return false;
		}

		cout << "zhujunqing show top wall id == " << wall_id << "  h_value == " << h_value << endl;
	}

	if (LEFT_BOARD)
	{
		//cv::Mat clipedImg = rasterImg.rowRange(0, clippedBaseboardHeight);
		int colRange = min(rasterImg.cols, clippedBaseboardHeight);
		cv::Mat clipedImg = rasterImg.colRange(0, colRange);
		cv::Mat blurImg;
		cv::GaussianBlur(clipedImg, blurImg, cv::Size(1, 5), 5);

		// transpose
		//cv::transpose(blurImg, blurImg);

		cv::Mat statMat;
		// add where(cols !=0)
		// where(==0) + delete(axis=1)
		for (int i = 0; i < blurImg.rows; i++) {
			bool isColNonZero = true;
			for (int j = 0; j < blurImg.cols; j++) {
				if (blurImg.at<uchar>(i, j) == 0) {
					isColNonZero = false;
					break;
				}
			}
			if (isColNonZero) {
				statMat.push_back(blurImg.row(i));
			}
		}
		//cv::transpose(statMat, statMat);

		cv::Mat compStatMat;
		// add where(cols == 0)
		// where(==0) + unique + sort
		for (int i = 0; i < blurImg.rows; i++) {
			bool isColExistZero = false;
			for (int j = 0; j < blurImg.cols; j++) {
				if (blurImg.at<uchar>(i, j) == 0) {
					isColExistZero = true;
					break;
				}
			}
			if (isColExistZero) {
				compStatMat.push_back(blurImg.row(i));
			}
		}
		//cv::transpose(compStatMat, compStatMat);

		//transpose
		//cv::transpose(blurImg, blurImg);

		SAVE_OBSTACLES_DIR_PREFIX = "D:/DEBUG";
		SAVE_OBSTACLES = _access(SAVE_OBSTACLES_DIR_PREFIX.c_str(), 0) == 0 ? true : false;
		SAVE_BOUNDARY_DIR = "rst_file_leftBoard";

		if (SAVE_OBSTACLES && _access(SAVE_BOUNDARY_DIR.c_str(), 0) != 0) {
			_mkdir(SAVE_BOUNDARY_DIR.c_str());
		}
		else if (SAVE_OBSTACLES && !clearObstaclesDir_leftBoard) {
			std::string delFile = "del /a /f /q " + SAVE_BOUNDARY_DIR + "/*.jpg";
			replace(delFile.begin() + 11, delFile.end(), '/', '\\');
			//std::cout << delFile << std::endl;
			system(delFile.c_str());
			clearObstaclesDir_leftBoard = true;
		}


		//detect
		std::vector<cv::Vec4i> lines;
		if (statMat.rows > 0) {
			cv::Mat cannyImg;
			cv::Canny(statMat, cannyImg, cannyMinThreshold, cannyMaxThreshold);
			cv::HoughLinesP(cannyImg, lines, 1, CV_PI / 180, 25, statMat.cols, 0.4*statMat.cols);
			if (DEBUG_BASE_BOARD)
			{

				cv::Mat houghImg;
				cv::cvtColor(cannyImg, houghImg, cv::COLOR_GRAY2RGB);
				for (size_t i = 0; i < lines.size(); i++)
				{
					cv::line(houghImg, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 0, 255), 3, 8);
				}
				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_blurImg_.jpg", blurImg);
				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_statMat_.jpg", statMat);
				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_cannyImg_.jpg", cannyImg);
				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_houghImg_.jpg", houghImg);
			}

		}
		std::vector<cv::Vec4i> compLines;
		if (compStatMat.rows > 0) {
			cv::Mat compCannyImg;
			cv::Canny(compStatMat, compCannyImg, 50, 150);
			cv::HoughLinesP(compCannyImg, compLines, 1, CV_PI / 180, 25, statMat.cols, 0.4*statMat.cols);
			if (DEBUG_BASE_BOARD)
			{
				cv::Mat compHoughImg;
				cv::cvtColor(compCannyImg, compHoughImg, cv::COLOR_GRAY2RGB);
				for (size_t i = 0; i < compLines.size(); i++)
				{
					cv::line(compHoughImg, cv::Point(compLines[i][0], compLines[i][1]), cv::Point(compLines[i][2], compLines[i][3]), cv::Scalar(0, 0, 255), 3, 8);
				}
				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_compStatMat_.jpg", compStatMat);
				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_compCannyImg_.jpg", compCannyImg);
				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_compHoughImg_.jpg", compHoughImg);
			}
		}

		int h_value = 0;
		if ((statMat.rows > 0.4 * blurImg.rows) && lines.size() > 0) {
			std::vector<int> heightList(lines.size());
			for (int i = 0; i < lines.size(); i++)
				heightList[i] = lines[i][0];
			h_value = *std::max_element(heightList.begin(), heightList.end());
		}
		else if (compLines.size() > 0) {
			std::vector<int> heightList(compLines.size());
			for (int i = 0; i < compLines.size(); i++)
				heightList[i] = compLines[i][0];
			h_value = *std::max_element(heightList.begin(), heightList.end());
		}
		else {
			//DEBUG
			//cv::namedWindow("rasterImg");
			//cv::imshow("rasterImg", rasterImg);
			//cv::waitKey(0);
			//return false;
		}

		cout << "zhujunqing show left wall id == " << wall_id << "  h_value == " << h_value << endl;
	}

	if (RIGHT_BOARD)
	{
		//cv::Mat clipedImg = rasterImg.rowRange(0, clippedBaseboardHeight);
		cv::Mat clipedImg = rasterImg.colRange(rasterImg.cols - clippedBaseboardHeight<0 ? 0: rasterImg.cols - clippedBaseboardHeight, rasterImg.cols);
		cv::Mat blurImg;
		cv::GaussianBlur(clipedImg, blurImg, cv::Size(1, 5), 5);

		// transpose
		//cv::transpose(blurImg, blurImg);

		cv::Mat statMat;
		// add where(cols !=0)
		// where(==0) + delete(axis=1)
		for (int i = 0; i < blurImg.rows; i++) {
			bool isColNonZero = true;
			for (int j = 0; j < blurImg.cols; j++) {
				if (blurImg.at<uchar>(i, j) == 0) {
					isColNonZero = false;
					break;
				}
			}
			if (isColNonZero) {
				statMat.push_back(blurImg.row(i));
			}
		}
		//cv::transpose(statMat, statMat);

		cv::Mat compStatMat;
		// add where(cols == 0)
		// where(==0) + unique + sort
		for (int i = 0; i < blurImg.rows; i++) {
			bool isColExistZero = false;
			for (int j = 0; j < blurImg.cols; j++) {
				if (blurImg.at<uchar>(i, j) == 0) {
					isColExistZero = true;
					break;
				}
			}
			if (isColExistZero) {
				compStatMat.push_back(blurImg.row(i));
			}
		}
		//cv::transpose(compStatMat, compStatMat);

		//transpose
		//cv::transpose(blurImg, blurImg);


		SAVE_OBSTACLES_DIR_PREFIX = "D:/DEBUG";
		SAVE_OBSTACLES = _access(SAVE_OBSTACLES_DIR_PREFIX.c_str(), 0) == 0 ? true : false;
		SAVE_BOUNDARY_DIR = "rst_file_rightBoard";

		if (SAVE_OBSTACLES && _access(SAVE_BOUNDARY_DIR.c_str(), 0) != 0) {
			_mkdir(SAVE_BOUNDARY_DIR.c_str());
		}
		else if (SAVE_OBSTACLES && !clearObstaclesDir_rightBoard) {
			std::string delFile = "del /a /f /q " + SAVE_BOUNDARY_DIR + "/*.jpg";
			replace(delFile.begin() + 11, delFile.end(), '/', '\\');
			//std::cout << delFile << std::endl;
			system(delFile.c_str());
			clearObstaclesDir_rightBoard = true;
		}


		//detect
		std::vector<cv::Vec4i> lines;
		if (statMat.rows > 0) {
			cv::Mat cannyImg;
			cv::Canny(statMat, cannyImg, cannyMinThreshold, cannyMaxThreshold);
			cv::HoughLinesP(cannyImg, lines, 1, CV_PI / 180, 25, 0.2*statMat.rows, 0.05*statMat.rows);
			if (DEBUG_BASE_BOARD)
			{
				cv::Mat houghImg;
				cv::cvtColor(cannyImg, houghImg, cv::COLOR_GRAY2RGB);
				for (size_t i = 0; i < lines.size(); i++)
				{
					cv::line(houghImg, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 0, 255), 3, 8);
				}

				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_blurImg_.jpg", blurImg);
				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_statMat_.jpg", statMat);
				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_cannyImg_.jpg", cannyImg);
				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_houghImg_.jpg", houghImg);
			}

		}
		std::vector<cv::Vec4i> compLines;
		if (compStatMat.rows > 0) {
			cv::Mat compCannyImg;
			cv::Canny(compStatMat, compCannyImg, 50, 150);
			cv::HoughLinesP(compCannyImg, compLines, 1, CV_PI / 180, 25, statMat.cols, 0.4*statMat.cols);
			if (DEBUG_BASE_BOARD)
			{

				cv::Mat compHoughImg;
				cv::cvtColor(compCannyImg, compHoughImg, cv::COLOR_GRAY2RGB);
				for (size_t i = 0; i < compLines.size(); i++)
				{
					cv::line(compHoughImg, cv::Point(compLines[i][0], compLines[i][1]), cv::Point(compLines[i][2], compLines[i][3]), cv::Scalar(0, 0, 255), 3, 8);
				}

				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_compStatMat_.jpg", compStatMat);
				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_compCannyImg_.jpg", compCannyImg);
				cv::imwrite(SAVE_BOUNDARY_DIR + "/" + std::to_string(wall_id) + "_compHoughImg_.jpg", compHoughImg);
			}
		}

		int h_value = 0;
		if ((statMat.rows > 0.4 * blurImg.rows) && lines.size() > 0) {
			std::vector<int> heightList(lines.size());
			for (int i = 0; i < lines.size(); i++)
				heightList[i] = lines[i][0];
			h_value = *std::min_element(heightList.begin(), heightList.end());
		}
		else if (compLines.size() > 0) {
			std::vector<int> heightList(compLines.size());
			for (int i = 0; i < compLines.size(); i++)
				heightList[i] = compLines[i][0];
			h_value = *std::min_element(heightList.begin(), heightList.end());
		}
		else {
			//DEBUG
			//cv::namedWindow("rasterImg");
			//cv::imshow("rasterImg", rasterImg);
			//cv::waitKey(0);
			//return false;
		}

	//	cout << "zhujunqing show right wall id == " << wall_id << "  h_value == " << h_value << endl;
	}
	return true;
}
bool DetectObstacle::DetectDoorFrame(const std::vector<cv::Point3f>& cloudPoints, const std::vector<std::vector<std::vector<cv::Point3f>>>& wall_door_window_pos, int wallId, WallObstacleInfo& obstacleInfo) {

	// clear history data
	rasterDoorFrameArray.resize(0, 4);
	//std::cout << "rasterDoorFrameArray.rows()" << rasterDoorFrameArray.rows() << std::endl;
	//std::cout << "rasterDoorFrameArray.cols()" << rasterDoorFrameArray.cols() << std::endl;

	// if (!(wall_id == 2)) return false;
	//std::cout << "detect doorframe -- wall_id:" << wall_id << std::endl;
	/*std::cout << "rasterImg(0,0)" << rasterImg.at<uchar>(0, 0) << std::endl;*/
	// remove beam height
	cv::Mat clipedRasterImg = rasterImg.clone();
	int clipedRasterHeight = rasterHeight;
	int beamHeight = 0;
	if (obstacleInfo.beamInfo.beamMap.count(wall_id) > 0) {
		int bias = 3;
		beamHeight = int(std::get<0>(obstacleInfo.beamInfo.beamMap[wall_id]) / pixelHeight) + bias;
		//modify by zhujunqing 20210508 begin
		if (rasterImg.rows > beamHeight) {
			clipedRasterImg = rasterImg.rowRange(beamHeight, rasterImg.rows);
			clipedRasterHeight -= beamHeight;
		}
		//modify by zhujunqing 20210508 end
	}
	// load hole
	// holePointsArray.row: x,z  holePointsArray.cols: cornerPoint1,cornerPoint2
	Eigen::Array2Xf holePointsArray;
	// WRONG: loop m_info_holes (size is different)
	/*for (int hole_id = 0; hole_id < obstacleInfo.doorFrameInfo.wall_door_window_pos.size(); hole_id++) {
		if (wall_id == scene_filter_output.wall_vertices[hole_id].first) {*/
	if (wall_door_window_pos.size() > wallId) {

		for (int hole_id = 0; hole_id < wall_door_window_pos[wallId].size(); hole_id++) {
			cv::Point3f cornerPoint1 = wall_door_window_pos[wallId][hole_id][0];
			cv::Point3f cornerPoint2 = wall_door_window_pos[wallId][hole_id][2];
			cv::Point3f cornerPoint1_coarse;
			cv::Point3f cornerPoint1_fine;
			cv::Point3f cornerPoint2_coarse;
			cv::Point3f cornerPoint2_fine;

			// rotate to 2d
			MeasureBase::RotatePoint(cornerPoint1, coarseRotationMat, cornerPoint1_coarse);
			MeasureBase::RotatePoint(cornerPoint1_coarse, fineRotationMat, cornerPoint1_fine);
			MeasureBase::RotatePoint(cornerPoint2, coarseRotationMat, cornerPoint2_coarse);
			MeasureBase::RotatePoint(cornerPoint2_coarse, fineRotationMat, cornerPoint2_fine);

			Eigen::Array2f cornerPoint1_eigen(cornerPoint1_fine.x, cornerPoint1_fine.z);
			Eigen::Array2f cornerPoint2_eigen(cornerPoint2_fine.x, cornerPoint2_fine.z);


			/*std::cout << "*****cornerPoint1_eigen" << std::endl << cornerPoint1_eigen << std::endl;
			std::cout << "*****cornerPoint2_eigen" << std::endl << cornerPoint2_eigen << std::endl;*/
			holePointsArray.conservativeResize(holePointsArray.rows(), holePointsArray.cols() + 2);
			holePointsArray.col(holePointsArray.cols() - 2) = cornerPoint1_eigen;
			holePointsArray.col(holePointsArray.cols() - 1) = cornerPoint2_eigen;
		}
	}
	if (holePointsArray.cols() < 2) return false;
	int strde = holePointsArray.rows();
	Eigen::Map<Eigen::Array2Xf, 0, Eigen::OuterStride<2 * 2>> oddHolePointsArray(holePointsArray.data(), strde, holePointsArray.cols() / 2);
	Eigen::Map<Eigen::Array2Xf, 0, Eigen::OuterStride<2 * 2>> evenHolePointsArray(holePointsArray.data() + 1, strde, holePointsArray.cols() / 2);
	/*std::cout << "*****holePointsArray" << std::endl << holePointsArray << std::endl;
	std::cout << "*****oddHolePointsArray" << std::endl << oddHolePointsArray << std::endl;
	std::cout << "*****evenHolePointsArray" << std::endl << evenHolePointsArray << std::endl;
	std::cout << "*****minCloudPoints_X" << std::endl << minCloudPoints_X << std::endl;
	std::cout << "*****minCloudPoints_Z" << std::endl << minCloudPoints_Z << std::endl;*/

	holePointsArray.row(0) -= minCloudPoints_X;
	holePointsArray.row(1) -= minCloudPoints_Z;
	holePointsArray.row(0) /= pixelWidth;
	holePointsArray.row(1) /= pixelHeight;

	//std::cout << "*****holePointsArray after calc" << std::endl << holePointsArray << std::endl;
	//std::cout << "*****oddHolePointsArray after calc" << std::endl << oddHolePointsArray << std::endl;

	//std::cout << "*****holePointsArray before flip" << std::endl << holePointsArray << std::endl;
	holePointsArray.row(1) = clipedRasterHeight - holePointsArray.row(1) - 1;
	//std::cout << "*****holePointsArray after flip" << std::endl << holePointsArray << std::endl;

	Eigen::Array2Xi imgHolePointsArray(holePointsArray.cast<int>());
	//std::cout << "*****imgHolePointsArray after cast" << std::endl << imgHolePointsArray << std::endl;

	//detect 
	cv::Mat holeImg;
	if (DEBUG_DOOR_FRAME || SAVE_OBSTACLES) {
		cv::cvtColor(rasterImg, holeImg, cv::COLOR_GRAY2RGB);
	}

	cv::Mat beforeCorrectImg = holeImg.clone(); // for debug

	Eigen::ArrayX4i detectBoxArray;

	for (int id_imgHole = 0; id_imgHole < imgHolePointsArray.cols() / 2; id_imgHole++) {

		int holeMinX = imgHolePointsArray.block(0, 2 * id_imgHole, 1, 2).minCoeff();
		int holeMaxX = imgHolePointsArray.block(0, 2 * id_imgHole, 1, 2).maxCoeff();
		int holeMinY = imgHolePointsArray.block(1, 2 * id_imgHole, 1, 2).minCoeff();
		int holeMaxY = imgHolePointsArray.block(1, 2 * id_imgHole, 1, 2).maxCoeff();
		//std::cout << "*****imgHolePointsArray" << std::endl << imgHolePointsArray << std::endl;

		//filter
		if (holeMinX >= rasterWidth || holeMinY >= clipedRasterHeight || holeMaxX < 0 || holeMaxY < 0)
			continue;

		int holeMinX_valid = max(0, holeMinX);
		int holeMinY_valid = max(0, holeMinY);
		int holeMaxX_valid = min(rasterWidth - 1, holeMaxX);
		int holeMaxY_valid = min(clipedRasterHeight - 1, holeMaxY);

		if (DEBUG_DOOR_FRAME || SAVE_OBSTACLES) {
			cv::rectangle(holeImg, cv::Point(holeMinX_valid, holeMinY_valid + beamHeight), cv::Point(holeMaxX_valid, holeMaxY_valid + beamHeight), cv::Scalar(0, 0, 255), 1, 16);
		}

		// **************** start expand ********************
		int holeMinX_updated = holeMinX_valid;
		int holeMinY_updated = holeMinY_valid;
		int holeMaxX_updated = holeMaxX_valid;
		int holeMaxY_updated = holeMaxY_valid;
		// expand axis -x 
		int idx = holeMinX_updated;
		cv::Mat colMat;
		while (idx > 0) {
			colMat = clipedRasterImg(cv::Range(holeMinY_updated, holeMaxY_updated + 1), cv::Range(idx, idx + 1));
			int cntZero = 0;
			for (int i = 0; i < colMat.rows; i++) {
				if (colMat.at<uchar>(i, 0) < minExpandPixelValueThreshold)		cntZero += 1;
			}
			if (cntZero > expandRatio * colMat.rows) {
				holeMinX_updated = idx;
				idx -= 1;
			}
			else { break; }
		}
		// expand axis +x 
		idx = holeMaxX_updated;
		while (idx < rasterWidth) {
			colMat = clipedRasterImg(cv::Range(holeMinY_updated, holeMaxY_updated + 1), cv::Range(idx, idx + 1));
			int cntZero = 0;
			for (int i = 0; i < colMat.rows; i++) {
				if (colMat.at<uchar>(i, 0) < minExpandPixelValueThreshold)		cntZero += 1;
			}
			if (cntZero > expandRatio * colMat.rows) {
				holeMaxX_updated = idx;
				idx += 1;
			}
			else { break; }
		}
		// expand axis -y
		int idy = holeMinY_updated;
		cv::Mat rowMat;
		while (idy > 0) {
			rowMat = clipedRasterImg(cv::Range(idy, idy + 1), cv::Range(holeMinX_updated, holeMaxX_updated + 1));
			int cntZero = 0;
			for (int j = 0; j < rowMat.cols; j++) {
				if (rowMat.at<uchar>(0, j) < minExpandPixelValueThreshold)		cntZero += 1;
			}
			if (cntZero > expandRatio * rowMat.cols) {
				holeMinY_updated = idy;
				idy -= 1;
			}
			else { break; }
		}
		// expand axis +y
		idy = holeMaxY_updated;
		while (idy < clipedRasterHeight) {
			rowMat = clipedRasterImg(cv::Range(idy, idy + 1), cv::Range(holeMinX_updated, holeMaxX_updated + 1));
			int cntZero = 0;
			for (int j = 0; j < rowMat.cols; j++) {
				if (rowMat.at<uchar>(0, j) < minExpandPixelValueThreshold)		cntZero += 1;
			}
			if (cntZero > expandRatio * rowMat.cols) {
				holeMaxY_updated = idy;
				idy += 1;
			}
			else { break; }
		}
		if (DEBUG_DOOR_FRAME || SAVE_OBSTACLES) {
			cv::rectangle(holeImg, cv::Point(holeMinX_updated, holeMinY_updated + beamHeight), cv::Point(holeMaxX_updated, holeMaxY_updated + beamHeight), cv::Scalar(0, 0, 255), 1, 16);
		}
		// ********** end expand *************

		cv::Mat preBlurImg;
		cv::medianBlur(clipedRasterImg, preBlurImg, 3);

		// ********** start doorframe detection **********
		int roi_min_x, roi_min_y, roi_max_x, roi_max_y, detectMinX, detectMaxX, detectMinY, detectMaxY;

		bool isDoor = (holeMaxY_updated - holeMinY_updated) > (holeMaxX_updated - holeMinX_updated) ? true : false;

		//if (!isDoor) {
		//	detectMinX = holeMinX_updated;
		//	detectMinY = holeMinY_updated;
		//	detectMaxX = holeMaxX_updated;
		//	detectMaxY = holeMaxY_updated;
		//}
		//cout << "zhujunqing start doorFrame==========" << endl;
		//cout << " zhujuniqng show rasterWidth === " << rasterWidth << endl;
		if (true) {
			cv::Mat clipedImg, blurImg, cannyImg;
			std::vector<cv::Vec4i> lines;
			// 1. left doorframe
			roi_min_x = std::max<int>(0, holeMinX_updated - clipWidth);
			//for test 

			//roi_min_x = std::max<int>(0, holeMinX_updated - (holeMaxX_updated - holeMinX_updated));

			roi_min_y = holeMinY_updated;
			roi_max_x = holeMinX_updated;
			roi_max_y = holeMaxY_updated;
			//for test
			//roi_min_x = roi_max_x + 1 < 20 ? 0 : 20;

			detectMinX = roi_max_x;
			if (!(roi_min_x == roi_max_x || roi_min_y == roi_max_y)) {
				clipedImg = preBlurImg(cv::Range(roi_min_y, roi_max_y + 1), cv::Range(roi_min_x, roi_max_x + 1));
				//cv::imshow("roi_-x_" + to_string(wall_id), clipedImg);
				cv::GaussianBlur(clipedImg, blurImg, cv::Size(1, 5), 5);
				// add where(rows != 0)
				// where(!=0) + unique + sort
				cv::Mat statMat;
				for (int i = 0; i < blurImg.rows; i++) {
					bool isRowExistNonZero = false;
					for (int j = 0; j < blurImg.cols; j++) {
						if (blurImg.at<uchar>(i, j) != 0) {
							isRowExistNonZero = true;
							break;
						}
					}
					if (isRowExistNonZero) {
						statMat.push_back(blurImg.row(i));
					}
				}
				if (statMat.rows > 0) {
					//cv::imshow("statMat-x" + to_string(wall_id), statMat);
					cannyImg.release();
					cv::Canny(statMat, cannyImg, 50, 150);
					cv::HoughLinesP(cannyImg, lines, 1, CV_PI / 180, 50, 0.3 * statMat.rows, 0.05 * statMat.rows);
					if (lines.size() > 0) {
						detectMinX = std::min<int>(roi_min_x + (*std::min_element(lines.begin(), lines.end(), compVec4iX))[0], detectMinX);
						// DEBUG : show lines
						/*for(auto line : lines) {}*/
					}
					//cv::namedWindow("statMat_left");
					//cv::imshow("statMat_left", cannyImg);
					//cv::waitKey(0);
					cv::Mat compHoughImg;
					cv::cvtColor(cannyImg, compHoughImg, cv::COLOR_GRAY2RGB);
					for (size_t i = 0; i < lines.size(); i++)
					{
						cv::line(compHoughImg, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 0, 255), 3, 8);
					}

					//cv::namedWindow("compHoughImg_left");
					//cv::imshow("compHoughImg_left", compHoughImg);
					//cv::waitKey(0);

				}
				//std::cout << "detectMinX:" << detectMinX << std::endl;
			}
			// 2. right doorframe
			roi_min_x = holeMaxX_updated;
			roi_min_y = holeMinY_updated;
			roi_max_x = min(rasterWidth - 1, holeMaxX_updated + clipWidth);
			//for test
			//roi_max_x = min(rasterWidth - 1, holeMaxX_updated + (holeMaxX_updated - holeMinX_updated));;
			roi_max_y = holeMaxY_updated;
			detectMaxX = roi_min_x;
			if (!(roi_min_x == roi_max_x || roi_min_y == roi_max_y)) {
				clipedImg = preBlurImg(cv::Range(roi_min_y, roi_max_y + 1), cv::Range(roi_min_x, roi_max_x + 1));
				cv::GaussianBlur(clipedImg, blurImg, cv::Size(1, 5), 5);

				// add where(rows != 0)
				// where(!=0) + unique + sort
				cv::Mat statMat;
				for (int i = 0; i < blurImg.rows; i++) {
					bool isRowExistNonZero = false;
					for (int j = 0; j < blurImg.cols; j++) {
						if (blurImg.at<uchar>(i, j) != 0) {
							isRowExistNonZero = true;
							break;
						}
					}
					if (isRowExistNonZero) {
						statMat.push_back(blurImg.row(i));
					}
				}
				if (statMat.rows > 0) {
					cannyImg.release();
					cv::Canny(statMat, cannyImg, 50, 150);
					cv::HoughLinesP(cannyImg, lines, 1, CV_PI / 180, 50, 0.3 * statMat.rows, 0.05 * statMat.rows);
					if (lines.size() > 0) {
						detectMaxX = std::max<int>(roi_min_x + (*std::max_element(lines.begin(), lines.end(), compVec4iX))[0], detectMaxX);
						// DEBUG : show lines
						/*for(auto line : lines) {					}*/
					}

					//cv::namedWindow("statMat_right");
				//	cv::imshow("statMat_right", cannyImg);
				//	cv::waitKey(0);
					cv::Mat compHoughImg;
					cv::cvtColor(cannyImg, compHoughImg, cv::COLOR_GRAY2RGB);
					for (size_t i = 0; i < lines.size(); i++)
					{
						cv::line(compHoughImg, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 0, 255), 3, 8);
					}

					//cv::namedWindow("compHoughImg_right");
					//cv::imshow("compHoughImg_right", compHoughImg);
					//cv::waitKey(0);
				}
			}
			//std::cout << "detectMaxX:" << detectMaxX << std::endl;

			// 3. up doorframe
			roi_min_x = holeMinX_updated;
			roi_min_y = max(0, holeMinY_updated - clipUpHeight);
			roi_max_x = holeMaxX_updated;
			roi_max_y = holeMinY_updated;
			detectMinY = roi_max_y;
			if (!(roi_min_x == roi_max_x || roi_min_y == roi_max_y)) {
				clipedImg = preBlurImg(cv::Range(roi_min_y, roi_max_y + 1), cv::Range(roi_min_x, roi_max_x + 1));

				//	cv::namedWindow("clipedImgFrameImg");
				//	cv::imshow("clipedImgFrameImg", clipedImg);

				cv::GaussianBlur(clipedImg, blurImg, cv::Size(5, 1), 5);

				// add where(cols != 0)
				// where(!=0) + unique + sort
				cv::Mat statMat;
				cv::transpose(blurImg, blurImg);
				for (int i = 0; i < blurImg.rows; i++) {
					bool isRowExistNonZero = false;
					for (int j = 0; j < blurImg.cols; j++) {
						if (blurImg.at<uchar>(i, j) != 0) {
							isRowExistNonZero = true;
							break;
						}
					}
					if (isRowExistNonZero) {
						statMat.push_back(blurImg.row(i));
					}
				}
				cv::transpose(statMat, statMat);


				//	cv::namedWindow("statMatFrameImg");
				//	cv::imshow("statMatFrameImg", statMat);

					//cv::imshow("statMat-Y" + to_string(wall_id), statMat);


				if (statMat.cols > 0) {
					cannyImg.release();
					cv::Canny(statMat, cannyImg, 50, 150);
					cv::HoughLinesP(cannyImg, lines, 1, CV_PI / 180, 50, 0.2 * statMat.cols, 0.05 * statMat.cols);

					if (DEBUG_DOOR_FRAME) {
						cv::Mat upDoorFrameImg = holeImg.clone();
						for (int m = 0; m < lines.size(); m++) {
							cv::line(upDoorFrameImg, cv::Point(lines[m][0], lines[m][1]), cv::Point(lines[m][2], lines[m][3]), cv::Scalar(0, 0, 255), 1);
						}
						cv::namedWindow("upDoorFrameImg");
						cv::imshow("upDoorFrameImg", upDoorFrameImg);
					}
					/*for (int i = 0; i < lines.size(); i++)
					{
						cout << "zhujunqing_linns_before===" << lines[i] << endl;
					}*/
					// filter
					while (true) {
						if (lines.size() > 0) {
							int minLineY = (*std::min_element(lines.begin(), lines.end(), compVec4iY))[1];
							if (minLineY > 0) {
								cv::Mat verifyImg = preBlurImg(cv::Range(minLineY - 1, minLineY), cv::Range(roi_min_x, roi_max_x));
								int cntZero = 0;
								for (int j = 0; j < verifyImg.cols; j++) {
									if (verifyImg.at<uchar>(0, j) == 0)		cntZero += 1;
								}
								if (0.2 * (roi_max_x - roi_min_x) < cntZero) {
									//filter lines
									std::vector<cv::Vec4i> filterLines;
									for (auto line : lines) {
										if (line[1] != minLineY) filterLines.push_back(line);
									}
									lines.assign(filterLines.begin(), filterLines.end());
								}
								else break;
							}
							else break;
						}
						else break;
					}
					/*for (int i = 0; i < lines.size(); i++)
					{
						cout << "zhujunqing_lins_after===" << lines[i] << endl;
					}*/
					if (lines.size() > 0) {

						//cout << "zhujunqing_roi_min_y===" << roi_min_y << endl;
						//cout << "zhujunqing_(*std::min_element(lines.begin(), lines.end(), compVec4iY))[1]===" << (*std::min_element(lines.begin(), lines.end(), compVec4iY))[1] << endl;
						//cout << "zhujunqing_detectMinY==before=" << detectMinY << endl;
						detectMinY = std::min<int>(roi_min_y + (*std::min_element(lines.begin(), lines.end(), compVec4iY))[1], detectMinY);
						//cout << "zhujunqing_detectMinY==after=" << detectMinY << endl;
						// DEBUG : show lines
						/*for(auto line : lines) {					}*/
					}
				}
			}
			//std::cout << "detectMinY:" << detectMinY << std::endl;

			// 4. bottom doorframe
			roi_min_x = holeMinX_updated;
			roi_min_y = holeMaxY_updated;
			roi_max_x = holeMaxX_updated;
			roi_max_y = min(clipedRasterHeight - 1, holeMaxY_updated + clipBottomHeight);
			detectMaxY = roi_min_y;
			if (!(roi_min_x == roi_max_x || roi_min_y == roi_max_y)) {
				clipedImg = preBlurImg(cv::Range(roi_min_y, roi_max_y + 1), cv::Range(roi_min_x, roi_max_x + 1));
				cv::GaussianBlur(clipedImg, blurImg, cv::Size(5, 1), 5);

				// add where(cols != 0)
				// where(!=0) + unique + sort
				cv::Mat statMat;
				cv::transpose(blurImg, blurImg);
				for (int i = 0; i < blurImg.rows; i++) {
					bool isRowExistNonZero = false;
					for (int j = 0; j < blurImg.cols; j++) {
						if (blurImg.at<uchar>(i, j) != 0) {
							isRowExistNonZero = true;
							break;
						}
					}
					if (isRowExistNonZero) {
						statMat.push_back(blurImg.row(i));
					}
				}
				cv::transpose(statMat, statMat);

				if (statMat.cols > 0) {
					cannyImg.release();
					cv::Canny(statMat, cannyImg, 50, 150);
					cv::HoughLinesP(cannyImg, lines, 1, CV_PI / 180, 50, 0.2 * statMat.cols, 0.05 * statMat.cols);

					if (lines.size() > 0) {
						detectMaxY = std::max<int>(roi_min_y + (*std::max_element(lines.begin(), lines.end(), compVec4iY))[1], detectMaxY);
						// DEBUG : show lines
						/*for(auto line : lines) {					}*/
					}
				}
			}
			//std::cout << "detectMaxY:" << detectMaxY << std::endl;
			//cout << "zhujunqing_beamHeight===" << beamHeight << endl;
			if (DEBUG_DOOR_FRAME || SAVE_OBSTACLES) {
				cv::rectangle(beforeCorrectImg, cv::Point(detectMinX, detectMinY + beamHeight), cv::Point(detectMaxX, detectMaxY + beamHeight), cv::Scalar(255, 255, 0), 1, 1);
			}

			// correct
			// top-left corner
			roi_min_x = max(0, holeMinX_updated - correctWidth);
			//cout << "zhujunqing_top-left corner=roi_min_x==" << roi_min_x << endl;
			roi_min_y = 0;
			roi_max_x = min(rasterWidth - 1, holeMinX_updated + correctWidth);
			//cout << "zhujunqing_top-left corner=roi_max_x==" << roi_max_x << endl;
			roi_max_y = min(clipedRasterHeight - 1, detectMinY);
			//cout << "zhujunqing_top-left corner=roi_max_y==" << roi_max_y << endl;
			bool isExistTopLeftLine = false;
			if (!(roi_min_x == roi_max_x || roi_min_y == roi_max_y) && roi_min_x > 2) { // roi_min_x > 2: Not close to border
				clipedImg = preBlurImg(cv::Range(roi_min_y, roi_max_y + 1), cv::Range(roi_min_x, roi_max_x + 1));
				cv::GaussianBlur(clipedImg, blurImg, cv::Size(1, 5), 5);

				//cv::namedWindow("preBlurImg1");
				//cv::imshow("preBlurImg1", blurImg);
			// add where(rows != 0)
			// where(!=0) + unique + sort
				cv::Mat statMat;
				for (int i = 0; i < blurImg.rows; i++) {
					bool isRowExistNonZero = false;
					for (int j = 0; j < blurImg.cols; j++) {
						if (blurImg.at<uchar>(i, j) != 0) {
							isRowExistNonZero = true;
							break;
						}
					}
					if (isRowExistNonZero) {
						statMat.push_back(blurImg.row(i));
					}
				}
				//cv::imshow("statMat_topleft", statMat);
				if (statMat.rows > 0) {
					cannyImg.release();
					cv::Canny(statMat, cannyImg, 50, 150);
					//cv::imshow("canny_topleft", cannyImg);
					cv::HoughLinesP(cannyImg, lines, 1, CV_PI / 180, int(0.6 * statMat.rows), 0.2 * statMat.rows, 0.05 * statMat.rows);
					if (lines.size() > 0) {
						for (auto line : lines) {
							// if exist vertical line
							if (line[0] == line[2] && line[0] > 1) { // line[0] > 1 make sure boundary
								bool hasEnoughNonZero = cv::countNonZero(statMat.col(line[0] - 1)) > 0.7 * statMat.rows;
								if (hasEnoughNonZero)
									isExistTopLeftLine = true;
							}
						}
					}
				}
			}
			// top-right corner
			roi_min_x = max(0, holeMaxX_updated - correctWidth);
			//cout << "zhujunqing_top-right corner=roi_min_x==" << roi_min_x << endl;
			roi_min_y = 0;
			roi_max_x = min(rasterWidth - 1, holeMaxX_updated + correctWidth);
			//cout << "zhujunqing_top-right corner=roi_max_x==" << roi_max_x << endl;
			roi_max_y = min(clipedRasterHeight - 1, detectMinY);
			//cout << "zhujunqing_top-right corner=roi_max_y==" << roi_max_y << endl;
			bool isExistTopRightLine = false;
			if (!(roi_min_x == roi_max_x || roi_min_y == roi_max_y) && roi_max_x < (rasterWidth - 2)) {// roi_max_x < (rasterWidth-2): Not close to border
				clipedImg = preBlurImg(cv::Range(roi_min_y, roi_max_y + 1), cv::Range(roi_min_x, roi_max_x + 1));
				cv::GaussianBlur(clipedImg, blurImg, cv::Size(1, 5), 5);
				// add where(rows != 0)
				// where(!=0) + unique + sort
				//cv::namedWindow("preBlurImg2");
				//cv::imshow("preBlurImg2", blurImg);
				cv::Mat statMat;
				for (int i = 0; i < blurImg.rows; i++) {
					bool isRowExistNonZero = false;
					for (int j = 0; j < blurImg.cols; j++) {
						if (blurImg.at<uchar>(i, j) != 0) {
							isRowExistNonZero = true;
							break;
						}
					}
					if (isRowExistNonZero) {
						statMat.push_back(blurImg.row(i));
					}
				}

				if (statMat.rows > 0) {
					cannyImg.release();
					cv::Canny(statMat, cannyImg, 50, 150);
					cv::HoughLinesP(cannyImg, lines, 1, CV_PI / 180, int(0.6 * statMat.rows), 0.2 * statMat.rows, 0.05 * statMat.rows);
					if (lines.size() > 0) {
						for (auto line : lines) {
							// if exist vertical line
							if (line[0] == line[2] && line[0] < (statMat.cols - 1)) { // line[0] < (statMat.rows - 1) make sure boundary
								bool hasEnoughNonZero = cv::countNonZero(statMat.col(line[0] + 1)) > 0.7 * statMat.rows;
								if (hasEnoughNonZero)
									isExistTopRightLine = true;
							}
						}
					}
				}
			}
			if (DEBUG_DOOR_FRAME) {
				std::cout << "isExistTopLeftLine:" << isExistTopLeftLine << std::endl;
				std::cout << "isExistTopRightLine:" << isExistTopRightLine << std::endl;
			}

			if (isExistTopLeftLine || isExistTopRightLine)
				detectMinY = 0;
		}

		// save box to detectBoxArray
		detectBoxArray.conservativeResize(detectBoxArray.rows() + 1, detectBoxArray.cols());
		Eigen::Array4i newBox(detectMinX, detectMinY, detectMaxX, detectMaxY);
		//std::cout << "detectBoxArray before add:" << detectBoxArray << std::endl;
		detectBoxArray.row(detectBoxArray.rows() - 1) = newBox;
		//std::cout << "detectBoxArray after add:" << detectBoxArray << std::endl;
      
		// ************ end doorframe detection **********

	}

	if (DEBUG_DOOR_FRAME) {
		cv::namedWindow("beforeCorrectImg");
		cv::imshow("beforeCorrectImg", beforeCorrectImg);
	}

	//merge box and save result
	//std::cout << "detectBoxArray before merge:" << detectBoxArray << std::endl;
	if (detectBoxArray.rows() > 0) {
		auto us = Union_set(detectBoxArray);
		for (int i = 0; i < detectBoxArray.rows(); i++) {
			for (int j = i + 1; j < detectBoxArray.rows(); j++) {
				if (!(detectBoxArray(i, 2) < detectBoxArray(j, 0) - doorFrameMergeGap || detectBoxArray(i, 3) < detectBoxArray(j, 1) - doorFrameMergeGap || detectBoxArray(j, 2) + doorFrameMergeGap < detectBoxArray(i, 0) || detectBoxArray(j, 3) + doorFrameMergeGap < detectBoxArray(i, 1))) {
					us.unionElement(i, j);
				}

			}
		}
		Eigen::ArrayX4i	 filterBoxList;
		for (int i = 0; i < detectBoxArray.rows(); i++) {
			if (us.father(i) == i) {
				filterBoxList.conservativeResize(filterBoxList.rows() + 1, filterBoxList.cols());
				filterBoxList.row(filterBoxList.rows() - 1) = us._detectBoxArray.row(i);
			}
		}
		//std::cout << "filterBoxList:" << filterBoxList << std::endl;
		detectBoxArray = filterBoxList;

		//std::cout << "detectBoxArray after merge:" << detectBoxArray << std::endl;
	}

	//NOTE: height should be added to beamHeight!!!!!!!!!!!!!!!!!!

	// save  result
	// expand doorframe result
	detectBoxArray.col(0) -= doorFrameExpandingValue;
	detectBoxArray.col(1) -= doorFrameExpandingValue;
	detectBoxArray.col(2) += doorFrameExpandingValue;
	detectBoxArray.col(3) += doorFrameExpandingValue;
	detectBoxArray.col(0) = detectBoxArray.col(0).cwiseMax(0);
	detectBoxArray.col(1) = detectBoxArray.col(1).cwiseMax(0);
	detectBoxArray.col(2) = detectBoxArray.col(2).cwiseMin(rasterWidth - 1);
	detectBoxArray.col(3) = detectBoxArray.col(3).cwiseMin(clipedRasterHeight - 1);

	// xmin,ymax,xmax,ymin -> xmin,ymin,xmax,ymax
	//detectBoxArray.col(1).swap(detectBoxArray.col(3));

	// save rasterDoorFrame Position
	rasterDoorFrameArray = detectBoxArray;
	obstacleInfo.doorFrameInfo.rasterDoorFrameArrayVector[origin_wall_id] = rasterDoorFrameArray;

	// save xz plane position
	Eigen::ArrayX4i detectBoxArrayXZ = rasterDoorFrameArray;
	detectBoxArrayXZ.col(0) = detectBoxArrayXZ.col(0) * pixelWidth;
	detectBoxArrayXZ.col(0) += minCloudPoints_X;
	detectBoxArrayXZ.col(1) = (rasterHeight - detectBoxArrayXZ.col(1)) * pixelHeight;
	detectBoxArrayXZ.col(1) += minCloudPoints_Z;
	detectBoxArrayXZ.col(2) = detectBoxArrayXZ.col(2) * pixelWidth;
	detectBoxArrayXZ.col(2) += minCloudPoints_X;
	detectBoxArrayXZ.col(3) = (rasterHeight - detectBoxArrayXZ.col(3)) * pixelHeight;
	detectBoxArrayXZ.col(3) += minCloudPoints_Z;
	obstacleInfo.doorFrameInfo.doorFrameArrayXZVector[origin_wall_id] = detectBoxArrayXZ;

	if (DEBUG_DOOR_FRAME || SAVE_OBSTACLES) {
		for (int i = 0; i < detectBoxArray.rows(); i++) {
			cv::rectangle(holeImg, cv::Point(detectBoxArray(i, 0), detectBoxArray(i, 1) + beamHeight), cv::Point(detectBoxArray(i, 2), detectBoxArray(i, 3) + beamHeight), cv::Scalar(255, 255, 0), 1, 1);
		}
	}

	for (int i = 0; i < detectBoxArray.rows(); i++) {

		// do not rotation by idxMat because some points has no corresponding idxMat map
		// idxMat(y, x)
		cv::Point3f bottomLeftPoint((detectBoxArray(i, 0)) * pixelWidth + minCloudPoints_X,
			meanCloudPoints_Y,
			(rasterHeight - (detectBoxArray(i, 1) + beamHeight)) * pixelHeight + minCloudPoints_Z);
		cv::Point3f bottomRightPoint((detectBoxArray(i, 2)) * pixelWidth + minCloudPoints_X,
			meanCloudPoints_Y,
			(rasterHeight - (detectBoxArray(i, 1) + beamHeight)) * pixelHeight + minCloudPoints_Z);
		cv::Point3f topRightPoint((detectBoxArray(i, 2)) * pixelWidth + minCloudPoints_X,
			meanCloudPoints_Y,
			(rasterHeight - (detectBoxArray(i, 3) + beamHeight)) * pixelHeight + minCloudPoints_Z);
		cv::Point3f topLeftPoint((detectBoxArray(i, 0)) * pixelWidth + minCloudPoints_X,
			meanCloudPoints_Y,
			(rasterHeight - (detectBoxArray(i, 3) + beamHeight)) * pixelHeight + minCloudPoints_Z);


		/*std::cout << "----bottomLeftPoint----:" << bottomLeftPoint << std::endl;
		std::cout << "----bottomRightPoint----:" << bottomRightPoint << std::endl;
		std::cout << "----topRightPoint----:" << topRightPoint << std::endl;
		std::cout << "----topLeftPoint----:" << topLeftPoint << std::endl;
		std::cout << "----idxMat----:" << idxMat(detectBoxArray(i, 3), detectBoxArray(i, 0)) << std::endl;
		std::cout << "----idxMat----:" << idxMat(detectBoxArray(i, 1), detectBoxArray(i, 2)) << std::endl;*/
		std::vector<cv::Point3f> pointsVector{ bottomLeftPoint, bottomRightPoint, topRightPoint, topLeftPoint };
		std::vector<cv::Point3f> coarsePointsVector, originPointsVector;
		MeasureBase::RotatePoints(pointsVector, backwardFineRotationMat, coarsePointsVector);
		MeasureBase::RotatePoints(coarsePointsVector, backwardCoarseRotationMat, originPointsVector);
		obstacleInfo.doorFrameInfo.wall_door_window_pos_3D[origin_wall_id].push_back(originPointsVector);
		/*std::cout << "**doorframe**" << std::endl;
		std::cout << "----bottomLeftPoint----:" << originPointsVector[0] << std::endl;
		std::cout << "----bottomRightPoint----:" << originPointsVector[1] << std::endl;
		std::cout << "----topRightPoint----:" << originPointsVector[2] << std::endl;
		std::cout << "----topLeftPoint----:" << originPointsVector[3] << std::endl;
		std::cout << "meanCloudPoints_Y: " << meanCloudPoints_Y << std::endl;*/
		//cout << "zhujunqing have door wall id == " << origin_wall_id << " door id == " << i << endl;
	}

	if (DEBUG_DOOR_FRAME) {
		cv::namedWindow("mergeImg");
		cv::imshow("mergeImg", holeImg);
		cv::waitKey(0);
	}
	if (SAVE_OBSTACLES) {
		cv::imwrite(SAVE_OBSTACLES_DIR + "/door_window_" + to_string(wall_id) + ".jpg", holeImg);
	}

	return true;
}


bool DetectObstacle::DetectCabinet(const std::vector<cv::Point3f>& cloudPoints, WallObstacleInfo& obstacleInfo) {
	//DEBUG: detection idx wall
	//if (!(wall_id == 5)) 
	//	return false;

	// clear history data
	rasterCabinetBoxArray.resize(0, 4);
	//std::cout << "rasterCabinetBoxArray.rows()" << rasterCabinetBoxArray.rows() << std::endl;
	//std::cout << "rasterCabinetBoxArray.cols()" << rasterCabinetBoxArray.cols() << std::endl;

	if (DEBUG_CABINET) std::cout << "--------------cabinet----------wall_id:" << wall_id << std::endl;


	DetectCabinetWithDepth(cloudPoints, wall_id, obstacleInfo);
	//DetectCabinetWithReflection(cloudPoints, wall_id, obstacleInfo);

	// merge result of depth and reflection
	obstacleInfo.cabinetInfo.cabinetBoxArrayXZVector[origin_wall_id] = Eigen::ArrayX4i(detectDepthCabinetArray.rows() + detectReflectionCabinetArray.rows(), detectDepthCabinetArray.cols());
	obstacleInfo.cabinetInfo.cabinetBoxArrayXZVector[origin_wall_id] << detectDepthCabinetArray, detectReflectionCabinetArray;
	auto us = Union_set(obstacleInfo.cabinetInfo.cabinetBoxArrayXZVector[origin_wall_id]);
	for (int i = 0; i < us._detectBoxArray.rows(); i++) {
		for (int j = i + 1; j < us._detectBoxArray.rows(); j++) {
			if (!(us._detectBoxArray(i, 2) < us._detectBoxArray(j, 0) || us._detectBoxArray(i, 3) < us._detectBoxArray(j, 1) || us._detectBoxArray(j, 2) < us._detectBoxArray(i, 0) || us._detectBoxArray(j, 3) < us._detectBoxArray(i, 1))) {
				us.unionElement(i, j);
			}
		}
	}
	Eigen::ArrayX4i	 filterBoxList;
	for (int i = 0; i < us._detectBoxArray.rows(); i++) {
		if (us.father(i) == i) {
			filterBoxList.conservativeResize(filterBoxList.rows() + 1, filterBoxList.cols());
			filterBoxList.row(filterBoxList.rows() - 1) = us._detectBoxArray.row(i);
		}
	}

	if (DEBUG_CABINET || SAVE_OBSTACLES) {
		cv::Mat cabinetResultImg;
		cv::cvtColor(rasterImg, cabinetResultImg, cv::COLOR_GRAY2RGB);
		for (int i = 0; i < filterBoxList.rows(); i++) {
			cv::rectangle(cabinetResultImg, cv::Point(filterBoxList(i, 0), filterBoxList(i, 1)), cv::Point(filterBoxList(i, 2), filterBoxList(i, 3)), cv::Scalar(255, 255, 0), 1, 1);
		}
		if (DEBUG_CABINET)
			cv::imwrite("G:/tmp/" + to_string(wall_id) + ".jpg", cabinetResultImg);
		if (SAVE_OBSTACLES)
			cv::imwrite(SAVE_OBSTACLES_DIR + "/cabinet_" + to_string(wall_id) + ".jpg", cabinetResultImg);
	}

	// save cabinet result
	// expand cabinet result
	filterBoxList.col(0) -= cabinetExpandingValue;
	filterBoxList.col(1) -= cabinetExpandingValue;
	filterBoxList.col(2) += cabinetExpandingValue;
	filterBoxList.col(3) += cabinetExpandingValue;
	filterBoxList.col(0) = filterBoxList.col(0).cwiseMax(0);
	filterBoxList.col(1) = filterBoxList.col(1).cwiseMax(0);
	filterBoxList.col(2) = filterBoxList.col(2).cwiseMin(rasterWidth - 1);
	filterBoxList.col(3) = filterBoxList.col(3).cwiseMin(rasterHeight - 1);

	// xmin,zmax,xmax,zmin -> xmin,zmin,xmax,zmax
	filterBoxList.col(1).swap(filterBoxList.col(3));

	// filter boxes which are intersect with doorFrame 
	for (int filterBoxID = 0; filterBoxID < filterBoxList.rows(); filterBoxID++) {
		bool isIntersect = false;
		Eigen::Array4i filterBox = filterBoxList.row(filterBoxID);
		for (int doorFrameBoxID = 0; doorFrameBoxID < obstacleInfo.doorFrameInfo.rasterDoorFrameArrayVector[origin_wall_id].rows(); doorFrameBoxID++) {
			Eigen::Array4i doorFrameBox = obstacleInfo.doorFrameInfo.rasterDoorFrameArrayVector[origin_wall_id].row(doorFrameBoxID);
			if (!(filterBox(2) < doorFrameBox(0) || filterBox(3) < doorFrameBox(1) || doorFrameBox(2) < filterBox(0) || doorFrameBox(3) < filterBox(1))) {
				isIntersect = true;
				break;
			}
		}
		if (!isIntersect) {
			rasterCabinetBoxArray.conservativeResize(rasterCabinetBoxArray.rows() + 1, rasterCabinetBoxArray.cols());
			rasterCabinetBoxArray.row(rasterCabinetBoxArray.rows() - 1) = filterBox;
		}
	}

	if (DEBUG_CABINET) {
		std::cout << "filterBoxList:" << std::endl << filterBoxList << std::endl;
		std::cout << "rasterCabinetBoxArray:" << std::endl << rasterCabinetBoxArray << std::endl;
		std::cout << "doorFrameBoxes:" << std::endl << obstacleInfo.doorFrameInfo.rasterDoorFrameArrayVector[origin_wall_id] << std::endl;
	}

	// save
	obstacleInfo.cabinetInfo.rasterCabinetBoxArrayVector[origin_wall_id] = rasterCabinetBoxArray;

	Eigen::ArrayX4i cabinetBoxArrayXZ = rasterCabinetBoxArray;
	// transform to XZplane 
	cabinetBoxArrayXZ.col(0) = cabinetBoxArrayXZ.col(0) * pixelWidth;
	cabinetBoxArrayXZ.col(0) += minCloudPoints_X;
	cabinetBoxArrayXZ.col(1) = (rasterHeight - cabinetBoxArrayXZ.col(1)) * pixelHeight;
	cabinetBoxArrayXZ.col(1) += minCloudPoints_Z;
	cabinetBoxArrayXZ.col(2) = cabinetBoxArrayXZ.col(2) * pixelWidth;
	cabinetBoxArrayXZ.col(2) += minCloudPoints_X;
	cabinetBoxArrayXZ.col(3) = (rasterHeight - cabinetBoxArrayXZ.col(3)) * pixelHeight;
	cabinetBoxArrayXZ.col(3) += minCloudPoints_Z;

	obstacleInfo.cabinetInfo.cabinetBoxArrayXZVector[origin_wall_id] = cabinetBoxArrayXZ;

	if (DEBUG_CABINET) {
		std::cout << "obstacleInfo.cabinetInfo.cabinetBoxArrayXZ:" << std::endl << obstacleInfo.cabinetInfo.cabinetBoxArrayXZVector[origin_wall_id] << std::endl;
	}

	return true;
}
bool DetectObstacle::DetectCabinetWithDepth(const std::vector<cv::Point3f>& cloudPoints, const int wall_id, ObstacleInfo& obstacleInfo)
{
	cv::Mat blurImg, cannyImg;
	cv::medianBlur(rasterImg, blurImg, 5);
	cv::Canny(blurImg, cannyImg, cannyMinThreshold, cannyMaxThreshold);

	cv::Mat cabinetImg;
	cv::cvtColor(rasterImg, cabinetImg, cv::COLOR_GRAY2RGB);

	cv::Mat dilateImg, erodeImg;
	cv::dilate(cannyImg, dilateImg, cv::Mat::ones(3, 3, CV_8U));
	cv::erode(dilateImg, erodeImg, cv::Mat::ones(3, 3, CV_8U));

	//cv::namedWindow("CabineterodeImg");
	//cv::imshow("CabineterodeImg", erodeImg);
	//cv::waitKey(0);
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(erodeImg, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
	// return false?
	//if (contours.size() == 0) {
	//	return false;
	//}

	//cv::drawContours(cabinetImg, contours, -1, cv::Scalar(255,0,0), 1);
	//cv::namedWindow("drawContours");
	//cv::imshow("drawContours", cabinetImg);
	//cv::waitKey(0);


	Eigen::ArrayX4i detectBoxArray;

	// DEBUG
	//detectBoxArray.conservativeResize(detectBoxArray.rows() + 1, detectBoxArray.cols());
	//detectBoxArray.row(0) = Eigen::Array4i(0, 0, 50, 50);
	//detectBoxArray.conservativeResize(detectBoxArray.rows() + 1, detectBoxArray.cols());
	//detectBoxArray.row(1) = Eigen::Array4i(25, 25, 75, 75);
	//detectBoxArray.conservativeResize(detectBoxArray.rows() + 1, detectBoxArray.cols());
	//detectBoxArray.row(2) = Eigen::Array4i(65, 65, 100, 100);
	//detectBoxArray.conservativeResize(detectBoxArray.rows() + 1, detectBoxArray.cols());
	//detectBoxArray.row(3) = Eigen::Array4i(90, 90, 120, 120);
	//auto us = Union_set(detectBoxArray);
	//for (int i = 0; i < detectBoxArray.rows(); i++) {
	//	for (int j = i + 1; j < detectBoxArray.rows(); j++) {
	//		if (!(detectBoxArray(i, 2) < detectBoxArray(j, 0) - cabinetMergeGap || detectBoxArray(i, 3) < detectBoxArray(j, 1) - cabinetMergeGap || detectBoxArray(j, 2) + cabinetMergeGap < detectBoxArray(i, 0) || detectBoxArray(j, 3) + cabinetMergeGap < detectBoxArray(i, 1))) {
	//			us.unionElement(i, j);
	//		}
	//	}
	//}
	//for (int i = 0; i < detectBoxArray.rows(); i++) {
	//	//std::cout << "us.father(i):" << us.father(i) << std::endl;
	//	if (us.father(i) == i) {
	//		std::cout<<"box"<< us._detectBoxArray.row(i)<<std::endl;
	//	}
	//}

	// DEBUG
	cv::Mat tmpDepthCabinetImg = rasterImg.clone();
	cv::cvtColor(tmpDepthCabinetImg, tmpDepthCabinetImg, cv::COLOR_GRAY2RGB);

	for (auto cnt : contours) {
		cv::Rect bbox = cv::boundingRect(cnt);
		int x(bbox.x), y(bbox.y), w(bbox.width), h(bbox.height);

		if (DEBUG_CABINET) {
			cv::rectangle(cabinetImg, cv::Point(x, y), cv::Point(x + w, y + h), cv::Scalar(255, 0, 0), 1, 16);
			//cv::putText(cabinetImg, to_string(h/w), cv::Point(x+2, y+2), 1, 1, cv::Scalar(255, 255, 0));
		}

		// filter size/position
		if (w > 48 || h > 48)	continue; // too big
		if (w < 5 || h < 5)	continue; // too small
		if (y < 10)	continue; // too high
		if (y > (rasterImg.rows - 16) || (y + h) > (rasterImg.rows - 4)) continue; // too low
		if ((w / h) > 3 || (h / w) > 1.3) continue; // wrong ratio

		// filter hole
		cv::Mat bboxImg = rasterImg(bbox).clone();
		cv::Mat nonZeroBboxImgMask = bboxImg.clone();
		nonZeroBboxImgMask.setTo(1, bboxImg != 0);
		bool isHole = cv::sum(nonZeroBboxImgMask)[0] < 0.9 * bboxImg.rows * bboxImg.cols;
		if (isHole)
			continue;

		if (DEBUG_CABINET) {
			/*cv::namedWindow("cabinetImg");
			cv::imshow("cabinetImg", cabinetImg);
			cv::waitKey(0);*/
			cv::rectangle(tmpDepthCabinetImg, cv::Point(x, y), cv::Point(x + w, y + h), cv::Scalar(0, 0, 255), 1, 16);
		}

		// save box to detectBoxArray
		detectBoxArray.conservativeResize(detectBoxArray.rows() + 1, detectBoxArray.cols());
		Eigen::Array4i newBox(x, y, x + w, y + h);
		//std::cout << "detectBoxArray before add:" << detectBoxArray << std::endl;
		detectBoxArray.row(detectBoxArray.rows() - 1) = newBox;

	}
	if (DEBUG_CABINET) {
		cv::imwrite("G:/tmp/depth_tmp_" + to_string(wall_id) + ".jpg", tmpDepthCabinetImg);
	}
	// merge box and save
	//std::cout << "detectBoxArray before merge:" << detectBoxArray << std::endl;
	if (detectBoxArray.rows() > 0) {
		auto us = Union_set(detectBoxArray);
		for (int i = 0; i < us._detectBoxArray.rows(); i++) {
			for (int j = i + 1; j < us._detectBoxArray.rows(); j++) {
				if (!(us._detectBoxArray(i, 2) < us._detectBoxArray(j, 0) - cabinetMergeGap || us._detectBoxArray(i, 3) < us._detectBoxArray(j, 1) - cabinetMergeGap || us._detectBoxArray(j, 2) + cabinetMergeGap < us._detectBoxArray(i, 0) || us._detectBoxArray(j, 3) + cabinetMergeGap < us._detectBoxArray(i, 1))) {
					us.unionElement(i, j);
				}
			}
		}
		Eigen::ArrayX4i	 filterBoxList;
		for (int i = 0; i < us._detectBoxArray.rows(); i++) {
			//std::cout << "us.father(i):" << us.father(i) << std::endl;
			if (us.father(i) == i) {
				//std::cout << "us.father == i" << std::endl;
				int x = us._detectBoxArray(i, 0);
				int y = us._detectBoxArray(i, 1);
				int w = us._detectBoxArray(i, 2) - us._detectBoxArray(i, 0);
				int h = us._detectBoxArray(i, 3) - us._detectBoxArray(i, 1);

				//fliter
				if (w > 48 || h > 48)	continue; // too big
				if (w < 5 || h < 5)	continue; // too small
				if (y < 10)	continue; // too high
				if (y > (rasterImg.rows - 16) || (y + h) > (rasterImg.rows - 4)) continue; // too low
				if ((w / h) > 3 || (h / w) > 1.3) continue; // wrong ratio

				filterBoxList.conservativeResize(filterBoxList.rows() + 1, filterBoxList.cols());
				filterBoxList.row(filterBoxList.rows() - 1) = us._detectBoxArray.row(i);
			}
		}
		//std::cout << "filterBoxList:" << filterBoxList << std::endl;
		detectBoxArray = filterBoxList;

		//std::cout << "detectBoxArray after merge:" << detectBoxArray << std::endl;
	}

	detectDepthCabinetArray = detectBoxArray;

	if (DEBUG_CABINET) {
		cv::Mat resultDepthCabinetImg = rasterImg.clone();
		cv::cvtColor(resultDepthCabinetImg, resultDepthCabinetImg, cv::COLOR_GRAY2RGB);

		for (int i = 0; i < detectBoxArray.rows(); i++) {
			cv::rectangle(resultDepthCabinetImg, cv::Point(detectBoxArray(i, 0), detectBoxArray(i, 1)), cv::Point(detectBoxArray(i, 2), detectBoxArray(i, 3)), cv::Scalar(255, 255, 0), 1, 1);
		}
		cv::imwrite("G:/tmp/depth_" + to_string(wall_id) + ".jpg", resultDepthCabinetImg);
		//cv::namedWindow("erodeImg");
		//cv::imshow("erodeImg", erodeImg);
		//cv::namedWindow("blurImg");
		//cv::imshow("blurImg", blurImg);
		/*cv::namedWindow("cannyImg");
		cv::imshow("cannyImg", cannyImg);
		cv::namedWindow("cabinetImg");
		cv::imshow("cabinetImg", cabinetImg);
		cv::waitKey(0);*/
	}

	return true;
}
bool DetectObstacle::JudgeDoorInGlassWall(const std::vector<std::vector<cv::Point3f>> plane_door_window_pos)
{
	float windowWidth;
	float windowHeight;
	float rasterWindowHeight; // raster rasterWindowHeight height
	float rasterWindowWidth;
	Eigen::Array2Xf holePointsArray;
	// WRONG: loop m_info_holes (size is different)
	/*for (int hole_id = 0; hole_id < obstacleInfo.doorFrameInfo.wall_door_window_pos.size(); hole_id++) {
		if (wall_id == scene_filter_output.wall_vertices[hole_id].first) {*/
	if (plane_door_window_pos.size() < 1)
		return false;
	for (int hole_id = 0; hole_id < plane_door_window_pos.size(); hole_id++) {

		if (hole_id > 1)
			return false;
		cv::Point3f cornerPoint1 = plane_door_window_pos[hole_id][0];
		cv::Point3f cornerPoint2 = plane_door_window_pos[hole_id][2];
		cv::Point3f cornerPoint1_coarse;
		cv::Point3f cornerPoint1_fine;
		cv::Point3f cornerPoint2_coarse;
		cv::Point3f cornerPoint2_fine;

		// rotate to 2d
		MeasureBase::RotatePoint(cornerPoint1, coarseRotationMat, cornerPoint1_coarse);
		MeasureBase::RotatePoint(cornerPoint1_coarse, fineRotationMat, cornerPoint1_fine);
		MeasureBase::RotatePoint(cornerPoint2, coarseRotationMat, cornerPoint2_coarse);
		MeasureBase::RotatePoint(cornerPoint2_coarse, fineRotationMat, cornerPoint2_fine);

		Eigen::Array2f cornerPoint1_eigen(cornerPoint1_fine.x, cornerPoint1_fine.z);
		Eigen::Array2f cornerPoint2_eigen(cornerPoint2_fine.x, cornerPoint2_fine.z);


		//cout << "zhujunqing wallWidth =  " << wallWidth << endl;
		//cout << "zhujunqing wallHeigth =  " << wallHeight << endl;
		windowWidth = abs(cornerPoint1_fine.x - cornerPoint2_fine.x);
		windowHeight = abs(cornerPoint1_fine.z - cornerPoint2_fine.z);
		rasterWindowWidth = windowWidth / pixelWidth + 1;
		rasterWindowHeight = windowHeight / pixelHeight + 1;
		//cout << "zhujunqing window width  = " << abs(cornerPoint1_fine.x - cornerPoint2_fine.x) << endl;
		//cout << "zhujunqing window height  = " << abs(cornerPoint1_fine.z - cornerPoint2_fine.z) << endl;

		/*std::cout << "*****cornerPoint1_eigen" << std::endl << cornerPoint1_eigen << std::endl;
		std::cout << "*****cornerPoint2_eigen" << std::endl << cornerPoint2_eigen << std::endl;*/
		//holePointsArray.conservativeResize(holePointsArray.rows(), holePointsArray.cols() + 2);
		//holePointsArray.col(holePointsArray.cols() - 2) = cornerPoint1_eigen;
		//holePointsArray.col(holePointsArray.cols() - 1) = cornerPoint2_eigen;
	}
	float widthRate = windowWidth / wallWidth;
	float heightRate = windowHeight / wallHeight;
	if (widthRate > 0.3 && heightRate > 0.9)
	{
		//cout << " zhujunqing widthRate = " << widthRate << endl;
		//cout << " zhujunqing heightRate = " << heightRate << endl;

		// clear history value
		rasterBeamHeight = 0;
		cv::Mat filterImg = rasterImg.clone();
		for (int i = 0; i < filterImg.rows; ++i) {
			for (int j = 0; j < filterImg.cols; ++j) {
				if (filterImg.at<uchar>(i, j) == 0) {
					filterImg.at<uchar>(i, j) = 110;//125 to 110 by zhujunqing for beam detect
				}
			}
		}
		//cv::namedWindow("filterImg");
		//cv::imshow("filterImg", filterImg);
		//cv::waitKey(0);

		/************ dilate & erode projected 2D image **********/

		cv::Mat projected_2Dimage_dilated;
		cv::dilate(filterImg, projected_2Dimage_dilated, cv::Mat::ones(15, 15, CV_8U));

		//cv::namedWindow("projected_2Dimage_dilated");
		//cv::imshow("projected_2Dimage_dilated", projected_2Dimage_dilated);
		//cv::waitKey(0);

		cv::Mat projected_2Dimage_eroded;
		cv::erode(projected_2Dimage_dilated, projected_2Dimage_eroded, cv::Mat::ones(5, 5, CV_8U));

		//cv::namedWindow("projected_2Dimage_eroded");
		//cv::imshow("projected_2Dimage_eroded", projected_2Dimage_eroded);
		//cv::waitKey(0);
		// erode dilate
		/*
		cv::Mat projected_2Dimage_eroded;
		cv::erode(GblurImg, projected_2Dimage_eroded, cv::Mat::ones(5, 5, CV_8U));

		cv::namedWindow("projected_2Dimage_eroded");
		cv::imshow("projected_2Dimage_eroded", projected_2Dimage_eroded);
		cv::waitKey(0);

		cv::Mat projected_2Dimage_dilated;
		cv::dilate(projected_2Dimage_eroded, projected_2Dimage_dilated, cv::Mat::ones(5, 5, CV_8U));

		cv::namedWindow("projected_2Dimage_dilated");
		cv::imshow("projected_2Dimage_dilated", projected_2Dimage_dilated);
		cv::waitKey(0);
		*/
		//cv::waitKey(0);
		//cv::Mat convertScale;
		//cv::convertScaleAbs(GblurImg, convertScale,1.5,10);


		//cv::namedWindow("convertScale");
		//cv::imshow("convertScale", convertScale);
		//cv::waitKey(0);

		//cv::Mat lPMat;
		//cv::Laplacian(GblurImg, lPMat, CV_8U);
		//cv::namedWindow("lPMat");
		//cv::imshow("lPMat", lPMat);
		//cv::waitKey(0);
		//cv::Mat binaryImg = blurImg.clone();
		//cv::threshold(convertScale, binaryImg, 0, 255, cv::THRESH_OTSU);
		//binaryImg = GblurImg.clone();
		//cv::namedWindow("testbeamImg");
		//cv::imshow("testbeamImg", binaryImg);
		//cv::waitKey(0);
		//cv::Mat grad_x, grad_y;
		//cv::Mat abs_grad_x, abs_grad_y, dst;
		//x 
		//cv::Sobel(projected_2Dimage_eroded, grad_x,CV_16S,1,0,3,1,1,cv::BORDER_DEFAULT);
		//cv::convertScaleAbs(grad_x, abs_grad_x);
		//cv::Sobel(projected_2Dimage_eroded, grad_y, CV_16S, 0, 1, 3, 1, 1, cv::BORDER_DEFAULT);
		//cv::convertScaleAbs(grad_y, abs_grad_y);
		//cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, dst);
		//dst.convertTo(dst, CV_8UC1);
		//cv::namedWindow("Sobel");
		//cv::imshow("Sobel", dst);
		//cv::waitKey(0);

		cv::Mat GblurImg;
		cv::GaussianBlur(projected_2Dimage_eroded, GblurImg, cv::Size(1, 5), 5);
		//cv::namedWindow("GblurImg");
		//cv::imshow("GblurImg", GblurImg);
		//cv::waitKey(0);

		cv::Mat binaryImg;
		cv::threshold(GblurImg, binaryImg, 0, 255, cv::THRESH_OTSU);
		//cv::namedWindow("threshold");
		//cv::imshow("threshold", binaryImg);
		//cv::waitKey(0);

		//test 
		cv::Mat cannyImg;
		std::vector<cv::Vec4i> lines;
		cv::Canny(binaryImg, cannyImg, 50, 150);


		//cv::HoughLinesP(cannyImg, lines, 1, CV_PI / 180, 50, 0.5 * cannyImg.cols, 0.03 * cannyImg.cols);
		//cv::namedWindow("cannyImg");
		//cv::imshow("cannyImg", cannyImg);
		//cv::waitKey(0);
		//cv::Mat compHoughImg;
		//cv::cvtColor(cannyImg, compHoughImg, cv::COLOR_GRAY2RGB);
		//for (size_t i = 0; i < lines.size(); i++)
		//{
		//	cv::line(compHoughImg, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 0, 255), 3, 8);
		//}

		//cv::namedWindow("compHoughImg");
		//cv::imshow("compHoughImg", compHoughImg);
		//cv::waitKey(0);



		std::vector<std::vector<cv::Point>> contours;
		cv::findContours(cannyImg, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		if (contours.size() == 0) {
			return false;
		}

		//cout << "zhujunqing find counters === " << contours.size() << endl;
		//binaryImg = 0;
		//cv::drawContours(binaryImg, contours, -1, cv::Scalar(125), 1);
		//cv::namedWindow("contours");
		//cv::imshow("contours", binaryImg);
		//cv::waitKey(0);


		std::vector<cv::Rect> rectVector;
		cv::Mat glassImg;
		cv::cvtColor(rasterImg, glassImg, cv::COLOR_GRAY2RGB);
		for (auto cnt : contours) {
			cv::Rect bbox = cv::boundingRect(cnt);
			float x(bbox.x), y(bbox.y), w(bbox.width), h(bbox.height);
		
			float rectWidthRate = w / rasterWindowWidth;
			float rectHeigthRate = h / rasterWindowHeight;

			//cout << " zhujunqing rect  w == " << w << "  h== " << h << endl;
			//cout << " zhujunqing rect  rectWidthRate == " << rectWidthRate << endl;
			//cout << " zhujunqing rect  rectHeigthRate == " << rectHeigthRate << endl;

			if (rectWidthRate<0.92 || rectWidthRate>1.08 || rectHeigthRate<0.8 || rectHeigthRate>1.2) continue;
			
			rectVector.push_back(bbox);
			// save result
			//float top_PointID_x = cloudPoints[idxMat(y, x)].x;
			//float top_PointID_y = cloudPoints[idxMat(y, x)].y;
			//float top_PointID_z = cloudPoints[idxMat(y, x)].z;
			//float beamWidth = w;
			//float beamHeight = h + beamExpandingValue; // return value
			//rasterBeamHeight = h + beamExpandingValue; // member value
			//beamResult = std::vector<float>{ top_PointID_x,top_PointID_y, top_PointID_z, beamWidth, beamHeight };

			if (DEBUG_GLASS || SAVE_OBSTACLES) {
				cv::rectangle(glassImg, cv::Point(x, y), cv::Point(x + w, y + h), cv::Scalar(0, 255, 0), 1, 16);
			}
		}
		
		if (DEBUG_GLASS) {
			cv::namedWindow("glassImg");
			cv::imshow("glassImg", glassImg);
			cv::waitKey(0);
		}
		if (SAVE_OBSTACLES) {
			cv::imwrite(SAVE_OBSTACLES_DIR + "/glass_" + to_string(wall_id) + ".jpg", glassImg);
		}
		if (rectVector.size() > 1)
		{
			//cout << "zhujunqing have two rect wall id = " << wall_id << " rectVector.size()== " << rectVector.size() << endl;
			return true;
		}
	}
	return false;
}