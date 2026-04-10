#include "WallXZRefectImg.h"
#include "MeasureBase.h"
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#define SMOOTH_RASTER_IMG true

#define sparseDistanceToCameraThreshold1 300  // merge pixel in 1cm if less than this value
#define sparseDistanceToCameraThreshold2 400  // merge pixel in 3cm if less than this value

CWallXZRefectImg::CWallXZRefectImg() :pixelHeight(10), pixelWidth(10)
{

}

CWallXZRefectImg::~CWallXZRefectImg()
{
}

bool CWallXZRefectImg::GetXzPlane(
	const std::vector<cv::Point3f>&ori_plane,
	const cv::Point3f& plane_normal,
	const cv::Point3f& plane_center,
	std::vector<cv::Point3f>& xz_plane)
{
	cv::Mat ori2XzMat;
	cv::Mat xz2OriMat;
	return GetXZPlaneAndMatHelper(ori_plane,plane_normal,plane_center,xz_plane,ori2XzMat,xz2OriMat);
}

bool CWallXZRefectImg::GetReflectImgPixelTen(
	const std::vector<cv::Point3f>& xzCloudPoints,
	const std::vector<int> planeReflectValues,
	cv::Mat& reflectRasterImgTen)
{
	const int pixelHeight = 10;
	const int pixelWidth = 10;
	return RasterXZImgWithReflectionCommHelper(xzCloudPoints,planeReflectValues,pixelHeight,pixelWidth,reflectRasterImgTen);
}

bool CWallXZRefectImg::GetReflectImgPixelOne(
	const std::vector<cv::Point3f>& xzCloudPoints,
	const std::vector<int> planeReflectValues,
	cv::Mat& reflectRasterImgOne)
{
	const int pixelHeight = 1;
	const int pixelWidth = 1;
	return RasterXZImgWithReflectionCommHelper(xzCloudPoints, planeReflectValues, pixelHeight, pixelWidth, reflectRasterImgOne);
}

bool CWallXZRefectImg::GetXZPlaneAndMatHelper(
	const std::vector<cv::Point3f>&plane,
	const cv::Point3f& normal,
	const cv::Point3f& center,
	std::vector<cv::Point3f>& xz_plane,
	cv::Mat& ori2XzMat,
	cv::Mat& xz2OriMat)
{
	if (plane.size()==0)
	{
		std::cout << "CWallXZRefectImg::GetXZPlaneAndMat plane empty!" << std::endl;
		return false;
	}

	float plane_normal[3];
	plane_normal[0] = normal.x;
	plane_normal[1] = normal.y;
	plane_normal[2] = normal.z;
	float wall_normal_uniform[3];
	float wall_center[3] = { center.x, center.y,center.z };
	MeasureBase::UniformNormals(plane_normal, wall_center, wall_normal_uniform);

	CmptRotateXZPlaneMatHelper(plane_normal, ori2XzMat, xz2OriMat);

	//rotate to XZ plane
	MeasureBase::RotatePoints(plane, ori2XzMat, xz_plane);
	return true;
}


bool CWallXZRefectImg::CmptRotateXZPlaneMatHelper(const float* plane_normal, cv::Mat&ori2Xz, cv::Mat& xz2Ori)
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
	//BOOL_FUNCTION_CHECK(MeasureBase::RotatePoints(plane_points, rotation_matrix_to_y_coarse, rot_plane_points_to_y_coarse));
	// rotation 2
	//std::vector<cv::Point3f> rot_plane_points_to_y_fine;
	float rotation_angle_to_y_fine;
	MeasureBase::CalcAngleVectorYZ2YAxis(rot_plane_normal_to_y_coarse, &rotation_angle_to_y_fine);
	cv::Mat rotation_matrix_to_y_fine = MeasureBase::TranslateAngleAroundX2RotationMatrix(rotation_angle_to_y_fine);
	//MeasureBase::RotatePoints(rot_plane_points_to_y_coarse, rotation_matrix_to_y_fine, rot_plane_points_to_y_fine);

	/*Backward rotation matrix*/
	cv::Mat backward_rotation_matrix_from_y_coarse(3, 3, CV_32FC1);
	cv::Mat backward_rotation_matrix_from_y_fine(3, 3, CV_32FC1);
	cv::transpose(rotation_matrix_to_y_coarse, backward_rotation_matrix_from_y_coarse);
	cv::transpose(rotation_matrix_to_y_fine, backward_rotation_matrix_from_y_fine);

	cv::Mat or_xz = rotation_matrix_to_y_coarse*rotation_matrix_to_y_fine;
	cv::Mat xz_ori = backward_rotation_matrix_from_y_fine*backward_rotation_matrix_from_y_coarse;
	ori2Xz = or_xz.clone();
	xz2Ori = xz_ori.clone();
	return true;
}


bool CWallXZRefectImg::RasterXZImgCommHelper(
	const std::vector<cv::Point3f>& cloudPoints,
	const int& pixelHeight, 
	const int& pixelWidth,
	Eigen::MatrixXi& countMat,
	Eigen::ArrayXf& coordX, 
	Eigen::ArrayXf& coordZ,
	cv::Mat &rstImg,
	float minGlobalX /*= FLT_MAX*/, float maxGlobalX /*= -FLT_MAX*/,
	float minGlobalZ /*= FLT_MAX*/, float maxGlobalZ /*= -FLT_MAX*/)
{
	int numPoints = cloudPoints.size();

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

	float minCloudPoints_X = cloudPoints_mapX.minCoeff();
	float minCloudPoints_Y = cloudPoints_mapY.minCoeff();
	float minCloudPoints_Z = cloudPoints_mapZ.minCoeff();

	float maxCloudPoints_X = cloudPoints_mapX.maxCoeff();
	float maxCloudPoints_Y = cloudPoints_mapY.maxCoeff();
	float maxCloudPoints_Z = cloudPoints_mapZ.maxCoeff();

	minCloudPoints_X = min(minCloudPoints_X, minGlobalX);
	minCloudPoints_Z = min(minCloudPoints_Z, minGlobalZ);
	maxCloudPoints_X = max(maxCloudPoints_X, maxGlobalX);
	maxCloudPoints_Z = max(maxCloudPoints_Z, maxGlobalZ);

	float meanCloudPoints_Y = cloudPoints_mapY.mean();

	float deltaCloudPointsY = maxCloudPoints_Y - minCloudPoints_Y;

	float width = maxCloudPoints_X - minCloudPoints_X;
	float height = maxCloudPoints_Z - minCloudPoints_Z;

	coordX = (cloudPoints_mapX - minCloudPoints_X);
	coordZ = (cloudPoints_mapZ - minCloudPoints_Z);
	//biasY = (cloudPoints_mapY - minCloudPoints_Y);
	Eigen::ArrayXf biasY = (cloudPoints_mapY - minCloudPoints_Y);

	Eigen::ArrayXi pixelValue = (255 * (cloudPoints_mapY - cloudPoints_mapY.minCoeff()) / (cloudPoints_mapY.maxCoeff() - cloudPoints_mapY.minCoeff())).cast<int>();

	int rasterWidth = int(width / pixelWidth) + 1;
	int rasterHeight = int(height / pixelHeight) + 1;

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
	Eigen::MatrixXi idxMat = Eigen::MatrixXi::Constant(rasterHeight, rasterWidth, -1);
	std::vector<std::vector<std::vector<int>>> indexMapMat;
	indexMapMat.resize(rasterHeight);
	std::vector<std::vector<std::vector<int>>> croppedWallIndexMapMat;
	croppedWallIndexMapMat.resize(rasterHeight);
	std::vector<std::vector<Eigen::Matrix3Xf>> meanCloudPointsMat;
	meanCloudPointsMat.resize(rasterHeight);

	cv::Mat minRasterYMat = cv::Mat(rasterHeight, rasterWidth, CV_32FC1, cv::Scalar(maxCloudPoints_Y));
	cv::Mat maxRasterYMat = cv::Mat(rasterHeight, rasterWidth, CV_32FC1, cv::Scalar(minCloudPoints_Y));
	cv::Mat meanRasterBiasYMat = cv::Mat(rasterHeight, rasterWidth, CV_32FC1, cv::Scalar(0));

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

	cv::Mat rasterImg = cv::Mat::zeros(rasterHeight, rasterWidth, CV_8UC1);
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
	rstImg = rasterImg.clone();
	rasterImg.release();
	return true;
}


bool CWallXZRefectImg::RasterXZImgWithReflectionCommHelper(
	const std::vector<cv::Point3f>& xzCloudPoints,
	const std::vector<int> planeReflectValues,
	const int& pixelHeight,
	const int& pixelWidth, 
	cv::Mat& reflectRasterImg)
{
	Eigen::MatrixXi countMat;
	Eigen::ArrayXf coordX;
	Eigen::ArrayXf coordZ;
	cv::Mat rasterImg;
	//std::cout << " before=== pixelHeight, pixelWidth:" << pixelHeight << ", " <<pixelWidth << std::endl;
	RasterXZImgCommHelper(xzCloudPoints, pixelHeight, pixelWidth, countMat, coordX, coordZ, rasterImg);
	int rasterHeight= rasterImg.rows;
	int rasterWidth= rasterImg.cols;
	//std::cout << " after=== pixelHeight, pixelWidth:" << pixelHeight << ", " << pixelWidth << std::endl;
	std::vector<float> floatPlaneReflectValues(planeReflectValues.begin(), planeReflectValues.end());
	int numPoints = xzCloudPoints.size();
	Eigen::Map<Eigen::ArrayXf> ReflectValues(floatPlaneReflectValues.data(), numPoints);
	//std::cout<<"ReflectValues:" << ReflectValues << std::endl;
	Eigen::ArrayXi reflectPixelValue = (255 * (ReflectValues - ReflectValues.minCoeff()) / (ReflectValues.maxCoeff() - ReflectValues.minCoeff())).cast<int>();
	//Eigen::ArrayXi reflectPixelValue = ReflectValues.cast<int>();
	Eigen::MatrixXi reflectValueMat(Eigen::MatrixXi::Zero(rasterHeight, rasterWidth));
	for (int id = 0; id < numPoints; id++) {
		//std::cout << pixelValue(id) << std::endl;
		int axisY = int(coordZ(id) / pixelHeight);
		int axisX = int(coordX(id) / pixelWidth);
		reflectValueMat(axisY, axisX) += reflectPixelValue(id);
	}

	cv::Mat reflectRasterImg_tmp = cv::Mat::zeros(rasterHeight, rasterWidth, CV_8UC1);
	for (int i = 0; i < rasterImg.rows; ++i) {
		for (int j = 0; j < rasterImg.cols; ++j) {
			if (countMat(i, j) != 0) {
				reflectRasterImg_tmp.at<uchar>(i, j) = int(reflectValueMat(i, j) / countMat(i, j));
			}
		}
	}
	//flip
	cv::flip(reflectRasterImg_tmp, reflectRasterImg_tmp, 0);
	reflectRasterImg = reflectRasterImg_tmp.clone();
	reflectRasterImg_tmp.release();
	return true;
}


bool CWallXZRefectImg::FindXzPlaneMinMax(const std::vector<cv::Point3f>& xzcloudPoints, cv::Point3f& pt_min, cv::Point3f& pt_max)
{
	int numPoints = xzcloudPoints.size();
	if (0 == numPoints)
	{
		return false;
	}
	std::vector<float> cloudPoints_X(numPoints);
	std::vector<float> cloudPoints_Y(numPoints);
	std::vector<float> cloudPoints_Z(numPoints);
	for (auto iter = xzcloudPoints.begin(); iter < xzcloudPoints.end(); iter++) {
		int id = iter - xzcloudPoints.begin();
		cloudPoints_X[id] = xzcloudPoints[id].x;
		cloudPoints_Y[id] = xzcloudPoints[id].y;
		cloudPoints_Z[id] = xzcloudPoints[id].z;
	}

	Eigen::Map<Eigen::ArrayXf> cloudPoints_mapX(cloudPoints_X.data(), numPoints);
	Eigen::Map<Eigen::ArrayXf> cloudPoints_mapY(cloudPoints_Y.data(), numPoints);
	Eigen::Map<Eigen::ArrayXf> cloudPoints_mapZ(cloudPoints_Z.data(), numPoints);

	pt_min.x = cloudPoints_mapX.minCoeff();
	pt_min.y = cloudPoints_mapY.minCoeff();
	pt_min.z = cloudPoints_mapZ.minCoeff();
	pt_max.x = cloudPoints_mapX.maxCoeff();
	pt_max.y = cloudPoints_mapY.maxCoeff();
	pt_max.z = cloudPoints_mapZ.maxCoeff();
	return true;
}