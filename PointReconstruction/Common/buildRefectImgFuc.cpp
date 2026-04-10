#include"buildRefectImgFuc.h"
#include "LookatOperation.h"
#include "MathOperation.hpp"
//#include <CGAL/Simple_cartesian.h> 
//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <Eigen/Dense>
#include <Eigen\src\Core\Matrix.h>
#include <opencv2\core\eigen.hpp>
//using namespace Eigen;
BuildRefect::BuildRefect()
{
}

BuildRefect::~BuildRefect()
{
} 

void BuildRefect::BuildRefectImageFuc(const std::vector<cv::Point3f>& points,
	const std::vector<unsigned char>& reflects,
	const cv::Point3f& pt_center,
	const cv::Point3f&pt_normal,
	cv::Mat & dstImg)
{
	cv::Point3f centre = pt_center;
	cv::Point3f normal = pt_normal;
	auto BuildRefectImageFuc = [&]()
	{
		Eigen::Vector3f centre_E(centre.x, centre.y, centre.z);
		Eigen::Vector3f normal_E(normal.x, normal.y, normal.z);
		auto word2local_ = [&]() {
			cv::Mat lookAt;
			CLookAt::GetLookAtMat(normal, centre, 100, lookAt);
			Eigen::Matrix4f mat;
			cv2eigen(lookAt, mat);
			return mat;
		};
		Eigen::Matrix4f mat = word2local_();
		Eigen::MatrixX3f points_E(points.size(), 3);
		auto transPoints = [&]() {
			for (int i = 0; i < points.size(); ++i)
			{
				Eigen::Vector4f temp(points[i].x, points[i].y, points[i].z, 1);
				temp = mat * temp;
				points_E.row(i) = Eigen::Vector3f(temp[0], temp[1], temp[2]);
			}
		};
		transPoints();

		Eigen::Vector3f voxelSize(10, 10, 10);
		Eigen::MatrixXf box(2, 3);
		box <<
			points_E.col(0).minCoeff(),
			points_E.col(1).minCoeff(),
			points_E.col(2).minCoeff(),
			points_E.col(0).maxCoeff(),
			points_E.col(1).maxCoeff(),
			points_E.col(2).maxCoeff();
		//cout << box;
		Eigen::Vector3i mapSize(ceil((box(1, 0) - box(0, 0)) / voxelSize[0])+1,
			ceil((box(1, 1) - box(0, 1)) / voxelSize[1])+1,
			ceil((box(1, 2) - box(0, 2)) / voxelSize[2])+1);
		Eigen::MatrixXf hashMap(mapSize[0], mapSize[1]);
		dstImg = cv::Mat::eye(mapSize[0], mapSize[1], CV_8UC1);
		//cv::imshow("test", dstImg*255);
		//cv::waitKey(0);
		std::vector<std::vector<std::vector<int>>> indexMapMat(mapSize[0], vector<vector<int>>(mapSize[1], vector<int>(0)));
		std::vector<std::vector<Eigen::Matrix3Xf>> meanCloudPointsMat(mapSize[0], vector<Eigen::Matrix3Xf>(mapSize[1], Eigen::Matrix3Xf(3, 0)));
		//MatrixXi reflectMap(mapSize[0], mapSize[1]);
		Eigen::MatrixXi reflectMap = Eigen::MatrixXi::Constant(mapSize[1], mapSize[0], 0);
		for (int id = 0; id < points_E.rows(); ++id)
		{
			Eigen::Vector3f temp = ((points_E.row(id) - box.row(0)));
			Eigen::Vector3i idNow(int(temp(0) / voxelSize(0)), int(temp(1) / voxelSize(1)), int(temp(2) / voxelSize(2)));
			indexMapMat[idNow(0)][idNow(1)].push_back(id);
			(meanCloudPointsMat[idNow(0)][idNow(1)]).conservativeResize(3, (meanCloudPointsMat[idNow(0)][idNow(1)]).cols() + 1);
			meanCloudPointsMat[idNow(0)][idNow(1)].col(meanCloudPointsMat[idNow(0)][idNow(1)].cols() - 1) =
				points_E.row(id);
			reflectMap(idNow(1), idNow(0)) = max(reflectMap(idNow(1), idNow(0)), int(reflects[id]));
			//Vector3i idNow()
		}
		auto NormalizeMap = [&]() {
			//auto arr_Map = reflectMap.array();

			//double sum = std::accumulate(, std::end(arr_Map), 0.0);
			//double mean = sum / resultSet.size(); 

			//double accum = 0.0;
			//std::for_each(std::begin(resultSet), std::end(resultSet), [&](const double d) {
			//	accum += (d - mean) * (d - mean);
			//	});

			//auto map_reshap = reflectMap.reshape(); Eigen virsion 3.4
			int mean = reflectMap.sum()/(reflectMap.array()>0).count();
			reflectMap = (reflectMap.array() + (128 - mean)).matrix();
		};
		//NormalizeMap();

		cv::Mat reflectMap_cv;
		cv::eigen2cv(reflectMap, reflectMap_cv);
		reflectMap_cv.convertTo(dstImg, CV_8UC1);
		auto NormalizeMap_ = [&]() {
			auto FindImageFilterZeroMinMax = [](const cv::Mat& srcImage, float& min, float& max)
			{
				min = 256;
				max = -1;
				for (int row = 0; row < srcImage.rows; row++)
				{
					for (int col = 0; col < srcImage.cols; col++)
					{
						int val = srcImage.at<uchar>(row, col);
						if (val > 0)
						{
							if (val < min)
							{
								min = val;
							}

							if (val > max)
							{
								max = val;
							}
						}
					}
				}
			};
			auto ContrastStretchBetweenMinMax = [FindImageFilterZeroMinMax](const cv::Mat& srcImage, cv::Mat& srcImage_stretch)
			{
				srcImage_stretch = srcImage.clone();
				if (srcImage.empty()) {
					std::cout << "COneMeter::ContrastStretchBetweenMinMax image empty" << std::endl;
					return false;
				}

				//double minVal, maxVal;
				//cv::minMaxLoc(srcImage_stretch, &minVal, &maxVal);
				//std::cout << "fcvFind min_a=" << minVal << " max_b=" << maxVal << std::endl;

				float minVal, maxVal;
				FindImageFilterZeroMinMax(srcImage_stretch, minVal, maxVal);
				//srcImage_stretch =(srcImage_stretch - minVal)*255 / (maxVal - minVal);
				//std::cout << "minVal=" << minVal << " maxVal=" << maxVal << std::endl;

				//create lut table
				cv::Mat lut(1, 256, CV_8U);
				for (int i = 0; i < 256; i++) {
					if (i < minVal) lut.at<uchar>(i) = 0;
					else if (i > maxVal) lut.at<uchar>(i) = 255;
					else lut.at<uchar>(i) = static_cast<uchar>(255.0 * (((float)i - minVal) / (maxVal - minVal)));
				}
				//apply lut
				cv::LUT(srcImage_stretch, lut, srcImage_stretch);
				return true; 
			};
			cv::Mat srcImage_stretch;
			ContrastStretchBetweenMinMax(dstImg, srcImage_stretch);
			dstImg = srcImage_stretch.clone();
		}; NormalizeMap_();
		//cv::imshow("dstImg", dstImg);
		//cv::waitKey(0);   
	};
	BuildRefectImageFuc();
}
