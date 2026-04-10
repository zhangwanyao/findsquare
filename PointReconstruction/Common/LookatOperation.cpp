#include "LookatOperation.h"

CLookAt::CLookAt()
{
}

CLookAt::~CLookAt()
{
}

void CLookAt::GetLookAtMat(const cv::Point3f& plane_normal, const cv::Point3f& plane_center, const int& n, cv::Mat &lookAt)
{
	//set camera point
	//int n = 100;
	cv::Point3f camPoint;
	camPoint.x = plane_center.x + n*plane_normal.x;
	camPoint.y = plane_center.y + n*plane_normal.y;
	camPoint.z = plane_center.z + n*plane_normal.z;

	//x axis
	cv::Vec3f direction = cv::Vec3f(plane_normal.x, plane_normal.y, plane_normal.z);
	direction = cv::normalize(direction);


	cv::Vec3f up = cv::Vec3f(0, 0, 1);
	if (abs(plane_normal.z) > 0.9)
		up = cv::Vec3f(0, 1, 0);

	//y axis
	cv::Vec3f right = direction.cross(up);
	right = cv::normalize(right);

	//z axis
	cv::Vec3f cameraUp = right.cross(direction);
	cameraUp = cv::normalize(cameraUp);


	//rotation
	cv::Mat R = cv::Mat::eye(4, 4, CV_32F);
	//row 1
	R.at<float>(0, 0) = -right[0];
	R.at<float>(0, 1) = -right[1];
	R.at<float>(0, 2) = -right[2];
	//row 2
	R.at<float>(1, 0) = -cameraUp[0];
	R.at<float>(1, 1) = -cameraUp[1];
	R.at<float>(1, 2) = -cameraUp[2];
	//row 3
	R.at<float>(2, 0) = direction[0];
	R.at<float>(2, 1) = direction[1];
	R.at<float>(2, 2) = direction[2];

	//transform
	cv::Mat t = cv::Mat::eye(4, 4, CV_32F);
	t.at<float>(0, 3) = -camPoint.x;
	t.at<float>(1, 3) = -camPoint.y;
	t.at<float>(2, 3) = -camPoint.z;

	// compute lookAt Martix
	lookAt = R*t;
}

void CLookAt::RotationPoints(const cv::Mat& defPoints, cv::Mat &finalPoints, const cv::Mat& lookAt)
{

	for (int row = 0; row < defPoints.rows; row++)
	{
		finalPoints.at<float>(row, 0) = lookAt.at<float>(0, 0)*defPoints.at<float>(row, 0) +
			lookAt.at<float>(0, 1)*defPoints.at<float>(row, 1) +
			lookAt.at<float>(0, 2)*defPoints.at<float>(row, 2) +
			lookAt.at<float>(0, 3);

		finalPoints.at<float>(row, 1) = lookAt.at<float>(1, 0)*defPoints.at<float>(row, 0) +
			lookAt.at<float>(1, 1)*defPoints.at<float>(row, 1) +
			lookAt.at<float>(1, 2)*defPoints.at<float>(row, 2) +
			lookAt.at<float>(1, 3);

		finalPoints.at<float>(row, 2) = lookAt.at<float>(2, 0)*defPoints.at<float>(row, 0) +
			lookAt.at<float>(2, 1)*defPoints.at<float>(row, 1) +
			lookAt.at<float>(2, 2)*defPoints.at<float>(row, 2) +
			lookAt.at<float>(2, 3);
	}
}

void CLookAt::RotationPoints(const std::vector<cv::Point3f>& points, std::vector<cv::Point3f> &rpoints, const cv::Mat& lookAt)
{
	rpoints.resize(points.size());
	for (int i = 0; i < points.size(); ++i)
	{
		cv::Point3f p;
		p.x = lookAt.at<float>(0, 0)*points[i].x +
			lookAt.at<float>(0, 1)*points[i].y +
			lookAt.at<float>(0, 2)*points[i].z +
			lookAt.at<float>(0, 3);

		p.y = lookAt.at<float>(1, 0)*points[i].x +
			lookAt.at<float>(1, 1)*points[i].y +
			lookAt.at<float>(1, 2)*points[i].z +
			lookAt.at<float>(1, 3);

		p.z = lookAt.at<float>(2, 0)*points[i].x +
			lookAt.at<float>(2, 1)*points[i].y +
			lookAt.at<float>(2, 2)*points[i].z +
			lookAt.at<float>(2, 3);

		rpoints[i] = p;
	}

}

void CLookAt::RotationPointsPair(
	const std::vector<std::pair<float, cv::Point3f>>& vec_pair,
	std::vector<std::pair<float, cv::Point3f>>& vec_pair_lookat,
	const cv::Mat& lookAt)
{
	vec_pair_lookat.resize(vec_pair.size());
	for (int i = 0; i < vec_pair.size(); ++i)
	{
		float value = vec_pair[i].first;
		cv::Point3f point= vec_pair[i].second;

		cv::Point3f p;
		p.x = lookAt.at<float>(0, 0)*point.x +
			lookAt.at<float>(0, 1)*point.y +
			lookAt.at<float>(0, 2)*point.z +
			lookAt.at<float>(0, 3);

		p.y = lookAt.at<float>(1, 0)*point.x +
			lookAt.at<float>(1, 1)*point.y +
			lookAt.at<float>(1, 2)*point.z +
			lookAt.at<float>(1, 3);

		p.z = lookAt.at<float>(2, 0)*point.x +
			lookAt.at<float>(2, 1)*point.y +
			lookAt.at<float>(2, 2)*point.z +
			lookAt.at<float>(2, 3);

		vec_pair_lookat[i] = std::make_pair(value,p);
		
	}
}

void CLookAt::Rotation3dTo2dPoints(const std::vector<cv::Point3f>& points, std::vector<cv::Point2f> &rpoints, const cv::Mat& lookAt)
{
	rpoints.resize(points.size());
	for (int i = 0; i < points.size(); ++i)
	{
		cv::Point2f p;
		p.x = lookAt.at<float>(0, 0)*points[i].x +
			lookAt.at<float>(0, 1)*points[i].y +
			lookAt.at<float>(0, 2)*points[i].z +
			lookAt.at<float>(0, 3);

		p.y = lookAt.at<float>(1, 0)*points[i].x +
			lookAt.at<float>(1, 1)*points[i].y +
			lookAt.at<float>(1, 2)*points[i].z +
			lookAt.at<float>(1, 3);

		//p.z = lookAt.at<float>(2, 0)*points[i].x +
		//	lookAt.at<float>(2, 1)*points[i].y +
		//	lookAt.at<float>(2, 2)*points[i].z +
		//	lookAt.at<float>(2, 3);

		rpoints[i] = p;
	}
}
