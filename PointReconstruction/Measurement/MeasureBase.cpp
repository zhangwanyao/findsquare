#include "MeasureBase.h"
#include <omp.h>
#include <limits>
#include "../log/log.h"
#include "..\Common\Voxel.h"
#include <stack>
#include "../IO/include/InOutData.h"
#include <Eigen/Dense>
#include <algorithm>
#include <map>
#include "../Common/VariableDefine.h"
#include "../Common/MathOperation.h"
/**
* \brief destructor
*/
Info_Hole::~Info_Hole() {
	m_width.clear();
	m_height.clear();
	m_ruler_width.clear();
	m_ruler_height.clear();
	m_info_ruler_width.clear();
	m_info_ruler_height.clear();
}

void Info_Hole::findNNbrCorner(const cv::Point3f& qPt, size_t& nn_idx, float& nn_dist)
{
	nn_idx = 0;
	nn_dist = std::numeric_limits<float>::infinity();
	if (m_corners.empty()) return;
	for (size_t i = 0; i < m_corners.size(); i++)
	{
		float dist = static_cast<float>(cv::norm(qPt - m_corners[i]));
		if (dist < nn_dist) {
			nn_idx = i;
			nn_dist = dist;
		}
	}
}

/**
* \brief update ruler information for export
*/
bool Info_Hole::updateInfoRuler() {
	m_info_ruler_width.clear();
	m_info_ruler_height.clear();
	// data validation
	if (m_width.size() != m_ruler_width.size()) return false;
	if (m_height.size() != m_ruler_height.size()) return false;
	// update data in info_ruler
	size_t num_width = m_width.size();
	size_t num_height = m_height.size();
	m_info_ruler_width.resize(num_width);
	m_info_ruler_height.resize(num_height);
	for (size_t i = 0; i < num_width; i++) {
		Info_Ruler& ruler = m_info_ruler_width[i];
		ruler.is_valid = true;
		ruler.value = m_width[i];
		ruler.ruler_endpts.push_back(m_ruler_width[i].first);
		ruler.ruler_endpts.push_back(m_ruler_width[i].second);
		size_t idx_corner;
		float dist;
		findNNbrCorner(m_ruler_width[i].first, idx_corner, dist);
		ruler.endpt_intersect_pts.push_back(std::make_pair(m_corners[idx_corner], m_corners[idx_corner]));
		ruler.endpt_intersect_dist.push_back(std::make_pair(dist, dist));
		findNNbrCorner(m_ruler_width[i].second, idx_corner, dist);
		ruler.endpt_intersect_pts.push_back(std::make_pair(m_corners[idx_corner], m_corners[idx_corner]));
		ruler.endpt_intersect_dist.push_back(std::make_pair(dist, dist));
	}
	for (size_t i = 0; i < num_height; i++) {
		Info_Ruler& ruler = m_info_ruler_height[i];
		ruler.is_valid = true;
		ruler.value = m_height[i];
		ruler.ruler_endpts.push_back(m_ruler_height[i].first);
		ruler.ruler_endpts.push_back(m_ruler_height[i].second);
		size_t idx_corner;
		float dist;
		findNNbrCorner(m_ruler_height[i].first, idx_corner, dist);
		ruler.endpt_intersect_pts.push_back(std::make_pair(m_corners[idx_corner], m_corners[idx_corner]));
		ruler.endpt_intersect_dist.push_back(std::make_pair(dist, dist));
		findNNbrCorner(m_ruler_height[i].second, idx_corner, dist);
		ruler.endpt_intersect_pts.push_back(std::make_pair(m_corners[idx_corner], m_corners[idx_corner]));
		ruler.endpt_intersect_dist.push_back(std::make_pair(dist, dist));
	}
	return true;
}

/**
* \brief print info
*/
void Info_Hole::log() {
	std::cout << "info_hole: \n";
	std::cout << "idx_plane: " << m_idx_plane << "\n";
	std::cout << "type: " << m_type << "\n";
	std::cout << "corners: \n";
	for (size_t i = 0; i < m_corners.size(); i++) {
		cv::Point3f pt = m_corners[i];
		std::cout << "(" << pt.x << ", " << pt.y << ", " << pt.z << ")\n";
	}
	std::cout << "widths: \n";
	for (size_t i = 0; i < m_width.size(); i++) {
		std::cout << "value: " << m_width[i] << "\n";
		cv::Point3f start = m_ruler_width[i].first;
		cv::Point3f end = m_ruler_width[i].second;
		std::cout << "pos: " << "(" << start.x << ", " << start.y << ", " << start.z << "), "
			<< "(" << end.x << ", " << end.y << ", " << end.z << ")\n";
	}
	std::cout << "heights: \n";
	for (size_t i = 0; i < m_height.size(); i++) {
		std::cout << "value: " << m_height[i] << "\n";
		cv::Point3f start = m_ruler_height[i].first;
		cv::Point3f end = m_ruler_height[i].second;
		std::cout << "pos: " << "(" << start.x << ", " << start.y << ", " << start.z << "), "
			<< "(" << end.x << ", " << end.y << ", " << end.z << ")\n";
	}
}


int MeasureBase::largestRectangleArea(std::vector<int>& height, int& start, int& end, int& hi) {
	std::list<int> s;
	height.push_back(0);//?��???��??��??��??????��?��??��???��?��???��?????1?��???t??��?
	int maxSize = 0;
	int i = 0;
	while (i < height.size()) {
		if (s.empty() || height[i] >= height[s.back()])
			s.push_back(i++);
		else {
			int cur = height[s.back()];
			s.pop_back();
			int curSize = cur * sqrt(s.empty() ? i : i - 1 - s.back());
			if (curSize > maxSize)
			{
				if (s.empty())
				{
					start = 0;
					end = i;
				}
				else
				{
					start = s.back();
					end = i - 1;
				}
				hi = cur;
				maxSize = curSize;
			}

		}
	}
	return maxSize;
}

void MeasureBase::GetLargestRect(cv::Mat& img, cv::Rect& rc)
{
	int maxArea = 0;

	int startidx = 0;
	int endidx = 0;
	int heightidx = 0;
	int maxrow = 0;

	int maxArea_fake = 0;
	int startidx_fake = 0;
	int endidx_fake = 0;
	int heightidx_fake = 0;
	int maxrow_fake = 0;
	std::vector<cv::Point2i> start_ends;
	start_ends.resize(img.rows);
	for (int row = 0; row < img.rows; row++)
	{

		std::vector<cv::Point2i> start_end_tmp;
		start_end_tmp.push_back(cv::Point2i(-1, img.cols - 1));
		int curid = 0;
		for (int col = 0; col < img.cols; col++)
		{
			if (start_end_tmp[curid].x == -1)
			{
				if (img.at<uchar>(row, col) == 255)
					start_end_tmp[curid].x = col;
			}
			else if (start_end_tmp[curid].x > -1)
			{
				if (img.at<uchar>(row, col) < 255)
				{
					start_end_tmp[curid].y = col;
					start_end_tmp.push_back(cv::Point2i(-1, img.cols - 1));
					curid++;
				}
			}

		}

		//Find Max
		int maxId = 0;
		for (int i = 0; i < start_end_tmp.size(); i++)
		{
			if (start_end_tmp[i].x != -1)
			{
				if ((start_end_tmp[i].y - start_end_tmp[i].x) > (start_end_tmp[maxId].y - start_end_tmp[maxId].x))
					maxId = i;
			}
		}
		//std::cout << start_end_tmp[maxId].x << "," << start_end_tmp[maxId].y << std::endl;
		start_ends[row] = start_end_tmp[maxId];
		if (start_ends[row].x == -1)
		{
			start_ends[row].x = 0;
			start_ends[row].y = 0;
		}


	}

	//	for (int i = 0; i < start_ends.size(); i++)
		//	std::cout << start_ends[i].x << "," << start_ends[i].y << std::endl;

	for (int row = 0; row < img.rows; row++)
	{
		int start = start_ends[row].x;
		int end = start_ends[row].y;

		if (start == 0 && end == 0)
			continue;

		std::vector<int> height;
		height.resize(end - start + 1);
		int heightid = 0;
		for (int col = start; col <= end; col++)
		{
			bool bFound = false;
			for (int h = row; h >= 0; h--)
			{
				if (img.at<uchar>(h, col) != 255)
				{
					height[heightid] = row - h;
					bFound = true;
					break;
				}
			}
			if (!bFound)
				height[heightid] = row + 1;

			heightid++;
		}

		int curstartidx = 0;
		int curendidx = 0;
		int curheightidx = 0;
		int curArea = largestRectangleArea(height, curstartidx, curendidx, curheightidx);
		if (maxArea < curArea && curheightidx >= 125)
		{
			maxArea = curArea;

			startidx = curstartidx + start;
			endidx = curendidx + start;
			heightidx = curheightidx;
			maxrow = row;

		}

		if (maxArea_fake < curArea)
		{
			maxArea_fake = curArea;
			startidx_fake = curstartidx + start;
			endidx_fake = curendidx + start;
			heightidx_fake = curheightidx;
			maxrow_fake = row;
		}

	}

	//If can't find valid rectangle, use the fake rectangle
	if (maxArea < 1)
	{
		startidx = startidx_fake;
		endidx = endidx_fake;
		heightidx = heightidx_fake;
		maxrow = maxrow_fake;
	}

	cv::Rect rctmp(startidx, maxrow - heightidx, endidx - startidx, heightidx);
	rc = rctmp;
}
//bool MeasureBase::FindBigestValidMinMaxXYZ(const std::vector<cv::Point3f>& input_points, std::vector<array<float, 4>> &hole_min_max, const int normal_axis, float* in_output_minmax_xyz)
//{
//
//	if (input_points.size() == 0)
//	{
//		std::cout << "FindBigestValidMinMaxXYZ: empty points are input!" << std::endl;
//		log_error("FindBigestValidMinMaxXYZ: empty points are input!");
//		return false;
//	}
//
//	cv::Mat plane_pts_mat(input_points.size(), 3, CV_64F);
//
//	for (int i = 0; i < input_points.size(); i++) {
//		plane_pts_mat.at<double>(i, 0) = input_points[i].x;
//		plane_pts_mat.at<double>(i, 1) = input_points[i].y;
//		plane_pts_mat.at<double>(i, 2) = input_points[i].z;
//	}
//
//
//	/*********** project 3D plane to 2D image **********/
//	unsigned int image_rows;// = max_y - min_y;
//	unsigned int image_cols;// = max_x - min_x;
//	double min_y = in_output_minmax_xyz[4];
//	double max_y = in_output_minmax_xyz[5];
//	double min_x = in_output_minmax_xyz[0];
//	double max_x = in_output_minmax_xyz[1];
//	int    x_idx = 0;
//	int    y_idx = 1;
//	if (normal_axis == 1)
//	{
//		image_rows = in_output_minmax_xyz[5] - in_output_minmax_xyz[4];
//		image_cols = in_output_minmax_xyz[1] - in_output_minmax_xyz[0];
//
//		double min_y = in_output_minmax_xyz[4];
//		double min_x = in_output_minmax_xyz[0];
//		x_idx = 0;
//		y_idx = 2;
//	}
//	else if (normal_axis == 2)
//	{
//		image_rows = in_output_minmax_xyz[5] - in_output_minmax_xyz[4];
//		image_cols = in_output_minmax_xyz[3] - in_output_minmax_xyz[2];
//
//		min_y = in_output_minmax_xyz[4];
//		max_y = in_output_minmax_xyz[5];
//		min_x = in_output_minmax_xyz[2];
//		max_x = in_output_minmax_xyz[3];
//		x_idx = 1;
//		y_idx = 2;
//	}
//	unsigned int image_row, image_col;
//	cv::Mat projected_2Dimage = cv::Mat::zeros(image_rows + 1, image_cols + 1, CV_8U); //black background
//	for (int i = 0; i < plane_pts_mat.rows; i++) {
//		if (plane_pts_mat.at<double>(i, x_idx) >= min_x && plane_pts_mat.at<double>(i, x_idx) <= max_x &&
//			plane_pts_mat.at<double>(i, y_idx)  >= min_y && plane_pts_mat.at<double>(i, y_idx) <= max_y)
//		{
//			image_row = plane_pts_mat.at<double>(i, y_idx) - min_y;
//			image_col = plane_pts_mat.at<double>(i, x_idx) - min_x;
//			projected_2Dimage.at<uchar>(image_row, image_col) = 255; //white pixel
//		}
//	}
//
//
//	for (int i = 0; i < hole_min_max.size(); i++)
//	{
//		if (abs(hole_min_max[i][0] - min_x) < 200 || abs(hole_min_max[i][1] - max_x) < 200 || abs(hole_min_max[i][0] - max_x) < 200)
//		{
//			continue;
//		}
//		int px = hole_min_max[i][0] - min_x - 200;
//		int dx = hole_min_max[i][1] - min_x + 400;
//		if (px < 0)
//			px = 0;
//
//		dx -= px;
//		int py = hole_min_max[i][2] - min_y - 200;
//		int dy = hole_min_max[i][3] - min_y;
//		if (py < 0)
//		{
//			py = 0;
//		}
//
//		dy -= py;
//		cv::Rect holeRect(px, py, dx, dy);
//		//std::cout << px << " " << py << "w=" << dx << "h=" << dy << std::endl;
//		cv::rectangle(projected_2Dimage, holeRect, cv::Scalar(255), -1);
//		//if(g_vflag)
//		//cv::imshow("projected_img_small1111", projected_2Dimage);
//	}
//
//
//	/************ dilate & erode projected 2D image **********/
//	cv::Mat projected_2Dimage_dilated;
//	cv::dilate(projected_2Dimage, projected_2Dimage_dilated, cv::Mat::ones(25, 25, CV_8U));
//
//	cv::Mat projected_2Dimage_eroded;
//	cv::erode(projected_2Dimage_dilated, projected_2Dimage_eroded, cv::Mat::ones(25, 25, CV_8U));
//	
//	//SHOW
//	/*cv::namedWindow("peojected", cv::WINDOW_NORMAL);
//	cv::namedWindow("dilated", cv::WINDOW_NORMAL);
//	cv::namedWindow("eroded", cv::WINDOW_NORMAL);
//	cv::imshow("peojected", projected_2Dimage);
//	cv::imshow("dilated", projected_2Dimage_dilated);
//	cv::imshow("eroded", projected_2Dimage_eroded);
//	cv::waitKey(0);*/
//
//	/************ find external contour *******/
//	std::vector<std::vector<cv::Point>> contours;
//	cv::findContours(projected_2Dimage_eroded, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
//
//	if (contours.size() == 0)
//	{
//		std::cout << "FilterPlaneByEdgeThickness: No enough points!" << std::endl;
//		log_error("FilterPlaneByEdgeThickness: No enough points!");
//		return false;
//	}
//
//	cv::Mat non_edge_rotated_2D_XY_bigest_plane_ROI;
//
//	cv::Mat projected_img_small;
//	cv::resize(projected_2Dimage_eroded, projected_img_small, cv::Size(projected_2Dimage_eroded.cols*0.125, projected_2Dimage_eroded.rows*0.125));
//	
//	for (int n = 0; n < 2; n++)
//	{
//		for (int row = 0; row < projected_img_small.rows; row++)
//		{
//			for (int col = 0; col < projected_img_small.cols; col++)
//			{
//				int endcols = projected_img_small.cols - 1;
//				if (projected_img_small.at<uchar>(row, col) == 0)
//				{
//					int iLeft = col - 32 < 0 ? 0 : col - 32;
//					int iRight = col + 32 > endcols ? endcols : col + 32;
//					if (projected_img_small.at<uchar>(row, iLeft) == 255 &&
//						projected_img_small.at<uchar>(row, iRight) == 255)
//						projected_img_small.at<uchar>(row, col) = 255;
//				}
//			}
//		}
//	}
//	std::vector<std::vector<cv::Point>> contours_small;
//	std::vector<std::vector<cv::Point>> contours_draw;
//	cv::findContours(projected_img_small, contours_small, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
//
//	//cv::imshow("projected_img_small", projected_img_small);
//	int maxid = 0;
//	for (int i = 0; i < contours_small.size(); i++)
//	{
//		if (contours_small[i].size() > contours_small[maxid].size())
//			maxid = i;
//	}
//
//	contours_draw.push_back(contours_small[maxid]);
//	projected_img_small = 0;
//	cv::drawContours(projected_img_small, contours_draw, -1, cv::Scalar(255), -1);
//	
//	cv::Rect bigestRect;// = InSquare(projected_img_small, center);
//	GetLargestRect(projected_img_small, bigestRect);
//	//rectangle(projected_img_small, bigestRect, cv::Scalar(0), -1);
//	//cv::imshow("?????��?????��?����", projected_img_small);
//	//cv::waitKey(0);
//	//bigestRect.x += 1;
//	//bigestRect.y += 1;
//	bigestRect.x *= 8;
//	bigestRect.y *= 8;
//	bigestRect.width *= 8;
//	bigestRect.height *= 8;
//
//	
//	if (normal_axis == 1)
//	{
//		in_output_minmax_xyz[0] += bigestRect.x;
//		in_output_minmax_xyz[1] = in_output_minmax_xyz[0] + bigestRect.width;
//		//in_output_minmax_xyz[4] += bigestRect.y;
//		//in_output_minmax_xyz[5] = in_output_minmax_xyz[4] + bigestRect.height;
//	}
//	else if (normal_axis == 2)
//	{
//		in_output_minmax_xyz[2] += bigestRect.x;
//		in_output_minmax_xyz[3] = in_output_minmax_xyz[2] + bigestRect.width;
//		//in_output_minmax_xyz[4] += bigestRect.y;
//		//in_output_minmax_xyz[5] = in_output_minmax_xyz[4] + bigestRect.height;
//	}
//}


bool MeasureBase::Cut_FindBigestValidMinMaxXYZ(const std::vector<cv::Point3f>& input_points, const array<float, 4>& hole_min_max,
	const int normal_axis, float* in_output_minmax_xyz)
{

	if (input_points.size() == 0)
	{
		std::cout << "FindBigestValidMinMaxXYZ: empty points are input!" << std::endl;
		log_error("FindBigestValidMinMaxXYZ: empty points are input!");
		return false;
	}

	cv::Mat plane_pts_mat(input_points.size(), 3, CV_64F);

	for (int i = 0; i < input_points.size(); i++) {
		plane_pts_mat.at<double>(i, 0) = input_points[i].x;
		plane_pts_mat.at<double>(i, 1) = input_points[i].y;
		plane_pts_mat.at<double>(i, 2) = input_points[i].z;
	}

	/*********** project 3D plane to 2D image **********/
	unsigned int image_rows;// = max_y - min_y;
	unsigned int image_cols;// = max_x - min_x;
	double min_y = in_output_minmax_xyz[4];
	double max_y = in_output_minmax_xyz[5];
	double min_x = in_output_minmax_xyz[0];
	double max_x = in_output_minmax_xyz[1];

	int    x_idx = 0;
	int    y_idx = 1;
	if (normal_axis == 1)
	{
		image_rows = in_output_minmax_xyz[5] - in_output_minmax_xyz[4];
		image_cols = in_output_minmax_xyz[1] - in_output_minmax_xyz[0];

		double min_y = in_output_minmax_xyz[4];
		double min_x = in_output_minmax_xyz[0];
		x_idx = 0;
		y_idx = 2;
	}
	else if (normal_axis == 2)
	{
		image_rows = in_output_minmax_xyz[5] - in_output_minmax_xyz[4];
		image_cols = in_output_minmax_xyz[3] - in_output_minmax_xyz[2];

		min_y = in_output_minmax_xyz[4];
		max_y = in_output_minmax_xyz[5];
		min_x = in_output_minmax_xyz[2];
		max_x = in_output_minmax_xyz[3];
		x_idx = 1;
		y_idx = 2;
	}
	else if (normal_axis == 3)
	{
		image_cols = in_output_minmax_xyz[1] - in_output_minmax_xyz[0];
		image_rows = in_output_minmax_xyz[3] - in_output_minmax_xyz[2];

		min_y = in_output_minmax_xyz[2];
		max_y = in_output_minmax_xyz[3];
		min_x = in_output_minmax_xyz[0];
		max_x = in_output_minmax_xyz[1];
		x_idx = 0;
		y_idx = 1;
	}
	unsigned int image_row, image_col;
	cv::Mat projected_2Dimage = cv::Mat::zeros(image_rows + 1, image_cols + 1, CV_8U); //black background
	for (int i = 0; i < plane_pts_mat.rows; i++) {
		if (plane_pts_mat.at<double>(i, x_idx) >= min_x && plane_pts_mat.at<double>(i, x_idx) <= max_x &&
			plane_pts_mat.at<double>(i, y_idx) >= min_y && plane_pts_mat.at<double>(i, y_idx) <= max_y)
		{
			image_row = plane_pts_mat.at<double>(i, y_idx) - min_y;
			image_col = plane_pts_mat.at<double>(i, x_idx) - min_x;
			projected_2Dimage.at<uchar>(image_row, image_col) = 255; //white pixel
		}
	}


		int px = hole_min_max[0] - min_x - 200;
		int dx = hole_min_max[1] - min_x + 200;
		if (px < 0)
			px = 0;

		dx -= px;
		int py = hole_min_max[2] - min_y;
		int dy = hole_min_max[3] - min_y;
		if (py < 0)
		{
			py = 0;
		}

		dy -= py;
		cv::Rect holeRect(px, py, dx, dy);
		//std::cout << px << " " << py << "w=" << dx << "h=" << dy << std::endl;
		cv::rectangle(projected_2Dimage, holeRect, cv::Scalar(255), -1);
		//if(g_vflag)
	


	/************ dilate & erode projected 2D image **********/
	cv::Mat projected_2Dimage_dilated;
	cv::dilate(projected_2Dimage, projected_2Dimage_dilated, cv::Mat::ones(25, 25, CV_8U));

	cv::Mat projected_2Dimage_eroded;
	cv::erode(projected_2Dimage_dilated, projected_2Dimage_eroded, cv::Mat::ones(25, 25, CV_8U));

	/************ find external contour *******/
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(projected_2Dimage_eroded, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	if (contours.size() == 0)
	{
		std::cout << "FilterPlaneByEdgeThickness: No enough points!" << std::endl;
		log_error("FilterPlaneByEdgeThickness: No enough points!");
		return false;
	}

	cv::Mat non_edge_rotated_2D_XY_bigest_plane_ROI;

	cv::Mat projected_img_small;
	cv::resize(projected_2Dimage_eroded, projected_img_small, cv::Size(projected_2Dimage_eroded.cols * 0.125, projected_2Dimage_eroded.rows * 0.125));

	std::vector<std::vector<cv::Point>> contours_small;
	std::vector<std::vector<cv::Point>> contours_draw;
	cv::findContours(projected_img_small, contours_small, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	//cv::imshow("projected_img_small", projected_img_small);
	int maxid = 0;
	for (int i = 0; i < contours_small.size(); i++)
	{
		if (contours_small[i].size() > contours_small[maxid].size())
			maxid = i;
	}

	contours_draw.push_back(contours_small[maxid]);
	projected_img_small = 0;
	cv::drawContours(projected_img_small, contours_draw, -1, cv::Scalar(255), -1);


	cv::Rect bigestRect;// = InSquare(projected_img_small, center);
	GetLargestRect(projected_img_small, bigestRect);
	cv::rectangle(projected_img_small, bigestRect, cv::Scalar(0), -1);
	//cv::imshow("bigest_rect_projected_img_small", projected_img_small);
	//cv::waitKey(0);
	//bigestRect.x += 1;
	//bigestRect.y += 1;
	bigestRect.x *= 8;
	bigestRect.y *= 8;
	bigestRect.width *= 8;
	bigestRect.height *= 8;
	cv::rectangle(projected_2Dimage, bigestRect, cv::Scalar(255), -1);
	/*std::cout << "bigestRect.x" << bigestRect.x << std::endl;
	std::cout << "bigestRect.y" << bigestRect.y << std::endl;
	std::cout << "bigestRect.width" << bigestRect.width << std::endl;
	std::cout << "bigestRect.height" << bigestRect.height << std::endl;
	std::cout << "projected_img_small height:" << projected_img_small.rows << std::endl;
	std::cout << "projected_img_small width:" <<projected_img_small.cols << std::endl;
	*/
	if (normal_axis == 1)
	{
		in_output_minmax_xyz[0] += bigestRect.x;
		in_output_minmax_xyz[1] = in_output_minmax_xyz[0] + bigestRect.width;
		in_output_minmax_xyz[4] += bigestRect.y;
		in_output_minmax_xyz[5] = in_output_minmax_xyz[4] + bigestRect.height;
	}
	else if (normal_axis == 2)
	{
		in_output_minmax_xyz[2] += bigestRect.x;
		in_output_minmax_xyz[3] = in_output_minmax_xyz[2] + bigestRect.width;
		//in_output_minmax_xyz[4] += bigestRect.y;
		//in_output_minmax_xyz[5] = in_output_minmax_xyz[4] + bigestRect.height;
	}
	//added by simon.jin@unre.com start
	else if (normal_axis == 3)
	{
		in_output_minmax_xyz[0] += bigestRect.x;
		in_output_minmax_xyz[1] = in_output_minmax_xyz[0] + bigestRect.width;
		in_output_minmax_xyz[2] += bigestRect.y;
		in_output_minmax_xyz[3] = in_output_minmax_xyz[2] + bigestRect.height;
	}
	
	return true;
 }



bool MeasureBase::FindBigestValidMinMaxXYZ(const std::vector<cv::Point3f>& input_points, std::vector<array<float, 4>>& hole_min_max,
	const int normal_axis, float* in_output_minmax_xyz, const int wall_id, ObstacleInfo* obstacleInfoPtr)
{

	if (input_points.size() == 0)
	{
		std::cout << "FindBigestValidMinMaxXYZ: empty points are input!" << std::endl;
		log_error("FindBigestValidMinMaxXYZ: empty points are input!");
		return false;
	}

	cv::Mat plane_pts_mat(input_points.size(), 3, CV_64F);

	for (int i = 0; i < input_points.size(); i++) {
		plane_pts_mat.at<double>(i, 0) = input_points[i].x;
		plane_pts_mat.at<double>(i, 1) = input_points[i].y;
		plane_pts_mat.at<double>(i, 2) = input_points[i].z;
	}

	//std::cout << "wall_id(FindBigestValidMinMaxXYZ):" << wall_id << std::endl;
	if (obstacleInfoPtr != nullptr && typeid(*(obstacleInfoPtr->getClassType())) == typeid(WallObstacleInfo)) {
		WallObstacleInfo* wallObstacleInfo = static_cast<WallObstacleInfo*>(obstacleInfoPtr);
		// remove beam
		if (wallObstacleInfo->beamInfo.beamMap.count(wall_id) > 0) {
			float beamHeight = std::get<0>(wallObstacleInfo->beamInfo.beamMap[wall_id]);
			//std::cout << "beamHeight" << beamHeight << std::endl;
			in_output_minmax_xyz[5] -= beamHeight;
		}
		// remove baseboard
		if (wallObstacleInfo->baseBoardInfo.baseBoardMap.count(wall_id) > 0) {
			float baseBoardHeight = wallObstacleInfo->baseBoardInfo.baseBoardMap[wall_id];
			//std::cout << "baseBoardHeight" << baseBoardHeight << std::endl;
			in_output_minmax_xyz[4] += baseBoardHeight;
		}
	}

	/*********** project 3D plane to 2D image **********/
	unsigned int image_rows;// = max_y - min_y;
	unsigned int image_cols;// = max_x - min_x;
	double min_y = in_output_minmax_xyz[4];
	double max_y = in_output_minmax_xyz[5];
	double min_x = in_output_minmax_xyz[0];
	double max_x = in_output_minmax_xyz[1];

	int    x_idx = 0;
	int    y_idx = 1;
	if (normal_axis == 1)
	{
		image_rows = in_output_minmax_xyz[5] - in_output_minmax_xyz[4];
		image_cols = in_output_minmax_xyz[1] - in_output_minmax_xyz[0];

		double min_y = in_output_minmax_xyz[4];
		double min_x = in_output_minmax_xyz[0];
		x_idx = 0;
		y_idx = 2;
	}
	else if (normal_axis == 2)
	{
		image_rows = in_output_minmax_xyz[5] - in_output_minmax_xyz[4];
		image_cols = in_output_minmax_xyz[3] - in_output_minmax_xyz[2];

		min_y = in_output_minmax_xyz[4];
		max_y = in_output_minmax_xyz[5];
		min_x = in_output_minmax_xyz[2];
		max_x = in_output_minmax_xyz[3];
		x_idx = 1;
		y_idx = 2;
	}
	else if (normal_axis == 3)
	{
		image_cols = in_output_minmax_xyz[1] - in_output_minmax_xyz[0];
		image_rows = in_output_minmax_xyz[3] - in_output_minmax_xyz[2];

		min_y = in_output_minmax_xyz[2];
		max_y = in_output_minmax_xyz[3];
		min_x = in_output_minmax_xyz[0];
		max_x = in_output_minmax_xyz[1];
		x_idx = 0;
		y_idx = 1;
	}
	unsigned int image_row, image_col;
	cv::Mat projected_2Dimage = cv::Mat::zeros(image_rows + 1, image_cols + 1, CV_8U); //black background
	for (int i = 0; i < plane_pts_mat.rows; i++) {
		if (plane_pts_mat.at<double>(i, x_idx) >= min_x && plane_pts_mat.at<double>(i, x_idx) <= max_x &&
			plane_pts_mat.at<double>(i, y_idx) >= min_y && plane_pts_mat.at<double>(i, y_idx) <= max_y)
		{
			image_row = plane_pts_mat.at<double>(i, y_idx) - min_y;
			image_col = plane_pts_mat.at<double>(i, x_idx) - min_x;
			projected_2Dimage.at<uchar>(image_row, image_col) = 255; //white pixel
		}
	}


	for (int i = 0; i < hole_min_max.size(); i++)
	{
		if (abs(hole_min_max[i][0] - min_x) < 200 || abs(hole_min_max[i][1] - max_x) < 200 || abs(hole_min_max[i][0] - max_x) < 200)
		{
			continue;
		}
		int px = hole_min_max[i][0] - min_x - 200;
		int dx = hole_min_max[i][1] - min_x + 200;
		if (px < 0)
			px = 0;

		dx -= px;
		int py = hole_min_max[i][2] - min_y;
		int dy = hole_min_max[i][3] - min_y;
		if (py < 0)
		{
			py = 0;
		}

		dy -= py;
		cv::Rect holeRect(px, py, dx, dy);
		//std::cout << px << " " << py << "w=" << dx << "h=" << dy << std::endl;
		cv::rectangle(projected_2Dimage, holeRect, cv::Scalar(255), -1);
		//if(g_vflag)
		//cv::imshow("projected_img_small1111", projected_2Dimage);
	}


	/************ dilate & erode projected 2D image **********/
	cv::Mat projected_2Dimage_dilated;
	cv::dilate(projected_2Dimage, projected_2Dimage_dilated, cv::Mat::ones(25, 25, CV_8U));

	cv::Mat projected_2Dimage_eroded;
	cv::erode(projected_2Dimage_dilated, projected_2Dimage_eroded, cv::Mat::ones(25, 25, CV_8U));

	//SHOW
	//cv::namedWindow("peojected", cv::WINDOW_NORMAL);
	//cv::namedWindow("dilated", cv::WINDOW_NORMAL);
	//cv::namedWindow("eroded", cv::WINDOW_NORMAL);
	//cv::imshow("peojected", projected_2Dimage);
	//cv::imshow("dilated", projected_2Dimage_dilated);
	//cv::imshow("eroded", projected_2Dimage_eroded);
	//cv::waitKey(0);

	/************ find external contour *******/
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(projected_2Dimage_eroded, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	if (contours.size() == 0)
	{
		std::cout << "FilterPlaneByEdgeThickness: No enough points!" << std::endl;
		log_error("FilterPlaneByEdgeThickness: No enough points!");
		return false;
	}

	cv::Mat non_edge_rotated_2D_XY_bigest_plane_ROI;

	cv::Mat projected_img_small;
	cv::resize(projected_2Dimage_eroded, projected_img_small, cv::Size(projected_2Dimage_eroded.cols * 0.125, projected_2Dimage_eroded.rows * 0.125));

	std::vector<std::vector<cv::Point>> contours_small;
	std::vector<std::vector<cv::Point>> contours_draw;
	cv::findContours(projected_img_small, contours_small, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	//cv::imshow("projected_img_small", projected_img_small);
	int maxid = 0;
	for (int i = 0; i < contours_small.size(); i++)
	{
		if (contours_small[i].size() > contours_small[maxid].size())
			maxid = i;
	}

	contours_draw.push_back(contours_small[maxid]);
	projected_img_small = 0;
	cv::drawContours(projected_img_small, contours_draw, -1, cv::Scalar(255), -1);
	//cv::imshow("projected_img_small", projected_img_small);

	cv::Rect bigestRect;// = InSquare(projected_img_small, center);
	GetLargestRect(projected_img_small, bigestRect);
	cv::rectangle(projected_img_small, bigestRect, cv::Scalar(0), -1);
	//cv::imshow("bigest_rect_projected_img_small", projected_img_small);
	//cv::waitKey(0);
	//bigestRect.x += 1;
	//bigestRect.y += 1;
	bigestRect.x *= 8;
	bigestRect.y *= 8;
	bigestRect.width *= 8;
	bigestRect.height *= 8;

	int ignore_pix = 100;
#ifdef FACTORY_MODE
	ignore_pix = 0;
#endif

	int distance_right = (projected_img_small.cols * 8 - (bigestRect.width + bigestRect.x));
	int distance_bottom = (projected_img_small.rows * 8 - (bigestRect.height + bigestRect.y));
	if (normal_axis == 1)
	{
		//if(bigestRect.x >= ignore_pix)
		//	in_output_minmax_xyz[0] += bigestRect.x;
		//if (distance_right  >= ignore_pix)
		//	in_output_minmax_xyz[1] -= distance_right;

		in_output_minmax_xyz[0] += bigestRect.x;
		in_output_minmax_xyz[1] = in_output_minmax_xyz[0] + bigestRect.width;
		in_output_minmax_xyz[5] -= bigestRect.y;
		in_output_minmax_xyz[4] = in_output_minmax_xyz[5] - bigestRect.height;

	}
	else if (normal_axis == 2)
	{
		if (bigestRect.x >= ignore_pix)
			in_output_minmax_xyz[2] += bigestRect.x;
		if (distance_right >= ignore_pix)
			in_output_minmax_xyz[3] -= distance_right;
	}
	//added by simon.jin@unre.com start
	else if (normal_axis == 3)
	{
		if (bigestRect.x >= ignore_pix)
			in_output_minmax_xyz[0] += bigestRect.x;
		if (distance_right  >= ignore_pix)
			in_output_minmax_xyz[1] -= distance_right;
		if (bigestRect.y >= ignore_pix)
			in_output_minmax_xyz[2] += bigestRect.y;
		if (distance_bottom >= ignore_pix)
			in_output_minmax_xyz[3] -= distance_bottom;
	}
	//added by simon.jin@unre.com end
	// DEBUG: show bigestRect or points
	//if (normal_axis == 1) {
	//	Eigen::ArrayXf xList(2);
	//	/*xList(0) = in_output_minmax_xyz[0];
	//	xList(1) = in_output_minmax_xyz[1];*/
	//	xList(0) = beamInfo[0];
	//	xList(1) = beamInfo[0]-beamInfo[2];
	//	Eigen::ArrayXf zList(2);
	//	/*zList(0) = in_output_minmax_xyz[4];
	//	zList(1) = in_output_minmax_xyz[5];*/
	//	zList(0) = beamInfo[1];
	//	zList(1) = beamInfo[1]+beamInfo[3];

	//	raster_XZ_img(input_points, xList, zList);
	//}

	return true;
}
bool MeasureBase::FindPlaneMinMaxXYZ(const std::vector<cv::Point3f>& input_points,
	const int normal_axis,
	const float length_plane_threshold1,
	const float length_plane_threshold2,
	float* output_minmax_xyz)
{
	if (input_points.empty())
		return false;

	if (!FindPointsMinMaxXYZ(input_points, output_minmax_xyz))
		return false;


	if (normal_axis == 1) //for vertical planes along y-axis
	{
		if (output_minmax_xyz[1] - output_minmax_xyz[0] < length_plane_threshold1 || output_minmax_xyz[5] - output_minmax_xyz[4] < length_plane_threshold2)
		{
#ifdef DEVELOPER_MODE
			std::cout << "plane not available along y-axis: " << output_minmax_xyz[1] - output_minmax_xyz[0] << "\t" << output_minmax_xyz[5] - output_minmax_xyz[4] << std::endl;
#else
			log_error("plane not available along y-axis: %f, %f", output_minmax_xyz[1] - output_minmax_xyz[0], output_minmax_xyz[5] - output_minmax_xyz[4]);
#endif
			return false;
		}
	}
	else if (normal_axis == 2) // for top/bottom planes
	{
		if (output_minmax_xyz[1] - output_minmax_xyz[0] < length_plane_threshold1 || output_minmax_xyz[3] - output_minmax_xyz[2] < length_plane_threshold2)
		{
#ifdef DEVELOPER_MODE
			std::cout << "plane not available top/bottom: " << output_minmax_xyz[1] - output_minmax_xyz[0] << "\t" << output_minmax_xyz[3] - output_minmax_xyz[2] << std::endl;
#else
			log_error("plane not available top/bottom: %f, %f", output_minmax_xyz[1] - output_minmax_xyz[0], output_minmax_xyz[3] - output_minmax_xyz[2]);
#endif
			return false;
		}
	}
	else if (normal_axis == 0)//for vertical planes, normal_axis == 0 along x-axis
	{
		if (output_minmax_xyz[3] - output_minmax_xyz[2] < length_plane_threshold1 || output_minmax_xyz[5] - output_minmax_xyz[4] < length_plane_threshold2)
		{
#ifdef DEVELOPER_MODE
			std::cout << "plane not available along x-axis: " << output_minmax_xyz[3] - output_minmax_xyz[2] << "\t" << output_minmax_xyz[5] - output_minmax_xyz[4] << std::endl;
#else
			log_error("plane not available along x-axis: %f, %f", output_minmax_xyz[3] - output_minmax_xyz[2], output_minmax_xyz[5] - output_minmax_xyz[4]);
#endif
			return false;
		}
	}
	return true;
}

bool MeasureBase::FindPlaneMinMaxXYZ2(const std::vector<cv::Point3f>& input_points,
	const int normal_axis,
	const float length_plane_threshold1,
	const float length_plane_threshold2,
	float* output_minmax_xyz)
{
	if (input_points.empty())
		return false;

	if (!FindPointsMinMaxXYZ(input_points, output_minmax_xyz))
		return false;

	if (normal_axis == 2)
	{
		if (output_minmax_xyz[1] - output_minmax_xyz[0] < length_plane_threshold1 || output_minmax_xyz[3] - output_minmax_xyz[2] < length_plane_threshold2)
		{
#ifdef DEVELOPER_MODE
			std::cout << "plane not available top/bottom: " << output_minmax_xyz[1] - output_minmax_xyz[0] << "\t" << output_minmax_xyz[3] - output_minmax_xyz[2] << std::endl;
#else
			log_error("plane not available top/bottom: %f, %f", output_minmax_xyz[1] - output_minmax_xyz[0], output_minmax_xyz[3] - output_minmax_xyz[2]);
#endif
			return false;
		}
	}
	else
	{
		float plane_width_xy = std::sqrt(std::pow(output_minmax_xyz[1] - output_minmax_xyz[0], 2.f) + std::pow(output_minmax_xyz[3] - output_minmax_xyz[2], 2.f));
		if (plane_width_xy < length_plane_threshold1 || output_minmax_xyz[5] - output_minmax_xyz[4] < length_plane_threshold2)
		{
#ifdef DEVELOPER_MODE
			std::cout << "plane not available xy: " << plane_width_xy << "\t" << output_minmax_xyz[5] - output_minmax_xyz[4] << std::endl;
#else
			log_error("plane not available xy: %f, %f", plane_width_xy, output_minmax_xyz[5] - output_minmax_xyz[4]);
#endif
			return false;
		}
	}

	return true;
}

bool MeasureBase::FindPointsMinMaxXYZ(const std::vector<cv::Point3f>& points, float* minmax_xyz)
{
	if (points.empty())
		return false;

	//float output_minmax_xyz[6];
	minmax_xyz[0] = std::numeric_limits<float>::infinity();		minmax_xyz[1] = -std::numeric_limits<float>::infinity();
	minmax_xyz[2] = std::numeric_limits<float>::infinity();		minmax_xyz[3] = -std::numeric_limits<float>::infinity();
	minmax_xyz[4] = std::numeric_limits<float>::infinity();		minmax_xyz[5] = -std::numeric_limits<float>::infinity();

	for (int i = 0; i < points.size(); i++)
	{
		minmax_xyz[0] = (points[i].x < minmax_xyz[0]) ? points[i].x : minmax_xyz[0];
		minmax_xyz[1] = (points[i].x > minmax_xyz[1]) ? points[i].x : minmax_xyz[1];

		minmax_xyz[2] = (points[i].y < minmax_xyz[2]) ? points[i].y : minmax_xyz[2];
		minmax_xyz[3] = (points[i].y > minmax_xyz[3]) ? points[i].y : minmax_xyz[3];

		minmax_xyz[4] = (points[i].z < minmax_xyz[4]) ? points[i].z : minmax_xyz[4];
		minmax_xyz[5] = (points[i].z > minmax_xyz[5]) ? points[i].z : minmax_xyz[5];
	}

	return true;
}

int MeasureBase::FindMaxPlaneInHolePlane(const std::vector<std::vector<cv::Point3f>>& pos_hole, AXIS_DIRECTION_XYZ axis, float* data_minmax_xyz)
{
	if (pos_hole.empty())
		return 0;

	//cut plane along x-axis
	std::vector<array<float, 2>> hole_min_max(pos_hole.size());
	cv::Mat center(pos_hole.size(), 1, CV_32F);
	cv::Mat indx_hole_min_max(pos_hole.size(), 1, CV_32S);
	for (int i = 0; i < hole_min_max.size(); i++)
	{
		float minval = (axis == AXIS_DIRECTION_X) ? pos_hole[i][0].x : pos_hole[i][0].y;
		float maxval = (axis == AXIS_DIRECTION_X) ? pos_hole[i][2].x : pos_hole[i][2].y;

		center.at<float>(i, 0) = (minval + maxval) * 0.5f;
		hole_min_max[i][0] = min(minval, maxval);
		hole_min_max[i][1] = max(minval, maxval);
		/*hole_min_max[i][2] = min(pos_hole[i][0].z, pos_hole[i][2].z);
		hole_min_max[i][3] = max(pos_hole[i][0].z, pos_hole[i][2].z);*/
		/*if((data_minmax_xyz[0]+ data_minmax_xyz[1])*0.5f>= (hole_min_max[i][0] - ruler_size[2]*0.5f) &&
		(data_minmax_xyz[0] + data_minmax_xyz[1])*0.5f <= (hole_min_max[i][1] + ruler_size[2] * 0.5f) )
		temp_flag_ruler3 = false;*/
	}
	//get indx of hole_min_max from l.h.s to r.h.s
	cv::sortIdx(center, indx_hole_min_max, CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);

	//find a part of wall with max length
	array<float, 2> minmax_max_length_part_wall;
	minmax_max_length_part_wall[0] = (axis == AXIS_DIRECTION_X) ? data_minmax_xyz[0] : data_minmax_xyz[2];
	minmax_max_length_part_wall[1] = hole_min_max[indx_hole_min_max.at<int>(0, 0)][0];
	float max_length_part_wall = minmax_max_length_part_wall[1] - minmax_max_length_part_wall[0];
	for (int i = 0; i < hole_min_max.size() - 1; i++)
	{
		if (hole_min_max[indx_hole_min_max.at<int>(i + 1, 0)][0] - hole_min_max[indx_hole_min_max.at<int>(i, 0)][1] > max_length_part_wall)
		{
			max_length_part_wall = hole_min_max[indx_hole_min_max.at<int>(i + 1, 0)][0] - hole_min_max[indx_hole_min_max.at<int>(i, 0)][1];
			minmax_max_length_part_wall[0] = hole_min_max[indx_hole_min_max.at<int>(i, 0)][1];
			minmax_max_length_part_wall[1] = hole_min_max[indx_hole_min_max.at<int>(i + 1, 0)][0];
		}
	}

	float max_value_axis = (axis == AXIS_DIRECTION_X) ? data_minmax_xyz[1] : data_minmax_xyz[3];
	if (max_value_axis - hole_min_max[indx_hole_min_max.at<int>(hole_min_max.size() - 1, 0)][1] > max_length_part_wall)
	{
		max_length_part_wall = max_value_axis - hole_min_max[indx_hole_min_max.at<int>(hole_min_max.size() - 1, 0)][1];
		minmax_max_length_part_wall[0] = hole_min_max[indx_hole_min_max.at<int>(hole_min_max.size() - 1, 0)][1];
		minmax_max_length_part_wall[1] = max_value_axis;
	}

	if (axis == AXIS_DIRECTION_X)
	{
		data_minmax_xyz[0] = minmax_max_length_part_wall[0];
		data_minmax_xyz[1] = minmax_max_length_part_wall[1];
	}
	else
	{
		data_minmax_xyz[2] = minmax_max_length_part_wall[0];
		data_minmax_xyz[3] = minmax_max_length_part_wall[1];
	}

	return 1;
}


bool MeasureBase::FindRulerBandPts(const std::vector<cv::Point3f>& input_points,
	const float ang_value,
	const float* ruler_band,
	std::vector <int>& num_data_in_rulerband)
{
	if (input_points.empty())
		return false;

	if (std::abs(ang_value) < M_PI * 0.25f)
	{
		for (int i = 0; i < input_points.size(); i++)
		{
			if (input_points[i].x >= ruler_band[0] && input_points[i].x <= ruler_band[1]
				&& input_points[i].z >= ruler_band[2] && input_points[i].z <= ruler_band[3])
			{
				num_data_in_rulerband.push_back(i);
			}
		}
	}
	else
	{
		for (int i = 0; i < input_points.size(); i++)
		{
			if (input_points[i].y >= ruler_band[0] && input_points[i].y <= ruler_band[1]
				&& input_points[i].z >= ruler_band[2] && input_points[i].z <= ruler_band[3])
			{
				num_data_in_rulerband.push_back(i);
			}
		}
	}

	return true;
}

void MeasureBase::FindVerticeXY(const float* data_minmax_xyz, const float plane_normal[3], int* vertice_indx)
{
	// find two vertices (x1,y1) & (x2,y2) of two planes on x-y plane, vertice_indx=[x1,x2,y1,y2]
	float normal_zero_dir_threshold = 0.3f;
	for (int i = 0; i < 4; i++)
		vertice_indx[i] = i;

	if (abs(plane_normal[0]) > normal_zero_dir_threshold  //plane not parallels to x-axis
		&& abs(plane_normal[1]) > normal_zero_dir_threshold)   //plane not parallels to y-axis
	{
		float temp_vector[2];
		temp_vector[0] = data_minmax_xyz[1] - data_minmax_xyz[0];
		temp_vector[1] = data_minmax_xyz[2] - data_minmax_xyz[3];

		float length = std::sqrt(std::pow(temp_vector[0], 2.f) + std::pow(temp_vector[1], 2.f));

		// line between two vertices is vertical to normal 
		if (abs(temp_vector[0] * plane_normal[0] + temp_vector[1] * plane_normal[1]) / length < normal_zero_dir_threshold)
		{
			vertice_indx[2] = 3;
			vertice_indx[3] = 2;
		}
	}
}


//Find virtual vertices for all planes
void MeasureBase::FindVertice(const int normal_axis,
	const float plane_normal[3],
	const bool ifRotate,
	const float ang_value,
	const float* data_minmax_xyz,
	std::vector<cv::Point3f>& vertice)
{
	if (normal_axis == 1 || normal_axis == 0)
	{
		float real_vertice[RULER_VERTICE_SIZE];

		if (ifRotate)
		{
			vertice[0].x = data_minmax_xyz[0] * cos(ang_value) + data_minmax_xyz[2] * sin(ang_value);
			vertice[0].y = -data_minmax_xyz[0] * sin(ang_value) + data_minmax_xyz[2] * cos(ang_value);

			vertice[1].x = data_minmax_xyz[1] * cos(ang_value) + data_minmax_xyz[2] * sin(ang_value);
			vertice[1].y = -data_minmax_xyz[1] * sin(ang_value) + data_minmax_xyz[2] * cos(ang_value);

			vertice[2].x = data_minmax_xyz[1] * cos(ang_value) + data_minmax_xyz[3] * sin(ang_value);
			vertice[2].y = -data_minmax_xyz[1] * sin(ang_value) + data_minmax_xyz[3] * cos(ang_value);

			vertice[3].x = data_minmax_xyz[0] * cos(ang_value) + data_minmax_xyz[3] * sin(ang_value);
			vertice[3].y = -data_minmax_xyz[0] * sin(ang_value) + data_minmax_xyz[3] * cos(ang_value);
		}
		else
		{
			int vertice_indx[4];
			FindVerticeXY(data_minmax_xyz, plane_normal, vertice_indx);
			real_vertice[0] = data_minmax_xyz[vertice_indx[0]];
			real_vertice[1] = data_minmax_xyz[vertice_indx[1]];
			real_vertice[2] = data_minmax_xyz[vertice_indx[2]];
			real_vertice[3] = data_minmax_xyz[vertice_indx[3]];

			vertice[0].x = real_vertice[0];
			vertice[0].y = real_vertice[2];

			vertice[1].x = real_vertice[1];
			vertice[1].y = real_vertice[2];

			vertice[2].x = real_vertice[1];
			vertice[2].y = real_vertice[3];

			vertice[3].x = real_vertice[0];
			vertice[3].y = real_vertice[3];

		}
		/*vertice[0].x = real_vertice[0];
		vertice[0].y = real_vertice[2];*/
		vertice[0].z = data_minmax_xyz[4];

		/*vertice[1].x = real_vertice[1];
		vertice[1].y = real_vertice[3];*/
		vertice[1].z = data_minmax_xyz[4];

		/*vertice[2].x = real_vertice[1];
		vertice[2].y = real_vertice[3];*/
		vertice[2].z = data_minmax_xyz[5];

		/*vertice[3].x = real_vertice[0];
		vertice[3].y = real_vertice[2];*/
		vertice[3].z = data_minmax_xyz[5];
	}
	else
	{
		if (ifRotate)
		{

			vertice[0].x = data_minmax_xyz[0] * cos(ang_value) + data_minmax_xyz[2] * sin(ang_value);
			vertice[0].y = -data_minmax_xyz[0] * sin(ang_value) + data_minmax_xyz[2] * cos(ang_value);

			vertice[1].x = data_minmax_xyz[1] * cos(ang_value) + data_minmax_xyz[2] * sin(ang_value);
			vertice[1].y = -data_minmax_xyz[1] * sin(ang_value) + data_minmax_xyz[2] * cos(ang_value);

			vertice[2].x = data_minmax_xyz[1] * cos(ang_value) + data_minmax_xyz[3] * sin(ang_value);
			vertice[2].y = -data_minmax_xyz[1] * sin(ang_value) + data_minmax_xyz[3] * cos(ang_value);

			vertice[3].x = data_minmax_xyz[0] * cos(ang_value) + data_minmax_xyz[3] * sin(ang_value);
			vertice[3].y = -data_minmax_xyz[0] * sin(ang_value) + data_minmax_xyz[3] * cos(ang_value);

		}
		else
		{
			vertice[0].x = data_minmax_xyz[0]; vertice[0].y = data_minmax_xyz[2];
			vertice[1].x = data_minmax_xyz[1]; vertice[1].y = data_minmax_xyz[2];
			vertice[2].x = data_minmax_xyz[1]; vertice[2].y = data_minmax_xyz[3];
			vertice[3].x = data_minmax_xyz[0]; vertice[3].y = data_minmax_xyz[3];
		}

		vertice[0].z = (data_minmax_xyz[4] + data_minmax_xyz[5]) * 0.5f;
		vertice[1].z = vertice[0].z;
		vertice[3].z = vertice[0].z;
		vertice[2].z = vertice[0].z;
	}
}
void MeasureBase::ReArrangeVerticeWithCompass(float CompassAng, std::vector<cv::Point3f>& vertice, std::vector<cv::Point3f>& verticeRot)
{
	float MaxDelt = -INFINITY;
	float MinDelt = INFINITY;
	int MaxDeltIdx = -1;
	int MinDeltIdx = -1;
	float delt = 0;
	float Ruler2XAxisPAng = INFINITY;
	int RulerNorthIdx = -1;
	float CompassNorthAng = CompassAng;
	if (CompassNorthAng < 0)
		CompassNorthAng += 360;
	std::vector<cv::Point2f> MarkPoint;
	cv::Point2f P0, P1, P2, P3;
	float a = 0;
	float b = -1;
	float c = 0;
	if (vertice[0].x == vertice[1].x) {
		P0.x = vertice[0].x;
		P0.y = 0;
	}
	else {
		a = (vertice[0].y - vertice[1].y) / (vertice[0].x - vertice[1].x);
		c = vertice[0].y - a* vertice[0].x;
		P0.x = (-a*c) / (a*a + b*b);
		P0.y = (-b*c) / (a*a + b*b);
	}
	MarkPoint.push_back(P0);
	if (vertice[1].x == vertice[2].x) {
		P1.x = vertice[1].x;
		P1.y = 0;
	}
	else {
		a = (vertice[1].y - vertice[2].y) / (vertice[1].x - vertice[2].x);
		c = vertice[1].y - a* vertice[1].x;
		P1.x = (-a*c) / (a*a + b*b);
		P1.y = (-b*c) / (a*a + b*b);
	}
	MarkPoint.push_back(P1);

	if (vertice[2].x == vertice[3].x) {
		P2.x = vertice[2].x;
		P2.y = 0;
	}
	else{
		a = (vertice[2].y - vertice[3].y) / (vertice[2].x - vertice[3].x);
		c = vertice[2].y - a*vertice[2].x;
		P2.x = (-a*c) / (a*a + b*b);
		P2.y = (-b*c) / (a*a + b*b);
	}
	MarkPoint.push_back(P2);
	if (vertice[3].x == vertice[0].x) {
		P3.x = vertice[3].x;
		P3.y = 0;
	}
	else {
		a = (vertice[3].y - vertice[0].y) / (vertice[3].x - vertice[0].x);
		c = vertice[3].y - a*vertice[3].x;
		P3.x = (-a*c) / (a*a + b*b);
		P3.y = (-b*c) / (a*a + b*b);
	}
	MarkPoint.push_back(P3);
	for (int i = 0; i < MarkPoint.size(); i++) {
		Ruler2XAxisPAng = -atan2(MarkPoint[i].y, MarkPoint[i].x) * 180 / M_PI;
		if (Ruler2XAxisPAng < 0)
			Ruler2XAxisPAng += 360;

		//	cout <<i<<": CompassNorthAng:"<< CompassNorthAng << " Ruler2XAxisPAng:"<< Ruler2XAxisPAng << endl;
		delt = abs(Ruler2XAxisPAng - CompassNorthAng);
		if (MaxDelt < delt) {
			MaxDelt = delt;
			MaxDeltIdx = i;
		}
		if (MinDelt > delt) {
			MinDelt = delt;
			MinDeltIdx = i;
		}
	}
	if ((360 - MaxDelt) < MinDelt) {
		RulerNorthIdx = MaxDeltIdx;
	}
	else {
		RulerNorthIdx = MinDeltIdx;
	}
	if (RulerNorthIdx == 0) {
		//cout << "RulerNorthIdx == 0" << endl;
		verticeRot.push_back(vertice[2]);
		verticeRot.push_back(vertice[3]);
		verticeRot.push_back(vertice[0]);
		verticeRot.push_back(vertice[1]);
	}
	if (RulerNorthIdx == 1) {
		//cout << "RulerNorthIdx == 1" << endl;
		verticeRot.push_back(vertice[3]);
		verticeRot.push_back(vertice[0]);
		verticeRot.push_back(vertice[1]);
		verticeRot.push_back(vertice[2]);
	}
	if (RulerNorthIdx == 2) {
		//cout << "RulerNorthIdx == 2" << endl;
		verticeRot.push_back(vertice[0]);
		verticeRot.push_back(vertice[1]);
		verticeRot.push_back(vertice[2]);
		verticeRot.push_back(vertice[3]);
	}
	if (RulerNorthIdx == 3) {
		//cout << "RulerNorthIdx == 3" << endl;
		verticeRot.push_back(vertice[1]);
		verticeRot.push_back(vertice[2]);
		verticeRot.push_back(vertice[3]);
		verticeRot.push_back(vertice[0]);
	}
}


int MeasureBase::FindCornerPts(const std::vector<cv::Point3f>& plane_points,
	const float* virtual_corner,
	const  cv::Mat rotationMatrix,
	float* cornerpt_z)
{
	float voxel_size = 100.f;
	//normal_z: [-1] means ceiling; [1] means ground
	if (plane_points.empty())
		return 0;

	//float temp_dist;
	int num_pts = 0;
	*cornerpt_z = 0;

	for (int i = 0; i < plane_points.size(); i++)
	{
		if ((plane_points[i].x >= virtual_corner[0] - voxel_size * 0.5f)
			&& (plane_points[i].x <= virtual_corner[0] + voxel_size * 0.5f)
			&& (plane_points[i].y >= virtual_corner[1] - voxel_size * 0.5f)
			&& (plane_points[i].y <= virtual_corner[1] + voxel_size * 0.5f))
		{
			*cornerpt_z += rotationMatrix.at<float>(2, 0) * plane_points[i].x
				+ rotationMatrix.at<float>(2, 1) * plane_points[i].y
				+ rotationMatrix.at<float>(2, 2) * plane_points[i].z;

			num_pts++;
		}
	}

	*cornerpt_z /= num_pts;
	return num_pts;
}


int MeasureBase::FindCornerPts(const std::vector<cv::Point3f>& plane_points,
	const float* virtual_corner,
	const float measure_ROI_length,
	const cv::Mat rotation_matrix,
	float* cornerpt_z)
{
	//normal_z: [-1] means ceiling; [1] means ground
	if (plane_points.empty())
		return 0;

	//float temp_dist;
	int num_pts = 0;
	*cornerpt_z = 0.f;

	for (int i = 0; i < plane_points.size(); i++)
	{
		if (std::abs(plane_points[i].x - virtual_corner[0]) < measure_ROI_length * 0.5f
			&& std::abs(plane_points[i].y - virtual_corner[1]) < measure_ROI_length * 0.5f)
		{
			*cornerpt_z += rotation_matrix.at<float>(2, 0) * plane_points[i].x
				+ rotation_matrix.at<float>(2, 1) * plane_points[i].y
				+ rotation_matrix.at<float>(2, 2) * plane_points[i].z;

			num_pts++;
		}
	}
	if (!MeasureBase::IsValZero(num_pts))
		*cornerpt_z /= (float)num_pts;

	return num_pts;
}

int MeasureBase::FindCornerPts(const std::vector<cv::Point3f>& plane_points,
	const float* virtual_corner,
	const float measure_ROI_length,
	const float offset_ROI_actualPts,
	unsigned int min_pt_num_in_quarter,
	const cv::Mat rotation_matrix,
	float* cornerpt_z)
{
	//normal_z: [-1] means ceiling; [1] means ground
	if (plane_points.empty())
		return 0;

	double z_total = 0;
	//float temp_dist;
	*cornerpt_z = 0.f;

	//divide measure_ROI into quarters, and check whether each quarter has fully-distributed points
	float max_delta_x_1, max_delta_y_1;
	max_delta_x_1 = max_delta_y_1 = -std::numeric_limits<float>::infinity();
	float max_delta_x_2, max_delta_y_2;
	max_delta_x_2 = max_delta_y_2 = -std::numeric_limits<float>::infinity();
	float max_delta_x_3, max_delta_y_3;
	max_delta_x_3 = max_delta_y_3 = -std::numeric_limits<float>::infinity();
	float max_delta_x_4, max_delta_y_4;
	max_delta_x_4 = max_delta_y_4 = -std::numeric_limits<float>::infinity();
	float delta_x, delta_y;
	float half_measure_ROI_length = measure_ROI_length * 0.5f;

	unsigned int pt_num_1 = 0;
	unsigned int pt_num_2 = 0;
	unsigned int pt_num_3 = 0;
	unsigned int pt_num_4 = 0;

	for (int i = 0; i < plane_points.size(); i++)
	{
		delta_x = plane_points[i].x - virtual_corner[0];
		delta_y = plane_points[i].y - virtual_corner[1];
		if (std::fabs(delta_x) <= half_measure_ROI_length && std::fabs(delta_y) <= half_measure_ROI_length)
		{
			/* removed by simon.jin@unre.com. why 
			*cornerpt_z += rotation_matrix.at<float>(2, 0) * plane_points[i].x
				+ rotation_matrix.at<float>(2, 1) * plane_points[i].y
				+ rotation_matrix.at<float>(2, 2) * plane_points[i].z;
			*/
			//added by simon.jin@unre.com
			//*cornerpt_z += plane_points[i].z;
			z_total += plane_points[i].z;
			if (delta_x >= 0.f && delta_x <= half_measure_ROI_length && delta_y >= 0.f && delta_y <= half_measure_ROI_length)
			{
				if (delta_x > max_delta_x_1)	max_delta_x_1 = delta_x;
				if (delta_y > max_delta_y_1)	max_delta_y_1 = delta_y;
				pt_num_1++;
			}
			else if (delta_x < 0.f && delta_x >= -half_measure_ROI_length && delta_y >= 0.f && delta_y <= half_measure_ROI_length)
			{
				if (-delta_x > max_delta_x_2)	max_delta_x_2 = -delta_x;
				if (delta_y > max_delta_y_2)	max_delta_y_2 = delta_y;
				pt_num_2++;
			}
			else if (delta_x < 0.f && delta_x >= -half_measure_ROI_length && delta_y < 0.f && delta_y >= -half_measure_ROI_length)
			{
				if (-delta_x > max_delta_x_3)	max_delta_x_3 = -delta_x;
				if (-delta_y > max_delta_y_3)	max_delta_y_3 = -delta_y;
				pt_num_3++;
			}
			else if (delta_x >= 0.f && delta_x <= half_measure_ROI_length && delta_y < 0.f && delta_y >= -half_measure_ROI_length)
			{
				if (delta_x > max_delta_x_4)	max_delta_x_4 = delta_x;
				if (-delta_y > max_delta_y_4)	max_delta_y_4 = -delta_y;
				pt_num_4++;
			}
		}
	}

	//ensure the actual points are fully distrubited in ROI
	float offset_margin = half_measure_ROI_length - offset_ROI_actualPts;
	if (max_delta_x_1 < offset_margin || max_delta_y_1 < offset_margin || pt_num_1 < min_pt_num_in_quarter
		|| max_delta_x_2 < offset_margin || max_delta_y_2 < offset_margin || pt_num_2 < min_pt_num_in_quarter
		|| max_delta_x_3 < offset_margin || max_delta_y_3 < offset_margin || pt_num_3 < min_pt_num_in_quarter
		|| max_delta_x_4 < offset_margin || max_delta_y_4 < offset_margin || pt_num_4 < min_pt_num_in_quarter)
		return 0;

	float num_pts = pt_num_1 + pt_num_2 + pt_num_3 + pt_num_4;
	if (!MeasureBase::IsValZero(num_pts))
		*cornerpt_z = z_total / (double)num_pts;
	return num_pts;
}


void MeasureBase::NormalizeVector(const float* vector, float* normalized_vector)
{
	float inv_norm = 1.f / std::sqrtf(std::pow(vector[0], 2.f) + std::pow(vector[1], 2.f) + std::pow(vector[2], 2.f));
	for (int i = 0; i < 3; i++)
		normalized_vector[i] = vector[i] * inv_norm;
}

void MeasureBase::AlignVectorSgn(const float* vector, float* aligned_vector)
{
	if (vector[0] >= 0.f)
		aligned_vector[0] = std::fabs(aligned_vector[0]);
	else
		aligned_vector[0] = -std::fabs(aligned_vector[0]);

	if (vector[1] >= 0.f)
		aligned_vector[1] = std::fabs(aligned_vector[1]);
	else
		aligned_vector[1] = -std::fabs(aligned_vector[1]);

	if (vector[2] >= 0.f)
		aligned_vector[2] = std::fabs(aligned_vector[2]);
	else
		aligned_vector[2] = -std::fabs(aligned_vector[2]);
}

void MeasureBase::UniformNormals(const float* normal, const float* center, float* uniform_normal)
{
	if (MathOperation::ComputeVectorDotProduct(normal, center) > 0.f)
	{
		uniform_normal[0] = -normal[0];
		uniform_normal[1] = -normal[1];
		uniform_normal[2] = -normal[2];
	}
	else
	{
		uniform_normal[0] = normal[0];
		uniform_normal[1] = normal[1];
		uniform_normal[2] = normal[2];
	}
}

void MeasureBase::CrossProduct(const float* a, const float* b, float* c)
{
	c[0] = a[1] * b[2] - a[2] * b[1];
	c[1] = a[2] * b[0] - a[0] * b[2];
	c[2] = a[0] * b[1] - a[1] * b[0];
}

double MeasureBase::DotProduct(const float a[3], const float b[3])
{
	double result;
	result = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

	return result;
}

void MeasureBase::DotProduct(const float* a, const float* b, float* value)
{
	*value = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

cv::Mat MeasureBase::CalRotationMatrixFromVectors(const float* vector_before, const float* vector_after)
{
	// Normalize
	float vector_before_normalized[3];
	float vector_after_normalized[3];

	NormalizeVector(vector_before, &vector_before_normalized[0]);
	NormalizeVector(vector_after, &vector_after_normalized[0]);

	//rotationAxis by cross product
	float u[3];
	CrossProduct(vector_before_normalized, vector_after_normalized, &u[0]);

	//rotationAngle
	float angle;
	angle = std::acosf(DotProduct(vector_before_normalized, vector_after_normalized));// / Normalize(vectorBefore) / Normalize(vectorAfter)

	float norm = std::sqrtf(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);

	if (norm > 0.00001f)
	{
		float inv_norm = 1.f / norm;
		u[0] = u[0] * inv_norm;
		u[1] = u[1] * inv_norm;
		u[2] = u[2] * inv_norm;
	}
	else {
		angle = 0.f;
	}

	cv::Mat rotationMatrix(3, 3, CV_32F);

	rotationMatrix.at<float>(0, 0) = std::cosf(angle) + u[0] * u[0] * (1 - std::cosf(angle));
	rotationMatrix.at<float>(0, 1) = u[0] * u[1] * (1 - std::cosf(angle)) - u[2] * std::sinf(angle);
	rotationMatrix.at<float>(0, 2) = u[1] * std::sinf(angle) + u[0] * u[2] * (1 - std::cosf(angle));

	rotationMatrix.at<float>(1, 0) = u[2] * std::sinf(angle) + u[0] * u[1] * (1 - std::cosf(angle));
	rotationMatrix.at<float>(1, 1) = std::cosf(angle) + u[1] * u[1] * (1 - std::cosf(angle));
	rotationMatrix.at<float>(1, 2) = -u[0] * std::sinf(angle) + u[1] * u[2] * (1 - std::cosf(angle));

	rotationMatrix.at<float>(2, 0) = -u[1] * std::sinf(angle) + u[0] * u[2] * (1 - std::cosf(angle));
	rotationMatrix.at<float>(2, 1) = u[0] * std::sinf(angle) + u[1] * u[2] * (1 - std::cosf(angle));
	rotationMatrix.at<float>(2, 2) = std::cosf(angle) + u[2] * u[2] * (1.f - std::cosf(angle));

	//std::cout << rotationMatrix << std::endl;

	return rotationMatrix;
}

void MeasureBase::CalcAngleVectorXY2YAxis(const float* vector, float* angle)
{
	float vector_xy_projection = std::sqrt(std::pow(vector[0], 2.f) + std::pow(vector[1], 2.f));
	float crosspro[3];
	CrossProduct(vector, AXIS_Y_DIRECTION, &crosspro[0]);
	*angle = (crosspro[2] > 0.f) ? acosf(vector[1] / vector_xy_projection) : -acosf(vector[1] / vector_xy_projection);
}

void MeasureBase::CalcAngleVectorXY2XAxis(const float* vector, float* angle)
{
	float vector_xy_projection = std::sqrt(std::pow(vector[0], 2.f) + std::pow(vector[1], 2.f));
	float crosspro[3];
	CrossProduct(vector, AXIS_X_DIRECTION, &crosspro[0]);
	*angle = (crosspro[2] > 0.f) ? acosf(vector[0] / vector_xy_projection) : -acosf(vector[0] / vector_xy_projection);
}

void MeasureBase::CalcAngleVectorYZ2YAxis(const float* vector, float* angle)
{
	float vector_yz_projection = std::sqrt(std::pow(vector[1], 2.f) + std::pow(vector[2], 2.f));
	float crosspro[3];
	CrossProduct(vector, AXIS_Y_DIRECTION, &crosspro[0]);
	//before fix
	//*angle = (crosspro[2]>0.f) ? acosf(vector[1] / vector_yz_projection) : -acosf(vector[1] / vector_yz_projection);
	//after fix
	*angle = (crosspro[0] > 0.f) ? acosf(vector[1] / vector_yz_projection) : -acosf(vector[1] / vector_yz_projection);

}

void MeasureBase::CalcAngleVectorYZ2ZAxis(const float* vector, float* angle)
{
	float vector_yz_projection = std::sqrt(std::pow(vector[1], 2.f) + std::pow(vector[2], 2.f));
	float crosspro[3];
	CrossProduct(vector, AXIS_Z_DIRECTION, &crosspro[0]);
	*angle = (crosspro[2] > 0.f) ? acosf(vector[2] / vector_yz_projection) : -acosf(vector[2] / vector_yz_projection);
}

void MeasureBase::CalcAngleVectorXZ2XAxis(const float* vector, float* angle)
{
	float vector_xz_projection = std::sqrt(std::pow(vector[0], 2.f) + std::pow(vector[2], 2.f));
	float crosspro[3];
	CrossProduct(vector, AXIS_X_DIRECTION, &crosspro[0]);
	*angle = (crosspro[2] > 0.f) ? acosf(vector[0] / vector_xz_projection) : -acosf(vector[0] / vector_xz_projection);
}

void MeasureBase::CalcAngleVectorXZ2ZAxis(const float* vector, float* angle)
{
	float vector_xz_projection = std::sqrt(std::pow(vector[0], 2.f) + std::pow(vector[2], 2.f));
	float crosspro[3];
	CrossProduct(vector, AXIS_Z_DIRECTION, &crosspro[0]);
	*angle = (crosspro[2] > 0.f) ? acosf(vector[2] / vector_xz_projection) : -acosf(vector[2] / vector_xz_projection);
}

cv::Mat MeasureBase::TranslateAngleAroundX2RotationMatrix(const float angle)
{
	cv::Mat rotation_matrix = cv::Mat::zeros(3, 3, CV_32FC1);
	rotation_matrix.at<float>(0, 0) = 1.f;
	rotation_matrix.at<float>(1, 1) = cos(angle);
	rotation_matrix.at<float>(1, 2) = -sin(angle);
	rotation_matrix.at<float>(2, 1) = sin(angle);
	rotation_matrix.at<float>(2, 2) = cos(angle);

	return rotation_matrix;
}

cv::Mat MeasureBase::TranslateAngleAroundY2RotationMatrix(const float angle)
{
	cv::Mat rotation_matrix = cv::Mat::zeros(3, 3, CV_32FC1);
	rotation_matrix.at<float>(0, 0) = cos(angle);
	rotation_matrix.at<float>(0, 2) = sin(angle);
	rotation_matrix.at<float>(1, 1) = 1.f;
	rotation_matrix.at<float>(2, 0) = -sin(angle);
	rotation_matrix.at<float>(2, 2) = cos(angle);

	return rotation_matrix;
}

cv::Mat MeasureBase::TranslateAngleAroundZ2RotationMatrix(const float angle)
{
	cv::Mat rotation_matrix = cv::Mat::zeros(3, 3, CV_32FC1);
	rotation_matrix.at<float>(0, 0) = cos(angle);
	rotation_matrix.at<float>(0, 1) = -sin(angle);
	rotation_matrix.at<float>(1, 0) = sin(angle);
	rotation_matrix.at<float>(1, 1) = cos(angle);
	rotation_matrix.at<float>(2, 2) = 1.f;

	return rotation_matrix;
}

void MeasureBase::RotateVector(const float* vector, const cv::Mat rotation_matrix, float* rotated_vector)
{
	rotated_vector[0] = rotation_matrix.at<float>(0, 0) * vector[0]
		+ rotation_matrix.at<float>(0, 1) * vector[1]
		+ rotation_matrix.at<float>(0, 2) * vector[2];
	rotated_vector[1] = rotation_matrix.at<float>(1, 0) * vector[0]
		+ rotation_matrix.at<float>(1, 1) * vector[1]
		+ rotation_matrix.at<float>(1, 2) * vector[2];
	rotated_vector[2] = rotation_matrix.at<float>(2, 0) * vector[0]
		+ rotation_matrix.at<float>(2, 1) * vector[1]
		+ rotation_matrix.at<float>(2, 2) * vector[2];
}

void MeasureBase::RotatePoint(const cv::Point3f point, const cv::Mat rotation_matrix, cv::Point3f& rotated_point)
{
	rotated_point.x = point.x * rotation_matrix.at<float>(0, 0)
		+ point.y * rotation_matrix.at<float>(0, 1)
		+ point.z * rotation_matrix.at<float>(0, 2);
	rotated_point.y = point.x * rotation_matrix.at<float>(1, 0)
		+ point.y * rotation_matrix.at<float>(1, 1)
		+ point.z * rotation_matrix.at<float>(1, 2);
	rotated_point.z = point.x * rotation_matrix.at<float>(2, 0)
		+ point.y * rotation_matrix.at<float>(2, 1)
		+ point.z * rotation_matrix.at<float>(2, 2);
}

void MeasureBase::RotatePoint(const float* point, const cv::Mat rotation_matrix, float* rotatePoint)
{
	rotatePoint[0] = rotation_matrix.at<float>(0, 0) * point[0]
		+ rotation_matrix.at<float>(0, 1) * point[1]
		+ rotation_matrix.at<float>(0, 2) * point[2];
	rotatePoint[1] = rotation_matrix.at<float>(1, 0) * point[0]
		+ rotation_matrix.at<float>(1, 1) * point[1]
		+ rotation_matrix.at<float>(1, 2) * point[2];
	rotatePoint[2] = rotation_matrix.at<float>(2, 0) * point[0]
		+ rotation_matrix.at<float>(2, 1) * point[1]
		+ rotation_matrix.at<float>(2, 2) * point[2];
}

bool MeasureBase::RotatePoints(const std::vector<cv::Point3f>& points, const cv::Mat& rotation_matrix, std::vector<cv::Point3f>& rotated_points)
{
	if (points.empty()) {
		std::cerr << "RotatePoints: points is empty!" << std::endl;
		return false;
	}
	if (rotation_matrix.empty() ||
		!((rotation_matrix.rows == 3 && rotation_matrix.cols == 3) || (rotation_matrix.rows == 4 && rotation_matrix.cols == 4))) {
		std::cerr << "RotatePoints: invalid rotation matrix size!" << std::endl;
		return false;
	}

	rotated_points.resize(points.size());

	cv::Mat mat_float;
	rotation_matrix.convertTo(mat_float, CV_32F);

	if (mat_float.rows == 3 && mat_float.cols == 3)
	{
#pragma omp parallel for
		for (int i = 0; i < points.size(); i++)
		{
			const auto& p = points[i];
			rotated_points[i].x = p.x * mat_float.at<float>(0, 0) + p.y * mat_float.at<float>(0, 1) + p.z * mat_float.at<float>(0, 2);
			rotated_points[i].y = p.x * mat_float.at<float>(1, 0) + p.y * mat_float.at<float>(1, 1) + p.z * mat_float.at<float>(1, 2);
			rotated_points[i].z = p.x * mat_float.at<float>(2, 0) + p.y * mat_float.at<float>(2, 1) + p.z * mat_float.at<float>(2, 2);
		}
	}
	else if (mat_float.rows == 4 && mat_float.cols == 4)
	{
#pragma omp parallel for
		for (int i = 0; i < points.size(); i++)
		{
			const auto& p = points[i];
			float x_h = p.x * mat_float.at<float>(0, 0) + p.y * mat_float.at<float>(0, 1) + p.z * mat_float.at<float>(0, 2) + mat_float.at<float>(0, 3);
			float y_h = p.x * mat_float.at<float>(1, 0) + p.y * mat_float.at<float>(1, 1) + p.z * mat_float.at<float>(1, 2) + mat_float.at<float>(1, 3);
			float z_h = p.x * mat_float.at<float>(2, 0) + p.y * mat_float.at<float>(2, 1) + p.z * mat_float.at<float>(2, 2) + mat_float.at<float>(2, 3);
			float w_h = p.x * mat_float.at<float>(3, 0) + p.y * mat_float.at<float>(3, 1) + p.z * mat_float.at<float>(3, 2) + mat_float.at<float>(3, 3);

			if (fabs(w_h) < 1e-6) w_h = 1e-6;
			rotated_points[i].x = x_h / w_h;
			rotated_points[i].y = y_h / w_h;
			rotated_points[i].z = z_h / w_h;
		}
	}
	else {
		std::cerr << "RotatePoints: unsupported matrix size!" << std::endl;
		return false;
	}

	return true;
}


bool MeasureBase::RotatePoints(const cv::Mat& points, const cv::Mat& rotation_matrix, cv::Mat& rotated_points)
{
	if (points.empty())
		return false;

	rotated_points = points * rotation_matrix;

	return true;
}

bool MeasureBase::RotatePointsWithSequence(const std::vector<cv::Point3f>& plane_points, const std::vector<cv::Mat>& rotation_matrices, std::vector<cv::Point3f>& rotated_plane_points)
{
	if (plane_points.empty())
		return false;

	rotated_plane_points.resize(plane_points.size());
	std::vector<cv::Point3f> temp_point(rotation_matrices.size() + 1);

	for (int i = 0; i < plane_points.size(); i++)
	{
		temp_point[0] = plane_points[i];

		for (int j = 0; j < rotation_matrices.size(); j++)
		{
			temp_point[j + 1].x = temp_point[j].x * rotation_matrices[j].at<float>(0, 0) + temp_point[j].y * rotation_matrices[j].at<float>(0, 1) + temp_point[j].z * rotation_matrices[j].at<float>(0, 2);
			temp_point[j + 1].y = temp_point[j].x * rotation_matrices[j].at<float>(1, 0) + temp_point[j].y * rotation_matrices[j].at<float>(1, 1) + temp_point[j].z * rotation_matrices[j].at<float>(1, 2);
			temp_point[j + 1].z = temp_point[j].x * rotation_matrices[j].at<float>(2, 0) + temp_point[j].y * rotation_matrices[j].at<float>(2, 1) + temp_point[j].z * rotation_matrices[j].at<float>(2, 2);
		}

		rotated_plane_points[i] = temp_point[temp_point.size() - 1];
	}
	return true;
}

bool MeasureBase::RotatePointsAroundZ(const std::vector<cv::Point3f>& points, const float angle, std::vector < cv::Point3f>& rotated_points)
{
	if (points.empty())
		return false;

	rotated_points.resize(points.size());
	if (angle == 0.f)
		rotated_points = points;
	else {
		cv::Mat rotation_matrix;
		rotation_matrix = TranslateAngleAroundZ2RotationMatrix(angle);
		RotatePoints(points, rotation_matrix, rotated_points);
	}

	return true;
}

void MeasureBase::RotatePointAroundZ(cv::Point3f& point, const float angle, cv::Point3f& rotated_point)
{
	if (angle == 0.f)
		rotated_point = point;
	else {
		cv::Mat rotation_matrix;
		rotation_matrix = TranslateAngleAroundZ2RotationMatrix(angle);
		RotatePoint(point, rotation_matrix, rotated_point);
	}
}

void MeasureBase::RotateVectorAroundZ(const float* vector, const float angle, float* rotated_vector)
{
	rotated_vector[0] = vector[0] * cos(angle) - vector[1] * sin(angle);
	rotated_vector[1] = vector[0] * sin(angle) + vector[1] * cos(angle);
	rotated_vector[2] = vector[2];
}


float MeasureBase::DistTwoPlane(const cv::Point3f& plane_pointsA_center,
	const float  plane_normalA[3],
	const cv::Point3f& plane_pointsB_center,
	const float plane_normalB[3])
{
	float dot_product_value = std::abs(DotProduct(plane_normalA, plane_normalB));

	cv::Point3f center_line = plane_pointsA_center - plane_pointsB_center;

	if (IsValZero(dot_product_value - 1.f))
	{
		return (std::abs(center_line.x * plane_normalA[0] + center_line.y * plane_normalA[1] + center_line.z * plane_normalA[2]));
	}
	else {
		float dist1 = std::abs(center_line.x * plane_normalA[0] + center_line.y * plane_normalA[1] + center_line.z * plane_normalA[2]);
		float dist2 = std::abs(center_line.x * plane_normalB[0] + center_line.y * plane_normalB[1] + center_line.z * plane_normalB[2]);

		return(min(dist1, dist2));
	}
}

cv::Point MeasureBase::ConvertVoxelIDToPts(const cv::Point pts_idx, const float* min_max_xyz, const float length_of_voxel) {
	//pts_idx.x = col_idx;pts_idx.y=row_idx
	cv::Point pts;
	pts.x = min_max_xyz[0] + pts_idx.x * length_of_voxel;
	pts.y = min_max_xyz[2] + pts_idx.y * length_of_voxel;
	return(pts);
}

void MeasureBase::ConvertXYToVoxelID(const float x, const float y,
	unsigned int cols_of_voxel, unsigned int rows_of_voxel,
	const float* min_max_xyz, const float length_of_voxel,
	unsigned int& col_idx, unsigned int& row_idx) {
	float length_of_voxel_inverse = 1 / length_of_voxel;
	col_idx = std::floor((x - min_max_xyz[0]) * length_of_voxel_inverse);
	row_idx = std::floor((y - min_max_xyz[2]) * length_of_voxel_inverse);
	//unsigned long long height_idx = std::floor((z - min_z) * length_z_of_voxel_inverse);

	if (col_idx == cols_of_voxel)
		col_idx--;
	if (row_idx == rows_of_voxel)
		row_idx--;
	//if (height_idx == depths_of_voxel)
	//height_idx--;

	return;
}

void MeasureBase::Generate2DImage(const std::vector<cv::Point3f>& input_data,
	const float* min_max_xyz, const float length_of_voxel,
	cv::Mat& image) {
	if (input_data.empty()) {
		std::string err_message;
#ifdef DELIVER_TO_CLIENTS
		throw err_message = "empty input";
#else
		throw err_message = "Generate2DImage(): empty input";
#endif
	}
	unsigned int  rows_of_voxel = 0;
	unsigned int  cols_of_voxel = 0;
	float length_of_voxel_inverse = 1.f / length_of_voxel;
	if ((min_max_xyz[1] - min_max_xyz[0]) != 0 && std::remainder((min_max_xyz[1] - min_max_xyz[0]), length_of_voxel) == 0)
		cols_of_voxel = std::floor((min_max_xyz[1] - min_max_xyz[0]) * length_of_voxel_inverse);
	else
		cols_of_voxel = std::floor((min_max_xyz[1] - min_max_xyz[0]) * length_of_voxel_inverse) + 1;

	if ((min_max_xyz[3] - min_max_xyz[2]) != 0 && std::remainder((min_max_xyz[3] - min_max_xyz[2]), length_of_voxel) == 0)
		rows_of_voxel = std::floor((min_max_xyz[3] - min_max_xyz[2]) * length_of_voxel_inverse);
	else
		rows_of_voxel = std::floor((min_max_xyz[3] - min_max_xyz[2]) * length_of_voxel_inverse) + 1;

	//cout pts in voxel
	image = cv::Mat::zeros(rows_of_voxel, cols_of_voxel, CV_8U);

	unsigned int col_idx, row_idx;
	unsigned int max_num = 0;
	for (int i = 0; i < input_data.size(); i++) {
		ConvertXYToVoxelID(input_data[i].x, input_data[i].y,
			cols_of_voxel, rows_of_voxel,
			min_max_xyz, length_of_voxel,
			col_idx, row_idx);
		//image.at<uchar>(row_idx, col_idx) += 1;
		image.at<uchar>(row_idx, col_idx) = 255;
		/*std::cout << row_idx<<", "<<col_idx << "; ";
		std::cout << image.at<uchar>(row_idx, col_idx) << std::endl;*/
		//max_num = (max_num > image.at<uchar>(row_idx, col_idx)) ? max_num : image.at<uchar>(row_idx, col_idx);
	}


	//convert to image
	//float max_num_inverse = 1.f / max_num;
	//for (int i = 0; i < rows_of_voxel; i++) {
	//	for (int j = 0; j < cols_of_voxel; j++) {
	//		//std::cout << image.at<uchar>(i, j)<<", ";
	//		image.at<uchar>(i, j) = std::floor(image.at<uchar>(i, j) *max_num_inverse * 255);//
	//																						 //std::cout << image.at<uchar>(i, j) << std::endl;
	//	}
	//}


	////**by XUeyan* project 3D plane to 2D image
	//cols_of_voxel = max_y - min_y;
	//rows_of_voxel = max_x - min_x;
	//unsigned int image_row, image_col;
	//image = cv::Mat::zeros(rows_of_voxel + 1, cols_of_voxel + 1, CV_8UC1);
	//for (int i = 0; i < input_data.size(); i++) {
	////printf("%d; ", i);
	//image_col = input_data[i].x - min_x;
	//image_row = input_data[i].y - min_y;
	//image.at<uchar>(image_row, image_col) = 255;
	//}

	//std::cout << image << std::endl;
	/* imshow("Original", image);
	waitKey(0);*/
	return;
}

bool MeasureBase::IfPointInsideRect(const cv::Point3f pt, const std::vector<cv::Point3f>& plane_vertices)
{
	if (plane_vertices.size() != 4)
		return false;

	cv::Point3f pt_porjected_on_plane;

	std::vector<float>	plane_n = MathOperation::ComputeVectorCrossProduct(plane_vertices[2] - plane_vertices[1], plane_vertices[3] - plane_vertices[1]);
	//porject pt on this plane;
	float plane_normal[3]; float v = 1.f / std::sqrt(std::pow(plane_n[0], 2) + std::pow(plane_n[1], 2) + std::pow(plane_n[2], 2));
	for (unsigned int i = 0; i < 3; i++)
		plane_normal[i] = plane_n[i] * v;

	pt_porjected_on_plane = MathOperation::CalPointIntersectPlanePoint(pt,
		plane_normal,
		plane_vertices[0]);

	std::vector<float> cross_value1(3), cross_value2(3);

	cross_value1 = MathOperation::ComputeVectorCrossProduct(pt_porjected_on_plane - plane_vertices[0], pt_porjected_on_plane - plane_vertices[1]);
	cross_value2 = MathOperation::ComputeVectorCrossProduct(pt_porjected_on_plane - plane_vertices[1], pt_porjected_on_plane - plane_vertices[2]);

	//std::cout << (cross_value1[0] * cross_value2[0] + cross_value1[1] * cross_value2[1] + cross_value1[2] * cross_value2[2]) << std::endl;
	bool ind;
	if ((cross_value1[0] * cross_value2[0] + cross_value1[1] * cross_value2[1] + cross_value1[2] * cross_value2[2]) >= 0.f)
		ind = true;
	else
		ind = false;

	cross_value1 = MathOperation::ComputeVectorCrossProduct(pt_porjected_on_plane - plane_vertices[2], pt_porjected_on_plane - plane_vertices[3]);
	cross_value2 = MathOperation::ComputeVectorCrossProduct(pt_porjected_on_plane - plane_vertices[3], pt_porjected_on_plane - plane_vertices[0]);

	//std::cout << (cross_value1[0] * cross_value2[0] + cross_value1[1] * cross_value2[1] + cross_value1[2] * cross_value2[2]) << std::endl;

	if (ind &&
		(cross_value1[0] * cross_value2[0] + cross_value1[1] * cross_value2[1] + cross_value1[2] * cross_value2[2]) >= 0.f)
		return true;// totallty inside
	else
		return false;

}

bool MeasureBase::FilterHolesbyVertical(const std::vector<std::vector<cv::Point3f>>& holes, const float Z_threshold, std::vector<std::vector<cv::Point3f>>& filtered_holes)
{
	if (holes.empty())	return false;

	filtered_holes.resize(holes.size());
	int filtered_holes_counter = 0;
	bool is_remained;

	for (int i = 0; i < holes.size(); i++)
	{
		is_remained = false;
		for (int j = 0; j < holes[i].size(); j++)
		{
			if (holes[i][j].z < Z_threshold)
			{
				is_remained = true;
				//std::cout << "is remained " << std::endl;
				break;
			}
		}

		if (is_remained)
		{
			filtered_holes[filtered_holes_counter] = holes[i];
			filtered_holes_counter++;
		}
	}

	filtered_holes.resize(filtered_holes_counter);

	return true;
}

void MeasureBase::ProjectToXYMat(const std::vector<cv::Point3f>& plane_points, const cv::Point3f& plane_normal, cv::Mat& mat, cv::Mat& project_points)
{
	/******** rotate 3D plane to 2D XY plane ***********/
	cv::Mat rot_mat_z(3, 3, CV_32F), rot_mat_y1(3, 3, CV_32F), rot_mat_y2(3, 3, CV_32F);
	cv::Mat rot_mat_zy(3, 3, CV_32F), rot_mat_zy_transpose(3, 3, CV_32F);
	cv::Mat fnl_rot_mat(3, 3, CV_32F), fnl_rot_mat_transpose(3, 3, CV_32F);

	float length_xy = std::sqrt(std::pow(plane_normal.x, 2) + std::pow(plane_normal.y, 2));
	float ang_y1 = -M_PI * 0.5;
	float ang_y2 = std::atan2f(-plane_normal.z, length_xy + 1e-10);
	float ang_z = std::atan2f(plane_normal.y, plane_normal.x + 1e-10);

	rot_mat_y1 = MathOperation::CreateRotationMat(ang_y1, 1);
	rot_mat_y2 = MathOperation::CreateRotationMat(ang_y2, 1);
	rot_mat_z = MathOperation::CreateRotationMat(ang_z, 2);

	rot_mat_zy = rot_mat_z * rot_mat_y2;
	cv::transpose(rot_mat_zy, rot_mat_zy_transpose);
	fnl_rot_mat = rot_mat_y1 * rot_mat_zy_transpose;
	cv::transpose(fnl_rot_mat, fnl_rot_mat_transpose); //Rotation matrix's inverse = rotation matrix's transpose
	//for (int t = 0; t < fnl_rot_mat.rows; t++)
	//	std::cout << "111111111: " << fnl_rot_mat.at<float>(t, 0) << "\t" << fnl_rot_mat.at<float>(t, 1) << "\t" << fnl_rot_mat.at<float>(t, 2) << std::endl;
	project_points = cv::Mat::zeros(plane_points.size(), 3, CV_32F); //float
	for (int i = 0; i < project_points.rows; i++)
	{
		project_points.at<float>(i, 0) = fnl_rot_mat.at<float>(0, 0) * plane_points[i].x +
			fnl_rot_mat.at<float>(0, 1) * plane_points[i].y + fnl_rot_mat.at<float>(0, 2) * plane_points[i].z;

		project_points.at<float>(i, 1) = fnl_rot_mat.at<float>(1, 0) * plane_points[i].x +
			fnl_rot_mat.at<float>(1, 1) * plane_points[i].y + fnl_rot_mat.at<float>(1, 2) * plane_points[i].z;

		project_points.at<float>(i, 2) = fnl_rot_mat.at<float>(2, 0) * plane_points[i].x +
			fnl_rot_mat.at<float>(2, 1) * plane_points[i].y + fnl_rot_mat.at<float>(2, 2) * plane_points[i].z;
	}
	mat = fnl_rot_mat;
}

void MeasureBase::FindMinMaxForXY(const cv::Mat& plane_points_mat, float& min_x, float& max_x, float& min_y, float& max_y)
{
	/******* find min/max X & Y *********/
	min_x = std::numeric_limits<float>::infinity();
	max_x = -std::numeric_limits<float>::infinity();
	min_y = std::numeric_limits<float>::infinity();
	max_y = -std::numeric_limits<float>::infinity();
	for (int i = 0; i < plane_points_mat.rows; i++)
	{
		if (min_x > plane_points_mat.at<float>(i, 0))	min_x = plane_points_mat.at<float>(i, 0);
		if (max_x < plane_points_mat.at<float>(i, 0))	max_x = plane_points_mat.at<float>(i, 0);
		if (min_y > plane_points_mat.at<float>(i, 1))	min_y = plane_points_mat.at<float>(i, 1);
		if (max_y < plane_points_mat.at<float>(i, 1))	max_y = plane_points_mat.at<float>(i, 1);
	}
}


bool MeasureBase::SavePoints(const std::vector<cv::Point3f>& plane_points, const std::string file_name)
{
	if (plane_points.empty())
		return false;

	ofstream fout3(file_name, 'w');

	if (!fout3.good())
		return false;

	for (int i = 0; i < plane_points.size(); i++)
	{
		fout3 << plane_points[i].x << '\t';
		fout3 << plane_points[i].y << '\t';
		fout3 << plane_points[i].z << std::endl;
	}

	fout3.close();

	return true;
}

bool MeasureBase::SavePoints(const cv::Mat plane_points, const std::string file_name)
{
	if (plane_points.empty())
		return false;

	ofstream fout3(file_name, 'w');

	if (!fout3.good())
		return false;

	for (int i = 0; i < plane_points.rows; i++)
	{
		fout3 << plane_points.at<float>(i, 0) << '\t';
		fout3 << plane_points.at<float>(i, 1) << '\t';
		fout3 << plane_points.at<float>(i, 2) << std::endl;
	}

	fout3.close();

	return true;
}