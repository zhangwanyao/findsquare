#include <iostream>
#include <cmath>
#include <limits>
#include <omp.h>
#include "util_math.hpp"
#include "../../log/log.h"
#include "detc_edge2d.h"
//#include "opencv2/opencv.hpp"
//#include <opencv2/imgproc/types_c.h>
#include "ModuleStruct.hpp"
#include "util_time.hpp"
#include "util_opencv.hpp"
#include "../../IO/include/InOutData.h"
#include <bitset>
 
 
using namespace std;
using namespace Util_CV;
using namespace Util_Math;
using namespace ModuleStruct;

Detc_Edge2D::Detc_Edge2D()
	: m_ptArray(nullptr)	// init m_ptArray nullptr
	, m_voxelgrid(nullptr)	// init m_voxelgrid nullptr
	, m_pts(nullptr)
	, data_center_input(false)
{
}

Detc_Edge2D::Detc_Edge2D(Point3fArray* ptArray, const Point3f nPlane, const double voxel_length)
	: m_ptArray(nullptr)	// init m_ptArray nullptr
	, m_voxelgrid(nullptr)	// init m_voxelgrid nullptr
	, m_pts(nullptr)
	, data_center_input(false)
{
	setPtArray(ptArray);
	setNPlane(nPlane);
	initImgVoxelGrid(1, 1);
	setVoxelSize(voxel_length);
}


Detc_Edge2D::~Detc_Edge2D() {
	clear();
}

bool Detc_Edge2D::init() {
	// data validation
	edge_pts_in_projected_plane.clear();
	edge_pts_in_projected_plane.shrink_to_fit();
	if (!m_ptArray) {
		log_error("error Detc_Edge2D::init->m_ptArray nullptr\n");
		return false;
	}else {
		if (m_ptArray->size() == 0) {
			log_error("error Detc_Edge2D::init->invalid m_ptArray size\n");
			return false;
		}
	}
	if (Util_Math::vec3_is_zero(m_nPlane)) {
		log_error("error Detc_Edge2D::init->invalid m_nPlane\n");
		return false;
	}

	min_point_num_of_contour = 20;
	/*------------------------------------------------*/

	// copy point positions to array for futher process
	// and calculate center of point-set
	Point3f* pts; // array of point positions
	int nPts = static_cast<int>(m_ptArray->size());
	pts = new Point3f[nPts];
	m_pts = pts;
	Point3f center(0.0f, 0.0f, 0.0f); // center position of point-set
	//#pragma omp parallel for
	if (!data_center_input)
	{
		for (int i = 0; i < nPts; i++) {
			pts[i] = (*m_ptArray)[i];
			center += pts[i];
		}
		center = Point3f(center.x / nPts, center.y / nPts, center.z / nPts);
		m_center = center;
	}
	else
	{
		for (int i = 0; i < nPts; i++) {
			pts[i] = (*m_ptArray)[i];
		}
	}

	// DBG_OUT << "center: " << center.x << " " << center.y << " " << center.z << "\n";
	/*------------------------------------------------*/

	// rotate model, make new nPlane parallel to nz(0.0f, 0.0f, 1.0f)
	// use quaternion rotation
	// ref: https://krasjet.github.io/quaternion/quaternion.pdf
	// nPlane normalization
	m_nPlane = static_cast<Point3f>(vec3_normalize(m_nPlane));
	Point3f nz(0.0f, 0.0f, 1.0f);
	// check if nPlane is parallel to nz
	// if parallel, no need
	bool isNeedRot = !vec3_are_same(m_nPlane, nz); // is need rotation
	
	// translate and rotate
	if (isNeedRot) {
#if 0
		// find normalized rotation axis vector u: rotate nPlane to nz
		// crossproduct(nPlane, nz)
		Point3f u = m_nPlane.cross(nz);
		// calculate rotation angle in radius, two vectors have unit length
		double theta_half = acosf(m_nPlane.dot(nz)) / 2.0f;
		// calculate rotation quaternion (a, bi, cj, dk)
		double a = cos(theta_half);
		double b = sin(theta_half) * u.x;
		double c = sin(theta_half) * u.y;
		double d = sin(theta_half) * u.z;
		// calculate transformation matrix elements
		double mat00 = 1 - 2 * c * c - 2 * d * d;
		double mat01 = 2 * b * c - 2 * a * d;
		double mat02 = 2 * a * c + 2 * b * d;
		double mat10 = 2 * b * c + 2 * a * d;
		double mat11 = 1 - 2 * b * b - 2 * d * d;
		double mat12 = 2 * c * d - 2 * a * b;
		double mat20 = 2 * b * d - 2 * a * c;
		double mat21 = 2 * a * b + 2 * c * d;
		double mat22 = 1 - 2 * b * b - 2 * c * c;
		// translate model based on center to origin (0.0f, 0.0f, 0.0f)
		// then rotate based on quaternion matrix
		#pragma omp parallel for
		for (int i = 0; i < nPts; i++) {
			pts[i] -= center;
			pts[i] = Point3f(
				mat00 * pts[i].x + mat01 * pts[i].y + mat02 * pts[i].z,
				mat10 * pts[i].x + mat11 * pts[i].y + mat12 * pts[i].z,
				mat20 * pts[i].x + mat21 * pts[i].y + mat22 * pts[i].z);
			// local test, no need to update original model
			//m_ptArray->points[i].point = pts[i];
		}
#endif
		CMat rot_mat(3, 3, CV_32F);
		rot_mat = CreateRotationMat4E(m_nPlane, nz);
#pragma omp parallel for
		for (int i = 0; i < nPts; i++) {
			pts[i] -= m_center;
			pts[i] = Point3f(
				rot_mat.at<float>(0, 0) * pts[i].x + rot_mat.at<float>(0, 1) * pts[i].y + rot_mat.at<float>(0, 2) * pts[i].z,
				rot_mat.at<float>(1, 0) * pts[i].x + rot_mat.at<float>(1, 1) * pts[i].y + rot_mat.at<float>(1, 2) * pts[i].z,
				rot_mat.at<float>(2, 0) * pts[i].x + rot_mat.at<float>(2, 1) * pts[i].y + rot_mat.at<float>(2, 2) * pts[i].z);
			// local test, no need to update original model
			//m_ptArray->points[i].point = pts[i];
		}
	}
	// only translate
	else {
		// translate model based on center to origin (0.0f, 0.0f, 0.0f)
		#pragma omp parallel for
		for (int i = 0; i < nPts; i++) {
			pts[i] -= m_center;
			// local test, no need to update original model
			//m_ptArray->points[i].point = pts[i];
			// local test, no need to update original model
		}
	}
	
	/*------------------------------------------------*/

	// create voxelgrid
	VoxelGrid2D_mesh* voxelgrid = new VoxelGrid2D_mesh;
	// parse point-set property

	// find bounding min/max value of x, y
	double x_min = std::numeric_limits<double>::max();
	double x_max = std::numeric_limits<double>::min();
	double y_min = std::numeric_limits<double>::max();
	double y_max = std::numeric_limits<double>::min();
	for (int i = 0; i < nPts - 1; i++) {
		if (pts[i].x < pts[i + 1].x) {
			if (pts[i + 1].x > x_max) x_max = pts[i + 1].x;
			if (pts[i].x < x_min) x_min = pts[i].x;
		}
		else {
			if (pts[i].x > x_max) x_max = pts[i].x;
			if (pts[i + 1].x < x_min) x_min = pts[i + 1].x;
		}
		if (pts[i].y < pts[i + 1].y) {
			if (pts[i + 1].y > y_max) y_max = pts[i + 1].y;
			if (pts[i].y < y_min) y_min = pts[i].y;
		}
		else {
			if (pts[i].y > y_max) y_max = pts[i].y;
			if (pts[i + 1].y < y_min) y_min = pts[i + 1].y;
		}
	}
	// set voxel size
	// calculate average density of points
	// assume uniform density, bounding box area / number of points gives a rough neighborhood of a point
    double vl_tmp = 2.0 * sqrt((x_max - x_min) * (y_max - y_min) / double(nPts));
	//log_info("vl_tmp = %f voxel_size =%f", vl_tmp, voxel_size);
	double vl;
	if (voxel_size != 0)
	{
		vl = voxel_size;
	}
	else
	{
		vl = vl_tmp;
	}
	//log_info("2D voxel size = %lf", vl);

	// set voxelgrid larger than the bounding box of model projection region
	double grid_x_min = x_min - (x_max - x_min) * 0.1;
	double grid_y_min = y_min - (y_max - y_min) * 0.1;
	double grid_x_max = x_max + (x_max - x_min) * 0.1;
	double grid_y_max = y_max + (y_max - y_min) * 0.1;
	unsigned int extent_x = MAX(1, static_cast<unsigned int>((grid_x_max - grid_x_min) / vl)); // MAX handle degenerate case
	unsigned int extent_y = MAX(1, static_cast<unsigned int>((grid_y_max - grid_y_min) / vl));
	// set voxelgrid property
	voxelgrid->setAbsOrigin(grid_x_min, grid_y_min); //left-upper as origin
	if (!voxelgrid->setLenVoxel(vl)) {
		log_error("error Detc_Edge2D::init -> setLenVoxel invalid\n");
		return false;
	}
	if (!voxelgrid->setExtent(extent_x, extent_y)) {
		log_error("error Detc_Edge2D::init -> setExtent invalid\n");
		return false;
	}
	
	// initialize voxelgrid
	// init image
	initImgVoxelGrid(extent_y, extent_x); //m_img_voxelgrid is 8UC1 image mask with dimension of extent_y x extent_x, indicating occupied or point intensity
	if (voxelgrid->create(pts, nPts, true, m_img_voxelgrid)) { //voxelgrid stores point info
		// if init success, set m_voxelgrid for feature detection
		m_voxelgrid = voxelgrid;
	}
	else {
		return false;
	}

	imwrite(DETC_EDGE2D_OUTPUT_PATH"voxelgrid.jpg", m_img_voxelgrid);
	std::string file_name = output_path + "voxelgrid.jpg";
	file_name = "D:/data/outmesh_30_test/test.jpg";
	imwrite(file_name, m_img_voxelgrid);

	// clear memory
	//delete pts;
	return true;
}

bool Detc_Edge2D::initImgVoxelGrid(unsigned int row, unsigned int col) {
	if (row && col) {
		m_img_voxelgrid = CMat::zeros(row, col, CV_8UC3);
		return true;
	}
	return false;
}

void Detc_Edge2D::clear() {
	setPtArray(nullptr);
	setNPlane(Point3f(0.0f, 0.0f, 0.0f));
	clearVoxelGrid();
	clearContours();
	m_img_voxelgrid.release();
	edge_pts_in_projected_plane.clear();
	edge_pts_in_projected_plane.shrink_to_fit();
	delete[] m_pts;
}

void Detc_Edge2D::clearVoxelGrid() {
	if (m_voxelgrid) {
		m_voxelgrid->clear();
		VoxelGrid2D_mesh* temp = m_voxelgrid;
		m_voxelgrid = nullptr;
		delete temp;
	}
}

void Detc_Edge2D::clearContours() {
	size_t n_cnts = m_cnts.size();
	for (unsigned int i = 0; i < n_cnts; i++) {
		m_cnts[i].clear();
	}
	m_cnts.clear();
}

bool Detc_Edge2D::detect() {
	detect_img_cnt();
	return true;
}

bool Detc_Edge2D::detect_bn() {
	detect_pt_edge_by_neighbor_voxels();
	return true;
}

void Detc_Edge2D::detect_img_cnt() {
	// detect image contour
	using namespace cv;
	CMat src, src_gray;
	src = m_img_voxelgrid; // shallow copy
	cvtColor(src, src_gray, CV_BGR2GRAY);
	CMat dst_thres;
	
	interim_value_type rtn_thrshd = static_cast<interim_value_type>(threshold(src_gray, dst_thres, 127, 255, 0));
	//Vector<CVec4i> hierarchy;
	//or use RETR_CCOMP ?
	findContours(dst_thres, m_cnts, m_hierarchy, RETR_TREE, CHAIN_APPROX_NONE);

	//CMat dst = src.clone();
	//drawContours(dst, m_cnts, -1, (0, 255, 255), 1);
	//std::string file_name = output_path + "voxelgrid_edge.jpg";
	//imwrite(file_name, dst);
}

void Detc_Edge2D::detect_img_edge() {
	using namespace cv;
	CMat src, src_gray;
	CMat dst, detected_edges;

	int edgeThresh = 3;
	int lowThreshold = 1;
	int const max_lowThreshold = 3;
	int ratio = 3;
	int kernel_size = 3;

	// Load an image
	src = m_img_voxelgrid;
	if (!src.data) { return; }
	// Create a matrix of the same type and size as src (for dst)
	dst.create(src.size(), src.type());
	// Convert the image to grayscale
	cvtColor(src, src_gray, CV_BGR2GRAY);
	//cvtColor(src, src_gray, C_BGR2GRAY);
	// using Canny edge detection
	detect_img_edge_Canny(src_gray, detected_edges, 
		lowThreshold, ratio, kernel_size, dst, src);
}

// test opencv functions
void Detc_Edge2D::detect_img_edge_Canny(CMat src_gray, 
	CMat detected_edges, int lowThreshold, int ratio,
	int kernel_size, CMat dst, CMat src)
{
	using namespace cv;
	// Reduce noise with a kernel 3x3
	blur(src_gray, detected_edges, CSize2i(3, 3));
	// Canny detector
	Canny(detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size);
	// Using Canny's output as a mask, we display our result
	//dst = Scalar::all(0);
	//src.copyTo(dst, detected_edges);

	//std::string file_name = output_path + "voxelgrid_edge.jpg";
	//imwrite(file_name, dst);
}

void Detc_Edge2D::detect_pt_edge_by_intensity() {
	//select low-intensity voxels
	float intensity_threshold_for_edge_voxel = 50.f; //to tune, lower is sparser
	//float voxel_intensity; //single voxel intensity
	unsigned int edge_voxel_tmp_size = m_voxelgrid->getNumCol() * m_voxelgrid->getNumRow();
	std::vector<std::pair<unsigned int, unsigned int>> edge_voxel_idx(edge_voxel_tmp_size); //storing edge-voxel row, col
	unsigned int edge_voxel_counter = 0;
	Voxel2D_mesh** voxels = m_voxelgrid->getVoxels();
	for (unsigned int i = 0; i < m_voxelgrid->getNumCol(); i++) {
		for (unsigned int j = 0; j < m_voxelgrid->getNumRow(); j++) {
			if (voxels[i][j].isOccupied() && voxels[i][j].getIntensity() <= intensity_threshold_for_edge_voxel) {
				edge_voxel_idx[edge_voxel_counter] = std::make_pair(i, j);
				edge_voxel_counter++;
			}
		}
	}
	edge_voxel_idx.resize(edge_voxel_counter);

	/* debug: save pts in low_intensity voxels*/
	std::string file_name, file_type;
	file_name = "E:/Code/3DModules-master/3dmodules/data/output/detc_edge/low_intensity_pts_xyz";
	file_type = ".txt";
	std::string test_output_path = file_name + "_" + file_type;
	std::string delimiter = ";";
	std::vector<unsigned int> pt_idx_in_voxel;
	std::vector<cv::Point3f> pts_in_voxels;
	for (int i = 0; i < edge_voxel_idx.size(); i++) {
		pt_idx_in_voxel = voxels[edge_voxel_idx[i].first][edge_voxel_idx[i].second].getPtIDs();
		for (int j = 0; j < pt_idx_in_voxel.size(); j++) {
			pts_in_voxels.push_back(m_pts[pt_idx_in_voxel[j]]);
		}
	}
	//if (IOData::SavePoint3fDataWithDelimiter(test_output_path, delimiter, pts_in_voxels))
	if (true)
		std::cout << "saving done" << std::endl;
	else
		std::cout << "saving failed" << std::endl;
}

void Detc_Edge2D::detect_pt_edge_by_neighbor_voxels() {
	Voxel2D_mesh** voxels = m_voxelgrid->getVoxels();
	min_pt_num_in_voxel = 50;
	//min_pt_num_in_voxel = 1;
	kernal_half_length = 25.f; /*long side of stripe kernal*/
	//kernal_half_length = 100.f; /*long side of stripe kernal*/
	kernal_half_height = kernal_half_length * 0.2f; /*short side of stripe kernal, more larger, more sparser the detected edge*/
	min_pt_num_in_kernal = (unsigned int)((float)min_pt_num_in_voxel * 0.2f);
	kernal_length_buffer = kernal_half_length * 0.05f; /*deadzone of edge point along long side of stripe kernal, it is MUST*/

	/********* middle area ************/
	/*collect voxel idx having edge poits*/
	/*non_edge_voxel if all 8 neighbors are occpuied && have enough points, otherwise edge_voxel*/
	unsigned int voxels_rows = m_voxelgrid->getNumRow();
	unsigned int voxels_cols = m_voxelgrid->getNumCol();
	std::vector<std::pair<unsigned int, unsigned int>> edge_voxel_idx(voxels_rows * voxels_cols); //storing edge-voxel row, col
	unsigned int edge_voxel_counter = 0;
	for (int i = 1; i < voxels_cols - 1; i++) { //x
		for (int j = 1; j < voxels_rows - 1; j++) { //y
			if (voxels[i][j].isOccupied() && voxels[i][j].getNumPtIds() > 1 /*self*/ && ( /*below are neighbors*/
				(!voxels[i - 1][j - 1].isOccupied() || voxels[i - 1][j - 1].getNumPtIds() <= min_pt_num_in_voxel) || (!voxels[i][j - 1].isOccupied() || voxels[i][j - 1].getNumPtIds() <= min_pt_num_in_voxel) || (!voxels[i + 1][j - 1].isOccupied() || voxels[i + 1][j - 1].getNumPtIds() <= min_pt_num_in_voxel)
				|| (!voxels[i - 1][j].isOccupied() || voxels[i - 1][j].getNumPtIds() <= min_pt_num_in_voxel) || (!voxels[i + 1][j].isOccupied() || voxels[i + 1][j].getNumPtIds() <= min_pt_num_in_voxel)
				|| (!voxels[i - 1][j + 1].isOccupied() || voxels[i - 1][j + 1].getNumPtIds() <= min_pt_num_in_voxel) || (!voxels[i][j + 1].isOccupied() || voxels[i][j + 1].getNumPtIds() <= min_pt_num_in_voxel) || (!voxels[i + 1][j + 1].isOccupied() || voxels[i + 1][j + 1].getNumPtIds() <= min_pt_num_in_voxel))) {
				//debug: save single edge voxel 
				edge_voxel_idx[edge_voxel_counter] = std::make_pair(i, j);
				edge_voxel_counter++;
			}
		}
	}
	//log_debug("edge_voxel_counter =%d", edge_voxel_counter);
	edge_voxel_idx.resize(edge_voxel_counter);
	/*find edge pts in each edge voxel: step 1 - collect points from self voxel & neigbhor voxels*/
	std::vector<std::pair<int, int>> neighbor_voxel_idx_offsets(8);
	neighbor_voxel_idx_offsets = {
		std::make_pair(-1, -1),		std::make_pair(-1, 0),		std::make_pair(-1, +1),
		std::make_pair(0, -1),		/*(0, 0) is self*/			std::make_pair(0, +1),
		std::make_pair(+1, -1),		std::make_pair(+1, 0),		std::make_pair(+1, +1) };
	std::vector<std::vector<unsigned int>> pts_in_edge_voxels; /*pts in multiple edge voxels*/
	std::vector<std::vector<unsigned int>> pts_in_edge_voxels_neighbors; /*pts in all neighbors of multiple edge voxels*/
	this->collect_edge_pts_and_neighbor_pts(voxels, edge_voxel_idx, neighbor_voxel_idx_offsets, pts_in_edge_voxels, pts_in_edge_voxels_neighbors);
	//VXL_PTS(pts_in_edge_voxels);
	//VXL_PTS(pts_in_edge_voxels_neighbors);

	edge_voxel_idx.clear();
	edge_voxel_idx.shrink_to_fit();
	edge_voxel_counter = 0;
	neighbor_voxel_idx_offsets.clear(); 
	neighbor_voxel_idx_offsets.shrink_to_fit();
	///* debug: save candidate pts for edge point searching in projected plane*/
	std::cout << "pts_in_edge_voxels_neighbors size: " << pts_in_edge_voxels_neighbors.size() << std::endl;
	std::vector<cv::Point3f> candidate_pts;
	for (int i = 0; i < pts_in_edge_voxels_neighbors.size(); i++) {
		std::cout << "pts_in_edge_voxels_neighbors " << i << " size: " << pts_in_edge_voxels_neighbors[i].size() << std::endl;
		for (int j = 0; j < pts_in_edge_voxels_neighbors[i].size(); j++) {
			candidate_pts.push_back(m_pts[pts_in_edge_voxels_neighbors[i][j]]);
		}
	}
	if (candidate_pts.empty()) { std::cout << "empty candidate pts" << std::endl; return; }
	std::string folder, file_name, file_type;
	folder = DETC_EDGE2D_OUTPUT_PATH;
	file_name = "candidate_pts";
	file_type = ".txt";
	std::string test_output_path = folder + file_name + file_type;
	std::string delimiter = ";";
	//if (IOData::SavePoint3fDataWithDelimiter(test_output_path, delimiter, candidate_pts))
	if (true)
		std::cout << "saving candiate pts done" << std::endl;
	else
		std::cout << "saving candidate pts failed" << std::endl;

	//find edge pts in each edge voxel: step 2 - scan edge points using 1st-order x- & y- kernals
	this->find_edge_pts_by_bilateral_kernal(pts_in_edge_voxels, pts_in_edge_voxels_neighbors, kernal_half_length, kernal_half_height);
	//log_debug("edge_pts_in_projected_plane size =%d", edge_pts_in_projected_plane.size());

	std::vector<std::vector<unsigned int>> tmp1;
	pts_in_edge_voxels.swap(tmp1);
	pts_in_edge_voxels_neighbors.swap(tmp1);
	/********* Left lateral ************/
	/*collect edge voxel in left lateral, assuming all occupied left-lateral voxels are edge voxels*/
	for (int i = 0; i < voxels_rows; i++) {
		if (voxels[0][i].isOccupied()) {
			//edge_voxel_idx[edge_voxel_counter] = std::make_pair(0, i);
			//edge_voxel_counter++;
			edge_voxel_idx.push_back(std::make_pair(0, i));
		}
	}
	//log_debug("edge_voxel_idx size =%d", edge_voxel_idx.size());

	//edge_voxel_idx.resize(edge_voxel_counter);
	/*find edge pts in each edge voxel: step 1 - collect points from self voxel & neigbhor voxels*/
	neighbor_voxel_idx_offsets = { std::make_pair(1, 0) };
	this->collect_edge_pts_and_neighbor_pts(voxels, edge_voxel_idx, neighbor_voxel_idx_offsets, pts_in_edge_voxels, pts_in_edge_voxels_neighbors);
	//VXL_PTS(pts_in_edge_voxels);
	//VXL_PTS(pts_in_edge_voxels_neighbors);

	edge_voxel_idx.clear();
	edge_voxel_idx.shrink_to_fit();
	edge_voxel_counter = 0;
	neighbor_voxel_idx_offsets.clear();
	neighbor_voxel_idx_offsets.shrink_to_fit();
	//find edge pts in each edge voxel: step 2 - scan edge points using 1st-order x- & y- kernals
	this->find_edge_pts_by_bilateral_kernal(pts_in_edge_voxels, pts_in_edge_voxels_neighbors, kernal_half_length, kernal_half_height);
	//log_debug("edge_pts_in_projected_plane size =%d", edge_pts_in_projected_plane.size());

	pts_in_edge_voxels.swap(tmp1);
	pts_in_edge_voxels_neighbors.swap(tmp1);

	/********* Right lateral ************/
	/*collect edge voxel in right lateral, assuming all occupied left-lateral voxels are edge voxels*/
	for (int i = 0; i < voxels_rows; i++) {
		if (voxels[voxels_cols - 1][i].isOccupied()) {
			//edge_voxel_idx[edge_voxel_counter] = std::make_pair(voxels_cols - 1, i);
			//edge_voxel_counter++;
			edge_voxel_idx.push_back(std::make_pair(voxels_cols - 1, i));
		}
	}
	//log_debug("edge_voxel_idx size =%d", edge_voxel_idx.size());

	//edge_voxel_idx.resize(edge_voxel_counter);
	/*find edge pts in each edge voxel: step 1 - collect points from self voxel & neigbhor voxels*/
	neighbor_voxel_idx_offsets = { std::make_pair(-1, 0) };
	this->collect_edge_pts_and_neighbor_pts(voxels, edge_voxel_idx, neighbor_voxel_idx_offsets, pts_in_edge_voxels, pts_in_edge_voxels_neighbors);
	//VXL_PTS(pts_in_edge_voxels);
	//VXL_PTS(pts_in_edge_voxels_neighbors);


	edge_voxel_idx.clear();
	edge_voxel_idx.shrink_to_fit();
	edge_voxel_counter = 0;
	neighbor_voxel_idx_offsets.clear();
	neighbor_voxel_idx_offsets.shrink_to_fit();
	//find edge pts in each edge voxel: step 2 - scan edge points using 1st-order x- & y- kernals
	this->find_edge_pts_by_bilateral_kernal(pts_in_edge_voxels, pts_in_edge_voxels_neighbors, kernal_half_length, kernal_half_height);
	//log_debug("edge_pts_in_projected_plane size =%d", edge_pts_in_projected_plane.size());

	pts_in_edge_voxels.swap(tmp1);
	pts_in_edge_voxels_neighbors.swap(tmp1);

	/********* Top lateral ************/
	/*collect edge voxel in top lateral, assuming all occupied top-lateral voxels are edge voxels*/
	for (int i = 0; i < voxels_cols; i++) {
		if (voxels[i][0].isOccupied()) {
			//edge_voxel_idx[edge_voxel_counter] = std::make_pair(i, 0);
			//edge_voxel_counter++;
			edge_voxel_idx.push_back(std::make_pair(i, 0));
		}
	}
	//log_debug("edge_voxel_idx size =%d", edge_voxel_idx.size());

	//edge_voxel_idx.resize(edge_voxel_counter);
	/*find edge pts in each edge voxel: step 1 - collect points from self voxel & neigbhor voxels*/
	neighbor_voxel_idx_offsets = { std::make_pair(0, 1) };
	this->collect_edge_pts_and_neighbor_pts(voxels, edge_voxel_idx, neighbor_voxel_idx_offsets, pts_in_edge_voxels, pts_in_edge_voxels_neighbors);
	//VXL_PTS(pts_in_edge_voxels);
	//VXL_PTS(pts_in_edge_voxels_neighbors);


	edge_voxel_idx.clear();
	edge_voxel_idx.shrink_to_fit();
	edge_voxel_counter = 0;
	neighbor_voxel_idx_offsets.clear();
	neighbor_voxel_idx_offsets.shrink_to_fit();
	//find edge pts in each edge voxel: step 2 - scan edge points using 1st-order x- & y- kernals
	this->find_edge_pts_by_bilateral_kernal(pts_in_edge_voxels, pts_in_edge_voxels_neighbors, kernal_half_length, kernal_half_height);
	//log_debug("edge_pts_in_projected_plane size =%d", edge_pts_in_projected_plane.size());

	pts_in_edge_voxels.swap(tmp1);
	pts_in_edge_voxels_neighbors.swap(tmp1);

	/********* Bottom lateral ************/
	/*collect edge voxel in bottom lateral, assuming all occupied bottom-lateral voxels are edge voxels*/
	for (int i = 0; i < voxels_cols; i++) {
		if (voxels[i][voxels_rows - 1].isOccupied()) {
			//edge_voxel_idx[edge_voxel_counter] = std::make_pair(i, voxels_rows - 1);
			//edge_voxel_counter++;
			edge_voxel_idx.push_back(std::make_pair(i, voxels_rows - 1));
		}
	}
	//log_debug("edge_voxel_idx size =%d", edge_voxel_idx.size());

	//edge_voxel_idx.resize(edge_voxel_counter);
	/*find edge pts in each edge voxel: step 1 - collect points from self voxel & neigbhor voxels*/
	neighbor_voxel_idx_offsets = { std::make_pair(0, -1) };
	this->collect_edge_pts_and_neighbor_pts(voxels, edge_voxel_idx, neighbor_voxel_idx_offsets, pts_in_edge_voxels, pts_in_edge_voxels_neighbors);
	//VXL_PTS(pts_in_edge_voxels);
	//VXL_PTS(pts_in_edge_voxels_neighbors);

	edge_voxel_idx.clear();
	edge_voxel_idx.shrink_to_fit();
	edge_voxel_counter = 0;
	neighbor_voxel_idx_offsets.clear();
	neighbor_voxel_idx_offsets.shrink_to_fit();
	//find edge pts in each edge voxel: step 2 - scan edge points using 1st-order x- & y- kernals
	this->find_edge_pts_by_bilateral_kernal(pts_in_edge_voxels, pts_in_edge_voxels_neighbors, kernal_half_length, kernal_half_height);
	//log_debug("edge_pts_in_projected_plane size =%d", edge_pts_in_projected_plane.size());

	pts_in_edge_voxels.swap(tmp1);
	pts_in_edge_voxels_neighbors.swap(tmp1);

	/* debug: save 3D edge pts in projected plane*/
	//if (edge_pts_in_projected_plane.empty()) { std::cout << "empty edge pts" << std::endl; return; }
	//std::string folder, file_name, file_type;
	//folder = output_path;
	//file_name = "edge_pts";
	//file_type = ".txt";
	//std::string test_output_path = folder + file_name + file_type;
	//std::string delimiter = ";";
	//if (IOData::SavePoint3fDataWithDelimiter(test_output_path, delimiter, edge_pts_in_projected_plane))
	//{
	//	log_info("saving edge pts done");
	//}
	//else
	//{
	//	log_error("saving edge pts failed");

	//}
}

void Detc_Edge2D::collect_edge_pts_and_neighbor_pts(Voxel2D_mesh** voxels,
	const std::vector<std::pair<unsigned int, unsigned int>>& edge_voxel_idx, 
	const std::vector<std::pair<int, int>>& neighbor_voxel_idx_offsets, 
	std::vector<std::vector<unsigned int>>& pts_in_edge_voxels, 
	std::vector<std::vector<unsigned int>>& pts_in_edge_voxels_neighbors) {
	std::vector<unsigned int> pts_in_edge_voxel; /*pts in one selected edge voxel*/ /*all are pt indices*/
	std::vector<unsigned int> pts_in_edge_voxel_neighbor; /*pts in self edge voxel or one neighbor of one selected edge voxel*/
	std::vector<unsigned int> pts_in_edge_voxel_neighbors; /*pts in self edge voxel + all neighbors of one selected edge voxel*/
	pts_in_edge_voxels.resize(edge_voxel_idx.size()); /*pts in multiple edge voxels*/
	pts_in_edge_voxels_neighbors.resize(edge_voxel_idx.size()); /*pts in all neighbors of multiple edge voxels*/
	for (int i = 0; i < edge_voxel_idx.size(); i++) {
		pts_in_edge_voxel = voxels[edge_voxel_idx[i].first][edge_voxel_idx[i].second].getPtIDs(); //points in self voxel
		pts_in_edge_voxel_neighbors.insert(pts_in_edge_voxel_neighbors.end(), pts_in_edge_voxel.begin(), pts_in_edge_voxel.end());
		for (int k = 0; k < neighbor_voxel_idx_offsets.size(); k++) {
			if (voxels[edge_voxel_idx[i].first + neighbor_voxel_idx_offsets[k].first][edge_voxel_idx[i].second + neighbor_voxel_idx_offsets[k].second].isOccupied()) {
				pts_in_edge_voxel_neighbor = voxels[edge_voxel_idx[i].first + neighbor_voxel_idx_offsets[k].first][edge_voxel_idx[i].second + neighbor_voxel_idx_offsets[k].second].getPtIDs();
				pts_in_edge_voxel_neighbors.insert(pts_in_edge_voxel_neighbors.end(), pts_in_edge_voxel_neighbor.begin(), pts_in_edge_voxel_neighbor.end());
				pts_in_edge_voxel_neighbor.clear();
				pts_in_edge_voxel_neighbor.shrink_to_fit();
			}
		}
		pts_in_edge_voxels[i] = pts_in_edge_voxel;
		pts_in_edge_voxels_neighbors[i] = pts_in_edge_voxel_neighbors;
		pts_in_edge_voxel.clear();
		pts_in_edge_voxel.shrink_to_fit();
		pts_in_edge_voxel_neighbors.clear();
		pts_in_edge_voxel_neighbors.shrink_to_fit();
	}
}

void Detc_Edge2D::find_edge_pts_by_bilateral_kernal(const std::vector<std::vector<unsigned int>>& pts_in_edge_voxels, 
	const std::vector<std::vector<unsigned int>>& pts_in_edge_voxels_neighbors, 
	const float kernal_half_length, 
	const float kernal_half_height) {
	float kernal_length_x_minus, kernal_length_x_plus, kernal_length_y_minus, kernal_length_y_plus;
	//float kernal_height_x_minus, kernal_height_x_plus, kernal_height_y_minus, kernal_height_y_plus;
	unsigned int x_kernal_minus_counter, x_kernal_plus_counter, y_kernal_minus_counter, y_kernal_plus_counter;
	float x_kernal_height_dist, y_kernal_height_dist;
	//std::cout << pts_in_edge_voxels.size() << "edge voxels" << std::endl;
	for (int i = 0; i < pts_in_edge_voxels.size(); i++) {
		//std::cout << "edge voxel " << i << ": " << pts_in_edge_voxels[i].size() << " pts" << std::endl;
		//std::cout << "neighbors of edge voxel " << i << ": " << pts_in_edge_voxels_neighbors[i].size() << " pts" << std::endl;
		for (int j = 0; j < pts_in_edge_voxels[i].size(); j++) {
			kernal_length_x_minus = m_pts[pts_in_edge_voxels[i][j]].x - kernal_half_length;	kernal_length_x_plus = m_pts[pts_in_edge_voxels[i][j]].x + kernal_half_length;
			kernal_length_y_minus = m_pts[pts_in_edge_voxels[i][j]].y - kernal_half_length;	kernal_length_y_plus = m_pts[pts_in_edge_voxels[i][j]].y + kernal_half_length;
			x_kernal_minus_counter = x_kernal_plus_counter = y_kernal_minus_counter = y_kernal_plus_counter = 0;
			for (int k = 0; k < pts_in_edge_voxels_neighbors[i].size(); k++) {
				x_kernal_height_dist = std::fabs(m_pts[pts_in_edge_voxels_neighbors[i][k]].y - m_pts[pts_in_edge_voxels[i][j]].y);
				y_kernal_height_dist = std::fabs(m_pts[pts_in_edge_voxels_neighbors[i][k]].x - m_pts[pts_in_edge_voxels[i][j]].x);
				if (m_pts[pts_in_edge_voxels_neighbors[i][k]].x >= kernal_length_x_minus && m_pts[pts_in_edge_voxels_neighbors[i][k]].x <= m_pts[pts_in_edge_voxels[i][j]].x - kernal_length_buffer && x_kernal_height_dist <= kernal_half_height)
					x_kernal_minus_counter++;
				else if (m_pts[pts_in_edge_voxels_neighbors[i][k]].x <= kernal_length_x_plus && m_pts[pts_in_edge_voxels_neighbors[i][k]].x > m_pts[pts_in_edge_voxels[i][j]].x + kernal_length_buffer && x_kernal_height_dist <= kernal_half_height)
					x_kernal_plus_counter++;
				if (m_pts[pts_in_edge_voxels_neighbors[i][k]].y >= kernal_length_y_minus && m_pts[pts_in_edge_voxels_neighbors[i][k]].y <= m_pts[pts_in_edge_voxels[i][j]].y - kernal_length_buffer && y_kernal_height_dist <= kernal_half_height)
					y_kernal_minus_counter++;
				else if (m_pts[pts_in_edge_voxels_neighbors[i][k]].y <= kernal_length_y_plus && m_pts[pts_in_edge_voxels_neighbors[i][k]].y > m_pts[pts_in_edge_voxels[i][j]].y + kernal_length_buffer && y_kernal_height_dist <= kernal_half_height)
					y_kernal_plus_counter++;
			}
			if ((x_kernal_minus_counter == 0 && x_kernal_plus_counter > 0) || (x_kernal_minus_counter > 0 && x_kernal_plus_counter == 0)
				|| (y_kernal_minus_counter == 0 && y_kernal_plus_counter > 0) || (y_kernal_minus_counter > 0 && y_kernal_plus_counter == 0)) {
				edge_pts_in_projected_plane.push_back(m_pts[pts_in_edge_voxels[i][j]]);
				//std::cout << "adding edge pt: "  << x_kernal_minus_counter << " , " << x_kernal_plus_counter << " , " << y_kernal_minus_counter << " , " << y_kernal_plus_counter << std::endl;
			}	
		}
	}
}

void Detc_Edge2D::setPtArray(Point3fArray* ptArray) {
	m_ptArray = ptArray;
}

void Detc_Edge2D::setNPlane(const Point3f nPlane) {
	m_nPlane = nPlane;
	// normalization
	m_nPlane = Util_Math::vec3_normalize(m_nPlane);
	// DBG_OUT << "m_nPlane: " << m_nPlane.x << " " << m_nPlane.y << " " << m_nPlane.z << "\n";
}

Point3fArray* Detc_Edge2D::getPtArray() {
	return m_ptArray;
}

Point3f Detc_Edge2D::getNPlane() {
	return m_nPlane;
}

VoxelGrid2D_mesh* Detc_Edge2D::getVoxelGrid() {
	return m_voxelgrid;
}

CMat Detc_Edge2D::getImgVoxelGrid() {
	return m_img_voxelgrid;
}

vector<vector<Point2i>> Detc_Edge2D::getContours() {
	return m_cnts;
}

bool Detc_Edge2D::setVoxelSize(double voxel_length) {
	voxel_size = voxel_length;
	return true;
}
double Detc_Edge2D::getVoxelSize() {
	return voxel_size;
}

bool Detc_Edge2D::get2DContours(Vector<Point2fArray> &contours)
{
	//vector<vector<Point3f>> project_contours;
	contours.clear();
	//contours.resize(m_cnts.size());
	Voxel2D_mesh **voxel_2d = m_voxelgrid->getVoxels();

	//int extend_x = m_voxelgrid->getExtentX();
	int extend_y = m_voxelgrid->getExtentY();
	double v_length_x = m_voxelgrid->getLenVoxelX();
	double v_length_y = m_voxelgrid->getLenVoxelY();

	//double inverse_v_lx = 1 / v_length_x;
	//double inverse_v_ly = 1 / v_length_y;

	double abs_ox = m_voxelgrid->getAbsOriginX();
	double abs_oy = m_voxelgrid->getAbsOriginY();

	for (int i = 0; i < m_cnts.size(); i++)
	{
		Point2fArray tmp_map;
		if (m_cnts[i].size() < min_point_num_of_contour) continue;
		//contours[i].clear();
		tmp_map.clear();
		for (int j = 0; j < m_cnts[i].size(); j++)
		{
			Point2i cur_point = m_cnts[i][j];
			Point2f point;
			point.x = static_cast<float>((cur_point.x) * v_length_x + abs_ox + v_length_x / 2);
			point.x = static_cast<float>((cur_point.x) * v_length_x + abs_ox + v_length_x / 2);
			point.y = static_cast<float>((extend_y-cur_point.y)*v_length_y + abs_oy - v_length_y / 2); // max point should be extend_y -1
			point.y = static_cast<float>((extend_y-cur_point.y)*v_length_y + abs_oy - v_length_y / 2); // max point should be extend_y -1
			//contours[i].push_back(point);
			tmp_map.push_back(point);
		}
		contours.push_back(tmp_map);
	}
	return true;	
}
bool Detc_Edge2D::get2DContours(Vector<Point2fArray> &contours, std::vector<cv::Vec<int, 4>>& hierarchy)
{
	//vector<vector<Point3f>> project_contours;
	contours.clear();
	//contours.resize(m_cnts.size());
	Voxel2D_mesh **voxel_2d = m_voxelgrid->getVoxels();

	int extend_x = m_voxelgrid->getExtentX();
	int extend_y = m_voxelgrid->getExtentY();
	double v_length_x = m_voxelgrid->getLenVoxelX();
	double v_length_y = m_voxelgrid->getLenVoxelY();

	//double inverse_v_lx = 1 / v_length_x;
	//double inverse_v_ly = 1 / v_length_y;

	double abs_ox = m_voxelgrid->getAbsOriginX();
	double abs_oy = m_voxelgrid->getAbsOriginY();

	//std::bitset<hierarchy.size()>  bVisit(0); 
	std::vector<std::bitset<1>> vIsVisit(hierarchy.size());
	//std::unordered_map<int, vector<int>> mapEdgeHoleList;
	std::unordered_map<int, int> mapEdgeHoleListTemp;
	//hierarchy is adjacency table,use level traversal
	//two level struct,and screen
	for (size_t i = 0; i < hierarchy.size(); i++)
	{
		vIsVisit[i].set();
		//if (hierarchy[i][3] > -1)
		//	mapEdgeHoleList[i].push_back(hierarchy[i][3]);
		//if (hierarchy[i][3] > -1/*&&m_cnts[i].size()>min_point_num_of_contour*/)
		//	mapEdgeHoleList[hierarchy[i][3]].push_back(i);
		if(m_cnts[i].size()>min_point_num_of_contour)
			mapEdgeHoleListTemp[i]=(hierarchy[i][3]);
	} 
	for (auto it = mapEdgeHoleListTemp.begin(); it != mapEdgeHoleListTemp.end(); it++)
	{
		if (it->second > -1 && 
			mapEdgeHoleListTemp[it->second]==-1)
			mapEdgeHoleList[it->second].push_back(it->first);
		else if (it->second == -1 &&
			mapEdgeHoleList.count(it->first)==0)
			mapEdgeHoleList[it->first] = {};
	}
///all tree
	contours.resize(m_cnts.size());

	vector<vector<int>> map(extend_x, vector<int>(extend_y, -1));
	for (auto y : mapEdgeHoleList)
	{
		for (int i = 0; i < m_cnts[y.first].size(); i++)
		{
			Point2i cur_point = m_cnts[y.first][i];
			if (map[cur_point.x][cur_point.y] == -1)
				map[cur_point.x][cur_point.y] = i;
			else
			{
				//指针的对应位置不一致导致的
				//contours[it->first].end()=contours[it->first].erase(contours[it->first].begin() +map[cur_point.x][cur_point.y],
				//			contours[it->first].end()),  
				//		map[cur_point.x][cur_point.y]=i;

				//cout << map[cur_point.x][cur_point.y]<< "," << i << endl;
				//inhibition sawtooth
				if (i - map[cur_point.x][cur_point.y] < 3)
					for (int j = 1; j < i - map[cur_point.x][cur_point.y]; j++)
						contours[y.first].pop_back();
				map[cur_point.x][cur_point.y] = i;
				continue;
			}

			Point2f point;
			//point.x = static_cast<float>((cur_point.x) * v_length_x + abs_ox + v_length_x / 2);
			point.x = static_cast<float>((cur_point.x) * v_length_x + abs_ox + v_length_x / 2) + m_center.x;
			//point.y = static_cast<float>((extend_y-cur_point.y)*v_length_y + abs_oy - v_length_y / 2); // max point should be extend_y -1
			point.y = static_cast<float>((extend_y - cur_point.y) * v_length_y + abs_oy - v_length_y / 2) + m_center.y; // max point should be extend_y -1
			contours[y.first].push_back(point);
		}
		//hole
		for (auto x : y.second)
		{
			for (int i = 0; i < m_cnts[x].size(); i++)
			{
				Point2i cur_point = m_cnts[x][i];
				if (map[cur_point.x][cur_point.y] == -1)
					map[cur_point.x][cur_point.y] = i;
				else
					continue;
				Point2f point;
				//point.x = static_cast<float>((cur_point.x) * v_length_x + abs_ox + v_length_x / 2);
				point.x = static_cast<float>((cur_point.x) * v_length_x + abs_ox + v_length_x / 2) + m_center.x;
				//point.y = static_cast<float>((extend_y-cur_point.y)*v_length_y + abs_oy - v_length_y / 2); // max point should be extend_y -1
				point.y = static_cast<float>((extend_y - cur_point.y) * v_length_y + abs_oy - v_length_y / 2) + m_center.y; // max point should be extend_y -1
				contours[x].push_back(point);
			}
		}
	}
	return true;
#ifdef By_Tree
	decltype(mapEdgeHoleList.begin()) max_it(mapEdgeHoleList.begin());
	for (auto it = ++mapEdgeHoleList.begin(); it != mapEdgeHoleList.end(); it++)
		if (m_cnts[it->first].size() > m_cnts[max_it->first].size())
			mapEdgeHoleList.erase(max_it),max_it = it;
		else
			mapEdgeHoleList.erase(it);
	vector<vector<int>> map(extend_x, vector<int>(extend_y, -1));
	for (int i = 0; i < m_cnts[max_it->first].size(); i++)
	{
		Point2i cur_point = m_cnts[max_it->first][i];
		if (map[cur_point.x][cur_point.y] == -1)
			map[cur_point.x][cur_point.y] = i;
		else
		{
			//指针的对应位置不一致导致的
			//contours[it->first].end()=contours[it->first].erase(contours[it->first].begin() +map[cur_point.x][cur_point.y],
			//			contours[it->first].end()),  
			//		map[cur_point.x][cur_point.y]=i;

			//cout << map[cur_point.x][cur_point.y]<< "," << i << endl;
			//inhibition sawtooth
			if (i - map[cur_point.x][cur_point.y] < 3)
				for (int j = 1; j < i - map[cur_point.x][cur_point.y]; j++)
					contours[max_it->first].pop_back();
			map[cur_point.x][cur_point.y] = i;
			continue;
		}

		Point2f point;
		//point.x = static_cast<float>((cur_point.x) * v_length_x + abs_ox + v_length_x / 2);
		point.x = static_cast<float>((cur_point.x) * v_length_x + abs_ox + v_length_x / 2) + m_center.x;
		//point.y = static_cast<float>((extend_y-cur_point.y)*v_length_y + abs_oy - v_length_y / 2); // max point should be extend_y -1
		point.y = static_cast<float>((extend_y - cur_point.y) * v_length_y + abs_oy - v_length_y / 2) + m_center.y; // max point should be extend_y -1
		contours[max_it->first].push_back(point);
	}
	//hole
	for (auto x : max_it->second)
	{ 
		for (int i = 0; i < m_cnts[x].size(); i++)
		{
			Point2i cur_point = m_cnts[x][i];
			if (map[cur_point.x][cur_point.y] == -1)
				map[cur_point.x][cur_point.y] = i;
			else
				continue;
			Point2f point;
			//point.x = static_cast<float>((cur_point.x) * v_length_x + abs_ox + v_length_x / 2);
			point.x = static_cast<float>((cur_point.x) * v_length_x + abs_ox + v_length_x / 2) + m_center.x;
			//point.y = static_cast<float>((extend_y-cur_point.y)*v_length_y + abs_oy - v_length_y / 2); // max point should be extend_y -1
			point.y = static_cast<float>((extend_y - cur_point.y) * v_length_y + abs_oy - v_length_y / 2) + m_center.y; // max point should be extend_y -1
			contours[x].push_back(point);
		}
	}
	return true;
#endif // By_Tree

	//vector<vector<int>> map(extend_x, vector<int>(extend_y, 0));
	//for (int i = 0; i < m_cnts.size(); i++)
	//{
	//	Point2fArray tmp_map;
	//	//if (m_cnts[i].size() < min_point_num_of_contour) continue;
	//	//contours[i].clear();
	//	tmp_map.clear();
	//	for (int j = 0; j < m_cnts[i].size(); j++)
	//	{
	//		Point2i cur_point = m_cnts[i][j];

	//		map[cur_point.x][cur_point.y] ++;
	//		if (map[cur_point.x][cur_point.y] == 1)
	//		{
	//		}
	//		else if (map[cur_point.x][cur_point.y] == 2)
	//		{
	//			continue;
	//		}
	//		else
	//		{
	//			continue;
	//		}
	//		Point2f point;
	//		//point.x = static_cast<float>((cur_point.x) * v_length_x + abs_ox + v_length_x / 2);
	//		point.x = static_cast<float>((cur_point.x) * v_length_x + abs_ox + v_length_x / 2)+m_center.x;
	//		//point.y = static_cast<float>((extend_y-cur_point.y)*v_length_y + abs_oy - v_length_y / 2); // max point should be extend_y -1
	//		point.y = static_cast<float>((extend_y-cur_point.y)*v_length_y + abs_oy - v_length_y / 2)+m_center.y; // max point should be extend_y -1
	//		//contours[i].push_back(point);
	//		tmp_map.push_back(point);
	//	}

	//	contours.push_back(tmp_map);
	//}
	//map.clear();
	return true;	
}
//int networkDelayTime(vector<vector<int>>& times, int n, int k) {
//	const int inf = INT_MAX / 2;
//	vector<vector<int>> g(n, vector<int>(n, inf));
//	for (auto& t : times) {
//		int x = t[0] - 1, y = t[1] - 1;
//		g[x][y] = t[2];
//	}
//
//	vector<int> dist(n, inf);
//	dist[k - 1] = 0;
//	vector<int> used(n);
//	for (int i = 0; i < n; ++i) {
//		int x = -1;
//		for (int y = 0; y < n; ++y) {
//			if (!used[y] && (x == -1 || dist[y] < dist[x])) {
//				x = y;
//			}
//		}
//		used[x] = true;
//		for (int y = 0; y < n; ++y) {
//			dist[y] = min(dist[y], dist[x] + g[x][y]);
//		}
//	}
//
//	int ans = *max_element(dist.begin(), dist.end());
//	return ans == inf ? -1 : ans;
//}
int networkDelayTime(vector<vector<int>>& times, int n, int k) {
	//拓扑排序
	vector<vector<int>> map(n, vector<int>(n, INT_MAX));
	for (size_t i = 0; i < times.size(); i++)
		map[times[i][0] - 1][times[i][1] - 1] = times[i][2];
	vector<int> path(n, INT_MAX);
	vector<int> point({ k });
	for (int i = 0; i < n; i++)
	{
		if (i == k)continue;
		for (auto x:point)
		{
			if(map[x][i]<INT_MAX&&path[x]<INT_MAX)
				path[i] = min(path[i], path[x] + map[x][i]);
		}
		point.push_back(i);
	}

	return *min_element(path.begin(),path.end());
}
//vector<int> eventualSafeNodes(vector<vector<int>>& graph) {
//	vector<int> dp(graph.size(), -1);
//	for (int i = 0; i < graph.size(); i++)
//		if (graph[i].size() == 0)
//			dp[i] = 1;
//	bool is = true;
//	for(int iter = 0;iter<graph.size();iter++)
//	for (int i = 0; i < graph.size(); i++)
//	{
//		for (int j = 0; j < graph[i].size(); j++)
//		{
//			is = is && (dp[graph[i][j]] == 1);
//		}
//		if (is)
//			dp[i] = 1;
//	}
//	vector<int> result;
//	for (int i = 0; i < dp.size(); i++)
//		if(dp[i]==1)
//		result.push_back(i);
//	return result;
//}
//
vector<int> eventualSafeNodes(vector<vector<int>>& graph)
{
	vector<int> dp(graph.size(),false);
	vector<bool> isVisit(graph.size());
	function<bool(int)> dfs = [&](int k) {
		auto is = true;
		
		if (graph[k].size() == 0)
			return is;
		for (auto x : graph[k])
			is = is && dfs(x);
		if (is)
			dp[k]=true;
		isVisit[k] = true;
		return is;
	};
	for (int i = 0; i < graph.size(); i++)
		if (!isVisit[i])
			dfs(i);
	vector<int> result;
	for (int i = 0; i < dp.size(); i++)
		if(dp[i])
			result.push_back(i);
	return result;
}

/* by contour method, not accurate */
bool Detc_Edge2D::getOrigMapOfContours(vector<Point3fArray> &orig_map)
{

	Point3f nz(0.0f, 0.0f, 1.0f);
	bool isNeedRot = !Util_Math::vec3_are_same(m_nPlane, nz); // is need rotation
	CMat rot_mat(3, 3, CV_32F);
	
	if (isNeedRot)
	{
		rot_mat = Util_Math::CreateRotationMat4E(nz, m_nPlane);
	}

	//vector<vector<Point3f>> project_contours;
	orig_map.clear();
	//orig_map.resize(m_cnts.size());
	Voxel2D_mesh **voxel_2d = m_voxelgrid->getVoxels();

	//int extend_x = m_voxelgrid->getExtentX();
	int extend_y = m_voxelgrid->getExtentY();
	double v_length_x = m_voxelgrid->getLenVoxelX();
	double v_length_y = m_voxelgrid->getLenVoxelY();

	//double inverse_v_lx = 1 / v_length_x;
	//double inverse_v_ly = 1 / v_length_y;

	double abs_ox = m_voxelgrid->getAbsOriginX();
	double abs_oy = m_voxelgrid->getAbsOriginY();

	for (int i = 0; i < m_cnts.size(); i++)
	{
		Point3fArray tmp_map;
		if (m_cnts[i].size() < min_point_num_of_contour) continue;
		//orig_map[i].clear();
		tmp_map.clear();
		for (int j = 0; j < m_cnts[i].size(); j++)
		{
			cv::Point cur_point = m_cnts[i][j];
			Point3f point,origin_point;
			point.x = static_cast<float>((cur_point.x) * v_length_x + abs_ox + v_length_x/2);
			point.y = static_cast<float>((extend_y - cur_point.y)*v_length_y + abs_oy - v_length_y/2); // max point should be extend_y -1
			point.z = 0.f;
			if (isNeedRot)
			{
				Point3f orig_point = Point3f(
					rot_mat.at<float>(0, 0) * point.x + rot_mat.at<float>(0, 1) * point.y + rot_mat.at<float>(0, 2) * point.z,
					rot_mat.at<float>(1, 0) * point.x + rot_mat.at<float>(1, 1) * point.y + rot_mat.at<float>(1, 2) * point.z,
					rot_mat.at<float>(2, 0) * point.x + rot_mat.at<float>(2, 1) * point.y + rot_mat.at<float>(2, 2) * point.z);

				orig_point = orig_point + m_center;
				//orig_map[i].push_back(orig_point);
				tmp_map.push_back(orig_point);
			}
			else
			{
				point = point + m_center;
				//orig_map[i].push_back(point);
				tmp_map.push_back(point);
			}
		}
		orig_map.push_back(tmp_map);
	}

	return true;
}


bool Detc_Edge2D::get2DContours_bn(Point2fArray& contours)
{
	contours.clear();
	contours.resize(edge_pts_in_projected_plane.size());

	for (int i = 0; i < contours.size(); i++)
	{
		contours[i].x = edge_pts_in_projected_plane[i].x;
		contours[i].y = edge_pts_in_projected_plane[i].y;
	}
	return true;
}



/*rotate back accurate edge points from projected plane to original 3D space*/
bool Detc_Edge2D::getOrigMapOfContours(ModuleStruct::Point3fArray& orig_map)
{
	Point3f nz(0.0f, 0.0f, 1.0f);
	bool isNeedRot = !Util_Math::vec3_are_same(m_nPlane, nz); // is need rotation
	CMat rot_mat(3, 3, CV_32F);

	if (isNeedRot)
	{
		rot_mat = Util_Math::CreateRotationMat4E(nz, m_nPlane);
	}
	//else
	//	return false;

	orig_map.resize(edge_pts_in_projected_plane.size());
	Point3f orig_point;
	for (int i = 0; i < edge_pts_in_projected_plane.size(); i++) 
	{

		if (isNeedRot)
		{
			orig_point = Point3f(
				rot_mat.at<float>(0, 0)* edge_pts_in_projected_plane[i].x + rot_mat.at<float>(0, 1) * edge_pts_in_projected_plane[i].y,
				rot_mat.at<float>(1, 0)* edge_pts_in_projected_plane[i].x + rot_mat.at<float>(1, 1) * edge_pts_in_projected_plane[i].y,
				rot_mat.at<float>(2, 0)* edge_pts_in_projected_plane[i].x + rot_mat.at<float>(2, 1) * edge_pts_in_projected_plane[i].y);			
			orig_point = orig_point + m_center;
			orig_map[i] = orig_point;
		}	
		else
		{
			orig_point.x = edge_pts_in_projected_plane[i].x;
			orig_point.y = edge_pts_in_projected_plane[i].y;
			orig_point.z = 0.f;
			orig_point = orig_point + m_center;
			orig_map[i] = orig_point;
		}

	}
	return true;
}

