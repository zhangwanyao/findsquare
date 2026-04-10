#include "proj2d.h"
#include "../utility/util_pca.h"
#include "bbox.h"
#include "util_math.hpp"

/**
* \brief check two opencv matrix or vector equal or not
*/
template<class T>
inline bool isEqual_cvType(const T &a, const T &b) {
	return cv::countNonZero(a != b) == 0;
}

Proj2D::Proj2D()
	: m_trans_3d(cv::Point3f(0.f, 0.f, 0.f))
	, m_trans_2d(cv::Point2f(0.f, 0.f))
	, m_voxelgrid(nullptr)
	, m_len_voxel(0.1f)
	, m_extent_x(10)
	, m_extent_y(10)
{
	m_proj = cv::Mat::eye(3, 3, CV_32F);
	m_inv_proj = cv::Mat::eye(3, 3, CV_32F);
}

Proj2D::~Proj2D()
{
	clear();
}

void Proj2D::clear()
{
	clearVoxelGrid();
}

void Proj2D::clearVoxelGrid() {
	if (m_voxelgrid) {
		m_voxelgrid->clear();
		VoxelGrid2D* temp = m_voxelgrid;
		m_voxelgrid = nullptr;
		delete temp;
	}
}

bool Proj2D::init()
{
	return true;
}

void Proj2D::process(const std::vector<cv::Point3f> &pts_orig, cv::Mat &img)
{
	// data validation
	if (pts_orig.empty()) return;
	// copy point data
	std::vector<cv::Point3f> pts = pts_orig;
	size_t npts = pts.size();
	// translate pts for rotation
	if (!isEqual_cvType(m_trans_3d, cv::Point3f(0.f, 0.f, 0.f))) {
//#pragma omp parallel for
		for (size_t i = 0; i < npts; i++) {
			pts[i] += m_trans_3d;
		}
	}
	// rotate pts to x-y plane
	cv::Mat mat_eye = cv::Mat::eye(3, 3, CV_32F);
	if (!isEqual_cvType(m_proj, mat_eye)){
		float mat00 = m_proj.at<float>(0, 0);
		float mat01 = m_proj.at<float>(0, 1);
		float mat02 = m_proj.at<float>(0, 2);
		float mat10 = m_proj.at<float>(1, 0);
		float mat11 = m_proj.at<float>(1, 1);
		float mat12 = m_proj.at<float>(1, 2);
		float mat20 = m_proj.at<float>(2, 0);
		float mat21 = m_proj.at<float>(2, 1);
		float mat22 = m_proj.at<float>(2, 2);
//#pragma omp parallel for
		for (size_t i = 0; i < npts; i++) {
			pts[i] = cv::Point3f(
				mat00 * pts[i].x + mat01 * pts[i].y + mat02 * pts[i].z,
				mat10 * pts[i].x + mat11 * pts[i].y + mat12 * pts[i].z,
				mat20 * pts[i].x + mat21 * pts[i].y + mat22 * pts[i].z);
		}
	}
	// translate pts for voxelization on 2d
	if (!isEqual_cvType(m_trans_2d, cv::Point2f(0.f, 0.f))) {
		cv::Point3f trans_xy(m_trans_2d.x, m_trans_2d.y, 0.f);
//#pragma omp parallel for
		for (size_t i = 0; i < npts; i++) {
			pts[i] += trans_xy;
		}
	}
	// init voxelgrid
	if (!m_voxelgrid) {
		m_voxelgrid = new VoxelGrid2D;
	}
	m_voxelgrid->setAbsOrigin(0, 0);
	m_voxelgrid->setLenVoxel(m_len_voxel);
	m_voxelgrid->setExtent(m_extent_x, m_extent_y);
	// create voxel
	img = cv::Mat::zeros(m_extent_y, m_extent_x, CV_8UC3);
	m_voxelgrid->create(pts, true, img);
	pts.clear();
}

/**
* \brief process
*/
void Proj2D::process(const std::vector<cv::Point3f> &pts_orig, std::vector<cv::Point2i> &ptVx, cv::Mat &img)
{
	// data validation
	if (pts_orig.empty()) return;
	// copy point data
	std::vector<cv::Point3f> pts = pts_orig;
	size_t npts = pts.size();
	// translate pts for rotation
	if (!isEqual_cvType(m_trans_3d, cv::Point3f(0.f, 0.f, 0.f))) {
//#pragma omp parallel for
		for (size_t i = 0; i < npts; i++) {
			pts[i] += m_trans_3d;
		}
	}
	// rotate pts to x-y plane
	cv::Mat mat_eye = cv::Mat::eye(3, 3, CV_32F);
	if (!isEqual_cvType(m_proj, mat_eye)) {
		float mat00 = m_proj.at<float>(0, 0);
		float mat01 = m_proj.at<float>(0, 1);
		float mat02 = m_proj.at<float>(0, 2);
		float mat10 = m_proj.at<float>(1, 0);
		float mat11 = m_proj.at<float>(1, 1);
		float mat12 = m_proj.at<float>(1, 2);
		float mat20 = m_proj.at<float>(2, 0);
		float mat21 = m_proj.at<float>(2, 1);
		float mat22 = m_proj.at<float>(2, 2);
//#pragma omp parallel for
		for (size_t i = 0; i < npts; i++) {
			pts[i] = cv::Point3f(
				mat00 * pts[i].x + mat01 * pts[i].y + mat02 * pts[i].z,
				mat10 * pts[i].x + mat11 * pts[i].y + mat12 * pts[i].z,
				mat20 * pts[i].x + mat21 * pts[i].y + mat22 * pts[i].z);
		}
	}
	// translate pts for voxelization on 2d
	if (!isEqual_cvType(m_trans_2d, cv::Point2f(0.f, 0.f))) {
		cv::Point3f trans_xy(m_trans_2d.x, m_trans_2d.y, 0.f);
//#pragma omp parallel for
		for (size_t i = 0; i < npts; i++) {
			pts[i] += trans_xy;
		}
	}
	// init voxelgrid
	if (!m_voxelgrid) {
		m_voxelgrid = new VoxelGrid2D;
	}
	m_voxelgrid->setAbsOrigin(0, 0);
	m_voxelgrid->setLenVoxel(m_len_voxel);
	m_voxelgrid->setExtent(m_extent_x, m_extent_y);
	// create voxel
	img = cv::Mat::zeros(m_extent_y, m_extent_x, CV_8UC3);
	m_voxelgrid->create(pts, ptVx, img);
	pts.clear();
}

void Proj2D::process(const std::vector<cv::Point3f> &pts_orig, 
	const bool isLocal, cv::Mat &img)
{
	// data validation
	if (pts_orig.empty()) return;
	// check if is local projection
	if (!isLocal) {
		process(pts_orig, img);
		return;
	}
	// copy point data
	std::vector<cv::Point3f> pts = pts_orig;
	size_t npts = pts.size();
	float inv_npts = 1.f / static_cast<float>(npts);
	
	// 3d translate
	// compute center
	cv::Point3f center(0.f, 0.f, 0.f);
	for (size_t i = 0; i < npts; i++) {
		center += pts[i];
	}
	center *= inv_npts;
	// set trans_3d
	m_trans_3d = -center;
	// translate
//#pragma omp parallel for
	for (size_t i = 0; i < npts; i++) {
		pts[i] += m_trans_3d;
	}
	// 3d rotate
	// compute projection/rotation matrix
	cv::Point3f normal = Proj2D::getPCANormal(pts);
	cv::Point3f nz(0.0f, 0.0f, 1.0f);
	cv::Point3f u = normal.cross(nz);
	// calculate rotation angle in radius, two vectors have unit length
	float theta = acosf(normal.dot(nz));

	Util_Math::get_rot_mat3(u, theta, m_proj);

	// rotate pts to x-y plane
	float mat00 = m_proj.at<float>(0, 0);
	float mat01 = m_proj.at<float>(0, 1);
	float mat02 = m_proj.at<float>(0, 2);
	float mat10 = m_proj.at<float>(1, 0);
	float mat11 = m_proj.at<float>(1, 1);
	float mat12 = m_proj.at<float>(1, 2);
	float mat20 = m_proj.at<float>(2, 0);
	float mat21 = m_proj.at<float>(2, 1);
	float mat22 = m_proj.at<float>(2, 2);
//#pragma omp parallel for
	for (size_t i = 0; i < npts; i++) {
		pts[i] = cv::Point3f(
			mat00 * pts[i].x + mat01 * pts[i].y + mat02 * pts[i].z,
			mat10 * pts[i].x + mat11 * pts[i].y + mat12 * pts[i].z,
			mat20 * pts[i].x + mat21 * pts[i].y + mat22 * pts[i].z);
	}

	// 2d translate
	// get bbox2d (x, y) of projected point cloud
	BBox2D *bbox2d = new BBox2D();
	bbox2d->compute(pts, BBox2D::TYPE_COORD::COORD_XY);
	// set trans_2d
	m_trans_2d = cv::Point2f(-bbox2d->m_min.x + 1, -bbox2d->m_min.y + 1);
	// translate
	cv::Point3f trans_xy(m_trans_2d.x, m_trans_2d.y, 0.f);
//#pragma omp parallel for
	for (size_t i = 0; i < npts; i++) {
		pts[i] += trans_xy;
	}

	// voxelize
	// compute voxel length
	cv::Point2f bbox_diag = bbox2d->m_max - bbox2d->m_min;
	m_len_voxel = MAX(bbox_diag.x, bbox_diag.y) / 256.f;
	m_extent_x = bbox_diag.x / m_len_voxel + 10;
	m_extent_y = bbox_diag.y / m_len_voxel + 10;

	// init voxelgrid
	if (!m_voxelgrid) {
		m_voxelgrid = new VoxelGrid2D;
	}
	m_voxelgrid->setAbsOrigin(0, 0);
	m_voxelgrid->setLenVoxel(m_len_voxel);
	m_voxelgrid->setExtent(m_extent_x, m_extent_y);
	// create voxel
	img = cv::Mat::zeros(m_extent_y, m_extent_x, CV_8UC3);
	m_voxelgrid->create(pts, true, img);
	pts.clear();

	delete bbox2d;
}



cv::Point3f Proj2D::getPCANormal(const std::vector<cv::Point3f> &pts)
{
	// compute pca of model
	// pca project point to plane
	cv::PCA pca;
	Util_PCA::pca3(pts, pca);
	cv::Mat_<float> eig_vals = pca.eigenvalues;
	// pick minimum eigen value
	float min_eig = eig_vals.at<float>(0, 0);
	unsigned int id_min_eig = 0;
	if (eig_vals.at<float>(1, 0) < min_eig) {
		min_eig = eig_vals.at<float>(1, 0);
		id_min_eig = 1;
	}
	if (eig_vals.at<float>(2, 0) < min_eig) {
		min_eig = eig_vals.at<float>(2, 0);
		id_min_eig = 2;
	}
	// compute normal components
	float a = pca.eigenvectors.at<float>(id_min_eig, 0);
	float b = pca.eigenvectors.at<float>(id_min_eig, 1);
	float c = pca.eigenvectors.at<float>(id_min_eig, 2);
	float len_n = sqrt(a * a + b * b + c * c);
	a = a / len_n; b = b / len_n; c = c / len_n;
	cv::Point3f normal(a, b, c);
	return 	normal;
}

/**
* \brief set trans 3d
*/
void Proj2D::setTrans3D(const cv::Point3f trans_3d) {
	m_trans_3d = trans_3d;
}
/**
* \brief get trans 3d
*/
cv::Point3f Proj2D::getTrans3D() const {
	return m_trans_3d;
}
/**
* \brief set trans 2d
*/
void Proj2D::setTrans2D(const cv::Point2f trans_2d) {
	m_trans_2d = trans_2d;
}
/**
* \brief get trans 2d
*/
cv::Point2f Proj2D::getTrans2D() const {
	return m_trans_2d;
}
/**
* \brief set projection matrix
*/
void Proj2D::setProj(const cv::Mat proj) {
	m_proj = proj;
}
/**
* \brief get projection matrix
*/
cv::Mat Proj2D::getProj() const {
	return m_proj;
}
/**
* \brief set inverse projection matrix
*/
void Proj2D::setInvProj(const cv::Mat inv_proj) {
	m_inv_proj = inv_proj;
}
/**
* \brief get inverse matrix
*/
cv::Mat Proj2D::getInvProj() const {
	return m_inv_proj;
}
/**
* \brief set length of voxel
*/
bool Proj2D::setLenVoxel(const float len_voxel) {
	if (len_voxel < 0.f) return false;
	m_len_voxel = len_voxel;
	return true;
}
/**
* \brief get length of voxel
*/
float Proj2D::getLenVoxel() const {
	return m_len_voxel;
}
/**
* \brief set voxelgrid extent of x direction
*/
bool Proj2D::setExtentX(const unsigned int extent_x) {
	if (extent_x == 0) return false;
	m_extent_x = extent_x;
	return true;
}
/**
* \brief get voxelgrid extent of x direction
*/
float Proj2D::getExtentX() const {
	return m_extent_x;
}
/**
* \brief set voxelgrid extent of y direction
*/
bool Proj2D::setExtentY(const unsigned int extent_y) {
	if (extent_y == 0) return false;
	m_extent_y = extent_y;
	return true;
}
/**
* \brief get voxelgrid extent of y direction
*/
float Proj2D::getExtentY() const {
	return m_extent_y;
}
/**
* \brief get area of occupied region, rough
*/
float Proj2D::getAreaOccupied() const {
	size_t n_voxels = m_voxelgrid->countOccupied();
	return n_voxels * m_len_voxel * m_len_voxel;
}

