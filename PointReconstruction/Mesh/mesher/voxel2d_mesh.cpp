#include "voxel2d_mesh.h"
#include <cstdlib>
#include <iostream>
//#include "../config_dbg.h"
#include "../../log/log.h"

// voxel2d
Voxel2D_mesh::Voxel2D_mesh() 
	: m_is_occupied(false)
	, m_intensity(0.0f)
{
}

Voxel2D_mesh::~Voxel2D_mesh() {
	clear();
}

void Voxel2D_mesh::clear() {
	m_pt_ids.clear();
	m_pt_ids.shrink_to_fit();
	m_pts.clear();
	m_pts.shrink_to_fit();
}
 
void Voxel2D_mesh::reset() {
	clear();
	setOccupied(false);
	setIntensity(0.0f);
}


void Voxel2D_mesh::appendPtID(const unsigned int pt_id) {
	m_pt_ids.push_back(pt_id);
}

std::vector<unsigned int> Voxel2D_mesh::getPtIDs() {
	return m_pt_ids;
}

unsigned int Voxel2D_mesh::getNumPtIds() const {
	return static_cast<unsigned int>(m_pt_ids.size());
}

void Voxel2D_mesh::appendPt(const cv::Point2f point2f){
	m_pts.push_back(point2f);
}

std::vector<cv::Point2f> Voxel2D_mesh::getPts() {
	return m_pts;
}


bool Voxel2D_mesh::isOccupied() const {
	return m_is_occupied;
}


void Voxel2D_mesh::setOccupied(const bool isOccupied) {
	m_is_occupied = isOccupied;
}

float Voxel2D_mesh::getIntensity() const {
	return m_intensity;
}

bool Voxel2D_mesh::setIntensity(const float intensity) {
	if (intensity < 0.0f) { // intensity >= 0.0f
		return false;
	}
	m_intensity = intensity;
	return true;
}

// voxelgrid
// init with one unit voxel
VoxelGrid2D_mesh::VoxelGrid2D_mesh()
	: m_voxels(nullptr)
	, m_extent_x(1)
	, m_extent_y(1)
	, m_len_vx(1.0f)
	, m_len_vy(1.0f)
	, m_abs_ox(0.0f)
	, m_abs_oy(0.0f)
	, m_is_voxelSquare(true)
	{

}

VoxelGrid2D_mesh::~VoxelGrid2D_mesh() {
	clear();
}

void VoxelGrid2D_mesh::reset() {
	// clear voxels data
	// reset other members
	clear();
	m_extent_x = 1;
	m_extent_y = 1;
	m_abs_ox = 0.0f;
	m_abs_oy = 0.0f;
	m_len_vx = 1.0f;
	m_len_vy = 1.0f;
	m_is_voxelSquare = true;
}

void VoxelGrid2D_mesh::clear() {
	clearVoxels();
}

void VoxelGrid2D_mesh::clearVoxels() {
	// clear voxels data
	if (m_voxels != nullptr) {
		for (unsigned int i = 0; i < m_extent_x; i++) {
			if (m_voxels[i] != nullptr) {
				for (unsigned int j = 0; j < m_extent_y; j++) {
					m_voxels[i][j].clear();
				}
			}
		}
		Voxel2D_mesh** temp = m_voxels;
		m_voxels = nullptr;
		for (unsigned int i = m_extent_x - 1; i > -1; i--) {
			delete temp[i];
		}
		delete temp;
	}
}


void VoxelGrid2D_mesh::initVoxels() {
	clearVoxels();
	m_voxels = new Voxel2D_mesh*[m_extent_x];
	for (unsigned int i = 0; i < m_extent_x; i++) {
		m_voxels[i] = new Voxel2D_mesh[m_extent_y];
	}
}
					
bool VoxelGrid2D_mesh::create(ModuleStruct::Point3f* pts, const unsigned int npts, const bool isBinary, ModuleStruct::CMat& img) {
	// validate points
	if (pts == nullptr) {
		log_error("error VoxelGrid2D_mesh::create->points nullptr\n");
		return false;
	}
	// initialize voxels
	initVoxels();
	// append point to voxel
	//#pragma omp parallel for

	double inv_len_vx = 1.0f / m_len_vx;
	double inv_len_vy = 1.0f / m_len_vy;

	for (unsigned int i = 0; i < npts; i++) {
		unsigned int id_x = static_cast<unsigned int>(floor((pts[i].x - m_abs_ox) * inv_len_vx));
		unsigned int id_y = static_cast<unsigned int>(floor((pts[i].y - m_abs_oy) * inv_len_vy));
		if (id_y == m_extent_y) id_y--;
		if (id_x == m_extent_x) id_x--;
		m_voxels[id_x][id_y].setOccupied(true);
		m_voxels[id_x][id_y].appendPtID(i);
		m_voxels[id_x][id_y].appendPt(cv::Point2f(pts[i].x - m_abs_ox, pts[i].y - m_abs_oy));
	}

	// set voxel-point intensity
	// find voxel with most points, record number for normalization
	unsigned int max_npts = 0;
	for (unsigned int i = 0; i < m_extent_x; i++) {
		for (unsigned int j = 0; j < m_extent_y; j++) {
			if (max_npts < m_voxels[i][j].getNumPtIds()) 
				max_npts = m_voxels[i][j].getNumPtIds();
		}
	}
	// set intensity
	for (unsigned int i = 0; i < m_extent_x; i++) {
		for (unsigned int j = 0; j < m_extent_y; j++) {
			// linear
			//m_voxels[i][j].setIntensity(
			//	unsigned int(255.0 * m_voxels[i][j].getNumPtIds() / max_npts));

			// non-linear
			//float e2x = exp(2.0f * float(m_voxels[i][j].getNumPtIds()) / float(avgPtInVoxel));
			float e2x = exp(2.0f * static_cast<float>(m_voxels[i][j].getNumPtIds()) / max_npts);
			float tanh = (e2x - 1.0f) / (e2x + 1.0f);
			m_voxels[i][j].setIntensity(static_cast<float>(floor(255.0f * tanh)));
		}
	}
	using namespace cv;
	// test export image
	//Mat img = Mat::zeros(m_extent_y, m_extent_x, CV_8UC3);
	if (isBinary) {
		for (unsigned int i = 0; i < m_extent_x; i++) {
			for (unsigned int j = 0; j < m_extent_y; j++) {
				// binary output: if occupied, set color white
				if (m_voxels[i][j].isOccupied()) {
					img.at<Vec3b>(m_extent_y - 1 - j, i)[0] = 255;
					img.at<Vec3b>(m_extent_y - 1 - j, i)[1] = 255;
					img.at<Vec3b>(m_extent_y - 1 - j, i)[2] = 255;
				}
			}
		}
	}
	else{
		for (unsigned int i = 0; i < m_extent_x; i++) {
			for (unsigned int j = 0; j < m_extent_y; j++) {
				//intepolation output
				unsigned int intensity = static_cast<unsigned int>(m_voxels[i][j].getIntensity());
				img.at<Vec3b>(m_extent_y - 1 - j, i)[0] = intensity;
				img.at<Vec3b>(m_extent_y - 1 - j, i)[1] = intensity;
				img.at<Vec3b>(m_extent_y - 1 - j, i)[2] = intensity;
			}
		}
	}
	return true;
}

bool VoxelGrid2D_mesh::create(const ModuleStruct::Point3fArray &pts, const bool isBinary, ModuleStruct::CMat &img)
{
	// validate points
	if (pts.empty()) {
		log_error( "error VoxelGrid2D_mesh::create->points empty\n");
		return false;
	}
	unsigned int npts = static_cast<unsigned int>(pts.size());
	// initialize voxels
	initVoxels();
	// append point to voxel
	//#pragma omp parallel for

	double inv_len_vx = 1.0f / m_len_vx;
	double inv_len_vy = 1.0f / m_len_vy;

	for (unsigned int i = 0; i < npts; i++) {
		unsigned int id_x = static_cast<unsigned int>(floor((pts[i].x - m_abs_ox) * inv_len_vx));
		unsigned int id_y = static_cast<unsigned int>(floor((pts[i].y - m_abs_oy) * inv_len_vy));
		m_voxels[id_x][id_y].setOccupied(true);
		m_voxels[id_x][id_y].appendPtID(i);
		m_voxels[id_x][id_y].appendPt(cv::Point2f(pts[i].x - m_abs_ox, pts[i].y - m_abs_oy));
	}

	// set voxel-point intensity
	// find voxel with most points, record number for normalization
	unsigned int max_npts = 0;
	for (unsigned int i = 0; i < m_extent_x; i++) {
		for (unsigned int j = 0; j < m_extent_y; j++) {
			if (max_npts < m_voxels[i][j].getNumPtIds()) max_npts = m_voxels[i][j].getNumPtIds();
		}
	}
	// set intensity
	for (unsigned int i = 0; i < m_extent_x; i++) {
		for (unsigned int j = 0; j < m_extent_y; j++) {
			// linear
			//m_voxels[i][j].setIntensity(
			//	unsigned int(255.0 * m_voxels[i][j].getNumPtIds() / max_npts));

			// non-linear
			//float e2x = exp(2.0f * float(m_voxels[i][j].getNumPtIds()) / float(avgPtInVoxel));
			float e2x = exp(2.0f * static_cast<float>(m_voxels[i][j].getNumPtIds()) / max_npts);
			float tanh = (e2x - 1.0f) / (e2x + 1.0f);
			m_voxels[i][j].setIntensity(static_cast<float>(floor(255.0f * tanh)));
		}
	}

	using namespace cv;
	// test export image
	//Mat img = Mat::zeros(m_extent_y, m_extent_x, CV_8UC3);
	if (isBinary) {
		for (unsigned int i = 0; i < m_extent_x; i++) {
			for (unsigned int j = 0; j < m_extent_y; j++) {
				// binary output: if occupied, set color white
				if (m_voxels[i][j].isOccupied()) {
					img.at<Vec3b>(j, i)[0] = 255;
					img.at<Vec3b>(j, i)[1] = 255;
					img.at<Vec3b>(j, i)[2] = 255;
				}
			}
		}
	}
	else {
		for (unsigned int i = 0; i < m_extent_x; i++) {
			for (unsigned int j = 0; j < m_extent_y; j++) {
				//intepolation output
				unsigned int intensity = static_cast<unsigned int>(m_voxels[i][j].getIntensity());
				img.at<Vec3b>(j, i)[0] = intensity;
				img.at<Vec3b>(j, i)[1] = intensity;
				img.at<Vec3b>(j, i)[2] = intensity;
			}
		}
	}

	//// test export image
	////Mat img = Mat::zeros(m_extent_y, m_extent_x, CV_8UC3);
	//if (isBinary) {
	//	for (unsigned int i = 0; i < m_extent_x; i++) {
	//		for (unsigned int j = 0; j < m_extent_y; j++) {
	//			// binary output: if occupied, set color white
	//			if (m_voxels[i][j].isOccupied()) {
	//				img.at<Vec3b>(m_extent_y - 1 - j, i)[0] = 255;
	//				img.at<Vec3b>(m_extent_y - 1 - j, i)[1] = 255;
	//				img.at<Vec3b>(m_extent_y - 1 - j, i)[2] = 255;
	//			}
	//		}
	//	}
	//}
	//else {
	//	for (unsigned int i = 0; i < m_extent_x; i++) {
	//		for (unsigned int j = 0; j < m_extent_y; j++) {
	//			//intepolation output
	//			unsigned int intensity = unsigned int(m_voxels[i][j].getIntensity());
	//			img.at<Vec3b>(m_extent_y - 1 - j, i)[0] = intensity;
	//			img.at<Vec3b>(m_extent_y - 1 - j, i)[1] = intensity;
	//			img.at<Vec3b>(m_extent_y - 1 - j, i)[2] = intensity;
	//		}
	//	}
	//}

	return true;
}

bool VoxelGrid2D_mesh::setExtent(const unsigned int x, const unsigned int y) {

	if (x < 1) {
		log_error ("error VoxelGrid2D_mesh::setExtent->invalid x\n");
		return false;
	}
	if (y < 1) {
		log_error("error VoxelGrid2D_mesh::setExtent->invalid y\n");
		return false;
	}
	m_extent_x = x;
	m_extent_y = y;
	return true;
}


bool VoxelGrid2D_mesh::setLenVoxel(const double l) {
	if (l <= 0.0f) {
		log_error("error VoxelGrid2D_mesh::setLenVoxel->invalid l\n");
		return false;
	}
	m_len_vx = l;
	m_len_vy = l;
	m_is_voxelSquare = true;
	return true;
}

bool VoxelGrid2D_mesh::setLenVoxel(const double lx, const double ly) {

	if (lx <= 0.0f) {
		log_error("error VoxelGrid2D_mesh::setLenVoxel->invalid lx\n");
		return false;
	}
	if (ly <= 0.0f) {
		log_error( "error VoxelGrid2D_mesh::setLenVoxel->invalid ly\n");
		return false;
	}
	// check if lx == ly
	if (abs(lx - ly) < 1e-6) {
		// consider square voxel
		m_len_vx = lx;
		m_len_vy = lx;
		m_is_voxelSquare = true;
	}
	else {
		// consider non-square voxel
		m_len_vx = lx;
		m_len_vy = ly;
		m_is_voxelSquare = false;
	}
	return true;
}

void VoxelGrid2D_mesh::setAbsOrigin(const double ox, const double oy) {
	m_abs_ox = ox;
	m_abs_oy = oy;
}

Voxel2D_mesh** VoxelGrid2D_mesh::getVoxels() {
	return m_voxels;
}

void VoxelGrid2D_mesh::getExtent(unsigned int& x, unsigned int& y) {
	x = m_extent_x;
	y = m_extent_y;
}

unsigned int VoxelGrid2D_mesh::getExtentX() const {
	return m_extent_x;
}

unsigned int VoxelGrid2D_mesh::getExtentY() const {
	return m_extent_y;
}

unsigned int VoxelGrid2D_mesh::getNumCol() const {
	return m_extent_x;
}

unsigned int VoxelGrid2D_mesh::getNumRow() const {
	return m_extent_y;
}

double VoxelGrid2D_mesh::getLenVoxelX() const {
	return m_len_vx;
}

double VoxelGrid2D_mesh::getLenVoxelY() const {
	return m_len_vy;
}

double VoxelGrid2D_mesh::getAbsOriginX() const {
	return m_abs_ox;
}

double VoxelGrid2D_mesh::getAbsOriginY() const {
	return m_abs_oy;
}

void VoxelGrid2D_mesh::getAbsOrigin(double& x, double& y) {
	x = m_abs_ox;
	y = m_abs_oy;
}

bool VoxelGrid2D_mesh::isVoxelSquare() const {
	return m_is_voxelSquare;
}

