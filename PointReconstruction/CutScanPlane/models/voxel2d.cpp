#include "voxel2d.h"
#include <cstdlib>
#include <iostream>
//#include "../config_dbg.h"

// voxel2d
Voxel2D::Voxel2D() 
	: m_is_occupied(false)
	, m_intensity(0.0f)
{
}

Voxel2D::~Voxel2D() {
	clear();
}

void Voxel2D::clear() {
	m_pt_ids.clear();
}

void Voxel2D::reset() {
	clear();
	setOccupied(false);
	setIntensity(0.0f);
}


void Voxel2D::appendPtID(const unsigned int pt_id) {
	m_pt_ids.push_back(pt_id);
}


vector<unsigned int> Voxel2D::getPtIDs() {
	return m_pt_ids;
}

unsigned int Voxel2D::getNumPtIds() const {
	return m_pt_ids.size();
}


bool Voxel2D::isOccupied() const {
	return m_is_occupied;
}


void Voxel2D::setOccupied(const bool isOccupied) {
	m_is_occupied = isOccupied;
}

float Voxel2D::getIntensity() const {
	return m_intensity;
}

bool Voxel2D::setIntensity(const float intensity) {
	if (intensity < 0.0f) { // intensity >= 0.0f
		return false;
	}
	m_intensity = intensity;
	return true;
}

// voxelgrid
// init with one unit voxel
VoxelGrid2D::VoxelGrid2D()
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

VoxelGrid2D::~VoxelGrid2D() {
	clear();
}

void VoxelGrid2D::reset() {
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

void VoxelGrid2D::clear() {
	clearVoxels();
}

void VoxelGrid2D::clearVoxels() {
	// clear voxels data
	if (m_voxels != nullptr) {
		for (unsigned int i = 0; i < m_extent_x; ++i) {
			if (m_voxels[i] != nullptr) {
				for (unsigned int j = 0; j < m_extent_y; ++j) {
					m_voxels[i][j].clear();
				}
			}
		}
		//Voxel2D** temp = m_voxels;
		//m_voxels = nullptr;
		//for (unsigned int i = m_extent_x - 1; i >=-1; i--) {
		//	delete temp[i];
		//}
		for(int i = 0; i < m_extent_x; i++) {
			if (m_voxels[i] != nullptr)
				delete []m_voxels[i];
		}
		delete m_voxels;
		m_voxels = nullptr;
	}
}


void VoxelGrid2D::initVoxels() {
	clearVoxels();
	m_voxels = new Voxel2D*[m_extent_x];
	for (unsigned int i = 0; i < m_extent_x; ++i) {
		m_voxels[i] = new Voxel2D[m_extent_y];
	}
}

bool VoxelGrid2D::create(cv::Point3f* pts, const unsigned int npts, const bool isBinary, cv::Mat& img) {
	// validate points
	if (pts == nullptr) {
		// DBG_OUT << "error VoxelGrid2D::create->points nullptr\n";
		return false;
	}
	// initialize voxels
	initVoxels();
	// append point to voxel
	//#pragma omp parallel for

	double inv_len_vx = 1.0f / m_len_vx;
	double inv_len_vy = 1.0f / m_len_vy;

	for (unsigned int i = 0; i < npts; ++i) {
		unsigned int id_x = (unsigned int)(floor((pts[i].x - m_abs_ox) * inv_len_vx));
		unsigned int id_y = (unsigned int)(floor((pts[i].y - m_abs_oy) * inv_len_vy));
		m_voxels[id_x][id_y].setOccupied(true);
		m_voxels[id_x][id_y].appendPtID(i);
	}

	// set voxel-point intensity
	// find voxel with most points, record number for normalization
	unsigned int max_npts = 0;
	for (unsigned int i = 0; i < m_extent_x; ++i) {
		for (unsigned int j = 0; j < m_extent_y; ++j) {
			if (max_npts < m_voxels[i][j].getNumPtIds()) max_npts = m_voxels[i][j].getNumPtIds();
		}
	}
	// set intensity
	for (unsigned int i = 0; i < m_extent_x; ++i) {
		for (unsigned int j = 0; j < m_extent_y; ++j) {
			// linear
			//m_voxels[i][j].setIntensity(
			//	unsigned int(255.0 * m_voxels[i][j].getNumPtIds() / max_npts));

			// non-linear
			//float e2x = exp(2.0f * float(m_voxels[i][j].getNumPtIds()) / float(avgPtInVoxel));
			float e2x = exp(2.0f * float(m_voxels[i][j].getNumPtIds()) / max_npts);
			float tanh = (e2x - 1.0f) / (e2x + 1.0f);
			m_voxels[i][j].setIntensity((unsigned int)(floor(255.0f * tanh)));
		}
	}
	using namespace cv;
	// test export image
	//Mat img = Mat::zeros(m_extent_y, m_extent_x, CV_8UC3);
	if (isBinary) {
		for (unsigned int i = 0; i < m_extent_x; ++i) {
			for (unsigned int j = 0; j < m_extent_y; ++j) {
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
		for (unsigned int i = 0; i < m_extent_x; ++i) {
			for (unsigned int j = 0; j < m_extent_y; ++j) {
				//intepolation output
				unsigned int intensity = (unsigned int)(m_voxels[i][j].getIntensity());
				img.at<Vec3b>(m_extent_y - 1 - j, i)[0] = intensity;
				img.at<Vec3b>(m_extent_y - 1 - j, i)[1] = intensity;
				img.at<Vec3b>(m_extent_y - 1 - j, i)[2] = intensity;
			}
		}
	}
	return true;
}

bool VoxelGrid2D::create(const std::vector<cv::Point3f> &pts, const bool isBinary, cv::Mat &img)
{
	// validate points
	if (pts.empty()) {
		//DBG_OUT << "error VoxelGrid2D::create->points empty\n";
		return false;
	}
	unsigned int npts = static_cast<unsigned int>(pts.size());
	// initialize voxels
	initVoxels();
	// append point to voxel
	//#pragma omp parallel for

	double inv_len_vx = 1.0f / m_len_vx;
	double inv_len_vy = 1.0f / m_len_vy;

	for (unsigned int i = 0; i < npts; ++i) {
		unsigned int id_x = (unsigned int)(floor((pts[i].x - m_abs_ox) * inv_len_vx));
		unsigned int id_y = (unsigned int)(floor((pts[i].y - m_abs_oy) * inv_len_vy));
		m_voxels[id_x][id_y].setOccupied(true);
		m_voxels[id_x][id_y].appendPtID(i);
	}

	// set voxel-point intensity
	// find voxel with most points, record number for normalization
	unsigned int max_npts = 0;
	for (unsigned int i = 0; i < m_extent_x; ++i) {
		for (unsigned int j = 0; j < m_extent_y; ++j) {
			if (max_npts < m_voxels[i][j].getNumPtIds()) max_npts = m_voxels[i][j].getNumPtIds();
		}
	}
	// set intensity
	for (unsigned int i = 0; i < m_extent_x; ++i) {
		for (unsigned int j = 0; j < m_extent_y; ++j) {
			// linear
			//m_voxels[i][j].setIntensity(
			//	unsigned int(255.0 * m_voxels[i][j].getNumPtIds() / max_npts));

			// non-linear
			//float e2x = exp(2.0f * float(m_voxels[i][j].getNumPtIds()) / float(avgPtInVoxel));
			float e2x = exp(2.0f * float(m_voxels[i][j].getNumPtIds()) / max_npts);
			float tanh = (e2x - 1.0f) / (e2x + 1.0f);
			m_voxels[i][j].setIntensity((unsigned int)(floor(255.0f * tanh)));
		}
	}

	using namespace cv;
	// test export image
	//Mat img = Mat::zeros(m_extent_y, m_extent_x, CV_8UC3);

	int r = rand() % 255 + 1;
	int g = rand() % 255 + 1;
	int b = rand() % 255 + 1;

	if (isBinary) {
		for (unsigned int i = 0; i < m_extent_x; ++i) {
			for (unsigned int j = 0; j < m_extent_y; ++j) {
				// binary output: if occupied, set color white
				if (m_voxels[i][j].isOccupied()) {
					img.at<Vec3b>(j, i)[0] = r;
					img.at<Vec3b>(j, i)[1] = g;
					img.at<Vec3b>(j, i)[2] = b;
				}
			}
		}
	}
	else {
		for (unsigned int i = 0; i < m_extent_x; ++i) {
			for (unsigned int j = 0; j < m_extent_y; ++j) {
				//intepolation output
				unsigned int intensity = (unsigned int)(m_voxels[i][j].getIntensity());
				img.at<Vec3b>(j, i)[0] = intensity;
				img.at<Vec3b>(j, i)[1] = intensity;
				img.at<Vec3b>(j, i)[2] = intensity;
			}
		}
	}

	//// test export image
	////Mat img = Mat::zeros(m_extent_y, m_extent_x, CV_8UC3);
	//if (isBinary) {
	//	for (unsigned int i = 0; i < m_extent_x; ++i) {
	//		for (unsigned int j = 0; j < m_extent_y; ++j) {
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
	//	for (unsigned int i = 0; i < m_extent_x; ++i) {
	//		for (unsigned int j = 0; j < m_extent_y; ++j) {
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

/**
* \brief create voxelgrid from point array
* \param[out] ptVx voxel index of each point locating in
* binary voxelization, not compute voxel intensity
*/
bool VoxelGrid2D::create(const std::vector<cv::Point3f> &pts, std::vector<cv::Point2i> &ptVx, cv::Mat &img)
{
	// validate points
	if (pts.empty()) {
		//DBG_OUT << "error VoxelGrid2D::create->points empty\n";
		return false;
	}
	size_t npts = pts.size();
	// init voxels
	initVoxels();
	// init pt voxel list
	ptVx.resize(npts);
	double inv_len_vx = 1.0f / m_len_vx;
	double inv_len_vy = 1.0f / m_len_vy;
	// set voxel occupation
//#pragma omp parallel for
	for (size_t i = 0; i < npts; ++i) {
		int id_x = int(floor((pts[i].x - m_abs_ox) * inv_len_vx));
		int id_y = int(floor((pts[i].y - m_abs_oy) * inv_len_vy));
		m_voxels[id_x][id_y].setOccupied(true);
		ptVx[i].x = id_x;
		ptVx[i].y = id_y;
		// m_voxels[id_x][id_y].appendPtID(i);
	}
	using namespace cv;
	int r = rand() % 255 + 1;
	int g = rand() % 255 + 1;
	int b = rand() % 255 + 1;
	for (unsigned int i = 0; i < m_extent_x; ++i) {
		for (unsigned int j = 0; j < m_extent_y; ++j) {
			// binary output: if occupied, set color white
			if (m_voxels[i][j].isOccupied()) {
				img.at<Vec3b>(j, i)[0] = r;
				img.at<Vec3b>(j, i)[1] = g;
				img.at<Vec3b>(j, i)[2] = b;
			}
		}
	}
	return true;
}

bool VoxelGrid2D::setExtent(const unsigned int x, const unsigned int y) {

	if (x < 1) {
		//DBG_OUT << "error VoxelGrid2D::setExtent->invalid x\n";
		return false;
	}
	if (y < 1) {
		//DBG_OUT << "error VoxelGrid2D::setExtent->invalid y\n";
		return false;
	}
	m_extent_x = x;
	m_extent_y = y;
	return true;
}


bool VoxelGrid2D::setLenVoxel(const double l) {
	if (l <= 0.0f) {
		//DBG_OUT << "error VoxelGrid2D::setLenVoxel->invalid l\n";
		return false;
	}
	m_len_vx = l;
	m_len_vy = l;
	m_is_voxelSquare = true;
	return true;
}

bool VoxelGrid2D::setLenVoxel(const double lx, const double ly) {

	if (lx <= 0.0f) {
		//DBG_OUT << "error VoxelGrid2D::setLenVoxel->invalid lx\n";
		return false;
	}
	if (ly <= 0.0f) {
		//DBG_OUT << "error VoxelGrid2D::setLenVoxel->invalid ly\n";
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

void VoxelGrid2D::setAbsOrigin(const double ox, const double oy) {
	m_abs_ox = ox;
	m_abs_oy = oy;
}

Voxel2D** VoxelGrid2D::getVoxels() {
	return m_voxels;
}

void VoxelGrid2D::getExtent(unsigned int& x, unsigned int& y) {
	x = m_extent_x;
	y = m_extent_y;
}

unsigned int VoxelGrid2D::getExtentX() const {
	return m_extent_x;
}

unsigned int VoxelGrid2D::getExtentY() const {
	return m_extent_y;
}

unsigned int VoxelGrid2D::getNumCol() const {
	return m_extent_x;
}

unsigned int VoxelGrid2D::getNumRow() const {
	return m_extent_y;
}

double VoxelGrid2D::getLenVoxelX() const {
	return m_len_vx;
}

double VoxelGrid2D::getLenVoxelY() const {
	return m_len_vy;
}

double VoxelGrid2D::getAbsOriginX() const {
	return m_abs_ox;
}

double VoxelGrid2D::getAbsOriginY() const {
	return m_abs_oy;
}

void VoxelGrid2D::getAbsOrigin(double& x, double& y) {
	x = m_abs_ox;
	y = m_abs_oy;
}

bool VoxelGrid2D::isVoxelSquare() const {
	return m_is_voxelSquare;
}

size_t VoxelGrid2D::countOccupied() const {
	size_t n = 0;
	for (size_t i = 0; i < m_extent_x; ++i) {
		for (size_t j = 0; j < m_extent_y; ++j) {
			if (m_voxels[i][j].isOccupied()) {
				++n;
			}
		}
	}
	return n;
}