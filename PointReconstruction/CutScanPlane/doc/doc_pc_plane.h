#ifndef _DOC_PC_PLANE_H_
#define _DOC_PC_PLANE_H_

#include "doc_pc.h"
#include <utility>
#include <Eigen\Geometry>
#include <Eigen\Eigenvalues> 
#include <opencv2\opencv.hpp>
#include "../utility/util_pca.h"
#include "../utility/util_math_near.h"
#include "../models/proj2d.h"
#include "MathOperation.hpp"

/**
* \brief document of point cloud plane
* \author Bichen JING
*/
class Doc_PC_Plane : public Doc_PC {
public:
	typedef Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 3, 3>> EigenSolver;
	/**
	* \brief constructor
	*/
	Doc_PC_Plane() 
		: m_idx_plane(-1)
		, m_normal(cv::Point3f(0.f, 0.f, 1.f))
		, m_proj_local(nullptr)
		, m_area_proj_local(0.f)
		, m_img_proj_local(cv::Mat::zeros(1, 1, CV_8UC3))
		, m_img_proj_global(cv::Mat::zeros(1, 1, CV_8UC3))
	{
		clearCoefs();
	}
	/**
	* \brief destructor
	*/
	virtual ~Doc_PC_Plane() {
		clear();
		m_img_proj_local.release();
		m_img_proj_global.release();
	}
	/**
	* \brief virtual function, clear data
	*/
	virtual void clear() {
		m_idx_plane = -1;
		m_normal = cv::Point3f(0.f, 0.f, 1.f);
		clearCoefs();
		m_area_proj_local = 0.f;
		clearProj();
		clearPtVx2();
	}
	/**
	* \brief clear coefficients
	*/
	void clearCoefs() {
		m_coefs.resize(4);
		for (int i = 0; i < 4; ++i) {
			m_coefs[i] = 0;
		}
	}
	/**
	* \brief clear projection
	*/
	void clearProj() {
		if (m_proj_local == nullptr) return;
		Proj2D *temp = m_proj_local;
		m_proj_local = nullptr;
		delete temp;
	}
	/**
	* \brief clear 2d voxel index of point
	*/
	void clearPtVx2() {
		m_ptVx2.clear();
	}
	/**
	* \brief virtual function, update model
	*/
	virtual void update() {

	}
	/**
	* \brief update plane coefficients
	*/
	bool updateCoefs(const std::vector<cv::Point3f> &pts) {
		// data validation
		if (pts.empty()) return false;
		updateCenter(pts);
		// compute pca and eigen
		unsigned int step_sample = pts.size() / 10000; // to accelerate speed

		//EigenSolver eigenSolver;
		//Util_PCA::pca3(pts, eigenSolver);
		//float l3 = eigenSolver.eigenvalues()[0];
		//float nx = eigenSolver.eigenvectors()(0, 0);
		//float ny = eigenSolver.eigenvectors()(1, 0);
		//float nz = eigenSolver.eigenvectors()(2, 0);		

		cv::PCA pca;
		Util_PCA::pca3(pts, step_sample, pca);
		// Util_PCA::pca3(pts, pca);
		
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
		// compute plane function
		float a = pca.eigenvectors.at<float>(id_min_eig, 0);
		float b = pca.eigenvectors.at<float>(id_min_eig, 1);
		float c = pca.eigenvectors.at<float>(id_min_eig, 2);
		float len_n = sqrt(a * a + b * b + c * c);
		a = a / len_n; b = b / len_n; c = c / len_n;
		float avg_x = m_center.x, avg_y = m_center.y, avg_z = m_center.z;
		// compute d
		float d = -(a * avg_x + b * avg_y + c * avg_z);
		// update coefficients
		m_coefs.resize(4);
		m_coefs[0] = a;
		m_coefs[1] = b;
		m_coefs[2] = c;
		m_coefs[3] = d;
		// update normal
		m_normal = cv::Point3f(a, b, c);
		return true;
	}

	bool updateCoefsRANSAC(std::vector<cv::Point3f> &pts) {
		// data validation
		if (pts.empty()) return false;
		std::vector<cv::Point3f> empty_vector;
		MathOperation::GetNewNormalAndCenter(pts, m_center, m_normal, empty_vector);
		
		return true;
	}
	/**
	* \brief compute intersection of two planes, return two points on the intersection line
	*/
	static bool calIntxn_2d(
		const std::vector<float> &coefs_0, const std::vector<float> &coefs_1, 
		const unsigned int axis, cv::Point2f &pt) 
	{
		// data validation
		if (coefs_0.size() < 4 || coefs_1.size() < 4) return false;
		// check parallel
		cv::Point3f n0(coefs_0[0], coefs_0[1], coefs_0[2]);
		cv::Point3f n1(coefs_1[0], coefs_1[1], coefs_1[2]);
		if (Util_Math_Near::isNearParl(n0, n1, 10)) return false;
		// prepare matrix
		cv::Mat A = cv::Mat::zeros(2, 2, CV_32F);
		A.at<float>(0, 0) = coefs_0[0];
		A.at<float>(0, 1) = coefs_0[1];
		A.at<float>(1, 0) = coefs_1[0];
		A.at<float>(1, 1) = coefs_1[1];
		cv::Mat b = cv::Mat::zeros(2, 1, CV_32F);
		b.at<float>(0, 0) = -coefs_0[3];
		b.at<float>(1, 0) = -coefs_1[3];
		// solve linear system
		cv::Mat invAb = A.inv() * b;
		pt.x = invAb.at<float>(0, 0);
		pt.y = invAb.at<float>(1, 0);
		// release
		A.release();
		b.release();
		invAb.release();
		return true;
	}
	/**
	* \brief update projection
	*/
	bool updateProjLocal(const std::vector<cv::Point3f> &pts) {
		// data validation
		if (pts.empty()) return false;
		// project
		Proj2D *proj = new Proj2D();
		proj->process(pts, true, m_img_proj_local);
		setProjLocal(proj);
		// compute local area
		m_area_proj_local = proj->getAreaOccupied();
		return true;
	}
	/**
	* \brief set index of plane
	*/
	void setIdxPlane(const int idx_plane) {
		m_idx_plane = idx_plane;
	}
	/**
	* \brief get index of plane
	*/
	int getIdxPlane() const {
		return m_idx_plane;
	}
	/**
	* \brief set normal
	*/
	void setNormal(const cv::Point3f normal) {
		m_normal = normal;
	}
	/**
	* \brief get normal
	*/
	cv::Point3f getNormal() const {
		return m_normal;
	}
	/**
	* \brief get coefficients
	*/
	bool getCoefs(std::vector<float> &coefs) {
		if (m_coefs.size() < 4) return false;
		coefs.resize(4);
		for (int i = 0; i < 4; i++) {
			coefs[i] = m_coefs[i];
		}
		return true;
	}
	/**
	* \brief set local projection
	*/
	void setProjLocal(Proj2D *proj_local) {
		if (m_proj_local != nullptr) {
			Proj2D *temp = m_proj_local;
			m_proj_local = proj_local;
			delete temp;
		}
		m_proj_local = proj_local;
	}
	/**
	* \brief get local projection
	*/
	Proj2D *getProjLocal() {
		return m_proj_local;
	}
	/**
	* \brief get local projection area
	*/
	float getAreaProjLocal() const {
		return m_area_proj_local;
	}
	/**
	* \brief get local projection image
	*/
	void getImgProjLocal(cv::Mat &img_proj_local) {
		img_proj_local = m_img_proj_local.clone();
	}
	/**
	* \brief set global projection image
	*/
	void setImgProjGlobal(const cv::Mat &img_proj_global) {
		m_img_proj_global = img_proj_global;
	}
	/**
	* \brief get global projection image
	*/
	void getImgProjGlobal(cv::Mat &img_proj_global) {
		img_proj_global = m_img_proj_global.clone();
	}
	/**
	* \brief set 2d voxel index of point
	*/
	void setPtVx2(const std::vector<cv::Point2i> &ptVx2) {
		m_ptVx2 = ptVx2;
	}
	/**
	* \brief get 2d voxel index of point
	*/
	const std::vector<cv::Point2i> &getPtVx2() const{
		return m_ptVx2;
	}
protected:
	int m_idx_plane;				/**< id of plane */
	cv::Point3f m_normal;			/**< normal of plane */
	std::vector<float> m_coefs;		/**< plane function coefficients */
	Proj2D *m_proj_local;			/**< local projection of plane */
	float m_area_proj_local;		/**< area of local projection */
	std::vector<cv::Point2i> m_ptVx2;	/**< 2d voxel index of point */
	cv::Mat m_img_proj_local;		/**< image of local projection */
	cv::Mat m_img_proj_global;		/**< image of global projection */
};

#endif // !_DOC_PC_PLANE_H_
