#ifndef _BIM_CLBR_H_
#define _BIM_CLBR_H_
#include "util_math.hpp"
#include "../utility/util_math_near.h"
#include "../doc/doc_bim.h"

/**
* \brief class of BIM calibration
*/
class BIM_Clbr {
public:
	/**
	* \brief orientation of UP direction
	*/
	enum class ORIEN_UP
	{
		AXIS_X,
		AXIS_Y,
		AXIS_Z
	};
	/**
	* \brief constructor
	*/
	BIM_Clbr() 
		: m_orien_up(ORIEN_UP::AXIS_Z)
		, m_mat_rot(cv::Mat::eye(3, 3, CV_32F))
	{
	}
	/**
	* \brief destructor
	*/
	~BIM_Clbr() {

	}

	/**
	* \brief process
	*/
	void compute(Doc_BIM &doc_bim) {
		// data validation
		std::vector<Doc_BIM_Plane *> docs;
		doc_bim.getDocPlane(docs);
		if (docs.empty()) return;
		// step 1: prepare data
		// set orien_up, orien_horiz vector
		cv::Point3f ex(1.f, 0.f, 0.f);
		cv::Point3f ey(0.f, 1.f, 0.f);
		cv::Point3f ez(0.f, 0.f, 1.f);
		cv::Point3f e_up(0.f, 0.f, 1.f);
		cv::Point3f e_h(0.f, 1.f, 0.f);
		switch (m_orien_up)
		{
		case ORIEN_UP::AXIS_X: e_up = ex; e_h = ez; break;
		case ORIEN_UP::AXIS_Y: e_up = ey; e_h = ez; break;
		case ORIEN_UP::AXIS_Z: e_up = ez; e_h = ey; break;
		default:
			break;
		}
		// find normals of floor, wall, vertical
		std::vector<cv::Point3f> norm_floor;
		std::vector<cv::Point3f> norm_wall;
		std::vector<cv::Point3f> norm_vert;
		for (size_t i = 0; i < docs.size(); ++i) {
			auto state = docs[i]->getType();
			if (!state.isValid()) continue;
			if (state.isFloor()) {
				norm_floor.push_back(docs[i]->getNormal());
			}
			if (state.isWall()) {
				norm_wall.push_back(docs[i]->getNormal());
				//cv::Point3f norm = docs[i]->getNormal();
				//cout << "norm_wall: " << norm.x << ", " << norm.y << ", " << norm.z;
			}
			if (state.isVert()) {
				norm_vert.push_back(docs[i]->getNormal());
			}
		}
		if (norm_floor.empty()) return;
		// step 2: rotate norm_up to e_up
		cv::Point3f norm_up(0.f, 0.f, 0.f);
		// refine orientation and sum
		for (size_t i = 0; i < norm_floor.size(); ++i) {
			cv::Point3f normal = norm_floor[i];
			if (Util_Math::vec3_angle_deg(normal, e_up) > 90) {
				norm_floor[i] = -normal;
			}
			norm_up += norm_floor[i];
		}
		norm_up *= (1.f / norm_floor.size());
		norm_up = Util_Math::vec3_normalize(norm_up);
		// DBG_OUT << norm_up.x << ", " << norm_up.y << ", " << norm_up.z;

		// compute rotation from norm_up to e_up
		using namespace Util_Math;
		cv::Mat_<float> rot_up = cv::Mat::eye(3, 3, CV_32F);
		get_rot_mat3(norm_up.cross(e_up), vec3_angle(norm_up, e_up), rot_up);
		
		// rotate wall and vert normals
		mat3_mult_pt3<cv::Mat_<float>, cv::Point3f, float>(rot_up, norm_wall);
		mat3_mult_pt3<cv::Mat_<float>, cv::Point3f, float>(rot_up, norm_vert);
		//for (size_t i = 0; i < norm_wall.size(); ++i) {
		//	cv::Point3f norm = norm_wall[i];
		//	DBG_OUT << norm.x << ", " << norm.y << ", " << norm.z;
		//}
		
		// step 3: compute rotation from normal of max cluster of wall/vertical plane to one horiz axis 
		cv::Mat_<float> rot_h = cv::Mat::eye(3, 3, CV_32F);
		cv::Point3f norm_h = e_h;
		// cluster normals and find the avg normal of max cluster
		float thres_deg = 5.f;
		bool hasVertPlane = false;
		std::vector<std::vector<unsigned int>> clusters;
		if (!norm_wall.empty()) {
			hasVertPlane = true;
			Util_Math_Near::clusterVecs(norm_wall, thres_deg, clusters);
			cv::Point3f avg(0.f, 0.f, 0.f);
			for (size_t i = 0; i < clusters[0].size(); ++i) {
				avg += norm_wall[clusters[0][i]];
			}
			avg *= (1.f / clusters[0].size());
			norm_h = vec3_normalize(avg);
		}
		else {
			if (!norm_vert.empty()) {
				hasVertPlane = true;
				Util_Math_Near::clusterVecs(norm_vert, thres_deg, clusters);
				cv::Point3f avg(0.f, 0.f, 0.f);
				for (size_t i = 0; i < clusters[0].size(); ++i) {
					avg += norm_vert[clusters[0][i]];
				}
				avg *= (1.f / clusters[0].size());
				norm_h = vec3_normalize(avg);
			}
		}
		// DBG_OUT << "norm_h: " << norm_h.x << ", " << norm_h.y << ", " << norm_h.z;
		// compute rotation around up
		if (hasVertPlane) {
			// only consider horizontal portion of norm_h for rotation
			norm_h = vec3_normalize(proj_vec3ToPlane(norm_h, e_up));
			get_rot_mat3(norm_h.cross(e_h), vec3_angle(norm_h, e_h), rot_h);
		}
		// DBG_OUT << "norm_h: " << norm_h.x << ", " << norm_h.y << ", " << norm_h.z;

		m_mat_rot = rot_h * rot_up;

		//DBG_OUT << m_mat_rot.at<float>(0, 0) << " " << m_mat_rot.at<float>(0, 1) << " " << m_mat_rot.at<float>(0, 2);
		//DBG_OUT << m_mat_rot.at<float>(1, 0) << " " << m_mat_rot.at<float>(1, 1) << " " << m_mat_rot.at<float>(1, 2);
		//DBG_OUT << m_mat_rot.at<float>(2, 0) << " " << m_mat_rot.at<float>(2, 1) << " " << m_mat_rot.at<float>(2, 2);

		// clear
		docs.clear();
		norm_floor.clear();
		norm_wall.clear();
		norm_vert.clear();
		for (size_t i = 0; i < clusters.size(); ++i) {
			clusters[i].clear();
		}
		clusters.clear();
	}
	/**
	* \brief get rotation matrix
	*/
	cv::Mat_<float> getMatRot() const {
		return m_mat_rot;
	}
private:
	ORIEN_UP m_orien_up;		/**< up orientation */
	cv::Mat_<float> m_mat_rot;	/**< rotation matrix */
};

#endif // !_BIM_CLBR_H_
