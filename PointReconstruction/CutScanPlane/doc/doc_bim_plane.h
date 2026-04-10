#ifndef _DOC_BIM_PLANE_H_
#define _DOC_BIM_PLANE_H_

#include "doc_pc_plane.h"
#include "state_bim.h"

/**
* \brief document of BIM point cloud plane
* \author Bichen JING
*/
class Doc_BIM_Plane : public Doc_PC_Plane {
public:
	/**
	* \brief constructor
	*/
	Doc_BIM_Plane() {
	}
	/**
	* \brief destructor
	*/
	virtual ~Doc_BIM_Plane() {
		clear();
	}
	/**
	* \brief virtual function, clear data
	*/
	virtual void clear() {

	}
	/**
	* \brief set type
	*/
	void setType(const State_BIM_Plane type) {
		m_type = type;
	}
	/**
	* \brief get type
	*/
	State_BIM_Plane getType() {
		return m_type;
	}
protected:
	State_BIM_Plane m_type;	/**< type of bim plane */
};

//	// test doc
//	std::vector<Model_PC*> model_set = g_modelManager->get_model_set();
//	std::vector<Doc_BIM_Plane*> doc_bim_plane;
//	for (size_t i = 0; i < model_set.size(); i++) {
//		if (model_set[i] == nullptr) continue;
//	
//		size_t npts = model_set[i]->getPointArray()->size;
//		// copy data to list
//		PointItem *points = model_set[i]->getPointArray()->points;
//		std::vector<Point3f> pts;
//		pts.resize(npts);
//#pragma omp parallel for
//		for (size_t i = 0; i < npts; i++) {
//			pts[i] = points[i].point;
//		}
//
//		Doc_BIM_Plane *doc = new Doc_BIM_Plane();
//		doc->setIdx(i);
//		doc->setIdxPlane(i);
//
//		doc->updateBBox(pts);
//		doc->updateCoefs(pts);
//
//		doc_bim_plane.push_back(doc);
//
//		pts.clear();
//	}
//
//	for (size_t i = 0; i < doc_bim_plane.size(); i++) {
//		Doc_BIM_Plane *doc = doc_bim_plane[i];
//		DBG_OUT << "plane " << doc->getIdxPlane() << " normal("
//			<< doc->getNormal().x << ", "
//			<< doc->getNormal().y << ", "
//			<< doc->getNormal().z << ")";
//	}
//
//	for (size_t i = 0; i < doc_bim_plane.size(); i++) {
//		if (doc_bim_plane[i] != nullptr) {
//			Doc_BIM_Plane *temp = doc_bim_plane[i];
//			doc_bim_plane[i] = nullptr;
//			delete temp;
//		}
//	}
//	doc_bim_plane.clear();

#endif // !_DOC_BIM_PLANE_H_
