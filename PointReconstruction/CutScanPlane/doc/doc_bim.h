#ifndef _DOC_BIM_H_
#define _DOC_BIM_H_

#include "doc.h"
#include <vector>
#include <utility>
#include <opencv2\core\core.hpp>
#include "doc_bim_plane.h"
#include "../topology/topo_graph.h"
#include "../../PlaneCuttingInterface.h"

/**
* \brief class of BIM document
* \author Bichen JING
*/
class Doc_BIM : public Doc
{
public:
	typedef std::pair<unsigned int, unsigned int> pairID;
	typedef std::vector<pairID> tablePairID;
	typedef std::vector<Doc_BIM_Plane *> tableDocPlane;
	/**
	* \brief constructor
	*/
	Doc_BIM()
		: m_topo_graph(nullptr)
		, m_mat_clbr(cv::Mat::eye(3, 3, CV_32F))
		, m_vec_clbr(cv::Point3f(0.f, 0.f, 0.f))
	{
	}
	/**
	* \brief destructor
	*/
	virtual ~Doc_BIM() {
		clear();
	}
	/**
	* \brief clear
	*/
	void clear() {
		clearPairIntxn();
		clearDocPlane();
		clearTopoGraph();
		clearIdxProc();
		clearClbr();
	}
	/**
	* \brief clear doc plane
	*/
	void clearDocPlane() {
		for (size_t i = 0; i < m_doc_plane.size(); i++) {
			Doc_BIM_Plane *temp = m_doc_plane[i];
			m_doc_plane[i] = nullptr;
			if (temp != nullptr) delete temp;
		}
		m_doc_plane.clear();
	}
	/**
	* \brief clear pair intersection
	*/
	void clearPairIntxn() {
		m_pair_intxn.clear();
	}
	/**
	* \brief clear topology graph
	*/
	void clearTopoGraph() {
		if (m_topo_graph != nullptr) {
			Topo_Graph *temp = m_topo_graph;
			m_topo_graph = nullptr;
			delete temp;
		}
	}
	/**
	* \brief clear idx list for processing
	*/
	void clearIdxProc() {
		m_idx_proc.clear();
	}
	/**
	* \brief clear calibration
	*/
	void clearClbr() {
		m_mat_clbr = cv::Mat::eye(3, 3, CV_32F);
		m_vec_clbr = cv::Point3f(0.f, 0.f, 0.f);
	}
	/**
	* \brief update
	*/
	void update() {
	}
	/**
	* \brief get all indices
	*/
	void getIdxAll(std::vector<unsigned int> &idxs) {
		idxs.clear();
		for (size_t i = 0; i < m_doc_plane.size(); ++i) {
			idxs.push_back(m_doc_plane[i]->getIdxPlane());
		}
	}
	/**
	* \brief get index list of given type
	*/
	void getIdxList(const ATTR_BIM_PLANE type, std::vector<unsigned int> &idxs) {
		idxs.clear();
		for (size_t i = 0; i < m_doc_plane.size(); ++i) {
			if (m_doc_plane[i]->getType().opAND(type)) {
				idxs.push_back(m_doc_plane[i]->getIdxPlane());
			}
		}
	}
	/**
	* \brief get index list of given type list, fulfill all type requirement
	*/
	void getIdxList(const std::vector<ATTR_BIM_PLANE> &types, std::vector<unsigned int> &idxs) {
		idxs.clear();
		for (size_t i = 0; i < m_doc_plane.size(); ++i) {
			State_BIM_Plane pType = m_doc_plane[i]->getType();
			bool isTrue = true;
			for (size_t k = 0; k < types.size(); ++k) {
				isTrue = isTrue && pType.opAND(types[k]);
			}
			if (isTrue) {
				idxs.push_back(m_doc_plane[i]->getIdxPlane());
			}
		}
	}
	/**
	* \brief set table of intersection pairs
	*/
	void setPairIntxn(const tablePairID &pair_intxn) {
		m_pair_intxn = pair_intxn;
	}
	/**
	* \brief get table of intersection pairs
	*/
	void getPairIntxn(tablePairID &pair_intxn) {
		pair_intxn = m_pair_intxn;
	}
	/**
	* \brief append intersection pairs
	*/
	void appendPairIntxn(const unsigned int id_0, const unsigned int id_1) {
		// data validation
		if (id_0 == id_1) return;
		pairID temp = std::make_pair(MIN(id_0, id_1), MAX(id_0, id_1));
		// check if exist
		bool isExist = false;
		for (size_t i = 0; i < m_pair_intxn.size(); i++) {
			if (temp == m_pair_intxn[i]) {
				isExist = true;
				break;
			}
		}
		if (!isExist) {
			m_pair_intxn.push_back(temp);
		}
	}
	/**
	* \brief get document of planes
	*/
	void getDocPlane(tableDocPlane &doc_plane) {
		doc_plane = m_doc_plane;
	}
	/**
	* \brief append document of planes
	*/
	void appendDocPlane(Doc_BIM_Plane *doc_plane) {
		if (doc_plane != nullptr) {
			m_doc_plane.push_back(doc_plane);
		}
	}
	/**
	* \brief set topology graph
	*/
	void setTopoGraph(Topo_Graph *topo_graph) {
		if (m_topo_graph != nullptr) {
			Topo_Graph *temp = m_topo_graph;
			m_topo_graph = nullptr;
			delete temp;
		}
		m_topo_graph = topo_graph;
	}
	/**
	* \brief get topology graph
	*/
	Topo_Graph *getTopoGraph() {
		return m_topo_graph;
	}
	/**
	* \brief set idx list for further processing
	*/
	void setIdxProc(const std::vector<unsigned int> &idx_proc) {
		m_idx_proc = idx_proc;
	}
	/**
	* \brief get idx list for further processing
	*/
	void getIdxProc(std::vector<unsigned int> &idx_proc) {
		idx_proc = m_idx_proc;
	}
	/**
	* \brief set calibration matrix
	*/
	void setMatClbr(const cv::Mat_<float> &mat_clbr) {
		m_mat_clbr = mat_clbr;
	}
	/**
	* \brief get calibration matrix
	*/
	void getMatClbr(cv::Mat_<float> &mat_clbr) {
		mat_clbr = m_mat_clbr;
	}
	/**
	* \brief set calibration vector
	*/
	void setVecClbr(const cv::Point3f &vec_clbr) {
		m_vec_clbr = vec_clbr;
	}
	/**
	* \brief get calibration vector
	*/
	void getVecClbr(cv::Point3f &vec_clbr) {
		vec_clbr = m_vec_clbr;
	}

protected:
	tablePairID m_pair_intxn;	/**< table of intersection pairs */
	tableDocPlane m_doc_plane;	/**< table of plane docs */
	Topo_Graph *m_topo_graph;	/**< topology graph */
	std::vector<unsigned int> m_idx_proc;	/**< idx list for further processing */
	cv::Mat_<float> m_mat_clbr;	/**< calibration matrix, rotation */
	cv::Point3f m_vec_clbr;		/**< calibration vector, translation */
};

#endif // !_DOC_BIM_H_
