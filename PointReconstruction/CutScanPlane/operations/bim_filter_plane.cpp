#include "bim_filter_plane.h"
#include "../utility/util_math_near.h"
#include "bim_intxn.h"
#include "bbox.h"

/**
* \brief constructor
*/
BIM_Filter_Plane::BIM_Filter_Plane() {
}
/**
* \brief destructor
*/
BIM_Filter_Plane::~BIM_Filter_Plane() {
}

/**
* \brief filter valid plane
*/
void BIM_Filter_Plane::filter_valid(Doc_BIM &doc_bim) {
	typedef std::pair<unsigned int, unsigned int> pairID;
	typedef std::vector<pairID> tablePairID;
	// get docs of planes
	std::vector<Doc_BIM_Plane *> docs;
	doc_bim.getDocPlane(docs);
	if (docs.empty()) return;
	// get indices for processing
	std::vector<unsigned int> idx_proc;
	doc_bim.getIdxProc(idx_proc);
	// get current status of planes
	std::vector<State_BIM_Plane> states; // state of all planes
	states.resize(docs.size());
	for (size_t i = 0; i < docs.size(); ++i) {
		states[i] = docs[i]->getType();
	}
	// filter criteria
	float thres_area = 1.5f; // m^2
	float dilate = 50.f; // bbox dilate value
						 // filter process
						 // compute all intxn, find isolate plane
	tablePairID pair_intxn;
	BIM_Intxn::compute(docs, idx_proc, dilate, pair_intxn);
	std::vector<bool> isLinked;
	isLinked.resize(docs.size());
	std::fill(isLinked.begin(), isLinked.end(), false);
	for (size_t i = 0; i < pair_intxn.size(); ++i) {
		isLinked[pair_intxn[i].first] = true;
		isLinked[pair_intxn[i].second] = true;
	}
	// filter isolate and area < thres as invalid
	for (size_t i = 0; i < docs.size(); ++i) {
		int idx = docs[i]->getIdxPlane();
		// check if idx of doc exists in processing list
		if (!Util_Math::isIdxInList(idx, idx_proc)) continue;
		// filter invalid based on area: 1.5m^2
		float area = docs[i]->getAreaProjLocal() / 1.0e6f;
		if (isLinked[i] == false && area < thres_area) {
			states[i].removeValid();
		}
		else {
			states[i].appendValid();
		}
	}
	// update state
	for (size_t i = 0; i < docs.size(); ++i) {
		docs[i]->setType(states[i]);
	}
	// clear temp
	isLinked.clear();
	pair_intxn.clear();
	idx_proc.clear();
	states.clear();
}

/**
* \brief filter horizontal plane
*/
void BIM_Filter_Plane::filter_horiz(Doc_BIM& doc_bim) {
	using namespace Util_Math_Near;
	// init data
	std::vector<Doc_BIM_Plane*> docs;
	doc_bim.getDocPlane(docs);
	if (docs.empty()) return;
	std::vector<unsigned int> idx_proc;
	doc_bim.getIdxProc(idx_proc);
	std::vector<State_BIM_Plane> states;
	states.resize(docs.size());
	for (size_t i = 0; i < docs.size(); ++i) {
		states[i] = docs[i]->getType();
	}

	float thres_angle = 10.f; 
	cv::Point3f ez(0.f, 0.f, 1.f); 
	cv::Point3f inv_ez(0.f, 0.f, -1.f);

	// filter process
	for (size_t i = 0; i < docs.size(); ++i) {
		int idx = docs[i]->getIdxPlane();
		cv::Point3f normal = docs[i]->getNormal();
		if (!Util_Math::isIdxInList(idx, idx_proc)) continue;

		normal = Util_Math::vec3_normalize(normal);
		bool is_horiz = states[i].isValid()
			&& (isNearDir(normal, ez, thres_angle) || isNearDir(normal, inv_ez, thres_angle));

		if (is_horiz) {
			states[i].appendHoriz(); 
			states[i].removeVert();
		}
		else {
			states[i].removeHoriz();
		}
	}

	// update state
	for (size_t i = 0; i < docs.size(); ++i) {
		docs[i]->setType(states[i]);
	}
	// clear temp
	idx_proc.clear();
	states.clear();
}
/**
* \brief filter vertical plane
*/
void BIM_Filter_Plane::filter_vert(Doc_BIM &doc_bim) {
	using namespace Util_Math_Near;
	// init data
	// get docs of planes
	std::vector<Doc_BIM_Plane *> docs;
	doc_bim.getDocPlane(docs);
	if (docs.empty()) return;
	// get indices for processing
	std::vector<unsigned int> idx_proc;
	doc_bim.getIdxProc(idx_proc);
	// get current status of planes
	std::vector<State_BIM_Plane> states; // state of all planes
	states.resize(docs.size());
	for (size_t i = 0; i < docs.size(); ++i) {
		states[i] = docs[i]->getType();
	}
	// filter criteria
	float thres_angle = 10.f; // degree
							 // filter process
							 // filter valid, horizontal and vertical
	for (size_t i = 0; i < docs.size(); ++i) {
		int idx = docs[i]->getIdxPlane();
		cv::Point3f normal = docs[i]->getNormal();
		// check if idx of doc exists in processing list
		if (!Util_Math::isIdxInList(idx, idx_proc)) continue;
		if (states[i].isHoriz()) {
			states[i].removeVert();
			continue;
		}

		normal = Util_Math::vec3_normalize(normal);
		// filter vertical plane
		if (states[i].isValid() && isNearDir(normal, cv::Point3f(normal.x, normal.y, 0.f), thres_angle) && fabs(normal.z) < 0.3) {
			states[i].appendVert();
		}
		else {
			states[i].removeVert();
		}
	}
	// update state
	for (size_t i = 0; i < docs.size(); ++i) {
		docs[i]->setType(states[i]);
	}
	// clear temp
	idx_proc.clear();
	states.clear();
}

void BIM_Filter_Plane::filter_ceiling(Doc_BIM& doc_bim) {
	typedef std::pair<unsigned int, unsigned int> pairID;
	typedef std::vector<pairID> tablePairID;
	// data validation
	std::vector<Doc_BIM_Plane*> docs;
	doc_bim.getDocPlane(docs);
	if (docs.empty()) return;
	std::vector<unsigned int> idx_proc;
	doc_bim.getIdxProc(idx_proc);
	// init states of all planes
	std::vector<State_BIM_Plane> states; // state of all planes
	states.resize(docs.size());
	for (size_t i = 0; i < docs.size(); ++i) {
		states[i] = docs[i]->getType();
	}
	float thres_ceiling = 800.f; // m, height threshold, compare with bbox_min.z
	float dilate = 50.f; // bbox dilate value
	// filter process
	std::vector<unsigned int> doc_cand; // candidate

	for (size_t i = 0; i < docs.size(); ++i) {
		int idx = docs[i]->getIdxPlane();
		BBox3D bbox = docs[i]->getBBox();
		// check if idx of doc exists in processing list
		if (!Util_Math::isIdxInList(idx, idx_proc)) continue;
		if (!states[i].isValid()) {
			states[i].removeCeiling();
		}

		// find candidates
		if (states[i].isValid() && bbox.m_min.z > thres_ceiling) {
			doc_cand.push_back(i);
		}
	}

	for (size_t i = 0; i < docs.size(); ++i) {
		if (!Util_Math::isIdxInList(i, doc_cand)) continue;
		if (docs[i]->getType().isHoriz()) {
			states[i].appendCeiling(); 
		}
	}

	tablePairID pair_intxn;
	BIM_Intxn::compute(docs, idx_proc, dilate, pair_intxn);
	// shrink pair_intxn, only contain the pair between candidates
	tablePairID pair_cand;
	for (size_t i = 0; i < pair_intxn.size(); ++i) {
		pairID pair = pair_intxn[i];
		if (Util_Math::isIdxInList(pair.first, doc_cand) && Util_Math::isIdxInList(pair.second, doc_cand)) {
			pair_cand.push_back(pair);
		}
	}
	// iteratively append candidate planes intxn with current ceiling till no ceiling found
	bool hasNewCeiling;
	do
	{
		hasNewCeiling = false;
		for (size_t i = 0; i < pair_cand.size(); ++i) {
			unsigned int id_0, id_1;
			id_0 = pair_cand[i].first;
			id_1 = pair_cand[i].second;
			if (states[id_0].isCeiling() && !states[id_1].isCeiling()) {
				states[id_1].appendCeiling();
				hasNewCeiling = true;
				break;
			}
			if (!states[id_0].isCeiling() && states[id_1].isCeiling()) {
				states[id_0].appendCeiling();
				hasNewCeiling = true;
				break;
			}
		}
		if (hasNewCeiling) continue;
	} while (hasNewCeiling);

	// option: add a step to remove isolate ceiling

	// update state
	for (size_t i = 0; i < docs.size(); ++i) {
		docs[i]->setType(states[i]);
	}
	// clear temp
	pair_intxn.clear();
	pair_cand.clear();
	idx_proc.clear();
	states.clear();
}

/**
* \brief filter ground
*/
void BIM_Filter_Plane::filter_floor(Doc_BIM &doc_bim) {
	// data validation
	std::vector<Doc_BIM_Plane *> docs;
	doc_bim.getDocPlane(docs);
	if (docs.empty()) return;
	std::vector<unsigned int> idx_proc;
	doc_bim.getIdxProc(idx_proc);
	// init states of all planes
	std::vector<State_BIM_Plane> states; // state of all planes
	states.resize(docs.size());
	for (size_t i = 0; i < docs.size(); ++i) {
		states[i] = docs[i]->getType();
	}
	// criteria
	//float thres_floor = -800.f;
	//changed by yu.liang@unre.com start
	float thres_floor = -600.0f;
	int nfloor = -1;
	float max_floor_area = -1.0f;

	BBox3D scanCenter(cv::Point3f(-300, -300, -2000), cv::Point3f(300, 300, 1000));
	for (size_t i = 0; i < docs.size(); ++i) {
		if (states[i].isValid() && states[i].isHoriz()
			&& docs[i]->getBBox().m_max.z < thres_floor && docs[i]->getBBox().isOverlap(scanCenter))
		{
			if (docs[i]->getAreaProjLocal() > max_floor_area)
			{
				max_floor_area = docs[i]->getAreaProjLocal();
				nfloor = i;
			}
		}
	}
	if (nfloor == -1)
	{
		for (size_t i = 0; i < docs.size(); ++i) {
			if (states[i].isValid() && states[i].isHoriz()
				&& docs[i]->getBBox().m_max.z < thres_floor)
			{
				float dis_center_squar = docs[i]->getCenter().x * docs[i]->getCenter().x + docs[i]->getCenter().y*docs[i]->getCenter().y;
				if (docs[i]->getAreaProjLocal() * (1.0f/(1+ dis_center_squar)) > max_floor_area)
				{
					max_floor_area = docs[i]->getAreaProjLocal();
					nfloor = i;
				}
			}
		}
	}
	thres_floor = docs[nfloor]->getBBox().m_max.z + 100.f;
	//changed by yu.liang@unre.com end
	// filter process
	// filter ground
	for (size_t i = 0; i < docs.size(); ++i) {
		int idx = docs[i]->getIdxPlane();
		BBox3D bbox = docs[i]->getBBox();
		cv::Point3f normal = docs[i]->getNormal();
		// check if idx of doc exists in processing list
		if (!isIdxInList(idx, idx_proc)) continue;
		// filter vertical plane
		if (states[i].isValid() && states[i].isHoriz()
			&& bbox.m_max.z < thres_floor)
		{
			states[i].appendFloor();
		}
		else {
			states[i].removeFloor();
		}
	}
	// update state
	for (size_t i = 0; i < docs.size(); ++i) {
		docs[i]->setType(states[i]);
	}
	// clear temp
	idx_proc.clear();
	states.clear();
}

/**
* \brief filter wall
*
* find plane intxn with union of ceiling and union of floor at same time
* candidate planes: vertical, not ceiling or floor
*/
void BIM_Filter_Plane::filter_wall(Doc_BIM &doc_bim) {
	// data validation
	std::vector<Doc_BIM_Plane *> docs;
	doc_bim.getDocPlane(docs);
	if (docs.empty()) return;
	std::vector<unsigned int> idx_proc;
	doc_bim.getIdxProc(idx_proc);
	// init states of all planes
	std::vector<State_BIM_Plane> states; // state of all planes
	states.resize(docs.size());
	for (size_t i = 0; i < docs.size(); ++i) {
		states[i] = docs[i]->getType();
	}
	// compute union bbox of ceiling and floor
	BBox3D bbox_ceiling;
	BBox3D bbox_floor;
	for (size_t i = 0; i < docs.size(); ++i) {
		if (states[i].isCeiling()) {
			bbox_ceiling = bbox_ceiling + docs[i]->getBBox();
		}
		if (states[i].isFloor()) {
			bbox_floor = bbox_floor + docs[i]->getBBox();
		}
	}
	float thres_dis = 50.f;
	float dilate = 50.f;
	float h_ceiling = bbox_ceiling.m_min.z - thres_dis;
	float h_floor = bbox_floor.m_max.z + thres_dis;

	bbox_ceiling.dilate(dilate);
	bbox_floor.dilate(dilate);
	for (size_t i = 0; i < docs.size(); ++i) {
		if (!states[i].isVert() || states[i].isCeiling() || states[i].isFloor())
		{
			states[i].removeWall();
			continue;
		}
		BBox3D bbox = docs[i]->getBBox();
		float wall_length = bbox.m_max.z - bbox.m_min.z;
		// dilate bbox
		bbox.dilate(dilate);
		// cond 1: intnx with bbox
		if (bbox.isOverlap(bbox_ceiling) && bbox.isOverlap(bbox_floor)) {
			states[i].appendWall();
		}
		// cond 2: only consider length of wall
		if (wall_length > 1000.0f) {
			if (bbox.isOverlap(bbox_ceiling) || bbox.isOverlap(bbox_floor)) {
				states[i].appendWall();
			}
		}

	}
	// update state
	for (size_t i = 0; i < docs.size(); ++i) {
		docs[i]->setType(states[i]);
	}
	// clear temp
	idx_proc.clear();
	states.clear();
}

/**
* \brief filter beam
* yu.liang@unre.com
* find plane intxn with union of ceiling and vertical planes
* candidate planes: vertical, not wall
*/
void BIM_Filter_Plane::filter_beam(Doc_BIM &doc_bim) {
	using namespace Util_Math_Near;
	// data validation
	std::vector<Doc_BIM_Plane *> docs;
	doc_bim.getDocPlane(docs);
	if (docs.empty()) return;
	std::vector<unsigned int> idx_proc;
	doc_bim.getIdxProc(idx_proc);
	// init states of all planes
	std::vector<State_BIM_Plane> states; // state of all planes
	states.resize(docs.size());
	for (size_t i = 0; i < docs.size(); ++i) {
		states[i] = docs[i]->getType();
	}

	float thres_angle = 5.f;
	for (size_t i = 0; i < docs.size(); ++i) {
		//if (states[i].isCeiling() && !states[i].isHoriz())
		cv::Point3f normal = docs[i]->getNormal();
		if (states[i].isCeiling() && isNearDir(normal, cv::Point3f(normal.x, normal.y, 0.f), thres_angle) && fabs(normal.x + normal.y) > 0.3)
		{
			states[i].appendBeam();
			states[i].removeCeiling();
		}
		else
		{
			states[i].removeBeam();
		}
	}
	// update state
	for (size_t i = 0; i < docs.size(); ++i) {
		docs[i]->setType(states[i]);
	}
	// clear temp
	idx_proc.clear();
	states.clear();
}