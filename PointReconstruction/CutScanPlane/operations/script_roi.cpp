#include <set>
#include "script_roi.h"
#include "bim_filter_plane.h"
#include "bim_clbr.h"
#include "bim_intxn.h"
#include "bim_loc_roi.h"
#include "log.h"
#include "MathOperation.hpp"
#include "InOutData.h"
#include "util_time.hpp"

/**
* \brief print information
*/
int MAX_G_IDX = -1, MAX_C_IDX = -1;
inline void printDocPlane(Doc_BIM_Plane *doc) {
	if (doc == nullptr) return;
	int idx = doc->getIdxPlane();
	cv::Point3f normal = doc->getNormal();
	cv::Point3f center = doc->getCenter();
	BBox3D bbox = doc->getBBox();
	float area = doc->getAreaProjLocal() / 1.0e6f;
	std::vector<float> coefs;
	doc->getCoefs(coefs);
	cout
		<< "ID: " << idx << "\n"
		<< "Normal: (" << normal.x << ", " << normal.y << ", " << normal.z << ")" << "\n"
		<< "Center: (" << center.x << ", " << center.y << ", " << normal.z << ")" << "\n"
		<< "BBox_Min: (" << bbox.m_min.x << ", " << bbox.m_min.y << ", " << bbox.m_min.z << ")" << "\n"
		<< "BBox_Max: (" << bbox.m_max.x << ", " << bbox.m_max.y << ", " << bbox.m_max.z << ")" << "\n"
		<< "Area: " << area << "\n"
		<< "Coefs: " << coefs[0] << "x + " << coefs[1] << "y + " << coefs[2] << "z + " << coefs[3] << " = 0\n";
}

Script_ROI::Script_ROI()
{
}

Script_ROI::~Script_ROI()
{
}

void Script_ROI::fillPlaneAreaFromDocs(PlaneCutResultInterface& cutResult)
{
	std::vector<Doc_BIM_Plane*> docs;
	m_doc_bim.getDocPlane(docs);

	cutResult.plane_area.resize(docs.size());

	for (size_t i = 0; i < docs.size(); ++i)
	{
		cutResult.plane_area[i] = docs[i]->getAreaProjLocal();
	}
}


/**
* \brief process
//changed by yu.liang@unre.com
//2020/12/23
*/
bool Script_ROI::process(const TYPE_ROI type_roi, PlaneCutResultInterface& cutResult,int station_size, bool cut_poly)
{
	createDoc(cutResult.plane_xyz);
	fillPlaneAreaFromDocs(cutResult);
	filterPlane();
	calibration(cutResult.plane_xyz);
	fillPlaneAreaFromDocs(cutResult);
	filterPlane();

	TIMING_DECLARE(X);
	OUT_TIMING_BEGIN(X);

	locROI(type_roi, cutResult, station_size,cut_poly);

//	if (!cut_poly) {
//
//#if 1
//		std::cout << "after ROI ground size: " << cutResult.plane_ground_idx.size() << std::endl;
//		std::cout << "after ROI ceiling size: " << cutResult.plane_ceiling_idx.size() << std::endl;
//		std::cout << "after ROI wall size: " << cutResult.plane_wall_idx.size() << std::endl;
//		std::cout << "after ROI beam size: " << cutResult.plane_beam_idx.size() << std::endl;
//#endif
//	}
//	else {
//		m_isSuccess = true;
//	}

	OUT_TIMING_END_ms("locROI", X);
	
	
	if (!m_isSuccess) return m_isSuccess;	

	inverseClbr(cutResult);

	std::vector<cv::Point3f> fitplane;
	for (size_t i = 0; i < cutResult.plane_xyz.size(); i++)
	{
		if (cutResult.plane_xyz[i].size() == 0) continue;
		MathOperation::GetNewNormalAndCenter(cutResult.plane_xyz[i], cutResult.plane_center[i], cutResult.plane_normals[i], fitplane);
	}

	log_debug("[LOCROI]'s bool is: %d",static_cast<int> (m_isSuccess));
	return m_isSuccess;
}
/**
* \brief create document
*/
void Script_ROI::createDoc(std::vector<std::vector<cv::Point3f>> &pts)
{
	// data validation
	if (pts.empty())
	{
		m_isSuccess = false;
		return;
	}
	// create document for BIM
	std::vector<unsigned int> idx_proc;
	//std::cout << "createDoc pts size: " << pts.size() << "\n";
	for (size_t i = 0; i < pts.size(); i++) {
		/*std::cout << "pts: " << i << "\n";
		std::cout << "pts size: " << pts[i].size() << "\n";*/
		// init doc plane
		Doc_BIM_Plane *doc_plane = new Doc_BIM_Plane();
		//std::cout << "Doc_BIM_Plane" << "\n";
		doc_plane->setIdx(i);
		//std::cout << "setIdx" << "\n";
		doc_plane->setIdxPlane(i);
		//std::cout << "setIdxPlane" << "\n";
		doc_plane->updateBBox(pts[i]);
		//std::cout << "updateBBox" << "\n";
		// doc_plane->updateCenter(pts); // computed in coefs
		doc_plane->updateCoefsRANSAC(pts[i]);
		//std::cout << "updateCoefsRANSAC" << "\n";
		doc_plane->updateProjLocal(pts[i]);
		//std::cout << "updateProjLocal" << "\n";
		m_doc_bim.appendDocPlane(doc_plane);
		//std::cout << "appendDocPlane" << "\n";
		// append plane id to idx list for further processing
		idx_proc.push_back(static_cast<unsigned int>(i));
	}
	m_doc_bim.setIdxProc(idx_proc);
	idx_proc.clear();
}
/**
* \brief update document
*/
void Script_ROI::updateDoc(std::vector<std::vector<cv::Point3f>> &pts)
{
	// data validation
	bool isValid = true;
	if (pts.empty()) isValid = false;
	std::vector<Doc_BIM_Plane *> docs;
	m_doc_bim.getDocPlane(docs);
	if (pts.size() != docs.size()) isValid = false;
	if (!isValid) {
		m_isSuccess = false;
		docs.clear();
		return;
	}
	for (size_t i = 0; i < docs.size(); ++i) {
		// get model idx
		unsigned int idx = docs[i]->getIdxPlane();
		// update doc plane
		docs[i]->updateBBox(pts[i]);
		docs[i]->updateCoefs(pts[i]);
	}
	docs.clear();
}

void Script_ROI::updateDocRANSAC(std::vector<std::vector<cv::Point3f>> &pts)
{
	// data validation
	bool isValid = true;
	if (pts.empty()) isValid = false;
	std::vector<Doc_BIM_Plane *> docs;
	m_doc_bim.getDocPlane(docs);
	if (pts.size() != docs.size()) isValid = false;
	if (!isValid) {
		m_isSuccess = false;
		docs.clear();
		return;
	}
	for (size_t i = 0; i < docs.size(); ++i) {
		// get model idx
		unsigned int idx = docs[i]->getIdxPlane();
		// update doc plane
		docs[i]->updateBBox(pts[i]);
		docs[i]->updateCoefsRANSAC(pts[i]);
	}
	docs.clear();
}
/**
* \brief classify planes
*/
void Script_ROI::filterPlane()
{
	// data validation
	std::vector<Doc_BIM_Plane *> docs;
	m_doc_bim.getDocPlane(docs);
	if (docs.empty())
	{
		m_isSuccess = false;
		return;
	}
	// filter process
	BIM_Filter_Plane::filter_valid(m_doc_bim);
	BIM_Filter_Plane::filter_horiz(m_doc_bim);

#if 0
	// 验证：统计filter_valid和filter_horiz后的结果（前置条件验证）
	std::vector<Doc_BIM_Plane*> docs_check;
	m_doc_bim.getDocPlane(docs_check);
	int valid_count = 0;
	int horiz_count = 0;
	int valid_horiz_count = 0; // 同时满足有效+水平的平面数量（地面候选基础）
	for (size_t i = 0; i < docs_check.size(); ++i) {
		State_BIM_Plane type = docs_check[i]->getType();
		bool is_valid = type.isValid();
		bool is_horiz = type.isHoriz();
		if (is_valid) valid_count++;
		if (is_horiz) horiz_count++;
		if (is_valid && is_horiz) valid_horiz_count++;

		// 额外输出：每个平面的关键信息（方便排查单个平面异常）
		std::cout << "Plane " << i << ": "
			<< "valid=" << (is_valid ? "Y" : "N") << ", "
			<< "horiz=" << (is_horiz ? "Y" : "N") << ", "
			<< "proj_area=" << docs_check[i]->getAreaProjLocal() / 1.0e6f << "㎡, "
			<< "bbox_max_z=" << docs_check[i]->getBBox().m_max.z << "mm" << std::endl;
	}
	std::cout << "===== 前置条件验证结果 =====" << std::endl;
	std::cout << "有效平面数量：" << valid_count << std::endl;
	std::cout << "水平平面数量：" << horiz_count << std::endl;
	std::cout << "有效+水平平面数量（地面候选）：" << valid_horiz_count << std::endl;
	std::cout << "=============================" << std::endl;
#endif

	BIM_Filter_Plane::filter_vert(m_doc_bim);
	BIM_Filter_Plane::filter_ceiling(m_doc_bim);
	BIM_Filter_Plane::filter_floor(m_doc_bim);
	BIM_Filter_Plane::filter_wall(m_doc_bim);
	BIM_Filter_Plane::filter_beam(m_doc_bim); //added by yu.liang
	// reset idx_proc according to conditions
	std::vector<unsigned int> idx_temp; // temp idx list, to replace idx_proc
	for (size_t i = 0; i < docs.size(); ++i) {
		int idx = docs[i]->getIdxPlane();
		auto type = docs[i]->getType();
		if (type.isValid()) {	// conditions
			idx_temp.push_back(static_cast<unsigned int>(idx));
		}
	}
	m_doc_bim.setIdxProc(idx_temp);

#if 0
	std::vector<Doc_BIM_Plane*> docs_final;
	m_doc_bim.getDocPlane(docs_final);
	int floor_count_final = 0;
	int ceiling_count_final = 0;
	int wall_count_final = 0;
	int beam_count_final = 0;
	for (size_t i = 0; i < docs_final.size(); ++i) {
		State_BIM_Plane type = docs_final[i]->getType();
		if (type.isFloor()) floor_count_final++;
		if (type.isCeiling()) ceiling_count_final++;
		if (type.isWall()) wall_count_final++;
		if (type.isBeam()) beam_count_final++;
	}
	std::cout << "===== filterPlane最终结果验证 =====" << std::endl;
	std::cout << "最终地面数量：" << floor_count_final << std::endl;
	std::cout << "最终天花板数量：" << ceiling_count_final << std::endl;
	std::cout << "最终墙面数量：" << wall_count_final << std::endl;
	std::cout << "最终梁数量：" << beam_count_final << std::endl;
	std::cout << "====================================" << std::endl;
#endif
	// clear
	idx_temp.clear();
}
/**
* \brief calibration
*/
void Script_ROI::calibration(std::vector<std::vector<cv::Point3f>> &pts)
{
	// compute calibration matrix
	BIM_Clbr clbr;
	clbr.compute(m_doc_bim);
	//mat_rot = clbr.getMatRot();
	m_doc_bim.setMatClbr(clbr.getMatRot());
	// update points
	pts = MathOperation::planes_rot(clbr.getMatRot(), pts);
	updateDocRANSAC(pts);
}


/**
* \brief locate region of interest
* \changed by yu.liang
*/
void Script_ROI::locROI(const TYPE_ROI type_roi, PlaneCutResultInterface& cutResult, int station_size, bool cut_poly)
{
	img_roi = cv::Mat::zeros(1, 1, CV_8UC3);
	BIM_LOC_ROI* loc_roi = new BIM_LOC_ROI((int)TYPE_ROI::DEMO);
	loc_roi->setWithInclinedPlane(m_withInclinedPlane);
	loc_roi->updateIdxList(m_doc_bim);
	cutResult.plane_wall_idx.clear();

	for (auto id : loc_roi->getWallIdx())
		cutResult.plane_wall_idx.push_back(id);

	LocROISnapshot snapshot;
	snapshot.cutResult = cutResult;
	snapshot.img_roi = img_roi.clone();

	{
		std::vector<Doc_BIM_Plane*> docs;
		m_doc_bim.getDocPlane(docs);

		snapshot.plane_states.reserve(docs.size());
		for (auto* p : docs) {
			snapshot.plane_states.push_back(p->getType());
		}
	}

	switch (type_roi)
	{
	case TYPE_ROI::ROI_WALL_LESS:
		log_info("[Script_ROI::locROI] roi type is: WALL_LESS\n");
		m_isSuccess = loc_roi->locROI_img_0(m_doc_bim, cutResult.plane_xyz, img_roi);
		break;
	case TYPE_ROI::ROI_WALL_BASE:
		log_info("[Script_ROI::locROI] roi type is: WALL_BASE\n");
		if (!loc_roi->locROI_img_1(m_doc_bim, cutResult.plane_xyz, img_roi)) {
			m_isSuccess = loc_roi->locROI_img_5(m_doc_bim, cutResult.plane_xyz, img_roi);
			//m_isSuccess = false;
		}
		break;
	case TYPE_ROI::ROI_BEAM_BASE:
		log_info("[Script_ROI::locROI] roi type is: BEAM_BASE\n");
		m_isSuccess = loc_roi->locROI_img_5(m_doc_bim, cutResult.plane_xyz, img_roi);
		break;
	case TYPE_ROI::ROI_NO_CUT:
		log_info("[Script_ROI::locROI] roi type is: ROI_NO_CUT\n");
		m_isSuccess = loc_roi->locROI_img_6(m_doc_bim, cutResult.plane_xyz, img_roi, station_size,cut_poly);
		break;
	case TYPE_ROI::MANUAL:
		log_info("[Script_ROI::locROI] roi type is: MANUAL\n");
		m_isSuccess = loc_roi->locROI_img_6(m_doc_bim, cutResult.plane_xyz, img_roi, station_size,cut_poly);
		img_roi = cv::Mat::ones(img_roi.cols, img_roi.rows, CV_8UC1) * 255;
		break;
	case TYPE_ROI::DEMO:
		log_info("[Script_ROI::locROI] roi type is: DEMO\n");
		m_isSuccess = loc_roi->locROI_img_6(m_doc_bim, cutResult.plane_xyz, img_roi, station_size,cut_poly);
		img_roi = cv::Mat::ones(img_roi.cols, img_roi.rows, CV_8UC1) * 255;
		break;
	default:
		break;
	}

	if (!m_isSuccess)
	{
		delete loc_roi;
		loc_roi = nullptr;
		return;
	}
	//cv::imwrite("img_roi.jpg", img_roi);
	// cull model
	cv::Mat_<float> mat_clbr;
	m_doc_bim.getMatClbr(mat_clbr);
	auto centers_rot = MathOperation::plane_rot(mat_clbr, cutResult.plane_center);
	auto rot = [&]() {
		std::vector<cv::Point3f> normals_sum(cutResult.plane_center.size());
		transform(cutResult.plane_center.begin(), cutResult.plane_center.end(),
			cutResult.plane_normals.begin(), normals_sum.begin(),
			[](Point3f a, Point3f b) { return a + b; });
		auto normals_tmp = MathOperation::plane_rot(mat_clbr, normals_sum);
		transform(normals_tmp.begin(), normals_tmp.end(),
			centers_rot.begin(), normals_sum.begin(),
			[](Point3f a, Point3f b) { return Util_Math::vec3_normalize(a - b); });
		return normals_sum;
	};

#if 1
	std::vector<Doc_BIM_Plane*> docs;
	m_doc_bim.getDocPlane(docs);
	int floor_count_final = 0;
	int ceiling_count_final = 0;
	int wall_count_final = 0;
	int beam_count_final = 0;
	for (size_t i = 0; i < docs.size(); ++i) {
		State_BIM_Plane type = docs[i]->getType();
		if (type.isFloor()) floor_count_final++;
		if (type.isCeiling()) ceiling_count_final++;
		if (type.isWall()) wall_count_final++;
		if (type.isBeam()) beam_count_final++;
	}

	/*std::cout << "===== locROI_cull前结果验证 =====" << std::endl;
	std::cout << "地面数量：" << floor_count_final << std::endl;
	std::cout << "天花板数量：" << ceiling_count_final << std::endl;
	std::cout << "墙面数量：" << wall_count_final << std::endl;
	std::cout << "梁数量：" << beam_count_final << std::endl;
	std::cout << "====================================" << std::endl;*/
#endif

	auto normals_rot = rot();
	loc_roi->locROI_cull(m_doc_bim, cutResult, normals_rot, centers_rot, img_roi); 
	
#if 1
	std::vector<Doc_BIM_Plane*> after_docs;
	m_doc_bim.getDocPlane(after_docs);
	int after_floor_count_final = 0;
	int after_ceiling_count_final = 0;
	int after_wall_count_final = 0;
	int after_beam_count_final = 0;
	for (size_t i = 0; i < after_docs.size(); ++i) {
		State_BIM_Plane after_type = after_docs[i]->getType();
		if (after_type.isFloor()) after_floor_count_final++;
		if (after_type.isCeiling()) after_ceiling_count_final++;
		if (after_type.isWall()) after_wall_count_final++;
		if (after_type.isBeam()) after_beam_count_final++;
	}


	/*std::cout << "===== locROI_cull后结果验证 =====" << std::endl;
	std::cout << "地面数量：" << after_floor_count_final << std::endl;
	std::cout << "天花板数量：" << after_ceiling_count_final << std::endl;
	std::cout << "墙面数量：" << after_wall_count_final << std::endl;
	std::cout << "梁数量：" << after_beam_count_final << std::endl;
	std::cout << "====================================" << std::endl;*/
#endif

	if ((after_wall_count_final < 4) && (after_wall_count_final < wall_count_final) && (type_roi == TYPE_ROI::ROI_NO_CUT)) {
		cut_poly = true;
		cutResult = snapshot.cutResult;
		img_roi = snapshot.img_roi.clone();

		std::vector<Doc_BIM_Plane*> docs;
		m_doc_bim.getDocPlane(docs);

		for (size_t i = 0; i < docs.size(); ++i) {
			docs[i]->setType(snapshot.plane_states[i]);
		}

		m_isSuccess = loc_roi->locROI_img_6(
			m_doc_bim, cutResult.plane_xyz, img_roi, station_size, cut_poly);

		loc_roi->locROI_cull(
			m_doc_bim, cutResult, normals_rot, centers_rot, img_roi);
	}



	if (loc_roi->get_valid())
	{
		// update doc
		updateDocRANSAC(cutResult.plane_xyz);
		// update idx
		std::vector<int> temp;
		loc_roi->updatePlaneType(m_doc_bim, cutResult);
		// Util_Img::showImg(img_roi);
	}
	else
		m_isSuccess = false;

	//img_roi.release();
	delete loc_roi;
	loc_roi = nullptr;
}
/**
* \brief inverse calibration
*/
void Script_ROI::inverseClbr(PlaneCutResultInterface& cutResult) {
	// get calibration matrix
	cv::Mat_<float> mat_clbr;
	m_doc_bim.getMatClbr(mat_clbr);
	// compute inverse calibration matrix
	cv::Mat_<float> inv_clbr = mat_clbr.inv();
	// rotate models
	cutResult.plane_xyz = MathOperation::planes_rot(inv_clbr, cutResult.plane_xyz);
	for (auto &holes : cutResult.door_window_info)
	{
		if (holes.size() == 0) continue;
		for (auto &info : holes)
		{
			info.corners = MathOperation::plane_rot(inv_clbr, info.corners);
		}
	}

	// update doc
	//updateDocRANSAC(pts);
	// reset bim_doc calibration
	m_doc_bim.clearClbr();
}

/**
* \brief post-processing, update center and check if plane valid
*/
void Script_ROI::postProc(
	const std::vector<std::vector<cv::Point3f>> &pts,
	//std::vector<cv::Point3f> &centers,
	std::vector<bool> &isValid)
{
	// data validation
	size_t n_planes = pts.size();
	//centers.resize(n_planes);
	isValid.resize(n_planes);
	for (size_t i = 0; i < n_planes; ++i)
	{
		const std::vector<cv::Point3f> &points = pts[i];
		size_t npts = pts[i].size();
		if (npts < 1) {
			isValid[i] = false;
			continue;
		}
		isValid[i] = true;
		/*cv::Point3d center(0.0, 0.0, 0.0);
		for (size_t k = 0; k < npts; ++k) {
			center += (cv::Point3d)points[k];
		}
		centers[i] = (center / double(npts));*/
	}
}

void Script_ROI::fixReflect(PlaneCutResultInterface& cutResult)
{
	cv::Point3f ori = cv::Point3f(0.f, 0.f, 0.f);
	for (size_t i = 0; i < cutResult.plane_xyz.size(); i++)
	{
		float Ori2PlaneDist = Util_Math::ComputePointToPlaneDist<float, cv::Point3f>(ori, cutResult.plane_normals[i], cutResult.plane_center[i]);
		for (size_t j = 0; j < cutResult.plane_xyz[i].size(); j++)
		{
			cv::Point3f PT_2d = cv::Point3f(cutResult.plane_xyz[i][j].x, cutResult.plane_xyz[i][j].y, 0.f);
			float Ori2PointDist = Util_Math::ComputePointToPointDist<float, cv::Point3f>(ori, PT_2d);
			/*float arccos_theta = Ori2PointDist / Ori2PlaneDist;*/
			cutResult.plane_reflect[i][j] = (uchar)((float)cutResult.plane_reflect[i][j] * pow(Ori2PointDist / Ori2PlaneDist, 0.25));
		}
	}
}
