#include "bim_proj.h"
#include "../utility/util_img.h"
#include "../utility/util_line_intersection.h"
#include "../models/proj2d.h"
#include "bim_intxn.h"
#include "MathOperation.hpp"
#include "util_math.hpp"
#include "InOutData.h"
#include "bbox.h"


/**
* \brief constructor
*/
BIM_Proj::BIM_Proj() {
}
/**
* \brief constructor
*/
BIM_Proj::BIM_Proj(const std::vector<std::vector<cv::Point3f>> &pts,
	const std::vector<unsigned int>& m_idx_valid, int type_roi) {
	m_bbox_g = new BBox3D();
	m_type_roi = type_roi;
	for (size_t i = 0; i < pts.size(); ++i) {
		// check if idx of doc exists in processing list
		if (!Util_Math::isIdxInList(i, m_idx_valid)) continue;
		BBox3D *bbox = new BBox3D();
		bbox->compute(pts[i]);
		*m_bbox_g = *m_bbox_g + *bbox;
		delete bbox;
		bbox = nullptr; 
	}
}
/**
* \brief destructor
*/
BIM_Proj::~BIM_Proj() {
	if (m_bbox_g != nullptr) {
		delete m_bbox_g;
		m_bbox_g = nullptr;
	}
}
/**
* \brief project locally
*/
void BIM_Proj::proj_local(Doc_BIM &doc_bim, std::vector<std::vector<cv::Point3f>> &pts)
{
	// data validation
	bool isValid = true;
	std::vector<Doc_BIM_Plane *> docs; // all docs
	doc_bim.getDocPlane(docs);
	if (docs.empty()) isValid = false; // check if docs init
	if (docs.size() != pts.size()) isValid = false; // check if doc and model match
	if (!isValid) {
		docs.clear();
		return;
	}
	// set idx_proc according to conditions
	std::vector<unsigned int> idx_proc; // temp idx list, to replace idx_proc
	for (size_t i = 0; i < docs.size(); ++i) {
		int idx = docs[i]->getIdxPlane();
		auto type = docs[i]->getType();
		if (type.isValid()) {
			idx_proc.push_back(static_cast<unsigned int>(idx));
		}
	}
	// iterate docs, then check if docs->idx in idx_proc
	for (size_t i = 0; i < docs.size(); ++i) {
		int idx = docs[i]->getIdxPlane();
		if (idx < 0) continue;
		// check if idx of doc exists in processing list
		if (!Util_Math::isIdxInList(idx, idx_proc)) continue;
		// data validation
		if (pts[i].empty()) continue;
		// copy data
		size_t npts = pts[i].size();
		docs[i]->updateProjLocal(pts[i]);
		cv::Mat img;
		docs[i]->getImgProjLocal(img);
		// export image
		std::stringstream ss;
		ss << "tempdata/proj_" << idx << ".jpg";
		cv::imwrite(ss.str(), img);
		img.release();
	}
	// clear
	docs.clear();
	idx_proc.clear();
}
/**
* \brief project globally
*/
void BIM_Proj::proj_global(Doc_BIM &doc_bim, std::vector<std::vector<cv::Point3f>> &pts, cv::Mat &img_g)
{
	// data validation
	bool isValid = true;
	std::vector<Doc_BIM_Plane *> docs; // all docs
	doc_bim.getDocPlane(docs);
	if (docs.empty()) isValid = false; // check if docs init
	if (docs.size() != pts.size()) isValid = false; // check if doc and model match
	if (!isValid) {
		docs.clear();
		return;
	}
	// set idx_proc according to conditions
	std::vector<unsigned int> idx_proc; // temp idx list, to replace idx_proc
	for (size_t i = 0; i < docs.size(); ++i) {
		int idx = docs[i]->getIdxPlane();
		auto type = docs[i]->getType();
		if (type.isValid()) {
			idx_proc.push_back(static_cast<unsigned int>(idx));
		}
	}
	proj_global(doc_bim, pts, idx_proc, idx_proc, img_g);
	// clear
	docs.clear();
	idx_proc.clear();
}
/**
* \brief locate roi on image
* \added by yu.liang@unre.com
* \2020/12/15
*/
void createIntxnGraph_Wall(
	const std::vector<Doc_BIM_Plane *> &docs,
	const std::vector<unsigned int>& idx_wall,
	const std::vector<unsigned int>& idx_beam,
	Topo_Graph *graph,
	std::vector<std::pair<unsigned int, unsigned int>>& pairIntxn)
{
	// compute intersection
	std::vector<std::pair<unsigned int, unsigned int>> pairIntxn_preprocess;
	std::vector<unsigned int> idx_wall_beam = idx_wall;
	idx_wall_beam.insert(idx_wall_beam.end(), idx_beam.begin(), idx_beam.end());
	pairIntxn.clear();
	BIM_Intxn::compute(docs, idx_wall_beam, 55, pairIntxn_preprocess); //计算出idx_proc中有交集的面的pair
																		 // build graph of walls
	for (int i = 0; i < pairIntxn_preprocess.size(); )
	{
		auto it = pairIntxn_preprocess[i];
		if (Util_Math::isIdxInList(it.first, idx_wall) && Util_Math::isIdxInList(it.second, idx_wall))
		{
			pairIntxn.push_back(it);
			swap(*(std::begin(pairIntxn_preprocess) + i), *(std::end(pairIntxn_preprocess) - 1));
			pairIntxn_preprocess.pop_back();
			continue;
		}
		if (Util_Math::isIdxInList(it.first, idx_beam) && Util_Math::isIdxInList(it.second, idx_beam))
		{
			swap(*(std::begin(pairIntxn_preprocess) + i), *(std::end(pairIntxn_preprocess) - 1));
			pairIntxn_preprocess.pop_back();
			continue;
		}
		i++;
	}
	//cout << "pairIntxn_preprocess is " << pairIntxn_preprocess.size() << endl;
	for (int i = 0; i < pairIntxn_preprocess.size(); i++)
	{
		auto it = pairIntxn_preprocess[i];
		if (Util_Math::isIdxInList(it.first, idx_beam) && Util_Math::isIdxInList(it.second, idx_wall))
		{
			for (int j = i + 1; j < pairIntxn_preprocess.size(); j++)
			{
				auto it_beam = pairIntxn_preprocess[j];
				if (it == it_beam) continue;

				if (it_beam.first == it.first)
				{
					auto newpair = std::make_pair(MIN(it.second, it_beam.second), MAX(it.second, it_beam.second));
					bool isNotExist = find(pairIntxn.begin(), pairIntxn.end(), newpair) == pairIntxn.end();
					if (isNotExist)
						pairIntxn.push_back(newpair);
				}

				if (it_beam.second == it.first)
				{
					auto newpair = std::make_pair(MIN(it.second, it_beam.first), MAX(it.second, it_beam.first));
					bool isNotExist = find(pairIntxn.begin(), pairIntxn.end(), newpair) == pairIntxn.end();
					if (isNotExist)
						pairIntxn.push_back(newpair);
				}
			}
		}
		if (Util_Math::isIdxInList(it.second, idx_beam) && Util_Math::isIdxInList(it.first, idx_wall))
		{
			for (int j = i + 1; j < pairIntxn_preprocess.size(); j++)
			{
				auto it_beam = pairIntxn_preprocess[j];
				if (it == it_beam) continue;

				if (it_beam.first == it.second)
				{
					auto newpair = std::make_pair(MIN(it.first, it_beam.second), MAX(it.first, it_beam.second));
					bool isNotExist = find(pairIntxn.begin(), pairIntxn.end(), newpair) == pairIntxn.end();
					if (isNotExist)
						pairIntxn.push_back(newpair);
				}

				if (it_beam.second == it.second)
				{
					auto newpair = std::make_pair(MIN(it.first, it_beam.first), MAX(it.first, it_beam.first));
					bool isNotExist = find(pairIntxn.begin(), pairIntxn.end(), newpair) == pairIntxn.end();
					if (isNotExist)
						pairIntxn.push_back(newpair);
				}
			}
		}
	}

	//test end
	std::vector<Topo_Node *> nodes;
	nodes.resize(docs.size());
	for (size_t i = 0; i < docs.size(); i++) {
		// init node
		nodes[i] = new Topo_Node();
		nodes[i]->setID(docs[i]->getIdxPlane());
	}
	graph->create(nodes, pairIntxn);
	nodes.clear();
	//pairIntxn.clear();
}

void createIntxnGraph_Wall_Beam(
	const std::vector<Doc_BIM_Plane *> &docs,
	const std::vector<unsigned int>& idx_wall,
	const std::vector<unsigned int>& idx_beam,
	Topo_Graph *graph,
	std::vector<std::pair<unsigned int, unsigned int>>& pairIntxn)
{
	// compute intersection
	std::vector<std::pair<unsigned int, unsigned int>> pairIntxn_preprocess;
	std::vector<unsigned int> idx_wall_beam = idx_wall;
	idx_wall_beam.insert(idx_wall_beam.end(), idx_beam.begin(), idx_beam.end());

	BIM_Intxn::compute(docs, idx_wall_beam, 55, pairIntxn); //?????idx_proc???н????????pair
																		 // build graph of walls

	//test end
	std::vector<Topo_Node *> nodes;
	nodes.resize(docs.size());
	for (size_t i = 0; i < docs.size(); i++) {
		// init node
		nodes[i] = new Topo_Node();
		nodes[i]->setID(docs[i]->getIdxPlane());
	}
	graph->create(nodes, pairIntxn);
	nodes.clear();
	//pairIntxn.clear();
}

void LargestConnecttedComponent(const cv::Mat& srcImage, cv::Mat &dstImage)
{
	cv::Mat temp;
	cv::Mat labels;
	srcImage.copyTo(temp);

	//1. 标记连通域
	int n_comps = connectedComponents(temp, labels, 4, CV_16U);
	vector<int> histogram_of_labels;
	for (int i = 0; i < n_comps; i++)//初始化labels的个数为0
	{
		histogram_of_labels.push_back(0);
	}

	int rows = labels.rows;
	int cols = labels.cols;
	for (int row = 0; row < rows; row++) //计算每个labels的个数
	{
		for (int col = 0; col < cols; col++)
		{
			histogram_of_labels.at(labels.at<unsigned short>(row, col)) += 1;
		}
	}
	histogram_of_labels.at(0) = 0; //将背景的labels个数设置为0

	//2. 计算最大的连通域labels索引
	int maximum = 0;
	int max_idx = 0;
	for (int i = 0; i < n_comps; i++)
	{
		if (histogram_of_labels.at(i) > maximum)
		{
			maximum = histogram_of_labels.at(i);
			max_idx = i;
		}
	}

	//3. 将最大连通域标记为1
	for (int row = 0; row < rows; row++)
	{
		for (int col = 0; col < cols; col++)
		{
			if (labels.at<unsigned short>(row, col) == max_idx)
			{
				labels.at<unsigned short>(row, col) = 255;
			}
			else
			{
				labels.at<unsigned short>(row, col) = 0;
			}
		}
	}

	//4. 将图像更改为CV_8U格式
	labels.convertTo(dstImage, CV_8U);
}

bool BIM_Proj::proj_global_wall_optimize(
	Doc_BIM& doc_bim,
	std::vector<std::vector<cv::Point3f>>& pts,
	const std::vector<unsigned int>& idx_area,
	std::vector<unsigned int>& idx_wall,
	std::vector<unsigned int>& idx_roomwall,
	const std::vector<unsigned int>& idx_beam,
	cv::Mat& img_g)
{
	// =========================================================
	// 0. 基础保护
	// =========================================================
	if (idx_wall.size() < 2)
		return true;

	std::vector<Doc_BIM_Plane*> docs;
	doc_bim.getDocPlane(docs);

	// =========================================================
	// 1. 提取每堵墙的稳定 2D 轴线（来自全局投影）
	// =========================================================
	std::unordered_map<unsigned int,
		std::pair<cv::Point2f, cv::Point2f>> wall_lines;
	std::unordered_map<unsigned int, bool> has_line;

	for (unsigned int wid : idx_wall)
	{
		has_line[wid] = false;

		if (wid >= docs.size())
			continue;

		cv::Mat img;
		docs[wid]->getImgProjGlobal(img);
		if (img.empty())
			continue;

		if (img.channels() == 3)
			cv::cvtColor(img, img, CV_BGR2GRAY);

		std::vector<std::vector<cv::Point>> contours;
		cv::findContours(img, contours,
			cv::RETR_EXTERNAL,
			cv::CHAIN_APPROX_SIMPLE);

		if (contours.empty())
			continue;

		std::vector<cv::Point> all_pts;
		for (auto& c : contours)
			all_pts.insert(all_pts.end(), c.begin(), c.end());

		if (all_pts.size() < 5)
			continue;

		cv::RotatedRect rr = cv::minAreaRect(all_pts);
		cv::Point2f p[4];
		rr.points(p);

		float len01 = cv::norm(p[0] - p[1]);
		float len12 = cv::norm(p[1] - p[2]);

		if (len01 > len12)
			wall_lines[wid] = { p[0], p[1] };
		else
			wall_lines[wid] = { p[1], p[2] };

		has_line[wid] = true;
	}

	// =========================================================
	// 2. 构建 Topo_Node（wall → node）
	// =========================================================
	std::vector<Topo_Node*> nodes;
	std::unordered_map<unsigned int, unsigned int> wall2node;

	for (unsigned int wid : idx_wall)
	{
		if (!has_line[wid])
			continue;

		Topo_Node* node = new Topo_Node();
		node->setID(wid);   // wall id 作为 node 语义 id

		wall2node[wid] = nodes.size();
		nodes.push_back(node);
	}

	if (nodes.size() < 2)
		return true;

	// =========================================================
	// 3. 构建边（真实几何相交）
	// =========================================================
	std::vector<std::pair<unsigned int, unsigned int>> edges;

	for (size_t i = 0; i < nodes.size(); ++i)
	{
		unsigned int wi = nodes[i]->getID();

		for (size_t j = i + 1; j < nodes.size(); ++j)
		{
			unsigned int wj = nodes[j]->getID();

			if (Util_Line_Intersection::isIntersect(
				wall_lines[wi].first,
				wall_lines[wi].second,
				wall_lines[wj].first,
				wall_lines[wj].second))
			{
				edges.emplace_back(i, j);
			}
		}
	}

	// =========================================================
	// 4. 构建拓扑图 & 连通分量
	// =========================================================
	Topo_Graph graph;
	if (!graph.create(nodes, edges))
		return true; // 兜底：不筛

	std::vector<std::vector<Topo_Node*>> components;
	graph.getCmpts(1, components);

	if (components.empty())
		return true;

	// =========================================================
	// 5. 选择最大连通分量（主结构墙）
	// =========================================================
	size_t best = 0;
	for (size_t i = 1; i < components.size(); ++i)
	{
		if (components[i].size() > components[best].size())
			best = i;
	}

	// =========================================================
	// 6. 回填 idx_wall / idx_roomwall（唯一筛选点）
	// =========================================================
	idx_wall.clear();
	for (Topo_Node* n : components[best])
		idx_wall.push_back(n->getID());

	idx_roomwall = idx_wall;

	// =========================================================
	// 7. 输出 img_g（保持原流程可用）
	// =========================================================
	bool img_g_initialized = false;

	for (unsigned int wid : idx_wall)
	{
		cv::Mat img;
		docs[wid]->getImgProjGlobal(img);

		if (img.empty())
			continue;

		if (img.channels() == 3)
			cv::cvtColor(img, img, CV_BGR2GRAY);

		if (!img_g_initialized)
		{
			img_g = cv::Mat::zeros(img.size(), CV_8UC1);
			img_g_initialized = true;
		}

		// 尺寸保护（极其重要）
		if (img.size() != img_g.size())
			continue;

		img_g |= img;
	}


	// =========================================================
	// 8. Debug（建议保留）
	// =========================================================
	std::cout << "[proj_global_wall_optimize_safe] "
		<< "input walls = " << wall2node.size()
		<< ", keep walls = " << idx_wall.size()
		<< std::endl;

	return true;
}



/**
* \brief project globally
* \param idx_area index list of models to get area/bbox for projection
* \param idx_wall index list of models to project
* \added by yu.liang@unre.com
* \2020/12/15
*/
//bool BIM_Proj::proj_global_wall_optimize(
//	Doc_BIM& doc_bim,
//	std::vector<std::vector<cv::Point3f>>& pts,
//	const std::vector<unsigned int>& idx_area,
//	std::vector<unsigned int>& idx_wall,
//	std::vector<unsigned int>& idx_roomwall,
//	const std::vector<unsigned int>& idx_beam,
//	cv::Mat& img_g)
//{
//	// ---------- 0. 基础校验 ----------
//	std::vector<Doc_BIM_Plane*> docs;
//	doc_bim.getDocPlane(docs);
//	if (docs.empty() || docs.size() != pts.size())
//		return false;
//
//	if (idx_wall.size() < 2)
//		return false; // 墙太少，不处理
//
//	const size_t n_model = pts.size();
//
//	// ---------- 1. 生成每面墙的 2D 线段（minAreaRect） ----------
//	std::vector<std::pair<cv::Point2f, cv::Point2f>> wall_lines(n_model);
//	std::vector<bool> has_line(n_model, false);
//
//	for (auto wid : idx_wall)
//	{
//		cv::Mat img;
//		docs[wid]->getImgProjGlobal(img);
//		cv::cvtColor(img, img, cv::COLOR_RGB2GRAY);
//
//		std::vector<std::vector<cv::Point>> contours;
//		cv::findContours(img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
//
//		if (contours.empty())
//			continue;
//
//		std::vector<cv::Point> merged;
//		for (auto& c : contours)
//			merged.insert(merged.end(), c.begin(), c.end());
//
//		if (merged.size() < 5)
//			continue;
//
//		cv::RotatedRect rrt = cv::minAreaRect(merged);
//		cv::Point2f p[4];
//		rrt.points(p);
//
//		// 取长边作为墙轴线
//		float d01 = cv::norm(p[0] - p[1]);
//		float d12 = cv::norm(p[1] - p[2]);
//
//		if (d01 > d12)
//			wall_lines[wid] = { p[0], p[1] };
//		else
//			wall_lines[wid] = { p[1], p[2] };
//
//		has_line[wid] = true;
//	}
//
//	// ---------- 2. 构建墙-墙相交图（真实几何） ----------
//	Topo_Graph graph;
//
//	for (size_t i = 0; i < idx_wall.size(); ++i)
//	{
//		unsigned int wi = idx_wall[i];
//		if (!has_line[wi]) continue;
//
//		graph.addNode(wi);
//
//		for (size_t j = i + 1; j < idx_wall.size(); ++j)
//		{
//			unsigned int wj = idx_wall[j];
//			if (!has_line[wj]) continue;
//
//			if (Util_Line_Intersection::isIntersect(
//				wall_lines[wi].first,
//				wall_lines[wi].second,
//				wall_lines[wj].first,
//				wall_lines[wj].second))
//			{
//				graph.addEdge(wi, wj);
//			}
//		}
//	}
//
//	// ---------- 3. 取最大连通分量（主结构） ----------
//	std::vector<std::vector<Topo_Node*>> components;
//	graph.getCmpts(1, components);
//
//	if (components.empty())
//		return false;
//
//	size_t best = 0;
//	for (size_t i = 1; i < components.size(); ++i)
//	{
//		if (components[i].size() > components[best].size())
//			best = i;
//	}
//
//	idx_wall.clear();
//	for (auto* node : components[best])
//		idx_wall.push_back(node->getID());
//
//	idx_roomwall = idx_wall;
//
//	// ---------- 4. 仅用于投影（不做删除） ----------
//	cv::Point3f bbox_diag = m_bbox_g->m_max - m_bbox_g->m_min;
//	float vl = std::max(bbox_diag.x, bbox_diag.y) / 512.f;
//	unsigned int extent_x = bbox_diag.x / vl + 10;
//	unsigned int extent_y = bbox_diag.y / vl + 10;
//
//	img_g = cv::Mat::zeros(extent_y, extent_x, CV_8UC3);
//
//	for (auto wid : idx_wall)
//	{
//		cv::Mat img;
//		docs[wid]->getImgProjGlobal(img);
//		Util_Img::merge_c(img, img_g);
//	}
//
//	docs.clear();
//	return true;
//}


bool BIM_Proj::proj_global_wall_beam_optimize(
	Doc_BIM &doc_bim, std::vector<std::vector<cv::Point3f>> &pts,
	const std::vector<unsigned int> &idx_area,
	std::vector<unsigned int> &idx_wall,
	const std::vector<unsigned int> &idx_beam,
	cv::Mat &img_g)
{
	// data validation
	bool isValid = true;
	std::vector<Doc_BIM_Plane *> docs; // all docs
	doc_bim.getDocPlane(docs);
	if (docs.empty()) isValid = false; // check if docs init
	if (docs.size() != pts.size()) isValid = false; // check if doc and model match
	if (!isValid) {
		docs.clear();
		return false;
	}
	// get idx needing processing
	size_t n_model = pts.size();

	// compute projection parameters
	cv::Point3f bbox_diag = m_bbox_g->m_max - m_bbox_g->m_min;
	float vl = MAX(bbox_diag.x, bbox_diag.y) / 512.f;  //why devided by 512?
	unsigned int extent_x = bbox_diag.x / vl + 10;
	unsigned int extent_y = bbox_diag.y / vl + 10;

	//add by yu.liang@unre.com start
	std::vector<std::vector<cv::Point2i>> line_endpoints(n_model, std::vector<cv::Point2i>(3, cv::Point2i(0, 0)));

	auto idx_WallAndBeam = idx_wall;
	idx_WallAndBeam.insert(idx_WallAndBeam.end(), idx_beam.begin(), idx_beam.end());
	for (auto i : idx_WallAndBeam)
	{
		//IOData::SavePoint3fData("F:\\Unre\\BIM\\test\\" + to_string(FILE_NAME) + "\\wall_" + to_string(i) + ".txt", pts[i]);

		cv::Mat img_temp;
		docs[i]->getImgProjGlobal(img_temp);
		cvtColor(img_temp, img_temp, CV_RGB2GRAY);
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		//imwrite("F:\\Unre\\BIM\\test\\" + to_string(FILE_NAME) + "\\" + to_string(i) + ".jpg", img_temp);

		findContours(img_temp, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point());
		//changed by yu.liang 2021/1/6 start
		int max_contoursize = 0;
		int max_contourid = 0;
		for (size_t id = 0; id < contours.size(); id++)
		{
			if (max_contoursize < contours[id].size())
			{
				max_contoursize = contours[id].size();
				max_contourid = id;
			}
		}
		cv::RotatedRect rrt = cv::minAreaRect(contours[max_contourid]);
		//changed by yu.liang 2021/1/6 end

		cv::Point2f ppts[4];
		rrt.points(ppts);

		cv::Point2i endpoint_a, endpoint_b;

		if (rrt.size.height > rrt.size.width)
		{
			endpoint_a = (ppts[0] + ppts[3]) / 2;
			endpoint_b = (ppts[1] + ppts[2]) / 2;
		}
		else {
			endpoint_a = (ppts[0] + ppts[1]) / 2;
			endpoint_b = (ppts[3] + ppts[2]) / 2;
		}
		cv::Point2i cpt = rrt.center;
		line_endpoints[i] = { endpoint_a ,endpoint_b,  cpt };

		//cvtColor(img_temp, img_temp, CV_GRAY2RGB);
		//for (int i = 0; i < 4; i++)
		//	cv::line(img_temp, ppts[i], ppts[(i + 1) % 4], cv::Scalar(0, 255, 0));
		//imwrite("F:/Unre/BIM/test/" + to_string(FILE_NAME) + "/c_"+to_string(i)+".jpg", img_temp);
	}

	//remove the wrong wall idx
	cv::Point2i ori = cv::Point2i(int(floor((-m_bbox_g->m_min.x + 1) / vl)), int(floor((-m_bbox_g->m_min.y + 1) / vl)));
	std::vector<unsigned int> idx_wall_bak = idx_WallAndBeam;

	for (int idx = 0; idx < idx_WallAndBeam.size(); idx++)
	{
		//if (MathOperation::CalLine2LineAngle(ori, line_endpoints[idx_WallAndBeam[idx]][2], line_endpoints[idx_WallAndBeam[idx]][0], line_endpoints[idx_WallAndBeam[idx]][1]) < 15.f)
		//{
		//	idx_wall_bak[idx] = std::numeric_limits<unsigned int>::max();
		//	State_BIM_Plane type = docs[idx_WallAndBeam[idx]]->getType();
		//	//type.removeWall();
		//	type.clearState();
		//	docs[idx_WallAndBeam[idx]]->setType(type);
		//	continue;
		//}

		for (auto i : idx_WallAndBeam)
		{
			if (i == idx_WallAndBeam[idx]) continue;
			if (Util_Line_Intersection::isIntersect(ori, line_endpoints[idx_WallAndBeam[idx]][2], line_endpoints[i][0], line_endpoints[i][1]) &&
				!Util_Line_Intersection::isIntersect(line_endpoints[idx_WallAndBeam[idx]][0], line_endpoints[idx_WallAndBeam[idx]][1], line_endpoints[i][0], line_endpoints[i][1]))
			{
				idx_wall_bak[idx] = std::numeric_limits<unsigned int>::max();

				break;
			}
		}
	}

	idx_WallAndBeam.clear();
	for (auto idx : idx_wall_bak)
	{
		if (idx != std::numeric_limits<unsigned int>::max())
		{
			idx_WallAndBeam.push_back(idx);
			//IOData::SavePoint3fData("F:\\Unre\\BIM\\test\\" + to_string(FILE_NAME) + "\\wall_" + to_string(idx) + ".txt", pts[idx]);
		}
	}

	// compute intersection
	Topo_Graph *graph = new Topo_Graph();
	//changed by yu.liang
	std::vector<std::pair<unsigned int, unsigned int>> pairIntxn;
	createIntxnGraph_Wall_Beam(docs, idx_wall, idx_beam, graph, pairIntxn);

	// find largest cluster of walls
	std::vector<std::vector<Topo_Node *>> cmpts;
	graph->getCmpts(2, cmpts);
	if (cmpts.empty()) {
		delete graph;
		graph = nullptr;
		docs.clear();
		return false;
	}
	else {
		unsigned int idx_cmpts = 0;
		unsigned int max_len = 0;
		for (size_t i = 0; i < cmpts.size(); ++i) {
			if (cmpts[i].size() > max_len) {
				max_len = cmpts[i].size();
				idx_cmpts = i;
			}
		}
		// replace idx_wall
		idx_WallAndBeam.clear();
		for (size_t i = 0; i < cmpts[idx_cmpts].size(); ++i) {
			idx_WallAndBeam.push_back(cmpts[idx_cmpts][i]->getID());
		}
		delete graph;
		graph = nullptr;
	}

	//if vaild wall number <3, then goto RoomAlignment::CutScanPlaneClassFcn()
	//added by yu.liang 2021/1/6 start
	if (idx_WallAndBeam.size() < 3) {
		docs.clear();
		return false;
	}
	//changed by yu.liang 2021/1/6 end

	idx_wall_bak = idx_WallAndBeam;
	for (int idx = 0; idx < idx_WallAndBeam.size(); idx++)
	{
		for (auto i : idx_WallAndBeam)
		{
			if (i == idx_WallAndBeam[idx]) continue;
			if (Util_Line_Intersection::isIntersect(ori, line_endpoints[idx_WallAndBeam[idx]][2], line_endpoints[i][0], line_endpoints[i][1]))
			{
				idx_wall_bak[idx] = std::numeric_limits<unsigned int>::max();
				/*State_BIM_Plane type = docs[idx_wall[idx]]->getType();
				type.removeWall();
				docs[idx_wall[idx]]->setType(type);*/
				break;
			}
		}
	}
	idx_WallAndBeam.clear();
	for (auto idx : idx_wall_bak)
	{
		if (idx != std::numeric_limits<unsigned int>::max())
		{
			idx_WallAndBeam.push_back(idx);
		}
	}
	// init global projection image
	img_g = cv::Mat::zeros(extent_y, extent_x, CV_8UC3);
	// project each model and append image to global one
	for (auto i : idx_WallAndBeam) {
		cv::Mat img_temp;
		docs[i]->getImgProjGlobal(img_temp);
		Util_Img::merge_c(img_temp, img_g);
	}
	std::vector<float> line_midpoints_cos(idx_WallAndBeam.size());
	std::vector<std::vector<cv::Point2i>> line_endpoints_sort(idx_WallAndBeam.size(), std::vector<cv::Point2i>(2, cv::Point2i(0, 0)));
	std::vector<int> all_endpoint_idx(idx_WallAndBeam.size());
	for (auto i = 0; i < idx_WallAndBeam.size(); i++)
	{
		all_endpoint_idx[i] = i;
		int idx = idx_WallAndBeam[i];
		line_endpoints_sort[i][0] = line_endpoints[idx][0];
		line_endpoints_sort[i][1] = line_endpoints[idx][1];

		line_midpoints_cos[i] = acos((line_endpoints[idx][2].x - ori.x) /
			(sqrt(pow(line_endpoints[idx][2].x - ori.x, 2) + pow(line_endpoints[idx][2].y - ori.y, 2))));
		if ((line_endpoints[idx][2].y - ori.y) <= 0)
			line_midpoints_cos[i] *= -1.f;
	}
	sort(all_endpoint_idx.begin(), all_endpoint_idx.end(),
		[&](const int& a, const int& b) {
		return (line_midpoints_cos[a] < line_midpoints_cos[b]);
	});
	std::vector<std::vector<cv::Point2i>> all_endpoints_sorted(1);
	for (int i = 0; i < all_endpoint_idx.size(); i++)
	{
		int wall_idx_sort = all_endpoint_idx[i];

		cv::Point2i a, b;
		a = line_endpoints_sort[wall_idx_sort][0];
		b = line_endpoints_sort[wall_idx_sort][1];

		int oa_cross_ob = (a.x - ori.x)*(b.y - ori.y) - (b.x - ori.x)*(a.y - ori.y);
		if (oa_cross_ob > 0)
		{
			all_endpoints_sorted[0].push_back(line_endpoints_sort[wall_idx_sort][0]);
			all_endpoints_sorted[0].push_back(line_endpoints_sort[wall_idx_sort][1]);
		}
		else
		{
			all_endpoints_sorted[0].push_back(line_endpoints_sort[wall_idx_sort][1]);
			all_endpoints_sorted[0].push_back(line_endpoints_sort[wall_idx_sort][0]);
		}
		//line(img_g, all_endpoints[all_endpoint_idx[i%n_all_endpoint]], all_endpoints[all_endpoint_idx[(i+ 1)% n_all_endpoint]], cv::Scalar(0, 0, 255), 2, 8, 0);
	}

	Util_Img::cvtRGB2BIN(img_g, img_g);
	cv::drawContours(img_g, all_endpoints_sorted, 0, cv::Scalar(255), CV_FILLED);

	//2*25mm = 50mm
	//close operate changed by yu.liang 2021/1/4
	cv::erode(img_g, img_g, cv::Mat::ones(7, 7, CV_8U));
	LargestConnecttedComponent(img_g, img_g);
	cv::dilate(img_g, img_g, cv::Mat::ones(7, 7, CV_8U));
	Util_Img::cvtGray2RGB(img_g, img_g);
	// changed by yu.liang@unre.com 2021/1/4 end

	// clear
	docs.clear();
	return true;
	//delete m_bbox_g;
}

/**
* \brief project globally
* \param idx_area index list of models to get area/bbox for projection
* \param idx_wall index list of models to project
* \added by yu.liang@unre.com
* \2020/12/15
*/
void BIM_Proj::proj_global_ceil_optimize(
	Doc_BIM &doc_bim, std::vector<std::vector<cv::Point3f>> &pts,
	const std::vector<unsigned int> &idx_area,
	std::vector<unsigned int> &idx_ceil, const cv::Mat &img_wall, cv::Mat &img_g)
{
	// data validation
	if (img_wall.cols < 1 || img_wall.rows < 1) return; // check if img_wall not empty
	bool isValid = true;
	std::vector<Doc_BIM_Plane *> docs; // all docs
	doc_bim.getDocPlane(docs);
	if (docs.empty()) isValid = false; // check if docs init
	if (docs.size() != pts.size()) isValid = false; // check if doc and model match
	if (!isValid) {
		docs.clear();
		return;
	}

	// find the ceiling above
	cv::Mat img_wall_bak = img_wall.clone();
	std::vector<uint> final_ceil_idx, final_ceil_idx_bak;
	int ceil_above_idx = -1;
	for (auto idx : idx_ceil)
	{
		BBox3D ceil_box = docs[idx]->getBBox();
		if (ceil_box.m_min.x < 0.0f && ceil_box.m_max.x > 0.0f && ceil_box.m_min.y < 0.0f && ceil_box.m_max.y > 0.0f)
		{
			final_ceil_idx_bak.push_back(idx);
			unsigned int center_num = 0;
			unsigned int num_threshold = 100;
			float radius = 150.f;
			for (auto pt : pts[idx])
			{
				if (pt.x < radius && pt.x > -radius && pt.y < radius && pt.y > -radius)
				{
					center_num++;
				}
			}
			if (center_num < num_threshold) continue;
			ceil_above_idx = idx;
			final_ceil_idx.push_back(idx);
		}
	}

	if (ceil_above_idx != -1)
	{
		BBox3D ceil_above_box = docs[ceil_above_idx]->getBBox();

		int retry = 10;
		int overlayCount = 0;
		while (retry--)
		{
			overlayCount = 0;
			for (auto idx : idx_ceil)
			{
				if (idx == ceil_above_idx) continue;
				if (std::find(final_ceil_idx.begin(), final_ceil_idx.end(), idx) != final_ceil_idx.end())
					continue;

				BBox3D ceil_box = docs[idx]->getBBox();
				if ((ceil_box.m_min.x < ceil_above_box.m_min.x &&
					ceil_box.m_max.x > ceil_above_box.m_max.x &&
					ceil_box.m_min.y < ceil_above_box.m_min.y &&
					ceil_box.m_max.y > ceil_above_box.m_max.y) ||

					(ceil_box.m_min.x > ceil_above_box.m_min.x &&
						ceil_box.m_max.x < ceil_above_box.m_max.x &&
						ceil_box.m_min.y > ceil_above_box.m_min.y &&
						ceil_box.m_max.y < ceil_above_box.m_max.y))
				{
					final_ceil_idx.push_back(idx);
					overlayCount++;
				}
			}
			if (overlayCount == 0)
				break;
		}
	}
	else if (!final_ceil_idx_bak.empty() /* || ceil_above_idx == -1*/)
	{
		printf("error, ceiling above head can't be found![1]\n");
		final_ceil_idx = final_ceil_idx_bak;
		//return;
	}
	else
	{
		final_ceil_idx = idx_ceil;
		printf("error, ceiling above head can't be found![2]\n");
	}
	for (auto idx : final_ceil_idx)
	{
		cv::Mat img;
		docs[idx]->getImgProjGlobal(img);
		Util_Img::merge_c(img, img_wall_bak);
	}
	cvtColor(img_wall_bak, img_wall_bak, CV_RGB2GRAY);
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	findContours(img_wall_bak, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point());
	//changed by yu.liang 2021/1/6 start
	int max_contoursize = 0;
	int max_contourid = 0;
	for (size_t id = 0; id < contours.size(); id++)
	{
		if (max_contoursize < contours[id].size())
		{
			max_contoursize = contours[id].size();
			max_contourid = id;
		}
	}
	//Util_Img::cvtRGB2BIN(img_wall_bak, img_wall_bak);
	cv::drawContours(img_wall_bak, contours, max_contourid, cv::Scalar(255), CV_FILLED);
	/*std::string debug_path = "F:/Unre/BIM/test/" + to_string(FILE_NAME) + "/";
	imwrite(debug_path + "img_wall_bak.jpg", img_wall_bak);*/
	// compute projection parameters
	cv::Point3f bbox_diag = m_bbox_g->m_max - m_bbox_g->m_min;
	float vl = MAX(bbox_diag.x, bbox_diag.y) / 512.f;  //why devided by 512?
	unsigned int extent_x = bbox_diag.x / vl + 10;
	unsigned int extent_y = bbox_diag.y / vl + 10;
	// init global projection image
	//img_g = cv::Mat::zeros(extent_y, extent_x, CV_8UC3);
	cv::cvtColor(img_wall_bak, img_g, CV_GRAY2RGB);
	// project each model and append image to global one
	//std::vector<cv::Point2i> ptVx;
	for (size_t i = 0; i < pts.size(); ++i) {
		// check if idx of doc exists in processing list
		if (!Util_Math::isIdxInList(i, idx_ceil)) {
			State_BIM_Plane state_temp = docs[i]->getType();
			state_temp.removeCeiling();
			docs[i]->setType(state_temp);
			continue;
		}

		cv::Point2i center;
		center.x = int(floor((docs[i]->getCenter().x - m_bbox_g->m_min.x + 1) / vl));
		center.y = int(floor((docs[i]->getCenter().y - m_bbox_g->m_min.y + 1) / vl));
		cv::Mat img_wall_bin;
		Util_Img::cvtGray2BIN(img_wall_bak, 5.f, img_wall_bin);
		if (255 != img_wall_bin.at<uchar>(center))
		{
			State_BIM_Plane state_temp = docs[i]->getType();
			state_temp.removeCeiling();
			docs[i]->setType(state_temp);
			continue;
		}

		cv::Mat img;
		//proj_global_single(pts[i], *m_bbox_g, extent_x, extent_y, vl, ptVx, img);
		docs[i]->getImgProjGlobal(img);
		//added by yu.liang@unre.com start
		//2020/12/16
		//find the contour of img, if half points of contour in img_wall, then add it to available ceiling
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::Mat img_gray;
		cvtColor(img, img_gray, CV_RGB2GRAY);
		findContours(img_gray, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point());
		int max_contour_size = 0, max_contour_idx = -1;
		for (int id = 0; id < contours.size(); id++)
		{
			if (contours[id].size() > max_contour_size)
			{
				max_contour_size = contours[id].size();
				max_contour_idx = id;
			}
		}
		int n_intersect_contours = 0;
		for (int j = 0; j < max_contour_size; j++)
		{
			if (255 == img_wall_bin.at<uchar>(contours[max_contour_idx][j]))
				n_intersect_contours++;
		}
		if (n_intersect_contours < (max_contour_size / 2))
		{
			State_BIM_Plane state_temp = docs[i]->getType();
			state_temp.removeCeiling();
			docs[i]->setType(state_temp);
			continue;
		}
		//added by yu.liang@unre.com end

		//docs[i]->setImgProjGlobal(img);
		//docs[i]->setPtVx2(ptVx);

		/*cv::circle(img_g_test, center2, 2, cv::Scalar(0, 0, 255), 1, 8, 0);
		cv::putText(img, to_string(i), center, cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 255),1,8);*/
		// export each projection image
		//std::stringstream ss;
		//ss << "tempdata/proj_" << i << ".jpg";
		//cv::imwrite(ss.str(), img);
		Util_Img::merge_c(img, img_g);
		img.release();
	}
	//cv::imwrite("F:\\Unre\\BIM\\test\\14\\img_g_test.jpg", img_g_test);
	auto idx_ceil_bak = idx_ceil;
	idx_ceil.clear();
	for (int i = 0; i < pts.size(); ++i) {
		if (docs[i]->getType().isCeiling())
		{
			idx_ceil.push_back(i);
		}
	}
	if (idx_ceil.size() == 0)
	{
		for (auto idx : idx_ceil_bak)
		{
			/*cv::Mat img;
			docs[idx]->getImgProjGlobal(img);
			Util_Img::merge_c(img, img_g);*/
			State_BIM_Plane state_temp = docs[idx]->getType();
			state_temp.appendCeiling();
			docs[idx]->setType(state_temp);
		}
	}
	// clear
	docs.clear();
	//delete m_bbox_g;
}

/**
* \brief project globally
* \param idx_area index list of models to get area/bbox for projection
* \param idx_wall index list of models to project
* \added by yu.liang@unre.com
* \2020/12/15
*/
bool BIM_Proj::proj_global_multiceiling_optimize(
	Doc_BIM &doc_bim, std::vector<std::vector<cv::Point3f>> &pts,
	const std::vector<unsigned int> &idx_area,
	std::vector<unsigned int> &idx_ceil, cv::Mat &img_g)
{
	// data validation
	bool isValid = true;
	std::vector<Doc_BIM_Plane *> docs; // all docs
	doc_bim.getDocPlane(docs);
	if (docs.empty()) isValid = false; // check if docs init
	if (docs.size() != pts.size()) isValid = false; // check if doc and model match

	if (!isValid) {
		docs.clear();
		return false;
	}

	// find the ceiling above
	std::vector<uint> final_ceil_idx, final_ceil_idx_bak;
	int ceil_above_idx = -1;
	for (auto idx : idx_ceil)
	{
		BBox3D ceil_box = docs[idx]->getBBox();
		if (ceil_box.m_min.x < 0.0f && ceil_box.m_max.x > 0.0f && ceil_box.m_min.y < 0.0f && ceil_box.m_max.y > 0.0f)
		{
			final_ceil_idx_bak.push_back(idx);
			unsigned int center_num = 0;
			unsigned int num_threshold = 100;
			float radius = 150.f;
			for (auto pt : pts[idx])
			{
				if (pt.x < radius && pt.x > -radius && pt.y < radius && pt.y > -radius)
				{
					center_num++;
				}
			}
			if (center_num < num_threshold) continue;
			ceil_above_idx = idx;
		}
	}

	if (ceil_above_idx != -1)
	{
		final_ceil_idx.push_back(ceil_above_idx);
		BBox3D ceil_above_box = docs[ceil_above_idx]->getBBox();

		int retry = 10;
		int overlayCount = 0;
		while (retry--)
		{
			overlayCount = 0;
			for (auto idx : idx_ceil)
			{
				if (idx == ceil_above_idx) continue;
				if (std::find(final_ceil_idx.begin(), final_ceil_idx.end(), idx) != final_ceil_idx.end())
					continue;
				BBox3D ceil_box = docs[idx]->getBBox();
				if (BBox3D::isOverlapXOY(ceil_above_box, ceil_box, 50.f) &&
					Util_Math::ComputePointToPointDist<float, cv::Point3f>(docs[ceil_above_idx]->getCenter(), docs[idx]->getCenter()) < 6000.f
					)
				{
					ceil_above_box = ceil_above_box + ceil_box;
					final_ceil_idx.push_back(idx);
					overlayCount++;
				}
			}
			if (overlayCount == 0)
				break;
		}
	}
	else if (!final_ceil_idx_bak.empty() /* || ceil_above_idx == -1*/)
	{
		printf("error, ceiling above head can't be found![1]\n");
		final_ceil_idx = final_ceil_idx_bak;
		//return;
	}
	else
	{
		final_ceil_idx = idx_ceil;
		printf("error, ceiling above head can't be found![2]\n");
	}
	cv::Point3f bbox_diag = m_bbox_g->m_max - m_bbox_g->m_min;
	float vl = MAX(bbox_diag.x, bbox_diag.y) / 512.f;  //why devided by 512?
	unsigned int extent_x = bbox_diag.x / vl + 10;
	unsigned int extent_y = bbox_diag.y / vl + 10;
	// init global projection image
	img_g = cv::Mat::zeros(extent_y, extent_x, CV_8UC3);
	for (auto idx : final_ceil_idx)
	{
		cv::Mat img;
		docs[idx]->getImgProjGlobal(img);
		Util_Img::merge_c(img, img_g);
	}

	idx_ceil.clear();
	idx_ceil = final_ceil_idx;
	// clear
	docs.clear();
	//delete m_bbox_g;
	/*std::string debug_path = "F:/Unre/BIM/test/1/test.jpg";
	imwrite(debug_path, img_g);
*/
	return true;
}

/**
* \brief project globally
* \param idx_area index list of models to get area/bbox for projection
* \param idx_wall index list of models to project
* \added by yu.liang@unre.com
* \2020/12/15
*/
void BIM_Proj::proj_global_floor_optimize(
	Doc_BIM &doc_bim, std::vector<std::vector<cv::Point3f>> &pts,
	const std::vector<unsigned int> &idx_area,
	std::vector<unsigned int> &idx_floor, const cv::Mat &img_wall, cv::Mat &img_g)
{
	// data validation
	bool isValid = true;
	std::vector<Doc_BIM_Plane *> docs; // all docs
	doc_bim.getDocPlane(docs);
	if (docs.empty()) isValid = false; // check if docs init
	if (docs.size() != pts.size()) isValid = false; // check if doc and model match
	if (!isValid) {
		docs.clear();
		return;
	}
	// get idx needing processing
	size_t n_model = pts.size();
	// compute global bbox
	//BBox3D *m_bbox_g = new BBox3D();
	//for (size_t i = 0; i < n_model; ++i) {
	//	// check if idx of doc exists in processing list
	//	if (!isIdxInList(i, idx_area)) continue;
	//	BBox3D *bbox = new BBox3D();
	//	bbox->compute(pts[i]);
	//	*m_bbox_g = *m_bbox_g + *bbox;
	//}
	// compute projection parameters
	cv::Point3f bbox_diag = m_bbox_g->m_max - m_bbox_g->m_min;
	float vl = MAX(bbox_diag.x, bbox_diag.y) / 512.f;  //why devided by 512?
	unsigned int extent_x = bbox_diag.x / vl + 10;
	unsigned int extent_y = bbox_diag.y / vl + 10;
	// init global projection image
	img_g = cv::Mat::zeros(extent_y, extent_x, CV_8UC3);
	// project each model and append image to global one
	std::vector<cv::Point2i> ptVx;
	for (size_t i = 0; i < n_model; ++i) {
		// check if idx of doc exists in processing list
		if (!Util_Math::isIdxInList(i, idx_floor)) {
			State_BIM_Plane state_temp = docs[i]->getType();
			state_temp.removeFloor();
			docs[i]->setType(state_temp);
			continue;
		}

		cv::Point2i center;
		center.x = int(floor((docs[i]->getCenter().x - m_bbox_g->m_min.x + 1) / vl));
		center.y = int(floor((docs[i]->getCenter().y - m_bbox_g->m_min.y + 1) / vl));
		//cv::Mat img_wall_bin;
		//Util_Img::cvtRGB2BIN(img_wall, img_wall_bin);
		//if (255 != img_wall_bin.at<uchar>(center))
		//{
		//	State_BIM_Plane state_temp = docs[i]->getType();
		//	state_temp.removeFloor();
		//	docs[i]->setType(state_temp);
		//	continue;
		//}

		cv::Mat img;
		proj_global_single(pts[i], *m_bbox_g, extent_x, extent_y, vl, ptVx, img);
		docs[i]->setImgProjGlobal(img);
		docs[i]->setPtVx2(ptVx);
		// export each projection image
		//std::stringstream ss;
		//ss << "tempdata/proj_" << i << ".jpg";
		//cv::imwrite(ss.str(), img);
		Util_Img::merge_c(img, img_g);
		img.release();
	}
	idx_floor.clear();
	for (size_t i = 0; i < n_model; ++i) {
		if (docs[i]->getType().isFloor())
		{
			idx_floor.push_back(i);
		}
	}
	// clear
	docs.clear();
	//delete m_bbox_g;
}
/**
* \brief project globally
* \param idx_area index list of models to get area/bbox for projection
* \param idx_proj index list of models to project
*/
void BIM_Proj::proj_global(
	Doc_BIM &doc_bim, std::vector<std::vector<cv::Point3f>> &pts,
	const std::vector<unsigned int> &idx_area,
	const std::vector<unsigned int> &idx_proj, cv::Mat &img_g)
{
	// data validation
	bool isValid = true;
	std::vector<Doc_BIM_Plane *> docs; // all docs
	doc_bim.getDocPlane(docs);
	if (docs.empty()) isValid = false; // check if docs init
	if (docs.size() != pts.size()) isValid = false; // check if doc and model match
	if (!isValid) {
		docs.clear();
		return;
	}
	// get idx needing processing
	size_t n_model = pts.size();

	// compute projection parameters
	cv::Point3f bbox_diag = m_bbox_g->m_max - m_bbox_g->m_min;
	float vl = MAX(bbox_diag.x, bbox_diag.y) / 512.f;
	unsigned int extent_x = bbox_diag.x / vl + 10;
	unsigned int extent_y = bbox_diag.y / vl + 10;
	// init global projection image
	img_g = cv::Mat::zeros(extent_y, extent_x, CV_8UC3);
	// project each model and append image to global one
	std::vector<cv::Point2i> ptVx;
	for (size_t i = 0; i < n_model; ++i) {
		// check if idx of doc exists in processing list
		if (!Util_Math::isIdxInList(i, idx_proj)) continue;
		cv::Mat img;
		proj_global_single(pts[i], *m_bbox_g, extent_x, extent_y, vl, ptVx, img);
		docs[i]->setImgProjGlobal(img);
		docs[i]->setPtVx2(ptVx);
		// export each projection image
		//std::stringstream ss;
		//ss << "tempdata/proj_" << i << ".jpg";
		//cv::imwrite(ss.str(), img);
		Util_Img::merge_c(img, img_g);
		img.release();
	}
	// clear
	docs.clear();
	/*delete m_bbox_g;*/
}
/**
* \brief project globally of single model
*/
void BIM_Proj::proj_global_single(const std::vector<cv::Point3f> &pts, const BBox3D &m_bbox_g,
	const unsigned int extent_x, const unsigned int extent_y, const float vl,
	std::vector<cv::Point2i> &ptVx, cv::Mat &img)
{
	size_t npts = pts.size();
	// project
	Proj2D *proj2d = new Proj2D();
	proj2d->setTrans2D(cv::Point2f(-m_bbox_g.m_min.x + 1, -m_bbox_g.m_min.y + 1));
	proj2d->setLenVoxel(vl);
	proj2d->setExtentX(extent_x);
	proj2d->setExtentY(extent_y);
	// proj2d->process(pts, img);
	proj2d->process(pts, ptVx, img);
	delete proj2d;
}

bool BIM_Proj::proj_global_ceil_beam_optimize(Doc_BIM & doc_bim,
	std::vector<std::vector<cv::Point3f>>& pts,
	const std::vector<unsigned int>& idx_area,
	const std::vector<unsigned int>& idx_ceiling,
	const std::vector<unsigned int>& idx_beam, cv::Mat & img_g)
{
	std::vector<Doc_BIM_Plane *> docs; // all docs
	doc_bim.getDocPlane(docs);
	// compute projection parameters
	cv::Point3f bbox_diag = m_bbox_g->m_max - m_bbox_g->m_min;
	float vl = MAX(bbox_diag.x, bbox_diag.y) / 512.f;  //why devided by 512?
	unsigned int extent_x = bbox_diag.x / vl + 10;
	unsigned int extent_y = bbox_diag.y / vl + 10;
	img_g = cv::Mat::zeros(extent_y, extent_x, CV_8UC3);
	//cv::Mat img_wall_bak = img_wall.clone();
	std::vector<uint> final_ceil_idx, final_ceil_idx_bak;
	int ceil_above_idx = -1;
	for (auto idx : idx_ceiling)
	{
		BBox3D ceil_box = docs[idx]->getBBox();
		if (ceil_box.m_min.x < 0.0f && ceil_box.m_max.x > 0.0f && ceil_box.m_min.y < 0.0f && ceil_box.m_max.y > 0.0f)
		{
			final_ceil_idx_bak.push_back(idx);
			unsigned int center_num = 0;
			unsigned int num_threshold = 100;
			float radius = 150.f;
			for (auto pt : pts[idx])
			{
				if (pt.x < radius && pt.x > -radius && pt.y < radius && pt.y > -radius)
				{
					center_num++;
				}
			}
			if (center_num < num_threshold) continue;
			ceil_above_idx = idx;
			final_ceil_idx.push_back(idx);
		}
	}
	if (ceil_above_idx == -1) return false;
	cv::Mat img;
	docs[ceil_above_idx]->getImgProjGlobal(img);
	Util_Img::merge_c(img, img_g);
	//std::string debug_path = "F:/Unre/BIM/test/108/test.jpg";
	//std::string debug_path2 = "F:/Unre/BIM/test/108/test2.jpg";
	//imwrite(debug_path, img_g);
	//imwrite(debug_path2, img);

	//Util_Img::merge_c(img, img_g);
	return true;
}