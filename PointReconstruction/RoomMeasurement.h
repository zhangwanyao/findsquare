#pragma once
#include "PointReconstruction.h"
#include "Measurement/MeasureLevelnessRange.h"
#include "Reconstruction/Reconstructer.h"
#include "..\concreteMesher.h"
#include "mesh_tool.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_mesher_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_vertex_base_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>
#include <CGAL/lloyd_optimize_mesh_2.h>
#include <CGAL/Boolean_set_operations_2.h>

typedef enum eRoomSquare_t
{
	SQUARE_BY_ROOMCONTOUR = 0,  //以最长边为参考
	SQUARE_BY_CUBOID = 1,    //内切长方体
	SQUARE_BY_CONVEXITY = 2, //墙面凸度    ——矩形找方（加了轴线就是轴线找方）
	SQUARE_BY_MIN_LOSS = 3,  //最小损失    ——多边形找方
}eRoomSquare_type;


struct ResultData
{
	MeasurementResultValueValuesPoints ground_levelness;
	std::vector<MeasurementRulerverticeStruct> ground_levelness_local_ruler_vertice_intersect_pts_dis;

	ResultData() {
		ground_levelness_local_ruler_vertice_intersect_pts_dis.resize(0);
	}
};

class RoomMeasurement
{
public:
	RoomMeasurement(PointReconstruction *pRecon);
	~RoomMeasurement();


public:
	/*
	function : Detect One Meter Line
	*/
	void DetectOneMeterLine(void);
	void DetectGroundAxisLine(void);//added by hgj

	ResultData MeasureGroundLevelness(void);

	bool MeasureStraightness(void);

	//bool MeasureDefectFcn(void);

	//void GetFlatenessDefect(std::vector<std::pair<int, std::vector<std::pair<float, cv::Point3f>>>>& flateness_defect);

	/*
	function : Find the Square line for Ground Contour

	parameters:
	ori_pts : the origin lines
	*/
	std::vector <std::pair< cv::Point3f, cv::Point3f>> MakeRoomSquare(
		cv::Mat &rotation_matrix,
		std::vector <std::pair< cv::Point3f, cv::Point3f>> &contour_squared1,
		std::vector<std::vector<std::vector<cv::Point3f>>> &holes_projected,
		std::vector <std::vector<StructuredHole>> &holes_sq,
		std::vector<std::vector<StructuredHole>> &holes_sq1,
		int square_height = -1, int square_width = -1);

	//added by hgj
	void GetContourAndHolesSquared(std::vector<cv::Point3f>&floor_contour_squared, std::vector<std::vector<std::vector<cv::Point3f>>>&hole_vertice_squared);
	bool HasOneMeterLine(void);

	bool HasGroundAxisLine(void);
	std::vector<std::pair<cv::Point3f, cv::Point3f>> GetGroundAxisLine(void) { return m_ground_axis_line; };

	float GetOneMerterLinePos(void) { return mOneMerterLineZ; };

	//added by hgj
	std::vector <std::pair< cv::Point3f, cv::Point3f>> GetContourSquared(void) { return contour_squared; };
	std::vector<std::vector<std::vector<cv::Point3f>>> GetHoleOrignal(void) { return holes_projected; };
	std::vector <std::vector<StructuredHole>> GetHoleSquared(void) { return holes_sq; };

private:
	double GetAngle(double x1, double y1, double x2, double y2, double x3, double y3);
	double GetAngleRel(double x1, double y1, double x2, double y2, double x3, double y3);
	cv::Point2f GetFootOfPerpendicular(const cv::Point2f & pt, const cv::Point2f & begin, const cv::Point2f & end);
	cv::Point3f GetFootOfPerpendicular3f(const cv::Point3f & pt, const cv::Point3f & begin, const cv::Point3f & end);
	void getLinePara(cv::Point2f p1, cv::Point2f p2, cv::Point2f & Pa);
	bool isPositive(cv::Point2f p0, cv::Point2f p1, cv::Point2f p2);
	bool getCorssedPoint(cv::Point2f pa, cv::Point2f pb, cv::Point2f p1, cv::Point2f p2, cv::Point2f & cp);
	void getTempPoint(cv::Point2f pt0, cv::Point2f pt1, cv::Point2f pt2, cv::Point2f & cp);
	std::vector<cv::Point2f> ContourSquare(std::vector<cv::Point3f>& pts0,std::vector<cv::Point3f>& pts, float &LAera, int ref_wall_contour_id,
		                                                   std::vector<int>& wall_list);
	//added by hgj
	bool GetSquaredFloorContourPoint3dHlper(const std::vector <std::pair< cv::Point2f, cv::Point2f>>& CuboidContourSquaredJson,
		const std::vector<cv::Point3f>& coutour_ori,
		std::vector<cv::Point3f>& countour_squared);
	//added by hgj
	bool GetSquaredHolePoint3dHlper(const std::vector<StructuredPlane>& mSPlane,
		std::vector<std::vector<std::vector<cv::Point3f>>>& vec_vec_vertice_s);
public:
	ResultData leveness;
	std::vector<std::vector<std::vector<cv::Point3f>>> holes_projected;
	std::vector <std::pair< cv::Point3f, cv::Point3f>> contour_squared;
	std::vector <std::pair< cv::Point3f, cv::Point3f>> contour_squared1;
	std::vector <cv::Point3f> contour_squared05;
	std::vector <float> contour_squared05_len;
	std::vector <std::vector<StructuredHole>> holes_sq;
	std::vector <std::vector<StructuredHole>> holes_sq1;
	std::vector<cv::Point3f> insertedPts;
	std::vector <int> insertedWallList;
	std::vector <int> insertMap;
	cv::Mat rotation_matrix = cv::Mat::zeros(3, 3, CV_32FC1);
	std::vector<Eigen::Vector4d> AxisLines;
	std::vector<std::pair<cv::Point3f, cv::Point3f>> markers_pairs;
	std::vector<std::pair<int, std::pair<cv::Point3f, cv::Point3f>>> markers_pairs_with_pid;
	std::vector<cv::Point3f> convPts;
	std::vector<bool> convPtsF;
	std::vector<cv::Point3f> wallConvPt;
	std::string axisEqnJsonPath;
	std::vector <std::pair <int, std::vector <std::pair< float, cv::Point3f>>>> ceiling_h_onemeter;
	std::vector <std::pair <int, std::vector <std::pair< float, cv::Point3f>>>> ground_h_onemeter;
	std::vector<std::vector<std::pair< cv::Point3f, cv::Point3f>>> ceiling_h_onemeter_int;
	std::vector<std::vector<std::pair< cv::Point3f, cv::Point3f>>> ground_h_onemeter_int;
	std::vector<std::pair<float, int>>wall_width_input;
	std::vector<std::pair<float, float>>wall_width_info;
	std::vector<std::vector<std::pair<float, cv::Point3f>>> squared_defect;
	std::vector<std::vector<std::pair<float, cv::Point3f>>> squared_defect_info;
	std::vector<std::pair<float, float>> square_defect_max;
	std::vector<std::pair<float, float>> filling_vol;
	std::vector<float> filling_vol_opt;
	std::vector<std::pair<float, float>> walls_squareness;

private:

	bool mHasOneMeterLine;//added by hgj
	float mOneMerterLineZ;//added by hgj

	bool mHasGroundAxisLine;//added by hgj
	std::vector<std::pair<cv::Point3f, cv::Point3f>> m_ground_axis_line;//added by hgj

	std::vector<std::pair<int, std::vector<std::pair<float, cv::Point3f>>>> flateness_defect;

	std::vector<std::vector<std::vector<cv::Point3f>>>m_hole_vertice_squared;//added by hgj
	std::vector<cv::Point3f> m_floor_contour_squared; //added by hgj
	std::vector<std::pair<std::pair<int,int>,float>> m_straightness;

	PointReconstruction *mPRecon;
public:
	typedef CGAL::Simple_cartesian<double>                                           Cartesian;
	typedef CGAL::Exact_predicates_inexact_constructions_kernel                        Epick;

	typedef Cartesian::Point_2										Point_2;
	typedef CGAL::Polygon_2<Cartesian>								Polygon_2;
	typedef CGAL::Polygon_with_holes_2<Cartesian>					Polygon_with_holes_2;
	typedef std::list<Polygon_with_holes_2>							Pwh_list_2;

	Reconstructer reconstructer;

	cv::Point2d getTriCentroid(const CGAL::Point_2<CGAL::Epick>& p0, const CGAL::Point_2<CGAL::Epick>& p1, const CGAL::Point_2<CGAL::Epick>& p2)
	{
		double x_Ce, y_Ce;
		x_Ce = (p0.x() + p1.x() + p2.x()) / 3;
		y_Ce = (p0.y() + p1.y() + p2.y()) / 3;

		cv::Point2d triCentroid{ x_Ce, y_Ce };

		return triCentroid;
	};

	cv::Point2d getTriCentroid(const cv::Point2f &p0, const cv::Point2f &p1, const cv::Point2f &p2)
	{
		double x_Ce, y_Ce;
		x_Ce = (p0.x + p1.x + p2.x) / 3;
		y_Ce = (p0.y + p1.y + p2.y) / 3;

		cv::Point2d triCentroid{ x_Ce, y_Ce };

		return triCentroid;
	}

	bool PtInPolygon(const cv::Point2d& triCentroid, const std::vector<cv::Point2d>& contour_squared_floor, int &nCount)
	{
		int nCross = 0;
		for (int i = 0; i < nCount; i++)
		{
			cv::Point2d p1 = contour_squared_floor[i];
			cv::Point2d p2 = contour_squared_floor[(i + 1) % nCount];

			if (p1.y == p2.y)
				continue;
			if (triCentroid.y < min(p1.y, p2.y))
				continue;
			if (triCentroid.y >= max(p1.y, p2.y))
				continue;

			double x = (double)(triCentroid.y - p1.y) * (double)(p2.x - p1.x) / (double)(p2.y - p1.y) + p1.x;

			if (x > triCentroid.x)
				nCross++;
		}

		return nCross % 2 == 1;
	};

	int find_index(std::vector<cv::Point2d> &polygon_2_corners, cv::Point2d &tri_corner)
	{
		for (int index = 0; index < polygon_2_corners.size(); index++)
		{
			if (polygon_2_corners[index].x == tri_corner.x && polygon_2_corners[index].y == tri_corner.y)
			{
				return index;
				break;
			}
		}
	};
	
	int find_index(std::vector<cv::Point3f> &temp_hole, cv::Point2f &intersect2Y)
	{
		for (int index = 0; index < temp_hole.size(); index++)
		{
			double deta_y = 0, deta_z = 0;
			deta_y = abs(temp_hole[index].y - intersect2Y.x);
			deta_z = abs(temp_hole[index].z - intersect2Y.y);
			if (deta_y < 0.5 && deta_z < 0.5)
			{
				return index;
				break;
			}
		}
	};

	float getTdAngle(cv::Point3f &wall_normal, cv::Point3f &ground_normal) 
	{	
		float vectorDot = wall_normal.x * ground_normal.x + wall_normal.y * ground_normal.y + wall_normal.z * ground_normal.z;
		double vectorMold1 = sqrt(pow(wall_normal.x, 2) + pow(wall_normal.y, 2) + pow(wall_normal.z, 2));
		double vectorMold2 = sqrt(pow(ground_normal.x, 2) + pow(ground_normal.y, 2) + pow(ground_normal.z, 2));

		double cosAngle = vectorDot / (vectorMold1 * vectorMold2);
		double radian = acos(cosAngle);

		return (float)(180 / M_PI * radian);
	};

	bool PointCmp(const cv::Point2f &a, const cv::Point2f &b, const cv::Point2f &center)
	{
		if (a.x >= 0 && b.x < 0)
			return true;
		if (a.x == 0 && b.x == 0)
			return a.y > b.y;
		
		float det = (a.x - center.x) * (b.y - center.y) - (b.x - center.x) * (a.y - center.y);
		if (det < 0)
			return true;
		if (det > 0)
			return false;
		
		float d1 = (a.x - center.x) * (a.x - center.x) + (a.y - center.y) * (a.y - center.y);
		float d2 = (b.x - center.x) * (b.x - center.y) + (b.y - center.y) * (b.y - center.y);
		return d1 > d2;
	}

	void ClockwiseSortPoints(std::vector<std::pair< cv::Point2f, float>> &vPoints)
	{
		cv::Point2f center;
		double x = 0, y = 0;
		for (int i = 0; i < vPoints.size(); i++)
		{
			x += vPoints[i].first.x;
			y += vPoints[i].first.y;
		}
		center.x = x / vPoints.size();
		center.y = y / vPoints.size();

		for (int i = 0; i < vPoints.size() - 1; i++)
		{
			for (int j = 0; j < vPoints.size() - i - 1; j++)
			{
				if (PointCmp(vPoints[j].first, vPoints[j + 1].first, center))
				{
					cv::Point2f tmp = vPoints[j].first;
					float temp_z = vPoints[j].second;
					vPoints[j].first = vPoints[j + 1].first;
					vPoints[j].second = vPoints[j + 1].second;
					vPoints[j + 1].first = tmp;
					vPoints[j + 1].second = temp_z;
				}
			}
		}
	}

	auto constructMeshByContour_squared(std::vector <std::pair< cv::Point3f, cv::Point3f>>& contour_squared, std::vector<StructuredPlane> &squaredPlanes, string path = "")
	{
		typedef CGAL::Simple_cartesian<float> Kernel;
		std::vector<StructuredPlane> mSPlane = mPRecon->GetStructuredPlanes();
		
		float ground_max = 0, ceiling_min = 0;
		for (auto x : mSPlane) {
			if (x.type == ePLANE_GROUND)
				ground_max = std::max_element(x.vertices.begin(), x.vertices.end(),
					[](cv::Point3f a, cv::Point3f b) {return a.z < b.z; })->z;
			if (x.type == ePLANE_CEILING)
				ceiling_min = std::min_element(x.vertices.begin(), x.vertices.end(),
					[](cv::Point3f a, cv::Point3f b) {return a.z < b.z; })->z;;
		}
		typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
		typedef CGAL::Delaunay_mesh_vertex_base_2<K>                Vb;
		typedef CGAL::Delaunay_mesh_face_base_2<K>                  Fb;
		typedef CGAL::Triangulation_data_structure_2<Vb, Fb>        Tds;
		typedef CGAL::Constrained_Delaunay_triangulation_2<K, Tds>  CDT;
		typedef CGAL::Delaunay_mesh_size_criteria_2<CDT>            Criteria;
		typedef CGAL::Delaunay_mesher_2<CDT, Criteria>              Mesher;
		Reconstructer reconstructer;
		concreteMesherDelauney mesher;

		CDT cdt;
		CDT cdt1;
		CDT cdt2;
		CDT::Point pre(contour_squared.back().first.x, contour_squared.back().first.y);
		for (auto& e : contour_squared) {
			cdt.insert_constraint(pre, { e.first.x,e.first.y });
			pre = { e.first.x,e.first.y };
		}

		std::vector<cv::Point2d> contour_squared_floor;
		contour_squared_floor.clear();
		for (auto& f : contour_squared)
		{
			if (f.first.z <= ground_max + 10)
			{
				cv::Point2d floor_vertex{ f.first.x, f.first.y };

				contour_squared_floor.emplace_back(floor_vertex);
			}
		}

		//floor
		std::vector<CGAL::Surface_mesh<Kernel::Point_3>> surface_mesh_components;
		CGAL::Surface_mesh<Kernel::Point_3> mesh;

		//???????
		std::vector<std::vector<cv::Point3f>> wall_plane_list;
		for (int i = 0; i < squaredPlanes.size(); i++)
		{
			if (squaredPlanes[i].type == ePLANE_WALL)
			{
				std::vector<Point3f> plane_corners;
				plane_corners.clear();

				for (int j = 0; j < squaredPlanes[i].vertices.size(); j++)
				{
					if (squaredPlanes[i].vertices[j].z <= ground_max + 100)
					{
						plane_corners.emplace_back(cv::Point3f(squaredPlanes[i].vertices[j].x, squaredPlanes[i].vertices[j].y, squaredPlanes[i].vertices[j].z));
					}
				}
				float deta_x2 = mSPlane[i].vertices[1].x - squaredPlanes[i].vertices[1].x;
				float deta_y2 = mSPlane[i].vertices[1].y - squaredPlanes[i].vertices[1].y;
				plane_corners.emplace_back(cv::Point3f(mSPlane[i].vertices[2].x - deta_x2, mSPlane[i].vertices[2].y - deta_y2, squaredPlanes[i].vertices[2].z));

				float deta_x3 = mSPlane[i].vertices[0].x - squaredPlanes[i].vertices[0].x;
				float deta_y3 = mSPlane[i].vertices[0].y - squaredPlanes[i].vertices[0].y;
				plane_corners.emplace_back(cv::Point3f(mSPlane[i].vertices[3].x - deta_x3, mSPlane[i].vertices[3].y - deta_y3, squaredPlanes[i].vertices[3].z));

				wall_plane_list.emplace_back(plane_corners);
			}
		}

		std::vector<std::vector<cv::Point3f>> squaredWallList;
		for (int i = 0; i < squaredPlanes.size(); i++)
		{
			if (squaredPlanes[i].type == ePLANE_WALL)
			{
				std::vector<cv::Point3f> wallCorners;
				wallCorners.clear();
				for (int j = 0; j < squaredPlanes[i].vertices.size(); j++)
				{
					wallCorners.emplace_back(cv::Point3f(squaredPlanes[i].vertices[j].x, squaredPlanes[i].vertices[j].y, squaredPlanes[i].vertices[j].z));
				}
				squaredWallList.emplace_back(wallCorners);
			}
		}

		std::vector<cv::Point3f> ceiling_plane_corners;
		ceiling_plane_corners.clear();
		std::vector<std::pair< cv::Point2f, float>> ceiling_corners_2_z;
		ceiling_corners_2_z.clear();
		for (int i = 0; i < squaredWallList.size(); i++)
		{
			int nPnt = 0;
			for (int j = 0; j < squaredWallList[i].size(); j++)
			{
				std::pair< cv::Point2f, float> ceiling_corner_2_z;
				cv::Point2f ceiling_corner_2;
				float ceiling_z = 0;
				if (squaredWallList[i][j].z > ground_max + 1000)
				{
					//cout << wall_plane_list[i][j].x << " " << wall_plane_list[i][j].y << " " << wall_plane_list[i][j].z << endl;
					ceiling_corner_2 = cv::Point2f(squaredWallList[i][j].x, squaredWallList[i][j].y);
					ceiling_z = squaredWallList[i][j].z;
					ceiling_corner_2_z.first = ceiling_corner_2;
					ceiling_corner_2_z.second = ceiling_z;
					ceiling_corners_2_z.emplace_back(ceiling_corner_2_z);
					
					nPnt += 1;
				}
				/*if (nPnt == 1)
				{
					break;
				}*/
			}
			/*if (nPnt == 1)
			{
				continue;
			}*/
		}

		for (int i = 0; i < contour_squared.size(); i++)
		{
			cv::Point3f floor_plane_corners = contour_squared[i].first;
			std::vector<float> deta_vec;

			for (int j = 0; j < ceiling_corners_2_z.size(); j++)
			{
				float deta_xy = 0;
				deta_xy = sqrt((floor_plane_corners.x - ceiling_corners_2_z[j].first.x)*(floor_plane_corners.x - ceiling_corners_2_z[j].first.x) + (floor_plane_corners.y - ceiling_corners_2_z[j].first.y)*(floor_plane_corners.y - ceiling_corners_2_z[j].first.y));
				deta_vec.emplace_back(deta_xy);
			}

			std::vector<float>::iterator smallest = std::min_element(deta_vec.begin(), deta_vec.end());

			int min_value_index = std::distance(deta_vec.begin(), smallest);

			/*cout << " 最小差值的索引： " << min_value_index << endl;

			cout << " 最小距离： " << deta_vec[min_value_index] << endl;*/

			ceiling_plane_corners.emplace_back(cv::Point3f(ceiling_corners_2_z[min_value_index].first.x, ceiling_corners_2_z[min_value_index].first.y, ceiling_corners_2_z[min_value_index].second));

			deta_vec.clear();
		}

		/*ClockwiseSortPoints(ceiling_corners_2_z);

		for (int i = 0; i < ceiling_corners_2_z.size(); i++)
		{
			ceiling_plane_corners.emplace_back(cv::Point3f(ceiling_corners_2_z[i].first.x, ceiling_corners_2_z[i].first.y, ceiling_corners_2_z[i].second));
		}*/

		if (ceiling_plane_corners.size() % 2 != 0)
		{
			ceiling_plane_corners.clear();
			
			for (int i = 0; i < contour_squared_floor.size(); i++)
			{
				ceiling_plane_corners.emplace_back(cv::Point3f(contour_squared_floor[i].x, contour_squared_floor[i].y, ceiling_corners_2_z[i].second));
			}
		}

		std::vector < std::pair<int, cv::Point3f>> walls_index_normal;
		walls_index_normal.clear();

		for (int plane_index = 0; plane_index < squaredPlanes.size(); plane_index++)
		{
			//wall
			if (squaredPlanes[plane_index].type == ePLANE_WALL)
			{
				Eigen::Matrix4d matrix;
				matrix = reconstructer.projectVP(squaredPlanes[plane_index].vertices, {}, {});
				std::vector<ModuleStruct::Point3fArray> edgePointsList;
				std::vector<std::vector<ModuleStruct::Point3fArray>> holePointsList;

				std::vector<cv::Point3f> plane_polygon_corners;
				plane_polygon_corners.clear();
				if (squaredPlanes[plane_index].vertices.size() > 0)
				{
					for (int j = 0; j < 4; j++)
					{
						plane_polygon_corners.emplace_back(squaredPlanes[plane_index].vertices[j]);
					}
				}

				std::vector<std::vector<cv::Point3f>> holes_vec;
				holes_vec.clear();
				if (squaredPlanes[plane_index].holes.size() > 0)
				{
					for (int j = 0; j < squaredPlanes[plane_index].holes.size(); j++)
					{
						std::vector<cv::Point3f> hole_corners;
						for (int k = 0; k < squaredPlanes[plane_index].holes[j].vertice.size(); k++)
						{
							hole_corners.emplace_back(squaredPlanes[plane_index].holes[j].vertice[k]);
						}
						holes_vec.emplace_back(hole_corners);
						hole_corners.clear();
					}
				}

				ModuleStruct::Point3fArray tempEdge;

				reconstructer.transformatVecCvP(plane_polygon_corners, matrix, tempEdge);
				edgePointsList.push_back(tempEdge);

				Polygon_2 P;
				if (tempEdge.size() > 0)
				{
					for (auto x : tempEdge)
						P.push_back(Point_2(x.x, x.y));
					if (P.orientation() == -1)
						P.reverse_orientation();
				}

				std::vector<Point3fArray> holelist;

				if (holes_vec.size() > 0)
				{
					for (int j = 0; j < holes_vec.size(); j++)
					{
						ModuleStruct::Point3fArray tempHole;
						reconstructer.transformatVecCvP(holes_vec[j], matrix, tempHole);
						holelist.push_back(tempHole);
					}
				}

				holePointsList.push_back(holelist);

				Eigen::Matrix4d matrix_inverse = matrix.inverse();
				mesher.buildMesh(edgePointsList, holePointsList, matrix_inverse, path + "_squared_" + std::to_string(plane_index) + ".obj");
			}

			//ground
			if (squaredPlanes[plane_index].type == ePLANE_GROUND)
			{
				for (auto itr = cdt.faces_begin(); itr != cdt.faces_end(); itr++) {
					auto p0 = cdt.point(itr->vertex(0));
					auto p1 = cdt.point(itr->vertex(1));
					auto p2 = cdt.point(itr->vertex(2));

					cv::Point2f triCentroid = getTriCentroid(p0, p1, p2);
					int nCount = contour_squared_floor.size();
					if (!PtInPolygon(triCentroid, contour_squared_floor, nCount))
					{
						continue;
					}

					auto v0 = mesh.add_vertex({ p0.x(),p0.y(),ground_max });
					auto v1 = mesh.add_vertex({ p1.x(),p1.y(),ground_max });
					auto v2 = mesh.add_vertex({ p2.x(),p2.y(),ground_max });
					mesh.add_face(v0, v1, v2);
				}
				CGAL::IO::write_OBJ(path + "_squared_" + std::to_string(plane_index) + ".obj", mesh);
				mesh.clear();
			}
			//ceiling
			if (squaredPlanes[plane_index].type == ePLANE_CEILING)
			{
				
				Eigen::Matrix4d matrix;
				matrix = reconstructer.projectVP(ceiling_plane_corners, {}, {});
				//matrix = reconstructer.projectVP(ceiling_plane_corners, {}, {});
				std::vector<ModuleStruct::Point3fArray> edgePointsList;
				std::vector<std::vector<ModuleStruct::Point3fArray>> holePointsList;
				ModuleStruct::Point3fArray tempEdge;

				reconstructer.transformatVecCvP(ceiling_plane_corners, matrix, tempEdge);
				edgePointsList.push_back(tempEdge);

				Polygon_2 P;
				if (tempEdge.size() > 0)
				{
					for (auto x : tempEdge)
						P.push_back(Point_2(x.x, x.y));
					if (P.orientation() == -1)
						P.reverse_orientation();
				}
				std::vector<ModuleStruct::Point3fArray> temphole;
				temphole.clear();
				holePointsList.push_back(temphole);

				Eigen::Matrix4d matrix_inverse = matrix.inverse();
				mesher.buildMesh(edgePointsList, holePointsList, matrix_inverse, path + "_squared_" + std::to_string(plane_index) + ".obj");
			}
		}

		std::vector<cv::Point3f> g_obj_points;
		std::vector<cv::Vec3i> g_face;
		size_t found_squared;
		found_squared = path.find_last_of("\\/");
		std::string curPath_squared = path.substr(0, found_squared);
		std::string filename_squared = path.substr(found_squared + 1, path.length());
		found_squared = curPath_squared.find_last_of("\\/");
		std::string parentPath_squared = curPath_squared.substr(0, found_squared);
		ofstream outtriangles(parentPath_squared + "//" + filename_squared + "_squared.obj");
		int idx_p = 0;
		int idx_f = 0;
		CMeshTool meshTool;
		for (int i = 0; i < squaredPlanes.size(); i++) {

			if (squaredPlanes.size() == 0)
				continue;
			std::vector<cv::Point3f> obj_points;
			std::vector<cv::Vec3i> face;
			std::string obj_path = path + "_squared_" + to_string(i) + ".obj";
			bool isGet = meshTool.GetObjData(obj_path, obj_points, face);
			if (!isGet) {
				std::cout << "meshTool.GetObjData fail! file:" << obj_path.c_str() << std::endl;
			}
			else {
				for (int j = 0; j < obj_points.size(); j++) {
					g_obj_points.push_back(obj_points[j]);
				}
				for (int j = 0; j < face.size(); j++) {
					cv::Vec3i tmp(idx_f, idx_f, idx_f);
					g_face.push_back(face[j] + tmp);
				}
				idx_f += obj_points.size();
			}
		}
		for (int i = 0; i < g_obj_points.size(); i++)
			outtriangles << "v " << g_obj_points[i].x << " " << g_obj_points[i].y << " " << g_obj_points[i].z << endl;
		for (int i = 0; i < g_face.size(); i++)
			outtriangles << "f " << g_face[i][0] << " " << g_face[i][1] << " " << g_face[i][2] << endl;
		outtriangles.close();

		ofstream out(parentPath_squared + "//" + "WallGroundAngle.txt");
		for (int i = 0; i < walls_index_normal.size(); i++)
		{
			cv::Point3f wall_normal = walls_index_normal[i].second;
			cv::Point3f ground_normal{ 0, 0, 1 };
			float WG_Angle = 0;
			WG_Angle = getTdAngle(wall_normal, ground_normal);

			out << "wall_id:" << walls_index_normal[i].first << "  WallandGround_Angle:" << WG_Angle << endl;
		}
		out.close();
	};

	auto constructMeshByContour_squared(std::vector<cv::Point3f>& contour_squared05, std::vector<StructuredPlane> &squaredPlanes, string path = "")
	{
		typedef CGAL::Simple_cartesian<float> Kernel;
		std::vector<StructuredPlane> mSPlane = mPRecon->GetStructuredPlanes();

		float ground_max = 0, ceiling_min = 0;
		for (auto x : mSPlane) {
			if (x.type == ePLANE_GROUND)
				ground_max = std::max_element(x.vertices.begin(), x.vertices.end(),
					[](cv::Point3f a, cv::Point3f b) {return a.z < b.z; })->z;
			if (x.type == ePLANE_CEILING)
				ceiling_min = std::min_element(x.vertices.begin(), x.vertices.end(),
					[](cv::Point3f a, cv::Point3f b) {return a.z < b.z; })->z;;
		}
		typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
		typedef CGAL::Delaunay_mesh_vertex_base_2<K>                Vb;
		typedef CGAL::Delaunay_mesh_face_base_2<K>                  Fb;
		typedef CGAL::Triangulation_data_structure_2<Vb, Fb>        Tds;
		typedef CGAL::Constrained_Delaunay_triangulation_2<K, Tds>  CDT;
		typedef CGAL::Delaunay_mesh_size_criteria_2<CDT>            Criteria;
		typedef CGAL::Delaunay_mesher_2<CDT, Criteria>              Mesher;
		Reconstructer reconstructer;
		concreteMesherDelauney mesher;

		CDT cdt;
		CDT cdt1;
		CDT cdt2;
		CDT::Point pre(contour_squared05.back().x, contour_squared05.back().y);
		for (auto& e : contour_squared05) {
			cdt.insert_constraint(pre, { e.x,e.y });
			pre = { e.x,e.y };
		}

		std::vector<cv::Point2d> contour_squared_floor;
		contour_squared_floor.clear();
		for (auto& f : contour_squared05)
		{
			if (f.z <= ground_max + 100)
			{
				cv::Point2d floor_vertex{ f.x, f.y };

				contour_squared_floor.emplace_back(floor_vertex);
			}
		}

		//floor
		std::vector<CGAL::Surface_mesh<Kernel::Point_3>> surface_mesh_components;
		CGAL::Surface_mesh<Kernel::Point_3> mesh;

		//???????
		std::vector<std::vector<cv::Point3f>> wall_plane_list;
		for (int i = 0; i < squaredPlanes.size(); i++)
		{
			if (squaredPlanes[i].type == ePLANE_WALL)
			{
				std::vector<Point3f> plane_corners;
				plane_corners.clear();

				for (int j = 0; j < squaredPlanes[i].vertices.size(); j++)
				{
					if (squaredPlanes[i].vertices[j].z <= ground_max + 100)
					{
						plane_corners.emplace_back(cv::Point3f(squaredPlanes[i].vertices[j].x, squaredPlanes[i].vertices[j].y, squaredPlanes[i].vertices[j].z));
					}
				}
				float deta_x2 = mSPlane[i].vertices[1].x - squaredPlanes[i].vertices[1].x;
				float deta_y2 = mSPlane[i].vertices[1].y - squaredPlanes[i].vertices[1].y;
				plane_corners.emplace_back(cv::Point3f(mSPlane[i].vertices[2].x - deta_x2, mSPlane[i].vertices[2].y - deta_y2, squaredPlanes[i].vertices[2].z));

				float deta_x3 = mSPlane[i].vertices[0].x - squaredPlanes[i].vertices[0].x;
				float deta_y3 = mSPlane[i].vertices[0].y - squaredPlanes[i].vertices[0].y;
				plane_corners.emplace_back(cv::Point3f(mSPlane[i].vertices[3].x - deta_x3, mSPlane[i].vertices[3].y - deta_y3, squaredPlanes[i].vertices[3].z));

				wall_plane_list.emplace_back(plane_corners);
			}
		}

		std::vector<std::vector<cv::Point3f>> squaredWallList;
		for (int i = 0; i < squaredPlanes.size(); i++)
		{
			if (squaredPlanes[i].type == ePLANE_WALL)
			{
				std::vector<cv::Point3f> wallCorners;
				wallCorners.clear();
				for (int j = 0; j < squaredPlanes[i].vertices.size(); j++)
				{
					wallCorners.emplace_back(cv::Point3f(squaredPlanes[i].vertices[j].x, squaredPlanes[i].vertices[j].y, squaredPlanes[i].vertices[j].z));
				}
				squaredWallList.emplace_back(wallCorners);
			}
		}

		std::vector<cv::Point3f> ceiling_plane_corners;
		ceiling_plane_corners.clear();
		std::vector<std::pair< cv::Point2f, float>> ceiling_corners_2_z;
		ceiling_corners_2_z.clear();
		for (int i = 0; i < squaredWallList.size(); i++)
		{
			int nPnt = 0;
			for (int j = 0; j < squaredWallList[i].size(); j++)
			{
				std::pair< cv::Point2f, float> ceiling_corner_2_z;
				cv::Point2f ceiling_corner_2;
				float ceiling_z = 0;
				if (squaredWallList[i][j].z > ground_max + 1000)
				{
					//cout << wall_plane_list[i][j].x << " " << wall_plane_list[i][j].y << " " << wall_plane_list[i][j].z << endl;
					ceiling_corner_2 = cv::Point2f(squaredWallList[i][j].x, squaredWallList[i][j].y);
					ceiling_z = squaredWallList[i][j].z;
					ceiling_corner_2_z.first = ceiling_corner_2;
					ceiling_corner_2_z.second = ceiling_z;
					ceiling_corners_2_z.emplace_back(ceiling_corner_2_z);

					nPnt += 1;
				}
				/*if (nPnt == 1)
				{
					break;
				}*/
			}
			/*if (nPnt == 1)
			{
				continue;
			}*/
		}

		for (int i = 0; i < contour_squared05.size(); i++)
		{
			cv::Point3f floor_plane_corners = contour_squared05[i];
			std::vector<float> deta_vec;

			for (int j = 0; j < ceiling_corners_2_z.size(); j++)
			{
				float deta_xy = 0;
				deta_xy = sqrt((floor_plane_corners.x - ceiling_corners_2_z[j].first.x)*(floor_plane_corners.x - ceiling_corners_2_z[j].first.x) + (floor_plane_corners.y - ceiling_corners_2_z[j].first.y)*(floor_plane_corners.y - ceiling_corners_2_z[j].first.y));
				deta_vec.emplace_back(deta_xy);
			}

			std::vector<float>::iterator smallest = std::min_element(deta_vec.begin(), deta_vec.end());

			int min_value_index = std::distance(deta_vec.begin(), smallest);

			/*cout << " 最小差值的索引： " << min_value_index << endl;

			cout << " 最小距离： " << deta_vec[min_value_index] << endl;*/

			ceiling_plane_corners.emplace_back(cv::Point3f(ceiling_corners_2_z[min_value_index].first.x, ceiling_corners_2_z[min_value_index].first.y, ceiling_corners_2_z[min_value_index].second));

			deta_vec.clear();
		}

		if (ceiling_plane_corners.size() % 2 != 0)
		{
			ceiling_plane_corners.clear();

			for (int i = 0; i < contour_squared_floor.size(); i++)
			{
				ceiling_plane_corners.emplace_back(cv::Point3f(contour_squared_floor[i].x, contour_squared_floor[i].y, ceiling_corners_2_z[i].second));
			}
		}

		std::vector < std::pair<int, cv::Point3f>> walls_index_normal;
		walls_index_normal.clear();

		for (int plane_index = 0; plane_index < squaredPlanes.size(); plane_index++)
		{
			//wall
			if (squaredPlanes[plane_index].type == ePLANE_WALL)
			{
				Eigen::Matrix4d matrix;
				matrix = reconstructer.projectVP(squaredPlanes[plane_index].vertices, {}, {});
				std::vector<ModuleStruct::Point3fArray> edgePointsList;
				std::vector<std::vector<ModuleStruct::Point3fArray>> holePointsList;

				std::vector<cv::Point3f> plane_polygon_corners;
				plane_polygon_corners.clear();
				if (squaredPlanes[plane_index].vertices.size() > 0)
				{
					for (int j = 0; j < 4; j++)
					{
						plane_polygon_corners.emplace_back(squaredPlanes[plane_index].vertices[j]);
					}
				}

				std::vector<cv::Point2d> plane_polygon_corners_2;
				plane_polygon_corners_2.clear();
				for (int i = 0; i < plane_polygon_corners.size(); i++)
				{
					plane_polygon_corners_2.emplace_back(cv::Point2d(plane_polygon_corners[i].x, plane_polygon_corners[i].z));
				}

				std::vector<std::vector<cv::Point3f>> holes_vec;
				holes_vec.clear();
				int nHoleCorners = 0;
				if (squaredPlanes[plane_index].holes.size() > 0)
				{
					for (int j = 0; j < squaredPlanes[plane_index].holes.size(); j++)
					{
						std::vector<cv::Point3f> hole_corners;
						bool PtInPoly;
						for (int k = 0; k < squaredPlanes[plane_index].holes[j].vertice.size(); k++)
						{
							hole_corners.emplace_back(squaredPlanes[plane_index].holes[j].vertice[k]);
						}
						holes_vec.emplace_back(hole_corners);
						hole_corners.clear();
					}
				}

				ModuleStruct::Point3fArray tempEdge;

				reconstructer.transformatVecCvP(plane_polygon_corners, matrix, tempEdge);
				edgePointsList.push_back(tempEdge);

				Polygon_2 P;
				if (tempEdge.size() > 0)
				{
					for (auto x : tempEdge)
						P.push_back(Point_2(x.x, x.y));
					if (P.orientation() == -1)
						P.reverse_orientation();
				}

				std::vector<Point3fArray> holelist;

				if (holes_vec.size() > 0)
				{
					for (int j = 0; j < holes_vec.size(); j++)
					{
						ModuleStruct::Point3fArray tempHole;
						reconstructer.transformatVecCvP(holes_vec[j], matrix, tempHole);
						holelist.push_back(tempHole);
					}
				}

				holePointsList.push_back(holelist);

				Eigen::Matrix4d matrix_inverse = matrix.inverse();
				mesher.buildMesh(edgePointsList, holePointsList, matrix_inverse, path + "_squared05_" + std::to_string(plane_index) + ".obj");
			}

			//ground
			if (squaredPlanes[plane_index].type == ePLANE_GROUND)
			{
				for (auto itr = cdt.faces_begin(); itr != cdt.faces_end(); itr++) {
					auto p0 = cdt.point(itr->vertex(0));
					auto p1 = cdt.point(itr->vertex(1));
					auto p2 = cdt.point(itr->vertex(2));

					cv::Point2f triCentroid = getTriCentroid(p0, p1, p2);
					int nCount = contour_squared_floor.size();
					if (!PtInPolygon(triCentroid, contour_squared_floor, nCount))
					{
						continue;
					}

					auto v0 = mesh.add_vertex({ p0.x(),p0.y(),ground_max });
					auto v1 = mesh.add_vertex({ p1.x(),p1.y(),ground_max });
					auto v2 = mesh.add_vertex({ p2.x(),p2.y(),ground_max });
					mesh.add_face(v0, v1, v2);
				}
				CGAL::IO::write_OBJ(path + "_squared05_" + std::to_string(plane_index) + ".obj", mesh);
				mesh.clear();
			}
			//ceiling
			if (squaredPlanes[plane_index].type == ePLANE_CEILING)
			{
				Eigen::Matrix4d matrix;
				matrix = reconstructer.projectVP(ceiling_plane_corners, {}, {});
				//matrix = reconstructer.projectVP(ceiling_plane_corners, {}, {});
				std::vector<ModuleStruct::Point3fArray> edgePointsList;
				std::vector<std::vector<ModuleStruct::Point3fArray>> holePointsList;
				ModuleStruct::Point3fArray tempEdge;

				reconstructer.transformatVecCvP(ceiling_plane_corners, matrix, tempEdge);
				edgePointsList.push_back(tempEdge);

				Polygon_2 P;
				if (tempEdge.size() > 0)
				{
					for (auto x : tempEdge)
						P.push_back(Point_2(x.x, x.y));
					if (P.orientation() == -1)
						P.reverse_orientation();
				}
				std::vector<ModuleStruct::Point3fArray> temphole;
				temphole.clear();
				holePointsList.push_back(temphole);

				Eigen::Matrix4d matrix_inverse = matrix.inverse();
				mesher.buildMesh(edgePointsList, holePointsList, matrix_inverse, path + "_squared05_" + std::to_string(plane_index) + ".obj");
			}
		}

		std::vector<cv::Point3f> g_obj_points;
		std::vector<cv::Vec3i> g_face;
		size_t found05;
		found05 = path.find_last_of("\\/");
		std::string curPath05 = path.substr(0, found05);
		std::string filename05 = path.substr(found05 + 1, path.length());
		found05 = curPath05.find_last_of("\\/");
		std::string parentPath05 = curPath05.substr(0, found05);
		ofstream outtriangles(parentPath05 + "//" + filename05 + "_squared05.obj");
		int idx_p = 0;
		int idx_f = 0;
		CMeshTool meshTool;
		for (int i = 0; i < squaredPlanes.size(); i++) {

			if (squaredPlanes.size() == 0)
				continue;
			std::vector<cv::Point3f> obj_points;
			std::vector<cv::Vec3i> face;
			std::string obj_path = path + "_squared05_" + to_string(i) + ".obj";
			bool isGet = meshTool.GetObjData(obj_path, obj_points, face);
			if (!isGet) {
				std::cout << "meshTool.GetObjData fail! file:" << obj_path.c_str() << std::endl;
			}
			else {
				for (int j = 0; j < obj_points.size(); j++) {
					g_obj_points.push_back(obj_points[j]);
				}
				for (int j = 0; j < face.size(); j++) {
					cv::Vec3i tmp(idx_f, idx_f, idx_f);
					g_face.push_back(face[j] + tmp);
				}
				idx_f += obj_points.size();
			}
		}
		for (int i = 0; i < g_obj_points.size(); i++)
			outtriangles << "v " << g_obj_points[i].x << " " << g_obj_points[i].y << " " << g_obj_points[i].z << endl;
		for (int i = 0; i < g_face.size(); i++)
			outtriangles << "f " << g_face[i][0] << " " << g_face[i][1] << " " << g_face[i][2] << endl;
		outtriangles.close();

		ofstream out(parentPath05 + "//" + "WallGroundAngle05.txt");
		for (int i = 0; i < walls_index_normal.size(); i++)
		{
			cv::Point3f wall_normal = walls_index_normal[i].second;
			cv::Point3f ground_normal{ 0, 0, 1 };
			float WG_Angle = 0;
			WG_Angle = getTdAngle(wall_normal, ground_normal);

			out << "wall_id:" << walls_index_normal[i].first << "  WallandGround_Angle:" << WG_Angle << endl;
		}
		out.close();
	};
};

void SaveDebugImages(
	const vector<cv::Point3f>& orig3d,
	const vector<cv::Point3f>& squared3d,
    const string& dir);
