// polygonal_surface_reconstruction_test.cpp
//
// Test the Polygonal Surface Reconstruction method with different
//    - kernels(Simple_cartesian, EPICK)
//    - solvers(GLPK, SCIP)
//    - use/ignore provided planar segmentation
//    - input file formats (pwn, ply). For ply format, a property "segment_index"
//      must be present storing the plane index for each point(-1 if the point is
//      not assigned to a plane).

#include "mesh_tool.h"
#include "../common/PlaneStructType.h"
#include "../RoomMeasurement.h"

#ifdef CGAL_USE_GLPK
#include <CGAL/GLPK_mixed_integer_program_traits.h> 
typedef CGAL::GLPK_mixed_integer_program_traits<double>                                GLPK_Solver;
#endif

 
#ifdef CGAL_USE_SCIP
#include <CGAL/SCIP_mixed_integer_program_traits.h> 
//#include <../../external/CGAL/cgal-5.3/Solver_interface/include/CGAL/SCIP_mixed_integer_program_traits.h>
typedef CGAL::SCIP_mixed_integer_program_traits<double>                                SCIP_Solver;
#endif 

#include <CGAL/Simple_cartesian.h> 
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include "..\PlaneSegmentationEx.h"
#include "util_plane_fit.hpp"
#include "util_time.hpp"
#include "MathOperation.hpp"
#include "DownSampleFilter.hpp"
#include "in_out_data.hpp"
#include "util_log.hpp"
#include "util_UNRE.hpp"
#include "util_normalEst.hpp"
#include "util_plane_fit.hpp"
#include "InOutData.h"
#include "..\PlaneCuttingEx.h"
#include "../Common/buildRefectImgFuc.h"

#include "Reconstructer.h"
#include "..\concreteMesher.h"
//#define TEST_PROCESSING_MODEL
//#define SAVE_SEG_POINTS 1
#ifdef _WIN32
#include<windows.h>
#include<cmath>
#else
#include <chrono>
#endif
// kernels:
typedef CGAL::Simple_cartesian<double>                                           Cartesian;
typedef CGAL::Exact_predicates_inexact_constructions_kernel                        Epick;

using namespace std;
using namespace Eigen;
#include <CGAL/point_generators_3.h>


#include <CGAL/Nef_polyhedron_2.h>
#include <CGAL/Bounded_kernel.h>
#include <CGAL/Exact_rational.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Boolean_set_operations_2.h>

#include "..\Mesh/mesher/detc_edge_api.h"
//typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel_;
typedef Cartesian::Point_2                                   Point_2;
typedef CGAL::Polygon_2<Cartesian>                           Polygon_2;
typedef CGAL::Polygon_with_holes_2<Cartesian>                Polygon_with_holes_2;
typedef std::list<Polygon_with_holes_2>                   Pwh_list_2;
#include <CGAL/Eigen_matrix.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/Convex_hull_traits_adapter_2.h>
#include <CGAL/Random.h>

#include "..\MultiStage/StageMatrix.h"
#include <CGAL/grid_simplify_point_set.h>

#include <CGAL/Classification.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Shape_detection/Region_growing.h>
#include <CGAL/Classification/Point_set_neighborhood.h>

#include <CGAL/Octree.h>
 
typedef Cartesian::Point_3                Point;
typedef Cartesian::Iso_cuboid_3           Iso_cuboid_3;
typedef CGAL::Point_set_3<Point> Point_set;
typedef Point_set::Point_map                   Pmap;
typedef Point_set::Vector_map                  Vmap;
typedef Point_set::Property_map<int>           Imap;
typedef Point_set::Property_map<unsigned char> UCmap;
typedef CGAL::Shape_detection::Point_set::Sphere_neighbor_query<Cartesian, Point_set, Pmap>                Neighbor_query;
typedef CGAL::Shape_detection::Point_set::Least_squares_plane_fit_region<Cartesian, Point_set, Pmap, Vmap> Region_type;
typedef CGAL::Shape_detection::Region_growing<Point_set, Neighbor_query, Region_type>                   Region_growing;

namespace Classification = CGAL::Classification;
namespace Feature = CGAL::Classification::Feature;
typedef Classification::Label_handle   Label_handle;
typedef Classification::Feature_handle Feature_handle;
typedef Classification::Label_set      Label_set;
typedef Classification::Feature_set    Feature_set;
typedef Classification::Local_eigen_analysis                                 Local_eigen_analysis;
typedef Classification::Point_set_feature_generator<Cartesian, Point_set, Pmap> Feature_generator;
typedef Classification::Point_set_neighborhood<Cartesian, Point_set, Pmap>       Neighborhood;
typedef Classification::Cluster<Point_set, Pmap>                             Cluster;

typedef CGAL::Octree<Cartesian, Point_set, Pmap> Octree;

typedef CGAL::Convex_hull_traits_adapter_2<Cartesian,
	CGAL::Pointer_property_map<Point_2>::type > Convex_hull_traits_2;

typedef Classification::Planimetric_grid<Cartesian, Point_set, Pmap>             Planimetric_grid;

//using Polygon = CGAL::Polygon_2<Cartesian>;
using Line_3 = CGAL::Simple_cartesian<double>::Line_3;
using Ray_3 = CGAL::Simple_cartesian<double>::Ray_3;
using Plane = CGAL::Simple_cartesian<double>::Plane_3;
typedef typename Cartesian::Iso_cuboid_3 BBox;
typedef typename Cartesian::FT                                FT;
typedef CGAL::internal::Planar_segment<Cartesian>                        Planar_segment;
typedef CGAL::internal::Point_set_with_planes<Cartesian>                Point_set_with_planes;

bool beP2P(Point& p_a, Point& p_b,
	vector<map<Point, vector<uint32_t>>*>& gridList,
	std::vector<cv::Point3f>& centreList,
	std::vector<cv::Point3f>& normalList)
{
	using Vector = Cartesian::Vector_3;
	Line_3 line(p_a, p_b);
	//Ray_3 ray_a2b(p_a, p_b);
	double len = CGAL_IA_MAX_DOUBLE;
	for (int i = 0; i < centreList.size(); i++)
	{
		Point centre(centreList[i].x, centreList[i].y, centreList[i].z);
		Vector normal(normalList[i].x, normalList[i].y, normalList[i].z);
		CGAL::Simple_cartesian<double>::Plane_3 midPlan(centre, normal);
		//if (midPlan.oriented_side(p_a) * midPlan.oriented_side(p_b) < 0)
		//Line_3 line(p_a, p_b);
		auto res = CGAL::intersection(midPlan, line);
		if (res)
		{
			auto p = *boost::get<Point>(&*res);
			Point ref(std::floor(p.x() / 400.),
				std::floor(p.y() / 400.),
				std::floor(p.z() / 400.));
			//this is wrong
			if ((*gridList[i]).find(ref) != (*gridList[i]).end())
				return false;

		}
	}
	return true;
}

bool beP2P(Point& p_a, Point& p_b,
	vector<map<Point, vector<uint32_t>>>& gridList,
	std::vector<cv::Point3f>& centreList,
	std::vector<cv::Point3f>& normalList)
{
	using Vector = Cartesian::Vector_3;
	Line_3 line(p_a, p_b);
	//Ray_3 ray_a2b(p_a, p_b);
	double len = CGAL_IA_MAX_DOUBLE;
	for (int i = 0; i < centreList.size(); i++)
	{
		Point centre(centreList[i].x, centreList[i].y, centreList[i].z);
		Vector normal(normalList[i].x, normalList[i].y, normalList[i].z);
		CGAL::Simple_cartesian<double>::Plane_3 midPlan(centre, normal);
		if (midPlan.oriented_side(p_a) * midPlan.oriented_side(p_b) < 0)
		{
			auto res = CGAL::intersection(midPlan, line);
			if (res)
			{
				auto p_3 = *boost::get<Point>(&*res);
				auto p = midPlan.to_2d(p_3);
				Point ref(std::floor(p.x() / 400.),
					std::floor(p.y() / 400.),
					std::floor(0 / 1.));
				if (gridList[i].find(ref) != (gridList[i]).end())
					return false;

			}
		}
		//Line_3 line(p_a, p_b);

	}
	return true;
}
void voxelize_point_set(std::size_t nb_pts, std::vector<std::uint32_t>& indices, Pmap point_map,
	float voxel_size)
{
	std::map<Point, std::vector<std::uint32_t> > grid;

	for (std::uint32_t i = 0; i < nb_pts; ++i)
	{
		const Point& p = get(point_map, i);
		Point ref(std::floor(p.x() / voxel_size),
			std::floor(p.y() / voxel_size),
			std::floor(p.z() / voxel_size));
		typename std::map<Point, std::vector<std::uint32_t> >::iterator it;
		boost::tie(it, boost::tuples::ignore)
			= grid.insert(std::make_pair(ref, std::vector<std::uint32_t>()));
		it->second.push_back(i);
	}

	for (typename std::map<Point, std::vector<std::uint32_t> >::iterator
		it = grid.begin(); it != grid.end(); ++it)
	{
		const std::vector<std::uint32_t>& pts = it->second;
		Point centroid = CGAL::centroid(CGAL::make_transform_iterator_from_property_map
		(pts.begin(), point_map),
			CGAL::make_transform_iterator_from_property_map
			(pts.end(), point_map));
		std::uint32_t chosen = 0;
		float min_dist = (std::numeric_limits<float>::max)();
		for (std::size_t i = 0; i < pts.size(); ++i)
		{
			float dist = float(CGAL::squared_distance(get(point_map, pts[i]), centroid));
			if (dist < min_dist)
			{
				min_dist = dist;
				chosen = pts[i];
			}
		}
		indices.push_back(chosen);
	}
}

void voxelize_point_set(std::size_t nb_pts, vector<std::vector<std::uint32_t>>& indices, Pmap point_map,
	float voxel_size)
{
	std::map<Point, std::vector<std::uint32_t> > grid;

	for (std::uint32_t i = 0; i < nb_pts; ++i)
	{
		const Point& p = get(point_map, i);
		Point ref(std::floor(p.x() / voxel_size),
			std::floor(p.y() / voxel_size),
			std::floor(p.z() / voxel_size));
		typename std::map<Point, std::vector<std::uint32_t> >::iterator it;
		boost::tie(it, boost::tuples::ignore)
			= grid.insert(std::make_pair(ref, std::vector<std::uint32_t>()));
		it->second.push_back(i);
	}

	for (typename std::map<Point, std::vector<std::uint32_t> >::iterator
		it = grid.begin(); it != grid.end(); ++it)
	{
		const std::vector<std::uint32_t>& pts = it->second;
		//Point centroid = CGAL::centroid(CGAL::make_transform_iterator_from_property_map
		//(pts.begin(), point_map),
		//	CGAL::make_transform_iterator_from_property_map
		//	(pts.end(), point_map));
		vector<std::uint32_t> chosen;
		//float min_dist = (std::numeric_limits<float>::max)();
		for (std::size_t i = 0; i < pts.size(); ++i)
		{
			//float dist = float(CGAL::squared_distance(get(point_map, pts[i]), centroid));
			//if (dist < min_dist)
			//{
			//	min_dist = dist;
			//	chosen = pts[i];
			//}
			chosen.push_back(pts[i]);
		}
		indices.push_back(chosen);
	}
}
std::map<Point, std::vector<std::uint32_t>>* voxelize_point_set(std::size_t nb_pts, Pmap point_map,
	float voxel_size)
{
	std::map<Point, std::vector<std::uint32_t>> grid;

	for (std::uint32_t i = 0; i < nb_pts; ++i)
	{
		const Point& p = get(point_map, i);
		Point ref(std::floor(p.x() / voxel_size),
			std::floor(p.y() / voxel_size),
			std::floor(p.z() / voxel_size));
		typename std::map<Point, std::vector<std::uint32_t> >::iterator it;
		boost::tie(it, boost::tuples::ignore)
			= grid.insert(std::make_pair(ref, std::vector<std::uint32_t>()));
		it->second.push_back(i);
	}
	return &grid;
}
bool voxelize_point_set(std::size_t nb_pts, Pmap point_map,
	float voxel_size,
	map<Point, vector<uint32_t>>& grid)
{
	//std::map<Point, std::vector<std::uint32_t>> grid;

	for (std::uint32_t i = 0; i < nb_pts; ++i)
	{
		const Point& p = get(point_map, i);
		Point ref(std::floor(p.x() / voxel_size),
			std::floor(p.y() / voxel_size),
			std::floor(p.z() / voxel_size));
		typename std::map<Point, std::vector<std::uint32_t> >::iterator it;
		boost::tie(it, boost::tuples::ignore)
			= grid.insert(std::make_pair(ref, std::vector<std::uint32_t>()));
		it->second.push_back(i);
	}
	return true;
}

Eigen::Matrix4d Reconstructer::projectVP(std::vector<cv::Point3f>& inPointCloud, cv::Point3f centre, cv::Point3f normal)
{
	MatrixXd planePoint(20, 3);
	for (size_t i = 0; i < 20; i++)
	{
		auto idx = rand() % inPointCloud.size();
		planePoint.row(i) << inPointCloud[idx].x, inPointCloud[idx].y, inPointCloud[idx].z;
	}
	//planePoint.re
	auto plane = [&](Vector3d& vPlaneVec, Vector3d& vPlaneCenter) {
		Eigen::RowVector3d meanVec = planePoint.colwise().mean();
		vPlaneCenter = meanVec.transpose();
		Eigen::MatrixXd zeroMeanMat = planePoint;
		zeroMeanMat.rowwise() -= meanVec;
		//Eigen::MatrixXd covMat = zeroMeanMat.transpose()*zeroMeanMat;//是否采用协方差矩阵，效果一样，但需要思考
		Eigen::MatrixXd covMat = zeroMeanMat;
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(covMat, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::Matrix3d V = svd.matrixV();
		Eigen::MatrixXd U = svd.matrixU();
		//Eigen::Matrix3d S = U.inverse() * covMat * V.transpose().inverse();       //why?
		//plane_output.n << V(6), V(7), V(8);
		vPlaneVec << V(6), V(7), V(8);
		//Eigen::MatrixXd D_Mat = plane_output.n * meanVec.transpose();
		//plane_output.D = D_Mat(0);
	};

	Eigen::Vector3d vPlanevec;
	Eigen::Vector3d vPlaneCenter;
	plane(vPlanevec, vPlaneCenter);
	Eigen::Vector3d vectorAfter(0., 0., 1.);

	auto rotMatrix = (Eigen::Quaterniond::FromTwoVectors(vPlanevec, vectorAfter)).toRotationMatrix();
	auto rotMatrix_ = Eigen::Quaterniond::FromTwoVectors(vPlanevec, vectorAfter).matrix();

	Matrix3d reversed_rotMatrix;// = rotMatrix.inverse();
	Matrix3d reversed_rotMatrix_ = Eigen::Quaterniond::FromTwoVectors(vectorAfter, vPlanevec).toRotationMatrix();

	//Matrix3d rotMatrix_3 = rotMatrix.inverse();
	Matrix3d rotMatrix_3 = rotMatrix;
	//for (auto x : inPointCloud)
	//{
	//	//move
	//	Eigen::Vector3d axi1Before(x.x - vPlaneCenter[0], x.y - vPlaneCenter[1], x.z - vPlaneCenter[2]);
	//	Eigen::Vector3d axi1After = rotMatrix * axi1Before;
	//	outPointCloud.push_back({ (float)axi1After.x(),(float)axi1After.y(),(float)axi1After.z() });
	//	//cout << axi1After.x() << "," << axi1After.y() << "," << axi1After.z() << "," << endl;
	//}
	//ofstream outPoints("D:/data/outmesh_30_test/outPoint.pts");
	//for (auto x : inPointCloud)
	//    outPoints << x[0] << " " << x[1] << " " << x[2] << endl;
	//outPoints.close();
	Matrix4d rotMatrix_4_test;
	rotMatrix_4_test << 1, 0, 0, -vPlaneCenter[0],
		0, 1, 0, -vPlaneCenter[1],
		0, 0, 1, -vPlaneCenter[2],
		0, 0, 0, 1;
	Matrix4d rotMatrix_4;
	rotMatrix_4 << rotMatrix_3(0, 0), rotMatrix_3(0, 1), rotMatrix_3(0, 2), 0,
		rotMatrix_3(1, 0), rotMatrix_3(1, 1), rotMatrix_3(1, 2), 0,
		rotMatrix_3(2, 0), rotMatrix_3(2, 1), rotMatrix_3(2, 2), 0,
		0, 0, 0, 1;
	Matrix4d rotMatrix_4_test_result = (rotMatrix_4*rotMatrix_4_test);

	return rotMatrix_4_test_result;
}

bool Reconstructer::transformatVecCvP(std::vector<cv::Point3f>& inPointCloud, Eigen::Matrix4d matrix, std::vector<cv::Point3f>& outPointCloud)
{
	for (auto x : inPointCloud)
	{
		//move
		Eigen::Vector4d axi1Before(x.x, x.y, x.z, 1.);
		Eigen::Vector4d axi1After = matrix * axi1Before;
		outPointCloud.push_back({ (float)axi1After.x(),(float)axi1After.y(),(float)axi1After.z() });
		//cout << axi1After.x() << "," << axi1After.y() << "," << axi1After.z() << "," << endl;
	}
	return true;
}

Reconstructer::Reconstructer(std::vector<std::vector<cv::Point3f>>& inPoints, vector<cv::Point3f>& normal, string type)
{
	using Point = Cartesian::Point_3;
	using Vector = Cartesian::Vector_3;
	m_name = type;
	for (int i = 0; i < inPoints.size(); i++)
	{
		std::vector<cv::Point3f> tempPoints;
		vector<unsigned int> point_to_centriod_arrays;
		DownSampleFilter::FilterGridApply<Point3f, float, unsigned int>(inPoints[i], 50., tempPoints, point_to_centriod_arrays);

		for (auto x : tempPoints)
			points.push_back(boost::tuple<Point, Vector, int>({ x.x,x.y,x.z }, { normal[i].x,normal[i].y,normal[i].z }, i));
	}
	vecPoints.resize(inPoints.size());
	reconstructPrompt();
}

Reconstructer::Reconstructer(PlaneCutResultInterface & cutResult, string type)
{
	inputData=&(cutResult);
	int planeId = 0;
	auto reconstructPointCloud = [&]() {
		Cartesian::Point_3 plane_centre(0, 0, 3);
		Cartesian::Vector_3 plane_normal(0, 0, 1);
		Plane plane_infinite(plane_centre, plane_normal);
		Point_set removePoints;
		for (int i = 0; i < cutResult.plane_xyz.size(); i++, planeId++)
		{
			std::vector<cv::Point3f> tempPoints;
			vector<unsigned int> point_to_centriod_arrays;
			DownSampleFilter::FilterGridApply<Point3f, float, unsigned int>(cutResult.plane_xyz[i], 50., tempPoints, point_to_centriod_arrays);

			for (auto x : tempPoints)
			{
				if (plane_infinite.has_on_positive_side({ x.x,x.y,x.z }))
					points.push_back(boost::tuple<Point, Vector, int>({ x.x,x.y,x.z }, { 0,0,0 }, planeId));
				else
					removePoints.insert({ x.x,x.y,x.z });
			}
		}
		const BBox& box = CGAL::bounding_box(removePoints.point_map().begin(), removePoints.point_map().end());

	};
	auto reconstructPointCloud_simple = [&]() {
		Cartesian::Point_3 plane_centre(0, 0, 3);
		Cartesian::Vector_3 plane_normal(0, 0, 1);
		Plane plane_infinite(plane_centre, plane_normal);
		Point_set removePoints;
		for (int i = 0; i < cutResult.plane_xyz.size(); i++, planeId++)
		{
			std::vector<cv::Point3f> tempPoints;
			vector<unsigned int> point_to_centriod_arrays;
			DownSampleFilter::FilterGridApply<Point3f, float, unsigned int>(cutResult.plane_xyz[i], 50., tempPoints, point_to_centriod_arrays);

			for (auto x : tempPoints)
			{
				points.push_back(boost::tuple<Point, Vector, int>({ x.x,x.y,x.z }, { 0,0,0 }, planeId));
			}
		}
	};
	reconstructPointCloud_simple();
	planeId = cutResult.plane_xyz.size()+1;
	auto getVirtualFace = [&]() {
		vector<Point_set> vecPS;
		for (auto x : (*inputData).door_window_info)
		{
			Point_set hole;
			for (auto y : x)
			{
				if (y.type == 1)//door
					continue;
				for (auto z : y.corners)
				{
					hole.insert(Point{ z.x,z.y,z.z });
				}
			}
			vecPS.push_back(hole);
		} 

		for (int i = 0; i < vecPS.size(); ++i)
		{
			if(vecPS[i].is_empty())
				continue;
			CGAL::Surface_mesh<Cartesian::Point_3> mesh;
			Plane hole(vecPS[i].point(0), vecPS[i].point(1), vecPS[i].point(2));
			auto vecHole = hole.orthogonal_vector();
			vecHole /= sqrt(vecHole.squared_length());
			//cout << vecHole;
			if (!hole.has_on_positive_side({ 0,0,0 }))
			{
				vecPS[i].point(0) +=vecHole*200;
				vecPS[i].point(1) +=vecHole*200;
				vecPS[i].point(2) +=vecHole*200;
				vecPS[i].point(3) +=vecHole*200;
			}
			else
			{
				vecPS[i].point(0) -= vecHole * 200;
				vecPS[i].point(1) -= vecHole * 200;
				vecPS[i].point(2) -= vecHole * 200;
				vecPS[i].point(3) -= vecHole * 200;
			}
			{
				auto u = mesh.add_vertex(vecPS[i].point(0));
				auto v = mesh.add_vertex(vecPS[i].point(1));
				auto w = mesh.add_vertex(vecPS[i].point(2));
				auto x = mesh.add_vertex(vecPS[i].point(3));
				mesh.add_face(u, v, w);
				mesh.add_face(x, u, w);
			}
			
			Point_set point_set;
			auto HolePoints = [&]() {
				CGAL::Random_points_in_triangle_mesh_3<CGAL::Surface_mesh<Cartesian::Point_3>> generator(mesh);
				std::size_t nb_pts = 100;
				point_set.reserve(nb_pts);
				for (std::size_t i = 0; i < nb_pts; ++i)
					point_set.insert(*(generator++));
			};
			HolePoints();

			//CGAL::IO::write_XYZ("E://test//hole"+to_string(i)+".xyz", point_set);
			for (auto x : point_set.points())
				points.push_back(boost::tuple<Point, Vector, int>(x, { 0,0,0 }, planeId));
			++planeId;
		}
	};

	// getVirtualFace();
	contourTreeList.resize(planeId);
	vecPoints.resize(planeId);
	reconstructPrompt();
}
Reconstructer::Reconstructer(PlaneCutResultInterface & cutResult, float ceil,string type)
{
	inputData=&(cutResult);
	int planeId = 0;
	auto reconstructPointCloud_simple = [&]() {
		Cartesian::Point_3 plane_centre(0, 0, ceil);
		Cartesian::Vector_3 plane_normal(0, 0, 1);
		Plane plane_infinite(plane_centre, plane_normal);
		Point_set removePoints;
		for (int i = 0; i < cutResult.plane_xyz.size(); i++, planeId++)
		{
			std::vector<cv::Point3f> tempPoints;
			vector<unsigned int> point_to_centriod_arrays;
			DownSampleFilter::FilterGridApply<Point3f, float, unsigned int>(cutResult.plane_xyz[i], 50., tempPoints, point_to_centriod_arrays);
			//BBox& box = CGAL::bounding_box(tempPoints.begin(), tempPoints.end());
			for (auto x : tempPoints)
			{
				if (plane_infinite.has_on_positive_side({ x.x,x.y,x.z })||
					(find(cutResult.plane_ceiling_idx.begin(), cutResult.plane_ceiling_idx.end(),i)!=cutResult.plane_ceiling_idx.end())
					)
					points.push_back(boost::tuple<Point, Vector, int>({ x.x,x.y,plane_centre.z() }, { 0,0,0 }, cutResult.plane_xyz.size()));
				else
					points.push_back(boost::tuple<Point, Vector, int>({ x.x,x.y,x.z }, { 0,0,0 }, planeId));
					
			}

		} 
	};
	reconstructPointCloud_simple();
	planeId = cutResult.plane_xyz.size() + 1;
	contourTreeList.resize(planeId);
	vecPoints.resize(planeId);
	ceilIds.insert(cutResult.plane_xyz.size());
	reconstructPrompt();
}

bool Reconstructer::FindNearestPointHelper(const ModuleStruct::Point3f& input_pt, const ModuleStruct::Point3fArray& needChangeHole,int& find_index)
{
	if (needChangeHole.size()==0)
	{
		return false;
	}
	int index = -1;
	float min_dist = std::numeric_limits<float>::infinity();
	for (int i = 0; i < needChangeHole.size(); i++)
	{
		ModuleStruct::Point3f pt = needChangeHole[i];
		float dist = std::sqrt((input_pt.x - pt.x)*(input_pt.x - pt.x) + (input_pt.y - pt.y)*(input_pt.y - pt.y) + (input_pt.z - pt.z)*(input_pt.z - pt.z));
		if (dist< min_dist)
		{
			min_dist = dist;
			index = i;
		}
	}
	if (index != -1)
	{
		find_index = index;
		return true;
	}
	return false;
}

void Reconstructer::ChangeHolePointHelper(const ModuleStruct::Point3fArray& inputHole, ModuleStruct::Point3fArray& needChangeHole)
{

	//if (inputHole.size()==4 && needChangeHole.size()>4)
	//{
		ModuleStruct::Point3fArray tmpHole = inputHole;
		for (int i=0;i< inputHole.size();i++)
		{
			ModuleStruct::Point3f pt = inputHole[i];
			int find_index = -1;
			bool isFind = FindNearestPointHelper(pt, needChangeHole, find_index);
			if (isFind)
			{
				tmpHole[i] = needChangeHole[find_index];
			}

		}
		needChangeHole = tmpHole;
	//}
}
std::vector<std::vector<Tree<Point3fArray>>> Reconstructer::getSaveAsObjContourTreeList()
{
	std::vector<Point3fArray> test_points_2Dcontours(contour.size());
	std::unordered_map<int, vector<int>> mapEdgeHoleList;
	std::unordered_map<int, int> mapEdgeHoleListTemp;
	//hierarchy is adjacency table,use level traversal
	//two level struct,and screen
	unordered_map<int, vector<int>> sameFace;
	for (size_t i = 0; i < contour.size(); i++)
	{
		mapEdgeHoleListTemp[i] = get<2>(contour[i]);
		sameFace[get<1>(contour[i])].push_back(i);
		for (auto x : get<0>(contour[i]))
		{
			auto p = modelProcessed.point(CGAL::SM_Vertex_index(x));
			test_points_2Dcontours[i].push_back({ (float)p.x(),(float)p.y(),(float)p.z() });
		}
	}
	for (auto it = mapEdgeHoleListTemp.begin(); it != mapEdgeHoleListTemp.end(); it++)
		if (it->second > -1 &&
			mapEdgeHoleListTemp[it->second] == -1)
			mapEdgeHoleList[it->second].push_back(it->first);
		else if (it->second == -1 &&
			mapEdgeHoleList.count(it->first) == 0)
			mapEdgeHoleList[it->first] = {};

	for (auto it = sameFace.begin(); it != sameFace.end(); it++)
	{
		if (it->first < 0 || it->first >= vecPoints.size()) {
			std::cerr << "Error: Invalid plane index " << it->first
				<< " (vecPoints.size()=" << vecPoints.size() << ")\n";
			continue;  // 跳过无效索引
		}

		Point_vector& planePoints = vecPoints[it->first];
		if (planePoints.empty()) {
			std::cerr << "Warning: Plane " << it->first << " has no points\n";
			continue;  // 跳过空平面
		}

		//maybe wrong
		auto faceId = get<2>(vecPoints[it->first][0]);
		for (auto x : it->second)
		{
			if (mapEdgeHoleList.count(x) == 0)
				continue;
			auto contour = make_shared<Tree<Point3fArray>>(test_points_2Dcontours[x]);
			for (auto y : mapEdgeHoleList[x])
			{
				contour->Add_Node(test_points_2Dcontours[x], test_points_2Dcontours[y]);
			}
			contourTreeList[faceId].push_back(*contour);
		}
	}


	auto hierarchyMeshTest = [&]() {
		auto hierarchy = contourTreeList;
		for (int i = 0; i < inputData->plane_xyz.size(); i++)
		{
			Eigen::Matrix4d matrix;
			if (hierarchy[i].size() > 0)
				matrix = projectVP(hierarchy[i][0].get()->_val, {}, {});
			else
				continue;
			Eigen::Matrix4d matrix_inverse = matrix.inverse();

			std::vector<ModuleStruct::Point3fArray> edgePointsList;
			std::vector<std::vector<ModuleStruct::Point3fArray>> holePointsList;
			for (int j = 0; j < hierarchy[i].size(); j++)
			{
				ModuleStruct::Point3fArray tempEdge;
				transformatVecCvP(hierarchy[i][j].get()->_val, matrix, tempEdge);
				edgePointsList.push_back(tempEdge);
				Polygon_2 P;
				for (auto x : tempEdge)
					P.push_back(Point_2(x.x, x.y));
				if (P.orientation() == -1)
					P.reverse_orientation();
				std::vector<Point3fArray> holelist;
				for (auto y : hierarchy[i][j].get()->_children)
				{
					ModuleStruct::Point3fArray tempHole;
					transformatVecCvP(y->_val, matrix, tempHole);
					holelist.push_back(tempHole);
				}
				for (auto x : inputData->door_window_info[i])
				{
					if (x.type == 0 && (!hierarchy[m_holes[&x]].size() > 0))	//Window
						continue;
					//ofstream hole(m_name+"_hole.pts",ios::app);
					//hole << x.corners << endl;
					//hole.close();
					ModuleStruct::Point3fArray tempHole;
					transformatVecCvP(x.corners, matrix, tempHole);
					//cout << tempHole << endl;
					//cout << tempEdge << endl;
					Polygon_2 Q;
					for (auto x : tempHole)
						Q.push_back(Point_2(x.x, x.y));
					if (Q.orientation() == -1)
						Q.reverse_orientation();
					if (CGAL::do_intersect(P, Q))
					{
						ModuleStruct::Point3fArray tempHoleRes;
						Pwh_list_2 R;
						CGAL::intersection(P, Q, std::back_inserter(R));
						for (auto x : R)
							for (auto y : x.outer_boundary())
							{
								tempHoleRes.push_back({ (float)y.x() + 0.01f/*+5.f*/,(float)y.y() + 0.01f/*+5.f*/,0. });
								//cout << "tempHoleRes:"<< x.outer_boundary().size() << endl;
							}
						//#define LOG
#ifdef LOG
						for (auto x : R)
						{
							cout << sqrt(x.outer_boundary().edge(0).squared_length()) << endl;
							cout << sqrt(x.outer_boundary().edge(1).squared_length()) << endl;
							cout << sqrt(x.outer_boundary().edge(2).squared_length()) << endl;
							cout << sqrt(x.outer_boundary().edge(3).squared_length()) << endl;
							cout << endl;
						}
#endif                 
						if(tempHole.size()==4 && tempHoleRes.size()>4)//added by hgj 20230904
						{
							ChangeHolePointHelper(tempHole, tempHoleRes);
							//cout << "tempHoleRes.size():" << tempHoleRes.size() << endl;
						}
						holelist.push_back(tempHoleRes);
						//cout << tempHole.size() << endl;
						//cout << tempHole << endl;
						//220112
						ModuleStruct::Point3fArray tempHole3D;
						transformatVecCvP(tempHoleRes, matrix_inverse,tempHole3D);
						hierarchy[i][j].Add_Node(hierarchy[i][j].get()->_val, tempHole3D);
					}

				}
				holePointsList.push_back(holelist);
			}
		}
	}; 
	hierarchyMeshTest();

	/*
	// 最终打印contourTreeList的整体情况
	std::cout << "[最终排查] contourTreeList总大小=" << contourTreeList.size() << std::endl;
	for (int i = 0; i < contourTreeList.size(); i++) {
		std::cout << "[最终排查] contourTreeList[" << i << "]元素数=" << contourTreeList[i].size() << std::endl;
		std::string file_name = "contourTreeList_" + std::to_string(i) + ".txt";
		std::ofstream single_file(file_name);
		if (contourTreeList[i].size() > 0)
		{
			for (auto& pt : contourTreeList[i][0].get()->_val) {
				std::cout << "点坐标：x=" << pt.x << ", y=" << pt.y << ", z=" << pt.z << std::endl;
				single_file << pt.x << " " << pt.y << " " << pt.z << std::endl;
			}
		}
		single_file.close();
	}*/


	return contourTreeList;
};

void Reconstructer::GetnormalBy3Pts(cv::Point3f &p1, cv::Point3f &p2, cv::Point3f &p3, cv::Point3f &normalByPlaneCorner)
{
	float x1 = p2.x - p1.x; float y1 = p2.y - p1.y; float z1 = p2.z - p1.z;
	float x2 = p3.x - p1.x; float y2 = p3.y - p1.y; float z2 = p3.z - p1.z;
	float a = y1 * z2 - y2 * z1;
	float b = z1 * x2 - z2 * x1;
	float c = x1 * y2 - x2 * y1;
	//????????
	if (c < 0)
	{
		a = -a;
		b = -b;
		c = -c;
	}
	//???
	float length = sqrt(a*a + b * b + c * c);
	a = a / length;
	b = b / length;
	c = c / length;
	normalByPlaneCorner.x = a;
	normalByPlaneCorner.y = b;
	normalByPlaneCorner.z = c;
}



void Reconstructer::ProjectToPlane(std::vector<cv::Point3f> corners, std::vector<cv::Point3f> wall_plane, std::vector<cv::Point3f> &cornersToPlane)
{
	cv::Point3f normalByPlaneCorners;
	GetnormalBy3Pts(wall_plane[0], wall_plane[1], wall_plane[2], normalByPlaneCorners);

	Point plane_point0(wall_plane[0].x, wall_plane[0].y, wall_plane[0].z);
	Vector plane_normal(normalByPlaneCorners.x, normalByPlaneCorners.y, normalByPlaneCorners.z);
	CGAL::Simple_cartesian<double>::Plane_3 plane_func(plane_point0, plane_normal);

	for (int i = 0; i < corners.size(); i++)
	{
		Point cornerI(corners[i].x, corners[i].y, corners[i].z);
		Line_3 line(cornerI, plane_normal);
		
		CGAL::Object res = CGAL::intersection(plane_func, line);
		if (const Point* pt = CGAL::object_cast<Point>(&res))
		{
			Point p_inter = *pt;
			cornersToPlane.emplace_back(cv::Point3f(p_inter.x(), p_inter.y(), p_inter.z()));
		}
	}
}

void Reconstructer::ProjectToPlane(cv::Point3f &point, std::vector<cv::Point3f> &wall_plane, cv::Point3f &pointToPlane)
{
	cv::Point3f normalByPlaneCorners;
	GetnormalBy3Pts(wall_plane[0], wall_plane[1], wall_plane[2], normalByPlaneCorners);

	Point plane_point0(wall_plane[0].x, wall_plane[0].y, wall_plane[0].z);
	Vector plane_normal(normalByPlaneCorners.x, normalByPlaneCorners.y, normalByPlaneCorners.z);
	CGAL::Simple_cartesian<double>::Plane_3 plane_func(plane_point0, plane_normal);

	Point cornerI(point.x, point.y, point.z);
	Line_3 line(cornerI, plane_normal);

	CGAL::Object res = CGAL::intersection(plane_func, line);
	if (const Point* pt = CGAL::object_cast<Point>(&res))
	{
		Point p_inter = *pt;
		pointToPlane = cv::Point3f(p_inter.x(), p_inter.y(), p_inter.z());
	}
}

void Reconstructer::SaveAsObj(std::vector<std::vector<MeasureDoorWindow::DoorWindowInfo>>& door_window_info, std::string file_dir)
{
	concreteMesherDelauney mesher;
	std::vector<std::vector<cv::Point3f>> real_plane_corners;
	real_plane_corners.clear();
	for (int i = 0; i < inputData->plane_corners.size(); i++)
	{
		if (inputData->plane_corners[i].size() > 0)
		{
			if (inputData->plane_wall_idx[i] >= 0 && inputData->plane_wall_idx[i] < 100)
			{
				real_plane_corners.emplace_back(inputData->plane_corners[i]);
			}
		}
		else
		{
			break;
		}
	}

	auto hierarchyMeshTest = [&]() {
		auto hierarchy = contourTreeList; 

		std::vector<std::vector<Tree<Point3fArray>>> real_hierarchy;
		std::vector<int> hie_index;
		hie_index.clear();
		real_hierarchy.clear();
		for (int i = 0; i < inputData->plane_xyz.size(); i++)
		{
			if (hierarchy[i].size() > 0)
			{
				real_hierarchy.emplace_back(hierarchy[i]);
				hie_index.emplace_back(i);
			}
		}

		for (int i = 0; i < real_plane_corners.size(); i++)
		{
			//TO FIX: not sure if it can solve the problem, but add a protection to avoid crash
			std::cout << "real_plane_corners size | real_hierarchy | i: " << real_plane_corners.size() << " | " << real_hierarchy.size() << " | " << i << std::endl;
			if (real_hierarchy.size() <= i)
				break;
			Eigen::Matrix4d matrix;
			matrix = projectVP(real_hierarchy[i][0].get()->_val, {}, {});
			std::vector<cv::Point3f> real_corners = real_plane_corners[i];

			Polygon_2 hierarchy_2X;
			Polygon_2 corners_2X;
			Polygon_2 hierarchy_2Y;
			Polygon_2 corners_2Y;
			std::vector<cv::Point3f> cornersToPlane;
			Pwh_list_2 intersectX;
			Pwh_list_2 intersectY;
			Pwh_list_2 intersectX_sec;
			Pwh_list_2 intersectY_sec;

			double area_P = 0;

			cv::Point3f normalByPlaneCorners;
			GetnormalBy3Pts(real_corners[0], real_corners[1], real_corners[2], normalByPlaneCorners);
			double cos_Alpha_x, cos_Beta_y, cos_Gamma_z;
			cos_Alpha_x = normalByPlaneCorners.x / sqrt(normalByPlaneCorners.x * normalByPlaneCorners.x + normalByPlaneCorners.y * normalByPlaneCorners.y/* + normalByPlaneCorners.z * normalByPlaneCorners.z*/);
			cos_Beta_y = normalByPlaneCorners.y / sqrt(normalByPlaneCorners.x * normalByPlaneCorners.x + normalByPlaneCorners.y * normalByPlaneCorners.y/* + normalByPlaneCorners.z * normalByPlaneCorners.z*/);
			double Alpha_x = acos(cos_Alpha_x); 
			if (Alpha_x > 1.57) 
				Alpha_x = 3.14 - Alpha_x;
			double Beta_y = acos(cos_Beta_y); 
			if (Beta_y > 1.57)
				Beta_y = 3.14 - Beta_y;

			for (int j = 0; j < real_hierarchy[i].size(); j++)
			{
				ProjectToPlane(real_corners, real_hierarchy[i][j].get()->_val, cornersToPlane);

				// Limit the number of plane vertices to 4
				float zMax = 0, zMin = 0;
				zMax = std::max_element(real_hierarchy[i][j].get()->_val.begin(), real_hierarchy[i][j].get()->_val.end(),
					[](cv::Point3f a, cv::Point3f b) {return a.z < b.z; })->z;
				cout << to_string(i) << endl;
				zMin = std::min_element(real_hierarchy[i][j].get()->_val.begin(), real_hierarchy[i][j].get()->_val.end(),
					[](cv::Point3f a, cv::Point3f b) {return a.z < b.z; })->z;
				
				// Remove excess points
				std::vector<cv::Point3f> temp_hie_corners;
				for (int N = 0; N < real_hierarchy[i][j].get()->_val.size(); N++)
				{
					float deta_zMax = 0, deta_zMin = 0;
					deta_zMax = abs(zMax - real_hierarchy[i][j].get()->_val[N].z);
					deta_zMin = abs(zMin - real_hierarchy[i][j].get()->_val[N].z);
					if (deta_zMax > 150 && deta_zMin > 150)
					{
						continue;
					}
					else
					{
						temp_hie_corners.emplace_back(real_hierarchy[i][j].get()->_val[N]);
					}
				}

				real_hierarchy[i][j].get()->_val.clear();
				for (int n = 0; n < temp_hie_corners.size(); n++)
				{
					real_hierarchy[i][j].get()->_val.emplace_back(temp_hie_corners[n]);
				}

				Pwh_list_2 result_intersectX;
				Pwh_list_2 result_intersectY;
				for (int k = 0; k < real_corners.size(); k++)
				{
					hierarchy_2X.push_back(Point_2(real_hierarchy[i][j].get()->_val[k].x, real_hierarchy[i][j].get()->_val[k].z));
					corners_2X.push_back(Point_2(cornersToPlane[k].x, cornersToPlane[k].z));
					hierarchy_2Y.push_back(Point_2(real_hierarchy[i][j].get()->_val[k].y, real_hierarchy[i][j].get()->_val[k].z));
					corners_2Y.push_back(Point_2(cornersToPlane[k].y, cornersToPlane[k].z));
				}

				if (CGAL::do_intersect(hierarchy_2X, corners_2X))
					CGAL::intersection(hierarchy_2X, corners_2X, std::back_inserter(result_intersectX));

				intersectX = result_intersectX;

				if (CGAL::do_intersect(hierarchy_2Y, corners_2Y))
					CGAL::intersection(hierarchy_2Y, corners_2Y, std::back_inserter(result_intersectY));

				intersectY = result_intersectY;

				if (corners_2X.area() > corners_2Y.area())
				{
					area_P = corners_2X.area() / hierarchy_2X.area();
				}
				else
				{
					area_P = corners_2Y.area() / hierarchy_2Y.area();
				}
			}

			std::vector<ModuleStruct::Point3fArray> edgePointsList;
			std::vector<std::vector<ModuleStruct::Point3fArray>> holePointsList;
			if (area_P < 0.6 && area_P > 0.1)
			{

				for (int j = 0; j < real_hierarchy[i].size(); j++)
				{
					for (auto x : intersectX)
					{
						real_hierarchy[i][j].get()->_val[0].x = x.outer_boundary()[0].x();
						real_hierarchy[i][j].get()->_val[1].x = x.outer_boundary()[1].x();
						real_hierarchy[i][j].get()->_val[2].x = x.outer_boundary()[2].x();
						real_hierarchy[i][j].get()->_val[3].x = x.outer_boundary()[3].x();
					}

					for (auto y : intersectY)
					{
						real_hierarchy[i][j].get()->_val[0].y = y.outer_boundary()[2].x();
						real_hierarchy[i][j].get()->_val[1].y = y.outer_boundary()[1].x();
						real_hierarchy[i][j].get()->_val[2].y = y.outer_boundary()[0].x();
						real_hierarchy[i][j].get()->_val[3].y = y.outer_boundary()[3].x();
					}
					ModuleStruct::Point3fArray tempEdge;

					transformatVecCvP(real_hierarchy[i][j].get()->_val, matrix, tempEdge);
					edgePointsList.push_back(tempEdge);
					Polygon_2 P;
					for (auto x : tempEdge)
					P.push_back(Point_2(x.x, x.y));
					if (P.orientation() == -1)
						P.reverse_orientation();
					std::vector<Point3fArray> holelist;
					for (auto y : real_hierarchy[i][j].get()->_children)
					{
						ModuleStruct::Point3fArray tempHole;
						transformatVecCvP(y->_val, matrix, tempHole);
						holelist.push_back(tempHole);
					}
					holePointsList.push_back(holelist);
				}
				Eigen::Matrix4d matrix_inverse = matrix.inverse();
				mesher.buildMesh(edgePointsList, holePointsList, matrix_inverse, file_dir + to_string(hie_index[i]) + ".obj");
			}
			else
			{
				std::vector<ModuleStruct::Point3fArray> edgePointsList;
				std::vector<std::vector<ModuleStruct::Point3fArray>> holePointsList;
				for (int j = 0; j < real_hierarchy[i].size(); j++)
				{
					ModuleStruct::Point3fArray tempEdge;

					transformatVecCvP(real_hierarchy[i][j].get()->_val, matrix, tempEdge);
					edgePointsList.push_back(tempEdge);
					Polygon_2 P;
					for (auto x : tempEdge)
					P.push_back(Point_2(x.x, x.y));
					if (P.orientation() == -1)
						P.reverse_orientation();
					std::vector<Point3fArray> holelist;
					for (auto y : real_hierarchy[i][j].get()->_children)
					{
						ModuleStruct::Point3fArray tempHole;
						transformatVecCvP(y->_val, matrix, tempHole);
						holelist.push_back(tempHole);
					}
					holePointsList.push_back(holelist);
				}
				Eigen::Matrix4d matrix_inverse = matrix.inverse();
				mesher.buildMesh(edgePointsList, holePointsList, matrix_inverse, file_dir + to_string(hie_index[i]) + ".obj");
			}
		}

		for (int i = real_plane_corners.size(); i < real_hierarchy.size(); i++)
		{
			Eigen::Matrix4d matrix;
			matrix = projectVP(real_hierarchy[i][0].get()->_val, {}, {});

			std::vector<ModuleStruct::Point3fArray> edgePointsList;
			std::vector<Point3fArray> holelist;
			std::vector<std::vector<ModuleStruct::Point3fArray>> holePointsList;

			for (int j = 0; j < real_hierarchy[i].size(); j++)
			{
				ModuleStruct::Point3fArray tempEdge;

				transformatVecCvP(real_hierarchy[i][j].get()->_val, matrix, tempEdge);
				edgePointsList.push_back(tempEdge);
				Polygon_2 P;
				for (auto x : tempEdge)
				P.push_back(Point_2(x.x, x.y));
				if (P.orientation() == -1)
					P.reverse_orientation();
				//std::vector<Point3fArray> holelist;
				for (auto y : real_hierarchy[i][j].get()->_children)
				{
					ModuleStruct::Point3fArray tempHole;
					transformatVecCvP(y->_val, matrix, tempHole);
					holelist.push_back(tempHole);
				}
				holePointsList.push_back(holelist);
			}

			Eigen::Matrix4d matrix_inverse = matrix.inverse();
			mesher.buildMesh(edgePointsList, holePointsList, matrix_inverse, file_dir + to_string(hie_index[i]) + ".obj");
		}
#ifdef FOR_DBG
				for (auto x : door_window_info[i])
				{
					if (x.type==0)
						continue;
					//ofstream hole(m_name+"_hole.pts",ios::app);
					//hole << x.corners << endl;
					//hole.close();
					ModuleStruct::Point3fArray tempHole;
					transformatVecCvP(x.corners, matrix, tempHole);
					//cout << tempHole << endl;
					//cout << tempEdge << endl;
					Polygon_2 Q;
					for (auto x : tempHole)
						Q.push_back(Point_2(x.x, x.y));
					if (Q.orientation() == -1)
						Q.reverse_orientation();
					if (CGAL::do_intersect(P, Q))
					{
						tempHole.clear();
						Pwh_list_2 R;
						CGAL::intersection(P, Q, std::back_inserter(R));
						for (auto x : R)
							for (auto y : x.outer_boundary())
								tempHole.push_back({ (float)y.x() + 0.01f/*+5.f*/,(float)y.y() + 0.01f/*+5.f*/,0. });
//#define LOG
#ifdef LOG
						for (auto x : R)
						{
							cout << sqrt(x.outer_boundary().edge(0).squared_length()) << endl;
							cout << sqrt(x.outer_boundary().edge(1).squared_length()) << endl;
							cout << sqrt(x.outer_boundary().edge(2).squared_length()) << endl;
							cout << sqrt(x.outer_boundary().edge(3).squared_length()) << endl;
							cout << endl;
						}
#endif
						holelist.push_back(tempHole);
						//cout << tempHole.size() << endl;
						//cout << tempHole << endl;
						//220112
						hierarchy[i][j].Add_Node(hierarchy[i][j].get()->_val,tempHole);
					}

				}
#endif 
	}; hierarchyMeshTest();
	auto MeshCeil = [&]() {
		auto hierarchy = getContourTreeList();
		for (int i = inputData->plane_xyz.size(); i <= inputData->plane_xyz.size(); i++)
		{
			Eigen::Matrix4d matrix;
			if (hierarchy[i].size() > 0)
				matrix = projectVP(hierarchy[i][0].get()->_val, {}, {});
			else
				continue;

			std::vector<ModuleStruct::Point3fArray> edgePointsList;
			std::vector<std::vector<ModuleStruct::Point3fArray>> holePointsList;
			for (int j = 0; j < hierarchy[i].size(); j++)
			{
				ModuleStruct::Point3fArray tempEdge;
				transformatVecCvP(hierarchy[i][j].get()->_val, matrix, tempEdge);
				edgePointsList.push_back(tempEdge);
				Polygon_2 P;
				for (auto x : tempEdge)
				P.push_back(Point_2(x.x, x.y));
				if (P.orientation() == -1)
					P.reverse_orientation();
				std::vector<Point3fArray> holelist;
				for (auto y : hierarchy[i][j].get()->_children)
				{
					ModuleStruct::Point3fArray tempHole;
					transformatVecCvP(y->_val, matrix, tempHole);
					holelist.push_back(tempHole);
				}
				holePointsList.push_back(holelist);

			}

			Eigen::Matrix4d matrix_inverse = matrix.inverse();
			mesher.buildMesh(edgePointsList, holePointsList, matrix_inverse, file_dir + to_string(i) + ".obj");
		}
	}; 
	if(!ceilIds.empty())
		MeshCeil();

	std::vector<cv::Point3f> g_obj_points;
	std::vector<cv::Vec3i> g_face;
	size_t found;
	found = file_dir.find_last_of("\\/");
	std::string curPath = file_dir.substr(0, found);
	std::string filename = file_dir.substr(found + 1, file_dir.length());
	found = curPath.find_last_of("\\/");
	std::string parentPath = curPath.substr(0, found);
	ofstream outtriangles(parentPath + "//" + filename + ".obj");
	int idx_p = 0;
	int idx_f = 0;
	CMeshTool meshTool;
	for (int i = 0; i < inputData->plane_xyz.size(); i++) {

		if (inputData->plane_xyz[i].size() == 0)
			continue;
		std::vector<cv::Point3f> obj_points;
		std::vector<cv::Vec3i> face;
		std::string obj_path = file_dir + to_string(i) + ".obj";
		bool isGet = meshTool.GetObjData(obj_path, obj_points, face);
		if (!isGet){
			std::cout << "meshTool.GetObjData fail! file:" << obj_path.c_str() << std::endl;
		}
		else {
			for (int j = 0; j < obj_points.size(); j++) {
				g_obj_points.push_back(obj_points[j]);
			}
			for (int j = 0; j < face.size(); j++) {
				cv::Vec3i tmp(idx_f, idx_f, idx_f);
				g_face.push_back(face[j]+tmp);
			}
			idx_f += obj_points.size();
		}
	}
	for(int i=0; i< g_obj_points.size(); i++)
		outtriangles << "v " << g_obj_points[i].x << " " << g_obj_points[i].y << " " << g_obj_points[i].z << endl;
	for (int i = 0; i < g_face.size(); i++)
		outtriangles << "f " << g_face[i][0] << " " << g_face[i][1] << " " << g_face[i][2] << endl;
	outtriangles.close();
}

bool Reconstructer::SaveModelAsObj(std::string file_dir)
{
	if (CGAL::IO::write_OBJ(file_dir + ".obj", model))
		return true;
}

bool Reconstructer::SaveModelPAsObj(std::string file_dir)
{
	if (CGAL::IO::write_OBJ(file_dir + "P.obj", modelProcessed))
		return true;

}

cv::Point3f Reconstructer::computeCenterPoint(std::vector<cv::Point3f>& centerPoints) {
	cv::Point3f centerPoint(0.0f, 0.0f, 0.0f);
	int size = centerPoints.size();
	for (int i = 0; i < size; i++) {
		centerPoint.x += centerPoints[i].x;
		centerPoint.y += centerPoints[i].y;
		centerPoint.z += centerPoints[i].z;
	}
	centerPoint = centerPoint / size;
	return centerPoint;
}

// 输入：平面中心点（x,y,z）、场景中心
// 输出：极径r（到极点的距离）、极角θ（绕Z轴的角度，0~2π）、仰角φ（与XY平面的夹角，-π/2~π/2）
std::tuple<double, double, double> calcPolarCoord(const cv::Point3f& center, const cv::Point3f& scene_center = cv::Point3f(0, 0, 0)) {
	// 相对场景中心的坐标
	double x = center.x - scene_center.x;
	double y = center.y - scene_center.y;
	double z = center.z - scene_center.z;

	// 极径（到极点的直线距离）
	double r = sqrt(x * x + y * y + z * z);
	// 极角θ（绕Z轴，atan2(y,x)，范围-π~π → 转换为0~2π）
	double theta = atan2(y, x);
	if (theta < 0) theta += 2 * M_PI;
	// 仰角φ（与XY平面的夹角，atan2(z, sqrt(x²+y²))）
	double phi = atan2(z, sqrt(x * x + y * y));

	return { r, theta, phi };
}

//bool Reconstructer::reconstructPrompt()
//{
//	if (points.empty())
//		return false;
//
//	for (std::size_t i = 0; i < points.size(); ++i)
//		vecPoints[points[i].template get<2>()].push_back(points[i]);
//
//	points.clear();
//	vectors.clear();
//	for (int i = 0; i < inputData->plane_normals.size(); i++)
//	{
//		cv::Point3f normal = inputData->plane_normals[i];
//		cv::Point3f center = inputData->plane_center[i];
//
//		vectors.push_back(
//			boost::tuple<Point, Vector, int>(
//				{ center.x, center.y, center.z },
//				{ normal.x, normal.y, normal.z },
//				vecPoints[i].size()
//			)
//		);
//	}
//
//	std::vector<cv::Point3f> centers;
//	for (int id : inputData->plane_ground_idx)
//		centers.push_back(inputData->plane_center[id]);
//	for (int id : inputData->plane_ceiling_idx)
//		centers.push_back(inputData->plane_center[id]);
//
//	cv::Point3f sceneCenter(0, 0, 0);
//	if (!centers.empty())
//		sceneCenter = computeCenterPoint(centers);
//
//	std::vector<std::tuple<int, double>> plane_info; // id, theta
//
//	for (int i = 0; i < vecPoints.size(); i++)
//	{
//		if (vecPoints[i].empty())
//			continue;
//
//		double r, theta, phi;
//		std::tie(r, theta, phi) =
//			calcPolarCoord(inputData->plane_center[i], sceneCenter);
//
//		plane_info.emplace_back(i, theta);
//	}
//
//	std::sort(plane_info.begin(), plane_info.end(),
//		[&](const auto& a, const auto& b)
//		{
//			int ia = std::get<0>(a);
//			int ib = std::get<0>(b);
//
//			/*if (vecPoints[ia].size() != vecPoints[ib].size())
//				return vecPoints[ia].size() > vecPoints[ib].size();*/
//
//			return std::get<1>(a) > std::get<1>(b);
//		});
//
//	for (auto& p : plane_info)
//	{
//		int id = std::get<0>(p);
//		for (auto& pt : vecPoints[id])
//			points.push_back(pt);
//	}
//
//	/*std::vector<Point_vector> vecPoints_sorted;
//
//	for (int new_id = 0; new_id < plane_info.size(); new_id++) {
//		int old_id = std::get<0>(plane_info[new_id]);
//
//		vecPoints_sorted.push_back(vecPoints[old_id]);
//	}
//	vecPoints = std::move(vecPoints_sorted);*/
//
//	for (auto& p : plane_info)
//	{
//		int id = std::get<0>(p);
//		std::cout << "plane=" << id
//			<< " count=" << vecPoints[id].size()
//			<< " theta=" << std::get<1>(p)
//			<< std::endl;
//	}
//
//	return reconstruct<SCIP_Solver>(points, model, vectors);
//}


bool Reconstructer::reconstructPrompt() 
{ 
	if (points.size() == 0) 
		return false; 
	int plane_index = 0; 
	for (std::size_t i = 0; i < points.size(); ++i) 
		vecPoints[points[i].template get<2>()].push_back(points[i]); 
	points.clear(); 
	for (int i = 0; i < inputData->plane_normals.size(); i++) 
	{ 
		cv::Point3f normalPoint = inputData->plane_normals[i]; 
		cv::Point3f centerPoint = inputData->plane_center[i]; 
		vectors.push_back(boost::tuple<Point, Vector, int>({ centerPoint.x,centerPoint.y,centerPoint.z }, 
			{ normalPoint.x,normalPoint.y,normalPoint.z }, vecPoints[i].size())); } 
	sort(vecPoints.begin(), vecPoints.end(), 
		[](Point_vector& a, Point_vector& b) 
		{ return a.size() > b.size(); }); 
	for (int i = 0; i < vecPoints.size(); i++) 
	{ 
		for (auto x : vecPoints[i]) 
		{ 
			points.push_back(x); 
		} 
	} 
	return reconstruct<SCIP_Solver>(points, model, vectors); 
}

#include <fstream>
#include <iostream>
#include <CGAL/Surface_mesh.h>

typedef CGAL::Simple_cartesian<double>::Point_3 Point;
typedef CGAL::Surface_mesh<Point> Surface_mesh;

bool save_model_as_ply(const Surface_mesh& model, const std::string& filepath) {
	// 修正1：用 std::ios::binary 替代 ios_binary
	std::ofstream out_file(filepath, std::ios::binary);
	if (!out_file) {
		std::cerr << "错误：无法打开文件 " << filepath << " 进行保存！" << std::endl;
		return false;
	}
	// 修正2：用 CGAL::IO::write_PLY 替代已弃用的 CGAL::write_ply
	if (!CGAL::IO::write_PLY(out_file, model)) {
		std::cerr << "错误：写入PLY文件失败！" << std::endl;
		out_file.close();
		return false;
	}
	out_file.close();
	std::cout << "模型已成功保存为PLY格式：" << filepath << std::endl;
	return true;
}

bool Reconstructer::ProcessingModel()
{
	modelProcessed = model;
	save_model_as_ply(model,"model.ply");
	modelProcessed.collect_garbage();
	typename Polygon_mesh::template Property_map<Face_descriptor, std::size_t> face_supporting_planes =
		modelProcessed.template property_map<Face_descriptor, std::size_t>("f:supp_index").first;
	if (face_supporting_planes == nullptr)
		return false;
	for (auto f : modelProcessed.faces())
		modelProcessed.remove_face(f);
	std::vector<std::vector<CGAL::SM_Vertex_index>> planeList;
	vector<CGAL::SM_Face_index> faceList;

	std::vector<int> beVisitH(modelProcessed.number_of_halfedges(), 0);
	for (auto h : modelProcessed.halfedges())
	{
		if (!beVisitH[h.idx()] && modelProcessed.is_border(h))
		{
			std::vector<CGAL::SM_Vertex_index> tempVerIndList;
			auto tempEdge = h;
			auto oneface = modelProcessed.face(modelProcessed.opposite(tempEdge));
			faceList.push_back(oneface);
			do
			{
				beVisitH[tempEdge.idx()] = 1;
				beVisitH[(modelProcessed.opposite(tempEdge)).idx()] = 1;

				tempVerIndList.push_back(modelProcessed.target(tempEdge));

				tempEdge = modelProcessed.next(tempEdge);
			} while (tempEdge != h);
			planeList.push_back(tempVerIndList);
		}
	}
	
	decltype(planeList) newPlaneList;
	for (int i = 0; i < planeList.size(); i++)
	{
		std::vector<CGAL::SM_Vertex_index> tempPlane;
		auto add_vertex = [&](int pre, int now, int next)
		{
			auto point_beg = modelProcessed.point(planeList[i][pre]);
			auto point_end = modelProcessed.point(planeList[i][next]);
			auto point_mid = modelProcessed.point(planeList[i][now]);
			auto approximateAngle = CGAL::approximate_angle(point_beg, point_mid, point_end);
			if (approximateAngle < 177.0) //阈值可调
			{
				tempPlane.push_back(planeList[i][now]);
			}
			//approximateAngle > 177. ? tempPlane : ;
		};

		add_vertex(planeList[i].size() - 1, 0, 1);
		for (int j = 1; j < planeList[i].size() - 1; j++)
			add_vertex(j - 1, j, j + 1);
		add_vertex(planeList[i].size() - 2, planeList[i].size() - 1, 0);

		newPlaneList.push_back(tempPlane);
		vecI face(tempPlane.begin(), tempPlane.end());
		contour.push_back(std::tuple<vecI, int, int>(face, face_supporting_planes[faceList[i]], -1));
	}

	/*
	typedef CGAL::Simple_cartesian<double> Kernel;
	typedef Kernel::Point_3  Point_3;
	typedef Kernel::Vector_3 Vector_3;
	for (int i = 0; i < newPlaneList.size(); i++)
	{
		if (newPlaneList[i].empty()) continue;

		// 核心绑定信息：平面ID+轮廓ID
		int bind_plane_idx = face_supporting_planes[faceList[i]]; // 绑定的平面ID（关键！）
		auto faceId = get<2>(vecPoints[bind_plane_idx][0]);
		CGAL::SM_Face_index bind_face = faceList[i];

		std::cout << "============================\n";
		std::cout << "[验证] 轮廓ID=" << i << " | 绑定平面ID=" << faceId << "\n"; // 重点输出平面ID

		// ---------- 1. 计算轮廓中心（单位：mm） ----------
		Vector_3 acc(0, 0, 0);
		for (auto v : newPlaneList[i])
		{
			Point_3 p = modelProcessed.point(v);
			acc = acc + (p - CGAL::ORIGIN);
		}
		acc = acc * (1.0 / newPlaneList[i].size());
		Point_3 contour_center = CGAL::ORIGIN + acc;

		// 绑定平面的中心（mm）
		cv::Point3f plane_center = inputData->plane_center[bind_plane_idx];

		std::cout << "[验证] 轮廓中心(mm): "
			<< contour_center.x() << ", "
			<< contour_center.y() << ", "
			<< contour_center.z() << "\n";

		std::cout << "[验证] 平面中心(mm): "
			<< plane_center.x << ", "
			<< plane_center.y << ", "
			<< plane_center.z << "\n";

		// ---------- 2. 两种距离计算（单位：mm） ----------
		// 2.1 中心距（轮廓中心-平面中心，参考）
		double center_distance = std::sqrt(
			std::pow(contour_center.x() - plane_center.x, 2) +
			std::pow(contour_center.y() - plane_center.y, 2) +
			std::pow(contour_center.z() - plane_center.z, 2)
		);
		std::cout << "[验证] 中心距(mm)=" << center_distance << "\n";

		// 2.2 轮廓到平面的实际距离（更靠谱：轮廓顶点到平面的平均最短距离）
		// 平面方程：ax + by + cz + d = 0（从plane_center和plane_normal推导）
		cv::Point3f plane_normal = inputData->plane_normals[bind_plane_idx];
		double a = plane_normal.x;
		double b = plane_normal.y;
		double c = plane_normal.z;
		double d = -(a * plane_center.x + b * plane_center.y + c * plane_center.z); // 平面方程的d值

		// 计算轮廓所有顶点到平面的距离，取平均值（更准确）
		double total_plane_dist = 0.0;
		int valid_vertex_cnt = 0;
		for (auto v : newPlaneList[i])
		{
			Point_3 p = modelProcessed.point(v);
			double x = p.x(), y = p.y(), z = p.z();
			// 点到平面的距离公式：|ax+by+cz+d| / sqrt(a²+b²+c²)
			double dist = fabs(a * x + b * y + c * z + d) / std::sqrt(a * a + b * b + c * c);
			total_plane_dist += dist;
			valid_vertex_cnt++;
		}
		double avg_plane_dist = valid_vertex_cnt > 0 ? (total_plane_dist / valid_vertex_cnt) : 0.0;
		std::cout << "[验证] 轮廓到平面平均距离(mm)=" << avg_plane_dist << "\n";

		// 距离判断（mm为单位，阈值设为50mm更合理）
		if (avg_plane_dist > 50.0) // 实际距离>50mm，判定绑定错误
			std::cout << "[⚠] 平面绑定错误（轮廓到平面距离过大）\n";
		else if (center_distance > 1000.0) // 中心距>1m但实际距离近，仅提醒
			std::cout << "[提醒] 中心距远但实际贴合平面（可能是大平面）\n";

		// ---------- 3. 法向量验证（优化异常处理） ----------
		auto h = modelProcessed.halfedge(bind_face);
		if (h == modelProcessed.null_halfedge()) {
			std::cout << "[⚠] 无效面，跳过法向量验证\n";
			continue;
		}

		Point_3 p0 = modelProcessed.point(modelProcessed.source(h));
		Point_3 p1 = modelProcessed.point(modelProcessed.target(h));
		Point_3 p2 = modelProcessed.point(modelProcessed.target(modelProcessed.next(h)));

		Vector_3 v1 = p1 - p0;
		Vector_3 v2 = p2 - p0;
		Vector_3 face_normal = CGAL::cross_product(v1, v2);

		double len_sq = face_normal.squared_length();
		if (len_sq > 1e-12)
			face_normal = face_normal / std::sqrt(len_sq);
		else {
			std::cout << "[⚠] 面法向量无效（共线顶点）\n";
			continue;
		}

		std::cout << "[验证] 面法向: "
			<< face_normal.x() << ", "
			<< face_normal.y() << ", "
			<< face_normal.z() << "\n";

		std::cout << "[验证] 平面法向: "
			<< plane_normal.x << ", "
			<< plane_normal.y << ", "
			<< plane_normal.z << "\n";

		double dot = face_normal.x() * plane_normal.x + face_normal.y() * plane_normal.y + face_normal.z() * plane_normal.z;
		std::cout << "[验证] 法向量点积=" << dot << "\n";

		if (fabs(dot) < 0.8)
			std::cout << " 法向量不匹配（绑定平面错误）\n";
	}*/

	for (int i = 0; i < newPlaneList.size(); i++)
	{
		for (int j = i + 1; j < newPlaneList.size(); j++)
		{
			if (face_supporting_planes[faceList[i]] == face_supporting_planes[faceList[j]])
			{
				if (pointsInFace(newPlaneList[i], modelProcessed, newPlaneList[j][0]))
					get<2>(contour[j]) = i;
				else if (pointsInFace(newPlaneList[j], modelProcessed, newPlaneList[i][0]))
					get<2>(contour[i]) = j;
				else
					;
			}
		}
	}
	for (auto x : newPlaneList)
		modelProcessed.add_face(x);
	getSaveAsObjContourTreeList();

#ifdef TEST_PROCESSING_MODEL
#if	SAVE_SEG_POINTS

	auto test_contour = [&]() {
		std::ofstream contourStream(m_name + ".contour");
		for (auto x : contour)
		{
			contourStream << get<2>(x) << " " << get<1>(x);
			for (auto y : get<0>(x))
				contourStream << " " << y;
			contourStream << endl;
		}
		contourStream.close();
	};
	return true;
#endif

	auto mesh_contour = [&]() {
		std::unordered_map<int, vector<int>> mapEdgeHoleList;
		std::unordered_map<int, int> mapEdgeHoleListTemp;
		//hierarchy is adjacency table,use level traversal
		//two level struct,and screen
		for (size_t i = 0; i < contour.size(); i++)
			mapEdgeHoleListTemp[i] = get<2>(contour[i]);
		for (auto it = mapEdgeHoleListTemp.begin(); it != mapEdgeHoleListTemp.end(); it++)
			if (it->second > -1 &&
				mapEdgeHoleListTemp[it->second] == -1)
				mapEdgeHoleList[it->second].push_back(it->first);
			else if (it->second == -1 &&
				mapEdgeHoleList.count(it->first) == 0)
				mapEdgeHoleList[it->first] = {};
		std::vector<Point3fArray> test_points_2Dcontours(contour.size());
		for (int i = 0; i < contour.size(); i++)
		{
			for (auto x : get<0>(contour[i]))
			{
				auto p = modelProcessed.point(CGAL::SM_Vertex_index(x));
				test_points_2Dcontours[i].push_back({ (float)p.x(),(float)p.y(),(float)p.z() });
			}
		}

		unordered_map<int, vector<int>> sameFace;

	};
#ifdef _WIN32
	SYSTEMTIME tm;
	GetLocalTime(&tm);
#else
	auto now = std::chrono::system_clock::now();
#endif

	concreteMesherDelauney mesher;

	auto test_outPoints = [&]() {
		for (auto x : contour)
		{
			CGAL::Point_set_3<Point, Vector> outPoints;
			outPoints.add_normal_map();
			for (auto y : vecPoints[get<1>(x)])
				outPoints.insert(get<0>(y), get<1>(y));
			//if (CGAL::IO::write_point_set("D:/data/poly/vertex" + to_string(get<1>(x)) + ".xyz", outPoints));
		}

		//if (CGAL::IO::write_XYZ("D:/data/poly/vertex1.xyz", vecPoints[0]));
	};
	test_outPoints();
#endif 
	std::cout << contour.size() << std::endl;
	return true;
}

bool Reconstructer::pointsInFace(std::vector<Vertex_descriptor>& face, const Polygon_mesh& mesh, Vertex_descriptor point) {
	using Plane = CGAL::Simple_cartesian<double>::Plane_3;
	using Polygon = CGAL::Polygon_2<Cartesian>;
	// The supporting plane of each face
	typename Polygon_mesh::template Property_map<Face_descriptor, const Plane*> face_supporting_planes =
		mesh.template property_map<Face_descriptor, const Plane*>("f:supp_plane").first;
	const typename Polygon_mesh::template Property_map<Vertex_descriptor, Point>& coords = mesh.points();
	// We do everything by projecting the point onto the face's supporting plane
	const Plane* supporting_plane = new Plane(coords[face[0]], coords[face[1]], coords[face[2]]);
	//const Plane* supporting_plane =new Plane(0,0,1,0);

	CGAL::Point_set_3<Point, Vector> outPoints;
	outPoints.add_normal_map();

	Polygon plg; // The projection of the face onto it supporting plane
	for (auto x : face)
	{
		auto p = mesh.point(x);
		auto q = supporting_plane->to_2d(p);
		outPoints.insert({ p.x(),p.y(),p.z() }, { 0,0,1 });

		plg.push_back(q);
	}
	//if (CGAL::IO::write_point_set("D:/data/poly/vertex" + to_string(908) + "3d.xyz", outPoints));

	if (plg.size() < 3 || !plg.is_simple())
	{
		std::cerr << plg << endl;
		CGAL::Point_set_3<Point, Vector> outPoints;
		outPoints.add_normal_map();
		for (auto y : plg)
			outPoints.insert({ y.x(),y.y(),0 }, { 0,0,1 });
		//if (CGAL::IO::write_point_set("D:/data/poly/vertex" + to_string(908) + ".xyz", outPoints));
		return false;
	}

	if (plg.bounded_side(supporting_plane->to_2d(mesh.point(point))) == CGAL::ON_BOUNDED_SIDE)
		return true;
	else
		return false;
}
#include <opencv2/core/eigen.hpp>
//#include <unsupported/Eigen/CXX11/Tensor>
bool Reconstructer::testReconstructerMethodInterface(std::string type)
{
	std::vector<cv::Point3f> points;
	std::vector<unsigned char> reflects;
	Point3f centre(2152,-613,358);
	Point3f normal(0,1,0);
	cv::Mat dstImg;
	type = "E:\\test\\plane\\66_6.txt";
	auto GetPointsReflect = [&]() {
		ifstream file(type);
		float x, y, z;
		int r, g, b;
		char reflect;
		int reflect_i; 
		while (file)
		{
			file >> x>>y>>z>>reflect_i;
			points.push_back({ x,y,z });
			reflects.push_back(reflect_i);
		}
		file.close();
	};
	GetPointsReflect();

//#define REFLECT
#ifdef REFLECT
	auto BuildReflectImage = [&]() {
		Vector3f centre_E(centre.x, centre.y, centre.z);
		Vector3f normal_E(normal.x, normal.y, normal.z);
		auto word2local = [&]() {

			Vector3f localCentre(0.f, 0.f, 0.f);
			Vector3f localNormal(0.f, 0.f, 1.f);
			auto rotMatrix_3 = (Eigen::Quaternionf::FromTwoVectors(normal_E, localNormal)).matrix();
			Matrix4f rotMatrix_4;
			rotMatrix_4 << rotMatrix_3(0, 0), rotMatrix_3(0, 1), rotMatrix_3(0, 2),0 ,
				rotMatrix_3(1, 0), rotMatrix_3(1, 1), rotMatrix_3(1, 2), 0,
				rotMatrix_3(2, 0), rotMatrix_3(2, 1), rotMatrix_3(2, 2), 0,
				0, 0, 0, 1;
			Matrix4f moveMatrix_4;
			moveMatrix_4 << 1, 0, 0, -centre_E[0],
				0, 1, 0, -centre_E[1],
				0, 0, 1, -centre_E[2],
				0, 0, 0, 1;

			return rotMatrix_4* moveMatrix_4;
		}; 
		Matrix4f mat = word2local();
		Eigen::MatrixX3f points_E(points.size(),3);
		auto transPoints = [&]() {
			for (int i = 0; i < points.size(); ++i)
			{
				Vector4f temp(points[i].x,points[i].y,points[i].z,1);
				temp = mat * temp;
				points_E.row(i) = Vector3f(temp[0], temp[1], temp[2]);
			}
		}; 
		transPoints();

		Vector3f voxelSize(10, 10, 10);
		MatrixXf box(2,3);
		box<<
		points_E.col(0).minCoeff(),
		points_E.col(1).minCoeff(),
		points_E.col(2).minCoeff(),
		points_E.col(0).maxCoeff(),
		points_E.col(1).maxCoeff(),
		points_E.col(2).maxCoeff();
		//cout << box;
		Vector3i mapSize(ceil((box(1, 0) - box(0, 0)) / voxelSize(0)),
			ceil((box(1, 1) - box(0, 1)) / voxelSize(1)),
			ceil ((box(1, 2) - box(0, 2)) / voxelSize(2)));
		MatrixXf hashMap(mapSize[0], mapSize[1]);
		dstImg = cv::Mat::eye(mapSize[0], mapSize[1], CV_8UC1);
		//cv::imshow("test", dstImg*255);
		//cv::waitKey(0);
		std::vector<std::vector<std::vector<int>>> indexMapMat(mapSize[0],vector<vector<int>>(mapSize[1],vector<int>(0)));
		std::vector<std::vector<Eigen::Matrix3Xf>> meanCloudPointsMat(mapSize[0], vector<Matrix3Xf>(mapSize[1], Matrix3Xf(3,0)));
		MatrixXi reflectMap(mapSize[0], mapSize[1]);
		for (int id = 0; id < points_E.rows(); ++id)
		{
			Vector3f temp = ((points_E.row(id) - box.row(0)));
			Vector3i idNow(int(temp(0) / voxelSize(0)), int(temp(1) / voxelSize(1)), int(temp(2) / voxelSize(2)));
			indexMapMat[idNow(0)][idNow(1)].push_back(id);
			(meanCloudPointsMat[idNow(0)][idNow(1)]).conservativeResize(3, (meanCloudPointsMat[idNow(0)][idNow(1)]).cols() + 1);
			meanCloudPointsMat[idNow(0)][idNow(1)].col(meanCloudPointsMat[idNow(0)][idNow(1)].cols() - 1) =
				points_E.row(id);
			reflectMap(idNow(0), idNow(1))=max(reflectMap(idNow(0), idNow(1)),int(reflects[id]));
			//Vector3i idNow()
		}
		//Tensor<unsigned char, 3> NormalImage(mapSize[0], mapSize[1], 3);
		Matrix<Matrix<unsigned char, 3,1>,Dynamic, Dynamic> NormalImage_m(mapSize[0], mapSize[1]);
		//Matrix<Vector<int, 3>,Dynamic, Dynamic> NormalImage_m(mapSize[0], mapSize[1]);//this is wrong 
		cv::Mat NormalImage_cv(mapSize[0],mapSize[1],CV_8UC3);
		auto normalMap_triangleKernel = [&]() {
			//right triangle 
			for (int i = 0; i < mapSize[0]-1; ++i)
			{
				for (int j = 0; j < mapSize[1]-1; ++j)
				{
					if (meanCloudPointsMat[i][j].cols() &&
						meanCloudPointsMat[i][j + 1].cols()&&
						meanCloudPointsMat[i + 1][j].cols())
					{
						Vector3f o = meanCloudPointsMat[i][j].col(0);
						Vector3f right = meanCloudPointsMat[i][j + 1].col(0);
						Vector3f down = meanCloudPointsMat[i + 1][j].col(0);
						Vector3f v_o2r = (right - o).normalized();
						Vector3f v_o2d = (down - o).normalized();
						Vector3f v_o = (v_o2r.cross(v_o2d)).normalized() * 255;

						//NormalImage(i, j,0) = (unsigned char)v_o(0);
						//NormalImage(i, j, 1) = (unsigned char)v_o(1);
						//NormalImage(i, j, 2) = (unsigned char)v_o(2);
						//((unsigned char)v_o(0),(unsigned char)v_o(1),(unsigned char)v_o(2) );
						NormalImage_cv.at<cv::Vec3b>(i, j)[0] = (unsigned char)v_o(0);
						NormalImage_cv.at<cv::Vec3b>(i, j)[1] = (unsigned char)v_o(1);
						NormalImage_cv.at<cv::Vec3b>(i, j)[2] = (unsigned char)v_o(2);
						//Mat_<Vec3b> _I = I;
						//for (int i = 0; i < I.rows; ++i)
						//	for (int j = 0; j < I.cols; ++j)
						//	{
						//		_I(i, j)[0] = table[_I(i, j)[0]];
						//		_I(i, j)[1] = table[_I(i, j)[1]];
						//		_I(i, j)[2] = table[_I(i, j)[2]];
						//	}
						//I = _I;
					}

				}
			}

			cv::imshow("test", NormalImage_cv);
			cv::waitKey(0);
		};
		normalMap_triangleKernel();
		cv::Mat reflectMap_cv;
		cv::Mat reflectMap_cv_8UC1;
		cv::eigen2cv(reflectMap, reflectMap_cv);
		reflectMap_cv.convertTo(reflectMap_cv_8UC1, CV_8UC1);
		cv::imshow("test", reflectMap_cv_8UC1);
		cv::waitKey(0);

	};
	BuildReflectImage();
#endif
	return false;
}
bool Reconstructer::BuildReflectImage(int id, cv::Mat& dstImg)
{
    //code struct changed by hgj
	std::vector<cv::Point3f>& points = inputData->plane_xyz[id];
	std::vector<unsigned char>& reflects = inputData->plane_reflect[id];
	cv::Point3f center = inputData->plane_center[id];
	cv::Point3f normal_inner = inputData->plane_normals[id];
	if (points.size()==0|| reflects.size()== 0 )
	{
		return false;
	}

	if (ComputeVectorDotProduct<float, Point3f>(normal_inner, center) > 0.f)
	{
		//point to center
		normal_inner *= -1;
	}
	BuildRefect::BuildRefectImageFuc(points, reflects, center, normal_inner, dstImg);
	return true;
}