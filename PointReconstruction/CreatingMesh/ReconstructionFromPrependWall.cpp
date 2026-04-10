#include "ReconstructionFromPrependWall.h"
#include "jansson.h"
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include <Eigen/Dense>
#include "mesh_tool.h"
#include "InOutData.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_mesher_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_vertex_base_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>
#include <CGAL/lloyd_optimize_mesh_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Surface_mesh/Surface_mesh.h>

using namespace std;
static std::string markers_json;


cv::Point3d CreatingMesh::coordinateTransformation(cv::Point3d & plane_vertices, cv::Point3d &plane_normal, double room_depth)
{
	//计算法向量的反方向与坐标轴之间的夹角
	cv::Point3d inverse_wall_normal = cv::Point3d((double)-plane_normal.x, (double)-plane_normal.y, (double)-plane_normal.z);

	//计算平移后的坐标
	cv::Point3d corner_transpose;
	corner_transpose.x = plane_vertices.x + room_depth * inverse_wall_normal.x / sqrt(inverse_wall_normal.x * inverse_wall_normal.x + inverse_wall_normal.y * inverse_wall_normal.y);
	corner_transpose.y = plane_vertices.y + room_depth * inverse_wall_normal.y / sqrt(inverse_wall_normal.x * inverse_wall_normal.x + inverse_wall_normal.y * inverse_wall_normal.y);
	corner_transpose.z = plane_vertices.z;

	return corner_transpose;
}

void CreatingMesh::Appendsufix(std::string &filedir)
{
	if (!(filedir[filedir.length() - 1] == '/' || filedir[filedir.length() - 1] == '\\'))
	{
		if (filedir.length() > 2)
		{
			if (filedir.find("\\") != string::npos)
				filedir = filedir + "\\";
			else
				filedir = filedir + "/";
		}
	}
}

void CreatingMesh::buildMesh(std::string jsonPath, float ratio, std::string output_path)
{
	markers_json = jsonPath;
	json_error_t error;

	json_t *markersJson = json_load_file(markers_json.c_str(), 0, &error);
	if (!markersJson)
	{
		std::cout << "error_import_paramter: on line " << error.line << ", " << error.text << std::endl;
		json_decref(markersJson);
		return;
	}

	json_t* target_plane_normal = json_object_get(markersJson, "target_plane_normal");
	json_t* target_plane_vertices = json_object_get(markersJson, "target_plane_vertices");

	cv::Point3d plane_normal;
	plane_normal.x = json_number_value(json_array_get(target_plane_normal, 0));
	plane_normal.y = json_number_value(json_array_get(target_plane_normal, 1));
	plane_normal.z = json_number_value(json_array_get(target_plane_normal, 2));

	std::vector<cv::Point3d> plane_vertices;
	for (int i = 0; i < json_array_size(target_plane_vertices); i++)
	{
		json_t* vertice_json = json_array_get(target_plane_vertices, i);

		cv::Point3f center;
		center.x = json_number_value(json_array_get(vertice_json, 0));
		center.y = json_number_value(json_array_get(vertice_json, 1));
		center.z = json_number_value(json_array_get(vertice_json, 2));

		plane_vertices.emplace_back(center);
	}

	std::vector<cv::Point3d> plane_foot_vertices;
	for (int i = 0; i < plane_vertices.size(); i++)
	{
		if (plane_vertices[i].z < 0)
		{
			plane_foot_vertices.emplace_back(plane_vertices[i]);
		}
	}

	double plane_width = sqrt((plane_foot_vertices[0].x - plane_foot_vertices[1].x) * (plane_foot_vertices[0].x - plane_foot_vertices[1].x) + \
								(plane_foot_vertices[0].y - plane_foot_vertices[1].y) * (plane_foot_vertices[0].y - plane_foot_vertices[1].y) + \
								(plane_foot_vertices[0].z - plane_foot_vertices[1].z) * (plane_foot_vertices[0].z - plane_foot_vertices[1].z));

	//Calculating the four vertices on the opposite wall
	std::vector<cv::Point3d> opposite_vertices;
	std::vector< pair<cv::Point3d, cv::Point3d>> vertices;
	for (int i = 0; i < plane_vertices.size(); i++)
	{
		cv::Point3d transCorners;
		pair<cv::Point3d, cv::Point3d> vertice_pair;
		transCorners = coordinateTransformation(plane_vertices[i], plane_normal, ratio * plane_width);
		opposite_vertices.emplace_back(transCorners);
		vertice_pair.first = plane_vertices[i];
		vertice_pair.second = transCorners;
		vertices.emplace_back(vertice_pair);
	}

	//get the vertices of ground and ceiling
	std::vector<cv::Point3d> floor_vertices;
	std::vector<cv::Point3d> ceiling_vertices;
	std::vector<std::vector<cv::Point3d>> walls_vertices;

	//Sort the walls counterclockwise by vertices of floor
	walls_vertices.emplace_back(plane_vertices); //vertices of original plane
	if (plane_vertices[0].z < 0 && plane_vertices[1].z < 0)
	{
		std::vector<cv::Point3d> tmp_vertices;
		tmp_vertices.emplace_back(vertices[3].first);
		tmp_vertices.emplace_back(vertices[3].second);
		tmp_vertices.emplace_back(vertices[0].second);
		tmp_vertices.emplace_back(vertices[0].first);
		walls_vertices.emplace_back(tmp_vertices);
		tmp_vertices.clear();

		tmp_vertices.emplace_back(opposite_vertices[3]);
		tmp_vertices.emplace_back(opposite_vertices[2]);
		tmp_vertices.emplace_back(opposite_vertices[1]);
		tmp_vertices.emplace_back(opposite_vertices[0]);
		walls_vertices.emplace_back(tmp_vertices);
		tmp_vertices.clear();

		tmp_vertices.emplace_back(vertices[1].first);
		tmp_vertices.emplace_back(vertices[1].second);
		tmp_vertices.emplace_back(vertices[2].second);
		tmp_vertices.emplace_back(vertices[2].first);
		walls_vertices.emplace_back(tmp_vertices);
		tmp_vertices.clear();

		floor_vertices.emplace_back(vertices[0].first);
		floor_vertices.emplace_back(vertices[0].second);
		floor_vertices.emplace_back(vertices[1].second);
		floor_vertices.emplace_back(vertices[1].first);

		ceiling_vertices.emplace_back(vertices[2].first);
		ceiling_vertices.emplace_back(vertices[2].second);
		ceiling_vertices.emplace_back(vertices[3].second);
		ceiling_vertices.emplace_back(vertices[3].first);
	}
	else if (plane_vertices[0].z < 0 && plane_vertices[1].z > 0)
	{
		std::vector<cv::Point3d> tmp_vertices;
		tmp_vertices.emplace_back(vertices[2].first);
		tmp_vertices.emplace_back(vertices[2].second);
		tmp_vertices.emplace_back(vertices[3].second);
		tmp_vertices.emplace_back(vertices[3].first);
		walls_vertices.emplace_back(tmp_vertices);
		tmp_vertices.clear();

		tmp_vertices.emplace_back(opposite_vertices[3]);
		tmp_vertices.emplace_back(opposite_vertices[2]);
		tmp_vertices.emplace_back(opposite_vertices[1]);
		tmp_vertices.emplace_back(opposite_vertices[0]);
		walls_vertices.emplace_back(tmp_vertices);
		tmp_vertices.clear();

		tmp_vertices.emplace_back(vertices[0].first);
		tmp_vertices.emplace_back(vertices[0].second);
		tmp_vertices.emplace_back(vertices[1].second);
		tmp_vertices.emplace_back(vertices[1].first);
		walls_vertices.emplace_back(tmp_vertices);
		tmp_vertices.clear();

		floor_vertices.emplace_back(vertices[3].first);
		floor_vertices.emplace_back(vertices[3].second);
		floor_vertices.emplace_back(vertices[0].second);
		floor_vertices.emplace_back(vertices[0].first);

		ceiling_vertices.emplace_back(vertices[1].first);
		ceiling_vertices.emplace_back(vertices[1].second);
		ceiling_vertices.emplace_back(vertices[2].second);
		ceiling_vertices.emplace_back(vertices[2].first);
	}
	else if (plane_vertices[0].z > 0 && plane_vertices[1].z > 0)
	{
		std::vector<cv::Point3d> tmp_vertices;
		tmp_vertices.emplace_back(vertices[1].first);
		tmp_vertices.emplace_back(vertices[1].second);
		tmp_vertices.emplace_back(vertices[2].second);
		tmp_vertices.emplace_back(vertices[2].first);
		walls_vertices.emplace_back(tmp_vertices);
		tmp_vertices.clear();

		tmp_vertices.emplace_back(opposite_vertices[3]);
		tmp_vertices.emplace_back(opposite_vertices[2]);
		tmp_vertices.emplace_back(opposite_vertices[1]);
		tmp_vertices.emplace_back(opposite_vertices[0]);
		walls_vertices.emplace_back(tmp_vertices);
		tmp_vertices.clear();

		tmp_vertices.emplace_back(vertices[3].first);
		tmp_vertices.emplace_back(vertices[3].second);
		tmp_vertices.emplace_back(vertices[0].second);
		tmp_vertices.emplace_back(vertices[0].first);
		walls_vertices.emplace_back(tmp_vertices);
		tmp_vertices.clear();

		floor_vertices.emplace_back(vertices[2].first);
		floor_vertices.emplace_back(vertices[2].second);
		floor_vertices.emplace_back(vertices[3].second);
		floor_vertices.emplace_back(vertices[3].first);

		ceiling_vertices.emplace_back(vertices[0].first);
		ceiling_vertices.emplace_back(vertices[0].second);
		ceiling_vertices.emplace_back(vertices[1].second);
		ceiling_vertices.emplace_back(vertices[1].first);
	}
	else if (plane_vertices[0].z > 0 && plane_vertices[1].z < 0)
	{
		std::vector<cv::Point3d> tmp_vertices;
		tmp_vertices.emplace_back(vertices[0].first);
		tmp_vertices.emplace_back(vertices[0].second);
		tmp_vertices.emplace_back(vertices[1].second);
		tmp_vertices.emplace_back(vertices[1].first);
		walls_vertices.emplace_back(tmp_vertices);
		tmp_vertices.clear();

		tmp_vertices.emplace_back(opposite_vertices[3]);
		tmp_vertices.emplace_back(opposite_vertices[2]);
		tmp_vertices.emplace_back(opposite_vertices[1]);
		tmp_vertices.emplace_back(opposite_vertices[0]);
		walls_vertices.emplace_back(tmp_vertices);
		tmp_vertices.clear();

		tmp_vertices.emplace_back(vertices[2].first);
		tmp_vertices.emplace_back(vertices[2].second);
		tmp_vertices.emplace_back(vertices[3].second);
		tmp_vertices.emplace_back(vertices[3].first);
		walls_vertices.emplace_back(tmp_vertices);
		tmp_vertices.clear();

		floor_vertices.emplace_back(vertices[1].first);
		floor_vertices.emplace_back(vertices[1].second);
		floor_vertices.emplace_back(vertices[2].second);
		floor_vertices.emplace_back(vertices[2].first);

		ceiling_vertices.emplace_back(vertices[3].first);
		ceiling_vertices.emplace_back(vertices[3].second);
		ceiling_vertices.emplace_back(vertices[0].second);
		ceiling_vertices.emplace_back(vertices[0].first);
	}


	//mesh plane
	typedef CGAL::Simple_cartesian<float> Kernel;
	CGAL::Surface_mesh<Kernel::Point_3> mesh;

	std::string meshPath;

	if (output_path.length() > 2)
	{
		meshPath = output_path + "meshesCreated\\";
		Appendsufix(meshPath);
	}
	system(("rmdir /q /s " + meshPath).c_str());
	IOData::createDirectory(meshPath);

	//walls
	for (int i = 0; i < walls_vertices.size(); i++)
	{
		std::vector<cv::Point3d> wall_vertices = walls_vertices[i];
		auto v0 = mesh.add_vertex({ wall_vertices[0].x, wall_vertices[0].y, wall_vertices[0].z });
		auto v1 = mesh.add_vertex({ wall_vertices[1].x, wall_vertices[1].y, wall_vertices[1].z });
		auto v2 = mesh.add_vertex({ wall_vertices[2].x, wall_vertices[2].y, wall_vertices[2].z });
		mesh.add_face(v0, v1, v2);

		v0 = mesh.add_vertex({ wall_vertices[0].x, wall_vertices[0].y, wall_vertices[0].z });
		v1 = mesh.add_vertex({ wall_vertices[2].x, wall_vertices[2].y, wall_vertices[2].z });
		v2 = mesh.add_vertex({ wall_vertices[3].x, wall_vertices[3].y, wall_vertices[3].z });
		mesh.add_face(v0, v1, v2);

		CGAL::IO::write_OBJ(meshPath + "meshCreated_" + std::to_string(i) + ".obj", mesh);
		mesh.clear();
	}

	//ground
	auto v0 = mesh.add_vertex({ floor_vertices[0].x, floor_vertices[0].y, floor_vertices[0].z });
	auto v1 = mesh.add_vertex({ floor_vertices[1].x, floor_vertices[1].y, floor_vertices[1].z });
	auto v2 = mesh.add_vertex({ floor_vertices[2].x, floor_vertices[2].y, floor_vertices[2].z });
	mesh.add_face(v0, v1, v2);

	v0 = mesh.add_vertex({ floor_vertices[0].x, floor_vertices[0].y, floor_vertices[0].z });
	v1 = mesh.add_vertex({ floor_vertices[2].x, floor_vertices[2].y, floor_vertices[2].z });
	v2 = mesh.add_vertex({ floor_vertices[3].x, floor_vertices[3].y, floor_vertices[3].z });
	mesh.add_face(v0, v1, v2);

	CGAL::IO::write_OBJ(meshPath + "meshCreated_" + std::to_string(4) + ".obj", mesh);
	mesh.clear();

	//ceiling
	v0 = mesh.add_vertex({ ceiling_vertices[0].x, ceiling_vertices[0].y, ceiling_vertices[0].z });
	v1 = mesh.add_vertex({ ceiling_vertices[1].x, ceiling_vertices[1].y, ceiling_vertices[1].z });
	v2 = mesh.add_vertex({ ceiling_vertices[2].x, ceiling_vertices[2].y, ceiling_vertices[2].z });
	mesh.add_face(v0, v1, v2);

	v0 = mesh.add_vertex({ ceiling_vertices[0].x, ceiling_vertices[0].y, ceiling_vertices[0].z });
	v1 = mesh.add_vertex({ ceiling_vertices[2].x, ceiling_vertices[2].y, ceiling_vertices[2].z });
	v2 = mesh.add_vertex({ ceiling_vertices[3].x, ceiling_vertices[3].y, ceiling_vertices[3].z });
	mesh.add_face(v0, v1, v2);

	CGAL::IO::write_OBJ(meshPath + "meshCreated_" + std::to_string(5) + ".obj", mesh);
	mesh.clear();


	

	std::vector<cv::Point3f> g_obj_points;
	std::vector<cv::Vec3i> g_face;
	size_t found_squared;
	found_squared = meshPath.find_last_of("\\/");
	std::string curPath_created = meshPath.substr(0, found_squared);
	std::string filename_created = meshPath.substr(found_squared + 1, meshPath.length());
	found_squared = curPath_created.find_last_of("\\/");
	std::string parentPath_squared = curPath_created.substr(0, found_squared);
	ofstream outtriangles(parentPath_squared + "//" + filename_created + "meshCreated.obj");
	int idx_p = 0;
	int idx_f = 0;
	CMeshTool meshTool;
	for (int i = 0; i < 6; i++) 
	{
		std::vector<cv::Point3f> obj_points;
		std::vector<cv::Vec3i> face;
		std::string obj_path = meshPath + "meshCreated_" + to_string(i) + ".obj";
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
}