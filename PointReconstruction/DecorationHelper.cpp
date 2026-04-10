#include "DecorationHelper.h"
#include "concreteMesher.h"
#include "InOutData.h"
#include "../Measurement/MeasureLevelnessRange.h"
#include "RoomMeasurement.h"
#include "../Common/MathOperation.h"
#include "../Common/Contour.h"
#include "../Common/buildRefectImgFuc.h"
#include "../Common/ContourPre.h"
#include "Measurement/RGBD/RGB2XYZ/BallsDetection_unre.h"
#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include "util_math.hpp"
#include "PointReconstructionGetConfig.h"
#include "CreatingMesh/ReconstructionFromPrependWall.h"

#ifdef MESH_COMPARE
#include "surfaceReconAPI.h"
#endif

using namespace Eigen;
//Default dir name and file name
const std::string  output_json_name = "output.json";
const std::string  origin_mesh_dir  = "origin_meshes";
const std::string  world_origin_meshes_dir = "world_origin_meshes";
const std::string  structed_mesh_dir = "meshes";
const std::string  squared_mesh_dir = "squared_meshes";
const std::string  squared_mesh_dir_min = "squared_meshes_min";
const std::string  squared_mesh_dir_min05 = "squared_meshes_min05";
const std::string  world_structed_mesh_dir = "world_meshes";
const std::vector<std::string> wall_dire_string = { "U", "E", "S", "W", "N" };
const std::vector<std::string> wall_type_string = { "unkown", "wall", "beam", "column", "ground", "ceiling" };
const std::vector<std::string> hole_type_string = { "unkown", "window", "door" };

#define SAVE_SQUARE_IMAGE 1
#define DEBUG_CONTOUR_IMAGE_TEST false

DecorationHelper::DecorationHelper(std::string  workDir, int seg_mode):
	mWorkDir(workDir),
	mSegMode(seg_mode),
	mPointRec(NULL),
	mRoomMeasurement(NULL)
{
	mpStageMatrix = NULL;
	mPointRec = new PointReconstruction();
	mRoomMeasurement = new RoomMeasurement(mPointRec);

	Appendsufix(mWorkDir);

	mJsonFileName = mWorkDir + output_json_name;
	mOriMeshDir = mWorkDir + origin_mesh_dir;
	mWorldOriMeshDir = mWorkDir + world_origin_meshes_dir;

	Appendsufix(mOriMeshDir);
	Appendsufix(mWorldOriMeshDir);

	mStructedMeshDir = mWorkDir + structed_mesh_dir;
	mSquaredMeshDir = mWorkDir + squared_mesh_dir;
	mSquaredMeshDirMin = mWorkDir + squared_mesh_dir_min;
	mSquaredMeshDirMin05 = mWorkDir + squared_mesh_dir_min05;
	mWorldStructedMeshDir = mWorkDir + world_structed_mesh_dir;
	Appendsufix(mStructedMeshDir);
	Appendsufix(mSquaredMeshDir);
	Appendsufix(mSquaredMeshDirMin);
	Appendsufix(mWorldStructedMeshDir);
}

DecorationHelper::~DecorationHelper()
{
	if (mRoomMeasurement) {
		delete mRoomMeasurement;
		mRoomMeasurement = nullptr;
	}
	if (mPointRec) {
		std::cout << "Delete mPointRec: " << mPointRec << std::endl;
		try {
			delete mPointRec;
			mPointRec = nullptr;
		}
		catch (std::exception& e) {
			std::cout << "Exception when delete mPointRec: " << e.what() << std::endl;
		}
	}

	if (mpStageMatrix) {
		delete mpStageMatrix;
		mpStageMatrix = nullptr;
	}
}

void DecorationHelper::SetStageMatrix(StageMatrix   *pstageMatrix)
{
	mpStageMatrix = pstageMatrix;
}

StageMatrix *DecorationHelper::GetStageMatrix()
{
	return mpStageMatrix;
}
void DecorationHelper::Appendsufix(std::string &filedir)
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

std::string DecorationHelper::GetOriMeshDir(void)
{
	return mOriMeshDir;
}

std::string DecorationHelper::GetOriMeshName(int wallId)
{
	std::ostringstream filename;
	filename << mOriMeshDir << "mesh" << wallId << ".obj";
	return filename.str();
}

std::string DecorationHelper::GetStructedMeshDir(void)
{
	return mStructedMeshDir;
}

std::string DecorationHelper::GetStructedMeshName(int wallId)
{
	std::ostringstream filename;
	filename << mStructedMeshDir << "mesh" << wallId << ".obj";
	return filename.str();
}

void DecorationHelper::SetJsonFileName(std::string file_name)
{
	mJsonFileName = file_name;
}

std::string DecorationHelper::GetJsonFileName(void)
{
	return mJsonFileName;
}

bool DecorationHelper::UpdateDoorWindowInfo(std::string updateJson)
{
	json_error_t error;

	json_t *root = json_load_file(updateJson.c_str(), 0, &error);
	if (!root)
	{
		log_error("update door windows info: on line %d: %s\n", error.line, error.text);
		json_decref(root);
		return false;
	}
	json_t* plane_infos = json_object_get(root, "plane_info");
	if (!json_is_array(plane_infos))
	{
		log_error("update door windows info: plane_info is not an object\n");
		json_decref(root);
		return false;
	}

	std::vector<StructuredPlane> planes;
	log_info("update door windows info: plane_info size=%d\n", json_array_size(plane_infos));
	//std::cout << json_array_size(plane_infos) << std::endl;
	for (int index = 0; index < json_array_size(plane_infos); index++)
	{
		auto plane_info = json_array_get(plane_infos, index);

		int wall_id = json_integer_value(json_object_get(plane_info, "id"));
		std::string wall_type =  json_string_value(json_object_get(plane_info, "type"));

		auto normal_values = json_object_get(plane_info, "normal");
		cv::Point3f normals;
		normals.x = json_number_value(json_array_get(normal_values, 0));
		normals.y = json_number_value(json_array_get(normal_values, 1));
		normals.z = json_number_value(json_array_get(normal_values, 2));

		auto center_values = json_object_get(plane_info, "center");
		cv::Point3f center;
		center.x = json_number_value(json_array_get(center_values, 0));
		center.y = json_number_value(json_array_get(center_values, 1));
		center.z = json_number_value(json_array_get(center_values, 2));

		std::string wall_dire = json_string_value(json_object_get(plane_info, "direction"));

		int height = json_integer_value(json_object_get(plane_info, "height"));
		int width = json_integer_value(json_object_get(plane_info, "width"));

		//std::string mesh_dir = json_string_value(json_object_get(plane_info, "mesh"));
		//std::string reflectmap = json_string_value(json_object_get(plane_info, "reflectmap"));
		//std::string contourmap = json_string_value(json_object_get(plane_info, "defectmap"));

		
		auto vertices_values = json_object_get(plane_info, "vertices");
		std::vector<cv::Point3f> vertices = JsonLoadVertices(vertices_values);

		StructuredPlane plane;
		plane.center = center;

		plane.direction = (ePlane_Direction)GetMatchId(wall_dire_string, wall_dire);
		plane.type = (ePlane_Type)GetMatchId(wall_type_string, wall_type);
		plane.normal = normals;
		plane.id = wall_id;
		plane.wall_height = height;
		plane.wall_width = width;
		plane.vertices = vertices;
		
		
		int door_window_count = json_number_value(json_object_get(plane_info, "door_window_count"));

		auto door_window = json_object_get(plane_info, "door_window");

		std::vector<StructuredHole>  holes;
		door_window_count = json_array_size(door_window);
		for (int dwid = 0; dwid < door_window_count; dwid++)
		{
			auto door_window_info = json_array_get(door_window, dwid);
			int hole_id = json_integer_value(json_object_get(door_window_info, "id"));
			std::string hole_type = json_string_value(json_object_get(door_window_info, "type"));
			int hole_height = json_integer_value(json_object_get(door_window_info, "height"));
			int hole_width = json_integer_value(json_object_get(door_window_info, "width"));
			auto hole_vertices_values = json_object_get(door_window_info, "vertices");
			std::vector<cv::Point3f> vertices = JsonLoadVertices(hole_vertices_values);
			
			StructuredHole hole;
			hole.height = hole_height;
			hole.width = hole_width;
			hole.id = hole_id;
			hole.vertice = vertices;
			hole.type = (eHole_Type)GetMatchId(hole_type_string, hole_type);

			holes.push_back(hole);
		}

		plane.holes = holes;
		planes.push_back(plane);

	}
	json_decref(root);

	mPointRec->UpdateDoorWindowInfo(planes);
	RedoRommSquare();
	return true;
}

int DecorationHelper::GetMatchId(std::vector<std::string> string_array, std::string value)
{
	for (int i = 0; i < string_array.size(); i++)
	{
		if (string_array[i].compare(value) == 0)
			return i;
	}
	return 0;
}

bool DecorationHelper::PtInPolygon(const cv::Point2f& triCentroid, const std::vector<cv::Point2f>& contour_squared_floor, int &nCount)
{
	int nCross = 0;
	for (int i = 0; i < nCount; i++)
	{
		cv::Point2f p1 = contour_squared_floor[i];
		cv::Point2f p2 = contour_squared_floor[(i + 1) % nCount];

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
}

std::vector<cv::Point3f> DecorationHelper::JsonLoadVertices(json_t* vertices_array)
{
	std::vector<cv::Point3f>  vertices;

	for (int i = 0; i < json_array_size(vertices_array); i++)
	{
		auto vertice_json = json_array_get(vertices_array, i);

		cv::Point3f center;
		center.x = json_number_value(json_array_get(vertice_json, 0));
		center.y = json_number_value(json_array_get(vertice_json, 1));
		center.z = json_number_value(json_array_get(vertice_json, 2));
		vertices.push_back(center);
	}
	return vertices;
}

/**
* brief:根据降采样后的点云数据及旋转矩阵 合并后的信息进行平面分割，生成轮廓图
* file_paths:降采样后的点云数据
* RTs：旋转矩阵
* station_pos：站点位置
* mergeDataDir：合并后的点云朝向
* compass_file：空
* compass_value：朝向
* [out] generate_contour：生成轮廓图
*/
bool DecorationHelper::PlaneSegmentation(std::vector<std::string> file_paths, std::vector<cv::Mat> RTs,cv::Mat relaRT,
	                                     std::vector<std::vector<float>> station_pos, float mergeDataDir, std::string compass_file, float compass_value, bool generate_contour)
{
	bool bres = false;

	bres =  mPointRec->PlaneSegmentation(file_paths,RTs,relaRT,(MEASUREMENT_MODE)mSegMode, station_pos, mergeDataDir, compass_file, compass_value, generate_contour);
	log_debug("mPointRec->PlaneSegmentation's bool is: %d",static_cast<int> (bres));

	// 检测一米线，测量
	DWORD start_prepare = GetTickCount();
	if (!generate_contour)
	{
		if (bres)
		{
			mRoomMeasurement->DetectOneMeterLine();
			mRoomMeasurement->MeasureStraightness();
	#if 0 //Don't need OneMeterLine adjustment for now
			if (mRoomMeasurement->HasOneMeterLine())
			{
				mPointRec->MovetoOneMeterLine(mRoomMeasurement->GetOneMerterLinePos());
			}
	#endif
		}
	}
	DWORD end_prepare = GetTickCount();
	log_info("DetectOneMeterLine and MeasureStraightness time = %lu ms.", end_prepare - start_prepare);
	//cout << "DetectOneMeterLine and MeasureStraightness time = " << end_prepare - start_prepare << "ms." << "\n";
	return bres;
}

void DecorationHelper::MarkerBallDetection(std::string sense_path, std::string output_path)
{
	BallDetection ballDetection;
	std::string centerPts_path;
	std::vector<std::vector<cv::Point3d>> sphere_center_vec_all;
	sphere_center_vec_all = ballDetection.ROI_map2points(sense_path);
	ballDetection.writeInfo_sphere_argument(sphere_center_vec_all, output_path);
}

bool DecorationHelper::IsMultiCeiling(void)
{
	return mPointRec->GetPlanes().plane_ceiling_idx.size() > 1;
}

void DecorationHelper::RedoRommSquare(eRoomSquare_type type)
{
	mRoomMeasurement->contour_squared = mRoomMeasurement->MakeRoomSquare(
		mRoomMeasurement->rotation_matrix,
		mRoomMeasurement->contour_squared1,
		mRoomMeasurement->holes_projected, 
		mRoomMeasurement->holes_sq, mRoomMeasurement->holes_sq);
}

extern std::vector <cv::Point3f> tempbr;
void DecorationHelper::SaveRoomSquareAsImage(std::string dir)
{
	bool cruler = false;
	bool gruler = false;
	int pointcount = mRoomMeasurement->contour_squared.size();
	std::vector<Point3f> roomContour = mPointRec->GetRoomContour();
	MatrixXf squared_pts(pointcount, 2);
	MatrixXf squared_pts1(pointcount, 2);
	MatrixXf squared_pts2(pointcount, 2);
	MatrixXf ground(pointcount, 2);

	if ((mRoomMeasurement->ceiling_h_onemeter_int.size()>0) && (mRoomMeasurement->ceiling_h_onemeter_int[0].size() == 2) && (mRoomMeasurement->ceiling_h_onemeter[0].second.size() == 2))
		cruler = true;
	if ((mRoomMeasurement->ground_h_onemeter_int.size()>0) && (mRoomMeasurement->ground_h_onemeter_int[0].size() == 2) && (mRoomMeasurement->ground_h_onemeter[0].second.size() == 2))
		gruler = true;
	MatrixXf ceiling_rule(3, 2);
	MatrixXf ceiling_rule1(3, 2);
	if (cruler) {
		ceiling_rule(0, 0) = mRoomMeasurement->ceiling_h_onemeter_int[0][0].first.x;
		ceiling_rule(0, 1) = mRoomMeasurement->ceiling_h_onemeter_int[0][0].first.y;
		ceiling_rule(1, 0) = mRoomMeasurement->ceiling_h_onemeter[0].second[0].second.x;
		ceiling_rule(1, 1) = mRoomMeasurement->ceiling_h_onemeter[0].second[0].second.y;
		ceiling_rule(2, 0) = mRoomMeasurement->ceiling_h_onemeter_int[0][0].second.x;
		ceiling_rule(2, 1) = mRoomMeasurement->ceiling_h_onemeter_int[0][0].second.y;

		ceiling_rule1(0, 0) = mRoomMeasurement->ceiling_h_onemeter_int[0][1].first.x;
		ceiling_rule1(0, 1) = mRoomMeasurement->ceiling_h_onemeter_int[0][1].first.y;
		ceiling_rule1(1, 0) = mRoomMeasurement->ceiling_h_onemeter[0].second[1].second.x;
		ceiling_rule1(1, 1) = mRoomMeasurement->ceiling_h_onemeter[0].second[1].second.y;
		ceiling_rule1(2, 0) = mRoomMeasurement->ceiling_h_onemeter_int[0][1].second.x;
		ceiling_rule1(2, 1) = mRoomMeasurement->ceiling_h_onemeter_int[0][1].second.y;
	}

	MatrixXf ground_rule(3, 2);
	MatrixXf ground_rule1(3, 2);
	if (gruler) {
		ground_rule(0, 0) = mRoomMeasurement->ground_h_onemeter_int[0][0].first.x;
		ground_rule(0, 1) = mRoomMeasurement->ground_h_onemeter_int[0][0].first.y;
		ground_rule(1, 0) = mRoomMeasurement->ground_h_onemeter[0].second[0].second.x;
		ground_rule(1, 1) = mRoomMeasurement->ground_h_onemeter[0].second[0].second.y;
		ground_rule(2, 0) = mRoomMeasurement->ground_h_onemeter_int[0][0].second.x;
		ground_rule(2, 1) = mRoomMeasurement->ground_h_onemeter_int[0][0].second.y;
		/*cout << ground_rule(0, 0) << " " << ground_rule(0, 1) << " "
			<< ground_rule(1, 0) << " " << ground_rule(1, 1) << " "
			<< ground_rule(2, 0) << " " << ground_rule(2, 1) << endl;*/

		ground_rule1(0, 0) = mRoomMeasurement->ground_h_onemeter_int[0][1].first.x;
		ground_rule1(0, 1) = mRoomMeasurement->ground_h_onemeter_int[0][1].first.y;
		ground_rule1(1, 0) = mRoomMeasurement->ground_h_onemeter[0].second[1].second.x;
		ground_rule1(1, 1) = mRoomMeasurement->ground_h_onemeter[0].second[1].second.y;
		ground_rule1(2, 0) = mRoomMeasurement->ground_h_onemeter_int[0][1].second.x;
		ground_rule1(2, 1) = mRoomMeasurement->ground_h_onemeter_int[0][1].second.y;
		/*cout << ground_rule1(0, 0) << " " << ground_rule1(0, 1) << " "
			<< ground_rule1(1, 0) << " " << ground_rule1(1, 1) << " "
			<< ground_rule1(2, 0) << " " << ground_rule1(2, 1) << endl;*/
	}
	//std::vector <cv::Point3f> tempbr = tempr;
	MatrixXf Axis(2, 2);
	if (tempbr.size()>0) {
		Axis(0, 0) = tempbr[0].x;
		Axis(0, 1) = tempbr[0].y;
		Axis(1, 0) = tempbr[1].x;
		Axis(1, 1) = tempbr[1].y;
		//cout << "Axis: " << tempbr[0] << " , " << tempbr[1] << endl;
	}

	for (int i = 0; i < mRoomMeasurement->contour_squared.size(); i++)
	{
		squared_pts(i, 0) = mRoomMeasurement->contour_squared[i].first.x;
		squared_pts(i, 1) = mRoomMeasurement->contour_squared[i].first.y;
		squared_pts1(i, 0) = mRoomMeasurement->contour_squared1[i].first.x;
		squared_pts1(i, 1) = mRoomMeasurement->contour_squared1[i].first.y;
		squared_pts2(i, 0) = mRoomMeasurement->contour_squared05[i].x;
		squared_pts2(i, 1) = mRoomMeasurement->contour_squared05[i].y;
		ground(i, 0) = roomContour[i].x;
		ground(i, 1) = roomContour[i].y;
	}

	ground.col(1) *= -1;
	squared_pts.col(1) *= -1;
	squared_pts1.col(1) *= -1;
	squared_pts2.col(1) *= -1;
	if (cruler) {
		ceiling_rule.col(1) *= -1;
		ceiling_rule1.col(1) *= -1;
	}

	if (gruler) {
		ground_rule.col(1) *= -1;
		ground_rule1.col(1) *= -1;
	}
	float minx = ground.col(0).minCoeff();
	float miny = ground.col(1).minCoeff();
	ground.col(0) = (ground.col(0).array() - minx).matrix();
	ground.col(1) = (ground.col(1).array() - miny).matrix();

	if (tempbr.size() > 0) {
		Axis.col(1) *= -1;
		Axis.col(0) = (Axis.col(0).array() - minx).matrix();
		Axis.col(1) = (Axis.col(1).array() - miny).matrix();
		Axis = (Axis.array() + 32).matrix();
	}

	float cols = ground.col(0).maxCoeff();
	float rows = ground.col(1).maxCoeff();
	squared_pts.col(0) = (squared_pts.col(0).array() - minx).matrix();
	squared_pts.col(1) = (squared_pts.col(1).array() - miny).matrix();
	squared_pts1.col(0) = (squared_pts1.col(0).array() - minx).matrix();
	squared_pts1.col(1) = (squared_pts1.col(1).array() - miny).matrix();
	squared_pts2.col(0) = (squared_pts2.col(0).array() - minx).matrix();
	squared_pts2.col(1) = (squared_pts2.col(1).array() - miny).matrix();
	if (cruler) {
		ceiling_rule.col(0) = (ceiling_rule.col(0).array() - minx).matrix();
		ceiling_rule.col(1) = (ceiling_rule.col(1).array() - miny).matrix();
		ceiling_rule1.col(0) = (ceiling_rule1.col(0).array() - minx).matrix();
		ceiling_rule1.col(1) = (ceiling_rule1.col(1).array() - miny).matrix();
		ceiling_rule1 = (ceiling_rule1.array() + 32).matrix();
		ceiling_rule = (ceiling_rule.array() + 32).matrix();
	}
	if (gruler) {
		ground_rule.col(0) = (ground_rule.col(0).array() - minx).matrix();
		ground_rule.col(1) = (ground_rule.col(1).array() - miny).matrix();
		ground_rule1.col(0) = (ground_rule1.col(0).array() - minx).matrix();
		ground_rule1.col(1) = (ground_rule1.col(1).array() - miny).matrix();
		ground_rule = (ground_rule.array() + 32).matrix();
		ground_rule1 = (ground_rule1.array() + 32).matrix();
	}

	ground = (ground.array() + 32).matrix();
	squared_pts = (squared_pts.array() + 32).matrix();
	squared_pts1 = (squared_pts1.array() + 32).matrix();
	squared_pts2 = (squared_pts2.array() + 32).matrix();
	
	cv::Mat draw_img = cv::Mat::zeros(rows + 64, cols + 64, CV_8UC3);

	int ref_wall_id = mPointRec->GetReferenceWall();
	std::vector<int> wall_list = mPointRec->GetWallList();

	int ref_wall_contour_id = -1;
	auto ref_wall_it = std::find(wall_list.begin(), wall_list.end(), ref_wall_id);
	if (ref_wall_it != wall_list.end())
		ref_wall_contour_id = ref_wall_it - wall_list.begin();

	std::vector<std::pair<int, std::vector<std::pair<float, cv::Point3f>>>> flateness_defect = mPointRec->GetPlanes().removeCritical_flateness_defect;
	for (int i = 0; i < pointcount; i++)
	{
		cv::Point2i linestart = cv::Point2i(ground(i, 0), ground(i, 1));
		cv::Point2i lineend = cv::Point2i(ground((i+1)% pointcount, 0), ground((i + 1) % pointcount, 1));
		cv::Point2i linestart_s = cv::Point2i(squared_pts(i, 0), squared_pts(i, 1));
		cv::Point2i lineend_s = cv::Point2i(squared_pts((i + 1) % pointcount, 0), squared_pts((i + 1) % pointcount, 1));
		cv::Point2i linestart_s1 = cv::Point2i(squared_pts1(i, 0), squared_pts1(i, 1));
		cv::Point2i lineend_s1 = cv::Point2i(squared_pts1((i + 1) % pointcount, 0), squared_pts1((i + 1) % pointcount, 1));
		cv::Point2i linestart_s2 = cv::Point2i(squared_pts2(i, 0), squared_pts2(i, 1));
		cv::Point2i lineend_s2 = cv::Point2i(squared_pts2((i + 1) % pointcount, 0), squared_pts2((i + 1) % pointcount, 1));
	//	cout << linestart_s1 << " " << linestart_s2 << endl;
		int thickness = 1;
		int lineType = 8;
	
		if (ref_wall_contour_id == i) {
			//cout <<" line  : "<< linestart <<" , "<< lineend << endl;
			cv::line(draw_img, linestart, lineend, cv::Scalar(255, 255, 255),2 , lineType);
		}
		else {
			//cout << " line  : " << linestart << " , " << lineend << endl;
			cv::line(draw_img, linestart, lineend, cv::Scalar(0, 255, 255), 1, lineType);
		}
		//cout << " line_sq: " << linestart_s << " , " << lineend_s << endl;
		cv::line(draw_img, linestart_s, lineend_s, cv::Scalar(0, 0, 255), 4, lineType);
		cv::line(draw_img, linestart_s1, lineend_s1, cv::Scalar(0, 255, 0), 4, lineType);
		cv::line(draw_img, linestart_s2, lineend_s2, cv::Scalar(255,255,255), 4, lineType);
		
		std::stringstream line_id;
		line_id << i;
		int font_face = cv::FONT_HERSHEY_COMPLEX;
		double font_scale = 2;
		cv::Point2i origin = (linestart + lineend) / 2;
		cv::putText(draw_img, line_id.str(), origin, font_face, font_scale, cv::Scalar(0, 255, 0), thickness, 8, 0);
		//cv::putText(draw_img, line_id.str(), (linestart_s + lineend_s) / 2, font_face, font_scale, cv::Scalar(0, 255, 0), thickness, 8, 0);


#if 0
		if (wall_list[i] != -1)
		{
			for(int k=0; k <flateness_defect.size();k++)
			{
				if (flateness_defect[k].first == wall_list[i])
				{
					for (auto &defect_pt : flateness_defect[k].second)
					{
						cv::Point3f cur_pt = defect_pt.second;
						cur_pt.x -= minx;
						cur_pt.y *= -1;
						cur_pt.y -= miny;
						cur_pt.x += 32;
						cur_pt.y += 32;
						cv::circle(draw_img, cv::Point(cur_pt.x, cur_pt.y), 1, cv::Scalar(255, 255, 255), -1);
					}
				}
			}
		}
#endif
		//}
	}
	if (tempbr.size() > 0) {
		cv::Point2i linestarta = cv::Point2i(Axis(0, 0), Axis(0, 1));
		cv::Point2i lineenda = cv::Point2i(Axis(1, 0), Axis(1, 1));
		//cout << "ground_axis_line: " << linestarta << " , " << lineenda << endl;
		cv::line(draw_img, linestarta, lineenda, cv::Scalar(0, 0, 255), 3, 8);
	}
	if (cruler) {
		for (int i = 0; i < 2; i++) {
			cv::Point2i linestart = cv::Point2i(ceiling_rule(i, 0), ceiling_rule(i, 1));
			cv::Point2i lineend = cv::Point2i(ceiling_rule(i + 1, 0), ceiling_rule(i + 1, 1));
			cv::line(draw_img, linestart, lineend, cv::Scalar(0, 255, 0), 1, 8);
		}
		for (int i = 0; i < 2; i++) {
			cv::Point2i linestart = cv::Point2i(ceiling_rule1(i, 0), ceiling_rule1(i, 1));
			cv::Point2i lineend = cv::Point2i(ceiling_rule1(i + 1, 0), ceiling_rule1(i + 1, 1));
			cv::line(draw_img, linestart, lineend, cv::Scalar(0, 255, 0), 3, 8);
		}
	}
	if (gruler) {
		for (int i = 0; i < 2; i++) {
			cv::Point2i linestart = cv::Point2i(ground_rule(i, 0), ground_rule(i, 1));
			cv::Point2i lineend = cv::Point2i(ground_rule(i + 1, 0), ground_rule(i + 1, 1));
			cv::line(draw_img, linestart, lineend, cv::Scalar(0, 0, 255), 1, 8);
		}
		for (int i = 0; i < 2; i++) {
			cv::Point2i linestart = cv::Point2i(ground_rule1(i, 0), ground_rule1(i, 1));
			cv::Point2i lineend = cv::Point2i(ground_rule1(i + 1, 0), ground_rule1(i + 1, 1));
			cv::line(draw_img, linestart, lineend, cv::Scalar(0, 0, 255), 3, 8);
		}
	}
	log_info("Save room square image to %s", (dir + "square_ness.jpg").c_str());
	//cout << "Save room square image to " << dir + "square_ness.jpg" << endl;
//	cv::flip(draw_img, draw_img, 0);
	cv::imwrite(dir + "square_ness.jpg", draw_img);
}
cv::Point3f mUniformNormals(const cv::Point3f normal, const cv::Point3f center)
{
	if (normal.dot(center) > 0.f)
	{
		return -normal;
	}
	return normal;
}

void DecorationHelper::SaveHolesSquareAsImage(std::string dir)
{
	std::vector<StructuredPlane>  sPlanes = mPointRec->GetStructuredPlanes();
	std::vector<int> wallList  = mPointRec->GetWallList();
	for (auto wall : wallList)
	{
		if (wall == -1)
			continue;
		//cout << "SaveHolesSquareAsImage mRoomMeasurement->holes_sq[wall].size(): " << mRoomMeasurement->holes_sq[wall].size()
			//<< " mRoomMeasurement->holes_projected[wall]: " << mRoomMeasurement->holes_projected[wall].size() << endl;
		if (mRoomMeasurement->holes_sq[wall].size() < 1)
			continue;

		std::stringstream ss;
		ss << wall;
		StructuredPlane sPlanesCur = sPlanes[wall];
	//	cout << "sPlanes[m].holes_sq.size(): " << mRoomMeasurement->holes_sq.size() << endl;

		cv::Point3f normal_plane = sPlanesCur.normal;
		normal_plane = -mUniformNormals(normal_plane, sPlanesCur.center);
		cv::Point3f rot_axis = { 0.f, 1.f, 0.f };
		cv::Point3f cross = Util_Math::ComputeVectorCrossProduct(normal_plane, rot_axis);
		float angle_rot = (cross.z > 0.f) ?
			acosf(normal_plane.dot(rot_axis) / std::sqrt(std::pow(normal_plane.x, 2) + std::pow(normal_plane.y, 2))) :
			-acosf(normal_plane.dot(rot_axis) / std::sqrt(std::pow(normal_plane.x, 2) + std::pow(normal_plane.y, 2)));
		//std::cout << angle_rot << std::endl;
		//std::cout << angle_rot * 180 / 3.1415926 << std::endl;
		cv::Mat Brotation_matrix = cv::Mat::zeros(3, 3, CV_32FC1);
		Brotation_matrix.at<float>(0, 0) = cos(angle_rot);
		Brotation_matrix.at<float>(0, 1) = -sin(angle_rot);
		Brotation_matrix.at<float>(1, 0) = sin(angle_rot);
		Brotation_matrix.at<float>(1, 1) = cos(angle_rot);
		Brotation_matrix.at<float>(2, 2) = 1.f;

		std::vector<cv::Point3f> vertices_cur = MathOperation::plane_rot(Brotation_matrix, sPlanesCur.vertices); 
		int vertices_cnt = vertices_cur.size();
		MatrixXf ground(vertices_cnt, 2);
		for (int i = 0; i < vertices_cnt; i++)
		{
			//cout << "vertices: " << vertices_cur[i] << endl;
			ground(i, 0) = vertices_cur[i].x;
			ground(i, 1) = vertices_cur[i].z;
		}
		ground.col(1) *= -1;
		float minx = ground.col(0).minCoeff();
		float miny = ground.col(1).minCoeff();
		ground.col(0) = (ground.col(0).array() - minx).matrix();
		ground.col(1) = (ground.col(1).array() - miny).matrix();
		float cols = ground.col(0).maxCoeff();
		float rows = ground.col(1).maxCoeff();
		ground = (ground.array() + 32).matrix();
		cv::Mat draw_img = cv::Mat::zeros(rows + 64, cols + 64, CV_16SC3);

		for (int i = 0; i < vertices_cnt; i++)
		{
			cv::Point2i linestart = cv::Point2i(ground(i, 0), ground(i, 1));
			cv::Point2i lineend = cv::Point2i(ground((i + 1) % vertices_cnt, 0), ground((i + 1) % vertices_cnt, 1));
			int thickness = 1;
			int lineType = 8;
			//cout << " line  : " << linestart << " , " << lineend << endl;
			cv::line(draw_img, linestart, lineend, cv::Scalar(0, 255, 255), 1, lineType);
#if 0
			std::stringstream line_id;
			line_id << i;
			int font_face = cv::FONT_HERSHEY_COMPLEX;
			double font_scale = 2;
			cv::Point2i origin = (linestart + lineend) / 2;
			cv::putText(draw_img, line_id.str(), origin, font_face, font_scale, cv::Scalar(255, 0, 0), thickness, 8, 0);
#endif
		}

		std::vector<std::vector<cv::Point3f>> choles = mRoomMeasurement->holes_projected[wall];
		for (int n = 0; n < choles.size(); n++) 
		{
			int pointcount = choles[n].size();
			MatrixXf squared_pts(pointcount, 2);
			std::vector<cv::Point3f> cholesn = MathOperation::plane_rot(Brotation_matrix, choles[n]);
			for (int k = 0; k < cholesn.size(); k++) 
			{
				//cout << "holes_projected: " << choles[n][k] << endl;
				squared_pts(k, 0) = cholesn[k].x;
				squared_pts(k, 1) = cholesn[k].z;
			}
			squared_pts.col(1) *= -1;
			squared_pts.col(0) = (squared_pts.col(0).array() - minx).matrix();
			squared_pts.col(1) = (squared_pts.col(1).array() - miny).matrix();
			squared_pts = (squared_pts.array() + 32).matrix();
			for (int i = 0; i < pointcount; i++)
			{
				cv::Point2i linestart_s = cv::Point2i(squared_pts(i, 0), squared_pts(i, 1));
				cv::Point2i lineend_s = cv::Point2i(squared_pts((i + 1) % pointcount, 0), squared_pts((i + 1) % pointcount, 1));
				int thickness = 1;
				int lineType = 8;
				cv::line(draw_img, linestart_s, lineend_s, cv::Scalar(0, 0, 255), 1, lineType);
			}
		}
		std::vector<StructuredHole> choles_sq = mRoomMeasurement->holes_sq[wall];
		for (int n = 0; n < choles_sq.size(); n++)
		{
			std::vector<cv::Point3f> choles_sqn = MathOperation::plane_rot(Brotation_matrix, choles_sq[n].vertice);
			int pointcount = choles_sqn.size();
			MatrixXf squared_pts(pointcount, 2);	
			for (int k = 0; k < choles_sq[n].vertice.size(); k++)
			{
				//cout << "squared_pts: " << choles_sq[n].vertice[k] << endl;
				squared_pts(k, 0) = choles_sqn[k].x;
				squared_pts(k, 1) = choles_sqn[k].z;
			}
			squared_pts.col(1) *= -1;
			squared_pts.col(0) = (squared_pts.col(0).array() - minx).matrix();
			squared_pts.col(1) = (squared_pts.col(1).array() - miny).matrix();
			squared_pts = (squared_pts.array() + 32).matrix();
			for (int i = 0; i < pointcount; i++)
			{
				cv::Point2i linestart_s = cv::Point2i(squared_pts(i, 0), squared_pts(i, 1));
				cv::Point2i lineend_s = cv::Point2i(squared_pts((i + 1) % pointcount, 0), squared_pts((i + 1) % pointcount, 1));
				int thickness = 1;
				int lineType = 8;
				cv::line(draw_img, linestart_s, lineend_s, cv::Scalar(0, 255, 0), 1, lineType);
			}
		}
		
		cv::imwrite(dir + "holes_"+ss.str()+"_square.jpg", draw_img);
	}
}



bool DecorationHelper::StartMeasurement(int room_squareness_mode, std::string axisEqnConfig)
{
	mRoomMeasurement->axisEqnJsonPath = axisEqnConfig;
	mRoomMeasurement->leveness = mRoomMeasurement->MeasureGroundLevelness();
	mPointRec->MeasureFlatenessDefectFcn(true);

	std::cout << "GetRoomContour 'size: " << mPointRec->GetRoomContour().size() << "\n";
	if (mPointRec->GetRoomContour().size() < 4) {
		log_info("ERROR!!!! mPointRec->GetRoomContour().size(): %zu", mPointRec->GetRoomContour().size());
		//cout << "ERROR!!!! mPointRec->GetRoomContour().size(): " << mPointRec->GetRoomContour().size() << endl;
		return false;
	}

	mRoomMeasurement->contour_squared = mRoomMeasurement->MakeRoomSquare(
		mRoomMeasurement->rotation_matrix,
		mRoomMeasurement->contour_squared1,
		mRoomMeasurement->holes_projected,
		mRoomMeasurement->holes_sq, mRoomMeasurement->holes_sq1);

	std::vector<cv::Point3f> origin_points;
	std::vector<cv::Point3f> contour_squared_points;

	origin_points = mPointRec->GetRoomContour();
	for (const auto& pair : mRoomMeasurement->contour_squared) {
		contour_squared_points.push_back(pair.first);
	}

	log_info("origin_points'size: %zu", origin_points.size());
	log_info("contour_squared_points'size: %zu", contour_squared_points.size());
	mSquarenessDir = "dll_log/";
	IOData::createDirectory(mSquarenessDir);
	SaveDebugImages(origin_points, contour_squared_points,mSquarenessDir);

	std::vector<cv::Point3f> contour_flat;
	contour_flat.reserve(mRoomMeasurement->contour_squared.size());

	for (const auto& pair : mRoomMeasurement->contour_squared) {
		contour_flat.push_back(pair.first);
		//contour_flat.push_back(pair.first/1000.f);
	}

	// Save the contour_flat points to a PLY file for debugging
	std::string filename = "result.ply";
	std::cout << "result size: " << contour_flat.size() << "\n";
	IOData::SavePLYPoints3f(filename, contour_flat, false);


	std::vector<StructuredPlane>  Planes = mPointRec->GetStructuredPlanes();
	std::vector<int> WallList = mPointRec->GetWallList();
	std::vector<int> GrdList = mPointRec->GetPlaneList(ePLANE_GROUND);
#if 0
	cout << "\n\nGetStructuredPlanes A: " << Planes.size() << endl;
	for (int i = 0; i < Planes.size(); i++) {
		cout << "\ni:" << i << " id: " << Planes[i].id << endl;
		cout << "normal: " << Planes[i].normal << " center : " << Planes[i].center << endl;
		cout << "vertices:\n " << Planes[i].vertices << endl;
		for (int j = 0; j < Planes[i].holes.size(); j++)
			cout << "holes: \n" << Planes[i].holes[j].vertice << endl;
	}
#endif
	std::vector<cv::Point3f>RoomContour;// = mPointRec->GetRoomContour();

	for (int i = 0; i < mRoomMeasurement->contour_squared.size(); i++) 
	{
		RoomContour.push_back(mRoomMeasurement->contour_squared[i].first);
	}


	for (int i=0; i< WallList.size();i++) 
	{
		if (WallList[i] == -1) {
			continue;
		}
		if (Planes[WallList[i]].vertices.size() > 0){
			int rd = RoomContour.size();
			Planes[WallList[i]].vertices[0].x = Planes[WallList[i]].vertices[3].x= RoomContour[i].x;
			Planes[WallList[i]].vertices[0].y = Planes[WallList[i]].vertices[3].y= RoomContour[i].y;
			Planes[WallList[i]].vertices[1].x = Planes[WallList[i]].vertices[2].x= RoomContour[(i + 1) % rd].x;
			Planes[WallList[i]].vertices[1].y = Planes[WallList[i]].vertices[2].y= RoomContour[(i + 1) % rd].y;
			if (Planes[WallList[i]].holes.size() > 0) {
				Planes[WallList[i]].holes = mRoomMeasurement->holes_sq[WallList[i]];
			}
		}
#ifdef WALL_CONVPT
		Planes[WallList[i]].wallConvPt = mRoomMeasurement->wallConvPt[WallList[i]];
#endif		
		if (room_squareness_mode == SQUARE_BY_CONVEXITY) {
			if (mRoomMeasurement->convPtsF[WallList[i]]) {
				Planes[WallList[i]].convPt = mRoomMeasurement->convPts[WallList[i]];
				Planes[WallList[i]].convPtF = true;
			}
		}
	}
	Planes[GrdList[0]].vertices.clear();
	Planes[GrdList[0]].vertices = RoomContour;
	mPointRec->UpdateStructuredPlanesSquared(Planes);
	mPointRec->UpdateRoomContourSquared(RoomContour);

	RoomContour.clear();
	for (int i = 0; i < mRoomMeasurement->contour_squared1.size(); i++)
	{
		RoomContour.push_back(mRoomMeasurement->contour_squared1[i].first);
	}
	for (int i = 0; i < WallList.size(); i++)
	{
		if (WallList[i] == -1) {
			continue;
		}
		if (Planes[WallList[i]].vertices.size() > 0){
			int rd = RoomContour.size();
			Planes[WallList[i]].vertices[0].x = Planes[WallList[i]].vertices[3].x = RoomContour[i].x;
			Planes[WallList[i]].vertices[0].y = Planes[WallList[i]].vertices[3].y = RoomContour[i].y;
			Planes[WallList[i]].vertices[1].x = Planes[WallList[i]].vertices[2].x = RoomContour[(i + 1) % rd].x;
			Planes[WallList[i]].vertices[1].y = Planes[WallList[i]].vertices[2].y = RoomContour[(i + 1) % rd].y;
			if (Planes[WallList[i]].holes.size() > 0){
				Planes[WallList[i]].holes = mRoomMeasurement->holes_sq1[WallList[i]];
			}
		}
	}
	Planes[GrdList[0]].vertices.clear();
	Planes[GrdList[0]].vertices = RoomContour;
	mPointRec->UpdateStructuredPlanesSquaredMin(Planes);
	mPointRec->UpdateRoomContourSquaredMin(RoomContour);

	RoomContour.clear();
	for (int i = 0; i < mRoomMeasurement->contour_squared05.size(); i++)
	{
		RoomContour.push_back(mRoomMeasurement->contour_squared05[i]);
	}
	for (int i = 0; i < WallList.size(); i++)
	{
		if (WallList[i] == -1) {
			continue;
		}
		if (Planes[WallList[i]].vertices.size() > 0) {
			int rd = RoomContour.size();
			Planes[WallList[i]].vertices[0].x = Planes[WallList[i]].vertices[3].x = RoomContour[i].x;
			Planes[WallList[i]].vertices[0].y = Planes[WallList[i]].vertices[3].y = RoomContour[i].y;
			Planes[WallList[i]].vertices[1].x = Planes[WallList[i]].vertices[2].x = RoomContour[(i + 1) % rd].x;
			Planes[WallList[i]].vertices[1].y = Planes[WallList[i]].vertices[2].y = RoomContour[(i + 1) % rd].y;
			if (Planes[WallList[i]].holes.size() > 0) {
				Planes[WallList[i]].holes = mRoomMeasurement->holes_sq1[WallList[i]];
			}
		}
	}
	Planes[GrdList[0]].vertices.clear();
	Planes[GrdList[0]].vertices = RoomContour;
	mPointRec->UpdateStructuredPlanesSquaredMin05(Planes);
	mPointRec->UpdateRoomContourSquaredMin05(RoomContour);
#if SAVE_SQUARE_IMAGE
	log_info("Save Square Image to : %s", mSquarenessDir.c_str());
	//mPointRec->MeasureFlatenessDefectFcn(true);
	SaveRoomSquareAsImage(mSquarenessDir);
	//SaveHolesSquareAsImage(mSquarenessDir);
#endif
	return true;
}

void JsonSaveOnePointReturnArray(const cv::Point3f& point, json_t* details_item_array)
{
	json_array_append(details_item_array, json_real(point.x));
	json_array_append(details_item_array, json_real(point.y));
	json_array_append(details_item_array, json_real(point.z));
}

void JsonSaveOnePoint2dReturnArray(const cv::Point2f& point, json_t* details_item_array)
{
	json_array_append(details_item_array, json_real(point.x));
	json_array_append(details_item_array, json_real(point.y));
}

json_t* MakeArrayOfDetails(const std::vector<MeasurementRulerverticeStruct>& input_details)
{
	json_t *details_array = json_array();
	for (int i = 0; i < input_details.size(); i++)
	{
		if (!input_details[i].is_valid)
		{
			continue;
		}
		json_t *details_item_array = json_array();
		json_t* details_object = json_object();

		if (input_details[i].ruler_endpts.size() > 0)
		{
			JsonSaveOnePointReturnArray(input_details[i].ruler_endpts[0], details_item_array);
		}
		json_object_set(details_object, "point", details_item_array);
		json_t *details_item_array_2d = json_array();
		if (input_details[i].ruler_endpts_2d.size() > 0)
		{
			JsonSaveOnePoint2dReturnArray(input_details[i].ruler_endpts_2d[0], details_item_array_2d);
		}
		json_object_set(details_object, "point2d", details_item_array_2d);
		json_object_set(details_object, "value", json_real(input_details[i].value));
		//rulers
		json_t* rulers_object = json_object();
		if (input_details[i].endpt_intersect_dist.size() > 0)
		{
			json_t *ruler_value_item_array = json_array();
			json_array_append(ruler_value_item_array, json_real(input_details[i].endpt_intersect_dist[0].first));
			json_array_append(ruler_value_item_array, json_real(input_details[i].endpt_intersect_dist[0].second));
			json_object_set(rulers_object, "value", ruler_value_item_array);
			json_decref(ruler_value_item_array);
		}

		if (input_details[i].endpt_intersect_pts.size() > 0)
		{
			json_t *local_rulers_line_array = json_array();
			for (int m = 0; m < 2; m++)
			{
				json_t *local_ruler_line_item_array = json_array();
				json_t *local_ruler_line_unit_array = json_array();
				cv::Point3f localpoint = input_details[i].endpt_intersect_pts[0].first;
				if (m == 1)
				{
					localpoint = input_details[i].endpt_intersect_pts[0].second;
				}

				JsonSaveOnePointReturnArray(localpoint, local_ruler_line_unit_array);
				json_array_append(local_ruler_line_item_array, details_item_array);
				json_array_append(local_ruler_line_item_array, local_ruler_line_unit_array);
				json_array_append(local_rulers_line_array, local_ruler_line_item_array);
				json_decref(local_ruler_line_unit_array);
				json_decref(local_ruler_line_item_array);
			}
			json_object_set(rulers_object, "line", local_rulers_line_array);
			json_decref(local_rulers_line_array);
		}

		json_object_set(details_object, "rulers", rulers_object);
		json_decref(rulers_object);
		json_decref(details_item_array);
		json_array_append(details_array, details_object);
		json_decref(details_object);
	}
	return details_array;
}
json_t* DecorationHelper::MakeVerticesJson(std::vector<cv::Point3f> vertices)
{
	json_t *vertice_array = json_array();
	for (auto pt : vertices)
	{
		json_t *pt_array = json_array();
		json_array_append(pt_array, json_real(pt.x));
		json_array_append(pt_array, json_real(pt.y));
		json_array_append(pt_array, json_real(pt.z));
		json_array_append(vertice_array, pt_array);
		json_decref(pt_array);
	}
		
	return vertice_array;
}

json_t* DecorationHelper::MakeRulerJson(std::pair< cv::Point3f, cv::Point3f> ruler)
{
	json_t *vertice_array = json_array();

	json_t *pt_array_f = json_array();
	json_array_append(pt_array_f, json_real(ruler.first.x));
	json_array_append(pt_array_f, json_real(ruler.first.y));
	json_array_append(pt_array_f, json_real(ruler.first.z));
	json_array_append(vertice_array, pt_array_f);
	json_decref(pt_array_f);

	json_t *pt_array_s = json_array();
	json_array_append(pt_array_s, json_real(ruler.first.x));
	json_array_append(pt_array_s, json_real(ruler.first.y));
	json_array_append(pt_array_s, json_real(ruler.first.z));
	json_array_append(vertice_array, pt_array_s);
	json_decref(pt_array_s);

	return vertice_array;
}

json_t* DecorationHelper::MakeGapPairJson(const GapPairResult& r) {
	json_t* obj = json_object();
	json_object_set(obj, "valid", json_boolean(r.valid));

	if (r.valid) {
		json_t* ceil_arr = json_array();
		json_array_append(ceil_arr, json_real(r.ceilPt.x));
		json_array_append(ceil_arr, json_real(r.ceilPt.y));
		json_array_append(ceil_arr, json_real(r.ceilPt.z));

		json_t* floor_arr = json_array();
		json_array_append(floor_arr, json_real(r.floorPt.x));
		json_array_append(floor_arr, json_real(r.floorPt.y));
		json_array_append(floor_arr, json_real(r.floorPt.z));

		json_object_set(obj, "ceil", ceil_arr);
		json_object_set(obj, "floor", floor_arr);
		json_object_set(obj, "gap", json_real(r.gap));

		json_decref(ceil_arr);
		json_decref(floor_arr);
	}

	return obj;
}
bool DecorationHelper::MakeArrayOriWallJson(const PlaneCutResultInterface& cutPlane, json_t* plane_array)
{
	std::vector<int> plane_wall_idx = cutPlane.plane_wall_idx;
	//door window corner
	std::vector<std::vector<MeasureDoorWindow::DoorWindowInfo>> door_window_info= cutPlane.door_window_info;

	//plane corner points
	std::vector<std::vector<cv::Point3f>> plane_corners= cutPlane.plane_corners;//add 20230714
	
	if (plane_wall_idx.size()==0 || plane_corners.size()==0)
	{
		log_info("MakeArrayOriWallJson wall_idx or corners size is zero!");
		//std::cout << "MakeArrayOriWallJson wall_idx or corners size is zero!" << std::endl;
		return false;
	}

	if (plane_corners.size() != door_window_info.size())
	{
		log_info("MakeArrayOriWallJson plane_corners no equal door_window_info size !");
		//std::cout << "MakeArrayOriWallJson plane_corners no equal door_window_info size !" << std::endl;
		return false;
	}

	for (int i=0;i<plane_wall_idx.size();i++)
	{
		int wall_id = plane_wall_idx[i];
		json_t * wall_obj = json_object();
		json_object_set(wall_obj, "id", json_integer(wall_id));
		if (wall_id >= plane_corners.size())
		{
			log_info("MakeArrayOriWallJson wall_id out range of plane_corners size !");
			//std::cout << "MakeArrayOriWallJson wall_id out range of plane_corners size !" << std::endl;
			continue;
		}
		//json_object_set(wall_obj, "type", json_string(wall_type_string[(int)1].c_str()));
		json_object_set(wall_obj, "type", json_string("wall"));
		json_t * vertices = MakeVerticesJson(plane_corners[wall_id]);
		json_object_set(wall_obj, "vertices", vertices);
		json_decref(vertices);

		int door_windows_count = door_window_info[wall_id].size();
		json_object_set(wall_obj, "door_window_count", json_integer(door_windows_count));

		//windows array
		json_t *holes_array = json_array();
		for (int j=0;j<door_window_info[wall_id].size();j++)
		{
			json_t * hole_obj = json_object();
			MeasureDoorWindow::DoorWindowInfo hole = door_window_info[wall_id][j];
			json_object_set(hole_obj, "id", json_integer(j));
			json_object_set(hole_obj, "type", json_string(hole_type_string[(int)hole.type].c_str()));
			//json_object_set(hole_obj, "height", json_integer(hole.height));
			//json_object_set(hole_obj, "width", json_integer(hole.width));
			json_t * vertice_array = MakeVerticesJson(hole.corners);
			json_object_set(hole_obj, "vertices", vertice_array);
			json_array_append(holes_array, hole_obj);

			json_decref(vertice_array);
			json_decref(hole_obj);

		}

		json_object_set(wall_obj, "door_window", holes_array);
		json_decref(holes_array);

		json_array_append(plane_array, wall_obj);
		json_decref(wall_obj);
	}

	return true;
}

bool DecorationHelper::ExportResult(std::string outputJson)
{
	log_info("ExportResult Start");
	if (outputJson.length() < 2)
	{
		outputJson = mJsonFileName;
	}
	json_t* head = json_object();
	if (!head)
	{
		log_info("create json object fail");
		//std::cout << "create json object fail" << std::endl;
		s_ReconstructionErrorCode = RECONSTRUCTION_CREATE_JSON_ERROR;
		return false;
	}

	std::vector<StructuredPlane>  strPlanes = mPointRec->GetStructuredPlanes();

	PlaneCutResultInterface cutPlane = mPointRec->GetPlaneCutResultInterface();

	json_object_set(head, "version", json_string("1.0"));
	json_object_set(head, "wall type description", json_string("beam, wall, ceiling, ground"));
	json_object_set(head, "wall direction", json_string("U(unkown), E(east),S(south), W(west), N(north)"));

	json_object_set(head, "DetectOneMeter", json_boolean(mRoomMeasurement->HasOneMeterLine()));
	json_object_set(head, "square type", json_integer(GetSquareMode()));

	json_object_set(head, "DefaultMinSquaredOffset", json_integer(GetMinSquareOffset()));

#if 1
	json_t *plane_arrayh = json_array();
	json_object_set(head, "ceiling_h_onemeter", plane_arrayh);
	json_decref(plane_arrayh);

	for (int i = 0; i < mRoomMeasurement->ceiling_h_onemeter.size(); i++)
	{
			json_t* plane_object = json_object();
			json_object_set(plane_object, "id", json_integer(mRoomMeasurement->ceiling_h_onemeter[i].first));

			if (mRoomMeasurement->ceiling_h_onemeter[i].second.size() != 2) continue;
			json_object_set(plane_object, "height_min", json_real(mRoomMeasurement->ceiling_h_onemeter[i].second[0].first));
			json_t *center_array = json_array();
			json_array_append(center_array, json_real(mRoomMeasurement->ceiling_h_onemeter[i].second[0].second.x));
			json_array_append(center_array, json_real(mRoomMeasurement->ceiling_h_onemeter[i].second[0].second.y));
			json_array_append(center_array, json_real(mRoomMeasurement->ceiling_h_onemeter[i].second[0].second.z));
			json_object_set(plane_object, "height_min_location", center_array);
			json_decref(center_array);
			if (mRoomMeasurement->ceiling_h_onemeter_int[i].size() != 2) continue;
			json_t * vertice_array = MakeRulerJson(mRoomMeasurement->ceiling_h_onemeter_int[i][0]);
			json_object_set(plane_object, "ruler_vertice_min", vertice_array);
			json_decref(vertice_array);

			json_object_set(plane_object, "height_max", json_real(mRoomMeasurement->ceiling_h_onemeter[i].second[1].first));
			json_t *center_array1 = json_array();
			json_array_append(center_array1, json_real(mRoomMeasurement->ceiling_h_onemeter[i].second[1].second.x));
			json_array_append(center_array1, json_real(mRoomMeasurement->ceiling_h_onemeter[i].second[1].second.y));
			json_array_append(center_array1, json_real(mRoomMeasurement->ceiling_h_onemeter[i].second[1].second.z));
			json_object_set(plane_object, "height_max_location", center_array1);
			json_decref(center_array1);
			json_t * vertice_array1 = MakeRulerJson(mRoomMeasurement->ceiling_h_onemeter_int[i][1]);
			json_object_set(plane_object, "ruler_vertice_max", vertice_array1);
			json_decref(vertice_array1);

			json_array_append(plane_arrayh, plane_object);
			json_decref(plane_object);
	}

	json_t *plane_arrayg = json_array();
	json_object_set(head, "ground_h_onemeter", plane_arrayg);
	json_decref(plane_arrayg);

	for (int i = 0; i < mRoomMeasurement->ground_h_onemeter.size(); i++)
	{
		json_t* plane_object = json_object();
		json_object_set(plane_object, "id", json_integer(mRoomMeasurement->ground_h_onemeter[i].first));

		if (mRoomMeasurement->ground_h_onemeter[i].second.size() != 2) continue;
		json_object_set(plane_object, "height_min", json_real(mRoomMeasurement->ground_h_onemeter[i].second[0].first));
		json_t *center_array = json_array();
		json_array_append(center_array, json_real(mRoomMeasurement->ground_h_onemeter[i].second[0].second.x));
		json_array_append(center_array, json_real(mRoomMeasurement->ground_h_onemeter[i].second[0].second.y));
		json_array_append(center_array, json_real(mRoomMeasurement->ground_h_onemeter[i].second[0].second.z));
		json_object_set(plane_object, "height_min_location", center_array);
		json_decref(center_array);
		if (mRoomMeasurement->ground_h_onemeter_int[i].size() != 2) continue;
		json_t * vertice_array = MakeRulerJson(mRoomMeasurement->ground_h_onemeter_int[i][0]);
		json_object_set(plane_object, "ruler_vertice_min", vertice_array);
		json_decref(vertice_array);

		json_object_set(plane_object, "height_max", json_real(mRoomMeasurement->ground_h_onemeter[i].second[1].first));
		json_t *center_array1 = json_array();
		json_array_append(center_array1, json_real(mRoomMeasurement->ground_h_onemeter[i].second[1].second.x));
		json_array_append(center_array1, json_real(mRoomMeasurement->ground_h_onemeter[i].second[1].second.y));
		json_array_append(center_array1, json_real(mRoomMeasurement->ground_h_onemeter[i].second[1].second.z));
		json_object_set(plane_object, "height_max_location", center_array1);
		json_decref(center_array1);
		json_t * vertice_array1 = MakeRulerJson(mRoomMeasurement->ground_h_onemeter_int[i][1]);
		json_object_set(plane_object, "ruler_vertice_max", vertice_array1);
		json_decref(vertice_array1);

		json_array_append(plane_arrayg, plane_object);
		json_decref(plane_object);
	}
#endif

#if 1
	if (IsRegularizedSquareMode()) {
		json_t* squared_defect_array = json_array();
		for (int w_id = 0; w_id < mRoomMeasurement->squared_defect.size(); w_id++)
		{
			json_t* plane_object = json_object();
			json_object_set(plane_object, "id", json_integer(w_id));
			json_object_set(plane_object, "squared_wall_fill_vol0", json_real(mRoomMeasurement->filling_vol[w_id].first));
			json_object_set(plane_object, "squared_wall_fill_vol1", json_real(mRoomMeasurement->filling_vol[w_id].second));
			json_object_set(plane_object, "squared_wall_fill_vol_opt", json_real(mRoomMeasurement->filling_vol_opt[w_id]));
			json_object_set(plane_object, "defect_max", json_real(mRoomMeasurement->square_defect_max[w_id].first));
			json_object_set(plane_object, "defect_size", json_integer(mRoomMeasurement->squared_defect[w_id].size()));			
#if 1
			json_object_set(plane_object, "defect_opt_size", json_integer(mRoomMeasurement->squared_defect_info[w_id].size()));
			json_object_set(plane_object, "input_wall_width", json_integer(mRoomMeasurement->wall_width_input[w_id].first));
			json_object_set(plane_object, "squared_wall_width", json_integer(mRoomMeasurement->wall_width_info[w_id].first));
			json_object_set(plane_object, "squared_wall_width_opt", json_integer(mRoomMeasurement->wall_width_info[w_id].second));

			json_t *squared_contour_array = json_array();
			for (int c_id = 0; c_id < mRoomMeasurement->squared_defect_info[w_id].size(); c_id++)
			{
				json_t *details_item_array = json_array();
				json_t* details_object = json_object();
				json_object_set(details_object, "mod_def", json_real(mRoomMeasurement->squared_defect_info[w_id][c_id].first));
				JsonSaveOnePointReturnArray(mRoomMeasurement->squared_defect_info[w_id][c_id].second, details_item_array);
				json_object_set(details_object, "point", details_item_array);
				json_array_append(squared_contour_array, details_object);
				json_decref(details_object);
			}
			json_object_set(plane_object, "measurements", squared_contour_array);
#endif

			json_array_append(squared_defect_array, plane_object);
		}
		json_object_set(head, "convexity_squared_defect", squared_defect_array);


	}
#endif




	std::vector<int> g_idxs= mPointRec->GetPlaneList(ePLANE_GROUND);
	log_info("room g_idxs size:%zu", g_idxs.size());
	//cout << "g_idxs:" << g_idxs.size() << endl;
	for (int i = 0; i <g_idxs.size(); i++) {
		json_t* floor_value = json_object();
		if (g_idxs[i] >= 0)
		{
			//cout << "g_idxs[0]:" << g_idxs[i] << endl;
			json_object_set(floor_value, "id", json_integer(g_idxs[i]));
		}
		else
		{
			json_object_set(floor_value, "id", json_null());
		}
		json_t *ground_measurement_array = json_array();
		json_t* ground_levelness_object = json_object();
		json_object_set(ground_levelness_object, "id", json_string("4"));

		if (mRoomMeasurement->leveness.ground_levelness.is_valid)
		{
			json_object_set(ground_levelness_object, "value", json_real(mRoomMeasurement->leveness.ground_levelness.value));
		}
		else
		{
			json_object_set(ground_levelness_object, "value", json_null());
		}
		//5 points and absolute levelness and rulers of ground
		json_t *ground_levelness_details_array = MakeArrayOfDetails(mRoomMeasurement->leveness.ground_levelness_local_ruler_vertice_intersect_pts_dis);
		json_object_set(ground_levelness_object, "details", ground_levelness_details_array);
		json_decref(ground_levelness_details_array);

		json_array_append(ground_measurement_array, ground_levelness_object);
		json_decref(ground_levelness_object);
		log_info("room ground levelness done");
		log_info("measurements floor :");
		//cout << "measurements floor :" << endl;
		json_object_set(floor_value, "measurements", ground_measurement_array);
		json_object_set(head, "floor", floor_value);


		json_t* squared_contour = json_object();
		if (g_idxs[i] >= 0){
			json_object_set(squared_contour, "id", json_integer(g_idxs[i]));
		}else{
			json_object_set(squared_contour, "id", json_null());
		}
		int ref_wall_id = mPointRec->GetReferenceWall();
		json_object_set(squared_contour, "ref_wall_id", json_integer(ref_wall_id));
		//cout << "mRoomMeasurement->rotation_matrix:\n" << mRoomMeasurement->rotation_matrix << endl;
		json_t* mat_array = json_array();
		json_array_append(mat_array, json_real(mRoomMeasurement->rotation_matrix.at<float>(0, 0)));
		json_array_append(mat_array, json_real(mRoomMeasurement->rotation_matrix.at<float>(0, 1)));
		json_array_append(mat_array, json_real(mRoomMeasurement->rotation_matrix.at<float>(0, 2)));
		json_array_append(mat_array, json_real(mRoomMeasurement->rotation_matrix.at<float>(1, 0)));
		json_array_append(mat_array, json_real(mRoomMeasurement->rotation_matrix.at<float>(1, 1)));
		json_array_append(mat_array, json_real(mRoomMeasurement->rotation_matrix.at<float>(1, 2)));
		json_array_append(mat_array, json_real(mRoomMeasurement->rotation_matrix.at<float>(2, 0)));
		json_array_append(mat_array, json_real(mRoomMeasurement->rotation_matrix.at<float>(2, 1)));
		json_array_append(mat_array, json_real(mRoomMeasurement->rotation_matrix.at<float>(2, 2)));
		json_object_set(squared_contour, "rotation_matrix", mat_array);
		json_decref(mat_array);

		json_object_set(squared_contour, "square_by_axis", json_integer(IsSquareByAxis()));

		for (int i = 0; i < mRoomMeasurement->AxisLines.size(); i++)
		{
			if (IsSquareByAxis() && mRoomMeasurement->AxisLines[i].size() == 4)
			{
				json_t* axis_array = json_array();
				json_array_append(axis_array, json_real(mRoomMeasurement->AxisLines[i][0]));
				json_array_append(axis_array, json_real(mRoomMeasurement->AxisLines[i][1]));
				json_array_append(axis_array, json_real(mRoomMeasurement->AxisLines[i][2]));
				json_array_append(axis_array, json_real(mRoomMeasurement->AxisLines[i][3]));
				std::string attribute = "axis_eqn_" + std::to_string(i);
				json_object_set(squared_contour, attribute.c_str(), axis_array);
				json_decref(axis_array);
			}
		}
		json_t* maker_pairs_array = json_array();
		for (int i = 0; i < mRoomMeasurement->markers_pairs_with_pid.size(); i++)
		{
			if (IsSquareByAxis())
			{
				int wallid = mRoomMeasurement->markers_pairs_with_pid[i].first;
				std::pair<cv::Point3f, cv::Point3f> markers_pairs = mRoomMeasurement->markers_pairs_with_pid[i].second;
				json_t* marker_pair_object = json_object();
				json_t* pos_array = json_array();
				json_object_set(marker_pair_object, "wallid", json_integer(wallid));
				json_array_append(pos_array, json_real(mRoomMeasurement->markers_pairs_with_pid[i].second.first.x));
				json_array_append(pos_array, json_real(mRoomMeasurement->markers_pairs_with_pid[i].second.first.y));
				json_array_append(pos_array, json_real(mRoomMeasurement->markers_pairs_with_pid[i].second.first.z));
				json_object_set(marker_pair_object, "ptsPos", pos_array);
				json_t* normal_array = json_array();
				json_array_append(normal_array, json_real(mRoomMeasurement->markers_pairs_with_pid[i].second.second.x));
				json_array_append(normal_array, json_real(mRoomMeasurement->markers_pairs_with_pid[i].second.second.y));
				json_array_append(normal_array, json_real(mRoomMeasurement->markers_pairs_with_pid[i].second.second.z));
				json_object_set(marker_pair_object, "normal", normal_array);
				json_array_append(maker_pairs_array, marker_pair_object);
				log_info("wallid: %d , marker_pos:(%f,%f,%f), normal:(%f,%f,%f)", wallid,
					mRoomMeasurement->markers_pairs_with_pid[i].second.first.x,
					mRoomMeasurement->markers_pairs_with_pid[i].second.first.y,
					mRoomMeasurement->markers_pairs_with_pid[i].second.first.z,
					mRoomMeasurement->markers_pairs_with_pid[i].second.second.x,
					mRoomMeasurement->markers_pairs_with_pid[i].second.second.y,
					mRoomMeasurement->markers_pairs_with_pid[i].second.second.z);
				//cout << wallid << " | " << mRoomMeasurement->markers_pairs[i].first << " | " << mRoomMeasurement->markers_pairs_with_pid[i].second.second << endl;
				json_decref(pos_array);
				json_decref(normal_array);
				json_decref(marker_pair_object);
			}
		}
		json_object_set(squared_contour, "markers", maker_pairs_array);
		json_decref(maker_pairs_array);

		std::vector<int> wall_list = mPointRec->GetWallList();

		int ref_wall_contour_id = -1;
		auto ref_wall_it = std::find(wall_list.begin(), wall_list.end(), ref_wall_id);
		if (ref_wall_it != wall_list.end())
			ref_wall_contour_id = ref_wall_it - wall_list.begin();

		json_t *squared_contour_array = json_array();
		log_info("contour size: %zu", mRoomMeasurement->contour_squared.size());
		//cout << "contour size: " << mRoomMeasurement->contour_squared.size() << endl;
		float squared_contour_min_z = 0.0f;
		if (!mRoomMeasurement->contour_squared.empty())
		{
			squared_contour_min_z = mRoomMeasurement->contour_squared[0].first.z;
			for (int c_id = 1; c_id < mRoomMeasurement->contour_squared.size(); c_id++)
			{
				if (mRoomMeasurement->contour_squared[c_id].first.z < squared_contour_min_z)
					squared_contour_min_z = mRoomMeasurement->contour_squared[c_id].first.z;
			}
		}
		for (int c_id = 0; c_id < mRoomMeasurement->contour_squared.size(); c_id++)
		{
			json_t *details_item_array = json_array();
			json_t* details_object = json_object();
			cv::Point3f contour_point = mRoomMeasurement->contour_squared[c_id].first;
			contour_point.z = squared_contour_min_z;
			JsonSaveOnePointReturnArray(contour_point, details_item_array);
			json_object_set(details_object, "point", details_item_array);
			log_info("ref_wall_contour_id: %d, c_id: %d", ref_wall_contour_id, c_id);
			//std::cout << ref_wall_contour_id << "," << c_id << std::endl;
			if (ref_wall_contour_id != -1 &&
				(ref_wall_contour_id == c_id || (ref_wall_contour_id + 1) % mRoomMeasurement->contour_squared.size() == c_id))
				json_object_set(details_object, "ref_wall", json_true());

			else
				json_object_set(details_object, "ref_wall", json_false());

			json_array_append(squared_contour_array, details_object);
			json_decref(details_object);
		}
		json_object_set(squared_contour, "measurements", squared_contour_array);
		json_object_set(head, "squared_contour", squared_contour);
#if 1
		json_t* squared_contour05 = json_object();
		json_t *squared_contour_array05 = json_array();
		for (int c_id = 0; c_id < mRoomMeasurement->contour_squared05.size(); c_id++)
		{
			json_t *details_item_array = json_array();
			json_t* details_object = json_object();
			JsonSaveOnePointReturnArray(mRoomMeasurement->contour_squared05[c_id], details_item_array);
			json_object_set(details_object, "point", details_item_array);
			json_array_append(squared_contour_array05, details_object);
			json_decref(details_object);
		}
		json_object_set(squared_contour05, "measurements", squared_contour_array05);
		json_object_set(head, "squared_contour05", squared_contour05);

		json_t* squared_contour_array05_len = json_array();
		for (auto len : mRoomMeasurement->contour_squared05_len)
			json_array_append(squared_contour_array05_len, json_real(len));
		json_object_set(head, "squared_contour05_len", squared_contour_array05_len);
		json_decref(squared_contour_array05_len);
#endif

	}

	json_t* wall_pair_array = json_array();
	std::vector<int> wall_idx = mPointRec->GetPlaneList(ePLANE_WALL);
	for (auto id : wall_idx)
		json_array_append(wall_pair_array, json_integer(id));
	json_object_set(head, "wall_idx", wall_pair_array);
	json_decref(wall_pair_array);


	json_t* wall_pair_array1 = json_array();
	std::vector<int> wall_idx1 = mPointRec->GetWallList();
	for (auto id : wall_idx1)
		json_array_append(wall_pair_array1, json_integer(id));
	json_object_set(head, "squared_wall_idx", wall_pair_array1);
	json_decref(wall_pair_array1);

#if 1
	json_t* wall_pair_array2 = json_array();
	std::vector<int> wall_idx2 = mPointRec->GetWallList();

	//std::cout << "wall_idx2.size(): " << wall_idx2.size() << endl;
	for (int idx=0; idx< wall_idx2.size(); idx++)
	{
		json_t* as_item_object0 = json_object();
		json_t* wall_pair_array0 = json_array();
		//std::cout << "wall_idx2.: " << wall_idx2[idx]<<" , " << wall_idx2[(idx + 1) % wall_idx2.size()] << endl;
		json_array_append(wall_pair_array0, json_integer(wall_idx2[idx]));
		json_array_append(wall_pair_array0, json_integer(wall_idx2[(idx+1)% wall_idx2.size()]));
		json_object_set(as_item_object0, "walls", wall_pair_array0);
		json_decref(wall_pair_array0);
#if 1
		if (mRoomMeasurement->walls_squareness.size() == wall_idx2.size())
		{
			json_t* walls_squareness_array = json_array();
			//std::cout << mRoomMeasurement->walls_squareness[idx].first <<" , "<< mRoomMeasurement->walls_squareness[idx].second << endl;
			json_array_append(walls_squareness_array, json_real(mRoomMeasurement->walls_squareness[idx].first));
			json_array_append(walls_squareness_array, json_real(mRoomMeasurement->walls_squareness[idx].second));
			json_object_set(as_item_object0, "squareness", walls_squareness_array);
			json_decref(walls_squareness_array);
		}
#endif
		json_array_append(wall_pair_array2, as_item_object0);
		json_decref(as_item_object0);
	}
	json_object_set(head, "walls_squreness", wall_pair_array2);
	json_decref(wall_pair_array2);
#endif

	json_t* ceiling_pair_array = json_array();
	json_t* ceiling_height_array = json_array();
	std::vector<int> ceiling_idx = mPointRec->GetPlaneList(ePLANE_CEILING);

	bool max_height_calculated = false;
	float global_max_height = 1500.0f;

	for (auto id : ceiling_idx) {
		json_array_append(ceiling_pair_array, json_integer(id));

		if (!max_height_calculated) {
			for (size_t i = 0; i < strPlanes.size(); i++) {
				if (i == id) {
					std::vector<float> heights;
					for (auto pt : strPlanes[i].vertices) {
						heights.push_back(pt.z);
					}
					global_max_height = *std::min_element(heights.begin(), heights.end());
					max_height_calculated = true;
					break; 
				}
			}
		}
	}
	json_array_append(ceiling_height_array, json_real(global_max_height));
		
	json_object_set(head, "ceiling_idx", ceiling_pair_array);
	json_object_set(head, "ceiling_height", ceiling_height_array);
	json_decref(ceiling_pair_array);
	json_decref(ceiling_height_array);

	json_t* ground_pair_array = json_array();
	json_t* ground_height_array = json_array();
	std::vector<int> ground_idx = mPointRec->GetPlaneList(ePLANE_GROUND);
	bool min_height_calculated = false;
	float global_min_height = -1000.0f;
	for (auto id : ground_idx) {
		json_array_append(ground_pair_array, json_integer(id));

		if (!min_height_calculated) {
			for (size_t i = 0; i < strPlanes.size(); i++) {
				if (i == id) {
					std::vector<float> heights;
					if (!strPlanes[i].vertices.empty()) {
						global_min_height = strPlanes[i].vertices[0].z;
						max_height_calculated = true;
					}
					break;
				}
			}
		}
	}
	json_array_append(ground_height_array, json_real(global_min_height));
	json_object_set(head, "ground_idx", ground_pair_array);
	json_object_set(head, "ground_height", ground_height_array);
	json_decref(ground_pair_array);
	json_decref(ground_height_array);

	json_t* gap_root = json_object();

	json_object_set(gap_root, "ceil_min_z",
		MakeGapPairJson(mPointRec->ceilMinPair));

	json_object_set(gap_root, "ceil_max_z",
		MakeGapPairJson(mPointRec->ceilMaxPair));

	json_object_set(gap_root, "floor_min_z",
		MakeGapPairJson(mPointRec->floorMinPair));

	json_object_set(gap_root, "floor_max_z",
		MakeGapPairJson(mPointRec->floorMaxPair));

	json_object_set(gap_root, "min_gap",
		MakeGapPairJson(mPointRec->minGapPair));

	json_object_set(gap_root, "max_gap",
		MakeGapPairJson(mPointRec->maxGapPair));

	json_object_set(head, "ceil_floor_gap_pairs", gap_root);
	json_decref(gap_root);

	json_t *plane_array = json_array();
	json_object_set(head, "plane_info", plane_array);
	json_decref(plane_array);

	for(int i=0; i < strPlanes.size(); i++)
	{
		if (strPlanes[i].vertices.size() > 0)
		{
			json_t* plane_object = json_object();
			json_object_set(plane_object, "id", json_integer(i));

		json_object_set(plane_object, "type", json_string(wall_type_string[(int)strPlanes[i].type].c_str()));

		json_t *normal_array = json_array();
		json_array_append(normal_array, json_real(strPlanes[i].normal.x));
		json_array_append(normal_array, json_real(strPlanes[i].normal.y));
		json_array_append(normal_array, json_real(strPlanes[i].normal.z));
		json_object_set(plane_object, "normal", normal_array);
		json_decref(normal_array);

		json_t *center_array = json_array();
		json_array_append(center_array, json_real(strPlanes[i].center.x));
		json_array_append(center_array, json_real(strPlanes[i].center.y));
		json_array_append(center_array, json_real(strPlanes[i].center.z));
		json_object_set(plane_object, "center", center_array);
		json_decref(center_array);

		json_object_set(plane_object, "direction", json_string(wall_dire_string[(int)strPlanes[i].direction].c_str()));

		//wall height and width, to be compute
		json_object_set(plane_object, "height", json_integer(strPlanes[i].wall_height));
		json_object_set(plane_object, "width", json_integer(strPlanes[i].wall_width));


		json_t * vertices = MakeVerticesJson(strPlanes[i].vertices);
		json_object_set(plane_object, "vertices", vertices);
		json_decref(vertices);


		int door_windows_count = strPlanes[i].holes.size();
		json_object_set(plane_object, "door_window_count", json_integer(door_windows_count));

		//windows array
		json_t *holes_array = json_array();
		for (auto hole : strPlanes[i].holes)
		{
			json_t * hole_obj = json_object();
			
			json_object_set(hole_obj, "id", json_integer(hole.id));
			json_object_set(hole_obj, "lid", json_integer(hole.local_id));
			json_object_set(hole_obj, "type", json_string(hole_type_string[(int)hole.type].c_str()));
			json_object_set(hole_obj, "height", json_integer(hole.height));
			json_object_set(hole_obj, "width", json_integer(hole.width));

			json_t * vertice_array = MakeVerticesJson(hole.vertice);
			json_object_set(hole_obj, "vertices", vertice_array);
			json_array_append(holes_array, hole_obj);
			json_decref(vertice_array);
		}
		json_object_set(plane_object, "door_window", holes_array);
		json_decref(holes_array);

			json_array_append(plane_array, plane_object);
			json_decref(plane_object);
		}
	}
	
	json_t *plane_array1 = json_array();
	json_object_set(head, "squared_plane_info", plane_array1);
	json_decref(plane_array1);
	strPlanes.clear();
	strPlanes = mPointRec->GetStructuredPlanesSquared();
	for (int i = 0; i < strPlanes.size(); i++)
	{
		if (strPlanes[i].vertices.size() > 0)
		{
			json_t* plane_object = json_object();
			json_object_set(plane_object, "id", json_integer(i));

		json_object_set(plane_object, "type", json_string(wall_type_string[(int)strPlanes[i].type].c_str()));

		json_t *normal_array = json_array();
		json_array_append(normal_array, json_real(strPlanes[i].normal.x));
		json_array_append(normal_array, json_real(strPlanes[i].normal.y));
		json_array_append(normal_array, json_real(strPlanes[i].normal.z));
		json_object_set(plane_object, "normal", normal_array);
		json_decref(normal_array);

		json_t *center_array = json_array();
		json_array_append(center_array, json_real(strPlanes[i].center.x));
		json_array_append(center_array, json_real(strPlanes[i].center.y));
		json_array_append(center_array, json_real(strPlanes[i].center.z));
		json_object_set(plane_object, "center", center_array);
		json_decref(center_array);

		json_t *wall_conv_array = json_array();
		json_array_append(wall_conv_array, json_real(strPlanes[i].wallConvPt.x));
		json_array_append(wall_conv_array, json_real(strPlanes[i].wallConvPt.y));
		json_array_append(wall_conv_array, json_real(strPlanes[i].wallConvPt.z));
		json_object_set(plane_object, "wall_conv_pt", wall_conv_array);
		json_decref(wall_conv_array);

		json_object_set(plane_object, "direction", json_string(wall_dire_string[(int)strPlanes[i].direction].c_str()));

		//wall height and width, to be compute
		json_object_set(plane_object, "height", json_integer(strPlanes[i].wall_height));
		json_object_set(plane_object, "width", json_integer(strPlanes[i].wall_width));

			//json_object_set(plane_object, "mesh", json_string(GetStructedMeshName(i).c_str()));

			//to be fill
			//json_object_set(plane_object, "reflectmap", json_string(GetReflectImageFileName(mStructedMeshDir, i).c_str()));
			//json_object_set(plane_object, "defectmap", json_string(GetDefectImageFileName(mStructedMeshDir, i).c_str()));

		if ((GetSquareMode() == SQUARE_BY_CONVEXITY)&&(strPlanes[i].convPtF)) {
			json_t *conv_array = json_array();
			json_array_append(conv_array, json_real(strPlanes[i].convPt.x));
			json_array_append(conv_array, json_real(strPlanes[i].convPt.y));
			json_array_append(conv_array, json_real(strPlanes[i].convPt.z));
			json_object_set(plane_object, "conv_pt", conv_array);
			json_decref(conv_array);
		}
		json_t * vertices = MakeVerticesJson(strPlanes[i].vertices);
		json_object_set(plane_object, "vertices", vertices);
		json_decref(vertices);

		int door_windows_count = strPlanes[i].holes.size();
		json_object_set(plane_object, "door_window_count", json_integer(door_windows_count));

		//windows array
		json_t *holes_array = json_array();
		for (auto hole : strPlanes[i].holes)
		{
			json_t * hole_obj = json_object();

			json_object_set(hole_obj, "id", json_integer(hole.id));
			json_object_set(hole_obj, "type", json_string(hole_type_string[(int)hole.type].c_str()));
			json_object_set(hole_obj, "height", json_integer(hole.height));
			json_object_set(hole_obj, "width", json_integer(hole.width));

			json_t * vertice_array = MakeVerticesJson(hole.vertice);
			json_object_set(hole_obj, "vertices", vertice_array);
			json_array_append(holes_array, hole_obj);
			json_decref(vertice_array);
		}
		json_object_set(plane_object, "door_window", holes_array);
		json_decref(holes_array);

			json_array_append(plane_array1, plane_object);
			json_decref(plane_object);
		}
	}
	json_t *plane_array2 = json_array();
	json_object_set(head, "min_squared_plane_info", plane_array2);
	json_decref(plane_array2);
	strPlanes.clear();
	strPlanes = mPointRec->GetStructuredPlanesSquaredMin();
	for (int i = 0; i < strPlanes.size(); i++)
	{
		if (strPlanes[i].vertices.size() > 0) 
		{
			json_t* plane_object = json_object();
			json_object_set(plane_object, "id", json_integer(i));

		json_object_set(plane_object, "type", json_string(wall_type_string[(int)strPlanes[i].type].c_str()));

		json_t *normal_array = json_array();
		json_array_append(normal_array, json_real(strPlanes[i].normal.x));
		json_array_append(normal_array, json_real(strPlanes[i].normal.y));
		json_array_append(normal_array, json_real(strPlanes[i].normal.z));
		json_object_set(plane_object, "normal", normal_array);
		json_decref(normal_array);

		json_t *center_array = json_array();
		json_array_append(center_array, json_real(strPlanes[i].center.x));
		json_array_append(center_array, json_real(strPlanes[i].center.y));
		json_array_append(center_array, json_real(strPlanes[i].center.z));
		json_object_set(plane_object, "center", center_array);
		json_decref(center_array);

		json_object_set(plane_object, "direction", json_string(wall_dire_string[(int)strPlanes[i].direction].c_str()));

		//wall height and width, to be compute
		json_object_set(plane_object, "height", json_integer(strPlanes[i].wall_height));
		json_object_set(plane_object, "width", json_integer(strPlanes[i].wall_width));

		json_t * vertices = MakeVerticesJson(strPlanes[i].vertices);
		json_object_set(plane_object, "vertices", vertices);
		json_decref(vertices);

		int door_windows_count = strPlanes[i].holes.size();
		json_object_set(plane_object, "door_window_count", json_integer(door_windows_count));

		//windows array
		json_t *holes_array = json_array();
		for (auto hole : strPlanes[i].holes)
		{
			json_t * hole_obj = json_object();

			json_object_set(hole_obj, "id", json_integer(hole.id));
			json_object_set(hole_obj, "type", json_string(hole_type_string[(int)hole.type].c_str()));
			json_object_set(hole_obj, "height", json_integer(hole.height));
			json_object_set(hole_obj, "width", json_integer(hole.width));

			json_t * vertice_array = MakeVerticesJson(hole.vertice);
			json_object_set(hole_obj, "vertices", vertice_array);
			json_array_append(holes_array, hole_obj);
			json_decref(vertice_array);
		}
		json_object_set(plane_object, "door_window", holes_array);
		json_decref(holes_array);

			json_array_append(plane_array2, plane_object);
			json_decref(plane_object);
		}
	}
	json_t* plane_array3 = json_array();
	json_object_set(head, "min05_squared_plane_info", plane_array3);
	json_decref(plane_array3);
	strPlanes.clear();
	strPlanes = mPointRec->GetStructuredPlanesSquaredMin05();
	for (int i = 0; i < strPlanes.size(); i++)
	{
		if (strPlanes[i].vertices.size() > 0)
		{
			json_t* plane_object = json_object();
			json_object_set(plane_object, "id", json_integer(i));

			json_object_set(plane_object, "type", json_string(wall_type_string[(int)strPlanes[i].type].c_str()));

			json_t* normal_array = json_array();
			json_array_append(normal_array, json_real(strPlanes[i].normal.x));
			json_array_append(normal_array, json_real(strPlanes[i].normal.y));
			json_array_append(normal_array, json_real(strPlanes[i].normal.z));
			json_object_set(plane_object, "normal", normal_array);
			json_decref(normal_array);

			json_t* center_array = json_array();
			json_array_append(center_array, json_real(strPlanes[i].center.x));
			json_array_append(center_array, json_real(strPlanes[i].center.y));
			json_array_append(center_array, json_real(strPlanes[i].center.z));
			json_object_set(plane_object, "center", center_array);
			json_decref(center_array);

			json_object_set(plane_object, "direction", json_string(wall_dire_string[(int)strPlanes[i].direction].c_str()));

			//wall height and width, to be compute
			json_object_set(plane_object, "height", json_integer(strPlanes[i].wall_height));
			json_object_set(plane_object, "width", json_integer(strPlanes[i].wall_width));

			json_t* vertices = MakeVerticesJson(strPlanes[i].vertices);
			json_object_set(plane_object, "vertices", vertices);
			json_decref(vertices);

			int door_windows_count = strPlanes[i].holes.size();
			json_object_set(plane_object, "door_window_count", json_integer(door_windows_count));

			//windows array
			json_t* holes_array = json_array();
			for (auto hole : strPlanes[i].holes)
			{
				json_t* hole_obj = json_object();

				json_object_set(hole_obj, "id", json_integer(hole.id));
				json_object_set(hole_obj, "type", json_string(hole_type_string[(int)hole.type].c_str()));
				json_object_set(hole_obj, "height", json_integer(hole.height));
				json_object_set(hole_obj, "width", json_integer(hole.width));

				json_t* vertice_array = MakeVerticesJson(hole.vertice);
				json_object_set(hole_obj, "vertices", vertice_array);
				json_array_append(holes_array, hole_obj);
				json_decref(vertice_array);
			}
			json_object_set(plane_object, "door_window", holes_array);
			json_decref(holes_array);

			json_array_append(plane_array3, plane_object);
			json_decref(plane_object);
		}
	}

	//============wall_ori_array start============//
	json_t* wall_ori_array = json_array();
	bool isOriwall = MakeArrayOriWallJson(cutPlane, wall_ori_array);
	if (isOriwall)
	{
		json_object_set(head, "wall_ori_info", wall_ori_array);
		log_info("wall_ori_info done!");
	}
	else
	{
		json_object_set(head, "wall_ori_info", wall_ori_array);
		log_info("wall_ori_info false!");
	}
	json_decref(wall_ori_array);
	//============wall_ori_array end============//

	int ret = -1;
	FILE* fp = fopen(outputJson.c_str(), "w");
	if ((ret = json_dumpf(head, fp, JSON_REAL_PRECISION(6))) != 0)
	{
		log_error("json_dump_file failed");
	}
	fclose(fp);

	json_decref(head);
	log_info("ExportResult End");
	return true;
}

bool DecorationHelper::StructureReconstruction_auto(void)
{
	return mPointRec->StructureReconstructionByBottomCeil();
}

bool DecorationHelper::StructureReconstruction(int planeid)
{

	std::vector<int> ceiling_idx = mPointRec->GetPlanes().plane_ceiling_idx;
	if (std::find(ceiling_idx.begin(), ceiling_idx.end(), planeid) != ceiling_idx.end())
	{
		return mPointRec->StructureReconstruction(mPointRec->GetPlanes().plane_center[planeid].z);
	}
	else
		return mPointRec->StructureReconstructionByBottomCeil();
}

bool DecorationHelper::StructureReconstruction(float fake_ceiling_z)
{
	if (fake_ceiling_z < 1E-7)
		return mPointRec->StructureReconstruction();
	else
		return mPointRec->StructureReconstruction(fake_ceiling_z);
}


void DecorationHelper::ExportStructureMeshesSquareMin(std::string mesh_dir)
{
	//mRoomMeasurement->contour_squared = mRoomMeasurement->MakeRoomSquare((eRoomSquare_type)room_squareness_mode);
	if (mesh_dir.length() > 2)
	{		
		mSquaredMeshDirMin = mesh_dir;
		Appendsufix(mSquaredMeshDirMin);
	}
	system(("rmdir /q /s " + mSquaredMeshDirMin).c_str());
	IOData::createDirectory(mSquaredMeshDirMin);
	std::vector<StructuredPlane> squaredPlanesMin = mPointRec->GetStructuredPlanesSquaredMin();
	
	int nPtOutPoly = 0;
	for (int i = 0; i < squaredPlanesMin.size(); i++)
	{
		std::vector<cv::Point2f> wallPolygon_2;
		wallPolygon_2.clear();
		for (int corner_index = 0; corner_index < squaredPlanesMin[i].vertices.size(); corner_index++)
		{
			wallPolygon_2.emplace_back(cv::Point2f(squaredPlanesMin[i].vertices[corner_index].x, squaredPlanesMin[i].vertices[corner_index].z));
		}

		if (squaredPlanesMin[i].holes.size() > 0)
		{
			bool PtInPoly;
			for (int j = 0; j < squaredPlanesMin[i].holes.size(); j++)
			{
				for (int k = 0; k < squaredPlanesMin[i].holes[j].vertice.size(); k++)
				{
					int nPlaneCorners = squaredPlanesMin[i].vertices.size();
					//TOFIX: door vertices is not accurate in Z axis, error tolerance here, but need to figure out why door vertices inaccuracy
					float temp_z;
					if (squaredPlanesMin[i].holes[j].vertice[k].z > 0)
						temp_z = squaredPlanesMin[i].holes[j].vertice[k].z - 50;
					else if (squaredPlanesMin[i].holes[j].vertice[k].z < 0)
						temp_z = squaredPlanesMin[i].holes[j].vertice[k].z + 50;
					PtInPoly = PtInPolygon(cv::Point2f(squaredPlanesMin[i].holes[j].vertice[k].x, temp_z), wallPolygon_2, nPlaneCorners);
					if (PtInPoly == 0)
					{
						nPtOutPoly += 1;
					}
				}
			}
		}
	}

	if (nPtOutPoly > 0)
	{
		log_error(" There are cases in the squaredPlanesMin where the apex of the gate appears outside the wall! ");
		//cout << "##" << " There are cases in the squaredPlanesMin where the apex of the gate appears outside the wall! " << endl;
	}
	else
	{
		mRoomMeasurement->constructMeshByContour_squared(mRoomMeasurement->contour_squared1, squaredPlanesMin, mSquaredMeshDirMin + "meshMin");
	}
}

void DecorationHelper::ExportStructureMeshesSquareMin05(std::string mesh_dir)
{
	//mRoomMeasurement->contour_squared = mRoomMeasurement->MakeRoomSquare((eRoomSquare_type)room_squareness_mode);

	if (mesh_dir.length() > 2)
	{
		mSquaredMeshDirMin05 = mesh_dir;
		Appendsufix(mSquaredMeshDirMin05);
	}
	system(("rmdir /q /s " + mSquaredMeshDirMin05).c_str());
	IOData::createDirectory(mSquaredMeshDirMin05);
	std::vector<StructuredPlane> squaredPlanesMin05 = mPointRec->GetStructuredPlanesSquaredMin05();

	int nPtOutPoly = 0;
	for (int i = 0; i < squaredPlanesMin05.size(); i++)
	{
		std::vector<cv::Point2f> wallPolygon_2;
		wallPolygon_2.clear();
		for (int corner_index = 0; corner_index < squaredPlanesMin05[i].vertices.size(); corner_index++)
		{
			wallPolygon_2.emplace_back(cv::Point2f(squaredPlanesMin05[i].vertices[corner_index].x, squaredPlanesMin05[i].vertices[corner_index].z));
		}

		if (squaredPlanesMin05[i].holes.size() > 0)
		{
			bool PtInPoly;
			for (int j = 0; j < squaredPlanesMin05[i].holes.size(); j++)
			{
				for (int k = 0; k < squaredPlanesMin05[i].holes[j].vertice.size(); k++)
				{
					int nPlaneCorners = squaredPlanesMin05[i].vertices.size();
					//TOFIX: door vertices is not accurate in Z axis, error tolerance here, but need to figure out why door vertices inaccuracy
					float temp_z;
					if (squaredPlanesMin05[i].holes[j].vertice[k].z > 0)
						temp_z = squaredPlanesMin05[i].holes[j].vertice[k].z - 50;
					else if (squaredPlanesMin05[i].holes[j].vertice[k].z < 0)
						temp_z = squaredPlanesMin05[i].holes[j].vertice[k].z + 50;
					PtInPoly = PtInPolygon(cv::Point2f(squaredPlanesMin05[i].holes[j].vertice[k].x, temp_z), wallPolygon_2, nPlaneCorners);
					//std::cout << cv::Point2f(squaredPlanesMin05[i].holes[j].vertice[k].x, squaredPlanesMin05[i].holes[j].vertice[k].z) << std::endl;
					//std::cout << wallPolygon_2 << std::endl;
					//std::cout << "PtInPoly = " << PtInPoly << std::endl;
					if (PtInPoly == 0)
					{
						nPtOutPoly += 1;
					}
				}
			}
		}
	}

	if (nPtOutPoly > 0)
	{
		log_error(" There are cases in the squaredPlanesMin05 where the apex of the gate appears outside the wall! ");
		//cout << "##" << " There are cases in the squaredPlanesMin05 where the apex of the gate appears outside the wall! " << endl;
	}
	else
	{
		mRoomMeasurement->constructMeshByContour_squared(mRoomMeasurement->contour_squared05, squaredPlanesMin05, mSquaredMeshDirMin05 + "meshMin");
	}

	//mRoomMeasurement->constructMeshByContour_squared(mRoomMeasurement->contour_squared05, squaredPlanesMin05, mSquaredMeshDirMin05 + "meshMin");
}


std::vector<int> DecorationHelper::GetOriginCeilingIdx()
{
	std::vector<int> idx = mPointRec->GetPlanes().plane_ceiling_idx;
	return idx;
}

int DecorationHelper::GetOriginBottomCeilingId()
{
	std::vector<int> ceiling_idx = mPointRec->GetPlanes().plane_ceiling_idx;
	if (ceiling_idx.size() == 0)
		return -1;

	int bottomceiling = ceiling_idx[0];
	for (auto id : ceiling_idx)
	{
		if (mPointRec->GetPlanes().plane_center[bottomceiling].z > mPointRec->GetPlanes().plane_center[id].z)
			bottomceiling = id;
	}
	return bottomceiling;
}


static cv::Point3f RotatePoints(cv::Point3f ori, cv::Mat rot_mat)
{
	cv::Point3f  pt= ori;
	pt.x = ori.x / 1000.0f;
	pt.y = ori.y / 1000.0f;
	pt.z = ori.z / 1000.0f;

	cv::Point3f c_pt = cv::Point3f(
		rot_mat.at<float>(0, 0) * pt.x + rot_mat.at<float>(0, 1) * pt.y + rot_mat.at<float>(0, 2) * pt.z,
		rot_mat.at<float>(1, 0) * pt.x + rot_mat.at<float>(1, 1) * pt.y + rot_mat.at<float>(1, 2) * pt.z,
		rot_mat.at<float>(2, 0) * pt.x + rot_mat.at<float>(2, 1) * pt.y + rot_mat.at<float>(2, 2) * pt.z);

	c_pt.x += rot_mat.at<float>(0, 3);
	c_pt.y += rot_mat.at<float>(1, 3);
	c_pt.z += rot_mat.at<float>(2, 3);

	pt.x = c_pt.x * 1000.0f;
	pt.y = c_pt.y * 1000.0f;
	pt.z = c_pt.z * 1000.0f;

	return pt;
}

static void RotatePoints(std::vector<cv::Point3f> &ori, cv::Mat rot_mat)
{
	for (int i = 0; i < ori.size(); i++)
	{
		ori[i] = RotatePoints(ori[i], rot_mat);
	}
}


bool PushVericeSquared2PlaneVerice(const std::vector<int>& wallColckWiseList,
	const std::vector<cv::Point3f>& floor_contour_squared,
	std::vector<StructuredPlane>& strPlaness)
{
	
	if (wallColckWiseList.size()==0 || strPlaness.size()==0)
	{
		return false;
	}
	if (wallColckWiseList.size()!= floor_contour_squared.size())
	{
		return false;
	}

	for (int i = 0; i < strPlaness.size(); i++)
	{
		strPlaness[i].vertices_s = strPlaness[i].vertices;
	}


	int floor_size = floor_contour_squared.size();
	for (int i = 0; i < floor_size; i++)
	{
		int wall_id = wallColckWiseList[i];
		if (wall_id >= strPlaness.size())
		{
			log_error("wall_id out range of arrary!");
			//cout << "wall_id out range of arrary!" << std::endl;
			return false;
		}
		//std::cout << "i:" << i << " wall_id:" << wall_id << std::endl;
		int idx_start = i;
		int idx_end = i + 1;
		if (i == (floor_size - 1))
		{
			idx_end = 0;
		}
		cv::Point3f pt_start = floor_contour_squared[idx_start];
		cv::Point3f pt_end = floor_contour_squared[idx_end];

		if (strPlaness[wall_id].vertices.size() != 4)
		{
			log_info("vertices.size()!=4!");
			//cout << "vertices.size()!=4!" << std::endl;
			continue;
		}
		strPlaness[wall_id].vertices_s[0].x = pt_start.x;
		strPlaness[wall_id].vertices_s[0].y = pt_start.y;

		strPlaness[wall_id].vertices_s[3].x = pt_start.x;
		strPlaness[wall_id].vertices_s[3].y = pt_start.y;

		strPlaness[wall_id].vertices_s[1].x = pt_end.x;
		strPlaness[wall_id].vertices_s[1].y = pt_end.y;

		strPlaness[wall_id].vertices_s[2].x = pt_end.x;
		strPlaness[wall_id].vertices_s[2].y = pt_end.y;

	}
	return true;
}


bool DecorationHelper::IsEqualEechotherNum(const std::vector<std::vector<std::vector<cv::Point3f>>>&hole_vertice_squared, std::vector<StructuredPlane>& strPlanes)
{
	if (hole_vertice_squared.size() == 0 || strPlanes.size() == 0)
	{
		log_info(" plane or hole num is zero!");
		//std::cout << " plane or hole num is zero!" << std::endl;
		return false;
	}

	if (hole_vertice_squared.size() != strPlanes.size())
	{
		log_info(" plane num no equal!");
		//std::cout << "plane num no equal!" << std::endl;
		return false;
	}

	for (int i = 0; i<strPlanes.size(); i++)
	{
		if (strPlanes[i].holes.size() != hole_vertice_squared[i].size())
		{
			log_info(" hole num no equal!");
			//std::cout << " hole num no equal!" << std::endl;
			return false;
		}
		for (int j = 0; j<strPlanes[i].holes.size(); j++)
		{
			if (strPlanes[i].holes[j].vertice.size() != hole_vertice_squared[i][j].size())
			{
				log_info(" hole vertice num no equal!");
				//std::cout << " hole vertice num no equal!" << std::endl;
				return false;
			}
		}
	}
	return true;
}

void DecorationHelper::PushHoleSquared2PlaneHole(const std::vector<std::vector<std::vector<cv::Point3f>>>&hole_vertice_squared, std::vector<StructuredPlane>& strPlanes)
{
	if (!IsEqualEechotherNum(hole_vertice_squared, strPlanes))
	{
		return;
	}

	for (int i = 0; i<strPlanes.size(); i++)
	{
		for (int j = 0; j<strPlanes[i].holes.size(); j++)
		{
			strPlanes[i].holes[j].vertice_s = hole_vertice_squared[i][j];
		}
	}
}


void DecorationHelper::SetFirstId(int firstid)
{
	mFirstId = firstid;
}
int  DecorationHelper::GetFirstId(void)
{
	return mFirstId;
}


void DecorationHelper::CreatingMesh_(std::string markerFile, float ratio, std::string output_path)
{
	CreatingMesh creatingMesh;
	creatingMesh.buildMesh(markerFile, ratio, output_path);
}
