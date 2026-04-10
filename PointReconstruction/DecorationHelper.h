#pragma once
#include <string>
#include "PointReconstruction.h"
#include "RoomMeasurement.h"
#include "jansson.h"
#include "MultiStage\StageMatrix.h"
#include "../Common/StructTypeInfo.h"

class DecorationHelper
{
public:
	//seg_mode: 0 wall base, 1 wall less, 2 beam base.
	DecorationHelper(std::string  workDir, int seg_mode);

	DecorationHelper(const DecorationHelper&) = delete;
	DecorationHelper& operator=(const DecorationHelper&) = delete;

	~DecorationHelper();

	json_t* MakeGapPairJson(const GapPairResult& r);
	bool UpdateDoorWindowInfo(std::string updateJson);

	bool PlaneSegmentation(std::vector<std::string> file_paths, std::vector<cv::Mat> RTs, cv::Mat relaRT,std::vector<std::vector<float>> station_pos, float mergeDataDir,
		                   std::string compass_file, float compass_value=0, bool generate_contour = false);

	void MarkerBallDetection(std::string sense_path, std::string output_path);

	bool StructureReconstruction_auto(void); // 自动选天花板
	bool StructureReconstruction(int planeid = -1); // 手动选取天花板（废弃）
	bool StructureReconstruction(float fake_ceiling_z = 1E-10); // 手动指定高度

	void ExportStructureMeshesSquareMin(std::string mesh_dir = "");
	void ExportStructureMeshesSquareMin05(std::string mesh_dir = "");

	bool IsMultiCeiling(void); // 是否检测到多天花板，可以用于手动选取判定

	bool StartMeasurement(int room_squareness_mode, std::string axisEqnConfig);
	void RedoRommSquare(eRoomSquare_type type = SQUARE_BY_ROOMCONTOUR);
	bool ExportResult(std::string outputJson = "");

	void SetJsonFileName(std::string file_name);
	std::string GetJsonFileName(void);

	std::vector<int> GetOriginCeilingIdx();
	int GetOriginBottomCeilingId();

	void SetStageMatrix(StageMatrix         *pstageMatrix);

	StageMatrix *GetStageMatrix();

	void SetFirstId(int firstid);
	int  GetFirstId(void);

	void CreatingMesh_(std::string markersFile, float ratio, std::string output_path);

	//void GetPolygon(const std::vector<cv::Point2f>& polygon_points);
	//void pixel2Point3(const cv::Point2f anchor, float dir);

private:
	std::string GetOriMeshDir(void);
	std::string GetOriMeshName(int wallId);
	std::string GetStructedMeshDir(void);
	std::string GetStructedMeshName(int wallId);

	json_t * MakeVerticesJson(std::vector<cv::Point3f> vertices);
	json_t * MakeRulerJson(std::pair<cv::Point3f, cv::Point3f> ruler);
	bool MakeArrayOriWallJson(const PlaneCutResultInterface& cutPlane, json_t* plane_array);

	std::vector<cv::Point3f> JsonLoadVertices(json_t* vertices_array);
	int  GetMatchId(std::vector<std::string>  string_arra, std::string value);

	bool PtInPolygon(const cv::Point2f& triCentroid, const std::vector<cv::Point2f>& contour_squared_floor, int &nCount);

	void Appendsufix(std::string &filedir);

	//added by hgj
	bool IsEqualEechotherNum(const std::vector<std::vector<std::vector<cv::Point3f>>>&hole_vertice_squared, std::vector<StructuredPlane>& strPlanes);
	//added by hgj
	void PushHoleSquared2PlaneHole(const std::vector<std::vector<std::vector<cv::Point3f>>>&hole_vertice_squared, std::vector<StructuredPlane>& strPlanes);

	void SaveRoomSquareAsImage(std::string dir);
	void SaveHolesSquareAsImage(std::string dir);

private:
	std::string  mWorkDir;
	int			 mSegMode;

	std::string  mJsonFileName;
	std::string  mOriMeshDir;
	std::string  mWorldOriMeshDir;
	std::string  mStructedMeshDir;
	std::string  mSquaredMeshDir;
	std::string  mSquaredMeshDirMin;
	std::string  mSquaredMeshDirMin05;
	std::string  mSquarenessDir;
	std::string  mWorldStructedMeshDir;

	//std::vector<cv::Point2f> polygon_points; 暂时停用


	PointReconstruction *mPointRec;
	RoomMeasurement     *mRoomMeasurement;
	StageMatrix         *mpStageMatrix;
	int         mFirstId;
};

