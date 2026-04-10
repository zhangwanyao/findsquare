#pragma once
#include "PlaneCuttingInterface.h"
#include "util_log.hpp"
#include <string>
#include <unordered_map>
#include "Reconstruction\Reconstructer.h"
#include <opencv2\opencv.hpp>
#include "Common\PlaneStructType.h"
#include "Measurement/MeasureBase.h"
#include "PointReconstructionGetConfig.h"

typedef struct PlaneCutResultInterface RoomPlanes;

struct GapPairResult
{
	bool valid = false;
	cv::Point3f ceilPt;
	cv::Point3f floorPt;
	double gap = 0;
};



extern RECONSTRUCTION_ERROR_CODE s_ReconstructionErrorCode;

class PointReconstruction
{
public:
	PointReconstruction();
	~PointReconstruction();

	void VisualizeClippingResult(
		const ModuleStruct::Point3fArray& original_points,
		const ModuleStruct::Point3fArray& filtered_points,
		const std::vector<FloatPoint2D>& polygon_xy,
		const std::string& save_path = "Clipping_Verify.png",
		const cv::Size& canvas_size = cv::Size(2000, 2000)
	);
	bool isPointInPolygon(const cv::Point3f& point, const std::vector<cv::Point2f>& polygon);
	/*
	function : Plane Segmentation, Call this API first, the plane result saved in mPlane
	*/
	//bool PlaneSegmentation(std::string file_path, MEASUREMENT_MODE seg_mode, std::string compass_file = "", float compass_value=0);

	bool PlaneSegmentation(std::vector<std::string> file_path, std::vector<cv::Mat> RTs, cv::Mat relaRT, MEASUREMENT_MODE seg_mode, std::vector<std::vector<float>> station_pos, float mergeDataDir,
		                   std::string compass_file = "", float compass_value = 0, bool generate_contour = false);

	bool ReadPoints(std::string file_path, ModuleStruct::Point3fArray &points, std::vector<unsigned char> &  reflectance);

	bool PrePlaneSegmentation(ModuleStruct::Point3fArray &points_cut, std::vector<unsigned char> &reflectance_cut,std::vector<cv::Mat> RTs, std::vector<std::vector<float>> station_pos, float mergeDataDir);

	bool RestorePlaneToOriginalCoordinate(PlaneCutResultInterface& mPlane, const cv::Mat& relaRT);
	std::vector<std::vector<std::vector<cv::Point3f>>> RemeasureHoles(void);

	PlaneCutResultInterface &GetPlanes(void);

	/*
	function : Plane Structure Reconstrcution 
	*/
	bool StructureReconstruction(void);

	/*
	function : Plane Structure Reconstrcution with z
	*/
	bool StructureReconstruction(float fake_ceiling_z);

	/*
	function : Plane Structure Reconstrcution By Bottom Ceiling
	*/
	bool StructureReconstructionByBottomCeil(void);

	/*
	function:calculate the extreme values of the ceiling and floor
	*/
	bool CalculateCeilFloorExtremes(void);

	/*
	function : Export Plane Structure Reconstrcution Plane to Obj file
	Paramters:
	hasReflect: the obj have reflect and normal map
	*/
	void ExportAsObj(std::string filename, bool hasMapping=false);

	/*
	function : Export Origin Plane points to Obj file, TOBE implementing
	*/
	void ExportOriPointAsObj(std::string filename);

	std::vector<cv::Point3f> GetRoomContour(bool rebuild=false);

	void DisplayRoomContour(void);
	void MakeRoomSquare(void);

	//
	std::vector<StructuredPlane>  GetStructuredPlanes(void);
	
	void UpdateStructuredPlanesSquared(std::vector<StructuredPlane> updatePlanes);
	void UpdateStructuredPlanesSquaredMin(std::vector<StructuredPlane> updatePlanes);
	void UpdateStructuredPlanesSquaredMin05(std::vector<StructuredPlane> updatePlanes);
	std::vector<StructuredPlane> GetStructuredPlanesSquared(void);
	std::vector<StructuredPlane> GetStructuredPlanesSquaredMin(void);
	std::vector<StructuredPlane> GetStructuredPlanesSquaredMin05(void);
	
	void UpdateRoomContourSquared(std::vector<cv::Point3f> contour);
	void UpdateRoomContourSquaredMin(std::vector<cv::Point3f> contour);
	void UpdateRoomContourSquaredMin05(std::vector<cv::Point3f> contour);
	std::vector<cv::Point3f> GetRoomContourSquared(void);
	std::vector<cv::Point3f> GetRoomContourSquaredMin(void);
	std::vector<cv::Point3f> GetRoomContourSquaredMin05(void);
	
	std::vector<std::pair<int, int>> GetLShapeWallIdx(void);
	
	std::vector<std::pair<int, int>> GetParallelWallIdx(void);
	
	int GetPreWallId(int curId);
	
	int GetNextWallId(int curId);
	//TO BE implementing
	std::vector<std::pair<int, int>> GetOriginStructuredList(void) {};

	std::vector<int>  GetWallList(void);

	std::vector<int> GetPlaneList(ePlane_Type type);
	
	int GetBigestPlane(ePlane_Type type);
	int GetReferenceWall(void);
	std::vector<ePlane_Direction> GetPlaneDire();
	PlaneCutResultInterface GetPlaneCutResultInterface();

	void MovetoOneMeterLine(float oneMeterPos);

	//defect3d true:output 3d x,y,z defect value
	bool MeasureFlatenessDefectFcn(bool defect3d=false);


	void UpdateDoorWindowInfo(std::vector<StructuredPlane> updatePlanes);


	void InsertFakePoint(std::vector<int> insertMap);
	void InsertFakePoint(std::vector<cv::Point3f> contour_new, std::vector<int>  wallList);
private:

	inline cv::Point3f voxelCenterPoint(
		int gx, int gy,
		float voxelSize,
		float z)
	{
		return cv::Point3f(
			(gx + 0.5f) * voxelSize,
			(gy + 0.5f) * voxelSize,
			z
		);
	}

	/*
	function :Rotate all planes by North, If doesn't exit the compass file, rotate by bigest wall
	Param:
	return: rorate angle
	TO BE implementing
	*/
	float RotateByNorth(void);

	bool BuildStructuredData(void);
	void BuildReflectImage(int id, cv::Mat & dstImg);

	cv::Point3f verticesNormal(std::vector<cv::Point3f> pts);
	ePlane_Type getWallType(int planeid);
	cv::Point3f UniformNormals(const cv::Point3f normal, const cv::Point3f center);
	double GetAngle(double x1, double y1, double x2, double y2, double x3, double y3);
	cv::Point2f GetFootOfPerpendicular(const cv::Point2f & pt, const cv::Point2f & begin, const cv::Point2f & end);

	bool ReadCompass(std::string compass_file);

	std::vector<ePlane_Direction> ComputeDirection(void);

	void CreateWallList(void);

	std::vector<cv::Point3f>  SortVertices(std::vector<cv::Point3f> vertices, cv::Point3f normal);

	std::vector<Tree<Point3fArray>> DevidePlanes(void);

	void ComputeReferenceWall(void);

public:
	GapPairResult ceilMinPair{}, ceilMaxPair{};
	GapPairResult floorMinPair{}, floorMaxPair{};
	GapPairResult minGapPair{}, maxGapPair{};
private:

	struct GridKey
	{
		int x;
		int y;

		bool operator==(const GridKey& other) const
		{
			return x == other.x && y == other.y;
		}
	};

	struct GridKeyHash
	{
		std::size_t operator()(GridKey const& k) const
		{
			return std::hash<long long>()(
				(static_cast<long long>(k.x) << 32) ^ k.y
				);
		}
	};

	struct GridCell
	{
		double sumZ = 0.0;
		int count = 0;
		//cv::Point3f sample_pt;

		void add(const cv::Point3f& p)
		{
			sumZ += p.z;
			count++;
			//sample_pt = p;
		}

		double avgZ() const
		{
			return count == 0 ? 0.0 : sumZ / count;
		}
	};


	std::vector<StructuredPlane>   mStructuredPlane; //?-¨º????¨°?¡§?¡ê¦Ì?
	std::vector<StructuredPlane>   mStructuredPlaneSquared;//add ?¨°¡¤?o¨®¦Ì??¡êD¨ª
	std::vector<StructuredPlane>   mStructuredPlaneSquaredMin;//add ?¨°¡¤?o¨®¦Ì???D?¡ã??¡êD¨ª
	std::vector<StructuredPlane>   mStructuredPlaneSquaredMin05; //add by Szh

	PlaneCutResultInterface  mPlane;

	shared_ptr<Reconstructer> mpReconstruction;

	float mCompassValue;
	float mRorateAngle;
	bool  mFakeCeiling;
	int   mRefWall;
	float mMinx;
	float mMiny;
	bool cut_poly = false;

	std::vector<ePlane_Direction>  mPlaneDire;
	std::vector<int>    mWallClockWiseList;
	std::vector<std::pair<int, std::vector<std::pair<float, cv::Point3f>>>> flateness_defect;
	std::vector<std::pair<int, std::vector<std::pair<float, cv::Point3f>>>> removeCritical_flateness_defect;

	std::vector<cv::Point3f> mContour;
	std::vector<cv::Point3f> mContourSquared;//add ?¨°¡¤?o¨®¦Ì?¦Ì?¡ã???¨¤a
	std::vector<cv::Point3f> mContourSquaredMin;//add ?¨°¡¤?o¨®¦Ì???D?¡ã?¦Ì?¡ã???¨¤a
	std::vector<cv::Point3f> mContourSquaredMin05;//add by Szh

	//WallObstacleInfo* wallObstacleInfo;
};

