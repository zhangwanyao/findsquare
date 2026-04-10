#pragma once
#include <opencv2\opencv.hpp>
#include <vector>

typedef enum ePlane_Direction_t
{
	ePLANE_DIR_UNKOWN,
	ePLANE_EAST,
	ePLANE_SOUTH,
	ePLANE_WEST,
	ePLANE_NORTH
}ePlane_Direction;

typedef enum ePlane_Type_t
{
	ePLANE_TYPE_UNKOWN,
	ePLANE_WALL,
	ePLANE_BEAM,
	ePLANE_COLUMN,
	ePLANE_GROUND,
	ePLANE_CEILING
}ePlane_Type;


typedef enum eHole_Type_t
{
	eHOLE_UNKOWN,
	eHOLE_WINDOWS,
	eHOLE_DOOR
}eHole_Type;


class StructuredHole
{
public:
	StructuredHole() {
		id = 0;
		local_id = 0;
		type = eHOLE_UNKOWN;
		width = 0;
		height = 0;
	};
	StructuredHole & operator = (const StructuredHole &structHole);

	int                   id;
	int						local_id;
	eHole_Type             type;
	std::vector<cv::Point3f> vertice;
	std::vector<cv::Point3f> vertice_s;
	float              height;
	float              width;
};

class StructuredPlane
{
public:
	StructuredPlane() {
		direction = ePLANE_DIR_UNKOWN;
		type = ePLANE_TYPE_UNKOWN;
		wall_height = 0;
		wall_width = 0;
	};

	StructuredPlane & operator = (const StructuredPlane &structPlane);

public:
	int   id;
	cv::Point3f normal;        //normal
	cv::Point3f center;        //center point
	cv::Mat     reflectImg;    //reflectimage
	cv::Mat     normalImg;
	ePlane_Direction  direction;
	ePlane_Type type;
	std::vector<cv::Point3f>  vertices;
	std::vector<cv::Point3f>  vertices_s; //added by hgj for output xml
	std::vector<StructuredHole>  holes;
	cv::Point3f convPt;
	cv::Point3f wallConvPt;
	bool convPtF = false;
	bool         truePlane;
	float       wall_height;
	float       wall_width;
};
