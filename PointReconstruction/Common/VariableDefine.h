#ifndef __VARIABLE_DEFINE_H
#define __VARIABLE_DEFINE_H
#include <time.h>
//Note: for factory build mode, don't remove FACTORY_MODE
//#define FACTORY_MODE
//#define CUSTOMER_RUIXIN
#ifdef CUSTOMER_RUIXIN
#define SORT_BY_NORTH
#endif
#include <opencv2/core/core.hpp>
//check meter or minimeter
#define CHECK_UNIT_POINT_SIZE	10000
#define CHECK_UNIT_POINT_THREADHOLD	0.8
#define CHECK_UNIT_POINT_LEN	50.f

//room size
#define PARALLEL_WALL_RATIO	0.5f
#define PARALLEL_WALL_ANGLE	0.5f

//plane filter
#define PLANE_DIAGONAL_LENGTH	2000.0f
#define PLANE_LATERAL_LENGTH	50.0f
#define	LSHAPE_ROOM_SQUARENESS_AREA_RATIO	0.6f
#define	LSHAPE_ROOM_SQUARENESS_LENGTH	1600.f
#define	WALL_NORMALS_AXIS_THRESHOLD	0.7F

#define NORMALS_VOXEL_ANGLE_THRESHOLD	11.f
#define MINIMUN_PLANE_POINT_SIZE	500

#define CEILING_ACTUAL_AREA_RATIO_TO_BOUNDBOX	0.8

#define HEAP_OBJECT_DELETE(p)	if(p){delete p; p = nullptr;}

#define RAD_OF_45_DEGREE	0.785f
#define RAD_OF_90_DEGREE	1.571f
#define RAD_OF_180_DEGREE	3.142f
#define RAD_OF_360_DEGREE	6.283f

enum PCD_DEVICE_TYPE
{
	POINTS_DEVICE_UCL360,
	POINTS_DEVICE_BLK360
};




#ifndef _WIN32
#include<unistd.h>
#include <sys/stat.h>
#include <sys/types.h>

#define INFINITE 0xFFFF

typedef unsigned long DWORD;

inline DWORD GetTickCount() { return clock() * 1000 / CLOCKS_PER_SEC; }


//#define NOMINMAX
inline double max(double a, double b) { return  (((a) > (b)) ? (a) : (b)); }
inline double min(double a, double b) { return  (((a) < (b)) ? (a) : (b)); }
inline int __cdecl _access(char const* _FileName,int _AccessMode)
{
	return access(_FileName, _AccessMode);
}
inline int _mkdir(char const* _Path) { return mkdir(_Path, 0755); }
inline int _rmdir( char const* _Path) { return rmdir(_Path); }
/*#ifndef max
#define max(a,b)            (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif
*/
//#endif  /* NOMINMAX */

//#define (std::max)(a,b) (((a) > (b)) ? (a) : (b))
//#define (std::min)(a,b) (((a) < (b)) ? (a) : (b))

#endif

extern PCD_DEVICE_TYPE input_device_hardware_type;

//extern void SetDeviceType(int type);

extern inline float GetPoint2PlaneDistance();

#define POINT_TO_PLANE_DISTANCE GetPoint2PlaneDistance()


extern float voxel_size_measurement;

extern inline void SetVoxelSizeFuction(float size);

//extern inline float GetVoxelSize();
  float GetVoxelSize();
//voxel size
#define BUILD_VOXEL_SIZE	GetVoxelSize()

//define parameter change for measurement on AIP bad data
#define PARAM_FOR_MEASURE_ON_AIP_BADDATA
//define radius for hole on the ground
//#define GROUND_HOLE_RADIUS 620.f
//structure for each flatness and ruler vertices
typedef struct MeasurementValueRulerverticeStruct {
	 bool valid;
	 int ruler_indx;
	 int ruler_type;
	 int subwall_id;//added by hgj 20210802
	 int subwall_type;//added by hgj 20210908
	 float value;//flatness or verticality
	 float rect_radius;// for rect_verticality, rect_height > rect_width ? rect_height : rect_width

	 std::vector<cv::Point3f> ruler_vertice;


	 //just for rect verticality pair ,verticality_value  and rectangle center point 
	 std::vector<std::pair<float, cv::Point3f>> rect_veriticality_pair;
	 //only available for flatness now
	 std::vector<float> local_value;
	 std::vector<std::vector<cv::Point3f>> local_rulervertice;
	 std::vector<float> margin_left_bottom_dist;//0-left,1-bottom added by hgj 20220117

	 MeasurementValueRulerverticeStruct()
	 {
		 ruler_indx = -1;
		 ruler_type = -1;
		 subwall_id = -1;
		 subwall_type = -1;
		 rect_radius=100;
		 value = -1.0f;
		 valid = true;
		 ruler_vertice.resize(0);
		 local_value.resize(0);
		 local_rulervertice.resize(0);
		 rect_veriticality_pair.resize(0);
		 margin_left_bottom_dist.resize(0);
	 }
}S_MeasurementValueRulerverticeStruct;

typedef struct MeasurementRulerverticeStruct { 
	bool is_valid;
	int ruler_indx;//added by hgj 20211216
	int subwall_id;
	int subwall_type;//added by hgj 20210908
	//for one ruler
	float value;//flatness / verticality/ ceiling height for levelness computation
	// std::vector<std::pair<cv::Point3f, std::vector<std::pair<cv::Point3f, float>>>> ruler_vertice_intersect_pts_dis 
	//two vectors for pairs of ruler vertice information < vertice, two intersect pts and dists on plane boundary>
	std::vector<cv::Point3f> ruler_endpts;//two center or end pts of ruler 
	std::vector<cv::Point2f> ruler_endpts_2d;//two center or end pts of ruler 2d 
	std::vector<std::pair<cv::Point3f, cv::Point3f>> endpt_intersect_pts;// two intersect pts on boundary for two center or end pts
	std::vector<std::pair<float, float>> endpt_intersect_dist;//dist btween two intersect pts and boundary for two center or end pts
	std::vector<float> margin_left_bottom_dist;//added by hgj 20220114 0-1eft distance,1-bottom distance
}S_MeasurementRulerverticeStruct;

#endif