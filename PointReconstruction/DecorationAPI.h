#ifndef DECORATION_API_H
#define DECORATION_API_H
#include <string>
#include <vector>

#ifdef _WIN32
#ifdef RECORATION_DLL_API_EXPORTS
#define DLL_API __declspec(dllexport)
#else
#define DLL_API __declspec(dllimport)
#endif
#else
#define DLL_API
#endif

typedef void * HDECOR;

/**
* \brief init decoration instance
*
* @param workDir the work directory for API calls, if 
* @param seg_mode : plane seg mode, 0 wall base, 1 wall less, 2 beam base.
* @return int: the handle of API calls, must save the handle
*/
DLL_API HDECOR InitDecoration(std::string  workDir, int seg_mode);

/**
* \brief release the handle
*/
DLL_API void DeinitDecoration(HDECOR handle);

/**
* \brief update door and windows information
*
* @param handle, the instance handle
* @param updateJson : the update json
*/
DLL_API void UpdateDoorWindowInfo(HDECOR handle, std::string updateJson);

/**
* \brief Plane Segmentation
*
* @param handle, the instance handle
* @param multi_json_file : multi json file
* @param station_ids : station ids
* @return bool: return the API result
*/
DLL_API bool PlaneSegmentation(HDECOR handle, std::string multi_json_file, std::vector<unsigned long long> station_ids, bool generate_contour = false);


/**
* \brief Marker Ball Detection and out put balls' center
*
* @param handle, the instance handle
* @param output_path, center.pts output dir
*/

DLL_API void MarkerBallDetectionAndSaveCenter(HDECOR handle, std::string sense_path, std::string output_path);

/**
* \brief structure reconstrcution
*
* @param handle, the instance handle
* @return bool: return the API result
*/
DLL_API bool StructureReconstruction_auto(HDECOR handle);

/**
* \brief structure reconstrcution
*
* @param handle, the instance handle
* @param fake_ceiling_z : the z value of fake ceiling
* @return bool: return the API result
*/
DLL_API bool StructureReconstruction(HDECOR handle, float fake_ceiling_z = 1E-10);

/**
* \brief structure reconstrcution
*
* @param handle, the instance handle
* @param plane_id : ceiling plane id
* @return bool: return the API result
*/
DLL_API bool StructureReconstruction(HDECOR handle, int plane_id=-1);


/**
* \brief export the structure Plane mesh min 
*
* @param handle, the instance handle
* @param mesh_dir : mesh output dir
*/
DLL_API void ExportStructureMeshesSquareMin(HDECOR handle, std::string mesh_dir = "");

/**
* \brief export the structure Plane mesh min05
*
* @param handle, the instance handle
* @param mesh_dir : mesh output dir
*/
DLL_API void ExportStructureMeshesSquareMin05(HDECOR handle, std::string mesh_dir = "");


/**
* \breif set districe polygon points
* 
* @param polygon_points : polygon points
*/
//DLL_API void SetDistrice(std::vector<cv::Point2f>& polygon_points);

/**
* \brief check if has muliti ceiling
*
* @param handle, the instance handle
* @param bool : the status
*/
DLL_API bool IsMultiCeiling(HDECOR handle);

/**
* \brief start measurement
*
* @param handle, the instance handle
* @param room_squareness_mode, 0: 2D squareness, 1: 3D  squareness, 2: 3D Convex squareness
* @param bool : the status
*/
DLL_API bool StartMeasurement(HDECOR handle);

/**
* \brief export all results(measurement result and structrue plane results)
*
* @param handle, the instance handle
* @param outputJson, the output json name.
* @param bool : the status
*/
DLL_API bool ExportResult(HDECOR handle, std::string outputJson="");


/**
* \brief Get Origin Ceiling index
*
* @param handle, the instance handle
* @param std::vector<int> : ceiling index
*/
DLL_API std::vector<int> GetOriginCeilingIdx(HDECOR handle);

/**
* \brief Get Bottom Ceiling
*
* @param handle, the instance handle
* @param std::int : ceiling index
*/
DLL_API int GetOriginBottomCeilingId(HDECOR handle);

/*
function : return last function error code
*/
DLL_API int GetErrorCode();


//DLL_API void SetPolygon(HDECOR handle,const std::vector<cv::Point2f>& points);


DLL_API void SetAllRulersJson(std::string rulerJsonString);

DLL_API void ExportCreatingMesh(HDECOR handle, std::string markersFile, float ratio, std::string output_path);

/**
* \brief log output
*/
DLL_API void EnableLog(std::string log_file_path);
DLL_API void DisbleLog(void);

// breif : get json path
DLL_API std::string GetJsonPath();

// breif : get station ids
DLL_API std::vector<unsigned long long> GetStations();

// breif : get server info path
DLL_API std::string GetServerInfoPath();
#endif