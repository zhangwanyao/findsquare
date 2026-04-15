#include "DecorationAPI.h"
#include "PointReconstructionGetConfig.h"
#include "DecorationHelper.h"
#include "log/log.h"
#include "PcdDsAPI.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "InOutData.h"

DecorationHelper* TO_HELPER(HDECOR handle) {
	return static_cast<DecorationHelper*>(handle);
}

RECONSTRUCTION_ERROR_CODE s_ReconstructionErrorCode;

static std::string all_rulers_json; // param.json
//static std::string g_DistriceSet_path = ""; // cut.json 转移用不到
static std::string g_mesh_ceiling_path = ""; 
static std::string g_mesh_opening_path = "";
static std::string g_update_json_path = "";
static std::string g_mesh_out_path = "";
static std::string g_json_path = "";
static std::string g_squared_out_path = "";
static std::string g_server_info_path = "";
static int g_seg_mode = 0; // model roi cut
static int g_ds_mode = 0;// downsampling mode
static int g_square_mode = 1; // find square mode
static bool g_square_by_axis = false;
static bool g_customize_square = false;
static int g_square_height = -1;
static int g_square_width = -1;
static int g_min_square_offset = 100;
static std::string g_axis_eqn_config = "";
static std::vector<unsigned long long> station_ids;
static std::vector<unsigned long long> dist_station_ids;
static std::vector<IntPoint2D> polypt_uv;
static std::vector<FloatPoint2D> polypt_xy;

DLL_API HDECOR InitDecoration(std::string  workDir, int seg_mode)
{
	DecorationHelper *pHelp = new DecorationHelper(workDir, GetSegMode());
	return (HDECOR)pHelp;
}

DLL_API void DeinitDecoration(HDECOR handle)
{
	delete (TO_HELPER(handle));
}


DLL_API void UpdateDoorWindowInfo(HDECOR handle, std::string updateJson)
{
	TO_HELPER(handle)->UpdateDoorWindowInfo(updateJson);
}


DLL_API void MarkerBallDetectionAndSaveCenter(HDECOR handle, std::string sense_path, std::string output_path)
{
	TO_HELPER(handle)->MarkerBallDetection(sense_path, output_path);
}

// 可以设置高度
DLL_API bool StructureReconstruction(HDECOR handle, float fake_ceiling_z)
{
	return TO_HELPER(handle)->StructureReconstruction(fake_ceiling_z);
}

DLL_API bool StructureReconstruction(HDECOR handle, int plane_id)
{
	return TO_HELPER(handle)->StructureReconstruction((int)plane_id);
}

DLL_API bool StructureReconstruction_auto(HDECOR handle)
{
	return TO_HELPER(handle)->StructureReconstruction_auto();
}


DLL_API void ExportStructureMeshesSquareMin(HDECOR handle, std::string mesh_dir)
{
	return TO_HELPER(handle)->ExportStructureMeshesSquareMin(mesh_dir);
}

DLL_API void ExportStructureMeshesSquareMin05(HDECOR handle, std::string mesh_dir)
{
	return TO_HELPER(handle)->ExportStructureMeshesSquareMin05(mesh_dir);
}

DLL_API bool IsMultiCeiling(HDECOR handle)
{
	return TO_HELPER(handle)->IsMultiCeiling();
}

DLL_API bool StartMeasurement(HDECOR handle)
{
	return TO_HELPER(handle)->StartMeasurement(GetSquareMode(), GetAxisEqnConfig());
}

DLL_API bool ExportResult(HDECOR handle, std::string outputJson)
{
	return TO_HELPER(handle)->ExportResult(outputJson);
}


DLL_API std::vector<int> GetOriginCeilingIdx(HDECOR handle)
{
	return TO_HELPER(handle)->GetOriginCeilingIdx();
}

DLL_API int GetOriginBottomCeilingId(HDECOR handle)
{
	return TO_HELPER(handle)->GetOriginBottomCeilingId();
}

DLL_API int GetErrorCode()
{
	return (int)s_ReconstructionErrorCode;
}

#include "MultiStage\StageMatrix.h"


DLL_API bool PlaneSegmentation(HDECOR handle, std::string multi_json_file, std::vector<unsigned long long> station_ids,bool generate_contour)
{
	// 加载serverinfo点云数据
	DWORD start_prepare = GetTickCount();
	StageMatrix *pstateMatrix = new StageMatrix();
	pstateMatrix->LoadStationInfo(multi_json_file);

	std::map<int, StationInfo> stationInfo;

	stationInfo = std::move(pstateMatrix->GetStationInfos()); // 移动语义构造站点信息
	float mergeDataDir = std::move(pstateMatrix->GetmergeDataDir()); // 房间朝向角度

	std::vector<std::vector<float>> station_pos;

	TO_HELPER(handle)->SetStageMatrix(pstateMatrix);

	std::vector<std::string> compute_files;
	std::vector<cv::Mat>  RTs;

	float dire = -1;
	int firstid = -1;

	log_info("downsampling mode = %d", GetDsMode());
	//std::cout << "downsampling mode = " << GetDsMode() << std::endl;
	log_info("station size = %zu", station_ids.size());
	//std::cout << "station size = " << station_ids.size() << std::endl;

	// 降采样ICO球面投影
	const long long FILE_SIZE_THRESHOLD = 100LL * 1024 * 1024;

	for (int i = 0; i < station_ids.size(); i++)
	{
		int id = -1;
		for (auto it = stationInfo.begin(); it != stationInfo.end(); it++)
		{
			if (it->second.pts_id == station_ids[i])
			{
				id = it->first;
				break;
			}
		}

		if (id != -1)
		{
			long long file_size = 0;
			struct stat stat_buf;
			if (stat(stationInfo[id].pts_filename.c_str(), &stat_buf) == 0) {
				file_size = stat_buf.st_size; // bytes
			}
			bool needSizeDownsample = (file_size > FILE_SIZE_THRESHOLD);

			if ((GetDsMode() == 1 && station_ids.size() > 2) || needSizeDownsample) {
				std::string station_ds_filename = stationInfo[id].pts_filename.substr(0, stationInfo[id].pts_filename.find(".pts"));
				station_ds_filename += ".unre";

				pcdIcoDownsample(stationInfo[id].pts_filename.c_str(), station_ds_filename.c_str(), 9);

				stationInfo[id].pts_filename = station_ds_filename;				
			}
			else if (GetDsMode() == 2) {
				std::string station_rmvOcc_filename = stationInfo[id].pts_filename.substr(0, stationInfo[id].pts_filename.find(".pts"));
				station_rmvOcc_filename += "_rmvOcc.pts";
				stationInfo[id].pts_filename = station_rmvOcc_filename;
			}
			compute_files.push_back(stationInfo[id].pts_filename);
			RTs.push_back(stationInfo[id].station_matrixs);
			station_pos.push_back(stationInfo[id].position);////###

			log_info("pts file: %s", stationInfo[id].pts_filename.c_str());
			//std::cout << stationInfo[id].pts_filename << std::endl;
			
			if (dire == -1)
			{
				dire = stationInfo[id].station_dire;
			}
			if (firstid == -1)
			{
				firstid = id;
				TO_HELPER(handle)->SetFirstId(firstid);
			}
		}
	}
	DWORD end_prepare = GetTickCount();
	std::cout << "prepare time = " << end_prepare - start_prepare << "ms." << "\n";

	// 平面分割
	return TO_HELPER(handle)->PlaneSegmentation(compute_files, RTs, pstateMatrix->GetRefMatrix(), station_pos, mergeDataDir, "", dire, generate_contour);
}


DLL_API void SetAllRulersJson(std::string rulerJsonString)
{

	all_rulers_json = rulerJsonString;
	json_error_t error;

	json_t *rulerJson = json_load_file(all_rulers_json.c_str(), 0, &error);
	if (!rulerJson)
	{
		log_error("error_import_paramter: on line %d: %s\n", error.line, error.text);
		json_decref(rulerJson);
		return;
	}

	if (json_object_get(rulerJson, "update_json_path"))
		g_update_json_path = json_string_value(json_object_get(rulerJson, "update_json_path"));
	if (json_object_get(rulerJson, "json_path"))
		g_json_path = json_string_value(json_object_get(rulerJson, "json_path"));
	if (json_object_get(rulerJson, "server_info_path"))
		g_server_info_path = json_string_value(json_object_get(rulerJson, "server_info_path"));

	if (json_object_get(rulerJson, "model"))
		g_seg_mode = json_integer_value(json_object_get(rulerJson, "model"));
	if (json_object_get(rulerJson, "ds_model"))
		g_ds_mode = json_integer_value(json_object_get(rulerJson, "ds_model"));
	if (json_object_get(rulerJson, "square_mode"))
		g_square_mode = json_integer_value(json_object_get(rulerJson, "square_mode"));
	if (g_square_mode == 2 || g_square_mode == 3) {
		log_info("legacy square_mode=%d detected, mapped to strategy_mode=%d", g_square_mode, GetSquareStrategyMode());
	}
	if (json_object_get(rulerJson, "square_by_axis"))
		g_square_by_axis = json_integer_value(json_object_get(rulerJson, "square_by_axis"));

	if (json_object_get(rulerJson, "customize_square"))
		g_customize_square = json_integer_value(json_object_get(rulerJson, "customize_square"));
	if (json_object_get(rulerJson, "customize_square_height"))
		g_square_height = json_integer_value(json_object_get(rulerJson, "customize_square_height"));
	if (json_object_get(rulerJson, "customize_square_width"))
		g_square_width = json_integer_value(json_object_get(rulerJson, "customize_square_width"));

	if (json_object_get(rulerJson, "min_square_offset"))
		g_min_square_offset = json_integer_value(json_object_get(rulerJson, "min_square_offset"));
	
	if (json_object_get(rulerJson, "stations")) {
		int st = json_array_size(json_object_get(rulerJson, "stations"));
		station_ids.resize(st);
		for (int i = 0; i < st; i++) {
			string str = json_string_value(json_array_get(json_object_get(rulerJson, "stations"), i));
			station_ids[i] = strtoll(str.c_str(), NULL, 10);
			log_info("station_ids %d:str: %llu", i, station_ids[i]);
			//std::cout << "station_ids " << i << ":str: " << station_ids[i] << endl;
		}
	}

	if (json_object_get(rulerJson, "regionalDivision")) {
		int st = json_array_size(json_object_get(rulerJson, "regionalDivision"));
		for (int i = 0; i < st; i++) {
			IntPoint2D temp;
			FloatPoint2D temp_xy;
			temp.x = json_integer_value(json_object_get(json_array_get(json_object_get(rulerJson, "regionalDivision"), i), "x"));
			temp.y = json_integer_value(json_object_get(json_array_get(json_object_get(rulerJson, "regionalDivision"), i), "y"));
			temp_xy.x = json_number_value(json_object_get(json_array_get(json_object_get(rulerJson, "regionalDivision"), i), "x3d"));
			temp_xy.y = json_number_value(json_object_get(json_array_get(json_object_get(rulerJson, "regionalDivision"), i), "y3d"));
			polypt_uv.push_back(temp);
			polypt_xy.push_back(temp_xy);
		}
	}
	else {
		polypt_uv.clear();
		polypt_xy.clear();
	}

	
	if (json_object_get(rulerJson, "axis_eqn_config"))
		g_axis_eqn_config = json_string_value(json_object_get(rulerJson, "axis_eqn_config"));
	json_decref(rulerJson);

	log_info("g_update_json_path: %s", g_update_json_path.c_str());
	log_info("g_json_path: %s", g_json_path.c_str());
	log_info("g_server_info_path: %s", g_server_info_path.c_str());
	log_info("g_seg_mode: %d", g_seg_mode);
	log_info("g_ds_mode: %d", g_ds_mode);
	log_info("g_square_mode(raw): %d", g_square_mode);
	log_info("g_square_strategy_mode(mapped): %d", GetSquareStrategyMode());
	log_info("g_square_by_axis: %d", g_square_by_axis);
	log_info("g_customize_square: %d", g_customize_square);
	log_info("g_square_height: %d", g_square_height);
	log_info("g_square_width: %d", g_square_width);
	log_info("g_min_square_offset: %d", g_min_square_offset);
	log_info("g_axis_eqn_config: %s", g_axis_eqn_config.c_str());
	log_info("SetAllRulersJson done.");

	/*std::cout << "\n g_update_json_path: " << g_update_json_path
		<< "\n g_json_path: " << g_json_path
		<< "\n g_server_info_path: " << g_server_info_path
		<< "\n g_seg_mode: " << g_seg_mode
		<< "\n g_ds_mode: " << g_ds_mode
		<< "\n g_square_mode: " << g_square_mode
		<< "\n g_square_by_axis: " << g_square_by_axis
		<< "\n g_customize_square: " << g_customize_square
		<< "\n g_square_height: " << g_square_height
		<< "\n g_square_width: " << g_square_width
		<< "\n g_square_width: " << g_min_square_offset
		<< "\n g_axis_eqn_config: " << g_axis_eqn_config
		<< endl;*/
}

DLL_API void ExportCreatingMesh(HDECOR handle, std::string markersFile, float ratio, std::string output_path)
{
	TO_HELPER(handle)->CreatingMesh_(markersFile, ratio, output_path);
}

DLL_API void EnableLog(std::string log_file_path)
{
	IOData::createDirectory(log_file_path);
	FILE* log_file = NULL;
	util_log::log_cfg_set(log_file_path, log_file, 1, LOG_TRACE);
}

DLL_API void DisbleLog(void)
{
	log_file_close();
}

DLL_API std::string GetJsonPath()
{
	return g_json_path;
}

DLL_API std::vector<unsigned long long> GetStations()
{
	return station_ids;
}

DLL_API std::string GetServerInfoPath()
{
	return g_server_info_path;
}
std::string GetMeshOpeningPath()
{
	return g_mesh_opening_path;
}

std::string GetSelectCeilingPath()
{
	return g_mesh_ceiling_path;
}

std::string GetUpdateJsonPath()
{
	return g_update_json_path;
}

std::string GetMeshOutPath()
{
	return g_mesh_out_path;
}



std::string GetSquaredOutPath()
{
	return g_squared_out_path;
}


int GetSegMode()
{
	return g_seg_mode;
}
int GetDsMode()
{
	return g_ds_mode;
}
int GetSquareMode()
{
	return g_square_mode;
}

int GetSquareStrategyMode()
{
	// New 2-mode mapping with backward compatibility
	// 0: regularized square
	// 1: fit-max-area square
	// 4: fit-max-area without orthogonal constraint
	// legacy 2(convexity) -> regularized
	// legacy 3(min-loss) -> fit-max-area
	if (g_square_mode == 4) {
		return SQUARE_STRATEGY_FIT_MAX_AREA_FREE;
	}
	if (g_square_mode == 1 || g_square_mode == 3) {
		return SQUARE_STRATEGY_FIT_MAX_AREA;
	}
	return SQUARE_STRATEGY_REGULARIZED;
}

bool IsRegularizedSquareMode()
{
	return GetSquareStrategyMode() == SQUARE_STRATEGY_REGULARIZED;
}
bool IsSquareByAxis()
{
	return g_square_by_axis;
}
bool IsCustomizeSuqare()
{
	return g_customize_square;
}

int GetSquareHeight()
{
	return g_square_height;
}

int GetSquareWidth()
{
	return g_square_width;
}

int GetMinSquareOffset()
{
	return g_min_square_offset;
}


std::vector<IntPoint2D> GetPolypt_uv()
{
	return polypt_uv;
}

std::vector<FloatPoint2D> GetPolypt_xy()
{
	return polypt_xy;
}

std::string GetAxisEqnConfig()
{
	return g_axis_eqn_config;
}
