#include "InOutData.h"
#include "DecorationHelper.h"
#include "DecorationAPI.h"
#include "PointReconstruction.h"
#include "PointReconstructionGetConfig.h"

HDECOR handle;
std::vector<cv::Point2f> polygon_points;

int APIStartSemantic() {

	handle = InitDecoration("", 0);
	cout << "GetServerInfoPath(): " << GetServerInfoPath() << "\n";
	/*if (!polygon_points.empty())
	{
		SetPolygon(handle, polygon_points);
	}*/
	log_info("[PlaneSegmentation] start");
	DWORD PlaneSegmentation_Start = GetTickCount();
	bool res = PlaneSegmentation(handle, GetServerInfoPath(), GetStations(),false);
	DWORD PlaneSegmentation_End = GetTickCount();
	log_info("[PlaneSegmentation] end");

	cout << "[APIStartSemantic] PlaneSegmentation time = " << PlaneSegmentation_End - PlaneSegmentation_Start << endl;

	return 1;
}

int APISelectCeiling()
{
	DWORD APISelectCeiling_Start = GetTickCount();

	log_info("[APISelectCeiling] StructureReconstrcution start");
	bool res = StructureReconstruction_auto(handle); // CGAL闭合计算
	//bool res = StructureReconstruction(handle,id);
	log_info("[APISelectCeiling] StructureReconstrcution end");
	log_info("[APISelectCeiling] StructureReconstrcution res: %d", static_cast<int>(res));
	if (!res)return -1;

	//cout << "[APISelectCeiling] ExportStructureMeshes start" << endl;
	//cout << "[APISelectCeiling] ExportStructureMeshes mesh_opening_path:" << GetSelectCeilingPath() << endl;
	//ExportStructureMeshes(handle, GetSelectCeilingPath()); // 根据选择的ceiling生成选择的obj 可以注释掉
	//cout << "[APISelectCeiling] Export Json" << endl;

	DWORD APISelectCeiling_End = GetTickCount();
	cout << "[APISelectCeiling] time = " << APISelectCeiling_End - APISelectCeiling_Start << endl;

	//if (ExportResult(handle, GetJsonPath())) { // 输出结果 可以注释
	//	return 0;
	//}
	//else {
	//	cout << "[APISelectCeiling] ExportResult fail!!!" << endl;
	//	return -1;
	//}
	return 0;
}

int APIStartMeasurement()
{
	DWORD StartMeasurement_Start = GetTickCount();
	log_info("[StartMeasurement] start");
	bool res = StartMeasurement(handle);
	log_info("[StartMeasurement] end");
	DWORD StartMeasurement_End = GetTickCount();
	cout << "[APIStartMeasurement] StartMeasurement time = " << StartMeasurement_End - StartMeasurement_Start << endl;

	if (!res)return -1;

	//ExportStructureMeshes(handle, GetMeshOutPath());  // 输出结果 可以注释
	//cout << "[APIStartMeasurement] ExportStructureMeshes" << endl;

	//ExportStructureMeshesSquare(handle, GetSquaredOutPath());
	//cout << "[APIStartMeasurement] ExportStructureMeshesSquare:" << GetSquaredOutPath() << endl;

	res = ExportResult(handle, GetJsonPath());
	cout << "[APIStartMeasurement] ExportResult res:" << res << endl;
	if (!res)return -1;

	/*std::string source_xml_path = GetXmlRoomcontourStruct();
	std::string xml_path = GetXmlRoomcontourSquared();
	res = ExportRoomcontourXml(handle, source_xml_path, xml_path);
	cout << "[APIStartMeasurement] ExportRoomcontourXml res:" << res << endl;*/

	DeinitDecoration(handle);
	return 0;
}

int App(std::string paramPath, std::string squareType) {

	DWORD dwStart = GetTickCount();

	log_info("paramPath: %s", paramPath.c_str());
	log_info("squareType: %s", squareType.c_str());
	SetAllRulersJson(paramPath);
	//SetDistrice(polygon_points);

	log_info("[APIStartSemantic] start ");
	int ret = APIStartSemantic();
	log_info("[APIStartSemantic] end ");

	if (ret==1)
		log_info("[APISelectCeiling] start ");
		APISelectCeiling();
		log_info("[APISelectCeiling] end ");

	log_info("[APIStartMeasurement] start ");
	APIStartMeasurement();
	log_info("[APIStartMeasurement] end ");

	std::cout << "ALL RUN SUCCESS" << std::endl;

	DWORD dwEnd = GetTickCount();

	std::cout << "Decoration total time = " << dwEnd - dwStart << std::endl;

	return 0;
}


//void EnableLog(std::string log_file_path)
//{
//	IOData::createDirectory(log_file_path);
//	FILE* log_file = NULL;
//	util_log::log_cfg_set(log_file_path, log_file, 1, LOG_TRACE);
//}

//void DisbleLog(void)
//{
//	log_file_close();
//}

//int main(int argc, char** argv) {
//	std::ios::sync_with_stdio(false);
//	std::cin.tie(nullptr);
//	//enable logs
//	std::string log_file = "dll_log\\find_Square.txt";
//	EnableLog(log_file);
//
//	App("E:\\Defect\\20251120_ifc\\860cba36-ff37-4ab7-a169-a6d5ec533637\\Semantic\\NgNtfnc0qKbM4TRS5gMU8uac\\param.json", "1");
//
//	DisbleLog();
//	return 0;
//}
