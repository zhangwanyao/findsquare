#include <iostream>
#include "DecorationAPI.h"
#include <windows.h>
#define CRT_DEBUG 1
#define MASTER_DEBUG 0

int main() {
	
#ifdef CRT_DEBUG
	DWORD dwStart = GetTickCount();

	HDECOR handle;
	std::string paramPath = "E:\\Defect\\20260408_findsquare\\Squareness\\43Twa9RSiQIR65qhXQhGYsNf\\param.json";
	bool res = false;

	std::string logtxt = "dll_log\\log.txt";
	EnableLog(logtxt);
	SetAllRulersJson(paramPath);
	handle = InitDecoration("", 0);
	if (handle == nullptr)
	{
		std::cerr << "ERROR: InitDecoration failed." << std::endl;
		return -1;
	}

	res = PlaneSegmentation(handle, GetServerInfoPath(), GetStations(), false);
	if (!res)
	{
		std::cerr << "ERROR: PlaneSegmentation() failed." << std::endl;
		DeinitDecoration(handle);
		return -1;
	}

	res = StructureReconstruction_auto(handle);
	if (!res)
	{
		std::cerr << "ERROR: StructureReconstruction_auto() failed." << std::endl;
		DeinitDecoration(handle);
		return -1;
	}

	res = StartMeasurement(handle);
	if (!res)
	{
		std::cerr << "ERROR: StartMeasurement() failed." << std::endl;
		DeinitDecoration(handle);
		return -1;
	}

	res = ExportResult(handle, GetJsonPath());
	if (!res)
	{
		std::cerr << "ERROR: ExportResult() failed." << std::endl;
		DeinitDecoration(handle);
		return -1;
	}

	DeinitDecoration(handle);
	DisbleLog();

	DWORD dwEnd = GetTickCount();

	std::cout << "Decoration total time = " << dwEnd - dwStart << std::endl;

#endif

//#ifdef MASTER_DEBUG
//	HDECOR handle;
//	std::string paramPath = "E:\\Defect\\20251226_cutdiff\\Semantic\\4cdic1KYNMPAl00zWwWcXjU0\\param.json";
//	bool res = false;
//	std::string serve_info_json = "E:\\Defect\\20251226_cutdiff\\Scanning\\server_info_4cdic1KYNMPAl00zWwWcXjU0.json";
//	std::string pts = "E:\\Defect\\20251226_cutdiff\\Scanning\\251225092141956\\scan_251225092141956.pts";
//	std::string dire = "E:\\Defect\\20251226_cutdiff\\Scanning\\251225092141956\\dire.txt";
//	SetAllRulersJson(paramPath);
//	handle = InitDecoration("", 0);
//	res = PlaneSegmentation(handle, pts, dire);
//
//	StructureReconstruction(handle, 2000.0f);
//	StartMeasurement(handle);
//
//	ExportResult(handle, "output.json");
//	DeinitDecoration(handle);
//#endif
	return 0;
}
