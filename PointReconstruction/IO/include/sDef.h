#pragma once
#include <vector>
#pragma pack(push, 1)
using namespace std;

typedef struct sPointInfo
{
	float x;
	float y;
	float z;
	unsigned char i;
}sPointInfo;

typedef struct sPointInfoV2
{
	float x;
	float y;
	float z;
	std::uint16_t i;
}sPointInfoV2;

typedef struct sPointInfoEx
{
	float x;
	float y;
	float z;
	int i;
	int r;
	int g;
	int b;
	float blendApha;
}sPointInfoEx;

typedef struct sLidarEx
{
	double distance;
	double fTheta;
	double Azimuth;
	double fThetaEx;
	int intensity;
}sLidarEx;

//extern "C" bool __declspec(dllimport) WriteEncryFile(vector<sPointInfo>* vecPtInfos, string strPath);
//extern "C" bool __declspec(dllimport) ReadEncryFile(vector<sPointInfo>* vecPtInfos, string strPath);
bool ReadEncryFile(vector<sPointInfo>* vecPtInfos, string strPath);

bool WriteEncryFile(const std::vector<sPointInfo>* vecPtInfos, const std::string& strPath);

bool ReadEncryFileV2(std::vector<sPointInfoV2>* vecPtInfos, const std::string& strPath);

bool ReadEncryFileV2_broken(std::vector<sPointInfoV2>* vecPtInfos, const std::string& strPath);


std::vector<std::string> splitString(const std::string& str, char delimiter, bool trimEmpty = false);

#pragma pack(pop)