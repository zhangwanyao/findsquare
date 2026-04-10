#include "sDef.h"

#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>

bool ReadEncryFile(vector<sPointInfo>* vecPtInfos, string strPath)
{

	// the old 2024.12.09
#if 0
	ifstream inFile(strPath, ios::in | ios::binary); //二进制读方式打开
	int nCount = 0;
	inFile.read((char*)&nCount, sizeof(int));
	char nSeed = nCount % 255;
	size_t count = nCount * sizeof(sPointInfo);
	char * buf = new char[count];
	inFile.read(buf, count);
	inFile.close();
	for (int n = 0; n < count; n++)
	{
		buf[n] = buf[n] ^ nSeed;
	}
	sPointInfo * t = (sPointInfo *)buf;
	vecPtInfos->insert(vecPtInfos->begin(), t, t + nCount);
	delete buf;
	return true;
#endif

	try
	{
		std::uint32_t pointCntShold = 0;
		std::uint32_t size = 0;
		{
			std::ifstream in(strPath.c_str());
			in.seekg(0, std::ios::end);
			size = in.tellg();
			in.close();
			std::cout << "FILE SIZE = " << size << std::endl;;;
			pointCntShold = static_cast<std::uint32_t>((size - sizeof(std::uint32_t)) / sizeof(sPointInfo));
		}
		std::ifstream inFile(strPath, std::ios::in | std::ios::binary); //二进制读方式打开
		std::uint32_t nCount = 0;
		inFile.read((char*)&nCount, sizeof(std::uint32_t));
		if (nCount != pointCntShold)
		{
			sPointInfo err;
			err.x = nCount;
			err.y = pointCntShold;
			vecPtInfos->emplace_back(err);
			std::cout << "try read from sPointInfoV1 :nCount!= pointCntShold (" << nCount << " vs." << pointCntShold << ")" << std::endl;;;
			return false;
		}
		char nSeed = nCount % 255;
		std::uint32_t count = nCount * sizeof(sPointInfo);
		char* buf = new char[count];
		inFile.read(buf, count);
		inFile.close();
		for (std::uint32_t n = 0; n < count; n++)
		{
			buf[n] = buf[n] ^ nSeed;
		}
		sPointInfo* t = (sPointInfo*)buf;
		vecPtInfos->insert(vecPtInfos->begin(), t, t + nCount);
		delete[]buf;
		return true;
	}
	catch (...)
	{
		return false;
	}
}

bool WriteEncryFile(const std::vector<sPointInfo>* vecPtInfos, const std::string& strPath)
{
	std::uint32_t nCount = vecPtInfos->size();
	char nSeed = nCount % 255;
	const sPointInfo* sInfos = vecPtInfos->data();
	std::ofstream outFile(strPath, std::ios::out | std::ios::binary);
	outFile.write((char*)&nCount, sizeof(std::uint32_t));
	outFile.flush();
	std::uint32_t count = vecPtInfos->size() * sizeof(sPointInfo);
	char* buf = new char[count];
	memcpy(buf, sInfos, count);
	for (std::uint32_t n = 0; n < count; n++)
	{
		buf[n] = buf[n] ^ nSeed;
	}
	outFile.write(buf, count);
	delete[] buf;
	outFile.close();
	return true;
}

bool ReadEncryFileV2(std::vector<sPointInfoV2>* vecPtInfos, const std::string& strPath)
{
	try
	{
		uint32_t pointCntShold = 0;
		std::uint32_t  size = 0;
		{
			std::ifstream in(strPath.c_str());
			in.seekg(0, std::ios::end);
			size = in.tellg();
			in.close();
			std::cout << "FILE SIZE = " << size << std::endl;;;
			pointCntShold = static_cast<uint32_t>((size - sizeof(uint32_t)) / sizeof(sPointInfoV2));
		}
		std::ifstream inFile(strPath, std::ios::in | std::ios::binary); //二进制读方式打开
		uint32_t nCount = 0;
		inFile.read((char*)&nCount, sizeof(uint32_t));
		if (nCount != pointCntShold * sizeof(sPointInfoV2))
		{
			sPointInfoV2 err;
			err.x = nCount;
			err.y = pointCntShold;
			vecPtInfos->emplace_back(err);
			std::cout << "try read from sPointInfoV2 :nCount!= pointCntShold (" << nCount << " vs." << pointCntShold << ")" << std::endl;;
			return false;
		}
		uint32_t count = nCount;
		char* buf = new char[count];
		inFile.read(buf, count);
		inFile.close();
		sPointInfoV2* t = (sPointInfoV2*)buf;
		vecPtInfos->insert(vecPtInfos->begin(), t, t + pointCntShold);
		delete[]buf;
		return true;
	}
	catch (...)
	{
		return false;
	}
}

bool ReadEncryFileV2_broken(std::vector<sPointInfoV2>* vecPtInfos, const std::string& strPath)
{
	std::cout << "ReadEncryFileV2_broken !!!" << std::endl;;
	try
	{
		std::uint32_t pointCntShold = 0;
		size_t size = 0;
		{
			std::ifstream in(strPath.c_str());
			in.seekg(0, std::ios::end);
			size = in.tellg();
			in.close();
			std::cout << "FILE SIZE = " << size << std::endl;;
			pointCntShold = static_cast<std::uint32_t>((size - sizeof(std::uint32_t)) / sizeof(sPointInfoV2));
		}
		std::ifstream inFile(strPath, std::ios::in | std::ios::binary); //二进制读方式打开
		std::uint32_t nCount = 0;
		inFile.read((char*)&nCount, sizeof(std::uint32_t));
		nCount = pointCntShold;
		std::uint32_t count = nCount * sizeof(sPointInfoV2);
		char* buf = new char[count];
		inFile.read(buf, count);
		inFile.close();
		sPointInfoV2* t = (sPointInfoV2*)buf;
		vecPtInfos->insert(vecPtInfos->begin(), t, t + pointCntShold);
		delete[]buf;
		return true;
	}
	catch (...)
	{
		return false;
	}

}

std::vector<std::string> splitString(const std::string& str, char delimiter, bool trimEmpty) {
	std::vector<std::string> tokens;
	std::string token;
	std::istringstream tokenStream(str);

	while (std::getline(tokenStream, token, delimiter)) {
		if (!trimEmpty || !token.empty()) {
			tokens.push_back(token);
		}
	}

	return tokens;
}

// this code implementates in InOutData.cpp
#if 0
std::vector<std::pair<cv::Point3f, int>> readFromXyz(const std::string& path, const bool& useEncryptData = true, std::vector<float>* intensity = nullptr)
{


	std::vector<cv::Point3f> cloud;
	std::vector<int> reflectance;
	std::vector<std::pair<cv::Point3f, int>> pair_cloud_reflect;
	std::string tail = path.substr(path.length() - 4);
	std::transform(tail.begin(), tail.end(), tail.begin(), ::toupper);
	if (useEncryptData)
	{
		std::vector<sPointInfo> data;
		std::vector<sPointInfoV2> dataV2;
		if (ReadEncryFile(&data, path))
		{
			if (data.size() == 1)
			{

				std::cout << "nCount!= pointCntShold (" << data[0].x << " vs." << data[0].y << ")" << std::endl;

				return std::vector<std::pair<cv::Point3f, int>>();
			}
		}
		else if (ReadEncryFileV2(&dataV2, path))
		{
			if (dataV2.size() == 1)
			{

				std::cout << "nCount!= pointCntShold (" << dataV2[0].x << " vs." << dataV2[0].y << ")" << std::endl;


				return std::vector<std::pair<cv::Point3f, int>>();
			}
		}
		else
		{
			return std::vector<std::pair<cv::Point3f, int>>();
		}
		if (data.size() > 1)
		{
			if (intensity)intensity->resize(data.size());
			cloud.resize(data.size());
			reflectance.resize(data.size());
#pragma omp parallel for 
			for (int i = 0; i < data.size(); i++)
			{
				cloud[i].x = data[i].x;
				cloud[i].y = data[i].y;
				cloud[i].z = data[i].z;
				if (intensity)(*intensity)[i] = data[i].i;
				reflectance[i] = int(dataV2[i].i);

			}
			data.clear();
			std::vector<sPointInfo>().swap(data);
		}
		else if (dataV2.size() > 1)
		{
			if (intensity)intensity->resize(dataV2.size());
			cloud.resize(dataV2.size());
#pragma omp parallel for 
			for (int i = 0; i < dataV2.size(); i++)
			{
				cloud[i].x = -0.001 * dataV2[i].y;
				cloud[i].y = -0.001 * dataV2[i].x;
				cloud[i].z = -0.001 * dataV2[i].z;
				if (intensity)(*intensity)[i] = dataV2[i].i;
				reflectance[i] = int(dataV2[i].i);
			}
			dataV2.clear();
			std::vector<sPointInfoV2>().swap(dataV2);
		}
	}
	else
	{
		try
		{
			std::string aline;
			std::fstream fin1(path, std::ios::in);
			std::getline(fin1, aline);
			std::vector<std::string>seg = splitString(aline, ' ', true);
			bool hasIntensity = false;
			if (seg.size() == 4)
			{
				hasIntensity = true;
			}
			if (!(seg.size() == 4 || seg.size() == 3 || seg.size() == 6))
			{
				return std::vector<std::pair<cv::Point3f, int>>();
			}
			else
			{
				std::stringstream ss(aline);
				{
					float x, y, z, intensity_ = 0;
					ss >> x >> y >> z >> intensity_;
					cloud.emplace_back(cv::Point3f(x, y, z));
					if (intensity && hasIntensity) { intensity->emplace_back(intensity_); };
					reflectance.emplace_back(int(intensity_));
				}
			}
			while (std::getline(fin1, aline))
			{
				std::stringstream ss(aline);
				{
					float x, y, z, intensity_ = 0;
					ss >> x >> y >> z >> intensity_;
					cloud.emplace_back(cv::Point3f(x, y, z));
					if (intensity && hasIntensity) { intensity->emplace_back(intensity_); };
					reflectance.emplace_back(int(intensity_));
				}
			}
			fin1.close();
		}
		catch (...)
		{
			return std::vector<std::pair<cv::Point3f, int>>();
		}
	}

	for (int i = 0; i < cloud.size(); i++)
	{
		std::pair<cv::Point3f, int> point_reflect;
		point_reflect.first = cloud[i];
		point_reflect.second = reflectance[i];

		pair_cloud_reflect.emplace_back(point_reflect);
	}


	return pair_cloud_reflect;
}
#endif

