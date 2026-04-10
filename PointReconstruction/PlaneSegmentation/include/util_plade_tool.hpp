#ifndef _UTIL_PLADE_TOOL_HPP_
#define _UTIL_PLADE_TOOL_HPP_

#include <string>
#include <iostream>
#include "config.h"
#include "log.h"
#include "ModuleStruct.hpp"

namespace util_plade_tool
{
	static inline void SavePladeData(bool is_m_unit, std::string out_path, std::string path, ModuleStruct::Point3fArray& input_data, Vector<PlaneSegOutput>* fit_plane_seg_out)
	{
		char file_base_name[_MAX_FNAME] = {};
		std::string folder = out_path + "plade_file\\";
		if (IOData::createDirectory(folder))
		{
			log_error("createDirectory %s failed", folder.c_str());
			return;
		}
		_splitpath(path.c_str(), nullptr, nullptr, file_base_name, nullptr);
		std::string filedata_out = folder + file_base_name + std::string(".xyz");
		std::string filedata_index = folder + file_base_name + std::string("_planeIndex.txt");
		std::string filedata_plane = folder + file_base_name + std::string("_plane.txt");

		log_info("filedata_out =%s filedata_index =%s filedata_plane =%s ", filedata_out.c_str(), filedata_index.c_str(), filedata_plane.c_str());
		ofstream findex(filedata_index), fplane(filedata_plane);

		fplane << std::setprecision(5) << std::fixed;

		if (is_m_unit)
		{
#pragma omp parallel for
			for (int i = 0; i < input_data.size(); i++) {
				input_data[i].x /= 1000.f;
				input_data[i].y /= 1000.f;
				input_data[i].z /= 1000.f;
			}
		}

		IOData::SavePoint3fData(filedata_out, input_data);
		for (int i = 0; i < fit_plane_seg_out->size(); i++)
		{
			auto& pi = fit_plane_seg_out->at(i).point_ids;
			auto& pn = fit_plane_seg_out->at(i).plane_normal;
			auto& pc = fit_plane_seg_out->at(i).plane_center;
			if (is_m_unit)
			{
				pc = fit_plane_seg_out->at(i).plane_center / 1000.f;
			}

			for (int j = 0; j < pi.size(); j++)
			{
				findex << pi[j] << '\t';
			}
			//normals uniform
			if ((pn.x * pc.x + pn.y * pc.y + pn.z * pc.z) > 0.f)
			{
				pn.x = -pn.x;
				pn.y = -pn.y;
				pn.z = -pn.z;
			}

			fplane << pn.x << '\t' << pn.y << '\t' << pn.z << '\t' << (pn.x * pc.x + pn.y * pc.y + pn.z * pc.z);

			if (i < fit_plane_seg_out->size() - 1)
			{
				findex << endl;
				fplane << endl;
			}
		}
		findex.close();
		fplane.close();
	}
}

#endif /*_UTIL_PLADE_TOOL_HPP_*/