#include "PlaneCuttingEx.h"

void ReWriteID(PlaneCutResultInterface& cutResult);
bool CutPlaneExt(MEASUREMENT_MODE mode, PlaneCutResultInterface& cutResult, bool withInclinedPlane,int station_size,bool cut_poly)
{
	bool isSuccess;
	try {
		Script_ROI script_roi;
		Script_ROI::TYPE_ROI processType;
		switch (mode)
		{
		case MEASUREMENT_MODE::WALL_BASE:
			processType = Script_ROI::TYPE_ROI::ROI_WALL_BASE;
			break;
		case MEASUREMENT_MODE::WALL_LESS:
			processType = Script_ROI::TYPE_ROI::ROI_WALL_LESS;
			break;
		case MEASUREMENT_MODE::BEAM_BASE:
			processType = Script_ROI::TYPE_ROI::ROI_BEAM_BASE;
			break;
		case MEASUREMENT_MODE::FLATNESS_ONLY:
			processType = Script_ROI::TYPE_ROI::ROI_NO_CUT;
			break;
		case MEASUREMENT_MODE::MANUAL:
			processType = Script_ROI::TYPE_ROI::MANUAL;
			break;
		default:
			break;
		}

		script_roi.setWithInclinedPlane(withInclinedPlane);
		isSuccess = script_roi.process(processType, cutResult,station_size,cut_poly);

		if(isSuccess)
			ReWriteID(cutResult);
	}
	catch (const std::string err) {
		std::cerr << err << std::endl;
		log_fatal(err.c_str());
	}
	return isSuccess;
}

void ReWriteID(PlaneCutResultInterface& cutResult)
{
	////#su: Rewrite All id:	new id
	std::vector<cv::Point3f> new_scene_plane_center;
	std::vector<cv::Point3f> new_scene_plane_normals;
	std::vector<std::vector<cv::Point3f>> new_scene_plane_xyz;
	std::vector<std::vector<uchar>> new_scene_plane_reflect;
	std::vector<int> new_scene_wall_idx;
	std::vector<int> new_scene_ceiling_idx;
	std::vector<int> new_scene_ground_idx;
	std::vector<int> new_scene_beam_idx;
	std::vector<std::pair<int, int>> new_scene_L_shape_idx;
	std::vector<std::pair<int, int>> new_scene_parallel_idx;
	std::vector<std::pair<int, int>> new_rewrite_idx;
	std::vector<std::vector<MeasureDoorWindow::DoorWindowInfo>> new_door_window_info;

	std::vector<float> new_plane_area;

	auto nPlane = cutResult.plane_xyz.size();
	vector<bool> id(nPlane, false);

	for (int i = 0; i < nPlane; i++)
	{
		if (cutResult.plane_xyz[i].size() > 0)
		{
			id[i] = true;
		}
	}

	int n = 0;
	
	//for wall
	for (auto i : cutResult.plane_wall_idx)
	{
		if (id[i]) {
			new_scene_plane_xyz.push_back(cutResult.plane_xyz[i]);
			new_scene_plane_reflect.push_back(cutResult.plane_reflect[i]);
			new_scene_plane_center.push_back(cutResult.plane_center[i]);
			new_scene_plane_normals.push_back(cutResult.plane_normals[i]);
			new_plane_area.push_back(cutResult.plane_area[i]);
			new_scene_wall_idx.push_back(n);
			std::pair<int, int> reID;
			reID.first = i;
			reID.second = n;
			//old:new
			new_rewrite_idx.push_back(reID);
			n++;
		}
		
	}

	//for beam
	for (auto i : cutResult.plane_beam_idx)
	{
		if (id[i]) {
			new_scene_plane_xyz.push_back(cutResult.plane_xyz[i]);
			new_scene_plane_reflect.push_back(cutResult.plane_reflect[i]);
			new_scene_plane_center.push_back(cutResult.plane_center[i]);
			new_scene_plane_normals.push_back(cutResult.plane_normals[i]);
			new_plane_area.push_back(cutResult.plane_area[i]);
			new_scene_beam_idx.push_back(n);
			std::pair<int, int> reID;
			reID.first = i;
			reID.second = n;
			new_rewrite_idx.push_back(reID);
			n++;
		}
		
	}


	//for ground
	for (auto i : cutResult.plane_ground_idx)
	{
		if (id[i]) {
			new_scene_plane_xyz.push_back(cutResult.plane_xyz[i]);
			new_scene_plane_reflect.push_back(cutResult.plane_reflect[i]);
			new_scene_plane_center.push_back(cutResult.plane_center[i]);
			new_scene_plane_normals.push_back(cutResult.plane_normals[i]);
			new_plane_area.push_back(cutResult.plane_area[i]);

			new_scene_ground_idx.push_back(n);
			std::pair<int, int> reID;
			reID.first = i;
			reID.second = n;
			new_rewrite_idx.push_back(reID);
			n++;
		}
	}

	//for ceiling
	for (auto i : cutResult.plane_ceiling_idx)
	{
		if (id[i]) {
			new_scene_plane_xyz.push_back(cutResult.plane_xyz[i]);
			new_scene_plane_reflect.push_back(cutResult.plane_reflect[i]);
			new_scene_plane_center.push_back(cutResult.plane_center[i]);
			new_scene_plane_normals.push_back(cutResult.plane_normals[i]);
			new_plane_area.push_back(cutResult.plane_area[i]);
			new_scene_ceiling_idx.push_back(n);
			std::pair<int, int> reID;
			reID.first = i;
			reID.second = n;
			new_rewrite_idx.push_back(reID);
			n++;
		}
	}


	//for new_scene_L_shape_idx
	for (auto i : cutResult.L_shape_plane_idx)
	{
		std::pair<int, int> newID;
		for (auto j : new_rewrite_idx)
		{
			if (j.first == i.first)
			{
				newID.first = j.second;
			}
			if (j.first == i.second)
			{
				newID.second = j.second;
			}
		}
		new_scene_L_shape_idx.push_back(newID);
	}

	//for new_scene_parallel_idx
	for (auto i : cutResult.parallel_plane_idx)
	{
		std::pair<int, int> newID;
		for (auto j : new_rewrite_idx)
		{
			if (j.first == i.first)
			{
				newID.first = j.second;
			}
			if (j.first == i.second)
			{
				newID.second = j.second;
			}
		}
		new_scene_parallel_idx.push_back(newID);
	}

	new_door_window_info.resize(new_scene_plane_xyz.size());
	for (size_t i = 0; i < cutResult.door_window_info.size(); i++)
	{
		for (auto IDpair : new_rewrite_idx)
		{
			if (IDpair.first == i)
			{
				int newid = IDpair.second;
				for (size_t j = 0; j < cutResult.door_window_info[i].size(); j++)
				{
					cutResult.door_window_info[i][j].wallId = newid;
				}
				new_door_window_info[newid] = cutResult.door_window_info[i];
				break;
			}
		}
	}

	cutResult.plane_wall_idx.swap(new_scene_wall_idx);
	cutResult.plane_ceiling_idx.swap(new_scene_ceiling_idx);
	cutResult.plane_ground_idx.swap(new_scene_ground_idx);
	cutResult.plane_beam_idx.swap(new_scene_beam_idx);
	cutResult.L_shape_plane_idx.swap(new_scene_L_shape_idx);
	cutResult.parallel_plane_idx.swap(new_scene_parallel_idx);
	cutResult.plane_xyz.swap(new_scene_plane_xyz);
	cutResult.plane_reflect.swap(new_scene_plane_reflect);
	cutResult.plane_center.swap(new_scene_plane_center);
	cutResult.plane_normals.swap(new_scene_plane_normals);
	cutResult.door_window_info.swap(new_door_window_info);
	cutResult.plane_area.swap(new_plane_area);
}
