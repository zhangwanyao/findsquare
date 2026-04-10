#ifndef _UTIL_LINE_DIST_HPP_
#define _UTIL_LINE_DIST_HPP_

#include <vector>
#include <string>
#include "InOutData.h"
#include "util_math.hpp"

/**
* \brief line seg elements after merging in 3D space
*/
struct Line3DItem
{
	/**
	* \brief line center
	*/
	ModuleStruct::Point3f line_center;

	/**
	* \brief line direction
	*/
	ModuleStruct::Point3f line_direction;


	/**
	* \brief start point of line
	*/
	ModuleStruct::Point3f line_seg_start;


	/**
	* \brief end point of line
	*/
	ModuleStruct::Point3f line_seg_end;
};

/**
* \brief door or window group have two pair lines, two lines for width direction, and two lines for height direction
*/
struct DWGroupLines
{
	/**
	* \brief door or window group name
	*/
	std::string group_name;
	/**
	* \brief door or window width direction line pair
	*/
	unsigned int width_line[2];
	/**
	* \brief door or window height direction line pair
	*/
	unsigned int height_line[2];
};



struct LinePlaneMse
{
	unsigned int plane_idx;
	float plane_mse;
};
namespace util_line_dist
{
// dot product (3D)
#define dot(u,v)   (u.x * v.x + u.y * v.y + u.z * v.z)
#define norm(v)     sqrt(dot(v,v))     // norm = length of  vector
#define d(u,v)      norm(u-v)          // distance = norm of difference

#define SMALL_NUM   0.0000001 // anything that avoids division overflow
#define perp(u,v)  (u.x * v.y - u.y * v.x)  // perp product  (2D)
#define SMALL_DIST_RATIO   0.001 // small dist



	/**
	* \brief determine if a point is inside a segment, reference: http://geomalgorithms.com/a05-_intersect-1.html#intersect2D_2Segments()
	* @param [in] P, point
	* @param [in] S, a collinear segment
	* return   1 = P is inside S, 0 = P is  not inside S
	* */
	static inline int inSegment(ModuleStruct::Point3f P, Line3DItem &S)
	{
		if (S.line_seg_start.x != S.line_seg_end.x) {    // S is not  vertical
			if (S.line_seg_start.x <= P.x && P.x <= S.line_seg_end.x)
				return 1;
			if (S.line_seg_start.x >= P.x && P.x >= S.line_seg_end.x)
				return 1;
		}
		else {    // S is vertical, so test y  coordinate
			if (S.line_seg_start.y <= P.y && P.y <= S.line_seg_end.y)
				return 1;
			if (S.line_seg_start.y >= P.y && P.y >= S.line_seg_end.y)
				return 1;
		}
		return 0;
	}

	/**
	* \brief find the 2D intersection of 2 lines, reference: http://geomalgorithms.com/a05-_intersect-1.html#intersect2D_2Segments()
	* @param [in] is_infinite, infinite segment
	* @param [in] S1, first finite segment
	* @param [in] S2, second finite segment
	* @param [out] I0, intersect point (when it exists)
	* @param [out] I1, endpoint of intersect segment [I0,I1] (when it exists)
	* return   0=disjoint (no intersect), 1=intersect  in unique point I0,2=overlap  in segment from I0 to I1
	* */
	static inline int intersect2D_2Segments(bool is_infinite, Line3DItem S1, Line3DItem S2, ModuleStruct::Point3f* I0, ModuleStruct::Point3f* I1)
	{
		//Vector    u = S1.P1 - S1.P0;
		//Vector    v = S2.P1 - S2.P0;
		//Vector    w = S1.P0 - S2.P0;

		ModuleStruct::Point3f u = S1.line_seg_end - S1.line_seg_start;
		ModuleStruct::Point3f v = S2.line_seg_end - S2.line_seg_start;
		ModuleStruct::Point3f w = S1.line_seg_start - S2.line_seg_start;
		float     D = perp(u, v);
		// test if  they are parallel (includes either being a point)
		if (fabs(D) < SMALL_NUM) {           // S1 and S2 are parallel
			if (perp(u, w) != 0 || perp(v, w) != 0) {
				return 0;                    // they are NOT collinear
			}
			// they are collinear or degenerate
			// check if they are degenerate  points
			float du = dot(u, u);
			float dv = dot(v, v);
			if (du == 0 && dv == 0) {            // both segments are points
				if (S1.line_seg_start != S2.line_seg_start)         // they are distinct  points
					return 0;
				*I0 = S1.line_seg_start;                 // they are the same point
				return 1;
			}
			if (du == 0) {                     // S1 is a single point
				if (inSegment(S1.line_seg_start , S2) == 0)  // but is not in S2
					return 0;
				*I0 = S1.line_seg_start;
				return 1;
			}
			if (dv == 0) {                     // S2 a single point
				if (inSegment(S2.line_seg_start, S1) == 0)  // but is not in S1
					return 0;
				*I0 = S2.line_seg_start;
				return 1;
			}
			// they are collinear segments - get  overlap (or not)
			float t0, t1;                    // endpoints of S1 in eqn for S2
			ModuleStruct::Point3f  w2 = S1.line_seg_end - S2.line_seg_start;
			if (v.x != 0) {
				t0 = w.x / v.x;
				t1 = w2.x / v.x;
			}
			else {
				t0 = w.y / v.y;
				t1 = w2.y / v.y;
			}
			if (t0 > t1) {                   // must have t0 smaller than t1
				float t = t0; t0 = t1; t1 = t;    // swap if not
			}
			if (t0 > 1 || t1 < 0) {
				return 0;      // NO overlap
			}
			t0 = t0 < 0 ? 0 : t0;               // clip to min 0
			t1 = t1 > 1 ? 1 : t1;               // clip to max 1
			if (t0 == t1) {                  // intersect is a point
				*I0 = S2.line_seg_start + t0 * v;
				return 1;
			}

			// they overlap in a valid subsegment
			*I0 = S2.line_seg_start + t0 * v;
			*I1 = S2.line_seg_start + t1 * v;
			return 2;
		}

		// the segments are skew and may intersect in a point
		// get the intersect parameter for S1
		float     sI = perp(v, w) / D;

		if (!is_infinite)
		{
			if (sI < 0 || sI > 1)                // no intersect with S1
				return 0;
		}
		// get the intersect parameter for S2
		float     tI = perp(u, w) / D;


		if (!is_infinite)
		{
			if (tI < 0 || tI > 1)                // no intersect with S2
				return 0;
		}

		*I0 = S1.line_seg_start + sI * u;                // compute S1 intersect point
		if (!is_infinite)
		{
			return 1;
		}
		else
		{
			// if is infinite segments, must check two 
			ModuleStruct::Point3f s2_intrsc = S2.line_seg_start + tI * v;                // compute S2 intersect point
			ModuleStruct::Point3f delt_point = s2_intrsc - *I0;
			double short_segments_length = norm(u) > norm(v) ? norm(u) : norm(v);
			double delt_point_ratio = norm(delt_point)/short_segments_length;
			if (delt_point_ratio < SMALL_DIST_RATIO)
			{
				*I0 = (*I0 + s2_intrsc) / 2;
				return 1;
			}
			else
			{
				log_info("2line intersect dist ratio %f> max ratio %f", delt_point_ratio, SMALL_DIST_RATIO);
				return 0;
			}
		}
    }

	/**
	* \brief get the distance of a point to a line,reference: http://geomalgorithms.com/a02-_lines.html
	* @param [in] P, a Point 
	* @param [in] LP0, first Point of line
	* @param [in] LP1, second Point of line
	* return  dist of P to line (P0,P1)
	* */
	static inline float dist_point_to_line(ModuleStruct::Point3f P, ModuleStruct::Point3f LP0, ModuleStruct::Point3f LP1)
	{
		ModuleStruct::Point3f v = LP1 - LP0;
		ModuleStruct::Point3f w = P - LP0;

		double c1 = dot(w, v);			
		double c2 = dot(v, v);
		double b = c1 / c2;
		ModuleStruct::Point3f Pb = LP0 + b * v;
		return static_cast<float>(Util_Math::ComputePointToPointDist<double, ModuleStruct::Point3f>(P, Pb));
		//return d(P, Pb);
	}

	static inline ModuleStruct::Point3f vec3_normalize(ModuleStruct::Point3f vec) {
		float len = sqrtf(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
		return (fabs(len) < numeric_limits<float>::epsilon()) ? vec
			: ModuleStruct::Point3f(vec.x / len, vec.y / len, vec.z / len);
	}

	static float dist_2lines(Line3DItem&line0, Line3DItem&line1)
	{
		//float dist0 = Util_Math::ComputePointToLineDist<float, ModuleStruct::Point3f>(line0.line_seg_start, line1.line_center,line1.line_direction);
		//float dist1 = Util_Math::ComputePointToLineDist<float, ModuleStruct::Point3f>(line0.line_seg_end, line1.line_center, line1.line_direction);
		//float dist2 = Util_Math::ComputePointToLineDist<float, ModuleStruct::Point3f>(line0.line_center, line1.line_center, line1.line_direction);

		//float dist3 = Util_Math::ComputePointToLineDist<float, ModuleStruct::Point3f>(line1.line_seg_start, line0.line_center, line0.line_direction);
		//float dist4 = Util_Math::ComputePointToLineDist<float, ModuleStruct::Point3f>(line1.line_seg_end, line0.line_center, line0.line_direction);
		//float dist5 = Util_Math::ComputePointToLineDist<float, ModuleStruct::Point3f>(line1.line_center, line0.line_center, line0.line_direction);

		float dist0 = dist_point_to_line(line0.line_seg_start, line1.line_seg_start,line1.line_seg_end);
		float dist1 = dist_point_to_line(line0.line_seg_end, line1.line_seg_start, line1.line_seg_end);
		float dist2 = dist_point_to_line(line0.line_center, line1.line_seg_start, line1.line_seg_end);

		float dist3 = dist_point_to_line(line1.line_seg_start, line0.line_seg_start, line0.line_seg_end);
		float dist4 = dist_point_to_line(line1.line_seg_end, line0.line_seg_start, line0.line_seg_end);
		float dist5 = dist_point_to_line(line1.line_center, line0.line_seg_start, line0.line_seg_end);
		//log_debug("line[%d,%d] dist0~5 =[%f,%f,%f,%f,%f,%f", line0, line1,dist0, dist1, dist2, dist3, dist4, dist5);
		return (dist0 + dist1 + dist2 + dist3 + dist4 + dist5)/6;
	}
	/**
	* \brief get line distance input file path by input parameters
	* @param [in] argc, number of input parameters
	* @param [in] argv, pointer of input data address
	* @param [out] output_path,  out put path, with endof "/" or"\\"
	* @param [out] input_data_file, input file path+name
	* @param [out] line_strs,  input line string array
	* return 0 is ok, <0 is error
	* */
	static inline int line_dist_arc_parse(const int argc, char** argv, std::string& input_data_file,\
		std::string& output_path,std::string&line_input_file)
	{
		if (IOData::parse_argument(argc, argv, "-i", input_data_file) <= 0)
		{
			printf("No input_data_file given\n");
			return -1;
		}

		if (IOData::parse_argument(argc, argv, "-o", output_path) <= 0)
		{
			printf("No Output Path given\n");
			return -1;
		}

		if (IOData::parse_argument(argc, argv, "-l", line_input_file) <= 0)
		{
			printf("No Output Path given\n");
			return -1;
		}

		//int line_cnt = 0;
		//for (int i = 1; i < argc; ++i)
		//{
		//	std::string tmp_str = argv[i];
		//	bool find_con = (std::string::npos != (tmp_str.find("door_width_"))) || (std::string::npos != (tmp_str.find("door_height")));
		//	find_con = find_con||(std::string::npos != (tmp_str.find("window_width_"))) || (std::string::npos != (tmp_str.find("windor_height")));
		//	if (find_con)
		//	{
		//		line_cnt++;
		//		if (line_cnt >= 6) break;
		//	}
		//}

		//if (line_cnt < 2)
		//{
		//	printf("have no enough line input\n");
		//	return -1;
		//}


		//if (line_cnt%2 != 0)
		//{
		//	printf("input error odd lines input\n");
		//}

		//line_strs.resize(line_cnt);
		//line_cnt = 0;
		//for (int i = 1; i < argc; ++i)
		//{
		//	std::string tmp_str = argv[i];
		//	bool find_con = (std::string::npos != (tmp_str.find("door_width_"))) || (std::string::npos != (tmp_str.find("door_height")));
		//	find_con = find_con || (std::string::npos != (tmp_str.find("window_width_"))) || (std::string::npos != (tmp_str.find("windor_height")));
		//	if (find_con)
		//	{
		//		line_strs[line_cnt].assign(tmp_str);
		//		line_cnt++;
		//		if (line_cnt >= 8) break;
		//	}
		//}		
		return 0;
	}


	// load line segment from.txt file
	static inline bool LoadLineInput(const string path_name, std::vector<std::string> &line_strs, std::vector<std::vector<unsigned int>>&line_idx, unsigned int&groud_plane_idx)
	{
		FILE* pFile;
#ifdef WIN32
		errno_t err = fopen_s(&pFile, path_name.c_str(), "r");
#else
		pFile = fopen(path_name.c_str(), "r");
#endif

		if (!pFile)
		{
			cout << "IOData::LoadLineInput: cannot open file" << endl;
			log_fatal("cannot open %s", path_name.c_str());
			return false;
		}

		int raw_data_row_num = 0;
		IOData::Get3DPtCloudRowNum(path_name, raw_data_row_num);

		if (raw_data_row_num <= 0) {
			cout << "LoadLineInput: Get3DPtCloudRowNum() error" << endl;
			log_fatal("NO row number at %s", path_name.c_str());
			return false;
		}

		line_strs.resize(raw_data_row_num);
		line_idx.resize(raw_data_row_num);

		//#pragma omp parallel for
		for (int i = 0; i < static_cast<int>(line_idx.size()); i++)
		{
			line_idx[i].resize(2);
		}

		int row_idx = 0;
		char line_string[20];
		while (!feof(pFile) && row_idx < raw_data_row_num)
		{
			int para_cnt = FSCANF(pFile, "%s%d%d%*[^\n]%*c", line_string, 20, &line_idx[row_idx][0], &line_idx[row_idx][1]);
			if (para_cnt == 3)
			{
				line_strs[row_idx].assign(line_string);
				//log_info("line_strs[%d] =%s line_idx[%d][%d,%d]", row_idx, line_strs[row_idx].c_str(), row_idx,line_idx[row_idx][0],line_idx[row_idx][1]);
				row_idx++;
			}
			else if(para_cnt == 2)
			{
				std::string tmp_str;
				tmp_str.assign(line_string);
				if (tmp_str.find("ground_plane") != std::string::npos)
				{
					groud_plane_idx = line_idx[row_idx][0];
				}
			}

		}
		fclose(pFile);
		return true;
	}

	static inline bool GetGroupFromInputlines(std::vector<std::string>& line_strs, std::vector<std::vector<unsigned int>>& line_idx_array, \
		std::map<std::string,std::map<std::string,std::vector<unsigned int>>> &group_lines)
	{

		std::map<std::string, std::map<std::string, std::vector<unsigned int>>>::iterator config_map_it; // current iterator of group lines
		std::map<std::string, std::vector<unsigned int>> config_kv; // current key and value per group
		std::map<std::string, std::vector<unsigned int>>::iterator config_kv_it;
		const int invalid_line_idx = std::numeric_limits<unsigned int>::max();
		int group_idx = 0;
		int config_idx = 0;
		for (size_t i = 0; i < line_strs.size(); i++)
		{
			bool dbg_con = i < 5;
			if (line_strs[i].empty())
			{
				log_info("line_strs[%d] is empty", i);
				continue;
			}
			unsigned int line0 = line_idx_array[i][0];
			unsigned int line1 = line_idx_array[i][1];
			std::string *cur_dist_str = &line_strs[i];
			size_t ipos = cur_dist_str->find_last_of('_');
			std::string cur_group_name = cur_dist_str->substr(0, ipos);
			std::string cur_group_type = cur_dist_str->substr(ipos+1, cur_dist_str->length());
			////if have no group name, add new group
			//config_kv.clear();
			//config_kv[cur_group_type].resize(2);
			//config_kv[cur_group_type][0] = line0;
			//config_kv[cur_group_type][1] = line1;
			//group_lines.insert(std::make_pair(cur_group_name, config_kv));
			//config_map_it->second = config_kv;

			config_map_it = group_lines.find(cur_group_name);
			if (config_map_it == group_lines.end())
			{
				//if have no group name, add new group
				config_kv.clear();
				group_lines.insert(std::make_pair(cur_group_name, config_kv));
				//log_info("cur_group_name =%s", cur_group_name.c_str());

				config_kv[cur_group_type].resize(2);
				config_kv[cur_group_type][0] = line0;
				config_kv[cur_group_type][1] = line1;

				config_kv_it = config_kv.find(cur_group_type);
				//log_info(" cur_group_type =%s onfig_kv_it->first =%s second = [%d,%d]", cur_group_type.c_str(), config_kv_it->first.c_str(), config_kv_it->second[0], config_kv_it->second[1]);
				config_map_it = group_lines.find(cur_group_name);
				config_map_it->second = config_kv;
			}
			else
			{
				//if (dbg_con) log_info("GetGroupFromInputlines i =%d cur_group_name =%d", i, cur_group_name.c_str());
				config_kv = config_map_it->second;
				config_kv_it = config_kv.find(cur_group_type);
				if (config_kv_it == config_kv.end())
				{
					config_kv[cur_group_type].resize(2);
					config_kv[cur_group_type][0] = line0;
					config_kv[cur_group_type][1] = line1;
					//if(dbg_con) log_info("config_map_it->first =%s  cur_group_type =%s second = [%d,%d]", config_map_it->first.c_str(),  cur_group_type.c_str(), line0, line1);
				}
				config_map_it->second = config_kv;
			}
		}
		return true;
	}

	static inline float GetLineToGroudDist(const unsigned int line_idx, const unsigned int groud_plane_idx,\
		std::vector<Line3DItem>& lines,std::string output_path, float&dist)
	{
		std::stringstream line_id;
		line_id << groud_plane_idx;
		std::string plane_center_normal_file = output_path+"plane_center_normal\\"+"plane_center_normal"+ line_id.str()+".txt";

		ModuleStruct::Point3Array plane_normal_arr;
		ModuleStruct::Point3Array plane_center_arr;
		if (!IOData::Load3DPtCloudDataWithNormal(plane_center_normal_file, plane_center_arr, plane_normal_arr))
		{
			std::cout << "can not open file: " << plane_center_normal_file << std::endl;
			return  false;
		}
		ModuleStruct::Point3f plane_center = plane_center_arr[0];
		ModuleStruct::Point3f plane_normal = plane_normal_arr[0];
		float dist0 = Util_Math::ComputePointToPlaneDist<float, ModuleStruct::Point3f>(lines[line_idx].line_seg_start, plane_normal, plane_center);
		float dist1 = Util_Math::ComputePointToPlaneDist<float, ModuleStruct::Point3f>(lines[line_idx].line_seg_end, plane_normal, plane_center);
		float dist2 = Util_Math::ComputePointToPlaneDist<float, ModuleStruct::Point3f>(lines[line_idx].line_center, plane_normal, plane_center);
		dist = (dist0 + dist1 + dist2)/3;
		return true;
	}

	static bool get_lines_plane_mse(std::string output_path, std::vector<LinePlaneMse>&line_plane_mse_arr)
	{
		std::string filename = output_path + "lines_plane_mse.txt";
		FILE* pFile;
#ifdef WIN32
		errno_t err = fopen_s(&pFile, filename.c_str(), "r");
#else
		pFile = fopen(path_name.c_str(), "r");
#endif

		if (!pFile)
		{
			cout << "IOData::get_lines_plane_mse: cannot open file" << endl;
			log_fatal("cannot open %s", filename.c_str());
			return false;
		}

		int raw_data_row_num = 0;
		IOData::Get3DPtCloudRowNum(filename, raw_data_row_num);
		int row_idx = 0;
		unsigned int line_idx;
		line_plane_mse_arr.clear();
		line_plane_mse_arr.resize(raw_data_row_num);
		while (!feof(pFile) && row_idx < raw_data_row_num)
		{
			if (FSCANF(pFile, "%d%d%f%*[^\n]%*c", &line_idx, &line_plane_mse_arr[row_idx].plane_idx, &\
				line_plane_mse_arr[row_idx].plane_mse) ==3)
			{
				row_idx++;
			}
		}
		fclose(pFile);	
		return true;
	}

	static inline bool ComputeGroupLineDist(std::string output_path, std::map<std::string, std::map<std::string, std::vector<unsigned int>>>& group_lines,\
		std::vector<Line3DItem>&lines,unsigned int groud_plane_idx)
	{
		std::map<std::string, std::map<std::string, std::vector<unsigned int>>>::iterator config_map_it; // current iterator of group lines
		std::map<std::string, std::vector<unsigned int>>::iterator config_kv_it; // current key and value per group		
		std::map<std::string, std::vector<unsigned int>> config_kv;

		std::vector<LinePlaneMse> lines_plane_mse;
		if (!get_lines_plane_mse(output_path, lines_plane_mse))
		{
			log_error("get_lines_plane_mse error");
			return false;
		}

		if (lines_plane_mse.size() != lines.size())
		{
			log_error("get_lines_plane_mse input error lines_plane_mse size =%d lines.size() =%d", lines_plane_mse.size(), lines.size());
			return false;
		}

		for (config_map_it = group_lines.begin(); config_map_it != group_lines.end(); config_map_it++)
		{
			int width_line0, width_line1, height_line0, height_line1;
			float width_calib = 0.f;
			float height_calib = 0.f;
			std::string group_method_name = config_map_it->first;
			size_t ipos = group_method_name.find_first_of('_');
			std::string group_name = group_method_name.substr(0, ipos);
			std::string group_method = group_method_name.substr(ipos + 1, group_method_name.length());
			width_line0 = width_line1 = height_line0 = height_line1 = std::numeric_limits<int>::max();
			ModuleStruct::Point3f itsc0[2], itsc1[2], itsc2[2], itsc3[2];
			config_kv = config_map_it->second;
			// get width line
			config_kv_it = config_kv.find("width");
			if (config_kv_it != config_kv.end())
			{
				//log_info("config_kv_it =%s ", config_kv_it->first.c_str());
				width_line0 = config_kv[config_kv_it->first][0];
				width_line1 = config_kv[config_kv_it->first][1];
				if (group_name.find("window") != group_name.npos)
				{
					width_calib = lines_plane_mse[width_line0].plane_mse *2;
				}
				else if (group_name.find("door") != group_name.npos)
				{
					width_calib = lines_plane_mse[width_line0].plane_mse ;
				}
			}
			// get height line
			config_kv_it = config_kv.find("height");
			if (config_kv_it != config_kv.end())
			{
				//log_info("cnt =%d 02 config_kv_it =%s ", cnt, config_kv_it->first.c_str());
				height_line0 = config_kv[config_kv_it->first][0];
				height_line1 = config_kv[config_kv_it->first][1];
				if (group_name.find("window") != group_name.npos)
				{
					height_calib = lines_plane_mse[height_line0].plane_mse * 2;
				}
			}
			// check if have four lines available
			bool line_available = (width_line0 < lines.size()) && (width_line1 < lines.size()) && (height_line0 < lines.size())\
				&& (width_line1 < lines.size());

			//log_info("cnt =%d 04 width_line=[%d,%d] height_line =[%d,%d] ", cnt, width_line0, width_line1, height_line0, height_line1);
			if (line_available)
			{
				// get intersect four points of  four lines
				int rtn_value0 = intersect2D_2Segments(true,lines[width_line0], lines[height_line0], &itsc0[0], &itsc0[1]);
				log_info("%s line[%d,%d] itsc0[0] =[%f,%f,%f]", config_map_it->first.c_str(), width_line0, height_line0, itsc0[0].x, itsc0[0].y, itsc0[0].z);
				int rtn_value1 = intersect2D_2Segments(true, lines[width_line0], lines[height_line1], &itsc1[0], &itsc1[1]);
				log_info("%s line[%d,%d] itsc1[0] =[%f,%f,%f]", config_map_it->first.c_str(), width_line0, height_line1, itsc1[0].x, itsc1[0].y, itsc1[0].z);
				int rtn_value2 = intersect2D_2Segments(true, lines[width_line1], lines[height_line0], &itsc2[0], &itsc2[1]);
				log_info("%s line[%d,%d] itsc2[0]  =[%f,%f,%f]", config_map_it->first.c_str(), width_line1, height_line0, itsc2[0].x, itsc2[0].y, itsc2[0].z);
				int rtn_value3 = intersect2D_2Segments(true, lines[width_line1], lines[height_line1], &itsc3[0], &itsc3[1]);
				log_info("%s line[%d,%d] itsc3[0]  =[%f,%f,%f]", config_map_it->first.c_str(), width_line1, height_line1, itsc3[0].x, itsc3[0].y, itsc3[0].z);
				log_info("rtn_value = [%d,%d,%d,%d]", rtn_value0, rtn_value1, rtn_value2, rtn_value3);
				// computer center, width center from [itsc0,itsc1] and  [itsc2,itsc3] , height center from [itsc0,itsc2] and  [itsc1,itsc3]
				bool cond = (rtn_value0 == 1) && (rtn_value1 == 1) && (rtn_value0 == 1) && (rtn_value0 == 1);
				if (cond && group_method == "1")
				{
					ModuleStruct::Point3f width_center0, width_center1, height_center0, height_center1;
					width_center0 = (itsc0[0] + itsc1[0]) / 2;
					width_center1 = (itsc2[0] + itsc3[0]) / 2;
					height_center0 = (itsc0[0] + itsc2[0]) / 2;
					height_center1 = (itsc1[0] + itsc3[0]) / 2;
					//compute window or door center dist from scanner
					ModuleStruct::Point3f center = (width_center0 + width_center1 + height_center0 + height_center1) / 4;
					//double center_dist = Util_Math::vec3_len<ModuleStruct::Point3f>(center);
					double center_dist = norm(center);
					log_info("%s center_dist = %f", config_map_it->first.c_str(), center_dist);

					// computer three width dist,  width from [itsc0,itsc2] and  [itsc1,itsc3] , [width_center0,width_center1]
					float cur_width0 = Util_Math::ComputePointToPointDist<float, ModuleStruct::Point3f>(itsc0[0], itsc2[0]);
					float cur_width1 = Util_Math::ComputePointToPointDist<float, ModuleStruct::Point3f>(itsc1[0], itsc3[0]);
					float cur_width2 = Util_Math::ComputePointToPointDist<float, ModuleStruct::Point3f>(width_center0, width_center1);
					float cur_width = (cur_width0 + cur_width1 + cur_width2) / 3;
					float cur_width_cal = cur_width + width_calib;
					log_info("method1 %s  line[%d,%d] width = %f width_calib =%f", config_map_it->first.c_str(), width_line0, width_line1, cur_width, cur_width_cal);
					float cur_height0 = Util_Math::ComputePointToPointDist<float, ModuleStruct::Point3f>(itsc0[0], itsc1[0]);
					float cur_height1 = Util_Math::ComputePointToPointDist<float, ModuleStruct::Point3f>(itsc2[0], itsc3[0]);
					float cur_height2 = Util_Math::ComputePointToPointDist<float, ModuleStruct::Point3f>(height_center0, height_center1);
					float cur_height = (cur_height0 + cur_height1 + cur_height2) / 3;
					float cur_height_cal = cur_height + height_calib;
					log_info("method1 %s line[%d,%d] height = %f height_calib =%f", config_map_it->first.c_str(), height_line0, height_line1, cur_height, cur_height_cal);
					
					std::string save_file = output_path + config_map_it->first.c_str() + ".xyz";
					std::vector<ModuleStruct::Point3f>point_array{ itsc0[0] ,itsc1[0] ,itsc2[0] ,itsc3[0] ,width_center0, width_center1,height_center0,height_center1 };
					IOData::SavePoint3fData(save_file, point_array);
				}
				else
				{
					ModuleStruct::Point3f center = (lines[width_line0].line_center + lines[width_line1].line_center + lines[height_line0].line_center + lines[height_line1].line_center) / 4;
					//double center_dist = Util_Math::vec3_len<ModuleStruct::Point3f>(center);
					double center_dist = norm(center);
					log_info("%s center_dist = %f", config_map_it->first.c_str(), center_dist);
				}

				if (group_method == "2")
				{
					//unsigned int up_door_line = lines[height_line0].line_center.z > lines[height_line1].line_center.z ? height_line0 : height_line1;

					if (groud_plane_idx == -1)
					{
						log_info("method2 %s can not find ground plane index", config_map_it->first.c_str());
						continue;
					}
					float dist0 = 0.f;
					float dist1 = 0.f;

					bool retn = GetLineToGroudDist(height_line0, groud_plane_idx, lines, output_path, dist0);
					if (!retn)
					{
						log_info("method2 %s get height_line %d height error", config_map_it->first.c_str(), height_line0);
					}
					retn = GetLineToGroudDist(height_line1, groud_plane_idx, lines, output_path, dist1);
					if (!retn)
					{
						log_info("method2 %s get height_line %d height error", config_map_it->first.c_str(), height_line1);
					}

					int up_door_line = dist1 > dist0 ? height_line1 : height_line0;
					float dist = dist1 > dist0 ? dist1 : dist0;
					log_info("method2 %s line[%d,%d] upline %d height = %f", config_map_it->first.c_str(), height_line0, height_line1, up_door_line, dist);
				}
			}

		}		
		return true;	
	}

	
}


#endif /*_UTIL_LINE_DIST_HPP_*/
