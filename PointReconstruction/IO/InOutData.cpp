#include "InOutData.h"
#include "log.h"
#include <locale>
#include <codecvt>
#include <iomanip>

#ifdef _WIN32
#include <direct.h>
#include <io.h>
#else
#include "VariableDefine.h"
#include <sys/stat.h>
#include <unistd.h>
#endif

#ifdef _WIN32
#define ACCESS _access
#define MKDIR(a) _mkdir((a))
#else
#define ACCESS access
#define MKDIR(a) mkdir((a),0755)
#endif

#include "opencv2/imgcodecs.hpp"

//save point3f data to .txt file
bool IOData::SavePoint3fData(const string path_name, const ModuleStruct::Point3 &input_data) {

	if (path_name.empty()) {
		cout << "IOData::SavePoint3fData: empty file path" << endl;
		log_fatal("empty filepath %s", path_name.c_str());
		return false;
	}

	ofstream fout(path_name);
	if (fout.fail()) {
		cout << "IOData::SavePoint3fData: cannot open file" << endl;
		log_fatal("cannot open %s", path_name.c_str());
		return false;
	}

	fout << input_data.x << '\t' << input_data.y << '\t' << input_data.z;
	
	fout.close();

	return true;
}

//save point3f data to .txt file
bool IOData::SavePoint3fData(const string path_name, const ModuleStruct::Point3Array &input_data, const ModuleStruct::Point3Array &input_normal){

	if (input_data.empty()) {
		cout << "IOData::SavePoint3fData: empty data" << endl;
		return false;
	}

	if (path_name.empty()) {
		cout << "IOData::SavePoint3fData: empty file path" << endl;
		return false;
	}

	ofstream fout(path_name);
	if (fout.fail()) {		
		log_fatal("cannot open %s", path_name.c_str());
		return false;
	}

	fout << setprecision(16);
	for (int i = 0; i < input_data.size(); i ++){
		fout << input_data[i].x << '\t' << input_data[i].y << '\t' << input_data[i].z << '\t' << input_normal[i].x << '\t' << input_normal[i].y << '\t' << input_normal[i].z;
		if (i < input_data.size() - 1) {fout << endl;}
	}
	fout.close();

	return true;
}

//save point3f data to .txt file
bool IOData::SavePoint3fData(const string path_name, const ModuleStruct::Point3Array& input_data) {

	if (input_data.empty()) {
		cout << "IOData::SavePoint3fData: empty data" << endl;
		return false;
	}

	if (path_name.empty()) {
		cout << "IOData::SavePoint3fData: empty file path" << endl;
		return false;
	}

	ofstream fout(path_name);
	if (fout.fail()) {
		log_fatal("cannot open %s", path_name.c_str());
		return false;
	}

	for (int i = 0; i < input_data.size(); i++) {
		fout << input_data[i].x << '\t' << input_data[i].y << '\t' << input_data[i].z;
		if (i < input_data.size() - 1) { fout << endl; }
	}
	fout.close();

	return true;
}

//save point3f data to .txt file
bool IOData::SavePoint3fData(const string path_name, const ModuleStruct::Vector<ModuleStruct::Point3Array> &input_data) {
	
	if (input_data.empty()) {
		cout << "IOData::SavePoint3fData: empty data" << endl;
		return false;
	}

	if (path_name.empty()) {
		cout << "IOData::SavePoint3fData: empty file path" << endl;
		return false;
	}

	ofstream fout(path_name);
	if (fout.fail()) {
		cout << "IOData::SavePoint3fData: cannot open file" << endl;
		return false;
	}

	for (int i = 0; i < input_data.size(); i++) {
		for (int j = 0; j < input_data[i].size(); j++) {
			fout << input_data[i][j].x << '\t' << input_data[i][j].y << '\t' << input_data[i][j].z;
			if (j < input_data[i].size() - 1) {
				fout << endl;
			}
		}
		if (i < input_data.size() - 1)
			fout << endl;
	}
	fout.close();

	return true;
}

//save point3d data to .txt file
bool IOData::SavePoint3dData(const string path_name, const ModuleStruct::Point3dArray &input_data){
	
	if (input_data.empty()) {
		cout << "IOData::SavePoint3fData: empty data" << endl;
		return false;
	}

	if (path_name.empty()) {
		cout << "IOData::SavePoint3fData: empty file path" << endl;
		return false;
	}

	ofstream fout(path_name);
	if (fout.fail()) {
		cout << "IOData::SavePoint3fData: cannot open file" << endl;
		return false;
	}
	for (int i = 0; i < input_data.size(); i++) {
		fout << input_data.at(i).x << '\t' << input_data.at(i).y << '\t' << input_data.at(i).z;
		if (i < input_data.size() - 1) { fout << endl; }
	}

	return true;
}

bool IOData::SavePoint3fDataWithDelimiter(const string path_name, const string delimiter, const ModuleStruct::Point3Array &point_array, const ModuleStruct::Vector<unsigned int> &point_in_voxel_array) {

	if (point_array.size() == 0) {
		cout << "IOData::SavePoint3fData: empty point_array" << endl;
		return false;
	}

	if (point_array.empty()) {
		cout << "IOData::SavePoint3fData: point_array pointer is NULL" << endl;
		return false;
	}

	if (point_in_voxel_array.size() == 0) {
		cout << "IOData::SavePoint3fData: empty point_in_voxel_array" << endl;
		return false;
	}

	if (point_in_voxel_array.empty()) {
		cout << "IOData::SavePoint3fData: point_in_voxel_array pointer is NULL" << endl;
		return false;
	}

	if (path_name.empty()) {
		cout << "IOData::SavePoint3fData: empty file path" << endl;
		return false;
	}

	ofstream fout(path_name);
	if (fout.fail()) {
		log_fatal("cannot open %s", path_name.c_str());
		return false;
	}

	unsigned int point_idx;
	ModuleStruct::Point3f point;
	for (unsigned int i = 0; i < point_in_voxel_array.size(); i++) {

		point_idx = point_in_voxel_array[i];
		point = point_array[point_idx];
		fout << point.x << delimiter << point.y << delimiter << point.z;
		if (i < point_in_voxel_array.size() - 1) {
			fout << endl;
		}
	}
	fout.close();
	return true;
}


bool IOData::SavePoint3fDataWithDelimiter(const std::string path_name, const std::string delimiter, const ModuleStruct::Vector<ModuleStruct::Point3>& input_data) {


	if (input_data.size() == 0) {
		std::cout << "IOData::SavePoint3fData: empty point_in_voxel_array" << std::endl;
		return false;
	}

	if (path_name.empty()) {
		std::cout << "IOData::SavePoint3fData: empty file path" << std::endl;
		return false;
	}

	std::ofstream fout(path_name);

	if (fout.fail()) {
		log_fatal("cannot open %s", path_name.c_str());
		return false;
	}
	ModuleStruct::Point3f point;
	for (unsigned int i = 0; i < input_data.size(); i++) {

		//point_idx = input_data[i];
		point = input_data[i];
		fout << point.x << delimiter << point.y << delimiter << point.z;
		if (i < input_data.size() - 1) {
			fout << std::endl;
		}
	}
	fout.close();
	return true;
}

//get number of rows of 3d point cloud (N x 3)
bool IOData::Get3DPtCloudRowNum(const string path_name, int &row_num){
	FILE *pFile;
#ifdef WIN32
	errno_t err = fopen_s(&pFile, path_name.c_str(), "r");
#else
	pFile = fopen(path_name.c_str(), "r");
#endif
	if (!pFile)
	{
		cout << "IOData::Load3DPtCloudData: cannot open file" << endl;
		return false;
	}
	row_num = 0;
	//string line;
	char line[256] = { 0 };
	while(!feof(pFile)){
		if(fgets(line, sizeof(line), pFile))
			row_num ++;

		/* revised by Tang Xueyan @08/11/2018 */
		//size_t pos = 0;
		//while ((pos = line.find(' ') || (pos = line.find('\n'))) != string::npos) {
		//	pos++;
		//}
		//if (pos++ == 3)
		//	row_num++;
	}
	fclose(pFile);

	return true;
}

//load 3d point cloud from .txt file
bool IOData::Load3DPtCloudData(const string path_name, ModuleStruct::CMat &pt_cloud_xyz){

	ifstream fin;
	fin.open(path_name, ios::in);
	if (!fin.is_open()){
		cout << "IOData::Load3DPtCloudData: cannot open file" << endl;
		return false;
	}

	int raw_data_row_num = 0;
    Get3DPtCloudRowNum(path_name, raw_data_row_num);
	if (raw_data_row_num <= 0) {
		cout << "IOData::Load3DPtCloudData: Get3DPtCloudRowNum() error" << endl;
		return false;
	}
	
	pt_cloud_xyz = ModuleStruct::CMat::zeros(raw_data_row_num, 3, CV_64F);
	
	string str_word;
	int idx = -1, col_idx = -1, row_idx = -1;
	double val;

	while(!fin.eof() && row_idx <= pt_cloud_xyz.rows - 1){
		idx ++;
		char temp; fin.get(temp); 
		str_word += temp;
		if (temp == ' '  || temp == '\n' || temp == '\t' || fin.eof()){
			if (idx == 0) continue;
			col_idx ++;
			str_word += '\0';
			val = stod(str_word);
			col_idx = (col_idx > 2)? 0:col_idx;
			if (col_idx == 0){
				row_idx ++;
				if (row_idx >= pt_cloud_xyz.rows)
					continue;
				pt_cloud_xyz.at<double>(row_idx, 0) = val;
			}
			else if (col_idx == 1)
				pt_cloud_xyz.at<double>(row_idx, 1) = val;
			else if (col_idx == 2)
				pt_cloud_xyz.at<double>(row_idx, 2) = val;


			str_word.clear();
			idx = -1;
		}
	}
	fin.close();

	return true;
}

//load 3d point cloud from file
bool IOData::Load3DPtCloudData(const string path_name, ModuleStruct::Point3Array &pt_cloud_xyz) {

	FILE* pFile;
#ifdef WIN32
	errno_t err = fopen_s(&pFile, path_name.c_str(), "r");
#else
	pFile = fopen(path_name.c_str(), "r");
#endif
	if (!pFile)
	{
		cout << "IOData::Load3DPtCloudData: cannot open file" << endl;
		log_fatal("cannot open %s", path_name.c_str());
		return false;
	}

	//bool is_ply_file = IdentifySuffix(path_name, ".ply");
	//if (is_ply_file)
	//{
	//	return LoadPLYData(path_name, pt_cloud_xyz);
	//}

#if 1
	int raw_data_row_num = 0;
	Get3DPtCloudRowNum(path_name, raw_data_row_num);

	bool is_pts_file = IdentifySuffix(path_name, ".pts");
	if (is_pts_file)
	{
		raw_data_row_num = raw_data_row_num - 1;
		//log_info("00 raw_data_row_num =%d", raw_data_row_num);
	}

	if (raw_data_row_num <= 0) {
		cout << "IOData::Load3DPtCloudData: Get3DPtCloudRowNum() error" << endl;
		log_fatal("NO row number at %s", path_name.c_str());
		return false;
	}

	pt_cloud_xyz.resize(raw_data_row_num);
	int row_idx = 0;
	while (!feof(pFile) && row_idx < raw_data_row_num)
	{
		if ((row_idx == 0) && is_pts_file)
		{
			int first_line;
			FSCANF(pFile, "%d%*[^\n]%*c", &first_line);
			//log_info("first_line =%d ", first_line);
		}
		if (FSCANF(pFile, "%f%f%f%*[^\n]%*c", &pt_cloud_xyz[row_idx].x, &pt_cloud_xyz[row_idx].y, &pt_cloud_xyz[row_idx].z) == 3)
		{
			row_idx++;
		}
		
	}
#else

	Point3f inputxyz;
	while (!feof(pFile))
	{
		if (FSCANF(pFile, "%f%f%f%*[^\n]%*c", &inputxyz.x, &inputxyz.y, &inputxyz.z) == 3)
		{
			pt_cloud_xyz.push_back(inputxyz);
		}
		else
		{
			//todo
		}
	}
#endif
	fclose(pFile);

	return true;
}


bool IOData::Load3DPtCloudData(const std::string path_name, std::vector<cv::Point3f>& pt_cloud_xyz, std::vector<int>& input_reflect, bool is_cad)
{
	FILE* pFile;
#ifdef WIN32
	errno_t err = fopen_s(&pFile, path_name.c_str(), "r");
#else
	pFile = fopen(path_name.c_str(), "r");
#endif
	if (!pFile)
	{
		std::cout << "IOData::Load3DPtCloudData: cannot open file" << std::endl;
		log_fatal("cannot open %s", path_name.c_str());
		return false;
	}

#if 1
	int raw_data_row_num = 0;
	Get3DPtCloudRowNum(path_name, raw_data_row_num);
	if (raw_data_row_num <= 0) {
		std::cout << "IOData::Load3DPtCloudData: Get3DPtCloudRowNum() error" << std::endl;
		log_fatal("NO row number at %s", path_name.c_str());
		return false;
	}

	if (!is_cad)
	{
		//skip first line of total line numbers
		char line[256] = { 0 };
		fgets(line, sizeof(line), pFile);
		raw_data_row_num -= 1;
	}

	pt_cloud_xyz.resize(raw_data_row_num);
	input_reflect.resize(raw_data_row_num);
	int row_idx = 0;
	while (!feof(pFile) && row_idx < raw_data_row_num)
	{
		if (fscanf(pFile, "%f%f%f%d%*[^\n]%*c", &pt_cloud_xyz[row_idx].x, &pt_cloud_xyz[row_idx].y, &pt_cloud_xyz[row_idx].z, &input_reflect[row_idx]) == 4)
		{
			row_idx++;
		}
	}
#else

	cv::Point3f inputxyz;
	while (!feof(pFile))
	{
		if (fscanf_s(pFile, "%f%f%f%*[^\n]%*c", &inputxyz.x, &inputxyz.y, &inputxyz.z) == 3)
		{
			pt_cloud_xyz.push_back(inputxyz);
		}
		else
		{
			//todo
		}
	}
#endif
	fclose(pFile);

	return true;
}

//load 3d point cloud from .txt file and with delimiter
bool IOData::Load3DPtCloudDataWithDelimiter(const string file_name, string delimiter, ModuleStruct::CMat &pt_cloud_xyz) {

	ifstream fin;
	fin.open(file_name, ios::in);
	if (!fin.is_open()){
		cout << "IOData::Load3DPtCloudDataWithDelimiter: cannot open file" << endl;
		return false;
	}

	pt_cloud_xyz = ModuleStruct::CMat::zeros(1, 3, CV_32F);
	int line_idx = 0;

	while (!fin.eof()) {
		//read one line
		string line;
		getline(fin, line);


		//check num in each line
		size_t pos = 0;
		size_t searching_begin_idx = 0;
		int num_each_line = 0;
		while ((pos = line.find(delimiter, searching_begin_idx)) < string::npos) {
			searching_begin_idx = pos + 1;
			num_each_line++;
		}
		num_each_line++;

		//read XYZ data of each line
		if (num_each_line == 3) {
			pos = 0;
			searching_begin_idx = 0;
			int begin_pos = 0;
			int end_pos = 0;
			int XYZ_idx = 0;
			ModuleStruct::CMat mat_line = ModuleStruct::CMat(1, 3, CV_32F);

			for (int i = 0; i < 3; i++) {
				if (i < 2)	pos = line.find(delimiter, searching_begin_idx);
				else		pos = sizeof(line);
				end_pos = (int)(pos);
				string single_num = line.substr(begin_pos, end_pos - begin_pos);
				mat_line.at<float>(0, i) = stof(single_num);
				begin_pos = (int)(pos + 1);

				searching_begin_idx = pos + 1;
			}

			//push one point into point file
			if (line_idx > 0)
				pt_cloud_xyz.push_back(mat_line);
			else {
				pt_cloud_xyz.at<float>(0, 0) = mat_line.at<float>(0, 0);
				pt_cloud_xyz.at<float>(0, 1) = mat_line.at<float>(0, 1);
				pt_cloud_xyz.at<float>(0, 1) = mat_line.at<float>(0, 1);
			}
			line_idx++;
		}
	}
	fin.close();

	return true;
}

//load 3d point cloud with normals from .txt file
bool IOData::Load3DPtCloudDataWithNormal(const string path_name, ModuleStruct::Point3Array &input_points, ModuleStruct::Point3Array &input_normals)
{
	FILE* pFile;
#ifdef WIN32
	errno_t err = fopen_s(&pFile, path_name.c_str(), "r");
#else
	pFile = fopen(path_name.c_str(), "r");
#endif

	if (!pFile)
	{
		cout << "IOData::Load3DPtCloudDataWithNormal: cannot open file" << endl;
		log_fatal("cannot open %s", path_name.c_str());
		return false;
	}

	int raw_data_row_num = 0;
	Get3DPtCloudRowNum(path_name, raw_data_row_num);
	bool is_pts_file = IdentifySuffix(path_name, ".pts");
	if (is_pts_file)
	{
		raw_data_row_num = raw_data_row_num - 1;
	}

	if (raw_data_row_num <= 0) {
		cout << "IOData::Load3DPtCloudDataWithNormal: Get3DPtCloudRowNum() error" << endl;
		log_fatal("NO row number at %s", path_name.c_str());
		return false;
	}

	input_points.resize(raw_data_row_num);
	input_normals.resize(raw_data_row_num);
	int row_idx = 0;
	while (!feof(pFile) && row_idx < raw_data_row_num)
	{
		if ((row_idx == 0) && is_pts_file)
		{
			int first_line;
			FSCANF(pFile, "%d%*[^\n]%*c", &first_line);
		}

		if (FSCANF(pFile, "%f%f%f%f%f%f%*[^\n]%*c", &input_points[row_idx].x, &input_points[row_idx].y, &input_points[row_idx].z,
			&input_normals[row_idx].x, &input_normals[row_idx].y, &input_normals[row_idx].z) == 6)
		{
			row_idx++;
		}
	}
	fclose(pFile);

	return true;
}

//write error log
bool IOData::WriteErrorLog(const string err_msg){
	ofstream fout("error_log.txt", ios::app);
	
	if (!fout.fail()) {
		fout << err_msg << endl;
	}

	fout.close();
	return true;
}

//load grid to occupied voxel index array from .txt file
bool IOData::LoadGridToOccupiedIdx(const string path_name, unsigned int* &grid_to_occupied_voxel_idx, unsigned int &grid_to_occupied_voxel_idx_size) {

	FILE* pFile;
	pFile = fopen(path_name.c_str(), "r");

	if (!pFile)
	{
		cout << "IOData::LoadGridToOccupiedIdx: cannot open file" << endl;
		log_fatal("cannot open %s", path_name.c_str());
		return false;
	}

	int raw_data_row_num = 0;
	Get3DPtCloudRowNum(path_name, raw_data_row_num);
	if (raw_data_row_num <= 0) {
		cout << "IOData::LoadGridToOccupiedIdx: Get3DPtCloudRowNum() error" << endl;
		log_fatal("NO row number at %s", path_name.c_str());
		return false;
	}

	grid_to_occupied_voxel_idx_size = raw_data_row_num;
	grid_to_occupied_voxel_idx = new unsigned int[raw_data_row_num];
	int row_idx = 0;
	int index = 0;
	while (!feof(pFile) && row_idx < raw_data_row_num)
	{
		if (FSCANF(pFile, "%d%d%*[^\n]%*c", &index, &grid_to_occupied_voxel_idx[row_idx]) == 2)
		{
			row_idx++;
		}
	}

	fclose(pFile);

	return true;

}

// filter noise data of input data
bool IOData::FilterInputData(ModuleStruct::Point3Array&scene_xyz, ModuleStruct::Point3Array &filtered_scene_xyz)
{
	const float max_size_x = 5000.0f;
	const float max_size_y = 5000.0f;
	const float rough_scale = 2.f;
	const float SceneDataRemoveNoiseFactor = 1.5f;

	ModuleStruct::Point3d scene_data_mean = { 0.0,0.0,0.0 };

	size_t size_of_input = scene_xyz.size();

	bool *is_inside = new bool[size_of_input];
	double thrld_size_1 = rough_scale * max_size_x;
	
	int size_of_filtered = 0;
	double sum_x = 0;
	double sum_y = 0;
#pragma omp parallel for reduction (+:size_of_filtered,sum_x,sum_y)
	for (int i = 0; i < scene_xyz.size(); i++)
	{
		is_inside[i] = false;
		bool condition = (fabs(scene_xyz[i].x) < thrld_size_1) && (fabs(scene_xyz[i].y) < thrld_size_1);
		if (!condition) continue;
		sum_x += scene_xyz[i].x;
		sum_y += scene_xyz[i].y;
		size_of_filtered++;
	}

	scene_data_mean.x = sum_x/size_of_filtered;
	scene_data_mean.y = sum_y/size_of_filtered;

	double thrld_size_2 = SceneDataRemoveNoiseFactor * max_size_x;
	size_of_filtered = 0;

#pragma omp parallel for reduction (+:size_of_filtered)
	for (int i = 0; i < scene_xyz.size(); i++)
	{
		bool condition = (fabs(scene_xyz[i].x - scene_data_mean.x) < thrld_size_2) && (fabs(scene_xyz[i].y - scene_data_mean.y) < thrld_size_2);
		if (condition)
		{
			is_inside[i] = true;
			size_of_filtered++;
		}
	}

	filtered_scene_xyz.resize(size_of_filtered);
	for (int i = 0, j = 0; i < scene_xyz.size(); i++)
	{
		if (is_inside[i]) filtered_scene_xyz[j++] = scene_xyz[i];
	}
	delete[] is_inside;
	return true;
}

bool IOData::IdentifySuffix(string filePath, const string suffix)
{
	char *path  = const_cast<char*>(filePath.c_str());

	char* pos = strrchr(path, '.');
	if (pos)
	{
		string str(pos + 1);
		transform(str.begin(), str.end(), str.begin(), ::tolower);
		str = pos;

		if (str.compare(suffix) == 0)
		{
			return true;
		}

		return false;
	}
	else
	{
		return false;
	}

}

bool IOData::SavePlyFile(const string path_name, const ModuleStruct::Point3Array input_points, ModuleStruct::Point3Array input_normals)
{
	ofstream fout(path_name);
	if (fout.fail())
	{
		cout << "cannot open file" << endl;
		return false;
	}
	//save header
	fout << "ply" << endl;
	fout << "format ascii 1.0" << endl;
	fout << "comment VCGLIB generated" << endl;
	fout << "element vertex " << input_points.size() << endl;
	fout << "property float x" << endl;
	fout << "property float y" << endl;
	fout << "property float z" << endl;
	fout << "property float nx" << endl;
	fout << "property float ny" << endl;
	fout << "property float nz" << endl;
	fout << "element face 0" << endl;
	fout << "property list uchar int vertex_indices" << endl;
	fout << "end_header" << endl;

	for (unsigned int i = 0; i < input_points.size(); i++)
	{
		fout << fixed << input_points[i].x << ' ' << input_points[i].y << ' ' << input_points[i].z << ' '
			//<< 0.0 << ' ' << 0.0 << ' ' << 1.0 << endl;
			<< input_normals[i].x << ' ' << input_normals[i].y << ' ' << input_normals[i].z << endl;
	}
	fout.close();
	return true;
}

int IOData::parse_argument(int argc, char** argv, const char* str, string &path)
{
	int index = -1;
	for (int i = 1; i < argc; ++i)
	{
		// Search for the string
		if (strcmp(argv[i], str) == 0)
		{
			index = i;
		}
	}

	index = index + 1;
	if (index > 0 && index < argc)
		path = argv[index];

	return index - 1;
}

int IOData::parse_argument(int argc, char** argv, const char* str, bool &is_include)
{
	int index = -1;
	for (int i = 1; i < argc; ++i)
	{
		if (strcmp(argv[i], str) == 0)
		{
			index = i;
		}
	}

	index = index + 1;
	if (index > 0 && index < argc)
		is_include = atoi(argv[index]) == 1;

	return index - 1;
}

int IOData::createDirectory(string Dir)
{
	size_t len = Dir.length();
	char tmpDirPath[500] = { 0 };
	for (int i = 0; i < len; i++)
	{
		tmpDirPath[i] = Dir[i];
		if (tmpDirPath[i] == '\\' || tmpDirPath[i] == '/')
		{
			if (ACCESS(tmpDirPath, 0) == -1)
			{
				int ret = MKDIR(tmpDirPath);
				if (ret == -1) return ret;
			}
		}
	}
	return 0;
}

wstring IOData::ReadUTF8FileString(string &filename)
{
	wifstream wif(filename);
#ifdef _WIN32
	wif.imbue(locale(locale::empty(), new codecvt_utf8<wchar_t>));
#else
    wif.imbue(locale(locale(), new codecvt_utf8<wchar_t>));
#endif
	wstringstream wss;
	wss << wif.rdbuf();
	return wss.str();
}

wstring IOData::ReadUTF8FileWString(wstring &filename)
{
#ifdef _WIN32
	wifstream wif(filename.c_str());
	wif.imbue(locale(locale::empty(), new codecvt_utf8<wchar_t>));
#else
    std::wstring_convert<std::codecvt_utf8<wchar_t>, wchar_t> converter;
    wifstream wif(converter.to_bytes(filename));
    wif.imbue(locale(locale(), new codecvt_utf8<wchar_t>));
#endif
	wstringstream wss;
	wss << wif.rdbuf();
	wif.close();
	return wss.str();
}

// load line segment from.txt file
bool IOData::LoadLineSegment(const string path_name, ModuleStruct::Vector<ModuleStruct::Line3> &lines)
{
	FILE* pFile;
#ifdef WIN32
	errno_t err = fopen_s(&pFile, path_name.c_str(), "r");
#else
	pFile = fopen(path_name.c_str(), "r");
#endif

	if (!pFile)
	{
		cout << "IOData::LoadLineSegment: cannot open file" << endl;
		log_fatal("cannot open %s", path_name.c_str());
		return false;
	}

	int raw_data_row_num = 0;
	Get3DPtCloudRowNum(path_name, raw_data_row_num);
	bool is_pts_file = IdentifySuffix(path_name, ".pts");
	if (is_pts_file)
	{
		raw_data_row_num = raw_data_row_num - 1;
	}

	if (raw_data_row_num <= 0) {
		cout << "IOData::LoadLineSegment: Get3DPtCloudRowNum() error" << endl;
		log_fatal("NO row number at %s", path_name.c_str());
		return false;
	}

	lines.resize(raw_data_row_num);
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(lines.size()); i++)
	{
		lines[i].points.resize(2);
	}

	int row_idx = 0;
	while (!feof(pFile) && row_idx < raw_data_row_num)
	{
		if ((row_idx == 0) && is_pts_file)
		{
			int first_line;
			FSCANF(pFile, "%d%*[^\n]%*c", &first_line);
		}

		if (FSCANF(pFile, "%f%f%f%f%f%f%*[^\n]%*c", &lines[row_idx].points[0].x, &lines[row_idx].points[0].y, &lines[row_idx].points[0].z,
			&lines[row_idx].points[1].x, &lines[row_idx].points[1].y, &lines[row_idx].points[1].z) == 6)
		{
			lines[row_idx].direction = { 0, 0, 0 };
			row_idx++;
		}
	}
	fclose(pFile);

	return true;
}

bool IOData::SavePoint3fData(const std::string path_name, std::vector<cv::Point3f> &input_data, const std::vector<unsigned char> &input_reflect)
{
	if (input_data.empty()) {
		std::cout << "IOData::SavePoint3fData: empty data" << std::endl;
		return false;
	}

	if (path_name.empty()) {
		std::cout << "IOData::SavePoint3fData: empty file path" << std::endl;
		return false;
	}

	std::ofstream fout(path_name);
	if (fout.fail()) {
		std::cout << "IOData::SavePoint3fData: cannot open file" << std::endl;
		log_fatal("cannot open %s", path_name.c_str());
		return false;
	}

	for (int i = 0; i < input_data.size(); i++) {
		fout << input_data[i].x << '\t' << input_data[i].y << '\t' << input_data[i].z << '\t' << (int)input_reflect[i];
		if (i < input_data.size() - 1) { fout << std::endl; }
	}
	fout.close();

	return true;
}
//=================hgj add loadply and saveply start===================================//

bool IOData::SavePLYPoints3fAsc(const std::string path_name, const std::vector<cv::Point3f> &input_data)
{
	if (input_data.empty()) {
		std::cout << "IOData::SavePLYPoints3fAsc: empty data" << std::endl;
		return false;
	}

	if (path_name.empty()) {
		std::cout << "IOData::SavePLYPoints3fAsc: empty file path" << std::endl;
		return false;
	}

	std::ofstream fout(path_name);
	if (fout.fail()) {
		std::cout << "IOData::SavePLYPoints3fAsc: cannot open file" << std::endl;
		return false;
	}

	int elementNum = input_data.size();
	fout << "ply" << std::endl;
	fout << "format ascii 1.0" << std::endl;
	fout << "comment File generated" << std::endl;
	fout << "element vertex " << elementNum << std::endl;
	fout << "property float x" << std::endl;
	fout << "property float y" << std::endl;
	fout << "property float z" << std::endl;
	fout << "end_header" << std::endl;

	for (int i = 0; i < elementNum; i++)
	{
		fout << input_data[i].x << " " << input_data[i].y << " " << input_data[i].z << " " << std::endl;
	}
	fout.close();
	return true;

}

bool IOData::SavePLYPoints3fBin(const std::string path_name, const std::vector<cv::Point3f> &input_data)
{
	if (input_data.empty()) {
		std::cout << "IOData::SavePLYPoints3fBin: empty data" << std::endl;
		return false;
	}

	if (path_name.empty()) {
		std::cout << "IOData::SavePLYPoints3fBin: empty file path" << std::endl;
		return false;
	}

	std::ofstream outFile(path_name, std::ios::out | std::ios::binary);
	if (outFile.fail()) {
		std::cout << "IOData::SavePLYPoints3fBin: cannot open file" << std::endl;
		return false;
	}

	int elementNum = input_data.size();
	outFile << "ply" << std::endl;
	outFile << "format binary_little_endian 1.0" << std::endl;
	outFile << "comment File generated" << std::endl;
	outFile << "element vertex " << elementNum << std::endl;
	outFile << "property float x" << std::endl;
	outFile << "property float y" << std::endl;
	outFile << "property float z" << std::endl;
	outFile << "end_header" << std::endl;

	for (int i = 0; i<elementNum; i++)
	{
		float x = input_data[i].x;
		float y = input_data[i].y;
		float z = input_data[i].z;
		//cout << "value:" << value << endl;
		outFile.write((char*)&x, sizeof(x));
		outFile.write((char*)&y, sizeof(y));
		outFile.write((char*)&z, sizeof(z));

	}
	outFile.close();
	return true;
}

bool IOData::LoadPLYPoints3fAsc(const std::string path_name, std::vector<cv::Point3f> &pt_cloud_xyz)
{

	ifstream in;
	in.open(path_name, std::ios::in);

	if (!in.is_open())
	{
		//std::cerr << "File " << ply_file << " could not be opened" << std::endl;
		std::cout << "IOData::LoadPLYPoints3fAsc " << path_name << " : cannot open file" << std::endl;
		return false;
	}


	std::string line;

	std::string num_flag = "element vertex ";

	int pos = num_flag.size();

	int num_pt = 0;
	//提取标识字段中所包含的点的数目信息
	while (!in.eof())
	{
		getline(in, line);

		if (line.find(num_flag) != string::npos)
		{

			string num = line.substr(pos);
			num_pt = atoi(num.c_str());

			cout << "There are " << num_pt << " Points" << endl;
		}

		if (line == "end_header")
		{
			break;
		}
	}

	float x, y, z;

	//ofstream out("asc_pts_check.txt");

	while (!in.eof())
	{
		in >> x >> y >> z;
		getline(in, line);
		if (in.fail())
			break;
		//out << x << " " << y << " " << z << endl;
		pt_cloud_xyz.push_back(cv::Point3f(x, y, z));

	}
	in.close();

	if (pt_cloud_xyz.size() <= 0) {
		std::cout << "IOData::LoadPLYPoints3fAsc: pt_cloud_xyz size() error" << std::endl;
		return false;
	}

	return true;
}

bool IOData::LoadPLYPoints3fBin(const std::string path_name, std::vector<cv::Point3f> &pt_cloud_xyz)
{

	FILE *fp;
	fp = fopen(path_name.c_str(), "rb");

	char strLine[1024];
	char end_flag[] = "end_header ";
	char num_flag[] = "element vertex ";
	char *p;
	char num[100];
	if (fp == NULL)
	{
		//printf("Error:Open input.c file fail!\n");
		std::cout << "IOData::LoadPLYPoints3fBin " << path_name << " : cannot open file" << std::endl;
		return false;
	}
	int numPts = 0;
	while (!feof(fp))//循环读取每一行，直到文件尾
	{
		fgets(strLine, 1024, fp);

		if (strlen(strLine) == (strlen(end_flag)))
		{
			break;
		}

		if ((p = strstr(strLine, num_flag)) != NULL)
		{
			int start = strlen(num_flag);
			int sub_len = strlen(strLine) - strlen(num_flag);

			for (int i = 0; i < sub_len; i++)
			{
				num[i] = strLine[start + i];    //从第start+i个元素开始向数组内赋值
			}
			numPts = atoi(num);
		}
	}

	if (numPts <= 0)
	{
		std::cout << "IOData::LoadPLYPoints3fBin numPts: " << numPts << " error!" << std::endl;
		return false;
	}

	float *pts = (float*)malloc(numPts * 3 * sizeof(float));

	float cnt = numPts * 3;

	fread(pts, sizeof(float), cnt, fp);

	fclose(fp);

	for (int i = 0; i<numPts; i++)
	{
		//注意点的访问方式
		pt_cloud_xyz.push_back(cv::Point3f(pts[3 * i + 0], pts[3 * i + 1], pts[3 * i + 2]));
	}
	return true;
}

bool IOData::LoadPLYPoints3f(const std::string path_name, std::vector<cv::Point3f> &pt_cloud_xyz)
{
	ifstream in;
	in.open(path_name);
	string line;

	string format_flag_bin = "binary";
	string format_flag_asc = "ascii";

	while (!in.eof())
	{
		getline(in, line);
		if (line.find(format_flag_bin) != string::npos)
		{
			cout << "binary format point cloud loaded! " << endl;
			bool isSuc = LoadPLYPoints3fBin(path_name, pt_cloud_xyz);
			if (isSuc)
			{
				in.close();
				return true;
			}
			else
			{
				in.close();
				return false;
			}
		}
		else if (line.find(format_flag_asc) != string::npos)
		{
			cout << "ascii format point cloud loaded! " << endl;
			bool isSuc = LoadPLYPoints3f(path_name, pt_cloud_xyz);
			if (isSuc)
			{
				in.close();
				return true;
			}
			else
			{
				in.close();
				return false;
			}
		}
		else
			continue;
	}
	return false;
}

bool IOData::SavePLYPoints3f(const std::string path_name, const std::vector<cv::Point3f> &input_data, bool isSaveBinary)
{
	if (isSaveBinary)
	{
		return SavePLYPoints3fBin(path_name, input_data);
	}
	else
	{
		return SavePLYPoints3fAsc(path_name, input_data);
	}
}

//=================hgj add loadply and saveply end=====================================//

std::vector<cv::Point3f> IOData::readFromXyz(const std::string& path, const bool& useEncryptData, std::vector<float>* intensity)
{
	std::vector<cv::Point3f> cloud;
	//std::vector<int> reflectance;
	//std::vector<std::pair<cv::Point3f, int>> pair_cloud_reflect;
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

				return std::vector<cv::Point3f>();
			}
		}
		else if (ReadEncryFileV2(&dataV2, path))
		{
			if (dataV2.size() == 1)
			{

				std::cout << "nCount!= pointCntShold (" << dataV2[0].x << " vs." << dataV2[0].y << ")" << std::endl;


				return std::vector<cv::Point3f>();
			}
		}
		else
		{
			return std::vector<cv::Point3f>();
		}
		if (data.size() > 1)
		{
			if (intensity)intensity->resize(data.size());
			cloud.resize(data.size());
			//reflectance.resize(data.size());
#pragma omp parallel for 
			for (int i = 0; i < data.size(); i++)
			{
				cloud[i].x = data[i].x;
				cloud[i].y = data[i].y;
				cloud[i].z = data[i].z;
				if (intensity)(*intensity)[i] = data[i].i;
				//reflectance[i] = int(dataV2[i].i);

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
				//reflectance[i] = int(dataV2[i].i);
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
				return std::vector<cv::Point3f>();
			}
			else
			{
				std::stringstream ss(aline);
				{
					float x, y, z, intensity_ = 0;
					ss >> x >> y >> z >> intensity_;
					cloud.emplace_back(cv::Point3f(x, y, z));
					if (intensity && hasIntensity) { intensity->emplace_back(intensity_); };
					//reflectance.emplace_back(int(intensity_));
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
					//reflectance.emplace_back(int(intensity_));
				}
			}
			fin1.close();
		}
		catch (...)
		{
			return std::vector<cv::Point3f>();
		}
	}

	for (int i = 0; i < cloud.size(); i++)
	{
		std::pair<cv::Point3f, int> point_reflect;
		point_reflect.first = cloud[i];
		//point_reflect.second = reflectance[i];

		//pair_cloud_reflect.emplace_back(point_reflect);
	}


	return cloud;
}