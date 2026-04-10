#include "mesh_tool.h"

CMeshTool::CMeshTool()
{
}

CMeshTool::~CMeshTool()
{
}

float CMeshTool::stringToFloat(const std::string& str)
{
	std::istringstream iss(str);
	float num;
	iss >> num;
	return num;
}

bool CMeshTool::GetObjData(const std::string& fileContent, std::vector<cv::Point3f> &points, std::vector<cv::Vec3i> &face,std::string* mtl, std::vector<cv::Vec2f>* vt)
{
	std::ifstream myfile(fileContent, std::ios::in);
	if (!myfile.is_open())
	{
		//std::cerr << "File " << ply_file << " could not be opened" << std::endl;
		std::cout << "CMeshTool::GetObjData " << fileContent << " : cannot open file" << std::endl;
		return false;
	}
	std::string line;
	//int n = 0;
	//int v_n = 0;
	while (!myfile.eof())
	{
		getline(myfile, line);
		std::istringstream iss(line);
		if (strlen(line.c_str()) > 0) {

			if (line[0] == 'v'&&line[1] == ' ')
			{
				std::string type, a, b, c;
				cv::Point3f v;
				iss >> type >> a >> b >> c;
				v.x = stringToFloat(a);
				v.y = stringToFloat(b);
				v.z = stringToFloat(c);
				points.push_back(v);
			}
			else if (line[0] == 'v'&&line[1] == 't')
			{
				if (vt != nullptr) {
					std::string type, a, b;
					cv::Vec2f vt_t;
					iss >> type >> a >> b;
					vt_t[0] = stringToFloat(a);
					vt_t[1] = stringToFloat(b);
					vt->push_back(vt_t);
				}
			}
			else if (line[0] == 'f' &&line[1] == ' ')
			{
				std::string type, a, b, c;
				cv::Vec3i f;
				iss >> type >> a >> b >> c;
				f[0] = std::stoi(a);
				f[1] = std::stoi(b);
				f[2] = std::stoi(c);
				face.push_back(f);
			}
			else if (line.find("mtllib") != -1)
			{
				if (mtl != nullptr) {
					*mtl = line;
				}
			}
		}
	}
	myfile.close();

	if (points.size() <= 0 || face.size()<=0) {
		std::cout << "CMeshTool::GetObjData: points or face size()<=0 error!" << std::endl;
		return false;
	}
	return true;
}


bool CMeshTool::SaveObjData(const int& meshId, 
	const std::string& fileContent, 
	const std::vector<cv::Point3f>& points, 
	const std::vector<cv::Vec3i>& face, 
	const std::vector<cv::Vec2f>& vt)
{
	if (points.empty()) {
		std::cout << "CMeshTool::SaveObjData points empty data!" << std::endl;
		return false;
	}

	if (face.empty()) {
		std::cout << "CMeshTool::SaveObjData face empty data!" << std::endl;
		return false;
	}

	if (vt.empty()) {
		std::cout << "CMeshTool::SaveObjData vt empty data!" << std::endl;
		return false;
	}

	if (fileContent.empty()) {
		std::cout << "CMeshTool::SaveObjData: empty file path!" << std::endl;
		return false;
	}

	std::ofstream obj(fileContent);
	if (obj.fail()) {
		std::cout << "CMeshTool::SaveObjData: cannot open file!" << std::endl;
		return false;
	}

	//obj << "mtllib ./mesh_" << meshId << ".mtl" << "\n";
	obj << "mtllib ./mesh" << meshId << ".mtl" << "\n";


	//save points
	for (int i = 0; i< points.size(); i++)
	{
		obj << "v" << " " << points[i].x << " " << points[i].y << " " << points[i].z << "\n";
	}

	//save texture
	for (int i = 0; i< vt.size(); i++)
	{
		obj << "vt" << " " << vt[i][0] << " " << vt[i][1] << "\n";
	}

	//save face
	for (int i = 0; i< face.size(); i++)
	{
		obj << "f" << " " << face[i][0] << "/" << face[i][0] << " " << face[i][1] << "/" << face[i][1] << " " << face[i][2] << "/" << face[i][2] << "\n";
	}

	obj.close();

	return true;

}

bool CMeshTool::SaveObjData(std::string mtl,
	const std::string& fileContent,
	const std::vector<cv::Point3f>& points,
	const std::vector<cv::Vec3i>& face,
	const std::vector<cv::Vec2f>& vt)
{
	if (points.empty()) {
		std::cout << "CMeshTool::SaveObjData points empty data!" << std::endl;
		return false;
	}

	if (face.empty()) {
		std::cout << "CMeshTool::SaveObjData face empty data!" << std::endl;
		return false;
	}

	if (fileContent.empty()) {
		std::cout << "CMeshTool::SaveObjData: empty file path!" << std::endl;
		return false;
	}

	std::ofstream obj(fileContent);
	if (obj.fail()) {
		std::cout << "CMeshTool::SaveObjData: cannot open file!" << std::endl;
		return false;
	}

	//obj << "mtllib ./mesh_" << meshId << ".mtl" << "\n";
	obj << mtl << "\n";


	//save points
	for (int i = 0; i< points.size(); i++)
	{
		obj << "v" << " " << points[i].x << " " << points[i].y << " " << points[i].z << "\n";
	}

	//save texture
	for (int i = 0; i< vt.size(); i++)
	{
		obj << "vt" << " " << vt[i][0] << " " << vt[i][1] << "\n";
	}

	//save face
	for (int i = 0; i< face.size(); i++)
	{
		obj << "f" << " " << face[i][0] << "/" << face[i][0] << " " << face[i][1] << "/" << face[i][1] << " " << face[i][2] << "/" << face[i][2] << "\n";
	}

	obj.close();

	return true;
}


void CMeshTool::SaveMtlfile(const int& meshId,std::string& mesh_str,const std::string& mtlPath)
{
	//mesh_str="mesh" for example
	std::string save_str_name = " " + mesh_str;
	std::ofstream mtl(mtlPath);
	//obj << "mtllib ./mesh6.mtl" << "\n";
	//mtl << "map_Kd" << " mesh" << meshId << ".jpg" << "\n";
	mtl << "map_Kd " << save_str_name << meshId << ".jpg" << "\n";
	mtl.close();
}

void CMeshTool::SaveMtlfileNew( const std::string& mtlPath, const std::string& mapkd_str)
{
	std::ofstream mtl(mtlPath);
	//mtl << "map_Kd " << save_str_name << meshId << "_.jpg" << "\n";
	mtl << "map_Kd " << mapkd_str  << ".jpg" << "\n";
	mtl.close();
}
