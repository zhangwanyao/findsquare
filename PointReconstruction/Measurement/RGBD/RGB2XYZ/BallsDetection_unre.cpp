#include "jansson.h"
#include <opencv2/core.hpp>
#include "codec.h"
#include "ballDetection.h"
#include "BallsDetection_unre.h"
#include "time.h"
#include <Eigen/Dense>
using namespace cv;
using namespace std;

BALLDETECTION balldetection;
json_t* b_root;
CloudPointCodec dodec;
Eigen::Matrix3d imuR;

using data_type = tuple<Eigen::Matrix3d, Eigen::Matrix3d, Eigen::Vector3d, cv::Mat,
	shared_ptr<THandleImgAndEdge>>;

map<int, map<int, data_type>> b_data_all;

std::string b_path;
std::vector<cv::Point3f> scene_xyz;
std::vector<int> scene_reflect;

//std::shared_ptr<PlaneSegResult> b_seg;
std::vector<std::vector<cv::Point2d>> points_RGB_vec;
std::vector<std::vector<std::vector<cv::Point3d>>> points_ROI_vec_all; 
std::vector<std::vector<std::vector<int>>> reflect_ROI_vec_all; 
std::vector<std::vector<std::vector<cv::Point3d>>> balls_points_vec_all;
std::vector<std::vector<cv::Vec4d>> sphere_argument_vec_all;
std::vector<std::vector<cv::Point3d>> sphere_center_vec_all;

bool ReadEncryFile(vector<scenePointInfo>* vecPtInfos, string strPath)
{

	ifstream inFile(strPath, ios::in | ios::binary); //二进制读方式打开
	int nCount = 0;
	inFile.read((char*)&nCount, sizeof(int));
	char nSeed = nCount % 255;
	int count = nCount * sizeof(scenePointInfo);
	char * buf = new char[count];
	inFile.read(buf, count);
	inFile.close();
	for (int n = 0; n < count; n++)
	{
		buf[n] = buf[n] ^ nSeed;
	}
	scenePointInfo * t = (scenePointInfo *)buf;
	vecPtInfos->insert(vecPtInfos->begin(), t, t + nCount);
	delete buf;
	return true;
}

auto BallDetection::Read(std::string &sense_path)
{
	try
	{
		json_error_t b_error;
		auto pos = (sense_path.find_last_of("\\") + 1) == 0 ? sense_path.find_last_of("/") + 1 : sense_path.find_last_of("\\") + 1;
		b_path = sense_path.substr(0, pos);
		b_root = json_load_file((b_path + "textureMeshParam.json").c_str(), 0, &b_error);
		auto param = json_object_get(b_root, "param");
		auto cameras = json_object_get(param, "cameras");

		json_t* valuei = nullptr; int i;
		json_array_foreach(cameras, i, valuei);
		{
			auto value = (json_object_get(valuei, "value"));
			auto key = json_string_value(json_object_get(valuei, "key"));

			auto intr = (json_object_get(value, "intr"));
			json_t* value_j; double tmp[9]; int j = 0;
			json_array_foreach(intr, j, value_j)
				tmp[j] = json_number_value(value_j);

			get<0>(b_data_all[i][0]) = std::move(Eigen::Map<Eigen::Matrix3d>(tmp).transpose());

			auto R0 = json_object_get(value, "R0");
			json_array_foreach(R0, j, value_j)
				tmp[j] = json_number_value(value_j);

			get<1>(b_data_all[i][0]) = std::move(Eigen::Map<Eigen::Matrix3d>(tmp).transpose());

			auto t0 = json_object_get(value, "t0");
			auto tmp_t0 = &get<2>(b_data_all[i][0]);
			json_array_foreach(t0, j, value_j)
				(*tmp_t0)(j) = json_number_value(value_j);

			auto album = (json_object_get(value, "album"));
			json_array_foreach(album, j, value_j)
			{
				auto value_k = json_object_get(value_j, "value");
				auto name = json_string_value(value_k);
				get<3>(b_data_all[i][j * 60]) = cv::imread(b_path + name);
			}

			cout << json_number_value(json_object_get(json_array_get(json_object_get(b_root, "param"), 0), "key")) << endl;
			cout << "data" << endl;
		}
	}
	catch (...)
	{
		cout << "error" << endl;
	}

}

auto BallDetection::alignment(std::string &sense_path)
{
	Eigen::Matrix3d imuR;
	auto pos = (sense_path.find_last_of("\\") + 1) == 0 ? sense_path.find_last_of("/") + 1 : sense_path.find_last_of("\\") + 1;
	b_path = sense_path.substr(0, pos);
	ifstream in(b_path + "imuR.txt");
	for (size_t i = 0; in; i++)
	{
		in >> imuR.data()[i];
	}
	imuR.transposeInPlace();
	cout << imuR << endl;

	Eigen::Matrix3d elementary_transformation; elementary_transformation << 1, 0, 0,
		0, 0, 1,
		0, 1, 0;
	imuR = elementary_transformation * imuR * elementary_transformation;

	for (auto iti = b_data_all.begin(); iti != b_data_all.end(); iti++)
	{
		auto camera_key = iti->first;

		auto camera_param = (iti->second.begin()->second);
		Eigen::Matrix3d intr(get<0>(camera_param));
		Eigen::Matrix3d r0(get<1>(camera_param));
		Eigen::Vector3d t0(get<2>(camera_param));
		for (auto itj = iti->second.begin(); itj != iti->second.end(); itj++)
		{
			auto camera_dir = itj->first;
			Eigen::AngleAxisd rot_vector(camera_dir * M_PI / 180, Eigen::Vector3d(0, 0, 1));
			Eigen::Matrix3d rot = rot_vector.toRotationMatrix();
			Eigen::Matrix3d unre2rightcoord;
			unre2rightcoord << 1, 0, 0,
				0, 0, 1,
				0, -1, 0;

			get<0>(itj->second) = intr;
			get<1>(itj->second) = r0 * unre2rightcoord *rot * imuR.transpose();
			get<2>(itj->second) = t0;
			get<4>(itj->second) = make_shared<THandleImgAndEdge>(get<3>(itj->second));
		}
	}
}

auto BallDetection::cv3f2eigen3f(const cv::Point3f &point3f)
{
	return Eigen::Vector3f(point3f.x, point3f.y, point3f.z);
}

auto BallDetection::eigen3d2cv3d(const Eigen::Vector3d &vector)
{
	return cv::Point3d(vector(0), vector(1), vector(2));
}

int BallDetection::cagaus(double a[], double b[], int n, double x[])
{
	int *js, l, k, i, j, is, p, q;
	double d, t;
	js = new int[n];
	l = 1;
	for (k = 0; k <= n - 2; k++)
	{
		d = 0.0;
		for (i = k; i <= n - 1; i++)
		{
			for (j = k; j <= n - 1; j++)
			{
				t = fabs(a[i*n + j]);
				if (t > d)
				{
					d = t;
					js[k] = j;
					is = i;
				}
			}
		}
		if (d + 1.0 == 1.0)
		{
			l = 0;
		}
		else
		{
			if (js[k] != k)
			{
				for (i = 0; i <= n - 1; i++)
				{
					p = i * n + k;
					q = i * n + js[k];
					t = a[p];
					a[p] = a[q];
					a[q] = t;
				}
			}
			if (is != k)
			{
				for (j = k; j <= n - 1; j++)
				{
					p = k * n + j;
					q = is * n + j;
					t = a[p];
					a[p] = a[q];
					a[q] = t;
				}
				t = b[k];
				b[k] = b[is];
				b[is] = t;
			}
		}
		if (l == 0)
		{
			delete js;
			return(0);
		}
		d = a[k*n + k];
		for (j = k + 1; j <= n - 1; j++)
		{
			p = k * n + j;
			a[p] = a[p] / d;
		}
		b[k] = b[k] / d;
		for (i = k + 1; i <= n - 1; i++)
		{
			for (j = k + 1; j <= n - 1; j++)
			{
				p = i * n + j;
				a[p] = a[p] - a[i*n + k] * a[k*n + j];
			}
			b[i] = b[i] - a[i*n + k] * b[k];
		}
	}
	d = a[(n - 1)*n + n - 1];
	if (fabs(d) + 1.0 == 1.0)
	{
		delete js;
		return(0);
	}
	x[n - 1] = b[n - 1] / d;
	for (i = n - 2; i >= 0; i--)
	{
		t = 0.0;
		for (j = i + 1; j <= n - 1; j++)
		{
			t = t + a[i*n + j] * x[j];
		}
		x[i] = b[i] - t;
	}
	js[n - 1] = n - 1;
	for (k = n - 1; k >= 0; k--)
	{
		if (js[k] != k)
		{
			t = x[k];
			x[k] = x[js[k]];
			x[js[k]] = t;
		}
	}
	delete js;
	return 1;
}

inline void BallDetection::estimate_sphere(const cv::Point3d & p1, const cv::Point3d & p2, const cv::Point3d & p3, const cv::Point3d & p4, cv::Point3d & center, double & radius2)
{
	double x1 = p1.x, y1 = p1.y, z1 = p1.z;
	double x2 = p2.x, y2 = p2.y, z2 = p2.z;
	double x3 = p3.x, y3 = p3.y, z3 = p3.z;
	double x4 = p4.x, y4 = p4.y, z4 = p4.z;

	double a11 = 2 * (x2 - x1);
	double a12 = 2 * (y2 - y1);
	double a13 = 2 * (z2 - z1);
	double a21 = 2 * (x3 - x2);
	double a22 = 2 * (y3 - y2);
	double a23 = 2 * (z3 - z2);
	double a31 = 2 * (x4 - x3);
	double a32 = 2 * (y4 - y3);
	double a33 = 2 * (z4 - z3);

	double b1 = x2 * x2 - x1 * x1 + y2 * y2 - y1 * y1 + z2 * z2 - z1 * z1;
	double b2 = x3 * x3 - x2 * x2 + y3 * y3 - y2 * y2 + z3 * z3 - z2 * z2;
	double b3 = x4 * x4 - x3 * x3 + y4 * y4 - y3 * y3 + z4 * z4 - z3 * z3;

	double d = a11 * a22 * a33 + a12 * a23 * a31 + a13 * a21 * a32 - a11 * a23 * a32 - a12 * a21 * a33 - a13 * a22 * a31;
	double d1 = b1 * a22 * a33 + a12 * a23 * b3 + a13 * b2 * a32 - b1 * a23 * a32 - a12 * b2 * a33 - a13 * a22 * b3;
	double d2 = a11 * b2 * a33 + b1 * a23 * a31 + a13 * a21 * b3 - a11 * a23 * b3 - b1 * a21 * a33 - a13 * b2 * a31;
	double d3 = a11 * a22 * b3 + a12 * b2 * a31 + b1 * a21 * a32 - a11 * b2 * a32 - a12 * a21 * b3 - b1 * a22 * a31;

	double x = d1 / d;
	double y = d2 / d;
	double z = d3 / d;

	radius2 = (x1 - x) * (x1 - x) + (y1 - y) * (y1 - y) + (z1 - z) * (z1 - z);
	center = { x, y, z };
}

inline void BallDetection::GetNRand(const int maxV, const int N, std::set<int>& idxs)
{
	if (N > maxV) {
		return;
	}

	while (idxs.size() < N) {
		idxs.insert(rand() % maxV);
	}
}

auto BallDetection::sphere_leastFit(const std::vector<cv::Point3d>& points, double & center_x, double & center_y, double & center_z, double & radius)
{
	center_x = 0.0f;
	center_y = 0.0f;
	center_z = 0.0f;

	int N = points.size();
	if (N < 4)
	{
		cout << "Sphere False!!" << endl;
	}

	double sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;

	for (int i = 0; i < points.size(); i++)
	{
		double x = points[i].x;
		double y = points[i].y;
		double z = points[i].z;
		sum_x += x;
		sum_y += y;
		sum_z += z;
	}
	double mean_x = sum_x / N;
	double mean_y = sum_y / N;
	double mean_z = sum_z / N;
	double sum_u2 = 0.0f, sum_v2 = 0.0f, sum_w2 = 0.0f;
	double sum_u3 = 0.0f, sum_v3 = 0.0f, sum_w3 = 0.0f;
	double sum_uv = 0.0f, sum_uw = 0.0f, sum_vw = 0.0f;
	double sum_u2v = 0.0f, sum_uv2 = 0.0f, sum_u2w = 0.0f, sum_v2w = 0.0f, sum_uw2 = 0.0f, sum_vw2 = 0.0f;
	for (int i = 0; i < N; i++)
	{
		double u = points[i].x - mean_x;
		double v = points[i].y - mean_y;
		double w = points[i].z - mean_z;
		double u2 = u * u;
		double v2 = v * v;
		double w2 = w * w;
		sum_u2 += u2;
		sum_v2 += v2;
		sum_w2 += w2;
		sum_u3 += u2 * u;
		sum_v3 += v2 * v;
		sum_w3 += w2 * w;
		sum_uv += u * v;
		sum_vw += v * w;
		sum_uw += u * w;
		sum_u2v += u2 * v;
		sum_u2w += u2 * w;
		sum_uv2 += u * v2;
		sum_v2w += v2 * w;
		sum_uw2 += u * w2;
		sum_vw2 += v * w2;
	}

	double A[3][3];
	double B[3] = { (sum_u3 + sum_uv2 + sum_uw2) / 2.0,
				   (sum_u2v + sum_v3 + sum_vw2) / 2.0,
				   (sum_u2w + sum_v2w + sum_w3) / 2.0 };

	A[0][0] = sum_u2;
	A[0][1] = sum_uv;
	A[0][2] = sum_uw;
	A[1][0] = sum_uv;
	A[1][1] = sum_v2;
	A[1][2] = sum_vw;
	A[2][0] = sum_uw;
	A[2][1] = sum_vw;
	A[2][2] = sum_w2;

	double ans[3];

	if (cagaus((double *)A, B, 3, ans) == 0)
	{
		return false;
	}
	else
	{
		center_x = ans[0] + mean_x;
		center_y = ans[1] + mean_y;
		center_z = ans[2] + mean_z;

		double sum = 0;
		for (int i = 0; i < N; i++)
		{
			double dx = points[i].x - center_x;
			double dy = points[i].y - center_y;
			double dz = points[i].z - center_z;
			sum += dx * dx + dy * dy + dz * dz;
		}
		sum /= N;
		radius = sqrt(sum);
	}
}

double BallDetection::FitCircleByRANSAC(const std::vector<cv::Point3d>& pointArray, cv::Point3d & center, double & radius, const int iterNum, const double e, const float ratio)
{
	const int N = pointArray.size();
	const int targetN = N * ratio;
	int iter = 0;
	std::vector<cv::Point3d> bestInliers;
	while (iter < iterNum) {
		std::set<int> seedIds;
		GetNRand(N, 4, seedIds);  // circle need 4 point
		if (seedIds.size() < 4)
		{
			break;
		}
		std::vector<cv::Point3d> seedPts;
		for (const int idx : seedIds)
		{
			seedPts.push_back(pointArray[idx]);
		}
		cv::Point3d seedCenter;
		double seedR2 = 0.0;
		estimate_sphere(seedPts[0], seedPts[1], seedPts[2], seedPts[3], seedCenter, seedR2);

		std::vector<cv::Point3d> maybeInliers;
		for (const cv::Point3d pt : pointArray)
		{
			if (std::abs((pt.x - seedCenter.x) * (pt.x - seedCenter.x) + (pt.y - seedCenter.y) * (pt.y - seedCenter.y) + (pt.z - seedCenter.z) * (pt.z - seedCenter.z) - seedR2) < e)
			{
				maybeInliers.push_back(pt);
			}
		}

		if (maybeInliers.size() > targetN)
		{
			// it show the inliers is enough
			sphere_leastFit(maybeInliers, center.x, center.y, center.z, radius);
		}
		else
		{
			if (maybeInliers.size() > bestInliers.size()) {
				bestInliers.swap(maybeInliers);
				for (const cv::Point3d pt : seedPts) {
					bestInliers.push_back(pt);
				}
			}
		}

		++iter;
	}
	return sphere_leastFit(bestInliers, center.x, center.y, center.z, radius);
}

auto BallDetection::points_filter(std::vector<cv::Point3d>& points_ROI, std::vector<int>& reflect_ROI)
{
	std::vector<cv::Point3d> ball_points;
	int max_reflect = 0;
	/*for (int ref : reflect_ROI)
	{
		if (max_reflect < ref) max_reflect = ref;
	}*/
	for (int i = 0; i < points_ROI.size(); i++)
	{
		if (max_reflect < reflect_ROI[i]) max_reflect = reflect_ROI[i];
	}

	cv::Point3d max_reflect_pts;
	cv::Point3d max_reflect_point;
	for (int i = 0; i < points_ROI.size(); i++)
	{
		double d0;
		double min_d0 = 100000;
		if (reflect_ROI[i] == max_reflect)
		{
			cout << "反射率为max_reflect的点： (" << points_ROI[i].x << ", " << points_ROI[i].y << ", " << points_ROI[i].z << ")" << endl;
			max_reflect_pts = points_ROI[i];
			d0 = sqrt(max_reflect_pts.x*max_reflect_pts.x + max_reflect_pts.y*max_reflect_pts.y + max_reflect_pts.z*max_reflect_pts.z);
			if (min_d0 > d0)
			{
				min_d0 = d0;
				max_reflect_point = max_reflect_pts;
			}
		}
	}
	cout << "反射率最大的点： (" << max_reflect_point.x << ", " << max_reflect_point.y << ", " << max_reflect_point.z << ")" << endl;

	for (int v = 0; v < points_ROI.size(); v++)
	{
		double deta_x = points_ROI[v].x - max_reflect_point.x;
		double deta_y = points_ROI[v].y - max_reflect_point.y;
		double deta_z = points_ROI[v].z - max_reflect_point.z;
		double d = sqrt(deta_x * deta_x + deta_y * deta_y + deta_z * deta_z);
		if (d <= 0.07)
		{
			ball_points.emplace_back(points_ROI[v]);
		}
	}
	return ball_points;
}

string BallDetection::WstringToString(const std::wstring wstr)
{
	std::string result;
	int len = WideCharToMultiByte(CP_ACP, 0, wstr.c_str(), wstr.size(), NULL, 0, NULL, NULL);
	if (len <= 0)
		return result;

	char* buffer = new char[len + 1];
	if (buffer == NULL)
		return result;

	WideCharToMultiByte(CP_ACP, 0, wstr.c_str(), wstr.size(), buffer, len, NULL, NULL);
	buffer[len] = '\0';
	result.append(buffer);
	delete[] buffer;

	return result;
}

void BallDetection::ReadEncryptionData(std::string path, std::vector<cv::Point3f>& data, std::vector<int> &reflectance)
{
	std::vector<scenePointInfo> encryptData;
	ReadEncryFile(&encryptData, path);
	data.resize(encryptData.size());
	reflectance.resize(encryptData.size());
	for (uint i = 0; i < encryptData.size(); i++)
	{
		data[i].x = encryptData[i].x, data[i].y = encryptData[i].y, data[i].z = encryptData[i].z;
		reflectance[i] = encryptData[i].i;
	}
}

std::vector<std::vector<cv::Point3d>> BallDetection::ROI_map2points(std::string & sense_path)
{
	Read(sense_path);
	alignment(sense_path);

	std::string file_ext = sense_path.substr(sense_path.find_first_of('.'));
	if (file_ext == ".ptsen")
	{
		CloudPointCodec dodec;
		dodec.Decode(sense_path, scene_xyz, scene_reflect);
	}
	else
	{
		if (b_path.size() > 0)
		{
			//#ifdef READ_DATA_ENCRYPTION_MODE
			/*if ((int)scene_path.find("decrypt") != -1)
				BOOL_FUNCTION_CHECK(IOData::Load3DPtCloudData(scene_path, scene_xyz, scene_reflect, false))
			else*/
			ReadEncryptionData(sense_path, scene_xyz, scene_reflect);
		}
	}

	for (int i = 0; i < 6; i++)  //6
	{
		cv::Mat image = get<3>(b_data_all[2][i * 60]);
		int height = image.rows;
		int width = image.cols;
		std::vector<cv::Point2d> points_RGB = balldetection.ballDetection(image);

		if (points_RGB.size() != 0)
		{
			std::vector<std::vector<cv::Point3d>> points_ROI_vec;
			std::vector<std::vector<int>> reflect_ROI_vec;
			std::vector<cv::Point3d> ball_points;
			std::vector<cv::Vec4d> sphere_argument_vec;
			std::vector<cv::Point3d> sphere_center_vec;
			float x_lt = 0.0, y_lt = 0.0, x_rb = 0.0, y_rb = 0.0;
			int nCountP = 0;
			for (int j = 0; j < points_RGB.size(); j++)
			{
				x_lt = points_RGB[j].x;
				y_lt = points_RGB[j].y;
				j += 1;
				x_rb = points_RGB[j].x;
				y_rb = points_RGB[j].y;

				//ofstream out1("C:/Users/zhihui.shi/Desktop/point_v_" + to_string(i) + "-" + to_string(j) + ".txt");
				/*ofstream out2("C:/Users/zhihui.shi/Desktop/balls_points_" + to_string(i) + "-" + to_string(j) + ".txt");*/
				cv::Vec4d sphere_argument;
				cv::Point3d sphere_center;
				std::vector<cv::Point3d> points_ROI; 
				std::vector<int> reflect_ROI;

				if (scene_xyz.size() <= 0)
				{
					cout << "get points failed!" << endl;
				}

				for (int v = 0; v < scene_xyz.size(); v++)
				{
					cv::Point3f point_v = scene_xyz[v];
					int reflect_v = scene_reflect[v];
					Eigen::Vector3d pointE_v = cv3f2eigen3f(point_v).cast<double>();
					Eigen::Vector3d point_v_local = xyz2local(pointE_v, b_data_all[2][i * 60]);
					Eigen::Vector3d point_v_pixel = local2pixel(point_v_local, b_data_all[2][i * 60]);
					point_v_pixel = point_v_pixel.transpose();
					if (point_v_pixel.z() > 0)
					{
						point_v_pixel.array() /= point_v_pixel.z();
						double xv = eigen3d2cv3d(point_v_pixel).x;
						double yv = eigen3d2cv3d(point_v_pixel).y;
						if (xv >= x_lt && xv <= x_rb)
						{
							if (yv >= y_lt && yv <= y_rb)
							{
								nCountP += 1;
								double d_P2O = sqrt(point_v.x*point_v.x + point_v.y*point_v.y + point_v.z*point_v.z);
								if ((d_P2O < 3) & (d_P2O > 0.5))
								{
									//out1 << point_v.x << " " << point_v.y << " " << point_v.z << " " << reflect_v << endl;
									points_ROI.emplace_back(point_v);
									reflect_ROI.emplace_back(reflect_v);
								}
							}
						}
					}
				}
				ball_points = points_filter(points_ROI, reflect_ROI);
				/*for (int k = 0; k < ball_points.size(); k++)
				{
					out2 << ball_points[k].x << " " << ball_points[k].y << " " << ball_points[k].z << endl;
				}*/
				double center_x, center_y, center_z;
				cv::Point3d best_center;
				double best_radius;
				double* circlePara;
				FitCircleByRANSAC(ball_points, best_center, best_radius, 10000, 0.001, 0.95);
				center_x = best_center.x;
				center_y = best_center.y;
				center_z = best_center.z;
				//sphere_leastFit(ball_points, center_x, center_y, center_z, radius);
				//sphere_leastFit(ball_points, center_x, center_y, center_z);
				/*if (best_radius > 0.065)
				{
					sphere_argument = (best_center.x, best_center.y, best_center.z, best_radius);
					sphere_center = cv::Point3d{ best_center.x, best_center.y, best_center.z };
					sphere_argument_vec.emplace_back(sphere_argument);
					sphere_center_vec.emplace_back(sphere_center);
				}*/

				sphere_center = cv::Point3d{ best_center.x, best_center.y, best_center.z };
				sphere_center_vec.emplace_back(sphere_center);

				/*points_ROI_vec.emplace_back(ball_points);
				reflect_ROI_vec.emplace_back(reflect_ROI);*/
			}
			/*points_ROI_vec_all.emplace_back(points_ROI_vec);
			reflect_ROI_vec_all.emplace_back(reflect_ROI_vec);*/
			sphere_argument_vec_all.emplace_back(sphere_argument_vec);
			sphere_center_vec_all.emplace_back(sphere_center_vec);
		}
		else
		{
			cout << "第" << std::to_string(i) << "张图像未检测到靶标球" << endl;
		}
	}

	//return sphere_argument_vec_all;
	return sphere_center_vec_all;
}

void BallDetection::writeInfo_sphere_argument(std::vector<std::vector<cv::Point3d>> sphere_center_vec_all, std::string address)
{
	ofstream out(address + "ballCenter.txt");
	ofstream outpts(address + "ballCenter.pts");
	for (int i = 0; i < sphere_center_vec_all.size(); i++)
	{
		//std::vector<cv::Vec4d> sphere_argument_vec = sphere_argument_vec_all[i];
		std::vector<cv::Point3d> sphere_center_vec = sphere_center_vec_all[i];
		for (int j = 0; j < sphere_center_vec.size(); j++)
		{
			//cv::Vec4d sphere_argument = sphere_center_vec[j];
			cv::Point3d sphere_center = sphere_center_vec[j];
			//out << sphere_argument[0] << " " << sphere_argument[1] << " " << sphere_argument[2] << " " << sphere_argument[3] << endl;
			if (sphere_center.x != 0)
			{
				outpts << sphere_center.x << " " << sphere_center.y << " " << sphere_center.z << endl;
			}
		}
	}
}

