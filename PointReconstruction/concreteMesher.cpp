#ifdef _WIN32
#include <Windows.h>
#endif
#include "Mesh/mesher/detc_edge_api.h"

#include "Mesh/mesher/triangle.h"


#include <time.h>
#include <iostream>
#include <vector>
#include <iterator>
#include <algorithm>
#include <array>
#include <random> 
#include <chrono>

#include <fstream>
#include <string>
#include <regex>
#include <cassert>

#include <unordered_map>
#include <Eigen\Dense> 
 

using namespace std;
using namespace Eigen;
using namespace ModuleStruct;

#include "Mesh/mesher/vec2.h"
#include <opencv2/core/types.hpp>
#include "concreteMesher.h"

Eigen::Matrix3d projectionXY(vector<vector<double>>& inPointCloud, vector<vector<double>>& outPointCloud)
{
    srand((unsigned int)time(0));
    MatrixXd planePoint(10, 3);
    for (size_t i = 0; i < 10; i++)
    {
        auto idx = rand() % inPointCloud.size();
        planePoint.row(i) << inPointCloud[idx][0], inPointCloud[idx][1], inPointCloud[idx][2];
    }
    //planePoint.re 
    auto plane = [&](Vector3d& vPlaneVec, Vector3d& vPlaneCenter) {
        Eigen::RowVector3d meanVec = planePoint.colwise().mean();
        vPlaneCenter = meanVec.transpose();
        Eigen::MatrixXd zeroMeanMat = planePoint;
        zeroMeanMat.rowwise() -= meanVec;
        //Eigen::MatrixXd covMat = zeroMeanMat.transpose()*zeroMeanMat;//是否采用协方差矩阵，效果一样，但需要思考
        Eigen::MatrixXd covMat = zeroMeanMat;
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(covMat, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::Matrix3d V = svd.matrixV();
        Eigen::MatrixXd U = svd.matrixU();
        //Eigen::Matrix3d S = U.inverse() * covMat * V.transpose().inverse();       //why?
        //plane_output.n << V(6), V(7), V(8);
        vPlaneVec << V(6), V(7), V(8);
        //Eigen::MatrixXd D_Mat = plane_output.n * meanVec.transpose();
        //plane_output.D = D_Mat(0);
    };

    Eigen::Vector3d vPlanevec;
    Eigen::Vector3d vPlaneCenter;
    plane(vPlanevec, vPlaneCenter);
    Eigen::Vector3d vectorAfter(0., 0., 1.);

    auto rotMatrix = (Eigen::Quaterniond::FromTwoVectors(vPlanevec, vectorAfter)).toRotationMatrix();
    //auto rotMatrix_ = Eigen::Quaterniond::FromTwoVectors(vPlanevec, vectorAfter).matrix();

    //auto reversed_rotMatrix = rotMatrix.reverse();
    //auto reversed_rotMatrix_ = Eigen::Quaterniond::FromTwoVectors(vectorAfter, vPlanevec).toRotationMatrix();

    Matrix3d rotMatrix_1 = rotMatrix.reverse();
    for (auto x : inPointCloud)
    {
        //move
        Eigen::Vector3d axi1Before(x[0] - vPlaneCenter[0], x[1] - vPlaneCenter[1], x[2] - vPlaneCenter[2]);
        Eigen::Vector3d axi1After = rotMatrix * axi1Before;
        outPointCloud.push_back({ (float)axi1After.x(),(float)axi1After.y(),(float)axi1After.z() });
        //cout << axi1After.x() << "," << axi1After.y() << "," << axi1After.z() << "," << endl;
    }
    return rotMatrix_1;
}

Eigen::Matrix4d projectionXY_(const vector<vector<double>>& inPointCloud, vector<vector<double>>& outPointCloud)
{
    MatrixXd planePoint(20, 3);
    for (size_t i = 0; i < 20; i++)
    {
        auto idx = rand() % inPointCloud.size();
        planePoint.row(i) << inPointCloud[idx][0], inPointCloud[idx][1], inPointCloud[idx][2];
    }
    //planePoint.re
    auto plane = [&](Vector3d& vPlaneVec, Vector3d& vPlaneCenter) {
        Eigen::RowVector3d meanVec = planePoint.colwise().mean();
        vPlaneCenter = meanVec.transpose();
        Eigen::MatrixXd zeroMeanMat = planePoint;
        zeroMeanMat.rowwise() -= meanVec;
        //Eigen::MatrixXd covMat = zeroMeanMat.transpose()*zeroMeanMat;//是否采用协方差矩阵，效果一样，但需要思考
        Eigen::MatrixXd covMat = zeroMeanMat;
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(covMat, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::Matrix3d V = svd.matrixV();
        Eigen::MatrixXd U = svd.matrixU();
        //Eigen::Matrix3d S = U.inverse() * covMat * V.transpose().inverse();       //why?
        //plane_output.n << V(6), V(7), V(8);
        vPlaneVec << V(6), V(7), V(8);
        //Eigen::MatrixXd D_Mat = plane_output.n * meanVec.transpose();
        //plane_output.D = D_Mat(0);
    };

    Eigen::Vector3d vPlanevec;
    Eigen::Vector3d vPlaneCenter;
    plane(vPlanevec, vPlaneCenter);
    Eigen::Vector3d vectorAfter(0., 0., 1.);

    auto rotMatrix = (Eigen::Quaterniond::FromTwoVectors(vPlanevec, vectorAfter)).toRotationMatrix();
    auto rotMatrix_ = Eigen::Quaterniond::FromTwoVectors(vPlanevec, vectorAfter).matrix();

    Matrix3d reversed_rotMatrix;// = rotMatrix.inverse();
    Matrix3d reversed_rotMatrix_ = Eigen::Quaterniond::FromTwoVectors(vectorAfter, vPlanevec).toRotationMatrix();

    Matrix3d rotMatrix_3 = rotMatrix.inverse();
    for (auto x : inPointCloud)
    {
        //move
        Eigen::Vector3d axi1Before(x[0] - vPlaneCenter[0], x[1] - vPlaneCenter[1], x[2] - vPlaneCenter[2]);
        Eigen::Vector3d axi1After = rotMatrix * axi1Before;
        outPointCloud.push_back({ (float)axi1After.x(),(float)axi1After.y(),(float)axi1After.z() });
        //cout << axi1After.x() << "," << axi1After.y() << "," << axi1After.z() << "," << endl;
    }
    //ofstream outPoints("D:/data/outmesh_30_test/outPoint.pts");
    //for(auto x:inPointCloud)
    //    outPoints << x[0] << " " << x[1] << " " << x[2]<< endl;
    //outPoints.close();
    Matrix4d rotMatrix_4_test;
    //rotMatrix_4_test << rotMatrix(0, 0), rotMatrix(0, 1), rotMatrix(0, 2), -vPlaneCenter[0],
    //    rotMatrix(1, 0), rotMatrix_3(1, 1), rotMatrix(1, 2), -vPlaneCenter[1],
    //    rotMatrix(2, 0), rotMatrix_3(2, 1), rotMatrix(2, 2), -vPlaneCenter[2],
    //    0, 0, 0, 1;
    rotMatrix_4_test << 1, 0, 0, vPlaneCenter[0],
        0, 1, 0, vPlaneCenter[1],
        0, 0, 1, vPlaneCenter[2],
        0, 0, 0, 1;
    auto rotMatrix_4_test_result = rotMatrix_4_test.inverse();
    Matrix4d rotMatrix_4;
    rotMatrix_4 << rotMatrix_3(0, 0), rotMatrix_3(0, 1), rotMatrix_3(0, 2), vPlaneCenter[0],
        rotMatrix_3(1, 0), rotMatrix_3(1, 1), rotMatrix_3(1, 2), vPlaneCenter[1],
        rotMatrix_3(2, 0), rotMatrix_3(2, 1), rotMatrix_3(2, 2), vPlaneCenter[2],
        0, 0, 0, 1;

    return rotMatrix_4;
}

Eigen::Matrix4d projectionXY_(std::vector<cv::Point3f>& inPointCloud, std::vector<cv::Point3f>& outPointCloud)
{
    MatrixXd planePoint(20, 3);
    for (size_t i = 0; i < 20; i++)
    {
        auto idx = rand() % inPointCloud.size();
        planePoint.row(i) << inPointCloud[idx].x, inPointCloud[idx].y, inPointCloud[idx].z;
    }
    //planePoint.re
    auto plane = [&](Vector3d& vPlaneVec, Vector3d& vPlaneCenter) {
        Eigen::RowVector3d meanVec = planePoint.colwise().mean();
        vPlaneCenter = meanVec.transpose();
        Eigen::MatrixXd zeroMeanMat = planePoint;
        zeroMeanMat.rowwise() -= meanVec;
        //Eigen::MatrixXd covMat = zeroMeanMat.transpose()*zeroMeanMat;//是否采用协方差矩阵，效果一样，但需要思考
        Eigen::MatrixXd covMat = zeroMeanMat;
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(covMat, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::Matrix3d V = svd.matrixV();
        Eigen::MatrixXd U = svd.matrixU();
        //Eigen::Matrix3d S = U.inverse() * covMat * V.transpose().inverse();       //why?
        //plane_output.n << V(6), V(7), V(8);
        vPlaneVec << V(6), V(7), V(8);
        //Eigen::MatrixXd D_Mat = plane_output.n * meanVec.transpose();
        //plane_output.D = D_Mat(0);
    };

    Eigen::Vector3d vPlanevec;
    Eigen::Vector3d vPlaneCenter;
    plane(vPlanevec, vPlaneCenter);
    Eigen::Vector3d vectorAfter(0., 0., 1.);

    auto rotMatrix = (Eigen::Quaterniond::FromTwoVectors(vPlanevec, vectorAfter)).toRotationMatrix();
    auto rotMatrix_ = Eigen::Quaterniond::FromTwoVectors(vPlanevec, vectorAfter).matrix();

    Matrix3d reversed_rotMatrix;// = rotMatrix.inverse();
    Matrix3d reversed_rotMatrix_ = Eigen::Quaterniond::FromTwoVectors(vectorAfter, vPlanevec).toRotationMatrix();

    Matrix3d rotMatrix_3 = rotMatrix.inverse();
    for (auto x : inPointCloud)
    {
        //move
        Eigen::Vector3d axi1Before(x.x - vPlaneCenter[0], x.y - vPlaneCenter[1], x.z - vPlaneCenter[2]);
        Eigen::Vector3d axi1After = rotMatrix * axi1Before;
        outPointCloud.push_back({ (float)axi1After.x(),(float)axi1After.y(),(float)axi1After.z() });
        //cout << axi1After.x() << "," << axi1After.y() << "," << axi1After.z() << "," << endl;
    }
    //ofstream outPoints("D:/data/outmesh_30_test/outPoint.pts");
    //for (auto x : inPointCloud)
    //    outPoints << x[0] << " " << x[1] << " " << x[2] << endl;
    //outPoints.close();
    Matrix4d rotMatrix_4_test;
    //rotMatrix_4_test << rotMatrix(0, 0), rotMatrix(0, 1), rotMatrix(0, 2), -vPlaneCenter[0],
    //    rotMatrix(1, 0), rotMatrix_3(1, 1), rotMatrix(1, 2), -vPlaneCenter[1],
    //    rotMatrix(2, 0), rotMatrix_3(2, 1), rotMatrix(2, 2), -vPlaneCenter[2],
    //    0, 0, 0, 1;
    rotMatrix_4_test << 1, 0, 0, vPlaneCenter[0],
        0, 1, 0, vPlaneCenter[1],
        0, 0, 1, vPlaneCenter[2],
        0, 0, 0, 1;
    auto rotMatrix_4_test_result = rotMatrix_4_test.inverse();
    Matrix4d rotMatrix_4;
    rotMatrix_4 << rotMatrix_3(0, 0), rotMatrix_3(0, 1), rotMatrix_3(0, 2), vPlaneCenter[0],
        rotMatrix_3(1, 0), rotMatrix_3(1, 1), rotMatrix_3(1, 2), vPlaneCenter[1],
        rotMatrix_3(2, 0), rotMatrix_3(2, 1), rotMatrix_3(2, 2), vPlaneCenter[2],
        0, 0, 0, 1;

    return rotMatrix_4;
}

Eigen::Matrix4d projectionXY_(std::vector<cv::Point3f>& inPointCloud, cv::Point3f inNormal,std::vector<cv::Point3f>& outPointCloud, cv::Point3f outNormal )
{
	MatrixXd planePoint(20, 3);
	for (size_t i = 0; i < 20; i++)
	{
		auto idx = rand() % inPointCloud.size();
		planePoint.row(i) << inPointCloud[idx].x, inPointCloud[idx].y, inPointCloud[idx].z;
	}
	//planePoint.re
	auto plane = [&](Vector3d& vPlaneVec, Vector3d& vPlaneCenter) {
		Eigen::RowVector3d meanVec = planePoint.colwise().mean();
		vPlaneCenter = meanVec.transpose();
		Eigen::MatrixXd zeroMeanMat = planePoint;
		zeroMeanMat.rowwise() -= meanVec;
		//Eigen::MatrixXd covMat = zeroMeanMat.transpose()*zeroMeanMat;//是否采用协方差矩阵，效果一样，但需要思考
		Eigen::MatrixXd covMat = zeroMeanMat;
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(covMat, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::Matrix3d V = svd.matrixV();
		Eigen::MatrixXd U = svd.matrixU();
		//Eigen::Matrix3d S = U.inverse() * covMat * V.transpose().inverse();       //why?
		//plane_output.n << V(6), V(7), V(8);
		vPlaneVec << V(6), V(7), V(8);
		//Eigen::MatrixXd D_Mat = plane_output.n * meanVec.transpose();
		//plane_output.D = D_Mat(0);
	};

	Eigen::Vector3d vPlanevec(inNormal.x, inNormal.y, inNormal.z );
	Eigen::Vector3d vPlaneCenter{ planePoint.block(0,0,10,1).mean(), planePoint.block(0,1,10,1).mean(), planePoint.block(0,2,10,1).mean() };
	//plane(vPlanevec, vPlaneCenter);
	Eigen::Vector3d vectorAfter(outNormal.x, outNormal.y, outNormal.z);

	auto rotMatrix = (Eigen::Quaterniond::FromTwoVectors(vPlanevec, vectorAfter)).toRotationMatrix();
	auto rotMatrix_ = Eigen::Quaterniond::FromTwoVectors(vPlanevec, vectorAfter).matrix();

	Matrix3d reversed_rotMatrix;// = rotMatrix.inverse();
	Matrix3d reversed_rotMatrix_ = Eigen::Quaterniond::FromTwoVectors(vectorAfter, vPlanevec).toRotationMatrix();

	Matrix3d rotMatrix_3 = rotMatrix.inverse();
	for (auto x : inPointCloud)
	{
		//move
		Eigen::Vector3d axi1Before(x.x - vPlaneCenter[0], x.y - vPlaneCenter[1], x.z - vPlaneCenter[2]);
		Eigen::Vector3d axi1After = rotMatrix * axi1Before;
		outPointCloud.push_back({ (float)axi1After.x(),(float)axi1After.y(),(float)axi1After.z() });
		//cout << axi1After.x() << "," << axi1After.y() << "," << axi1After.z() << "," << endl;
	}
	//ofstream outPoints("D:/data/outmesh_30_test/outPoint.pts");
	//for (auto x : inPointCloud)
	//    outPoints << x[0] << " " << x[1] << " " << x[2] << endl;
	//outPoints.close();
	Matrix4d rotMatrix_4_test;
	//rotMatrix_4_test << rotMatrix(0, 0), rotMatrix(0, 1), rotMatrix(0, 2), -vPlaneCenter[0],
	//    rotMatrix(1, 0), rotMatrix_3(1, 1), rotMatrix(1, 2), -vPlaneCenter[1],
	//    rotMatrix(2, 0), rotMatrix_3(2, 1), rotMatrix(2, 2), -vPlaneCenter[2],
	//    0, 0, 0, 1;
	rotMatrix_4_test << 1, 0, 0, vPlaneCenter[0],
		0, 1, 0, vPlaneCenter[1],
		0, 0, 1, vPlaneCenter[2],
		0, 0, 0, 1;
	auto rotMatrix_4_test_result = rotMatrix_4_test.inverse();
	Matrix4d rotMatrix_4;
	rotMatrix_4 << rotMatrix_3(0, 0), rotMatrix_3(0, 1), rotMatrix_3(0, 2), vPlaneCenter[0],
		rotMatrix_3(1, 0), rotMatrix_3(1, 1), rotMatrix_3(1, 2), vPlaneCenter[1],
		rotMatrix_3(2, 0), rotMatrix_3(2, 1), rotMatrix_3(2, 2), vPlaneCenter[2],
		0, 0, 0, 1;

	return rotMatrix_4;
}
void GetDotCloud(vector<vector<double>>& points, string filename)
{
    ifstream file(filename);

    double x = 0, y = 0, z = 0;
    int red = 0, green = 0, blue = 0;
	string head;
	for (auto i = 0; i < 7; i++)
		getline(file,head);
    while (file)
    {
        char tempChar;
        file >> x >> y >> z;
        vector<double> temp = { x,y,z };
        points.push_back(temp);
        //duplicate removal
    }
    //points.pop_back();
#ifdef true
    sort(duplicate_removal.begin(), duplicate_removal.end());
    //auto it = unique(duplicate_removal.begin(), duplicate_removal.end());
    auto duplicate_end = duplicate_removal.begin();
    for (auto itl = duplicate_removal.begin() + 1; itl != duplicate_removal.end(); itl++)
        if (*itl == *(itl - 1))
            (*duplicate_end++) = *itl;

    //for (auto lit = it; lit != duplicate_removal.end(); lit++)
        //points.erase(remove(points.begin(), points.end(),lit));
    for (auto itp = points.begin(); itp != points.end(); itp++)
    {
        for (auto lit = duplicate_removal.begin(); lit != duplicate_end + 1; lit++)
            if (static_cast<int>((*itp)[0]) == (*lit).first &&
                static_cast<int>((*itp)[1]) == (*lit).second)
                itp = points.erase(itp);
    }

    //vecprint.push_back(vec[0]);
    //for (int i = 1; i < vec.size(); i++)
    //{
    //    bool tag = true;
    //    for (int j = 0; j < i; j++)
    //    {
    //        if (vec[i] == vec[j])
    //        {
    //            tag = false;
    //            break;
    //        }
    //    }
    //    if (tag == true)	vecprint.push_back(vec[i]);
    //}
#endif
    file.close();
}
bool get(vector<vector<float>>& inCloud, vector<vector<float>>& outCloud)
{
    //box
    float minX = inCloud[0][0], minY = inCloud[0][1], minZ = 0;
    float maxX = inCloud[0][0], maxY = inCloud[0][1], maxZ = 0;
    for (auto x : inCloud)
    {
        minX = min(x[0], minX);
        minY = min(x[1], minY);
        maxX = max(x[0], maxX);
        maxY = max(x[1], maxY);
    }
    //hash
    int diagonal = 60;
    int offset0_5 = floor(maxX - minX) / diagonal / 20;
    int offset = offset0_5 * 2;
    int projectX = floor(maxX - minX) / diagonal + offset + 1, projectY = floor(maxY - minY) / diagonal + offset + 1;
    //vector<int> map(projectX * projectY, 0);
    //for (auto x : *inCloud)
    //{
    //    map[(floor(x.x - minX) / diagonal + offset0_5-1) * projectY + floor(x.y - minY) / diagonal + offset0_5-1]++;
    //}
    ////
    //for (int x_y = 0; x_y < map.size(); x_y++)
    //{
    //    int x = x_y+1 / projectY;
    //    int y = x_y+1 % projectY;
    //    if (x == 0 || y == 0 || x == projectX-1 || y == projectY-1)
    //        continue;
    //    if (map[x * projectY+y] > 1 && (map[(x - 1) * projectY + y - 1] < 2 ||
    //                                    map[(x - 1) * projectY + y] < 2 ||
    //                                    map[(x - 1) * projectY + y+1] < 2 ||
    //                                    map[(x) * projectY + y-1] < 2 ||
    //                                    map[(x) * projectY + y+1] < 2 ||
    //                                    map[(x + 1) * projectY + y-1] < 2 ||
    //                                    map[(x + 1) * projectY + y] < 2 ||
    //                                    map[(x + 1) * projectY + y+1] < 2))
    //        map[x*projectY +y]=-map[x* projectY +y];
    //}

    //for (auto x : *inCloud)
    //{
    //    if(map[(floor(x.x - minX) / diagonal + offset0_5-1) * projectX+ floor(x.y - minY) / diagonal + offset0_5]<0)
    //        outCloud->push_back(x);
    //}
    vector<vector<int>> map2(projectX, vector<int>(projectY, 0));
    vector<vector<int>> map2_flag(projectX, vector<int>(projectY, 0));
    for (auto x : inCloud)
        map2[floor(x[0] - minX) / diagonal + offset0_5][floor(x[1] - minY) / diagonal + offset0_5]++;

    ofstream temp("D:\\data\\map2.txt");
    for (int i = 0; i < map2.size(); i++)
    {
        for (int j = 0; j < map2.begin()->size(); j++)
        {
            temp << map2[i][j] << " ";
        }
        temp << endl;
    }
    temp.close();

    for (int i = 1; i < map2.size() - 1; i++)
    {
        for (int j = 1; j < map2.begin()->size() - 1; j++)
        {
            /*if (map2[i][j] > 1 && (
                map2[i - 1][j - 1] < 2 ||
                map2[i - 1][j] < 2 ||
                map2[i - 1][j + 1] < 2 ||
                map2[i][j - 1] < 2 ||
                map2[i][j + 1] < 2 ||
                map2[i + 1][j - 1] < 2 ||
                map2[i + 1][j] < 2 ||
                map2[i + 1][j + 1] < 2
                ))
                map2_flag[i][j] = -map2[i][j];*/
            if (map2[i][j] > 1)
            {
                int lineWeight = (
                    int(map2[i - 1][j - 1] < 2) +
                    int(map2[i - 1][j] < 2) +
                    int(map2[i - 1][j + 1] < 2) +
                    int(map2[i][j - 1] < 2) +
                    int(map2[i][j + 1] < 2) +
                    int(map2[i + 1][j - 1] < 2) +
                    int(map2[i + 1][j] < 2) +
                    int(map2[i + 1][j + 1] < 2)
                    );
                map2_flag[i][j] = lineWeight;

                //int cenWeight = (
                //    int(map2[i - 1][j - 1] > 2) +
                //    int(map2[i - 1][j] > 2) +
                //    int(map2[i - 1][j + 1] > 2) +
                //    int(map2[i][j - 1] > 2) +
                //    int(map2[i][j + 1] > 2) +
                //    int(map2[i + 1][j - 1] > 2) +
                //    int(map2[i + 1][j] > 2) +
                //    int(map2[i + 1][j + 1] > 2)
                //    );
                //if (cenWeight >= 6 && (i%2==0&&j%2==0))
                //    map2_flag[i][j] = -1;
            }
        }
    }

    std::unordered_map<string, vector<float>> result;
    for (auto x : inCloud)
    {
        int i = floor(x[0] - minX) / diagonal + offset0_5;
        int j = floor(x[1] - minY) / diagonal + offset0_5;
        auto temp = map2_flag[i][j];
        if (temp > 1 && temp < 5 || temp < 0)
        {
            //outCloud->push_back(x);
            if (result.find(to_string(i) + "_" + to_string(j)) != result.end())
            {
                vector<float> temp(result[to_string(i) + "_" + to_string(j)]);
                temp[0] = 0.5 * (temp[0] + x[0]);
                temp[1] = 0.5 * (temp[1] + x[1]);
                result[to_string(i) + "_" + to_string(j)] = temp;
            }
            else
                result.insert(make_pair(to_string(i) + "_" + to_string(j), x));
        }
    }
    for (auto itr = result.begin(); itr != result.end(); itr++)
        outCloud.push_back(itr->second);
    return true;
}

bool get(vector<vector<float>>& inCloud, vector<vector<float>>& outCloud, vector<vector<vector<int>>>& mapPointId)
{
    //box
    float minX = inCloud[0][0], minY = inCloud[0][1], minZ = 0;
    float maxX = inCloud[0][0], maxY = inCloud[0][1], maxZ = 0;
    for (auto x : inCloud)
    {
        minX = min(x[0], minX);
        minY = min(x[1], minY);
        maxX = max(x[0], maxX);
        maxY = max(x[1], maxY);
    }
    //hash
    int diagonal = 60;
    int offset0_5 = floor(maxX - minX) / diagonal / 20;
    int offset = offset0_5 * 2;
    int projectX = floor(maxX - minX) / diagonal + offset + 1, projectY = floor(maxY - minY) / diagonal + offset + 1;
    //vector<int> map(projectX * projectY, 0);
    //for (auto x : *inCloud)
    //{
    //    map[(floor(x.x - minX) / diagonal + offset0_5-1) * projectY + floor(x.y - minY) / diagonal + offset0_5-1]++;
    //}
    ////
    //for (int x_y = 0; x_y < map.size(); x_y++)
    //{
    //    int x = x_y+1 / projectY;
    //    int y = x_y+1 % projectY;
    //    if (x == 0 || y == 0 || x == projectX-1 || y == projectY-1)
    //        continue;
    //    if (map[x * projectY+y] > 1 && (map[(x - 1) * projectY + y - 1] < 2 ||
    //                                    map[(x - 1) * projectY + y] < 2 ||
    //                                    map[(x - 1) * projectY + y+1] < 2 ||
    //                                    map[(x) * projectY + y-1] < 2 ||
    //                                    map[(x) * projectY + y+1] < 2 ||
    //                                    map[(x + 1) * projectY + y-1] < 2 ||
    //                                    map[(x + 1) * projectY + y] < 2 ||
    //                                    map[(x + 1) * projectY + y+1] < 2))
    //        map[x*projectY +y]=-map[x* projectY +y];
    //}

    //for (auto x : *inCloud)
    //{
    //    if(map[(floor(x.x - minX) / diagonal + offset0_5-1) * projectX+ floor(x.y - minY) / diagonal + offset0_5]<0)
    //        outCloud->push_back(x);
    //}
    vector<vector<int>> map2(projectX, vector<int>(projectY, 0));
    vector<vector<int>> map2_flag(projectX, vector<int>(projectY, 0));
    mapPointId.resize(projectX, vector<vector<int>>(projectY));
    int index = 0;
    for (auto x : inCloud)
    {
        map2[floor(x[0] - minX) / diagonal + offset0_5][floor(x[1] - minY) / diagonal + offset0_5]++;
        mapPointId[floor(x[0] - minX) / diagonal + offset0_5][floor(x[1] - minY) / diagonal + offset0_5].push_back(index++);
    }


    ofstream temp("D:\\data\\map2.txt");
    for (int i = 0; i < map2.size(); i++)
    {
        for (int j = 0; j < map2.begin()->size(); j++)
        {
            temp << map2[i][j] << " ";
        }
        temp << endl;
    }
    temp.close();

    for (int i = 1; i < map2.size() - 1; i++)
    {
        for (int j = 1; j < map2.begin()->size() - 1; j++)
        {
            /*if (map2[i][j] > 1 && (
                map2[i - 1][j - 1] < 2 ||
                map2[i - 1][j] < 2 ||
                map2[i - 1][j + 1] < 2 ||
                map2[i][j - 1] < 2 ||
                map2[i][j + 1] < 2 ||
                map2[i + 1][j - 1] < 2 ||
                map2[i + 1][j] < 2 ||
                map2[i + 1][j + 1] < 2
                ))
                map2_flag[i][j] = -map2[i][j];*/
            if (map2[i][j] > 1)
            {
                int lineWeight = (
                    int(map2[i - 1][j - 1] < 2) +
                    int(map2[i - 1][j] < 2) +
                    int(map2[i - 1][j + 1] < 2) +
                    int(map2[i][j - 1] < 2) +
                    int(map2[i][j + 1] < 2) +
                    int(map2[i + 1][j - 1] < 2) +
                    int(map2[i + 1][j] < 2) +
                    int(map2[i + 1][j + 1] < 2)
                    );
                map2_flag[i][j] = lineWeight;

                //int cenWeight = (
                //    int(map2[i - 1][j - 1] > 2) +
                //    int(map2[i - 1][j] > 2) +
                //    int(map2[i - 1][j + 1] > 2) +
                //    int(map2[i][j - 1] > 2) +
                //    int(map2[i][j + 1] > 2) +
                //    int(map2[i + 1][j - 1] > 2) +
                //    int(map2[i + 1][j] > 2) +
                //    int(map2[i + 1][j + 1] > 2)
                //    );
                //if (cenWeight >= 6 && (i%2==0&&j%2==0))
                //    map2_flag[i][j] = -1;
            }
        }
    }

    std::unordered_map<string, vector<float>> result;
    for (auto x : inCloud)
    {
        int i = floor(x[0] - minX) / diagonal + offset0_5;
        int j = floor(x[1] - minY) / diagonal + offset0_5;
        auto temp = map2_flag[i][j];
        if (temp > 1 && temp < 5 || temp < 0)
        {
            //outCloud->push_back(x);
            if (result.find(to_string(i) + "_" + to_string(j)) != result.end())
            {
                vector<float> temp(result[to_string(i) + "_" + to_string(j)]);
                temp[0] = 0.5 * (temp[0] + x[0]);
                temp[1] = 0.5 * (temp[1] + x[1]);
                result[to_string(i) + "_" + to_string(j)] = temp;
            }
            else
                result.insert(make_pair(to_string(i) + "_" + to_string(j), x));
        }
    }
    for (auto itr = result.begin(); itr != result.end(); itr++)
        outCloud.push_back(itr->second);
    return true;
}

vector<int> split(const string& str, const string& delim) {
    vector<int> res;
    if ("" == str) return res;
    //先将要切割的字符串从string类型转换为char*类型  
    char* strs = new char[str.length() + 1]; //不要忘了  
    strcpy(strs, str.c_str());

    char* d = new char[delim.length() + 1];
    strcpy(d, delim.c_str());

    char* p = strtok(strs, d);
    while (p) {
        int s = stoi(p); //分割得到的字符串转换为string类型  
        res.push_back(s); //存入结果数组  
        p = strtok(NULL, d);
    }

    return res;
}

void GetMap(vector<vector<int>>& map)
{
    ifstream temp("D:\\data\\map2.txt");
    while (temp)
    {
        string tempStr;
        getline(temp, tempStr);
        //regex tempReg(tempStr.c_str());
        vector<int> tempVec = split(tempStr, " ");
        map.push_back(tempVec);
    }
    map.pop_back();
    temp.close();
}
void  GetDotCloud(vector<vector<double>>& points)
{
    string filename = "D:\\data\\EdgePointOnly.pts";
    //filename = "D:\\data\\linesData.pts";

    cout << "Enter name of file in resource directory: ";
    //cin >> filename;
    //filename = "Resource\\" + filename;
    ifstream file(filename);

    double x = 0, y = 0, z = 0;
    int red = 0, green = 0, blue = 0;
    while (file)
    {
        //char temp;
        file >> x >> y >> z;
        vector<double> temp = { x,y,z };
        points.push_back(temp);
    }
    file.close();
}
triangulateio inDotCloud(vector<vector<double>>& points)
{
    struct triangulateio in;

    in.numberofpoints = points.size();
    in.pointlist = (REAL*)malloc(in.numberofpoints * 2 * sizeof(REAL));
    int i = 0;
    for (auto x : points)
    {
        in.pointlist[i++] = x[0];
        in.pointlist[i++] = x[1];
        //cout << y << endl;
    }


    in.numberofpointattributes = 0;
    //in.pointattributelist = (REAL*)malloc(in.numberofpoints *
    //    in.numberofpointattributes *
    //    sizeof(REAL));
    //in.pointattributelist[0] = 0.0;
    //in.pointattributelist[1] = 1.0;
    //in.pointattributelist[2] = 11.0;
    //in.pointattributelist[3] = 10.0;
    in.pointmarkerlist = (int*)NULL;
    //in.pointmarkerlist[0] = 100;
    //in.pointmarkerlist[1] = 200;
    //in.pointmarkerlist[2] = 100;
    //in.pointmarkerlist[3] = 1000;

    //segEdge
    in.numberofsegments = points.size();
    in.segmentlist = (int*)malloc(in.numberofpoints * 2 * sizeof(int));
    for (int i = 0; i < in.numberofsegments; i++)
    {
        if (i == in.numberofsegments - 1)
        {
            in.segmentlist[2 * i] = i;
            in.segmentlist[2 * i + 1] = 0;
        }
        else
        {
            in.segmentlist[2 * i] = i;
            in.segmentlist[2 * i + 1] = i + 1;
        }
    }
    in.segmentmarkerlist = (int*)malloc(in.numberofsegments * sizeof(int));
    //for (size_t i = 0; i < in.numberofsegments; i++)
    //{   
    //    in.segmentmarkerlist[i] = 1;
    //}
    in.numberofholes = 0;
    in.numberofregions = 1;
    in.regionlist = (REAL*)malloc(in.numberofregions * 4 * sizeof(REAL));
    in.regionlist[0] = 0.5;
    in.regionlist[1] = 5.0;
    in.regionlist[2] = 7.0;            /* Regional attribute (for whole mesh). */
    in.regionlist[3] = 0.1;          /* Area constraint that will not be used. */
    return in;
}

triangulateio inDotCloud(vector<vector<double>>& points, vector<vector<double>>& hole)
{
    struct triangulateio in;

    in.numberofpoints = points.size() + hole.size();
    in.pointlist = (REAL*)malloc(in.numberofpoints * 2 * sizeof(REAL));
    int i = 0;
    for (auto x : points)
    {
        in.pointlist[i++] = x[0];
        in.pointlist[i++] = x[1];
        //cout << y << endl;
    }
    for (auto x : hole)
    {
        in.pointlist[i++] = x[0];
        in.pointlist[i++] = x[1];
        //cout << y << endl;
    }

    in.numberofpointattributes = 0;
    //in.pointattributelist = (REAL*)malloc(in.numberofpoints *
    //    in.numberofpointattributes *
    //    sizeof(REAL));
    //in.pointattributelist[0] = 0.0;
    //in.pointattributelist[1] = 1.0;
    //in.pointattributelist[2] = 11.0;
    //in.pointattributelist[3] = 10.0;
    in.pointmarkerlist = (int*)NULL;
    //in.pointmarkerlist[0] = 100;
    //in.pointmarkerlist[1] = 200;
    //in.pointmarkerlist[2] = 100;
    //in.pointmarkerlist[3] = 1000;

    //segEdge
    in.numberofsegments = points.size() + hole.size();
    in.segmentlist = (int*)malloc(in.numberofpoints * 2 * sizeof(int));
    for (int i = 0; i < in.numberofsegments; i++)
    {
        if (i == in.numberofsegments - 1)
        {
            in.segmentlist[2 * i] = i;
            in.segmentlist[2 * i + 1] = 0;
        }
        else
        {
            in.segmentlist[2 * i] = i;
            in.segmentlist[2 * i + 1] = i + 1;
        }
    }
    in.segmentmarkerlist = (int*)malloc(in.numberofsegments * sizeof(int));
    for (size_t i = 0; i < in.numberofsegments; i++)
    {
        in.segmentmarkerlist[i] = 1;
    }

    //in.numberofholes=hole.size();                                      /* In / copied out */
    //in.holelist = (REAL* )malloc(in.numberofholes*2*sizeof(REAL));                        /* In / pointer to array copied out */
    //int j = 0;
    //for (auto x : hole)
    //{
    //    in.holelist[j++] = x[0];
    //    in.holelist[j++] = x[1];
    //    //cout << y << endl;
    //}

    in.numberofregions = 1;
    in.regionlist = (REAL*)malloc(in.numberofregions * 4 * sizeof(REAL));
    in.regionlist[0] = 0.5;
    in.regionlist[1] = 5.0;
    in.regionlist[2] = 7.0;            /* Regional attribute (for whole mesh). */
    in.regionlist[3] = 0.1;          /* Area constraint that will not be used. */
    return in;
}

triangulateio inDotCloud(vector<vector<double>>& points, vector<vector<vector<double>>>& holes)
{
    struct triangulateio in;

    //where boundery
    in.numberofpoints = points.size();
    for (auto x : holes)
        in.numberofpoints += x.size();
    in.pointlist = (REAL*)malloc(in.numberofpoints * 2 * sizeof(REAL));
    int i = 0;
    for (auto x : points)
    {
        in.pointlist[i++] = x[0];
        in.pointlist[i++] = x[1];
        //cout << y << endl;
    }
    for (auto y : holes)
        for (auto x : y)
        {
            in.pointlist[i++] = x[0];
            in.pointlist[i++] = x[1];
            //cout << y << endl;
        }

    in.numberofpointattributes = 0;
    //in.pointattributelist = (REAL*)malloc(in.numberofpoints *
    //    in.numberofpointattributes *
    //    sizeof(REAL));
    //in.pointattributelist[0] = 0.0;
    //in.pointattributelist[1] = 1.0;
    //in.pointattributelist[2] = 11.0;
    //in.pointattributelist[3] = 10.0;
    in.pointmarkerlist = (int*)NULL;
    //in.pointmarkerlist[0] = 100;
    //in.pointmarkerlist[1] = 200;
    //in.pointmarkerlist[2] = 100;
    //in.pointmarkerlist[3] = 1000;

    //segEdge
    in.numberofsegments = in.numberofpoints;
    in.segmentlist = (int*)malloc(in.numberofpoints * 2 * sizeof(int));

    int begin = 0;
    int end = points.size();
    int edgeNum = 0;
    for (int i = 0; i < in.numberofsegments; i++)
    {
        if (i == end - 1)
        {
            in.segmentlist[2 * i] = i;
            in.segmentlist[2 * i + 1] = begin;
            begin = end;
            edgeNum < holes.size() ? end += holes[edgeNum++].size() : end;
        }
        else
        {
            in.segmentlist[2 * i] = i;
            in.segmentlist[2 * i + 1] = i + 1;
        }
    }
    in.segmentmarkerlist = (int*)malloc(in.numberofsegments * sizeof(int));
    //for (size_t i = 0; i < in.numberofsegments; i++)
    //{   
    //    in.segmentmarkerlist[i] = 1;
    //}

    in.numberofholes = holes.size();                                      /* In / copied out */
    in.holelist = (REAL*)malloc(in.numberofholes * 2 * sizeof(REAL));                        /* In / pointer to array copied out */
    int j = 0;
    for (auto x : holes)
    {
        in.holelist[j++] = (x[0][0] + x[1][0] + x[2][0]) / 3;
        in.holelist[j++] = (x[0][1] + x[1][1] + x[2][1]) / 3;
        //cout << y << endl;
    }

    in.numberofregions = 1;
    in.regionlist = (REAL*)malloc(in.numberofregions * 4 * sizeof(REAL));
    in.regionlist[0] = 0.5;
    in.regionlist[1] = 5.0;
    in.regionlist[2] = 7.0;            /* Regional attribute (for whole mesh). */
    in.regionlist[3] = 0.1;          /* Area constraint that will not be used. */
    return in;
}
triangulateio inDotCloud(Point2fArray& points, std::vector<Point2fArray>& holes)
{
    struct triangulateio in;

    //where boundery
    in.numberofpoints = points.size();
    for (auto x : holes)
        in.numberofpoints += x.size();
    in.pointlist = (REAL*)malloc(in.numberofpoints * 2 * sizeof(REAL));
    int i = 0;
    for (auto x : points)
    {
        in.pointlist[i++] = x.x;
        in.pointlist[i++] = x.y;
        //cout << x.x<<","<<x.y << endl;
    }
    for (auto y : holes)
        for (auto x : y)
        {
            in.pointlist[i++] = x.x;
            in.pointlist[i++] = x.y;
            //cout << x.x << "," << x.y << endl;
        }

    in.numberofpointattributes = 0;
    //in.pointattributelist = (REAL*)malloc(in.numberofpoints *
    //    in.numberofpointattributes *
    //    sizeof(REAL));
    //in.pointattributelist[0] = 0.0;
    //in.pointattributelist[1] = 1.0;
    //in.pointattributelist[2] = 11.0;
    //in.pointattributelist[3] = 10.0;
    in.pointmarkerlist = (int*)NULL;
    //in.pointmarkerlist[0] = 100;
    //in.pointmarkerlist[1] = 200;
    //in.pointmarkerlist[2] = 100;
    //in.pointmarkerlist[3] = 1000;

    //segEdge
    in.numberofsegments = in.numberofpoints;
    in.segmentlist = (int*)malloc(in.numberofpoints * 2 * sizeof(int));

    int begin = 0;
    int end = points.size();
    int edgeNum = 0;
    for (int i = 0; i < in.numberofsegments; i++)
    {
        if (i == end - 1)
        {
            in.segmentlist[2 * i] = i;
            in.segmentlist[2 * i + 1] = begin;
            begin = end;
            edgeNum < holes.size() ? end += holes[edgeNum++].size() : end;
        }
        else
        {
            in.segmentlist[2 * i] = i;
            in.segmentlist[2 * i + 1] = i + 1;
        }
    }
    in.segmentmarkerlist = (int*)malloc(in.numberofsegments * sizeof(int));
    //for (size_t i = 0; i < in.numberofsegments; i++)
    //{   
    //    in.segmentmarkerlist[i] = 1;
    //}

    in.numberofholes = holes.size();                                      /* In / copied out */
    in.holelist = (REAL*)malloc(in.numberofholes * 2 * sizeof(REAL));                        /* In / pointer to array copied out */
    int hole_itr = 0;
    //this maybe not inner Point
    //for (auto x : holes)
    //{
    //    in.holelist[j++] = (x[0].x + x[1].x + x[2].x) / 3;
    //    in.holelist[j++] = (x[0].y + x[1].y + x[2].y) / 3;
    //    //cout << y << endl;
    //}
    // 
    //find inner point
    //bool pointInPolygon=[]() {

    //    int   i, j = polySides - 1;
    //    bool  oddNodes = false;

    //    for (i = 0; i < polySides; i++) {
    //        if ((polyY[i] < y && polyY[j] >= y
    //            || polyY[j] < y && polyY[i] >= y)
    //            && (polyX[i] <= x || polyX[j] <= x)) {
    //            oddNodes ^= (polyX[i] + (y - polyY[i]) / (polyY[j] - polyY[i]) * (polyX[j] - polyX[i]) < x);
    //        }
    //        j = i;
    //    }
    //    return oddNodes;
    //}

    // maybe wrong  
    for (auto data : holes)
    {
        for (int i = 2; i < data.size(); i++)
        {
			auto check = [&]() {
				Eigen::Vector2f p1(data[i - 2].x,data[i-2].y);
				Eigen::Vector2f p2(data[i - 1].x,data[i-1].y);
				Eigen::Vector2f p3(data[i ].x, data[i].y);
				auto v1 = (p1 - p2).normalized();
				auto v2 = (p3 - p2).normalized();
				auto ang = acos(v1.dot(v2));
				return ang*180/ 3.1416;
			};
			if (check() > 175.)
				continue;
            double x = (data[i-2].x + data[i-1].x + data[i].x) / 3;
            double y = (data[i-2].y + data[i-1].y + data[i].y) / 3;

            bool  oddNodes = false;

            int end_j = data.size()-1;
            for(int j=0;j<data.size();j++)
            {
				if ((data[j].y < y && data[end_j].y >= y
					|| data[end_j].y < y && data[j].y >= y)
					&& (data[j].x <= x || data[end_j].x <= x)) {
					oddNodes ^= (data[j].x + (y - data[j].y) / (data[end_j].y - data[j].y) * (data[end_j].x - data[j].x) < x);
                 }
                end_j = j;
            }

            if (oddNodes)
            {
                in.holelist[hole_itr++] = x;
                in.holelist[hole_itr++] = y;
                break;
            }
        }
    }
	auto test_holelist = [&]() {
		if ((hole_itr + 1) / 2 != holes.size())
		{
			cout << "the num of hole is not" << endl;
		}
		ofstream log_holeList("log_holeList.obj");
		for (int i = 0; i < holes.size(); i++)
		{
			log_holeList << "hole:" << i <<"has point:"<<holes.size()<< endl;
			log_holeList << "v " << in.holelist[2 * i] << " " << in.holelist[2 * i + 1] << " "<<0<<endl;
			for (auto x : holes[i])
			{
				log_holeList << "v " << x.x << " " << x.y << " " << 0 << endl;
			}
		}
		log_holeList.close();
	};
	test_holelist();
    in.numberofregions = 1;
    in.regionlist = (REAL*)malloc(in.numberofregions * 4 * sizeof(REAL));
    in.regionlist[0] = 0.5;
    in.regionlist[1] = 5.0;
    in.regionlist[2] = 7.0;            /* Regional attribute (for whole mesh). */
    in.regionlist[3] = 0.1;          /* Area constraint that will not be used. */
    return in;
}
triangulateio inDotCloud(vector<double>& points)
{
    struct triangulateio in;
    points.push_back(0);
    points.push_back(0);

    points.push_back(30);
    points.push_back(0);

    points.push_back(30);
    points.push_back(30);

    points.push_back(20);
    points.push_back(30);

    points.push_back(20);
    points.push_back(20);

    points.push_back(10);
    points.push_back(20);

    points.push_back(10);
    points.push_back(30);

    points.push_back(0);
    points.push_back(30);
    //holepoints
    points.push_back(5);
    points.push_back(5);

    points.push_back(25);
    points.push_back(5);

    points.push_back(25);
    points.push_back(15);

    points.push_back(5);
    points.push_back(15);

    in.numberofpoints = points.size() / 2;
    in.pointlist = (REAL*)malloc(in.numberofpoints * 2 * sizeof(REAL));
    int i = 0;
    for (auto x : points)
    {
        in.pointlist[i++] = x;
        //cout << y << endl;
    }

    in.numberofsegments = 12;
    in.segmentlist = (int*)malloc(in.numberofpoints * 2 * sizeof(int));
    for (int i = 0; i < 8; i++)
    {
        //Peripheral Boundary
        if (i == 8 - 1)
        {
            in.segmentlist[2 * i] = i;
            in.segmentlist[2 * i + 1] = 0;
        }
        else
        {
            in.segmentlist[2 * i] = i;
            in.segmentlist[2 * i + 1] = i + 1;
        }
    }
    for (int i = 8; i < 12; i++)
    {
        if (i == in.numberofsegments - 1)
        {
            in.segmentlist[2 * i] = i;
            in.segmentlist[2 * i + 1] = 8;
        }
        else
        {
            in.segmentlist[2 * i] = i;
            in.segmentlist[2 * i + 1] = i + 1;
        }
    }
    in.segmentmarkerlist = (int*)malloc(in.numberofsegments * sizeof(int));
    //for (size_t i = 0; i < in.numberofsegments; i++)
    //{
    //    in.segmentmarkerlist[i] = 1;
    //}

    in.numberofpointattributes = 0;
    //in.pointattributelist = (REAL*)malloc(in.numberofpoints *
    //    in.numberofpointattributes *
    //    sizeof(REAL));
    //in.pointattributelist[0] = 0.0;
    //in.pointattributelist[1] = 1.0;
    //in.pointattributelist[2] = 11.0;
    //in.pointattributelist[3] = 10.0;
    in.pointmarkerlist = (int*)NULL;
    //in.pointmarkerlist[0] = 100;
    //in.pointmarkerlist[1] = 200;
    //in.pointmarkerlist[2] = 100;
    //in.pointmarkerlist[3] = 1000;

    in.numberofholes = 1;                                      /* In / copied out */
    in.holelist = (REAL*)malloc(in.numberofholes * 2 * sizeof(REAL));                        /* In / pointer to array copied out */
    int j = 0;
    //for (auto x : hole)
    {
        in.holelist[j++] = 10;
        in.holelist[j++] = 10;
    }

    //in.numberofregions = 1;
    //in.regionlist = (REAL*)malloc(in.numberofregions * 4 * sizeof(REAL));
    //in.regionlist[0] = 0.5;
    //in.regionlist[1] = 5.0;
    //in.regionlist[2] = 7.0;            /* Regional attribute (for whole mesh). */
    //in.regionlist[3] = 0.1;          /* Area constraint that will not be used. */
    return in;
}
triangulateio inDotCloud(Point3fArray& points, std::vector<Point3fArray>& holes)
{
	struct triangulateio in;

	//where boundery
	in.numberofpoints = points.size();
	for (auto x : holes)
		in.numberofpoints += x.size();
	in.pointlist = (REAL*)malloc(in.numberofpoints * 2 * sizeof(REAL));
	int i = 0;
	for (auto x : points)
	{
		in.pointlist[i++] = x.x;
		in.pointlist[i++] = x.y;
		//cout << x.x<<","<<x.y << endl;
	}
	for (auto y : holes)
		for (auto x : y)
		{
			in.pointlist[i++] = x.x;
			in.pointlist[i++] = x.y;
			//cout << x.x << "," << x.y << endl;
		}

	in.numberofpointattributes = 0;
	//in.pointattributelist = (REAL*)malloc(in.numberofpoints *
	//    in.numberofpointattributes *
	//    sizeof(REAL));
	//in.pointattributelist[0] = 0.0;
	//in.pointattributelist[1] = 1.0;
	//in.pointattributelist[2] = 11.0;
	//in.pointattributelist[3] = 10.0;
	in.pointmarkerlist = (int*)NULL;
	//in.pointmarkerlist[0] = 100;
	//in.pointmarkerlist[1] = 200;
	//in.pointmarkerlist[2] = 100;
	//in.pointmarkerlist[3] = 1000;

	//segEdge
	in.numberofsegments = in.numberofpoints;
	in.segmentlist = (int*)malloc(in.numberofpoints * 2 * sizeof(int));

	int begin = 0;
	int end = points.size();
	int edgeNum = 0;
	for (int i = 0; i < in.numberofsegments; i++)
	{
		if (i == end - 1)
		{
			in.segmentlist[2 * i] = i;
			in.segmentlist[2 * i + 1] = begin;
			begin = end;
			edgeNum < holes.size() ? end += holes[edgeNum++].size() : end;
		}
		else
		{
			in.segmentlist[2 * i] = i;
			in.segmentlist[2 * i + 1] = i + 1;
		}
	}
	in.segmentmarkerlist = (int*)malloc(in.numberofsegments * sizeof(int));
	//for (size_t i = 0; i < in.numberofsegments; i++)
	//{   
	//    in.segmentmarkerlist[i] = 1;
	//}

	in.numberofholes = holes.size();                                      /* In / copied out */
	in.holelist = (REAL*)malloc(in.numberofholes * 2 * sizeof(REAL));                        /* In / pointer to array copied out */
	int hole_itr = 0;
	//this maybe not inner Point
	//for (auto x : holes)
	//{
	//    in.holelist[j++] = (x[0].x + x[1].x + x[2].x) / 3;
	//    in.holelist[j++] = (x[0].y + x[1].y + x[2].y) / 3;
	//    //cout << y << endl;
	//}
	// 
	//find inner point
	//bool pointInPolygon=[]() {

	//    int   i, j = polySides - 1;
	//    bool  oddNodes = false;

	//    for (i = 0; i < polySides; i++) {
	//        if ((polyY[i] < y && polyY[j] >= y
	//            || polyY[j] < y && polyY[i] >= y)
	//            && (polyX[i] <= x || polyX[j] <= x)) {
	//            oddNodes ^= (polyX[i] + (y - polyY[i]) / (polyY[j] - polyY[i]) * (polyX[j] - polyX[i]) < x);
	//        }
	//        j = i;
	//    }
	//    return oddNodes;
	//}

	// maybe wrong  
	for (auto data : holes)
	{
		for (int i = 2; i < data.size(); i++)
		{
			auto check = [&]() {
				Eigen::Vector2f p1(data[i - 2].x, data[i - 2].y);
				Eigen::Vector2f p2(data[i - 1].x, data[i - 1].y);
				Eigen::Vector2f p3(data[i].x, data[i].y);
				auto v1 = (p1 - p2).normalized();
				auto v2 = (p3 - p2).normalized();
				auto ang = acos(v1.dot(v2));
				return ang * 180 / 3.1416;
			};
			if (check() > 175.)
				continue;
			double x = (data[i - 2].x + data[i - 1].x + data[i].x) / 3;
			double y = (data[i - 2].y + data[i - 1].y + data[i].y) / 3;

			bool  oddNodes = false;

			int end_j = data.size() - 1;
			for (int j = 0; j < data.size(); j++)
			{
				if ((data[j].y < y && data[end_j].y >= y
					|| data[end_j].y < y && data[j].y >= y)
					&& (data[j].x <= x || data[end_j].x <= x)) {
					oddNodes ^= (data[j].x + (y - data[j].y) / (data[end_j].y - data[j].y) * (data[end_j].x - data[j].x) < x);
				}
				end_j = j;
			}

			if (oddNodes)
			{
				in.holelist[hole_itr++] = x;
				in.holelist[hole_itr++] = y;
				break;
			}
		}
	}
	auto test_holelist = [&]() {
		if ((hole_itr + 1) / 2 != holes.size())
		{
			cout << "the num of hole is not" << endl;
		}
		ofstream log_holeList("log_holeList.obj");
		for (int i = 0; i < holes.size(); i++)
		{
			log_holeList << "hole:" << i << "has point:" << holes.size() << endl;
			log_holeList << "v " << in.holelist[2 * i] << " " << in.holelist[2 * i + 1] << " " << 0 << endl;
			for (auto x : holes[i])
			{
				log_holeList << "v " << x.x << " " << x.y << " " << 0 << endl;
			}
		}
		log_holeList.close();
	};
	test_holelist();
	in.numberofregions = 1;
	in.regionlist = (REAL*)malloc(in.numberofregions * 4 * sizeof(REAL));
	in.regionlist[0] = 0.5;
	in.regionlist[1] = 5.0;
	in.regionlist[2] = 7.0;            /* Regional attribute (for whole mesh). */
	in.regionlist[3] = 0.1;          /* Area constraint that will not be used. */
	return in;
}
vector<vector<double>> box(vector<vector<double>>& point)
{
    float minX = point[0][0], minY = point[0][1];
    float maxX = point[0][0], maxY = point[0][1];

    for (auto x : point)
    {
        minX = min((float)x[0], minX);
        minY = min((float)x[1], minY);
        maxX = max((float)x[0], maxX);
        maxY = max((float)x[1], maxY);
    }
    vector<vector<double>> result = { {minX,minY},{maxX,maxY} };
    return result;
}

vector<int> hashtoMap(vector<double>& point, vector<vector<double>>& MinMax)
{
    float minX = MinMax[0][0], minY = MinMax[0][1];
    float maxX = MinMax[1][0], maxY = MinMax[1][1];

    //hash
    int diagonal = 60;
    int offset0_5 = floor(maxX - minX) / diagonal / 20;
    int offset = offset0_5 * 2;
    int projectX = floor(maxX - minX) / diagonal + offset + 1, projectY = floor(maxY - minY) / diagonal + offset + 1;
    //vector<int> map(projectX * projectY, 0);
    //for (auto x : *inCloud)
    //{
    //    map[(floor(x.x - minX) / diagonal + offset0_5-1) * projectY + floor(x.y - minY) / diagonal + offset0_5-1]++;
    //}
    vector<int> result = { int((point[0] - minX) / diagonal) + offset0_5,int((point[1] - minY) / diagonal) + offset0_5 };
    //cout << result[0] << " " << result[1] << endl;
    //assert(result[1] >-1);
    return result;
}
bool isTriangleInPointCloud(vector<vector<int>>& map, vector<vector<double>>& point, vector<vector<double>>& MinMax)
{
    vector<vector<double>> MinMaxXY = move(box(point));
    vector<int> minIJ = move(hashtoMap(MinMaxXY[0], MinMax));
    vector<int> maxIJ = move(hashtoMap(MinMaxXY[1], MinMax));
    //random method is fast
    auto minmaxX = maxIJ[0] - minIJ[0];
    auto minmaxY = maxIJ[1] - minIJ[1];
    if (minmaxX <= 0 || minmaxY <= 0)
        return false;
    //Strict strategy
    auto hit_ratio = 0;
    //for (int i = minIJ[0]; i < minIJ[1]; i++)
    //    for (int j = maxIJ[0]; j < maxIJ[1]; j++)
    //    {
    //        if (map[i][j] > 0)
    //            hit_ratio++;//Multiple hits in a row Is considered success
    //        else
    //            hit_ratio--;
    //    }

    //for (int i = 0; i < 10; i++)
    //    if (map[rand() % minmaxX + minIJ[0]][rand() % minmaxY + minIJ[1]] > 0)
    //        hit_ratio++;//Multiple hits in a row Is considered success
    //    else
    //        /*hit_ratio--*/;
    //if (hit_ratio >4)
    //    return true;
    //else
    //    return false;

    vector<double> centre = { (point[0][0] + point[1][0] + point[2][0]) / 3,
        (point[0][1] + point[1][1] + point[2][1]) / 3 };
    auto centreMap = move(hashtoMap(centre, MinMax));

    if (map[centreMap[0]][centreMap[1] + 1] > 0 ||
        map[centreMap[0]][centreMap[1] - 1] > 0 ||
        map[centreMap[0] + 1][centreMap[1]] > 0 ||
        map[centreMap[0] - 1][centreMap[1]] > 0)
        return true;
    else
    {
        //for (int i = 0; i < 10; i++)
        //    if (map[rand() % minmaxX + minIJ[0]][rand() % minmaxY + minIJ[1]] > 0)
        //        hit_ratio++;//Multiple hits in a row Is considered success
        //    else
        //        /*hit_ratio--*/;
        //if (hit_ratio >5)
        //    return true;
        //else
        return false;
    }

    //calculation method is accuracy
    if ("strategy" != "strategy")
    {
        //int accuracy
        //u = ((v1•v1)(v2•v0)-(v1•v0)(v2•v1)) / ((v0•v0)(v1•v1)-(v0•v1)(v1•v0))
        //v = ((v0•v0)(v2•v1)-(v0•v1)(v2•v0)) / ((v0•v0)(v1•v1)-(v0•v1)(v1•v0))
        vector<int> triangleA = move(hashtoMap(point[0], MinMax));
        vector<int> triangleB = move(hashtoMap(point[1], MinMax));
        vector<int> triangleC = move(hashtoMap(point[2], MinMax));

        auto areaD = [&]() {
            return cro(sub(point[1], point[0]), sub(point[2], point[0])) * 0.5;
        };
        auto v0 = sub(triangleB, triangleA);
        auto v1 = sub(triangleC, triangleA);

        auto area = /*0.5 * */cro(v0, v1);
        cout << "area=" << area << " areaD = "/*<< cro(sub(point[1], point[0]), sub(point[2], point[0]))*//* * 0.5*/;
        int triangleArea = 0;
        int effectiveArea = 0;
        if (area == 0)
            return true;
        for (int i = minIJ[0]; i < minIJ[1]; i++)
            for (int j = maxIJ[0]; j < maxIJ[1]; j++)
            {
                //no accuracy 
                if (map[i][j] < 1)
                    continue;
                auto v2 = sub({ i,j }, triangleA);
                auto u = (double)(dot(v1, v1) * dot(v2, v0) - (dot(v1, v0) * dot(v2, v1))) /
                    (dot(v0, v0) * dot(v1, v1) - dot(v0, v1) * dot(v1, v0));
                auto v = (double)(dot(v0, v0) * dot(v2, v1) - (dot(v0, v1) * dot(v2, v0))) /
                    (dot(v0, v0) * dot(v1, v1) - dot(v0, v1) * dot(v1, v0));
                if (u > 0 && v > 0 && u + v < 1)
                    triangleArea++;
            }
        cout << " triangelArea=" << triangleArea << endl;
        if (triangleArea < 0)
            return false;
    }

    return true;
}

//void concreteMesherDelauney::AutoTestPts(vector<wstring>& fileName)
//{
//    WIN32_FIND_DATAW filedata;
//    //std::wstring scenePath = L"D:/data/outmesh_30/";
//    //std::wstring scenePath = L"D:/data/outmesh/outmesh_66/outmesh_66_test/";
//    //std::wstring scenePath = L"D:/data/outmesh/outmesh_66/"; 
//    std::wstring scenePath = L"D:\\data\\outmesh\\outmesh_1\\"; 
//    std::wstring temp = scenePath + L"*.ply";
//    //HANDLE hFile = ::FindFirstFileW(L(scenePath)+L"E:\\2.Data\\ptsdata \\*.pts", &filedata);
//    HANDLE hFile = ::FindFirstFileW((temp).c_str(), &filedata);
//
//    if (hFile == INVALID_HANDLE_VALUE)
//        return;
//    do
//    {
//        //MeasurementResult* result = new MeasurementResult();
//        const std::wstring cadPath = L"";
//        const std::wstring meshPath = L"outputmesh";
//        const std::wstring jsonPath = L"output.json";
//        const std::wstring compass_path = L"dire.txt";
//
//        DWORD dwStart = GetTickCount();
//        //bool b =  StartMeasurement(MEASUREMENT_CUSTOMER_UNRE, scenePath, cadPath, result);
//        //bool b = StartMeasurementAndSaveMesh(MEASUREMENT_CUSTOMER_UNRE, scenePath, cadPath, result, compass_path, meshPath);
//        fileName.push_back(scenePath + filedata.cFileName);
//        std::cout << "StartMeasurement time = " << GetTickCount() - dwStart << std::endl;
//
//        //delete result;
//
//    } while (::FindNextFileW(hFile, &filedata));
//    FindClose(hFile);
//}

bool concreteMesherDelauney::buildMesh(vector<cv::Point3f> inPointCloud, string plane_file_name)
{
    struct triangulateio in, mid, out, vorout;

    /* Define input points. */
#if false
    in.numberofpoints = 4;
    in.numberofpointattributes = 1;
    in.pointlist = (REAL*)malloc(in.numberofpoints * 2 * sizeof(REAL));
    in.pointlist[0] = 0.0;
    in.pointlist[1] = 0.0;
    in.pointlist[2] = 1.0;
    in.pointlist[3] = 0.0;
    in.pointlist[4] = 1.0;
    in.pointlist[5] = 10.0;
    in.pointlist[6] = 0.0;
    in.pointlist[7] = 10.0;

    in.pointattributelist = (REAL*)malloc(in.numberofpoints *
        in.numberofpointattributes *
        sizeof(REAL));
    in.pointattributelist[0] = 0.0;
    in.pointattributelist[1] = 1.0;
    in.pointattributelist[2] = 11.0;
    in.pointattributelist[3] = 10.0;

    in.pointmarkerlist = (int*)malloc(in.numberofpoints * sizeof(int));
    in.pointmarkerlist = (int*)NULL;
    // 
    in.pointmarkerlist[0] = 0;
    in.pointmarkerlist[1] = 0;
    in.pointmarkerlist[2] = 0;
    in.pointmarkerlist[3] = 0;

    in.numberofsegments = 0;
    in.numberofholes = 0;
    in.numberofregions = 1;
    in.regionlist = (REAL*)malloc(in.numberofregions * 4 * sizeof(REAL));
    in.regionlist[0] = 0.5;
    in.regionlist[1] = 5.0;
    in.regionlist[2] = 7.0;            /* Regional attribute (for whole mesh). */
    in.regionlist[3] = 0.1;          /* Area constraint that will not be used. */
#endif

    vector<vector<double>> points;
    /*GetDotCloud(points,"D:\\data\\EdgePointOnly.pts");
    projectionXY(points, points);*/

    GetDotCloud(points, "D:/code/PlaneLine20210616/data/output/detc_edge/edge_xyz/2Dedge_xyz_0.txt");

    vector<vector<vector<double>>> hole(1);
    GetDotCloud(hole[0], "D:/code/PlaneLine20210616/data/output/detc_edge/edge_xyz/2Dedge_xyz_1.txt");

    vector<double> pointCloud;
    //in = inDotCloud(points);
    //in = inDotCloud(points,hole);
    in = inDotCloud(points, hole);
    //in = inDotCloud(pointCloud);

    vector<vector<int>> map;
    GetMap(map);

    for (size_t i = 0; i < in.numberofpoints * 2; i = i + 2)
    {
        cout << in.pointlist[i] << "," << in.pointlist[i + 1] << endl;
    }
    //printf("Input point set:\n\n");
    //report(&in, 1, 0, 0, 0, 0, 0);

    /* Make necessary initializations so that Triangle can return a */
    /*   triangulation in `mid' and a voronoi diagram in `vorout'.  */

    mid.pointlist = (REAL*)NULL;            /* Not needed if -N switch used. */
    /* Not needed if -N switch used or number of point attributes is zero: */
    mid.pointattributelist = (REAL*)NULL;
    mid.pointmarkerlist = (int*)NULL; /* Not needed if -N or -B switch used. */
    mid.trianglelist = (int*)NULL;          /* Not needed if -E switch used. */
    /* Not needed if -E switch used or number of triangle attributes is zero: */
    mid.triangleattributelist = (REAL*)NULL;
    mid.neighborlist = (int*)NULL;         /* Needed only if -n switch used. */
    /* Needed only if segments are output (-p or -c) and -P not used: */
    mid.segmentlist = (int*)NULL;
    /* Needed only if segments are output (-p or -c) and -P and -B not used: */
    mid.segmentmarkerlist = (int*)NULL;
    mid.edgelist = (int*)NULL;             /* Needed only if -e switch used. */
    mid.edgemarkerlist = (int*)NULL;   /* Needed if -e used and -B not used. */

    vorout.pointlist = (REAL*)NULL;        /* Needed only if -v switch used. */
    /* Needed only if -v switch used and number of attributes is not zero: */
    vorout.pointattributelist = (REAL*)NULL;
    vorout.edgelist = (int*)NULL;          /* Needed only if -v switch used. */
    vorout.normlist = (REAL*)NULL;         /* Needed only if -v switch used. */

    /* Triangulate the points.  Switches are chosen to read and write a  */
    /*   PSLG (p), preserve the convex hull (c), number everything from  */
    /*   zero (z), assign a regional attribute to each element (A), and  */
    /*   produce an edge list (e), a Voronoi diagram (v), and a triangle */
    /*   neighbor list (n).                                              */

    triangulate("pzAevn", &in, &mid, &vorout);
    //triangulate("pczAevn", &in, &mid, &vorout);

    /*printf("Initial triangulation:\n\n");
    report(&mid, 1, 1, 1, 1, 1, 0);
    printf("Initial Voronoi diagram:\n\n");
    report(&vorout, 0, 0, 0, 0, 1, 1);*/

    /* Attach area constraints to the triangles in preparation for */
    /*   refining the triangulation.                               */

    /* Needed only if -r and -a switches used: */
    mid.trianglearealist = (REAL*)malloc(mid.numberoftriangles * sizeof(REAL));
    mid.trianglearealist[0] = 3.0;
    mid.trianglearealist[1] = 1.0;

    /* Make necessary initializations so that Triangle can return a */
    /*   triangulation in `out'.                                    */

    out.pointlist = (REAL*)NULL;            /* Not needed if -N switch used. */
    /* Not needed if -N switch used or number of attributes is zero: */
    out.pointattributelist = (REAL*)NULL;
    out.trianglelist = (int*)NULL;          /* Not needed if -E switch used. */
    /* Not needed if -E switch used or number of triangle attributes is zero: */
    out.triangleattributelist = (REAL*)NULL;

    /* Refine the triangulation according to the attached */
    /*   triangle area constraints.                       */

    triangulate("prazBP", &mid, &out, (struct triangulateio*)NULL);
    /// 
    //for (size_t i = 0; i < out.numberofpoints*2; i=i+2)
    //{
    //    cout<< out.pointlist[i]<<" "<<out.pointlist[i+1] /*<< out.pointlist[(int)out.trianglearealist[i]]*/ << endl;
    //}
    //for (size_t i = 0; i < out.numberoftriangles* out.numberofcorners; i = i+3)
    //{
    //    cout << out.pointlist[out.trianglelist[i]] << " " << out.pointlist[out.trianglelist[i] + 1] << endl<<
    //        out.pointlist[out.trianglelist[i+1]]<< " " << out.pointlist[out.trianglelist[i+1] + 1] << endl<<
    //        out.pointlist[out.trianglelist[i + 2]] <<" "<< out.pointlist[out.trianglelist[i + 2]]<< endl;
    //}

    //ofstream outtriangles("D:\\data\\trianglelist.obj");
    //for (size_t i = 0; i < out.numberoftriangles * out.numberofcorners; i+=3)
    //{
    //    outtriangles << "v " <<out.pointlist[out.trianglelist[i]] << " " << out.pointlist[out.trianglelist[i] + 1] << " 0" << endl <<
    //        "v " << out.pointlist[out.trianglelist[i + 1]] << " " << out.pointlist[out.trianglelist[i + 1] + 1] << " 0" << endl <<
    //        "v " << out.pointlist[out.trianglelist[i + 2]] << " " << out.pointlist[out.trianglelist[i + 2]+1] << " 0" << endl;
    //}
    //for (size_t i = 0; i < out.numberoftriangles * out.numberofcorners; i += 3)
    //{
    //    outtriangles << "f " << i << " " << i+1 << " " << i+2 << endl;
    //}
    //outtriangles.close();

    //obj

    ofstream outtriangles("D:\\data\\triangle.obj");
    vector<vector<double>> outPoints;
    for (size_t i = 0; i < out.numberofpoints * 2; i = i + 2)
    {
        outtriangles << "v " << out.pointlist[i] << " " << out.pointlist[i + 1] << " 0" << endl;
        outPoints.push_back({ out.pointlist[i],out.pointlist[i + 1] });
    }
    vector<vector<double>> MinMax_test = box(outPoints);
    vector<vector<double>> MinMax = { { -4723.4,-3971.8 }, { 4436.2,4223.2 } };//box(points);
    for (size_t i = 0; i < out.numberoftriangles * out.numberofcorners; i += 3)
    {
        vector<vector<double>> point = { {out.pointlist[out.trianglelist[i] * 2], out.pointlist[out.trianglelist[i] * 2 + 1]},
            {out.pointlist[out.trianglelist[i + 1] * 2] , out.pointlist[out.trianglelist[i + 1] * 2 + 1] },
            {out.pointlist[out.trianglelist[i + 2] * 2] , out.pointlist[out.trianglelist[i + 2] * 2 + 1]} };
        //   cout << "v " <<out.pointlist[out.trianglelist[i]*2] << " " << out.pointlist[out.trianglelist[i]*2 + 1] << " 0" << endl <<
        //"v " << out.pointlist[out.trianglelist[i + 1]*2] << " " << out.pointlist[out.trianglelist[i + 1]*2 + 1] << " 0" << endl <<
        //"v " << out.pointlist[out.trianglelist[i + 2]*2] << " " << out.pointlist[out.trianglelist[i + 2]*2+1] << " 0" << endl;

        //   cout << "v " << out.trianglelist[i] << " " << endl <<
        //       "v " << out.trianglelist[i + 1] << " " << endl <<
        //       "v " << out.trianglelist[i + 2] << " " << endl;
        //if (!isTriangleInPointCloud(map,point,MinMax))
        //    continue;
        outtriangles << "f " << out.trianglelist[i] + 1 << " " << out.trianglelist[i + 1] + 1 << " " << out.trianglelist[i + 2] + 1 << endl;
    }
    outtriangles.close();

    /// 
   // printf("Refined triangulation:\n\n");
    //report(&out, 0, 1, 0, 0, 0, 0);

    /* Free all allocated arrays, including those allocated by Triangle. */

    free(in.pointlist);
    //free(in.pointattributelist);
    free(in.pointmarkerlist);
    free(in.regionlist);
    free(mid.pointlist);
    //free(mid.pointattributelist);
    free(mid.pointmarkerlist);
    free(mid.trianglelist);
    free(mid.triangleattributelist);
    free(mid.trianglearealist);
    free(mid.neighborlist);
    free(mid.segmentlist);
    free(mid.segmentmarkerlist);
    free(mid.edgelist);
    free(mid.edgemarkerlist);
    free(vorout.pointlist);
    free(vorout.pointattributelist);
    free(vorout.edgelist);
    free(vorout.normlist);
    free(out.pointlist);
    free(out.pointattributelist);
    free(out.trianglelist);
    free(out.triangleattributelist);
    return true;
}

bool buildMesh_(std::vector<ModuleStruct::Point2fArray>& edgePointsList,
	std::vector<std::vector<ModuleStruct::Point2fArray>>& holePointsList,
	Matrix4d& rmMatrix, string plane_file_name)
{
	ofstream outtriangles(plane_file_name);
	size_t offset = 1;
	vector < vector<unsigned long long>> faceList;
	for (int i =0;i<edgePointsList.size();i++)
	{
		struct triangulateio in, mid, out, vorout;
		vector<double> pointCloud;

		in = inDotCloud(edgePointsList[i], holePointsList[i]);


		//for (size_t i = 0; i < in.numberofpoints * 2; i = i + 2)
		//{
		//    cout << in.pointlist[i] << "," << in.pointlist[i + 1] << endl;
		//}

		/* Make necessary initializations so that Triangle can return a */
		/*   triangulation in `mid' and a voronoi diagram in `vorout'.  */

		mid.pointlist = (REAL*)NULL;            /* Not needed if -N switch used. */
		/* Not needed if -N switch used or number of point attributes is zero: */
		mid.pointattributelist = (REAL*)NULL;
		mid.pointmarkerlist = (int*)NULL; /* Not needed if -N or -B switch used. */
		mid.trianglelist = (int*)NULL;          /* Not needed if -E switch used. */
		/* Not needed if -E switch used or number of triangle attributes is zero: */
		mid.triangleattributelist = (REAL*)NULL;
		mid.neighborlist = (int*)NULL;         /* Needed only if -n switch used. */
		/* Needed only if segments are output (-p or -c) and -P not used: */
		mid.segmentlist = (int*)NULL;
		/* Needed only if segments are output (-p or -c) and -P and -B not used: */
		mid.segmentmarkerlist = (int*)NULL;
		mid.edgelist = (int*)NULL;             /* Needed only if -e switch used. */
		mid.edgemarkerlist = (int*)NULL;   /* Needed if -e used and -B not used. */

		vorout.pointlist = (REAL*)NULL;        /* Needed only if -v switch used. */
		/* Needed only if -v switch used and number of attributes is not zero: */
		vorout.pointattributelist = (REAL*)NULL;
		vorout.edgelist = (int*)NULL;          /* Needed only if -v switch used. */
		vorout.normlist = (REAL*)NULL;         /* Needed only if -v switch used. */

		triangulate("pzAevnQ", &in, &mid, &vorout);


		out.pointlist = (REAL*)NULL;            /* Not needed if -N switch used. */
		/* Not needed if -N switch used or number of attributes is zero: */
		out.pointattributelist = (REAL*)NULL;
		out.trianglelist = (int*)NULL;          /* Not needed if -E switch used. */
		/* Not needed if -E switch used or number of triangle attributes is zero: */
		out.triangleattributelist = (REAL*)NULL;

		/* Refine the triangulation according to the attached */
		/*   triangle area constraints.                       */

		//triangulate("prazBP", &mid, &out, (struct triangulateio*)NULL);
		triangulate("pzBPQ", &mid, &out, (struct triangulateio*)NULL);

		auto tempOffset = 0;
		for (size_t i = 0; i < out.numberofpoints * 2; i = i + 2)
		{
			Vector4d point(out.pointlist[i], out.pointlist[i + 1], 0, 1);
			Vector4d point3d = rmMatrix * point;
			outtriangles << "v " << point3d[0] << " " << point3d[1] << " " << point3d[2] << endl;
			//outtriangles << "v " << out.pointlist[i] << " " << out.pointlist[i + 1] << " 0" << endl;

			tempOffset++;
		}
											
		for (size_t i = 0; i < out.numberoftriangles * out.numberofcorners; i += 3)
		{
			//outtriangles << "f " << out.trianglelist[i] + offset << " " << out.trianglelist[i + 1] + offset << " " << out.trianglelist[i + 2] + offset << endl;
			faceList.push_back({ out.trianglelist[i] + offset ,out.trianglelist[i + 1] + offset,out.trianglelist[i + 2] + offset });
		}
		offset += tempOffset;//outtriangles.();		 //outtriangles.end - outtriangles.beg;
		/// 
		//printf("Refined triangulation:\n\n");
		//report(&out, 0, 1, 0, 0, 0, 0);

		/* Free all allocated arrays, including those allocated by Triangle. */
		free(in.segmentlist);
		free(in.holelist);

		free(in.pointlist);
		//free(in.pointattributelist);
		free(in.pointmarkerlist); 
		free(in.regionlist);
		free(mid.pointlist);
		//free(mid.pointattributelist);
		free(mid.pointmarkerlist);
		free(mid.trianglelist);
		free(mid.triangleattributelist);
		//free(mid.trianglearealist);
		free(mid.neighborlist);
		free(mid.segmentlist);
		free(mid.segmentmarkerlist);
		free(mid.edgelist);
		free(mid.edgemarkerlist);
		free(vorout.pointlist);
		free(vorout.pointattributelist);
		free(vorout.edgelist);
		free(vorout.normlist);
		free(out.pointlist);
		free(out.pointattributelist);
		free(out.trianglelist);
		free(out.triangleattributelist);
	}
	for (auto x : faceList)
	{
		outtriangles << "f " << x[0] << " " << x[1] << " " << x[2] << endl;
	}

	outtriangles.close();
	return true;
}

bool buildMesh_(ModuleStruct::Point2fArray& edgePoints,
                                        std::vector<ModuleStruct::Point2fArray>& holePoints, 
                                        Matrix4d& rmMatrix, string plane_file_name)
{
    struct triangulateio in, mid, out, vorout;
    vector<double> pointCloud;

    in = inDotCloud(edgePoints, holePoints);


    //for (size_t i = 0; i < in.numberofpoints * 2; i = i + 2)
    //{
    //    cout << in.pointlist[i] << "," << in.pointlist[i + 1] << endl;
    //}

    /* Make necessary initializations so that Triangle can return a */
    /*   triangulation in `mid' and a voronoi diagram in `vorout'.  */

    mid.pointlist = (REAL*)NULL;            /* Not needed if -N switch used. */
    /* Not needed if -N switch used or number of point attributes is zero: */
    mid.pointattributelist = (REAL*)NULL;
    mid.pointmarkerlist = (int*)NULL; /* Not needed if -N or -B switch used. */
    mid.trianglelist = (int*)NULL;          /* Not needed if -E switch used. */
    /* Not needed if -E switch used or number of triangle attributes is zero: */
    mid.triangleattributelist = (REAL*)NULL;
    mid.neighborlist = (int*)NULL;         /* Needed only if -n switch used. */
    /* Needed only if segments are output (-p or -c) and -P not used: */
    mid.segmentlist = (int*)NULL;
    /* Needed only if segments are output (-p or -c) and -P and -B not used: */
    mid.segmentmarkerlist = (int*)NULL;
    mid.edgelist = (int*)NULL;             /* Needed only if -e switch used. */
    mid.edgemarkerlist = (int*)NULL;   /* Needed if -e used and -B not used. */

    vorout.pointlist = (REAL*)NULL;        /* Needed only if -v switch used. */
    /* Needed only if -v switch used and number of attributes is not zero: */
    vorout.pointattributelist = (REAL*)NULL;
    vorout.edgelist = (int*)NULL;          /* Needed only if -v switch used. */
    vorout.normlist = (REAL*)NULL;         /* Needed only if -v switch used. */

    /* Triangulate the points.  Switches are chosen to read and write a  */
    /*   PSLG (p), preserve the convex hull (c), number everything from  */
    /*   zero (z), assign a regional attribute to each element (A), and  */
    /*   produce an edge list (e), a Voronoi diagram (v), and a triangle */
    /*   neighbor list (n).                                              */

    //triangulate("pzAevn", &in, &mid, &vorout);
    //triangulate("pczAevn", &in, &mid, &vorout);
	triangulate("pzAevnQ", &in, &mid, &vorout);

    /*printf("Initial triangulation:\n\n");
    report(&mid, 1, 1, 1, 1, 1, 0);
    printf("Initial Voronoi diagram:\n\n");
    report(&vorout, 0, 0, 0, 0, 1, 1);*/

    /* Attach area constraints to the triangles in preparation for */
    /*   refining the triangulation.                               */

    /* Needed only if -r and -a switches used: */
    //mid.trianglearealist = (REAL*)malloc(mid.numberoftriangles * sizeof(REAL));
    //mid.trianglearealist[0] = 3.0;
    //mid.trianglearealist[1] = 1.0;

    /* Make necessary initializations so that Triangle can return a */
    /*   triangulation in `out'.                                    */

    out.pointlist = (REAL*)NULL;            /* Not needed if -N switch used. */
    /* Not needed if -N switch used or number of attributes is zero: */
    out.pointattributelist = (REAL*)NULL;
    out.trianglelist = (int*)NULL;          /* Not needed if -E switch used. */
    /* Not needed if -E switch used or number of triangle attributes is zero: */
    out.triangleattributelist = (REAL*)NULL;

    /* Refine the triangulation according to the attached */
    /*   triangle area constraints.                       */

    //triangulate("prazBP", &mid, &out, (struct triangulateio*)NULL);
    triangulate("pzBPQ", &mid, &out, (struct triangulateio*)NULL);
    //triangulate("pzBP", &mid, &out, (struct triangulateio*)NULL);
    /// 
    //for (size_t i = 0; i < out.numberofpoints*2; i=i+2)
    //{
    //    cout<< out.pointlist[i]<<" "<<out.pointlist[i+1] /*<< out.pointlist[(int)out.trianglearealist[i]]*/ << endl;
    //}
    //for (size_t i = 0; i < out.numberoftriangles* out.numberofcorners; i = i+3)
    //{
    //    cout << out.pointlist[out.trianglelist[i]] << " " << out.pointlist[out.trianglelist[i] + 1] << endl<<
    //        out.pointlist[out.trianglelist[i+1]]<< " " << out.pointlist[out.trianglelist[i+1] + 1] << endl<<
    //        out.pointlist[out.trianglelist[i + 2]] <<" "<< out.pointlist[out.trianglelist[i + 2]]<< endl;
    //}

    //ofstream outtriangles("D:\\data\\trianglelist.obj");
    //for (size_t i = 0; i < out.numberoftriangles * out.numberofcorners; i+=3)
    //{
    //    outtriangles << "v " <<out.pointlist[out.trianglelist[i]] << " " << out.pointlist[out.trianglelist[i] + 1] << " 0" << endl <<
    //        "v " << out.pointlist[out.trianglelist[i + 1]] << " " << out.pointlist[out.trianglelist[i + 1] + 1] << " 0" << endl <<
    //        "v " << out.pointlist[out.trianglelist[i + 2]] << " " << out.pointlist[out.trianglelist[i + 2]+1] << " 0" << endl;
    //}
    //for (size_t i = 0; i < out.numberoftriangles * out.numberofcorners; i += 3)
    //{
    //    outtriangles << "f " << i << " " << i+1 << " " << i+2 << endl;
    //}
    //outtriangles.close();

    //obj
     
    //ofstream outtriangles("D:\\data\\triangle.obj");
    ofstream outtriangles(plane_file_name);
    vector<vector<double>> outPoints;
    for (size_t i = 0; i < out.numberofpoints * 2; i = i + 2)
    {
        //Vector4d point(out.pointlist[i], out.pointlist[i + 1], 0, 1);
        //Vector4d point3d = rmMatrix * point;
        //outtriangles << "v " << point3d[0] << " " << point3d[1] <<" "<< point3d[2] << endl;
		outtriangles << "v " << out.pointlist[i] << " " << out.pointlist[i + 1] << " 0" << endl;

        outPoints.push_back({ out.pointlist[i],out.pointlist[i + 1] }); 
    }

    for (size_t i = 0; i < out.numberoftriangles * out.numberofcorners; i += 3)
    {
        //vector<vector<double>> point = { {out.pointlist[out.trianglelist[i] * 2], out.pointlist[out.trianglelist[i] * 2 + 1]},
        //    {out.pointlist[out.trianglelist[i + 1] * 2] , out.pointlist[out.trianglelist[i + 1] * 2 + 1] },
        //    {out.pointlist[out.trianglelist[i + 2] * 2] , out.pointlist[out.trianglelist[i + 2] * 2 + 1]} };
        
        //   cout << "v " <<out.pointlist[out.trianglelist[i]*2] << " " << out.pointlist[out.trianglelist[i]*2 + 1] << " 0" << endl <<
        //"v " << out.pointlist[out.trianglelist[i + 1]*2] << " " << out.pointlist[out.trianglelist[i + 1]*2 + 1] << " 0" << endl <<
        //"v " << out.pointlist[out.trianglelist[i + 2]*2] << " " << out.pointlist[out.trianglelist[i + 2]*2+1] << " 0" << endl;

        //   cout << "v " << out.trianglelist[i] << " " << endl <<
        //       "v " << out.trianglelist[i + 1] << " " << endl <<
        //       "v " << out.trianglelist[i + 2] << " " << endl;
        //if (!isTriangleInPointCloud(map,point,MinMax))
        //    continue;
        outtriangles << "f " << out.trianglelist[i] + 1 << " " << out.trianglelist[i + 1] + 1 << " " << out.trianglelist[i + 2] + 1 << endl;
    }
    outtriangles.close();

    /// 
    //printf("Refined triangulation:\n\n");
    //report(&out, 0, 1, 0, 0, 0, 0);

    /* Free all allocated arrays, including those allocated by Triangle. */
    free(in.segmentlist);
    free(in.holelist);

    free(in.pointlist);
    //free(in.pointattributelist);
    free(in.pointmarkerlist);
    free(in.regionlist);
    free(mid.pointlist);
    //free(mid.pointattributelist);
    free(mid.pointmarkerlist);
    free(mid.trianglelist);
    free(mid.triangleattributelist);
    //free(mid.trianglearealist);
    free(mid.neighborlist);
    free(mid.segmentlist);
    free(mid.segmentmarkerlist);
    free(mid.edgelist);
    free(mid.edgemarkerlist);
    free(vorout.pointlist);
    free(vorout.pointattributelist);
    free(vorout.edgelist);
    free(vorout.normlist);
    free(out.pointlist);
    free(out.pointattributelist);
    free(out.trianglelist);
    free(out.triangleattributelist);
    return true;
}
bool vecvecd2vecp3f(vector<vector<double>>& proPointCloud, Point3fArray& type_points)
{
    for (auto x : proPointCloud)
    {
        type_points.push_back({ (float)x[0],(float)x[1],(float)x[2] });
    }
    return true;
}
bool outPoint(std::vector<Point2fArray>& test_points_2Dcontours)
{
    ofstream outPoints("D:/data/outPoint.pts");
    for (auto x : test_points_2Dcontours)
        for(auto y:x)
        outPoints << y.x << " " << y.y << " " << endl;
    outPoints.close();
    return true;
}
bool concreteMesherDelauney::meshAllPlaneInFilePath(string planeFilePath)
{
	vector<wstring> fileName({ /*L"D:\\data\\Q_0908\\mesh2.obj.ply" ,*/L"D:\\data\\Q_0908\\mesh7.obj.ply" });
    //AutoTestPts(fileName);
    vector < vector < vector<double>>> pointClouds;
    vector<vector<double>> points; 
    for (auto x : fileName)
    {
        string temp(x.begin(), x.end());
        GetDotCloud(points, temp);
        pointClouds.push_back(points);
        points.clear();
    } 
    vector < vector < vector<double>>> proPointClouds(fileName.size());
    
    //for (auto x : pointClouds[0])
    //    cout << x[0] << " " << x[1] << " " << x[2] << endl;

    vector<Matrix4d> rmMatrixList; 
    for (int i = 0; i < pointClouds.size(); i++)
    {
        Eigen::Matrix4d rotMatrix3d = projectionXY_(pointClouds[i], proPointClouds[i]);
        rmMatrixList.push_back(rotMatrix3d);
    }
    for (int i = 0; i < rmMatrixList.size(); i++)
    {
        void* edge_handle = Features_EdgeDetcCreateHandle();
        Point3fArray test_points;
        vecvecd2vecp3f(proPointClouds[i], test_points); 
        std::vector<Point2fArray> test_points_2Dcontours;
        std::unordered_map<int, vector<int>> tree;
        Features_getOrigMapOfContours2D("", edge_handle, { 0,0,1 }, &test_points, 15.0, test_points_2Dcontours,tree);
        //Features_getOrigMapOfContours2D("", edge_handle, { 0,0,1 }, &test_points, 0, test_points_2Dcontours);
        //(Features_getOrigMapOfContours2D(false, output_path, handle, plane_normal, {}, input_data, voxel_size, contours,tree));

        //outPoint(test_points_2Dcontours);
        string temp(fileName[i].begin(), fileName[i].end());
		//decltype(tree.begin()) it_max=(tree.begin());
  //      for (auto it = tree.begin(); it != tree.end(); it++)
		//	if (test_points_2Dcontours[it->first].size() > test_points_2Dcontours[it_max->first].size())
		//		it_max = it;
		//std::vector<Point2fArray> holelist;
		//std::vector<Point2fArray> holelist_empty;
		//for (auto x : it_max->second)
		//	holelist.emplace_back(test_points_2Dcontours[x]);
		//buildMesh_(test_points_2Dcontours[it_max->first], /*holelist_empty*/holelist, rmMatrixList[i], temp + ".obj");
		std::vector<ModuleStruct::Point2fArray> edgePointsList;
		std::vector<std::vector<ModuleStruct::Point2fArray>> holePointsList;
		for (auto it = tree.begin(); it != tree.end(); it++)
		{
			edgePointsList.emplace_back(test_points_2Dcontours[it->first]);
			std::vector<Point2fArray> holelist;
			for (auto x : it->second)
			{
				//if (test_points_2Dcontours[x].size() != 15)continue;
				holelist.emplace_back(test_points_2Dcontours[x]);
			}
			holePointsList.emplace_back(holelist);
		}
		buildMesh_(edgePointsList, holePointsList, rmMatrixList[i], temp + ".obj");
    }
    return true; 
}


bool concreteMesherDelauney::MeshPlane(std::vector<cv::Point3f>& plane_points, cv::Point3f plane_normal, const std::string plane_file_name)
{
        std::vector<cv::Point3f> proPointCloud;
		Eigen::Matrix4d rotMatrix4d = projectionXY_(plane_points, plane_normal, proPointCloud, {0.,0.,1.});

        void* edge_handle = Features_EdgeDetcCreateHandle();

        std::vector<Point2fArray> test_points_2Dcontours;
        std::unordered_map<int, vector<int>> tree;
        Features_getOrigMapOfContours2D("", edge_handle, { 0,0,1 }, &proPointCloud, 15., test_points_2Dcontours, tree);
        //Features_getOrigMapOfContours2D("", edge_handle, { 0,0,1 }, &test_points, 0, test_points_2Dcontours);
        //(Features_getOrigMapOfContours2D(false, output_path, handle, plane_normal, {}, input_data, voxel_size, contours,tree));
		std::vector<ModuleStruct::Point2fArray> edgePointsList;
		std::vector<std::vector<ModuleStruct::Point2fArray>> holePointsList;
        for (auto it = tree.begin(); it != tree.end(); it++)
        {
			edgePointsList.emplace_back(test_points_2Dcontours[it->first]);
            std::vector<Point2fArray> holelist;
            for (auto x : it->second)
				//if (test_points_2Dcontours[x].size() < 20)continue;else 
                holelist.emplace_back(test_points_2Dcontours[x]);
			holePointsList.emplace_back(holelist);
        } 
		buildMesh_(edgePointsList, holePointsList, rotMatrix4d, plane_file_name);
    return true;
} 

cv::Point3d concreteMesherDelauney::getTriCentroid(const cv::Point3d &p0, const cv::Point3d &p1, const cv::Point3d &p2)
{
	double x_Ce, y_Ce, z_Ce;
	x_Ce = (p0.x + p1.x + p2.x) / 3;
	y_Ce = (p0.y + p1.y + p2.y) / 3;
	z_Ce = (p0.z + p1.z + p2.z) / 3;


	cv::Point3d triCentroid{ x_Ce, y_Ce, z_Ce };

	return triCentroid;
}

double concreteMesherDelauney::getTdAngle(cv::Point3d &wall_normal, cv::Point3d &ground_normal)
{
	double vectorDot = wall_normal.x * ground_normal.x + wall_normal.y * ground_normal.y + wall_normal.z * ground_normal.z;
	double vectorMold1 = sqrt(pow(wall_normal.x, 2) + pow(wall_normal.y, 2) + pow(wall_normal.z, 2));
	double vectorMold2 = sqrt(pow(ground_normal.x, 2) + pow(ground_normal.y, 2) + pow(ground_normal.z, 2));

	double cosAngle = vectorDot / (vectorMold1 * vectorMold2);
	double radian = acos(cosAngle);

	return (double)(180 / M_PI * radian);
}

void concreteMesherDelauney::GetnormalBy3Pts(cv::Point3d &p1, cv::Point3d &p2, cv::Point3d &p3, cv::Point3d &normalByPlaneCorner)
{
	float x1 = p2.x - p1.x; float y1 = p2.y - p1.y; float z1 = p2.z - p1.z;
	float x2 = p3.x - p1.x; float y2 = p3.y - p1.y; float z2 = p3.z - p1.z;
	float a = y1 * z2 - y2 * z1;
	float b = z1 * x2 - z2 * x1;
	float c = x1 * y2 - x2 * y1;

	float length = sqrt(a*a + b * b + c * c);
	a = a / length;
	b = b / length;
	c = c / length;
	normalByPlaneCorner.x = a;
	normalByPlaneCorner.y = b;
	normalByPlaneCorner.z = c;
}

bool concreteMesherDelauney::buildMesh(std::vector<ModuleStruct::Point3fArray>& edgePointsList,
	std::vector<std::vector<ModuleStruct::Point3fArray>>& holePointsList,
	Eigen::Matrix4d& rmMatrix, std::string plane_file_name)
{
	ofstream outtriangles(plane_file_name);
	size_t offset = 1;
	vector < vector<unsigned long long>> faceList;
	std::vector<cv::Point3d> corners_list;
	corners_list.clear();
	for (int i = 0; i < edgePointsList.size(); i++)
	{
		struct triangulateio in, mid, out, vorout;
		vector<double> pointCloud;

		in = inDotCloud(edgePointsList[i], holePointsList[i]);


		//for (size_t i = 0; i < in.numberofpoints * 2; i = i + 2)
		//{
		//    cout << in.pointlist[i] << "," << in.pointlist[i + 1] << endl;
		//}

		/* Make necessary initializations so that Triangle can return a */
		/*   triangulation in `mid' and a voronoi diagram in `vorout'.  */

		mid.pointlist = (REAL*)NULL;            /* Not needed if -N switch used. */
		/* Not needed if -N switch used or number of point attributes is zero: */
		mid.pointattributelist = (REAL*)NULL;
		mid.pointmarkerlist = (int*)NULL; /* Not needed if -N or -B switch used. */
		mid.trianglelist = (int*)NULL;          /* Not needed if -E switch used. */
		/* Not needed if -E switch used or number of triangle attributes is zero: */
		mid.triangleattributelist = (REAL*)NULL;
		mid.neighborlist = (int*)NULL;         /* Needed only if -n switch used. */
		/* Needed only if segments are output (-p or -c) and -P not used: */
		mid.segmentlist = (int*)NULL;
		/* Needed only if segments are output (-p or -c) and -P and -B not used: */
		mid.segmentmarkerlist = (int*)NULL;
		mid.edgelist = (int*)NULL;             /* Needed only if -e switch used. */
		mid.edgemarkerlist = (int*)NULL;   /* Needed if -e used and -B not used. */

		vorout.pointlist = (REAL*)NULL;        /* Needed only if -v switch used. */
		/* Needed only if -v switch used and number of attributes is not zero: */
		vorout.pointattributelist = (REAL*)NULL;
		vorout.edgelist = (int*)NULL;          /* Needed only if -v switch used. */
		vorout.normlist = (REAL*)NULL;         /* Needed only if -v switch used. */

		triangulate("pzAevnQ", &in, &mid, &vorout);


		out.pointlist = (REAL*)NULL;            /* Not needed if -N switch used. */
		/* Not needed if -N switch used or number of attributes is zero: */
		out.pointattributelist = (REAL*)NULL;
		out.trianglelist = (int*)NULL;          /* Not needed if -E switch used. */
		/* Not needed if -E switch used or number of triangle attributes is zero: */
		out.triangleattributelist = (REAL*)NULL;

		/* Refine the triangulation according to the attached */
		/*   triangle area constraints.                       */

		//triangulate("prazBP", &mid, &out, (struct triangulateio*)NULL);
		triangulate("pzBPQ", &mid, &out, (struct triangulateio*)NULL);

		auto tempOffset = 0;
		for (size_t i = 0; i < out.numberofpoints * 2; i = i + 2)
		{
			Vector4d point(out.pointlist[i], out.pointlist[i + 1], 0, 1);
			Vector4d point3d = rmMatrix * point;
			outtriangles << "v " << point3d[0] << " " << point3d[1] << " " << point3d[2] << endl;
			//outtriangles << "v " << out.pointlist[i] << " " << out.pointlist[i + 1] << " 0" << endl;
			corners_list.emplace_back(cv::Point3d(point3d[0], point3d[1], point3d[2]));

			tempOffset++;
		}

		for (size_t i = 0; i < out.numberoftriangles * out.numberofcorners; i += 3)
		{
			//outtriangles << "f " << out.trianglelist[i] + offset << " " << out.trianglelist[i + 1] + offset << " " << out.trianglelist[i + 2] + offset << endl;
			faceList.push_back({ out.trianglelist[i] + offset ,out.trianglelist[i + 1] + offset,out.trianglelist[i + 2] + offset });
		}
		offset += tempOffset;//outtriangles.();		 //outtriangles.end - outtriangles.beg;
		/// 
		//printf("Refined triangulation:\n\n");
		//report(&out, 0, 1, 0, 0, 0, 0);

		/* Free all allocated arrays, including those allocated by Triangle. */
		free(in.segmentlist);
		free(in.holelist);

		free(in.pointlist);
		//free(in.pointattributelist);
		free(in.pointmarkerlist);
		free(in.regionlist);
		free(mid.pointlist);
		//free(mid.pointattributelist);
		free(mid.pointmarkerlist);
		free(mid.trianglelist);
		free(mid.triangleattributelist);
		//free(mid.trianglearealist);
		free(mid.neighborlist);
		free(mid.segmentlist);
		free(mid.segmentmarkerlist);
		free(mid.edgelist);
		free(mid.edgemarkerlist);
		free(vorout.pointlist);
		free(vorout.pointattributelist);
		free(vorout.edgelist);
		free(vorout.normlist);
		free(out.pointlist);
		free(out.pointattributelist);
		free(out.trianglelist);
		free(out.triangleattributelist);
	}
	for (auto x : faceList)
	{
		cv::Point3d p0, p1, p2;
		p0 = corners_list[x[0] - 1];
		p1 = corners_list[x[1] - 1];
		p2 = corners_list[x[2] - 1];

		cv::Point3d triangle_normal;
		GetnormalBy3Pts(p0, p1, p2, triangle_normal);

		cv::Point3d triCentroid;
		triCentroid = getTriCentroid(p0, p1, p2);

		cv::Point3d triCentroidToOrigin{ -triCentroid.x, -triCentroid.y, -triCentroid.z };

		double vetorialAngle = getTdAngle(triangle_normal, triCentroidToOrigin);

		if (vetorialAngle > 90)
		{
			outtriangles << "f " << x[2] << " " << x[1] << " " << x[0] << endl;
		}
		else
		{
			outtriangles << "f " << x[0] << " " << x[1] << " " << x[2] << endl;
		}
	}

	outtriangles.close();
	return true;
}