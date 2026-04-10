#include "Contour.h"
#include "MathOperation.h"
#include "mesh_tool.h"
#include "LookatOperation.h"
#include "FindTool.h"
#include "InOutData.h"
#define DEBUG_CONTOUR_TEST false

CContour::CContour()
{
}

CContour::~CContour()
{
}

void CContour::ChangeDefectPoints(const std::vector<std::pair<float, cv::Point3f>>& defect_pair, std::vector<float>& defect_val,std::vector<cv::Point3f>& defect_pt )
{
	size_t pt_num = defect_pair.size();
	defect_pt.resize(pt_num);
	defect_val.resize(pt_num);
	for (int i = 0; i<pt_num; i++)
	{
		defect_val[i]= defect_pair[i].first;
		defect_pt[i]=defect_pair[i].second;
	}

}


bool CContour::GetContourImg(
	const CONTOUR_IMAGE_TYPE contour_type,
	const int& wall_id, 
	const std::string& mesh_path,
	const float& imgExpectMaxPixel,
	const float& rect_radius,
	const cv::Point3f& plane_normal,
	const cv::Point3f& plane_center,
	const std::vector<std::pair<float, cv::Point3f>>& defect_pair,
	cv::Mat& contourImg)
{
	PIXEL_SIZE_TYPE pixelType = PIXEL_SIZE_TYPE::PIXEL_RECT;
	if (abs(plane_normal.z)>=0.9)
	{
		pixelType = PIXEL_SIZE_TYPE::PIXEL_CIRCLE;
	}

	return GetContourImgFromPixeltype(contour_type,wall_id, mesh_path, imgExpectMaxPixel,rect_radius,plane_normal,plane_center,defect_pair,pixelType, contourImg);

}



bool CContour::GetContourImgFromPixeltype(
	const CONTOUR_IMAGE_TYPE contour_type, 
	const int& wall_id,
	const std::string& mesh_path,
	const float& imgExpectMaxPixel,
	const float& rect_radius,
	const cv::Point3f& plane_normal,
	const cv::Point3f& plane_center,
	const std::vector<std::pair<float, cv::Point3f>>& defect_pair,
	const PIXEL_SIZE_TYPE& pixelType,
	cv::Mat& contourImg)
{
	if (defect_pair.size()==0)
	{
		std::cout << "  CContour::GetContourImgFromPixeltype defect_pair size is zero!" << std::endl;
		return false;
	}
	//std::cout << "===wall_id:" << wall_id << " rect_radius:" << rect_radius << std::endl;
	//set value: mImgLegth, mRectRaidus
	mImgLegth = imgExpectMaxPixel;
	mRectRaidus = rect_radius;
	//set matvalue: mLookAt
	CLookAt::GetLookAtMat(plane_normal, plane_center, 100, mLookAt);

	//std::vector<cv::Point3f> plane_vetrtice_lookat;
	//CLookAt::RotationPoints(plane_vetrtice, plane_vetrtice_lookat, mLookAt);
	//FindMinMaxHelper(plane_vetrtice_lookat, mLookatMinPt, mLookatMaxPt);

	std::vector<float> defect_val;
	std::vector<cv::Point3f> defect_pt;
	ChangeDefectPoints(defect_pair, defect_val, defect_pt);
	if (DEBUG_CONTOUR_TEST)
	{
		IOData::SavePLYPoints3f(mesh_path+"defect_" + std::to_string(wall_id) + "_.ply", defect_pt, false);
	}

	std::vector<cv::Point3f> defect_pt_lookat;
	CLookAt::RotationPoints(defect_pt, defect_pt_lookat, mLookAt);
	if (DEBUG_CONTOUR_TEST)
	{
		IOData::SavePLYPoints3f(mesh_path + "defect_lookat" + std::to_string(wall_id) + "_.ply", defect_pt_lookat, false);
	}

	FindTool::FindMinMaxHelper(defect_pt_lookat, mLookatMinPt, mLookatMaxPt);
	//std::cout << wall_id << "=2= mLookatMinPt: " << mLookatMinPt << " mLookatMaxPt:" << mLookatMaxPt << std::endl;
	if (contour_type == K_STRUCTURE)
	{
		std::vector<cv::Point3f> obj_points;
		bool isObjPoint= GetObjPointHelper(wall_id, mesh_path, obj_points);
		if (!isObjPoint)
		{
			std::cout << " K_STRUCTURE  isObjPoint false !" << std::endl;
			return false;
		}

		std::vector<cv::Point3f> obj_points_lookat;
		CLookAt::RotationPoints(obj_points, obj_points_lookat, mLookAt);
		cv::Point3f minPt, maxPt;
		FindTool::FindMinMaxHelper(obj_points_lookat, minPt, maxPt);

		mLookatMinPt.x = std::min(mLookatMinPt.x , minPt.x);
		mLookatMinPt.y = std::min(mLookatMinPt.y, minPt.y);
		mLookatMinPt.z = std::min(mLookatMinPt.z, minPt.z);
		mLookatMaxPt.x = std::max(mLookatMaxPt.x , maxPt.x);
		mLookatMaxPt.y = std::max(mLookatMaxPt.y, maxPt.y);
		mLookatMaxPt.z = std::max(mLookatMaxPt.z, maxPt.z);

	}
	//set value: mWidth3d, mHeight3d
	mWidth3d = mLookatMaxPt.x - mLookatMinPt.x;
	mHeight3d = mLookatMaxPt.y - mLookatMinPt.y;
	mMin3dX= mLookatMinPt.x;
	mMin3dY= mLookatMinPt.y;

	if (mWidth3d < 25.f || mHeight3d < 25.f)
		return false;
	//set value:mImgWidth, mImgHeight, mPixelSize 
	//GetImgWidthHeightAndPixelSize(imgExpectMaxPixel, rect_radius, mWidth3d, mHeight3d, mImgWidth, mImgHeight, mPixelSize, pixelType);

	//set value:mImgWidth, mImgHeight, mPixelSize 
	GetImg2dInfoFromType(pixelType);
	//std::cout << "SizeRect==== mHeight3d:" << mHeight3d << " mWidth3d:" << mWidth3d << " mPixelSize:" << mPixelSize << std::endl;
	GetIsohypseImgFromType(defect_pair, pixelType, mImgDef, mImgOri);
	GetColorCoutourImg(contourImg);
	bool isSavemtl=ReadAndSaveObjAndMtlFile(wall_id, mesh_path);
	if (!isSavemtl)
	{
		std::cout << "CContour::GetContourImgFromPixeltype save mtl false!" << std::endl;
		return false;
	}
	return true;
}

bool CContour::GetObjPointHelper(const int& wall_id, const std::string& mesh_path, std::vector<cv::Point3f>& obj_points)
{
	std::vector<cv::Point3f> points;
	std::vector<cv::Vec3i> face;
	std::string obj_path = mesh_path + "mesh" + std::to_string(wall_id) + ".obj";


	CMeshTool meshTool;
	//¶ÁÈ¡obj
	bool isGet = meshTool.GetObjData(obj_path, points, face);
	if (!isGet)
	{
		return false;
	}
	obj_points = points;
	return true;
}

bool CContour::ReadAndSaveObjAndMtlFile(const int& wall_id, const std::string& mesh_path)
{
	std::vector<cv::Point3f> points;
	std::vector<cv::Vec3i> face;
	//std::string obj_path = "origin_meshes\\mesh" + std::to_string(wall_id) + ".obj";
	std::string obj_path = mesh_path +"mesh"+ std::to_string(wall_id) + ".obj";


	CMeshTool meshTool;
	//读取obj
	bool isGet = meshTool.GetObjData(obj_path, points, face);
	if (!isGet)
	{
		return false;
	}
	if (DEBUG_CONTOUR_TEST)
	{
		IOData::SavePLYPoints3f(mesh_path + "point_" + std::to_string(wall_id) + "_.ply", points, false);
	}
	std::vector<cv::Point3f> rpoints;

	CLookAt::RotationPoints(points, rpoints, mLookAt);
	if (DEBUG_CONTOUR_TEST)
	{
		IOData::SavePLYPoints3f(mesh_path + "point_lookat_" + std::to_string(wall_id) + "_.ply", rpoints, false);
	}
	cv::Point3f minPt, maxPt;
	FindTool::FindMinMaxHelper(rpoints, minPt, maxPt);

	float min_3dx = minPt.x;
	float min_3dy = minPt.y;
	float width3d = maxPt.x - minPt.x;
	float height3d = maxPt.y - minPt.y;
	//重新投影位置
	std::vector<cv::Vec2f> vt;
	for (int i = 0; i < rpoints.size(); i++)
	{
		int x = (rpoints[i].x - min_3dx) * mImgWidth / width3d;
		int y = (rpoints[i].y - min_3dy) * mImgHeight / height3d;

		cv::Vec2f texture;
		texture[0] = float(x) / mImgWidth;
		texture[1] = 1.0 - float(y) / mImgHeight;
		vt.push_back(texture);
	}

	//std::stringstream saveobjPath;
	//saveobjPath << meshPath << "\\" << "mesh" << meshId << ".obj";
	//std::string save_obj_path = "origin_meshes\\mesh_" + std::to_string(wall_id) + ".obj";
	std::string save_obj_path = obj_path;
	bool isSvae=meshTool.SaveObjData(wall_id, save_obj_path, points, face, vt);
	if (!isSvae)
	{
		return false;
	}
	//std::stringstream mtlPath;
	//mtlPath << meshPath << "\\" << "mesh" << meshId << ".mtl";

	std::string mtlPath = mesh_path + "mesh" + std::to_string(wall_id) + ".mtl";
	std::string str_name = "mesh";
	meshTool.SaveMtlfile(wall_id, str_name, mtlPath);
	return true;
}



//========get img width height pixelsize funtion start==============//
void CContour::GetImgWidthHeightHelper(const int& imgExpectMaxPixel,const float& width3d, const float& height3d,int&imgWidth, int&imgHeight)
{
	//int imgWidthInput = 1024; //default
	if (height3d<width3d)
	{
		imgWidth = imgExpectMaxPixel;
		imgHeight = height3d / width3d * imgWidth;
		//int pixelSize = 1.414 * 100 * imgWidth / width3d;
		//pixelSize = 1.414 * rect_Radius * imgWidth / width3d;
	}
	else
	{
		imgHeight = imgExpectMaxPixel;
		imgWidth = width3d / height3d *imgHeight;
		//pixelSize = 1.414 * rect_Radius * imgHeight / height3d;
	}
}

void CContour::GetImg2dInfoFromType(const PIXEL_SIZE_TYPE& pixel_type)
{
	GetImgWidthHeightAndPixelSize(mImgLegth,mRectRaidus,mWidth3d,mHeight3d,mImgWidth, mImgHeight, mPixelSize, pixel_type);
}

void CContour::GetImgWidthHeightAndPixelSize(
	const int& imgExpectMaxPixel,
	const float& rect_Radius,
	const float& width3d,
	const float& height3d,
	int&imgWidth, int&imgHeight, int& pixelSize, const PIXEL_SIZE_TYPE& pixel_type)
{
	if (pixel_type == PIXEL_CIRCLE)
	{
		GetImgWidthHeightAndPixelSizeCircleHelper(imgExpectMaxPixel, rect_Radius, width3d, height3d, imgWidth, imgHeight, pixelSize);
		//std::cout << "==============PIXEL_CIRCLE============== " << std::endl;
	}
	else if (pixel_type == PIXEL_RECT)
	{
		GetImgWidthHeightAndPixelSizeRectHelper(imgExpectMaxPixel, rect_Radius, width3d, height3d, imgWidth, imgHeight, pixelSize);
		//std::cout << "==============PIXEL_RECT============== " << std::endl;
	}
}

void CContour::GetImgWidthHeightAndPixelSizeCircleHelper(
	const int& imgExpectMaxPixel,
	const float& rect_Radius,
	const float& width3d,
	const float& height3d,
	int&imgWidth, int&imgHeight, int& pixelSize)
{
	GetImgWidthHeightHelper(imgExpectMaxPixel, width3d, height3d, imgWidth, imgHeight);
	pixelSize = 1;
	//circle
	if (height3d<width3d)
	{
		//int pixelSize = 1.414 * 100 * imgWidth / width3d;
		pixelSize = 1.414 * rect_Radius * imgWidth / width3d;
	}
	else
	{
		pixelSize = 1.414 * rect_Radius * imgHeight / height3d;
	}

}

void CContour::GetImgWidthHeightAndPixelSizeRectHelper(
	const int& imgExpectMaxPixel,
	const float& rect_Radius,
	const float& width3d,
	const float& height3d,
	int&imgWidth, int&imgHeight, int& pixelSize)
{
	GetImgWidthHeightHelper(imgExpectMaxPixel, width3d, height3d, imgWidth, imgHeight);
	pixelSize = 1;
	//rect
	if (height3d<width3d)
	{
		pixelSize = rect_Radius * imgWidth / width3d + 1;
	}
	else
	{
		pixelSize = rect_Radius * imgHeight / height3d + 1;
	}
	//std::cout << " pixelSize:" << pixelSize << std::endl;

}
//========get img width height pixelsize funtion end==============//

void CContour::GetIsohypseImgCircleHelper(const std::vector<cv::Point3f>& defect_pt_lookat, const std::vector<float>& defect_value, cv::Mat& img_def, cv::Mat& img_ori)
{
	cv::Mat img;
	img.create(mImgHeight, mImgWidth, CV_32F);
	img = 0;

	cv::Mat imgOri;
	imgOri.create(mImgHeight, mImgWidth, CV_8UC1);
	imgOri = 0;

	float width_rate= mImgWidth / mWidth3d;
	float height_rate = mImgHeight / mHeight3d;
	for (int i = 0; i < defect_pt_lookat.size(); i++)
	{
		cv::Point3f pt = defect_pt_lookat[i];
		int x = (pt.x - mMin3dX) * width_rate;
		int y = (pt.y - mMin3dY) * height_rate;

		x += mPixelSize / 2;
		y -= mPixelSize / 2;

		for (int k1 = -mPixelSize / 2; k1 <= mPixelSize / 2; k1++)
		{
			for (int k2 = -mPixelSize / 2; k2 <= mPixelSize / 2; k2++)
			{
				if ((y + k1) >= 0 && (y + k1) < mImgHeight && (x + k2) < mImgWidth && (x + k2) >= 0)
				{
					float dr = sqrt((k1) * (k1)+(k2)*(k2));
					if (dr <= mPixelSize / 2)
					{
						if (abs(img.at<float>(y + k1, x + k2)) < abs(defect_value[i]))
						{
							img.at<float>(y + k1, x + k2) = defect_value[i];
							imgOri.at<uchar>(y + k1, x + k2) = 255;
						}
					}
				}
			}
			//img.at<float>(y, x) = defPoints.at<float>(i, 3);
		}
	}

	img_def = img.clone();
	img_ori = imgOri.clone();
}

void CContour::GetIsohypseImgRectHelper(const std::vector<cv::Point3f>& defect_pt_lookat, const std::vector<float>& defect_value, cv::Mat& img_def, cv::Mat& img_ori)
{
	cv::Mat img;
	img.create(mImgHeight, mImgWidth, CV_32F);
	img = 0;

	cv::Mat imgOri;
	imgOri.create(mImgHeight, mImgWidth, CV_8UC1);
	imgOri = 0;

	float width_rate= mImgWidth / mWidth3d;
	float height_rate = mImgHeight / mHeight3d;
	for (int i = 0; i < defect_pt_lookat.size(); i++)
	{
		cv::Point3f pt = defect_pt_lookat[i];
		int x = (pt.x - mMin3dX) * width_rate;
		int y = (pt.y - mMin3dY) * height_rate;

		x += mPixelSize / 2;
		y -= mPixelSize / 2;

		for (int k1 = -mPixelSize / 2; k1 <= mPixelSize / 2; k1++)
		{
			for (int k2 = -mPixelSize / 2; k2 <= mPixelSize / 2; k2++)
			{
				if ((y + k1) >= 0 && (y + k1) < mImgHeight && (x + k2) < mImgWidth && (x + k2) >= 0)
				{
					img.at<float>(y + k1, x + k2) = defect_value[i];
					imgOri.at<uchar>(y + k1, x + k2) = 255;
				}
			}
		}
	}

	//int kernel = ((int)rect_Radius / 8) * 2 + 1;
	////std::cout << kernel << std::endl;
	//cv::GaussianBlur(img, img, cv::Size(kernel, kernel), 9, 9);
	img_def = img.clone();
	img_ori = imgOri.clone();
}
void CContour::GetIsohypseImgRect(const std::vector<cv::Point3f>& defect_pt_lookat, const std::vector<float>& defect_value, cv::Mat& img_def, cv::Mat& img_ori)
{
	GetIsohypseImgRectHelper(defect_pt_lookat, defect_value, img_def, img_ori);
	int kernel = ((int)mRectRaidus / 8) * 2 + 1;
	cv::GaussianBlur(img_def, img_def, cv::Size(kernel, kernel), 9, 9);
}

void CContour::GetIsohypseImgCircle(const std::vector<cv::Point3f>& defect_pt_lookat, const std::vector<float>& defect_value, cv::Mat& img_def, cv::Mat& img_ori)
{
	GetIsohypseImgCircleHelper(defect_pt_lookat, defect_value, img_def, img_ori);
	int kernel = ((int)mRectRaidus / 8) * 2 + 1;
	cv::GaussianBlur(img_def, img_def, cv::Size(kernel, kernel), 9, 9);
	cv::dilate(img_ori, img_ori, cv::Mat::ones(25, 25, CV_8U));
	cv::erode(img_ori, img_ori, cv::Mat::ones(25, 25, CV_8U));
}

void CContour::GetIsohypseImgFromType(const std::vector<std::pair<float, cv::Point3f>>& defect_pair, const PIXEL_SIZE_TYPE& pixel_type, cv::Mat& img_def, cv::Mat& img_ori)
{
	std::vector<float> defect_value;
	std::vector<cv::Point3f> defect_pt;
	ChangeDefectPoints(defect_pair, defect_value, defect_pt);

	std::vector<cv::Point3f> defect_pt_lookat;
	CLookAt::RotationPoints(defect_pt, defect_pt_lookat, mLookAt);

	if (pixel_type== PIXEL_SIZE_TYPE::PIXEL_CIRCLE)
	{
		GetIsohypseImgCircle(defect_pt_lookat, defect_value, img_def, img_ori);
	}
	else if(pixel_type == PIXEL_SIZE_TYPE::PIXEL_RECT)
	{
		GetIsohypseImgRect(defect_pt_lookat, defect_value, img_def, img_ori);
	}
}

void CContour::IsohypseImgDrawTypeRect(const cv::Mat& defPointsLookat, const float& rect_Radius,
	const int& imgHeight, const int& imgWidth, const float& height3d, const float& width3d,
	const float& min3d_x, const float& min3d_y, const int& pixelSize,
	cv::Mat& img_def, cv::Mat& img_ori)
{
	cv::Mat img;
	img.create(imgHeight, imgWidth, CV_32F);
	img = 0;

	cv::Mat imgOri;
	imgOri.create(imgHeight, imgWidth, CV_8UC1);
	imgOri = 0;

	for (int i = 0; i < defPointsLookat.rows; i++)
	{
		int x = (defPointsLookat.at<float>(i, 0) - min3d_x) * imgWidth / width3d;
		int y = (defPointsLookat.at<float>(i, 1) - min3d_y) * imgHeight / height3d;

		x += pixelSize / 2;
		y -= pixelSize / 2;

		for (int k1 = -pixelSize / 2; k1 <= pixelSize / 2; k1++)
		{
			for (int k2 = -pixelSize / 2; k2 <= pixelSize / 2; k2++)
			{
				if ((y + k1) >= 0 && (y + k1) < imgHeight && (x + k2) < imgWidth && (x + k2) >= 0)
				{
					img.at<float>(y + k1, x + k2) = defPointsLookat.at<float>(i, 3);
					imgOri.at<uchar>(y + k1, x + k2) = 255;
				}
			}
		}
	}

	int kernel = ((int)rect_Radius / 8) * 2 + 1;
	//std::cout << kernel << std::endl;
	cv::GaussianBlur(img, img, cv::Size(kernel, kernel), 9, 9);
	//cv::dilate(imgOri, imgOri, cv::Mat::ones(5, 5, CV_8U));
	//cv::erode(imgOri, imgOri, cv::Mat::ones(5, 5, CV_8U));

	img_def = img.clone();
	img_ori = imgOri.clone();

}

void CContour::GetRGBValueFromLevel(const float& colorLevel, int& clr_r, int& clr_g, int& clr_b)
{
	clr_r = 128;//defalut
	clr_g = 255;//defalut
	clr_b = 128;//defalut
	if (colorLevel < 5 && colorLevel >= 3)
	{
		clr_r = 255;
		clr_g = 229;
		clr_b = 150;
	}
	//»ÆÉ«
	else if (colorLevel >= 5 && colorLevel < 7)
	{
		clr_r = 255;
		clr_g = 174;
		clr_b = 0;
	}
	//³ÈÉ«
	else if (colorLevel >= 7 && colorLevel < 9)
	{
		clr_r = 255;
		clr_g = 97;
		clr_b = 0;
	}
	//ºìÉ«
	else if (colorLevel >= 9)
	{
		clr_r = 255;
		clr_g = 0;
		clr_b = 0;
	}
	//ºþÀ¶É«
	else if (colorLevel > -7 && colorLevel <= -5)
	{
		clr_r = 30;
		clr_g = 144;
		clr_b = 255;


		/*clr_r = 193;
		clr_g = 210;
		clr_b = 240;*/
	}
	//Ç³À¶É«
	else if (colorLevel >= -9 && colorLevel <= -7)
	{
		clr_r = 51;
		clr_g = 102;
		clr_b = 255;

		/*clr_r = 193;
		clr_g = 210;
		clr_b = 240;*/
	}
	//ÉîÀ¶É«
	else if (colorLevel <= -9)
	{
		clr_r = 0;
		clr_g = 0;
		clr_b = 255;
	}
	else if (colorLevel > -5 && colorLevel <= -3)
	{
		clr_r = 179;
		clr_g = 221;
		clr_b = 255;
	}
	else if (colorLevel > -3 && colorLevel <= -1)
	{
		clr_r = 208;
		clr_g = 238;
		clr_b = 255;
	}
	else if (colorLevel >= 1 && colorLevel < 3)
	{
		clr_r = 254;
		clr_g = 255;
		clr_b = 210;
	}

}

void CContour::GetTextureMap(const cv::Mat& img_def, cv::Mat &textureImg)
{
	//cv::Mat denggaoxian = cv::Mat(img.rows, img.cols, CV_8UC3);

	//int Level = 5;
	cv::Mat convexPoints = img_def.clone();
	convexPoints.setTo(0, convexPoints<2);

	cv::Mat concavePoints = img_def.clone();
	concavePoints.setTo(0, concavePoints>(-2));
	//cv::Mat mapPoints = convexPoints+ concavePoints;
	cv::Mat mapPoints = img_def.clone();

	//ÑÕÉ«ÐÅÏ¢
	for (int row = 0; row < mapPoints.rows; row++)
	{
		for (int col = 0; col < mapPoints.cols; col++)
		{
			int clr_r = 128;
			int clr_g = 255;
			int clr_b = 128;
			float colorLevel = mapPoints.at<float>(row, col);
			GetRGBValueFromLevel(colorLevel, clr_r, clr_g, clr_b);

			// MatÖÐRGB´æ·ÅË³ÐòÎª BGR
			textureImg.at<cv::Vec3b>(row, col)[0] = clr_b;
			textureImg.at<cv::Vec3b>(row, col)[1] = clr_g;
			textureImg.at<cv::Vec3b>(row, col)[2] = clr_r;

		}

	}
	//cv::imwrite("mesh6.jpg", textureImg);

}

void CContour::PutHoleInContourImg(const cv::Mat& img_ori, cv::Mat &contourImg)
{
	for (int row = 0; row < img_ori.rows; row++)
	{
		for (int col = 0; col < img_ori.cols; col++)
		{
			if (img_ori.at<uchar>(row, col) == 0)
			{
				contourImg.at<cv::Vec3b>(row, col)[0] = 255;
				contourImg.at<cv::Vec3b>(row, col)[1] = 255;
				contourImg.at<cv::Vec3b>(row, col)[2] = 255;
			}
		}
	}
}

void CContour::GetColorCoutourImgHelper(const cv::Mat& img_def, const cv::Mat& img_ori, cv::Mat &contourImg)
{
	contourImg = cv::Mat(img_def.rows, img_def.cols, CV_8UC3);
	//get colour img
	GetTextureMap(img_def, contourImg);
	PutHoleInContourImg(img_ori, contourImg);
}

void CContour::GetColorCoutourImg(cv::Mat &contourImg)
{
	GetColorCoutourImgHelper(mImgDef, mImgOri, contourImg);
}