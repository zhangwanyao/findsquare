#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>

typedef enum
{
	PIXEL_CIRCLE,
	PIXEL_RECT
}PIXEL_SIZE_TYPE;


typedef enum
{
	K_ORIGIN,
	K_STRUCTURE
}CONTOUR_IMAGE_TYPE;

class CContour
{
public:
	CContour();
	~CContour();

	bool GetContourImg(
		const CONTOUR_IMAGE_TYPE contour_type,
		const int& wall_id, 
		const std::string& mesh_path,
		const float& imgExpectMaxPixel,
		const float& rect_radius,
		const cv::Point3f& plane_normal,
		const cv::Point3f& plane_center,
		const std::vector<std::pair<float, cv::Point3f>>& defect_pair,
		cv::Mat& contourImg);

private:

	bool ReadAndSaveObjAndMtlFile(const int& wall_id, const std::string& save_path);
	bool GetObjPointHelper(const int& wall_id, const std::string& mesh_path, std::vector<cv::Point3f>& obj_points);

	bool GetContourImgFromPixeltype(
		const CONTOUR_IMAGE_TYPE contour_type, 
		const int& wall_id,
		const std::string& mesh_path,
		const float& imgExpectMaxPixel,
		const float& rect_radius,
		const cv::Point3f& plane_normal,
		const cv::Point3f& plane_center,
		const std::vector<std::pair<float, cv::Point3f>>& defect_pair,
		const PIXEL_SIZE_TYPE& pixelType,
		cv::Mat& isohpyseMat);

	void GetIsohypseImgFromType(const std::vector<std::pair<float, cv::Point3f>>& defect_pair, const PIXEL_SIZE_TYPE& pixel_type, cv::Mat& img_def, cv::Mat& img_ori);
	void GetIsohypseImgCircleHelper(const std::vector<cv::Point3f>& defect_pt_lookat, const std::vector<float>& defect_value, cv::Mat& img_def, cv::Mat& img_ori);
	void GetIsohypseImgRect(const std::vector<cv::Point3f>& defect_pt_lookat, const std::vector<float>& defect_value, cv::Mat& img_def, cv::Mat& img_ori);
	void GetIsohypseImgCircle(const std::vector<cv::Point3f>& defect_pt_lookat, const std::vector<float>& defect_value, cv::Mat& img_def, cv::Mat& img_ori);

	void GetIsohypseImgRectHelper(const std::vector<cv::Point3f>& defect_pt_lookat, const std::vector<float>& defect_value, cv::Mat& img_def, cv::Mat& img_ori);

	void ChangeDefectPoints(const std::vector<std::pair<float, cv::Point3f>>& defect_pair, std::vector<float>& defect_val, std::vector<cv::Point3f>& defect_pt);

	////normal and wall 4 corner
	//void GetLookatMat(const cv::Mat& wallNormal, const cv::Mat& wallCorner, const int& n, cv::Mat &lookAt);

	//========get img width height pixelsize funtion start==============//
	void GetImgWidthHeightHelper(const int& imgExpectMaxPixel,const float& width3d, const float& height3d,int&imgWidth, int&imgHeight);

	void GetImg2dInfoFromType(const PIXEL_SIZE_TYPE& pixel_type);

	void GetImgWidthHeightAndPixelSize(
		const int& imgExpectMaxPixel,
		const float& rect_Radius,
		const float& width3d,
		const float& height3d,
		int&imgWidth, 
		int&imgHeight, 
		int& pixelSize, 
		const PIXEL_SIZE_TYPE& pixel_type);

	void GetImgWidthHeightAndPixelSizeCircleHelper(
		const int& imgExpectMaxPixel,
		const float& rect_Radius,
		const float& width3d,
		const float& height3d,
		int&imgWidth, 
		int&imgHeight, 
		int& pixelSize);

	void GetImgWidthHeightAndPixelSizeRectHelper(
		const int& imgExpectMaxPixel,
		const float& rect_Radius,
		const float& width3d,
		const float& height3d,
		int&imgWidth,
		int&imgHeight,
		int& pixelSize);
	//========get img width height pixelsize funtion end==============//

	void GetRGBValueFromLevel(const float& colorLevel, int& clr_r, int& clr_g, int& clr_b);
	void GetTextureMap(const cv::Mat& img_def, cv::Mat &textureImg);
	void PutHoleInContourImg(const cv::Mat& img_ori, cv::Mat &coutourImg);
	void GetColorCoutourImgHelper(const cv::Mat& img_def, const cv::Mat& img_ori,cv::Mat &coutourImg);
	void GetColorCoutourImg(cv::Mat &coutourImg);

	void IsohypseImgDrawTypeRect(const cv::Mat& defPointsLookat, const float& rect_Radius,
		const int& imgHeight, const int& imgWidth, const float& height3d, const float& width3d,
		const float& min3d_x, const float& min3d_y, const int& pixelSize,
		cv::Mat& img_def, cv::Mat& img_ori);

private:
	cv::Mat mLookAt;

	//float m3dMinX;
	//float m3dMaxX;
	//float m3dMinY;
	//float m3dMaxY;
	cv::Point3f mLookatMinPt;
	cv::Point3f mLookatMaxPt;
	int mImgHeight;
	int mImgWidth;
	int mPixelSize;

	float mImgLegth;//defult 1024 imgExpectMaxPixel,
	float mRectRaidus;
	float mWidth3d;
	float mHeight3d;
	float mMin3dX;
	float mMin3dY;
	cv::Mat mImgDef;
	cv::Mat mImgOri;


};

