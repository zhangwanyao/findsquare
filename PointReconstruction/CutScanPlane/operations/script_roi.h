#ifndef _SCRIPT_ROI_H_
#define _SCRIPT_ROI_H_
#include <vector>
#include <opencv2\core.hpp>
#include "../doc/doc_bim.h"
/**
* \brief script of locate ROI
* \author Bichen JING
*/
class Script_ROI
{
public:
	/**
	* \brief type of finding ROI
	*/
	enum class TYPE_ROI
	{
		ROI_WALL_LESS,
		ROI_WALL_BASE,
		ROI_BEAM_BASE,
		ROI_NO_CUT,
		MANUAL,
		DEMO		
	};

	enum class TYPE_PLANE
	{
		CEILING,
		GROUND
	};
	/**
	* \brief constructor
	*/
	Script_ROI();
	/**
	* \brief destructor
	*/
	~Script_ROI();
	/**
	* \brief process
	*/
	bool process(const TYPE_ROI type_roi, PlaneCutResultInterface& cutResult,int station_size, bool cut_poly);

	/**
	* \brief create document
	*/
	void createDoc(std::vector<std::vector<cv::Point3f>> &pts);
	/**
	* \brief update document
	*/
	void updateDoc(std::vector<std::vector<cv::Point3f>> &pts);
	void updateDocRANSAC(std::vector<std::vector<cv::Point3f>> &pts);
	/**
	* \brief classify planes
	*/
	void filterPlane();
	/**
	* \brief calibration
	*/
	void calibration(std::vector<std::vector<cv::Point3f>> &pts);
	/**
	* \brief inverse calibration
	*/
	void inverseClbr(PlaneCutResultInterface& cutResult);
	/**
	* \brief locate region of interest
	*/
	void locROI(const TYPE_ROI type_roi, PlaneCutResultInterface& cutResult, int station_size, bool cut_poly);
	/**
	* \brief post-processing, update center and check if plane valid
	*/
	void postProc(
		const std::vector<std::vector<cv::Point3f>> &pts,
		//std::vector<cv::Point3f> &centers,
		std::vector<bool> &isValid);
	void setWithInclinedPlane(bool withInclinedPlane)
	{
		m_withInclinedPlane = withInclinedPlane;
	};

	void fillPlaneAreaFromDocs(PlaneCutResultInterface& cutResult);
private:
	void fixReflect(PlaneCutResultInterface& cutResult);


public:
	Doc_BIM m_doc_bim;	/**< document of BIM */
	struct LocROISnapshot {
		PlaneCutResultInterface cutResult;
		cv::Mat img_roi;

		std::vector<State_BIM_Plane> plane_states;
	};


private:
	bool m_isSuccess = true;
	bool m_withInclinedPlane = true;
	//cv::Mat mat_rot;
	cv::Mat img_roi;
};

#endif // !_SCRIPT_ROI_H_
