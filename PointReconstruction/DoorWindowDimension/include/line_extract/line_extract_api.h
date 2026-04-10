#ifndef _LINE_EXTRACT_API_H_
#define _LINE_EXTRACT_API_H_

#include "common_struct.h"
#include "pxl_struct_inf.h"

#if defined WIN32 || defined _WIN32 || defined __CYGWIN__
#ifdef LINE_EXTRACT_MODULE
#define LINE_EXTRACT_EXPORT __declspec(dllexport)
#else
#define LINE_EXTRACT_EXPORT __declspec(dllimport)
#endif
#else
#define LINE_EXTRACT_EXPORT
#endif

/**
* \brief  Interface of line extration , create instance of line extraction
*/
LINE_EXTRACT_EXPORT void* Features_LineExtractCreateHandle();

/**
* \brief  Interface of line extration, distroy instance of line extraction
* @param [in]  handle, line extraction handle
*/
LINE_EXTRACT_EXPORT void Features_LineExtractDestroyHandle(void** handle);


/**
* \brief  Interface of Output of line segmentation  extraction in 2D space with default configure parameters
* @param [in]  handle, line extraction handle
* @param [in]  input_data, input data in 2D space
* @param [out] line_seg, pointer of line extraction output in 2D space
* @return bool operation success or not
*/
LINE_EXTRACT_EXPORT bool Features_GetLinePoints2D(void* handle, const ModuleStruct::Point2fArray&input_data, ModuleStruct::Vector<Line2DSegOutItem> &line_seg);


/**
* \brief  Interface of Output of line segmentation  in 2D space with  configure file
* @param [in]  handle, line extraction handle
* @param [in]  config_file, input config file
* @param [in]  input_data, input data in 2D space
* @param [out] line_seg, pointer of line extraction output in 2D space
* @return bool operation success or not
*/
LINE_EXTRACT_EXPORT bool Features_GetLinePoints2D(void* handle, const std::string config_file, const ModuleStruct::Point2fArray& input_data, ModuleStruct::Vector<Line2DSegOutItem>& line_seg);


/**
* \brief  Interface of Output of line segmentation  in 2D space with  configure file and debug info
* @param [in]  handle, line extraction handle
* @param [in]  config_file, input config file
* @param [in]  input_data, input data in 2D space
* @param [out] line_seg, pointer of line extraction output in 2D space
* @return bool operation success or not
*/
LINE_EXTRACT_EXPORT bool Features_GetLinePoints2D(void* handle, const std::string config_file, const ModuleStruct::Point2fArray& input_data, ModuleStruct::Vector<Line2DSegOutItemDebug>& line_seg);


/**
* \brief  Interface of Output of line segmentation  in 2D space with  configure file and debug info
* @param [in]  handle, line extraction handle
* @param [in]  input_data, input data in 2D space
* @param [out] line_seg, pointer of line extraction output in 2D space
* @return bool operation success or not
*/
LINE_EXTRACT_EXPORT bool Features_GetLinePoints2D(void* handle, const ModuleStruct::Point2fArray&input_data, ModuleStruct::Vector<Line2DSegOutItemDebug>& line_seg);




/**
* \brief  Interface of config setting for line extraction in 2D space
* @param [in]  handle, line extraction handle
* @param [in]  sys_control_para, system control para for line config
* @param [in]  line_config_params, line extraction configure parametes
* @param [in]  line_seg_thrshld, line segmentation threshold
* @param [in]  PixelSize, pixel size of line segmentation
*/
LINE_EXTRACT_EXPORT void Features_SetLineConfig(void* handle, const SysCntrlParams sys_control_para, const LineExtrCntrlParams line_config_params, \
	const LineFitThresholds line_seg_thrshld, const PixelSize pixel_size);


/**
* \brief  Interface of debug setting for line extraction in 2D space
* @param [in]  handle, line extraction handle
* @param [in]  output_path, out put path
*/
LINE_EXTRACT_EXPORT bool Features_SetLineDbgPath(void* handle, const std::string output_path);



/**
* \brief  Interface of config setting for line extraction in 2D space
* @param [in]  handle, line extraction handle
* @param [in]  sys_control_para, system control para for line config
* @param [in]  line_config_params, line extraction configure parametes
* @param [in]  line_seg_thrshld, line segmentation threshold
* @param [in]  PixelSize, pixel size of line segmentation
* @param [in]  line_dbg_para, line extraction debug parameters
*/
LINE_EXTRACT_EXPORT void Features_SetLineConfig(void* handle, const SysCntrlParams sys_control_para, const LineExtrCntrlParams line_config_params, \
	const LineFitThresholds line_seg_thrshld, const PixelSize pixel_size, const Line2DDebugParams line_dbg_para);


#endif // _LINE_EXTRACT_API_H_
