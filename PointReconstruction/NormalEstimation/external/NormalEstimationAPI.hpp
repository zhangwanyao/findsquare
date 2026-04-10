#ifndef _NORMAL_ESTIMATION_INTERFACE_
#define _NORMAL_ESTIMATION_INTERFACE_
#include "NormalStruct.hpp"

#define NORMAL_ESTIMATION_MODULE

#ifdef WIN32
#ifdef NORMAL_ESTIMATION_MODULE
#define NORMALS_EXPORT __declspec(dllexport)
#else
#define NORMALS_EXPORT __declspec(dllimport)
#endif
#else
#define NORMALS_EXPORT
#endif

/*! \brief : create handle
*	\return the normalEstimation pointer
*/
NORMALS_EXPORT void* Features_NormalsCreateHandle();

/*! \brief : release handle
*	\param handle : normalEstimation pointer
*	\param handle is normalEstimation pointer
*/
NORMALS_EXPORT void Features_NormalsDestroyHandle(void** handle);

/*! \brief : parameters setting
*	\param params : normalEstimation parameters
*/
NORMALS_EXPORT void Features_NormalsSetParams(void* handle, NormalEstimationParamters* params);

/*! \brief : set the density of input data
*	\param handle : normalEstimation pointer
*	\param input_data : input structure
*	\param isDetected : voxel size estimated
*	\param density : point to pont distance
*/
NORMALS_EXPORT NormalEstimationResults Features_NormalsCalculate(void* handle, bool isDetected, interim_value_type density);

/*! \brief : get result after normal estimation
*	\param handle : normalEstimation pointer
*	\param input_data : input structure
*	\param isDetected : voxel size estimated
*/
NORMALS_EXPORT NormalEstimationResults Features_NormalsCalculate(void* handle, bool isDetected);

/*! \brief : output information for other module
*	\param handle : normalEstimation pointer
*	\param input_data : input structure
*/
NORMALS_EXPORT void Features_GetOutputInfo(void* handle, NormalEstimationCtrParam& info);

/*! \brief : enable bad points information
*	\param handle : normalEstimation pointer
*/
NORMALS_EXPORT void Features_EnableBadPointsInfo(void* handle);

/*! \brief : output bad points information for other module
*	\param handle : normalEstimation pointer
*	\param input_data : input structure
*/
NORMALS_EXPORT void Features_GetBadPointsInfo(void* handle, NormalEstimationBadPoints& info);

#endif // !NORMAL_ESTIMATION_INTERFACE