#include "NormalEstimationAPI.hpp"
#include "normalEstimation.h"

using namespace NormalCoreSpace;

//static NormalEstimation* ns_Handle = nullptr;
//
//static inline NormalEstimation* GetInstanse()
//{
//	return ns_Handle ? ns_Handle : (ns_Handle = new NormalEstimation());
//}

void* Features_NormalsCreateHandle()
{
	return (new NormalEstimation());
}

void Features_NormalsDestroyHandle(void** handle)
{
	if (handle)
	{
		NormalEstimation* ptr = reinterpret_cast<NormalEstimation*>(*handle);
		delete ptr;
		ptr = nullptr;
	}
}

void Features_NormalsSetParams(void* handle, NormalEstimationParamters* params)
{
	NormalEstimation* ptr = reinterpret_cast<NormalEstimation*>(handle);
	ptr->SetParamsHandle(params);
}

NormalEstimationResults Features_NormalsCalculate(void* handle, bool isDetected)
{
	//NormalEstimation instance;
	START_TIME_COUNT(t)
	NormalEstimation* ptr = reinterpret_cast<NormalEstimation*>(handle);
	NormalEstimationResults res;
	ptr->Run(res, isDetected);
	END_TIME_COUNT("OUT run time: ", t)
	return res;
}

NormalEstimationResults Features_NormalsCalculate(void* handle, bool isDetected, interim_value_type density)
{
	//NormalEstimation instance;
	NormalEstimation* ptr = reinterpret_cast<NormalEstimation*>(handle);
	ptr->SetDataDensity(density);
	NormalEstimationResults res;
	ptr->Run(res, isDetected);
	return res;
}

void Features_GetOutputInfo(void* handle, NormalEstimationCtrParam& info)
{
	interim_value_type density;
	NormalEstimation* ptr = reinterpret_cast<NormalEstimation*>(handle);
	info.is_point_density_set = ptr->GetDataDensity(density);
	if (info.is_point_density_set)
		info.point_density = (value_type)density;
}

void Features_EnableBadPointsInfo(void* handle)
{
	NormalEstimation* ptr = reinterpret_cast<NormalEstimation*>(handle);
	ptr->EnableBadPointsOutput();
}

void Features_GetBadPointsInfo(void* handle, NormalEstimationBadPoints& info)
{
	NormalEstimation* ptr = reinterpret_cast<NormalEstimation*>(handle);
	info = ptr->GetBadPointsInfo();
}
