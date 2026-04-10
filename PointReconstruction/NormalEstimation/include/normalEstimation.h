#ifndef NORMAL_ESTIMATION_H
#define NORMAL_ESTIMATION_H

#include "ConstantDefines.h"
#include "VoxelNormals.h"
#include <thread>
#include "SafeQueue.h"
#include "log.h"

//#define TBB_QUEUE_SYNC_USING
#ifdef TBB_QUEUE_SYNC_USING
#include "TBBConcurrentQueue.h"
#include "tbb\concurrent_vector.h"
#else
#include "MoodycamelQueue.h"
#endif
//#define MULTI_THREADS_PROCESS
//#define TIME_ELAPSED_COUNT
#ifdef TIME_ELAPSED_COUNT
#define START_TIME_COUNT(t) clock_t t = clock();
#define END_TIME_COUNT(str, t) log_info("%s %f second", str, float(clock() - t) / CLOCKS_PER_SEC);
#else
#define START_TIME_COUNT(t)
#define END_TIME_COUNT(str, t)
#endif

namespace NormalCoreSpace {
	enum VOXEL_THREAD_TYPE
	{
		IDLE_VOXEL_THREAD,
		GOOD_VOXEL_THREAD,
		BAD_VOXEL_THREAD
	};

	typedef struct normalVoxelmodel
	{
		bool isOccupied;
		bool isGood; // good mark, maybe not real good if normals angle difference
		std::vector<ns_uint32> points_index;
		VoxelNormals voxel_normal;
		std::vector<normalVoxelmodel*> neighbours; // all neighbours of voxel_normal

		normalVoxelmodel()
		{
			isOccupied = false;
			isGood = false;
			points_index.clear();
			neighbours.clear();
		}
	} normalVoxelmodel;

	class NormalEstimation
	{
	public:
		NormalEstimation();
		~NormalEstimation();

		bool Run(NormalEstimationResults& output_data, bool isDetected = false);

		//set cloud points density
		void SetDataDensity(interim_value_type density);
		//get cloud points density
		bool GetDataDensity(interim_value_type& density);

		//save bad points information
		void EnableBadPointsOutput();
		//get bad points information
		NormalEstimationBadPoints GetBadPointsInfo();

		//get voxel index from queue
		void GetIndexofVoxelFromQueue(VOXEL_THREAD_TYPE type);
		//voxels grid core function for multi-thread
		void VoxelGridInitializeThreadCore(ns_uint32 start, ns_uint32 end);

		//paramter input
		//voxel size set
		//void SetDefaultVoxelSize(int size);
		//// points set pointer
		//void SetPointCloudData(Point3Array* data);
		////threshold setting
		//void SetMaxAngleOfNormal(value_type value);
		//void SetMinPointsOfVoxel(value_type value);
		//void SetMinMseOfVoxel(value_type value);
		void SetParamsHandle(NormalEstimationParamters* handle);
	private:
		void _Init();
		void _Reset();
		void _ThresholdSetting(interim_value_type voxel_size_x, interim_value_type voxel_size_y, interim_value_type voxel_size_z);
		bool _GetMinMaxXYZ(const Vector<Point3f>& input_data);
		bool _AllocVoxelGrid();
		void _VoxelGridInitialize();
		void _VoxelGridInitializeThread();
		ns_uint32 _ConvertPointToVoxelID(const interim_value_type x, const interim_value_type y, const interim_value_type z);
		void _SaveOutput();

		// all points of voxel was normal of plane if good
		bool _VoxelPlaneCalculation(normalVoxelmodel& voxel);
		//calculate neighbours of this voxel
		void _AddNeighborToVoxel(const ns_uint32 voxel_index);
		//mark occupied voxel, get normals for less points and bad mse voxels
		void _OccupiedVoxelNeighbourCalculation();
		//check all good voxel if real good
		void _GoodVoxelVerified();
		//check all good voxel if real good core function
		void _GoodVoxelCore(ns_uint32 index);
		//get normals for real good and bad angle voxels
		void _GoodVoxelClassified(const ns_uint32 voxel_index);
		// the condition of good/bad normal
		bool _NeighbourVoxelComparison(const ns_uint32 voxel_index);
		// points of voxel get normal by p2p if bad
		void _VoxelPointByPointCalculation(int index);

		void _StartThread(VOXEL_THREAD_TYPE type);
		void _EndThread(VOXEL_THREAD_TYPE type);

		//detect voxel size
		interim_value_type _VoxelSizeDetected();
		//detect minimum radius for normals calculation from point to point
		void _MiniRadiusDetected();
		ns_uint32 _FindNearCenterPoint(const std::vector<ns_uint32>& voxel_indexs);

	private:
		interim_value_type length_x_voxel, length_y_voxel, length_z_voxel;
		interim_value_type min_x, min_y, min_z, max_x, max_y, max_z;
		ns_uint32 cols_of_voxel, rows_of_voxel, depths_of_voxel;
		interim_value_type range_search_radius, min_p2p_mse, bad_voxel_mse;
		ns_uint32 numbers_voxel_grid;
		interim_value_type voxel_angle_threshold;
		interim_value_type input_density, mini_p2p_radius;
		bool isDensitySet;

		NormalEstimationParamters* ns_input_data;
		NormalEstimationResults ns_output_data;
		ns_uint32 voxel_array_size;
		std::vector<normalVoxelmodel> all_voxel_items;

		//26 neighbours index
		int movement_x[VOXEL_NEIGHBOUR_SIZE] = { -1, -1, -1,  1,  1,  1, 0, 0,  0,  0,  0,  0, 0,  0, -1, -1, -1, -1,  -1, -1, 1, 1,  1,  1,  1,  1 };
		int movement_y[VOXEL_NEIGHBOUR_SIZE] = { 0,  0,  0,  0,  0,  0, 1, 1,  1, -1, -1, -1, 0,  0,  1,  1,  1, -1,  -1, -1, 1, 1,  1, -1, -1, -1 };
		int movement_z[VOXEL_NEIGHBOUR_SIZE] = { 0,  1, -1,  0,  1, -1, 0, 1, -1,  0,  1, -1, 1, -1,  0,  1, -1,  0,   1, -1, 0, 1, -1,  0,  1, -1 };

		//multi-thread
		std::vector<std::thread> bad_voxel_thread_list, good_voxel_thread_list;

#ifdef TBB_QUEUE_SYNC_USING
		TBBConcurrentQueue<int> safeBadVoxelQueue, safeGoodVoxelQueue;
#else
		SafeQueue<int> safeBadVoxelQueue, safeGoodVoxelQueue;
		//MoodyCamelQueue<int> safeBadVoxelQueue, safeGoodVoxelQueue;
#endif

		mutable std::mutex grid_mutex;
		std::vector<std::thread> voxel_grid_thread;

		NormalEstimationBadPoints save_badpoints_info;
		bool saveBadPointsInfo;
	};
}

#endif
