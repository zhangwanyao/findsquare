// normalEstimation.cpp : start。
//
#include "normalEstimation.h"
#include "util_math.hpp"
#include <fstream> 
#ifdef	WIN32
#define _USE_MATH_DEFINES
#include <math.h>
#else
#include "util_time.hpp"
#endif
//#define CPP_STANDARD_THREADS_USING
//#define OMP_THREADS_USING
#define TBB_THREADS_PARALLEL_USING
#ifdef OMP_THREADS_USING
#include <omp.h>
#endif
#ifdef TBB_THREADS_PARALLEL_USING
#include <tbb/parallel_for.h>
#endif

//#define	FAST_EIGEN_PROCESS
//#define SAVE_LOG_DEBUG
//#define NORMALS_COMPUTE_DEBUG
//#define	NORMALS_RANGE_AS_CUBE
//#define	DOWNSAMPLE_PROCESS
#ifdef DOWNSAMPLE_PROCESS
#include "DownSample.h"
#endif


#ifdef NORMALS_COMPUTE_DEBUG
ns_uint32 plane_voxel_size = 0;
ns_uint32 normal_point_less_3 = 0;
ns_uint32 p2p_voxel_size = 0;
ns_uint32 exceed_mse_voxel_size = 0;
ns_uint32 points_radius_decrease = 0;
ns_uint32 points_radius_decrease_min_points = 0;
char* badpoints_of_less_min_points = "badpoints_of_less_min_points.txt";
char* badpoints_of_mse = "badpoints_of_mse.txt";
char* less5_point = "less5_point.txt";
char* badpoints_of_bad_angle = "badpoints_of_bad_angle.ply";
ns_uint32 catch_voxel_id = 0;
interim_value_type watch_point_x[] = { 0, 0 };
interim_value_type watch_point_y[] = { 0, 0 };
interim_value_type watch_point_z[] = { 0, 0 };
#endif


void CalcVoxelGridThread(NormalCoreSpace::NormalEstimation* pointer, ns_uint32 start, ns_uint32 end)
{
	pointer->VoxelGridInitializeThreadCore(start, end);
}

void VoxelNormalThread(NormalCoreSpace::NormalEstimation* pointer, NormalCoreSpace::VOXEL_THREAD_TYPE type)
{
	pointer->GetIndexofVoxelFromQueue(type);
}

NormalCoreSpace::NormalEstimation::NormalEstimation()
{

	ns_input_data = nullptr;
	saveBadPointsInfo = false;
	isDensitySet = false;
}

NormalCoreSpace::NormalEstimation::~NormalEstimation()
{

}

void NormalCoreSpace::NormalEstimation::_Init()
{
	length_x_voxel = length_y_voxel = length_z_voxel = 0;
	min_x = min_y = min_z = max_x = max_y = max_z = 0;
	cols_of_voxel = rows_of_voxel = depths_of_voxel = 0;
	range_search_radius = min_p2p_mse = bad_voxel_mse = 0;
	numbers_voxel_grid = 0;
	voxel_angle_threshold = 0;
	voxel_array_size = 0;
	mini_p2p_radius = 0;
	all_voxel_items.clear();
	save_badpoints_info.badPoint_map.clear();
	ns_output_data.normals.resize(ns_input_data->points->size());
}

void NormalCoreSpace::NormalEstimation::_Reset()
{
	isDensitySet = false;
	input_density = 0;
}

void NormalCoreSpace::NormalEstimation::SetParamsHandle(NormalEstimationParamters* handle)
{
	ns_input_data = handle;
}

void NormalCoreSpace::NormalEstimation::EnableBadPointsOutput()
{
	saveBadPointsInfo = true;
}

NormalEstimationBadPoints NormalCoreSpace::NormalEstimation::GetBadPointsInfo()
{
	return save_badpoints_info;
}

interim_value_type NormalCoreSpace::NormalEstimation::_VoxelSizeDetected()
{
	interim_value_type x_length = max_x - min_x;
	interim_value_type y_length = max_y - min_y;
	interim_value_type z_length = max_z - min_z;

	interim_value_type min_length = std::min(std::min(x_length, y_length), z_length);
	interim_value_type max_length = std::max(std::max(x_length, y_length), z_length);

	//initialize the mini voxel size and max voxel size
	interim_value_type voxel_size = min_length / DETECT_VOXEL_SIZE_RATIO;
	interim_value_type max_voxel_size = max_length / DETECT_VOXEL_SIZE_RATIO * 1.5;

	while (true)
	{
		_ThresholdSetting(voxel_size, voxel_size, voxel_size);
		if (!_AllocVoxelGrid())
		{
			break;
		}
		std::vector<normalVoxelmodel> all_voxels(numbers_voxel_grid);
		// alloc points to each voxel
		for (ns_uint32 i = 0; i < ns_input_data->points->size(); i++)
		{
			ns_uint32 voxel_id = _ConvertPointToVoxelID(ns_input_data->points->at(i).x, ns_input_data->points->at(i).y, ns_input_data->points->at(i).z);
			all_voxels[voxel_id].points_index.push_back(i);
		}

		ns_uint32 valid_voxel_size = 0;
		ns_uint32 occupied_voxel_size = 0;
		for (auto &voxel : all_voxels)
		{
			//VOXEL_FITPLANE_MIN_POINTS
			if (voxel.points_index.size() >= OCCUPIED_VOXEL_POINTS)
			{
				if (voxel.points_index.size() < ns_input_data->threshold_min_point_num_of_valid_normal_voxel)
				{
					valid_voxel_size++;
				}
				occupied_voxel_size++;
			}
		}

		// find the ratio of voxels of points < 20 less than 5 percent
		if (valid_voxel_size < occupied_voxel_size / DETECT_VOXEL_VALID_SIZE_RATIO || voxel_size + (voxel_size / DETECT_VOXEL_SIZE_STEP_RATIO) > max_voxel_size)
		{
			break;
		}
		else
		{
			voxel_size += (voxel_size / DETECT_VOXEL_SIZE_STEP_RATIO);
		}
	}
	return voxel_size;
}

void NormalCoreSpace::NormalEstimation::_ThresholdSetting(interim_value_type voxel_size_x, interim_value_type voxel_size_y, interim_value_type voxel_size_z)
{
	ns_input_data->voxel_size_x = (value_type)(length_x_voxel = voxel_size_x);
	ns_input_data->voxel_size_y = (value_type)(length_y_voxel = voxel_size_y);
	ns_input_data->voxel_size_z = (value_type)(length_z_voxel = voxel_size_z);
	range_search_radius = ((length_x_voxel + length_y_voxel + length_z_voxel) / 3 / 2);
	ns_input_data->threshold_min_mse_of_voxel = (value_type)((length_x_voxel + length_y_voxel + length_z_voxel) / 3 / 8);
	bad_voxel_mse = ns_input_data->threshold_min_mse_of_voxel * 2 / 3;
	min_p2p_mse = ns_input_data->threshold_min_mse_of_voxel / 4;
	voxel_angle_threshold = std::cos(ns_input_data->threshold_max_normal_angle_of_two_voxel * M_PI / 180.f);
}

//get min ,max value for x, y, z
bool NormalCoreSpace::NormalEstimation::_GetMinMaxXYZ(const Vector<Point3f>& input_data)
{
	min_x = min_y = min_z = std::numeric_limits<interim_value_type>::infinity();
	max_x = max_y = max_z = -std::numeric_limits<interim_value_type>::infinity();
	for (ns_uint32 i = 0; i < input_data.size(); i++)
	{
		min_x = (min_x > input_data[i].x) ? input_data[i].x : min_x;
		max_x = (max_x < input_data[i].x) ? input_data[i].x : max_x;
		min_y = (min_y > input_data[i].y) ? input_data[i].y : min_y;
		max_y = (max_y < input_data[i].y) ? input_data[i].y : max_y;
		min_z = (min_z > input_data[i].z) ? input_data[i].z : min_z;
		max_z = (max_z < input_data[i].z) ? input_data[i].z : max_z;
	}

	return true;
}

bool NormalCoreSpace::NormalEstimation::_AllocVoxelGrid()
{
		cols_of_voxel = ns_uint32(std::floor((max_x - min_x) / length_x_voxel) + 1);

		rows_of_voxel = ns_uint32(std::floor((max_y - min_y) / length_y_voxel) + 1);

		depths_of_voxel = ns_uint32(std::floor((max_z - min_z) / length_z_voxel) + 1);

	numbers_voxel_grid = cols_of_voxel * rows_of_voxel * depths_of_voxel;

	if (numbers_voxel_grid <= 0) 
	{
		log_error("cannot form any voxel %ld, %ld, %ld", cols_of_voxel, rows_of_voxel, depths_of_voxel);
		return false;
	}
	return true;
}

// alloc total voxels, assign points to corresponding voxel
void NormalCoreSpace::NormalEstimation::_VoxelGridInitialize()
{
	all_voxel_items.clear();
	all_voxel_items.resize(numbers_voxel_grid);
	for (ns_uint32 i = 0; i < ns_input_data->points->size(); i++)
	{
		ns_uint32 voxel_id = _ConvertPointToVoxelID(ns_input_data->points->at(i).x, ns_input_data->points->at(i).y, ns_input_data->points->at(i).z);
#ifdef NORMALS_COMPUTE_DEBUG
		if (ns_input_data->points.points[i].point.x > watch_point_x[0] && ns_input_data->points.points[i].point.x < watch_point_x[1]
			&& ns_input_data->points.points[i].point.y > watch_point_y[0] && ns_input_data->points.points[i].point.y < watch_point_y[1]
			&& ns_input_data->points.points[i].point.z > watch_point_z[0] && ns_input_data->points.points[i].point.z < watch_point_z[1])
		{
			std::cout << "watch points voxel id: " << voxel_id << std::endl;
			catch_voxel_id = voxel_id;
		}
#endif
		all_voxel_items[voxel_id].points_index.push_back(i);
		if (!all_voxel_items[voxel_id].isOccupied && all_voxel_items[voxel_id].points_index.size() >= OCCUPIED_VOXEL_POINTS)
		{
			all_voxel_items[voxel_id].isOccupied = true;
			_AddNeighborToVoxel(voxel_id);
			voxel_array_size++;
		}
	}
}

void NormalCoreSpace::NormalEstimation::VoxelGridInitializeThreadCore(ns_uint32 start, ns_uint32 end)
{
	for (ns_uint32 i = start; i < end; i++)
	{
		ns_uint32 voxel_id = _ConvertPointToVoxelID(ns_input_data->points->at(i).x, ns_input_data->points->at(i).y, ns_input_data->points->at(i).z);
		{
			std::lock_guard<std::mutex> lock(grid_mutex);
			all_voxel_items[voxel_id].points_index.push_back(i);
			if (!all_voxel_items[voxel_id].isOccupied && all_voxel_items[voxel_id].points_index.size() >= OCCUPIED_VOXEL_POINTS)
			{
				all_voxel_items[voxel_id].isOccupied = true;
				_AddNeighborToVoxel(voxel_id);
				voxel_array_size++;
			}
		}
	}
}

void NormalCoreSpace::NormalEstimation::_VoxelGridInitializeThread()
{
	all_voxel_items.clear();
	all_voxel_items.resize(numbers_voxel_grid);
	ns_uint32 thread_count = std::thread::hardware_concurrency() - 10;
	voxel_grid_thread.resize(thread_count);

	size_t dotd = ns_input_data->points->size() % thread_count;
	size_t intd = ns_input_data->points->size() / thread_count;
	std::vector<ns_uint32> allocate_thread_voxel(thread_count);
	for (ns_uint32 j = 0; j < allocate_thread_voxel.size(); j++)
	{
		allocate_thread_voxel[j] = (dotd-- > 0) ? intd + 1 : intd;
	}

	for (ns_uint32 i = 0, vcount = 0; i < allocate_thread_voxel.size(); i++)
	{
		voxel_grid_thread[i] = std::thread(CalcVoxelGridThread, this, vcount, vcount + allocate_thread_voxel[i]);
		vcount += allocate_thread_voxel[i];
	}

	for (std::thread & th : voxel_grid_thread)
	{
		if (th.joinable())
			th.join();
	}
}

void NormalCoreSpace::NormalEstimation::_OccupiedVoxelNeighbourCalculation()
{
#ifdef NORMALS_COMPUTE_DEBUG
	std::ofstream less_min_points(badpoints_of_less_min_points, std::ios_base::app);
	std::ofstream bad_mse_points(badpoints_of_mse, std::ios_base::app);
#endif //  NORMALS_COMPUTE_DEBUG
	voxel_array_size = 0;
	for (ns_uint32 i = 0; i < all_voxel_items.size(); i++)
	{
		if (all_voxel_items[i].points_index.size() >= OCCUPIED_VOXEL_POINTS)
		{
			all_voxel_items[i].isOccupied = true;
			_AddNeighborToVoxel(i);
			voxel_array_size++;
		}
	}
#ifdef NORMALS_COMPUTE_DEBUG
	std::cout << "total occupied voxel size: " << ns_input_data->voxel_array_size << std::endl;
	std::cout << "voxel size of few points: " << p2p_voxel_size << "\nvoxel size of possible good: " << ns_input_data->voxel_array_size - exceed_mse_voxel_size - p2p_voxel_size 
		<< "\nvoxel size of mse exceed: " << exceed_mse_voxel_size << std::endl;
	less_min_points.close();
	bad_mse_points.close();
#endif // NORMALS_COMPUTE_DEBUG

}

void NormalCoreSpace::NormalEstimation::_EndThread(VOXEL_THREAD_TYPE type)
{
	if (type == BAD_VOXEL_THREAD)
	{
		safeBadVoxelQueue.SetState(THREAD_QUEUE_CLOSE);
		for (std::thread & th : bad_voxel_thread_list)
		{
			// If thread Object is Joinable then Join that thread.
			if (th.joinable())
				th.join();
		}
	}
	else if (type == GOOD_VOXEL_THREAD)
	{
		safeGoodVoxelQueue.SetState(THREAD_QUEUE_CLOSE);
		for (std::thread & th : good_voxel_thread_list)
		{
			// If thread Object is Joinable then Join that thread.
			if (th.joinable())
				th.join();
		}
	}
}

void NormalCoreSpace::NormalEstimation::_StartThread(NormalCoreSpace::VOXEL_THREAD_TYPE type)
{
	int thread_count = std::thread::hardware_concurrency() - 1;
	if (type == BAD_VOXEL_THREAD)
	{
		safeBadVoxelQueue.SetState(THREAD_QUEUE_START);
		bad_voxel_thread_list.resize(thread_count);
		for (int i = 0; i < thread_count; i++)
		{
			bad_voxel_thread_list[i] = std::thread(VoxelNormalThread, this, type);
		}
	}
	else if (type == GOOD_VOXEL_THREAD)
	{
		safeGoodVoxelQueue.SetState(THREAD_QUEUE_START);
		good_voxel_thread_list.resize(thread_count);
		for (int i = 0; i < thread_count; i++)
		{
			good_voxel_thread_list[i] = std::thread(VoxelNormalThread, this, type);
		}
	}


}

void NormalCoreSpace::NormalEstimation::GetIndexofVoxelFromQueue(NormalCoreSpace::VOXEL_THREAD_TYPE type)
{
	int index;
	//if (type == BAD_VOXEL_THREAD)
	//{
	//	while (safeBadVoxelQueue.Dequeue_nb(index))
	//	{
	//		VoxelPointByPointCalculation(index);
	//	}
	//}
	//else if (type == GOOD_VOXEL_THREAD)
	//{
	//	while (safeGoodVoxelQueue.Dequeue_nb(index))
	//	{
	//		_GoodVoxelCore(index);
	//	}
	//}
	while (safeGoodVoxelQueue.Dequeue(index))
	{
		_GoodVoxelCore(index);
	}
	while (safeBadVoxelQueue.Dequeue(index))
	{
		_VoxelPointByPointCalculation(index);
	}
}

void NormalCoreSpace::NormalEstimation::_VoxelPointByPointCalculation(int index)
{

#ifdef NORMALS_COMPUTE_DEBUG
	std::ofstream less5_points(less5_point, std::ios_base::app);
#endif //  NORMALS_COMPUTE_DEBUG

	const auto &all_points = *ns_input_data->points;
	auto& all_normals = ns_output_data.normals;

	// put self points to candinate
	std::vector<ns_uint32> candinate_points = all_voxel_items[index].points_index;

	//put neighbours points to candinate
	//ns_uint32 candinate_points_limited = std::floor((length_x_voxel * length_y_voxel * length_z_voxel) / (input_density * input_density * input_density) / 4);
	for (const auto &neighbour : all_voxel_items[index].neighbours)
	{
		candinate_points.insert(candinate_points.end(), neighbour->points_index.begin(), neighbour->points_index.end());
		//for (ns_uint32 j = 0; j < all_voxel_items[index].neighbours[i]->points_index.size(); j++)
		//{
		//	candinate_points.push_back(all_voxel_items[index].neighbours[i]->points_index[j]);
		//}
	}

#ifdef DOWNSAMPLE_PROCESS
	VoxelDownsample candidateDownsample(ns_input_data->voxel_para.length_x_of_voxel / DOWNSAMPLE_VOXEL_SIZE_RATIO,
		ns_input_data->points.points, candinate_points);
	candidateDownsample.Run();

	VoxelDownsample currentDownsample(ns_input_data->voxel_para.length_x_of_voxel / DOWNSAMPLE_VOXEL_SIZE_RATIO,
		ns_input_data->points.points, all_voxel_items[index].points_index);
	currentDownsample.Run();

	ns_uint32 currentPointsSize = currentDownsample.Voxels_Grid.size();
	ns_uint32 candidatePointsSize = candidateDownsample.Voxels_Grid.size();
	//std::cout << "sample size: " << currentPointsSize << "\t" << candidatePointsSize << std::endl;
#else
	ns_uint32 currentPointsSize = (ns_uint32)all_voxel_items[index].points_index.size();
	ns_uint32 candidatePointsSize = (ns_uint32)candinate_points.size();
#endif

#ifdef NORMALS_RANGE_AS_CUBE
	std::vector<Point3f> candinate_delta_point(candidatePointsSize);
	if (candinate_delta_point.capacity() < candidatePointsSize)
	{
		// std::cout << "+ candinate_delta_point(" << &candinate_delta_point << ") " << candinate_delta_point.capacity() << " -> " << candidatePointsSize << std::endl;
		candinate_delta_point.reserve(candidatePointsSize);
	}
#else
	Vector<interim_value_type> candinate_points_distance(candidatePointsSize);
	if (candinate_points_distance.capacity() < candidatePointsSize)
	{
		// std::cout << "+ candinate_points_distance(" << &candinate_points_distance << ") " << candinate_points_distance.capacity() << " -> " << candidatePointsSize << std::endl;
		candinate_points_distance.reserve(candidatePointsSize);
	}
#endif
	Vector<Point3f> radius_points(candidatePointsSize);
	if (radius_points.capacity() < candidatePointsSize)
	{
		// std::cout << "+ radius_points(" << &radius_points << ") " << radius_points.capacity() << " -> " << candidatePointsSize << std::endl;
		radius_points.reserve(candidatePointsSize);
	}
	Vector<ns_uint32> radius_points_index(candidatePointsSize);

	VoxelNormals points_normal;
	for (ns_uint32 i = 0; i < currentPointsSize; i++)
	{
		ns_uint32 current_point_index = all_voxel_items[index].points_index[i];
		bool radius_decrease_by_mse = false;
		//get points radius initialized
		interim_value_type points_radius = range_search_radius < mini_p2p_radius ? mini_p2p_radius : range_search_radius;
		interim_value_type radius_step = points_radius / 20;

		ns_uint32 k = 0;
		for (ns_uint32 j = 0; j < candidatePointsSize; j++)
		{
#ifdef DOWNSAMPLE_PROCESS
			Point3f ptsdelta = currentDownsample.Voxels_Grid[i].sample_point - candidateDownsample.Voxels_Grid[j].sample_point;
			candinate_points_distance[j] = std::fabs(ptsdelta.x) + std::fabs(ptsdelta.y) + std::fabs(ptsdelta.z);
			if (candinate_points_distance[j] < points_radius)
			{
				//radius_points.push_back(all_points[candinate_points[j]].point);
				radius_points[k++] = candidateDownsample.Voxels_Grid[j].sample_point;
			}
#else
#ifdef NORMALS_RANGE_AS_CUBE
			candinate_delta_point[j].x = std::fabs(all_points[candinate_points[j]].point.x - all_points[current_point_index].point.x);
			candinate_delta_point[j].y = std::fabs(all_points[candinate_points[j]].point.y - all_points[current_point_index].point.y);
			candinate_delta_point[j].z = std::fabs(all_points[candinate_points[j]].point.z - all_points[current_point_index].point.z);
			if ((candinate_delta_point[j].x <= points_radius) && (candinate_delta_point[j].y <= points_radius) && (candinate_delta_point[j].z <= points_radius))
			{
				radius_points.push_back(all_points[candinate_points[j]].point);
			}
#else
			//candinate_points_distance[j] = MathOperation::ComputePointToPointDist(all_points[voxel.points_index[i]].point, all_points[candinate_points[j]].point);
			//candinate_points_distance[j] = cv::norm(all_points[current_point_index] - all_points[candinate_points[j]]);
			Point3f ptsdelta = all_points[current_point_index] - all_points[candinate_points[j]];
			candinate_points_distance[j] = std::fabs(ptsdelta.x) + std::fabs(ptsdelta.y) + std::fabs(ptsdelta.z);
			if (candinate_points_distance[j] < points_radius)
			{
				//radius_points.push_back(all_points[candinate_points[j]].point);
				if (saveBadPointsInfo)
					radius_points_index[k] = candinate_points[j];
				radius_points[k++] = all_points[candinate_points[j]];
			}
#endif
#endif
		}
		//radius_points.resize(k);
		bool run_first = true;
		while (true)
		{
			if (run_first)
			{
				run_first = false;
			}
			else
			{
				k = 0;
				//get candinate points to fit plane
				for (ns_uint32 j = 0; j < candidatePointsSize; j++)
				{
#ifdef NORMALS_RANGE_AS_CUBE
					if((candinate_delta_point[j].x <= points_radius) && (candinate_delta_point[j].y <= points_radius) && (candinate_delta_point[j].z <= points_radius))
#else
					if (candinate_points_distance[j] <= points_radius)
#endif
					{
#ifdef DOWNSAMPLE_PROCESS
						radius_points[k++] = candidateDownsample.Voxels_Grid[j].sample_point;
#else
						//radius_points.push_back(all_points[candinate_points[j]].point);
						if (saveBadPointsInfo)
							radius_points_index[k] = candinate_points[j];
						radius_points[k++] = all_points[candinate_points[j]];
#endif
					}
				}
			}

			if (k < COMPUTE_NORMALS_MIN_POINTS)
			{
				if (radius_decrease_by_mse) // get last normals 
				{
#ifdef  NORMALS_COMPUTE_DEBUG
					points_radius_decrease_min_points++;
#endif //  NORMALS_COMPUTE_DEBUG
#ifdef DOWNSAMPLE_PROCESS
					currentDownsample.SaveNormals(points_normal.GetNormals(), i);
#else
					all_normals[current_point_index] = points_normal.GetNormals();
#endif
					break;
				}
				else if (k < candidatePointsSize) // add radius to calculate normals when normals didn't computed
				{
#ifdef  NORMALS_COMPUTE_DEBUG
					normal_point_less_3++;
					less5_points << all_points[voxel.points_index[i]].point.x << '\t'
						<< all_points[voxel.points_index[i]].point.y << '\t'
						<< all_points[voxel.points_index[i]].point.z << std::endl;
#endif //  NORMALS_COMPUTE_DEBUG
					points_radius += radius_step;
					continue;
				}
				else
				{
					log_warn("No enough points(%d) to get normals", candidatePointsSize);
				}
			}

			// compute normal
			points_normal.Init(k, PLANE_NORMALS_CENTER_ARRAY);
			//points_normal.Push(radius_points);
			for (ns_uint32 t = 0; t < k; t++)
			{
				points_normal.Push(radius_points[t], t);
			}
#ifdef FAST_EIGEN_PROCESS
			voxel.points_normal.FastProcess();
#else
			points_normal.cvProcess();
#endif // FAST_EIGEN_PROCESS


			if (saveBadPointsInfo)
			{
				std::vector<ns_uint32> temp = radius_points_index;
				temp.resize(k);
				std::lock_guard<std::mutex> lock(grid_mutex);
				save_badpoints_info.badPoint_map[current_point_index] = temp;
			}

			if (points_normal.GetWeightedMse() < min_p2p_mse || points_radius - radius_step < mini_p2p_radius)
			{
#ifdef  NORMALS_COMPUTE_DEBUG
				if(radius_decrease_by_mse) points_radius_decrease++;
#endif //  NORMALS_COMPUTE_DEBUG
#ifdef DOWNSAMPLE_PROCESS
				currentDownsample.SaveNormals(points_normal.GetNormals(), i);
#else
				//set normal for point of i
				all_normals[current_point_index] = points_normal.GetNormals();
#endif
				break;
			}
			else
			{
				//dichotomy
				points_radius = (points_radius + mini_p2p_radius) / 2;
				//progressively smaller
				//points_radius -= radius_step
				radius_decrease_by_mse = true;
				//std::cout << "decrease radius to adapte mse" << std::endl;
			}
		}
	}

#ifdef NORMALS_COMPUTE_DEBUG
	less5_points.close();
#endif //  NORMALS_COMPUTE_DEBUG
}

bool NormalCoreSpace::NormalEstimation::_VoxelPlaneCalculation(normalVoxelmodel& voxel)
{
	voxel.voxel_normal.Init((ns_uint32)voxel.points_index.size(), PLANE_NORMALS_CENTER_ARRAY);
	for (ns_uint32 i = 0; i < voxel.points_index.size(); i++)
	{
		voxel.voxel_normal.Push(ns_input_data->points->at(voxel.points_index[i]), i);
	}
#ifdef FAST_EIGEN_PROCESS
	voxel.voxel_normal.FastProcess();
#else
	voxel.voxel_normal.cvProcess();
#endif // FAST_EIGEN_PROCESS

	if (voxel.voxel_normal.GetMse() < min_p2p_mse)
	{
		voxel.isGood = true;
	}
	return voxel.isGood;
}

// neighbours normals compare with this normal
bool NormalCoreSpace::NormalEstimation::_NeighbourVoxelComparison(const ns_uint32 voxel_index)
{
#ifdef SAVE_LOG_DEBUG
	ns_uint32 bad_neighbor_size = 0;
	ns_uint32 point_few_size = 0;
#endif // SAVE_LOG_DEBUG


	std::vector<ns_uint32> candinatepoints = all_voxel_items[voxel_index].points_index;
	for (const auto &neighbour : all_voxel_items[voxel_index].neighbours)
	{
		if (neighbour->isOccupied)
		{
			candinatepoints.insert(candinatepoints.end(), neighbour->points_index.begin(), neighbour->points_index.end());
		}
	}
	
	VoxelNormals normalinstance;
	normalinstance.Init((ns_uint32)candinatepoints.size(), PLANE_NORMALS_CENTER_ARRAY);
	for (ns_uint32 i = 0; i < candinatepoints.size(); i++)
	{
		normalinstance.Push(ns_input_data->points->at(candinatepoints[i]), i);
	}

#ifdef FAST_EIGEN_PROCESS
	normalinstance.FastProcess();
#else
	normalinstance.cvProcess();
#endif

	//float normal_diff = std::fabs(MathOperation::ComputeVectorDotProduct(all_voxel_items[voxel_index].voxel_normal.GetNormals(), normalinstance.GetNormals()));
	if (all_voxel_items[voxel_index].voxel_normal.GetNormals().dot(normalinstance.GetNormals()) < voxel_angle_threshold || normalinstance.GetWeightedMse() > bad_voxel_mse) // cos value bigger , angle smaller
	{
		return false;
	}
#ifdef SAVE_LOG_DEBUG
	log_info("good voxel id: %u, bad_side: %u, normal_diff: %u, few_points:%u", voxel_index, bad_neighbor_size, normals_invalid - bad_neighbor_size, point_few_size);
#endif // SAVE_LOG_DEBUG

	return true;
}

void NormalCoreSpace::NormalEstimation::_GoodVoxelClassified(const ns_uint32 voxel_index)
{
#ifdef NORMALS_COMPUTE_DEBUG
	std::ofstream bad_angle(badpoints_of_bad_angle, std::ios_base::app);
#endif //  NORMALS_COMPUTE_DEBUG
	if (_NeighbourVoxelComparison(voxel_index))
	{
		// real good voxel
		for (int i = 0; i < all_voxel_items[voxel_index].points_index.size(); i++)
		{
			ns_output_data.normals[all_voxel_items[voxel_index].points_index[i]] = all_voxel_items[voxel_index].voxel_normal.GetNormals();
		}
#ifdef NORMALS_COMPUTE_DEBUG
		plane_voxel_size++;
#endif
	}
	else
	{
		all_voxel_items[voxel_index].isGood = false;
#ifdef MULTI_THREADS_PROCESS
		safeBadVoxelQueue.Enqueue(voxel_index);
#else
		_VoxelPointByPointCalculation(voxel_index);
#endif
#ifdef NORMALS_COMPUTE_DEBUG
		for (int i = 0; i < all_voxel_items[voxel_index].points_index.size(); i++)
		{
			bad_angle << ns_input_data->points.points[all_voxel_items[voxel_index].points_index[i]].point.x << ' '
				<< ns_input_data->points.points[all_voxel_items[voxel_index].points_index[i]].point.y << ' '
				<< ns_input_data->points.points[all_voxel_items[voxel_index].points_index[i]].point.z << ' ' 
				<< ns_input_data->points.points[all_voxel_items[voxel_index].points_index[i]].normal.x << ' '
				<< ns_input_data->points.points[all_voxel_items[voxel_index].points_index[i]].normal.y << ' '
				<< ns_input_data->points.points[all_voxel_items[voxel_index].points_index[i]].normal.z << ' '
				<< std::endl;
		}
#endif
	}
#ifdef NORMALS_COMPUTE_DEBUG
	bad_angle.close();
#endif
}

void NormalCoreSpace::NormalEstimation::_GoodVoxelCore(ns_uint32 index)
{
	if (all_voxel_items[index].points_index.size() < ns_input_data->threshold_min_point_num_of_valid_normal_voxel)
	{
		//ns_input_data->voxel_array[j].is_good_voxel = false;
#ifdef MULTI_THREADS_PROCESS
		safeBadVoxelQueue.Enqueue(index);
#else
		_VoxelPointByPointCalculation(index);
#endif
#ifdef  NORMALS_COMPUTE_DEBUG
		p2p_voxel_size++;
		for (ns_uint32 j = 0; j < all_voxel_items[i].points_index.size(); j++)
		{
			less_min_points << ns_input_data->points.points[all_voxel_items[i].points_index[j]].point.x << '\t'
				<< ns_input_data->points.points[all_voxel_items[i].points_index[j]].point.y << '\t'
				<< ns_input_data->points.points[all_voxel_items[i].points_index[j]].point.z << std::endl;
		}
#endif //  NORMALS_COMPUTE_DEBUG
	}
	else
	{
		//get mse is bad or good
		if (!_VoxelPlaneCalculation(all_voxel_items[index]))
		{
			// calculate the normals of mse > threshold
#ifdef MULTI_THREADS_PROCESS
			safeBadVoxelQueue.Enqueue(index);
#else
			_VoxelPointByPointCalculation(index);
#endif
#ifdef  NORMALS_COMPUTE_DEBUG
			exceed_mse_voxel_size++;
			for (ns_uint32 j = 0; j < all_voxel_items[i].points_index.size(); j++)
			{
				bad_mse_points << ns_input_data->points.points[all_voxel_items[i].points_index[j]].point.x << '\t'
					<< ns_input_data->points.points[all_voxel_items[i].points_index[j]].point.y << '\t'
					<< ns_input_data->points.points[all_voxel_items[i].points_index[j]].point.z << std::endl;
			}
#endif //  NORMALS_COMPUTE_DEBUG
		}
		else
		{
			_GoodVoxelClassified(index);
		}
	}
}

void NormalCoreSpace::NormalEstimation::_GoodVoxelVerified()
{
	for (ns_uint32 i = 0; i < all_voxel_items.size(); i++)
	{
		if (all_voxel_items[i].isOccupied)
		{
#ifdef MULTI_THREADS_PROCESS
			safeGoodVoxelQueue.Enqueue(i);
#else
			_GoodVoxelCore(i);
#endif
		}
	}
}

void NormalCoreSpace::NormalEstimation::_SaveOutput()
{
	//put VoxelItem grid_idx & point_idx, bad voxel if < VOXEL_FITPLANE_MIN_POINTS
	ns_output_data.voxel_array.resize(voxel_array_size);
	ns_output_data.grid_to_occupied_voxel_idx.resize((ns_uint32)all_voxel_items.size());
	ns_output_data.voxel_idx.resize(ns_input_data->points->size());

	for (ns_uint32 i = 0, j = 0; i < all_voxel_items.size(); i++)
	{
		if (all_voxel_items[i].isOccupied)
		{
			ns_output_data.voxel_array[j].is_good_voxel = all_voxel_items[i].isGood;
			ns_output_data.voxel_array[j].grid_idx = i;
			ns_output_data.voxel_array[j].points_index.resize((ns_uint32)all_voxel_items[i].points_index.size());
			for (ns_uint32 k = 0; k < ns_output_data.voxel_array[j].points_index.size(); k++)
			{
				ns_output_data.voxel_array[j].points_index[k] = all_voxel_items[i].points_index[k];
				ns_output_data.voxel_idx[ns_output_data.voxel_array[j].points_index[k]] = j;
			}
			ns_output_data.grid_to_occupied_voxel_idx[i] = j++;
		}
		else
		{
			ns_output_data.grid_to_occupied_voxel_idx[i] = std::numeric_limits<unsigned int>::max();
		}
	}
#ifdef NORMALS_COMPUTE_DEBUG
	std::cout << "voxel size of real good: " << plane_voxel_size << std::endl;
	std::cout << "voxel size of angle bad: " << ns_input_data->voxel_array_size - exceed_mse_voxel_size - p2p_voxel_size - plane_voxel_size << std::endl;
	std::cout << "voxel size of normal points < 5: " << normal_point_less_3 << std::endl;
	std::cout << "number of decrease radius to compute normals: " << points_radius_decrease + points_radius_decrease_min_points << "\nmin points of radius iteration: " << points_radius_decrease_min_points << std::endl;
	
#endif // NORMALS_COMPUTE_DEBUG

	// put PointItem voxel_idx
	//for (ns_uint32 j = 0; j < ns_input_data->voxel_array_size; j++)
	//{
	//	for (ns_uint32 k = 0; k < ns_input_data->voxel_array[j].points.size; k++)
	//	{
	//		ns_input_data->points.points[ns_input_data->voxel_array[j].points.point_idx[k]].voxel_idx = j;
	//	}
	//}
}

// find neighbours
void NormalCoreSpace::NormalEstimation::_AddNeighborToVoxel(const ns_uint32 voxel_index)
{
	//if (voxel_grid[current_voxel_rid]->num_of_pt <= 0)
	//	return;

	//get column, row and height index
	int idx = all_voxel_items[voxel_index].points_index[0];
	int col_idx = (int)(std::floor((ns_input_data->points->at(idx).x - min_x) / length_x_voxel));
	int row_idx = (int)(std::floor((ns_input_data->points->at(idx).y - min_y) / length_y_voxel));
	int height_idx = (int)(std::floor((ns_input_data->points->at(idx).z - min_z) / length_z_voxel));

	//get neighbors
	for (int i = 0; i < VOXEL_NEIGHBOUR_SIZE; i++)
	{
		if (height_idx + movement_z[i] >= (int)depths_of_voxel || height_idx + movement_z[i] < 0)
			continue;
		if (row_idx + movement_y[i] >= (int)rows_of_voxel || row_idx + movement_y[i] < 0)
			continue;
		if (col_idx + movement_x[i] >= (int)cols_of_voxel || col_idx + movement_x[i] < 0)
			continue;

		int neighbor_voxel_rid = (int)((height_idx + movement_z[i])*(cols_of_voxel*rows_of_voxel) + (row_idx + movement_y[i])*cols_of_voxel + (col_idx + movement_x[i]));

		if (neighbor_voxel_rid < 0 || neighbor_voxel_rid >= (int)numbers_voxel_grid) continue;

		all_voxel_items[voxel_index].neighbours.push_back(&all_voxel_items[neighbor_voxel_rid]);
	}
}

ns_uint32 NormalCoreSpace::NormalEstimation::_ConvertPointToVoxelID(const interim_value_type x, const interim_value_type y, const interim_value_type z)
{

	ns_uint32 col_idx = ns_uint32(std::floor((x - min_x) / length_x_voxel));
	ns_uint32 row_idx = ns_uint32(std::floor((y - min_y) / length_y_voxel));
	ns_uint32 height_idx = ns_uint32(std::floor((z - min_z) / length_z_voxel));
	//if (col_idx == cols_of_voxel)
	//	col_idx--;
	//if (row_idx == rows_of_voxel)
	//	row_idx--;
	//if (height_idx == depths_of_voxel)
	//	height_idx--;
	//if (col_idx == cols_of_voxel || row_idx == rows_of_voxel || height_idx == depths_of_voxel)
	//{
	//	log_warn("voxel size exceed x<%ld, %ld>, y<%ld, %ld>, z<%ld, %ld>", col_idx, cols_of_voxel, row_idx, rows_of_voxel, height_idx, depths_of_voxel);
	//}

	return height_idx*(cols_of_voxel*rows_of_voxel) + row_idx*cols_of_voxel + col_idx;
}

ns_uint32 NormalCoreSpace::NormalEstimation::_FindNearCenterPoint(const std::vector<ns_uint32>& voxel_indexs)
{
	interim_value_type local_min_x, local_min_y, local_min_z, local_max_x, local_max_y, local_max_z;
	local_min_x = local_min_y = local_min_z = std::numeric_limits<interim_value_type>::infinity();
	local_max_x = local_max_y = local_max_z = -std::numeric_limits<interim_value_type>::infinity();
	for (ns_uint32 i = 0; i < voxel_indexs.size(); i++)
	{
		local_min_x = (local_min_x > ns_input_data->points->at(voxel_indexs[i]).x) ? ns_input_data->points->at(voxel_indexs[i]).x : local_min_x;
		local_max_x = (local_max_x < ns_input_data->points->at(voxel_indexs[i]).x) ? ns_input_data->points->at(voxel_indexs[i]).x : local_max_x;
		local_min_y = (local_min_y > ns_input_data->points->at(voxel_indexs[i]).y) ? ns_input_data->points->at(voxel_indexs[i]).y : local_min_y;
		local_max_y = (local_max_y < ns_input_data->points->at(voxel_indexs[i]).y) ? ns_input_data->points->at(voxel_indexs[i]).y : local_max_y;
		local_min_z = (local_min_z > ns_input_data->points->at(voxel_indexs[i]).z) ? ns_input_data->points->at(voxel_indexs[i]).z : local_min_z;
		local_max_z = (local_max_z < ns_input_data->points->at(voxel_indexs[i]).z) ? ns_input_data->points->at(voxel_indexs[i]).z : local_max_z;
	}
	//get voxel center position
	interim_value_type center_x, center_y, center_z;
	center_x = (local_max_x + local_min_x) / 2;
	center_y = (local_max_y + local_min_y) / 2;
	center_z = (local_max_z + local_min_z) / 2;

	interim_value_type mini_distance = std::numeric_limits<interim_value_type>::infinity();
	ns_uint32 near_center_index = 0;
	// find the nearest point by center position
	for (ns_uint32 i = 0; i < voxel_indexs.size(); i++)
	{
		interim_value_type value = Util_Math::ComputePointToPointDist<interim_value_type, Point3f>(Point3f(center_x, center_y, center_z),
			ns_input_data->points->at(voxel_indexs[i]));
		if (value < mini_distance)
		{
			mini_distance = value;
			near_center_index = i;
		}
	}
	return near_center_index;
}

void NormalCoreSpace::NormalEstimation::_MiniRadiusDetected()
{
	if (!isDensitySet)
	{
		std::vector<interim_value_type> voxels_distance;
		//the voxel step is 20
		for (ns_uint32 i = 0; i < numbers_voxel_grid; i += DETECT_RADIUS_VOXEL_STEP)
		{
			// two points in voxel at least
			if (all_voxel_items[i].points_index.size() > OCCUPIED_VOXEL_POINTS)
			{
				ns_uint32 center_index = _FindNearCenterPoint(all_voxel_items[i].points_index);
				std::vector<interim_value_type> voxel_item_distance;
				for (ns_uint32 j = 0; j < all_voxel_items[i].points_index.size(); j++)
				{
					if (j != center_index)
					{
						voxel_item_distance.push_back(Util_Math::ComputePointToPointDist<interim_value_type, Point3f>(
							ns_input_data->points->at(all_voxel_items[i].points_index[j]),
							ns_input_data->points->at(all_voxel_items[i].points_index[center_index])));
					}
				}
				std::sort(voxel_item_distance.begin(), voxel_item_distance.end());
				//get average distance for current voxel
				ns_uint32 average_distance_size = DETECT_RADIUS_NEAR_POINTS;
				if (voxel_item_distance.size() < DETECT_RADIUS_NEAR_POINTS)
				{
					average_distance_size = (ns_uint32)voxel_item_distance.size();
				}
				interim_value_type items_total_distance = 0;
				for (ns_uint32 m = 0; m < average_distance_size; m++)
				{
					items_total_distance += voxel_item_distance[m];
				}
				if (items_total_distance > 0)
				{
					voxels_distance.push_back(items_total_distance / average_distance_size);
				}
			}
		}
		
		//get average distance for all detected voxels
		interim_value_type total_detected_distance = 0;
		for (interim_value_type i : voxels_distance)
		{
			total_detected_distance += i;
		}
		input_density = total_detected_distance / voxels_distance.size();
	}

	mini_p2p_radius = input_density * DENSITY_RADIUS_SCALE;
	ns_output_data.density = input_density;
}

bool NormalCoreSpace::NormalEstimation::Run(NormalEstimationResults& output_data, bool isDetected)
{
	log_info("NormalEstimation::Run start");
	_Init();
#ifdef NORMALS_COMPUTE_DEBUG
	std::remove(badpoints_of_less_min_points);
	std::remove(badpoints_of_mse);
	std::remove(less5_point);
	std::remove(badpoints_of_bad_angle);
#endif
	if (!ns_input_data)
	{
		log_error("empty input");
		return false;
	}

	//get min/max value of x, y, z
	START_TIME_COUNT(t1)
	_GetMinMaxXYZ(*ns_input_data->points);
	END_TIME_COUNT("_GetMinMaxXYZ time: ", t1)

	interim_value_type voxel_size_detected = ns_input_data->voxel_size_x;
	if (isDetected)
	{
		START_TIME_COUNT(t2)
		//voxel_size_detected = 5.0f;
		voxel_size_detected = _VoxelSizeDetected();
		END_TIME_COUNT("voxel_size_detected time: ", t2)
	}

	if (Util_Math::IsValZero(voxel_size_detected))
	{
		log_error("voxel size is 0");
		return false;
	}

	START_TIME_COUNT(t3)
	_ThresholdSetting(voxel_size_detected, voxel_size_detected, voxel_size_detected);
	END_TIME_COUNT("_InitParameters time: ", t3)

#ifdef NORMALS_COMPUTE_DEBUG
	std::cout << "point to points radius: " << range_search_radius << "\nmse threshold: " << ns_input_data->threshold.THRESHOLD_MIN_MSE_OF_VOXEL << std::endl;
#endif // NORMALS_COMPUTE_DEBUG

	// calculate all grid of voxel
	START_TIME_COUNT(t4)
	_AllocVoxelGrid();
	END_TIME_COUNT("_AllocVoxelGrid time: ", t4)

	//allocate voxel struct
	START_TIME_COUNT(t5)
#if 0
	_VoxelGridInitializeThread();
#else
	_VoxelGridInitialize();
#endif
	END_TIME_COUNT("_VoxelGridInitialize time: ", t5)

	//detect mini radius to calculate point to point distance
	START_TIME_COUNT(t6)
	_MiniRadiusDetected();
	//input_density = 0.638f;
	//mini_p2p_radius = input_density * DENSITY_RADIUS_SCALE;
	END_TIME_COUNT("MiniRadiusDectected time: ", t6)
	log_info("current voxel size: %lf, points density: %lf", voxel_size_detected, input_density);
#ifdef NORMALS_COMPUTE_DEBUG
	std::cout << "current voxel size: " << voxel_size_detected << "\tpoints density: " << mini_p2p_radius << std::endl;
	std::ofstream bad_angle(badpoints_of_bad_angle);
	bad_angle << "ply" << std::endl;
	bad_angle << "format ascii 1.0" << std::endl;
	bad_angle << "comment VCGLIB generated" << std::endl;
	bad_angle << "element vertex " << 0 << std::endl;
	bad_angle << "property float x" << std::endl;
	bad_angle << "property float y" << std::endl;
	bad_angle << "property float z" << std::endl;
	bad_angle << "property float nx" << std::endl;
	bad_angle << "property float ny" << std::endl;
	bad_angle << "property float nz" << std::endl;
	bad_angle << "element face 0" << std::endl;
	bad_angle << "property list uchar int vertex_indices" << std::endl;
	bad_angle << "end_header" << std::endl;
	bad_angle.close();
#endif

	//occupied voxel normal, neighbour, calculation
	//START_TIME_COUNT(t7)
	//_OccupiedVoxelNeighbourCalculation();
	//END_TIME_COUNT("_OccupiedVoxelNeighbourCalculation time: ", t7)

	//debug
	//for (int i = 0; i < all_voxel_items.size(); i++)
	//{
	//	int size = all_voxel_items[i].neighbours.size();
	//	if (size > 0 && size != 7 && size != 11 && size != 17 && size != 26)
	//	{
	//		std::cout << "voxel " << i << " neighbour invalid size " << size << std::endl;
	//	}
	//}


	//good / bad voxel classified
	START_TIME_COUNT(t10)
	_GoodVoxelVerified();
	END_TIME_COUNT("_GoodVoxelVerified time: ", t10)

#ifdef MULTI_THREADS_PROCESS
	START_TIME_COUNT(t11)
#ifdef TBB_THREADS_PARALLEL_USING
	tbb::parallel_for(0, (int)std::thread::hardware_concurrency() - 1, [&](int i) {
		VoxelNormalThread(this, GOOD_VOXEL_THREAD);});
#elif OMP_THREADS_USING
	omp_set_num_threads(omp_get_num_procs() - 1);
#pragma omp parallel
	{
		VoxelNormalThread(this, GOOD_VOXEL_THREAD);
	}
#else // some error when tbb used
	START_TIME_COUNT(t9)
	_StartThread(GOOD_VOXEL_THREAD);
	END_TIME_COUNT("_StartThread good time: ", t9)
	_EndThread(GOOD_VOXEL_THREAD);

	//START_TIME_COUNT(t8)
	//_StartThread(BAD_VOXEL_THREAD);
	//END_TIME_COUNT("_StartThread bad time: ", t8)

	//START_TIME_COUNT(t12)
	//_EndThread(BAD_VOXEL_THREAD);
	//END_TIME_COUNT("_EndThread bad time: ", t12)
#endif
	END_TIME_COUNT("_EndThread good time: ", t11)
#endif

	//save details for output
	START_TIME_COUNT(t13)
	_SaveOutput();
	END_TIME_COUNT("_SaveOutput time: ", t13)

	output_data = ns_output_data;
	_Reset();
	return true;
}

void NormalCoreSpace::NormalEstimation::SetDataDensity(interim_value_type density)
{
	isDensitySet = true;
	input_density = density;
}

bool NormalCoreSpace::NormalEstimation::GetDataDensity(interim_value_type& density)
{
	if (isDensitySet)
	{
		density = input_density;
		return true;
	}
	return false;
}
