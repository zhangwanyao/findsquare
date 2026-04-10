#ifndef __DOWNSAMPLE_VOXEL_POINTS__
#define __DOWNSAMPLE_VOXEL_POINTS__

#include "DataStruct.h"
#include "ConstantDefines.h"

typedef struct DownSample_Grid
{
	std::vector<ns_uint32> points_index;
	Point3f sample_point;
};

class VoxelDownsample
{
public:
	VoxelDownsample(float voxel_size, PointItem* points, const std::vector<ns_uint32>& points_index);
	~VoxelDownsample();

	void Run();
	void SaveNormals(Point3f normal, const int index);

	std::vector<DownSample_Grid> Voxels_Grid;

private:
	bool _GetMinMaxXYZ();
	bool _AllocVoxelGrid();
	ns_uint32 _ConvertPointToVoxelID(const IntermediateType x, const IntermediateType y, const IntermediateType z);
	void _VoxelGridInitialize();

	float Voxel_Size;
	PointItem* Points;
	std::vector<ns_uint32> Points_Index;

	IntermediateType min_x, min_y, min_z, max_x, max_y, max_z;
	ns_uint32 cols_of_voxel, rows_of_voxel, depths_of_voxel;
};

VoxelDownsample::VoxelDownsample(float voxel_size, PointItem* points, const std::vector<ns_uint32>& points_index)
{
	Voxel_Size = voxel_size;
	Points = points;
	Points_Index = points_index;

	Voxels_Grid.resize(0);
}

VoxelDownsample::~VoxelDownsample()
{
}

void VoxelDownsample::Run()
{
	_GetMinMaxXYZ();
	if (!_AllocVoxelGrid())
	{
		return;
	}
	_VoxelGridInitialize();
	for (std::vector<DownSample_Grid>::iterator i = Voxels_Grid.begin(); i < Voxels_Grid.end();)
	{
		if (i->points_index.size() > 0)
		{
			Point3f sum;
			for (int j = 0; j < i->points_index.size(); j++)
			{
				sum += Points[i->points_index[j]].point;
			}
			i->sample_point = sum / (float)i->points_index.size();
			i++;
		}
		else
			i = Voxels_Grid.erase(i);
	}
}

void VoxelDownsample::SaveNormals(Point3f normal, const int index)
{
	for (int i = 0; i < Voxels_Grid[index].points_index.size(); i++)
	{
		Points[Voxels_Grid[index].points_index[i]].normal = normal;
		//std::cout << "save normal mapping: " << normal.x << "\t" << normal.y << "\t" << normal.z << "\tindex: " << i << std::endl;
	}
}

bool VoxelDownsample::_GetMinMaxXYZ()
{
	if (!Points)
	{
		return false;
	}

	min_x = min_y = min_z = std::numeric_limits<IntermediateType>::infinity();
	max_x = max_y = max_z = -std::numeric_limits<IntermediateType>::infinity();
	for (ns_uint32 i = 0; i < Points_Index.size(); i++)
	{
		min_x = (min_x > Points[Points_Index[i]].point.x) ? Points[Points_Index[i]].point.x : min_x;
		max_x = (max_x < Points[Points_Index[i]].point.x) ? Points[Points_Index[i]].point.x : max_x;
		min_y = (min_y > Points[Points_Index[i]].point.y) ? Points[Points_Index[i]].point.y : min_y;
		max_y = (max_y < Points[Points_Index[i]].point.y) ? Points[Points_Index[i]].point.y : max_y;
		min_z = (min_z > Points[Points_Index[i]].point.z) ? Points[Points_Index[i]].point.z : min_z;
		max_z = (max_z < Points[Points_Index[i]].point.z) ? Points[Points_Index[i]].point.z : max_z;
	}

	return true;
}

bool VoxelDownsample::_AllocVoxelGrid()
{
	if ((max_x - min_x) != 0 && std::remainder((max_x - min_x), Voxel_Size) == 0)
		cols_of_voxel = ns_uint32(std::floor((max_x - min_x) / Voxel_Size));
	else
		cols_of_voxel = ns_uint32(std::floor((max_x - min_x) / Voxel_Size) + 1);

	if ((max_y - min_y) != 0 && std::remainder((max_y - min_y), Voxel_Size) == 0)
		rows_of_voxel = ns_uint32(std::floor((max_y - min_y) / Voxel_Size));
	else
		rows_of_voxel = ns_uint32(std::floor((max_y - min_y) / Voxel_Size) + 1);

	if ((max_z - min_z) != 0 && std::remainder((max_z - min_z), Voxel_Size) == 0)
		depths_of_voxel = ns_uint32(std::floor((max_z - min_z) / Voxel_Size));
	else
		depths_of_voxel = ns_uint32(std::floor((max_z - min_z) / Voxel_Size) + 1);

	if (cols_of_voxel * rows_of_voxel * depths_of_voxel <= 0)
	{
		return false;
	}
	Voxels_Grid.resize(cols_of_voxel * rows_of_voxel * depths_of_voxel);
	return true;
}

ns_uint32 VoxelDownsample::_ConvertPointToVoxelID(const IntermediateType x, const IntermediateType y, const IntermediateType z)
{
	ns_uint32 col_idx = ns_uint32(std::floor((x - min_x) / Voxel_Size));
	ns_uint32 row_idx = ns_uint32(std::floor((y - min_y) / Voxel_Size));
	ns_uint32 height_idx = ns_uint32(std::floor((z - min_z) / Voxel_Size));
	if (col_idx == cols_of_voxel)
		col_idx--;
	if (row_idx == rows_of_voxel)
		row_idx--;
	if (height_idx == depths_of_voxel)
		height_idx--;

	return height_idx * (cols_of_voxel*rows_of_voxel) + row_idx * cols_of_voxel + col_idx;
}

void VoxelDownsample::_VoxelGridInitialize()
{
	for (ns_uint32 i = 0; i < Points_Index.size(); i++)
	{
		ns_uint32 voxel_id = _ConvertPointToVoxelID(Points[Points_Index[i]].point.x, Points[Points_Index[i]].point.y, Points[Points_Index[i]].point.z);
		Voxels_Grid[voxel_id].points_index.push_back(Points_Index[i]);
	}
}
#endif
