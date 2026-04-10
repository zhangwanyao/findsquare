#ifndef VOXEL_H
#define VOXEL_H

#include <opencv/cv.hpp>
#include <set>
#include <iostream>
//#define DEBUG_INFO

namespace VoxelDataStruct {
	//for plane edge detection
	struct SimpleVoxelGrid
	{
	public:

		bool is_occupied;
		int plane_point_num;
		int root_id;

	public:

		SimpleVoxelGrid() {
			is_occupied = false;
			plane_point_num = 0;
		};
	};

	//voxel grid base structure
	struct VoxelGrid_Base {

	public:

		//number of 3d points in this voxel
		int num_of_pt;

		//used for calculating covariance matrix
		double sum_xx, sum_yy, sum_zz, sum_xy, sum_xz, sum_yz, sum_x, sum_y, sum_z;

		//plane mean squared error and curvature
		float plane_mse, plane_curvature;

		//plane center
		float plane_center[3];

		//plane normals
		float plane_normals[3];

	public:

		VoxelGrid_Base() {
			this->Init();
		};

		//reset voxel properties
		inline void Init() {
			num_of_pt = 0;
			plane_mse = std::numeric_limits<float>::infinity();
			plane_curvature = std::numeric_limits<float>::infinity();
			sum_xx = sum_yy = sum_zz = sum_xy = sum_xz = sum_yz = sum_x = sum_y = sum_z = 0.f;
			std::memset(plane_normals, 0.f, sizeof(float) * 3);
			std::memset(plane_center, 0.f, sizeof(float) * 3);
		}

		//push data for calculating covariance matrix
		inline void Push(const float x, const float y, const float z) {
			this->sum_x += x; this->sum_y += y; this->sum_z += z;
			this->sum_xx += x*x; this->sum_yy += y*y; this->sum_zz += z*z;
			this->sum_xy += x*y; this->sum_xz += x*z; this->sum_yz += y*z;
			this->num_of_pt++;
		}

		inline void Push(const cv::Point3f pt) {
			Push(pt.x, pt.y, pt.z);
		}

		inline void Pop(const float x, const float y, const float z) {
			this->sum_x -= x; this->sum_y -= y; this->sum_z -= z;
			this->sum_xx -= x*x; this->sum_yy -= y*y; this->sum_zz -= z*z;
			this->sum_xy -= x*y; this->sum_xz -= x*z; this->sum_yz -= y*z;
			this->num_of_pt--;
		}

		inline void Pop(const cv::Point3f pt) {
			Pop(pt.x, pt.y, pt.z);
		}

		//get plane center
		inline void CalPlaneCenter() {
			if (this->num_of_pt <= 0) return;

			double point_num_inverse = (double)1.0 / this->num_of_pt;

			//plane center
			this->plane_center[0] = sum_x*point_num_inverse;
			this->plane_center[1] = sum_y*point_num_inverse;
			this->plane_center[2] = sum_z*point_num_inverse;
		}

		inline float ComputeNormalsSimilarity(float plane_normals[3]) {
			return this->plane_normals[0] * plane_normals[0] + this->plane_normals[1] * plane_normals[1] + this->plane_normals[2] * plane_normals[2];
		}

		//compute plane properties
		virtual inline void Compute() {

			//if (!this->is_occupied || num_of_pt < 4) return;
			if (num_of_pt < 4) return;
			double point_num_inverse = (double)1.0 / this->num_of_pt;

			//plane center
			this->plane_center[0] = sum_x*point_num_inverse;
			this->plane_center[1] = sum_y*point_num_inverse;
			this->plane_center[2] = sum_z*point_num_inverse;

			//calculate covariance matrix
			double cov_mat_element[6] = {	sum_xx - sum_x*plane_center[0],sum_xy - sum_x*plane_center[1],
											sum_xz - sum_x*plane_center[2],sum_yy - sum_y*plane_center[1],
											sum_yz - sum_y*plane_center[2],sum_zz - sum_z*plane_center[2] };

			//double cov_mat_element2[6];
			//cov_mat_element2[0] = (sum_xx - 2 * plane_center[0] * sum_x)*point_num_inverse + std::pow(plane_center[0], 2);
			//std::cout << cov_mat_element2[0] << std::endl;

			double cov_mat_arr[3][3] = { { cov_mat_element[0],cov_mat_element[1],cov_mat_element[2] },
			{ cov_mat_element[1], cov_mat_element[3], cov_mat_element[4] },
			{ cov_mat_element[2], cov_mat_element[4], cov_mat_element[5] } };
		
			cv::Mat cov_mat(3, 3, CV_64F, cov_mat_arr);
			cov_mat *= point_num_inverse;

			//std::cout << cov_mat << std::endl;

			cv::Mat eig_val_mat, eig_vec_mat;

			cv::eigen(cov_mat, eig_val_mat, eig_vec_mat);

			//std::cout << eig_val_mat << std::endl;
			//std::cout << eig_vec_mat << std::endl;
			for (int i = 0; i < 3; i++)
				this->plane_normals[i] = eig_vec_mat.at<double>(2, i);

			//plane mean squared error
			this->plane_mse = eig_val_mat.at<double>(2) * point_num_inverse;

			//curvature
			this->plane_curvature = eig_val_mat.at<double>(2) / (eig_val_mat.at<double>(0) + eig_val_mat.at<double>(1) + eig_val_mat.at<double>(2));

			//TODO: shift all the eigen code written by chen meng to math operation
			//double eig_val[3], eig_vec[3];
			//TODO: it still has some bad result
			//MatrixOperation::FastEigen_cm(cov_mat_element, eig_val, eig_vec);
			//TODO: waiting for test
			//MathOperation::ComputeFastEigenParallel(&cov_mat_element[0], &eig_val[0], &eig_vec[0]);
			//for (int i = 0; i < 3; i++)
			//	this->plane_normals[i] = eig_vec[i];

			////plane mean squared error
			//this->plane_mse = eig_val[2] * point_num_inverse;

			////curvature
			//this->plane_curvature = eig_val[2] / (eig_val[0] + eig_val[1] + eig_val[2]);
		}

	};

	//for region grow plane segmentation
	struct VoxelGridPlaneSeg :VoxelGrid_Base {
	public:

		typedef VoxelGridPlaneSeg *voxel_grid_ptr;
		typedef std::set<int> NeighborSet;

		//define if this voxel is occupied (occupied = have 1 - 3 pts)
		bool is_occupied;

		//define if this voxel is valid (valid = have neighbors)
		bool is_having_neighbors;

		//voxel id of this voxel
		int root_id;

		//parent id of this voxel
		int parent_root_id;

		//plane id of this voxel
		int plane_id;

		////number of voxels merged to this voxel
		//int voxel_num;

		//point cloud index of pts within this voxel
		std::vector<int> point_cloud_idx;

		//good and bad neighbors
		NeighborSet neighbor, bad_neighbor;

	public:

		VoxelGridPlaneSeg() :VoxelGrid_Base() {
			this->Reset();
		};

		~VoxelGridPlaneSeg() {
		}

		inline void Reset() {
			this->Init();
			plane_id = -1;
			is_occupied = is_having_neighbors = false;
			//voxel_num = 1;

			if (!point_cloud_idx.empty()) {
				point_cloud_idx.clear(); 
				point_cloud_idx.shrink_to_fit();
			}

			
			if (!neighbor.empty()) 
				neighbor.clear(); 
					
			if (!bad_neighbor.empty())
				bad_neighbor.clear();
		}

		inline void ConnectNeighbor(VoxelGridPlaneSeg *v, bool is_good_neighbor) {

			if (is_good_neighbor){
				neighbor.insert(v->root_id);
				v->neighbor.insert(this->root_id);
			}
			else{
				bad_neighbor.insert(v->root_id);
				v->bad_neighbor.insert(this->root_id);
			}
		}

		inline void DisconnectNeighbor(const std::vector<VoxelGridPlaneSeg *> &voxels) {

			NeighborSet::iterator itr = neighbor.begin();
			for (; itr != neighbor.end(); ++itr) {
				int v = (*itr);
				voxels[v]->neighbor.erase(root_id);
				//this->neighbor.erase(v); // by wei.fan
			}

			itr = bad_neighbor.begin();
			for (; itr != bad_neighbor.end(); ++itr) {
				int v = (*itr);
				voxels[v]->bad_neighbor.erase(root_id);
				//this->bad_neighbor.erase(v); // by wei.fan
			}
		}

		inline void Merge(VoxelGridPlaneSeg *v) {

			//update plane properties
			this->sum_x += v->sum_x;  this->sum_y += v->sum_y;  this->sum_z += v->sum_z;
			this->sum_xx += v->sum_xx; this->sum_yy += v->sum_yy; this->sum_zz += v->sum_zz;
			this->sum_xy += v->sum_xy; this->sum_xz += v->sum_xz; this->sum_yz += v->sum_yz;

			this->num_of_pt += v->num_of_pt;
			//this->voxel_num += v->voxel_num;
			this->Compute();

			// may be need check normal similarity before merge neighbor . by wei.fan
			
			//update good neighbors
			this->neighbor.insert(v->neighbor.begin(), v->neighbor.end());
			this->neighbor.erase(v->root_id);
			this->neighbor.erase(root_id);

			//update bad neighbors
			this->bad_neighbor.insert(v->bad_neighbor.begin(), v->bad_neighbor.end());
			this->bad_neighbor.erase(v->root_id);
			this->bad_neighbor.erase(root_id);

			//disconnect each others
			v->parent_root_id = this->parent_root_id;
			//v->DisconnectNeighbor();
			//this->DisconnectNeighbor();
		}

		inline float GetPointPlaneDist(float point[3]) {
			return std::fabs(this->plane_normals[0] * (point[0] - this->plane_center[0]) +
							 this->plane_normals[1] * (point[1] - this->plane_center[1]) +
							 this->plane_normals[2] * (point[2] - this->plane_center[2]));
		}

	};

	//for sub - plane estimation
	struct VoxelGrid_Normals :VoxelGrid_Base {

	public:
		//be care of set order if voxel grid growing
		typedef std::set<VoxelGrid_Normals*> NeighborSet;

		//voxel id
		int root_id;

		//define if the voxel is occupied
		bool is_occupied;

		//points normals within this voxel
		std::vector<cv::Point3f> pt_normals;

		//point index within this voxel
		std::vector<int> point_cloud_idx;

		//neighbors of this voxel
		NeighborSet neighbor;

	public:

		VoxelGrid_Normals() :VoxelGrid_Base() {
			this->Reset();
		}

		inline void Reset() {
			this->Init();
			is_occupied = false;

			if (!pt_normals.empty()) {
				pt_normals.clear();
				pt_normals.shrink_to_fit();
			}
			

			if (!point_cloud_idx.empty()) {
				point_cloud_idx.clear();
				point_cloud_idx.shrink_to_fit();
			}
			

			if (!neighbor.empty())
				neighbor.clear();
		}

		inline void ConnectNeighbor(VoxelGrid_Normals *v) {
			neighbor.insert(v);
		}

	};

	//for point estimation
	struct Point_Normals :VoxelGrid_Base {

	public:
		Point_Normals() :VoxelGrid_Base() {
		}
	};

	//for line approximation
	//TODO refactor VoxelGrid_LinesApprox
	struct VoxelGrid_LinesApprox {

	public:

		//number of 3d points in this voxel
		int num_of_pt;

		//used for calculating covariance matrix
		float sum_xx, sum_yy, sum_zz, sum_xy, sum_xz, sum_yz, sum_x, sum_y, sum_z;

		//plane mean squared error and curvature
		float plane_mse, plane_curvature;

		//line direction
		float line_direction[3];

		//center
		float center[3];

	public:

		typedef VoxelGrid_LinesApprox *voxel_grid_ptr;
		//be care of set order if voxel grid growing
		typedef std::set<voxel_grid_ptr> NeighborSet;

		//define if this voxel is occupied (occupied = have 1 - 3 pts)
		bool is_occupied;

		//define if this voxel is valid (valid = have neighbors)
		bool is_valid;

		//voxel id of this voxel
		int root_id;

		//parent id of this voxel
		int parent_root_id;

		//plane id of this voxel
		int line_id;

		//number of voxels merged to this voxel
		int voxel_num;

		//point cloud index of pts within this voxel
		std::vector<int> point_cloud_idx;

		//neighbors
		NeighborSet neighbor;

	public:

		VoxelGrid_LinesApprox() {
			this->Init();
		};

		//reset voxel properties
		inline void Init() {
			num_of_pt = 0;
			sum_xx = sum_yy = sum_zz = sum_xy = sum_xz = sum_yz = sum_x = sum_y = sum_z = 0.f;
			std::memset(line_direction, 0.f, sizeof(float) * 3);
			std::memset(center, 0.f, sizeof(float) * 3);
			is_occupied = is_valid = false;
			voxel_num = 1;

			if (!point_cloud_idx.empty())
				point_cloud_idx.clear();
			if (!neighbor.empty())
				neighbor.clear();
		}

		inline void Reset() {
			this->Init();
		}

		//push data for calculating covariance matrix
		inline void Push(const float x, const float y, const float z) {
			this->sum_x += x; this->sum_y += y; this->sum_z += z;
			this->sum_xx += x*x; this->sum_yy += y*y; this->sum_zz += z*z;
			this->sum_xy += x*y; this->sum_xz += x*z; this->sum_yz += y*z;
			this->num_of_pt++;
		}

		inline void Push(const cv::Point3f pt) {
			Push(pt.x, pt.y, pt.z);
		}

		//get plane center
		inline void CalPlaneCenter() {
			if (this->num_of_pt <= 0) return;

			double point_num_inverse = (double)1.0 / this->num_of_pt;

			//plane center
			this->center[0] = sum_x*point_num_inverse;
			this->center[1] = sum_y*point_num_inverse;
			this->center[2] = sum_z*point_num_inverse;
		}

		//compute plane properties
		inline void Compute() {

			if (num_of_pt < 4) return;
			double point_num_inverse = (double)1.0 / this->num_of_pt;

			//plane center
			this->center[0] = sum_x*point_num_inverse;
			this->center[1] = sum_y*point_num_inverse;
			this->center[2] = sum_z*point_num_inverse;

			//calculate covariance matrix
			double cov_mat_element[6] = { sum_xx - sum_x*center[0],sum_xy - sum_x*center[1],
				sum_xz - sum_x*center[2],sum_yy - sum_y*center[1],
				sum_yz - sum_y*center[2],sum_zz - sum_z*center[2] };

			double cov_mat_arr[3][3] = { {cov_mat_element[0],cov_mat_element[1],cov_mat_element[2]},
			{cov_mat_element[1], cov_mat_element[3], cov_mat_element[4]},
			{cov_mat_element[2], cov_mat_element[4], cov_mat_element[5]} };

			cv::Mat cov_mat(3, 3, CV_64F, cov_mat_arr);
			cv::Mat eig_val, eig_vec;

			cv::eigen(cov_mat, eig_val, eig_vec);

			for (int i = 0; i < 3; i++) {
				this->line_direction[i] = eig_vec.at<double>(0, i);
			}
		}

	public:

		//compute line direction similarity
		inline float ComputeLineDirSimilarity(float line_direction[3]) {
			return this->line_direction[0] * line_direction[0] + this->line_direction[1] * line_direction[1] + this->line_direction[2] * line_direction[2];
		}

		inline float ComputePtLineDist(float pt[3]) {
			float temp_pt[3], vec1[3], vec2[3], vec3[3];
			for (int i = 0; i < 3; i++) {
				temp_pt[i] = this->center[i] + this->line_direction[i];
				vec1[i] = temp_pt[i] - this->center[i];
				vec2[i] = pt[i] - this->center[i];
			}

			float d = 0.f;
			for (int i = 0; i < 3; i++)
				d += vec1[i] * vec2[i];

			for (int i = 0; i < 3; i++)
				vec3[i] = d*this->line_direction[i];

			float dist = std::sqrt(std::pow(vec2[0] - vec3[0], 2) + std::pow(vec2[1] - vec3[1], 2) + std::pow(vec2[2] - vec3[2], 2));

			return dist;
		}

		inline void ConnectNeighbor(VoxelGrid_LinesApprox *v) {
			neighbor.insert(v);
			v->neighbor.insert(this);
		}

		inline void DisconnectNeighbor() {
			//delete self from neighbors' neigbour list
			NeighborSet::iterator itr = neighbor.begin();
			for (; itr != neighbor.end(); ++itr) {
				VoxelGrid_LinesApprox* v = (*itr);
				v->neighbor.erase(this);
			}
		}

		inline void Merge(VoxelGrid_LinesApprox *v) {

			//update plane properties
			this->sum_x += v->sum_x;  this->sum_y += v->sum_y;  this->sum_z += v->sum_z;
			this->sum_xx += v->sum_xx; this->sum_yy += v->sum_yy; this->sum_zz += v->sum_zz;
			this->sum_xy += v->sum_xy; this->sum_xz += v->sum_xz; this->sum_yz += v->sum_yz;

			this->num_of_pt += v->num_of_pt;
			this->voxel_num += v->voxel_num;
			this->Compute();

			//insert v's neighbour list to this voxel
			this->neighbor.insert(v->neighbor.begin(), v->neighbor.end());
			//erase v from this voxel's neighbor list
			this->neighbor.erase(v);
			//erase self from this voxel's neighbor list
			this->neighbor.erase(this);

			//set parent id of v to be this voxel's parent id 
			v->parent_root_id = this->parent_root_id;

			//disconnect v from v's neighbour		
			v->DisconnectNeighbor();
			this->DisconnectNeighbor();
		}

	};
}
#endif VOXEL_H
