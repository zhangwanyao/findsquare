cmake_minimum_required(VERSION 3.6)

if(WIN32)
    add_definitions(-D_USE_MATH_DEFINES)
endif()

set(CUR_PROJECT_NAME PointReconstruction)

# --- C++ 标准要放在最上面 ---
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) 


# ========== Android 特殊配置 ==========
if(ANDROID)

    # --- 1. 警告配置：建议不要过度关闭 warning ---
    add_compile_options(
        -Wall
        -Wextra
        -Wno-unused-parameter
        -Wno-unused-variable
        -Wno-deprecated-declarations
        -Wno-deprecated-copy  # 新增：禁用Eigen隐式拷贝构造警告
        -Wno-int-in-bool-context  # 新增：禁用枚举转布尔警告
        -Wno-invalid-partial-specialization  # 解决TensorStorage模板错误
        -Wno-sign-compare  # 临时禁用符号比较警告
        -fdeclspec 
        -fdiagnostics-color=always
    )

    # --- 2. Sanitizer（ASan + UBSan） ---
    add_compile_options(
        -fsanitize=address,undefined
        -fno-sanitize-recover=all
    )

    add_link_options(
        -fsanitize=address,undefined
        -fno-sanitize-recover=all
    )

endif()


set(Draco_DIR ${CMAKE_INSTALL_PREFIX}/lib/draco/cmake)
find_package(Draco REQUIRED )

set(jansson_DIR ${CMAKE_INSTALL_PREFIX}/lib/cmake/jansson)
find_package(jansson REQUIRED )

find_package( OpenCV 3 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(src_root "${CMAKE_CURRENT_SOURCE_DIR}")
file(GLOB Common_src  "${src_root}/Common/*.cpp")
file(GLOB CutScanPlane_src  "${src_root}/CutScanPlane/DoorWindowDetector/*.cpp"
       "${src_root}/CutScanPlane/models/*.cpp"
       "${src_root}/CutScanPlane/operations/*.cpp")
file(GLOB DoorWindowDimension_src  "${src_root}/DoorWindowDimension/src/*.cpp")
file(GLOB IO_src  "${src_root}/IO/*.cpp")
file(GLOB log_src  "${src_root}/log/*.c") 
file(GLOB mcl_src  "${src_root}/mcl/*.cpp")
file(GLOB creatingmesh_src "${src_root}/CreatingMesh/*.cpp")
file(GLOB Measurement_src  "${src_root}/Measurement/*.cpp"
               "${src_root}/Measurement/RGBD/*.cpp")
file(GLOB Mesh_src "${src_root}/Mesh/mesher/*.c"
	 "${src_root}/Mesh/mesher/*.cpp")
file(GLOB MultiStage_src  "${src_root}/MultiStage/*.cpp")
file(GLOB NormalEstimation_src  "${src_root}/NormalEstimation/*.cpp")
#file(GLOB Object_src  "${src_root}/Object/*.cpp")
file(GLOB PlaneSegmentation_src  "${src_root}/PlaneSegmentation/*.cpp")
file(GLOB Reconstruction_src  "${src_root}/Reconstruction/*.cpp")
#file(GLOB SplitFloor_src  "${src_root}/SplitFloor/*.cpp")
file(GLOB tinyxml_src  "${src_root}/tinyxml/*.cpp")
file(GLOB src  "${src_root}/*.cpp")

add_library(${CUR_PROJECT_NAME} SHARED ${Common_src} ${CutScanPlane_src} ${DoorWindowDimension_src} ${DoorWindowDimension_src} ${IO_src} ${creatingmesh_src} ${log_src} ${mcl_src} ${Measurement_src} ${Mesh_src} ${mcl_src} ${MultiStage_src} ${NormalEstimation_src}  ${PlaneSegmentation_src} ${Reconstruction_src}  ${tinyxml_src} ${src})

add_definitions(-D_CRT_SECURE_NO_WARNINGS)

add_library(libopencv_java3 SHARED IMPORTED)
#set_target_properties(libopencv_java3  PROPERTIES IMPORTED_LOCATION  ${src_root}/../../../Android/sdk/native/libs/arm64-v8a/libopencv_world.so)






# directorys of headers
include_directories(
	${src_root}/DoorWindowDimension/include
	${src_root}/../../../Android/include/draco/api
	${src_root}/../../../Android/include
	${src_root}/../../../external/OpenCV_Android/sdk/native/jni/include
	${src_root}/../../../external/tbb/include
	${src_root}/PlaneSegmentation/include
	${src_root}/PlaneSegmentation/external
	${src_root}/NormalEstimation/include
	${src_root}/NormalEstimation/external
	${src_root}/log
	${src_root}/IO/include
	${src_root}/Common/include
	${src_root}/Common
	${src_root}/../../../external/CGAL/cgal-5.3/Polygonal_surface_reconstruction/test/Polygonal_surface_reconstruction
	${src_root}/../../../external/CGAL/cgal-5.3/AABB_tree/include
	${src_root}/../../../external/CGAL/cgal-5.3/Advancing_front_surface_reconstruction/include
	${src_root}/../../../external/CGAL/cgal-5.3/Algebraic_foundations/include
	${src_root}/../../../external/CGAL/cgal-5.3/Algebraic_kernel_d/include
	${src_root}/../../../external/CGAL/cgal-5.3/Algebraic_kernel_for_circles/include
	${src_root}/../../../external/CGAL/cgal-5.3/Algebraic_kernel_for_spheres/include
	${src_root}/../../../external/CGAL/cgal-5.3/Alpha_shapes_2/include
	${src_root}/../../../external/CGAL/cgal-5.3/Alpha_shapes_3/include
	${src_root}/../../../external/CGAL/cgal-5.3/Apollonius_graph_2/include
	${src_root}/../../../external/CGAL/cgal-5.3/Arithmetic_kernel/include
	${src_root}/../../../external/CGAL/cgal-5.3/Arrangement_on_surface_2/include
	${src_root}/../../../external/CGAL/cgal-5.3/BGL/include
	${src_root}/../../../external/CGAL/cgal-5.3/Barycentric_coordinates_2/include
	${src_root}/../../../external/CGAL/cgal-5.3/Boolean_set_operations_2/include
	${src_root}/../../../external/CGAL/cgal-5.3/Bounding_volumes/include
	${src_root}/../../../external/CGAL/cgal-5.3/Box_intersection_d/include
	${src_root}/../../../external/CGAL/cgal-5.3/CGAL_Core/include
	${src_root}/../../../external/CGAL/cgal-5.3/CGAL_ImageIO/include
	${src_root}/../../../external/CGAL/cgal-5.3/CGAL_ipelets/include
	${src_root}/../../../external/CGAL/cgal-5.3/Cartesian_kernel/include
	${src_root}/../../../external/CGAL/cgal-5.3/Circular_kernel_2/include
	${src_root}/../../../external/CGAL/cgal-5.3/Circular_kernel_3/include
	${src_root}/../../../external/CGAL/cgal-5.3/Circulator/include
	${src_root}/../../../external/CGAL/cgal-5.3/Classification/include
	${src_root}/../../../external/CGAL/cgal-5.3/Combinatorial_map/include
	${src_root}/../../../external/CGAL/cgal-5.3/Cone_spanners_2/include
	${src_root}/../../../external/CGAL/cgal-5.3/Convex_decomposition_3/include
	${src_root}/../../../external/CGAL/cgal-5.3/Convex_hull_2/include
	${src_root}/../../../external/CGAL/cgal-5.3/Convex_hull_3/include
	${src_root}/../../../external/CGAL/cgal-5.3/Convex_hull_d/include
	${src_root}/../../../external/CGAL/cgal-5.3/Distance_2/include
	${src_root}/../../../external/CGAL/cgal-5.3/Distance_3/include
	${src_root}/../../../external/CGAL/cgal-5.3/Envelope_2/include
	${src_root}/../../../external/CGAL/cgal-5.3/Envelope_3/include
	${src_root}/../../../external/CGAL/cgal-5.3/Filtered_kernel/include
	${src_root}/../../../external/CGAL/cgal-5.3/Generalized_map/include
	${src_root}/../../../external/CGAL/cgal-5.3/Generator/include
	${src_root}/../../../external/CGAL/cgal-5.3/Geomview/include
	${src_root}/../../../external/CGAL/cgal-5.3/GraphicsView/include
	${src_root}/../../../external/CGAL/cgal-5.3/HalfedgeDS/include
	${src_root}/../../../external/CGAL/cgal-5.3/Hash_map/include
	${src_root}/../../../external/CGAL/cgal-5.3/Heat_method_3/include
	${src_root}/../../../external/CGAL/cgal-5.3/Homogeneous_kernel/include
	${src_root}/../../../external/CGAL/cgal-5.3/Hyperbolic_triangulation_2/include
	${src_root}/../../../external/CGAL/cgal-5.3/Inscribed_areas/include
	${src_root}/../../../external/CGAL/cgal-5.3/Installation/include
	${src_root}/../../../external/CGAL/cgal-5.3/Interpolation/include
	${src_root}/../../../external/CGAL/cgal-5.3/Intersections_2/include
	${src_root}/../../../external/CGAL/cgal-5.3/Intersections_3/include
	${src_root}/../../../external/CGAL/cgal-5.3/Interval_skip_list/include
	${src_root}/../../../external/CGAL/cgal-5.3/Interval_support/include
	${src_root}/../../../external/CGAL/cgal-5.3/Jet_fitting_3/include
	${src_root}/../../../external/CGAL/cgal-5.3/Kernel_23/include
	${src_root}/../../../external/CGAL/cgal-5.3/Kernel_d/include
	${src_root}/../../../external/CGAL/cgal-5.3/LEDA/include
	${src_root}/../../../external/CGAL/cgal-5.3/Linear_cell_complex/include
	${src_root}/../../../external/CGAL/cgal-5.3/Matrix_search/include
	${src_root}/../../../external/CGAL/cgal-5.3/Mesh_2/include
	${src_root}/../../../external/CGAL/cgal-5.3/Mesh_3/include
	${src_root}/../../../external/CGAL/cgal-5.3/Mesher_level/include
	${src_root}/../../../external/CGAL/cgal-5.3/Minkowski_sum_2/include
	${src_root}/../../../external/CGAL/cgal-5.3/Minkowski_sum_3/include
	${src_root}/../../../external/CGAL/cgal-5.3/Modifier/include
	${src_root}/../../../external/CGAL/cgal-5.3/Modular_arithmetic/include
	${src_root}/../../../external/CGAL/cgal-5.3/Nef_2/include
	${src_root}/../../../external/CGAL/cgal-5.3/Nef_3/include
	${src_root}/../../../external/CGAL/cgal-5.3/Nef_S2/include
	${src_root}/../../../external/CGAL/cgal-5.3/NewKernel_d/include
	${src_root}/../../../external/CGAL/cgal-5.3/Number_types/include
	${src_root}/../../../external/CGAL/cgal-5.3/OpenNL/include
	${src_root}/../../../external/CGAL/cgal-5.3/Optimal_bounding_box/include
	${src_root}/../../../external/CGAL/cgal-5.3/Optimal_transportation_reconstruction_2/include
	${src_root}/../../../external/CGAL/cgal-5.3/Optimisation_basic/include
	${src_root}/../../../external/CGAL/cgal-5.3/Orthtree/include
	${src_root}/../../../external/CGAL/cgal-5.3/Partition_2/include
	${src_root}/../../../external/CGAL/cgal-5.3/Periodic_2_triangulation_2/include
	${src_root}/../../../external/CGAL/cgal-5.3/Periodic_3_mesh_3/include
	${src_root}/../../../external/CGAL/cgal-5.3/Periodic_3_triangulation_3/include
	${src_root}/../../../external/CGAL/cgal-5.3/Periodic_4_hyperbolic_triangulation_2/include
	${src_root}/../../../external/CGAL/cgal-5.3/Point_set_2/include
	${src_root}/../../../external/CGAL/cgal-5.3/Point_set_3/include
	${src_root}/../../../external/CGAL/cgal-5.3/Point_set_processing_3/include
	${src_root}/../../../external/CGAL/cgal-5.3/Poisson_surface_reconstruction_3/include
	${src_root}/../../../external/CGAL/cgal-5.3/Polygon/include
	${src_root}/../../../external/CGAL/cgal-5.3/Polygon_mesh_processing/include
	${src_root}/../../../external/CGAL/cgal-5.3/Polygonal_surface_reconstruction/include
	${src_root}/../../../external/CGAL/cgal-5.3/Polyhedron/include
	${src_root}/../../../external/CGAL/cgal-5.3/Polyline_simplification_2/include
	${src_root}/../../../external/CGAL/cgal-5.3/Polynomial/include
	${src_root}/../../../external/CGAL/cgal-5.3/Polytope_distance_d/include
	${src_root}/../../../external/CGAL/cgal-5.3/Principal_component_analysis/include
	${src_root}/../../../external/CGAL/cgal-5.3/Principal_component_analysis_LGPL/include
	${src_root}/../../../external/CGAL/cgal-5.3/Profiling_tools/include
	${src_root}/../../../external/CGAL/cgal-5.3/Property_map/include
	${src_root}/../../../external/CGAL/cgal-5.3/QP_solver/include
	${src_root}/../../../external/CGAL/cgal-5.3/Random_numbers/include
	${src_root}/../../../external/CGAL/cgal-5.3/Ridges_3/include
	${src_root}/../../../external/CGAL/cgal-5.3/STL_Extension/include
	${src_root}/../../../external/CGAL/cgal-5.3/Scale_space_reconstruction_3/include
	${src_root}/../../../external/CGAL/cgal-5.3/SearchStructures/include
	${src_root}/../../../external/CGAL/cgal-5.3/Segment_Delaunay_graph_2/include
	${src_root}/../../../external/CGAL/cgal-5.3/Segment_Delaunay_graph_Linf_2/include
	${src_root}/../../../external/CGAL/cgal-5.3/Set_movable_separability_2/include
	${src_root}/../../../external/CGAL/cgal-5.3/Shape_detection/include
	${src_root}/../../../external/CGAL/cgal-5.3/Skin_surface_3/include
	${src_root}/../../../external/CGAL/cgal-5.3/Snap_rounding_2/include
	${src_root}/../../../external/CGAL/cgal-5.3/Solver_interface/include
	${src_root}/../../../external/CGAL/cgal-5.3/Spatial_searching/include
	${src_root}/../../../external/CGAL/cgal-5.3/Spatial_sorting/include
	${src_root}/../../../external/CGAL/cgal-5.3/Straight_skeleton_2/include
	${src_root}/../../../external/CGAL/cgal-5.3/Stream_lines_2/include
	${src_root}/../../../external/CGAL/cgal-5.3/Stream_support/include
	${src_root}/../../../external/CGAL/cgal-5.3/Subdivision_method_3/include
	${src_root}/../../../external/CGAL/cgal-5.3/Surface_mesh/include
	${src_root}/../../../external/CGAL/cgal-5.3/Surface_mesh_approximation/include
	${src_root}/../../../external/CGAL/cgal-5.3/Surface_mesh_deformation/include
	${src_root}/../../../external/CGAL/cgal-5.3/Surface_mesh_parameterization/include
	${src_root}/../../../external/CGAL/cgal-5.3/Surface_mesh_segmentation/include
	${src_root}/../../../external/CGAL/cgal-5.3/Surface_mesh_shortest_path/include
	${src_root}/../../../external/CGAL/cgal-5.3/Surface_mesh_simplification/include
	${src_root}/../../../external/CGAL/cgal-5.3/Surface_mesh_skeletonization/include
	${src_root}/../../../external/CGAL/cgal-5.3/Surface_mesh_topology/include
	${src_root}/../../../external/CGAL/cgal-5.3/Surface_mesher/include
	${src_root}/../../../external/CGAL/cgal-5.3/Surface_sweep_2/include
	${src_root}/../../../external/CGAL/cgal-5.3/TDS_2/include
	${src_root}/../../../external/CGAL/cgal-5.3/TDS_3/include
	${src_root}/../../../external/CGAL/cgal-5.3/Testsuite/include
	${src_root}/../../../external/CGAL/cgal-5.3/Tetrahedral_remeshing/include
	${src_root}/../../../external/CGAL/cgal-5.3/Three/include
	${src_root}/../../../external/CGAL/cgal-5.3/Triangulation/include
	${src_root}/../../../external/CGAL/cgal-5.3/Triangulation_2/include
	${src_root}/../../../external/CGAL/cgal-5.3/Triangulation_3/include
	${src_root}/../../../external/CGAL/cgal-5.3/Triangulation_on_sphere_2/include
	${src_root}/../../../external/CGAL/cgal-5.3/Union_find/include
	${src_root}/../../../external/CGAL/cgal-5.3/Visibility_2/include
	${src_root}/../../../external/CGAL/cgal-5.3/Voronoi_diagram_2/include
	${src_root}/../../../external/CGAL/gmp/include
	${src_root}/../../../external/CGAL/boost_1_72_0
	${src_root}/../../../external/Eigen/eigen
	${src_root}/../../../Android/include
	)

add_definitions(-DCGAL_EIGEN3_ENABLED -DCGAL_USE_SCIP -D_CRT_SECURE_NO_WARNINGS -DBOOST_ALL_DYN_LINK=1)

add_library(libscip SHARED  IMPORTED)
set_target_properties(libscip PROPERTIES IMPORTED_LOCATION
        ${src_root}/../../../Android/lib/libscip.so
       )

#add_library(libncnn SHARED  IMPORTED)
#set_target_properties(libncnn PROPERTIES IMPORTED_LOCATION
#        ${measurement_root}/../external/ncnn/arm64-v8a/lib/libncnn.so
#       )

#add_library(libonnxruntime SHARED  IMPORTED)
#set_target_properties(libonnxruntime PROPERTIES IMPORTED_LOCATION
#        ${measurement_root}/../external/onnxruntime-linux-aarch64-1.12.1/lib/libonnxruntime.so
#       )

link_directories(${OpenCV_LIBRARY_DIRS}
		${Draco_LIBRARY_DIRS}
                         ${jansson_LIBRARY_DIRS}
		)

find_library(ndk-lib log)
		
MESSAGE (STATUS "${OpenCV_LIBS}")
target_link_libraries(${CUR_PROJECT_NAME}
		"${CMAKE_INSTALL_PREFIX}/lib/libjansson.a"
		"${CMAKE_INSTALL_PREFIX}/lib/libcodec_api.a"
		"${CMAKE_INSTALL_PREFIX}/lib/libdraco.a"
		${OpenCV_LIBS}
		libscip
		#libncnn
        #libonnxruntime
		${log-lib}
		${ndk-lib}
		)

#install to target
install(TARGETS ${CUR_PROJECT_NAME}
        EXPORT ${CUR_PROJECT_NAME} 
        LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/${CUR_PROJECT_NAME}/lib
		ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/${CUR_PROJECT_NAME}/lib
		)

install(FILES "${src_root}/DecorationAPI.h"
  DESTINATION ${CMAKE_INSTALL_PREFIX}/${CUR_PROJECT_NAME}/include
  )
  