#ifndef _MODULES_STRUCT_H_
#define _MODULES_STRUCT_H_

#include <opencv2/core.hpp>
#include <unordered_map>
 
namespace ModuleStruct {

	// Data Struture 
	typedef cv::Point3i Point3i;
	typedef cv::Point3f Point3f; //Internal struct used as default
	typedef cv::Point3d Point3d;
	typedef cv::Point2i Point2i;
	typedef cv::Point2f Point2f;
	typedef cv::Point2d Point2d;
	typedef cv::Mat		CMat;
    typedef cv::Scalar	CScalar;
    typedef cv::Size2i  CSize2i;
    typedef cv::Vec<int, 4> CVec4i;

	typedef float value_type;
	typedef double interim_value_type;

    template <typename T, class A = std::allocator<T>>
    using Vector = std::vector<T, A>;

    using Point3iArray = Vector<Point3i>;
    using Point3fArray = Vector<Point3f>;
    using Point3dArray = Vector<Point3d>;
    using Point2iArray = Vector<Point2i>;
    using Point2fArray = Vector<Point2f>;
    using Point2dArray = Vector<Point2d>;

    //should be replaced
    //struct Point2
    //{
    //    value_type x;
    //    value_type y;
    //};
    //using Point2Array = Vector<Point2>;
    //struct Point3
    //{
    //    value_type x;
    //    value_type y;
    //    value_type z;
    //};
    //using Point3Array = Vector<Point3>;

    using Point2Array = Point2fArray;
    using Point3Array = Point3fArray;
    using Point2 = Point2f;
    using Point3 = Point3f;

    //2D
    // straight line
    struct Line2
    {
        Point2Array points;
        Point2 direction;
    };

    struct Rect2
    {
        Point2 vertices[4];
    };

    // free line
    struct Freeline2
    {
        Point2Array points;
    };

    struct Circle2
    {
        Point2 center;
        value_type radius;
    };

    struct Arc2
    {
        Point2 center;
        value_type starting_angle;
        value_type arc_angle;
        value_type radius;
    };

    //3D
    struct Line3
    {
        Point3Array points;
        Point3 direction;
    };

    struct Rect3
    {
        Point3 vertices[4];
    };

    struct Cube3
    {
        Point3 vertices[8];
    };

    // free line
    struct Freeline3
    {
        Point3Array points;
    };

    struct Plane3
    {
        Point3Array points;
        Point3 normal; // perpendicular with plane
        Point3 center;
        // Cube3 vertices; // minimum bounding box for `points`
    };

    struct Sphere3
    {
        Point3Array points;
        Point3 center;
        value_type radius;
    };

    struct Surface3
    {
        Point3Array points;
    };

	//Did not use template for module API interface
	template <typename T> using CMat_ = cv::Mat_<T>;

    using value_type = float;

    using interim_value_type = double;

    using Handle = void*;
}


#endif // !_MODULES_STRUCT_H_
