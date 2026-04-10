#ifndef _UTIL_LINE_INTERSECTION_H_
#define _UTIL_LINE_INTERSECTION_H_

//Detect if line segment ab cd intersect
//by yu.liang@unre.com
//03.Dec.2020
//url: https://www.cnblogs.com/wuwangchuxin0924/p/6218494.html
//url: https://www.cnblogs.com/devymex/archive/2010/08/19/1803885.html

#include <opencv2/opencv.hpp>
namespace Util_Line_Intersection {

	static bool isIntersect(const cv::Point2i &a, const cv::Point2i &b, const cv::Point2i &c, const cv::Point2i &d)
	{
		/*
		快速排斥：
		两个线段为对角线组成的矩形，如果这两个矩形没有重叠的部分，那么两条线段是不可能出现重叠的
		*/
		if (!(MIN(a.x, b.x) <= MAX(c.x, d.x) && MIN(c.y, d.y) <= MAX(a.y, b.y) &&
			MIN(c.x, d.x) <= MAX(a.x, b.x) && MIN(a.y, b.y) <= MAX(c.y, d.y)))//这里的确如此，这一步是判定两矩形是否相交							
			/*特别要注意一个矩形含于另一个矩形之内的情况*/
			return false;
		/*
		跨立实验：
		如果两条线段相交，那么必须跨立，就是以一条线段为标准，另一条线段的两端点一定在这条线段的两段
		也就是说a b两点在线段cd的两端，c d两点在线段ab的两端
		*/
		long long u, v, w, z;//分别记录两个向量
		u = (c.x - a.x)*(b.y - a.y) - (b.x - a.x)*(c.y - a.y);
		v = (d.x - a.x)*(b.y - a.y) - (b.x - a.x)*(d.y - a.y);
		w = (a.x - c.x)*(d.y - c.y) - (d.x - c.x)*(a.y - c.y);
		z = (b.x - c.x)*(d.y - c.y) - (d.x - c.x)*(b.y - c.y);
		/*cout << u * v << "  " << w * z << endl;*/
		return (u*v < 0 && w*z < 0);
	}
}
#endif // !_UTIL_LINE_INTERSECTION_H_
