#pragma once
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2\core\eigen.hpp>
#include <unsupported/Eigen/CXX11/Tensor>
#define _DEBUG
#define LIBREALSENSE2_
#define _DEBUG_
#ifndef max
#define max(a,b)            (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif 

class Irgbd
{
public:
    using Line2 = Eigen::Hyperplane<float, 2>;
    using Vec2 = Eigen::Vector2f;

    using Point = cv::Point2f;
    using pixel = std::pair<int, int>;
    auto ExtendLine(cv::Vec4i& line) {
        Vec2 p0(line[0], line[1]), p1(line[2], line[3]);
        Vec2 v = (p1 - p0);
        auto l = v.norm() * 0.5;
        v.normalize();
        p0 -= v * l;
        p1 += v * l;
        line = cv::Vec4i(p0[0], p0[1], p1[0], p1[1]);
    }

    auto ExtendLine(cv::Vec4i& lineIn, float l, cv::Vec4f& lineOut) {
        Vec2 p0(lineIn[0], lineIn[1]), p1(lineIn[2], lineIn[3]);
        Vec2 v = (p1 - p0).normalized();
        p0 -= v * l;
        p1 += v * l;
        lineOut = { p0[0], p0[1], p1[0], p1[1] };
    }

    auto ExtendLine_(cv::Vec4f& lineIn, float l, cv::Vec4f& lineOut) {
        Vec2 p0(lineIn[0], lineIn[1]), p1(lineIn[2], lineIn[3]);
        Vec2 v = (p1 - p0).normalized();
        auto c = (p0 + p1) / 2;
        p0 -= v * l;
        p1 += v * l;
        lineOut = { p0[0], p0[1], p1[0], p1[1] };
    }

    auto ExtendLine(cv::Vec4f& lineIn, float l, cv::Vec4f& lineOut) {
        Vec2 p0(lineIn[0], lineIn[1]), p1(lineIn[2], lineIn[3]);
        Vec2 v = (p1 - p0).normalized();
        auto c = (p0 + p1) / 2;
        p0 = c - v * l;
        p1 = c + v * l;
        lineOut = { p0[0], p0[1], p1[0], p1[1] };
    }

    auto ExtendLine(cv::Vec4i& lineIn, cv::Vec4i& box, cv::Vec4f& lineOut) {
        auto l = sqrt(box[2] * box[2] + box[3] * box[3]);
        ExtendLine(lineIn, l, lineOut);
        cv::Vec4f l1(box[0], box[1], box[2], box[1]),
            l2(box[0], box[1], box[0], box[3]),
            l3(box[2], box[1], box[2], box[3]),
            l4(box[0], box[3], box[2], box[3]);

        return true;
    }


    auto HoughLineP(cv::Mat& edge) {
        cv::Mat midImage(edge.size(), edge.type());
        //cv::cvtColor(edge, midImage,cv::COLOR_GRAY2BGR);

        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(edge, lines, 1, CV_PI / 180, 80, 50, 10);
#ifdef _DEBUG

        for (size_t i = 0; i < lines.size(); i++)
        {
            cv::Vec4i l = lines[i];
            line(midImage, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(186, 88, 255), 1, cv::LINE_AA);

        }
        //Show(edge);
        cv::imwrite("edge.png", midImage);

#endif // _DEBUG
    }

    auto HoughLineP(cv::Mat& edge, std::vector<cv::Vec4i>& lines) {
        cv::Mat midImage(edge.size(), edge.type());
        //cv::cvtColor(edge, midImage,cv::COLOR_GRAY2BGR);
        cv::HoughLinesP(edge, lines, 1, CV_PI / 240, 80, 100, 10);
#ifdef _DEBUG

        for (size_t i = 0; i < lines.size(); i++)
        {
            cv::Vec4i l = lines[i];
            line(midImage, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(186, 88, 255), 1, cv::LINE_AA);
            cv::putText(midImage, std::to_string(i), cv::Point(l[0], l[1]), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));

        }

        cv::imwrite("edge.png", midImage);
#endif // _DEBUG
    }
    auto HoughLineP(cv::Mat& edge, std::vector<cv::Vec4f>& lines) {
        cv::Mat midImage(edge.size(), edge.type());
        //cv::cvtColor(edge, midImage,cv::COLOR_GRAY2BGR);
        cv::HoughLinesP(edge, lines, 1, CV_PI / 180, 80, 100, 10);
#ifdef _DEBUG

        for (size_t i = 0; i < lines.size(); i++)
        {
            cv::Vec4f l = lines[i];
            line(midImage, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(186, 88, 255), 1, cv::LINE_AA);
            cv::putText(midImage, std::to_string(i), cv::Point(l[0], l[1]), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));

        }

        cv::imwrite("edge.png", midImage);
#endif // _DEBUG
    }

    auto DetectEdge(cv::Mat& rgb, cv::Mat& dst) {
        cv::Mat edge, gray;
        dst.create(rgb.size(), rgb.type());
        cv::cvtColor(rgb, gray, cv::COLOR_BGR2GRAY);
        cv::blur(gray, edge, cv::Size(3, 3));
        cv::Canny(edge, edge,
            30, 60, 3);
        //cv::Canny(edge, edge,
        //    10, 20, 3);
        //dst = cv::Scalar::all(0);
        //rgb.copyTo(dst, edge);
        //cv::waitKey();
        dst = edge;
    }

    auto Draw(cv::Mat& img, std::vector<cv::Point>& points, std::vector<int>& hull)
    {
        cv::Point p0 = points.back();
        for (size_t i = 0; i < hull.size(); i++)
        {
            auto p = points[hull[i]];
            line(img, p0, p, cv::Scalar(255, 255, 255));
            p0 = p;
        }
    }

    auto Draw(cv::Mat& img, std::vector<cv::Vec4f>& lines)
    {
        for (size_t i = 0; i < lines.size(); i++)
        {
            Point p0(lines[i][0], lines[i][1]), p1(lines[i][2], lines[i][3]);
            line(img, p0, p1, cv::Scalar((int)lines[0][0] % 255, (int)lines[0][1] % 255, lines.size() % 255));
        }
    }

    auto Draw(cv::Mat& img, std::vector<cv::Vec4f>& lines, const cv::Scalar& color, int thickness = 2)
    {
        for (size_t i = 0; i < lines.size(); i++)
        {
            Point p0(lines[i][0], lines[i][1]), p1(lines[i][2], lines[i][3]);
            line(img, p0, p1, color, max(/*int(color[2])%1*/0, thickness));
        }
    }

    //排斥实验
    bool IsRectCross(const Point& p1, const Point& p2, const Point& q1, const Point& q2)
    {
        bool ret = min(p1.x, p2.x) <= max(q1.x, q2.x) &&
            min(q1.x, q2.x) <= max(p1.x, p2.x) &&
            min(p1.y, p2.y) <= max(q1.y, q2.y) &&
            min(q1.y, q2.y) <= max(p1.y, p2.y);
        return ret;
    }

    bool IsRectCross(cv::Point2f& p1, cv::Point2f& p2, cv::Point2f& q1, cv::Point2f& q2)
    {
        bool ret = min(p1.x, p2.x) <= max(q1.x, q2.x) &&
            min(q1.x, q2.x) <= max(p1.x, p2.x) &&
            min(p1.y, p2.y) <= max(q1.y, q2.y) &&
            min(q1.y, q2.y) <= max(p1.y, p2.y);
        return ret;
    }

    //跨立判断
    bool IsLineSegmentCross(const Point& P1, const Point& P2, const Point& Q1, const Point& Q2)
    {
        if (
            ((Q1.x - P1.x) * (Q1.y - Q2.y) - (Q1.y - P1.y) * (Q1.x - Q2.x)) * ((Q1.x - P2.x) * (Q1.y - Q2.y) - (Q1.y - P2.y) * (Q1.x - Q2.x)) < 0 ||
            ((P1.x - Q1.x) * (P1.y - P2.y) - (P1.y - Q1.y) * (P1.x - P2.x)) * ((P1.x - Q2.x) * (P1.y - P2.y) - (P1.y - Q2.y) * (P1.x - P2.x)) < 0
            //(P1-Q1).cross(Q2-Q1)*(Q2-Q1).cross(P2-P1)>0&&
            //(Q1-P1).cross(P2-P1)*(P2-P1).cross(Q2-P1)>0
            )
            return true;
        else
            return false;
    }

    bool GetCrossPoint(const Point& p1, const Point& p2, const Point& q1, const Point& q2, float& x, float& y)
    {
        if (IsRectCross(p1, p2, q1, q2))
        {
            if (IsLineSegmentCross(p1, p2, q1, q2))
            {
                //求交点
                long tmpLeft, tmpRight;
                tmpLeft = (q2.x - q1.x) * (p1.y - p2.y) - (p2.x - p1.x) * (q1.y - q2.y);
                tmpRight = (p1.y - q1.y) * (p2.x - p1.x) * (q2.x - q1.x) + q1.x * (q2.y - q1.y) * (p2.x - p1.x) - p1.x * (p2.y - p1.y) * (q2.x - q1.x);

                x = ((double)tmpRight / (double)tmpLeft);

                tmpLeft = (p1.x - p2.x) * (q2.y - q1.y) - (p2.y - p1.y) * (q1.x - q2.x);//和上面的叉乘差负号
                tmpRight = p2.y * (p1.x - p2.x) * (q2.y - q1.y) + (q2.x - p2.x) * (q2.y - q1.y) * (p1.y - p2.y) - q2.y * (q1.x - q2.x) * (p2.y - p1.y);
                y = ((double)tmpRight / (double)tmpLeft);
                return true;
            }
        }
        return false;
    }
    bool GetCrossPoint(const cv::Vec4f l1, const cv::Vec4f l2, Point& p)
    {
        Point p1(l1[0], l1[1]), p2(l1[2], l1[3]);
        Point q1(l2[0], l2[1]), q2(l2[2], l2[3]);
        return GetCrossPoint(p1, p2, q1, q2, p.x, p.y);
    }


    //getCrossPoint 计算两直线交点
    int GetCrossPoint(cv::Point& A, cv::Point& B, cv::Point& C, cv::Point& D, cv::Point& crosspoint)
    {
        double m = (B.y - A.y) * (C.x - D.x) - (D.y - C.y) * (A.x - B.x);
        if (m == 0) {   //平行不相交

            crosspoint.x = 0;
            crosspoint.y = 0;
            return -1;
        }
        else
        {
            crosspoint.x = ((D.x * C.y - C.x * D.y) * (A.x - B.x) - (B.x * A.y - A.x * B.y) * (C.x - D.x)) / m;
            crosspoint.y = ((B.x * A.y - A.x * B.y) * (D.y - C.y) - (D.x * C.y - C.x * D.y) * (B.y - A.y)) / m;
        }
        return 0;
    }
#ifdef _WIN32
    int getCrossPoint(cv::Point2f& A, cv::Point2f& B, cv::Point2f& C, cv::Point2f& D, cv::Point2f& crosspoint)
    {

        {
            //auto AB = B - A;
            //auto CD = D - C;
            //auto m = AB.cross(CD);
            //crosspoint = m == 0 ? cv::Point2f(0, 0) : C.cross(D)/m*AB+A.cross(B)/m*CD;
            //return m!=0;
        }

        double m = (B.y - A.y) * (C.x - D.x) - (D.y - C.y) * (A.x - B.x);
        if (m == 0) {   //平行不相交

            crosspoint.x = 0;
            crosspoint.y = 0;
            return 0;
        }
        else
        {
            crosspoint.x = ((D.x * C.y - C.x * D.y) * (A.x - B.x) - (B.x * A.y - A.x * B.y) * (C.x - D.x)) / m;
            crosspoint.y = ((B.x * A.y - A.x * B.y) * (D.y - C.y) - (D.x * C.y - C.x * D.y) * (B.y - A.y)) / m;
        }
        return 1;
    }

    int getCrossPoint(const cv::Vec4f l1, const cv::Vec4f l2, Point& p) {
        return getCrossPoint(cv::Point2f{ l1[0], l1[1] }, cv::Point2f{ l1[2], l1[3] },
            cv::Point2f{ l2[0], l2[1] }, cv::Point2f{ l2[2], l2[3] }, p);
    }
#endif
    //这个方法斜率会退化，不健壮不建议用
    cv::Point2f getCrossPoint(cv::Vec4i LineA, cv::Vec4i LineB)
    {
        double ka, kb;
        ka = (double)(LineA[3] - LineA[1]) / (double)(LineA[2] - LineA[0]); //求出LineA斜率
        kb = (double)(LineB[3] - LineB[1]) / (double)(LineB[2] - LineB[0]); //求出LineB斜率

        cv::Point2f crossPoint;
        crossPoint.x = (ka * LineA[0] - LineA[1] - kb * LineB[0] + LineB[1]) / (ka - kb);
        crossPoint.y = (ka * kb * (LineA[0] - LineB[0]) + ka * LineB[1] - kb * LineA[1]) / (ka - kb);
        return crossPoint;
    }

    auto getCrossPoint(Eigen::Vector2f& p1, Eigen::Vector2f& p2, Eigen::Vector2f& q1, Eigen::Vector2f& q2) {
        auto v1 = p2 - p1;
        auto v2 = q2 - q1;

        Eigen::Vector2f n1(-v1(1), v1(0));
        Eigen::Vector2f n2(-v2(1), v2(0));

        Eigen::Matrix2f A;
        A << n1, n2;
        Eigen::Vector2f b(n1.dot(p1), n2.dot(q1));
        //std::cout << b << std::endl;

        auto res = A.inverse() * b;

        //auto res = A.lu().solve(b);
        auto res_partialPivLu = A.partialPivLu().solve(b);
        auto res_fullPivLu = A.fullPivLu().solve(b);
        auto res_fullPivHouseholderQr = A.fullPivHouseholderQr().solve(b);
        auto res_llt = A.llt().solve(b);
        A.ldlt().solve(b);
        A.bdcSvd().solve(b);
        A.jacobiSvd().solve(b);
        A.householderQr().solve(b);
        //cv::PCA pca_analysis();
        //auto res = A.colPivHouseholderQr().solve(b);

        return res;
    }

    auto Intersect(cv::Vec4f& line1, cv::Vec4f& line2, Point& res) {
        Point p1(line1[0], line1[1]), p2(line1[2], line1[3]);
        Point q1(line2[0], line2[1]), q2(line2[2], line2[3]);

        double m = (p2.y - p1.y) * (q1.x - q2.x) - (q2.y - q1.y) * (p1.x - p2.x);
        if (m == 0) {   //平行不相交

            res.x = 0;
            res.y = 0;
        }
        else
        {
            res.x = ((q2.x * q1.y - q1.x * q2.y) * (p1.x - p2.x) - (p2.x * p1.y - p1.x * p2.y) * (q1.x - q2.x)) / m;
            res.y = ((p2.x * p1.y - p1.x * p2.y) * (q2.y - q1.y) - (q2.x * q1.y - q1.x * q2.y) * (p2.y - p1.y)) / m;
        }

        return IsRectCross(p1, p2, q1, q2);
    }

};

