#pragma once
#include "../Irgbd.h"
#include <iostream>
#include <string>
#include "opencv2/core/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
using namespace std;
using namespace cv;
#define M_PI       3.14159265358979323846   // pi
class THandleImgAndEdge :
    public Irgbd
{
    using line_type = cv::Vec4f;
    std::vector<line_type> m_lines_d;
public:
    cv::Mat m_img;
    cv::Mat m_img_debug;

    cv::Mat m_img_point;
    map<tuple<int, int, int>, pair<Point3f, pair<float, float>>> m_pixels;
    map<pair<int, int>, pair<Point3f, int>> m_pixels_;


    THandleImgAndEdge(cv::Mat& img) :m_img(img.clone()), m_img_debug(img), m_img_point(img.size(), CV_8UC1)
    {
        //GetLineHough(img);
        //GetLineLsd(img);
    }



    auto Assessment(cv::Vec4f& line1, cv::Vec4f& line2, float _lnsm) {
        cv::Point2f l1p1(line1[0], line1[1]);
        cv::Point2f l1p2(line1[2], line1[3]);
        cv::Point2f l2p1(line2[0], line2[1]);
        cv::Point2f l2p2(line2[2], line2[3]);

        auto l1_centre = (l1p1 + l1p2) / 2;
        auto l2_centre = (l2p1 + l2p2) / 2;
        auto l1_vec = l1p1 - l1p2;
        auto l2_vec = l2p1 - l2p2;
        auto l1_len = norm(l1_vec);
        auto l2_len = norm(l2_vec);
        l1_vec /= l1_len;
        l2_vec /= l2_len;
        //Proportional is more similar
        float lnsm = min(l1_len, l2_len) / max(l1_len, l2_len);

        auto l1_l2_centre = l1_centre - l2_centre;
        auto l1_l2_len = norm(l1_l2_centre);
        l1_l2_centre /= l1_l2_len;

        //auto len = l1_l2_len*sin(abs(norm(l1_l2_centre.cross(l1_vec))));
        auto len_vertical = l1_l2_len * abs(l1_l2_centre.cross(l1_vec));
        auto len_level = l1_l2_len * abs(l1_l2_centre.dot(l1_vec));
        float psm = max((1.f - (len_vertical) / (l1_len / 2)), 0.f);

        float psm_level = max((1.f - (len_level) / (l1_len / 2)), 0.f);

        auto ang = acos(l1_vec.dot(l2_vec));
        ang = ang > (M_PI / 2) ? M_PI - ang : ang;
        float osm = max((1 - ang / (M_PI / 2)), 0);

        //std::cout << l1_l2_centre << std::endl;
        //std::cout  << " " << psm << std::endl;
        //std::cout << ang << " " << osm << std::endl;
        float omega_lnsm = 0.1, omega_psm = 0.8, omega_osm = 0.8;
        return (omega_lnsm * lnsm + omega_psm * psm + omega_osm * osm + _lnsm * psm_level) /
            (omega_lnsm + omega_psm + omega_osm + _lnsm);
    }

    auto Assessment(cv::Point2f l1p1, cv::Point2f l1p2, cv::Point2f l2p1, cv::Point2f l2p2,
        float omega_lnsm = 0.0, float omega_psm_level = 0.05, float omega_psm = 1.2, float omega_osm = 1.6) {

        auto l1_centre = (l1p1 + l1p2) / 2;
        auto l2_centre = (l2p1 + l2p2) / 2;
        auto l1_vec = l1p1 - l1p2;
        auto l2_vec = l2p1 - l2p2;
        auto l1_len = norm(l1_vec);
        auto l2_len = norm(l2_vec);
        l1_vec /= l1_len;
        l2_vec /= l2_len;
        //Proportional is more similar
        float lnsm = min(l1_len, l2_len) / max(l1_len, l2_len);

        auto l1_l2_centre = l1_centre - l2_centre;
        auto l1_l2_len = norm(l1_l2_centre);
        l1_l2_centre /= l1_l2_len;

        //auto len = l1_l2_len*sin(abs(norm(l1_l2_centre.cross(l1_vec))));

        auto p1_p1 = l1p1 - l2p1;
        auto p1_p2 = l1p1 - l2p2;
        auto l1_l2_len_p1 = norm(p1_p1);
        auto l1_l2_len_p2 = norm(p1_p2);
        p1_p1 /= l1_l2_len_p1;
        p1_p2 /= l1_l2_len_p2;
        auto l1_l2_len_p = (l1_l2_len_p1 < l1_l2_len_p2) ? p1_p1 : p1_p2;
        auto l1_l2_centre_p1 = min(l1_l2_len_p1, l1_l2_len_p2);

        auto len_vertical = l1_l2_centre_p1 * abs(l1_l2_len_p.cross(l1_vec));
        auto len_level = l1_l2_centre_p1 * abs(l1_l2_len_p.dot(l1_vec));
        float psm = max((1.f - (len_vertical) / (l1_len)), 0.f);

        float psm_level = max((1.f - (len_level) / (l1_len)), 0.f);

        auto ang = acos(l1_vec.dot(l2_vec));
        ang = ang > (M_PI / 2) ? M_PI - ang : ang;
        float osm = max((1 - ang / (M_PI / 2)), 0);

        //std::cout << l1_l2_centre << std::endl;
        //std::cout  << " " << psm << std::endl;
        //std::cout << ang << " " << osm << std::endl;
        return (omega_lnsm * lnsm + omega_psm * psm + omega_psm_level * psm_level + omega_osm * osm) /
            (omega_lnsm + omega_psm + omega_osm + omega_psm_level);
    }

    auto Assessment(cv::Vec4f& line1, cv::Vec4f& line2) {

        cv::Point2f l1p1(line1[0], line1[1]);
        cv::Point2f l1p2(line1[2], line1[3]);
        cv::Point2f l2p1(line2[0], line2[1]);
        cv::Point2f l2p2(line2[2], line2[3]);
        return Assessment(l1p1, l1p2, l2p1, l2p2);
    }
    auto GetLineHough(cv::Mat& img) {
        cv::Mat image_edge;
        DetectEdge(img, image_edge);

        cv::imwrite("rgb.png", img);
        cv::imwrite("image_edge.png", image_edge);

        HoughLineP(image_edge, m_lines_d);
        Draw(m_img, m_lines_d, cv::Scalar(255, 0, 0));
    }

    auto GetLineLsd(cv::Mat& img) {
#if 1
        cv::Mat image;
        Canny(img, image, 50, 200, 3); // Apply canny edge
#endif
        // Create and LSD detector with standard or no refinement.
#if 1
        Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_STD);
#else
        Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_NONE);
#endif
        double start = double(getTickCount());
        vector<Vec4f> lines_std;
        // Detect the lines
        ls->detect(image, m_lines_d);
        double duration_ms = (double(getTickCount()) - start) * 1000 / getTickFrequency();
        std::cout << "It took " << duration_ms << " ms." << std::endl;
        // Show found lines
        Mat drawnLines(img);
        ls->drawSegments(drawnLines, lines_std);
        imshow("Standard refinement", drawnLines);
        waitKey();
    }

    auto iteration() {
        Mat src = Mat(1000, 1000, CV_8UC3, Scalar(0, 0, 0));


        Point a;
        vector<Point> Points;
        Points.clear();
        // 生成点图
        for (int i = 0; i < 100; i++)
        {
            // 400 —— 700的随机数
            int temp = rand() % 300;
            a.x = temp + 400 + rand() % 10;
            a.y = temp + 400 + rand() % 20;
            circle(src, Point(a.x, a.y), 1, Scalar(255, 255, 255));
            Points.push_back(a);
        }

        for (int i = 0; i < 100; i++)
        {
            // 400 —— 700的随机数
            a.x = rand() % 300 + 400;
            a.y = rand() % 300 + 400;
            circle(src, Point(a.x, a.y), 1, Scalar(255, 255, 255));
            Points.push_back(a);
        }

        imshow("src", src);

        float tolerance = 10.0; // 容差
        int best_PointSize = 0;
        int PointSize = 0;
        double k, b; // 直线方程参数
        double best_k, best_b; // 最终的直线方程参数
        double dis; // 点到直线距离
        for (int num1 = 0; num1 < Points.size() - 1 - 1; num1++)
        {
            for (int num2 = num1 + 1; num2 < Points.size() - 1; num2++)
            {
                if (abs(Points[num2].x - Points[num1].x) < 0.001)
                {
                    k = -1;
                    b = -1;
                    for (int n = 0; n < Points.size(); n++)
                    {
                        dis = abs(Points[n].x - Points[num2].x);
                        if (dis < tolerance)
                            PointSize++; // 内点++
                    }
                }
                else
                {
                    k = (Points[num2].y - Points[num1].y) / (Points[num2].x - Points[num1].x);
                    b = k * Points[num2].x - Points[num2].y;
                    for (int n = 0; n < Points.size(); n++)
                    {
                        dis = abs(k * Points[n].x - Points[n].y + b) / sqrt(1 + k * k);
                        if (dis < tolerance)
                            PointSize++; // 内点++
                    }
                }

                if (PointSize > best_PointSize)
                {
                    best_PointSize = PointSize;
                    best_k = k;
                    best_b = b;
                }

                if (best_PointSize > 40)
                    break;
            }

            if (best_PointSize > 40)
                break;
        }
        Point Point_start;
        Point Point_end;
        Point_start.x = 0;
        Point_start.y = best_b;
        Point_end.x = 1000;
        Point_end.y = 1000 * best_k + best_b;

        line(src, Point_start, Point_end, Scalar(255, 255, 255), 3);
        imshow("src1", src);

        waitKey(0);
    }

    vector<Mat> m_img_rois;
    auto iteration(line_type& line) {

        cv::Point2f l1p1(line[0], line[1]);
        cv::Point2f l1p2(line[2], line[3]);
        auto centre = (l1p1 + l1p2) / 2;
        auto l1_vec = l1p2 - l1p1;
        auto l1_len = norm(l1_vec);
        l1_vec /= l1_len;

        //auto rot = getRotationMatrix2D();
        Point2f vec_right(l1_vec.y, -l1_vec.x);
        Point2f vec_left(-l1_vec.y, l1_vec.x);

        float range = 30;
        //cout << l1p1 + (range * vec_left) + (l1_len * l1_vec);
        //cout << l1p2+(range * vec_left);
        //Vec2f vecs[2];
        //const Point2f& _point1= l1p1 + (range * vec_left), _point2= l1p1, _point3= l1p1 + (range * l1_vec);
        //vecs[0] = Vec2f(_point1 - _point2);
        //vecs[1] = Vec2f(_point2 - _point3);
        //cout << abs(vecs[0].dot(vecs[1])) / (norm(vecs[0]) * norm(vecs[1]));
        //RotatedRect rotRect(l1p1 + (range * vec_left), l1p1, l1p1 + (range * l1_vec));
        vector<cv::Point> contour{
            l1p1 + (range * vec_right), l1p1 + (range * vec_left), l1p2 + (range * vec_left), l1p2 + (range * vec_right) };
        RotatedRect rotRect = minAreaRect(contour);

        auto roi = rotRect.boundingRect();
        roi = roi & Rect(Point2f(), m_img.size());
        auto img_roi = m_img(roi);
        if (img_roi.size().area() < 10)
            return;
        vector<int> hierarchy({ 1,2,4 });

        for (auto y : hierarchy)
        {

            Mat img_hierarchy;
            resize(img_roi, img_hierarchy, img_roi.size() / y);

            cv::Mat image_edge;
            DetectEdge(img_hierarchy, image_edge);

            std::vector<line_type> lines;
            //cv::HoughLinesP(image_edge, lines, 1, CV_PI / 180, 80, 100, 10);
            cv::HoughLinesP(image_edge, lines, 1, CV_PI / 180, 80, 100 / y, 10 / y);

            for (auto& x : lines)
            {
                x *= y;
                auto incontour = pointPolygonTest(contour, Point(x[0] + roi.x, x[1] + roi.y), false);
                if (incontour > 0) {
                    x = line_type(x[0] + roi.x, x[1] + roi.y, x[2] + roi.x, x[3] + roi.y);
                    m_lines_d.push_back(x);
                }

            }

        }
#ifdef DEBUG_
        Draw(m_img_debug, m_lines_d, cv::Scalar(255, 0, 0), 1);
#endif
        //polylines(m_img_debug,contour,true,cv::Scalar(255, 0, 255));
        m_img_rois.emplace_back(img_roi);
    }
    map<float, vector<pair<line_type, float>> > m_line_score;
    auto GetClosestPointByCorner(std::vector<line_type>& point) {
        vector<pair<line_type, float>> line_score;
        for (auto& x : point)
        {
            iteration(x);
            float min_assessment = 0.98;
            line_type tmp = x;
            for (size_t j = 0; j < m_lines_d.size(); j++)
            {
                float line_Similarity = Assessment(tmp, m_lines_d[j]);
                m_line_score[tmp[0]].push_back({ m_lines_d[j],line_Similarity });
                if (line_Similarity > min_assessment)
                {
                    min_assessment = line_Similarity;
                    x = m_lines_d[j];
                }

            }
        }
        Draw(m_img_debug, point, cv::Scalar(0, 255, 0));

        Point p;
#ifdef _WIN32
        getCrossPoint(point[0], point[1], p);
#else
        cv::Point2f A = cv::Point2f{ point[0][0], point[0][1] };
        cv::Point2f B = cv::Point2f{ point[0][2], point[0][3] };
        cv::Point2f C = cv::Point2f{ point[1][0], point[1][1] };
        cv::Point2f D = cv::Point2f{ point[1][2], point[1][3] };

        double m = (B.y - A.y) * (C.x - D.x) - (D.y - C.y) * (A.x - B.x);
        if (m == 0) {   //平行不相交

            p.x = 0;
            p.y = 0;
        }
        else
        {
            p.x = ((D.x * C.y - C.x * D.y) * (A.x - B.x) - (B.x * A.y - A.x * B.y) * (C.x - D.x)) / m;
            p.y = ((B.x * A.y - A.x * B.y) * (D.y - C.y) - (D.x * C.y - C.x * D.y) * (B.y - A.y)) / m;
        }
#endif
        circle(m_img_debug, p, 4, cv::Scalar(0, 0, 255), 4);
        return cv::Point3f(p.x, p.y, 1);
    }
    auto GetClosestPointByCorner(std::vector<line_type>& point, cv::Point3f res) {
        vector<pair<line_type, float>> line_score;
        for (auto& x : point)
        {
            float min_assessment = 0.0;
            for (size_t j = 0; j < m_lines_d.size(); j++)
            {
                float line_Similarity = Assessment(x, m_lines_d[j]);
                if (line_Similarity > min_assessment)
                {
                    min_assessment = line_Similarity;
                    x = m_lines_d[j];
                }

            }
            line_score.push_back({ x, min_assessment });
        }
        Draw(m_img, point, cv::Scalar(0, 255, 0));

        Point p;
#ifdef _WIN32
        getCrossPoint(point[0], point[1], p);
#else
        cv::Point2f A = cv::Point2f{ point[0][0], point[0][1] };
        cv::Point2f B = cv::Point2f{ point[0][2], point[0][3] };
        cv::Point2f C = cv::Point2f{ point[1][0], point[1][1] };
        cv::Point2f D = cv::Point2f{ point[1][2], point[1][3] };

        double m = (B.y - A.y) * (C.x - D.x) - (D.y - C.y) * (A.x - B.x);
        if (m == 0) {   //平行不相交

            p.x = 0;
            p.y = 0;
        }
        else
        {
            p.x = ((D.x * C.y - C.x * D.y) * (A.x - B.x) - (B.x * A.y - A.x * B.y) * (C.x - D.x)) / m;
            p.y = ((B.x * A.y - A.x * B.y) * (D.y - C.y) - (D.x * C.y - C.x * D.y) * (B.y - A.y)) / m;
        }
#endif
        circle(m_img, p, 4, cv::Scalar(0, 0, 255), 4);
        res = { p.x, p.y, 1 };
        return line_score;
    }

    auto GetClosestPointByCorner(line_type line) {

        cv::Point2f l1p1(line[0], line[1]);
        cv::Point2f l1p2(line[2], line[3]);
        auto centre = (l1p1 + l1p2) / 2;
        auto l1_vec = l1p2 - l1p1;
        auto l1_len = norm(l1_vec);
        l1_vec /= l1_len;

        //auto rot = getRotationMatrix2D();
        Point2f vec_right(l1_vec.y, -l1_vec.x);
        Point2f vec_left(-l1_vec.y, l1_vec.x);

        float range = 30;
        RotatedRect rotRect = minAreaRect(vector<Point2f>{
            l1p1 + (range * vec_right), l1p1 + (range * vec_left), l1p2 + (range * vec_left), l1p2 + (range * vec_right)});

        auto roi = rotRect.boundingRect();
        roi = roi & Rect(Point2f(), m_img.size());
        auto img_roi = m_img(roi);
        if (img_roi.size().area() < 10)
            return;
        cv::Mat image_edge;
        DetectEdge(img_roi, image_edge);
        vector<Point> points;
        for (size_t i = 0; i < image_edge.rows; i++)
        {
            for (size_t j = 0; j < image_edge.cols; j++)
            {
                if (image_edge.at<uchar>(i, j) > 0)
                    points.push_back(Point(i, j));
            }
        }

        std::vector<line_type> lines;


        cv::HoughLinesP(image_edge, lines, 1, CV_PI / 180, 80, 100, 10);
        for (auto& x : lines)
        {
            x = line_type(x[0] + roi.x, x[1] + roi.y, x[2] + roi.x, x[3] + roi.y);
            m_lines_d.push_back(x);
        }
        Draw(m_img_debug, lines, cv::Scalar(255, 0, 0));

        imwrite(to_string(roi.x) + "img_roi" + to_string(roi.y) + ".jpg", img_roi);
        imwrite("m_img_debug.jpg", m_img_debug);
    }

    auto test(string path) {
        ////Rect roi(Point(2100,0), Point(3333, 735));
        //Rect roi(2200,0,1800,800);
        //image_edge = image_edge(roi);
        //for (auto& x : lines)
        //{
        //    x = line_type(x[0] + roi.x, x[1] + roi.y, x[2] + roi.x, x[3] + roi.y);
        //}
        //imwrite("m_img_debug.jpg", m_img_debug);
        //
        m_img = imread(path);
        cv::Mat img_roi(m_img);
        vector<int> hierarchy({ 1,2,4 });

        for (auto y : hierarchy)
        {

            Mat img_hierarchy;
            resize(img_roi, img_hierarchy, img_roi.size() / y);

            cv::Mat image_edge;
            DetectEdge(img_hierarchy, image_edge);

            std::vector<line_type> lines;
            cv::HoughLinesP(image_edge, lines, 1, CV_PI / 180, 80 / y, 100 / y, 10 / y);
            for (auto& x : lines)
            {
                x *= y;
                auto incontour = 1;// pointPolygonTest(contour, Point(x[0] + roi.x, x[1] + roi.y), false);
                if (incontour > 0) {
                    //x = line_type(x[0] + roi.x, x[1] + roi.y, x[2] + roi.x, x[3] + roi.y);
                    m_lines_d.push_back(x);
                }

            }

        }
        Draw(m_img, m_lines_d, cv::Scalar(0, 255, 0));
        imwrite("image_edge_.jpg", m_img);

    }

    auto lineMat2Point(line_type line, Mat& planeIndex, map<int, vector<cv::Point>>& collect) {

        cv::Point2f l1p1(line[0], line[1]);
        cv::Point2f l1p2(line[2], line[3]);
        auto centre = (l1p1 + l1p2) / 2;
        auto l1_vec = l1p2 - l1p1;
        auto l1_len = norm(l1_vec);
        l1_vec /= l1_len;

        //auto rot = getRotationMatrix2D();
        Point2f vec_right(l1_vec.y, -l1_vec.x);
        Point2f vec_left(-l1_vec.y, l1_vec.x);

        float range = 30;
        //cout << l1p1 + (range * vec_left) + (l1_len * l1_vec);
        //cout << l1p2+(range * vec_left);
        //Vec2f vecs[2];
        //const Point2f& _point1= l1p1 + (range * vec_left), _point2= l1p1, _point3= l1p1 + (range * l1_vec);
        //vecs[0] = Vec2f(_point1 - _point2);
        //vecs[1] = Vec2f(_point2 - _point3);
        //cout << abs(vecs[0].dot(vecs[1])) / (norm(vecs[0]) * norm(vecs[1]));
        //RotatedRect rotRect(l1p1 + (range * vec_left), l1p1, l1p1 + (range * l1_vec));
        vector<cv::Point> contour{
            l1p1 + (range * vec_right), l1p1 + (range * vec_left), l1p2 + (range * vec_left), l1p2 + (range * vec_right) };
        RotatedRect rotRect = minAreaRect(contour);
        auto roi = rotRect.boundingRect();
        roi = roi & Rect(Point2f(), m_img.size());
        auto planeIndex_roi = planeIndex(roi);

        for (size_t i = 0; i < planeIndex_roi.cols; i++)
        {
            for (size_t j = 0; j < planeIndex_roi.rows; j++)
            {
                auto planeId = planeIndex_roi.at<uchar>(i, j);
                if (planeId < 255)
                    collect[planeId].emplace_back(cv::Point(i, j));
            }
        }
    }
};

