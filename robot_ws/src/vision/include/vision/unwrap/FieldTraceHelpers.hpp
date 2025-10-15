#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <vector>

// Preprocessing: grayscale, adaptive threshold, noise removal
enum PreprocessParams { WINDOW_SIZE = 101, PERCENTAGE = -40, ERODE_DILATE_ITER = 4 };
void stepPreprocess(const cv::Mat &input,
                    cv::Mat &gray,
                    cv::Mat &binary,
                    cv::Mat &filtered);

// Edge detection and probabilistic Hough lines
enum HoughParams { CANNY_THRESH1 = 50, CANNY_THRESH2 = 150,
                   HOUGH_RHO = 1, HOUGH_THRESH = 50,
                   HOUGH_MIN_LINE_LEN = 60, HOUGH_MAX_LINE_GAP = 5 };
void stepEdgesAndHough(const cv::Mat &binary,
                       cv::Mat &edge_img,
                       std::vector<cv::Vec4i> &raw_lines);

// Merge + cluster removal
enum MergeParams { ORIENT_THRESH_DEG = 10, DIST_CLOSE = 10,
                   MAX_CLUSTER_SIZE = 1 };
std::vector<std::pair<cv::Point, cv::Point>> tooManyLines(
    const std::vector<cv::Vec4i> &raw_lines);

// Filter “blue‐condition” lines and draw them in red
std::vector<std::pair<cv::Point, cv::Point>> stepFilterRedLines(
    const std::vector<std::pair<cv::Point, cv::Point>> &lines,
    int frameWidth,
    int frameHeight,
    cv::Mat &drawing);

// Circle detection & line‐circle intersections
void stepCircleDetection(
    cv::Mat &drawing,
    const cv::Mat &gray,
    const std::vector<std::pair<cv::Point, cv::Point>> &red_lines);

// Line‐line intersections
void stepIntersections(
    cv::Mat &drawing,
    const std::vector<std::pair<cv::Point, cv::Point>> &red_lines);

// Helper functions
std::vector<cv::Point2f> getCircleLineIntersections(
    const cv::Point &p1,
    const cv::Point &p2,
    const cv::Point &center,
    float radius);

bool getLineIntersection(
    const std::pair<cv::Point, cv::Point> &l1,
    const std::pair<cv::Point, cv::Point> &l2,
    cv::Point2f &intersection);

bool isPointOnExtendedSegment(
    const cv::Point2f &p,
    const std::pair<cv::Point, cv::Point> &seg,
    float tol = 2.0f,
    float extension = 20.0f);

std::pair<cv::Point, cv::Point> extendLine(
    const std::pair<cv::Point, cv::Point> &line,
    int width,
    int height);

// Merge helpers
float getLineOrientation(
    const std::pair<cv::Point2f, cv::Point2f> &L);

std::vector<std::pair<cv::Point2f, cv::Point2f>> iterativeMerge(
    std::vector<std::pair<cv::Point2f, cv::Point2f>> segments,
    std::function<bool(const std::pair<cv::Point2f, cv::Point2f>&,
                       const std::pair<cv::Point2f, cv::Point2f>&)> canMerge);

std::vector<std::pair<cv::Point2f, cv::Point2f>> removeLargeClusters(
    const std::vector<std::pair<cv::Point2f, cv::Point2f>> &lines);

bool areLinesClose(
    const std::pair<cv::Point2f, cv::Point2f> &A,
    const std::pair<cv::Point2f, cv::Point2f> &B);

std::pair<cv::Point2f, cv::Point2f> mergeTwoLines(
    const std::pair<cv::Point2f, cv::Point2f> &A,
    const std::pair<cv::Point2f, cv::Point2f> &B);

std::pair<cv::Point2f, cv::Point2f> mergeCluster(
    const std::vector<int> &cluster,
    const std::vector<std::pair<cv::Point2f, cv::Point2f>> &lines);
