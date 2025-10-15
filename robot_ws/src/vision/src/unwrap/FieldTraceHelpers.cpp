#include "vision/unwrap/FieldTraceHelpers.hpp"
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <queue>
#include <algorithm>
#include <vector>
#include <utility>
#include <functional>

using namespace cv;
using namespace std;

// --- Preprocessing ---
void stepPreprocess(
    const Mat &input,
    Mat &gray,
    Mat &binary,
    Mat &filtered)
{
    cvtColor(input, gray, COLOR_BGR2GRAY);
    adaptiveThreshold(
      gray, binary,
      255,
      ADAPTIVE_THRESH_MEAN_C,
      THRESH_BINARY,
      WINDOW_SIZE,
      PERCENTAGE
    );

    Mat eroded, recovered;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3,3));
    erode(binary, eroded, kernel, Point(-1,-1), ERODE_DILATE_ITER);
    dilate(eroded, recovered, kernel, Point(-1,-1), ERODE_DILATE_ITER);
    subtract(binary, recovered, filtered);
}

// --- Edge detection & probabilistic Hough ---
void stepEdgesAndHough(
    const Mat &binary,
    Mat &edge_img,
    vector<Vec4i> &raw_lines)
{
    Canny(binary, edge_img,
          CANNY_THRESH1,
          CANNY_THRESH2,
          3);
    HoughLinesP(
      binary,
      raw_lines,
      HOUGH_RHO,
      CV_PI/180.0,
      HOUGH_THRESH,
      HOUGH_MIN_LINE_LEN,
      HOUGH_MAX_LINE_GAP
    );
}

// --- Merge + cluster removal ---
vector<pair<Point,Point>> tooManyLines(
    const vector<Vec4i> &raw_lines)
{
    vector<pair<Point2f,Point2f>> segments;
    for (auto &l : raw_lines) {
        segments.emplace_back(
            Point2f((float)l[0], (float)l[1]),
            Point2f((float)l[2], (float)l[3])
        );
    }
    segments = iterativeMerge(segments,
        [](auto &A, auto &B){
            return areLinesClose(A,B);
        });
    segments = removeLargeClusters(segments);

    vector<pair<Point,Point>> output;
    output.reserve(segments.size());
    for (auto &s : segments) {
        output.emplace_back(
            Point(cvRound(s.first.x),  cvRound(s.first.y)),
            Point(cvRound(s.second.x), cvRound(s.second.y))
        );
    }
    return output;
}

// --- Filter “blue‐condition” lines and draw red ---
vector<pair<Point,Point>> stepFilterRedLines(
    const vector<pair<Point,Point>> &finalLines,
    int frameWidth,
    int frameHeight,
    Mat &drawing)
{
    const int leftMiddleY = frameHeight/2;
    const int thresholdPx  = 45;
    vector<pair<Point,Point>> red_lines;

    for (auto &L : finalLines) {
        auto ext = extendLine(L, frameWidth, frameHeight);
        bool skip = false;
        // skip if intersects left middle
        if (ext.first.x==0 || ext.second.x==0) {
            int y_int = (ext.first.x==0)
                        ? ext.first.y
                        : ext.second.y;
            if (abs(y_int - leftMiddleY) < thresholdPx)
                skip = true;
        }
        else if (abs(L.second.x - L.first.x) > 1e-6f) {
            float t = -float(L.first.x)
                      / (L.second.x - L.first.x);
            if (t>=0 && t<=1) {
                int y_int = int(
                  std::round(L.first.y + t*(L.second.y - L.first.y))
                );
                if (abs(y_int - leftMiddleY) < thresholdPx)
                    skip = true;
            }
        }
        if (!skip) {
            red_lines.push_back(L);
            line(drawing,
                 L.first, L.second,
                 Scalar(0,0,255), 2);
        }
    }
    return red_lines;
}

// --- Circle detection + intersections ---
void stepCircleDetection(
    Mat &drawing,
    const Mat &gray,
    const vector<pair<Point,Point>> &red_lines)
{
    vector<Vec3f> circles;
    Mat sm;
    GaussianBlur(gray, sm, Size(9,9), 2, 2);
    HoughCircles(
      sm, circles,
      HOUGH_GRADIENT,
      1.2,
      gray.rows/8,
      50, 55,
      40, 60
    );
    if (!circles.empty()) {
        Vec3f c = circles[0];
        Point center(
          int(std::round(c[0])),
          int(std::round(c[1]))
        );
        int radius = int(std::round(c[2]));
        circle(drawing, center, radius,
               Scalar(255,0,0), 2);
        circle(drawing, center, 3,
               Scalar(255,0,0), -1);

        for (auto &L : red_lines) {
            auto pts = getCircleLineIntersections(
                L.first, L.second,
                center, float(radius)
            );
            if (pts.size()==1) {
                Point2f mpt = Point2f(
                  2*center.x, 2*center.y
                ) - pts[0];
                pts.push_back(mpt);
            }
            for (auto &pt : pts) {
                circle(drawing, pt, 5,
                       Scalar(0,255,255), -1);
            }
        }
    }
}

// --- Line‐line intersection marking ---
void stepIntersections(
    Mat &drawing,
    const vector<pair<Point,Point>> &red_lines)
{
    for (size_t i=0; i<red_lines.size(); ++i) {
        for (size_t j=i+1; j<red_lines.size(); ++j) {
            Point2f p;
            if (getLineIntersection(red_lines[i],
                                     red_lines[j],
                                     p))
            {
                if (isPointOnExtendedSegment(p, red_lines[i]) &&
                    isPointOnExtendedSegment(p, red_lines[j]))
                {
                    if (p.x>=0 && p.y>=0 &&
                        p.x<drawing.cols &&
                        p.y<drawing.rows)
                    {
                        circle(drawing, p, 4,
                               Scalar(0,255,0), -1);
                    }
                }
            }
        }
    }
}

// --- Helpers ---

vector<Point2f> getCircleLineIntersections(
    const Point &p1,
    const Point &p2,
    const Point &center,
    float r)
{
    vector<Point2f> inters;
    Point2f P1(p1), P2(p2), C(center);
    Point2f d = P2 - P1;
    Point2f f = P1 - C;
    float a = d.dot(d);
    float b = 2*f.dot(d);
    float c = f.dot(f) - r*r;
    float disc = b*b - 4*a*c;
    if (disc >= 0) {
        disc = sqrt(disc);
        float t1 = (-b - disc)/(2*a);
        float t2 = (-b + disc)/(2*a);
        if (t1>=0 && t1<=1) inters.push_back(P1 + d*t1);
        if (t2>=0 && t2<=1 && disc>1e-3f) inters.push_back(P1 + d*t2);
    }
    return inters;
}

bool getLineIntersection(
    const pair<Point,Point> &l1,
    const pair<Point,Point> &l2,
    Point2f &out)
{
    Point p1 = l1.first, p2 = l1.second;
    Point p3 = l2.first, p4 = l2.second;
    float denom =
      (p1.x-p2.x)*(p3.y-p4.y)
    - (p1.y-p2.y)*(p3.x-p4.x);
    if (fabs(denom) < 1e-6f) return false;
    float x = ((p1.x*(float)p2.y - p1.y*(float)p2.x)*(p3.x-p4.x)
             - (p1.x-p2.x)*(p3.x*(float)p4.y - p3.y*(float)p4.x))
            / denom;
    float y = ((p1.x*(float)p2.y - p1.y*(float)p2.x)*(p3.y-p4.y)
             - (p1.y-p2.y)*(p3.x*(float)p4.y - p3.y*(float)p4.x))
            / denom;
    out = Point2f(x,y);
    return true;
}

bool isPointOnExtendedSegment(
    const Point2f &p,
    const pair<Point,Point> &seg,
    float tol,
    float extension)
{
    Point2f A(seg.first), B(seg.second);
    Point2f AB = B - A;
    float len = norm(AB);
    if (len < 1e-6f) return false;
    float t = (p - A).dot(AB) / (len*len);
    float extFrac = extension / len;
    if (t < -extFrac || t > 1.0f + extFrac) return false;
    Point2f proj = A + AB*t;
    return (norm(p - proj) <= tol);
}

pair<Point,Point> extendLine(
    const pair<Point,Point> &line,
    int width,
    int height)
{
    Point p1 = line.first, p2 = line.second;
    Point2f d(p2.x-p1.x, p2.y-p1.y);
    if (fabs(d.x)<1e-6f && fabs(d.y)<1e-6f)
        return line;

    vector<Point> pts;
    // x = 0
    if (fabs(d.x)>1e-6f) {
        float t = (0 - p1.x) / d.x;
        Point pt(
          int(std::round(p1.x + d.x*t)),
          int(std::round(p1.y + d.y*t))
        );
        if (pt.y>=0 && pt.y<height) pts.push_back(pt);
    }
    // x = width-1
    if (fabs(d.x)>1e-6f) {
        float t = ((width-1) - p1.x) / d.x;
        Point pt(
          int(std::round(p1.x + d.x*t)),
          int(std::round(p1.y + d.y*t))
        );
        if (pt.y>=0 && pt.y<height) pts.push_back(pt);
    }
    // y = 0
    if (fabs(d.y)>1e-6f) {
        float t = (0 - p1.y) / d.y;
        Point pt(
          int(std::round(p1.x + d.x*t)),
          int(std::round(p1.y + d.y*t))
        );
        if (pt.x>=0 && pt.x<width) pts.push_back(pt);
    }
    // y = height-1
    if (fabs(d.y)>1e-6f) {
        float t = ((height-1) - p1.y) / d.y;
        Point pt(
          int(std::round(p1.x + d.x*t)),
          int(std::round(p1.y + d.y*t))
        );
        if (pt.x>=0 && pt.x<width) pts.push_back(pt);
    }
    if (pts.size()<2) return line;

    double maxD = 0;
    pair<Point,Point> best;
    for (size_t i=0; i<pts.size(); ++i) {
        for (size_t j=i+1; j<pts.size(); ++j) {
            double dist = norm(pts[i] - pts[j]);
            if (dist > maxD) {
                maxD = dist;
                best = { pts[i], pts[j] };
            }
        }
    }
    return best;
}

float getLineOrientation(
    const pair<Point2f,Point2f> &L)
{
    Point2f d = L.second - L.first;
    float ang = atan2(d.y, d.x);
    if (ang < 0) ang += (float)CV_PI;
    return ang;
}

vector<pair<Point2f,Point2f>> iterativeMerge(
    vector<pair<Point2f,Point2f>> segments,
    function<bool(const pair<Point2f,Point2f>&,
                  const pair<Point2f,Point2f>&)> canMerge)
{
    bool changed = true;
    while (changed) {
        changed = false;
        for (size_t i=0; i<segments.size(); ++i) {
            for (size_t j=i+1; j<segments.size(); ++j) {
                if (canMerge(segments[i], segments[j])) {
                    auto m = mergeTwoLines(segments[i], segments[j]);
                    segments.erase(segments.begin()+j);
                    segments.erase(segments.begin()+i);
                    segments.push_back(m);
                    changed = true;
                    break;
                }
            }
            if (changed) break;
        }
    }
    return segments;
}

vector<pair<Point2f,Point2f>> removeLargeClusters(
    const vector<pair<Point2f,Point2f>> &lines)
{
    size_t N = lines.size();
    vector<vector<int>> adj(N);
    for (size_t i=0; i<N; ++i) {
        for (size_t j=i+1; j<N; ++j) {
            if (areLinesClose(lines[i], lines[j])) {
                adj[i].push_back(j);
                adj[j].push_back(i);
            }
        }
    }
    vector<bool> visited(N,false);
    vector<pair<Point2f,Point2f>> result;
    const size_t MAX_CLUSTER = MAX_CLUSTER_SIZE;
    queue<int> q;

    for (size_t i=0; i<N; ++i) {
        if (!visited[i]) {
            vector<int> cluster;
            visited[i] = true;
            q.push(i);
            while (!q.empty()) {
                int u = q.front(); q.pop();
                cluster.push_back(u);
                for (int v: adj[u]) {
                    if (!visited[v]) {
                        visited[v] = true;
                        q.push(v);
                    }
                }
            }
            if (cluster.size() == 1) {
                result.push_back(lines[cluster[0]]);
            }
            else if (cluster.size() <= MAX_CLUSTER) {
                result.push_back(mergeCluster(cluster, lines));
            }
        }
    }
    return result;
}

bool areLinesClose(
    const pair<Point2f,Point2f> &A,
    const pair<Point2f,Point2f> &B)
{
    float thr = ORIENT_THRESH_DEG * CV_PI/180.0f;
    float oA = getLineOrientation(A);
    float oB = getLineOrientation(B);
    float diff = fabs(oA - oB);
    diff = min(diff, (float)CV_PI - diff);
    if (diff > thr) return false;

    Point2f a1 = A.first, a2 = A.second;
    Point2f dir = a2 - a1;
    float len = norm(dir);
    if (len < 1e-6f) return false;
    dir /= len;

    auto perpDist = [&](const Point2f &p){
        Point2f dp = p - a1;
        float proj = dp.dot(dir);
        Point2f pp = a1 + dir*proj;
        return norm(p - pp);
    };

    Point2f midB = (B.first + B.second)*0.5f;
    if (perpDist(B.first) > DIST_CLOSE) return false;
    if (perpDist(midB)    > DIST_CLOSE) return false;
    if (perpDist(B.second)> DIST_CLOSE) return false;
    return true;
}

pair<Point2f,Point2f> mergeTwoLines(
    const pair<Point2f,Point2f> &A,
    const pair<Point2f,Point2f> &B)
{
    vector<Point2f> pts = { A.first, A.second, B.first, B.second };
    Vec4f fit;
    fitLine(pts, fit, DIST_L2, 0, 0.01, 0.01);
    Point2f dir(fit[0], fit[1]), p0(fit[2], fit[3]);
    vector<float> tvals;
    for (auto &p: pts) {
        tvals.push_back((p.x - p0.x)*dir.x + (p.y - p0.y)*dir.y);
    }
    float mn = *min_element(tvals.begin(), tvals.end());
    float mx = *max_element(tvals.begin(), tvals.end());
    return { p0 + dir*mn, p0 + dir*mx };
}

pair<Point2f,Point2f> mergeCluster(
    const vector<int> &cluster,
    const vector<pair<Point2f,Point2f>> &lines)
{
    vector<Point2f> pts;
    for (int idx: cluster) {
        pts.push_back(lines[idx].first);
        pts.push_back(lines[idx].second);
    }
    Vec4f fit;
    fitLine(pts, fit, DIST_L2, 0, 0.01, 0.01);
    Point2f dir(fit[0], fit[1]), p0(fit[2], fit[3]);
    vector<float> tvals;
    for (auto &p: pts) {
        tvals.push_back((p.x - p0.x)*dir.x + (p.y - p0.y)*dir.y);
    }
    float mn = *min_element(tvals.begin(), tvals.end());
    float mx = *max_element(tvals.begin(), tvals.end());
    return { p0 + dir*mn, p0 + dir*mx };
}
