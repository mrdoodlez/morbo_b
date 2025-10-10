#include "vodom.h"
#include "params.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <atomic>
#include <thread>
#include <iostream>

typedef enum
{
    VODOM_NOT_DETECTED = 0,
    VODOM_DETECTED_RED = 1,
    VODOM_DETECTED_GREEN = 2 // future use
} VodomStatus;

typedef struct
{
    uint64_t t_us; // capture timestamp (monotonic/us)
    VodomStatus status;
    float dx_m; // in base_link
    float dy_m;
    float range_m;     // sqrt(dx^2+dy^2)
    float bearing_rad; // atan2(dy,dx)
    float d_px;        // measured diameter in pixels
    float quality;     // 0..1 (or reprojection-err-like)
} VodomResult;

struct VodomInternals
{
    // lifecycle
    std::atomic<bool> run{true};

    // camera
    cv::VideoCapture cap;

    // undistort maps (faster than cv::undistort per frame)
    cv::Mat map1, map2; // CV_16SC2 or CV_32FC1
    bool have_maps = false;

    // ROI tracking
    cv::Rect roi; // empty => full frame
    int lostFrames = 0;
    int consecutiveHits = 0;

    // last result (published atomically)
    std::atomic<VodomResult> latest;
};

static VodomInternals g_vOdomInternals;

static void _Vodom_Worker(VodomInternals &s, const VodomParams &P);

static inline uint64_t nowMonotonicUS()
{
    using namespace std::chrono;
    return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}

/*******************************************************************************/

static bool Vodom_OpenCamera(cv::VideoCapture &cap,
                             const std::string &dev,
                             const VodomParams &p)
{
    // Prefer V4L2 backend (Linux)
#ifdef CV_CAP_V4L2
    if (!cap.open(dev, cv::CAP_V4L2))
        return false;
#else
    if (!cap.open(dev))
        return false;
#endif

    // Set format
    cap.set(cv::CAP_PROP_FRAME_WIDTH, p.W);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, p.H);
    cap.set(cv::CAP_PROP_FPS, p.FPS);
    // Ask MJPEG to keep CPU low (fallback is fine if not supported)
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

    // Try to fix exposure/focus (may be ignored depending on driver/cam)
    // AUTO_EXPOSURE: 0.25 = manual (OpenCV quirk); 0.75 = auto on many builds
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);
    // CAP_PROP_EXPOSURE expects "log2(shutter)" on some cams; tune as needed
    // cap.set(cv::CAP_PROP_EXPOSURE, 0.02); // seconds (may be ignored)

    cap.set(cv::CAP_PROP_AUTOFOCUS, 0);
    // cap.set(cv::CAP_PROP_FOCUS, 120); // 0..255 (C920; may be ignored)

    // Read one frame to verify
    cv::Mat probe;
    if (!cap.read(probe) || probe.empty())
        return false;
    return true;
}

static void Vodom_PrepareUndistortMaps(VodomInternals &S, const VodomParams &P)
{
    if (P.D.empty() || P.K.empty())
    {
        S.have_maps = false;
        return;
    }
    const cv::Size sz(P.W, P.H);
    cv::Mat Knew = cv::getOptimalNewCameraMatrix(P.K, P.D, sz, 0.0);
    cv::initUndistortRectifyMap(P.K, P.D, cv::Mat(), Knew, sz,
                                CV_16SC2, S.map1, S.map2);
    S.have_maps = true;
}

/*******************************************************************************/

int Vodom_Start(const std::string &videoDev)
{
    ParamsView v{};
    if (Controller_GetParams(ParamPage_Vodom, &v) != 0 || v.size != sizeof(VodomParams))
    {
        std::cout << "Failed to load visual odometry parameters" << std::endl;
        return -10;
    }

    auto *vp = static_cast<const VodomParams *>(v.ptr);
    VodomParams params = *vp;

    if (!Vodom_OpenCamera(g_vOdomInternals.cap, videoDev, params))
    {
        std::cout << "Failed to open camera: " << videoDev << "\n";
        g_vOdomInternals.run.store(false, std::memory_order_relaxed);
        return -20;
    }

    // Prepare undistort
    Vodom_PrepareUndistortMaps(g_vOdomInternals, params);

    // Publish an initial empty result
    VodomResult init{};
    init.status = VODOM_NOT_DETECTED;
    init.t_us = 0;
    g_vOdomInternals.latest.store(init, std::memory_order_release);

    std::thread([&]
                { _Vodom_Worker(g_vOdomInternals, params); })
        .detach();

    return 0;
}

/*******************************************************************************/

static void _Vodom_Worker(VodomInternals &s, const VodomParams &P)
{
    cv::Mat frame, und, hsv, maskA, maskB, mask, edges;
    int consecutiveHits = 0;

    uint32_t nFrame = 0;

    // pre-loop
    cv::Mat kOpen, kClose;
    if (P.morph_open > 0)
        kOpen = cv::getStructuringElement(cv::MORPH_ELLIPSE, {P.morph_open, P.morph_open});
    if (P.morph_close > 0)
        kClose = cv::getStructuringElement(cv::MORPH_ELLIPSE, {P.morph_close, P.morph_close});

    while (s.run.load(std::memory_order_relaxed))
    {
        uint64_t t_us = nowMonotonicUS();

        if (!s.cap.read(frame))
            continue;

        if (s.have_maps)
            cv::remap(frame, und, s.map1, s.map2, cv::INTER_LINEAR);
        else
            und = frame;

        if ((nFrame % 30) == 0)
            std::cout << "frame #" << nFrame << "\n";
        ++nFrame;

        cv::Mat view = und;
        if (s.roi.area() > 0)
        {
            s.roi &= cv::Rect(0, 0, und.cols, und.rows);
            view = und(s.roi);
        }

        cv::cvtColor(view, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(P.H1a, P.SminA, P.VminA), cv::Scalar(P.H2a, 255, 255), maskA);
        cv::inRange(hsv, cv::Scalar(P.H1b, P.SminB, P.VminB), cv::Scalar(P.H2b, 255, 255), maskB);
        cv::bitwise_or(maskA, maskB, mask);

        // (optional debug every 30th frame)
        if (1) //(nFrame % 30) == 0)
        {
            std::cout << "roi=" << s.roi << " view=" << view.cols << "x" << view.rows << "\n";
            std::cout << "hsv=" << hsv.cols << "x" << hsv.rows << " type=" << hsv.type() << "\n";
            std::cout << "maskA nnz=" << cv::countNonZero(maskA)
                      << " maskB nnz=" << cv::countNonZero(maskB) << "\n";
        }

        if (mask.type() != CV_8UC1)
            mask.convertTo(mask, CV_8U);

        // Morphology (tmp + swap)
        cv::Mat tmp;
        if (!mask.empty() && !kOpen.empty())
        {
            cv::morphologyEx(mask, tmp, cv::MORPH_OPEN, kOpen, cv::Point(-1, -1), 1, cv::BORDER_CONSTANT, 0);
            std::swap(mask, tmp);
        }
        if (!mask.empty() && !kClose.empty())
        {
            cv::morphologyEx(mask, tmp, cv::MORPH_CLOSE, kClose, cv::Point(-1, -1), 1, cv::BORDER_CONSTANT, 0);
            std::swap(mask, tmp);
        }

        std::cout << "processing 0001\n";

        // Contours → candidate circles
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        struct Cand
        {
            cv::Point2f c;
            float dpx;
            float fill;
            float axisRatio;
            float score;
            cv::RotatedRect ell;
        };
        std::vector<Cand> cands;

        std::cout << "processing 0002\n";

        for (auto &c : contours)
        {
            if (c.size() < 5)
                continue; // fitEllipse needs 5
            auto ell = cv::fitEllipse(c);
            float a = std::max(ell.size.width, ell.size.height) * 0.5f; // major
            float b = std::min(ell.size.width, ell.size.height) * 0.5f; // minor
            if (b <= 0)
                continue;
            float axisRatio = a / b;
            float dpx = 2.0f * 0.5f * (a + b); // mean diameter
            if (dpx < P.dpx_min || dpx > P.dpx_max)
                continue;
            if (axisRatio > P.max_axis_ratio)
                continue;

            // color fill ratio inside ellipse mask
            cv::Mat ellMask(view.rows, view.cols, CV_8U, cv::Scalar(0));
            cv::ellipse(ellMask, ell, cv::Scalar(255), cv::FILLED);
            cv::Mat overlap;
            cv::bitwise_and(mask, ellMask, overlap);
            float fill = (float)cv::countNonZero(overlap) / (float)cv::countNonZero(ellMask);
            if (!std::isfinite(fill) || fill < P.min_color_fill)
                continue;

            // score: prefer bigger & more circular
            float score = dpx / (1.0f + (axisRatio - 1.0f) * 5.0f);
            cands.push_back({ell.center, dpx, fill, axisRatio, score, ell});
        }

        VodomResult out{};
        out.t_us = t_us;
        out.status = VODOM_NOT_DETECTED;

        std::cout << "processing 0003\n";

        if (!cands.empty())
        {
            // pick best
            auto best = *std::max_element(cands.begin(), cands.end(),
                                          [](auto &A, auto &B)
                                          { return A.score < B.score; });

            // Back to full-image coords if ROI used
            float u = best.c.x + (s.roi.area() ? s.roi.x : 0);
            float v = best.c.y + (s.roi.area() ? s.roi.y : 0);
            float dpx = best.dpx;

            // Camera 3D (assume fronto-parallel disk): Z = f * D / dpx
            float Z = (P.fx * P.disc_D_m) / std::max(dpx, 1.0f);
            float X = (u - P.cx) * Z / P.fx;
            float Y = (v - P.cy) * Z / P.fy;

            cv::Mat pc = (cv::Mat_<double>(3, 1) << X, Y, Z);
            cv::Mat pb = P.Rbc * pc + P.tbc;

            double dx = pb.at<double>(0), dy = pb.at<double>(1);
            out.dx_m = (float)dx;
            out.dy_m = (float)dy;
            out.range_m = std::hypot(out.dx_m, out.dy_m);
            out.bearing_rad = std::atan2(out.dy_m, out.dx_m);
            out.d_px = dpx;
            // simple quality: normalized fill clamped, scaled by diameter
            out.quality = std::min(1.f, std::max(0.f, best.fill)) * std::min(1.f, dpx / 80.f);
            out.status = VODOM_DETECTED_RED;

            // Temporal confirmation + ROI tracking
            consecutiveHits++;
            if (consecutiveHits >= P.confirm_hits)
            {
                // lock in ROI around detection
                int R = int(best.dpx * P.roi_expand);
                cv::Rect newROI(int(u - R / 2), int(v - R / 2), R, R);
                s.roi = newROI & cv::Rect(0, 0, und.cols, und.rows);
            }
            s.lostFrames = 0;
        }
        else
        {
            consecutiveHits = 0;
            if (++s.lostFrames >= P.forget_miss)
                s.roi = cv::Rect(); // reset to full frame
        }

        s.latest.store(out, std::memory_order_release);
    }
}