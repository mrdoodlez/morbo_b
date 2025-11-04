#include "vodom.h"
#include "params.h"
#include "logger.h"
#include "controller.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <atomic>
#include <thread>
#include <iostream>

struct VodomInternals
{
    // camera
    cv::VideoCapture cap;

    // undistort maps (faster than cv::undistort per frame)
    cv::Mat map1, map2; // CV_16SC2 or CV_32FC1
    bool have_maps = false;

    // ROI tracking
    cv::Rect roi; // empty => full frame
    int lostFrames = 0;
    int consecutiveHits = 0;
};

static VodomInternals g_vOdomInternals;
static VodomParams g_params;

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
        vlog.text << "Failed to load visual odometry parameters" << std::endl;
        return -10;
    }

    auto *vp = static_cast<const VodomParams *>(v.ptr);
    g_params = *vp;

    if (!Vodom_OpenCamera(g_vOdomInternals.cap, videoDev, g_params))
    {
        vlog.text << "Failed to open camera: " << videoDev << std::endl;
        return -20;
    }

    // Prepare undistort
    Vodom_PrepareUndistortMaps(g_vOdomInternals, g_params);

    std::thread([&]
                { _Vodom_Worker(g_vOdomInternals, g_params); })
        .detach();

    return 0;
}

/*******************************************************************************/

static void _Vodom_Worker(VodomInternals &s, const VodomParams &P)
{
    int consecutiveHits = 0;

    uint32_t nFrame = 0;

    auto sane = [](int k)
    {
        k = std::max(1, k);
        if ((k % 2) == 0)
            k += 1;          // force odd
        k = std::min(k, 51); // cap (adjust if needed)
        return k;
    };
    int kOpenSz = sane(P.morph_open);
    int kCloseSz = sane(P.morph_close);

    cv::Mat kOpen, kClose;
    if (kOpenSz > 0)
        kOpen = cv::getStructuringElement(cv::MORPH_ELLIPSE, {kOpenSz, kOpenSz});
    if (kCloseSz > 0)
        kClose = cv::getStructuringElement(cv::MORPH_ELLIPSE, {kCloseSz, kCloseSz});

    // Debug once
    static bool printed = false;
    if (!printed)
    {
        printed = true;
        vlog.text << "kOpen=" << (kOpen.empty() ? 0 : kOpen.rows)
                  << " kClose=" << (kClose.empty() ? 0 : kClose.rows) << std::endl;
    }

    extern std::atomic<bool> g_stop;
    while (!g_stop.load(std::memory_order_relaxed))
    {
        cv::Mat frame, und, hsv, mask, edges;

        uint64_t t_us = nowMonotonicUS();

        if (!s.cap.read(frame))
            continue;

        vlog.text << "frame #" << nFrame << std::endl;
        nFrame++;

        vlog.Write("raw", frame);

        if (s.have_maps)
            cv::remap(frame, und, s.map1, s.map2, cv::INTER_LINEAR);
        else
            und = frame;

        vlog.Write("und", und);

        cv::Mat view = und;
        if (s.roi.area() > 0)
        {
            s.roi &= cv::Rect(0, 0, und.cols, und.rows);
            view = und(s.roi);
        }

        vlog.Write("view", view);

        cv::cvtColor(view, hsv, cv::COLOR_BGR2HSV);

        CV_Assert(hsv.type() == CV_8UC3);

        // Split channels
        std::vector<cv::Mat> ch;
        cv::split(hsv, ch);
        cv::Mat H = ch[0], S = ch[1], V = ch[2];

        // Parameters
        const int bins = 12;
        const float h_range[] = {0.f, 180.f};
        const float sv_range[] = {0.f, 256.f};
        const float *rangesH[] = {h_range};
        const float *rangesSV[] = {sv_range};

        // Compute histograms (CV_32F)
        /*
        cv::Mat hHist, sHist, vHist;
        cv::calcHist(&H, 1, 0, cv::Mat(), hHist, 1, &bins, rangesH, true, false);
        cv::calcHist(&S, 1, 0, cv::Mat(), sHist, 1, &bins, rangesSV, true, false);
        cv::calcHist(&V, 1, 0, cv::Mat(), vHist, 1, &bins, rangesSV, true, false);

        cv::normalize(hHist, hHist, 1.0, 0.0, cv::NORM_L1);
        cv::normalize(sHist, sHist, 1.0, 0.0, cv::NORM_L1);
        cv::normalize(vHist, vHist, 1.0, 0.0, cv::NORM_L1);

        // Print results
        vlog.text << "H: ";
        for (int i = 0; i < bins; ++i)
            vlog.text << "h[" << i << "]=" << hHist.at<float>(i) << " ";
        vlog.text << "\nS: ";
        for (int i = 0; i < bins; ++i)
            vlog.text << "s[" << i << "]=" << sHist.at<float>(i) << " ";
        vlog.text << "\nV: ";
        for (int i = 0; i < bins; ++i)
            vlog.text << "v[" << i << "]=" << vHist.at<float>(i) << " ";
        vlog.text << std::endl;
        */

        cv::inRange(hsv, cv::Scalar(P.Hmin, P.Smin, P.Vmin), cv::Scalar(179, 255, 255), mask);

        if (mask.type() != CV_8UC1)
        {
            if (mask.channels() == 1)
                mask.convertTo(mask, CV_8U);
            else
                cv::cvtColor(mask, mask, cv::COLOR_BGR2GRAY); // belt & suspenders
        }

        vlog.Write("mask", mask);

        vlog.text << "roi=" << s.roi << " view=" << view.cols << "x" << view.rows << std::endl;
        vlog.text << "hsv=" << hsv.cols << "x" << hsv.rows << " type=" << hsv.type() << std::endl;
        vlog.text << "mask nnz=" << cv::countNonZero(mask) << std::endl;

        // Morphology (tmp + swap)
        cv::Mat tmp;
        if (!mask.empty() && !kOpen.empty())
        {
            cv::morphologyEx(mask, tmp, cv::MORPH_OPEN, kOpen, cv::Point(-1, -1), 1, cv::BORDER_CONSTANT, 0);
            std::swap(mask, tmp);
        }

        vlog.Write("m_open", mask);

        if (!mask.empty() && !kClose.empty())
        {
            cv::morphologyEx(mask, tmp, cv::MORPH_CLOSE, kClose, cv::Point(-1, -1), 1, cv::BORDER_CONSTANT, 0);
            std::swap(mask, tmp);
        }

        vlog.Write("m_close", mask);

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

            vlog.text << "ellipse found: " << a << " " << b << " " << score << std::endl;
        }

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

            vlog.text << "best candidate at: " << u << " " << v << std::endl;

            // Camera 3D (assume fronto-parallel disk): Z = f * D / dpx
            float Z = (P.fx * P.disc_D_m) / std::max(dpx, 1.0f);
            float X = (u - P.cx) * Z / P.fx;
            float Y = (v - P.cy) * Z / P.fy;

            vlog.text << "cam frame: " << Z << " " << X << " " << Y << std::endl;

            cv::Mat pc = (cv::Mat_<double>(3, 1) << X, Y, Z);
            cv::Mat pb = P.Rbc * pc + P.tbc;

            VodomMsg out{};
            out.t_us = t_us;

            double dx = pb.at<double>(0), dy = pb.at<double>(1);
            out.dx_m = (float)dx;
            out.dy_m = (float)dy;
            out.range_m = std::hypot(out.dx_m, out.dy_m);
            out.bearing_rad = std::atan2(out.dy_m, out.dx_m);
            out.d_px = dpx;

            vlog.text << "dx: " << dx << "dy " << dy << std::endl;

            // simple quality: normalized fill clamped, scaled by diameter
            out.quality = std::min(1.f, std::max(0.f, best.fill)) * std::min(1.f, dpx / 80.f);

            if (out.range_m > 2.0f)
            {
                out.quality = std::min(out.quality, 0.1f);
            }

            out.status = VodomMsg::VodomStatus::VODOM_DETECTED_RED;

            ControllerMsg msg{};
            msg.ts_ms = out.t_us / 1000ULL; // convert to ms for the controller clock
            msg.type = ControllerMsg::Type::TYPE_VODOM;
            msg.payload.vodom = out; // POD copy

            Controller_PostMessage(msg);

            // Temporal confirmation + ROI tracking
            consecutiveHits++;
            if (consecutiveHits >= P.confirm_hits)
            {
                // lock in ROI around detection
                int R = int(best.dpx * P.roi_expand);
                cv::Rect newROI(int(u - R / 2), int(v - R / 2), R, R);
                s.roi = newROI & cv::Rect(0, 0, und.cols, und.rows);

                vlog.text << "ellipse locked!" << std::endl;
            }
            s.lostFrames = 0;
        }
        else
        {
            consecutiveHits = 0;
            if (++s.lostFrames >= P.forget_miss)
                s.roi = cv::Rect(); // reset to full frame

            vlog.text << "loss of lock!" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}