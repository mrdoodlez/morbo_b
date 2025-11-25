#include "vodom.h"
#include "params.h"
#include "logger.h"
#include "controller.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>
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

static void _Vodom_Worker(VodomInternals &s, VodomParams P);

static inline uint64_t nowMonotonicUS()
{
    using namespace std::chrono;
    return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}

/*******************************************************************************/

static bool Vodom_OpenCamera(cv::VideoCapture &cap,
                             const VodomParams &p)
{
    // Prefer V4L2 backend (Linux)
#ifdef CV_CAP_V4L2
    if (!cap.open(p.camera_dev, cv::CAP_V4L2))
        return false;
#else
    if (!cap.open(p.camera_dev))
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

int Vodom_Start()
{
    ParamsView v{};
    if (Controller_GetParams(ParamPage_Vodom, &v) != 0 || v.size != sizeof(VodomParams))
    {
        vlog.text << "Failed to load visual odometry parameters" << std::endl;
        return -10;
    }

    VodomParams vp = *reinterpret_cast<const VodomParams*>(v.ptr);

    if (!Vodom_OpenCamera(g_vOdomInternals.cap, vp))
    {
        vlog.text << "Failed to open camera: " << vp.camera_dev << std::endl;
        return -20;
    }

    // Prepare undistort
    Vodom_PrepareUndistortMaps(g_vOdomInternals, vp);

    std::thread([vp]
                { _Vodom_Worker(g_vOdomInternals, vp); })
        .detach();

    return 0;
}

/*******************************************************************************/

static void _Vodom_Worker(VodomInternals &s, VodomParams P)
{
    int consecutiveHits = 0;
    uint32_t nFrame = 0;

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

        static cv::Ptr<cv::aruco::Dictionary> dict =
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        static cv::Ptr<cv::aruco::DetectorParameters> detParams =
            cv::aruco::DetectorParameters::create();

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(view, dict, corners, ids, detParams);

        cv::Mat dbg_markers = view.clone();
        cv::aruco::drawDetectedMarkers(dbg_markers, corners, ids);
        vlog.Write("aruco_markers", dbg_markers);

        struct Cand
        {
            cv::Point2f c;
            float dpx;
            float score;
        };
        std::vector<Cand> cands;

        if (!corners.empty())
        {
            for (size_t i = 0; i < corners.size(); ++i)
            {
                const auto &cs = corners[i];

                float w = cv::norm(cs[0] - cs[1]);
                float h = cv::norm(cs[1] - cs[2]);
                float dpx = 0.5f * (w + h);
                float areaScore = w * h;

                cv::Point2f c(0, 0);
                for (int k = 0; k < 4; ++k)
                    c += cs[k];
                c *= 0.25f;

                cands.push_back({c, dpx, areaScore});
            }
        }

        bool gotTag = !cands.empty();

        if (gotTag)
        {
            auto best = *std::max_element(
                cands.begin(), cands.end(),
                [](const Cand &A, const Cand &B)
                { return A.score < B.score; });

            float u = best.c.x + (s.roi.area() ? s.roi.x : 0);
            float v = best.c.y + (s.roi.area() ? s.roi.y : 0);
            float dpx = best.dpx;

            float Z = (P.fx * P.tag_size_m) / std::max(dpx, 1.0f);
            float X = (u - P.cx) * Z / P.fx;
            float Y = (v - P.cy) * Z / P.fy;

            cv::Mat pc = (cv::Mat_<double>(3, 1) << X, Y, Z);
            cv::Mat pb = P.Rbc * pc + P.tbc;

            double dx = pb.at<double>(0);
            double dy = pb.at<double>(1);
            float range_m = std::hypot((float)dx, (float)dy);
            float bearing = std::atan2((float)dy, (float)dx);

            consecutiveHits++;

            if (consecutiveHits >= P.confirm_hits)
            {
                int R = int(dpx * P.roi_expand);
                cv::Rect newROI(int(u - R / 2), int(v - R / 2), R, R);
                s.roi = newROI & cv::Rect(0, 0, und.cols, und.rows);

                VodomMsg out{};
                out.t_us = t_us;
                out.dx_m = (float)dx;
                out.dy_m = (float)dy;
                out.range_m = range_m;
                out.bearing_rad = bearing;
                out.d_px = dpx;
                out.quality = 1.0f;
                if (out.range_m > 2.0f)
                    out.quality = std::min(out.quality, 0.1f);
                out.status = VodomMsg::VodomStatus::DETECTED;

                ControllerMsg msg{};
                msg.ts_ms = out.t_us / 1000ULL;
                msg.type = ControllerMsg::Type::TYPE_VODOM;
                msg.payload.vodom = out;

                Controller_PostMessage(msg);

                vlog.text << "tag locked + reported!" << std::endl;
            }

            s.lostFrames = 0;
        }
        else
        {
            consecutiveHits = 0;
            if (++s.lostFrames >= P.forget_miss)
            {
                s.roi = cv::Rect();
            }

            vlog.text << "TARGET_LOST (no tag in frame)" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}