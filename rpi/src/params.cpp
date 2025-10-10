#include "params.h"

VodomParams g_vodomParams;

static int get_page(ParamPage p, ParamsView *out)
{
    switch (p)
    {
    case ParamPage_Vodom:
    {
        out->ptr = &g_vodomParams;
        out->size = sizeof(VodomParams);
        out->version = 0;
        return 0;
    }
    default:
        return -10;
    }

    return 0;
}

int Controller_LoadParams()
{
    g_vodomParams = VodomParams{};

    // -------- Image / IO --------
    g_vodomParams.W = 640;
    g_vodomParams.H = 480;
    g_vodomParams.FPS = 30;

    // -------- Target geometry --------
    // Diameter of your signal disk (adjust if different)
    g_vodomParams.disc_D_m = 0.08f; // 80 mm

    // -------- Intrinsics (from your C920 calibration @ 640x480) --------
    // K (3x3, CV_64F)
    g_vodomParams.K = (cv::Mat_<double>(3, 3) << 650.3330, 0.0, 331.8511,
                       0.0, 648.2093, 238.5136,
                       0.0, 0.0, 1.0);
    // D (1x5, CV_64F)  [k1, k2, p1, p2, k3]
    g_vodomParams.D = (cv::Mat_<double>(1, 5) << 0.00215, -0.06148, -0.00144, 0.00551, -0.11276);

    // Cache intrinsics
    g_vodomParams.fx = g_vodomParams.K.at<double>(0, 0);
    g_vodomParams.fy = g_vodomParams.K.at<double>(1, 1);
    g_vodomParams.cx = g_vodomParams.K.at<double>(0, 2);
    g_vodomParams.cy = g_vodomParams.K.at<double>(1, 2);

    // -------- Extrinsics base<-camera (defaults: camera at origin, facing forward) --------
    // Rbc = I, tbc = 0 — replace later with your measured/optimized values
    g_vodomParams.Rbc = cv::Mat::eye(3, 3, CV_64F);
    g_vodomParams.tbc = cv::Mat::zeros(3, 1, CV_64F);

    // -------- HSV thresholds (red, two bands) --------
    g_vodomParams.H1a = 0;
    g_vodomParams.H2a = 10;
    g_vodomParams.SminA = 90;
    g_vodomParams.VminA = 80;
    g_vodomParams.H1b = 170;
    g_vodomParams.H2b = 180;
    g_vodomParams.SminB = 90;
    g_vodomParams.VminB = 80;

    // -------- Morphology --------
    g_vodomParams.morph_open = 3;
    g_vodomParams.morph_close = 5;

    // -------- Detection thresholds --------
    g_vodomParams.dpx_min = 12;
    g_vodomParams.dpx_max = 300;
    g_vodomParams.max_axis_ratio = 1.20f;
    g_vodomParams.min_color_fill = 0.65f;

    // -------- Temporal filtering --------
    g_vodomParams.confirm_hits = 2;
    g_vodomParams.forget_miss = 3;

    // -------- ROI growth factor --------
    g_vodomParams.roi_expand = 2.0f;

    return 0;
}

int Controller_GetParams(ParamPage page, ParamsView *out_view)
{
    if (!out_view)
        return -10;

    return get_page(page, out_view);
}