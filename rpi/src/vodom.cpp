#include "vodom.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <atomic>

typedef enum {
    VODOM_NOT_DETECTED = 0,
    VODOM_DETECTED_RED = 1,
    VODOM_DETECTED_GREEN = 2  // future use
} VodomStatus;

typedef struct {
    uint64_t t_us;     // capture timestamp (monotonic/us)
    VodomStatus status;
    float dx_m;        // in base_link
    float dy_m;
    float range_m;     // sqrt(dx^2+dy^2)
    float bearing_rad; // atan2(dy,dx)
    float d_px;        // measured diameter in pixels
    float quality;     // 0..1 (or reprojection-err-like)
} VodomResult;

struct VodomInternals {
  // Camera
  cv::VideoCapture cap;
  cv::Mat K, D;            // intrinsics (3x3, 1x5/1x8)
  float fx, fy, cx, cy;    // cached from K
  // Extrinsics base<-camera
  cv::Mat Rbc;             // 3x3
  cv::Mat tbc;             // 3x1
  // Target
  float disc_D_m;          // physical diameter, meters
  // Runtime
  std::atomic<VodomResult> latest;
  std::atomic<bool> run{true};
  // ROI tracking
  cv::Rect roi;            // empty = full frame
  int lostFrames = 0;
};

struct Params {
  // Image
  int W=640, H=480, FPS=30;

  // Red HSV (two bands, 0..180 hue)
  // band A ~ [0..10], band B ~ [170..180]
  int H1a=0,   H2a=10,  SminA=90,  VminA=80;
  int H1b=170, H2b=180, SminB=90,  VminB=80;

  // Morphology
  int morph_open=3;   // kernel size
  int morph_close=5;

  // Circle size in px (limits)
  int dpx_min=12;     // reject too small (far or noise)
  int dpx_max=300;    // reject too big (too near / partial)

  // Roundness & fill
  float max_axis_ratio=1.2f;    // ellipse major/minor
  float min_color_fill=0.65f;   // mask pixels / area

  // Temporal
  int confirm_hits=2;           // frames to confirm detection
  int forget_miss=3;            // frames to lose detection

  // ROI
  float roi_expand=2.0f;        // grow factor around last circle
};

static void _Vodom_Worker(VodomInternals& s, const Params& P);

int Vodom_Start(const std::string& videoDev)
{
    return 0;
}

/*******************************************************************************/