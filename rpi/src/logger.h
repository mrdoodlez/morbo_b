#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <atomic>
#include <cstdio>
#include <fstream>
#include <map>
#include <unordered_map>
#include <memory>
#include <mutex>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

class TextLogger
{
public:
    explicit TextLogger(const std::string &path = "vpos.log",
                        bool enabled = true,
                        bool auto_timestamp = true);

    ~TextLogger();

    void set_auto_timestamp(bool on);
    bool auto_timestamp() const;

    template <typename T>
    TextLogger &operator<<(const T &v)
    {
        if (!enabled_)
            return *this;
        std::lock_guard<std::mutex> lk(m_);
        ensure_open_();
        prefix_if_newline_();
        ofs_ << v;
        dirty_line_ = true;
        return *this;
    }

    using Manip = std::ostream &(*)(std::ostream &);
    TextLogger &operator<<(Manip manip)
    {
        if (!enabled_)
            return *this;
        std::lock_guard<std::mutex> lk(m_);
        ensure_open_();
        prefix_if_newline_();
        manip(ofs_);
        ofs_.flush();
        dirty_line_ = false;
        at_line_start_ = true;
        return *this;
    }

private:
    void ensure_open_();
    void prefix_if_newline_();
    static std::string now_ts_();

    mutable std::mutex m_;
    std::ofstream ofs_;
    std::string path_;
    bool enabled_{true};
    bool auto_ts_{true};
    bool at_line_start_{true};
    bool dirty_line_{false};
};

class Logger
{
private:
    TextLogger text_impl_;

public:
    static Logger &Get();

    // Text logger (unchanged)
    TextLogger &text;

    enum class VideoSinkKind
    {
        FFmpegPipe,
        OpenCVVideoWriter
    };

    struct VideoOpts
    {
        VideoSinkKind kind;
        int width;
        int height;
        double fps;

        // ---------- Output targets ----------
        std::string ffmpeg_output; // used if kind == FFmpegPipe (may contain {name})
        std::string cv_file_path;  // used if kind == OpenCVVideoWriter (may contain {name})
        int fourcc;

        // ---------- Core FFmpeg settings (no mixing) ----------
        // Container:
        std::string format; // e.g. "flv" (RTMP), "mpegts" (UDP), "mp4" (file)
        // Video:
        std::string vcodec;  // e.g. "libx264"
        std::string preset;  // e.g. "veryfast"
        std::string tune;    // e.g. "zerolatency" ("" to disable)
        std::string pix_fmt; // e.g. "yuv420p"
        std::string b_v;     // e.g. "4M"
        std::string maxrate; // e.g. "5M"
        std::string bufsize; // e.g. "10M"
        int gop;             // keyframe interval, e.g. 30

        // Extra flags appended LAST (optional; for expert tweaks/overrides)
        std::vector<std::string> ffmpeg_extra_args;

        // ---- Constructor with good RTMP defaults ----
        VideoOpts(VideoSinkKind kind_ = VideoSinkKind::FFmpegPipe)
            : kind(kind_),
              width(0), height(0), fps(30.0),
              fourcc(cv::VideoWriter::fourcc('a', 'v', 'c', '1')),
              // RTMP-friendly defaults:
              format("flv"),
              vcodec("libx264"),
              preset("veryfast"),
              tune("zerolatency"),
              pix_fmt("yuv420p"),
              b_v("4M"), maxrate("5M"), bufsize("10M"),
              gop(30)
        {
            if (kind == VideoSinkKind::FFmpegPipe)
            {
                ffmpeg_output = "rtmp://localhost/live/{name}";
                ffmpeg_extra_args = {"-flush_packets", "1", "-fflags", "nobuffer", "-flags", "low_delay"};
            }
            else
            {
                cv_file_path = "vod/{name}.mp4";
            }
        }
    };

    // Set defaults once; all streams will use these when auto-created
    void SetVideoOpts(const VideoOpts &opts);

    // Push a frame. If the stream doesn't exist yet, it will be created
    // using the *current* defaults and then the frame will be written.
    void Write(const std::string &streamName, const cv::Mat &frame);

    // Close one / all streams
    void CloseVideoStream(const std::string &name);
    void CloseAll();

private:
    Logger();
    ~Logger();
    Logger(const Logger &) = delete;
    Logger &operator=(const Logger &) = delete;

    // Internals
    struct FfmpegPipe
    {
        std::unique_ptr<FILE, int (*)(FILE *)> pipe{nullptr, nullptr};
        int width{0}, height{0};
        double fps{30.0};
        std::string cmd;
    };
    struct CvWriter
    {
        cv::VideoWriter vw;
        int width{0}, height{0};
        double fps{30.0};
    };
    struct VideoStream
    {
        VideoSinkKind kind;
        VideoOpts opts; // resolved opts for this stream
        std::unique_ptr<FfmpegPipe> ff;
        std::unique_ptr<CvWriter> cvw;
        std::mutex mtx;
        bool initialized{false};
    };

    // Helpers
    static std::string build_ffmpeg_cmd_(const VideoOpts &o, int w, int h);
    static std::unique_ptr<FILE, int (*)(FILE *)> popen_write_binary_(const std::string &cmd);
    static bool as_contiguous_bgr24_(const cv::Mat &in, cv::Mat &out, int w, int h);
    static std::string expand_(const std::string &templ, const std::string &name, const char *fallback_ext);

    bool init_stream_(VideoStream &s, const cv::Mat &first);
    void write_frame_ffmpeg_(VideoStream &s, const cv::Mat &frame);
    void write_frame_cv_(VideoStream &s, const cv::Mat &frame);

    // State
    std::mutex vstreams_mtx_;
    std::map<std::string, std::unique_ptr<VideoStream>> vstreams_;

    std::mutex opts_mtx_;
    VideoOpts vopts_;
};

#define vlog (Logger::Get())
