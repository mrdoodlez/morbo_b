#include "logger.h"
#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>

#define POPEN popen
#define PCLOSE pclose
#define POPEN_MODE "w"

// ================= TextLogger =================
TextLogger::TextLogger(const std::string &path, bool enabled, bool auto_timestamp)
    : path_(path), enabled_(enabled), auto_ts_(auto_timestamp)
{
    ensure_open_();
    if (ofs_.is_open())
        ofs_ << "---- logger start " << now_ts_() << " ----" << std::endl;
}

TextLogger::~TextLogger()
{
    std::lock_guard<std::mutex> lk(m_);
    if (ofs_.is_open())
        ofs_.flush();
}

void TextLogger::set_auto_timestamp(bool on) { auto_ts_ = on; }
bool TextLogger::auto_timestamp() const { return auto_ts_; }

void TextLogger::ensure_open_()
{
    if (!enabled_)
        return;
    if (!ofs_.is_open())
    {
        ofs_.open(path_, std::ios::out /*| std::ios::app*/);
    }
}

std::string TextLogger::now_ts_()
{
    using clock = std::chrono::system_clock;
    auto now = clock::now();
    auto now_s = std::chrono::time_point_cast<std::chrono::seconds>(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - now_s).count();

    std::time_t t = clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&t, &tm);

    // seconds since midnight
    int sec_of_day = tm.tm_hour * 3600 + tm.tm_min * 60 + tm.tm_sec;

    char buf[32];
    std::snprintf(buf, sizeof(buf), "%05d.%03lld", sec_of_day, static_cast<long long>(ms));
    return std::string(buf);
}

void TextLogger::prefix_if_newline_()
{
    if (auto_ts_ && at_line_start_)
    {
        ofs_ << "[" << now_ts_() << "] ";
        at_line_start_ = false;
    }
}

// ================= Video Logger =================
// ...includes unchanged...

Logger &Logger::Get()
{
    static Logger g;
    return g;
}

Logger::Logger()
    : text_impl_("vpos.log", /*enabled*/ true, /*auto_ts*/ true), text(text_impl_) {}

Logger::~Logger() { CloseAll(); }

void Logger::SetVideoOpts(const VideoOpts &opts)
{
    std::lock_guard<std::mutex> lk(opts_mtx_);
    vopts_ = opts;
}

void Logger::Write(const std::string &streamName, const cv::Mat &frame)
{
    if (frame.empty())
        return;

    // Lazily create or find
    std::unique_ptr<VideoStream> *s_ptr = nullptr;
    {
        std::lock_guard<std::mutex> lk(vstreams_mtx_);
        std::lock_guard<std::mutex> dlk(opts_mtx_);
        auto it = vstreams_.find(streamName);
        if (it == vstreams_.end())
        {
            VideoOpts opts = vopts_;

            // Expand target using streamName
            if (opts.kind == VideoSinkKind::FFmpegPipe)
            {
                opts.ffmpeg_output = expand_(opts.ffmpeg_output, streamName, ".mp4");
            }
            else
            {
                opts.cv_file_path = expand_(opts.cv_file_path, streamName, ".mp4");
            }

            auto s = std::make_unique<VideoStream>();
            s->kind = opts.kind;
            s->opts = opts;
            auto [ins, _] = vstreams_.emplace(streamName, std::move(s));
            s_ptr = &ins->second;
        }
        else
        {
            s_ptr = &it->second;
        }
    }

    VideoStream &s = *(*s_ptr);
    if (!s.initialized)
    {
        if (!init_stream_(s, frame))
        {
            std::cerr << "[Logger] stream init failed: " << streamName << std::endl;
            return;
        }
    }

    if (s.kind == VideoSinkKind::FFmpegPipe)
        write_frame_ffmpeg_(s, frame);
    else
        write_frame_cv_(s, frame);
}

// Expand "{name}" if present; else if path looks like a dir or ends with '/',
// append name + fallback_ext; otherwise return as-is.
std::string Logger::expand_(const std::string &templ, const std::string &name, const char *fallback_ext)
{
    if (templ.empty())
        return name + std::string(fallback_ext ? fallback_ext : "");
    auto pos = templ.find("{name}");
    if (pos != std::string::npos)
    {
        std::string out = templ;
        out.replace(pos, 6, name);
        return out;
    }
    // Heuristic: if ends with '/', treat as directory
    if (!templ.empty() && templ.back() == '/')
    {
        return templ + name + (fallback_ext ? fallback_ext : "");
    }
    return templ; // leave as provided
}

void Logger::CloseVideoStream(const std::string &name)
{
    std::unique_ptr<VideoStream> s;
    {
        std::lock_guard<std::mutex> g(vstreams_mtx_);
        auto it = vstreams_.find(name);
        if (it == vstreams_.end())
            return;
        s = std::move(it->second);
        vstreams_.erase(it);
    }
    if (!s)
        return;

    std::lock_guard<std::mutex> lk(s->mtx);
    if (s->kind == VideoSinkKind::FFmpegPipe && s->ff && s->ff->pipe)
    {
        PCLOSE(s->ff->pipe.release()); // waits for ffmpeg to exit → key released
        s->ff.reset();
    }
    else if (s->kind == VideoSinkKind::OpenCVVideoWriter && s->cvw)
    {
        s->cvw->vw.release();
        s->cvw.reset();
    }
    s->initialized = false;
}

void Logger::CloseAll()
{
    std::lock_guard<std::mutex> lk(vstreams_mtx_);
    for (auto &[name, s] : vstreams_)
    {
        if (s->kind == VideoSinkKind::FFmpegPipe && s->ff && s->ff->pipe)
        {
            PCLOSE(s->ff->pipe.release());
        }
        else if (s->kind == VideoSinkKind::OpenCVVideoWriter && s->cvw)
        {
            s->cvw->vw.release();
        }
    }
    vstreams_.clear();
}

// ---- helpers ----
std::string Logger::build_ffmpeg_cmd_(const VideoOpts &o, int w, int h)
{
    std::ostringstream ss;
    ss << "ffmpeg -hide_banner -loglevel error -y "
       << "-f rawvideo -pix_fmt bgr24 "
       << "-s " << w << "x" << h << " "
       << "-r " << (o.fps > 0 ? o.fps : 30.0) << " "
       << "-i - ";

    // Core, structured args (no duplication with extras)
    if (!o.format.empty())
        ss << "-f " << o.format << " ";
    if (!o.vcodec.empty())
        ss << "-c:v " << o.vcodec << " ";
    if (!o.preset.empty())
        ss << "-preset " << o.preset << " ";
    if (!o.tune.empty())
        ss << "-tune " << o.tune << " ";
    if (!o.pix_fmt.empty())
        ss << "-pix_fmt " << o.pix_fmt << " ";
    if (!o.b_v.empty())
        ss << "-b:v " << o.b_v << " ";
    if (!o.maxrate.empty())
        ss << "-maxrate " << o.maxrate << " ";
    if (!o.bufsize.empty())
        ss << "-bufsize " << o.bufsize << " ";
    if (o.gop > 0)
        ss << "-g " << o.gop << " ";

    // Extra args LAST (for advanced tweaks/overrides)
    for (const auto &arg : o.ffmpeg_extra_args)
        ss << arg << " ";

    ss << "\"" << o.ffmpeg_output << "\"";
    return ss.str();
}

std::unique_ptr<FILE, int (*)(FILE *)>
Logger::popen_write_binary_(const std::string &cmd)
{
    FILE *f = POPEN(cmd.c_str(), POPEN_MODE);
    if (!f)
    {
        std::cerr << "[Logger] popen failed (errno=" << errno
                  << " \"" << std::strerror(errno) << "\")\n  cmd: " << cmd << std::endl;
        return {nullptr, PCLOSE};
    }
    return std::unique_ptr<FILE, int (*)(FILE *)>(f, PCLOSE);
}

bool Logger::as_contiguous_bgr24_(const cv::Mat &in, cv::Mat &out, int w, int h)
{
    if (in.cols != w || in.rows != h)
        cv::resize(in, out, cv::Size(w, h));
    else
        out = in;

    if (out.channels() == 1)
        cv::cvtColor(out, out, cv::COLOR_GRAY2BGR);
    else if (out.type() == CV_8UC4)
        cv::cvtColor(out, out, cv::COLOR_BGRA2BGR);
    else if (out.type() != CV_8UC3)
        out.convertTo(out, CV_8UC3);

    if (!out.isContinuous())
        out = out.clone();
    return out.isContinuous() && out.type() == CV_8UC3;
}

bool Logger::init_stream_(VideoStream &s, const cv::Mat &first)
{
    std::lock_guard<std::mutex> lk(s.mtx);
    if (s.initialized)
        return true;

    int w = s.opts.width ? s.opts.width : first.cols;
    int h = s.opts.height ? s.opts.height : first.rows;

    if (s.kind == VideoSinkKind::FFmpegPipe)
    {
        auto cmd = build_ffmpeg_cmd_(s.opts, w, h);
        auto pipe = popen_write_binary_(cmd);
        if (!pipe)
        {
            std::cerr << "[Logger] FFmpeg popen failed: " << cmd << std::endl;
            return false;
        }
        s.ff = std::make_unique<FfmpegPipe>();
        s.ff->pipe = std::move(pipe);
        s.ff->width = w;
        s.ff->height = h;
        s.ff->fps = s.opts.fps;
        s.ff->cmd = cmd;
        s.initialized = true;
        vlog.text << "[Logger] FFmpeg stream ready: " << cmd << std::endl;
        return true;
    }
    else
    {
        s.cvw = std::make_unique<CvWriter>();
        s.cvw->width = w;
        s.cvw->height = h;
        s.cvw->fps = s.opts.fps;
        if (!s.cvw->vw.open(s.opts.cv_file_path, s.opts.fourcc, s.opts.fps, cv::Size(w, h), true))
        {
            std::cerr << "[Logger] OpenCV VideoWriter open failed: " << s.opts.cv_file_path << std::endl;
            s.cvw.reset();
            return false;
        }
        s.initialized = true;
        vlog.text << "[Logger] OpenCV video writer ready: " << s.opts.cv_file_path << std::endl;
        return true;
    }
}

void Logger::write_frame_ffmpeg_(VideoStream &s, const cv::Mat &frame)
{
    std::lock_guard<std::mutex> lk(s.mtx);
    if (!s.ff || !s.ff->pipe)
        return;

    cv::Mat bgr;
    if (!as_contiguous_bgr24_(frame, bgr, s.ff->width, s.ff->height))
        return;
    const size_t bytes = static_cast<size_t>(bgr.cols) * bgr.rows * 3;

    if (std::fwrite(bgr.data, 1, bytes, s.ff->pipe.get()) != bytes)
    {
        std::cerr << "[Logger] ffmpeg fwrite failed (errno: " << std::strerror(errno) << "). cmd=" << s.ff->cmd << std::endl;
    }
    std::fflush(s.ff->pipe.get());
}

void Logger::write_frame_cv_(VideoStream &s, const cv::Mat &frame)
{
    std::lock_guard<std::mutex> lk(s.mtx);
    if (!s.cvw)
        return;

    cv::Mat out;
    if (frame.cols != s.cvw->width || frame.rows != s.cvw->height)
        cv::resize(frame, out, cv::Size(s.cvw->width, s.cvw->height));
    else
        out = frame;

    if (out.channels() == 1)
        cv::cvtColor(out, out, cv::COLOR_GRAY2BGR);
    s.cvw->vw.write(out);
}
