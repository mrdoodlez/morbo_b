#include "params.h"
#include "mcu_params.h"
#include <fstream>
#include <stdexcept>
#include <cmath>
#include <nlohmann/json.hpp>
#include "logger.h"

using json = nlohmann::json;

static VodomParams g_vodomParams;
static SystemParams g_sysParams;
static McuParams_t g_mcuParams;

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
    case ParamPage_System:
    {
        out->ptr = &g_sysParams;
        out->size = sizeof(SystemParams);
        out->version = 0;
        return 0;
    }
    case ParamPage_Mcu:
    {
        out->ptr = &g_mcuParams;
        out->size = sizeof(McuParams_t);
        out->version = 0;
        return 0;
    }
    default:
        return -10;
    }

    return 0;
}

static int16_t encode_q1k(float v)
{
    float x = v * MCU_Q_SCALE_1K;
    if (x > 32767.0f)
        x = 32767.0f;
    if (x < -32768.0f)
        x = -32768.0f;
    return static_cast<int16_t>(std::lrintf(x));
}

static uint8_t encode_pwm01(float v)
{
    if (v < 0.0f)
        v = 0.0f;
    if (v > 1.0f)
        v = 1.0f;
    return static_cast<uint8_t>(std::lrintf(v * 255.0f));
}

static int load_mcu_params_strict(const json &jm, McuParams_t &m, std::string &err)
{
    try
    {
        // ---------- rover ----------
        const auto &jr = jm.at("rover");
        m.enc_pulses = static_cast<uint16_t>(jr.at("enc_pulses").get<int>());
        m.wheel_d_mm = static_cast<uint16_t>(std::lrintf(jr.at("wheel_d").get<float>() * 1000.0f));       // m→mm
        m.wheel_base_mm = static_cast<uint16_t>(std::lrintf(jr.at("wheel_base").get<float>() * 1000.0f)); // m→mm

        // ---------- omega ----------
        const auto &jo = jm.at("omega");
        m.omega_window_sz = static_cast<uint8_t>(jo.at("window_sz").get<int>());
        m.reserved0 = 0;

        // ---------- pid_v ----------
        {
            const auto &jp = jm.at("pid_v");
            m.pid_v_kp_q = encode_q1k(jp.at("kp").get<float>());
            m.pid_v_ki_q = encode_q1k(jp.at("ki").get<float>());
            m.pid_v_kd_q = encode_q1k(jp.at("kd").get<float>());
        }

        // ---------- pid_w ----------
        {
            const auto &jp = jm.at("pid_w");
            m.pid_w_kp_q = encode_q1k(jp.at("kp").get<float>());
            m.pid_w_ki_q = encode_q1k(jp.at("ki").get<float>());
            m.pid_w_kd_q = encode_q1k(jp.at("kd").get<float>());
        }

        // ---------- limits ----------
        {
            const auto &jl = jm.at("limits");
            m.pwm_max_q = encode_pwm01(jl.at("pwm_max").get<float>());
            m.reserved1 = 0;
        }

        // ---------- go to ----------
        {
            const auto &jp = jm.at("path");
            m.path_eps_x_q = encode_q1k(jp.at("eps_x").get<float>());
            m.path_eps_phi_q = encode_q1k(jp.at("eps_phi").get<float>());

            m.path_replan_dt_us = static_cast<uint64_t>(jp.at("replan_dt").get<double>());
            m.path_v_max_q = encode_q1k(jp.at("v_max").get<float>());
            m.path_w_max_q = encode_q1k(jp.at("w_max").get<float>());
        }

        // ---------- tag_track ----------
        {
            const auto &jt = jm.at("tag_track");
            m.tag_lost_dt_us = static_cast<uint64_t>(jt.at("lost_dt").get<double>());
            m.tag_kv_q = encode_q1k(jt.at("kv").get<float>());
            m.tag_kw_q = encode_q1k(jt.at("kw").get<float>());
            m.tag_eps_x_q = encode_q1k(jt.at("eps_x").get<float>());
            m.tag_eps_y_q = encode_q1k(jt.at("eps_y").get<float>());
        }

        return -1;
    }
    catch (const std::exception &e)
    {
        err = e.what();
        return 0;
    }
}

int Controller_LoadParams(const std::string &config_path)
{
    std::ifstream f(config_path);
    if (!f.is_open())
    {
        return -1;
    }

    json j;
    try
    {
        f >> j;
    }
    catch (const std::exception &e)
    {
        // Parse error
        return -2;
    }

    try
    {
        // -------- VODOM --------
        if (!j.contains("vodom"))
            return -3;

        const auto &jv = j.at("vodom");

        // Device
        g_vodomParams.camera_dev = jv.value("camera_dev", std::string("/dev/video0"));

        // Image
        g_vodomParams.W = jv.value("W", 800);
        g_vodomParams.H = jv.value("H", 600);
        g_vodomParams.FPS = jv.value("FPS", 30);

        g_vodomParams.tag_size_m = jv.value("tag_size_m", 0.105f);

        if (jv.contains("K"))
        {
            auto JK = jv.at("K");
            g_vodomParams.K = cv::Mat(3, 3, CV_64F);
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    g_vodomParams.K.at<double>(r, c) = JK[r][c];
        }

        if (jv.contains("D"))
        {
            auto JD = jv.at("D");
            g_vodomParams.D = cv::Mat(1, JD.size(), CV_64F);
            for (int i = 0; i < (int)JD.size(); ++i)
                g_vodomParams.D.at<double>(0, i) = JD[i];
        }

        if (!g_vodomParams.K.empty())
        {
            g_vodomParams.fx = g_vodomParams.K.at<double>(0, 0);
            g_vodomParams.fy = g_vodomParams.K.at<double>(1, 1);
            g_vodomParams.cx = g_vodomParams.K.at<double>(0, 2);
            g_vodomParams.cy = g_vodomParams.K.at<double>(1, 2);
        }

        if (jv.contains("Rbc"))
        {
            auto JR = jv.at("Rbc");
            g_vodomParams.Rbc = cv::Mat(3, 3, CV_64F);
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    g_vodomParams.Rbc.at<double>(r, c) = JR[r][c];
        }

        if (jv.contains("tbc"))
        {
            auto JT = jv.at("tbc");
            g_vodomParams.tbc = cv::Mat(3, 1, CV_64F);
            for (int i = 0; i < 3 && i < (int)JT.size(); ++i)
                g_vodomParams.tbc.at<double>(i, 0) = JT[i];
        }

        // Temporal filtering
        g_vodomParams.confirm_hits = jv.value("confirm_hits", 2);
        g_vodomParams.forget_miss = jv.value("forget_miss", 3);

        // ROI
        g_vodomParams.roi_expand = jv.value("roi_expand", 2.0f);

        // -------- SYSTEM --------
        if (j.contains("system"))
        {
            const auto &js = j.at("system");
            g_sysParams.mcu_dev = js.value("mcu_dev", std::string("/dev/ttyUSB0"));
            g_sysParams.host_dev = js.value("host_dev", std::string("/dev/ttyUSB1"));
        }

        // -------- MCU --------
        if (!j.contains("mcu"))
        {
            vlog.text << "[params] 'mcu' section missing in " << config_path << std::endl;
            return -2;
        }

        std::string mcuErr;
        if (!load_mcu_params_strict(j["mcu"], g_mcuParams, mcuErr))
        {
            vlog.text << "[params] invalid 'mcu' section in " << config_path << ", " << mcuErr;
            return -3;
        }
    }
    catch (const std::exception &e)
    {
        return -4;
    }

    return 0;
}

int Controller_GetParams(ParamPage page, ParamsView *out_view)
{
    if (!out_view)
        return -10;

    return get_page(page, out_view);
}
