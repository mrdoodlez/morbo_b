#include "params.h"
#include <fstream>
#include <stdexcept>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

static VodomParams g_vodomParams;
static SystemParams g_sysParams;
static McuParams g_mcuParams;

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
        out->size = sizeof(McuParams);
        out->version = 0;
        return 0;
    }
    default:
        return -10;
    }

    return 0;
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
        g_vodomParams.W   = jv.value("W",   800);
        g_vodomParams.H   = jv.value("H",   600);
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
        g_vodomParams.forget_miss  = jv.value("forget_miss",  3);

        // ROI
        g_vodomParams.roi_expand   = jv.value("roi_expand",   2.0f);

        // -------- SYSTEM --------
        if (j.contains("system"))
        {
            const auto &js = j.at("system");
            g_sysParams.mcu_dev  = js.value("mcu_dev",  std::string("/dev/ttyUSB0"));
            g_sysParams.host_dev = js.value("host_dev", std::string("/dev/ttyUSB1"));
        }

        // -------- MCU --------
        if (j.contains("mcu"))
        {
            const auto &jm = j.at("mcu");

            if (jm.contains("pid_angle"))
            {
                const auto &p = jm.at("pid_angle");
                g_mcuParams.angle_kp = p.value("kp", 1.0);
                g_mcuParams.angle_ki = p.value("ki", 0.1);
                g_mcuParams.angle_kd = p.value("kd", 0.01);
            }

            if (jm.contains("pid_rate"))
            {
                const auto &p = jm.at("pid_rate");
                g_mcuParams.rate_kp = p.value("kp", 0.8);
                g_mcuParams.rate_ki = p.value("ki", 0.05);
                g_mcuParams.rate_kd = p.value("kd", 0.005);
            }

            if (jm.contains("pid_pos"))
            {
                const auto &p = jm.at("pid_pos");
                g_mcuParams.pos_kp = p.value("kp", 0.5);
                g_mcuParams.pos_ki = p.value("ki", 0.0);
                g_mcuParams.pos_kd = p.value("kd", 0.0);
            }
        }
    }
    catch (const std::exception &e)
    {
        // Any unexpected schema issues
        return -4;
    }

    return 0; // success
}

int Controller_GetParams(ParamPage page, ParamsView *out_view)
{
    if (!out_view)
        return -10;

    return get_page(page, out_view);
}
