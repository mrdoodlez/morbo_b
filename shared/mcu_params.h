#ifndef _MCU_PARAMS_H_
#define _MCU_PARAMS_H_

#include <stdint.h>

#define MCU_Q_SCALE_1K 1000.0f // for gains, eps, v, w

typedef struct
{
    /* Rover geometry */
    uint16_t enc_pulses;    // [pulses / rev]
    uint16_t wheel_d_mm;    // wheel diameter [mm]
    uint16_t wheel_base_mm; // wheel base [mm]

    /* Averaging window */
    uint8_t omega_window_sz; // window size (e.g. 16)
    uint8_t reserved0;       // padding / future use

    /* PID (linear velocity) – Q1K */
    int16_t pid_v_kp_q;
    int16_t pid_v_ki_q;
    int16_t pid_v_kd_q;

    /* PID (angular velocity) – Q1K */
    int16_t pid_w_kp_q;
    int16_t pid_w_ki_q;
    int16_t pid_w_kd_q;

    /* Limits */
    uint8_t pwm_max_q; // [0..255] ~= [0..1]
    uint8_t reserved1; // padding / future use

    /* Path-planning – Q1K + dt in us */
    int16_t path_eps_x_q;       // EPS_X
    int16_t path_eps_phi_q;     // EPS_PHI
    uint64_t path_replan_dt_us; // REPLAN_DT [us]
    int16_t path_v_max_q;       // V_MAX
    int16_t path_w_max_q;       // W_MAX

    /* Tag tracking – Q1K + dt in us */
    uint64_t tag_lost_dt_us; // TRG_TRK_LOST_DT [us]
    int16_t tag_kv_q;        // TRG_TRK_KV
    int16_t tag_kw_q;        // TRG_TRK_KW
    int16_t tag_eps_x_q;     // TRG_TRK_EPS_X
    int16_t tag_eps_y_q;     // TRG_TRK_EPS_Y

} __attribute__((packed)) McuParams_t;

#endif //_MCU_PARAMS_H_