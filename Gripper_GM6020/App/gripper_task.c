#include "gripper_task.h"
#include "gm6020.h"
#include "can_bsp.h"
#include "vofa.h"
#include "bsp_user_key.h"
#include "pid.h"
#include "filter.h"

/* ===================== control period ===================== */
#define T_CTRL_S              0.001f     /* 1 kHz, matches reference */

/* ============ speed loop (reference motorControl.c) ============ */
#define KP_OMEGA              300.0f
#define KI_OMEGA              3.0f
#define KD_OMEGA              0.3f
#define OMEGA_INTEG_LIMIT     200.0f
#define OMEGA_OUT_LIMIT       2000.0f    /* mA */

/* ============ angle loop (design doc initial values) ============ */
#define KP_ANGLE              50.0f
#define KI_ANGLE              0.0f
#define KD_ANGLE              0.0f
#define ANGLE_INTEG_LIMIT     0.0f
#define ANGLE_OUT_LIMIT_RPS   0.4774648f /* 3 rad/s expressed in rps */

/* ===================== protections (design doc) ===================== */
#define CUR_LIMIT_MA          2000.0f
#define ANGLE_LIMIT_DEG       180.0f
#define STALL_TIME_MS         500
#define STALL_CUR_MA          500.0f
#define STALL_SPEED_RPS       0.1f     /* must stay below ANGLE_OUT_LIMIT_RPS (0.4775) */
#define TEMP_LIMIT_C          80
#define FB_TIMEOUT_MS         500

/* ============== angle rotation test sequence ============== */
#define SEQ_STEP_MS           1000
#define SEQ_STEPS             3
static const float seq_target_deg[SEQ_STEPS] = { 90.0f, -90.0f, 0.0f };

typedef enum
{
    ST_IDLE = 0,
    ST_RUN,
    ST_FAULT
} GrpState_t;

/* reference-style controller state */
static PID          pid_angle;
static PID          pid_omega;
static KalmanFilter motor0_kalman = { 0.0f, 1.0f, 0.1f, 1.0f };

static float target_angle_deg = 0.0f;
static float target_omega_rps = 0.0f;
static float cmd_ma           = 0.0f;

static uint8_t  stall_active   = 0;
static uint32_t stall_start_ms = 0;

static float abss(float v)
{
    return (v < 0.0f) ? -v : v;
}

static float clampf(float v, float lim)
{
    if (v >  lim) return  lim;
    if (v < -lim) return -lim;
    return v;
}

/* forbid driving past the +-180 deg mechanical limit */
static float limit_angle_command(float angle_deg, float cmd)
{
    if (angle_deg >=  ANGLE_LIMIT_DEG && cmd > 0.0f) return 0.0f;
    if (angle_deg <= -ANGLE_LIMIT_DEG && cmd < 0.0f) return 0.0f;
    return cmd;
}

/* one-time init (reference motor_initial style) */
static void gripper_controllers_init(void)
{
    PID_Init(&pid_omega, KP_OMEGA, KI_OMEGA, KD_OMEGA, OMEGA_INTEG_LIMIT, OMEGA_OUT_LIMIT);
    PID_Init(&pid_angle, KP_ANGLE, KI_ANGLE, KD_ANGLE, ANGLE_INTEG_LIMIT, ANGLE_OUT_LIMIT_RPS);
    PID_SetTarget(&pid_angle, target_angle_deg);
}

/* soft reset between runs: clear integral, seed Last_measured to avoid derivative kick */
static void gripper_controllers_reset(void)
{
    pid_omega.integral = 0.0f;
    pid_angle.integral = 0.0f;
    pid_omega.Last_measured = gm6020.speed_rpm / 60.0f;
    pid_angle.Last_measured = gm6020.angle_unwrap;
    PID_SetTarget(&pid_angle, target_angle_deg);
}

/* outer angle loop -> inner speed loop -> current (reference omiga_loopControl) */
static void gripper_loop_control(void)
{
    float angle_deg = gm6020.angle_unwrap;
    float omega_rps = KalmanFilter_Update(&motor0_kalman, gm6020.speed_rpm / 60.0f);
    float speed_cmd;
    float cur;

    speed_cmd = PID_compute(&pid_angle, angle_deg, T_CTRL_S);
    speed_cmd = limit_angle_command(angle_deg, speed_cmd);
    target_omega_rps = speed_cmd;
    PID_SetTarget(&pid_omega, speed_cmd);

    cur = PID_compute(&pid_omega, omega_rps, T_CTRL_S);
    cmd_ma = clampf(cur, CUR_LIMIT_MA);
}

void GripperTask_Entry(void const *argument)
{
    GrpState_t st       = ST_IDLE;
    uint32_t   st_tick  = HAL_GetTick();
    uint8_t    seq_idx  = 0;
    uint8_t    key_prev = 0;

    (void)argument;

    target_angle_deg = 0.0f;
    gripper_controllers_init();
    can_bsp_init();

    for (;;)
    {
        uint32_t now;
        uint8_t  key;

        osDelay(1);                       /* 1 kHz control loop */
        now = HAL_GetTick();

        /* ---- key press -> start angle rotation test sequence ---- */
        key = BSP_UserKey_Detect();
        if ((key == BUTTON_PRESSED) && (key_prev == 0) && (st == ST_IDLE))
        {
            st              = ST_RUN;
            seq_idx         = 0;
            st_tick         = now;
            target_angle_deg = seq_target_deg[0];
            gripper_controllers_reset();
        }
        key_prev = key;

        /* ---- communication watchdog ---- */
        if (!GM6020_IsOnline(FB_TIMEOUT_MS))
        {
            st          = ST_FAULT;
            cmd_ma      = 0.0f;
            stall_active = 0;
        }
        else if (st == ST_FAULT)
        {
            st      = ST_RUN;
            st_tick = now;
            gripper_controllers_reset();
        }

        if (st == ST_RUN)
        {
            /* ---- stall detection: current but no speed ---- */
            if ((abss(cmd_ma) > STALL_CUR_MA) && (abss(gm6020.speed_rpm / 60.0f) < STALL_SPEED_RPS))
            {
                if (!stall_active)
                {
                    stall_active   = 1;
                    stall_start_ms = now;
                }
                else if ((now - stall_start_ms) >= STALL_TIME_MS)
                {
                    st           = ST_FAULT;
                    cmd_ma       = 0.0f;
                    stall_active = 0;
                }
            }
            else
            {
                stall_active = 0;
            }

            if (st == ST_RUN)
                gripper_loop_control();

            /* ---- advance the test sequence ---- */
            if ((now - st_tick) >= SEQ_STEP_MS)
            {
                seq_idx++;
                if (seq_idx >= SEQ_STEPS)
                {
                    st     = ST_IDLE;
                    cmd_ma = 0.0f;
                }
                else
                {
                    st_tick = now;
                    target_angle_deg = seq_target_deg[seq_idx];
                    PID_SetTarget(&pid_angle, target_angle_deg);
                }
            }
        }
        else
        {
            cmd_ma = 0.0f;
        }

        /* ---- over-temperature protection (always active) ---- */
        if (gm6020.temp >= TEMP_LIMIT_C)
            cmd_ma = 0.0f;

        GM6020_SendCurrentMA(cmd_ma);

        /* ---- VOFA+: state / target angle / angle / target speed / speed / current ---- */
        vofa_send_data(0, (float)st);
        vofa_send_data(1, target_angle_deg);
        vofa_send_data(2, gm6020.angle_unwrap);
        vofa_send_data(3, target_omega_rps);
        vofa_send_data(4, gm6020.speed_rpm / 60.0f);
        vofa_send_data(5, cmd_ma);
        vofa_sendframetail();
    }
}
