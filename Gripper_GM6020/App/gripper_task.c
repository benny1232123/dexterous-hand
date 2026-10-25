#include "gripper_task.h"
#include "gm6020.h"
#include "can_bsp.h"
#include "vofa.h"
#include "bsp_user_key.h"
#include "pid.h"
#include "tim.h"

/* ===================== control period ===================== */
#define T_CTRL_S              0.001f     /* TIM1 update, 1 kHz, matches reference */

/* ============ speed loop (loaded bench tuning) ============ */
#define KP_OMEGA              3000.0f
#define KI_OMEGA              3.0f
#define KD_OMEGA              0.3f
#define OMEGA_INTEG_LIMIT     200.0f
#define OMEGA_OUT_LIMIT       2000.0f    /* mA */
#define FRICTION_FF_MA        400.0f     /* breakaway + loaded friction */

/* ============ angle loop (design Kp=50 rad/s per rad = 0.139 rps/deg) ============ */
#define KP_ANGLE              0.1389f
#define KI_ANGLE              0.0f
#define KD_ANGLE              0.0f
#define ANGLE_INTEG_LIMIT     0.0f
#define ANGLE_OUT_LIMIT_RPS   0.3f      /* slower first loaded test */
#define ANGLE_DEADBAND_DEG    0.5f
#define ANGLE_DAMPING         0.5f      /* velocity feedback for damping */
#define SPEED_SLEW_RPS_PER_S  3.0f      /* limit speed-command slope */
#define FRICTION_FF_RAMP_RPS  0.1f      /* fade feedforward near zero speed */

/* ===================== protections (design doc) ===================== */
#define CUR_LIMIT_MA          2000.0f
#define CUR_SLEW_MA_PER_S     10000.0f  /* 10 A/s: soften motor inrush */
#define ANGLE_LIMIT_DEG       180.0f
#define STALL_TIME_MS         1000
#define STALL_SPEED_RPS       0.03f    /* loaded slow motion is still motion */
#define TEMP_LIMIT_C          80
#define FB_TIMEOUT_MS         500

/* ============== angle rotation test sequence ============== */
#define SEQ_MIN_STEP_MS       1000
#define SEQ_STEP_MARGIN_MS    400    /* settle margin on top of travel time */
#define SEQ_SETTLE_DEG        1.0f   /* advance when this close and slow */
#define SEQ_SETTLE_RPS        0.05f
#define SEQ_DWELL_MS          2000   /* hold each settled target this long */
#define SEQ_TIMEOUT_EXTRA_MS  6000   /* loaded moves can be slow */
#define SEQ_STEPS             3
static const float seq_target_deg[SEQ_STEPS] = { 90.0f, -90.0f, 0.0f };

/* ============== telemetry (reference main loop cadence) ============== */
#define TELEMETRY_MS          10

typedef enum
{
    ST_IDLE = 0,
    ST_RUN,
    ST_FAULT,      /* comms lost; auto-recovers when feedback returns */
    ST_STALL       /* stall latched; needs a new key press */
} GrpState_t;

/* reference-style controller state */
static PID          pid_angle;
static PID          pid_omega;

/* written by the TIM1 ISR, read by the telemetry task */
static volatile GrpState_t st               = ST_IDLE;
static volatile uint32_t   st_tick          = 0;
static volatile uint8_t    seq_idx          = 0;
static volatile uint8_t    start_req        = 0;
static volatile float      target_angle_deg = 0.0f;
static volatile float      target_omega_rps = 0.0f;
static volatile float      cmd_ma           = 0.0f;

/* TIM1 ISR only */
static uint8_t  stall_active   = 0;
static uint32_t stall_start_ms = 0;
static uint32_t seq_step_ms    = SEQ_MIN_STEP_MS;
static float    speed_cmd_slew = 0.0f;
static uint8_t  seq_settled    = 0;
static uint32_t seq_settle_ms  = 0;
static float    cmd_ma_slew    = 0.0f;

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

/* shortest time the angle loop can cover delta_deg at its speed cap */
static uint32_t seq_step_duration(float from_deg, float to_deg)
{
    float delta = abss(to_deg - from_deg);
    float deg_per_ms = ANGLE_OUT_LIMIT_RPS * 360.0f / 1000.0f;
    uint32_t step_ms = (uint32_t)(delta / deg_per_ms) + SEQ_STEP_MARGIN_MS;

    if (step_ms < SEQ_MIN_STEP_MS)
        step_ms = SEQ_MIN_STEP_MS;
    return step_ms;
}

/* one-time init, runs before TIM1 is started */
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
    pid_omega.Last_measured = gm6020.speed_rps;
    pid_angle.Last_measured = gm6020.angle_unwrap;
    PID_SetTarget(&pid_angle, target_angle_deg);
}

/* outer angle loop -> inner speed loop -> current (reference omiga_loopControl) */
static void gripper_loop_control(void)
{
    float angle_deg = gm6020.angle_unwrap;
    float omega_rps = gm6020.speed_rps;      /* Kalman-filtered in the RX callback */
    float speed_cmd;
    float cur;
    float ff;
    float mag;

    speed_cmd = PID_compute(&pid_angle, angle_deg, T_CTRL_S);

    /* velocity feedback damps the approach and the settle */
    speed_cmd -= ANGLE_DAMPING * omega_rps;

    /* stop hunting when already at the target */
    if (abss(target_angle_deg - angle_deg) < ANGLE_DEADBAND_DEG)
        speed_cmd = 0.0f;

    /* slew-limit the speed command so reversals are not a step */
    if (speed_cmd > speed_cmd_slew + SPEED_SLEW_RPS_PER_S * T_CTRL_S)
        speed_cmd_slew += SPEED_SLEW_RPS_PER_S * T_CTRL_S;
    else if (speed_cmd < speed_cmd_slew - SPEED_SLEW_RPS_PER_S * T_CTRL_S)
        speed_cmd_slew -= SPEED_SLEW_RPS_PER_S * T_CTRL_S;
    else
        speed_cmd_slew = speed_cmd;

    speed_cmd = limit_angle_command(angle_deg, speed_cmd_slew);
    target_omega_rps = speed_cmd;
    PID_SetTarget(&pid_omega, speed_cmd);

    cur = PID_compute(&pid_omega, omega_rps, T_CTRL_S);

    /* friction feedforward, faded out near zero so the settle is smooth */
    ff = FRICTION_FF_MA;
    mag = abss(speed_cmd);
    if (mag < FRICTION_FF_RAMP_RPS)
        ff *= mag / FRICTION_FF_RAMP_RPS;

    if (speed_cmd > 0.0f)
        cur += ff;
    else if (speed_cmd < 0.0f)
        cur -= ff;

    cmd_ma = clampf(cur, CUR_LIMIT_MA);

    /* slew-limit the current command to soften the startup inrush */
    if (cmd_ma > cmd_ma_slew + CUR_SLEW_MA_PER_S * T_CTRL_S)
        cmd_ma_slew += CUR_SLEW_MA_PER_S * T_CTRL_S;
    else if (cmd_ma < cmd_ma_slew - CUR_SLEW_MA_PER_S * T_CTRL_S)
        cmd_ma_slew -= CUR_SLEW_MA_PER_S * T_CTRL_S;
    else
        cmd_ma_slew = cmd_ma;

    cmd_ma = cmd_ma_slew;
}

/* 1 kHz control step, called from HAL_TIM_PeriodElapsedCallback(TIM1) */
void GripperControl_OnTimer(void)
{
    uint32_t now = HAL_GetTick();

    /* ---- key press -> start angle rotation test sequence ---- */
    if (start_req)
    {
        start_req = 0;
        if (st == ST_RUN)
        {
            /* key press during a run = immediate stop */
            st               = ST_IDLE;
            cmd_ma           = 0.0f;
            target_omega_rps = 0.0f;
            stall_active     = 0;
            speed_cmd_slew   = 0.0f;
            cmd_ma_slew      = 0.0f;
        }
        else if ((st == ST_IDLE) || (st == ST_STALL))
        {
            st               = ST_RUN;
            seq_idx          = 0;
            st_tick          = now;
            target_angle_deg = seq_target_deg[0];
            seq_step_ms      = seq_step_duration(gm6020.angle_unwrap, target_angle_deg);
            speed_cmd_slew   = 0.0f;
            seq_settled      = 0;
            seq_settle_ms    = now;
            cmd_ma_slew      = 0.0f;
            gripper_controllers_reset();
        }
    }

    /* ---- communication watchdog: guards an active run, never starts one ---- */
    if (!GM6020_IsOnline(FB_TIMEOUT_MS))
    {
        if (st == ST_RUN)
        {
            st           = ST_FAULT;
            cmd_ma       = 0.0f;
            stall_active = 0;
        }
    }
    else if (st == ST_FAULT)
    {
        st      = ST_RUN;
        st_tick = now;
        gripper_controllers_reset();
    }

    if (st == ST_RUN)
    {
        /* ---- stall detection: commanded to rotate, but not rotating ---- */
        if ((abss(target_omega_rps) > STALL_SPEED_RPS) && (abss(gm6020.speed_rps) < STALL_SPEED_RPS))
        {
            if (!stall_active)
            {
                stall_active   = 1;
                stall_start_ms = now;
            }
            else if ((now - stall_start_ms) >= STALL_TIME_MS)
            {
                st           = ST_STALL;   /* latched: needs a new key press */
                cmd_ma       = 0.0f;
                stall_active = 0;
            }
        }
        else
        {
            stall_active = 0;
        }

        if (st == ST_RUN)
        {
            gripper_loop_control();

            /* ---- hold the settled target, then advance ---- */
            {
                uint8_t at_target = (abss(gm6020.angle_unwrap - target_angle_deg) < SEQ_SETTLE_DEG) &&
                                    (abss(gm6020.speed_rps) < SEQ_SETTLE_RPS);
                uint8_t dwell_done = 0;

                if (at_target)
                {
                    if (!seq_settled)
                    {
                        seq_settled   = 1;
                        seq_settle_ms = now;
                    }
                    dwell_done = ((now - seq_settle_ms) >= SEQ_DWELL_MS);
                }
                else
                {
                    seq_settled = 0;
                }

                if (dwell_done || ((now - st_tick) >= (seq_step_ms + SEQ_DWELL_MS + SEQ_TIMEOUT_EXTRA_MS)))
                {
                    seq_idx++;
                    seq_settled = 0;
                    if (seq_idx >= SEQ_STEPS)
                    {
                        st     = ST_IDLE;
                        cmd_ma = 0.0f;
                    }
                    else
                    {
                        st_tick = now;
                        target_angle_deg = seq_target_deg[seq_idx];
                        seq_step_ms = seq_step_duration(seq_target_deg[seq_idx - 1], target_angle_deg);
                        PID_SetTarget(&pid_angle, target_angle_deg);
                    }
                }
            }
        }
    }
    else
    {
        cmd_ma      = 0.0f;
        cmd_ma_slew = 0.0f;
    }

    /* ---- over-temperature protection (always active) ---- */
    if (gm6020.temp >= TEMP_LIMIT_C)
        cmd_ma = 0.0f;

    GM6020_SendCurrentMA(cmd_ma);
}

void GripperTask_Entry(void const *argument)
{
    uint32_t last_tx;
    uint32_t last_key_count;

    (void)argument;

    target_angle_deg = 0.0f;
    gripper_controllers_init();
    can_bsp_init();

    /* TIM1 update interrupt drives the 1 kHz control loop */
    HAL_NVIC_SetPriority(TIM1_UP_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
    HAL_TIM_Base_Start_IT(&htim1);

    last_tx = HAL_GetTick();
    last_key_count = BSP_UserKey_PressCount();

    for (;;)
    {
        uint32_t now;
        uint32_t key_count;

        osDelay(1);

        /* KeyTask is the single debouncer; consume its press counter */
        key_count = BSP_UserKey_PressCount();
        if (key_count != last_key_count)
        {
            last_key_count = key_count;
            start_req = 1;
        }

        now = HAL_GetTick();
        if ((now - last_tx) >= TELEMETRY_MS)
        {
            last_tx = now;

            /* ---- VOFA+: state / target angle / angle / target speed / speed / current ---- */
            vofa_send_data(0, (float)st);
            vofa_send_data(1, target_angle_deg);
            vofa_send_data(2, gm6020.angle_unwrap);
            vofa_send_data(3, target_omega_rps);
            vofa_send_data(4, gm6020.speed_rps);
            vofa_send_data(5, cmd_ma);
            vofa_sendframetail();
        }
    }
}
