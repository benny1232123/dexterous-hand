#include "gripper_task.h"
#include "gm6020.h"
#include "can_bsp.h"
#include "vofa.h"
#include "bsp_user_key.h"

/* ---------------- test parameters ---------------- */
#define TEST_OPEN_CUR_MA    200.0f     /* open-loop spin current          */
#define OPEN_DURATION_MS    4000       /* each open-loop phase            */
#define IDLE_DURATION_MS    1500       /* wait for feedback before spin   */
#define SPEED_TARGET_RAD    2.0f       /* closed-loop speed target        */
#define SPEED_DURATION_MS   3000       /* each closed-loop direction      */
#define FEEDBACK_TIMEOUT_MS 500        /* watchdog: no frame -> 0 mA      */

/* speed PID */
#define PID_KP              80.0f      /* mA per (rad/s)                  */
#define PID_KI              20.0f
#define PID_KD              0.0f
#define PID_INTEG_LIMIT     1000.0f
#define PID_OUT_LIMIT       2000.0f    /* mA, conservative for first run  */

typedef enum
{
    ST_IDLE = 0,
    ST_OPENCW,
    ST_OPENCCW,
    ST_SPEED,
    ST_FAULT
} GrpState_t;

static float pid_integ   = 0.0f;
static float pid_last_me = 0.0f;
static float cmd_ma      = 0.0f;

static float SpeedPID(float target, float meas, float dt)
{
    float err  = target - meas;
    float dmeas;
    float out;

    pid_integ += PID_KI * err * dt;
    if (pid_integ >  PID_INTEG_LIMIT) pid_integ =  PID_INTEG_LIMIT;
    if (pid_integ < -PID_INTEG_LIMIT) pid_integ = -PID_INTEG_LIMIT;

    dmeas = (meas - pid_last_me) / dt;
    pid_last_me = meas;

    out = PID_KP * err + pid_integ - PID_KD * dmeas;

    if (out >  PID_OUT_LIMIT) out =  PID_OUT_LIMIT;
    if (out < -PID_OUT_LIMIT) out = -PID_OUT_LIMIT;
    return out;
}

void GripperTask_Entry(void const *argument)
{
    GrpState_t  st      = ST_IDLE;
    uint32_t    st_tick = HAL_GetTick();
    float       target  = 0.0f;
    uint8_t     speed_dir = 1;

    (void)argument;
    can_bsp_init();

    for (;;)
    {
        uint32_t now;
        uint32_t elapsed;

        osDelay(1);                              /* 1 kHz control loop */
        now = HAL_GetTick();
        elapsed = now - st_tick;

        /* ---- watchdog: no feedback -> cut current ---- */
        if ((st != ST_IDLE) && (st != ST_FAULT) && !GM6020_IsOnline(FEEDBACK_TIMEOUT_MS))
        {
            cmd_ma  = 0.0f;
            target  = 0.0f;
            st      = ST_FAULT;
            st_tick = now;
            GM6020_SendCurrentMA(0.0f);
        }
        else if (st == ST_FAULT)
        {
            cmd_ma = 0.0f;
            GM6020_SendCurrentMA(0.0f);
            if (GM6020_IsOnline(FEEDBACK_TIMEOUT_MS))   /* motor back -> restart */
            {
                st      = ST_IDLE;
                st_tick = now;
            }
        }
        else
        {
            /* ---- test sequence state machine ---- */
            switch (st)
            {
            case ST_IDLE:
                cmd_ma = 0.0f;
                if (GM6020_IsOnline(FEEDBACK_TIMEOUT_MS) && (BSP_UserKey_Detect() == BUTTON_PRESSED))
                {
                    st      = ST_OPENCW;
                    st_tick = now;
                }
                break;

            case ST_OPENCW:
                cmd_ma = TEST_OPEN_CUR_MA;
                if (elapsed >= OPEN_DURATION_MS)
                {
                    st      = ST_OPENCCW;
                    st_tick = now;
                }
                break;

            case ST_OPENCCW:
                cmd_ma = -TEST_OPEN_CUR_MA;
                if (elapsed >= OPEN_DURATION_MS)
                {
                    st      = ST_SPEED;
                    st_tick = now;
                    speed_dir = 1;
                    pid_integ = 0.0f;
                    pid_last_me = gm6020.speed_rad_s;
                }
                break;

            case ST_SPEED:
                target = speed_dir ? SPEED_TARGET_RAD : -SPEED_TARGET_RAD;
                cmd_ma = SpeedPID(target, gm6020.speed_rad_s, 0.001f);
                if (elapsed >= SPEED_DURATION_MS)
                {
                    speed_dir = !speed_dir;
                    st_tick   = now;
                    pid_integ = 0.0f;
                }
                break;

            default:
                cmd_ma = 0.0f;
                break;
            }

            GM6020_SendCurrentMA(cmd_ma);
        }

        /* ---- VOFA: state/target/measured/cmd/angle ---- */
        vofa_send_data(0, (float)st);
        vofa_send_data(1, target);
        vofa_send_data(2, gm6020.speed_rad_s);
        vofa_send_data(3, cmd_ma);
        vofa_send_data(4, gm6020.angle_unwrap);
        vofa_sendframetail();
    }
}
