#ifndef GM6020_H
#define GM6020_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CAN IDs (DIP switch ID = 1) */
#define GM6020_TX_ID_CUR_1_4   0x1FEU   /* current control, motors 1-4  */
#define GM6020_TX_ID_CUR_5_7   0x2FEU   /* current control, motors 5-7  */
#define GM6020_TX_ID_VLT_1_4   0x1FFU   /* voltage control, motors 1-4   */
#define GM6020_FB_ID_BASE      0x204U   /* feedback = 0x204 + motor ID   */

#define GM6020_RAW_MAX         16384    /* +-16384 <-> +-3000 mA         */
#define GM6020_CUR_MAX_MA      3000.0f

typedef struct
{
    volatile uint16_t angle_raw;      /* 0..8191                          */
    volatile int16_t  speed_rpm;
    volatile int16_t  current_raw;
    volatile int8_t   temp;           /* degC                             */

    volatile float    angle_deg;      /* 0..360                           */
    volatile float    speed_rpm_f;
    volatile float    speed_rad_s;
    volatile float    current_ma;
    volatile float    angle_unwrap;   /* multi-turn deg                   */

    volatile uint32_t last_rx_ms;
    volatile uint8_t  online;

    uint16_t prev_raw;
    int32_t  turn_count;
} GM6020_Motor_t;

extern GM6020_Motor_t gm6020;          /* motor DIP ID = 1 -> feedback 0x205 */

/* Send current command frame (4 motors packed, raw +-16384).
 * Routed to the bus where feedback was last seen, else all three buses. */
void GM6020_SendCurrent(int16_t cur1, int16_t cur2, int16_t cur3, int16_t cur4);

/* Convenience: single motor (ID1), value in mA, clamped to +-16384 raw. */
void GM6020_SendCurrentMA(float ma);

/* Parse one received CAN frame (call from FDCAN RX callback). */
void GM6020_OnRx(FDCAN_HandleTypeDef *hfdcan, uint32_t id, const uint8_t *d);

/* 1 if feedback received within timeout_ms */
uint8_t GM6020_IsOnline(uint32_t timeout_ms);

/* Clamp helper: raw from mA */
int16_t GM6020_MaToRaw(float ma);

#ifdef __cplusplus
}
#endif

#endif /* GM6020_H */
