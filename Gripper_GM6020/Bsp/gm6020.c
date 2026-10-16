#include "gm6020.h"
#include "can_bsp.h"

GM6020_Motor_t gm6020;

static FDCAN_HandleTypeDef *gm6020_tx_bus = 0;
static int16_t gm6020_cmd[4] = {0, 0, 0, 0};

int16_t GM6020_MaToRaw(float ma)
{
    if (ma >  GM6020_CUR_MAX_MA) ma =  GM6020_CUR_MAX_MA;
    if (ma < -GM6020_CUR_MAX_MA) ma = -GM6020_CUR_MAX_MA;
    return (int16_t)(ma * ((float)GM6020_RAW_MAX / GM6020_CUR_MAX_MA));
}

static void gm6020_pack(uint8_t *d)
{
    d[0] = (uint8_t)((gm6020_cmd[0] >> 8) & 0xFF);
    d[1] = (uint8_t)( gm6020_cmd[0]       & 0xFF);
    d[2] = (uint8_t)((gm6020_cmd[1] >> 8) & 0xFF);
    d[3] = (uint8_t)( gm6020_cmd[1]       & 0xFF);
    d[4] = (uint8_t)((gm6020_cmd[2] >> 8) & 0xFF);
    d[5] = (uint8_t)( gm6020_cmd[2]       & 0xFF);
    d[6] = (uint8_t)((gm6020_cmd[3] >> 8) & 0xFF);
    d[7] = (uint8_t)( gm6020_cmd[3]       & 0xFF);
}

void GM6020_SendCurrent(int16_t cur1, int16_t cur2, int16_t cur3, int16_t cur4)
{
    uint8_t d[8];

    gm6020_cmd[0] = cur1;
    gm6020_cmd[1] = cur2;
    gm6020_cmd[2] = cur3;
    gm6020_cmd[3] = cur4;
    gm6020_pack(d);

    if (gm6020_tx_bus)
    {
        fdcanx_send_data(gm6020_tx_bus, GM6020_TX_ID_CUR_1_4, d, 8);
    }
    else
    {
        fdcanx_send_data(&hfdcan1, GM6020_TX_ID_CUR_1_4, d, 8);
        fdcanx_send_data(&hfdcan2, GM6020_TX_ID_CUR_1_4, d, 8);
        fdcanx_send_data(&hfdcan3, GM6020_TX_ID_CUR_1_4, d, 8);
    }
}

void GM6020_SendCurrentMA(float ma)
{
    GM6020_SendCurrent(GM6020_MaToRaw(ma), 0, 0, 0);
}

void GM6020_OnRx(FDCAN_HandleTypeDef *hfdcan, uint32_t id, const uint8_t *d)
{
    uint16_t raw;
    int16_t  diff;

    if (id != (GM6020_FB_ID_BASE + 1))      /* single motor, DIP ID = 1 */
        return;

    gm6020_tx_bus = hfdcan;                 /* remember live bus for TX  */

    raw = (uint16_t)((d[0] << 8) | d[1]);
    gm6020.angle_raw    = raw;
    gm6020.speed_rpm    = (int16_t)((d[2] << 8) | d[3]);
    gm6020.current_raw  = (int16_t)((d[4] << 8) | d[5]);
    gm6020.temp         = (int8_t)d[6];

    gm6020.angle_deg    = (float)raw * (360.0f / 8192.0f);
    gm6020.speed_rpm_f  = (float)gm6020.speed_rpm;
    gm6020.speed_rad_s  = (float)gm6020.speed_rpm * (6.28318530718f / 60.0f);
    gm6020.current_ma   = (float)gm6020.current_raw * (GM6020_CUR_MAX_MA / (float)GM6020_RAW_MAX);

    /* multi-turn unwrap */
    diff = (int16_t)(raw - gm6020.prev_raw);
    if (diff < -4096)       gm6020.turn_count++;      /* crossed 0 going +  */
    else if (diff > 4096)   gm6020.turn_count--;      /* crossed 0 going -  */
    gm6020.prev_raw = raw;
    gm6020.angle_unwrap = (float)gm6020.turn_count * 360.0f + gm6020.angle_deg;

    gm6020.last_rx_ms = HAL_GetTick();
    gm6020.online     = 1;
}

uint8_t GM6020_IsOnline(uint32_t timeout_ms)
{
    return (HAL_GetTick() - gm6020.last_rx_ms) <= timeout_ms;
}
