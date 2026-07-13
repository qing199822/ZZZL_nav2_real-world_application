#include "seasky.h"
#include <string.h>

/* ---- 发送回调 (USB CDC: CDC_Transmit_FS) ---- */
static seasky_send_fn_t seasky_send = NULL;


/*  CRC-8 查找表 (SHT75)                                            */

static const uint8_t crc8_table[256] = {
    0,   49,  98,  83,  196, 245, 166, 151, 185, 136, 219, 234, 125, 76,  31,  46,
    67,  114, 33,  16,  135, 182, 229, 212, 250, 203, 152, 169, 62,  15,  92,  109,
    134, 183, 228, 213, 66,  115, 32,  17,  63,  14,  93,  108, 251, 202, 153, 168,
    197, 244, 167, 150, 1,   48,  99,  82,  124, 77,  30,  47,  184, 137, 218, 235,
    61,  12,  95,  110, 249, 200, 155, 170, 132, 181, 230, 215, 64,  113, 34,  19,
    126, 79,  28,  45,  186, 139, 216, 233, 199, 246, 165, 148, 3,   50,  97,  80,
    187, 138, 217, 232, 127, 78,  29,  44,  2,   51,  96,  81,  198, 247, 164, 149,
    248, 201, 154, 171, 60,  13,  94,  111, 65,  112, 35,  18,  133, 180, 231, 214,
    122, 75,  24,  41,  190, 143, 220, 237, 195, 242, 161, 144, 7,   54,  101, 84,
    57,  8,   91,  106, 253, 204, 159, 174, 128, 177, 226, 211, 68,  117, 38,  23,
    252, 205, 158, 175, 56,  9,   90,  107, 69,  116, 39,  22,  129, 176, 227, 210,
    191, 142, 221, 236, 123, 74,  25,  40,  6,   55,  100, 85,  194, 243, 160, 145,
    71,  118, 37,  20,  131, 178, 225, 208, 254, 207, 156, 173, 58,  11,  88,  105,
    4,   53,  102, 87,  192, 241, 162, 147, 189, 140, 223, 238, 121, 72,  27,  42,
    193, 240, 163, 146, 5,   52,  103, 86,  120, 73,  26,  43,  188, 141, 222, 239,
    130, 179, 224, 209, 70,  119, 36,  21,  59,  10,  89,  104, 255, 206, 157, 172
};

static uint8_t crc_8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00;
    for (uint16_t i = 0; i < len; i++)
        crc = crc8_table[data[i] ^ crc];
    return crc;
}


/*  CRC-16 (Modbus, poly=0xA001, init=0xFFFF)                       */

static uint16_t crc16_table[256];
static uint8_t  crc16_ready = 0;

static void crc16_init(void)
{
    for (uint16_t i = 0; i < 256; i++)
    {
        uint16_t crc = 0, c = i;
        for (uint8_t j = 0; j < 8; j++)
        {
            if ((crc ^ c) & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc = crc >> 1;
            c = c >> 1;
        }
        crc16_table[i] = crc;
    }
    crc16_ready = 1;
}

static uint16_t crc_16(const uint8_t *data, uint16_t len)
{
    if (!crc16_ready) crc16_init();
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++)
        crc = (crc >> 8) ^ crc16_table[(crc ^ (uint16_t)data[i]) & 0x00FF];
    return crc;
}


/*  接收状态机                                                       */

typedef enum {
    STATE_SYNC = 0,
    STATE_HEADER,
    STATE_PAYLOAD,
} parser_state_t;

static parser_state_t rx_state = STATE_SYNC;
static uint8_t  rx_buf[SEASKY_FRAME_MAX];
static uint16_t rx_pos  = 0;

/* ---- 解析结果 ---- */
static fix_control_t latest_fix_ctrl;
static uint8_t       fix_ctrl_valid = 0;
static uint32_t      fix_ctrl_tick  = 0;   /* 最后收到的时间 */


/*  帧解析: 收完一帧后调用                                           */

static void frame_dispatch(const uint8_t *frame, uint16_t total_len)
{
    uint16_t payload_len = frame[1] | ((uint16_t)frame[2] << 8);
    uint16_t msg_id      = frame[4] | ((uint16_t)frame[5] << 8);
    const uint8_t *payload = frame + 8;

    switch (msg_id)
    {
    case MSG_ID_FIX_CONTROL:
        if (payload_len == 21)
        {
            memcpy(&latest_fix_ctrl, payload, 21);
            fix_ctrl_valid = 1;
            fix_ctrl_tick  = HAL_GetTick();
        }
        break;
    default:
        break;
    }
}


/*  喂一个字节到状态机                                               */

static void parser_feed(uint8_t byte)
{
    switch (rx_state)
    {
    case STATE_SYNC:
        if (byte == SEASKY_HEADER)
        {
            rx_buf[0] = byte;
            rx_pos = 1;
            rx_state = STATE_HEADER;
        }
        break;

    case STATE_HEADER:
        rx_buf[rx_pos++] = byte;
        if (rx_pos >= 4)
        {
            if (crc_8(rx_buf, 3) != rx_buf[3])
            {
                /* CRC8 失败, 从 byte[1] 找新帧头 */
                for (uint16_t i = 1; i < 4; i++)
                {
                    if (rx_buf[i] == SEASKY_HEADER)
                    {
                        memmove(rx_buf, rx_buf + i, 4 - i);
                        rx_pos = 4 - i;
                        return;
                    }
                }
                rx_state = STATE_SYNC;
                return;
            }

            uint16_t plen = rx_buf[1] | ((uint16_t)rx_buf[2] << 8);
            if (plen > SEASKY_PAYLOAD_MAX)
            {
                rx_state = STATE_SYNC;
                return;
            }
            rx_state = STATE_PAYLOAD;
        }
        break;

    case STATE_PAYLOAD:
        rx_buf[rx_pos++] = byte;
        {
            uint16_t plen = rx_buf[1] | ((uint16_t)rx_buf[2] << 8);
            uint16_t total = 10 + plen;
            if (rx_pos >= total)
            {
                uint16_t crc_calc = crc_16(rx_buf, total - 2);
                uint16_t crc_recv = rx_buf[total - 2] | ((uint16_t)rx_buf[total - 1] << 8);
                if (crc_calc == crc_recv)
                    frame_dispatch(rx_buf, total);
                rx_state = STATE_SYNC;
            }
        }
        break;
    }
}


/*  API: 初始化                                                      */

void Seasky_Init(seasky_send_fn_t send_fn)
{
    seasky_send = send_fn;
    rx_state = STATE_SYNC;
    rx_pos = 0;
    fix_ctrl_valid = 0;
    if (!crc16_ready) crc16_init();
}


/*  API: USB CDC 收到数据后喂入                                       */

void Seasky_FeedBytes(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
        parser_feed(data[i]);
}


/*  API: 获取最新 fix_control                                        */

const fix_control_t *Seasky_GetFixControl(void)
{
    if (fix_ctrl_valid && (HAL_GetTick() - fix_ctrl_tick) > 200)
        fix_ctrl_valid = 0;   /* 200ms没收新帧 → 超时失效 */
    return fix_ctrl_valid ? &latest_fix_ctrl : NULL;
}


/*  内部: 发送一帧                                                   */

static void seasky_send_frame(uint16_t msg_id, const uint8_t *payload, uint16_t len)
{
    if (!seasky_send) return;

    uint8_t frame[SEASKY_FRAME_MAX];
    uint16_t pos = 0;

    frame[pos++] = SEASKY_HEADER;
    frame[pos++] = (uint8_t)(len & 0xFF);
    frame[pos++] = (uint8_t)((len >> 8) & 0xFF);
    frame[pos++] = crc_8(frame, 3);
    frame[pos++] = (uint8_t)(msg_id & 0xFF);
    frame[pos++] = (uint8_t)((msg_id >> 8) & 0xFF);
    frame[pos++] = 0x00;
    frame[pos++] = 0x00;

    if (payload && len > 0)
        memcpy(frame + pos, payload, len);
    pos += len;

    uint16_t crc = crc_16(frame, pos);
    frame[pos++] = (uint8_t)(crc & 0xFF);
    frame[pos++] = (uint8_t)((crc >> 8) & 0xFF);

    seasky_send(frame, pos);
}

/*  API: 发送                                                       */

void Seasky_SendChassis(float vx, float vy, float wz)
{
    uint8_t p[12];
    memcpy(p + 0, &vx, 4);
    memcpy(p + 4, &vy, 4);
    memcpy(p + 8, &wz, 4);
    seasky_send_frame(MSG_ID_CHASSIS, p, 12);
}

void Seasky_SendSentry(float battery, float life, float color,
                       float bullet, float fault_flag)
{
    uint8_t p[20];
    memcpy(p + 0,  &battery,    4);
    memcpy(p + 4,  &life,       4);
    memcpy(p + 8,  &color,      4);
    memcpy(p + 12, &bullet,     4);
    memcpy(p + 16, &fault_flag, 4);
    seasky_send_frame(MSG_ID_SENTRY, p, 20);
}

void Seasky_SendHeartbeat(uint16_t timestamp, uint8_t set_launch,
                          uint8_t set_arm, uint8_t fault_flag)
{
    uint8_t p[5] = {
        (uint8_t)(timestamp & 0xFF),
        (uint8_t)((timestamp >> 8) & 0xFF),
        set_launch, set_arm, fault_flag
    };
    seasky_send_frame(MSG_ID_HEARTBEAT, p, 5);
}
