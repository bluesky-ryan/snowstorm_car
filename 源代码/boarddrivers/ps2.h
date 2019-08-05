#ifndef __PS2_H__
#define __PS2_H__
#include <board.h>
#include <rtthread.h>
#include <rtdevice.h>


struct ps2_table
{
    int ps2_cmd;
    int standard_cmd;
};

// PIN
#define PS2_DO_PIN      GET_PIN(A, 7)
#define PS2_DI_PIN      GET_PIN(A, 6)
#define PS2_CS_PIN      GET_PIN(C, 2)
#define PS2_SCK_PIN     GET_PIN(A, 5)


// COMMAND
#define PS2_CMD_VIBRATE     1

// MODE
#define PS2_NO_MODE         0
#define PS2_GREEN_MODE      0x41
#define PS2_RED_MODE        0x73

// KEY
#define PS2_BTN_SELECT      (1 << 0)
#define PS2_BTN_L3          (1 << 1)
#define PS2_BTN_R3          (1 << 2)
#define PS2_BTN_START       (1 << 3)
#define PS2_BTN_UP          (1 << 4)
#define PS2_BTN_RIGHT       (1 << 5)
#define PS2_BTN_DOWN        (1 << 6)
#define PS2_BTN_LEFT        (1 << 7)
#define PS2_BTN_L2          (1 << 8)
#define PS2_BTN_R2          (1 << 9)
#define PS2_BTN_L1          (1 << 10)
#define PS2_BTN_R1          (1 << 11)
#define PS2_BTN_TRIANGLE    (1 << 12)
#define PS2_BTN_CICLE       (1 << 13)
#define PS2_BTN_FORK        (1 << 14)
#define PS2_BTN_SQUARE      (1 << 15)


#define PS2_KEY_COUNTS       20

typedef enum
{
    PS2_KEY_SELECT      = 0,
    PS2_KEY_L3          = 1,
    PS2_KEY_R3          = 2,
    PS2_KEY_START       = 3,
    PS2_KEY_PAD_UP      = 4,
    PS2_KEY_PAD_RIGHT   = 5,
    PS2_KEY_PAD_DOWN    = 6,
    PS2_KEY_PAD_LEFT    = 7,
    PS2_KEY_L2          = 8,
    PS2_KEY_R2          = 9,
    PS2_KEY_L1          = 10,
    PS2_KEY_R1          = 11,
    PS2_KEY_GREEN       = 12,
    PS2_KEY_RED         = 13,
    PS2_KEY_BLUE        = 14,
    PS2_KEY_PINK        = 15,
    PS2_KEY_ROCKER_LX   = 16,
    PS2_KEY_ROCKER_LY   = 17,
    PS2_KEY_ROCKER_RX   = 18,
    PS2_KEY_ROCKER_RY   = 19,
}ps2_key_index;

typedef struct
{ 
    uint8_t     key_value[PS2_KEY_COUNTS];  /* 获取对应键值key_value[ps2_key_index] */
}ps2_keys_data_t;


typedef struct 
{
    uint16_t button;        // 16
    uint8_t left_stick_x;
    uint8_t left_stick_y;
    uint8_t right_stick_x;
    uint8_t right_stick_y;
}ps2_ctrl_data_t;

uint8_t         ps2_get_mode(void);
ps2_keys_data_t ps2_get_current_keys(void);
ps2_ctrl_data_t ps2_get_current_ctrl(void);


#endif
