#ifndef __MOTOR_H__
#define __MOTOR_H__
#include <stm32f1xx.h>
#include <board.h>

#define ARRAY_SIZE(ar) (sizeof(ar) / sizeof(ar[0]))

#define CH4N_PIN            GET_PIN(B, 12)
#define ENCODE_1A_PIN       GET_PIN(A, 15)
#define ENCODE_1B_PIN       GET_PIN(B, 2)


typedef enum {
    MOTOR_CH1     =      TIM_CHANNEL_1,
    MOTOR_CH2     =      TIM_CHANNEL_3,
    MOTOR_CH3     =      TIM_CHANNEL_2,
    MOTOR_CH4     =      TIM_CHANNEL_4,
}motor_chx;

#define MOTOR_PWM_MAX       120


typedef struct {
    int16_t     count[4];
}encode_t;

int  motor_init(void);
void motor_encode_all_get(encode_t *encode);
void motor_encode_chx_get(motor_chx ch, int16_t *value);
void motor_pwm_set(motor_chx ch,  int16_t speed);

#endif
