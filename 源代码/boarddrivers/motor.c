#include "motor.h"

//#define DRV_DEBUG
#define LOG_TAG             "motor"
#include <drv_log.h>


typedef enum {
    MOTOR_DIR_STOP,
    MOTOR_DIR_FORWARD,
    MOTOR_DIR_REVERSE,
}motor_dir;

static TIM_HandleTypeDef htim1 = {0};
static TIM_HandleTypeDef htim3 = {0};
static TIM_HandleTypeDef htim4 = {0};
static TIM_HandleTypeDef htim5 = {0};

static int16_t __encode1_cnt = 0;

extern void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
static void encode_1a_irq_callback(void *argv);
static void encode_1b_irq_callback(void *argv);


/**
 * @ingroup motor
 *
 * 初始化定时器
 * 
 * @param  none
 * @retrun none
 */
static void moto_pwm_init(void)
{

    /* USER CODE BEGIN TIM1_Init 0 */

    /* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 71;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = MOTOR_PWM_MAX - 1;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM1_Init 2 */

    /* USER CODE END TIM1_Init 2 */
    HAL_TIM_MspPostInit(&htim1);
    
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    LOG_I("motor pwm initialization ok.\r\n");
    
}

static void motor_gpio_init(void)
{
    rt_pin_mode(CH4N_PIN,  PIN_MODE_OUTPUT);
    rt_pin_write(CH4N_PIN, PIN_LOW);

    /* EXTI interrupt init*/
    rt_pin_attach_irq(ENCODE_1A_PIN, PIN_IRQ_MODE_RISING_FALLING, encode_1a_irq_callback, (void *)&__encode1_cnt);
    rt_pin_attach_irq(ENCODE_1B_PIN, PIN_IRQ_MODE_RISING_FALLING, encode_1b_irq_callback, (void *)&__encode1_cnt);
    rt_pin_irq_enable(ENCODE_1A_PIN, RT_TRUE);
    rt_pin_irq_enable(ENCODE_1B_PIN, RT_TRUE); 

    LOG_I("motor gpio initialization ok.\r\n");
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void motor_encode2_init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  LOG_I("motor encode2 initialization ok.\r\n");

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void motor_encode3_init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
    LOG_I("motor encode3 initialization ok.\r\n");
}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void motor_encode4_init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  LOG_I("motor encode4 initialization ok.\r\n");
}

static void motor_encode_enable(void)
{
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
}

static void encode_1a_irq_callback(void *argv)
{
    int32_t pin_a = rt_pin_read(ENCODE_1A_PIN);
    int32_t pin_b = rt_pin_read(ENCODE_1B_PIN);
 
    if(pin_a == pin_b)
    	--*(int16_t*)(argv);
    else
    	++*(int16_t*)(argv);

//    rt_kprintf("ENCODE 1A out count:%d PA:%d  PB:%d\r\n", *(int16_t*)(argv), pin_a, pin_b);
}

static void encode_1b_irq_callback(void *argv)
{
    int32_t pin_a = rt_pin_read(ENCODE_1A_PIN);
    int32_t pin_b = rt_pin_read(ENCODE_1B_PIN);

    if(pin_a == pin_b)
    	++*(int16_t*)(argv);
    else
    	--*(int16_t*)(argv);
    
 //   rt_kprintf("ENCODE 1B out count:%d PA:%d  PB:%d\r\n", *(int16_t*)(argv), pin_a, pin_b);
}

/**
 * @ingroup motor
 *
 * 初始化电动机
 *
 * @param  none
 * @retrun none
 */
int motor_init(void)
{
    /* GPIO init */
    motor_gpio_init();
    
    /* TIM init */
    moto_pwm_init();
    motor_encode2_init();
    motor_encode3_init();
    motor_encode4_init();
    motor_encode_enable();

    LOG_I("motor initialization completed.\r\n");
	
	return RT_EOK;
}

/**
 * @ingroup motor
 *
 * 控制电动机
 *
 * @param  ch       控制通道MOTOR_CH1/TMOTOR_CH2/MOTOR_CH3/TMOTOR_CH4，可组合使用MOTOR_CH1|TMOTOR_CH2
 * @param  dir      运动方向
 * @param  speed    pwm控制量0-1440
 * @retrun none
 */
void motor_pwm_control(uint32_t ch, motor_dir dir, uint16_t speed)
{
    /* 通道1控制 */
    if (TIM_CHANNEL_1 == ch)
    {
         /* 通道停止 */
        if (MOTOR_DIR_STOP == dir)
        {
            __HAL_TIM_SET_COMPARE(&htim1, ch, 0);
            __HAL_TIM_SET_COUNTER(&htim1, 0);
            HAL_TIM_PWM_Stop(&htim1, ch);
            HAL_TIMEx_PWMN_Stop(&htim1, ch);
        }
        /* 通道前进 */
        else if (MOTOR_DIR_FORWARD == dir)
        {
            __HAL_TIM_SET_COMPARE(&htim1, ch, speed);
            HAL_TIM_PWM_Start(&htim1, ch);
            HAL_TIMEx_PWMN_Stop(&htim1, ch);  
        }
        /* 通道后退 */
        else if (MOTOR_DIR_REVERSE == dir)
        {
            __HAL_TIM_SET_COMPARE(&htim1, ch, speed);
            HAL_TIM_PWM_Stop(&htim1, ch);
            HAL_TIMEx_PWMN_Start(&htim1, ch);  
        }
    }
    /* 通道2控制 */
    if (TIM_CHANNEL_2 == ch)
    {
         /* 通道停止 */
        if (MOTOR_DIR_STOP == dir)
        {
            __HAL_TIM_SET_COMPARE(&htim1, ch, 0);
            __HAL_TIM_SET_COUNTER(&htim1, 0);
            HAL_TIM_PWM_Stop(&htim1, ch);
            HAL_TIMEx_PWMN_Stop(&htim1, ch);
        }
        /* 通道前进 */
        else if (MOTOR_DIR_FORWARD == dir)
        {
            __HAL_TIM_SET_COMPARE(&htim1, ch, speed);
            HAL_TIM_PWM_Stop(&htim1, ch);
            HAL_TIMEx_PWMN_Start(&htim1, ch);  
        }
        /* 通道后退 */
        else if (MOTOR_DIR_REVERSE == dir)
        {
            __HAL_TIM_SET_COMPARE(&htim1, ch, speed);
            HAL_TIM_PWM_Start(&htim1, ch);
            HAL_TIMEx_PWMN_Stop(&htim1, ch);  
        }
    }
    /* 通道3控制 */
    if (TIM_CHANNEL_3 == ch)
    {
         /* 通道停止 */
        if (MOTOR_DIR_STOP == dir)
        {
            __HAL_TIM_SET_COMPARE(&htim1, ch, 0);
            __HAL_TIM_SET_COUNTER(&htim1, 0);
            HAL_TIM_PWM_Stop(&htim1, ch);
            HAL_TIMEx_PWMN_Stop(&htim1, ch);
        }
        /* 通道前进 */
        else if (MOTOR_DIR_FORWARD == dir)
        {
            __HAL_TIM_SET_COMPARE(&htim1, ch, speed);
            HAL_TIM_PWM_Stop(&htim1, ch);
            HAL_TIMEx_PWMN_Start(&htim1, ch);  
        }
        /* 通道后退 */
        else if (MOTOR_DIR_REVERSE == dir)
        {
            __HAL_TIM_SET_COMPARE(&htim1, ch, speed);
            HAL_TIM_PWM_Start(&htim1, ch);
            HAL_TIMEx_PWMN_Stop(&htim1, ch);  
        }
    }
    /* CH4使用CH/GPIO输出 */
    if (TIM_CHANNEL_4 == ch)
    {
        /* 通道停止 */
        if (MOTOR_DIR_STOP == dir)
        {
            __HAL_TIM_SET_COMPARE(&htim1, ch, 0);
            __HAL_TIM_SET_COUNTER(&htim1, 0);
            HAL_TIM_PWM_Stop(&htim1, ch);
            rt_pin_write(CH4N_PIN, PIN_LOW);  
        }
        /* 通道前进 */
        else if (MOTOR_DIR_FORWARD == dir)
        {
            __HAL_TIM_SET_COMPARE(&htim1, ch, speed);
            HAL_TIM_PWM_Start(&htim1, ch); 
            rt_pin_write(CH4N_PIN, PIN_LOW); 
        }
        /* 通道后退 */
        else if (MOTOR_DIR_REVERSE == dir)
        {
            __HAL_TIM_SET_COMPARE(&htim1, ch, MOTOR_PWM_MAX-speed);
            HAL_TIM_PWM_Start(&htim1, ch); 
            rt_pin_write(CH4N_PIN, PIN_HIGH);
        }
        
    }
}

/**
 * @ingroup motor
 *
 * 控制电动机标量控制，正直表示正转，负值表示反转
 *
 * @param  ch       控制通道MOTOR_CH1/TMOTOR_CH2/TMOTOR_CH3/MOTOR_CH4，可组合使用MOTOR_CH_1|MOTOR_CH_2
 * @param  speed    pwm控制量[-1000, 1000]
 * @retrun none
 */
void motor_pwm_set(motor_chx ch,  int16_t speed)
{
    
    /* 反转 */
    if (0 > speed)
    {
        if (-MOTOR_PWM_MAX > speed)
            speed = -MOTOR_PWM_MAX;
        
        motor_pwm_control(ch, MOTOR_DIR_REVERSE, -speed);

    }
    /* 正转 */
    else if (0 < speed)
    {
        if (MOTOR_PWM_MAX < speed)
            speed = MOTOR_PWM_MAX;
        motor_pwm_control(ch, MOTOR_DIR_FORWARD, speed);
    }
    /* 停止 */
    else
    {
        motor_pwm_control(ch, MOTOR_DIR_STOP, speed);
    }
}

/**
 * @ingroup motor
 *
 * 控制电动机速度编码获取
 *
 * @param  encode   编码结构体接收数据
 * @retrun none
 */
void motor_encode_all_get(encode_t *encode)
{
	encode->count[0] = __encode1_cnt;
	encode->count[1] = -(int16_t)(__HAL_TIM_GET_COUNTER(&htim3));
	encode->count[2] = (int16_t)(__HAL_TIM_GET_COUNTER(&htim4));
	encode->count[3] = (int16_t)(__HAL_TIM_GET_COUNTER(&htim5));

	__encode1_cnt = 0;
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	__HAL_TIM_SET_COUNTER(&htim5, 0);
    LOG_I("encode1:%d encode2:%d encode3:%d encode4:%d", encode->count[0], encode->count[1], encode->count[2], encode->count[3]);
}

/**
 * @ingroup motor
 *
 * 获取指定通道编码器值
 *
 * @param  chx      电机通道MOTOR_CH1、MOTOR_CH2、MOTOR_CH3、MOTOR_CH4
 * @param  value    编码计数值
 * @retrun none
 */
void motor_encode_chx_get(motor_chx ch, int16_t *value)
{
    switch(ch)
    {
        case MOTOR_CH1:
            *value = __encode1_cnt;
            __encode1_cnt = 0;
        break;
        case MOTOR_CH2:
            *value = -(int16_t)(__HAL_TIM_GET_COUNTER(&htim3));
            __HAL_TIM_SET_COUNTER(&htim3, 0);
        break;
        case MOTOR_CH3:
            *value = (int16_t)(__HAL_TIM_GET_COUNTER(&htim4));
            __HAL_TIM_SET_COUNTER(&htim4, 0);
        break;
        case MOTOR_CH4:
            *value = -(int16_t)(__HAL_TIM_GET_COUNTER(&htim5));
            __HAL_TIM_SET_COUNTER(&htim5, 0);
        break;
    }
}


/* FINSH 调试函数 */
#ifdef RT_USING_FINSH
#include <finsh.h>
FINSH_FUNCTION_EXPORT_ALIAS(motor_pwm_control, motor_control, channel direction speed);
FINSH_FUNCTION_EXPORT_ALIAS(motor_pwm_set, motor_set, channel speed);


/* FINSH 调试命令 */
#ifdef FINSH_USING_MSH
struct motor_test_t {
    rt_thread_t thread;
    encode_t    encode;
    char        cmd[50];
};

struct motor_test_t m_test = {
    .thread = RT_NULL,
    .encode = {0},
    .cmd    = {0},
};
static void motor_test_entry(void *parameter)
{  
    while (1)
    {
        if (0 == strcmp(m_test.cmd, "-ge"))
        {
            motor_encode_all_get(&m_test.encode);
            LOG_I("encode1:%d encode2:%d encode3:%d encode4:%d", m_test.encode.count[0], m_test.encode.count[1], m_test.encode.count[2], m_test.encode.count[3]);
        }

        if (0 == strcmp(m_test.cmd, "-q"))
        {
            if (RT_NULL != m_test.thread)
            {
                rt_thread_delete(m_test.thread);
                m_test.thread = RT_NULL;
                LOG_I("motor test process deleted!\r\n");
            }
            
        }
        rt_thread_mdelay(500);
    }
}

int motor_test(int argc, char **argv)
{
    if (RT_NULL == m_test.thread)
    {
        m_test.thread = rt_thread_create("motor_test",
                                        motor_test_entry, NULL,
                                        1024,
                                        15,
                                        5);
        if (m_test.thread != RT_NULL)
            rt_thread_startup(m_test.thread);
        else
            return -RT_ERROR;
    }

    memset(m_test.cmd, 0, sizeof(m_test.cmd));
    strncpy(m_test.cmd, argv[1], sizeof(m_test.cmd));
        
    return RT_EOK;
}
MSH_CMD_EXPORT(motor_test, motor_test -ge/-q);

#endif /* FINSH_USING_MSH */
#endif /* RT_USING_FINSH */

//INIT_APP_EXPORT(motor_init);
