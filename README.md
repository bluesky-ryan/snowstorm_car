# RT-Thread robot-car制作

**_1.小车制作仓_**

*https://github.com/bluesky-ryan/snowstorm_car*

***2.官方Robort-car连接***

*https://github.com/RT-Thread-packages/rt-robot*

***3.RK3399 ROS系统部分代码地址***

*https://github.com/bluesky-ryan/snowstorm_ros_rk3399/tree/master*



## 概述

- 最近有幸参加了一期RT-Thread官方发起的rt-robot car DIY活动，跟着大神们的步伐我也成功的做出了一辆麦克纳姆轮PS2遥控车，心里非常的Happy，特意记录了这个制作过程用作给小白们借鉴。
- 不多逼逼了，来开始我们造车之旅。



## 选材

初次探索智能车本着节约成本和最低风险的原则，我们尽量选用现成的硬件材料。在探索成功后学会了理解了其中的原理，再根据自己的需求完全设计自己的小车。

#### 1.主控板

我们选用淘宝成品主控板（主控芯片STM32F103RCT6）

图样：

![control](https://github.com/bluesky-ryan/snowstorm_car/blob/master/Image/control.png)

#### 2.底座

麦克纳姆轮底座，某宝多的是自行选购

样图：

![car](https://github.com/bluesky-ryan/snowstorm_car/blob/master/Image/car.png)

#### 3.电机

买底座基本都带电机，我们选用带AB编码器的1：30减速电机

样图：

![motor](https://github.com/bluesky-ryan/snowstorm_car/blob/master/Image/motor.png)

#### 4.遥控

普通SONY PS2遥控30-40块钱

样图：

![ps2](https://github.com/bluesky-ryan/snowstorm_car/blob/master/Image/ps2.png)

#### 5.电池

选用3S 11V航模电池

样图：

![batter](https://github.com/bluesky-ryan/snowstorm_car/blob/master/Image/batter.png)

6.USB转串口线一根

样图：

![usbtouart](https://github.com/bluesky-ryan/snowstorm_car/blob/master/Image/usbtouart.png)



## 核心知识点

1.RT-Thread bsp移植。

2.STM32-CubeMXs使用。

3.RTOS使用。

4.PID控制理论。

5.麦克拉姆拉控制理论。

6.简单运动模型



## 开发环境

- 使用Keil V5作为编译器

- 使用 rt-thread 最新版本

- 使用自己移植的 bsp

  

## 详细步骤

- #### BSP移植

  1. 关于BSP的移植RT-Thread官网有非常详细的文档描述：

     https://github.com/RT-Thread/rt-thread/blob/master/bsp/stm32/docs/STM32系列BSP制作教程.md

  2. 移植过程不做累述，按照官方的步骤一步一步的走即可。

  3. 先只配置控制台串口和系统呼吸灯：

     - console串口：UART2

     - 系统LED灯：PD2

  

- #### 电机控制

  - 主控板电机驱动芯片为TI的DIV8833芯片，一颗DIV8833芯片可以驱动两个电机，我们有4个电机用到了2个芯片。芯片采用对偶PWM方波输入驱动，频率手册没写，我们先使用10KHZ。

  - STM32F103RTC6高级定时器1、8都带有对偶PWM输出，我们的主控板用的高级定时器1，悲剧的是定时器1只能输出3路对偶PWM方波和1路普通PWM方波，可是我们有4个轮子，所以最后一个轮子的对偶极只能GPIO来代替控制了。

  - 驱动逻辑表：

    ![1565942461038](https://github.com/bluesky-ryan/snowstorm_car/blob/master/Image/drv8833-pwm.png)

  - 驱动资料有了逻辑也清晰了，我们接下要做的就只是按照要求输出几个PWM方波了

    - 第一步：CubMX配置定时器1为PWM对偶模式

      ![](https://github.com/bluesky-ryan/snowstorm_car/blob/master/Image/cubemx-time1.png)

    - 第二步：封装初始化、通道控制等电机控制接口（具体封装参照源码motor.c文件），最后给上层提供一个初始化接口，一个通道速度控制接口。

      ```
      /**
      *@ingroup motor
      *
      *初始化定时器
      *@param  none
      *@retrun none
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
      ```

      ```
      /**
      *@ingroup motor
      *
      *控制电动机标量控制，正直表示正转，负值表示反转
      *
      *@param  ch    控制通道MOTOR_CH1/TMOTOR_CH2/TMOTOR_CH3/MOTOR_CH4，可组合使用MOTOR_CH_1|MOTOR_CH_2
      *@param  speed pwm控制量[-1000, 1000]
      *@retrun none
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
      ```

    - 第三步：把主要函数加入Finsh控制台命令中，通过命令调试控制效果

      ```
      /* FINSH 调试函数 */
      #ifdef RT_USING_FINSH
      #include <finsh.h>
      FINSH_FUNCTION_EXPORT_ALIAS(motor_pwm_control, motor_control, channel direction speed);
      FINSH_FUNCTION_EXPORT_ALIAS(motor_pwm_set, motor_set, channel speed);
      
      /* FINSH 调试命令 */
      #ifdef FINSH_USING_MSH
      
      #endif /* FINSH_USING_MSH */
      #endif /* RT_USING_FINSH */
      ```

      

    - 第四步：通过Finsh控制台调试命令测试电机通道和PWM控制量

      ![1565968683143](https://github.com/bluesky-ryan/snowstorm_car/blob/master/Image/pwm-set)

- #### 编码器数据获取

  - 电机测速我们使用520电机自带的AB相霍尔编码器，编码器线数为390，4倍线数后轮子转一圈收到：390*4=1560个脉冲。

  - stm32自带AB相霍尔解码器，一个通道需要消耗一个定时器。我们主控板电机2、3、4使用的timer 3/4/5硬件解码，电机1没有接定时器，坑爹啊，那只能用外部中断根据时序解码。

  - 编码器时序：

    ![1565945676024](https://github.com/bluesky-ryan/snowstorm_car/blob/master/Image/ab-huoer-encode)

  - 资料有了思路也清晰了，接下来我们要做的只是初始化一下解码器，把实时编码数读出即可

    - 第一步：CubeMx配置解码定时器和中断

      ![1565945920922](https://github.com/bluesky-ryan/snowstorm_car/blob/master/Image/cubemx-encode-timer.png)

      

    - 第二步：编写初始化函数和编码器数据获取函数，电机1使用中断解码，电机2、3、4使用定时器解码。（具体代码参照github上面源码，这里不再累述）

          /* TIM init */
          moto_pwm_init();
          motor_encode2_init();
          motor_encode3_init();
          motor_encode4_init();
          motor_encode_enable();
          
          LOG_I("motor initialization completed.\r\n");

    - 第三步：加入Finsh调试函数，旋转轮子查看编码值是否准确。（输入 motor_test -ge实时查看编码器值）

      ```
      MSH_CMD_EXPORT(motor_test, motor_test -ge/-q);
      ```

    ![1565968589135](https://github.com/bluesky-ryan/snowstorm_car/blob/master/Image/motor_test)

- #### PID

  - 有了编码器作为反馈器，有了PWM作为控制器，那我们就可以加入PID控制器了。加入PID控制器的目的是精确控制轮子的速度，提供轮子的控制达到一致。

  - PID原理知识自行百度网上资料一大把，个人理解是：

    - 比例Kp: 粗调，大幅度调节控制量让测量值逼近理论值，但是由于单位较大无法精确到达理论值，有响应快，调节尺度大的特点。
    - 微分Kd: 状态预测，Kd控制的是速度的斜率相当于预测下一步速度的趋势，可以加快调节速度。
    - 积分Ki：细调， 通过微小的积分累加，让测量值不断逼近理论值，细调控制量让测量值逼近理论值。

  - PID框图：

    ![img](https://github.com/bluesky-ryan/snowstorm_car/blob/master/Image/pid-ctr)

    ![img](https://github.com/bluesky-ryan/snowstorm_car/blob/master/Image/pid-ctr1) 

  - PID公式：

  ![img](https://github.com/bluesky-ryan/snowstorm_car/blob/master/Image/pid-company)

  - 理论知识有了，按照公式做个具体实现就好了

    第一步：实现增量PID刷新公式，具体查看源码（pid.c）

    ```
    float pid_update(pid_control_t* pid, float measure_value)
    ```

    第二步：将测量值输出到虚拟波形器软件上面便于观测各个值的当前情况，作者使用的是《山外多功能调试助手》，直接将数据输出到控制台串口。数据发送接口实现如下：

    ```
    /* 输出数据到虚拟波形软件 */
    rt_err_t send_waveform_fomate(void *buf, uint32_t size)
    {
        const char start[2] = {0x03, 0xfc};
        const char end[2]   = {0xfc, 0x03};
        rt_device_t console = rt_console_get_device();
        
    	rt_device_write(console, -1, start, 2);	//发送起始字符
    	rt_device_write(console, -1, buf, size);//发送通道数据
    	rt_device_write(console, -1, end, 2);	//发送结束字符
    
    	return RT_EOK;
    }
    ```

    ![1565964410845](https://github.com/bluesky-ryan/snowstorm_car/blob/master/Image/pid-measure)

    第三步：调节合适的速度刷新周期和PID刷新周期，周期不合适电机会剧烈抖动。作者设置周期为：

    ```
    p_car->pid_sample_time = 20;             /* PID刷新间隔ms */
    p_car->vct_sample_time = 10;             /* 速度刷新间隔ms */
    ```

    第四步：调试合适的PID参数，由于作者选用电机一致性不好，所以设置参数时每个轮子正传和反转的PID参数都是独立的。具体实现查看代码：

    ```
    wheel_select_pid_kx(&p_car->m_wheel[i]); /* 根据速度设置PID参数 */
    ```

    作者样车PID参数：

    ```
    #define CHX_PID_KX_TABLE      \
    { \
        {MOTOR_CH1, {1.100, 0.400, 0.500}, {1.100, 0.400, 0.500}},\
        {MOTOR_CH2, {1.100, 0.400, 0.500}, {1.100, 0.400, 0.500}},\
        {MOTOR_CH3, {1.100, 0.400, 0.500}, {1.100, 0.400, 0.500}},\
        {MOTOR_CH4, {0.765, 0.330, 0.100}, {0.260, 0.200, 0.010}},\
    }
    ```

- #### PS2遥控器

  - PS2 由手柄与接收器两部分组成，手柄主要负责发送按键信息。都接通电源并打开手柄开关时，手柄与接收器自动配对连接，在未配对成功的状态下，接收器绿灯闪烁，手柄上的灯也会闪烁，配对成功后，接收器上绿灯常亮.

  - PS2遥控有两个模式一个红灯模式、绿灯模式，区别就是红灯模式遥控输出的是模拟值，绿灯输出的只有最大值。我们输出固定速度选用绿灯模式。

  - PS2传输协议有点像SPI，不同的是PS2每次传输数据帧都是9个字节，里面包含了各个按键的当前值。

  - PS2更多详细信息查看:[ps2解码通讯手册V1.5.pdf](https://github.com/bluesky-ryan/snowstorm_car/blob/master/资料文档/ps2解码通讯手册V1.5.pdf)

  - PS2时序图：

    ![1565965439212](https://github.com/bluesky-ryan/snowstorm_car/blob/master/Image/ps2-time)

  - PS2我们只需要读取遥控数据，一个扫描函数搞定，定期刷新一个按键值即可，具体代码参照（ps2.c）：

    ```
    int ps2_scan(ps2_ctrl_data_t *pt)
    ```

- #### 遥控功能

  - 最后只剩遥控功能了，我们只需要将PS2遥控的当前值映射到对应控制值，再将对应控制值映射到轮子即可。

  - 车子方向控制图：

    ![麦克纳姆轮方向控制](https://github.com/bluesky-ryan/snowstorm_car/blob/master/Image/麦克纳姆轮方向控制.gif)

  - 遥控映射到控制值：

    ```
    /* PS映射car cmd表格，组合键命令在头添加，单键命令放后面 */
    car_ps2_cmd_t ps2_to_cmd_table[] = {
        {PS2_BTN_RIGHT| PS2_BTN_UP,     CAR_CMD_FORWARD_RIGHT},
        {PS2_BTN_LEFT | PS2_BTN_UP,     CAR_CMD_FORWARD_LEFT},
        {PS2_BTN_RIGHT| PS2_BTN_DOWN,   CAR_CMD_BACK_RIGHT},
        {PS2_BTN_LEFT | PS2_BTN_DOWN,   CAR_CMD_BACK_LEFT},
        {PS2_BTN_UP,                    CAR_CMD_FORWARD},
        {PS2_BTN_DOWN,                  CAR_CMD_BACK},
        {PS2_BTN_RIGHT,                 CAR_CMD_RIGHT},
        {PS2_BTN_LEFT,                  CAR_CMD_LEFT},
        {PS2_BTN_CICLE,                 CAR_CMD_TURN_RIGHT},
        {PS2_BTN_SQUARE,                CAR_CMD_TURN_LEFT},
    };
    ```

  - 控制映射到4个轮子的具体速度值：

    ```
    /* 命令映射到几何控制参数 */
    car_cmd_math_t cmd_to_math_table[] = {
    {CAR_CMD_INVALID,       { 0,      0,    0,    0}},
    {CAR_CMD_STOP,          { 0,      0,    0,    0}},
    {CAR_CMD_FORWARD_LEFT,  {   0,  120,  120,    0}},
    {CAR_CMD_FORWARD_RIGHT, { 120,    0,    0,  120}},
    {CAR_CMD_BACK_LEFT,    	{-120,    0,    0, -120}},
    {CAR_CMD_BACK_RIGHT,    {   0, -120, -120,    0}}, 
    {CAR_CMD_FORWARD,       { 120,  120,  120,  120}},
    {CAR_CMD_BACK,          {-120, -120, -120, -120}},
    {CAR_CMD_RIGHT,         { 120, -120, -120,  120}},
    {CAR_CMD_LEFT,          {-120,  120,  120, -120}},
    {CAR_CMD_TURN_RIGHT,    { 120, -120,  120, -120}},
    {CAR_CMD_TURN_LEFT,    	{-120,  120, -120,  120}},
    };
    ```

  - 再开一个线程定期刷新各个轮子的控制即可。

## 视频预览

PS2遥控小车视频地址：https://www.bilibili.com/video/BV1fU4y187hK/

RK3399 ROS路径规划视频地址：https://www.bilibili.com/video/BV1By4y1x735

<video src="https://v.youku.com/v_show/id_XNDMxNzQ4MDU1Mg==.html?spm=a2h0j.11185381.listitem_page1.5~A"></video>

## 经验总结

- 电机控制电路设计上应该与控制板完全隔离，比如光耦隔离器件，避免电流压降造成主控不稳定。
- 主控板需要有较强的抗大电流和抗干扰性，一块好的主板事半功倍，主动不稳定容易出现未知问题很难定位。
- PDI控制环节速度应尽量使用瞬时速度，也就是说在保证精度的情况下刷新时间要尽量的短。
