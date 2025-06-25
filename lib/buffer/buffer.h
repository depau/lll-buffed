/**
  ***************************************************************************************
  * @file    buffer.cpp
  * @author  lijihu
  * @version V1.0.0
  * @date    2025/05/10
  * @brief   实现缓冲器功能
        *缓冲器说明
        光感：遮挡1，不遮挡0；
        耗材开关：有耗材0，无耗材1；
        按键：按下0，松开1；

        引脚：
        HALL1 --> PB2 (光感3)
        HALL2 --> PB3 (光感2)
        HALL3 --> PB4 (光感1)
        ENDSTOP_3 --> PB7(耗材开关)
        KEY1 --> PB13(后退)
        KEY2 --> PB12(前进)
  *
  * @note
  ***************************************************************************************
  * 版权声明 COPYRIGHT 2024 xxx@126.com
  ***************************************************************************************
**/

#ifndef __BUFFER_H__
#define __BUFFER_H__

#include <Arduino.h>
#include <EEPROM.h>
#include <TMCStepper.h>

#define HALL1 PB2 // 光感3
#define HALL2 PB3 // 光感2
#define HALL3 PB4 // 光感1

#define ENDSTOP_3 PB7 // 耗材开关

#define KEY1 PB13 // 后退
#define KEY2 PB12 // 前进

#define EN_PIN PA6 // 使能
#define DIR_PIN PA7 // 方向
#define STEP_PIN PC13 // 步
#define UART PB1 // 软串口

#define DUANLIAO PB15 // 断料
#define ERR_LED PA15 // 指示灯
#define START_LED PA8 // 指示灯

#define DRIVER_ADDRESS 0b00 // TMC Driver address according to MS1 and MS2
#define R_SENSE 0.11f // Match to your driver

#define SPEED 300 // 转速(单位：r/min)
#define Move_Divide_NUM ((int32_t) (64)) //(每步柔性件控制细分量)
#define VACTRUAL_VALUE (uint32_t) (SPEED * Move_Divide_NUM * 200 / 60 / 0.715) // VACTUAL寄存器值

#define STOP 0 // 停止
#define I_CURRENT (600) // 电流
#define WRITE_EN_PIN(x) digitalWrite(EN_PIN, x) // 使能EN引脚
#define FORWARD 1 // 耗材方向
#define BACK 0

#define DEBUG 0

// 定义结构体存储缓冲器中各传感器的的状态
typedef struct Buffer {
  // buffer1
  bool buffer1_pos1_sensor_state;
  bool buffer1_pos2_sensor_state;
  bool buffer1_pos3_sensor_state;
  bool buffer1_material_swtich_state;
  bool key1;
  bool key2;

} Buffer;

// 电机状态控制枚举
typedef enum {
  Forward = 0, // 向前
  Stop, // 停止
  Back // 后退
} Motor_State;

extern void buffer_sensor_init();
extern void buffer_motor_init();

extern void read_sensor_state(void);
extern void motor_control(void);

extern void buffer_init();
extern void buffer_loop(void);
extern void timer_it_callback();
extern void buffer_debug(void);

extern bool is_error;
extern uint32_t front_time; // 前进时间
extern uint32_t timeout;
extern bool is_front;
extern TMC2209Stepper driver;

#endif