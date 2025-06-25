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
  * 版权声明 COPYRIGHT 2025 xxx@126.com
  ***************************************************************************************
**/

#include "buffer.h"
TMC2209Stepper driver(UART, UART, R_SENSE, DRIVER_ADDRESS);
Buffer buffer = { 0 }; // 存储个传感器状态
Motor_State motor_state = Stop;

bool is_front = false; // 前进标志位
uint32_t front_time = 0; // 前进时间
const int EEPROM_ADDR_TIMEOUT = 0;
const uint32_t DEFAULT_TIMEOUT = 30000;
uint32_t timeout = 30000; // 超时时间，单位：ms;
bool is_error = false; // 错误标志位，如果连续30s推送耗材没停过，则认为错误
String serial_buf;

static HardwareTimer timer(TIM6); // 超时出错

void buffer_init() {
  buffer_sensor_init();
  buffer_motor_init();
  delay(1000);

  EEPROM.get(EEPROM_ADDR_TIMEOUT, timeout);
  // 判断读取的值是否有效（例如首次写入前是 0xFFFFFFFF 或 0）
  if (timeout == 0xFFFFFFFF || timeout == 0) {
    timeout = DEFAULT_TIMEOUT;
    EEPROM.put(EEPROM_ADDR_TIMEOUT, timeout);
    Serial.println("EEPROM is empty");
  } else {
    Serial.print("read timeout: ");
    Serial.println(timeout);
  }

  timer.pause();
  timer.setPrescaleFactor(48); // 48分频  48000000/48=1000000
  timer.setOverflow(1000); // 1ms
  timer.attachInterrupt(&timer_it_callback);
  timer.resume();
}

void buffer_loop() {

  uint32_t lastToggleTime = millis(); // 记录上次切换的时间
  while (1) {

    if (millis() - lastToggleTime >= 500) // 每隔 500ms
    {
      lastToggleTime = millis(); // 记录当前时间
      digitalToggle(ERR_LED);
    }
    // 1、读取各传感器的值
    read_sensor_state();

#if DEBUG
    buffer_debug();
    while (Serial.available() > 0) {
      char c = Serial.read();
      serial_buf += c;
      int pos_enter = -1;
      pos_enter = serial_buf.indexOf("\n");
      if (pos_enter != -1) {
        String str = serial_buf.substring(0, pos_enter);
        serial_buf = serial_buf.substring(pos_enter + 1);
        if (strstr(str.c_str(), "gconf") != NULL) {
          TMC2208_n::CHOPCONF_t gconf{ 0 };

          // 提取 "gconf" 后面的十六进制字符串
          int pos = str.indexOf("gconf");
          if (pos != -1) {
            String hexPart = str.substring(pos + 5); // 跳过 "gconf"
            hexPart.trim(); // 去除前后空白符

            // 将字符串转换为 32 位无符号整数
            uint32_t hexValue = strtoul(hexPart.c_str(), NULL, 16);

            // 赋值给结构体（按你的结构定义赋值）
            gconf.sr = hexValue; // 假设 sr 是结构体中的原始寄存器值字段
          }
          driver.GCONF(gconf.sr);
          Serial.print("write GCONF:0x");
          Serial.println(gconf.sr, HEX);
          Serial.print("read GCONF: 0x");
          Serial.println(driver.GCONF(), HEX);
        }
      }
    }
#else
    motor_control();

    while (Serial.available() > 0) {
      char c = Serial.read();
      serial_buf += c;
    }
    if (serial_buf.length() > 0) {

      if (serial_buf == "rt") {
        Serial.print("read timeout=");
        Serial.println(timeout);
        serial_buf = "";
      } else if (serial_buf.startsWith("set")) {
        serial_buf.remove(0, 3);
        int64_t num = serial_buf.toInt();
        if (num < 0 || num > 0xffffffff) {
          serial_buf = "";
          Serial.println("Error: Invalid timeout value.");
          continue;
        }
        timeout = num;
        EEPROM.put(EEPROM_ADDR_TIMEOUT, timeout);
        serial_buf = "";
        Serial.print("set succeed! timeout=");
        Serial.println(timeout);
      } else {
        Serial.println(serial_buf.c_str());
        Serial.println("command error!");
        serial_buf = "";
      }
    }

#endif
  }
}

void buffer_sensor_init() {
  // 传感器初始化
  pinMode(HALL1, INPUT);
  pinMode(HALL2, INPUT);
  pinMode(HALL3, INPUT);
  pinMode(ENDSTOP_3, INPUT);
  pinMode(KEY1, INPUT);
  pinMode(KEY2, INPUT);

  // 耗材指示灯初始化
  pinMode(DUANLIAO, OUTPUT);
  pinMode(ERR_LED, OUTPUT);
  pinMode(START_LED, OUTPUT);
}

void buffer_motor_init() {
  // 电机驱动引脚初始化
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // Enable driver in hardware

  // 电机驱动初始化
  driver.begin(); // UART: Init SW UART (if selected) with default 115200 baudrate
  driver.beginSerial(9600);
  driver.I_scale_analog(false);
  driver.toff(5); // Enables driver in software
  driver.rms_current(I_CURRENT); // Set motor RMS current
  driver.microsteps(Move_Divide_NUM); // Set microsteps to 1/16th
  driver.VACTUAL(STOP); // Set velocity
  driver.en_spreadCycle(true);
  driver.pwm_autoscale(true);
}

/**
 * @brief  读取各传感器状态
 * @param  NULL
 * @retval NULL
 **/
void read_sensor_state(void) {
  buffer.buffer1_pos1_sensor_state = digitalRead(HALL3);
  buffer.buffer1_pos2_sensor_state = digitalRead(HALL2);
  buffer.buffer1_pos3_sensor_state = digitalRead(HALL1);
  buffer.buffer1_material_swtich_state = digitalRead(ENDSTOP_3);
  buffer.key1 = digitalRead(KEY1);
  buffer.key2 = digitalRead(KEY2);
}

/**
 * @brief  电机控制
 * @param  NULL
 * @retval NULL
 **/
void motor_control(void) {

  static Motor_State last_motor_state = Stop;

  // 按键控制电机
  // 按键1按下
  if (!digitalRead(KEY1)) {
    WRITE_EN_PIN(0); // 使能
    driver.VACTUAL(STOP); // 停止

    driver.shaft(BACK);
    driver.VACTUAL(VACTRUAL_VALUE);
    while (!digitalRead(KEY1))
      ; // 等待松手

    driver.VACTUAL(STOP); // 停止
    motor_state = Stop;

    is_front = false;
    front_time = 0;
    is_error = false;
    WRITE_EN_PIN(1); // 失能

  } else if (!digitalRead(KEY2)) // 按键2按下
  {
    WRITE_EN_PIN(0);
    driver.VACTUAL(STOP); // 停止

    driver.shaft(FORWARD);
    driver.VACTUAL(VACTRUAL_VALUE);
    while (!digitalRead(KEY2))
      ;

    driver.VACTUAL(STOP); // 停止
    motor_state = Stop;

    is_front = false;
    front_time = 0;
    is_error = false;
    WRITE_EN_PIN(1);
  }

  // 判断耗材
  if (digitalRead(ENDSTOP_3)) {
    // 无耗材，停止电机
    driver.VACTUAL(STOP); // 停止
    motor_state = Stop;

    // 断料引脚输出低电平
    digitalWrite(DUANLIAO, 0);

    // 关闭指示灯
    digitalWrite(START_LED, 0);

    is_front = false;
    front_time = 0;
    is_error = false;
    WRITE_EN_PIN(1);

    return; // 无耗材，结束
  }

  // 有耗材，断料引脚输出高电平
  digitalWrite(DUANLIAO, 1);

  // 开启指示灯
  digitalWrite(START_LED, 1);

  // 判断是否有错误
  if (is_error) {
    // 停止电机
    driver.VACTUAL(STOP); // 停止
    motor_state = Stop;
    WRITE_EN_PIN(1);
    return;
  }

  // 缓冲器位置记录
  if (buffer.buffer1_pos1_sensor_state) // 缓冲器位置为1，耗材往前推
  {
    last_motor_state = motor_state; // 记录上一次状态
    motor_state = Forward;
    is_front = true;

  } else if (buffer.buffer1_pos2_sensor_state) // 缓冲器位置为2,电机停止转动
  {
    last_motor_state = motor_state; // 记录上一次状态
    motor_state = Stop;
    is_front = false;
    front_time = 0;
  } else if (buffer.buffer1_pos3_sensor_state) // 缓冲器位置为3，回退耗材
  {
    last_motor_state = motor_state; // 记录上一次状态
    motor_state = Back;
    is_front = false;
    front_time = 0;
  }

  if (motor_state == last_motor_state) // 如果上次状态跟这次状态一致，则不需要再次发送控制命令,结束此次函数
    return;

  // 电机控制
  switch (motor_state) {
  case Forward: // 向前
  {
    WRITE_EN_PIN(0);
    if (last_motor_state == Back)
      driver.VACTUAL(STOP); // 上次是后退，先停下再前进
    driver.shaft(FORWARD);
    driver.VACTUAL(VACTRUAL_VALUE);

  } break;
  case Stop: // 停止
  {
    WRITE_EN_PIN(1);
    driver.VACTUAL(STOP);

  } break;
  case Back: // 向后
  {
    WRITE_EN_PIN(0);
    if (last_motor_state == Forward)
      driver.VACTUAL(STOP);
    ; // 上次是前进，先停下再后退
    driver.shaft(BACK);
    driver.VACTUAL(VACTRUAL_VALUE);
  } break;
  }
}

void timer_it_callback() {
  if (is_front) { // 如果往前推
    front_time++;
    if (front_time > timeout) { // 如果超时
      is_error = true;
    }
  }
}

void buffer_debug(void) {
  // Serial.print("buffer1_pos1_sensor_state:");Serial.println(buffer.buffer1_pos1_sensor_state);
  // Serial.print("buffer1_pos2_sensor_state:");Serial.println(buffer.buffer1_pos2_sensor_state);
  // Serial.print("buffer1_pos3_sensor_state:");Serial.println(buffer.buffer1_pos3_sensor_state);
  // Serial.print("buffer1_material_swtich_state:");Serial.println(buffer.buffer1_material_swtich_state);
  // Serial.print("key1:");Serial.println(buffer.key1);
  // Serial.print("key2:");Serial.println(buffer.key2);
  static int i = 0;
  if (i < 0x1ff) {
    Serial.print("i:");
    Serial.println(i);
    driver.GCONF(i);
    driver.PWMCONF(i);
    i++;
  }
  uint32_t gconf = driver.GCONF();
  uint32_t chopconf = driver.CHOPCONF();
  uint32_t pwmconf = driver.PWMCONF();
  if (driver.CRCerror) {
    Serial.println("CRCerror");
  } else {
    Serial.print("GCONF():0x");
    Serial.println(gconf, HEX);
    Serial.print("CHOPCONF():0x");
    char buf[11]; // "0x" + 8 digits + null terminator
    sprintf(buf, "%08lX", chopconf); // %08lX -> 8位大写十六进制（long unsigned）
    Serial.println(buf);
    Serial.print("PWMCONF():0x");
    sprintf(buf, "%08lX", pwmconf); // %08lX -> 8位大写十六进制（long unsigned）
    Serial.println(buf);
    Serial.println("");
  }
  delay(1000);
}