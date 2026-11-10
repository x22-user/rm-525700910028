/**
 ******************************************************************************
 * @file    control.cpp/h
 * @brief   Robot control design. 机器人控制设计（模式/键位）
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/control.h"
#include "app/arm.h"

#include "iwdg.h"

// #include "app/imu_monitor.h"
#include "app/motor_monitor.h"
// #include "base/bsp/bsp_buzzer.h"
#include "base/bsp/bsp_led.h"
#include "base/common/math.h"
#include "base/remote/remote.h"
#include "base/remote/wfly.h"

void iwdgHandler(bool iwdg_refresh_flag);
void robotPowerStateFSM(bool stop_flag);
void robotReset(void);
bool robotStartup(void);
void robotControl(void);
void boardLedHandle(void);

#ifdef RF206S
extern WFLY rc;
#elif defined(DR16)
extern RC rc;
#endif
// extern IMU board_imu;

extern Arm arm;
BoardLed led;

// 上电状态
enum RobotPowerState_e {
  STOP = 0,
  STARTUP = 1,
  WORKING = 2,
} robot_state;
// 初始化标志
bool robot_init_flag = false;
// 遥控器挡位记录
#ifdef RF206S
WFLY::RCSwitch last_rc_switch;
#endif
#ifdef DR16
RC::RCSwitch last_rc_switch;
#endif

uint8_t arm_step = 0;  // 0 空闲，1 预备，2~5 依次走向 B、C、D、A
volatile bool arm_start_ok = false;  // 最近一次轨迹是否启动成功

// 控制初始化
void controlInit(void) {
  (RC_UART)->Init.BaudRate = 100000;
  (RC_UART)->Init.WordLength = UART_WORDLENGTH_9B;
#ifdef RF206S
  (RC_UART)->Init.StopBits = UART_STOPBITS_2;
#endif
#ifdef DR16
  (RC_UART)->Init.StopBits = UART_STOPBITS_1;
#endif
  (RC_UART)->Init.Parity = UART_PARITY_EVEN;
  (RC_UART)->Init.Mode = UART_MODE_RX;
  (RC_UART)->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  (RC_UART)->Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(RC_UART) != HAL_OK)
  {
      Error_Handler();
  }
  led.init();
  robot_state = STOP;
}

// 控制主循环
void controlLoop(void) {
  //PID调参
  //  HAL_IWDG_Refresh(&hiwdg);
  //  robot_state = WORKING;
  //  J0.mode_ = Motor::WORKING;
  //  J1.mode_ = Motor::WORKING;
  // //POWEROFF
  //  bool current_key_pressed = (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET);
  //  static bool last_key_pressed = false;
  //  static uint32_t last_key_tick = 0;
  //  if (current_key_pressed && !last_key_pressed) {
  //    if (HAL_GetTick() - last_key_tick > 200) {
  //      J0.targetAngle() += 10.0f;
  //      J1.targetAngle() += 10.0f;
  //      last_key_tick = HAL_GetTick();
  //    }
  //  }
  //  last_key_pressed = current_key_pressed;

  //按键控制画正方形
  // HAL_IWDG_Refresh(&hiwdg);
  // robot_state = WORKING;
  // static bool is_arm_inited = false;
  // if (arm.mode() == Arm::Mode::Disabled) {
  //   J0.mode_ = Motor::WORKING;
  //   J1.mode_ = Motor::WORKING;
  //   arm.calibrate(0, 0);
  //   arm.enable();
  // }
  // static bool last_key = false;
  // bool current_key = (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET);
  // if (current_key && !last_key) {
  //   if (arm_step == 0) arm_step = 1;
  // }
  // last_key = current_key;
  // switch (arm_step) {
  //   case 1: {
  //     const float prev[2] = {arm.reference().q[0], arm.reference().q[1]};
  //     CartesianReference a;
  //     a.x = 0.070f; a.y = 0.070f;
  //     float qA[2];
  //     if (ArmKinematics::inverse(a, prev, 1, qA)) {
  //       if (arm.trajStartJoint(qA[0], qA[1], 2.0f)) arm_step = 2;
  //     }
  //     break;
  //   }
  //   case 2:
  //     if (arm.trajFinished() && arm.trajStart(0.070f, -0.070f, 3.0f)) arm_step = 3;
  //     break;
  //   case 3:
  //     if (arm.trajFinished() && arm.trajStart(-0.070f, -0.070f, 3.0f)) arm_step = 4;
  //     break;
  //   case 4:
  //     if (arm.trajFinished() && arm.trajStart(-0.070f, 0.070f, 3.0f)) arm_step = 5;
  //     break;
  //   case 5:
  //     if (arm.trajFinished() && arm.trajStart(0.070f, 0.070f, 3.0f)) arm_step = 6;
  //     break;
  //   case 6:
  //     arm_step = 0;
  //     break;
  // }
  //
  // arm.handle();
  // boardLedHandle();

  // 正常运行
  iwdgHandler(rc.connect_.check());
  robotPowerStateFSM(!rc.connect_.check() || rc.switch_.r == decltype(rc)::DOWN);

  if (robot_state == STOP) {
    robotReset();
  } else if (robot_state == STARTUP) {
    robot_init_flag = robotStartup();  // 开机状态判断
  } else if (robot_state == WORKING) {
    robotControl();  // 机器人控制
  }
  arm.handle();
  boardLedHandle();
}

// IWDG处理，true持续刷新，false进入STOP状态并停止刷新
void iwdgHandler(bool iwdg_refresh_flag) {
  if (!iwdg_refresh_flag) {
    robot_state = STOP;
  } else {
    HAL_IWDG_Refresh(&hiwdg);
  }
}

// STOP(断电，安全模式)/开机/正常运行 状态机
void robotPowerStateFSM(bool stop_flag) {
  if (robot_state == STOP) {
    if (!stop_flag) {
      robot_state = STARTUP;
    }
  } else if (robot_state == STARTUP) {
    if (stop_flag) {
      robot_state = STOP;
    } else if (robot_init_flag) {
      // 初始化/复位完成
      robot_state = WORKING;
    }
  } else if (robot_state == WORKING) {
    if (stop_flag) {
      robot_state = STOP;
    }
  }
}

// 重置各功能状态
void robotReset(void) {
  robot_init_flag = false;
  arm_step = 0;
  arm_start_ok = false;
  arm.disable();
  last_rc_switch = rc.switch_;
}

// 开机上电启动处理
bool robotStartup(void) {
  return rc.connect_.check();
}

// 机器人控制
void robotControl(void) {
  const bool calib_mode =
      (rc.switch_.l == decltype(rc)::MID && rc.switch_.r == decltype(rc)::MID);
  const bool manual_mode =
      (rc.switch_.l == decltype(rc)::UP && rc.switch_.r == decltype(rc)::UP);
  const bool square_mode =
      (rc.switch_.l == decltype(rc)::MID && rc.switch_.r == decltype(rc)::UP);

  if (calib_mode) {
    if (arm.mode() != Arm::Mode::Disabled) arm.disable();
    arm.calibrate(0.f, 0.f);
    arm_step = 0;
    arm_start_ok = false;
  } else if (manual_mode) {
    if (arm.mode() != Arm::Mode::Joint && !arm.enable()) {
      last_rc_switch = rc.switch_;
      return;
    }
    // constexpr float kSpeedMax = 2.0f;
    // const float k = kSpeedMax / 660.f;
    // arm.addJointReference(rc.channel_.l_row * k, rc.channel_.l_col * k, 0.001f);
    arm_step = 0;
    arm_start_ok = false;
  } else if (square_mode) {
    if (arm.mode() != Arm::Mode::Joint && !arm.enable()) {
      last_rc_switch = rc.switch_;
      return;
    }
    const bool entering =
        !(last_rc_switch.l == decltype(rc)::MID &&
          last_rc_switch.r == decltype(rc)::UP);
    if (entering && arm_step == 0) {
      arm.trajAbort();
      arm_step = 1;
      arm_start_ok = false;
    }
    switch (arm_step) {
      case 0:
        if (entering) arm_step = 1;
        break;
      case 1: {
        // 移动到 A 点 (极限拉伸 99.5%)
        const float prev[2] = {arm.reference().q[0], arm.reference().q[1]};
        CartesianReference a;
        a.x = 0.143f;  // 👈 极限加长
        a.y = 0.143f;
        float qA[2];
        if (ArmKinematics::inverse(a, prev, 1, qA)) {
          // 移动到 A 点给 3 秒
          arm_start_ok = arm.trajStartJoint(qA[0], qA[1], 0.5f);
          if (arm_start_ok) arm_step = 2;
        }
        break;
      }
      case 2:
        if (arm.trajFinished() && arm.trajStart(0.143f, -0.143f, 0.3f))
          arm_step = 3;
        break;
      case 3:
        if (arm.trajFinished() && arm.trajStart(-0.143f, -0.143f, 0.3f))
          arm_step = 4;
        break;
      case 4:
        if (arm.trajFinished() && arm.trajStart(-0.143f, 0.143f, 0.3f))
          arm_step = 5;
        break;
      case 5:
        if (arm.trajFinished() && arm.trajStart(0.143f, 0.143f, 0.3f))
          arm_step = 6;
        break;
      case 6:
        arm_step = 0;
        break;
      default:
        arm_step = 0;
        break;
    }
  } else {
    if (arm.mode() != Arm::Mode::Joint) arm.enable();
    arm_step = 0;
    arm_start_ok = false;
  }
  last_rc_switch = rc.switch_;
}

// 板载LED指示灯效果
void boardLedHandle(void) {
#ifdef DBC
  if (robot_state == STOP) {
    led.setColor(255, 0, 0);  // red
    led.setModeOn();
  } else if (robot_state == STARTUP) {
    led.setColor(150, 150, 0);  // yellow
    led.setModeBreath();
  } else if (robot_state == WORKING) {
    led.setColor(0, 0, 255);  // blue
    if (rc.switch_.l == decltype(rc)::UP && rc.switch_.r == decltype(rc)::UP) {
      // 遥控器挡位左上右上
      led.setModeBlink(1);
    } else if (rc.switch_.l == decltype(rc)::MID && rc.switch_.r == decltype(rc)::UP) {
      // 遥控器挡位左中右上
      led.setModeBlink(2);
    } else if (rc.switch_.l == decltype(rc)::DOWN && rc.switch_.r == decltype(rc)::UP) {
      // 遥控器挡位左下右上
      led.setModeBlink(3);
    } else if (rc.switch_.l == decltype(rc)::UP && rc.switch_.r == decltype(rc)::MID) {
      // 遥控器挡位左上右中
      led.setModeBlink(4);
    } else if (rc.switch_.l == decltype(rc)::MID && rc.switch_.r == decltype(rc)::MID) {
      // 遥控器挡位左中右中
      led.setModeBlink(5);
    } else if (rc.switch_.l == decltype(rc)::DOWN && rc.switch_.r == decltype(rc)::MID) {
      // 遥控器挡位左下右中
      led.setModeBlink(6);
    }
  }
  led.handle();
#elif defined DBA
  if (robot_state == STOP) {
    led.setLED(true, false, 0);  // red
  } else if (robot_state == STARTUP) {
    led.setLED(true, true, 0);  // red+green
  } else if (robot_state == WORKING) {
    if (rc.switch_.l == decltype(rc)::UP && rc.switch_.r == decltype(rc)::UP) {
      // 遥控器挡位左上右上
      led.setLED(false, true, 1);  // green
    } else if (rc.switch_.l == decltype(rc)::MID && rc.switch_.r == decltype(rc)::UP) {
      // 遥控器挡位左中右上
      led.setLED(false, true, 2);  // green
    } else if (rc.switch_.l == decltype(rc)::DOWN && rc.switch_.r == decltype(rc)::UP) {
      // 遥控器挡位左下右上
      led.setLED(false, true, 3);  // green
    } else if (rc.switch_.l == decltype(rc)::UP && rc.switch_.r == decltype(rc)::MID) {
      // 遥控器挡位左上右中
      led.setLED(false, true, 4);  // green
    } else if (rc.switch_.l == decltype(rc)::MID && rc.switch_.r == decltype(rc)::MID) {
      // 遥控器挡位左中右中
      led.setLED(false, true, 5);  // green
    } else if (rc.switch_.l == decltype(rc)::DOWN && rc.switch_.r == decltype(rc)::MID) {
      // 遥控器挡位左下右中
      led.setLED(false, true, 6);  // green
    }
  }
#endif
}
