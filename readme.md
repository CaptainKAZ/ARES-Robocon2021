# 项目结构

```
├─Application (FREERTOS任务)
│  ├─chassis_task
│  │      test_chassis_task.c (最简单的底盘任务 测试用)
│  │
│  ├─feedback_task (数据反馈任务 通过UART2发送)
│  │      feedback_task.c
│  │      feedback_task.h
│  │
│  ├─INS_task (惯性导航任务并且将传感器校准写入FALSH)
│  │      INS_task.c
│  │      INS_task.h
│  │
│  ├─led_monitor_task (LED灯语任务 负责开机动画和CPU占用率显示)
│  │      led_monitor_task.c
│  │      led_monitor_task.h
│  │
│  ├─motor_task (电机控制任务 1kHZ运行电机控制算法)
│  │      motor_task.c
│  │      motor_task.h
│  │
│  ├─quick_test_task (快速测试任务 快速测试装置)
│  │      quick_test_task.c
│  │      quick_test_task.h
│  │
│  └─user_init (启动调度器前的硬件初始化)
│          user_init.c
│          user_init.h
│
├─Driver (内部和外部的驱动程序)
│  ├─BSP (板级支持包 开发板内部硬件驱动)
│  │  │  user_it.c (所有编程的中断处理)
│  │  │
│  │  ├─can_comm (CAN通信中间层)
│  │  │      can_comm.c
│  │  │      can_comm.h
│  │  │
│  │  ├─flash (FLASH读写封装)
│  │  │      flash_io.c
│  │  │      flash_io.h
│  │  │
│  │  └─imu (IMU相关驱动)
│  │          imu.c (MPU6050与IST8310的读写配置与解析)
│  │          imu.h
│  │          imu_comm.c (MPU6050与IST8310的通信)
│  │          imu_comm.h
│  │          ist8310reg.h (IST8310寄存器表)
│  │          mpu6500reg.h (MPU6050寄存器表)
│  │
│  └─Device (外部设备驱动)
│      ├─encoder (目前只有CAN多圈编码器驱动)
│      │      encoder.c
│      │      encoder.h
│      │
│      ├─motor (电机驱动)
│      │      motor.c (电机大类驱动 提供统一API)
│      │      motor.h
│      │      rmd_motor.c (光毓机电电机驱动)
│      │      rmd_motor.h
│      │      rm_motor.c (RM电机驱动)
│      │      rm_motor.h
│      │      vesc_motor.c (VESC电机驱动)
│      │      vesc_motor.h
│      │
│      ├─ops (OPS-9定位系统驱动)
│      │      ops.c
│      │      ops.h
│      │
│      └─sbus (SBUS遥控器驱动)
│              sbus.c
│              sbus.h
│
└─Library (支持函数库)
    ├─algorithm (一般算法)
    │      quintic_trajactory.c (五次多项式生成)
    │      quintic_trajectory.h
    │      user_lib.c (各种数学函数 扒RM官方开源的)
    │      user_lib.h
    │
    ├─controller (控制器类)
    │      controller.c (控制器大类 统一API)
    │      controller.h
    │      FSF.c (全反馈(Full State Feedback)控制器)
    │      FSF.h
    │      PID.c (PID控制器 实现了增量式和位置式PID)
    │      PID.h
    │
    ├─cpu_utils (CPU占用率统计 从ST官方固件库扒来的)
    │      cpu_utils.c
    │      cpu_utils.h
    │
    └─stopwatch (微秒级计时库 通过TIM6可以为程序提供微秒级计时)
            stopwatch.c
            stopwatch.h
```
