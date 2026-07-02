# C_Board_Framework_Old

  基于 STM32F407 的双板机器人控制框架，重点支持 **云台板**、**底盘板** 与 **板间 CAN 通信**。

  本工程通过同一套代码编译出两种固件：

  - `GIMBAL`：云台板固件
  - `CHASSIS`：底盘板固件

  ## 核心架构

  ```text
  遥控器 / 视觉 / IMU
          |
          v
      云台板 GIMBAL
          |
          | CAN 板间通信
          v
      底盘板 CHASSIS
          |
          v
    3508 底盘电机 / 麦轮底盘

  ## 云台板

  云台板主要负责上层控制和整车指令生成。

  相关文件：

  User/App/Gimbal_Task.c
  User/App/Robot.c
  User/Start/WHW_IRQN.c
  User/Start/All_Init.c

  主要功能：

  - 遥控器数据解析
  - 云台 yaw / pitch 双环 PID 控制
  - BMI088 IMU 姿态解算
  - DM4310 云台电机控制
  - 生成底盘控制指令
  - 通过 CAN 向底盘板发送运动目标

  云台板发送到底盘板的数据包括：

   字段            含义                                                                                                                                                                                  
  ━━━━━━━━━━━━━━  ━━━━━━━━━━━━━━━━━━
   vx              底盘 X 方向速度
  ──────────────  ──────────────────
   vy              底盘 Y 方向速度
  ──────────────  ──────────────────
   vw              底盘旋转速度
  ──────────────  ──────────────────
   yaw_relative    云台相对底盘角度
  ──────────────  ──────────────────
   mode            底盘模式
  ──────────────  ──────────────────
   cap_enable      电容使能

  ## 底盘板

  底盘板主要负责底盘运动控制。

  相关文件：

  User/App/Chassis_Task.c
  User/App/Robot.c
  User/Start/WHW_IRQN.c

  主要功能：

  - 接收云台板控制指令
  - 判断云台板是否在线
  - 根据 mode 切换底盘工作模式
  - 根据 yaw_relative 做底盘坐标变换
  - 麦轮运动解算
  - 4 个 DJI 3508 电机速度环 PID
  - 向云台板回传底盘状态

  底盘板反馈给云台板的数据包括：

   字段      含义                                                                                                                                                                                        
  ━━━━━━━━  ━━━━━━━━━━━━
   wheel1    1 号轮状态
  ────────  ────────────
   wheel2    2 号轮状态
  ────────  ────────────
   wheel3    3 号轮状态
  ────────  ────────────
   wheel4    4 号轮状态

  ## 底盘模式

  底盘模式由云台板下发的 mode 决定。

   mode    模式                                                                                                                                                                                          
  ━━━━━━  ━━━━━━━━━━━━━━
   1       小陀螺模式
  ──────  ──────────────
   2       底盘跟随云台
  ──────  ──────────────
   其他    普通控制模式

  当底盘板检测到云台板离线时，会将 vx / vy / vw 清零，避免失控。

  ## 板间通信

  板间通信模块位于：

  User/App/bord_transmit.c
  User/App/inc/bord_transmit.h

  主要接口：

  void BoardCom_Init(void);

  void BoardCom_SendGimbalCmd(CAN_HandleTypeDef *hcan,
                              int16_t vx,
                              int16_t vy,
                              int16_t vw,
                              int16_t yaw_relative,
                              uint8_t mode,
                              uint8_t cap_enable);

  void BoardCom_SendChassisFb(CAN_HandleTypeDef *hcan,
                              int16_t wheel1,
                              int16_t wheel2,
                              int16_t wheel3,
                              int16_t wheel4);

  void BoardCom_Decode(uint16_t std_id, const uint8_t data[8]);
  void BoardCom_OfflineTick(void);

  uint8_t BoardCom_IsGimbalOnline(void);
  uint8_t BoardCom_IsChassisOnline(void);

  ### 云台板到地盘板

  第一帧：运动控制指令

   字节    内容                                                                                                                                                                                          
  ━━━━━━  ━━━━━━━━━━━━━━
   0-1     vx
  ──────  ──────────────
   2-3     vy
  ──────  ──────────────
   4-5     vw
  ──────  ──────────────
   6-7     yaw_relative

  第二帧：模式与功能控制

   字节    内容                                                                                                                                                                                          
  ━━━━━━  ━━━━━━━━━━━━
   0       mode
  ──────  ────────────
   1       cap_enable
  ──────  ────────────
   2       seq

  ### 底盘板到云台板

   字节    内容                                                                                                                                                                                          
  ━━━━━━  ━━━━━━━━
   0-1     wheel1
  ──────  ────────
   2-3     wheel2
  ──────  ────────
   4-5     wheel3
  ──────  ────────
   6-7     wheel4

  ### 离线判断

  #define BOARD_COM_OFFLINE_TIME_MS 50U

  收到对应 CAN 帧后在线计数清零。

  周期任务中调用：

  BoardCom_OfflineTick();

  超过 50 ms 未收到数据时，认为对端离线。

  ## 板级角色配置

  板级角色配置位于：

  User/App/inc/board_config.h

  支持两种角色：

  #define ROBOT_BOARD_GIMBAL  1
  #define ROBOT_BOARD_CHASSIS 2

  代码中可使用：

  BOARD_IS_GIMBAL
  BOARD_IS_CHASSIS

  进行条件编译。

  ## 构建

  工程使用 CMake Presets。

   Preset             说明                                                                                                                                                                               
  ━━━━━━━━━━━━━━━━━  ━━━━━━━━━━━━━━━━
   Debug-Gimbal       Debug 云台板
  ─────────────────  ────────────────
   Debug-Chassis      Debug 底盘板
  ─────────────────  ────────────────
   Release-Gimbal     Release 云台板
  ─────────────────  ────────────────
   Release-Chassis    Release 底盘板

  构建云台板：

  cmake --preset Release-Gimbal
  cmake --build --preset Release-Gimbal

  构建底盘板：

  cmake --preset Release-Chassis
  cmake --build --preset Release-Chassis

  ## 输出文件

  云台板：

  build/Release-Gimbal/C_Board_Framework_Old_GIMBAL.elf
  build/Release-Gimbal/C_Board_Framework_Old_GIMBAL.hex
  build/Release-Gimbal/C_Board_Framework_Old_GIMBAL.bin

  底盘板：

  build/Release-Chassis/C_Board_Framework_Old_CHASSIS.elf
  build/Release-Chassis/C_Board_Framework_Old_CHASSIS.hex
  build/Release-Chassis/C_Board_Framework_Old_CHASSIS.bin

  ## 工程结构

  .
  ├── Core/                   # STM32CubeMX 生成代码
  ├── Drivers/                # STM32 HAL / CMSIS
  ├── Middlewares/            # FreeRTOS、USB、CMSIS-DSP、SEGGER RTT
  ├── USB_DEVICE/             # USB CDC
  ├── User/
  │   ├── App/                # 云台、底盘、整车任务、板间通信
  │   ├── Bsp/                # 电机、CAN、遥控器、IMU、裁判系统等 BSP
  │   ├── Algorithm/          # PID、滤波、姿态解算、数学工具
  │   ├── Start/              # 初始化、任务入口、中断回调
  │   └── UI/                 # 机器人 UI
  ├── cmake/                  # CMake 工具链配置
  ├── CMakeLists.txt
  └── CMakePresets.json

  ## 主要文件

   文件                           作用                                                                                                                                                                   
  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
   User/App/Gimbal_Task.c         云台控制
  ─────────────────────────────  ───────────────────────────────
   User/App/Chassis_Task.c        底盘控制
  ─────────────────────────────  ───────────────────────────────
   User/App/bord_transmit.c       板间通信
  ─────────────────────────────  ───────────────────────────────
   User/App/Robot.c               整车任务分发
  ─────────────────────────────  ───────────────────────────────
   User/Start/WHW_IRQN.c          FreeRTOS 任务与 CAN/UART 回调
  ─────────────────────────────  ───────────────────────────────
   User/Start/All_Init.c          外设初始化
  ─────────────────────────────  ───────────────────────────────
   User/App/inc/board_config.h    云台板/底盘板角色配置

  ## 开发说明

  - 修改云台逻辑：优先看 User/App/Gimbal_Task.c
  - 修改底盘逻辑：优先看 User/App/Chassis_Task.c
  - 修改板间协议：优先看 User/App/inc/bord_transmit.h
  - 修改任务周期：优先看 User/Start/WHW_IRQN.c
  - 修改硬件初始化：优先看 User/Start/All_Init.c
