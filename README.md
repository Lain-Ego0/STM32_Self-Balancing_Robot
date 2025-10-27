# STM32_Self-Balancing_Robot

本项目完成于2024年1月13日，不再更新。

This project was completed on January 13, 2024, and will no longer be updated.

## 一、项目概述
本项目基于STM32F103C8T6系列单片机开发自平衡车系统，集成**姿态检测（MPU6050）**、**电机速度反馈（编码器）**、**状态显示（OLED）**和**PID闭环控制**等模块，通过多传感器融合与控制算法实现两轮自平衡功能，适用于嵌入式控制与机器人技术学习。


## 二、硬件引脚说明
以下为系统关键硬件的STM32引脚分配，基于GPIO功能设计：

| 功能模块       | 引脚          | 功能说明       |
|----------------|---------------|----------------|
| 左轮编码器（AB相） | PA0、PA1      | 采集左轮转速脉冲 |
| 右轮编码器（AB相） | PA6、PA7      | 采集右轮转速脉冲 |
| 通用IO口1~4    | PA2、PA3、PA4、PA5 | 扩展输入/输出（可用于电机方向、开关等） |
| MPU6050（I2C） | PB10（SCL）、PB11（SDA） | 姿态传感器I2C通信 |
| MPU6050中断    | PA8           | 姿态数据中断同步 |
| OLED（I2C）    | PB8（SCL）、PB9（SDA） | 显示模块I2C通信 |
| 左轮PWM输出    | PB7           | 控制左轮电机转速 |
| 右轮PWM输出    | PB6           | 控制右轮电机转速 |


## 三、软件工程结构
工程基于Keil MDK开发，采用**模块化分层设计**，便于功能扩展与维护，文件结构如下：

### 1. 文件夹层级
- **User**：主程序模块，包含系统初始化与主循环逻辑。
  - `main.c/.h`：主函数与全局配置头文件。
  - `stm32f10x_conf.h/.it.c/.it.h`：STM32中断与外设配置文件。
- **Hardware**：硬件驱动模块，封装各外设底层驱动。
  - `OLED.c/.h/OLED_Font.h`：OLED显示驱动。
  - `ADC.c/.h`：ADC模块驱动（若需电压采集）。
  - `Encoder.c/.h`：编码器脉冲采集与速度计算。
  - `Exit.c/.h`：外部中断配置。
  - `PWM.c/.h`：定时器PWM输出配置。
  - `PID.c/.h`：PID控制算法实现。
  - `Motor.c/.h`：电机驱动逻辑。
- **MPU6050DMP**：姿态传感器驱动。
  - `MPU6050.c/.h`：MPU6050基础驱动。
  - `MPU6050_I2C.c/.h`：I2C通信底层封装。
  - `inv_mpu.c/.h`、`inv_mpu_dmp_motion_driver.c/.h`：InvenSense官方DMP运动处理库（用于姿态解算）。
  - `dmpKey.h`、`dmpmap.h`：DMP配置密钥与映射表。
- **system**：系统时钟与外设初始化配置。
- **Library**：STM32F10x标准外设库。
- **Start**：启动文件。


## 四、核心模块功能说明
### 1. 编码器模块（`Encoder.c/.h`）
- 功能：通过PA0、PA1（左轮）和PA6、PA7（右轮）的GPIO外部中断，采集电机转动的AB相脉冲，计算**电机转速**与**累计位移**。
- 应用：为PID速度环提供实时速度反馈，实现速度闭环控制。

### 2. MPU6050模块（`MPU6050DMP`文件夹）
- 功能：通过I2C接口（PB10、PB11）读取**加速度**、**角速度**数据，借助DMP库实现**姿态解算**（欧拉角/四元数），并通过PA8中断引脚实现数据同步。
- 应用：为PID角度环提供姿态反馈，是平衡控制的核心传感器。

### 3. OLED模块（`OLED.c/.h`）
- 功能：通过I2C接口（PB8、PB9）驱动OLED显示屏，显示车辆状态（如当前角度、电机转速、PID参数等）。
- 应用：调试与状态监控。

### 4. PWM模块（`PWM.c/.h`）
- 功能：配置定时器输出PWM波（PB6、PB7），通过**占空比**调节电机转速。
- 应用：执行PID控制输出，驱动电机运动。

### 5. PID控制模块（`PID.c/.h`）
- 功能：实现**角度环**（基于MPU6050角度）和**速度环**（基于编码器转速）的双闭环PID控制，计算电机控制量。
- 应用：通过反馈调节PWM输出，使车辆保持平衡并跟踪速度指令。

### 6. 电机驱动模块（`Motor.c/.h`）
- 功能：接收PID模块的控制量，输出PWM信号并配合IO口控制电机方向，驱动车轮运动。


## 五、系统工作流程
1. **系统初始化**：配置时钟、GPIO、I2C、定时器、中断等外设，初始化OLED、MPU6050、编码器、PID等模块。
2. **数据采集**：MPU6050周期性采集姿态数据，编码器实时采集电机转速。
3. **PID控制计算**：角度环根据当前姿态与目标角度（如竖直0°）计算速度指令，速度环根据速度指令与实际转速计算PWM占空比。
4. **电机驱动与显示**：输出PWM控制电机，同时通过OLED刷新车辆状态。
5. **循环执行**：重复步骤2~4，实现实时闭环控制。

# STM32_Self-Balancing_Robot

## 1. Project Overview

This project develops a self-balancing robot system based on the STM32F103C8T6 microcontroller. It integrates modules such as **attitude detection (MPU6050)**, **motor speed feedback (encoder)**, **status display (OLED)**, and **PID closed-loop control**. Through multi-sensor fusion and control algorithms, it achieves the self-balancing function of a two-wheeled robot, making it suitable for learning embedded control and robotics technology.


## 2. Hardware Pinout

The following is the STM32 pin assignment for key hardware modules, designed based on GPIO functions:

| Functional Module       | Pins                     | Description                                |
| ------------------------ | ------------------------ | ------------------------------------------ |
| Left Wheel Encoder (AB Phase) | PA0, PA1                 | Collect speed pulses of the left wheel     |
| Right Wheel Encoder (AB Phase) | PA6, PA7                 | Collect speed pulses of the right wheel    |
| General IO Ports 1~4    | PA2, PA3, PA4, PA5       | Extended input/output (can be used for motor direction, switches, etc.) |
| MPU6050 (I2C)           | PB10 (SCL), PB11 (SDA)   | I2C communication for the attitude sensor  |
| MPU6050 Interrupt        | PA8                      | Interrupt synchronization for attitude data|
| OLED (I2C)              | PB8 (SCL), PB9 (SDA)     | I2C communication for the display module   |
| Left Wheel PWM Output   | PB7                      | Control the speed of the left motor        |
| Right Wheel PWM Output  | PB6                      | Control the speed of the right motor       |


## 3. Software Architecture

The project is developed with Keil MDK and adopts a **modular hierarchical design** for easy function expansion and maintenance. The file structure is as follows:

### 1. Folder Hierarchy

- **User**: Main program module, including system initialization and main loop logic.
  - `main.c/.h`: Main function and global configuration header file.
  - `stm32f10x_conf.h/.it.c/.it.h`: STM32 interrupt and peripheral configuration files.
- **Hardware**: Hardware driver module, encapsulating the underlying drivers of each peripheral.
  - `OLED.c/.h/OLED_Font.h`: OLED display driver.
  - `ADC.c/.h`: ADC module driver (if voltage acquisition is needed).
  - `Encoder.c/.h`: Encoder pulse acquisition and speed calculation.
  - `Exit.c/.h`: External interrupt configuration.
  - `PWM.c/.h`: Timer PWM output configuration.
  - `PID.c/.h`: Implementation of the PID control algorithm.
  - `Motor.c/.h`: Motor drive logic.
- **MPU6050DMP**: Attitude sensor driver.
  - `MPU6050.c/.h`: Basic driver for MPU6050.
  - `MPU6050_I2C.c/.h`: Underlying encapsulation of I2C communication.
  - `inv_mpu.c/.h`, `inv_mpu_dmp_motion_driver.c/.h`: InvenSense official DMP motion processing library (for attitude calculation).
  - `dmpKey.h`, `dmpmap.h`: DMP configuration key and mapping table.
- **system**: System clock and peripheral initialization configuration.
- **Library**: STM32F10x standard peripheral library.
- **Start**: Startup files.


## 4. Core Modules Description

### 1. Encoder Module (`Encoder.c/.h`)

- Function: Collects AB-phase pulses of motor rotation through GPIO external interrupts of PA0, PA1 (left wheel) and PA6, PA7 (right wheel), and calculates **motor speed** and **cumulative displacement**.
- Application: Provides real-time speed feedback for the PID speed loop to achieve speed closed-loop control.

### 2. MPU6050 Module (`MPU6050DMP` Folder)

- Function: Reads **acceleration** and **angular velocity** data through the I2C interface (PB10, PB11), realizes **attitude calculation** (Euler angles/quaternions) with the help of the DMP library, and achieves data synchronization through the PA8 interrupt pin.
- Application: Provides attitude feedback for the PID angle loop and is the core sensor for balance control.

### 3. OLED Module (`OLED.c/.h`)

- Function: Drives the OLED display through the I2C interface (PB8, PB9) to display the vehicle status (such as current angle, motor speed, PID parameters, etc.).
- Application: Debugging and status monitoring.

### 4. PWM Module (`PWM.c/.h`)

- Function: Configures the timer to output PWM waves (PB6, PB7) and adjusts the motor speed through **duty cycle**.
- Application: Executes PID control output to drive motor movement.

### 5. PID Control Module (`PID.c/.h`)

- Function: Implements dual closed-loop PID control for the **angle loop** (based on MPU6050 angle) and **speed loop** (based on encoder speed) to calculate the motor control quantity.
- Application: Adjusts PWM output through feedback to keep the vehicle balanced and track speed commands.

### 6. Motor Drive Module (`Motor.c/.h`)

- Function: Receives the control quantity from the PID module, outputs PWM signals, and controls the motor direction with the help of IO ports to drive the wheels.


## 5. System Workflow

1. **System Initialization**: Configure peripherals such as clock, GPIO, I2C, timer, and interrupt, and initialize modules such as OLED, MPU6050, encoder, and PID.
2. **Data Acquisition**: MPU6050 periodically collects attitude data, and the encoder collects motor speed in real time.
3. **PID Control Calculation**: The angle loop calculates the speed command based on the current attitude and target angle (such as vertical 0°), and the speed loop calculates the PWM duty cycle based on the speed command and actual speed.
4. **Motor Drive and Display**: Output PWM to control the motor, and refresh the vehicle status through OLED at the same time.
5. **Loop Execution**: Repeat steps 2~4 to achieve real-time closed-loop control.
