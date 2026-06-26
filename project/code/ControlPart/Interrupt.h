/**
 * @file    Interrupt.h
 * @brief   轮腿机器人中断服务模块头文件
 *
 * 定义多级定时中断服务函数（1ms/2ms/4ms/8ms/16ms/40ms），
 * 每级中断负责不同频率的控制任务（IMU更新、PID计算、舵机控制等）。
 *
 * 中断调度层级：
 *   1ms  → IMU偏航角累积、圈数跟踪、单腿计时
 *   2ms  → 跳跃控制、按键扫描、AI调参、陀螺仪数据读取、角速度环PID（内环）
 *   4ms  → 卡尔曼滤波、角度环PID（外环）、转向PID模式切换
 *   8ms  → 单腿高度控制、舵机平衡控制
 *   16ms → 速度环PID（最外层）、惯导坐标更新
 *   40ms → 斑马线检测超时、偏航角慢漂补偿
 */

#ifndef CODE_CONTROLPART_INTERRUPT_H_
#define CODE_CONTROLPART_INTERRUPT_H_

/* 单腿站立计时器（在1ms中断中递增） */
extern uint16 a11111;

/**
 * 中心控制结构体
 *
 * 存储各级中断计算出的PID输出值和传感器数据，
 * 供运动控制模块（Adapt_Terrain、舵机输出等）读取使用。
 */
typedef struct
{
    float Outp_Gyro_Pitch;  /* 俯仰角速度环PID输出 */
    float Outp_Angle_Pitch; /* 俯仰角度环PID输出 */
    float Outp_Speed_Pitch; /* 俯仰速度环PID输出 */

    float Outp_Gyro_Roll;  /* 翻滚角速度环PID输出 */
    float Outp_Angle_Roll; /* 翻滚角度环PID输出 */

    float Outp_Gyro_Yaw;  /* 偏航角速度环PID输出 */
    float Outp_Angle_Yaw; /* 偏航角度环PID输出 */

    float Outp_turn; /* 转向PID最终输出（驱动电机差速） */

    int16 Encoder_Left;  /* 左轮编码器速度 */
    int16 Encoder_Right; /* 右轮编码器速度 */

    int Target_Speed;    /* 目标速度（设定值） */
    float Target_height; /* 目标高度（设定值，单位cm） */
} Center_struct;

extern Center_struct Yao; /* 全局中心控制结构体实例 */

/* ---- AI调参 ---- */
extern uint16_t flag_ai_open; /* AI调参模式开关 */

/* ---- 系统标志 ---- */
extern int16_t flag_main;  /* 主状态标志 */
extern uint16_t flag_text; /* 文本显示标志 */
extern uint8 flag_count;

/* ---- 转向控制 ---- */
extern uint8 steer_control_mode; /* 转向控制模式：0=角度控制(逐飞方案), 1=PWM控制(无速度环) */
extern uint8 turn_mode;          /* 转向模式：
                                  *   0=关闭, 1=逐飞PID转向,
                                  *   2=串级转向, 3=偏航角度闭环走直线,
                                  *   4=视觉转向, 5=GPS转向,
                                  *   6=原地旋转(Spin3), 7=惯导转向 */
extern uint8 fuzzy_mode;         /* 模糊控制模式：0=关闭(使用固定KP), 1=开启(动态KP范围) */
extern float kp_roll;            /* 翻滚KP系数 */
extern float k11;                /* 急加速补偿系数1 */
extern float k22;                /* 急加速补偿系数2 */
extern float Target_Yaw;         /* 目标偏航角（度） */

/* ---- 传感器 ---- */
extern float Battery_voltage;  /* 电池电压（V） */
extern uint16 dis_tof_mm;      /* TOF距离传感器读数（mm） */
extern volatile float angle_Z; /* 连续累计偏航角（度，可超过±180，用于多圈旋转） */

/* ---- 惯导 ---- */
extern uint8 ins_open; /* 惯导系统开关：0=关闭, 1=开启 */

/* ---- 菜单 ---- */
extern uint8 menu_open; /* 菜单模式：0=关闭, 1=打开菜单和Flash, 2=只读取不打开菜单 */
extern uint8 ins_getdata;
/* ---- 标志位 ---- */
extern uint8 flag_stop;             /* 停止标志：1=停止, 0=运行 */
extern uint8 flag_yawan;            /* 偏航辅助使能：0=关闭, 1=开启 */
extern uint8 telemetry_enable;      /* 遥测使能：0=关闭, 1=开启（40ms中断通过无线串口发送调试数据） */
extern uint8 ins_telemetry_enable;  /* 惯导遥测使能：0=关闭, 1=开启 */
extern uint8 jump_telemetry_enable; /* 跳跃遥测使能：0=关闭, 1=开启（发送 $J 帧） */
extern uint8 angle_wireless;        /* 姿态角遥测使能：0=关闭, 1=开启（发送 A 帧） */
extern uint8 kalman_wireless;       /* 卡尔曼诊断遥测使能：0=关闭, 1=开启（发送 $K 帧） */

/* ---- 定时器计数 ---- */
extern uint16 TCount_4ms;     /* 4ms中断计数器（受TCount_falg_4ms控制） */
extern uint16 TCount_40ms;    /* 40ms中断计数器（用于斑马线超时检测） */
extern uint8 TCount_falg_4ms; /* 4ms计数使能标志 */

/* ---- GPS转向 ---- */
extern volatile float steer_gps_target_bearing_deg;    /* GPS目标方位角（度, [0, 360)） */
extern volatile float steer_gps_distance_to_wp_m;      /* GPS到目标点距离（米） */
extern volatile float steer_gps_to_imu_yaw_offset_deg; /* IMU偏航角与地理航向的偏移量 */

/* ---- 视觉转向 ---- */
extern float desired_yaw; /* 期望偏航角（度） */

/* ---- 开机角度校准 ---- */
extern volatile uint8_t calibrate_state;  /* 校准状态：0=未开始, 1=采集中, 2=完成 */
extern volatile float calibrate_offset;   /* 校准角度偏移量（度） */
extern volatile uint16_t calibrate_count; /* 校准采样计数 */
extern volatile float calibrate_sum;      /* 校准角度累加和 */
extern float Target_Speed;                /* 目标速度 */
#define CALIBRATE_SAMPLES 500             /* 校准采样数（500个样本@2ms=1秒） */

/* 中断任务调度（标志位由 ISR 累加，主函数消费执行） */
void Interrupt_Flag_Increment(void); /* ISR 中每 1ms 调用，累加各时间级标志位 */
void Run_Interrupt_Tasks(void);      /* 主函数轮询调用，消费标志位并执行对应任务 */

/* ---- 中断服务函数 ---- */
void Interrupt_1ms(void);
void Interrupt_2ms(void);
void Interrupt_4ms(void);
void Interrupt_8ms(void);
void Interrupt_16ms(void);
void Interrupt_40ms(void);

/* ---- 内部API函数（按中断时间分组） ---- */

/* 1ms中断API */
// void Update_TOF_Sensor(void);       /* 更新TOF距离传感器读数 */
void Update_Yaw_Integration(void);  /* 更新偏航角积分 */
void Update_Angle_Z(void);          /* 更新偏航角归一化和跨边界圈数检测 */
void Update_Single_Leg_Timer(void); /* 更新单腿站立计时器 */

/* 2ms中断API */
void jump_text(void); /* 跳跃控制文本显示 */
// void Update_Jump_Control(void);  /* 更新跳跃控制 */
void Update_Gyro_PID_Loop(void); /* 更新角速度环PID（内环） */

/* 4ms中断API */
void Update_Kalman_Filter(void);    /* 更新卡尔曼滤波姿态 */
void Update_4ms_Counter(void);      /* 更新4ms计数器 */
void Update_Angle_PID_Loop(void);   /* 更新俯仰角度环PID（外环） */
void Update_Steering_Control(void); /* 更新转向控制 */

/* 16ms中断API */
void Update_INS_Coordinate(void); /* 更新惯导坐标 */
void Update_Speed_PID_Loop(void); /* 更新速度环PID（最外环） */

/* 40ms中断API */
void Update_Zebra_Timeout(void); /* 更新斑马线超时检测 */
// void Update_Yaw_Drift_Compensation(void); /* 更新偏航角慢漂补偿 */
void Update_Telemetry_Send(void); /* 更新遥测数据发送 */

/* 主循环API */
void control_main(void); /* 电机控制输出（主循环调用） */

#endif /* CODE_CONTROLPART_INTERRUPT_H_ */
