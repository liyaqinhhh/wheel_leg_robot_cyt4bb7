/*
 * Ins.h
 *
 *  Created on: 2024年6月6日
 *      Author: LateRain
 *
 *  惯导模块 —— 单轨航点记录与循迹 + 分段惯导转发
 *  =====================================================
 *
 *  模块职责:
 *    1. 坐标系定义: Coordinates 结构体 (double 精度 XY 坐标)
 *    2. 实时坐标更新: get_realtime_coordinate() 航位推算
 *    3. 目标计算: get_target() 计算距离 dis_ins 和方位角 yaw_ins
 *    4. 单轨惯导: ins_navigation() 状态机 (ins_mode=0 录点, ins_mode=1 循迹)
 *    5. 分段惯导转发: ins_mode=2/3 → ins_segment.c
 *
 *  调用链路:
 *    Interrupt.c 16ms中断 → get_realtime_coordinate(speed, 0.016, yaw)
 *    main_cm7_0.c 主循环   → ins_navigation() [每帧]
 *    Interrupt.c 4ms中断  → turn_mode==7 → 读取 yaw_ins 做 PID 闭环转向
 *
 *  全局变量:
 *    cod_realtime      实时坐标 (由 get_realtime_coordinate 累加更新)
 *    cod_saved[30]     录制暂存 (ins_mode=0 按键录点)
 *    cod_target[30]    导航目标 (ins_mode=1 Flash 加载后的航点)
 *    dis_ins           到目标点的距离 (由 get_target 计算)
 *    yaw_ins           到目标点的方位角 0~360° (由 get_target 计算)
 *    ins_mode          惯导模式: 0=录点 1=循迹 2=分段编辑 3=分段导航
 *    n                 航点数量 (录制/加载)
 *    target            当前目标航点索引 (ins_mode=1)
 *    flag_save         Flash 保存状态标记
 */

#ifndef CODE_CENTER_INS_H_
#define CODE_CENTER_INS_H_

/* 分段惯导模块: ins_mode=2/3 的入口声明 */

/*
 * 角度/弧度互转宏
 * 使用 double 精度 PI, 避免 float 截断误差在航位推算中累积
 */
#define ANGLE_TO_RAD(x) ((x) * PI / 180.0) /* 度 → 弧度 */
#define RAD_TO_ANGLE(x) ((x) * 180.0 / PI) /* 弧度 → 度 */
#define PI (3.1415926535898)               /* double 精度圆周率 */

/* 轮周长(cm): 轮半径 3.5cm × 2π, 用于转速→线速度换算 */
#define WHEEL_CIRCUMFERENCE_CM (2.0f * 3.14159265f * 3.1f / 60.0f)

/*
 * 坐标结构体
 * 使用 double (64位 IEEE 754) 而非 float (32位):
 *   - CYT4BB7 Cortex-M7 有硬件 FPU, double 运算开销可接受
 *   - 航位推算需长时间累加, float 的 7 位有效数字不够,
 *     例如跑 1000 米后 float 精度降至 ±0.1m,
 *     double 的 15 位有效数字可保持 ±0.0001m 精度
 */
typedef struct
{
    double x; /* X 坐标 (与速度×时间×cos(偏航) 同量纲) */
    double y; /* Y 坐标 (与速度×时间×sin(偏航) 同量纲) */
} Coordinates;

/*
 * 实时坐标更新 (每 16ms 由中断调用)
 *   speed: 瞬时速度 (电机编码器单位)
 *   time:  时间间隔 (秒, 当前为 0.016 = 16ms)
 *   yaw:   当前偏航角 (度, 0~359°, 来自 imu660ra.eulerAngle.yaw)
 *
 * 公式: dx = speed × time × cos(yaw_rad)
 *       dy = speed × time × sin(yaw_rad)
 * 累加到 cod_realtime.x / cod_realtime.y
 *
 * 副作用: 写 IPS200 屏幕 (行 96/112) 显示 X/Y 坐标
 */
void get_realtime_coordinate(int speed, float time, float yaw);

/*
 * 计算从 (x1,y1) 到 (x2,y2) 的距离和方位角
 * 输出设置全局变量:
 *   dis_ins = sqrt((x2-x1)^2 + (y2-y1)^2)   距离
 *   yaw_ins = atan2(dy, dx) → 角度 [-180°, 180°]  方位角
 *
 * 此函数无副作用 (除设置 dis_ins/yaw_ins 外),
 * 可被分段惯导模块 ins_segment.c 复用。
 */
void get_target(double x1, double y1, double x2, double y2);

/*
 * 惯导主状态机 (在主循环中每帧调用)
 *
 * ins_mode=0 (INS_MODE_RECORD):
 *   单轨录点模式。按键操作:
 *     KEY_3 短按 → 记录 cod_realtime 到 cod_saved[n], n++
 *     KEY_2 短按 → 保存航点到 Flash (页2=坐标, 页12=n)
 *     KEY_1 短按 → 进入导航模式 ins_mode=1
 *
 * ins_mode=1 (INS_MODE_NAV_SINGLE):
 *   单轨循迹模式。首次进入从 Flash 加载航点到 cod_target[]。
 *   每帧调用 get_target() 计算 dis_ins/yaw_ins。
 *   到达判定: dis_ins < 20 且 dis_ins != 0
 *   到达后 target++ 指向下一航点。
 *   target == 0 表示回到起点(库位) → 停车退出。
 *   target == n 时回绕到 target = 0 (循环至原点)。
 *
 * ins_mode=2 (INS_MODE_SEG_EDIT):
 *   分段编辑模式 → 转发到 ins_seg_edit_mode() (ins_segment.c)
 *
 * ins_mode=3 (INS_MODE_SEG_RUN):
 *   分段导航模式 → 转发到 ins_seg_run_mode() (ins_segment.c)
 */
void ins_navigation(void);

/* ---- 全局变量 extern ---- */

#define TURN_SPEED 500

extern Coordinates cod_realtime;   /* 实时坐标: 由 get_realtime_coordinate 不断累加 */
extern Coordinates cod_saved[30];  /* 录制暂存: ins_mode=0 按键录点时暂存, 最多30个 */
extern Coordinates cod_target[30]; /* 导航目标: ins_mode=1 从 Flash 加载的航点 */
extern double dis_ins;             /* 到目标点的距离 */
extern double yaw_ins;             /* 到目标点的方位角 0~360° */
extern volatile uint8 ins_mode;    /* 惯导模式: 0/1/2/3 */
extern uint8 n;                    /* 航点数量 */
extern uint8 target;               /* 当前目标航点索引 */
extern uint8 flag_save;            /* Flash 保存状态: 0=未保存 1=已保存 */
extern bool flag_1;
extern bool flag_2; /* 首次进入 ins_mode=1 的标志: true=需从Flash加载 */

#endif /* CODE_CENTER_INS_H_ */
