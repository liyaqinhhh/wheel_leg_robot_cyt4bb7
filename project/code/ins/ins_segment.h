/*
 * ins_segment.h
 *
 *  Created on: 2026年6月6日
 *      Author: LateRain
 *
 *  分段惯导 ---- 多段独立导航 + 用户自定义事件集
 *  ===============================================
 *
 *  设计背景:
 *    原 Ins.c 仅支持单条轨迹(最多30个航点)，所有航点共享同一套参数(速度、到达阈值)。
 *    无法满足"分段执行不同任务"的需求，例如:
 *      段0: 快速出库(高速, 大阈值) → 段1: 慢速接近目标(低速, 小阈值) → 段2: 返回入库(中速)
 *
 *  模块职责:
 *    - 管理最多 INS_SEG_MAX_SEGMENTS 个导航段，每段独立配置
 *    - 每段包含独立航点列表、目标速度、到达判定阈值、段结束行为
 *    - 支持用户注册自定义事件回调，替代硬编码的循环/跳转行为
 *    - 通过 Flash 页 49~57 持久化，断电不丢失
 *
 *  与旧模块的关系:
 *    - Ins.c/Ins.h 保持不动，ins_mode 新增值 2/3 转发到本模块
 *    - 复用 Ins.c 的 get_target() 计算距离/方位
 *    - 复用 Ins.c 的 cod_realtime 实时坐标
 *    - 输出 dis_ins / yaw_ins 供 Interrupt.c 的 turn_mode==7 PID 闭环使用
 *
 *  调用链路:
 *    main_cm7_0.c → ins_navigation() → case 2: ins_seg_edit_mode()
 *                                    → case 3: ins_seg_run_mode()
 *    Interrupt.c 16ms → get_realtime_coordinate() → cod_realtime 更新
 *    Interrupt.c 4ms  → turn_mode==7 → yaw_ins 作为 PID 目标角
 */

#ifndef _INS_SEGMENT_H_
#define _INS_SEGMENT_H_

#include "zf_common_typedef.h"

/* ==================================================================
 *  容量常量
 *  - 所有容量均可按需调整，但需同步修改 Flash 布局
 *  - 8段 × 15航点 = 最多 120 个航点，覆盖大多数场景
 *  - 内存开销: 8段 × ~252字节/段 ≈ 2KB，CYT4BB7 SRAM 可容纳
 * ================================================================== */

#define INS_SEG_MAX_SEGMENTS        8   /* 最大段数，对应 Flash 页 50~57 */
#define INS_SEG_MAX_WP_PER_SEG     15   /* 每段最大航点数 */

/* Flash 页分配 (每页 2048 字节 = 512 个 uint32):
 *   页 2, 12: 原 Ins.c 单轨数据 (不动)
 *   页 25:    IMU 零偏 (ins_interface)
 *   页 26~49: ins_track 轨迹数据 (24页)
 *   页 49:    本模块元数据 (magic + seg_count)
 *   页 50~57: 段0~7 完整数据 (航点 + 元数据)
 *   页 0,1:   已被 menu.c / ins_track 元数据占用
 */
#define INS_SEG_FLASH_PAGE_BASE    50   /* 段数据起始页(50~57 共8页) */
#define INS_SEG_FLASH_PAGE_META    49   /* 元数据页: [0]=magic, [1]=seg_count */

/*
 * INS_SEG_MAGIC = 0x5345474D = ASCII "SEGM"
 * Flash 出厂默认全 0xFF，擦除后也为 0xFF。
 * 写入数据后 Magic 位置不再是 0xFF。
 * 上电读取时先校验 Magic: 匹配 → 数据有效; 不匹配 → 从未写入或被破坏。
 * 与 ins_track.c 的 INS_TRACK_META_MAGIC(0x494E5354="INST") 同模式。
 */
#define INS_SEG_MAGIC              0x5345474Du

/* 新建段时的默认参数 */
#define INS_SEG_DEFAULT_SPEED       50     /* 默认目标速度 (电机 PWM 单位) */
#define INS_SEG_DEFAULT_THRESHOLD   20.0   /* 默认到达判定距离 (与旧 Ins.c 一致) */

/* ==================================================================
 *  段结束行为
 *
 *  当一个段的所有航点遍历完毕后，根据 arrival_behavior 决定下一步动作。
 *  其中 CUSTOM 行为查用户注册的事件回调表，实现任意自定义逻辑。
 * ================================================================== */

#define INS_SEG_BEHAVIOR_STOP       0   /* 停止: 停车, 回退 ins_mode=0 */
#define INS_SEG_BEHAVIOR_NEXT       1   /* 继续下一段: seg_current++，超界则同 STOP */
#define INS_SEG_BEHAVIOR_RETURN     2   /* 返回原点: 导航回 (0,0)，到达后 STOP */
#define INS_SEG_BEHAVIOR_CUSTOM     3   /* 自定义: 查事件表按 custom_event_id 调用回调 */

/* ==================================================================
 *  自定义事件集
 *
 *  替代硬编码的 LOOP/JUMP 行为。
 *  用户注册回调函数 → 段配置 custom_event_id → 段结束自动调用。
 *
 *  回调签名: uint8 callback(uint8 seg_idx)
 *     seg_idx:  当前段索引，回调可直接修改 ins_seg_current/ins_seg_wp_current
 *     返回值:   0 = 继续导航(回调已设置目标段/航点)
 *               1 = 全部完成(等同 STOP)
 *
 *  示例: 实现"巡逻循环"
 *    uint8 patrol_loop(uint8 seg_idx) {
 *        ins_seg_current = seg_idx;     // 重走当前段
 *        ins_seg_wp_current = 0;
 *        return 0;
 *    }
 *    ins_seg_event_register(1, patrol_loop);
 *    ins_seg_set_params(0, 50, 20, INS_SEG_BEHAVIOR_CUSTOM, 1);
 * ================================================================== */

#define INS_SEG_EVENT_MAX           8   /* 最多注册 8 个自定义事件 */

/* 事件回调: 参数=当前段索引, 返回 0=继续 1=完成 */
typedef uint8 (*InsSeg_EventCallback)(uint8 seg_idx);

/* 事件表条目: event_id 供段引用, callback 为处理函数 */
typedef struct {
    uint8  event_id;                 /* 事件ID, 与段的 custom_event_id 匹配 */
    InsSeg_EventCallback callback;   /* 回调函数指针, NULL 表示已删除 */
} InsSeg_EventEntry;

/* ==================================================================
 *  航点 / 段结构
 * ================================================================== */

/* 单个航点: 使用 double 保持与 Ins.c cod_realtime 一致, 避免类型转换精度损失 */
typedef struct {
    double x;                        /* X 坐标 (与 cod_realtime.x 同量纲) */
    double y;                        /* Y 坐标 (与 cod_realtime.y 同量纲) */
} InsSeg_Waypoint;

/*
 * 导航段: 包含航点列表和运行时参数
 *
 * 内存布局 (约 252 字节):
 *   waypoints[15]:  15 × 16字节(double×2) = 240 字节
 *   元数据字段:     ~12 字节
 *
 * Flash 存储布局 (每段一页，uint32 索引):
 *   [0  ..119]  航点数据 (15×4 个 uint32, 每航点: x_hi, x_lo, y_hi, y_lo)
 *   [120]       wp_count          (uint8)
 *   [121]       index             (uint8)
 *   [122]       target_speed      (float, 占1个uint32)
 *   [123]       arrival_threshold (float, 占1个uint32)
 *   [124]       arrival_behavior  (uint8)
 *   [125]       custom_event_id   (uint8)
 *   [126..127]  预留
 */
typedef struct {
    InsSeg_Waypoint waypoints[INS_SEG_MAX_WP_PER_SEG]; /* 航点数组, 按顺序执行 */
    uint8  wp_count;           /* 实际航点数 (1 ~ INS_SEG_MAX_WP_PER_SEG) */
    uint8  index;              /* 段自索引 (冗余校验, Flash 加载时验证) */
    float  target_speed;       /* 本段目标速度 (电机 PWM 单位, 写入 Yao.Target_Speed) */
    float  arrival_threshold;  /* 到达判定距离阈值 (dis_ins < 此值 → 到达航点) */
    uint8  arrival_behavior;   /* 段结束行为 (INS_SEG_BEHAVIOR_*) */
    uint8  custom_event_id;    /* 当 behavior==CUSTOM 时, 匹配事件表的 event_id */
} InsSeg_Segment;

/* ==================================================================
 *  全局变量 (extern 声明)
 *  定义在 ins_segment.c 中, 用户代码可直接读写以扩展功能
 * ================================================================== */

extern InsSeg_Segment  ins_seg_pool[INS_SEG_MAX_SEGMENTS]; /* 段池 (RAM 中常驻) */
extern uint8  ins_seg_count;     /* 已定义的段总数 (≤ INS_SEG_MAX_SEGMENTS) */
extern uint8  ins_seg_current;   /* 当前正在执行的段索引 (导航模式) */
extern uint8  ins_seg_wp_current;/* 当前段内的航点索引 (导航模式) */

/* ==================================================================
 *  事件集 API
 *  在进入导航模式前调用, 注册自定义段结束行为
 * ================================================================== */

/* 注册自定义事件回调。event_id 需与段的 custom_event_id 匹配。cb 为 NULL 则忽略。 */
void ins_seg_event_register(uint8 event_id, InsSeg_EventCallback cb);

/* 清空全部已注册事件 */
void ins_seg_event_clear(void);

/* 删除指定 event_id 的事件 (将其回调置 NULL) */
void ins_seg_event_clear_id(uint8 event_id);

/* ==================================================================
 *  段编辑 API
 *  在 ins_mode=2 (分段编辑模式) 中通过按键调用,
 *  也可在用户代码中直接调用以编程方式构建段
 * ================================================================== */

/* 选择当前编辑的段索引, 若该段为空则初始化默认参数 */
void ins_seg_select(uint8 idx);

/* 将当前 cod_realtime 坐标记录为当前段的航点, wp_count++ */
void ins_seg_record_waypoint(void);

/* 完成当前段编辑: s_edit_idx++ 并创建下一段(默认参数)。
 * 调用时机: 当前段所有航点录完后, 准备录下一段时调用。
 * 注意: 会更新 ins_seg_count 如果新索引超出之前记录的范围。 */
void ins_seg_finish_segment(void);

/* 设置指定段的运行时参数。
 * 参数 behavior = INS_SEG_BEHAVIOR_CUSTOM 时需同时传入 custom_event_id。
 * 参数 behavior ≠ CUSTOM 时 custom_event_id 被忽略。 */
void ins_seg_set_params(uint8 idx, float speed, float threshold,
                        uint8 behavior, uint8 custom_event_id);

/* ==================================================================
 *  持久化 API
 *  Flash 写入会先擦除整页再写入, 操作期间禁止中断
 * ================================================================== */

/* 保存所有段 (0 ~ ins_seg_count-1) 到 Flash, 含元数据。
 * 先依次写每段数据页, 最后写元数据页。 */
void ins_seg_save_to_flash(void);

/* 从 Flash 加载所有段到 RAM 的 ins_seg_pool。
 * 先读元数据(含 Magic 校验), 再依次读每段数据。
 * 返回 1=加载成功, 0=Magic 不匹配(从未写入或被破坏)。 */
uint8 ins_seg_load_from_flash(void);

/* 擦除所有段 Flash 数据 (页 49~57) 并清空 RAM 中的 ins_seg_pool。
 * 不可逆操作, 通常用于调试或重新录点前。 */
void ins_seg_erase_flash(void);

/* ==================================================================
 *  导航 API
 * ================================================================== */

/* 开始分段导航: 重置状态并从段0航点0开始, 自动进入 ins_mode=3。
 * 导航从 Flash 加载数据, 故需事先 ins_seg_save_to_flash()。 */
void ins_seg_start_navigation(void);

/* 紧急停止: 关电机、回退 turn_mode=2、ins_mode=0。
 * 同时由 seg_transition() 在正常完成(STOP/超界/回原点)时调用。 */
void ins_seg_stop_navigation(void);

/*
 * 状态机入口 (供 ins_navigation() 的 case 2 / case 3 转发):
 *   ins_seg_edit_mode() -- 在主循环中每帧调用, 处理按键录点/保存/切段
 *   ins_seg_run_mode()  -- 在主循环中每帧调用, 执行分段导航逻辑
 * 用户代码不应直接调用这两个函数。
 */
void ins_seg_edit_mode(void);
void ins_seg_run_mode(void);

/* ==================================================================
 *  状态查询
 * ================================================================== */

uint8 ins_seg_get_count(void);       /* 获取已定义段总数 */
uint8 ins_seg_get_current(void);     /* 获取当前执行段索引 (仅导航模式有效) */
uint8 ins_seg_get_wp_current(void);  /* 获取当前段内航点索引 (仅导航模式有效) */

#endif /* _INS_SEGMENT_H_ */
