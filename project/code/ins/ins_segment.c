/*
 * ins_segment.c
 *
 *  Created on: 2026年6月6日
 *      Author: LateRain
 *
 *  分段惯导 —— 完整实现
 *  ======================
 *
 *  文件结构:
 *    1. 内部全局变量 — RAM 中的段池、编辑/导航状态、事件表
 *    2. 事件集 API — 用户注册自定义段结束行为
 *    3. Flash I/O — 段数据与元数据的持久化/加载
 *    4. 导航核心 — 目标计算、航点推进、段间转移
 *    5. 状态机入口 — ins_mode=2 编辑模式 / ins_mode=3 导航模式
 *    6. 公开 API — 段编辑、持久化、导航控制、状态查询
 *
 *  关键状态变量:
 *    s_edit_idx      — 当前正在编辑的段索引 (编辑模式)
 *    s_run_init      — 导航模式是否已初始化 (首次进入时加载 Flash)
 *    s_is_returning  — 是否处于"返回原点"状态 (影响 seg_calc_target)
 *
 *  s_is_returning 的设计原因:
 *    seg_transition() 设置 RETURN 行为后, 下一帧的 seg_calc_target() 会再次
 *    调用 get_target()。如果不加标记, seg_calc_target() 会以段内航点为目标,
 *    覆盖 RETURN 设置的 (0,0) 目标。因此用 s_is_returning 标记告诉
 *    seg_calc_target() 当前应以原点(0,0)作为目标计算 dis_ins/yaw_ins。
 */

#include "zf_common_headfile.h"
#include "Ins.h"
#include "ins_segment.h"
#include "Interrupt.h"
#include "small_driver_uart_control.h"

/* ==================================================================
 *  1. 内部全局变量
 * ================================================================== */

/* 段池: 所有段在 RAM 中常驻。上电后从 Flash 加载或通过 API/按键构建。
 * 总大小: 8段 × 252字节 ≈ 2KB, CYT4BB7 SRAM 可容纳。 */
InsSeg_Segment  ins_seg_pool[INS_SEG_MAX_SEGMENTS] = {0};

/* 已定义的段总数。取值范围 [0, INS_SEG_MAX_SEGMENTS]。
 * 导航时只遍历 0 ~ ins_seg_count-1。 */
uint8  ins_seg_count = 0;

/* 当前执行段索引 (导航模式), 当前段内航点索引 (导航模式) */
uint8  ins_seg_current = 0;
uint8  ins_seg_wp_current = 0;

/* ---- 内部状态 (static, 模块外不可见) ---- */

/* 编辑模式: 当前正在编辑的段索引 */
static uint8  s_edit_idx = 0;

/* 导航模式: 首次进入标志。0=需要初始化(加载Flash+设置turn_mode), 1=已初始化 */
static uint8  s_run_init = 0;

/* 预留: 循环迭代计数, 供未来 LOOP 行为或用户自定义回调使用 */
static uint8  s_loop_iter = 0;

/* 返回原点标记: 0=正常导航, 1=正在返回原点(0,0)。
 * 由 seg_transition() 的 RETURN 分支设置,
 * 由 seg_calc_target() 检查以决定计算目标,
 * 由 ins_seg_run_mode() 在到达原点后清除。 */
static uint8  s_is_returning = 0;

/* ==================================================================
 *  2. 事件集
 *
 *  事件表存储在 RAM 中 (不上 Flash), 断电丢失。
 *  用户需在每次上电初始化时重新注册事件回调。
 *
 *  查找复杂度: O(n), n ≤ INS_SEG_EVENT_MAX(8), 对主循环无影响。
 * ================================================================== */

static InsSeg_EventEntry s_event_table[INS_SEG_EVENT_MAX] = {0};
static uint8 s_event_count = 0;  /* 已注册事件数 */

/* Flash 页内偏移常量:
 *   每航点占 4 个 uint32 (x_hi, x_lo, y_hi, y_lo)
 *   15 航点 × 4 = 120 个 uint32 = WP_U32_COUNT
 *   元数据紧随其后, 起始偏移 = META_OFFSET */
#define WP_U32_COUNT  (INS_SEG_MAX_WP_PER_SEG * 4)   /* = 120 */
#define META_OFFSET   WP_U32_COUNT                    /* = 120 */
/* 元数据占 6 个 uint32: wp_count(1) + index(1) + target_speed(1)
 *                       + arrival_threshold(1) + arrival_behavior(1)
 *                       + custom_event_id(1) = 6
 * 远小于 Flash 页剩余容量 (512 - 120 = 392), 有大量预留空间。 */

/* ==================================================================
 *  3. 事件集 API
 * ================================================================== */

/*
 * 注册自定义事件回调。
 * 将 (event_id, cb) 追加到事件表末尾。
 * 若表已满或 cb==NULL 则静默忽略(不报错, 嵌入式环境无日志)。
 *
 * 注意: 同一 event_id 重复注册不会覆盖旧条目,
 *       查找时返回第一个匹配的条目。
 *       若需覆盖, 先调用 ins_seg_event_clear_id() 删除旧条目。
 */
void ins_seg_event_register(uint8 event_id, InsSeg_EventCallback cb)
{
    if (s_event_count < INS_SEG_EVENT_MAX && cb != NULL)
    {
        s_event_table[s_event_count].event_id = event_id;
        s_event_table[s_event_count].callback = cb;
        s_event_count++;
    }
}

/* 清空全部事件表 (通常在系统复位时调用) */
void ins_seg_event_clear(void)
{
    memset(s_event_table, 0, sizeof(s_event_table));
    s_event_count = 0;
}

/* 按 event_id 删除事件 (标记 callback 为 NULL, event_id 清零) */
void ins_seg_event_clear_id(uint8 event_id)
{
    for (uint8 i = 0; i < s_event_count; i++)
    {
        if (s_event_table[i].event_id == event_id)
        {
            s_event_table[i].callback = NULL;
            s_event_table[i].event_id = 0;
        }
    }
}

/*
 * 按 event_id 查找回调。
 * 返回 NULL 表示未找到 (event_id 不存在或回调已被删除)。
 * O(n) 线性查找, n ≤ 8, 性能无影响。
 */
static InsSeg_EventCallback event_find(uint8 event_id)
{
    for (uint8 i = 0; i < s_event_count; i++)
    {
        if (s_event_table[i].event_id == event_id && s_event_table[i].callback != NULL)
            return s_event_table[i].callback;
    }
    return NULL;
}

/* ==================================================================
 *  4. Flash I/O
 *
 *  Flash 操作约束:
 *    - 写入前必须先擦除整页 (flash_erase_page), 擦除后全页为 0xFF
 *    - 写入单位: flash_union_buffer[0..FLASH_PAGE_LENGTH-1]
 *    - 写入函数: flash_write_page_from_buffer(sector, page, len)
 *    - 读取函数: flash_read_page_to_buffer(sector, page, len)
 *    - 操作期间禁止中断, 不能被打断
 *
 *  Double 存储方案 (复用 Ins.c 的 writeDoubleToFlash1 模式):
 *    double 是 64 位 IEEE 754, 拆分为两个 uint32:
 *      hi = (uint32_t)(*(uint64_t*)&d >> 32)
 *      lo = (uint32_t)(*(uint64_t*)&d & 0xFFFFFFFF)
 *    读取时反向拼接:
 *      combined = ((uint64_t)hi << 32) | lo
 *      d = *(double*)&combined
 *    该模式在 CYT4BB7 (ARM Cortex-M7, little-endian) 上验证通过。
 * ================================================================== */

/*
 * 保存单个段到其对应 Flash 页。
 * 操作: 清 buffer → 写航点(15×4 uint32) → 写元数据(6 uint32) → 擦除页 → 写入
 */
static void flash_save_segment(uint8 idx)
{
    InsSeg_Segment *seg = &ins_seg_pool[idx];
    uint32 page = INS_SEG_FLASH_PAGE_BASE + idx;  /* 段 idx → 页 50+idx */

    flash_buffer_clear();

    /* 1. 写入全部 15 个航点 (含未使用的, 写为 0.0 即全零) */
    for (uint8 w = 0; w < INS_SEG_MAX_WP_PER_SEG; w++)
    {
        uint32 base = w * 4;  /* 每航点起点在 buffer 中的 uint32 索引 */
        uint64_t *px = (uint64_t *)&seg->waypoints[w].x;
        uint64_t *py = (uint64_t *)&seg->waypoints[w].y;
        flash_union_buffer[base + 0].uint32_type = (uint32_t)(*px >> 32);
        flash_union_buffer[base + 1].uint32_type = (uint32_t)(*px & 0xFFFFFFFF);
        flash_union_buffer[base + 2].uint32_type = (uint32_t)(*py >> 32);
        flash_union_buffer[base + 3].uint32_type = (uint32_t)(*py & 0xFFFFFFFF);
    }

    /* 2. 写入段元数据 (6 个 uint32) */
    flash_union_buffer[META_OFFSET + 0].uint8_type = seg->wp_count;
    flash_union_buffer[META_OFFSET + 1].uint8_type = seg->index;
    flash_union_buffer[META_OFFSET + 2].float_type = seg->target_speed;
    flash_union_buffer[META_OFFSET + 3].float_type = seg->arrival_threshold;
    flash_union_buffer[META_OFFSET + 4].uint8_type = seg->arrival_behavior;
    flash_union_buffer[META_OFFSET + 5].uint8_type = seg->custom_event_id;

    /* 3. 先擦除再写入 (Flash 硬件要求) */
    flash_erase_page(0, page);
    flash_write_page_from_buffer(0, page, FLASH_PAGE_LENGTH);
    flash_buffer_clear();
}

/*
 * 从 Flash 页加载单个段到 RAM。
 * 操作: 读页到 buffer → 解析航点 → 解析元数据 → 有效性校验
 *
 * 有效性校验: wp_count 超出范围或 index 超出范围 → 整段清零。
 * 这是为了防止 Flash 位翻转或写入中断导致的数据损坏扩散。
 */
static void flash_load_segment(uint8 idx)
{
    InsSeg_Segment *seg = &ins_seg_pool[idx];
    uint32 page = INS_SEG_FLASH_PAGE_BASE + idx;

    flash_buffer_clear();
    flash_read_page_to_buffer(0, page, FLASH_PAGE_LENGTH);

    /* 1. 读取全部 15 个航点 */
    for (uint8 w = 0; w < INS_SEG_MAX_WP_PER_SEG; w++)
    {
        uint32 base = w * 4;
        uint64_t cx = ((uint64_t)flash_union_buffer[base + 0].uint32_type << 32)
                    |  flash_union_buffer[base + 1].uint32_type;
        uint64_t cy = ((uint64_t)flash_union_buffer[base + 2].uint32_type << 32)
                    |  flash_union_buffer[base + 3].uint32_type;
        seg->waypoints[w].x = *(double *)&cx;
        seg->waypoints[w].y = *(double *)&cy;
    }

    /* 2. 读取段元数据 */
    seg->wp_count          = flash_union_buffer[META_OFFSET + 0].uint8_type;
    seg->index             = flash_union_buffer[META_OFFSET + 1].uint8_type;
    seg->target_speed      = flash_union_buffer[META_OFFSET + 2].float_type;
    seg->arrival_threshold = flash_union_buffer[META_OFFSET + 3].float_type;
    seg->arrival_behavior  = flash_union_buffer[META_OFFSET + 4].uint8_type;
    seg->custom_event_id   = flash_union_buffer[META_OFFSET + 5].uint8_type;

    /* 3. 有效性校验: 越界数据 → 清空整段, 防止后续导航使用脏数据 */
    if (seg->wp_count > INS_SEG_MAX_WP_PER_SEG || seg->index >= INS_SEG_MAX_SEGMENTS)
        memset(seg, 0, sizeof(InsSeg_Segment));

    flash_buffer_clear();
}

/*
 * 保存元数据到 Flash 页 INS_SEG_FLASH_PAGE_META (49)。
 * 格式: [0] = INS_SEG_MAGIC (uint32), [1] = ins_seg_count (uint8)
 * 写入长度 = 2 个 uint32 (8 字节), 远小于一页(2048 字节)。
 *
 * 注意: flash_write_page_from_buffer 的 len 参数是 uint32 元素个数, 不是字节数。
 */
static void flash_save_meta(void)
{
    flash_buffer_clear();
    flash_union_buffer[0].uint32_type = INS_SEG_MAGIC;
    flash_union_buffer[1].uint8_type  = ins_seg_count;
    flash_erase_page(0, INS_SEG_FLASH_PAGE_META);
    flash_write_page_from_buffer(0, INS_SEG_FLASH_PAGE_META, 2);
    flash_buffer_clear();
}

/*
 * 从 Flash 页 49 加载元数据。
 * 返回 1 = Magic 匹配, 数据有效。
 * 返回 0 = Magic 不匹配, Flash 从未写入或已损坏 → ins_seg_count 保持原值。
 * 额外的 ins_seg_count 越界检查防止 Flash 位翻转导致计数异常。
 */
static uint8 flash_load_meta(void)
{
    flash_buffer_clear();
    flash_read_page_to_buffer(0, INS_SEG_FLASH_PAGE_META, 2);
    if (flash_union_buffer[0].uint32_type != INS_SEG_MAGIC)
    {
        flash_buffer_clear();
        return 0;
    }
    ins_seg_count = flash_union_buffer[1].uint8_type;
    if (ins_seg_count > INS_SEG_MAX_SEGMENTS)
        ins_seg_count = 0;
    flash_buffer_clear();
    return 1;
}

/* ==================================================================
 *  5. 导航核心
 *
 *  调用链 (每帧主循环):
 *    ins_seg_run_mode()
 *      → seg_calc_target()    设置 dis_ins, yaw_ins, Yao.Target_Speed, turn_mode
 *      → 检查 dis_ins < threshold → 到达航点?
 *        → seg_advance_waypoint()   wp_current++, 判断段是否完成
 *          → seg_transition()       段间转移 (STOP/NEXT/RETURN/CUSTOM)
 * ================================================================== */

/*
 * 计算当前目标的距离和方位, 并设置控制变量。
 *
 * 正常模式: 以 ins_seg_pool[ins_seg_current].waypoints[ins_seg_wp_current] 为目标
 * RETURN 模式: 以原点 (0.0, 0.0) 为目标
 *
 * 输出 (修改全局变量):
 *   dis_ins           — 到目标点的距离 (复用 Ins.c get_target)
 *   yaw_ins           — 到目标点的方位角 0~360° (复用 Ins.c get_target)
 *   Yao.Target_Speed  — 目标速度, 供 PID 速度环使用
 *   turn_mode         — 设为 7, 使能 Interrupt.c 的 yaw_ins PID 闭环转向
 */
static void seg_calc_target(void)
{
    InsSeg_Segment *seg = &ins_seg_pool[ins_seg_current];

    if (s_is_returning)
    {
        /* RETURN 模式: 目标固定为原点 (0,0)。
         * 此时段内航点已遍历完毕, ins_seg_wp_current 已重置为 0,
         * 不能再用 waypoints[0] 作为目标。 */
        get_target(cod_realtime.x, cod_realtime.y, 0.0, 0.0);
    }
    else
    {
        InsSeg_Waypoint *wp = &seg->waypoints[ins_seg_wp_current];
        get_target(cod_realtime.x, cod_realtime.y, wp->x, wp->y);
    }

    Yao.Target_Speed = (int)(seg->target_speed);
    turn_mode = 7;  /* 惯导偏航闭环模式, 见 Interrupt.c turn_mode==7 代码块 */
}

/*
 * 推进到段内下一个航点。
 * 返回 1 = 段完成 (已遍历所有航点, wp_current 已重置为 0)
 * 返回 0 = 段内还有航点待执行
 */
static uint8 seg_advance_waypoint(void)
{
    ins_seg_wp_current++;

    if (ins_seg_wp_current >= ins_seg_pool[ins_seg_current].wp_count)
    {
        ins_seg_wp_current = 0;  /* 重置, 供下一段使用 */
        return 1;                /* 当前段完成 */
    }
    return 0;                    /* 继续当前段 */
}

/*
 * 段间转移: 当前段的所有航点遍历完毕后, 根据 arrival_behavior 决定下一步。
 *
 * 返回 1 = 全部导航完成 (调用者应退出导航循环)
 * 返回 0 = 继续执行 (seg_current/seg_wp_current 已更新)
 *
 * STOP:
 *   立即停车并退出导航。所有姿态环和电机归零, turn_mode 回退到 2(级联模式)。
 *
 * NEXT:
 *   seg_current 自增。若超出 seg_count 则同 STOP, 否则从新段航点 0 开始。
 *
 * RETURN:
 *   设置 s_is_returning 标记, seg_calc_target() 持续计算到原点的距离/方位。
 *   到达原点后由 ins_seg_run_mode() 触发 STOP。
 *   注意: 这里只设标记不直接检查距离, 因为 dis_ins 由上一帧的 seg_calc_target
 *   计算的是到原最后一个航点的距离而非原点的距离。下一帧 seg_calc_target 才会
 *   因 s_is_returning==1 改用 (0,0) 作目标。
 *
 * CUSTOM:
 *   按 custom_event_id 查找事件回调。找到则调用, 以回调返回值决定完成/继续。
 *   回调内可直接修改 ins_seg_current 和 ins_seg_wp_current。
 *   找不到回调 → 退化为 STOP (防止段配置错误导致无限循环)。
 */
static uint8 seg_transition(void)
{
    InsSeg_Segment *seg = &ins_seg_pool[ins_seg_current];

    switch (seg->arrival_behavior)
    {
    case INS_SEG_BEHAVIOR_STOP:
        ins_seg_stop_navigation();
        return 1;

    case INS_SEG_BEHAVIOR_NEXT:
        ins_seg_current++;
        if (ins_seg_current >= ins_seg_count)
        {
            /* 没有更多段了, 同 STOP */
            ins_seg_stop_navigation();
            return 1;
        }
        ins_seg_wp_current = 0;
        return 0;

    case INS_SEG_BEHAVIOR_RETURN:
        /* 设置返回标记, 后续帧的 seg_calc_target 将以 (0,0) 为目标。
         * 到达判定在 ins_seg_run_mode() 中统一处理。 */
        s_is_returning = 1;
        return 0;

    case INS_SEG_BEHAVIOR_CUSTOM:
        {
            InsSeg_EventCallback cb = event_find(seg->custom_event_id);
            if (cb != NULL)
                return cb(ins_seg_current);
            /* 事件未注册 → 退化为 STOP, 避免因配置错误导致行为未定义 */
            ins_seg_stop_navigation();
            return 1;
        }

    default:
        /* 未知行为 → 安全侧: STOP */
        ins_seg_stop_navigation();
        return 1;
    }
}

/* ==================================================================
 *  6. 状态机入口
 *
 *  这两个函数由 Ins.c 的 ins_navigation() 在主循环中每帧调用:
 *    case 2: ins_seg_edit_mode();   // 分段编辑
 *    case 3: ins_seg_run_mode();    // 分段导航
 *
 *  ins_mode 的切换途径:
 *    途径1 — 菜单直设: menu.c 页1第6行变量 ins_mode, KEY_1减/KEY_3加
 *    途径2 — 程序自动:
 *      ins_mode=2 KEY_1短按 → ins_mode=3 (编辑→导航)
 *      ins_mode=3 STOP → ins_mode=0 (导航完成退出)
 * ================================================================== */

/*
 * ins_mode=2: 分段编辑模式
 *
 * 每帧执行一次 (主循环频率), 检测按键并执行对应操作。
 *
 * 按键绑定:
 *   KEY_3 短按 — 记录 cod_realtime 到当前段的航点列表末尾
 *   KEY_2 短按 — 保存所有段到 Flash
 *   KEY_2 长按 — 完成当前段编辑, 切换到下一段(自动设默认参数)
 *   KEY_1 短按 — 保存到 Flash 并进入分段导航 (ins_mode=3)
 *   KEY_1 长按 — 重置: 清空所有段数据, 从段 0 重新开始
 *
 * 显示: 使用行 144/160 (避开 Ins.c 已用的 80/96/112/128)
 *   SEG: s_edit_idx   — 当前编辑段号
 *   WP:  wp_count     — 当前段已录航点数
 *
 * 注意: key_get_state 返回的是按键状态标志, 需手动 key_clear_state 清除,
 *       否则下次检测会重复触发。
 */
void ins_seg_edit_mode(void)
{
    /* 显示当前编辑状态: 行 144/160 */
    ips200_show_string(0, 144, "SEG:");
    ips200_show_float(40, 144, (float)s_edit_idx, 1, 0);
    ips200_show_string(0, 160, "WP:");
    ips200_show_float(40, 160, (float)ins_seg_pool[s_edit_idx].wp_count, 1, 0);

    /* KEY_3 短按: 记录当前实时坐标为航点 */
    if (key_get_state(KEY_3) == KEY_SHORT_PRESS)
    {
        key_clear_state(KEY_3);
        InsSeg_Segment *seg = &ins_seg_pool[s_edit_idx];
        if (seg->wp_count < INS_SEG_MAX_WP_PER_SEG)
        {
            seg->waypoints[seg->wp_count].x = cod_realtime.x;
            seg->waypoints[seg->wp_count].y = cod_realtime.y;
            seg->wp_count++;
            seg->index = s_edit_idx;  /* 冗余记录段索引, Flash 加载时校验 */
        }
        /* wp_count 已达上限 → 静默忽略, 需用户手动切到下一段 */
    }

    /* KEY_2 短按: 保存所有段到 Flash (掉电不丢失) */
    if (key_get_state(KEY_2) == KEY_SHORT_PRESS)
    {
        key_clear_state(KEY_2);
        ins_seg_save_to_flash();
    }

    /* KEY_2 长按: 完成当前段, 创建下一段 */
    if (key_get_state(KEY_2) == KEY_LONG_PRESS)
    {
        key_clear_state(KEY_2);
        ins_seg_finish_segment();
    }

    /* KEY_1 短按: 保存并进入分段导航模式 */
    if (key_get_state(KEY_1) == KEY_SHORT_PRESS)
    {
        key_clear_state(KEY_1);
        ins_seg_save_to_flash();  /* 先保存, 确保导航从最新数据开始 */
        ins_mode = 3;
    }

    /* KEY_1 长按: 彻底重置所有段数据 (不可逆, 相当于"格式化") */
    if (key_get_state(KEY_1) == KEY_LONG_PRESS)
    {
        key_clear_state(KEY_1);
        memset(ins_seg_pool, 0, sizeof(ins_seg_pool));
        ins_seg_count = 0;
        s_edit_idx = 0;
        ins_seg_current = 0;
        ins_seg_wp_current = 0;
        s_loop_iter = 0;
        s_is_returning = 0;
        s_run_init = 0;
        /* 注意: 只清 RAM, 不清 Flash。如需清 Flash 需单独调用 ins_seg_erase_flash() */
    }
}

/*
 * ins_mode=3: 分段导航模式
 *
 * 每帧执行一次 (主循环频率)。
 *
 * 执行流程:
 *   1. 首次进入 → 从 Flash 加载所有段 + 初始化状态 + 计算首个目标
 *   2. 每帧 → seg_calc_target() 更新 dis_ins/yaw_ins
 *   3. 检测到达 → dis_ins < arrival_threshold → 推进航点
 *   4. 段完成 → seg_transition() 决定下一步 (STOP/NEXT/RETURN/CUSTOM)
 *   5. RETURN 模式 → 持续导航回原点, 到达原点时 STOP
 *
 * 显示: 使用行 144/160/176 (避开 Ins.c 的 80/96/112/128)
 *   SEG: ins_seg_current   — 当前执行段号
 *   WP:  ins_seg_wp_current — 当前段内航点号
 *   DIS: dis_ins           — 到目标点的实时距离
 *
 * 退出条件:
 *   - Flash 加载失败 → ins_mode=0 (数据无效, 防止空跑)
 *   - seg_transition() 返回 1 (STOP / NEXT超界 / CUSTOM回调返回1)
 *   - RETURN 模式到达原点
 */
void ins_seg_run_mode(void)
{
    /* ---- 首次进入: 从 Flash 加载数据 ---- */
    if (!s_run_init)
    {
        if (!ins_seg_load_from_flash())
        {
            /* Flash 数据无效 (从未录点或已损坏) → 回退, 防止空段导航 */
            ins_mode = 0;
            return;
        }

        /* 验证至少有一段有效数据 */
        if (ins_seg_count == 0)
        {
            ins_mode = 0;
            return;
        }

        /* 初始化导航状态 */
        ins_seg_current = 0;       /* 从段 0 开始 */
        ins_seg_wp_current = 0;    /* 从航点 0 开始 */
        s_loop_iter = 0;
        s_is_returning = 0;
        turn_mode = 7;             /* 惯导偏航闭环模式 */
        seg_calc_target();         /* 计算首个目标 */
        s_run_init = 1;
    }

    InsSeg_Segment *seg = &ins_seg_pool[ins_seg_current];

    /* ---- 每帧: 更新目标 ---- */
    seg_calc_target();

    /* ---- 显示当前导航状态 ---- */
    ips200_show_string(0, 144, "SEG:");
    ips200_show_float(40, 144, (float)ins_seg_current, 1, 0);
    ips200_show_string(0, 160, "WP:");
    ips200_show_float(40, 160, (float)ins_seg_wp_current, 1, 0);
    ips200_show_string(0, 176, "DIS:");
    ips200_show_float(40, 176, (float)dis_ins, 2, 2);

    /* ---- 到达判定 ---- */
    if (dis_ins < seg->arrival_threshold && dis_ins > 0)
    {
        /*
         * RETURN 模式优先检查: 到达原点则 STOP。
         * 必须放在 seg_advance_waypoint() 之前, 因为 RETURN 模式下
         * ins_seg_wp_current 已被重置为 0, 调用 seg_advance_waypoint 会误推进。
         */
        if (s_is_returning)
        {
            ins_seg_stop_navigation();
            s_is_returning = 0;
            s_run_init = 0;
            return;
        }

        /* 正常模式: 推进到段内下一个航点 */
        if (seg_advance_waypoint())
        {
            /* 段完成 → 段间转移 */
            if (seg_transition())
            {
                /* 全部导航完成 → 退出导航循环 */
                s_run_init = 0;
                return;
            }
        }
        /* 更新目标到新航点 (或转移后的新段首航点) */
        seg_calc_target();
    }
}

/* ==================================================================
 *  7. 公开 API
 * ================================================================== */

/*
 * 选择当前编辑段。
 * 若该段尚未初始化 (wp_count==0), 自动填入默认参数。
 * 通常在开始编辑新段前调用, 也可用于在编辑过程中切换段。
 */
void ins_seg_select(uint8 idx)
{
    if (idx < INS_SEG_MAX_SEGMENTS)
    {
        s_edit_idx = idx;
        if (ins_seg_pool[idx].wp_count == 0)
        {
            /* 空段: 初始化默认参数, 避免使用未初始化的字段 */
            memset(&ins_seg_pool[idx], 0, sizeof(InsSeg_Segment));
            ins_seg_pool[idx].index = idx;
            ins_seg_pool[idx].target_speed = INS_SEG_DEFAULT_SPEED;
            ins_seg_pool[idx].arrival_threshold = INS_SEG_DEFAULT_THRESHOLD;
            ins_seg_pool[idx].arrival_behavior = INS_SEG_BEHAVIOR_NEXT;
        }
    }
}

/*
 * 将当前 cod_realtime 坐标记录为当前编辑段的航点。
 * 调用前需确保已通过 ins_seg_select() 或按键选择了编辑段。
 * wp_count 达到上限时静默忽略。
 */
void ins_seg_record_waypoint(void)
{
    InsSeg_Segment *seg = &ins_seg_pool[s_edit_idx];
    if (seg->wp_count < INS_SEG_MAX_WP_PER_SEG)
    {
        seg->waypoints[seg->wp_count].x = cod_realtime.x;
        seg->waypoints[seg->wp_count].y = cod_realtime.y;
        seg->wp_count++;
        seg->index = s_edit_idx;
    }
}

/*
 * 完成当前段编辑并创建下一段。
 *   1. s_edit_idx 自增 (最大 INS_SEG_MAX_SEGMENTS-1, 不溢出)
 *   2. 若新索引超出之前的 ins_seg_count, 则更新 ins_seg_count
 *   3. 若新段为空, 填入默认参数
 */
void ins_seg_finish_segment(void)
{
    s_edit_idx++;
    if (s_edit_idx >= INS_SEG_MAX_SEGMENTS)
        s_edit_idx = INS_SEG_MAX_SEGMENTS - 1;

    /* 更新段计数: 确保导航时能遍历到当前编辑的段 */
    if (s_edit_idx + 1 > ins_seg_count)
        ins_seg_count = s_edit_idx + 1;

    InsSeg_Segment *seg = &ins_seg_pool[s_edit_idx];
    if (seg->wp_count == 0)
    {
        memset(seg, 0, sizeof(InsSeg_Segment));
        seg->index = s_edit_idx;
        seg->target_speed = INS_SEG_DEFAULT_SPEED;
        seg->arrival_threshold = INS_SEG_DEFAULT_THRESHOLD;
        seg->arrival_behavior = INS_SEG_BEHAVIOR_NEXT;
    }
}

/*
 * 设置指定段的运行时参数。
 * 通常在录完所有航点后、进入导航前调用, 覆盖默认参数。
 * 参数 custom_event_id 仅在 behavior==INS_SEG_BEHAVIOR_CUSTOM 时有效,
 * 其他 behavior 下该值被忽略。
 */
void ins_seg_set_params(uint8 idx, float speed, float threshold,
                        uint8 behavior, uint8 custom_event_id)
{
    if (idx >= INS_SEG_MAX_SEGMENTS)
        return;

    InsSeg_Segment *seg = &ins_seg_pool[idx];
    seg->target_speed = speed;
    seg->arrival_threshold = threshold;
    seg->arrival_behavior = behavior;
    seg->custom_event_id = custom_event_id;
}

/*
 * 保存所有段到 Flash。
 * 先依次保存每段数据 (页 50~50+ins_seg_count-1),
 * 最后保存元数据 (页 49)。
 *
 * 元数据最后写是为了防止: 如果写入中途断电, 下次上电加载时
 * Magic 不匹配(因为元数据还没写), 整个数据被判定为无效。
 * 如果反过来先写元数据再写段数据, 中途断电会导致元数据指向
 * 未写入的段页(全 0xFF), 加载时解析出无效数据。
 */
void ins_seg_save_to_flash(void)
{
    for (uint8 i = 0; i < ins_seg_count; i++)
        flash_save_segment(i);
    flash_save_meta();
}

/*
 * 从 Flash 加载所有段到 RAM。
 * 先读元数据 (含 Magic 校验), 校验通过后依次加载各段。
 * 返回 0 时 ins_seg_pool 保持原有数据不变 (不清零, 因为可能是瞬态故障)。
 */
uint8 ins_seg_load_from_flash(void)
{
    if (!flash_load_meta())
        return 0;

    for (uint8 i = 0; i < ins_seg_count; i++)
        flash_load_segment(i);

    return 1;
}

/*
 * 擦除所有段 Flash 数据 (页 49~57) 并清空 RAM。
 * 不可逆操作。通常在以下场景调用:
 *   - 调试: 重新录点前彻底清理
 *   - 出厂复位: 清除所有用户数据
 */
void ins_seg_erase_flash(void)
{
    flash_erase_page(0, INS_SEG_FLASH_PAGE_META);
    for (uint8 i = 0; i < INS_SEG_MAX_SEGMENTS; i++)
        flash_erase_page(0, INS_SEG_FLASH_PAGE_BASE + i);

    /* 清空 RAM 中的段数据, 与其他 static 变量保持一致 */
    memset(ins_seg_pool, 0, sizeof(ins_seg_pool));
    ins_seg_count = 0;
    s_edit_idx = 0;
    ins_seg_current = 0;
    ins_seg_wp_current = 0;
    /* 注意: s_run_init, s_is_returning, s_loop_iter 不在此清零,
     *       因为调用此函数通常意味着整个系统重置, 下次导航会重新初始化 */
}

/*
 * 开始分段导航。
 * 重置所有导航状态, 设置 ins_mode=3。
 * 实际的 Flash 加载和初始化在下一帧的 ins_seg_run_mode() 中完成。
 *
 * 设计原因: 不在本函数中立即加载 Flash, 因为主循环还没开始,
 * 立刻加载可能导致 Flash 操作与当前中断冲突。延迟到主循环首帧加载更安全。
 */
void ins_seg_start_navigation(void)
{
    ins_seg_current = 0;
    ins_seg_wp_current = 0;
    s_loop_iter = 0;
    s_is_returning = 0;
    s_run_init = 0;     /* 触发 ins_seg_run_mode 重新初始化 */
    ins_mode = 3;
}

/*
 * 停止导航 (正常完成 或 紧急停止)。
 * 执行以下操作:
 *   1. 目标速度归零 → PID 速度环输出归零
 *   2. 姿态环输出归零 → 电机停转
 *   3. 电机占空比归零 → 硬件级停止
 *   4. turn_mode 回退到 2 (级联平衡模式, 机器人原地自平衡)
 *   5. ins_mode 回退到 0
 *
 * 注意: 不清除 s_run_init, 下次进入导航时自动重新加载 Flash。
 */
void ins_seg_stop_navigation(void)
{
    Yao.Target_Speed = 0;
    Yao.Outp_Gyro_Pitch = 0;
    Yao.Outp_Angle_Pitch = 0;
    Yao.Outp_Speed_Pitch = 0;
    small_driver_set_duty(0, 0);
    turn_mode = 2;       /* 回退到级联平衡模式 */
    s_run_init = 0;
    s_is_returning = 0;
    ins_mode = 0;
}

/* ---- 状态查询 (inline 风格, 编译器会优化为直接内存读取) ---- */

uint8 ins_seg_get_count(void)       { return ins_seg_count; }
uint8 ins_seg_get_current(void)     { return ins_seg_current; }
uint8 ins_seg_get_wp_current(void)  { return ins_seg_wp_current; }
