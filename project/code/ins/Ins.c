/*
 * Ins.c
 *
 *  Created on: 2024年6月6日
 *      Author: LateRain
 *
 *  惯导模块实现 —— 航位推算 + 单轨录点/循迹 + 分段惯导转发
 *  ==========================================================
 *
 *  文件结构:
 *    1. 全局变量 — 坐标、距离、方位角、模式标志
 *    2. Flash double 读写辅助 — 64位浮点数与 Flash 32位存储的互转
 *    3. 实时坐标更新 — 航位推算 (dead reckoning)
 *    4. 目标计算 — 两点间距离和方位角
 *    5. 主状态机 — ins_navigation() 四个模式的 switch
 *
 *  Flash 布局 (sector 0):
 *   页 2:  航点坐标数据 (cod_saved → cod_target)
 *   页 12: 航点数量 n
 *
 *  坐标系约定:
 *   X 轴正方向 = 机器人初始偏航角 0° 方向 (cos 分量)
 *   Y 轴正方向 = 机器人初始偏航角 90° 方向 (sin 分量)
 *   yaw 角: 0~359°, 从 X 轴正方向逆时针旋转
 *
 *  航位推算原理:
 *   每 16ms 中断调用一次 get_realtime_coordinate():
 *     dx = speed × 0.016 × cos(yaw_rad)
 *     dy = speed × 0.016 × sin(yaw_rad)
 *   累加到 cod_realtime, 本质上是对速度的离散时间积分。
 */

#include "zf_common_headfile.h"
#include "Ins.h"
#include "Interrupt.h"

/* ==================================================================
 *  1. 全局变量
 * ================================================================== */

Coordinates cod_realtime;       /* 实时坐标: 上电后由航位推算持续累加, 断电丢失 */
Coordinates cod_saved[30];      /* 录制暂存: ins_mode=0 按键录点时暂存 (最多30个) */
Coordinates cod_target[30];     /* 导航目标: ins_mode=1 从 Flash 加载的航点序列 */

double dis_ins;                /* 到当前目标点的距离 (由 get_target 计算) */
double yaw_ins;                /* 到当前目标点的方位角 0~360° (由 get_target 计算) */

uint8 n = 0;                   /* 航点数量: 录制时递增, 加载时从 Flash 页12读取 */
uint8 target = 1;              /* 当前目标航点索引: 从1开始(0=原点/库位) */
float temp_erect_speed_go;     /* 预留: 暂未使用 */
uint8 ins_mode = 0;            /* 惯导模式: 0=录点 1=循迹 2=分段编辑 3=分段导航 */
bool flag_1;                   /* 首次进入 ins_mode=1 的标志: true=需从Flash加载 */

/*
 * flag_save: Flash 保存状态标记
 *   0 = 未保存 (默认)
 *   1 = 已保存 (KEY_2 按下后置1)
 * 用于屏幕上显示保存状态, 无逻辑控制作用。
 */
uint8 flag_save = 0;

/* ==================================================================
 *  2. Flash double ↔ uint32 读写辅助
 *
 *  CYT4BB7 Flash 操作以 uint32 为最小单位 (flash_union_buffer[i].uint32_type)。
 *  double 是 64 位 IEEE 754 浮点数, 需拆分为两个 uint32 存储。
 *
 *  拆分方案 (little-endian, ARM Cortex-M7):
 *    double 在内存中占 8 字节, 低 32 位在前。
 *    *(uint64_t*)&d 将 double 按 uint64 解释 → 取高 32 位和低 32 位。
 *    hi = (uint32_t)(*(uint64_t*)&d >> 32)   // 高32位
 *    lo = (uint32_t)(*(uint64_t*)&d & 0xFFFFFFFF)  // 低32位
 *
 *  存储布局 (flash_union_buffer 中):
 *    每对坐标(x,y)占 4 个 uint32: [x_hi, x_lo, y_hi, y_lo]
 *    每页(FLASH_PAGE_LENGTH=512 个 uint32)可存 512/4 = 128 对坐标
 *    实际使用: 30 对坐标占 120 个 uint32, 远小于一页容量
 * ================================================================== */

extern flash_data_union flash_union_buffer[FLASH_PAGE_LENGTH];

/*
 * 将一对 double 坐标写入 flash_union_buffer。
 *   data1: X 坐标 (double)
 *   data2: Y 坐标 (double)
 *   z:     写入位置索引 (0~127)
 *          对应 buffer 位置: [4z, 4z+1, 4z+2, 4z+3]
 *
 * 调用方需在填充完 buffer 后调用 flash_write_page_from_buffer() 写入 Flash,
 * 然后调用 flash_buffer_clear() 清除 buffer。
 */
void writeDoubleToFlash1(double data1, double data2, uint8 z)
{
    /* 将第一个 double (X 坐标) 拆分为高/低 32 位 */
    uint32_t upperPart1 = (uint32_t)((*(uint64_t *)&data1) >> 32);
    uint32_t lowerPart1 = (uint32_t)((*(uint64_t *)&data1) & 0xFFFFFFFF);

    /* 将第二个 double (Y 坐标) 拆分为高/低 32 位 */
    uint32_t upperPart2 = (uint32_t)((*(uint64_t *)&data2) >> 32);
    uint32_t lowerPart2 = (uint32_t)((*(uint64_t *)&data2) & 0xFFFFFFFF);

    /* 按顺序写入 buffer:
     *   [4z+0] = X_hi, [4z+1] = X_lo
     *   [4z+2] = Y_hi, [4z+3] = Y_lo */
    flash_union_buffer[4 * z].uint32_type = upperPart1;
    flash_union_buffer[4 * z + 1].uint32_type = lowerPart1;
    flash_union_buffer[4 * z + 2].uint32_type = upperPart2;
    flash_union_buffer[4 * z + 3].uint32_type = lowerPart2;
}

/*
 * 从 flash_union_buffer 中读取一个 double 值。
 *   a: 选择读 X 还是 Y
 *      false(0) = 读 X 坐标 (前两个 uint32)
 *      true(1)  = 读 Y 坐标 (后两个 uint32)
 *   x: 读取位置索引 (0~127), 对应 buffer 位置 [4x .. 4x+3]
 *
 * 调用前需先 flash_read_page_to_buffer() 将 Flash 页读到 buffer。
 */
double readFlash_to_double1(bool a, uint8 x)
{
    uint32_t upperPart1;
    uint32_t lowerPart1;

    if (a == 0) /* 读取 X 坐标: buffer[4x], buffer[4x+1] */
    {
        upperPart1 = flash_union_buffer[4 * x].uint32_type;
        lowerPart1 = flash_union_buffer[4 * x + 1].uint32_type;
    }
    else if (a == 1) /* 读取 Y 坐标: buffer[4x+2], buffer[4x+3] */
    {
        upperPart1 = flash_union_buffer[4 * x + 2].uint32_type;
        lowerPart1 = flash_union_buffer[4 * x + 3].uint32_type;
    }

    /* 将两个 uint32 拼接为 uint64, 再按 double 解释 */
    uint64_t combinedData = ((uint64_t)upperPart1 << 32) | lowerPart1;
    double result = *(double *)&combinedData;

    return result;
}

/* ==================================================================
 *  3. 实时坐标更新 (航位推算)
 *
 *  调用频率: 每 16ms 一次 (在 Interrupt.c 的 16ms 中断中调用)
 *  时间参数: time = 0.016 秒 (16ms)
 *
 *  精度分析:
 *    speed 为电机编码器速度 (有符号整数)。
 *    实际物理速度需乘以标定系数, 此函数仅做数学运算,
 *    speed 的单位转换由调用方负责。
 *
 *    cos/sin 使用 double 精度计算, 结果累加到 double 坐标,
 *    避免 float 在长时间运行后的累积误差。
 *
 *  副作用: 向 IPS200 屏幕输出 X/Y 坐标 (行 96/112)
 * ================================================================== */

void get_realtime_coordinate(int speed, float time, float yaw)
{
    double dx, dy;
    double temp1;

    /* 偏航角 度 → 弧度 */
    temp1 = ANGLE_TO_RAD(yaw);
    /* 航位推算: 速度 × 时间 = 位移, 分解到 X/Y 轴 */
    dx = speed * time * cos(temp1);
    dy = speed * time * sin(temp1);

    /* 累加到实时坐标 (积分) */
    cod_realtime.x += dx;
    cod_realtime.y += dy;

    /* 屏幕显示 (行 96/112, 与分段惯导的 144/160/176 不冲突) */
    ips200_show_string(0 , 96 , "X:");
    ips200_show_float( 60 , 96 , cod_realtime.x , 3 , 3 );
    ips200_show_string(0 , 112 , "Y:");
    ips200_show_float( 60 , 112 , cod_realtime.y , 3 , 3 );
}

/*
 * 计算从 (x1,y1) 到 (x2,y2) 的:
 *   dis_ins = 欧几里得距离
 *   yaw_ins = 方位角 (0~360°, 从 X 轴正方向逆时针)
 *
 * 此函数被以下模块复用:
 *   - Ins.c ins_mode=1: 单轨循迹目标计算
 *   - ins_segment.c seg_calc_target(): 分段导航目标计算
 *   - ins_segment.c seg_transition() RETURN: 返回原点计算
 *
 * atan2 返回值范围 [-π, +π], 加 2π 后转换到 [0, 2π),
 * 再转角度得到 [0, 360°)。
 */
void get_target(double x1, double y1, double x2, double y2)
{
    double temp1, temp2;
    double dx = x2 - x1;   /* X 方向差 */
    double dy = y2 - y1;   /* Y 方向差 */

    /* atan2(dy, dx): 返回弧度 [-π, +π] */
    temp1 = atan2(dy, dx);
    /* sqrt: 欧几里得距离 */
    temp2 = sqrt(dx * dx + dy * dy);

    /* 将角度转换到 [0, 2π) 范围 */
    if (temp1 < 0)
        temp1 += 2 * PI;

    dis_ins = temp2;                   /* 距离 */
    yaw_ins = RAD_TO_ANGLE(temp1);     /* 弧度 → 度 [0, 360) */
}

/* ==================================================================
 *  4. 主状态机 —— ins_navigation()
 *
 *  调用: 在 main_cm7_0.c 主循环中, 当 ins_open==1 时每帧调用。
 *  职责: 根据 ins_mode 分发到对应的处理逻辑。
 *
 *  显示: 行 80(n) 和行 128(flag_save, target), 与分段惯导不冲突。
 *
 *  ins_mode 切换路径:
 *    初始化: ins_mode=0
 *    ins_mode=0 → (KEY_1) → ins_mode=1  单轨录点→循迹
 *    ins_mode=2 → (KEY_1) → ins_mode=3  分段编辑→分段导航
 *    ins_mode=3 → (STOP) → ins_mode=0   导航完成自动退出
 *    菜单页1可直接修改 ins_mode 为任意值
 * ================================================================== */

void ins_navigation(void)
{
    /* 屏幕显示: 行 80/128 */
    

    switch (ins_mode)
    {

    /* ================================================================
     *  ins_mode=0: 单轨录点模式
     *
     *  用途: 遥控机器人走一圈, 按键记录沿途航点。
     *
     *  KEY_3 短按 → 记录当前 cod_realtime 坐标到 cod_saved[n], n++
     *  KEY_2 短按 → 保存到 Flash:
     *                先检查并擦除页2(坐标)和页12(n),
     *                然后写坐标到页2, 写 n 到页12。
     *  KEY_1 短按 → 切换到导航模式 ins_mode=1
     * ================================================================ */
    case 0:
        /* KEY_3 短按: 记录当前坐标为航点 */
        if (key_get_state(KEY_3) == KEY_SHORT_PRESS)
        {
            key_clear_state(KEY_3);
            cod_saved[n].x = cod_realtime.x;
            cod_saved[n].y = cod_realtime.y;
            n++;                       /* 航点计数 +1 */
            ins_getdata = 1;           /* 标记有新数据 (供其他模块查询) */
        }

        /* KEY_2 短按: 保存航点到 Flash */
        if (key_get_state(KEY_2) == KEY_SHORT_PRESS)
        {
            key_clear_state(KEY_2);

            /* 先擦除旧数据 (Flash 写入前必须擦除) */
            if (flash_check(0, 2))     /* 页2有数据 → 擦除 */
                flash_erase_page(0, 2);
            if (flash_check(0, 12))    /* 页12有数据 → 擦除 */
                flash_erase_page(0, 12);

            /* 写坐标数据到页2 */
            if (!flash_check(0, 2))    /* 确认已擦除 (全0xFF) */
            {
                for (uint8 nn = 0; nn < n; nn++)
                    writeDoubleToFlash1(cod_saved[nn].x, cod_saved[nn].y, nn);
                flash_write_page_from_buffer(0, 2, FLASH_PAGE_LENGTH);
                flag_save = 1;         /* 标记已保存 */
                flash_buffer_clear();
            }

            /* 写航点数量 n 到页12 */
            if (!flash_check(0, 12))
            {
                flash_union_buffer[0].uint8_type = n;
                flash_write_page_from_buffer(0, 12, FLASH_PAGE_LENGTH);
                flash_buffer_clear();
            }
        }

        /* KEY_1 短按: 进入导航模式 */
        if (key_get_state(KEY_1) == KEY_SHORT_PRESS)
        {
            key_clear_state(KEY_1);
            ins_getdata = 0;           /* 清除新数据标记 */
            flag_1 = 1;                /* 触发 case 1 的 Flash 加载 */
            ins_mode = 1;
            flag_save = 0;
        }
        break;

    /* ================================================================
     *  ins_mode=1: 单轨循迹模式
     *
     *  首次进入 (flag_1==1):
     *    从 Flash 页12 读取 n (航点数量)
     *    从 Flash 页2  读取 cod_target[0..n-1] (航点坐标)
     *    此加载只执行一次, 之后 flag_1=0。
     *
     *  每帧:
     *    get_target() → 计算 dis_ins/yaw_ins (到当前目标航点)
     *
     *  到达判定: dis_ins < 20 (固定阈值) 且 dis_ins != 0
     *
     *  特殊处理 target == 0:
     *    航点 0 是原点/库位。到达 target==0 表示完成一圈回到原点→停车退出。
     *
     *  正常推进: target++ → 指向下一航点
     *    若 target == n → target = 0 (循环回第一个航点, 即原点)
     * ================================================================ */
    case 1:
        /* 首次进入: 从 Flash 加载航点数据 */
        if (flag_1)
        {
            /* 读取航点数量 n */
            flash_read_page_to_buffer(0, 12, FLASH_PAGE_LENGTH);
            n = flash_union_buffer[0].uint8_type;
            flash_buffer_clear();

            /* 读取所有航点坐标 (从页2, 每对坐标占4个uint32位置) */
            for (uint8 nnn = 0; nnn < n; nnn++)
            {
                flash_read_page_to_buffer(0, 2, FLASH_PAGE_LENGTH);
                cod_target[nnn].x = readFlash_to_double1(0, nnn);  /* 读X */
                cod_target[nnn].y = readFlash_to_double1(1, nnn);  /* 读Y */
            }
            flash_buffer_clear();

            flag_1 = 0;  /* 加载完成, 后续帧不再加载 */
        }

        /* 每帧: 计算到当前目标航点的距离和方位 */
        get_target(cod_realtime.x, cod_realtime.y,
                   cod_target[target].x, cod_target[target].y);

        /* 到达判定: dis_ins < 20 (与旧代码硬编码常数一致) */
        if (dis_ins < 20 && dis_ins != 0)
        {
            /* target==0 表示回到库位 → 停车退出 */
            if (target == 0)
            {
                /* 姿态和速度全部归零, 电机关闭 */
                Yao.Outp_Gyro_Pitch = 0;
                Yao.Outp_Angle_Pitch = 0;
                Yao.Outp_Speed_Pitch = 0;
                small_driver_set_duty(0, 0);
                break;  /* 退出 switch, 保持在 ins_mode=1 但不再执行 */
            }

            /* 推进到下一个航点 */
            target++;

            /* 航点循环: 到达最后一个航点后, 下一目标是原点(索引0) */
            if (target == n)
                target = 0;
        }
        break;

    /* ================================================================
     *  ins_mode=2: 分段编辑模式 → 转发到 ins_segment.c
     *  ins_mode=3: 分段导航模式 → 转发到 ins_segment.c
     *
     *  两个 case 仅做转发, 所有逻辑在 ins_segment.c 中实现。
     *  这样 Ins.c 无需引入任何分段惯导的内部实现细节,
     *  只通过 Ins.h → ins_segment.h 的声明来调用。
     * ================================================================ */
    case 2:
        ins_seg_edit_mode();
        break;

    case 3:
        ins_seg_run_mode();
        break;

    default:
        break;
    }
}
