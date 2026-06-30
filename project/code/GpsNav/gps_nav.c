/*********************************************************************************************************************
 * CYT4BB 智能车 GPS 导航 -  核心导航模块
 *
 * 文件: gps_nav.c
 * 模块: M2 核心导航
 * 功能: 状态机 + 方位计算 + 低通滤波 + GPS 转向融合
 *
 * 任务:
 *   T08: 状态机 (IDLE、CALIBRATING、NAVIGATING、ARRIVED、COMPLETE)
 *   T09: 方位计算/到达检测 + 低通滤波
 *   T10: 航点推进 (到达判定)
 *   T11: 漂移修正 + 转向融合 + GPS 转向输出
 *
 * 架构:
 *   gps_nav_proc()  在主循环 while 中以 10Hz 调用
 *   导航输出 buf[write_idx] 写入
 *   ISR 从 buf[read_idx] 读取，约 250Hz
 *
 * GPS 信号保护:
 *   连续丢失帧 (10Hz 下 5s = 50 帧) 自动停止导航
 *   信号恢复时重置丢失帧计数
 *   利用 system_getval_us() 获取微秒时间戳
 ********************************************************************************************************************/

#include "gps_nav.h"
#include "gps_waypoint.h"
#include "gps_calibration.h"
#include "zf_device_gnss.h"
#include "zf_device_imu660ra.h"

/* turn_mode 在 Interrupt.h 中定义 */
extern uint8 turn_mode;

//====================================================全局变量====================================================

gps_steer_pp_t  gps_steer_pp;
uint8           gps_nav_state = GPS_NAV_IDLE;

//====================================================私有变量====================================================

static float    lpf_bearing       = GPS_NAV_FIRST_FRAME_MAGIC;  // 低通滤波方位角（初始值=魔法值）
static uint8    is_near_waypoint  = 0;                          // 是否接近航点（滞回标记）
static uint8    gps_loss_frames   = 0;                          // GPS 信号丢失帧数

//====================================================私有函数====================================================

static void     gps_steer_commit(void);
static gps_steer_output_t* gps_steer_write_buf(void);
static float    angle_lpf_circular(float old_val, float new_val, float alpha);

/*--------------------------------------------------------------------------------------------------------------------
 * 功能: 交换 write_idx <-> read_idx
 * 原理: 先写完 buf[write_idx] 再交换
 * 作用: ISR 读取时不会读到半写数据
 * 注意: volatile uint8 写入在 ARM 上是原子的
 *--------------------------------------------------------------------------------------------------------------------*/
static void gps_steer_commit(void)
{
    uint8 old_write = gps_steer_pp.write_idx;
    uint8 old_read  = gps_steer_pp.read_idx;
    gps_steer_pp.read_idx  = old_write;   // ISR 读取新写入的数据
    gps_steer_pp.write_idx = old_read;    // 下次写入旧读取位置
}

/*--------------------------------------------------------------------------------------------------------------------
 * 获取写入缓冲区指针 (主循环 while 专用)
 *--------------------------------------------------------------------------------------------------------------------*/
static gps_steer_output_t* gps_steer_write_buf(void)
{
    return &gps_steer_pp.buf[gps_steer_pp.write_idx];
}

/*--------------------------------------------------------------------------------------------------------------------
 * 功能: 环形低通滤波，支持跨越±180度
 * 防止: 179度 到 -179度 差值只有 2度而非 358度
 *--------------------------------------------------------------------------------------------------------------------*/
static float angle_lpf_circular(float old_val, float new_val, float alpha)
{
    float diff = new_val - old_val;
    if (diff > 180.0f)  diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;
    return old_val + alpha * diff;
}

//====================================================T08: 状态机====================================================

/*--------------------------------------------------------------------------------------------------------------------
 * 功能: 初始化导航模块 + 航点系统 + 状态机
 * 依赖: Flash 航点 + gnss 驱动 + IMU 数据
 *--------------------------------------------------------------------------------------------------------------------*/
void gps_nav_init(void)
{
    /* 初始化航点 */
    gps_wp_init();

    /* 初始化双缓冲：写入端=0，读取端=1 */
    gps_steer_pp.buf[0].target_bearing_deg = 0.0f;
    gps_steer_pp.buf[0].distance_to_wp_m   = 0.0f;
    gps_steer_pp.buf[0].imu_yaw_offset_deg = 0.0f;
    gps_steer_pp.buf[1].target_bearing_deg = 0.0f;
    gps_steer_pp.buf[1].distance_to_wp_m   = 0.0f;
    gps_steer_pp.buf[1].imu_yaw_offset_deg = 0.0f;
    gps_steer_pp.write_idx = 0;
    gps_steer_pp.read_idx  = 1;

    /* 初始化状态机 */
    gps_nav_state = GPS_NAV_IDLE;
    lpf_bearing      = GPS_NAV_FIRST_FRAME_MAGIC;
    is_near_waypoint = 0;
    gps_loss_frames  = 0;
}

/*--------------------------------------------------------------------------------------------------------------------
 * 功能: 10Hz 主循环处理函数
 * 前置: gnss_flag==1 时由 gnss_data_parse() 置位
 *--------------------------------------------------------------------------------------------------------------------*/
void gps_nav_proc(void)
{
    /* === T11: GPS 信号保护 === */
    if (gnss.state != 1)
    {
        if (gps_nav_state == GPS_NAV_NAVIGATING || gps_nav_state == GPS_NAV_ARRIVED)
        {
            gps_loss_frames++;
            if (gps_loss_frames >= GPS_NAV_SIGNAL_LOSS_FRAMES)
            {
                /* 连续丢失 5s 则停止导航 */
                gps_nav_stop();
                return;
            }
        }
        /* 无信号时不更新 ISR 输出 */
        return;
    }

    /* GPS 信号恢复，重置丢失帧计数 */
    gps_loss_frames = 0;

    /* === T11: turn_mode 保护 === */
    if (turn_mode != 5 && gps_nav_state != GPS_NAV_IDLE)
    {
        gps_nav_state = GPS_NAV_IDLE;
        /* 警告: ISR 中 turn_mode!=5 时不会读取 GPS 输出
         * 但 read_idx 仍指向旧数据（安全）
         */
    }

    /* === 状态机处理 === */
    switch (gps_nav_state)
    {
    case GPS_NAV_IDLE:
        /* 等待 gps_nav_start() 调用并设置 turn_mode==5 */
        break;

    case GPS_NAV_CALIBRATING:
    {
        /* 校准: 用 IMU yaw 与 GPS 方位角计算初始偏移 */
        if (gps_cal_startpoint())
        {
            /* 校准成功 */
            lpf_bearing      = GPS_NAV_FIRST_FRAME_MAGIC;  // 重置低通滤波
            is_near_waypoint = 0;
            gps_nav_state    = GPS_NAV_NAVIGATING;
        }
        else
        {
            /* 校准失败: 航点数 < 2 或偏移过大 */
            gps_nav_state = GPS_NAV_IDLE;
        }
        break;
    }

    case GPS_NAV_NAVIGATING:
    {
        /* === T09: 方位计算 === */
        gps_waypoint_t *wp = gps_wp_current();
        if (wp == (gps_waypoint_t *)0)
        {
            gps_nav_stop();
            break;
        }

        /* 计算方位角 [0, 360)  (度) */
        float raw_bearing = (float)get_two_points_azimuth(
            gnss.latitude, gnss.longitude, wp->lat, wp->lng);
        float distance = (float)get_two_points_distance(
            gnss.latitude, gnss.longitude, wp->lat, wp->lng);

        /* 低通滤波: 平滑方位角跳变 (度) */
        if (lpf_bearing == GPS_NAV_FIRST_FRAME_MAGIC)
        {
            lpf_bearing = raw_bearing;
        }
        else
        {
            lpf_bearing = angle_lpf_circular(lpf_bearing, raw_bearing, GPS_NAV_LPF_ALPHA);
        }

        /* 距离 0 时直接标记到达 */
        if (distance < 0.01f)
        {
            gps_nav_state = GPS_NAV_ARRIVED;
            break;
        }

        /* === T11: 漂移修正 === */
        gps_cal_drift_correction();

        /* 更新转向输出 */
        gps_steer_output_t *out = gps_steer_write_buf();
        out->target_bearing_deg = lpf_bearing;
        out->distance_to_wp_m   = distance;
        out->imu_yaw_offset_deg = gps_cal_get_offset();

        /* 提交到 ISR */
        gps_steer_commit();

        /* === T16: debug output (uncomment for wireless serial) === */
        // printf("NAV: wp=%d, bear=%.1f, dist=%.1f, yaw=%.1f, des=%.1f, off=%.1f\r\n",
        //        gps_wp_get_current_index(), lpf_bearing, distance,
        //        imu660ra.eulerAngle.yaw,
        //        lpf_bearing + gps_cal_get_offset(),
        //        gps_cal_get_offset());

        /* === T10: 航点到达判定 (滞回) === */
        if (!is_near_waypoint)
        {
            /* 进入判定: 距离 < 进入阈值 且 首次进入 */
            if (distance < GPS_NAV_ARRIVE_ENTER_M)
            {
                is_near_waypoint = 1;
                gps_nav_state    = GPS_NAV_ARRIVED;
            }
        }
        else
        {
            /* 离开判定: 距离 > 离开阈值 时重置标记 */
            if (distance > GPS_NAV_ARRIVE_LEAVE_M)
            {
                is_near_waypoint = 0;
            }
        }
        break;
    }

    case GPS_NAV_ARRIVED:
    {
        /* 推进航点 */
        if (gps_wp_advance())
        {
            /* 还有下一航点 */
            is_near_waypoint = 0;
            lpf_bearing      = GPS_NAV_FIRST_FRAME_MAGIC;  // 重置低通滤波
            gps_nav_state    = GPS_NAV_NAVIGATING;
        }
        else
        {
            /* 所有航点已走完 */
            gps_nav_state = GPS_NAV_COMPLETE;
        }
        break;
    }

    case GPS_NAV_COMPLETE:
    {
        /* 导航完成: 关闭转向 turn_mode=0 */
        turn_mode     = 0;
        gps_nav_state = GPS_NAV_IDLE;
        break;
    }

    default:
        gps_nav_state = GPS_NAV_IDLE;
        break;
    }
}

/*--------------------------------------------------------------------------------------------------------------------
 * 功能: 启动导航
 * 前置: 航点数 >= 2
 *--------------------------------------------------------------------------------------------------------------------*/
void gps_nav_start(void)
{
    if (!gps_wp_set.valid || gps_wp_set.count < 2)
        return;

    gps_nav_state = GPS_NAV_CALIBRATING;
    turn_mode     = 5;
}

/*--------------------------------------------------------------------------------------------------------------------
 * 功能: 停止导航，复位到 IDLE + turn_mode=0
 *--------------------------------------------------------------------------------------------------------------------*/
void gps_nav_stop(void)
{
    gps_nav_state = GPS_NAV_IDLE;
    turn_mode     = 0;

    /* 清零输出（防止 ISR 读到旧数据） */
    gps_steer_output_t *out = gps_steer_write_buf();
    out->target_bearing_deg = 0.0f;
    out->distance_to_wp_m   = 0.0f;
    out->imu_yaw_offset_deg = 0.0f;
}

/*--------------------------------------------------------------------------------------------------------------------
 * 功能: 获取当前导航状态
 *--------------------------------------------------------------------------------------------------------------------*/
uint8 gps_nav_get_state(void)
{
    return gps_nav_state;
}

/*--------------------------------------------------------------------------------------------------------------------
 * 功能: 设置航点索引
 * 返回: 1=成功, 0=失败（索引越界）
 *--------------------------------------------------------------------------------------------------------------------*/
uint8 gps_nav_set_wp_index(uint8 idx)
{
    if (idx >= gps_wp_set.count)
        return 0;

    gps_wp_set.current_index = idx;
    is_near_waypoint = 0;
    lpf_bearing      = GPS_NAV_FIRST_FRAME_MAGIC;  // 重置低通滤波
    return 1;
}
