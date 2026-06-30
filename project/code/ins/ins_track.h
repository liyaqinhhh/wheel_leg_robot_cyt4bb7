/*
 * ins_track.h
 * 轨迹存取与 Pure Pursuit 循迹声明
 */

#ifndef _INS_TRACK_H_
#define _INS_TRACK_H_

#include "zf_common_typedef.h"

 //------------------------------------------- Flash 页配置 ----------------------------------------------------
#define INS_TRACK_FLASH_PAGE_MAX       24       // 最多使用 24 页存储轨迹数据
#define INS_TRACK_FLASH_PAGE_BEGIN     26       // 起始页编号（页 25 留给 IMU 零偏）
#define INS_TRACK_FLASH_META_PAGE      0        // 元数据页（使用页 0）
#define INS_TRACK_FLOAT_PER_POINT      4        // 每个轨迹点 4 个 float（x, y, yaw, speed_dir）
#define INS_TRACK_PAGE_FLOAT_NUM       510      // 每页可用 float 数量（512-2 预留）
#define INS_TRACK_POINT_PER_PAGE       (INS_TRACK_PAGE_FLOAT_NUM / INS_TRACK_FLOAT_PER_POINT)  // 127

 //------------------------------------------- 循迹参数 --------------------------------------------------------
#define INS_TRACK_SAMPLE_STEP          0.1f     // 采样间距（米），每 10cm 记录一个点
#define INS_TRACK_LOOKAHEAD_DIST       0.4f     // Pure Pursuit 前视距离（米）
#define INS_TRACK_FOLLOW_SPEED         0.35f    // 默认循迹速度（m/s）
#define INS_TRACK_WHEELBASE            0.2107f  // 轮距（米）【待实测】
#define INS_TRACK_STEER_FILTER_ALPHA   0.85f    // 转向一阶低通滤波系数

 //------------------------------------------- 轨迹点结构 ------------------------------------------------------
typedef struct
{
    float x;
    float y;
    float yaw;
    float speed_dir;   // 1.0=前进，0.0=后退
} Ins_TrackPoint;

 //------------------------------------------- API 声明 --------------------------------------------------------

// @brief  轨迹模块初始化，从Flash加载轨迹元数据
void ins_track_init(void);

// 轨迹记录
// @brief  开始轨迹记录模式
void ins_track_start_save(void);
// @brief  停止轨迹记录并保存到Flash
void ins_track_stop_save(void);
// @brief  清除Flash中的轨迹数据
void ins_track_clear(void);
// @brief  每4ms调用，根据采样间距自动判断是否需要记录当前轨迹点
void ins_track_push_point(void);                // 每 4ms 调用，自动判断是否需要采样

// 轨迹循迹
// @brief  开始Pure Pursuit循迹模式，从Flash加载轨迹数据
void ins_track_start_follow(void);
// @brief  停止循迹模式
void ins_track_stop_follow(void);
// @brief  每4ms调用，执行Pure Pursuit算法计算转向输出
void ins_track_follow_proc(void);               // 每 4ms 调用，执行 Pure Pursuit

// 状态查询
// @brief  获取当前已记录的轨迹点总数
uint32 ins_track_get_point_count(void);
// @brief  查询是否正在记录轨迹
uint8 ins_track_is_saving(void);
// @brief  查询是否正在循迹
uint8 ins_track_is_following(void);
// @brief  获取滤波后的转向角度输出（度）
float ins_track_get_steer_output_deg(void);     // 获取滤波后的转向角度（度）

// 统一入口
// @brief  每4ms统一调用入口，内部根据状态判断执行记录或循迹
void ins_track_proc(void);                      // 每 4ms 调用，内部判断记录/循迹

#endif /* _INS_TRACK_H_ */
