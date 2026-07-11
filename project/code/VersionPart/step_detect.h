/**
 * @file    step_detect.h
 * @brief   黑白交替台阶识别与跳跃控制模块
 * @note    独立于 image.c 视觉判断逻辑，专用于台阶环境下的黑白交替检测
 *
 * 台阶环境说明：
 *   - 第一级（地面）和第三级为低灰度（黑色）
 *   - 第二级和第四级为高灰度（白色）
 *   - 小车在每次台阶切换（黑→白 或 白→黑）时触发跳跃
 */

#ifndef _STEP_DETECT_H_
#define _STEP_DETECT_H_

#include "zf_common_typedef.h"

/*===========================================================================
 * 图像尺寸宏定义（与 image.h 中 Image_X / Image_Y 对应）
 *===========================================================================*/
#define STEP_IMAGE_X 80 /**< 压缩后图像宽度（像素） */
#define STEP_IMAGE_Y 60 /**< 压缩后图像高度（像素） */

/*===========================================================================
 * 台阶检测可调参数宏定义
 *===========================================================================*/

/** @brief 行扫描：待扫描的三行（图像坐标，行号越小越靠近图像顶部/远处） */
#define SCAN_LINE_1 57 /**< 扫描行1（较远处，约图像下1/3处） */
// #define SCAN_LINE_2     35      /**< 扫描行2（中间区域） */
// #define SCAN_LINE_3     50      /**< 扫描行3（较近处，约图像下4/5处） */

/** @brief 白色像素个数阈值：超过此阈值则认为该行为"白台阶" */
#define STEP_WHITE_THRESHOLD_MAX 200
#define STEP_WHITE_THRESHOLD_MIN 40

/** @brief 消抖时间参数（毫秒） */
#define STEP_DEBOUNCE_MS 80 /**< 状态稳定所需最短持续时间 */

/** @brief 检测函数调用周期（毫秒），用于计算消抖计数上限 */
#define STEP_CALL_PERIOD_MS 8 /**< 假定每 8ms 调用一次检测函数 */

/** @brief 消抖计数值上限 = DEBOUNCE_MS / CALL_PERIOD_MS */
#define STEP_DEBOUNCE_CNT_MAX (STEP_DEBOUNCE_MS / STEP_CALL_PERIOD_MS)

/*===========================================================================
 * 外部接口变量声明
 *===========================================================================*/
extern uint8 Step_flag_jump;                             /**< 跳跃标志位：1=触发跳跃动作 */
extern uint8 Step_flag_change;                           /**< 台阶状态标志位：1=白台阶  0=黑台阶 */
extern uint8 Step_Bin_Image[STEP_IMAGE_Y][STEP_IMAGE_X]; /**< 二值化后的台阶图像 */
extern uint8 Step_Src_Image[STEP_IMAGE_Y][STEP_IMAGE_X]; /**< 压缩后的原始灰度图像（用于图传） */
extern short otsu_threshold;
/*===========================================================================
 * 外部接口函数声明
 *===========================================================================*/

/**
 * @brief  台阶检测综合图像处理（压缩 + OTSU + 二值化）
 * @note   内部依次执行：原始图像压缩 → 大津法求阈值 → 全局二值化
 *         处理结果存入 Step_Bin_Image
 */
void Step_Process_Image(void);

/**
 * @brief  台阶检测与跳跃触发状态机
 * @note   必须周期性调用（建议 2ms 周期），内部含 16ms 消抖逻辑。
 *         检测 SCAN_LINE_1/2/3 三行白色像素占比，判定当前处于白/黑台阶，
 *         当 flag_change 发生边沿跳变时置位 flag_jump 触发跳跃。
 */
void Step_Detect_And_Jump(void);

#endif /* _STEP_DETECT_H_ */