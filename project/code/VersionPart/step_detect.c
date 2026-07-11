/**
 * @file    step_detect.c
 * @brief   黑白交替台阶识别与跳跃控制模块 —— 实现
 *
 * 处理流程：
 *   1. Step_Process_Image():  原始图像压缩 → 大津法OTSU求阈值 → 二值化
 *   2. Step_Detect_And_Jump(): 三行扫描白色像素统计 → 16ms消抖 → 边沿触发跳跃
 *
 * 算法来源：
 *   - 大津法(OTSU) 移植自 image.c 第 34~91 行
 *   - 图像压缩    移植自 image.c 第 633~675 行 Copy_Zip_Image()
 */

#include "zf_common_headfile.h"
#include "math.h"
#include "step_detect.h"
#include "zf_device_mt9v03x.h"
#include "servo.h"

/*===========================================================================
 * 模块内部全局变量
 *===========================================================================*/

/** 压缩后的原始灰度图像（80x60），作为 OTSU 输入 */
uint8 Step_Src_Image[STEP_IMAGE_Y][STEP_IMAGE_X];

/** 二值化后的台阶图像：0=黑色(低灰度台阶)  1=白色(高灰度台阶) */
uint8 Step_Bin_Image[STEP_IMAGE_Y][STEP_IMAGE_X];

/** 跳跃标志位：边缘跳变时置1，外部读取后需清零 */
uint8 Step_flag_jump = 0;

/** 台阶状态标志位：0=黑台阶  1=白台阶 */
uint8 Step_flag_change = 0;

/*===========================================================================
 * 步骤一：大津法 (OTSU) 移植
 * 来源：image.c 第 34~91 行 GetOSTU()
 * 变更：变量名加前缀 Step_，尺寸宏替换为 STEP_IMAGE_X / STEP_IMAGE_Y
 *===========================================================================*/

/**
 * @brief  大津法 (OTSU) 求最佳二值化阈值
 * @param  tmImage  输入灰度图像 [STEP_IMAGE_Y][STEP_IMAGE_X]
 * @return 最佳阈值 (0~255)
 * @note   原 image.c 中 GetOSTU 的直接移植，参数维度已适配 80×60
 */
static short Step_GetOSTU(unsigned char tmImage[STEP_IMAGE_Y][STEP_IMAGE_X])
{
    signed short i, j;
    unsigned long Amount = 0;
    unsigned long PixelBack = 0;
    unsigned long PixelshortegralBack = 0;
    unsigned long Pixelshortegral = 0;
    signed long PixelshortegralFore = 0;
    signed long PixelFore = 0;
    float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma;
    signed short MinValue, MaxValue;
    signed short Threshold = 0;
    unsigned char HistoGram[256] = {0};

    /* 统计灰度直方图 */
    for (j = 0; j < STEP_IMAGE_Y; j++)
    {
        for (i = 0; i < STEP_IMAGE_X; i++)
        {
            HistoGram[tmImage[j][i]]++;
        }
    }

    /* 获取最小/最大灰度值 */
    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++)
        ;
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--)
        ;

    if (MaxValue == MinValue)
        return MaxValue;
    if (MinValue + 1 == MaxValue)
        return MinValue;

    /* 像素总数 */
    for (j = MinValue; j <= MaxValue; j++)
        Amount += HistoGram[j];

    /* 灰度值总数 */
    Pixelshortegral = 0;
    for (j = MinValue; j <= MaxValue; j++)
    {
        Pixelshortegral += HistoGram[j] * j;
    }

    /* 遍历寻找最大类间方差对应的阈值 */
    SigmaB = -1;
    for (j = MinValue; j < MaxValue; j++)
    {
        PixelBack = PixelBack + HistoGram[j];
        PixelFore = Amount - PixelBack;
        OmegaBack = (float)PixelBack / Amount;
        OmegaFore = (float)PixelFore / Amount;
        PixelshortegralBack += HistoGram[j] * j;
        PixelshortegralFore = Pixelshortegral - PixelshortegralBack;
        MicroBack = (float)PixelshortegralBack / PixelBack;
        MicroFore = (float)PixelshortegralFore / PixelFore;
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);
        if (Sigma > SigmaB)
        {
            SigmaB = Sigma;
            Threshold = j;
        }
    }
    return Threshold;
}

/*===========================================================================
 * 步骤一：图像压缩 移植
 * 来源：image.c 第 633~675 行 Copy_Zip_Image()
 * 变更：变量名加前缀 Step_，目标图像改为 Step_Src_Image
 *===========================================================================*/

/**
 * @brief  从总钻风原始图像 (188×120) 压缩拷贝到处理图像 (80×60)
 * @note   原始 188 列 → 目标 80 列，列采样系数 ≈ 2.35 (188/80)
 *         原始 120 行 → 目标 60 行，行采样系数 = 2 (120/60)
 *         压缩后灰度图存入 Step_Src_Image
 */
static void Step_Copy_Zip_Image(void)
{
    uint8 i, j;

    /* 等待摄像头一帧采集完成 */
    if (mt9v03x_finish_flag == 1)
    {
        for (i = 0; i < STEP_IMAGE_Y; i++)
        {
            for (j = 0; j < STEP_IMAGE_X; j++)
            {
                /* 行采样：每 2 行取 1 行；列采样：按 2.35 比例缩放 */
                Step_Src_Image[i][j] = mt9v03x_image[i * 2][(uint8)(j * 2.35f)];
            }
        }
        /* 清除摄像头完成标志，等待下一帧 */
        mt9v03x_finish_flag = 0;
    }
}

/*===========================================================================
 * 步骤二：综合图像处理（压缩 + OTSU 阈值 + 二值化）
 *
 * 执行顺序严格为：
 *   ① 调用 Step_Copy_Zip_Image()   — 从摄像头获取并压缩图像
 *   ② 调用 Step_GetOSTU()           — 计算最佳二值化阈值
 *   ③ 全局二值化                    — 逐像素比较阈值，生成 Step_Bin_Image
 *===========================================================================*/

/**
 * @brief  台阶检测综合图像处理流水线
 * @note   依次执行：摄像头图像压缩 → OTSU 自适应阈值 → 全局二值化
 *         二值化结果：0 = 黑色(低灰度台阶面)   1 = 白色(高灰度台阶面)
 */
short otsu_threshold;
void Step_Process_Image(void)
{
    uint8 i, j;
    //short otsu_threshold;

    /*--- 第①步：图像压缩（188×120 → 80×60） ---*/
    Step_Copy_Zip_Image();

    /*--- 第②步：大津法求自适应阈值 ---*/
    otsu_threshold = Step_GetOSTU(Step_Src_Image);

    /*--- 第③步：全局二值化 ---*/
    for (i = 0; i < STEP_IMAGE_Y; i++)
    {
        for (j = 0; j < STEP_IMAGE_X; j++)
        {
            if (Step_Src_Image[i][j] > otsu_threshold)
            {
                Step_Bin_Image[i][j] = 1; /* 高于阈值 → 白色 */
            }
            else
            {
                Step_Bin_Image[i][j] = 0; /* 低于阈值 → 黑色 */
            }
        }
    }
}

/*===========================================================================
 * 步骤三：台阶特征识别与跳跃触发状态机
 *
 * 逻辑：
 *   1. 扫描 SCAN_LINE_1/2/3 三行，统计白色像素个数
 *   2. 白色像素数 > STEP_WHITE_THRESHOLD → 认为当前为"白台阶"
 *      白色像素数 ≤ STEP_WHITE_THRESHOLD → 认为当前为"黑台阶"
 *   3. 16ms 消抖：需连续满足条件才更新 flag_change
 *   4. 检测 flag_change 边沿跳变 → 触发 flag_jump = 1
 *
 * 调用要求：每 STEP_CALL_PERIOD_MS (默认2ms) 调用一次
 *===========================================================================*/

void Step_Detect_And_Jump(void)
{
    uint8 i, j;
    uint8 line_idx;
    uint16 white_cnt = 0; /* 三行白色像素总数 */

    /* 上次的 flag_change 值，用于边沿检测 */
    static uint8 last_flag_change = 0;

    /* 消抖计数器 */
    static uint16 white_stable_cnt = 0; /* 连续满足"白台阶"条件的次数 */
    static uint16 black_stable_cnt = 0; /* 连续满足"黑台阶"条件的次数 */

    /*===================================================================
     * 第①步：扫描三行，统计白色像素
     *===================================================================*/
    const uint8 scan_lines[3] = {SCAN_LINE_1 + 1, SCAN_LINE_1, SCAN_LINE_1 - 1};

    for (i = 0; i < 3; i++)
    {
        line_idx = scan_lines[i];
        for (j = 0; j < STEP_IMAGE_X; j++)
        {
            if (Step_Bin_Image[line_idx][j] == 1)
            {
                white_cnt++;
            }
        }
    }

    /*===================================================================
     * 第②步：根据白色像素数判定当前台阶颜色
     *===================================================================*/
    if (white_cnt > STEP_WHITE_THRESHOLD_MAX)
    {
        /* 当前帧检测为"白台阶" */
        white_stable_cnt++;
        black_stable_cnt = 0; /* 白台阶条件下，黑色计数器清零 */
    }
    else if(white_cnt < STEP_WHITE_THRESHOLD_MIN)
    {
        /* 当前帧检测为"黑台阶" */
        black_stable_cnt++;
        white_stable_cnt = 0; /* 黑台阶条件下，白色计数器清零 */
    }

    /*===================================================================
     * 第③步：16ms 消抖 —— 仅当持续稳定才更新 flag_change
     *===================================================================*/
    if (white_stable_cnt >= STEP_DEBOUNCE_CNT_MAX)
    {
        /* 白台阶状态已稳定 16ms 以上，确认更新标志 */
        Step_flag_change = 1;
        white_stable_cnt = STEP_DEBOUNCE_CNT_MAX; /* 防止无限累加溢出 */
    }
    else if (black_stable_cnt >= STEP_DEBOUNCE_CNT_MAX)
    {
        /* 黑台阶状态已稳定 16ms 以上，确认更新标志 */
        Step_flag_change = 0;
        black_stable_cnt = STEP_DEBOUNCE_CNT_MAX; /* 防止无限累加溢出 */
    }
    /* 否则 flag_change 保持不变（消抖中） */

    /*===================================================================
     * 第④步：边沿检测 —— flag_change 跳变时触发跳跃
     *===================================================================*/
    if (Step_flag_change != last_flag_change)
    {
        /* 发生了边沿跳变（0→1 黑转白台阶  或 1→0 白转黑台阶） */
        flag_jump = 1; /* 触发跳跃动作 */
        last_flag_change = Step_flag_change;
    }
}