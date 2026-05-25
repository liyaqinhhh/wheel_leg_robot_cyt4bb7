/*
 * Ins.c
 *
 *  Created on: 2024年6月6日
 *      Author: LateRain
 */

#include "zf_common_headfile.h"
#include "Ins.h"

Coordinates cod_realtime;
Coordinates cod_saved[30];
Coordinates cod_target[30];

double dis_ins, yaw_ins;
uint8 n = 0;
uint8 target = 1;
float temp_erect_speed_go;
uint8 ins_mode = 0;
bool flag_1;

/*******************************double to uint32***************************************/

extern flash_data_union flash_union_buffer[FLASH_PAGE_LENGTH];

// 将 double 类型的值写入到 flash_data_union 缓存单元中的两个成员中
void writeDoubleToFlash1(double data1, double data2, uint8 z)
{
    // 将第一个 double 数据分成两个 32 位的部分
    uint32_t upperPart1 = (uint32_t)((*(uint64_t *)&data1) >> 32);
    uint32_t lowerPart1 = (uint32_t)((*(uint64_t *)&data1) & 0xFFFFFFFF);

    // 将第二个 double 数据分成两个 32 位的部分
    uint32_t upperPart2 = (uint32_t)((*(uint64_t *)&data2) >> 32);
    uint32_t lowerPart2 = (uint32_t)((*(uint64_t *)&data2) & 0xFFFFFFFF);

    // 写入到 flash_data_union 缓存单元中的两个成员中
    flash_union_buffer[4 * z].uint32_type = upperPart1;
    flash_union_buffer[4 * z + 1].uint32_type = lowerPart1;
    flash_union_buffer[4 * z + 2].uint32_type = upperPart2;
    flash_union_buffer[4 * z + 3].uint32_type = lowerPart2;
}

// 将缓存里的值进行读取
double readFlash_to_double1(bool a, uint8 x)
{
    // 从缓存中读取第一个和第二个值的上下半部分
    uint32_t upperPart1;
    uint32_t lowerPart1;
    if (a == 0) // 读出纬度latitude
    {
        upperPart1 = flash_union_buffer[4 * x].uint32_type;
        lowerPart1 = flash_union_buffer[4 * x + 1].uint32_type;
    }
    else if (a == 1) // 读出经度longitude
    {
        upperPart1 = flash_union_buffer[4 * x + 2].uint32_type;
        lowerPart1 = flash_union_buffer[4 * x + 3].uint32_type;
    }
    // 将两个 32 位的数据重新组合成一个 64 位的数据
    uint64_t combinedData = ((uint64_t)upperPart1 << 32) | lowerPart1;

    // 将组合后的数据转换为 double 类型
    double result = *(double *)&combinedData;

    return result;
}

/*******************************double to uint32***************************************/

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     得到实时坐标
// 参数说明     speed          瞬时速度
// 参数说明     time           变化时间段
// 参数说明     yaw            车头航向角
// 返回参数
// 使用示例     get_realtime_coordinate( speed_MOTOR , 0.008 ,  yaw_gyro );
// 备注信息     yaw为逆时针变大，0~359°范围，切每次上电发车方向应该相同
//-------------------------------------------------------------------------------------------------------------------
void get_realtime_coordinate(int speed, float time, float yaw)
{
    double dx, dy;
    double temp1;

    // 计算出坐标增量
    temp1 = ANGLE_TO_RAD(yaw);
    dx = speed * time * cos(temp1);
    dy = speed * time * sin(temp1);

    cod_realtime.x += dx;
    cod_realtime.y += dy;
}
void get_target(double x1, double y1, double x2, double y2)
{
    double temp1, temp2;
    double dx = x2 - x1;
    double dy = y2 - y1;
    // 计算夹角，距离
    temp1 = atan2(dy, dx);
    temp2 = sqrt(dx * dx + dy * dy);

    // 将角度范围转换为 0 到 2π
    if (temp1 < 0)
        temp1 += 2 * PI;

    dis_ins = temp2;
    yaw_ins = RAD_TO_ANGLE(temp1);
}

void ins_navigation(void)
{
    switch (ins_mode)
    {
    case 0:                                          // 存点模式，用遥控跑一圈或者手推一圈
        if (key_get_state(KEY_3) == KEY_SHORT_PRESS) // 记录坐标点
        {
            key_clear_state(KEY_3);
            cod_saved[n].x = cod_realtime.x;
            cod_saved[n].y = cod_realtime.y;
            n++;
        }
        if (key_get_state(KEY_2) == KEY_SHORT_PRESS) // 存入flash
        {
            key_clear_state(KEY_2);

            if (flash_check(0, 0)) // 有数据，清除
                flash_erase_page(0, 0);
            if (flash_check(0, 10)) // 有数据，清除
                flash_erase_page(0, 10);
            if (!flash_check(0, 0)) // 没有数据
            {
                for (uint8 nn = 0; nn < n; nn++)
                    writeDoubleToFlash1(cod_saved[nn].x, cod_saved[nn].y, nn);
                flash_write_page_from_buffer(0, 0, FLASH_PAGE_LENGTH); // 将目标点坐标写入第0页
                flash_buffer_clear();
            }
            if (!flash_check(0, 10)) // 没有数据
            {
                flash_union_buffer[0].uint8_type = n;
                flash_write_page_from_buffer(0, 10, FLASH_PAGE_LENGTH); // 将点数写入第10页
                flash_buffer_clear();
            }
        }
        if (key_get_state(KEY_1) == KEY_SHORT_PRESS)
        {
            key_clear_state(KEY_1);
            flag_1 = 1;
            ins_mode = 1;
        }
        break;

    case 1:
        if (flag_1) // 上电进入导航模式时，读取一次flash
        {
            flash_read_page_to_buffer(0, 10, FLASH_PAGE_LENGTH); // 读存点数
            n = flash_union_buffer[0].uint8_type;
            flash_buffer_clear();

            for (uint8 nnn = 0; nnn < n; nnn++) // 读坐标
            {
                flash_read_page_to_buffer(0, 0, FLASH_PAGE_LENGTH);
                cod_target[nnn].x = readFlash_to_double1(0, nnn);
                cod_target[nnn].y = readFlash_to_double1(1, nnn);
            }
            flash_buffer_clear();

            //                flash_read_page_to_buffer(0, 2);        //读目标点
            //                target = flash_union_buffer[0].uint8_type;
            //                if( target == 0 )
            //                    target = 1;
            //                flash_buffer_clear();
            flag_1 = 0;
        }
        // 得到目标距离和角度
        get_target(cod_realtime.x, cod_realtime.y, cod_target[target].x, cod_target[target].y);

        if (dis_ins < 20 && dis_ins != 0) // 到达目标点，切换到下一个
        {
            //                speed_flag.flag_1_yw = 1;
            if (target == 0) // 已经回库，倒地
                // Jarvis.dynamic_pitch_angle = 20;
                small_driver_set_duty(0, 0);

            target++;

            if (target == n) // 最后一个点，下一个点为库
                target = 0;
            //                if(flash_check(0, 2))       //有数据，清除
            //                    flash_erase_page(0, 1);
            //                if(!flash_check(0,2))       //没有数据，更新下一个目标点后写入第2页
            //                {
            //                    flash_union_buffer[0].uint8_type = target;
            //                    flash_write_page_from_buffer(0,2);
            //                    flash_buffer_clear();
            //                }
        }

        break;

    default:
        break;
    }
}
