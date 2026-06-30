/*********************************************************************************************************************
 * CYT4BB 智能车 GPS 导航 -  航点管理模块
 *
 * 文件: gps_waypoint.c
 * 模块: M1 航点管理
 * 功能: 航点CRUD + Flash 持久化
 *
 * 任务:
 *   T02: 航点 CRUD (gps_wp_add, gps_wp_clear, gps_wp_current, gps_wp_next, gps_wp_advance, ...)
 *   T03: Flash 存储 (gps_wp_save_to_flash, gps_wp_load_from_flash, double<->uint32 转换)
 *
 * Flash 存储布局:
 *   页 50 (GPS_WP_FLASH_DATA_PAGE): 数据页，每个航点占 2 double = 4 uint32
 *   页 51 (GPS_WP_FLASH_META_PAGE): 元数据页，魔数 + count + current_index + 校验和
 *   写入顺序: 先写数据页 50, 再写元数据页 51 (保证原子性)
 *   读取顺序: 先读元数据页 51, 再读数据页 50
 *
 *  Flash API 参考:
 *   flash_buffer_clear()              - 清空缓冲区
 *   flash_write_page_from_buffer(sector, page, len)  - 写入页
 *   flash_read_page_to_buffer(sector, page, len)     - 读取页
 *   flash_union_buffer[]              - 缓冲区，每个元素为 uint32
 ********************************************************************************************************************/

#include "gps_waypoint.h"
#include "zf_driver_flash.h"

//====================================================全局变量====================================================

gps_waypoint_set_t gps_wp_set = {0};

//====================================================私有函数====================================================

static uint8 gps_wp_load_from_flash(void);

static void double_to_two_uint32(double val, uint32 *hi, uint32 *lo)
{
    uint64 raw = *(uint64 *)&val;
    *hi = (uint32)(raw >> 32);
    *lo = (uint32)(raw & 0xFFFFFFFF);
}

static double two_uint32_to_double(uint32 hi, uint32 lo)
{
    uint64 raw = ((uint64)hi << 32) | (uint64)lo;
    return *(double *)&raw;
}

//====================================================T02: 航点 CRUD====================================================

/*--------------------------------------------------------------------------------------------------------------------
 * 功能: 初始化航点系统，从 Flash 加载
 * 前置: Flash 驱动已初始化
 * 返回: 加载成功 valid=1，失败 valid=0
 *--------------------------------------------------------------------------------------------------------------------*/
void gps_wp_init(void)
{
    if (!gps_wp_load_from_flash())
    {
        gps_wp_set.valid = 0;
    }
}

/*--------------------------------------------------------------------------------------------------------------------
 * 功能: 添加航点
 * 前置: count < GPS_WP_MAX_COUNT，坐标合法
 * 返回: 1=成功, 0=失败（满/越界）
 *--------------------------------------------------------------------------------------------------------------------*/
uint8 gps_wp_add(double lat, double lng)
{
    if (gps_wp_set.count >= GPS_WP_MAX_COUNT)
        return 0;

    /* 坐标范围检查: 纬度 [-90, 90], 经度 [-180, 180] */
    if (lat < -90.0 || lat > 90.0 || lng < -180.0 || lng > 180.0)
        return 0;

    gps_wp_set.waypoints[gps_wp_set.count].lat = lat;
    gps_wp_set.waypoints[gps_wp_set.count].lng = lng;
    gps_wp_set.count++;
    return 1;
}

/*--------------------------------------------------------------------------------------------------------------------
 * 功能: 清空所有航点
 * 结果: count=0, current_index=0, valid=0
 *--------------------------------------------------------------------------------------------------------------------*/
void gps_wp_clear(void)
{
    gps_wp_set.count = 0;
    gps_wp_set.current_index = 0;
    gps_wp_set.valid = 0;
}

/*--------------------------------------------------------------------------------------------------------------------
 * 获取当前航点指针
 * 返回: valid==1 且 current_index < count 时返回指针，否则返回 NULL
 *--------------------------------------------------------------------------------------------------------------------*/
gps_waypoint_t* gps_wp_current(void)
{
    if (!gps_wp_set.valid || gps_wp_set.current_index >= gps_wp_set.count)
        return (gps_waypoint_t *)0;
    return &gps_wp_set.waypoints[gps_wp_set.current_index];
}

/*--------------------------------------------------------------------------------------------------------------------
 * 获取下一个航点指针
 * 返回: current_index+1 < count 时返回指针，否则返回 NULL
 *--------------------------------------------------------------------------------------------------------------------*/
gps_waypoint_t* gps_wp_next(void)
{
    if (gps_wp_set.current_index + 1 >= gps_wp_set.count)
        return (gps_waypoint_t *)0;
    return &gps_wp_set.waypoints[gps_wp_set.current_index + 1];
}

/*--------------------------------------------------------------------------------------------------------------------
 * 推进航点索引
 * 返回: 1=推进成功, 0=已是最后一个航点
 *--------------------------------------------------------------------------------------------------------------------*/
uint8 gps_wp_advance(void)
{
    if (gps_wp_set.current_index + 1 >= gps_wp_set.count)
        return 0;
    gps_wp_set.current_index++;
    return 1;
}

/*--------------------------------------------------------------------------------------------------------------------
 * 获取航点数量
 *--------------------------------------------------------------------------------------------------------------------*/
uint8 gps_wp_get_count(void)
{
    return gps_wp_set.count;
}

/*--------------------------------------------------------------------------------------------------------------------
 * 获取当前航点索引
 *--------------------------------------------------------------------------------------------------------------------*/
uint8 gps_wp_get_current_index(void)
{
    return gps_wp_set.current_index;
}

//====================================================T03: Flash 存储====================================================

/*--------------------------------------------------------------------------------------------------------------------
 * 保存航点到 Flash
 * 前置: 航点数量 > 0
 * 返回: 1=成功, 0=失败 (count==0)
 *
 * 顺序:
 *   1. 写页 50 (数据页)
 *   2. 写页 51 (元数据页: 魔数 + count + current_index + 校验和)
 * 注意: 先写页 50 成功后, 再写页 51 标记完成，保证原子性
 *--------------------------------------------------------------------------------------------------------------------*/
uint8 gps_wp_save_to_flash(void)
{
    uint8 i;
    uint32 hi, lo;
    uint32 checksum;

    if (gps_wp_set.count == 0)
        return 0;

    /* --- 写入页 50: 数据页 --- */
    flash_buffer_clear();

    for (i = 0; i < gps_wp_set.count; i++)
    {
        double_to_two_uint32(gps_wp_set.waypoints[i].lat, &hi, &lo);
        flash_union_buffer[4 * i].uint32_type     = hi;
        flash_union_buffer[4 * i + 1].uint32_type = lo;

        double_to_two_uint32(gps_wp_set.waypoints[i].lng, &hi, &lo);
        flash_union_buffer[4 * i + 2].uint32_type = hi;
        flash_union_buffer[4 * i + 3].uint32_type = lo;
    }

    flash_write_page_from_buffer(0, GPS_WP_FLASH_DATA_PAGE, FLASH_PAGE_LENGTH);

    /* --- 写入页 51: 元数据页 --- */
    flash_buffer_clear();

    checksum = (uint32)gps_wp_set.count ^ (uint32)gps_wp_set.current_index;

    flash_union_buffer[0].uint8_type  = GPS_WP_MAGIC;
    flash_union_buffer[1].uint8_type  = gps_wp_set.count;
    flash_union_buffer[2].uint8_type  = gps_wp_set.current_index;
    flash_union_buffer[3].uint32_type = checksum;

    flash_write_page_from_buffer(0, GPS_WP_FLASH_META_PAGE, FLASH_PAGE_LENGTH);

    return 1;
}

/*--------------------------------------------------------------------------------------------------------------------
 *  从 Flash 加载航点 (内部函数，由 gps_wp_init 调用)
 * 前置: Flash 驱动已初始化
 * 返回: 1=成功, 0=失败 (无数据/校验错)
 *
 * 流程:
 *   1. 读取页 51 (元数据)，校验魔数 + 校验和
 *   2. 读取航点数据页 50 (数据页)
 *--------------------------------------------------------------------------------------------------------------------*/
static uint8 gps_wp_load_from_flash(void)
{
    uint8 i;
    uint8 count;
    uint8 current_index;
    uint32 checksum;

    /* --- 读取页 51: 元数据页 --- */
    flash_read_page_to_buffer(0, GPS_WP_FLASH_META_PAGE, FLASH_PAGE_LENGTH);

    /* 魔数校验 */
    if (flash_union_buffer[0].uint8_type != GPS_WP_MAGIC)
    {
        flash_buffer_clear();
        return 0;
    }

    count         = flash_union_buffer[1].uint8_type;
    current_index = flash_union_buffer[2].uint8_type;

    /* 校验和验证 */
    checksum = (uint32)count ^ (uint32)current_index;
    if (flash_union_buffer[3].uint32_type != checksum)
    {
        flash_buffer_clear();
        return 0;
    }

    /* 范围检查 */
    if (count > GPS_WP_MAX_COUNT || current_index >= count)
    {
        flash_buffer_clear();
        return 0;
    }

    flash_buffer_clear();

    /* --- 读取页 50: 数据页 --- */
    flash_read_page_to_buffer(0, GPS_WP_FLASH_DATA_PAGE, FLASH_PAGE_LENGTH);

    for (i = 0; i < count; i++)
    {
        gps_wp_set.waypoints[i].lat = two_uint32_to_double(
            flash_union_buffer[4 * i].uint32_type,
            flash_union_buffer[4 * i + 1].uint32_type);
        gps_wp_set.waypoints[i].lng = two_uint32_to_double(
            flash_union_buffer[4 * i + 2].uint32_type,
            flash_union_buffer[4 * i + 3].uint32_type);
    }

    flash_buffer_clear();

    gps_wp_set.count         = count;
    gps_wp_set.current_index = current_index;
    gps_wp_set.valid         = 1;

    return 1;
}
