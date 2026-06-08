/*********************************************************************************************************************
 * CYT4BB ˮֿ˻ GPS  -  ʵ㹹ģʵ
 *
 * ļ: gps_waypoint.c
 * ģ: M1 ʵ㹹
 * : ʵڴCRUD + Flash д
 *
 * ʵ:
 *   T02: ڴ CRUD (gps_wp_add, gps_wp_clear, gps_wp_current, gps_wp_next, gps_wp_advance, ...)
 *   T03: Flash д (gps_wp_save_to_flash, gps_wp_load_from_flash, double↔uint32 )
 *
 * Flash :
 *   ҳ 50 (GPS_WP_FLASH_DATA_PAGE): ʵ, ÿʵ 2 double = 4 uint32
 *   ҳ 51 (GPS_WP_FLASH_META_PAGE): Ԫ, ħ + count + current_index + У
 *   д˳: ȳдҳ 50, дҳ 51 (ϵͳԪУʧ)
 *   ˳: Уҳ 51, ٶҳ 50
 *
 *  API ʹ:
 *   flash_buffer_clear()              - 
 *   flash_write_page_from_buffer(sector, page, len)  - д
 *   flash_read_page_to_buffer(sector, page, len)     - 
 *   flash_union_buffer[]              - ȫ uint32 
 ********************************************************************************************************************/

#include "gps_waypoint.h"
#include "zf_driver_flash.h"

//====================================================ȫֱ====================================================

gps_waypoint_set_t gps_wp_set = {0};

//====================================================˽к====================================================

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

//====================================================T02: ڴ CRUD====================================================

/*--------------------------------------------------------------------------------------------------------------------
 * ʼ:  Flash ʵ
 * ǰ: Flash ѳʼ
 * : ɹ valid=1, ʧ valid=0
 *--------------------------------------------------------------------------------------------------------------------*/
void gps_wp_init(void)
{
    if (!gps_wp_load_from_flash())
    {
        gps_wp_set.valid = 0;
    }
}

/*--------------------------------------------------------------------------------------------------------------------
 * ʵ
 * ǰ: count < GPS_WP_MAX_COUNT, γЧ
 * : 1=ɹ, 0=ʧ (ѳ/γǷ)
 *--------------------------------------------------------------------------------------------------------------------*/
uint8 gps_wp_add(double lat, double lng)
{
    if (gps_wp_set.count >= GPS_WP_MAX_COUNT)
        return 0;

    /* γȼ: γ [-90, 90],  [-180, 180] */
    if (lat < -90.0 || lat > 90.0 || lng < -180.0 || lng > 180.0)
        return 0;

    gps_wp_set.waypoints[gps_wp_set.count].lat = lat;
    gps_wp_set.waypoints[gps_wp_set.count].lng = lng;
    gps_wp_set.count++;
    return 1;
}

/*--------------------------------------------------------------------------------------------------------------------
 * ʵ
 * : count=0, current_index=0, valid=0
 *--------------------------------------------------------------------------------------------------------------------*/
void gps_wp_clear(void)
{
    gps_wp_set.count = 0;
    gps_wp_set.current_index = 0;
    gps_wp_set.valid = 0;
}

/*--------------------------------------------------------------------------------------------------------------------
 * ȡǰĿʵָ
 * : valid==1  current_index < count ʱָ, 򷵻 NULL
 *--------------------------------------------------------------------------------------------------------------------*/
gps_waypoint_t* gps_wp_current(void)
{
    if (!gps_wp_set.valid || gps_wp_set.current_index >= gps_wp_set.count)
        return (gps_waypoint_t *)0;
    return &gps_wp_set.waypoints[gps_wp_set.current_index];
}

/*--------------------------------------------------------------------------------------------------------------------
 * ȡһʵָ
 * : current_index+1 < count ʱָ, 򷵻 NULL
 *--------------------------------------------------------------------------------------------------------------------*/
gps_waypoint_t* gps_wp_next(void)
{
    if (gps_wp_set.current_index + 1 >= gps_wp_set.count)
        return (gps_waypoint_t *)0;
    return &gps_wp_set.waypoints[gps_wp_set.current_index + 1];
}

/*--------------------------------------------------------------------------------------------------------------------
 * лһʵ
 * : 1=лɹ, 0=Ѿһʵ
 *--------------------------------------------------------------------------------------------------------------------*/
uint8 gps_wp_advance(void)
{
    if (gps_wp_set.current_index + 1 >= gps_wp_set.count)
        return 0;
    gps_wp_set.current_index++;
    return 1;
}

/*--------------------------------------------------------------------------------------------------------------------
 * ȡʵ
 *--------------------------------------------------------------------------------------------------------------------*/
uint8 gps_wp_get_count(void)
{
    return gps_wp_set.count;
}

/*--------------------------------------------------------------------------------------------------------------------
 * ȡǰʵ
 *--------------------------------------------------------------------------------------------------------------------*/
uint8 gps_wp_get_current_index(void)
{
    return gps_wp_set.current_index;
}

//====================================================T03: Flash д====================================================

/*--------------------------------------------------------------------------------------------------------------------
 * ʵ浽 Flash
 * ǰ: ڴʵ
 * : 1=ɹ, 0=ʧ (count==0)
 *
 * д˳:
 *   1. ҳ 50 (ʵ)
 *   2. ҳ 51 (Ԫ: ħ + count + current_index + У)
 * ϵͳ: ҳ 50 дϵ, ҳ 51 ħУʧ, ᳢Լ
 *--------------------------------------------------------------------------------------------------------------------*/
uint8 gps_wp_save_to_flash(void)
{
    uint8 i;
    uint32 hi, lo;
    uint32 checksum;

    if (gps_wp_set.count == 0)
        return 0;

    /* --- дҳ 50: ʵ --- */
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

    /* --- дҳ 51: Ԫ --- */
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
 *  Flash ʵ (˽к,  gps_wp_init )
 * ǰ: Flash ѳʼ
 * : 1=ɹ, 0=ʧ (ħУ/Уʧ)
 *
 * ˳:
 *   1. ȶҳ 51 (Ԫ), Уħ + У
 *   2. Уͨٶҳ 50 (ʵ)
 *--------------------------------------------------------------------------------------------------------------------*/
static uint8 gps_wp_load_from_flash(void)
{
    uint8 i;
    uint8 count;
    uint8 current_index;
    uint32 checksum;

    /* --- ҳ 51: Ԫ --- */
    flash_read_page_to_buffer(0, GPS_WP_FLASH_META_PAGE, FLASH_PAGE_LENGTH);

    /* ħУ */
    if (flash_union_buffer[0].uint8_type != GPS_WP_MAGIC)
    {
        flash_buffer_clear();
        return 0;
    }

    count         = flash_union_buffer[1].uint8_type;
    current_index = flash_union_buffer[2].uint8_type;

    /* Уͼ */
    checksum = (uint32)count ^ (uint32)current_index;
    if (flash_union_buffer[3].uint32_type != checksum)
    {
        flash_buffer_clear();
        return 0;
    }

    /* Խ */
    if (count > GPS_WP_MAX_COUNT || current_index >= count)
    {
        flash_buffer_clear();
        return 0;
    }

    flash_buffer_clear();

    /* --- ҳ 50: ʵ --- */
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
