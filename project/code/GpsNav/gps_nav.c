/*********************************************************************************************************************
 * CYT4BB ˮֿ˻ GPS  -  ״̬ģʵ
 *
 * ļ: gps_nav.c
 * ģ: M2 ״̬
 * : ״̬ +  + ƹ + GPS ǳ
 *
 * ʵ:
 *   T08: ״̬ (IDLE→CALIBRATING→NAVIGATING→ARRIVED→COMPLETE)
 *   T09: λǼ/ + ˲
 *   T10: ʵж (ֵ)
 *   T11: Ʋ + ƹύ + GPS ǳ
 *
 * ʱ:
 *   gps_nav_proc()  while  10Hz 
 *   ƹд buf[write_idx] ɺԭӽ
 *   ISR  buf[read_idx] ȡ 250Hz
 *
 * GPS ǳ:
 *   ʹ֡ʽ (10Hz × 5s = 50 ֡)
 *   ǳڼ䲻ύƹ ISR ȡ
 *   ֡ʽ system_getval_us() ƽ̨
 ********************************************************************************************************************/

#include "gps_nav.h"
#include "gps_waypoint.h"
#include "gps_calibration.h"
#include "zf_device_gnss.h"
#include "zf_device_imu660ra.h"

/* turn_mode  Interrupt.h  */
extern uint8 turn_mode;

//====================================================ȫֱ====================================================

gps_steer_pp_t  gps_steer_pp;
uint8           gps_nav_state = GPS_NAV_IDLE;

//====================================================˽б====================================================

static float    lpf_bearing       = GPS_NAV_FIRST_FRAME_MAGIC;  // ˲ֵ (ֵ=δʼ)
static uint8    is_near_waypoint  = 0;                          // ʵٽ־ (ֵ)
static uint8    gps_loss_frames   = 0;                          // GPS ǳ֡

//====================================================˽к====================================================

static void     gps_steer_commit(void);
static gps_steer_output_t* gps_steer_write_buf(void);
static float    angle_lpf_circular(float old_val, float new_val, float alpha);

/*--------------------------------------------------------------------------------------------------------------------
 * ƹύ: ԭӽ write_idx ↔ read_idx
 * ʱ: д buf[write_idx] ֮
 * : ISR ´ζȡ¿
 * ԭ:  volatile uint8 д ARM ϸԭ
 *--------------------------------------------------------------------------------------------------------------------*/
static void gps_steer_commit(void)
{
    uint8 old_write = gps_steer_pp.write_idx;
    uint8 old_read  = gps_steer_pp.read_idx;
    gps_steer_pp.read_idx  = old_write;   //  ISR µ
    gps_steer_pp.write_idx = old_read;    //  д
}

/*--------------------------------------------------------------------------------------------------------------------
 * ȡдָ (main while ʹ)
 *--------------------------------------------------------------------------------------------------------------------*/
static gps_steer_output_t* gps_steer_write_buf(void)
{
    return &gps_steer_pp.buf[gps_steer_pp.write_idx];
}

/*--------------------------------------------------------------------------------------------------------------------
 * ˲:  ±180 ߽
 * : 179 → -179 ʵֻ 2  358
 *--------------------------------------------------------------------------------------------------------------------*/
static float angle_lpf_circular(float old_val, float new_val, float alpha)
{
    float diff = new_val - old_val;
    if (diff > 180.0f)  diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;
    return old_val + alpha * diff;
}

//====================================================T08: ״̬====================================================

/*--------------------------------------------------------------------------------------------------------------------
 * ʼ: ʵ + ƹ + ״̬
 * ǰ: Flash  + gnss + IMU ѳʼ
 *--------------------------------------------------------------------------------------------------------------------*/
void gps_nav_init(void)
{
    /* ʵ */
    gps_wp_init();

    /* ƹʼ: ˫, write=0, read=1 */
    gps_steer_pp.buf[0].target_bearing_deg = 0.0f;
    gps_steer_pp.buf[0].distance_to_wp_m   = 0.0f;
    gps_steer_pp.buf[0].imu_yaw_offset_deg = 0.0f;
    gps_steer_pp.buf[1].target_bearing_deg = 0.0f;
    gps_steer_pp.buf[1].distance_to_wp_m   = 0.0f;
    gps_steer_pp.buf[1].imu_yaw_offset_deg = 0.0f;
    gps_steer_pp.write_idx = 0;
    gps_steer_pp.read_idx  = 1;

    /* ״̬ */
    gps_nav_state = GPS_NAV_IDLE;
    lpf_bearing      = GPS_NAV_FIRST_FRAME_MAGIC;
    is_near_waypoint = 0;
    gps_loss_frames  = 0;
}

/*--------------------------------------------------------------------------------------------------------------------
 * : 10Hz  while 
 * ǰ: gnss_flag==1  gnss_data_parse() ѵ
 *--------------------------------------------------------------------------------------------------------------------*/
void gps_nav_proc(void)
{
    /* === T11: GPS ǳ === */
    if (gnss.state != 1)
    {
        if (gps_nav_state == GPS_NAV_NAVIGATING || gps_nav_state == GPS_NAV_ARRIVED)
        {
            gps_loss_frames++;
            if (gps_loss_frames >= GPS_NAV_SIGNAL_LOSS_FRAMES)
            {
                /* ǳ 5s ͣ */
                gps_nav_stop();
                return;
            }
        }
        /* ǳڼ䲻ύƹ ISR ȡ */
        return;
    }

    /* GPS ָ → ü */
    gps_loss_frames = 0;

    /* === T11: turn_mode Լ === */
    if (turn_mode != 5 && gps_nav_state != GPS_NAV_IDLE)
    {
        gps_nav_state = GPS_NAV_IDLE;
        /* ƹȫ: ISR  turn_mode!=5 ʱ GPS ת֧
         *  read_idx ָľɱȫ (ֵ)
         */
    }

    /* === ״ַ̬ === */
    switch (gps_nav_state)
    {
    case GPS_NAV_IDLE:
        /* ȴ gps_nav_start()  turn_mode==5 */
        break;

    case GPS_NAV_CALIBRATING:
    {
        /* У׼:  IMU yaw  GPS λǼƫ */
        if (gps_cal_startpoint())
        {
            /* У׼ɹ */
            lpf_bearing      = GPS_NAV_FIRST_FRAME_MAGIC;  // ˲
            is_near_waypoint = 0;
            gps_nav_state    = GPS_NAV_NAVIGATING;
        }
        else
        {
            /* У׼ʧ: ʵ < 2 ƫƳ */
            gps_nav_state = GPS_NAV_IDLE;
        }
        break;
    }

    case GPS_NAV_NAVIGATING:
    {
        /* === T09:  === */
        gps_waypoint_t *wp = gps_wp_current();
        if (wp == (gps_waypoint_t *)0)
        {
            gps_nav_stop();
            break;
        }

        /* λǼ [0, 360)  (׼) */
        float raw_bearing = (float)get_two_points_azimuth(
            gnss.latitude, gnss.longitude, wp->lat, wp->lng);
        float distance = (float)get_two_points_distance(
            gnss.latitude, gnss.longitude, wp->lat, wp->lng);

        /* ˲: ֱ֡Ӹ (ֵ) */
        if (lpf_bearing == GPS_NAV_FIRST_FRAME_MAGIC)
        {
            lpf_bearing = raw_bearing;
        }
        else
        {
            lpf_bearing = angle_lpf_circular(lpf_bearing, raw_bearing, GPS_NAV_LPF_ALPHA);
        }

        /* Ϊ 0 ʱֱжϵ */
        if (distance < 0.01f)
        {
            gps_nav_state = GPS_NAV_ARRIVED;
            break;
        }

        /* === T11: Ʋ === */
        gps_cal_drift_correction();

        /* дƹд */
        gps_steer_output_t *out = gps_steer_write_buf();
        out->target_bearing_deg = lpf_bearing;
        out->distance_to_wp_m   = distance;
        out->imu_yaw_offset_deg = gps_cal_get_offset();

        /* ԭӽύ */
        gps_steer_commit();

        /* === T16: debug output (uncomment for wireless serial) === */
        // printf("NAV: wp=%d, bear=%.1f, dist=%.1f, yaw=%.1f, des=%.1f, off=%.1f\r\n",
        //        gps_wp_get_current_index(), lpf_bearing, distance,
        //        imu660ra.eulerAngle.yaw,
        //        lpf_bearing + gps_cal_get_offset(),
        //        gps_cal_get_offset());

        /* === T10: ʵж (ֵ) === */
        if (!is_near_waypoint)
        {
            /* δ״̬:  < ֵ → жϵ */
            if (distance < GPS_NAV_ARRIVE_ENTER_M)
            {
                is_near_waypoint = 1;
                gps_nav_state    = GPS_NAV_ARRIVED;
            }
        }
        else
        {
            /* ѵ״̬:  > 뿪ֵ →  */
            if (distance > GPS_NAV_ARRIVE_LEAVE_M)
            {
                is_near_waypoint = 0;
            }
        }
        break;
    }

    case GPS_NAV_ARRIVED:
    {
        /* лһʵ */
        if (gps_wp_advance())
        {
            /* һʵ */
            is_near_waypoint = 0;
            lpf_bearing      = GPS_NAV_FIRST_FRAME_MAGIC;  // ˲
            gps_nav_state    = GPS_NAV_NAVIGATING;
        }
        else
        {
            /* ʵ */
            gps_nav_state = GPS_NAV_COMPLETE;
        }
        break;
    }

    case GPS_NAV_COMPLETE:
    {
        /* ͣ: turn_mode=0 */
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
 * ʼ: У׼
 * ǰ: ʵ >= 2
 *--------------------------------------------------------------------------------------------------------------------*/
void gps_nav_start(void)
{
    if (!gps_wp_set.valid || gps_wp_set.count < 2)
        return;

    gps_nav_state = GPS_NAV_CALIBRATING;
    turn_mode     = 5;
}

/*--------------------------------------------------------------------------------------------------------------------
 * ֹͣ: IDLE + turn_mode=0
 *--------------------------------------------------------------------------------------------------------------------*/
void gps_nav_stop(void)
{
    gps_nav_state = GPS_NAV_IDLE;
    turn_mode     = 0;

    /* д ( ISR ȡɱȫ) */
    gps_steer_output_t *out = gps_steer_write_buf();
    out->target_bearing_deg = 0.0f;
    out->distance_to_wp_m   = 0.0f;
    out->imu_yaw_offset_deg = 0.0f;
}

/*--------------------------------------------------------------------------------------------------------------------
 * ȡǰ״̬
 *--------------------------------------------------------------------------------------------------------------------*/
uint8 gps_nav_get_state(void)
{
    return gps_nav_state;
}

/*--------------------------------------------------------------------------------------------------------------------
 * лĿʵ
 * : 1=ɹ, 0=ʧ (Խ)
 *--------------------------------------------------------------------------------------------------------------------*/
uint8 gps_nav_set_wp_index(uint8 idx)
{
    if (idx >= gps_wp_set.count)
        return 0;

    gps_wp_set.current_index = idx;
    is_near_waypoint = 0;
    lpf_bearing      = GPS_NAV_FIRST_FRAME_MAGIC;  // ˲
    return 1;
}
