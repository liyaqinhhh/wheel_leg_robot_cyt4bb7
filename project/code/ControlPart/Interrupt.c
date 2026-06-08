/**
 * @file    Interrupt.c
 * @brief   ���Ȼ����˶༶��ʱ�жϷ���ʵ��
 *
 * �жϵ��ȼܹ���Ƶ���ɶ�ʱ��Ӳ����Ƶ��������
 *   1ms  �� IMUƫ�����ۻ���Ȧ�����٣�Dirchange��������վ����ʱ
 *   2ms  �� ��Ծ���ơ�����ɨ�衢AI�������ݴ��������������ݶ�ȡ��
 *          ���ٶȻ�PID���ڻ�����Ӧ��죩
 *   4ms  �� �������˲����ǶȻ�PID���⻷����ת��PIDģʽ�л�
 *          ��turn_mode 0~7��֧�����/����/�Ӿ�/GPS/�ߵ�/ԭ����ת��
 *   8ms  �� ���ȸ߶��л�����(Single_Control)�����ƽ�����(servo_balance)
 *   16ms �� �ٶȻ�PID����������������㣩���ߵ�ʵʱ�������
 *   40ms �� �����߼�ⳬʱ������ƫ������Ư����
 *
 * Created on: 2024��2��
 *      Author: LateRain
 */
#include "zf_common_headfile.h"
#include "Interrupt.h"
#include "PID.h"
#include "servo.h"
#include "menu.h"
#include "imu660.h"
#include "image.h"
#include "small_driver_uart_control.h"
#include "kalman.h"
#include "Math_Advanced.h"
#include "Init.h"
#include "ins_interface.h"
#include "Ins.h"
#include "AI_Pid_Tuner.h"

#include "Ins.h" // 高度系统的yaw_ins等

#include "gps_nav.h"

/* 全局控制目标平衡结构体实例 */
Center_struct Yao;

/* ---- �����Ƕ�У׼���� ----
 * ���̣��ϵ��ɼ�CALIBRATE_SAMPLES�������ĸ����ǣ�
 * ȡƽ����Ϊ���ƫ�ƣ������Ƕȼ�ȥ��ƫ�ơ�
 */
volatile uint8_t calibrate_state = 0;  /* У׼״̬��0=δ��ʼ, 1=�ɼ���, 2=��� */
volatile float calibrate_offset = 0;   /* У׼�õ��ĽǶ�ƫ�ƣ��ȣ� */
volatile uint16_t calibrate_count = 0; /* �ɼ����� */
volatile float calibrate_sum = 0;      /* �Ƕ��ۼӺ� */

/* ---- ADC���ص�ѹ ---- */
uint16 adc0;
float Battery_voltage;

/* ---- ת�������ģʽ ----
 * steer_control_mode: 0=�Ƕȿ���(��ɷ���), 1=PWM����(���ٶȻ�)
 * turn_mode:          0=�ر�, 1=���PIDת��, 2=����ת��,
 *                     3=ƫ���Ƕȱջ���ֱ��, 4=�Ӿ�ת��,
 *                     5=GPSת��, 6=ԭ����ת(Spin3), 7=�ߵ�ת��
 * fuzzy_mode:         0=�ر�ģ��(ʹ�ù̶�KP), 1=����ģ��(��̬KP��Χ)
 */
uint8 steer_control_mode = 0;
uint8 turn_mode = 7;
uint8 fuzzy_mode = 0;

/* ---- �˵������ ---- */
uint8 menu_open = 1;  /* 0=�رղ˵���Flash(��֤���Բ�����дFlash),
                         1=�򿪲˵�, 2=ֻ�򿪶�ȡ���򿪲˵� */
uint8 flag_yawan = 1; /* ƫ������(yawan)ʹ�ܣ�0=�ر�, 1=���� */
uint8 flag_stop = 1;  /* ֹͣ��־��1=ֹͣ(���������λ), 0=�������� */
uint8 ins_open = 1;   /* �ߵ�ת�򿪹أ�0=�ر�, 1=���� */
uint8 ins_getdata = 0;

/* ---- ���������־ ---- */
uint16 a11111 = 0;     /* ����վ����ʱ����1ms�жϵ����� */
uint16 a2222 = 0;      /* ������ʱ����������Ծ������ */
bool ff = 0;           /* �����м��־ */
uint16 dis_tof_mm = 0; /* TOF���봫�������� */

uint16 time_flag = 0;   /* ��ʱ��־��Ԥ���� */
uint16_t flag_open = 0; /* ������־��Ԥ���� */
int16_t flag_main = 0;  /* ��״̬��־ */
uint16_t flag_text = 0; /* �ı���ʾ��־ */

/* ---- ƫ���Ǳ��� ----
 * angle_Z: �����ۼ�ƫ���ǣ��ɳ�����180�㣩�����ڶ�Ȧ��ת�ȳ�����
 *          ���㹫ʽ��angle_Z = 360 * Dirchange + yaw
 *          Dirchange ��ÿ�ο����180��߽�ʱ��1�������ۼ�Ȧ����
 */
volatile float angle_Z = 0;

void control_main(void)
{
    small_driver_get_speed();

    if (motor_value.receive_right_speed_data < -3500 
        || motor_value.receive_right_speed_data > 3500 
        ||motor_value.receive_left_speed_data < -3500 
        || motor_value.receive_left_speed_data > 3500
        || ins_getdata)
    {
        //            if(!flag_jump_stop)
        flag_main = 1;
        flag_stop = 1;
        Yao.Outp_Gyro_Pitch = 0;
        Yao.Outp_Angle_Pitch = 0;
        Yao.Outp_Speed_Pitch = 0;
        //            motor_value.receive_left_speed_data = 0;
        //            motor_value.receive_right_speed_data = 0;
        small_driver_set_duty(0, 0);
    }

    if (flag_main)
    {
        flag_stop = 1;
        Yao.Outp_Gyro_Pitch = 0;
        Yao.Outp_Angle_Pitch = 0;
        Yao.Outp_Speed_Pitch = 0;
        //            motor_value.receive_left_speed_data = 0;
        //            motor_value.receive_right_speed_data = 0;
        small_driver_set_duty(0, 0);
    }
    else
    {
        if (ins_open == 0 || menu_mode == 1)
        {
            // small_driver_set_duty((int16)(-(Yao.Outp_Gyro_Pitch)),     // ���ַ���ռ�ձ�
            //                       (int16)(-Yao.Outp_Gyro_Pitch )); // ���ַ���ռ�ձ�
            // small_driver_set_duty(0, 0);
            
            small_driver_set_duty((int16)(-(Yao.Outp_Gyro_Pitch - Yao.Outp_Gyro_Yaw)),  // ���ַ���ռ�ձ�
                                  (int16)(-(Yao.Outp_Gyro_Pitch + Yao.Outp_Gyro_Yaw))); // ���ַ���ռ�ձ�
            // small_driver_set_duty(500,-500);
        }
        else
        {
            
            small_driver_set_duty((int16)(-(Yao.Outp_Gyro_Pitch - Yao.Outp_Gyro_Yaw)),  // ���ַ���ռ�ձ�
                                  (int16)(-(Yao.Outp_Gyro_Pitch + Yao.Outp_Gyro_Yaw))); // ���ַ���ռ�ձ�
        }
    }
}
/**
 * @brief   1ms�жϷ�����
 *
 * ִ������
 *   1. TOF���봫������ȡ���˵�ģʽ�£�
 *   2. IMUƫ���ǻ����ۻ���gyro_z * 0.001 �� ����yaw��
 *   3. ƫ���ǹ�һ����[-180��, 180��]
 *   4. ��߽�Ȧ����⣨Dirchange����/�ݼ���
 *   5. �����ۼ�ƫ���� angle_Z ����
 *   6. ����վ����ʱ�� a11111 ����
 */
void Interrupt_1ms(void)
{

    if (menu_mode)
    {
        dis_tof_mm = tof_dl1b_get_mm();
        // ips200_show_uint( 0, 30*8, dis_tof_mm, 5 );
    }
    // EKF_UpData();
    // imu660ra.eulerAngle.pitch = euler_angle.roll - imu660ra.offset_angle.pitch;
    // imu660ra.eulerAngle.roll  = euler_angle.pitch  - imu660ra.offset_angle.roll;

    /* ƫ���ǻ��֣�gyro_z(��/��) * 0.001�� = ÿ���ڽǶ����� */
    imu660ra.eulerAngle.yaw += imu660ra.data_Raw.gyro_z * 0.001;

    // Buzzer_Control();

    /* ƫ���ǹ�һ���� [-180��, 180��] ��Χ */
    if (imu660ra.eulerAngle.yaw > 180)
        imu660ra.eulerAngle.yaw -= 360;
    if (imu660ra.eulerAngle.yaw < -180)
        imu660ra.eulerAngle.yaw += 360;

    /* ��߽�Ȧ�����
     *   ��+180���䵽-180��������<-350���� ��תһȦ��Dirchange++
     *   ��-180���䵽+180��������>350���� ��תһȦ��Dirchange--
     */
    if ((imu660ra.eulerAngle.yaw - imu660ra.eulerAngle.last_yaw) < -350)
        imu660ra.eulerAngle.Dirchange++;
    else if ((imu660ra.eulerAngle.yaw - imu660ra.eulerAngle.last_yaw) > 350)
        imu660ra.eulerAngle.Dirchange--;

    /* ���������ۼ�ƫ���ǣ����ܡ�180�����ƣ� */
    angle_Z = 360 * imu660ra.eulerAngle.Dirchange + imu660ra.eulerAngle.yaw;
    imu660ra.eulerAngle.last_yaw = imu660ra.eulerAngle.yaw;

    // if(imu660ra.eulerAngle.pitch > 0)
    //     imu660ra.eulerAngle.pitch -= 180;
    // else if(imu660ra.eulerAngle.pitch < 0)
    //     imu660ra.eulerAngle.pitch += 180;

    /* ����վ����ʱ����flag_Single=1ʱ�������������� */
    // a11111++;

    // control_main();

    if (flag_Single)
    {
        a11111++;
    }
    else
        a11111 = 0;

    /* ����Ϊ��ע�͵��Զ�90��ת����Դ��� ---- */
    // if (time_flag == 0)
    //  {
    //      Count++;
    //      if (Count >= 100)
    //      {
    //          Target_Yaw += 90;
    //          time_flag = 1;
    //     }

    // }

    // if(key_detect(KEY_2, KEY_SHORT_PRESS))
    //
    //     ff = ~ff;
    // if(ff)
    //     a2222++;
    // else
    // {
    //     a2222 = 0;
    //     Yao.Target_Speed = 0;
    //     Deviation_Value = 0;
    // }
    //
    // if(a2222 >= 3000)
    // {
    //     flag_jump=1;
    //     ff=0;
    //     Deviation_Value = -0.7;
    //     Yao.Target_Speed = 300;
    // }
    // else if(3000 < a2222 && a2222 <= 6000)
    // {
    //     Deviation_Value = 0.7;
    // }
    //// else if(6000 < a2222 && a2222 <= 9000)
    //// {
    ////     Deviation_Value = -0.3;
    //// }
    // else if (a2222 > 6000)
    //     a2222 = 0;
}

float integer = 0;     /* ������Y���ۼ�ֵ�����ڱ궨�� */
uint32 num_t = 0;      /* ������Y��������� */
float integer1 = 0;    /* ������X���ۼ�ֵ�����ڱ궨�� */
uint32 num_t1 = 0;     /* ������X��������� */
volatile float y1 = 0; /* �������ٶȵ�ͨ�˲�ֵ�����ڽ��ٶȻ��� */
float ddddd = 0;       /* Ԥ������ */

/**
 * @brief   2ms�жϷ�����
 *
 * ִ������
 *   1. ��Ծ���ƣ�flag_jump=1ʱÿ���ڵ��� jump_control() �ƽ�״̬��
 *   2. AI���Σ�flag_ai_open=1ʱ�������ڽ��յ�PID����
 *   3. �˵���menu_open=1ʱ���в˵��߼����������
 *   4. ����KEY_2��3�룩��������Ծ��a2222>=3000ʱflag_jump��ת��
 *   5. IMU���ݶ�ȡ��date_handle��
 *   6. �����Ƕ�У׼����ע�ͣ�Ԥ����
 *   7. �������ٶȵ�ͨ�˲� + ���ٶȻ�PID���ڻ���
 *   8. ƫ�����ٶȻ�PID���ڻ���
 */
void Interrupt_2ms(void)
{
    /* 1. ��Ծ���ƣ�ÿ�����ƽ���Ծ״̬�� */
    if (flag_jump)
    {
        time_j++;
        jump_control();
    }
    else
        time_j = 0;

    key_scanner();

    /* 2. AI���Σ��������ڽ��յ�PID�������� */
    if (flag_ai_open)
    {
        AI_Pid_Tuner_ProcessRx();
    }

    /* 3. �˵����� */
    // if (menu_open == 1)
    //     //menu();
    // else
    menu_mode = 1;

    // if (menu_mode && key_detect(KEY_1, KEY_SHORT_PRESS))
    //    flag_track = 1;

    // servo_set_angle(LF, 180);
    // servo_set_angle(RF, 180);
    // servo_set_angle(LB, 0);
    // servo_set_angle(RB, 0);
    // servo_set_angle(LF, 265.7f);
    // servo_set_angle(RF, 265.7f);
    // servo_set_angle(LB, 42.2f);
    // servo_set_angle(RB, 42.2f);

    /* 4. ����KEY_2������Ծ��Լ3��=3000*2ms�� */
    // if (menu_mode && key_detect(KEY_2, KEY_SHORT_PRESS))
    // {
    //     ff = ~ff;
    // }
    // if (ff)
    //     a2222++;
    // else
    // {
    //     a2222 = 0;
    // }
    // if (a2222 >= 3000)
    // {
    //     flag_jump = ~flag_jump;
    //     ff = 0;
    // }

    // pwm_set_duty(ATOM0_CH0_P21_2, 1500);
    // pwm_set_duty(ATOM0_CH1_P21_3, 1500);
    // pwm_set_duty(ATOM0_CH2_P21_4, 1500);
    // pwm_set_duty(ATOM0_CH3_P21_5, 1500);

    /* 5. IMU���ݶ�ȡ����̬���� */
    // get_eulerAngle();
    date_handle();

    /* 6. �����Ƕ�У׼����ע�ͣ�Ԥ�����ܣ� */
    /*if (calibrate_state == 1)
    {
        calibrate_sum += imu660ra.eulerAngle.pitch;
        calibrate_count++;

        if (calibrate_count >= CALIBRATE_SAMPLES)
        {
            calibrate_offset = calibrate_sum / calibrate_count;
            calibrate_state = 2;  // ??????
        }
         ips200_show_float( 30 , 80 , calibrate_offset , 3 , 3 );
    }*/

    /* 7. ����ģʽ��IMU���ֱ�־����Z�������ǻ��֣�������ƫ���ԣ� */
    if (menu_mode)
    {
        // if(num_t >= 3000)
        // date_handle();
        if (IMU_JF_Flag)
        {
            Z_Yaw += imu660ra.data_Raw.gyro_z / 500;
        }
        else
        {
            Z_Yaw = 0;
        }

        // else
        //     num_t++;
    }

    // imu660ra_get_gyro();

    // if(num_t <= 10000)
    // {
    //     num_t++;
    //     integer += (float)imu660ra_gyro_y-2.5f;
    // }
    // ips200_show_float( 0, 0*8, integer, 5,5 );
    // ips200_show_float( 0, 1*8, (integer/num_t), 5,5 );
    // if(num_t1 <= 10000)
    // {
    //     num_t1++;
    //     integer1 += (float)imu660ra_gyro_x-1.62f;
    // }
    // ips200_show_float( 0, 3*8, integer1, 5,5 );
    // ips200_show_float( 0, 4*8, (integer1/num_t1), 5,5 );

    /* 8. ����PID���ٶȻ������ڲ㣩----
     * ������������X���ͨ�˲�(0.15��+0.85��) �� ���ٶ�PID �� �޷���8000
     * ƫ����������Z�� �� ���ٶ�PID �� �޷���8000
     * ע�⣺�������ٶȻ���SetPoint���ԽǶȻ�����ĸ�ֵ��-Yao.Outp_Angle_Pitch����
     *       ��������ƥ�䴫�������������������
     */
    y1 = 0.15f * ((float)-imu660ra_gyro_x) + 0.85f * y1;
    Yao.Outp_Gyro_Pitch = -limit(Cascade_gyro_Pitch(&PID_all.Pid_Gyro_Pitch, erect_Gyro_Pitch, y1, -Yao.Outp_Angle_Pitch), 8000.0f);
    Yao.Outp_Gyro_Yaw = limit(Cascade_gyro_Yaw(&PID_all.Pid_Gyro_Yaw, erect_Gyro_Yaw, imu660rb_gyro_z, Yao.Outp_turn), 8000.0f);
    // if(fabs(Yao.Outp_Gyro_Pitch) < 80) Yao.Outp_Gyro_Pitch = 0;
}

/* ---- 4ms�ж���ȫ�ֱ��� ---- */
volatile float aa1 = 0;                                  /* �����ǵ�ͨ�˲�ֵ�����ڽǶȻ��� */
volatile float aa2 = 0;                                  /* ת��ǵ�ͨ�˲�ֵ������ת��PID�� */
volatile float dd = 0;                                   /* ƫ��ƫ���ͨ�˲�ֵ */
float temp_erect_turn[4];                                /* ת��PID��ʱ������ģ��ģʽ��̬����KP�� */
float k11 = 0;                                           /* �����ٲ���ϵ��1 */
float k22 = 0;                                           /* �����ٲ���ϵ��2 */
float kp_roll = 0.9;                                     /* ����KPϵ�� */
float Target_Yaw = 0;                                    /* Ŀ��ƫ���ǣ�turn_mode=3��ֱ��ģʽ�� */
float V_trans = 0;                                       /* �����ٶȣ�Ԥ���� */
uint8 TCount_falg_4ms = 0;                               /* 4ms����ʹ�ܱ�־ */
uint16 TCount_4ms = 0;                                   /* 4ms���ڼ����� */
float dt = 0.004f;                                       /* �������ڣ�4ms=0.004�룩 */
float desired_yaw = 0.0f;                                /* ����ƫ���ǣ��Ӿ�/GPSģʽ�� */
float raw_vision_yaw = 0.0f;                             /* �Ӿ�ԭʼƫ���ǣ���׶����ǰ�� */
static float steer_vision_cmd_lpf_alpha = 0.35f;         /* �Ӿ�ָ���ͨ�˲�ϵ�� */
static float steer_vision_cmd_lpf = 0.0f;                /* �Ӿ�ָ���ͨ�˲�ֵ */
volatile float steer_vision_target_yaw_deg = 0.0f;       /* �Ӿ�Ŀ��ƫ���ǣ��ȣ� */
volatile float steer_vision_cone_avoid_delta_deg = 0.0f; /* �Ӿ���׶�Ƕ�ƫ�ƣ��ȣ� */

/* ---- GPS转向数据已迁移至乒乓缓冲 gps_steer_pp ---- */

/**
 * @brief   4ms�жϷ�����
 *
 * ִ������
 *   1. �������˲�������̬��imu963ra_kalman_filter_update��
 *   2. ��̬��ƫ�Ʋ�������ȥoffset_angle��
 *   3. �Ƕ�����������|pitch|<0.4��ʱ���㣩
 *   4. �����ǶȻ�PID���⻷������ �����Ϊ���ٶȻ���Ŀ��ֵ
 *   5. ת��PIDģʽ�л���ִ�У�turn_mode 0~7��
 *
 * ת��ģʽ��⣺
 *   mode 0: �ر�ת��
 *   mode 1: ���PIDת��PID_turn_seekfree��ʹ��erect_turn������
 *   mode 2: ����ת��Cascade_angle_Yaw��֧��fuzzy_mode��̬KP��
 *   mode 3: ƫ���Ƕȱջ���ֱ�ߣ�Cascade_angle_Yaw_2��Target_Yaw�̶���
 *   mode 4: �Ӿ�ת��Cascade_angle_Yaw_3��desired_yaw=�Ӿ�Ŀ��+��׶ƫ�ƣ���ͨ�˲���
 *   mode 5: GPSת��Cascade_angle_Yaw_4��desired_yaw=GPS��λ��+IMUƫ�ƣ�
 *   mode 6: ԭ����תSpin3��Cascade_angle_Yaw_2��angle_ZΪĿ�꣬��λ���Զ��л�mode2��
 *   mode 7: �ߵ�ת��Cascade_angle_Yaw_2��Ŀ��=�ߵ�����
 */
void Interrupt_4ms(void)
{
    // V_trans = (float)(motor_value.receive_left_speed_data-motor_value.receive_right_speed_data);

    /* 1. �������˲����ں�����������ٶȼƣ�����roll/pitch */
    imu963ra_kalman_filter_update(&imu);

    // imu963ra_menc15a_kalman_filter_Update(&vel_kf, 0, imu.ay_linear);

    /* 2. ƫ�Ʋ�������ȥ�ϵ�궨�����ƫ�� */
    imu.roll -= imu660ra.offset_angle.roll;
    imu.pitch -= imu660ra.offset_angle.pitch;
    imu660ra.eulerAngle.roll = imu.pitch; /* ע�⣺��������rollӳ�䵽eulerAngle.pitch */
    imu660ra.eulerAngle.pitch = imu.roll; /* ��������pitchӳ�䵽eulerAngle.roll������ϵת���� */

    /* 3. �Ƕ�������|pitch|<0.4��ʱǿ�����㣬����΢���� */
    if (imu660ra.eulerAngle.pitch < 0.4 && imu660ra.eulerAngle.pitch > -0.4)
        imu660ra.eulerAngle.pitch = 0;

    /* 4ms������ */
    if (TCount_falg_4ms)
        TCount_4ms++;
    else
        TCount_4ms = 0;

    /* 4. �����ǶȻ�PID���⻷��----
     * ���룺�����ǵ�ͨ�˲�ֵ aa1���˲�ϵ��0.1��+0.9�ɣ�
     * �趨ֵ��Yao.Outp_Speed_Pitch�������ٶȻ������
     * ����޷�����12000
     */
    /*float pitch_corrected = imu660ra.eulerAngle.pitch;
    if (calibrate_state == 2)  // ��?????????
    {
        pitch_corrected -= calibrate_offset;
    }*/
    aa1 = 0.1f * imu660ra.eulerAngle.pitch + 0.9f * aa1;
    if (steer_control_mode == 0)
    {
        Yao.Outp_Angle_Pitch = Cascade_angle_Pitch(&PID_all.Pid_Angle_Pitch, erect_Angle_Pitch, aa1, 0);
        Yao.Outp_Angle_Pitch = -limit(Yao.Outp_Angle_Pitch, 12000.0f);
    }
    else
    {
        Yao.Outp_Angle_Pitch = Cascade_angle_Pitch(&PID_all.Pid_Angle_Pitch, erect_Angle_Pitch, aa1, 0);
        Yao.Outp_Angle_Pitch = -limit(Yao.Outp_Angle_Pitch, 12000.0f);
    }

    /* ƫ��ƫ���ͨ�˲� */
    dd = 0.1f * Deviation_Value + 0.9f * dd;

    /* 5. ת����� ---- */
    if (turn_mode == 1)
    {
        /* ģʽ1�����PIDת�򣨺�������ǰ���� */
        Yao.Outp_turn = PID_turn_seekfree(&PID_all.Pid_turn, erect_turn, imu660ra.data_Raw.gyro_z, Deviation_Value * 10 + 0.2f);
        Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);
    }
    else if (turn_mode == 2)
    {
        /* ģʽ2������ת��
         * fuzzy_mode=0: ʹ�ù̶�KP(erect_Angle_Yaw)
         * fuzzy_mode=1: ʹ��ģ����̬KP(Get_P����ƫ���С����)
         */
        if (fuzzy_mode == 0)
        {
            temp_erect_turn[0] = erect_Angle_Yaw[0];
            temp_erect_turn[1] = erect_Angle_Yaw[1];
            temp_erect_turn[2] = erect_Angle_Yaw[2];
            temp_erect_turn[3] = erect_Angle_Yaw[3];
        }
        else
        {
            temp_erect_turn[0] = Get_P(Y_Meet, Deviation_Value);
            temp_erect_turn[1] = erect_Angle_Yaw[1];
            temp_erect_turn[2] = erect_Angle_Yaw[2];
            temp_erect_turn[3] = erect_Angle_Yaw[3];
        }

        // if(flag_Single_HighState == 1)
        //     temp_erect_turn[0] = 100;
        Yao.Outp_turn = Cascade_angle_Yaw(&PID_all.Pid_turn, temp_erect_turn, Deviation_Value * 10 /*+ kp_roll * stab_roll*/, 0);
        Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);
    }
    else if (turn_mode == 3)
    {
        /* ģʽ3��ƫ���Ƕȱջ���ֱ�ߣ�Target_Yaw���̶ֹ����� */
        Yao.Outp_turn = Cascade_angle_Yaw_2(&PID_all.Pid_Angle_Yaw, erect_Angle_Yaw_2, imu660ra.eulerAngle.yaw, Target_Yaw);
        Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);
        // printf("Target_Yaw: %.2f, Current_Yaw: %.2f\n", Target_Yaw, imu660ra.eulerAngle.yaw);
    }
    else if (turn_mode == 4)
    {
        /* ģʽ4���Ӿ�ת��
         * Ŀ��Ƕ� = �Ӿ�Ŀ�� + ��׶ƫ����
         * ������ͨ�˲�(a=0.35)ƽ��Ŀ��ֵ�����ٱ�׶˲��Ķ���
         */
        raw_vision_yaw = steer_vision_target_yaw_deg + steer_vision_cone_avoid_delta_deg;
        raw_vision_yaw = steer_wrap_deg180(raw_vision_yaw);

        /* ��ͨ�˲�ƽ��Ŀ��ƫ���� */
        steer_vision_cmd_lpf = (1.0f - steer_vision_cmd_lpf_alpha) * steer_vision_cmd_lpf + steer_vision_cmd_lpf_alpha * raw_vision_yaw;
        desired_yaw = steer_vision_cmd_lpf;
        Yao.Outp_turn = Cascade_angle_Yaw_3(&PID_all.Pid_turn1, erect_Angle_Yaw_3, imu660ra.eulerAngle.yaw, desired_yaw);
        Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);
        // Target_Yaw = steer_target_yaw_deg;  // mirror to existing debug variable
    }
    else if (turn_mode == 5)
    {
        /* 模式5：GPS转向（乒乓缓冲读取）
         * 从 gps_steer_pp 的 read_idx 侧读取数据
         * 目标角度 = GPS方位角 + IMU偏航偏移量
         */
        const gps_steer_output_t *gps_steer = GPS_STEER_READ();
        desired_yaw = gps_steer->target_bearing_deg + gps_steer->imu_yaw_offset_deg;
        Yao.Outp_turn = Cascade_angle_Yaw_4(&PID_all.Pid_turn2, erect_Angle_Yaw_3, imu660ra.eulerAngle.yaw, desired_yaw);
        Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);
    }
    else if (turn_mode == 6)
    {
        /* ģʽ6��ԭ����ת��Spin3��
         * ʹ�������ۼƽǶ� angle_Z�����ܡ�180�����ƣ���Ϊ��ǰ�Ƕȣ�
         * Ŀ��Ƕ� spin3_target_angle����ʼ�ǡ�1080��=3Ȧ����
         *
         * ��λ�������Ƕ���� < spin3_angle_ok_deg ��
         *           ����Z����ٶ� < spin3_gyro_ok_dps
         * ���� spin3_hold_ticks ���ں� �� �л�mode2��������ǰƫ��ΪTarget_Yaw
         */
        Yao.Outp_turn = Cascade_angle_Yaw_2(&PID_all.Pid_Angle_Yaw, erect_Angle_Yaw_2, angle_Z, spin3_target_angle);
        Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);

        /* ��鵽λ�������Ƕ�+���ٶ�ͬʱ���㣩 */
        if (func_abs(spin3_target_angle - angle_Z) < spin3_angle_ok_deg &&
            func_abs((float)imu660rb_gyro_z) < spin3_gyro_ok_dps)
        {
            spin3_hold_cnt++;
        }
        else
        {
            spin3_hold_cnt = 0;
        }

        /* ��λ�����㹻���ں��˳���ת */
        if (spin3_hold_cnt >= spin3_hold_ticks)
        {
            spin3_active = 0;
            spin3_hold_cnt = 0;
            Yao.Outp_turn = 0;
            turn_mode = 2;                        /* �лش���ת��ģʽ */
            Target_Yaw = imu660ra.eulerAngle.yaw; /* ������ǰƫ��ΪĿ�� */
        }
    }
    else if (turn_mode == 7)
    {
        /* ģʽ7���ߵ�ת��
         * ins_open=1ʱʹ�ùߵ������ yaw_ins����һ����[-180,180]����ΪĿ��
         */
        if (ins_open)
        {
            float yaw_ins_deg = (float)yaw_ins;
            if (yaw_ins_deg > 180.0f)
                yaw_ins_deg -= 360.0f;
            if (yaw_ins_deg < -180.0f)
                yaw_ins_deg += 360.0f;
            /* ȷ�� setpoint �ڵ�ǰ yaw �� ��180�� ��Χ�ڣ�ȡ���ת��·�� */
            {
                float diff = yaw_ins_deg - imu660ra.eulerAngle.yaw;
                if (diff > 180.0f)
                    yaw_ins_deg -= 360.0f;
                else if (diff < -180.0f)
                    yaw_ins_deg += 360.0f;
            }
            // aa2 = 0.95f * yaw_ins_deg + 0.05f * aa2;  /* �ߵ�Ŀ��ǵ�ͨ�˲� */
            Yao.Outp_turn = Cascade_angle_Yaw_2(&PID_all.Pid_Angle_Yaw, erect_Angle_Yaw_2, imu660ra.eulerAngle.yaw, yaw_ins_deg);
            Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);
        }
        else
        {
            Yao.Outp_turn = 0;
        }
    }
    else
    {
        /* mode 0 ��������ת�����Ϊ�� */
        Yao.Outp_turn = 0;
    }
}

/* ---- 8ms�ж���ȫ�ֱ��� ---- */
int16 recordL = 0; /* ���ּ�¼ֵ��Ԥ���� */
int16 recordR = 0; /* ���ּ�¼ֵ��Ԥ���� */
float vv2 = 0;     /* ���ٲ��˲�ֵ��Ԥ���� */

/**
 * @brief   8ms�жϷ�����
 *
 * ִ������
 *   1. ���ȸ߶��л����ƣ�Single_Control����flag_Single=1ʱ����
 *   2. ���ƽ����ƣ�servo_balance����steer_control_mode=1ʱ����
 *   3. ������⣨KEY_1����flag_track��
 *
 * ע�⣺ԭ�ٶȻ�PID��Cascade_speed_Pitch����Ǩ�Ƶ�16ms�жϡ�
 *       Adapt_Terrain() ����Ҳ��ע�ͣ���ǰ����mainѭ���е��ã���
 */
void Interrupt_8ms(void)
{
    /*
     // ��ص�ѹ
     adc0 = adc_mean_filter_convert( ADC0_CH6_A6 , 5 );
     Battery_voltage = adc0 / 114.8936;

     if(menu_mode)
     {
     Yao.Encoder_Left  = motor_value.receive_right_speed_data;
     Yao.Encoder_Right = -motor_value.receive_left_speed_data;
     }
     else
     {
         Yao.Encoder_Left = 0;
         Yao.Encoder_Right = 0;
     }
     Yao.Outp_Speed_Pitch = -Cascade_speed_Pitch(&PID_all.Pid_Speed_Pitch, erect_Speed_Pitch, (float)(Yao.Encoder_Left+Yao.Encoder_Right)/2, (float)Yao.Target_Speed);
     Yao.Outp_Speed_Pitch = limit( Yao.Outp_Speed_Pitch, 30.0f );
     */
    // vv2 = 0.05f * (float)(motor_value.receive_left_speed_data-motor_value.receive_right_speed_data) + 0.95 * vv2;
    // Yao.Outp_Speed_Pitch = -Cascade_speed_Pitch(&PID_all.Pid_Speed_Pitch, erect_Speed_Pitch, vv2, (float)Yao.Target_Speed);
    // Yao.Outp_Speed_Pitch = limit( Yao.Outp_Speed_Pitch, 100.0f );

    // ��Ծ���Ʋ���
    // if(flag_jump)
    // {
    //     time_j++;
    //     jump_control();
    // }
    // else
    //     time_j = 0;

    /**************************************************************/

    /* 1. ����վ���߶��л����� */
    if (flag_Single)
    {
        Single_Control();
    }

    // if (menu_mode == 1 /*&& flag_jump == 0 && Element_State != Jump_State */ && steer_control_mode == 0)
    // Adapt_Terrain();
    // else

    /* 2. ���ƽ����ƣ�PWMģʽ��ֱ��ʹ�����ٲ */

    servo_balance();

    /******************************************************************** */

    /* 3. ������⣺�̰�KEY_1����Ѳ��ģʽ */
    /*if (menu_open == 1)
       // menu();
    else
        menu_mode = 1;*/

    // if (menu_mode && key_detect(KEY_1, KEY_SHORT_PRESS))
    //     flag_track = 1;

    // if(Element_State == Jump_State)
    // {
    //     if(flag_jump == 0)
    //     servo_set_angle(RF, 210);servo_set_angle(RB, 0);
    //     servo_set_angle(LF, 210);servo_set_angle(LB, 0);
    // }

    // small_driver_set_duty(0, 0);
    // processImage();
}

/* ---- 16ms�ж���ȫ�ֱ��� ---- */
volatile float aa11 = 0;        /* ���ٲ��ͨ�˲�ֵ�������ٶȻ����룩 */
volatile float speed_MOTOR = 0; /* ƽ�����٣����ڹߵ�������£� */

/**
 * @brief   16ms�жϷ�����
 *
 * ִ������
 *   1. �ߵ�ʵʱ������£�get_realtime_coordinate����ins_open=1ʱ����
 *   2. ƽ�����ټ��㣺�������ٲ�/2
 *   3. ���ٵ�ͨ�˲���ϵ��0.1��+0.9�ɣ�
 *   4. �ٶȻ�PID��������������㣩��
 *      ����=�˲����ƽ�����٣��趨ֵ=0��ƽ��ʱĿ���ٶ�Ϊ0��������޷���100
 *
 * ע�⣺�ٶȻ���� Yao.Outp_Speed_Pitch ��Ϊ�ǶȻ���SetPoint��
 *       �γ�"�ٶȻ����ǶȻ������ٶȻ�"�����������ơ�
 */
void Interrupt_16ms(void)
{

    // ��ص�ѹ
    // adc0 = adc_mean_filter_convert( ADC0_CH6_A6 , 5 );
    // Battery_voltage = adc0 / 114.8936;

    // if(menu_mode)
    // {
    //     if(func_abs(Yao.Encoder_Left-motor_value.receive_right_speed_data) <= 100)
    //         Yao.Encoder_Left  = 0.5f * motor_value.receive_right_speed_data + 0.5f * Yao.Encoder_Left;
    //     if(func_abs(Yao.Encoder_Right-motor_value.receive_left_speed_data) <= 100)
    //         Yao.Encoder_Right = -(0.5f * motor_value.receive_left_speed_data + 0.5f * Yao.Encoder_Right);

    //     Yao.Encoder_Left  = motor_value.receive_right_speed_data;
    //     if(motor_value.receive_left_speed_data < 0)
    //         Yao.Encoder_Right = -motor_value.receive_left_speed_data+22;
    //     else if(motor_value.receive_left_speed_data > 0)
    //         Yao.Encoder_Right = -motor_value.receive_left_speed_data-23;
    //     else
    //         Yao.Encoder_Right = -motor_value.receive_left_speed_data;

    //     Yao.Encoder_Left  = motor_value.receive_right_speed_data;
    //     Yao.Encoder_Right = -motor_value.receive_left_speed_data;
    // }
    // else
    // {
    //     Yao.Encoder_Left = 0;
    //     Yao.Encoder_Right = 0;
    // }

    /* 1. �ߵ�ʵʱ������£�16ms���ڣ���ǰƫ���ǣ� */
    if (ins_open)
        get_realtime_coordinate(speed_MOTOR, 0.016, imu660ra.eulerAngle.yaw);

    /* 2. ƽ�����ټ��㣨��������ƥ�䷽�����ַ�ת�� */
    speed_MOTOR = (float)(-motor_value.receive_left_speed_data + motor_value.receive_right_speed_data) / 2.0f;

    /* 3. ���ٵ�ͨ�˲� */
    aa11 = 0.1f * (((float)(-motor_value.receive_left_speed_data + motor_value.receive_right_speed_data)) / 2.0f) + 0.9f * aa11;

    /* 4. �ٶȻ�PID������㣩
     * �趨ֵ=0��ƽ��ʱĿ���ٶ�Ϊ0����ֹƽ�⣩
     * ����޷���100����ֹ�ٶȻ���������½ǶȻ�����
     */
    Yao.Outp_Speed_Pitch = -Cascade_speed_Pitch(&PID_all.Pid_Speed_Pitch, erect_Speed_Pitch, aa11, 0);
    Yao.Outp_Speed_Pitch = limit(Yao.Outp_Speed_Pitch, 100.0f);
}

/* ---- 40ms�ж��� ---- */
extern uint8 Zebra_Count_Flag; /* �����߼�����־���ⲿ���壩 */
uint16 TCount_40ms = 0;        /* 40ms���ڼ����������ڰ����߳�ʱ��⣩ */

/* ---- ң�ⷢ�Ϳ��� ---- */
uint8 telemetry_enable = 1;     /* ң��ʹ�ܣ�0=�ر�, 1=������ͨ�����ߴ��ڷ��͵������ݣ� */
uint8 ins_telemetry_enable = 1; /* �ߵ�ң��ʹ�ܣ�0=�ر�, 1=���� */

/**
 * @brief   40ms�жϷ�����
 *
 * ִ������
 *   1. �����߳�ʱ���
 *   2. ƫ������Ư����
 *   3. ң�����ݽ��淢�ͣ�ÿ80ms�л� $T ��̬ң�� / $I �ߵ�ң�⣩
 *      $T��ʽ: tick,pitch,roll,yaw,gx,gy,gz,outp_turn,... (20�ֶ�)
 *      $I��ʽ: x,y,ins_mode,dis_ins,yaw_ins,n,target,flag_save (8�ֶ�)
 *   4. �ߵ�¼��˲�䷢�� $W ����֡
 */
void Interrupt_40ms(void)
{
    /* 1. �����߳�ʱ��⣺4����δ��⵽�°������򴥷���־ */
    if (Zebra_Count_Flag == 0)
    {
        Zebra_Flag = 0;
        TCount_40ms++;
    }
    else
        TCount_40ms = 0;

    if (TCount_40ms >= 100)
    {
        Zebra_Flag = 1;
        Zebra_Count_Flag = 1;
    }

    // yaokong_data_deal();

    /* 2. ƫ������Ư���� */
    imu660ra.eulerAngle.yaw += 0.0001;

    /* 3. ң�����ݽ��淢�ͣ�
     *    telemetry_toggle=0 �� $T ��̬ң��
     *    telemetry_toggle=1 �� $I �ߵ�ң��
     *    ��ռһ�������ÿ��ÿ80ms����һ��
     */
    if (telemetry_enable || ins_telemetry_enable)
    {
        static uint8 telemetry_toggle = 0;
        telemetry_toggle++;
        if (telemetry_toggle >= 2)
        {
            telemetry_toggle = 0;

            if (telemetry_enable)
            {
                /* $T ��̬ң��֡ */
                char telemetry_buf[128];
                int len = sprintf(telemetry_buf,
                                  "$T,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%.1f,%d,%.2f,%.1f,%.1f,%d,%d,%.2f,%.2f,%.1f\r\n",
                                  (int)TCount_40ms,
                                  imu660ra.eulerAngle.pitch,
                                  imu660ra.eulerAngle.roll,
                                  imu660ra.eulerAngle.yaw,
                                  imu660ra.data_Raw.gyro_x,
                                  imu660ra.data_Raw.gyro_y,
                                  imu660ra.data_Raw.gyro_z,
                                  (int)Yao.Outp_turn,
                                  (int)Yao.Outp_Gyro_Pitch,
                                  Target_Yaw,
                                  (int)turn_mode,
                                  Deviation_Value,
                                  imu.pitch,
                                  imu.roll,
                                  (int)motor_value.receive_left_speed_data,
                                  (int)motor_value.receive_right_speed_data,
                                  imu.ax_linear,
                                  imu.ay_linear,
                                  angle_Z);
                if (len > 0 && len < (int)sizeof(telemetry_buf))
                {
                    wireless_uart_send_string(telemetry_buf);
                }
            }
            else if (ins_telemetry_enable)
            {
                /* $I �ߵ�ң��֡��$T�ر�ʱ��ռ���ʹ����� */
                char ins_buf[100];
                int len = sprintf(ins_buf,
                                  "$I,%.1f,%.1f,%d,%.1f,%.1f,%d,%d,%d\r\n",
                                  cod_realtime.x,
                                  cod_realtime.y,
                                  (int)ins_mode,
                                  dis_ins,
                                  yaw_ins,
                                  (int)n,
                                  (int)target,
                                  (int)flag_save);
                if (len > 0 && len < (int)sizeof(ins_buf))
                {
                    wireless_uart_send_string(ins_buf);
                }
            }
        }
        else if (telemetry_enable && ins_telemetry_enable)
        {
            /* toggle=1 ʱ���� $I �ߵ�ң��֡���� $T ���棩 */
            char ins_buf[100];
            int len = sprintf(ins_buf,
                              "$I,%.1f,%.1f,%d,%.1f,%.1f,%d,%d,%d\r\n",
                              cod_realtime.x,
                              cod_realtime.y,
                              (int)ins_mode,
                              dis_ins,
                              yaw_ins,
                              (int)n,
                              (int)target,
                              (int)flag_save);
            if (len > 0 && len < (int)sizeof(ins_buf))
            {
                wireless_uart_send_string(ins_buf);
            }
        }
    }

    /* 4. �ߵ�¼��˲�� �� ���� $W ����֡ */
    if (ins_telemetry_enable && ins_getdata)
    {
        char wp_buf[50];
        int wp_len = sprintf(wp_buf,
                             "$W,%d,%.1f,%.1f,%d\r\n",
                             (int)(n - 1),                /* ���º������� */
                             cod_saved[(uint8)(n - 1)].x, /* ����X */
                             cod_saved[(uint8)(n - 1)].y, /* ����Y */
                             (int)n                       /* ��ǰ�������� */
        );
        if (wp_len > 0 && wp_len < (int)sizeof(wp_buf))
        {
            wireless_uart_send_string(wp_buf);
        }
        ins_getdata = 0; /* ���η��ͣ������־ */
    }
}
