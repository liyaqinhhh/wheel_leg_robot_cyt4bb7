/*
 * Interrupt.h
 */

#ifndef CODE_CONTROLPART_INTERRUPT_H_
#define CODE_CONTROLPART_INTERRUPT_H_

extern uint16 a11111;

typedef struct
{
    float Outp_Gyro_Pitch;
    float Outp_Angle_Pitch;
    float Outp_Speed_Pitch;

    float Outp_Gyro_Roll;
    float Outp_Angle_Roll;

    float Outp_Gyro_Yaw;
    float Outp_Angle_Yaw;

    float Outp_turn;

    int16 Encoder_Left;
    int16 Encoder_Right;

    int Target_Speed;
    float Target_height;

} Center_struct;

extern Center_struct Yao;

extern uint8 steer_control_mode;
extern uint8 turn_mode;
extern float Battery_voltage;
extern uint8 flag_stop;
extern uint8 menu_open;
extern uint8 flag_yawan;
extern float kp_roll;
extern float k11;
extern float k22;
extern float Target_Yaw;
extern uint16 dis_tof_mm;
extern uint8 fuzzy_mode;
extern uint16 TCount_4ms;
extern uint16 TCount_40ms;
extern uint8 TCount_falg_4ms;
extern volatile float steer_gps_target_bearing_deg; // deg, [0, 360)
extern volatile float steer_gps_distance_to_wp_m;   // meter

extern float desired_yaw;
// IMU yaw and geographic heading alignment offset:
// imu_yaw = geo_heading + steer_gps_to_imu_yaw_offset_deg
extern volatile float steer_gps_to_imu_yaw_offset_deg;

extern volatile float angle_Z;

// 开机角度校准相关
extern volatile uint8_t calibrate_state;   // 0:未开始 1:采集中 2:完成
extern volatile float calibrate_offset;    // 校准得到的角度偏移
extern volatile uint16_t calibrate_count;  // 采集计数
extern volatile float calibrate_sum;       // 角度累加和
#define CALIBRATE_SAMPLES 500              // 采集500个样本（1秒@2ms）

void Interrupt_1ms(void);
void Interrupt_2ms(void);
void Interrupt_4ms(void);
void Interrupt_8ms(void);
void Interrupt_16ms(void);
void Interrupt_40ms(void);

#endif /* CODE_CONTROLPART_INTERRUPT_H_ */
