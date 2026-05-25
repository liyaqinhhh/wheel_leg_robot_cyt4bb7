/*
 * Ins.h
 *
 *  Created on: 2024年6月6日
 *      Author: LateRain
 */

#ifndef CODE_CENTER_INS_H_
#define CODE_CENTER_INS_H_

#define ANGLE_TO_RAD(x)     ( (x) * PI / 180.0 )                                // 角度转换为弧度
#define RAD_TO_ANGLE(x)     ( (x) * 180.0 / PI )                                // 弧度转换为角度
#define PI                  ( 3.1415926535898 )

typedef struct {
    double x;
    double y;
} Coordinates;

void get_realtime_coordinate( int speed , float time ,float yaw );
void get_target( double x1,double y1 , double x2,double y2 );
void ins_navigation(void);

extern Coordinates cod_realtime;
extern Coordinates cod_saved[30];
extern Coordinates cod_target[30];
extern double dis_ins, yaw_ins;
extern uint8 ins_mode;
extern uint8 n;
extern uint8 target;

#endif /* CODE_CENTER_INS_H_ */
