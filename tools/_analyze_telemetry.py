# -*- coding: utf-8 -*-
import csv

path = r'd:\Yao_Port_4bb7_01 - backups\Yao_Port_4bb7_01\Seekfree_CYT4BB_Opensource_Library_\docs\telemetry_turn_test_20260622_204700.csv'
with open(path, 'r') as f:
    rows = list(csv.DictReader(f))
n = len(rows)

pitch = [float(r['pitch']) for r in rows]
roll = [float(r['roll']) for r in rows]
yaw = [float(r['yaw']) for r in rows]
gx = [float(r['gyro_x']) for r in rows]
gy = [float(r['gyro_y']) for r in rows]
gz = [float(r['gyro_z']) for r in rows]
ot = [int(float(r['outp_turn'])) for r in rows]
ogp = [int(float(r['outp_gyro_pitch'])) for r in rows]
yins = [float(r['yaw_ins']) for r in rows]
ml = [int(float(r['motor_l'])) for r in rows]
mr = [int(float(r['motor_r'])) for r in rows]

phases = [
    (0, 429, 'A-静止/初始化'),
    (429, 545, 'B-小角度转弯(逆时针)'),
    (545, 769, 'C-直道加速'),
    (769, 1134, 'D-中速转弯'),
    (1134, 1320, 'E-低速连续转弯'),
    (1320, n, 'F-稳态直行微调'),
]

for s, e, lbl in phases:
    if s >= n:
        break
    e = min(e, n)
    print(f'\n=== {lbl}: rows[{s}:{e}] ({e-s} pts) ===')
    print(f'  pitch      : {min(pitch[s:e]):7.1f} ~ {max(pitch[s:e]):7.1f}')
    print(f'  roll       : {min(roll[s:e]):7.1f} ~ {max(roll[s:e]):7.1f}')
    print(f'  yaw        : {min(yaw[s:e]):7.1f} ~ {max(yaw[s:e]):7.1f}')
    print(f'  gyro_x     : {min(gx[s:e]):7.1f} ~ {max(gx[s:e]):7.1f}')
    print(f'  gyro_y     : {min(gy[s:e]):7.1f} ~ {max(gy[s:e]):7.1f}')
    print(f'  gyro_z     : {min(gz[s:e]):7.1f} ~ {max(gz[s:e]):7.1f}')
    print(f'  gyro_z_abs_max: {max(abs(v) for v in gz[s:e]):7.0f}')
    print(f'  outp_turn  : {min(ot[s:e]):7d} ~ {max(ot[s:e]):7d}')
    print(f'  outp_gyro_pitch: {min(ogp[s:e]):7d} ~ {max(ogp[s:e]):7d}')
    print(f'  yaw_ins    : {min(yins[s:e]):7.1f} ~ {max(yins[s:e]):7.1f}')
    nz_ml = [v for v in ml[s:e] if v != 0]
    nz_mr = [v for v in mr[s:e] if v != 0]
    if nz_ml:
        print(f'  motor_L    : {min(nz_ml):7d} ~ {max(nz_ml):7d}')
        print(f'  motor_R    : {min(nz_mr):7d} ~ {max(nz_mr):7d}')

print(f'\n=== 全量数据 (n={n}) ===')
print(f'  pitch: {min(pitch):.1f} ~ {max(pitch):.1f}')
print(f'  roll : {min(roll):.1f} ~ {max(roll):.1f}')
print(f'  yaw  : {min(yaw):.1f} ~ {max(yaw):.1f}')
print(f'  gyro_x: {min(gx):.1f} ~ {max(gx):.1f}')
print(f'  gyro_y: {min(gy):.1f} ~ {max(gy):.1f}')
print(f'  gyro_z: {min(gz):.1f} ~ {max(gz):.1f}')
print(f'  outp_turn: {min(ot)} ~ {max(ot)}')
print(f'  outp_gyro_pitch: {min(ogp)} ~ {max(ogp)}')
print(f'  yaw_ins: {min(yins):.1f} ~ {max(yins):.1f}')
