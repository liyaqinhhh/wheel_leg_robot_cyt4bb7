# -*- coding: utf-8 -*-
import csv

path = r'd:\Yao_Port_4bb7_01 - backups\Yao_Port_4bb7_01\Seekfree_CYT4BB_Opensource_Library_\docs\telemetry_turn_test_20260622_210221.csv'
with open(path, 'r') as f:
    rows = list(csv.DictReader(f))
n = len(rows)

p = [float(r['pitch']) for r in rows]
rl = [float(r['roll']) for r in rows]
y = [float(r['yaw']) for r in rows]
gz = [float(r['gyro_z']) for r in rows]
ot = [int(float(r['outp_turn'])) for r in rows]
ogp = [int(float(r['outp_gyro_pitch'])) for r in rows]
ty = [float(r['target_yaw']) for r in rows]
ml = [int(float(r['motor_l'])) for r in rows]
mr = [int(float(r['motor_r'])) for r in rows]

print('n=%d' % n)
print('pitch: %.1f ~ %.1f' % (min(p), max(p)))
print('roll: %.1f ~ %.1f' % (min(rl), max(rl)))
print('yaw: %.1f ~ %.1f' % (min(y), max(y)))
print('gz_peak: %.0f' % max(abs(v) for v in gz))
print('outp_turn: %d ~ %d' % (min(ot), max(ot)))
print('outp_gyro_pitch: %d ~ %d' % (min(ogp), max(ogp)))
print('target_yaw: %.1f ~ %.1f' % (min(ty), max(ty)))

# target_yaw transitions
prev = 9999
print('\ntarget_yaw transitions:')
for i, r in enumerate(rows):
    v = float(r['target_yaw'])
    if abs(v - prev) > 0.001:
        print('  row %d: %.1f' % (i, v))
        prev = v

# Phase analysis by target_yaw groups
phase_starts = [0]
prev_ty = ty[0]
for i in range(1, n):
    if abs(ty[i] - prev_ty) > 0.01:
        phase_starts.append(i)
        prev_ty = ty[i]
phase_starts.append(n)

for k in range(len(phase_starts)-1):
    s, e = phase_starts[k], phase_starts[k+1]
    if e - s < 5:
        continue
    label = 'TY=%.1f' % ty[s]
    print('\n%s [%d:%d] %dpts:' % (label, s, e, e-s))
    print('  pitch %.1f~%.1f roll %.1f~%.1f yaw %.1f~%.1f' % (
        min(p[s:e]), max(p[s:e]), min(rl[s:e]), max(rl[s:e]), min(y[s:e]), max(y[s:e])))
    print('  gz_peak %.0f  ot %d~%d  ogp %d~%d' % (
        max(abs(v) for v in gz[s:e]), min(ot[s:e]), max(ot[s:e]), min(ogp[s:e]), max(ogp[s:e])))
    print('  motor_L %d~%d  motor_R %d~%d' % (min(ml[s:e]), max(ml[s:e]), min(mr[s:e]), max(mr[s:e])))
