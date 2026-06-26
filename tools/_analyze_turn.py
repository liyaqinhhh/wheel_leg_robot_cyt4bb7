# -*- coding: utf-8 -*-
import csv

csv_path = r'd:\Yao_Port_4bb7_01 - backups\Yao_Port_4bb7_01\Seekfree_CYT4BB_Opensource_Library_\docs\telemetry_turn_test_20260621_212234.csv'

rows = []
with open(csv_path, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        rows.append({k: float(v) for k, v in row.items()})

phase1 = [r for r in rows if -2 < r['yaw'] < 5]
phase2 = [r for r in rows if 30 < r['yaw'] < 175 and abs(r['gyro_z']) > 20]
phase3 = [r for r in rows if r['yaw'] > 170]

print('=== 180deg Turn Analysis (turn_mode=7) ===')
print()
print(f'Total frames: {len(rows)}')
print(f'Before turn (yaw -2~5): {len(phase1)} frames')
print(f'During turn (yaw 30~175, |gz|>20): {len(phase2)} frames')
print(f'After turn (yaw>170): {len(phase3)} frames')
print()

if phase1:
    ap1 = sum(r['pitch'] for r in phase1) / len(phase1)
    print(f'Before: avg pitch={ap1:.1f} deg')
if phase2:
    ap2 = sum(r['pitch'] for r in phase2) / len(phase2)
    print(f'During: avg pitch={ap2:.1f}  max={max(r["pitch"] for r in phase2):.1f}  min={min(r["pitch"] for r in phase2):.1f}')
if phase3:
    ap3 = sum(r['pitch'] for r in phase3) / len(phase3)
    print(f'After:  avg pitch={ap3:.1f} deg')
print()

if phase1:
    aogp1 = sum(abs(r['outp_gyro_pitch']) for r in phase1) / len(phase1)
    print(f'Before: avg |outp_gyro_pitch|={aogp1:.0f}')
if phase2:
    aogp2 = sum(abs(r['outp_gyro_pitch']) for r in phase2) / len(phase2)
    mogp2 = max(abs(r['outp_gyro_pitch']) for r in phase2)
    print(f'During: avg |outp_gyro_pitch|={aogp2:.0f}  max={mogp2:.0f}')
if phase3:
    aogp3 = sum(abs(r['outp_gyro_pitch']) for r in phase3) / len(phase3)
    print(f'After:  avg |outp_gyro_pitch|={aogp3:.0f}')
print()

if phase2:
    aml = sum(r['motor_l'] for r in phase2) / len(phase2)
    amr = sum(r['motor_r'] for r in phase2) / len(phase2)
    sat_l = sum(1 for r in phase2 if abs(r['motor_l']) > 7900)
    sat_r = sum(1 for r in phase2 if abs(r['motor_r']) > 7900)
    print(f'During: avg motor_L={aml:.0f}  motor_R={amr:.0f}')
    print(f'During: motor_L saturated: {sat_l}/{len(phase2)}  motor_R saturated: {sat_r}/{len(phase2)}')
print()

if phase2:
    aax = sum(r['ax_linear'] for r in phase2) / len(phase2)
    aay = sum(r['ay_linear'] for r in phase2) / len(phase2)
    may = max(abs(r['ay_linear']) for r in phase2)
    print(f'During: avg ax={aax:.2f}g  avg ay={aay:.2f}g  max|ay|={may:.2f}g')
print()

print('=== Top 5 pitch tipping frames ===')
sbp = sorted(rows, key=lambda r: r['pitch'], reverse=True)
for r in sbp[:5]:
    print(f'  pitch={r["pitch"]:6.1f}  roll={r["roll"]:5.1f}  gx={r["gyro_x"]:6.1f}  gz={r["gyro_z"]:6.1f}')
    print(f'  ot={r["outp_turn"]:5.0f}  ogp={r["outp_gyro_pitch"]:5.0f}  mL={r["motor_l"]:5.0f}  mR={r["motor_r"]:5.0f}')
    print(f'  ax={r["ax_linear"]:6.2f}  ay={r["ay_linear"]:6.2f}  yaw={r["yaw"]:6.1f}')
    print()

print('=== Top 5 largest |outp_gyro_pitch| ===')
sbo = sorted(rows, key=lambda r: abs(r['outp_gyro_pitch']), reverse=True)
for r in sbo[:5]:
    print(f'  ogp={r["outp_gyro_pitch"]:5.0f}  pitch={r["pitch"]:6.1f}  gx={r["gyro_x"]:6.1f}  gz={r["gyro_z"]:6.1f}')
    print(f'  mL={r["motor_l"]:5.0f}  mR={r["motor_r"]:5.0f}  ot={r["outp_turn"]:5.0f}  yaw={r["yaw"]:6.1f}')
    print()

print('=== Tipping moment (pitch flips from negative to positive) ===')
for i, r in enumerate(rows):
    if i > 0 and rows[i-1]['pitch'] < -1 and r['pitch'] > 1 and abs(r['gyro_z']) > 30:
        print(f'  Frame {i}: pitch {rows[i-1]["pitch"]:.1f} -> {r["pitch"]:.1f}')
        print(f'  gz={r["gyro_z"]:.1f}  ot={r["outp_turn"]:.0f}  ogp={r["outp_gyro_pitch"]:.0f}')
        print(f'  mL={r["motor_l"]:.0f}  mR={r["motor_r"]:.0f}  ax={r["ax_linear"]:.2f}  ay={r["ay_linear"]:.2f}')
        print()
        break

print('=== outp_turn vs pitch correlation ===')
if phase2:
    hi = [r for r in phase2 if abs(r['outp_turn']) > 500]
    lo = [r for r in phase2 if abs(r['outp_turn']) <= 500]
    if hi:
        aph = sum(abs(r['pitch']) for r in hi) / len(hi)
        print(f'  |outp_turn|>500: avg|pitch|={aph:.1f} deg ({len(hi)} frames)')
    if lo:
        apl = sum(abs(r['pitch']) for r in lo) / len(lo)
        print(f'  |outp_turn|<=500: avg|pitch|={apl:.1f} deg ({len(lo)} frames)')
