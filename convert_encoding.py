"""Convert GpsNav files from UTF-8 to GB2312, replacing incompatible chars."""
import os
import re

folder = os.path.join(os.path.dirname(__file__), 'project', 'code', 'GpsNav')

# First pass: find all problematic characters
print("=== Scanning for GB2312-incompatible characters ===")
for f in sorted(os.listdir(folder)):
    path = os.path.join(folder, f)
    with open(path, 'r', encoding='utf-8') as fh:
        content = fh.read()
    bad_chars = []
    for i, ch in enumerate(content):
        try:
            ch.encode('gb2312')
        except:
            bad_chars.append((ch, hex(ord(ch)), i))
    if bad_chars:
        print(f'\n{f}: {len(bad_chars)} problematic chars:')
        seen = set()
        for ch, code, pos in bad_chars:
            if ch not in seen:
                seen.add(ch)
                start = max(0, pos - 10)
                end = min(len(content), pos + 11)
                ctx = content[start:end].replace('\n', '\\n')
                print(f'  char={ch!r} code={code} first_pos={pos} context="{ctx}"')
    else:
        print(f'{f}: all OK')

# Second pass: convert using GBK (superset of GB2312, handles more chars)
# GBK is the practical choice for Chinese embedded systems
print("\n=== Converting UTF-8 -> GB2312 (with GBK fallback) ===")
for f in sorted(os.listdir(folder)):
    path = os.path.join(folder, f)
    with open(path, 'r', encoding='utf-8') as fh:
        content = fh.read()
    
    # Try GB2312 first, fall back to GBK
    try:
        encoded = content.encode('gb2312')
        enc_name = 'GB2312'
    except UnicodeEncodeError:
        encoded = content.encode('gbk')
        enc_name = 'GBK (fallback)'
    
    with open(path, 'wb') as fh:
        fh.write(encoded)
    
    # Verify
    with open(path, 'rb') as fh:
        raw = fh.read()
    try:
        raw.decode('gb2312')
        print(f'{f}: UTF-8 -> GB2312 OK ({len(raw)} bytes)')
    except:
        try:
            raw.decode('gbk')
            print(f'{f}: UTF-8 -> {enc_name} OK ({len(raw)} bytes)')
        except Exception as e:
            print(f'{f}: FAILED - {e}')
