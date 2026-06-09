"""Convert GpsNav files from UTF-8 to GB2312.
Try to recover garbled Chinese characters first, then convert."""
import os

folder = os.path.join(os.path.dirname(__file__), 'project', 'code', 'GpsNav')

# First, let's check if the garbled chars can be recovered
# by trying different encoding round-trips
def try_recover(text):
    """Try to recover garbled Chinese text through encoding round-trips."""
    # Try: UTF-8 -> latin-1 bytes -> GBK decode
    try:
        raw = text.encode('latin-1')
        recovered = raw.decode('gbk')
        return recovered, 'latin1->gbk'
    except:
        pass
    
    # Try: UTF-8 -> cp437 bytes -> GBK decode
    try:
        raw = text.encode('cp437')
        recovered = raw.decode('gbk')
        return recovered, 'cp437->gbk'
    except:
        pass
    
    # Try: UTF-8 -> cp1252 bytes -> GBK decode
    try:
        raw = text.encode('cp1252')
        recovered = raw.decode('gbk')
        return recovered, 'cp1252->gbk'
    except:
        pass
    
    return None, None

# Test recovery on a sample
for f in sorted(os.listdir(folder)):
    path = os.path.join(folder, f)
    with open(path, 'r', encoding='utf-8') as fh:
        content = fh.read()
    
    # Find garbled section (first line with problematic chars)
    for line in content.split('\n')[:5]:
        if 'CYT4BB' in line:
            print(f"\n{f} sample line (UTF-8):")
            print(f"  {line}")
            
            recovered, method = try_recover(line)
            if recovered:
                print(f"  Recovered ({method}):")
                print(f"  {recovered}")
            else:
                print(f"  Recovery failed")
            break

print("\n" + "="*60)
print("Attempting full file conversion with recovery...")
print("="*60)

success_count = 0
for f in sorted(os.listdir(folder)):
    path = os.path.join(folder, f)
    with open(path, 'r', encoding='utf-8') as fh:
        content = fh.read()
    
    # Try to recover the entire content
    recovered, method = try_recover(content)
    if recovered:
        # Check if recovered content can be encoded as GB2312
        try:
            encoded = recovered.encode('gb2312')
            with open(path, 'wb') as fh:
                fh.write(encoded)
            print(f"{f}: Recovered via {method}, saved as GB2312 ({len(encoded)} bytes)")
            success_count += 1
            continue
        except UnicodeEncodeError as e:
            # Try GBK as fallback
            try:
                encoded = recovered.encode('gbk')
                with open(path, 'wb') as fh:
                    fh.write(encoded)
                print(f"{f}: Recovered via {method}, saved as GBK ({len(encoded)} bytes)")
                success_count += 1
                continue
            except UnicodeEncodeError as e2:
                print(f"{f}: Recovered but still can't encode: {e2}")
    
    # If recovery failed, try direct GB2312 with errors='replace'
    try:
        encoded = content.encode('gb2312', errors='replace')
        with open(path, 'wb') as fh:
            fh.write(encoded)
        print(f"{f}: Direct GB2312 with replacement ({len(encoded)} bytes, some chars replaced)")
        success_count += 1
    except Exception as e:
        print(f"{f}: FAILED - {e}")

print(f"\n{success_count}/{len(os.listdir(folder))} files converted successfully")
