"""
合并冲突解决脚本
策略：保留 UTF-8 分支的功能代码（更好的PID值、turn_mode=7、完整实现），
      然后统一转换为 GBK 编码。
"""
import os
import sys

def resolve_conflicts_and_convert_to_gbk(filepath):
    """读取含合并冲突的文件，保留 other 分支内容，转换为 GBK 编码"""
    
    print(f"\n{'='*60}")
    print(f"Processing: {filepath}")
    print(f"{'='*60}")
    
    # 读取原始字节
    with open(filepath, 'rb') as f:
        content = f.read()
    
    # 按行分割
    lines = content.split(b'\n')
    
    result_lines = []
    in_head = False
    in_other = False
    conflict_count = 0
    head_lines = 0
    other_lines = 0
    
    for line in lines:
        stripped = line.strip()
        
        if stripped.startswith(b'<<<<<<< '):
            in_head = True
            conflict_count += 1
            continue
        elif stripped == b'=======' and in_head:
            in_head = False
            in_other = True
            continue
        elif stripped.startswith(b'>>>>>>> '):
            in_other = False
            continue
        
        if in_head:
            head_lines += 1
            continue  # 跳过 HEAD 部分
        elif in_other:
            other_lines += 1
            result_lines.append(line)  # 保留 other 分支内容
        else:
            result_lines.append(line)  # 保留非冲突行
    
    # 合并结果
    result = b'\n'.join(result_lines)
    
    print(f"  冲突数: {conflict_count}")
    print(f"  跳过 HEAD 行数: {head_lines}")
    print(f"  保留 other 行数: {other_lines}")
    print(f"  结果总行数: {len(result_lines)}")
    
    # 尝试 UTF-8 解码
    try:
        text = result.decode('utf-8')
        print("  UTF-8 解码: 成功")
    except UnicodeDecodeError as e:
        print(f"  UTF-8 解码失败: {e}")
        # 尝试 GBK 解码（非冲突区可能有 GBK 字节）
        try:
            text = result.decode('gbk')
            print("  GBK 解码: 成功")
        except UnicodeDecodeError as e2:
            print(f"  GBK 解码失败: {e2}")
            # 最终回退：使用 errors='replace'
            text = result.decode('utf-8', errors='replace')
            print("  回退到 UTF-8 (errors=replace)")
    
    # 转换为 GBK 编码
    try:
        gbk_bytes = text.encode('gbk')
        print("  GBK 编码: 成功")
    except UnicodeEncodeError as e:
        print(f"  GBK 编码失败: {e}")
        gbk_bytes = text.encode('gbk', errors='replace')
        print("  GBK 编码 (errors=replace)")
    
    # 写回文件
    with open(filepath, 'wb') as f:
        f.write(gbk_bytes)
    
    print(f"  写入 {len(gbk_bytes)} 字节")
    print(f"  完成!")

# 处理三个冲突文件
base = r'c:\Users\keywo\Desktop\CYT4BB7_work\wheel_leg_robot_cyt4bb7\project\code\ControlPart'
files = [
    os.path.join(base, 'Interrupt.c'),
    os.path.join(base, 'PID.c'),
    os.path.join(base, 'PID.h'),
]

for f in files:
    if os.path.exists(f):
        resolve_conflicts_and_convert_to_gbk(f)
    else:
        print(f"文件不存在: {f}")

print("\n" + "="*60)
print("所有文件处理完成!")
print("="*60)
