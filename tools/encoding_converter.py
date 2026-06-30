#!/usr/bin/env python3
"""
批量文件编码转换工具 - 基于 charset-normalizer

功能:
  1. scan    - 扫描项目文件编码状态（只读，不修改）
  2. convert - 批量转换非 GB2312 的 C/H 文件为 GB2312
  3. fix     - 扫描 + 确认后转换（交互模式）
  4. wrap    - 单文件 UTF-8 -> GB2312 转换（AI 写入 C/H 后调用）

安全约束:
  charset-normalizer 仅用于编码干净文件的转换。
  编码混乱文件（双重编码、混合编码、含 U+FFFD 替换字符等）
  会被自动检测并跳过，避免转换过程中文件损坏。
"""

import argparse
import os
import sys
from pathlib import Path

try:
    from charset_normalizer import from_path
except ImportError:
    print("ERROR: charset-normalizer 未安装，请先执行: pip install charset-normalizer")
    sys.exit(1)


PROJECT_ROOT = Path(__file__).resolve().parent.parent
GB2312_EXTS = {".c", ".h"}
UTF8_EXTS = {".md", ".py", ".json", ".xml", ".yaml", ".yml", ".toml", ".txt", ".icf"}
DEFAULT_SCAN_DIRS = ["project/code", "project/user"]
DEFAULT_SCAN_DIRS_EXTRA = [".claude/rules", "docs"]


def _p(text):
    """安全打印：Windows GBK 终端无法显示 emoji"""
    for emoji, plain in {"✅": "[OK]", "⚠️": "[!!]", "❌": "[X]", "\U0001f389": "[done]"}.items():
        text = text.replace(emoji, plain)
    return text


def is_encoding_chaotic(filepath):
    """
    检测文件是否存在编码混乱。

    编码混乱的典型特征:
      1. 同一段字节既能用 UTF-8 解码又能用 GBK 解码（且都含中文）-> 双重编码
      2. 解码后含有 U+FFFD 替换字符 -> 编码损坏残留
      3. 解码后含有"锟斤拷" -> 经典双重编码损坏标志

    返回: (is_chaotic: bool, reason: str)
    """
    raw = filepath.read_bytes()
    if len(raw) == 0:
        return False, ""

    # 检测1: UTF-8 和 GBK 都能解码出中文 -> 疑似双重编码
    utf8_text = None
    try:
        utf8_text = raw.decode("utf-8")
    except UnicodeDecodeError:
        pass

    gbk_text = None
    try:
        gbk_text = raw.decode("gbk", errors="strict")
    except (UnicodeDecodeError, ValueError):
        pass

    if utf8_text is not None and gbk_text is not None:
        utf8_has_cjk = any(0x4E00 <= ord(c) <= 0x9FFF for c in utf8_text[:200])
        gbk_has_cjk = any(0x4E00 <= ord(c) <= 0x9FFF for c in gbk_text[:200])
        if utf8_has_cjk and gbk_has_cjk:
            return True, "双重编码: UTF-8 和 GBK 均能解码出中文"

    # 检测2 和 3: 在可解码的文本中查找损坏标志
    check_text = utf8_text if utf8_text is not None else gbk_text
    if check_text:
        fffd_count = check_text.count("�")
        if fffd_count >= 2:
            return True, "含 %d 个 U+FFFD 替换字符（编码损坏标志）" % fffd_count
        if "锝斤拷" in check_text:  # 锟斤拷
            return True, "含'锟斤拷'（经典双重编码损坏标志）"

    return False, ""


def expected_encoding(filepath):
    ext = filepath.suffix.lower()
    if ext in GB2312_EXTS:
        return "gb2312"
    return "utf-8"


def detect_encoding(filepath):
    raw_size = filepath.stat().st_size
    if raw_size == 0:
        return {"path": filepath, "encoding": "empty", "expected": expected_encoding(filepath),
                "is_correct": True, "needs_convert": False, "size": 0,
                "is_chaotic": False, "chaos_reason": ""}

    raw = filepath.read_bytes()
    is_chaotic, chaos_reason = is_encoding_chaotic(filepath)

    # 纯 ASCII
    is_ascii = False
    try:
        raw.decode("ascii")
        is_ascii = True
    except (UnicodeDecodeError, ValueError):
        pass

    if is_ascii:
        exp = expected_encoding(filepath)
        return {"path": filepath, "encoding": "ascii", "expected": exp,
                "is_correct": True, "needs_convert": False, "size": raw_size,
                "is_chaotic": False, "chaos_reason": ""}

    # UTF-8
    is_utf8 = False
    try:
        raw.decode("utf-8")
        is_utf8 = True
    except UnicodeDecodeError:
        pass

    # GB2312/GBK/GB18030
    is_gb = False
    gb_name = None
    for enc in ("gb2312", "gbk", "gb18030"):
        try:
            raw.decode(enc)
            is_gb = True
            gb_name = enc
            break
        except (UnicodeDecodeError, ValueError):
            pass

    exp = expected_encoding(filepath)

    if exp == "gb2312":
        if is_gb:
            return {"path": filepath, "encoding": gb_name, "expected": "gb2312",
                    "is_correct": True, "needs_convert": False, "size": raw_size,
                    "is_chaotic": is_chaotic, "chaos_reason": chaos_reason}
        elif is_utf8:
            return {"path": filepath, "encoding": "utf-8", "expected": "gb2312",
                    "is_correct": False, "needs_convert": not is_chaotic,
                    "size": raw_size, "is_chaotic": is_chaotic, "chaos_reason": chaos_reason}
    else:
        if is_utf8:
            return {"path": filepath, "encoding": "utf-8", "expected": "utf-8",
                    "is_correct": True, "needs_convert": False, "size": raw_size,
                    "is_chaotic": is_chaotic, "chaos_reason": chaos_reason}
        elif is_gb:
            return {"path": filepath, "encoding": gb_name, "expected": "utf-8",
                    "is_correct": False, "needs_convert": not is_chaotic,
                    "size": raw_size, "is_chaotic": is_chaotic, "chaos_reason": chaos_reason}

    # charset-normalizer 兜底
    results = from_path(str(filepath))
    best = results.best()
    if best is None:
        return {"path": filepath, "encoding": "unknown", "expected": exp,
                "is_correct": False, "needs_convert": False, "size": raw_size,
                "is_chaotic": True, "chaos_reason": "无法确定编码"}

    detected = str(best.encoding).lower().replace("-", "").replace("_", "")
    is_gb_detected = detected in ("gb2312", "gbk", "gb18030")
    is_utf8_detected = detected in ("utf8", "utf8sig")
    correct = (exp == "gb2312" and is_gb_detected) or (exp == "utf-8" and is_utf8_detected)

    return {"path": filepath, "encoding": str(best.encoding), "expected": exp,
            "is_correct": correct, "needs_convert": (not correct) and (not is_chaotic),
            "size": raw_size, "is_chaotic": is_chaotic, "chaos_reason": chaos_reason}


def collect_files(dirs, extensions=None):
    files = []
    for d in dirs:
        base = PROJECT_ROOT / d
        if not base.exists():
            continue
        if extensions:
            for ext in extensions:
                files.extend(base.rglob(f"*{ext}"))
        else:
            for ext in GB2312_EXTS | UTF8_EXTS:
                files.extend(base.rglob(f"*{ext}"))
    return sorted(set(files))


def scan_files(dirs, extensions=None):
    files = collect_files(dirs, extensions)
    if not files:
        print("未找到匹配的文件")
        return []

    print(f"\n{'文件':<60s} {'当前编码':<12s} {'期望编码':<10s} {'大小':>8s} {'状态'}")
    print("-" * 105)

    results = []
    for f in files:
        info = detect_encoding(f)
        rel_path = f.relative_to(PROJECT_ROOT)
        status = "[!!]" if info.get("is_chaotic") or not info["is_correct"] else "[OK]"
        size_str = f"{info['size']:>6d}B"
        line = f"{str(rel_path):<60s} {info['encoding']:<12s} {info['expected']:<10s} {size_str:>8s} {status}"
        print(_p(line))
        if info.get("is_chaotic") and info.get("chaos_reason"):
            print(f"  >> 编码混乱: {info['chaos_reason']}")
        results.append(info)

    correct = sum(1 for r in results if r["is_correct"] and not r.get("is_chaotic"))
    chaotic = [r for r in results if r.get("is_chaotic")]
    need_convert = [r for r in results if r["needs_convert"]]

    print("-" * 105)
    print(f"总计: {len(results)} 个文件 | 编码正确: {correct} | 可转换: {len(need_convert)} | 编码混乱: {len(chaotic)}")

    if need_convert:
        print(f"\n可安全转换的文件:")
        for r in need_convert:
            rel = r["path"].relative_to(PROJECT_ROOT)
            print(f"  {rel}  ({r['encoding']} -> {r['expected']})")

    if chaotic:
        print(f"\n编码混乱文件（已跳过，需人工处理）:")
        for r in chaotic:
            rel = r["path"].relative_to(PROJECT_ROOT)
            print(f"  {rel}  ({r['encoding']}) - {r.get('chaos_reason', '未知原因')}")

    return results


def convert_files(dirs, extensions=None, dry_run=False):
    """批量转换（自动跳过编码混乱文件，写入前做往返验证）"""
    results = scan_files(dirs, extensions)
    need_convert = [r for r in results if r["needs_convert"]]

    if not need_convert:
        print(_p("\n[done] 所有可安全转换的文件已符合规范！"))
        return []

    prefix = '[DRY RUN] ' if dry_run else ''
    print(f"\n{prefix}准备转换 {len(need_convert)} 个文件:")

    converted = []
    for r in need_convert:
        rel = r["path"].relative_to(PROJECT_ROOT)
        target_enc = r["expected"]

        # 二次确认：混乱文件不转换
        if r.get("is_chaotic"):
            print(f"  [SKIP] {rel}: 编码混乱，跳过（{r.get('chaos_reason', '')}）")
            continue

        if dry_run:
            print(f"  [DRY RUN] {rel}: {r['encoding']} -> {target_enc}")
            converted.append(r)
            continue

        try:
            if target_enc == "gb2312":
                try:
                    text = r["path"].read_text(encoding="utf-8")
                except UnicodeDecodeError:
                    cn_results = from_path(str(r["path"]))
                    best = cn_results.best()
                    if best is None:
                        print(f"  [X] {rel}: 无法解码，跳过")
                        continue
                    text = str(best)

                # 编码 + 往返验证
                try:
                    encoded = text.encode("gb2312")
                except UnicodeEncodeError as e:
                    print(f"  [X] {rel}: 含 GB2312 不兼容字符 - {e}")
                    continue

                if encoded.decode("gb2312") != text:
                    print(f"  [X] {rel}: 编码往返验证失败，跳过")
                    continue

                r["path"].write_bytes(encoded)
                print(f"  [OK] {rel}: {r['encoding']} -> gb2312")

            else:
                try:
                    text = r["path"].read_text(encoding="gbk")
                except UnicodeDecodeError:
                    cn_results = from_path(str(r["path"]))
                    best = cn_results.best()
                    if best is None:
                        print(f"  [X] {rel}: 无法解码，跳过")
                        continue
                    text = str(best)

                encoded = text.encode("utf-8")
                if encoded.decode("utf-8") != text:
                    print(f"  [X] {rel}: 编码往返验证失败，跳过")
                    continue

                r["path"].write_bytes(encoded)
                print(f"  [OK] {rel}: {r['encoding']} -> utf-8")

            converted.append(r)

        except Exception as e:
            print(f"  [X] {rel}: 转换失败 - {e}")

    if not dry_run and converted:
        print(f"\n[OK] 成功转换 {len(converted)}/{len(need_convert)} 个文件")
    elif dry_run:
        print(f"\n[DRY RUN] 共 {len(converted)} 个文件将被转换")

    return converted


def wrap_single(filepath):
    """
    单文件 UTF-8 -> GB2312 转换（AI 写完 C/H 后调用）。

    安全保证:
      1. 编码混乱文件自动跳过
      2. 写入前先验证编码往返无损
      3. 写入失败时原文件保持不变
    """
    p = Path(filepath)
    if not p.exists():
        print(f"[X] 文件不存在: {filepath}")
        return False

    if p.suffix.lower() not in GB2312_EXTS:
        print(f"[SKIP] 非 C/H 文件，无需转 GB2312: {filepath}")
        return True

    raw = p.read_bytes()

    # 已是 GB2312 则跳过
    try:
        raw.decode("gb2312")
        try:
            raw.decode("utf-8")
            cn = from_path(str(p))
            best = cn.best()
            if best and str(best.encoding).lower().replace("-","").replace("_","") in ("gb2312", "gbk", "gb18030"):
                print(f"[OK] 已是 GB2312 编码: {filepath}")
                return True
        except UnicodeDecodeError:
            print(f"[OK] 已是 GB2312 编码: {filepath}")
            return True
    except UnicodeDecodeError:
        pass

    # 编码混乱检测
    is_chaotic, chaos_reason = is_encoding_chaotic(p)
    if is_chaotic:
        print(f"[SKIP] {filepath}: 编码混乱，跳过（{chaos_reason}）")
        print(f"       需人工修复后再转换")
        return False

    # 读取
    try:
        text = p.read_text(encoding="utf-8")
    except UnicodeDecodeError:
        cn = from_path(str(p))
        best = cn.best()
        if best is None:
            print(f"[X] 无法解码文件: {filepath}")
            return False
        text = str(best)

    # 编码 + 往返验证
    try:
        encoded = text.encode("gb2312")
    except UnicodeEncodeError as e:
        print(f"[X] {filepath}: 含 GB2312 不兼容字符 - {e}")
        print(f"    提示: 需先替换不兼容字符后再重试")
        return False

    if encoded.decode("gb2312") != text:
        print(f"[X] {filepath}: 编码往返验证失败，中止写入")
        return False

    p.write_bytes(encoded)
    print(f"[OK] {filepath}: utf-8 -> gb2312")
    return True


def fix_interactive(dirs, extensions=None):
    results = scan_files(dirs, extensions)
    need_convert = [r for r in results if r["needs_convert"]]

    if not need_convert:
        print(_p("\n[done] 所有可安全转换的文件已符合规范！"))
        return

    print(f"\n发现 {len(need_convert)} 个文件可安全转换，是否执行？")
    answer = input("输入 y 确认转换，其他键取消: ").strip().lower()

    if answer == "y":
        convert_files(dirs, extensions, dry_run=False)
    else:
        print("已取消")


def main():
    parser = argparse.ArgumentParser(
        description="批量文件编码转换工具 (charset-normalizer)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("action", choices=["scan", "convert", "fix", "wrap"],
        help="scan=扫描编码 | convert=批量转换 | fix=交互修复 | wrap=单文件转GB2312")
    parser.add_argument("--dir", nargs="+", default=None,
        help="扫描目录 (默认: project/code project/user)")
    parser.add_argument("--ext", nargs="+", default=None,
        help="文件扩展名 (默认: 自动检测 .c .h .md .py 等)")
    parser.add_argument("--dry-run", action="store_true", help="仅预览，不实际修改文件")
    parser.add_argument("filepath", nargs="?", default=None,
        help="wrap 模式: 指定要转换的文件路径")

    args = parser.parse_args()

    if args.action == "wrap":
        if not args.filepath:
            print("ERROR: wrap 模式需要指定文件路径")
            print("用法: python tools/encoding_converter.py wrap <filepath>")
            sys.exit(1)
        success = wrap_single(args.filepath)
        sys.exit(0 if success else 1)

    dirs = args.dir if args.dir else DEFAULT_SCAN_DIRS + DEFAULT_SCAN_DIRS_EXTRA
    exts = args.ext if args.ext else None

    if args.action == "scan":
        scan_files(dirs, exts)
    elif args.action == "convert":
        convert_files(dirs, exts, dry_run=args.dry_run)
    elif args.action == "fix":
        fix_interactive(dirs, exts)


if __name__ == "__main__":
    main()
