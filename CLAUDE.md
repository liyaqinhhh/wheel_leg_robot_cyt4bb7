# 项目编码规范

## 强制规则

- `.c` / `.h` 文件 → **GB2312** 编码（底层驱动要求，UTF-8 会导致 IAR 中文乱码）
- 其他文件（`.md` `.py` `.json` `.xml` 等）→ **UTF-8** 编码

## 写入 .c/.h 文件的流程（MUST）

AI 生成代码后必须执行编码转换，**不得跳过**：

```bash
# 1. 正常写入文件（Write/Edit 默认 UTF-8）
# 2. 立即转为 GB2312
python tools/encoding_converter.py wrap <filepath>
```

## 编码转换工具

工具: `tools/encoding_converter.py` (charset-normalizer)

```bash
# 扫描编码状态
python tools/encoding_converter.py scan

# AI 写完 C/H 后调用（UTF-8→GB2312）
python tools/encoding_converter.py wrap <filepath>

# 批量修正编码不符合规范的文件
python tools/encoding_converter.py convert

# 预览不修改
python tools/encoding_converter.py convert --dry-run
```
