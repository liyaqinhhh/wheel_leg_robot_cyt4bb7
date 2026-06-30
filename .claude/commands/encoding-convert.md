# 编码转换工具 (encoding-convert)

使用 `tools/encoding_converter.py` 管理项目文件编码。

## 编码规范
- `.c` / `.h` → GB2312（底层驱动要求）
- 其他文件 → UTF-8

## 参数
- `$ACTION`: scan | convert | fix | wrap - 默认: `scan`
- `$DIR`: 扫描目录 - 默认: `project/code project/user .claude/rules docs`
- `$EXT`: 文件扩展名 - 默认: 自动检测

## 执行步骤

1. 如果是 scan：
```bash
python tools/encoding_converter.py scan --dir $DIR
```

2. 如果是 convert，先 dry-run：
```bash
python tools/encoding_converter.py convert --dry-run --dir $DIR
```
确认后执行：
```bash
python tools/encoding_converter.py convert --dir $DIR
```

3. 如果是 wrap（AI 写完 C/H 后）：
```bash
python tools/encoding_converter.py wrap <filepath>
```

4. 转换完成后验证：
```bash
python tools/encoding_converter.py scan --dir $DIR
```
