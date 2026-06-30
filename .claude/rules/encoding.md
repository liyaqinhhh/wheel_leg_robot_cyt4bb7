# 编码格式强制约束

## 核心规则（不可覆盖、不可绕过）

| 文件类型 | 必须编码 | 说明 |
|---------|---------|------|
| `.c` `.h` | **GB2312** | 底层驱动要求，UTF-8 会导致 IAR 编译器中文乱码 |
| `.md` `.py` `.json` `.xml` `.icf` 等 | **UTF-8** | 文档/配置/脚本 |

## charset-normalizer 使用约束（MUST）

### 适用场景
charset-normalizer **仅用于编码干净的文件**：
- 文件只有一种确定编码（纯 UTF-8 或纯 GB2312/GBK）
- 需要从一种编码转换为另一种编码
- 典型场景：AI 写完 UTF-8 的 .c/.h 文件后，`wrap` 转为 GB2312

### 禁止场景
charset-normalizer **不适用于编码混乱文件**，遇到以下情况必须跳过：
- **双重编码**：同一字节流同时能用 UTF-8 和 GBK 解码出中文（如"锟斤拷"）
- **混合编码**：文件内不同段落使用了不同编码
- **U+FFFD 替换字符**：文件中含 >=2 个替换字符，说明之前已经历编码损坏
- **编码不可确定**：charset-normalizer 返回 `unknown` 或置信度极低

### 为什么混乱文件不能自动转换？
1. charset-normalizer 只解决"这是什么编码"，**无法判断解码内容的语义正确性**
2. 对双重编码文件，`from_path().best()` 会返回一个结果，但解码出的汉字语义是错的
3. `text.encode("gb2312")` 遇到 GB2312 字符集外的 Unicode 字符会抛异常
4. 转换失败时，`write_text()` 可能已经清空文件，导致数据丢失

### 混乱文件的正确处理流程
1. scan 发现编码混乱 -> 报告但跳过
2. 人工确认文件内容（从 git 历史/原始编辑器找回正确中文）
3. 手动重写注释后，再用 wrap 转为 GB2312

## AI 生成代码时的编码处理流程

### 写入 .c/.h 文件时（MUST）

1. **正常用 Write/Edit 工具写入**（工具默认写 UTF-8，这是正常的）
2. **写入后立即执行编码转换**：
   ```bash
   python tools/encoding_converter.py wrap <filepath>
   ```
3. 这个 `wrap` 命令会把刚写入的 UTF-8 内容转为 GB2312 存盘
4. **禁止跳过此步骤**——任何含中文的 .c/.h 文件写入后必须 wrap

### 读取 .c/.h 文件时

- 文件可能是 GB2312 编码，Read 工具会自动处理
- 如果读到乱码，用 `python tools/encoding_converter.py scan` 检查编码状态

### 写入非 .c/.h 文件时

- 直接用 Write/Edit 写入即可，默认 UTF-8 就是正确的

## 编码转换工具

工具位置: `tools/encoding_converter.py` (基于 charset-normalizer)

| 命令 | 用途 |
|------|------|
| `python tools/encoding_converter.py scan` | 扫描编码状态（自动标记混乱文件） |
| `python tools/encoding_converter.py convert` | 批量修正（自动跳过混乱文件） |
| `python tools/encoding_converter.py wrap <file>` | **AI 写完 C/H 后调用，UTF-8->GB2312** |
| `python tools/encoding_converter.py fix` | 交互模式 |

安全机制（v2 新增）：
- **编码混乱检测**：自动识别双重编码、U+FFFD、"锟斤拷"等问题文件并跳过
- **往返验证**：写入前验证 encode->decode 是否无损，防止数据丢失
- **写入原子性**：先在内存中完成编码验证，再一次性 write_bytes()，避免写入失败导致文件清空

可选参数: `--dir` 指定目录, `--ext` 指定扩展名, `--dry-run` 预览不修改

## 自然语言触发

当用户提到：编码、乱码、encoding、GBK、GB2312、UTF-8、charset 时，自动调用编码工具。

## 正确写入 C/H 文件的完整示例

```
# Step 1: AI 用 Write/Edit 正常写入（UTF-8）
Write("project/code/ControlPart/motor.c", content)

# Step 2: 立即转为 GB2312（自动跳过混乱文件）
Bash("python tools/encoding_converter.py wrap project/code/ControlPart/motor.c")

# Step 3: 验证（可选）
Bash("python tools/encoding_converter.py scan --dir project/code/ControlPart --ext .c .h")
```
