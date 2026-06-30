# 项目编码规范

## 强制规则

- `.c` / `.h` 文件 → **GB2312** 编码（底层驱动要求，UTF-8 会导致 IAR 中文乱码）
- 其他文件（`.md` `.py` `.json` `.xml` 等）→ **UTF-8** 编码

## charset-normalizer 使用约束

### 适用场景 ✅
- 文件编码干净（只有一种确定编码）
- 需要从一种编码转为另一种（如 UTF-8 → GB2312）
- 典型场景：AI 写完 .c/.h 文件后调用 `wrap` 转为 GB2312

### 禁止场景 ❌
以下情况 charset-normalizer **不可调用**，必须跳过并报告：
- **双重编码**：同一字节流同时能用 UTF-8 和 GBK 解码出中文
- **混合编码**：文件内不同段落使用了不同编码
- **U+FFFD 替换字符**：含 >=2 个替换字符（编码损坏标志）
- **编码不可确定**：charset-normalizer 返回 `unknown` 或置信度极低

### 原因
charset-normalizer 只回答"这是什么编码"，**无法判断解码内容的语义正确性**。对编码混乱文件调用会导致：
- 解码出语义错误的汉字
- encode 失败抛异常
- write_text() 可能清空文件

### 混乱文件处理流程
1. `scan` 发现编码混乱 → 报告但跳过
2. 人工从 git 历史找回正确中文
3. 手动重写后再转 GB2312

## 写入 .c/.h 文件的流程（MUST）

AI 生成代码后必须执行编码转换，**不得跳过**：

```bash
# 1. 先正常写入文件（默认 UTF-8）
# 2. 立即转为 GB2312
python tools/encoding_converter.py wrap <filepath>
```

## 编码转换工具

工具: `tools/encoding_converter.py` (charset-normalizer)

```bash
# 扫描编码状态（自动标记混乱文件）
python tools/encoding_converter.py scan

# AI 写完 C/H 后调用（UTF-8→GB2312）
python tools/encoding_converter.py wrap <filepath>

# 批量修正编码不符合规范的文件（自动跳过混乱文件）
python tools/encoding_converter.py convert

# 预览不修改
python tools/encoding_converter.py convert --dry-run
```

安全机制（v2）：
- 编码混乱检测：自动跳过双重编码/U+FFFD/锟斤拷等问题文件
- 往返验证：写入前验证 encode→decode 无损
- 写入原子性：内存验证后一次性 write_bytes()
