# 编码整改与注释补充 — 约束文件

> 项目：轮腿机器人 CYT4BB7
> 范围：`project/code/` 下所有子文件夹的 `.c` / `.h` 文件
> 日期：2026-06-05

---

## 1. 编码格式规则

### 1.1 文件分类

| 分类 | 文件 | 处理方式 |
|------|------|----------|
| **A: 干净 UTF-8** | 31 个文件（见下方清单） | **不转码**，保持 UTF-8 |
| **B: 编码混乱/GBK** | 8 个文件（见下方清单） | 转为 GB2312 |

### 1.2 A 类文件清单（保持 UTF-8，不转码）

```
ControlPart/Buzzer.c
ControlPart/Buzzer.h
ControlPart/Init.h
ControlPart/Interrupt.h
ControlPart/Math_Advanced.c
ControlPart/Math_Advanced.h
ControlPart/ekf.c
ControlPart/ekf.h
ControlPart/imu660.c
ControlPart/imu660.h
ControlPart/kalman.c
ControlPart/kalman.h
ControlPart/matrix.c
ControlPart/matrix.h
ControlPart/servo.c
ControlPart/servo.h
ControlPart/small_driver_uart_control.c
ControlPart/small_driver_uart_control.h
ControlPart/yaokong.c
ControlPart/yaokong.h
ins/ins_core.c
ins/ins_core.h
ins/ins_interface.c
ins/ins_interface.h
ins/ins_track.c
ins/ins_track.h
Menu/menu.c
Menu/menu.h
VersionPart/image.c
VersionPart/image.h
VersionPart/ips.c
VersionPart/ips.h
```

### 1.3 B 类文件清单（转 GB2312）

| 文件 | 当前编码 | 问题 |
|------|----------|------|
| `ControlPart/PID.c` | 混合编码（非 UTF-8 非 GBK） | 中文注释严重乱码 |
| `ControlPart/PID.h` | 混合编码（非 UTF-8 非 GBK） | 中文注释乱码 |
| `ControlPart/Init.c` | UTF-8（中文为双重编码乱码） | 中文注释为 GBK 误存为 UTF-8 |
| `ControlPart/Interrupt.c` | UTF-8（中文为双重编码乱码） | 中文注释为 GBK 误存为 UTF-8 |
| `ControlPart/zf_device_lora3a22.c` | GBK | 逐飞科技第三方库 |
| `ControlPart/zf_device_lora3a22.h` | GBK | 逐飞科技第三方库 |
| `ins/Ins.c` | GBK | 中文正常 |
| `ins/Ins.h` | GBK | 中文正常 |

### 1.4 转码操作规范

1. **读取**：先用正确编码（GBK/GB2312）读取源文件内容
2. **修复**：对乱码内容，根据代码上下文语义还原正确的中文
3. **写入**：以 GB2312 编码写回文件
4. **验证**：写回后重新读取验证中文显示正确

### 1.5 禁止项

- ❌ 不得对 A 类文件做编码转换
- ❌ 不得对 `image.c` 的数据部分添加注释（405KB 图像点阵数据）
- ❌ 不得修改任何代码逻辑，只改编码和注释
- ❌ 不得引入新的 `#include` 或修改现有 `#include`
- ❌ 不得修改变量名、函数名或宏定义名

---

## 2. 注释补充规则

### 2.1 注释语言

- **所有新增注释使用中文**
- 代码关键字、变量名、函数名保持英文不变

### 2.2 注释风格

参照项目中已有的良好注释风格（如 `small_driver_uart_control.c`、`ins_core.c`）：

```c
//-------------------------------------------------------------------------------------------------------------------
//  函数简介     简短的函数功能描述
//  参数说明     param1  参数1说明
//              param2  参数2说明
//  返回参数     返回值说明
//  使用示例     function_name(arg1, arg2);
//  备注信息     额外说明
//-------------------------------------------------------------------------------------------------------------------
```

或简洁风格：

```c
// @brief  简短功能描述
// @param  param1  参数1说明
// @return 返回值说明
```

### 2.3 注释补充位置

#### 必须注释

1. **函数头**：每个函数必须有功能说明
   - 函数功能（1 句话）
   - 参数含义
   - 返回值含义（非 void 时）

2. **关键算法步骤**：
   - PID 计算各环（角度环、速度环、转向环）
   - 卡尔曼滤波更新步骤
   - EKF 预测/更新步骤
   - 运动学正/逆解
   - INS 坐标累加与 ZUPT 检测

3. **重要变量声明**：
   - PID 参数数组（说明每个元素对应什么参数）
   - 状态标志位（说明每个值的含义）
   - 控制模式变量（说明各模式含义）

4. **模块间接口**：
   - 中断回调函数
   - 串口通信协议
   - 数据流向说明

#### 可选注释

- 简单的赋值语句（明显的不需要）
- 已有清晰注释的代码段（不重复）

### 2.4 第三方库注释规则

- `zf_device_lora3a22.c/h`：**只转编码，不改注释内容**（逐飞科技库）
- 其他第三方代码（如有标注 SEEKFREE 等的）：同样只转编码不改注释

### 2.5 乱码修复规则

对 B 类文件中的乱码注释：
1. 根据代码上下文、变量名、函数名推断原意
2. 用正确的中文重写注释
3. 无法确定原意的，保留合理推断并标注 `// [推测]`

---

## 3. 文件修改约束

### 3.1 修改范围

- 只修改 `project/code/` 目录下的 `.c` 和 `.h` 文件
- 不修改其他目录下的任何文件
- 不创建新文件（约束文件和任务集除外）

### 3.2 保留项

- 保留所有现有代码逻辑
- 保留 `Created on` / `Author` 等原始文件头
- 保留 `#include` 顺序
- 保留 `#ifndef` / `#define` / `#endif` 宏保护
- 保留代码缩进风格（Tab 缩进）

### 3.3 image.c 特殊处理

- `VersionPart/image.c`（405KB）仅处理文件头部注释
- 大数组数据部分（图像点阵）不做任何修改
- 不对该文件做编码转换

---

## 4. 质量检查

完成所有任务后，对每个修改的文件进行：

1. **编码验证**：文件可被正确编码读取，无乱码
2. **编译验证**：注释修改不引入语法错误（确保 `/* */` 配对、`//` 行末正确）
3. **完整性验证**：所有函数都有注释头，关键变量都有行内注释
4. **一致性验证**：注释风格与项目已有风格一致
