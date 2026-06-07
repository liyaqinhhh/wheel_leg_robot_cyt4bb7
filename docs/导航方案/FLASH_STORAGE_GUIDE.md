# Flash 存储分区科普 — CYT4BB7 Work Flash

## 1. 硬件基础

CYT4BB7 的 Work Flash 是一块 **独立于主 Flash 的非易失存储区**，特点：

| 属性 | 值 |
|------|-----|
| 总容量 | 192 KB |
| 页大小 | 2 KB (2048 字节) |
| 总页数 | 96 页 (编号 0~95) |
| 每页 uint32 数量 | 512 个 |
| 擦除粒度 | 整页擦除（写 0xFF） |
| 写入粒度 | 按 uint32 写入 |
| 寿命 | ~10 万次擦写周期 |

## 2. API 速查

```c
// 所有 API 的 sector_num 参数固定为 0

flash_check(0, page_num)              // 检查页是否有数据（非全 0xFF）
flash_erase_page(0, page_num)         // 擦除一页（变为全 0xFF）
flash_read_page(0, page_num, buf, len)  // 读 len 个 uint32 到 buf
flash_write_page(0, page_num, buf, len) // 擦除+写入 len 个 uint32

// 缓冲区方式（项目实际使用的方式）
flash_read_page_to_buffer(0, page_num, len)   // 读到全局缓冲区 flash_union_buffer[]
flash_write_page_from_buffer(0, page_num, len) // 从全局缓冲区写入
flash_buffer_clear()                           // 清空缓冲区为 0xFF
```

### 关键全局变量

```c
flash_data_union flash_union_buffer[FLASH_PAGE_LENGTH];  // 512 个元素的联合体缓冲区
```

`flash_data_union` 是一个联合体，可以同一位置用不同类型访问：

```c
typedef union {
    uint32  uint32_type;
    int32   int32_type;
    float   float_type;
    uint16  uint16_type;
    int16   int16_type;
    uint8   uint8_type;
    int8    int8_type;
} flash_data_union;
```

## 3. 写入流程（必须遵守）

```
┌─────────────────────────────────────────────────┐
│  Step 1: flash_buffer_clear()                   │
│          清空缓冲区                               │
├─────────────────────────────────────────────────┤
│  Step 2: 填充 flash_union_buffer[]              │
│          用 .uint32_type / .float_type 等写入数据 │
├─────────────────────────────────────────────────┤
│  Step 3: flash_write_page_from_buffer(          │
│            0, page_num, len)                     │
│          内部自动：检查→擦除→写入                  │
└─────────────────────────────────────────────────┘
```

### 读取流程

```
┌─────────────────────────────────────────────────┐
│  Step 1: flash_read_page_to_buffer(             │
│            0, page_num, len)                     │
│          数据加载到 flash_union_buffer[]          │
├─────────────────────────────────────────────────┤
│  Step 2: 从 flash_union_buffer[] 读取           │
│          用 .uint32_type / .float_type 等读取    │
├─────────────────────────────────────────────────┤
│  Step 3: flash_buffer_clear()                   │
│          用完后清空（可选但推荐）                   │
└─────────────────────────────────────────────────┘
```

## 4. double 类型的存储技巧

一个 `double` 占 8 字节 = 2 个 `uint32`。项目中已有拆分/重组方法：

```c
// 写入：double → 2×uint32
void writeDoubleToFlash1(double data1, double data2, uint8 z)
{
    uint32_t upperPart1 = (uint32_t)((*(uint64_t *)&data1) >> 32);
    uint32_t lowerPart1 = (uint32_t)((*(uint64_t *)&data1) & 0xFFFFFFFF);
    uint32_t upperPart2 = (uint32_t)((*(uint64_t *)&data2) >> 32);
    uint32_t lowerPart2 = (uint32_t)((*(uint64_t *)&data2) & 0xFFFFFFFF);
    
    flash_union_buffer[4*z].uint32_type   = upperPart1;  // data1 高32位
    flash_union_buffer[4*z+1].uint32_type = lowerPart1;  // data1 低32位
    flash_union_buffer[4*z+2].uint32_type = upperPart2;  // data2 高32位
    flash_union_buffer[4*z+3].uint32_type = lowerPart2;  // data2 低32位
}

// 读取：2×uint32 → double
double readFlash_to_double1(bool a, uint8 x)  // a=0读latitude, a=1读longitude
{
    uint32_t upperPart1, lowerPart1;
    if(a == 0) { upperPart1 = flash_union_buffer[4*x].uint32_type;
                  lowerPart1 = flash_union_buffer[4*x+1].uint32_type; }
    else       { upperPart1 = flash_union_buffer[4*x+2].uint32_type;
                  lowerPart1 = flash_union_buffer[4*x+3].uint32_type; }
    uint64_t combinedData = ((uint64_t)upperPart1 << 32) | lowerPart1;
    return *(double *)&combinedData;
}
```

**每个航点（lat + lng 两个 double）占用 4 个 uint32 = 16 字节。**

## 5. 当前项目 Flash 分区现状

| 页号 | 用途 | 使用者 |
|------|------|--------|
| 0 | INS 坐标数据（cod_saved） | Ins.c |
| 1 | 菜单参数（93 个 uint32） | menu.c |
| 10 | INS 点数量 | Ins.c |
| 25 | IMU 零偏（2 个 uint32） | ins_interface.c |
| 26~49 | 轨迹数据（ins_track） | ins_track.c |
| 0 | 元数据页（ins_track） | ins_track.c |

> ⚠️ **冲突警告**：页 0 同时被 `Ins.c` 和 `ins_track.c` 使用！这两个模块不能同时运行。

## 6. GPS 导航方案的分区规划

### 需求计算

- 最多 40 个航点
- 每个航点：lat(double) + lng(double) = 4 个 uint32 = 16 字节
- 40 个航点：40 × 4 = 160 个 uint32 = 640 字节
- 1 页容量：512 个 uint32 = 2048 字节
- **40 个航点只需 1 页！**（160 < 512）

### 推荐分区

| 页号 | 用途 | 容量 | 备注 |
|------|------|------|------|
| 0 | ⚠️ 冲突页，需统一 | — | Ins.c 和 ins_track.c 都用 |
| 1 | 菜单参数 | 93 uint32 | 已占用，不动 |
| 10 | INS 点数量 | 1 uint32 | 已占用，不动 |
| 25 | IMU 零偏 | 2 uint32 | 已占用，不动 |
| **50** | **GPS 航点数据** | **160 uint32** | **新增，存 40 个航点** |
| **51** | **GPS 元数据** | **~10 uint32** | **新增：航点数量、校准参数等** |
| 26~49 | 轨迹数据 | 24 页 | ins_track 专用，不动 |

### 页 51 元数据布局

| 偏移 | 类型 | 内容 |
|------|------|------|
| [0] | uint32 | 魔数 0x47505300 ("GPS\0") |
| [1] | uint32 | 版本号 1 |
| [2] | uint32 | 航点数量 N (0~40) |
| [3] | float | IMU-GPS 航向偏移量 |
| [4] | float | 到达判定半径 (米) |
| [5] | uint32 | 校准标志位 |
| [6~9] | 预留 | — |

## 7. 注意事项

1. **写入前必须擦除**：`flash_write_page_from_buffer` 内部会自动检查并擦除，但频繁擦写影响寿命
2. **不要频繁写 Flash**：只在"保存航点"操作时写入，不要在循环中反复写
3. **上电先读**：开机时读取页 51 的元数据，确认魔数和版本号正确后再加载航点
4. **页 0 冲突**：如果 GPS 导航和 ins_track 需要共存，必须重新分配页 0 的用途
5. **断电安全**：Flash 写入过程中断电可能导致数据损坏，建议写入后回读校验
