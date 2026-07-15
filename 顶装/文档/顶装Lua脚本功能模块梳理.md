# 顶装 Lua 脚本功能模块梳理

更新时间：2026-07-15  
脚本根目录：`顶装/脚本/lua/`

## 1. 整理结论

顶装 Lua 已按“当前入口、版本快照、离线测试”分层整理：

- `lua/` 根目录只保留各功能模块的当前最新入口，便于部署时直接识别。
- `lua/versions/<module>/` 只保存对应模块的带版本号代码快照，不放测试文件。
- `lua/tests/<module>/` 只保存对应模块的离线测试，不放生产入口或版本快照。
- 四个当前入口彼此独立，一次只能选择一个作为 DTU 任务部署，不能同时运行。

这里的“最新”是指各功能分支自己的最新版本，不表示姿态版可以无条件替代人数稳定版。

## 2. 当前目录结构

```text
顶装/脚本/lua/
├── radar_iot_task.lua                       # 人数统计当前入口：Count 1.3
├── radar_iot_task_door_exit.lua             # 门区退出当前入口：Door Exit 1.0
├── radar_iot_task_track.lua                 # 扩展轨迹当前入口：Track 2.0
├── radar_iot_task_posture.lua               # 姿态识别当前入口：Posture 3.1
├── versions/
│   ├── count/
│   │   ├── radar_iot_task_v1.2.lua
│   │   └── radar_iot_task_v1.3.lua
│   ├── door_exit/
│   │   └── radar_iot_task_door_exit_v1.0.lua
│   ├── track/
│   │   └── radar_iot_task_track_v2.0.lua
│   └── posture/
│       ├── radar_iot_task_posture_v3.0.lua
│       └── radar_iot_task_posture_v3.1.lua
└── tests/
    ├── count/
    │   ├── test_radar_iot_task.lua
    │   ├── test_track_roi_count.lua
    │   └── test_v1_3_static_hold_and_traj_dedupe.lua
    ├── door_exit/
    │   └── test_radar_iot_task_door_exit.lua
    ├── track/
    │   ├── test_top_track_extreme.lua
    │   └── test_top_track_three_person.lua
    └── posture/
        ├── test_top_posture_bed_desk_zone.lua
        ├── test_top_posture_three_person.lua
        └── test_top_posture_transitions.lua
```

## 3. 模块总览

| 模块 | 当前入口 | `pver` | 定位 | 当前状态 |
|---|---|---|---|---|
| 人数统计 | `radar_iot_task.lua` | `Radar_Top_Count_1.3` | ROI 内确认轨迹计数、静止保持、弱误报清理、重复轨迹上报抑制 | 稳定基线，默认优先部署 |
| 门区退出 | `radar_iot_task_door_exit.lua` | `Radar_Top_Door_Exit_1.0` | 在人数逻辑上增加门区退出确认、门事件辅助和门侧幽灵轨迹清理 | 独立实验版，需标定门区 |
| 扩展轨迹 | `radar_iot_task_track.lua` | `Radar_Top_Track_2.0` | 稳定软件 UID、三条完整云轨迹、路径、方向、质量和生命周期事件 | 轨迹实验版 |
| 姿态识别 | `radar_iot_task_posture.lua` | `Radar_Top_Posture_3.1` | Track 2.0 能力加站/坐/躺状态机、过道/床桌分区和姿态事件 | 最新姿态实验版，需真机标定 |

## 4. 各模块代码与版本

### 4.1 人数统计模块 `count`

当前入口：`顶装/脚本/lua/radar_iot_task.lua`

核心职责：

- 解析 `0x82/0x02` 原始轨迹，以软件确认轨迹作为 `people_count` 主来源。
- 只统计顶装 ROI 内的确认轨迹；范围外目标继续保留为诊断数据。
- 雷达 `0x86/0x0A` 实时人数和 `0x86/0x0C` 精准人数只作诊断，不覆盖 ROI 人数。
- `0x86/0x14` 门事件只记录，不直接增减人数。
- 对成熟静止轨迹最长保持 300 秒，并抑制完全相同轨迹的重复上报。

版本文件：

| 文件 | `pver` | SHA256 | 说明 |
|---|---|---|---|
| `versions/count/radar_iot_task_v1.2.lua` | `Radar_Top_Count_1.2` | `A25EC96038C090A3A86093D0029D88CA793C02F6122275CF8B67ED5B32B3F700` | 历史回退版；当前没有直接针对该快照的测试 |
| `versions/count/radar_iot_task_v1.3.lua` | `Radar_Top_Count_1.3` | `3342022DF7749B26862812A090653877EF5DB7795669081C73DA90668DE1E9D5` | 与当前入口字节一致 |

### 4.2 门区退出模块 `door_exit`

当前入口：`顶装/脚本/lua/radar_iot_task_door_exit.lua`

核心职责：

- 只有轨迹进入并满足门区条件后消失，才进入退出确认流程。
- `0x86/0x14` 门外出事件用于缩短确认，不允许一次事件批量删除多人。
- `0x86` 参考人数仅用于限制门侧幽灵轨迹，不替代软件轨迹人数。
- 当前门区为 `X=-250..-100cm`、`Y=-100..100cm`，部署前必须按现场坐标重新标定。

版本文件：

| 文件 | `pver` | SHA256 | 说明 |
|---|---|---|---|
| `versions/door_exit/radar_iot_task_door_exit_v1.0.lua` | `Radar_Top_Door_Exit_1.0` | `8154F612C181078FB468723358A61BAA8525EEB0C85633608B411AA8A850C937` | 本次补齐的固定版本副本，与当前入口字节一致 |

### 4.3 扩展轨迹模块 `track`

当前入口：`顶装/脚本/lua/radar_iot_task_track.lua`

核心职责：

- 使用软件 UID 维持跨雷达原始 ID 跳变的轨迹连续性。
- 输出 `active`、`holding`、`revived`、`removed` 等生命周期状态或事件。
- 每条云轨迹包含最近 12 个路径点、累计路径长度、速度、方向、质量和丢失时间。
- 软件单帧最多解析 23 个目标，内部轨迹表容量为 32，云端完整轨迹槽位为 3；这些数值不代表雷达硬件实测人数上限。

版本文件：

| 文件 | `pver` | SHA256 | 说明 |
|---|---|---|---|
| `versions/track/radar_iot_task_track_v2.0.lua` | `Radar_Top_Track_2.0` | `89EAFA6172059F3FA82B12E80270484758101A053D8F7725F9FE1CE04BBBCDB2` | 与当前入口字节一致 |

### 4.4 姿态识别模块 `posture`

当前入口：`顶装/脚本/lua/radar_iot_task_posture.lua`

核心职责：

- 在稳定软件 UID 上识别 `unknown`、`standing`、`sitting`、`lying`。
- 使用高度窗口、中位数、迟滞、连续命中数和持续时间抑制瞬时跳变。
- Posture 3.1 增加 `aisle` 与 `bed_desk` 分区，以及床面高度未标定保护。
- 当前 `BED_SURFACE_HEIGHT_CM=0`；床面未标定时，不对疑似上铺目标给出确定姿态。

版本文件：

| 文件 | `pver` | SHA256 | 说明 |
|---|---|---|---|
| `versions/posture/radar_iot_task_posture_v3.0.lua` | `Radar_Top_Posture_3.0` | `48E741980C81A8AE5D18EEDD64EE34A0BCC486BEE0F4621C09C0B0481E27DCAB` | 无床桌分区的回退版 |
| `versions/posture/radar_iot_task_posture_v3.1.lua` | `Radar_Top_Posture_3.1` | `ECBD9FAABD5D703716C00ECFED512E9B0D31D7D0FA5CDF6113A483F91446CFE4` | 与当前入口字节一致 |

## 5. 离线测试映射

所有命令都从 `顶装/脚本/lua/` 目录执行。

| 模块 | 测试文件 | 实际测试目标 | 主要覆盖 |
|---|---|---|---|
| 人数 | `tests/count/test_radar_iot_task.lua` | 当前 `radar_iot_task.lua`，测试内切换为 accurate 回退配置 | 精准人数确认、陈旧回退、串口失联清零 |
| 人数 | `tests/count/test_track_roi_count.lua` | 当前 `radar_iot_task.lua` | ROI 内 1 人计数，范围外 3 个目标仅诊断 |
| 人数 | `tests/count/test_v1_3_static_hold_and_traj_dedupe.lua` | 当前 `radar_iot_task.lua` | 静止保持、弱轨清理、失联清空、轨迹去重 |
| 门区 | `tests/door_exit/test_radar_iot_task_door_exit.lua` | 当前 `radar_iot_task_door_exit.lua` | 室内丢失保持、门侧删除、门事件单轨匹配、幽灵轨迹清理 |
| 轨迹 | `tests/track/test_top_track_extreme.lua` | 当前 `radar_iot_task_track.lua` | 23 目标软件解析、内部容量 32、云端 3 条完整轨迹 |
| 轨迹 | `tests/track/test_top_track_three_person.lua` | 当前 `radar_iot_task_track.lua` | 三 UID、ID 跳变、holding、revived、路径连续性 |
| 姿态 | `tests/posture/test_top_posture_three_person.lua` | `versions/posture/radar_iot_task_posture_v3.0.lua` | 三人独立姿态、滤波、holding 与恢复 |
| 姿态 | `tests/posture/test_top_posture_transitions.lua` | `versions/posture/radar_iot_task_posture_v3.0.lua` | 姿态迟滞和持续转换 |
| 姿态 | `tests/posture/test_top_posture_bed_desk_zone.lua` | 当前 `radar_iot_task_posture.lua` | 3.1 过道/床桌分区、边界、未标定上铺保护 |

单项示例：

```powershell
cd .\顶装\脚本\lua
lua .\tests\count\test_v1_3_static_hold_and_traj_dedupe.lua
lua .\tests\door_exit\test_radar_iot_task_door_exit.lua
lua .\tests\track\test_top_track_three_person.lua
lua .\tests\posture\test_top_posture_bed_desk_zone.lua
```

## 6. 四个入口的共享数据流

```text
UART1 原始字节
  → 53 59 帧头、长度、校验和、54 43 帧尾校验
  → ctrl/cmd 分发
      ├─ 0x80：存在、运动、体动
      ├─ 0x82/0x02：11 字节/目标的原始轨迹
      └─ 0x86：实时人数、精准人数、门事件
  → 模块状态机（Count / Door Exit / Track / Posture）
  → 参数筛选与 JSON 组装
  → PronetSetSendCh 上云
```

四个入口共享协议解析、基础轨迹关联、注册和参数读写框架。Track 在 Count 基础上增加完整轨迹输出；Posture 在 Track 基础上增加姿态与区域；Door Exit 是 Count 的门区退出特化分支。当前仍为独立单文件实现，不是通过 `require` 组合的运行时模块。

## 7. 部署选择

| 现场目标 | 应部署入口 | 不应混用的入口 |
|---|---|---|
| 稳定人数统计 | `radar_iot_task.lua` | 不要同时运行 Track/Posture/Door Exit |
| 验证门区退出减员 | `radar_iot_task_door_exit.lua` | 异常时回退 Count 1.3 |
| 验证三人完整轨迹 | `radar_iot_task_track.lua` | 不要覆盖 Count 1.3 固定副本 |
| 验证站/坐/躺及床桌区域 | `radar_iot_task_posture.lua` | 床面和姿态阈值未标定前不能作为准确率结论 |

部署后首先核对云端 `pver`，再确认 `radar_frame_count` 持续增长、`uart_bytes` 为完整合法帧。

## 8. 版本维护规则

1. 日常开发只修改 `lua/` 根目录对应模块的无版本号入口。
2. 发布新版本时先更新 `pver`，测试通过后再复制到 `versions/<module>/`，文件名必须带版本号。
3. 已发布的版本快照约定为不可直接修改；修复必须产生新版本文件。
4. 每个版本文件只能进入所属模块目录，禁止把 Count、Track、Posture、Door Exit 混放。
5. 测试文件只能进入 `tests/<module>/`，并在本文件测试映射表中登记实际目标。
6. 当前入口与同版本快照发布时必须校验 SHA256 一致。
7. 测试记录证明的是 Lua 解析和状态机行为，不等价于雷达硬件并发上限或现场识别准确率。

## 9. 当前已知风险

- Count 1.2 版本快照没有直接回归测试；现有通用人数测试实际加载当前 Count 1.3。
- Posture 3.0 有三人和转换测试；Posture 3.1 当前只新增了床桌分区测试，尚未单独证明 3.0 的全部三人姿态场景在 3.1 上完整继承。
- 门区坐标、床面高度、姿态阈值、Y 轴原点和雷达硬件最大轨迹数仍需真机标定。
- 四个入口存在较多相同协议和轨迹代码；本次只整理文件归属，没有改动运行逻辑，也没有抽取共享 Lua 依赖，以免改变 DTU 部署方式。

