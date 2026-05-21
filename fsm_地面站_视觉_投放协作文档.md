# 微型无人机第一阶段控制方案与联调接口说明

版本：v0.1  
用途：给视觉组、地面站组、投放机构组说明当前无人机控制方案、任务执行方式和 ROS 消息接口。  
适用代码：`micro_uav_stage1_fsm.py` + `stage1_mission.yaml` + 对应 launch 启动文件。  
说明：本文不讨论 ROS 包名迁移和坐标系摆放问题；实际使用时按队伍 ROS 工作空间组织文件，并按现场摆放方向执行。

---

## 1. 当前系统定位

当前无人机端的核心是一个任务状态机节点：

```text
micro_uav_stage1_fsm.py
```

它负责完成以下事情：

1. 从 YAML 读取航点、动作类型、速度、高度、降落点映射等参数。
2. 通过 MAVROS 给 PX4 发布本地位置/速度/yaw 控制指令。
3. 按比赛流程依次执行：起飞、二维码扫描、绕障、图片靶判断投放、特殊靶投放、圆环穿越、动态降落点选择。
4. 给视觉节点发布当前需要识别的目标类型。
5. 接收视觉节点返回的二维码内容和图片类别。
6. 在到达指定投放点并满足条件后发布投放命令。
7. 向地面站发布当前 FSM 状态、安全状态和降落状态。

当前代码属于“任务调度与控制框架”，视觉识别、靶心精确对准、投放机构闭环确认还需要后续节点接入。

---

## 2. 整体任务流程

本阶段任务按下面顺序执行：

```text
等待开始
  ↓
起飞到安全高度
  ↓
前往二维码点并扫描二维码
  ↓
解析二维码：目标类别1、目标类别2、降落方向 left/right
  ↓
顺时针绕障碍物飞行
  ↓
依次经过 4 个图片靶
  ↓
每到一个图片靶：下降、识别类别、判断是否属于二维码目标类别
  ↓
若匹配且该类别未投放过，则发布对应图片靶投放命令
  ↓
前往特殊靶
  ↓
靠近特殊靶后直接进入瞄准/投放动作
  ↓
前往圆环区域，后续由视觉结果修正圆环穿越路径
  ↓
根据二维码 left/right 选择正确降落点
  ↓
悬停在降落点上方，等待降落或后续自动降落逻辑
```

二维码内容格式示例：

```text
man,apple,left
```

含义：

```text
第一个图片目标类别：man
第二个图片目标类别：apple
最终降落方向：left
```

---

## 3. 状态机大状态说明

当前 FSM 大状态如下：

| 状态 | 含义 | 主要行为 |
|---|---|---|
| `WAIT_START` | 等待开始 | 地面站发布 `/uav/start=True` 后才开始任务 |
| `WAIT_FCU` | 等待飞控连接/进入任务准备 | 持续发布起飞 setpoint，必要时请求 OFFBOARD/ARM |
| `TAKEOFF` | 起飞阶段 | 起飞到 YAML 中设置的高度并等待停稳 |
| `MISSION` | 正常任务阶段 | 按 YAML 航点依次执行导航和动作 |
| `STAGE1_DONE` | 第一阶段任务结束 | 保持在最后航点，等待降落或人工确认 |
| `LANDING` | 普通降落 | 取消任务输出，进入降落流程 |
| `EMERGENCY_LAND` | 急停降落 | 收到 `/uav/stop=True` 后取消任务并降落 |
| `DISARMING` | 安全上锁 | 在允许条件下执行上锁 |
| `WAIT_RESET` | 等待复位 | 任务异常结束或降落后，必须 `/uav/reset=True` 才能重新开始 |

对外联调时，地面站最重要的是显示：

```text
/uav/fsm_state
/uav/safety_state
/uav/land_status
```

这样操作者可以知道无人机当前是在等待开始、执行任务、急停降落，还是等待复位。

---

## 4. 航点内部执行方式

每个航点不是直接“飞过去就完事”，而是分成几个内部阶段：

```text
INIT → YAW_ALIGN → MOVE → HOLD → ACTION
```

含义如下：

| 阶段 | 含义 |
|---|---|
| `INIT` | 读取当前航点，确定目标位置和目标 yaw |
| `YAW_ALIGN` | 先原地转向，避免边飞边转导致视觉方向不稳定 |
| `MOVE` | 飞向目标航点 |
| `HOLD` | 到点后保持悬停，等待速度和姿态稳定 |
| `ACTION` | 执行当前航点动作，比如扫码、识别、投放、穿环等待 |

YAML 中航点分为三种控制方式：

| 控制模式 | 用途 | 特点 |
|---|---|---|
| `position` | 普通航段 | 主要用于快速通过，精度要求不高 |
| `fusion` | 扫码/投放/精细点 | 位置 + 速度融合控制，到点后要求停稳 |
| `fusion_route` | 圆环等复杂通过点 | 位置 + 速度融合控制，但不强制长时间停稳 |

当前设计思想是：

```text
普通路段快一点，扫码/投放/圆环相关位置慢一点、稳一点。
```

---

## 5. 图片靶、特殊靶、圆环的任务理解

### 5.1 图片靶

图片靶是当前任务中难度较高的部分，因为它不是单纯飞到固定点就投放，而是要完成：

```text
飞到图片靶上方
  ↓
下降到投放高度
  ↓
视觉识别图片类别
  ↓
判断该类别是否属于二维码给出的两个目标类别
  ↓
若匹配，则根据靶面圆环中心进行瞄准
  ↓
发布投放命令
  ↓
爬升回安全高度
```

当前 FSM 已经完成“到达图片靶、识别类别、判断是否需要投放、发布投放命令”的框架。

后续视觉组需要补充“靶心定位/偏差量输出”，控制组再根据偏差量实现精确对准。

当前图片靶遍历顺序为：

```text
IMG2 → IMG1 → IMG3 → IMG4
```

每个图片靶都采用：

```text
OVER 高处到达 → DROP 下降识别/投放 → UP 原地爬升
```

这样可以避免无人机在 0.75 m 左右低空横移。

### 5.2 特殊靶

特殊靶对我们来说难度低于图片靶。

原因是特殊靶不需要判断 CIFAR-100 图片类别，也不需要判断是否属于二维码中的两个类别。当前策略是：

```text
飞到特殊靶附近固定点
  ↓
下降到投放高度
  ↓
视觉节点开始寻找特殊靶中心
  ↓
利用圆环/四角灯/靶面特征进行瞄准
  ↓
发布 special_drop 投放命令
```

也就是说，特殊靶的重点不是“是否投放”，而是“靠近后如何瞄准中心”。

当前 FSM 在 `SPECIAL_DROP` 航点会发布：

```text
/uav/scan_enable = True
/uav/scan_target = special_target
/uav/drop_cmd = special_drop
```

后续可以把这里改成：先等待视觉对准成功，再发布 `special_drop`。

### 5.3 圆环

当前 YAML 里已经给出圆环相关默认航点：

```text
RING_SEARCH_START → RING_PRE → RING_CENTER → RING_POST
```

当前版本可以用于固定圆环位置的流程测试。

后续正式逻辑应该是：

```text
到达圆环搜索起点
  ↓
视觉节点开始识别圆环
  ↓
输出圆环中心相对无人机的偏差
  ↓
FSM 根据偏差修正 RING_PRE / RING_CENTER / RING_POST
  ↓
无人机保持姿态稳定，沿圆环法向穿过
```

短期目标不是一步到位完成最优穿环，而是先完成：

```text
能识别圆环 → 能估计中心 → 能生成通过点 → 能稳定穿过
```

---

## 6. 当前 ROS 消息接口

### 6.1 地面站 → FSM

| 话题 | 类型 | 含义 |
|---|---|---|
| `/uav/start` | `std_msgs/Bool` | `True` 表示开始任务 |
| `/uav/stop` | `std_msgs/Bool` | `True` 表示急停降落 |
| `/uav/land` | `std_msgs/Bool` | `True` 表示普通降落 |
| `/uav/disarm` | `std_msgs/Bool` | `True` 表示请求安全上锁 |
| `/uav/reset` | `std_msgs/Bool` | `True` 表示复位 FSM，允许下一次 start |

建议地面站按钮：

```text
开始任务：发布 /uav/start=True
急停降落：发布 /uav/stop=True
普通降落：发布 /uav/land=True
安全上锁：发布 /uav/disarm=True
复位状态机：发布 /uav/reset=True
```

### 6.2 FSM → 地面站

| 话题 | 类型 | 含义 |
|---|---|---|
| `/uav/fsm_state` | `std_msgs/String` | 当前 FSM 大状态、导航阶段、航点名 |
| `/uav/safety_state` | `std_msgs/String` | 当前安全状态 |
| `/uav/land_status` | `std_msgs/String` | 当前降落/上锁状态 |
| `/uav/drop_cmd` | `std_msgs/String` | 当前发布的投放命令，地面站可显示记录 |

`/uav/fsm_state` 示例：

```text
MISSION | safety=START_REQUESTED | land=IDLE | phase=ACTION | wp=IMG2_DROP
```

地面站建议至少显示：

```text
当前大状态
当前航点
当前动作阶段
二维码结果
当前识别类别
已投放数量
安全状态
降落状态
```

### 6.3 FSM → 视觉节点

| 话题 | 类型 | 含义 |
|---|---|---|
| `/uav/scan_enable` | `std_msgs/Bool` | 是否启用视觉识别 |
| `/uav/scan_target` | `std_msgs/String` | 当前需要识别的目标类型 |

`/uav/scan_target` 当前取值：

```text
none            不识别
qr              识别二维码
image_target    识别图片靶
special_target  识别特殊靶
```

建议后续增加：

```text
ring_gate       识别圆环
```

视觉节点的基本逻辑：

```text
如果 /uav/scan_enable=False：可以不跑重识别，只保持相机预览或低频检测。
如果 /uav/scan_enable=True：根据 /uav/scan_target 选择对应识别算法。
```

### 6.4 视觉节点 → FSM：当前兼容接口

当前代码已经支持两个最小接口：

| 话题 | 类型 | 含义 |
|---|---|---|
| `/uav/qr_text` | `std_msgs/String` | 二维码文本，格式为 `类别1,类别2,left/right` |
| `/uav/image_class` | `std_msgs/String` | 当前图片靶识别类别 |

二维码示例：

```text
man,apple,left
```

图片类别示例：

```text
apple
```

当前 FSM 的图片靶判断逻辑：

```text
如果 image_class == qr_class_1 且类别1未投放：发布 image_drop_1
如果 image_class == qr_class_2 且类别2未投放：发布 image_drop_2
否则跳过当前图片靶
```

### 6.5 推荐扩展接口：视觉结果 JSON

为了后续做瞄准、圆环穿越和地面站显示，建议增加一个统一视觉结果话题：

```text
/uav/vision_result
类型：std_msgs/String
内容：JSON 字符串
```

这样可以避免一开始就写自定义 `.msg` 文件，方便多成员联调。

二维码结果示例：

```json
{
  "target": "qr",
  "detected": true,
  "qr_text": "man,apple,left",
  "class_1": "man",
  "class_2": "apple",
  "land_side": "left",
  "confidence": 1.0,
  "stable_count": 5
}
```

图片靶结果示例：

```json
{
  "target": "image_target",
  "detected": true,
  "class_name": "apple",
  "confidence": 0.86,
  "cx": 318,
  "cy": 242,
  "offset_x_m": -0.03,
  "offset_y_m": 0.02,
  "stable_count": 8
}
```

特殊靶结果示例：

```json
{
  "target": "special_target",
  "detected": true,
  "method": "circle_or_lights",
  "cx": 322,
  "cy": 238,
  "offset_x_m": 0.01,
  "offset_y_m": -0.02,
  "confidence": 0.90,
  "stable_count": 6
}
```

圆环结果示例：

```json
{
  "target": "ring_gate",
  "detected": true,
  "cx": 320,
  "cy": 240,
  "offset_x_m": 0.02,
  "offset_z_m": -0.04,
  "distance_m": 1.20,
  "confidence": 0.88,
  "stable_count": 6
}
```

字段解释：

| 字段 | 含义 |
|---|---|
| `target` | 当前识别目标类型，对应 `/uav/scan_target` |
| `detected` | 是否检测到目标 |
| `class_name` | 图片靶类别名 |
| `qr_text` | 原始二维码文本 |
| `land_side` | 二维码给出的降落方向 |
| `cx, cy` | 目标中心在图像中的像素坐标 |
| `offset_x_m, offset_y_m` | 靶心相对无人机投放中心的水平偏差，单位 m |
| `offset_z_m` | 圆环中心相对期望高度的偏差，单位 m |
| `distance_m` | 目标距离，主要用于圆环 |
| `confidence` | 识别置信度 |
| `stable_count` | 连续稳定识别帧数 |

---

## 7. 投放命令接口

当前 FSM 通过下面话题发布投放命令：

```text
/uav/drop_cmd
类型：std_msgs/String
```

当前取值：

| 命令 | 含义 |
|---|---|
| `image_drop_1` | 投放二维码中第一个图片类别对应的小物块 |
| `image_drop_2` | 投放二维码中第二个图片类别对应的小物块 |
| `special_drop` | 投放特殊靶对应物块 |

当前版本没有等待投放机构反馈，而是按 YAML 中的 `drop_time` 等固定时间。

后续建议投放机构组增加反馈：

```text
/uav/drop_done
类型：std_msgs/String 或 std_msgs/Bool
```

推荐 `std_msgs/String` 格式：

```text
image_drop_1_done
image_drop_2_done
special_drop_done
```

未来 FSM 可以改成：

```text
发布 drop_cmd
  ↓
等待 drop_done
  ↓
等待 hold_after_action
  ↓
进入下一个航点
```

---

## 8. 各小组分工与联调边界

### 8.1 控制组负责

```text
micro_uav_stage1_fsm.py
stage1_mission.yaml
航点顺序
飞行高度
速度参数
状态机安全逻辑
视觉/投放接口调用时机
```

控制组不直接写具体视觉算法，只负责告诉视觉组“现在该看什么”。

### 8.2 视觉组负责

```text
订阅 /uav/scan_enable
订阅 /uav/scan_target
识别二维码
识别图片靶类别
识别图片靶圆环中心
识别特殊靶中心
识别圆环中心
发布 /uav/qr_text
发布 /uav/image_class
后续发布 /uav/vision_result
```

视觉组需要保证类别名和二维码类别名完全一致，建议统一小写英文、去掉首尾空格。

### 8.3 地面站组负责

```text
发布 /uav/start
发布 /uav/stop
发布 /uav/land
发布 /uav/disarm
发布 /uav/reset
显示 /uav/fsm_state
显示 /uav/safety_state
显示 /uav/land_status
显示二维码结果
显示识别类别
显示投放命令
```

地面站必须把 `/uav/stop` 做成醒目的急停按钮。

### 8.4 投放机构组负责

```text
订阅 /uav/drop_cmd
根据命令控制对应舵机/电磁铁/机械结构
后续反馈 /uav/drop_done
```

短期内可以先不接真实投放机构，只在地面站打印命令，确认 FSM 的投放决策是否正确。

---

## 9. 当前阶段建议联调顺序

建议按下面顺序联调，不要一开始就把所有功能混在一起测：

1. 只跑 FSM 和地面站：验证 start、stop、land、reset、状态显示是否正常。
2. 接入二维码节点：验证 `/uav/qr_text` 能否被 FSM 正确解析。
3. 接入图片识别节点：验证 `/uav/image_class` 与二维码类别匹配后是否发布 `image_drop_1/image_drop_2`。
4. 不接真实投放机构，先让地面站显示 `/uav/drop_cmd`，确认投放决策正确。
5. 接入特殊靶识别，让特殊靶节点在 `special_target` 时输出中心偏差。
6. 接入圆环识别，让圆环节点在 `ring_gate` 时输出圆环中心偏差。
7. 控制组再根据视觉偏差做图片靶、特殊靶和圆环的精确对准。
8. 最后再接入真实投放机构，并增加 `/uav/drop_done` 反馈。

---

## 10. 当前最小可运行接口清单

如果队友只想先做最小联调，必须保证下面这些接口能工作：

视觉组至少发布：

```text
/uav/qr_text       std_msgs/String   例如：man,apple,left
/uav/image_class   std_msgs/String   例如：apple
```

视觉组至少订阅：

```text
/uav/scan_enable   std_msgs/Bool
/uav/scan_target   std_msgs/String
```

地面站至少发布：

```text
/uav/start         std_msgs/Bool
/uav/stop          std_msgs/Bool
/uav/land          std_msgs/Bool
/uav/reset         std_msgs/Bool
```

地面站至少显示：

```text
/uav/fsm_state
/uav/safety_state
/uav/land_status
/uav/drop_cmd
```

投放机构后续至少订阅：

```text
/uav/drop_cmd      std_msgs/String
```

---

## 11. 后续需要对 FSM 增加的功能

当前 FSM 已经能完成主流程框架，但后续为了真正比赛，需要继续补充：

1. 增加 `/uav/vision_result` 解析函数，用于接收靶心和圆环中心偏差。
2. 图片靶投放前，不仅判断类别，还要等待靶心偏差足够小。
3. 特殊靶投放前，等待特殊靶中心偏差足够小。
4. 圆环穿越前，根据圆环识别结果动态修正通过点。
5. 投放动作从固定等待时间改为等待 `/uav/drop_done`。
6. 地面站显示二维码、图片类别、当前投放目标、圆环识别状态。
7. 完整任务完成后，把降落流程从“等待人工降落”升级为“自动下降 + 落地确认 + 上锁”。

当前优先级最高的是：

```text
二维码识别 → 图片类别判断 → 投放决策 → 圆环中心识别 → 穿环路径修正
```

投放机构控制可以稍后再接入，不影响前期验证识别和任务决策逻辑。

