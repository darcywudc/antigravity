# OpenSees 隔震支座分析系统

## 概述

本系统用于分析隔震支座的静力学和动力学特性，包括：
- **静力学分析**：支座高度变化导致的反力重新分配和刚度中心偏移
- **动力学分析**：地震作用下的时程响应、位移、加速度和扭转效应

## 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                     前端 (HTML/JavaScript)                    │
│  experiments/opensees-combined.html                          │
│  - 静力学参数输入（支座刚度、高度调整）                           │
│  - 动力学参数输入（地震波类型、PGA、周期）                         │
│  - 结果可视化（图表、轨迹动画）                                   │
└─────────────────────────────────────────────────────────────┘
                              │ HTTP API
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    后端 (Python/Flask)                        │
│  opensees_server.py                                          │
│  - /api/calculate: 静力分析                                    │
│  - /api/dynamic: 动力时程分析                                   │
│  - /api/health: 健康检查                                       │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    OpenSeesPy 引擎                            │
│  - 3D刚体板模型                                                │
│  - rigidLink约束                                              │
│  - zeroLength弹簧单元                                          │
│  - Newmark时程积分                                             │
└─────────────────────────────────────────────────────────────┘
```

## 模型结构

### 静力学模型

```
         ┌─────────────────────────────────────┐
         │          刚性板 (rigidLink约束)       │  ← 上部结构
         └─────────────────────────────────────┘
              │     │     │     │     │     │
              ▼     ▼     ▼     ▼     ▼     ▼
            弹簧  弹簧  弹簧  弹簧  弹簧  弹簧
            (Kv)  (Kv)  (Kv)  (Kv)  (Kv)  (Kv)
              │     │     │     │     │     │
         ════╧═════╧═════╧═════╧═════╧═════╧════  ← 地面（固定）
```

### 动力学模型

- **质心节点 (100)**：集中质量和转动惯量
- **支座节点 (1-16)**：底部固定，顶部通过rigidLink与质心刚性连接
- **角点节点 (101-104)**：用于监测边角位移
- **弹簧单元**：X、Y方向水平刚度Kh，Z方向竖向刚度Kv

## 核心功能

### 1. 静力分析 (`/api/calculate`)

输入：
- `supports`: 支座位置和刚度
- `total_weight`: 总重量(kN)
- `heights`: 各支座高度调整(mm)

输出：
- `reactions`: 各支座反力(kN)
- `stiffness_center`: 等效刚度中心
- `eccentricity`: 偏心距
- `kh_effective`: 各支座等效水平刚度

**等效水平刚度计算**：
```python
Kh_eff = Kh_design × (R_actual / R_design)
```
支座反力越大，等效水平刚度越大，刚度中心会向该支座偏移。

### 2. 动力分析 (`/api/dynamic`)

输入：
- `supports`: 支座配置
- `ground_motion`: 地震加速度时程 {ax, ay}
- `mass_center`: 质心位置
- `dt`, `duration`: 时间参数

输出：
- `period_x`, `period_theta`: 平动/扭转周期
- `center_disp_x/y`: 质心位移时程
- `center_accel_x/y`: 质心加速度时程
- `center_rot_z`: 扭转角时程
- `bearing_disp`: 各支座变形时程
- `amplification_x/y`: 放大系数

## 支座布置

默认6个支座，2行3列：

```
     Y
     ↑
     │   支座8    支座11    支座25
     │  (-sx,+sy) (0,+sy)  (+sx,+sy)
     │
     │   支座4    支座16    支座30
     │  (-sx,-sy) (0,-sy)  (+sx,-sy)
     └────────────────────────────→ X
```

间距可配置：`sx`=X方向间距，`sy`=Y方向间距

## 单位约定

| 参数 | 输入单位 | OpenSees内部单位 |
|------|----------|-----------------|
| 坐标 (x, y) | m | m |
| 高度变化 | mm | m |
| 刚度 (Kv, Kh) | kN/m | N/m |
| 重量 | kN | N |
| 加速度 | g | m/s² |
| 位移输出 | mm | - |
| 转角输出 | mrad | - |

## 本地运行

```bash
# 激活x86环境（macOS需要Rosetta）
arch -x86_64 bash -c "source opensees_x86_env/bin/activate && python3 opensees_server.py"

# 服务启动后访问
# http://localhost:5050/api/health - 健康检查
# file:///.../experiments/opensees-combined.html - 综合分析页面
```

## API示例

### 静力分析
```javascript
fetch('http://localhost:5050/api/calculate', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
        supports: {
            '8':  { x: -0.6, y: 0.2, Kv: 5520, Kh: 55.2 },
            '11': { x: 0,    y: 0.2, Kv: 13475, Kh: 134.75 },
            // ...
        },
        total_weight: 85.58,
        heights: { '8': 2.0 }  // 支座8抬高2mm
    })
});
```

### 动力分析
```javascript
fetch('http://localhost:5050/api/dynamic', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
        supports: { /* ... */ },
        total_weight: 85.58,
        ground_motion: { ax: [...], ay: [...] },
        dt: 0.02,
        duration: 10,
        mass_center: { x: 0, y: -0.1 }  // 偏心
    })
});
```

## 云端部署

当前配置的Dockerfile需要修改以支持OpenSeesPy。由于OpenSeesPy主要支持x86架构，建议：
1. 使用x86/amd64基础镜像
2. 或考虑使用预编译的OpenSees Docker镜像

详见 `Dockerfile` 和 `cloudbuild.yaml`。
