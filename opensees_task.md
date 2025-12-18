# OpenSees 支座反力分析 - 完成

## 目标
使用OpenSeesPy计算弹性支座的反力分配。

## 当前模型

### 模型类型：简化刚性板
- 所有支座节点Z方向位移相同（板不能倾斜）
- 使用`equalDOF`约束
- `ndf=3`（只有3个平移DOF）

### 关键配置
```python
# 节点约束
ops.fix(base_node, 1, 1, 1)    # 基础节点完全固定
ops.fix(top_node, 1, 1, 0)     # 板节点X,Y固定，Z自由

# 刚性板约束
ops.equalDOF(first_node, other_node, 3)  # Z方向位移相同

# 弹簧单元
ops.element('zeroLength', eid, base_node, top_node, '-mat', mat_id, '-dir', 3)

# 高程变化建模
# h > 0: 抬高 → 弹簧压缩 → InitStrain = -h
# h < 0: 沉降 → 弹簧拉伸 → InitStrain = +|h|
ops.uniaxialMaterial('InitStrainMaterial', mat_id+100, mat_id, -h)
```

## 符号约定
- **高程变化 h**：正值=抬高，负值=沉降
- **反力 N**：正值=受压

## 验证结果

| 工况 | 8 | 11 | 25 | 4 | 16 | 30 | 总计 |
|------|-----|-----|-----|-----|-----|-----|------|
| 无变化 | 9.63 | 23.52 | 9.63 | 9.63 | 23.52 | 9.63 | 85.58 |
| 8,4抬高1mm | **13.91** | 20.49 | 8.39 | **13.91** | 20.49 | 8.39 | 85.58 |
| 8抬高1mm | **14.53** | 22.00 | 9.01 | 9.01 | 22.00 | 9.01 | 85.58 |

**验证通过：**
- ✓ 抬高支座反力增加
- ✓ 总反力=总重
- ✓ 对称性正确

## 文件
- `opensees_server.py` - Flask后端API（端口5050）
- `rubber-bearing-sim.html` - 前端（调用OpenSees API）
- `opensees_final.py` - 独立验证脚本

## 启动服务
```bash
arch -x86_64 bash
source opensees_x86_env/bin/activate
python3 opensees_server.py
```

## 限制
当前模型假设板不能倾斜，适用于对称工况验证。
如需考虑板倾斜，需要使用梁单元建模。
