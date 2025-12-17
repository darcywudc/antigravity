"""
橡胶支座反力验证脚本
使用Python手动计算，验证刚性板+弹性支座模型

问题：刚性板坐落在6个弹性支座上，给支座施加强制位移，求反力变化。

模型：
- 刚性板：3个自由度 (z, θx, θy)
- 6个弹性支座：刚度Kv
- 强制位移：各支座高度h

平衡方程：
[K]{d} = {F}
其中 d = [z0, θx, θy]
K = Σ Kv × [1, yi, xi; yi, yi², xi×yi; xi, xi×yi, xi²]
F = [W + Σ(Kv×h), Σ(Kv×h×y), Σ(Kv×h×x)]
"""

import numpy as np

# 支座位置
supports = {
    '8':  {'x': -0.6, 'y': 0.2},
    '11': {'x': 0,    'y': 0.2},
    '25': {'x': 0.6,  'y': 0.2},
    '4':  {'x': -0.6, 'y': -0.2},
    '16': {'x': 0,    'y': -0.2},
    '30': {'x': 0.6,  'y': -0.2},
}

# 支座刚度 (kN/m)
Kv = {
    '8':  45000,
    '11': 70000,
    '25': 45000,
    '4':  45000,
    '16': 70000,
    '30': 45000,
}

# 初始反力 (kN)
initial_reactions = {
    '8':  10.0075,
    '11': 22.7747,
    '25': 10.0053,
    '4':  10.0175,
    '16': 22.7675,
    '30': 10.0153,
}

# 总重 (kN)
W = sum(initial_reactions.values())
print(f"总重 W = {W:.2f} kN")

def solve_rigid_plate(heights):
    """
    求解刚性板+弹性支座问题
    
    heights: dict, 各支座的强制位移 (mm)
    返回: dict, 各支座的反力 (kN)
    """
    # 转换为m
    h = {k: v/1000 for k, v in heights.items()}
    
    # 组装刚度矩阵和荷载向量
    # d = [z0, θx, θy]
    # di = z0 + θx×yi + θy×xi
    # Ni = Kv × (di - hi)
    # 
    # 平衡方程:
    # Σ Ni = W
    # Σ Ni×xi = 0
    # Σ Ni×yi = 0
    
    K = np.zeros((3, 3))
    F = np.zeros(3)
    
    for sid, pos in supports.items():
        x = pos['x']
        y = pos['y']
        kv = Kv[sid]
        hi = h[sid]
        
        # 刚度矩阵贡献
        K[0, 0] += kv
        K[0, 1] += kv * y
        K[0, 2] += kv * x
        K[1, 0] += kv * y
        K[1, 1] += kv * y * y
        K[1, 2] += kv * x * y
        K[2, 0] += kv * x
        K[2, 1] += kv * x * y
        K[2, 2] += kv * x * x
        
        # 荷载向量（包含重力和强制位移）
        F[0] += W/6 + kv * hi  # 简化：假设重力均匀分配，实际上应该是外荷载
        F[1] += kv * hi * y
        F[2] += kv * hi * x
    
    # 修正：荷载向量应该是 [W + Σ(Kv×h), Σ(Kv×h×y), Σ(Kv×h×x)]
    F[0] = W
    for sid, pos in supports.items():
        x = pos['x']
        y = pos['y']
        kv = Kv[sid]
        hi = h[sid]
        F[0] += kv * hi
        F[1] += kv * hi * y
        F[2] += kv * hi * x
    
    print(f"\n刚度矩阵 K:\n{K}")
    print(f"\n荷载向量 F: {F}")
    
    # 求解
    d = np.linalg.solve(K, F)
    z0, theta_x, theta_y = d
    
    print(f"\n求解结果:")
    print(f"  z0 = {z0*1000:.4f} mm (竖向位移)")
    print(f"  θx = {theta_x*1000:.4f} mrad (绕X轴转角)")
    print(f"  θy = {theta_y*1000:.4f} mrad (绕Y轴转角)")
    
    # 计算各支座反力
    reactions = {}
    total = 0
    for sid, pos in supports.items():
        x = pos['x']
        y = pos['y']
        kv = Kv[sid]
        hi = h[sid]
        
        # 板在该支座处的位移
        di = z0 + theta_x * y + theta_y * x
        
        # 支座压缩量 = 板位移 - 地基位移
        compression = di - hi
        
        # 反力
        Ni = kv * compression
        reactions[sid] = Ni
        total += Ni
        
    print(f"\n各支座反力:")
    for sid in ['8', '11', '25', '4', '16', '30']:
        init = initial_reactions[sid]
        curr = reactions[sid]
        diff = curr - init
        print(f"  {sid}: {curr:.2f} kN (初始: {init:.2f}, 变化: {diff:+.2f})")
    
    print(f"\n总反力: {total:.2f} kN (应等于 {W:.2f})")
    
    return reactions

# 测试1: 水平状态
print("=" * 60)
print("测试1: 水平状态 (所有h=0)")
print("=" * 60)
solve_rigid_plate({'8': 0, '11': 0, '25': 0, '4': 0, '16': 0, '30': 0})

# 测试2: X向倾斜
print("\n" + "=" * 60)
print("测试2: X向倾斜 (左-3, 中0, 右+3)")
print("=" * 60)
solve_rigid_plate({'8': -3, '11': 0, '25': 3, '4': -3, '16': 0, '30': 3})

# 测试3: 支座8单独下沉
print("\n" + "=" * 60)
print("测试3: 支座8单独下沉5mm")
print("=" * 60)
solve_rigid_plate({'8': -5, '11': 0, '25': 0, '4': 0, '16': 0, '30': 0})

# 测试4: 中间沉降
print("\n" + "=" * 60)
print("测试4: 中间沉降 (11和16下沉3mm)")
print("=" * 60)
solve_rigid_plate({'8': 0, '11': -3, '25': 0, '4': 0, '16': -3, '30': 0})
