"""
使用PyNite验证支座反力计算

PyNite是一个3D结构分析库，我们用它来建模：
- 6个弹性支座
- 刚性连杆连接
- 施加重力和强制位移
"""

from Pynite import FEModel3D

# 支座位置
supports = {
    '8':  {'x': -0.6, 'y': 0.2, 'z': 0},
    '11': {'x': 0,    'y': 0.2, 'z': 0},
    '25': {'x': 0.6,  'y': 0.2, 'z': 0},
    '4':  {'x': -0.6, 'y': -0.2, 'z': 0},
    '16': {'x': 0,    'y': -0.2, 'z': 0},
    '30': {'x': 0.6,  'y': -0.2, 'z': 0},
}

# 支座竖向刚度 (kN/m)
Kv = {
    '8':  45000,
    '11': 70000,
    '25': 45000,
    '4':  45000,
    '16': 70000,
    '30': 45000,
}

# 总重 (kN)
W = 85.58

def analyze_with_pynite(heights_mm):
    """使用PyNite分析支座反力"""
    model = FEModel3D()
    
    # 添加节点
    for sid, pos in supports.items():
        h = heights_mm.get(sid, 0) / 1000  # mm -> m
        model.add_node(sid, pos['x'], pos['y'], h)
    
    # 添加中心节点（刚性板的中心）
    model.add_node('C', 0, 0, 0)
    
    # 添加材料（用于刚性连杆）
    E = 2.1e11  # Pa (钢材)
    G = 8.0e10  # Pa
    model.add_material('Steel', E, G, 0.3, 7850)
    
    # 添加刚性连杆从中心到各支座
    # 使用非常大的截面使其接近刚性
    A = 1.0    # m^2
    Iy = 1.0   # m^4
    Iz = 1.0   # m^4
    J = 1.0    # m^4
    
    for sid in supports.keys():
        member_name = f'M_{sid}'
        model.add_member(member_name, 'C', sid, 'Steel', Iy, Iz, J, A)
    
    # 添加支座约束（弹性支座）
    for sid, kv in Kv.items():
        # kv是kN/m，PyNite需要N/m
        model.def_support(sid, False, False, True)  # 只约束Z方向
        # PyNite没有直接的弹性支座，我们用不同的方法
        # 暂时用固定支座
    
    # 添加重力荷载
    model.add_node_load('C', 'FZ', -W)  # kN (向下)
    
    # 分析
    model.analyze()
    
    # 获取支座反力
    print("\nPyNite分析结果:")
    total = 0
    for sid in supports.keys():
        try:
            rxn = model.Nodes[sid].RxnFZ
            print(f"  支座 {sid}: RZ = {rxn:.2f} kN")
            total += rxn if rxn else 0
        except:
            print(f"  支座 {sid}: 无法获取反力")
    print(f"  总反力: {total:.2f} kN")

# 测试1: 水平状态
print("=" * 50)
print("测试: 水平状态")
print("=" * 50)
try:
    analyze_with_pynite({'8': 0, '11': 0, '25': 0, '4': 0, '16': 0, '30': 0})
except Exception as e:
    print(f"错误: {e}")

print("\n注意: PyNite不直接支持弹性支座，需要使用弹簧单元或其他方法建模。")
print("对于刚性板+弹性支座问题，手工矩阵计算可能是最直接的方法。")
