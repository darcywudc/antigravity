#!/usr/bin/env python3
"""
OpenSees静力与动力计算后端服务

提供REST API供HTML前端调用，计算支座反力分配和时程响应。
"""
import sys
import os

# 先启动Flask，延迟导入OpenSeesPy
print("Starting server...", flush=True)

from flask import Flask, request, jsonify, send_from_directory
from flask_cors import CORS
import math

app = Flask(__name__, static_folder='experiments')
CORS(app)

# 延迟导入OpenSeesPy
ops = None

def get_ops():
    """延迟加载OpenSeesPy"""
    global ops
    if ops is None:
        print("Loading OpenSeesPy...", flush=True)
        import openseespy.opensees as _ops
        ops = _ops
        print("OpenSeesPy loaded successfully", flush=True)
    return ops


# 静态文件服务
@app.route('/')
def index():
    return send_from_directory('.', 'index.html')


@app.route('/experiments/<path:filename>')
def experiments(filename):
    return send_from_directory('experiments', filename)


def analyze_static(supports, total_weight, heights_mm):
    """
    使用OpenSees进行静力分析
    
    简化模型：所有支座节点Z方向位移相同（板不倾斜）
    
    Args:
        supports: dict, 支座配置 {id: {'x': x, 'y': y, 'Kv': kv}}
        total_weight: float, 总重 kN
        heights_mm: dict, 高程变化 {id: delta_mm}，正值=抬高，负值=沉降
    
    Returns:
        dict, 反力结果 {id: reaction_kN}
    """
    ops = get_ops()  # 延迟加载OpenSeesPy
    ops.wipe()
    ops.model('basic', '-ndm', 3, '-ndf', 3)
    
    base_nodes = {}
    top_nodes = {}
    nid = 1
    
    for sid, p in supports.items():
        # 基础节点（固定）
        base_nodes[sid] = nid
        ops.node(nid, p['x'], p['y'], 0)
        ops.fix(nid, 1, 1, 1)
        nid += 1
        
        # 板上节点
        top_nodes[sid] = nid
        ops.node(nid, p['x'], p['y'], 0)
        ops.fix(nid, 1, 1, 0)  # X,Y固定，Z自由
        nid += 1
    
    # 刚性板约束：所有Z方向位移相同
    first_top = list(top_nodes.values())[0]
    for tn in list(top_nodes.values())[1:]:
        ops.equalDOF(first_top, tn, 3)
    
    # 弹簧单元
    eid = 1
    for sid, p in supports.items():
        kv = p['Kv'] * 1000  # kN/m -> N/m
        h = heights_mm.get(sid, 0) / 1000  # mm -> m
        # h > 0: 抬高 -> 弹簧压缩 -> InitStrain为负
        # h < 0: 沉降 -> 弹簧拉伸 -> InitStrain为正
        
        if h != 0:
            ops.uniaxialMaterial('Elastic', eid, kv)
            ops.uniaxialMaterial('InitStrainMaterial', eid+100, eid, -h)
            ops.element('zeroLength', eid, base_nodes[sid], top_nodes[sid],
                        '-mat', eid+100, '-dir', 3)
        else:
            ops.uniaxialMaterial('Elastic', eid, kv)
            ops.element('zeroLength', eid, base_nodes[sid], top_nodes[sid],
                        '-mat', eid, '-dir', 3)
        eid += 1
    
    # 荷载
    ops.timeSeries('Linear', 1)
    ops.pattern('Plain', 1, 1)
    ops.load(first_top, 0, 0, -total_weight * 1000)
    
    # 分析
    ops.system('BandGeneral')
    ops.constraints('Transformation')
    ops.numberer('RCM')
    ops.algorithm('Linear')
    ops.integrator('LoadControl', 1)
    ops.analysis('Static')
    
    if ops.analyze(1) != 0:
        return None
    
    ops.reactions()
    
    reactions = {}
    for sid in supports:
        r = ops.nodeReaction(base_nodes[sid])
        reactions[sid] = r[2] / 1000  # N -> kN
    
    displacement = ops.nodeDisp(first_top)[2] * 1000  # m -> mm
    
    # 计算名义刚度中心（基于设计Kv，不随高度变化）
    sum_kx = 0
    sum_ky = 0
    sum_k = 0
    for sid, p in supports.items():
        kv = p['Kv']
        sum_kx += kv * p['x']
        sum_ky += kv * p['y']
        sum_k += kv
    
    nominal_stiffness_center = {
        'x': sum_kx / sum_k if sum_k > 0 else 0,
        'y': sum_ky / sum_k if sum_k > 0 else 0
    }
    
    # 计算等效水平刚度Kh_eff
    # 假设: Kh与竖向荷载成正比 (线性关系)
    # Kh_eff = Kh_design × (R_actual / R_design)
    # 其中 R_design = 总重 / 支座数 × Kv_i / Kv_avg
    total_weight_calc = sum(reactions.values())  # 实际总竖向反力
    total_kv = sum(p['Kv'] for p in supports.values())
    
    kh_effective = {}
    for sid, p in supports.items():
        # 设计反力 = 总重 × (Kv_i / 总Kv)
        design_reaction = total_weight_calc * (p['Kv'] / total_kv) if total_kv > 0 else 0
        actual_reaction = reactions[sid]
        
        # 等效Kh = 设计Kh × (实际反力 / 设计反力)
        if design_reaction > 0 and actual_reaction > 0:
            ratio = actual_reaction / design_reaction
            kh_effective[sid] = p['Kh'] * ratio
        else:
            kh_effective[sid] = 0  # 如果反力为0或负，Kh_eff = 0
    
    # 计算基于等效Kh的刚度中心（这个会随高度变化）
    sum_khx = 0
    sum_khy = 0
    sum_kh = 0
    for sid, p in supports.items():
        kh = kh_effective[sid]
        sum_khx += kh * p['x']
        sum_khy += kh * p['y']
        sum_kh += kh
    
    effective_stiffness_center = {
        'x': sum_khx / sum_kh if sum_kh > 0 else 0,
        'y': sum_khy / sum_kh if sum_kh > 0 else 0
    }
    
    # 偏心 = 等效刚度中心相对于几何中心(0,0)的距离
    eccentricity = {
        'x': effective_stiffness_center['x'],
        'y': effective_stiffness_center['y'],
        'magnitude': math.sqrt(effective_stiffness_center['x']**2 + effective_stiffness_center['y']**2)
    }
    
    return {
        'reactions': reactions,
        'displacement_mm': displacement,
        'kh_effective': kh_effective,
        'total_kh_effective': sum_kh,
        'nominal_stiffness_center': nominal_stiffness_center,
        'stiffness_center': effective_stiffness_center,  # 用于兼容
        'eccentricity': eccentricity
    }


@app.route('/api/calculate', methods=['POST'])
def calculate():
    """静力计算API"""
    try:
        data = request.get_json()
        
        supports = data.get('supports', {})
        total_weight = data.get('total_weight', 85.58)
        heights = data.get('heights', {})
        
        result = analyze_static(supports, total_weight, heights)
        
        if result:
            return jsonify({
                'success': True,
                **result
            })
        else:
            return jsonify({
                'success': False,
                'error': 'Analysis failed'
            }), 500
            
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500


def analyze_dynamic(supports, total_weight, heights_mm, ground_motion, dt, duration, mass_center=(0, 0)):
    """
    OpenSees动力学时程分析 - 考虑偏心效应
    
    模型：3DOF系统（X,Y平动 + Z轴转动）
    考虑质心与刚度中心不重合导致的平动-扭转耦合
    
    Args:
        supports: dict, 支座配置 {id: {'x': x, 'y': y, 'Kv': kv, 'Kh': kh}}
        total_weight: float, 总重 kN
        heights_mm: dict, 高程变化
        ground_motion: dict, {'ax': [...], 'ay': [...]} 地震加速度时程 (g)
        dt: float, 时间步长 (s)
        duration: float, 分析持续时间 (s)
        mass_center: tuple, 质心位置 (x, y) in m
    
    Returns:
        dict, 时程响应结果
    """
    g = 9.81  # m/s²
    mass = total_weight / g * 1000  # kg
    
    # 计算刚度中心
    sum_kx, sum_ky, sum_k = 0, 0, 0
    for sid, p in supports.items():
        kh = p.get('Kh', p['Kv'] * 0.01)
        sum_kx += kh * p['x']
        sum_ky += kh * p['y']
        sum_k += kh
    
    stiffness_center = (sum_kx / sum_k, sum_ky / sum_k) if sum_k > 0 else (0, 0)
    
    # 偏心距
    ex = mass_center[0] - stiffness_center[0]
    ey = mass_center[1] - stiffness_center[1]
    eccentricity = math.sqrt(ex**2 + ey**2)
    
    # 总水平刚度 kN/m -> N/m
    total_kh = sum_k * 1000
    
    # 扭转刚度 J = Σ(Kh × r²)，r是到刚度中心的距离
    J = 0
    for sid, p in supports.items():
        kh = p.get('Kh', p['Kv'] * 0.01) * 1000  # N/m
        dx = p['x'] - stiffness_center[0]
        dy = p['y'] - stiffness_center[1]
        r2 = dx**2 + dy**2
        J += kh * r2
    
    # 转动惯量（假设矩形板）
    Lx, Ly = 1.2, 0.4  # 板尺寸 m
    I_mass = mass * (Lx**2 + Ly**2) / 12  # kg·m²
    
    # 自振频率
    omega_x = math.sqrt(total_kh / mass)  # 平动
    omega_theta = math.sqrt(J / I_mass) if J > 0 and I_mass > 0 else omega_x
    
    period_x = 2 * math.pi / omega_x
    period_theta = 2 * math.pi / omega_theta if omega_theta > 0 else 0
    
    # ========== OpenSees模型 ==========
    # 使用真实的多支座空间模型
    ops = get_ops()  # 延迟加载OpenSeesPy
    ops.wipe()
    ops.model('basic', '-ndm', 3, '-ndf', 6)
    
    # 节点编号：
    # 1-6: 支座底部节点（固定）
    # 11-16: 支座顶部节点
    # 100: 质心主节点
    # 101-104: 角点节点
    
    support_list = list(supports.items())
    base_nodes = {}
    top_nodes = {}
    
    # 创建支座节点
    for i, (sid, p) in enumerate(support_list):
        base_id = i + 1
        top_id = i + 11
        base_nodes[sid] = base_id
        top_nodes[sid] = top_id
        
        ops.node(base_id, p['x'], p['y'], 0)
        ops.fix(base_id, 1, 1, 1, 1, 1, 1)
        
        ops.node(top_id, p['x'], p['y'], 0.1)
    
    # 质心节点
    ops.node(100, mass_center[0], mass_center[1], 0.1)
    ops.mass(100, mass, mass, mass/10, I_mass, I_mass, I_mass)
    
    # 角点节点
    corners = [
        (-0.6, 0.2),   # 101: 左上
        (0.6, 0.2),    # 102: 右上
        (-0.6, -0.2),  # 103: 左下
        (0.6, -0.2)    # 104: 右下
    ]
    for i, (cx, cy) in enumerate(corners):
        ops.node(101 + i, cx, cy, 0.1)
    
    # 创建支座弹簧
    mat_id = 1
    for sid, p in supports.items():
        kh = p.get('Kh', p['Kv'] * 0.01) * 1000  # kN/m -> N/m
        kv = p['Kv'] * 1000
        
        ops.uniaxialMaterial('Elastic', mat_id, kh)      # X
        ops.uniaxialMaterial('Elastic', mat_id+1, kh)    # Y
        ops.uniaxialMaterial('Elastic', mat_id+2, kv)    # Z
        
        ops.element('zeroLength', mat_id, base_nodes[sid], top_nodes[sid],
                    '-mat', mat_id, mat_id+1, mat_id+2, '-dir', 1, 2, 3)
        mat_id += 3
    
    # 刚性连接所有顶部节点和质心（使用rigidLink）
    for sid in supports:
        ops.rigidLink('beam', 100, top_nodes[sid])
    
    # 刚性连接角点到质心
    for i in range(4):
        ops.rigidLink('beam', 100, 101 + i)
    
    # 约束质心的Z位移（简化）
    # ops.fix(100, 0, 0, 1, 0, 0, 0)  # 不约束，让其自由
    
    # 地震动
    ax = ground_motion.get('ax', [0] * int(duration / dt))
    ay = ground_motion.get('ay', [0] * int(duration / dt))
    n_steps = min(len(ax), len(ay), int(duration / dt))
    
    # X方向
    ops.timeSeries('Path', 1, '-dt', dt, '-values', *[a * g for a in ax[:n_steps]])
    ops.pattern('UniformExcitation', 1, 1, '-accel', 1)
    
    # Y方向
    ops.timeSeries('Path', 2, '-dt', dt, '-values', *[a * g for a in ay[:n_steps]])
    ops.pattern('UniformExcitation', 2, 2, '-accel', 2)
    
    # Rayleigh阻尼（5%）
    zeta = 0.05
    alpha = 2 * zeta * omega_x
    ops.rayleigh(alpha, 0, 0, 0)
    
    # 分析设置
    ops.system('BandGeneral')
    ops.constraints('Transformation')
    ops.numberer('RCM')
    ops.algorithm('Newton')
    ops.integrator('Newmark', 0.5, 0.25)
    ops.analysis('Transient')
    
    # 记录响应 - 直接从OpenSees获取原始数据
    time_history = []
    
    # 质心节点(100)的响应
    center_disp_x, center_disp_y, center_disp_z = [], [], []
    center_rot_x, center_rot_y, center_rot_z = [], [], []
    center_accel_x, center_accel_y, center_accel_z = [], [], []
    
    # 角点节点(101-104)的响应
    corner_disp = {101: [], 102: [], 103: [], 104: []}
    
    # 支座变形（顶部节点相对于底部节点的位移）
    # 底部节点固定，所以支座变形就是顶部节点位移
    bearing_disp = {}
    for sid in supports:
        bearing_disp[sid] = []
    
    # 地面加速度输入
    input_accel_x, input_accel_y = [], []
    
    t = 0
    for i in range(n_steps):
        ok = ops.analyze(1, dt)
        if ok != 0:
            print(f'Analysis failed at step {i}')
            break
        
        t += dt
        time_history.append(t)
        
        # 质心节点位移/转角（OpenSees原始值: 相对位移，单位m, rad）
        d = ops.nodeDisp(100)
        center_disp_x.append(d[0] * 1000)  # m -> mm
        center_disp_y.append(d[1] * 1000)
        center_disp_z.append(d[2] * 1000)
        center_rot_x.append(d[3] * 1000)  # rad -> mrad
        center_rot_y.append(d[4] * 1000)
        center_rot_z.append(d[5] * 1000)
        
        # 质心节点加速度（OpenSees原始值: 相对加速度，单位m/s²）
        a = ops.nodeAccel(100)
        # 绝对加速度 = 相对加速度 + 地面加速度
        ag_x = ax[i] * g if i < len(ax) else 0
        ag_y = ay[i] * g if i < len(ay) else 0
        center_accel_x.append((a[0] + ag_x) / g)  # -> g
        center_accel_y.append((a[1] + ag_y) / g)
        center_accel_z.append(a[2] / g)
        
        # 角点节点位移
        for cid in [101, 102, 103, 104]:
            cd = ops.nodeDisp(cid)
            corner_disp[cid].append({
                'x': cd[0] * 1000,  # mm
                'y': cd[1] * 1000,
                'z': cd[2] * 1000
            })
        
        # 支座变形（顶部节点位移 = 支座变形，因为底部固定）
        for sid in supports:
            top_id = top_nodes[sid]
            td = ops.nodeDisp(top_id)
            bearing_disp[sid].append({
                'x': td[0] * 1000,  # mm
                'y': td[1] * 1000,
                'z': td[2] * 1000,
                'mag': math.sqrt(td[0]**2 + td[1]**2) * 1000
            })
        
        # 记录地面加速度输入
        input_accel_x.append(ax[i] if i < len(ax) else 0)
        input_accel_y.append(ay[i] if i < len(ay) else 0)
    
    # 峰值响应 - 使用新变量名
    max_disp_x = max(abs(d) for d in center_disp_x) if center_disp_x else 0
    max_disp_y = max(abs(d) for d in center_disp_y) if center_disp_y else 0
    max_disp_z = max(abs(d) for d in center_disp_z) if center_disp_z else 0
    max_rot_z = max(abs(r) for r in center_rot_z) if center_rot_z else 0
    max_disp = math.sqrt(max_disp_x**2 + max_disp_y**2)
    
    max_accel_x = max(abs(a) for a in center_accel_x) if center_accel_x else 0
    max_accel_y = max(abs(a) for a in center_accel_y) if center_accel_y else 0
    max_accel = math.sqrt(max_accel_x**2 + max_accel_y**2)
    
    # 角点最大位移
    corner_max = {}
    for cid in [101, 102, 103, 104]:
        if corner_disp[cid]:
            mx = max(abs(d['x']) for d in corner_disp[cid])
            my = max(abs(d['y']) for d in corner_disp[cid])
            corner_max[cid] = {'x': mx, 'y': my, 'mag': math.sqrt(mx**2 + my**2)}
        else:
            corner_max[cid] = {'x': 0, 'y': 0, 'mag': 0}
    
    # 支座变形峰值
    bearing_max = {}
    for sid in supports:
        if bearing_disp[sid]:
            mx = max(abs(d['x']) for d in bearing_disp[sid])
            my = max(abs(d['y']) for d in bearing_disp[sid])
            mz = max(abs(d['z']) for d in bearing_disp[sid])
            mm = max(d['mag'] for d in bearing_disp[sid])
            bearing_max[sid] = {'x': mx, 'y': my, 'z': mz, 'mag': mm}
        else:
            bearing_max[sid] = {'x': 0, 'y': 0, 'z': 0, 'mag': 0}
    
    # PGA和放大系数
    pga_x = max(abs(a) for a in ax) if ax else 0.001
    pga_y = max(abs(a) for a in ay) if ay else 0.001
    amp_x = max_accel_x / pga_x if pga_x > 0 else 1
    amp_y = max_accel_y / pga_y if pga_y > 0 else 1
    
    # 周期比
    period_ratio = period_theta / period_x if period_x > 0 else 1
    
    return {
        'success': True,
        'time': time_history,
        
        # 质心位移（OpenSees相对位移）
        'center_disp_x': center_disp_x,
        'center_disp_y': center_disp_y,
        'center_disp_z': center_disp_z,
        'center_rot_x': center_rot_x,
        'center_rot_y': center_rot_y,
        'center_rot_z': center_rot_z,
        
        # 质心加速度（绝对加速度）
        'center_accel_x': center_accel_x,
        'center_accel_y': center_accel_y,
        'center_accel_z': center_accel_z,
        
        # 角点位移
        'corner_disp': corner_disp,
        'corner_max': corner_max,
        
        # 地面加速度输入
        'input_accel_x': input_accel_x,
        'input_accel_y': input_accel_y,
        
        # 支座变形
        'bearing_disp': bearing_disp,
        'bearing_max': bearing_max,
        
        # 峰值
        'max_disp_x': max_disp_x,
        'max_disp_y': max_disp_y,
        'max_disp_z': max_disp_z,
        'max_disp': max_disp,
        'max_rot_z': max_rot_z,
        'max_accel_x': max_accel_x,
        'max_accel_y': max_accel_y,
        'max_accel': max_accel,
        'pga_x': pga_x,
        'pga_y': pga_y,
        'amplification_x': amp_x,
        'amplification_y': amp_y,
        
        # 动力特性（用周期表示）
        'period_x': period_x,
        'period_theta': period_theta,
        'period_ratio': period_ratio,
        'total_kh': total_kh / 1000,  # kN/m
        'torsional_stiffness': J / 1000,  # kN·m/rad
        'mass': mass,  # kg
        'I_mass': I_mass,  # kg·m²
        
        # 偏心
        'stiffness_center': {'x': stiffness_center[0], 'y': stiffness_center[1]},
        'mass_center': {'x': mass_center[0], 'y': mass_center[1]},
        'eccentricity': {'x': ex, 'y': ey, 'magnitude': eccentricity}
    }


def compute_response_spectrum(accel_g, dt, periods, zeta=0.05):
    """
    计算加速度反应谱
    
    Args:
        accel_g: list, 地面加速度时程 (g)
        dt: float, 时间步长 (s)
        periods: list, 周期点 (s)
        zeta: float, 阻尼比 (默认5%)
    
    Returns:
        dict, 反应谱 {Sa: [...], Sd: [...], Sv: [...]}
    """
    g = 9.81
    accel = [a * g for a in accel_g]  # g -> m/s²
    n = len(accel)
    
    Sa = []  # 加速度谱 (g)
    Sd = []  # 位移谱 (mm)
    Sv = []  # 速度谱 (mm/s)
    
    for T in periods:
        if T <= 0:
            Sa.append(max(abs(a) for a in accel_g) if accel_g else 0)
            Sd.append(0)
            Sv.append(0)
            continue
        
        omega = 2 * math.pi / T
        omega_d = omega * math.sqrt(1 - zeta**2)
        
        # Newmark-beta积分
        u, v, a_resp = 0, 0, 0
        max_u, max_v, max_a = 0, 0, 0
        
        gamma, beta = 0.5, 0.25
        k = omega**2
        c = 2 * zeta * omega
        
        a1 = 1 / (beta * dt**2) + gamma * c / (beta * dt)
        a2 = 1 / (beta * dt) + (gamma / beta - 1) * c
        a3 = (1 / (2 * beta) - 1) + dt * c * (gamma / (2 * beta) - 1)
        k_eff = k + a1
        
        for i in range(1, n):
            p_eff = -accel[i] + a1 * u + a2 * v + a3 * a_resp
            u_new = p_eff / k_eff
            v_new = gamma / (beta * dt) * (u_new - u) + (1 - gamma / beta) * v + dt * (1 - gamma / (2 * beta)) * a_resp
            a_new = (u_new - u) / (beta * dt**2) - v / (beta * dt) - (1 / (2 * beta) - 1) * a_resp
            
            u, v, a_resp = u_new, v_new, a_new
            
            # 绝对加速度 = 相对加速度 + 地面加速度
            a_abs = a_resp + accel[i]
            
            if abs(u) > max_u: max_u = abs(u)
            if abs(v) > max_v: max_v = abs(v)
            if abs(a_abs) > max_a: max_a = abs(a_abs)
        
        Sa.append(max_a / g)  # m/s² -> g
        Sd.append(max_u * 1000)  # m -> mm
        Sv.append(max_v * 1000)  # m/s -> mm/s
    
    return {'periods': periods, 'Sa': Sa, 'Sd': Sd, 'Sv': Sv}


@app.route('/api/spectrum', methods=['POST'])
def response_spectrum():
    """反应谱分析API"""
    try:
        data = request.get_json()
        
        accel_x = data.get('accel_x', [])
        accel_y = data.get('accel_y', [])
        dt = data.get('dt', 0.01)
        periods = data.get('periods', [0.05, 0.1, 0.2, 0.3, 0.5, 0.7, 1.0, 1.5, 2.0, 3.0])
        zeta = data.get('zeta', 0.05)
        
        spectrum_x = compute_response_spectrum(accel_x, dt, periods, zeta)
        spectrum_y = compute_response_spectrum(accel_y, dt, periods, zeta)
        
        return jsonify({
            'success': True,
            'periods': periods,
            'spectrum_x': spectrum_x,
            'spectrum_y': spectrum_y
        })
        
    except Exception as e:
        import traceback
        return jsonify({
            'success': False,
            'error': str(e),
            'traceback': traceback.format_exc()
        }), 500


@app.route('/api/dynamic', methods=['POST'])
def dynamic_analysis():
    """动力学时程分析API"""
    try:
        data = request.get_json()
        
        supports = data.get('supports', {})
        total_weight = data.get('total_weight', 85.58)
        heights = data.get('heights', {})
        ground_motion = data.get('ground_motion', {'ax': [], 'ay': []})
        dt = data.get('dt', 0.01)
        duration = data.get('duration', 10.0)
        mass_center = data.get('mass_center', (0, 0))
        if isinstance(mass_center, dict):
            mass_center = (mass_center.get('x', 0), mass_center.get('y', 0))
        
        result = analyze_dynamic(supports, total_weight, heights, ground_motion, dt, duration, mass_center)
        
        return jsonify(result)
            
    except Exception as e:
        import traceback
        return jsonify({
            'success': False,
            'error': str(e),
            'traceback': traceback.format_exc()
        }), 500



@app.route('/api/health', methods=['GET'])
def health():
    return jsonify({
        'status': 'ok', 
        'engine': 'OpenSeesPy', 
        'features': ['static', 'dynamic', 'spectrum']
    })


if __name__ == '__main__':
    # Cloud Run使用PORT环境变量，本地默认5050
    port = int(os.environ.get('PORT', 5050))
    
    print('='*60)
    print('  OpenSees计算后端服务')
    print(f'  端口: {port}')
    print(f'  静力分析: http://localhost:{port}/api/calculate')
    print(f'  动力分析: http://localhost:{port}/api/dynamic')
    print('='*60)
    
    # Cloud Run不支持debug模式
    debug = os.environ.get('K_SERVICE') is None  # K_SERVICE在Cloud Run中自动设置
    app.run(host='0.0.0.0', port=port, debug=debug)
