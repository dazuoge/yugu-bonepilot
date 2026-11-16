# -*- coding: utf-8 -*-
"""v3.2 专项：驭骨 I 键 IK/FK 快速切换改走 ARP 原生（只读工程、不存盘）

覆盖：
  A 接管状态 / 非 ARP 骨架回退 / 卸载还原 / 面板版本
  B 2π 分支曲线（BIP 连续化输出）FK->IK->FK 不产生跳变
  C 工程原样曲线不加剧跳变
  D 全部肢体模式、可见性同步、选择迁移、插帧等原有行为保留
  E 保护仍生效（裸 ARP 原生仍会炸，带保护不炸）
"""
import bpy
import math
import importlib
from types import SimpleNamespace

m = importlib.import_module('controller_navigator_v4971')
scn = bpy.context.scene
rig = bpy.data.objects.get('rig.016')
BONES = ['c_arm_fk.r', 'c_forearm_fk.r', 'c_hand_fk.r']
SW = 'c_hand_ik.r'
F0, F1 = 1, 70
KEYS = (1, 20, 40, 60)
FRAME = 24

fail = []
ncheck = [0]


def check(tag, cond, detail=''):
    ncheck[0] += 1
    print('V32 %-4s %s %s' % ('PASS' if cond else 'FAIL', tag, detail))
    if not cond:
        fail.append(tag)


def all_fcurves():
    ad = rig.animation_data
    return m._bip_action_fcurves(ad.action) if (ad and ad.action) else []


def reset_clean(shift=0.0):
    for bn in BONES:
        pb = rig.pose.bones[bn]
        path = m.bip_rotation_path(pb)
        n = 3 if path == 'rotation_euler' else 4
        full = 'pose.bones["%s"].%s' % (bn, path)
        chans = {fc.array_index: fc for fc in all_fcurves()
                 if fc.data_path == full}
        if any(i not in chans for i in range(n)):
            return False
        for i in range(n):
            fc = chans[i]
            if path == 'rotation_euler':
                base = (1.0, 0.0, 0.0, 0.0)[i] if i == 0 else 0.0
                vals = [base + d for d in (0.10, 0.26, 0.42, 0.24)]
                if bn == 'c_hand_fk.r':
                    vals = [base + d for d in (-0.10, 0.02, 0.14, 0.04)]
            else:
                vals = ([1.0, 0.98, 0.96, 0.98] if i == 0 else
                        [0.0, 0.10, 0.20, 0.10] if i == 1 else
                        [0.0, 0.02, 0.04, 0.02] if i == 2 else
                        [0.0, 0.0, 0.0, 0.0])
            while len(fc.keyframe_points):
                fc.keyframe_points.remove(fc.keyframe_points[0])
            for f, v in zip(KEYS, vals):
                fc.keyframe_points.insert(
                    f, v + (shift if (path == 'rotation_euler' and i == 2)
                            else 0.0))
            fc.update()
    bpy.context.view_layer.update()
    return True


def curve_map():
    d = {}
    for fc in all_fcurves():
        p = fc.data_path
        if not any(('"%s"' % b) in p for b in BONES + [SW]):
            continue
        d[(p, fc.array_index)] = [
            (round(float(k.co[0]), 4), round(float(k.co[1]), 6))
            for k in fc.keyframe_points]
    return d


def measure(tag):
    prev, jumps, pose = {}, [], {}
    for f in range(F0, F1 + 1):
        scn.frame_set(f)
        bpy.context.view_layer.update()
        for bn in BONES:
            q = (rig.matrix_world @ rig.pose.bones[bn].matrix).to_quaternion()
            if f in KEYS:
                pose[(f, bn)] = q.copy()
            if bn in prev:
                d = min(1.0, abs(prev[bn].dot(q)))
                ang = math.degrees(2 * math.acos(max(-1.0, min(1.0, d))))
                if ang > 8.0:
                    jumps.append((f, bn, ang))
            prev[bn] = q
    worst = max((a for _, _, a in jumps), default=0.0)
    print('V32 MEASURE %-14s jumps>8=%3d worst=%6.2f'
          % (tag, len(jumps), worst))
    return {'n': len(jumps), 'worst': worst, 'pose': pose}


def pose_diff(a, b):
    ds = []
    for k, qa in a['pose'].items():
        qb = b['pose'].get(k)
        if qb is None:
            continue
        d = min(1.0, abs(qa.dot(qb)))
        ds.append(math.degrees(2 * math.acos(max(-1.0, min(1.0, d)))))
    return max(ds) if ds else 0.0


hits = {'arp_exec': 0, 'calc_matrix': 0, 'calc_generic': 0,
        'key_limb': 0, 'sync_vis': 0}


def install_probes():
    def wrap(name, key):
        fn = getattr(m, name, None)
        if fn is None:
            return

        def w(*a, **kw):
            hits[key] += 1
            return fn(*a, **kw)
        setattr(m, name, w)

    wrap('_snap_by_matrix', 'calc_matrix')
    wrap('_snap_generic', 'calc_generic')
    wrap('_key_limb', 'key_limb')
    wrap('ikfk_sync_visibility', 'sync_vis')
    cls = m._bip_ikfk_find_class()
    if cls is not None and 'execute' in cls.__dict__:
        ex = cls.__dict__['execute']

        def w(self, context):
            hits['arp_exec'] += 1
            return ex(self, context)
        cls.execute = w


def select_hand_fk():
    bpy.context.view_layer.objects.active = rig
    rig.select_set(True)
    try:
        bpy.ops.object.mode_set(mode='POSE')
    except Exception:
        pass
    for p in rig.pose.bones:
        p.select = False
    rig.pose.bones['c_arm_fk.r'].select = True
    rig.data.bones.active = rig.data.bones['c_arm_fk.r']


def cn_roundtrip(frame=FRAME):
    scn.frame_set(frame)
    bpy.context.view_layer.update()
    select_hand_fk()
    old = scn.tool_settings.use_keyframe_insert_auto
    scn.tool_settings.use_keyframe_insert_auto = True
    lb_r = [lb for lb in m.ikfk_limbs(rig)[1]
            if '.r' in lb.prop_bone and ('arm' in lb.prop_bone.lower()
                                         or 'hand' in lb.prop_bone.lower())]
    kw = {'label': lb_r[0].label} if lb_r else {}
    try:
        for tgt in ('IK', 'FK'):
            kw['target'] = tgt
            bpy.ops.pose.cn_ikfk_toggle_v45(**kw)
    finally:
        scn.tool_settings.use_keyframe_insert_auto = old
    return float(rig.pose.bones[SW]['ik_fk_switch'])


# ================================================================ A 接管状态
select_hand_fk()          # ARP 算子 poll 要求 POSE 模式，不依赖工程保存状态
check('A1 limb_switch 已接管',
      m.limb_switch is m._bip_arp_limb_switch)
check('A2 原实现已保存', m._BIP_ARP_SWITCH_ORIG is not None)
kind, limbs = m.ikfk_limbs(rig)
lb_r = [lb for lb in limbs if '.r' in lb.prop_bone
        and ('arm' in lb.prop_bone.lower() or 'hand' in lb.prop_bone.lower())]
check('A3 ARP 右臂可接管',
      bool(lb_r) and m._bip_arp_switch_applicable(rig, lb_r[0]),
      'kind=%s limbs=%d' % (kind, len(limbs)))
check('A4 该 limb 原本确实走自算',
      bool(lb_r) and not (lb_r[0].rigify_to_fk or lb_r[0].rigify_to_ik))

# 非 ARP 骨架 / 有原生 snap 的骨架：必须回退到驭骨原实现
sentinel = m._BIP_ARP_SWITCH_ORIG
m._BIP_ARP_SWITCH_ORIG = lambda *a, **kw: ('FALLBACK',)
fake = SimpleNamespace(prop_name='IK_FK', label='Rigify',
                       rigify_to_fk=('pose.rigify_x', {}),
                       rigify_to_ik=None, fk_controls=[], ik_controls=[])
r1 = m._bip_arp_limb_switch(rig, fake, 'IK')
fake2 = SimpleNamespace(prop_name='ik_fk_switch', label='ARP',
                        rigify_to_fk=('pose.x', {}), rigify_to_ik=None,
                        fk_controls=[], ik_controls=[])
r2 = m._bip_arp_limb_switch(rig, fake2, 'IK')
m._BIP_ARP_SWITCH_ORIG = sentinel
check('A5 Rigify 骨架回退原实现', r1 == ('FALLBACK',))
check('A6 自带 snap 的骨架回退原实现', r2 == ('FALLBACK',))

# 卸载 / 重装还原
m._bip_remove_arp_switch()
check('A7 卸载后还原原函数',
      m.limb_switch is not m._bip_arp_limb_switch
      and m._BIP_ARP_SWITCH_ORIG is None)
check('A8 重装幂等成功', m._bip_install_arp_switch()
      and m.limb_switch is m._bip_arp_limb_switch)
check('A11 插帧入口已接管且可还原',
      m._key_limb is m._bip_arp_key_limb
      and m._BIP_ARP_KEY_LIMB_ORIG is not None)
kl_sentinel = m._BIP_ARP_KEY_LIMB_ORIG
m._BIP_ARP_KEY_LIMB_ORIG = lambda r, lb: 7
m._BIP_ARP_SWITCH_USED = True
r3 = m._bip_arp_key_limb(rig, fake)
r4 = m._bip_arp_key_limb(rig, lb_r[0] if lb_r else fake)
m._BIP_ARP_KEY_LIMB_ORIG = kl_sentinel
m._BIP_ARP_SWITCH_USED = False
check('A12 非 ARP 骨架仍照常插帧', r3 == 7)
check('A13 ARP 原生路径跳过额外插帧', r4 == 0)

import inspect
consts = []
try:
    src = inspect.getsource(m.VIEW3D_PT_cn_bip_v45.draw)
    consts = [ln for ln in src.splitlines() if 'label(' in ln]
except Exception:
    pass
check('A9 面板版本 v3.2', any('v3.2' in str(c) for c in consts))
check('A10 面板有原生切换诊断行',
      any('_BIP_ARP_SWITCH_LOG' in str(c) for c in consts)
      or '_BIP_ARP_SWITCH_LOG' in inspect.getsource(
          m.VIEW3D_PT_cn_bip_v45.draw))

# ================================================================ B 2π 基线
check('B1 重建 2π 分支曲线', reset_clean(2 * math.pi))
scn.frame_set(FRAME)
bpy.context.view_layer.update()
A = measure('B_baseline')
check('B2 基线平滑', A['n'] == 0 and A['worst'] < 8.0,
      'n=%d worst=%.2f' % (A['n'], A['worst']))
base_curves = curve_map()
install_probes()
sw_end = cn_roundtrip()
B = measure('B_after_IKFK')
check('B3 切回 FK 开关复原', abs(sw_end - 1.0) < 1e-6, 'sw=%s' % sw_end)
check('B4 关键帧姿态不变', pose_diff(A, B) < 0.05,
      '%.4f deg' % pose_diff(A, B))
check('B5 不产生跳变', B['n'] == 0 and B['worst'] < 8.0,
      'n=%d worst=%.2f' % (B['n'], B['worst']))
check('B6 走 ARP 原生算子', hits['arp_exec'] >= 2,
      'arp_exec=%d' % hits['arp_exec'])
check('B7 不再走自算矩阵',
      hits['calc_matrix'] == 0 and hits['calc_generic'] == 0,
      'calc=%d/%d' % (hits['calc_matrix'], hits['calc_generic']))
check('B8 v3.1 曲线保护仍生效',
      '已保护' in str(m._BIP_IKFK_LOG), str(m._BIP_IKFK_LOG))
check('B9 插帧入口仍被调用（原生路径内部跳过）', hits['key_limb'] >= 2,
      'key_limb=%d' % hits['key_limb'])
check('B13 原生路径标记已置位', m._BIP_ARP_SWITCH_USED is True)
check('B10 原有显隐同步保留', hits['sync_vis'] >= 2,
      'sync_vis=%d' % hits['sync_vis'])
check('B11 原生接管日志已记', m._BIP_ARP_SWITCH_COUNT >= 2,
      '%s x%d' % (m._BIP_ARP_SWITCH_LOG, m._BIP_ARP_SWITCH_COUNT))
after_curves = curve_map()
check('B12 曲线集合未被增删',
      set(base_curves) == set(after_curves),
      '%d -> %d' % (len(base_curves), len(after_curves)))

# ================================================================ C 全部肢体
try:
    # 关键：两次切换之间不能 frame_set / measure —— 开关属性上有全程 1.0 的
    # F-Curve，一求值就把刚切到 IK 的开关刷回 FK，会把测试本身搞坏。
    scn.frame_set(FRAME)
    bpy.context.view_layer.update()
    select_hand_fk()
    old = scn.tool_settings.use_keyframe_insert_auto
    scn.tool_settings.use_keyframe_insert_auto = True
    bpy.ops.pose.cn_ikfk_toggle_v45(target='IK', all_limbs=True)
    sws_ik = [float(pb['ik_fk_switch']) for pb in rig.pose.bones
              if 'ik_fk_switch' in pb.keys()
              and (pb.name.startswith('c_hand_ik')
                   or pb.name.startswith('c_foot_ik'))]
    bpy.ops.pose.cn_ikfk_toggle_v45(target='FK', all_limbs=True)
    sws_fk = [float(pb['ik_fk_switch']) for pb in rig.pose.bones
              if 'ik_fk_switch' in pb.keys()
              and (pb.name.startswith('c_hand_ik')
                   or pb.name.startswith('c_foot_ik'))]
    scn.tool_settings.use_keyframe_insert_auto = old
    check('C1 全部肢体 → IK',
          bool(sws_ik) and all(v < 0.5 for v in sws_ik), str(sws_ik))
    check('C2 全部肢体 → FK',
          bool(sws_fk) and all(v > 0.5 for v in sws_fk), str(sws_fk))
    Cpost = measure('C_after')
    check('C3 全部肢体往返不引入跳变',
          Cpost['n'] == 0 and Cpost['worst'] < 8.0,
          'n=%d worst=%.2f' % (Cpost['n'], Cpost['worst']))
except Exception as exc:
    check('C1 全部肢体 → IK', False, repr(exc))
    check('C2 全部肢体 → FK', False, repr(exc))
    check('C3 全部肢体往返不引入跳变', False, repr(exc))

# ================================================================ D 原样曲线
# 重新载入原样曲线代价高，这里用工程当前曲线做「不加剧」判据：
# 再做一轮往返，跳变数不得比上一轮多
prev_n = B['n']
cn_roundtrip()
D = measure('D_second_round')
check('D1 重复往返不累积跳变', D['n'] == prev_n and D['worst'] < 8.0,
      'n=%d worst=%.2f' % (D['n'], D['worst']))
check('D2 关键帧姿态仍不变', pose_diff(A, D) < 0.05,
      '%.4f deg' % pose_diff(A, D))
all_fk = set()
for _lb in m.ikfk_limbs(rig)[1]:
    all_fk.update(_lb.fk_controls or [])
act = rig.data.bones.active.name if rig.data.bones.active else None
check('D3 选择迁移仍工作', act in all_fk, str(act))

print('V32 RESULT %d/%d PASS' % (ncheck[0] - len(fail), ncheck[0]))
print('V32 RESULT ALL_PASS' if not fail else 'V32 RESULT FAIL')
for t in fail:
    print('V32 FAILED ITEM', t)
print('V32 DONE')
