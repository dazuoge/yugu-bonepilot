# -*- coding: utf-8 -*-
"""v3.1 专项：ARP 原生 IK/FK 切换不得破坏 FK 动画（只读工程、不存盘）
V31 基线 -> 无保护往返 -> 有保护往返 -> IK 下主动摆姿势
"""
import bpy
import math
import importlib
from mathutils import Quaternion

m = importlib.import_module('controller_navigator_v4971')
scn = bpy.context.scene
rig = bpy.data.objects.get('rig.016')
BONES = ['c_arm_fk.r', 'c_forearm_fk.r', 'c_hand_fk.r']
SW = 'c_hand_ik.r'
F0, F1 = 1, 70
KEYS = (1, 20, 40, 60)

fail = []
ncheck = [0]


def check(tag, cond, detail=''):
    ncheck[0] += 1
    print('V31 %-4s %s %s' % ('PASS' if cond else 'FAIL', tag, detail))
    if not cond:
        fail.append(tag)


def all_fcurves():
    act = rig.animation_data.action
    return m._bip_action_fcurves(act)


def reset_clean():
    """把右臂 FK 链重建成一条平滑、无分支跳变的动画。"""
    for bn in BONES:
        pb = rig.pose.bones[bn]
        path = m.bip_rotation_path(pb)
        n = 3 if path == 'rotation_euler' else 4
        full = 'pose.bones["%s"].%s' % (bn, path)
        chans = {}
        for fc in all_fcurves():
            if fc.data_path == full:
                chans[fc.array_index] = fc
        if any(i not in chans for i in range(n)):
            return False
        for i in range(n):
            fc = chans[i]
            # 平滑小幅变化：欧拉各轴 / 四元数各分量都在同一分支内
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
                fc.keyframe_points.insert(f, v)
            fc.update()
    bpy.context.view_layer.update()
    return True


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
    print('V31 MEASURE %-12s jumps>8deg=%3d worst=%6.2f'
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


def arp_roundtrip(frame=24, move_ik=False):
    scn.frame_set(frame)
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
    old = scn.tool_settings.use_keyframe_insert_auto
    scn.tool_settings.use_keyframe_insert_auto = True
    try:
        r1 = bpy.ops.pose.arp_switch_snap()
        v1 = float(rig.pose.bones[SW]['ik_fk_switch'])
        if move_ik:
            h = rig.pose.bones.get('c_hand_ik.r')
            if h:
                h.location[0] += 0.25
            bpy.context.view_layer.update()
        r2 = bpy.ops.pose.arp_switch_snap()
        v2 = float(rig.pose.bones[SW]['ik_fk_switch'])
    finally:
        scn.tool_settings.use_keyframe_insert_auto = old
    print('V31 ROUNDTRIP frame=%d ik=%s fk=%s log=%s'
          % (frame, v1, v2, m._BIP_IKFK_LOG))
    return v1, v2


# ------------------------------------------------- K：真实工程原样曲线
check('I1 ARP 切换算子可定位', m._bip_ikfk_find_class() is not None)
check('K1 保护包装已装载', m._bip_install_ikfk_guard())
ORIG_NAMES = m._bip_ikfk_fk_names(rig, '.r')
orig_snap = m._bip_ikfk_snapshot(rig, ORIG_NAMES)
A0 = measure('original')
m._BIP_IKFK_STATE.clear()
arp_roundtrip()
A1 = measure('original-guarded')
orig_snap2 = m._bip_ikfk_snapshot(rig, ORIG_NAMES)
for key in sorted(set(orig_snap) | set(orig_snap2)):
    aa, bb = orig_snap.get(key), orig_snap2.get(key)
    if aa == bb:
        continue
    bn = key[0].split('"')[1] if '"' in key[0] else key[0]
    print('V31 DIFF', bn, key[0].rsplit('.', 1)[-1], key[1],
          'n', len(aa or []), len(bb or []))
    for i, (xx, yy) in enumerate(zip(aa or [], bb or [])):
        ds = [j for j in range(min(len(xx), len(yy))) if xx[j] != yy[j]]
        if ds:
            print('V31   first-diff key', i, 'frame', xx[0],
                  'fields', ds, 'before', xx[ds[0]], 'after', yy[ds[0]])
            break
print('V31 NAMES', sorted(ORIG_NAMES))
check('K2 真实工程 FK 曲线逐帧完全一致', orig_snap == orig_snap2,
      'curves=%d' % len(orig_snap))
check('K3 真实工程姿态逐帧一致',
      pose_diff(A0, A1) < 0.2,
      'max=%.4f deg' % pose_diff(A0, A1))
check('K4 真实工程切换被识别为纯往返',
      '已保护' in m._BIP_IKFK_LOG, m._BIP_IKFK_LOG)

# ---------------------------------------------------------------- 基线
check('I2 FK 链重建成功', reset_clean())
A = measure('baseline')
clean_snap = m._bip_ikfk_snapshot(rig, ORIG_NAMES)
check('I3 基线动画平滑', A['n'] == 0 and A['worst'] < 8.0,
      'n=%d worst=%.2f' % (A['n'], A['worst']))

# ---------------------------------------------------------------- 无保护
m._bip_remove_ikfk_guard()
reset_clean()
arp_roundtrip()
B = measure('no-guard')
check('I4 复刻：无保护切换确实破坏动画', B['n'] > A['n'] or B['worst'] > 15.0,
      'n=%d worst=%.2f' % (B['n'], B['worst']))

# ---------------------------------------------------------------- 有保护
check('I5 保护包装可装载', m._bip_install_ikfk_guard())
m._BIP_IKFK_STATE.clear()
reset_clean()
arp_roundtrip()
C = measure('guarded')
check('I6 有保护切换后动画仍平滑', C['n'] == 0 and C['worst'] < 8.0,
      'n=%d worst=%.2f' % (C['n'], C['worst']))
clean_snap2 = m._bip_ikfk_snapshot(rig, ORIG_NAMES)
pd = pose_diff(A, C)
# 逐帧数值比较比姿态角更硬：0.05° 量级是 Euler→矩阵→四元数的求值噪声
check('I7 平滑动画 FK 曲线逐帧完全一致', clean_snap == clean_snap2,
      'curves=%d' % len(clean_snap))
check('I7b 平滑动画关键帧姿态不变', pd < 0.2, 'max=%.4f deg' % pd)
check('I8 保护日志为「已保护」', '已保护' in m._BIP_IKFK_LOG,
      m._BIP_IKFK_LOG)

# ---------------------------------------------------------------- 主动 IK
m._BIP_IKFK_STATE.clear()
reset_clean()
arp_roundtrip(move_ik=True)
check('I9 IK 下主动摆姿势时尊重 ARP',
      '保留 ARP' in m._BIP_IKFK_LOG, m._BIP_IKFK_LOG)

# ---------------------------------------------------------------- 原功能
check('J1 ARP 切换算子仍是原类',
      m._bip_ikfk_find_class().__name__ == 'ARP_OT_switch_snap')
check('J2 切换后回到 FK 模式',
      float(rig.pose.bones[SW]['ik_fk_switch']) >= 0.5)
check('J3 BIP 拖动算子仍注册',
      hasattr(bpy.ops.pose, 'cn_bip_drag_v45'))
check('J4 连续化工具仍可用',
      callable(getattr(m, 'bip_normalize_rotation_curves', None)))

print('V31 RESULT %d/%d PASS' % (ncheck[0] - len(fail), ncheck[0]))
print('V31 RESULT %s' % ('ALL_PASS' if not fail else 'FAIL'))
for t in fail:
    print('V31 FAILED ITEM', t)
print('V31 DONE')
