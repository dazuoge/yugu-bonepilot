# -*- coding: utf-8 -*-
"""v2.6 回归：真实工程 E 踏步回斩.blend 上复刻「画弧多帧暴跳」。

只读打开原文件（绝不 save_mainfile），验证：
  A 旧工具漏掉的 f40（只有 W/Z 有键的符号翻转帧）现在能被检查/修复发现；
  B BIP 在 44/47/51/55 连续画弧插帧后，自动把邻域分支理顺；
  C 30~50 帧的逐帧姿态跳变与手部轨迹突变消失；
  D 所有原有关键帧姿态完全不变、旋转模式/位置/约束/IK 开关不变。
"""
import sys, os, math, importlib
import bpy
from mathutils import Vector, Quaternion

try:
    sys.stdout.reconfigure(encoding='utf-8')
except Exception:
    pass

m = importlib.import_module('controller_navigator_v4971')
sc = bpy.context.scene
rig = bpy.data.objects['rig.016']
act = rig.animation_data.action
print('V26 FILE', bpy.data.filepath, '| rig', rig.name, '| act', act.name,
      '| frame', sc.frame_current, '| AUTOKEY',
      sc.tool_settings.use_keyframe_insert_auto)

NAMES = ('c_arm_fk.r', 'c_forearm_fk.r', 'c_hand_fk.r')
ALL = ('c_arm_fk.r', 'c_forearm_fk.r', 'c_hand_fk.r',
       'c_arm_fk.l', 'c_forearm_fk.l', 'c_hand_fk.l')
PASS = []


def check(tag, ok, extra=''):
    PASS.append((tag, bool(ok)))
    print('V26 %-28s %s %s' % (tag, 'PASS' if ok else 'FAIL', extra))


def all_fcurves():
    out = []
    for layer in act.layers:
        for strip in layer.strips:
            for bag in strip.channelbags:
                out.extend(list(bag.fcurves))
    return out


def chans(name):
    p = rig.pose.bones[name]
    tail = m.bip_rotation_path(p)
    pref = 'pose.bones["%s"].%s' % (name, tail)
    n = 3 if tail == 'rotation_euler' else 4
    d = {}
    for fc in all_fcurves():
        if fc.data_path == pref:
            d[fc.array_index] = fc
    if any(j not in d for j in range(n)):
        return None
    return [d[j] for j in range(n)]


def keyframes(name):
    cs = chans(name)
    return sorted({round(k.co.x, 3) for fc in cs
                   for k in fc.keyframe_points}) if cs else []


def keypose(name, f):
    """该帧曲线数值表示的姿态（四元数，符号归一到 w>=0）。"""
    cs = chans(name)
    if not cs:
        return None
    p = rig.pose.bones[name]
    v = [fc.evaluate(f) for fc in cs]
    q = m._bip_vals_quat(p, v)
    if q is None:
        return None
    q = Quaternion((q.w, q.x, q.y, q.z)).normalized()
    if q.w < 0:
        q.negate()
    return q


def short_deg(qa, qb):
    a = math.degrees(qa.rotation_difference(qb).angle)
    return 360 - a if a > 180 else a


def rot_steps(name, f0=1, f1=80):
    p = rig.pose.bones[name]
    prev, out = None, []
    for f in range(f0, f1 + 1):
        sc.frame_set(f)
        q = p.matrix.to_quaternion().normalized()
        if prev is not None:
            out.append((f, short_deg(prev, q)))
        prev = q
    return out


def hand_path(f0=1, f1=80):
    out = []
    for f in range(f0, f1 + 1):
        sc.frame_set(f)
        p = rig.pose.bones['c_hand_fk.r']
        out.append(rig.matrix_world @ p.matrix.translation.copy())
    return out


def path_jumps(pts, start=1):
    return [(f + start, (pts[f] - pts[f - 1]).length)
            for f in range(1, len(pts))]


# ------------------------------------------------------------ A 检查工具
base_keys = {n: keyframes(n) for n in ALL}
base_pose = {n: {f: keypose(n, f) for f in base_keys[n]} for n in ALL}
modes0 = {n: rig.pose.bones[n].rotation_mode for n in ALL}
loc0 = {n: tuple(rig.pose.bones[n].location) for n in ALL}
con0 = {n: [(c.name, c.type, c.mute) for c in rig.pose.bones[n].constraints]
        for n in ALL}

dry = m.bip_normalize_rotation_curves(rig, bone_names=set(ALL), dry_run=True)
print('V26 DRY', dry)
check('A1 检查发现脏帧', dry['fixed'] >= 1,
      'fixed=%d' % dry['fixed'])
check('A2 扫到缺通道帧', dry['partial'] >= 1,
      'partial=%d' % dry['partial'])

# ------------------------------------------------ 基线（修复前）暴跳证据
s0 = {n: rot_steps(n) for n in NAMES}
h0 = path_jumps(hand_path())
peak0 = {n: max(a for _, a in s0[n][29:51]) for n in NAMES}
bad0 = [f for f, d in h0 if 30 <= f <= 50 and d > 0.18]
print('V26 BASE peak30-50',
      {n: round(v, 1) for n, v in peak0.items()},
      '| path jumps', [(f, round(d, 3)) for f, d in h0
                       if 30 <= f <= 50 and d > 0.18])
check('A3 复刻出暴跳', peak0['c_forearm_fk.r'] > 80 and
      peak0['c_hand_fk.r'] > 80 and len(bad0) >= 2)

# ------------------------------------------- B 模拟 BIP 连续画弧（自动修复）
for o in bpy.data.objects:
    try:
        o.select_set(False)
    except Exception:
        pass
rig.select_set(True)
bpy.context.view_layer.objects.active = rig
if rig.mode != 'POSE':
    bpy.ops.object.mode_set(mode='POSE')
for p in rig.pose.bones:
    p.select = False
rig.pose.bones['c_hand_fk.r'].select = True
rig.data.bones.active = rig.data.bones['c_hand_fk.r']
m.bip_register()
amap = m.bip_resolve(bpy.context, active=rig.pose.bones['c_hand_fk.r'])
check('B0 解析手臂链', amap is not None,
      str(amap['fk']) if amap else '')

if amap:
    start_logs = []
    for fr in (44, 47, 51, 55):
        sc.frame_set(fr)
        st = m.bip_begin(rig, amap, prepare=True)   # 镜像真实 invoke
        # 与真实 invoke 一致：锚点必须取自 bip_begin 返回的状态。
        # 起点修复可能改变当前帧插值姿态，外面先取的坐标会过时。
        start = st['grab_arm'].copy()
        start_logs.append(m._BIP_REPAIR_LOG)
        for k in range(1, 5):
            goal = start + Vector((0.012 * k, -0.010 * k, 0.014 * k))
            m.bip_solve(st, goal)
        m.bip_end(st, commit=True)
        print('V26 DRAG frame', fr, 'ok')
    check('B0b 起点已理顺脏曲线',
          any('自动理顺' in x for x in start_logs), start_logs[0])
    check('B1 收尾自动修复已运行',
          '自动理顺' in m._BIP_REPAIR_LOG or '已连续' in m._BIP_REPAIR_LOG,
          m._BIP_REPAIR_LOG)
    dry2 = m.bip_normalize_rotation_curves(
        rig, bone_names=set(ALL), dry_run=True)
    check('B2 曲线再无脏帧', dry2['fixed'] == 0, 'fixed=%d' % dry2['fixed'])

    # -------------------------------------------- C 暴跳消失
    sc.frame_set(sc.frame_current)
    s1 = {n: rot_steps(n) for n in NAMES}
    h1 = path_jumps(hand_path())
    peak1 = {n: max(a for _, a in s1[n][29:51]) for n in NAMES}
    bad1 = [f for f, d in h1 if 30 <= f <= 50 and d > 0.18]
    print('V26 AFTER peak30-50',
          {n: round(v, 1) for n, v in peak1.items()},
          '| path jumps', [(f, round(d, 3)) for f, d in h1
                           if 30 <= f <= 50 and d > 0.18])
    for n in ('c_forearm_fk.r', 'c_hand_fk.r'):
        check('C1 %s 跳变消除' % n, peak1[n] < 25,
              '%.1f -> %.1f deg' % (peak0[n], peak1[n]))
    check('C2 轨迹打结消除', len(bad1) == 0,
          '%d -> %d 处' % (len(bad0), len(bad1)))
    for f in (34, 35, 44, 45):
        d0 = dict(h0).get(f, 0.0)
        d1 = dict(h1).get(f, 0.0)
        check('C3 f%d 位移' % f, d1 < d0 * 0.5 + 0.02,
              '%.3f -> %.3f m' % (d0, d1))

    # -------------------------------------------- D 姿态/数据零改动
    sc.frame_set(51)     # location 带动画，必须回到采样时的帧再比
    DRAG_FRAMES = {44, 47, 51, 55}
    worst = 0.0
    for n in ALL:
        for f in base_keys[n]:
            if f in DRAG_FRAMES:
                continue
            qa, qb = base_pose[n][f], keypose(n, f)
            if qa is None or qb is None:
                continue
            worst = max(worst, short_deg(qa, qb))
    print('V26 KEYPOSE max err %.8f deg' % worst)
    check('D1 原关键帧姿态不变', worst < 1e-3, '%.8f deg' % worst)
    check('D2 旋转模式不变',
          all(rig.pose.bones[n].rotation_mode == modes0[n] for n in ALL))
    check('D3 位置通道不变',
          all(tuple(rig.pose.bones[n].location) == loc0[n] for n in ALL))
    check('D4 约束不变',
          all([(c.name, c.type, c.mute)
               for c in rig.pose.bones[n].constraints] == con0[n]
              for n in ALL))
    try:
        sw = rig.pose.bones[amap['ik_target']].get('ik_fk_switch')
    except Exception:
        sw = None
    check('D5 ik_fk_switch 仍为 FK', sw is None or abs(sw - 1.0) < 1e-6,
          str(sw))

ok = all(v for _, v in PASS)
print('V26 RESULT %s (%d/%d)' %
      ('ALL_PASS' if ok else 'FAIL', sum(v for _, v in PASS), len(PASS)))
print('V26 DONE')
