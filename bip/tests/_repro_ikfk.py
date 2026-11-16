# -*- coding: utf-8 -*-
"""复刻：ARP 原生 IK/FK 切换写坏 FK 曲线 + BIP 连续化自愈验证
只读工程、不存盘。A 原样 / B 修复 / C 真实 ARP 切换 / D 再修复
"""
import bpy
import math
import importlib
from mathutils import Quaternion

m = importlib.import_module('controller_navigator_v4971')
scn = bpy.context.scene
rig = bpy.data.objects.get('rig.016')
BONES = ['c_arm_fk.r', 'c_forearm_fk.r', 'c_hand_fk.r']
SWITCH = 'c_hand_ik.r'
F0, F1 = 1, 101


def fcurves(act):
    out = []
    try:
        for layer in act.layers:
            for strip in layer.strips:
                for cb in strip.channelbags:
                    out.extend(list(cb.fcurves))
    except Exception:
        pass
    if not out:
        try:
            out = list(act.fcurves)
        except Exception:
            pass
    return out


def key_frames():
    act = rig.animation_data.action
    fs = set()
    for fc in fcurves(act):
        if any(b in fc.data_path for b in BONES) and 'rotation_' in fc.data_path:
            for k in fc.keyframe_points:
                fs.add(int(round(k.co.x)))
    return sorted(f for f in fs if F0 <= f <= F1)


KEYS = key_frames()
print('REP keyframes', KEYS)


def measure(tag):
    """逐帧世界姿态角差"""
    prev = {}
    jumps = {b: [] for b in BONES}
    pose = {}
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
                if ang > 12.0:
                    jumps[bn].append((f, ang))
            prev[bn] = q
    total = sum(len(v) for v in jumps.values())
    worst = max((a for v in jumps.values() for f, a in v), default=0.0)
    bad = sorted(set(f for v in jumps.values() for f, a in v if a > 30.0))
    print('REP %-6s jumps>12deg=%3d worst=%6.1fdeg bad>30=%s'
          % (tag, total, worst, bad[:14]))
    return {'total': total, 'worst': worst, 'bad': bad, 'pose': pose}


def pose_diff(a, b):
    ds = []
    for k, qa in a['pose'].items():
        qb = b['pose'].get(k)
        if qb is None:
            continue
        d = min(1.0, abs(qa.dot(qb)))
        ds.append(math.degrees(2 * math.acos(max(-1.0, min(1.0, d)))))
    return (max(ds) if ds else 0.0, sum(ds) / len(ds) if ds else 0.0)


# ---------------- A：工程原样
A = measure('A')
dry = m.bip_normalize_rotation_curves(rig, bone_names=set(BONES), dry_run=True)
print('REP dry_run', dry)

# ---------------- B：连续化修复
st = m.bip_normalize_rotation_curves(rig, bone_names=set(BONES), dry_run=False)
bpy.context.view_layer.update()
print('REP fix stat', st)
B = measure('B')
pd = pose_diff(A, B)
print('REP A-vs-B keyframe pose max/mean deg', round(pd[0], 4), round(pd[1], 4))

# ---------------- C：真实 ARP 切换 FK -> IK -> FK
print('REP arp operator available:', hasattr(bpy.ops.pose, 'arp_switch_snap'))
try:
    scn.frame_set(24)
    bpy.context.view_layer.objects.active = rig
    rig.select_set(True)
    try:
        bpy.ops.object.mode_set(mode='POSE')
    except Exception as e:
        print('REP mode_set err', e)
    for p in rig.pose.bones:
        p.select = False
    rig.pose.bones['c_arm_fk.r'].select = True
    rig.data.bones.active = rig.data.bones['c_arm_fk.r']
    print('REP selected', [p.name for p in bpy.context.selected_pose_bones]
          if bpy.context.selected_pose_bones else [])
    old_auto = scn.tool_settings.use_keyframe_insert_auto
    scn.tool_settings.use_keyframe_insert_auto = True
    print('REP mode', bpy.context.mode, 'switch before',
          float(rig.pose.bones[SWITCH]['ik_fk_switch']))
    r1 = bpy.ops.pose.arp_switch_snap()
    print('REP switch#1', r1, 'switch', float(rig.pose.bones[SWITCH]['ik_fk_switch']))
    r2 = bpy.ops.pose.arp_switch_snap()
    print('REP switch#2', r2, 'switch', float(rig.pose.bones[SWITCH]['ik_fk_switch']))
    scn.tool_settings.use_keyframe_insert_auto = old_auto
    C = measure('C')
    pd2 = pose_diff(B, C)
    print('REP B-vs-C keyframe pose max/mean deg', round(pd2[0], 4), round(pd2[1], 4))
except Exception as exc:
    import traceback
    print('REP arp switch FAILED', repr(exc))
    traceback.print_exc()
    C = None

# ---------------- D：再修复
st2 = m.bip_normalize_rotation_curves(rig, bone_names=set(BONES), dry_run=False)
bpy.context.view_layer.update()
print('REP fix2 stat', st2)
D = measure('D')

print('REP SUMMARY A=%d B=%d C=%s D=%d'
      % (A['total'], B['total'], C['total'] if C else 'NA', D['total']))
print('REP DONE')
