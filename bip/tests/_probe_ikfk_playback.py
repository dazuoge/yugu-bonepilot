# -*- coding: utf-8 -*-
"""只读：逐帧评估 rig.016 右臂 FK 姿态连续性 + ik_fk_switch 曲线"""
import bpy
import math
from mathutils import Quaternion

scn = bpy.context.scene
rig = bpy.data.objects.get('rig.016')
print('PLY rig', rig.name if rig else None, 'range', scn.frame_start, scn.frame_end)

BONES = ['c_arm_fk.r', 'c_forearm_fk.r', 'c_hand_fk.r']
SW = 'c_hand_ik.r'

# switch 曲线
def all_fcurves(act):
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

act = rig.animation_data.action if rig.animation_data else None
fcs = all_fcurves(act)
for fc in fcs:
    if 'ik_fk_switch' in fc.data_path and SW in fc.data_path:
        print('PLY SWCURVE', fc.data_path,
              [(round(k.co[0], 1), round(k.co[1], 3), k.interpolation)
               for k in fc.keyframe_points])

# FK 骨有效旋转曲线与 mode
for bn in BONES:
    pb = rig.pose.bones[bn]
    print('PLY MODE', bn, pb.rotation_mode)
    for fc in fcs:
        if bn in fc.data_path and ('rotation_euler' in fc.data_path or
                                   'rotation_quaternion' in fc.data_path):
            ks = [(round(k.co[0], 1), round(k.co[1], 4))
                  for k in fc.keyframe_points]
            print('PLY FC', bn, fc.data_path.split('.')[-1], fc.array_index,
                  'n', len(ks), ks[:16])

# 逐帧姿态
prev = {}
jumps = {b: [] for b in BONES}
sw_vals = {}
for f in range(scn.frame_start, min(scn.frame_end, 110) + 1):
    scn.frame_set(f)
    bpy.context.view_layer.update()
    sw_vals[f] = round(float(rig.pose.bones[SW]['ik_fk_switch']), 2)
    for bn in BONES:
        q = (rig.matrix_world @ rig.pose.bones[bn].matrix).to_quaternion()
        if bn in prev:
            d = min(1.0, abs(prev[bn].dot(q)))
            ang = math.degrees(2 * math.acos(max(-1.0, min(1.0, d))))
            if ang > 12.0:
                jumps[bn].append((f, round(ang, 1)))
        prev[bn] = q

print('PLY SWITCH VALUES', sorted(set(sw_vals.items()))[:40])
for bn in BONES:
    js = sorted(jumps[bn], key=lambda x: -x[1])
    print('PLY JUMPS', bn, 'count', len(js), 'top', js[:12])

# 关键：暴跳帧处 switch 值
allbad = sorted(set(f for bn in BONES for f, a in jumps[bn] if a > 30))
print('PLY BADFRAMES', [(f, sw_vals.get(f)) for f in allbad[:30]])
print('PLY DONE')
