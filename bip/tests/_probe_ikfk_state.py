# -*- coding: utf-8 -*-
"""只读探测：ARP IK/FK 切换后 FK 曲线状态（不改工程、不存盘）"""
import bpy
from mathutils import Quaternion
import math

print('PRB file', bpy.data.filepath)
print('PRB frame', bpy.context.scene.frame_current,
      'range', bpy.context.scene.frame_start, bpy.context.scene.frame_end)

rigs = [o for o in bpy.data.objects if o.type == 'ARMATURE']
print('PRB rigs', [o.name for o in rigs])

for rig in rigs:
    sw = [(p.name, round(float(p['ik_fk_switch']), 3))
          for p in rig.pose.bones if 'ik_fk_switch' in p.keys()]
    print('PRB RIG', rig.name, 'switches', sw[:20], 'count', len(sw))
    if not sw:
        continue
    ad = rig.animation_data
    act = ad.action if ad else None
    print('PRB action', act.name if act else None,
          'slots', len(act.slots) if act else 0,
          'layers', len(act.layers) if act else 0)

    # Blender 4.4+ layered action：遍历 channelbags
    fcurves = []
    if act:
        try:
            for layer in act.layers:
                for strip in layer.strips:
                    for cb in strip.channelbags:
                        fcurves.extend(list(cb.fcurves))
        except Exception as e:
            print('PRB layered err', e)
        if not fcurves:
            try:
                fcurves = list(act.fcurves)
            except Exception:
                pass
    print('PRB fcurve count', len(fcurves))

    # 按骨骼 + 旋转通道归类
    from collections import defaultdict
    by_bone = defaultdict(list)
    for fc in fcurves:
        dp = fc.data_path
        if 'rotation_' not in dp and 'ik_fk_switch' not in dp:
            continue
        bone = dp.split('"')[1] if '"' in dp else dp
        by_bone[bone].append(fc)
    print('PRB bones with rot curves', len(by_bone))

    for bn in sorted(by_bone):
        if not any(k in bn for k in ('arm_fk', 'forearm_fk', 'hand_fk',
                                     'arm_ik', 'forearm_ik', 'hand_ik')):
            continue
        fcs = by_bone[bn]
        chans = sorted(set((fc.data_path.split('.')[-1], fc.array_index) for fc in fcs))
        pb = rig.pose.bones.get(bn)
        print('PRB B', bn, '| mode', pb.rotation_mode if pb else '?',
              '| chans', chans,
              '| keys', sorted(set(len(fc.keyframe_points) for fc in fcs)))

    # 找 FK 骨的关键帧跳变（四元数角距离 / 欧拉分量差）
    for bn in sorted(by_bone):
        if 'arm_fk' not in bn and 'forearm_fk' not in bn and 'hand_fk' not in bn:
            continue
        fcs = by_bone[bn]
        quats = [fc for fc in fcs if fc.data_path.endswith('rotation_quaternion')]
        euls = [fc for fc in fcs if fc.data_path.endswith('rotation_euler')]
        if quats and len(quats) == 4:
            q = {fc.array_index: fc for fc in quats}
            pts = sorted(set(round(k.co[0], 2) for k in q[0].keyframe_points))
            worst = []
            for a, b in zip(pts, pts[1:]):
                vals = []
                for i in range(4):
                    fc = q[i]
                    v = [k.co[1] for k in fc.keyframe_points
                         if abs(k.co[0] - a) < .01]
                    v2 = [k.co[1] for k in fc.keyframe_points
                          if abs(k.co[0] - b) < .01]
                    if v and v2:
                        vals.append((v[0], v2[0]))
                if len(vals) == 4:
                    qa = Quaternion([x[0] for x in vals])
                    qb = Quaternion([x[1] for x in vals])
                    dot = min(1.0, abs(qa.dot(qb)))
                    ang = math.degrees(2 * math.acos(max(-1, min(1, dot))))
                    worst.append((ang, a, b, qa.dot(qb)))
            worst.sort(reverse=True)
            print('PRB QJUMP', bn, 'frames', len(pts),
                  'top', [(round(w[0], 1), w[1], w[2], round(w[3], 3))
                          for w in worst[:5]])
        if euls:
            for fc in sorted(euls, key=lambda f: f.array_index):
                ks = sorted(fc.keyframe_points, key=lambda k: k.co[0])
                jumps = [(abs(b.co[1] - a.co[1]), a.co[0], b.co[0])
                         for a, b in zip(ks, ks[1:])]
                jumps.sort(reverse=True)
                print('PRB EJUMP', bn, 'idx', fc.array_index,
                      'keys', len(ks), 'top',
                      [(round(j[0], 2), j[1], j[2]) for j in jumps[:3]])

print('PRB DONE')
