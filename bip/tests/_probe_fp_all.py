# -*- coding: utf-8 -*-
"""只读诊断：all_limbs 往返中 v3.1 保护的 IK 指纹在哪一步变了。"""
import bpy
import importlib

m = importlib.import_module('controller_navigator_v4971')
rig = bpy.data.objects.get('rig.016')
scn = bpy.context.scene
scn.frame_set(24)
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

orig_fp = m._bip_ikfk_fingerprint
LOG = []
STAGE = ['pre']


def fp(r, side):
    v = orig_fp(r, side)
    LOG.append((STAGE[0], side, v))
    return v


m._bip_ikfk_fingerprint = fp
old = scn.tool_settings.use_keyframe_insert_auto
scn.tool_settings.use_keyframe_insert_auto = True
STAGE[0] = 'to_IK'
bpy.ops.pose.cn_ikfk_toggle_v45(target='IK', all_limbs=True)
STAGE[0] = 'to_FK'
bpy.ops.pose.cn_ikfk_toggle_v45(target='FK', all_limbs=True)
scn.tool_settings.use_keyframe_insert_auto = old
print('FPD guard_log', m._BIP_IKFK_LOG)
print('FPD calls', len(LOG))
for side in ('.l', '.r'):
    rows = [(s, v) for s, sd, v in LOG if sd == side]
    print('FPD SIDE', side, 'n', len(rows))
    for i, (s, v) in enumerate(rows):
        print('FPD  [%02d] %-6s %s' % (i, s, str(v)[:220]))
    # 打印相邻两次之间的差异项
    for i in range(1, len(rows)):
        a, b = rows[i - 1][1], rows[i][1]
        if a != b:
            diffs = [(j, x, y) for j, (x, y) in enumerate(zip(a, b)) if x != y]
            print('FPD  DIFF %d->%d %s' % (i - 1, i, diffs[:6]))
print('FPD DONE')
