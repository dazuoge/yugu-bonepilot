# -*- coding: utf-8 -*-
"""诊断 all_limbs 模式下某条腿切不回 FK。mode = v32 / v31(卸载接管)"""
import bpy
import sys
import importlib

MODE = 'v32'
if '--' in sys.argv:
    t = sys.argv[sys.argv.index('--') + 1:]
    if t:
        MODE = t[0]

m = importlib.import_module('controller_navigator_v4971')
if MODE == 'v31':
    m._bip_remove_arp_switch()
rig = bpy.data.objects.get('rig.016')
scn = bpy.context.scene
scn.frame_set(24)
bpy.context.view_layer.objects.active = rig
rig.select_set(True)
try:
    bpy.ops.object.mode_set(mode='POSE')
except Exception:
    pass

kind, limbs = m.ikfk_limbs(rig)
print('PRB MODE', MODE, 'kind', kind, 'limbs', len(limbs),
      '| limb_switch', getattr(m.limb_switch, '__name__', '?'))
for lb in limbs:
    print('PRB LIMB', lb.label, '| prop', lb.prop_bone,
          '| fk', list(lb.fk_controls)[:5],
          '| ik', list(lb.ik_controls)[:5])

for tgt in ('IK', 'FK'):
    for lb in limbs:
        pb = rig.pose.bones.get(lb.prop_bone)
        before = float(pb[lb.prop_name]) if pb else -1
        ok, msg = m.limb_switch(rig, lb, tgt)
        after = float(pb[lb.prop_name]) if pb else -1
        sel = [p.name for p in (bpy.context.selected_pose_bones or [])]
        print('PRB SW', tgt, lb.label, 'ok=%s' % ok,
              'sw=%.1f->%.1f' % (before, after), '|', msg[:70])
        print('PRB   sel', sel, '| log', m._BIP_ARP_SWITCH_LOG,
              '| poll', bpy.ops.pose.arp_switch_snap.poll())
    sws = [(p.name, float(p['ik_fk_switch'])) for p in rig.pose.bones
           if 'ik_fk_switch' in p.keys()
           and (p.name.startswith('c_hand_ik')
                or p.name.startswith('c_foot_ik'))]
    print('PRB ALLSW', tgt, sws)
print('PRB DONE', MODE)
