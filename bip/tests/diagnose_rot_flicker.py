# -*- coding: utf-8 -*-
"""复刻：驭骨原生「旋转模式」独显被 v2.9 显隐守卫误判成 BIP 残留，整副控制器被亮回来。

用法：blender --background <E 踏步回斩.blend> --python diagnose_rot_flicker.py
只读，不保存。
"""
import importlib
import bpy

m = importlib.import_module('controller_navigator_v4971')
rig = bpy.data.objects['rig.016']
ctx = bpy.context
HAND = 'c_hand_fk.r'

m._BIP_SOLO_FORCE = True


def hide_map():
    return {b.name: bool(b.hide) for b in rig.pose.bones}


def visible():
    return {k for k, v in hide_map().items() if not v}


def set_cn_solo(keep):
    """按驭骨主文件 _solo_apply 的写法模拟原生 ROT 模态独显。"""
    for b in rig.pose.bones:
        b.hide = b.name not in keep


base = hide_map()
print('DIAG base bones', len(base), 'visible', len(visible()))

# 1. 模拟一次完整的 BIP 拖动：apply 独显 -> restore 还原（CLEAN/keep 留着）
m.bip_solo_apply(ctx, rig, HAND, {HAND})
print('DIAG BIP applied visible', len(visible()))
m.bip_solo_restore(ctx)
clean = m._BIP_SOLO_CLEAN
print('DIAG after BIP restore diff', sum(1 for k in base if hide_map()[k] != base[k]))
print('DIAG CLEAN', bool(clean), 'keep', sorted(clean[2]) if clean else None,
      'armed', getattr(m, '_BIP_SOLO_ARMED', 'N/A'))

# 2. 进入驭骨原生 ROT 模态：它按同样的规则独显，可见集与 BIP 完全一致
set_cn_solo({HAND})
print('DIAG CN ROT solo visible', len(visible()))
sig = m.bip_solo_leak_signature(rig, clean[1], set(clean[2]))
print('DIAG leak_signature thinks residual =', sig)

# 3. 定时守卫路径（v2.9 每 0.5s 一次）
before = hide_map()
r1 = m.bip_solo_guard()
after = hide_map()
diff = sum(1 for k in base if before[k] != after[k])
print('DIAG TIMER guard returned', r1, 'changed', diff, 'visible now', len(visible()),
      'FLICKER', diff > 50)

# 4. 撤销回调路径（旋转时一次 undo push 就会触发）
set_cn_solo({HAND})
before = hide_map()
r2 = m.bip_solo_repair_after_undo('撤回')
after = hide_map()
diff2 = sum(1 for k in base if before[k] != after[k])
print('DIAG UNDO guard returned', r2, 'changed', diff2, 'visible now', len(visible()),
      'FLICKER', diff2 > 50)

# 5. 还原现场
for n, h in base.items():
    rig.pose.bones[n].hide = h
m._BIP_SOLO_ACTIVE = None
m._BIP_SOLO_KEEP = set()
print('DIAG restored diff', sum(1 for k in base if hide_map()[k] != base[k]))

# ---------------------------------------------------------------- v3.0 复测
# 上面 3/4 是无模态上下文的后台等效场景，代表 v2.9 的误判。
# 真实 GUI 里「谁在管显隐」由驭骨 _MODAL_ONCE 闸门 / window.modal_operators
# 决定；后台无法起真模态，用 _BIP_FOREIGN_MODAL_PROBE 注入同样的上下文。
print('DIAG --- v3.0 复测：带模态上下文 ---')
m.bip_solo_apply(ctx, rig, HAND, {HAND})
m.bip_solo_restore(ctx)
m._BIP_FOREIGN_MODAL_PROBE = lambda: True
set_cn_solo({HAND})
b3 = hide_map()
r3 = m.bip_solo_guard()
d3 = sum(1 for k in base if b3[k] != hide_map()[k])
print('DIAG V30 TIMER guard returned', r3, 'changed', d3,
      'visible now', len(visible()), 'FLICKER', d3 > 50)
set_cn_solo({HAND})
b4 = hide_map()
r4 = m.bip_solo_repair_after_undo('撤回')
d4 = sum(1 for k in base if b4[k] != hide_map()[k])
print('DIAG V30 UNDO guard returned', r4, 'changed', d4,
      'visible now', len(visible()), 'FLICKER', d4 > 50)

# 第二道保险：operator 正常收尾会消费授权，此后即使模态探测完全失效，
# 定时守卫也不会对原生 ROT 的独显补刀。
m._BIP_FOREIGN_MODAL_PROBE = None
m._bip_solo_disarm()
set_cn_solo({HAND})
b5 = hide_map()
r5 = m.bip_solo_guard()
d5 = sum(1 for k in base if b5[k] != hide_map()[k])
print('DIAG V30 DISARM TIMER returned', r5, 'changed', d5, 'FLICKER', d5 > 50)

for n, h in base.items():
    rig.pose.bones[n].hide = h
m._BIP_SOLO_ACTIVE = None
m._BIP_SOLO_KEEP = set()
m._BIP_SOLO_ARMED = False
print('DIAG final restored diff', sum(1 for k in base if hide_map()[k] != base[k]))
print('DIAG DONE')
