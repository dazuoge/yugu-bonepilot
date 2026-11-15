# -*- coding: utf-8 -*-
"""复刻 v2.7「BIP 拖完其他控制器不回来、要按 Alt H」的残留路径。

只读打开真实工程；_BIP_SOLO_FORCE=True 让后台也能验证 PoseBone.hide。
"""
import sys, importlib
import bpy

try:
    sys.stdout.reconfigure(encoding='utf-8')
except Exception:
    pass

m = importlib.import_module('controller_navigator_v4971')
ctx = bpy.context
rig = bpy.data.objects['rig.016']
N = len(rig.pose.bones)
m._BIP_SOLO_FORCE = True
m.bip_solo_restore()

orig = {b.name: bool(b.hide) for b in rig.pose.bones}
for i, b in enumerate(rig.pose.bones):
    b.hide = (i % 17 == 0)
base = {b.name: bool(b.hide) for b in rig.pose.bones}
HAND = 'c_hand_fk.r'
ARM = 'c_arm_fk.r'


def hidden_count():
    return sum(1 for b in rig.pose.bones if b.hide)


def diff_count():
    return sum(1 for b in rig.pose.bones if bool(b.hide) != base[b.name])


class FakeSelf:
    rig = rig
    state = {'map': {'fk_hand': HAND}}
    _handed_off = False

    def report(self, *a, **k):
        pass


fs = FakeSelf()
orig_end = m.bip_end
orig_start = m.bip_start_cn_mode
orig_mode = m.bip_handoff_mode
m.bip_end = lambda state, commit=True, auto_key=True: None
m.bip_handoff_mode = lambda pb: 'ROT'

print('DIAG BASE hidden=%d/%d visible=%d' % (hidden_count(), N, N - hidden_count()))

# --- 路径 1：正常确认（对照组）
m.bip_solo_apply(ctx, rig, HAND, {HAND})
m.POSE_OT_cn_bip_drag_v45._finish(fs, ctx, True)
print('DIAG P1 confirm     hidden=%d diff=%d active=%s'
      % (hidden_count(), diff_count(), m._BIP_SOLO_ACTIVE is not None))
for b in rig.pose.bones:
    b.hide = base[b.name]

# --- 路径 2：交还驭骨失败（v2.7 会 reapply 后返回 FINISHED）
m.bip_start_cn_mode = lambda context, mode: {'CANCELLED'}
m.bip_solo_apply(ctx, rig, HAND, {HAND})
res = m.POSE_OT_cn_bip_drag_v45._handoff(fs, ctx, ARM)
print('DIAG P2 handoff-fail hidden=%d diff=%d res=%s active=%s'
      % (hidden_count(), diff_count(), res, m._BIP_SOLO_ACTIVE is not None))
print('DIAG P2 LEAK', hidden_count() > N * 0.9)
m.bip_solo_restore(ctx)
for b in rig.pose.bones:
    b.hide = base[b.name]

# --- 路径 3：Blender 外部取消模态（切模式 / 换文件 / 别的算子抢占）
has_cancel = hasattr(m.POSE_OT_cn_bip_drag_v45, 'cancel')
m.bip_solo_apply(ctx, rig, HAND, {HAND})
# 模拟 Blender 丢弃模态：不调用 _finish，直接让 operator 结束
if has_cancel:
    m.POSE_OT_cn_bip_drag_v45.cancel(fs, ctx)
print('DIAG P3 external-cancel has_cancel=%s hidden=%d diff=%d LEAK=%s'
      % (has_cancel, hidden_count(), diff_count(),
         hidden_count() > N * 0.9))
print('DIAG P3 LEAK', hidden_count() > N * 0.9)
m.bip_solo_restore(ctx)
for b in rig.pose.bones:
    b.hide = base[b.name]

# --- 路径 4：残留状态下再次开拖，快照会不会被抓脏
m.bip_solo_apply(ctx, rig, HAND, {HAND})     # 先制造残留
m.bip_solo_forget()                          # 快照被清（模拟交还/重载/禁用插件）
m.bip_solo_apply(ctx, rig, HAND, {HAND})     # 新一轮：抓到的就是全隐藏状态
saved = m._BIP_SOLO_ACTIVE[1] if m._BIP_SOLO_ACTIVE else {}
dirty = sum(1 for n, h in saved.items() if h and not base.get(n, False))
m.bip_solo_restore(ctx)
print('DIAG P4 dirty-snapshot keys=%d dirty=%d after_restore_diff=%d LEAK=%s'
      % (len(saved), dirty, diff_count(), diff_count() > 0))
print('DIAG P4 LEAK', diff_count() > 0)

# --- 路径 5：用户按 Alt H 之后才恢复
for b in rig.pose.bones:
    b.hide = False
print('DIAG P5 alt-H visible=%d' % (N - hidden_count()))

for b in rig.pose.bones:
    b.hide = orig[b.name]
m.bip_solo_forget()
m._BIP_SOLO_FORCE = False
m.bip_end = orig_end
m.bip_start_cn_mode = orig_start
m.bip_handoff_mode = orig_mode
print('DIAG DONE')
