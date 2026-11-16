# -*- coding: utf-8 -*-
"""v2.8 回归：BIP 拖动结束后控制器必定自动还原，不需要按 Alt H。

只读打开 E 踏步回斩.blend；_BIP_SOLO_FORCE=True 让后台也能验证 PoseBone.hide。
复刻出的 4 条残留路径逐条验证已封死：
  P2 交还驭骨失败   P3 外部取消模态   P4 脏快照被固化   守卫/自愈兜底
"""
import os
import sys
import importlib
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
m.bip_solo_forget()

orig = {b.name: bool(b.hide) for b in rig.pose.bones}
for i, b in enumerate(rig.pose.bones):
    b.hide = (i % 17 == 0)
base = {b.name: bool(b.hide) for b in rig.pose.bones}
HAND = 'c_hand_fk.r'
ARM = 'c_arm_fk.r'
OTHER = next(n for n in base if not base[n] and n not in (HAND, ARM))
OP = m.POSE_OT_cn_bip_drag_v45

PASS = []


def check(tag, ok, extra=''):
    PASS.append((tag, bool(ok)))
    print('V28 %-38s %s %s' % (tag, 'PASS' if ok else 'FAIL', extra))


def hmap():
    return {b.name: bool(b.hide) for b in rig.pose.bones}


def diff(a=None, b=None):
    a = a if a is not None else hmap()
    b = b if b is not None else base
    return [n for n in a if a[n] != b.get(n)]


def visible():
    return {n for n, h in hmap().items() if not h}


def reset():
    for b in rig.pose.bones:
        b.hide = base[b.name]
    m.bip_solo_forget()
    m._BIP_DRAG_ACTIVE = False


class FakeSelf:
    rig = rig

    def __init__(self):
        self.state = {'map': {'fk_hand': HAND}}
        self._handed_off = False
        self._bip_ended = False

    def report(self, *a, **k):
        pass

    def _finish(self, context, commit):
        return OP._finish(self, context, commit)

    def _handoff(self, context, new_name):
        return OP._handoff(self, context, new_name)

    def cancel(self, context):
        return OP.cancel(self, context)


orig_end = m.bip_end
orig_start = m.bip_start_cn_mode
orig_mode = m.bip_handoff_mode
end_calls = []
m.bip_end = lambda state, commit=True, auto_key=True: end_calls.append(commit)
m.bip_handoff_mode = lambda pb: 'ROT'

print('V28 FILE', bpy.data.filepath, '| rig', rig.name, '| bones', N)

# ---------------------------------------------------------- A 结构：cancel
check('A1 operator 已实现 cancel()', hasattr(OP, 'cancel'))
check('A2 invoke 设置活跃标记',
      '_BIP_DRAG_ACTIVE' in OP.invoke.__code__.co_names)
check('A3 _finish / modal / cancel 都清理标记',
      all('_BIP_DRAG_ACTIVE' in getattr(OP, n).__code__.co_names
          for n in ('_finish', 'modal', 'cancel')))
check('A4 register / unregister 装拆守卫',
      '_bip_install_solo_guard' in m.bip_register.__code__.co_names
      and '_bip_remove_solo_guard' in m.bip_unregister.__code__.co_names)
consts = m.VIEW3D_PT_cn_bip_v45.draw.__code__.co_consts
check('A5 面板有版本标记',
      any(('v2.' in str(c) or 'v3.' in str(c)) for c in consts))
check('A6 面板有兜底诊断行',
      any('显隐兜底' in str(c) for c in consts))

# ------------------------------------------------- B 外部取消（P3）
fs = FakeSelf()
m.bip_solo_apply(ctx, rig, HAND, {HAND})
m._BIP_DRAG_ACTIVE = True
fs.cancel(ctx)
check('B1 cancel 后逐骨精确还原', diff() == [],
      'diff=%d' % len(diff()))
check('B2 cancel 后快照已清', m._BIP_SOLO_ACTIVE is None)
check('B3 cancel 后活跃标记已清', m._BIP_DRAG_ACTIVE is False)
check('B4 cancel 回滚了姿态', end_calls == [False], str(end_calls))
reset()
end_calls.clear()

# ------------------------------- C cancel 不会二次 bip_end（防回滚已提交）
fs = FakeSelf()
m.bip_solo_apply(ctx, rig, HAND, {HAND})
m._BIP_DRAG_ACTIVE = True
fs._finish(ctx, True)
fs.cancel(ctx)
check('C1 确认后再 cancel 不二次 bip_end', end_calls == [True],
      str(end_calls))
check('C2 确认后显隐仍正确', diff() == [])
reset()
end_calls.clear()

# ------------------------------------------- D 交还失败（P2，v2.7 的 bug）
m.bip_start_cn_mode = lambda context, mode: {'CANCELLED'}
fs = FakeSelf()
m.bip_solo_apply(ctx, rig, HAND, {HAND})
m._BIP_DRAG_ACTIVE = True
res = fs._handoff(ctx, ARM)
check('D1 交还失败返回 FINISHED', 'FINISHED' in res, str(res))
check('D2 交还失败不残留隐藏', diff() == [],
      'hidden=%d diff=%d' % (sum(hmap().values()), len(diff())))
check('D3 交还失败快照已清', m._BIP_SOLO_ACTIVE is None)
check('D4 交还失败活跃标记已清', m._BIP_DRAG_ACTIVE is False)
check('D5 交还失败只提交一次', end_calls == [True], str(end_calls))
reset()
end_calls.clear()

# --------------------------------------------------- E 交还成功（对照）
m.bip_start_cn_mode = lambda context, mode: {'RUNNING_MODAL'}
fs = FakeSelf()
m.bip_solo_apply(ctx, rig, HAND, {HAND})
m._BIP_DRAG_ACTIVE = True
fs._handoff(ctx, ARM)
check('E1 交还成功显隐干净', diff() == [])
check('E2 交还成功快照作废', m._BIP_SOLO_ACTIVE is None)
reset()
end_calls.clear()

# ------------------------------------------------ F 定时守卫兜底
m.bip_solo_apply(ctx, rig, HAND, {HAND})
m._BIP_DRAG_ACTIVE = False          # 模拟 Blender 连 cancel 都没给就丢了模态
hits0 = m._BIP_SOLO_GUARD_HITS
ok = m.bip_solo_guard()
check('F1 守卫自动还原残留', ok and diff() == [],
      'diff=%d' % len(diff()))
check('F2 守卫清空快照', m._BIP_SOLO_ACTIVE is None)
check('F3 守卫计数与日志', m._BIP_SOLO_GUARD_HITS == hits0 + 1
      and '兜底' in m._BIP_SOLO_GUARD_LOG, m._BIP_SOLO_GUARD_LOG)
m.bip_solo_apply(ctx, rig, HAND, {HAND})
m._BIP_DRAG_ACTIVE = True           # 真在拖动：守卫绝不能插手
check('F4 拖动中守卫不误伤',
      m.bip_solo_guard() is False and visible() == {HAND})
m._BIP_DRAG_ACTIVE = False
m.bip_solo_restore(ctx)
reset()

# --------------------------------------------- G 定时器注册（手动兜底）
try:
    m._bip_install_solo_guard()
    timer_ok = bpy.app.timers.is_registered(m._bip_solo_timer)
except Exception as exc:
    timer_ok = False
    print('V28 TIMER ERR', exc)
check('G1 定时守卫已注册', timer_ok)
try:
    m._bip_remove_solo_guard()
    timer_gone = not bpy.app.timers.is_registered(m._bip_solo_timer)
except Exception:
    timer_gone = False
check('G2 卸载可注销定时守卫', timer_gone)
m._bip_install_solo_guard()         # 留给后续 / 真实会话

# ------------------------------------- H 脏快照自愈（P4，最恶心的固化）
# 先建立「已知干净」基准
m.bip_solo_apply(ctx, rig, HAND, {HAND})
m.bip_solo_restore(ctx)
check('H1 干净基准已记录',
      m._BIP_SOLO_CLEAN is not None
      and diff(dict(m._BIP_SOLO_CLEAN[1]), base) == []
      and set(m._BIP_SOLO_CLEAN[2]) == {HAND})
# 制造 v2.7 式残留：独显后 forget（快照没了，hide 还在）
m.bip_solo_apply(ctx, rig, HAND, {HAND})
m.bip_solo_forget()
check('H2 残留复刻', visible() == {HAND} and m._BIP_SOLO_ACTIVE is None)
hits0 = m._BIP_SOLO_GUARD_HITS
m.bip_solo_apply(ctx, rig, HAND, {HAND})     # 新一轮开拖
saved = m._BIP_SOLO_ACTIVE[1]
check('H3 抓快照前已自愈（快照不含残留 hide）',
      diff(dict(saved), base) == [],
      'dirty=%d' % len(diff(dict(saved), base)))
check('H4 自愈计数与日志', m._BIP_SOLO_GUARD_HITS == hits0 + 1
      and '自愈' in m._BIP_SOLO_GUARD_LOG, m._BIP_SOLO_GUARD_LOG)
m.bip_solo_restore(ctx)
check('H5 还原后逐骨精确', diff() == [])
reset()

# ----------------------------------- I 自愈判据严格：不误伤手动隐藏
m.bip_solo_apply(ctx, rig, HAND, {HAND})
m.bip_solo_restore(ctx)                     # clean.keep == {HAND}
for b in rig.pose.bones:
    b.hide = True
for n in (HAND, ARM, OTHER):                # 可见集 != keep
    rig.pose.bones[n].hide = False
hits0 = m._BIP_SOLO_GUARD_HITS
healed = m.bip_solo_heal_if_leaked(rig)
check('I1 可见集不等于 keep 时不自愈',
      (not healed) and visible() == {HAND, ARM, OTHER},
      str(sorted(visible()))[:60])
check('I2 未误改干净基准', m._BIP_SOLO_GUARD_HITS == hits0)
# 干净基准本身就是「大部分隐藏」时也不猜
m._BIP_SOLO_CLEAN = (rig.name, {n: True for n in base}, {HAND})
for b in rig.pose.bones:
    b.hide = True
rig.pose.bones[HAND].hide = False
check('I3 基准本身几乎全隐藏时不自愈',
      m.bip_solo_heal_if_leaked(rig) is False)
# 骨架换了名字（换文件）不猜
m._BIP_SOLO_CLEAN = ('另一个骨架', base, {HAND})
check('I4 不同骨架不自愈',
      m.bip_solo_heal_if_leaked(rig) is False)
reset()

# -------------------------------------------------- J 原有路径仍正确
fs = FakeSelf()
m.bip_solo_apply(ctx, rig, HAND, {HAND})
m._BIP_DRAG_ACTIVE = True
fs._finish(ctx, False)
check('J1 Esc 取消仍精确还原', diff() == [])
m.bip_solo_apply(ctx, rig, HAND, {HAND})
m._BIP_DRAG_ACTIVE = True
fs2 = FakeSelf()
fs2._finish(ctx, True)
check('J2 确认仍精确还原', diff() == [])
check('J3 restore 幂等', m.bip_solo_restore(ctx) is False)
reset()

# --------------------------------------------------------------- 收尾
for b in rig.pose.bones:
    b.hide = orig[b.name]
m._BIP_SOLO_FORCE = False
m._BIP_DRAG_ACTIVE = False
m._bip_remove_solo_guard()
m.bip_end = orig_end
m.bip_start_cn_mode = orig_start
m.bip_handoff_mode = orig_mode
check('K1 原始显隐已复位', diff(hmap(), orig) == [])

bad = [t for t, ok in PASS if not ok]
print('V28 RESULT', 'ALL_PASS' if not bad else 'FAIL',
      '%d/%d' % (len(PASS) - len(bad), len(PASS)), bad)
