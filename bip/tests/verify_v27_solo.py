# -*- coding: utf-8 -*-
"""v2.7 回归：真实工程上验证 BIP 拖动期间「只显示当前控制器」。

只读打开 E 踏步回斩.blend（绝不 save_mainfile）。后台没有视口，
用 m._BIP_SOLO_FORCE = True 让显隐数据照样可测。覆盖：
  A 独显生效 / 多选保留 / 换手 set_keep / 快照只抓一次
  B 导航快照：被临时隐藏的骨在切骨过滤里仍算可见
  C 确认 / 取消 / 交还驭骨 / 交还失败 / 异常兜底 全部还原且不脏
  D 与驭骨共用 solo_display 偏好；还原幂等；路径刷新不报错
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
print('V27 FILE', bpy.data.filepath, '| rig', rig.name,
      '| bones', len(rig.pose.bones))

PASS = []


def check(tag, ok, extra=''):
    PASS.append((tag, bool(ok)))
    print('V27 %-34s %s %s' % (tag, 'PASS' if ok else 'FAIL', extra))


def hide_map():
    return {b.name: bool(b.hide) for b in rig.pose.bones}


def visible_names():
    return {b.name for b in rig.pose.bones if not b.hide}


def diff(a, b):
    return [n for n in a if a[n] != b.get(n)]


orig_hide = hide_map()
m._BIP_SOLO_FORCE = True
m.bip_solo_restore()

# 造一份「有藏有显」的基线，才能验证快照是逐骨精确还原
for i, b in enumerate(rig.pose.bones):
    b.hide = (i % 17 == 0)
base = hide_map()
# bone_visible 不只看 pb.hide，还看所在集合与控件形状缩放；导航快照比的是它，
# 不能拿 not hide 当真值（真实工程里 333 根骨因此不同）。
m._NAV_VIS = None
m._NAV_VIS_RIG = None
base_vis = {b.name: bool(m.bone_visible(b)) for b in rig.pose.bones}
HAND_R = 'c_hand_fk.r' if 'c_hand_fk.r' in rig.pose.bones else list(base)[0]
HAND_L = 'c_hand_fk.l' if 'c_hand_fk.l' in rig.pose.bones else list(base)[1]
ARM_R = 'c_arm_fk.r' if 'c_arm_fk.r' in rig.pose.bones else list(base)[2]
OTHER = next(n for n in base if (not base[n]) and base_vis[n]
             and n not in (HAND_R, HAND_L, ARM_R))

print('V27 BASE hidden=%d visible=%d | hands %s / %s | other %s'
      % (sum(base.values()), len(base) - sum(base.values()),
         HAND_R, HAND_L, OTHER))

# ---------------------------------------------------------------- A 独显
ok = m.bip_solo_apply(ctx, rig, HAND_R, {HAND_R})
vis = visible_names()
check('A1 apply 返回 True', ok)
check('A2 只剩当前控制器可见', vis == {HAND_R},
      'visible=%d' % len(vis))
check('A3 全部 pose bone 都写入显隐', sum(hide_map().values()) == len(base) - 1,
      'hidden=%d / %d' % (sum(hide_map().values()), len(base)))
check('A4 快照已建立', m._BIP_SOLO_ACTIVE is not None
      and m._BIP_SOLO_ACTIVE[0] is rig)
check('A5 快照等于开拖前显隐', diff(m._BIP_SOLO_ACTIVE[1], base) == [])

# -------------------------------------------------- B 导航快照（切骨关键）
nav = getattr(m, '_NAV_VIS', None)
check('B1 _NAV_VIS 已设置', isinstance(nav, dict) and len(nav) == len(base))
check('B2 _NAV_VIS_RIG 指向骨架 data',
      getattr(m, '_NAV_VIS_RIG', None) is rig.data)
pb_other = rig.pose.bones[OTHER]
check('B3 被独显临时藏的骨仍算可见',
      (not base[OTHER]) and base_vis[OTHER]
      and bool(m.bone_visible(pb_other)),
      OTHER)
hidden_before = next(n for n in base if base[n])
check('B4 开拖前就藏的骨仍算不可见',
      not bool(m.bone_visible(rig.pose.bones[hidden_before])),
      hidden_before)
check('B5 快照值与开拖前一致',
      all(bool(nav[n]) == base_vis[n] for n in base),
      'mismatch=%d' % sum(1 for n in base if bool(nav[n]) != base_vis[n]))

# ------------------------------------------- C 重复 apply / 换手 / 多选
saved_id = id(m._BIP_SOLO_ACTIVE[1])
rig.pose.bones[OTHER].hide = False          # 模拟中途状态被改动
m.bip_solo_apply(ctx, rig, HAND_R, {HAND_R})
check('C1 重复 apply 不重抓快照',
      id(m._BIP_SOLO_ACTIVE[1]) == saved_id
      and diff(m._BIP_SOLO_ACTIVE[1], base) == [])
m.bip_solo_set_keep(ctx, rig, HAND_L, {HAND_L})
check('C2 换手后可见集跟着换', visible_names() == {HAND_L},
      str(sorted(visible_names()))[:60])
check('C3 换手不重抓快照', diff(m._BIP_SOLO_ACTIVE[1], base) == [])
m.bip_solo_set_keep(ctx, rig, HAND_R, {HAND_R, ARM_R})
check('C4 多选保留所有选中控制器',
      visible_names() == {HAND_R, ARM_R}, str(sorted(visible_names())))

# ------------------------------------------------------------- D 偏好开关
orig_enabled = m.bip_solo_enabled
m.bip_solo_restore(ctx)
before = hide_map()
m.bip_solo_enabled = lambda: False
ok = m.bip_solo_apply(ctx, rig, HAND_R, {HAND_R})
check('D1 偏好关闭时不独显', (not ok) and hide_map() == before)
check('D2 偏好关闭时有诊断', '关闭' in m._BIP_SOLO_LOG, m._BIP_SOLO_LOG)
m.bip_solo_enabled = orig_enabled

# -------------------------------------------------------- E 还原 / 幂等
m.bip_solo_apply(ctx, rig, HAND_R, {HAND_R})
ok = m.bip_solo_restore(ctx)
after = hide_map()
check('E1 restore 返回 True', ok)
check('E2 逐骨精确还原', diff(after, base) == [],
      'diff=%d' % len(diff(after, base)))
check('E3 导航快照已清空',
      getattr(m, '_NAV_VIS', None) is None
      and getattr(m, '_NAV_VIS_RIG', None) is None)
check('E4 快照已丢弃', m._BIP_SOLO_ACTIVE is None)
check('E5 还原后可见性回到真实值',
      bool(m.bone_visible(rig.pose.bones[OTHER])) == base_vis[OTHER])
check('E6 restore 幂等', m.bip_solo_restore(ctx) is False)

# ------------------------------------------- F 操作符各退出路径的还原顺序
orig_end = m.bip_end
orig_start = m.bip_start_cn_mode
orig_handoff_mode = m.bip_handoff_mode
log = []


class FakeSelf:
    rig = rig
    state = {'map': {'fk_hand': HAND_R}}
    _handed_off = False

    def report(self, *a, **k):
        pass

    def _finish(self, context, commit):
        return m.POSE_OT_cn_bip_drag_v45._finish(self, context, commit)

    def _handoff(self, context, new_name):
        return m.POSE_OT_cn_bip_drag_v45._handoff(self, context, new_name)


fs = FakeSelf()
m.bip_end = lambda state, commit=True, auto_key=True: log.append('end')

# F1 确认 / 取消：_finish 必须先还原再 bip_end
for commit in (True, False):
    m.bip_solo_apply(ctx, rig, HAND_R, {HAND_R})
    m.POSE_OT_cn_bip_drag_v45._finish(fs, ctx, commit)
    check('F1 %s 后精确还原' % ('确认' if commit else '取消'),
          diff(hide_map(), base) == [] and m._BIP_SOLO_ACTIVE is None)

# F2 交还驭骨：调用 bip_start_cn_mode 时显隐必须已经是开拖前的样子
m.bip_handoff_mode = lambda pb: 'ROT'
seen = {}


def fake_start(context, mode, result=None):
    seen['hide_at_start'] = hide_map()
    seen['mode'] = mode
    return result or {'RUNNING_MODAL'}


m.bip_start_cn_mode = fake_start
m.bip_solo_apply(ctx, rig, HAND_R, {HAND_R})
res = m.POSE_OT_cn_bip_drag_v45._handoff(fs, ctx, ARM_R)
check('F2 交还返回 FINISHED', 'FINISHED' in res, str(res))
check('F3 交还时显隐已还原', diff(seen.get('hide_at_start', {}), base) == []
      and len(seen.get('hide_at_start', {})) == len(base),
      'diff=%d' % len(diff(seen.get('hide_at_start', {}), base)))
check('F4 交还后快照作废', m._BIP_SOLO_ACTIVE is None)
check('F5 交还后显隐仍是原始值', diff(hide_map(), base) == [])

# F6~F8：v2.8 行为变更（修「拖完要按 Alt H」）。
# v2.7 在这里断言「重新独显」，但 _handoff 紧接着返回 FINISHED、BIP 模态
# 已经结束，那份独显从此无人还原 —— 实测 425/426 根骨留在 hide=True。
# 现在改为按开拖前快照还原并清掉快照。
m.bip_start_cn_mode = lambda context, mode: fake_start(
    context, mode, {'CANCELLED'})
m.bip_solo_apply(ctx, rig, HAND_R, {HAND_R})
m.POSE_OT_cn_bip_drag_v45._handoff(fs, ctx, ARM_R)
check('F6 交还失败精确还原', diff(hide_map(), base) == [],
      'diff=%d' % len(diff(hide_map(), base)))
check('F7 失败后快照已清', m._BIP_SOLO_ACTIVE is None)
m.bip_solo_apply(ctx, rig, HAND_R, {HAND_R})
check('F8 失败后仍可正常再独显', visible_names() == {HAND_R})
m.bip_solo_restore(ctx)
check('F8b 再次还原精确', diff(hide_map(), base) == [])

m.bip_end = orig_end
m.bip_start_cn_mode = orig_start
m.bip_handoff_mode = orig_handoff_mode

# ------------------------------------------------ G 独显期间路径刷新不报错
m.bip_solo_apply(ctx, rig, HAND_R, {HAND_R})
try:
    m.bip_live_motion_paths(rig, force=True)
    path_ok = True
except Exception as exc:
    path_ok = False
    print('V27 PATH ERR', exc)
check('G1 独显期间路径更新不抛异常', path_ok, m._BIP_LIVE_LOG[:50])
m.bip_solo_restore(ctx)

# ------------------------------- I 集成：独显时 Alt+滚轮切骨仍能切到别的骨
m.bip_solo_apply(ctx, rig, HAND_R, {HAND_R})
before = after = None
switched = False
try:
    rig.data.bones.active = rig.data.bones[HAND_R]
    before = m.bip_active_name(ctx)
    bpy.ops.pose.controller_nav_v45('EXEC_DEFAULT', mode='NEXT')
    after = m.bip_active_name(ctx)
    switched = after is not None and after != before
except Exception as exc:
    print('V27 CUT ERR', type(exc).__name__, str(exc)[:90])
check('I1 独显时切骨算子仍能换骨', switched,
      '%s -> %s' % (before, after))
m.bip_solo_restore(ctx)

# ------------------------------------------------------------------ 收尾
for b in rig.pose.bones:
    b.hide = orig_hide[b.name]
m._BIP_SOLO_FORCE = False
check('H1 原始显隐已复位', diff(hide_map(), orig_hide) == [])
check('H2 卸载兜底函数存在',
      'bip_solo_restore' in m.bip_unregister.__code__.co_names)

bad = [t for t, ok in PASS if not ok]
print('V27 RESULT', 'ALL_PASS' if not bad else 'FAIL',
      '%d/%d' % (len(PASS) - len(bad), len(PASS)), bad)
