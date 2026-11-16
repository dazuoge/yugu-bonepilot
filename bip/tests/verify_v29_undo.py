# -*- coding: utf-8 -*-
"""v2.9 回归：BIP 拖完按 Ctrl+Z，控制器不能再残留「只有手掌」的独显。

关键：不重置工程显隐，保留 E 踏步回斩.blend 里 426 骨中本来就有
~350 根隐藏的真实分布。v2.8 的判据依赖「干净态里隐藏骨 < 50%」，
在这种工程里恒不成立，回归若人为把骨全设可见就测不出问题。

后台模式 ed.undo 被 Blender 禁用（poll 失败），因此这里把 hide 手工
设成「撤销后恢复出的独显态」，再通过 undo_post / redo_post 回调链
触发守卫，验证的是同一条代码路径。姿态与曲线零改动单独断言。
"""
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
m._BIP_DRAG_ACTIVE = False
m._BIP_SOLO_CLEAN = None
m._BIP_UNDO_HITS = 0
m._BIP_UNDO_LOG = '未触发'

base = {b.name: bool(b.hide) for b in rig.pose.bones}
HAND = 'c_hand_fk.r'
ARM = 'c_arm_fk.r'
OTHER = next(n for n in base if not base[n] and n not in (HAND, ARM))
PASS = []


def check(tag, ok, extra=''):
    PASS.append((tag, bool(ok)))
    print('V29 %-40s %s %s' % (tag, 'PASS' if ok else 'FAIL', extra))


def hmap():
    return {b.name: bool(b.hide) for b in rig.pose.bones}


def diff(a=None, b=None):
    a = a if a is not None else hmap()
    b = b if b is not None else base
    return [k for k in a if a[k] != b.get(k)]


def visible():
    return {k for k, h in hmap().items() if not h}


def to_base():
    for b in rig.pose.bones:
        b.hide = base[b.name]


def to_solo():
    """等价于 Blender 把撤销快照里的独显态恢复出来。"""
    for b in rig.pose.bones:
        b.hide = (b.name != HAND)


def pose_snapshot():
    out = {}
    for b in rig.pose.bones:
        try:
            out[b.name] = (tuple(b.location), tuple(b.rotation_quaternion),
                           tuple(b.rotation_euler), tuple(b.scale),
                           b.rotation_mode)
        except Exception:
            pass
    return out


def curve_snapshot():
    out = []
    try:
        for a in bpy.data.actions:
            for fc in getattr(a, 'fcurves', []) or []:
                out.append((fc.data_path, fc.array_index,
                            tuple(round(k.co.x, 4) for k in fc.keyframe_points),
                            tuple(round(k.co.y, 6) for k in fc.keyframe_points)))
    except Exception:
        pass
    return out


print('V29 BASE rig=%s bones=%d hidden=%d visible=%d' %
      (rig.name, N, sum(base.values()), N - sum(base.values())))

# ------------------------------------------------ A 真实分布下判据必须成立
check('A1 保留工程原有隐藏分布', sum(base.values()) > N * 0.5,
      'hidden=%d/%d' % (sum(base.values()), N))
m.bip_solo_apply(ctx, rig, HAND, {HAND})
check('A2 开拖后只剩手控骨', visible() == {HAND}, str(sorted(visible()))[:60])
m.bip_solo_restore(ctx)
check('A3 收尾精确还原', diff() == [], 'diff=%d' % len(diff()))
clean = m._BIP_SOLO_CLEAN
check('A4 干净基准已记录', clean is not None and clean[0] == rig.name
      and diff(dict(clean[1]), base) == [] and set(clean[2]) == {HAND})
to_solo()
sig = m.bip_solo_leak_signature(rig, clean[1], set(clean[2]))
check('A5 新判据在真实分布下成立', sig >= N * 0.1, 'wrong=%d' % sig)
check('A6 v2.8 旧判据在此工程确会失效',
      sum(clean[1].values()) > N * 0.5,
      'clean_hidden=%d > %d' % (sum(clean[1].values()), N // 2))
to_base()

# ------------------------------------------------ B 撤销 / 重做即时守卫
pose0, curve0 = pose_snapshot(), curve_snapshot()
to_solo()
check('B0 复刻撤销后的独显态', visible() == {HAND})
hits0 = m._BIP_UNDO_HITS
m._bip_undo_post_handler(ctx.scene)
check('B1 撤回后显隐精确还原', diff() == [], 'diff=%d' % len(diff()))
check('B2 撤回守卫计数与日志', m._BIP_UNDO_HITS == hits0 + 1
      and '撤回' in m._BIP_UNDO_LOG, m._BIP_UNDO_LOG)
check('B3 姿态与缩放零改动', pose_snapshot() == pose0)
check('B4 关键帧曲线零改动', curve_snapshot() == curve0)
check('B5 导航快照已清', getattr(m, '_NAV_VIS', None) is None
      and getattr(m, '_NAV_VIS_RIG', None) is None)

to_solo()
hits0 = m._BIP_UNDO_HITS
m._bip_redo_post_handler(ctx.scene)
check('B6 重做后同样还原', diff() == [] and m._BIP_UNDO_HITS == hits0 + 1
      and '重做' in m._BIP_UNDO_LOG, m._BIP_UNDO_LOG)

# 连续撤销
for _ in range(3):
    to_solo()
    m._bip_undo_post_handler(ctx.scene)
check('B7 连续撤销每次都还原', diff() == [] and m._BIP_UNDO_HITS >= 5,
      'hits=%d' % m._BIP_UNDO_HITS)

# 干净态时不应误触发
hits0 = m._BIP_UNDO_HITS
m._bip_undo_post_handler(ctx.scene)
check('B8 干净态不重复计数', m._BIP_UNDO_HITS == hits0 and diff() == [])

# ------------------------------------------------ C 回调注册 / 幂等 / 注销
undo_lst = bpy.app.handlers.undo_post
redo_lst = bpy.app.handlers.redo_post


def has(lst, name):
    return any(getattr(fn, '__name__', '') == name for fn in lst)


try:
    m._bip_install_undo_hooks()
    c1 = (has(undo_lst, '_bip_undo_post_handler')
          and has(redo_lst, '_bip_redo_post_handler'))
except Exception as exc:
    c1 = False
    print('V29 HOOK ERR', exc)
print('V29 HOOKS undo=%s redo=%s flag=%s' %
      ([getattr(f, '__name__', '?') for f in undo_lst],
       [getattr(f, '__name__', '?') for f in redo_lst],
       m._BIP_UNDO_HOOKS_ON))
check('C1 两个回调已注册', c1)
n0 = (len(undo_lst), len(redo_lst))
m._bip_install_undo_hooks()
m._bip_install_undo_hooks()
check('C2 重复安装不重复注册', (len(undo_lst), len(redo_lst)) == n0,
      str((len(undo_lst), len(redo_lst))))
m._bip_remove_undo_hooks()
check('C3 卸载可注销',
      not has(undo_lst, '_bip_undo_post_handler')
      and not has(redo_lst, '_bip_redo_post_handler'))
m._bip_install_undo_hooks()

# 走真实 handler 列表（Blender 调用方式），而不是直接调函数
to_solo()
for fn in list(undo_lst):
    try:
        fn(ctx.scene)
    except Exception:
        pass
check('C4 经 handlers 列表触发也生效', diff() == [], 'diff=%d' % len(diff()))

# ------------------------------------------------ D 严格判据：不误伤
m._BIP_UNDO_HITS = 0
m._BIP_UNDO_LOG = '未触发'
for b in rig.pose.bones:
    b.hide = True
for n in (HAND, ARM, OTHER):
    rig.pose.bones[n].hide = False
hits0 = m._BIP_UNDO_HITS
m._bip_undo_post_handler(ctx.scene)
check('D1 可见集不等于 keep 时不插手',
      m._BIP_UNDO_HITS == hits0 and visible() == {HAND, ARM, OTHER})
to_base()

m._BIP_SOLO_CLEAN = (rig.name, {n: True for n in base}, {HAND})
to_solo()
check('D2 基准全隐藏时不猜', m.bip_solo_repair_after_undo('撤回') is False)
m._BIP_SOLO_CLEAN = clean

m._BIP_SOLO_CLEAN = ('另一个骨架', dict(clean[1]), {HAND})
to_solo()
check('D3 不同骨架不猜', m.bip_solo_repair_after_undo('撤回') is False)
m._BIP_SOLO_CLEAN = clean

saved_clean = m._BIP_SOLO_CLEAN
m._BIP_SOLO_CLEAN = None
to_solo()
check('D4 无干净基准不猜', m.bip_solo_repair_after_undo('撤回') is False)
m._BIP_SOLO_CLEAN = saved_clean

m._BIP_DRAG_ACTIVE = True
to_solo()
check('D5 拖动中守卫不插手',
      m.bip_solo_repair_after_undo('撤回') is False and visible() == {HAND})
m._BIP_DRAG_ACTIVE = False

# ------------------------------------------------ E 定时守卫兜底
to_base()
m.bip_solo_apply(ctx, rig, HAND, {HAND})
m.bip_solo_restore(ctx)                 # 建立干净基准，快照 forget
to_solo()
m._BIP_SOLO_ACTIVE = None
m._BIP_DRAG_ACTIVE = False
check('E1 定时守卫能补上撤销残留',
      m.bip_solo_guard() is True and diff() == [], 'diff=%d' % len(diff()))
to_solo()
m._BIP_DRAG_ACTIVE = True
check('E2 拖动中定时守卫不误伤',
      m.bip_solo_guard() is False and visible() == {HAND})
m._BIP_DRAG_ACTIVE = False
to_base()

# ------------------------------------------------ F 原有能力与统一判据
check('F1 v2.8 开拖自愈在真实分布下也生效',
      (lambda: (to_solo(),
                m.bip_solo_apply(ctx, rig, HAND, {HAND}),
                diff(dict(m._BIP_SOLO_ACTIVE[1]), base) == []))()[2])
m.bip_solo_restore(ctx)
check('F2 自愈后仍可精确还原', diff() == [])
check('F3 定时器仍在注册', bpy.app.timers.is_registered(m._bip_solo_timer))
consts = m.VIEW3D_PT_cn_bip_v45.draw.__code__.co_consts
check('F4 面板版本标记 v2.9',
      any('v2.9' in str(c) for c in consts))

ok = all(v for _, v in PASS)
print('V29 RESULT %s  (%d/%d)' %
      ('ALL_PASS' if ok else 'FAIL', sum(v for _, v in PASS), len(PASS)))
for t, v in PASS:
    if not v:
        print('V29 FAILED ITEM', t)
print('V29 DONE')
