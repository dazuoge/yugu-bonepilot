# -*- coding: utf-8 -*-
"""v3.0 专项回归：驭骨原生旋转模态独显不再被 BIP 守卫误还原。

用法：blender --background <E 踏步回斩.blend> --python verify_v30_flicker.py
真实工程 426 骨（干净态 74 根可见），全程只读、不保存。
后台无法启动真 GUI 模态，用 _BIP_FOREIGN_MODAL_PROBE 注入「别的模态在跑」，
生产代码走的是 window.modal_operators + 驭骨 _MODAL_ONCE 闸门。
"""
import importlib
import bpy

m = importlib.import_module('controller_navigator_v4971')
rig = bpy.data.objects['rig.016']
ctx = bpy.context
HAND = 'c_hand_fk.r'
ARM = 'c_arm_fk.r'
FORE = 'c_forearm_fk.r'

PASS = []


def check(name, ok, extra=''):
    PASS.append((name, bool(ok)))
    print('V30 %-42s %s %s' % (name, 'PASS' if ok else 'FAIL', extra))


def hide_map():
    return {b.name: bool(b.hide) for b in rig.pose.bones}


def visible():
    return {k for k, v in hide_map().items() if not v}


def diff(a, b):
    return sum(1 for k in a if a[k] != b[k])


def set_cn_solo(keep):
    """按驭骨主文件 _solo_apply 的写法模拟原生 ROT 模态独显。"""
    for b in rig.pose.bones:
        b.hide = b.name not in keep


def bip_round():
    """完整一轮 BIP 拖动：授权 + 留下 CLEAN/keep 基准。"""
    m.bip_solo_apply(ctx, rig, HAND, {HAND})
    m.bip_solo_restore(ctx)


def reset(armed=False):
    for n, h in BASE.items():
        rig.pose.bones[n].hide = h
    m._BIP_SOLO_ACTIVE = None
    m._BIP_SOLO_KEEP = set()
    m._BIP_SOLO_ARMED = armed
    m._BIP_FOREIGN_MODAL_PROBE = None
    m._BIP_UNDO_BUSY = False


m._BIP_SOLO_FORCE = True
BASE = hide_map()
CLEAN_VIS = len(visible())
print('V30 BASE bones=%d visible=%d' % (len(BASE), CLEAN_VIS))

# ---------------------------------------------------------------- A 闪烁主场景
reset()
bip_round()
clean = m._BIP_SOLO_CLEAN
m._BIP_FOREIGN_MODAL_PROBE = lambda: True       # 驭骨 ROT 模态运行中
set_cn_solo({HAND})
check('A1 原生 ROT 独显可见集 = 1', visible() == {HAND})
sig = m.bip_solo_leak_signature(rig, clean[1], set(clean[2]))
check('A2 数值判据仍会误判（说明靠上下文拦截）', sig > 50, 'sig=%d' % sig)
skips0 = m._BIP_FOREIGN_SKIPS
before = hide_map()
r1 = m.bip_solo_guard()
check('A3 定时守卫放行不动显隐', (not r1) and diff(before, hide_map()) == 0,
      'changed=%d' % diff(before, hide_map()))
check('A4 定时放行后仍只显示手控骨', visible() == {HAND})
before = hide_map()
r2 = m.bip_solo_repair_after_undo('撤回')
check('A5 撤销守卫放行不动显隐', (not r2) and diff(before, hide_map()) == 0,
      'changed=%d' % diff(before, hide_map()))
check('A6 撤销放行后仍只显示手控骨', visible() == {HAND})
check('A7 放行次数已累计', m._BIP_FOREIGN_SKIPS > skips0,
      'skips=%d' % m._BIP_FOREIGN_SKIPS)
check('A8 外来模态出现即撤销授权', m._BIP_SOLO_ARMED is False)
m._BIP_FOREIGN_MODAL_PROBE = None
r3 = m.bip_solo_guard()
check('A9 模态结束后定时器不补刀', (not r3) and visible() == {HAND},
      'visible=%d' % len(visible()))

# ------------------------------------------------- B 真实 undo 残留仍能修（v2.9）
reset()
bip_round()
set_cn_solo({HAND})                              # 撤销把独显态恢复出来
hits0 = m._BIP_UNDO_HITS
before = hide_map()
r4 = m.bip_solo_repair_after_undo('撤回')
check('B1 真实撤回仍触发修复', r4, 'wrong=%d' % diff(before, hide_map()))
check('B2 修复后逐骨显隐回到干净态', diff(hide_map(), BASE) == 0)
check('B3 修复后可见骨数复原', len(visible()) == CLEAN_VIS,
      '%d/%d' % (len(visible()), CLEAN_VIS))
check('B4 撤销计数递增', m._BIP_UNDO_HITS > hits0)
check('B5 修复后授权已消费', m._BIP_SOLO_ARMED is False)

# ------------------------------------------------------------ C 定时兜底需授权
reset(armed=False)
set_cn_solo({HAND})
r5 = m.bip_solo_guard()
check('C1 未授权时定时兜底不修', (not r5) and visible() == {HAND})
reset(armed=True)
set_cn_solo({HAND})
r6 = m.bip_solo_guard()
check('C2 已授权且无外来模态仍可兜底', r6 and diff(hide_map(), BASE) == 0)

# -------------------------------------------------------- D 外来模态检测函数
reset()
check('D1 无注入 / 无模态 -> False', m._bip_foreign_modal_active() is False)
m._BIP_FOREIGN_MODAL_PROBE = lambda: True
check('D2 注入活跃 -> True', m._bip_foreign_modal_active() is True)
m._BIP_FOREIGN_MODAL_PROBE = lambda: False
check('D3 注入非活跃 -> False', m._bip_foreign_modal_active() is False)
m._BIP_FOREIGN_MODAL_PROBE = lambda: 1 / 0
check('D4 注入异常安全返回 False', m._bip_foreign_modal_active() is False)
m._BIP_FOREIGN_MODAL_PROBE = None
g = m.globals() if hasattr(m, 'globals') else None
gate = getattr(m, '_MODAL_ONCE', None)
if gate is not None:
    old_gate = gate[0]
    gate[0] = True
    check('D5 后台模式不采信残留闸门', m._bip_foreign_modal_active() is False,
          'background=%s' % bpy.app.background)
    gate[0] = old_gate
else:
    check('D5 驭骨闸门存在', False)
check('D6 探测函数对空模态列表不误报',
      m._bip_foreign_modal_active() is False)

# ------------------------------------------------------- E BIP 正常流程不受影响
reset()
m.bip_solo_apply(ctx, rig, HAND, {HAND})
check('E1 开拖正常独显', visible() == {HAND})
check('E2 开拖即授权兜底', m._BIP_SOLO_ARMED is True)
check('E3 CLEAN 基准已记录', bool(m._BIP_SOLO_CLEAN))
m.bip_solo_restore(ctx)
check('E4 收尾精确还原', diff(hide_map(), BASE) == 0)
check('E5 ACTIVE 已清空', m._BIP_SOLO_ACTIVE is None)
check('E6 CLEAN 保留给下轮自愈', bool(m._BIP_SOLO_CLEAN))

# ------------------------------------------------------------- F 姿态零改动
reset()
POSE_NAMES = (ARM, FORE, HAND)


def pose_snapshot():
    out = []
    for n in POSE_NAMES:
        pb = rig.pose.bones[n]
        out.append((n, tuple(pb.location), tuple(pb.scale),
                    tuple(getattr(pb, m.bip_rotation_path(pb)))))
    return out


p0 = pose_snapshot()
bip_round()
m._BIP_FOREIGN_MODAL_PROBE = lambda: True
set_cn_solo({HAND})
m.bip_solo_guard()
m.bip_solo_repair_after_undo('撤回')
p1 = pose_snapshot()
check('F1 守卫全程不动姿态通道', p0 == p1)

# ------------------------------------------------------------ G handler 注册
undo_lst = bpy.app.handlers.undo_post
redo_lst = bpy.app.handlers.redo_post
m._bip_install_undo_hooks()
n0 = (len(undo_lst), len(redo_lst))
m._bip_install_undo_hooks()
check('G1 重复安装不重复注册', (len(undo_lst), len(redo_lst)) == n0)
check('G2 两个回调仍在列表',
      any(getattr(f, '__name__', '') == '_bip_undo_post_handler' for f in undo_lst)
      and any(getattr(f, '__name__', '') == '_bip_redo_post_handler' for f in redo_lst))

# ------------------------------------------------------------ H 手动隐藏不误伤
reset(armed=True)
for n in (ARM, FORE):
    rig.pose.bones[n].hide = True
manual = hide_map()
r7 = m.bip_solo_repair_after_undo('撤回')
check('H1 非残留分布不触发修复', not r7)
check('H2 手动隐藏保持原样', diff(hide_map(), manual) == 0)

# ------------------------------------------------------------------ 面板版本
consts = []
try:
    # 与 v2.8 / v2.9 回归一致，直接读 draw 的常量池（类名为 VIEW3D_PT_cn_bip_v45）
    consts = list(m.VIEW3D_PT_cn_bip_v45.draw.__code__.co_consts)
except Exception:
    pass
check('I1 面板版本标记 v3.x', any(('v3.0' in str(c) or 'v3.1' in str(c)
                                  or 'v3.2' in str(c)) for c in consts))
check('I2 面板有模态隔离诊断行', any('模态隔离' in str(c) for c in consts))

# ------------------------------- K 真实收尾链路：消费授权后 v2.9 撤回修复仍在
# _finish / cancel / 交还成功都会调 _bip_solo_disarm()，必须确认这不会把
# 「拖完按 Ctrl+Z 又只剩手掌」的修复一起关掉。
reset()
bip_round()
m._bip_solo_disarm()
check('K1 正常收尾已消费授权', m._BIP_SOLO_ARMED is False)
set_cn_solo({HAND})
r8 = m.bip_solo_repair_after_undo('撤回')
check('K2 消费授权后撤回残留仍修复',
      r8 and diff(hide_map(), BASE) == 0, 'wrong=%d' % diff(hide_map(), BASE))
check('K3 修复后可见骨数复原', len(visible()) == CLEAN_VIS,
      '%d/%d' % (len(visible()), CLEAN_VIS))

# ------------------------------------------------------------------ 现场还原
reset()
check('J1 测试结束显隐复原', diff(hide_map(), BASE) == 0)

ok = all(v for _, v in PASS)
print('V30 RESULT %s  (%d/%d)' %
      ('ALL_PASS' if ok else 'FAIL', sum(v for _, v in PASS), len(PASS)))
if not ok:
    print('V30 FAILED', [n for n, v in PASS if not v])
