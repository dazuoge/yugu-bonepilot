# -*- coding: utf-8 -*-
"""只读 A/B 复刻：驭骨自写 IK/FK 切换 vs ARP 原生切换

用法： blender --background "<blend>" --python _repro_cn_ikfk.py -- <base|cn|arp>

不保存工程、不修改插件文件。三段用独立 Blender 进程跑，基线完全一致。
"""
import bpy
import sys
import math
import importlib

MODE = 'base'
BASE = 'clean'          # clean / 2pi / orig
if '--' in sys.argv:
    tail = sys.argv[sys.argv.index('--') + 1:]
    if tail:
        MODE = tail[0]
    if len(tail) > 1:
        BASE = tail[1]

m = importlib.import_module('controller_navigator_v4971')
scn = bpy.context.scene
rig = bpy.data.objects.get('rig.016')
BONES = ['c_arm_fk.r', 'c_forearm_fk.r', 'c_hand_fk.r']
SW = 'c_hand_ik.r'
F0, F1 = 1, 70
KEYS = (1, 20, 40, 60)
FRAME = 24


def out(*a):
    print('RPT', *a)


def all_fcurves():
    ad = rig.animation_data
    return m._bip_action_fcurves(ad.action) if (ad and ad.action) else []


def reset_clean(shift=0.0):
    """把右臂 FK 链重建成平滑、无分支跳变的动画（等价于 BIP 写出来的曲线）。

    shift != 0：把 euler 第 3 轴整体平移 2π —— 姿态完全等价，但曲线落在
    「非主分支」上，这正是 BIP v2.3 连续等价表示对齐后的真实形态。
    """
    for bn in BONES:
        pb = rig.pose.bones[bn]
        path = m.bip_rotation_path(pb)
        n = 3 if path == 'rotation_euler' else 4
        full = 'pose.bones["%s"].%s' % (bn, path)
        chans = {}
        for fc in all_fcurves():
            if fc.data_path == full:
                chans[fc.array_index] = fc
        if any(i not in chans for i in range(n)):
            return False
        for i in range(n):
            fc = chans[i]
            if path == 'rotation_euler':
                base = (1.0, 0.0, 0.0, 0.0)[i] if i == 0 else 0.0
                vals = [base + d for d in (0.10, 0.26, 0.42, 0.24)]
                if bn == 'c_hand_fk.r':
                    vals = [base + d for d in (-0.10, 0.02, 0.14, 0.04)]
            else:
                vals = ([1.0, 0.98, 0.96, 0.98] if i == 0 else
                        [0.0, 0.10, 0.20, 0.10] if i == 1 else
                        [0.0, 0.02, 0.04, 0.02] if i == 2 else
                        [0.0, 0.0, 0.0, 0.0])
            while len(fc.keyframe_points):
                fc.keyframe_points.remove(fc.keyframe_points[0])
            for f, v in zip(KEYS, vals):
                extra = shift if (path == 'rotation_euler' and i == 2) else 0.0
                fc.keyframe_points.insert(f, v + extra)
            fc.update()
    bpy.context.view_layer.update()
    return True


def curve_map():
    """全部 FK/IK 控制器相关曲线 + 开关曲线的 (帧, 值) 序列。"""
    d = {}
    for fc in all_fcurves():
        p = fc.data_path
        if not any(('"%s"' % b) in p for b in BONES + [SW]):
            continue
        d[(p, fc.array_index)] = [
            (round(float(k.co[0]), 4), round(float(k.co[1]), 6))
            for k in fc.keyframe_points]
    return d


def sw_curve():
    p = 'pose.bones["%s"]["ik_fk_switch"]' % SW
    for fc in all_fcurves():
        if fc.data_path == p:
            return [(round(float(k.co[0]), 4), round(float(k.co[1]), 6))
                    for k in fc.keyframe_points]
    return None


def measure(tag):
    prev, jumps, pose = {}, [], {}
    for f in range(F0, F1 + 1):
        scn.frame_set(f)
        bpy.context.view_layer.update()
        for bn in BONES:
            q = (rig.matrix_world @ rig.pose.bones[bn].matrix).to_quaternion()
            if f in KEYS:
                pose[(f, bn)] = q.copy()
            if bn in prev:
                d = min(1.0, abs(prev[bn].dot(q)))
                ang = math.degrees(2 * math.acos(max(-1.0, min(1.0, d))))
                if ang > 8.0:
                    jumps.append((f, bn, ang))
            prev[bn] = q
    worst = max((a for _, _, a in jumps), default=0.0)
    bad = sorted(jumps, key=lambda t: -t[2])[:3]
    out('MEASURE', tag, 'jumps>8=%d' % len(jumps), 'worst=%.2f' % worst,
        'top=%s' % [(f, b, round(a, 1)) for f, b, a in bad])
    return {'n': len(jumps), 'worst': worst, 'pose': pose}


def pose_diff(a, b):
    ds = []
    for k, qa in a['pose'].items():
        qb = b['pose'].get(k)
        if qb is None:
            continue
        d = min(1.0, abs(qa.dot(qb)))
        ds.append(math.degrees(2 * math.acos(max(-1.0, min(1.0, d)))))
    return max(ds) if ds else 0.0


# ---------------------------------------------------------------- 探针
hits = {'native': 0, 'calc_matrix': 0, 'calc_generic': 0,
        'key_limb': 0, 'arp_exec': 0}
_orig = {}


def install_probes():
    def wrap(name, key):
        fn = getattr(m, name, None)
        if fn is None:
            return
        _orig[name] = fn

        def w(*a, **kw):
            hits[key] += 1
            return fn(*a, **kw)
        setattr(m, name, w)

    wrap('_native_call', 'native')
    wrap('_snap_by_matrix', 'calc_matrix')
    wrap('_snap_generic', 'calc_generic')
    wrap('_key_limb', 'key_limb')
    # ARP 原生算子 execute（v3.1 已包一层，这里再包一层计数）
    cls = m._bip_ikfk_find_class() if hasattr(m, '_bip_ikfk_find_class') else None
    if cls is not None and 'execute' in cls.__dict__:
        ex = cls.__dict__['execute']

        def w(self, context):
            hits['arp_exec'] += 1
            return ex(self, context)
        cls.execute = w


def select_hand_fk():
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


# ---------------------------------------------------------------- 主流程
out('MODE', MODE)
out('BLEND', bpy.data.filepath)
kind, limbs = m.ikfk_limbs(rig)
out('LIMBS kind=%s count=%d' % (kind, len(limbs)))
for lb in limbs:
    if '.r' in lb.prop_bone and ('arm' in lb.prop_bone.lower()
                                 or 'hand' in lb.prop_bone.lower()):
        out('LIMB_R', 'label=%s prop=%s/%s' % (lb.label, lb.prop_bone, lb.prop_name),
            'fk_value=%s ik_value=%s' % (lb.fk_value, lb.ik_value),
            'rigify_to_fk=%s rigify_to_ik=%s' % (lb.rigify_to_fk, lb.rigify_to_ik),
            'pairs_to_fk=%d pairs_to_ik=%d'
            % (len(lb.pairs_to_fk or ()), len(lb.pairs_to_ik or ())),
            'fk_controls=%s' % (list(lb.fk_controls),),
            'ik_controls=%s' % (list(lb.ik_controls),))
        out('NATIVE_POSSIBLE',
            bool(lb.rigify_to_fk or lb.rigify_to_ik))

out('ARP_OP_AVAILABLE', hasattr(bpy.ops.pose, 'arp_switch_snap'))
out('CN_OP_AVAILABLE', hasattr(bpy.ops.pose, 'cn_ikfk_toggle_v45'))

if BASE == 'orig':
    ok = 'orig (工程原样曲线，不重建)'
elif BASE == '2pi':
    ok = reset_clean(2 * math.pi)
else:
    ok = reset_clean()
out('RESET_CLEAN base=%s ok=%s' % (BASE, ok))
scn.frame_set(FRAME)
bpy.context.view_layer.update()
A = measure('baseline')
base_curves = curve_map()
base_sw = sw_curve()
out('BASE curves=%d keys=%d sw=%s'
    % (len(base_curves), sum(len(v) for v in base_curves.values()), base_sw))
out('BASE switch_value_frame24',
    float(rig.pose.bones[SW]['ik_fk_switch']))

if MODE == 'arpraw':
    # 测「裸 ARP 原生切换」：卸掉 v3.1 保护，避免把保护效果算成原生效果
    try:
        m._bip_remove_ikfk_guard()
        out('GUARD_REMOVED', True)
    except Exception as exc:
        out('GUARD_REMOVED_ERR', repr(exc))
elif MODE == 'cnallv31':
    # 对照：卸掉 v3.2 原生接管，回到 v3.1 自算路径
    m._bip_remove_arp_switch()
    out('V31_BASELINE', True)
elif MODE in ('nativeonly', 'nopluginkey', 'both'):
    # 隔离实验：只换 snap / 只关掉 _key_limb 全量插帧 / 两个都改
    real_switch = m.limb_switch

    def _native_switch(rig_, lb_, target_, hide_other=True):
        select_hand_fk()
        r = bpy.ops.pose.arp_switch_snap()
        return ('FINISHED' in r), 'arp_native:%s' % (r,)

    if MODE in ('nativeonly', 'both'):
        m.limb_switch = _native_switch
    if MODE in ('nopluginkey', 'both'):
        m._key_limb = lambda rig_, lb_: 0
    out('PATCH', MODE, 'limb_switch_native=%s key_limb_disabled=%s'
        % (MODE in ('nativeonly', 'both'), MODE in ('nopluginkey', 'both')))
# measure() 会把 frame_current 留在 F1，切回目标帧再插帧
scn.frame_set(FRAME)
bpy.context.view_layer.update()
install_probes()
select_hand_fk()
old_auto = scn.tool_settings.use_keyframe_insert_auto
scn.tool_settings.use_keyframe_insert_auto = True

try:
    if MODE in ('cn', 'nativeonly', 'nopluginkey', 'both',
                'cnall', 'cnallv31'):
        for tgt in ('IK', 'FK'):
            kw = dict(target=tgt)
            if MODE in ('cnall', 'cnallv31'):
                kw['all_limbs'] = True
            else:
                # 强制指定肢体，避免无头下选中态丢失
                lb_r = [lb for lb in limbs if '.r' in lb.prop_bone
                        and ('arm' in lb.prop_bone.lower()
                             or 'hand' in lb.prop_bone.lower())]
                if lb_r:
                    kw['label'] = lb_r[0].label
            try:
                r = bpy.ops.pose.cn_ikfk_toggle_v45(**kw)
                out('CN_SWITCH', tgt, r, 'sw=%.3f'
                    % float(rig.pose.bones[SW]['ik_fk_switch']))
            except Exception as exc:
                out('CN_SWITCH_ERR', tgt, repr(exc))
    elif MODE in ('arp', 'arpraw'):
        for i in range(2):
            try:
                r = bpy.ops.pose.arp_switch_snap()
                out('ARP_SWITCH', i + 1, r, 'sw=%.3f'
                    % float(rig.pose.bones[SW]['ik_fk_switch']))
            except Exception as exc:
                out('ARP_SWITCH_ERR', i + 1, repr(exc))
finally:
    scn.tool_settings.use_keyframe_insert_auto = old_auto

out('PROBES', 'native=%d calc_matrix=%d calc_generic=%d key_limb=%d arp_exec=%d'
    % (hits['native'], hits['calc_matrix'], hits['calc_generic'],
       hits['key_limb'], hits['arp_exec']))
out('IKFK_GUARD_LOG', getattr(m, '_BIP_IKFK_LOG', 'n/a'))

B = measure('after_%s' % MODE)
out('POSE_DIFF_KEYFRAMES %.4f deg' % pose_diff(A, B))

after_curves = curve_map()
after_sw = sw_curve()
changed, added, removed, key_diff = [], [], [], []
for k in sorted(set(base_curves) | set(after_curves)):
    a = base_curves.get(k)
    b = after_curves.get(k)
    if a is None:
        added.append(k)
    elif b is None:
        removed.append(k)
    elif a != b:
        changed.append(k)
        key_diff.append((k[0].split('"]')[-1], k[1], len(a), len(b),
                         max(abs(x[1] - y[1]) for x, y in zip(a, b))
                         if len(a) == len(b) else -1))
out('CURVES', 'base=%d after=%d changed=%d added=%d removed=%d'
    % (len(base_curves), len(after_curves), len(changed),
       len(added), len(removed)))
out('ADDED_CURVES', [k[0] + '[%d]' % k[1] for k in added][:14])
out('CHANGED_DETAIL', [(a, i, na, nb, round(d, 4))
                       for a, i, na, nb, d in key_diff][:14])
out('SW_CURVE', 'base=%s after=%s' % (base_sw, after_sw))


def f24(cm):
    d = {}
    for (p, i), pts in cm.items():
        if 'rotation_' not in p:
            continue
        bone = p.split('"')[1] if '"' in p else '?'
        short = p.split('"]')[-1].lstrip('.')
        for x, y in pts:
            if abs(x - FRAME) < 0.01:
                d['%s.%s[%d]' % (bone, short, i)] = y
    return d


out('F24_BASE', f24(base_curves))
out('F24_AFTER', f24(after_curves))
out('DONE', MODE, BASE)
