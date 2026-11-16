"""复现：BIP 拖动完成后按 Ctrl+Z，骨架回到「只有手掌控制器」的独显状态。

只读打开真实工程，不保存。用几种不同的 undo step 组合模拟 Blender 在
BIP modal 期间可能产生的撤销边界，找出到底是哪一步把 hide 记进了历史。
"""
import bpy
import importlib

m = importlib.import_module('controller_navigator_v4971')
rig = bpy.data.objects.get('rig.016') or next(
    o for o in bpy.data.objects
    if o.type == 'ARMATURE' and 'c_hand_fk.r' in o.pose.bones)
ctx = bpy.context
m._BIP_SOLO_FORCE = True

HAND = 'c_hand_fk.r'


def snap():
    return {b.name: bool(b.hide) for b in rig.pose.bones}


def reset():
    m.bip_solo_forget()
    m._BIP_SOLO_CLEAN = None
    for b in rig.pose.bones:
        b.hide = False
    m._BIP_DRAG_ACTIVE = False


base = snap()
print('DIAG rig=%s bones=%d base_hidden=%d' %
      (rig.name, len(base), sum(base.values())))


def run(tag, steps):
    """steps: 一串 callable，按顺序执行；最后统一 undo 一次。"""
    reset()
    before = snap()
    try:
        for s in steps:
            s()
    except Exception as exc:
        print('DIAG %s STEP ERR %s' % (tag, exc))
    mid = snap()
    try:
        r = bpy.ops.ed.undo()
        undo_ret = str(r)
    except Exception as exc:
        undo_ret = 'EXC ' + str(exc)[:80]
    after = snap()
    print('DIAG %-22s before=%3d after_steps=%3d after_undo=%3d '
          'diff_from_base=%3d undo=%s' %
          (tag, sum(before.values()), sum(mid.values()),
           sum(after.values()),
           sum(1 for k in after if after[k] != base[k]), undo_ret))
    return after


def apply():
    m.bip_solo_apply(ctx, rig, HAND, {HAND})


def restore():
    m.bip_solo_restore(ctx)


def push(msg='x'):
    def f():
        bpy.ops.ed.undo_push(message=msg)
    return f


def kf():
    def f():
        p = rig.pose.bones[HAND]
        p.keyframe_insert(data_path='location',
                          frame=bpy.context.scene.frame_current, group=HAND)
    return f


# S0：最朴素 —— apply / restore 之后直接撤销
run('S0 apply-restore', [apply, restore])
# S1：apply 之后有一次 undo_push（模拟 BIP 期间插帧 / 路径算子建立撤销边界）
run('S1 push-after-apply', [apply, push('BIP key'), restore])
# S2：apply 前 push，apply 后 push，restore 后 push（每个边界都记显隐）
run('S2 push-everywhere',
    [push('pre'), apply, push('BIP key'), restore, push('BIP end')])
# S3：真实插帧
run('S3 keyframe', [apply, kf(), restore])
# S4：restore 之后再 push 一次（BIP 带 UNDO，收尾可能自己 push）
run('S4 push-after-restore', [apply, restore, push('BIP op')])
# S5：apply / keyframe / restore / paths_update
def paths():
    def f():
        try:
            bpy.ops.pose.paths_update()
        except Exception as exc:
            print('DIAG paths_update:', str(exc)[:70])
    return f
run('S5 paths-update', [apply, kf(), restore, paths()])

reset()
print('DIAG DONE')
