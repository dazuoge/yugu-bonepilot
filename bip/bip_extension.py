# >>> ARP BIP EXT BEGIN (WorkBuddy) >>>
#
# ARP BIP Drag —— 内嵌于「驭骨 · BonePilot」的扩展块（自包含，不依赖独立插件）。
#
# 触发：选中「FK 手控骨」且该侧处于 FK 模式时，按驭骨自己的「进入移动」键
#       （默认裸 Z，跟随偏好设置）即进入整臂 BIP 拖动。其它任何骨骼 / IK 模式 /
#       关闭开关 / 解析不出映射，都原样走驭骨自己的 MOVE 模态。
#
# 通用性：不写死 c_hand_fk 之类的名字。以当前活动骨为锚点：
#   1) 侧别：.l/.L/_l/Left/左/右/驼峰 … 多种写法，实在没有线索就按 rest 空间 x 回退；
#   2) FK 链：优先沿 parent 链向上取两节控制骨，再按「前臂/上臂 + FK」关键词打分兜底；
#   3) IK 链 / IK 目标：按「同侧 + 角色 + IK 标记 + 带 ik_fk_switch」打分，
#      带 ik_fk_switch 属性的手控骨优先当 IK 目标；
#   4) 任一必需角色缺失 -> 不接管，绝不误改姿态。
#   角色词表含中英文（hand/wrist/palm/手/腕/掌、forearm/elbow/前臂/肘、
#   arm/shoulder/上臂/肩…），腿/脚类词直接排除。
#
# 原理：ARP 的 IK 求解链在 FK 模式下仍在求解，只是变形骨改由 FK 链驱动。
#       拖动期间把 IK 手控当临时目标，读 ARP 解出的姿态写回两根 FK 控制骨的
#       旋转通道；松手还原目标与约束。不切 ik_fk_switch、不新增约束。
#       v1.8 防跳变：① 以「目标刚定位到手位置」时的 IK 链为基线，每帧只把
#       IK 链的相对变化叠加到 FK 初始姿态（否则第一次移动就把两者 1~19° 的
#       初始差整笔灌进 FK）；② 只写 FK 控制骨的旋转通道，绝不碰 location ——
#       真实工程里 c_arm_fk.r 带位置关键帧，旧写法一步就改掉 3.5 cm。
#       每帧把 FK 手控骨的世界朝向补偿回拖动开始时的值（BIP 拖手不改腕朝向）。
#       v1.9 插帧连续性：同一个旋转有多组等价数值表示（q / -q、欧拉的多组
#       解、轴角 ±2pi）。Blender 的旋转 F-Curve 是逐分量插值的，落错分支时
#       终点姿态一样、中间却会绕远路：实测真实工程 c_arm_fk.r 的欧拉三个
#       分量同时跳 ~pi，22->27 每帧转 34~48 度，上臂整段乱转。现在写通道与
#       插关键帧前都挑「与拖动起点 / 相邻关键帧连续」的等价表示，姿态零改变。
#       v2.0 铰链约束：ARP 的前臂 FK 控制骨 lock_rotation=(T,T,F)，只允许绕
#       一个轴弯肘，起点四元数只有 W/Z 非零；IK 链解出的却是完整 3-DOF 旋转，
#       旧写法把 X/Y 也灌进前臂通道（实测 -0.09 ~ -0.49），与骨上的
#       LIMIT_ROTATION 约束打架，用户看到「本来锁死的轴线开始出数值」。现在
#       写回时把前臂投影回它自己的允许子群（被锁分量保持拖动起点值），再让
#       3-DOF 自由的上臂迭代补偿肩->手的方向，手位置不但不丢反而更准。
#       只对检测到锁定通道的前臂启用，其它骨架 / 全自由前臂保持原行为。
#       v2.0 同时把欧拉等价解改成 Blender 内置 to_euler(order, euler_compat)，
#       任意 XYZ/YZX/ZXY/XZY/YXZ/ZYX 顺序通用（旧硬编码翻转公式只对 XYZ 最优）。
#       v2.1 近伸直 / IK 失配：手臂几乎完全伸直时 ARP 的 IK 链处在奇异区，
#       实测 E 踏步回斩 frame 29 右臂伸直度 0.996、IK 链解与 FK 姿态差 95.4 度；
#       往内拖（需要弯肘）时 IK 链靠 stretch 吸距离而几乎不弯肘，铰链投影把
#       解整笔丢掉 -> 手完全不动，上臂补偿发散，第 11/17 步出现 108/212 度的
#       猛跳、肘位置跳 23cm。现在检测到「单轴铰链 + IK/FK 姿态差 >= 20 度」
#       就整段改用解析二连杆：余弦定理由肩->手距离求前臂铰链角、绕肩对准
#       方向、距离闭环吸收约束误差；符号由几何判定（左右臂自动镜像）。
#       完全绕开 IK 奇异，实测内拖误差 0.3mm、手位移精确等于目标、零猛跳。
#       正常场景（IK/FK 差 < 20 度）仍走 v2.0 的 IK 链路径，行为不变。
#       v2.2 可往外拉直：v2.1 为避开近奇异区把伸展上限钳在「本次拖动起点
#       距离 D0」，用户先拉弯松手后再拖时新 D0 是弯的，等于永久禁止伸直
#       （实测往外 12cm 手位移 0.00000、肘角 134->85 度后回不去）。现在
#       Dmax 改为二连杆几何臂长 L1+L2，允许一路拉到完全伸直，超长才钳位；
#       近伸直区 sin(beta)->0 导致牛顿步长爆炸（outward 首步猛跳真因）改由
#       单轮步长上限 + theta 物理区间 [beta0-pi, beta0] 钳位消除。
#       v2.3 旋转模式插帧连续性：驭骨自由旋转每帧写 matrix_basis，Blender
#       会重新分解出规范分支的旋转通道值；真实 ARP 工程的 F-Curve 常在
#       -120/-210/-150 度这种远分支上，直接插帧会让中间帧绕 200 度远路
#       （实测单帧 123.5 度，原生 gizmo 只有 2.8 度）。现在包装驭骨的
#       _insert_keys，插帧前把旋转通道换成与前后关键帧连续的等价表示，
#       BIP 收尾插帧共用同一函数；另提供「修复旋转曲线分支」按钮，把以前
#       写坏的旧关键帧也归一化（只改数值表示和手柄，姿态逐帧零改变）。
#       v2.4 兼容 5.3「转换旋转模式」：该命令会把骨骼 rotation_mode 直接
#       设成 ZYX/XZY/... 六种轴序之一并整组重写动画曲线。实测
#       `pb.rotation_euler[:] = vals` 这种切片赋值会把 rotation_euler.order
#       与 rotation_mode 一并打回 XYZ（ZYX 骨骼姿态当场偏 55.5 度、YXZ 偏
#       152.8 度），而曲线数值仍按新轴序分解 —— 轴序一转换，旋转就暴跳。
#       现在所有通道写入统一走 bip_set_rot_values（逐分量赋值 + 轴序守护），
#       所有解释曲线值的地方统一走 bip_euler_order（以 rotation_mode 为准），
#       参考缓存也把轴序放进 key；六种欧拉 + 四元数 + 轴角全部验证通过。
#       v2.5 运动路径实时联动：原生 transform 结束即同步重算，而驭骨把重算
#       排进 timer（50 ms + 等手停 0.30 s + 0.2 s 一轮 + 重算 190 ms），
#       松手后要 0.5~0.7 s 路径才动。全量重算不能逐事件调用（会冻结事件
#       循环），但路径「当前帧」那个点是可写缓存：实测 points[i].co 赋值后
#       view_layer.update() 不覆盖、只有 paths_update() 才重算。现在拖动中
#       只写当前帧点为骨骼实时头部世界坐标（O(路径数)，depsgraph 刷新
#       8.5 ms 节流到 30 Hz），松手时同步全量重算；索引 frame-frame_start、
#       世界坐标均由真实工程实测。驭骨 MOVE/ROT 与 BIP 拖手两条链路共用。
#       v2.6 多帧画弧不再暴跳：真实工程里 ARP 铰链前臂的 X/Y 被锁成 0，
#       Blender 自动插帧不会给这两个通道补关键点，于是某些帧只有 W/Z 有键
#       （实测 E 踏步回斩 c_forearm_fk.r 的 f40：W=+0.9467，前后帧 W 都约
#       -0.99，整组四元数被翻了符号）。旧修复工具只取「四个通道共有」的帧，
#       恰好把 f40 排除，检查/修复都报 0 帧；这一帧留在曲线里，f34/f35 转
#       52.8/122.4 度、f44/f45 转 94.3/55.9 度，运动路径在 40~47 附近打结。
#       现在改成取并集帧，并对「缺通道的帧」做姿态等价校验后才改写；更关键的是
#       BIP 每次收尾插帧、以及驭骨自己的自由旋转/移动插帧后，都自动对这次
#       涉及的 FK 骨做一次局部归一化（只换数值表示，关键帧姿态逐帧零改变），
#       画弧时相邻几帧里残留的旧脏帧会被顺手理顺，无需手动点「修手臂」。
#       真实操作符 invoke 在开拖前也做一次（Auto Key 开启时），避免第一下
#       正好落在脏区间里把错姿态插成新关键帧；直接调 bip_begin 的求解器
#       回归不受影响，保证 v1.9~v2.5 的求解行为一字不变。
#       v2.7 BIP 独显控制器：驭骨的 _solo_apply 长在它自己的模态操作符里，
#       BIP 却在 invoke 分流阶段就接管了，从没进过那个模态 —— 这就是拖手掌
#       时别的控制器全都还在的原因。现在 BIP 复用同一套机制（偏好
#       solo_display、只写 PoseBone.hide、进模态前的 _NAV_VIS 快照）：
#       开拖抓一次快照并只显示手控骨（多选保留全部选中）；换手只换可见集；
#       _NAV_VIS 保持开拖前的可见性，Alt+滚轮切骨不会被自己临时藏的骨挡住；
#       确认 / 取消 / 异常 / 交还驭骨 / 卸载插件一律按快照原样还原。
#       交还前必须先还原：否则驭骨把「已隐藏」当作进模态前的状态抓成快照，
#       切一次骨之后整副骨架的控制器会永久看不见（真实可复现的坑）。
#
# 切骨联动：拖动中按「切骨」手势（默认 Alt+滚轮，跟随驭骨偏好）时，调用驭骨
#       自己的 pose.controller_nav_v45 切一格，再按新骨能力交还它的 ROT/MOVE
#       模态。切到上臂就自动变成旋转，不会再卡在手掌移动里。
#
# 路径联动（v2.5 拖动中跟手 + 收尾零延后）：
#       拖动中只写当前帧路径点（O(1) 缓存，不冻结事件循环）；每次有效拖动
#       收尾（确认 / 切骨交还 / 换手重锚 / 驭骨 MOVE·ROT 提交）当场同步调用
#       pose.paths_update() 重算**已有**的路径，松手瞬间整条曲线就是新的；
#       同时取消驭骨 _mpath_queue 里针对本骨架的待刷项，避免二次计算。
#       偏好开关、override、只刷不新增、错误冲刷、重绘全部沿用驭骨原生机制。
#       对驭骨的两个包装都只发生在运行时，unregister 会原样还原。
#
# 本块只使用 bpy 与标准库；与驭骨原有代码之间只有 3 个挂钩点：
#   POSE_OT_cn_quick_rotate_v45.invoke 开头 / register() / unregister()。
# 由 install_bip_patch.py 幂等植入，升级驭骨后重跑该脚本即可恢复。

import re as _bip_re
import math as _bip_math
import time as _bip_time
import itertools as _bip_iter
import bpy as _bip_bpy
from bpy.props import (StringProperty as _BipString,
                       EnumProperty as _BipEnum,
                       BoolProperty as _BipBool)
from bpy_extras import view3d_utils as _bip_v3d
from mathutils import (Vector as _BipVector,
                       Quaternion as _BipQuaternion,
                       Euler as _BipEuler)

_BIP_OWNED = False
_BIP_FINE = 0.15
_BIP_NAV_SETTLE = 0.08       # 原生导航模态退场的时间下限（与驭骨一致）

# 视角类事件：一律放行给 Blender 原生键位，绝不吞。
# 不含 NUMPAD_ENTER（它是「确认」），不含 Z/X/M（放行会二次触发驭骨模态）。
_BIP_VIEW_PASS = {
    'WHEELUPMOUSE', 'WHEELDOWNMOUSE', 'MOUSEROTATE',
    'TRACKPADPAN', 'TRACKPADZOOM', 'HOME',
    'NUMPAD_0', 'NUMPAD_1', 'NUMPAD_2', 'NUMPAD_3', 'NUMPAD_4',
    'NUMPAD_5', 'NUMPAD_6', 'NUMPAD_7', 'NUMPAD_8', 'NUMPAD_9',
    'NUMPAD_PERIOD', 'NUMPAD_SLASH', 'NUMPAD_PLUS', 'NUMPAD_MINUS',
    'NUMPAD_ASTERIX',
}
_BIP_MOVES = {'MOUSEMOVE', 'INBETWEEN_MOUSEMOVE'}
_BIP_NAV_LOG = '未使用'      # 面板诊断：上一次视图导航的启动结果
_BIP_WARP_LOG = '未使用'     # 面板诊断：上一次光标归位的结果
_BIP_MPATH_LOG = '未使用'    # 面板诊断：上一次运动路径刷新的结果
_BIP_MODE_LOG = '未开始'     # 面板诊断：上一次拖动用的求解模式（v2.1）

# 身体中枢 / 头颈类「命名 token」：命中即绝不接管，按 Z 必须走驭骨原生 MOVE。
# 用 token 精确匹配而不是子串，避免误伤 c_hand_fk / c_forearm_fk 这类名字。
_BIP_BODY_TOKENS = {
    'root', 'roots', 'master', 'traj', 'trajectory', 'global', 'world',
    'cog', 'pelvis', 'hip', 'hips', 'spine', 'chest', 'torso', 'body',
    'trunk', 'neck', 'head', 'bend', 'offset', 'center', 'centre', '重心',
}
_BIP_BODY_CN = ('质心', '重心', '躯干', '骨盆', '脊椎', '脊柱',
                '颈', '头', '世界', '全局', '根骨', '腰')

_BIP_HAND = ('hand', 'wrist', 'palm', '手', '腕', '掌')
_BIP_FORE = ('forearm', 'lowerarm', 'lower_arm', 'elbow', '前臂', '小臂', '肘')
_BIP_UPPER = ('upperarm', 'upper_arm', 'arm', 'shoulder', 'clavicle',
              '上臂', '大臂', '臂', '肩')
_BIP_LEG = ('foot', 'feet', 'ankle', 'toe', 'toes', 'leg', 'thigh', 'shin',
            'calf', 'knee', '脚', '足', '踝', '趾', '腿', '膝')
_BIP_FINGER = ('finger', 'thumb', 'index', 'middle', 'pinky', 'pinkie',
               '指', '拇')
_BIP_CTRL = ('ctrl', 'ctl', 'control', 'widget', 'wgt')


# ------------------------------------------------------------ 名称 / 结构解析
def bip_side_of_name(name):
    """从名字解析侧别，支持 .l / _L / -left / 左 / Right / 驼峰 LeftHand 等。"""
    if not name:
        return None
    low = name.lower()
    for token, side in (('left', '.l'), ('right', '.r'),
                        ('左', '.l'), ('右', '.r')):
        if token in low:
            return side
    parts = [p for p in _bip_re.split(r'[._\-\s]+', name) if p]
    for p in reversed(parts):
        if p.lower() in ('l', 'left'):
            return '.l'
        if p.lower() in ('r', 'right'):
            return '.r'
    if _bip_re.search(r'(^|[a-z])L([A-Z]|$)', name):
        return '.l'
    if _bip_re.search(r'(^|[a-z])R([A-Z]|$)', name):
        return '.r'
    return None


def _bip_x_ref(rig):
    """用骨架里带侧别标记的骨，学出「+x 是左还是右」以及左右分界。"""
    lx, rx = [], []
    for b in rig.data.bones:
        s = bip_side_of_name(b.name)
        if s == '.l':
            lx.append(b.head_local.x)
        elif s == '.r':
            rx.append(b.head_local.x)
    if lx and rx:
        lm, rm = sum(lx) / len(lx), sum(rx) / len(rx)
        return ((lm + rm) / 2.0, '.l' if lm > rm else '.r')
    return (0.0, '.l')


def bip_side_of_pb(pb, rig, xref=None):
    s = bip_side_of_name(pb.name)
    if s:
        return s
    mid, pos_side = xref or _bip_x_ref(rig)
    try:
        x = pb.bone.head_local.x
    except Exception:
        return None
    hit = '.l' if x >= mid else '.r'
    return pos_side if hit == '.l' else ('.r' if pos_side == '.l' else '.l')


def bip_role(name):
    low = (name or '').lower()
    if any(w in low for w in _BIP_FINGER):
        return 'finger'
    if any(w in low for w in _BIP_LEG):
        return 'leg'
    if any(w in low for w in _BIP_FORE):
        return 'forearm'
    if any(w in low for w in _BIP_HAND):
        return 'hand'
    if any(w in low for w in _BIP_UPPER):
        return 'upper'
    return None


def bip_is_fk(name):
    low = (name or '').lower()
    return 'fk' in low or 'forward' in low


def bip_is_ik(name):
    low = (name or '').lower()
    return 'ik' in low and 'fk' not in low


def bip_is_ctrl(pb):
    try:
        if pb.custom_shape is not None:
            return True
    except Exception:
        pass
    low = pb.name.lower()
    if low.startswith('c_'):
        return True
    return any(w in _bip_re.split(r'[._\-\s]+', low) for w in _BIP_CTRL)


def bip_is_body_ctrl(name):
    """质心 / 根 / 躯干 / 头颈类控制器 —— 按 Z 必须走驭骨原生 MOVE，不接管。"""
    low = (name or '').lower()
    if any(w in low for w in _BIP_BODY_CN):
        return True
    tokens = [t for t in _bip_re.split(r'[._\-\s]+', low) if t]
    return any(t in _BIP_BODY_TOKENS for t in tokens)


def bip_parent_has_arm_chain(hand, limit=6):
    """结构准入：parent 链上必须真的挂着「前臂 + 上臂」两节控制骨。

    这是把质心/躯干/头/手指挡在门外的关键。只靠 x 坐标兜底 + 名称打分，
    ARP 的 c_root_master / c_spine_01 / c_ring1 都能凑出一条假手臂链；
    但它们的父链上不存在 c_forearm_fk + c_arm_fk 这样的控制骨。
    """
    fore = upper = False
    p, steps = hand.parent, 0
    while p is not None and steps < limit:
        if bip_is_ctrl(p):
            r = bip_role(p.name)
            if r == 'forearm':
                fore = True
            elif r == 'upper':
                upper = True
        if fore and upper:
            return True
        p, steps = p.parent, steps + 1
    return False


def _bip_pick(cands, role, want_fk=None, want_ik=None,
              ctrl=None, prefer_switch=False, exclude=()):
    """同侧候选里按角色打分挑一根；返回 (name, score)，score<=0 视为没挑到。"""
    best, best_sc = None, -999
    for p in cands:
        if p.name in exclude or bip_role(p.name) != role:
            continue
        sc = 1
        if want_fk is True:
            sc += 4 if bip_is_fk(p.name) else -4
        elif want_fk is False and bip_is_fk(p.name):
            sc -= 4
        if want_ik is True:
            sc += 4 if bip_is_ik(p.name) else -4
        elif want_ik is False and bip_is_ik(p.name):
            sc -= 4
        if ctrl is True and not bip_is_ctrl(p):
            sc -= 3
        if ctrl is False and bip_is_ctrl(p):
            sc -= 3
        if prefer_switch and 'ik_fk_switch' in p.keys():
            sc += 6
        if sc > best_sc:
            best, best_sc = p, sc
    return (best.name, best_sc) if best is not None and best_sc > 0 else (None, 0)


def bip_is_arp_rig(obj):
    """ARP 标记优先；没有标记时，只要骨架带 ik_fk_switch 也按 ARP 风格放行。"""
    if obj is None or obj.type != 'ARMATURE':
        return False
    try:
        if 'arp_updated' in obj.data.keys() or 'arp_updated' in obj.keys():
            return True
    except Exception:
        pass
    try:
        return any('ik_fk_switch' in p.keys() for p in obj.pose.bones)
    except Exception:
        return False


def bip_fk_mode(rig, amap):
    name = amap.get('switch_bone')
    p = rig.pose.bones.get(name) if name else None
    if p is None:
        # 同名侧里再找一次带开关的骨
        for q in rig.pose.bones:
            if 'ik_fk_switch' in q.keys() and bip_side_of_pb(q, rig) == amap['side']:
                p = q
                amap['switch_bone'] = q.name
                break
    if p is None or 'ik_fk_switch' not in p.keys():
        return True          # 找不到开关就当 FK，不拦截
    try:
        return float(p['ik_fk_switch']) >= 0.5
    except Exception:
        return True


def bip_resolve(context, active=None):
    """以活动骨为锚点解析整条手臂映射；缺任一必需角色就返回 None。"""
    rig = context.active_object
    if not bip_is_arp_rig(rig):
        return None
    pbs = list(rig.pose.bones)
    hand = active or context.active_pose_bone
    if hand is None or hand.name not in rig.pose.bones:
        return None
    hand_role = bip_role(hand.name)
    # 上臂 / 前臂 / 腿 / 手指 一律不是「手控骨」。
    # 注意不能按「父骨也是手类」排除：ARP 在 c_hand_fk 上面还有一层
    # c_hand_fk_scale_fix，名字同样带 hand。
    if hand_role in ('leg', 'upper', 'forearm', 'finger') or not bip_is_ctrl(hand):
        return None
    if bip_is_ik(hand.name) and not bip_is_fk(hand.name):
        return None
    # 闸 A：质心 / 根 / 躯干 / 头颈一律交还驭骨，绝不抢 Z。
    if bip_is_body_ctrl(hand.name):
        return None
    # 闸 B：父链上必须真有「前臂 + 上臂」两节控制骨。
    # 没有的话，即便名字完全无语义（CTRL_07 之类）也不是手臂末端，
    # 只是靠 x 兜底 + 名称打分凑出了一条假链。
    if not bip_parent_has_arm_chain(hand):
        return None
    # 若下面还挂着前臂/上臂控制骨，说明选中的是链的上游而非末端。
    for c in hand.children:
        if bip_is_ctrl(c) and bip_role(c.name) in ('forearm', 'upper'):
            return None
    xref = _bip_x_ref(rig)
    side = bip_side_of_pb(hand, rig, xref)
    if side is None:
        return None
    same = [p for p in pbs if bip_side_of_pb(p, rig, xref) == side]
    excl = {hand.name}

    # FK 链：先沿 parent 链找两节控制骨（最不依赖命名）
    fk_fore = fk_upper = None
    p, steps = hand.parent, 0
    while p is not None and steps < 6:
        r = bip_role(p.name)
        if bip_is_ctrl(p):
            if r == 'forearm' and fk_fore is None:
                fk_fore = p.name
            elif r == 'upper' and fk_upper is None:
                fk_upper = p.name
        if fk_fore and fk_upper:
            break
        p, steps = p.parent, steps + 1
    # parent 链是控制骨但完全没语义词时，直接按层级顺序取父 / 祖父
    if fk_fore is None and hand.parent is not None and bip_is_ctrl(hand.parent):
        fk_fore = hand.parent.name
    if fk_upper is None and fk_fore is not None:
        gp = rig.pose.bones[fk_fore].parent
        if gp is not None and bip_is_ctrl(gp):
            fk_upper = gp.name
    # 名称打分兜底 / 校正
    if fk_fore is None:
        fk_fore, _ = _bip_pick(same, 'forearm', want_fk=True, ctrl=True, exclude=excl)
    if fk_upper is None:
        fk_upper, _ = _bip_pick(same, 'upper', want_fk=True, ctrl=True, exclude=excl)

    # IK 目标：带 ik_fk_switch 的手控骨优先，其次同侧 hand + ik
    ik_target, target_sc = _bip_pick(
        same, 'hand', want_ik=True, want_fk=False, prefer_switch=True,
        exclude=excl)
    if ik_target is None:
        ik_target, target_sc = _bip_pick(
            same, 'hand', ctrl=True, exclude=excl)
    # IK 求解链
    ik_fore, _ = _bip_pick(same, 'forearm', want_ik=True, want_fk=False,
                           exclude=excl | {fk_fore or ''})
    ik_upper, _ = _bip_pick(same, 'upper', want_ik=True, want_fk=False,
                            exclude=excl | {fk_upper or ''})
    if ik_fore is None:
        ik_fore, _ = _bip_pick(same, 'forearm', ctrl=False, want_fk=False,
                               exclude=excl | {fk_fore or ''})
    if ik_upper is None:
        ik_upper, _ = _bip_pick(same, 'upper', ctrl=False, want_fk=False,
                                exclude=excl | {fk_upper or ''})

    needed = (fk_fore, fk_upper, ik_target, ik_fore, ik_upper)
    if any(n is None for n in needed) or len(set(needed)) != len(needed):
        return None
    amap = {
        'rig': rig,
        'side': side,
        'fk_hand': hand.name,
        'fk_fore': fk_fore,
        'fk_upper': fk_upper,
        'ik_target': ik_target,
        'ik_fore': ik_fore,
        'ik_upper': ik_upper,
        'switch_bone': ik_target if target_sc > 0 else None,
        'fk': (fk_upper, fk_fore),
        'ik': (ik_upper, ik_fore),
    }
    if not bip_fk_mode(rig, amap):
        return None
    return amap


def bip_enabled():
    try:
        return bool(_bip_bpy.context.scene.cn_bip_enabled)
    except Exception:
        return True


def bip_hand_side(context):
    amap = bip_resolve(context) if bip_enabled() else None
    return amap['side'] if amap else None


def bip_should_takeover(context):
    return bip_hand_side(context) is not None


# ------------------------------------------------------------------ core
_BIP_EULER_ORDERS = ('XYZ', 'XZY', 'YXZ', 'YZX', 'ZXY', 'ZYX')


def bip_euler_order(pb):
    """取骨骼当前生效的欧拉轴序；权威来源是 rotation_mode，不是 Euler.order。

    Blender 5.3 新增「转换旋转模式」(anim.rotation_mode_convert) 后，骨骼的
    rotation_mode 会直接是 ZYX / XZY / ... 六种轴序之一，动画曲线里的数值
    也按该轴序重新分解。而 `pb.rotation_euler.order` 只是 Euler RNA 的属性，
    会被切片赋值重置（见 bip_set_rot_values），此时读它就会把新轴序的曲线
    数值当成 XYZ 解释，实测 ZYX 姿态偏 55°、YXZ 偏 153°。
    """
    mode = pb.rotation_mode
    if mode in _BIP_EULER_ORDERS:
        return mode
    try:
        return pb.rotation_euler.order
    except Exception:
        return 'XYZ'


def bip_set_rot_values(pb, vals):
    """逐分量写旋转通道，绝不切片赋值。

    Blender 5.3 实测（E 踏步回斩.blend / c_arm_fk.r）：
      pb.rotation_euler[j] = v   -> order 与 rotation_mode 保持，姿态 0.0°
      pb.rotation_euler[:] = v   -> order 与 rotation_mode 一并被打回 XYZ，
                                   ZYX 姿态偏 55.46°、YXZ 偏 152.77°
    事后补 `rotation_euler.order` 也救不回来（rotation_mode 已变）。
    以前所有骨骼都是 XYZ，切片赋值重置无害；5.3 转换轴序后，每次写对齐值
    都把骨骼打回 XYZ，而曲线仍是新轴序，中间插值与终点姿态就整段乱掉。
    """
    mode0 = pb.rotation_mode
    order0 = bip_euler_order(pb) if mode0 in _BIP_EULER_ORDERS else None
    ch = getattr(pb, bip_rotation_path(pb))
    for j in range(min(len(ch), len(vals))):
        ch[j] = vals[j]
    # 防御：某些版本/路径仍可能重置，立即修回来
    if pb.rotation_mode != mode0:
        try:
            pb.rotation_mode = mode0
        except Exception:
            pass
    if order0:
        try:
            if pb.rotation_euler.order != order0:
                pb.rotation_euler.order = order0
        except Exception:
            pass
    return True


def bip_basis_quat(pb):
    """取当前旋转通道对应的四元数（不触发依赖图求值）。"""
    mode = pb.rotation_mode
    if mode == 'QUATERNION':
        return pb.rotation_quaternion.copy().normalized()
    if mode == 'AXIS_ANGLE':
        a = pb.rotation_axis_angle
        return _BipQuaternion(_BipVector(a[1:]), a[0]).normalized()
    return _BipEuler(list(pb.rotation_euler),
                     bip_euler_order(pb)).to_quaternion().normalized()


def bip_best_repr(pb, q, refs):
    """挑一个与 q 姿态等价、且与 refs 里各参考通道值最连续的数值表示。

    refs: [(通道值列表, 权重), ...]。旋转通道不是单值的：四元数 q / -q、
    欧拉的多组解、轴角的 ±2pi 都表示同一姿态，但 F-Curve 逐分量插值，
    相邻关键帧落在不同分支就会绕远路（上臂整段乱转）。这里只换「数值表示」，
    不换姿态：每个候选都验证过转回四元数与 q 的点积绝对值 > 0.99999。
    """
    q = _BipQuaternion((q.w, q.x, q.y, q.z)).normalized()
    refs = [(list(r), float(w)) for r, w in refs if r is not None]
    mode = pb.rotation_mode
    dot = lambda a, b: abs(sum(x * y for x, y in zip(a, b)))

    if mode == 'QUATERNION':
        neg = _BipQuaternion((-q.w, -q.x, -q.y, -q.z))
        cands = [list(q), list(neg)]
        score = lambda v: sum(
            w * sum((v[j] - r[j]) ** 2 for j in range(4))
            for r, w in refs if len(r) == 4)
        return min(cands, key=score)

    if mode == 'AXIS_ANGLE':
        axis, angle = q.to_axis_angle()
        cands = []
        for k in range(-2, 3):
            cands.append([angle + 2 * _bip_math.pi * k,
                          axis.x, axis.y, axis.z])
            cands.append([-angle + 2 * _bip_math.pi * k,
                          -axis.x, -axis.y, -axis.z])
        good = [v for v in cands
                if dot(_BipQuaternion(_BipVector(v[1:4]), v[0]).normalized(), q)
                > 0.99999]
        if not good:
            return [angle, axis.x, axis.y, axis.z]
        score = lambda v: sum(
            w * sum((v[j] - r[j]) ** 2 for j in range(4))
            for r, w in refs if len(r) == 4)
        return min(good, key=score)

    # 欧拉：Blender 内置 to_euler(order, euler_compat) 直接给出「与参考无轴
    # 翻转」的等价解，对 XYZ / YZX / ZXY / XZY / YXZ / ZYX 全部适用。旧版
    # 硬编码的 (x±pi, pi-y, z±pi) 只对 XYZ 最优，实测 XZY/YXZ 下会选到比
    # 兼容解远 3 倍的分支（dist 19.1 vs 6.0）。
    order = bip_euler_order(pb)
    usable = [(r, w) for r, w in refs if len(r) == 3]
    wsum = sum(w for _, w in usable)
    rmean = ([sum(w * r[j] for r, w in usable) / wsum for j in range(3)]
             if wsum else [0.0, 0.0, 0.0])
    compat = list(q.to_euler(order, _BipEuler(rmean, order)))
    base = list(q.to_euler(order))
    two_pi = 2 * _bip_math.pi
    seeds = [tuple(compat), tuple(base)]
    for sx, sz in ((-1, -1), (-1, 1), (1, -1), (1, 1)):
        seeds.append((compat[0] + sx * _bip_math.pi,
                      _bip_math.pi - compat[1],
                      compat[2] + sz * _bip_math.pi))
    score = lambda e: sum(
        w * sum((e[j] - r[j]) ** 2 for j in range(3)) for r, w in usable)
    # 加权平方和可分：每个分量离加权参考均值最近的 2pi 倍数即该组解最优。
    cands = []
    for s in seeds:
        e = [s[j] + two_pi * round((rmean[j] - s[j]) / two_pi)
             for j in range(3)]
        cands.append((score(e), e))
    cands.sort(key=lambda t: t[0])
    for _, e in cands:
        if dot(_BipEuler(e, order).to_quaternion().normalized(), q) > 0.99999:
            return e
    # 极端兜底：多参考冲突时再全量枚举一次
    for s in seeds:
        for k in _bip_iter.product((-2, -1, 0, 1, 2), repeat=3):
            e = [s[j] + two_pi * k[j] for j in range(3)]
            if dot(_BipEuler(e, order).to_quaternion().normalized(), q) > 0.99999:
                return e
    return compat                    # 仍是同一姿态，且与参考同分支


def bip_rot_channels(rig, pb):
    """找该骨当前生效旋转通道的 F-Curve（兼容 Blender 4.4+ layered action）。"""
    ad = rig.animation_data
    act = ad.action if ad else None
    if act is None:
        return None
    path = bip_rotation_path(pb)
    full = 'pose.bones["%s"].%s' % (pb.name, path)
    n = 3 if path == 'rotation_euler' else 4
    chans = {}
    try:
        bags = [bag for layer in act.layers
                for strip in layer.strips
                for bag in strip.channelbags]
    except Exception:
        bags = []
    for bag in bags:
        try:
            for fc in bag.fcurves:
                if fc.data_path == full:
                    chans[fc.array_index] = fc
        except Exception:
            pass
    if any(j not in chans for j in range(n)):
        return None
    return [chans[j] for j in range(n)]


_BIP_ALIGN_CACHE = {}


def bip_rotation_refs(rig, pb, frame):
    """插帧时用来挑「连续等价表示」的参考通道值。

    v2.3：原来只取 `fc.evaluate(frame-1)`。真实工程里当前帧往往本来就有
    一个旧关键帧（Bezier 手柄会把 frame-1 的值往旧值那边拉），参考就被带偏。
    现在优先取**紧邻的前一个真实关键帧**，再补前一行曲线值与后一个关键帧；
    当前帧上的旧值一律不参与（它正是要被覆盖的脏数据）。

    结果按 (骨架, 动作, 骨名, 帧) + 曲线关键点数量缓存，拖拽期间自动插帧
    是每帧调用的，缓存后一次拖动只扫一遍曲线。
    """
    chans = bip_rot_channels(rig, pb)
    if not chans:
        return []
    act = rig.animation_data.action if rig.animation_data else None
    # v2.4：轴序变化后曲线数值被 5.3 转换功能整组重写，缓存必须跟着失效。
    key = (id(rig), id(act), pb.name, int(frame), pb.rotation_mode)
    sig = tuple(len(fc.keyframe_points) for fc in chans)
    hit = _BIP_ALIGN_CACHE.get(key)
    if hit is not None and hit[0] == sig:
        return hit[1]
    try:
        frames = sorted({round(k.co.x, 3) for fc in chans
                         for k in fc.keyframe_points})
        earlier = [f for f in frames if f < frame - 0.01]
        later = [f for f in frames if f > frame + 0.01]
        refs = []
        if earlier:
            refs.append(([fc.evaluate(earlier[-1]) for fc in chans], 1.0))
        refs.append(([fc.evaluate(frame - 1) for fc in chans], 0.35))
        if later:
            refs.append(([fc.evaluate(later[0]) for fc in chans], 0.5))
    except Exception:
        refs = []
    if len(_BIP_ALIGN_CACHE) > 256:
        _BIP_ALIGN_CACHE.clear()
    _BIP_ALIGN_CACHE[key] = (sig, refs)
    return refs


def bip_align_rotation(rig, pb, frame, fallback=None, hinge=None):
    """把当前旋转通道换成与相邻关键帧连续的等价表示（姿态一字不改）。

    四元数 q/-q、欧拉的多组解、轴角 ±2pi 都是同一姿态，但 F-Curve 是
    **逐分量**插值的。驭骨自由旋转写 matrix_basis 时由 Blender 重新分解，
    落回 [-pi,pi] 的规范分支；真实 ARP 工程里这些骨的曲线常常在
    -120°/-210°/-150° 这种「远分支」上，新关键帧一插进去，中间帧就要
    绕 200°+ 的远路（实测单帧 123.5°，正常 2.8°）—— 就是用户看到的
    「中间的差值暴跳」。BIP 拖手走的是同一套写值，故两处共用本函数。
    """
    refs = bip_rotation_refs(rig, pb, frame)
    if not refs and fallback is not None:
        refs = [(fallback, 1.0)]
    if not refs:
        return False
    try:
        path = bip_rotation_path(pb)
        cur = [x for x in getattr(pb, path)]
        best = bip_best_repr(pb, bip_basis_quat(pb), refs)
        if len(best) == len(cur) and max(
                abs(a - b) for a, b in zip(best, cur)) < 1e-9:
            return False
        bip_set_rot_values(pb, best)
        # v2.0：前臂换分支后仍要守住被锁的通道。若投影会改变姿态（欧拉的
        # 另一分支恰好动了锁定分量），宁可保留连续表示、不强行投影。
        if hinge:
            before = bip_basis_quat(pb)
            bip_project_hinge(pb, hinge)
            if bip_basis_quat(pb).rotation_difference(before).angle > 1e-4:
                bip_set_rot_values(pb, best)
        return True
    except Exception:
        return False


def bip_align_keyframe(rig, pb, frame, fallback=None, hinge=None):
    """兼容旧入口：BIP 收尾插帧前的连续表示对齐。"""
    return bip_align_rotation(rig, pb, frame, fallback, hinge)


# ------------------------------------------------------- 已有关键帧分支修复
_BIP_FIX_LOG = '未运行'


def _bip_vals_quat(pb, vals):
    try:
        if pb.rotation_mode == 'QUATERNION':
            return _BipQuaternion(vals).normalized()
        if pb.rotation_mode == 'AXIS_ANGLE':
            return _BipQuaternion(_BipVector(vals[1:4]), vals[0]).normalized()
        return _BipEuler(vals, bip_euler_order(pb)).to_quaternion().normalized()
    except Exception:
        return None


def _bip_action_fcurves(act):
    fcs = []
    try:
        bags = [bag for layer in act.layers
                for strip in layer.strips
                for bag in strip.channelbags]
    except Exception:
        bags = []
    if bags:
        for bag in bags:
            try:
                fcs.extend(list(bag.fcurves))
            except Exception:
                pass
    else:
        try:
            fcs = list(act.fcurves)
        except Exception:
            pass
    return fcs


def _bip_apply_key_transform(chans, frame, old, new):
    """把一组旋转 F-Curve 在某个帧上的关键点换成等价表示，手柄一起变换。

    等价表示的每个分量只有两种映射：
      - 平移 new = old + 2k*pi（欧拉任意分量加整圈、翻转解的 X/Z）→ 手柄同量平移；
      - 反射 new = c - old，c 是 pi 的整数倍（欧拉翻转解的中间轴、
        四元数 q/-q、轴角反轴）→ 手柄按同一 c 反射。
    其余情况退化为平移，关键帧姿态始终精确不变。
    """
    two_pi = 2 * _bip_math.pi
    for j, fc in enumerate(chans):
        o, n = old[j], new[j]
        d = n - o
        if abs(d) < 1e-9:
            continue
        c = n + o
        reflect = False
        if abs(d / two_pi - round(d / two_pi)) > 1e-4:
            reflect = abs(c / _bip_math.pi - round(c / _bip_math.pi)) < 1e-4
        for k in fc.keyframe_points:
            if abs(k.co.x - frame) > 1e-3:
                continue
            if reflect:
                k.handle_left.y = c - k.handle_left.y
                k.handle_right.y = c - k.handle_right.y
            else:
                k.handle_left.y += d
                k.handle_right.y += d
            k.co.y = n


def bip_arm_rot_bone_names(rig):
    """修复工具默认只动 FK 手臂链，别把整条脊柱/腿的曲线也改一遍。"""
    names = set()
    try:
        for p in rig.pose.bones:
            if bip_is_fk(p.name) and bip_role(p.name) in ('hand', 'forearm', 'upper'):
                names.add(p.name)
    except Exception:
        pass
    return names


def bip_normalize_rotation_curves(rig, bone_names=None, dry_run=False,
                                  min_delta=0.05):
    """修以前被自由旋转写坏的旧关键帧（数值落在规范分支、邻居却在远分支）。

    只改数值表示与手柄，**每个关键帧的姿态完全不变**；覆盖欧拉多解、
    四元数 q/-q、轴角 ±2pi。判据是「换成等价表示后，相邻两帧的数值插值
    路径是否缩短 > 0.9 rad」，真实大幅挥臂动作不会被误当成分支跳变。
    只处理骨骼当前生效的旋转通道，动作里换模式残留的旧曲线一律跳过。

    v2.6：帧集合从「各通道共有」改为**并集**。ARP 铰链前臂被锁的 X/Y
    恒为 0，自动插帧不会给它们补键，结果 f40 这种只有 W/Z 有键的帧以前
    被整帧忽略（实测「检查/修手臂」都报 0，符号翻转永远留在曲线里）。
    并集帧里对缺通道的帧额外做姿态等价校验：只改有键通道后，若整组数值
    已不再表示同一姿态，就跳过该帧，绝不写坏曲线。

    返回统计字典；dry_run=True 只统计不写。
    """
    out = {'groups': 0, 'keys': 0, 'fixed': 0, 'pose_err': 0.0,
           'bad': 0, 'partial': 0}
    act = getattr(getattr(rig, 'animation_data', None), 'action', None)
    if act is None:
        return out
    groups = {}
    for fc in _bip_action_fcurves(act):
        dp = getattr(fc, 'data_path', '') or ''
        if not dp.startswith('pose.bones["'):
            continue
        try:
            bone = dp.split('"', 2)[1]
            tail = dp.rsplit('.', 1)[-1]
        except Exception:
            continue
        if tail not in ('rotation_euler', 'rotation_quaternion',
                        'rotation_axis_angle'):
            continue
        if bone_names and bone not in bone_names:
            continue
        groups.setdefault((bone, tail), {})[fc.array_index] = fc
    for (bone, tail), idx in groups.items():
        p = rig.pose.bones.get(bone)
        if p is None:
            continue
        # 动作里常残留「换旋转模式之前」的旧曲线（例如骨骼现在是
        # QUATERNION，动作里却还留着 rotation_euler 三条 F-Curve）。
        # 那些曲线当前根本不参与求值，按现在的模式去解释会写出乱码。
        if tail != bip_rotation_path(p):
            continue
        n = 3 if tail == 'rotation_euler' else 4
        if any(j not in idx for j in range(n)):
            continue
        chans = [idx[j] for j in range(n)]
        # v2.6：取并集帧。ARP 铰链骨的 X/Y 恒 0、自动插帧不补键，
        # 用交集会把「只有 W/Z 有键」的脏帧（真实工程 f40）整帧漏掉。
        # 缺通道的帧在下面逐帧做姿态等价校验，安全才改。
        frames = sorted({round(k.co.x, 4) for fc in chans
                         for k in fc.keyframe_points})
        if len(frames) < 2:
            continue
        out['groups'] += 1
        out['keys'] += len(frames)
        # 链式锚点（Blender Euler Discontinuity Filter 的做法）：首帧保持
        # 原样，之后每帧都参考「上一帧最终采用的表示」，而不是原值快照里的
        # 前一帧。否则一处脏帧修好后，它后面本来干净的关键帧会拿这个脏
        # 原值当参考，被反向翻到另一分支，把绕远路挪到后半段。
        # 判据不是「离首帧最近」，而是「换成等价表示后，这一段插值是否
        # 明显变短」—— 真实的大幅挥臂动作不该被 2pi 平移改写掉。
        snap = [(f, [fc.evaluate(f) for fc in chans]) for f in frames]
        prev = snap[0][1]
        for f, old in snap[1:]:
            q = _bip_vals_quat(p, old)
            if q is None:
                prev = old
                continue
            best = bip_best_repr(p, q, [(prev, 1.0)])
            if len(best) != len(old):
                out['bad'] += 1
                prev = old
                continue
            q2 = _bip_vals_quat(p, best)
            dot = abs(sum(a * b for a, b in zip(q, q2))) if q2 is not None else 0.0
            if dot < 0.99999:
                out['bad'] += 1
                prev = old
                continue
            delta = max(abs(a - b) for a, b in zip(best, old))
            if delta < max(min_delta, 1e-9):
                prev = old
                continue
            d_old = sum((a - b) ** 2 for a, b in zip(old, prev)) ** 0.5
            d_new = sum((a - b) ** 2 for a, b in zip(best, prev)) ** 0.5
            # 只有当新表示把这一段数值路径显著缩短，才认定原值落错分支。
            # 0.9 rad ≈ 52°：容得下真实的大幅动作，又拦得住 2pi / q->-q。
            if d_old - d_new < 0.9:
                prev = old
                continue
            # v2.6：并集帧里可能只有部分通道打了键。只改有键通道后，
            # 整组数值必须仍表示同一姿态；否则宁可跳过这一帧。
            keyed = [any(abs(k.co.x - f) < 1e-3
                         for k in fc.keyframe_points) for fc in chans]
            if not all(keyed):
                out['partial'] += 1
                partial = [best[j] if keyed[j] else old[j]
                           for j in range(n)]
                qp = _bip_vals_quat(p, partial)
                if qp is None or abs(sum(a * b for a, b in
                                         zip(q, qp))) < 0.99999:
                    prev = old
                    continue
            out['fixed'] += 1
            if not dry_run:
                _bip_apply_key_transform(chans, f, old, best)
            prev = best
    if not dry_run:
        # 改了 co.y / 手柄后必须刷新曲线缓存，否则求值仍用旧插值。
        for (_bone, _tail), _idx in groups.items():
            for _fc in _idx.values():
                try:
                    _fc.update()
                except Exception:
                    pass
        try:
            _BIP_ALIGN_CACHE.clear()
        except Exception:
            pass
    return out


_BIP_REPAIR_LOG = '未运行'
_BIP_REPAIR_LAST = 0.0
_BIP_REPAIR_THROTTLE = 0.30      # 驭骨连续旋转自动插帧时节流，避免每帧全扫


def bip_repair_rotation_curves(rig, names, tag=''):
    """自动理顺本次涉及的 FK 骨旋转曲线分支（关键帧姿态逐帧不变）。

    v2.6：画弧时几个关键帧挤在一起，只要其中还留着一个旧脏帧（四元数整组
    翻符号 / 欧拉落错分支），逐分量插值就会在中间绕远路，运动路径当场打结。
    写值前的 bip_align_rotation 只能保证「新键」连续，管不到历史关键帧。

    两个调用点都不能少：
      · BIP 起点：当前帧若正好落在脏区间里，插值姿态本来就是错的，先理顺
        再从干净姿态起拖；否则第一下拖动会把「错姿态」固化成新关键帧。
      · BIP 收尾 / 驭骨插帧：理顺拖动过程中新增或暴露的分支。
    修完后只做一次 view_layer.update()：实测它足以让当前帧按新曲线重算
    （probe_refresh P3），不需要 scene.frame_set()——后者会连带把用户
    未插帧的内存姿态冲掉，是实测踩过的坑。
    """
    global _BIP_REPAIR_LOG, _BIP_FIX_LOG
    st = {'fixed': 0, 'groups': 0, 'keys': 0}
    try:
        st = bip_normalize_rotation_curves(
            rig, bone_names=set(names), dry_run=False)
    except Exception as exc:
        _BIP_REPAIR_LOG = '自动修复失败: %s' % str(exc)[:50]
        return st
    if st['fixed']:
        _BIP_REPAIR_LOG = ('%s · 自动理顺 %d 帧分支（姿态不变）'
                           % (tag, st['fixed']))
        _BIP_FIX_LOG = _BIP_REPAIR_LOG
        try:
            _bip_bpy.context.view_layer.update()
        except Exception:
            pass
    else:
        _BIP_REPAIR_LOG = '%s · 曲线已连续' % tag
    return st


def bip_set_basis_quat(pb, q, refs=None):
    """只写旋转通道，不碰 location / scale / 约束；refs 给定时保持表示连续。

    v2.4：统一走 bip_set_rot_values 逐分量写入，避免切片赋值把 5.3 转换过
    的欧拉轴序打回 XYZ。
    """
    mode = pb.rotation_mode
    if refs:
        try:
            bip_set_rot_values(pb, bip_best_repr(pb, q, refs))
            return
        except Exception:
            pass
    if mode == 'QUATERNION':
        bip_set_rot_values(pb, list(q))
    elif mode == 'AXIS_ANGLE':
        axis, angle = q.to_axis_angle()
        bip_set_rot_values(pb, [angle, axis.x, axis.y, axis.z])
    else:
        bip_set_rot_values(pb, list(q.to_euler(bip_euler_order(pb))))


def bip_set_world_rot(pb, target_q, refs=None, passes=3):
    """只写旋转通道，把骨骼的世界朝向转到 target_q。

    绝不碰 location / scale / 约束 —— FK 控制骨常常带位置关键帧（实测真实
    工程里 c_arm_fk.r 的 location 是 (-1.212,-0.287,-0.404)），用
    pose_bone.matrix setter 写整张矩阵会连带把 location 通道改掉 3.5 cm，
    大臂/小臂控制器就会在拖动第一帧猛跳一下，还污染动画数据。
    """
    for _ in range(passes):
        cur = pb.matrix.to_quaternion().normalized()
        delta = cur.rotation_difference(target_q)   # 最短弧，w >= 0
        if delta.angle < 1e-5:
            return
        bip_set_basis_quat(pb, bip_basis_quat(pb) @ delta, refs)
        _bip_bpy.context.view_layer.update()


def bip_hinge_info(pb):
    """检测 FK 前臂控制骨被锁死的旋转通道，返回铰链投影参数或 None。

    ARP 的前臂 FK 是肘铰链：lock_rotation=(True,True,False)，只允许绕一个轴
    弯肘，且常带 LIMIT_ROTATION 约束；起点四元数只有 W/Z 非零。IK 链解出的
    却是完整 3-DOF 旋转，直接灌进去会把 X/Y 写成非零（实测 -0.09 ~ -0.49），
    通道值与约束互相打架，播放时手臂就乱转。

    只有**明确检测到锁定通道**才启用；全自由前臂、或三个轴全锁（起点恰好是
    单位四元数这种会被误判的情况）都不投影，保持 v1.9 的原行为。
    """
    path = bip_rotation_path(pb)
    try:
        base = [x for x in getattr(pb, path)]
    except Exception:
        return None
    lock = tuple(pb.lock_rotation)
    mode = pb.rotation_mode
    if mode in ('QUATERNION', 'AXIS_ANGLE'):
        # (w,x,y,z) / (angle,ax,ay,az)：后三项对应 lock_rotation 的 X/Y/Z
        locked = [j for j in (1, 2, 3) if lock[j - 1]]
        if len(locked) >= 3:
            return None
    else:
        locked = [j for j in (0, 1, 2) if lock[j]]
        if len(locked) >= 3:
            return None
    if not locked:
        return None
    return {'path': path, 'mode': mode, 'base': base, 'locked': locked}


def bip_project_hinge(pb, info):
    """把旋转通道投影回该骨允许的子群：被锁分量恢复为拖动起点值。

    只改旋转通道，不碰 location / scale / 约束。四元数置零后重新归一化，
    旋转角（w）保持不变、半球与起点一致，所以不会引入首帧跳变。
    """
    if not info:
        return False
    try:
        vals = [x for x in getattr(pb, info['path'])]
        for j in info['locked']:
            vals[j] = info['base'][j]
        if info['mode'] in ('QUATERNION', 'AXIS_ANGLE'):
            if info['mode'] == 'QUATERNION':
                q = _BipQuaternion(vals).normalized()
                if sum(q[j] * info['base'][j] for j in range(4)) < 0:
                    q.negate()
                vals = [q[j] for j in range(4)]
        bip_set_rot_values(pb, vals)
        return True
    except Exception:
        return False


def bip_lock_hand_rot(state, passes=3):
    """把 FK 手控骨的世界朝向钉在拖动开始时的值。

    手控骨是前臂 FK 的子骨：只写回上臂/前臂旋转时，它的局部旋转虽然没变，
    世界朝向仍会跟着前臂一起转（实测一次连续拖动可累计 37 deg），视觉上
    就是「手掌控制器自己转 / 突然跳变」。BIP 拖手不改腕朝向，所以每帧用
    父链旋转的反量补偿它的旋转通道；只动旋转，位置零影响、无新增约束。
    """
    if not state.get('lock_hand_rot'):
        return
    hand = state['rig'].pose.bones[state['map']['fk_hand']]
    ref = state.get('rot_ref', {}).get(state['map']['fk_hand'])
    bip_set_world_rot(hand, state['hand_rot'],
                      [(ref, 1.0)] if ref is not None else None,
                      passes=passes)


# ------------------------------------- v2.1 近伸直 / IK 失配的解析二连杆
# 触发阈值：IK 链解与 FK 初始姿态的最大夹角。实测 E 踏步回斩 frame 29 右臂
# 95.4 度 / 左臂 37.1 度（有僵硬跳变），D 踏步上挑 12.4 / 4.0 度、
# bip 手感测试 0 度（一直正常）。20 度把两类干净分开。
_BIP_ANALYTIC_MIN_GAP_DEG = 20.0
_BIP_AXES = (_BipVector((1.0, 0.0, 0.0)),
             _BipVector((0.0, 1.0, 0.0)),
             _BipVector((0.0, 0.0, 1.0)))


def bip_analytic_info(state):
    """判断本段拖动是否需要绕开 ARP 的 IK 链，改走解析二连杆求解。

    两个条件都要满足，避免动到已验证正常的场景：
      1) 前臂 FK 是明确的单轴铰链（QUATERNION / AXIS_ANGLE，恰好一个自由轴）；
      2) 目标已定位到手腕时，IK 链姿态与 FK 姿态相差 >= 20 度 —— 说明 ARP
         的 IK 链解的肘平面和 FK 铰链轴不是一回事，尤其在近伸直奇异区会
         靠 ik_stretch 吸收距离、几乎不弯肘，铰链投影后手就完全不动。
    """
    try:
        pb = state['rig'].pose.bones
        amap = state['map']
        hinge = state.get('hinge')
        if not hinge or hinge['mode'] not in ('QUATERNION', 'AXIS_ANGLE'):
            return None
        locked = set(hinge['locked'])
        free = [j for j in (1, 2, 3) if j not in locked]
        if len(free) != 1:
            return None
        upper, fore, hand = (pb[amap['fk'][0]], pb[amap['fk'][-1]],
                             pb[amap['fk_hand']])
        S, E, W = (upper.matrix.translation.copy(),
                   fore.matrix.translation.copy(),
                   hand.matrix.translation.copy())
        L1, L2, D0 = (E - S).length, (W - E).length, (W - S).length
        if min(L1, L2, D0) < 1e-6:
            return None
        gap = max(_bip_math.degrees(
            pb[fk].matrix.to_quaternion().normalized().rotation_difference(
                pb[ik].matrix.to_quaternion().normalized()).angle)
            for fk, ik in zip(amap['fk'], amap['ik']))
        if gap < _BIP_ANALYTIC_MIN_GAP_DEG:
            return None

        axis = _BIP_AXES[free[0] - 1]          # basis 局部铰链轴
        q_fore0 = bip_basis_quat(fore)
        # basis 右乘绕局部轴 R，等价于世界系绕 q_world @ axis 旋转同一角度。
        h_world = (fore.matrix.to_quaternion().normalized() @ axis).normalized()
        v_test = _BipQuaternion(h_world, 0.05) @ (W - E).normalized()
        sign = (1.0 if (L1 * (E - S).normalized() + L2 * v_test).length < D0
                else -1.0)                     # 镜像臂自动取反
        beta0 = _bip_math.acos(max(-1.0, min(1.0,
            (L1 * L1 + L2 * L2 - D0 * D0) / (2.0 * L1 * L2))))
        # v2.2：伸展上限是「二连杆的几何臂长」，不是本次拖动的起点距离。
        # v2.1 曾把 Dmax 设成 D0 以避开近奇异区，但用户先把手拉弯、松手、
        # 再按 Z 拖时新的 D0 就是弯的，等于永远禁止伸直（实测往外拉 12cm
        # 手位移 0.00000、肘角 134->85 度后再也回不去）。现在允许一路拉到
        # 完全伸直，超过臂长才钳位；首步弹跳改由闭环的步长/角度保护消除。
        return {
            'S': S, 'L1': L1, 'L2': L2, 'D0': D0,
            'Dmin': min(abs(L1 - L2) + 2e-4, D0),
            'Dmax': max(D0, L1 + L2 - 2e-4),
            'beta0': beta0, 'axis': axis, 'sign': sign,
            'q_fore0': q_fore0,
            'refs_f': [(hinge['base'], 1.0)],
            'refs_u': [(state['rot_ref'][amap['fk'][0]], 1.0)],
            'names': (amap['fk'][0], amap['fk'][-1], amap['fk_hand']),
            'gap': gap,
        }
    except Exception:
        return None


def bip_analytic_solve(state, goal_arm):
    """解析二连杆 IK：肘角由余弦定理求，上臂绕肩对准方向，距离闭环收误差。

    只写 FK 旋转通道，不碰 location / scale / 约束 / ik_fk_switch；前臂只沿
    它本来被允许的那一个铰链轴弯，锁死通道恒为起点值；腕朝向沿用拖动起点。
    IK 目标照样移动以保持 rig 其它通道一致，但**不读 IK 链的解**。
    """
    a = state['analytic']
    pb = state['rig'].pose.bones
    upper, fore, hand = (pb[n] for n in a['names'])
    # 解析模式完全不读 IK 链的解，因此不逐帧移动 IK 目标：省一次全骨架依赖
    # 图求值，也避免 FK 模式下 IK 目标乱动影响别的通道。bip_begin 已把它
    # 定位到手腕并 mute 约束，bip_end 会完整还原。
    S, L1, L2 = a['S'], a['L1'], a['L2']
    dv = _BipVector(goal_arm) - S
    D = dv.length
    if D < 1e-8:
        return hand.matrix.translation.copy()
    Dc = max(a['Dmin'], min(a['Dmax'], D))
    Gc = S + dv.normalized() * Dc

    def beta(d):
        return _bip_math.acos(max(-1.0, min(1.0,
            (L1 * L1 + L2 * L2 - d * d) / (2.0 * L1 * L2))))

    theta = a['beta0'] - beta(Dc)
    sign = a['sign']
    # theta 的物理范围：beta(肘内角) ∈ [0, pi]，theta = beta0 - beta。
    # 往外伸直 theta -> beta0-pi（负），往内弯 theta -> beta0（正）。
    # 接近完全伸直时 sin(beta)->0、牛顿导数趋零，步长会炸成几百度 —— 这才是
    # v2.1 之前 outward 首步猛跳的真因；用「单轮步长上限 + 物理区间钳位」
    # 化解，既能一路拉直，又不会在奇异区发散。
    lo, hi = a['beta0'] - _bip_math.pi, a['beta0']
    # 牛顿闭环：解析初值通常已很准，2~4 轮把 LIMIT_ROTATION 约束、控制骨
    # 偏移、父链非刚体等带来的毫米级偏差收干净。
    for _ in range(4):
        bip_set_basis_quat(
            fore, a['q_fore0'] @ _BipQuaternion(a['axis'], sign * theta),
            a['refs_f'])
        _bip_bpy.context.view_layer.update()
        D_now = (hand.matrix.translation - S).length
        err = Dc - D_now
        if abs(err) < 2e-5:
            break
        # deriv = dD/d(theta)。实际写入的骨骼转角是 sign*theta，镜像臂
        # (sign=-1) 的 dD/d(alpha) 会反号，但 d(alpha)/d(theta)=sign，两者
        # 相乘后 sign^2=1 —— dD/d(theta) 与 sign 无关。v2.1 多乘了一次
        # sign，往内拖时解析初值够准、闭环没触发所以没暴露；v2.2 往外
        # 拉到极限时 err 变大，左臂就被往回推、卡在 161.9 度。
        deriv = (-L1 * L2 * _bip_math.sin(beta(D_now))
                 / max(D_now, 1e-6))
        if abs(deriv) < 2e-5:
            break
        step = max(-0.12, min(0.12, err / deriv))   # 单轮最多约 6.9 度
        new_theta = max(lo, min(hi, theta + step))
        if abs(new_theta - theta) < 1e-9:
            break                                  # 已抵几何/约束边界
        theta = new_theta
    # 绕肩把整条链的方向对准目标（长度已由肘角决定），1~2 轮即收敛。
    for _ in range(2):
        cur = hand.matrix.translation - S
        delta = cur.rotation_difference(Gc - S)
        if delta.angle < 1e-7:
            break
        bip_set_world_rot(
            upper, delta @ upper.matrix.to_quaternion().normalized(),
            a['refs_u'], passes=2)
    bip_lock_hand_rot(state, passes=3)
    return hand.matrix.translation.copy()


def bip_begin(rig, amap, prepare=False):
    """prepare=True 由真实操作符 invoke 传入：开拖前先理顺旧脏帧。

    只换数值表示，关键帧姿态不变。当前帧若正好落在旧脏帧的插值区间里，
    不理顺就会把「被绕远路转错的姿态」固化成新关键帧（实测第一下拖在
    f44 时，f42 会留下 0.5 m 的位移突变）。仅在开启 Auto Key 时做：
    没插帧的手动姿态不能被重算冲掉，而 Auto Key 关闭时本次拖动本来也不会
    新增关键帧、不影响已有插值。直接调 bip_begin 的求解器回归默认关闭，
    保持 v1.9~v2.5 已验证的求解行为一字不变。
    """
    pb = rig.pose.bones
    need = (amap['fk_hand'], amap['fk_fore'], amap['fk_upper'],
            amap['ik_target'], amap['ik_fore'], amap['ik_upper'])
    if any(n not in pb for n in need):
        raise RuntimeError('解析到的骨骼不存在: %s' % (need,))
    if prepare:
        try:
            auto_key = _bip_bpy.context.scene.tool_settings.use_keyframe_insert_auto
        except Exception:
            auto_key = False
        if auto_key:
            bip_repair_rotation_curves(
                rig, list(amap['fk']) + [amap['fk_hand']], 'BIP 起点')
    tgt = pb[amap['ik_target']]
    state = {
        'rig': rig,
        'map': amap,
        'target_basis': tgt.matrix_basis.copy(),
        'target_matrix': tgt.matrix.copy(),
        'constraints': [(c, c.mute) for c in tgt.constraints],
        'fk_basis': [pb[n].matrix_basis.copy() for n in amap['fk']],
        'grab_arm': pb[amap['fk_hand']].matrix.translation.copy(),
        'hand_basis': pb[amap['fk_hand']].matrix_basis.copy(),
        'hand_rot': pb[amap['fk_hand']].matrix.to_quaternion().normalized(),
        'lock_hand_rot': (not pb[amap['fk_hand']].constraints
                          and not all(pb[amap['fk_hand']].lock_rotation)),
        # v1.9：拖动起点的旋转「数值表示」，整段拖动都跟着它保持同一分支，
        # 否则欧拉/四元数的等价解会让插帧绕远路。
        'rot_ref': {
            n: [x for x in getattr(pb[n], bip_rotation_path(pb[n]))]
            for n in set(list(amap['fk']) + [amap['fk_hand']])},
        # v2.0：前臂肘铰链只允许一个轴，锁定通道的信息在拖动起点采样一次
        'hinge': bip_hinge_info(pb[amap['fk'][-1]]),
    }
    for c, _ in state['constraints']:
        c.mute = True
    # pose_bone.matrix setter 会处理 rest -> basis；直接写 basis 会差一个 rest head。
    m = state['target_matrix'].copy()
    m.translation = state['grab_arm']
    tgt.matrix = m
    _bip_bpy.context.view_layer.update()
    # v1.8：记下「IK 目标已定位到手位置」这一刻的基线。之后每帧只把 IK 链
    # 相对基线的变化量叠加到 FK 初始姿态；若直接写 IK 链的绝对解，第一次
    # 移动就会把两者的初始差（实测 1.3°~18.9°，视姿态而定）整笔灌进 FK，
    # 大臂/小臂控制器就会猛跳一下。
    state['fk_init'] = [pb[n].matrix.copy() for n in amap['fk']]
    state['ik_delta_init'] = [
        (pb[n].matrix @ pb[n].bone.matrix_local.inverted()).copy()
        for n in amap['ik']]
    # v2.1：目标已定位、姿态未改，正好在这里判一次 IK 链是否可靠。
    state['analytic'] = bip_analytic_info(state)
    a = state['analytic']
    globals()['_BIP_MODE_LOG'] = (
        '解析二连杆 · IK/FK 差 %.1f° · 可伸直 %.1fcm'
        % (a['gap'], max(0.0, a['Dmax'] - a['D0']) * 100.0)
        if a else 'IK 链 · 标准模式')
    return state


def bip_solve(state, goal_arm):
    if state.get('analytic'):
        return bip_analytic_solve(state, goal_arm)
    rig = state['rig']
    pb = rig.pose.bones
    amap = state['map']
    tgt = pb[amap['ik_target']]
    m = state['target_matrix'].copy()
    m.translation = _BipVector(goal_arm)
    tgt.matrix = m
    _bip_bpy.context.view_layer.update()
    init = state.get('ik_delta_init')
    fk_init = state.get('fk_init')
    for i, (fk_name, ik_name) in enumerate(zip(amap['fk'], amap['ik'])):
        fk, ik = pb[fk_name], pb[ik_name]
        d_now = ik.matrix @ ik.bone.matrix_local.inverted()
        if (init is not None and fk_init is not None
                and i < len(init) and i < len(fk_init)):
            # 只取「IK 链相对拖动起点的变化」叠到 FK 初始姿态上
            target_m = (d_now @ init[i].inverted()) @ fk_init[i]
        else:
            target_m = d_now @ fk.bone.matrix_local   # 老 state 的兼容兜底
        # 只写旋转通道：FK 控制骨的 location 常带动画关键帧，绝不能被覆盖
        ref = state.get('rot_ref', {}).get(fk_name)
        bip_set_world_rot(fk, target_m.to_quaternion().normalized(),
                          [(ref, 1.0)] if ref is not None else None)
    # v2.0：前臂只保留它本来被允许的旋转自由度（被锁的 X/Y 保持起点值）。
    # 丢掉的那部分自由度交给 3-DOF 自由的上臂：绕肩把手方向重新对准目标，
    # 两轮即收敛（实测 2 / 4 / 6 轮结果完全一致），手位置比不投影还准。
    hinge = state.get('hinge')
    if hinge:
        fore = pb[amap['fk'][-1]]
        if bip_project_hinge(fore, hinge):
            _bip_bpy.context.view_layer.update()
            upper = pb[amap['fk'][0]]
            shoulder = upper.matrix.translation.copy()
            hand = pb[amap['fk_hand']]
            goal = _BipVector(goal_arm)
            ref_u = state.get('rot_ref', {}).get(amap['fk'][0])
            refs_u = [(ref_u, 1.0)] if ref_u is not None else None
            for _ in range(1):
                d_now = hand.matrix.translation - shoulder
                d_goal = goal - shoulder
                if d_now.length < 1e-8 or d_goal.length < 1e-8:
                    break
                delta = d_now.rotation_difference(d_goal)
                if delta.angle < 1e-6:
                    break
                bip_set_world_rot(
                    upper,
                    delta @ upper.matrix.to_quaternion().normalized(),
                    refs_u, passes=2)
    bip_lock_hand_rot(state)
    return pb[amap['fk_hand']].matrix.translation.copy()


def bip_rotation_path(pb):
    if pb.rotation_mode == 'QUATERNION':
        return 'rotation_quaternion'
    if pb.rotation_mode == 'AXIS_ANGLE':
        return 'rotation_axis_angle'
    return 'rotation_euler'


# ------------------------------------------------- 移动结束后刷新运动路径
def _bip_cancel_pending(rig):
    """撤掉驭骨异步队列里针对本骨架的待刷项，避免同步刷完又异步刷一次。"""
    g = globals()
    q = g.get('_MPATH_Q')
    if not isinstance(q, dict):
        return
    try:
        pending = q.get('rig') or ''
        if pending and pending != rig.name:
            return                     # 别的骨架在排队，别动它
        q['rig'] = ''
        q['defer'] = 0
        q['timer'] = False
        tick = g.get('_mpath_timer_tick')
        if callable(tick) and _bip_bpy.app.timers.is_registered(tick):
            _bip_bpy.app.timers.unregister(tick)
    except Exception:
        pass


def _bip_mpath_override(rig):
    """优先用驭骨自己的 _mpath_ctx_kw（带视口 window/area/region）。"""
    fn = globals().get('_mpath_ctx_kw')
    if callable(fn):
        try:
            kw = fn(rig)
            if isinstance(kw, dict) and kw:
                return kw
        except Exception:
            pass
    kw = {'object': rig, 'active_object': rig}
    try:
        ctx = _bip_bpy.context
        win = ctx.window
        if win is not None:
            kw['window'] = win
            area = getattr(ctx, 'area', None)
            if area is not None and area.type == 'VIEW_3D':
                kw['area'] = area
                reg = getattr(ctx, 'region', None)
                if reg is not None and reg.type == 'WINDOW':
                    kw['region'] = reg
    except Exception:
        pass
    return kw


def _bip_trim_new_paths(rig, before, kw):
    """只重算已有路径：万一某版本 paths_update 长出新的，立刻清掉并还原选择。"""
    try:
        new = [p for p in rig.pose.bones
               if p.motion_path and p.name not in before]
        if not new:
            return
        saved = [(p, p.select) for p in rig.pose.bones if p.select]
        try:
            active = rig.data.bones.active
        except Exception:
            active = None
        names = {p.name for p in new}
        try:
            for p in rig.pose.bones:
                p.select = p.name in names
            rig.data.bones.active = new[0].bone
            with _bip_bpy.context.temp_override(**kw):
                _bip_bpy.ops.pose.paths_clear(only_selected=True)
        finally:
            for p, s in saved:
                p.select = s
            try:
                rig.data.bones.active = active
            except Exception:
                pass
    except Exception:
        pass
    finally:
        _bip_wash()


def _bip_wash():
    """冲刷 bpy 遗留的 C 错误（驭骨 _error_wash），独立运行时静默跳过。"""
    fn = globals().get('_error_wash')
    if callable(fn):
        try:
            fn()
        except Exception:
            pass


def bip_refresh_motion_paths(rig):
    """BIP 有效拖动收尾后「立刻」同步重算已有运动路径（v1.7 零延后）。

    为什么必须显式刷新：运动路径缓存只在原生 transform 结束时失效；BIP 是
    直写 FK 旋转通道 + 自己 keyframe_insert，曲线变了缓存不动（驭骨 v5.2.30
    实测 update_tag 全部无效，只有 pose.paths_update() 能刷）。

    v1.6 走驭骨 _mpath_queue 异步队列：50 ms 起跳 + 等手停 0.3 s + 0.2 s 一轮
    防抖，松手后常常还要 0.3~0.5 s 路径才动。现在改成在收尾点直接同步调用
    pose.paths_update()：本次事件内就算完，松手瞬间路径已经是新姿态；代价是
    这次收尾会多约 0.15~0.25 s 的同步重算（发生在松手之后，不影响拖动跟手）。

    仍沿用驭骨原生机制：偏好「确认后刷新运动路径」开关、_mpath_ctx_kw 的
    override、只重算本来就有路径的骨、清掉意外新增的路径、错误冲刷与视口重绘。
    同步失败时才退回异步队列兜底，保证功能不丢。v2.5 起驭骨自己的
    MOVE/ROT 提交点也通过运行时包装走这条同步路径（原来走异步队列，
    松手后要等 0.3~0.5 s），拖动中另有 bip_live_motion_paths 实时跟手。
    """
    global _BIP_MPATH_LOG
    if rig is None or getattr(rig, 'type', '') != 'ARMATURE':
        _BIP_MPATH_LOG = '跳过: 无骨架'
        return False
    try:
        _pref = globals().get('pref')
        if callable(_pref) and not _pref('mpath_refresh', True):
            _BIP_MPATH_LOG = '关闭: 驭骨偏好「确认后刷新运动路径」'
            return False
    except Exception:
        pass                       # 独立运行 / 旧版驭骨：按默认开启处理
    try:
        before = {p.name for p in rig.pose.bones if p.motion_path}
    except Exception:
        _BIP_MPATH_LOG = '跳过: 无法读取运动路径'
        return False
    if not before:
        _BIP_MPATH_LOG = '跳过: 骨架没有已计算的运动路径'
        return False

    _bip_cancel_pending(rig)
    kw = _bip_mpath_override(rig)
    t0 = _bip_time.perf_counter()
    try:
        with _bip_bpy.context.temp_override(**kw):
            _bip_bpy.ops.pose.paths_update()
    except Exception as exc:
        _bip_wash()
        # 同步路径在个别上下文里不可用时，退回驭骨异步队列，别把功能整个丢掉
        queue = globals().get('_mpath_queue')
        if callable(queue):
            try:
                if queue(rig):
                    _BIP_MPATH_LOG = '同步失败已转异步: %s' % str(exc)[:30]
                    return True
            except Exception:
                pass
        _BIP_MPATH_LOG = '失败: %s' % str(exc)[:50]
        return False
    finally:
        _bip_wash()

    _bip_trim_new_paths(rig, before, kw)
    # 全量重算已经覆盖所有预览点，临时记录可以丢掉。
    try:
        _BIP_LIVE_DIRTY.clear()
    except Exception:
        pass
    redraw = globals().get('_mpath_redraw')
    if callable(redraw):
        try:
            redraw()
        except Exception:
            pass
    _BIP_MPATH_LOG = '已立刻同步重算: %d 根 / %.0f ms' % (
        len(before), (_bip_time.perf_counter() - t0) * 1000.0)
    return True


# ------------------------------------- v2.5：运动路径拖动中实时跟手
# 原生 transform 结束时是同步重算（松手即见效），而驭骨 v5.2.37/39 把重算
# 放进 timer：50 ms 起跳 + 等手停 0.30 s + 0.2 s 一轮，再加 paths_update
# 本身实测 190 ms —— 松手后常要 0.5~0.7 s 路径才动，就是视频里的「好一会儿」。
# 全量重算不能塞进每个鼠标事件（v5.2.39 已踩过 160 ms 冻结 + 姿态猛跳），
# 但运动路径「当前帧」那一个点是**可写**的缓存：实测 points[i].co 赋值后
# view_layer.update() 不会覆盖它，只有 paths_update() 才会重算。于是分两层：
#   拖动中：只把当前帧点写成骨骼实时头部的世界坐标，O(路径数)；唯一的
#           depsgraph 刷新 8.5 ms，节流到 30 Hz —— 路径当场跟手、不冻结事件循环；
#   松手时：同步全量 paths_update()（v1.7 已有），整条曲线回到与动画一致。
# 索引 = frame_current - frame_start，坐标空间 = 世界空间，均由真实工程实测。
_BIP_LIVE_INTERVAL = 0.033      # 30 Hz，足够跟手，又不会把事件循环吃满
_BIP_LIVE_LAST = 0.0
_BIP_LIVE_LOG = '未使用'
# 临时写过的点要能原样还回去：取消拖动 / 拖动中换帧 / 卸载包装时，
# 不能把「预览姿态」的路径点留在缓存里。key 里带帧索引，值是首次写入前的原值。
_BIP_LIVE_DIRTY = {}
_BIP_LIVE_FRAME = None


def bip_restore_live_motion_paths():
    """撤销拖动期间的所有临时路径点，返回还原点数（O(脏点数)，零求值）。"""
    global _BIP_LIVE_DIRTY, _BIP_LIVE_FRAME
    if not _BIP_LIVE_DIRTY:
        _BIP_LIVE_FRAME = None
        return 0
    n = 0
    for key, old in list(_BIP_LIVE_DIRTY.items()):
        rname, bname, i = key
        try:
            r = _bip_bpy.data.objects.get(rname)
            p = r.pose.bones.get(bname) if r is not None else None
            mp = p.motion_path if p is not None else None
            if mp is not None and 0 <= i < len(mp.points):
                mp.points[i].co = old
                n += 1
        except Exception:
            continue
    _BIP_LIVE_DIRTY = {}
    _BIP_LIVE_FRAME = None
    if n:
        try:
            redraw = globals().get('_mpath_redraw')
            if callable(redraw):
                redraw()
        except Exception:
            pass
    return n


def bip_live_motion_paths(rig=None, force=False):
    """拖动中把「当前帧」的路径点实时写到骨骼头部，返回改动点数。

    只改绘制缓存里的一个点，不碰任何动画数据；松手时的全量重算会覆盖成
    最终形态。没有已计算路径 / 偏好关闭 / 当前帧不在路径范围内都是零开销。
    """
    global _BIP_LIVE_LAST, _BIP_LIVE_LOG, _BIP_LIVE_FRAME
    now = _bip_time.perf_counter()
    if not force and now - _BIP_LIVE_LAST < _BIP_LIVE_INTERVAL:
        return 0
    _BIP_LIVE_LAST = now
    try:
        _pref = globals().get('pref')
        if callable(_pref) and not _pref('mpath_refresh', True):
            return 0
    except Exception:
        pass
    # 与驭骨原 _update_motion_paths 的守卫一致：没开自动关键帧的纯摆姿
    # 不产生动画数据，路径没有跟随意义，保持零开销。
    try:
        if not _bip_bpy.context.scene.tool_settings.use_keyframe_insert_auto:
            return 0
    except Exception:
        return 0
    try:
        frame = int(_bip_bpy.context.scene.frame_current)
    except Exception:
        return 0
    if _BIP_LIVE_FRAME is not None and _BIP_LIVE_FRAME != frame:
        bip_restore_live_motion_paths()   # 换了帧，旧帧的预览点先还回去
    _BIP_LIVE_FRAME = frame
    # 当前操作的骨架优先；其余带路径的骨架一起刷（工程里红/绿两条路径
    # 常来自不同骨架），遍历本身相对全量重算可以忽略。
    rigs = []
    if rig is not None:
        rigs.append(rig)
    try:
        for ob in _bip_bpy.data.objects:
            if ob is not rig and getattr(ob, 'type', '') == 'ARMATURE':
                rigs.append(ob)
    except Exception:
        pass
    targets = []
    for r in rigs:
        try:
            for pb in r.pose.bones:
                mp = pb.motion_path
                if mp is None:
                    continue
                i = frame - int(mp.frame_start)
                if 0 <= i < len(mp.points):
                    targets.append((r, pb, mp, i))
        except Exception:
            continue
    if not targets:
        return 0
    try:
        _bip_bpy.context.view_layer.update()
    except Exception:
        pass
    n = 0
    for r, pb, mp, i in targets:
        try:
            head = pb.head if getattr(mp, 'use_bone_head', True) else pb.tail
            co = r.matrix_world @ head
            old = mp.points[i].co
            if sum((a - b) ** 2 for a, b in zip(old, co)) > 1e-12:
                key = (r.name, pb.name, i)
                if key not in _BIP_LIVE_DIRTY:
                    _BIP_LIVE_DIRTY[key] = tuple(old)
                mp.points[i].co = co
                n += 1
        except Exception:
            continue
    if n:
        try:
            redraw = globals().get('_mpath_redraw')
            if callable(redraw):
                redraw()
            else:
                for a in _bip_bpy.context.screen.areas:
                    if a.type == 'VIEW_3D':
                        a.tag_redraw()
        except Exception:
            pass
    _BIP_LIVE_LOG = '实时 %d 点 / %d 路径' % (n, len(targets))
    return n


def bip_end(state, commit=True, auto_key=True):
    rig = state['rig']
    pb = rig.pose.bones
    amap = state['map']
    tgt = pb[amap['ik_target']]
    if not commit:
        for name, basis in zip(amap['fk'], state['fk_basis']):
            pb[name].matrix_basis = basis
        if state.get('lock_hand_rot'):
            pb[amap['fk_hand']].matrix_basis = state['hand_basis']
    tgt.matrix_basis = state['target_basis']
    for c, mute in state['constraints']:
        c.mute = mute
    _bip_bpy.context.view_layer.update()
    if commit and auto_key and _bip_bpy.context.scene.tool_settings.use_keyframe_insert_auto:
        frame = _bip_bpy.context.scene.frame_current
        names = list(amap['fk'])
        if state.get('lock_hand_rot'):
            names.append(amap['fk_hand'])
        for name in names:
            p = pb[name]
            # v1.9：先把新关键帧的数值表示对齐到相邻关键帧（姿态不变），
            # 否则终点一样、中间却会绕远路整段乱转。
            bip_align_keyframe(rig, p, frame,
                               state.get('rot_ref', {}).get(name),
                               state.get('hinge')
                               if name == amap['fk'][-1] else None)
            p.keyframe_insert(data_path=bip_rotation_path(p), frame=frame, group=name)
        # v2.6：新键插完后，把这条手臂曲线里残留的旧脏帧一并理顺。
        # 只换数值表示（q/-q、欧拉其它分支），关键帧姿态逐帧零改变；
        # 放在路径重算之前，松手看到的运动路径已经是修好的曲线。
        bip_repair_rotation_curves(rig, names, 'BIP 收尾')
    # 有效移动收尾（确认 / 切骨交还 / 换手重锚）：当场同步刷新，松手即见效。
    # 取消时姿态与关键帧完全还原、路径本来就没变，跳过这次全量重算更跟手。
    # 守卫全在函数内部：没有已计算路径的骨架零开销，绝不会新增路径。
    if commit:
        bip_refresh_motion_paths(rig)
    else:
        # 取消：姿态已还原，把拖动中临时写脏的当前帧点也还回去。
        # 不做全量重算（190 ms），取消操作保持跟手。
        bip_restore_live_motion_paths()


# ------------------------------------------------------- 模态内的原生视图导航
def bip_orbit_view(context, event):
    """在 BIP 模态里嵌套启动 Blender 原生视图导航（照驭骨 _orbit_view 的做法）。

    中键 = 旋转，Shift+中键 = 平移，Ctrl+中键 = 缩放。
    view3d.rotate / move / zoom 只认视口 WINDOW 区域，所以必须 temp_override。
    成功后原生操作符自己成为上层 modal，一直吃到松手；我们这边不碰指针、
    不改姿态，等重新收到事件时再重取鼠标投影参考点。
    """
    global _BIP_NAV_LOG
    win = getattr(context, 'window', None)
    area = getattr(context, 'area', None)
    region = getattr(context, 'region', None)
    space = getattr(context, 'space_data', None)
    if win is None or area is None:
        _BIP_NAV_LOG = '失败: 不在 3D 视口'
        return False
    if space is None or getattr(space, 'type', None) != 'VIEW_3D':
        space = area.spaces.active
    if space is None or getattr(space, 'type', None) != 'VIEW_3D':
        _BIP_NAV_LOG = '失败: 非 VIEW_3D 区域'
        return False
    if region is None or getattr(region, 'type', None) != 'WINDOW':
        region = next((r for r in area.regions if r.type == 'WINDOW'), None)
    if region is None:
        _BIP_NAV_LOG = '失败: 找不到 WINDOW 区域'
        return False
    ctrl = bool(getattr(event, 'ctrl', False))
    shift = bool(getattr(event, 'shift', False))
    op_name = 'zoom' if ctrl else ('move' if shift else 'rotate')
    op = getattr(_bip_bpy.ops.view3d, op_name, None)
    if op is None:
        _BIP_NAV_LOG = '失败: 缺少 view3d.%s' % op_name
        return False
    try:
        with context.temp_override(window=win, area=area,
                                   region=region, space_data=space):
            res = op('INVOKE_DEFAULT')
    except Exception as exc:
        _BIP_NAV_LOG = '失败: %s' % str(exc)[:60]
        return False
    ok = bool(res) and bool({'RUNNING_MODAL', 'FINISHED'} & set(res))
    _BIP_NAV_LOG = ('%s: 已启动 (%s)' % (op_name, ', '.join(sorted(res)))
                    if ok else '%s: 原生未接收 %s' % (op_name, res))
    return ok


def bip_warp_to_hand(context, rig, bone_name):
    """视角操作后把指针送回 FK 手控骨（姿态后 head）的屏幕位置。

    与驭骨 _warp_to_bone / bone_window_point 同一套语义：读它的
    warp_after_view 开关与 warp_dx/dy 偏移，投影后夹在视口内再 cursor_warp。
    这里自包含实现，不 patch 也不调用驭骨的方法，避免影响它自己的模态。
    """
    global _BIP_WARP_LOG
    try:
        if _bip_bpy.app.background:
            _BIP_WARP_LOG = '跳过: 后台模式'
            return False
        pref_fn = globals().get('pref')
        if pref_fn is not None and not pref_fn('warp_after_view', True):
            _BIP_WARP_LOG = '关闭: 驭骨偏好 warp_after_view'
            return False
        win = getattr(context, 'window', None)
        area = getattr(context, 'area', None)
        region = getattr(context, 'region', None)
        space = getattr(context, 'space_data', None)
        if win is None or area is None or region is None:
            _BIP_WARP_LOG = '跳过: 无窗口/区域'
            return False
        if getattr(area, 'type', None) != 'VIEW_3D' or region.type != 'WINDOW':
            _BIP_WARP_LOG = '跳过: 非 3D 视口'
            return False
        if space is None or getattr(space, 'type', None) != 'VIEW_3D':
            space = area.spaces.active
        rv3d = getattr(space, 'region_3d', None) if space is not None else None
        if rv3d is None or getattr(win, 'cursor_warp', None) is None:
            _BIP_WARP_LOG = '跳过: 无 region_3d'
            return False
        pb = rig.pose.bones.get(bone_name) if rig is not None else None
        if pb is None:
            _BIP_WARP_LOG = '跳过: 找不到手控骨'
            return False
        co = rig.matrix_world @ pb.head
        p = _bip_v3d.location_3d_to_region_2d(region, rv3d, co)
        if p is None or not (0 <= p.x <= region.width and 0 <= p.y <= region.height):
            _BIP_WARP_LOG = '跳过: 控制器在画面外'
            return False
        dx = pref_fn('warp_dx', 0) if pref_fn else 0
        dy = pref_fn('warp_dy', 0) if pref_fn else 0
        x = int(round(region.x + p.x + dx))
        y = int(round(region.y + p.y + dy))
        x = min(max(x, region.x), region.x + region.width - 1)
        y = min(max(y, region.y), region.y + region.height - 1)
        win.cursor_warp(x, y)
        _BIP_WARP_LOG = '已归位 (%d, %d)' % (x, y)
        return True
    except Exception as exc:
        _BIP_WARP_LOG = '失败: %s' % str(exc)[:50]
        return False


# ------------------------------------------- 切骨：把手掌移动交还驭骨自己
_BIP_HANDOFF_LOG = '未使用'      # 面板诊断：上一次切骨 / 交还结果


def bip_wheel_action(event):
    """复用驭骨的滚轮动作判定，自动跟随 mod_bone_mods 等偏好设置。"""
    fn = globals().get('_modal_wheel_action')
    if fn is None:
        return ''
    try:
        return fn(event)
    except Exception:
        return ''


def bip_active_name(context):
    """直接读 data.bones.active：operator 执行后它立刻更新，不依赖 view_layer。"""
    rig = context.active_object
    if rig is None or rig.type != 'ARMATURE':
        return None
    ab = rig.data.bones.active
    return ab.name if ab is not None else None


def bip_nav_step(context, event):
    """让驭骨自己的切骨算子处理这一格（与模态外按 Alt+滚轮完全一致）。

    它内部会 select_bone / remember_mode / 视图跟随，BIP 不自己复刻，
    保证切骨顺序、集合范围、加选行为与原生零偏差。返回新活动骨名。
    """
    global _BIP_HANDOFF_LOG
    op = getattr(getattr(_bip_bpy.ops, 'pose', None), 'controller_nav_v45', None)
    if op is None:
        _BIP_HANDOFF_LOG = '失败: 缺少 pose.controller_nav_v45'
        return None
    before = bip_active_name(context)
    mode = 'NEXT' if event.type == 'WHEELDOWNMOUSE' else 'PREV'
    try:
        res = op('EXEC_DEFAULT', mode=mode,
                 extend=bool(getattr(event, 'shift', False)))
    except Exception as exc:
        _BIP_HANDOFF_LOG = '失败: 切骨 %s' % str(exc)[:50]
        return None
    after = bip_active_name(context)
    if after is None or after == before:
        _BIP_HANDOFF_LOG = '失败: 切骨未改活动骨 (%s)' % ', '.join(sorted(res))
        return None
    return after


def bip_start_cn_mode(context, mode):
    """启动驭骨自己的 ROT / MOVE 模态（独立函数，便于无头回归里替换）。"""
    return _bip_bpy.ops.pose.cn_quick_rotate_v45(
        'INVOKE_DEFAULT', start_mode=mode)


def bip_handoff_mode(pb):
    """新骨该进 MOVE 还是 ROT —— 直接用驭骨自己的能力判定，语义与它一致。"""
    mov, rot = globals().get('_bone_movable'), globals().get('_bone_rotatable')
    try:
        movable = bool(mov(pb)) if mov else False
    except Exception:
        movable = False
    try:
        rotatable = bool(rot(pb)) if rot else True
    except Exception:
        rotatable = True
    if movable:
        return 'MOVE'
    if rotatable:
        return 'ROT'
    return None      # 移动 / 旋转都全锁：不交还，留在 BIP，免得卡死在无效模态


# ------------------------------------------- v2.7：BIP 拖动期间独显控制器
_BIP_SOLO_ACTIVE = None      # (rig, {bone_name: hide_before})
_BIP_SOLO_KEEP = set()       # 本轮独显的可见集，交还失败时原样恢复
_BIP_SOLO_LOG = '未使用'     # 面板诊断：上一次独显的结果
_BIP_SOLO_FORCE = False      # 无头回归用：后台没有视口也照样验证显隐数据


def bip_solo_enabled():
    """与驭骨共用同一个偏好开关，用户关掉时两边都不独显。"""
    f = globals().get('pref')
    try:
        return bool(f('solo_display', True)) if f else True
    except Exception:
        return True


def bip_solo_keep(context, active):
    keep = set()
    if active:
        keep.add(active)
    try:
        for b in (context.selected_pose_bones or []):
            if b is not None:
                keep.add(b.name)
    except Exception:
        pass
    return keep


def bip_solo_nav_snapshot(rig):
    """照驭骨进模态的做法：在改动任何 hide 之前，用真实显隐算导航快照。

    没有这张快照，bone_visible() 会把被 solo 临时藏起来的骨判成不可见，
    Alt+滚轮切骨（走驭骨自己的切骨算子 + skip_hidden 过滤）就切不动了。
    """
    g = globals()
    g['_NAV_VIS'] = None
    g['_NAV_VIS_RIG'] = None
    fn = g.get('bone_visible')
    try:
        vis = {b.name: (bool(fn(b)) if fn else not b.hide)
               for b in rig.pose.bones}
    except Exception:
        vis = {b.name: not b.hide for b in rig.pose.bones}
    g['_NAV_VIS'] = vis
    # 实测 5.3：pb.id_data 是 Object，bone_visible 里取的是
    # getattr(pb.id_data, 'data') —— 也就是 Armature data。原版驭骨把这里
    # 记成 Object，比较永远不相等，快照分支根本不生效（独显时切骨会被自己
    # 临时藏起来的骨挡住）。BIP 记 data，让这张快照真正起作用。
    g['_NAV_VIS_RIG'] = rig.data
    return vis


def bip_solo_redraw(context):
    try:
        area = getattr(context, 'area', None)
        if area is not None:
            area.tag_redraw()
    except Exception:
        pass


def bip_solo_apply(context, rig, active=None, keep=None):
    """只显示当前控制器。快照只在第一次抓，之后重复调用只是换可见集。"""
    global _BIP_SOLO_ACTIVE, _BIP_SOLO_KEEP, _BIP_SOLO_LOG
    if rig is None or getattr(rig, 'type', '') != 'ARMATURE':
        return False
    if not bip_solo_enabled():
        _BIP_SOLO_LOG = '偏好 solo_display 已关闭'
        return False
    if _bip_bpy.app.background and not _BIP_SOLO_FORCE:
        return False
    if _BIP_SOLO_ACTIVE is None:
        _BIP_SOLO_ACTIVE = (rig, {b.name: bool(b.hide)
                                  for b in rig.pose.bones})
    # 必须在写 hide 之前；交还失败重新 apply 时 hide 已被还原成原始值。
    bip_solo_nav_snapshot(rig)
    ks = set(keep if keep is not None else bip_solo_keep(context, active))
    _BIP_SOLO_KEEP = set(ks)
    n = 0
    for b in rig.pose.bones:
        try:
            b.hide = b.name not in ks
            n += 1
        except Exception:
            pass
    _BIP_SOLO_LOG = '只显示 %s / 共 %d 骨' % (
        ', '.join(sorted(ks)) or '无', n)
    bip_solo_redraw(context)
    return True


def bip_solo_set_keep(context, rig, active=None, keep=None):
    """换到另一只手：沿用同一份快照换可见集，绝不重抓。"""
    global _BIP_SOLO_KEEP, _BIP_SOLO_LOG
    if _BIP_SOLO_ACTIVE is None:
        return bip_solo_apply(context, rig, active, keep)
    ks = set(keep if keep is not None else bip_solo_keep(context, active))
    _BIP_SOLO_KEEP = set(ks)
    for b in rig.pose.bones:
        try:
            b.hide = b.name not in ks
        except Exception:
            pass
    _BIP_SOLO_LOG = '切骨 · 只显示 %s' % (', '.join(sorted(ks)) or '无')
    bip_solo_redraw(context)
    return True


def bip_solo_reapply(context):
    """交还驭骨失败 / 提交异常：用本轮记录的可集原样重新独显。

    不能重新按 context.selected_pose_bones 算——切骨会改变选中骨，
    那样会凭空多亮出别的控制器。
    """
    if _BIP_SOLO_ACTIVE is None:
        return False
    return bip_solo_apply(context, _BIP_SOLO_ACTIVE[0],
                          keep=_BIP_SOLO_KEEP)


def bip_solo_restore(context=None, forget=True):
    """按开拖前的快照还原显隐；forget=False 时留着快照给交还失败重试。"""
    global _BIP_SOLO_ACTIVE, _BIP_SOLO_KEEP
    item = _BIP_SOLO_ACTIVE
    if item is None:
        return False
    rig, saved = item
    if forget:
        _BIP_SOLO_ACTIVE = None
        _BIP_SOLO_KEEP = set()
    try:
        for name, hidden in saved.items():
            b = rig.pose.bones.get(name)
            if b is not None:
                b.hide = hidden
    except Exception:
        pass
    # 这两个是驭骨的模块级全局，不清会污染普通状态下的切骨过滤。
    g = globals()
    g['_NAV_VIS'] = None
    g['_NAV_VIS_RIG'] = None
    bip_solo_redraw(context)
    return True


def bip_solo_forget():
    """交还成功：显隐已由驭骨自己的模态接管，丢掉本轮快照。"""
    global _BIP_SOLO_ACTIVE, _BIP_SOLO_KEEP
    _BIP_SOLO_ACTIVE = None
    _BIP_SOLO_KEEP = set()


# ------------------------------------------------------------------ operator
class POSE_OT_cn_bip_drag_v45(_bip_bpy.types.Operator):
    """BIP 整臂拖动：IK 只是手势，数据写回 FK 旋转。"""
    bl_idname = 'pose.cn_bip_drag_v45'
    bl_label = 'BIP 拖动手掌 (ARP / FK)'
    bl_description = '拖动手掌，整臂 IK 跟随，结果写回 FK 控制骨旋转'
    bl_options = {'REGISTER', 'UNDO'}

    side: _BipString(name='Side', default='', options={'SKIP_SAVE'})

    @classmethod
    def poll(cls, context):
        if context.mode != 'POSE':
            return False
        return bip_resolve(context) is not None

    def invoke(self, context, event):
        amap = bip_resolve(context)
        if self.side and amap and self.side != amap['side']:
            amap = None
        if amap is None:
            rig = context.active_object
            if not bip_is_arp_rig(rig):
                self.report({'ERROR'}, '未检测到 ARP 骨架')
            else:
                self.report({'ERROR'}, '无法解析手臂 FK/IK 链，请检查骨骼命名')
            return {'CANCELLED'}
        try:
            self.state = bip_begin(amap['rig'], amap, prepare=True)
        except Exception as exc:
            self.report({'ERROR'}, '启动失败: %s' % exc)
            return {'CANCELLED'}
        self.rig = amap['rig']
        # v2.7：与驭骨 MOVE/ROT 一样，进模态就只显示正在操作的控制器。
        bip_solo_apply(context, self.rig, amap['fk_hand'])
        self.grab_world = self.rig.matrix_world @ self.state['grab_arm']
        mx = getattr(event, 'mouse_region_x', event.mouse_x)
        my = getattr(event, 'mouse_region_y', event.mouse_y)
        self.last_3d = self._mouse_to_world(context, mx, my)
        self.offset = _BipVector((0.0, 0.0, 0.0))
        # 视图导航状态：视角变了必须重取鼠标投影参考点，否则同一屏幕位移
        # 在新视角下会被解释成很大的世界位移，姿态会突然跳一下。
        self._nav_active = False      # 原生导航模态仍在跑
        self._nav_pending = False     # 导航已启动，等它退场
        self._nav_t = 0.0
        self._alt_nav = False         # Alt+中键 = 原生正交视角拖放
        self.warp_bone = amap['fk_hand']
        self._warp_pending = False    # 视角动过，等导航退场后把指针送回控制器
        self._warp_t = 0.0
        self._handed_off = False     # 已交还驭骨，避免重复提交 / 二次 bip_end
        context.window_manager.modal_handler_add(self)
        if context.area:
            context.area.header_text_set(
                'BIP 拖动 [%s]: 鼠标 | 中键转视角 | Alt+滚轮切骨(自动转旋转/移动) | '
                'Shift 精细 | 左键·Enter 确认 | Esc·右键 取消'
                % ('左臂' if amap['side'] == '.l' else '右臂'))
        return {'RUNNING_MODAL'}

    def _rebase_after_view(self):
        """视角/光标归位后重锚：下一帧只取点，且从当前手位置继续累积位移。

        只清 last_3d 还不够：光标 warp 回控制器是一次瞬移，视角也换了投影
        平面，下一帧的 (cur - last_3d) 可能是个很大的世界位移，会把 IK 目标
        甩出去、手掌朝向跟着骤变。把 grab_world 重锚到当前手位置、offset 归零，
        姿态一点不动，后续鼠标增量从当前位置连续接上。
        """
        self.last_3d = None
        if not getattr(self, 'state', None) or not getattr(self, 'rig', None):
            return
        try:
            hand = self.rig.pose.bones[self.state['map']['fk_hand']]
            self.grab_world = (self.rig.matrix_world @
                               hand.matrix.translation.copy())
            self.offset = _BipVector((0.0, 0.0, 0.0))
        except Exception:
            pass

    def _queue_warp(self):
        self._warp_pending = True
        self._warp_t = _bip_time.perf_counter()

    def _warp_cursor_to_hand(self, context):
        ok = bip_warp_to_hand(context, self.rig, self.warp_bone)
        # 指针瞬移了：下一帧的 MOUSEMOVE 只取新参考点，绝不能算成一次拖动
        self._rebase_after_view()
        return ok

    def _mouse_to_world(self, context, mx, my):
        region, rv3d = context.region, context.region_data
        if region is None or rv3d is None:
            return None
        try:
            return _bip_v3d.region_2d_to_location_3d(
                region, rv3d, (mx, my), self.grab_world)
        except Exception:
            return None

    def _finish(self, context, commit):
        if getattr(self, '_handed_off', False):
            # 已由 _handoff 提交并结束，不能再 bip_end 一次
            return {'FINISHED'}
        # 先还原显隐，再让 bip_end 重算路径：重算时看到的是正常可见性。
        bip_solo_restore(context)
        bip_end(self.state, commit=commit)
        if context.area:
            context.area.header_text_set(None)
        return {'FINISHED'} if commit else {'CANCELLED'}

    def _handoff(self, context, new_name):
        """提交手掌姿态，结束 BIP，把控制权交还驭骨在新骨上的 ROT / MOVE。"""
        global _BIP_HANDOFF_LOG
        pb = self.rig.pose.bones.get(new_name) if new_name else None
        if pb is None:
            return None
        mode = bip_handoff_mode(pb)
        if mode is None:
            _BIP_HANDOFF_LOG = '%s 全锁，继续 BIP' % new_name
            return None
        old = self.state['map']['fk_hand']
        # v2.7：交还前必须还原显隐。驭骨 invoke 会抓「进模态之前」的显隐快照，
        # 若此刻仍独显，它就把 hide=True 当成原始状态，切一次骨之后整副骨架
        # 的控制器会永久看不见。forget=False 留着快照给失败时重新 apply。
        bip_solo_restore(context, forget=False)
        try:
            bip_end(self.state, commit=True)   # 已摆好的手掌姿态保留（切骨=提交点）
        except Exception as exc:
            bip_solo_reapply(context)
            _BIP_HANDOFF_LOG = '失败: 提交 %s' % str(exc)[:50]
            return None
        self._handed_off = True
        ok = False
        try:
            # hook 只在 start_mode='MOVE' 时分流；新骨不是手控骨，这里绝不会
            # 再绕回 BIP，驭骨按自己的能力判定进入原生 ROT / MOVE 模态。
            res = bip_start_cn_mode(context, mode)
            ok = bool({'RUNNING_MODAL', 'FINISHED'} & set(res))
            _BIP_HANDOFF_LOG = '%s → %s · 交还驭骨 %s (%s)' % (
                old, new_name, mode, ', '.join(sorted(res)))
        except Exception as exc:
            _BIP_HANDOFF_LOG = '%s → %s 交还失败: %s' % (
                old, new_name, str(exc)[:50])
        if ok:
            # 驭骨模态已自己抓了正确的快照并接管显隐，本轮 BIP 快照作废。
            bip_solo_forget()
        else:
            # 姿态已安全提交，只是模态没接上：恢复 BIP 独显，别把整副骨架
            # 亮着扔给用户；快照还在，重新隐藏不会把状态记脏。
            bip_solo_reapply(context)
            try:
                self.report({'WARNING'},
                            '已切到 %s，但未自动进入%s；再按一次%s键即可'
                            % (new_name, '旋转' if mode == 'ROT' else '移动',
                               '旋转' if mode == 'ROT' else '移动'))
            except Exception:
                pass
        if context.area:
            context.area.header_text_set(None)
        return {'FINISHED'}

    def _reanchor(self, context, amap):
        """切到另一根 FK 手控骨：提交当前手臂，重新锚定后继续 BIP 拖动。"""
        global _BIP_HANDOFF_LOG
        old = self.state['map']
        # 同一条手臂内的近邻（ARP 的 c_hand_fk_scale_fix 之类）：链条完全一样，
        # 不打断当前拖动，只换锚点与光标归位目标，避免无谓提交 / 锚点跳变。
        if all(old.get(k) == amap.get(k) for k in
               ('side', 'fk_fore', 'fk_upper', 'ik_target', 'ik_fore', 'ik_upper')):
            old['fk_hand'] = amap['fk_hand']
            self.warp_bone = amap['fk_hand']
            bip_solo_set_keep(context, amap['rig'], amap['fk_hand'])
            self._rebase_after_view()
            _BIP_HANDOFF_LOG = '已切到 %s，同一手臂继续 BIP' % amap['fk_hand']
            return {'RUNNING_MODAL'}
        try:
            bip_end(self.state, commit=True)
            st = bip_begin(amap['rig'], amap)
        except Exception as exc:
            _BIP_HANDOFF_LOG = '失败: 重新锚定 %s' % str(exc)[:50]
            return {'RUNNING_MODAL'}
        self.state = st
        self.rig = amap['rig']
        # 换到另一只手：可见集跟着新手控骨走，快照仍是开拖前那一份。
        bip_solo_set_keep(context, self.rig, amap['fk_hand'])
        self.grab_world = self.rig.matrix_world @ st['grab_arm']
        self.warp_bone = amap['fk_hand']
        self.offset = _BipVector((0.0, 0.0, 0.0))
        self._rebase_after_view()
        _BIP_HANDOFF_LOG = '已切到 %s，继续 BIP' % amap['fk_hand']
        return {'RUNNING_MODAL'}

    def _on_bone_changed(self, context, new_name):
        """切骨后的统一分流：仍是手控骨 -> 重锚；否则 -> 交还驭骨。"""
        if new_name == self.state['map']['fk_hand']:
            self._rebase_after_view()
            return {'RUNNING_MODAL'}
        pb = self.rig.pose.bones.get(new_name)
        amap = bip_resolve(context, active=pb) if pb is not None else None
        if amap is not None:
            return self._reanchor(context, amap)
        return self._handoff(context, new_name) or {'RUNNING_MODAL'}

    def modal(self, context, event):
        try:
            return self._modal(context, event)
        except Exception as exc:
            bip_solo_restore(context)
            try:
                if not getattr(self, '_handed_off', False):
                    bip_end(self.state, commit=False)
            except Exception:
                pass
            try:
                self.report({'ERROR'}, 'BIP 拖动已中断: %s' % exc)
            except Exception:
                pass
            if context.area:
                context.area.header_text_set(None)
            return {'CANCELLED'}

    def _modal(self, context, event):
        et = event.type
        amap0 = (self.state or {}).get('map') or {}
        anchor = amap0.get('fk_hand')

        # --- 兜底：方向键 / 切集合 / 直接点选也可能改了活动骨。只要不再是
        # 本次锚定的手控骨，就按新骨交还驭骨，避免「骨切了却还卡在手掌移动」。
        if anchor and not getattr(self, '_handed_off', False) and et != 'TIMER':
            cur = bip_active_name(context)
            if cur and cur != anchor:
                return self._on_bone_changed(context, cur)

        # --- 切骨手势（默认 Alt+滚轮，跟随驭骨偏好）：同步调用驭骨自己的
        # 切骨算子，再分流。之前一律 PASS_THROUGH 当视角，活动骨被原生改了，
        # BIP 却还锚在原手掌上，于是「切到上臂还是手掌移动」。
        if (et in {'WHEELUPMOUSE', 'WHEELDOWNMOUSE'}
                and event.value == 'PRESS'
                and bip_wheel_action(event) == 'BONE'
                and anchor):
            new_name = bip_nav_step(context, event)
            if new_name is None:
                return {'RUNNING_MODAL'}
            return self._on_bone_changed(context, new_name)

        # --- Alt + 中键：原生「拖到最近正交视角」/ 拾取对焦，整段放行。
        # CLICK_DRAG 是键位合成事件，PRESS 被吞就永远形不成，必须 PASS_THROUGH。
        if et == 'MIDDLEMOUSE' and getattr(event, 'alt', False):
            if event.value == 'PRESS':
                self._alt_nav = True
            elif event.value == 'RELEASE' and self._alt_nav:
                self._alt_nav = False
                self._rebase_after_view()
                self._queue_warp()
            return {'PASS_THROUGH'}
        if et in _BIP_MOVES and self._alt_nav:
            if getattr(event, 'alt', False):
                return {'PASS_THROUGH'}
            self._alt_nav = False
            self._rebase_after_view()
            self._queue_warp()
            return {'RUNNING_MODAL'}

        # --- 原生导航模态退场：重新收到事件说明那层已经结束。
        # 时间下限防止在人家转到一半时就把参考点搬走。
        if self._nav_pending and et != 'TIMER':
            if _bip_time.perf_counter() - self._nav_t >= _BIP_NAV_SETTLE:
                self._nav_pending = False
                self._nav_active = False
                self._rebase_after_view()
            elif et in _BIP_MOVES:
                return {'RUNNING_MODAL'}

        # --- 视角动过且原生导航已退场：把指针送回控制器（与驭骨一致）。
        # 中键事件本身跳过，等导航彻底结束后再归位，避免打扰正在转的视角。
        if (self._warp_pending and et != 'TIMER' and et != 'MIDDLEMOUSE'
                and not self._nav_active
                and _bip_time.perf_counter() - self._warp_t >= _BIP_NAV_SETTLE):
            self._warp_pending = False
            if self._warp_cursor_to_hand(context) and et in _BIP_MOVES:
                # 这一帧的坐标还是 warp 之前的，不能拿它当参考点
                return {'RUNNING_MODAL'}

        # --- 中键：嵌套启动原生旋转 / 平移 / 缩放，姿态一点不动。
        if et == 'MIDDLEMOUSE':
            if event.value == 'PRESS':
                self._rebase_after_view()
                self._queue_warp()
                if bip_orbit_view(context, event):
                    self._nav_active = True
                    self._nav_pending = True
                    self._nav_t = _bip_time.perf_counter()
                    return {'RUNNING_MODAL'}
                # 起不来（非 3D 视口等）就交还原生键位
                return {'PASS_THROUGH'}
            return {'RUNNING_MODAL'}

        # --- 滚轮 / 小键盘 / 触控板视角手势：全部放行，视角变了重取参考点。
        if et in _BIP_VIEW_PASS:
            self._rebase_after_view()
            self._queue_warp()
            return {'PASS_THROUGH'}

        # --- 鼠标拖动
        if et in _BIP_MOVES:
            if self._nav_active:
                # 原生导航还在跑：鼠标事件必须继续往下传，吞了它视角就转不动。
                # 这一帧也不碰姿态，导航结束后由 _nav_pending 重取参考点。
                self.last_3d = None
                return {'PASS_THROUGH'}
            mx = getattr(event, 'mouse_region_x', event.mouse_x)
            my = getattr(event, 'mouse_region_y', event.mouse_y)
            cur = self._mouse_to_world(context, mx, my)
            if cur is not None:
                if self.last_3d is None:
                    self.last_3d = cur
                else:
                    factor = _BIP_FINE if getattr(event, 'shift', False) else 1.0
                    self.offset += (cur - self.last_3d) * factor
                    self.last_3d = cur
                    goal_world = self.grab_world + self.offset
                    goal_arm = self.rig.matrix_world.inverted() @ goal_world
                    bip_solve(self.state, goal_arm)
                    bip_live_motion_paths(self.rig)
            return {'RUNNING_MODAL'}

        if et == 'LEFTMOUSE' and event.value == 'RELEASE':
            # 导航刚起来的那一瞬间漏过来的松手不当确认
            if (self._nav_active and
                    _bip_time.perf_counter() - self._nav_t < _BIP_NAV_SETTLE):
                return {'RUNNING_MODAL'}
            self._nav_active = self._nav_pending = False
            return self._finish(context, True)
        if et in {'RET', 'NUMPAD_ENTER', 'SPACE'} and event.value == 'PRESS':
            return self._finish(context, True)
        if et in {'ESC', 'RIGHTMOUSE'} and event.value == 'PRESS':
            return self._finish(context, False)
        # 其余按键吞掉：放行 Z/X/M 之类会二次触发驭骨自己的模态，叠出双份 HUD。
        return {'RUNNING_MODAL'}


# ------------------------------------- v2.5：驭骨 MOVE/ROT 的路径挂钩
# 不改驭骨源码，运行时包装两个方法，register/unregister 成对安装与还原：
#   _modal               —— 每个拖动事件后做一次 O(1) 的实时点更新；
#   _update_motion_paths —— 提交点从「异步等手停」改成同步全量重算。
_BIP_ORIG_MODAL = None
_BIP_ORIG_UPDATE_MPATH = None


def _bip_modal_mpath_wrapped(self, context, event):
    ret = _BIP_ORIG_MODAL(self, context, event)
    try:
        if ret and 'RUNNING_MODAL' in ret:
            bip_live_motion_paths(getattr(self, '_rig', None))
        elif ret and ('CANCELLED' in ret or 'FINISHED' in ret):
            # 取消：原模态已还原姿态，临时路径点同步还原；确认时全量重算
            # 已在 _update_motion_paths 里做完，这里只兜底没走到重算的情况。
            if 'CANCELLED' in ret:
                bip_restore_live_motion_paths()
            else:
                _BIP_LIVE_DIRTY.clear()
    except Exception:
        pass
    return ret


def _bip_update_motion_paths_wrapped(self, context):
    """提交点：当场同步重算，不再排队等 0.3 s 手停（对齐原生 transform）。"""
    rig = getattr(self, '_rig', None)
    if rig is None:
        return _BIP_ORIG_UPDATE_MPATH(self, context)
    # 守卫与驭骨原实现一致：偏好关闭 / 没开自动关键帧 / 没有已计算路径
    try:
        _pref = globals().get('pref')
        if callable(_pref) and not _pref('mpath_refresh', True):
            return False
        if not context.scene.tool_settings.use_keyframe_insert_auto:
            return False
        if not any(p.motion_path for p in rig.pose.bones):
            return False
    except Exception:
        return _BIP_ORIG_UPDATE_MPATH(self, context)
    if bip_refresh_motion_paths(rig):
        return True
    return _BIP_ORIG_UPDATE_MPATH(self, context)   # 同步不可用时退回异步兜底


def _bip_install_mpath_hooks():
    global _BIP_ORIG_MODAL, _BIP_ORIG_UPDATE_MPATH
    cls = globals().get('POSE_OT_cn_quick_rotate_v45')
    if cls is None:
        return False
    cur = cls.__dict__.get('_modal')
    if cur is not None and getattr(cur, '__name__', '') != '_bip_modal_mpath_wrapped':
        _BIP_ORIG_MODAL = cur
        cls._modal = _bip_modal_mpath_wrapped
    cur = cls.__dict__.get('_update_motion_paths')
    if (cur is not None
            and getattr(cur, '__name__', '') != '_bip_update_motion_paths_wrapped'):
        _BIP_ORIG_UPDATE_MPATH = cur
        cls._update_motion_paths = _bip_update_motion_paths_wrapped
    return True


def _bip_remove_mpath_hooks():
    global _BIP_ORIG_MODAL, _BIP_ORIG_UPDATE_MPATH
    try:
        bip_restore_live_motion_paths()   # 卸载时不能留下预览姿态的路径点
    except Exception:
        pass
    cls = globals().get('POSE_OT_cn_quick_rotate_v45')
    if cls is not None:
        try:
            if (_BIP_ORIG_MODAL is not None
                    and getattr(cls.__dict__.get('_modal'), '__name__', '')
                    == '_bip_modal_mpath_wrapped'):
                cls._modal = _BIP_ORIG_MODAL
        except Exception:
            pass
        try:
            if (_BIP_ORIG_UPDATE_MPATH is not None
                    and getattr(cls.__dict__.get('_update_motion_paths'),
                                '__name__', '')
                    == '_bip_update_motion_paths_wrapped'):
                cls._update_motion_paths = _BIP_ORIG_UPDATE_MPATH
        except Exception:
            pass
    _BIP_ORIG_MODAL = None
    _BIP_ORIG_UPDATE_MPATH = None


# ------------------------------------- v2.3：驭骨自动插帧入口的连续性挂钩
_BIP_ORIG_INSERT_KEYS = None


def _bip_insert_keys_wrapped(self, context):
    """驭骨自由旋转/移动自动插帧前，先把旋转通道换成连续等价表示。

    自由旋转每帧写的是 matrix_basis，Blender 会重新分解出规范分支的欧拉
    / 四元数；ARP 工程里这些骨的 F-Curve 常在远分支，直接插帧就会让
    中间帧绕远路。这里只换数值表示、姿态一字不改。
    """
    global _BIP_REPAIR_LAST
    rig, names = None, []
    try:
        rig = getattr(self, '_rig', None)
        ts = getattr(context.scene, 'tool_settings', None)
        if rig is not None and ts is not None and ts.use_keyframe_insert_auto:
            frame = context.scene.frame_current
            for b in (getattr(self, '_targets', None) or ()):
                p = rig.pose.bones.get(b.name)
                if p is not None:
                    bip_align_rotation(rig, p, frame)
                    names.append(p.name)
    except Exception:
        pass
    ret = _BIP_ORIG_INSERT_KEYS(self, context)
    # v2.6：插完帧再理顺邻域分支（只换表示，姿态不变）。0.3 s 节流，
    # 连续旋转时不会每个事件都全扫一遍动作曲线。
    if rig is not None and names:
        try:
            t = _bip_time.perf_counter()
            if t - _BIP_REPAIR_LAST >= _BIP_REPAIR_THROTTLE:
                _BIP_REPAIR_LAST = t
                bip_repair_rotation_curves(rig, names, '驭骨插帧')
        except Exception:
            pass
    return ret


def _bip_install_insert_hook():
    """运行时包装 POSE_OT_cn_quick_rotate_v45._insert_keys（不改驭骨源码）。

    这样做扩展块与驭骨之间仍然只有原来的 3 个文本挂钩点，升级驭骨后
    重跑安装器即可；包装在 register/unregister 里成对安装与还原。
    """
    global _BIP_ORIG_INSERT_KEYS
    cls = globals().get('POSE_OT_cn_quick_rotate_v45')
    if cls is None:
        return False
    cur = cls.__dict__.get('_insert_keys')
    if cur is None:
        return False
    if getattr(cur, '__name__', '') == '_bip_insert_keys_wrapped':
        return True
    _BIP_ORIG_INSERT_KEYS = cur
    cls._insert_keys = _bip_insert_keys_wrapped
    return True


def _bip_remove_insert_hook():
    global _BIP_ORIG_INSERT_KEYS
    cls = globals().get('POSE_OT_cn_quick_rotate_v45')
    if cls is not None and _BIP_ORIG_INSERT_KEYS is not None:
        try:
            cur = cls.__dict__.get('_insert_keys')
            if getattr(cur, '__name__', '') == '_bip_insert_keys_wrapped':
                cls._insert_keys = _BIP_ORIG_INSERT_KEYS
        except Exception:
            pass
    _BIP_ORIG_INSERT_KEYS = None


class POSE_OT_cn_bip_fix_curves_v45(_bip_bpy.types.Operator):
    """一键修复已有旋转关键帧的等价表示分支（姿态不变）。"""
    bl_idname = 'pose.cn_bip_fix_curves_v45'
    bl_label = '修复旋转曲线分支'
    bl_description = ('把旋转关键帧换成与相邻帧连续的等价表示，姿态完全不变；'
                      '默认只处理 FK 手臂链，修以前被旋转模式写坏的旧关键帧')
    bl_options = {'REGISTER', 'UNDO'}

    dry_run: _BipBool(name='仅检查', default=False, options={'HIDDEN'})
    scope_all: _BipBool(name='整条骨架', default=False, options={'HIDDEN'})

    @classmethod
    def poll(cls, context):
        obj = context.active_object
        return obj is not None and obj.type == 'ARMATURE'

    def execute(self, context):
        global _BIP_FIX_LOG
        rig = context.active_object
        names = None
        if not self.scope_all:
            names = bip_arm_rot_bone_names(rig) or None
        st = bip_normalize_rotation_curves(
            rig, bone_names=names, dry_run=bool(self.dry_run))
        scope = '整条骨架' if self.scope_all else 'FK 手臂链'
        msg = ('%s(%s)：%d 组 / %d 关键帧 / 修正 %d 帧 / 姿态不变'
               % ('检查' if self.dry_run else '已修复', scope,
                  st['groups'], st['keys'], st['fixed']))
        if st['bad']:
            msg += ' / %d 帧跳过' % st['bad']
        if st.get('partial'):
            msg += ' / %d 帧缺通道' % st['partial']
        _BIP_FIX_LOG = msg
        self.report({'INFO'}, msg)
        try:
            _bip_bpy.context.view_layer.update()
        except Exception:
            pass
        # v2.6：修完当场重算已有运动路径，打结的轨迹立刻消失。
        if not self.dry_run and st['fixed']:
            try:
                bip_refresh_motion_paths(rig)
            except Exception:
                pass
        return {'FINISHED'}


class VIEW3D_PT_cn_bip_v45(_bip_bpy.types.Panel):
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'BIP'
    bl_label = 'BIP 拖动手臂 (ARP / FK)'

    def draw(self, context):
        layout = self.layout
        try:
            key_label = globals()['move_key_label']()
        except Exception:
            key_label = '移动键'
        layout.label(text='已并入驭骨 v2.7 · BIP 也独显控制器',
                     icon='CHECKMARK')
        box = layout.box()
        box.label(text='选中 FK 手控骨按 %s' % key_label, icon='LINKED')
        box.label(text='= 整臂 BIP 拖动（仍写 FK 旋转）')
        box.label(text='其它骨骼按 %s 照常走驭骨' % key_label)
        box.label(text='每次移动完自动刷新已有运动路径')
        box.label(text='拖动时只显示当前控制器 · 跟随驭骨偏好')
        box = layout.box()
        box.label(text='拖动中可自由看视角:', icon='VIEW3D')
        box.label(text='中键转 / Shift+中键平移 / Ctrl+中键缩放')
        box.label(text='滚轮缩放 · 小键盘切视图 · Alt+中键正交')
        box.label(text='视角后光标自动回控制器', icon='INFO')
        box = layout.box()
        box.label(text='Alt+滚轮切骨 → 交还驭骨:', icon='LINKED')
        box.label(text='可旋转的骨自动进旋转，可移动的进移动')
        box.label(text='路径只重算已计算的，不新增 · 跟随驭骨偏好')
        box.label(text='上次切骨: %s' % _BIP_HANDOFF_LOG, icon='INFO')
        box.label(text='上次导航: %s' % _BIP_NAV_LOG, icon='INFO')
        box.label(text='上次归位: %s' % _BIP_WARP_LOG, icon='INFO')
        box.label(text='上次路径: %s' % _BIP_MPATH_LOG, icon='INFO')
        box.label(text='拖动中: %s' % _BIP_LIVE_LOG, icon='INFO')
        box.label(text='求解模式: %s' % _BIP_MODE_LOG, icon='INFO')
        box.label(text='独显: %s' % _BIP_SOLO_LOG, icon='INFO')
        # v2.3：已有动画被旧版旋转模式写坏过关键帧时，一键归一化分支。
        fixbox = layout.box()
        fixbox.label(text='旧关键帧中间插值乱转？', icon='ERROR')
        fixbox.label(text='换成连续等价表示 · 关键帧姿态不变')
        frow = fixbox.row(align=True)
        op = frow.operator('pose.cn_bip_fix_curves_v45', text='检查',
                           icon='ZOOM_ALL')
        op.dry_run, op.scope_all = True, False
        op = frow.operator('pose.cn_bip_fix_curves_v45', text='修手臂',
                           icon='FILE_TICK')
        op.dry_run, op.scope_all = False, False
        op = frow.operator('pose.cn_bip_fix_curves_v45', text='整骨架')
        op.dry_run, op.scope_all = False, True
        fixbox.label(text=_BIP_FIX_LOG, icon='INFO')
        fixbox.label(text='自动理顺: %s' % _BIP_REPAIR_LOG, icon='INFO')
        layout.prop(context.scene, 'cn_bip_enabled', text='启用 BIP 接管')
        amap = _bip_bpy.context.active_object and bip_resolve(context)
        if amap:
            layout.label(text='探测: %s (%s)' % (amap['fk_hand'], amap['side']))
            col = layout.column(align=True)
            col.label(text='FK: %s / %s' % (amap['fk_upper'], amap['fk_fore']))
            col.label(text='IK: %s / %s' % (amap['ik_upper'], amap['ik_fore']))
            col.label(text='目标: %s' % amap['ik_target'])
            # v2.0：让用户直接看到前臂被锁的通道是不是恒为 0，不必切 N 面板
            try:
                hp = context.active_object.pose.bones[amap['fk_fore']]
                hi = bip_hinge_info(hp)
                if hi:
                    if hi['mode'] in ('QUATERNION', 'AXIS_ANGLE'):
                        letters, off = 'WXYZ', 1
                    else:
                        letters, off = 'XYZ', 0
                    locked = ','.join(letters[j - off] for j in hi['locked'])
                    vals = [x for x in getattr(hp, hi['path'])]
                    col.label(text='前臂锁轴 %s: %s' % (
                        locked, ' / '.join('%.5f' % vals[j]
                                          for j in hi['locked'])),
                        icon='LOCKED')
                else:
                    col.label(text='前臂全自由度 · 不投影')
            except Exception:
                pass
        else:
            layout.label(text='未识别到手臂 FK 手控骨', icon='INFO')
        row = layout.row(align=True)
        op = row.operator('pose.cn_bip_drag_v45', text='重新探测并拖左手', icon='HAND')
        op.side = '.l'
        op = row.operator('pose.cn_bip_drag_v45', text='右手', icon='HAND')
        op.side = '.r'
        layout.prop(context.scene, 'cn_bip_default_side', text='默认侧')


def bip_register():
    global _BIP_OWNED
    if _BIP_OWNED:
        return
    try:
        _bip_bpy.types.Scene.cn_bip_enabled = _BipBool(
            name='启用 BIP 接管', default=True,
            description='选中 FK 手控骨时用驭骨的移动键直接做整臂 BIP 拖动')
        _bip_bpy.types.Scene.cn_bip_default_side = _BipEnum(
            name='默认侧', items=[('.l', '左臂', ''), ('.r', '右臂', '')],
            default='.l')
    except Exception as exc:
        print('[ARP BIP] scene props: %s' % exc)
    try:
        parent = getattr(_bip_bpy.types, 'VIEW3D_PT_controller_nav_v45', None)
        if parent is not None and getattr(parent, 'bl_category', None):
            VIEW3D_PT_cn_bip_v45.bl_category = parent.bl_category
    except Exception:
        pass
    for cls in (POSE_OT_cn_bip_drag_v45,
                POSE_OT_cn_bip_fix_curves_v45,
                VIEW3D_PT_cn_bip_v45):
        try:
            _bip_bpy.utils.register_class(cls)
        except Exception as exc:
            print('[ARP BIP] register class %s: %s' % (cls.__name__, exc))
    try:
        _bip_install_insert_hook()
    except Exception as exc:
        print('[ARP BIP] insert hook: %s' % exc)
    try:
        _bip_install_mpath_hooks()
    except Exception as exc:
        print('[ARP BIP] mpath hook: %s' % exc)
    try:
        _BIP_ALIGN_CACHE.clear()
    except Exception:
        pass
    _BIP_OWNED = True


def bip_unregister():
    global _BIP_OWNED
    if not _BIP_OWNED:
        return
    # 禁用插件时若 BIP 模态还挂着，先把显隐还原，别把骨架留在独显状态。
    bip_solo_restore()
    _bip_remove_mpath_hooks()
    _bip_remove_insert_hook()
    for cls in (VIEW3D_PT_cn_bip_v45,
                POSE_OT_cn_bip_fix_curves_v45,
                POSE_OT_cn_bip_drag_v45):
        try:
            _bip_bpy.utils.unregister_class(cls)
        except Exception:
            pass
    for attr in ('cn_bip_enabled', 'cn_bip_default_side'):
        try:
            delattr(_bip_bpy.types.Scene, attr)
        except Exception:
            pass
    _BIP_OWNED = False
# <<< ARP BIP EXT END (WorkBuddy) <<<
