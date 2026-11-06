# -*- coding: utf-8 -*-
# Controller Navigator v5.0.0 — IK/FK 一键切换 + 全键位自定义 + 集合独占
#
# v5.2.39 — 修「旋转还是有几率卡一下、然后姿态猛跳一段」：
#   现象：不是每次都出现，偶尔拖到一半顿一下，接着姿态窜出去一段。
#   实测定位（rig.002 / 420 骨 / 219 根带路径）：
#   ① v5.2.37 把运动路径刷新挪进 bpy.app.timers，但 timer 是**在模态事件之间**
#      执行的 —— pose.paths_update() 是同步全量重算，实测 154~161 ms，正好
#      落在拖拽中途，把事件循环冻住这么久；这期间鼠标位移全攒着，恢复后作为
#      **一个** MOUSEMOVE 进来，dx/dy 上百像素，一次吃进去就是「猛跳一段」。
#      拖拽事件本身只 0.093 ms/帧（cProfile 实测），所以问题一直在事件之外。
#      修法：模块级 _GESTURE_T 记录最近一次模态事件时间，timer 在「手还在动」
#      （<0.30 s）时用返回间隔自我重排、不执行；手停下才刷。上限 16 s 防饿死。
#   ② 位移突变守卫补上「中途」这一档：原来只有「本段第一帧」有 1/3 视口守卫，
#      中途攒出来的大位移照单全收。现在任何一帧位移超过 180 px 都按比例截断；
#      单轴 RIM 的方位角另加 ±25°/帧 限幅（它不读 dx，直接吃绝对方位角）。
#   效果：卡顿源头（160 ms 同步重算）不再出现在手势里；万一被别的阻塞（撤销、
#   大重估）卡住，姿态最多前进一小段，不会飞出去。
#
# v5.2.38 — 修「自由旋转时姿态会突然猛甩 / 越转越歪」（老问题，v5.2.30 就有）：
#   根因：`_ensure_baseline()` 发现姿态被动画重估拽走后，会先把姿态写回来、
#   再把当前姿态冻结成新基线（rebaseline），但**没有清空累计旋转量**。
#   而 `_apply()` 每帧都是「从基线出发、叠加绝对累计角」，基线一旦前移，
#   同一段旋转就被叠加第二遍 —— 实测 q=36° 时单帧多转 36°、q=98° 时多转
#   98°，连续触发时越甩越歪，HUD 上的角度却还停在原处。
#   修法：rebaseline 之后调 `_bake_gesture()`，把这段旋转记进「累计」并把
#   `_quat/_quat0/_track_dx/_ang_h/_rim_acc` 全部归零（位移走 `_mv_bases`，
#   自带基准，不受影响）。单轴 RIM 的 `_s_phi` 一起重置，避免下一帧把方位
#   角累计又算回旧总量。
#   ② 触发源：`_sync_keys` 原来限流 40ms，曲线最多落后姿态 40ms —— 实测
#   拖拽中途一次 frame_set，姿态被拽回 43.7°，接着就撞上上面那条叠加。
#   改成逐帧同步（实测单骨 0.16ms、30 骨 0.68ms、每帧总代价 +0.024ms）。
#   实测（rig.002 / 420 骨）：旧版注入重估后单帧最大 102°，新版 4.2°。
#
# v5.2.37 — 修「v5.2.36 之后移动/旋转完运动路径不刷新」：
#   根因两条叠加：
#   ① v5.2.35 把「确认后刷新运动路径」默认关掉、还给老用户做了迁移（存着的
#      True 被强制写成 False）—— 用户实际要的是「只刷新已有路径、别新增」，
#      不是「完全不刷新」，这条迁移把功能整个关死了；
#   ② v5.2.35 加的「清掉新增路径」兜底用了 Blender 4.x 的 API：
#      rig.pose.bones.active（5.x 已移除，改成 rig.data.bones.active）和
#      pb.bone.select（Bone.select 在 5.x 已移除，改成 PoseBone.select）。
#      整段兜底一进 try 就抛异常被吞掉，等于从来没生效过。
#   修法：偏好恢复默认开启 + 一次性反向迁移（v5.2.35 关掉的放回来，之后手动关
#   不再干预）；兜底改用 5.x API。刷新本身改成 bpy.app.timers 异步执行
#   （延后 50ms、同名骨架合并防抖）—— paths_update 是骨架级全量重算，
#   420 骨 / 219 根带路径实测 245ms，直接放进确认里会把 v5.2.36 修好的手感
#   再打回去；异步后操作零等待，路径下一帧之后自己跟上。
# v5.2.36 — 修「移动 ↔ 旋转 切换模式时卡顿」：
#   实测（420 骨 ARP 骨架 + 角色，Blender 5.2.3）：`bpy.ops.transform.rotate`
#   与 `transform.translate` 在自动关键帧打开时单次 252 / 254 ms，关掉后
#   5.7 / 5.3 ms —— 44~48 倍。切换模式、切轴、切骨、确认都会走 _flush_rotation /
#   _move_flush 里的原生落地，于是每按一次切模式键就卡 250ms 左右。
#   修法：两个原生封装（_native_rotate / _native_translate）在调用期间临时
#   `use_keyframe_insert_auto = False`，结束后恢复；同一个提交点改由
#   _insert_keys 显式补帧（单通道 0.01ms）。同时把选中态保存/恢复从「全骨架
#   420 根遍历两遍」收窄到「只处理当前选中的几根」（2.5ms → ~0）。
#
# v5.2.35 — 修「旋转/移动模态操作完，被操作骨自己长出运动路径，最后视口铺满」：
#   「旋转后刷新运动路径」改为默认关闭，并给老偏好做一次性迁移（存着的 True
#   降下来，之后手动开回不会被再关）。这个开关每次确认都全骨架重算路径，
#   还会波及本来没路径的骨 —— 用户视角就是“插件自己给骨算路径”。
#   另加兜底：就算开着，paths_update 之后也比对刷新前后有哪些骨带路径，
#   新冒出来的全部 paths_clear 掉 —— 插件在任何情况下都不新增路径。
#
# v5.2.34 — 修「FK 模式滚轮会滑到隐藏的 IK 控制器上」：
#   现象：ARP 骨架在 FK 态用滚轮切骨，列表里混着 c_thigh_ik.r / c_foot_ik.r
#   这些已经看不见的控制器，滚轮停上去什么都抓不到。
#   根因两条（rig.002 复刻现场实测）：
#   ① ARP 的「隐藏」是把 custom_shape_scale_xyz 缩到 0，bone.hide 始终是 False、
#      集合也开着 —— bone_visible 只认 hide 和集合，把它们全判成可见；
#   ② 索引骨往往不是 IK/FK 控制器本身（c_foot_roll_cursor.x / c_foot_01.r
#      这类通用控制器不带 ik/fk 标记），filter_nav_names 里 mode 解析成空，
#      整段「同模式过滤」被跳过；而且当前骨一旦被滤掉就退回全表，
#      刚好把另一套整套放回来。
#   修法：新增 widget_ghost（缩放<0.02 且带 custom_shape 即视为隐身），
#   bone_visible 认它；导航过滤**恒定**剔除隐身骨（关掉「跳过隐藏骨骼」也不放行）；
#   当前骨自身是隐身骨时不再退回全表；mode 为空时从集合里带标记的成员反查
#   这条肢体的开关，补上缺失的模式。
#
# v5.2.33 — 继续修「ARP 骨架 FK 控件不显示」：补上可见性的第二道闸。
#   现象：v5.2.32 之后极性对了、缩放也对，但腿部 FK 控制器在视口里仍然一根不见。
#   根因：ARP 的显示有两道闸 ——①控件缩放 custom_shape_scale 由开关驱动；
#   ②**骨骼集合可见性**。实测 rig.002：c_thigh_fk.r 等 10 根 FK 控制器被归到
#   `根部` 集合，而 `根部` 是关着的 → 切到 FK 时缩放回到 1，视口里还是空。
#   （同一文件的 rig / rig.001 把 FK 与 IK 放在同一个部位集合里，所以正常。）
#   修法：新增 _ensure_set_visible —— 目标那套**整组都看不见**时，打开装它们
#   最多的那个集合（跳过 mch/color/def/org/vis 机构集合），并连祖先链一起开。
#   只要有一根能看见就直接返回，对正常绑定零副作用。_col_chain_visible 提供
#   「连父集合一起算」的判定，_col_visible 只看自己。
#
# v5.2.32 — 修「Auto-Rig Pro 骨架 IK/FK 切完控件全不见（FK 控件不显示）」：
#   现象：ARP 绑定按切换键，切过去的那套控制器一个都不出现，常常两套全没了。
#   根因两条叠加（真实 ARP 骨架实测）：
#   ① 极性判反。_switch_values 只算驱动器**表达式**，忽略驱动器自带的关键帧
#      曲线 —— ARP 的约束影响两边都写 `var`，反相全靠曲线（0→1 / 1→0）。
#      于是 IK/FK 两侧都算成「1 端生效」，旧代码默认信 IK，得出 0=FK、1=IK，
#      与实际（0=IK、1=FK）相反 → 切 FK 写进去的是 IK 的值，腿一直停在对岸。
#   ② 用错了隐藏机制。ARP 3.x 从不 hide 控制器，它把离开那套的
#      custom_shape_scale 缩到 0；插件却往 bone.hide 写，而绑定永不回解。
#      结果：切过去那套被插件藏死，另一套被绑定缩成 0 —— 两套一起不可见。
#   修法：_drv_value 先算表达式再走驱动器曲线；_scale_switch_values 直接从
#   「控件缩放由谁驱动」读出极值（绑定自己的显示逻辑，最可靠）；_switch_values
#   在两侧读数相同时返回 None 而不是默认信 IK；ikfk_sync_visibility 认出缩放
#   驱动型绑定后**完全不碰 hide**，只把旧版本误藏的控制器放出来。
#
# v5.2.31 — 只做瘦身，不新增任何功能，行为与 v5.2.30 完全一致：
#   · 删掉「观察视角」整套（书签读写 / 视角后退栈 / 4 个算子 / 面板块 / 键位项）
#     —— 用户确认不常用；注册时顺带清掉老键位表里指向它们的悬空条目
#     （REMOVED_OP_IDS + remove_legacy_keymaps，避免按键跳「未知算子」）；
#   · 删掉 21 个零引用符号（含 rigUI 弹窗/面板补丁链 —— 那段早已 return False
#     停用、弹窗自动关闭偏好也随之失效，一并下掉）、7 个死常量、4 个停用偏好；
#   · _bone_get / _pb_get 两份逐字相同的实现合成 _rna_get；
#   · 文件头变更记录 v2.1→v5.2.24 移到旁挂 md，文件内只留最近六版。
#
# v5.2.30 — 修「旋转完成后运动路径不自动更新」：
#   现象：原生 R+轴转完会自动重算运动路径，插件旋转模式转完路径还是旧的。
#   实测根因（Blender 5.2.3）：运动路径缓存只在**原生变换结束**时重算；
#   插件预览是直写 matrix_basis + keyframe_insert，曲线变了但缓存不失效
#   （插帧 0/24 采样点变；update_tag / data.update_tag / pb.update_tag 全无效）。
#   而插件只有 SINGLE 单轴模式会把这段旋转交给原生 rotate；TRACKBALL /
#   DUAL / 移动全程直写矩阵 —— 所以路径不跟。
#   修法：新增 _update_motion_paths()，在**最终提交点**（左键确认 / 右键撤销回退）
#   调 pose.paths_update 重算一次 —— 不放在滚轮切骨骼、切轴这些高频路径上
#   （重骨架一次全量重算可达数百 ms~数秒，会拖慢连滚）。该操作符是骨架级、与选中无关
#   （官方描述 Recalculate paths for bones that already have them）；只在骨架确实有
#   运动路径、且自动关键帧开着时才调，无路径的骨架零开销。
# v5.2.29 — 修「手动 H 掉的骨骼，滚轮轮到它时被揭出来」：
#   v5.2.11 起集合内滚轮/↑↓ 一律完整轮（keep_all），目标骨被 H 时 select_bone 顺手
#   显示兜底 —— 于是用户手动藏掉的骨骼照样会被滚轮选中并揭出来。
#   现在 keep_all 也认偏好里的「跳过隐藏骨骼」（默认开）：隐藏骨直接跳过；
#   当前骨被跳过、一根不剩或剩不到 2 根时退回全表，不卡死。骨架脚本按 IK/FK
#   态藏的那套（v5.2.11 Anby 的大臂）想一起轮，关掉该偏好即可；select_bone 的揭出兜底
#   保留（进集合/回退路径仍可能落到隐藏骨）。
# v5.2.28 — 修「饼菜单切走再切回，滚轮跳过集合里几根骨」：
#   复现（RIG-Orphie 稳定复现）：从 Torso 饼切到别的集合再切回，滚轮（含 ↑↓）只剩 torso/hips/chest，
#   neck/head/Shoulder_L/R 永远轮不到，面板计数从 3/7 变成 3/5、3/3。根因两条叠加：
#   ① goto_collection 进集合时 remember_mode(第一根骨) 把**别的部位**的 IK/FK
#      模式写进 cn_last_mode（全局），切回原集合后一直残留；
#   ② filter_nav_names 判每根骨的模式标记时带了 bone_mode_inherited（沿父链继承）——
#      Torso 里 neck/head/Shoulder 自己名字没有 ik/fk，却从父链继承到 fk 标记，
#      撞上泄漏进来的 ik 模式就被整根剔除。
#   修法：导航过滤（keep_all 与进集合两条路）只认**骨骼自己名字里的**标记
#   （v5.2.15 注释本就写的“按名字认”，实现里多带了父链继承）。名字没标记的集合
#   成员永远完整轮（v5.2.11 的约定）；真正带 ik/fk 名字的另一套控制器仍按
#   当前模式跳过，ARP 的 v5.2.15 修复不回退。Orphie 31 集合往返 31/31 全量可轮。
# v5.2.27 — 修「IK 态点 FK 格，把藏起来的 FK 控制器揭出来」：
#   饼格指向的集合可能是当前 IK/FK 态的**反面**（RIG-Orphie 的「右臂（正向
#   动力学）」这类按 UI 层建的集合）。以前 enter_pie_collection 直接
#   show_collection 目标集合、isolate_previous_collection 把整张饼的槽位
#   集合全亮、星标预览 solo_apply 目标集合 —— 三条路都会把 rig 脚本藏掉的
#   那套控制器显示出来，选中的还是看不见的骨骼。
#   新增 _coll_mode_tag / _resolve_mode_collection：按成员骨名（无标记再看
#   集合名）判集合侧别；与当前模式相反时改落到「同部位、当前态」的那套 ——
#   只认同部位的另一侧当替身（集合名剔掉模式词后相同，或 ikfk_limbs 认出的
#   IK/FK 两端），别的部位可见不能当「目标藏着」的证据（Wise_V1 实测栽在这）。
#   显示 = 集合可见 且 骨骼没被 H（Rigify 逐根 H、容器绑定切集合显隐，两个
#   机制都认）；目标自己显示着就绝不换进整套藏起来的那套。三处入口统一走它，
#   反向（FK 态点 IK 格）同一套逻辑。566 个端到端用例实测 0 根反侧骨被揭出。
# v5.2.26 — 修「重心 c_traj 动过之后，脚/手 IK 第一下拖动整条腿弹起」：
#   c_foot_ik / c_hand_ik 挂 Child Of(c_traj)，可见矩阵由约束求出，而写
#   `pb.matrix` 的 setter 不认约束 —— 读出来再写回去会把 c_traj 的偏移再叠一遍
#   （实测 0.62m）。改用新增的 _apply_visible_translation（带 K 共轭）写 basis，
#   _move_preview 与 _move_flush 兜底两处同步。
# v5.2.25 — 修「根环（大环）旋转/移动后，其他控制器第一下拖动就跳」：
#   _bone_point_world 父级链换算少了 parent rest 还原（实测偏 1.4~1.9 单位、
#   判轴方向错 93°~180°），_screen_len 改用骨骼真实头尾投影，模态加首帧大位移守卫
# ---------------------------------------------------------------- 历史记录
#
# v5.2.24 及更早的全部变更（v2.1 → v5.2.24，约 950 行）已移到发布包同目录的
# 「驭骨BonePilot_变更记录_v2.1-v5.2.24.md」；文件内只保留最近六个版本，
# 翻代码时对照足够。功能与那份记录完全一致，未删改任何行为。
#
# 与旧版共存：本版本用独立的 operator / 面板 ID（_v45 后缀），可以和 v2.x 同时装着
# （但别同时启用，键位会抢）。
#
# 安装：Edit > Preferences > Add-ons > Install from Disk，选中本文件并启用。
#
# 快捷键（Pose Mode 下生效，键位可在偏好里换）：
#   导航      ↑ / ↓（默认）或 Ctrl+滚轮 等
#   加选      按住 Shift 再按导航键
#   M        弹出预设集合菜单（可在偏好里关掉）
#   Alt+左键拖动  弹出集合饼菜单（8 个方位 = 8 个集合，松手跳过去并滑到整组）

bl_info = {
    "name": "驭骨 · BonePilot",
    "author": "默尔佐",
    "version": (5, 2, 43),
    "blender": (5, 2, 0),
    "location": "Pose Mode > N 面板 > Nav",
    "description": "驭骨 · BonePilot：按集合导航控制器，快速旋转/移动，逐段撤销，让动画操作跟手",
    "category": "Rigging",
}

import ast
import json
import math
import re
import sys
import time

import bpy
from mathutils import Matrix, Vector, Quaternion
from bpy_extras.view3d_utils import location_3d_to_region_2d

ADDON_ID = "controller_navigator_v45"

# 默认预设集合（偏好里可以随便改）
DEFAULT_PRESETS = ("头部", "躯干", "左臂", "左手指", "右臂", "右手指", "左腿", "右腿")

# 导航快捷键方案
NAV_KEY_ITEMS = (
    ('ARROWS', "↑ / ↓ 方向键", "不碰滚轮，视图缩放完全不受影响（默认）"),
    ('CTRL_WHEEL', "Ctrl + 滚轮", "纯滚轮照常缩放，只有 Ctrl+滚轮 被导航占用"),
    ('ALT_WHEEL', "Alt + 滚轮", "纯滚轮照常缩放，只有 Alt+滚轮 被导航占用"),
    ('PAGE', "PageUp / PageDown", "翻页键上下，不碰滚轮"),
    ('BRACKET', "[ / ] 方括号", "不碰滚轮，左手也能按"),
    ('WHEEL', "滚轮（不推荐）", "Blender 的滚轮缩放是写死的，绑纯滚轮就一定抢缩放"),
    ('CUSTOM', "自定义（不自动绑键）", "自己去 Preferences > Keymap 里绑 pose.controller_nav_v45"),
)

# 本版本自己绑定的 operator（清理旧版残留时不能误删）
OWN_OP_IDS = {
    "pose.controller_nav_v45",
    "pose.cn_preset_menu_v45",
    "pose.cn_assign_preset_v45",
    "pose.cn_new_collection_v45",
    "pose.cn_set_collection_v45",
    "pose.cn_clear_collection_v45",
}

# 旧版（v2.1–v2.4）导航/菜单 operator 的 idname 前缀
LEGACY_PREFIXES = ("pose.controller_nav", "pose.cn_preset_menu")

# v5.2.31 起下掉的 operator（观察视角书签 / 视角后退）：老用户键位表里
# 可能还留着指向它们的条目，注册时一并清掉，免得按键跳「未知算子」
REMOVED_OP_IDS = (
    "pose.cn_book_goto_v45",
    "pose.cn_book_save_v45",
    "pose.cn_book_clear_v45",
    "pose.cn_view_back_v45",
)

# 安装成插件后 __name__ 就是模块名；直接跑脚本时退回 ADDON_ID
MODULE_NAME = ADDON_ID if __name__ == "__main__" else __name__

# 进入旋转/移动模态前抓的可见性快照；solo 只改显示，不改导航过滤
_NAV_VIS = None
_NAV_VIS_RIG = None


# ---------------------------------------------------------------- 偏好设置

def get_prefs():
    for key in (__name__, ADDON_ID):
        try:
            return bpy.context.preferences.addons[key].preferences
        except Exception:
            continue
    return None


def pref(name, default):
    p = get_prefs()
    return getattr(p, name, default) if p else default


class _ErrWash(Exception):
    pass


def _error_wash():
    """清掉 C 层「悄悄挂上但没抛出」的异常状态（等价 PyErr_Clear）。

    Blender 5.x 实测：个别 RNA 写入（pb.hide / keyframe_insert 这类）失败时
    会把错误状态留在解释器里却不抛 Python 异常 —— 周围的 try/except 根本
    接不住，然后下一个毫不相干的 bpy_prop_collection.get() 就报
    SystemError "<built-in method get of bpy_prop_collection ...>
    returned a result with an exception set"。主动 raise 一个自己的异常
    再接住，错误指示器就被清干净了；没有残留时这一下零开销。
    """
    # 真正的 PyErr_Clear：raise/except 不一定能清掉 C 层
    # 已经挂起但没有 Python 异常对象的错误状态。
    try:
        import ctypes
        ctypes.pythonapi.PyErr_Clear()
        return
    except Exception:
        pass
    try:
        raise _ErrWash
    except _ErrWash:
        pass


# ---- 运动路径刷新队列（v5.2.37）
# pose.paths_update() 是骨架级全量重算（420 骨 / 219 根带路径实测 245 ms），
# 放进模态收尾会把操作手感拖回去。这里排队到 timer 里做：延后 50 ms、防抖，
# 连续确认只算最后一次；操作本身零等待，路径下一帧之后自己跟上。
_MPATH_Q = {"rig": "", "timer": False, "defer": 0}

# v5.2.39：最近一次模态事件的时间戳。timer 会在模态事件之间插进来执行，而
# pose.paths_update() 是 154~161 ms 的同步全量重算（420 骨 / 219 根路径实测），
# 一旦落在拖拽中途，事件循环就冻结那 160 ms；松手后攒下的鼠标位移作为**一个**
# MOUSEMOVE 喂进来，就是用户说的「卡一下、然后姿态猛跳一段」。
# 拖拽期间一律不刷，等手停下来再做。
_GESTURE_T = 0.0
MPATH_IDLE_SECONDS = 0.30      # 手停多久之后才允许刷路径
MPATH_DEFER_LIMIT = 80         # 最多让 80 × 0.2 s = 16 s，之后照算，别饿死


def _mpath_ctx_kw(rig):
    """给 paths_update 用的 override：对象 + （有就带上）视口 window/area/region。"""
    kw = {"object": rig, "active_object": rig}
    try:
        win = bpy.context.window
    except Exception:
        win = None
    if win is not None:
        kw["window"] = win
        area = next((a for a in win.screen.areas if a.type == 'VIEW_3D'), None)
        if area is not None:
            kw["area"] = area
            reg = next((r for r in area.regions if r.type == 'WINDOW'), None)
            if reg is not None:
                kw["region"] = reg
    return kw


def _mpath_redraw():
    try:
        for w in bpy.context.window_manager.windows:
            for a in w.screen.areas:
                if a.type == 'VIEW_3D':
                    a.tag_redraw()
    except Exception:
        pass


def _mpath_timer_tick():
    # v5.2.39：拖拽期间不许插进来（见 _GESTURE_T 的注释）。用返回间隔自我
    # 重排，不重复 register —— 重复注册会让同一个回调进队列两次。
    if (time.perf_counter() - _GESTURE_T) < MPATH_IDLE_SECONDS:
        _MPATH_Q["defer"] = int(_MPATH_Q.get("defer", 0)) + 1
        if _MPATH_Q["defer"] <= MPATH_DEFER_LIMIT:
            return 0.20
    _MPATH_Q["defer"] = 0
    _MPATH_Q["timer"] = False
    name = _MPATH_Q["rig"]
    _MPATH_Q["rig"] = ""
    rig = bpy.data.objects.get(name) if name else None
    if rig is None or getattr(rig, "type", "") != 'ARMATURE':
        return None
    if getattr(rig, "mode", "") != 'POSE':
        return None                     # 已经离开姿态模式，别硬算
    kw = _mpath_ctx_kw(rig)
    try:
        before = {pb.name for pb in rig.pose.bones if pb.motion_path}
    except Exception:
        before = None
    try:
        with bpy.context.temp_override(**kw):
            bpy.ops.pose.paths_update()
    except Exception:
        _error_wash()
        return None
    finally:
        _error_wash()
    # 兜底：插件不允许「新增」运动路径（用户视角就是“给操作的骨自己长出路径”）。
    # 实测 paths_update 只重算已有的，这里只是防个别版本行为变化。
    # 注意 5.x API：pose.bones 没有 .active 了（改 data.bones.active），
    # 骨骼选中是 PoseBone.select（Bone.select 已移除）。
    if before is not None:
        try:
            new = [pb for pb in rig.pose.bones
                   if pb.motion_path and pb.name not in before]
            if new:
                saved = [(pb, pb.select) for pb in rig.pose.bones if pb.select]
                try:
                    saved_active = rig.data.bones.active
                except Exception:
                    saved_active = None
                for pb in rig.pose.bones:
                    pb.select = pb.name in {x.name for x in new}
                try:
                    rig.data.bones.active = new[0].bone
                except Exception:
                    pass
                try:
                    with bpy.context.temp_override(**kw):
                        bpy.ops.pose.paths_clear(only_selected=True)
                finally:
                    for pb, s in saved:
                        pb.select = s
                    try:
                        rig.data.bones.active = saved_active
                    except Exception:
                        pass
        except Exception:
            pass
        finally:
            _error_wash()
    _mpath_redraw()
    return None


def _mpath_queue(rig):
    """排队一次路径重算（同名骨架合并成一次）。"""
    try:
        _MPATH_Q["rig"] = rig.name
        if _MPATH_Q.get("timer"):
            return True
        try:
            if bpy.app.timers.is_registered(_mpath_timer_tick):
                _MPATH_Q["timer"] = True
                return True
        except Exception:
            pass
        _MPATH_Q["timer"] = True
        bpy.app.timers.register(_mpath_timer_tick, first_interval=0.05)
        return True
    except Exception:
        _MPATH_Q["timer"] = False
        return False


def _rna_get(rig, name, pose=False):
    """带 C 错误冲刷的骨骼查找：rig.data.bones 与 rig.pose.bones 共用一套。

    Blender 5.x 上个别 RNA 写失败会把错误状态留在解释器里却不抛异常，
    之后毫不相关的 get() 就报 SystemError；这里先冲干净，真出错再冲一次重试。
    """
    if rig is None:
        return None
    _error_wash()
    for _ in range(2):
        try:
            return (rig.pose.bones if pose else rig.data.bones).get(name)
        except SystemError:
            _error_wash()
        except Exception:
            return None
    return None


def _bone_get(rig, name):
    return _rna_get(rig, name)


def _pb_get(rig, name):
    """pose bone 查找（见 _rna_get）。"""
    return _rna_get(rig, name, pose=True)


# ---------------------------------------------------------------- 自定义键位
#
# v5.0：所有动作的键都从这里取，不再散在各处硬编码。
# 老字段（nav_keys / rot_key / collection_keys …）留着，只在首次迁移时读一次 ——
# 用户已经调好的东西不能因为升级丢光。

KB_ACTIONS = (
    ('NAV_PREV', "上一个控制器", "按住 Shift = 加选"),
    ('NAV_NEXT', "下一个控制器", "按住 Shift = 加选"),
    ('COLL_PREV', "上一个集合", ""),
    ('COLL_NEXT', "下一个集合", ""),
    ('ROT_ENTER', "进入旋转", "先选中控制器"),
    ('MOVE_ENTER', "进入移动", "先选中控制器"),
    ('MENU_COLL', "集合操作菜单", "归组 / 移出 / 新建"),
    ('PIE_1', "主饼 · 身体核心", "八个高频集合"),
    ('PIE_2', "细节饼 · 眼耳发指", ""),
    ('PIE_3', "附属饼 · 装备衣尾", ""),
    ('IKFK_TOGGLE', "IK/FK 切换", "选中 IK 或 FK 控制器后按"),
    ('IKFK_POPUP', "IK/FK 面板", "弹出每个肢体一行的切换面板"),
)

KB_LABELS = {a: lbl for a, lbl, _d in KB_ACTIONS}

MOD_FLAGS = (('ctrl', "Ctrl"), ('shift', "Shift"), ('alt', "Alt"))

_KB_PRETTY = {
    "BACK_SPACE": "BackSpace", "LEFTMOUSE": "左键", "RIGHTMOUSE": "右键",
    "MIDDLEMOUSE": "中键", "WHEELUPMOUSE": "滚轮上", "WHEELDOWNMOUSE": "滚轮下",
    "LEFT_BRACKET": "[", "RIGHT_BRACKET": "]", "UP_ARROW": "↑",
    "DOWN_ARROW": "↓", "PAGE_UP": "PageUp", "PAGE_DOWN": "PageDown",
    "SPACE": "空格", "TAB": "Tab", "RET": "回车", "ESC": "Esc",
    "BUTTON4MOUSE": "侧键4", "BUTTON5MOUSE": "侧键5",
}


class CN_KeyBind_v45(bpy.types.PropertyGroup):
    action: bpy.props.StringProperty()
    key: bpy.props.StringProperty(default="")
    ctrl: bpy.props.BoolProperty(default=False)
    shift: bpy.props.BoolProperty(default=False)
    alt: bpy.props.BoolProperty(default=False)


def _old_mod_flags(mod):
    mod = str(mod or 'NONE')
    return {'CTRL': (True, False, False),
            'SHIFT': (False, True, False),
            'ALT': (False, False, True)}.get(mod, (False, False, False))


def kb_defaults():
    """首次迁移：把老的枚举方案换算成具体键。"""
    out = {
        'MENU_COLL': (('M' if pref('use_m_key', True) else ''),
                      False, False, False),
        'PIE_1': ('LEFTMOUSE', False, False, True),
        'PIE_2': ('LEFTMOUSE', False, True, True),
        'PIE_3': ('LEFTMOUSE', True, False, True),
        'IKFK_TOGGLE': ('I', False, False, True),
        'IKFK_POPUP': ('O', False, False, True),
    }
    nav_table = {
        'ARROWS': ('UP_ARROW', 'DOWN_ARROW', False, False, False),
        'PAGE': ('PAGE_UP', 'PAGE_DOWN', False, False, False),
        'BRACKET': ('LEFT_BRACKET', 'RIGHT_BRACKET', False, False, False),
        'WHEEL': ('WHEELUPMOUSE', 'WHEELDOWNMOUSE', False, False, False),
        'ALT_WHEEL': ('WHEELUPMOUSE', 'WHEELDOWNMOUSE', False, False, True),
        'CTRL_WHEEL': ('WHEELUPMOUSE', 'WHEELDOWNMOUSE', True, False, False),
    }
    nav = nav_table.get(str(pref('nav_keys', 'ALT_WHEEL')))
    if nav:
        out['NAV_PREV'] = (nav[0], nav[2], nav[3], nav[4])
        out['NAV_NEXT'] = (nav[1], nav[2], nav[3], nav[4])
    coll_table = {
        'CTRL_ALT_WHEEL': ('WHEELUPMOUSE', 'WHEELDOWNMOUSE', True, False, True),
        'CTRL_WHEEL': ('WHEELUPMOUSE', 'WHEELDOWNMOUSE', True, False, False),
        'ALT_WHEEL': ('WHEELUPMOUSE', 'WHEELDOWNMOUSE', False, False, True),
        'PAGE': ('PAGE_UP', 'PAGE_DOWN', False, False, False),
    }
    coll = coll_table.get(str(pref('collection_keys', 'CTRL_ALT_WHEEL')))
    if coll:
        out['COLL_PREV'] = (coll[0], coll[2], coll[3], coll[4])
        out['COLL_NEXT'] = (coll[1], coll[2], coll[3], coll[4])
    out['ROT_ENTER'] = (str(pref('rot_key', 'X') or 'X') if pref('use_x_rotate', True)
                        else '', False, False, False)
    out['MOVE_ENTER'] = (str(pref('move_key', 'Z') or 'Z'),) \
        + _old_mod_flags(pref('move_key_mod', 'NONE'))
    return out


def ensure_keybinds():
    """保证每个动作都有一条记录；返回是否有改动。"""
    p = get_prefs()
    if p is None:
        return False
    try:
        kb = p.keybinds
    except Exception:
        return False
    if len(kb) == 0:
        table = kb_defaults()
        for act, _lbl, _d in KB_ACTIONS:
            d = table.get(act)
            it = kb.add()
            it.action = act
            if d:
                it.key = d[0]
                it.ctrl = bool(d[1])
                it.shift = bool(d[2])
                it.alt = bool(d[3])
        try:
            p.kb_ready = True
        except Exception:
            pass
        return True
    have = set(it.action for it in kb)
    changed = False
    for act, _lbl, _d in KB_ACTIONS:
        if act not in have:
            kb.add().action = act
            changed = True
    return changed


def kb_item(action):
    p = get_prefs()
    if p is None:
        return None
    try:
        for it in p.keybinds:
            if it.action == action:
                return it
    except Exception:
        pass
    return None


def kb_spec(action):
    """(event_type, mods) —— 没绑就返回 None。"""
    it = kb_item(action)
    if it is None or not it.key:
        return None
    kw = {}
    if it.ctrl:
        kw['ctrl'] = True
    if it.shift:
        kw['shift'] = True
    if it.alt:
        kw['alt'] = True
    return (it.key, kw)


def kb_label(action):
    it = kb_item(action)
    if it is None or not it.key:
        return "（未绑定）"
    parts = [name for flag, name in MOD_FLAGS if getattr(it, flag, False)]
    parts.append(_KB_PRETTY.get(it.key, it.key))
    return " + ".join(parts)


MOD_COMBO_ITEMS = (
    ('BARE', "裸滚轮", "什么修饰键都不按，直接滚"),
    ('SHIFT', "Shift", "按住 Shift"),
    ('ALT', "Alt", "按住 Alt"),
    ('CTRL', "Ctrl", "按住 Ctrl"),
    ('CTRL_ALT', "Ctrl + Alt", "两个一起按"),
    ('SHIFT_ALT', "Shift + Alt", "两个一起按"),
    ('CTRL_SHIFT', "Ctrl + Shift", "两个一起按"),
    ('NONE', "不绑定", "这个动作不占滚轮"),
)

MOD_COMBO_DEFAULT = {'ZOOM': 'SHIFT', 'BONE': 'ALT',
                     'COLL': 'CTRL_ALT', 'AXIS': 'BARE'}


def _combo_parts(combo):
    if str(combo or 'NONE') == 'BARE':
        return set()                       # 裸滚轮：不要求任何修饰键
    parts = set(str(combo or 'NONE').split('_'))
    return set() if 'NONE' in parts else parts


def _event_combo(event):
    c, s, a = bool(event.ctrl), bool(event.shift), bool(event.alt)
    if c and a:
        return 'CTRL_ALT'
    if c and s:
        return 'CTRL_SHIFT'
    if s and a:
        return 'SHIFT_ALT'
    if c:
        return 'CTRL'
    if s:
        return 'SHIFT'
    if a:
        return 'ALT'
    return 'NONE'


WHEEL_PREF_KEYS = {'ZOOM': 'mod_zoom_mods', 'BONE': 'mod_bone_mods',
                   'COLL': 'mod_coll_mods', 'AXIS': 'mod_axis_mods'}


def _wheel_label(action, default):
    """HUD 上显示的「Shift+滚轮」这类标签；没绑就返回空串。"""
    raw = str(pref(WHEEL_PREF_KEYS[action], default) or 'NONE')
    if raw == 'NONE':
        return ""
    if raw == 'BARE':
        return "滚轮"
    name = dict((k, n) for k, n, _d in MOD_COMBO_ITEMS).get(raw, raw)
    return name + "+滚轮"


def _modal_wheel_action(event):
    """滚轮 + 修饰键 → 'ZOOM' / 'COLL' / 'BONE' / 'AXIS' / ''。

    匹配规则：谁要求的修饰键多谁先。默认 Ctrl+Alt=切集合、Alt=切骨骼、
    Shift=缩放、裸滚轮=切轴，于是 Shift+Alt+滚轮（想加选切骨骼）不会被
    Shift 抢走；把切轴改绑到 Shift+Alt 也照样能压过单独按 Shift。
    裸滚轮只和「裸滚轮」那一项对得上，不会误吃带修饰键的组合。
    """
    combo = _combo_parts(_event_combo(event))
    cands = []
    for key, act in (('mod_coll_mods', 'COLL'),
                     ('mod_axis_mods', 'AXIS'),
                     ('mod_bone_mods', 'BONE'),
                     ('mod_zoom_mods', 'ZOOM')):
        raw = pref(key, MOD_COMBO_DEFAULT[act])
        need = _combo_parts(raw)
        if str(raw) == 'BARE':
            # 裸滚轮：只在真的没按修饰键时生效
            if not combo:
                cands.append((0, act))
            continue
        if need and need <= combo:
            cands.append((len(need), act))
    if not cands:
        return ''
    cands.sort(key=lambda c: -c[0])
    return cands[0][1]


# ---------------------------------------------------------------- 快捷键方案

def nav_key_specs():
    """返回 ((上, 修饰键), (下, 修饰键))，修饰键是给 keymap_items.new 的 kwargs。

    滚轮事件在 Blender 里是 WHEELUPMOUSE / WHEELDOWNMOUSE，value 用 PRESS。
    v5.0 起优先用用户自己绑的键；没绑才退回老的枚举方案。
    """
    up, dn = kb_spec('NAV_PREV'), kb_spec('NAV_NEXT')
    if up is not None and dn is not None:
        return up, dn
    choice = pref('nav_keys', 'ARROWS')
    specs = {
        'ARROWS': ('UP_ARROW', {}, 'DOWN_ARROW', {}),
        'CTRL_WHEEL': ('WHEELUPMOUSE', {'ctrl': True}, 'WHEELDOWNMOUSE', {'ctrl': True}),
        'ALT_WHEEL': ('WHEELUPMOUSE', {'alt': True}, 'WHEELDOWNMOUSE', {'alt': True}),
        'PAGE': ('PAGE_UP', {}, 'PAGE_DOWN', {}),
        'BRACKET': ('LEFT_BRACKET', {}, 'RIGHT_BRACKET', {}),
        'WHEEL': ('WHEELUPMOUSE', {}, 'WHEELDOWNMOUSE', {}),
    }.get(choice)
    if specs is None:  # CUSTOM 或未知值
        return None
    return (specs[0], specs[1]), (specs[2], specs[3])


# 集合循环键（和骨骼导航键分开，避免互相抢）
COLL_KEY_ITEMS = (
    ('CTRL_ALT_WHEEL', "Ctrl + Alt + 滚轮", "默认：切换集合（不与 Ctrl+滚轮 / Alt+滚轮 抢）"),
    ('CTRL_WHEEL', "Ctrl + 滚轮", "切换到下一个 / 上一个集合"),
    ('ALT_WHEEL', "Alt + 滚轮", "切换到下一个 / 上一个集合"),
    ('PAGE', "PageUp / PageDown", "切换到下一个 / 上一个集合"),
    ('NONE', "不绑定", "只用面板按钮切换集合"),
)

COLL_KEY_DEFAULT = 'CTRL_ALT_WHEEL'


def coll_key_specs():
    """集合切换键；和当前导航键冲突时返回 None（宁可不绑，也不能抢）。"""
    up, dn = kb_spec('COLL_PREV'), kb_spec('COLL_NEXT')
    if up is not None and dn is not None:
        nav = nav_key_specs()
        if nav is not None and (up, dn) == nav:
            return None
        return up, dn
    choice = pref('collection_keys', COLL_KEY_DEFAULT)
    table = {
        'CTRL_ALT_WHEEL': (('WHEELUPMOUSE', {'ctrl': True, 'alt': True}),
                           ('WHEELDOWNMOUSE', {'ctrl': True, 'alt': True})),
        'CTRL_WHEEL': (('WHEELUPMOUSE', {'ctrl': True}), ('WHEELDOWNMOUSE', {'ctrl': True})),
        'ALT_WHEEL': (('WHEELUPMOUSE', {'alt': True}), ('WHEELDOWNMOUSE', {'alt': True})),
        'PAGE': (('PAGE_UP', {}), ('PAGE_DOWN', {})),
    }
    spec = table.get(choice)
    if spec is None:
        return None
    nav = nav_key_specs()
    if nav is not None and spec == nav:
        return None
    return spec


# ---------------------------------------------------------------- 旋转 / 移动进入键

KEY_ITEMS = tuple((chr(c), chr(c), "") for c in range(ord('A'), ord('Z') + 1)) + \
    tuple(("F%d" % i, "F%d" % i, "") for i in range(1, 13))

KEY_MOD_ITEMS = (
    ('NONE', "无", "只用键本身"),
    ('CTRL', "Ctrl", "按住 Ctrl"),
    ('SHIFT', "Shift", "按住 Shift"),
    ('ALT', "Alt", "按住 Alt"),
)


def rot_key_spec():
    spec = kb_spec('ROT_ENTER')
    if spec is not None:
        return spec
    return str(pref('rot_key', 'X') or 'X'), {}


def move_key_spec():
    spec = kb_spec('MOVE_ENTER')
    if spec is not None:
        return spec
    key = str(pref('move_key', 'Z') or 'Z')
    mod = str(pref('move_key_mod', 'NONE') or 'NONE')
    kw = {}
    if mod == 'CTRL':
        kw['ctrl'] = True
    elif mod == 'SHIFT':
        kw['shift'] = True
    elif mod == 'ALT':
        kw['alt'] = True
    return key, kw


def move_key_conflict():
    """移动进入键的冲突原因；None = 可以绑。"""
    key, kw = move_key_spec()
    rkey, _ = rot_key_spec()
    if key == rkey and not kw and pref('use_x_rotate', True):
        return "和旋转键相同"
    if key == 'M' and not kw and pref('use_m_key', True):
        return "和 M 预设菜单键相同"
    nav = nav_key_specs()
    if nav is not None:
        for k, mods in nav:
            if k == key and mods == kw:
                return "和导航键相同"
    return None


def key_label(key, kw):
    parts = []
    for flag, name in (('ctrl', "Ctrl"), ('shift', "Shift"), ('alt', "Alt")):
        if kw.get(flag):
            parts.append(name)
    parts.append(key if len(key) <= 2 else key.title())
    return " + ".join(parts)


def key_match(event, key, kw):
    """事件是不是「这个键 + 这组修饰键」。

    修饰键必须完全一致：配的是裸 Z，就只认不按 Ctrl/Shift/Alt 的 Z，
    否则 Shift+Z 这类组合会被误当成切模式。
    """
    if getattr(event, "type", None) != key:
        return False
    for flag in ('ctrl', 'shift', 'alt'):
        if bool(kw.get(flag)) != bool(getattr(event, flag, False)):
            return False
    return True


def rot_key_label():
    return key_label(*rot_key_spec())


def move_key_label():
    return key_label(*move_key_spec())


def move_key_ok():
    return move_key_conflict() is None


# ---------------------------------------------------------------- 选中

def select_bone(rig, bone_name, extend=False):
    """选中骨骼。extend=False 单选，extend=True 加选。

    v5.2.9：目标骨被单独 H 隐藏时顺手显示出来 —— 滚轮兜底会走到隐藏骨
    （见 filter_nav_names 的回退），不显示的话视图飞过去却空无一物，
    用户只会更懵。集合可见性不动，只动骨骼自己的 hide。
    """
    pb = _pb_get(rig, bone_name)
    if pb is None:
        return False
    try:
        if pb.bone.hide or pb.hide:
            pb.bone.hide = False
            pb.hide = False
    except Exception:
        pass
    if not extend:
        for p in rig.pose.bones:
            p.select = False
    pb.select = True
    rig.data.bones.active = pb.bone
    return True


# ---------------------------------------------------------------- 鼠标跟随

def window_point(region, p, dx=0, dy=0):
    """把「视口内 2D 点」换算成「窗口坐标」（Blender 的 cursor_warp 吃这个）。

    两个坑：
    1. Region.x / Region.y 本身就是相对窗口的坐标（不是相对 Area），
       所以只加 region 偏移；再加 area.x/y 会偏一个面板宽度。
    2. Blender 的窗口坐标是左下角原点、y 向上，和 location_3d_to_region_2d
       返回的 y 方向一致，所以不需要翻转。

    dx / dy 是落点偏移（像素，X 右为正、Y 上为正），
    用来把指针从骨骼头部（Gizmo 正中心）挪到旋转环或位移轴上。
    """
    return (int(round(region.x + p.x + dx)),
            int(round(region.y + p.y + dy)))


def bone_window_point(context, rig, bone_name):
    """骨骼当前（姿态后）头部在「窗口坐标」里的落点（含 warp_dx/dy 偏移）。

    任何条件不满足都返回 None：后台模式、鼠标不在 3D 视口、骨骼在画面外、
    拿不到 region_3d —— 都不会抛异常，也不会乱甩鼠标。
    """
    if bpy.app.background:
        return None
    pb = rig.pose.bones.get(bone_name) if rig is not None else None
    if pb is None:
        return None

    area = getattr(context, "area", None)
    region = getattr(context, "region", None)
    win = getattr(context, "window", None)
    space = getattr(context, "space_data", None)
    if area is None or region is None or win is None or space is None:
        return None
    if area.type != 'VIEW_3D' or region.type != 'WINDOW':
        return None

    rv3d = getattr(space, "region_3d", None)
    if rv3d is None:
        return None

    co = rig.matrix_world @ pb.head
    p = location_3d_to_region_2d(region, rv3d, co)
    if p is None:
        return None
    # 骨骼本身在画面外就不跟（偏移前判断，避免偏移把边缘骨骼算成可见）
    if not (0 <= p.x <= region.width and 0 <= p.y <= region.height):
        return None
    if getattr(win, "cursor_warp", None) is None:
        return None

    x, y = window_point(region, p, pref('warp_dx', 0), pref('warp_dy', 0))
    # 偏移后仍然夹在视口内，别把指针送进别的面板
    x = min(max(x, region.x), region.x + region.width - 1)
    y = min(max(y, region.y), region.y + region.height - 1)
    return int(x), int(y)


def warp_cursor_to_bone(context, rig, bone_name):
    """把鼠标移到该骨骼当前（姿态后）头部在视口里的位置（含落点偏移）。

    任何条件不满足都安静返回 False。
    """
    pt = bone_window_point(context, rig, bone_name)
    if pt is None:
        return False
    win = getattr(context, "window", None)
    try:
        win.cursor_warp(pt[0], pt[1])
    except Exception:
        return False
    return True


def follow_cursor_to(context, rig, bone_name):
    """带偏好开关的跟随入口。"""
    if not pref('follow_mouse', True):
        return False
    return warp_cursor_to_bone(context, rig, bone_name)


# ---------------------------------------------------------------- 视口跟随

_VIEW_TWEEN = {}
VIEW_TWEEN_SECONDS = 0.28      # 单点平移：骨骼跑出视口时"追上去"
VIEW_FRAME_SECONDS = 0.34      # 整组框选：饼菜单跳转时滑过去


def _smoothstep(t):
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def point_visible_in_region(region, p, margin=0.04):
    if region is None or p is None:
        return False
    mw = region.width * margin
    mh = region.height * margin
    return (mw <= p.x <= region.width - mw) and (mh <= p.y <= region.height - mh)


def view_tween_busy():
    """视口正在缓动中。别在动画中途再塞一个新动画 —— 两个定时器抢同一个
    view_location，表现就是"抖一下然后瞬移"。"""
    return bool(_VIEW_TWEEN)


def _tween_stop(unregister=True):
    if unregister:
        handle = _VIEW_TWEEN.get("handle")
        if handle is not None:
            try:
                bpy.app.timers.unregister(handle)
            except Exception:
                pass
    _VIEW_TWEEN.clear()


def _tween_warp(st):
    """动画结束后把鼠标归位到目标骨骼上（跟随开着才做）。"""
    if not pref('follow_mouse', True):
        return
    rv3d = st.get("rv3d")
    region = st.get("region")
    win = st.get("window")
    co = st.get("warp")
    if rv3d is None or region is None or win is None or co is None:
        return
    try:
        p = location_3d_to_region_2d(region, rv3d, co)
        if p is None:
            return
        warp = getattr(win, "cursor_warp", None)
        if warp is None:
            return
        x, y = window_point(region, p)
        warp(int(x), int(y))
    except Exception:
        pass


def _view_tween_step():
    """定时器：把视角缓动到目标，结束时把鼠标归位。

    两种动画共用一套：
      · kind="pan"    只动 view_location（骨骼跑出视口，追上去）
      · kind="frame"  同时动 view_location / view_distance / view_rotation
                      （饼菜单跳转，框住整组控制器）
    """
    st = _VIEW_TWEEN
    if not st:
        return None
    rv3d = st.get("rv3d")
    if rv3d is None:
        _tween_stop(unregister=False)
        return None

    t = (time.time() - st.get("t0", 0.0)) / max(st.get("duration", VIEW_TWEEN_SECONDS), 1e-4)
    k = _smoothstep(t)
    try:
        if st.get("kind") == "frame":
            rv3d.view_location = st["start"].lerp(st["end_loc"], k)
            rv3d.view_distance = st["start_dist"] + (st["end_dist"] - st["start_dist"]) * k
            q0, q1 = st.get("start_quat"), st.get("end_quat")
            if q0 is not None and q1 is not None:
                rv3d.view_rotation = q0.slerp(q1, k)
        else:
            rv3d.view_location = st["start"].lerp(st["end"], k)
    except Exception:
        _tween_stop(unregister=False)
        return None
    try:
        area = st.get("area")
        if area is not None:
            area.tag_redraw()
    except Exception:
        pass

    if t >= 1.0:
        _tween_warp(st)
        _tween_stop(unregister=False)
        return None
    return 0.0


def smooth_view_to(context, rig, bone_name):
    """骨骼不在视口内时，用缓动把视口平移过去；返回是否启动了动画。"""
    if bpy.app.background or not pref('auto_frame', True):
        return False
    if view_tween_busy():           # 已经在滑动，别抢
        return False
    area = getattr(context, "area", None)
    region = getattr(context, "region", None)
    win = getattr(context, "window", None)
    space = getattr(context, "space_data", None)
    if area is None or region is None or win is None or space is None:
        return False
    if area.type != 'VIEW_3D' or region.type != 'WINDOW':
        return False
    rv3d = getattr(space, "region_3d", None)
    pb = rig.pose.bones.get(bone_name) if rig is not None else None
    if rv3d is None or pb is None:
        return False

    co = rig.matrix_world @ pb.head
    p = location_3d_to_region_2d(region, rv3d, co)
    if point_visible_in_region(region, p):
        return False

    _tween_stop()
    _VIEW_TWEEN.update({
        "kind": "pan",
        "rv3d": rv3d,
        "area": area,
        "region": region,
        "window": win,
        "start": rv3d.view_location.copy(),
        "end": co.copy(),
        "warp": co.copy(),
        "t0": time.time(),
        "duration": VIEW_TWEEN_SECONDS,
    })
    try:
        _VIEW_TWEEN["handle"] = bpy.app.timers.register(_view_tween_step, first_interval=0.0)
    except Exception:
        _tween_stop(unregister=False)
        return False
    return True


def native_view_selected(context):
    """跑一次原生 view_selected（Home / 小键盘 . 那套框选），拿不到上下文就 False。"""
    area = getattr(context, "area", None)
    if area is None or area.type != 'VIEW_3D':
        return False
    win = getattr(context, "window", None)
    reg = next((r for r in area.regions if r.type == 'WINDOW'), None)
    if win is None or reg is None:
        return False
    try:
        with context.temp_override(window=win, area=area, region=reg,
                                   space_data=area.spaces.active):
            bpy.ops.view3d.view_selected()
        return True
    except Exception:
        return False


# ---------------------------------------------------------------- 取景余量
# 原生 view_selected 是**零留白**的：选中骨骼的包围盒直接顶到视口边缘。
# 而控制器的可见外形（圆环、方块、箭头）比骨骼 head→tail 那根线大一圈，
# 于是框完就是贴着屏幕边、甚至被裁掉一截；集合越小越明显（单根骨骼会贴脸）。
# 这里的做法不改原生取景的朝向和中心 —— 只在它算出的**距离**上乘一个用户
# 可调的倍率，再吃一次最小视距兜底。
FRAME_MARGIN_DEFAULT = 1.5
FRAME_MIN_DIST_DEFAULT = 0.8


def frame_margin_factor():
    """用户设置的取景余量倍率（>= 1.0，读不到就用默认值）。"""
    try:
        m = float(pref('frame_margin', FRAME_MARGIN_DEFAULT))
    except Exception:
        m = FRAME_MARGIN_DEFAULT
    if m < 1.0:
        m = 1.0
    return m


def frame_min_distance():
    """取景距离下限；0 表示不限制。"""
    try:
        d = float(pref('frame_min_dist', FRAME_MIN_DIST_DEFAULT))
    except Exception:
        d = FRAME_MIN_DIST_DEFAULT
    return d if d > 0.0 else 0.0


def frame_distance(dist):
    """原生框选距离 → 加了余量的距离。"""
    d = float(dist) * frame_margin_factor()
    floor = frame_min_distance()
    if floor > 0.0 and d < floor:
        d = floor
    return d


def native_frame_selected(context):
    """兜底路径：原生框选 + 和缓动路径**同一套余量**。

    缓动没起来（上下文不对、定时器注册失败、或用户关掉了自动取景）时走这里。
    两条路的最终距离必须一致 —— 否则兜底那一下会比缓动近 1/余量 倍，
    观感就是"画面突然贴脸放大"。原生自带的 200ms 平滑动画也要按住，
    不然它会在我们加完余量之后把视距又拉回去。
    """
    view_prefs = getattr(getattr(bpy.context, "preferences", None), "view", None)
    old_smooth = None
    if view_prefs is not None:
        try:
            old_smooth = view_prefs.smooth_view
            view_prefs.smooth_view = 0
        except Exception:
            old_smooth = None
    try:
        if not native_view_selected(context):
            return False
        space = getattr(context, "space_data", None)
        rv3d = getattr(space, "region_3d", None) if space is not None else None
        if rv3d is None:
            return False
        rv3d.view_distance = frame_distance(rv3d.view_distance)
    except Exception:
        return False
    finally:
        if old_smooth is not None:
            try:
                view_prefs.smooth_view = old_smooth
            except Exception:
                pass
    return True


def frame_view_animated(context, rig, warp_bone=None):
    """框住当前选中的整组控制器 —— 缓动滑过去，不是瞬移。

    框选的**精度**完全交给原生 view_selected（和 Home / 小键盘 . 一模一样，
    不用自己算包围盒、不用自己算 FOV），**动效**自己做：

        记下当前视角 → 跑一次原生框选读出「目标视角」→ 立刻把视角还原
        → 0.34 秒里把 view_location / view_distance / view_rotation 缓到目标

    原生那一下自带的瞬时动画用 preferences.view.smooth_view=0 临时按住，
    否则它会和我们的定时器抢同一个 view_location（表现是先瞬移再往回滑）。
    返回是否真的启动了动画。
    """
    if bpy.app.background or not pref('auto_frame', True):
        return False
    area = getattr(context, "area", None)
    region = getattr(context, "region", None)
    win = getattr(context, "window", None)
    space = getattr(context, "space_data", None)
    if area is None or region is None or win is None or space is None:
        return False
    if area.type != 'VIEW_3D' or region.type != 'WINDOW':
        return False
    rv3d = getattr(space, "region_3d", None)
    if rv3d is None:
        return False

    start_loc = rv3d.view_location.copy()
    start_dist = rv3d.view_distance
    start_quat = rv3d.view_rotation.copy()

    view_prefs = getattr(getattr(bpy.context, "preferences", None), "view", None)
    old_smooth = None
    if view_prefs is not None:
        try:
            old_smooth = view_prefs.smooth_view
            view_prefs.smooth_view = 0
        except Exception:
            old_smooth = None

    try:
        if not native_view_selected(context):
            return False
        end_loc = rv3d.view_location.copy()
        end_dist = rv3d.view_distance
        end_quat = rv3d.view_rotation.copy()
    finally:
        # 不管成没成，先把视角恢复成"跳之前"的样子，再由缓动接管
        try:
            rv3d.view_location = start_loc
            rv3d.view_distance = start_dist
            rv3d.view_rotation = start_quat
        except Exception:
            pass
        if old_smooth is not None:
            try:
                view_prefs.smooth_view = old_smooth
            except Exception:
                pass

    # 原生取景是零留白的：这里只在它算出来的**距离**上加余量，朝向与中心不动。
    # 透视和正交视口都是用 view_distance 控制缩放的（ortho_scale 只属于相机），
    # 所以这里一套逻辑就够，不用分情况。
    end_dist = frame_distance(end_dist)

    # 已经停在**带余量**的目标取景上 —— 这是"第二次进同一个集合"。
    # 必须返回 True：返回 False 会让调用者走 native_view_selected 兜底，
    # 而原生取景是零留白的，视距会从 native×余量 一下子跳到 native，
    # 表现就是"连着切同一个集合，画面瞬间贴脸放大"。
    #
    # 阈值不能取 1e-6：原生框选每次都重新算包围盒中心，和上一帧的视角位置
    # 有 1e-4 级的浮点差，位置比较永远过不去，等于这道闸白设。按视距的
    # 千分位给容差 —— 视觉上完全分辨不出来的量级。
    pos_eps = max(1e-3, abs(float(start_dist)) * 5e-3)
    dist_eps = max(1e-4, abs(float(start_dist)) * 1e-2)
    if ((end_loc - start_loc).length < pos_eps
            and abs(end_dist - start_dist) < dist_eps):
        return True

    pb = rig.pose.bones.get(warp_bone) if (rig is not None and warp_bone) else None
    warp_co = (rig.matrix_world @ pb.head) if pb is not None else end_loc.copy()

    _tween_stop()
    _VIEW_TWEEN.update({
        "kind": "frame",
        "rv3d": rv3d,
        "area": area,
        "region": region,
        "window": win,
        "start": start_loc,
        "end_loc": end_loc,
        "start_dist": start_dist,
        "end_dist": end_dist,
        "start_quat": start_quat,
        "end_quat": end_quat,
        "warp": warp_co.copy(),
        "t0": time.time(),
        "duration": VIEW_FRAME_SECONDS,
    })
    try:
        _VIEW_TWEEN["handle"] = bpy.app.timers.register(_view_tween_step, first_interval=0.0)
    except Exception as _e:
        _tween_stop(unregister=False)
        return False
    return True


def selected_bones(rig):
    """当前选中的姿态骨骼（按 pose.bones 顺序，稳定）。"""
    if rig is None or rig.pose is None:
        return []
    return [pb for pb in rig.pose.bones if pb.select]


def selected_names(rig):
    return [pb.name for pb in rig.pose.bones if pb.select]


def edit_ok(rig):
    """骨架可编辑（不是库链接进来的只读数据）。"""
    if rig is None or rig.type != 'ARMATURE':
        return False
    if rig.library is not None or rig.data.library is not None:
        return False
    return True


def pose_poll(context):
    ob = context.object
    return (context.mode == 'POSE'
            and ob is not None
            and ob.type == 'ARMATURE'
            and edit_ok(ob))


# ---------------------------------------------------------------- 集合判定

def col_visible(col):
    """集合是否可见（优先用 5.x 的 is_visible_effectively，能考虑父级折叠）。"""
    eff = getattr(col, "is_visible_effectively", None)
    if eff is not None:
        return eff
    return col.is_visible


def nav_collection(rig, bone):
    """决定用哪个集合导航。优先级：

    1. 手动/跳转进来的集合（rig.data["cn_collection"]，随文件保存）
    2. 上一次导航用的集合（rig.data["cn_scope"]）—— 只要当前骨骼还在里面就继续用它
    3. 偏好里填的名字关键字（collection_keyword）
    4. **骨骼数最多**的可见集合

    第 2、4 条是 v4.9.88 改的。原来遇到一根骨骼同时属于好几个集合时
    「取骨骼数最少的那个」：用户把 13 根头发放进「前发」，其中 8 根又被别的
    骨架工具顺手加进了「Hair」、5 根加进了别的集合，于是滚轮有时在 5 根里转、
    有时在 8 根里转，永远凑不齐 13 根。现在按「进入的集合 → 上次的集合 →
    最大的集合」走，同一个集合里的骨骼永远在同一份列表里。
    """
    if bone is None:
        return None
    cols = [c for c in bone.collections if col_visible(c)]
    if not cols:
        return None

    for key in ("cn_collection", "cn_scope"):
        want = rig.data.get(key)
        if want:
            for c in cols:
                if c.name == want:
                    return c

    kw = (pref('collection_keyword', '') or '').strip()
    if kw:
        for c in cols:
            if kw.lower() in c.name.lower():
                return c

    cols.sort(key=lambda c: len(c.bones), reverse=True)
    return cols[0]


def remember_scope(rig, name):
    """记住这次导航用的集合（只在操作符/模态里调，别在 draw 里调）。"""
    if rig is None or not name:
        return
    try:
        if rig.data.get("cn_scope") != name:
            rig.data["cn_scope"] = name
    except Exception:
        pass
    _error_wash()   # v5.2.16：自定义属性写入也可能静默挂异常


# ---------------------------------------------------------------- 空间排序

class CN_RigSpatialSettings_v45(bpy.types.PropertyGroup):
    refine_mode: bpy.props.BoolProperty(
        name="细化阶段",
        default=False,
        description="开启后旋转默认单轴；手指仍用自己的手指默认轴",
    )
    refine_axis: bpy.props.EnumProperty(
        name="细化轴",
        items=(('X', "X", ""), ('Y', "Y", ""), ('Z', "Z", "")),
        default='Y',
    )


def _rig_spatial_settings(rig):
    try:
        return getattr(rig, "cn_spatial", None)
    except Exception:
        return None


SORT_MODE_ITEMS = (
    ('ASSIGN', "集合顺序", "完全按 assign 进集合的先后"),
    ('SPATIAL', "空间排序", "按世界 Z 从上到下"),
    ('HIERARCHY', "父子层级", "根 → 子 → 孙，同一父级按集合顺序"),
    ('FINGER', "手指排序", "拇指 → 小指，编号自然排序"),
)


def collection_sort_mode(rig, col):
    if rig is None or col is None:
        return 'ASSIGN'
    try:
        modes = rig.data.get("cn_sort_modes", {}) or {}
        mode = str(modes.get(col.name, 'ASSIGN'))
        return 'ASSIGN' if mode == 'INHERIT' else mode
    except Exception:
        return 'ASSIGN'


def set_collection_sort_mode(rig, col, mode):
    if rig is None or col is None:
        return False
    try:
        modes = dict(rig.data.get("cn_sort_modes", {}) or {})
        modes[col.name] = mode
        rig.data["cn_sort_modes"] = modes
        return True
    except Exception:
        return False


def collection_sort_label(rig, col):
    mode = collection_sort_mode(rig, col)
    for key, label, _ in SORT_MODE_ITEMS:
        if key == mode:
            return label
    return "集合顺序"


def hierarchy_names(rig, col):
    """集合内的父子层级顺序：根 → 子 → 孙。

    集合里可能只放了每根头发的根控制器，子关节在别的集合里；
    父子层级模式会继续往子级走，直到没有子级（跳过 ORG/MCH/DEF 等内部骨）。
    """
    if col is None:
        return []
    base = [b.name for b in col.bones if rig.pose.bones.get(b.name) is not None]
    base_set = set(base)
    roots = []
    for n in base:
        b = rig.data.bones.get(n)
        parent = b.parent.name if b is not None and b.parent is not None else None
        if not parent or parent not in base_set:
            roots.append(n)

    internal = re.compile(r"^(ORG|MCH|DEF|VIS|WGT|IK|FK)[-_]", re.I)

    def child_names(name):
        b = rig.data.bones.get(name)
        if b is None:
            return []
        return [c.name for c in b.children]

    out = []
    seen = set()

    def walk(n):
        if n in seen:
            return
        seen.add(n)
        out.append(n)
        for c in child_names(n):
            if c in base_set or not internal.match(c):
                walk(c)

    for n in roots:
        walk(n)
    for n in base:       # 环/异常数据兜底
        walk(n)
    return out


def bone_key(rig, bone_name):
    """空间排序键：(head 世界 Z, tail 世界 Z)。

    用 rest pose 坐标，排序不随动画姿态变化。

    为什么要加 tail：控制器常出现 head 完全重合的情况，最典型的是
    c_root.x（腰部，朝下）与 c_spine_01.x（朝上）——两者 head 都在 z=1.125，
    只比 head 无法分出上下。此时 tail 的 Z 恰好相反（1.018 vs 1.232），
    于是朝上的骨骼自然排在上面、朝下的排在下面，符合视觉直觉。
    """
    b = rig.data.bones.get(bone_name)
    if b is None:
        return (0.0, 0.0)
    h = (rig.matrix_world @ b.head_local).z
    t = (rig.matrix_world @ b.tail_local).z
    return (round(h, 3), round(t, 3))


_FINGER_WORDS = (
    ("thumb", "thumb"), ("拇指", "thumb"),
    ("index", "index"), ("食指", "index"),
    ("middle", "middle"), ("中指", "middle"),
    ("ring", "ring"), ("无名", "ring"),
    ("pinky", "pinky"), ("little", "pinky"),
)
_FINGER_RANK = {"thumb": 0, "index": 1, "middle": 2, "ring": 3, "pinky": 4}

_NUM_RE = re.compile(r"(\d+)")


def finger_rank(name):
    """拇指 0 → 食指 1 → 中指 2 → 无名 3 → 小指 4；不是手指返回 99。"""
    low = (name or "").lower()
    for token, key in _FINGER_WORDS:
        if token in low:
            return _FINGER_RANK[key]
    return 99


def natural_key(s):
    """自然排序键：f_index_2 排在 f_index_10 前面。"""
    parts = _NUM_RE.split((s or "").lower())
    return [(0, int(p)) if p.isdigit() else (1, p) for p in parts]


def collection_bone_names(rig, col):
    """集合内的骨骼名（手指集合按拇指→小指，其余按空间排序或 assign 顺序）。"""
    if col is None:
        return []
    names = [b.name for b in col.bones if _pb_get(rig, b.name) is not None]

    mode = collection_sort_mode(rig, col)
    if mode == 'HIERARCHY':
        return hierarchy_names(rig, col)
    if mode == 'ASSIGN':
        # 「集合顺序」= 面板里排过的自定义顺序优先，没排过就是 assign 原序
        return ordered_bone_names(rig, col.name, names)
    if mode == 'FINGER':
        if any(finger_rank(n) < 99 for n in names):
            names.sort(key=lambda n: (finger_rank(n), natural_key(n)))
        return names
    if mode == 'SPATIAL':
        names.sort(key=lambda n: bone_key(rig, n), reverse=True)
        return names
    return names


# ---------------------------------------------------------------- 集合内骨骼顺序
#
# v5.2.4：集合顺序不再只能听 assign 的。面板里能一根一根上下挪，也能批量写：
#
#   · 链式收录   —— 选链头，沿父子一路收到底（头发那种一连串控制器最省事）
#   · 框选排序   —— 乱序框一堆，按「父先于子 + 编号自然序」自动排
#   · 按编号展开 —— hair_fk.001 → 同前后缀的 001..00N 一次排好
#   · 点选排队   —— 一根一根点，按点选的先后写（Blender 自己不给选中顺序）
#
# 顺序存在 rig.data 的 `cn_boneord_<集合名>`（字符串数组），随 .blend 保存。
# 没进表里的骨骼按原来的相对次序跟在后面 —— 加骨、删骨都不会把顺序弄乱。

_BONE_ORDER_PREFIX = "cn_boneord_"
_ORDER_INTERNAL_RE = re.compile(r"^(ORG|MCH|DEF|VIS|WGT|IK|FK)[-_]", re.I)


def bone_order_key(col_name):
    return _BONE_ORDER_PREFIX + (col_name or "")


def bone_order(rig, col_name):
    """这个集合自定义过的骨骼顺序；空列表 = 没排过。"""
    if rig is None or not col_name:
        return []
    try:
        raw = rig.data.get(bone_order_key(col_name), None)
    except Exception:
        return []
    if not raw:
        return []
    try:
        return [str(n) for n in raw]
    except Exception:
        return []


def set_bone_order(rig, col_name, names):
    """写自定义顺序；传空列表就删掉，回到 assign 原序。"""
    if rig is None or not col_name:
        return False
    key = bone_order_key(col_name)
    try:
        if names:
            rig.data[key] = [str(n) for n in names]
        elif key in rig.data:
            del rig.data[key]
        return True
    except Exception:
        return False


def ordered_bone_names(rig, col_name, names):
    """按自定义顺序重排；不在表里的保持原相对次序排在后面（sorted 稳定）。"""
    names = list(names)
    order = bone_order(rig, col_name)
    if not order:
        return names
    rank = {}
    for i, n in enumerate(order):
        rank.setdefault(n, i)
    big = len(order)
    return sorted(names, key=lambda n: rank.get(n, big))


def bone_order_is_custom(rig, col_name):
    return bool(bone_order(rig, col_name))


def collection_members(rig, col):
    """集合里的骨骼名（去掉孤立的、骨架里已经不存在的）。"""
    if col is None:
        return []
    return [b.name for b in col.bones if rig.pose.bones.get(b.name) is not None]


def order_chain_names(rig, col, roots):
    """从链头沿父子关系一路收：表里只放集合成员，但中间跳过非成员骨继续走。

    这样「一根头发分在两个集合里」也能接上；同层多子级时按编号自然序走，
    所以 hair_fk.002 不会排到 hair_fk.010 后面去。
    """
    members = set(collection_members(rig, col))
    out, seen = [], set()

    def walk(name, depth):
        if name in seen or len(out) >= 800 or depth > 96:
            return
        seen.add(name)
        if name in members:
            out.append(name)
        b = rig.data.bones.get(name)
        if b is None:
            return
        kids = [c.name for c in b.children if c.name not in seen]
        kids.sort(key=natural_key)
        for k in kids:
            if _ORDER_INTERNAL_RE.match(k):
                continue
            walk(k, depth + 1)

    for r in roots:
        walk(r, 0)
    return out


def order_topology_names(rig, col, names):
    """一堆骨骼按「父先于子」排：框选也能排出 1→N 的顺序。"""
    selset = set(n for n in names if n)
    roots = []
    for n in selset:
        b = rig.data.bones.get(n)
        if b is None:
            continue
        par = b.parent.name if b.parent is not None else None
        if par is None or par not in selset:
            roots.append(n)
    roots.sort(key=natural_key)
    out, seen = [], set()

    def walk(name):
        if name in seen or len(out) >= 800:
            return
        seen.add(name)
        out.append(name)
        b = rig.data.bones.get(name)
        if b is None:
            return
        for c in sorted([c.name for c in b.children
                         if c.name in selset and c.name not in seen], key=natural_key):
            walk(c)

    for r in roots:
        walk(r)
    for n in sorted(selset - seen, key=natural_key):      # 环/孤点兜底
        out.append(n)
        seen.add(n)
    return out


_ORDER_NUM_RE = re.compile(r"^(.*?)(\d+)(\D*)$")


def order_series_names(rig, col, seed):
    """hair_fk.001 → 同前后缀的整条编号序列，按数字大小排。"""
    m = _ORDER_NUM_RE.match(seed or "")
    if not m:
        return []
    head, tail = m.group(1), m.group(3)
    found = []
    for n in collection_members(rig, col):
        mm = _ORDER_NUM_RE.match(n)
        if mm and mm.group(1) == head and mm.group(3) == tail:
            found.append((int(mm.group(2)), n))
    found.sort()
    return [n for _i, n in found]


def order_merge(rig, col, head_names):
    """把新排的一段放到最前，其余成员按当前顺序跟在后面（不丢骨）。"""
    head = [n for n in dict.fromkeys(head_names or []) if n]
    rest = [n for n in collection_bone_names(rig, col) if n not in set(head)]
    return head + rest


def write_bone_order(rig, col, names, report=None):
    """落盘顺序 + 顺手把排序模式切到「集合顺序」（不然会被空间排序盖掉）。"""
    if set_bone_order(rig, col.name, names):
        if collection_sort_mode(rig, col) != 'ASSIGN':
            set_collection_sort_mode(rig, col, 'ASSIGN')
        return True
    if report is not None:
        report({'WARNING'}, "这根骨架是只读的，改不了顺序")
    return False


# ---------------------------------------------------------------- 可见 / IK-FK 过滤

_MODE_IK = re.compile(r"(^|[_.\-\s])ik($|[_.\-\s])")
_MODE_FK = re.compile(r"(^|[_.\-\s])fk($|[_.\-\s])")


def bone_mode(name):
    """骨骼属于 IK 还是 FK：按名字里的 ik / fk 标记判断。

    · c_arm_ik.l / c_hand_fk.l → ik / fk
    · c_arms_pole.l（极向控制器）算 ik，避免 IK 模式下选不到它
    · 没有标记（如 c_shoulder.l）返回空字符串，表示「不区分模式」
    """
    if not name:
        return ""
    low = name.lower()
    if "pole" in low:
        return "ik"
    if _MODE_IK.search(low):
        return "ik"
    if _MODE_FK.search(low):
        return "fk"
    return ""


def widget_ghost(pb):
    """控制器被绑定用「缩放」关掉了形状 —— 视口里画不出来，但不等于 bone.hide。

    ARP 3.x 的做法：离开那一套的控制器 `custom_shape_scale_xyz` 归 0，
    `bone.hide` 一直是 False、集合也开着。只看 hide 的话，导航会把它当成
    可见骨骼，滚轮就停在看不见的控制器上（用户 2026-11-02 报的就是这个）。
    `select_bone` 那边也不能靠「清 hide」把它揭出来 —— 那是显隐的另一道闸。
    """
    try:
        if pb.custom_shape is None:
            return False
        s = pb.custom_shape_scale_xyz
        return max(abs(s[0]), abs(s[1]), abs(s[2])) < 0.02
    except Exception:
        return False


def bone_visible(pb):
    """骨骼当前是否可见：自身没隐藏、所在集合开着、控件形状没被缩到 0。

    模态期间优先用进入前抓的快照：solo 会把其他骨骼临时隐藏，
    不能因此把它们从导航列表里删掉。
    """
    if pb is None:
        return False
    if _NAV_VIS is not None and _NAV_VIS_RIG is not None:
        try:
            idd = pb.id_data
            arm = getattr(idd, "data", idd)
            if arm == _NAV_VIS_RIG and pb.name in _NAV_VIS:
                return bool(_NAV_VIS[pb.name])
        except Exception:
            pass
    if pb.bone.hide or pb.hide:
        return False
    cols = list(pb.bone.collections)
    if cols and not any(col_visible(c) for c in cols):
        return False
    if widget_ghost(pb):
        return False
    return True


# ---------------------------------------------------------------- 从 rig 读 IK/FK 状态

_SIDE_RE = re.compile(r"[._]([lr])$", re.IGNORECASE)


def bone_side(name):
    """c_shoulder.l / shoulder.L → 'l'；c_hand_fk.r / hand_fk.R → 'r'。"""
    m = _SIDE_RE.search(name or "")
    return m.group(1).lower() if m else ""


def collection_limb(col_name):
    low = (col_name or "").lower()
    if any(k in low for k in ("腿", "脚", "leg", "foot")):
        return "leg"
    if any(k in low for k in ("臂", "手", "arm", "hand")):
        return "arm"
    return ""


_LIMB_WORDS = {
    "arm": ("arm", "hand", "wrist", "shoulder", "elbow", "forearm", "clavicle"),
    "leg": ("leg", "foot", "thigh", "shin", "knee", "ankle", "toe", "calf"),
}


def bone_limb(name):
    """从骨骼名猜部位：Shoulder_ik_target_L → arm，Thigh_FK_L → leg。"""
    low = (name or "").lower()
    for limb in ("leg", "arm"):
        if any(w in low for w in _LIMB_WORDS[limb]):
            return limb
    return ""


def _switch_candidates(side):
    """按优先级列出可能存着 IK/FK 开关的骨骼名。

    · Rigify：upper_arm_parent.L / thigh_parent.L（IK_FK）
    · Auto-Rig Pro：c_ikfk_arm.l / c_hand_ik.l（ik_fk_switch）
    大小写两种写法都试（Rigify 用 .L，ARPro 用 .l）。
    """
    up, low = side.upper(), side.lower()
    arm = [
        "upper_arm_parent.%s" % up, "upper_arm_parent.%s" % low,
        "hand_ik.%s" % up, "hand_ik.%s" % low,
        "c_ikfk_arm.%s" % low, "c_hand_ik.%s" % low,
    ]
    leg = [
        "thigh_parent.%s" % up, "thigh_parent.%s" % low,
        "foot_ik.%s" % up, "foot_ik.%s" % low,
        "c_ikfk_leg.%s" % low, "c_foot_ik.%s" % low,
    ]
    return arm, leg


def _read_switch(pb):
    """读骨骼上的 IK/FK 开关：Rigify 用 IK_FK，ARPro 用 ik_fk_switch。"""
    for key in ("IK_FK", "ik_fk_switch"):
        try:
            val = pb.get(key)
        except Exception:
            val = None
        if val is not None:
            try:
                return "ik" if float(val) < 0.5 else "fk"
            except Exception:
                continue
    return ""


def _find_switch_in_rig(rig, side, limb):
    """扫描整条 rig，找出该侧、该部位最可能的 IK/FK 开关骨骼。

    自定义 Rigify rig（比如 RIG-Klukai）的开关骨骼名不固定，
    不能只靠 hand_ik / upper_arm_parent 这类固定名字。
    """
    words = _LIMB_WORDS.get(limb, ())
    best_mode, best_score = "", -999
    for pb in rig.pose.bones:
        mode = _read_switch(pb)
        if not mode:
            continue
        ps = bone_side(pb.name)
        if ps and ps != side:
            continue
        low = pb.name.lower()
        score = 0
        if words:
            if any(w in low for w in words):
                score += 5
            else:
                score -= 2
        if "ik" in low:
            score += 2
        if "parent" in low or "master" in low or "switch" in low:
            score += 1
        if not side and score <= 0:
            # 侧别未知时只接受「名字确实对得上这个部位」的开关。否则会拿
            # 别的肢体（乃至无侧别的杂项开关）的值冒充，把模式判死成一边。
            continue
        if score > best_score:
            best_score, best_mode = score, mode
    return best_mode


def rig_switch_mode(rig, bone_name, col_name=""):
    """从 rig 的 IK/FK 开关属性判断当前模式（0 = IK，1 = FK）。

    先按已知命名找（Rigify / ARPro），找不到再扫描整条 rig
    （兼容自定义 Rigify rig，比如开关挂在 Wrist_L 这类骨骼上的）。
    """
    side = bone_side(bone_name)
    if not side:
        return ""
    limb = collection_limb(col_name) or bone_limb(bone_name)
    arm, leg = _switch_candidates(side)
    order = leg + arm if limb == "leg" else arm + leg
    for n in order:
        pb = _pb_get(rig, n)
        if pb is None:
            continue
        mode = _read_switch(pb)
        if mode:
            return mode

    return _find_switch_in_rig(rig, side, limb)
    return ""


def effective_mode(rig, bone_name, col_name=""):
    """当前该按哪套导航。

    优先级：rig 的 IK/FK 开关属性 → 骨骼名里的 ik/fk 标记 → 上次用过的模式。

    为什么要让 rig 开关优先：Rigify 里 IK 和 FK 两套控制器同时显示，
    你可能点到一根 FK 骨骼，但手臂实际由 IK 控制——这时应该按 rig 的状态走，
    否则滚轮会跑到「不控制手臂」的那套控制器上。
    """
    m = rig_switch_mode(rig, bone_name, col_name)
    if m:
        return m
    m = bone_mode(bone_name)
    if m:
        return m
    return rig.data.get("cn_last_mode", "")


def remember_mode(rig, bone_name):
    m = bone_mode(bone_name)
    if m:
        rig.data["cn_last_mode"] = m


def bone_mode_inherited(rig, name):
    """名字没有 ik/fk 时，沿父链找最近的模式标记。"""
    m = bone_mode(name)
    if m:
        return m
    try:
        pb = rig.pose.bones.get(name)
        seen = 0
        while pb is not None and seen < 12:
            pb = pb.parent
            seen += 1
            if pb is None:
                break
            m = bone_mode(pb.name)
            if m:
                return m
    except Exception:
        pass
    return ""


def _bone_own_mode(rig, name, fallback=""):
    """按候选骨骼自己的 side/limb 读当前 IK/FK，支持四肢独立。"""
    try:
        bone = rig.data.bones.get(name)
        col = nav_collection(rig, bone) if bone is not None else None
        m = rig_switch_mode(rig, name, col.name if col is not None else "")
        if m:
            return m
    except Exception:
        pass
    return fallback


# ---------------------------------------------------------------- 饼菜单：骨骼集合
#
# 8 个方位是固定的，但**顺序不是顺时针**：Blender 按一张固定方向表分配扇区
# （左/右/下/上/左上/右上/左下/右下），见下面 PIE_EMIT_ORDER 的说明。
# 这里强制填满 8 格，空格用「可点击的占位项」占着。

PIE_DIRS = ("上", "右上", "右", "右下", "下", "左下", "左", "左上")
PIE_DIR_HINTS = ("12 点", "1 点半", "3 点", "4 点半", "6 点", "7 点半", "9 点", "10 点半")

# Blender 往 menu_pie() 里塞的项，方向**不是**按顺序顺时针分 360° 的，而是照
# 一张固定的方向表（源码 uiDirection 的顺序）：
#     第 1 项=左  第 2 项=右  第 3 项=下  第 4 项=上
#     第 5 项=左上 第 6 项=右上 第 7 项=左下 第 8 项=右下
# 对照物：Blender 自带的 3D 视图「操纵器」饼菜单，源码里写着
#     # 1: Left  # 2: Right  # 3: Down  # 4: Up  # 5: Up/Left
# 另外饼菜单每项右边那个小数字就是小键盘方位号（8=上 6=右 2=下 4=左），
# 也正是按这张表来的：项数变化时方向**不跟着重排**（手册明确说"其它项不会
# 移动来填空位"），所以下面这张表是常数。
#
# 我们的 PIE_DIRS 是"语义顺序"（上=头、右=右臂……），和 Blender 的绘制顺序
# 对不上，直接用就会把"上"画到左边去。PIE_EMIT_ORDER 就是两者的换算法：
# 第 k 个画出去的位置该放 PIE_DIRS 的第几项。
PIE_EMIT_ORDER = (6, 2, 4, 0, 7, 1, 5, 3)

# 每个方位的默认语义。第一次拿到一根新骨架时按这些候选词自动认集合，
# 认完就写进骨架数据；用户改过一次之后不再自动覆盖。
PIE_DEFAULT_CANDS = (
    ("头", ("头", "头部", "脖子", "Head", "head", "Neck", "neck")),
    ("右手指", ("右手指", "右手", "右手掌", "Fingers.R", "finger.R", "Hand.R", "hand.R")),
    ("右臂", ("右臂", "右上臂", "右手臂", "Arm.R", "arm.R", "UpperArm.R")),
    ("右腿", ("右腿", "右脚", "大腿.R", "Leg.R", "leg.R", "Foot.R", "Thigh.R")),
    ("躯干", ("躯干", "躯干控制", "身体", "腰", "Torso", "Main", "Spine", "spine")),
    ("左腿", ("左腿", "左脚", "大腿.L", "Leg.L", "leg.L", "Foot.L", "Thigh.L")),
    ("左臂", ("左臂", "左上臂", "左手臂", "Arm.L", "arm.L", "UpperArm.L")),
    ("左手指", ("左手指", "左手", "左手掌", "Fingers.L", "finger.L", "Hand.L", "hand.L")),
)

PIE_SLOT_KEYS = tuple("cn_pie_slot_%d" % i for i in range(8))

# ── 三层饼 ────────────────────────────────────────────────────────────────
# 一个角色不止身体：还有头发、衣服、装备、尾巴……8 格装不下，硬塞就要翻页。
# 按「动画师的使用频率」分成三层，同一个手势配修饰键弹不同的饼：
#     主饼   Alt+左键          身体核心 8 个高频集合（肌肉记忆主场，永远不变）
#     细节饼 Shift+Alt+左键    眼/耳/发束/指节 这类分节细节
#     附属饼 Ctrl+Alt+左键     装备/衣服/尾巴/道具
# Blender 匹配键位时修饰键多的条目优先，所以按住 Shift 甩不会误触主饼。
# 三层槽位各存一套骨架数据（主饼沿用旧键名，升级不丢用户已配好的布局）。
PIE_LAYERS = (
    (1, "主饼", "Alt+左键", 'MESH_CIRCLE'),
    (2, "细节饼", "Shift+Alt+左键", 'CURVES'),
    (3, "附属饼", "Ctrl+Alt+左键", 'MOD_CLOTH'),
)

# 图标名跨版本会变（HAIR 在 5.x 被砍掉、换成 CURVES/STRANDS），label/icon
# 传不存在的枚举会直接抛 TypeError 并把整个面板从那一行截断 —— v4.9.86
# 真机翻车就是这个。任何"按名字给的图标"过一道这里，没有就退回 NONE。
_ICON_OK = {}


def safe_icon(name):
    if name in _ICON_OK:
        return _ICON_OK[name]
    ok = False
    try:
        p = bpy.types.UILayout.bl_rna.functions["label"].parameters.get("icon")
        if p is not None:
            ok = any(e.identifier == name for e in p.enum_items)
    except Exception:
        ok = False
    _ICON_OK[name] = name if ok else 'NONE'
    return _ICON_OK[name]


def pie_slot_keys(layer=1):
    """某一层的 8 个槽位键。层 1 沿用旧键名 cn_pie_slot_0..7。"""
    if layer == 1:
        return PIE_SLOT_KEYS
    return tuple(("cn_pie%d_slot_" % layer) + str(i) for i in range(8))


def pie_init_key(layer=1):
    """某一层的「已初始化」标记键。"""
    return "cn_pie_init" if layer == 1 else ("cn_pie%d_init" % layer)


# 集合名分前缀用的分隔符：头发_01 / Fingers.R / 装备.剑 这类都能切开。
_PREFIX_SEPS = "_-. "


def collection_prefix(name):
    """取集合名的前缀（第一个分隔符之前的部分）；没有分隔符就用全名。"""
    for i, ch in enumerate(name):
        if ch in _PREFIX_SEPS:
            if i == 0:
                continue          # 以下划线开头的怪名，别把空串当前缀
            return name[:i]
    return name


def pie_group_collections(rig, used):
    """把还没被占用的集合按前缀归组，返回 [(代表集合名, 组员数), ...]。

    组按成员数从多到少排 —— 成员多的组往往是「头发_01/头发_02/…」这种
    成套细节，比散件更值得占一个饼位；同数的按出现顺序保持稳定。
    代表选组里名字最短的那个（前缀本体优先，比如 头发 优先于 头发_01）。
    """
    groups = {}
    order = []
    for c in pie_usable_collections(rig):
        if c.name in used:
            continue
        p = collection_prefix(c.name)
        if p not in groups:
            groups[p] = []
            order.append(p)
        groups[p].append(c.name)
    idx = {p: i for i, p in enumerate(order)}
    out = []
    for p in sorted(order, key=lambda p: (-len(groups[p]), idx[p])):
        members = groups[p]
        rep = p if p in members else min(members, key=len)
        out.append((rep, len(members)))
    return out


def _arm_of(rig):
    """rig 传 Object 或 Armature 数据块都行，统一拿 Armature。"""
    return rig.data if getattr(rig, "type", None) == 'ARMATURE' else rig


def _colls(rig):
    """全部骨骼集合（**含子集合**）。Blender 5.x 把 collections 改成
    只含顶层 —— Eldreda 的 Fingers / Arm.L (IK) / Hair 子级等 33 个
    子集合就是这么被漏掉的；collections_all 才是完整列表。
    4.x 没有 collections_all，回退 collections（那时它就是全部）。"""
    ar = _arm_of(rig)
    allc = getattr(ar, "collections_all", None)
    return allc if allc is not None else ar.collections


def _coll_get(rig, name):
    """按名字取集合：顶层走快速 .get，子集合在 collections_all 里找。"""
    if not name:
        return None
    ar = _arm_of(rig)
    try:
        c = ar.collections.get(name)
    except Exception:
        c = None
    if c is not None:
        return c
    try:
        for c in ar.collections_all:
            if c.name == name:
                return c
    except AttributeError:
        pass
    return None


def pie_usable_collections(rig):
    """饼菜单/列表里能选的集合：有骨骼、且不是 MCH 那类内部集合。"""
    out = []
    if rig is None or getattr(rig, "type", None) != 'ARMATURE':
        return out
    try:
        for c in _colls(rig):
            if not len(c.bones):
                continue
            if _INTERNAL_COLL_RE.match(c.name):
                continue
            out.append(c)
    except Exception:
        return out
    return out


def _is_divider_collection(name):
    """`======CLOTHES======` 这类视觉横幅：名字靠 `=` 拉出一条分隔线。

    分配清单里只用来少几行噪音 —— **空**的横幅不列（跳进去也没内容，
    真正的控制器在它的子集合里，那些子集合自己会各占一行）；
    有骨骼的同名集合一律照列，不因为带 `=` 就被吃掉。
    """
    s = (name or "").strip()
    return bool(s) and "=" in s


def assign_collection_rows(rig):
    """饼槽位分配弹窗的清单：骨架里**全部**集合都列，不只插件自己的预设。

    v5.0.4 修的：以前这里走的是 pie_usable_collections，条件是「有骨骼 +
    非内部集合」。于是 Rigify 那类 `Arm.R (FK)` / `Arm.R (Tweak)` /
    `Leg.L (FK)`（切到 FK 模式前是空集合）和纯容器集合
    `======CLOTHES======` 全被筛掉，用户只能从插件的预设名单里挑。

    现在一个不落、顺序跟骨骼面板完全一致：有骨骼的标根数，空的标「（空）」
    —— Rigify 的空 FK 集合是有意义的，切过一次模式才会有成员。
    内部集合（MCH / DEF / ORG）单独一段返回，纯装饰标题不列。

    返回 (native, internal)，每项都是 (集合名, 菜单文字, 图标)。
    """
    native, internal = [], []
    if rig is None or getattr(rig, "type", None) != 'ARMATURE':
        return native, internal
    try:
        cols = list(_colls(rig))
    except Exception:
        return native, internal
    for c in cols:
        n = len(c.bones)
        if not n and _is_divider_collection(c.name):
            continue
        text = ("%s    %d 根" % (c.name, n)) if n else ("%s    （空）" % c.name)
        icon = 'BONE_DATA' if n else 'OUTLINER_COLLECTION'
        bucket = internal if _INTERNAL_COLL_RE.match(c.name) else native
        bucket.append((c.name, text, icon))
    return native, internal


def pie_is_initialized(rig, layer=1):
    try:
        return bool(rig.data.get(pie_init_key(layer), 0))
    except Exception:
        return False


# ══════════════════════════════════════════════════════════════════════════
# 原生集合自动归类（v5.1）
#
# 需求原话：「读取一个没动过的任何绑定，把它默认有的那些集合归类到插件
# 自己的集合里」。所以三层饼不再靠偏好里那 8 个硬编码中文名去凑，而是
# 直接读 _colls(rig)，判断每个集合属于哪个部位、哪个变体。
#
# 三条独立证据，按可靠性排：
#   ① 名字词典（中/英，词长降序）—— 认出九成以上，连一根骨骼都没有的空
#      集合也能认（Rigify 的 Arm.L (FK) 剥掉 (FK) 就是 Arm.L，这一条直接
#      治好了「空 FK/Tweak 集合永远上不了饼」的老毛病）；
#   ② 变体后缀 —— (Tweak)/(Extra) 进附属饼、(Detail) 进细节饼、(FK)/(IK)
#      跟着身体部件进主饼；
#   ③ 几何质心 —— 名字认不出时兜底判部位，名字缺侧别时定左右。
#
# 左右约定**实测过**（这几个 rig 的 .L 都在 +X、.R 都在 -X）：
#   Arm.L (IK) cx=+0.3144   Arm.R (IK) cx=-0.3111
#   Leg.L (IK) cx=+0.0601   Leg.R (IK) cx=-0.0601
# 腿的 |x| 只有 0.06（包围盒半宽 0.56），阈值所以放到 0.05 —— 给 0.12
# 的话左右腿永远判不出侧。
# ══════════════════════════════════════════════════════════════════════════

# 部位词按「优先级」排：越靠前越可信（torso 比 root 更像躯干）。
_COLL_PART_WORDS = (
    ('HAND',  ('手指', '手掌', '手', '指', 'fingers', 'finger', 'hand',
               'palm', 'thumb', 'index', 'middle', 'ring', 'pinky')),
    ('ARM',   ('上臂', '前臂', '手臂', '肩', '臂', 'upperarm', 'forearm',
               'shoulder', 'arm', 'elbow')),
    ('LEG',   ('大腿', '小腿', '脚', '腿', '膝', 'thigh', 'shin', 'calf',
               'foot', 'knee', 'toe', 'leg')),
    ('HEAD',  ('脖子', '脑袋', '头', '颈', '眼', '牙', '舌', 'facerig',
               'face', '脸', 'head', 'neck', 'eye', 'jaw', 'tongue', 'teeth')),
    ('TORSO', ('躯干', '身体', '腰', '胸', '背', '脊', 'torso', 'spine',
               'chest', 'body', 'hip', 'pelvis', 'waist', 'root', '根部')),
    # 尾巴 / 翅膀 / 耳朵这类非人体主干，认得出来但主饼没它们的位子，
    # 会走「部位有值但没槽位」那条路下沉到细节饼。排在最后 = 优先级最低。
    ('TAIL',  ('尾巴', '尾', 'tail')),
)

# (词, 部位, 优先级)：优先级 = 在原始表里的顺序，小的更可信。
_COLL_WORDS = []
_RANK = 0
for _k, _ws in _COLL_PART_WORDS:
    for _w in _ws:
        _COLL_WORDS.append((_w.lower(), _k, _RANK))
        _RANK += 1
# 匹配时词长的先试：「手指」不能被「手」抢走，「手臂」不能被「手」抢走。
_COLL_WORDS.sort(key=lambda p: -len(p[0]))

# 变体标记**只能**出现在括号里、名字末尾、或者 TWK_ 前缀。
# v5.1 修：原来用 `\b(extra)\b`，把 `Extra COGs`（额外的重心控制器）
# 误判成 Extra 变体塞进了附属饼 —— 那里的 Extra 是形容词，不是后缀。
# 另外单独放行「整个名字就叫 Tweaks」这一种（Rigify 的修形集合）。
# v5.2.23 补：SAnby 系（Anby / Ju_Fufu / Trigger）的修形集合是 TWK_ 前缀
#   （TWK_Arm.L / TWK_Leg.R），以前变体认不出 → 当 BODY 挤进细节饼，
#   把 Hair / Accessories 那些真细节挤到一格两个。前缀认出来走附属饼。
_COLL_VARIANT_RE = re.compile(
    r"[\[（(]\s*(tweak|detail|extra|parent|fk|ik)\s*[\]）)]"
    r"|[/_\-\s](tweak|detail|extra|parent|fk|ik)s?\s*$"
    r"|^\s*(tweaks?)\s*$"
    r"|^\s*twk[\s_\-]",
    re.I)

_COLL_SIDE_L = re.compile(r"(\.l$|\.l\b|_l$|_l\b|^l_|\bleft\b|左)", re.I)
_COLL_SIDE_R = re.compile(r"(\.r$|\.r\b|_r$|_r\b|^r_|\bright\b|右)", re.I)

# 内部 / 工具集合：不上饼、不进候选。
# AniMate 的 Deform（44 根全变形骨）和 Reference（66 根纯参考骨）名字
# 不带 MCH/DEF 前缀，靠名字 + 「一根有形状的都没有」两道判据兜住。
_INTERNAL_COLL_NAMES = frozenset((
    'deform', 'reference', 'internal', 'bones', 'tweak',
    'mch', 'org', 'def', 'wgt', 'widget', 'dorm',
))
# 注意：`Secondary` 不在名单里 —— AniMate 的 Secondary 是 43 根带形状的
# 辅助控制器（8 个约束目标指向它），是真给人点的一套，不是变形骨。

# 只配进细节饼的集合：脸部表情那一整套（Facerig / 表情 / facial）。
# 它们是「点开才有用」的表情控制，占掉主饼的「头」格会把真正的头部控制器
# 挤掉 —— 用户要的是主饼只放身体部件。
_SECONDARY_ONLY_RE = re.compile(
    r"(facerig|face[_\- ]?rig|facial|expression|morph|shape[_\- ]?key"
    r"|表情|面部控制|五官)", re.I)

# 主饼 8 格按方位排：0 上 / 1 右上 / 2 右 / 3 右下 / 4 下 / 5 左下 / 6 左 / 7 左上
_PART_SLOT = {
    ('HEAD', ''): 0, ('HEAD', 'L'): 0, ('HEAD', 'R'): 0,
    ('HAND', ''): 1, ('HAND', 'R'): 1,
    ('ARM', ''): 2, ('ARM', 'R'): 2,
    ('LEG', ''): 3, ('LEG', 'R'): 3,
    ('TORSO', ''): 4, ('TORSO', 'L'): 4, ('TORSO', 'R'): 4,
    ('LEG', 'L'): 5,
    ('ARM', 'L'): 6,
    ('HAND', 'L'): 7,
}


def _coll_variant(name):
    """'BODY'（主饼）/ 'DETAIL'（细节饼）/ 'TWEAK'（附属饼）。"""
    hit = _COLL_VARIANT_RE.search(name or "")
    if not hit:
        return 'BODY'
    v = next((g for g in hit.groups() if g), "").lower()
    if v in ('fk', 'ik'):
        return 'BODY'
    if v == 'detail':
        return 'DETAIL'
    return 'TWEAK'          # tweak / extra / parent


# v5.2.23：这组名字是不是「一套 IK/FK 配对的真控制器」。
#   括号式 `Arm.L (IK)` / 后缀式 `Torso_FK` / 前缀式 `IK_Arm.R` 都算。
_NAME_IKFK_PREFIX_RE = re.compile(r"^\s*(ik|fk)[\s_\-]", re.I)


def _name_has_ikfk(name):
    hit = _COLL_VARIANT_RE.search(name or "")
    if hit:
        v = next((g for g in hit.groups() if g), "").lower()
        if v in ('fk', 'ik'):
            return True
    return bool(_NAME_IKFK_PREFIX_RE.match(name or ""))


def _coll_side(name):
    s = name or ""
    if _COLL_SIDE_L.search(s):
        return 'L'
    if _COLL_SIDE_R.search(s):
        return 'R'
    return ''


def _coll_centroid(rig, col):
    """集合质心（局部 X / Z）。只关心左右和高度，Y 不用。"""
    xs, zs = [], []
    for b in col.bones:
        bb = rig.data.bones.get(b.name)
        if bb is None:
            continue
        for p in (bb.head_local, bb.tail_local):
            xs.append(p.x)
            zs.append(p.z)
    if not zs:
        return None
    return (sum(xs) / float(len(xs)), sum(zs) / float(len(zs)))


def _rig_bounds(rig):
    """整根骨架的包围盒 (minx, maxx, minz, maxz)，用来归一化。"""
    xs, zs = [], []
    for b in rig.data.bones:
        xs += [b.head_local.x, b.tail_local.x]
        zs += [b.head_local.z, b.tail_local.z]
    if not xs:
        return (0.0, 1.0, 0.0, 1.0)
    return (min(xs), max(xs), min(zs), max(zs))


def _coll_geo_part(rig, col, box):
    """质心位置猜部位。返回 (部位, 侧别)；实在判不出返回 (None, '')。"""
    c = _coll_centroid(rig, col)
    if c is None:
        return None, ''
    minx, maxx, minz, maxz = box
    z = (c[1] - minz) / max(1e-6, maxz - minz)
    half = max(1e-6, (maxx - minx) / 2.0)
    x = c[0] / half
    # .L 在 +X、.R 在 -X（实测）。阈值 0.05 是为了让左右腿（|x|≈0.11）
    # 也判得出来 —— 0.12 的话腿永远判不出侧。
    side = 'L' if x > 0.05 else ('R' if x < -0.05 else '')
    if z > 0.80 and abs(x) < 0.30:
        return 'HEAD', ''
    if abs(x) > 0.35:
        return ('ARM' if z > 0.55 else 'LEG'), side
    if z <= 0.30:
        return 'LEG', side
    return 'TORSO', ''


def _constraint_targets(rig):
    """被约束引用的骨骼名 —— 用来认「纯参考骨」集合（Reference）。"""
    out = set()
    try:
        for pb in rig.pose.bones:
            for cn in pb.constraints:
                t = getattr(cn, 'subtarget', '')
                if t:
                    out.add(t)
    except Exception:
        pass
    return out


def _widget_coverage(rig):
    """整根骨架里带自定义形状的骨骼占比。

    用来判断这个 rig 到底有没有 widget：课鲁凯.blend 里的 `Klukai`
    整根骨架 custom_shape 全是 None（append 时 widget 丢了），这种 rig
    不能按「有没有形状」来判集合性质，会把整个 rig 判死。
    """
    total = shaped = 0
    try:
        for pb in rig.pose.bones:
            total += 1
            if pb.custom_shape is not None:
                shaped += 1
    except Exception:
        return 0.0
    return (shaped / float(total)) if total else 0.0


def _shaped_ratio(rig, col):
    """集合里有多少骨骼带自定义形状。变形骨集合这个值接近 0。"""
    if not len(col.bones):
        return 0.0
    shaped = 0
    for b in col.bones:
        pb = rig.pose.bones.get(b.name)
        if pb is not None and pb.custom_shape is not None:
            shaped += 1
    return shaped / float(len(col.bones))


def _coll_is_internal(rig, col, targets=None):
    """内部 / 工具集合：不上饼、不进候选。"""
    name = (col.name or "").strip()
    if not name:
        return True
    if _INTERNAL_COLL_RE.match(name):
        return True                        # MCH- / ORG- / DEF- / WGT- 前缀
    if name.lower() in _INTERNAL_COLL_NAMES:
        return True                        # AniMate 的 Deform / Reference
    if not len(col.bones) and _is_divider_collection(name):
        return True                        # ======CLOTHES====== 纯横幅
    # v5.2.23：空的「装饰名」集合也不吃饼位 —— Anby / Ju_Fufu 实测：
    #   `------------` 这种纯符号分隔行、`[ ORG ]` / `[ DEF ]` 空壳标签
    #   集合以前一路排进细节饼，白占两三格，把 Hair / Accessories 挤到
    #   一格两个。有骨骼的不受影响（Rigify 的空 FK 集合是有意义的，
    #   切过一次模式才有成员 —— 名字里带正常字符的一律照留）。
    if not len(col.bones):
        s2 = name.strip()
        if _BRACKET_INTERNAL_RE.match(s2):
            return True
        if not re.search(r"[0-9A-Za-z\u4e00-\u9fff]", s2):
            return True
    # v5.1 删掉了原来的「一根自定义形状都没有 → 纯参考骨」判据：
    # 课鲁凯.blend 里的 Klukai 整根骨架 custom_shape 全是 None（widget
    # 在 append 时丢了），那条判据会把 Torso / Arm.L (IK) / Hair 全判成
    # 内部集合，33 个集合只剩 12 个上饼。宁可放过，不可错杀 —— 认错的
    # 进细节饼，用户点一下就能改回来。
    return False


# v5.2.23：ASCII 部位词改成「词边界 + 可选复数 s」匹配。
#   原来 `w in base` 纯子串，Necklace（项链）里的 neck 命中「头」、
#   Headphones（耳机）里的 head 也是 —— Wise_V1 实测：Necklace 靠 neck
#   以更高的可信级抢走了主饼头格，Eyes 反而被挤进细节饼。
#   规则：词前面不能是字母/数字（Arm.R 的 . 算边界）、后面只能是
#   结尾 / 符号 / 数字 / 复数 s —— necklac**e**、headphon**es** 这类
#   复合词不再命中；中文词没有词边界，保持子串匹配。
_ASCII_WORD_RE_CACHE = {}


def _word_in_base(w, base):
    if not w.isascii():
        return w in base
    pat = _ASCII_WORD_RE_CACHE.get(w)
    if pat is None:
        pat = re.compile(r"(?<![a-z0-9])" + re.escape(w) + r"s?(?![a-z])")
        _ASCII_WORD_RE_CACHE[w] = pat
    return bool(pat.search(base))


# v5.2.23：`[ ORG ]` / `[ DEF ]` 这种括号空壳标签集合（空的时候不上饼）
_BRACKET_INTERNAL_RE = re.compile(r"^\[\s*(ORG|MCH|DEF|WGT)\s*\]$", re.I)


# v5.2.23：饰品 / 配饰类名字不按部位词抢主饼格 —— 课鲁凯皮肤骨架的
#   `Head Accessories`（头部饰品：帽子发卡）以前靠 "head" 占走主饼头格。
#   这些是穿戴物，归细节饼（geo / unsure 路径）。
_ACCESSORY_RE = re.compile(r"accessor|ornament|配饰|饰品|装饰", re.I)


def classify_collection(rig, col, box=None, targets=None):
    """一个原生集合 → (部位, 变体, 侧别, 依据, 优先级)。

    部位：'HEAD' / 'TORSO' / 'ARM' / 'LEG' / 'HAND' / None
    变体：'BODY' / 'DETAIL' / 'TWEAK'
    侧别：'L' / 'R' / ''
    依据：'name'（名字认出来）/ 'geo'（靠位置猜）/ ''
    优先级：命中词在词表里的原始顺序，越小越可信（torso 比 root 可信）
    """
    name = col.name or ""
    variant = _coll_variant(name)
    base = _COLL_VARIANT_RE.sub(" ", name).lower()
    part, conf, rank = None, '', 999
    if not _ACCESSORY_RE.search(base):
        for w, key, rk in _COLL_WORDS:
            if _word_in_base(w, base):
                part, conf, rank = key, 'name', rk
                break
    side = _coll_side(name)
    if part is None and len(col.bones) and box is not None:
        part, gside = _coll_geo_part(rig, col, box)
        if part:
            conf = 'geo'
            side = side or gside
    return part, variant, side, conf, rank


_AUTO_CACHE = {}


def _mixed_side_members(rig, col):
    """集合里的骨骼按名字分左右，返回 (左手名单, 右手名单)；分不开返回 None。

    只认名字**结尾**的侧别（`.L` / `_L` / `.l` / 左 / Left）—— 中间的 "_l"
    会把 `c_leg_pole.r` 这类名字算成左侧。
    """
    if col is None or not len(col.bones):
        return None
    lft, rgt = [], []
    for b in col.bones:
        if _COLL_SIDE_L.search(b.name):
            lft.append(b.name)
        elif _COLL_SIDE_R.search(b.name):
            rgt.append(b.name)
    if not lft or not rgt:
        return None
    return lft, rgt


def _split_collection_by_side(rig, col):
    """把左右混在一起的集合（Rigify 的 `Fingers`）拆成 `Fingers.L` / `Fingers.R`。

    原集合一根骨骼都不动 —— 只是**另建**两个集合，把同一批骨骼按侧别再挂一遍，
    所以绑定的结构、权重、约束全不受影响，只是多了两条能单独导航的入口。
    重复调用是幂等的：已经拆过就直接复用，不会增生 `.L.001`。
    """
    got = _mixed_side_members(rig, col)
    if got is None:
        return None
    out = []
    for tag, members in ((".L", got[0]), (".R", got[1])):
        name = col.name + tag
        target = _coll_get(rig, name)
        try:
            if target is None:
                target = _arm_of(rig).collections.new(name)
        except Exception:
            return None
        have = set(b.name for b in target.bones)
        for n in members:
            if n in have:
                continue
            pb = rig.pose.bones.get(n)
            if pb is None:
                continue
            try:
                target.assign(pb)
            except Exception:
                pass
        out.append(target)
    return out


def auto_layout(rig, use_cache=True, allow_create=True):
    """读原生集合，自动排三层饼。

    返回 (slots, report)：
      slots  = {1: [[组], ...8], 2: [...], 3: [...]}   一格可以挂多个集合
      report = {'total','placed','layers','skip','overflow','unsure'}

    分层规则（用户定的）：
      · 主饼    —— 身体部件，同一部位的 IK/FK 合成一格；
      · 细节饼  —— 头发 / 马尾 / 衣服 / 装备 / 武器，以及 (Detail) 变体；
      · 附属饼  —— (Tweak)/(Extra)/(Parent) 这类修形专用骨。

    **只有名字认出来的才进主饼**，靠几何猜的进细节饼 —— 实测 `马尾`
    质心在躯干高度，几何会把它塞进「躯干」格，那就错了。
    """
    slots = {1: [[] for _ in range(8)],
             2: [[] for _ in range(8)],
             3: [[] for _ in range(8)]}
    report = {'total': 0, 'placed': 0,
              'layers': {1: 0, 2: 0, 3: 0},
              'skip': [], 'overflow': [], 'unsure': [], 'split': []}
    if rig is None or getattr(rig, 'type', None) != 'ARMATURE':
        return slots, report

    # 面板 draw 里每帧都会问到布局，而整根骨架 795 根骨头的包围盒 +
    # 约束目标扫描不便宜。按「骨骼数 + 集合数」做键缓存；操作符里
    # （真正要落盘时）传 use_cache=False 强制重算。
    ckey = None
    if use_cache:
        try:
            ckey = (rig.name, len(rig.data.bones), len(_colls(rig)))
            hit = _AUTO_CACHE.get(ckey)
            if hit is not None:
                return hit
        except Exception:
            ckey = None

    box = _rig_bounds(rig)
    targets = _constraint_targets(rig)
    # 只有带 widget 的 rig 才用「有没有形状」判集合性质
    widget_ok = _widget_coverage(rig) >= 0.05
    order = {}
    for i, c in enumerate(_colls(rig)):
        order[c.name] = i

    body, other = [], []
    for col in _colls(rig):
        report['total'] += 1
        if _coll_is_internal(rig, col, targets):
            if not _is_divider_collection(col.name):
                report['skip'].append(col.name)
            continue
        part, variant, side, conf, rank = classify_collection(
            rig, col, box, targets)
        # 名字认不出部位、且集合里基本全是变形骨（没有可点的控件）→ 排除。
        # Dialyn-琉音 暴露的：Clothes(65 根 0 形状)、Cage(18 根 0 形状)、
        # Correctives(19 根 0 形状)、Misc(120 根只有 6 根有形状) —— 这些
        # 上了饼也点不出东西。反面例子是 Hair(103 根 102 根有形状），那是
        # 能摆的，必须留。
        # 判据挂在「依据」上，不能挂 part is None —— 几何兜底会给
        # Clothes / Cage / Misc 也填上一个部位（质心在躯干高度），
        # 那样 part 就不是 None 了，判据被绕过，变形骨照样上饼。
        if conf == 'geo' and widget_ok and _shaped_ratio(rig, col) < 0.30:
            report['skip'].append(col.name)
            continue
        if conf == 'geo':
            report['unsure'].append(col.name)
        if variant == 'BODY':
            layer = 1 if (part and conf == 'name') else 2
        elif variant == 'DETAIL':
            layer = 2
        else:
            layer = 3
        # 脸部表情那一整套（Facerig / 表情 / facial）不进主饼：它是「点开才有用」
        # 的表情骨，占到主饼的头格就把真正的头部控制器挤没了。
        if _SECONDARY_ONLY_RE.search(col.name or ""):
            layer = 2
        base = _COLL_VARIANT_RE.sub(" ", col.name or "").strip()
        # 手指类原集合常常左右混在一起（Rigify 的 `Fingers`）：拆成
        # `Fingers.L` / `Fingers.R` 两个真集合，左进左手指格、右进右手指格。
        # 不拆的话整个集合只能塞进两格里，两边点进去都是同一锅骨骼。
        # allow_create=False（面板 draw 里那条只读路径）不建集合 ——
        # draw 里新建 datablock 是要出事的，那条路退回「两边共用一格」。
        if allow_create and part == 'HAND' and not side and layer in (1, 2):
            pair = _split_collection_by_side(rig, col)
            if pair:
                report['split'].append(col.name)
                for tag, sub in zip((".L", ".R"), pair):
                    report['split'].append(sub.name)
                    if layer == 2:
                        pref = 7 if tag == ".L" else 1
                        if not slots[2][pref]:
                            slots[2][pref] = [sub.name]
                            continue
                        other.append((order.get(col.name, 999), sub.name, 2))
                    else:
                        body.append((order.get(col.name, 999), sub.name,
                                     part, tag[1], rank, len(base)))
                continue
        rec = (order.get(col.name, 999), col.name, part, side, rank, len(base))
        if layer == 1:
            body.append(rec)
        else:
            other.append((order.get(col.name, 999), col.name, layer))

    # ---- 主饼：同一个 (部位, 侧别) 只留「最可信」的那批，其余下沉细节饼 ----
    # 否则 `躯干` 和 `躯干叠加骨骼` 会挤进同一格，点一下选中两套骨骼。
    grouped = {}
    pending = {}
    for _o, name, part, side, rank, blen in sorted(body):
        grouped.setdefault((part, side), []).append((rank, blen, name))
    for (part, side), items in sorted(grouped.items()):
        # 整组都是空集合就别占主饼的位子 —— rig.002 的 `脖子，脑袋` 是 0 根
        # 骨骼的空集合，靠名字抢到了「头」那一格，点进去什么都没有。
        if not any(len(_coll_get(rig, n).bones) for _r, _l, n in items
                   if _coll_get(rig, n) is not None):
            for _r, _l, name in items:
                other.append((order.get(name, 999), name, 2))
            continue
        # 同部位只留「最可信 + 名字最短」的那批。`躯干` / `躯干控制` /
        # `躯干叠加骨骼` 三个都命中「躯干」，rank 相同，靠名字长度分主次 ——
        # 否则点一格会同时选中三套骨骼。而 `Arm.L (IK)` 和 `Arm.L (FK)`
        # 剥掉变体后都是 `Arm.L`，长度一致，会被当成同一套的同格兄弟。
        items.sort(key=lambda it: (it[0], it[1], it[2]))
        best_rank, best_len = items[0][0], items[0][1]
        slot = _PART_SLOT.get((part, side))
        if slot is None:
            slot = _PART_SLOT.get((part, ''))
        if slot is None:
            for _r, _l, name in items:
                other.append((order.get(name, 999), name, 2))
            continue
        # v5.2.23：先记到 pending，等全部组都算完再落位 —— 不同
        # (部位, 侧别) 组映射到同一格时不再无脑叠加。
        #   课鲁凯皮肤骨架实测：`Upper Arm`（换装皮肤的整臂控制器）靠
        #   (ARM,'') 和 Arm.R (FK)/(IK) 的 (ARM,'R') 一起落进右臂格，
        #   `Toes (Cerulean Breaker)` 同理挤进右腿格 —— 点一格选中三套。
        #   仲裁：带 IK/FK 变体标记的那组胜（一套配对的真控制器才带），
        #   都不带时取名字最短那组；输家整组下沉细节饼。
        kept = [n for r, l, n in items if r == best_rank and l == best_len]
        rest = [n for r, l, n in items if not (r == best_rank and l == best_len)]
        hasv = any(_name_has_ikfk(n) for n in kept)
        min_len = min(l for r, l, n in items if r == best_rank and l == best_len)
        pending.setdefault(slot, []).append(
            (0 if hasv else 1, min_len, part, side, kept, rest))
    for slot, cands in pending.items():
        cands.sort(key=lambda c: (c[0], c[1], c[3]))
        for _sc, _ml, _p, _s, kept, rest in cands[1:]:
            for n in kept + rest:
                other.append((order.get(n, 999), n, 2))
        _sc, _ml, part, side, kept, rest = cands[0]
        for name in rest:
            other.append((order.get(name, 999), name, 2))
        for name in kept:
            slots[1][slot].append(name)
            # Rigify 的 `Fingers` 是双手共用一个集合，没有侧别。只放
            # 右手位的话饼的左边空一格，用户往左推会落空 —— 同时占
            # 左右两个位，两边都能点进去，反正指的是同一组骨骼。
            if part == 'HAND' and not side:
                slots[1][7 if slot == 1 else 1].append(name)

    # ---- 细节饼 / 附属饼：一格一个，满了往下一层溢 ----
    # 细节饼里「剥掉变体后同名」的合并成一格：`Tail (IK)` + `Tail (FK)`
    # 在面板里是两个集合，在饼上该是一个「尾巴」。
    merged = {}
    queue = []
    for o, name, layer in sorted(other):
        base = _COLL_VARIANT_RE.sub(" ", name or "").strip()
        if layer == 2 and base and base != name.strip():
            merged.setdefault(base, []).append((o, name))
        else:
            queue.append((o, [name], layer))
    for base, items in merged.items():
        queue.append((items[0][0], [n for _o, n in items], 2))
    queue.sort(key=lambda q: q[0])

    for _o, names, layer in queue:
        chain = (2, 3) if layer == 2 else (3,)
        placed = False
        for lay in chain:
            for i in range(8):
                if not slots[lay][i]:
                    slots[lay][i] = list(names)
                    placed = True
                    break
            if placed:
                break
            # 这一层满了也别丢：从后往前找「只挂 1 个」的格子追加，
            # 一格最多 2 个。点进去会一起选中，但总比找不到强；
            # 报告里会列出来，面板里点一下就能换回去。
            for i in range(7, -1, -1):
                if len(slots[lay][i]) < 2:
                    slots[lay][i].extend(names)
                    placed = True
                    break
            if placed:
                break
        if not placed:
            report['overflow'].extend(names)

    seen = set()
    for lay in (1, 2, 3):
        report['layers'][lay] = sum(1 for g in slots[lay] if g)
        for g in slots[lay]:
            seen.update(g)
    report['placed'] = len(seen)
    if ckey is not None:
        if len(_AUTO_CACHE) > 8:
            _AUTO_CACHE.clear()
        _AUTO_CACHE[ckey] = (slots, report)
    return slots, report


def _split_group(raw):
    """槽位值 → 集合名列表。一格可以挂多个，用 | 分隔。"""
    return [n for n in (raw or "").split("|") if n.strip()]


def pie_slot_groups(rig, layer=1):
    """某一层每格的**完整集合组**（v5.1：一格可以挂多个集合）。

    这是只读函数，跟 pie_slots 一样不许在 draw 里落盘。
    """
    groups = [[] for _ in range(8)]
    if rig is None or getattr(rig, 'type', None) != 'ARMATURE':
        return groups
    if not pie_is_initialized(rig, layer):
        if not pref('pie_auto_fill', True):
            return groups
        try:
            auto, _rep = auto_layout(rig, allow_create=False)
            return auto.get(layer, groups)
        except Exception:
            # 老逻辑兜底：一格一个集合
            return [[n] if n else [] for n in pie_default_slots(rig, layer)]
    for i, k in enumerate(pie_slot_keys(layer)):
        groups[i] = [n for n in _split_group(rig.data.get(k, ""))
                     if _coll_get(rig, n) is not None]
    return groups


def set_pie_slot_group(rig, layer, slot, names):
    """写一格（多个集合用 | 连接）。只能是操作符里调，draw 里写会被拒。"""
    if rig is None or not (0 <= slot < 8) or layer not in (1, 2, 3):
        return False
    key = pie_slot_keys(layer)[slot]
    val = "|".join([n for n in (names or []) if n])
    rig.data[key] = val
    return True


def pie_all_default_slots(rig):
    """三层默认布局一次算齐，返回 (slots1, slots2, slots3) —— 纯只读。

    主饼按方位语义认名字（见 PIE_DEFAULT_CANDS），认完的空位拿预设清单补；
    剩下的集合按前缀归组（见 pie_group_collections）：组大的先上，
    细节饼拿前 8 组、附属饼拿后 8 组，还有剩的散件顺着空格补。
    三层加起来每个集合只出现一次。

    **不写任何数据** —— 面板和饼菜单都在 draw() 里调这条链，draw 上下文
    写 ID 属性会被 Blender 直接拒绝（关键区别见 pie_slots 的说明）。
    """
    slots1, slots2, slots3 = [""] * 8, [""] * 8, [""] * 8
    cols = pie_usable_collections(rig)
    if not cols:
        return slots1, slots2, slots3
    used = set()

    def claim(cands, fuzzy):
        for cand in cands:
            low = cand.lower()
            for c in cols:
                if c.name in used:
                    continue
                hit = (low in c.name.lower() or c.name.lower() == low) if fuzzy \
                    else (c.name == cand or c.name.lower() == low)
                if hit:
                    return c
        return None

    for i, (_label, cands) in enumerate(PIE_DEFAULT_CANDS):
        hit = claim(cands, fuzzy=False) or claim(cands, fuzzy=True)
        if hit is not None:
            slots1[i] = hit.name
            used.add(hit.name)

    rest = [n for n in preset_names()
            if n not in used and _coll_get(rig, n) is not None]
    for i in range(8):
        if slots1[i] or not rest:
            continue
        slots1[i] = rest.pop(0)
        used.add(slots1[i])

    # 剩余集合按前缀归组：细节饼拿前 8 组，附属饼拿后 8 组
    reps = [rep for rep, _n in pie_group_collections(rig, used)]

    # 兜底池：只收「独苗组」的代表。多员组的非代表成员（头发_02/头发_03
    # 这类）不自动上饼 —— 它们和代表是同一组，占了格子反而挤掉别的功能，
    # 要选它们在指定弹窗里按组挑。
    groups = {}
    for c in cols:
        if c.name in used:
            continue
        groups.setdefault(collection_prefix(c.name), []).append(c.name)
    pool = [p for p, ms in groups.items() if len(ms) == 1 and p in ms
            and p not in used]

    for i in range(8):
        if i < len(reps):
            slots2[i] = reps[i]
            used.add(reps[i])
    for i in range(8):
        j = 8 + i
        if j < len(reps):
            slots3[i] = reps[j]
            used.add(reps[j])

    # 独苗散件顺着细节饼/附属饼的空格补
    for slots in (slots2, slots3):
        for i in range(8):
            if slots[i]:
                continue
            for n in pool:
                if n not in used:
                    slots[i] = n
                    used.add(n)
                    break
    return slots1, slots2, slots3


def pie_default_slots(rig, layer=1):
    """某一层的默认布局（纯只读，不落盘）。"""
    return pie_all_default_slots(rig)[layer - 1]


def pie_slots(rig, layer=1):
    """某一层的 8 个饼菜单槽位（空位是空串）—— **每格取组里第一个名字**。

    v5.1 起一格可以挂多个集合（`Arm.L (IK)` + `Arm.L (FK)` 同属「左臂」），
    完整组用 pie_slot_groups 取。这里保留「每格一个名字」的老签名，是因为
    既有的大量绘制 / 导航 / 测试代码都按名字用，没必要全改一遍。

    **只读。** 面板和饼菜单都在 draw() 里调它，draw 上下文里写
    rig.data[...] 会抛 AttributeError 并把整块 UI 截断，所以写入只发生在
    操作符里（freeze_pie_slots / init_pie_slots / set_pie_slot）。
    """
    return [(g[0] if g else "") for g in pie_slot_groups(rig, layer)]


def freeze_pie_slots(rig, layer=1):
    """把某一层「当前看到的布局」固化进骨架数据。只能在操作符/非 draw 上下文调用。"""
    if rig is None or getattr(rig, "type", None) != 'ARMATURE':
        return [""] * 8
    groups = pie_slot_groups(rig, layer)
    for i, g in enumerate(groups):
        rig.data[pie_slot_keys(layer)[i]] = "|".join(g)
    rig.data[pie_init_key(layer)] = 1
    return [("|".join(g) if g else "") for g in groups]


def init_pie_slots(rig, layer=1):
    """按默认规则强制重填某一层的 8 格（会覆盖手改）。只能在操作符里调用。"""
    if rig is None or getattr(rig, "type", None) != 'ARMATURE':
        return [""] * 8
    for k in pie_slot_keys(layer):
        rig.data[k] = ""
    # v5.1：不再走老的「方位语义 + 偏好预设补位」，直接吃 auto_layout 的
    # 归类结果 —— 「重新自动填充」和「一键配置」从此是同一份答案。
    slots, _rep = auto_layout(rig, use_cache=False)
    groups = slots.get(layer, [[] for _ in range(8)])
    for i in range(8):
        set_pie_slot_group(rig, layer, i, groups[i])
    rig.data[pie_init_key(layer)] = 1
    return [("|".join(g) if g else "") for g in groups]


def set_pie_slot(rig, index, name, layer=1):
    """改某一层的一格。改之前先把该层其余格子的当前显示值固化 —— 否则「未初始化」
    状态被打破的那一刻，别的格子会因为自动逻辑重算而悄悄移位。"""
    if rig is None or not (0 <= index < 8):
        return
    if not pie_is_initialized(rig, layer):
        freeze_pie_slots(rig, layer)
    rig.data[pie_slot_keys(layer)[index]] = name or ""
    rig.data[pie_init_key(layer)] = 1


def pie_current_collection(rig):
    """当前导航集合名（饼菜单高亮用）。"""
    if rig is None:
        return ""
    name = rig.data.get("cn_collection", "")
    if name and _coll_get(rig, name) is not None:
        return name
    active = rig.data.bones.active
    if active is not None:
        col = nav_collection(rig, active)
        if col is not None:
            return col.name
    return ""


def _coll_mode_tag(rig, col):
    """集合的 IK/FK 侧别：按成员骨骼名里的 ik/fk 标记投票。

    全部带同一标记 → 'ik' / 'fk'；混装或没人带标记 → ''（不分侧）。
    """
    if rig is None or col is None:
        return ""
    tags = set()
    try:
        names = collection_bone_names(rig, col)
    except Exception:
        names = []
    for n in names:
        t = bone_mode(n)
        if t:
            tags.add(t)
            if len(tags) > 1:
                break
    if len(tags) == 1:
        return tags.pop()
    if not tags:
        # 骨骼名里没有标记时看集合名：`Arm.L (IK)` / `FK_Arm.L` /
        # `右臂（反向动力学）` 这类。混装（两种标记都有）不猜。
        low = (col.name or "").lower()
        if re.search(r"[(\[（]\s*ik\s*[)\]）]|^ik[_\-\s]|[_\-\s]ik$|反向动力学",
                     low):
            return "ik"
        if re.search(r"[(\[（]\s*fk\s*[)\]）]|^fk[_\-\s]|[_\-\s]fk$|正向动力学",
                     low):
            return "fk"
    return ""


# 集合名对部位时剔掉的模式词：IK_Arm.L / FK_Arm.L / 右臂（正向动力学）
# 剔完都剩「右臂 / arm」—— 剩下的部分一样才算同一部位的两套。
_RESOLVE_STRIP_RE = re.compile(
    r"(正向动力学|反向动力学|动力学|正向|反向|fk|ik)", re.I)


def _coll_limb_mode(rig, names):
    """按「集合里的骨骼属于哪条肢体」读该肢体自己的 IK/FK 开关。

    集合里常混进与部位无关的骨头（版权幌子骨 JD_License.*、道具、饰品），
    拿 names[0] 去猜侧别会直接猜空，于是模式判不出来。ikfk_limbs 已经把
    每条肢体的 IK/FK 控制器两端都认了出来，用交集最大的那条肢体读值最稳。
    """
    s = set(names)
    if not s:
        return ""
    try:
        _kind, limbs = ikfk_limbs(rig)
    except Exception:
        limbs = []
    best_hit, best_lb = 0, None
    for lb in limbs:
        hit = len(s & (set(lb.ik_controls) | set(lb.fk_controls)))
        if hit > best_hit:
            best_hit, best_lb = hit, lb
    if best_lb is None:
        return ""
    try:
        m = limb_mode(rig, best_lb)
    except Exception:
        m = None
    return (m or "").lower()


def _resolve_mode_collection(rig, col_name):
    """格子指向的集合与当前 IK/FK 态相反时，改指**实际显示**的那一套。

    RIG-Orphie / 课鲁凯这类绑定把 IK、FK 两套控制器分装两个集合，rig 的
    UI 脚本只显示当前态的那套（另一套藏起来）。饼格若按名字自动认格认到了
    FK 集合、而视图正处在 IK 态，预期是点这格直接落在看得见的 IK 控制器上，
    而不是把藏起来的 FK 揭出来。

    规则：
      · 只认**同部位另一侧**的集合当替身（集合名剔掉模式词后相同，或
        ikfk_limbs 认出的 IK/FK 控制器两端）。别处的集合可见，不能拿来当
        「目标藏着」的证据。
      · 当前模式优先读 rig 自己的 IK/FK 开关（按集合骨骼所属肢体读）。
        开关读不出来时以**显示**为准。
      · 显示 = 集合可见 且 骨骼没被 H。Rigify 逐根 H，容器绑定切集合显隐，
        两个机制都得认。
      · 找不到替身、两侧都没显示、混装、无标记 —— 一律原样返回。
    """
    if rig is None or getattr(rig, "type", None) != 'ARMATURE':
        return col_name
    col = _coll_get(rig, col_name)
    if col is None:
        return col_name
    tag = _coll_mode_tag(rig, col)
    if not tag:
        return col_name
    try:
        names = collection_bone_names(rig, col)
    except Exception:
        names = []
    want_set = set(names)
    # 当前模式只认 rig 自己的 IK/FK 开关（按目标集合的肢体读）。活动骨
    # **只有在它本身就属于这个集合**时才算数 —— 否则上一格选中的余温骨
    # （比如躯干的 Spine1_M_fk）会把「当前模式」带偏。
    act = rig.data.bones.active
    probes = []
    if act is not None and act.name in want_set:
        probes.append(act.name)
    if names:
        probes.append(names[0])
    for bn in names:
        if bone_side(bn):
            probes.append(bn)
            break
    try:
        mode = _coll_limb_mode(rig, names)
    except Exception:
        mode = ""
    if not mode:
        for bn in probes:
            try:
                m = rig_switch_mode(rig, bn, col.name)
            except Exception:
                m = ""
            if m:
                mode = m.lower()
                break

    def _vis(nm_list):
        return sum(1 for n in nm_list if bone_visible(rig.pose.bones.get(n)))

    def _shown(c, nm_list):
        """真正显示在视图里的骨数：集合关着就一根都不算。"""
        if not col_visible(c):
            return 0
        return _vis(nm_list)

    def _base(nm):
        b = _RESOLVE_STRIP_RE.sub("", (nm or "").lower())
        return re.sub(r"[\s_\-.（）()\[\]（）]+", "", b)

    my_base = _base(col.name)
    try:
        _kind, limbs = ikfk_limbs(rig)
    except Exception:
        limbs = []
    want = set(names)

    def _paired(nm, cs):
        """nm/cs 是不是本集合的**同部位另一侧**。"""
        if my_base and _base(nm) == my_base:
            return True
        if limbs:
            for lb in limbs:
                ik_s, fk_s = set(lb.ik_controls), set(lb.fk_controls)
                if ((want & fk_s and cs & ik_s)
                        or (want & ik_s and cs & fk_s)):
                    return True
        return False

    self_shown = _shown(col, names)
    mates = []                       # 同部位另一侧：(集合, 骨名表, 显示骨数)
    for _c in _colls(rig):
        if _c.name == col.name:
            continue
        _ct = _coll_mode_tag(rig, _c)
        if not _ct or _ct == tag:
            continue
        try:
            _cn = collection_bone_names(rig, _c)
        except Exception:
            _cn = []
        if not _cn:
            continue
        if not _paired(_c.name, set(_cn)):
            continue
        mates.append((_c, _cn, _shown(_c, _cn)))
    other_shown = max([m[2] for m in mates] or [0])
    force_vis = (self_shown == 0 and other_shown > 0)

    if not force_vis:
        if mode and tag == mode:
            return col_name
        if not mode:
            # 开关读不出来：目标自己显示着 → 它就是要用的那套。
            if self_shown > 0 or not mates or other_shown == 0:
                return col_name

    best_key, best_name = None, col_name
    for c, cn, shown in mates:
        ctag = _coll_mode_tag(rig, c)
        if force_vis and shown == 0:
            continue                  # 只认「正显示着」的那套顶替
        if self_shown > 0 and shown == 0:
            continue                  # 目标自己显示着：绝不换进整套藏起来的那套
        if not force_vis and mode and ctag != mode:
            continue                  # 开关可读时必须正好是当前态
        key = (1 if shown > 0 else 0,
               1 if (mode and ctag == mode) else 0,
               1 if col_visible(c) else 0,
               len(cn))
        if best_key is None or key > best_key:
            best_key, best_name = key, c.name
    return best_name


def enter_pie_collection(context, rig, col_name, pie_layer=0):
    """进入饼菜单指定的集合：先框住整组控制器，再落到第一根。

    框选走原生 view_selected（和 Home 键同一套算法，不用自己算包围盒），
    但**用缓动滑过去**（frame_view_animated），不再瞬移 —— 瞬移会让人看丢
    "我现在在骨架的哪个位置"。之后交给 goto_collection，让它把「显示集合、
    选中第一根、记住循环下标、鼠标跟随」这些既有逻辑统一走一遍。
    """
    col = _coll_get(rig, col_name)
    if col is None:
        return False, "骨架里没有集合「%s」" % col_name

    # v5.2.27：格子指向的集合与当前 IK/FK 态相反（视图在 IK 态、格子里
    # 是 FK 集合，或反之）时，改落在当前实际显示的那一套 —— 不把 rig
    # 脚本藏起来的另一套揭出来。
    alt = _resolve_mode_collection(rig, col_name)
    if alt != col_name:
        col_name = alt
        col = _coll_get(rig, col_name)
        if col is None:
            return False, "骨架里没有集合「%s」" % col_name

    # ★ v5.2.43：独显态下跳集合 —— 目标不在独显里就并进去。
    #   否则目标骨骼全被 solo 遮住，跳过去屏幕毫无反应（用户会以为「没跳成」）。
    #   已在独显覆盖范围内的不重复补星：祖先打了星，子集合本来就亮着。
    try:
        if solo_active(rig) and not solo_covers(rig, col_name):
            solo_set(rig, solo_names(rig) + [col_name], commit=True)
    except Exception:
        pass

    # 隐藏的集合也要能进：collection_nav_names 会先把它显示出来
    names = collection_nav_names(rig, col)
    if not names:
        return False, "集合「%s」里没有骨骼 —— 先把控制器放进去" % col_name
    remember_scope(rig, col_name)

    # 先清空选择：不然后面 view_selected 会把上一条指令选中的残留一起框进来，
    # 框出来的范围比这个集合大一圈，看着就像"跳歪了"
    for pb in rig.pose.bones:
        pb.select = False
    first = default_first_bone(rig, col_name)
    if first not in names:
        first = smart_first_bone(rig, names)
    for n in names:
        select_bone(rig, n, extend=True)

    if not frame_view_animated(context, rig, first):
        # 兜底：缓动没启动（上下文不对、定时器注册失败、或用户关掉了自动取景）。
        # 这条路的框选也得吃同一套余量 —— 否则它比缓动路径近 1/余量 倍，
        # 一旦走到这里就是"瞬间贴脸"。
        if not native_frame_selected(context):
            smooth_view_to(context, rig, names[0])
    # 从饼里进过的集合顺手加进循环清单，滚轮接着切才接得上
    ensure_preset_name(col_name)
    return goto_collection(context, rig, col_name, pie_layer)


def collection_nav_names(rig, col, mode="", cur=""):
    """集合里能导航的骨骼名：**先把集合显示出来，再过滤**。

    v4.9.87 及以前是「先按可见性过滤，再显示集合」，所以跳进一个被隐藏的
    集合（控制台里勾掉了眼晴）时，13 根控制器全被判成不可见 → 报
    「集合「马尾」里没有可用的控制器」—— 明明里面全是控制器。
    现在顺序反过来：先 show_collection，再按可见性/IK-FK 过滤；
    万一过滤之后一个不剩（控制器被单独 H 掉了），就退回集合全量 ——
    用户既然点名要进这个集合，就不该给他一个空结果。
    """
    if col is None or rig is None:
        return []
    show_collection(col)
    names = [n for n in collection_bone_names(rig, col)
             if rig.pose.bones.get(n) is not None]
    if not names:
        return []
    nav = filter_nav_names(rig, names, mode, cur)
    return nav or names


def filter_nav_names(rig, names, mode="", cur_name="", keep_all=False):
    """导航过滤：可见性优先，再按每根骨骼自己的 IK/FK 模式收敛。

    ARP 的四肢可以各自 IK/FK；不能用一个全局 mode 过滤整张列表。
    通用控制器（无 ik/fk 标记）始终保留。

    v5.2.9 兜底：过滤后**剩不到 2 根**、而原始列表有 2 根以上时，退回
    原始列表。实测 bug（RIG - Anby_SoldierZero）：FK_Arm.L 三根 FK 里两根
    被 H 隐藏（rig 脚本在 IK 态藏 FK），过滤后只剩 FK_Hand.L 一根 ——
    集合顺序排得再好滚轮也永远切不动。

    v5.2.11：**集合导航一律 keep_all**。实测 v5.2.9 的「剩<2 才兜底」
    卡不住剩 2 根的场景：FK_Arm.L 三根里大臂被 H，滚轮永远在
    前臂↔手之间打转，大臂怎么都轮不到（用户的原话：明明归类在这个
    集合里却切不到）。集合是用户一根根归进去的，成员就该完整轮，
    隐藏骨由 select_bone 顺手显示兜住；可见性过滤只留给全局列表。
    """
    if keep_all:
        full = [n for n in names if _pb_get(rig, n) is not None]
        # v5.2.29：偏好「跳过隐藏骨骼」现在也管集合内导航 —— 用户手动 H 掉
        # 的骨骼，滚轮/↑↓ 直接跳过，不再由 select_bone 揭出来（v5.2.11 的「完整轮」
        # 只对看得见的成员生效）。当前骨被跳过、一根不剩或剩不到 2 根时
        # 退回全表，不重演 v5.2.9 的「轮不动」卡死。
        # v5.2.34：缩放隐身（ARP 离开那套的控制器）**恒定**跳过 —— 形状画不出来，
        # 鼠标根本点不到，留在导航里只会让滚轮空转。关掉「跳过隐藏骨骼」也不放行。
        if pref('skip_hidden', True):
            vis = [n for n in full if bone_visible(rig.pose.bones.get(n))]
        else:
            vis = [n for n in full if not widget_ghost(rig.pose.bones.get(n))]
        # 当前骨自己就是个隐身的控制器（刚切完模式，手上这根正属于离开的那套）
        # 时，不该为了留住它退回全表 —— 那样另一套又全回来了。只认缩放隐身，不认手动 H。
        cur_pb = rig.pose.bones.get(cur_name) if cur_name else None
        ghost = bool(cur_pb is not None and widget_ghost(cur_pb))
        if len(vis) >= 2 and (not cur_name or cur_name in vis or ghost):
            full = vis
        # v5.2.15：keep_all 也要跳过「当前模式另一套」的控制器 —— ARP 骨架
        # FK 态滚轮会切到看不见的 c_hand_ik.r（集合里 IK/FK 混排，模式标记
        # 按名字认，跟可见性无关）。Anby 那种「被 H 的 FK 骨」不受影响：
        # 它们有 FK 标记、当前模式也是 FK，照样完整轮；只有和当前模式
        # 相反的那套被跳过。当前骨被滤掉或一根不剩 → 退回全表（不重演
        # v5.2.9 的「轮不动」卡死）。
        if pref('same_mode', True):
            m = mode or (bone_mode(cur_name) if cur_name else "") \
                or (bone_mode_inherited(rig, cur_name) if cur_name else "")
            if not m:
                # v5.2.34：当前骨不带 ik/fk 标记时（ARP 的 c_foot_roll_cursor.x、
                # c_foot_01.l 这类「同集合里的通用控制器」），以前整段模式过滤
                # 直接跳过 —— FK 态滚轮于是能滑到隐藏的 IK 控制器上。改成从集合里
                # 带标记的成员反查这条肢体的开关，拿得到就照常过滤。
                for n in full:
                    if not bone_mode(n):
                        continue
                    m = _bone_own_mode(rig, n, "")
                    if m:
                        break
            if m:
                kept = []
                for n in full:
                    # v5.2.28：只认骨骼自己名字里的标记。以前还带
                    # bone_mode_inherited（父链继承），Orphie 的 neck/head/Shoulder
                    # 自己没标记却继承到 fk，撞上从别的集合泄漏来的
                    # ik 模式就被整根剔出导航 —— 滚轮跳项。父链证据太弱，
                    # 不该用来把集合成员除名。
                    tag = bone_mode(n)
                    if not tag:
                        kept.append(n)
                        continue
                    own = _bone_own_mode(rig, n, m)
                    if own and tag != own:
                        continue
                    kept.append(n)
                if kept and (not cur_name or cur_name in kept):
                    return kept
        return full

    out = []
    for n in names:
        pb = rig.pose.bones.get(n)
        if pb is None:
            continue
        if widget_ghost(pb):          # v5.2.34：缩放隐身恒定跳过
            continue
        if pref('skip_hidden', True) and not bone_visible(pb):
            continue
        out.append(n)

    if pref('same_mode', True):
        kept = []
        for n in out:
            tag = bone_mode(n)  # v5.2.28：同上，父链继承不再参与剔除
            if not tag:
                kept.append(n)
                continue
            own = _bone_own_mode(rig, n, mode)
            if own and tag != own:
                continue
            kept.append(n)
        if kept:
            out = kept

    full = [n for n in names if rig.pose.bones.get(n) is not None]
    if cur_name and cur_name in full and cur_name not in out:
        return full
    if len(out) < 2 and len(full) >= 2:
        return full
    return out


def nav_list(rig, cur_name):
    """当前集合内的骨骼（已按可见性和 IK/FK 模式过滤）。

    匹配空间排序关键字的集合：按 (head_z, tail_z) 从高到低。
    其余集合：完全按你 assign 进集合的顺序。
    """
    _error_wash()
    bone = _bone_get(rig, cur_name) if cur_name else None

    if bone is not None:
        col = nav_collection(rig, bone)
        if col is not None:
            mode = effective_mode(rig, cur_name, col.name)
            return filter_nav_names(rig, collection_bone_names(rig, col), mode,
                                    cur_name, keep_all=True)
    mode = effective_mode(rig, cur_name, "")
    return filter_nav_names(rig, [pb.name for pb in rig.pose.bones], mode, cur_name)


def nav_neighbor_name(rig, cur_name, step):
    """当前导航列表里，向前/向后 step 的下一个骨骼名（会绕回）。

    快速旋转模态里滚轮切关节就是用它，保证和 ↑↓ 的范围完全一致
    （同样的集合、同样的 IK/FK 和可见性过滤）。

    **先把这次用的集合记下来（remember_scope）再取列表**：不然切到一根同时
    属于「前发」和「Hair」的骨骼时，next 一步会按"最小集合"重算范围，
    13 根就变成一会儿 5 根、一会儿 8 根，永远轮不完。
    """
    _error_wash()
    bone = _bone_get(rig, cur_name) if cur_name else None
    col = nav_collection(rig, bone) if bone is not None else None
    if col is not None:
        remember_scope(rig, col.name)
        _error_wash()
        mode = effective_mode(rig, cur_name, col.name)
        names = filter_nav_names(rig, collection_bone_names(rig, col), mode,
                                 cur_name, keep_all=True)
    else:
        _error_wash()
        names = nav_list(rig, cur_name)
    if not names:
        return ""
    if cur_name not in names:
        return names[0]
    idx = (names.index(cur_name) + step) % len(names)
    return names[idx]


def scope_label(rig, cur_name):
    bone = rig.data.bones.get(cur_name) if cur_name else None
    if bone is None:
        return "全局"
    col = nav_collection(rig, bone)
    if col is None:
        return "全局（不在集合内）"
    return "%s（%d 个，%s）" % (col.name, len(col.bones),
                                collection_sort_label(rig, col))


# ---------------------------------------------------------------- 预设集合数据

class CN_Preset_v45(bpy.types.PropertyGroup):
    col_name: bpy.props.StringProperty(
        name="集合名",
        default="",
        description="要创建/归入的骨骼集合名",
    )


def ensure_default_presets():
    """第一次启用时把默认预设填进偏好（用户清空后不会再强行填回来）。"""
    p = get_prefs()
    if p is None:
        return
    try:
        if p.presets_initialized:
            return
        if len(p.presets) == 0:
            for name in DEFAULT_PRESETS:
                p.presets.add().col_name = name
        p.presets_initialized = True
    except Exception:
        pass


def preset_names():
    """偏好里的预设集合名（去空、去重、保序）。"""
    p = get_prefs()
    if p is None:
        return []
    out = []
    for item in p.presets:
        name = (item.col_name or "").strip()
        if name and name not in out:
            out.append(name)
    return out


def ensure_preset_name(name):
    """确保集合名在偏好预设清单里；新建集合后 M 菜单能看到它。"""
    name = (name or "").strip()
    if not name:
        return False
    p = get_prefs()
    if p is None:
        return False
    for item in p.presets:
        if (item.col_name or "").strip() == name:
            return False
    try:
        p.presets.add().col_name = name
        return True
    except Exception:
        return False


def ensure_collection(arm, name):
    """拿到集合，没有就新建。返回 (collection, 是否新建)。"""
    col = _coll_get(arm, name)
    if col is not None:
        return col, False
    return arm.collections.new(name), True


# ---------------------------------------------------------------- 集合循环

_INTERNAL_COLL_RE = re.compile(r"^(MCH|ORG|DEF|WGT|Widget)", re.IGNORECASE)


def importable_collections(rig):
    """骨架里适合放进循环清单的集合：可见、非空、且不是 MCH/ORG/DEF/Widget 内部集合。"""
    out = []
    if rig is None:
        return out
    for c in _colls(rig):
        if len(c.bones) == 0:
            continue
        if _INTERNAL_COLL_RE.match(c.name):
            continue
        if not col_visible(c):
            continue
        out.append(c.name)
    return out


def cycle_order(rig):
    """固定循环顺序：预设清单里所有存在的集合（含空集合）。

    循环下标按这个完整列表记，所以某个集合被清空/加回骨骼时，
    顺序不会重置（空集合在切换时会被跳过，但不影响下标体系）。
    """
    out = []
    if rig is None:
        return out
    for n in preset_names():
        if _coll_get(rig, n) is not None:
            out.append(n)
    return out


def selection_collection_name(rig, bone=None):
    """当前选中骨骼真正所在的集合（优先预设集合），忽略“手动指定”的旧缓存。"""
    if rig is None:
        return ""
    if bone is None:
        bone = rig.data.bones.active
    if bone is None:
        return ""
    cols = [c for c in bone.collections if col_visible(c)]
    if not cols:
        return ""
    preset = set(preset_names())
    for c in cols:
        if c.name in preset:
            return c.name
    cols.sort(key=lambda c: len(c.bones))
    return cols[0].name


def current_cycle_name(rig, bone=None):
    """当前处在哪个集合：活动骨骼所在的集合优先，其次才是手动指定的。"""
    name = selection_collection_name(rig, bone)
    if name:
        return name
    forced = rig.data.get("cn_collection")
    if forced and _coll_get(rig, forced):
        return forced
    return None


def _is_descendant(col, ancestor):
    """col 是不是 ancestor 的子代（含多层）。"""
    c = col.parent if col is not None else None
    while c is not None:
        if c == ancestor:
            return True
        c = c.parent
    return False


_VIS_SNAPSHOT_KEY = "cn_vis_snapshot"


def save_visibility_snapshot(rig, force=False):
    """记下所有集合的「看得见 / 单独显示」状态。

    只在第一次进饼时存（force=True 可以覆盖），因为饼间互斥会不停改
    is_visible，跟着更新的话快照就跟着脏了，还原等于没还原。
    """
    if rig is None or getattr(rig, 'type', None) != 'ARMATURE':
        return False
    if not force and rig.data.get(_VIS_SNAPSHOT_KEY):
        return False
    try:
        data = {}
        for c in _colls(rig):
            data[c.name] = [bool(c.is_visible), bool(c.is_solo)]
        rig.data[_VIS_SNAPSHOT_KEY] = json.dumps(data, ensure_ascii=False)
        return True
    except Exception:
        return False


def restore_visibility(rig):
    """按快照还原集合可见性。返回还原了几个；没有快照返回 -1。"""
    if rig is None or getattr(rig, 'type', None) != 'ARMATURE':
        return -1
    raw = rig.data.get(_VIS_SNAPSHOT_KEY, "")
    if not raw:
        return -1
    try:
        data = json.loads(raw)
    except Exception:
        return -1
    n = 0
    for c in _colls(rig):
        st = data.get(c.name)
        if st is None:
            continue
        try:
            c.is_visible = bool(st[0])
            c.is_solo = bool(st[1])
            n += 1
        except Exception:
            pass
    return n


def hide_collection_tree(col):
    """连子集合一起隐藏。"""
    if col is None:
        return
    try:
        for child in list(col.children):
            hide_collection_tree(child)
        col.is_visible = False
    except Exception:
        pass


def _pie_managed_names(rig):
    """三个饼层槽位里出现过的集合（并集）—— 就是「饼菜单管得着的那批」。

    v5.1：必须走 pie_slot_groups 展开整组。一格挂多个集合时
    （`手指` 格 = 手指 + 右手指 + 左手指），只取每格第一个的话，
    组里的其余集合会被当成「别的饼的」而藏掉 —— 明明就在当前这张饼上。
    """
    out = set()
    for layer in (1, 2, 3):
        try:
            for g in pie_slot_groups(rig, layer):
                for n in g:
                    if n and _coll_get(rig, n) is not None:
                        out.add(n)
        except Exception:
            pass
    return out


def isolate_previous_collection(rig, keep_name, pie_layer=0):
    """独占显示：**饼与饼之间**互斥，同一张饼里不互相挤。

    v5.0.3 按用户反馈改的：之前是「进哪个集合就只留哪个」，同一张饼里换
    个部位，别的集合也被收掉 —— 多余了，反正对骨骼做移动/旋转时本来就会
    独占。现在跨饼规则只有一条：

      - 进饼时把**这张饼的全部槽位集合**亮出来，其他饼的集合整批收起
        （「每一次只显示一张饼的内容」）。同一张饼里的兄弟集合不会被藏，
        所以同饼内换部位屏幕不变。

    饼槽位是自动推导的：进过的集合会被补进预设清单、槽位跟着漂移，所以
    这里每次都**现算**，不用缓存 —— 现算的结果和用户眼前那张饼永远一致，
    还能顺手收拾「一打开所有集合都亮着」的初始状态。
    滚轮 / 侧栏面板路径（pie_layer=0）不知道自己在哪张饼里，不参与互斥。
    """
    if not pref('isolate_collection', True):
        return
    if not pie_layer:
        return
    # ★ v5.2.43：独显生效时整段跳过。
    #   solo 遮罩优先于 is_visible，这里的 hide_collection_tree / show_collection
    #   对屏幕没有任何实际作用；而 show_collection 里那句「看不见就清星标」
    #   会把整份独显清光 —— 这就是「跳进独显集合却退出独显」的直接原因。
    #   而且 solo 本身已经比「饼间互斥」更严格，跳过不会让画面变乱。
    if solo_active(rig):
        try:
            rig.data["cn_last_pie_layer"] = pie_layer
        except Exception:
            pass
        return
    # 第一次进饼先拍一张原始显示状态，之后随时能一键还原
    save_visibility_snapshot(rig)
    # 同样是整组展开，不是每格第一个
    cur = set()
    for g in pie_slot_groups(rig, pie_layer):
        for n in g:
            if n and _coll_get(rig, n) is not None:
                cur.add(n)
    if not cur:
        return
    keep = _coll_get(rig, keep_name)
    for n in _pie_managed_names(rig) - cur:
        if n == keep_name:
            continue
        col = _coll_get(rig, n)
        if col is None or col is keep:
            continue
        # 要进的集合是它的子代就别藏父级，否则刚显示又被自己藏掉
        if keep is not None and _is_descendant(keep, col):
            continue
        hide_collection_tree(col)
    # 当前这张饼的全部槽位集合亮出来。例外：某格的集合与当前 IK/FK 态
    # 相反、而它的当前态搭档也在本饼 —— 那套是 rig 脚本特意藏起来的
    # （IK 态藏 FK），跟着亮出来就是「点一下饼，藏起来的 FK 满屏」。
    for n in sorted(cur):
        col = _coll_get(rig, n)
        if col is not None and _coll_mode_tag(rig, col):
            alt = _resolve_mode_collection(rig, n)
            if alt != n and alt in cur:
                continue
        show_collection(col)
    rig.data["cn_last_pie_layer"] = pie_layer


def show_collection(col):
    """显示集合及其所有父级；**非独显态**下被遮挡才清 solo。

    v5.2.43 修正：原来是无条件「不可见就清光所有星标」。但 col_visible 走的是
    `is_visible_effectively`，它**把 solo 算在内** —— 于是独显三个部位之后，
    饼里任何一格不在独显里的集合都会把整份独显清掉。用户报的「滑到格子上就
    退出独显」就是这么来的（isolate_previous_collection 会对整张饼 8 格逐个
    show_collection，必然踩到不在独显里的格子）。

    solo 是遮罩，不是「看不见就得拆」的东西。它只允许被两种操作改写：
    新的 solo_set（重新打星）和用户的显式还原（solo_clear）。
    """
    if col is None:
        return
    chain = []
    c = col
    while c is not None:
        chain.append(c)
        c = c.parent
    for c in chain:
        try:
            c.is_visible = True
        except Exception:
            pass
    if not col_visible(col) and not solo_active(col.id_data):
        try:
            for c in _colls(col.id_data):
                c.is_solo = False
        except Exception:
            pass
        for c in chain:
            try:
                c.is_visible = True
            except Exception:
                pass


def default_first_bone(rig, col_name):
    """某集合的「默认第一根骨骼」（切换集合时先选它）。空串 = 用排序第一根。"""
    if rig is None or not col_name:
        return ""
    try:
        return rig.data.get("cn_first_" + col_name, "") or ""
    except Exception:
        return ""


# 名字里带这些的，永远不配当「第一根」：许可证水印、属性开关、机制骨
# `parent` 这一类是 Rigify 系的 IK/FK 属性开关骨（`thigh_parent.L` /
# `Hip_parent_L` / `Shoulder_parent_L`），它们自带 widget 形状、名字也干净，
# 光靠 widget 打分会被选成第一根 —— 用户报的「大腿根部那个属性骨」。
_FIRST_BAD_RE = re.compile(
    r"(license|licence|copyright|watermark|水印|属性|(^|[-_. ])jd([-_. ]|$)"
    r"|(^|[-_. ])parent([-_. ]|$)|(^|[-_. ])prop([-_. ]|$)"
    r"|^mch|[-_. ]mch([-_. ]|$)|^org[-_.]"
    r"|^def[-_.]|deform)", re.I)
# 名字里带这些的，是给人转的控制骨，优先
_FIRST_GOOD_RE = re.compile(r"(master|ctrl|control|grip|_fk|_ik|[.]fk|[.]ik)", re.I)


def smart_first_bone(rig, names):
    """没手动指定「默认骨骼」时，挑真正有动画用途的那根当第一根。

    v5.1 及以前直接取 names[0] —— 但集合里的第一根往往不是能转的控制器：
    Rigify 系每个肢体都把 `Hip_parent_L` / `thigh_parent.L` 排在腿格最前，
    而 IK/FK 开关就长在它身上。点进「左腿」先选中它，方向键一转是在翻属性，
    脚一点不动 —— 用户报的「第一个控件应该是脚底 IK 而不是大腿根属性骨」。

    判定顺序：
      1. 集合里有 IK/FK 肢体命中 → 按当前模式落**末端控制器**
         （IK 态落 Ankle_ik / foot_ik，FK 态落 Ankle_fk / foot_fk）；
      2. 否则打分：有 widget 形状、名字带 master/ctrl 的优先，
         许可证 / 属性 / MCH / ORG / DEF 一律垫底。
    """
    if rig is None or not names:
        return ""
    pool = [n for n in names if rig.pose.bones.get(n) is not None]
    if not pool:
        return names[0]
    try:
        _kind, limbs = ikfk_limbs(rig)
    except Exception:
        limbs = []
    if limbs:
        pset = set(pool)
        for lb in limbs:
            ik_hit = [n for n in lb.ik_controls if n in pset]
            fk_hit = [n for n in lb.fk_controls if n in pset]
            if not (ik_hit or fk_hit):
                continue
            mode = limb_mode(rig, lb) or "IK"
            # 先按当前模式那一侧挑；这一侧一根都没在这个集合里（比如 FK 骨
            # 被单独收进别的集合），退到另一侧，总比落回属性骨强
            pick = _primary_control(ik_hit if mode == "IK" else fk_hit)
            if not pick:
                pick = _primary_control(fk_hit if mode == "IK" else ik_hit)
            if pick:
                return pick
    best, best_score = pool[0], None
    for n in pool:
        pb = rig.pose.bones.get(n)
        score = 0
        if _FIRST_BAD_RE.search(n):
            score -= 100
        if _FIRST_GOOD_RE.search(n):
            score += 3
        try:
            if pb is not None and pb.custom_shape is not None:
                score += 4
        except Exception:
            pass
        if best_score is None or score > best_score:
            best, best_score = n, score
    return best


def goto_collection(context, rig, name, pie_layer=0):
    """切到指定集合：显示它、选中第一根骨骼、把它设为导航集合。

    选第一根时会沿用切换前的 IK/FK 模式（比如从 FK 切过来就优先选 FK 控制器）；
    如果给这个集合指定过「默认骨骼」（面板里设，存在 cn_first_<集合名>），
    就直接落到它上面 —— 比如腿部集合默认落脚部 IK，不再落在没用的 MCH/根骨骼。

    pie_layer：从哪一张饼进来的（1/2/3），0 = 滚轮或侧栏面板这条路。
    饼菜单要靠它把「上一张饼的集合」整套收掉，见 isolate_previous_collection。
    """
    col = _coll_get(rig, name)
    if col is None:
        return False, "骨架里没有集合「%s」" % name
    active = rig.data.bones.active
    mode = effective_mode(rig, active.name, name) if active is not None else ""
    bones = collection_nav_names(rig, col, mode)
    if not bones:
        return False, "集合「%s」里没有骨骼 —— 先把控制器放进去" % name
    remember_scope(rig, name)
    # 独占：上一个集合 / 上一张饼的集合整组收起（骨骼清单已经取过，
    # 这里藏不会影响本次跳转）
    isolate_previous_collection(rig, name, pie_layer)
    first = default_first_bone(rig, name)
    if first not in bones:
        first = smart_first_bone(rig, bones)
    select_bone(rig, first, extend=False)
    remember_mode(rig, first)
    rig.data["cn_collection"] = name

    # 记住在完整循环顺序里的位置（含空集合），清空集合也不会打乱顺序
    ordered = cycle_order(rig)
    if name in ordered:
        rig.data["cn_cycle_index"] = ordered.index(name)
        rig.data["cn_cycle_sig"] = "|".join(ordered)

    if view_tween_busy():
        # 视口正在滑过去的中途：别再起第二个动画（两个定时器抢 view_location
        # 会抖一下然后瞬移），只把"动画结束后鼠标落在哪"改成本次要选中的骨骼
        pb = rig.pose.bones.get(first)
        if pb is not None:
            _VIEW_TWEEN["warp"] = rig.matrix_world @ pb.head
    elif not smooth_view_to(context, rig, first):
        follow_cursor_to(context, rig, first)
    return True, "%s（%d 个）" % (name, len(bones))


def step_collection(context, rig, delta):
    names = cycle_order(rig)
    if not names:
        return False, "循环清单里没有已存在的集合，先点「＋ 当前集合」添加"

    sig = "|".join(names)
    idx = rig.data.get("cn_cycle_index", -1)
    if rig.data.get("cn_cycle_sig", "") != sig or not (0 <= idx < len(names)):
        cur = current_cycle_name(rig, rig.data.bones.active)
        idx = names.index(cur) if cur in names else (-1 if delta > 0 else 0)

    # 空集合跳过，但下标仍按完整清单推进，顺序不会被打乱
    for k in range(1, len(names) + 1):
        ni = (idx + delta * k) % len(names)
        col = _coll_get(rig, names[ni])
        if col is not None and len(col.bones) > 0:
            return goto_collection(context, rig, names[ni])
    return False, "循环清单里的集合都是空的，先往里放控制器"


def apply_bones_to_collection(arm, col, bones):
    """把骨骼放进集合。

    偏好 move_bones=True（默认）：先从其他所有集合移出，等于原生「移动」。
    move_bones=False：只添加，保留原来的集合（复制）。

    bones 可以是 PoseBone 也可以是 Bone（BoneCollection.assign 只吃 Bone）。
    """
    move = bool(pref('move_bones', True))
    for b in bones:
        bone = getattr(b, "bone", b)
        if move:
            for other in list(bone.collections):
                if other.name != col.name:
                    other.unassign(bone)
        col.assign(bone)
    return len(bones)


def rig_collection_rows(rig):
    """当前 rig 里存在、但不在全局预设清单里的可导航集合。"""
    if rig is None:
        return []
    presets = set(preset_names())
    rows = []
    for col in _colls(rig):
        if col.name in presets:
            continue
        if _INTERNAL_COLL_RE.match(col.name):
            continue
        if not col_visible(col):
            continue
        rows.append((col.name, "%s   %d 个" % (col.name, len(col.bones)),
                     'BONE_DATA'))
    return rows


def preset_menu_rows(rig):
    """(集合名, 菜单文字, 图标) —— 纯数据，方便脱离 UI 测试。"""
    rows = []
    for name in preset_names():
        col = _coll_get(rig, name) if rig else None
        if col is None:
            rows.append((name, "%s   ＋新建" % name, 'ADD'))
        else:
            rows.append((name, "%s   %d 个" % (name, len(col.bones)), 'BONE_DATA'))
    return rows


# ---------------------------------------------------------------- 预设集合操作

class POSE_OT_cn_create_presets_v45(bpy.types.Operator):
    """把偏好里的预设集合补建到当前骨架。

    已存在的集合原样跳过（不会清空里面的骨骼），新建的是空集合。
    """
    bl_idname = "pose.cn_create_presets_v45"
    bl_label = "创建预设集合"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return pose_poll(context)

    def execute(self, context):
        rig = context.object
        names = preset_names()
        if not names:
            self.report({'WARNING'}, "偏好里还没有预设集合名，先去插件偏好里添加")
            return {'CANCELLED'}

        created = []
        for name in names:
            col, is_new = ensure_collection(rig.data, name)
            if is_new:
                created.append(name)

        if created:
            self.report({'INFO'}, "新建 %d 个空集合：%s" % (len(created), "、".join(created)))
        else:
            self.report({'INFO'}, "预设集合都已存在，无需新建")
        return {'FINISHED'}


def cross_move_sources(rig, bones, target_name, preset_only=True):
    """这些骨骼当前还在哪些「其它预设集合」里。

    用来在归组前提醒：如果选中的骨骼里有别的部位（比如左臂 FK 还选着），
    直接点「右臂」会把它们从左臂搬走。
    """
    allowed = set(preset_names()) if preset_only else None
    out = {}
    for pb in bones:
        for c in pb.bone.collections:
            if c.name == target_name:
                continue
            if allowed is not None and c.name not in allowed:
                continue
            out.setdefault(c.name, []).append(pb.name)
    return out


class POSE_OT_cn_assign_preset_v45(bpy.types.Operator):
    """把选中的控制器归入指定集合（集合不存在会自动新建）。"""
    bl_idname = "pose.cn_assign_preset_v45"
    bl_label = "归入预设集合"
    bl_options = {'REGISTER', 'UNDO'}

    collection: bpy.props.StringProperty(name="集合名")

    @classmethod
    def poll(cls, context):
        return pose_poll(context)

    def invoke(self, context, event):
        rig = context.object
        name = (self.collection or "").strip()
        bones = selected_bones(rig)
        if not bones:
            self.report({'WARNING'}, "先在视口里选中要归入的控制器")
            return {'CANCELLED'}
        sources = cross_move_sources(rig, bones, name)
        if sources and pref('confirm_cross_move', True):
            return context.window_manager.invoke_props_dialog(self, width=460)
        return self.execute(context)

    def draw(self, context):
        layout = self.layout
        rig = context.object
        name = (self.collection or "").strip()
        bones = selected_bones(rig)
        sources = cross_move_sources(rig, bones, name)

        layout.label(text="要移入「%s」的 %d 根骨骼里，" % (name, len(bones)), icon='INFO')
        for src in sorted(sources):
            names = sources[src]
            layout.label(text="有 %d 根目前属于「%s」，会被移走：" % (len(names), src), icon='ERROR')
            shown = "、".join(names[:8]) + ("  …" if len(names) > 8 else "")
            layout.label(text="    " + shown)
        layout.separator()
        layout.label(text="确认要一起移走就点 OK；否则点 Cancel（不会改动任何东西）", icon='INFO')

    def execute(self, context):
        rig = context.object
        name = (self.collection or "").strip()
        if not name:
            self.report({'WARNING'}, "集合名是空的")
            return {'CANCELLED'}

        bones = selected_bones(rig)
        if not bones:
            self.report({'WARNING'}, "先在视口里选中要归入的控制器")
            return {'CANCELLED'}

        sources = cross_move_sources(rig, bones, name)
        col, is_new = ensure_collection(rig.data, name)
        apply_bones_to_collection(rig.data, col, bones)

        msg = "已归入「%s」%d 个控制器%s" % (name, len(bones), "（新建集合）" if is_new else "")
        if sources:
            msg += "；同时从 %s 移出" % "、".join("「%s」%d 根" % (s, len(v))
                                                for s, v in sorted(sources.items()))
        self.report({'INFO'}, msg)
        return {'FINISHED'}


class POSE_OT_cn_new_collection_v45(bpy.types.Operator):
    """新建一个集合，并把选中的控制器放进去（相当于原生 M 新建+重命名）。"""
    bl_idname = "pose.cn_new_collection_v45"
    bl_label = "新建集合并归入"
    bl_options = {'REGISTER', 'UNDO'}

    collection_name: bpy.props.StringProperty(name="集合名", default="新集合")

    @classmethod
    def poll(cls, context):
        return pose_poll(context)

    def invoke(self, context, event):
        if not selected_bones(context.object):
            self.report({'WARNING'}, "先在视口里选中要归入的控制器")
            return {'CANCELLED'}
        return context.window_manager.invoke_props_dialog(self, width=320)

    def execute(self, context):
        rig = context.object
        name = (self.collection_name or "").strip()
        if not name:
            self.report({'WARNING'}, "集合名不能为空")
            return {'CANCELLED'}
        bones = selected_bones(rig)
        if not bones:
            self.report({'WARNING'}, "先在视口里选中要归入的控制器")
            return {'CANCELLED'}

        col, is_new = ensure_collection(rig.data, name)
        apply_bones_to_collection(rig.data, col, bones)
        try:
            rig.data["cn_collection"] = col.name
        except Exception:
            pass
        self.report({'INFO'}, "已把 %d 个控制器归入「%s」%s"
                    % (len(bones), col.name,
                       "（新建集合）" if is_new else "（集合已存在）"))
        return {'FINISHED'}


class POSE_OT_cn_remove_from_collection_v45(bpy.types.Operator):
    """把选中的控制器从集合里移出（集合、骨骼都保留，只是不再属于它）"""
    bl_idname = "pose.cn_remove_from_collection_v45"
    bl_label = "从集合移出"
    bl_options = {'REGISTER', 'UNDO'}

    collection: bpy.props.StringProperty(name="集合名", default="")
    all_collections: bpy.props.BoolProperty(name="所有集合", default=False)

    @classmethod
    def poll(cls, context):
        return pose_poll(context)

    def execute(self, context):
        rig = context.object
        bones = selected_bones(rig)
        if not bones:
            self.report({'WARNING'}, "先在视口里选中要移出的控制器")
            return {'CANCELLED'}

        if self.all_collections:
            targets = list(_colls(rig))
            label = "所有集合"
        else:
            name = (self.collection or "").strip()
            if not name:
                name = current_cycle_name(rig, rig.data.bones.active) or ""
            col = _coll_get(rig, name) if name else None
            if col is None:
                self.report({'WARNING'}, "当前没有可移出的集合")
                return {'CANCELLED'}
            targets = [col]
            label = "「%s」" % name

        moved = 0
        for pb in bones:
            bone = pb.bone
            for c in targets:
                if c.bones.get(bone.name) is not None:
                    c.unassign(bone)
                    moved += 1

        if moved == 0:
            self.report({'WARNING'}, "选中的控制器本来就不在这些集合里")
            return {'CANCELLED'}
        self.report({'INFO'}, "已把 %d 根骨骼从%s移出" % (len(bones), label))
        return {'FINISHED'}


class CN_MT_preset_collections_v45(bpy.types.Menu):
    bl_idname = "CN_MT_preset_collections_v45"
    bl_label = "预设集合"

    def draw(self, context):
        layout = self.layout
        rig = context.object
        has_sel = bool(selected_bones(rig))
        rows = preset_menu_rows(rig)
        col = layout.column(align=True)
        for name, text, icon in rows:
            row = col.row()
            row.enabled = has_sel
            op = row.operator("pose.cn_assign_preset_v45", text=text, icon=icon)
            op.collection = name


class CN_MT_other_collections_v45(bpy.types.Menu):
    bl_idname = "CN_MT_other_collections_v45"
    bl_label = "当前骨架其它集合"

    def draw(self, context):
        layout = self.layout
        rig = context.object
        has_sel = bool(selected_bones(rig))
        rows = rig_collection_rows(rig)
        col = layout.column(align=True)
        for name, text, icon in rows:
            row = col.row(align=True)
            c = _coll_get(rig, name)
            empty = c is not None and len(c.bones) == 0
            row.enabled = has_sel or empty
            op = row.operator("pose.cn_assign_preset_v45", text=text, icon=icon)
            op.collection = name
            if empty:
                op2 = row.operator("pose.cn_delete_empty_collection_v45",
                                   text="", icon='TRASH')
                op2.collection = name


def pie_rig(context):
    """饼菜单绘制时的骨架。context.object 在某些 UI 上下文里是空的，逐层兜底。

    v5.2.19：一个文件里好几套骨架（元骨 + 成品骨 + 表情骨）时，active 常常
    不是骨架，原来「从 selected_objects 里挑第一根」会挑到跟用户正在看的
    无关的那套 —— 弹窗里列的集合就驴唇不对马嘴（Eldreda_Sunbringer 实测）。
    现在挑到多根时优先取**配置过饼**的那根（data 里有 cn_* 记号）。
    """
    for attr in ("object", "active_object"):
        ob = getattr(context, attr, None)
        if ob is not None and getattr(ob, "type", None) == 'ARMATURE':
            return ob
    cands = []
    for ob in (getattr(context, "selected_objects", None) or []):
        if getattr(ob, "type", None) == 'ARMATURE':
            cands.append(ob)
    if not cands:
        ob = bpy.context.object
        if ob is not None and getattr(ob, "type", None) == 'ARMATURE':
            cands.append(ob)
    if len(cands) > 1:
        for ob in cands:
            try:
                if any(str(k).startswith("cn_") for k in ob.data.keys()):
                    return ob
            except Exception:
                pass
    return cands[0] if cands else None


def pie_menu_poll(context):
    """饼菜单项的统一 poll —— **恒 True，一个筛都不做**。

    pie 里 poll 返回 False 的项会被**整项丢弃**：那个方位的按钮直接消失，
    你按肌肉记忆往那边甩就落空（或者甩到相邻那格上）。所以这里绝不做任何
    检查：有没有骨架、是不是只读库链接、集合在不在，统统放到 execute 里报错。
    """
    return True


class POSE_OT_cn_pie_note_v45(bpy.types.Operator):
    """饼菜单里的提示项（没有骨架时占位，点了就说怎么才能用）"""
    bl_idname = "pose.cn_pie_note_v45"
    bl_label = "提示"
    bl_options = {'INTERNAL'}

    message: bpy.props.StringProperty()

    @classmethod
    def poll(cls, context):
        return True

    def execute(self, context):
        self.report({'WARNING'}, self.message or "这里没东西可跳")
        return {'CANCELLED'}


class POSE_OT_cn_pie_jump_v45(bpy.types.Operator):
    """跳到集合：框住整组控制器，再选中第一根"""
    bl_idname = "pose.cn_pie_jump_v45"
    bl_label = "跳转集合"
    bl_options = {'REGISTER', 'UNDO'}

    collection: bpy.props.StringProperty()
    # 从哪张饼点进来的。饼之间的「上一张整套收起来」全靠它，
    # 缺了就只能退化成「只藏上一个集合」那个不够用的老规则。
    layer: bpy.props.IntProperty(name="饼层", default=0, min=0, max=3)

    @classmethod
    def poll(cls, context):
        return pie_menu_poll(context)

    def execute(self, context):
        rig = pie_rig(context)
        if rig is None:
            self.report({'WARNING'}, "先选中一根骨架（姿态模式）再用饼菜单")
            return {'CANCELLED'}
        if not edit_ok(rig):
            self.report({'WARNING'}, "这根骨架是只读的（库链接）")
            return {'CANCELLED'}
        ok, msg = enter_pie_collection(context, rig, self.collection, self.layer)
        self.report({'INFO'} if ok else {'WARNING'}, msg)
        return {'FINISHED'} if ok else {'CANCELLED'}


class POSE_OT_cn_pie_assign_v45(bpy.types.Operator):
    """给饼菜单的某一格指定集合（小窗：过滤框 + 本骨架集合列表 + 清空）

    v5.2.10：v5.2.9 还是塞了一堆「预设新建 / 内部集合 / 格内顺序 / 整格
    换位」—— 用户点开这里只有一个目的：把这格换成另一个集合。多余的
    全删（新建集合在骨骼面板按 M 就行）。另修一个嵌套弹窗 bug：弹窗里
    点任何一行都会走 invoke() 又开一层新弹窗（叠三层就是它），现在
    from_popup 的调用直接落 execute，点完即写入并关窗。
    """

    bl_idname = "pose.cn_pie_assign_v45"
    bl_label = "指定饼菜单格子"
    bl_options = {'REGISTER', 'UNDO'}

    slot: bpy.props.IntProperty(name="格位", default=0, min=0, max=7)
    layer: bpy.props.IntProperty(name="饼层", default=1, min=1, max=3)
    # v5.2.16：这四个必须 SKIP_SAVE —— 侧栏/饼菜单按钮只设置 slot 和 layer，
    # 其它属性会继承「本会话上次调用留下的值」：弹窗里点过一次集合后
    # from_popup=True + collection=那个集合 被记住，之后侧栏每点一格都
    # 直接走 execute 把同一个集合写进去，弹窗都不开（用户实测：
    # 点了一下「头部」，之后所有格子全变「头部」）。
    collection: bpy.props.StringProperty(
        name="集合", options={'SKIP_SAVE'})
    # v5.1：一格能挂多个集合（左臂 = Arm.L (IK) + Arm.L (FK)）
    append: bpy.props.BoolProperty(
        name="加进这一格", default=False, options={'SKIP_SAVE'})
    # v5.2.14：从格内摘掉一个集合（保留其余成员），配合「这一格」小节用
    remove: bpy.props.BoolProperty(
        default=False, options={'HIDDEN', 'SKIP_SAVE'})
    # 集合名是中文，交给 EnumProperty 当 identifier 会在下拉里显示成乱码
    # （第一项还是空 identifier，更要命），所以这里用 StringProperty + 自绘
    # 弹窗列表，标识符只走整数 slot，绝不拿集合名当 ID。
    filter: bpy.props.StringProperty(
        name="过滤", default="",
        description="输入关键字筛选集合（如 arm / 头 / 武器）",
    )
    from_popup: bpy.props.BoolProperty(
        default=False, options={'HIDDEN', 'SKIP_SAVE'})

    MAX_ROWS = 14

    @classmethod
    def poll(cls, context):
        return pie_menu_poll(context)

    def _item(self, lay, nm, text, icon):
        r = lay.row(align=True)
        op = r.operator("pose.cn_pie_assign_v45", text=text, icon=icon)
        op.slot = self.slot
        op.layer = self.layer
        op.collection = nm
        op.from_popup = True
        # v5.2.21：行尾 ＝ 把集合**追加**进这一格 —— 之前格内能 ✕ 摘除、
        # 列表行却只能整格替换，用户「不知道怎么把集合加进去」。
        opa = r.operator("pose.cn_pie_assign_v45", text="", icon='ADD')
        opa.slot = self.slot
        opa.layer = self.layer
        opa.collection = nm
        opa.append = True
        opa.from_popup = True

    def _draw_body(self, lay, context):
        idx, layer = self.slot, self.layer
        lname = dict((l, n) for l, n, _k, _i in PIE_LAYERS).get(layer, "")
        rig = pie_rig(context)
        lay.label(text="%s · %s（%s）"
                  % (lname, PIE_DIRS[idx], PIE_DIR_HINTS[idx]),
                  icon='MESH_CIRCLE')
        lay.prop(self, "filter", text="", icon='VIEWZOOM')
        if rig is None:
            lay.label(text="先选中一根骨架", icon='ERROR')
            return

        flt = (self.filter or "").strip().lower()

        # v5.2.19：文件里不止一套骨架时把本弹窗绑的是谁挑明 —— 元骨/成品骨/
        # 表情骨混在一个 blend 里，绑错了用户根本看不出来（只会觉得「明明
        # 有集合怎么弹窗里没东西」）。
        if rig is not None:
            try:
                n_arms = sum(1 for o in bpy.data.objects
                             if o.type == 'ARMATURE')
            except Exception:
                n_arms = 1
            if n_arms > 1:
                lay.label(text="骨架：%s" % rig.name, icon='ARMATURE_DATA')

        # v5.2.14：「这一格」小节 —— 一格挂多个集合时（格名只显示第 1 个
        # 再缀 +N），这里把整组摊开看清楚：排 1 的是点进去先选中的，
        # 后面每个带 ✕，点一下只摘这一个、其余不动。
        try:
            grp = [n for n in pie_slot_groups(rig, layer)[idx] if n]
        except Exception:
            grp = []
        if grp:
            lay.separator()
            head = lay.row()
            head.label(text="这一格挂了 %d 个集合" % len(grp), icon='BOOKMARKS')
            for i, nm in enumerate(grp):
                r2 = lay.row(align=True)
                col = _coll_get(rig, nm)
                cnt = len(col.bones) if col else 0
                icon = 'LAYER_ACTIVE' if i == 0 else 'BLANK1'
                r2.label(text="%s%s · %d 根" % (
                    "① " if i == 0 else "%d. " % (i + 1), nm, cnt), icon=icon)
                opx = r2.operator("pose.cn_pie_assign_v45", text="",
                                  icon='X')
                opx.slot = idx
                opx.layer = layer
                opx.collection = nm
                opx.remove = True
                opx.from_popup = True
            lay.separator()

        native, internal = assign_collection_rows(rig)
        if native:
            hits = [r for r in native if not flt or flt in r[0].lower()]
            for nm, text, icon in hits[:self.MAX_ROWS]:
                self._item(lay, nm, text, icon)
            if len(hits) > self.MAX_ROWS:
                lay.label(text="还有 %d 个 —— 输入关键字筛选"
                          % (len(hits) - self.MAX_ROWS), icon='INFO')
        else:
            lay.label(text="这根骨架没有任何骨骼集合", icon='ERROR')
        # v5.2.20：内部集合（ORG/MCH/DEF）段按用户要求去掉 —— v5.2.19 真正的
        # 病根是 5.x 的 collections 只剩顶层、子集合全漏，那批「没东西」靠
        # _colls() 修好了，这个补丁段失去了存在意义，只添堵。

        # 清空：只有这格真有内容才显示，点了直接写空并关窗
        if any(pie_slot_groups(rig, layer)[idx]):
            lay.separator()
            op = lay.operator("pose.cn_pie_assign_v45", text="清空这一格",
                              icon='X')
            op.slot = idx
            op.layer = layer
            op.collection = ""
            op.from_popup = True

    def invoke(self, context, event):
        if self.from_popup:
            # 弹窗里点行：直接写入并关窗，绝不再 invoke 开新弹窗
            return self.execute(context)
        rig = pie_rig(context)
        if rig is None:
            return {'CANCELLED'}
        self.filter = ""
        wm = context.window_manager
        try:
            return wm.invoke_popup(self, width=300)
        except Exception:
            # 极老版本没有 invoke_popup：退回菜单，至少功能还在
            wm.popup_menu(self._draw_menu,
                          title="%s · %s" % (
                              dict((l, n) for l, n, _k, _i in PIE_LAYERS
                                   ).get(self.layer, "饼菜单"),
                              PIE_DIRS[self.slot]))
            return {'FINISHED'}

    def draw(self, context):
        self._draw_body(self.layout, context)

    def _draw_menu(self, menu, context):
        self._draw_body(menu.layout, context)

    def execute(self, context):
        rig = pie_rig(context)
        if rig is None or not edit_ok(rig):
            return {'CANCELLED'}
        if self.remove and self.collection:
            # 只摘掉这一个成员，其余保留（摘光 = 清空这一格）
            cur = [n for n in pie_slot_groups(rig, self.layer)[self.slot]
                   if n and n != self.collection]
            set_pie_slot(rig, self.slot, "|".join(cur), self.layer)
            self.report({'INFO'}, "已从格内移除 %s" % self.collection)
        elif self.append and self.collection:
            cur = [n for n in pie_slot_groups(rig, self.layer)[self.slot] if n]
            if self.collection not in cur:
                cur.append(self.collection)
            set_pie_slot(rig, self.slot, "|".join(cur), self.layer)
        else:
            set_pie_slot(rig, self.slot, self.collection, self.layer)
        if self.append and self.collection:
            self.report({'INFO'}, "已把 %s 加进这一格（共 %d 个）"
                        % (self.collection,
                           len([n for n in pie_slot_groups(
                               rig, self.layer)[self.slot] if n])))
        else:
            self.report({'INFO'}, "%s %s → %s"
                        % (dict((l, n) for l, n, _k, _i in PIE_LAYERS).get(
                            self.layer, "饼"),
                           PIE_DIRS[self.slot], self.collection or "（空）"))
        if self.from_popup:
            # 小窗里点完就关：模拟一次 Esc（插件在 rigui 弹窗上用的同款手法）
            try:
                win = context.window
                win.event_simulate(type='ESC', value='PRESS')
                win.event_simulate(type='ESC', value='RELEASE')
            except Exception:
                pass
        # v5.2.15：写完立刻重绘侧栏 —— invoke_popup 弹窗不会带着侧栏重绘，
        # 格子明明写进去了面板还停在（空），看起来像没生效，用户只能反复
        # 重开弹窗重试（「一直弹这个」的真相）。
        try:
            for area in context.window.screen.areas:
                if area.type == 'VIEW_3D':
                    area.tag_redraw()
        except Exception:
            pass
        return {'FINISHED'}


class POSE_OT_cn_slot_order_v45(bpy.types.Operator):
    """把一个格子里的集合往前/往后挪 —— 排第一的那个就是「点进去先选中」的"""
    bl_idname = "pose.cn_slot_order_v45"
    bl_label = "调整格内顺序"
    bl_options = {'REGISTER', 'UNDO'}

    slot: bpy.props.IntProperty(default=0, min=0, max=7)
    layer: bpy.props.IntProperty(default=1, min=1, max=3)
    collection: bpy.props.StringProperty()
    delta: bpy.props.IntProperty(default=-1)

    @classmethod
    def poll(cls, context):
        return pie_menu_poll(context)

    def execute(self, context):
        rig = pie_rig(context)
        if rig is None or not edit_ok(rig):
            return {'CANCELLED'}
        grp = list(pie_slot_groups(rig, self.layer)[self.slot])
        if len(grp) < 2 or self.collection not in grp:
            return {'CANCELLED'}
        i = grp.index(self.collection)
        j = max(0, min(len(grp) - 1, i + self.delta))
        if i == j:
            return {'CANCELLED'}
        grp.insert(j, grp.pop(i))
        set_pie_slot(rig, self.slot, "|".join(grp), self.layer)
        self.report({'INFO'}, "%s → 第 %d 位" % (self.collection, j + 1))
        return {'FINISHED'}


class POSE_OT_cn_slot_swap_v45(bpy.types.Operator):
    """把这一格的集合整体换到另一格（两格内容对调）"""
    bl_idname = "pose.cn_slot_swap_v45"
    bl_label = "交换两格"
    bl_options = {'REGISTER', 'UNDO'}

    layer: bpy.props.IntProperty(default=1, min=1, max=3)
    slot_a: bpy.props.IntProperty(default=0, min=0, max=7)
    slot_b: bpy.props.IntProperty(default=1, min=0, max=7)

    @classmethod
    def poll(cls, context):
        return pie_menu_poll(context)

    def execute(self, context):
        rig = pie_rig(context)
        if rig is None or not edit_ok(rig):
            return {'CANCELLED'}
        if self.slot_a == self.slot_b:
            return {'CANCELLED'}
        groups = pie_slot_groups(rig, self.layer)
        ga = list(groups[self.slot_a])
        gb = list(groups[self.slot_b])
        set_pie_slot(rig, self.slot_a, "|".join(gb), self.layer)
        set_pie_slot(rig, self.slot_b, "|".join(ga), self.layer)
        self.report({'INFO'}, "%s ↔ %s" % (PIE_DIRS[self.slot_a],
                                           PIE_DIRS[self.slot_b]))
        return {'FINISHED'}


class POSE_OT_cn_auto_layout_v45(bpy.types.Operator):
    """一键配置：读骨架原生集合，自动归类进三层饼菜单

    跟「重新自动填充」的区别：那个一次只填一层，这个三层一起定，
    而且会先算一整份归类再落盘，层与层之间不会抢集合。
    """
    bl_idname = "pose.cn_auto_layout_v45"
    bl_label = "一键配置饼菜单"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return (context.mode == 'POSE'
                and context.object is not None
                and context.object.type == 'ARMATURE')

    def execute(self, context):
        rig = context.object
        if rig is None or not edit_ok(rig):
            self.report({'WARNING'}, "这根骨架是只读的（库链接）")
            return {'CANCELLED'}
        slots, rep = auto_layout(rig, use_cache=False)
        if rep['placed'] == 0:
            self.report({'WARNING'}, "这根骨架里没有认出可用的集合")
            return {'CANCELLED'}
        for layer in (1, 2, 3):
            groups = slots.get(layer, [[] for _ in range(8)])
            for i in range(8):
                set_pie_slot_group(rig, layer, i, groups[i])
            rig.data[pie_init_key(layer)] = 1
        msg = "配好 %d 个：主饼 %d 格 / 细节 %d 格 / 附属 %d 格" % (
            rep['placed'], rep['layers'][1], rep['layers'][2], rep['layers'][3])
        if rep['skip']:
            msg += " · 排除 %d 个" % len(rep['skip'])
        if rep['overflow']:
            msg += " · %d 个没放下" % len(rep['overflow'])
        self.report({'INFO'}, msg)
        return {'FINISHED'}


class POSE_OT_cn_restore_visibility_v45(bpy.types.Operator):
    """把集合的显示/隐藏还原成「第一次进饼之前」的样子"""
    bl_idname = "pose.cn_restore_visibility_v45"
    bl_label = "还原原始显示"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return (context.object is not None
                and context.object.type == 'ARMATURE')

    def execute(self, context):
        rig = context.object
        n = restore_visibility(rig)
        if n < 0:
            self.report({'WARNING'}, "还没有快照 —— 先进一次饼菜单，或点「以当前为准」")
            return {'CANCELLED'}
        self.report({'INFO'}, "还原了 %d 个集合的显示状态" % n)
        return {'FINISHED'}


class POSE_OT_cn_snapshot_visibility_v45(bpy.types.Operator):
    """把当前显示状态记为新的还原基准（手动调好之后用）"""
    bl_idname = "pose.cn_snapshot_visibility_v45"
    bl_label = "以当前为准"
    bl_options = {'REGISTER'}

    @classmethod
    def poll(cls, context):
        return (context.object is not None
                and context.object.type == 'ARMATURE')

    def execute(self, context):
        rig = context.object
        if not save_visibility_snapshot(rig, force=True):
            self.report({'WARNING'}, "存不下来（骨架是只读的？）")
            return {'CANCELLED'}
        self.report({'INFO'}, "已记下 %d 个集合的当前状态"
                    % len(_colls(rig)))
        return {'FINISHED'}


class POSE_OT_cn_pie_autofill_v45(bpy.types.Operator):
    """按语义/前缀自动填饼菜单格子（layer=0 表示三层全填）"""
    bl_idname = "pose.cn_pie_autofill_v45"
    bl_label = "重新自动填充"
    bl_options = {'REGISTER', 'UNDO'}

    layer: bpy.props.IntProperty(name="饼层", default=0, min=0, max=3)

    @classmethod
    def poll(cls, context):
        return pie_menu_poll(context)

    def execute(self, context):
        rig = pie_rig(context)
        if rig is None or not edit_ok(rig):
            return {'CANCELLED'}
        layers = (1, 2, 3) if self.layer == 0 else (self.layer,)
        n = 0
        for ly in layers:
            n += len([s for s in init_pie_slots(rig, ly) if s])
        if n == 0:
            self.report({'WARNING'}, "这根骨架里没认出可用的集合，手动指定吧")
        else:
            self.report({'INFO'}, "自动填了 %d 格" % n)
        return {'FINISHED'}


class POSE_OT_cn_pie_clear_v45(bpy.types.Operator):
    """清空某一层饼菜单的 8 格（layer=0 表示三层全清）"""
    bl_idname = "pose.cn_pie_clear_v45"
    bl_label = "清空饼菜单"
    bl_options = {'REGISTER', 'UNDO'}

    layer: bpy.props.IntProperty(name="饼层", default=0, min=0, max=3)

    @classmethod
    def poll(cls, context):
        return pie_menu_poll(context)

    def execute(self, context):
        rig = pie_rig(context)
        if rig is None or not edit_ok(rig):
            return {'CANCELLED'}
        layers = (1, 2, 3) if self.layer == 0 else (self.layer,)
        for ly in layers:
            for k in pie_slot_keys(ly):
                rig.data[k] = ""
            rig.data[pie_init_key(ly)] = 1   # 记住"用户清空过"，别再自动填回来
        return {'FINISHED'}


class POSE_OT_cn_first_bone_v45(bpy.types.Operator):
    """把当前选中的骨骼设为所在集合的默认第一根（切进集合先落在它上面）"""
    bl_idname = "pose.cn_first_bone_v45"
    bl_label = "设为集合默认骨骼"
    bl_options = {'REGISTER', 'UNDO'}

    clear: bpy.props.BoolProperty(name="清除", default=False)

    @classmethod
    def poll(cls, context):
        return pie_menu_poll(context)

    def execute(self, context):
        rig = pie_rig(context)
        if rig is None or not edit_ok(rig):
            return {'CANCELLED'}
        col_name = rig.data.get("cn_collection", "") or ""
        if not col_name:
            self.report({'WARNING'}, "当前不在任何集合里，先用导航/饼菜单进一个集合")
            return {'CANCELLED'}
        key = "cn_first_" + col_name
        if self.clear:
            try:
                del rig.data[key]
            except Exception:
                pass
            self.report({'INFO'}, "已清除「%s」的默认骨骼（回到排序第一根）" % col_name)
            return {'FINISHED'}
        sel = selected_bones(rig)
        if len(sel) != 1:
            self.report({'WARNING'}, "先选中恰好一根骨骼（当前选了 %d 根）" % len(sel))
            return {'CANCELLED'}
        col = _coll_get(rig, col_name)
        if col is not None and sel[0].name not in collection_bone_names(rig, col):
            self.report({'WARNING'},
                        "%s 不属于集合「%s」，先把骨骼放进集合再设" % (sel[0].name, col_name))
            return {'CANCELLED'}
        rig.data[key] = sel[0].name
        self.report({'INFO'}, "「%s」的默认骨骼 = %s" % (col_name, sel[0].name))
        return {'FINISHED'}


# ---------------------------------------------------------------- 独显（solo）
#
# 只显示一个集合的控制器，其余骨骼集合关掉可见性。快照存在骨架自己身上：
#   rig.data["cn_solo_prev"] = {集合名: 当时的可见性}
#   rig.data["cn_solo_col"]  = 当前独显的集合名
# 所以「关掉再打开文件」也能还原回你原来的显示状态。

_SOLO_COL_KEY = "cn_solo_col"
_SOLO_COLS_KEY = "cn_solo_cols"      # v5.2.40：多集合独显（| 分隔，第一个是主控）
_SOLO_PREV_KEY = "cn_solo_prev"


def solo_collection_name(rig):
    """主控集合名（多选时是第一个）—— 老判断靠它，保持单值语义。"""
    try:
        return str(rig.data.get(_SOLO_COL_KEY, "") or "")
    except Exception:
        return ""


def solo_names(rig):
    """当前独显的集合名列表（保序）。新键优先，旧单值键兜底。

    Blender 原生骨骼集合的 is_solo 本来就是**并集**语义（实测 5.2.3：
    两个集合同时打星 → 两组骨骼都可见、其余全隐），所以多部位独显
    不需要自己算可见性，只要把星打在一组集合上。
    """
    try:
        raw = str(rig.data.get(_SOLO_COLS_KEY, "") or "")
    except Exception:
        raw = ""
    if raw:
        return [n for n in raw.split("|") if n]
    one = solo_collection_name(rig)
    return [one] if one else []


def solo_active(rig):
    """当前是否有集合打了 ★（原生 solo 正在生效）。

    solo 是**最高优先级**的可见性遮罩：它生效期间，往任何集合写 is_visible
    都只是改标志、屏幕不会变；真正会发生的副作用只剩「把星标清光」。
    所以「solo 期间该不该做可见性操作」必须单独判断 —— 否则每一个顺手
    调 show_collection 的路径（跳集合、换饼、导航取骨）都会静默拆掉独显。
    """
    try:
        for c in _colls(rig):
            if c.is_solo:
                return True
    except Exception:
        return False
    return False


def solo_covers(rig, name):
    """name 是否已被现有独显覆盖（它就是某个星标集合，或它的子孙）。

    solo 是「集合 + 全部子集合」一起亮，所以祖先打了星时，子集合本来就看得见，
    不需要再给它补星 —— 补了反而会改变可见范围。
    """
    if not name:
        return False
    for n in solo_names(rig):
        c = _coll_get(rig, n)
        if c is None:
            continue
        try:
            for c2 in solo_tree(c):
                if c2.name == name:
                    return True
        except Exception:
            continue
    return False


def solo_tree(col):
    """集合自己 + 所有子集合（Rigify 的集合是分层的）。"""
    out = [col]
    stack = list(col.children)
    n = 0
    while stack and n < 256:
        c = stack.pop()
        n += 1
        out.append(c)
        stack.extend(c.children)
    return out


def solo_set(rig, names, commit=False):
    """多集合独显：把 names（集合名列表）**连同各自子集合**一起打星。

    原生 solo：一个集合的 is_visible 都不动 —— 取消星标后可见性由
    Blender 自动还原，侧栏面板上也能直接看到星标亮在哪。范围保持旧
    语义：每个目标集合 + 全部子集合一起标星（Hair 这类 0 骨分组、
    以及 Rigify 分层，纯原生只标自己会「一颗星下去什么都看不见」）。

    names 传空列表 = 清空独显。
    """
    if isinstance(names, str):
        names = [names]
    names = [n for n in (names or []) if n]
    keep = set()
    for n in names:
        col = _coll_get(rig, n)
        if col is None:
            continue
        keep |= set(c.name for c in solo_tree(col))
    if names and not keep:
        return False
    try:
        for c in _colls(rig):
            want = c.name in keep
            if bool(c.is_solo) != want:
                c.is_solo = want
    except Exception:
        return False
    if commit:
        try:
            if names:
                rig.data[_SOLO_COLS_KEY] = "|".join(names)
                rig.data[_SOLO_COL_KEY] = names[0]
            else:
                for k in (_SOLO_COLS_KEY, _SOLO_COL_KEY):
                    if k in rig.data:
                        del rig.data[k]
        except Exception:
            pass
    return True


def solo_apply(rig, col_name, commit=False):
    """单集合入口（老签名）——内部转到 solo_set，参数也接受列表。"""
    return solo_set(rig, col_name, commit)


def solo_clear(rig, restore=True):
    # 原生 solo：清掉星标即可 —— 各集合的 is_visible 从头到尾没被碰过，
    # Blender 自己把显示还原回去，不需要快照。restore 参数保留旧签名。
    try:
        for c in _colls(rig):
            if c.is_solo:
                c.is_solo = False
    except Exception:
        pass
    # 老版本把可见性快照存在 data 里，顺手清掉历史残留
    for k in (_SOLO_PREV_KEY, _SOLO_COL_KEY, _SOLO_COLS_KEY):
        try:
            if k in rig.data:
                del rig.data[k]
        except Exception:
            pass
    return True


def solo_toggle(rig, col_name, report=None):
    """点一下只显示这个集合，再点一下（或点别的集合）还原 / 换一个。"""
    if not col_name:
        return ""
    if solo_names(rig) == [col_name]:
        solo_clear(rig)
        return "restore"
    solo_apply(rig, col_name, commit=True)
    return "solo"


class POSE_OT_cn_solo_collection_v45(bpy.types.Operator):
    """独显这个集合的控制器（再点一次还原）"""
    bl_idname = "pose.cn_solo_collection_v45"
    bl_label = "独显集合"
    bl_options = {'REGISTER', 'UNDO'}

    collection: bpy.props.StringProperty()
    restore: bpy.props.BoolProperty(default=False)

    @classmethod
    def poll(cls, context):
        return pie_menu_poll(context)

    def execute(self, context):
        rig = pie_rig(context)
        if rig is None or not edit_ok(rig):
            return {'CANCELLED'}
        if self.restore:
            solo_clear(rig)
            self.report({'INFO'}, "已还原原来的显示")
            return {'FINISHED'}
        res = solo_toggle(rig, self.collection, self.report)
        if res == "restore":
            self.report({'INFO'}, "已还原原来的显示")
        elif res == "solo":
            self.report({'INFO'}, "只显示「%s」" % self.collection)
        else:
            return {'CANCELLED'}
        return {'FINISHED'}


# ---- 星标饼（自绘）。原生饼菜单拿不到悬停事件，只有自己画才能做到
#      「鼠标移到星标上就独显」。默认关掉：偏好里勾「星标饼」才接管热键，
#      出问题随时切回原生饼（键位照样用）。

_WHEEL_R_IN = 66.0
_WHEEL_R_OUT = 168.0
_WHEEL_STAR_GAP = 18.0
_WHEEL_STAR_R = 9.0
# 触发死区（像素）：超过它就按方向选中，**不用**滑到格子上。
# 原生饼十几像素就高亮，这里默认 12，偏好里可调（0 = 一动就选）。
_WHEEL_DEAD = 12.0
_WHEEL_FONT = 13


def _wheel_geo():
    """从偏好取星标饼的几何参数（画和命中判定共用一份，免得对不上）。"""
    try:
        r_out = float(max(80, int(pref("pie_radius", _WHEEL_R_OUT))))
        r_in = float(max(8, int(pref("pie_hole", _WHEEL_R_IN))))
        if r_in > r_out - 36.0:
            r_in = max(8.0, r_out - 36.0)
        star_r = float(max(3, int(pref("pie_star_size", _WHEEL_STAR_R))))
        dead = float(max(0, min(120, int(pref("pie_dead_zone", _WHEEL_DEAD)))))
        font = max(8, min(24, int(pref("pie_label_size", _WHEEL_FONT))))
    except Exception:
        r_out, r_in, star_r, dead, font = (_WHEEL_R_OUT, _WHEEL_R_IN,
                                          _WHEEL_STAR_R, _WHEEL_DEAD, _WHEEL_FONT)
    return {"r_in": r_in, "r_out": r_out, "gap": _WHEEL_STAR_GAP,
            "star_r": star_r, "dead": dead, "font": font}


def _wheel_sector(mx, my, cx, cy, count=8):
    """(扇区号, 半径)。扇区 0 = 正上方，顺时针。"""
    dx, dy = mx - cx, my - cy
    r = math.hypot(dx, dy)
    if r < 1.0:
        return -1, r
    ang = math.degrees(math.atan2(dy, dx))
    step = 360.0 / count
    idx = int(round((90.0 - ang) / step)) % count
    return idx, r


def _wheel_star_r(geo):
    """星标命中环带的内边界（外边界见 _wheel_slot_polar）。

    口径就是「方框外围那圈星」：扇区外沿往外到星标外侧再多一点。
    内圈（扇区身上）一律不算命中 —— 否则刚开饼挪一下鼠标就误触。
    """
    return geo["r_out"] - 1.0


def _wheel_slot_polar(cx, cy, ang_deg, r, geo, star_only=False):
    """极坐标反推扇区号；不在有效环带内返回 -1（与 modal 的判定同源）。"""
    if star_only:
        if r < _wheel_star_r(geo):
            return -1
    elif r < geo["dead"]:
        return -1
    if r > geo["r_out"] + geo["gap"] + geo["star_r"] + 12:
        return -1
    idx = int(round((90.0 - ang_deg) / 45.0)) % 8
    return idx


def _wheel_sweep(ax, ay, bx, by, cx, cy, geo, last=-1, star_only=False):
    """两点之间「扫过」的扇区序列（不含起点所在格，相邻不重复）。

    为什么需要它：modal 拿到的 MOUSEMOVE 是离散的，快速划动一帧能跨过
    两三个格子；直接按当前帧所在扇区判断，表现就是「扫过三个只锁到一个」。
    这里按角度插值补采样（7.5° 一步）。

    star_only=True 时只有**外圈星标环带**算命中，扇区身上不算 —— 这是
    「必须滑到方框外的星标才开始独显」的实现（误触太多）。

    两种情形不插值：
      · 任一端不在命中环带里 —— 从中心往外划，本来就没「扫过」这回事；
      · 两点连线穿过中心孔 —— 那是「从 A 直拉到 B」，不是沿弧扫过，
        插值会把中间两格也锁上。
    """
    dx0, dy0 = ax - cx, ay - cy
    dx1, dy1 = bx - cx, by - cy
    r0 = math.hypot(dx0, dy0)
    r1 = math.hypot(dx1, dy1)
    a0 = math.degrees(math.atan2(dy0, dx0))
    a1 = math.degrees(math.atan2(dy1, dx1))
    lo = _wheel_star_r(geo) if star_only else geo["dead"]

    def _one(ang, rad):
        s = _wheel_slot_polar(cx, cy, ang, rad, geo, star_only)
        # 兜底分支也要过滤「还是刚才那一格」，否则移出内圈再回同格会误解锁
        return [s] if (s >= 0 and s != last) else []

    if r0 < lo or r1 < lo:
        return _one(a1, r1)
    seg = math.hypot(bx - ax, by - ay)
    if seg > 1e-6:
        d = abs((bx - ax) * (ay - cy) - (by - ay) * (ax - cx)) / seg
        # 「穿过中心孔」一律按死区半径判 —— 外圈两点的弦离圆心也有一百多
        # 像素，用星标内径当基准会把正常的外圈扫动误判成直穿中心
        if d < geo["dead"] * 0.9:
            return _one(a1, r1)

    da = (a1 - a0 + 180.0) % 360.0 - 180.0
    n = int(abs(da) / 7.5) + 1
    if n > 48:
        n = 48
    out = []
    for k in range(1, n + 1):
        t = k / float(n)
        s = _wheel_slot_polar(cx, cy, a0 + da * t, r0 + (r1 - r0) * t, geo, star_only)
        if s < 0 or s == last:
            continue
        out.append(s)
        last = s
    return out


def _sector_tris(cx, cy, a0, a1, r0, r1, seg=8):
    out = []
    for k in range(seg):
        t0 = a0 + (a1 - a0) * k / float(seg)
        t1 = a0 + (a1 - a0) * (k + 1) / float(seg)
        p0 = (cx + r0 * math.cos(t0), cy + r0 * math.sin(t0))
        p1 = (cx + r1 * math.cos(t0), cy + r1 * math.sin(t0))
        p2 = (cx + r1 * math.cos(t1), cy + r1 * math.sin(t1))
        p3 = (cx + r0 * math.cos(t1), cy + r0 * math.sin(t1))
        out += [p0, p1, p2, p0, p2, p3]
    return out


def _star_tris(cx, cy, r_out, r_in, points=5):
    tips = []
    for k in range(points * 2):
        ang = math.pi / 2 + k * math.pi / points
        r = r_out if k % 2 == 0 else r_in
        tips.append((cx + r * math.cos(ang), cy + r * math.sin(ang)))
    tris = []
    for k in range(points * 2):
        tris += [(cx, cy), tips[k], tips[(k + 1) % (points * 2)]]
    return tris


class POSE_OT_cn_wheel_v45(bpy.types.Operator):
    """星标饼：8 格集合，每格外沿一个星标 —— 松手在哪就生效在哪

    一套拖拽手势（对齐原生饼）：按住给出**方向**就高亮（十几像素即触发，
    不用滑到格子上，死区可调），松手 → 跳进集合并选中第一根骨；滑到外沿
    星星 → 当场独显（预览），滑回格子 → 退出独显，松手在星星上 → 独显
    定格。松手即退出饼。空格子 / 拖出饼外松手只关饼，不弹任何面板。
    画不出来就自动退回原生饼，键位不会失效。
    """
    bl_idname = "pose.cn_wheel_v45"
    bl_label = "星标饼"
    bl_options = {'REGISTER'}

    layer: bpy.props.IntProperty(default=1, min=1, max=3)

    def _native(self, context):
        menu = {1: "CN_MT_pie_v45", 2: "CN_MT_pie2_v45", 3: "CN_MT_pie3_v45"}.get(self.layer)
        if not menu:
            return {'CANCELLED'}
        try:
            bpy.ops.wm.call_menu_pie('INVOKE_DEFAULT', name=menu)
        except Exception:
            pass
        return {'CANCELLED'}

    def invoke(self, context, event):
        try:
            area = context.area
            if area is None or area.type != 'VIEW_3D':
                return self._native(context)
            if getattr(event, "mouse_region_x", None) is None:
                return self._native(context)
            if context.window is None:
                return self._native(context)
        except Exception:
            return self._native(context)
        self._center = (event.mouse_region_x, event.mouse_region_y)
        self._hover = -1
        self._star_hover = -1
        self._preview = None      # 拖拽期间预览独显的集合名（滑回格子就还原）
        self._slots = []
        self._names = []
        self._handle = None
        # v5.2.42：C3 语义 —— **不按任何键**。
        #   扫一颗 = 只独显这一颗（替换，和 v5.2.39 以前完全一致）；
        #   扫多颗 = 开饼时已有的星 ∪ 本次扫过的（累加）。
        #   之所以彻底不用修饰键：饼本身是 Alt+左键 / Alt+Shift+左键 触发的，
        #   任何修饰键都可能既是触发键又是模式键 —— 细节饼因此永远退不出累加态。
        self._last = (event.mouse_region_x, event.mouse_region_y)
        self._last_r = 0.0           # 上一帧离饼心的半径（判断是否从内圈重新瞄准）
        self._locks = []             # 本次扫过的格（有序）
        self._snap = []              # 开饼前已有的星标（名字）
        self._snap_slots = []        # 上面那些星在本层的格位（只用于画灰星）
        self._sweep = -1
        self._applied = None         # 上一次真正落场景的集合组合，用来去重
        self._rcache = {}
        self._load_slots(context)
        try:
            rig0 = pie_rig(context)
        except Exception:
            rig0 = None
        if rig0 is not None:
            try:
                self._snap = solo_names(rig0)
            except Exception:
                self._snap = []
            if self._snap:
                # 只在画面上标出来（灰星 = 当前独显、但不是本次选中的），
                # **不参与本次选中集** —— 否则「单颗 = 替换」就退化成累加了
                want = set(self._snap)
                for i in range(8):
                    if self._names[i] and self._resolved(i, rig0) in want:
                        self._snap_slots.append(i)
        try:
            self._handle = bpy.types.SpaceView3D.draw_handler_add(
                self._draw, (), 'WINDOW', 'POST_PIXEL')
        except Exception:
            return self._native(context)
        context.window_manager.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def _load_slots(self, context):
        rig = pie_rig(context)
        self._names = []
        self._slots = []
        if rig is None or rig.type != 'ARMATURE':
            return
        try:
            groups = pie_slot_groups(rig, self.layer)
        except Exception:
            groups = []
        for i in range(8):
            grp = groups[i] if i < len(groups) else []
            name = grp[0] if grp else ""
            self._names.append(name)
            self._slots.append(name)

    # ---- v5.2.40：多部位独显 -------------------------------------------------
    def _resolved(self, slot, rig):
        """格子名 → 当前 IK/FK 态下**实际显示**的集合名（带缓存）。

        每帧都对 8 格调 _resolve_mode_collection 会遍历骨骼名，首次开饼
        就掉十几毫秒；格子在整个模态里不变，缓存一次即可。
        """
        if slot in self._rcache:
            return self._rcache[slot]
        nm = self._names[slot] if 0 <= slot < len(self._names) else ""
        if not nm or rig is None:
            self._rcache[slot] = ""
            return ""
        try:
            pv = _resolve_mode_collection(rig, nm)
        except Exception:
            pv = nm
        self._rcache[slot] = pv or ""
        return self._rcache[slot]

    def _lock_names(self, rig):
        out = []
        for s in self._locks:
            nm = self._resolved(s, rig)
            if nm and nm not in out:
                out.append(nm)
        return out

    def _commit_names(self, rig):
        """C3：本次扫过的格决定最终结果。

        扫一颗 → 只独显这一颗（替换；老行为一字不变）；
        扫多颗 → 开饼时已有的星 ∪ 本次扫过的（累加）。
        「扫过」是加法，「单颗」是替换 —— 这样既能顺手加一块，也不会
        因为开饼时本来就有星，就把「只想看这一个」变成「全留下来」。
        """
        locked = self._lock_names(rig)
        if len(locked) <= 1:
            return locked
        out = []
        for nm in list(self._snap or []) + locked:
            if nm and nm not in out:
                out.append(nm)
        return out

    def _apply_preview(self, rig):
        """落场景（不写键）。同一组集合不重复写，省得每帧刷可见性。"""
        if rig is None:
            return
        names = self._commit_names(rig)
        key = tuple(names)
        if key == self._applied:
            return
        self._applied = key
        try:
            if names:
                solo_set(rig, names, commit=False)
            else:
                solo_set(rig, self._snap, commit=False)
        except Exception:
            pass
        self._preview = "|".join(names) if names else None

    def _toggle_lock(self, slot, rig, add_only=False):
        """切换一格的锁。add_only=True 只加不减（从中心区重新瞄准时用）。"""
        if slot < 0 or slot >= 8 or not self._names[slot]:
            return False
        if slot in self._locks:
            if add_only:
                return False
            self._locks.remove(slot)
        else:
            # 同一个集合可能占了两格（自动布局会重复），锁新的就把旧的让出来，
            # 否则中心计数和实际独显的集合数对不上
            nm = self._resolved(slot, rig)
            if nm:
                for other in list(self._locks):
                    if self._resolved(other, rig) == nm:
                        self._locks.remove(other)
            self._locks.append(slot)
        self._apply_preview(rig)
        return True

    def _restore_snap(self, rig):
        """退出时按开饼前的星标恢复 —— 预览过再 ESC 也不会把原来的星弄丢。"""
        if rig is None:
            return
        self._applied = None
        try:
            solo_set(rig, self._snap, commit=False)
        except Exception:
            pass

    def _draw(self):
        try:
            import gpu
            import blf
            from gpu_extras.batch import batch_for_shader
        except Exception:
            return
        cx, cy = self._center
        try:
            shader = gpu.shader.from_builtin('UNIFORM_COLOR')
        except Exception:
            return
        geo = _wheel_geo()
        r_in, r_out = geo["r_in"], geo["r_out"]
        try:
            gpu.state.blend_set('ALPHA')
            # 中心孔里的方向扇：鼠标只在死区里给出方向时就靠它反馈
            # （原生饼十几像素就高亮，不用真滑到格子上）
            if self._hover >= 0 and self._star_hover < 0:
                mid = math.radians(90.0 - self._hover * 45.0)
                a0, a1 = mid - math.radians(22.5), mid + math.radians(22.5)
                wv = _sector_tris(cx, cy, a0, a1, max(6.0, geo["dead"] * 0.8), r_in)
                wb = batch_for_shader(shader, 'TRIS', {"pos": wv})
                shader.bind()
                shader.uniform_float("color", (0.24, 0.40, 0.68, 0.55))
                wb.draw(shader)
            locks = getattr(self, "_locks", [])
            snap_slots = getattr(self, "_snap_slots", [])
            for i in range(8):
                mid = math.radians(90.0 - i * 45.0)
                a0, a1 = mid - math.radians(22.5), mid + math.radians(22.5)
                locked = i in locks
                # 开饼时就独显、但本次没扫到 —— 灰金色，提示「松手它会掉」
                kept = (i in snap_slots) and not locked
                if locked:
                    col = ((0.60, 0.44, 0.10, 0.96) if i == self._hover
                           else (0.46, 0.34, 0.09, 0.96))
                elif kept:
                    col = (0.26, 0.21, 0.11, 0.94)
                elif i == self._hover:
                    col = (0.24, 0.40, 0.68, 0.96)
                else:
                    col = (0.16, 0.17, 0.20, 0.92)
                verts = _sector_tris(cx, cy, a0, a1, r_in, r_out)
                batch = batch_for_shader(shader, 'TRIS', {"pos": verts})
                shader.bind()
                shader.uniform_float("color", col)
                batch.draw(shader)
                if not self._slots[i]:
                    continue
                sx = cx + (r_out + geo["gap"]) * math.cos(mid)
                sy = cy + (r_out + geo["gap"]) * math.sin(mid)
                sr = geo["star_r"] * (1.35 if locked else 1.0)
                star = _star_tris(sx, sy, sr, sr * 0.42, 5)
                sb = batch_for_shader(shader, 'TRIS', {"pos": star})
                if locked:
                    sc = (1.0, 0.86, 0.42, 1.0)
                elif kept:
                    sc = (0.80, 0.64, 0.30, 0.95)
                else:
                    sc = (0.66, 0.66, 0.70, 1.0)
                shader.bind()
                shader.uniform_float("color", sc)
                sb.draw(shader)
                txt = self._slots[i]
                if len(txt) > 12:
                    txt = txt[:11] + "…"
                blf.size(0, geo["font"])
                if locked:
                    blf.color(0, 1.0, 0.93, 0.74, 1.0)
                elif kept:
                    blf.color(0, 0.86, 0.80, 0.66, 1.0)
                else:
                    blf.color(0, 0.92, 0.93, 0.95, 1.0)
                tx = cx + (r_in + r_out) * 0.5 * math.cos(mid)
                ty = cy + (r_in + r_out) * 0.5 * math.sin(mid)
                w, h = blf.dimensions(0, txt)
                blf.position(0, tx - w * 0.5, ty - h * 0.5, 0)
                blf.draw(0, txt)
            if locks:
                # 中心孔报数：多选时一眼看到锁了几个
                num = "%d" % len(locks)
                blf.size(0, max(14, geo["font"] + 6))
                blf.color(0, 1.0, 0.85, 0.45, 1.0)
                w, h = blf.dimensions(0, num)
                blf.position(0, cx - w * 0.5, cy - h * 0.5, 0)
                blf.draw(0, num)
            gpu.state.blend_set('NONE')
        except Exception:
            try:
                gpu.state.blend_set('NONE')
            except Exception:
                pass

    def _end(self, context):
        if self._handle is not None:
            try:
                bpy.types.SpaceView3D.draw_handler_remove(self._handle, 'WINDOW')
            except Exception:
                pass
            self._handle = None
        return

    def _commit(self, context):
        """定格本次扫选（C3 收尾）。"""
        self._end(context)
        try:
            rig = pie_rig(context)
        except Exception:
            rig = None
        if rig is None:
            return {'FINISHED'}
        names = self._commit_names(rig)
        if not names:
            self._restore_snap(rig)
            return {'FINISHED'}
        try:
            if sorted(names) == sorted(self._snap):
                # 和开饼前一模一样 → 按「再点一次还原」处理，别留一个空动作
                # （单颗扫过已经独显的那一格 = 关掉它，老肌肉记忆还在）
                solo_clear(rig)
            else:
                solo_set(rig, names, commit=True)
        except Exception:
            pass
        self._locks = []
        self._preview = None
        return {'FINISHED'}

    def modal(self, context, event):
        try:
            if event.type == 'MOUSEMOVE':
                mx, my = event.mouse_region_x, event.mouse_region_y
                geo = _wheel_geo()
                px, py = self._last
                prev_r = self._last_r
                self._last = (mx, my)
                idx, r = _wheel_sector(mx, my, self._center[0], self._center[1])
                self._last_r = r
                self._r = r
                star_r = _wheel_star_r(geo)
                r_max = geo["r_out"] + geo["gap"] + geo["star_r"] + 12
                if r < geo["dead"] or r > r_max:
                    idx = -1            # 中心死区 / 拖太远 = 不选
                # ★ 只有**外圈星标环带**才算命中；扇区身上一律不算。
                #   用户实测：刚开饼挪一下鼠标就被独显，太灵敏。
                star = idx if (idx >= 0 and r >= star_r) else -1
                changed = (idx != self._hover) or (star != self._star_hover)
                self._hover, self._star_hover = idx, star
                try:
                    rig = pie_rig(context)
                except Exception:
                    rig = None
                if r < geo["dead"] and self._locks:
                    # 收回中心孔 = 清空本次选中。这是唯一的「反悔」手势，
                    # 不用按任何键；手停在中心孔里松手也等于什么都没选。
                    self._locks = []
                    self._sweep = -1
                    self._apply_preview(rig)
                    changed = True
                if rig is not None and star >= 0:
                    # 沿轨迹补采样防漏格（一帧跨两三格很常见）。
                    # 上一帧还在内圈 = 重新瞄准，这一把只加不减，免得手抖
                    # 移出去再移回同一格就把锁解了；沿着星标环再划回来才是撤销。
                    add_only = prev_r < star_r
                    for s in _wheel_sweep(px, py, mx, my,
                                          self._center[0], self._center[1],
                                          geo, self._sweep, star_only=True):
                        if self._toggle_lock(s, rig, add_only):
                            changed = True
                    self._sweep = star
                if changed and context.area is not None:
                    context.area.tag_redraw()
                return {'RUNNING_MODAL'}
            if event.type in {'WHEELUPMOUSE', 'WHEELDOWNMOUSE'}:
                self._end(context)
                # 换层：直接重开，热键手感一致
                try:
                    self._restore_snap(pie_rig(context))
                except Exception:
                    pass
                self._locks = []
                self._preview = None
                self._sweep = -1
                self._rcache = {}
                nxt = self.layer + (1 if event.type == 'WHEELUPMOUSE' else -1)
                if nxt < 1:
                    nxt = 3
                if nxt > 3:
                    nxt = 1
                self.layer = nxt
                self._load_slots(context)
                try:
                    self._handle = bpy.types.SpaceView3D.draw_handler_add(
                        self._draw, (), 'WINDOW', 'POST_PIXEL')
                except Exception:
                    return {'CANCELLED'}
                return {'RUNNING_MODAL'}
            if event.type == 'LEFTMOUSE':
                if event.value == 'PRESS':
                    return {'RUNNING_MODAL'}
                # 扫过星标 = 本次选中，直接按 C3 规则定格
                if self._locks:
                    return self._commit(context)
                hover = self._hover
                self._end(context)
                rig = pie_rig(context)
                if rig is not None and hover >= 0:
                    nm = self._names[hover] if hover < len(self._names) else ""
                    if nm:
                        # 松手即选中：跳进集合并选第一根（顺序表指定者优先）
                        enter_pie_collection(context, rig, nm, self.layer)
                        return {'FINISHED'}
                # 空格子 / 拖出饼外松手：直接关，不弹任何面板
                return {'CANCELLED'}
            if event.type in {'RIGHTMOUSE', 'ESC', 'MIDDLEMOUSE'}:
                self._end(context)
                try:
                    self._restore_snap(pie_rig(context))
                except Exception:
                    pass
                self._locks = []
                self._preview = None
                return {'CANCELLED'}
        except Exception:
            try:
                self._end(context)
            except Exception:
                pass
            try:
                self._restore_snap(pie_rig(context))
            except Exception:
                pass
            return self._native(context)
        return {'RUNNING_MODAL'}


def _pie_draw_layer(self, context, layer):
    """三层饼共用的绘制主体。

    ★ 扇区必须挂在 layout.menu_pie() 返回的子布局上。直接往 self.layout
      里加 item，Blender 会把它们当成普通菜单一项一行地排在圆的**正中心**
      —— 现象就是"8 项全叠在一起"（v4.9.84 真机截图）。
    """
    pie = self.layout.menu_pie()
    _lname, _n, lkey, _ic = PIE_LAYERS[layer - 1]

    rig = pie_rig(context)
    if rig is None or getattr(rig, "type", None) != 'ARMATURE':
        # 没有骨架也凑满 8 项，别让用户对着一张空圈发懵
        for k in range(8):
            i = PIE_EMIT_ORDER[k]
            try:
                op = pie.operator("pose.cn_pie_note_v45",
                                  text="%s · 没有骨架" % PIE_DIRS[i], icon='ERROR')
                op.message = "先选中一根骨架（姿态模式），再 %s 拖动" % lkey
            except Exception:
                pass
        return

    # draw 是只读上下文：这里只读数据，任何写入都会抛异常并把整张饼截断。
    try:
        slots = pie_slots(rig, layer)
        cur = pie_current_collection(rig)
        cols = _colls(rig)
    except Exception:
        slots, cur, cols = [""] * 8, "", None

    # 先把 8 项全算成纯数据，再按 Blender 的方向表顺序交给 pie。
    # 中途任何一项出问题都只让那一格变灰，不会把整张饼截断。
    items = []
    for k in range(8):
        i = PIE_EMIT_ORDER[k]          # 第 k 个绘制位置 ← PIE_DIRS 第 i 项
        name = slots[i] if i < len(slots) else ""
        if cols is None:
            items.append(("%s · 读不到骨架数据" % PIE_DIRS[i],
                          "pose.cn_pie_note_v45",
                          {"message": "这根骨架的数据读不出来（文件损坏或只读库）"},
                          'ERROR'))
        elif not name:
            items.append(("＋ %s" % PIE_DIRS[i], "pose.cn_pie_assign_v45",
                          {"slot": i, "layer": layer,
                           "collection": "", "from_popup": False,
                           "remove": False, "append": False}, 'ADD'))
        elif cols.get(name) is None:
            items.append(("%s · 集合已丢失" % name, "pose.cn_pie_assign_v45",
                          {"slot": i, "layer": layer,
                           "collection": "", "from_popup": False,
                           "remove": False, "append": False}, 'ERROR'))
        else:
            text = ("● " + name) if name == cur else name
            items.append((text, "pose.cn_pie_jump_v45",
                          {"collection": name, "layer": layer}, 'BONE_DATA'))

    for text, idname, props, icon in items:
        try:
            op = pie.operator(idname, text=text, icon=icon)
            for k, v in props.items():
                setattr(op, k, v)
        except Exception:
            # 兜底：宁可用提示项占住这个扇区，也不能少一格
            try:
                pie.operator("pose.cn_pie_note_v45", text=text, icon='ERROR')
            except Exception:
                pass


class CN_MT_pie_v45(bpy.types.Menu):
    """主饼：身体核心 8 个高频集合（Alt+左键）"""
    bl_idname = "CN_MT_pie_v45"
    bl_label = "主饼 · 身体核心"

    def draw(self, context):
        _pie_draw_layer(self, context, 1)


class CN_MT_pie2_v45(bpy.types.Menu):
    """细节饼：眼/耳/发束/指节（Shift+Alt+左键）"""
    bl_idname = "CN_MT_pie2_v45"
    bl_label = "细节饼 · 眼耳发指"

    def draw(self, context):
        _pie_draw_layer(self, context, 2)


class CN_MT_pie3_v45(bpy.types.Menu):
    """附属饼：装备/衣服/尾巴/道具（Ctrl+Alt+左键）"""
    bl_idname = "CN_MT_pie3_v45"
    bl_label = "附属饼 · 装备衣尾"

    def draw(self, context):
        _pie_draw_layer(self, context, 3)


class POSE_OT_cn_bone_order_v45(bpy.types.Operator):
    """调整集合里骨骼的**导航顺序**：上移/下移/置顶/移出顺序表/恢复原序

    排第一的那根，就是进这个集合时先被选中的那根，也是 ↑↓ / 滚轮走到这个
    集合时停的第一站。挪完会顺手把集合排序切到「集合顺序」—— 否则
    「空间排序 / 手指排序」会把它盖掉，用户会以为按钮没反应。
    """
    bl_idname = "pose.cn_bone_order_v45"
    bl_label = "调整骨骼顺序"
    bl_options = {'REGISTER', 'UNDO'}

    collection: bpy.props.StringProperty()
    bone: bpy.props.StringProperty()
    delta: bpy.props.IntProperty(default=0)
    to_index: bpy.props.IntProperty(default=-1)
    remove: bpy.props.BoolProperty(default=False, name="移出顺序表")
    reset: bpy.props.BoolProperty(default=False, name="恢复原顺序")

    @classmethod
    def poll(cls, context):
        return pie_menu_poll(context)

    def execute(self, context):
        rig = pie_rig(context)
        if rig is None or not edit_ok(rig):
            return {'CANCELLED'}
        col = _coll_get(rig, self.collection) if self.collection else None
        if col is None:
            return {'CANCELLED'}
        if self.reset:
            if set_bone_order(rig, col.name, []):
                self.report({'INFO'}, "「%s」已恢复 assign 原顺序" % col.name)
                return {'FINISHED'}
            self.report({'WARNING'}, "这根骨架是只读的，改不了顺序")
            return {'CANCELLED'}
        names = list(collection_bone_names(rig, col))
        if not names:
            return {'CANCELLED'}
        # 没点名就用当前选中的那根 —— 长集合不用翻到底也能一键排到第一
        bone = self.bone
        if not bone:
            act = rig.data.bones.active
            bone = act.name if act is not None else ""
        if not bone or bone not in names:
            self.report({'WARNING'}, "先选中集合「%s」里的一根骨骼" % col.name)
            return {'CANCELLED'}
        i = names.index(bone)
        if self.remove:
            names.pop(i)
            names.append(bone)          # 掉到末尾：既不丢骨，也回到「没排过」的状态
            if not write_bone_order(rig, col, names, self.report):
                return {'CANCELLED'}
            self.report({'INFO'}, "%s 已移出顺序表（排到最后）" % bone)
            return {'FINISHED'}
        if self.to_index >= 0:
            j = max(0, min(len(names) - 1, self.to_index))
        else:
            j = max(0, min(len(names) - 1, i + self.delta))
        if i == j:
            return {'CANCELLED'}
        names.insert(j, names.pop(i))
        if not write_bone_order(rig, col, names, self.report):
            return {'CANCELLED'}
        self.report({'INFO'}, "%s → 第 %d 位（共 %d 根）" % (bone, j + 1, len(names)))
        return {'FINISHED'}


class POSE_OT_cn_bone_order_batch_v45(bpy.types.Operator):
    """批量排集合顺序：链式收录 / 框选排序 / 按编号展开 / 反转 / 自然序 / 还原

    这四个「填」的方式都是把新排的一段放最前、其余成员按原顺序跟在后面，
    所以不会把没选到的骨骼从表里弄丢。
    """
    bl_idname = "pose.cn_bone_order_batch_v45"
    bl_label = "批量排骨骼顺序"
    bl_options = {'REGISTER', 'UNDO'}

    collection: bpy.props.StringProperty()
    mode: bpy.props.EnumProperty(items=(
        ('CHAIN', "链式收录", "从选中的链头沿父子一路收到底"),
        ('TOPOLOGY', "框选排序", "按「父先于子 + 编号自然序」排选中的骨骼"),
        ('SERIES', "按编号展开", "从当前骨骼的编号展开整条序列"),
        ('REVERSE', "反转顺序", "把当前顺序倒过来"),
        ('NATURAL', "按名字排序", "只按名字的自然序（01 在 10 前面）"),
        ('RESET', "恢复原顺序", "删掉自定义顺序，回到 assign 原序"),
    ), default='CHAIN')

    @classmethod
    def poll(cls, context):
        return pie_menu_poll(context)

    def execute(self, context):
        rig = pie_rig(context)
        if rig is None or not edit_ok(rig):
            return {'CANCELLED'}
        col = _coll_get(rig, self.collection) if self.collection else None
        if col is None:
            self.report({'WARNING'}, "找不到这个集合")
            return {'CANCELLED'}
        members = collection_members(rig, col)
        if not members:
            self.report({'WARNING'}, "「%s」里没有骨骼" % col.name)
            return {'CANCELLED'}
        if self.mode == 'RESET':
            set_bone_order(rig, col.name, [])
            self.report({'INFO'}, "「%s」已恢复 assign 原顺序" % col.name)
            return {'FINISHED'}
        if self.mode == 'REVERSE':
            cur = list(collection_bone_names(rig, col))
            cur.reverse()
            return self._done(rig, col, cur, "反转")
        if self.mode == 'NATURAL':
            cur = sorted(collection_bone_names(rig, col), key=natural_key)
            return self._done(rig, col, cur, "按名字")

        sel = selected_names(rig)
        act = rig.data.bones.active
        seed = act.name if act is not None else ""
        if self.mode == 'CHAIN':
            roots = [n for n in sel if n in set(members)] or ([seed] if seed else [])
            if not roots:
                self.report({'WARNING'}, "先选中链头那根骨骼（只选一根就够）")
                return {'CANCELLED'}
            roots.sort(key=natural_key)
            got = order_chain_names(rig, col, roots)
            if len(got) < 2:
                self.report({'WARNING'}, "这条链在「%s」里只有 %d 根 —— 换个链头，"
                                         "或先用「框选排序」" % (col.name, len(got)))
                return {'CANCELLED'}
            return self._done(rig, col, order_merge(rig, col, got), "链式收录", len(got))
        if self.mode == 'TOPOLOGY':
            pool = [n for n in sel if n in set(members)] or list(members)
            got = order_topology_names(rig, col, pool)
            return self._done(rig, col, got, "框选排序", len(got))
        if self.mode == 'SERIES':
            if not seed:
                self.report({'WARNING'}, "先选中一根带编号的骨骼（如 hair_fk.001）")
                return {'CANCELLED'}
            got = order_series_names(rig, col, seed)
            if len(got) < 2:
                self.report({'WARNING'}, "%s 没有可展开的编号序列（同前后缀只有一根）" % seed)
                return {'CANCELLED'}
            return self._done(rig, col, order_merge(rig, col, got), "按编号展开", len(got))
        return {'CANCELLED'}

    def _done(self, rig, col, names, what, count=None):
        if not write_bone_order(rig, col, names, self.report):
            return {'CANCELLED'}
        self.report({'INFO'}, "「%s」%s：%d 根已排好" % (col.name, what, count or len(names)))
        return {'FINISHED'}


# ---- 点选排序（v5.2.6）：唯一的顺序录入方式 ------------------------------
# Blender 不保存选中顺序（selected_pose_bones 永远按骨架内部先后返回），
# 所以用一个 0.12 秒的后台计时器盯「当前骨骼」：谁变成当前骨骼谁进队尾。
# 不是模态 —— 点一下开始，你照常加选，点回来写入，中间不打断任何操作。
_PICK_ORDER = {"armed": False, "rig": "", "col": "", "members": set(),
               "queue": [], "last": "", "timer_on": False}


def _pick_order_tick():
    """点选排序的心跳：活动骨骼一变就往队尾加一根。"""
    st = _PICK_ORDER
    if not st["armed"]:
        st["timer_on"] = False
        return None                      # 返回 None = 注销计时器
    rig = bpy.data.objects.get(st["rig"])
    col = _coll_get(rig, st["col"]) if rig is not None else None
    if rig is None or col is None or bpy.context.mode != 'POSE':
        st["armed"] = False
        st["timer_on"] = False
        return None
    act = rig.data.bones.active
    name = act.name if act is not None else ""
    if name and name != st["last"]:
        st["last"] = name
        if name in st["members"] and name not in st["queue"]:
            st["queue"].append(name)
    return 0.12


def _pick_order_timer(on):
    st = _PICK_ORDER
    if on:
        if not st["timer_on"]:
            try:
                bpy.app.timers.register(_pick_order_tick, first_interval=0.12,
                                        persistent=True)
                st["timer_on"] = True
            except Exception:
                st["timer_on"] = False
    else:
        st["armed"] = False
        if st["timer_on"]:
            try:
                bpy.app.timers.unregister(_pick_order_tick)
            except Exception:
                pass
            st["timer_on"] = False


class POSE_OT_cn_bone_order_pick_v45(bpy.types.Operator):
    """点选排序：按你一根一根加选的先后写集合顺序

    点第一下 = 开始记录（开始前只选了一根时，它算第 1 根）；
    然后在视图里正常加选，谁被点谁排后面；
    点第二下 = 写入。
    """
    bl_idname = "pose.cn_bone_order_pick_v45"
    bl_label = "点选排序"
    bl_options = {'REGISTER', 'UNDO'}

    collection: bpy.props.StringProperty()

    @classmethod
    def poll(cls, context):
        return pie_menu_poll(context)

    def execute(self, context):
        rig = pie_rig(context)
        if rig is None or not edit_ok(rig):
            return {'CANCELLED'}
        col = _coll_get(rig, self.collection) if self.collection else None
        if col is None:
            self.report({'WARNING'}, "找不到这个集合")
            return {'CANCELLED'}
        st = _PICK_ORDER
        armed = st["armed"] and st["rig"] == rig.name and st["col"] == col.name
        if armed:
            # 第二下：写入（一根都没点 = 取消）
            _pick_order_timer(False)
            queue = [n for n in st["queue"] if n in st["members"]]
            if not queue:
                self.report({'INFO'}, "一根都没点，已取消，顺序没改")
                return {'CANCELLED'}
            if not write_bone_order(rig, col, order_merge(rig, col, queue),
                                    self.report):
                return {'CANCELLED'}
            self.report({'INFO'}, "按点选顺序写入 %d 根（「%s」）"
                        % (len(queue), col.name))
            return {'FINISHED'}
        # 第一下：开始记录
        st["rig"] = rig.name
        st["col"] = col.name
        st["members"] = set(collection_members(rig, col))
        st["queue"] = []
        act = rig.data.bones.active
        st["last"] = act.name if act is not None else ""
        sel = [pb.name for pb in rig.pose.bones if pb.select]
        if len(sel) == 1 and sel[0] in st["members"]:
            # 按按钮前你已经选好了第 1 根：把它记进队列
            st["queue"].append(sel[0])
        st["armed"] = True
        _pick_order_timer(True)
        tip = ("（第 1 根已记：%s）" % st["queue"][0]) if st["queue"] else ""
        self.report({'INFO'}, "开始记录：去视图里一根一根加选，点完再按一次「点选排序」写入" + tip)
        return {'FINISHED'}


class POSE_OT_cn_preset_menu_v45(bpy.types.Operator):
    """弹出集合菜单：归入 / 移出 / 新建"""
    bl_idname = "pose.cn_preset_menu_v45"
    bl_label = "集合操作"
    bl_options = {'REGISTER'}

    @classmethod
    def poll(cls, context):
        return pose_poll(context)

    def invoke(self, context, event):
        rig = context.object
        n = len(selected_bones(rig))
        title = "集合操作" if n == 0 else "集合操作（已选 %d 个）" % n
        context.window_manager.popup_menu(self.draw_menu, title=title, icon='BONE_DATA')
        return {'FINISHED'}

    def draw_menu(self, menu, context):
        layout = menu.layout
        rig = context.object
        has_sel = bool(selected_bones(rig))

        if not has_sel:
            layout.label(text="先选中控制器，再点集合名", icon='INFO')

        rows = preset_menu_rows(rig)
        extra = rig_collection_rows(rig)
        if rows:
            layout.menu("CN_MT_preset_collections_v45",
                        text="预设集合（%d）" % len(rows), icon='PRESET')
        if extra:
            layout.menu("CN_MT_other_collections_v45",
                        text="当前骨架其它集合（%d）" % len(extra), icon='BONE_DATA')
        if not rows and not extra:
            layout.label(text="还没有可用集合", icon='ERROR')
            layout.operator("preferences.addon_show", text="去偏好里配置预设…",
                            icon='PREFERENCES').module = MODULE_NAME

        layout.separator()
        cur = current_cycle_name(rig, rig.data.bones.active) if rig else None
        row = layout.row()
        row.enabled = has_sel and bool(cur)
        op = row.operator("pose.cn_remove_from_collection_v45",
                          text="从「%s」移出" % (cur or "当前集合"), icon='REMOVE')
        if cur:
            op.collection = cur

        row = layout.row()
        row.enabled = has_sel
        row.operator("pose.cn_remove_from_collection_v45",
                     text="从所有集合移出", icon='X').all_collections = True

        layout.separator()
        row = layout.row()
        row.enabled = has_sel
        row.operator("pose.cn_new_collection_v45", text="新建集合并归入…", icon='ADD')
        layout.operator("pose.cn_create_presets_v45", text="补建缺失的预设集合", icon='FILE_REFRESH')


# ---------------------------------------------------------------- 指定集合

class POSE_OT_cn_set_collection_v45(bpy.types.Operator):
    """手动指定当前骨架用哪个集合导航（保存在骨架数据中，随文件保存）。"""
    bl_idname = "pose.cn_set_collection_v45"
    bl_label = "指定导航集合"
    bl_options = {'REGISTER'}

    collection: bpy.props.StringProperty()

    def execute(self, context):
        context.object.data["cn_collection"] = self.collection
        return {'FINISHED'}


class POSE_OT_cn_clear_collection_v45(bpy.types.Operator):
    """取消指定，恢复自动判断。"""
    bl_idname = "pose.cn_clear_collection_v45"
    bl_label = "取消指定（恢复自动）"
    bl_options = {'REGISTER'}

    def execute(self, context):
        context.object.data.pop("cn_collection", None)
        return {'FINISHED'}


# ---------------------------------------------------------------- 导航操作

class POSE_OT_controller_nav_v45(bpy.types.Operator):
    bl_idname = "pose.controller_nav_v45"
    bl_label = "驭骨导航"
    bl_options = {'REGISTER'}

    mode: bpy.props.EnumProperty(
        name="Mode",
        items=(
            ('PREV', "往上一个", "集合内往上一个"),
            ('NEXT', "往下一个", "集合内往下一个"),
        ),
        default='NEXT',
    )

    extend: bpy.props.BoolProperty(name="加选", default=False)

    @classmethod
    def poll(cls, context):
        return (context.mode == 'POSE'
                and context.object is not None
                and context.object.type == 'ARMATURE')

    def execute(self, context):
        rig = context.object
        if rig is None or rig.type != 'ARMATURE' or context.mode != 'POSE':
            self.report({'WARNING'}, "请在 Pose Mode 下选中骨架")
            return {'CANCELLED'}

        # v5.2.15：旋转/移动进行中切骨骼，个别 RNA 访问会带出 SystemError
        # （"returned a result with an exception set"）—— 捕获后按普通提示报，
        # 不再往信息栏甩原始异常。
        _error_wash()   # v5.2.16：进来先清 C 层静默残留
        try:
            return self._nav(context, rig)
        except Exception as exc:
            self.report({'WARNING'}, "导航失败：%s" % exc)
            return {'CANCELLED'}

    def _nav(self, context, rig):
        cur = rig.data.bones.active

        if cur is None:
            items = nav_list(rig, None)
            if not items:
                return {'CANCELLED'}
            select_bone(rig, items[0], self.extend)
            remember_mode(rig, items[0])
            if not smooth_view_to(context, rig, items[0]):
                follow_cursor_to(context, rig, items[0])
            return {'FINISHED'}

        items = nav_list(rig, cur.name)
        if not items:
            return {'CANCELLED'}

        idx = items.index(cur.name) if cur.name in items else 0
        delta = -1 if self.mode == 'PREV' else 1
        target = items[(idx + delta) % len(items)]
        select_bone(rig, target, self.extend)
        remember_mode(rig, target)
        if not smooth_view_to(context, rig, target):
            follow_cursor_to(context, rig, target)
        return {'FINISHED'}


class POSE_OT_cn_warp_mouse_v45(bpy.types.Operator):
    """把鼠标移到当前活动骨骼所在的视口位置"""
    bl_idname = "pose.cn_warp_mouse_v45"
    bl_label = "鼠标移到当前骨骼"
    bl_options = {'REGISTER'}

    @classmethod
    def poll(cls, context):
        return pose_poll(context)

    def execute(self, context):
        rig = context.object
        cur = rig.data.bones.active
        if cur is None:
            self.report({'WARNING'}, "没有活动骨骼")
            return {'CANCELLED'}
        if warp_cursor_to_bone(context, rig, cur.name):
            return {'FINISHED'}
        self.report({'WARNING'}, "骨骼不在当前视口里，或鼠标不在 3D 视图上")
        return {'CANCELLED'}


class POSE_OT_cn_warp_offset_reset_v45(bpy.types.Operator):
    """把落点偏移归零（回到骨骼头部 / Gizmo 正中心）"""
    bl_idname = "pose.cn_warp_offset_reset_v45"
    bl_label = "落点偏移归零"
    bl_options = {'REGISTER'}

    @classmethod
    def poll(cls, context):
        return get_prefs() is not None

    def execute(self, context):
        p = get_prefs()
        if p is None:
            return {'CANCELLED'}
        p.warp_dx = 0
        p.warp_dy = 0
        return {'FINISHED'}


class POSE_OT_cn_collection_step_v45(bpy.types.Operator):
    """切到循环清单里的下一个 / 上一个集合，并选中其中第一根骨骼"""
    bl_idname = "pose.cn_collection_step_v45"
    bl_label = "切换集合"
    bl_options = {'REGISTER', 'UNDO'}

    mode: bpy.props.EnumProperty(
        name="方向",
        items=(('PREV', "上一个集合", "切到清单里的上一个集合"),
               ('NEXT', "下一个集合", "切到清单里的下一个集合")),
        default='NEXT',
    )

    @classmethod
    def poll(cls, context):
        return pose_poll(context)

    def execute(self, context):
        ok, msg = step_collection(context, context.object,
                                  -1 if self.mode == 'PREV' else 1)
        if not ok:
            self.report({'WARNING'}, msg)
            return {'CANCELLED'}
        self.report({'INFO'}, msg)
        return {'FINISHED'}


class POSE_OT_cn_collection_goto_v45(bpy.types.Operator):
    """直接跳到指定集合，并选中其中第一根骨骼"""
    bl_idname = "pose.cn_collection_goto_v45"
    bl_label = "跳转到集合"
    bl_options = {'REGISTER', 'UNDO'}

    collection: bpy.props.StringProperty()

    @classmethod
    def poll(cls, context):
        return pose_poll(context)

    def execute(self, context):
        ok, msg = goto_collection(context, context.object, self.collection)
        if not ok:
            self.report({'WARNING'}, msg)
            return {'CANCELLED'}
        self.report({'INFO'}, msg)
        return {'FINISHED'}


class POSE_OT_cn_cycle_add_v45(bpy.types.Operator):
    """把当前集合加进循环清单（也就是预设集合清单）"""
    bl_idname = "pose.cn_cycle_add_v45"
    bl_label = "加入循环清单"
    bl_options = {'REGISTER'}

    @classmethod
    def poll(cls, context):
        return pose_poll(context)

    def execute(self, context):
        rig = context.object
        name = current_cycle_name(rig, rig.data.bones.active)
        if not name:
            self.report({'WARNING'}, "当前不在任何集合里")
            return {'CANCELLED'}
        p = get_prefs()
        if p is None:
            return {'CANCELLED'}
        if any((it.col_name or "").strip() == name for it in p.presets):
            self.report({'INFO'}, "「%s」已经在清单里" % name)
            return {'FINISHED'}
        p.presets.add().col_name = name
        self.report({'INFO'}, "已加入循环清单：%s" % name)
        return {'FINISHED'}


class POSE_OT_cn_cycle_remove_v45(bpy.types.Operator):
    """把当前集合从循环清单里移除（骨骼和集合本身不动）"""
    bl_idname = "pose.cn_cycle_remove_v45"
    bl_label = "移出循环清单"
    bl_options = {'REGISTER'}

    @classmethod
    def poll(cls, context):
        return pose_poll(context)

    def execute(self, context):
        rig = context.object
        name = current_cycle_name(rig, rig.data.bones.active)
        if not name:
            self.report({'WARNING'}, "当前不在任何集合里")
            return {'CANCELLED'}
        p = get_prefs()
        if p is None:
            return {'CANCELLED'}
        removed = 0
        for i in range(len(p.presets) - 1, -1, -1):
            if (p.presets[i].col_name or "").strip() == name:
                p.presets.remove(i)
                removed += 1
        if removed:
            self.report({'INFO'}, "已把「%s」移出循环清单" % name)
        else:
            self.report({'INFO'}, "清单里本来就没有「%s」" % name)
        return {'FINISHED'}


class POSE_OT_cn_import_collections_v45(bpy.types.Operator):
    """把当前骨架里可见的集合导入循环清单（跳过 MCH/ORG/DEF/Widget 内部集合）"""
    bl_idname = "pose.cn_import_collections_v45"
    bl_label = "导入骨架集合"
    bl_options = {'REGISTER'}

    @classmethod
    def poll(cls, context):
        return pose_poll(context)

    def execute(self, context):
        rig = context.object
        names = importable_collections(rig)
        if not names:
            self.report({'WARNING'}, "没有可导入的可见集合")
            return {'CANCELLED'}
        p = get_prefs()
        if p is None:
            return {'CANCELLED'}

        existing = set(preset_names())
        added = []
        for n in names:
            if n in existing:
                continue
            p.presets.add().col_name = n
            added.append(n)
        if added:
            self.report({'INFO'}, "导入 %d 个集合：%s" % (len(added), "、".join(added[:8])))
        else:
            self.report({'INFO'}, "骨架里的可见集合都已在清单里")
        return {'FINISHED'}


# ---------------------------------------------------------------- 快速旋转

# 按 X / 滚轮循环的顺序：X → Z → Y（和你的习惯一致）
# 每项：(显示名, 局部轴向量, lock_rotation 里的下标)
ROT_AXES = (
    ('X', (1.0, 0.0, 0.0), 0),
    ('Z', (0.0, 0.0, 1.0), 2),
    ('Y', (0.0, 1.0, 0.0), 1),
)

# 移动的轴：下标 = location / lock_location 的下标（0=X 1=Y 2=Z，世界轴）
MOVE_AXES = ('X', 'Y', 'Z')

# 模态单实例闸门：模态运行期间，任何漏到键位映射的事件都可能再次触发进入键，
# 叠出第二个模态（多份 HUD + 右键要按多次）。用列表当可变标志，注册/清理都改它。
_MODAL_ONCE = [False]
MOVE_UNIT = (Vector((1.0, 0.0, 0.0)),
             Vector((0.0, 1.0, 0.0)),
             Vector((0.0, 0.0, 1.0)))
# 滚轮向下推一格 = 往下一根轴走：自由 → X → Y → Z → 自由
MOVE_CYCLE = (None, 0, 1, 2)

# ---- 旋转灵敏度：锚到原生 gizmo 的量级
# 老版 BASE 0.5 ×（120 / 屏幕长度）把半屏大的骨骼衰减到 0.15°/px：转 90° 要拖
# 600px，而原生拖 100px 就过 90°。现在锚点直接取原生量级（≈0.85°/px），
# 自适应只保留一条很软的曲线（指数 0.3），手指和大腿差不到 2 倍，
# 再靠偏好里的倍率微调 —— 这样"调倍率"才有意义。
NATIVE_DEG_PER_PX = 1.0    # 锚点骨骼（屏幕投影 ROT_REF_PX）上的 °/px
ROT_REF_PX = 140.0         # 锚点骨骼的屏幕投影长度
ROT_SOFT_POW = 0.30        # 长度自适应强度：(REF / len) ** POW
ROT_MIN_PX = 24.0          # 骨骼极短时的长度下限，防止灵敏度爆表
ROT_MIN_DEG_PER_PX = 0.35
ROT_MAX_DEG_PER_PX = 2.6
FINE_FACTOR = 0.15

# 拖动中把姿态同步进关键帧的最小间隔（秒）。自动关键帧开着时，姿态必须
# 一直等于当前帧曲线上的值，否则任何一次动画重估都会把它拽回旧值。
# v5.2.38：从 0.04 改成 0 —— 限流期间曲线最多落后 40ms，实测拖拽中途
# frame_set 一次，姿态被拽回 43.7°，再被 _ensure_baseline 写回来就是
# 一帧"闪跳"。而插入本身只对 _targets 生效：实测单骨 0.16ms、30 骨
# 0.68ms，逐帧插入完全吃得住，没有限流的必要。
KEY_SYNC_SECONDS = 0.0

# ---- 自由旋转（TRACKBALL）的原生角度曲线
# 原生 gizmo 的角度不是线性的：绕枢轴在球面上划，angle = 2·atan(位移 / 球半径)。
# 起手快、越远越收敛。球半径跟活动半径同源（ring_zone），指针顶到活动半径
# 那一刻正好约 90°，和原生拖满一圈 gizmo 的手感一致。
TRACKBALL_R_PX = 110.0

# ---- 轴探测 / 手势分级
ROT_PROBE_DEG = 2.0        # 探测角：把每根未锁轴转这么多，看末端往屏幕哪边动

# ---- 指针回绕：离视口边缘多少像素就把指针拉回中心
ROT_WRAP_MARGIN = 70


def axis_locks(pb):
    """读取骨骼的旋转轴锁定：返回 (锁X, 锁Y, 锁Z)。"""
    try:
        return tuple(bool(v) for v in pb.lock_rotation)
    except Exception:
        return (False, False, False)


FINGER_AXIS_ITEMS = (
    ('X', "X", "本地 X"),
    ('Z', "Z", "本地 Z"),
    ('Y', "Y", "本地 Y"),
)
_AXIS_INDEX = {'X': 0, 'Z': 1, 'Y': 2}


def _finger_segment(pb):
    """返回手指节数 1/2/3，不是手指返回 None。

    覆盖：
      Rigify f_index.01.L / f_index.01_master.L / f_index.01.L.001
      ARP    c_index1.l / c_middle2.l / c_thumb3.l
      自定义 c_index_01.l / finger_index_01.l / c_f_index1.l
    """
    name = pb.name
    m = re.search(r"(index|middle|ring|pinky|thumb)[._\-\s]?(\d{1,2})",
                  name, re.I)
    if m:
        try:
            seg = int(m.group(2))
            return seg if 1 <= seg <= 9 else None
        except Exception:
            return None
    # 名字完全不像，但集合明确是手指：按集合兜底
    try:
        cols = [c.name.lower() for c in pb.bone.collections]
    except Exception:
        return None
    if any("finger" in c or "手指" in c for c in cols):
        digits = re.findall(r"\d", name)
        if digits:
            return max(1, min(9, int(digits[-1])))
    return None


def _finger_default_axis(pb):
    """手指默认单轴：返回 ROT_AXES 下标；不是手指/没开/全锁返回 None。"""
    if not pref('finger_single', True):
        return None
    if _finger_segment(pb) is None:
        return None
    idx = _AXIS_INDEX.get(pref('finger_axis', 'X'), 0)
    if not axis_locked(pb, idx):
        return idx
    for i in range(len(ROT_AXES)):
        if not axis_locked(pb, i):
            return i
    return None


MOTION_MODE_ITEMS = (
    ('BOTH', "可转可移", "旋转和移动都允许"),
    ('ROT', "只旋转", "这个集合里的骨骼不允许移动"),
    ('MOVE', "只移动", "这个集合里的骨骼不允许旋转"),
)


def collection_motion_mode(rig, col):
    if rig is None or col is None:
        return 'BOTH'
    try:
        modes = rig.data.get("cn_motion_modes", {}) or {}
        return str(modes.get(col.name, 'BOTH'))
    except Exception:
        return 'BOTH'


def set_collection_motion_mode(rig, col, mode):
    if rig is None or col is None:
        return False
    try:
        modes = dict(rig.data.get("cn_motion_modes", {}) or {})
        modes[col.name] = mode
        rig.data["cn_motion_modes"] = modes
        return True
    except Exception:
        return False


def _bone_motion_modes(pb):
    try:
        idd = pb.id_data
        arm = getattr(idd, "data", idd)      # PoseBone.id_data 可能是 Object
        modes = arm.get("cn_motion_modes", {}) or {}
        out = set()
        for c in pb.bone.collections:
            out.add(str(modes.get(c.name, 'BOTH')))
        return out
    except Exception:
        return set()


BONE_MOTION_ITEMS = (
    ('INHERIT', "跟随集合", "使用所在集合的动作权限"),
    ('BOTH', "可转可移", "这根骨骼允许旋转和移动"),
    ('ROT', "只旋转", "这根骨骼禁止移动"),
    ('MOVE', "只移动", "这根骨骼禁止旋转"),
)


def bone_motion_mode(pb):
    try:
        idd = pb.id_data
        arm = getattr(idd, "data", idd)
        modes = arm.get("cn_bone_motions", {}) or {}
        return str(modes.get(pb.name, 'INHERIT'))
    except Exception:
        return 'INHERIT'


def set_bone_motion_mode(pb, mode):
    try:
        idd = pb.id_data
        arm = getattr(idd, "data", idd)
        modes = dict(arm.get("cn_bone_motions", {}) or {})
        modes[pb.name] = mode
        arm["cn_bone_motions"] = modes
        return True
    except Exception:
        return False


def rig_refine_mode(rig):
    st = _rig_spatial_settings(rig)
    return bool(getattr(st, "refine_mode", False)) if st is not None else False


def rig_refine_axis(rig):
    st = _rig_spatial_settings(rig)
    label = getattr(st, "refine_axis", 'Y') if st is not None else 'Y'
    return _AXIS_INDEX.get(label, 2)


def _available_axis(pb, want):
    if not axis_locked_eff(pb, want):
        return want
    for i in range(len(ROT_AXES)):
        if not axis_locked_eff(pb, i):
            return i
    return None


def _default_rotate_axis(rig, pb):
    """细化模式：非手指用细化轴；手指始终用手指自己的默认轴。"""
    if _finger_segment(pb) is not None:
        return _finger_default_axis(pb)
    if rig_refine_mode(rig):
        return _available_axis(pb, rig_refine_axis(rig))
    return None


def _bone_rotatable(pb):
    override = bone_motion_mode(pb)
    if override == 'MOVE':
        return False
    if override in {'INHERIT', ''} and 'MOVE' in _bone_motion_modes(pb):
        return False
    return any(not axis_locked_eff(pb, i) for i in range(len(ROT_AXES)))


def _bone_needs_native(pb):
    """ARP 的 pole 用 Child Of 约束驱动，写 location 会被覆盖，必须原生 G。

    Rigify 的 pole 没有这些 Child Of，继续走自定义 location 移动，
    保留中键视角 + 滚轮切轴。
    """
    try:
        if 'pole' not in pb.name.lower():
            return False
        return any(c.type == 'CHILD_OF' for c in pb.constraints)
    except Exception:
        return False


def _bone_movable(pb):
    override = bone_motion_mode(pb)
    if override == 'ROT':
        return False
    # 跟随集合时：只旋转的集合不允许移动
    if override in {'INHERIT', ''} and 'ROT' in _bone_motion_modes(pb):
        return False
    # 连接骨不能平移（location 通道无效）；三轴位置锁也视为不可移动
    try:
        if pb.bone.use_connect:
            return False
    except Exception:
        pass
    return any(not bool(pb.lock_location[i]) for i in range(3))


def _finger_auto_unlocked(pb):
    """兼容旧代码：手指自动轴已停用，这里永远返回 None。"""
    return None


def axis_locked_eff(pb, axis_index):
    """实际手势里该轴是否不可用：骨骼锁 + 手指自动轴。"""
    if axis_locked(pb, axis_index):
        return True
    allowed = _finger_auto_unlocked(pb)
    if allowed is not None and axis_index not in allowed:
        return True
    return False


def axis_locked(pb, axis_index):
    """当前 ROT_AXES 下标对应的轴是否被锁。"""
    locks = axis_locks(pb)
    return locks[ROT_AXES[axis_index % len(ROT_AXES)][2]]


def rotate_basis(start_matrix, angle_rad, axis_index):
    """在骨骼自身坐标系里绕指定轴旋转（等价于 R X X）。"""
    axis = ROT_AXES[axis_index % len(ROT_AXES)][1]
    return start_matrix @ Matrix.Rotation(angle_rad, 4, axis)


def _parent_matrix(pb):
    """P：把 matrix_basis 映射到 armature 空间的左乘矩阵。"""
    if pb.parent is not None:
        return (pb.parent.matrix
                @ pb.parent.bone.matrix_local.inverted()
                @ pb.bone.matrix_local)
    return pb.bone.matrix_local.copy()


def _apply_pivot_rotation(pb, parent_m, m_start, pivot, quat):
    """绕 armature 空间的 pivot 转 quat，再反解 matrix_basis。"""
    R = quat.to_matrix().to_4x4()
    W = Matrix.Translation(pivot) @ R @ Matrix.Translation(-pivot)
    pb.matrix_basis = parent_m.inverted() @ (W @ m_start)


def _constraint_shifts(parent_m, m_start, basis_st):
    """这根骨骼的可见姿态是不是被约束改写过（朝向或位置任一对不上都算）。

    判据：`b.matrix`（含约束的求值结果）和「父级映射 × matrix_basis」不一致。
    一致说明没有约束参与，可以照旧用世界矩阵反解；不一致就必须把约束带来的
    那层偏移（K）一起带上算，否则既会转错轴，也会在提交点跳一下。
    """
    try:
        plain = parent_m @ basis_st
        if abs(plain.to_quaternion().rotation_difference(
                m_start.to_quaternion()).angle) > 0.0087:      # 0.5°
            return True
        return (plain.to_translation() - m_start.translation).length > 1e-4
    except Exception:
        return False


def _visible_offset(parent_m, m_start, basis_st):
    """K = 可见姿态 · (父级 × basis)⁻¹：约束叠在骨骼上的那层固定偏移。

    v4.9.94：c_arm_fk.r 这类控制器带 ArmLock(ARMATURE) + Copy Location，
    K 的旋转可以有几十度、平移接近 1m。以前只在 basis 空间里按 P 共轭
    （P⁻¹RP），等于把这个 K 当成单位阵 —— 结果就是「转的方向不是 gizmo 上
    那根轴」。把 K 一起夹进去（K⁻¹RPK）得到的才是可见空间里的真旋转。
    """
    try:
        return m_start @ (parent_m @ basis_st).inverted()
    except Exception:
        return None


def _apply_visible_rotation(pb, parent_m, m_start, basis_st, pivot, quat):
    """让「可见姿态」绕 armature 空间的 pivot 转 quat（轴按可见空间给）。

    可见姿态 M = K · P · basis（K 见 _visible_offset）。要让 M 多转 R，
    等价于 basis 左乘 P⁻¹ · K⁻¹ · R_p · K · P，R_p 为绕 pivot 的旋转。
    K 是单位阵时（无约束骨骼）它精确退化成 P⁻¹·R_p·P —— 和原来的
    _apply_pivot_rotation 等价，所以两条路径可以合并成这一条。
    """
    K = _visible_offset(parent_m, m_start, basis_st)
    if K is None:
        K = Matrix.Identity(4)
    R = quat.to_matrix().to_4x4()
    Rp = Matrix.Translation(pivot) @ R @ Matrix.Translation(-pivot)
    D = parent_m.inverted() @ K.inverted() @ Rp @ K @ parent_m
    pb.matrix_basis = D @ basis_st


def _apply_visible_translation(pb, parent_m, m_start, basis_st, delta):
    """让「可见姿态」在 armature 空间整体平移 delta（约束偏移 K 一起带上）。

    和 _apply_visible_rotation 同一个套路：可见 M = K · P · basis，要让 M 变成
    T·M，就得让 basis 左乘 P⁻¹ · K⁻¹ · T · K · P。K 是单位阵时退化成
    P⁻¹·T·P，等价于写 `pb.matrix = T @ m`。

    为什么不能直接写 `pb.matrix`：ARP 的 c_foot_ik / c_hand_ik 挂 Child Of(c_traj)，
    可见矩阵由约束求出，而 `pb.matrix` 的 setter 只按 rest 关系反解 basis、不认约束。
    只要 c_traj 不是单位姿态（动了根部/重心），"读出来再写回去"就会把 c_traj 的
    那份偏移再叠一遍 —— 实测 0.62m，就是「脚 IK 第一下拖动整条腿弹起来」。
    """
    K = _visible_offset(parent_m, m_start, basis_st)
    if K is None:
        K = Matrix.Identity(4)
    T = Matrix.Translation(delta)
    D = parent_m.inverted() @ K.inverted() @ T @ K @ parent_m
    pb.matrix_basis = D @ basis_st


def _restore_to_start(pb, parent_m, m_start, basis_st=None):
    """把姿态还原到基线（等于不转）。

    basis_st 就是那个基线的 matrix_basis 原件，直接写回去永远精确；只有拿不
    到时才退回「经 parent_m 反解」——反解要求 m_start == P·basis，带约束的
    骨骼不满足（实测 c_arm_fk.r 差 76°），写出来会当场跳一下。
    """
    try:
        if basis_st is not None:
            pb.matrix_basis = basis_st.copy()
        else:
            pb.matrix_basis = parent_m.inverted() @ m_start
    except Exception:
        pass


def _pivot_armature_space(context, rig, pb):
    """按场景的 transform_pivot_point 取枢轴（armature 空间）。"""
    ts = getattr(context.scene, "tool_settings", None)
    mode = getattr(ts, "transform_pivot_point", 'MEDIAN_POINT') if ts else 'MEDIAN_POINT'
    if mode == 'CURSOR':
        return rig.matrix_world.inverted() @ context.scene.cursor.location
    if mode == 'MEDIAN_POINT':
        sel = [b for b in (context.selected_pose_bones or []) if b is not None]
        if len(sel) > 1:
            acc = Vector((0.0, 0.0, 0.0))
            for b in sel:
                acc += b.matrix.translation
            return acc / len(sel)
        return pb.matrix.translation.copy()
    return pb.matrix.translation.copy()


def _view_axes(rv3d):
    """视口的屏幕右方向 / 上方向（世界空间单位向量）。"""
    try:
        q = rv3d.view_rotation
        return q @ Vector((1.0, 0.0, 0.0)), q @ Vector((0.0, 1.0, 0.0))
    except Exception:
        return Vector((1.0, 0.0, 0.0)), Vector((0.0, 1.0, 0.0))


# ---------------------------------------------------------------- 屏幕投影

_HUD_OK = [None]


def _hud_available():
    """blf 是否可用；画失败一次就永久关掉，避免刷屏。"""
    if _HUD_OK[0] is None:
        try:
            import blf  # noqa: F401
            _HUD_OK[0] = True
        except Exception:
            _HUD_OK[0] = False
    return _HUD_OK[0]


def _view_pair(context):
    """取 (region, rv3d)；任一缺失返回 (None, None)。"""
    region = getattr(context, "region", None)
    space = getattr(context, "space_data", None)
    if region is None or space is None:
        return None, None
    if region.type != 'WINDOW':
        return None, None
    return region, getattr(space, "region_3d", None)


def _bone_point_world(rig, pb, matrix_basis, local_offset):
    """把骨骼上的某个局部点（沿自身 Y 一个骨长，再偏一点 X）换算到世界坐标。

    matrix_basis 是「假设姿态」= 只推演、不落盘；自动判轴靠这个试探三根轴
    各自会把骨骼推向屏幕的哪个方向。

    v5.2.25 修复：原来写的是 `parent.matrix @ basis @ bone.matrix_local`，
    既少了父级 rest 的还原（`parent.bone.matrix_local⁻¹`），又把 basis 放到了
    错误的一侧。实测（插件测试 根部控制器…blend）脊椎/手臂/头部偏 1.4~1.9
    个单位（≈5 个骨长），判轴方向错 93°~180°，屏幕长度跟着乱 —— 灵敏度会
    随根骨（大环）的变换漂移，拖第一下就容易"跳"。正确式子是 armature 空间的
    P·basis（P 的定义见 _parent_matrix）。
    """
    basis = pb.matrix_basis.copy() if matrix_basis is None else matrix_basis
    try:
        m = _parent_matrix(pb) @ basis
    except Exception:
        m = pb.matrix
    return rig.matrix_world @ (m @ local_offset)


def _probe_offsets(pb):
    """两个观测点。

    第二个点从末端沿局部 X 偏出 0.3 个骨长：绕骨骼自身轴（Y，也就是扭转）时，
    正末端正好落在轴上、屏幕上纹丝不动，靠这个偏心点才观测得到。
    """
    try:
        length = float(pb.bone.length)
    except Exception:
        length = 1.0
    if length < 1e-6:
        length = 1.0
    return (Vector((0.0, length, 0.0)),
            Vector((length * 0.3, length, 0.0)))


def _screen_of(region, rv3d, co):
    if region is None or rv3d is None:
        return None
    try:
        return location_3d_to_region_2d(region, rv3d, co)
    except Exception:
        return None


def _has_active_ik(pb):
    """骨骼上有没有真正生效的 IK 约束（有就直接写 matrix_basis 会被覆盖）。"""
    try:
        for c in pb.constraints:
            if c.type == 'IK' and not c.mute and getattr(c, "influence", 1.0) > 0.001:
                return True
    except Exception:
        pass
    return False


class POSE_OT_cn_quick_rotate_v45(bpy.types.Operator):
    """自由旋转：按 X 进入后鼠标直接转，跟拽 gizmo 一个手感；轴限制跟随骨骼轴锁"""
    bl_idname = "pose.cn_quick_rotate_v45"
    bl_label = "自由旋转"
    bl_options = {'REGISTER', 'UNDO'}

    start_mode: bpy.props.EnumProperty(items=(
        ('ROT', "旋转", ""),
        ('MOVE', "移动", ""),
    ), default='ROT')

    @classmethod
    def poll(cls, context):
        return (context.mode == 'POSE'
                and context.object is not None
                and context.object.type == 'ARMATURE'
                and context.active_pose_bone is not None)

    def invoke(self, context, event):
        if _MODAL_ONCE[0]:
            # 已经在模态里了。没有这道闸，任何走 PASS_THROUGH 的按键都会被键位映射
            # 再触发一次进入键 → 叠出第二个模态（多份 HUD、右键要按好几次才退干净）。
            self.report({'WARNING'}, "已经在旋转/移动模态里了：左键确认，右键退一步")
            return {'CANCELLED'}
        pb = context.active_pose_bone
        if pb is None:
            self.report({'WARNING'}, "没有活动骨骼")
            return {'CANCELLED'}
        if self.start_mode == 'ROT' and not _bone_rotatable(pb):
            self.report({'WARNING'}, "这根骨骼不能旋转（轴全锁或集合只允许移动）")
            return {'CANCELLED'}
        if self.start_mode == 'MOVE' and not _bone_movable(pb):
            self.report({'WARNING'}, "这根骨骼不能移动（连接骨或位置锁定）")
            return {'CANCELLED'}
        # ARP 的 Child Of 极点：继续进自定义模态，但移动时用原生 transform 逐帧写入


        self._rig = context.object
        self._pb = pb
        sel = [b for b in (context.selected_pose_bones or []) if b is not None]
        if pb not in sel:
            sel = [pb]
        # 只旋转“选中层级里的根骨骼”：父级也被选中时，子级跟着父级走，
        # 不再自己再转一次，避免整条手臂被叠加旋转。
        roots = [b for b in sel if b.parent not in sel]
        if not roots:
            roots = [pb]
        self._targets = roots
        self._orig = {b.name: b.matrix_basis.copy() for b in sel}
        # 本段触碰名单：右键只回滚这里的骨骼。切骨骼/切集合 = 提交点，
        # 名单换成新骨骼 —— 上一段提交过的骨骼从结构上进不了回滚范围，
        # 就算 _orig 快照时序出问题也绝不会被连带撤掉。
        self._touched = set(self._orig.keys())
        # 整段会话碰过的骨骼全集：快照一律按它取，这样右键回退能跨控制器一起还原
        self._session = set(self._orig.keys())
        # 撤销步骤栈：每项 = 提交点上的「整段会话姿态 + 当时的焦点控制器」。
        # [0] 是进入模态时的状态；右键逐级回退，退到 [0] 就是回到原样。
        self._kind = 'ROT'               # 先定住，_make_step 要读它
        self._steps = [self._make_step()]
        self._steps_i = 0
        self._steps0 = {b.name: b.matrix_basis.copy() for b in sel}   # 兼容字段
        self._session0 = {b.name: b.matrix_basis.copy() for b in sel}  # 双击右键整体还原
        self._undo_none = False          # 右键已经没有更早的步骤可退（HUD 提示用）
        self._states = {}
        self._states_basis = {}      # 基线对应的 matrix_basis（识别带约束的骨骼）
        for b in roots:
            bpy.context.view_layer.update()
            self._capture_state(b)
        self._start = pb.matrix_basis.copy()
        bpy.context.view_layer.update()
        self._m_start = pb.matrix.copy()
        self._parent_m = _parent_matrix(pb)
        self._pivot = _pivot_armature_space(context, self._rig, pb)
        self._kind = 'ROT'
        self._axis_lock = _default_rotate_axis(self._rig, pb)
        self._move_axis = None
        self._move_amount = 0.0
        self._mv_pending = Vector((0.0, 0.0, 0.0))   # 还没提交给原生 G 的位移（armature 空间）
        self._mv_bones = []                          # 参与移动的骨骼（选中层级里的根）
        self._mv_bases = {}                          # name -> (matrix, matrix_basis)
        self._snap_step = 0.0   # 当前吸附档，0 = 不吸附
        self._last_rmb = 0.0
        self._mmb_click = None
        self._scaling = False
        self._scale_states = {}
        self._scale_mouse = (0, 0)
        self._scale_factor = 1.0
        self._primed = False    # 第一次 MOUSEMOVE 只更新参考点，不当成输入
        self._moved_any = False  # 本段（切模式/切轴/切骨骼为界）是否已吃进过输入
        self._handle = None
        self._cursor_hidden = False
        self._rx = 0            # 指针在 region 内的坐标（HUD 用）
        self._ry = 0

        self._ref_x = event.mouse_x
        self._ref_y = event.mouse_y

        # 手势分级：三轴全开 → trackball；两轴 → 双通道；一轴 → 单轴
        self._quat = Quaternion()          # TRACKBALL 的绝对世界旋转
        self._quat0 = Quaternion()         # 视图旋转前的累积，新拖动从它接着转
        self._track_dx = 0.0
        self._track_dy = 0.0
        self._ang_h = 0.0                  # DUAL 水平通道 / SINGLE 的累计角度
        self._ang_v = 0.0                  # DUAL 垂直通道
        self._assign_gesture(context)
        self._sens_base = self._deg_per_px(context)
        self._skip_move = False
        self._pending_warp = False   # 中键导航结束后要把指针送回控制器（延迟到那时才动）
        self._alt_nav = False        # Alt+中键放行给原生 view_axis：那层没退场前不吃 MOVE
        self._orbit_t = 0.0          # 最近一次交出去做视图操作的时刻
        self._solo_saved = None      # solo 显示：进模态时的骨骼显隐快照
        self._tot_rot = 0.0          # 跨模式切换的累计旋转（度）
        self._tot_move = 0.0         # 跨模式切换的累计位移
        self._zone_hit = False       # 指针是否正贴着活动半径边界（HUD 提示用）
        # v4.9.93：我们「最后一次亲手写出去」的姿态。动画重估（切帧 / 别处插帧 /
        # 撤销）会把 matrix_basis 拽回当前帧的关键帧值，靠这份记录才能认出来
        # 并且把姿态写回去，而不是把被拽回去的旧姿态当成新基线接受下来。
        self._last_pose = {}
        self._key_t = 0.0            # 上次同步关键帧的时刻（限流用）
        self._zone_hit = False       # 指针是否正贴着活动半径边界（HUD 提示用）
        # 先抓“进入 solo 之前”的可见性；导航用这份快照，
        # 这样 ARP/Rigify 隐藏的 IK 会跳过，solo 隐藏的其他控制器不会被误删。
        global _NAV_VIS, _NAV_VIS_RIG
        _NAV_VIS = None
        _NAV_VIS_RIG = None
        _NAV_VIS = {b.name: bone_visible(b) for b in self._rig.pose.bones}
        _NAV_VIS_RIG = self._rig

        if self.start_mode == 'MOVE':
            self._move_begin(context)
        self._solo_apply(context)    # 只显示当前控制器，滚轮换骨骼时跟着换

        if _has_active_ik(pb):
            self.report({'WARNING'},
                        "%s 上有生效的 IK 约束，直接改姿态会被约束覆盖" % pb.name)

        if pref('show_hud', True) and _hud_available() and not bpy.app.background:
            try:
                self._handle = bpy.types.SpaceView3D.draw_handler_add(
                    self._draw_hud, (context,), 'WINDOW', 'POST_PIXEL')
                context.window.cursor_set('NONE')
                self._cursor_hidden = True
            except Exception:
                self._handle = None
                self._cursor_hidden = False

        self._header(context)
        _MODAL_ONCE[0] = True
        try:
            context.window_manager.modal_handler_add(self)
        except Exception:
            _MODAL_ONCE[0] = False
            self._clear(context)
            raise
        return {'RUNNING_MODAL'}

    # ---- 手势分级

    def _assign_gesture(self, context):
        """按未锁轴的数量分级，两轴时用真实屏幕投影分配水平/垂直通道。

        分配结果写进 _mode / _ax_h / _ax_v / _ax_s 和 _dir_win（窗口坐标方向，
        y 向下，和 event.mouse_y 同一坐标系），HUD 与 header 直接展示，不猜。
        """
        pb = self._pb
        unlocked = [i for i in range(len(ROT_AXES)) if not axis_locked_eff(pb, i)]
        self._ax_h = self._ax_v = self._ax_s = None
        self._dir_win = {}
        # 单轴方向映射（v4.9.99）：进场/换轴/换视角后第一次拖动时懒算一次
        self._s_mode = None           # 'RIM' 轮缘式 / 'LIN' 线性式
        self._s_sign = 1.0            # RIM：轴朝观察者 +1，背向 -1
        self._s_u = None              # LIN：屏幕内固定方向（y 向上）
        self._s_pv = None             # RIM：枢轴的屏幕位置（region，y 向上）
        self._s_phi = None            # RIM：上一帧指针的方位角（度）
        self._rim_acc = 0.0           # RIM：累计方位角（度）

        if self._axis_lock is not None and self._axis_lock in unlocked:
            self._mode = 'SINGLE'
            self._ax_s = self._axis_lock
            d = self._tip_dirs(context)[self._axis_lock]
            if d is not None:
                self._dir_win[self._axis_lock] = Vector((d.x, -d.y))
            return

        if len(unlocked) == 0:
            self._mode = 'LOCKED'
            return
        if len(unlocked) >= 3:
            self._mode = 'TRACKBALL'
            return
        if len(unlocked) == 1:
            self._mode = 'SINGLE'
            self._ax_s = unlocked[0]
            d = self._tip_dirs(context)[unlocked[0]]
            if d is not None:
                self._dir_win[unlocked[0]] = Vector((d.x, -d.y))
            return

        # 两根轴：屏幕投影最接近水平的给水平通道，剩下的给垂直通道
        dirs = self._tip_dirs(context)
        for i in unlocked:
            if dirs[i] is not None:
                self._dir_win[i] = Vector((dirs[i].x, -dirs[i].y))

        h_best, h_score = None, -2.0
        for i in unlocked:
            d = self._dir_win.get(i)
            if d is None:
                continue
            if abs(d.x) > h_score:
                h_score, h_best = abs(d.x), i
        if h_best is None:
            h_best = unlocked[0]
        rest = [i for i in unlocked if i != h_best]
        v_best = rest[0] if rest else unlocked[-1]
        self._mode = 'DUAL'
        self._ax_h, self._ax_v = h_best, v_best

    # ---- 反馈

    def _mode_line(self):
        """当前手势的一句话说明，HUD 和 header 共用。"""
        if getattr(self, "_scaling", False):
            return "缩放 x%.2f" % self._scale_factor
        if self._kind == 'MOVE':
            if self._move_axis is None:
                return "移动（自由）"
            return "移动  全局 %s 轴" % MOVE_AXES[self._move_axis]
        if self._mode == 'LOCKED':
            return "全部轴锁定"
        if self._mode == 'TRACKBALL':
            return "自由旋转"
        if self._mode == 'DUAL':
            ah = ROT_AXES[self._ax_h][0]
            av = ROT_AXES[self._ax_v][0]
            return "水平→%s轴  垂直→%s轴" % (ah, av)
        return "单轴 %s（原生 rotate）" % ROT_AXES[self._ax_s][0]

    def _angles_line(self):
        """当前角度的文本（各通道吸附后）。"""
        if getattr(self, "_scaling", False):
            return "x%.3f" % self._scale_factor
        if self._kind == 'MOVE':
            # 自由移动没有「沿轴的距离」，用累计位移长度（原来恒显示 +0.0000）
            if self._move_axis is None:
                return "%+.3f" % getattr(self, '_mv_pending', Vector()).length
            return "%+.3f" % self._move_amount
        if self._mode == 'TRACKBALL':
            total = self._quat.angle
            return "%+.1f°" % math.degrees(total) if total > 1e-4 else "0.0°"
        if self._mode == 'DUAL':
            return "%s %+.1f°   %s %+.1f°" % (
                ROT_AXES[self._ax_h][0], self._eff_deg(self._ang_h),
                ROT_AXES[self._ax_v][0], self._eff_deg(self._ang_v))
        return "%+.1f°" % self._eff_deg(self._ang_h)

    def _header(self, context):
        """刷新状态反馈：视口顶部文字 + 鼠标旁 HUD。

        HUD 是 SpaceView3D 的 draw handler，只有该区域重绘才会跑；
        换轴/换骨骼/切模式时没有鼠标移动 → 不重绘 → 文字停在旧值上。
        所以这里必须显式 tag_redraw，让 HUD 跟着状态立刻变。
        """
        area = getattr(context, "area", None)
        if area is None:
            return
        snap = ("  吸附 %g°" % self._snap_step) if self._snap_step else ""
        if getattr(self, '_undo_none', False):
            snap += "  已退到进入模态时的状态（左键确认 / 双击右键退出）"
        mk = move_key_label() if move_key_ok() else "移动键"
        rk = rot_key_label() if pref('use_x_rotate', True) else "旋转键"
        xyz = "X/Y/Z 锁单轴 · " if pref('use_xyz_keys', True) else ""
        # 滚轮那几项按偏好里的实际绑定显示，别在 HUD 上写死一个早就改掉的组合
        wh_parts = []
        for _act, _def, _word in (('ZOOM', 'SHIFT', "缩放视图"),
                                  ('AXIS', 'BARE', "切轴"),
                                  ('COLL', 'CTRL_ALT', "切集合"),
                                  ('BONE', 'ALT', "切骨骼")):
            _lab = _wheel_label(_act, _def)
            if _lab:
                wh_parts.append("%s %s" % (_lab, _word))
        wh = " · ".join(wh_parts)
        # 旋转里只提示「切移动」（X 此刻是锁轴不是切旋转），移动里只提示「切旋转」
        hint = (xyz + "S 缩放 · " + wh + " · 中键单击清旋转 · %s 切移动 · 中键转视角 / Alt+中键对齐正交 / Shift 平移 / Ctrl 缩放 · 左键确认 · 右键退一步 · 双击右键全撤"
                % mk
                if self._kind == 'ROT' else
                xyz + wh + " · %s 切旋转 · 中键转视角 / Alt+中键对齐正交 / Shift 平移 / Ctrl 缩放 · 左键确认 · 右键退一步 · 双击右键全撤" % rk)
        # 实时灵敏度读数：调偏好里的倍率时能看到真正生效的量级，
        # 不用靠"感觉快了没有"来猜。TRACKBALL 走曲线，取起手斜率当读数。
        try:
            if self._kind == 'ROT' and self._mode == 'TRACKBALL':
                sens_txt = "   %.2f°/px 起手" % self._native_curve_deg(1.0)
            elif self._kind == 'ROT':
                sens_txt = "   %.2f°/px" % self._sens_base
            else:
                sens_txt = ""
        except Exception:
            sens_txt = ""
        area.header_text_set(
            "%s  %s  ·  %s  ·  %s%s%s%s    %s"
            % ("自由旋转" if self._kind == 'ROT' else "移动",
               self._pb.name, self._mode_line(), self._angles_line(),
               self._total_line(), snap, sens_txt, hint))
        try:
            area.tag_redraw()
        except Exception:
            pass

    # ---- 姿态写入

    def _eff_deg(self, raw):
        """吸附后的角度（度）。吸附作用在总量上，不是逐帧增量，所以不会漂移。"""
        if self._snap_step:
            return round(raw / self._snap_step) * self._snap_step
        return raw

    # ---- 移动（v4.9.13 改为原生 G 驱动）

    def _px_scale(self, region, rv3d):
        """每像素对应多少世界单位 —— 用真实投影反算，保证「手走多少，画面走多少」。

        v4.9.96：原来用 `view_distance·2·tan(20°)/height` 这个近似式。它只对
        「透视 + 约 40° 垂直 FOV + 视距恰好等于枢轴深度」成立；正交视口下
        view_distance 与可见高度的关系完全不同。实测（pose 瞎玩.blend，
        正交，view_distance 1.350，视口 1188×1151）：

            真实 0.001637 m/px      公式 0.000854 m/px      只有 52%

        也就是手拖 100px，骨骼只在屏幕上走 52px —— 用户说的「按着控件
        不跟鼠标 / 跟得很慢」。而且正交与透视差一倍、缩放级别不同偏差也不同，
        所以「有时很明显、有时只是慢」，无法稳定复现。

        现在直接在枢轴处投影 1 个世界单位的相机右向量，量屏幕上走多少像素，
        再取倒数：正交、透视、任意焦距、任意缩放都精确 1:1。
        """
        try:
            pb = getattr(self, "_pb", None)
            if self._kind == 'MOVE' and pb is not None:
                centre = self._rig.matrix_world @ pb.head
            else:
                centre = self._rig.matrix_world @ self._pivot
            p0 = location_3d_to_region_2d(region, rv3d, centre)
            right = (rv3d.view_matrix.to_3x3().inverted()
                     @ Vector((1.0, 0.0, 0.0))).normalized()
            p1 = location_3d_to_region_2d(region, rv3d, centre + right * 1.0)
            if p0 is not None and p1 is not None:
                d = (p1 - p0).length
                if d > 1e-6:
                    return 1.0 / d
        except Exception:
            pass
        # 枢轴落在视口外（投影失败）时退回旧公式，至少不会崩
        return (max(getattr(rv3d, "view_distance", 1.0), 0.001) * 2.0
                * math.tan(math.radians(20.0)) / max(region.height, 1))

    def _axis_dir(self, region, rv3d, idx):
        """全局轴的 (世界单位向量, 屏幕方向)；轴正对镜头时退化成竖直拖动。"""
        axis_w = (self._rig.matrix_world.to_3x3() @ MOVE_UNIT[idx]).normalized()
        head = self._rig.matrix_world @ self._pb.head
        p0 = location_3d_to_region_2d(region, rv3d, head)
        p1 = location_3d_to_region_2d(region, rv3d, head + axis_w * 0.2)
        if p0 is None or p1 is None or (p1 - p0).length < 1e-5:
            return axis_w, Vector((0.0, 1.0))
        return axis_w, (p1 - p0).normalized()

    def _lock_filter(self, pb, world_d):
        """把世界位移转成该骨骼父空间里的位移，并清掉 lock_location 锁住的分量。"""
        if not any(pb.lock_location):
            return world_d
        rig3 = self._rig.matrix_world.to_3x3()
        arm_d = rig3.inverted() @ world_d
        try:
            r = _parent_matrix(pb).to_3x3()
            local = r.inverted() @ arm_d
            for i in range(3):
                if pb.lock_location[i]:
                    local[i] = 0.0
            return rig3 @ (r @ local)
        except Exception:
            return world_d

    def _move_build(self, context):
        """参与移动的骨骼 = 选中层级里的根（父级也被选中时，子级跟着父级走）。"""
        pb = self._pb
        sel = [b for b in (context.selected_pose_bones or []) if b is not None]
        if pb not in sel:
            sel = [pb] if pb is not None else []
        roots = [b for b in sel if b.parent not in sel] or sel
        if not roots and pb is not None:
            roots = [pb]
        return roots

    def _total_line(self):
        """跨模式切换的累计量，证明「切过去以后上一次的成果还在」。"""
        try:
            if self._kind == 'ROT' and getattr(self, '_tot_rot', 0.0) > 0.05:
                return "   累计 %.1f°" % self._tot_rot
            if self._kind == 'MOVE' and getattr(self, '_tot_move', 0.0) > 1e-4:
                return "   累计 %.3f" % self._tot_move
        except Exception:
            pass
        return ""

    def _move_begin(self, context):
        self._kind = 'MOVE'
        self._move_axis = None
        self._move_amount = 0.0
        self._mv_pending = Vector((0.0, 0.0, 0.0))
        self._mv_native_step = _bone_needs_native(self._pb)
        self._mv_bones = self._move_build(context)
        self._mv_bases = {}
        self._mv_locs = {}
        for b in self._mv_bones:
            bpy.context.view_layer.update()
            self._orig.setdefault(b.name, b.matrix_basis.copy())
            self._session.add(b.name)      # 移动过的骨骼也要进快照范围
            self._session0.setdefault(b.name, b.matrix_basis.copy())
            self._mv_bases[b.name] = (b.matrix.copy(), b.matrix_basis.copy())
            self._mv_locs[b.name] = b.location.copy()
        touched = getattr(self, '_touched', None)
        if touched is not None:
            touched |= {b.name for b in self._mv_bones}

    def _move_preview(self):
        """把「还没提交的位移」画出来。

        v4.9.95：这里**不再每帧调 `view_layer.update()`**。
        移动模式跟手性差就是它引起的：每个 MOUSEMOVE 都强制求值一次依赖图，
        这套 420 骨的 rig + 角色网格实测 3.4ms/次，鼠标 125Hz 时事件开始排队，
        画面比手慢好几帧 —— 就是「跟得很慢」，而且轻重随场景/视角变化
        （"有时候很明显、有时候只是慢"）。旋转路径从不 update，所以只有移动有这毛病。

        实测（bench_move）：写 `matrix` 不求值 0.0066ms，求值 3.36ms（577×），
        结果完全一致（误差 7e-15）；求值交给下一次重绘统一做一次即可。

        pole 控制器不写 matrix，改写 location（父空间），让约束/父级重新求值，
        避免脚部剧烈旋转后直接写 world matrix 造成跳变。
        """
        rig3 = self._rig.matrix_world.to_3x3()
        for b in self._mv_bones:
            base = self._mv_bases.get(b.name)
            if base is None:
                continue
            try:
                world_d = self._lock_filter(b, self._mv_pending)
                if 'pole' in b.name.lower():
                    P = _parent_matrix(b).to_3x3()
                    local_d = P.inverted() @ (rig3.inverted() @ world_d)
                    loc0 = (getattr(self, "_mv_locs", {}) or {}).get(b.name)
                    if loc0 is None:
                        loc0 = base[1].to_translation()
                    b.location = loc0 + local_d
                else:
                    P = _parent_matrix(b)
                    if (base[1] is not None
                            and _constraint_shifts(P, base[0], base[1])):
                        # 可见姿态被约束改写过（Child Of 等）：直接写 matrix 会把
                        # 那层偏移再叠一遍，首帧整根控制器就弹走。改走带 K 的共轭。
                        _apply_visible_translation(b, P, base[0], base[1], world_d)
                    else:
                        b.matrix = Matrix.Translation(world_d) @ base[0]
            except Exception:
                pass
        # 移动也要记账：不记的话下一次 _ensure_baseline 会把这帧位移当成
        # 「别人改的姿势」还原回去，表现就是拖不动 / 一松手弹回。
        self._remember_pose(move=True)

    def _native_translate(self, context, bones, world_delta):
        """调用原生 transform.translate（= G 键同一个操作符）走完这段位移。"""
        rig = self._rig
        ts = getattr(context.scene, "tool_settings", None)
        prop_saved = getattr(ts, "use_proportional_edit", None) if ts else None
        # v5.2.36 提速：原生 transform 开着自动关键帧时实测 254ms/次（420 骨 + 角色），
        # 关掉后 5.3ms —— 差 48 倍，切模式/切骨那一下的卡顿全在这里。落地过程不需要
        # 它插帧：同一个提交点由 _insert_keys 补（单通道 0.01ms），最终结果一致。
        auto_saved = getattr(ts, "use_keyframe_insert_auto", None) if ts else None
        saved = [(p, p.select) for p in rig.pose.bones if p.select]
        try:
            for p, _s in saved:
                p.select = False
            for b in bones:
                b.select = True
            try:
                rig.data.bones.active = bones[0].bone
            except Exception:
                pass
            if ts is not None:
                ts.use_proportional_edit = False   # 别让比例编辑把别的骨骼带上
            if ts is not None and auto_saved is not None:
                ts.use_keyframe_insert_auto = False
            bpy.ops.transform.translate(value=world_delta, orient_type='GLOBAL')
            return True
        except Exception:
            return False
        finally:
            if ts is not None and prop_saved is not None:
                try:
                    ts.use_proportional_edit = prop_saved
                except Exception:
                    pass
            if ts is not None and auto_saved is not None:
                try:
                    ts.use_keyframe_insert_auto = auto_saved
                except Exception:
                    pass
            for p, s in saved:
                try:
                    p.select = s
                except Exception:
                    pass

    def _native_step(self, context, world_delta):
        """用原生 transform.exec 写入一帧位移：约束/Child Of 正常求值。"""
        if world_delta.length < 1e-9:
            return
        edit = getattr(context.preferences, "edit", None)
        ts = getattr(context.scene, "tool_settings", None)
        saved_undo = getattr(edit, "use_global_undo", None)
        saved_auto = getattr(ts, "use_keyframe_insert_auto", None)
        try:
            if edit is not None and saved_undo is not None:
                edit.use_global_undo = False
            if ts is not None and saved_auto is not None:
                ts.use_keyframe_insert_auto = False
            for b in self._mv_bones:
                d = self._lock_filter(b, world_delta)
                if d.length < 1e-9:
                    continue
                self._native_translate(context, [b], d)
            bpy.context.view_layer.update()
            self._remember_pose(move=True)
        finally:
            if edit is not None and saved_undo is not None:
                try:
                    edit.use_global_undo = saved_undo
                except Exception:
                    pass
            if ts is not None and saved_auto is not None:
                try:
                    ts.use_keyframe_insert_auto = saved_auto
                except Exception:
                    pass
        bpy.context.view_layer.update()

    def _move_flush(self, context):
        """把累积位移交给原生 G 落地，并把预览基准更新到新状态。"""
        pend = self._mv_pending
        self._mv_pending = Vector((0.0, 0.0, 0.0))
        self._move_amount = 0.0
        if pend.length < 1e-9 or not self._mv_bones:
            return
        rig3 = self._rig.matrix_world.to_3x3()
        if getattr(self, "_mv_native_step", False):
            # 每一帧已经由 native transform 写入；这里只更新基准，不再叠加
            self._mv_bases = {}
            self._mv_locs = {}
            for b in self._mv_bones:
                bpy.context.view_layer.update()
                self._session.add(b.name)
                self._session0.setdefault(b.name, b.matrix_basis.copy())
                self._mv_bases[b.name] = (b.matrix.copy(), b.matrix_basis.copy())
                self._mv_locs[b.name] = b.location.copy()
            return
        if any('pole' in b.name.lower() for b in self._mv_bones):
            # pole 的 location 预览已经在当前姿态上，直接更新基准，不要再调原生 G
            self._mv_bases = {}
            self._mv_locs = {}
            for b in self._mv_bones:
                bpy.context.view_layer.update()
                self._session.add(b.name)
                self._session0.setdefault(b.name, b.matrix_basis.copy())
                self._mv_bases[b.name] = (b.matrix.copy(), b.matrix_basis.copy())
                self._mv_locs[b.name] = b.location.copy()
            return
        ok = False
        if len(self._mv_bones) == 1:
            pb = self._mv_bones[0]
            base = self._mv_bases.get(pb.name)
            if base is not None:
                # 预览已经把姿态画上去了，先回滚到基准，否则原生 G 会再叠一倍
                pb.matrix_basis = base[1].copy()
                bpy.context.view_layer.update()
            want = self._lock_filter(pb, pend)     # 世界空间位移，GLOBAL 直接用
            before = pb.matrix.translation.copy()
            ok = self._native_translate(context, [pb], want)
            bpy.context.view_layer.update()
            # v4.9.95：原生 translate 在某些上下文里会返回 FINISHED 却一格不动
            # （实测 pose 瞎玩.blend 的 c_root_master.x 就是这样）。它不动就等于
            # 把刚回滚掉的位移丢了 —— 必须验一下，没动就走下面的兜底。
            if ok and (pb.matrix.translation - before).length < max(
                    want.length * 0.5, 1e-6):
                ok = False
        if not ok:
            # 多骨骼（各根锁不同）或原生调用失败：直接写 matrix_basis，结果等价
            for b in self._mv_bones:
                base = self._mv_bases.get(b.name)
                if base is None:
                    continue
                try:
                    d = self._lock_filter(b, pend)
                    P = _parent_matrix(b)
                    if (base[1] is not None
                            and _constraint_shifts(P, base[0], base[1])):
                        _apply_visible_translation(b, P, base[0], base[1], d)
                    else:
                        b.matrix = Matrix.Translation(d) @ base[0]
                except Exception:
                    pass
            bpy.context.view_layer.update()
        # v5.2.36：原生 transform 里的 autokey 已临时关掉，位移落地后在这里补帧
        self._insert_keys(context)
        self._mv_bases = {}
        self._mv_locs = {}
        for b in self._mv_bones:
            bpy.context.view_layer.update()
            self._session.add(b.name)
            self._session0.setdefault(b.name, b.matrix_basis.copy())
            self._mv_bases[b.name] = (b.matrix.copy(), b.matrix_basis.copy())
            self._mv_locs[b.name] = b.location.copy()

    def _move_step(self, context, dx, dy):
        region, rv3d = _view_pair(context)
        if region is None or rv3d is None or not self._mv_bones:
            return
        # 约定：模态传进来的 dy 是「向下为正」（dy = -(mouse_y 增量)），
        # 而视口的上方向向量和屏幕投影的 y 都是「向上为正」，
        # 所以这里先翻成 up_px 再参与计算，v4.9.13 忘了翻 → 鼠标往上骨骼往下。
        up_px = -dy
        scale = self._px_scale(region, rv3d)
        right, up = _view_axes(rv3d)
        if self._move_axis is None:
            world_d = right * (dx * scale) + up * (up_px * scale)
        else:
            axis_w, d2 = self._axis_dir(region, rv3d, self._move_axis)
            amount = (dx * d2.x + up_px * d2.y) * scale
            self._move_amount += amount
            world_d = axis_w * amount
        if getattr(self, "_mv_native_step", False):
            # 每一帧都交给原生 transform，约束每帧重新求值
            self._native_step(context, world_d)
            self._move_amount += world_d.length
            return
        # 关键：位移必须累积。原来每帧只拿“这一帧的增量”去乘起始矩阵，
        # 于是骨骼只在起点附近抖动 —— 看起来就是“根本移动不了”。
        self._mv_pending += world_d
        self._move_preview()

    def _cycle_move_axis(self, step):
        seq = MOVE_CYCLE
        i = seq.index(self._move_axis) if self._move_axis in seq else 0
        self._move_axis = seq[(i + step) % len(seq)]
        self._move_amount = 0.0

    def _pose_matches(self, snap, eps=1e-6):
        """当前姿态和快照是否一致（用来判断"这一步还能不能退"）。"""
        for name, mat in snap.items():
            pb = _pb_get(self._rig, name)
            if pb is None:
                continue
            cur = pb.matrix_basis
            for r in range(4):
                for c in range(4):
                    if abs(cur[r][c] - mat[r][c]) > eps:
                        return False
        return True

    def _make_step(self):
        """抓一份「整段会话」的姿态快照，连同焦点控制器一起记下来。

        快照按 _session（这次模态里碰过的所有骨骼）取，而不是只取当前选中的。
        右键要能一路退回上一个控制器，快照里就必须连带记着那些骨骼的状态，
        否则退回跨控制器的那一步时只能还原一半。
        """
        bones = {}
        for name in sorted(getattr(self, '_session', ())):
            pb = self._rig.pose.bones.get(name)
            if pb is not None:
                bones[name] = pb.matrix_basis.copy()
        pb = getattr(self, '_pb', None)
        return {
            'bones': bones,
            'focus': pb.name if pb is not None else None,
            'targets': [b.name for b in (getattr(self, '_targets', None) or [])],
            'kind': getattr(self, '_kind', 'ROT'),
            'axis': getattr(self, '_axis_lock', None),
        }

    def _step_matches(self, a, b, eps=1e-6):
        """两个快照是否算同一状态（焦点/目标/姿态都要一样）。"""
        if a.get('focus') != b.get('focus') or a.get('kind') != b.get('kind'):
            return False
        if list(a.get('targets') or []) != list(b.get('targets') or []):
            return False
        ba, bb = a['bones'], b['bones']
        if set(ba) != set(bb):
            return False
        for name, ma in ba.items():
            mb = bb[name]
            for r in range(4):
                for c in range(4):
                    if abs(ma[r][c] - mb[r][c]) > eps:
                        return False
        return True

    def _push_undo_step(self):
        """在提交点记录一步，并把当前指针移到这一步。"""
        try:
            step = self._make_step()
        except Exception:
            return
        steps = getattr(self, '_steps', None)
        if steps is None:
            self._steps = [step]
            self._steps_i = 0
            return
        # 撤销过以后又做了新操作：丢掉后面的分支
        if getattr(self, '_steps_i', len(steps) - 1) < len(steps) - 1:
            del steps[self._steps_i + 1:]
        if steps and self._steps_i == len(steps) - 1 and self._step_matches(steps[-1], step, 1e-4):
            steps[-1] = step
            self._steps_i = len(steps) - 1
            return
        steps.append(step)
        self._steps_i = len(steps) - 1

    def _undo_step_back(self, context):
        """右键：先回到当前段的起点，再逐级退回上一段。"""
        steps = getattr(self, '_steps', None) or []
        if not steps:
            try:
                self._steps = [self._make_step()]
                self._steps_i = 0
            except Exception:
                pass
            # 没有历史可退，但累计旋转必须清掉：留着它，下一帧会把旧角度
            # 重新叠到基线上 —— 用户看到的就是"撤销了却没生效，一动又弹回"
            self._resync_after_undo(context)
            return False
        i = max(0, min(getattr(self, '_steps_i', len(steps) - 1), len(steps) - 1))
        self._steps_i = i
        # 当前有未提交的改动：先退回这一段开始时的姿态，保留同一段的基准
        if not self._pose_matches(steps[i]['bones'], 1e-4):
            self._restore_step(context, steps[i])
            return True
        # 已经在这一段起点：退到上一段
        if i <= 0:
            # 没有更早的步骤了，但姿态可能刚被别的东西改过（比如中键清空），
            # 这里同样要对齐基线与累计量
            self._resync_after_undo(context)
            return False
        self._steps_i = i - 1
        self._restore_step(context, steps[i - 1])
        return True

    def _restore_step(self, context, step):
        """把一步快照原样写回，焦点也切回当时那个控制器。"""
        for name, mat in step['bones'].items():
            pb = self._rig.pose.bones.get(name)
            if pb is not None:
                pb.matrix_basis = mat
        bpy.context.view_layer.update()

        # 快照里就是当时那一段的起点：_orig 按它重建，之后的提交点才有正确基线
        self._orig = {k: v.copy() for k, v in step['bones'].items()}
        focus = self._rig.pose.bones.get(step.get('focus') or '')
        if focus is not None:
            self._pb = focus
            try:
                select_bone(self._rig, focus.name)   # 焦点切回上一个控制器
            except Exception:
                pass
        targets = [self._rig.pose.bones.get(n) for n in (step.get('targets') or [])]
        targets = [b for b in targets if b is not None] or ([focus] if focus else [])
        self._targets = targets
        self._touched = {b.name for b in targets}
        # 右键只回退姿态/焦点，不改变当前模式：
        # 在移动模式里右键不会突然跳回旋转模式，反之亦然。
        if self._kind == 'MOVE':
            self._axis_lock = None
        self._resync_after_undo(context)
        if self._kind == 'ROT':
            if step.get('kind') == 'ROT':
                ax = step.get('axis')
                if ax is not None and self._pb is not None and axis_locked_eff(self._pb, ax):
                    ax = None
                self._axis_lock = ax
            try:
                self._assign_gesture(context)
                self._sens_base = self._deg_per_px(context)
            except Exception:
                pass
        try:
            self._solo_apply(context)             # 可见集跟着焦点换回上一个控制器
        except Exception:
            pass

    def _resync_after_undo(self, context):
        """回退一步之后把这一段的基准全部对齐到当前姿态。

        和 _rebase 同一个道理：旋转每帧都从 _states 重算，不改基准的话
        下一次拖动的第一帧就会把刚退回去的姿态又抹掉。
        """
        self._rebaseline(context)
        self._reset_gesture_accum()
        # 撤销/清空不仅要把姿态写回来，还要把它写进当前帧的关键帧：
        # 曲线上留着撤销前的值，一次动画重估就会把画面拽回去。
        self._sync_keys(context, force=True)
        self._update_motion_paths(context)   # v5.2.30：回退后路径跟着回去
        if self._kind == 'MOVE':
            ax = self._move_axis          # 回退不该把已经选好的移动轴也重置掉
            try:
                self._move_begin(context)
            except Exception:
                pass
            self._move_axis = ax

    def _rebaseline(self, context=None):
        """把当前姿态冻结成新基线（_states / _pb 的引用量 / _orig）。

        `_states` 是「每帧重算的出发点」，只要姿态被我们以外的东西改过
        （撤销、中键清空、关键帧重估），它就必须跟着换，否则下一帧 `_apply`
        会从过期基线重算，把刚撤掉的姿态又写回去。
        """
        bpy.context.view_layer.update()
        targets = list(self._targets)
        if not targets and self._pb is not None:
            targets = [self._pb]        # 目标列表空时用焦点兜底，别留空基线
        self._states = {}
        for b in targets:
            bpy.context.view_layer.update()
            self._capture_state(b)
            try:
                self._orig[b.name] = b.matrix_basis.copy()
            except Exception:
                pass
        pb = self._pb
        if pb is not None:
            self._start = pb.matrix_basis.copy()
            self._m_start = pb.matrix.copy()
            self._parent_m = _parent_matrix(pb)
            if context is not None:
                try:
                    self._pivot = _pivot_armature_space(context, self._rig, pb)
                except Exception:
                    pass
        self._remember_pose()

    def _reset_gesture_accum(self):
        """把「未落地的输入」全部清零。撤销/清空之后必须做，否则旧累计量
        会在下一帧被重新叠到新基线上 —— 表现就是"撤销完动一下就弹回去"。"""
        self._quat = Quaternion()
        self._quat0 = Quaternion()
        self._track_dx = 0.0
        self._track_dy = 0.0
        self._ang_h = 0.0
        self._ang_v = 0.0
        self._mv_pending = Vector((0.0, 0.0, 0.0))
        self._move_amount = 0.0
        self._tot_rot = 0.0
        self._tot_move = 0.0
        # 撤销/清空之后重新取一次参考点：右键、中键落下时指针可能已经跟
        # 撤销前的拖动差了很远，直接把那一截位移当输入会让姿态跳一下。
        self._primed = False

    def _capture_state(self, b):
        """记下这一帧的基线：可见矩阵 + 父级映射 + 枢轴 + matrix_basis。

        多存一份 matrix_basis 是为了识别「带约束的骨骼」：它的 b.matrix 里
        混着约束的结果，不能拿来反解 basis（见 _apply）。
        """
        try:
            self._states[b.name] = (b.matrix.copy(), _parent_matrix(b),
                                    b.matrix.translation.copy())
            self._states_basis[b.name] = b.matrix_basis.copy()
        except Exception:
            pass

    def _remember_pose(self, move=False):
        """记下我们「最后一次亲手写出去」的姿态（matrix_basis）。

        move=True 时把参与位移的骨骼也记进来 —— 位移是写 `b.matrix` 的，
        参与的可能包含 _targets 之外的层级根。
        """
        pose = {}
        bones = list(getattr(self, '_targets', None) or [])
        if move:
            bones += list(getattr(self, '_mv_bones', None) or [])
        for b in bones:
            try:
                pose[b.name] = b.matrix_basis.copy()
            except Exception:
                pass
        self._last_pose = pose

    def _pose_drifted(self, eps=1e-4):
        """姿态是否被我们之外的东西改过（动画重估、撤销、外部脚本）。"""
        last = getattr(self, '_last_pose', None) or {}
        if not last:
            return False
        for name, mat in last.items():
            pb = self._rig.pose.bones.get(name)
            if pb is None:
                continue
            cur = pb.matrix_basis
            for r in range(4):
                for c in range(4):
                    if abs(cur[r][c] - mat[r][c]) > eps:
                        return True
        return False

    def _ensure_baseline(self, context=None):
        """姿态被动画重估拽回去时，**把我们自己的姿态写回来**，不要接受它。

        v4.9.93：这里原来是「骨骼矩阵和基线对不上就重建基线」——那等于把
        被拽回去的旧姿态当成新基线**接受**下来。而动作在当前帧本来就有关键帧
        （上一次确认留下的），一旦 Blender 重估动画，matrix_basis 会被写成
        那条曲线的值；下一次鼠标移动就变成「从旧姿态接着转」，用户看到的是
        「撤销完轻移一下，画面跳回撤销前」。
        现在改成：只要发现姿态和我们最后写出去的姿态不一致，就先写回来，
        再对齐基线。写回来之后画面不动，一切照旧。
        """
        if self._mode == 'LOCKED':
            return False
        if not self._pose_drifted():
            return False
        for name, mat in (getattr(self, '_last_pose', None) or {}).items():
            pb = self._rig.pose.bones.get(name)
            if pb is None:
                continue
            try:
                pb.matrix_basis = mat
            except Exception:
                pass
        bpy.context.view_layer.update()
        self._rebaseline(None)
        # v5.2.38：基线已经换成「含这段旋转的姿态」，累计量必须同时清零。
        # 不清的话下一帧 _apply 会把这同一段旋转**再叠一次**：实测 q=36° 时
        # 单帧多转 36°，q=98° 时多转 98° —— 用户看到的正是「转着转着姿态
        # 猛地甩一下」，而 HUD 上的角度还在原地，越甩越离谱。
        self._bake_gesture()
        if context is not None:
            self._sync_keys(context, force=True)   # 曲线也跟上，否则下次重估又被拽走
        return True

    def _bake_gesture(self):
        """把「已经写进姿态的这一段手势」记进累计、并把累计量清零。

        rebaseline 之后 `_states` 已经是含这段旋转/位移的姿态，而 `_apply`
        每次都从 `_states` 出发重新叠绝对角度 —— 所以累计量必须跟着归零，
        否则同一段旋转会被叠加第二遍（位移走的是 `_mv_bases` + `_mv_pending`，
        自带基准，不受影响）。HUD 的「累计」先记一笔，读数不会倒退。
        """
        try:
            if self._kind == 'MOVE':
                return      # 位移的基准在 _mv_bases，rebaseline 不改它，无需动
            if self._mode == 'TRACKBALL':
                self._tot_rot = getattr(self, '_tot_rot', 0.0) + math.degrees(self._quat.angle)
            else:
                self._tot_rot = (getattr(self, '_tot_rot', 0.0)
                                 + abs(self._ang_h) + abs(self._ang_v))
        except Exception:
            pass
        self._quat = Quaternion()
        self._quat0 = Quaternion()
        self._track_dx = 0.0
        self._track_dy = 0.0
        self._ang_h = 0.0
        self._ang_v = 0.0
        # 单轴 RIM 的方位角累计同样要清：不清的话下一帧 `_ang_h` 会被
        # `_rim_acc` 重新算回旧总量，等于又叠一遍。`_s_phi = None` 让它从
        # 当前指针位置重新起算，那一帧只取基准、不产生角度。
        self._rim_acc = 0.0
        self._s_phi = None

    def _native_rotate(self, context, bones, axis_idx, deg, pivot=None):
        """调用原生 transform.rotate（= 按 R 再按轴 的同一个操作符）落地这段旋转。

        orient_type='LOCAL' + constraint_axis + center_override 的组合实测
        （test_native_rot_axis.py）与插件自己的矩阵算法完全等价：绕骨骼自身该轴、
        以骨骼头为枢轴、父级补偿一致，差 < 0.001°。所以真正执行可以交给原生，
        数值不变，换来的是和 gizmo / R+轴 一模一样的算子语义。
        """
        rig = self._rig
        if rig is None or not bones:
            return False
        ts = getattr(context.scene, "tool_settings", None)
        prop_saved = getattr(ts, "use_proportional_edit", None) if ts else None
        # v5.2.36 提速：原生 transform 开着自动关键帧时实测 252ms/次（420 骨 + 角色），
        # 关掉后 5.7ms —— 差 44 倍，切模式 / 切轴那一下的卡顿全在这里。落地过程不需要
        # 它插帧：同一个提交点由 _insert_keys 补（单通道 0.01ms），最终结果一致。
        auto_saved = getattr(ts, "use_keyframe_insert_auto", None) if ts else None
        saved = [(p, p.select) for p in rig.pose.bones if p.select]
        try:
            for p, _s in saved:
                p.select = False
            for b in bones:
                b.select = True
            try:
                rig.data.bones.active = bones[0].bone
            except Exception:
                pass
            if ts is not None:
                ts.use_proportional_edit = False   # 别让比例编辑把别的骨骼带上
            if ts is not None and auto_saved is not None:
                ts.use_keyframe_insert_auto = False
            name = ROT_AXES[axis_idx][0]
            try:
                if pivot is not None:
                    pivot_w = rig.matrix_world @ pivot
                else:
                    pivot_w = rig.matrix_world @ self._pivot
            except Exception:
                pivot_w = rig.matrix_world @ bones[0].matrix.translation.copy()
            bpy.ops.transform.rotate(
                value=math.radians(deg),
                orient_axis=name,
                orient_type='LOCAL',
                constraint_axis=tuple(n == name for n in ('X', 'Y', 'Z')),
                center_override=pivot_w,
            )
            return True
        except Exception:
            return False
        finally:
            if ts is not None and prop_saved is not None:
                try:
                    ts.use_proportional_edit = prop_saved
                except Exception:
                    pass
            if ts is not None and auto_saved is not None:
                try:
                    ts.use_keyframe_insert_auto = auto_saved
                except Exception:
                    pass
            for p, s in saved:
                try:
                    p.select = s
                except Exception:
                    pass

    def _flush_rotation(self, context):
        """提交点：把「这一段转出来的角度」交给原生 rotate 落地。

        预览阶段每帧直接写 matrix_basis（流畅、不往 undo 栈里塞东西）；到
        切骨骼 / 切轴 / 切模式 / 左键确认这些提交点，先回滚到本段基线，再用原生
        操作符一次转完 —— 结果与预览数值一致（已实测等价），但姿态是由原生
        rotate 写出来的。多骨骼目标或原生调用失败时什么都不做：预览的结果本来
        就是对的，退回原状态即可。
        """
        if self._kind != 'ROT' or getattr(self, '_mode', None) != 'SINGLE':
            return False
        if len(getattr(self, '_targets', []) or []) != 1 or self._pb is None:
            return False
        try:
            deg = self._eff_deg(self._ang_h)
        except Exception:
            return False
        if abs(deg) < 1e-9:
            return False
        b = self._targets[0]
        st = getattr(self, '_states', {}).get(b.name)
        if st is None:
            return False
        m_start, parent_m, head_pivot = st
        basis_st = getattr(self, '_states_basis', {}).get(b.name)
        snap = b.matrix_basis.copy()
        try:
            # v4.9.94：回滚必须写回 basis 原件。带约束的骨骼不能用
            # P⁻¹·m_start 反解（实测差 76°，切轴那一下就是它跳的）。
            _restore_to_start(b, parent_m, m_start, basis_st)
            bpy.context.view_layer.update()
        except Exception:
            return False
        # 枢轴跟预览对齐：预览一直是绕 bone.matrix.translation 转的，
        # 这里若改用场景枢轴（self._pivot），提交瞬间就会绕另一个点转一下。
        if self._native_rotate(context, [b], self._ax_s, deg, pivot=head_pivot):
            # v5.2.36：原生那次调用里 autokey 是临时关掉的，这里补回来 ——
            # 与旧版「交给原生插帧」等价（keyframe_insert 单通道 0.01ms）
            self._insert_keys(context)
            return True
        # 原生失败：把预览的姿态放回去，行为与之前完全一致
        try:
            b.matrix_basis = snap
            bpy.context.view_layer.update()
        except Exception:
            pass
        return False

    # ---- 缩放子模式（Rigify 手指伸展/收屈）

    def _scale_begin(self, context, event):
        self._rebased_before_scale = False
        self._scale_states = {}
        for b in self._targets:
            try:
                self._scale_states[b.name] = b.matrix.copy()
            except Exception:
                pass
        self._scaling = True
        self._scale_factor = 1.0
        self._scale_mouse = (event.mouse_x, event.mouse_y)

    def _scale_apply(self, event):
        dx = event.mouse_x - self._scale_mouse[0]
        fine = 0.15 if getattr(event, "shift", False) else 1.0
        f = max(0.01, 1.0 + dx * 0.005 * fine)
        self._scale_factor = f
        for b in self._targets:
            base = self._scale_states.get(b.name)
            if base is None:
                continue
            try:
                # 只缩本地 Y（骨骼长度方向），不缩 X/Z
                b.matrix = base @ Matrix.Diagonal((1.0, f, 1.0, 1.0))
            except Exception:
                pass
        bpy.context.view_layer.update()
        self._remember_pose()

    def _scale_commit(self, context):
        self._scaling = False
        for b in self._targets:
            try:
                self._capture_state(b)
                self._orig[b.name] = b.matrix_basis.copy()
            except Exception:
                pass
        pb = self._pb
        if pb is not None:
            self._m_start = pb.matrix.copy()
            self._parent_m = _parent_matrix(pb)
            self._pivot = _pivot_armature_space(context, self._rig, pb)
            self._start = pb.matrix_basis.copy()
        self._quat = Quaternion()
        self._quat0 = Quaternion()
        self._track_dx = 0.0
        self._track_dy = 0.0
        self._ang_h = 0.0
        self._ang_v = 0.0
        self._assign_gesture(context)
        try:
            self._push_undo_step()
        except Exception:
            pass

    def _scale_cancel(self, context):
        self._scaling = False
        for b in self._targets:
            base = self._scale_states.get(b.name)
            if base is None:
                continue
            try:
                b.matrix = base
            except Exception:
                pass
        bpy.context.view_layer.update()
        self._scale_factor = 1.0

    def _rebase(self, context):
        """把「当前姿态」冻结成新的基线：之后的旋转/移动都在它上面叠加。

        为什么必须有这一步：旋转的每帧都是「从 _states 里的起始矩阵出发、绕枢轴转
        累计角度」重算出来的，而 _states 是进入模态那一刻的快照。一旦中途做了别的
        变换（移动、换轴），姿态已经变了，_states 却还是旧的 —— 下一帧重算就把那次
        变换整个抹掉（看起来就是"切一下模式/切一下轴，刚调好的全清零了"）。
        所以每次切换模式 / 切换轴之前，先把当前姿态写回基线、角度归零。
        """
        # 切换前先记账：HUD 的「累计」跨模式保留，肉眼就能确认继承没丢
        try:
            if self._kind == 'MOVE':
                self._tot_move = getattr(self, '_tot_move', 0.0) + self._mv_pending.length
            elif self._mode == 'TRACKBALL':
                self._tot_rot = getattr(self, '_tot_rot', 0.0) + math.degrees(self._quat.angle)
            else:
                self._tot_rot = (getattr(self, '_tot_rot', 0.0)
                                 + abs(self._ang_h) + abs(self._ang_v))
        except Exception:
            pass
        if self._kind == 'MOVE':
            self._move_flush(context)      # 已经拖出来的位移先落地（走原生 G）
        elif self._kind == 'ROT':
            self._flush_rotation(context)  # 单轴：这段旋转交给原生 rotate 落地
        bpy.context.view_layer.update()
        for b in self._targets:
            try:
                self._capture_state(b)
                # 撤销范围同步收窄：切模式 / 切轴 = 前一段操作算提交了，
                # 右键只回滚到这次切换的时刻，不把旋转+移动整段一起撤掉。
                self._orig[b.name] = b.matrix_basis.copy()
            except Exception:
                pass
        pb = self._pb
        if pb is not None:
            self._m_start = pb.matrix.copy()
            self._parent_m = _parent_matrix(pb)
            self._pivot = _pivot_armature_space(context, self._rig, pb)
            self._start = pb.matrix_basis.copy()
        self._moved_any = False            # 新一段手势，第一条大位移仍要重新取参考点
        self._quat = Quaternion()          # TRACKBALL 的绝对旋转归零（已进基线）
        self._quat0 = Quaternion()
        self._track_dx = 0.0
        self._track_dy = 0.0
        self._ang_h = 0.0
        self._ang_v = 0.0
        self._push_undo_step()             # 切模式 / 切轴 = 一步的结束，右键可退回到这里

    def _cycle_rotate_axis(self, step, context):
        order = (('X', 0), ('Y', 2), ('Z', 1))
        avail = [idx for _, idx in order
                 if not axis_locked_eff(self._pb, idx)]
        if not avail:
            return
        refine = rig_refine_mode(self._rig) and _finger_segment(self._pb) is None
        if refine:
            # 换轴前先把这一轴转出来的角度落进基线，否则上一轴会被清零
            self._rebase(context)
            self._push_undo_step()
            if self._axis_lock is None or self._axis_lock not in avail:
                self._axis_lock = avail[0] if step > 0 else avail[-1]
            else:
                i = avail.index(self._axis_lock)
                nxt = i + (1 if step > 0 else -1)
                self._axis_lock = avail[nxt % len(avail)]
            st = _rig_spatial_settings(self._rig)
            if st is not None:
                st.refine_axis = ROT_AXES[self._axis_lock][0]
            self._quat = Quaternion()
            self._track_dx = 0.0
            self._track_dy = 0.0
            self._ang_h = 0.0
            self._ang_v = 0.0
            self._assign_gesture(context)
            return
        # 换轴前先把这根轴已经转出来的角度落进基线，否则切轴会把上一次旋转清零
        self._rebase(context)
        self._push_undo_step()
        if self._axis_lock is None or self._axis_lock not in avail:
            self._axis_lock = avail[0] if step > 0 else avail[-1]
        else:
            i = avail.index(self._axis_lock)
            nxt = i + (1 if step > 0 else -1)
            self._axis_lock = None if nxt < 0 or nxt >= len(avail) else avail[nxt]
        self._assign_gesture(context)

    def _clear_rotation(self, context):
        """中键单击：等同原生 Alt+R（pose.rot_clear）。

        清空是硬重置，不进入右键撤销历史：
        - 不 push 清空前的姿态
        - 清空后把撤销栈重置成当前清空后的基准
        - 右键不会再把清空前的旋转拿回来
        """
        try:
            bpy.ops.pose.rot_clear()
        except Exception:
            return
        bpy.context.view_layer.update()
        try:
            self._resync_after_undo(context)
        except Exception:
            pass
        try:
            self._steps = [self._make_step()]
            self._steps_i = 0
        except Exception:
            pass
        self._header(context)
        if context.area is not None:
            context.area.tag_redraw()

    def _zoom_view(self, context, step):
        """Shift+滚轮：缩放 3D 视图（wheel up = zoom in）。"""
        try:
            rv3d = context.space_data.region_3d
            factor = 0.9 if step < 0 else 1.111111
            rv3d.view_distance = max(0.01, float(rv3d.view_distance) * factor)
            if context.area is not None:
                context.area.tag_redraw()
        except Exception:
            pass

    def _enter_move(self, context):
        # 切移动之前先把已完成的旋转冻结成基线，别在回到旋转时被抹掉
        self._rebase(context)
        self._move_begin(context)

    def _enter_rotate(self, context):
        # 切旋转之前先把已经拖出来的位移落地、把当前姿态定为基线
        self._rebase(context)
        self._kind = 'ROT'
        self._axis_lock = _default_rotate_axis(self._rig, self._pb)
        self._mv_native_step = False
        bpy.context.view_layer.update()
        self._m_start = self._pb.matrix.copy()
        self._parent_m = _parent_matrix(self._pb)
        self._pivot = _pivot_armature_space(context, self._rig, self._pb)
        self._assign_gesture(context)
        self._sens_base = self._deg_per_px(context)

    def _apply(self):
        if self._mode == 'LOCKED':
            return
        for b in self._targets:
            st = self._states.get(b.name)
            if st is None:
                continue
            m_start, parent_m, pivot = st
            m3 = m_start.to_3x3()
            basis_st = getattr(self, '_states_basis', {}).get(b.name)

            def _to_start():
                _restore_to_start(b, parent_m, m_start, basis_st)

            if self._mode == 'TRACKBALL':
                q = self._quat
                if q.angle < 1e-9:
                    _to_start()
                    continue
                m3_inv = self._rig.matrix_world.inverted().to_3x3()
                axis = (m3_inv @ q.axis).normalized()
                q_arm = Quaternion(axis, q.angle)
            elif self._mode == 'DUAL':
                if axis_locked_eff(b, self._ax_h) and axis_locked_eff(b, self._ax_v):
                    _to_start()
                    continue
                axis_h = (m3 @ Vector(ROT_AXES[self._ax_h][1])).normalized()
                qh = Quaternion(axis_h, math.radians(self._eff_deg(self._ang_h)))
                axis_v = (m3 @ Vector(ROT_AXES[self._ax_v][1])).normalized()
                qv = Quaternion(qh @ axis_v, math.radians(self._eff_deg(self._ang_v)))
                q_arm = qv @ qh
            else:
                if axis_locked_eff(b, self._ax_s):
                    _to_start()
                    continue
                axis_h = (m3 @ Vector(ROT_AXES[self._ax_s][1])).normalized()
                q_arm = Quaternion(axis_h, math.radians(self._eff_deg(self._ang_h)))

            if q_arm.angle < 1e-9:
                _to_start()
                continue
            try:
                # v4.9.94：不再分「带约束 / 不带约束」两条路。带 K 的共轭在
                # K=单位阵时精确退化成原来的反解，一条公式覆盖两种情况，
                # 也就不会出现「预览一种轴、提交一种轴」的分裂。
                if basis_st is None:
                    _apply_pivot_rotation(b, parent_m, m_start, pivot, q_arm)
                else:
                    _apply_visible_rotation(b, parent_m, m_start, basis_st,
                                            pivot, q_arm)
            except Exception:
                _to_start()
        self._remember_pose()

    # ---- 自动关键帧

    def _sync_keys(self, context, force=False):
        """拖动过程中把姿态同步进当前帧的关键帧（自动关键帧开启时）。

        为什么必须同步：动作在当前帧本来就有别的值（上一次确认留下的关键帧），
        而我们拖动时只改 matrix_basis、不写曲线。只要 Blender 重估一次动画
        （切帧、别处插帧、撤销、刷新），曲线上的旧值就会把姿态拽回去 ——
        用户看到的就是「撤销完轻移一下，画面跳回撤销前」。把姿态写进当前帧的
        关键帧之后，重估拿到的就是我们自己的姿态，怎么重估都不会跳。
        v5.2.38：不再限流（原来 40ms 一次），曲线必须逐帧等于姿态，否则
        中间任何一次动画重估都会把姿态拽回去。
        """
        try:
            if not context.scene.tool_settings.use_keyframe_insert_auto:
                return
        except Exception:
            return
        ad = getattr(self._rig, "animation_data", None)
        if ad is None or getattr(ad, "action", None) is None:
            return          # 纯摆姿势（没有动作）时不凭空建 action
        now = time.perf_counter()
        if not force and (now - getattr(self, '_key_t', 0.0)) < KEY_SYNC_SECONDS:
            return
        self._key_t = now
        self._insert_keys(context)

    def _insert_keys(self, context):
        try:
            if not context.scene.tool_settings.use_keyframe_insert_auto:
                return
        except Exception:
            return
        frame = context.scene.frame_current
        for name in [b.name for b in self._targets]:
            pb = self._rig.pose.bones.get(name)
            if pb is None:
                continue
            mode = getattr(pb, "rotation_mode", 'QUATERNION')
            path = ('rotation_quaternion' if mode == 'QUATERNION'
                    else 'rotation_axis_angle' if mode == 'AXIS_ANGLE'
                    else 'rotation_euler')
            try:
                pb.keyframe_insert(path, frame=frame)
                pb.keyframe_insert("location", frame=frame)
                pb.keyframe_insert("scale", frame=frame)
            except Exception:
                pass
        _error_wash()

    def _update_motion_paths(self, context):
        """提交后重算运动路径，对齐原生变换的自动重算（v5.2.37：改为异步排队）。

        为什么必须显式重算（v5.2.30 实测，Blender 5.2.3）：运动路径缓存只在
        原生 transform 结束时失效；插件预览是直写 matrix_basis +
        keyframe_insert，曲线变了缓存不动（update_tag / data.update_tag /
        pb.update_tag 全无效），只有 pose.paths_update() 能刷新，
        且该操作符是骨架级、与选中无关（Recalculate paths for bones that
        already have them），子骨的路径也跟着更新。

        为什么不在确认里同步调（v5.2.37）：全量重算很贵 —— 420 骨 /
        219 根带路径实测 245 ms，直接塞进 _finish 会把 v5.2.36 刚修好的
        「切模式不卡」又打回去。改成 timer 延后 50 ms 执行 + 防抖
        （连续确认只算最后一次）：操作零等待，路径下一帧之后自己跟上，
        和原生变换结束后的重算体验一致。

        只在「骨架确实有运动路径」且「自动关键帧开着」时才排队 ——
        没有动画的纯摆姿、没开运动路径的骨架都是零开销。
        """
        if not pref('mpath_refresh', True):
            return False
        try:
            if not context.scene.tool_settings.use_keyframe_insert_auto:
                return False
        except Exception:
            return False
        rig = self._rig
        if rig is None:
            return False
        try:
            if not any(pb.motion_path for pb in rig.pose.bones):
                return False
        except Exception:
            return False
        return _mpath_queue(rig)

    # ---- 自动判轴

    def _tip_dirs(self, context):
        """三根局部轴各自会让骨骼末端在屏幕上朝哪动（单位 2D 向量，y 向上）。

        做法是真的把 matrix_basis 绕该轴转 ROT_PROBE_DEG 度、把观测点重新投影一次，
        两点之差就是运动方向。比解析求叉乘稳，父级变换和缩放都被算进去了。
        被锁的轴、以及在当前视角下完全看不出变化的轴，返回 None。
        """
        region, rv3d = _view_pair(context)
        if region is None or rv3d is None:
            return [None] * len(ROT_AXES)
        base = self._pb.matrix_basis.copy()
        pts = _probe_offsets(self._pb)
        origins = [_screen_of(region, rv3d,
                              _bone_point_world(self._rig, self._pb, base, o))
                   for o in pts]
        if all(p is None for p in origins):
            return [None] * len(ROT_AXES)
        out = []
        for i in range(len(ROT_AXES)):
            if axis_locked_eff(self._pb, i):
                out.append(None)
                continue
            probe = rotate_basis(base, math.radians(ROT_PROBE_DEG), i)
            acc = Vector((0.0, 0.0))
            for o, p0 in zip(pts, origins):
                if p0 is None:
                    continue
                p1 = _screen_of(region, rv3d,
                                _bone_point_world(self._rig, self._pb, probe, o))
                if p1 is None:
                    continue
                acc += (p1 - p0)
            out.append(acc.normalized() if acc.length > 1e-6 else None)
        return out

    # ---- 单轴（原生 rotate 手感）

    RIM_TILT = 0.5        # |A·n| ≥ 它：圆环基本正对镜头，用轮缘式
    RIM_MIN_PX = 26.0     # 指针离枢轴小于它时方位角没意义，先不动
    # v5.2.39：单个事件允许的最大位移（px）与单轴 RIM 单帧最大方位角增量（°）。
    # 都按「人手一个事件周期内物理上能移动多少」封顶：位移 60~90 px/帧已是
    # 极快拖动，180 留足余量；方位角 25°/帧 相当于 1500°/s，正常拖拽碰不到。
    # 超过就说明事件循环被卡过、这是攒出来的，截断比整段吃掉更接近用户意图。
    DRAG_STEP_MAX = 180.0
    RIM_STEP_MAX_DEG = 25.0
    # 自由旋转 / 双轴 / 投影式单轴：单个事件最多转过多少度。正常拖拽 1~10°，
    # 30° 相当于 1800°/s，人手碰不到；事件被卡过之后攒出来的位移会撞上它。
    STEP_MAX_DEG = 30.0

    def _single_axis_setup(self, context):
        """算一次单轴拖动的方向映射，整段手势都用这一份（v4.9.99）。

        只按「轴在这段视角里长什么样」分两种，都由几何定死，拖动过程中
        **不重算**（重算就是 v4.9.97/98 反向+抖动的根源）：

        · RIM —— 轴基本对着镜头，圆环看起来是个圈。指针绕枢轴转到哪，骨骼
          就转到哪（1:1，等价于抓住轮缘转轮子）。屏幕逆时针为正，再看轴朝
          观察者还是背向：朝观察者为正转。
        · LIN —— 轴躺在屏幕平面里，圆环被压成一条线。方向 u = A×n 投到屏幕
          （n 指向观察者）：A×n 是「离观察者最近那一侧的点、绕 +A 转时的
          运动方向」，所以往 u 拖 = 正转 = 最近的一侧跟手。轴躺着时圆环两侧
          往相反方向动，可见运动本来二义；取最近的一侧 = 视线里最先生变化
          的那一片 —— 往上拖就是往上抬，不会再「鼠标往上、角色往下弯」。
        """
        self._s_mode = None
        self._s_sign = 1.0
        self._s_u = None
        self._s_pv = None
        self._s_phi = None
        self._rim_acc = 0.0
        if self._mode != 'SINGLE' or self._ax_s is None or self._pb is None:
            return
        region, rv3d = _view_pair(context)
        if region is None or rv3d is None:
            return
        try:
            A = self._rig.matrix_world.to_3x3() @ (
                self._m_start.to_3x3() @ Vector(ROT_AXES[self._ax_s][1]))
            A.normalize()
            n = -(rv3d.view_rotation @ Vector((0.0, 0.0, -1.0))).normalized()
        except Exception:
            return
        s = A.dot(n)
        try:
            pv = _screen_of(region, rv3d, self._rig.matrix_world @ self._pivot)
        except Exception:
            pv = None
        if pv is not None and abs(s) >= self.RIM_TILT:
            self._s_mode = 'RIM'
            self._s_sign = 1.0 if s > 0.0 else -1.0
            # _screen_of 与 event.mouse_region_* 同一个系（region，y 向上）
            self._s_pv = Vector((pv.x, pv.y))
            return
        right, up = _view_axes(rv3d)
        w = A.cross(n)
        u = Vector((w.dot(right), w.dot(up)))
        if u.length < 1e-6:
            d = self._dir_win.get(self._ax_s)       # 极端退化：用进场缓存
            u = Vector((d.x, -d.y)) if d is not None else Vector((0.0, 1.0))
        self._s_mode = 'LIN'
        self._s_u = u.normalized()

    def _single_axis_step(self, context, event, dx, dy, factor):
        """单轴拖动：按 _single_axis_setup 定好的映射累计角度（v4.9.99）。

        RIM 用指针绕枢轴的方位角增量（可无限圈、不漂）；LIN 把手位移投到固定
        方向 u 上按 °/px 累加。吸附（Ctrl）、微调（Shift）照旧。
        """
        if self._s_mode is None:
            self._single_axis_setup(context)
        sens = self._sens_base * factor
        # 模态传进来的 dy 是「向下为正」（dy = -(mouse_y 增量)，mouse_y 向上），
        # 而屏幕投影 / _s_u / 方位角全是「向上为正」—— 先统一翻成 dyu。
        # v4.9.97/98 就是漏了这一步（还叠了个 y 向下的 u），整条 Y 通道反着走。
        dyu = -dy
        if self._s_mode == 'RIM' and self._s_pv is not None:
            mx = float(getattr(event, 'mouse_region_x', 0.0))
            my = float(getattr(event, 'mouse_region_y', 0.0))
            v = Vector((mx, my)) - self._s_pv
            if v.length < self.RIM_MIN_PX:
                # 指针就压在枢轴上（骨头里）：方位角全是噪声，先不转，
                # 等手把它拖出死区再接管（_s_phi 置空 → 重新起算）
                self._s_phi = None
                return
            phi = math.degrees(math.atan2(v.y, v.x))
            if self._s_phi is None:
                # 第一步：上一帧指针位置用这一帧的位移反推，不丢这一下
                prev = v - Vector((dx, dyu))
                self._rim_acc = self._ang_h * self._s_sign
                self._s_phi = (phi if prev.length < 1e-6 else
                               math.degrees(math.atan2(prev.y, prev.x)))
            d = phi - self._s_phi
            d = (d + 180.0) % 360.0 - 180.0        # 归到 (-180, 180]，可绕多圈
            self._s_phi = phi
            # v5.2.39：单帧方位角突变限幅。事件循环被卡过时指针会「瞬移」，
            # 方位角随之猛跳 —— 限幅之后姿态最多前进 25°，不再整段飞出去。
            cap_rim = getattr(self, 'RIM_STEP_MAX_DEG', 25.0)
            if d > cap_rim:
                d = cap_rim
            elif d < -cap_rim:
                d = -cap_rim
            self._rim_acc += d * factor
            self._ang_h = self._rim_acc * self._s_sign
            return
        u = self._s_u
        if u is None:
            return
        # u 是屏幕上「y 向上」的 2D 方向
        add = (dx * u.x + dyu * u.y) * sens
        # v5.2.39：单帧转角限幅，同上
        cap = getattr(self, 'STEP_MAX_DEG', 30.0)
        if add > cap:
            add = cap
        elif add < -cap:
            add = -cap
        self._ang_h += add

    def _confine_cursor(self, context, event):
        """把指针夹在控制器周围的活动半径内 —— 自由 / 双轴 / 投影式单轴都走这里。

        只夹位置、不改旋转量：指针沿半径被拉回圆周，同时把 _ref_x/_ref_y 对齐到
        新位置，所以下一帧的 dx/dy 仍然是手的真实位移，旋转继续累计，但指针始终
        留在控制器附近（确认后能马上抓到 gizmo）。
        """
        zone = int(pref('ring_zone', 110) or 0)
        if zone <= 0:
            return False
        region, rv3d = _view_pair(context)
        if region is None or rv3d is None or self._rig is None:
            return False
        # 移动时枢轴要跟着骨骼走（骨骼本身在动），旋转时沿用进场时的枢轴
        if self._kind == 'MOVE' and self._pb is not None:
            try:
                centre = self._rig.matrix_world @ self._pb.matrix.translation
            except Exception:
                centre = self._rig.matrix_world @ self._pivot
        else:
            centre = self._rig.matrix_world @ self._pivot
        try:
            pv = _screen_of(region, rv3d, centre)
        except Exception:
            pv = None
        if pv is None:
            return False
        m = Vector((float(getattr(event, 'mouse_region_x', 0.0)),
                    float(getattr(event, 'mouse_region_y', 0.0))))
        v = m - pv
        if v.length <= float(zone):
            self._zone_hit = False
            return False
        self._zone_hit = True
        self._ring_warp(context, pv + v.normalized() * float(zone))
        return True

    def _ring_warp(self, context, target):
        """把指针拉回圆周上（target 是 region 坐标）。

        只改半径、不改方位角，所以 angle 不受影响，角度完全连续；
        也不设 _skip_move —— 那不是"瞬移"，是沿着半径方向夹住，不需要丢参考角。
        """
        win = getattr(context, "window", None)
        region = getattr(context, "region", None)
        if win is None or region is None:
            return
        wx = int(region.x + target.x)
        wy = int(region.y + target.y)
        try:
            win.cursor_warp(wx, wy)
        except Exception:
            return
        self._ref_x, self._ref_y = wx, wy
        self._rx = wx - region.x
        self._ry = wy - region.y

    # ---- trackball

    def _trackball_step(self, context, dx, dy, factor=1.0):
        """鼠标相对起点的总位移 → 一个绝对旋转（不再逐帧累乘）。

        逐帧累乘时，绕不同屏幕轴的小旋转不满足交换律，来回画圈会越走越偏；
        改成从起点累计 dx/dy 重新算一次，闭路径会精确回到原位。

        v4.9.30 起角度走原生曲线 2·atan(位移/球半径)，不再线性：
        线性时 0.2°/px 要拖半个屏幕才转 90°，原生 gizmo 拖 100px 就过 90°。
        曲线保留"起手跟手、越远越收"的原生手感，倍率只改球半径。
        """
        region, rv3d = _view_pair(context)
        if region is None or rv3d is None:
            return
        base_x, base_y = self._track_dx, self._track_dy
        self._track_dx += dx * factor
        self._track_dy += dy * factor
        right, up = _view_axes(rv3d)

        def _q(sx, sy):
            return (Quaternion(up, math.radians(self._native_curve_deg(sx)))
                    @ Quaternion(right, math.radians(self._native_curve_deg(sy)))
                    @ self._quat0)

        q_new = _q(self._track_dx, self._track_dy)
        # v5.2.39：单帧转角限幅。事件循环被卡过（路径重算、撤销、大重估）时，
        # 恢复后的第一个事件带的是攒下的位移；而这条角度曲线在原点附近最陡
        # （1 px ≈ 1°），180 px 就能给出 120° —— 像素上限在这里挡不住。
        # 按「这一段最多转多少度」二分回缩累计位移：正常拖动 1~10°/帧，碰不到。
        if self._quat is not None:
            step = math.degrees((self._quat.inverted() @ q_new).angle)
            cap_deg = getattr(self, 'STEP_MAX_DEG', 30.0)
            if step > cap_deg:
                lo, hi = 0.0, 1.0
                for _ in range(9):
                    mid = (lo + hi) * 0.5
                    qm = _q(base_x + (self._track_dx - base_x) * mid,
                            base_y + (self._track_dy - base_y) * mid)
                    if math.degrees((self._quat.inverted() @ qm).angle) > cap_deg:
                        hi = mid
                    else:
                        lo = mid
                self._track_dx = base_x + (self._track_dx - base_x) * lo
                self._track_dy = base_y + (self._track_dy - base_y) * lo
                q_new = _q(self._track_dx, self._track_dy)
        # 转完视角时会把当时的旋转冻结进 _quat0，并把 _track_dx/dy 归零。
        # 直接在 _quat0 之上叠这一段的增量 —— 否则累计位移会被当前视角的
        # right/up 重新解释一遍，姿态瞬间跳到完全不同的方向（剧烈扭动）。
        self._quat = q_new

    # ---- 灵敏度 / 指针回绕

    def _screen_len(self, context):
        """骨骼在屏幕上的投影长度（像素），用来归一化灵敏度。

        v5.2.25：直接投骨骼可见的头 / 尾两个端点。原来投的是"头 + 沿局部 Y
        一个骨长"的推演点，一旦父级链换算有偏差（见 _bone_point_world），
        长度就会随根骨的旋转变来变去 —— 同样手拖 10px，转角能差 30%，
        表现就是"根环转过之后手感突然变了 / 第一下跳"。头尾投影没有这个
        推演环节，长度恒等于肉眼看到的骨长。
        """
        region, rv3d = _view_pair(context)
        if region is None or rv3d is None:
            return ROT_REF_PX
        pb = self._pb
        mw = self._rig.matrix_world
        p0 = _screen_of(region, rv3d, mw @ pb.head)
        p1 = _screen_of(region, rv3d, mw @ pb.tail)
        if p0 is None or p1 is None:
            return ROT_REF_PX
        return max((p1 - p0).length, ROT_MIN_PX)

    def _deg_per_px(self, context):
        """双轴 / 单轴用的线性灵敏度（°/px），量级锚在原生 gizmo 上。

        长度自适应只留很软的一条：半屏骨骼 ≈ 0.85°/px，手指 ≈ 1.5°/px。
        老版是 (REF/len) 的一次方、基准还只有 0.5，半屏骨骼被压到 0.15°/px，
        转 90° 要拖 600px —— 那是「拖很远」的根源，不是倍率能救的。
        """
        soft = (ROT_REF_PX / self._screen_len(context)) ** ROT_SOFT_POW
        v = NATIVE_DEG_PER_PX * soft * pref('rot_sensitivity', 1.0)
        return max(ROT_MIN_DEG_PER_PX, min(ROT_MAX_DEG_PER_PX, v))

    def _native_curve_deg(self, px):
        """原生 gizmo 的角度曲线：2·atan(位移 / 球半径)，带符号。"""
        gain = float(pref('rot_sensitivity', 1.0) or 1.0)
        r = TRACKBALL_R_PX / max(gain, 1e-6)      # 倍率越大球越小 = 越跟手
        d = abs(px) / max(r, 1e-6)
        deg = math.degrees(2.0 * math.atan(d))
        return deg if px >= 0 else -deg

    # ---- 模态内转视角 / 收尾

    def _cancel_all(self, context):
        """回到进入模态时的姿态并退出。"""
        restore = getattr(self, '_session0', None) or getattr(self, '_steps0', None) or {}
        for name, mat in restore.items():
            pb = self._rig.pose.bones.get(name)
            if pb is not None:
                pb.matrix_basis = mat
        self._clear(context)
        return {'CANCELLED'}

    def _finish(self, context):
        """确认：移动先把累积位移落地（走原生 G），再插关键帧并结束。"""
        self._solo_restore()        # 原生 G 的撤销步骤不能记录 solo 隐藏
        if self._kind == 'MOVE':
            self._move_flush(context)
        elif self._kind == 'ROT':
            self._flush_rotation(context)   # 单轴最后一段也走原生 rotate
        self._insert_keys(context)
        self._update_motion_paths(context)   # v5.2.30：和原生一样重算路径
        self._clear(context)
        return {'FINISHED'}

    def _orbit_view(self, context, event=None):
        """中键 = 原生导航，嵌套在当前模态里跑。

        和 Blender 原生一致：中键旋转、Shift+中键平移、Ctrl+中键缩放。
        返回 True 表示事件已经交给原生导航操作符。姿态不做任何改动：
        移动的预览基准、旋转的基准矩阵都留着，视图操作完接着拖是连续的。
        """
        win = getattr(context, "window", None)
        area = getattr(context, "area", None)
        region = getattr(context, "region", None)
        space = getattr(context, "space_data", None)
        if win is None or area is None:
            return False
        if space is None or getattr(space, "type", None) != 'VIEW_3D':
            space = area.spaces.active
        if space is None or getattr(space, "type", None) != 'VIEW_3D':
            return False
        # view3d.rotate / move / zoom 只认视口的 WINDOW 区域，别的区域先换过来
        if region is None or getattr(region, "type", None) != 'WINDOW':
            region = next((r for r in area.regions if r.type == 'WINDOW'), None)
        if region is None:
            return False

        shift = bool(getattr(event, "shift", False)) if event is not None else False
        ctrl = bool(getattr(event, "ctrl", False)) if event is not None else False
        if ctrl:
            op_name = 'zoom'        # Ctrl+中键：原生缩放
        elif shift:
            op_name = 'move'        # Shift+中键：原生平移
        else:
            op_name = 'rotate'      # 中键：原生旋转

        op = getattr(bpy.ops.view3d, op_name, None)
        if op is None:
            return False
        try:
            with context.temp_override(window=win, area=area,
                                       region=region, space_data=space):
                res = op('INVOKE_DEFAULT')
        except Exception:
            return False
        if not res or not ({'RUNNING_MODAL', 'FINISHED'} & set(res)):
            return False
        # 视角变了：下一帧 MOUSEMOVE 只取参考点，
        # 否则「操作视角时那段位移」会被当成一次拖动，姿态会突然跳一下。
        self._primed = False
        if 'FINISHED' in set(res):
            self._warp_to_bone(context)     # 当场跑完的少数情况，直接归位
        else:
            # 原生导航是模态操作符：INVOKE 立刻返回，它继续吃鼠标。此刻不能动指针，
            # 否则等于在人家转到一半时插一脚。标记一下，等本模态重新收到事件
            # （说明那层已经退掉）再把指针送回控制器。
            self._pending_warp = True
            self._orbit_t = time.perf_counter()
        if op_name != 'rotate':
            # 平移/缩放不改朝向，但光标位置变了，取完参考点就能继续
            self._header(context)
            return True
        self._rebase_rotation()
        self._refresh_projection(context)
        self._header(context)
        return True

    def _after_view_change(self, context):
        """视角被动过（Alt+中键的原生 view_axis / view_center_pick）之后的善后。

        和中键导航同一套：下一帧 MOUSEMOVE 只取参考点（_primed=False），
        指针归位走 _pending_warp，旋转基线冻结、拖动方向重算。
        """
        self._primed = False
        self._pending_warp = True
        self._orbit_t = time.perf_counter()
        self._rebase_rotation()
        self._refresh_projection(context)
        self._header(context)

    def _rebase_rotation(self):
        """把当前旋转冻结成新基准，后续拖动从它接着转。

        TRACKBALL 的 _quat 是用「累计位移 × 当前视角的屏幕上/右方向」算出来的，
        视角一转，同一份累计位移会被新坐标重新解释，姿态会瞬间甩到别处。
        所以转完视角必须把已完成的旋转存进 _quat0、累计位移归零。
        """
        if self._kind != 'ROT' or self._mode != 'TRACKBALL':
            return
        self._quat0 = self._quat.copy()
        self._track_dx = 0.0
        self._track_dy = 0.0

    def _refresh_projection(self, context):
        """视角变化后重算拖动方向；角度跟着轴走，灵敏度保持不变。"""
        if self._kind != 'ROT':
            return              # 移动每帧都按当前视角现算，不用缓存
        old = {}
        if self._mode == 'DUAL':
            old[self._ax_h] = self._ang_h
            old[self._ax_v] = self._ang_v
        elif self._mode == 'SINGLE':
            old[self._ax_s] = self._ang_h
        prev_mode = self._mode
        try:
            # 只重算方向映射（_dir_win）。灵敏度 _sens_base 故意不动：
            # 它按骨骼屏幕投影长度归一，视角一变投影就变，重算会让手感突然变飘。
            self._assign_gesture(context)
        except Exception:
            return
        if self._mode != prev_mode:
            return              # 手势级别都变了，保留原角度，别把姿态拉回去
        if self._mode == 'DUAL':
            # 水平/垂直通道可能因为新视角换了轴，把角度还给原来那根轴，避免跳变
            self._ang_h = old.get(self._ax_h, 0.0)
            self._ang_v = old.get(self._ax_v, 0.0)
        elif self._mode == 'SINGLE':
            self._ang_h = old.get(self._ax_s, 0.0)
            self._ang_v = 0.0

    def _warp_to_bone(self, context):
        """把指针送回当前控制器（骨骼头部）的屏幕位置。

        中键转完视角，指针常常被甩到别处，要继续调就得先把鼠标找回来。
        这里直接把指针落在控制器上，落点偏移沿用偏好里的 warp_dx/dy，
        和导航切骨骼时的鼠标跟随是同一套坐标换算。
        """
        if not pref('warp_after_view', True):
            return False
        win = getattr(context, "window", None)
        region = getattr(context, "region", None)
        if win is None or region is None or self._rig is None or self._pb is None:
            return False
        pt = bone_window_point(context, self._rig, self._pb.name)
        if pt is None:
            return False
        try:
            win.cursor_warp(pt[0], pt[1])
        except Exception:
            return False
        # 指针跳了：把它当成新的参考点，这次跳变不能算成一次拖动
        self._ref_x, self._ref_y = pt
        self._rx = pt[0] - region.x
        self._ry = pt[1] - region.y
        self._primed = True
        self._skip_move = True
        try:
            context.area.tag_redraw()
        except Exception:
            pass
        return True

    def _wrap_cursor(self, context, event):
        """指针快撞到视口边缘时把指针拉回中心，避免「转不动了」。

        角度只由相对位移累加，所以指针瞬移不会让角度跳变。
        """
        if not pref('wrap_cursor', True):
            return
        region = getattr(context, "region", None)
        win = getattr(context, "window", None)
        if region is None or win is None or region.type != 'WINDOW':
            return
        m = ROT_WRAP_MARGIN
        rw, rh = region.width, region.height
        if m >= rw // 2 or m >= rh // 2:
            return
        rx, ry = event.mouse_region_x, event.mouse_region_y
        if m <= rx <= rw - m and m <= ry <= rh - m:
            return
        cx = int(region.x + rw // 2)
        cy = int(region.y + rh // 2)
        try:
            win.cursor_warp(cx, cy)
            self._ref_x, self._ref_y = cx, cy
            self._skip_move = True
        except Exception:
            pass

    # ---- HUD

    def _draw_hud(self, context):
        """在指针旁边画一行状态（轴 / 角度 / 吸附档），视线不用抬到视口顶部。"""
        if not _hud_available() or not pref('show_hud', True):
            return
        region = getattr(context, "region", None)
        if region is None:
            return
        try:
            import blf
            fid = 0
            try:
                blf.size(fid, 13.0)
            except TypeError:
                blf.size(fid, 13.0, 72)
            blf.enable(fid, blf.SHADOW)
            try:
                blf.shadow(fid, 3, 0.0, 0.0, 0.0, 0.9)
                blf.shadow_offset(fid, 1, -1)
            except Exception:
                pass

            x = self._rx + 16
            y = self._ry + 14
            if x > region.width - 210:
                x = max(4, self._rx - 210)
            if y > region.height - 34:
                y = max(4, self._ry - 34)

            main = "%s   %s%s" % (self._mode_line(), self._angles_line(),
                                  self._total_line())
            bits = []
            if self._snap_step:
                bits.append("吸附 %g°" % self._snap_step)
            if getattr(self, '_zone_hit', False):
                bits.append("指针已贴活动半径")
            _xyz = "X/Y/Z 锁轴 · " if pref('use_xyz_keys', True) else ""
            _mk = move_key_label() if move_key_ok() else "移动键"
            bits.append(("%s 切移动 · 滚轮↓ 切 X→Y→Z · 中键转视角 / ⇧平移 · 左键确认 · 右键退一步"
                         % _mk)
                        if self._kind == 'ROT' else
                        _xyz + "滚轮↓ 切 自由→X→Y→Z · 中键转视角 / ⇧平移 · 左键确认 · 右键退一步")
            sub = "   ".join(bits)

            blf.position(fid, x, y, 0)
            blf.color(fid, 1.0, 1.0, 1.0, 0.96)
            blf.draw(fid, main)

            try:
                blf.size(fid, 11.0)
            except TypeError:
                blf.size(fid, 11.0, 72)
            blf.position(fid, x, y - 15, 0)
            blf.color(fid, 1.0, 1.0, 1.0, 0.68)
            blf.draw(fid, sub)
        except Exception:
            _HUD_OK[0] = False
            return

    # ---- 收尾

    # ---- Solo 显示

    def _solo_apply(self, context):
        """只显示当前控制器（多选时留所有选中的），其余骨骼临时隐藏。

        快照只在第一次进模态时取一次（原始 hide 状态），之后换骨骼只改可见集；
        结束时 _solo_restore 按快照原样恢复。只写显示开关，不碰姿态数据。

        必须用 PoseBone.hide：Bone.hide 的语义是「编辑模式下不可见」，
        在姿势模式里设它完全没效果（v4.9.19 就是踩了这个坑）。
        """
        if not pref('solo_display', True) or bpy.app.background:
            return
        rig = self._rig
        if rig is None or rig.type != 'ARMATURE':
            return
        if self._solo_saved is None:
            self._solo_saved = {pb.name: bool(pb.hide) for pb in rig.pose.bones}
        keep = set()
        if self._pb is not None:
            keep.add(self._pb.name)
        try:
            for b in (context.selected_pose_bones or []):
                if b is not None:
                    keep.add(b.name)
        except Exception:
            pass
        for pb in rig.pose.bones:
            try:
                pb.hide = pb.name not in keep
            except Exception:
                pass
        self._solo_redraw(context)

    def _solo_redraw(self, context):
        try:
            area = getattr(context, "area", None)
            if area is not None:
                area.tag_redraw()
        except Exception:
            pass

    def _solo_restore(self, forget=True):
        """把骨骼显隐恢复成进模态前的样子。

        forget=False 时保留快照（用于调原生 ops 前临时恢复，事后再 _solo_apply
        重新隐藏——避免隐藏状态被 ops 的撤销步骤记录进去）。重复调用安全。
        """
        saved = self._solo_saved
        if not saved:
            return
        rig = self._rig
        if rig is not None and rig.type == 'ARMATURE':
            for name, hidden in saved.items():
                pb = _pb_get(rig, name)
                if pb is not None:
                    try:
                        pb.hide = hidden
                    except Exception:
                        pass
        _error_wash()
        if forget:
            self._solo_saved = None

    def _clear(self, context):
        """移除 HUD、恢复光标、清 header、恢复 solo 显隐。FINISHED / CANCELLED 都走这里。"""
        global _NAV_VIS, _NAV_VIS_RIG
        _NAV_VIS = None
        _NAV_VIS_RIG = None
        _MODAL_ONCE[0] = False      # 先放闸，后面任何一步抛异常也不会卡死再次进入
        try:
            self._solo_restore()
        except Exception:
            pass
        self._solo_redraw(context)
        if self._handle is not None:
            try:
                bpy.types.SpaceView3D.draw_handler_remove(self._handle, 'WINDOW')
            except Exception:
                pass
            self._handle = None
        if self._cursor_hidden:
            try:
                context.window.cursor_set('DEFAULT')
            except Exception:
                pass
            self._cursor_hidden = False
        area = getattr(context, "area", None)
        if area is not None:
            area.header_text_set(None)

    def _switch_bone(self, context, step, extend=False):
        """滚轮切到导航列表里的上一根/下一根骨骼，保持旋转模式。

        extend=True（Shift+滚轮）时把目标追加到当前选择，和导航的 Shift 加选一致。
        """
        try:
            self._solo_restore(forget=False)   # 原生 G 的撤销步骤不能记录 solo 隐藏
            if self._kind == 'MOVE':
                self._move_flush(context)
            self._insert_keys(context)
        except Exception:
            pass
        # v5.2.16：上面的 RNA 写入（pb.hide / keyframe_insert）在 Blender 5.x
        # 下可能静默挂异常不抛 —— 不冲刷的话下一个 .get 就报 SystemError
        # "returned a result with an exception set"（用户截图里的刷屏就是这个）。
        _error_wash()
        target = nav_neighbor_name(self._rig, self._pb.name, step)
        if not target or target == self._pb.name:
            return False

        # 沿这个方向找：旋转模式优先可旋转骨骼；只有移动能力的骨骼自动切移动；
        # 两个都不行的（全锁）跳过，避免又卡到不能操作的骨骼上。
        start = self._pb.name
        pb = None
        auto_move = False
        auto_rotate = False
        for _ in range(256):
            cand = _pb_get(self._rig, target)
            if cand is None:
                return False
            if self._kind == 'ROT':
                if _bone_rotatable(cand):
                    pb = cand
                    break
                if _bone_movable(cand):
                    pb = cand
                    auto_move = True
                    break
            else:
                if _bone_movable(cand):
                    pb = cand
                    break
                if _bone_rotatable(cand):
                    pb = cand
                    auto_rotate = True
                    break
            target = nav_neighbor_name(self._rig, target, step)
            if not target or target == start:
                self.report({'WARNING'}, "这个方向上没有可用的控制器")
                return False
        if pb is None:
            return False
        target = pb.name

        # 先把当前骨骼的旋转定下来（不丢），再换目标
        self._start = self._pb.matrix_basis.copy()
        # 旧骨骼已经插过关键帧、姿态定稿：把它在 _orig 里的快照更新成
        # 「切换这一刻」的状态。否则右键撤销会按模态开始时的快照把上一根
        # 骨骼也一并回滚 —— 滚到下一个控制器转完，一撤连上一个的旋转都没了。
        # 注意必须用切换前的 self._pb，且放在确认能切换之后：
        # 滚到头切不动时，右键仍可撤销当前这次操作。
        self._orig[self._pb.name] = self._pb.matrix_basis.copy()
        select_bone(self._rig, target, extend=bool(extend))

        self._pb = pb
        self._axis_lock = _default_rotate_axis(self._rig, pb)
        if extend:
            sel = [b for b in (context.selected_pose_bones or []) if b is not None]
            if pb not in sel:
                sel.append(pb)
            roots = [b for b in sel if b.parent not in sel] or [pb]
            self._orig.update({b.name: b.matrix_basis.copy() for b in sel})
            self._session.update(b.name for b in sel)   # 快照覆盖整段会话
            for b in sel:
                self._session0.setdefault(b.name, b.matrix_basis.copy())
            self._targets = roots
            # 提交点：回滚名单换成这一段加选进来的骨骼
            self._touched = {b.name for b in sel}
            self._states = {}
            for b in roots:
                self._capture_state(b)
        else:
            self._orig.setdefault(pb.name, pb.matrix_basis.copy())
            self._session.add(pb.name)      # 快照覆盖整段会话
            self._session0.setdefault(pb.name, pb.matrix_basis.copy())
            self._targets = [pb]
            # 提交点：回滚名单只留新骨骼，旧骨骼已提交、绝不回滚
            self._touched = {pb.name}
            self._capture_state(pb)
        self._remember_pose()
        self._push_undo_step()          # 切骨骼 = 一步结束，右键可退回到这里
        self._start = pb.matrix_basis.copy()
        bpy.context.view_layer.update()
        self._m_start = pb.matrix.copy()
        self._parent_m = _parent_matrix(pb)
        self._pivot = _pivot_armature_space(context, self._rig, pb)
        self._quat = Quaternion()
        self._quat0 = Quaternion()      # 视图旋转后累积到这里，新拖动从它接着转
        self._track_dx = 0.0
        self._track_dy = 0.0
        self._ang_h = 0.0
        self._ang_v = 0.0
        self._mv_pending = Vector((0.0, 0.0, 0.0))
        self._mv_bases = {}
        self._move_axis = None
        self._move_amount = 0.0
        self._snap_step = 0.0
        self._assign_gesture(context)   # 新骨骼轴锁可能不同，重新分级
        self._sens_base = self._deg_per_px(context)
        self._primed = False       # 换人后重新取参考点，避免角度跳变
        self._tot_rot = 0.0
        self._tot_move = 0.0
        if auto_rotate:
            self._kind = 'ROT'
            self._axis_lock = _default_rotate_axis(self._rig, pb)
            self._assign_gesture(context)
            self._sens_base = self._deg_per_px(context)
        elif auto_move:
            self._kind = 'MOVE'
            self._move_begin(context)
        elif self._kind == 'MOVE':
            self._move_begin(context)
        self._solo_apply(context)   # 可见集换成新骨骼（多选时保留全部选中的）
        self._header(context)
        return True

    def _switch_collection(self, context, step):
        try:
            self._solo_restore(forget=False)   # 原生 G 的撤销步骤不能记录 solo 隐藏
            if self._kind == 'MOVE':
                self._move_flush(context)
            self._insert_keys(context)
        except Exception:
            pass
        ok, msg = step_collection(context, self._rig, step)
        if not ok:
            self.report({'WARNING'}, msg)
            return False
        pb = _pb_get(self._rig, self._rig.data.bones.active.name) \
            if self._rig.data.bones.active else None
        if pb is None:
            return False
        # 同 _switch_bone：切走前把旧骨骼的快照更新成切换这一刻的姿态，
        # 右键撤销只回滚新骨骼，不连上一个控制器一起撤。
        self._orig[self._pb.name] = self._pb.matrix_basis.copy()
        self._pb = pb
        self._axis_lock = _default_rotate_axis(self._rig, pb)
        self._orig.setdefault(pb.name, pb.matrix_basis.copy())
        self._targets = [pb]
        self._session.add(pb.name)     # 快照覆盖整段会话
        self._session0.setdefault(pb.name, pb.matrix_basis.copy())
        self._touched = {pb.name}      # 提交点：回滚名单只留新控制器
        self._capture_state(pb)
        self._remember_pose()
        self._push_undo_step()          # 切集合 = 一步结束，右键可退回到这里
        bpy.context.view_layer.update()
        self._m_start = pb.matrix.copy()
        self._parent_m = _parent_matrix(pb)
        self._pivot = _pivot_armature_space(context, self._rig, pb)
        self._start = pb.matrix_basis.copy()
        self._quat = Quaternion()
        self._quat0 = Quaternion()      # 视图旋转后累积到这里，新拖动从它接着转
        self._track_dx = 0.0
        self._track_dy = 0.0
        self._ang_h = 0.0
        self._ang_v = 0.0
        self._mv_pending = Vector((0.0, 0.0, 0.0))
        self._mv_bases = {}
        self._move_axis = None
        self._move_amount = 0.0
        self._snap_step = 0.0
        self._assign_gesture(context)
        self._sens_base = self._deg_per_px(context)
        self._primed = False
        self._tot_rot = 0.0
        self._tot_move = 0.0
        if self._kind == 'MOVE':
            self._move_begin(context)
        self._solo_apply(context)   # 可见集换成新集合的活动骨骼
        self._header(context)
        return True

    def modal(self, context, event):
        try:
            return self._modal(context, event)
        except Exception as exc:
            try:
                self._clear(context)
            except Exception:
                pass
            self.report({'ERROR'}, "旋转/移动已中断：%s" % exc)
            return {'CANCELLED'}

    def _modal(self, context, event):
        # v5.2.39：记下「手还在动」。异步路径刷新靠这个时间戳判断能不能开跑，
        # 详见 _GESTURE_T 处的注释。
        global _GESTURE_T
        _GESTURE_T = time.perf_counter()
        et = event.type
        # 中键单击 = 清旋转；移动超过 4px 才转交给转视角
        if (self._mmb_click is not None
                and et in {'MOUSEMOVE', 'INBETWEEN_MOUSEMOVE'}):
            mx, my = self._mmb_click[0], self._mmb_click[1]
            if math.hypot(event.mouse_x - mx, event.mouse_y - my) > 4.0:
                self._mmb_click = None
                self._orbit_view(context, event)
            return {'RUNNING_MODAL'}

        # 中键导航结束后把指针送回控制器：原生导航操作符是模态的、会一直吃到松手，
        # 本模态只要重新收到事件，就说明那层已经退掉了。这一帧只归位，不吃位移。
        if getattr(self, "_pending_warp", False) and et != 'TIMER':
            # 时间下限是兜底：万一原生导航还在跑就把事件漏给了我们，
            # 也不能在人家转到一半时把指针搬走。
            if (time.perf_counter() - getattr(self, "_orbit_t", 0.0)) >= 0.08:
                self._pending_warp = False
                self._warp_to_bone(context)
                # 导航彻底结束，视角已定：重算拖动方向。中键旋转后不重算的话，
                # _dir_win 还是转视角之前的投影，接着拖会不跟手。
                self._refresh_projection(context)
            if et in {'MOUSEMOVE', 'INBETWEEN_MOUSEMOVE'}:
                return {'RUNNING_MODAL'}

        if et in {'MOUSEMOVE', 'INBETWEEN_MOUSEMOVE'} and getattr(self, "_alt_nav", False):
            # Alt 还按着：这是原生 view_axis 的拖动，必须继续放行，
            # 不能在这里 RUNNING_MODAL 把它吃掉（否则 CLICK_DRAG 永远形不成）。
            if getattr(event, 'alt', False):
                return {'PASS_THROUGH'}
            # Alt 已经松开：原生那层结束，做善后；这一帧只取参考点。
            self._alt_nav = False
            self._after_view_change(context)
            return {'RUNNING_MODAL'}

        if et == 'MOUSEMOVE':
            if self._skip_move:
                self._skip_move = False
                self._ref_x = event.mouse_x
                self._ref_y = event.mouse_y
                self._rx = event.mouse_region_x
                self._ry = event.mouse_region_y
                # 指针被瞬移过（视口边缘回绕等）：RIM 的方位角基准作废，重起算
                self._s_phi = None
                return {'RUNNING_MODAL'}
            # 进入后的第一帧只取当前指针位置当基准，避免按下 X 那段位移被算成输入
            if not self._primed:
                self._primed = True
                self._ref_x = event.mouse_x
                self._ref_y = event.mouse_y
                self._rx = event.mouse_region_x
                self._ry = event.mouse_region_y
                return {'RUNNING_MODAL'}

            # 先把「姿态被外部改过、基线没跟上」的情形纠掉，再算这一帧的位移。
            # 必须放在累计量更新之前：_single_axis_step / _quat 一旦累加，
            # 自检就分不清"基线过期"和"用户正在拖"了。
            self._ensure_baseline(context)

            dx = event.mouse_x - self._ref_x
            dy = -(event.mouse_y - self._ref_y)
            self._ref_x = event.mouse_x
            self._ref_y = event.mouse_y
            self._rx = event.mouse_region_x
            self._ry = event.mouse_region_y

            # v5.2.25：参考点错位守卫。指针被外部挪过（warp、边缘回绕、
            # 窗口焦点切换、滚轮切骨骼后指针落点）时，第一帧的 dx/dy 会是
            # 整块屏幕的距离；原样吃下去就是「拖第一下骨骼跳到别处」。
            # 本段第一次有效输入若超过视口三分之一，丢掉这一帧当参考点重取
            # —— 正常拖动第一帧只有几像素，不会误伤。
            if not getattr(self, '_moved_any', False):
                span = math.hypot(dx, dy)
                limit = 120.0
                try:
                    if context.region is not None:
                        limit = max(120.0, 0.35 * context.region.height)
                except Exception:
                    pass
                if span > limit:
                    self._header(context)
                    return {'RUNNING_MODAL'}
            self._moved_any = True

            # v5.2.39：拖拽中途的「补一刀」守卫。事件循环被卡过（路径全量重算、
            # 撤销、大 depsgraph 重估、定时器）时，恢复后的**第一个** MOUSEMOVE
            # 带的是这段时间攒下的总位移 —— 一次吃进去就是「卡一下、姿态猛跳
            # 一段」。人手一帧最多也就 60~90 px（2000 px/s @ 30fps），超过
            # DRAG_STEP_MAX 的部分一律按上限截断，剩下的丢弃（参考点已经对齐
            # 到当前指针，下一帧正常跟手）。单轴 RIM 的方向在 step 里单独限幅。
            span = math.hypot(dx, dy)
            cap_px = getattr(self, 'DRAG_STEP_MAX', 180.0)
            if span > cap_px:
                k = cap_px / span
                dx *= k
                dy *= k

            # 吸附档：Ctrl = 偏好里的角度（trackball 不吸附，没有单一角度可吸）
            if event.ctrl and self._kind == 'ROT' and self._mode != 'TRACKBALL':
                self._snap_step = 15.0 if event.shift else float(pref('snap_deg', 5) or 5)
                factor = 1.0
            elif event.shift:
                self._snap_step = 0.0
                factor = FINE_FACTOR
            else:
                self._snap_step = 0.0
                factor = 1.0

            sens = self._sens_base * factor
            if getattr(self, "_scaling", False):
                self._scale_apply(event)
                self._sync_keys(context)
                self._header(context)
                try:
                    context.area.tag_redraw()
                except Exception:
                    pass
                return {'RUNNING_MODAL'}
            if self._kind == 'MOVE':
                # Shift 微调和旋转共用同一个 factor
                self._move_step(context, dx * factor, dy * factor)
                self._sync_keys(context)
                self._header(context)
                # v4.9.95：只在指针快撞到视口边缘时把它拉回中心（和旋转同一套）。
                # 不做活动半径吸附：位移本来就是 1:1 累计，半径吸附只会让指针
                # 莫名其妙往回跳，反而破坏跟手感。
                self._wrap_cursor(context, event)
                try:
                    context.area.tag_redraw()
                except Exception:
                    pass
                return {'RUNNING_MODAL'}
            if self._mode == 'TRACKBALL':
                self._trackball_step(context, dx, dy, factor)
            elif self._mode == 'DUAL':
                # 两个通道各吃各的位移分量，一次拖动可以同时调两个轴
                uh = self._dir_win.get(self._ax_h)
                uv = self._dir_win.get(self._ax_v)
                dh = dx * sens * (1.0 if uh is None or uh.x >= 0 else -1.0)
                dv = dy * sens * (1.0 if uv is None or uv.y >= 0 else -1.0)
                # v5.2.39：单帧转角限幅（事件被卡过时 dx 是攒出来的）
                cap = getattr(self, 'STEP_MAX_DEG', 30.0)
                dh = cap if dh > cap else (-cap if dh < -cap else dh)
                dv = cap if dv > cap else (-cap if dv < -cap else dv)
                self._ang_h += dh
                self._ang_v += dv
            else:
                # 单轴：环绕式（绕枢轴画圈 1:1 跟手），拿不到枢轴时退回投影式
                self._single_axis_step(context, event, dx, dy, factor)

            self._apply()
            self._sync_keys(context)     # 姿态 = 当前帧曲线值，重估也不会跳回去
            self._header(context)
            # 活动半径：自由 / 双轴 / 单轴全都夹——指针始终留在控制器附近，
            # 但旋转继续按手的位移累计（夹只改位置，不改角度）。
            self._confine_cursor(context, event)
            self._wrap_cursor(context, event)
            try:
                context.area.tag_redraw()
            except Exception:
                pass
            return {'RUNNING_MODAL'}

        if et in {'WHEELUPMOUSE', 'WHEELDOWNMOUSE'} and event.value == 'PRESS':
            # 滚轮向下推一格 = 走到下一根轴：自由 → X → Y → Z → 自由
            step = 1 if et == 'WHEELDOWNMOUSE' else -1
            # 四个动作的修饰键组合全在偏好里自己设
            # （默认 Shift=缩放 / Alt=切骨骼 / Ctrl+Alt=切集合 / 裸滚轮=切轴）
            wheel_act = _modal_wheel_action(event)
            if wheel_act == 'ZOOM':
                self._zoom_view(context, step)
                self._header(context)
                return {'RUNNING_MODAL'}
            if wheel_act == 'COLL':
                if self._kind == 'MOVE':
                    self._move_flush(context)
                # v5.2.15：切集合出错只报提示，模态继续 —— 原来异常一路
                # 冒到 modal() 的兜底，整个旋转/移动被踢掉（切到一半的
                # 姿态也跟着中断），比切不动严重得多。
                _error_wash()   # v5.2.16：同 BONE，先清静默异常
                try:
                    self._switch_collection(context, step)
                except Exception as exc:
                    self.report({'WARNING'}, "切集合失败（模态继续）：%s" % exc)
                self._header(context)
                return {'RUNNING_MODAL'}
            if wheel_act == 'BONE':
                if self._kind == 'MOVE':
                    self._move_flush(context)
                _error_wash()   # 拖拽阶段的静默异常会毒化下一个 RNA 读
                try:
                    self._switch_bone(context, step, extend=event.shift)
                except Exception as exc:
                    self.report({'WARNING'}, "切骨骼失败（模态继续）：%s" % exc)
                self._header(context)
                return {'RUNNING_MODAL'}
            if wheel_act == 'AXIS':
                if self._kind == 'MOVE':
                    self._move_flush(context)   # 换轴前把已拖出的位移落币
                    self._cycle_move_axis(step)
                else:
                    self._cycle_rotate_axis(step, context)
                self._header(context)
                return {'RUNNING_MODAL'}
            # 这个修饰键组合谁都没绑：滚轮当没发生，别偷偷改轴
            return {'RUNNING_MODAL'}

        if event.value == 'PRESS' and et == 'S':
            if getattr(self, "_scaling", False):
                self._scale_commit(context)
            else:
                self._scale_begin(context, event)
            self._header(context)
            return {'RUNNING_MODAL'}

        # 模式互切：用偏好里自己设的进入键（默认 X = 旋转、Z = 移动），
        # 不用原生 G/R。放在切轴判断前面 —— 否则 Z 会被当成「锁 Z 轴」吃掉。
        if event.value == 'PRESS' and self._kind in {'MOVE', 'ROT'}:
            mkey, mkw = move_key_spec()
            rkey, rkw = rot_key_spec()
            if (self._kind != 'MOVE' and move_key_ok()
                    and key_match(event, mkey, mkw)):
                # 旋转 → 移动：无缝衔接，之前转好的姿态原样继承
                try:
                    self._enter_move(context)
                except Exception as exc:
                    self.report({'WARNING'}, "切移动失败：%s" % exc)
                self._header(context)
                return {'RUNNING_MODAL'}
            if self._kind == 'MOVE' and key_match(event, rkey, rkw):
                try:
                    self._enter_rotate(context)
                except Exception as exc:
                    self.report({'WARNING'}, "切旋转失败：%s" % exc)
                self._header(context)
                return {'RUNNING_MODAL'}

        if event.value == 'PRESS' and et in {'X', 'Y', 'Z'} \
                and self._kind in {'MOVE', 'ROT'}:
            if not pref('use_xyz_keys', True):
                # 开关关掉：模态里 X/Y/Z 不做事，但仍然吃掉。
                # 不能放行——放行会让事件走到键位映射，而 X 就是「进入旋转」的键，
                # 于是原地又开一个模态叠上去（多份 HUD、右键要按好几次）。切轴只走滚轮。
                return {'RUNNING_MODAL'}
            if self._kind == 'MOVE':
                idx = MOVE_AXES.index(et)
                self._move_flush(context)
                self._move_axis = None if self._move_axis == idx else idx
            else:
                idx = next(i for i, a in enumerate(ROT_AXES) if a[0] == et)
                # 锁轴切换前先把当前旋转落进基线，否则切轴等于把上一次旋转撤了
                self._rebase(context)
                self._push_undo_step()
                if rig_refine_mode(self._rig) and _finger_segment(self._pb) is None:
                    self._axis_lock = idx
                    st = _rig_spatial_settings(self._rig)
                    if st is not None:
                        st.refine_axis = ROT_AXES[idx][0]
                else:
                    self._axis_lock = None if self._axis_lock == idx else idx
                self._assign_gesture(context)
            self._header(context)
            return {'RUNNING_MODAL'}

        if et == 'MIDDLEMOUSE' and event.value == 'PRESS':
            self._mmb_click = None

        if et == 'MIDDLEMOUSE' and getattr(event, 'alt', False):
            # Alt+中键 = 原生「拖到最近正交视角」（view3d.view_axis CLICK_DRAG，
            # NORTH→顶视 SOUTH→底视 EAST→右视 WEST→左视，斜向→前/后视）
            # 和「Alt+点击拾取对焦」（view_center_pick）。整个按-拖-放都放行给
            # Blender 自己的键位（CLICK_DRAG 是键位合成事件，PRESS 被吃就永远
            # 不会形成），这里不调 _orbit_view。原生那层退场后的善后看
            # MOUSEMOVE 的 _alt_nav 分支和下面的 RELEASE 分支。
            if event.value == 'PRESS':
                self._alt_nav = True
            elif event.value == 'RELEASE' and self._alt_nav:
                self._alt_nav = False
                self._after_view_change(context)
            return {'PASS_THROUGH'}

        if et == 'MIDDLEMOUSE' and event.value == 'PRESS':
            # 旋转模式下：中键单击 = 清空旋转；按住拖动超过 4px 才转视角。
            if (self._kind == 'ROT' and not getattr(event, "alt", False)
                    and not getattr(event, "shift", False)
                    and not getattr(event, "ctrl", False)
                    and not getattr(self, "_scaling", False)):
                self._mmb_click = (event.mouse_x, event.mouse_y,
                                  time.perf_counter())
                return {'RUNNING_MODAL'}
            if not self._orbit_view(context, event):
                self.report({'WARNING'}, "这里不能操作视角，请在 3D 视口里用中键")
            return {'RUNNING_MODAL'}

        if et == 'MIDDLEMOUSE' and event.value == 'RELEASE' and self._mmb_click is not None:
            mx, my = self._mmb_click[0], self._mmb_click[1]
            self._mmb_click = None
            if math.hypot(event.mouse_x - mx, event.mouse_y - my) <= 4.0:
                self._clear_rotation(context)
            return {'RUNNING_MODAL'}

        if et == 'LEFTMOUSE' and event.value == 'PRESS':
            if getattr(self, "_scaling", False):
                self._scale_commit(context)
                self._header(context)
                return {'RUNNING_MODAL'}
            return self._finish(context)

        if et == 'RIGHTMOUSE' and event.value == 'PRESS':
            now = time.perf_counter()
            last = getattr(self, "_last_rmb", 0.0)
            if now - last <= 0.25:
                self._last_rmb = 0.0
                return self._cancel_all(context)
            self._last_rmb = now
            if getattr(self, "_scaling", False):
                self._scale_cancel(context)
                self._header(context)
                return {'RUNNING_MODAL'}
            # 单击右键 = 退回上一步；快速双击右键 = 全部撤销并退出。
            if self._undo_step_back(context):
                self._undo_none = False
                self._solo_redraw(context)
            else:
                self._undo_none = True
            self._header(context)
            return {'RUNNING_MODAL'}

        if et == 'ESC' and event.value == 'PRESS':
            if getattr(self, "_scaling", False):
                self._scale_cancel(context)
                self._header(context)
                return {'RUNNING_MODAL'}
            # ESC 仍然保留为整体放弃
            return self._cancel_all(context)

        # 没用到的按键放行给 Blender，别吞掉原生快捷键
        if event.value == 'PRESS':
            return {'PASS_THROUGH'}
        return {'RUNNING_MODAL'}


# ---------------------------------------------------------------- Rigify IK/FK

_RIG_OP_RE = re.compile(r"operator\('(pose\.rigify_[a-z0-9_]+)',\s*text='([^']+)'")
_RIG_ANY_ID_RE = re.compile(
    r"pose\.rigify_(?:limb_ik2fk|limb_fk2ik|generic_snap|finger_fk2ik)_([A-Za-z0-9_.\-]+)")
_RIG_PROP_RE = re.compile(r"props\.([a-z_]+)\s*=\s*(.+)$")
_RIG_IKFK_PROP_RE = re.compile(
    r"pose_bones\[['\"]([^'\"]+)['\"]\][^\n]*?\[['\"](IK_FK|FK_IK)['\"]\]")
_RIG_PROP_LABEL_RE = re.compile(r"text\s*=\s*['\"][^'\"]*\(([^)]+)\)['\"]")


def _read_op_props(lines, start):
    out = {}
    j = start + 1
    while j < len(lines) and j < start + 16:
        s = lines[j].strip()
        if s.startswith("group2") or s.startswith("layout.") or "operator(" in s:
            break
        m = _RIG_PROP_RE.match(s)
        if m:
            raw = m.group(2)
            try:
                out[m.group(1)] = ast.literal_eval(raw)
            except Exception:
                out[m.group(1)] = raw.strip("'\"")
        j += 1
    return out


def rigify_limbs(rig):
    """解析 rig 自带的 rig_ui.py，拿到 Rigify 原生 snap 按钮的完整参数。

    返回 label → {label, prop_bone, prop_name, to_ik_op, to_ik, to_fk_op, to_fk}

    v5.2：手指也带 IK/FK（`Finger IK (f_index.01.L)`），属性叫 `FK_IK`，
    而且两个按钮分别是 `rigify_finger_fk2ik_*` 和 `rigify_generic_snap_*`。
    以前只认 `IK_FK` 这一个属性名，手指那 10 条全被漏掉。
    """
    if rig is None or rig.type != 'ARMATURE':
        return []
    rid = rig.data.get("rig_id") or ""
    texts = []
    for text in bpy.data.texts:
        try:
            texts.append(text.as_string())
        except Exception:
            continue
    source = None
    if rid:
        for s in texts:
            if ("pose.rigify_limb_ik2fk_" + rid) in s or ("pose.rigify_generic_snap_" + rid) in s \
                    or ("pose.rigify_finger_fk2ik_" + rid) in s:
                source = s
                break
    if source is None:
        # 没有 rig_id（自定义绑定常会清掉，或者生成后又换了骨架）：直接从 UI
        # 文本里反推 rig_id —— 只要文本里有 rigify 的 snap 按钮就认它。
        for s in texts:
            m = _RIG_ANY_ID_RE.search(s)
            if m:
                rid = m.group(1)
                source = s
                break
    if source is None:
        return []
    lines = source.splitlines()

    limbs = {}
    prop_map = {}
    for i, line in enumerate(lines):
        m = _RIG_IKFK_PROP_RE.search(line)
        if m:
            lm = _RIG_PROP_LABEL_RE.search(line)
            if lm:
                prop_map[lm.group(1)] = (m.group(1), m.group(2))
        m = _RIG_OP_RE.search(line)
        if not m:
            continue
        op_id, text_label = m.group(1), m.group(2)
        if "FK->IK (" in text_label:
            label = text_label.split("FK->IK (", 1)[1].rstrip(")")
            props = _read_op_props(lines, i)
            item = limbs.setdefault(label, {"label": label})
            item["to_ik_op"] = op_id
            item["to_ik"] = props
        elif "IK->FK (" in text_label:
            label = text_label.split("IK->FK (", 1)[1].rstrip(")")
            props = _read_op_props(lines, i)
            item = limbs.setdefault(label, {"label": label})
            item["to_fk_op"] = op_id
            item["to_fk"] = props
            if props.get("prop_bone"):
                item["prop_bone"] = props["prop_bone"]
    for label, item in limbs.items():
        if not item.get("prop_bone") and label in prop_map:
            item["prop_bone"] = prop_map[label][0]
            item["prop_name"] = prop_map[label][1]
    return [v for v in limbs.values()
            if v.get("prop_bone") and v.get("to_ik_op") and v.get("to_fk_op")]


# ============================================================ IK / FK 切换
#
# 两套 rig 各走一条路，对外是同一套接口：
#
#   Rigify —— 骨架有 rig_id，UI 文本里有原生 snap 按钮。按钮参数里
#             （fk_bones / ctrl_bones / extra_ctrls）直接写明哪根骨是控制器，
#             不用猜命名；snap 也交给它，pole 和 stretch 全对。
#
#   AniMate Pro 类 —— 控制器骨上挂着 ik_fk_switch（0=IK / 1=FK），十几条
#             driver 盯着它改约束强度。没有原生 snap，就自己算矩阵快照：
#             目标骨 matrix = 源骨 matrix @ (源 rest⁻¹ @ 目标 rest)。
#
# 两边共同的坑：IK 和 FK 控制器**混在同一个集合里**（"左臂"里既有 *_ik 又有
# *_fk），所以「切完藏掉另一套」只能逐骨骼 hide，按集合隐藏没有用。

_IKFK_CACHE = {}


class CN_Limb(object):
    """一个肢体：一个开关 + 两套控制器 + 两次矩阵快照。"""
    __slots__ = ("label", "prop_bone", "prop_name", "fk_value", "ik_value",
                 "ik_controls", "fk_controls",
                 "pairs_to_fk", "pairs_to_ik",
                 "rigify_to_fk", "rigify_to_ik", "is_finger")

    def __init__(self):
        self.label = ""
        self.prop_bone = ""
        self.prop_name = "IK_FK"
        self.fk_value = 1.0
        self.ik_value = 0.0
        self.ik_controls = []
        self.fk_controls = []
        self.pairs_to_fk = []
        self.pairs_to_ik = []
        self.rigify_to_fk = None
        self.rigify_to_ik = None
        self.is_finger = False


def _as_bone_list(raw):
    """Rigify 的 props 里骨骼清单是 JSON 字符串（'["A", "B"]'）。"""
    if not raw:
        return []
    if isinstance(raw, (list, tuple)):
        return [str(x) for x in raw]
    try:
        val = json.loads(raw)
    except Exception:
        return []
    return [str(x) for x in val] if isinstance(val, list) else []


def _as_names(raw):
    """单个骨骼名（'f_index.01_ik.L'）和 JSON 清单都吃。"""
    if not raw:
        return []
    if isinstance(raw, (list, tuple)):
        return [str(x) for x in raw]
    s = str(raw).strip()
    if not s:
        return []
    if s.startswith("["):
        return _as_bone_list(s)
    return [s]


def _chain_depth(pb):
    d = 0
    p = pb.parent
    while p is not None and d < 96:
        d += 1
        p = p.parent
    return d


def _depth_of(rig, name):
    pb = rig.pose.bones.get(name)
    return _chain_depth(pb) if pb is not None else 0


# ---- Rigify

def _limbs_rigify(rig):
    out = []
    for info in rigify_limbs(rig):
        to_fk = info.get("to_fk") or {}
        to_ik = info.get("to_ik") or {}
        lb = CN_Limb()
        lb.label = info.get("label") or info.get("prop_bone", "?")
        lb.prop_bone = info.get("prop_bone", "")
        pname = info.get("prop_name") or "IK_FK"
        pb = rig.pose.bones.get(lb.prop_bone)
        if pb is not None and pname not in pb:
            pname = "FK_IK" if "FK_IK" in pb else pname
        lb.prop_name = pname
        # IK_FK：1 = FK；FK_IK（Rigify 手指）：1 = IK —— 两者的语义是反的
        if pname == "FK_IK":
            lb.fk_value, lb.ik_value = 0.0, 1.0
        else:
            lb.fk_value, lb.ik_value = 1.0, 0.0
        # 手指的那套按钮是 finger_fk2ik / generic_snap，参数名不一样：
        # ik_control / fk_master / fk_chain / ik_chain / constraint_bone
        finger = bool(to_ik.get("fk_master") or to_ik.get("ik_control"))
        if finger:
            lb.is_finger = True
            ik = _as_names(to_ik.get("ik_control")) or \
                _as_names(to_fk.get("output_bones"))
            fk = _as_names(to_ik.get("fk_master")) or \
                _as_names(to_fk.get("input_bones"))
        else:
            fk = _as_names(to_fk.get("fk_bones")) or \
                _as_names(to_ik.get("output_bones"))
            ik = (_as_names(to_fk.get("ctrl_bones"))
                  + _as_names(to_fk.get("tail_bones"))
                  + _as_names(to_fk.get("extra_ctrls")))
            if not ik:
                ik = _as_names(to_ik.get("ctrl_bones"))
        lb.fk_controls = [n for n in dict.fromkeys(fk)
                          if rig.pose.bones.get(n) is not None]
        lb.ik_controls = [n for n in dict.fromkeys(ik)
                          if rig.pose.bones.get(n) is not None
                          and not n.startswith("MCH-")]
        if info.get("to_fk_op"):
            lb.rigify_to_fk = (info["to_fk_op"], to_fk)
        if info.get("to_ik_op"):
            lb.rigify_to_ik = (info["to_ik_op"], to_ik)
        if lb.ik_controls and lb.fk_controls:
            out.append(lb)
    return out


# ---- Rigify 结构兜底（没有 rig_ui.py 文本也认得出肢体）
#
# 自定义绑定十有八九是「Rigify 生成后大改」：rig_ui 文本被删掉、rig_id 对不上，
# 或者手搓但沿用 Rigify 命名（upper_arm_fk / thigh_ik / *_parent）。这时候按
# 按钮参数解析那条路一条都用不上，退回按**骨架结构**认：
#
#   开关   = 带 IK_FK / FK_IK 自定义属性的骨（值 0~1）
#   控制器 = 同侧别、同部位词、名字里带 _fk / _ik 标记的骨（剔除 MCH/ORG/DEF）
#
# 这么认出来的肢体没有原生 snap，走自算快照（几何收敛到毫米级），和「脚本
# 自动运行被关掉」时是同一条路 —— 精度略低但姿势不会弹回 rest。

_IKFK_SWITCH_PROP_RE = re.compile(r"^(ik[_-]?fk|fk[_-]?ik)([_-]?(switch|mode|value))?$", re.I)
_FK_MARK_RE = re.compile(r"[._-]fk(?=[._-]|$)", re.I)
_IK_MARK_RE = re.compile(r"[._-]ik(?=[._-]|$)", re.I)
_INTERNAL_BONE_RE = re.compile(r"^(MCH|ORG|DEF|VIS|WGT)[-_]", re.I)


def _switch_props(rig):
    """骨架里所有像「IK/FK 开关」的自定义属性：[(骨名, 属性名)]。"""
    out = []
    for pb in rig.pose.bones:
        for key in pb.keys():
            if key == "_RNA_UI" or not isinstance(key, str):
                continue
            if not _IKFK_SWITCH_PROP_RE.match(key):
                continue
            try:
                v = float(pb[key])
            except Exception:
                continue
            if -0.001 <= v <= 1.001:
                out.append((pb.name, key))
                break
    return out


def _looks_rigify(rig):
    if rig.data.get("rig_id"):
        return True
    for b in rig.data.bones:
        if b.name.upper().startswith("ORG-"):
            return True
    return False


def _limbs_rigify_struct(rig):
    out = []
    for bone_name, prop in _switch_props(rig):
        side = _limb_side(bone_name)
        words = _limb_words(bone_name)
        if side is None or words is None:
            continue          # 侧别/部位认不出来就不猜，免得把两条腿混成一条
        ik, fk = [], []
        for pb in rig.pose.bones:
            n = pb.name
            if _INTERNAL_BONE_RE.match(n) or _bone_side(n) != side:
                continue
            low = n.lower()
            if not any(w in low for w in words):
                continue
            if _FK_MARK_RE.search(low):
                fk.append(n)
            elif _IK_MARK_RE.search(low) or "pole" in low:
                ik.append(n)
        fk = sorted(dict.fromkeys(fk), key=lambda n: _depth_of(rig, n))
        ik = sorted(dict.fromkeys(ik), key=lambda n: _depth_of(rig, n))
        if len(fk) < 2 or not ik:
            continue
        lb = CN_Limb()
        col = next(iter(rig.pose.bones[bone_name].bone.collections), None)
        lb.label = col.name if col is not None else bone_name
        lb.prop_bone = bone_name
        lb.prop_name = prop
        # FK_IK：1 = IK；IK_FK：1 = FK —— 属性名就能定语义，不用猜 0.5
        if prop.lower().replace("_", "").replace("-", "").startswith("fkik"):
            lb.fk_value, lb.ik_value = 0.0, 1.0
        else:
            lb.fk_value, lb.ik_value = 1.0, 0.0
        lb.fk_controls = fk
        lb.ik_controls = ik
        out.append(lb)
    return out


# ---- AniMate Pro 类（控制器带 ik_fk_switch）

_ARM_WORDS = ("arm", "hand", "forearm", "elbow", "shoulder", "wrist")
_LEG_WORDS = ("thigh", "leg", "foot", "toes", "knee", "ankle")


def _limb_side(prop_bone):
    low = prop_bone.lower()
    for tail in (".l", "_l", ".r", "_r"):
        if low.endswith(tail):
            return tail[1:] if tail.startswith(".") else tail
    if ".l" in low:
        return "l"
    if ".r" in low:
        return "r"
    if "_l" in low:
        return "l"
    if "_r" in low:
        return "r"
    return None


def _bone_side(name):
    """按后缀判侧别。必须只认结尾，否则 "c_leg_pole.r" 会因为中间的
    "_l" 被算进左侧，链到对侧骨头上去。"""
    low = name.lower()
    if low.endswith(".l") or low.endswith("_l"):
        return "l"
    if low.endswith(".r") or low.endswith("_r"):
        return "r"
    return None


def _limb_words(prop_bone):
    low = prop_bone.lower()
    hit_arm = any(w in low for w in _ARM_WORDS)
    hit_leg = any(w in low for w in _LEG_WORDS)
    if hit_arm and not hit_leg:
        return _ARM_WORDS
    if hit_leg and not hit_arm:
        return _LEG_WORDS
    if hit_arm:
        return _ARM_WORDS
    return None


def _animate_deform(rig, ctrl_name, tag):
    """c_arm_fk.l -> arm.l ；c_hand_ik.l -> hand.l ；c_toes_fk.l -> toes_01.l"""
    n = ctrl_name[2:] if ctrl_name.startswith("c_") else ctrl_name
    i = n.lower().find("_" + tag)
    if i < 0:
        return None
    base = n[:i]
    tail = n[i + len(tag) + 1:]
    for cand in (base + tail, base + "_01" + tail, base + "_1" + tail):
        if rig.pose.bones.get(cand) is not None:
            return cand
    return None


def _pairs_for(rig, ctrl_names, tag):
    """返回 [(控制器, 代表它的变形骨)]，按变形骨的层级从近到远。"""
    pairs = []
    for c in ctrl_names:
        d = _animate_deform(rig, c, tag)
        if d:
            pairs.append((c, d))
    pairs.sort(key=lambda p: _depth_of(rig, p[0]))
    return pairs


def _limbs_animate(rig):
    out = []
    for pb in rig.pose.bones:
        if "ik_fk_switch" not in pb.keys():
            continue
        side = _limb_side(pb.name)
        words = _limb_words(pb.name)
        if side is None or words is None:
            continue
        ik, fk = [], []
        for other in rig.pose.bones:
            n = other.name
            low = n.lower()
            if not low.startswith("c_"):
                continue
            if _bone_side(n) != side:
                continue
            if not any(w in low for w in words):
                continue
            if "_fk" in low:
                fk.append(n)
            elif "_ik" in low or "pole" in low:
                ik.append(n)
        if not ik or not fk:
            continue
        lb = CN_Limb()
        col = next(iter(pb.bone.collections), None)
        lb.label = col.name if col is not None else pb.name
        lb.prop_bone = pb.name
        lb.prop_name = "ik_fk_switch"
        lb.fk_value = 1.0
        lb.ik_controls = sorted(dict.fromkeys(ik), key=lambda n: _depth_of(rig, n))
        lb.fk_controls = sorted(dict.fromkeys(fk), key=lambda n: _depth_of(rig, n))
        lb.pairs_to_fk = _pairs_for(rig, lb.fk_controls, "fk")
        lb.pairs_to_ik = _pairs_for(rig, [pb.name], "ik")
        out.append(lb)
    return out


# ---- 游戏系自定义绑定（Anby / SAnby_Rigui 这类）----
#
# 不是 Rigify 标准骨名，也不是 c_ 前缀：开关属性叫 `Arm_IK-FK.L`、
# `Leg_IK_FK.R`，挂在 PROPERTIES 这种属性骨上；snap 按钮是绑定作者自己
# 写的（`sanby_rig.ik_fk_arm`，带 side 参数、自带矩阵吸附）。上面三条路
# 都认不出。这里做两件事：
#   1. 找「像开关」的属性：ik 在前 fk 在后，可带部位前缀和 .L/.R 后缀；
#   2. 从 rig_ui 文本里找自家的 snap 按钮（bl_idname 含 ik_fk / fk_ik），
#      注册成功的归到对应肢体 —— 语义由按钮源码背书（ik_fk_* 把开关写成
#      1.0 = FK，fk_ik_* 写 0.0 = IK），有原生 snap 用原生的，没有就自算。

_CUSTOM_SWITCH_PROP_RE = re.compile(
    r"^[a-z0-9_\-]*ik[-_]?fk(?:[._-](?:l|r))?$", re.I)
_UI_SNAP_IDNAME_RE = re.compile(
    r"""bl_idname\s*=\s*["']([a-z0-9_]+)\.((?:ik[_-]?fk|fk[_-]?ik)[a-z0-9_]*)["']""",
    re.I)
# FK/IK 标记：前缀（FK_Upperarm.L）和中缀（upper_arm_fk.L）都认
_FK_TAG_RE = re.compile(r"(?:^|[._-])fk(?:[._-]|$)", re.I)
_IK_TAG_RE = re.compile(r"(?:^|[._-])ik(?:[._-]|$)", re.I)


def _limb_word_of(text):
    """从属性名 / 按钮名里认部位词：arm / leg，认不出返回 None。"""
    low = (text or "").lower()
    for w in _ARM_WORDS:
        if w in low:
            return "arm"
    for w in _LEG_WORDS:
        if w in low:
            return "leg"
    return None


def _custom_snap_ops(rig):
    """扫 rig_ui 文本，找自家 snap 按钮：{部位: {'fk': (op_id, 有side), 'ik': …}}。

    只认**注册成功**的操作符 —— 文本还在、类没跑起来的烂摊子不碰。
    """
    out = {}
    for txt in bpy.data.texts:
        try:
            s = txt.as_string()
        except Exception:
            continue
        if not s or len(s) > 4_000_000:
            continue
        if ("ik_fk" not in s and "fk_ik" not in s
                and "IK_FK" not in s and "FK_IK" not in s):
            continue
        for m in _UI_SNAP_IDNAME_RE.finditer(s):
            mod, name = m.group(1), m.group(2)
            low = name.lower()
            # ik_fk_* = 切到 FK（源码里把开关写成 1.0）；fk_ik_* = 切到 IK
            direction = "fk" if low.startswith("ik") else "ik"
            limb = _limb_word_of(low)
            if limb is None:
                continue
            mod_obj = None
            try:
                mod_obj = getattr(bpy.ops, mod, None)
                fn = getattr(mod_obj, name, None) if mod_obj is not None \
                    else None
                if fn is None:
                    continue
                rna = fn.get_rna_type()
            except Exception:
                continue
            names = set(p.identifier for p in rna.properties)
            entry = out.setdefault(limb, {})
            if direction not in entry:
                entry[direction] = (mod + "." + name, "side" in names)
    return out


# ---- 开关属性到底哪一端是 IK（不能信按钮上的字）----------------------
#
# 用户报障（Anby）：模型明明是 FK 态，插件判成 IK，按「→ FK」反而切回 IK。
# 实测（移动 IK_Hand.L / FK_Hand.L 看 ORG-Hand.L 跟不跟）：
#   Arm_IK-FK.L = 0.0 → FK 驱动；= 1.0 → IK 驱动。
# 而作者按钮 `sanby_rig.ik_fk_arm`（写着 IK > FK）写的是 1.000 —— 也就是
# 按钮文字和实际语义正好相反（这颗按钮自己按下去才是不生效的那颗）。
# 所以端值只能从**驱动器**读：找被这个属性驱动的约束，看它 COPY 的来源骨
# 是 IK 骨还是 FK 骨，再按表达式算属性 0/1 时它的影响值，影响大的那一端
# 就是这套控制器真正干活的那一端。

_CONS_INF_SV_RE = re.compile(
    r'pose\.bones\["([^"]+)"\]\.constraints\["([^"]+)"\]\.influence$')


def _expr_at(expr, varnames, v):
    """把驱动器表达式按「变量=v」算个数值。只认四则运算，其它一律放弃。"""
    try:
        tree = ast.parse(expr, mode="eval")
    except Exception:
        return None

    def ev(node):
        if isinstance(node, ast.Expression):
            return ev(node.body)
        if isinstance(node, ast.Constant):
            return float(node.value)
        if isinstance(node, ast.Name):
            return float(varnames.get(node.id, v))
        if isinstance(node, ast.UnaryOp):
            val = ev(node.operand)
            return -val if isinstance(node.op, ast.USub) else val
        if isinstance(node, ast.BinOp):
            a, b = ev(node.left), ev(node.right)
            if isinstance(node.op, ast.Add):
                return a + b
            if isinstance(node.op, ast.Sub):
                return a - b
            if isinstance(node.op, ast.Mult):
                return a * b
            if isinstance(node.op, ast.Div):
                return a / b if b else None
        raise ValueError("unsupported")

    try:
        return ev(tree)
    except Exception:
        return None


def _drv_value(d, varnames, v):
    """驱动器在「变量全取 v」时的实际输出：先算表达式，再走驱动器的关键帧曲线。

    v5.2.32 新增。以前只算表达式，忽略驱动器自带的曲线 —— Auto-Rig Pro 的
    约束影响两边都写 `var`，极性全靠曲线做反相（0→1 / 1→0）。只看表达式时
    IK 和 FK 两侧都算成「1 端生效」，极性被读反，开关就写到了对面那一端。
    """
    raw = _expr_at(d.driver.expression, varnames, v)
    if raw is None:
        return None
    try:
        if len(d.keyframe_points):
            return float(d.evaluate(raw))
    except Exception:
        pass
    return raw


def _cons_source_tag(c):
    """约束「照着」的那根骨是 IK 还是 FK（Copy Transforms / Child Of / Armature）。"""
    names = []
    try:
        if getattr(c, "subtarget", ""):
            names.append(c.subtarget)
    except Exception:
        pass
    for t in getattr(c, "targets", []) or []:
        try:
            if t.subtarget:
                names.append(t.subtarget)
        except Exception:
            pass
    for n in names:
        low = n.lower()
        if "ik_fk" in low or "fk_ik" in low:
            continue
        if _FK_TAG_RE.search(low):
            return "FK"
        if _IK_TAG_RE.search(low) or "pole" in low:
            return "IK"
    return ""


def _switch_values(rig, prop_bone, prop_name):
    """返回 (fk_value, ik_value)；读不出来返回 None。"""
    ad = getattr(rig, "animation_data", None)
    if ad is None:
        return None
    want = 'pose.bones["%s"]["%s"]' % (prop_bone, prop_name)
    tally = {}
    for d in ad.drivers:
        vars_ = list(d.driver.variables or [])
        hit = False
        for var in vars_:
            for t in (var.targets or []):
                if t.id == rig and t.data_path == want:
                    hit = True
        if not hit:
            continue
        m = _CONS_INF_SV_RE.match(d.data_path or "")
        if not m:
            continue
        pb = rig.pose.bones.get(m.group(1))
        if pb is None:
            continue
        c = pb.constraints.get(m.group(2))
        if c is None:
            continue
        tag = _cons_source_tag(c)
        if not tag:
            continue
        names = dict((var.name, 0.0) for var in vars_)
        i0 = _drv_value(d, names, 0.0)
        names1 = dict((var.name, 1.0) for var in vars_)
        i1 = _drv_value(d, names1, 1.0)
        if i0 is None or i1 is None or abs(i1 - i0) < 1e-6:
            continue
        live = 1.0 if i1 > i0 else 0.0
        tally.setdefault(tag, []).append(live)
    if "IK" not in tally and "FK" not in tally:
        return None
    ik = max(set(tally["IK"]), key=tally["IK"].count) if "IK" in tally else None
    fk = max(set(tally["FK"]), key=tally["FK"].count) if "FK" in tally else None
    if ik is not None and fk is not None and ik == fk:
        # 两侧都指同一端 = 极性读不出来（曲线没读到 / 绑定用了别的机制）。
        # 旧版在这里默认信 IK，于是把整套判反 —— 交给调用方换显示层或默认值。
        return None
    if ik is not None:
        return (0.0 if ik else 1.0), ik
    return fk, (0.0 if fk else 1.0)


_SCALE_DRV_IDX = {}


def _scale_drv_index(rig):
    """{被驱动的骨名: [(驱动器, 它读的那个属性 data_path)]}，只收控件缩放的驱动。"""
    ad = getattr(rig, "animation_data", None)
    if ad is None:
        return {}
    key = (rig.name, len(ad.drivers))
    hit = _SCALE_DRV_IDX.get(key)
    if hit is not None:
        return hit
    out = {}
    pre = 'pose.bones["'
    for d in ad.drivers:
        dp = d.data_path or ""
        if not dp.startswith(pre) or "custom_shape_scale" not in dp:
            continue
        bone = dp[len(pre):].partition('"')[0]
        if not bone:
            continue
        src = ""
        for var in d.driver.variables:
            for t in (var.targets or []):
                if t.id == rig and (t.data_path or ""):
                    src = t.data_path
                    break
            if src:
                break
        out.setdefault(bone, []).append((d, src))
    _SCALE_DRV_IDX.clear()
    _SCALE_DRV_IDX[key] = out
    return out


def _scale_switch_values(rig, lb):
    """从「控件显示」这一层读极值：哪一端让 IK 控件露出来、哪一端让 FK 露出来。

    Auto-Rig Pro 3.x 不 hide 控制器，它把离开那套的控件 `custom_shape_scale`
    缩到 0（`1-var` + 反相曲线）。这层就是绑定自己的显示逻辑，比按约束名猜
    极性可靠，也不受驱动器曲线、修饰器影响。返回 (fk_value, ik_value) 或 None。
    """
    idx = _scale_drv_index(rig)
    if not idx:
        return None
    want = 'pose.bones["%s"]["%s"]' % (lb.prop_bone, lb.prop_name)

    def probe(names):
        for n in names:
            for d, src in idx.get(n, ()):
                if src != want:
                    continue
                s0 = _drv_value(d, dict((v.name, 0.0) for v in d.driver.variables), 0.0)
                s1 = _drv_value(d, dict((v.name, 1.0) for v in d.driver.variables), 1.0)
                if s0 is None or s1 is None:
                    continue
                if s0 > 0.5 >= s1:
                    return 0.0
                if s1 > 0.5 >= s0:
                    return 1.0
        return None

    ik_v = probe(lb.ik_controls)
    fk_v = probe(lb.fk_controls)
    if ik_v is None or fk_v is None or ik_v == fk_v:
        return None
    return fk_v, ik_v


def _widget_scale_rig(rig, lb):
    """这套绑定的控件可见性是不是「缩放驱动」的（ARP 3.x 一类）。

    是的话插件绝不能往 `bone.hide` 上写：ARP 从不 hide 控制器，离开那套只是
    把控件缩到 0。插件再 hide 一把，切过去的那套就被永久藏住 —— 用户报的
    「IK/FK 切完 FK 控件根本不显示」就是这个叠加出来的。
    """
    idx = _scale_drv_index(rig)
    if not idx:
        return False
    want = 'pose.bones["%s"]["%s"]' % (lb.prop_bone, lb.prop_name)
    for n in list(lb.ik_controls) + list(lb.fk_controls):
        for _d, src in idx.get(n, ()):
            if src == want:
                return True
    return False


def _driven_switch_props(rig):
    """被任何驱动器引用过的自定义开关属性集合 {(骨名, 属性名)}。

    整根骨架没有任何驱动器时返回 None（调用方据此跳过过滤）——
    驱动器引用格式是 pose.bones["PROPERTIES"]["Leg_IK_FK.R"] 这类
    data_path，出现在驱动器变量的目标上。Object 和 Armature 两份
    animation_data 都扫（约束驱动器挂在 Object 上，个别挂 data 上）。
    """
    out = set()
    total = 0
    for ad in (getattr(rig, "animation_data", None),
               getattr(rig.data, "animation_data", None)):
        if not ad:
            continue
        try:
            drivers = ad.drivers
        except Exception:
            continue
        for d in drivers:
            total += 1
            for v in d.driver.variables:
                for t in v.targets:
                    dp = t.data_path or ""
                    m = re.search(r'pose\.bones\["([^"]+)"\]\["([^"]+)"\]', dp)
                    if m:
                        out.add((m.group(1), m.group(2)))
    if not total:
        return None
    return out


def _limbs_custom_ui(rig):
    """从开关属性 + 自家 snap 按钮认肢体。认不出控制器的属性直接跳过。"""
    props = []
    for pb in rig.pose.bones:
        for key in pb.keys():
            if key == "_RNA_UI" or not isinstance(key, str):
                continue
            if not _CUSTOM_SWITCH_PROP_RE.match(key):
                continue
            try:
                v = float(pb[key])
            except Exception:
                continue
            if -0.001 <= v <= 1.001:
                props.append((pb.name, key))
    if not props:
        return []
    # v5.2.24：驱动器零引用的开关属性是**死开关**，直接丢弃。
    #   Ju_Fufu 实测：PROPERTIES 上同时挂着 Leg_IK_FK.R（下划线，驱动
    #   MCH-SWI-Thigh/Calf/Foot 切换链，真开关）和 Leg_IK-FK.R（连字符，
    #   作者改名后的残留，零驱动器引用）——两根都认出来就是 5 行肢体，
    #   右腿出现两个长得一样的开关，用户「摸不着头脑」。
    #   判据沿用 v5.2.13 的哲学：只信驱动器。整根骨架一个驱动器都没有
    #   的极端情况不过滤（机制可能不走驱动器，宁可信其有）。
    driven = _driven_switch_props(rig)
    snap_ops = _custom_snap_ops(rig)
    out = []
    for prop_bone, prop in props:
        if driven is not None and (prop_bone, prop) not in driven:
            continue          # 没有任何驱动器引用它 —— 废属性
        side = _limb_side(prop) or _limb_side(prop_bone)
        limb = _limb_word_of(prop) or _limb_word_of(prop_bone)
        if side is None or limb is None:
            continue          # 侧别 / 部位认不出来就不猜
        words = _ARM_WORDS if limb == "arm" else _LEG_WORDS
        fk, ik = [], []
        for pb in rig.pose.bones:
            n = pb.name
            if _INTERNAL_BONE_RE.match(n) or _bone_side(n) != side:
                continue
            low = n.lower()
            if "ik_fk" in low or "fk_ik" in low:
                continue      # 混合机制骨（FK-IK_Hand 这类），不算控制器
            if not any(w in low for w in words):
                continue
            if _FK_TAG_RE.search(low):
                fk.append(n)
            elif _IK_TAG_RE.search(low) or "pole" in low:
                ik.append(n)
        fk = sorted(dict.fromkeys(fk), key=lambda n: _depth_of(rig, n))
        ik = sorted(dict.fromkeys(ik), key=lambda n: _depth_of(rig, n))
        if len(fk) < 2 or not ik:
            continue
        lb = CN_Limb()
        lb.label = prop
        lb.prop_bone = prop_bone
        lb.prop_name = prop
        # 端值从驱动器实测（见 _switch_values）。Anby 的 Arm_IK-FK.L 是
        # 0 = FK / 1 = IK，跟作者按钮上写反了 —— 不能拿按钮文字当语义。
        vals = _switch_values(rig, prop_bone, prop)
        lb.fk_value, lb.ik_value = vals if vals else (1.0, 0.0)
        lb.fk_controls = fk
        lb.ik_controls = ik
        snap = snap_ops.get(limb) or {}
        if "fk" in snap:
            op_id, has_side = snap["fk"]
            lb.rigify_to_fk = (op_id, {"side": side.upper()} if has_side
                               else {})
        if "ik" in snap:
            op_id, has_side = snap["ik"]
            lb.rigify_to_ik = (op_id, {"side": side.upper()} if has_side
                               else {})
        out.append(lb)
    return out


# ---- 统一入口

def _ikfk_sig(rig):
    try:
        return (rig.name, len(rig.pose.bones),
                rig.data.get("rig_id") or "")
    except Exception:
        return None


def ikfk_limbs(rig):
    """返回 (类型, [CN_Limb])；类型是 'rigify' / 'animate' / ''。"""
    if rig is None or getattr(rig, "type", None) != 'ARMATURE':
        return "", []
    sig = _ikfk_sig(rig)
    if sig is not None and sig in _IKFK_CACHE:
        return _IKFK_CACHE[sig]
    kind, limbs = "", []
    try:
        limbs = _limbs_rigify(rig)
        if limbs:
            kind = "rigify"
    except Exception:
        limbs = []
    # 有 rig_ui 文本的肢体优先（能走原生 snap）；文本里没写到的肢体
    # （创作者自己加的、或者 UI 被删掉一半的）再按结构补一遍。
    try:
        if kind == "rigify":
            have = set(lb.prop_bone for lb in limbs)
            for lb in _limbs_rigify_struct(rig):
                if lb.prop_bone not in have:
                    limbs.append(lb)
        elif _looks_rigify(rig):
            limbs = _limbs_rigify_struct(rig)
            if limbs:
                kind = "rigify"
    except Exception:
        pass
    if not limbs:
        try:
            limbs = _limbs_animate(rig)
            if limbs:
                kind = "animate"
        except Exception:
            limbs = []
    # 游戏系自定义绑定（开关属性挂 PROPERTIES 骨、snap 按钮是作者自己写
    # 的）：标准 Rigify / 结构兜底都认不出的属性，在这里补上。已认出的
    # （prop_bone, prop_name）不重复加。
    try:
        have = set((lb.prop_bone, lb.prop_name) for lb in limbs)
        for lb in _limbs_custom_ui(rig):
            if (lb.prop_bone, lb.prop_name) not in have:
                limbs.append(lb)
                if not kind:
                    kind = "custom"
    except Exception:
        pass
    # 端值统一按驱动器实测校正：各绑定的开关语义完全自由（Rigify 的
    # IK_FK 是 1=FK，Anby 的 Arm_IK-FK.L 反过来是 1=IK，手指的 FK_IK 又是
    # 1=IK）。只要能从驱动器读出来，就以驱动器为准 —— 读不出来才用默认值。
    for lb in limbs:
        vals = (_scale_switch_values(rig, lb)
                or _switch_values(rig, lb.prop_bone, lb.prop_name))
        if vals:
            lb.fk_value, lb.ik_value = vals
    if sig is not None:
        _IKFK_CACHE[sig] = (kind, limbs)
    return kind, limbs


def limb_mode(rig, lb):
    """当前是 IK 还是 FK —— 按这个肢体自己的两个端点值比，不写死 0.5。

    Rigify 的肢体用 `IK_FK`（1 = FK），手指用 `FK_IK`（1 = IK），
    两者语义正好相反，拿 0.5 硬比会把手指永远判成 IK。
    """
    pb = rig.pose.bones.get(lb.prop_bone)
    if pb is None or lb.prop_name not in pb:
        return None
    try:
        v = float(pb[lb.prop_name])
    except Exception:
        return None
    fk_v = float(getattr(lb, "fk_value", 1.0))
    ik_v = float(getattr(lb, "ik_value", 0.0))
    if abs(v - fk_v) <= abs(v - ik_v):
        return "FK"
    return "IK"


def _op_prop_kwargs(op_id, props):
    """按操作符真实的属性类型摆参数。

    rig_ui.py 里 `props.fk_chain = '["a","b"]'` 是一段**字符串字面量**，
    ast.literal_eval 会把它变成 Python list；直接喂给 StringProperty 会
    TypeError，snap 就悄悄失败了（表现出来就是「一切换姿势就弹回 rest」）。
    这里查一次操作符的 RNA，字符串属性收到 list 就转回 JSON 文本。
    """
    out = {}
    if not props:
        return out
    spec = {}
    try:
        mod, _, name = op_id.partition(".")
        if "." in op_id and mod != "pose":
            fn = getattr(getattr(bpy.ops, mod), name)
        else:
            fn = getattr(bpy.ops.pose, name)
        rna = fn.get_rna_type()
        spec = dict((p.identifier, p.type) for p in rna.properties)
    except Exception:
        spec = {}
    for k, v in props.items():
        want = spec.get(k)
        if want == 'STRING' and isinstance(v, (list, tuple)):
            out[k] = json.dumps([str(x) for x in v])
        elif want == 'BOOLEAN' and isinstance(v, (list, tuple)):
            out[k] = tuple(bool(x) for x in v)
        elif want == 'FLOAT' and isinstance(v, (list, tuple)):
            out[k] = tuple(float(x) for x in v)
        else:
            out[k] = v
    return out


def _snap_by_matrix(rig, pairs):
    """目标骨 matrix = 源骨 matrix @ (源 rest⁻¹ @ 目标 rest)。"""
    vl = bpy.context.view_layer
    done = 0
    for tgt_name, src_name in pairs:
        tgt = rig.pose.bones.get(tgt_name)
        src = rig.pose.bones.get(src_name)
        if tgt is None or src is None:
            continue
        off = src.bone.matrix_local.inverted() @ tgt.bone.matrix_local
        tgt.matrix = src.matrix @ off
        try:
            vl.update()
        except Exception:
            pass
        done += 1
    return done


_DEF_STRIP_FK_RE = re.compile(r"[._]fk(?=[._]|$)", re.I)
_DEF_STRIP_IK_RE = re.compile(r"[._]ik(?=[._]|$)", re.I)
# 主 IK 目标挂在末端（Rigify 的 thigh_ik / Hip_ik 在脚底），名字里的
# 关节词是「这条链从哪根开始」，不是「目标点在哪根」，不能按名字配。
_PROXIMAL_IK_RE = re.compile(
    r"(thigh|hip|upper_arm|upperarm|shoulder|clavicle|shin|knee)", re.I)


def _deform_pool(rig):
    """配对候选骨：ORG 优先，其次 DEF。

    Rigify 里 FK 控制器和 IK 目标都是照 **ORG** 骨（原始关节）做的，
    DEF 是蒙皮骨，带着扭转分布（upper_arm 的 DEF 会把扭转折到手臂上）。
    拿 DEF 去对 FK 控制器，手会偏 10 cm —— 这是 v5.2 实测踩到的。
    """
    orgs = [b.name for b in rig.data.bones
            if b.name.upper().startswith("ORG")]
    defs = [b.name for b in rig.data.bones
            if b.name.upper().startswith("DEF")]
    if orgs or defs:
        return orgs + defs
    # 没有 ORG/DEF 前缀的绑定（Auto-Rig Pro 系）：退回蒙皮骨，同样剔机制骨
    pool = _skin_bone_names(rig)
    return pool or [b.name for b in rig.data.bones]


def _deform_of(rig, ctrl, strip_re):
    base = strip_re.sub("", ctrl or "")
    for cand in ("ORG-" + base, "ORG_" + base,
                 "DEF-" + base, "DEF_" + base, base):
        # 不能把控制器自己当变形骨：`f_index.01_master.L` 剥不掉后缀时
        # base 就是自己，配成 (自己, 自己) 等于什么都没做还返回「成功」。
        if cand and cand != ctrl and rig.pose.bones.get(cand) is not None:
            return cand
    return None


def _nearest_deform_head(rig, ctrl, pool, limit=0.08):
    """rest 头部位置最近的变形骨。"""
    pb = rig.pose.bones.get(ctrl)
    if pb is None or not pool:
        return None
    ref = pb.bone.head_local
    best, bestd = None, None
    for n in pool:
        b = rig.data.bones.get(n)
        if b is None:
            continue
        d = (b.head_local - ref).length
        if bestd is None or d < bestd:
            best, bestd = n, d
    return best if bestd is not None and bestd <= limit else None


def _limb_pole_angle(rig, lb):
    """这根肢体 IK 约束上的 pole_angle（Blender 对 pole 向量的角度偏置）。

    Rigify 的腿是 -90°。不把这个角算进去，pole 摆的方向会整体差 90°，
    解出来的膝盖偏离原位两三厘米、转 20° —— 看着就是「切完膝盖扭一下」。
    """
    pool = set(lb.ik_controls) | set(lb.fk_controls)
    for pb in rig.pose.bones:
        for c in pb.constraints:
            if c.type != 'IK':
                continue
            if (c.pole_subtarget and c.pole_subtarget in pool) or \
                    (c.subtarget and c.subtarget in pool):
                try:
                    return float(c.pole_angle)
                except Exception:
                    return 0.0
    return 0.0


def _pole_targets(rig, lb):
    """真正挂在 IK 约束 `pole_subtarget` 上的骨骼名。

    v5.2.1 的判据是「名字里带 target 或 pole 就挪」。Dialyn-琉音 实测：
    手臂的 `upper_arm_ik_target.L` 名字里确实有 target，可它是**IK 目标本身**
    （MCH-upper_arm_ik_target 的父级、约束的 pole_subtarget 都指向它），
    一挪就是 223.7 mm；腿的 `thigh_ik_target.L` 更狠，840.2 mm。
    挪完之后无论走原生 snap 还是自算快照，读到的都是「已经被挪坏的 IK 状态」，
    于是 14 根肢体全部被判「切不过去」。现在只认约束上真实的 pole_subtarget，
    名字一概不信。
    """
    pool = set(lb.ik_controls) | set(lb.fk_controls)
    out = []
    for pb in rig.pose.bones:
        for c in pb.constraints:
            if c.type != 'IK' or not c.pole_subtarget:
                continue
            n = c.pole_subtarget
            if (n in pool and n not in out
                    and rig.pose.bones.get(n) is not None):
                out.append(n)
    return out


def _snap_pole(rig, lb):
    """把 pole 摆到「当前膝盖（手肘）弯向的那一侧」，**保持它到链根的半径**。

    v5.2.3：两处改动。
      1. 只碰 `_pole_targets()` 认出来的真 pole，不再按名字猜。
      2. 半径必须保住：原实现把 pole 摆到「肘 + 方向 × 上臂长」，那是按上臂
         长度硬算的；Rigify 系手臂的 pole 本来就在离身体很远的地方
         （rest 时 y=0.39 m），改半径等于把肘平面整个转过去。
    最后按约束的 pole_angle 反向旋转一次，抵消 Blender 对 pole 向量的偏置。
    """
    names = lb.fk_controls[:3]
    if len(names) < 3:
        return 0
    a, b, c = [rig.pose.bones.get(n) for n in names]
    if a is None or b is None or c is None:
        return 0
    pa, pm, pc = a.head.copy(), b.head.copy(), c.head.copy()
    axis = pc - pa
    if axis.length < 1e-6:
        return 0
    an = axis.normalized()
    proj = pa + an * (pm - pa).dot(an)
    v = pm - proj
    if v.length < 1e-4:
        return 0              # 完全伸直：没有弯曲方向，pole 保持原样
    dirv = v.normalized()
    ang = _limb_pole_angle(rig, lb)
    if abs(ang) > 1e-4:
        try:
            dirv = Matrix.Rotation(ang, 4, an) @ dirv
        except Exception:
            pass
    done = 0
    for pn in _pole_targets(rig, lb):
        p = rig.pose.bones.get(pn)
        if p is None:
            continue
        cur = p.matrix.translation.copy()
        rad = (cur - pa).length
        if rad < 1e-5:
            rad = (pm - pa).length
        m = p.matrix.copy()
        m.translation = pa + dirv * rad
        p.matrix = m
        try:
            bpy.context.view_layer.update()
        except Exception:
            pass
        done += 1
    return done


def _pole_optimize(rig, lb, plan, want):
    """翻完开关后再决定要不要动 pole —— 只有真的把中段误差压小才留。

    做法：记下原状，绕「链根→末端」轴把 pole 转 7 个方向，量肘/膝到目标的
    距离；最好的那个要明显优于原状（< 98%）才采用，否则整根还原。

    Dialyn-琉音 实测：同一个 MCH 上叠了两条 chain_count=2 的 IK 约束，pole
    基本不起作用 —— 转满 360° 误差只在 21.9~22.0 mm 之间徘徊。这种时候
    原地不动就是最优解；v5.2.1 在翻开关前盲挪 22 cm，把整条手臂甩飞。
    """
    if not plan:
        return False
    poles = [n for n in (plan.get("poles") or []) if rig.pose.bones.get(n)]
    if not poles:
        return False
    root_n, mid_n, end_n = (plan.get("root_def"), plan.get("mid_def"),
                            plan.get("end_def"))
    rd, md, ed = (rig.pose.bones.get(root_n or ""),
                  rig.pose.bones.get(mid_n or ""),
                  rig.pose.bones.get(end_n or ""))
    if rd is None or md is None or ed is None or want.get(mid_n) is None:
        return False
    base = {n: rig.pose.bones[n].matrix.copy() for n in poles}
    root = rd.matrix.translation.copy()
    goal = ed.matrix.translation.copy()
    axis = goal - root
    if axis.length < 1e-6:
        return False
    an = axis.normalized()
    offs = {n: base[n].translation - root for n in poles}
    base_err = (md.matrix.translation - want[mid_n]).length
    best_err, best_mats = base_err, None
    for k in range(1, 8):
        rot = Matrix.Rotation(math.radians(45.0 * k), 4, an)
        cand = {}
        for n in poles:
            m = base[n].copy()
            m.translation = root + (rot @ offs[n])
            rig.pose.bones[n].matrix = m
            cand[n] = m
        _flush_rig(rig)
        d = (md.matrix.translation - want[mid_n]).length
        if d < best_err:
            best_err, best_mats = d, cand
    if best_mats is None or best_err >= base_err * 0.98:
        for n in poles:
            rig.pose.bones[n].matrix = base[n]
        _flush_rig(rig)
        return False
    for n in poles:
        rig.pose.bones[n].matrix = best_mats[n]
    _flush_rig(rig)
    return True


def _snap_generic(rig, lb, target):
    """Rigify 原生 snap 用不了时的自算快照：宁可慢一点，也不许姿势弹回 rest。"""
    pool = _deform_pool(rig)
    pairs = []
    if target == "FK":
        for c in lb.fk_controls:
            d = _deform_of(rig, c, _DEF_STRIP_FK_RE)
            if d is None and pool:
                d = _nearest_deform_head(rig, c, pool)
            if d:
                pairs.append((c, d))
        return _snap_by_matrix(rig, pairs) > 0
    for c in lb.ik_controls:
        low = c.lower()
        if "target" in low or "pole" in low:
            continue
        if _PROXIMAL_IK_RE.search(c):
            d = _nearest_deform_head(rig, c, pool) if pool else None
        else:
            d = _deform_of(rig, c, _DEF_STRIP_IK_RE)
            if d is None and pool:
                d = _nearest_deform_head(rig, c, pool)
        if d is None:
            continue
        pairs.append((c, d))
    n = _snap_by_matrix(rig, pairs)
    _snap_pole(rig, lb)
    return n > 0


def _set_bone_hidden(rig, name, hidden):
    """藏/显一根骨：`Bone.hide` 和 `PoseBone.hide` 两个都要动。

    不同绑定的藏法不一样 —— Rigify 系走 `Bone.hide`，而 RIG-Klukai 这类
    自带 UI 脚本的绑定走 `PoseBone.hide`。只改前者的话，切完 FK 骨头还是
    藏着的，导航列表里只剩 `Hip_parent_L` 那个属性骨（用户报的第一根问题）。
    """
    b = rig.data.bones.get(name)
    if b is not None:
        try:
            b.hide = hidden
        except Exception:
            pass
    pb = rig.pose.bones.get(name)
    if pb is not None:
        try:
            pb.hide = hidden
        except Exception:
            pass


def _col_tag(name):
    """集合名里的 IK / FK 标记：`IK_Arm.L`、`Arm.L (IK)`、`Leg_IK_FK.L`。"""
    toks = [t.lower() for t in re.split(r"[^0-9A-Za-z]+", name or "") if t]
    if "ik" in toks:
        return "IK"
    if "fk" in toks:
        return "FK"
    return ""


def _bone_cols(pb):
    try:
        return list(pb.bone.collections)
    except Exception:
        return []


def _col_visible(col):
    v = getattr(col, "is_visible", None)
    return True if v is None else bool(v)


def _col_set_visible(col, visible):
    """单个集合显/隐；要显的时候把祖先链一起打开（父集合关着，开子集合没用）。"""
    if visible:
        chain, c, guard = [], col, 0
        while c is not None and guard < 16:
            chain.append(c)
            c = getattr(c, "parent", None)
            guard += 1
        for c in reversed(chain):
            if not _col_visible(c):
                try:
                    c.is_visible = True
                except Exception:
                    pass
    elif _col_visible(col):
        try:
            col.is_visible = False
        except Exception:
            pass


_SKIP_VIS_COL = re.compile(r"^(mch|color|def|org|vis)[_\-\.]", re.I)


def _col_chain_visible(col):
    """集合自己连同所有祖先都开着，才算真能看见。父集合关着，子集合开了也没用。"""
    c, guard = col, 0
    while c is not None and guard < 16:
        if not _col_visible(c):
            return False
        c = getattr(c, "parent", None)
        guard += 1
    return True


def _ensure_set_visible(rig, names):
    """目标那套控制器是不是**整组**都落在关闭的集合里 —— 是就把该集合打开。

    v5.2.33 新增。背景（rig.002 实测）：ARP 的可见性有两道闸，第一道是
    `custom_shape_scale` 由开关驱动（0/1），第二道是**骨骼集合可见性**。
    插件过去只管第一道。这套骨架的腿部 FK 控制器（c_thigh_fk.r 等）被归到
    `根部` 集合，而 `根部` 是关着的 —— 于是切到 FK 时缩放虽然回到 1，
    视口里一根都画不出来，用户看到的就是「FK 控件根本没显示」。
    命名里带 IK/FK 的配对集合（Anby / Rigify）走不到这里，那是上面分支的事。

    只在**一组全不可见**时动手：正常绑定一根就能看见，直接返回 0，零副作用。
    挑选原则是「装这套控制器最多的那个集合」，并跳过 mch/color/def/org/vis
    这类机构或着色集合，避免为了显示控制器把机构骨翻出来。
    """
    groups = []
    for n in names:
        pb = rig.pose.bones.get(n)
        if pb is None:
            continue
        cols = [c for c in _bone_cols(pb) if c is not None]
        if not cols:
            continue
        if any(_col_chain_visible(c) for c in cols):
            return 0                      # 这套本来就能看见，什么都不做
        groups.append(cols)
    if not groups:
        return 0
    tally = {}
    for cols in groups:
        for c in cols:
            if _SKIP_VIS_COL.match(c.name or ""):
                continue
            tally[c.name] = tally.get(c.name, 0) + 1
    if not tally:                          # 全是机构/着色集合：退而求其次
        for cols in groups:
            for c in cols:
                tally[c.name] = tally.get(c.name, 0) + 1
    if not tally:
        return 0
    best = max(sorted(tally), key=lambda k: tally[k])
    col = _coll_get(rig, best)
    if col is None:
        return 0
    try:
        _col_set_visible(col, True)
    except Exception:
        return 0
    return 1


def _vis_twin(rig, name):
    """`IK_Arm_Pole.L` 有没有 `VIS-IK_Arm_Pole.L` 这种「显示替身」。

    有替身的骨是刻意的机制骨：真正抓的那个控件是 VIS 骨，自己藏着才对。
    """
    for pre in ("VIS-", "VIS_", "vis-"):
        if rig.pose.bones.get(pre + name) is not None:
            return True
    return False


def ikfk_sync_visibility(rig, lb, target):
    """切完只留当前那套控制器：**优先按骨骼集合切**，切不了才逐根藏。

    v5.2.12 重写。用户报「另一套还留着 / 该出的没出」，根因是这里只写了
    `bone.hide`：Anby 这类绑定把两套控制器分装进 `IK_Arm.L` / `FK_Arm.L`
    集合（Rigify 是 `Arm.L (IK)` / `Arm.L (FK)`），真正让控制器出现在视图
    里的是**集合可见性**。逐根 hide 只能藏掉自己那份，集合里的其他成员
    （`VIS-…` 假骨、`Elbow_Pin`、`IK_Palm`、`Arm_Parent`）照样挂在视图里
    ——「另一套还留着」；反过来，目标那套的集合若关着，把 hide 清掉也什么
    都看不见 ——「该出的没出」（Anby 的 `IK_Arm.L` 存盘时就是关着的）。
    所以：两侧都有名字带 IK/FK 的集合 → 关一套开一套；没有配对集合
    （控制器都堆在同一个集合里）→ 退回逐根 hide。
    """
    want = "FK" if target == "FK" else "IK"
    other = "IK" if want == "FK" else "FK"
    keep = list(lb.fk_controls if want == "FK" else lb.ik_controls)
    drop = list(lb.ik_controls if want == "FK" else lb.fk_controls)
    if _widget_scale_rig(rig, lb):
        # v5.2.32：缩放驱动型绑定（ARP 3.x）自己负责显示 —— 开关一翻，离开
        # 那套的控件 `custom_shape_scale` 归 0，进来的那套回到 1。插件在这里
        # 只做一件事：把历史上被本插件写进 `bone.hide` 的控制器放出来。
        # 旧版本对这类绑定照样 hide，而绑定永远不会去解 hide，于是切一次少一套，
        # 最后两套一起不可见（用户报的「FK 控件根本没显示出来」）。
        repaired = 0
        for n in keep + drop:
            pb = rig.pose.bones.get(n)
            b = rig.data.bones.get(n)
            if pb is None and b is None:
                continue
            if (b is not None and b.hide) or (pb is not None and pb.hide):
                _set_bone_hidden(rig, n, False)
                repaired += 1
        # 第二道闸：整套控制器若被关在某个集合里，缩放=1 也画不出来。
        repaired += _ensure_set_visible(rig, keep)
        if repaired:
            _flush_rig(rig)
        return
    if not pref("sync_collections", True):
        return
    keep_cols, drop_cols = {}, {}
    for names, tag, bucket in ((keep, want, keep_cols), (drop, other, drop_cols)):
        for n in names:
            pb = rig.pose.bones.get(n)
            if pb is None:
                continue
            for c in _bone_cols(pb):
                if _col_tag(c.name) == tag:
                    bucket.setdefault(c.name, c)
    if keep_cols or drop_cols:
        for c in keep_cols.values():
            _col_set_visible(c, True)
        for c in drop_cols.values():
            _col_set_visible(c, False)
        # 集合开着、骨却是单根 H 过的（Anby 存盘状态正是如此）：清掉，
        # 否则「该出的」还是差几根。带 VIS 替身的不动 —— 那是机制骨。
        for n in keep:
            pb = rig.pose.bones.get(n)
            if pb is None or not pb.bone.hide:
                continue
            if _vis_twin(rig, n):
                continue
            if any(_col_tag(c.name) == want for c in _bone_cols(pb)):
                _set_bone_hidden(rig, n, False)
        return
    for n in drop:
        _set_bone_hidden(rig, n, True)
    for n in keep:
        _set_bone_hidden(rig, n, False)
        # Rigify 的手指 IK 控制器住在一直关着的 `Fingers (Detail)` 里：
        # 光清 hide 它还是不出现（实测 10 根手指全「该出的没出」）。
        # 只在它一根可见集合都没有时才把集合打开，别去动别的。
        pb = rig.pose.bones.get(n)
        if pb is None:
            continue
        cols = _bone_cols(pb)
        if not any(_col_visible(c) for c in cols):
            for c in cols:
                _col_set_visible(c, True)


_POSE_DRIFT_LIMIT = 0.12      # 12 cm：到这程度才算"炸姿势"，宁可保持原模式
_POSE_ANG_LIMIT = math.radians(15.0)   # 15°：脚掌被扭过去也算炸姿势
# v5.2.3 的「照切但提示」档。Rigify 系骨架 FK→IK 的物理下限就是这个量级：
# 手/脚能对到 0.0 mm，肘/膝受骨骼长度与 IK 求解器限制会差 2~4 cm。
_ACCEPT_DRIFT = 0.06          # 6 cm：肢体自身最大位移
_ACCEPT_ANG = math.radians(30.0)   # 30°：肢体自身最大转角


def _flush_rig(rig):
    try:
        rig.update_tag()
        bpy.context.view_layer.update()
        bpy.context.evaluated_depsgraph_get().update()
        bpy.context.view_layer.update()
    except Exception:
        pass


def _pose_state(rig):
    """整根骨架的矩阵快照（matrix_basis，不依赖约束求值顺序）。"""
    out = []
    for pb in rig.pose.bones:
        try:
            out.append((pb.name, pb.matrix_basis.copy()))
        except Exception:
            pass
    return out


def _pose_restore(rig, state):
    for name, mb in state:
        pb = rig.pose.bones.get(name)
        if pb is not None:
            try:
                pb.matrix_basis = mb
            except Exception:
                pass
    _flush_rig(rig)


def _measured_bones(rig):
    """量谁：DEF（蒙皮）+ ORG（原始关节）。

    ORG 必须算进来 —— 几何收敛要靠它当「肩/肘/腕」三个关节的参照，
    只量 DEF 的话手臂收起时偏差会被蒙皮骨夸大，收敛也没数据可用。

    没有 DEF/ORG 前缀的绑定（Auto-Rig Pro 系：`rig` / `rig.001` 这类，
    变形骨直接叫 `hand.l`）退回 `use_deform`，再退到全部骨骼 —— 否则
    量测集是空的，`_skin_drift` 永远返回 0，炸姿势也会被判成「切成功」。
    """
    out = []
    for b in rig.data.bones:
        up = b.name.upper()
        if up.startswith("DEF") or up.startswith("ORG"):
            out.append(b.name)
    if out:
        return out
    out = _skin_bone_names(rig)
    return out or [b.name for b in rig.data.bones]


# 没有 DEF/ORG 的绑定里，use_deform 会混进机制骨：ARP 把 stretch / twist /
# bend 也标成变形骨，而 IK↔FK 一切换它们本来就该变长度、变朝向。
# 拿它们当「不许动」的基准，能报出 61° 的假失败。
_HELPER_BONE_RE = re.compile(
    r"(^c_|^ctrl|^wgt|widget|stretch|twist|bend|pole|roll|follow"
    r"|_ik([._]|$)|_fk([._]|$)|^mch|^org[-_.]|^def[-_.]"
    r"|license|licence|watermark|水印)", re.I)
# 肢体量测集里也要剔掉的补偿骨：拧转 / 拉伸 / 弯曲。IK 与 FK 对它们的
# 驱动方式本来就不同，往返切一次能差几十度，但蒙皮位置一动不动。
_TWIST_HELPER_RE = re.compile(r"(twist|stretch|bend)", re.I)


def _skin_bone_names(rig):
    """真正带动网格的骨：use_deform 里剔掉机制骨；剔完空了就退回原集合。"""
    raw = [b.name for b in rig.data.bones if b.use_deform]
    if not raw:
        return []
    clean = [n for n in raw if not _HELPER_BONE_RE.search(n)]
    return clean or raw


def _skin_positions(rig):
    """切换前后要「不许动」的那批骨的头部位置。"""
    out = {}
    for name in _measured_bones(rig):
        pb = rig.pose.bones.get(name)
        if pb is not None:
            out[name] = pb.matrix.translation.copy()
    return out


def _skin_drift(rig, before):
    worst = 0.0
    for n, pos in before.items():
        pb = rig.pose.bones.get(n)
        if pb is None:
            continue
        d = (pb.matrix.translation - pos).length
        if d > worst:
            worst = d
    return worst


def _skin_rot_drift(rig, before_q):
    """旋转漂移（弧度）：只量位置会漏掉「脚掌被扭了 78°」这种。

    RIG-Skins 的左脚就是：踝位置对得上，脚掌整体转过去 78°，
    而位置量测显示 0 mm —— 于是被判成「切成功」。
    """
    worst = 0.0
    for n, q in (before_q or {}).items():
        pb = rig.pose.bones.get(n)
        if pb is None:
            continue
        try:
            ang = pb.matrix.to_quaternion().rotation_difference(q).angle
        except Exception:
            continue
        ang = min(ang, 2.0 * math.pi - ang)
        if ang > worst:
            worst = ang
    return worst


def _finger_chain(lb):
    """手指的 FK 链骨名（`f_index.01.L` / `.02` / `.03`）。

    不能直接用 lb.fk_controls：Rigify 的手指只把它那个 master 控制骨算进
    fk_controls，真正带变形骨的是链上三节，名字藏在原生按钮的 props 里
    （fk_chain / output_bones）。这里把两处都捞一遍。
    """
    out = []
    for call in (lb.rigify_to_ik, lb.rigify_to_fk):
        if not call:
            continue
        props = call[1] or {}
        for key in ("fk_chain", "output_bones", "fk_bones"):
            for n in _as_names(props.get(key)):
                if n not in out:
                    out.append(n)
    return out


def _finger_calc(rig, lb, target):
    """手指的兜底快照（只在骨架原生 snap 不可用时走这条）。

    手指的原生按钮（`rigify_finger_fk2ik` / `generic_snap`）要靠文件里
    「自动运行脚本」开着才会注册。关着的时候 bpy.ops 里根本没有这两个
    算子，v5.2.1 于是直接拒绝切换 —— 手指永远停在原模式。
    这里退回按矩阵对齐：IK 方向把 `f_xxx.01_ik` 摆到 FK 末节（指尖）的位置，
    FK 方向按各节变形骨把 FK 链摆回去。精度不如原生（原生会连 roll 一起算），
    但姿势不会弹回 rest。
    """
    if not lb.ik_controls:
        return False
    chain = [n for n in _finger_chain(lb) if rig.pose.bones.get(n)]
    if not chain:
        return False
    if target == "FK":
        pairs = []
        for c in chain:
            d = _deform_of(rig, c, _DEF_STRIP_FK_RE)
            if d:
                pairs.append((c, d))
        return _snap_by_matrix(rig, pairs) > 0
    # 从末端往前找第一节能对上变形骨的 —— fk_chain 末尾常带
    # `f_index.01.L.001` 这种辅助骨，它没有对应的 ORG，直接取 [-1] 会配对失败。
    d = None
    for c in reversed(chain):
        d = _deform_of(rig, c, _DEF_STRIP_FK_RE)
        if d:
            break
    if not d:
        return False
    return _snap_by_matrix(rig, [(lb.ik_controls[0], d)]) > 0


def _limb_deform_names(rig, lb):
    """这条肢体自己的变形骨（判定用的量测集，不含全身）。

    v5.2.1 判定时量整根骨架 795 根骨：任何一根与本次切换无关的骨头
    （头发、另一条手臂、表情骨）只要转了 15°，切换就被判失败。
    Dialyn-琉音 上「自算 24 mm / 转 15°」正是这么被判死的。

    v5.2.10：无 ORG/DEF 的前缀式绑定（FK_Upperarm.L / IK_Hand.L 这类，
    游戏系 rig）`_deform_of` 全部落空 —— 量测集退化成控制器本身，吸附
    本来就要动控制器，位移全是假阳性，两条路都会被判「切不过去」。
    这里对落空的控制器按 rest 头部位置找最近的变形骨兜底。
    """
    out = []
    pool = None
    for c in lb.fk_controls:
        d = _deform_of(rig, c, _DEF_STRIP_FK_RE)
        if d is None:
            if pool is None:
                pool = _deform_pool(rig)
            d = _nearest_deform_head(rig, c, pool) if pool else None
        if d and d not in out:
            out.append(d)
    for c in lb.ik_controls:
        d = _deform_of(rig, c, _DEF_STRIP_IK_RE)
        if d is None:
            if pool is None:
                pool = _deform_pool(rig)
            d = _nearest_deform_head(rig, c, pool) if pool else None
        if d and d not in out:
            out.append(d)
    # 拧转 / 拉伸骨不能进量测集：IK 和 FK 用的是两套扭量分配，
    # 往返切一次，同一根 `ORG-ForearmTwist01.L` 能差 77°，而手的位置只差
    # 5 mm、蒙皮看着一模一样（实测第二次 IK→FK 必被判「转 77°」卡死）。
    # 它们本来就是「为了不扭坏皮肤才存在的补偿骨」，不是姿势。
    out = [n for n in out if not _TWIST_HELPER_RE.search(n)]
    return out or list(lb.fk_controls) + list(lb.ik_controls)


def _ik_plan(rig, lb):
    """给「切到 IK」准备一份修正计划：末端控制器、髋/膝/踝三根变形骨、pole。

    自算快照只能做到「大概对」（这根 rig 上踝差 6 mm、膝差 15 mm）。切完之后
    还要靠这份计划做几何收敛：按踝残差平移末端目标，按膝残差绕 髋→踝 轴把
    pole 转到目标方向。两三次就到 5 mm 以内。
    """
    if len(lb.fk_controls) < 3:
        return None
    pool = _deform_pool(rig)
    chain = []
    for n in lb.fk_controls[:3]:
        d = _deform_of(rig, n, _DEF_STRIP_FK_RE)
        if d is None and pool:
            d = _nearest_deform_head(rig, n, pool)
        chain.append(d)
    if not all(chain):
        return None
    end_ctrl = None
    end_def = chain[2]
    for c in lb.ik_controls:
        low = c.lower()
        if "target" in low or "pole" in low:
            continue
        d = _deform_of(rig, c, _DEF_STRIP_IK_RE)
        if d is None and pool:
            d = _nearest_deform_head(rig, c, pool)
        if d == end_def:
            end_ctrl = c
            break
    if end_ctrl is None:
        return None
    poles = [c for c in lb.ik_controls
             if "target" in c.lower() or "pole" in c.lower()]
    # 没有 pole 目标的 IK 链（RIG-Skins 的腿就是：IK 约束上 pole_subtarget
    # 是空的），膝盖朝向由链根的朝向决定 —— 那就转链根那根控制骨来摆膝盖。
    steer = None
    if not _limb_has_real_pole(rig, lb):
        steer = _limb_steer_bone(rig, lb, chain[0])
    return {'root_def': chain[0], 'mid_def': chain[1], 'end_def': end_def,
            'end_ctrl': end_ctrl, 'poles': poles, 'steer': steer}


def _limb_has_real_pole(rig, lb):
    """这条肢体的 IK 约束上到底挂了 pole 目标没有。"""
    pool = set(lb.ik_controls) | set(lb.fk_controls)
    for pb in rig.pose.bones:
        for c in pb.constraints:
            if c.type == 'IK' and c.pole_subtarget and c.pole_subtarget in pool:
                return True
    return False


def _limb_steer_bone(rig, lb, root_def):
    """没 pole 时用来摆膝盖（手肘）朝向的控制骨：IK 侧的链根，如 `Hip_ik_L`。"""
    for c in lb.ik_controls:
        low = c.lower()
        if "target" in low or "pole" in low or "heel" in low or "spin" in low:
            continue
        try:
            if _deform_of(rig, c, _DEF_STRIP_IK_RE) == root_def:
                return c
        except Exception:
            continue
    return None


def _search_bend(apply_angle):
    """一维搜索弯曲角：先保扭转，再在这一带里挑位置最准的。

    扭转 100° 和位置差 4 cm 视觉上不是一个量级 —— 所以先用「扭转 + 3°」
    圈出一个可接受区间，再在区间内按位置误差挑。返回最终角度（度）。
    """
    samples = []

    def probe(deg):
        rot, pos = apply_angle(deg)
        samples.append((float(deg), rot, pos))
        return rot, pos

    probe(0.0)
    for deg in range(-180, 181, 45):
        probe(float(deg))
    best_rot = min(s[1] for s in samples)
    near = [s for s in samples if s[1] <= best_rot + 3.0]
    best_deg = min(near, key=lambda s: s[2])[0]
    for step in (12.0, 4.0):
        around = best_deg
        cand = []
        for k in (-2, -1, 1, 2):
            deg = around + k * step
            rot, pos = probe(deg)
            cand.append((deg, rot, pos))
        best_rot = min(c[1] for c in cand)
        good = [c for c in cand if c[1] <= best_rot + 3.0]
        best_deg = min(good, key=lambda c: c[2])[0]
    apply_angle(best_deg)
    return best_deg


def _refine_ik(rig, plan, want, want_q=None):
    """几何收敛：踝对点、膝对**方向**（连膝盖的扭转一起对）。

    实测这套 rig：脚踝对好之后，膝盖位置还差 2 cm，而 DEF-shin 的**扭转**
    能差 112° —— 换 pole 绕 髋→踝 轴的角度能把这个扭转从 112° 压到 7°，
    位置几乎不变。所以最后一步是拿 pole 角度做一维搜索，用
    「扭转角 + 150 × 位置误差」当代价，先粗扫 15° 再细扫 5°。
    """
    if not plan:
        return 0
    want_q = want_q or {}
    mid_name = plan.get('mid_def') or ""
    for _ in range(2):
        moved = 0
        ec = rig.pose.bones.get(plan.get('end_ctrl') or "")
        ed = rig.pose.bones.get(plan.get('end_def') or "")
        md = rig.pose.bones.get(mid_name)
        rd = rig.pose.bones.get(plan.get('root_def') or "")
        if ec is not None and ed is not None and want.get(plan.get('end_def')):
            err = want[plan['end_def']] - ed.matrix.translation
            if err.length > 1e-4:
                mm = ec.matrix.copy()
                mm.translation = mm.translation + err
                ec.matrix = mm
                _flush_rig(rig)
                moved += 1
        poles = [pn for pn in (plan.get('poles') or [])
                 if rig.pose.bones.get(pn) is not None]
        steer_pb = rig.pose.bones.get(plan.get('steer') or "")
        if (md is not None and rd is not None and ed is not None
                and mid_name in want_q and mid_name in want
                and (poles or steer_pb is not None)):
            root = rd.matrix.translation.copy()
            goal = ed.matrix.translation.copy()
            axis = goal - root
            if axis.length > 1e-5:
                an = axis.normalized()
                q_want = want_q[mid_name]
                tgt = want[mid_name]

                def measure():
                    _flush_rig(rig)
                    pos_err = (md.matrix.translation - tgt).length
                    d = md.matrix.to_quaternion().rotation_difference(q_want)
                    deg_err = math.degrees(d.angle)
                    return min(deg_err, 360.0 - deg_err), pos_err

                if poles:
                    # 常规路：绕 髋→踝 轴摆 pole
                    base = [rig.pose.bones[pn].matrix.translation.copy() - root
                            for pn in poles]

                    def apply_angle(deg):
                        rot = Matrix.Rotation(math.radians(deg), 4, an)
                        for pn, off in zip(poles, base):
                            pb2 = rig.pose.bones[pn]
                            mm = pb2.matrix.copy()
                            mm.translation = root + (rot @ off)
                            pb2.matrix = mm
                        return measure()
                else:
                    # 兜底路：这条 IK 链没挂 pole —— 绕同一根轴转**链根**控制骨，
                    # 大腿跟着转、IK 把脚踝钉住，膝盖就绕着这根轴挪到该在的地方
                    steer_base = steer_pb.matrix.copy()

                    def apply_angle(deg):
                        rot = Matrix.Rotation(math.radians(deg), 4, an)
                        spin = (Matrix.Translation(root) @ rot
                                @ Matrix.Translation(-root))
                        steer_pb.matrix = spin @ steer_base
                        return measure()

                best_deg = _search_bend(apply_angle)
                if abs(best_deg) > 1.0:
                    moved += 1
        if not moved:
            break
    return 1


def _fk_pairs_and_mats(rig, lb):
    """FK 方向的配对 + 翻转开关前那一瞬间的源矩阵。

    为什么要存矩阵：FK 控制器的父级是 `MCH-upper_arm_parent` / `MCH-hand_fk`
    这种跟着开关变形的机制骨。翻开关的一刻父级会动，控制器被一起带走 ——
    实测手偏 10 cm。所以翻完必须拿「翻转前的世界矩阵」再对一次，迭代到收敛。
    """
    pool = _deform_pool(rig)
    pairs, mats = [], {}
    for c in lb.fk_controls:
        d = _deform_of(rig, c, _DEF_STRIP_FK_RE)
        if d is None and pool:
            d = _nearest_deform_head(rig, c, pool)
        if not d:
            continue
        src = rig.pose.bones.get(d)
        if src is None:
            continue
        pairs.append((c, d))
        mats[d] = src.matrix.copy()
    return pairs, mats


def _apply_saved(rig, pairs, mats):
    n = 0
    for tgt_name, src_name in pairs:
        tgt = rig.pose.bones.get(tgt_name)
        src_b = rig.data.bones.get(src_name)
        M = mats.get(src_name)
        if tgt is None or src_b is None or M is None:
            continue
        tgt.matrix = M @ (src_b.matrix_local.inverted() @ tgt.bone.matrix_local)
        _flush_rig(rig)
        n += 1
    return n


def _refine_fk(rig, plan):
    """翻开关之后按保存的矩阵再对齐 2 轮（父级变了，得跟着收敛）。"""
    if not plan:
        return 0
    pairs, mats = plan.get('fk_pairs') or [], plan.get('fk_mats') or {}
    if not pairs or not mats:
        return 0
    for _ in range(3):
        if not _apply_saved(rig, pairs, mats):
            return 0
        _flush_rig(rig)
    return 1


def _native_call(lb, target):
    """这个方向到底有没有「可用的」原生 snap。

    Dialyn-琉音 的 UI 脚本里，手臂那颗标着 `FK->IK (hand.L)` 的按钮，props 是
    `output_bones = [upper_arm_fk.L, forearm_fk.L, hand_fk.L]`、
    `input_bones = [upper_arm_ik.L, MCH-forearm_ik.L, MCH-upper_arm_ik_target.L]`。
    而 rig_ui 的通用快照是「读 input、写 output」——
    也就是说这颗按钮**写的是 FK 骨**，实际方向是 IK→FK，跟标签正好相反。
    v5.2.1 拿它当 FK→IK 用：一按就把摆好的 FK 姿势抹回 rest（实测手直接飞
    393 mm），随后翻开关，等于把姿势整个丢掉。
    现在按 output_bones 反查方向，方向对不上就当没有原生可用，走自算快照。
    """
    call = lb.rigify_to_fk if target == "FK" else lb.rigify_to_ik
    if call is None:
        return None
    op_id, props = call
    try:
        out = set(_as_names((props or {}).get("output_bones")))
    except Exception:
        out = set()
    if target == "IK" and out and out == set(lb.fk_controls):
        return None
    return call


def _switch_once(rig, lb, target, kind):
    """做一次快照+翻开关。kind = 'native'（rig 自带 snap）/ 'calc'（自算矩阵）。

    返回 (ok, why, plan)。
    """
    if target == "IK":
        plan = _ik_plan(rig, lb)
    else:
        pairs, mats = _fk_pairs_and_mats(rig, lb)
        plan = {'fk_pairs': pairs, 'fk_mats': mats}
    # v5.2.3：这里原来会先调 _snap_pole()「摆正 pole」。在 Rigify 系骨架里那一步
    # 会把真正的 IK 目标骨挪走 22~84 cm，后面读到的全是坏状态。
    # 现在挪 pole 统一放到翻完开关之后（_pole_optimize），改不动就还原。
    if kind == "native":
        call = _native_call(lb, target)
        if call is None:
            return False, "没有可用的原生 snap", plan
        op_id, props = call
        try:
            mod, _, name = op_id.partition(".")
            if mod != "pose" and "." in op_id:
                # 自定义绑定的 snap 按钮不在 pose 空间（sanby_rig.ik_fk_arm 这类）
                fn = getattr(getattr(bpy.ops, mod, None), name, None)
            else:
                fn = getattr(bpy.ops.pose, name, None)
        except Exception:
            fn = None
        if fn is None:
            return False, "snap 没注册", plan
        try:
            ret = fn(**_op_prop_kwargs(op_id, props))
        except Exception as exc:
            return False, str(exc), plan
        if not (isinstance(ret, (set, frozenset)) and 'FINISHED' in ret):
            return False, "snap 返回 %s" % (ret,), plan
        return True, "", plan
    if getattr(lb, "is_finger", False):
        return (True, "", plan) if _finger_calc(rig, lb, target) \
            else (False, "手指快照配不上骨骼", plan)
    pairs = lb.pairs_to_fk if target == "FK" else lb.pairs_to_ik
    done = _snap_by_matrix(rig, pairs) if pairs else 0
    if not done and not _snap_generic(rig, lb, target):
        return False, "自算快照配不上骨骼", plan
    return True, "", plan


def limb_switch(rig, lb, target, hide_other=True):
    """切到 target（'IK' / 'FK'），并**保证模型姿势不跳**。

    v5.1 及以前就是「调一遍 rig 自带 snap → 翻 IK_FK」。两个坑：
      1. `fn(**props)` 不看返回值 —— snap 被 CANCELLED 也照样翻开关，
         约束一换，模型瞬间弹回 rest（用户报的那个跳变）；
      2. Rigify 的 FK→IK 只搬目标点、不动 pole。腿上次用 IK 时 pole 在
         别处，膝盖会当场翻向，实测脚趾能甩 53 cm。
    v5.2 改成「两条路都试一遍、量了再选」：
      · native —— rig 自带 snap（快，但各家的按钮语义未必一致）；
      · calc   —— 自己按变形骨矩阵摆目标点 + 解析摆 pole。
    每条路量一次蒙皮骨的最大**位移**和最大**旋转**（只看位置会漏掉
    「脚踝对上了、脚掌却转了 78°」这种），两项都归一化到各自阈值后取小的
    那条；两条都超（位移 12 cm / 转角 15°）或者干脆没路可走，就
    **整根还原 + 保持原模式**，绝不交出炸姿势。
    """
    cur = limb_mode(rig, lb)
    if cur is None:
        return False, "读不到 %s 的 %s" % (lb.prop_bone, lb.prop_name)
    if cur == target:
        if hide_other:
            ikfk_sync_visibility(rig, lb, target)
        return False, "%s 已经是 %s" % (lb.label, target)

    pb = rig.pose.bones.get(lb.prop_bone)
    if pb is None or lb.prop_name not in pb:
        return False, "读不到 %s 的 %s" % (lb.prop_bone, lb.prop_name)
    want = lb.fk_value if target == "FK" else lb.ik_value
    try:
        orig = float(pb[lb.prop_name])
    except Exception:
        return False, "读不到 %s 的值" % lb.prop_name

    state = _pose_state(rig)
    # v5.2.3：判定只看这条肢体自己的骨头。
    # 原来量全骨 795 根 —— 只要有一根头发骨 / 另一条手臂的骨转了 15.1°，
    # 明明手已经对齐的切换也会被判「切不过去」。Dialyn-琉音 上 14 根肢体
    # 全军覆没就是这么来的（自算位移 24 mm、转角 15°，卡在阈值上）。
    # 全身只留「位移 12 cm」这一道兜底，那是真的炸了才会触发。
    meas = _limb_deform_names(rig, lb)
    skin = {n: rig.pose.bones[n].matrix.translation.copy()
            for n in meas if rig.pose.bones.get(n) is not None}
    skin_q = {n: rig.pose.bones[n].matrix.to_quaternion().copy()
              for n in meas if rig.pose.bones.get(n) is not None}
    glob = _skin_positions(rig)

    has_native = bool(lb.rigify_to_fk or lb.rigify_to_ik)
    if getattr(lb, "is_finger", False):
        # 手指的链骨（f_index.01/.02/.03）没有和变形骨一一对应，
        # 自算只能对到指尖位置，精度不如原生 —— 所以原生在前、兜底在后。
        order = ["native", "calc"] if has_native else ["calc"]
    else:
        # 自算在前：它按「变形骨矩阵」走，结果可验证；反正是两条都量，
        # 自带 snap 更好的话会被选上，顺序不影响结果，只影响默认偏好。
        order = ["calc", "native"] if has_native else ["calc"]

    best = None
    notes = []
    for kind in order:
        _pose_restore(rig, state)
        try:
            pb[lb.prop_name] = orig
        except Exception:
            pass
        _flush_rig(rig)
        ok, why, plan = _switch_once(rig, lb, target, kind)
        if not ok:
            notes.append("%s:%s" % ("自带" if kind == "native" else "自算", why))
            continue
        try:
            pb[lb.prop_name] = want
        except Exception:
            pass
        _flush_rig(rig)
        if target == "IK":
            # 切完先看 pole 能不能真的改善中段，再做一轮几何收敛
            try:
                _pole_optimize(rig, lb, plan, skin)
                _refine_ik(rig, plan, skin, skin_q)
            except Exception:
                pass
        else:
            # FK 方向：父级机制骨跟着开关动过，按翻转前的矩阵再对一次
            try:
                _refine_fk(rig, plan)
            except Exception:
                pass
        drift = _skin_drift(rig, skin)
        ang = _skin_rot_drift(rig, skin_q)
        gdrift = _skin_drift(rig, glob)
        # 位置和旋转都归一化到自己的阈值上再相加 —— 只看位置的话，
        # 「脚踝对上了、整只脚却转过去 78°」会被判成完美切换。
        cost = drift / _POSE_DRIFT_LIMIT + ang / _POSE_ANG_LIMIT
        if drift > _POSE_DRIFT_LIMIT or ang > _POSE_ANG_LIMIT:
            notes.append("%s:位移 %.0f mm / 转 %.0f°" % (
                "自带" if kind == "native" else "自算",
                drift * 1000, math.degrees(ang)))
        if best is None or cost < best[0]:
            best = (cost, drift, ang, kind, _pose_state(rig),
                    float(pb[lb.prop_name]), gdrift)
        if cost <= 1.0:
            break

    # 两档判定：
    #   · cost ≤ 1.0            —— 完全无感，静默；
    #   · 肢体位移 ≤ 6 cm、转 ≤ 30°、全身位移 ≤ 12 cm —— 照切，结果里写明残差。
    #     「切不过去」比「肘偏 2 cm」更糟：用户会卡死在当前模式里出不来。
    #   · 超出上面任何一条 —— 整根还原，绝不交出炸掉的姿势。
    ok_soft = (best is not None
               and best[1] <= _ACCEPT_DRIFT
               and best[2] <= _ACCEPT_ANG
               and best[6] <= _POSE_DRIFT_LIMIT)
    if best is None or (best[0] > 1.0 and not ok_soft):
        _pose_restore(rig, state)
        try:
            pb[lb.prop_name] = orig
        except Exception:
            pass
        _flush_rig(rig)
        why = "；".join(notes) if notes else ("位移 %.0f mm" % (best[1] * 1000)
                                            if best else "没有可用的快照方式")
        return False, "%s 这套姿势切不过去（%s），已保持原模式" % (lb.label, why)

    # 落最优的那条：恢复它的姿势，属性也跟着它的值
    _pose_restore(rig, best[4])
    try:
        pb[lb.prop_name] = best[5]
    except Exception:
        pass
    _flush_rig(rig)
    if hide_other:
        ikfk_sync_visibility(rig, lb, target)
    msg = "%s → %s" % (lb.label, target)
    if best[1] > 0.005:
        msg += "（最大位移 %.0f mm" % (best[1] * 1000)
        if best[2] > math.radians(2.0):
            msg += " / 转 %.1f°" % math.degrees(best[2])
        msg += "）"
    return True, msg


def _counterpart_name(name, target):
    """Wrist_ik_L -> Wrist_fk_L ；c_hand_ik.l -> c_hand_fk.l"""
    if not name:
        return None
    if target == "FK":
        for a, b in (("_ik", "_fk"), ("_IK", "_FK"), ("_Ik", "_Fk")):
            if a in name:
                return name.replace(a, b, 1)
    else:
        for a, b in (("_fk", "_ik"), ("_FK", "_IK"), ("_Fk", "_Ik")):
            if a in name:
                return name.replace(a, b, 1)
    return None


def _selected_bone_names(context, rig):
    names = [pb.name for pb in (context.selected_pose_bones or [])]
    if names:
        return names
    act = rig.data.bones.active
    return [act.name] if act is not None else []


def _limbs_for_bones(limbs, bone_names):
    pool = set(bone_names)
    return [lb for lb in limbs
            if (set(lb.ik_controls) | set(lb.fk_controls)) & pool]


# 「主控制器」候选词，按「离末端由近到远」分档。一档里全没命中才看下一档 ——
# 之前是一张平词表，`Hip_ik_L` 和 `Ankle_ik_L` 同权，谁排前面选谁，
# 于是腿格第一根落在大腿根部而不是脚底。
_PRIMARY_TIERS = (
    ("hand", "wrist", "foot", "ankle", "toe", "fingers"),
    ("knee", "elbow", "head", "neck"),
    ("hip", "thigh", "shoulder", "clavicle", "pelvis", "chest",
     "spine", "torso", "root"),
)
# 同一档里的非主控制骨：pole / 目标点 / 脚跟翻转 / 抓握辅助 / 属性开关
_PRIMARY_BAD_RE = re.compile(
    r"(parent|prop|target|pole|heel|spin|roll|follow|twist|switch"
    r"|license|licence|watermark|属性|水印)", re.I)


def _primary_control(names):
    """一组控制器里挑「主控制器」——手/脚那种末端目标，而不是 pole 或辅助骨。

    自动跳选中时的兜底：手 IK 切成 FK 之后该落到 c_hand_fk 上，落到
    c_arms_pole 或者 c_thigh_fk 上都没法接着转。
    """
    names = [n for n in (names or []) if n]
    if not names:
        return None
    pool = [n for n in names if not _PRIMARY_BAD_RE.search(n)] or names
    for tier in _PRIMARY_TIERS:
        for n in pool:
            low = n.lower()
            if low.startswith("c_"):
                low = low[2:]
            if any(w in low for w in tier):
                return n
    return pool[0]


def _move_selection(rig, old_name, lb, target):
    """切完把选择挪到新的控制器上，接着就能直接转。"""
    newname = _counterpart_name(old_name, target)
    if newname is None or rig.pose.bones.get(newname) is None:
        pool = lb.fk_controls if target == "FK" else lb.ik_controls
        newname = _primary_control(pool)
    if not newname:
        return None
    for pb in rig.pose.bones:
        pb.select = False
    pb = rig.pose.bones[newname]
    pb.select = True
    if pb.bone.hide:
        pb.bone.hide = False
    rig.data.bones.active = pb.bone
    return newname


def _keying_wanted(rig):
    """这次切换后要不要自动插帧（打关键帧）。

    「只在已有动画时插帧」是默认开着的一道闸：纯摆姿势看效果的时候
    不该凭空给骨架建出一个 Action 来。
    """
    if not pref('ikfk_auto_key', True):
        return False
    if not pref('ikfk_key_only_anim', True):
        return True
    ad = getattr(rig, "animation_data", None)
    if ad is not None and ad.action is not None:
        return True
    try:
        return bool(bpy.context.scene.tool_settings.use_keyframe_insert_auto)
    except Exception:
        return False


def _key_limb(rig, lb):
    """把这一套肢体的两侧控制器 + IK/FK 开关属性都钉在当前帧上。

    只钉「切过去那一侧」是不够的：拖时间轴时另一侧没有关键帧，
    会从别处插值回来，姿态当场飞掉。所以两侧一起打。
    """
    frame = bpy.context.scene.frame_current
    names = []
    for n in list(lb.fk_controls) + list(lb.ik_controls):
        if n not in names:
            names.append(n)
    count = 0
    for name in names:
        pb = rig.pose.bones.get(name)
        if pb is None:
            continue
        if pb.rotation_mode == 'QUATERNION':
            rpath = "rotation_quaternion"
        elif pb.rotation_mode == 'AXIS_ANGLE':
            rpath = "rotation_axis_angle"
        else:
            rpath = "rotation_euler"
        for path in ("location", rpath, "scale"):
            try:
                pb.keyframe_insert(path, frame=frame, group=name)
                count += 1
            except Exception:
                pass
    # AniMate 一类：开关本身也得钉，不然拖时间轴时 driver 会照着插值后的值翻
    if lb.prop_bone:
        pb = rig.pose.bones.get(lb.prop_bone)
        if pb is not None and lb.prop_name in pb.keys():
            try:
                pb.keyframe_insert('["%s"]' % lb.prop_name,
                                   frame=frame, group=lb.prop_bone)
                count += 1
            except Exception:
                pass
    return count


class POSE_OT_cn_ikfk_fingers_v45(bpy.types.Operator):
    """展开 / 收起手指那 10 条 IK-FK（面板别再被撐得老长）"""
    bl_idname = "pose.cn_ikfk_fingers_v45"
    bl_label = "手指 IK/FK 展开"
    bl_options = {'REGISTER'}

    @classmethod
    def poll(cls, context):
        return (context.object is not None
                and context.object.type == 'ARMATURE')

    def execute(self, context):
        rig = context.object
        cur = bool(rig.data.get("cn_ikfk_fingers", 1))
        rig.data["cn_ikfk_fingers"] = 0 if cur else 1
        return {'FINISHED'}


class POSE_OT_cn_ikfk_toggle_v45(bpy.types.Operator):
    """选中控制器按一下就切到另一边，并把离开的那套控制器收起来"""
    bl_idname = "pose.cn_ikfk_toggle_v45"
    bl_label = "IK/FK 切换"
    bl_options = {'REGISTER', 'UNDO'}

    target: bpy.props.EnumProperty(
        name="切到",
        items=(('TOGGLE', "切换", "按当前状态切到另一边"),
               ('IK', "切到 IK", "强制切到 IK"),
               ('FK', "切到 FK", "强制切到 FK")),
        default='TOGGLE',
    )
    label: bpy.props.StringProperty()
    all_limbs: bpy.props.BoolProperty(name="全部肢体", default=False)

    @classmethod
    def poll(cls, context):
        return (context.mode == 'POSE'
                and context.object is not None
                and context.object.type == 'ARMATURE')

    def execute(self, context):
        rig = context.object
        kind, limbs = ikfk_limbs(rig)
        if not limbs:
            self.report({'WARNING'},
                        "这套骨架没有 IK/FK：只认 Rigify（有 rig_id）"
                        "和控制器带 ik_fk_switch 的架子")
            return {'CANCELLED'}

        if self.all_limbs:
            picked = list(limbs)
        elif self.label:
            picked = [lb for lb in limbs if lb.label == self.label]
        else:
            sel = _selected_bone_names(context, rig)
            if not sel:
                self.report({'WARNING'}, "先选中 IK 或 FK 控制器再按")
                return {'CANCELLED'}
            picked = _limbs_for_bones(limbs, sel)
            if not picked:
                self.report({'WARNING'},
                            "选中的骨骼不属于任何 IK/FK 控制器（试试点手 IK 或脚 IK）")
                return {'CANCELLED'}

        sel = _selected_bone_names(context, rig)
        moved, last, done = 0, None, []
        msgs = []
        for lb in picked:
            cur = limb_mode(rig, lb)
            tgt = self.target if self.target in ("IK", "FK") else (
                "FK" if cur == "IK" else "IK")
            ok, msg = limb_switch(rig, lb, tgt)
            if ok:
                moved += 1
                last = (lb, tgt)
                done.append(lb)
            elif msg:
                msgs.append(msg)
        if not moved:
            # 别再一律报「已经在目标模式了」—— 真正的原因（已经是 / 切不过去）
            # 必须让用户看见，否则他会以为插件坏了
            if msgs:
                self.report({'WARNING'}, "；".join(msgs[:3]))
            else:
                self.report({'INFO'}, "已经在目标模式了")
            return {'CANCELLED'}

        if last is not None and not self.all_limbs:
            lb, tgt = last
            _move_selection(rig, sel[0] if sel else "", lb, tgt)

        keyed = 0
        if _keying_wanted(rig):
            for lb in done:
                keyed += _key_limb(rig, lb)
        msg = "已切换 %d 个肢体" % moved
        if keyed:
            msg += "，插帧 %d" % keyed
        self.report({'INFO'}, msg)
        return {'FINISHED'}


class POSE_OT_cn_ikfk_popup_v45(bpy.types.Operator):
    """弹出 IK/FK 小面板：每个肢体一行，点一下切换"""
    bl_idname = "pose.cn_ikfk_popup_v45"
    bl_label = "IK / FK"
    bl_options = {'REGISTER'}

    # 面板宽 238：v5.0 用 popup_menu 自绘，宽度由内容撑开，在四视图里
    # 会被拉成一整条横贯屏幕的长条（用户报「面板太大」）。改成定宽弹窗。
    # v5.2.24：238 太窄 —— SAnby 系开关名（Leg_IK_FK.R）在名称列被截成
    # 「Leg_IK...」（用户：前面的文字要显示全）。加宽到 296，名称列
    # 占比 0.55 → 0.78；「FK → IK」按钮文字本身短，不受影响。
    WIDTH = 296

    @classmethod
    def poll(cls, context):
        return (context.mode == 'POSE'
                and context.object is not None
                and context.object.type == 'ARMATURE')

    def invoke(self, context, event):
        wm = context.window_manager
        try:
            return wm.invoke_popup(self, width=self.WIDTH)
        except Exception:
            # 极老版本没有 invoke_popup：退回菜单，至少功能还在
            wm.popup_menu(self.draw_menu, title="IK / FK", icon='ARMATURE_DATA')
            return {'FINISHED'}

    def draw(self, context):
        draw_ikfk_popup_body(self.layout, context)

    def execute(self, context):
        # invoke_popup 自己不会走 execute；留一个空实现，
        # 免得从搜索菜单里调用时报「没有 execute」。
        return {'FINISHED'}

    def draw_menu(self, menu, context):
        draw_ikfk_popup_body(menu.layout, context)


def draw_ikfk_popup_body(layout, context):
    """浮动面板的紧凑版：一行一个肢体，按钮直接写「当前 → 目标」。"""
    rig = pie_rig(context) if "pie_rig" in globals() else context.object
    if rig is None or getattr(rig, "type", None) != 'ARMATURE':
        layout.label(text="先选中一根骨架", icon='INFO')
        return
    kind, limbs = ikfk_limbs(rig)
    if not limbs:
        layout.label(text="这套骨架没有 IK/FK 开关", icon='INFO')
        return

    sel = set(_selected_bone_names(context, rig))
    picked = _limbs_for_bones(limbs, sel)

    head = layout.row(align=True)
    head.label(text="IK / FK", icon='ARMATURE_DATA')
    n_finger = sum(1 for lb in limbs if getattr(lb, "is_finger", False))
    head.label(text="%d" % (len(limbs) - n_finger))

    col = layout.column(align=True)
    for lb in limbs:
        if getattr(lb, "is_finger", False):
            continue
        mode = limb_mode(rig, lb) or "?"
        tgt = "FK" if mode == "IK" else "IK"
        row = col.row(align=True)
        name = row.row(align=True)
        name.scale_x = 0.78
        name.label(text=("● %s" if lb in picked else "%s") % lb.label)
        op = row.operator("pose.cn_ikfk_toggle_v45",
                          text="%s → %s" % (mode, tgt))
        op.label = lb.label
        op.target = tgt

    layout.separator(factor=0.4)
    row = layout.row(align=True)
    op = row.operator("pose.cn_ikfk_toggle_v45", text="全部 → IK")
    op.target = "IK"
    op.all_limbs = True
    op = row.operator("pose.cn_ikfk_toggle_v45", text="全部 → FK")
    op.target = "FK"
    op.all_limbs = True
    if n_finger:
        layout.label(text="手指 IK/FK %d 条：侧栏面板里切" % n_finger,
                     icon='INFO')
    if kind == "rigify":
        layout.label(text="Rigify 原生 snap", icon='CHECKMARK')


TOOLTIP_ZH = {
    "Super charged UI bone collections": (
        "骨骼集合增强操作（默认设置）：\n"
        "    - 左键：选中该集合里的骨骼；隐藏时自动取消隐藏。\n"
        "    - Shift + 左键：把这些骨骼追加到当前选择。\n"
        "    - Ctrl + 左键：单独显示该集合 / 恢复之前的集合状态。\n"
        "    - Ctrl + Shift + 左键：切换集合可见性。\n\n"
        "可在 偏好设置 > 键位映射 里改默认快捷键"
    ),
    "Toggle on/off": "开启 / 关闭",
    "Display help for this section": "查看这一节的帮助",
    "Get the mouse modifiers": "获取鼠标修饰键",
    "Configure this section": "配置这一节",
    "Turn on this mode": "开启该模式",
    "Set the category for the selected action": "为选中动作设置分类",
    "Add a new category": "新建分类",
    "Remove the selected category": "移除选中分类",
    "Move the selected action up or down in the pinned action list": "在固定动作列表里上移 / 下移选中动作",
    "Move the selected category up or down in the list": "在列表里上移 / 下移选中分类",
    "Refresh the action list and update pinned actions": "刷新动作列表并更新固定动作",
    "Display a popup menu to select an action": "弹出菜单选择动作",
    "Set the selected action as the armature's current action or play/pause if it's already active": "把选中动作设为骨架当前动作；已是当前动作时播放 / 暂停",
    "Remove the current action and reset transforms if enabled": "移除当前动作，并按设置重置变换",
    "Remove all users of the selected action": "移除选中动作的所有使用者",
    "Hold Shift to bypass confirmation dialog.\nThis will permanently delete the action from the file": "按住 Shift 跳过确认框。\n这会从文件中永久删除该动作",
    "Toggle pin status for all selected actions": "切换所有选中动作的固定状态",
    "Toggle selection status with a filter": "按过滤条件切换选中状态",
    "Click to select a bone in the 3D Viewport to use as the root motion bone": "在 3D 视口点选一根骨骼作为根运动骨骼",
    "Apply the retimer to all selected actions": "对选中动作应用重定时",
    "Reset toggles to their default values on the first frame and optionally keyframe them": "在第一帧把开关重置为默认值，并可选打关键帧",
    "Switch between IK and FK mode.\n- Ctrl: Force switch to FK mode\n- Shift: Force switch to IK mode": "切换 IK / FK 模式。\n- Ctrl：强制切到 FK\n- Shift：强制切到 IK",
    "Add a new limb definition to the list": "向列表添加一条肢体定义",
    "Remove the selected limb definition from the list": "从列表移除选中肢体定义",
    "Move the selected limb definition up or down in the list": "在列表里上移 / 下移选中肢体定义",
    "Create a new limb definition from the selected one renaming all\n    the elements from L to R or vice versa": "以选中肢体为模板新建一条定义，\n并把所有元素从 L 改名到 R（或反向）",
    "Add Bone Collection to the list": "向列表添加骨骼集合",
    "Add this bone collection to the list": "把这个骨骼集合加入列表",
    "Remove Bone Collection": "移除骨骼集合",
    "Add the active bone to the selected limb definition bone list": "把活动骨骼加入选中肢体的骨骼列表",
    "Remove Bone": "移除骨骼",
    "Move the selected bone up or down in the list": "在列表里上移 / 下移选中骨骼",
    "Remove the selected bone pair from the list": "从列表移除选中骨骼对",
    "Move the selected bone pair up or down in the list": "在列表里上移 / 下移选中骨骼对",
    "Edit pair snapping properties": "编辑骨骼对捕捉属性",
    "Edit extra bones properties": "编辑额外骨骼属性",
    "Assign the selected bone to the specified property or list": "把选中骨骼分配为指定属性 / 加入列表",
    "Assign the selected bone to the specified pair property": "把选中骨骼分配到指定骨骼对属性",
    "Display all custom properties of the selected bone and allow selection": "显示选中骨骼的所有自定义属性并允许选择",
    "Set this custom property as the IK/FK switch property": "把这个自定义属性设为 IK/FK 开关",
    "Bake the IK/FK animation to a range of frames": "把 IK/FK 动画烘焙到指定帧范围",
    "Refresh the list of constraints with 'RIG_UI_PS' in their name.": "刷新名称含 'RIG_UI_PS' 的约束列表",
    "Select the object or bone in the 3D view.": "在 3D 视图里选择对象或骨骼",
    "Move the selected constraint up or down in the list": "在列表里上移 / 下移选中约束",
    "Select the target object or subtarget bone": "选择目标对象或子目标骨骼",
    "Change Active Parent": "切换活动父级",
    "Change the active parent to this object/bone": "把这个对象 / 骨骼设为活动父级",
    "Remove the selected constraint": "移除选中约束",
    "Add a Child Of constraint to the selected element": "给选中元素添加 Child Of 约束",
    "Add an Armature constraint to the selected element": "给选中元素添加 Armature 约束",
    "Remove a bone from the Armature constraint of the selected element": "从选中元素的 Armature 约束里移除骨骼",
    "Add a bone to the Armature constraint of the selected element": "给选中元素的 Armature 约束添加骨骼",
    "Add a parent switch constraint between the selected child and parent": "在选中的子级和父级之间添加父级切换约束",
    "Add a selected object or bone to the extra elements list": "把选中对象 / 骨骼加入额外元素列表",
    "Remove the selected extra element from the list": "从列表移除选中额外元素",
    "Change the active parent to this object/bone and keyframe the switch over the selected\nframe range.": "把这个对象 / 骨骼设为活动父级，并在选中帧范围打切换关键帧",
    "Export a full Rig UI setup to a JSON file. This includes bone collection properties and pose bones.": "把完整 Rig UI 配置导出为 JSON，包含骨骼集合属性和姿态骨骼。",
    "Import a full Rig UI setup from a JSON file. This includes bone collection properties and pose bones.": "从 JSON 导入完整 Rig UI 配置，包含骨骼集合属性和姿态骨骼。",
    "Toggle the visibility of all mask modifiers in this group.\nGroups are defined by elements with the same display name. Their visibility will be\ntoggled together as one": "切换这一组里所有遮罩修改器的可见性。\n同显示名的元素视为一组，一起切换。",
    "Aligns two bones to a pole target, adjusting only the joint and keeping bone lengths": "把两根骨骼对齐到极向目标，只调整关节、保持骨骼长度",
}


# ---------------------------------------------------------------- rigUI 补丁

_RIGUI_TIMER = {"timer": None, "tries": 0, "done": False}
_TOOLTIP_BACKUP = {}


def _all_operator_classes():
    out, seen = [], set()
    stack = list(bpy.types.Operator.__subclasses__())
    while stack:
        cls = stack.pop()
        if cls in seen:
            continue
        seen.add(cls)
        out.append(cls)
        stack.extend(cls.__subclasses__())
    return out


DOCS_ZH = {
    "ik_fk_snapping_limb_definitions": ("IK/FK 肢体定义", (
        "在这一节里可以定义用于 IK/FK 捕捉的肢体。\n\n"
        "肢体列表包含名称和元素。你可以用右侧的 ADD / REMOVE 图标增删肢体，"
        "或拖动排序来在视觉上分组。选中列表里的肢体后，下方会显示它的更多信息。\n\n"
        "肢体定义完整后，可以用 Mirror Limb 按钮镜像。命名规则详见 Limb Name 帮助。"
    )),
    "ik_fk_snapping_limb_name": ("肢体名称", (
        "这是肢体的名称，用于在 UI 和其他属性里识别该肢体。建议保持名称唯一。\n\n"
        "如果想让整个肢体定义能被镜像，请给名称加上 Blender 能识别的左右后缀，"
        "例如 \".L\" 表示左、\".R\" 表示右。\n\n"
        "可以用 \".\"、\"_\" 或 \"-\" 分隔左右和名称，例如 \"Arm.L\"、\"Arm_L\"、\"Arm-L\"；"
        "大小写都有效，只要放在分隔符后面。"
    )),
    "ik_fk_snapping_to_fk": ("到 FK 的骨骼对（FK - IK）", (
        "这一组骨骼对用于把 FK 骨骼捕捉到 IK 骨骼，切换肢体到 FK 模式时使用。\n\n"
        "添加方法：同时选中两根骨骼，先选 FK 骨骼，再选 IK 骨骼，然后点带吸管图标的 Add Bone Pair。\n\n"
        "每对骨骼在磁铁图标里还有额外选项：\n"
        "    - 要捕捉的变换：让第一根骨骼匹配第二根骨骼的位置 / 旋转 / 缩放。\n"
        "    - 保持相对偏移：如果勾选位置捕捉，会保留两根骨骼在静止姿态下的相对偏移。\n"
        "    - 捕捉后选中：切换到 FK 后自动选中第一根骨骼。"
    )),
    "ik_fk_snapping_to_ik": ("到 IK 的骨骼对（IK - FK）", (
        "这一组骨骼对用于把 IK 骨骼捕捉到 FK 骨骼，切换肢体到 IK 模式时使用。\n"
        "第一根骨骼会匹配第二根骨骼选定的变换。\n\n"
        "添加方法：同时选中两根骨骼，先选 IK 骨骼，再选 FK 骨骼，然后点带吸管图标的 Add Bone Pair。\n\n"
        "每对骨骼在磁铁图标里还有额外选项：\n"
        "    - 要捕捉的变换：让第一根骨骼匹配第二根骨骼的位置 / 旋转 / 缩放。\n"
        "    - 保持相对偏移：如果勾选位置捕捉，会保留两根骨骼在静止姿态下的相对偏移。\n"
        "    - 捕捉后选中：切换到 IK 后自动选中第一根骨骼。"
    )),
    "ik_fk_snapping_custom_prop": ("IK/FK 自定义属性", (
        "这一节定义用来切换 IK/FK 的自定义属性和它的值。\n"
        "    - 自定义属性骨骼：拥有该属性的骨骼。\n"
        "    - 属性：用于切换 IK/FK 的属性名。\n"
        "    - FK 值：切到 FK 时该属性应该取的值。\n\n"
        "可以用吸管把选中骨骼加进自定义属性；"
        "也可以点属性旁边的 ADD 按钮，从该骨骼已有的自定义属性里选择。"
    )),
    "bc_modifier_keys": ("骨骼集合快捷键", (
        "修饰键可以扩展骨骼集合按钮的操作方式，"
        "用 Shift / Ctrl / Alt 配合鼠标点击执行不同动作。\n\n"
        "默认操作：\n"
        "- 左键：选中该集合里的骨骼；隐藏时自动取消隐藏。\n"
        "- Shift + 左键：把这些骨骼追加到当前选择。\n"
        "- Ctrl + 左键：切换集合可见性。\n"
        "- Ctrl + Shift + 左键：单独显示集合 / 恢复之前的集合状态。\n\n"
        "可以在 Rig UI 偏好设置 > 键位映射 里自定义这些修饰键。"
    )),
}


def apply_rigui_docs():
    """把 rigUI 的帮助弹窗正文换成中文（运行时替换 docs 字典，不改源码）。"""
    changed = 0
    for name, mod in list(sys.modules.items()):
        if not name.endswith("amp_rig_ui.documentation"):
            continue
        docs = getattr(mod, "docs", None)
        if not isinstance(docs, dict):
            continue
        for key, (header, body) in DOCS_ZH.items():
            item = docs.get(key)
            if isinstance(item, dict):
                item["header"] = header
                item["body"] = body
                changed += 1
    return changed


def apply_rigui_tooltips():
    """把 rigUI 面板上可见的英文 tooltip 换成中文（只改显示，不改逻辑）。"""
    changed = 0
    for cls in _all_operator_classes():
        desc = getattr(cls, "bl_description", "")
        if not desc or not isinstance(desc, str):
            continue
        hit = None
        for key, zh in TOOLTIP_ZH.items():
            if key in desc:
                hit = zh
                break
        if hit is not None and desc != hit:
            try:
                cls.bl_description = hit
                changed += 1
            except Exception:
                pass
    return changed


def unpatch_rigui_popup():
    cls = _POPUP_PATCH.get("cls")
    orig = _POPUP_PATCH.get("orig")
    if cls is not None and orig is not None:
        try:
            cls.draw = orig
            if hasattr(cls, "_cn_wrapped"):
                delattr(cls, "_cn_wrapped")
        except Exception:
            pass
    _POPUP_PATCH.clear()


def restore_rigui_tooltips():
    for cls, desc in list(_TOOLTIP_BACKUP.items()):
        try:
            cls.bl_description = desc
        except Exception:
            pass
    _TOOLTIP_BACKUP.clear()


def _rigui_patch_timer():
    st = _RIGUI_TIMER
    st["tries"] += 1
    apply_rigui_tooltips()
    apply_rigui_docs()
    # 5.2 fix: 不再 monkey-patch rigUI 的 Panel.draw。
    # 运行时替换别人已注册面板的 draw，会在 popover 重画时触发
    # EXCEPTION_ACCESS_VIOLATION（panel_draw -> bpy_class_call）。
    # 导航 UI 继续用它自己的独立面板，不再注入 rigUI 的弹窗/侧栏。
    if st["tries"] >= 40:
        st["timer"] = None
        return None
    return 0.5


# ---------------------------------------------------------------- 面板

class VIEW3D_PT_controller_nav_v45(bpy.types.Panel):
    bl_label = "驭骨 · BonePilot"
    bl_idname = "VIEW3D_PT_controller_nav_v45"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = '驭骨'

    @classmethod
    def poll(cls, context):
        return (context.mode == 'POSE'
                and context.object is not None
                and context.object.type == 'ARMATURE')

    def draw(self, context):
        layout = self.layout
        rig = context.object
        cur = rig.data.bones.active
        prefs = get_prefs()

        # 折叠
        if prefs is not None:
            hrow = layout.row(align=True)
            hrow.prop(prefs, "panel_open", text="", emboss=False,
                      icon='TRIA_DOWN' if prefs.panel_open else 'TRIA_RIGHT')
            hrow.label(text="控制器导航")
            hrow.operator("preferences.addon_show", text="", icon='PREFERENCES').module = MODULE_NAME
            if not prefs.panel_open:
                return

        # 常用设置（直接读写偏好）
        if prefs is not None:
            pbox = layout.box()
            pbox.label(text="快速设置", icon='PREFERENCES')
            pbox.prop(prefs, "rot_sensitivity", text="旋转灵敏度")
            pbox.prop(prefs, "ring_zone", text="旋转活动半径")
            pbox.prop(prefs, "finger_single", text="手指默认单轴")
            if prefs.finger_single:
                pbox.prop(prefs, "finger_axis", text="手指单轴")
            # 跳集合时的取景留白：原生框选是贴边的，集合越小越要放大一点
            pbox.prop(prefs, "frame_margin", text="集合取景余量", slider=True)
            pbox.prop(prefs, "frame_min_dist", text="最小视距")

        if cur is not None:
            box = layout.box()
            sst_top = _rig_spatial_settings(rig)
            if sst_top is not None:
                row = box.row(align=True)
                row.prop(sst_top, "refine_mode", text="细化阶段")
                if sst_top.refine_mode:
                    row.label(text="轴 " + str(sst_top.refine_axis))
            box.label(text=cur.name, icon='BONE_DATA')
            items = nav_list(rig, cur.name)
            pos = "%d / %d" % (items.index(cur.name) + 1, len(items)) if cur.name in items else "-"
            box.label(text="%s    %s" % (scope_label(rig, cur.name), pos))

            cur_col = nav_collection(rig, cur)
            if cur_col is not None:
                crow = box.row(align=True)
                crow.label(text=cur_col.name, icon='COLLECTION_COLOR_01')
                solo_now = cur_col.name in solo_names(rig)
                sc = crow.operator(
                    "pose.cn_solo_collection_v45", text="独显" if not solo_now else "还原",
                    icon=safe_icon('SOLO_ON') if solo_now else safe_icon('SOLO_OFF'),
                    depress=solo_now)
                sc.collection = cur_col.name
                row = box.row(align=True)
                row.operator_menu_enum("pose.cn_set_collection_motion_v45", "mode",
                                       text="动作：" + dict((k, v) for k, v, _ in MOTION_MODE_ITEMS).get(
                                           collection_motion_mode(rig, cur_col), "可转可移"),
                                       icon='TRANSFORM_ORIGINS')
                row.operator_menu_enum("pose.cn_set_collection_sort_v45", "mode",
                                       text="排序：" + collection_sort_label(rig, cur_col),
                                       icon='SORTALPHA')
                # 集合的「默认第一根」：切进集合直接落在它上面
                dflt = default_first_bone(rig, cur_col.name)
                row = box.row(align=True)
                row.label(text="默认骨骼：%s" % (dflt or "（排序第一根）"),
                          icon='OBJECT_ORIGIN')
                if dflt:
                    row.operator("pose.cn_first_bone_v45", text="", icon='X').clear = True
                opd = row.operator("pose.cn_first_bone_v45",
                                   text="设为当前选中" if dflt else "选中骨骼设为默认")
                opd.clear = False

                # ── 集合里的骨骼顺序（v5.2.6：只留点选排序）──────────────
                # 点一下开始 → 视图里一根一根加选 → 点回来写入。
                # 排第一的那根就是进集合先选中的那根，↑↓ 也从它开始走。
                col_bones = collection_bone_names(rig, cur_col)
                custom = bone_order_is_custom(rig, cur_col.name)
                bopen = bool(getattr(prefs, "bone_order_open", False)) if prefs is not None else False
                bh = box.row(align=True)
                if prefs is not None:
                    bh.prop(prefs, "bone_order_open", text="", emboss=False,
                            icon='TRIA_DOWN' if bopen else 'TRIA_RIGHT')
                bh.label(text="骨骼顺序（%d 根%s）" % (
                    len(col_bones), "，已自定义" if custom else ""),
                    icon=safe_icon('SORTALPHA'))
                if bopen:
                    if collection_sort_mode(rig, cur_col) != 'ASSIGN':
                        box.label(text="当前排序是「%s」—— 排一次会自动切回集合顺序"
                                  % collection_sort_label(rig, cur_col), icon='INFO')
                    st = _PICK_ORDER
                    armed = (st["armed"] and st["rig"] == rig.name
                             and st["col"] == cur_col.name)
                    r1 = box.row(align=True)
                    r1.scale_y = 1.35
                    if armed:
                        txt = ("✓ 写入顺序（已点 %d 根）" % len(st["queue"])
                               if st["queue"] else "没点骨骼 · 再点一下取消")
                    else:
                        txt = "点选排序：点这里开始 → 加选 → 点回来写入"
                    pk = r1.operator("pose.cn_bone_order_pick_v45", text=txt,
                                     icon=safe_icon('CURSOR'), depress=armed)
                    pk.collection = cur_col.name
                    if armed:
                        if st["queue"]:
                            prev = " → ".join(st["queue"][:8])
                            if len(st["queue"]) > 8:
                                prev += " …"
                            box.label(text="已记录：%s" % prev, icon='BLANK1')
                        else:
                            box.label(text="现在一根一根点骨骼（加选）：谁被点谁排后面",
                                      icon='BLANK1')
                    # 长集合（头发/手指动辄几十根）只列前 24 根
                    shown = col_bones[:24]
                    for k, nm in enumerate(shown):
                        r = box.row(align=True)
                        r.label(text="%d. %s" % (k + 1, nm),
                                icon='BONE_DATA' if nm == cur.name else 'BLANK1')
                        up = r.operator("pose.cn_bone_order_v45", text="",
                                        icon=safe_icon('TRIA_UP'))
                        up.collection, up.bone, up.delta = cur_col.name, nm, -1
                        dn = r.operator("pose.cn_bone_order_v45", text="",
                                        icon=safe_icon('TRIA_DOWN'))
                        dn.collection, dn.bone, dn.delta = cur_col.name, nm, 1
                        rm = r.operator("pose.cn_bone_order_v45", text="", icon='X')
                        rm.collection, rm.bone, rm.remove = cur_col.name, nm, True
                    if len(col_bones) > len(shown):
                        box.label(text="……还有 %d 根（列表只显示前 24 根，用点选排序整段排）"
                                  % (len(col_bones) - len(shown)), icon='BLANK1')
                    if custom:
                        rs = box.row(align=True).operator(
                            "pose.cn_bone_order_v45", text="恢复 assign 原顺序",
                            icon=safe_icon('LOOP_BACK'))
                        rs.collection, rs.reset = cur_col.name, True
                    box.label(text="顺序只影响滚轮 / ↑↓ 的先后，不改集合成员", icon='BLANK1')

        # ---- 骨骼操作：选中骨骼的动作模式 + 轴锁，紧贴上面的骨骼信息
        # 这三块是同一件事（这根骨骼能怎么动），v4.9.89 起连在一起，
        # 中间不再夹「观察视角」这种看画面的功能。
        if cur is not None:
            abox = layout.box()
            abox.label(text="选中骨骼动作", icon='BONE_DATA')
            abox.operator_menu_enum(
                "pose.cn_set_bone_motion_v45", "mode",
                text="当前：" + dict((k, v) for k, v, _ in BONE_MOTION_ITEMS).get(
                    bone_motion_mode(cur), "跟随集合"),
                icon='DOWNARROW_HLT')

        # 手动锁轴
        lbox = layout.box()
        lbox.label(text="手动锁轴", icon='LOCKED')
        lr1 = lbox.row(align=True)
        lr1.operator("pose.cn_lock_axes_v45", text="只留 X").mode = 'ONLY_X'
        lr1.operator("pose.cn_lock_axes_v45", text="只留 Y").mode = 'ONLY_Y'
        lr1.operator("pose.cn_lock_axes_v45", text="只留 Z").mode = 'ONLY_Z'
        lr2 = lbox.row(align=True)
        lr2.operator("pose.cn_lock_axes_v45", text="X + Z").mode = 'XZ'
        lr2.operator("pose.cn_lock_axes_v45", text="全锁").mode = 'ALL'
        lr2.operator("pose.cn_lock_axes_v45", text="全解锁").mode = 'NONE'

        # 饼菜单：三层（主饼/细节饼/附属饼），各 8 格，点名字换集合
        # 每块都能折叠 —— 主饼配好之后基本不用再看，收起来省一大截滚动
        # 一键配置（v5.1）：直接读骨架原生的集合，自动归类进三层饼。
        # 新角色 / 新绑定打开就能点，不用先去偏好里配一套名字。
        abox = layout.box()
        arow = abox.row(align=True)
        arow.scale_y = 1.45
        arow.operator("pose.cn_auto_layout_v45",
                      text="一键配置饼菜单", icon='AUTO')
        cnt = [sum(1 for g in pie_slot_groups(rig, ly) if g) for ly in (1, 2, 3)]
        abox.label(text="主饼 %d 格 · 细节 %d 格 · 附属 %d 格 ／ 原生集合 %d 个"
                   % (cnt[0], cnt[1], cnt[2], len(_colls(rig))),
                   icon='COLLECTION_COLOR_01')
        if prefs is not None:
            # 星标饼：默认开着（用户实测比原生饼顺手）；关了就回原生饼，键位一样
            abox.prop(prefs, "pie_custom", text="饼用星标饼（自绘 · 每格带 ★）")
        snames = solo_names(rig)
        if snames:
            if len(snames) == 1:
                stxt = snames[0]
            elif len(snames) <= 3:
                stxt = " + ".join(snames)
            else:
                stxt = "%s 等 %d 个" % (" + ".join(snames[:2]), len(snames))
            srow = abox.row(align=True)
            srow.label(text="正在独显：%s" % stxt, icon='SOLO_ON')
            srow.operator("pose.cn_solo_collection_v45", text="还原", icon='LOOP_BACK').restore = True
        # 饼间互斥会改集合的显示状态（Blender 没有临时的集合隐藏），
        # 那是会写进 .blend 的持久数据 —— 给两个按钮兜住这件事。
        vrow = abox.row(align=True)
        vrow.scale_y = 0.85
        vrow.operator("pose.cn_restore_visibility_v45",
                      text="还原原始显示", icon='LOOP_BACK')
        vrow.operator("pose.cn_snapshot_visibility_v45",
                      text="以当前为准", icon='FILE_TICK')

        cur_col_pie = pie_current_collection(rig)
        for layer, lname, lhint, licon in PIE_LAYERS:
            groups = pie_slot_groups(rig, layer)
            pbox = layout.box()
            head = pbox.row(align=True)
            flag = "pie_open_%d" % layer
            opened = bool(getattr(prefs, flag, True)) if prefs is not None else True
            if prefs is not None:
                head.prop(prefs, flag, text="", emboss=False,
                          icon='TRIA_DOWN' if opened else 'TRIA_RIGHT')
            head.label(text="饼菜单 · %s（%s）" % (lname, lhint),
                       icon=safe_icon(licon))
            if any(groups):
                opf = head.operator("pose.cn_pie_autofill_v45",
                                    text="", icon='FILE_REFRESH')
                opf.layer = layer
                opc = head.operator("pose.cn_pie_clear_v45", text="", icon='X')
                opc.layer = layer
            elif opened:
                # 8 格全空（刚装好，或者你手动清空过）：把自动填充做成
                # 显眼的红条，别让人对着一排空格子发懵
                hrow2 = pbox.row(align=True)
                hrow2.alert = True
                opf = hrow2.operator("pose.cn_pie_autofill_v45",
                                     text="8 格是空的 · 点这里自动填充",
                                     icon='FILE_REFRESH')
                opf.layer = layer
            if not opened:
                # 收起来的层给一行摘要：一眼看得出这层配了几个、都放了谁
                names = [g[0] for g in groups if g]
                if names:
                    pbox.label(text="  ".join(names[:4]) +
                               ("  …" if len(names) > 4 else ""),
                               icon='DOT')
                continue
            for i, grp in enumerate(groups):
                row = pbox.row(align=True)
                row.scale_y = 0.85
                sub = row.row(align=True)
                sub.ui_units_x = 3.2
                name = grp[0] if grp else ""
                act = bool(name) and name == cur_col_pie
                sub.label(text=PIE_DIRS[i], icon='LAYER_ACTIVE' if act else 'BLANK1')
                # 一格挂多个集合时用「+N」收尾，别把整行撑爆
                label = name
                if len(grp) > 1:
                    label = "%s  +%d" % (name, len(grp) - 1)
                # 和饼里点空格子走同一个入口：点一下直接弹出集合列表
                opx = row.operator("pose.cn_pie_assign_v45",
                                   text=label or "（空）",
                                   icon='BONE_DATA' if name else 'ADD',
                                   depress=act)
                opx.slot = i
                opx.layer = layer
                # v5.2.16：显式清掉，防算子属性残留把弹窗跳过去
                opx.collection = ""
                opx.from_popup = False
                opx.remove = False
                opx.append = False
                # 星标：点一下只显示这个集合（再点一次还原）
                if name:
                    solo_now = name in solo_names(rig)
                    st = row.operator(
                        "pose.cn_solo_collection_v45", text="",
                        icon=safe_icon('SOLO_ON') if solo_now else safe_icon('SOLO_OFF'),
                        depress=solo_now)
                    st.collection = name

    def draw_presets(self, layout, context, rig):
        names = preset_names()
        bones = selected_bones(rig)
        n_sel = len(bones)

        pbox = layout.box()
        header = pbox.row(align=True)
        header.label(text="预设集合", icon='BONE_DATA')
        if names:
            header.label(text="%d 个" % len(names))

        if not names:
            pbox.label(text="还没配置预设集合", icon='INFO')
            pbox.operator("preferences.addon_show", text="去偏好里配置…",
                          icon='PREFERENCES').module = MODULE_NAME
            return

        if n_sel:
            pbox.label(text="已选 %d 个控制器 — 点集合名即可移入" % n_sel)
        else:
            pbox.label(text="先选中控制器，再点集合名")

        grid = pbox.column(align=True)
        arm = rig.data
        for name in names:
            col = _coll_get(arm, name)
            if col is None:
                text, icon = "%s   ＋新建" % name, 'ADD'
            else:
                text, icon = "%s   %d 个" % (name, len(col.bones)), 'BONE_DATA'
            row = grid.row(align=True)
            row.enabled = n_sel > 0
            op = row.operator("pose.cn_assign_preset_v45", text=text, icon=icon)
            op.collection = name

        row = pbox.row(align=True)
        row.operator("pose.cn_create_presets_v45", text="创建全部预设", icon='FILE_REFRESH')
        row.operator("pose.cn_preset_menu_v45", text="菜单 M", icon='MENU_PANEL')


# ---------------------------------------------------------------- 偏好里的预设编辑

class CN_OT_preset_add_v45(bpy.types.Operator):
    bl_idname = "cn.preset_add_v45"
    bl_label = "添加预设集合"
    bl_options = {'REGISTER'}

    @classmethod
    def poll(cls, context):
        return get_prefs() is not None

    def execute(self, context):
        p = get_prefs()
        used = {it.col_name for it in p.presets}
        base, i = "新集合", 1
        name = base
        while name in used:
            i += 1
            name = "%s.%03d" % (base, i)
        p.presets.add().col_name = name
        return {'FINISHED'}


class CN_OT_preset_remove_v45(bpy.types.Operator):
    bl_idname = "cn.preset_remove_v45"
    bl_label = "删除预设集合"
    bl_options = {'REGISTER'}

    index: bpy.props.IntProperty()

    @classmethod
    def poll(cls, context):
        return get_prefs() is not None

    def execute(self, context):
        p = get_prefs()
        if 0 <= self.index < len(p.presets):
            p.presets.remove(self.index)
        return {'FINISHED'}


class CN_OT_preset_move_v45(bpy.types.Operator):
    bl_idname = "cn.preset_move_v45"
    bl_label = "调整预设顺序"
    bl_options = {'REGISTER'}

    index: bpy.props.IntProperty()
    delta: bpy.props.IntProperty(default=-1)

    @classmethod
    def poll(cls, context):
        return get_prefs() is not None

    def execute(self, context):
        p = get_prefs()
        i = self.index
        j = i + self.delta
        if not (0 <= i < len(p.presets)) or not (0 <= j < len(p.presets)):
            return {'CANCELLED'}
        p.presets.move(i, j)
        return {'FINISHED'}


class CN_OT_preset_reset_v45(bpy.types.Operator):
    bl_idname = "cn.preset_reset_v45"
    bl_label = "恢复默认预设"
    bl_options = {'REGISTER'}

    @classmethod
    def poll(cls, context):
        return get_prefs() is not None

    def execute(self, context):
        p = get_prefs()
        while len(p.presets):
            p.presets.remove(0)
        for name in DEFAULT_PRESETS:
            p.presets.add().col_name = name
        return {'FINISHED'}


# ---------------------------------------------------------------- 偏好

# ---------------------------------------------------------------- 键位录入

class POSE_OT_cn_key_capture_v45(bpy.types.Operator):
    """点一下，然后按下你要用的键（鼠标中键 / 侧键 / 滚轮方向都行）"""
    bl_idname = "pose.cn_key_capture_v45"
    bl_label = "按键录入"
    bl_options = {'INTERNAL', 'REGISTER'}

    action: bpy.props.StringProperty()

    def invoke(self, context, event):
        self._armed = False
        context.window_manager.modal_handler_add(self)
        try:
            context.area.tag_redraw()
        except Exception:
            pass
        return {'RUNNING_MODAL'}

    def modal(self, context, event):
        et = event.type
        if et in {'MOUSE_MOVE', 'INBETWEEN_MOUSEMOVE', 'TIMER', 'TIMER_REPORT',
                  'WINDOW_DEACTIVATE', 'NOTHING'}:
            return {'RUNNING_MODAL'}
        if et == 'ESC':
            return {'CANCELLED'}
        if event.value != 'PRESS':
            return {'RUNNING_MODAL'}
        if et in {'LEFT_SHIFT', 'RIGHT_SHIFT', 'LEFT_CTRL', 'RIGHT_CTRL',
                  'LEFT_ALT', 'RIGHT_ALT', 'OSKEY'}:
            return {'RUNNING_MODAL'}
        # 点按钮本身那一下不算，得再按一次
        if not self._armed and et in {'LEFTMOUSE', 'RIGHTMOUSE'} \
                and not (event.ctrl or event.shift or event.alt):
            self._armed = True
            return {'RUNNING_MODAL'}
        it = kb_item(self.action)
        if it is None:
            return {'CANCELLED'}
        it.key = et
        it.ctrl = bool(event.ctrl)
        it.shift = bool(event.shift)
        it.alt = bool(event.alt)
        refresh_keymap()
        self.report({'INFO'}, "%s → %s"
                    % (KB_LABELS.get(self.action, self.action), kb_label(self.action)))
        return {'FINISHED'}


class POSE_OT_cn_key_clear_v45(bpy.types.Operator):
    """清空这个动作的键（不绑定）"""
    bl_idname = "pose.cn_key_clear_v45"
    bl_label = "清除"
    bl_options = {'INTERNAL', 'REGISTER'}

    action: bpy.props.StringProperty()

    def execute(self, context):
        it = kb_item(self.action)
        if it is not None:
            it.key = ""
            it.ctrl = it.shift = it.alt = False
            refresh_keymap()
        return {'FINISHED'}


class POSE_OT_cn_key_reset_v45(bpy.types.Operator):
    """把全部键位恢复成默认（会丢掉你自己设的）"""
    bl_idname = "pose.cn_key_reset_v45"
    bl_label = "全部恢复默认"
    bl_options = {'INTERNAL', 'REGISTER'}

    def execute(self, context):
        p = get_prefs()
        if p is None:
            return {'CANCELLED'}
        p.keybinds.clear()
        ensure_keybinds()
        refresh_keymap()
        self.report({'INFO'}, "键位已恢复默认")
        return {'FINISHED'}


def _kb_bootstrap():
    """register 之后 prefs 才存在，所以延迟一下再补默认键位。"""
    try:
        if ensure_keybinds():
            refresh_keymap()
    except Exception:
        pass
    # v5.2.35：老偏好的 mpath_refresh 存的还是旧默认 True，一次性迁到新默认 False。
    # 只迁一次，之后用户手动开回来不会被再关掉。
    try:
        p = get_prefs()
        if p is not None and not p.mpath_refresh_migrated:
            p.mpath_refresh = False
            p.mpath_refresh_migrated = True
    except Exception:
        pass
    # v5.2.37：v5.2.35 那次「默认关闭」是判断错了 —— 用户要的是「刷新已有路径，
    # 但别给没路径的骨新增」，不是「干脆不刷新」。这里一次性把被那条迁移关掉的
    # 偏好放回开启（之后再手动关就不管了）；刷新本身已改成异步 + 防抖，不卡手。
    try:
        p = get_prefs()
        if p is not None and not p.mpath_refresh_v37:
            p.mpath_refresh = True
            p.mpath_refresh_v37 = True
    except Exception:
        pass
    return None


class ControllerNavPrefs_v45(bpy.types.AddonPreferences):
    bl_idname = MODULE_NAME

    # ---- 预设集合
    presets: bpy.props.CollectionProperty(type=CN_Preset_v45)
    presets_initialized: bpy.props.BoolProperty(default=False, options={'HIDDEN'})

    # ---- 自定义键位（v5.0）：每个动作一条，key 存 Blender 的 event.type
    keybinds: bpy.props.CollectionProperty(type=CN_KeyBind_v45)
    kb_ready: bpy.props.BoolProperty(default=False, options={'HIDDEN'})

    # ---- 模态内滚轮修饰键（旋转 / 移动模式里）
    mod_zoom_mods: bpy.props.EnumProperty(
        name="滚轮缩放视图", items=MOD_COMBO_ITEMS, default='SHIFT',
        description="旋转/移动模式里按住它推滚轮 = 缩放视图",
    )
    mod_bone_mods: bpy.props.EnumProperty(
        name="滚轮切骨骼", items=MOD_COMBO_ITEMS, default='ALT',
        description="旋转/移动模式里按住它推滚轮 = 换下一根控制器",
    )
    mod_coll_mods: bpy.props.EnumProperty(
        name="滚轮切集合", items=MOD_COMBO_ITEMS, default='CTRL_ALT',
        description="旋转/移动模式里按住它推滚轮 = 换下一个集合",
    )
    mod_axis_mods: bpy.props.EnumProperty(
        name="滚轮切轴向", items=MOD_COMBO_ITEMS, default='BARE',
        description="推滚轮换轴（自由 → X → Y → Z）。默认「裸滚轮」，"
                    "也可以改成 Shift / Alt 之类，或者选「不绑定」把滚轮让出去",
    )

    move_bones: bpy.props.BoolProperty(
        name="归组时从其他集合移出（移动而非复制）",
        default=True,
        description="关掉则只添加、保留控制器原来的集合",
    )

    confirm_cross_move: bpy.props.BoolProperty(
        name="从别的预设集合移走前先确认",
        default=True,
        description="归组时如果选中的骨骼还属于另一个预设集合（例如左臂 FK 还选着），"
                    "先弹窗列出会被移走的骨骼，确认后才动手",
    )

    use_m_key: bpy.props.BoolProperty(
        name="在 Pose Mode 用 M 打开预设菜单",
        default=True,
        description="占用 Pose 模式下的 M 键；关掉后只能从面板或快捷键之外的方式调用",
        update=lambda self, context: refresh_keymap(),
    )

    nav_keys: bpy.props.EnumProperty(
        name="导航快捷键",
        items=NAV_KEY_ITEMS,
        default='ALT_WHEEL',
        description="改完立即生效；加选永远是再按住 Shift",
        update=lambda self, context: refresh_keymap(),
    )

    pie_open_1: bpy.props.BoolProperty(name="主饼展开", default=True)
    pie_open_2: bpy.props.BoolProperty(name="细节饼展开", default=True)
    pie_open_3: bpy.props.BoolProperty(name="附属饼展开", default=True)

    bone_order_open: bpy.props.BoolProperty(
        name="展开骨骼顺序",
        default=False,
        description="侧栏面板里「骨骼顺序」那一块默认收起来（长集合展开很长），"
                    "点小箭头展开：点选排序 + 一根一根上下挪。顺序只作用于滚轮/↑↓ 的导航先后",
    )

    pie_custom: bpy.props.BoolProperty(
        name="星标饼（自绘）",
        default=True,
        description="三个饼用自绘版本：每格外沿一个 ★，滑 ★ 独显该集合，滑格子进入集合。"
                    "画不出来会自动退回原生饼",
        update=lambda self, context: refresh_keymap(),
    )

    # ---- 星标饼形态（v5.2.7：响应速度 / 大小都能自己调）
    pie_dead_zone: bpy.props.IntProperty(
        name="触发死区",
        default=12, min=0, max=120,
        subtype='PIXEL',
        description="鼠标离饼心超过这个距离就按方向选中格子 —— 原生饼十几像素就高亮。"
                    "想更跟手就调小（0 = 一动就选），容易误选就调大",
    )

    pie_radius: bpy.props.IntProperty(
        name="饼半径",
        default=168, min=100, max=320,
        subtype='PIXEL',
        description="格子外圈半径（饼有多大）",
    )

    pie_hole: bpy.props.IntProperty(
        name="中心孔",
        default=66, min=8, max=200,
        subtype='PIXEL',
        description="中间空心圆半径。只影响外观，选中判定只看「触发死区」",
    )

    pie_star_size: bpy.props.IntProperty(
        name="星标大小",
        default=9, min=3, max=20,
        subtype='PIXEL',
        description="每格外沿星标的半径",
    )

    pie_label_size: bpy.props.IntProperty(
        name="格子文字",
        default=13, min=8, max=26,
        description="格子名字号",
    )

    collection_keys: bpy.props.EnumProperty(
        name="集合切换键",
        items=COLL_KEY_ITEMS,
        default=COLL_KEY_DEFAULT,
        description="在预设集合之间循环；和导航键冲突时不会绑定",
        update=lambda self, context: refresh_keymap(),
    )

    use_x_rotate: bpy.props.BoolProperty(
        name="绑定旋转进入键",
        default=True,
        description="Pose 模式按下面的键进入旋转：鼠标直接转，跟拽 gizmo 一个手感；"
                    "手势按骨骼的旋转轴锁自动分级（全开=自由，两轴=水平/垂直分通道）",
        update=lambda self, context: refresh_keymap(),
    )

    rot_key: bpy.props.EnumProperty(
        name="旋转进入键",
        items=KEY_ITEMS,
        default='X',
        description="进旋转模态的键（Pose 模式）。改完立即生效，不用去 Keymap 里自己绑",
        update=lambda self, context: refresh_keymap(),
    )

    move_key: bpy.props.EnumProperty(
        name="移动进入键",
        items=KEY_ITEMS,
        default='Z',
        description="进移动模态的键（Pose 模式）。改完立即生效；"
                    "模态里滚轮切 自由→X→Y→Z；旋转中按这个键直接切移动",
        update=lambda self, context: refresh_keymap(),
    )

    move_key_mod: bpy.props.EnumProperty(
        name="移动键修饰",
        items=KEY_MOD_ITEMS,
        default='NONE',
        description="给移动进入键加修饰键，避开 Blender 原生快捷键",
        update=lambda self, context: refresh_keymap(),
    )

    # ---- 面板折叠

    panel_open: bpy.props.BoolProperty(
        name="展开侧栏面板",
        default=True,
        description="侧栏面板顶部的小箭头：一键把整个面板收起来，只留标题行",
    )

    # ---- 手指默认单轴
    finger_single: bpy.props.BoolProperty(
        name="手指默认单轴",
        default=True,
        description="切到手指骨骼时自动锁成单轴旋转，而不是自由/全轴",
    )
    finger_axis: bpy.props.EnumProperty(
        name="手指单轴",
        items=FINGER_AXIS_ITEMS,
        default='X',
        description="切换到手指骨骼时默认使用的本地轴",
    )

    # ---- 自由旋转（v4.8）

    snap_deg: bpy.props.EnumProperty(
        name="Ctrl 吸附角度",
        items=(
            ('5', "5°", "按住 Ctrl 时吸附到 5 的倍数"),
            ('15', "15°", "按住 Ctrl 时吸附到 15 的倍数"),
            ('45', "45°", "按住 Ctrl 时吸附到 45 的倍数"),
        ),
        default='5',
        description="旋转时按住 Ctrl 吸附到这个角度；Ctrl+Shift 固定吸附 15°",
    )

    rot_sensitivity: bpy.props.FloatProperty(
        name="旋转灵敏度",
        default=1.0,
        min=0.1, max=5.0,
        soft_min=0.2, soft_max=3.0,
        precision=2,
        description="整体倍率。旋转的角度曲线已经按原生 gizmo 的球面映射标定"
                    "（拖 110px ≈ 90°），这里只是把球半径按倍率收紧/放松："
                    "1.0 = 原生手感，调大更跟手、调小更精细；"
                    "视口顶部会实时显示当前生效的 °/px",
    )

    show_hud: bpy.props.BoolProperty(
        name="指针旁显示轴与角度",
        default=True,
        description="在光标旁边画一行「轴名 + 角度 + 吸附档」，不用抬头看视口顶部。"
                    "blf 不可用时自动退回原来的顶部提示",
    )

    wrap_cursor: bpy.props.BoolProperty(
        name="指针贴边时自动回中",
        default=True,
        description="修掉「一直往一个方向转，指针撞到屏幕边缘就转不动了」："
                    "靠近边缘时把指针拉回视口中心，角度只按相对位移累加，不会跳变",
    )

    ring_zone: bpy.props.IntProperty(
        name="旋转活动半径",
        default=110,
        min=0, max=500,
        soft_min=40, soft_max=240,
        subtype='PIXEL',
        description="单轴旋转时指针最多离开控制器多少像素。顶到边界就把指针拉回圆周上"
                    "（方位角不变，角度照样连续增长），所以手可以一直往一个方向推，"
                    "指针始终待在控制器附近 —— 确认后能马上抓到 gizmo。"
                    "0 = 不限制（指针可以跑到任意远处）",
    )

    follow_mouse: bpy.props.BoolProperty(
        name="鼠标跟随骨骼",
        default=True,
        description="用导航键切换控制器时，把鼠标指针移到该骨骼在视口里的位置；"
                    "骨骼在画面外时不动鼠标",
    )

    solo_display: bpy.props.BoolProperty(
        name="旋转/移动时只显示当前控制器",
        default=True,
        description="进入旋转/移动模态后，把其它骨骼全部临时隐藏，只留当前控制器（多选时留所有选中的）；"
                    "滚轮切换骨骼时跟着换，结束模态（左键确认 / 右键退一步 / 双击右键全撤）自动恢复原样。"
                    "只改显示，不影响选择、动画和渲染",
    )

    use_xyz_keys: bpy.props.BoolProperty(
        name="X/Y/Z 键切单轴",
        default=False,
        description="旋转/移动模态里按 X/Y/Z 直接锁到对应单轴。关掉后这两个模式里 "
                    "按 X/Y/Z 不做事（按键仍被模态吃掉，避免触发进入键再叠一个模态），"
                    "只能用滚轮切换轴向",
    )

    warp_after_view: bpy.props.BoolProperty(
        name="视图操作后指针回到控制器",
        default=True,
        description="旋转/移动模态里用中键操作完视角（含 Shift 平移、Ctrl 缩放）后，"
                    "把鼠标送回当前控制器上，不用自己再把指针找回来；"
                    "落点偏移和上面的 warp_dx/dy 共用",
    )

    isolate_collection: bpy.props.BoolProperty(
        name="切集合时只留当前这一个",
        default=True,
        description="从一个集合跳到另一个集合时，把上一次进过的那组整个隐藏，"
                    "屏幕上永远只有一套控制器。想同时看两组就关掉它",
    )

    ikfk_auto_key: bpy.props.BoolProperty(
        name="切换后自动插帧",
        default=True,
        description="IK/FK 切换完，在**当前帧**给这个肢体两侧的控制器都打上关键帧，"
                    "连同 IK/FK 开关属性。只钉一侧的话，拖时间轴时另一侧会从"
                    "别处插值回来，姿态当场飞掉",
    )

    ikfk_key_only_anim: bpy.props.BoolProperty(
        name="只在已有动画时插帧",
        default=True,
        description="骨架还没有 Action、也没开自动关键帧时不插 —— 免得纯摆姿势"
                    "看效果也凭空建出一个动作。想看「纯摆也插针」就关掉它",
    )

    sync_collections: bpy.props.BoolProperty(
        name="IK/FK 同步骨骼集合可见性",
        default=True,
        description="切换 IK/FK 时自动隐藏另一模式的集合（Arm.L (IK) / Arm.L (FK) 等）",
    )


    auto_frame: bpy.props.BoolProperty(
        name="视口外自动平滑移过去",
        default=True,
        description="切换到的控制器不在当前 3D 视口里时，用约 0.28 秒的缓动把视口平移过去，"
                    "而不是瞬移；移动结束后鼠标再归位",
    )

    frame_margin: bpy.props.FloatProperty(
        name="集合取景余量",
        default=1.5,
        min=1.0, max=4.0, soft_min=1.0, soft_max=3.0,
        precision=2, step=5,
        description="跳进一个集合时，在 Blender 原生框选算出的距离上再乘这个倍率。"
                    "1.00 = 完全原生（控制器贴着屏幕边），越大离得越远、留白越多。"
                    "集合越小越该调大",
    )

    frame_min_dist: bpy.props.FloatProperty(
        name="最小视距",
        default=0.8,
        min=0.0, max=50.0, soft_max=10.0, precision=2,
        description="取景距离的下限（Blender 单位，一般 1 单位 = 1 米）。"
                    "集合里只有一根骨骼时原生框选会贴到脸上，用这个兜底；0 = 不限制",
    )

    warp_dx: bpy.props.IntProperty(
        name="落点 X 偏移",
        default=0,
        min=-600, max=600,
        description="落点左右偏移（像素，右为正）。用来把指针从 Gizmo 正中心挪到旋转环/位移轴上",
    )
    warp_dy: bpy.props.IntProperty(
        name="落点 Y 偏移",
        default=0,
        min=-600, max=600,
        description="落点上下偏移（像素，上为正）。用来把指针从 Gizmo 正中心挪到旋转环/位移轴上",
    )

    # ---- 导航
    skip_hidden: bpy.props.BoolProperty(
        name="跳过隐藏骨骼",
        default=True,
        description="骨骼自身隐藏、或它所在的所有集合都隐藏时跳过（含集合可见性判据）；"
                    "集合内滚轮/↑↓ 同样跳过被 H 的骨骼。骨架按 IK/FK 态藏骨、"
                    "想连藏起来的那套一起轮时关掉此项",
    )
    mpath_refresh: bpy.props.BoolProperty(
        name="确认后刷新运动路径",
        default=True,
        description="旋转/移动确认后重算**已有**的运动路径缓存（原生变换结束时本来"
                    "就会重算，插件预览是直写矩阵，不显式刷新就停在旧值）。"
                    "v5.2.37 起恢复默认开启：刷新改成延后 50ms 的异步任务并防抖，"
                    "不会拖慢操作手感；只重算本来就有路径的骨骼，不会新增路径。"
                    "完全不用运动路径、想省掉这份开销时可以关掉",
    )
    mpath_refresh_migrated: bpy.props.BoolProperty(
        default=False, options={'HIDDEN'},
    )
    mpath_refresh_v37: bpy.props.BoolProperty(
        default=False, options={'HIDDEN'},
    )
    same_mode: bpy.props.BoolProperty(
        name="同 IK/FK 模式过滤",
        default=True,
        description="当前骨骼是 IK 就只在 IK 控制器里转，是 FK 就只在 FK 里转；"
                    "名字里没有 ik/fk 标记的骨骼不受影响",
    )
    collection_keyword: bpy.props.StringProperty(
        name="优先集合名关键字（留空则用最小集合）",
        default="",
        description="骨骼同时属于多个集合时，优先用名字含该关键字的集合。例如填「左手」",
    )

    def draw(self, context):
        layout = self.layout

        # ---- 星标独显（放在最上面：功能新，藏到下面找不到）
        sbox = layout.box()
        sbox.label(text="星标饼 / 独显", icon='SOLO_ON')
        sbox.prop(self, "pie_custom", text="饼菜单用星标饼（自绘 · 每格带 ★）")
        if self.pie_custom:
            sbox.label(text="滑到格子 = 高亮，松手进集合（选第一根）；滑到 ★ = 独显这一格，"
                            "滑回格子就退出独显", icon='CHECKMARK')
            sbox.label(text="不用真的滑到格子上：给出方向就选中（见下面的「触发死区」）",
                       icon='CHECKMARK')
            shape = sbox.column(align=True)
            shape.label(text="饼形态 / 手感", icon='PREFERENCES')
            r = shape.row(align=True)
            r.prop(self, "pie_dead_zone", text="死区")
            r.prop(self, "pie_label_size", text="字号")
            r2 = shape.row(align=True)
            r2.prop(self, "pie_radius", text="半径")
            r2.prop(self, "pie_hole", text="中心孔")
            shape.prop(self, "pie_star_size", text="星标大小")
            shape.label(text="死区 0 = 一动就选；调小更跟手，调大更稳", icon='INFO')
            sbox.label(text="画不出来（老显卡 / API 变动）会自动退回原生饼，键位不受影响",
                       icon='INFO')
        else:
            sbox.label(text="现在是原生饼：稳，但拿不到悬停事件，也没有外沿星标",
                       icon='INFO')
        sbox.label(text="不开星标饼也能独显：侧栏「驭骨」面板每个集合行右边有「独显」按钮，"
                        "饼菜单配置区每格后面有 ★", icon='INFO')
        sbox.label(text="独显快照存在骨架里（cn_solo_prev），重开文件也能还原", icon='INFO')

        # ---- 快捷键
        kbox = layout.box()
        kbox.label(text="快捷键（Pose 模式）", icon='ARROW_LEFTRIGHT')
        kbox.prop(self, "nav_keys", text="")
        if self.nav_keys == 'WHEEL':
            kbox.label(text="纯滚轮会抢掉视图缩放，Pose 模式下滚轮不能缩放", icon='ERROR')
            kbox.label(text="想保留缩放，请选「Ctrl + 滚轮」或方向键", icon='INFO')
        elif self.nav_keys in ('CTRL_WHEEL', 'ALT_WHEEL'):
            kbox.label(text="纯滚轮照常缩放，只有带修饰键的那一下归导航", icon='CHECKMARK')
        elif self.nav_keys == 'CUSTOM':
            kbox.label(text="不自动绑键：Preferences > Keymap 里搜 controller_nav 自己绑", icon='INFO')
        else:
            kbox.label(text="不碰滚轮，视图缩放完全不受影响", icon='CHECKMARK')
        kbox.prop(self, "collection_keys", text="集合切换")
        if coll_key_specs() is None and self.collection_keys != 'NONE':
            kbox.label(text="集合切换键和导航键冲突了，没绑；换一个组合即可", icon='ERROR')
        else:
            kbox.label(text="集合切换：在预设集合之间循环，并选中新集合的第一根骨骼", icon='INFO')
        kbox.prop(self, "use_m_key")

        # ---- 全部动作的键位（v5.0）
        allbox = layout.box()
        allbox.label(text="全部快捷键（Pose 模式）", icon='KEYINGSET')
        allbox.label(text="点键名 → 直接按下要用的键；鼠标中键 / 侧键 / 滚轮方向都能绑",
                     icon='INFO')
        grid = allbox.column(align=True)
        for act, lbl, desc in KB_ACTIONS:
            row = grid.row(align=True)
            row.label(text=lbl)
            sub = row.row(align=True)
            sub.alignment = 'RIGHT'
            btn = sub.operator("pose.cn_key_capture_v45", text=kb_label(act))
            btn.action = act
            if desc:
                btn = sub.operator("pose.cn_key_capture_v45", text="", icon='QUESTION')
                btn.action = act
            clr = sub.operator("pose.cn_key_clear_v45", text="", icon='X')
            clr.action = act
        allbox.operator("pose.cn_key_reset_v45", text="全部恢复默认", icon='LOOP_BACK')

        # ---- 旋转/移动模式里的滚轮
        wbox = layout.box()
        wbox.label(text="旋转 / 移动模式里的滚轮", icon='MOUSE_MOVE')
        wbox.prop(self, "mod_zoom_mods")
        wbox.prop(self, "mod_bone_mods")
        wbox.prop(self, "mod_coll_mods")
        wbox.prop(self, "mod_axis_mods")
        ax = self.mod_axis_mods
        if ax == 'NONE':
            wbox.label(text="切轴没绑键：滚轮在这个组合下什么都不做", icon='ERROR')
        elif ax == 'BARE':
            wbox.label(text="不按修饰键推滚轮 = 切换轴向（自由 → X → Y → Z）", icon='INFO')
        else:
            wbox.label(text="按住 %s 推滚轮 = 切换轴向（自由 → X → Y → Z）"
                       % dict((k, n) for k, n, _d in MOD_COMBO_ITEMS).get(ax, ax),
                       icon='INFO')
        wbox.label(text="四个动作都别绑同一个组合，撞了就按「修饰键多的先」算", icon='INFO')
        wbox.label(text="修饰键写得多的先匹配，所以 Shift+Alt 不会被 Shift 抢走", icon='INFO')

        # ---- 旋转 / 移动进入键（旧入口，值仍然同步到上面那张表）
        abox = layout.box()
        abox.label(text="旋转 / 移动进入键", icon='DRIVER_ROTATIONAL_DIFFERENCE')
        row = abox.row(align=True)
        row.prop(self, "use_x_rotate", text="")
        row.prop(self, "rot_key", text="旋转")
        row = abox.row(align=True)
        row.prop(self, "move_key", text="移动")
        row.prop(self, "move_key_mod", text="")
        reason = move_key_conflict()
        if reason:
            abox.label(text="移动键没绑上：%s，换一个键或加修饰键" % reason, icon='ERROR')
        else:
            abox.label(text="移动键已绑定：%s（Pose 模式）" % move_key_label(),
                       icon='CHECKMARK')
        abox.label(text="模态里滚轮向下 = 下一根轴：自由 → X → Y → Z", icon='INFO')
        abox.label(text="模态里按上面这两个键互相切换（%s 切移动 / %s 切旋转），之前的调整保留"
                   % (move_key_label() if move_key_ok() else "移动键", rot_key_label()), icon='INFO')
        abox.label(text="中键不进入移动；进入后按中键即可原地转视角，姿态不会丢", icon='INFO')

        # ---- 自由旋转设置
        rbox = layout.box()
        rbox.label(text="自由旋转", icon='DRIVER_ROTATIONAL_DIFFERENCE')
        rrow = rbox.row(align=True)
        rrow.prop(self, "snap_deg", expand=True)
        rrow.label(text="按住 Ctrl 时吸附")
        rbox.prop(self, "finger_single")
        if self.finger_single:
            rbox.prop(self, "finger_axis")
        rbox.prop(self, "rot_sensitivity")
        rbox.prop(self, "ring_zone")
        rbox.prop(self, "show_hud")
        rbox.prop(self, "wrap_cursor")
        rbox.label(text="按 %s 进入后：鼠标直接转，跟拽 gizmo 一个手感" % rot_key_label(), icon='INFO')
        rbox.label(text="三轴全开=自由转；两轴=水平/垂直各管一根；一轴=只转那根", icon='INFO')
        rbox.label(text="轴锁在骨骼属性里设，或点侧栏「手动锁轴」一键切", icon='INFO')
        rbox.label(text="滚轮换关节 · Shift 微调 · Ctrl 吸附 · 中键转视角 · 左键确认 · 右键退一步", icon='INFO')
        rbox.label(text="（blf 不可用时自动退回顶部的文字提示）", icon='INFO')

        # ---- 鼠标跟随
        mbox = layout.box()
        mbox.label(text="鼠标跟随", icon='CURSOR')
        mbox.prop(self, "isolate_collection")
        mbox.prop(self, "follow_mouse")
        mbox.prop(self, "warp_after_view")
        mbox.prop(self, "solo_display")
        mbox.prop(self, "use_xyz_keys")
        mbox.prop(self, "auto_frame")
        row = mbox.row(align=True)
        row.prop(self, "frame_margin", text="集合取景余量", slider=True)
        row.prop(self, "frame_min_dist", text="最小视距")
        mbox.label(text="取景余量：1.00 = Blender 原生（控制器贴屏幕边），集合越小越要调大", icon='INFO')
        mbox.label(text="视口跟随：控制器在画面外时，0.28 秒缓动平移过去，不会瞬移", icon='INFO')
        mbox.label(text="旋转/移动模态里中键操作完视角，指针自动回到当前控制器", icon='INFO')
        mbox.label(text="导航键切换骨骼时，指针自动移到该控制器上，接着按 G/R/S 即可", icon='INFO')
        mbox.label(text="骨骼在视口外时不动鼠标；也可以点面板的「鼠标→当前骨骼」手动归位", icon='INFO')
        row = mbox.row(align=True)
        row.prop(self, "warp_dx", text="X 偏移")
        row.prop(self, "warp_dy", text="Y 偏移")
        row.operator("pose.cn_warp_offset_reset_v45", text="", icon='LOOP_BACK')
        mbox.label(text="落点微调：X 右为正、Y 上为正（像素）", icon='INFO')
        mbox.label(text="指针落在 Gizmo 正中心时，往旋转环方向偏一点就能直接拖旋转", icon='INFO')

        kbox = layout.box()
        kbox.label(text="IK / FK", icon='ARMATURE_DATA')
        kbox.prop(self, "ikfk_auto_key")
        if self.ikfk_auto_key:
            kbox.prop(self, "ikfk_key_only_anim")
        kbox.prop(self, "sync_collections")
        kbox.label(text="插帧 = 切换那一帧给两侧控制器都打关键帧，拖时间轴不会跳回原姿势", icon='INFO')
        kbox.label(text="关掉「只在已有动画时插帧」= 纯摆姿势也会插针 / 建动作", icon='INFO')

        box = layout.box()
        box.label(text="预设集合（按这里的名字创建 / 归组）", icon='BONE_DATA')
        col = box.column(align=True)
        for i, item in enumerate(self.presets):
            row = col.row(align=True)
            row.prop(item, "col_name", text="")
            up = row.row(align=True)
            up.enabled = i > 0
            up.operator("cn.preset_move_v45", text="", icon='TRIA_UP').index = i
            up = row.row(align=True)
            up.enabled = i < len(self.presets) - 1
            op = up.operator("cn.preset_move_v45", text="", icon='TRIA_DOWN')
            op.index = i
            op.delta = 1
            row.operator("cn.preset_remove_v45", text="", icon='X').index = i

        if not self.presets:
            box.label(text="清单是空的，点下面的「添加」写几个名字", icon='INFO')

        bar = box.row(align=True)
        bar.operator("cn.preset_add_v45", text="添加", icon='ADD')
        bar.operator("cn.preset_reset_v45", text="恢复默认", icon='LOOP_BACK')

        box.prop(self, "move_bones")
        if self.move_bones:
            box.prop(self, "confirm_cross_move")
            box.label(text="归组前会提示：有哪些骨骼会被从别的集合移走", icon='INFO')

        box = layout.box()
        box.label(text="导航", icon='ARROW_LEFTRIGHT')
        box.prop(self, "skip_hidden")
        box.prop(self, "same_mode")
        box.prop(self, "mpath_refresh")
        box.prop(self, "collection_keyword")
        box.label(text="同 IK/FK：当前骨骼带 _ik / _fk 标记时，导航只在同模式里转", icon='INFO')
        box.label(text="名字里带 pole 的极向控制器按 IK 处理", icon='INFO')

        layout.label(text="做法：把控制器归到集合，导航键就在该集合内循环")
        layout.label(text="加选 = 按住 Shift 再按导航键　M = 归入预设集合")
        layout.label(text="旋转 = 按 X 后直接拖鼠标，轴自动判断，滚轮换关节")


# ---------------------------------------------------------------- 注册


class POSE_OT_cn_set_bone_motion_v45(bpy.types.Operator):
    """设置选中骨骼自己的动作权限（覆盖集合）"""
    bl_idname = "pose.cn_set_bone_motion_v45"
    bl_label = "骨骼动作权限"
    bl_options = {'REGISTER', 'UNDO'}

    mode: bpy.props.EnumProperty(items=BONE_MOTION_ITEMS, default='INHERIT')

    def execute(self, context):
        sel = [b for b in (context.selected_pose_bones or []) if b is not None]
        if not sel and context.active_pose_bone is not None:
            sel = [context.active_pose_bone]
        if not sel:
            self.report({'WARNING'}, "没有选中的骨骼")
            return {'CANCELLED'}
        for pb in sel:
            set_bone_motion_mode(pb, self.mode)
        return {'FINISHED'}


class POSE_OT_cn_set_collection_motion_v45(bpy.types.Operator):
    """设置当前活动骨骼所在集合的动作权限"""
    bl_idname = "pose.cn_set_collection_motion_v45"
    bl_label = "集合动作权限"
    bl_options = {'REGISTER', 'UNDO'}

    mode: bpy.props.EnumProperty(items=MOTION_MODE_ITEMS, default='BOTH')

    @classmethod
    def poll(cls, context):
        return (context.mode == 'POSE'
                and context.object is not None
                and context.object.type == 'ARMATURE')

    def execute(self, context):
        rig = context.object
        cur = rig.data.bones.active if rig.data.bones.active else None
        col = nav_collection(rig, cur) if cur else None
        if col is None:
            self.report({'WARNING'}, "当前骨骼不在可用集合里")
            return {'CANCELLED'}
        if not set_collection_motion_mode(rig, col, self.mode):
            self.report({'WARNING'}, "写入集合动作权限失败")
            return {'CANCELLED'}
        return {'FINISHED'}


class POSE_OT_cn_delete_empty_collection_v45(bpy.types.Operator):
    """删除一个 0 骨骼的集合（骨骼不会被删除）"""
    bl_idname = "pose.cn_delete_empty_collection_v45"
    bl_label = "删除空集合"
    bl_options = {'REGISTER', 'UNDO'}

    collection: bpy.props.StringProperty()

    @classmethod
    def poll(cls, context):
        return (context.object is not None
                and context.object.type == 'ARMATURE')

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=360)

    def draw(self, context):
        layout = self.layout
        layout.label(text="删除空集合「%s」？" % (self.collection or ""), icon='ERROR')
        layout.label(text="骨骼不会被删除，只会取消这个集合的归属。", icon='INFO')

    def execute(self, context):
        rig = context.object
        if rig is None or rig.type != 'ARMATURE':
            return {'CANCELLED'}
        col = _coll_get(rig, self.collection or "")
        if col is None:
            self.report({'WARNING'}, "集合不存在")
            return {'CANCELLED'}
        if len(col.bones) != 0:
            self.report({'WARNING'}, "这个集合不是空集合，不能删除")
            return {'CANCELLED'}
        _arm_of(rig).collections.remove(col)
        return {'FINISHED'}


class POSE_OT_cn_set_collection_sort_v45(bpy.types.Operator):
    """设置当前活动骨骼所在集合的排序方式"""
    bl_idname = "pose.cn_set_collection_sort_v45"
    bl_label = "集合排序方式"
    bl_options = {'REGISTER', 'UNDO'}

    mode: bpy.props.EnumProperty(items=SORT_MODE_ITEMS, default='ASSIGN')

    @classmethod
    def poll(cls, context):
        return (context.mode == 'POSE'
                and context.object is not None
                and context.object.type == 'ARMATURE')

    def execute(self, context):
        rig = context.object
        cur = rig.data.bones.active if rig.data.bones.active else None
        col = nav_collection(rig, cur) if cur else None
        if col is None:
            self.report({'WARNING'}, "当前骨骼不在可用集合里")
            return {'CANCELLED'}
        if not set_collection_sort_mode(rig, col, self.mode):
            self.report({'WARNING'}, "写入集合排序失败")
            return {'CANCELLED'}
        return {'FINISHED'}


class POSE_OT_cn_lock_axes_v45(bpy.types.Operator):
    """把选中骨骼的旋转锁设成指定组合"""
    bl_idname = "pose.cn_lock_axes_v45"
    bl_label = "设置旋转锁"
    bl_options = {'REGISTER', 'UNDO'}

    mode: bpy.props.EnumProperty(items=(
        ('ONLY_X', "只留 X", ""),
        ('ONLY_Y', "只留 Y", ""),
        ('ONLY_Z', "只留 Z", ""),
        ('XZ', "X + Z", ""),
        ('ALL', "全锁", ""),
        ('NONE', "全解锁", ""),
    ), default='ONLY_X')

    def execute(self, context):
        sel = [b for b in (context.selected_pose_bones or []) if b is not None]
        if not sel and context.active_pose_bone is not None:
            sel = [context.active_pose_bone]
        if not sel:
            self.report({'WARNING'}, "没有选中的骨骼")
            return {'CANCELLED'}
        table = {
            'ONLY_X': (False, True, True),
            'ONLY_Y': (True, False, True),
            'ONLY_Z': (True, True, False),
            'XZ': (False, True, False),
            'ALL': (True, True, True),
            'NONE': (False, False, False),
        }
        locks = table[self.mode]
        for pb in sel:
            pb.lock_rotation = locks
        return {'FINISHED'}

CLASSES = (
    CN_KeyBind_v45,          # prefs 里的 CollectionProperty 引用它，必须最先注册
    CN_RigSpatialSettings_v45,
    POSE_OT_cn_delete_empty_collection_v45,
    POSE_OT_cn_set_collection_sort_v45,
    POSE_OT_cn_set_bone_motion_v45,
    POSE_OT_cn_set_collection_motion_v45,
    POSE_OT_cn_bone_order_v45,
    POSE_OT_cn_bone_order_batch_v45,
    POSE_OT_cn_bone_order_pick_v45,
    POSE_OT_cn_solo_collection_v45,
    POSE_OT_cn_wheel_v45,
    CN_Preset_v45,
    POSE_OT_cn_create_presets_v45,
    POSE_OT_cn_assign_preset_v45,
    POSE_OT_cn_new_collection_v45,
    POSE_OT_cn_preset_menu_v45,
    POSE_OT_cn_pie_jump_v45,
    POSE_OT_cn_pie_note_v45,
    POSE_OT_cn_pie_assign_v45,
    POSE_OT_cn_slot_order_v45,
    POSE_OT_cn_slot_swap_v45,
    POSE_OT_cn_auto_layout_v45,
    POSE_OT_cn_restore_visibility_v45,
    POSE_OT_cn_snapshot_visibility_v45,
    POSE_OT_cn_pie_autofill_v45,
    POSE_OT_cn_pie_clear_v45,
    POSE_OT_cn_first_bone_v45,
    CN_MT_pie_v45,
    CN_MT_pie2_v45,
    CN_MT_pie3_v45,
    CN_MT_preset_collections_v45,
    CN_MT_other_collections_v45,
    POSE_OT_cn_set_collection_v45,
    POSE_OT_cn_clear_collection_v45,
    POSE_OT_controller_nav_v45,
    POSE_OT_cn_warp_mouse_v45,
    POSE_OT_cn_warp_offset_reset_v45,
    POSE_OT_cn_collection_step_v45,
    POSE_OT_cn_collection_goto_v45,
    POSE_OT_cn_cycle_add_v45,
    POSE_OT_cn_cycle_remove_v45,
    POSE_OT_cn_remove_from_collection_v45,
    POSE_OT_cn_import_collections_v45,
    POSE_OT_cn_quick_rotate_v45,
    POSE_OT_cn_lock_axes_v45,
    POSE_OT_cn_ikfk_toggle_v45,
    POSE_OT_cn_ikfk_fingers_v45,
    POSE_OT_cn_ikfk_popup_v45,
    POSE_OT_cn_key_capture_v45,
    POSE_OT_cn_key_clear_v45,
    POSE_OT_cn_key_reset_v45,
    VIEW3D_PT_controller_nav_v45,
    CN_OT_preset_add_v45,
    CN_OT_preset_remove_v45,
    CN_OT_preset_move_v45,
    CN_OT_preset_reset_v45,
    ControllerNavPrefs_v45,
)

addon_keymaps = []


def remove_legacy_keymaps():
    """删掉指向已不存在 operator 的键位（v2.1–v2.4 残留 + 本版下掉的书签键）。"""
    removed = []
    wm = bpy.context.window_manager
    if wm is None:
        return removed
    for kc_name in ("user", "addon"):
        kc = getattr(wm.keyconfigs, kc_name, None)
        if kc is None:
            continue
        for km in kc.keymaps:
            for kmi in list(km.keymap_items):
                idn = getattr(kmi, "idname", "")
                if idn in OWN_OP_IDS:
                    continue
                if not (idn.startswith(LEGACY_PREFIXES) or idn in REMOVED_OP_IDS):
                    continue
                try:
                    km.keymap_items.remove(kmi)
                    removed.append("%s:%s" % (km.name, idn))
                except Exception:
                    pass
    return removed


def purge_stale_mouse_binding():
    """删掉 v4.9.12/13 留下的「鼠标中键进移动」绑定。

    旧版把 MOVE 绑在 MIDDLEMOUSE 上；升级后如果这条还在，用户想用
    中键/Alt+中键 仍然会被抢走。这里把所有指向本插件旋转 operator 的
    MIDDLEMOUSE 绑定都清掉（本版不再使用中键进入）。
    """
    wm = getattr(bpy.context, "window_manager", None)
    if wm is None:
        return 0
    removed = 0
    bad_ops = {'pose.cn_quick_rotate_v45', 'pose.cn_quick_move_v45'}
    for kc_name in ("user", "addon"):
        kc = getattr(wm.keyconfigs, kc_name, None)
        if kc is None:
            continue
        for km in kc.keymaps:
            for kmi in list(km.keymap_items):
                if getattr(kmi, "idname", "") not in bad_ops:
                    continue
                if getattr(kmi, "type", "") != 'MIDDLEMOUSE':
                    continue
                try:
                    km.keymap_items.remove(kmi)
                    removed += 1
                except Exception:
                    pass
    return removed


def register_keymaps():
    """导航键（按偏好里的方案）+ 可选的 M 预设菜单。"""
    # prefs 已经就绪就立刻补默认键位；还没就绪则交给 _kb_bootstrap 定时器
    try:
        ensure_keybinds()
    except Exception:
        pass
    purge_stale_mouse_binding()
    remove_legacy_keymaps()
    kc = bpy.context.window_manager.keyconfigs.addon
    if kc is None:
        return
    km = kc.keymaps.new(name='Pose', space_type='EMPTY')

    specs = nav_key_specs()
    if specs is not None:
        (key_up, mods_up), (key_dn, mods_dn) = specs
        for key, mods, mode in ((key_up, mods_up, 'PREV'), (key_dn, mods_dn, 'NEXT')):
            for extend in (False, True):
                kw = dict(mods)
                if extend:
                    kw['shift'] = True
                kmi = km.keymap_items.new('pose.controller_nav_v45', key, 'PRESS', **kw)
                kmi.properties.mode = mode
                kmi.properties.extend = extend
                addon_keymaps.append((km, kmi))

    mspec = kb_spec('MENU_COLL')
    if mspec is not None:
        try:
            kmi = km.keymap_items.new(
                'pose.cn_preset_menu_v45', mspec[0],
                'CLICK' if 'MOUSE' in mspec[0] else 'PRESS', **mspec[1])
            addon_keymaps.append((km, kmi))
        except Exception:
            pass

    # 三层饼：先绑修饰键多的。键盘映射按添加顺序匹配，少修饰的才不会先吃掉
    for act, menu_name in (('PIE_3', "CN_MT_pie3_v45"),
                           ('PIE_2', "CN_MT_pie2_v45"),
                           ('PIE_1', "CN_MT_pie_v45")):
        spec = kb_spec(act)
        if spec is None:
            continue
        key, mods = spec
        try:
            mode = 'CLICK_DRAG' if ('MOUSE' in key and 'WHEEL' not in key) else 'PRESS'
            if bool(pref('pie_custom', False)):
                # 星标饼：自己画的，才有「移上去就独显」的悬停事件
                kmi = km.keymap_items.new('pose.cn_wheel_v45', key, mode, **mods)
                kmi.properties.layer = {'PIE_1': 1, 'PIE_2': 2, 'PIE_3': 3}.get(act, 1)
            else:
                kmi = km.keymap_items.new('wm.call_menu_pie', key, mode, **mods)
                kmi.properties.name = menu_name
            addon_keymaps.append((km, kmi))
        except Exception:
            pass

    if pref('use_x_rotate', True):
        rkey, rkw = rot_key_spec()
        try:
            kmi = km.keymap_items.new('pose.cn_quick_rotate_v45', rkey, 'PRESS', **rkw)
            kmi.properties.start_mode = 'ROT'
            addon_keymaps.append((km, kmi))
        except Exception:
            pass

    # 移动进入键：在插件偏好里改，不再占鼠标中键（中键留给原生视图旋转）
    if move_key_ok():
        mkey, mkw = move_key_spec()
        try:
            kmi = km.keymap_items.new('pose.cn_quick_rotate_v45', mkey, 'PRESS', **mkw)
            kmi.properties.start_mode = 'MOVE'
            addon_keymaps.append((km, kmi))
        except Exception:
            pass

    # 集合切换键（和导航键冲突时 coll_key_specs 会返回 None，宁可不绑）
    cspec = coll_key_specs()
    if cspec is not None:
        (key_up, mods_up), (key_dn, mods_dn) = cspec
        for key, mods, mode in ((key_up, mods_up, 'PREV'), (key_dn, mods_dn, 'NEXT')):
            kmi = km.keymap_items.new('pose.cn_collection_step_v45', key, 'PRESS', **mods)
            kmi.properties.mode = mode
            addon_keymaps.append((km, kmi))

    # IK/FK：一个键就地切换，一个键弹面板
    for act, op_id in (('IKFK_TOGGLE', 'pose.cn_ikfk_toggle_v45'),
                       ('IKFK_POPUP', 'pose.cn_ikfk_popup_v45')):
        spec = kb_spec(act)
        if spec is None:
            continue
        try:
            kmi = km.keymap_items.new(op_id, spec[0],
                                      'CLICK' if 'MOUSE' in spec[0] else 'PRESS',
                                      **spec[1])
            addon_keymaps.append((km, kmi))
        except Exception:
            pass


def unregister_keymaps():
    for km, kmi in addon_keymaps:
        try:
            km.keymap_items.remove(kmi)
        except Exception:
            pass
    addon_keymaps.clear()


def refresh_keymap():
    unregister_keymaps()
    register_keymaps()


def rigui_sidepanel_category(addons=None):
    """如果装了 rigUI，就返回它侧栏面板所在的标签名，让我们的面板并进同一个标签。"""
    try:
        prefs = bpy.context.preferences
    except Exception:
        return ""
    if addons is None:
        addons = prefs.addons
    for key in ("bl_ext.user_default.amp_rig_ui", "amp_rig_ui"):
        addon = addons.get(key)
        if addon is None:
            continue
        p = getattr(addon, "preferences", None)
        name = getattr(p, "rig_ui_sidepanel_location", "") if p is not None else ""
        if name:
            return name
    return ""


def register():
    _MODAL_ONCE[0] = False      # 重载/重启后闸门必须是开的
    # 并进 rigUI 的侧栏标签（没装 rigUI 就用自己的 Nav）
    category = rigui_sidepanel_category()
    if category:
        VIEW3D_PT_controller_nav_v45.bl_category = category

    for cls in CLASSES:
        bpy.utils.register_class(cls)

    bpy.types.Object.cn_spatial = bpy.props.PointerProperty(
        type=CN_RigSpatialSettings_v45)
    ensure_default_presets()
    register_keymaps()
    bpy.app.timers.register(_kb_bootstrap, first_interval=0.6)
    _RIGUI_TIMER["timer"] = bpy.app.timers.register(_rigui_patch_timer, first_interval=1.0)


def unregister():
    try:
        restore_rigui_tooltips()
    except Exception:
        pass
    try:
        unpatch_rigui_popup()
    except Exception:
        pass
    for key in ("timer",):
        h = _RIGUI_TIMER.get(key)
        if h is not None:
            try: bpy.app.timers.unregister(h)
            except Exception: pass
            _RIGUI_TIMER[key] = None
    _pick_order_timer(False)          # 点选排序的心跳也要收掉
    unregister_keymaps()
    try:
        del bpy.types.Object.cn_spatial
    except Exception:
        pass
    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
