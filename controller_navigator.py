# -*- coding: utf-8 -*-
# Controller Navigator v5.0.0 — IK/FK 一键切换 + 全键位自定义 + 集合独占
#
# 这一版四件事：
#
# ① IK/FK 切换（新）。选中 IK 或 FK 控制器，按一下键就切到另一边，并自动把
#    刚离开的那套控制器收起来、把选择挪到新的控制器上。两套骨架自动识别：
#      · Rigify    —— 有 rig_id，直接调它自带的 snap，pole / stretch 全对
#      · AniMate 类 —— 控制器骨上带 ik_fk_switch，自己算矩阵快照
#    实测（pose 瞎玩.blend / rig.002，手臂 IK→FK）：只改属性会跳 26.3 cm，
#    补上快照后 0.1 mm；腿 FK→IK 从 17.8 cm 压到 0.0 mm。
#    Rigify 那边同一基线下和「自带按钮」的结果差 0.000000 m。
#
# ② 所有动作的键位都能自己绑。偏好里一张表：点键名 → 直接按你要的键，
#    鼠标中键 / 侧键 / 滚轮方向都认。旋转、移动模式里的滚轮也一样 ——
#    缩放 / 切骨骼 / 切集合 三个修饰键组合自由配，默认 Shift / Alt / Ctrl+Alt。
#
# ③ 切集合只留一个。从一个集合跳到另一个，上一次进的那组整组隐藏，
#    屏幕上永远只有一套控制器。
#
# ④ 删掉了移动模式里「Ctrl+滚轮 预排下次进旋转用哪个轴」。滚轮轮询 + 中键
#    确认两步才到位，而直接按旋转键一步就到，纯属多余。
#
# v4.9.99 — 单轴方向「进场定死」：RIM / LIN 两式，删掉每帧现算的切向
#
#   用户报障：v4.9.98 把原本没问题的转向也弄反了，角度还在乱跳。
#   两个独立的错，都修掉了：
#
#   ① dy 的符号。模态传进来的 dy 是「向下为正」（dy = -(mouse_y 增量)），
#      屏幕投影 / 方向向量却是「向上为正」。v4.9.97/98 的
#      `dx*u.x + (-dy)*u.y` 里 u 又是 y 向下的，两次取反 = 没取反，
#      于是竖直方向的转向整体反了 —— 用户看到的「X 轴鼠标往上、角色往下弯」。
#      现在统一翻成 dyu = -dy，u / 方位角全在 y 向上系里算。
#      （移动 _move_step 早就翻对了，所以移动一直没问题。）
#
#   ② 「每帧按当前姿态现算切向」这条路本身不成立：姿态一转，算出来的屏幕
#      切向就会翻面（v4.9.97 的探测法、v4.9.98 的解析法都一样，只是噪声大小
#      不同），迟滞只能压住符号不能修正方向，压不住的时候整段甩飞。
#
#   现在只看进场那一刻的视角，把映射定死，整段手势不再变卦：
#     · RIM（轴基本对着镜头，圆环是个圈）→ 指针绕枢轴的方位角 = 骨骼转角，
#       屏幕逆时针为正、再看轴朝观察者还是背向。等价于抓住轮缘转轮子。
#     · LIN（轴躺在屏幕平面，圆环是一条线）→ 固定方向 u = A×n 投屏
#       （n 指向观察者）= 离观察者最近那一侧绕 +A 的运动方向。往 u 拖 = 正转。
#       轴躺着时圆环两侧反向动，可见运动本来二义，取最近侧 = 视线里最先生
#       变化的一片，往上拖就是往上抬。
#   删除：_tip_dir_live / _ring_fallback / _live_u / _had_live（连续翻面的源头）。
#
#   v4.9.98 — 单轴旋转反向的一次失败尝试（保留记录）：解析切向 + 废除极角分支
#
#   用户报障（正交视图）：X 轴鼠标往上、角色向下弯腰；Y 轴鼠标往左、角色往右转。
#   v4.9.97 的「实时探测」没修好，实测抓到两个独立病灶：
#
#   ① 探测轴 ≠ 施加轴。_apply 绕的是手势基线冻结的固定轴
#      A = m_start旋转 @ 本地轴（可见矩阵 M' = R_pivot @ m_start，带约束也成立）；
#      而 _tip_dir_live 探测绕的是「随骨骼转动的本地轴」（rotate_basis）。
#      姿态一转两个轴就分开，实测切向 u 隔一步反一次
#      （(0,+1) ↔ (−0.55,−0.84) 振荡），累计量一步进一步退 —— X 轴的反向。
#   ② 极角分支（轴接近镜头方向时启用）「屏幕逆时针=正转」物理上没错，
#      但直线拖动的转向取决于指针碰巧在枢轴上方还是下方：
#      指针在下方往左拖 = 屏幕顺时针 = 负角度 = 角色往右转 —— Y 轴的反向。
#
#   修法：_tip_dir_live 改解析计算 —— 末端 +dθ 速度 = A × (p−pivot)，
#    p = R(θ) @ p_start 用当前累计角直接算，与施加路径严格同轴、零噪声、
#    带约束骨骼同样成立；_single_axis_step 废除极角分支，全视角统一走
#    解析切向。绕圈拖依然自洽（切向跟着姿态转，绕圈的手始终追得上），
#    任意指针位置都保证 dot(末端位移, 手位移) ≥ 0。
#
#   v4.9.97 — 单轴旋转「手往右、控件往左转」：方向冻结改为实时切向
#
#   用户报障（正交前视图 / c_root_master.x / 单轴）：明明往右拖，控件却往左转；
#   视角正对轴向时绕圈反而直观。实测（diag_rot_dir.py）：前视图里本地 X 的
#   屏幕切向 u = (-0.095, +0.995)，几乎纯竖直 —— 水平拖动只有 9.5% 被算成转动，
#   符号还随投影细微差别乱定；而且旧版在进场时把方向算一次就冻结
#   （_assign_gesture → _dir_win），骨骼转起来之后切向跟着转，转过临界点
#   固定方向就和真实切向背离，屏幕运动与手背道而驰。
#
#   修法：单轴拖动的切向每帧按「当前姿态」现算（_tip_dir_live，探测末端
#   屏幕位移方向），缓存 _dir_win 只作退化兜底。数学上保证
#   dot(末端屏幕位移, 手位移) = sens·|v|·(Δ·u)² ≥ 0 —— 永远不会跟手对着干。
#   代价（几何必然）：一次直线拖到切向转成垂直（约 90°）后要换手继续。
#   轴接近镜头方向仍走极角绕枢轴（那条路本来就 1:1 跟手）。
#   只动 _single_axis_step / 新增 _tip_dir_live；提交、切轴、撤销、移动不动。
#
#   v4.9.96 — 移动模式「跟不动」的真正主因：像素→世界比例在正交下算错一半
#
#   用户报障（pose 瞎玩.blend / rig.002 / c_root_master.x，正交视口）：
#   移动模式按住控制器，手走一大段，骨骼只走一小段 ——「不跟鼠标 / 跟得很慢」，
#   时轻时重、无法稳定复现。
#
#   实测（exp_px.py，正交 view_distance 1.350、视口 1188×1151）：
#       真实 0.001637 m/px      _px_scale 旧公式 0.000854 m/px     只有 52%
#   旧公式 `view_distance·2·tan(20°)/height` 只在「透视 + 约 40° 垂直 FOV +
#   视距恰等于枢轴深度」时成立；正交视口下 view_distance 与可见高度没有这个
#   关系，偏差随缩放变化 → 所以「有时明显、有时只是慢」。
#   透视下也偏（同一文件切到透视后仍偏高 34%）。
#
#   修法：_px_scale 改成在枢轴处投影 1 个世界单位的相机右向量、量屏幕走多少
#   像素，取倒数。正交 / 透视 / 任意焦距 / 任意缩放都精确。
#   验收（exp_screen2.py，真实文件真实事件流）：
#       正交  手走 200px → 骨骼屏幕位移 200.0px（比值 1.000）
#       透视  手走 200px → 骨骼屏幕位移 200.0px（比值 1.000）
#   该函数只被 _move_step 使用，旋转灵敏度走 _deg_per_px，手感不受影响。
#
#   v4.9.95 — 移动模式「按着控制器不跟鼠标 / 跟得很慢」
#
#   用户报障（pose 瞎玩.blend / rig.002 / c_root_master.x，自动关键帧开）：
#   移动模式下偶尔跟不住鼠标，有时轻有时重，稳定复现不出来。
#
#   根因：_move_preview 每个 MOUSEMOVE 都调一次 view_layer.update()，强制求值
#   整条依赖图。实测（bench_move2.py，420 骨 + 角色网格）：
#     写 matrix + update = 3.36ms/次   写 matrix 不求值 = 0.0066ms（577×），
#     两者结果完全一致（误差 7e-15）。单次 update 空转 0.001ms，贵的是「姿态
#     变了之后的那次求值」。鼠标 125Hz 时 3.4ms×125 = 425ms/s，事件立刻排队，
#   画面比手慢好几帧 —— 就是「跟得很慢」，场景越重越明显（"有时很明显"）。
#   旋转路径从不 update，所以只有移动模式有这个毛病。
#   改法：预览只写 matrix_basis/matrix，求值交给下一次重绘统一做一次
#   （实测每帧 3.2ms → 0.10~0.21ms）。
#
#   顺带修掉 _move_flush 的一个隐患：它先回滚到基准再交给原生 translate，
#   而原生 transform.translate 在这个 rig 上会返回 FINISHED 却一格不动
#   （实测位移 0.0000）—— 等于提交时把整段位移丢了。现在提交后校验实际位移，
#   没动就退回直接写矩阵。
#
#   v4.9.94 — 带约束控制器「转的不是 gizmo 那根轴」+「切轴瞬间跳一下」
#
#   用户报障（同一真实工程，c_arm_fk.r = ArmLock(ARMATURE)+Copy Location）：
#   旋转模式切轴向会跳变；单轴旋转根本不在 gizmo 的本地轴上转。
#
#   实测（test_axis_diag / test_native_axis）：该骨骼的「约束偏移」
#   K = 可见姿态·(P·basis)⁻¹ 是 34°/1.8°/-68° + 0.88m 平移，绝不能被当成单位阵。
#     · 预览走的 P⁻¹RP（只按父级共轭）把 K 漏了 → 实际转轴偏掉一大截；
#     · 提交点 _flush_rotation 回滚用的是 P⁻¹·m_start，带约束时它 ≠ basis 原件，
#       实测差 76.117° → 切轴/切骨骼/左键确认那一下就是它在跳。
#   修法：写成 P⁻¹·K⁻¹·R_pivot·K·P（K=单位阵时精确退化成原算法），回滚一律写
#   basis 原件，提交枢轴跟预览对齐（都用骨骼头）。原生 transform.rotate
#   (LOCAL, center_override=头) 与这套公式实测误差 0.000°，所以提交仍交给原生。
#
#   v4.9.93 — 「撤销完轻移一下就跳回撤销前」的真正根因：曲线把姿态拽回去了
#
#   用户报障（真实工程 B 单手斩击 逆斜斩.blend，rig.016 / 动作「逆斜斩」/
#   第 24 帧 / 自动关键帧开）：右键撤销、中键清空之后再轻移一下，整个人跳回
#   撤销前的姿态。v4.9.92 清了累计量仍复现 —— 因为根因不在累计量。
#
#   实测（test_animtag / test_undo_user2）：
#     · 拖动时姿态只写在 matrix_basis，动作在当前帧仍留着上一次确认时的
#       关键帧值（frame 24 = 0.6190），拖动中的姿态根本没进曲线；
#     · 曲线一旦被重估（frame_set / 切帧 / 别处插帧 / 撤销重算），
#       matrix_basis 当场被写成曲线上的旧值 —— 实测 0.0067 → 0.6190；
#     · 下一次鼠标移动，`_ensure_baseline` 发现「姿态和基线不一致」，就地
#       重建基线 —— 等于把被拽回去的旧姿态**接受**成新基线，于是跳变定格。
#
#   两处修：
#     1. `_sync_keys()`：自动关键帧开着时，拖动中每 40ms 把姿态写进当前帧的
#        关键帧（撤销/清空/自愈时强制写）。姿态恒等于曲线值，重估不再有分歧。
#     2. `_ensure_baseline()`：不再接受被拽回去的姿态，而是把「我们最后一次
#        亲手写出去的姿态」(_last_pose) 写回来，再对齐基线 —— 画面不动。
#
#   v4.9.92 — 撤销 / 中键清空之后，动一下不再弹回撤销前
#
#   用户反馈：右键撤销、中键清空旋转之后，鼠标稍微一动，姿态又回到撤销前。
#   根因：撤销只把姿态写回去了，**「未落地的旋转累计」(_quat / _ang_h /
#   _ang_v / _mv_pending) 没清零**。旋转是每帧「基线 + 累计量」重算出来的，
#   累计量还在，下一帧 `_apply` 就把那个角度原样叠回新基线上 —— 数值上
#   正好等于撤销前的姿态，看着就是"弹回去"。
#   两条防线：
#     1. 撤销的每个分支（空栈 / 已在栈底 / 正常回退）都走 `_resync_after_undo`，
#        由 `_reset_gesture_accum()` 统一清零累计量。
#     2. `_ensure_baseline()`：累计量为 0 但姿态和 `_states` 对不上时，说明
#        姿态是被别的东西改的（撤销、中键清空、关键帧重估、外部脚本），
#        就地用当前姿态重建基线。挂在 mousemove 算 dx **之前**（累计量更新
#        之前）—— 放进 `_apply` 会因为累计量已经更新而永远判不出来。
#
#   v4.9.91 — 连切同一个集合不再"瞬间急速放大"
#
#   用户反馈：从饼菜单连着两次进同一个集合，第二次画面猛地贴脸。
#   根因在**兜底路径**：第二次进同一集合时视角已经停在"原生×余量"上，
#   frame_view_animated 判定"没差别"却返回 False，调用者于是走
#   native_view_selected 兜底 —— 原生框选是零留白的，视距当场从
#   原生×1.5 掉到 原生，就是"瞬间放大 1.5 倍贴到脸上"。
#   两处修：
#     1. "已就位"改为返回 True（调用者就不会再兜底）；容差也放宽到
#        视距的千分位 —— 原生每次都重算包围盒中心，1e-6 的阈值永远
#        过不去，那道闸等于没有。
#     2. 兜底改走 native_frame_selected()：同样乘余量、同样按住原生的
#        200ms 平滑动画，两条路的落点必须一致。
#
#   v4.9.90 — 集合取景不再贴脸（取景余量 + 最小视距）
#
#   用户反馈：从饼菜单跳进一个集合，自动缩放"框得太近"。根因是原生
#   view_selected 零留白 —— 选中骨骼的包围盒直接顶到视口边缘，而控制器
#   的可见外形（圆环/方块）比骨骼 head→tail 那根线大一圈，所以框完就贴边
#   甚至被裁；集合越小越明显（单根骨骼直接贴脸）。
#   做法：不改原生取景的朝向与中心，只在它算出的**距离**上乘一个可调倍率
#   （frame_margin，默认 1.5），再吃一次最小视距兜底（frame_min_dist，
#   默认 0.8）。两个值都在面板「快速设置」和插件偏好里可调。
#   透视和正交视口都靠 view_distance 控制缩放（ortho_scale 只属于相机），
#   所以倍率只乘一处就够，两种视口都生效。
#
#   v4.9.89 — 面板归类：骨骼的三块并到一起
#
#   骨骼信息（名字/集合/动作/排序/默认骨骼）后面紧接「选中骨骼动作」和
#   「手动锁轴」，中间不再夹「观察视角」。原来观察视角卡在骨骼信息和
#   选中骨骼动作之间，调一根骨骼的轴锁得跳过一整块看画面的功能。
#   新顺序：快速设置 → 骨骼信息 → 选中骨骼动作 → 手动锁轴 →
#   观察视角 → 饼菜单。
#
#   v4.9.88 — 隐藏集合能进 / 同一集合不再被拆成几段 / 观察视角书签
#
#   ① 跳进被隐藏的集合会报「集合里没有可用的控制器」：原来先按可见性过滤
#      再显示集合，隐藏集合的骨骼全被过滤光。改成先 show_collection 再过滤，
#      过滤空了就退回集合全量（collection_nav_names）。
#   ② 一个集合里的 13 根骨骼，滚轮只能在其中 5 根/8 根里转：nav_collection
#      原来遇到「一根骨骼属于多个集合」就取骨骼数最少的那个，于是同一批头发
#      被拆到「前发 / Hair / 马尾」几个范围里。现在按 进入的集合 → 上次的
#      范围（cn_scope）→ 最大的集合 走，而且 nav_neighbor_name 会先把范围
#      记下来，滚轮永远在同一份列表里循环。
#   ③ 「视图返回」升级成「观察视角」书签：面板上点「记住当前视角」把自己
#      挑的机位存在骨架上，之后按一个**可自定义**的键（偏好里选键 + 修饰）
#      缓动回去；没记过书签时仍然退回上一个视角。
#   ④ 面板顺序调整：观察视角 / 选中骨骼动作 / 手动锁轴挪到饼菜单**上面**，
#      三层饼各自可折叠（折叠后只留一行摘要）。
#
#   v4.9.87 — 修 v4.9.86 真机翻车：HAIR 图标在 5.x 不存在
#
#   UILayout.label(icon='HAIR') 抛 TypeError，面板从饼菜单那行整个截断，
#   控制台每次重绘刷屏（用户看到的"指定默认骨骼报错/视图返回没有"都是它
#   连带的）。新增 safe_icon()：按当前版本的图标表校验，没有就退回 NONE；
#   细节饼图标 HAIR → CURVES。另外视图快照下沉到 goto_collection ——
#   原来只有饼菜单跳转才记，滚轮切集合 BackSpace 没东西可回。
#
#   v4.9.86 — 三层饼 + 前缀分组 + 视图返回 + 集合默认骨骼
#
#   · 三层饼：8 格装不下一个完整角色，按动画师的使用频率分层 ——
#     主饼 Alt+左键（身体核心，肌肉记忆主场）/ 细节饼 Shift+Alt+左键
#     （眼耳发指）/ 附属饼 Ctrl+Alt+左键（装备衣尾道具）。三层槽位
#     各存一套骨架数据，主饼沿用旧键名，升级不丢已配好的布局。
#   · 前缀分组：集合多起来后按前缀归组（头发_01/头发_02 → 一组），
#     自动填充时组大的先占饼位；指定集合的弹窗里按组列小标题。
#   · 视图返回：每次饼跳转前把出发视角压栈，调完细节按 BackSpace
#     缓动弹回全景，连按逐级后退（pose.cn_view_back_v45）。
#   · 集合默认骨骼：面板里把选中骨骼设为所在集合的「默认第一根」
#     （存 cn_first_<集合名>），切进集合直接落在有用的控制器上，
#     比如腿部集合默认落脚部 IK，不再落在没用的骨骼上。
#
#   v4.9.85 新增 frame_view_animated()：先记下当前视角 → 跑一次原生
#   view_selected 读出"目标视角"→ 立刻还原 → 0.34 秒里用 smoothstep 把
#   view_location / view_distance / view_rotation 缓到目标。
#   框选精度和 Home 键完全一致（还是原生那套算法），动效自己掌控。
#   动画期间用 view_tween_busy() 挡住第二个动画，鼠标等动画结束再归位。
#
# v4.9.84 — 修掉饼菜单在 draw() 里写数据导致的崩溃与空饼
#
# v4.9.83 在真机上炸了，报的是：
#   AttributeError: Writing to ID classes in this context is not allowed:
#   rig.002, Armature data-block, error setting Armature.cn_pie_slot_0
#   位置在 面板 draw() -> pie_slots() -> init_pie_slots() -> rig.data[...] = n
#
# 根因：pie_slots() 是"首次访问时自动填一遍并写进骨架"的设计，而它被面板和
# 饼菜单的 draw() 调用。Blender 的 draw 上下文是**只读**的，写 ID 属性会被
# 直接拒绝。而且这个异常会在 layout 只构建到一半时抛出，后果是：
#   · 饼菜单只弹出一项（draw 被截断，剩下 7 个扇区根本没加进去）
#   · 面板那 8 行只剩空壳、看不到集合名
#   · 每次重绘都再抛一次，日志刷屏
#
# v4.9.84 四处修正：
#   1. 数据写入与绘制彻底分离。pie_slots() 现在是**纯只读**：没初始化就现场
#      算一份默认布局返回，不落盘（新增纯函数 pie_default_slots）。落盘只发生
#      在操作符里 —— freeze_pie_slots / init_pie_slots / set_pie_slot。
#   2. set_pie_slot 改一格之前，先把其余格子的当前显示值固化，否则"未初始化"
#      被打破的那一刻，别的格子会因为自动逻辑重算而悄悄移位。
#   3. 面板/饼里的"选集合"不再用 EnumProperty。集合名是中文，拿它当枚举
#      identifier 会在下拉里显示成 ó§Uè 那样的乱码（第一项还是空 identifier，
#      更要命）。改成 StringProperty + window_manager.popup_menu 自绘列表，
#      标识只走整数 slot，集合名只当普通字符串传。
#   4. 饼菜单 draw 改成"先把 8 项算成纯数据、再统一交给 layout"，中途任何一项
#      出问题都只让那一格变灰，不会把整张饼截断；饼项的 poll 统一收成
#      pie_menu_poll（只看有没有骨架），可编辑性检查挪到 execute 里 ——
#      pie 里 poll 失败的项会被整项丢弃，那是方位错位的直接原因。
#
# v4.9.83 — 集合饼菜单重写：8 方位固定、自己配、选完框住整组

#
# v4.9.83 饼菜单整块重写（旧的 PIE_NODES / cn_pie_map / 面板 8 个下拉全部删除）：
#
#   旧版的病根是"同一个东西维护三份"：
#     · 偏好里的「预设集合清单」驱动滚轮切集合、M 菜单、面板集合循环；
#     · 饼菜单却另写一套写死的 8 个部位节点（PIE_NODES），绑定另存一份
#       rig.data["cn_pie_map"]，名字还是两套（清单叫「头部」，饼里叫「头」）。
#     结果就是改清单饼菜单不动、换根骨架装完全是"未绑定"。
#     另外 PIE_NODES 每项带的坐标字段 (0,108)/(±200,58)/(±108,8) 全文件无人
#     读取 —— 上一版原本想自绘饼图排版，后来退回原生 pie，坐标成了死数据。
#
#   现在的做法：
#     1. 槽位固定 8 个，存在骨架自己身上（rig.data["cn_pie_slot_0..7"]），
#        跟骨架走，换 rig 各自配置。8 格永远全输出，空位放可点击的
#        "＋ 方位名"占位项 —— 空格也要能被点开去指定集合。
#     2. 一根新骨架第一次用饼时，按方位语义自动认集合（上=头、右上=右手指、
#        右=右臂、右下=右腿、下=躯干、左下=左腿、左=左臂、左上=左手指，候选词
#        中英文都有），剩下的空位拿预设清单补。认过之后一律以你的配置为准：
#        手改过、甚至整片清空过，都不会被自动逻辑覆盖回来
#        （rig.data["cn_pie_init"] 就是记这个事实的）。
#     3. 面板那 8 行改成"方位名 + 集合名"，点集合名直接弹可搜索列表换；
#        右上角两个小按钮：按方位重填 / 清空。
#     4. 进集合复用既有的 goto_collection（显示集合、选中第一根、记循环下标、
#        鼠标跟随），跳转前先把整组选中走原生 view_selected 框住 —— 和 Home 键
#        同一套算法，不再自己算包围盒，也不再另写一份 enter 逻辑。
#        进过的集合顺手加进循环清单，滚轮接着从这个集合切。
#     5. 饼项的 poll 不再沿用 pose_poll：饼绘制/执行的 context 未必带 mode，
#        一失败对应扇区会整个消失（表现就是"饼弹出来是空的"）。改成只看
#        "有骨架且可编辑"。
#
# Controller Navigator v4.9.31 — Alt+中键放行给原生「对齐正交视角」
#
# v4.9.31 一处改动（视角）：
#   模态里所有中键都被 _orbit_view 截走，Alt+中键（按住拖 = 对齐到最近的正交
#   视角，Alt+点击 = 拾取对焦）整个失效。现在 Alt+中键的按-拖-放全部
#   PASS_THROUGH 还给 Blender 自己的键位（view3d.view_axis CLICK_DRAG /
#   view_center_pick CLICK —— 这俩是键位层合成的 CLICK_DRAG/CLICK 事件，
#   PRESS 被模态吃掉就永远不会形成，所以必须从 PRESS 起就放行）。
#   原生那层退场后：下一帧 MOUSEMOVE 只取参考点、指针归位、旋转基线冻结、
#   拖动方向重算（_after_view_change），接着拖是连续的。
#   顺带把中键导航结束后的 _refresh_projection 补上了（此前中键旋转视角后
#   _dir_win 一直是旧的，接着单轴/双轴拖会不跟手）。
#
# v4.9.30 — 自由旋转换成原生 gizmo 的角度曲线（短距离也能大角度）
#
# v4.9.30 一处改动（旋转灵敏度/曲线，直击「原生拖一小段 = 大变化，插件要拖很远」）：
#   老版自由旋转是线性 °/px，而且基准还按骨骼屏幕投影长度归一：
#   腿/躯干占半屏时只有 0.15~0.25°/px，转 90° 要拖 400~600px；原生 gizmo
#   拖 100px 就过 90°。把倍率拉到 5 也只有 0.75~1.2°/px，而且线性意味着
#   「小角度想稳、大角度想快」这两个需求永远打架 —— 这就是「调灵敏度效果一般」。
#   现在改成原生语义：绕枢轴的球面映射
#       angle = 2·atan(位移 / 球半径)     （球半径 = 偏好 ring_zone，默认 110px）
#   0px→0°，55px→53°，110px→90°，220px→127°，470px→154°：
#   起手极跟手、越往后越收敛，正是原生 gizmo 那条曲线，靠倍率复现不出来。
#   双轴 / 单轴（线性直线拖）的基准同步重锚到原生量级（约 0.85°/px，不再按
#   骨骼长度大幅衰减），所以三种旋转模式的手感现在是同一个量级。
#   平滑、吸附、微调、活动半径、撤销栈全部保留；HUD 右上角新增实时 °/px 读数，
#   调灵敏度时能看到实际生效值。
#
# v4.9.29 — 右键撤销能一路退回上一个控制器，且不再退出模态
#
# v4.9.29 两处改动（撤销范围/退出条件）：
#   1. 快照从「当前选中的骨骼」扩成「整段会话碰过的所有骨骼」(_session)，
#      每步还额外记下当时的焦点控制器 / 目标骨骼 / 模式。跨控制器的那一步
#      因此能完整还原：滚轮切到下一根转完再右键，退回去的是上一根连同它的姿态，
#      焦点、选中、solo 可见集一起切回上一个控制器，接着就能继续调它。
#   2. 右键不再退出模态。退到进入模态时的状态只提示一句「已退到进入模态时的状态」，
#      退出只认左键确认 / Esc —— 原来"再按一下想退一格"会直接掉出模态。
#      同时按 1e-4 容差跳过和当前姿态相同的空步骤（原生 rotate 落地后的尾数漂移
#      会生成这种"按一下没反应"的步骤），右键每一下都真的退一格。
#   说明：这里没直接调 bpy.ops.ed.undo()。原生撤销会把正在跑的模态操作符一并
#   取消（undo 会清空 operator stack），一按右键就掉出模态，正是要修的问题。
#   所以姿态落地用原生 rotate / translate，撤销的记账按原生 undo 的语义自己做。
#
# v4.9.28 单轴旋转改成「原生 rotate + 直线拖」，不再画圈
#
# v4.9.28 两处改动（单轴手感）：
#   1. 输入：v4.9.24 的环绕式（指针绕枢轴转多少度骨骼就转多少度）对脊柱这类
#      枢轴埋在身体里的骨骼很别扭——圈要绕过角色、稍快就甩过头，而且灵敏度随
#      半径变。现在改回线性：位移投影到该轴的切向方向，按固定 °/px 累加，
#      沿着轴推几乎不转。手柄感与「R + 轴」约束后一致，也和移动模式同一套直觉。
#   2. 落地：单轴的角度由**原生 transform.rotate** 执行（orient_type='LOCAL' +
#      constraint_axis + center_override=骨骼头）。实测该组合与插件原矩阵算法
#      等价（差 < 1e-3°、父级补偿一致），所以姿态数值不变，算子是真原生。
#      每帧仍写 matrix_basis 做预览（流畅、不刷 undo），到提交点（切骨骼/切轴/
#      切模式/左键确认）再回滚到基线、用原生 rotate 一次转完；原生失败就保留
#      预览结果，行为退回到改动前。多骨骼目标不走原生（各骨骼枢轴不同）。
#   活动半径夹持（ring_zone）现在对自由/双轴/单轴一律生效，指针不再跑远。
#
# v4.9.27 活动半径覆盖自由旋转（上一版只有单轴生效）
#
# v4.9.27 一处修正：v4.9.26 的活动半径只写在单轴环绕式里（_single_axis_step），
#   而你实际用的是**自由旋转**（HUD 显示「自由旋转 +xx°」），那段代码根本没走，
#   所以指针照样跑到 300px 外。现在抽成通用 _confine_cursor()，在模态 MOUSEMOVE
#   里对自由旋转 / 双轴 / 投影式单轴统一生效：
#   - 只夹位置、不动旋转量：指针沿半径拉回圆周，同时把 _ref 对齐到新位置，
#     下一帧 dx/dy 仍是你手的真实位移，旋转继续累计，指针留在控制器附近；
#   - 贴边时 HUD 显示「指针已贴活动半径」，避免"指针不动但还在转"的困惑；
#   - 环绕式单轴仍由 _single_axis_step 内部夹（它要用夹后的方位角算角度）。
#
# v4.9.26 旋转活动半径 + 右键「退一步」而不退出模态
#
# v4.9.26 两处改动：
#   1. 旋转活动半径（偏好 ring_zone，默认 110px）：单轴旋转时指针最多离开控制器
#      这么多像素，顶到边界就沿半径夹回圆周、方位角不变（角度完全连续），
#      手可以一直往一个方向推，指针始终留在控制器附近 —— 确认后能马上抓到 gizmo。
#      0 = 不限制。径向推不产生旋转，只有绕圈的切向分量才记角度。
#   2. 右键 = 退回上一步，仍留在旋转/移动模式里，可以连续按一路退。步骤栈在
#      提交点（切骨骼 / 切集合 / 切模式 / 切轴）记账，姿态没变不重复记。
#      ESC = 整个放弃（回到进入模态时的姿态）并退出；左键 = 确认并插关键帧。
#      退回后重新对齐 _states / 起点 / 轴心，接着拖是从退回去的姿态继续。
#
# v4.9.25 右键撤销改「触碰名单」制，结构性杜绝连坐
#
# v4.9.25 一处改动（撤销范围，v4.9.23 的加固）：
#   v4.9.23 靠「提交点刷新 _orig 快照」收窄右键回滚范围，实测仍有连坐反馈。
#   根因是那条路依赖提交时序永远正确，任何一环漏刷快照，右键就会按过期
#   快照把旧骨骼卷回去。现在改成结构性保证：
#   - 新增 _touched「本段触碰名单」：进入模态时=当前选中的骨骼；
#     切骨骼 / 切集合（含 Shift 加选）= 提交点，名单直接换成新骨骼；
#     进入移动时把参与移动的骨骼并入名单。
#   - 右键 / ESC 只回滚名单里的骨骼 —— 上一段提交过的骨骼根本不在名单里，
#     就算 _orig 快照过期也绝不会被连带撤掉。
#   - 没切过任何东西时名单就是全部选中骨骼，右键仍撤当前全部（原行为不变）。
#
# v4.9.24 一处改动（单轴旋转手感）：
#   旋转的单轴模式原来用「位移投影换角度」：拖动方向和轴的屏幕投影夹角越大越
#   不跟手。现在改成环绕式：以枢轴的屏幕投影为圆心，指针绕它转多少度，骨骼就
#   绕该轴转多少度（1:1，gizmo 手感）。方向按右手定则：轴指向观察者时逆时针
#   为正，背离则相反。细节：
#   - 轴大致指向/背离屏幕（|轴向量和视线夹角分量|>0.2）才用环绕式；
#     轴躺在屏幕平面里时圈变成一条线，自动退回旧投影式。
#   - 跨 ±180° 用 unwrap 累计，连转多圈不断；Shift 微调仍 ×0.15，Ctrl 吸附不变。
#   - 指针瞬移（边缘回绕、转视角后归位）只重取参考角，不会突跳；
#     枢轴屏幕点挪动超过 4px（= 视角变过）那一帧也不算输入。
#   - 两轴（DUAL）和自由（TRACKBALL）保持原逻辑。
#
# v4.9.23 两处改动（撤销范围）：
#   1. 滚轮切到下一个控制器再右键，会把上一个控制器的旋转一起撤掉——
#      原因：_orig 快照一直是「进入模态那一刻」，右键按它整体回滚。
#      现在 _switch_bone / _switch_collection 切走前把旧骨骼的 _orig
#      更新成切换这一刻的姿态（旧骨骼已插关键帧、算提交）。
#   2. 旋转 → 切移动 → 移动没弄好右键，旋转也一并没了——同一根源。
#      _rebase（切模式/切轴的提交点）现在也把 _orig 更新成当前姿态。
#      语义从此固定：**每次切骨骼/切集合/切模式/切轴 = 提交点，
#      右键只撤销最近一段**；什么都没切时右键仍撤当前骨骼的全部操作。
#   注意：切到头滚不动时不会提交（右键仍可撤销当前这次操作）。
#
# v4.9.22 三处改动：
#   1. 模态里切模式原来写死 G（移动）/ R（旋转），和偏好里设的 X / Z 对不上：
#      旋转中按 Z 只会被当成「锁 Z 轴」，切不到移动。现在读偏好里的
#      rot_key / move_key（默认 X / Z），修饰键也按偏好严格匹配；
#      模式切换的判断放在切轴前面，所以旋转中按 Z = 切移动，移动中按 X = 切旋转。
#   2. 面板里的「手指自动轴」整块删掉（只留「手动锁轴」），底层 _finger_auto_unlocked
#      一并停用，免得看不见开关却还在背后锁轴。
#   3. 面板顶部加展开/收拢箭头（偏好 panel_open，重启保持）：收起后只剩一行标题，
#      面板太长的问题解决。
#
# v4.9.21 三处改动：
#   1. solo 上一版是空操作：写的是 Bone.hide，而它的语义是「编辑模式下不可见」，
#      姿势模式里完全无效。改用 PoseBone.hide（「除编辑模式外不可见」）才真的隐藏。
#   2. 模式切换的继承做成可见：HUD/顶部显示「累计」旋转角度与位移，跨模式切换不清零，
#      切完还能看到之前一共调了多少。数值本身早已保留（_rebase 冻结基线）。
#   3. 模式切换包了 try/except：切失败只报提示、不把整个模态踢掉（原来异常=直接退出）。
#
# v4.9.20 三处改动：
#   1. 「多按几下 X/G 屏幕上堆满提示、右键要按好几次才退出」——模态叠加。
#      模态里返回 PASS_THROUGH 的按键会继续走键位映射，而 X/Z/G 本身就是进入键，
#      于是原地又开一个模态叠上去（每个实例一份 HUD，各自画在当时的指针位置）。
#      修复：模块级 _MODAL_ONCE 闸门，invoke 发现已有实例在跑就直接 CANCELLED；
#      关掉 X/Y/Z 开关时这两个键改为静默吃掉，不再放行。
#   2. solo 显示 / X/Y/Z 开关也放到 3D 视口侧栏（原来看不到，只在插件偏好页里）。
#   3. 闸门在 _clear / register 都会重置，任何退出路径都不会把模式卡死。
#
# v4.9.19 两处改动：
#   1. solo 显示：进入旋转/移动模态后其它骨骼全部临时隐藏，只留当前控制器
#      （多选时留所有选中的）；滚轮切骨骼/切集合时可见集跟着换，结束模态
#      自动按快照恢复。只写 bone.hide，纯显示。调原生 ops（transform.translate）
#      前会先恢复显隐，避免隐藏状态被记进撤销步骤。
#   2. X/Y/Z 键切单轴做成偏好开关 use_xyz_keys（默认开）。关掉后旋转/移动模态里
#      X/Y/Z 放行给 Blender 原生，只能用滚轮切换轴向；HUD/header 提示同步更新。
#
# v4.9.18 三处改动：
#   1. 「移动完切旋转，移动清零；旋转完切移动/换轴，旋转清零」——根因是旋转每帧都
#      从 _states（进入模态那一刻的快照）重算整根骨骼的姿态。中途做了别的变换以后，
#      这份快照就过期了，下一帧重算把那次变换整个抹掉。现在新增 _rebase()：切模式、
#      换轴之前先把当前姿态写成新基线、角度归零，新动作在它上面叠加。
#   2. 顺手修掉同一族的隐性跳变：_apply() 里「角度为 0 / 该轴被锁」时把 armature 空间
#      的姿态矩阵直接写进 matrix_basis，会多叠一份父级偏移（骨骼整根跳开）。
#      现在统一走 _restore_to_start()，经 parent_m 反解。
#   3. 中键操作完视角，把指针送回当前控制器（偏好可关），落点与导航跟随共用
#      warp_dx/dy 偏移。原生导航是模态操作符、会一直吃到松手，所以归位延迟到
#      本模态重新收到事件时才做，并带一个 80ms 的时间下限兜底。
#
# v4.9.17 三处改动：
#   1. Shift+中键 = 原生视图平移，Ctrl+中键 = 原生缩放。v4.9.15 只把中键交给了
#      view3d.rotate，Shift+中键被一起吃成旋转，平移用不了。
#   2. 转完视角再旋转会剧烈扭动：_trackball_step 用「跨视角累计的位移 × 当前视角的
#      屏幕上/右方向」重算绝对旋转，视角一变，同一份累计位移被新坐标重新解释，
#      姿态直接甩到别处（实测 40/25px 拖完再转 60°，旧代码跳 23.3°）。
#      现在转完视角把当时的旋转冻结进 _quat0、累计位移归零，新拖动在 _quat0 上叠加。
#   3. 手感变飘：_refresh_projection 里连带重算了 _deg_per_px（按骨骼屏幕投影长度
#      归一），视角一变投影长度就变，同一个位移算出的角度跟着变。现在只重算拖动
#      方向 _dir_win，灵敏度保持进入模态时的值。
#
# v4.9.16 改动：
#   HUD 是 SpaceView3D 的 draw handler，只有视口重绘才会跑。换轴/换骨骼/切模式时
#   没有鼠标移动 → 不重绘 → 文字停在旧值，必须拖一下才更新。现在 _header() 里统一
#   tag_redraw，任何一个状态变化都会立刻反映到鼠标旁的文字和视口顶部。
#
# v4.9.15 改动：
#   中键原来在模态里是「确认」，和「边调边转视角看效果」直接冲突。
#   现在中键在模态内嵌套调用原生 view3d.rotate：按住中键拖动 = 转视角，
#   转完自动回到本次模态，姿态、轴向、累积位移全部保留，接着拖是连续的。
#   确认统一交给左键（右键 / Esc 仍是撤销）。
#   转完视角后屏幕投影变了，会重算旋转手势的方向与灵敏度；角度跟着轴走，
#   不跟着水平/垂直通道走，所以不会因为通道换轴而跳变。
#
# v4.9.14 三处改动：
#   1. 纵向方向反了：模态里的 dy 是「向下为正」（dy = -(mouse_y 增量)），而视口的
#      up 向量和屏幕投影都是「向上为正」，v4.9.13 直接把 dy 乘上去 → 鼠标往上骨骼往下。
#      现在先翻成 up_px 再算，自由移动和单轴移动都跟着修正。
#   2. 移动不再由鼠标中键进入：中键恢复成 Blender 原生视图旋转，只在控制器边上想转视角
#      也不会被抢走。改为按键进入，键和修饰键都在 插件偏好 > 旋转/移动进入键 里改，
#      改完立即重新绑定，不用去 Keymap 里手动绑。
#   3. 模态内部固定用 G / R 互相切换（HUD 里有提示），进入键换掉也不影响。
#
# 滚轮切轴未改动：向下推一格 = 下一根轴，自由 → X → Y → Z → 自由。
#
# v4.9.13 三处改动（本版重点）：
#   1. 移动模式「根本移动不了」的根因：逐帧增量被反复乘到同一个起始矩阵上
#      （pb.matrix = T(本帧增量) @ 起始矩阵），总位移永远等于最后一帧的那点像素，
#      骨骼只在起点附近抖，看起来完全不动。现改为累积位移 _mv_pending，
#      再统一落地。实测 100px 拖动 = 0.6066 单位（视距 5 / 高 600px）。
#   2. 移动改由原生 transform.translate（= G 键同一个操作符）落地：
#      拖动过程中用 matrix_basis 做零成本预览，确认/换轴时回滚基准再交给原生 G，
#      所以最终结果走的是 Blender 自己的 transform 通道（约束、驱动、自动关键帧、
#      多骨骼选择关系全部按原生规则处理），undo 只留一步，不会刷满撤销栈。
#   3. 中键进入判不准：原来只认「活动骨骼 head 附近 30px」，而控制器可见的是
#      custom shape 形状、形状常常远离骨骼线段，于是多半判不中 → 变成视口旋转。
#      现在同时测「骨骼屏幕线段」和「形状屏幕包围球」，并在所有骨骼里挑最近的，
#      点到哪个控制器就抓哪个（顺带把它设为活动骨骼）。
#
# 滚轮（旋转、移动两套模式一致）：向下推一格 = 下一根轴
#   自由 → X → Y → Z → 自由；向上推一格 = 反向。
#   移动的轴向是全局轴，下标直接对应 location / lock_location 的下标
#   （v4.9.12 里用的是 ROT_AXES 顺序 X/Z/Y，选到「Z」实际动的是 Y）。
#

# v4.8 新增：按 X 进入 = 直接进入 gizmo 那种自由旋转手感（本版重点）
#   不用对准 gizmo 环，选中控制器按 X，鼠标一动就开始转，跟按住旋转 gizmo 拽一样。
#   手势按骨骼的旋转轴锁（lock_rotation）自动分级，骨骼允许转什么就给什么：
#   · 三轴全开  → 自由 trackball：跟按住 gizmo 中心拽的手感一致
#   · 只开两轴  → 双轴分通道（手指类常这样）：水平拖动 = 绕一根轴，垂直拖动 =
#     绕另一根，两个通道同时可动、互不干扰，一次拖动同时调两个自由度
#   · 只开一轴  → 怎么拖都只转那一根
#   分级结果和「水平→哪根轴 / 垂直→哪根轴」都画在指针旁 HUD 上，没有隐藏状态。
#   轴分配的依据是真实的屏幕投影：把每根未锁轴转 2° 看末端往屏幕哪个方向动，
#   最接近水平的分配给水平通道、最接近垂直的给垂直通道，视角变了分配也跟着对。
#
# v4.7 已有（继续沿用）：
#   灵敏度按骨骼屏幕投影长度归一（手指和大腿手感一致）、Ctrl 吸附 5°/15°/45°、
#   光标旁 blf HUD、指针贴边自动回中、未处理按键 PASS_THROUGH 不抢原生快捷键。
#   v4.7 的「自动判轴 + 自动换轴」在本版移除：双轴同时可动之后不再需要换轴，
#   也就没有了「现在转的是哪根轴」这个隐藏状态。
#
# v4.5.2 已有：见下方历史（rigUI 协作、IK/FK、集合循环、鼠标跟随等）
#
#
# 工作方式：你自己把控制器归到骨骼集合（Bone Collection）里，
# 选中任意一个控制器后，↑ ↓ 就只在它所属的那个集合里循环。
#
# v4.5.1 新增：
#   1. 补上 "Super charged UI bone collections" 的 tooltip 中文；翻译改为面板绘制时再应用
#   2. 切 IK/FK 自动同步 Arm/Leg 的 (IK)/(FK) 骨骼集合可见性
#   3. 去掉独立 Controller Nav 面板，改为注入 rigUI 弹窗与侧栏面板
#
# v4.5 新增：Rigify 一键 IK/FK（适配自定义 UI 文本名）
#   不再只找 rig_ui.py：扫描所有 Text，找到含当前 rig_id 的 rigify snap 定义，
#   解析出肢体参数，直接调用 Rigify 原生 snap 再设 IK_FK，无需手动配置。
#   帮助弹窗正文（IK/FK、骨骼集合）运行时替换为中文。
#
# v4.5 已有：rigUI 协作
#   1. Rigify 原生 IK/FK：解析 rig_ui.py 里 Rigify 自己生成的 snap 按钮参数，
#      直接调用原生操作符，再设 IK_FK；不需要手动填骨骼对，切换不跳。
#   2. 骨骼集合弹窗：鼠标移出 0.35 秒后自动关闭（可在偏好里关）。
#   3. Rig UI 面板 tooltip 中文化：只覆盖 bl_description 显示，不改逻辑；
#      卸载时恢复英文。覆盖 60 条可见提示。
#
# v4.5 已有：两处修正
#   1. M 菜单 / 「－ 当前集合」改为按“当前选中骨骼真正所在的集合”取名字，
#      不再用之前锁定的旧集合（选中左手指却显示右手指的问题）
#   2. 循环下标改为基于“完整预设清单顺序（含空集合）”。
#      某个集合被清空/加回骨骼时顺序不会重置；空集合只是被跳过。
#
# v4.5 已有：视口跟随
#   切换到的控制器不在当前 3D 视口内时，用 0.28 秒 smoothstep 缓动把视口平移过去，
#   结束后再把鼠标移到该骨骼；已经在画面里则完全不动视口。
#   偏好里可关（「视口外自动平滑移过去」）。
#
# v4.5 已有：三处修正
#   1. 集合切换严格按预设清单顺序：记录循环下标，不再受“骨骼同属多个集合”影响
#   2. 手指集合按 拇指→食指→中指→无名指→小指 排序，每指内按编号自然排序（02 在 10 前）
#   3. 快速旋转换轴默认“不继承”当前轴：按 X / Shift+滚轮会退回本轴开始前的姿态；
#      只有 Shift+左键 才把当前轴定下来、切下一轴继续
#
# v4.5 已有：旋转中滚轮直接切关节
#   手指这种一串关节：按 X 进入旋转 → 转完一节 → 滚轮直接切下一节，
#   仍在旋转模式，接着移动鼠标就转下一节，不用退出再按 X。
#   滚轮切的是导航列表（同一集合、同样的 IK/FK 与可见性过滤），会绕回。
#   换轴改为 Shift+滚轮 或按 X；Shift 仍是微调；Shift+左键仍是“确认并下一轴”。
#   右键/Esc 会把这轮改过的所有骨骼一起还原。
#
# v4.5 已有：快速旋转（X）—— 尊重旋转锁定、Shift 微调、滚轮换轴、Shift+左键连续多轴
#   导航到控制器后按 X：进入旋转模态，左右移动鼠标直接改角度；
#   再按 X 换轴（X → Z → Y 循环）；滚轮临时调灵敏度；左键确认、右键/Esc 取消。
#   旋转发生在骨骼自身坐标系里（等价于 R → X → X），只作用于当前活动骨骼。
#   偏好里可关 X 绑定、可调灵敏度（度/像素）。
#
# v4.5 已有：自动扫描自定义 rig 的 IK/FK 开关
#   RIG-Klukai 这类自定义 Rigify，开关不在 upper_arm_parent 上，而是挂在
#   Wrist_L / Ankle_L 之类的骨骼上。v4.5 会扫描整条 rig 上所有带
#   IK_FK / ik_fk_switch 的骨骼，按「侧别 + 手臂/腿关键字」打分选最合适的那个。
#   另外，找不到已知命名时也用它兜底，所以自定义 rig 也能正确按 FK/IK 过滤。
#
# v4.5 已有：移出集合 + Rigify 以 rig 状态为准
#   · M 菜单里新增：从当前集合移出 / 从所有集合移出（面板「集合循环」也有按钮）
#   · IK/FK 判定优先级改为「rig 的开关属性 → 骨骼名标记 → 上次模式」：
#     Rigify 里 IK/FK 两套同时显示，点到 FK 骨骼但手臂实际由 IK 控制时，
#     现在会按 rig 状态只走 IK，不会滚到不控制手臂的 FK 上
#
# v4.5 已有：Rigify 适配
#   · IK/FK 判定同时支持 Rigify：upper_arm_parent.L / thigh_parent.L 上的 IK_FK
#     （0 = IK，1 = FK），集合名 Arm.L (IK) / Leg.L (FK) 也能识别手臂还是腿
#   · 骨骼名的侧别支持大写 .L / .R
#   · 新按钮「导入骨架集合」：把当前 rig 的可见集合一键加进循环清单
#     （跳过 MCH/ORG/DEF/Widget 等内部集合，Rigify 直接能用）
#
# v4.5 已有：肩膀/无标记骨骼的 IK/FK 判定
#   c_shoulder.l 这类名字里没有 _ik/_fk 的骨骼，现在会去读 Auto-Rig Pro 的
#   ik_fk_switch 属性（0 = IK，1 = FK）来判断当前是哪套：
#     · 从肩膀滚动时，只在当前那套控制器里走，不会跳到隐藏的另一套
#     · 无标记骨骼在 IK/FK 两边都保留，所以从手臂能往上走到肩膀，再回来
#   读不到属性时，退回「上次导航用过的模式」，再不行就不做模式过滤。
#
# v4.5 已有：跨集合移走前确认
#   用「全选 FK」之类的操作后，选中的可能包含左右两侧或另一套控制器；
#   这时直接点「右臂」会把左臂的骨骼也搬走（左臂 7 → 4）。
#   现在归组前会弹窗列出「左臂 里的 3 根会被移走」，确认后才动手。
#
# v4.5 已有：IK/FK 模式过滤
#   同一个集合里同时放了 IK 和 FK 控制器时，按 IK/FK 键切到哪一边，
#   导航（↑↓ / Alt+滚轮）就只在同一边的控制器里转，不会再跳去隐藏的那套。
#   判断依据：骨骼名里的 _ik / _fk 标记（极向 pole 控制器算 IK）。
#   名字里没有标记的骨骼（肩、根等）不受影响，照常参与导航。
#   同时把「跳过隐藏骨骼」加强为：骨骼自身隐藏，或它所在的所有集合都隐藏。
#
# v4.5 已有：集合循环
#   调完躯干想接着调手臂，不用再去点骨骼集合列表：
#     · Ctrl + Alt + 滚轮（默认，可在偏好换）切到清单里的下一个 / 上一个集合
#       默认用三个键的组合，是为了不和 Ctrl+滚轮 / Alt+滚轮 这些常用绑定打架
#     · 切换时自动显示该集合、选中里面第一根骨骼（鼠标跟随也生效）
#     · 空集合自动跳过，不会卡在还没放控制器的集合上
#     · 面板「集合循环」区可以：＋当前集合 / －当前集合 / 点名字直达
#   循环清单直接复用「预设集合」清单，不用维护两份。
#
# v2.7 已有：鼠标跟随 + 落点偏移
#   ↑ ↓（或自定义导航键）跳到某个控制器时，鼠标指针自动移到该骨骼在视口里的位置。
#   落点是骨骼头部（Gizmo 正中心），可用「落点偏移 X / Y」把它挪到旋转环或位移轴上：
#   例如偏到外圈就是旋转环，偏到某条轴上就是对应位移手柄。面板上可直接改数值。
#   骨骼投影在视口外时自动跳过，不会把鼠标甩到奇怪的地方。
#   偏好里可关（「鼠标跟随」），面板上也有开关和一个「鼠标→当前骨骼」按钮。
#
# v2.5 已有：快捷键可配置（解决「滚轮抢了视图缩放」）
#   Blender 的滚轮缩放是写死的导航行为，不在 Keymap 里，所以只要在 Pose 键位
#   里绑了纯滚轮，Pose 模式下的缩放就被这个绑定吃掉。v2.5 把导航键做成偏好选项：
#     · ↑ / ↓ 方向键（默认，完全不碰滚轮）
#     · Ctrl + 滚轮 / Alt + 滚轮（纯滚轮仍然是缩放，只占带修饰键的那一个）
#     · PageUp / PageDown、[ / ]
#     · 滚轮（不推荐，会占满缩放）
#     · 自定义（不自动绑，自己去 Preferences > Keymap 里绑）
#   另外带一个「清掉旧版本残留快捷键」按钮，用来扫掉 v2.1–v2.4 留在键位表里的滚轮绑定。
#
# v2.4 已有：预设集合
#   - 偏好里维护一份「预设集合名」清单（默认：躯干 / 头部 / 左臂 / 右臂 / 左腿 / 右腿）
#   - 面板一键「创建全部预设」，在骨架里补建这些空集合（已存在的跳过，不动里面的骨骼）
#   - 选中若干控制器后：
#       · 点面板上的集合名      → 直接归入该集合（不存在的自动新建）
#       · 按 M                  → 弹出预设菜单选一个归入
#       · 菜单里「新建集合并归入…」→ 输入名字，等价于原生 M 的新建+重命名流程
#   - 「移动」还是「复制」可在偏好里切换（默认移动：从其他集合移出）
#
# v2.3 已有：空间排序可以按集合开启 ——
#   偏好里填集合名关键字（逗号分隔多个，如「躯干,头」），
#   只有名字匹配的集合按空间上下排序，其余集合完全按你 assign 的顺序循环。
#   关键字留空 = 所有集合都空间排序（v2.2 的行为）。
#
# 与旧版共存：本版本使用独立的 operator / 面板 ID（_v45 后缀），
# 不会覆盖 v2.1 / v2.2 / v2.3 / v2.4，但多个版本不要同时启用（键位会抢）。
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
    "version": (5, 0, 4),
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
import sys
import time

import bpy
from mathutils import Matrix, Vector, Quaternion
from bpy_extras.view3d_utils import (location_3d_to_region_2d,
                                     region_2d_to_vector_3d,
                                     region_2d_to_origin_3d)

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

NAV_KEY_LABELS = {key: label for key, label, _desc in NAV_KEY_ITEMS}

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
    ('BOOK_GOTO', "回到观察视角", "没记过书签就退回上一个视角"),
    ('VIEW_BACK', "视角逐级后退", ""),
    ('IKFK_TOGGLE', "IK/FK 切换", "选中 IK 或 FK 控制器后按"),
    ('IKFK_POPUP', "IK/FK 面板", "弹出每个肢体一行的切换面板"),
)

KB_LABELS = {a: lbl for a, lbl, _d in KB_ACTIONS}
KB_DESCS = {a: d for a, _lbl, d in KB_ACTIONS}

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
        'VIEW_BACK': ('BACK_SPACE', False, False, False),
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
    out['BOOK_GOTO'] = (str(pref('book_key', 'BACK_SPACE') or 'BACK_SPACE'),) \
        + _old_mod_flags(pref('book_key_mod', 'NONE'))
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



def kb_reason(action):
    """和别的动作撞车时返回原因，用来在面板上提示。"""
    spec = kb_spec(action)
    if spec is None:
        return ""
    for other, lbl, _d in KB_ACTIONS:
        if other == action:
            continue
        if kb_spec(other) == spec and lbl:
            return "与「%s」重复" % lbl
    return ""


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


def nav_key_label():
    return NAV_KEY_LABELS.get(pref('nav_keys', 'ARROWS'), "↑ / ↓")


# 集合循环键（和骨骼导航键分开，避免互相抢）
COLL_KEY_ITEMS = (
    ('CTRL_ALT_WHEEL', "Ctrl + Alt + 滚轮", "默认：切换集合（不与 Ctrl+滚轮 / Alt+滚轮 抢）"),
    ('CTRL_WHEEL', "Ctrl + 滚轮", "切换到下一个 / 上一个集合"),
    ('ALT_WHEEL', "Alt + 滚轮", "切换到下一个 / 上一个集合"),
    ('PAGE', "PageUp / PageDown", "切换到下一个 / 上一个集合"),
    ('NONE', "不绑定", "只用面板按钮切换集合"),
)
COLL_KEY_LABELS = {key: label for key, label, _d in COLL_KEY_ITEMS}

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


def coll_key_label():
    choice = pref('collection_keys', COLL_KEY_DEFAULT)
    if choice == 'NONE':
        return "不绑定"
    if coll_key_specs() is None:
        return "未绑定（与导航键冲突）"
    return COLL_KEY_LABELS.get(choice, "Ctrl + Alt + 滚轮")


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


# ---- 观察视角（书签）快捷键

BOOK_KEY_ITEMS = (('BACK_SPACE', "BackSpace", "原来的回退键，不占别的功能"),) \
    + KEY_ITEMS


def book_key_spec():
    spec = kb_spec('BOOK_GOTO')
    if spec is not None:
        return spec
    key = str(pref('book_key', 'BACK_SPACE') or 'BACK_SPACE')
    mod = str(pref('book_key_mod', 'NONE') or 'NONE')
    kw = {}
    if mod == 'CTRL':
        kw['ctrl'] = True
    elif mod == 'SHIFT':
        kw['shift'] = True
    elif mod == 'ALT':
        kw['alt'] = True
    return key, kw


def book_key_label():
    key, kw = book_key_spec()
    # key_label 会 key.title()，把 BackSpace 变成 "Backspace"；这里自己拼
    pretty = "BackSpace" if key == 'BACK_SPACE' else key
    parts = [name for flag, name in (('ctrl', "Ctrl"), ('shift', "Shift"), ('alt', "Alt"))
             if kw.get(flag)]
    parts.append(pretty)
    return " + ".join(parts)


# ---------------------------------------------------------------- 选中

def select_bone(rig, bone_name, extend=False):
    """选中骨骼。extend=False 单选，extend=True 加选。"""
    pb = rig.pose.bones.get(bone_name)
    if pb is None:
        return False
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


def view_back_push(context, rig):
    """跳走之前把当前视角压进「返回栈」。栈顶和当前视角几乎一样时不重复压
    （连跳两次不会把全景冲掉），栈深上限 32，足够一整个镜头的工作量。"""
    space = getattr(context, "space_data", None)
    rv3d = getattr(space, "region_3d", None) if space is not None else None
    if rv3d is None or rig is None:
        return
    try:
        st = (rig.name,
              rv3d.view_location.copy(),
              float(rv3d.view_distance),
              rv3d.view_rotation.copy())
    except Exception:
        return
    if _VIEW_BACK_STACK:
        p = _VIEW_BACK_STACK[-1]
        try:
            if (p[0] == st[0] and (p[1] - st[1]).length < 1e-4
                    and abs(p[2] - st[2]) < 1e-4):
                return
        except Exception:
            pass
    _VIEW_BACK_STACK.append(st)
    del _VIEW_BACK_STACK[:-VIEW_BACK_STACK_MAX]


def view_back_pop():
    return _VIEW_BACK_STACK.pop() if _VIEW_BACK_STACK else None


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
    names = [b.name for b in col.bones if rig.pose.bones.get(b.name) is not None]

    mode = collection_sort_mode(rig, col)
    if mode == 'HIERARCHY':
        return hierarchy_names(rig, col)
    if mode == 'ASSIGN':
        return names
    if mode == 'FINGER':
        if any(finger_rank(n) < 99 for n in names):
            names.sort(key=lambda n: (finger_rank(n), natural_key(n)))
        return names
    if mode == 'SPATIAL':
        names.sort(key=lambda n: bone_key(rig, n), reverse=True)
        return names
    return names


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


def bone_visible(pb):
    """骨骼当前是否可见：自身没隐藏，且至少有一个所在集合是可见的。

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
        pb = rig.pose.bones.get(n)
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


def pie_usable_collections(rig):
    """饼菜单/列表里能选的集合：有骨骼、且不是 MCH 那类内部集合。"""
    out = []
    if rig is None or getattr(rig, "type", None) != 'ARMATURE':
        return out
    try:
        for c in rig.data.collections:
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
        cols = list(rig.data.collections)
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


def missing_preset_rows(rig):
    """偏好预设里、这根骨架还没有的集合 —— 点了会现场新建。"""
    out = []
    try:
        have = set(c.name for c in rig.data.collections) if rig else set()
    except Exception:
        have = set()
    for name in preset_names():
        if name not in have:
            out.append((name, "%s    ＋新建" % name, 'ADD'))
    return out


def pie_is_initialized(rig, layer=1):
    try:
        return bool(rig.data.get(pie_init_key(layer), 0))
    except Exception:
        return False


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
            if n not in used and rig.data.collections.get(n) is not None]
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
    """某一层的 8 个饼菜单槽位（空位是空串）。

    **这是只读函数。** 面板和饼菜单都在 draw() 里调它，而 draw 上下文是
    只读的：在里面写 rig.data[...] 会抛
    ``AttributeError: Writing to ID classes in this context is not allowed``，
    而且异常会把整个面板/整张饼截断（现象就是饼只弹出一项、面板那 8 行空白）。
    所以这里绝不落盘：没初始化就现场算一份默认布局返回，写入只发生在
    操作符里（freeze_pie_slots / init_pie_slots / set_pie_slot）。
    """
    slots = [""] * 8
    if rig is None or getattr(rig, "type", None) != 'ARMATURE':
        return slots
    if not pie_is_initialized(rig, layer):
        if not pref('pie_auto_fill', True):
            return slots
        return pie_default_slots(rig, layer)
    for i, k in enumerate(pie_slot_keys(layer)):
        n = rig.data.get(k, "") or ""
        if n and rig.data.collections.get(n) is not None:
            slots[i] = n
    return slots


def freeze_pie_slots(rig, layer=1):
    """把某一层「当前看到的布局」固化进骨架数据。只能在操作符/非 draw 上下文调用。"""
    if rig is None or getattr(rig, "type", None) != 'ARMATURE':
        return [""] * 8
    slots = pie_slots(rig, layer)
    for i, n in enumerate(slots):
        rig.data[pie_slot_keys(layer)[i]] = n
    rig.data[pie_init_key(layer)] = 1
    return slots


def init_pie_slots(rig, layer=1):
    """按默认规则强制重填某一层的 8 格（会覆盖手改）。只能在操作符里调用。"""
    if rig is None or getattr(rig, "type", None) != 'ARMATURE':
        return [""] * 8
    for k in pie_slot_keys(layer):
        rig.data[k] = ""
    slots = pie_default_slots(rig, layer)
    for i, n in enumerate(slots):
        rig.data[pie_slot_keys(layer)[i]] = n
    rig.data[pie_init_key(layer)] = 1
    return slots


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
    if name and rig.data.collections.get(name) is not None:
        return name
    active = rig.data.bones.active
    if active is not None:
        col = nav_collection(rig, active)
        if col is not None:
            return col.name
    return ""


def enter_pie_collection(context, rig, col_name, pie_layer=0):
    """进入饼菜单指定的集合：先框住整组控制器，再落到第一根。

    框选走原生 view_selected（和 Home 键同一套算法，不用自己算包围盒），
    但**用缓动滑过去**（frame_view_animated），不再瞬移 —— 瞬移会让人看丢
    "我现在在骨架的哪个位置"。之后交给 goto_collection，让它把「显示集合、
    选中第一根、记住循环下标、鼠标跟随」这些既有逻辑统一走一遍。
    """
    col = rig.data.collections.get(col_name)
    if col is None:
        return False, "骨架里没有集合「%s」" % col_name

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
        first = names[0]
    for n in names:
        select_bone(rig, n, extend=True)

    # 跳走之前记住出发时的视角：调完细节按 BackSpace 一键弹回全景
    view_back_push(context, rig)

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


def filter_nav_names(rig, names, mode="", cur_name=""):
    """导航过滤：可见性优先，再按每根骨骼自己的 IK/FK 模式收敛。

    ARP 的四肢可以各自 IK/FK；不能用一个全局 mode 过滤整张列表。
    通用控制器（无 ik/fk 标记）始终保留。
    """
    out = []
    for n in names:
        pb = rig.pose.bones.get(n)
        if pb is None:
            continue
        if pref('skip_hidden', True) and not bone_visible(pb):
            continue
        out.append(n)

    if pref('same_mode', True):
        kept = []
        for n in out:
            tag = bone_mode(n) or bone_mode_inherited(rig, n)
            if not tag:
                kept.append(n)
                continue
            own = _bone_own_mode(rig, n, mode)
            if own and tag != own:
                continue
            kept.append(n)
        if kept:
            out = kept
    return out


def nav_list(rig, cur_name):
    """当前集合内的骨骼（已按可见性和 IK/FK 模式过滤）。

    匹配空间排序关键字的集合：按 (head_z, tail_z) 从高到低。
    其余集合：完全按你 assign 进集合的顺序。
    """
    bone = rig.data.bones.get(cur_name) if cur_name else None

    if bone is not None:
        col = nav_collection(rig, bone)
        if col is not None:
            mode = effective_mode(rig, cur_name, col.name)
            return filter_nav_names(rig, collection_bone_names(rig, col), mode, cur_name)
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
    bone = rig.data.bones.get(cur_name) if cur_name else None
    col = nav_collection(rig, bone) if bone is not None else None
    if col is not None:
        remember_scope(rig, col.name)
        mode = effective_mode(rig, cur_name, col.name)
        names = filter_nav_names(rig, collection_bone_names(rig, col), mode, cur_name)
    else:
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
    col = arm.collections.get(name)
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
    for c in rig.data.collections:
        if len(c.bones) == 0:
            continue
        if _INTERNAL_COLL_RE.match(c.name):
            continue
        if not col_visible(c):
            continue
        out.append(c.name)
    return out


def cycle_names(rig, only_non_empty=False):
    """可循环的集合：预设清单里、且当前骨架里确实存在的那些（保序）。

    only_non_empty=True 时跳过没有骨骼的空集合（切换集合时用，
    免得循环卡在还没放控制器的空集合上）。
    """
    out = []
    if rig is None:
        return out
    for n in preset_names():
        col = rig.data.collections.get(n)
        if col is None:
            continue
        if only_non_empty and len(col.bones) == 0:
            continue
        out.append(n)
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
        if rig.data.collections.get(n) is not None:
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
    if forced and rig.data.collections.get(forced):
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
    """三个饼层槽位里出现过的集合（并集）—— 就是「饼菜单管得着的那批」。"""
    out = set()
    for layer in (1, 2, 3):
        try:
            for n in pie_slots(rig, layer):
                if n and rig.data.collections.get(n) is not None:
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
    cur = set(n for n in pie_slots(rig, pie_layer)
              if n and rig.data.collections.get(n) is not None)
    if not cur:
        return
    keep = rig.data.collections.get(keep_name)
    for n in _pie_managed_names(rig) - cur:
        if n == keep_name:
            continue
        col = rig.data.collections.get(n)
        if col is None or col is keep:
            continue
        # 要进的集合是它的子代就别藏父级，否则刚显示又被自己藏掉
        if keep is not None and _is_descendant(keep, col):
            continue
        hide_collection_tree(col)
    # 当前这张饼的全部槽位集合亮出来
    for n in sorted(cur):
        show_collection(rig.data.collections.get(n))
    rig.data["cn_last_pie_layer"] = pie_layer



def show_collection(col):
    """显示集合及其所有父级；如果被 solo 挡住，清掉 solo。"""
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
    if not col_visible(col):
        try:
            for c in col.id_data.collections:
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


def goto_collection(context, rig, name, pie_layer=0):
    """切到指定集合：显示它、选中第一根骨骼、把它设为导航集合。

    选第一根时会沿用切换前的 IK/FK 模式（比如从 FK 切过来就优先选 FK 控制器）；
    如果给这个集合指定过「默认骨骼」（面板里设，存在 cn_first_<集合名>），
    就直接落到它上面 —— 比如腿部集合默认落脚部 IK，不再落在没用的 MCH/根骨骼。

    pie_layer：从哪一张饼进来的（1/2/3），0 = 滚轮或侧栏面板这条路。
    饼菜单要靠它把「上一张饼的集合」整套收掉，见 isolate_previous_collection。
    """
    col = rig.data.collections.get(name)
    if col is None:
        return False, "骨架里没有集合「%s」" % name
    # 无论从哪条路进集合（饼菜单/滚轮循环/面板）都记下出发视角，BackSpace
    # 才有东西可回。饼菜单路径在 enter_pie_collection 里已压过一次，
    # 栈顶同视角去重，不会记两条。
    view_back_push(context, rig)
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
        first = bones[0]
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
        col = rig.data.collections.get(names[ni])
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
    for col in rig.data.collections:
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
        col = rig.data.collections.get(name) if rig else None
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
            targets = list(rig.data.collections)
            label = "所有集合"
        else:
            name = (self.collection or "").strip()
            if not name:
                name = current_cycle_name(rig, rig.data.bones.active) or ""
            col = rig.data.collections.get(name) if name else None
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
            c = rig.data.collections.get(name)
            empty = c is not None and len(c.bones) == 0
            row.enabled = has_sel or empty
            op = row.operator("pose.cn_assign_preset_v45", text=text, icon=icon)
            op.collection = name
            if empty:
                op2 = row.operator("pose.cn_delete_empty_collection_v45",
                                   text="", icon='TRASH')
                op2.collection = name


def pie_rig(context):
    """饼菜单绘制时的骨架。context.object 在某些 UI 上下文里是空的，逐层兜底。"""
    for attr in ("object", "active_object"):
        ob = getattr(context, attr, None)
        if ob is not None and getattr(ob, "type", None) == 'ARMATURE':
            return ob
    for ob in (getattr(context, "selected_objects", None) or []):
        if getattr(ob, "type", None) == 'ARMATURE':
            return ob
    ob = bpy.context.object
    if ob is not None and getattr(ob, "type", None) == 'ARMATURE':
        return ob
    return None


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
    """给饼菜单的某一格指定集合（饼里/面板里点一下都会弹集合列表）"""
    bl_idname = "pose.cn_pie_assign_v45"
    bl_label = "指定饼菜单格子"
    bl_options = {'REGISTER', 'UNDO'}

    slot: bpy.props.IntProperty(name="格位", default=0, min=0, max=7)
    layer: bpy.props.IntProperty(name="饼层", default=1, min=1, max=3)
    collection: bpy.props.StringProperty(name="集合")
    # 集合名是中文，交给 EnumProperty 当 identifier 会在下拉里显示成乱码
    # （第一项还是空 identifier，更要命），所以这里用 StringProperty + 自绘
    # 弹窗列表，标识符只走整数 slot，绝不拿集合名当 ID。

    @classmethod
    def poll(cls, context):
        return pie_menu_poll(context)

    def invoke(self, context, event):
        rig = pie_rig(context)
        if rig is None:
            return {'CANCELLED'}
        idx, layer = self.slot, self.layer
        lname = dict((l, n) for l, n, _k, _i in PIE_LAYERS).get(layer, "")
        wm = context.window_manager

        def draw(menu_self, _ctx):
            lay = menu_self.layout

            def item(nm, text, icon):
                op = lay.operator("pose.cn_pie_assign_v45", text=text, icon=icon)
                op.slot = idx
                op.layer = layer          # ★ 不带上它，弹窗里再点会写回主饼
                op.collection = nm

            lay.label(text="%s · %s（%s）"
                      % (lname, PIE_DIRS[idx], PIE_DIR_HINTS[idx]),
                      icon='MESH_CIRCLE')
            lay.separator()

            # v5.0.4：先列**骨架原生的全部集合**（顺序跟骨骼面板一致），
            # 再列偏好预设里有、这根骨架还没有的。以前只列「有骨骼的非内部
            # 集合」，Rigify 的空 FK/Tweak 集合和容器集合都进不了候选。
            native, internal = assign_collection_rows(rig)
            if native:
                lay.label(text="本骨架的集合（%d 个）" % len(native),
                          icon='OUTLINER_COLLECTION')
                for nm, text, icon in native:
                    item(nm, text, icon)
            else:
                lay.label(text="这根骨架没有可控集合", icon='ERROR')

            miss = missing_preset_rows(rig)
            if miss:
                lay.separator()
                lay.label(text="预设里有、这根骨架没有（点了新建）", icon='ADD')
                for nm, text, icon in miss:
                    item(nm, text, icon)

            if internal:
                lay.separator()
                lay.label(text="内部集合 MCH / DEF / ORG（一般不用）",
                          icon='COLLECTION_COLOR_01')
                for nm, text, icon in internal:
                    item(nm, text, icon)

            lay.separator()
            op = lay.operator("pose.cn_pie_assign_v45", text="清空这一格", icon='X')
            op.slot = idx
            op.layer = layer
            op.collection = ""

        wm.popup_menu(draw, title="%s · %s" % (lname or "饼菜单", PIE_DIRS[idx]))
        return {'RUNNING_MODAL'}

    def execute(self, context):
        rig = pie_rig(context)
        if rig is None or not edit_ok(rig):
            return {'CANCELLED'}
        set_pie_slot(rig, self.slot, self.collection, self.layer)
        self.report({'INFO'}, "%s %s → %s"
                    % (dict((l, n) for l, n, _k, _i in PIE_LAYERS).get(
                        self.layer, "饼"),
                       PIE_DIRS[self.slot], self.collection or "（空）"))
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
        col = rig.data.collections.get(col_name)
        if col is not None and sel[0].name not in collection_bone_names(rig, col):
            self.report({'WARNING'},
                        "%s 不属于集合「%s」，先把骨骼放进集合再设" % (sel[0].name, col_name))
            return {'CANCELLED'}
        rig.data[key] = sel[0].name
        self.report({'INFO'}, "「%s」的默认骨骼 = %s" % (col_name, sel[0].name))
        return {'FINISHED'}


# 视图返回栈在 frame_view_animated 后面定义（_VIEW_BACK_STACK / view_back_push）
_VIEW_BACK_STACK = []
VIEW_BACK_STACK_MAX = 32


def start_view_tween(context, end_loc, end_dist, end_quat, warp=None, persp=None):
    """把视口缓动到指定视角。返回 True = 动画已启动。

    「视图返回」「观察视角」共用这一段：拿当前视角当起点，0.34 秒 smoothstep
    把 location / distance / rotation 一起送过去；persp 不为空时顺便恢复
    透视/正交（书签里记过，回书签要跟记的时候一模一样）。
    """
    area = getattr(context, "area", None)
    space = getattr(context, "space_data", None)
    if area is None or area.type != 'VIEW_3D' or space is None:
        return False
    rv3d = getattr(space, "region_3d", None)
    if rv3d is None or view_tween_busy():
        return False

    start_loc = rv3d.view_location.copy()
    start_dist = rv3d.view_distance
    start_quat = rv3d.view_rotation.copy()
    if ((end_loc - start_loc).length < 1e-4
            and abs(end_dist - start_dist) < 1e-4
            and (persp is None or rv3d.view_perspective == persp)):
        return True              # 已经在目标视角上，什么都不用做

    if persp is not None:
        try:
            rv3d.view_perspective = persp
        except Exception:
            persp = None
        # 切透视/正交后 view_distance 会被 Blender 重新解释，起点要重取
        start_loc = rv3d.view_location.copy()
        start_dist = rv3d.view_distance

    _tween_stop()
    _VIEW_TWEEN.update({
        "kind": "frame",
        "rv3d": rv3d,
        "area": area,
        "region": getattr(context, "region", None),
        "window": getattr(context, "window", None),
        "start": start_loc,
        "end_loc": end_loc.copy(),
        "start_dist": start_dist,
        "end_dist": end_dist,
        "start_quat": start_quat,
        "end_quat": end_quat.copy(),
        "warp": (warp.copy() if warp is not None else start_loc.copy()),
        "t0": time.time(),
        "duration": VIEW_FRAME_SECONDS,
    })
    try:
        _VIEW_TWEEN["handle"] = bpy.app.timers.register(
            _view_tween_step, first_interval=0.0)
    except Exception:
        _tween_stop(unregister=False)
        return False
    return True


def book_read(rig):
    """读出这根骨架的「观察视角」书签；没记过返回 None。"""
    if rig is None:
        return None
    try:
        if not rig.data.get("cn_book_set", 0):
            return None
        loc = Vector(list(rig.data.get("cn_book_loc", (0.0, 0.0, 0.0))))
        dist = float(rig.data.get("cn_book_dist", 1.0))
        q = rig.data.get("cn_book_quat", (1.0, 0.0, 0.0, 0.0))
        quat = Quaternion((q[0], q[1], q[2], q[3]))
        persp = str(rig.data.get("cn_book_persp", 'PERSP') or 'PERSP')
    except Exception:
        return None
    if dist <= 0.0:
        return None
    return loc, dist, quat, persp


def book_write(rig, rv3d):
    """把当前视角存成书签（只能在操作符里调，别在 draw 里调）。"""
    if rig is None or rv3d is None:
        return False
    try:
        rig.data["cn_book_loc"] = [float(v) for v in rv3d.view_location]
        rig.data["cn_book_dist"] = float(rv3d.view_distance)
        q = rv3d.view_rotation
        rig.data["cn_book_quat"] = [float(q.w), float(q.x), float(q.y), float(q.z)]
        rig.data["cn_book_persp"] = str(rv3d.view_perspective or 'PERSP')
        rig.data["cn_book_set"] = 1
        return True
    except Exception:
        return False


class POSE_OT_cn_view_back_v45(bpy.types.Operator):
    """回到跳转前的视角（连按逐级后退，像浏览器的「后退」）"""
    bl_idname = "pose.cn_view_back_v45"
    bl_label = "视图返回"
    bl_options = {'REGISTER'}

    @classmethod
    def poll(cls, context):
        return True

    def execute(self, context):
        st = view_back_pop()
        if st is None:
            self.report({'INFO'}, "没有可返回的视角（先跳进一个集合）")
            return {'CANCELLED'}
        area = getattr(context, "area", None)
        space = getattr(context, "space_data", None)
        if area is None or area.type != 'VIEW_3D' or space is None:
            self.report({'WARNING'}, "在 3D 视口里按才有用")
            return {'CANCELLED'}
        _name, end_loc, end_dist, end_quat = st
        start_view_tween(context, end_loc, end_dist, end_quat)
        return {'FINISHED'}


class POSE_OT_cn_book_save_v45(bpy.types.Operator):
    """把当前视角记成「观察视角」——随时一键回到这个机位看整体动作"""
    bl_idname = "pose.cn_book_save_v45"
    bl_label = "记住当前视角"
    bl_options = {'REGISTER'}

    @classmethod
    def poll(cls, context):
        return True

    def execute(self, context):
        space = getattr(context, "space_data", None)
        area = getattr(context, "area", None)
        rv3d = getattr(space, "region_3d", None) if space is not None else None
        if area is None or area.type != 'VIEW_3D' or rv3d is None:
            self.report({'WARNING'}, "在 3D 视口里按才有用")
            return {'CANCELLED'}
        rig = pie_rig(context) or context.object
        if rig is None or getattr(rig, "type", None) != 'ARMATURE':
            self.report({'WARNING'}, "先选中一根骨架")
            return {'CANCELLED'}
        if not book_write(rig, rv3d):
            self.report({'WARNING'}, "视角记不下来（骨架是只读库链接？）")
            return {'CANCELLED'}
        self.report({'INFO'}, "已记住当前视角为「观察视角」，按 %s 随时回来"
                    % book_key_label())
        return {'FINISHED'}


class POSE_OT_cn_book_goto_v45(bpy.types.Operator):
    """回到「观察视角」；没记过就退回上一个视角（逐级后退）"""
    bl_idname = "pose.cn_book_goto_v45"
    bl_label = "回到观察视角"
    bl_options = {'REGISTER'}

    @classmethod
    def poll(cls, context):
        return True

    def execute(self, context):
        space = getattr(context, "space_data", None)
        if space is None or getattr(context, "area", None) is None:
            return {'CANCELLED'}
        rig = pie_rig(context) or context.object
        bk = book_read(rig)
        if bk is None:
            # 还没记过书签：退回「上一个视角」，保持老习惯不断档
            st = view_back_pop()
            if st is None:
                self.report({'INFO'}, "还没记过观察视角 —— 在面板上点「记住当前视角」")
                return {'CANCELLED'}
            start_view_tween(context, st[1], st[2], st[3])
            return {'FINISHED'}
        loc, dist, quat, persp = bk
        if not start_view_tween(context, loc, dist, quat, persp=persp):
            return {'CANCELLED'}
        return {'FINISHED'}


class POSE_OT_cn_book_clear_v45(bpy.types.Operator):
    """清掉「观察视角」书签"""
    bl_idname = "pose.cn_book_clear_v45"
    bl_label = "清除观察视角"
    bl_options = {'REGISTER'}

    @classmethod
    def poll(cls, context):
        return True

    def execute(self, context):
        rig = pie_rig(context) or context.object
        if rig is None:
            return {'CANCELLED'}
        try:
            for k in ("cn_book_loc", "cn_book_dist", "cn_book_quat",
                      "cn_book_persp", "cn_book_set"):
                if k in rig.data:
                    del rig.data[k]
        except Exception:
            pass
        return {'FINISHED'}


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
        cols = rig.data.collections
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
                          {"slot": i, "layer": layer}, 'ADD'))
        elif cols.get(name) is None:
            items.append(("%s · 集合已丢失" % name, "pose.cn_pie_assign_v45",
                          {"slot": i, "layer": layer}, 'ERROR'))
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
KEY_SYNC_SECONDS = 0.04

# ---- 自由旋转（TRACKBALL）的原生角度曲线
# 原生 gizmo 的角度不是线性的：绕枢轴在球面上划，angle = 2·atan(位移 / 球半径)。
# 起手快、越远越收敛。球半径跟活动半径同源（ring_zone），指针顶到活动半径
# 那一刻正好约 90°，和原生拖满一圈 gizmo 的手感一致。
TRACKBALL_R_PX = 110.0

# ---- 轴探测 / 手势分级
ROT_PROBE_DEG = 2.0        # 探测角：把每根未锁轴转这么多，看末端往屏幕哪边动
ROT_SCREEN_ALIGN = 0.5     # 屏幕方向归类阈值：投影和水平/垂直的点积要超过它才算"顺着"

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


def basis_from_world_rotation(rig, pb, basis, quat):
    """把一个「世界空间的旋转」换算成骨骼的 matrix_basis 增量。

    matrix = parent.matrix @ basis @ bone.matrix_local，想要世界旋转 q 左乘上去，
    就得把 q 依次转进 armature 空间、再转进父级空间，最后左乘到 basis 上。
    q 是纯旋转，两边夹相似变换后仍然是纯旋转，不会引入缩放/平移。
    """
    m = rig.matrix_world
    r = (m.inverted() @ quat.to_matrix().to_4x4() @ m)
    parent = pb.parent
    if parent is not None:
        r = parent.matrix.inverted() @ r @ parent.matrix
    return r @ basis


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
    """
    basis = pb.matrix_basis.copy() if matrix_basis is None else matrix_basis
    try:
        m = basis @ pb.bone.matrix_local
        parent = pb.parent
        if parent is not None:
            m = parent.matrix @ m
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
        saved = [(p, p.select) for p in rig.pose.bones]
        try:
            for p in rig.pose.bones:
                p.select = False
            for b in bones:
                b.select = True
            try:
                rig.data.bones.active = bones[0].bone
            except Exception:
                pass
            if ts is not None:
                ts.use_proportional_edit = False   # 别让比例编辑把别的骨骼带上
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
                    b.matrix = Matrix.Translation(
                        self._lock_filter(b, pend)) @ base[0]
                except Exception:
                    pass
            bpy.context.view_layer.update()
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
            pb = self._rig.pose.bones.get(name)
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
        if context is not None:
            self._sync_keys(context, force=True)   # 曲线也跟上，否则下次重估又被拽走
        return True

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
        saved = [(p, p.select) for p in rig.pose.bones]
        try:
            for p in rig.pose.bones:
                p.select = False
            for b in bones:
                b.select = True
            try:
                rig.data.bones.active = bones[0].bone
            except Exception:
                pass
            if ts is not None:
                ts.use_proportional_edit = False   # 别让比例编辑把别的骨骼带上
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
        限流到 40ms 一次，避免 400+ 骨骼的 rig 上拖不动。
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
            self._rim_acc += d * factor
            self._ang_h = self._rim_acc * self._s_sign
            return
        u = self._s_u
        if u is None:
            return
        # u 是屏幕上「y 向上」的 2D 方向
        self._ang_h += (dx * u.x + dyu * u.y) * sens

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
        self._track_dx += dx * factor
        self._track_dy += dy * factor
        right, up = _view_axes(rv3d)
        ang_h = math.radians(self._native_curve_deg(self._track_dx))
        ang_v = math.radians(self._native_curve_deg(self._track_dy))
        delta = (Quaternion(up, ang_h) @ Quaternion(right, ang_v))
        # 转完视角时会把当时的旋转冻结进 _quat0，并把 _track_dx/dy 归零。
        # 直接在 _quat0 之上叠这一段的增量 —— 否则累计位移会被当前视角的
        # right/up 重新解释一遍，姿态瞬间跳到完全不同的方向（剧烈扭动）。
        self._quat = delta @ self._quat0

    # ---- 灵敏度 / 指针回绕

    def _screen_len(self, context):
        """骨骼在屏幕上的投影长度（像素），用来归一化灵敏度。"""
        region, rv3d = _view_pair(context)
        if region is None or rv3d is None:
            return ROT_REF_PX
        pts = _probe_offsets(self._pb)
        m = self._rig.matrix_world @ self._pb.matrix
        p0 = _screen_of(region, rv3d, m.translation.copy())
        p1 = _screen_of(region, rv3d, _bone_point_world(self._rig, self._pb, None, pts[0]))
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
                pb = rig.pose.bones.get(name)
                if pb is not None:
                    try:
                        pb.hide = hidden
                    except Exception:
                        pass
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
            cand = self._rig.pose.bones.get(target)
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
        pb = self._rig.pose.bones.get(self._rig.data.bones.active.name) \
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
                self._ang_h += dx * sens * (1.0 if uh is None or uh.x >= 0 else -1.0)
                self._ang_v += dy * sens * (1.0 if uv is None or uv.y >= 0 else -1.0)
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
                self._switch_collection(context, step)
                self._header(context)
                return {'RUNNING_MODAL'}
            if wheel_act == 'BONE':
                if self._kind == 'MOVE':
                    self._move_flush(context)
                self._switch_bone(context, step, extend=event.shift)
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
_RIG_PROP_RE = re.compile(r"props\.([a-z_]+)\s*=\s*(.+)$")
_RIG_IKFK_PROP_RE = re.compile(r"pose_bones\['([^']+)'\].*IK_FK")


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

    返回 label → {label, prop_bone, to_ik_op, to_ik, to_fk_op, to_fk}
    """
    if rig is None or rig.type != 'ARMATURE':
        return []
    if not rig.data.get("rig_id"):
        return []
    rid = rig.data.get("rig_id")
    source = None
    for text in bpy.data.texts:
        try:
            s = text.as_string()
        except Exception:
            continue
        if ("pose.rigify_limb_ik2fk_" + rid) in s or ("pose.rigify_generic_snap_" + rid) in s:
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
            lm = re.search(r"IK-FK \(([^)]+)\)", line)
            if lm:
                prop_map[lm.group(1)] = m.group(1)
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
        item.setdefault("prop_bone", prop_map.get(label, ""))
    return [v for v in limbs.values() if v.get("prop_bone") and v.get("to_ik_op") and v.get("to_fk_op")]


def rigify_limb_mode(rig, label):
    info = next((x for x in rigify_limbs(rig) if x["label"] == label), None)
    if not info:
        return None
    pb = rig.pose.bones.get(info["prop_bone"])
    if pb is None or "IK_FK" not in pb:
        return None
    return "IK" if float(pb["IK_FK"]) < 0.5 else "FK"


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
    __slots__ = ("label", "prop_bone", "prop_name", "fk_value",
                 "ik_controls", "fk_controls",
                 "pairs_to_fk", "pairs_to_ik",
                 "rigify_to_fk", "rigify_to_ik")

    def __init__(self):
        self.label = ""
        self.prop_bone = ""
        self.prop_name = "IK_FK"
        self.fk_value = 1.0
        self.ik_controls = []
        self.fk_controls = []
        self.pairs_to_fk = []
        self.pairs_to_ik = []
        self.rigify_to_fk = None
        self.rigify_to_ik = None


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
        fk = _as_bone_list(to_fk.get("fk_bones")) or \
            _as_bone_list(to_ik.get("output_bones"))
        ik = (_as_bone_list(to_fk.get("ctrl_bones"))
              + _as_bone_list(to_fk.get("tail_bones"))
              + _as_bone_list(to_fk.get("extra_ctrls")))
        if not ik:
            ik = _as_bone_list(to_ik.get("ctrl_bones"))
        lb = CN_Limb()
        lb.label = info.get("label") or info.get("prop_bone", "?")
        lb.prop_bone = info.get("prop_bone", "")
        lb.prop_name = "IK_FK"
        lb.fk_value = 1.0
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
    if not limbs:
        try:
            limbs = _limbs_animate(rig)
            if limbs:
                kind = "animate"
        except Exception:
            limbs = []
    if sig is not None:
        _IKFK_CACHE[sig] = (kind, limbs)
    return kind, limbs


def limb_mode(rig, lb):
    pb = rig.pose.bones.get(lb.prop_bone)
    if pb is None or lb.prop_name not in pb:
        return None
    try:
        v = float(pb[lb.prop_name])
    except Exception:
        return None
    return "FK" if v >= 0.5 else "IK"


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


def ikfk_sync_visibility(rig, lb, target):
    """切完只留当前那套控制器，另一套逐根藏起来。

    不能按集合藏：IK 和 FK 控制器就躺在同一个集合里（"左臂"里两者都有）。
    """
    if not pref("sync_collections", True):
        return
    keep = lb.fk_controls if target == "FK" else lb.ik_controls
    drop = lb.ik_controls if target == "FK" else lb.fk_controls
    for n in drop:
        b = rig.data.bones.get(n)
        if b is not None:
            b.hide = True
    for n in keep:
        b = rig.data.bones.get(n)
        if b is not None:
            b.hide = False


def limb_switch(rig, lb, target, hide_other=True):
    """切到 target（'IK' / 'FK'）：先快照姿态，再翻开关，最后换控制器可见性。"""
    cur = limb_mode(rig, lb)
    if cur is None:
        return False, "读不到 %s 的 %s" % (lb.prop_bone, lb.prop_name)
    if cur == target:
        if hide_other:
            ikfk_sync_visibility(rig, lb, target)
        return False, "%s 已经是 %s" % (lb.label, target)

    call = lb.rigify_to_fk if target == "FK" else lb.rigify_to_ik
    if call is not None:
        op_id, props = call
        fn = getattr(bpy.ops.pose, op_id.split(".")[-1], None)
        if fn is None:
            return False, "Rigify 的 snap 没注册：请勾选「自动运行脚本」后重开文件"
        try:
            fn(**props)
        except Exception as exc:
            return False, "Rigify snap 出错：%s" % exc
    else:
        pairs = lb.pairs_to_fk if target == "FK" else lb.pairs_to_ik
        if not pairs:
            return False, "%s 没有可以快照的骨骼" % lb.label
        if _snap_by_matrix(rig, pairs) == 0:
            return False, "%s 快照失败" % lb.label

    pb = rig.pose.bones.get(lb.prop_bone)
    if pb is not None and lb.prop_name in pb:
        try:
            pb[lb.prop_name] = (lb.fk_value if target == "FK"
                                else (0.0 if lb.fk_value >= 0.5 else 1.0))
        except Exception:
            pass
    try:
        rig.update_tag()
        bpy.context.view_layer.update()
    except Exception:
        pass
    if hide_other:
        ikfk_sync_visibility(rig, lb, target)
    return True, "%s → %s" % (lb.label, target)


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


_PRIMARY_WORDS = ("hand", "foot", "wrist", "ankle")


def _primary_control(names):
    """一组控制器里挑「主控制器」——手/脚那种末端目标，而不是 pole 或辅助骨。

    自动跳选中时的兜底：手 IK 切成 FK 之后该落到 c_hand_fk 上，落到
    c_arms_pole 或者 c_thigh_fk 上都没法接着转。
    """
    for n in names:
        low = n.lower()
        if low.startswith("c_"):
            low = low[2:]
        if any(w in low for w in _PRIMARY_WORDS):
            return n
    return names[0] if names else None


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
        for lb in picked:
            cur = limb_mode(rig, lb)
            tgt = self.target if self.target in ("IK", "FK") else (
                "FK" if cur == "IK" else "IK")
            ok, _msg = limb_switch(rig, lb, tgt)
            if ok:
                moved += 1
                last = (lb, tgt)
                done.append(lb)
        if not moved:
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
    WIDTH = 238

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
    head.label(text="%d" % len(limbs))

    col = layout.column(align=True)
    for lb in limbs:
        mode = limb_mode(rig, lb) or "?"
        tgt = "FK" if mode == "IK" else "IK"
        row = col.row(align=True)
        name = row.row(align=True)
        name.scale_x = 0.55
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
    if kind == "rigify":
        layout.label(text="Rigify 原生 snap", icon='CHECKMARK')


def draw_ikfk_items(layout, context):
    """侧栏面板和浮动面板共用的一份内容。"""
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

    col = layout.column(align=True)
    for lb in limbs:
        mode = limb_mode(rig, lb) or "?"
        mark = "● " if lb in picked else ""
        row = col.row(align=True)
        row.label(text="%s%s" % (mark, lb.label))
        sub = row.row(align=True)
        sub.scale_x = 0.85
        op = sub.operator("pose.cn_ikfk_toggle_v45",
                          text="IK" if mode == "FK" else "FK",
                          icon='ARROW_LEFTRIGHT')
        op.label = lb.label
        op.target = "IK" if mode == "FK" else "FK"
        tag = sub.row(align=True)
        tag.scale_x = 0.7
        tag.label(text=mode)

    layout.separator()
    row = layout.row(align=True)
    op = row.operator("pose.cn_ikfk_toggle_v45", text="全部 → IK")
    op.target = "IK"
    op.all_limbs = True
    op = row.operator("pose.cn_ikfk_toggle_v45", text="全部 → FK")
    op.target = "FK"
    op.all_limbs = True
    if kind == "rigify":
        layout.label(text="Rigify 原生 snap", icon='CHECKMARK')


def draw_ikfk_panel(layout, context):
    rig = context.object
    if rig is None or getattr(rig, "type", None) != 'ARMATURE':
        return
    kind, limbs = ikfk_limbs(rig)
    if not limbs:
        return
    box = layout.box()
    head = box.row(align=True)
    head.label(text="IK / FK", icon='ARMATURE_DATA')
    head.label(text="%d 个肢体" % len(limbs))
    draw_ikfk_items(box, context)


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
_POPUP_WATCH = {"timer": None, "window": None, "region": None, "outside": 0.0, "unknown": 0}
_TOOLTIP_BACKUP = {}
_POPUP_PATCH = {}


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


def _cursor_in_region(region, win):
    """鼠标是否在 popup 区域内；返回 None 表示无法判断（不关）。"""
    try:
        import ctypes
        from ctypes import wintypes
        pt = wintypes.POINT()
        if not ctypes.windll.user32.GetCursorPos(ctypes.byref(pt)):
            return None
    except Exception:
        return None
    x = pt.x - win.x
    y = win.height - (pt.y - win.y)
    if not (0 <= x <= win.width and 0 <= y <= win.height):
        return None
    return (region.x <= x <= region.x + region.width and
            region.y <= y <= region.y + region.height)


def _popup_watch_step():
    st = _POPUP_WATCH
    win, region = st.get("window"), st.get("region")
    if win is None or region is None:
        st["timer"] = None
        return None
    inside = _cursor_in_region(region, win)
    if inside is None:
        st["unknown"] = st.get("unknown", 0) + 1
        if st["unknown"] >= 100:
            st["timer"] = None
            return None
        return 0.1
    if inside:
        st["outside"] = 0.0
        return 0.1
    st["outside"] = st.get("outside", 0.0) + 0.1
    if st["outside"] >= 0.35:
        try:
            win.event_simulate(type='ESC', value='PRESS')
        except Exception:
            pass
        st["timer"] = None
        return None
    return 0.1


def start_popup_watch(context):
    if not pref("popup_auto_close", True):
        return
    win = getattr(context, "window", None)
    region = getattr(context, "region", None)
    if win is None or region is None:
        return
    _POPUP_WATCH.update(window=win, region=region, outside=0.0, unknown=0)
    if _POPUP_WATCH.get("timer") is None:
        try:
            _POPUP_WATCH["timer"] = bpy.app.timers.register(_popup_watch_step, first_interval=0.1)
        except Exception:
            pass


def patch_rigui_popup():
    return False  # disabled: 见 _rigui_patch_timer 注释
    cls = getattr(bpy.types, "RIG_UI_PT_openPopUp", None)
    if cls is None or getattr(cls, "_cn_wrapped", False):
        return False
    original = cls.draw

    def draw(self, context):
        original(self, context)
        start_popup_watch(context)

    _POPUP_PATCH["cls"] = cls
    _POPUP_PATCH["orig"] = original
    cls.draw = draw
    cls._cn_wrapped = True
    return True


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


_NAV_PANEL_PATCH = {}


def draw_controller_nav_section(layout, context):
    """把 Controller Nav 的集合按钮 + IK/FK 状态画进 rigUI 面板。"""
    rig = getattr(context, "object", None)
    if rig is None or rig.type != 'ARMATURE':
        return
    box = layout.box()
    head = box.row(align=True)
    head.label(text="驭骨 · BonePilot", icon='ARMATURE_DATA')
    head.label(text="集合 / IK-FK")

    names = [n for n in preset_names() if rig.data.collections.get(n) is not None]
    if names:
        cur = rig.data.get("cn_collection") or selection_collection_name(rig)
        col = box.column(align=True)
        row = None
        for i, name in enumerate(names):
            if i % 2 == 0:
                row = col.row(align=True)
            op = row.operator("pose.cn_collection_goto_v45", text=name, depress=(name == cur))
            op.collection = name

    draw_ikfk_panel(box, context)


def patch_rigui_panels():
    return False  # disabled: 见 _rigui_patch_timer 注释
    for idname in ("RIG_UI_PT_openPopUp", "RIG_UI_PT_BoneCollectionsSidePanel"):
        cls = getattr(bpy.types, idname, None)
        if cls is None or getattr(cls, "_cn_nav_patched", False):
            continue
        original = cls.draw

        def draw(self, context, _orig=original):
            _orig(self, context)
            draw_controller_nav_section(self.layout, context)

        _NAV_PANEL_PATCH[idname] = (cls, original)
        cls.draw = draw
        cls._cn_nav_patched = True


def unpatch_rigui_panels():
    for idname, (cls, original) in list(_NAV_PANEL_PATCH.items()):
        try:
            cls.draw = original
            if hasattr(cls, "_cn_nav_patched"):
                delattr(cls, "_cn_nav_patched")
        except Exception:
            pass
    _NAV_PANEL_PATCH.clear()


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
                box.label(text=cur_col.name, icon='COLLECTION_COLOR_01')
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

        # ---- 观察视角：自己记一个「看整体动作」的机位，一键回去
        # 摆在骨骼操作**下面**、饼菜单上面：骨骼那几块先聚成一坨，
        # 看整体动作是调完细节之后的事，顺着往下按。
        vbox = layout.box()
        vh = vbox.row(align=True)
        vh.label(text="观察视角", icon='VIEW_CAMERA')
        bk = book_read(rig)
        if bk is not None:
            vh.label(text="已记录（距离 %.1f）" % bk[1])
        row = vbox.row(align=True)
        row.operator("pose.cn_book_save_v45",
                     text="重记当前视角" if bk is not None else "记住当前视角",
                     icon='FILE_REFRESH' if bk is not None else 'ADD')
        if bk is not None:
            row.operator("pose.cn_book_clear_v45", text="", icon='X')
        row = vbox.row(align=True)
        row.scale_y = 1.15
        row.operator("pose.cn_book_goto_v45",
                     text=("回到观察视角（%s）" if bk is not None
                           else "回到上一个视角（%s）") % book_key_label(),
                     icon='LOOP_BACK')
        if _VIEW_BACK_STACK:
            brow = vbox.row(align=True)
            brow.operator("pose.cn_view_back_v45",
                          text="上一个视角（还剩 %d 个）" % len(_VIEW_BACK_STACK),
                          icon=safe_icon('BACK'))
        vbox.label(text="键位在插件偏好里改（观察视角键 / 修饰）", icon='INFO')

        # 饼菜单：三层（主饼/细节饼/附属饼），各 8 格，点名字换集合
        # 每块都能折叠 —— 主饼配好之后基本不用再看，收起来省一大截滚动
        cur_col_pie = pie_current_collection(rig)
        for layer, lname, lhint, licon in PIE_LAYERS:
            slots = pie_slots(rig, layer)
            pbox = layout.box()
            head = pbox.row(align=True)
            flag = "pie_open_%d" % layer
            opened = bool(getattr(prefs, flag, True)) if prefs is not None else True
            if prefs is not None:
                head.prop(prefs, flag, text="", emboss=False,
                          icon='TRIA_DOWN' if opened else 'TRIA_RIGHT')
            head.label(text="饼菜单 · %s（%s）" % (lname, lhint),
                       icon=safe_icon(licon))
            if any(slots):
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
                names = [s for s in slots if s]
                if names:
                    pbox.label(text="  ".join(names[:4]) +
                               ("  …" if len(names) > 4 else ""),
                               icon='DOT')
                continue
            for i, name in enumerate(slots):
                row = pbox.row(align=True)
                row.scale_y = 0.85
                sub = row.row(align=True)
                sub.ui_units_x = 3.2
                act = bool(name) and name == cur_col_pie
                sub.label(text=PIE_DIRS[i], icon='LAYER_ACTIVE' if act else 'BLANK1')
                # 和饼里点空格子走同一个入口：点一下直接弹出集合列表
                opx = row.operator("pose.cn_pie_assign_v45",
                                   text=name or "（空）",
                                   icon='BONE_DATA' if name else 'ADD',
                                   depress=act)
                opx.slot = i
                opx.layer = layer

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
            col = arm.collections.get(name)
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

    book_key: bpy.props.EnumProperty(
        name="观察视角键",
        items=BOOK_KEY_ITEMS,
        default='BACK_SPACE',
        description="按这个键回到你自己记下的「观察视角」；"
                    "还没记过书签时，它会退回上一个视角（逐级后退）",
        update=lambda self, context: refresh_keymap(),
    )

    book_key_mod: bpy.props.EnumProperty(
        name="观察视角键修饰",
        items=KEY_MOD_ITEMS,
        default='NONE',
        description="给观察视角键加修饰键，避开 Blender 原生快捷键",
        update=lambda self, context: refresh_keymap(),
    )

    pie_open_1: bpy.props.BoolProperty(name="主饼展开", default=True)
    pie_open_2: bpy.props.BoolProperty(name="细节饼展开", default=True)
    pie_open_3: bpy.props.BoolProperty(name="附属饼展开", default=True)

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

    # ---- 手指自动轴（v4.9.22 起停用，属性留着兼容旧偏好文件）
    auto_finger: bpy.props.BoolProperty(
        name="手指自动轴",
        default=False,
        description="已停用：手指轴自动限制不再生效，请用下面的手动锁轴",
    )
    finger_first_a: bpy.props.EnumProperty(
        name="第一节轴 1", items=FINGER_AXIS_ITEMS, default='X')
    finger_first_b: bpy.props.EnumProperty(
        name="第一节轴 2", items=FINGER_AXIS_ITEMS, default='Z')
    finger_other: bpy.props.EnumProperty(
        name="第二/三节轴", items=FINGER_AXIS_ITEMS, default='X')

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

    popup_auto_close: bpy.props.BoolProperty(
        name="骨骼集合弹窗移出自动关",
        default=True,
        description="Rig UI 的骨骼集合弹窗：鼠标移出 0.35 秒后自动关闭，不用再去 3D 视图点一下",
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
        description="骨骼自身隐藏、或它所在的所有集合都隐藏时跳过（含集合可见性判据）",
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

        # ---- 观察视角键（回自己记的机位）
        vkb = layout.box()
        vkb.label(text="观察视角键", icon='VIEW_CAMERA')
        row = vkb.row(align=True)
        row.prop(self, "book_key", text="键")
        row.prop(self, "book_key_mod", text="")
        vkb.label(text="当前：%s —— 按它回到「观察视角」；还没记过书签时退回上一个视角"
                   % book_key_label(), icon='CHECKMARK')
        vkb.label(text="面板里点「记住当前视角」把现在的机位记下来（存在骨架上，随文件保存）",
                  icon='INFO')

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
        mbox.prop(self, "popup_auto_close")
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
        box.prop(self, "collection_keyword")
        box.label(text="同 IK/FK：当前骨骼带 _ik / _fk 标记时，导航只在同模式里转", icon='INFO')
        box.label(text="名字里带 pole 的极向控制器按 IK 处理", icon='INFO')

        layout.label(text="做法：把控制器归到集合，导航键就在该集合内循环")
        layout.label(text="加选 = 按住 Shift 再按导航键　M = 归入预设集合")
        layout.label(text="旋转 = 按 X 后直接拖鼠标，轴自动判断，滚轮换关节")


# ---------------------------------------------------------------- 注册

HIT_PX = 26.0


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
        col = rig.data.collections.get(self.collection or "")
        if col is None:
            self.report({'WARNING'}, "集合不存在")
            return {'CANCELLED'}
        if len(col.bones) != 0:
            self.report({'WARNING'}, "这个集合不是空集合，不能删除")
            return {'CANCELLED'}
        rig.data.collections.remove(col)
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
    CN_Preset_v45,
    POSE_OT_cn_create_presets_v45,
    POSE_OT_cn_assign_preset_v45,
    POSE_OT_cn_new_collection_v45,
    POSE_OT_cn_preset_menu_v45,
    POSE_OT_cn_pie_jump_v45,
    POSE_OT_cn_pie_note_v45,
    POSE_OT_cn_pie_assign_v45,
    POSE_OT_cn_pie_autofill_v45,
    POSE_OT_cn_pie_clear_v45,
    POSE_OT_cn_view_back_v45,
    POSE_OT_cn_book_save_v45,
    POSE_OT_cn_book_goto_v45,
    POSE_OT_cn_book_clear_v45,
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
    """删掉指向 v2.1–v2.4 operator 的键位（主要是残留的滚轮绑定）。"""
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
                if not idn.startswith(LEGACY_PREFIXES):
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
            kmi = km.keymap_items.new('wm.call_menu_pie', key, mode, **mods)
            kmi.properties.name = menu_name
            addon_keymaps.append((km, kmi))
        except Exception:
            pass

    try:
        # 观察视角：一键回到自己记下的机位（没记过就退回上一个视角）
        bkey, bkw = book_key_spec()
        kmi = km.keymap_items.new('pose.cn_book_goto_v45', bkey, 'PRESS', **bkw)
        addon_keymaps.append((km, kmi))
        # 逐级后退：自己绑的键
        vspec = kb_spec('VIEW_BACK')
        if vspec is not None and vspec != (bkey, bkw):
            try:
                kmi = km.keymap_items.new(
                    'pose.cn_view_back_v45', vspec[0],
                    'CLICK' if 'MOUSE' in vspec[0] else 'PRESS', **vspec[1])
                addon_keymaps.append((km, kmi))
            except Exception:
                pass
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
    h = _POPUP_WATCH.get("timer")
    if h is not None:
        try: bpy.app.timers.unregister(h)
        except Exception: pass
        _POPUP_WATCH["timer"] = None
    unregister_keymaps()
    try:
        del bpy.types.Object.cn_spatial
    except Exception:
        pass
    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
