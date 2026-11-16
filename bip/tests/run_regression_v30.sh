#!/usr/bin/env bash
# v3.0 全套回归：真实工程只读，逐场景独立 Blender 进程
set -u
BL="D:/CG WORK/BLENDRE/animation BL 5.3/blender.exe"
BASE="D:/CG WORK/BLENDRE/blender file/animation/卢十五/第九章·单击"
E="$BASE/E 踏步回斩.blend"
D="$BASE/D踏步上挑.blend"
T="C:/Users/dazuoge/Desktop/bip 手感测试.blend"
W="C:/Users/dazuoge/WorkBuddy/2026-11-13-09-56-28"
LOG="$W/_regression_v30.log"
: > "$LOG"

# 每个场景先写独立临时文件，再追加进总日志：RESULT 只从本场景取。
run() {
  local tag="$1" blend="$2" script="$3"; shift 3
  echo "=== $tag ===" >> "$LOG"
  local tmp="$LOG.part"
  env "$@" "$BL" --background "$blend" --python "$W/$script" > "$tmp" 2>&1
  cat "$tmp" >> "$LOG"
  local last
  last=$(grep -E "RESULT (ALL_PASS|PASS|FAIL)" "$tmp" | tail -1)
  printf '%-46s %s\n' "$tag" "${last:-NO RESULT}"
}

run_factory() {
  local tag="$1" blend="$2" script="$3"; shift 3
  echo "=== $tag ===" >> "$LOG"
  local tmp="$LOG.part"
  env "$@" "$BL" --factory-startup --background "$blend" \
      --python "$W/$script" > "$tmp" 2>&1
  cat "$tmp" >> "$LOG"
  local last
  last=$(grep -E "RESULT (ALL_PASS|PASS|FAIL)" "$tmp" | tail -1)
  printf '%-46s %s\n' "$tag" "${last:-NO RESULT}"
}

# --- v3.0 旋转模式闪烁专项 + v2.9 撤销显隐 + v2.8 / v2.7 独显
run "verify_v30_flicker | FLICKER"     "$E" verify_v30_flicker.py
run "verify_v29_undo | UNDO"           "$E" verify_v29_undo.py
run "verify_v28_solo | V28"            "$E" verify_v28_solo.py
run "verify_v27_solo | SOLO"           "$E" verify_v27_solo.py

# --- v2.6 画弧多帧
run "verify_v26_arc | ARC"             "$E" verify_v26_arc.py

# --- v2.3 旋转插帧连续性
for s in ROT_RAW ROT_FIX REPAIR BIP HOOK; do
  run "verify_rot_interp | $s"         "$E" verify_rot_interp.py VRI_SCEN="$s"
done

# --- BIP 基础功能（D 工程 / 测试工程）
run "verify_bip_no_jitter.py | D"      "$D" verify_bip_no_jitter.py
run "verify_bip_interp.py | D"         "$D" verify_bip_interp.py
run "verify_bip_hinge.py | D"          "$D" verify_bip_hinge.py
run "verify_bip_no_jitter | test"      "$T" verify_bip_no_jitter.py

# --- v2.1/v2.2 近伸直 / 拉直
for side in r l; do
  run "verify_bip_straight | $side"    "$E" verify_bip_straight.py VST_SIDE="$side"
  run "verify_bip_extend | $side"      "$E" verify_bip_extend.py   VEX_SIDE="$side"
done

# --- v2.4 六种轴序 + 四元数 / 轴角
for mode in XYZ ZYX XZY ZXY YZX YXZ QUATERNION AXIS_ANGLE; do
  run "verify_order_convert | $mode ROT" "$E" verify_order_convert.py VOC_MODE="$mode" VOC_SCEN=ROT
done
for mode in XYZ ZYX YXZ QUATERNION; do
  run "verify_order_convert | $mode BIP" "$E" verify_order_convert.py VOC_MODE="$mode" VOC_SCEN=BIP
done
for mode in XYZ ZYX YXZ; do
  run "verify_order_convert | $mode FIX" "$E" verify_order_convert.py VOC_MODE="$mode" VOC_SCEN=FIX
done
for mode in ZYX XYZ; do
  run "verify_order_convert | $mode BAD" "$E" verify_order_convert.py VOC_MODE="$mode" VOC_SCEN=BAD
done

# --- 注册 / 导航 / 切骨 / 旋转 / 路径
for s in verify_bip_merge verify_bip_nav verify_bip_switch verify_bip_rot verify_bip_mpath_now verify_mpath_registered; do
  run "$s.py | registered"             "$E" "$s.py"
done

# --- v2.5 运动路径实时联动
for s in LIVE SYNC OLD DELAY HOOKS PREF_OFF NO_AUTOKEY MODAL CANCEL FRAME PERF; do
  run_factory "verify_mpath_live | $s" "$E" verify_mpath_live.py VML_SCEN="$s"
done

echo
echo "===== SUMMARY ====="
echo "PASS : $(grep -c 'RESULT ALL_PASS\|RESULT PASS' "$LOG")"
echo "FAIL : $(grep -c 'RESULT FAIL' "$LOG")"
echo "LOG  : $LOG"
