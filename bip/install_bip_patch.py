# -*- coding: utf-8 -*-
"""把 ARP BIP 扩展幂等植入「驭骨 · BonePilot」插件。

用法（直接跑，无需 Blender）：
    python install_bip_patch.py                    # 自动找 5.3 / 5.2.4 里的驭骨主文件并植入
    python install_bip_patch.py --file <路径>      # 指定驭骨 .py
    python install_bip_patch.py --uninstall        # 只移除扩展，恢复原文件

特点：
  - 每次运行先做时间戳备份（同目录 *.bak-wb-bip-...）；
  - 重复运行不会叠加：先移除旧扩展块与 3 处挂钩，再重新植入；
  - 植入前做语法编译检查，失败绝不写目标文件；
  - 保留原文件的 CRLF / LF 换行风格。

驭骨升级后扩展被覆盖时，重跑一次本脚本即可。
"""
import argparse
import glob
import os
import py_compile
import re
import shutil
import sys
import tempfile
from datetime import datetime

try:
    sys.stdout.reconfigure(encoding='utf-8')
except Exception:
    pass

HERE = os.path.dirname(os.path.abspath(__file__))
SEARCH_DIRS = (
    r'D:\CG WORK\BLENDRE\animation BL 5.3\portable\scripts\addons',
    r'D:\CG WORK\BLENDRE\animation BL\portable\scripts\addons',
)

EXT_BEGIN = '# >>> ARP BIP EXT BEGIN (WorkBuddy) >>>'
EXT_END = '# <<< ARP BIP EXT END (WorkBuddy) <<<'
HOOK_NAMES = ('INVOKE', 'REGISTER', 'UNREGISTER')

HOOK_TEXT = {
    'INVOKE': """        # >>> ARP BIP HOOK INVOKE (WorkBuddy) >>>
        if self.start_mode == 'MOVE':
            try:
                _bip_side = bip_hand_side(context)
            except Exception:
                _bip_side = None
            if _bip_side is not None:
                try:
                    _bip_res = bpy.ops.pose.cn_bip_drag_v45(
                        'INVOKE_DEFAULT', side=_bip_side)
                    if 'CANCELLED' not in _bip_res:
                        return {'FINISHED'}
                except Exception as _bip_exc:
                    print('[ARP BIP] invoke: %s' % _bip_exc)
        # <<< ARP BIP HOOK INVOKE (WorkBuddy) <<<
""",
    'REGISTER': """    # >>> ARP BIP HOOK REGISTER (WorkBuddy) >>>
    try:
        bip_register()
    except Exception as _bip_exc:
        print('[ARP BIP] register: %s' % _bip_exc)
    # <<< ARP BIP HOOK REGISTER (WorkBuddy) <<<
""",
    'UNREGISTER': """    # >>> ARP BIP HOOK UNREGISTER (WorkBuddy) >>>
    try:
        bip_unregister()
    except Exception as _bip_exc:
        print('[ARP BIP] unregister: %s' % _bip_exc)
    # <<< ARP BIP HOOK UNREGISTER (WorkBuddy) <<<
""",
}

ANCHORS = {
    'INVOKE': ("    def invoke(self, context, event):\n"
               "        if _MODAL_ONCE[0]:\n"),
    'REGISTER': ("    register_keymaps()\n"
                 "    bpy.app.timers.register(_kb_bootstrap, first_interval=0.6)\n"),
    'UNREGISTER': ("    unregister_keymaps()\n"
                   "    try:\n"
                   "        del bpy.types.Object.cn_spatial\n"),
}

MAIN_ANCHOR = 'if __name__ == "__main__":\n    register()\n'


def discover():
    # 按目录优先级选：5.3 里有驭骨合并版就只动 5.3，不要误改 5.2.4。
    for d in SEARCH_DIRS:
        found = [p for p in glob.glob(os.path.join(d, 'controller_navigator_v*.py'))
                 if os.path.isfile(p)]
        if found:
            return max(found, key=os.path.getmtime)
    raise SystemExit('未找到驭骨主文件，请用 --file 指定路径')


def strip_blocks(text):
    text = re.sub(r'[ \t]*' + re.escape(EXT_BEGIN) + r'.*?'
                  + re.escape(EXT_END) + r'[ \t]*\n?',
                  '', text, flags=re.S)
    for name in HOOK_NAMES:
        b = '# >>> ARP BIP HOOK %s (WorkBuddy) >>>' % name
        e = '# <<< ARP BIP HOOK %s (WorkBuddy) <<<' % name
        text = re.sub(r'[ \t]*' + re.escape(b) + r'.*?'
                      + re.escape(e) + r'[ \t]*\n?',
                      '', text, flags=re.S)
    return text


def syntax_check(text):
    fd, tmp = tempfile.mkstemp(suffix='.py', prefix='wb_bip_')
    os.close(fd)
    try:
        with open(tmp, 'w', encoding='utf-8', newline='\n') as f:
            f.write(text)
        py_compile.compile(tmp, doraise=True)
    finally:
        try:
            os.unlink(tmp)
        except Exception:
            pass
        try:
            os.unlink(tmp + 'c')
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--file', default=None, help='驭骨主 .py 路径')
    ap.add_argument('--extension', default=os.path.join(HERE, 'bip_extension.py'))
    ap.add_argument('--uninstall', action='store_true')
    args = ap.parse_args()

    target = args.file or discover()
    if not os.path.isfile(target):
        raise SystemExit('目标文件不存在: %s' % target)
    with open(target, 'rb') as f:
        raw = f.read()
    crlf = b'\r\n' in raw
    text = raw.decode('utf-8').replace('\r\n', '\n')
    base = strip_blocks(text)

    if args.uninstall:
        new_text = base
    else:
        with open(args.extension, 'r', encoding='utf-8') as f:
            ext = f.read().replace('\r\n', '\n').rstrip('\n') + '\n'
        if ext.count(EXT_BEGIN) != 1 or ext.count(EXT_END) != 1:
            raise SystemExit('扩展块标记不完整')
        new_text = base
        for name in HOOK_NAMES:
            anchor = ANCHORS[name]
            if new_text.count(anchor) != 1:
                raise SystemExit('挂钩锚点不唯一或缺失: %s (count=%d)'
                                 % (name, new_text.count(anchor)))
            head, tail = anchor.split('\n', 1)
            # 保留锚点首行，把 hook 插在它后面
            new_text = new_text.replace(
                anchor, head + '\n' + HOOK_TEXT[name] + tail, 1)
        if new_text.count(MAIN_ANCHOR) != 1:
            raise SystemExit('文件尾 __main__ 锚点不唯一或缺失')
        new_text = new_text.replace(
            MAIN_ANCHOR, '\n' + ext + '\n' + MAIN_ANCHOR, 1)

    syntax_check(new_text)
    out = new_text.replace('\n', '\r\n') if crlf else new_text

    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    backup = '%s.bak-wb-bip-%s' % (target, stamp)
    shutil.copy2(target, backup)
    with open(target, 'wb') as f:
        f.write(out.encode('utf-8'))

    print('TARGET :', target)
    print('BACKUP :', backup)
    print('MODE   :', 'uninstall' if args.uninstall else 'install')
    print('BLOCKS :', new_text.count(EXT_BEGIN),
          '| hooks:', sum(new_text.count('# >>> ARP BIP HOOK %s' % n)
                          for n in HOOK_NAMES))
    print('BYTES  :', len(raw), '->', len(out.encode('utf-8')))
    print('SYNTAX : OK')
    print('DONE   : 重启 Blender（或禁用/启用驭骨插件）后生效')


if __name__ == '__main__':
    main()
