#!/usr/bin/env python3
"""Gera phy_ac_wlreplay56.h (b43_ac56_wlops) a partir de um rastro do wltrace.

uso: mkreplay.py RASTRO SAIDA.h

Tipos: 1-4 wphy/mphy/aphy/ophy, 5-8 wrad/mrad/arad/orad, 9 delay (mask<<16|val us),
10 cflags do d11 (addr = bits 16..27 da mascara, mask/val = bits 0..15),
15 bits 16..31 do valor do cflags anterior, 11 phyclk_fgc, 12/13 suspend/enable MAC,
14 write_shm, 16 raw write of the PHY data window (48 bit table words). pmuchip/pmupll/mhf/tbl ficam de fora.
"""
import sys

src, dst = sys.argv[1], sys.argv[2]
REG = {'wphy': 1, 'mphy': 2, 'aphy': 3, 'ophy': 4, 'wrad': 5, 'mrad': 6, 'arad': 7, 'orad': 8}
ops = []
cur = None
for line in open(src):
    f = line.split()
    if len(f) < 3:
        continue
    t = f[0]
    if t in REG and len(f) >= 5:
        ops.append((REG[t], int(f[2], 16), int(f[3], 16), int(f[4], 16)))
    elif t == 'delay':
        us = int(f[2], 16)
        ops.append((9, 0, us >> 16, us & 0xffff))
    elif t == 'cflags':
        m, v = int(f[2], 16), int(f[3], 16)
        ops.append((10, (m >> 16) & 0xfff, m & 0xffff, v & 0xffff))
        if v >> 16:
            ops.append((15, 0, 0, v >> 16))
    elif t == 'phyclk_fgc':
        ops.append((11, 0, 0, int(f[2], 16) & 0xffff))
    elif t == 'suspend_mac':
        ops.append((12, 0, 0, 0))
    elif t == 'enable_mac':
        ops.append((13, 0, 0, 0))
    elif t == 'tblx':			# phy_utils_write_phytable_ext: id, width, len, regs
        cur = (int(f[2], 16), int(f[3], 16))
    elif t == 'tbld' and cur:		# one entry of the last tblx: offset, value
        tid, w = cur
        off, v = int(f[2], 16), int(f[3], 16)
        ops.append((1, 0x0d, 0xffff, tid))
        ops.append((1, 0x0e, 0xffff, off & 0xffff))
        if w == 48:			# word 0 to 0x11, words 1 and 2 raw to the PHY data window (type 16)
            ops.append((1, 0x11, 0xffff, v & 0xffff))
            ops.append((16, 0, 0, v >> 16))
            ops.append((16, 0, 0, int(f[4], 16) & 0xffff))
            continue
        if w == 32:
            ops.append((1, 0x10, 0xffff, v >> 16))
        ops.append((1, 0x0f, 0xffff, v & 0xffff))
    elif t == 'write_shm':
        ops.append((14, int(f[2], 16) & 0xffff, 0xffff, int(f[3], 16) & 0xffff))

with open(dst, 'w') as o:
    o.write('/* gerado por tools/mkreplay.py de %s (%d ops).\n' % (src.split('/')[-1], len(ops)))
    o.write(' * 1-8 PHY/radio; 9 delay (mask<<16|val us); 10 core IOCTL (addr = mascara bits 16..27,\n')
    o.write(' * mask/val = bits 0..15); 15 valor dos bits 16..31; 11 phyclk_fgc; 12/13 suspend/enable MAC; 14 SHM */\n')
    o.write('struct b43_ac56_op { u8 type; u16 addr, mask, val; };\n')
    o.write('static const struct b43_ac56_op b43_ac56_wlops[] = {\n')
    for t, a, m, v in ops:
        o.write('\t{%d,0x%03x,0x%04x,0x%04x},\n' % (t, a, m, v))
    o.write('};\n')
print(len(ops), 'ops')
