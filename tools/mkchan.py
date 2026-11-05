#!/usr/bin/env python3
"""Gera phy_ac_wlchan56.h: a troca de canal do wl para cada canal visto no rastro.

uso: mkchan.py RASTRO SAIDA.h [RASTRO2 LINHA ...]  (janela extra: de LINHA ao fim de RASTRO2)

O wl grava o chanspec na SHM 0x0a0 em cada troca de canal (write_shm 000000a0);
a troca roda com o MAC suspenso: do ultimo suspend_mac antes do marcador ate o
primeiro enable_mac depois. Para cada canal fica a ultima janela vista. As ops
usam a mesma codificacao do replay (tools/mkreplay.py).
"""
import sys

REG = {'wphy': 1, 'mphy': 2, 'aphy': 3, 'ophy': 4, 'wrad': 5, 'mrad': 6, 'arad': 7, 'orad': 8}


def convert(lines):
    ops, cur = [], None
    for line in lines:
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
        elif t == 'tblx':
            cur = (int(f[2], 16), int(f[3], 16))
        elif t == 'tbld' and cur:
            tid, w = cur
            off, v = int(f[2], 16), int(f[3], 16)
            ops.append((1, 0x0d, 0xffff, tid))
            ops.append((1, 0x0e, 0xffff, off & 0xffff))
            if w == 48:
                ops.append((1, 0x11, 0xffff, v & 0xffff))
                ops.append((16, 0, 0, v >> 16))
                ops.append((16, 0, 0, int(f[4], 16) & 0xffff))
                continue
            if w == 32:
                ops.append((1, 0x10, 0xffff, v >> 16))
            ops.append((1, 0x0f, 0xffff, v & 0xffff))
        elif t == 'write_shm':
            ops.append((14, int(f[2], 16) & 0xffff, 0xffff, int(f[3], 16) & 0xffff))
    return ops


src, dst = sys.argv[1], sys.argv[2]
extra = sys.argv[3:]
L = open(src).read().split('\n')
marks = [i for i, l in enumerate(L) if l.startswith('write_shm') and l.split()[2] == '000000a0']
win = {}
starts = [(m, m - 1514) for m in marks if m >= 1514]
for k, (m, start) in enumerate(starts):
    # from the suspend that opens this channel set to the start of the next
    # one (the dwell and the end-of-scan restores belong to the channel too);
    # the last one runs to the end of the trace
    end = starts[k + 1][1] if k + 1 < len(starts) else len(L)
    chspec = int(L[m].split()[3], 16)
    win[chspec] = (chspec, L[start + 1:end])

for k in range(0, len(extra), 2):
    L2 = open(extra[k]).read().split('\n')
    st = int(extra[k + 1]) - 1
    seg = L2[st:]
    cs = [int(l.split()[3], 16) for l in seg if l.startswith('write_shm') and l.split()[2] == '000000a0']
    if cs:
        win[cs[-1]] = (cs[-1], seg)

allops, idx = [], []
for ch in sorted(win):
    chspec, lines = win[ch]
    bw = chspec & 0x3800; c = chspec & 0xff
    ctl = c if bw == 0x1000 else (c - 6 + 4 * ((chspec >> 8) & 7) if bw == 0x2000 else c - 2 + 4 * ((chspec >> 8) & 7))
    ops = convert(lines)
    idx.append((ctl, chspec, len(allops), len(ops)))
    allops += ops
with open(dst, 'w') as o:
    o.write('/* gerado por tools/mkchan.py de %s: troca de canal do wl por canal */\n' % src.split('/')[-1])
    o.write('/* chan = control channel; chanspec bits 11..13: 0x1000 20, 0x1800 40, 0x2000 80 MHz */\n')
    o.write('struct b43_ac56_chan_seg { u8 chan; u16 chanspec; u32 start, len; };\n')
    o.write('static const struct b43_ac56_chan_seg b43_ac56_chansegs[] = {\n')
    for ch, cs, st, n in idx:
        o.write('\t{%d, 0x%04x, %d, %d},\n' % (ch, cs, st, n))
    o.write('};\nstatic const struct b43_ac56_op b43_ac56_chanops[] = {\n')
    for t, a, m, v in allops:
        o.write('\t{%d,0x%03x,0x%04x,0x%04x},\n' % (t, a, m, v))
    o.write('};\n')
for ch, cs, st, n in idx:
    print('canal %3d chanspec 0x%04x: %d ops' % (ch, cs, n))
