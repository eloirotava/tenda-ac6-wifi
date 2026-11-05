#!/usr/bin/env python3
# compara rastro do wl (wphy/mphy/aphy/ophy/wrad/mrad/...) com o do b43 (wphy/wrad)
import sys, collections
def load(p):
    ph, ra = collections.OrderedDict(), collections.OrderedDict()
    for l in open(p):
        f = l.split()
        if len(f) < 5 or f[0] == 'tbl': continue
        t, a, m, v = f[0], int(f[2], 16), int(f[3], 16), int(f[4], 16)
        d = ph if t.endswith('phy') else ra
        cur = d.get(a, [0, 0])          # [valor conhecido, mascara conhecida]
        if t[0] == 'w': cur = [v, 0xffff]
        elif t[0] == 'm': cur = [(cur[0] & ~m) | (v & m), cur[1] | m]
        elif t[0] == 'a': cur = [cur[0] & v, cur[1] | (~v & 0xffff)]
        elif t[0] == 'o': cur = [cur[0] | v, cur[1] | v]
        d[a] = cur
    return ph, ra
wl = load(sys.argv[1]); b = load(sys.argv[2])
for nome, dw, db in (('PHY', wl[0], b[0]), ('RADIO', wl[1], b[1])):
    so_wl = [a for a in dw if a not in db]
    dif = [a for a in dw if a in db and ((dw[a][0] ^ db[a][0]) & dw[a][1] & db[a][1])]
    print('== %s: wl %d regs, b43 %d regs, so no wl %d, valor diferente %d' % (nome, len(dw), len(db), len(so_wl), len(dif)))
    print('  so no wl:', ' '.join('%03x' % a for a in sorted(so_wl)[:120]))
    print('  diferentes (reg wl/b43):', ' '.join('%03x=%04x/%04x' % (a, dw[a][0], db[a][0]) for a in sorted(dif)[:60]))
