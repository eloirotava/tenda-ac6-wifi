#!/usr/bin/env python3
# Embrulha funcoes do wl_core.o (ELF32 LE, relocavel): cria __real_<f> no endereco
# original e deixa <f> indefinido, para as chamadas internas (por relocacao) irem ao wrapper.
# uso: hook-core.py <entrada.o> <saida.o> f1 f2 ...
import struct, subprocess, sys, shutil
src, dst, funcs = sys.argv[1], sys.argv[2], sys.argv[3:]
OC = 'arm-linux-gnueabi-objcopy'
def load(p):
    d = bytearray(open(p, 'rb').read())
    shoff, = struct.unpack_from('<I', d, 0x20)
    shentsize, shnum, shstrndx = struct.unpack_from('<HHH', d, 0x2e)
    sh = [list(struct.unpack_from('<IIIIIIIIII', d, shoff + i * shentsize)) for i in range(shnum)]
    shstr = sh[shstrndx]
    nm = lambda off, base: d[base + off:d.index(b'\0', base + off)].decode()
    secname = [nm(s[0], shstr[4]) for s in sh]
    st = next(i for i, s in enumerate(sh) if s[1] == 2)
    strtab = sh[sh[st][6]]
    syms = {}
    for i in range(sh[st][5] // 16):
        o = sh[st][4] + i * 16
        n, val, size, info, oth, ndx = struct.unpack_from('<IIIBBH', d, o)
        syms.setdefault(nm(n, strtab[4]), (o, val, size, info, ndx))
    return d, secname, syms
d, secname, syms = load(src)
args = []
for f in funcs:
    o, val, size, info, ndx = syms[f]
    assert ndx not in (0, 0xfff1, 0xfff2) and (info >> 4) == 1, f
    args += ['--add-symbol', '__real_%s=%s:0x%x,global,function' % (f, secname[ndx], val)]
subprocess.check_call([OC] + args + [src, dst])
d, secname, syms = load(dst)
for f in funcs:
    o, val, size, info, ndx = syms[f]
    struct.pack_into('<IIBBH', d, o + 4, 0, 0, (1 << 4) | 0, 0, 0)   # GLOBAL NOTYPE UND
open(dst, 'wb').write(d)
print('ok', len(funcs), 'funcoes')
