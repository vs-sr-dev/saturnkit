"""Function discovery for flat SH-2 programs without symbols.

    python -m saturnkit.recomp.discover FILE.BIN --base 0600B000 [--seeds a,b,..] [--report]

On the SH-2, code and data share the text: every function is followed by
its literal pool, switch tables sit right after the jump that reads them,
and nothing says where one ends. So this is a recursive descent that
classifies each halfword as code or data while it finds the functions,
written for Hitachi SHC's output (and GCC's, which is close):

1. Seeds: the entry point, the caller's list, and every call target found
   on the way: `bsr` targets, and `jsr @rn` whose rn was loaded from the
   literal pool in the same straight line (`mov.l @(disp,PC),rn`).
2. A function is the code reachable from its entry: conditional branches
   (both ways), `bra`, and `jmp @rn` with a literal target (SHC's branch
   for distances beyond `bra`'s 4 KiB), switch tables, fall-through. A
   branch to another function's entry is a tail call and is not followed.
   Functions may share code (a tail into the middle of another is followed
   as its own); the recompiler duplicates it, which is harmless.
3. Every delayed branch runs its slot: the slot is code, its literal is
   data.
4. Literal loads mark their 2 or 4 bytes as data; `mova` marks nothing
   until a switch claims the table (hand-written assembly also takes code
   addresses with it, to jump to).
5. SHC switch: `mov #N,r1; cmp/hs r1,r0; bt default; shll|shll2 r0;
   mov r0,r1; mova TABLE,r0; mov.w|mov.l @(r0,r1),r0; braf r0`. N entries
   of 16 or 32 bits, each an offset from the `braf` + 4. The runtime
   library's form: `mov #N,rB; cmp/hs r0,rB; bf default; mov.l TABLE,rT;
   mov.l @(r0,rT),rJ; jmp @rJ`, r0 a byte offset, N/4 + 1 absolute entries;
   and its shift ladders, a table of signed bytes added to a base (also
   with the index offset by a constant). GCC's switch: `mova TABLE,r0;
   mov.w @(r0,rI),rI; add rI,r0; jmp @r0`, entries relative to the table
   (8-bit ones in hand-written assembly), or `mov.l @(r0,rI),rJ; jmp @rJ`
   with absolute entries; bounded by `cmp/hi` or `cmp/hs` against a
   constant, or by an `and` mask, and never running into its own targets.
6. Unreached code: after the pass, a value in the data or in unclassified
   words that points into the program is a function reached by pointer (a
   callback, an interrupt handler, a slave job) if it is not dereferenced
   where it is loaded (a load or store through it: a variable's address),
   is not text (a string passed as an argument) nor two pointers (a table
   or a pool), and its descent meets no undefined opcode, no data, and no
   pair of pointers. It must also be at a *boundary* (after data, or after
   a terminator's delay slot), open with a stack-frame prologue, be loaded
   by the code as a literal, or sit in a table of pointers; for the last
   two, which is how hand-written handlers are reached, the descent must
   be at least 8 instructions (a data record can decode cleanly for a few).
   Failing all that, unclassified code that
   starts with a stack-frame prologue at a boundary (SHC links whole
   object files, so uncalled functions sit between called ones) is taken
   the same way. Repeat to a fixed point.
7. Registers that hold call targets across a function (SHC keeps a
   often-called address in r8-r14) are found by forward constant
   propagation over the function's code; calls clobber r0-r7.

`Program` keeps: funcs {entry: Function}, code (halfword addresses), data
(halfword addresses), switches {braf address: [targets]}, and the
indirect jumps and calls it could not resolve.
"""
import argparse
import bisect
import collections

from .. import sh2

TERMINATORS = {"rts", "rte", "bra", "jmp", "braf"}


def _writes(ins):
    """Registers an instruction writes (for the literal look-back)."""
    f = ins.fmt
    out = set()
    if f.endswith("Rn") and ins.n is not None:
        out.add(ins.n)
    elif f.endswith(",r0") or ins.op == "mova":
        out.add(0)
    elif ins.op in ("movt", "dt") or (f.split()[0] == ins.op and f.endswith("Rn")):
        out.add(ins.n)
    if "@Rm+" in f and ins.m is not None:
        out.add(ins.m)
    if "@-Rn" in f and ins.n is not None:
        out.add(ins.n)
    return out


def _transfer(img, ins, st):
    """Register constants after `ins`, from `st` (a list of 16 values or None)."""
    op, f = ins.op, ins.fmt
    if op in ("jsr", "bsr", "bsrf"):
        for r in range(8):              # SHC: r0-r7 caller-saved
            st[r] = None
        return
    if ins.target is not None and op in ("mov.l", "mov.w") and ins.size:
        st[ins.n] = img.literal(ins)
        return
    if op == "mova":
        st[0] = ins.target
        return
    if op == "mov" and ins.imm is not None:
        st[ins.n] = ins.imm & 0xFFFFFFFF
        return
    if op == "mov" and f == "mov Rm,Rn":
        st[ins.n] = st[ins.m]
        return
    if op == "add" and ins.imm is not None:
        v = st[ins.n]
        st[ins.n] = None if v is None else (v + ins.imm) & 0xFFFFFFFF
        return
    if op == "add" and f == "add Rm,Rn":   # SHC's -pic: mova LABEL,r0; add r0,rn
        v, w = st[ins.n], st[ins.m]
        st[ins.n] = None if v is None or w is None else (v + w) & 0xFFFFFFFF
        return
    for r in _writes(ins):
        st[r] = None


class Function:
    __slots__ = ("entry", "code", "calls", "tails", "unresolved", "bad", "tables", "slot_only")

    def __init__(self, entry):
        self.entry = entry
        self.code = set()          # instruction addresses reached
        self.calls = set()         # call targets (bsr, jsr with literal)
        self.tails = set()         # tail calls to other entries
        self.unresolved = []       # (addr, what) indirect jumps not resolved
        self.bad = None            # first problem met, or None
        self.tables = set()        # halfwords of the switch tables it reads
        self.slot_only = set()     # delay slots not (yet) reached as instructions of their own


class Program:
    def __init__(self, img, seeds=(), bounds=None):
        self.img = img
        self.lo, self.hi = bounds or (img.base, img.end)
        self.funcs = {}
        self.code = set()
        self.data = set()
        self.switches = {}
        self.tables = set()        # halfwords of switch tables (not function pointers)
        self.ext_calls = collections.Counter()   # calls through pointers: loaded-from address
        self._pending = list(seeds) or [img.base]
        self._rejected = set()
        self._data_addrs, self._scanned = set(), set()   # _data_addresses, incrementally
        self._scanned_pic = set()                         # _computed_seeds, likewise
        self._lits, self._lit_scanned = set(), set()     # _literal_values, likewise
        self.run()

    # -- helpers
    def inside(self, a):
        return self.lo <= a < self.hi and not a & 1

    def _mark_data(self, a, size):
        for k in range(0, size, 2):
            self.data.add(a + k)

    def _literal_for(self, at, reg, limit=24):
        """Value loaded into `reg` by a literal load in the straight line before `at`.

        Returns ("lit", value, slot), ("ptr", address) for `mov.l @rX,reg` with
        rX a literal (a call through a pointer, e.g. a BIOS vector), or None.
        """
        a = at - 2
        want = reg
        for _ in range(limit):
            if not self.inside(a) or a in self.data:
                return None
            ins = self.img.insn(a)
            if ins.op in ("bt", "bf", "bt/s", "bf/s", "bra", "bsr", "jmp", "jsr", "rts", "rte", "braf", "bsrf"):
                return None
            if want in _writes(ins):
                if ins.op == "mov.l" and ins.size == 4 and ins.target is not None:
                    v = self.img.literal(ins)
                    return ("lit", v, ins.target) if v is not None else None
                if ins.op == "mov.l" and ins.fmt in ("mov.l @Rm,Rn", "mov.l @(disp,Rm),Rn"):
                    inner = self._literal_for(a, ins.m, limit)
                    if inner and inner[0] == "lit" and ins.fmt == "mov.l @Rm,Rn":
                        return ("ptr", inner[1])
                    if inner:                      # a table reached through a pointer
                        return ("ptr", inner[1])
                    return None
                if ins.op == "mov" and ins.fmt == "mov Rm,Rn":
                    want = ins.m
                    a -= 2
                    continue
                return None
            a -= 2
        return None

    def _abs_switch(self, jmp):
        """SHC's absolute jump table: mov #N,rB; cmp/hs r0,rB; bf default;
        mov.l @(disp,PC),rT; mov.l @(r0,rT),rJ; jmp @rJ  (r0 a byte offset <= N)."""
        img = self.img
        j = img.insn(jmp)
        ld = img.insn(jmp - 2)
        if ld.op != "mov.l" or ld.fmt != "mov.l @(r0,Rm),Rn" or ld.n != j.n:
            return None
        lit = self._literal_for(jmp - 2, ld.m)
        if not lit or lit[0] != "lit":
            return None
        table = lit[1]
        n = None
        a = jmp - 4
        for _ in range(8):
            ins = img.insn(a)
            if ins.op == "cmp/hs" and ins.m == 0:
                nxt = img.insn(a + 2)
                for b in range(a - 2, a - 12, -2):
                    k = img.insn(b)
                    if k.op == "mov" and k.imm is not None and k.n == ins.n:
                        n = k.imm // 4 + 1 if nxt.op in ("bf", "bf/s") else None
                        break
                break
            a -= 2
        if not n or n > 1024 or not img.contains(table, 4 * n):
            return None
        return [img.u32(table + 4 * k) for k in range(n)], table, 4 * n

    def _byte_switch(self, jmp):
        """The runtime library's shift ladders: mov.l TAB,rT; add ri,rT;
        mov.b @rT,rT; mov.l BASE,rJ; add rT,rJ; jmp @rJ, bounded by
        mov #N,rN; cmp/ge rN,ri; bt (index < N). Targets: BASE + signed byte."""
        img = self.img
        seq = [img.insn(jmp - 2 * k) for k in range(5, 0, -1)]
        j = img.insn(jmp)
        t1, add1, ldb, t2, add2 = seq
        if not (t1.op == "mov.l" and t1.size == 4 and add1.op == "add" and add1.fmt == "add Rm,Rn"
                and add1.n == t1.n and ldb.fmt == "mov.b @Rm,Rn" and ldb.m == t1.n
                and t2.op == "mov.l" and t2.size == 4 and t2.n == j.n
                and add2.fmt == "add Rm,Rn" and add2.n == j.n and add2.m == ldb.n):
            return None
        tab, base = img.literal(t1), img.literal(t2)
        return self._byte_targets(jmp, tab, 0, add1.m, base)

    def _byte_switch_offset(self, jmp):
        """The same ladder with the index offset: mov.l TAB,rT; add #k,ri;
        add rT,ri; mov.b @ri,rB; mov.l BASE,rJ; add rB,rJ; jmp @rJ.
        Entry i (0 <= i < N) is the byte at TAB + k + i."""
        img = self.img
        t1, addk, add1, ldb, t2, add2 = [img.insn(jmp - 2 * k) for k in range(6, 0, -1)]
        j = img.insn(jmp)
        if not (t1.op == "mov.l" and t1.size == 4 and addk.op == "add" and addk.imm is not None
                and add1.fmt == "add Rm,Rn" and add1.m == t1.n and add1.n == addk.n
                and ldb.fmt == "mov.b @Rm,Rn" and ldb.m == add1.n
                and t2.op == "mov.l" and t2.size == 4 and t2.n == j.n
                and add2.fmt == "add Rm,Rn" and add2.n == j.n and add2.m == ldb.n):
            return None
        return self._byte_targets(jmp, img.literal(t1), addk.imm, addk.n, img.literal(t2))

    def _byte_targets(self, jmp, tab, k, idx, base):
        """Targets BASE + signed byte for index 0 <= i < N, the bound read
        from `mov #N,rN; cmp/ge rN,ri` before the jump."""
        img = self.img
        n = None
        for a in range(jmp - 12, jmp - 40, -2):
            ins = img.insn(a)
            if ins.op == "cmp/ge" and ins.n == idx:
                for b in range(a - 2, a - 12, -2):
                    mv = img.insn(b)
                    if mv.op == "mov" and mv.imm is not None and mv.n == ins.m:
                        n = mv.imm
                        break
                break
        if tab is None or base is None or not n or n > 256:
            return None
        start = (tab + k) & 0xFFFFFFFF
        if not img.contains(start, n):
            return None
        targets = []
        for i in range(n):
            v = img.data[start - img.base + i]
            targets.append((base + (v - 256 if v & 0x80 else v)) & 0xFFFFFFFF)
        lo = start & ~1
        return targets, lo, ((start + n + 1) & ~1) - lo

    _LOADS = {"mov.b @(r0,Rm),Rn": 1, "mov.w @(r0,Rm),Rn": 2, "mov.l @(r0,Rm),Rn": 4}
    _STOPS = ("bra", "jmp", "rts", "rte", "braf")

    def _mova_switch(self, jmp, before=None):
        """GCC's switch and its kin, a `mova` table read by `jmp`:

            mova TABLE,r0; mov.w|mov.b @(r0,rI),rI; add rI,r0; jmp @r0   (entries relative to TABLE)
            mova TABLE,r0; mov.l @(r0,rI),rJ; ...; jmp @rJ              (absolute entries)

        The number of entries comes from the bound before it, `mov #K,rB;
        cmp/hi rB,rX` (K + 1) or `cmp/hs` (K), or a mask, `and #M` (M + 1
        entries if the index is scaled after it, M / size + 1 if not);
        failing that, the table runs until it would overlap a target. Either
        way it stops before the first target past it: a table does not run
        into the code it jumps to.

        `before`: GCC also computes the target, then `bra`s to a `jmp @r0`
        placed after a pool; the computation is then read back from the
        `bra` at `before`."""
        img = self.img
        j = img.insn(jmp)
        load = mova = None
        rel = False
        end = before if before is not None else jmp
        for a in range(end - 2, end - 16, -2):
            if not self.inside(a):
                return None
            ins = img.insn(a)
            if ins.op in self._STOPS or ins.delay:
                return None
            if ins.fmt in self._LOADS and ins.m != 0 and load is None:
                load = (a, ins)
            elif ins.op == "mova" and load is not None:
                mova = (a, ins)
                break
        if not load or not mova:
            return None
        la, ld = load
        size = self._LOADS[ld.fmt]
        between = [img.insn(b) for b in range(la + 2, end, 2)]
        if before is not None:
            between.append(img.insn(before + 2))       # the bra's delay slot
        if j.n == 0 and any(i.fmt == "add Rm,Rn" and i.n == 0 and i.m == ld.n for i in between):
            rel = True
        elif j.n != ld.n or size != 4 or any(ld.n in _writes(i) for i in between):
            return None
        table = mova[1].target
        n = self._switch_bound(mova[0], size)
        entries, first_after = [], None
        for k in range(n or 1024):
            ea = table + k * size
            if not img.contains(ea, size) or (first_after is not None and ea >= first_after):
                break
            v = img.data[ea - img.base] if size == 1 else img.u16(ea) if size == 2 else img.u32(ea)
            if rel:
                bits = 8 * size
                v = (table + (v - (1 << bits) if v >> (bits - 1) else v)) & 0xFFFFFFFF
            if not self.inside(v):
                if n:
                    return None
                break
            entries.append(v)
            if v > table and (first_after is None or v < first_after):
                first_after = v
        if not entries or (n and len(entries) != n):
            return None
        span = len(entries) * size
        return entries, table & ~1, ((table + span + 1) & ~1) - (table & ~1)

    def _mova_base(self, jmp):
        """T for `mova T,r0; ... add rX,r0; jmp @r0` in the straight line before
        `jmp`, r0 not otherwise written in between; None otherwise."""
        img = self.img
        if img.insn(jmp).n != 0:
            return None
        added = False
        for a in range(jmp - 2, jmp - 14, -2):
            if not self.inside(a):
                return None
            ins = img.insn(a)
            if ins.op in self._STOPS or ins.delay:
                return None
            if ins.op == "mova":
                return ins.target if added and self.inside(ins.target) else None
            if ins.fmt == "add Rm,Rn" and ins.n == 0:
                added = True
            elif 0 in _writes(ins):
                return None
        return None

    def _record_switch(self, jmp):
        """A table of records whose first word is the target (hand-written
        assembly): `and #M` on the index, shifts, `mova TABLE,r0; add rI,r0;
        mov.l @r0+,rJ` (or `@r0`), more loads from the record, `jmp @rJ`.
        The offsets are every index the mask lets through, scaled by the
        shifts between the mask and the `mova`."""
        img = self.img
        j = img.insn(jmp)
        ld = None
        for a in range(jmp - 2, jmp - 24, -2):
            if not self.inside(a):
                return None
            ins = img.insn(a)
            if ins.op in self._STOPS or ins.delay:
                return None
            if ins.fmt in ("mov.l @Rm+,Rn", "mov.l @Rm,Rn") and ins.m == 0 and ins.n == j.n:
                ld = a
                break
            if j.n in _writes(ins):
                return None
        if ld is None:
            return None
        add, mova = img.insn(ld - 2), img.insn(ld - 4)
        if not (add.fmt == "add Rm,Rn" and add.n == 0 and mova.op == "mova"):
            return None
        idx, scale, mask = add.m, 1, None
        for a in range(ld - 6, ld - 30, -2):
            ins = img.insn(a)
            if ins.op in self._STOPS:
                return None
            if ins.n == idx and ins.op == "shll":
                scale *= 2
            elif ins.n == idx and ins.op == "shll2":
                scale *= 4
            elif ins.n == idx and ins.fmt == "add Rm,Rn" and ins.m == idx:
                scale *= 2
            elif ins.n == idx and ins.fmt == "mov Rm,Rn":
                idx = ins.m
            elif ins.op == "and" and ins.imm is not None and idx == 0:
                mask = ins.imm
                break
            elif idx in _writes(ins):
                return None
        if mask is None or mask > 1023:
            return None
        table = mova.target
        offsets = [v * scale for v in range(mask + 1) if v & ~mask == 0]
        targets = []
        for o in offsets:
            if not img.contains(table + o, 4):
                return None
            t = img.u32(table + o)
            if not self.inside(t):
                return None
            targets.append(t)
        # the records are data; the table spans them all (the other fields included)
        stride = scale * (mask & -mask or 1)
        return targets, table & ~1, ((table + offsets[-1] + stride + 1) & ~1) - (table & ~1)

    def _switch_bound(self, at, size):
        """The number of entries of a table indexed just after `at`, from the
        bound or the mask before it in the same straight line, or None."""
        img = self.img
        scaled = False
        for a in range(at - 2, at - 24, -2):
            if not self.inside(a):
                return None
            ins = img.insn(a)
            if ins.op in self._STOPS:
                return None
            if ins.op in ("shll", "shll2") or (ins.fmt == "add Rm,Rn" and ins.m == ins.n):
                scaled = True
            if ins.op in ("cmp/hi", "cmp/hs") and a + 2 < at and img.insn(a + 2).op in ("bt", "bt/s"):
                k = self._reg_const(a, ins.m)
                if k is None:
                    return None
                return k + 1 if ins.op == "cmp/hi" else k
            if ins.op == "and":
                m = ins.imm if ins.imm is not None else self._reg_const(a, ins.m)
                if m is None or m >= 4096:
                    return None
                return m + 1 if scaled else m // size + 1
        return None

    def _reg_const(self, at, reg):
        """The constant `mov #imm,reg` or a literal load gives `reg` in the few instructions before `at`."""
        img = self.img
        for a in range(at - 2, at - 12, -2):
            if not self.inside(a):
                return None
            ins = img.insn(a)
            if ins.op in self._STOPS:
                return None
            if reg in _writes(ins):
                if ins.op == "mov" and ins.imm is not None:
                    return ins.imm & 0xFFFFFFFF
                if ins.op in ("mov.w", "mov.l") and ins.target is not None and ins.size:
                    return img.literal(ins)
                return None
        return None

    def _switch(self, braf):
        """Targets of an SHC switch ending at `braf`, and its table, or None."""
        img = self.img
        ld = img.insn(braf - 2)
        mova = img.insn(braf - 4)
        if mova.op != "mova" or ld.op not in ("mov.w", "mov.l") or "@(r0," not in ld.fmt:
            return None
        size = 2 if ld.op == "mov.w" else 4
        table = mova.target
        n = None
        # the bound: mov #N,rX ; cmp/hs rX,r0 ; bt default  (within a few instructions)
        a = braf - 6
        for _ in range(8):
            ins = img.insn(a)
            if ins.op == "cmp/hs" and ins.n == 0:
                for b in range(a - 2, a - 12, -2):
                    j = img.insn(b)
                    if j.op == "mov" and j.imm is not None and j.n == ins.m:
                        n = j.imm
                        break
                break
            a -= 2
        if not n or n <= 0 or n > 1024:
            return None
        base = braf + 4
        targets = []
        for k in range(n):
            ea = table + k * size
            if not img.contains(ea, size):
                return None
            v = img.u16(ea) if size == 2 else img.u32(ea)
            if size == 2 and v & 0x8000:
                v -= 0x10000
            if size == 4 and v & 0x80000000:
                v -= 1 << 32
            targets.append((base + v) & 0xFFFFFFFF)
        return targets, table, n * size

    def _constants(self, code, entry, wanted, switches=None):
        """Register constants at each address in `wanted`, by forward dataflow over `code`."""
        img = self.img
        state = {entry: [None] * 16}
        work = [entry]
        out = {}
        while work:
            a = work.pop()
            st = list(state[a])
            ins = img.insn(a)
            if a in wanted:
                out[a] = {r: st[r] for r in range(16)}
            _transfer(img, ins, st)
            succ = []
            if ins.delay:
                slot = img.insn(a + 2)
                if ins.op in ("jsr", "bsr", "bsrf"):
                    # the slot runs before the call clobbers
                    st2 = list(state[a])
                    _transfer(img, slot, st2)
                    _transfer(img, ins, st2)
                    st = st2
                    succ = [a + 4]
                else:
                    _transfer(img, slot, st)
                    if ins.op in ("bt/s", "bf/s"):
                        succ = [ins.target, a + 4]
                    elif ins.op == "bra":
                        succ = [ins.target]
            elif ins.op in ("bt", "bf"):
                succ = [ins.target, a + 2]
            elif ins.op not in ("rts", "rte", "jmp", "braf"):
                succ = [a + 2]
            if ins.op in ("braf", "jmp"):
                succ = (switches or {}).get(a) or self.switches.get(a, [])
                # `jmp @rn` to a constant the descent followed (a far branch,
                # or a tail into code not yet known as a function's entry)
                if not succ and ins.op == "jmp" and state[a][ins.n] in code:
                    succ = [state[a][ins.n]]
            for t in succ:
                if t not in code:
                    continue
                old = state.get(t)
                if old is None:
                    state[t] = st
                    work.append(t)
                else:
                    merged = [x if x == y else None for x, y in zip(old, st)]
                    if merged != old:
                        state[t] = merged
                        work.append(t)
        return out

    # -- descent
    def _descend(self, entry, commit=True):
        f = Function(entry)
        data, switches = set(), {}
        work, pending = [entry], []
        while True:
            while work:
                self._walk(f, work.pop(), data, switches, work, pending)
            if not pending:
                break
            # indirect calls and jumps left: constants over the function so far
            consts = self._constants(f.code, entry, [at for at, _ in pending], switches)
            for at, what in pending:
                v = consts.get(at, {}).get(self.img.insn(at).n)
                if v is None:
                    f.unresolved.append((at, what))
                    continue
                if what in ("bsrf", "braf"):
                    v = (at + 4 + v) & 0xFFFFFFFF
                if what in ("jsr", "bsrf"):
                    f.calls.add(v)
                elif (v in self.funcs and v != entry) or not self.inside(v):
                    f.tails.add(v)
                else:
                    work.append(v)
            pending = []
            if not work:
                break
        # code that the descent itself later marked as data is a contradiction
        clash = f.code & data
        if clash and not f.bad:
            f.bad = ("code is data", min(clash))
        if commit:
            self.funcs[entry] = f
            self.code |= f.code
            self.data |= data
            self.switches.update(switches)
            self.tables |= f.tables
        return f, data

    def _code_literal(self, ins, data):
        # not `mova`: its target may be a table, or code (hand-written assembly
        # takes a label's address with it and jumps to it)
        if ins.target is not None and ins.op in ("mov.w", "mov.l") and ins.size:
            for k in range(0, ins.size, 2):
                data.add(ins.target + k)

    def _walk(self, f, a, data, switches, work, pending):
        """Follow straight-line code from `a` until a terminator or known code."""
        img = self.img
        while True:
            if a in f.code and a not in f.slot_only:
                return
            if not self.inside(a):
                f.bad = f.bad or ("outside", a)
                return
            if a in self.data or a in data:
                f.bad = f.bad or ("into data", a)
                return
            ins = img.insn(a)
            if ins.op == ".word":
                f.bad = f.bad or ("undefined", a)
                return
            f.slot_only.discard(a)
            f.code.add(a)
            self._code_literal(ins, data)
            op = ins.op
            if ins.delay:
                slot = a + 2
                if self.inside(slot) and slot not in self.data:
                    s = img.insn(slot)
                    if s.op == ".word" or s.delay:
                        f.bad = f.bad or ("bad slot", slot)
                    else:
                        if slot not in f.code:
                            f.slot_only.add(slot)
                        f.code.add(slot)
                        self._code_literal(s, data)
            if op in ("bt", "bf"):
                work.append(ins.target)
                a += 2
            elif op in ("bt/s", "bf/s"):
                work.append(ins.target)
                a += 4
            elif op == "bsr":
                f.calls.add(ins.target)
                a += 4
            elif op == "bsrf":
                pending.append((a, "bsrf"))       # PC-relative: a + 4 + the register
                a += 4
            elif op == "jsr":
                r = self._literal_for(a, ins.n)
                if r and r[0] == "lit":
                    f.calls.add(r[1])
                elif r and r[0] == "ptr":
                    self.ext_calls[r[1]] += 1
                else:
                    pending.append((a, "jsr"))
                a += 4
            elif op == "bra":
                t = ins.target
                if t in self.funcs and t != f.entry:
                    f.tails.add(t)
                else:
                    work.append(t)
                    # GCC's switch computed here, dispatched by a `jmp @r0` after a pool
                    if self.inside(t) and t not in switches and img.insn(t).op == "jmp":
                        sw = self._mova_switch(t, before=a)
                        if sw:
                            targets, table, size = sw
                            switches[t] = targets
                            for k in range(0, size, 2):
                                data.add(table + k)
                                f.tables.add(table + k)
                            work.extend(targets)
                return
            elif op == "jmp":
                r = self._literal_for(a, ins.n)
                if a in switches:                  # a switch found at the bra that leads here
                    pass
                elif r and r[0] == "lit":
                    t = r[1]
                    if (t in self.funcs and t != f.entry) or not self.inside(t):
                        f.tails.add(t)
                    else:
                        work.append(t)
                elif r and r[0] == "ptr":
                    self.ext_calls[r[1]] += 1
                else:
                    sw = self._abs_switch(a) or self._byte_switch(a) or self._byte_switch_offset(a) \
                        or self._mova_switch(a) or self._record_switch(a)
                    if sw:
                        targets, table, size = sw
                        switches[a] = targets
                        for k in range(0, size, 2):
                            data.add(table + k)
                            f.tables.add(table + k)
                        work.extend(targets)
                    else:
                        pending.append((a, "jmp"))
                        # a computed jump into straight code near a label
                        # (`mova T,r0; add rX,r0; jmp @r0`, an unrolled copy
                        # entered part-way): T and the code up to it are the
                        # function's, so that the jump can land anywhere in it
                        t = self._mova_base(a)
                        if t is not None:
                            work.append(t)
                            if t > a:
                                work.append(a + 4)
                return
            elif op == "braf":
                sw = self._switch(a)
                if sw:
                    targets, table, size = sw
                    switches[a] = targets
                    for k in range(0, size, 2):
                        data.add(table + k)
                        f.tables.add(table + k)
                    work.extend(targets)
                else:
                    pending.append((a, "braf"))
                return
            elif op in ("rts", "rte"):
                return
            else:
                a += 2

    def run(self):
        while True:
            while self._pending:
                e = self._pending.pop()
                if e in self.funcs or not self.inside(e):
                    continue
                f, _ = self._descend(e)
                for t in sorted(f.calls | f.tails):
                    if t not in self.funcs and self.inside(t):
                        self._pending.append(t)
            added = self._pointer_seeds()
            if not added:
                added = self._prologue_seeds()
            if not added:
                added = self._address_taken()
            if not added:
                added = self._computed_seeds()
            if not added:
                break
        # branches into another function's entry found later are tail calls:
        # harmless for the recompiler (duplicated code), counted for the report
        self.shared = sum(1 for f in self.funcs.values()
                          for e in self.funcs if e != f.entry and e in f.code)
        self.landings = self._landings()

    def _landings(self):
        """Addresses returned to that no call returns to: `mov.l #X,rn; lds rn,pr`,
        then `rts` with PR unchanged (a task system's exit back into its
        scheduler, a longjmp). The recompiler makes them labels its calls can
        unwind to (emit.Body); the code there must be this program's."""
        img, out = self.img, set()
        for a in sorted(self.code):
            ins = img.insn(a)
            if ins.op != "lds" or ins.fmt != "lds Rm,pr":
                continue
            lit = self._literal_for(a, ins.m)
            if not lit or lit[0] != "lit" or lit[1] not in self.code:
                continue
            for b in range(a + 2, a + 12, 2):
                j = img.insn(b)
                if b not in self.code or j.op in ("sts.l", "lds.l", "lds", "jsr", "bsr", "bsrf") \
                        or j.op in TERMINATORS and j.op != "rts":
                    break
                if j.op == "rts":
                    out.add(lit[1])
                    break
        return out

    def _boundary(self, a):
        p = a - 2
        if p in self.data:
            return True
        q = a - 4
        if q in self.code:
            i = self.img.insn(q)
            return i.op in TERMINATORS and i.op != "braf"
        return p < self.lo or not self.img.contains(p)

    _BASE_M = ("@Rm,", "@Rm+", "@(disp,Rm)", "@(r0,Rm)")
    _BASE_N = ("@Rn", "@-Rn", "@(disp,Rn)", "@(r0,Rn)")

    def _data_addresses(self):
        """Literal values the code dereferences: `mov.l LIT,rX` followed, in
        the same straight line and before rX changes, by a load or store
        through rX. They point at data, whatever the bytes there decode to
        (a program's `main` reads the BSS end its crt0 keeps in its pool)."""
        img, out = self.img, self._data_addrs
        for a in self.code - self._scanned:
            ins = img.insn(a)
            if ins.op != "mov.l" or ins.size != 4 or ins.target is None:
                continue
            v = img.literal(ins)
            if v is None:
                continue
            r, b = ins.n, a + 2
            for _ in range(8):
                if b not in self.code:
                    break
                j = img.insn(b)
                if j.op in ("jsr", "jmp", "bsr", "bra", "bt", "bf", "bt/s", "bf/s", "rts", "rte", "braf", "bsrf"):
                    break
                f = j.fmt
                if (j.m == r and any(k in f for k in self._BASE_M)) or \
                        (j.n == r and any(k in f for k in self._BASE_N)):
                    out.add(v)
                    break
                if r in _writes(j):
                    break
                b += 2
        self._scanned |= self.code
        return out

    def _pointer_seeds(self):
        data_addrs = self._data_addresses()
        cands, tabled = set(), set()
        words = [a for a in self.data if not a & 3 and a + 2 in self.data]
        # and the unclassified words: pointer tables in the data section
        for start, n in self.gaps():
            words.extend(range((start + 3) & ~3, start + n - 3, 4))
        into = lambda w: self.img.contains(w, 4) and self.lo <= self.img.u32(w) < self.hi
        for a in sorted(words):
            if not self.img.contains(a, 4):
                continue
            v = self.img.u32(a)
            if self.inside(v) and v not in self.code and v not in self.data and v not in self._rejected \
                    and v not in data_addrs:
                cands.add(v)
                if into(a - 4) or into(a + 4):     # a word of a table of pointers
                    tabled.add(v)
        literals = self._literal_values()
        cands |= {v for v in literals if self.inside(v) and v not in self.code and v not in self.data
                  and v not in self._rejected and v not in data_addrs}
        added = 0
        for v in sorted(cands):
            # not after known code or data: it may become a boundary once its
            # neighbour is found, unless it opens with a stack frame (a pointer
            # to a prologue is a function, whatever unclassified words precede it:
            # an interrupt handler after a pool nothing reads), or the code loads
            # it as a literal (a handler or a callback handed to a function:
            # hand-written ones save r0-r7 first, not the callee-saved registers),
            # or it sits in a table of pointers (hand-written dispatch tables)
            if not self._boundary(v) and not (v & 1 == 0 and self._is_prologue(self.img.u16(v))) \
                    and v not in literals and v not in tabled:
                continue
            if self._is_text(v) or self._is_pointers(v):   # a string or a table passed as an argument
                self._rejected.add(v)
                continue
            f, data = self._descend(v, commit=False)
            if f.bad or f.code & self.data or data & self.code or self._through_pointers(f.code):
                self._rejected.add(v)
                continue
            # taken only as a literal or for its table: the address of a data
            # record (passed to a function, or in a table of pointers) can decode
            # cleanly for a few halfwords; a handler is longer
            # (not rejected for good: it may yet become a boundary)
            # (but `rts; nop` loaded as a literal is a callback that does nothing,
            # and a descent wholly in code already found is code: another entry
            # into it, as the short tail entries of an unrolled copy's table)
            weak = not self._boundary(v) and not self._is_prologue(self.img.u16(v))
            empty = v in literals and self.img.contains(v, 4) and self.img.u32(v) == 0x000B0009
            if weak and len(f.code) < 8 and not empty and not f.code <= self.code:
                continue
            self._descend(v)
            for t in f.calls | f.tails:
                if t not in self.funcs and self.inside(t):
                    self._pending.append(t)
            added += 1
        return added

    def _through_pointers(self, code):
        """The code runs over two consecutive words that point into the
        program: it is a table decoded as instructions."""
        return any(not a & 3 and a + 6 in code and self._is_pointers(a) for a in code)

    def _literal_values(self):
        """The 32-bit values the code loads from its literal pools, and the
        addresses `mova` takes and stores (`mova L,r0; mov.l r0,@rN`: a code
        address handed over, a slave's entry written to the BIOS's vector),
        incrementally."""
        out, img = self._lits, self.img
        for a in self.code - self._lit_scanned:
            ins = img.insn(a)
            if ins.op == "mov.l" and ins.size == 4 and ins.target is not None:
                v = img.literal(ins)
                if v is not None:
                    out.add(v)
            elif ins.op == "mova":
                for b in range(a + 2, a + 10, 2):
                    j = img.insn(b)
                    if j.op == "mov.l" and (j.m == 0 and j.fmt.startswith("mov.l Rm,@")
                                            or j.fmt == "mov.l r0,@(disp,gbr)"):
                        out.add(ins.target)
                        break
                    if 0 in _writes(j) or j.op in self._STOPS:
                        break
        self._lit_scanned |= self.code
        return out

    def _is_text(self, a):
        """A NUL-terminated run of printable ASCII (or tabs and newlines) starts
        at `a`. Code rarely looks like this: prologues and most ALU forms have
        a byte above 0x7E, and its NUL bytes come early (the high byte of `nop`,
        `rts`…). Two printable bytes and a NUL can still be code (`add r3,r4;
        nop` is "4<"), so a string shorter than 3 is taken only if zeros pad it
        to the next 4-byte boundary, as the compiler aligns strings."""
        img = self.img
        for k in range(64):
            if not img.contains(a + k, 1):
                return False
            c = img.data[a + k - img.base]
            if c == 0:
                if k >= 3:
                    return True
                end = (a + k + 4) & ~3
                return k > 0 and img.contains(a + k, end - a - k) and \
                    not any(img.data[a + k - img.base:end - img.base])
            if not (0x20 <= c < 0x7F or c in (9, 10, 13)):
                return False
        return False

    def _is_pointers(self, a):
        """Two 32-bit words into the program start at `a`: a pointer table or a
        literal pool, not code (`mov.b r0,@(r0,r6)` twice, each followed by a
        branch, does not open a function)."""
        img = self.img
        return not a & 3 and img.contains(a, 8) and \
            all(self.lo <= img.u32(a + k) < self.hi for k in (0, 4))

    def _address_taken(self):
        """Code addresses that appear as 32-bit literals become entries of their own.

        They are function pointers (callbacks, jump targets reached through a
        register) or tail calls made with `jmp`; followed so far as part of
        the function that reached them, they now also get their own entry.
        """
        added = 0
        for a in sorted(self.data - self.tables):
            if a & 3 or a + 2 not in self.data:
                continue
            v = self.img.u32(a)
            if v in self.code and v not in self.funcs and v not in self._rejected:
                f, data = self._descend(v, commit=False)
                if f.bad:
                    self._rejected.add(v)
                    continue
                self._descend(v)
                added += 1
        return added

    def _computed_seeds(self):
        """Addresses position-independent code computes, `mov.l #k,rn; mova T,r0;
        add r0,rn` (SHC's -pic), that nothing calls or jumps to directly: a
        function pointer handed on (a task's handler, a callback). Taken
        when they start at a boundary and descend into clean code that meets
        no known data; a data address the code computes the same way fails
        these as a prologue seed would."""
        img, added = self.img, 0
        todo = sorted(self.code - self._scanned_pic)
        self._scanned_pic |= set(todo)
        for a in todo:
            ins = img.insn(a)
            if ins.op != "add" or ins.fmt != "add Rm,Rn" or ins.m != 0 or ins.n == 0:
                continue
            t = None
            for b in range(a - 2, a - 34, -2):          # the mova that set r0, in the straight line
                if b not in self.code:
                    break
                j = img.insn(b)
                if j.op == "mova":
                    t = j.target
                    break
                if 0 in _writes(j) or j.op in TERMINATORS or j.op in ("bt", "bf", "bt/s", "bf/s", "jsr", "bsr", "bsrf"):
                    break
            lit = self._literal_for(a, ins.n) if t is not None else None
            if not lit or lit[0] != "lit":
                continue
            v = (t + lit[1]) & 0xFFFFFFFF
            if v & 1 or not self.inside(v) or v in self.funcs or v in self.data or v in self._rejected:
                continue
            known = v in self.code              # reached inside another function: an entry of its own too
            # a boundary, or right after a terminator and its slot, reached or not
            # (a `braf` too, when it is not a switch: SHC's -pic tail jump)
            after = self.inside(v - 4) and v - 4 not in self.switches and \
                sh2.decode(img.u16(v - 4), v - 4).op in ("rts", "rte", "bra", "jmp", "braf")
            # or after a literal pool, known or not (a halfword no instruction has)
            after = after or any(self.inside(v - k) and sh2.decode(img.u16(v - k), v - k).op == ".word"
                                 for k in (2, 4))
            if not known and not self._boundary(v) and not after:
                continue
            f, data = self._descend(v, commit=False)
            if f.bad or not known and (f.code & self.data or data & self.code):
                self._rejected.add(v)
                continue
            self._descend(v)
            for t2 in f.calls | f.tails:
                if t2 not in self.funcs and self.inside(t2):
                    self._pending.append(t2)
            added += 1
        return added

    @staticmethod
    def _is_prologue(w):
        # mov.l rN,@-r15 (N = 8..14) ; sts.l pr,@-r15 ; add #-N,r15
        return (w & 0xFF0F) == 0x2F06 and 8 <= (w >> 4) & 15 <= 14 or w == 0x4F22             or (w & 0xFF80) == 0x7F80

    def _prologue_seeds(self):
        """Unreached code that starts with a stack-frame prologue at a boundary:
        the start of a gap (after padding), or anywhere in it right after a
        terminator and its delay slot."""
        added = 0
        img = self.img
        for start, n in self.gaps():
            end = start + n
            a = start
            while a < end and img.u16(a) in (0x0009, 0x0000):
                a += 2
            cands = [a] if a < end and self._boundary(a) else []
            for p in range(start + 4, end, 2):
                if sh2.decode(img.u16(p - 4), p - 4).op in ("rts", "rte", "bra", "jmp"):
                    q = p
                    while q < end and img.u16(q) in (0x0009, 0x0000):
                        q += 2
                    cands.append(q)
            for c in cands:
                if c >= end or c in self._rejected or c in self.code or c in self.funcs:
                    continue
                if not self._is_prologue(img.u16(c)):
                    continue
                f, data = self._descend(c, commit=False)
                if f.bad or f.code & self.data or data & self.code:
                    self._rejected.add(c)
                    continue
                self._descend(c)
                for t in f.calls | f.tails:
                    if t not in self.funcs and self.inside(t):
                        self._pending.append(t)
                added += 1
        return added

    # -- report
    def gaps(self):
        """Runs of halfwords that are neither code nor data: (start, length)."""
        out, start = [], None
        for a in range(self.lo, self.hi, 2):
            known = a in self.code or a in self.data
            if not known and start is None:
                start = a
            elif known and start is not None:
                out.append((start, a - start))
                start = None
        if start is not None:
            out.append((start, self.hi - start))
        return out

    def summary(self):
        bad = [f for f in self.funcs.values() if f.bad]
        unres = [u for f in self.funcs.values() for u in f.unresolved]
        gaps = self.gaps()
        gap_bytes = sum(n for _, n in gaps)
        return {
            "functions": len(self.funcs),
            "code bytes": len(self.code) * 2,
            "data bytes": len(self.data) * 2,
            "unclassified bytes": gap_bytes,
            "switch tables": len(self.switches),
            "unresolved indirect": len(unres),
            "functions with problems": len(bad),
            "entries inside other functions": self.shared,
            "calls through pointers": sum(self.ext_calls.values()),
        }


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("file")
    ap.add_argument("--base", default="0600B000")
    ap.add_argument("--seeds", default="", help="extra entry points, hex, comma-separated")
    ap.add_argument("--end", help="end of text (hex), default the end of the file")
    ap.add_argument("--report", action="store_true", help="list problems, unresolved jumps, gaps")
    ap.add_argument("--out", help="write the function list (entry, size of reached code) as TSV")
    a = ap.parse_args(argv)
    data = open(a.file, "rb").read()
    base = int(a.base, 16)
    img = sh2.Image(data, base)
    seeds = [base] + [int(s, 16) for s in a.seeds.split(",") if s]
    bounds = (base, int(a.end, 16)) if a.end else None
    p = Program(img, seeds, bounds)
    for k, v in p.summary().items():
        print("%-32s %d" % (k, v))
    if a.report:
        print("\nfunctions with problems:")
        for f in sorted(p.funcs.values(), key=lambda f: f.entry):
            if f.bad:
                print("  %08X  %s at %08X" % (f.entry, f.bad[0], f.bad[1]))
        print("\nunresolved indirect jumps:")
        for f in sorted(p.funcs.values(), key=lambda f: f.entry):
            for at, what in f.unresolved:
                print("  %08X  %s (in %08X)" % (at, what, f.entry))
        print("\ncalls through pointers (pointer address: count):")
        for ptr, n in p.ext_calls.most_common():
            from .. import hw
            print("  %08X  %4d  %s" % (ptr, n, hw.name(ptr) or ""))
        print("\nlargest unclassified runs:")
        for s, n in sorted(p.gaps(), key=lambda g: -g[1])[:25]:
            print("  %08X  %6d bytes" % (s, n))
    if a.out:
        with open(a.out, "w") as fo:
            for e in sorted(p.funcs):
                fo.write("%08X\t%d\n" % (e, len(p.funcs[e].code) * 2))


if __name__ == "__main__":
    main()
