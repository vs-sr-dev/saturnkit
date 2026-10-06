"""SH-2 instruction -> C++ statement(s), against saturnkit/runtime/sh2.h.

stmt(ins, literal) gives the C++ for one instruction that does not branch,
with the semantics of saturnkit/sh2emu.py. Body turns a discovered function
into a C++ function:

* a label wherever something jumps; straight-line order otherwise, with a
  `goto` where the code continues past a literal pool;
* a delayed branch reads its target and its condition (`bt/s` reads T,
  `jsr @rn` reads rn, `rts` reads PR), runs the slot, then goes;
* `bsr` and `jsr`/`bsrf` as C++ calls with PR set first, the return checked
  (SH2_RET); a branch to another entry as a tail call; `rts`/`rte` return;
* computed jumps (`jmp`, `braf`) as a `switch` over the targets analysis
  found: switch tables, literals, constants; one left unresolved gets every
  instruction of its own function as a case. Whatever the switch does not
  know goes to sh2_call, as a tail. Calls through registers compare with the
  target analysis expects and otherwise dispatch: a wrong guess costs a
  lookup, never a wrong call;
* safe points (SH2_POLL) at loop back-edges, before calls and after
  `ldc ...,sr`;
* in a function holding a landing (an address the program returns to
  without a call, discover.Program.landings: a longjmp's target), every call
  catches SH2Unwind and goes on at the landing it names; the runtime throws
  it when a return goes to a landing instead of the call's own address.

Every multi-statement instruction is its own block, so a `goto` never
crosses an initialisation.
"""


class Unsupported(Exception):
    pass


def fname(addr):
    return "f_%08X" % addr


def _h(v):
    v &= 0xFFFFFFFF
    return "0x%08Xu" % v if v > 0xFFFF else "0x%Xu" % v


def _r(n):
    return "c.r[%d]" % n


def _plus(base, d):
    return "%s + %d" % (base, d) if d else base


_LD = {1: "ld8", 2: "ld16", 4: "ld32"}
_ST = {1: "st8", 2: "st16", 4: "st32"}
_SIZE = {"mov.b": 1, "mov.w": 2, "mov.l": 4}


def _ext(size, e):
    return {1: "(uint32_t)(int8_t)%s", 2: "(uint32_t)(int16_t)%s", 4: "%s"}[size] % e


def _mov(ins, form, size, literal):
    n, m, d = ins.n, ins.m, ins.disp
    ld, st = _LD[size], _ST[size]
    R = _r
    if form == "@T,Rn":                                  # pc-relative literal
        if literal is not None:
            return "%s = %s;" % (R(n), _h(literal))
        return "%s = %s;" % (R(n), _ext(size, "%s(%s)" % (ld, _h(ins.target))))
    if form == "Rm,@Rn":
        return "%s(%s, %s);" % (st, R(n), R(m))
    if form == "@Rm,Rn":
        return "%s = %s;" % (R(n), _ext(size, "%s(%s)" % (ld, R(m))))
    if form == "Rm,@-Rn":
        if n == m:
            return "{ uint32_t v = %s; %s -= %d; %s(%s, v); }" % (R(m), R(n), size, st, R(n))
        return "%s -= %d; %s(%s, %s);" % (R(n), size, st, R(n), R(m))
    if form == "@Rm+,Rn":
        load = "%s = %s;" % (R(n), _ext(size, "%s(%s)" % (ld, R(m))))
        return load if n == m else "%s %s += %d;" % (load, R(m), size)
    if form == "Rm,@(r0,Rn)":
        return "%s(c.r[0] + %s, %s);" % (st, R(n), R(m))
    if form == "@(r0,Rm),Rn":
        return "%s = %s;" % (R(n), _ext(size, "%s(c.r[0] + %s)" % (ld, R(m))))
    if form == "r0,@(disp,Rn)":
        return "%s(%s, c.r[0]);" % (st, _plus(R(n), d))
    if form == "@(disp,Rm),r0":
        return "c.r[0] = %s;" % _ext(size, "%s(%s)" % (ld, _plus(R(m), d)))
    if form == "Rm,@(disp,Rn)":
        return "%s(%s, %s);" % (st, _plus(R(n), d), R(m))
    if form == "@(disp,Rm),Rn":
        return "%s = %s(%s);" % (R(n), ld, _plus(R(m), d))
    if form == "r0,@(disp,gbr)":
        return "%s(%s, c.r[0]);" % (st, _plus("c.gbr", d))
    if form == "@(disp,gbr),r0":
        return "c.r[0] = %s;" % _ext(size, "%s(%s)" % (ld, _plus("c.gbr", d)))
    raise Unsupported(ins.text)


_SREG = {"sr": "sh2_get_sr(c)", "gbr": "c.gbr", "vbr": "c.vbr", "mach": "c.mach",
         "macl": "c.macl", "pr": "c.pr"}


def stmt(ins, literal=None):
    """C++ for one instruction that does not branch (`literal`: the value a
    pc-relative load reads, or None to read memory)."""
    op, f, n, m, imm = ins.op, ins.fmt, ins.n, ins.m, ins.imm
    form = f.split(" ", 1)[1] if " " in f else ""
    R = _r
    if op == "nop":
        return ";"
    if op in _SIZE:
        return _mov(ins, form, _SIZE[op], literal)
    if op == "mov":
        return "%s = %s;" % (R(n), _h(imm) if imm is not None else R(m))
    if op == "mova":
        return "c.r[0] = %s;" % _h(ins.target)
    if op == "movt":
        return "%s = c.t;" % R(n)
    if op == "swap.b":
        return "%s = (%s & 0xFFFF0000u) | (%s & 0xFFu) << 8 | (%s >> 8 & 0xFFu);" % (R(n), R(m), R(m), R(m))
    if op == "swap.w":
        return "%s = %s << 16 | %s >> 16;" % (R(n), R(m), R(m))
    if op == "xtrct":
        return "%s = %s << 16 | %s >> 16;" % (R(n), R(m), R(n))
    # ----- arithmetic
    if op == "add":
        if imm is not None:
            return "%s %s= %d;" % (R(n), "+" if imm >= 0 else "-", abs(imm))
        return "%s += %s;" % (R(n), R(m))
    if op == "addc":
        return ("{ uint64_t s = (uint64_t)%s + %s + c.t; c.t = (uint32_t)(s >> 32); %s = (uint32_t)s; }"
                % (R(n), R(m), R(n)))
    if op in ("addv", "subv"):
        return ("{ int64_t s = (int64_t)(int32_t)%s %s (int32_t)%s; c.t = s > INT32_MAX || s < INT32_MIN; "
                "%s = (uint32_t)s; }" % (R(n), "+" if op == "addv" else "-", R(m), R(n)))
    if op == "sub":
        return "%s -= %s;" % (R(n), R(m))
    if op == "subc":
        return ("{ uint32_t a = %s, b = %s, t = c.t; %s = a - b - t; c.t = (uint64_t)a < (uint64_t)b + t; }"
                % (R(n), R(m), R(n)))
    if op == "neg":
        return "%s = 0u - %s;" % (R(n), R(m))
    if op == "negc":
        return ("{ uint32_t b = %s, t = c.t; %s = 0u - b - t; c.t = (uint64_t)b + t != 0; }" % (R(m), R(n)))
    if op == "dt":
        return "%s -= 1; c.t = %s == 0;" % (R(n), R(n))
    if op == "exts.b":
        return "%s = (uint32_t)(int32_t)(int8_t)%s;" % (R(n), R(m))
    if op == "exts.w":
        return "%s = (uint32_t)(int32_t)(int16_t)%s;" % (R(n), R(m))
    if op == "extu.b":
        return "%s = %s & 0xFFu;" % (R(n), R(m))
    if op == "extu.w":
        return "%s = %s & 0xFFFFu;" % (R(n), R(m))
    # ----- compare
    if op == "cmp/eq":
        if imm is not None:
            return "c.t = c.r[0] == %s;" % _h(imm)
        return "c.t = %s == %s;" % (R(n), R(m))
    if op in ("cmp/hs", "cmp/hi"):
        return "c.t = %s %s %s;" % (R(n), ">=" if op == "cmp/hs" else ">", R(m))
    if op in ("cmp/ge", "cmp/gt"):
        return "c.t = (int32_t)%s %s (int32_t)%s;" % (R(n), ">=" if op == "cmp/ge" else ">", R(m))
    if op in ("cmp/pz", "cmp/pl"):
        return "c.t = (int32_t)%s %s 0;" % (R(n), ">=" if op == "cmp/pz" else ">")
    if op == "cmp/str":
        return ("{ uint32_t x = %s ^ %s; c.t = !(x & 0xFFu) || !(x & 0xFF00u) || !(x & 0xFF0000u) "
                "|| !(x & 0xFF000000u); }" % (R(n), R(m)))
    # ----- logic
    if op in ("and", "or", "xor"):
        sym = {"and": "&", "or": "|", "xor": "^"}[op]
        if imm is not None:
            return "c.r[0] %s= %s;" % (sym, _h(imm))
        return "%s %s= %s;" % (R(n), sym, R(m))
    if op == "tst":
        if imm is not None:
            return "c.t = (c.r[0] & %s) == 0;" % _h(imm)
        return "c.t = (%s & %s) == 0;" % (R(n), R(m))
    if op in ("and.b", "or.b", "xor.b"):
        sym = {"and.b": "&", "or.b": "|", "xor.b": "^"}[op]
        return "{ uint32_t a = c.gbr + c.r[0]; st8(a, ld8(a) %s %s); }" % (sym, _h(imm))
    if op == "tst.b":
        return "c.t = (ld8(c.gbr + c.r[0]) & %s) == 0;" % _h(imm)
    if op == "not":
        return "%s = ~%s;" % (R(n), R(m))
    if op == "tas.b":
        return "{ uint32_t a = %s, v = ld8(a); c.t = v == 0; st8(a, v | 0x80u); }" % R(n)
    # ----- shifts and rotates
    if op in ("shll", "shal"):
        return "c.t = %s >> 31; %s <<= 1;" % (R(n), R(n))
    if op == "shlr":
        return "c.t = %s & 1; %s >>= 1;" % (R(n), R(n))
    if op == "shar":
        return "c.t = %s & 1; %s = (uint32_t)((int32_t)%s >> 1);" % (R(n), R(n), R(n))
    if op in ("shll2", "shll8", "shll16"):
        return "%s <<= %s;" % (R(n), op[4:])
    if op in ("shlr2", "shlr8", "shlr16"):
        return "%s >>= %s;" % (R(n), op[4:])
    if op == "rotl":
        return "c.t = %s >> 31; %s = %s << 1 | c.t;" % (R(n), R(n), R(n))
    if op == "rotr":
        return "c.t = %s & 1; %s = %s >> 1 | c.t << 31;" % (R(n), R(n), R(n))
    if op == "rotcl":
        return "{ uint32_t t = %s >> 31; %s = %s << 1 | c.t; c.t = t; }" % (R(n), R(n), R(n))
    if op == "rotcr":
        return "{ uint32_t t = %s & 1; %s = %s >> 1 | c.t << 31; c.t = t; }" % (R(n), R(n), R(n))
    # ----- multiply and divide
    if op == "mul.l":
        return "c.macl = %s * %s;" % (R(n), R(m))
    if op == "muls.w":
        return "c.macl = (uint32_t)((int32_t)(int16_t)%s * (int16_t)%s);" % (R(n), R(m))
    if op == "mulu.w":
        return "c.macl = (%s & 0xFFFFu) * (%s & 0xFFFFu);" % (R(n), R(m))
    if op == "dmuls.l":
        return ("{ uint64_t p = (uint64_t)((int64_t)(int32_t)%s * (int32_t)%s); "
                "c.mach = (uint32_t)(p >> 32); c.macl = (uint32_t)p; }" % (R(n), R(m)))
    if op == "dmulu.l":
        return ("{ uint64_t p = (uint64_t)%s * %s; c.mach = (uint32_t)(p >> 32); c.macl = (uint32_t)p; }"
                % (R(n), R(m)))
    if op == "mac.w":
        return "sh2_mac_w(c, %d, %d);" % (n, m)
    if op == "mac.l":
        return "sh2_mac_l(c, %d, %d);" % (n, m)
    if op == "clrmac":
        return "c.mach = c.macl = 0;"
    if op == "div0u":
        return "c.m = c.q = c.t = 0;"
    if op == "div0s":
        return "c.q = %s >> 31; c.m = %s >> 31; c.t = c.q != c.m;" % (R(n), R(m))
    if op == "div1":
        return "sh2_div1(c, %s, %s);" % (R(n), R(m))
    # ----- T and the system registers
    if op == "clrt":
        return "c.t = 0;"
    if op == "sett":
        return "c.t = 1;"
    if op in ("ldc", "ldc.l", "lds", "lds.l"):
        reg = form.split(",")[1]
        if op.endswith(".l"):
            head, v = "uint32_t v = ld32(%s); %s += 4; " % (R(m), R(m)), "v"
        else:
            head, v = "", R(m)
        if reg == "sr":
            body = "sh2_set_sr(c, %s & 0x3F3u); SH2_POLL(c);" % v
        else:
            body = "c.%s = %s;" % (reg, v)
        return "{ %s%s }" % (head, body) if head else body
    if op in ("stc", "stc.l", "sts", "sts.l"):
        reg = form.split(",")[0]
        if op.endswith(".l"):
            return "%s -= 4; st32(%s, %s);" % (R(n), R(n), _SREG[reg])
        return "%s = %s;" % (R(n), _SREG[reg])
    if op == "trapa":
        return "sh2_trapa(c, %d, %s);" % (imm, _h(ins.pc + 2))
    if op == "sleep":
        return "sh2_sleep(c, %s);" % _h(ins.pc)
    raise Unsupported(ins.text)


BRANCHES = {"bt", "bf", "bt/s", "bf/s", "bra", "bsr", "jmp", "jsr", "braf", "bsrf", "rts", "rte"}


class Body:
    """One discovered function as C++.

    `entries`: this program's entry points (direct calls and tail calls go
    to them by name); `volatile`: literal addresses not to fold into
    constants. After emit(): `sites` counts the calls and jumps by how they
    were resolved, `unknown` lists static targets that are not entries here.
    """

    def __init__(self, prog, f, entries, volatile=frozenset(), comments=True, hooks=frozenset()):
        self.p, self.f, self.img = prog, f, prog.img
        self.entries, self.volatile, self.comments = entries, volatile, comments
        self.hooks = hooks
        self.ops = {a: prog.img.insn(a) for a in f.code}
        self.slots = {a + 2 for a, i in self.ops.items() if i.delay and a + 2 in f.code}
        self.sites = {}
        self.unknown = []
        self._targets = self._resolve()
        self.land = sorted(set(getattr(prog, "landings", ())) & f.code)

    # -- analysis
    def _resolve(self):
        """{address of a computed call/jump: (candidates, how)}."""
        p, out, need = self.p, {}, []
        for a, ins in self.ops.items():
            if ins.op not in ("jmp", "braf", "jsr", "bsrf"):
                continue
            if a in p.switches:
                out[a] = (list(dict.fromkeys(p.switches[a])), "switch")
                continue
            if ins.op in ("jmp", "jsr"):
                lit = p._literal_for(a, ins.n)
                if lit and lit[0] == "lit" and lit[2] not in self.volatile:
                    out[a] = ([lit[1]], "literal")
                    continue
                if lit and lit[0] == "lit":         # a slot the code writes: its value at run time
                    out[a] = ([], "unresolved")
                    continue
                if lit and lit[0] == "ptr":
                    out[a] = ([], "pointer")
                    continue
            need.append(a)
        if need:
            consts = p._constants(self.f.code, self.f.entry, need)
            for a in need:
                ins = self.ops[a]
                v = consts.get(a, {}).get(ins.n)
                if v is None:
                    out[a] = ([], "unresolved")
                elif ins.op in ("braf", "bsrf"):
                    out[a] = ([(a + 4 + v) & 0xFFFFFFFF], "constant")
                else:
                    out[a] = ([v], "constant")
        return out

    def _literal(self, ins):
        if ins.op in ("mov.w", "mov.l") and ins.size and ins.target not in self.volatile:
            return self.img.literal(ins)
        return None

    def _stmt(self, ins):
        return stmt(ins, self._literal(ins))

    # -- control flow
    def _goto(self, t, src):
        if t not in self.f.code:
            return "{ %s }" % self._tail(t)
        if t <= src:
            return "SH2_POLL(c); goto L_%08X;" % t
        return "goto L_%08X;" % t

    def _tail(self, t):
        """Leave for `t`, a static target outside this function's code."""
        if t in self.entries:
            return "%s(c); return;" % fname(t)
        self.unknown.append(t)
        return "sh2_call(c, %s); return;" % _h(t)

    def _call(self, t):
        if t in self.entries:
            return "%s(c);" % fname(t)
        self.unknown.append(t)
        return "sh2_call(c, %s);" % _h(t)

    def _dcall(self, a):
        """The call of a jsr/bsrf through `x`, guarded by what analysis expects."""
        cands, how = self._targets[a]
        self.sites[how] = self.sites.get(how, 0) + 1
        known = [v for v in cands if v in self.entries]
        self.unknown += [v for v in cands if v not in self.entries]
        if not known:
            return "sh2_call(c, x);"
        if len(known) == 1:
            return "if (x == %s) %s(c); else sh2_call(c, x);" % (_h(known[0]), fname(known[0]))
        cases = " ".join("case %s: %s(c); break;" % (_h(v), fname(v)) for v in known)
        return "switch (x) { %s default: sh2_call(c, x); }" % cases

    def _djump(self, a):
        """The jump of a jmp/braf through `x`: local targets as gotos, other
        entries as tail calls, the rest dispatched."""
        cands, how = self._targets[a]
        if how == "unresolved":
            how, cands = "unresolved jump", self._fallback
        self.sites[how] = self.sites.get(how, 0) + 1
        code = self.f.code
        cases, back = [], False
        for v in dict.fromkeys(cands):
            if v in code:
                cases.append("case %s: goto L_%08X;" % (_h(v), v))
                back |= v <= a
            elif v in self.entries:
                cases.append("case %s: %s(c); return;" % (_h(v), fname(v)))
            else:
                self.unknown.append(v)
        poll = "SH2_POLL(c); " if back else ""
        if not cases:
            return "sh2_call(c, x); return;"
        return "%sswitch (x) { %s default: sh2_call(c, x); return; }" % (poll, " ".join(cases))

    def _unwind(self, call):
        """A call that a longjmp to one of this function's landings unwinds to."""
        if not self.land:
            return call
        cases = " ".join("if (u.pc == %s) goto L_%08X;" % (_h(t), t) for t in self.land)
        return "try { %s } catch (const SH2Unwind& u) { %s throw; }" % (call, cases)

    def labels(self):
        out = set(self.land)
        code = self.f.code
        self._fallback = sorted(a for a in code if a not in self.slots)
        for a, ins in self.ops.items():
            if ins.op in ("bt", "bf", "bt/s", "bf/s", "bra") and ins.target in code:
                out.add(ins.target)
            elif a in self._targets and ins.op in ("jmp", "braf"):
                cands, how = self._targets[a]
                out |= {v for v in (self._fallback if how == "unresolved" else cands) if v in code}
        return out

    def _insn(self, a):
        """(C++ for the instruction at `a`, the address it continues at or None)."""
        ins = self.ops[a]
        op = ins.op
        if a in self.hooks:
            if op in BRANCHES or a in self.slots:
                raise Unsupported("a hook at %08X, on a branch or a delay slot: %s" % (a, ins.text))
            return "%s sh2_hook(c, %s);" % (self._stmt(ins), _h(a)), a + 2
        if op not in BRANCHES:
            return self._stmt(ins), a + 2
        if op in ("bt", "bf"):
            return "if (%sc.t) { %s }" % ("" if op == "bt" else "!", self._goto(ins.target, a)), a + 2
        slot = self._stmt(self.img.insn(a + 2))
        if slot == ";":
            slot = ""
        s = slot + " " if slot else ""
        ret = _h(a + 4)
        if op in ("bt/s", "bf/s"):
            return ("{ const bool k = %sc.t; %sif (k) { %s } }"
                    % ("" if op == "bt/s" else "!", s, self._goto(ins.target, a)), a + 4)
        if op == "bra":
            return s + self._goto(ins.target, a), None
        if op == "bsr":
            return ("c.pr = %s; %sSH2_POLL(c); %s SH2_RET(c, %s);"
                    % (ret, s, self._unwind(self._call(ins.target)), ret), a + 4)
        if op in ("jsr", "bsrf"):
            x = _r(ins.n) if op == "jsr" else "%s + %s" % (ret, _r(ins.n))
            return ("{ const uint32_t x = %s; c.pr = %s; %sSH2_POLL(c); %s SH2_RET(c, %s); }"
                    % (x, ret, s, self._unwind(self._dcall(a)), ret), a + 4)
        if op in ("jmp", "braf"):
            x = _r(ins.n) if op == "jmp" else "%s + %s" % (ret, _r(ins.n))
            return "{ const uint32_t x = %s; %s%s }" % (x, s, self._djump(a)), None
        if op == "rts":
            return "{ const uint32_t x = c.pr; %sc.pc = x; return; }" % s, None
        if op == "rte":
            return ("{ const uint32_t x = ld32(c.r[15]); c.r[15] += 4; sh2_set_sr(c, ld32(c.r[15]) & 0x3F3u); "
                    "c.r[15] += 4; %sc.pc = x; return; }" % s), None
        raise Unsupported(ins.text)

    def emit(self):
        """The C++ function, as a list of lines."""
        labels = self.labels()
        code = self.f.code
        order = sorted(a for a in code if a not in self.slots or a in labels or a == self.f.entry)
        lines = ["void %s(SH2Context& c) {" % fname(self.f.entry)]
        if order and order[0] != self.f.entry:            # code shared from below the entry
            labels.add(self.f.entry)
            lines.append("    goto L_%08X;" % self.f.entry)
        late = set()
        for i, a in enumerate(order):
            if a in labels or a in late:
                lines.append("L_%08X:" % a)
            text, succ = self._insn(a)
            if self.comments:
                ins = self.ops[a]
                note = ins.text
                if ins.delay:
                    note += " ; " + self.img.insn(a + 2).text
                text += "  // %08X %s" % (a, note)
            lines.append("    " + text)
            nxt = order[i + 1] if i + 1 < len(order) else None
            if succ is not None and succ != nxt:
                if succ in code:
                    late.add(succ)
                    lines.append("    goto L_%08X;" % succ)
                else:
                    lines.append("    " + self._tail(succ))
        lines.append("}")
        return lines
