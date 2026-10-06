"""Recompile flat SH-2 programs to C++, one module per program.

    python -m saturnkit.recomp --out build/recomp NAME=FILE@BASE[+SEED,...] ... [--optest]
                               [--per-file 8000] [--no-comments] [--hook NAME:ADDR,...]
                               [--overlay NAME,...]

Each program is discovered (recomp/discover.py) and emitted into its own
namespace, p_<name>. Programs loaded at the same address are separate
modules; the runtime activates the one whose image it finds in memory
(sh2_identify, by the image's crc32). Writes, into --out:

    p_<name>_funcs.h        prototypes and the module descriptor
    p_<name>_NNN.cpp        the functions, split by instruction count
    p_<name>_table.cpp      the module: image base, size, crc32, entries
    modules.cpp             every module of the build (g_sh2_modules)
    CMakeLists.txt          the library `recomp`, the runtime, the self-test
    report.txt              per module: functions, calls and jumps by how
                            they were resolved, targets outside the module

--hook NAME:ADDR,... calls sh2_hook(c, ADDR) after the instruction at ADDR
in program NAME (not a branch, not a delay slot): a place where the
runtime can change what the game computed (runtime/core.cpp, `--hook` of
the saturn executable).

--overlay NAME,... marks those programs as overlays: a call to their base
is an ordinary call that returns to its caller (a resident program that
loads them one after another at one address and calls each), not a
program start that resets the stack (runtime/core.cpp, sh2_call).

--optest adds saturnkit's own instruction test (recomp/selftest.py): a
synthetic program with every SH-2 instruction form, as module OPTEST, and
its vectors in --out/selftest/optest.txt.
"""
import argparse
import os
import time
import zlib

from .. import sh2
from . import discover
from . import emit as E

RUNTIME = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "runtime"))


class Out:
    """A generated file, written only if its content changed: a rebuild after
    a small change recompiles only what it touched."""

    def __init__(self, path):
        self.path, self.parts = path, []

    def write(self, s):
        self.parts.append(s)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        text = "".join(self.parts)
        try:
            with open(self.path, encoding="utf-8", newline="") as f:
                if f.read() == text:
                    return
        except OSError:
            pass
        with open(self.path, "w", encoding="utf-8", newline="") as f:
            f.write(text)


def parse_spec(spec):
    """NAME=FILE@BASE[+SEED,SEED...] -> (name, path, base, seeds)."""
    name, _, rest = spec.partition("=")
    path, _, where = rest.rpartition("@")
    base, _, seeds = where.partition("+")
    return name, path, int(base, 16), [int(s, 16) for s in seeds.split(",") if s]


def volatile_literals(prog):
    """Literal-pool slots that some 32-bit literal points at, or that a `mova`
    takes to store through (`mova SLOT,r0; mov.l rN,@r0`, hand-written
    assembly keeping its variables in its pools): data the code may write,
    so their loads read memory instead of folding the constant."""
    img, pools, values = prog.img, {}, set()
    for a in prog.code:
        ins = img.insn(a)
        if ins.op == "mova":
            # stores through r0 in the straight line after, until r0 changes
            for b in range(a + 2, a + 12, 2):
                if b not in prog.code:
                    break
                j = img.insn(b)
                if j.n == 0 and j.fmt in ("mov.l Rm,@Rn", "mov.w Rm,@Rn", "mov.b Rm,@Rn"):
                    values.add(ins.target)
                elif j.n == 0 and j.fmt in ("mov.l Rm,@(disp,Rn)", "mov.w r0,@(disp,Rn)", "mov.b r0,@(disp,Rn)"):
                    values.add(ins.target + j.disp)
                if j.op in discover.Program._STOPS or j.op in ("bt", "bf", "bt/s", "bf/s", "jsr", "bsr") \
                        or 0 in discover._writes(j):
                    break
        if ins.op in ("mov.w", "mov.l") and ins.size and ins.target is not None:
            pools[ins.target] = ins.size
            if ins.size == 4:
                v = img.literal(ins)
                if v is not None:
                    values.add(v & 0x1FFFFFFF if v >> 29 == 1 else v)
    out = set()
    for v in values:
        for t in (v, v & ~1, v & ~3, (v & ~1) - 2):
            if t in pools and t <= v < t + pools[t]:
                out.add(t)
    return out


class Module:
    def __init__(self, name, path, base, seeds=(), hooks=(), overlay=False):
        self.name, self.path, self.base, self.overlay = name, path, base, overlay
        self.hooks = frozenset(hooks)
        self.ns = "p_" + name.lower()
        self.data = open(path, "rb").read()
        self.crc = zlib.crc32(self.data)
        self.img = sh2.Image(self.data, base)
        self.prog = discover.Program(self.img, [base] + list(seeds))
        self.entries = sorted(self.prog.funcs)
        self.files, self.sites, self.unknown = [], {}, []
        self.n_ins = self.n_calls = 0

    def generate(self, out, per_file, comments, log):
        prog, entries = self.prog, set(self.entries)
        volatile = volatile_literals(prog)
        self.volatile = len(volatile)
        buf, count, idx = [], 0, 0

        def flush():
            nonlocal buf, count, idx
            if not buf:
                return
            name = "%s_%03d.cpp" % (self.ns, idx)
            with Out(os.path.join(out, name)) as f:
                f.write('#include "%s_funcs.h"\n\nnamespace %s {\n\n' % (self.ns, self.ns))
                f.write("\n\n".join(buf))
                f.write("\n\n}  // namespace %s\n" % self.ns)
            self.files.append(name)
            buf, count, idx = [], 0, idx + 1

        for e in self.entries:
            fn = prog.funcs[e]
            body = E.Body(prog, fn, entries, volatile, comments, self.hooks & set(fn.code))
            buf.append("\n".join(body.emit()))
            for k, v in body.sites.items():
                self.sites[k] = self.sites.get(k, 0) + v
            self.unknown += [(t, e) for t in body.unknown]
            self.n_calls += sum(1 for a in fn.code if body.ops[a].op == "bsr")
            count += len(fn.code)
            self.n_ins += len(fn.code)
            if count >= per_file:
                flush()
        flush()

        with Out(os.path.join(out, self.ns + "_funcs.h")) as f:
            f.write('#pragma once\n#include "sh2.h"\n\nnamespace %s {\n' % self.ns)
            for e in self.entries:
                f.write("void %s(SH2Context& c);\n" % E.fname(e))
            f.write("extern const SH2Module module;\n}  // namespace %s\n" % self.ns)
        with Out(os.path.join(out, self.ns + "_table.cpp")) as f:
            f.write('#include "%s_funcs.h"\n\nnamespace %s {\n\n' % (self.ns, self.ns))
            f.write("static const SH2FuncEntry funcs[] = {\n")
            for e in self.entries:
                f.write("    {0x%08Xu, %s},\n" % (e, E.fname(e)))
            f.write("};\n\n")
            land = sorted(prog.landings)
            if land:
                f.write("static const uint32_t landings[] = {%s};\n\n" % ", ".join("0x%08Xu" % t for t in land))
            f.write('extern const SH2Module module = {"%s", 0x%08Xu, %du, 0x%08Xu, funcs, %d, %s, %s, %d};\n'
                    % (self.name, self.base, len(self.data), self.crc, len(self.entries),
                       "SH2_MODULE_OVERLAY" if self.overlay else "0", "landings" if land else "nullptr", len(land)))
            f.write("\n}  // namespace %s\n" % self.ns)
        self.files.append(self.ns + "_table.cpp")


def classify(modules):
    """Targets outside their own module's entries: {module: (entry of another
    module that can be loaded with it, not an entry anywhere, outside every
    module)}. Programs at the same base replace each other, so an address in
    another program at one's own base counts as not an entry."""
    out = {}
    for m in modules:
        other = inside = outside = 0
        seen = set()
        for t, _ in m.unknown:
            if t in seen:
                continue
            seen.add(t)
            homes = [o for o in modules if o.base <= t < o.base + len(o.data)
                     and (o is m or o.base != m.base)]
            if any(t in o.prog.funcs for o in homes):
                other += 1
            elif homes or m.base <= t < m.base + len(m.data):
                inside += 1
            else:
                outside += 1
        out[m.name] = (other, inside, outside)
    return out


CMAKE = """cmake_minimum_required(VERSION 3.20)
project(saturnkit_recomp CXX)
set(CMAKE_CXX_STANDARD 20)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(SATURNKIT_RUNTIME "{rt}")
add_library(recomp STATIC
    {srcs})
target_include_directories(recomp PUBLIC ${{SATURNKIT_RUNTIME}})
target_compile_options(recomp PRIVATE -Wno-unused-label -Wno-tautological-compare)
# the runtime: saturnkit_core (memory, dispatch over the modules),
# saturnkit_stub (no hardware: tests that only run the game's own code) and
# saturnkit_hw (the Saturn)
include(${{SATURNKIT_RUNTIME}}/runtime.cmake)
add_executable(selftest ${{SATURNKIT_RUNTIME}}/selftest.cpp)
target_link_libraries(selftest saturnkit_core saturnkit_stub recomp)
# the Saturn with no screen: boots a disc into these modules
add_executable(saturn ${{SATURNKIT_RUNTIME}}/main.cpp)
target_link_libraries(saturn saturnkit_core saturnkit_hw recomp Threads::Threads)
# a project can add its own targets (the game) with -DSATURNKIT_EXTRA=file.cmake
if(DEFINED SATURNKIT_EXTRA)
  include(${{SATURNKIT_EXTRA}})
endif()
"""


def generate(specs, out, per_file=8000, comments=True, optest=False, log=print, hooks=None, overlays=()):
    """hooks: {program name: [address, ...]}, see --hook; overlays: program names, see --overlay."""
    hooks = hooks or {}
    overlays = set(overlays)
    unknown = overlays - {s[0] for s in specs}
    if unknown:
        raise SystemExit("--overlay %s: no such program" % ", ".join(sorted(unknown)))
    t0 = time.time()
    os.makedirs(out, exist_ok=True)
    if optest:
        from . import selftest
        path, base, entries = selftest.write_optest(os.path.join(out, "selftest"))
        specs = list(specs) + [("OPTEST", path, base, entries)]
    modules = []
    for name, path, base, seeds in specs:
        t = time.time()
        m = Module(name, path, base, seeds, hooks.get(name, ()), name in overlays)
        m.generate(out, per_file, comments, log)
        missing = m.hooks - {a for f in m.prog.funcs.values() for a in f.code}
        if missing:
            raise SystemExit("%s: hook at %s, not in any function's code" % (name, ", ".join("%08X" % a for a in sorted(missing))))
        modules.append(m)
        log("%-9s %5d functions %8d instructions %3d files  %.1f s"
            % (name, len(m.entries), m.n_ins, len(m.files), time.time() - t))
    with Out(os.path.join(out, "modules.cpp")) as f:
        for m in modules:
            f.write('#include "%s_funcs.h"\n' % m.ns)
        f.write("\nextern const SH2Module* const g_sh2_modules[] = {\n")
        for m in modules:
            f.write("    &%s::module,\n" % m.ns)
        f.write("};\nextern const int g_sh2_nmodules = %d;\n" % len(modules))
    srcs = [x for m in modules for x in m.files] + ["modules.cpp"]
    with Out(os.path.join(out, "CMakeLists.txt")) as f:
        f.write(CMAKE.format(rt=RUNTIME.replace("\\", "/"), srcs="\n    ".join(srcs)))
    cls = classify(modules)
    hows = ["switch", "literal", "constant", "pointer", "unresolved", "unresolved jump"]
    heads = ["switch", "literal", "constant", "pointer", "unres.call", "unres.jump"]
    rows = ["%-9s %6s %8s %6s  %s  %s  %s" % ("module", "funcs", "insns", "bsr",
                                               " ".join("%10s" % h for h in heads),
                                               "volatile", "targets: other module / non-entry / outside")]
    for m in modules:
        o, i, x = cls[m.name]
        rows.append("%-9s %6d %8d %6d  %s  %8d  %d / %d / %d" % (
            m.name, len(m.entries), m.n_ins, m.n_calls,
            " ".join("%10d" % m.sites.get(h, 0) for h in hows), m.volatile, o, i, x))
    tot = ("total: %d modules, %d functions, %d instructions, %d files, %.1f s"
           % (len(modules), sum(len(m.entries) for m in modules), sum(m.n_ins for m in modules),
              len(srcs), time.time() - t0))
    with open(os.path.join(out, "report.txt"), "w", encoding="utf-8") as f:
        f.write("calls and jumps through a register (jsr, bsrf, jmp, braf) by how their target was found;\n"
                "unresolved ones go through sh2_call at run time, jumps after a switch over their\n"
                "function's own instructions:\n")
        f.write("\n".join(rows) + "\n" + tot + "\n\n")
        for m in modules:
            seen = set()
            for t, e in sorted(m.unknown):
                if t not in seen:
                    seen.add(t)
                    f.write("%s: target %08X (from %08X) is not an entry of %s\n" % (m.name, t, e, m.name))
    log(tot)
    return modules


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("programs", nargs="*", help="NAME=FILE@BASE[+SEED,...]")
    ap.add_argument("--out", required=True)
    ap.add_argument("--per-file", type=int, default=8000)
    ap.add_argument("--no-comments", action="store_true")
    ap.add_argument("--optest", action="store_true", help="add saturnkit's instruction test module")
    ap.add_argument("--hook", action="append", default=[], help="NAME:ADDR,...: sh2_hook after these instructions")
    ap.add_argument("--overlay", action="append", default=[], help="NAME,...: programs called at their base, that return")
    a = ap.parse_args(argv)
    hooks = {}
    for h in a.hook:
        name, _, addrs = h.partition(":")
        hooks.setdefault(name, []).extend(int(x, 16) for x in addrs.split(",") if x)
    overlays = [n for o in a.overlay for n in o.split(",") if n]
    generate([parse_spec(s) for s in a.programs], a.out, a.per_file, not a.no_comments, a.optest, hooks=hooks,
             overlays=overlays)


if __name__ == "__main__":
    main()
