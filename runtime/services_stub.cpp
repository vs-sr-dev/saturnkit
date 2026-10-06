// saturnkit runtime — no hardware: the services for builds that only run the
// game's own code (the self-test, the link check). A hardware access or an
// exception is reported and ends the run; so does a return that went
// elsewhere, which would mean the recompiled control flow is wrong.
#include "sh2.h"
#include <cstdio>
#include <cstdlib>

static void fail(SH2Context& c, const char* what, uint32_t a) {
    std::fprintf(stderr, "saturnkit: %s %08X (cpu %d, pr %08X, r15 %08X)\n", what, a, c.cpu, c.pr, c.r[15]);
    std::exit(1);
}

// Without hardware nothing can end a wait: a call that spends its whole
// budget of safe points (loop back-edges, calls) is stuck.
void sh2_poll(SH2Context& c) { fail(c, "safe-point budget spent; pr", c.pr); }

void sh2_sleep(SH2Context& c, uint32_t pc) { fail(c, "sleep at", pc); }
void sh2_trapa(SH2Context& c, uint32_t imm, uint32_t pc) { (void)imm; fail(c, "trapa at", pc); }
void sh2_bad_return(SH2Context& c, uint32_t expected) {
    if (sh2_is_landing(c.pc)) throw SH2Unwind{c.pc};
    std::fprintf(stderr, "saturnkit: returned to %08X, expected %08X\n", c.pc, expected);
    std::exit(1);
}
void sh2_call_unknown(SH2Context& c, uint32_t addr) { fail(c, "call to a non-entry", addr); }
// No machine to restart: a program start is an ordinary call into the module
// whose image is in memory there.
void sh2_program_start(SH2Context& c, uint32_t addr) {
    if (const SH2Module* m = sh2_identify(addr)) sh2_activate(m);
    if (SH2Func f = sh2_lookup(addr)) f(c);
    else fail(c, "program start with no module at", addr);
}
void sh2_overlay_call(SH2Context& c, uint32_t addr, const SH2Module* m) {
    if (!m) fail(c, "call to an overlay base with no module at", addr);
    sh2_activate(m);
}

uint32_t sh2_mmio_read(uint32_t a, int size) {
    std::fprintf(stderr, "saturnkit: %d-byte read of %08X (no hardware)\n", size, a);
    std::exit(1);
}
void sh2_mmio_write(uint32_t a, uint32_t v, int size) {
    std::fprintf(stderr, "saturnkit: %d-byte write of %08X to %08X (no hardware)\n", size, v, a);
    std::exit(1);
}
