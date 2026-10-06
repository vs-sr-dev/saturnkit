// saturnkit runtime — the machine: the run loop, time, interrupts taken by a
// CPU, the slave SH-2, and program starts.
//
// Time. The recompiled code has no cycle counts; it has safe points (loop
// back-edges, calls, `ldc ...,sr`). Every kBudget safe points the master
// polls: time moves on, the devices run, interrupts are taken. Time is
// virtual by default (kNsPerSafePoint per safe point: a run is the same run
// every time) or the host's clock (`realtime`).
//
// Interrupts are taken at a poll, as the CPU would between two instructions:
// SR and a return PC pushed on the guest stack, the mask raised to the
// level, the handler reached through the vector table at VBR. A vector that
// still points at the BIOS's dispatcher goes to the handler SYS_SETUINT
// installed, called like a function (the BIOS saves the registers and does
// the rte); any other is the game's own handler and ends with its rte.
//
// The slave SH-2 is a second context on a host thread used as a coroutine:
// exactly one CPU runs at a time, and control changes hands at fixed points,
// so a run stays deterministic. SSHON boots it (from the address the BIOS
// reads at 0x06000250, vector 0x94, which SYS_SETSINT sets); it runs until it
// waits: it reads its FRT's input-capture flag and finds nothing there (the
// SBL slave loop), or it has used a slice of safe points. A write to SINIT
// sets that flag and hands it the CPU at once; while it has work, the master
// gives it a slice at each of its own polls.
//
// Program starts. A call to a module's base address is a crt0: it resets the
// stack and never returns. The runtime identifies the image in memory,
// activates its module, and throws back to its own loop, which calls the
// entry on a fresh host stack: the old program's C++ frames do not pile up
// under the new one.
#include "saturn.h"
#include <chrono>
#include <condition_variable>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <map>
#include <mutex>
#include <thread>
#include <vector>

SaturnConfig g_cfg;
SH2Context g_master, g_slave;
SH2Context* g_cpu = &g_master;

static const int32_t kBudget = 256;             // safe points between polls
static const uint64_t kNsPerSafePoint = 400;    // virtual time: ~11 instructions at 28.6 MHz
static const uint32_t kIntPC = 0xFFFFFFE4u;     // the PC an interrupt pushes: where rte must go
static const int kSlaveSlice = 64;              // polls the slave runs before it gives the CPU back

struct ProgramStart { uint32_t addr; };
struct RunStop {};
struct SlaveReset {};

static uint64_t g_vtime;                        // virtual ns
static std::chrono::steady_clock::time_point g_t0;
static int g_starts;
static int g_overlays;

uint64_t sat_now() {
    if (g_cfg.realtime)
        return (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now() - g_t0).count();
    return g_vtime;
}

static void vprint(const char* tag, const char* fmt, va_list ap) {
    std::fprintf(stderr, "[%8.3f %s] ", sat_now() / 1e9, tag);
    std::vfprintf(stderr, fmt, ap);
    std::fputc('\n', stderr);
}

void sat_trace(const char* fmt, ...) {
    if (!g_cfg.trace) return;
    va_list ap;
    va_start(ap, fmt);
    vprint(g_cpu->cpu ? "S" : "M", fmt, ap);
    va_end(ap);
}

void sat_note(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vprint(g_cpu->cpu ? "S" : "M", fmt, ap);
    va_end(ap);
}

// Where a CPU is, roughly: the words on its stack that point into WRAM-H
// code but are not entries (return addresses, mostly).
static void backtrace(const SH2Context& c) {
    std::fprintf(stderr, "  stack (cpu %d, r15 %08X):", c.cpu, c.r[15]);
    int n = 0;
    for (uint32_t a = c.r[15] & ~3u; a < (c.r[15] & ~3u) + 0x400 && n < 24; a += 4) {
        if (!SH2_IS_WRAM_H(a)) break;
        uint32_t v = ld32(a);
        if (v >= 0x06004000u && v < 0x06100000u && !(v & 1) && !sh2_lookup(v)) {
            std::fprintf(stderr, " %08X", v);
            ++n;
        }
    }
    std::fprintf(stderr, "\n");
}

// Where the master has been polling: the host return address of each of the
// last 4096 polls lies in the recompiled function that polled; the modules'
// tables map it back to that function's guest entry.
static void* g_poll_ring[4096];
static unsigned g_poll_pos;

// The recompiled function a host code address lies in: its module and guest entry
struct HostFn { uintptr_t host; uint32_t addr; const char* mod; };
static const HostFn* host_function(const void* p) {
    static std::vector<HostFn> fns;
    if (fns.empty()) {
        for (int i = 0; i < g_sh2_nmodules; ++i)
            for (uint32_t k = 0; k < g_sh2_modules[i]->nfuncs; ++k)
                fns.push_back({(uintptr_t)g_sh2_modules[i]->funcs[k].fn, g_sh2_modules[i]->funcs[k].addr,
                               g_sh2_modules[i]->name});
        std::sort(fns.begin(), fns.end(), [](const HostFn& a, const HostFn& b) { return a.host < b.host; });
    }
    auto it = std::upper_bound(fns.begin(), fns.end(), (uintptr_t)p, [](uintptr_t v, const HostFn& f) { return v < f.host; });
    return it == fns.begin() ? nullptr : &*(it - 1);
}

// --watch over the work RAMs: every store, its value, the function that made it
static void watch_report(uint32_t a, uint32_t v, int size, const void* host) {
    if (sat_vblanks() < g_cfg.watch_from || sat_vblanks() > g_cfg.watch_to) return;
    const HostFn* f = host_function(host);
    sat_note("store%d %08X = %0*X in %s:%08X (pr %08X, VBlank %llu)", size * 8, a, size * 2, v,
             f ? f->mod : "?", f ? f->addr : 0, g_cpu->pr, (unsigned long long)sat_vblanks());
}

static void hot_spots() {
    std::map<std::pair<const char*, uint32_t>, int> hist;
    for (void* p : g_poll_ring) {
        if (!p) continue;
        if (const HostFn* f = host_function(p)) ++hist[{f->mod, f->addr}];
    }
    std::vector<std::pair<int, std::pair<const char*, uint32_t>>> top;
    for (auto& [k, n] : hist) top.push_back({n, k});
    std::sort(top.rbegin(), top.rend());
    std::fprintf(stderr, "  last polls in:");
    for (size_t i = 0; i < top.size() && i < 8; ++i)
        std::fprintf(stderr, " %s:%08X x%d", top[i].second.first, top[i].second.second, top[i].first);
    std::fprintf(stderr, "\n");
}

// --peek ADDR[:WORDS],...: 32-bit words of memory, through the ordinary reads
static void peek(const std::string& spec) {
    size_t i = 0;
    while (i < spec.size()) {
        size_t j = spec.find(',', i);
        std::string item = spec.substr(i, j == std::string::npos ? std::string::npos : j - i);
        i = j == std::string::npos ? spec.size() : j + 1;
        uint32_t a = (uint32_t)std::stoul(item, nullptr, 16), n = 1;
        size_t colon = item.find(':');
        if (colon != std::string::npos) n = (uint32_t)std::stoul(item.substr(colon + 1));
        std::fprintf(stderr, "  %08X:", a);
        for (uint32_t k = 0; k < n; ++k) std::fprintf(stderr, " %08X", ld32(a + 4 * k));
        std::fprintf(stderr, "\n");
    }
}

static uint64_t g_ints[0x80];

static void report_ints() {
    std::fprintf(stderr, "  interrupts taken:");
    for (int v = 0; v < 0x80; ++v)
        if (g_ints[v]) std::fprintf(stderr, " %02X:%llu", v, (unsigned long long)g_ints[v]);
    std::fprintf(stderr, "\n");
}

void sat_fatal(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vprint("FATAL", fmt, ap);
    va_end(ap);
    SH2Context& c = *g_cpu;
    std::fprintf(stderr, "  cpu %d  pr %08X  pc %08X  sr %03X\n", c.cpu, c.pr, c.pc, sh2_get_sr(c));
    for (int i = 0; i < 16; ++i) std::fprintf(stderr, "  r%-2d %08X%s", i, c.r[i], i % 4 == 3 ? "\n" : "");
    backtrace(c);
    report_ints();
    hot_spots();
    mmio_log_write(g_cfg.out + "/hw-log.txt");
    std::fflush(stderr);
    std::_Exit(1);
}

static const char* g_stop_why;
void sat_stop(const char* why) {
    g_stop_why = why;
    if (g_cpu->cpu) sat_fatal("stop requested on the slave: %s", why);
    throw RunStop{};
}

// ---- the slave: a coroutine on its own thread -----------------------------------------------
static std::mutex g_mu;
static std::condition_variable g_cv;
static int g_turn;                  // 0 the master runs, 1 the slave
static bool g_slave_on, g_slave_idle = true, g_slave_reset, g_slave_thread;
static int g_slave_polls;

static void give(int to) {          // hand the CPU over and wait until it comes back
    std::unique_lock<std::mutex> lk(g_mu);
    g_turn = to;
    g_cv.notify_all();
    g_cv.wait(lk, [&] { return g_turn == 1 - to; });
}

static void to_slave() {
    give(1);
    g_cpu = &g_master;
}

static void to_master() {
    give(0);
    g_cpu = &g_slave;
    g_slave_polls = 0;
    if (g_slave_reset) throw SlaveReset{};
}

static void slave_main() {
    {
        std::unique_lock<std::mutex> lk(g_mu);
        g_cv.wait(lk, [] { return g_turn == 1; });
    }
    g_cpu = &g_slave;
    for (;;) {
        try {
            g_slave_reset = false;
            while (!g_slave_on) { g_slave_idle = true; to_master(); }
            // the BIOS's slave boot: its own vector table, interrupts masked,
            // its stack, then the entry the program set as vector 0x94
            SH2Context& s = g_slave;
            s = SH2Context{};
            s.cpu = 1;
            s.vbr = 0x06000400u;
            s.imask = 15;
            s.r[15] = 0x06001000u;
            s.budget = kBudget;
            uint32_t entry = ld32(0x06000250u);
            sat_trace("slave boots at %08X", entry);
            g_slave_idle = false;
            sh2_call(s, entry);
            sat_fatal("the slave's entry %08X returned", entry);
        } catch (SlaveReset&) {
            sat_trace("slave reset");
        }
    }
}

void slave_on() {
    if (!g_slave_thread) {
        g_slave_thread = true;
        std::thread(slave_main).detach();
    }
    if (g_slave_on) return;
    g_slave_on = true;
    onchip_reset(1);
    sat_trace("SSHON");
    to_slave();                     // it boots and runs to its first wait
}

void slave_off() {
    sat_trace("SSHOFF");
    if (!g_slave_on) return;
    g_slave_on = false;
    g_slave_reset = true;
    to_slave();                     // it unwinds and parks
}

void slave_idle_check(uint32_t ftcsr) {
    if (g_cpu->cpu != 1 || (ftcsr & 0x80)) return;
    g_slave_idle = true;
    to_master();
}

// SINIT: the master pulses the slave's FRT input capture
void slave_kick() {
    onchip_input_capture(1);
    if (!g_slave_on) return;
    g_slave_idle = false;
    to_slave();
}

// ---- time, devices, interrupts ----------------------------------------------------------
void master_poll_devices() {
    uint64_t now = sat_now();
    video_tick(now);
    cd_tick();
    smpc_tick();
    sound_tick();
    if (g_slave_on && !g_slave_idle) to_slave();
    if (g_cfg.stop_vblanks && sat_vblanks() >= g_cfg.stop_vblanks) sat_stop("VBlank limit");
}

// Virtual time passes to `until` on the master, and the interrupts are taken. While the
// SCU lets HBlank-IN through, one raster line at a time: each line's HBlank is taken
// before the next line starts, as a handler that changes VDP2 a line at a time needs
// (otherwise a poll spans more than a line, and HBlanks raised twice are taken once).
static bool advance(SH2Context& c, uint64_t until) {
    if (scu_mask() >> IRQ_HBLANK_IN & 1) {
        g_vtime = until;
        master_poll_devices();
        return scu_deliver(c);
    }
    bool any = false;
    while (g_vtime < until) {
        g_vtime = std::min(until, video_next_line(g_vtime));
        master_poll_devices();
        any |= scu_deliver(c);
    }
    return any;
}

void sh2_poll(SH2Context& c) {
    c.budget = kBudget;
    if (c.cpu == 1) {
        if (++g_slave_polls >= kSlaveSlice) to_master();
        return;
    }
    g_poll_ring[g_poll_pos++ % 4096] = __builtin_return_address(0);
    advance(c, g_vtime + kBudget * kNsPerSafePoint);
}

static int g_irq_active = -1;

int sat_interrupt_active() { return g_irq_active; }

void sat_interrupt(SH2Context& c, uint32_t vec, uint32_t level) {
    ++g_ints[vec & 0x7F];
    int outer = g_irq_active;
    g_irq_active = c.cpu == 0 ? (int)vec : outer;
    SH2Context saved = c;
    uint32_t target = ld32(c.vbr + vec * 4);
    c.r[15] -= 4; st32(c.r[15], sh2_get_sr(c));
    c.r[15] -= 4; st32(c.r[15], kIntPC);
    c.imask = level;
    if (bios_is_dispatcher(vec, target)) {
        bios_dispatch(c, vec);
    } else {
        c.pc = 0;
        sh2_call(c, target);
        if (c.pc != kIntPC) sat_fatal("interrupt %02X: handler %08X did not rte (went to %08X)", vec, target, c.pc);
    }
    int32_t budget = c.budget;
    c = saved;
    c.budget = budget;
    g_irq_active = outer;
}

void sh2_sleep(SH2Context& c, uint32_t pc) {
    // wait for an interrupt: time passes until one is taken
    for (int i = 0; i < 100000; ++i)
        if (advance(c, g_vtime + kBudget * kNsPerSafePoint)) return;
    sat_fatal("sleep at %08X: nothing woke it", pc);
}

void sh2_trapa(SH2Context& c, uint32_t imm, uint32_t pc) {
    (void)c;
    sat_fatal("trapa #%u at %08X", imm, pc);
}

void sh2_bad_return(SH2Context& c, uint32_t expected) {
    sat_fatal("returned to %08X, expected %08X", c.pc, expected);
}

void sh2_call_unknown(SH2Context& c, uint32_t addr) {
    if (bios_call(c, addr)) return;
    mmio_log_call(addr, c.pr);
    sat_fatal("call to %08X, not an entry of an active module (from pr %08X)", addr, c.pr);
}

void sh2_program_start(SH2Context& c, uint32_t addr) {
    if (c.cpu != 0) sat_fatal("the slave started a program at %08X", addr);
    const SH2Module* m = sh2_identify(addr);
    if (m) sh2_activate(m);
    else if (!sh2_lookup(addr)) sat_fatal("program start at %08X: no module matches the image there", addr);
    ++g_starts;
    sat_note("program start %d: %s at %08X (r4 %08X, r5 %08X)", g_starts, m ? m->name : "(same module)",
             addr, c.r[4], c.r[5]);
    throw ProgramStart{addr};
}

void sh2_overlay_call(SH2Context& c, uint32_t addr, const SH2Module* m) {
    if (!m) {
        uint8_t img[0x100];
        sh2_mem_read(addr, img, sizeof img);
        sat_fatal("call to the overlay base %08X: no module matches the image there (crc32 of its first 256 bytes "
                  "%08X, from pr %08X)", addr, sh2_crc32(img, sizeof img), c.pr);
    }
    sh2_activate(m);
    ++g_overlays;
    sat_note("overlay %d: %s at %08X (from pr %08X)", g_overlays, m->name, addr, c.pr);
}

int saturn_main(const SaturnConfig& cfg) {
    g_cfg = cfg;
    g_t0 = std::chrono::steady_clock::now();
    mmio_watch(cfg.watch_lo, cfg.watch_hi);
    if (cfg.watch_lo <= cfg.watch_hi && (SH2_IS_WRAM_H(cfg.watch_lo) || SH2_IS_WRAM_L(cfg.watch_lo))) {
        extern void (*g_sh2_watch_report)(uint32_t, uint32_t, int, const void*);
        g_sh2_watch_lo = cfg.watch_lo & 0xDFFFFFFFu;
        g_sh2_watch_len = cfg.watch_hi - cfg.watch_lo + 1;
        g_sh2_watch_report = watch_report;
    }
    smpc_input_script(cfg.input);
    if (!cdrom_open(cfg.cue)) { std::fprintf(stderr, "cannot open the disc %s\n", cfg.cue.c_str()); return 2; }
    g_master = SH2Context{};
    g_master.budget = kBudget;
    video_init();
    sound_init();
    cd_init();
    onchip_reset(0);
    bios_boot();
    uint32_t entry = bios_first_read();
    const SH2Module* m = sh2_identify(entry);
    if (!m) { std::fprintf(stderr, "no module matches the 1st read file at %08X\n", entry); return 2; }
    sh2_activate(m);
    sat_note("boot: %s at %08X", m->name, entry);
    // IP.BIN's initial program, when the build has IP.BIN as a module: the
    // BIOS runs IP.BIN's code from its area block on (each area's `bra`
    // leads to the next, the last to the initial program), then the 1st
    // read address if that returns. A game whose initial program is its
    // crt0 never returns from it.
    const uint32_t kAreaBlock = 0x06002E00u, first = entry;
    if (const SH2Module* ip = sh2_identify(0x06002000u)) {
        sh2_activate(ip);
        if (sh2_lookup(kAreaBlock)) {
            entry = kAreaBlock;
            sat_note("boot: IP.BIN's initial program (%s) from %08X", ip->name, entry);
        }
    }
    for (;;) {
        try {
            SH2Func f = sh2_lookup(entry);
            if (!f) sat_fatal("no function at %08X", entry);
            f(g_master);
            if (entry == kAreaBlock && first != kAreaBlock) { entry = first; continue; }
            sat_fatal("the program at %08X returned", entry);
        } catch (ProgramStart& s) {
            entry = s.addr;
            if (cfg.stop_starts && g_starts >= cfg.stop_starts) { g_stop_why = "program-start limit"; break; }
        } catch (RunStop&) {
            break;
        }
    }
    sat_note("stopped (%s) after %llu VBlanks, %.3f s, %d program starts, %d overlay calls, %llu VDP1 frame changes, %llu draws",
             g_stop_why ? g_stop_why : "?", (unsigned long long)sat_vblanks(), sat_now() / 1e9, g_starts, g_overlays,
             (unsigned long long)video_frame_changes(), (unsigned long long)video_draws());
    report_ints();
    std::fprintf(stderr, "  master pr %08X, sr %03X\n", g_master.pr, sh2_get_sr(g_master));
    backtrace(g_master);
    hot_spots();
    peek(cfg.peek);
    sound_close();
    mmio_log_write(cfg.out + "/hw-log.txt");
    bios_save();
    std::fflush(stderr);
    std::fflush(stdout);
    std::_Exit(0);                  // the slave's thread is parked: do not wait for it
}
