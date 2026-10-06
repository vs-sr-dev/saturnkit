// saturnkit runtime — the address map outside the work RAMs, and a log of
// every access to it.
//
// sh2_io_* (core.cpp) folds the cache-through mirror and WRAM-H's mirrors,
// and hands the rest here: canonical 27-bit addresses for the external bus,
// full addresses for the cache arrays (0x6..., 0xC...) and the on-chip
// registers (0xFFFFFE00-0xFFFFFFFF).
//
// The log counts accesses by register (address, size, read or write) and, for
// the areas that are memory (VRAM, CRAM, the framebuffer, sound RAM, backup
// RAM, the BIOS ROM, the cartridge), by area with the lowest and highest
// address touched. Calls to addresses that are not entries are logged too.
// mmio_log_write puts it in a text file, one line each:
//     reg  R|W  SIZE  ADDR  COUNT
//     area R|W  NAME  LO  HI  COUNT
//     call TARGET FROM COUNT
#include "saturn.h"
#include <cstdio>
#include <map>
#include <tuple>

uint32_t mem_rd(const uint8_t* p, uint32_t o, int size) {
    uint32_t v = 0;
    for (int i = 0; i < size; ++i) v = v << 8 | p[o + i];
    return v;
}

void mem_wr(uint8_t* p, uint32_t o, uint32_t v, int size) {
    for (int i = size - 1; i >= 0; --i, v >>= 8) p[o + i] = (uint8_t)v;
}

// ---- the log ---------------------------------------------------------------------------
struct AreaLog { uint32_t lo = 0xFFFFFFFFu, hi = 0; uint64_t n = 0; };
static std::map<std::tuple<uint32_t, int, char>, uint64_t> g_regs;
static std::map<std::pair<std::string, char>, AreaLog> g_areas;
static std::map<std::pair<uint32_t, uint32_t>, uint64_t> g_calls;

// A register access costs the CPU a safe point of time: a loop that waits on a
// device (a status register, the CD block's answers) lets time move on.
static void log_reg(uint32_t a, int size, char rw) {
    ++g_regs[{a, size, rw}];
    --g_cpu->budget;
}
// --watch LO:HI: every address in the range, one by one
static std::map<std::pair<uint32_t, char>, uint64_t> g_watch;
static uint32_t g_watch_lo = 1, g_watch_hi = 0;

static void log_area(const char* name, uint32_t a, char rw) {
    if (a >= g_watch_lo && a <= g_watch_hi) {
        ++g_watch[{a, rw}];
        uint64_t v = sat_vblanks();
        bool window = g_cfg.watch_from || g_cfg.watch_to != ~0ull;   // --watch-vblanks: printed, --trace or not
        if (window && v >= g_cfg.watch_from && v <= g_cfg.watch_to)
            sat_note("watch %c %08X (pr %08X, VBlank %llu)", rw, a, g_cpu->pr, (unsigned long long)v);
        else if (!window)
            sat_trace("watch %c %08X (pr %08X)", rw, a, g_cpu->pr);
    }
    AreaLog& l = g_areas[{name, rw}];
    if (a < l.lo) l.lo = a;
    if (a > l.hi) l.hi = a;
    ++l.n;
}

void mmio_watch(uint32_t lo, uint32_t hi) { g_watch_lo = lo; g_watch_hi = hi; }

void mmio_log_call(uint32_t target, uint32_t from) { ++g_calls[{target, from}]; }

void mmio_log_write(const std::string& path) {
    FILE* f = std::fopen(path.c_str(), "w");
    if (!f) return;
    for (auto& [k, n] : g_regs)
        std::fprintf(f, "reg %c %d %08X %llu\n", std::get<2>(k), std::get<1>(k), std::get<0>(k), (unsigned long long)n);
    for (auto& [k, l] : g_areas)
        std::fprintf(f, "area %c %s %08X %08X %llu\n", k.second, k.first.c_str(), l.lo, l.hi, (unsigned long long)l.n);
    for (auto& [k, n] : g_watch)
        std::fprintf(f, "watch %c %08X %llu\n", k.second, k.first, (unsigned long long)n);
    for (auto& [k, n] : g_calls)
        std::fprintf(f, "call %08X %08X %llu\n", k.first, k.second, (unsigned long long)n);
    std::fclose(f);
}

// ---- memory with no device behind it (yet) ------------------------------------------------------
static uint8_t g_backup[0x10000];       // backup RAM, odd bytes; 0x00180000-0x001FFFFF repeats it every 64 KiB (Mednafen)
static uint8_t g_cache[2][0x1000];      // each CPU's cache as RAM (0xC0000000)

static const char* area_of(uint32_t a) {
    if (a < 0x00080000u) return "BIOS ROM";
    if (a >= 0x00180000u && a < 0x00200000u) return "backup RAM";
    if (a >= 0x02000000u && a < 0x05000000u) return "A-bus cartridge";
    if (a >= 0x05A00000u && a < 0x05B00000u) return "SCSP RAM";
    if (a >= 0x05C00000u && a < 0x05C80000u) return "VDP1 VRAM";
    if (a >= 0x05C80000u && a < 0x05D00000u) return "VDP1 framebuffer";
    if (a >= 0x05E00000u && a < 0x05F00000u) return "VDP2 VRAM";
    if (a >= 0x05F00000u && a < 0x05F80000u) return "VDP2 CRAM";
    return nullptr;
}

uint32_t sh2_mmio_read(uint32_t a, int size) {
    if (a >= 0xFFFFFE00u) { log_reg(a, size, 'R'); return onchip_read(*g_cpu, a, size); }
    uint32_t area = a >> 29;
    if (area == 6) { log_area("cache data array", a, 'R'); return mem_rd(g_cache[g_cpu->cpu], a & 0xFFF, size); }
    if (area != 0) { log_area("cache address array", a, 'R'); return 0; }
    if (const char* n = area_of(a)) {
        log_area(n, a, 'R');
        if (a >= 0x00180000u && a < 0x00200000u) return mem_rd(g_backup, a & 0xFFFF, size);
        if (video_owns(a)) return video_read(a, size);
        if (a >= 0x02000000u && a < 0x05000000u) return size == 1 ? 0xFF : size == 2 ? 0xFFFF : 0xFFFFFFFFu;
        return 0;                               // the BIOS ROM is not here
    }
    log_reg(a, size, 'R');
    if (a >= 0x00100000u && a < 0x00100080u) {
        if (size != 1) sat_fatal("%d-byte read of the SMPC at %08X", size, a);
        return smpc_read(a & 0x7F);
    }
    if (a >= 0x05800000u && a < 0x05900000u) return cd_read(a & 0xFFFFF, size);
    if (a >= 0x05FE0000u && a < 0x05FE0100u) return scu_read(a & 0xFF, size);
    if (video_owns(a)) return video_read(a, size);
    sat_fatal("%d-byte read of %08X: nothing there", size, a);
}

void sh2_mmio_write(uint32_t a, uint32_t v, int size) {
    if (a >= 0xFFFFFE00u) { log_reg(a, size, 'W'); onchip_write(*g_cpu, a, v, size); return; }
    uint32_t area = a >> 29;
    if (area == 6) { log_area("cache data array", a, 'W'); mem_wr(g_cache[g_cpu->cpu], a & 0xFFF, v, size); return; }
    if (area != 0) { log_area(area == 2 ? "cache purge" : "cache address array", a, 'W'); return; }
    if (const char* n = area_of(a)) {
        log_area(n, a, 'W');
        if (a >= 0x00180000u && a < 0x00200000u) { mem_wr(g_backup, a & 0xFFFF, v, size); return; }
        if (video_owns(a)) { video_write(a, v, size); return; }
        return;                                 // ROM, no cartridge
    }
    log_reg(a, size, 'W');
    if (a >= 0x00100000u && a < 0x00100080u) {
        if (size != 1) sat_fatal("%d-byte write of the SMPC at %08X", size, a);
        smpc_write(a & 0x7F, v);
        return;
    }
    if (a >= 0x01000000u && a < 0x01800000u) { slave_kick(); return; }             // SINIT
    if (a >= 0x01800000u && a < 0x02000000u) { onchip_input_capture(0); return; }  // MINIT
    if (a >= 0x05800000u && a < 0x05900000u) { cd_write(a & 0xFFFFF, v, size); return; }
    if (a >= 0x05FE0000u && a < 0x05FE0100u) { scu_write(a & 0xFF, v, size); return; }
    if (video_owns(a)) { video_write(a, v, size); return; }
    sat_fatal("%d-byte write of %08X to %08X: nothing there", size, v, a);
}
