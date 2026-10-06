// saturnkit runtime — the part every build shares: the work RAMs, the
// address map outside them, and the address -> function dispatch over the
// active modules (one per address range: the resident programs and the one
// swapped in).
#include "sh2.h"
#include <cstdio>
#include <cstring>

alignas(64) uint8_t g_wram_l[0x100000 + 8];     // +8: a word read at the last byte stays inside
alignas(64) uint8_t g_wram_h[0x100000 + 8];

// ---- the address map outside the fast paths ------------------------------------------
// Areas by the top three bits: 0 cached, 1 cache-through (the same 27-bit
// external bus), 2 associative purge (writes drop a cache line: nothing to
// do), 3 the cache's address array, 6 its data array, 7 the on-chip
// registers. WRAM-H repeats every MiB up to 0x07FFFFFF.

static uint32_t wram_h_read(uint32_t o, int size) {
    uint32_t v = 0;
    for (int i = 0; i < size; ++i) v = v << 8 | g_wram_h[(o + i) & 0xFFFFFu];
    return v;
}

uint32_t sh2_io_read(uint32_t a, int size) {
    uint32_t area = a >> 29;
    if (area <= 1) {
        uint32_t p = a & 0x07FFFFFFu;
        if (p >= 0x06000000u) return wram_h_read(p, size);
        return sh2_mmio_read(p, size);
    }
    if (area == 2) return 0;
    return sh2_mmio_read(a, size);
}

void sh2_io_write(uint32_t a, uint32_t v, int size) {
    uint32_t area = a >> 29;
    if (area <= 1) {
        uint32_t p = a & 0x07FFFFFFu;
        if (p >= 0x06000000u) {
            for (int i = size - 1; i >= 0; --i, v >>= 8) g_wram_h[(p + i) & 0xFFFFFu] = (uint8_t)v;
            return;
        }
        sh2_mmio_write(p, v, size);
        return;
    }
    if (area == 2) return;
    sh2_mmio_write(a, v, size);
}

static uint8_t* host(uint32_t a) {
    a &= 0x1FFFFFFFu;
    if (a >= 0x06000000u && a < 0x08000000u) return g_wram_h + (a & 0xFFFFFu);
    if (a >= 0x00200000u && a < 0x00300000u) return g_wram_l + (a & 0xFFFFFu);
    return nullptr;
}

bool sh2_mem_write(uint32_t addr, const void* src, size_t n) {
    const uint8_t* s = (const uint8_t*)src;
    for (size_t i = 0; i < n; ++i) {
        uint8_t* p = host(addr + (uint32_t)i);
        if (!p) return false;
        *p = s[i];
    }
    return true;
}

bool sh2_mem_read(uint32_t addr, void* dst, size_t n) {
    uint8_t* d = (uint8_t*)dst;
    for (size_t i = 0; i < n; ++i) {
        uint8_t* p = host(addr + (uint32_t)i);
        if (!p) return false;
        d[i] = *p;
    }
    return true;
}

// ---- modules ---------------------------------------------------------------------
uint32_t sh2_crc32(const uint8_t* p, size_t n, uint32_t crc) {
    static uint32_t table[256];
    if (!table[1])
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int k = 0; k < 8; ++k) c = c & 1 ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            table[i] = c;
        }
    crc = ~crc;
    for (size_t i = 0; i < n; ++i) crc = table[(crc ^ p[i]) & 0xFF] ^ (crc >> 8);
    return ~crc;
}

const SH2Module* sh2_module(const char* name) {
    for (int i = 0; i < g_sh2_nmodules; ++i)
        if (!std::strcmp(g_sh2_modules[i]->name, name)) return g_sh2_modules[i];
    return nullptr;
}

const SH2Module* sh2_identify(uint32_t base) {
    for (int i = 0; i < g_sh2_nmodules; ++i) {
        const SH2Module* m = g_sh2_modules[i];
        uint8_t* p = host(base);
        if (m->base != base || !p || !host(base + m->size - 1)) continue;
        if (sh2_crc32(p, m->size) == m->crc) return m;
    }
    return nullptr;
}

static const int kSlots = 8;
static const SH2Module* g_active[kSlots];

void sh2_activate(const SH2Module* m) {
    for (auto& a : g_active)
        if (a && a->base < m->base + m->size && m->base < a->base + a->size) a = nullptr;
    for (auto& a : g_active)
        if (!a) { a = m; return; }
    std::fprintf(stderr, "saturnkit: more than %d modules active\n", kSlots);
}

SH2Func sh2_lookup(uint32_t addr) {
    if (addr >> 29 == 1) addr &= 0x1FFFFFFFu;           // code run cache-through
    for (const SH2Module* m : g_active) {
        if (!m || addr - m->base >= m->size) continue;
        uint32_t lo = 0, hi = m->nfuncs;
        while (lo < hi) {
            uint32_t mid = (lo + hi) / 2;
            if (m->funcs[mid].addr < addr) lo = mid + 1; else hi = mid;
        }
        if (lo < m->nfuncs && m->funcs[lo].addr == addr) return m->funcs[lo].fn;
    }
    return nullptr;
}

// A call to a module's base address is a program start: on the Saturn the
// 1st read address, where a crt0 resets the stack. The services decide what
// that means (the game's runtime unwinds the host stack, the tests just call).
// A call to an overlay's base is an ordinary call into the overlay found
// there: the services activate it, then it runs and returns to its caller.
// is_base: 0 not a base, 1 a program's, 2 an overlay's (some module there is one).
static int is_base(uint32_t addr) {
    static uint32_t bases[kSlots * 2];
    static int kinds[kSlots * 2];
    static int n = -1;
    if (n < 0) {
        n = 0;
        for (int i = 0; i < g_sh2_nmodules; ++i) {
            const SH2Module* m = g_sh2_modules[i];
            int kind = m->flags & SH2_MODULE_OVERLAY ? 2 : 1, k = 0;
            while (k < n && bases[k] != m->base) ++k;
            if (k < n) { if (kind == 2) kinds[k] = 2; }
            else if (n < kSlots * 2) { bases[n] = m->base; kinds[n++] = kind; }
        }
    }
    for (int k = 0; k < n; ++k)
        if (bases[k] == addr) return kinds[k];
    return 0;
}

void sh2_call(SH2Context& c, uint32_t addr) {
    if (addr >> 29 == 1) addr &= 0x1FFFFFFFu;
    if (int kind = is_base(addr); SH2_UNLIKELY(kind)) {
        const SH2Module* m = kind == 2 ? sh2_identify(addr) : nullptr;
        if (kind == 1 || (m && !(m->flags & SH2_MODULE_OVERLAY))) { sh2_program_start(c, addr); return; }
        sh2_overlay_call(c, addr, m);
    }
    if (SH2Func f = sh2_lookup(addr)) f(c);
    else sh2_call_unknown(c, addr);
}

// ---- the store watch ------------------------------------------------------------------------
// Called from the recompiled function that stores (the inline st8/16/32):
// its return address lies in that function, which the report maps back to
// the guest's.
uint32_t g_sh2_watch_lo, g_sh2_watch_len;
void (*g_sh2_watch_report)(uint32_t a, uint32_t v, int size, const void* host);

__attribute__((noinline)) void sh2_watch_store(uint32_t a, uint32_t v, int size) {
    if (g_sh2_watch_report) g_sh2_watch_report(a, v, size, __builtin_return_address(0));
}

// ---- hooks ----------------------------------------------------------------------------------
// The recompiler calls sh2_hook after the instructions a build asked for
// (recomp --hook); what a hook does is set here: from the command line
// (main.cpp --hook), a register takes a value; from a game layer compiled
// into the executable (sh2_hook_add), a function runs. A hook nothing was
// set for does nothing.
struct Hook { uint32_t addr; int reg; uint32_t v; SH2HookFn fn; };
static Hook g_hooks[64];
static int g_nhooks;

void sh2_hook_set(uint32_t addr, int reg, uint32_t v) {
    if (g_nhooks < 64) g_hooks[g_nhooks++] = {addr, reg & 15, v, nullptr};
}

void sh2_hook_add(uint32_t addr, SH2HookFn fn) {
    if (g_nhooks < 64) g_hooks[g_nhooks++] = {addr, 0, 0, fn};
}

void sh2_hook(SH2Context& c, uint32_t addr) {
    for (int i = 0; i < g_nhooks; ++i)
        if (g_hooks[i].addr == addr) {
            if (g_hooks[i].fn) g_hooks[i].fn(c, addr);
            else c.r[g_hooks[i].reg] = g_hooks[i].v;
        }
}
