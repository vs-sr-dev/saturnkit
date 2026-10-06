// saturnkit runtime — the SH-2 as seen by recompiled code.
//
// Recompiled functions have the signature `void f_XXXXXXXX(SH2Context& c)`,
// one namespace per program (programs swapped at one address each have
// their own set, see SH2Module). They touch guest state only through this
// header: registers in SH2Context, memory through ld*/st* (big-endian guest
// byte order; the two work RAMs as host arrays, everything else through
// sh2_io_*), and a few out-of-line services the runtime provides (calls
// through registers, safe points for interrupts, sleep, trapa).
//
// Semantics are saturnkit/sh2emu.py's, instruction for instruction: the
// self-test (runtime/selftest.cpp) holds the two together.
#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>

#define SH2_UNLIKELY(x) __builtin_expect(!!(x), 0)

struct SH2Context {
    uint32_t r[16];
    uint32_t t, s, q, m, imask;         // SR, a field each
    uint32_t gbr, vbr, mach, macl, pr;
    uint32_t pc;                        // where the last rts/rte went: the caller checks it
    int32_t  budget;                    // safe points left before sh2_poll
    int      cpu;                       // 0 master, 1 slave
};

typedef void (*SH2Func)(SH2Context&);

// A program's recompiled code: the image it was generated from (to recognise
// it in memory) and its entries, sorted by address.
struct SH2FuncEntry { uint32_t addr; SH2Func fn; };
struct SH2Module {
    const char* name;
    uint32_t base, size, crc;           // crc32 of the image as loaded at base
    const SH2FuncEntry* funcs;
    uint32_t nfuncs;
    uint32_t flags;                     // SH2_MODULE_*
    const uint32_t* landings;           // addresses returned to without a call (a longjmp's targets)
    uint32_t nlandings;
};
// An overlay is called at its base as a function and returns to its caller;
// a call to any other module's base is a program start (sh2_program_start).
enum { SH2_MODULE_OVERLAY = 1 };

// ---- services provided by the runtime ---------------------------------------------
extern uint8_t g_wram_l[];              // 0x00200000, 1 MiB
extern uint8_t g_wram_h[];              // 0x06000000, 1 MiB
void     sh2_call(SH2Context& c, uint32_t addr);        // jsr/jmp through a register, calls between programs
void     sh2_poll(SH2Context& c);                       // the budget is spent: time, interrupts
void     sh2_sleep(SH2Context& c, uint32_t pc);
void     sh2_trapa(SH2Context& c, uint32_t imm, uint32_t pc);
void     sh2_hook(SH2Context& c, uint32_t addr);         // after an instruction the build hooked (core.cpp)
void     sh2_hook_set(uint32_t addr, int reg, uint32_t v);   // that hook sets register reg to v
typedef void (*SH2HookFn)(SH2Context& c, uint32_t addr);
void     sh2_hook_add(uint32_t addr, SH2HookFn fn);        // ... or runs fn (a game layer's)
void     sh2_bad_return(SH2Context& c, uint32_t expected);   // rts/rte went elsewhere
// A return to a landing of an active module, not to the call's own address:
// thrown by sh2_bad_return, caught by the calls of the function holding the
// landing, which go on there (the recompiler's emit.Body).
struct SH2Unwind { uint32_t pc; };
bool     sh2_is_landing(uint32_t pc);                   // core.cpp
uint32_t sh2_io_read(uint32_t a, int size);             // anything outside the work RAMs (core.cpp)
void     sh2_io_write(uint32_t a, uint32_t v, int size);
uint32_t sh2_mmio_read(uint32_t a, int size);           // what sh2_io_* does not fold: the hardware
void     sh2_mmio_write(uint32_t a, uint32_t v, int size);

// A safe point: loop back-edges, calls, and `ldc ...,sr` (the mask may drop).
#define SH2_POLL(c) do { if (SH2_UNLIKELY(--(c).budget < 0)) sh2_poll(c); } while (0)
// After a call: the callee's rts must have come back here.
#define SH2_RET(c, a) do { if (SH2_UNLIKELY((c).pc != (a))) sh2_bad_return(c, a); } while (0)

// ---- memory -------------------------------------------------------------------------
// The work RAMs directly, at their cached (0x0...) and cache-through (0x2...)
// addresses; the rest (mirrors, hardware, the on-chip registers) in sh2_io_*.
#define SH2_IS_WRAM_H(a) (((a) & 0xDFF00000u) == 0x06000000u)
#define SH2_IS_WRAM_L(a) (((a) & 0xDFF00000u) == 0x00200000u)

// A store into [g_sh2_watch_lo, g_sh2_watch_lo + g_sh2_watch_len) (addresses
// with the cache-through bit cleared) goes to sh2_watch_store first: the
// runtime's --watch over a work-RAM range. Length 0, the default: none.
extern uint32_t g_sh2_watch_lo, g_sh2_watch_len;
void sh2_watch_store(uint32_t a, uint32_t v, int size);
#define SH2_WATCH(a, v, size) \
    do { if (SH2_UNLIKELY(((a) & 0xDFFFFFFFu) - g_sh2_watch_lo < g_sh2_watch_len)) sh2_watch_store(a, v, size); } while (0)

static inline uint32_t ld8(uint32_t a) {
    if (SH2_IS_WRAM_H(a)) return g_wram_h[a & 0xFFFFFu];
    if (SH2_IS_WRAM_L(a)) return g_wram_l[a & 0xFFFFFu];
    return sh2_io_read(a, 1) & 0xFFu;
}
static inline uint32_t ld16(uint32_t a) {
    uint16_t v;
    if (SH2_IS_WRAM_H(a)) std::memcpy(&v, g_wram_h + (a & 0xFFFFFu), 2);
    else if (SH2_IS_WRAM_L(a)) std::memcpy(&v, g_wram_l + (a & 0xFFFFFu), 2);
    else return sh2_io_read(a, 2) & 0xFFFFu;
    return __builtin_bswap16(v);
}
static inline uint32_t ld32(uint32_t a) {
    uint32_t v;
    if (SH2_IS_WRAM_H(a)) std::memcpy(&v, g_wram_h + (a & 0xFFFFFu), 4);
    else if (SH2_IS_WRAM_L(a)) std::memcpy(&v, g_wram_l + (a & 0xFFFFFu), 4);
    else return sh2_io_read(a, 4);
    return __builtin_bswap32(v);
}
static inline void st8(uint32_t a, uint32_t v) {
    SH2_WATCH(a, v, 1);
    if (SH2_IS_WRAM_H(a)) g_wram_h[a & 0xFFFFFu] = (uint8_t)v;
    else if (SH2_IS_WRAM_L(a)) g_wram_l[a & 0xFFFFFu] = (uint8_t)v;
    else sh2_io_write(a, v & 0xFFu, 1);
}
static inline void st16(uint32_t a, uint32_t v) {
    SH2_WATCH(a, v, 2);
    uint16_t w = __builtin_bswap16((uint16_t)v);
    if (SH2_IS_WRAM_H(a)) std::memcpy(g_wram_h + (a & 0xFFFFFu), &w, 2);
    else if (SH2_IS_WRAM_L(a)) std::memcpy(g_wram_l + (a & 0xFFFFFu), &w, 2);
    else sh2_io_write(a, v & 0xFFFFu, 2);
}
static inline void st32(uint32_t a, uint32_t v) {
    SH2_WATCH(a, v, 4);
    uint32_t w = __builtin_bswap32(v);
    if (SH2_IS_WRAM_H(a)) std::memcpy(g_wram_h + (a & 0xFFFFFu), &w, 4);
    else if (SH2_IS_WRAM_L(a)) std::memcpy(g_wram_l + (a & 0xFFFFFu), &w, 4);
    else sh2_io_write(a, v, 4);
}

// ---- status register --------------------------------------------------------------
static inline uint32_t sh2_get_sr(const SH2Context& c) {
    return c.m << 9 | c.q << 8 | c.imask << 4 | c.s << 1 | c.t;
}
static inline void sh2_set_sr(SH2Context& c, uint32_t v) {
    c.m = v >> 9 & 1; c.q = v >> 8 & 1; c.imask = v >> 4 & 15; c.s = v >> 1 & 1; c.t = v & 1;
}

// ---- arithmetic that needs more than an expression ---------------------------------
// One step of non-restoring division, as the SH-2 manual defines it. rn and
// rm may be the same register: rm is read after rn has been shifted.
static inline void sh2_div1(SH2Context& c, uint32_t& rn, const uint32_t& rm) {
    uint32_t old_q = c.q;
    c.q = rn >> 31;
    rn = rn << 1 | c.t;
    uint32_t tmp0 = rn, tmp1;
    if (old_q == c.m) {                 // same signs: subtract, tmp1 = borrow
        rn -= rm;
        tmp1 = rn > tmp0;
    } else {                            // add, tmp1 = carry
        rn += rm;
        tmp1 = rn < tmp0;
    }
    c.q = c.m == 0 ? (c.q ? !tmp1 : tmp1) : (c.q ? tmp1 : !tmp1);
    c.t = c.q == c.m;
}

// mac.w @Rm+,@Rn+: with S set, MACL saturates at 32 bits and an overflow
// sets MACH bit 0; otherwise MACH:MACL accumulates 64 bits.
static inline void sh2_mac_w(SH2Context& c, int n, int m) {
    int32_t a = (int16_t)ld16(c.r[n]);
    c.r[n] += 2;
    int32_t b = (int16_t)ld16(c.r[m]);
    c.r[m] += 2;
    int64_t p = (int64_t)a * b;
    if (c.s) {
        int64_t acc = (int64_t)(int32_t)c.macl + p;
        if (acc > INT32_MAX) { acc = INT32_MAX; c.mach |= 1; }
        else if (acc < INT32_MIN) { acc = INT32_MIN; c.mach |= 1; }
        c.macl = (uint32_t)acc;
    } else {
        uint64_t acc = ((uint64_t)c.mach << 32 | c.macl) + (uint64_t)p;
        c.mach = (uint32_t)(acc >> 32); c.macl = (uint32_t)acc;
    }
}

// mac.l @Rm+,@Rn+: 64-bit accumulate; with S set, saturated to 48 bits.
static inline void sh2_mac_l(SH2Context& c, int n, int m) {
    int64_t a = (int32_t)ld32(c.r[n]);
    c.r[n] += 4;
    int64_t b = (int32_t)ld32(c.r[m]);
    c.r[m] += 4;
    __int128 acc = (__int128)(int64_t)((uint64_t)c.mach << 32 | c.macl) + (__int128)a * b;
    if (c.s) {
        const __int128 hi = ((__int128)1 << 47) - 1, lo = -((__int128)1 << 47);
        if (acc > hi) acc = hi;
        if (acc < lo) acc = lo;
    }
    uint64_t u = (uint64_t)acc;
    c.mach = (uint32_t)(u >> 32); c.macl = (uint32_t)u;
}

// ---- the runtime's view of the modules (core.cpp) -----------------------------------
// The generated modules.cpp lists every module of the build. A module is
// active in its address range once sh2_activate has been called for it;
// sh2_call looks the target up in the active modules.
extern const SH2Module* const g_sh2_modules[];
extern const int g_sh2_nmodules;
const SH2Module* sh2_module(const char* name);
const SH2Module* sh2_identify(uint32_t base);     // the module whose image is in memory at base
void    sh2_activate(const SH2Module* m);          // replaces any active module it overlaps
SH2Func sh2_lookup(uint32_t addr);                  // nullptr if not an entry of an active module
void    sh2_call_unknown(SH2Context& c, uint32_t addr);   // services: not an entry (the BIOS...)
void    sh2_program_start(SH2Context& c, uint32_t addr);  // services: a call to a module's base
void    sh2_overlay_call(SH2Context& c, uint32_t addr, const SH2Module* m);   // services: a call to an
                                                    // overlay's base, m the image found there (or nullptr)
uint32_t sh2_crc32(const uint8_t* p, size_t n, uint32_t crc = 0);
// Guest memory as bytes, for loaders and tests (work RAMs only).
bool sh2_mem_write(uint32_t addr, const void* src, size_t n);
bool sh2_mem_read(uint32_t addr, void* dst, size_t n);
