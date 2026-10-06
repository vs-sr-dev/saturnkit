// saturnkit runtime — the SCU: its interrupt controller, the two timers and
// the three DMA levels (the DSP is in scudsp.cpp).
//
// Interrupts: a source sets its bit in IST; an interrupt whose bit is not
// masked in IMS goes to the master at its fixed level, and is taken at the
// master's next poll if the level is above the CPU's mask. Taking it clears
// the bit.
//
// DMA runs at once when started (DxEN with GO for the "start at once"
// factor, or when its start factor comes: VBlank-IN/OUT, sprite draw end),
// through the ordinary memory functions, so a transfer to a device's
// registers or from the CD block's data port does what the CPU would.
#include "saturn.h"
#include <cstdio>
#include <cstring>

static uint32_t g_ist, g_ims = 0xBFFFu;
static uint32_t g_t0c, g_t1s, g_t1md;
static uint32_t g_regs[0x40];              // everything else, as written

struct Dma { uint32_t r, w, c, ad, en, md; };
static Dma g_dma[3];

// levels by IST bit (ST-097), vectors 0x40 + bit
static const uint8_t kLevel[16] = {0xF, 0xE, 0xD, 0xC, 0xB, 0xA, 0x9, 0x8, 0x8, 0x6, 0x6, 0x5, 0x3, 0x2, 0, 0};

void scu_raise(int irq) { g_ist |= 1u << irq; }
void scu_set_mask(uint32_t ims) { g_ims = ims; }
uint32_t scu_mask() { return g_ims; }

bool scu_deliver(SH2Context& c) {
    bool any = false;
    for (;;) {
        uint32_t pend = g_ist & ~g_ims & 0x3FFFu;
        if (!pend) return any;
        int best = -1;
        for (int b = 0; b < 14; ++b)
            if ((pend >> b & 1) && (best < 0 || kLevel[b] > kLevel[best])) best = b;
        if (kLevel[best] <= c.imask) return any;
        g_ist &= ~(1u << best);
        sat_interrupt(c, 0x40 + best, kLevel[best]);
        any = true;
    }
}

// ---- DMA -------------------------------------------------------------------------------
static bool bbus(uint32_t a) {
    a &= 0x07FFFFFFu;
    return a >= 0x05A00000u && a < 0x05FE0000u;
}

static void copy(uint32_t& src, uint32_t& dst, uint32_t bytes, uint32_t radd, uint32_t wadd) {
    // 32-bit units; the B bus takes them as two 16-bit writes, each moving by the write add
    for (uint32_t i = 0; i < bytes; i += 4) {
        uint32_t n = bytes - i < 4 ? bytes - i : 4;
        if (n == 4) {
            uint32_t v = ld32(src);
            if (bbus(dst)) { st16(dst, v >> 16); dst += wadd; st16(dst, v & 0xFFFF); dst += wadd; }
            else { st32(dst, v); dst += wadd; }
        } else {
            for (uint32_t k = 0; k < n; ++k) st8(dst + k, ld8(src + k));
            dst += wadd;
        }
        src += radd;
    }
}

static void dma_run(int lvl) {
    Dma& d = g_dma[lvl];
    uint32_t radd = (d.ad >> 8 & 1) ? 4 : 0;
    uint32_t wadd = (d.ad & 7) ? 1u << (d.ad & 7) : 0;
    if (d.md >> 24 & 1) {                       // indirect: {count, write, read} triples at the write address
        uint32_t t = d.w, n = 0;
        char first[160] = "";                   // the first transfers, for the trace
        for (;;) {
            uint32_t cnt = ld32(t), dst = ld32(t + 4), src = ld32(t + 8);
            bool end = src >> 31;
            src &= 0x07FFFFFFu;
            if (n < 3) {
                size_t l = std::strlen(first);
                std::snprintf(first + l, sizeof first - l, "%s%X bytes %08X -> %08X", n ? ", " : ": ", cnt, src, dst);
            }
            copy(src, dst, cnt ? cnt : (lvl ? 0x1000 : 0x100000), radd, wadd);
            t += 12; ++n;
            if (end || n > 4096) break;
        }
        sat_trace("SCU DMA %d indirect: %u transfers from the table at %08X%s%s", lvl, n, d.w, first, n > 3 ? ", ..." : "");
        if (d.md >> 8 & 1) d.w = t;
    } else {
        uint32_t cnt = d.c & (lvl ? 0xFFF : 0xFFFFF);
        if (!cnt) cnt = lvl ? 0x1000 : 0x100000;
        uint32_t src = d.r, dst = d.w;
        copy(src, dst, cnt, radd, wadd);
        sat_trace("SCU DMA %d: %X bytes %08X -> %08X", lvl, cnt, d.r, d.w);
        if (d.md >> 16 & 1) d.r = src;
        if (d.md >> 8 & 1) d.w = dst;
    }
    scu_raise(lvl == 0 ? IRQ_DMA0 : lvl == 1 ? IRQ_DMA1 : IRQ_DMA2);
}

void scu_frame_event(int what) {
    for (int l = 0; l < 3; ++l)
        if ((g_dma[l].en >> 8 & 1) && (int)(g_dma[l].md & 7) == what) dma_run(l);
}

// Timer 0 fires on the line T0C names; timer 1, T1S clocks into a line, on
// every line (T1MD bit 8 clear) or only on timer 0's line (bit 8 set). An
// interrupt raised again before it is taken is one interrupt, so at the
// runtime's poll rate timer 1 comes about once a poll.
void scu_line(int line) {
    if (!(g_t1md & 1)) return;
    bool t0 = line == (int)(g_t0c & 0x3FF);
    if (t0) scu_raise(IRQ_TIMER0);
    if (!(g_t1md >> 8 & 1) || t0) scu_raise(IRQ_TIMER1);
}

// ---- registers ---------------------------------------------------------------------------
uint32_t scu_read(uint32_t off, int size) {
    if (size != 4) {                            // a part of a 32-bit register
        uint32_t v = scu_read(off & ~3u, 4);
        return size == 2 ? (off & 2 ? v & 0xFFFF : v >> 16) : v >> (8 * (3 - (off & 3))) & 0xFF;
    }
    if (off < 0x60) {
        Dma& d = g_dma[off / 0x20];
        switch (off % 0x20) {
        case 0x00: return d.r;
        case 0x04: return d.w;
        case 0x08: return d.c;
        case 0x0C: return d.ad;
        case 0x10: return d.en;
        case 0x14: return d.md;
        }
    }
    switch (off) {
    case 0x7C: return 0;                        // DSTA: nothing in progress
    case 0x80: case 0x84: case 0x88: case 0x8C: return scu_dsp_read(off);
    case 0x90: return g_t0c;
    case 0x94: return g_t1s;
    case 0x98: return g_t1md;
    case 0xA0: return g_ims;
    case 0xA4: return g_ist;
    case 0xC8: return 4;                        // VER
    }
    return g_regs[off / 4];
}

void scu_write(uint32_t off, uint32_t v, int size) {
    if (size != 4) {
        uint32_t old = scu_read(off & ~3u, 4);
        int sh = size == 2 ? (off & 2 ? 0 : 16) : 8 * (3 - (off & 3));
        uint32_t m = (size == 2 ? 0xFFFFu : 0xFFu) << sh;
        v = (old & ~m) | (v << sh & m);
        off &= ~3u;
    }
    if (off < 0x60) {
        int l = off / 0x20;
        Dma& d = g_dma[l];
        switch (off % 0x20) {
        case 0x00: d.r = v & 0x07FFFFFFu; return;
        case 0x04: d.w = v & 0x07FFFFFFu; return;
        case 0x08: d.c = v; return;
        case 0x0C: d.ad = v; return;
        case 0x10:
            d.en = v & 0x100;
            if ((v & 0x101) == 0x101 && (d.md & 7) == 7) dma_run(l);
            return;
        case 0x14: d.md = v; return;
        }
    }
    switch (off) {
    case 0x60:                                  // DSTP: stop
        return;
    case 0x90: g_t0c = v; return;
    case 0x94: g_t1s = v; return;
    case 0x98: g_t1md = v; return;
    case 0xA0: g_ims = v; return;
    case 0xA4: g_ist &= v; return;              // writing 0 clears
    case 0x80: case 0x84: case 0x88: case 0x8C:
        scu_dsp_write(off, v);
        return;
    }
    g_regs[off / 4] = v;
}
