// saturnkit runtime — the SH7604's on-chip registers, one set per CPU
// (0xFFFFFE00-0xFFFFFFFF): the division unit, the free-running timer, the
// DMA controller; the rest (serial port, watchdog, bus and cache control,
// interrupt priorities) kept as written.
//
// DIVU: a write to DVDNT (or DVDNTL) starts a 32/32 (or 64/32) signed
// division; the quotient is read at once from DVDNT/DVDNTL, the remainder from
// DVDNTH, and both again from their shadows at 0x118/0x11C (written only by a
// division or a store there, as in Mednafen). Overflow sets DVCR.OVF. The FRT's counter runs from time at φ/8
// (TCR selects /8, /32, /128); SINIT/MINIT set the input-capture flag and
// latch the count in FICR. The DMAC transfers at once when enabled in
// auto-request mode.
#include "saturn.h"

struct OnChip {
    uint8_t regs[0x200];
    uint64_t frc_t0;                        // time the FRC was last written
    uint16_t frc0;
};
static OnChip g_oc[2];

static uint32_t rd(OnChip& o, uint32_t off, int size) { return mem_rd(o.regs, off, size); }
static void wr(OnChip& o, uint32_t off, uint32_t v, int size) { mem_wr(o.regs, off, v, size); }

void onchip_reset(int cpu) {
    OnChip& o = g_oc[cpu];
    for (auto& b : o.regs) b = 0;
    o.frc_t0 = sat_now();
    o.frc0 = 0;
    o.regs[0x11] = 0x01;                    // FTCSR
    o.regs[0x16] = 0x00;                    // TCR
    o.regs[0xE2] = 0; o.regs[0xE3] = 0;     // IPRA
    o.regs[0x92] = 0;                       // CCR
}

static uint16_t frc(OnChip& o) {
    static const int div[4] = {8, 32, 128, 1};
    int d = div[o.regs[0x16] & 3];
    uint64_t clocks = (sat_now() - o.frc_t0) * 28636 / 1000000;   // φ = 28.6 MHz
    return (uint16_t)(o.frc0 + clocks / d);
}

void onchip_input_capture(int cpu) {
    OnChip& o = g_oc[cpu];
    o.regs[0x11] |= 0x80;                   // ICF
    uint16_t f = frc(o);
    o.regs[0x18] = f >> 8; o.regs[0x19] = (uint8_t)f;   // FICR
}

static void divide(OnChip& o, bool wide) {
    int32_t dvsr = (int32_t)rd(o, 0x100, 4);
    int64_t n = wide ? (int64_t)((uint64_t)rd(o, 0x110, 4) << 32 | rd(o, 0x114, 4)) : (int32_t)rd(o, 0x104, 4);
    int64_t q, r;
    bool ovf = dvsr == 0;
    if (!ovf) {
        q = n / dvsr; r = n % dvsr;
        ovf = q > INT32_MAX || q < INT32_MIN;
    }
    if (ovf) {
        wr(o, 0x108, rd(o, 0x108, 4) | 1, 4);
        q = ((n < 0) != (dvsr < 0)) ? INT32_MIN : INT32_MAX;
        r = 0;
    }
    wr(o, 0x104, (uint32_t)q, 4); wr(o, 0x114, (uint32_t)q, 4);
    wr(o, 0x110, (uint32_t)r, 4);
    wr(o, 0x118, (uint32_t)r, 4); wr(o, 0x11C, (uint32_t)q, 4);   // DVDNTH/DVDNTL's shadows (SGL reads 0x11C)
    // the unit's registers are mirrored at +0x20
    for (uint32_t k = 0x100; k < 0x120; k += 4) wr(o, k + 0x20, rd(o, k, 4), 4);
}

static void dma(OnChip& o, int ch) {
    uint32_t base = 0x180 + ch * 0x10;
    uint32_t chcr = rd(o, base + 0xC, 4), dmaor = rd(o, 0x1B0, 4);
    if (!(chcr & 1) || (chcr & 2) || !(dmaor & 1)) return;
    if (!(chcr >> 9 & 1)) { sat_trace("DMAC %d: waits for an external request", ch); return; }
    uint32_t sar = rd(o, base, 4), dar = rd(o, base + 4, 4), tcr = rd(o, base + 8, 4) & 0xFFFFFF;
    if (!tcr) tcr = 0x1000000;
    static const int unit[4] = {1, 2, 4, 16};
    int ts = chcr >> 10 & 3, u = unit[ts];
    int sm = chcr >> 12 & 3, dm = chcr >> 14 & 3;
    auto step = [&](int mode) { return mode == 1 ? u : mode == 2 ? -u : 0; };
    uint32_t n = ts == 3 ? (tcr + 3) / 4 : tcr;   // 16-byte units: TCR counts longwords
    for (uint32_t i = 0; i < n; ++i) {
        if (ts == 3) {
            for (int k = 0; k < 16; k += 4) st32(dar + (dm ? k : 0), ld32(sar + (sm ? k : 0)));
            sar += step(sm); dar += step(dm);
            continue;
        }
        uint32_t v = u == 1 ? ld8(sar) : u == 2 ? ld16(sar) : ld32(sar);
        if (u == 1) st8(dar, v); else if (u == 2) st16(dar, v); else st32(dar, v);
        sar += step(sm); dar += step(dm);
    }
    sat_trace("DMAC %d: %u bytes %08X -> %08X", ch, ts == 3 ? n * 16 : tcr * u, rd(o, base, 4), rd(o, base + 4, 4));
    wr(o, base, sar, 4); wr(o, base + 4, dar, 4); wr(o, base + 8, 0, 4);
    wr(o, base + 0xC, chcr | 2, 4);          // TE
    if (chcr & 4) sat_fatal("DMAC %d: end interrupt requested, not emulated", ch);
}

uint32_t onchip_read(SH2Context& c, uint32_t a, int size) {
    OnChip& o = g_oc[c.cpu];
    uint32_t off = a & 0x1FF;
    if (off == 0x12 || off == 0x13) {       // FRC
        uint16_t f = frc(o);
        if (size == 2) return f;
        return off == 0x12 ? f >> 8 : f & 0xFF;
    }
    uint32_t v = rd(o, off, size);
    if (off == 0x11 && size == 1) {         // FTCSR: a slave waiting here has nothing to do
        slave_idle_check(v);
        v = rd(o, off, size);
    }
    return v;
}

void onchip_write(SH2Context& c, uint32_t a, uint32_t v, int size) {
    OnChip& o = g_oc[c.cpu];
    uint32_t off = a & 0x1FF;
    switch (off) {
    case 0x11:                              // FTCSR: flags clear by writing 0
        o.regs[0x11] = (uint8_t)((o.regs[0x11] & (v | 0x01) & 0x8E) | (v & 0x01));
        return;
    case 0x12: case 0x13:
        if (size == 2) o.frc0 = (uint16_t)v;
        else o.frc0 = off == 0x12 ? (uint16_t)((v & 0xFF) << 8 | (frc(o) & 0xFF)) : (uint16_t)((frc(o) & 0xFF00) | (v & 0xFF));
        o.frc_t0 = sat_now();
        return;
    }
    // DIVU at 0x100-0x11F and its mirror at 0x120-0x13F
    if (off >= 0x100 && off < 0x140) {
        off = 0x100 + ((off - 0x100) & 0x1F);
        wr(o, off, v, size);
        if (off == 0x104) { wr(o, 0x114, v, 4); wr(o, 0x110, (int32_t)v < 0 ? 0xFFFFFFFFu : 0, 4); divide(o, false); }
        else if (off == 0x114) divide(o, true);
        else for (uint32_t k = 0x100; k < 0x120; k += 4) wr(o, k + 0x20, rd(o, k, 4), 4);
        return;
    }
    wr(o, off, v, size);
    if (off >= 0x18C && off < 0x190) dma(o, 0);
    else if (off >= 0x19C && off < 0x1A0) dma(o, 1);
    else if (off >= 0x1B0 && off < 0x1B4) { dma(o, 0); dma(o, 1); }
}
