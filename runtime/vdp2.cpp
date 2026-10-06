// saturnkit runtime — VDP2: the picture of a field, composed in software
// from VRAM, colour RAM, the registers and VDP1's framebuffer.
//
// What is done: the four normal scroll screens (NBG0-NBG3) in cell mode
// (1x1 or 2x2 cells, 1- or 2-word pattern names with the supplement, plane
// sizes, map offsets, 16/256/2048 colours and RGB) and NBG0/NBG1 in bitmap
// mode; NBG0/NBG1's fractional scroll and coordinate increments (zoom), and
// their line scroll (X, Y and X zoom every 1-8 lines); the rotation screen
// RBG0 (parameters A and B, by coefficient too, coefficient tables per line
// or per dot in VRAM or colour RAM, cells or bitmap, screen-over);
// the sprite layer from VDP1's framebuffer, every sprite type, palette and
// RGB mixed (SPCLMD), with its priorities and colour-calculation ratios;
// priorities with the chip's order for ties (sprite, RBG0, NBG0, NBG1, NBG2,
// NBG3), and the special priority and special colour calculation (SFPRMD,
// SFCCMD: per character, per dot by the special function codes, by the
// colour's MSB); colour calculation of the top two layers (ratio of the top one, or
// add); the colour offsets A and B; the back screen (one colour or one a
// line); windows 0 and 1 (rectangles or line tables, inside or outside, OR
// or AND) on NBG0-NBG3 and the sprite layer; the display bit. What is not:
// RBG1, rotation parameters chosen by window, the coefficients' line colour,
// vertical-cell scroll, mosaic, the
// sprite window and the colour-calculation and rotation-parameter windows,
// the line colour screen, shadows on VDP2's layers,
// gradation, extended colour calculation, high and exclusive resolutions.
// A register that asks for one of them is noted once.
#include "saturn.h"
#include "video.h"
#include <algorithm>
#include <cstdlib>
#include <cstring>

uint8_t g_vdp2_vram[0x80000], g_vdp2_cram[0x1000], g_vdp2_regs[0x200];

static uint16_t reg(uint32_t off) { return (uint16_t)(g_vdp2_regs[off] << 8 | g_vdp2_regs[off + 1]); }
static uint8_t vb(uint32_t a) { return g_vdp2_vram[a & 0x7FFFF]; }
static uint16_t vw(uint32_t a) { a &= 0x7FFFE; return (uint16_t)(g_vdp2_vram[a] << 8 | g_vdp2_vram[a + 1]); }

int vdp2_lines() {
    switch (reg(0x00) >> 4 & 3) {
    case 1: return 240;
    case 2: return 256;
    default: return 224;
    }
}

static int cram_mode() { return reg(0x0E) >> 12 & 3; }

// A colour RAM entry as 0x00RRGGBB, and its MSB (the colour calculation bit).
static uint32_t cram(uint32_t i, bool* msb) {
    if (cram_mode() == 2) {
        uint32_t a = (i & 0x3FF) * 4;
        uint32_t v = (uint32_t)g_vdp2_cram[a] << 24 | g_vdp2_cram[a + 1] << 16 | g_vdp2_cram[a + 2] << 8 | g_vdp2_cram[a + 3];
        if (msb) *msb = v >> 31;
        return (v & 0xFF) << 16 | (v & 0xFF00) | (v >> 16 & 0xFF);
    }
    uint32_t a = (i & (cram_mode() == 1 ? 0x7FF : 0x3FF)) * 2;
    uint16_t v = (uint16_t)(g_vdp2_cram[a] << 8 | g_vdp2_cram[a + 1]);
    if (msb) *msb = v >> 15;
    return (uint32_t)((v & 0x1F) << 3) << 16 | ((v >> 5 & 0x1F) << 3) << 8 | ((v >> 10 & 0x1F) << 3);
}

static uint32_t rgb555(uint16_t v) {
    return (uint32_t)((v & 0x1F) << 3) << 16 | ((v >> 5 & 0x1F) << 3) << 8 | ((v >> 10 & 0x1F) << 3);
}

// One layer's pixel on a line.
struct Pix {
    uint32_t rgb;
    uint8_t prio;                               // 0: nothing here
    uint8_t ratio;                              // colour-calculation ratio, 0-31
    bool cc;                                    // colour calculation on
    bool offset;                                // colour offset on
    bool offset_b;                              // ... offset B rather than A
    // for the special priority and colour calculation (SFPRMD, SFCCMD): the pattern name's
    // (or the bitmap's) special bits, the dot's colour code and colour RAM MSB, RGB or not
    bool spr, scc, msb, direct;
    uint16_t dcc;
};

enum { L_SPRITE, L_RBG0, L_NBG0, L_NBG1, L_NBG2, L_NBG3, L_COUNT };

// ---- the scroll screens ---------------------------------------------------------------------
struct Nbg {
    bool on, opaque, bitmap, cell2x2, word1, cnsm;
    int colors;                                 // 0 16, 1 256, 2 2048, 3 32K RGB, 4 16M RGB
    uint16_t supp;                              // the pattern name supplement
    int plane_w, plane_h;                       // in pages
    uint32_t plane[4];                          // A B C D: VRAM addresses
    uint32_t page_bytes;
    int bm_w, bm_h;                             // bitmap size
    uint32_t bm_addr;
    uint16_t bm_pal;
    uint32_t caos;                              // colour RAM offset, in entries
    int32_t sx, sy, dx, dy;                     // scroll and increments, 8 fraction bits
    uint8_t prio, ratio;
    bool cc, offset, offset_b;
    bool bm_spr, bm_scc;                        // a bitmap's special priority and colour bits
    bool line;                                  // NBG0/NBG1's line scroll: each line's X scroll,
    int32_t lx[256], ly[256], ldx[256];         // Y coordinate and X increment
};

// NBG0/NBG1's line scroll table (SCRCTL, LSTA): every 1, 2, 4 or 8 lines it gives an X scroll
// added to SCX, a Y scroll added to SCY (the Y coordinate counts again from it) and an X
// increment that replaces ZMXN, each two words (11 integer bits, then 8 fraction bits).
static void setup_line_scroll(int n, Nbg& s, int lines) {
    uint8_t sc = (uint8_t)(reg(0x9A) >> (n * 8));
    if (!(sc & 0x0E)) return;
    s.line = true;
    int lss = sc >> 4 & 3;
    uint32_t lsta = ((uint32_t)(reg(0xA0 + n * 4) & 7) << 16 | (reg(0xA2 + n * 4) & 0xFFFE)) * 2;
    auto entry = [&](bool zoom) {
        int32_t v = zoom ? (int32_t)((vw(lsta) & 7) << 8 | vw(lsta + 2) >> 8)
                         : (int32_t)((vw(lsta) & 0x7FF) << 8 | vw(lsta + 2) >> 8);
        lsta += 4;
        return v;
    };
    int32_t x = s.sx, y = s.sy, dx = s.dx, acc = 0;
    for (int l = 0; l < lines && l < 256; ++l) {
        if ((l & ((1 << lss) - 1)) == 0) {
            if (sc & 2) x = entry(false) + s.sx;
            if (sc & 4) { acc = 0; y = entry(false) + s.sy; }
            if (sc & 8) dx = entry(true);
        }
        s.lx[l] = x; s.ly[l] = y + acc; s.ldx[l] = dx;
        acc += s.dy;
    }
}

static void setup_nbg(int n, Nbg& s) {
    s = Nbg{};
    uint16_t bgon = reg(0x20);
    s.on = bgon >> n & 1;
    s.opaque = bgon >> (8 + n) & 1;
    if (!s.on) return;
    uint16_t chctl = n < 2 ? reg(0x28) : reg(0x2A);
    int sh = (n & 1) * 8;
    if (n < 2) {
        uint16_t c = (uint16_t)(chctl >> sh);
        s.colors = n == 0 ? c >> 4 & 7 : c >> 4 & 3;
        s.bitmap = c >> 1 & 1;
        s.cell2x2 = c & 1;
        int bmsz = c >> 2 & 3;
        s.bm_w = bmsz & 2 ? 1024 : 512;
        s.bm_h = bmsz & 1 ? 512 : 256;
    } else {
        uint16_t c = (uint16_t)(chctl >> ((n - 2) * 4));
        s.colors = c >> 1 & 1;
        s.cell2x2 = c & 1;
    }
    uint16_t pncn = reg(0x30 + n * 2);
    s.word1 = pncn >> 15 & 1;
    s.cnsm = pncn >> 14 & 1;
    s.supp = pncn & 0x3FF;
    int plsz = reg(0x3A) >> (n * 2) & 3;
    s.plane_w = plsz == 0 ? 1 : 2;
    s.plane_h = plsz == 3 ? 2 : 1;
    uint32_t mpof = reg(0x3C) >> (n * 4) & 7;
    s.page_bytes = (s.cell2x2 ? 0x800u : 0x2000u) * (s.word1 ? 1 : 2);
    uint32_t mask = (uint32_t)(s.plane_w * s.plane_h - 1);
    for (int i = 0; i < 4; ++i) {
        uint16_t mp = reg(0x40 + n * 4 + (i / 2) * 2);
        uint32_t m = (mpof << 6 | (uint32_t)(mp >> ((i & 1) * 8) & 0x3F)) & ~mask;
        s.plane[i] = m * s.page_bytes;
    }
    if (s.bitmap) {
        s.bm_addr = mpof * 0x20000;
        uint16_t bmpn = (uint16_t)(reg(0x2C) >> sh);
        s.bm_pal = (uint16_t)((bmpn & 7) << 4);
        s.bm_spr = bmpn >> 5 & 1;
        s.bm_scc = bmpn >> 4 & 1;
    }
    int caos_sh = n * 4;
    s.caos = (uint32_t)(reg(0xE4) >> caos_sh & 7) << 8;
    if (n < 2) {
        uint32_t b = 0x70 + n * 0x10;
        s.sx = (int32_t)((reg(b) & 0x7FF) << 8 | reg(b + 2) >> 8);
        s.sy = (int32_t)((reg(b + 4) & 0x7FF) << 8 | reg(b + 6) >> 8);
        s.dx = (int32_t)((reg(b + 8) & 7) << 8 | reg(b + 10) >> 8);
        s.dy = (int32_t)((reg(b + 12) & 7) << 8 | reg(b + 14) >> 8);
    } else {
        uint32_t b = 0x90 + (n - 2) * 4;
        s.sx = (int32_t)(reg(b) & 0x7FF) << 8;
        s.sy = (int32_t)(reg(b + 2) & 0x7FF) << 8;
        s.dx = s.dy = 0x100;
    }
    uint16_t prin = reg(0xF8 + (n / 2) * 2);
    s.prio = (uint8_t)(prin >> ((n & 1) * 8) & 7);
    uint16_t ccrn = reg(0x108 + (n / 2) * 2);
    s.ratio = (uint8_t)(ccrn >> ((n & 1) * 8) & 0x1F);
    s.cc = reg(0xEC) >> n & 1;
    s.offset = reg(0x110) >> n & 1;
    s.offset_b = reg(0x112) >> n & 1;
    if (n < 2) setup_line_scroll(n, s, vdp2_lines());
}

// the dot of a character (or bitmap) at (x, y) in its own pixels: false if transparent
static bool dot(const Nbg& s, uint32_t base, int x, int y, int width, uint32_t pal, Pix& p) {
    uint32_t i = (uint32_t)(y * width + x);
    uint32_t v;
    switch (s.colors) {
    case 0: v = vb(base + i / 2); v = (i & 1) ? (v & 0xF) : (v >> 4); break;
    case 1: v = vb(base + i); break;
    case 2: v = vw(base + i * 2) & 0x7FF; break;
    case 3: {
        uint16_t w = vw(base + i * 2);
        if (!(w & 0x8000) && !s.opaque) return false;
        p.rgb = rgb555(w);
        p.direct = true; p.msb = w >> 15;
        return true;
    }
    default: {
        uint32_t w = (uint32_t)vw(base + i * 4) << 16 | vw(base + i * 4 + 2);
        if (!(w & 0x80000000u) && !s.opaque) return false;
        p.rgb = (w & 0xFF) << 16 | (w & 0xFF00) | (w >> 16 & 0xFF);
        p.direct = true; p.msb = w >> 31;
        return true;
    }
    }
    if (v == 0 && !s.opaque) return false;
    uint32_t index = s.colors == 0 ? pal * 16 + v : s.colors == 1 ? (pal & 0x70) * 16 + v : v;
    p.rgb = cram(index + s.caos, &p.msb);
    p.direct = false; p.dcc = (uint16_t)v;
    return true;
}

// The cell of a map of planes (side x side of them, at the addresses in planes[]) at map
// coordinates (sx, sy), already wrapped into the map; or, with pnd_over set, the cell named
// by the pattern name *pnd_over (a rotation screen's screen-over pattern).
static bool cell_pixel(const Nbg& s, const uint32_t* planes, int side, uint32_t sx, uint32_t sy, Pix& p,
                       const uint16_t* pnd_over = nullptr) {
    uint32_t pw = (uint32_t)s.plane_w * 512, ph = (uint32_t)s.plane_h * 512;
    int cs = s.cell2x2 ? 16 : 8;
    uint32_t ix = sx % pw, iy = sy % ph;
    uint32_t pnd_addr = 0;
    if (!pnd_over) {
        int pl = (int)(sy / ph) * side + (int)(sx / pw);
        uint32_t page = (iy / 512) * (uint32_t)s.plane_w + ix / 512;
        uint32_t cx = (ix % 512) / (uint32_t)cs, cy = (iy % 512) / (uint32_t)cs;
        uint32_t cpr = 512 / (uint32_t)cs;
        pnd_addr = planes[pl] + page * s.page_bytes + (cy * cpr + cx) * (s.word1 ? 2 : 4);
    }
    uint32_t chr, pal;
    bool hf, vf;
    if (s.word1 || pnd_over) {
        uint16_t w = pnd_over ? *pnd_over : vw(pnd_addr);
        p.spr = s.supp >> 9 & 1; p.scc = s.supp >> 8 & 1;
        pal = s.colors == 0 ? (uint32_t)(w >> 12 & 0xF) | (s.supp >> 1 & 0x70) : (uint32_t)(w >> 8 & 0x70);
        if (!s.cnsm) {
            hf = w & 0x400; vf = w & 0x800;
            chr = s.cell2x2 ? ((w & 0x3FFu) << 2) | (s.supp & 3u) | ((s.supp & 0x1Cu) << 10)
                            : (w & 0x3FFu) | ((s.supp & 0x1Fu) << 10);
        } else {
            hf = vf = false;
            chr = s.cell2x2 ? ((w & 0xFFFu) << 2) | (s.supp & 3u) | ((s.supp & 0x10u) << 10)
                            : (w & 0xFFFu) | ((s.supp & 0x1Cu) << 10);
        }
    } else {
        uint16_t w0 = vw(pnd_addr), w1 = vw(pnd_addr + 2);
        hf = w0 & 0x4000; vf = w0 & 0x8000;
        p.spr = w0 >> 13 & 1; p.scc = w0 >> 12 & 1;
        pal = w0 & 0x7F;
        chr = w1 & 0x7FFF;
    }
    int px = (int)(ix % (uint32_t)cs), py = (int)(iy % (uint32_t)cs);
    if (hf) px = cs - 1 - px;
    if (vf) py = cs - 1 - py;
    static const uint32_t kCellBytes[] = {32, 64, 128, 128, 256};
    uint32_t base = chr * 0x20;
    if (s.cell2x2) base += (uint32_t)((py >> 3) * 2 + (px >> 3)) * kCellBytes[s.colors];
    return dot(s, base, px & 7, py & 7, 8, pal, p);
}

static bool nbg_pixel(const Nbg& s, int x, int y, Pix& p) {
    int32_t bx = s.sx, by = s.sy + y * s.dy, dx = s.dx;
    if (s.line) { bx = s.lx[y]; by = s.ly[y]; dx = s.ldx[y]; }
    uint32_t sx = (uint32_t)(bx + x * dx) >> 8, sy = (uint32_t)by >> 8;
    if (s.bitmap) {
        sx &= (uint32_t)s.bm_w - 1;
        sy &= (uint32_t)s.bm_h - 1;
        p.spr = s.bm_spr; p.scc = s.bm_scc;
        return dot(s, s.bm_addr, (int)sx, (int)sy, s.bm_w, s.bm_pal, p);
    }
    uint32_t pw = (uint32_t)s.plane_w * 512, ph = (uint32_t)s.plane_h * 512;
    return cell_pixel(s, s.plane, 2, sx & (pw * 2 - 1), sy & (ph * 2 - 1), p);   // the map: 2x2 planes
}

// ---- the rotation screen RBG0 -----------------------------------------------------------------
// Each dot's map coordinates come from rotation parameter A or B (RPMD: A, B, or B where A's
// coefficient is transparent). A parameter is a table in VRAM (RPTA, A then B 0x80 bytes on):
// the screen's start (Xst, Yst, Zst) and its steps per line (dXst, dYst) and per dot (dX, dY),
// the matrix A-F, the viewpoint P, the centre C, the shift M and the scale kx, ky. A coefficient
// table (KTCTL, KTAOF) may give, per line or per dot, kx and ky, one of them, or Xp, and a
// transparent bit. The map is 4x4 planes of the parameter's plane size, or a bitmap; outside it
// (PLSZ's screen-over) it repeats, shows one pattern (OVPNR), or is transparent.
struct RotParam {
    Nbg chr;                                    // characters, plane size, the map's pattern names
    uint32_t plane[16];
    int32_t xst, yst, zst, dxst, dyst, dx, dy, m[6], px, py, pz, cx, cy, cz, mx, my, kx, ky;
    uint32_t ka0;                               // KTAOF and KAst, 10 fraction bits
    int32_t dkast, dkax;
    bool coef, coef1w;
    int coef_mode, over;
    uint16_t over_pn;
    // on the current line
    int32_t xsp, ysp, xp, yp, ldx, ldy;
    uint32_t ka, line_coef;
};

struct Rbg {
    bool on;
    int mode;                                   // RPMD
    bool crkte, perdot, bank_coef[4];
    RotParam par[2];
    uint8_t prio, ratio;
    bool cc, offset, offset_b;
};

static int32_t sext(uint32_t v, int bits) { return (int32_t)(v << (32 - bits)) >> (32 - bits); }
static uint32_t vl(uint32_t a) { return (uint32_t)vw(a) << 16 | vw(a + 2); }

static void setup_rbg(Rbg& r) {
    r = Rbg{};
    uint16_t bgon = reg(0x20);
    r.on = bgon >> 4 & 1;
    if (!r.on) return;
    r.mode = reg(0xB0) & 3;
    uint16_t ramctl = reg(0x0E);
    r.crkte = ramctl >> 15 & 1;
    for (int b = 0; b < 4; ++b) {                // which VRAM banks hold coefficients (RDBS), split or not
        int esb = b & (2 | (ramctl >> (8 + (b >> 1)) & 1));
        r.bank_coef[b] = r.crkte || (ramctl >> (esb * 2) & 3) == 1;
        r.perdot |= r.bank_coef[b];
    }
    r.prio = (uint8_t)(reg(0xFC) & 7);
    r.ratio = (uint8_t)(reg(0x10C) & 0x1F);
    r.cc = reg(0xEC) >> 4 & 1;
    r.offset = reg(0x110) >> 4 & 1;
    r.offset_b = reg(0x112) >> 4 & 1;
    uint16_t chctl = (uint16_t)(reg(0x2A) >> 8), pncr = reg(0x38);
    uint32_t rpta = ((uint32_t)(reg(0xBC) & 7) << 16 | (reg(0xBE) & 0xFFFE)) * 2 & 0xFFF7C;
    for (int i = 0; i < 2; ++i) {
        RotParam& p = r.par[i];
        Nbg& s = p.chr;
        s.on = true;
        s.opaque = bgon >> 12 & 1;
        s.colors = chctl >> 4 & 7;
        s.bitmap = i == 0 && (chctl >> 1 & 1);
        s.cell2x2 = chctl & 1;
        s.bm_w = 512;
        s.bm_h = chctl >> 2 & 1 ? 512 : 256;
        s.bm_pal = (uint16_t)((reg(0x2E) & 7) << 4);
        s.bm_spr = reg(0x2E) >> 5 & 1;
        s.bm_scc = reg(0x2E) >> 4 & 1;
        s.word1 = pncr >> 15 & 1;
        s.cnsm = pncr >> 14 & 1;
        s.supp = pncr & 0x3FF;
        s.caos = (uint32_t)(reg(0xE6) & 7) << 8;
        int plsz = reg(0x3A) >> (8 + i * 4) & 3;
        s.plane_w = plsz == 0 ? 1 : 2;
        s.plane_h = plsz == 3 ? 2 : 1;
        s.page_bytes = (s.cell2x2 ? 0x800u : 0x2000u) * (s.word1 ? 1 : 2);
        p.over = reg(0x3A) >> (10 + i * 4) & 3;
        p.over_pn = reg(0xB8 + i * 2);
        uint32_t mpof = reg(0x3E) >> (i * 4) & 7, mask = (uint32_t)(s.plane_w * s.plane_h - 1);
        for (int k = 0; k < 16; ++k) {
            uint32_t mp = reg(0x50 + i * 0x10 + (k / 2) * 2) >> ((k & 1) * 8) & 0x3F;
            p.plane[k] = ((mpof << 6 | mp) & ~mask) * s.page_bytes;
        }
        s.bm_addr = mpof * 0x20000;
        uint32_t a = rpta + (uint32_t)i * 0x80;
        p.xst = sext(vl(a + 0x00) >> 6, 23);
        p.yst = sext(vl(a + 0x04) >> 6, 23);
        p.zst = sext(vl(a + 0x08) >> 6, 23);
        p.dxst = sext(vl(a + 0x0C) >> 6, 13);
        p.dyst = sext(vl(a + 0x10) >> 6, 13);
        p.dx = sext(vl(a + 0x14) >> 6, 13);
        p.dy = sext(vl(a + 0x18) >> 6, 13);
        for (int k = 0; k < 6; ++k) p.m[k] = sext(vl(a + 0x1C + k * 4) >> 6, 14);
        p.px = sext(vw(a + 0x34), 14); p.py = sext(vw(a + 0x36), 14); p.pz = sext(vw(a + 0x38), 14);
        p.cx = sext(vw(a + 0x3C), 14); p.cy = sext(vw(a + 0x3E), 14); p.cz = sext(vw(a + 0x40), 14);
        p.mx = sext(vl(a + 0x44) >> 6, 24);
        p.my = sext(vl(a + 0x48) >> 6, 24);
        p.kx = sext(vl(a + 0x4C), 24);
        p.ky = sext(vl(a + 0x50), 24);
        uint16_t ktctl = (uint16_t)(reg(0xB4) >> (i * 8));
        p.coef = ktctl & 1;
        p.coef1w = ktctl >> 1 & 1;
        p.coef_mode = ktctl >> 2 & 3;
        p.ka0 = ((uint32_t)(reg(0xB6) >> (i * 8) & 7) << 26) + (vl(a + 0x54) >> 6);
        p.dkast = sext(vl(a + 0x58) >> 6, 20);
        p.dkax = sext(vl(a + 0x5C) >> 6, 20);
    }
}

// a coefficient's address (in words) from its table address (10 fraction bits)
static uint32_t coef_addr(const Rbg& r, const RotParam& p, uint32_t ka) {
    uint32_t a = (ka >> 10) << (p.coef1w ? 0 : 1);
    return a & (r.crkte ? 0x3FFu : 0x3FFFFu);
}

// a coefficient: bit 31 transparent, bits 23-0 the value (7.16)
static uint32_t coef_read(const Rbg& r, const RotParam& p, uint32_t addr) {
    auto word = [&](uint32_t w) -> uint16_t {
        if (!r.crkte) return vw(w * 2);
        uint32_t b = (0x800 + w * 2) & 0xFFE;
        return (uint16_t)(g_vdp2_cram[b] << 8 | g_vdp2_cram[b + 1]);
    };
    if (p.coef1w) {
        uint16_t t = word(addr);
        return ((uint32_t)sext((uint32_t)t << 6, 21) & 0xFFFFFF) | (uint32_t)(t & 0x8000) << 16;
    }
    return (uint32_t)word(addr) << 16 | word(addr + 1);
}

static void rbg_line(Rbg& r, int y) {
    for (RotParam& p : r.par) {
        int64_t x0 = p.xst + (int64_t)p.dxst * y - p.px * 1024, y0 = p.yst + (int64_t)p.dyst * y - p.py * 1024;
        int64_t z0 = p.zst - p.pz * 1024;
        p.xsp = (int32_t)((p.m[0] * x0 + p.m[1] * y0 + p.m[2] * z0) >> 10);
        p.ysp = (int32_t)((p.m[3] * x0 + p.m[4] * y0 + p.m[5] * z0) >> 10);
        p.xp = p.m[0] * (p.px - p.cx) + p.m[1] * (p.py - p.cy) + p.m[2] * (p.pz - p.cz) + p.cx * 1024 + p.mx;
        p.yp = p.m[3] * (p.px - p.cx) + p.m[4] * (p.py - p.cy) + p.m[5] * (p.pz - p.cz) + p.cy * 1024 + p.my;
        p.ldx = (p.m[0] * p.dx + p.m[1] * p.dy) >> 10;
        p.ldy = (p.m[3] * p.dx + p.m[4] * p.dy) >> 10;
        p.ka = p.ka0 + (uint32_t)(p.dkast * y);
        p.line_coef = coef_read(r, p, coef_addr(r, p, p.ka));
    }
}

static bool rbg_pixel(const Rbg& r, int x, Pix& out) {
    int sel = r.mode == 1 ? 1 : 0;              // RPMD 3 (by the rotation-parameter window) is taken as A
    auto coef_at = [&](int i) {
        const RotParam& p = r.par[i];
        if (!r.perdot) return p.line_coef;
        uint32_t a = coef_addr(r, p, p.ka + (uint32_t)(x * p.dkax));
        return r.bank_coef[r.crkte ? 0 : (a >> 16) & 3] ? coef_read(r, p, a) : 0u;
    };
    uint32_t c = 0;
    if (r.mode == 2) {
        c = r.par[0].coef ? coef_at(0) : 0;
        if (c >> 31) { sel = 1; c = r.par[1].line_coef; }
    } else if (r.par[sel].coef) {
        c = coef_at(sel);
    }
    const RotParam& p = r.par[sel];
    int32_t kx = p.kx, ky = p.ky;
    uint32_t xp = (uint32_t)p.xp;
    if (p.coef) {
        if (c >> 31) return false;              // a transparent coefficient
        int32_t v = sext(c, 24);
        switch (p.coef_mode) {
        case 0: kx = ky = v; break;
        case 1: kx = v; break;
        case 2: ky = v; break;
        default: xp = (uint32_t)v << 2; break;
        }
    }
    uint32_t ix = (xp + (uint32_t)(((int64_t)kx * (int32_t)(p.xsp + p.ldx * x)) >> 16)) >> 10;
    uint32_t iy = ((uint32_t)p.yp + (uint32_t)(((int64_t)ky * (int32_t)(p.ysp + p.ldy * x)) >> 16)) >> 10;
    const Nbg& s = p.chr;
    uint32_t mw = s.bitmap ? (uint32_t)s.bm_w : (uint32_t)s.plane_w * 512 * 4;
    uint32_t mh = s.bitmap ? (uint32_t)s.bm_h : (uint32_t)s.plane_h * 512 * 4;
    bool outside = false;
    if (p.over == 3) outside = (ix | iy) & ~511u;
    else if (p.over) outside = (ix & ~(mw - 1)) || (iy & ~(mh - 1));
    if (outside && (p.over & 2)) return false;
    if (s.bitmap) {
        out.spr = s.bm_spr; out.scc = s.bm_scc;
        return dot(s, s.bm_addr, (int)(ix & (mw - 1)), (int)(iy & (mh - 1)), s.bm_w, s.bm_pal, out);
    }
    return cell_pixel(s, p.plane, 4, ix & (mw - 1), iy & (mh - 1), out, outside ? &p.over_pn : nullptr);
}

// ---- the sprite layer -----------------------------------------------------------------------
struct SpriteType { int pr_shift, pr_mask, cc_shift, cc_mask, dc_mask; };
static const SpriteType kTypes[16] = {
    {14, 3, 11, 7, 0x7FF}, {13, 7, 11, 3, 0x7FF}, {14, 1, 11, 7, 0x7FF}, {13, 3, 11, 3, 0x7FF},
    {13, 3, 10, 7, 0x3FF}, {12, 7, 11, 1, 0x7FF}, {12, 7, 10, 3, 0x3FF}, {12, 7, 9, 7, 0x1FF},
    {7, 1, 0, 0, 0x7F},    {7, 1, 6, 1, 0x3F},    {6, 3, 0, 0, 0x3F},    {0, 0, 6, 3, 0x3F},
    {7, 1, 0, 0, 0xFF},    {7, 1, 6, 1, 0xFF},    {6, 3, 0, 0, 0xFF},    {0, 0, 6, 3, 0xFF},
};

static bool sprite_pixel(uint16_t d, Pix& p) {
    uint16_t spctl = reg(0xE0);
    int type = spctl & 0xF;
    bool mixed = spctl >> 5 & 1;
    int ccmode = spctl >> 12 & 3, ccnum = spctl >> 8 & 7;
    if (d == 0) return false;
    auto prio_of = [](int i) { return (uint8_t)(reg(0xF0 + (i / 2) * 2) >> ((i & 1) * 8) & 7); };
    auto ratio_of = [](int i) { return (uint8_t)(reg(0x100 + (i / 2) * 2) >> ((i & 1) * 8) & 0x1F); };
    bool msb;
    if (mixed && (d & 0x8000)) {
        p.rgb = rgb555(d);
        p.prio = prio_of(0);
        p.ratio = ratio_of(0);
        msb = true;
    } else {
        const SpriteType& t = kTypes[type];
        if (type >= 8) d &= 0xFF;
        uint16_t dc = d & t.dc_mask;
        if (dc == 0) return false;
        int pr = t.pr_mask ? d >> t.pr_shift & t.pr_mask : 0;
        int cc = t.cc_mask ? d >> t.cc_shift & t.cc_mask : 0;
        p.prio = prio_of(pr);
        p.ratio = ratio_of(cc);
        uint32_t caos = (uint32_t)(reg(0xE6) >> 4 & 7) << 8;
        p.rgb = cram(dc + caos, &msb);
    }
    switch (ccmode) {
    case 0: p.cc = p.prio <= ccnum; break;
    case 1: p.cc = p.prio == ccnum; break;
    case 2: p.cc = p.prio >= ccnum; break;
    default: p.cc = msb; break;
    }
    p.cc = p.cc && (reg(0xEC) >> 6 & 1);
    p.offset = reg(0x110) >> 6 & 1;
    p.offset_b = reg(0x112) >> 6 & 1;
    return p.prio != 0;
}

// ---- the windows ----------------------------------------------------------------------------
// Windows 0 and 1: a rectangle (WPSX/WPSY/WPEX/WPEY), or with the line window table on
// (LWTA bit 15) a start and end X for each line read from VRAM. A layer's WCTL byte: bit 0/2
// the area of W0/W1 (0 inside, 1 outside), bit 1/3 their enable, bit 7 the logic (0 OR, 1 AND).
// Where the enabled windows' areas combine, the layer is transparent.
struct Window { int x0, y0, x1, y1; bool lines; uint32_t table; };
static Window g_win[2];

static void setup_windows() {
    bool hires = (reg(0x00) & 6) != 0;          // X counts pixels in hi-res, half-pixels otherwise
    for (int i = 0; i < 2; ++i) {
        Window& w = g_win[i];
        uint32_t b = 0xC0 + i * 8;
        int sh = hires ? 0 : 1;
        w.x0 = (reg(b) & 0x3FF) >> sh;
        w.y0 = reg(b + 2) & 0x1FF;
        w.x1 = (reg(b + 4) & 0x3FF) >> sh;
        w.y1 = reg(b + 6) & 0x1FF;
        uint16_t hi = reg(0xD8 + i * 4), lo = reg(0xDA + i * 4);
        w.lines = hi & 0x8000;
        w.table = ((uint32_t)(hi & 7) << 16 | (lo & 0xFFFE)) << 1;
    }
}

static bool window_area(int i, int x, int y) {
    const Window& w = g_win[i];
    if (y < w.y0 || y > w.y1) return false;
    int x0 = w.x0, x1 = w.x1;
    if (w.lines) {
        int sh = (reg(0x00) & 6) ? 0 : 1;
        x0 = (vw(w.table + (uint32_t)y * 4) & 0x3FF) >> sh;
        x1 = (vw(w.table + (uint32_t)y * 4 + 2) & 0x3FF) >> sh;
    }
    return x >= x0 && x <= x1;
}

// true where the windows make this layer transparent (ctl: the layer's WCTL byte)
static bool windowed(uint8_t ctl, int x, int y) {
    bool used = false, any = false, all = true;
    for (int i = 0; i < 2; ++i) {
        if (!(ctl >> (i * 2 + 1) & 1)) continue;
        bool in = window_area(i, x, y);
        bool area = (ctl >> (i * 2) & 1) ? !in : in;
        used = true; any |= area; all &= area;
    }
    return used && (ctl & 0x80 ? all : any);
}

static uint8_t wctl_of(int layer) {
    switch (layer) {
    case L_SPRITE: return (uint8_t)(reg(0xD4) >> 8);
    case L_RBG0: return (uint8_t)reg(0xD4);
    case L_NBG0: return (uint8_t)reg(0xD0);
    case L_NBG1: return (uint8_t)(reg(0xD0) >> 8);
    case L_NBG2: return (uint8_t)reg(0xD2);
    default: return (uint8_t)(reg(0xD2) >> 8);
    }
}

// ---- composing ------------------------------------------------------------------------------
static void note_unsupported() {
    static bool noted[8];
    auto once = [](int i, bool cond, const char* what, uint16_t v) {
        if (cond && !noted[i]) { noted[i] = true; sat_note("VDP2: %s (%04X) is not done", what, v); }
    };
    once(0, (reg(0x20) & 0x20) || ((reg(0x20) & 0x10) && (reg(0xB0) & 3) == 3),
         "RBG1, or rotation parameters chosen by window (BGON, RPMD)", reg(0x20));
    once(1, reg(0x9A) & 0x0101, "vertical cell scroll (SCRCTL)", reg(0x9A));
    once(2, reg(0x22) & 0xF, "mosaic (MZCTL)", reg(0x22));
    once(3, ((reg(0xD0) | reg(0xD2) | reg(0xD4)) & 0x2020) || (reg(0xD6) & 0x2A2A),
         "the sprite window, or the rotation-parameter or colour-calculation window (WCTL)",
         reg(0xD0) | reg(0xD2) | reg(0xD4) | reg(0xD6));
    once(4, reg(0xE2) & 0x13F, "shadows on VDP2's layers (SDCTL)", reg(0xE2));
    once(5, reg(0xEC) & 0x8600 && reg(0xEC) & 0x5F, "extended colour calculation or gradation (CCCTL)", reg(0xEC));
    once(6, (reg(0x00) & 7) >= 2, "a high or exclusive resolution (TVMD)", reg(0x00));
    once(7, reg(0xE8) & 0x3F, "the line colour screen (LNCLEN)", reg(0xE8));
}

// ---- the special functions ------------------------------------------------------------------
// layer: 0-3 NBG0-NBG3, 4 RBG0 (two bits each in SFPRMD and SFCCMD, one in SFSEL)
// SFPRMD: 0 the screen's priority; 1 its LSB from the character's (or bitmap's) special
// priority bit; 2 that bit only where the dot's colour code is one of the special function
// codes (SFSEL picks code A or B, SFCODE has them; codes are the dot's bits 3-1). SFCCMD alike
// for colour calculation, and 3: the colour's MSB.
static bool sf_code(int layer, const Pix& p) {
    uint8_t code = (uint8_t)(reg(0x26) >> ((reg(0x24) >> layer & 1) * 8));
    return !p.direct && (code >> ((p.dcc & 0xE) >> 1) & 1);
}

static uint8_t special_prio(int layer, uint8_t prio, const Pix& p) {
    int mode = reg(0xEA) >> (layer * 2) & 3;
    if (!mode) return prio;
    bool bit = mode == 1 ? p.spr : mode == 2 ? (p.spr && sf_code(layer, p)) : false;
    return (uint8_t)((prio & ~1) | (bit ? 1 : 0));
}

static bool special_cc(int layer, bool cc, const Pix& p) {
    switch (reg(0xEE) >> (layer * 2) & 3) {
    case 0: return cc;
    case 1: return cc && p.scc;
    case 2: return cc && p.scc && sf_code(layer, p);
    default: return cc && p.msb;
    }
}

static uint32_t apply_offset(uint32_t rgb, bool b) {
    uint32_t base = b ? 0x11A : 0x114;
    int out = 0;
    for (int i = 0; i < 3; ++i) {
        int o = (int)((uint32_t)(reg(base + i * 2) & 0x1FF) << 23) >> 23;
        int c = (int)(rgb >> (16 - i * 8) & 0xFF) + o;
        out = out << 8 | std::clamp(c, 0, 255);
    }
    return (uint32_t)out;
}

void vdp2_compose(Frame& f) {
    uint16_t tvmd = reg(0x00);
    f.w = (tvmd & 7) == 1 ? 352 : 320;
    f.h = vdp2_lines();
    f.px.assign((size_t)f.w * f.h, 0);
    if (!(tvmd & 0x8000)) return;                // display off
    note_unsupported();
    // a debugging aid: SATURNKIT_VDP2_HIDE=MASK leaves out layers (1 sprite, 2 RBG0, 4-32 NBG0-NBG3)
    static const unsigned hide = [] { const char* e = std::getenv("SATURNKIT_VDP2_HIDE"); return e ? (unsigned)std::strtoul(e, nullptr, 0) : 0u; }();
    static Nbg nbg[4];
    static Rbg rbg;
    uint8_t wctl[L_COUNT];
    uint32_t bk = 0;
    bool bk_lines = false, add = false;
    uint16_t sfprmd = 0;
    Pix back{};
    auto setup = [&] {
        for (int n = 0; n < 4; ++n) setup_nbg(n, nbg[n]);
        setup_rbg(rbg);
        setup_windows();
        for (int i = 0; i < L_COUNT; ++i) wctl[i] = wctl_of(i) & 0x8F;    // W0, W1, the logic
        bk = ((uint32_t)(reg(0xAC) & 7) << 16 | reg(0xAE)) * 2;
        bk_lines = reg(0xAC) & 0x8000;
        add = reg(0xEC) >> 8 & 1;
        sfprmd = reg(0xEA);
        back.offset = reg(0x110) >> 5 & 1;
        back.offset_b = reg(0x112) >> 5 & 1;
    };
    setup();
    // Raster effects: each line sees the registers as the HBlank handlers left them by then
    // (those they never wrote as the field ends; those they wrote as the field began, until
    // the line of their first write).
    const std::vector<RasterWrite>& raster = video_raster_writes();
    uint8_t end_regs[0x200], cur[0x200];
    size_t next_write = 0;
    if (!raster.empty()) {
        std::memcpy(end_regs, g_vdp2_regs, sizeof end_regs);
        std::memcpy(cur, g_vdp2_regs, sizeof cur);
        const uint8_t* start = video_field_regs();
        for (const RasterWrite& w : raster)
            for (int b = 0; b < w.size; ++b) cur[(w.off + b) & 0x1FF] = start[(w.off + b) & 0x1FF];
    }
    const uint16_t* fb = vdp1_display();
    for (int y = 0; y < f.h; ++y) {
        if (!raster.empty()) {
            bool changed = y == 0;
            for (; next_write < raster.size() && raster[next_write].line <= y; ++next_write, changed = true) {
                const RasterWrite& w = raster[next_write];
                for (int b = 0; b < w.size; ++b) cur[(w.off + b) & 0x1FF] = (uint8_t)(w.value >> (8 * (w.size - 1 - b)));
            }
            if (changed) { std::memcpy(g_vdp2_regs, cur, sizeof cur); setup(); }
        }
        back.rgb = rgb555(vw(bk + (bk_lines ? (uint32_t)y * 2 : 0)));
        if (rbg.on) rbg_line(rbg, y);
        for (int x = 0; x < f.w; ++x) {
            Pix l[L_COUNT];
            int top = -1, second = -1;
            auto consider = [&](int i) {
                if (!l[i].prio) return;
                if (top < 0 || l[i].prio > l[top].prio) { second = top; top = i; }
                else if (second < 0 || l[i].prio > l[second].prio) second = i;
            };
            // in the order that wins ties: sprite, RBG0, then NBG0..NBG3
            l[L_SPRITE] = {};
            if (!(hide & 1) && !(wctl[L_SPRITE] && windowed(wctl[L_SPRITE], x, y)) && sprite_pixel(fb[y * 512 + x], l[L_SPRITE]))
                consider(L_SPRITE);
            Pix& rp = l[L_RBG0];
            rp = {};
            if (rbg.on && (rbg.prio || sfprmd >> 8 & 3) && !(hide & 2) && !(wctl[L_RBG0] && windowed(wctl[L_RBG0], x, y))
                && rbg_pixel(rbg, x, rp) && (rp.prio = special_prio(4, rbg.prio, rp))) {
                rp.ratio = rbg.ratio; rp.cc = special_cc(4, rbg.cc, rp); rp.offset = rbg.offset; rp.offset_b = rbg.offset_b;
                consider(L_RBG0);
            }
            for (int n = 0; n < 4; ++n) {
                Pix& p = l[L_NBG0 + n];
                p = {};
                const Nbg& s = nbg[n];
                if (!s.on || (!s.prio && !(sfprmd >> (n * 2) & 3)) || (hide >> (2 + n) & 1)) continue;
                if (wctl[L_NBG0 + n] && windowed(wctl[L_NBG0 + n], x, y)) continue;
                if (!nbg_pixel(s, x, y, p)) continue;
                if (!(p.prio = special_prio(n, s.prio, p))) { p = {}; continue; }
                p.ratio = s.ratio; p.cc = special_cc(n, s.cc, p); p.offset = s.offset; p.offset_b = s.offset_b;
                consider(L_NBG0 + n);
            }
            const Pix& t = top >= 0 ? l[top] : back;
            const Pix& u = second >= 0 ? l[second] : back;
            uint32_t rgb = t.rgb;
            if (top >= 0 && t.cc) {
                uint32_t out = 0;
                for (int sh = 16; sh >= 0; sh -= 8) {
                    int a = (int)(t.rgb >> sh & 0xFF), b = (int)(u.rgb >> sh & 0xFF);
                    int c = add ? std::min(a + b, 255) : (a * (32 - t.ratio) + b * t.ratio) >> 5;
                    out |= (uint32_t)c << sh;
                }
                rgb = out;
            }
            if (t.offset) rgb = apply_offset(rgb, t.offset_b);
            f.px[(size_t)y * f.w + x] = rgb;
        }
    }
    if (!raster.empty()) std::memcpy(g_vdp2_regs, end_regs, sizeof end_regs);
}
