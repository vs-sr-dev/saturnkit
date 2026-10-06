// saturnkit runtime — the raster timing that drives the frame, and the video
// and sound chips' place on the bus (the sound side is sound.cpp's).
//
// Timing is NTSC: 263 lines at 59.94 Hz, whatever the disc's area (TVSTAT's
// PAL bit reads 0). Each line reached runs the SCU's timer 0 compare and
// raises HBlank-IN; line 0 raises VBlank-OUT, the first line after the
// display (224, or 240 with TVMD's VRESO 1; 224 too for VRESO 2, PAL's 256
// lines, which a 60 Hz raster has no room for) VBlank-IN. At VBlank-IN the
// field that has ended is composed (vdp2.cpp) when a window or a picture
// file wants it; at VBlank-OUT VDP1 erases and changes frame (vdp1.cpp).
//
// VDP1 is vdp1.cpp, VDP2's picture vdp2.cpp; VDP2's registers, VRAM and
// colour RAM keep what is written.
#include "saturn.h"
#include "video.h"
#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

static const uint64_t kFrameNs = 16683350;      // 59.94 Hz
static const int kLines = 263;
static uint64_t g_line_abs;                      // raster lines since power-on
static uint64_t g_vblanks;
static Frame g_frame;

uint64_t sat_vblanks() { return g_vblanks; }
uint64_t video_frame_changes() { return vdp1_frame_changes(); }
uint64_t video_draws() { return vdp1_draws(); }

static uint16_t reg16(const uint8_t* r, uint32_t o) { return (uint16_t)(r[o] << 8 | r[o + 1]); }
static void set16(uint8_t* r, uint32_t o, uint16_t v) { r[o] = v >> 8; r[o + 1] = (uint8_t)v; }

void video_init() {
    vdp1_init();
    if (!host_open()) sat_fatal("no window");
}

static int display_lines() {
    int vreso = reg16(g_vdp2_regs, 0x00) >> 4 & 3;
    return vreso == 1 ? 240 : 224;
}

static bool listed(const std::string& list, const std::string& item) {
    return !list.empty() && ("," + list + ",").find("," + item + ",") != std::string::npos;
}

// --dump: VDP1 VRAM, its draw framebuffer, VDP1's registers, VDP2 VRAM, CRAM, VDP2's registers,
// one after the other (big-endian words)
static void dump(const char* prefix, uint64_t n) {
    char path[512];
    std::snprintf(path, sizeof path, "%s/dump-%s%llu.bin", g_cfg.out.c_str(), prefix, (unsigned long long)n);
    FILE* f = std::fopen(path, "wb");
    if (!f) return;
    vdp1_dump(f);
    std::fwrite(g_vdp2_vram, 1, sizeof g_vdp2_vram, f);
    std::fwrite(g_vdp2_cram, 1, sizeof g_vdp2_cram, f);
    std::fwrite(g_vdp2_regs, 1, 0x200, f);
    std::fclose(f);
}

static void vblank_in(uint64_t now) {
    ++g_vblanks;
    std::string n = std::to_string(g_vblanks);
    if (listed(g_cfg.dump, n)) dump("", g_vblanks);
    bool shot = listed(g_cfg.shots, n);
    bool compose = shot || host_wants_frame();
    if (vdp1_interp_field(compose) && compose) {
        uint8_t saved[0x30];
        std::copy(g_vdp2_regs + 0x70, g_vdp2_regs + 0xA0, saved);
        vdp1_interp_scroll(g_vdp2_regs + 0x70);
        vdp2_compose(g_frame);
        std::copy(saved, saved + 0x30, g_vdp2_regs + 0x70);
        if (shot) write_png(g_cfg.out + "/shot-" + n + ".png", g_frame);
        host_present(g_frame);
    } else if (compose) {
        vdp2_compose(g_frame);
        if (shot) write_png(g_cfg.out + "/shot-" + n + ".png", g_frame);
        host_present(g_frame);
    }
    scu_raise(IRQ_VBLANK_IN);
    scu_frame_event(0);
    host_pace(now);
}

void video_tick(uint64_t now) {
    uint64_t target = now / (kFrameNs / kLines);
    while (g_line_abs < target) {
        ++g_line_abs;
        int line = (int)(g_line_abs % kLines);
        if (line == 0) { vdp1_vblank_out(); scu_raise(IRQ_VBLANK_OUT); scu_frame_event(1); }
        if (line == display_lines()) vblank_in(g_line_abs * (kFrameNs / kLines));
        scu_raise(IRQ_HBLANK_IN);
        scu_line(line);
    }
    vdp1_tick(now);
}

// ---- the bus -------------------------------------------------------------------------
bool video_owns(uint32_t a) {
    return (a >= 0x05A00000u && a < 0x05B01000u) || (a >= 0x05C00000u && a < 0x05D00020u) ||
           (a >= 0x05E00000u && a < 0x05F80200u);
}

static uint8_t* area(uint32_t a, uint32_t& off) {
    if (a < 0x05C80000u) { off = a & 0x7FFFF; return g_vdp1_vram; }
    if (a < 0x05E00000u) return nullptr;                  // VDP1's framebuffer and registers
    if (a < 0x05F00000u) { off = a & 0x7FFFF; return g_vdp2_vram; }
    if (a < 0x05F80000u) { off = a & 0xFFF; return g_vdp2_cram; }
    return nullptr;
}

uint32_t video_read(uint32_t a, int size) {
    if (a < 0x05B01000u) return sound_read(a, size);
    uint32_t off;
    if (uint8_t* p = area(a, off)) return mem_rd(p, off, size);
    if (a < 0x05D00000u) return vdp1_fb_read(a & 0x3FFFF, size);
    if (a < 0x05D00020u) return vdp1_reg_read(a & 0x1F, size);
    off = a & 0x1FF;
    if (off == 0x04 || off == 0x08 || off == 0x0A) {   // TVSTAT, HCNT, VCNT: from the raster
        int line = (int)(g_line_abs % kLines);
        // HBLANK: the last sixth of every line (about 10.9 of NTSC's 63.6 us)
        const uint64_t line_ns = kFrameNs / kLines;
        bool hblank = sat_now() % line_ns >= line_ns - line_ns / 6;
        uint16_t tvstat = (uint16_t)((line >= display_lines() ? 8 : 0) | (hblank ? 4 : 0) | ((g_vblanks & 1) ? 2 : 0));
        set16(g_vdp2_regs, 0x04, tvstat);
        set16(g_vdp2_regs, 0x0A, (uint16_t)line);
    }
    return mem_rd(g_vdp2_regs, off, size);
}

void video_write(uint32_t a, uint32_t v, int size) {
    if (a < 0x05B01000u) { sound_write(a, v, size); return; }
    uint32_t off;
    if (uint8_t* p = area(a, off)) { mem_wr(p, off, v, size); return; }
    if (a < 0x05D00000u) { vdp1_fb_write(a & 0x3FFFF, v, size); return; }
    if (a < 0x05D00020u) { vdp1_reg_write(a & 0x1F, v, size); return; }
    mem_wr(g_vdp2_regs, a & 0x1FF, v, size);
}
