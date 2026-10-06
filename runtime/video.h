// saturnkit runtime — what the video chips share with each other and with
// the host window: VDP1 (vdp1.cpp), VDP2 (vdp2.cpp), the raster and the bus
// (video.cpp), the window (host.cpp).
#pragma once
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

// ---- VDP1 (vdp1.cpp) --------------------------------------------------------------------
extern uint8_t g_vdp1_vram[0x80000];                 // big-endian, as the bus sees it
void vdp1_init();
uint32_t vdp1_reg_read(uint32_t off, int size);      // 0x05D00000 + off
void vdp1_reg_write(uint32_t off, uint32_t v, int size);
uint32_t vdp1_fb_read(uint32_t off, int size);       // the draw framebuffer, 0x05C80000 + off
void vdp1_fb_write(uint32_t off, uint32_t v, int size);
void vdp1_vblank_out();                              // erase, frame change, the draw PTMR 2 asks for
void vdp1_tick(uint64_t now);                        // the end of a draw
const uint16_t* vdp1_display();                      // the framebuffer VDP2 shows: 512x256, 16 bits
uint64_t vdp1_frame_changes();
uint64_t vdp1_draws();
void vdp1_dump(FILE* f);                             // VRAM, the draw framebuffer, the registers
bool vdp1_interp_field(bool compose);                // --interp: a frame in between for this field?
void vdp1_interp_scroll(uint8_t* regs);              // ... and VDP2's scroll registers for it
void vdp1_next_draw_keys(std::vector<uint64_t> keys); // --interp: what each command of the next draw is

// ---- VDP2 (vdp2.cpp) --------------------------------------------------------------------
extern uint8_t g_vdp2_vram[0x80000], g_vdp2_cram[0x1000], g_vdp2_regs[0x200];
struct Frame {
    int w = 0, h = 0;
    std::vector<uint32_t> px;                       // 0x00RRGGBB, w*h, top row first
};
void vdp2_compose(Frame& out);                       // the picture of the field that has just ended
// Raster effects: VDP2 register writes an HBlank-IN handler made during the field, each with
// the line it was made on (it shows from that line on), and the registers as the field began.
struct RasterWrite { int line; uint16_t off; uint8_t size; uint32_t value; };
const std::vector<RasterWrite>& video_raster_writes();
const uint8_t* video_field_regs();                   // VDP2's registers at the field's line 0
int vdp2_lines();                                    // 224, 240 or 256 (TVMD VRESO)

// ---- the host (host.cpp) ------------------------------------------------------------------
bool host_open();                                    // the window (unless headless); false if it failed
void host_present(const Frame& f);                   // show it, take the events
void host_pace(uint64_t now);                        // wait until the host's clock has caught up
bool host_wants_frame();                             // a window to show it in
uint16_t host_pad();                                 // buttons held, smpc.cpp's bits
void write_png(const std::string& path, const Frame& f);
