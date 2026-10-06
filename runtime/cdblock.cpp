// saturnkit runtime — the CD block (the "CS2") at its registers.
//
// The host writes a command into CR1-CR4 (CR4 last) and reads the answer
// from the same registers; HIRQ holds the flags (writing clears the bits
// written as 0). Commands complete at once. The drive seeks (timed as
// Mednafen's drive does it, ST_SEEK meanwhile), then delivers sectors of the
// disc (cdrom.cpp) at double speed, 150 a second: each goes from the CD
// device connection through the filters (FAD range, subheader) into a
// partition of the 200-sector buffer, where the host gets it through the
// data port (0x25818000). CD-DA plays at 75 sectors a second; each sector's
// 588 stereo samples go to the SCSP's external input (cd_audio_sample, taken
// by sound.cpp once a sample). The file-system commands (0x70-0x75) work on
// the ISO 9660 directory the way the CD block's own firmware does.
//
// Command set and answer layouts: Sega's CD block documentation as used by
// SBL's CDC library; where the documents leave a value open, the choice is
// noted here.
#include "saturn.h"
#include <cstring>
#include <deque>
#include <string>
#include <vector>

enum : uint16_t {
    CMOK = 0x0001, DRDY = 0x0002, CSCT = 0x0004, BFUL = 0x0008, PEND = 0x0010, DCHG = 0x0020,
    ESEL = 0x0040, EHST = 0x0080, ECPY = 0x0100, EFLS = 0x0200, SCDQ = 0x0400,
};
enum : uint8_t { ST_BUSY = 0, ST_PAUSE = 1, ST_STANDBY = 2, ST_PLAY = 3, ST_SEEK = 4, ST_PERI = 0x20, ST_TRNS = 0x40 };

static const int kBlocks = 200, kParts = 24, kFilters = 24;
struct Block { uint8_t raw[2352]; uint32_t fad; uint8_t fn, cn, sm, ci; };
struct Filter {
    uint32_t fad, range;
    uint8_t mode, chan, smmask, cimask, fid, smval, cival, tru, fals;
};

static Block g_block[kBlocks];
static std::vector<int> g_free;
static std::deque<int> g_part[kParts];
static Filter g_filter[kFilters];
static uint8_t g_cddev = 0xFF, g_lastbuf = 0xFF;

static uint16_t g_hirq, g_hmask, g_cr[4], g_out[4];
static bool g_cmd_busy;
static uint64_t g_next_peri;

// the drive
static uint8_t g_status = ST_PAUSE;
static uint32_t g_fad = 150, g_play_end, g_play_start;
static int g_repeat, g_repeat_left, g_repeats_done;
static uint32_t g_cmd_start, g_cmd_end;         // the last Play's positions, as the command gave them
static bool g_audio_play;
static uint64_t g_play_t0, g_play_done;         // time the play began, sectors read since
static uint64_t g_seek_end;                     // ST_SEEK: when the pickup is there and the play begins
static int g_getlen = 2048;
static std::deque<int16_t> g_cdda;              // the audio played, not yet taken by the SCSP (L, R)

// the host transfer
enum Xfer { X_NONE, X_WORDS, X_SECTORS };
static Xfer g_xfer;
static std::vector<uint16_t> g_words;
static std::vector<int> g_xblocks;
static size_t g_xpos;                           // in words
static bool g_xdelete;
static int g_xpart;
static uint32_t g_xcount;                       // words read, for End Data Transfer
static uint32_t g_calcsize;

// the file system
struct FsFile { uint32_t fad, size; uint8_t attr, fn; std::string name; };
static std::vector<FsFile> g_dir;               // 0 '.', 1 '..', then the records

// ---- helpers -------------------------------------------------------------------------------
static void reset_filter(int i) { g_filter[i] = Filter{0, 0xFFFFFFFFu, 0, 0, 0, 0, 0, 0, 0, (uint8_t)i, 0xFF}; }

static void clear_part(int p) {
    for (int b : g_part[p]) g_free.push_back(b);
    g_part[p].clear();
}

static void reset_all() {
    g_free.clear();
    for (int i = kBlocks - 1; i >= 0; --i) g_free.push_back(i);
    for (auto& p : g_part) p.clear();
    for (int i = 0; i < kFilters; ++i) reset_filter(i);
    g_cddev = 0xFF;
    g_lastbuf = 0xFF;
    g_xfer = X_NONE;
}

static int track_of(uint32_t fad, uint32_t* ctrladr) {
    CdTrack t[99];
    int n = cdrom_tracks(t, 99), k = 0;
    for (int i = 0; i < n; ++i)
        if (fad >= t[i].fad) k = i;
    if (ctrladr) *ctrladr = n ? t[k].ctrladr : 0x41;
    return k + 1;
}

static void report() {
    uint32_t ctrladr;
    int track = track_of(g_fad, &ctrladr);
    uint8_t st = g_status | (g_xfer != X_NONE ? ST_TRNS : 0);
    g_out[0] = (uint16_t)(st << 8 | (g_repeats_done & 0xF));
    g_out[1] = (uint16_t)(ctrladr << 8 | track);
    g_out[2] = (uint16_t)(1 << 8 | (g_fad >> 16 & 0xFF));
    g_out[3] = (uint16_t)(g_fad & 0xFFFF);
}

static void answer(uint16_t a, uint16_t b, uint16_t c, uint16_t d) {
    g_out[0] = a; g_out[1] = b; g_out[2] = c; g_out[3] = d;
}

static uint16_t status_hi() { return (uint16_t)((g_status | (g_xfer != X_NONE ? ST_TRNS : 0)) << 8); }

static uint32_t track_fad(int track, bool end) {
    CdTrack t[99];
    int n = cdrom_tracks(t, 99);
    if (track < 1) track = 1;
    if (track > n) track = n;
    if (!end) return t[track - 1].fad;
    return track < n ? t[track].fad : cdrom_leadout();
}

// sectors of partition p, from offset o, count n (0xFFFF: the last one / all the rest)
static std::vector<int> pick(int p, uint32_t o, uint32_t n) {
    std::vector<int> out;
    if (p >= kParts) return out;
    auto& q = g_part[p];
    if (o == 0xFFFF) o = q.empty() ? 0 : (uint32_t)q.size() - 1;
    if (n == 0xFFFF) n = o < q.size() ? (uint32_t)q.size() - o : 0;
    for (uint32_t i = o; i < o + n && i < q.size(); ++i) out.push_back(q[i]);
    return out;
}

static void remove_blocks(int p, const std::vector<int>& bl) {
    auto& q = g_part[p];
    for (int b : bl)
        for (auto it = q.begin(); it != q.end(); ++it)
            if (*it == b) { q.erase(it); g_free.push_back(b); break; }
}

static int data_offset(const Block& b) {
    int mode = b.raw[15];
    switch (g_getlen) {
    case 2352: return 0;
    case 2340: return 12;
    case 2336: return 16;
    }
    return mode == 2 ? 24 : 16;
}

// ---- the drive ------------------------------------------------------------------------------
static bool filter_pass(const Filter& f, const Block& b) {
    bool ok = true;
    if (f.mode & 0x40) ok &= b.fad >= f.fad && b.fad - f.fad < f.range;
    bool sub = true;
    if (f.mode & 0x01) sub &= b.fn == f.fid;
    if (f.mode & 0x02) sub &= b.cn == f.chan;
    if (f.mode & 0x04) sub &= (b.sm & f.smmask) == f.smval;
    if (f.mode & 0x08) sub &= (b.ci & f.cimask) == f.cival;
    if (f.mode & 0x10) sub = !sub;
    return ok && sub;
}

static bool deliver_sector() {
    if (g_free.empty()) { g_hirq |= BFUL; return false; }
    int bi = g_free.back();
    Block& b = g_block[bi];
    if (!cdrom_read(g_fad, b.raw)) return false;
    b.fad = g_fad;
    bool m2 = b.raw[15] == 2;
    b.fn = m2 ? b.raw[16] : 0; b.cn = m2 ? b.raw[17] : 0; b.sm = m2 ? b.raw[18] : 0; b.ci = m2 ? b.raw[19] : 0;
    int f = g_cddev;
    for (int hop = 0; f != 0xFF && hop < kFilters; ++hop) {
        if (f >= kFilters) { f = 0xFF; break; }
        if (filter_pass(g_filter[f], b)) {
            int p = g_filter[f].tru;
            if (p < kParts) {
                g_free.pop_back();
                g_part[p].push_back(bi);
                g_lastbuf = (uint8_t)p;
            }
            break;
        }
        f = g_filter[f].fals;
    }
    g_hirq |= CSCT;
    return true;
}

static void play_ended() {
    if (g_repeat_left > 0 || g_repeat == 0xF) {
        if (g_repeat != 0xF) --g_repeat_left;
        if (g_repeats_done < 0xE) ++g_repeats_done;
        g_fad = g_play_start;
        g_play_t0 = sat_now();
        g_play_done = 0;
        return;
    }
    g_status = ST_PAUSE;
    g_hirq |= PEND;
    sat_trace("CD: play ended at FAD %u", g_fad);
}

void cd_tick() {
    uint64_t now = sat_now();
    if (g_status == ST_SEEK && now >= g_seek_end) {
        g_status = ST_PLAY;
        g_play_t0 = g_seek_end;
        g_play_done = 0;
    }
    if (g_status == ST_PLAY) {
        uint64_t rate = g_audio_play ? 75 : 150;
        uint64_t due = (now - g_play_t0) * rate / 1000000000ull;
        while (g_play_done < due && g_status == ST_PLAY) {
            if (g_fad >= g_play_end || g_fad < g_play_start) { play_ended(); break; }
            if (!g_audio_play && !deliver_sector()) {       // the buffer is full: wait
                g_play_t0 = now - g_play_done * 1000000000ull / rate;
                break;
            }
            uint8_t raw[2352];
            if (g_audio_play && cdrom_read(g_fad, raw))
                for (int i = 0; i < 2352; i += 2) g_cdda.push_back((int16_t)(raw[i] | raw[i + 1] << 8));
            ++g_fad;
            ++g_play_done;
        }
    }
    if (now >= g_next_peri) {                   // the periodic report
        g_next_peri = now + 16666667;
        g_hirq |= SCDQ;
        if (!g_cmd_busy) {
            report();
            g_out[0] |= ST_PERI << 8;
        }
    }
}

// One stereo sample of the CD audio at 44 100 Hz, or silence
void cd_audio_sample(int16_t lr[2]) {
    if (g_cdda.size() < 2) { lr[0] = lr[1] = 0; return; }
    lr[0] = g_cdda.front(); g_cdda.pop_front();
    lr[1] = g_cdda.front(); g_cdda.pop_front();
    while (g_cdda.size() > 2352 * 4) g_cdda.pop_front();   // more than 4 sectors behind: catch up
}

// ---- the file system ---------------------------------------------------------------------------
static void load_dir(uint32_t fad, uint32_t size) {
    g_dir.clear();
    std::vector<uint8_t> d(size);
    uint8_t raw[2352];
    for (uint32_t o = 0; o < size; o += 2048)
        if (cdrom_read(fad + o / 2048, raw)) std::memcpy(d.data() + o, raw + 16, size - o < 2048 ? size - o : 2048);
    for (uint32_t o = 0; o < size;) {
        uint8_t len = d[o];
        if (!len) { o = (o / 2048 + 1) * 2048; continue; }
        uint32_t lba = (uint32_t)d[o + 2] | d[o + 3] << 8 | d[o + 4] << 16 | (uint32_t)d[o + 5] << 24;
        uint32_t sz = (uint32_t)d[o + 10] | d[o + 11] << 8 | d[o + 12] << 16 | (uint32_t)d[o + 13] << 24;
        std::string name((const char*)&d[o + 33], d[o + 32]);
        g_dir.push_back({150 + lba, sz, d[o + 25], 0, name});
        o += len;
    }
}

static void root_dir() {
    uint8_t raw[2352];
    cdrom_read(150 + 16, raw);
    const uint8_t* r = raw + 16 + 156;
    uint32_t lba = (uint32_t)r[2] | r[3] << 8 | r[4] << 16 | (uint32_t)r[5] << 24;
    uint32_t size = (uint32_t)r[10] | r[11] << 8 | r[12] << 16 | (uint32_t)r[13] << 24;
    load_dir(150 + lba, size);
}

static void put_info(const FsFile& f) {
    g_words.push_back((uint16_t)(f.fad >> 16)); g_words.push_back((uint16_t)f.fad);
    g_words.push_back((uint16_t)(f.size >> 16)); g_words.push_back((uint16_t)f.size);
    g_words.push_back(0);                                   // unit size, gap size
    g_words.push_back((uint16_t)(f.fn << 8 | (f.attr & 2 ? 2 : 0)));   // file number, attribute
}

// ---- commands -------------------------------------------------------------------------------
// Play Disc. A position 0xFFFFFF is the last Play's; an end given in sectors counts from the
// start given. The mode's low nibble is the repeat count (0xF: for ever), taken only when bits
// 4-6 are clear (0xFF: no change); bit 7 leaves the pickup where it is, so a stream whose end
// is pushed further on keeps reading on (GFS_SGL's streams), and a position outside the new
// range ends the play. As Mednafen's CD block does it.
static void start_play(uint32_t start, uint32_t end, int mode) {
    if (start == 0xFFFFFF) start = g_cmd_start;
    if (end == 0xFFFFFF) end = g_cmd_end;
    else if ((start & 0x800000) && (end & 0x800000)) end = 0x800000 | ((start + end) & 0x7FFFFF);
    g_cmd_start = start;
    g_cmd_end = end;
    uint32_t from = start & 0x800000 ? start & 0x7FFFFF : start ? track_fad(start >> 8, false) : g_fad;
    if (end & 0x800000) g_play_end = end & 0x7FFFFF;
    else if (end) g_play_end = track_fad(end >> 8, true);
    else g_play_end = cdrom_leadout();
    if (!(mode & 0x70)) g_repeat = mode & 0xF;
    g_repeat_left = g_repeat;
    g_repeats_done = 0;
    g_play_start = from;
    bool reading_on = (mode & 0x80) && g_status == ST_PLAY;
    int64_t delta = (mode & 0x80) ? 0 : (int64_t)from - (int64_t)g_fad;
    if (!(mode & 0x80)) g_fad = from;
    g_audio_play = cdrom_is_audio(g_fad);
    if (reading_on) {
        g_play_t0 = sat_now();
        g_play_done = 0;
    } else {
        // the seek, Mednafen's drive timing in its clock of 44 100 x 256 Hz: the start (255 500),
        // 12 sectors' time, 26 a sector forward or 28 back, a sector's time more back or from 150
        // sectors on, a sector's time to read the subcode; the first sector a sector's time later
        const uint64_t sector = 44100 * 256 / 150;
        uint64_t clocks = 255500 + 12 * sector + (uint64_t)(delta < 0 ? -delta * 28 : delta * 26)
                        + (delta < 0 || delta >= 150 ? sector : 0) + sector;
        g_status = ST_SEEK;
        g_seek_end = sat_now() + clocks * 1000000000ull / (44100 * 256);
    }
    if (g_audio_play)
        sat_note("CD-DA: play track %d, FAD %u-%u, repeat %d", track_of(g_fad, nullptr), g_fad,
                 g_play_end, g_repeat);
    else
        sat_trace("CD: read FAD %u-%u into filter %d", g_fad, g_play_end, g_cddev);
}

static void command() {
    uint16_t cr1 = g_cr[0], cr2 = g_cr[1], cr3 = g_cr[2], cr4 = g_cr[3];
    uint8_t cmd = cr1 >> 8;
    int f = cr3 >> 8;
    switch (cmd) {
    case 0x00:                                  // Get Status
        report();
        g_hirq |= CMOK;
        break;
    case 0x01:                                  // Get Hardware Info
        answer(status_hi(), 0x0201, 0x0000, 0x0400);
        g_hirq |= CMOK;
        break;
    case 0x02: {                                // Get TOC
        CdTrack t[99];
        int n = cdrom_tracks(t, 99);
        g_words.clear();
        auto put = [&](uint32_t v) { g_words.push_back((uint16_t)(v >> 16)); g_words.push_back((uint16_t)v); };
        for (int i = 0; i < 99; ++i) put(i < n ? t[i].ctrladr << 24 | t[i].fad : 0xFFFFFFFFu);
        put(t[0].ctrladr << 24 | 1u << 16);
        put(t[n - 1].ctrladr << 24 | (uint32_t)n << 16);
        put(t[n - 1].ctrladr << 24 | cdrom_leadout());
        g_xfer = X_WORDS; g_xpos = 0; g_xcount = 0;
        answer(status_hi() | (uint16_t)(ST_TRNS << 8), 0xCC, 0, 0);
        g_hirq |= CMOK | DRDY;
        break;
    }
    case 0x03:                                  // Get Session Info
        if ((cr1 & 0xFF) == 0) answer(status_hi(), 0, (uint16_t)(0x0100 | (cdrom_leadout() >> 16 & 0xFF)), (uint16_t)cdrom_leadout());
        else if ((cr1 & 0xFF) == 1) answer(status_hi(), 0, 0x0100, 0);
        else answer(status_hi(), 0, 0xFFFF, 0xFFFF);
        g_hirq |= CMOK;
        break;
    case 0x04:                                  // Initialize CD System
        if (cr1 & 1) { reset_all(); g_status = ST_PAUSE; }
        if (cr1 & 0x10) sat_trace("CD: 1x speed asked for (kept at 2x)");
        report();
        g_hirq |= CMOK | ESEL;
        break;
    case 0x06:                                  // End Data Transfer
        if (g_xfer == X_NONE && !g_xcount) answer(status_hi() | 0xFF, 0xFFFF, 0, 0);
        else answer((uint16_t)(status_hi() | (g_xcount >> 16 & 0xFF)), (uint16_t)g_xcount, 0, 0);
        if (g_xfer == X_SECTORS && g_xdelete) { remove_blocks(g_xpart, g_xblocks); g_hirq |= EHST; }
        else if (g_xfer == X_SECTORS) g_hirq |= EHST;
        g_xfer = X_NONE; g_xblocks.clear(); g_xcount = 0;
        g_out[0] = (uint16_t)(g_out[0] & ~(ST_TRNS << 8));
        g_hirq |= CMOK;
        break;
    case 0x10:                                  // Play Disc
        start_play((uint32_t)(cr1 & 0xFF) << 16 | cr2, (uint32_t)(cr3 & 0xFF) << 16 | cr4, cr3 >> 8);
        report();
        g_hirq |= CMOK;
        break;
    case 0x11: {                                // Seek Disc
        uint32_t pos = (uint32_t)(cr1 & 0xFF) << 16 | cr2;
        if (pos == 0) g_status = ST_STANDBY;
        else {                                  // 0xFFFFFF: pause where it is
            if (pos == 0xFFFFFF) {}
            else if (pos & 0x800000) g_fad = pos & 0x7FFFFF;
            else g_fad = track_fad(pos >> 8, false);
            g_status = ST_PAUSE;
        }
        report();
        g_hirq |= CMOK;
        break;
    }
    case 0x20: {                                // Get Subcode: Q
        if ((cr1 & 0xFF) != 0) sat_fatal("CD: Get Subcode R-W");
        uint32_t ctrladr;
        int track = track_of(g_fad, &ctrladr);
        uint32_t rel = g_fad - track_fad(track, false), abs = g_fad;
        auto bcd = [](uint32_t v) { return (uint16_t)((v / 10) << 4 | (v % 10)); };
        uint8_t q[10] = {(uint8_t)ctrladr, (uint8_t)bcd(track), 1, (uint8_t)bcd(rel / 75 / 60), (uint8_t)bcd(rel / 75 % 60),
                         (uint8_t)bcd(rel % 75), 0, (uint8_t)bcd(abs / 75 / 60), (uint8_t)bcd(abs / 75 % 60), (uint8_t)bcd(abs % 75)};
        g_words.clear();
        for (int i = 0; i < 10; i += 2) g_words.push_back((uint16_t)(q[i] << 8 | q[i + 1]));
        g_xfer = X_WORDS; g_xpos = 0; g_xcount = 0;
        answer(status_hi(), 5, 0, 0);
        g_hirq |= CMOK | DRDY;
        break;
    }
    case 0x30: g_cddev = (uint8_t)f; answer(status_hi(), 0, 0, 0); g_hirq |= CMOK | ESEL; break;
    case 0x31: answer(status_hi(), 0, (uint16_t)(g_cddev << 8), 0); g_hirq |= CMOK; break;
    case 0x32: answer(status_hi(), 0, (uint16_t)(g_lastbuf << 8), 0); g_hirq |= CMOK; break;
    case 0x40:                                  // Set Filter Range
        if (f < kFilters) { g_filter[f].fad = (uint32_t)(cr1 & 0xFF) << 16 | cr2; g_filter[f].range = (uint32_t)(cr3 & 0xFF) << 16 | cr4; }
        answer(status_hi(), 0, 0, 0); g_hirq |= CMOK | ESEL;
        break;
    case 0x41: {
        Filter& F = g_filter[f % kFilters];
        answer((uint16_t)(status_hi() | (F.fad >> 16 & 0xFF)), (uint16_t)F.fad, (uint16_t)(f << 8 | (F.range >> 16 & 0xFF)), (uint16_t)F.range);
        g_hirq |= CMOK | ESEL;
        break;
    }
    case 0x42:                                  // Set Filter Subheader Conditions
        if (f < kFilters) {
            Filter& F = g_filter[f];
            F.chan = cr1 & 0xFF; F.smmask = cr2 >> 8; F.cimask = cr2 & 0xFF; F.fid = cr3 & 0xFF;
            F.smval = cr4 >> 8; F.cival = cr4 & 0xFF;
        }
        answer(status_hi(), 0, 0, 0); g_hirq |= CMOK | ESEL;
        break;
    case 0x43: {
        Filter& F = g_filter[f % kFilters];
        answer((uint16_t)(status_hi() | F.chan), (uint16_t)(F.smmask << 8 | F.cimask), (uint16_t)(f << 8 | F.fid), (uint16_t)(F.smval << 8 | F.cival));
        g_hirq |= CMOK | ESEL;
        break;
    }
    case 0x44:                                  // Set Filter Mode
        if (f < kFilters) {
            if (cr1 & 0x80) { uint8_t t = g_filter[f].tru, fl = g_filter[f].fals; reset_filter(f); g_filter[f].tru = t; g_filter[f].fals = fl; }
            else g_filter[f].mode = cr1 & 0xFF;
        }
        answer(status_hi(), 0, 0, 0); g_hirq |= CMOK | ESEL;
        break;
    case 0x45: answer((uint16_t)(status_hi() | g_filter[f % kFilters].mode), 0, (uint16_t)(f << 8), 0); g_hirq |= CMOK | ESEL; break;
    case 0x46:                                  // Set Filter Connection
        if (f < kFilters) {
            if (cr1 & 1) g_filter[f].tru = cr2 >> 8;
            if (cr1 & 2) g_filter[f].fals = cr2 & 0xFF;
        }
        answer(status_hi(), 0, 0, 0); g_hirq |= CMOK | ESEL;
        break;
    case 0x47: answer(status_hi(), (uint16_t)(g_filter[f % kFilters].tru << 8 | g_filter[f % kFilters].fals), (uint16_t)(f << 8), 0); g_hirq |= CMOK | ESEL; break;
    case 0x48: {                                // Reset Selector
        uint8_t fl = cr1 & 0xFF;
        if (!fl) { if (f < kParts) clear_part(f); }
        else {
            if (fl & 0x04) for (int p = 0; p < kParts; ++p) clear_part(p);
            if (fl & 0x10) for (int i = 0; i < kFilters; ++i) { uint8_t t = g_filter[i].tru, x = g_filter[i].fals; reset_filter(i); g_filter[i].tru = t; g_filter[i].fals = x; }
            if (fl & 0x20) g_cddev = 0xFF;
            if (fl & 0x40) for (int i = 0; i < kFilters; ++i) g_filter[i].tru = (uint8_t)i;
            if (fl & 0x80) for (int i = 0; i < kFilters; ++i) g_filter[i].fals = 0xFF;
        }
        answer(status_hi(), 0, 0, 0); g_hirq |= CMOK | ESEL;
        break;
    }
    case 0x50: answer(status_hi(), (uint16_t)g_free.size(), 0x1800, kBlocks); g_hirq |= CMOK; break;
    case 0x51: answer(status_hi(), 0, 0, (uint16_t)(f < kParts ? g_part[f].size() : 0)); g_hirq |= CMOK; break;
    case 0x52: {                                // Calculate Actual Size (words)
        g_calcsize = 0;
        for (int b : pick(f, cr2, cr4)) { (void)b; g_calcsize += g_getlen / 2; }
        answer(status_hi(), 0, 0, 0); g_hirq |= CMOK | ESEL;
        break;
    }
    case 0x53: answer((uint16_t)(status_hi() | (g_calcsize >> 16 & 0xFF)), (uint16_t)g_calcsize, 0, 0); g_hirq |= CMOK; break;
    case 0x54: {                                // Get Sector Info
        auto bl = pick(f, cr2 & 0xFF, 1);
        if (bl.empty()) { answer(status_hi() | 0x8000, 0, 0, 0); g_hirq |= CMOK; break; }   // WAIT: rejected
        Block& b = g_block[bl[0]];
        answer((uint16_t)(status_hi() | (b.fad >> 16 & 0xFF)), (uint16_t)b.fad, (uint16_t)(b.fn << 8 | b.cn), (uint16_t)(b.sm << 8 | b.ci));
        g_hirq |= CMOK | ESEL;
        break;
    }
    case 0x60: {                                // Set Sector Length
        static const int len[4] = {2048, 2336, 2340, 2352};
        if ((cr1 & 0xFF) < 4) g_getlen = len[cr1 & 0xFF];
        answer(status_hi(), 0, 0, 0); g_hirq |= CMOK | ESEL;
        break;
    }
    case 0x61: case 0x63: {                     // Get (Then Delete) Sector Data
        g_xblocks = pick(f, cr2, cr4);
        g_xfer = X_SECTORS; g_xpos = 0; g_xcount = 0; g_xdelete = cmd == 0x63; g_xpart = f;
        answer(status_hi(), 0, 0, 0);
        g_hirq |= CMOK | DRDY;
        break;
    }
    case 0x62:                                  // Delete Sector Data
        remove_blocks(f, pick(f, cr2, cr4));
        answer(status_hi(), 0, 0, 0); g_hirq |= CMOK | EHST;
        break;
    case 0x67: answer(status_hi(), 0, 0, 0); g_hirq |= CMOK; break;   // Get Copy Error: none
    case 0x70: case 0x71: {                     // Change / Read Directory
        uint32_t fid = (uint32_t)(cr3 & 0xFF) << 16 | cr4;
        if (fid == 0xFFFFFF || g_dir.empty()) root_dir();
        else if (fid < g_dir.size()) load_dir(g_dir[fid].fad, g_dir[fid].size);
        report(); g_hirq |= CMOK | EFLS;
        break;
    }
    case 0x72:                                  // Get File System Scope
        if (g_dir.empty()) root_dir();
        answer(status_hi(), (uint16_t)(g_dir.size() - 2), 0x0100, 0x0002);
        g_hirq |= CMOK | EFLS;
        break;
    case 0x73: {                                // Get File Info
        if (g_dir.empty()) root_dir();
        uint32_t fid = (uint32_t)(cr3 & 0xFF) << 16 | cr4;
        g_words.clear();
        if (fid == 0xFFFFFF) for (size_t i = 2; i < g_dir.size() && i < 256; ++i) put_info(g_dir[i]);
        else if (fid < g_dir.size()) put_info(g_dir[fid]);
        g_xfer = X_WORDS; g_xpos = 0; g_xcount = 0;
        answer(status_hi(), (uint16_t)g_words.size(), 0, 0);
        g_hirq |= CMOK | DRDY;
        break;
    }
    case 0x74: {                                // Read File
        uint32_t off = (uint32_t)(cr1 & 0xFF) << 16 | cr2, fid = (uint32_t)(cr3 & 0xFF) << 16 | cr4;
        if (g_dir.empty()) root_dir();
        if (fid >= g_dir.size()) sat_fatal("CD: Read File %u", fid);
        const FsFile& F = g_dir[fid];
        g_cddev = (uint8_t)f;
        start_play(0x800000 | (F.fad + off), 0x800000 | ((F.size + 2047) / 2048 - off), 0);
        report(); g_hirq |= CMOK;
        break;
    }
    case 0x75: g_status = ST_PAUSE; report(); g_hirq |= CMOK | EFLS; break;   // Abort File
    case 0xE0: report(); g_hirq |= CMOK | EFLS | CSCT; break;                 // Authenticate Device
    case 0xE1: answer(status_hi(), 4, 0, 0); g_hirq |= CMOK; break;                 // a Saturn disc
    default:
        sat_fatal("CD block command %02X (CR %04X %04X %04X %04X)", cmd, cr1, cr2, cr3, cr4);
    }
    sat_trace("CD cmd %02X (%04X %04X %04X %04X) -> %04X %04X %04X %04X, HIRQ %04X", cmd, cr1, cr2, cr3, cr4,
              g_out[0], g_out[1], g_out[2], g_out[3], g_hirq);
}

// ---- the data port -----------------------------------------------------------------------------
static uint16_t next_word() {
    ++g_xcount;
    if (g_xfer == X_WORDS) return g_xpos < g_words.size() ? g_words[g_xpos++] : 0;
    if (g_xfer == X_SECTORS) {
        size_t per = g_getlen / 2, i = g_xpos / per, k = g_xpos % per;
        ++g_xpos;
        if (i >= g_xblocks.size()) return 0;
        const Block& b = g_block[g_xblocks[i]];
        const uint8_t* p = b.raw + data_offset(b) + k * 2;
        return (uint16_t)(p[0] << 8 | p[1]);
    }
    return 0;
}

void cd_init() {
    reset_all();
    g_cddev = 0;                                // as the BIOS leaves it after reading the boot files
    g_status = ST_PAUSE;
    g_fad = 150;
    g_hirq = CMOK | CSCT | PEND | ESEL | EHST | ECPY | EFLS;
    report();
}

uint32_t cd_read(uint32_t off, int size) {
    if (off == 0x18000 || off == 0x98000) {     // the data port
        if (size == 4) { uint32_t hi = next_word(); return hi << 16 | next_word(); }
        if (size == 2) return next_word();
        sat_fatal("byte read of the CD data port");
    }
    switch (off & ~3u) {
    case 0x90008: return g_hirq;
    case 0x9000C: return g_hmask;
    case 0x90018: return g_out[0];
    case 0x9001C: return g_out[1];
    case 0x90020: return g_out[2];
    case 0x90024: g_cmd_busy = false; return g_out[3];
    }
    sat_fatal("CD block read %05X (%d bytes)", off, size);
}

void cd_write(uint32_t off, uint32_t v, int size) {
    (void)size;
    switch (off & ~3u) {
    case 0x90008: g_hirq &= (uint16_t)v; return;
    case 0x9000C: g_hmask = (uint16_t)v; return;
    case 0x90018: g_cr[0] = (uint16_t)v; g_cmd_busy = true; return;
    case 0x9001C: g_cr[1] = (uint16_t)v; return;
    case 0x90020: g_cr[2] = (uint16_t)v; return;
    case 0x90024: g_cr[3] = (uint16_t)v; command(); return;
    }
    sat_fatal("CD block write %05X = %X (%d bytes)", off, v, size);
}
