// saturnkit runtime — the disc: a .cue with its .bin files (one per track or
// one for all), sectors by FAD, the TOC, and ISO 9660 lookups for the boot.
//
// FADs are absolute: the first track's INDEX 01 is FAD 150. A file's first
// sector follows the previous file's last; a track starts at its INDEX 01,
// so a pregap stored in the file (INDEX 00) sits before it. A PREGAP line is
// a pregap the file does not hold (an older rip's one .bin keeps track 2's
// so): it takes its place on the disc and reads as silence. Each track has
// its own mode, also when several share a file. Data tracks are MODE1/2352
// (raw) or MODE1/2048 (the header is made up).
#include "saturn.h"
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <vector>

// A run of the disc's sectors: from one file (f, from its sector `sector`),
// or a PREGAP held by no file (f null).
struct Extent { uint32_t fad, count; FILE* f; uint32_t sector; int secsize; bool audio; };
struct CueTrack { int num; uint32_t fad; bool audio; };
static std::vector<Extent> g_extents;
static std::vector<CueTrack> g_tracks;
static uint32_t g_leadout;

static uint32_t msf(const std::string& s) {
    int m = 0, sec = 0, fr = 0;
    std::sscanf(s.c_str(), "%d:%d:%d", &m, &sec, &fr);
    return (uint32_t)((m * 60 + sec) * 75 + fr);
}

bool cdrom_open(const std::string& cue) {
    std::ifstream in(cue);
    if (!in) return false;
    std::string dir = cue.substr(0, cue.find_last_of("/\\") + 1), line;
    // the sheet as written: files, and each file's tracks with their indexes
    struct T { int num; bool audio; int secsize; uint32_t gap; int64_t idx0 = -1, idx1 = -1; };
    struct F { FILE* f; uint32_t size; std::vector<T> tracks; };
    std::vector<F> files;
    while (std::getline(in, line)) {
        std::istringstream ss(line);
        std::string kw;
        ss >> kw;
        if (kw == "FILE") {
            size_t q1 = line.find('"'), q2 = line.rfind('"');
            std::string name = q1 != std::string::npos && q2 > q1 ? line.substr(q1 + 1, q2 - q1 - 1) : "";
            FILE* f = std::fopen((dir + name).c_str(), "rb");
            if (!f) { std::fprintf(stderr, "cannot open %s\n", (dir + name).c_str()); return false; }
            std::fseek(f, 0, SEEK_END);
            files.push_back({f, (uint32_t)std::ftell(f), {}});
        } else if (kw == "TRACK" && !files.empty()) {
            int num = 0;
            std::string mode;
            ss >> num >> mode;
            files.back().tracks.push_back({num, mode == "AUDIO", mode == "MODE1/2048" ? 2048 : 2352, 0});
        } else if (kw == "PREGAP" && !files.empty() && !files.back().tracks.empty()) {
            std::string pos;
            ss >> pos;
            files.back().tracks.back().gap = msf(pos);
        } else if (kw == "INDEX" && !files.empty() && !files.back().tracks.empty()) {
            int idx;
            std::string pos;
            ss >> idx >> pos;
            T& t = files.back().tracks.back();
            if (idx == 0) t.idx0 = msf(pos);
            if (idx == 1) t.idx1 = msf(pos);
        }
    }
    // laid out on the disc
    uint32_t fad = 150;
    for (const F& f : files) {
        for (size_t i = 0; i < f.tracks.size(); ++i) {
            const T& t = f.tracks[i];
            if (t.idx1 < 0) return false;
            // a file's first track takes it from its first sector, a later one from its INDEX 00
            uint32_t start = i == 0 ? 0 : (uint32_t)(t.idx0 >= 0 ? t.idx0 : t.idx1);
            uint32_t end = f.size / t.secsize;
            if (i + 1 < f.tracks.size()) {
                const T& n = f.tracks[i + 1];
                end = (uint32_t)(n.idx0 >= 0 ? n.idx0 : n.idx1);
            }
            if (t.gap) {
                g_extents.push_back({fad, t.gap, nullptr, 0, t.secsize, t.audio});
                fad += t.gap;
            }
            g_tracks.push_back({t.num, fad + (uint32_t)t.idx1 - start, t.audio});
            if (end > start) {
                g_extents.push_back({fad, end - start, f.f, start, t.secsize, t.audio});
                fad += end - start;
            }
        }
    }
    if (g_extents.empty() || g_tracks.empty()) return false;
    g_leadout = fad;
    return true;
}

static const Extent* extent_of(uint32_t fad) {
    for (const Extent& e : g_extents)
        if (fad >= e.fad && fad < e.fad + e.count) return &e;
    return nullptr;
}

bool cdrom_is_audio(uint32_t fad) {
    const Extent* e = extent_of(fad);
    return e && e->audio;
}

bool cdrom_read(uint32_t fad, uint8_t* raw) {
    const Extent* e = extent_of(fad);
    if (!e) return false;
    if (!e->f) {                                // a PREGAP the file does not hold: silence
        std::memset(raw, 0, 2352);
        return true;
    }
    long pos = (long)(e->sector + (fad - e->fad)) * e->secsize;
    std::fseek(e->f, pos, SEEK_SET);
    if (e->secsize == 2352) return std::fread(raw, 1, 2352, e->f) == 2352;
    // MODE1/2048: sync, header (BCD MSF, mode 1), data; EDC/ECC left zero
    std::memset(raw, 0, 2352);
    std::memset(raw + 1, 0xFF, 10);
    auto bcd = [](uint32_t v) { return (uint8_t)((v / 10) << 4 | (v % 10)); };
    raw[12] = bcd(fad / 75 / 60); raw[13] = bcd(fad / 75 % 60); raw[14] = bcd(fad % 75); raw[15] = 1;
    return std::fread(raw + 16, 1, 2048, e->f) == 2048;
}

int cdrom_tracks(CdTrack* out, int max) {
    int n = 0;
    for (const CueTrack& t : g_tracks)
        if (n < max) out[n++] = {t.fad, t.audio ? 0x01u : 0x41u};
    return n;
}

uint32_t cdrom_leadout() { return g_leadout; }

// ---- ISO 9660 ----------------------------------------------------------------------------
static std::vector<uint8_t> read_user(uint32_t lba, uint32_t size) {
    std::vector<uint8_t> out(size);
    uint8_t raw[2352];
    for (uint32_t o = 0; o < size; o += 2048)
        if (cdrom_read(150 + lba + o / 2048, raw))
            std::memcpy(out.data() + o, raw + 16, size - o < 2048 ? size - o : 2048);
    return out;
}

struct Rec { std::string name; uint32_t lba, size; bool dir; };

static std::vector<Rec> list(uint32_t lba, uint32_t size) {
    std::vector<Rec> out;
    std::vector<uint8_t> d = read_user(lba, size);
    for (uint32_t o = 0; o < size;) {
        uint8_t len = d[o];
        if (!len) { o = (o / 2048 + 1) * 2048; continue; }
        uint32_t l = (uint32_t)d[o + 2] | d[o + 3] << 8 | d[o + 4] << 16 | (uint32_t)d[o + 5] << 24;
        uint32_t s = (uint32_t)d[o + 10] | d[o + 11] << 8 | d[o + 12] << 16 | (uint32_t)d[o + 13] << 24;
        uint8_t nlen = d[o + 32];
        std::string name((const char*)&d[o + 33], nlen);
        if (!(nlen == 1 && (name[0] == 0 || name[0] == 1))) {
            size_t semi = name.find(';');
            if (semi != std::string::npos) name.resize(semi);
            if (!name.empty() && name.back() == '.') name.pop_back();
            out.push_back({name, l, s, (d[o + 25] & 2) != 0});
        }
        o += len;
    }
    return out;
}

static bool root(uint32_t& lba, uint32_t& size) {
    std::vector<uint8_t> pvd = read_user(16, 2048);
    if (pvd[0] != 1 || std::memcmp(&pvd[1], "CD001", 5)) return false;
    const uint8_t* r = &pvd[156];
    lba = (uint32_t)r[2] | r[3] << 8 | r[4] << 16 | (uint32_t)r[5] << 24;
    size = (uint32_t)r[10] | r[11] << 8 | r[12] << 16 | (uint32_t)r[13] << 24;
    return true;
}

bool cdrom_find(const char* path, uint32_t* fad, uint32_t* size) {
    uint32_t lba, sz;
    if (!root(lba, sz)) return false;
    std::string p = path;
    size_t start = 0;
    for (;;) {
        size_t slash = p.find('/', start);
        std::string part = p.substr(start, slash == std::string::npos ? std::string::npos : slash - start);
        bool found = false;
        for (const Rec& r : list(lba, sz))
            if (r.name == part) { lba = r.lba; sz = r.size; found = true; break; }
        if (!found) return false;
        if (slash == std::string::npos) break;
        start = slash + 1;
    }
    *fad = 150 + lba;
    *size = sz;
    return true;
}

std::string cdrom_first_file() {
    uint32_t lba, sz;
    if (!root(lba, sz)) return "";
    for (const Rec& r : list(lba, sz))
        if (!r.dir) return r.name;
    return "";
}
