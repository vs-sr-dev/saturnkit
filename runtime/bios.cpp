// saturnkit runtime — the BIOS, high level: the boot and the services.
//
// The boot does what the BIOS does before it jumps to a game: IP.BIN to
// 0x06002000, the 1st read file (the first file record of the root
// directory) to the 1st read address, the vector tables and the service
// pointers in the work area, VBR at 0x06000000, the stack from IP.BIN.
// The security and area code in IP.BIN is not run.
//
// Every pointer slot of the two vector tables (master 0x06000000, slave
// 0x06000400, 256 each) holds a "BIOS ROM" address of its own,
// 0x00001000 + slot*4 (slave 0x00001400 + slot*4): calling it lands here
// (sh2_call_unknown -> bios_call), and the slot says which service it is.
// So the service pointers in 0x06000200-0x060003FF (SBL's SYS_* macros)
// work, and so does a pointer the game saves and puts back. Vectors 0x40-0x5F
// point at the dispatcher for the SCU interrupts: it calls the handler
// SYS_SETUINT registered (the BIOS's default handler returns at once).
//
// The backup memory (BUP, through 0x06000354/0x06000358) is kept in a file,
// out/backup.bin; see the BUP section for what is and is not implemented.
#include "saturn.h"
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <vector>

static const uint32_t kSlotBase = 0x00001000u;       // master table slots
static const uint32_t kSlaveSlotBase = 0x00001400u;  // slave table slots
static const uint32_t kDefaultHandler = 0x00002000u; // + vec*4: SYS_SETUINT's default, returns
static const uint32_t kBupFuncs = 0x00003000u;       // + k*4: the BUP library's functions
static const uint32_t kBupTable = 0x06000A00u;       // where 0x06000354 points
static const uint32_t kRet = 0xFFFFFFE8u;            // the dispatcher's return address

static uint32_t g_first_read = 0x06004000u;
static bool g_pal;                                   // the disc is for Europe (PAL) alone
static uint32_t g_uint[0x80];                        // SYS_SETUINT's handlers, vectors 0x00-0x7F
static uint32_t g_uipr[0x20];                        // SYS_CHGUIPR's table: the SCU mask while 0x40+i runs
static uint8_t g_sem[0x100];

uint32_t bios_first_read() { return g_first_read; }
bool bios_pal() { return g_pal; }

// ---- the boot ------------------------------------------------------------------------
static bool load_file(const char* path, uint32_t addr, uint32_t* size_out) {
    uint32_t fad, size;
    if (!cdrom_find(path, &fad, &size)) return false;
    uint8_t raw[2352];
    for (uint32_t o = 0; o < size; o += 2048, ++fad) {
        if (!cdrom_read(fad, raw)) return false;
        uint32_t n = size - o < 2048 ? size - o : 2048;
        if (!sh2_mem_write(addr + o, raw + 16, n)) return false;
    }
    if (size_out) *size_out = size;
    return true;
}

void bios_boot() {
    // IP.BIN: the first 16 sectors of the data track
    uint8_t ip[0x8000], raw[2352];
    for (int i = 0; i < 16; ++i) {
        if (!cdrom_read(150 + i, raw)) sat_fatal("cannot read IP.BIN");
        std::memcpy(ip + i * 2048, raw + 16, 2048);
    }
    if (std::memcmp(ip, "SEGA SEGASATURN ", 16)) sat_fatal("IP.BIN: not a Saturn disc");
    auto be = [&](int o) { return (uint32_t)ip[o] << 24 | ip[o + 1] << 16 | ip[o + 2] << 8 | ip[o + 3]; };
    uint32_t ip_size = be(0xE0), mstack = be(0xE8), sstack = be(0xEC);
    g_first_read = be(0xF0);
    std::string areas(reinterpret_cast<const char*>(ip + 0x40), 10);   // area symbols: J T U B K A E L
    g_pal = areas.find('E') != std::string::npos && areas.find_first_of("JTUBKAL") == std::string::npos;
    smpc_set_area(areas[0]);
    sh2_mem_write(0x06002000u, ip, ip_size && ip_size <= sizeof ip ? ip_size : sizeof ip);

    std::string first = cdrom_first_file();
    uint32_t size = 0;
    if (first.empty() || !load_file(first.c_str(), g_first_read, &size))
        sat_fatal("cannot load the 1st read file");
    sat_note("IP.BIN: %.10s %.6s, areas %.10s; 1st read file %s, %u bytes at %08X", ip + 0x20, ip + 0x2A, ip + 0x40,
             first.c_str(), size, g_first_read);

    // the vector tables and the service pointers
    for (uint32_t v = 0; v < 256; ++v) {
        st32(0x06000000u + v * 4, kSlotBase + v * 4);
        st32(0x06000400u + v * 4, kSlaveSlotBase + v * 4);
    }
    for (uint32_t v = 0; v < 0x80; ++v) g_uint[v] = kDefaultHandler + v * 4;
    st32(0x06000324u, 0);                       // SYS_GETSYSCK: 320 dots, 26.8 MHz
    st32(0x06000348u, 0xFFFFFFFFu);             // SYS_GETSCUIM: all masked
    scu_set_mask(0xFFFFFFFFu);
    st32(0x06000354u, kBupTable);
    for (uint32_t k = 0; k < 16; ++k) st32(kBupTable + k * 4, kBupFuncs + k * 4);

    SH2Context& c = g_master;
    c.vbr = 0x06000000u;
    // The 1st read is entered with the CPU's interrupts open: on the Saturn the
    // IP.BIN's initial program (run by the BIOS, not here) installs VBlank
    // handlers, waits on a counter the VBlank-in handler counts (level 15, so
    // the mask is below it), masks VBlank at the SCU again and calls the 1st
    // read with SR as it was. The SCU's mask stays all set, so nothing is
    // taken until the game unmasks a source.
    c.imask = 0;
    c.r[15] = mstack ? mstack : 0x06002000u;
    (void)sstack;
}

// ---- the services ---------------------------------------------------------------------
static const char* slot_name(uint32_t p) {
    switch (p) {
    case 0x0600026C: return "0x0600026C (return to the system)";
    case 0x06000300: return "SYS_SETUINT";
    case 0x06000304: return "SYS_GETUINT";
    case 0x06000310: return "SYS_SETSINT";
    case 0x06000314: return "SYS_GETSINT";
    case 0x06000320: return "SYS_CHGSYSCK";
    case 0x06000330: return "SYS_TASSEM";
    case 0x06000334: return "SYS_CLRSEM";
    case 0x06000340: return "SYS_SETSCUIM";
    case 0x06000344: return "SYS_CHGSCUIM";
    case 0x06000358: return "BUP_Init";
    case 0x06000280: return "SYS_CHGUIPR";
    }
    return nullptr;
}

static void set_mask(uint32_t m) {
    st32(0x06000348u, m);
    scu_set_mask(m);
}

static bool bup_call(SH2Context& c, uint32_t k);

bool bios_is_dispatcher(uint32_t vec, uint32_t target) {
    return vec >= 0x40 && vec < 0x80 && target == kSlotBase + vec * 4;
}

// While an SCU interrupt's handler runs, the BIOS masks the SCU sources its
// table (SYS_CHGUIPR, 0x06000280) lists for that vector, so that only the
// ones the game allows can nest (SGL lets the DMA ends into its VBlank-IN
// handler, which waits for them). The table's mask is added to the current
// one and the current one comes back after the handler.
void bios_dispatch(SH2Context& c, uint32_t vec) {
    uint32_t h = g_uint[vec & 0x7F];
    uint32_t mask = scu_mask();
    if (vec >= 0x40 && vec < 0x60) scu_set_mask(mask | (g_uipr[vec - 0x40] & 0xFFFFu));
    c.r[15] -= 0x30;                            // r0-r7, mach, macl, pr, gbr saved by the BIOS
    c.pr = kRet;
    c.pc = 0;
    sh2_call(c, h);
    if (c.pc != kRet) sat_fatal("interrupt %02X: handler %08X returned to %08X", vec, h, c.pc);
    scu_set_mask(mask);
}

bool bios_call(SH2Context& c, uint32_t addr) {
    addr &= 0x1FFFFFFFu;
    if (addr >= kDefaultHandler && addr < kDefaultHandler + 0x200) {   // an SCU default handler
        c.pc = c.pr;
        return true;
    }
    if (addr >= kBupFuncs && addr < kBupFuncs + 0x40) {
        if (!bup_call(c, (addr - kBupFuncs) / 4)) return false;
        c.pc = c.pr;
        return true;
    }
    if (addr >= kSlaveSlotBase && addr < kSlaveSlotBase + 0x400)
        sat_fatal("slave vector %02X taken with no handler", (addr - kSlaveSlotBase) / 4);
    if (addr < kSlotBase || addr >= kSlotBase + 0x400) return false;
    uint32_t p = 0x06000000u + (addr - kSlotBase);
    uint32_t r4 = c.r[4], r5 = c.r[5];
    switch (p) {
    case 0x06000300:                            // SYS_SETUINT(vec, handler)
        if (r4 >= 0x80) sat_fatal("SYS_SETUINT(%X)", r4);
        g_uint[r4] = r5 ? r5 : kDefaultHandler + r4 * 4;
        st32(0x06000000u + r4 * 4, kSlotBase + r4 * 4);
        sat_trace("SYS_SETUINT(%02X, %08X)", r4, r5);
        break;
    case 0x06000304:                            // SYS_GETUINT(vec)
        if (r4 >= 0x80) sat_fatal("SYS_GETUINT(%X)", r4);
        c.r[0] = g_uint[r4];
        break;
    case 0x06000310:                            // SYS_SETSINT(vec, handler): the vector itself; 0 = the BIOS's
        if (r4 >= 0x100) sat_fatal("SYS_SETSINT(%X)", r4);
        st32(0x06000000u + r4 * 4, r5 ? r5 : kSlotBase + r4 * 4);
        sat_trace("SYS_SETSINT(%02X, %08X)", r4, r5);
        break;
    case 0x06000314:                            // SYS_GETSINT(vec)
        if (r4 >= 0x100) sat_fatal("SYS_GETSINT(%X)", r4);
        c.r[0] = ld32(0x06000000u + r4 * 4);
        break;
    case 0x06000320:                            // SYS_CHGSYSCK(mode)
        st32(0x06000324u, r4);
        sat_trace("SYS_CHGSYSCK(%u)", r4);
        break;
    case 0x06000330:                            // SYS_TASSEM(n): nonzero when taken
        c.r[0] = g_sem[r4 & 0xFF] ? 0 : 1;
        g_sem[r4 & 0xFF] = 1;
        break;
    case 0x06000334:                            // SYS_CLRSEM(n)
        g_sem[r4 & 0xFF] = 0;
        break;
    case 0x06000340:                            // SYS_SETSCUIM(mask)
        set_mask(r4);
        sat_trace("SYS_SETSCUIM(%08X)", r4);
        break;
    case 0x06000344:                            // SYS_CHGSCUIM(and, or)
        set_mask((ld32(0x06000348u) & r4) | r5);
        sat_trace("SYS_CHGSCUIM(%08X, %08X) -> %08X", r4, r5, ld32(0x06000348u));
        break;
    case 0x06000358:                            // BUP_Init(lib, work, config[3])
        if (!bup_call(c, 0x100)) return false;
        break;
    case 0x06000280:                            // SYS_CHGUIPR(table): 32 words, the SCU mask for each of 0x40-0x5F
        for (uint32_t i = 0; i < 0x20; ++i) g_uipr[i] = ld32(r4 + i * 4);
        sat_trace("SYS_CHGUIPR(%08X)", r4);
        break;
    case 0x0600026C:
        sat_note("the program called 0x0600026C (the BIOS's exit to the system), from %08X", c.pr);
        sat_stop("exit to the system");
    default:
        sat_fatal("BIOS service at %08X (%s) not implemented, from %08X", p,
                  slot_name(p) ? slot_name(p) : "unknown", c.pr);
    }
    c.pc = c.pr;
    return true;
}

// ---- the backup memory -----------------------------------------------------------------
// SBL's BUP interface: BUP_Init (0x06000358) sets 0x06000354 to a table of the
// library's functions: +4 SelPart, +8 Format, +12 Stat, +16 Write, +20 Read,
// +24 Delete, +28 Dir, +32 Verify, +36 GetDate, +40 SetDate. Only device 0
// (the internal memory, 32 KiB in 64-byte blocks) exists. Saves are kept by
// name in out/backup.bin.
struct BupFile { std::string comment; uint8_t language; uint32_t date; std::vector<uint8_t> data; };
static std::map<std::string, BupFile> g_bup;
static bool g_bup_loaded;

static std::string gstr(uint32_t a, int max) {
    std::string s;
    for (int i = 0; i < max; ++i) {
        uint8_t ch = (uint8_t)ld8(a + i);
        if (!ch) break;
        s += (char)ch;
    }
    return s;
}

static void bup_load() {
    if (g_bup_loaded) return;
    g_bup_loaded = true;
    FILE* f = std::fopen((g_cfg.out + "/backup.bin").c_str(), "rb");
    if (!f) return;
    char name[12], comment[11];
    uint8_t lang;
    uint32_t date, size;
    while (std::fread(name, 1, 12, f) == 12 && std::fread(comment, 1, 11, f) == 11 &&
           std::fread(&lang, 1, 1, f) == 1 && std::fread(&date, 4, 1, f) == 1 && std::fread(&size, 4, 1, f) == 1) {
        BupFile b{std::string(comment, strnlen(comment, 11)), lang, date, std::vector<uint8_t>(size)};
        if (size && std::fread(b.data.data(), 1, size, f) != size) break;
        g_bup[std::string(name, strnlen(name, 12))] = b;
    }
    std::fclose(f);
}

void bios_save() {
    if (!g_bup_loaded || g_bup.empty()) return;
    FILE* f = std::fopen((g_cfg.out + "/backup.bin").c_str(), "wb");
    if (!f) return;
    for (auto& [name, b] : g_bup) {
        char n[12] = {}, cm[11] = {};
        std::memcpy(n, name.data(), name.size() < 12 ? name.size() : 12);
        std::memcpy(cm, b.comment.data(), b.comment.size() < 11 ? b.comment.size() : 11);
        uint32_t size = (uint32_t)b.data.size();
        std::fwrite(n, 1, 12, f); std::fwrite(cm, 1, 11, f); std::fwrite(&b.language, 1, 1, f);
        std::fwrite(&b.date, 4, 1, f); std::fwrite(&size, 4, 1, f);
        std::fwrite(b.data.data(), 1, size, f);
    }
    std::fclose(f);
}

static uint32_t days_in(uint32_t y, uint32_t m) {
    static const uint8_t md[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    return m == 2 && y % 4 == 0 && (y % 100 || y % 400 == 0) ? 29 : md[(m - 1) % 12];
}

static const uint32_t kBupSize = 32768, kBupBlock = 64;
static uint32_t blocks(uint32_t size) { return (size + 34 + kBupBlock - 1) / kBupBlock + 1; }

// BupDir: filename[12], comment[11], language, date, datasize, blocksize (u16)
static void put_dir(uint32_t a, const std::string& name, const BupFile& b) {
    for (int i = 0; i < 12; ++i) st8(a + i, i < (int)name.size() ? name[i] : 0);
    for (int i = 0; i < 11; ++i) st8(a + 12 + i, i < (int)b.comment.size() ? b.comment[i] : 0);
    st8(a + 23, b.language);
    st32(a + 24, b.date);
    st32(a + 28, (uint32_t)b.data.size());
    st16(a + 32, blocks((uint32_t)b.data.size()));
}

static bool bup_call(SH2Context& c, uint32_t k) {
    bup_load();
    uint32_t r4 = c.r[4], r5 = c.r[5], r6 = c.r[6], r7 = c.r[7];
    static const char* names[] = {"?", "SelPart", "Format", "Stat", "Write", "Read", "Delete", "Dir",
                                  "Verify", "GetDate", "SetDate"};
    sat_trace("BUP_%s(%08X, %08X, %08X, %08X)", k == 0x100 ? "Init" : k < 11 ? names[k] : "?", r4, r5, r6, r7);
    uint32_t used = 0;
    for (auto& [n, b] : g_bup) used += blocks((uint32_t)b.data.size());
    switch (k) {
    case 0x100:                                 // Init(lib, work, BupConfig[3]): {u16 unit id, u16 partitions}
        st16(r6 + 0, 1); st16(r6 + 2, 1);       // the internal memory
        st16(r6 + 4, 0); st16(r6 + 6, 0);       // no cartridge
        st16(r6 + 8, 0); st16(r6 + 10, 0);
        st32(0x06000354u, kBupTable);
        return true;
    case 1: case 2:                             // SelPart(device, part), Format(device)
        c.r[0] = r4 == 0 ? 0 : 1;
        return true;
    case 3: {                                   // Stat(device, datasize, BupStat*)
        if (r4 != 0) { c.r[0] = 1; return true; }
        uint32_t total = kBupSize / kBupBlock, freeb = total - 2 - used;
        st32(r6 + 0, kBupSize); st32(r6 + 4, total); st32(r6 + 8, kBupBlock);
        st32(r6 + 12, freeb * kBupBlock); st32(r6 + 16, freeb);
        st32(r6 + 20, r5 ? freeb / blocks(r5) : 0);
        c.r[0] = 0;
        return true;
    }
    case 4: {                                   // Write(device, BupDir*, data*, mode)
        std::string name = gstr(r5, 11);
        BupFile b{gstr(r5 + 12, 10), (uint8_t)ld8(r5 + 23), ld32(r5 + 24), {}};
        uint32_t size = ld32(r5 + 28);
        if (g_bup.count(name) && r7 == 1) { c.r[0] = 6; return true; }   // exists, no overwrite (BUP_FOUND?)
        b.data.resize(size);
        for (uint32_t i = 0; i < size; ++i) b.data[i] = (uint8_t)ld8(r6 + i);
        g_bup[name] = b;
        sat_note("BUP: wrote %s (%u bytes)", name.c_str(), size);
        bios_save();
        c.r[0] = 0;
        return true;
    }
    case 5: {                                   // Read(device, name, data*)
        auto it = g_bup.find(gstr(r5, 11));
        if (r4 != 0 || it == g_bup.end()) { c.r[0] = 5; return true; }
        for (size_t i = 0; i < it->second.data.size(); ++i) st8(r6 + (uint32_t)i, it->second.data[i]);
        c.r[0] = 0;
        return true;
    }
    case 6: {                                   // Delete(device, name)
        c.r[0] = g_bup.erase(gstr(r5, 11)) ? 0 : 5;
        bios_save();
        return true;
    }
    case 7: {                                   // Dir(device, name prefix, table size, BupDir*): matches
        std::string pre = gstr(r5, 11);
        int n = 0;
        for (auto& [name, b] : g_bup)
            if (!name.compare(0, pre.size(), pre)) {
                if ((uint32_t)n < (r6 & 0xFFFF)) put_dir(r7 + n * 36, name, b);
                ++n;
            }
        c.r[0] = r4 == 0 ? (uint32_t)n : (uint32_t)-1;
        return true;
    }
    case 8: {                                   // Verify(device, name, data*)
        auto it = g_bup.find(gstr(r5, 11));
        if (it == g_bup.end()) { c.r[0] = 5; return true; }
        bool same = true;
        for (size_t i = 0; i < it->second.data.size(); ++i) same &= ld8(r6 + (uint32_t)i) == it->second.data[i];
        c.r[0] = same ? 0 : 7;
        return true;
    }
    case 9: {                                   // GetDate(date, BupDate*): minutes since 1980-01-01
        uint32_t days = r4 / 1440, mins = r4 % 1440, y = 1980, m = 1;
        uint32_t week = (days + 2) % 7;         // 1980-01-01 was a Tuesday (0 = Sunday)
        for (;;) {
            uint32_t yd = y % 4 == 0 && (y % 100 || y % 400 == 0) ? 366 : 365;
            if (days < yd) break;
            days -= yd; ++y;
        }
        for (;; ++m) {
            uint32_t md = days_in(y, m);
            if (days < md) break;
            days -= md;
        }
        st8(r5 + 0, y - 1980); st8(r5 + 1, m); st8(r5 + 2, days + 1);
        st8(r5 + 3, mins / 60); st8(r5 + 4, mins % 60); st8(r5 + 5, week);
        return true;
    }
    case 10: {                                  // SetDate(BupDate*) -> date
        uint32_t y = 1980 + ld8(r4), m = ld8(r4 + 1), d = ld8(r4 + 2), days = 0;
        for (uint32_t k = 1980; k < y; ++k) days += k % 4 == 0 && (k % 100 || k % 400 == 0) ? 366 : 365;
        for (uint32_t k = 1; k < m && k <= 12; ++k) days += days_in(y, k);
        days += d ? d - 1 : 0;
        c.r[0] = (days * 24 + ld8(r4 + 3)) * 60 + ld8(r4 + 4);
        return true;
    }
    }
    sat_fatal("BUP function %u not implemented", k);
}
