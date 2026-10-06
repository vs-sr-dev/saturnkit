"""Saturn disc images: .cue/.bin sets and plain .iso; IP.BIN, tracks, ISO 9660.

    python -m saturnkit.disc GAME.cue --info
    python -m saturnkit.disc GAME.cue --list
    python -m saturnkit.disc GAME.cue --extract out/         # files + IP.BIN
    python -m saturnkit.disc GAME.cue --audio out/           # CD-DA tracks as WAV

Layers, from the outside in:

CUE   One FILE per track (Redump) or one FILE for all (older rips). Track 1 is
      data (MODE1/2352, or MODE1/2048 for a plain .iso); the others are
      usually CD-DA (AUDIO, 2352-byte frames of 16-bit stereo 44.1 kHz,
      little-endian). INDEX 00 is the pregap, INDEX 01 the start of the track.
      A PREGAP line is a pregap that is not in the file (silence on the
      disc, as an older rip of one .bin for every track keeps track 2's).
      Disc time (MSF, 75 frames a second) starts at 00:02:00 = LBA 0.

Data  A MODE1/2352 sector is 12 sync bytes, 4 header bytes (M S F mode),
      2048 user bytes, EDC/ECC. A MODE2/2352 sector (CD-ROM XA) has the same
      sync and header, then an 8-byte subheader (file number, channel,
      submode, coding info, twice), then 2048 user bytes (Form 1, submode
      bit 5 clear) or 2324 (Form 2). Only the user bytes matter above this
      layer. A disc may hold its file system over several data tracks; a
      sector is found by its disc LBA, whatever track holds it.

IP    Sectors 0-15 of the data track are the system area. IP.BIN (usually
      0x1000-0x8000 bytes) starts at sector 0:
        0x00 hardware id "SEGA SEGASATURN "  0x10 maker id
        0x20 product number (10)             0x2A version (6)
        0x30 release date YYYYMMDD           0x38 device info "CD-1/1  "
        0x40 area symbols (J T U B K A E L)  0x50 peripherals (J A K M S T G ...)
        0x60 title (112)                     0xE0 IP size
        0xE8 master stack   0xEC slave stack 0xF0 1st read address
        0xF4 1st read size (0 = whole file)
      At 0x100 the security code (SYS_SEC), then one area code block per
      area symbol (SYS_ARE*), then the game's own IP code (SYS_INIT etc.),
      run by the BIOS before the 1st read file.

ISO   Standard ISO 9660 from sector 16 (PVD). The BIOS loads the 1st read
      file: the first file record in the root directory, i.e. the first
      entry after '.' and '..' (Sega's tools put it there; the name is
      conventional, 0.BIN, 1ST_READ.BIN, A0.BIN...), to the 1st read address.

      A file record may be interleaved: its file unit size (byte 26) and
      interleave gap (byte 27) say the file is `unit` sectors, then `gap`
      sectors of other files, and so on. Its CD-ROM XA system use field
      ("XA" at +6) gives the attributes and the file number the sectors'
      subheaders carry (the CD block's filters select on it). An XA
      attribute 0x4000 marks a record that points at a CD-DA track.
"""
import argparse
import os
import re
import struct

SECTOR_RAW = 2352
SECTOR = 2048

AREAS = {"J": "Japan", "T": "Asia NTSC", "U": "North America", "B": "Brazil",
         "K": "Korea", "A": "Asia PAL", "E": "Europe", "L": "Latin America"}
PERIPHERALS = {"J": "control pad", "A": "analog pad", "M": "mouse",
               "K": "keyboard", "S": "steering wheel", "T": "multitap",
               "G": "light gun", "W": "RAM cartridge", "C": "link cable",
               "F": "floppy drive", "D": "modem", "X": "X-band modem",
               "E": "3D control pad", "P": "video CD card", "R": "Photo CD"}


# ---------------------------------------------------------------- CUE / tracks

class Track:
    def __init__(self, number, mode, path, file_offset, pregap):
        self.number, self.mode, self.path = number, mode, path
        self.file_offset = file_offset   # byte offset of INDEX 01 in path
        self.pregap = pregap             # frames between INDEX 00 and 01
        self.gap = 0                     # frames of a PREGAP not in the file, before those
        self.sector_size = 2048 if mode == "MODE1/2048" else SECTOR_RAW
        self.user_offset = 24 if mode.startswith("MODE2") else 16
        self.frames = 0                  # filled in by parse_cue
        self.lba = 0                     # disc LBA of INDEX 01

    @property
    def is_audio(self):
        return self.mode == "AUDIO"

    def __repr__(self):
        return "<Track %02d %s lba=%d frames=%d>" % (self.number, self.mode,
                                                     self.lba, self.frames)


def _msf(s):
    m, sec, f = (int(x) for x in s.split(":"))
    return (m * 60 + sec) * 75 + f


def parse_cue(path):
    """Tracks of a .cue sheet, with their disc LBAs (pregaps counted)."""
    base = os.path.dirname(os.path.abspath(path))
    tracks, cur_file, cur = [], None, None
    idx00 = {}
    for line in open(path, encoding="latin-1"):
        tok = line.strip()
        m = re.match(r'FILE\s+"(.*)"\s+(\S+)', tok) or re.match(r"FILE\s+(\S+)\s+(\S+)", tok)
        if m:
            cur_file = os.path.join(base, m.group(1))
            continue
        m = re.match(r"TRACK\s+(\d+)\s+(\S+)", tok)
        if m:
            cur = Track(int(m.group(1)), m.group(2), cur_file, None, 0)
            tracks.append(cur)
            continue
        m = re.match(r"PREGAP\s+(\S+)", tok)
        if m and cur:
            cur.gap = _msf(m.group(1))
            continue
        m = re.match(r"INDEX\s+(\d+)\s+(\S+)", tok)
        if m and cur:
            n, t = int(m.group(1)), _msf(m.group(2))
            if n == 0:
                idx00[cur.number] = t
            elif n == 1:
                cur.file_offset = t * cur.sector_size
                cur.pregap = t - idx00.get(cur.number, t)
    # sizes and LBAs: one file per track (Redump) or several tracks per file
    by_file = {}
    for t in tracks:
        by_file.setdefault(t.path, []).append(t)
    for f, ts in by_file.items():
        size = os.path.getsize(f)
        for i, t in enumerate(ts):
            end = ts[i + 1].file_offset - ts[i + 1].pregap * ts[i + 1].sector_size \
                if i + 1 < len(ts) else size
            t.frames = (end - t.file_offset) // t.sector_size
    lba = 0
    for t in tracks:
        lba += t.gap + t.pregap
        t.lba = lba
        lba += t.frames
    return tracks


class Disc:
    """A Saturn disc: tracks, the data track's user sectors, IP.BIN, ISO 9660."""

    def __init__(self, path):
        if path.lower().endswith(".cue"):
            self.tracks = parse_cue(path)
        else:                                   # plain .iso or single .bin
            size = os.path.getsize(path)
            with open(path, "rb") as f:
                head = f.read(16)
            mode = "MODE1/2352" if head[:12] == b"\x00" + b"\xff" * 10 + b"\x00" else "MODE1/2048"
            t = Track(1, mode, path, 0, 0)
            t.frames = size // t.sector_size
            self.tracks = [t]
        self.data = self.tracks[0]
        self._fh = {}
        self.ip = IP(self.read_sectors(0, 16))
        self.iso = ISO9660(self)

    def track_at(self, lba):
        """The track holding disc LBA `lba` (its pregap counted as its own)."""
        found = self.tracks[0]
        for t in self.tracks:
            if t.lba - t.pregap - t.gap <= lba:
                found = t
        return found

    def raw_sector(self, lba):
        """(stored bytes, track) of the sector at disc LBA `lba`: 2352 bytes, or 2048 for a plain .iso."""
        t = self.track_at(lba)
        if lba < t.lba - t.pregap:              # a PREGAP that is not in the file
            return bytes(t.sector_size), t
        fh = self._fh.get(t.path)
        if fh is None:
            fh = self._fh[t.path] = open(t.path, "rb")
        fh.seek(t.file_offset + (lba - t.lba) * t.sector_size)
        return fh.read(t.sector_size), t

    def read_sectors(self, lba, count=1):
        """User bytes (2048 a sector) of `count` data sectors from disc LBA `lba`."""
        out = bytearray()
        for i in range(count):
            raw, t = self.raw_sector(lba + i)
            out += raw[t.user_offset:t.user_offset + SECTOR] if t.sector_size == SECTOR_RAW else raw
        return bytes(out)

    def read(self, lba, size, unit=0, gap=0):
        """`size` bytes from `lba`; interleaved if `unit`: `unit` sectors, then `gap` skipped, again."""
        n = (size + SECTOR - 1) // SECTOR
        if not unit:
            return self.read_sectors(lba, n)[:size]
        out = bytearray()
        while len(out) < n * SECTOR:
            out += self.read_sectors(lba, min(unit, n - len(out) // SECTOR))
            lba += unit + gap
        return bytes(out[:size])

    def is_cdda(self, rec):
        """A record of a CD-DA track: by its XA attribute, or by the track its LBA falls in."""
        return rec.is_cdda or (not rec.is_dir and self.track_at(rec.lba).is_audio)

    def read_file(self, rec):
        return self.read(rec.lba, rec.size, rec.unit, rec.gap)

    def first_read(self):
        """(record, bytes) of the 1st read file."""
        rec = self.iso.first_file()
        return rec, self.read_file(rec)


# ---------------------------------------------------------------- IP.BIN

class IP:
    def __init__(self, data):
        self.raw = data
        s = lambda a, b: data[a:b].decode("latin-1").rstrip()
        self.hardware = s(0x00, 0x10)
        self.maker = s(0x10, 0x20)
        self.product = s(0x20, 0x2A)
        self.version = s(0x2A, 0x30)
        self.date = s(0x30, 0x38)
        self.device = s(0x38, 0x40)
        if re.fullmatch(rb"\d{4}-\d\d-\d\d", data[0x30:0x3A]):   # some discs write YYYY-MM-DD into the device field
            self.date, self.device = s(0x30, 0x3A), s(0x3A, 0x40).lstrip()
        self.areas = s(0x40, 0x50).replace(" ", "")
        self.peripherals = s(0x50, 0x60).replace(" ", "")
        self.title = s(0x60, 0xD0)
        (self.ip_size, _, self.master_stack, self.slave_stack,
         self.first_read_addr, self.first_read_size) = struct.unpack(">6I", data[0xE0:0xF8])
        self.valid = self.hardware == "SEGA SEGASATURN"

    def area_blocks(self):
        """Offsets of the area code blocks (each contains the area's banner string)."""
        out = []
        for m in re.finditer(rb"For (JAPAN|USA|EUROPE|TAIWAN|KOREA|BRAZIL|[A-Z. ]+?)\.", self.raw[:self.ip_size or 0x8000]):
            out.append((m.start(), m.group(0).decode("latin-1")))
        return out

    def describe(self):
        lines = [
            "hardware   %s" % self.hardware,
            "maker      %s" % self.maker,
            "product    %s  version %s  date %s" % (self.product, self.version, self.date),
            "device     %s" % self.device,
            "areas      %s  (%s)" % (self.areas, ", ".join(AREAS.get(c, c) for c in self.areas)),
            "peripheral %s  (%s)" % (self.peripherals, ", ".join(PERIPHERALS.get(c, c) for c in self.peripherals)),
            "title      %s" % self.title,
            "IP size    0x%X" % self.ip_size,
            "stacks     master 0x%08X  slave 0x%08X" % (self.master_stack, self.slave_stack),
            "1st read   at 0x%08X, size %s" % (self.first_read_addr,
                                               "whole file" if self.first_read_size == 0 else "0x%X" % self.first_read_size),
        ]
        for off, txt in self.area_blocks():
            lines.append("area block 0x%04X  %r" % (off, txt))
        return "\n".join(lines)


# ---------------------------------------------------------------- ISO 9660

class Record:
    def __init__(self, name, lba, size, is_dir, path, unit=0, gap=0, xa_attr=None, xa_file=0):
        self.name, self.lba, self.size, self.is_dir, self.path = name, lba, size, is_dir, path
        self.unit, self.gap = unit, gap                # interleave: file unit size, gap (sectors)
        self.xa_attr, self.xa_file = xa_attr, xa_file  # CD-ROM XA attributes and file number

    @property
    def is_cdda(self):
        return self.xa_attr is not None and bool(self.xa_attr & 0x4000)

    def __repr__(self):
        return "<%s %s lba=%d size=%d>" % ("dir" if self.is_dir else "file", self.path, self.lba, self.size)


class ISO9660:
    def __init__(self, disc):
        self.disc = disc
        pvd = disc.read_sectors(16)
        if pvd[1:6] != b"CD001":
            raise ValueError("no ISO 9660 primary volume descriptor at sector 16")
        self.volume_id = pvd[40:72].decode("latin-1").rstrip()
        self.system_id = pvd[8:40].decode("latin-1").rstrip()
        self.volume_size = struct.unpack("<I", pvd[80:84])[0]
        self.publisher = pvd[318:446].decode("latin-1").rstrip()
        self.preparer = pvd[446:574].decode("latin-1").rstrip()
        self.application = pvd[574:702].decode("latin-1").rstrip()
        self.copyright_file = pvd[702:739].decode("latin-1").rstrip()
        self.abstract_file = pvd[739:776].decode("latin-1").rstrip()
        self.biblio_file = pvd[776:813].decode("latin-1").rstrip()
        self.created = pvd[813:830].decode("latin-1")
        root = pvd[156:190]
        self.root = Record("", struct.unpack("<I", root[2:6])[0], struct.unpack("<I", root[10:14])[0], True, "/")

    def listdir(self, rec):
        data = self.disc.read(rec.lba, rec.size)
        out, pos = [], 0
        while pos < len(data):
            ln = data[pos]
            if ln == 0:                       # records never cross a sector
                pos = (pos // SECTOR + 1) * SECTOR
                continue
            r = data[pos:pos + ln]
            lba, size = struct.unpack("<I", r[2:6])[0], struct.unpack("<I", r[10:14])[0]
            flags, unit, gap, nlen = r[25], r[26], r[27], r[32]
            name = r[33:33 + nlen]
            su = r[33 + nlen + (1 - nlen % 2):]      # system use, after the name's padding
            xa_attr, xa_file = (struct.unpack(">H", su[4:6])[0], su[8]) if su[6:8] == b"XA" else (None, 0)
            pos += ln
            if name in (b"\x00", b"\x01"):
                continue
            name = name.decode("latin-1").split(";")[0]
            out.append(Record(name, lba, size, bool(flags & 2),
                              rec.path.rstrip("/") + "/" + name, unit, gap, xa_attr, xa_file))
        return out

    def walk(self, rec=None):
        for r in self.listdir(rec or self.root):
            yield r
            if r.is_dir:
                yield from self.walk(r)

    def first_file(self):
        for r in self.listdir(self.root):
            if not r.is_dir:
                return r
        raise ValueError("no file in the root directory")

    def find(self, path):
        want = path.strip("/").upper()
        for r in self.walk():
            if r.path.strip("/").upper() == want:
                return r
        raise KeyError(path)


# ---------------------------------------------------------------- CD-DA

def audio_frames(track):
    """Raw 16-bit LE stereo PCM of an audio track, INDEX 01 onward."""
    with open(track.path, "rb") as f:
        f.seek(track.file_offset)
        return f.read(track.frames * SECTOR_RAW)


def write_wav(path, pcm, rate=44100, channels=2):
    with open(path, "wb") as f:
        f.write(b"RIFF" + struct.pack("<I", 36 + len(pcm)) + b"WAVE")
        f.write(b"fmt " + struct.pack("<IHHIIHH", 16, 1, channels, rate,
                                      rate * channels * 2, channels * 2, 16))
        f.write(b"data" + struct.pack("<I", len(pcm)) + pcm)


# ---------------------------------------------------------------- CLI

def _time(frames):
    s = frames / 75
    return "%d:%05.2f" % (s // 60, s % 60)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("image")
    ap.add_argument("--info", action="store_true")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--extract", metavar="DIR")
    ap.add_argument("--audio", metavar="DIR")
    a = ap.parse_args(argv)
    d = Disc(a.image)
    if a.info or not (a.list or a.extract or a.audio):
        print(d.ip.describe())
        iso = d.iso
        print("volume     %r  system %r  %d sectors" % (iso.volume_id, iso.system_id, iso.volume_size))
        print("publisher  %r  preparer %r" % (iso.publisher, iso.preparer))
        print("app        %r" % iso.application)
        print("cpy/abs/bib %r %r %r  created %s" % (iso.copyright_file, iso.abstract_file, iso.biblio_file, iso.created))
        fr = iso.first_file()
        print("1st read file %s (%d bytes, lba %d)" % (fr.path, fr.size, fr.lba))
        print("tracks:")
        for t in d.tracks:
            print("  %02d %-10s lba %6d  %6d frames  %s%s%s" % (
                t.number, t.mode, t.lba, t.frames, _time(t.frames),
                "  pregap %d" % (t.gap + t.pregap) if t.gap + t.pregap else "",
                " (%d not in the file)" % t.gap if t.gap else ""))
    if a.list:
        for r in d.iso.walk():
            extra = "  CD-DA, track %02d" % d.track_at(r.lba).number if d.is_cdda(r) else ""
            if r.unit:
                extra += "  interleaved %d/%d, XA file %d" % (r.unit, r.gap, r.xa_file)
            print("%-40s %8s lba %6d%s" % (r.path, "<dir>" if r.is_dir else r.size, r.lba, extra))
    if a.extract:
        os.makedirs(a.extract, exist_ok=True)
        with open(os.path.join(a.extract, "IP.BIN"), "wb") as f:
            f.write(d.ip.raw[:d.ip.ip_size or len(d.ip.raw)])
        n, cdda = 0, []
        for r in d.iso.walk():
            out = os.path.join(a.extract, *r.path.strip("/").split("/"))
            if r.is_dir:
                os.makedirs(out, exist_ok=True)
            elif d.is_cdda(r):                       # its sound is the track's: --audio
                cdda.append("%s (track %02d)" % (r.path, d.track_at(r.lba).number))
            else:
                with open(out, "wb") as f:
                    f.write(d.read_file(r))
                n += 1
        print("%d files and IP.BIN extracted to %s" % (n, a.extract))
        if cdda:
            print("records of CD-DA tracks, not extracted: %s" % ", ".join(cdda))
    if a.audio:
        os.makedirs(a.audio, exist_ok=True)
        for t in d.tracks:
            if t.is_audio:
                write_wav(os.path.join(a.audio, "track%02d.wav" % t.number), audio_frames(t))
        print("%d audio tracks written to %s" % (sum(t.is_audio for t in d.tracks), a.audio))


if __name__ == "__main__":
    main()
