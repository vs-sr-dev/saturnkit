# saturnkit

A game-agnostic toolkit for Sega Saturn reverse engineering and native PC
ports: disc images, the SH-2 CPU, the hardware's address map, a static
recompiler from SH-2 code to C++, and a runtime that replaces the Saturn's
hardware under the recompiled game: both SH-2s, VDP1 and VDP2, the SCU, the
SMPC, the CD block, and the sound side (the 68000 and the SCSP).

The same idea as [wiikit](https://github.com/vs-sr-dev/wiikit) and
[ps2kit](https://github.com/vs-sr-dev/pc-extermination/tree/main/ps2kit),
for the Saturn. Each Saturn game has its own engine and formats, but a
large part of every port is the *same* work: the same disc layout, the same
two SH-2s, the same VDP1, VDP2, SCU, SMPC, SCSP and CD block, and very
often the same Sega libraries (SBL, SGL). saturnkit collects that shared
part. It grows inside the ports: each piece is written because a game
needed it, then kept free of that game's knowledge. Game formats and game
fixes live in the ports.

## Ports built on it

| Port | Game | What it asked of saturnkit |
|---|---|---|
| [pc-virtualhydlide](https://github.com/vs-sr-dev/pc-virtualhydlide) | Virtual Hydlide (1995) | the disc, the SH-2 decoder, the address map, function discovery, cross-program matching, the interpreter, the recompiler, the runtime core (HLE boot and BIOS, SCU, SMPC, the slave as a coroutine, the CD block), VDP1 and VDP2 in software in a window with the pad, the 68000 and the SCSP (sound), recompiler hooks for a game layer, and the fields between a game's frames drawn moving (`--interp`) |
| [pc-deepfear](https://github.com/vs-sr-dev/pc-deepfear) | Deep Fear (1998) | function discovery for GCC and SGL code (GCC's switches, `mova` tables and pointers, tables of records, computed jumps into unrolled code, SGL's hand-written handlers), the SCU DSP (a disassembler, and an interpreter in the runtime), `SYS_CHGUIPR` and the per-interrupt SCU masks, literal-pool slots that the code writes, GFS_SGL's streams on the CD block (Play's modes and a pickup kept where it reads), the division unit's shadow registers, the DMAC's 16-byte transfers, `--dump` with VDP2's registers, VDP2's windows |
| [pc-xjapan](https://github.com/vs-sr-dev/pc-xjapan) | X JAPAN Virtual Shock 001 (1995) | a file system over a Mode 1 and a CD-ROM XA track, interleaved files and records of CD-DA tracks, function discovery through far jumps and callbacks that do nothing, TVSTAT's HBLANK, the pad read directly through the SMPC's ports, the disc's area code, a game played in the window recorded and given back headless (`--record-input`, `--input @FILE`) |

## Using it

A port takes saturnkit as a git submodule at `saturnkit/`, so that
`python -m saturnkit.…` works from the port's root:

```sh
git submodule add https://github.com/vs-sr-dev/saturnkit.git saturnkit
git clone --recursive <port>          # or: git submodule update --init
```

Each port pins a saturnkit commit and moves it forward deliberately.

```sh
python -m saturnkit.disc GAME.cue --info              # IP.BIN, ISO 9660, tracks
python -m saturnkit.disc GAME.cue --list
python -m saturnkit.disc GAME.cue --extract build/extract   # files + IP.BIN
python -m saturnkit.disc GAME.cue --audio build/audio       # CD-DA as WAV
python -m saturnkit.sh2 FILE.BIN --find-base               # where it loads
python -m saturnkit.sh2 FILE.BIN --base 0600B000 --at 0600B000 --count 64
python -m saturnkit.sh2 FILE.BIN --base 0600B000 --refs 25D00000:25D00018
python -m saturnkit.sh2 FILE.BIN --base 0600B000 --census
python -m saturnkit.hw 25D00002 25F80114 06000310
python -m saturnkit.recomp.discover FILE.BIN --base 0600B000 --report     # functions, code/data
python -m saturnkit.recomp.match A.BIN@0600B000 B.BIN@0600B000 --names a.tsv --out b.tsv
python -m saturnkit.sh2emu FILE.BIN --base 0600B000 --call 060224DC --regs r1=100,r0=7
python -m saturnkit.recomp --out build/recomp A=A.BIN@0600B000 B=B.BIN@0600B000 --optest   # to C++
python -m saturnkit.recomp --out build/recomp A=A.BIN@0600B000 --hook A:0600B6F4   # sh2_hook after that instruction
python -m saturnkit.recomp.selftest --out a.txt --image A=A.BIN@0600B000 --test A --auto  # vectors
cmake -S build/recomp -B build/recomp-build -G Ninja -DCMAKE_CXX_COMPILER=clang++
ninja -C build/recomp-build && build/recomp-build/selftest build/recomp/selftest/optest.txt a.txt
build/recomp-build/saturn --cue GAME.cue                              # the game in a window, with the pad and sound
build/recomp-build/saturn --cue GAME.cue --interp                     # the fields between the game's frames drawn moving
build/recomp-build/saturn --cue GAME.cue --headless --vblanks 3600 --wav run.wav   # the run's sound as a WAV
build/recomp-build/saturn --cue GAME.cue --hook 0600B6F4:r5=1         # a register set at a hooked instruction
build/recomp-build/saturn --cue GAME.cue --headless --vblanks 600 --trace --shot 300,600   # headless, pictures
build/recomp-build/saturn --cue GAME.cue --headless --input 1200:START,1210: --peek 25F80000:8 --watch 05A00000:05A01000
build/recomp-build/saturn --cue GAME.cue --record-input play.txt          # the pad as played in the window, as a script
build/recomp-build/saturn --cue GAME.cue --headless --input @play.txt --vblanks 9000 --shot 7760   # that game again, headless
build/recomp-build/saturn --cue GAME.cue --headless --watch 06058D6C:06058DCF --watch-vblanks 7300:7310   # every store, by function
```

## Layers

| Layer | Question it answers | Now | Next |
|---|---|---|---|
| 1. Recognise | What is on this disc? | `disc --info`: IP.BIN (product, areas, peripherals, stacks, 1st read), ISO 9660 volume, track list with pregaps | a `fingerprint`: SBL/SGL and their versions by code signature, CRI middleware, sound driver versions |
| 2. Extract | Turn standard formats into standard files | `disc --extract` (a file system over MODE1 and MODE2 tracks, interleaved files, records of CD-DA tracks left out), `disc --audio` (CD-DA to WAV) | VDP1/VDP2 image decoders (4/8/16 bpp, CLUT, CRAM), Sega FILM/Cinepak, SCSP tone banks |
| 3. Map code | What does the code do, where? | `sh2` (SH7604 decoder, disassembly with literal pools resolved and hardware registers named, `--refs`, `--census`, `--find-base`), `hw` (address map, register names, BIOS service pointers, SCU vectors), `sh2emu` (an SH-2 interpreter for isolated functions: the recompiler's oracle) | executable map (crt0, BSS, programs swapped at one address); `fingerprint` (SBL by signature) |
| 4. Translate | Turn SH-2 code into C++ | `recomp.discover` (functions and code/data in stripped SHC code: recursive descent, constant propagation for register calls, four switch forms, pointer and prologue seeds), `recomp.match` (the same function across programs linked at other addresses; names carried), `recomp` (one C++ function per entry: delay slots, calls checked on return, guarded dispatch through registers, switches, safe points; one module per program, recognised in memory by its crc32), `recomp.selftest` (vectors from `sh2emu`: every instruction form, and a program's functions that run alone), hooks (`--hook`: a call to `sh2_hook` after chosen instructions, where the runtime sets a register or a port's game layer runs) | replacing a whole function by a hand-written one |
| 5. Runtime | Replace the hardware | `runtime/` (C++20): the SH-2 context, the work RAMs with the cache-through addresses, dispatch over the active modules, the self-test harness; the Saturn (`saturn` executable): virtual or host time, program starts with the host stack unwound, HLE boot and BIOS services (interrupt dispatch, SCU mask, semaphores, BUP in a host file), SCU (interrupts, timers, DMA), SMPC (INTBACK, the pad read directly through PDR1/DDR1, the disc's area, a scripted pad, a played pad recorded and given back, slave and sound on/off), the slave SH-2 as a deterministic coroutine, the CD block at its registers over .cue/.bin, the SH7604's division unit, FRT and DMAC, the raster timing (TVSTAT's VBLANK and HBLANK), VDP1 drawn in software (every command, colour mode and colour calculation, the chip's way of walking quadrilaterals), VDP2 composed in software (NBG0–3 in cell and bitmap modes with zoom, the sprite layer, priorities, colour calculation and offsets, the back screen, windows 0 and 1), an SDL3/OpenGL 4.5 window with the keyboard and a gamepad as the pad; the 68000 (Musashi) and the SCSP (slots, envelopes, LFOs, FM, timers, interrupts, DMA, the DSP, CD-DA at its external input) in virtual time, out through an SDL3 audio stream or a WAV; `--interp`, the fields between a game's frames drawn by VDP1 with every command moved part of the way (matched by keys a game layer gives); a log of every register touched and a store watch over the work RAMs | VDP1 on the GPU, the rest of VDP2 (rotation, the sprite window, line scroll…), `--interp` without its frame of latency |

## Principles

* Pure Python, no dependencies, for layers 1–4; the runtime (layer 5) is
  C++20 with SDL3 and OpenGL 4.5, built with CMake, Ninja and clang, as in wiikit.
* Every claim is checked on a real disc before it goes in.
* Game knowledge stays out.
* Every change is checked on every port before it goes in.

## Checks behind each module

| Module | Checked by |
|---|---|
| `disc` | Virtual Hydlide (Europe), a Redump .cue with 28 tracks: IP.BIN parsed (MK-81380, area E), 458 files extracted from ISO 9660, the 1st read file found (`A.BIN`, identical to the game's own `OPEN.BIN`), track LBAs with their pregaps. A disc whose ISO 9660 spans a MODE1/2352 track and a MODE2/2352 one (Form 1): 270 files extracted, five of them interleaved (file unit 4 or 3, gap 15 or 16), every one of their 45 546 sectors carrying the XA file number of its record; the other files byte-identical to before on Virtual Hydlide and on a two-disc SGL game |
| `sh2` | every one of the 65 536 16-bit words against capstone 5 (SH-2 mode): 53 752 decode to the same mnemonic and operands; the 452 capstone alone decodes are SH-2A extensions (`movua`, `divs`, `jsr/n`, `clips`…) that the SH7604 does not have. `--find-base` puts 14 of 15 Virtual Hydlide executables at the address their crt0 implies |
| `recomp.discover` | Virtual Hydlide, 15 programs: no function whose descent meets data or an undefined opcode; every resolved switch target is code. Against Ghidra 12's auto-analysis of the field program: 571 of its 576 functions found; the other 5 are shared tails (common epilogues several functions branch to), which discovery follows as part of each caller |
| `recomp.match` | Virtual Hydlide: the frame limiter, found in each program by its bytes, is where the match carries its name in 11 of 11 programs |
| `sh2emu` | the SHC runtime's four division helpers run on 3 000 random operands each: quotient and remainder, signed and unsigned, 12 000 of 12 000 equal to Python |
| `recomp` (emit) | Virtual Hydlide, 15 programs: 10 259 functions, 1.5 million instructions, compiled with clang 22 and linked; no static target outside the modules. The self-test: 775 functions of the instruction test (126 forms, alone and in a delay slot, and the control flow) and 2 653 of the game's, 51 532 vectors, 0 differences in registers or memory |
| `runtime` (the Saturn) | Virtual Hydlide from the boot of its 1st read file to its first field: the opening movie through the CD block and the PCM handshake, three program starts (HYDSYS, the title menu, the field), both CPUs, the field's frame loop at its 12 fps cap; 120 s of game time with no wrong return and no call to a non-entry, the same run every time |
| `runtime` (the pad, HBLANK, recorded play) | a 1995 FMV game on SBL's Cinepak player: it waits for TVSTAT's HBLANK before writing TVMD and reads its pad only through PDR1 (the ID 0xB, START, the directions, A); from the boot through its movies, the title and the backstage to its first item, played in the window by the user, then given back headless with `--input @FILE` along the same route. Virtual Hydlide and a two-disc SGL game give byte-identical pictures and sound before and after |
| `runtime` (the CD block's Play, DIVU, DMAC) | a two-disc SGL game from the boot through its title, menu, loading (its music streaming while files are read), opening movie and into its first room, the player walking, against Beetle Saturn's pictures of the same room: the background, the 3D player, the gauges and the room's name the same. Virtual Hydlide and a 1995 FMV game the same frames and sound before and after |
| `runtime` (sound) | Virtual Hydlide against Beetle Saturn's recording: the opening movie's sound correlates at 0.99 with Beetle's, same stereo image, at a constant offset (1.6 dB louder); the title silent in both; the menus' and the field's music and effects driven by the game's own sound drivers |
| `runtime` (`--interp`) | Virtual Hydlide's field, with its game layer's keys: 500 fields of walking and turning, no cracks beyond the game's own, trees whole through a turn; the movie, the title and the menus the same picture as without it, pixel for pixel; played by the user |
| `runtime` (VDP1, VDP2, the window) | Virtual Hydlide against Beetle Saturn: the title and the opening movie's frames the same pictures, the menus and the first field the same layout and look; the movie at its own 15 fps, paced by the PCM play position |
| `sh2.encode` | every word that decodes (53 752 of the 65 536): encoded back from its format and fields, 53 752 of 53 752 equal |
| `hw` | the register names Virtual Hydlide's code uses, read in context (VDP1 FBCR/PTMR/EDSR, VDP2 TVSTAT and colour offset, SMPC COMREG, FRT FTCSR in the slave's wait loop) |

## Known gaps

* Discovery leaves unreached what nothing references: leaf functions with
  no prologue called only by dead code (about 7% of the opening program's
  text). The runtime stops at a call to an address that is not an entry
  and logs it, to be given as a seed; Virtual Hydlide made none from the
  boot to its first field.
* A computed jump that fits none of the four switch forms stays
  unresolved; the recompiled code tries every instruction of its own
  function, then dispatches. In Virtual Hydlide all 36 left are tail calls
  through tables of function pointers, dispatched at run time.
* The recompiled code's semantics are `sh2emu`'s. Where the interpreter
  could differ from the hardware (`mac.w` saturation, `div1` corner cases)
  the self-test cannot tell; the running game is the check.
* Interrupts reach recompiled code at safe points only (back-edges, calls,
  `ldc …,sr`); there are no cycle counts. An interrupt raised twice before
  a poll takes it is taken once (an SCU timer on every line comes about
  once a poll).
* Sound: MIDI is not done (its input reads empty); CD-DA reaches the
  SCSP's external input but no game has played it yet. VDP1 draws in
  software at the Saturn's resolution; VDP2's rotation planes, the sprite window, line and cell scroll,
  mosaic, line colour screen and special functions are not done (the
  runtime notes a register that asks for one). Not emulated: the
  SH-2's on-chip interrupts, the caches.
* `match` needs a fingerprint unique on both sides; identical small
  functions (library copies) are matched only through their callers.
* `--find-base` scores literals pointing at function prologues. A small
  program that calls into a large resident one can score higher at a
  wrong base (Virtual Hydlide's MENU.BIN): check against the crt0, whose
  BSS starts where the file ends.
* `hw.BIOS`: the service-pointer names come from the SBL headers; those not
  marked as checked by a game are unconfirmed.
* `sh2.Image.literal_refs` reads every `mov.l @(disp,PC)` in the file,
  data included; a few hits in data areas are noise.

## Licence

MIT — see [LICENSE](LICENSE). saturnkit contains no game data and no Sega
code; it reads and replaces, it does not include. The runtime includes
Musashi (the 68000, MIT) with SoftFloat, and its SCSP is in part derived
from MAME's (BSD-3-Clause): their terms are in
[THIRD_PARTY.md](THIRD_PARTY.md).

saturnkit has been proven on three games so far (the ports table above); its interfaces will
still change as the next one asks things of it.
