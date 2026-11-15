# AlphaStation 200/400 ("Avanti") emulation plan

**Goal:** a PCI Alpha that **Tru64 UNIX V5.1A and V5.1B** support, running
the real SRM console. The DEC 3000 runs Tru64 up to V5.1, but 5.1A drops
TURBOchannel. The Avanti family (AlphaStation 200, 250, 255 and 400) is the
closest PCI machine to what already works: an EV4 CPU, and PC-style I/O
that is mostly emulated already for the Jensen.

Statements marked *(verify)* are from memory or from other operating
systems' sources, not yet checked against the firmware or the hardware.

---

## 1. Why this machine

Both 5.1A (Rev. 1885) and 5.1B (Rev. 2650) GENERIC kernels still have the
same platform modules (5.1B adds Marvel): `apecs.c`, `lca.c`, `kn20aa.c`,
`eb164.c`, `eb64p.c`, `eb66.c`, `titan.c`, `ev6_common.c`, `kn300.c`,
`kn8ae.c`, ... and the platform names `DEC2100_A50` (Avanti), `DEC1000`,
`DECAXPPCI_33`, `ALPHAPC64`, `EB66PLUS`, `ALPHABOOK1`, `KN20AA`, `EB164`.
The 5.1A release notes retire only the XMI bus (and TURBOchannel, by
leaving it out).

The systems differ most in their CPU, and with the real SRM the CPU's
IPRs and PALcode environment must be exact:

| CPU | Systems | Work |
|-----|---------|------|
| EV4/EV45 (21064/21064A) | **AlphaStation 200/250/255/400**, AlphaPC64, EB64+, AlphaServer 1000 (APECS); AXPpci33, EB66+, AlphaBook 1 (21066 LCA) | EV4 runs SRM, OSF/1, Tru64 5.1 and NT already |
| EV5/EV56 (21164) | AlphaStation 500/600/600A, PWS au, EB164, AlphaServer 1000A/800/1200/4100 | new IPRs, new PAL environment: a CPU project |
| EV6 (21264) | DS10/20/25, XP1000, ES40/45 | more so, plus Tsunami/Titan |

Among the EV4 systems, the Avanti is the mainstream workstation with the
most archived firmware. The AXPpci33 (LCA, `dev_lca.c` exists) is a fallback
if Avanti firmware can't be found. The AlphaServer 1000 has EISA and would
need the ECU, as the Jensen did.

**Model:** start with an **AlphaStation 200 4/166 or 400 4/166** (21064,
EV4). The 4/233 and 4/266, and the 255, are 21064A (EV45): bigger caches
and a few ABOX/BIU differences, a small step later.

---

## 2. System architecture

```
          +---------------------------+
          |   21064 (EV4) / 21064A    |
          +-------------+-------------+
                        | 128-bit system bus
          +-------------+--------------------------+
          |                                        |
 +--------+---------+                  +-----------+-----------+
 | 21071-CA         |                  | 21071-DA "EPIC"       |
 | "COMANCHE"       |                  | PCI bridge: sparse/   |
 | memory controller|                  | dense windows, HAXR,  |
 | (+ 21071-BA data |                  | DMA windows and the   |
 |  path slices)    |                  | scatter-gather TLB    |
 +------------------+                  +-----------+-----------+
                                                   | PCI (32-bit, 33 MHz)
      +---------------+---------------+------------+-----+-----------+
      |               |               |                  |           |
 +----+-----+   +-----+-----+   +-----+------+     +-----+----+ +----+----+
 | NCR      |   | 21040     |   | 82378ZB    |     | PCI slot | | PCI slot|
 | 53C810   |   | Tulip     |   | SIO        |     | (TGA,    | |         |
 | SCSI     |   | Ethernet  |   | PCI-to-ISA |     |  S3 ...) | |         |
 +----------+   +-----------+   +-----+------+     +----------+ +---------+
                                      | ISA
     +--------------------------------+---------------------------+
     | SIO: 2x 8259, 8254, 2x 8237 DMA, PIRQ routing, X-bus        |
     | Super I/O (PC87312): 2x 16550, LPT, 82077 floppy            |
     | 8242 keyboard and mouse, MC146818 RTC, 1 MB flash (SRM+ARC) |
     +-------------------------------------------------------------+
```

### Physical address map (as the DROM and SRM use it; NetBSD `apecsreg.h`)

| Address | |
|---------|--|
| 0x0_0000_0000 | memory |
| 0x1_8000_0000 | COMANCHE (memory controller) CSRs |
| 0x1_A000_0000 | EPIC (PCI bridge) CSRs |
| 0x1_B000_0000 | PCI interrupt acknowledge / special cycle |
| 0x1_C000_0000 | PCI sparse I/O space |
| 0x1_E000_0000 | PCI configuration space (type 0/1 by HAXR2) |
| 0x2_0000_0000 | PCI sparse memory space (HAXR1 supplies the high bits) |
| 0x3_0000_0000 | PCI dense memory space |

Sparse space works as on the Jensen (address bits 5+ carry the PCI
address, bits 3..4 the length); the Jensen's sparse decoding is a model.

### Interrupts

The PCI interrupts are routed through the SIO's PIRQ route control
registers (config 60h..63h) into the 8259s, and the 8259 INTR goes to one
EV4 IRQ pin; NetBSD's `dec_2100_a50.c` and Linux's `sys_avanti` have the
slot and device numbers and the PIRQ assignments (verify, and see what the
SRM writes). The 8254 timer is the system clock (IRQ 0). Error interrupts
(COMANCHE/EPIC) come on another pin (verify).

---

## 3. What exists, what is new

**Reuse:**
- EV4 CPU: SRM, OSF/1, Digital UNIX, Tru64 5.1 and NT 3.1 run on it.
  Tru64 5.x fixes (WH64/ECB, SCSI VPD pages, IEEE compares) are in.
- 8259 PICs, 8254, 8237 DMA (`dev_isadma.c`, with the 82357 extensions
  the SIO doesn't have), 82077 floppy (`dev_fdc.c`), 8242 keyboard and
  PS/2 mouse (`dev_pckbc.c`), MC146818 (`dev_mc146818.c`), 16550
  (`dev_ns16550.c`), all from the Jensen, wired up through `bus_isa_init()`.
- `bus_pci.c`: PCI configuration space, and a stub for the 82378ZB.
- `dev_osiop.c`: a SCRIPTS processor for the NCR 53C710, the 53C810's
  predecessor.
- `dev_dec21143.c`: a Tulip (21143); Tru64 also supports the DE500
  (21143) in a slot.
- `dev_vga.c` and the QVision's VGA core, if a graphics card comes later.
- The Jensen's SROM emulation and flash handling (`dev_jensen_srom()`,
  `<nvram file>.flash`) as a model.

**New:**
1. `machine_avanti.c` and `dev_apecs.c`: COMANCHE and EPIC CSRs (memory
   configuration, error and status registers that must read clean, HAXR
   registers, the DMA windows (WBASE/WMASK/TBASE) and the scatter-gather
   TLB, which Tru64 uses for DMA), the sparse and dense windows, config
   cycles.
2. The 82378ZB SIO's own registers: PIRQ routing, ISA clock divisor,
   X-bus chip selects (which enable the RTC, keyboard, flash, ...), and
   its 8237 without the EISA extensions.
3. The Super I/O's configuration registers (index/data at 398h (verify)).
4. The flash (four 28F020), programmable from SRM, kept in a file as
   on the Jensen.
5. **NCR 53C810** (PCI 1000:0001): the biggest piece. The register layout
   differs from the 710, and the SCRIPTS set adds things (memory move,
   table indirect, ...). SRM's driver and Tru64's `psiop` are both SCRIPTS
   based. Write it from the Symbios 53C810 data manual, using
   `dev_osiop.c` as the model for the SCRIPTS engine and the SCSI phases.
6. Later: the 21040 (1011:0002), or a 21143 in a slot; graphics (TGA or
   S3) if wanted. SRM and Tru64 run on the serial console without them.

---

## 4. Firmware

In `~/src/alphafw` (not in the repository):

**ROM dumps of two AlphaStation 200 boards** (TL866II, 2019), the 1 MB
flash on four Intel 28F020 (256 KB each) and a 64 KB TMS27PC512:

| Chip | `avanti/` | `as200/` |
|------|-----------|----------|
| E13 + E19 (flash) | SRM V7.0-9 | SRM V6.4-1 |
| E33 + E46 (flash) | ARC 4.58 | ARC 4.49 |
| E47 (EPROM, "Diagnostic ROM") | DROM V1.7 | DROM V1.2 |

Every image starts with the same 64-byte header: `01 01 00 00 55 00 aa
ff`, the version (8 bytes), "DEC", "AVANTI", "ALPH", the image length at
0x2c (SRM V7.0-9: 0x66c00, so it runs on from E13 into E19; the second
chip of each pair is a continuation, not interleaved, and E46 is mostly
erased), the name ("AVANTI_SRM", "AVANTI_ARC", "AVANTI-DROM") at 0x30, and
`44 33 22 11`. The SRM and ARC images are compressed (no strings after the
header); the DROM is code (it builds its messages in registers). So
presumably: the SROM loads the DROM, which runs the power-up tests and
picks, decompresses and starts SRM or ARC from the flash (verify by
disassembling the DROM). Emulating only the SROM, and running the real
DROM, keeps the console's own view of the hardware intact, as on the
Jensen (`dev_jensen_srom()`).

**Firmware Update CDs** V3.6 to V7.3. The newest for the AlphaStation
200/250 is `/ALPHA200/AS200_V7_0.EXE` (SRM V7.0, ARC 4.58, OSF PALcode
1.46-2), the same release as the V7.0-9 dump: the dump is current. The
AlphaStation 400 has its own line, up to `/ALPHA400/AS400_V72_1.EXE` (SRM
V7.2-1; its onboard Ethernet is a 21143 on some boards), and the 255 has
`/ALPHA255/AS255_V8_1.EXE` (SRM V7.0, AlphaBIOS 5.68). The `.EXE` files
are updaters with the images packed differently (the dump's image isn't
in them byte for byte); not needed for the AlphaStation 200.

**Target: AlphaStation 200 4/166 (21064) with the V7.0-9 dump.** On the
DEC 3000, Tru64 5.x needed SRM V7.0 (V3.3 closed all channels at the first
console call after the virtual remap); V7.0 is what we have here. ARC 4.58
in the same flash means Windows NT can come later for free.

Before writing code, disassemble the DROM: what the SROM must have set up
(PAL base, registers, what's in memory), which CSRs and SIO registers it
touches first, how it reads the model straps and finds the flash, and how
it decompresses the console.

---

## 5. Milestones

### M0: material
- DROM disassembled, the flash layout and the console decompression
  understood (see above).
- Tru64 5.1A and 5.1B install CDs (on hand), a NetBSD/alpha and a Linux
  (Debian 3.x/4.0 alpha) CD as stepping stones: both support the Avanti
  and have readable drivers for APECS, SIO and the 53C810.

### M1: SRM prints on the serial port
- `machine_avanti.c`: memory, the SROM's work (load the DROM, PAL base,
  registers), the flash (E13/E19/E33/E46 concatenated, 1 MB) and the DROM
  (E47) where the DROM expects them, APECS CSRs that read back,
  the SIO and the ISA devices, 16550 COM1.
- `apecs` and `sio` log subsystems; unknown CSR accesses logged.
- Done when the banner and `>>>` appear on tty0.

### M2: a clean console
- `show config`, `show device`, `test` (or the power-up tests) pass;
  PCI probe finds the SIO; RTC, NVRAM/flash environment variables
  persist across restarts (`set auto_action`, `set bootdef_dev`).
- Interrupt routing checked against what SRM programs.

### M3: SCSI
- 53C810 enough for SRM: `show device` lists `dka0`, `dka400`; `boot
  dka400` reads the CD's boot block and loads a kernel.

### M4: an OS kernel boots
- NetBSD/alpha or Linux to multi-user on the 53C810: exercises interrupts,
  DMA windows, the 8254 clock, and SCRIPTS with disconnect/reselect.

### M5: Tru64 UNIX V5.1B
- Install from CD to an RZ-type disk (AdvFS, all subsets), boot multi-user.
  Expect the sort of fixes the DEC 3000 needed: CSRs Tru64 reads at boot
  (`apecs.c` checks error registers and the memory configuration), the
  scatter-gather TLB, machine checks that must not happen.
- Then V5.1A, and the 5.1B-4 patch kit.

### M6: extras
- Network (21040 onboard, or a DE500), floppy, the EV45 models,
  graphics with CDE.

---

## 6. Status (2026-11-14)

**M1 done, M2 mostly:** the real DROM V1.7 runs all its power-up tests
and starts SRM V7.0-9 from the flash, which comes up to `>>>` on COM1;
`show config` shows the 21064-3 at 166 MHz, 64 MB, the NCR 810 in PCI slot
6 (`pka0`) and the SIO in slot 7, and the ISA devices. Run it with
`"machine": { "type": "avanti" }` and the ROM file (the five chip dumps
concatenated, `cat E13 E19 E33 E46 E47`).

What it took (`dev_avanti.c`, `dev_apecs.c`):

- **The serial ROM handoff:** the DROM's code (after its header) in memory
  at 8000, which is also the PAL base, started in PALmode; PAL_TEMP14 and
  PAL_TEMP18 hold ABOX_CTL and BIU_CTL; 2 at physical 70 makes the DROM
  time the CPU (166 MHz).
- **The ROM bus** at 1.0000.0000 (address latch, `address << 8`) and
  1.0010.0000 (data byte); address bit 23 set for a write. Flash at 0,
  "NVRAM on WART" at 10.0000 (8 KB so far; the SRM's ISA configuration
  table), progress display at 14.0000, jumpers at 1C.0000 (bit 0: boot the
  console, bit 1: clear runs a B-cache test and the floppy loader).
- **The DROM's tests** (its strings, decoded from the code, are in its
  monitor's command list): memory, flash ID, WART NVRAM, 53C810 (SCRATCHB
  through configuration space 0xdc: the 810 mirrors its operating
  registers at 80h..FFh), 82378 (configuration register 45h), "3287", 8242,
  87312, interrupts (the RTC's periodic interrupt on **EV4 irq_h[4]**, a
  line of its own).
- **PCI configuration addresses** are `device << 11 | function << 8 | reg`
  for bus 0 too (the EPIC drives IDSEL); reads return the whole longword.
- **COMANCHE banksets** set up as the serial ROM does (the SRM sizes memory
  from them; with none, it machine checks past the end of memory): base
  address and configuration registers from the 21071 data sheet. COMANCHE
  ED and EPIC DCSR report pass 2.
- **8254:** counters 0 and 2 count in host time, on this machine only (the
  DROM times the CPU with counter 0; on the Jensen, counting makes ARC find
  its EISA configuration inconsistent).
- **EV4:** a TB entry read after ITBZAP reads as zero (the console PALcode
  tells a pass 1 21064 from later ones that way, and halts on a pass 1);
  SL_XMIT is decoded into characters (the `srom` log subsystem), which shows
  what the DROM prints, e.g. its `DIAGROM>` monitor.

**M3 done, M4 started (2026-11-14):** the 53C810 works (`dev_sym53c810.c`):
SRM's `show device` lists `dka0` and `dka400` with their names, `boot dka0`
boots, and Tru64 V5.1's `genvmunix` comes up to the single-user prompt with
`scsi0 at psiop0`, `ace0`/`ace1`, `lp0`, `fdi0`/`fd0` (console on `ace0`).

- **53C810:** the operating registers in both BARs (the BARs as programmed;
  Tru64 uses the memory BAR), the SCRIPTS processor (block moves, I/O,
  register moves, transfer control, memory moves, LOAD/STORE), and a SCSI
  bus with the disk images as targets: SDTR is accepted as asked, WDTR
  answered with 8 bits, only LUN 0 exists, nothing ever disconnects, so
  WAIT RESELECT waits for SIGP. SCRIPTS (SRM's and Tru64's) and data go
  through the DMA windows; Tru64's 810 SCRIPTS load and save registers with
  memory moves to the chip's own memory BAR.
- **DMA windows** in the EPIC: direct mapped and scatter/gather.
- **The SIO's class code reads 0** (the real 82378ZB predates class codes).
  SRM reads the revision ID with a byte read (configuration reads return
  the whole longword) and compares it with 88h; with the class code there,
  it left window 2 at PCI 0 instead of 1 GB at 1 GB, and Tru64, which
  reserves [base, 2 × base) for it, panicked in `rmget`.
- **Interrupts:** the OSF PALcode (UNIX V1.46) dispatches irq_h[0] (EPIC
  errors), **irq_h[2] (the SIO: it reads ISA port 26h, an EB64+ register
  that floats here, then the interrupt acknowledge, vector 800h + 10h *
  IRQ)**, irq_h[3] (NMI), irq_h[4] (RTC), irq_h[5] (halt), and ignores
  irq_h[1]. PCI interrupts go through the SIO's PIRQ route registers; the
  810 is on PIRQ3 (SRM routes it to IRQ 11, `control_irq11 scsi`).
- **8259s** latch requests that arrive while in service (as on the
  Jensen): Tru64's floppy driver starts a seek before the EOI of the last
  interrupt.
- SCSI is the default drive bus; INQUIRY says SCSI-2 (response data format
  2), or SRM shows no vendor and product.

**M5 done (2026-11-14): Tru64 UNIX V5.1B installs from CD and runs
multi-user.** With the 5.1B OS CD as `dka400` and a blank RZ29B-sized
image (8380080 sectors) as `dka0`: `boot dka400`, U.S. English, all
software (115 subsets), mandatory kernel options, default layout on
AdvFS. The subsets load in about 35 minutes; the installer reboots through
SRM on its own (`boot dka0 -flags A`), configures the subsets, builds a
kernel for the machine (`cpu "DEC2100_A50"`), reboots again and comes up
to `login:`. `hwmgr` sees dsk0 (RZ29B), cdrom0 (RRD45) and floppy0. M4 (a
NetBSD or Linux kernel) was skipped: Tru64 came first.

- **The TOY clock:** Tru64's readtodr() for these machines wants register
  A's divider at 32 kHz, register B binary and 24-hour (SRM sets it so),
  and counts the year register from 1952 (it adds 52); anything before
  1996 is a "preposterous time in TOY clock". The Avanti's RTC holds the
  year minus 1952 (2026 is 74). The ARC console counts from 1980; see
  below.
- **The preset** "avanti", "200" (256 MB, `avanti.rom`, a disk at ID 0
  and a CD-ROM at ID 4, the serial console on telnet), DEC's drive models
  (now with the RRD45), and no PROM emulation checkbox for it.

**Network (2026-11-14):** first with a DE500-BA (21143, `dev_dec21143.c`)
at device 11; now the on-board 21040 is there (see below). SRM shows it as
`ewa0`, Tru64 V5.1B as `tu0`; after
a `doconfig` (the installed kernel was sized without it), `rcmgr`
NETDEV_0/IFCONFIG_0 and `/etc/routes`, it comes up at boot: ping, DNS
through the simulated gateway (`domain` and `nameserver 10.0.0.254` in
`/etc/resolv.conf`, `hosts=local,bind`), TCP.

- **The 21143 on a real PCI bus:** BARs as the firmware programs them
  (128 bytes each), DMA through the EPIC's windows, a version 3 SROM as on
  a DE500-BA (SRM V7.0 misreads version 4; extended media block lengths
  don't count the length byte) with subsystem ID 1011:500B.
- **21143 fixes for every machine:** a setup frame's descriptor gets only
  TDES0 = 7FFFFFFFh written back (the other words were overwritten with
  ones, wrecking Tru64's ring); CSR5 shows the transmit and receive
  process states (TS, RS); NIS/AIS summarize only the interrupts enabled
  in CSR7 (Tru64 leaves TU masked and ends its interrupt handler with
  poll demands, which set TU again: an interrupt storm otherwise); the
  transmit threshold bits are accepted.
- **APECS sparse space:** aligned word and longword cycles to PCI devices
  are one access (the 21143's CSRs are longwords); below 1000h (I/O) and
  1 MB (memory) they are still byte cycles, for the ISA devices behind the
  SIO.
- **CMOS:** bytes 0Eh..3Fh are the RTC's battery-backed RAM, kept in the
  NVRAM file (they were lost at every start). SRM keeps `quick_start` in
  3Eh (31h = ON), the DROM the console choice in 3Fh.
- **SRM's console heap** is sized from the memory size that the serial
  ROM leaves at physical 8 (and passes on through the DROM): with 25 MB or
  more, the heap reaches up to 2 MB (928 KB here); with 0 (as the SROM
  emulation left it at first), it is 192 KB, and the `ew` driver's
  receive buffers didn't fit ("insufficient dynamic memory for a request
  of 1536 bytes" in `rx_ewa0`, and a restart, forever), nor did booting
  with both cards. The SROM emulation now leaves the memory size there.
- **gxemul's NAT** (not the card): its TCP was stop-and-wait, and long
  downloads stalled near the end (1.6-2 MB of 2 MB, on the DEC 3000's
  LANCE too). Fixed 2026-11-15, see "The NAT's TCP" below.

**Graphics (2026-11-14): CDE runs.** With the display on, a ZLXp-E1 (a
21030 "TGA" with 2 MB, 8 planes, and a Bt485: `dev_tga.c`) is in the
second slot (device 12, PIRQ1). SRM lists "Digital ZLXp Graphics
Controller" (and sets up the video and the Bt485 itself; its graphics
console would run the card's x86 option ROM, which we don't have, so the
console stays serial). Tru64 V5.1B attaches `tga0` ("depth 8, map size 4MB,
1280x1024, ZLXp-E"); after a `doconfig`, dtlogin comes up on it, and a
CDE session works with the keyboard and mouse (8242).

- **The 21030** (from DEC's reference manual, EC-N0683-72): the 128 MB BAR
  of 4 MB core spaces (alternate ROM space, registers every 512 bytes,
  the frame buffer), simple, opaque/transparent/block stipple,
  opaque/transparent/block fill, opaque/transparent line (with the slope
  registers' Bresenham setup and continuation) and copy mode (and the
  copy-64 registers), the plane mask, raster operations, the one-shot and
  persistent pixel mask, video timing and base, the 21030's own cursor,
  end-of-frame and timer interrupts, and the Bt485 behind EPSR/EPDR
  (palette, cursor colors, 32/64 pixel cursor). Not there: the DMA and 3D
  modes (logged when used; CDE hasn't), the "odd" 4 pixels (SRM's
  1284-pixel timing shows them).
- **8242 keyboard:** Tru64 uses scancode set 3 with translation off; the
  PC-style controller sent set 1 whenever any set was chosen. Now the
  translate bit in its command byte decides. Reading the output buffer
  drops the interrupt, and the next byte raises it again (Tru64 reads one
  byte per interrupt: a line that stayed up never interrupted again, and
  keys "stuck"). Typed text goes in at about 30 keys a second (a burst
  overflowed Tru64's queue and lost a Shift release). Windows NT on the
  Jensen still types correctly.

**The on-board 21040 (2026-11-14).** DEC's "AlphaStation 200/400 Series
Technical Information" (EK-PCDSA-TI, on NetBSD's dec-docs) gives the
riser card's PCI devices: the 53C810 at IDSEL AD17 (device 6), the SIO at
AD18 (7), the 21040 at AD22 (device 11, PIRQ0), slot 1 at AD23 (device
12; INTA..D on PIRQ1, 0, 2, 1) and slot 2 at AD24 (device 13; PIRQ2, 1,
0, 2). So the 21040 is always at device 11 (unconnected without a
network), the TGA is in slot 1, and a DE500-BA would go in a slot. The
21040 is a variant of the 21143 model: PCI ID 1011:0002, pass 2.4, no
MII, and its Ethernet address ROM is read a byte at a time through CSR9
(a write starts it over), in DEC's 32-byte format (address, checksum,
reversed, again, test pattern). SRM: "DECchip 21040 Network
Controller"; Tru64: "tu0: DECchip 21040: Revision: 2.4 ... DEC TULIP
(10Mbps)", ping, DNS, a 300 KB TCP download intact.

**The ARC console (2026-11-14).** With `set os_type NT` in SRM, CMOS
byte 3Fh becomes 1, and at the next power-up the DROM starts ARC 4.58
instead: "ARC Multiboot Alpha AXP Version 4.58" on the TGA at 640x480,
21064 rev 3 at 166.66 MHz, 256 MB, "8 Plane Frame Buffer video card:
ZLXp-E1". ARC's Setup menu, "Switch to OpenVMS or Digital UNIX
console", "Switch to Digital UNIX" goes back: it writes 3 to CMOS 3Fh,
and at the next power-up (a restart of gxemul) the DROM starts SRM again
(checked 2026-11-15). What it took:

- **PCI BARs:** ARC writes all ones to all six BARs, then reads the ID
  register, then the BARs. A BAR now reads as its size until it is
  written again (this used to end at the next read of anything), and on
  this machine the BARs a device doesn't declare read as 0 when sized.
  Before, they claimed 1 MB, and ARC disabled the devices with BARs it
  couldn't place.
- **The TOY clock:** register B is battery-backed and starts binary; ARC
  reads the time as binary without checking. With CMOS 3Fh = 1, the year
  register counts from 1980 (2026 is 46), as ARC and NT expect.
- **ARC's environment** (CONSOLEIN, SYSTEMPARTITION, LANG_ID, ...) is in
  the WART NVRAM at 10.0000, kept in `<nvram file>.wart`. "Supplementary
  menu, and save changes" writes it and restarts. With an empty
  environment, ARC asks for the language at every power-up and wants
  "Set default environment variables" and then "Set default
  configuration" before it will boot NT.
- **The CPU speed:** ARC's PALcode takes the cycle time the DROM passes
  in a2 only if the serial ROM's signature is in a3 (0xdeca, 0xdecb or
  0xdecc and up in bits 31:16, the system revision in 15:0). Without one
  it assumes 8000 ps, and shows 125 MHz. The handoff now has 0xdeca0000
  at 18 (the DROM passes 0..48 on in a0..).
- **8254:** a reload value of 0 means 65536 (18.2 Hz), not "stopped".
  ARC programs counter 0 so (on all machines).
- **RTC periodic interrupts** are counted in host time from the device's
  tick, so they come at their real times. timer.c's 65 Hz signal
  delivered 1024 Hz in bursts of 16, and SRM, which counts them against
  the cycle counter, measured 158 MHz (the HWRPB, and Tru64's `psrinfo`).
  Now it measures 166666666 Hz, and Tru64 says 167 MHz. The catch-up of
  up to a second's backlog stays. Checked: Tru64 V5.1B (its clock keeps
  host time), OSF/1 V3.0 on the DEC 3000, NT 3.1 on the Jensen, and
  Ultrix 4.5 on a 5000/200.

No Windows NT for Alpha with AlphaStation 200 support is here: only NT
3.1, which predates the machine.

### Tru64 V5.1A (2026-11-14)

Installed with no emulator changes, from the V5.1A OS CD (Rev. 1885) to
an RZ29B over the serial console: `boot dka400`, all software, mandatory
kernel options, AdvFS root and /usr (/var in /usr). The 100 subsets load
in about 50 minutes. Unlike V5.1B, the installer builds the kernel
(`cpu "DEC2100_A50"`) in its first reboot by itself, with no manual
`boot dka0 -flags A`, and comes up to `login:`. `psrinfo` says 167 MHz
EV4, the clock keeps host time, and tu0 (the on-board 21040) pings the
NAT gateway, also after a reboot with the address in rc.config.

### The EV45 models: AlphaStation 400 4/233, 200 4/233, 250 4/266 (2026-11-15)

- **The model is the serial ROM's code at 0x70.** The DROM (at 0xa8) times
  the CPU only for codes 2 and 8, and replaces the code from the speed:
  for 2, 9 = 100 MHz, 3 = 133, 2 = 166, 99 = 200, 4 = 233, 11 = 266,
  10 = 300; for 8, 8 = 266, 12 = 300. With it, it stores the cycle time
  (ps) at 0x10, the MHz at 0x68 and BIU_CTL's B-cache fields in
  PAL_TEMP18. Any other code it takes as it is, with those values. The
  console makes the HWRPB's system variation `code << 10`, and names the
  machine from it: 1, 5, 6, 7 "AlphaStation 400", 2, 3, 4, 9 "200", 8
  "250", 0 "Unknown System type", with the MHz from 0x68. (NetBSD's rpb.h
  has 0x2000, code 8, as a 400 4/266; SRM V7.0 calls it the 250.) The
  400 4/233 passes code 5 (NetBSD's 0x1400), 4291 ps, 233 and 0xf460,
  which is what the DROM picks for 233 MHz.
- **21064A:** the console PALcode maps DC_STAT's CHIP_ID (bits 2:0) to
  the HWRPB's CPU type and pass: 0 a 21064 pass 2, 7 pass 3, 1 a 21064A
  pass 1, 3 pass 2, 5 pass 0 (table at 0xc340). The new CPU type
  "21064A" (PCS type 6) reads 3: SRM shows "21064A-2", Tru64 "EV4.5".
  Nothing else differs in the emulation (no caches are modelled).
- **The 400's PCI layout:** three slots at devices 11, 12 and 13, no
  21040. With a network, a DE500-BA is in device 11 (PIRQ0); the TGA is
  in device 12, as on the 200.
- Models: `200` (4/166, 21064), `200-233`, `400` (4/233) and `250`
  (4/266, 2 MB cache, the 200's layout), all 21064A but the first. The
  400 has a preset.
- **Checked:** the installed V5.1A disk (through an overlay) boots on the
  400 4/233: "AlphaStation 400 4/233", `psrinfo` "EV4.5 (21064) ... 234
  MHz" (the HWRPB says 233100233 Hz; Tru64 rounds up, as 167 for the
  166), tu0 a 21143 in slot 11, ping through the gateway, the clock in
  step with the host. The 250 and the 200 4/233 come up in SRM with their
  names and speeds.

**TIMER_TPS (2026-11-15).** SRM's "TIMER_TPS environment variable
invalid / Defaulting to 1mS per tick" at every start: it keeps TIMER_TPS
in CMOS byte 4Fh, which our RTC (50 battery-backed bytes, 0Eh..3Fh)
didn't keep. The Avanti's RTC now has 128 bytes; 40h..7Fh go in the NVRAM
file after the clock, as a "GXRTC128" block (older files still load).

**The NAT's TCP (2026-11-15).** Data from the outside went to the guest
one 1400-byte segment at a time, the next only after the guest's ACK
(which a guest delays), and a lost segment waited 10000 polling rounds:
Tru64 V5.1A got 7 KB/s, and a long download eventually reset. Now
`net_ip.c` keeps up to 64 KB from the outside socket and sends what the
guest's window allows (with its window scale, from its SYN); the guest's
ACKs drop data from the buffer, three duplicates or 300 ms of host time
without progress send again from the first unacknowledged byte, and the
FIN goes after the last byte is acknowledged, again until it is. The
other way, the data in the guest's FIN segment (the last of an upload)
was dropped; it is now sent on. Tru64 V5.1A on the 400 (DE500): 200 MB
from the host in 13 s, every byte right, 40 MB to it in 4 s, and an HTTP
GET from the Internet. OSF/1 V3.0 on the DEC 3000 (LANCE): 20 MB in 16 s
with telnet, all there.

**Windows NT 4.0 (2026-11-15).** From the NT Server 4.0 CD (the
multi-architecture one, NTSRV40A), under ARC 4.58: `arcinst` from the CD
("Run a program", `scsi(0)cdrom(4)fdisk(0)\alpha\arcinst`) makes a 10 MB
FAT system partition; "Install Windows NT from CD-ROM" then starts setup.
It hung at first, in a firmware routine that times the CPU with the 8254's
counter 2 (mode 3, gated through port 61h) and waits for edges of its
output in port 61h bit 5, which was always 1. Counter 2's output and gate
now follow its count (dev_8253.c, from dev_pckbc.c's port 61h). Text-mode
setup finds "Digital AlphaStation 200/400 Family", the ZLXp-E (TGA) and
the C810 (symc810), partitions and copies in under two minutes, restarts
through ARC, and the graphical setup runs on NT's TGA driver.

**Next:** the TGA's DMA modes if something uses them; Windows NT, when a
version that supports the Avanti turns up.

## 7. Risks
- **Firmware:** on hand (V7.0-9 and V6.4-1 dumps), but the DROM and the
  compressed console are a two-stage start to get through before any
  console output.
- **53C810:** SCRIPTS must be right in the corners (disconnects, phase
  mismatch, interrupts on the fly), and Tru64's `psiop` uses them.
- **DMA:** EPIC's scatter-gather windows must translate exactly, or disks
  read garbage, quietly.
- **Undocumented CSR expectations:** SRM and Tru64 both check COMANCHE and
  EPIC error registers and the memory configuration registers; the 21071
  data sheet (DECchip 21071 and 21072 Core Logic Chipsets Data Sheet) is
  the reference.
