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
     | Super I/O (PC87312 (verify)): 2x 16550, LPT, 82077 floppy   |
     | 8242 keyboard and mouse, MC146818 RTC, 1 MB flash (SRM+ARC) |
     +-------------------------------------------------------------+
```

### Physical address map (from Linux `core_apecs.h` (verify))

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
4. The flash (28F008SA (verify)), programmable from SRM, kept in a file as
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

## 6. Risks
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
