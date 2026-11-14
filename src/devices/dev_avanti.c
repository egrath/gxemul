/*
 *  Copyright (C) 2026  Rico Pajarola.  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. The name of the author may not be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 *  OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 *  OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 *  SUCH DAMAGE.
 *
 *
 *  COMMENT: AlphaStation 200/400 ("Avanti") system board
 *
 *  An EV4 with the APECS chipset (dev_apecs.c), an 82378ZB SIO PCI-to-ISA
 *  bridge with the usual PC devices behind it, and a "ROM bus":
 *
 *	1.0000.0000	address latch: a longword write of (address << 8)
 *	1.0010.0000	data: the byte at the latched address
 *
 *  On the ROM bus (worked out from the DROM V1.7), where address bit 23
 *  is set to write:
 *
 *	00.0000		1 MB flash, four Intel 28F020 (256 KB each): the
 *			SRM console at 0 (E13, E19), ARC at 8.0000 (E33,
 *			E46). Each image starts with a 0x40-byte header.
 *	10.0000		NVRAM on the "WART" chip, 8 KB so far (the DROM's
 *			"NVRAM Test" writes 0xaa and 0x55 to its first byte;
 *			the SRM console keeps its ISA configuration table in
 *			it). With an NVRAM file, it is kept in
 *			"<nvram file>.wart".
 *	14.0000		progress code display (write)
 *	1C.0000		configuration jumpers (read): bit 0 set boots the
 *			console (clear: the DROM's debug monitor), bit 1
 *			(clear: a path that ends in beep code 3-2-3)
 *
 *  The serial ROM loads the "DROM" (the 64 KB E47 EPROM) into memory and
 *  starts it in PALmode; the DROM runs the power-up tests, copies the
 *  console image chosen by CMOS byte 0x3f (1: ARC) from the flash to
 *  10.0000 (the header just below) and starts it in PALmode. See
 *  doc/avanti-plan.md.
 *
 *  PCI interrupts go to the SIO's PIRQ0..3 inputs, which its PIRQ route
 *  control registers (configuration space 60h..63h) send to ISA IRQs; the
 *  on-board 53C810 is on PIRQ3 (as in NetBSD's pci_2100_a50.c).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "bus_isa.h"
#include "bus_pci.h"
#include "cpu.h"
#include "cpu_alpha.h"
#include "device.h"
#include "devices.h"
#include "emul.h"
#include "interrupt.h"
#include "machine.h"
#include "memory.h"
#include "misc.h"


#define	AVANTI_ROMBUS_BASE	0x100000000ULL
#define	AVANTI_ROMBUS_SIZE	0x200000ULL
#define	AVANTI_ROMBUS_DATA	0x100000ULL

#define	AVANTI_FLASH_SIZE	(1024 * 1024)
#define	AVANTI_FLASH_CHIP_SIZE	(256 * 1024)
#define	AVANTI_FLASH_CHIPS	4
#define	AVANTI_DROM_SIZE	(64 * 1024)
#define	AVANTI_ROM_HEADER_SIZE	0x40

#define	AVANTI_ROMBUS_WRITE	0x800000
#define	AVANTI_ROMBUS_NVRAM	0x100000
#define	AVANTI_NVRAM_SIZE	8192
#define	AVANTI_ROMBUS_LED	0x140000
#define	AVANTI_ROMBUS_JUMPERS	0x1c0000

/*  Jumpers: boot the console from the flash.  */
#define	AVANTI_JUMPERS_DEFAULT	0x03

/*  Where the emulated PCI/ISA devices are (bus address + this):  */
#define	AVANTI_IO_BASE		0x10000000000ULL
#define	AVANTI_MEM_BASE		0x20000000000ULL

/*
 *  EV4 interrupt lines, as the OSF PALcode (UNIX V1.46) dispatches them:
 *
 *	irq_h[0]	EPIC (PCI) errors
 *	irq_h[2]	the SIO's 8259s: the PALcode reads ISA ports 26h/27h
 *			(an EB64+ interrupt register; nothing answers here,
 *			so bit 10 reads as set, "ISA"), then the interrupt
 *			acknowledge at 1.B000.0000: vector 800h + 10h * IRQ
 *	irq_h[3]	NMI (port 61h)
 *	irq_h[4]	the RTC (also the DROM's interrupt test: "EV4 IRQ4
 *			Line Was Not Set")
 *	irq_h[5]	halt
 *
 *  irq_h[1] isn't looked at.
 */
#define	AVANTI_IRQ_SIO		2
#define	AVANTI_IRQ_RTC		4

/*  The DROM runs here (its PAL base); it copies the console to 10.0000.  */
#define	AVANTI_DROM_ADDR	0x8000

/*  PCI devices:  */
#define	AVANTI_PCI_DEV_SCSI	6
#define	AVANTI_PCI_DEV_SIO	7
#define	AVANTI_PCI_DEV_SLOT0	11	/*  slots: devices 11..13  */

#define	SIO_PIRQ_ROUTE		0x60	/*  4 bytes: bit 7 off, <3:0> IRQ  */
#define	SIO_PIRQ_DISABLE	0x80

/*  Intel 28F020 commands:  */
#define	F28_READ		0x00
#define	F28_READ_ID		0x90
#define	F28_ERASE		0x20
#define	F28_ERASE_VERIFY	0xa0
#define	F28_PROGRAM		0x40
#define	F28_PROGRAM_VERIFY	0xc0
#define	F28_RESET		0xff


struct avanti_data {
	struct machine		*machine;
	struct apecs_data	*apecs;
	uint64_t		ram_size;
	int			subsys;

	struct interrupt	irq_sio;
	struct interrupt	irq_rtc;

	/*  The ROM bus:  */
	uint32_t		latch;
	uint8_t			nvram[AVANTI_NVRAM_SIZE];
	char			*nvram_file;
	int			nvram_dirty;
	uint8_t			jumpers;
	uint8_t			led;

	unsigned char		flash[AVANTI_FLASH_SIZE];
	int			have_flash;
	uint8_t			flash_cmd[AVANTI_FLASH_CHIPS];
	int			flash_erase_setup[AVANTI_FLASH_CHIPS];

	unsigned char		drom[AVANTI_DROM_SIZE];
	int			have_drom;

	struct isadma_data	*dma;

	/*  PCI interrupts, through the SIO:  */
	char			*isa_irq_path;
	struct interrupt	isa_irq[16];
	int			isa_irq_connected[16];
	int			isa_irq_count[16];
	int			pirq_routed[4];	/*  the ISA IRQ, or -1  */

	/*  Flash contents are kept in this file, if set:  */
	char			*flash_file;
	int			flash_dirty;
	time_t			flash_changed;
};


/*
 *  Flash persistence, as on the Jensen: with an NVRAM file configured, the
 *  whole flash is kept in "<nvram file>.flash", and once that file exists
 *  it is used instead of the ROM image. The WART NVRAM is kept in
 *  "<nvram file>.wart" (the RTC's CMOS is in the NVRAM file itself).
 */
static struct avanti_data *avanti_flash_owner = NULL;

static void avanti_nvram_save(struct avanti_data *d)
{
	FILE *f;

	if (d->nvram_file == NULL || !d->nvram_dirty)
		return;

	f = fopen(d->nvram_file, "wb");
	if (f == NULL || fwrite(d->nvram, 1, AVANTI_NVRAM_SIZE, f) !=
	    AVANTI_NVRAM_SIZE)
		fatal("[ avanti: could not write %s ]\n", d->nvram_file);
	if (f != NULL)
		fclose(f);
	d->nvram_dirty = 0;
}

static void avanti_flash_save(struct avanti_data *d)
{
	FILE *f;

	avanti_nvram_save(d);

	if (d->flash_file == NULL || !d->flash_dirty)
		return;

	f = fopen(d->flash_file, "wb");
	if (f == NULL) {
		fatal("[ avanti: could not write the flash to %s ]\n",
		    d->flash_file);
		d->flash_file = NULL;
		return;
	}
	if (fwrite(d->flash, 1, AVANTI_FLASH_SIZE, f) != AVANTI_FLASH_SIZE)
		fatal("[ avanti: short write to %s ]\n", d->flash_file);
	fclose(f);
	d->flash_dirty = 0;
}

static void avanti_flash_save_atexit(void)
{
	if (avanti_flash_owner != NULL)
		avanti_flash_save(avanti_flash_owner);
}

DEVICE_TICK(avanti)
{
	struct avanti_data *d = (struct avanti_data *) extra;

	if ((d->flash_dirty || d->nvram_dirty) &&
	    time(NULL) > d->flash_changed + 1)
		avanti_flash_save(d);
}


/*
 *  One of the four 28F020s (bulk erase, byte program, commands written
 *  twice for erase and reset).
 */
static uint8_t avanti_flash_read(struct avanti_data *d, uint32_t addr)
{
	int chip = addr / AVANTI_FLASH_CHIP_SIZE;

	switch (d->flash_cmd[chip]) {
	case F28_READ_ID:
		return (addr & 1)? 0xbd : 0x89;
	default:
		return d->flash[addr];
	}
}

static void avanti_flash_write(struct avanti_data *d, uint32_t addr,
	uint8_t val)
{
	int chip = addr / AVANTI_FLASH_CHIP_SIZE;

	if (d->flash_cmd[chip] == F28_PROGRAM) {
		/*  Programming only clears bits.  */
		d->flash[addr] &= val;
		d->flash_cmd[chip] = F28_READ;
		d->flash_dirty = 1;
		d->flash_changed = time(NULL);
		return;
	}

	if (d->flash_erase_setup[chip]) {
		d->flash_erase_setup[chip] = 0;
		if (val == F28_ERASE) {
			memset(d->flash + chip * AVANTI_FLASH_CHIP_SIZE, 0xff,
			    AVANTI_FLASH_CHIP_SIZE);
			d->flash_dirty = 1;
			d->flash_changed = time(NULL);
			debugmsg(d->subsys, "avanti", VERBOSITY_INFO,
			    "flash chip %i erased\n", chip);
			d->flash_cmd[chip] = F28_READ;
			return;
		}
	}

	switch (val) {
	case F28_ERASE:
		d->flash_erase_setup[chip] = 1;
		break;
	case F28_READ_ID:
	case F28_PROGRAM:
		d->flash_cmd[chip] = val;
		break;
	case F28_READ:
	case F28_ERASE_VERIFY:
	case F28_PROGRAM_VERIFY:
	case F28_RESET:
		d->flash_cmd[chip] = F28_READ;
		break;
	default:
		debugmsg(d->subsys, "avanti", VERBOSITY_WARNING,
		    "flash chip %i: unknown command 0x%02x\n", chip, val);
	}
}


/*  The ROM bus: the address latch, and the data byte.  */
DEVICE_ACCESS(avanti_rombus)
{
	struct avanti_data *d = (struct avanti_data *) extra;
	uint32_t addr = d->latch & ~AVANTI_ROMBUS_WRITE;
	uint64_t val = 0;

	if (relative_addr < AVANTI_ROMBUS_DATA) {
		if (relative_addr != 0)
			debugmsg(d->subsys, "avanti", VERBOSITY_WARNING,
			    "ROM bus offset 0x%llx\n",
			    (long long) relative_addr);
		if (writeflag == MEM_WRITE)
			d->latch = memory_readmax64(cpu, data, len) >> 8;
		else
			memory_writemax64(cpu, data, len,
			    (uint64_t) d->latch << 8);
		return 1;
	}

	if (writeflag == MEM_WRITE)
		val = memory_readmax64(cpu, data, len) & 0xff;

	if (addr < AVANTI_FLASH_SIZE) {
		if (writeflag == MEM_WRITE)
			avanti_flash_write(d, addr, val);
		else
			val = avanti_flash_read(d, addr);
	} else if (addr >= AVANTI_ROMBUS_NVRAM &&
	    addr < AVANTI_ROMBUS_NVRAM + AVANTI_NVRAM_SIZE) {
		if (writeflag == MEM_WRITE) {
			d->nvram[addr - AVANTI_ROMBUS_NVRAM] = val;
			d->nvram_dirty = 1;
			d->flash_changed = time(NULL);
		} else
			val = d->nvram[addr - AVANTI_ROMBUS_NVRAM];
	} else if (addr == AVANTI_ROMBUS_JUMPERS) {
		if (writeflag == MEM_READ)
			val = d->jumpers;
	} else if (addr == AVANTI_ROMBUS_LED) {
		if (writeflag == MEM_WRITE) {
			d->led = val;
			debugmsg(d->subsys, "avanti", VERBOSITY_INFO,
			    "progress code 0x%02x\n", d->led);
		} else
			val = d->led;
	} else {
		debugmsg(d->subsys, "avanti", VERBOSITY_WARNING,
		    "ROM bus address 0x%06x %s 0x%02x\n", addr,
		    writeflag == MEM_WRITE? "write" : "read", (int) val);
		if (writeflag == MEM_READ)
			val = 0xff;
	}

	if (writeflag == MEM_READ)
		memory_writemax64(cpu, data, len, val);

	return 1;
}


static void avanti_sio_interrupt_assert(struct interrupt *interrupt)
{
	struct avanti_data *d = (struct avanti_data *) interrupt->extra;
	INTERRUPT_ASSERT(d->irq_sio);
}

static void avanti_sio_interrupt_deassert(struct interrupt *interrupt)
{
	struct avanti_data *d = (struct avanti_data *) interrupt->extra;
	INTERRUPT_DEASSERT(d->irq_sio);
}

static void avanti_rtc_interrupt_assert(struct interrupt *interrupt)
{
	struct avanti_data *d = (struct avanti_data *) interrupt->extra;
	INTERRUPT_ASSERT(d->irq_rtc);
}

static void avanti_rtc_interrupt_deassert(struct interrupt *interrupt)
{
	struct avanti_data *d = (struct avanti_data *) interrupt->extra;
	INTERRUPT_DEASSERT(d->irq_rtc);
}


/*  PIRQ0..3, routed by the SIO to an ISA IRQ:  */
static void avanti_pirq_set(struct avanti_data *d, int pirq, int assert)
{
	int irq = d->pirq_routed[pirq];

	if (assert && irq < 0) {
		uint32_t route = bus_pci_cfg_read32(dev_apecs_pci(d->apecs),
		    0, AVANTI_PCI_DEV_SIO, 0, SIO_PIRQ_ROUTE);

		route = (route >> (pirq * 8)) & 0xff;
		if (route & SIO_PIRQ_DISABLE) {
			debugmsg(d->subsys, "avanti", VERBOSITY_DEBUG,
			    "PIRQ%i is not routed\n", pirq);
			return;
		}

		irq = route & 0xf;
		if (!d->isa_irq_connected[irq]) {
			char tmp[400];
			snprintf(tmp, sizeof(tmp), "%s.%i", d->isa_irq_path,
			    irq);
			INTERRUPT_CONNECT(tmp, d->isa_irq[irq]);
			d->isa_irq_connected[irq] = 1;
		}

		debugmsg(d->subsys, "avanti", VERBOSITY_DEBUG,
		    "PIRQ%i: ISA IRQ %i\n", pirq, irq);
		d->pirq_routed[pirq] = irq;
		if (d->isa_irq_count[irq]++ == 0)
			INTERRUPT_ASSERT(d->isa_irq[irq]);
	} else if (!assert && irq >= 0) {
		d->pirq_routed[pirq] = -1;
		if (--d->isa_irq_count[irq] == 0)
			INTERRUPT_DEASSERT(d->isa_irq[irq]);
	}
}

static void avanti_pirq_assert(struct interrupt *interrupt)
{
	avanti_pirq_set((struct avanti_data *) interrupt->extra,
	    interrupt->line, 1);
}

static void avanti_pirq_deassert(struct interrupt *interrupt)
{
	avanti_pirq_set((struct avanti_data *) interrupt->extra,
	    interrupt->line, 0);
}


/*
 *  dev_avanti_init():
 */
struct avanti_data *dev_avanti_init(struct machine *machine,
	struct memory *mem, uint64_t ram_size, const char *cpu_irq)
{
	struct avanti_data *d;
	struct interrupt templ;
	char sio_irq_path[300], fdc_irq_path[310], tmp[310];
	int i;

	CHECK_ALLOCATION(d = (struct avanti_data *) malloc(sizeof(*d)));
	memset(d, 0, sizeof(*d));

	d->machine = machine;
	d->ram_size = ram_size;
	d->subsys = debugmsg_register_subsystem("avanti");
	d->jumpers = AVANTI_JUMPERS_DEFAULT;
	memset(d->flash, 0xff, sizeof(d->flash));

	snprintf(tmp, sizeof(tmp), "%s.%i", cpu_irq, AVANTI_IRQ_SIO);
	INTERRUPT_CONNECT(tmp, d->irq_sio);
	snprintf(tmp, sizeof(tmp), "%s.%i", cpu_irq, AVANTI_IRQ_RTC);
	INTERRUPT_CONNECT(tmp, d->irq_rtc);

	/*  The SIO's 8259s interrupt the CPU:  */
	snprintf(sio_irq_path, sizeof(sio_irq_path), "%s.avanti",
	    machine->path);
	memset(&templ, 0, sizeof(templ));
	templ.line = 0;
	templ.name = sio_irq_path;
	templ.extra = d;
	templ.interrupt_assert = avanti_sio_interrupt_assert;
	templ.interrupt_deassert = avanti_sio_interrupt_deassert;
	interrupt_handler_register(&templ);

	/*  The RTC interrupts the CPU on a line of its own:  */
	snprintf(tmp, sizeof(tmp), "%s.rtc", sio_irq_path);
	templ.name = tmp;
	templ.interrupt_assert = avanti_rtc_interrupt_assert;
	templ.interrupt_deassert = avanti_rtc_interrupt_deassert;
	interrupt_handler_register(&templ);

	memory_device_register(mem, "avanti_rombus", AVANTI_ROMBUS_BASE,
	    AVANTI_ROMBUS_SIZE, dev_avanti_rombus_access, d, DM_DEFAULT, NULL);

	/*  PCI interrupts:  */
	for (i = 0; i < 4; i++) {
		snprintf(tmp, sizeof(tmp), "%s.pirq.%i", sio_irq_path, i);
		templ.line = i;
		templ.name = tmp;
		templ.interrupt_assert = avanti_pirq_assert;
		templ.interrupt_deassert = avanti_pirq_deassert;
		interrupt_handler_register(&templ);
		d->pirq_routed[i] = -1;
	}

	snprintf(tmp, sizeof(tmp), "%s.isa", sio_irq_path);
	CHECK_ALLOCATION(d->isa_irq_path = strdup(tmp));
	d->apecs = dev_apecs_init(machine, mem, AVANTI_IO_BASE,
	    AVANTI_MEM_BASE, sio_irq_path, tmp);
	dev_apecs_set_memory(d->apecs, ram_size);

	bus_pci_add(machine, dev_apecs_pci(d->apecs), mem, 0,
	    AVANTI_PCI_DEV_SCSI, 0, "sym53c810");
	bus_pci_add(machine, dev_apecs_pci(d->apecs), mem, 0,
	    AVANTI_PCI_DEV_SIO, 0, "i82378zb");

	/*  With a network, a DE500-BA (21143) in the first PCI slot:  */
	if (machine->emul->net != NULL)
		bus_pci_add(machine, dev_apecs_pci(d->apecs), mem, 0,
		    AVANTI_PCI_DEV_SLOT0, 0, "dec21143");

	/*  Behind the SIO: 8259s, 8254, RTC, keyboard, serial ports, LPT,
	    and the DMA controllers with the floppy (IRQ 6, channel 2).  */
	bus_isa_init(machine, sio_irq_path, BUS_ISA_LPTBASE_3BC |
	    BUS_ISA_RTC_OWN_IRQ | BUS_ISA_PIT_COUNTS, AVANTI_IO_BASE,
	    AVANTI_MEM_BASE);

	/*
	 *  The PALcode acknowledges ISA interrupts with an interrupt
	 *  acknowledge cycle (dev_apecs.c), so requests that arrive while
	 *  the same IRQ is in service must survive the EOI, as on a real
	 *  8259 (Tru64's floppy driver starts the next command before the
	 *  EOI of the last one); the cascade input is a level.
	 */
	if (machine->isa_pic_data.pic1 != NULL &&
	    machine->isa_pic_data.pic2 != NULL) {
		machine->isa_pic_data.pic1->latch_in_service = 1;
		machine->isa_pic_data.pic2->latch_in_service = 1;
		machine->isa_pic_data.pic1->elcr |= 0x04;
	}

	d->dma = dev_isadma_init(machine, mem, AVANTI_IO_BASE);
	snprintf(fdc_irq_path, sizeof(fdc_irq_path), "%s.isa.6",
	    sio_irq_path);
	dev_fdc_init(machine, mem, AVANTI_IO_BASE + 0x3f0, fdc_irq_path,
	    d->dma, 2);

	return d;
}


/*
 *  dev_avanti_set_rom():
 *
 *  The 1 MB flash, and the 64 KB DROM.
 */
void dev_avanti_set_rom(struct avanti_data *d, const unsigned char *flash,
	const unsigned char *drom)
{
	memcpy(d->flash, flash, AVANTI_FLASH_SIZE);
	memcpy(d->drom, drom, AVANTI_DROM_SIZE);
	d->have_flash = d->have_drom = 1;

	if (d->machine->nvram_filename != NULL) {
		size_t n = strlen(d->machine->nvram_filename) + 7;
		FILE *f;

		CHECK_ALLOCATION(d->flash_file = (char *) malloc(n));
		snprintf(d->flash_file, n, "%s.flash",
		    d->machine->nvram_filename);

		f = fopen(d->flash_file, "rb");
		if (f != NULL) {
			unsigned char *buf;

			CHECK_ALLOCATION(buf = (unsigned char *)
			    malloc(AVANTI_FLASH_SIZE));
			if (fread(buf, 1, AVANTI_FLASH_SIZE, f) ==
			    AVANTI_FLASH_SIZE)
				memcpy(d->flash, buf, AVANTI_FLASH_SIZE);
			else
				fatal("[ avanti: %s has the wrong size; using "
				    "the ROM image instead ]\n", d->flash_file);
			free(buf);
			fclose(f);
		}

		n = strlen(d->machine->nvram_filename) + 6;
		CHECK_ALLOCATION(d->nvram_file = (char *) malloc(n));
		snprintf(d->nvram_file, n, "%s.wart",
		    d->machine->nvram_filename);
		f = fopen(d->nvram_file, "rb");
		if (f != NULL) {
			if (fread(d->nvram, 1, AVANTI_NVRAM_SIZE, f) !=
			    AVANTI_NVRAM_SIZE)
				fatal("[ avanti: %s has the wrong size ]\n",
				    d->nvram_file);
			fclose(f);
		}

		if (avanti_flash_owner == NULL)
			atexit(avanti_flash_save_atexit);
		avanti_flash_owner = d;
		machine_add_tickfunction(d->machine, dev_avanti_tick, d, 24);
	}
}


/*
 *  dev_avanti_srom():
 *
 *  What the serial ROM leaves for the DROM: memory set up, the DROM's code
 *  (after its header) at AVANTI_DROM_ADDR, which is also the PAL base, and
 *  started in PALmode at its first instruction; copies of ABOX_CTL and
 *  BIU_CTL in PAL_TEMP14 and PAL_TEMP18 (the DROM updates both); and
 *  parameters in memory at 0..0x70, which the DROM passes on to the
 *  console in a0..t11, t12 and at (0x00..0x48, 0x58, 0x60):
 *
 *	0x70	the CPU speed code: 2 or 8 makes the DROM time the CPU
 *		against the 8254 and pick the speed (2 = 166 MHz, ...),
 *		and store the cycle time in ps at 0x10, MHz at 0x68 and
 *		the code at 0x70.
 *
 *  What the rest mean, the console will show.
 *  Returns the entry point.
 */
uint64_t dev_avanti_srom(struct avanti_data *d, struct cpu *cpu)
{
	unsigned char speed_code[8] = { 2, 0, 0, 0, 0, 0, 0, 0 };

	cpu->memory_rw(cpu, cpu->mem, AVANTI_DROM_ADDR,
	    d->drom + AVANTI_ROM_HEADER_SIZE,
	    AVANTI_DROM_SIZE - AVANTI_ROM_HEADER_SIZE, MEM_WRITE,
	    PHYSICAL | NO_EXCEPTIONS);

	cpu->memory_rw(cpu, cpu->mem, 0x70, speed_code, sizeof(speed_code),
	    MEM_WRITE, PHYSICAL | NO_EXCEPTIONS);

	/*  Nothing is connected to the serial ROM port: its receive line
	    idles at 1 (mark).  */
	cpu->cd.alpha.sl_rcv = 1;

	cpu->cd.alpha.pal_temp[14] = EV4_ABOX_MCHK_EN | EV4_ABOX_CRD_EN;
	cpu->cd.alpha.pal_temp[18] = 0;
	cpu->cd.alpha.pal_base = AVANTI_DROM_ADDR;
	cpu->cd.alpha.palmode = 1;

	return ALPHA_PALMODE_TAG | AVANTI_DROM_ADDR;
}
