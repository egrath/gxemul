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
 *  COMMENT: DECchip 21071 (APECS) core logic: COMANCHE and EPIC
 *
 *  The 21071-CA "COMANCHE" memory controller and the 21071-DA "EPIC" PCI
 *  bridge, as on the AlphaStation 200/400 ("Avanti"), between an EV4
 *  and PCI. Physical address map of the CPU side:
 *
 *	1.8000.0000	COMANCHE (memory controller) CSRs
 *	1.A000.0000	EPIC (PCI bridge) CSRs
 *	1.B000.0000	PCI interrupt acknowledge
 *	1.C000.0000	PCI sparse I/O space
 *	1.E000.0000	PCI configuration space
 *	2.0000.0000	PCI sparse memory space
 *	3.0000.0000	PCI dense memory space
 *
 *  The CSRs are 32 bits wide, 0x20 bytes apart. In the sparse spaces,
 *  address bits <4:3> of the CPU address give the length of the bus
 *  access (byte, word, tribyte, longword) and the bus address is the CPU
 *  address shifted right by 5; the data is on the byte lanes of the bus
 *  address within a longword. The high bits of sparse memory addresses
 *  come from HAXR1, and HAXR2 bits <1:0> select a type 1 configuration
 *  cycle.
 *
 *  Cycles on PCI (and through the SIO, ISA) go to the emulated devices
 *  registered at io_base + port and mem_base + address.
 *
 *  Not yet: the DMA windows and the scatter-gather TLB (devices don't do
 *  DMA yet), and the meaning of most CSR bits; COMANCHE and EPIC CSRs are
 *  kept as written, with their error bits (which are cleared by writing
 *  ones) never set. Both chips say they are pass 2. The memory banksets are
 *  set up as the serial ROM would (dev_apecs_set_memory()).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bus_pci.h"
#include "cpu.h"
#include "device.h"
#include "devices.h"
#include "machine.h"
#include "memory.h"
#include "misc.h"


#define	APECS_COMANCHE_BASE	0x180000000ULL
#define	APECS_COMANCHE_SIZE	0x20000000ULL
#define	APECS_EPIC_BASE		0x1a0000000ULL
#define	APECS_EPIC_SIZE		0x10000000ULL
#define	APECS_IACK_BASE		0x1b0000000ULL
#define	APECS_IACK_SIZE		0x10000000ULL
#define	APECS_SPARSE_IO_BASE	0x1c0000000ULL
#define	APECS_SPARSE_IO_SIZE	0x20000000ULL
#define	APECS_CONF_BASE		0x1e0000000ULL
#define	APECS_CONF_SIZE		0x20000000ULL
#define	APECS_SPARSE_MEM_BASE	0x200000000ULL
#define	APECS_SPARSE_MEM_SIZE	0x100000000ULL
#define	APECS_DENSE_MEM_BASE	0x300000000ULL
#define	APECS_DENSE_MEM_SIZE	0x100000000ULL

#define	APECS_N_CSRS		128

/*  COMANCHE CSRs (offset / 0x20), from the DECchip 21071 data sheet:  */
#define	COMANCHE_GCR		0x00
#define	COMANCHE_ED		0x02	/*  error and diagnostic status  */
#define	COMANCHE_BAR(n)		(0x40 + (n))	/*  bankset base address  */
#define	COMANCHE_CNFG(n)	(0x50 + (n))	/*  bankset configuration  */

#define	COMANCHE_ED_ERRORS	0x0000003f	/*  cleared by writing ones  */
#define	COMANCHE_ED_PASS2	0x00002000

#define	COMANCHE_CNFG_VALID	0x0001

/*  EPIC CSRs (offset / 0x20):  */
#define	EPIC_DCSR		0x00
#define	EPIC_PEAR		0x01
#define	EPIC_SEAR		0x02
#define	EPIC_HAXR0		0x0c
#define	EPIC_HAXR1		0x0d
#define	EPIC_HAXR2		0x0e

/*  EPIC_DCSR: bits <15:6> are errors, cleared by writing ones.  */
#define	EPIC_DCSR_ERRORS	0x0000ffc0
#define	EPIC_DCSR_PASS2		0x80000000


struct apecs_data {
	struct machine		*machine;
	struct pci_data		*pci;
	int			subsys;

	uint64_t		io_base;	/*  where PCI I/O devices are  */
	uint64_t		mem_base;	/*  where PCI memory devices are  */

	uint32_t		comanche[APECS_N_CSRS];
	uint32_t		epic[APECS_N_CSRS];
};


struct pci_data *dev_apecs_pci(struct apecs_data *d)
{
	return d->pci;
}


/*
 *  A bus cycle of 1..4 bytes at bus address busaddr (in the space at
 *  base), with the data on the byte lanes of busaddr & 3 in val.
 */
static uint64_t apecs_bus_cycle(struct cpu *cpu, uint64_t base,
	uint64_t busaddr, int n, uint64_t val, int writeflag)
{
	int i, lane = busaddr & 3;
	uint64_t result = 0;

	for (i = 0; i < n; i++) {
		uint8_t b = val >> ((lane + i) * 8);
		memory_device_dispatch(cpu, cpu->mem, base + busaddr + i,
		    &b, 1, writeflag);
		result |= (uint64_t) b << ((lane + i) * 8);
	}

	return result;
}


/*
 *  Sparse spaces: bits <4:3> of the CPU address are the length, the bus
 *  address is the CPU address >> 5.
 */
static int apecs_sparse(struct cpu *cpu, struct apecs_data *d, uint64_t base,
	uint64_t busaddr, uint64_t relative_addr, unsigned char *data,
	size_t len, int writeflag)
{
	int n = ((relative_addr >> 3) & 3) + 1;
	uint64_t val = 0;

	if (writeflag == MEM_WRITE)
		val = memory_readmax64(cpu, data, len);

	val = apecs_bus_cycle(cpu, base, busaddr, n, val, writeflag);

	if (writeflag == MEM_READ)
		memory_writemax64(cpu, data, len, val);

	return 1;
}


DEVICE_ACCESS(apecs_sparse_io)
{
	struct apecs_data *d = (struct apecs_data *) extra;

	return apecs_sparse(cpu, d, d->io_base, (relative_addr >> 5) &
	    0xffffff, relative_addr, data, len, writeflag);
}


DEVICE_ACCESS(apecs_sparse_mem)
{
	struct apecs_data *d = (struct apecs_data *) extra;
	uint64_t busaddr = ((relative_addr >> 5) & 0x7ffffff) |
	    (d->epic[EPIC_HAXR1] & 0xf8000000);

	return apecs_sparse(cpu, d, d->mem_base, busaddr, relative_addr,
	    data, len, writeflag);
}


DEVICE_ACCESS(apecs_dense_mem)
{
	struct apecs_data *d = (struct apecs_data *) extra;

	return memory_device_dispatch(cpu, cpu->mem, d->mem_base +
	    relative_addr, data, len, writeflag);
}


/*
 *  PCI configuration space: the bus address has the bus number in <23:16>,
 *  the device in <15:11>, the function in <10:8> and the register in <7:0>
 *  (the EPIC drives the IDSEL line of the device for a type 0 cycle on bus
 *  0; HAXR2 bits <1:0> select a type 1 cycle for the other buses).
 */
DEVICE_ACCESS(apecs_conf)
{
	struct apecs_data *d = (struct apecs_data *) extra;
	uint32_t a = relative_addr >> 5;
	int n = ((relative_addr >> 3) & 3) + 1;
	int bus = 0, dev = (a >> 11) & 0x1f, func = (a >> 8) & 7;
	int reg = a & 0xfc, lane = a & 3;
	uint64_t val = 0, old;

	if (writeflag == MEM_WRITE)
		val = memory_readmax64(cpu, data, len);

	if (d->epic[EPIC_HAXR2] & 3)
		bus = (a >> 16) & 0xff;

	bus_pci_setaddr(cpu, d->pci, bus, dev, func, reg);

	/*  A read returns the whole longword; the bytes are on their lanes.  */
	if (writeflag == MEM_READ) {
		bus_pci_data_access(cpu, d->pci, &val, 4, MEM_READ);
		memory_writemax64(cpu, data, len, val);
		return 1;
	}

	/*  A write of less than a longword: merge it into the longword.  */
	val = (val >> (lane * 8)) & (n == 4? 0xffffffffULL :
	    ((1ULL << (n * 8)) - 1));
	if (n < 4) {
		bus_pci_data_access(cpu, d->pci, &old, 4, MEM_READ);
		old &= ~(((1ULL << (n * 8)) - 1) << (lane * 8));
		val = old | (val << (lane * 8));
	}
	bus_pci_setaddr(cpu, d->pci, bus, dev, func, reg);
	bus_pci_data_access(cpu, d->pci, &val, 4, MEM_WRITE);

	return 1;
}


/*
 *  PCI interrupt acknowledge: the SIO's 8259s answer with the vector of
 *  the highest priority pending interrupt, and mark it in service.
 */
DEVICE_ACCESS(apecs_iack)
{
	struct pic8259_data *pic1 = cpu->machine->isa_pic_data.pic1;
	struct pic8259_data *pic2 = cpu->machine->isa_pic_data.pic2;
	int vector = 7, irq;

	if (writeflag == MEM_WRITE)
		return 1;

	if (pic1 != NULL) {
		int active1 = pic1->irr & ~pic1->ier;
		vector = pic1->irq_base + 7;
		for (irq = 0; irq < 8; irq++)
			if (active1 & (1 << irq))
				break;
		if (irq == 2 && pic2 != NULL) {
			int active2 = pic2->irr & ~pic2->ier, irq2;
			for (irq2 = 0; irq2 < 7; irq2++)
				if (active2 & (1 << irq2))
					break;
			pic2->isr |= 1 << irq2;
			pic1->isr |= 1 << 2;
			vector = pic2->irq_base + irq2;
		} else if (irq < 8) {
			pic1->isr |= 1 << irq;
			vector = pic1->irq_base + irq;
		}
	}

	memory_writemax64(cpu, data, len, vector);
	return 1;
}


/*  COMANCHE and EPIC CSRs:  */
static int apecs_csr(struct cpu *cpu, struct apecs_data *d, uint32_t *csrs,
	const char *name, uint64_t relative_addr, unsigned char *data,
	size_t len, int writeflag)
{
	int i = (relative_addr >> 5) & (APECS_N_CSRS - 1);
	uint32_t val;

	if (writeflag == MEM_WRITE) {
		val = memory_readmax64(cpu, data, len);
		if (csrs == d->epic && i == EPIC_DCSR)
			val = (val & ~(EPIC_DCSR_ERRORS | EPIC_DCSR_PASS2)) |
			    (csrs[i] & EPIC_DCSR_ERRORS & ~val) |
			    EPIC_DCSR_PASS2;
		if (csrs == d->comanche && i == COMANCHE_ED)
			val = (csrs[i] & ~COMANCHE_ED_ERRORS & ~val) |
			    COMANCHE_ED_PASS2;
		debugmsg(d->subsys, "apecs", VERBOSITY_DEBUG,
		    "%s CSR 0x%03x = 0x%08x\n", name, i << 5, val);
		csrs[i] = val;
		return 1;
	}

	debugmsg(d->subsys, "apecs", VERBOSITY_DEBUG,
	    "%s CSR 0x%03x read: 0x%08x\n", name, i << 5, csrs[i]);
	memory_writemax64(cpu, data, len, csrs[i]);
	return 1;
}

DEVICE_ACCESS(apecs_comanche)
{
	struct apecs_data *d = (struct apecs_data *) extra;

	return apecs_csr(cpu, d, d->comanche, "COMANCHE", relative_addr,
	    data, len, writeflag);
}

DEVICE_ACCESS(apecs_epic)
{
	struct apecs_data *d = (struct apecs_data *) extra;

	return apecs_csr(cpu, d, d->epic, "EPIC", relative_addr, data, len,
	    writeflag);
}


/*
 *  dev_apecs_set_memory():
 *
 *  Sets up the memory banksets for ram_size bytes, as the serial ROM does:
 *  banksets of 8 to 512 MB (a power of two), naturally aligned, the largest
 *  first, so that memory is contiguous. Bankset base address registers have
 *  the base<33:23> in bits <15:5>; the configuration registers have the
 *  valid bit and the size code in bits <4:1> (1 = 512 MB .. 7 = 8 MB).
 */
void dev_apecs_set_memory(struct apecs_data *d, uint64_t ram_size)
{
	uint64_t base = 0, size;
	int n, code;

	for (n = 0; n < 8 && ram_size >= (8 << 20); n++) {
		for (size = 512 << 20, code = 1; size > ram_size;
		    size >>= 1, code++)
			;
		d->comanche[COMANCHE_BAR(n)] = (base >> 23) << 5;
		d->comanche[COMANCHE_CNFG(n)] = (code << 1) |
		    COMANCHE_CNFG_VALID;
		base += size;
		ram_size -= size;
	}
}


/*
 *  dev_apecs_init():
 *
 *  io_base and mem_base are where the emulated PCI and ISA devices are
 *  registered (at io_base + port, mem_base + address); pci_irq_path and
 *  isa_irq_path are passed on to the PCI bus.
 */
struct apecs_data *dev_apecs_init(struct machine *machine, struct memory *mem,
	uint64_t io_base, uint64_t mem_base, const char *pci_irq_path,
	const char *isa_irq_path)
{
	struct apecs_data *d;

	CHECK_ALLOCATION(d = (struct apecs_data *) malloc(sizeof(*d)));
	memset(d, 0, sizeof(*d));

	d->machine = machine;
	d->io_base = io_base;
	d->mem_base = mem_base;
	d->subsys = debugmsg_register_subsystem("apecs");
	d->comanche[COMANCHE_ED] = COMANCHE_ED_PASS2;
	d->epic[EPIC_DCSR] = EPIC_DCSR_PASS2;

	d->pci = bus_pci_init(machine, pci_irq_path, io_base, mem_base,
	    0x10000, 0x1000000, pci_irq_path, 0, 0, isa_irq_path);

	memory_device_register(mem, "apecs_comanche", APECS_COMANCHE_BASE,
	    APECS_COMANCHE_SIZE, dev_apecs_comanche_access, d, DM_DEFAULT,
	    NULL);
	memory_device_register(mem, "apecs_epic", APECS_EPIC_BASE,
	    APECS_EPIC_SIZE, dev_apecs_epic_access, d, DM_DEFAULT, NULL);
	memory_device_register(mem, "apecs_iack", APECS_IACK_BASE,
	    APECS_IACK_SIZE, dev_apecs_iack_access, d, DM_DEFAULT, NULL);
	memory_device_register(mem, "apecs_sparse_io", APECS_SPARSE_IO_BASE,
	    APECS_SPARSE_IO_SIZE, dev_apecs_sparse_io_access, d, DM_DEFAULT,
	    NULL);
	memory_device_register(mem, "apecs_conf", APECS_CONF_BASE,
	    APECS_CONF_SIZE, dev_apecs_conf_access, d, DM_DEFAULT, NULL);
	memory_device_register(mem, "apecs_sparse_mem", APECS_SPARSE_MEM_BASE,
	    APECS_SPARSE_MEM_SIZE, dev_apecs_sparse_mem_access, d,
	    DM_DEFAULT, NULL);
	memory_device_register(mem, "apecs_dense_mem", APECS_DENSE_MEM_BASE,
	    APECS_DENSE_MEM_SIZE, dev_apecs_dense_mem_access, d,
	    DM_DEFAULT, NULL);

	return d;
}
