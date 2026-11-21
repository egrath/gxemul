/*
 *  Copyright (C) 2005-2021  Anders Gavare.  All rights reserved.
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
 *  Hitachi SuperH ("SH") CPU emulation.
 *
 *  TODO: It would be nice if this could encompass both 64-bit SH5, and
 *        32-bit SH encodings. Right now, it only really supports 32-bit mode.
 *
 *  TODO: This actually only works or SH4 so far, not SH3.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>

#include "cpu.h"
#include "device.h"
#include "float_emul.h"
#include "interrupt.h"
#include "machine.h"
#include "memory.h"
#include "misc.h"
#include "settings.h"
#include "symbol.h"

#include "thirdparty/sh4_exception.h"
#include "thirdparty/sh4_mmu.h"


#define DYNTRANS_32
#define DYNTRANS_DELAYSLOT
#include "tmp_sh_head.c"



void sh_pc_to_pointers(struct cpu *);

void sh3_cpu_interrupt_assert(struct interrupt *interrupt);
void sh3_cpu_interrupt_deassert(struct interrupt *interrupt);


/*
 *  sh_cpu_new():
 *
 *  Create a new SH cpu object.
 *
 *  Returns 1 on success, 0 if there was no matching SH processor with
 *  this cpu_type_name.
 */
int sh_cpu_new(struct cpu *cpu, struct memory *mem, struct machine *machine,
	int cpu_id, char *cpu_type_name)
{
	int i = 0;
	struct sh_cpu_type_def cpu_type_defs[] = SH_CPU_TYPE_DEFS;

	/*  Scan the cpu_type_defs list for this cpu type:  */
	while (cpu_type_defs[i].name != NULL) {
		if (strcasecmp(cpu_type_defs[i].name, cpu_type_name) == 0) {
			break;
		}
		i++;
	}
	if (cpu_type_defs[i].name == NULL)
		return 0;

	cpu->memory_rw = sh_memory_rw;

	cpu->cd.sh.cpu_type = cpu_type_defs[i];
	cpu->byte_order = EMUL_LITTLE_ENDIAN;
	cpu->is_32bit = cpu->cd.sh.cpu_type.bits == 32;

	if (!cpu->is_32bit) {
		fatal_error_cpu(cpu, SUBSYS_CPU, "cpu", "SH64 emulation not implemented. Sorry.\n");
	}

	cpu->instruction_has_delayslot = sh_cpu_instruction_has_delayslot;

	cpu->translate_v2p = sh_translate_v2p;

	cpu->run_instr = sh_run_instr;
	cpu->update_translation_table = sh_update_translation_table;
	cpu->invalidate_translation_caches =
	    sh_invalidate_translation_caches;
	cpu->invalidate_code_translation =
	    sh_invalidate_code_translation;

	/*  Initial value of FPSCR (according to the SH4 manual):  */
	cpu->cd.sh.fpscr = 0x00040001;

	/*  (Initial value of the program counter on reboot is 0xA0000000.)  */

	/*  Start in Privileged Mode:  */
	cpu->cd.sh.sr = SH_SR_MD | SH_SR_IMASK;

	/*  Stack pointer at end of physical RAM:  */
	cpu->cd.sh.r[15] = cpu->machine->physical_ram_in_mb * 1048576 - 64;

	CPU_SETTINGS_ADD_REGISTER64("pc", cpu->pc);
	CPU_SETTINGS_ADD_REGISTER32("sr", cpu->cd.sh.sr);
	CPU_SETTINGS_ADD_REGISTER32("pr", cpu->cd.sh.pr);
	CPU_SETTINGS_ADD_REGISTER32("vbr", cpu->cd.sh.vbr);
	CPU_SETTINGS_ADD_REGISTER32("sgr", cpu->cd.sh.sgr);
	CPU_SETTINGS_ADD_REGISTER32("gbr", cpu->cd.sh.gbr);
	CPU_SETTINGS_ADD_REGISTER32("dbr", cpu->cd.sh.dbr);
	CPU_SETTINGS_ADD_REGISTER32("macl", cpu->cd.sh.macl);
	CPU_SETTINGS_ADD_REGISTER32("mach", cpu->cd.sh.mach);
	CPU_SETTINGS_ADD_REGISTER32("expevt", cpu->cd.sh.expevt);
	CPU_SETTINGS_ADD_REGISTER32("intevt", cpu->cd.sh.intevt);
	CPU_SETTINGS_ADD_REGISTER32("tra", cpu->cd.sh.tra);
	CPU_SETTINGS_ADD_REGISTER32("fpscr", cpu->cd.sh.fpscr);
	CPU_SETTINGS_ADD_REGISTER32("fpul", cpu->cd.sh.fpul);
	for (i=0; i<SH_N_GPRS; i++) {
		char tmpstr[5];
		snprintf(tmpstr, sizeof(tmpstr), "r%i", i);
		CPU_SETTINGS_ADD_REGISTER32(tmpstr, cpu->cd.sh.r[i]);
	}
	for (i=0; i<SH_N_GPRS_BANKED; i++) {
		char tmpstr[15];
		snprintf(tmpstr, sizeof(tmpstr), "r%i_bank", i);
		CPU_SETTINGS_ADD_REGISTER32(tmpstr, cpu->cd.sh.r_bank[i]);
	}
	for (i=0; i<SH_N_FPRS; i++) {
		char tmpstr[6];
		snprintf(tmpstr, sizeof(tmpstr), "fr%i", i);
		CPU_SETTINGS_ADD_REGISTER32(tmpstr, cpu->cd.sh.fr[i]);
		snprintf(tmpstr, sizeof(tmpstr), "xf%i", i);
		CPU_SETTINGS_ADD_REGISTER32(tmpstr, cpu->cd.sh.xf[i]);
	}
	for (i=0; i<SH_N_ITLB_ENTRIES; i++) {
		char tmpstr[15];
		snprintf(tmpstr, sizeof(tmpstr), "itlb_hi_%i", i);
		CPU_SETTINGS_ADD_REGISTER32(tmpstr, cpu->cd.sh.itlb_hi[i]);
		snprintf(tmpstr, sizeof(tmpstr), "itlb_lo_%i", i);
		CPU_SETTINGS_ADD_REGISTER32(tmpstr, cpu->cd.sh.itlb_lo[i]);
	}
	for (i=0; i<SH_N_UTLB_ENTRIES; i++) {
		char tmpstr[15];
		snprintf(tmpstr, sizeof(tmpstr), "utlb_hi_%i", i);
		CPU_SETTINGS_ADD_REGISTER32(tmpstr, cpu->cd.sh.utlb_hi[i]);
		snprintf(tmpstr, sizeof(tmpstr), "utlb_lo_%i", i);
		CPU_SETTINGS_ADD_REGISTER32(tmpstr, cpu->cd.sh.utlb_lo[i]);
	}

	/*  Register the CPU's interrupts:  */
	if (cpu->cd.sh.cpu_type.arch == 4) {
		for (i=SH_INTEVT_NMI; i<0x1000; i+=0x20) {
			struct interrupt templ;
			char name[100];
			snprintf(name, sizeof(name), "%s.irq[0x%x]",
			    cpu->path, i);
			memset(&templ, 0, sizeof(templ));
			templ.line = i;
			templ.name = name;
			templ.extra = cpu;
			templ.interrupt_assert = sh_cpu_interrupt_assert;
			templ.interrupt_deassert = sh_cpu_interrupt_deassert;
			interrupt_handler_register(&templ);
		}
	} else {
		struct interrupt templ;
		memset(&templ, 0, sizeof(templ));
		templ.line = i;
		templ.name = cpu->path;
		templ.extra = cpu;
		templ.interrupt_assert = sh3_cpu_interrupt_assert;
		templ.interrupt_deassert = sh3_cpu_interrupt_deassert;
		interrupt_handler_register(&templ);
	}

	/*  SH4-specific memory mapped registers, TLBs, caches, etc:  */
	if (cpu->cd.sh.cpu_type.arch == 4) {
		cpu->cd.sh.pcic_pcibus = (struct pci_data *) device_add(machine, "sh4");

		/*
		 *  Interrupt Controller initial values, according to the
		 *  SH7760 manual:
		 */
		cpu->cd.sh.intc_iprd = 0xda74;
		cpu->cd.sh.intc_intmsk00 = 0xf3ff7fff;
		cpu->cd.sh.intc_intmsk04 = 0x00ffffff;
		/*  All others are zero.  */

		/*  TODO: Initial priorities?  */
		cpu->cd.sh.intc_intpri00 = 0x33333333;
		cpu->cd.sh.intc_intpri04 = 0x33333333;
		cpu->cd.sh.intc_intpri08 = 0x33333333;
		cpu->cd.sh.intc_intpri0c = 0x33333333;
	}

	sh_update_interrupt_priorities(cpu);

	return 1;
}


/*
 *  sh_update_interrupt_priorities():
 *
 *  SH interrupts are a bit complicated; there are several intc registers
 *  controlling priorities for various peripherals:
 *
 *  Register:  Bits 15..12  11..8  7..4      3..0
 *  ---------  -----------  -----  ----      ----
 *  ipra       TMU0         TMU1   TMU2      Reserved
 *  iprb       WDT          REF    Reserved  Reserved
 *  iprc       GPIO         DMAC   Reserved  H-UDI
 *  iprd       IRL0         IRL1   IRL2      IRL3
 *
 *  Register:  31..28  27..24  23..20  19..16  15..12  11..8   7..4   3..0
 *  ---------  ------  ------  ------  ------  ------  -----   ----   ----
 *  intpri00   IRQ4    IRQ5    IRQ6    IRQ7    Rsrvd.  Rsrvd.  Rsrvd. Reserved
 *  intpri04   HCAN2,0 HCAN2,1 SSI(0)  SSI(1)  HAC(0)  HAC(1)  I2C(0) I2C(1)
 *  intpri08   USB     LCDC    DMABRG  SCIF(0) SCIF(1) SCIF(2) SIM    HSPI
 *  intpri0c   Reserv. Reserv. MMCIF   Reserv. MFI     Rsrvd.  ADC    CMT
 */
void sh_update_interrupt_priorities(struct cpu *cpu)
{
	int i;

	/*
	 *  Set priorities of known interrupts, without affecting the
	 *  SH_INT_ASSERTED bit:
	 */

	for (i=SH4_INTEVT_IRQ0; i<=SH4_INTEVT_IRQ14; i+=0x20) {
		cpu->cd.sh.int_prio_and_pending[i/0x20] &= ~SH_INT_PRIO_MASK;
		cpu->cd.sh.int_prio_and_pending[i/0x20] |= (15 - ((i - 
		    SH4_INTEVT_IRQ0) / 0x20));
	}

	cpu->cd.sh.int_prio_and_pending[SH_INTEVT_TMU0_TUNI0 / 0x20] &=
	    ~SH_INT_PRIO_MASK;
	cpu->cd.sh.int_prio_and_pending[SH_INTEVT_TMU0_TUNI0 / 0x20] |=
	    (cpu->cd.sh.intc_ipra >> 12) & 0xf;

	cpu->cd.sh.int_prio_and_pending[SH_INTEVT_TMU1_TUNI1 / 0x20] &=
	    ~SH_INT_PRIO_MASK;
	cpu->cd.sh.int_prio_and_pending[SH_INTEVT_TMU1_TUNI1 / 0x20] |=
	    (cpu->cd.sh.intc_ipra >> 8) & 0xf;

	cpu->cd.sh.int_prio_and_pending[SH_INTEVT_TMU2_TUNI2 / 0x20] &=
	    ~SH_INT_PRIO_MASK;
	cpu->cd.sh.int_prio_and_pending[SH_INTEVT_TMU2_TUNI2 / 0x20] |=
	    (cpu->cd.sh.intc_ipra >> 4) & 0xf;

	for (i=SH4_INTEVT_SCIF_ERI; i<=SH4_INTEVT_SCIF_TXI; i+=0x20) {
		cpu->cd.sh.int_prio_and_pending[i/0x20] &= ~SH_INT_PRIO_MASK;
		cpu->cd.sh.int_prio_and_pending[i/0x20] |=
		    ((cpu->cd.sh.intc_intpri08 >> 16) & 0xf);
	}
}


/*
 *  sh3_cpu_interrupt_assert():
 *  sh3_cpu_interrupt_deassert():
 */
void sh3_cpu_interrupt_assert(struct interrupt *interrupt)
{
	/*  TODO  */
}
void sh3_cpu_interrupt_deassert(struct interrupt *interrupt)
{
	/*  TODO  */
}       


/*
 *  sh_cpu_interrupt_assert():
 */
void sh_cpu_interrupt_assert(struct interrupt *interrupt)
{
	struct cpu *cpu = (struct cpu *) interrupt->extra;
	unsigned int irq_nr = interrupt->line;
	unsigned int index = irq_nr / 0x20;
	unsigned int prio;

	/*  Assert the interrupt, and check its priority level:  */
	cpu->cd.sh.int_prio_and_pending[index] |= SH_INT_ASSERTED;
	prio = cpu->cd.sh.int_prio_and_pending[index] & SH_INT_PRIO_MASK;

	if (prio == 0) {
		/*  Interrupt not implemented? Hm.  */
		debugmsg(SUBSYS_CPU, "cpu", VERBOSITY_DEBUG, "[ SH interrupt 0x%x, prio 0 (?), aborting ]", irq_nr);
	}

	if (cpu->cd.sh.int_to_assert == 0 || prio > cpu->cd.sh.int_level) {
		cpu->cd.sh.int_to_assert = irq_nr;
		cpu->cd.sh.int_level = prio;
	}
}


/*
 *  sh_cpu_interrupt_deassert():
 */
void sh_cpu_interrupt_deassert(struct interrupt *interrupt)
{
	struct cpu *cpu = (struct cpu *) interrupt->extra;
	int irq_nr = interrupt->line;
	int index = irq_nr / 0x20;

	/*  Deassert the interrupt:  */
	if (cpu->cd.sh.int_prio_and_pending[index] & SH_INT_ASSERTED) {
		cpu->cd.sh.int_prio_and_pending[index] &= ~SH_INT_ASSERTED;

		/*  Calculate new interrupt assertion:  */
		cpu->cd.sh.int_to_assert = 0;
		cpu->cd.sh.int_level = 0;

		/*  NOTE/TODO: This is slow, but should hopefully work:  */
		for (index=0; index<0x1000/0x20; index++) {
			uint8_t x = cpu->cd.sh.int_prio_and_pending[index];
			uint8_t prio = x & SH_INT_PRIO_MASK;
			if (x & SH_INT_ASSERTED &&
			    prio > cpu->cd.sh.int_level) {
				cpu->cd.sh.int_to_assert = index * 0x20;
				cpu->cd.sh.int_level = prio;
			}
		}
	}
}


/*
 *  sh_cpu_list_available_types():
 *
 *  Print a list of available SH CPU types.
 */
void sh_cpu_list_available_types(void)
{
	int i = 0, j;
	struct sh_cpu_type_def tdefs[] = SH_CPU_TYPE_DEFS;

	while (tdefs[i].name != NULL) {
		printf("%s", tdefs[i].name);
		for (j=10 - strlen(tdefs[i].name); j>0; j--)
			printf(" ");
		i ++;
		if ((i % 6) == 0 || tdefs[i].name == NULL)
			printf("\n");
	}
}


/*
 *  sh_cpu_dumpinfo():
 */
void sh_cpu_dumpinfo(struct cpu *cpu, bool verbose)
{
	debugmsg(SUBSYS_MACHINE, "cpu", VERBOSITY_INFO,
	    "%s (%s-endian)",
	    cpu->name,
	    cpu->byte_order == EMUL_BIG_ENDIAN? "Big" : "Little");
}


/*
 *  sh_cpu_instruction_has_delayslot():
 *
 *  Return 1 if an opcode is a branch, 0 otherwise.
 */
int sh_cpu_instruction_has_delayslot(struct cpu *cpu, unsigned char *ib)
{
	uint16_t iword = *((uint16_t *)&ib[0]);
	int hi4, lo8; // , lo4

	if (!cpu->is_32bit)
		return 0;

	if (cpu->byte_order == EMUL_BIG_ENDIAN)
		iword = BE16_TO_HOST(iword);
	else
		iword = LE16_TO_HOST(iword);

	hi4 = iword >> 12;
	lo8 = iword & 255;
	// lo4 = iword & 15;

        switch (hi4) {
	case 0x0:
		if (iword == 0x000b)	/*  rts  */
			return 1;
		if (iword == 0x002b)	/*  rte  */
			return 1;
		if (lo8 == 0x03)	/*  bsrf  */
			return 1;
		if (lo8 == 0x23)	/*  braf  */
			return 1;
		break;
	case 0x4:
		switch (lo8) {
		case 0x0b:	/*  jsr  */
		case 0x2b:	/*  jmp  */
			return 1;
		}
		break;
	case 0x8:
		switch ((iword >> 8) & 0xf) {
		case 0xd:	/*  bt/s  */
		case 0xf:	/*  bf/s  */
			return 1;
		}
		break;
	case 0xa:	/*  bra  */
	case 0xb:	/*  bsr  */
		return 1;
	}

	return 0;
}


/*
 *  sh_cpu_register_dump():
 *
 *  Dump cpu registers in a relatively readable format.
 *
 *  gprs: set to non-zero to dump GPRs and some special-purpose registers.
 *  coprocs: set bit 0..3 to dump registers in coproc 0..3.
 */
void sh_cpu_register_dump(struct cpu *cpu, int gprs, int coprocs)
{
	char *symbol;
	uint64_t offset;
	int i, x = cpu->cpu_id;

	if (gprs) {
		/*  Special registers (pc, ...) first:  */
		symbol = get_symbol_name(&cpu->machine->symbol_context,
		    cpu->pc, &offset);

		printf("cpu%i: pc  = 0x%08" PRIx32, x, (uint32_t)cpu->pc);
		printf("  <%s>\n", symbol != NULL? symbol : " no symbol ");

		printf("cpu%i: sr  = 0x%08" PRIx32"  (%s, %s, %s, %s, %s, %s,"
		    " imask=0x%x, %s, %s)\n", x, (int32_t)cpu->cd.sh.sr,
		    (cpu->cd.sh.sr & SH_SR_MD)? "MD" : "!md",
		    (cpu->cd.sh.sr & SH_SR_RB)? "RB" : "!rb",
		    (cpu->cd.sh.sr & SH_SR_BL)? "BL" : "!bl",
		    (cpu->cd.sh.sr & SH_SR_FD)? "FD" : "!fd",
		    (cpu->cd.sh.sr & SH_SR_M)? "M" : "!m",
		    (cpu->cd.sh.sr & SH_SR_Q)? "Q" : "!q",
		    (cpu->cd.sh.sr & SH_SR_IMASK) >> SH_SR_IMASK_SHIFT,
		    (cpu->cd.sh.sr & SH_SR_S)? "S" : "!s",
		    (cpu->cd.sh.sr & SH_SR_T)? "T" : "!t");

		symbol = get_symbol_name(&cpu->machine->symbol_context,
		    cpu->cd.sh.pr, &offset);
		printf("cpu%i: pr  = 0x%08" PRIx32, x, (uint32_t)cpu->cd.sh.pr);
		printf("  <%s>\n", symbol != NULL? symbol : " no symbol ");

		printf("cpu%i: mach = 0x%08" PRIx32"  macl = 0x%08" PRIx32
		    "  gbr = 0x%08" PRIx32"\n", x, (uint32_t)cpu->cd.sh.mach,
		    (uint32_t)cpu->cd.sh.macl, (uint32_t)cpu->cd.sh.gbr);

		for (i=0; i<SH_N_GPRS; i++) {
			if ((i % 4) == 0)
				printf("cpu%i:", x);
			printf(" r%-2i = 0x%08x ", i, (int)cpu->cd.sh.r[i]);
			if ((i % 4) == 3)
				printf("\n");
		}
	}

	if (coprocs & 1) {
		/*  Floating point:  */
		printf("cpu%i: fpscr = 0x%08" PRIx32" (%s,%s,%s)  fpul = 0x%08"
		    PRIx32"\n", x, cpu->cd.sh.fpscr,
		    cpu->cd.sh.fpscr & SH_FPSCR_PR? "PR" : "!pr",
		    cpu->cd.sh.fpscr & SH_FPSCR_SZ? "SZ" : "!sz",
		    cpu->cd.sh.fpscr & SH_FPSCR_FR? "FR" : "!fr",
		    cpu->cd.sh.fpul);

		for (i=0; i<SH_N_FPRS; i++) {
			if ((i % 4) == 0)
				printf("cpu%i:", x);
			printf(" fr%-2i=0x%08x ", i, (int)cpu->cd.sh.fr[i]);
			if ((i % 4) == 3)
				printf("\n");
		}

		for (i=0; i<SH_N_FPRS; i++) {
			if ((i % 4) == 0)
				printf("cpu%i:", x);
			printf(" xf%-2i=0x%08x ", i, (int)cpu->cd.sh.xf[i]);
			if ((i % 4) == 3)
				printf("\n");
		}
	}

	if (coprocs & 2) {
		/*  System registers, etc:  */
		printf("cpu%i: vbr = 0x%08" PRIx32"  sgr = 0x%08" PRIx32
		    "  dbr = 0x%08" PRIx32"\n", x, cpu->cd.sh.vbr, cpu->cd.sh.sgr,
		    cpu->cd.sh.dbr);
		printf("cpu%i: spc = 0x%08" PRIx32"  ssr = 0x%08" PRIx32"\n",
		    x, cpu->cd.sh.spc, cpu->cd.sh.ssr);
		printf("cpu%i: expevt = 0x%" PRIx32"  intevt = 0x%" PRIx32
		    "  tra = 0x%" PRIx32"\n", x, cpu->cd.sh.expevt,
		    cpu->cd.sh.intevt, cpu->cd.sh.tra);

		for (i=0; i<SH_N_GPRS_BANKED; i++) {
			if ((i % 2) == 0)
				printf("cpu%i:", x);
			printf(" r%i_bank = 0x%08x ", i,
			    (int)cpu->cd.sh.r_bank[i]);
			if ((i % 2) == 1)
				printf("\n");
		}
	}
}


/*
 *  sh_cpu_tlbdump():
 *
 *  Called from the debugger to dump the TLB in a readable format.
 *
 *  If rawflag is nonzero, then the TLB contents isn't formated nicely,
 *  just dumped.
 */
void sh_cpu_tlbdump(struct cpu* cpu, int rawflag)
{
	int i, j;
	struct machine* m = cpu->machine;

	for (j=0; j<m->ncpus; j++) {
		if (cpu != m->cpus[j])
			continue;

		for (i=0; i<SH_N_ITLB_ENTRIES; i++)
			printf("cpu%i: itlb_hi_%-2i = 0x%08" PRIx32"  "
			    "itlb_lo_%-2i = 0x%08" PRIx32"\n", j, i,
			    (uint32_t) cpu->cd.sh.itlb_hi[i], i,
			    (uint32_t) cpu->cd.sh.itlb_lo[i]);
		for (i=0; i<SH_N_UTLB_ENTRIES; i++)
			printf("cpu%i: utlb_hi_%-2i = 0x%08" PRIx32"  "
			    "utlb_lo_%-2i = 0x%08" PRIx32"\n", j, i,
			    (uint32_t) cpu->cd.sh.utlb_hi[i], i,
			    (uint32_t) cpu->cd.sh.utlb_lo[i]);
	}
}


/*
 *  sh_update_sr():
 *
 *  Writes a new value to the status register.
 */
void sh_update_sr(struct cpu *cpu, uint32_t new_sr)
{
	uint32_t old_sr = cpu->cd.sh.sr;

	if ((new_sr & SH_SR_RB) != (old_sr & SH_SR_RB)) {
		int i;
		for (i=0; i<SH_N_GPRS_BANKED; i++) {
			uint32_t tmp = cpu->cd.sh.r[i];
			cpu->cd.sh.r[i] = cpu->cd.sh.r_bank[i];
			cpu->cd.sh.r_bank[i] = tmp;
		}
	}

	cpu->cd.sh.sr = new_sr;
}


/*
 *  sh_update_fpscr():
 *
 *  Writes a new value to the floating-point status/control register.
 */
void sh_update_fpscr(struct cpu *cpu, uint32_t new_fpscr)
{
	uint32_t old_fpscr = cpu->cd.sh.fpscr;

	if ((new_fpscr & SH_FPSCR_FR) != (old_fpscr & SH_FPSCR_FR)) {
		int i;
		for (i=0; i<SH_N_FPRS; i++) {
			uint32_t tmp = cpu->cd.sh.fr[i];
			cpu->cd.sh.fr[i] = cpu->cd.sh.xf[i];
			cpu->cd.sh.xf[i] = tmp;
		}
	}

	cpu->cd.sh.fpscr = new_fpscr;
}


/*
 *  sh_exception():
 *
 *  Causes a transfer of control to an exception or interrupt handler.
 *  If intevt > 0, then it is an interrupt, otherwise an exception.
 *
 *  vaddr contains the faulting address, on TLB exceptions.
 */
void sh_exception(struct cpu *cpu, int expevt, int intevt, uint32_t vaddr)
{
	uint32_t vbr = cpu->cd.sh.vbr;

		if (intevt > 0)
		debugmsg(SUBSYS_CPU, "cpu_sh", VERBOSITY_DEBUG, "[ interrupt 0x%03x", intevt);
	else
		debugmsg(SUBSYS_CPU, "cpu_sh", VERBOSITY_DEBUG, "[ exception 0x%03x", expevt);

	debugmsg(SUBSYS_CPU, "cpu_sh", VERBOSITY_DEBUG, ", pc=0x%08" PRIx32" ", (uint32_t)cpu->pc);
	if (intevt == 0)
		debugmsg(SUBSYS_CPU, "cpu_sh", VERBOSITY_DEBUG, "vaddr=0x%08" PRIx32" ", vaddr);

	debugmsg(SUBSYS_CPU, "cpu_sh", VERBOSITY_DEBUG, " ]\n");


	if (cpu->cd.sh.sr & SH_SR_BL) {
		debugmsg(SUBSYS_CPU, "cpu", VERBOSITY_DEBUG, "[ sh_exception(): BL bit already set. ]\n");

		/*  This is actually OK in two cases: a User Break,
		    or on NMI interrupts if a special flag is set?  */
		/*  TODO  */

		expevt = EXPEVT_RESET_POWER;
	}

	if (cpu->is_halted) {
		/*
		 *  If the exception occurred on a 'sleep' instruction, then let
		 *  the instruction following the sleep instruction be the one
		 *  where execution resumes when the interrupt service routine
		 *  returns.
		 */
		cpu->is_halted = false;
		cpu->pc += sizeof(uint16_t);
	}

	if (cpu->delay_slot) {
		cpu->delay_slot = EXCEPTION_IN_DELAY_SLOT;
		cpu->pc -= sizeof(uint16_t);
	}


	/*
	 *  Stuff common to all exceptions:
	 */

	cpu->cd.sh.spc = cpu->pc;
	cpu->cd.sh.ssr = cpu->cd.sh.sr;
	cpu->cd.sh.sgr = cpu->cd.sh.r[15];

	if (intevt > 0) {
		cpu->cd.sh.intevt = intevt;
		expevt = -1;
	} else {
		cpu->cd.sh.expevt = expevt;
	}

	sh_update_sr(cpu, cpu->cd.sh.sr | SH_SR_MD | SH_SR_RB | SH_SR_BL);

	/*  Most exceptions set PC to VBR + 0x100.  */
	cpu->pc = vbr + 0x100;


	/*  Specific cases:  */
	switch (expevt) {

	case -1:	/*  Interrupt  */
		cpu->pc = vbr + 0x600;
		break;

	case EXPEVT_RESET_POWER:
	case EXPEVT_RESET_MANUAL:
		cpu->pc = 0xa0000000;
		cpu->cd.sh.vbr = 0x00000000;
		sh_update_sr(cpu, (cpu->cd.sh.sr | SH_SR_IMASK) & ~SH_SR_FD);
		break;

	case EXPEVT_TLB_MISS_LD:
	case EXPEVT_TLB_MISS_ST:
		cpu->pc = vbr + 0x400;
		// fall through
	case EXPEVT_TLB_PROT_LD:
	case EXPEVT_TLB_PROT_ST:
	case EXPEVT_TLB_MOD:
		cpu->cd.sh.tea = vaddr;
		cpu->cd.sh.pteh &= ~SH4_PTEH_VPN_MASK;
		cpu->cd.sh.pteh |= (vaddr & SH4_PTEH_VPN_MASK);
		break;

	case EXPEVT_TRAPA:
		/*
		 *  Note: The TRA register is already set by the implementation
		 *  of the trapa instruction. See cpu_sh_instr.c for details.
		 *  Here, spc is incremented, so that a return from the trap
		 *  handler transfers control to the instruction _following_
		 *  the trapa.
		 */
		cpu->cd.sh.spc += sizeof(uint16_t);
		break;

	case EXPEVT_RES_INST:
		/*
		 *  Note: Having this code here makes it possible to catch
		 *  reserved instructions; during normal instruction execution,
		 *  these are not very common.
		 */
#if 1
		fatal_error_cpu(cpu, SUBSYS_CPU, "sh", "RESERVED SuperH instruction at spc=%08" PRIx32,
		    cpu->cd.sh.spc);
#else
		break;
#endif

	case EXPEVT_FPU_DISABLE:
		break;

	default:fatal_error_cpu(cpu, SUBSYS_CPU, "cpu", "sh_exception(): exception 0x%x is not yet "
		    "implemented.", expevt);
	}

	sh_pc_to_pointers(cpu);
}


/*
 *  sh_cpu_disassemble_instr():
 *
 *  SHcompact instruction disassembly. The top 4 bits of each 16-bit
 *  instruction word is used as the main opcode. For most instructions, the
 *  lowest 4 or 8 bits then select sub-opcode.
 *
 *  This function convert an instruction word into human readable format,
 *  for instruction tracing.
 *
 *  If running is 1, cpu->pc should be the address of the instruction.
 *
 *  If running is 0, things that depend on the runtime environment (eg.
 *  register contents) will not be shown, and addr will be used instead of
 *  cpu->pc for relative addresses.
 */
int sh_cpu_disassemble_instr(struct cpu *cpu, unsigned char *instr,
	int running, uint64_t dumpaddr)
{
	char *symbol;
	uint64_t offset, addr;
	uint16_t iword;

	if (running)
		dumpaddr = cpu->pc;

	symbol = get_symbol_name(&cpu->machine->symbol_context,
	    dumpaddr, &offset);
	if (symbol != NULL && offset==0)
		printf("<%s>\n", symbol);

	if (cpu->machine->ncpus > 1 && running)
		printf("cpu%i: ", cpu->cpu_id);

	printf("%08" PRIx32, (uint32_t) dumpaddr);

	if (cpu->byte_order == EMUL_BIG_ENDIAN)
		iword = (instr[0] << 8) + instr[1];
	else
		iword = (instr[1] << 8) + instr[0];

	printf(":  %04x %s", iword, cpu->delay_slot? "(d)" : "   ");

	cpu_print_pc_indicator_in_disassembly(cpu, running, dumpaddr);

	const int hi4 = iword >> 12, lo4 = iword & 15, lo8 = iword & 255;
	int r8 = (iword >> 8) & 15, r4 = (iword >> 4) & 15;


	/*
	 *  Decode the instruction:
	 */

	switch (hi4) {
	case 0x0:
		if (lo8 == 0x02)
			printf("stc\tsr,r%i\n", r8);
		else if (lo8 == 0x03)
			printf("bsrf\tr%i\n", r8);
		else if (lo4 >= 4 && lo4 <= 6) {
			if (lo4 == 0x4)
				printf("mov.b\tr%i,@(r0,r%i)", r4, r8);
			else if (lo4 == 0x5)
				printf("mov.w\tr%i,@(r0,r%i)", r4, r8);
			else if (lo4 == 0x6)
				printf("mov.l\tr%i,@(r0,r%i)", r4, r8);
			if (running) {
				addr = cpu->cd.sh.r[0] + cpu->cd.sh.r[r8];
				printf("\t; r0+r%i = ", r8);
				symbol = get_symbol_name(
				    &cpu->machine->symbol_context,
				    addr, &offset);
				if (symbol != NULL)
					printf("<%s>", symbol);
				else
					printf("0x%08" PRIx32, (uint32_t) addr);
			}
			printf("\n");
		} else if (lo4 == 0x7)
			printf("mul.l\tr%i,r%i\n", r4, r8);
		else if (iword == 0x0008)
			printf("clrt\n");
		else if (iword == 0x0009)
			printf("nop\n");
		else if (lo8 == 0x0a)
			printf("sts\tmach,r%i\n", r8);
		else if (iword == 0x000b)
			printf("rts\n");
		else if (lo4 >= 0xc && lo4 <= 0xe) {
			if (lo4 == 0xc)
				printf("mov.b\t@(r0,r%i),r%i", r4, r8);
			else if (lo4 == 0xd)
				printf("mov.w\t@(r0,r%i),r%i", r4, r8);
			else if (lo4 == 0xe)
				printf("mov.l\t@(r0,r%i),r%i", r4, r8);
			if (running) {
				addr = cpu->cd.sh.r[0] + cpu->cd.sh.r[r4];
				printf("\t; r0+r%i = ", r4);
				symbol = get_symbol_name(
				    &cpu->machine->symbol_context,
				    addr, &offset);
				if (symbol != NULL)
					printf("<%s>", symbol);
				else
					printf("0x%08" PRIx32, (uint32_t) addr);
			}
			printf("\n");
		} else if (lo8 == 0x12)
			printf("stc\tgbr,r%i\n", r8);
		else if (iword == 0x0018)
			printf("sett\n");
		else if (iword == 0x0019)
			printf("div0u\n");
		else if (lo8 == 0x1a)
			printf("sts\tmacl,r%i\n", r8);
		else if (iword == 0x001b)
			printf("sleep\n");
		else if (lo8 == 0x22)
			printf("stc\tvbr,r%i\n", r8);
		else if (lo8 == 0x23)
			printf("braf\tr%i\n", r8);
		else if (iword == 0x0028)
			printf("clrmac\n");
		else if (lo8 == 0x29)
			printf("movt\tr%i\n", r8);
		else if (lo8 == 0x2a)
			printf("sts\tpr,r%i\n", r8);
		else if (iword == 0x002b)
			printf("rte\n");
		else if (lo8 == 0x32)
			printf("stc\tssr,r%i\n", r8);
		else if (iword == 0x0038)
			printf("ldtlb\n");
		else if (iword == 0x003b)
			printf("brk\n");
		else if (lo8 == 0x42)
			printf("stc\tspc,r%i\n", r8);
		else if (iword == 0x0048)
			printf("clrs\n");
		else if (iword == 0x0058)
			printf("sets\n");
		else if (lo8 == 0x5a)
			printf("sts\tfpul,r%i\n", r8);
		else if (lo8 == 0x6a)
			printf("sts\tfpscr,r%i\n", r8);
		else if ((lo8 & 0x8f) == 0x82)
			printf("stc\tr%i_bank,r%i\n", (lo8 >> 4) & 7, r8);
		else if (lo8 == 0x83)
			printf("pref\t@r%i\n", r8);
		else if (lo8 == 0x93)
			printf("ocbi\t@r%i\n", r8);
		else if (lo8 == 0xa3)
			printf("ocbp\t@r%i\n", r8);
		else if (lo8 == 0xb3)
			printf("ocbwb\t@r%i\n", r8);
		else if (lo8 == 0xc3)
			printf("movca.l\tr0,@r%i\n", r8);
		else if (lo8 == 0xfa)
			printf("stc\tdbr,r%i\n", r8);
		else if (iword == SH_INVALID_INSTR)
			printf("gxemul_dreamcast_prom_emul\n");
		else
			printf("UNIMPLEMENTED hi4=0x%x, lo8=0x%02x\n", hi4, lo8);
		break;
	case 0x1:
		printf("mov.l\tr%i,@(%i,r%i)", r4, lo4 * 4, r8);
		if (running) {
			addr = cpu->cd.sh.r[r8] + lo4 * 4;
			printf("\t; r%i+%i = ", r8, lo4 * 4);
			symbol = get_symbol_name(&cpu->machine->symbol_context,
			    addr, &offset);
			if (symbol != NULL)
				printf("<%s>", symbol);
			else
				printf("0x%08" PRIx32, (uint32_t) addr);
		}
		printf("\n");
		break;
	case 0x2:
		if (lo4 == 0x0)
			printf("mov.b\tr%i,@r%i", r4, r8);
		else if (lo4 == 0x1)
			printf("mov.w\tr%i,@r%i", r4, r8);
		else if (lo4 == 0x2)
			printf("mov.l\tr%i,@r%i", r4, r8);
		else if (lo4 == 0x4)
			printf("mov.b\tr%i,@-r%i", r4, r8);
		else if (lo4 == 0x5)
			printf("mov.w\tr%i,@-r%i", r4, r8);
		else if (lo4 == 0x6)
			printf("mov.l\tr%i,@-r%i", r4, r8);
		else if (lo4 == 0x7)
			printf("div0s\tr%i,r%i", r4, r8);
		else if (lo4 == 0x8)
			printf("tst\tr%i,r%i", r4, r8);
		else if (lo4 == 0x9)
			printf("and\tr%i,r%i", r4, r8);
		else if (lo4 == 0xa)
			printf("xor\tr%i,r%i", r4, r8);
		else if (lo4 == 0xb)
			printf("or\tr%i,r%i", r4, r8);
		else if (lo4 == 0xc)
			printf("cmp/str\tr%i,r%i", r4, r8);
		else if (lo4 == 0xd)
			printf("xtrct\tr%i,r%i", r4, r8);
		else if (lo4 == 0xe)
			printf("mulu.w\tr%i,r%i", r4, r8);
		else if (lo4 == 0xf)
			printf("muls.w\tr%i,r%i", r4, r8);
		else
			printf("UNIMPLEMENTED hi4=0x%x, lo8=0x%02x", hi4, lo8);
		if (running && lo4 <= 6) {
			printf("\t; r%i = 0x%08" PRIx32, r8, cpu->cd.sh.r[r8]);
		}
		printf("\n");
		break;
	case 0x3:
		if (lo4 == 0x0)
			printf("cmp/eq\tr%i,r%i\n", r4, r8);
		else if (lo4 == 0x2)
			printf("cmp/hs\tr%i,r%i\n", r4, r8);
		else if (lo4 == 0x3)
			printf("cmp/ge\tr%i,r%i\n", r4, r8);
		else if (lo4 == 0x4)
			printf("div1\tr%i,r%i\n", r4, r8);
		else if (lo4 == 0x5)
			printf("dmulu.l\tr%i,r%i\n", r4, r8);
		else if (lo4 == 0x6)
			printf("cmp/hi\tr%i,r%i\n", r4, r8);
		else if (lo4 == 0x7)
			printf("cmp/gt\tr%i,r%i\n", r4, r8);
		else if (lo4 == 0x8)
			printf("sub\tr%i,r%i\n", r4, r8);
		else if (lo4 == 0xa)
			printf("subc\tr%i,r%i\n", r4, r8);
		else if (lo4 == 0xb)
			printf("subv\tr%i,r%i\n", r4, r8);
		else if (lo4 == 0xc)
			printf("add\tr%i,r%i\n", r4, r8);
		else if (lo4 == 0xd)
			printf("dmuls.l\tr%i,r%i\n", r4, r8);
		else if (lo4 == 0xe)
			printf("addc\tr%i,r%i\n", r4, r8);
		else if (lo4 == 0xf)
			printf("addv\tr%i,r%i\n", r4, r8);
		else
			printf("UNIMPLEMENTED hi4=0x%x, lo8=0x%02x\n", hi4, lo8);
		break;
	case 0x4:
		if (lo8 == 0x00)
			printf("shll\tr%i\n", r8);
		else if (lo8 == 0x01)
			printf("shlr\tr%i\n", r8);
		else if (lo8 == 0x02)
			printf("sts.l\tmach,@-r%i\n", r8);
		else if (lo8 == 0x03)
			printf("stc.l\tsr,@-r%i\n", r8);
		else if (lo8 == 0x04)
			printf("rotl\tr%i\n", r8);
		else if (lo8 == 0x05)
			printf("rotr\tr%i\n", r8);
		else if (lo8 == 0x06)
			printf("lds.l\t@r%i+,mach\n", r8);
		else if (lo8 == 0x07)
			printf("ldc.l\t@r%i+,sr\n", r8);
		else if (lo8 == 0x08)
			printf("shll2\tr%i\n", r8);
		else if (lo8 == 0x09)
			printf("shlr2\tr%i\n", r8);
		else if (lo8 == 0x0a)
			printf("lds\tr%i,mach\n", r8);
		else if (lo8 == 0x0b)
			printf("jsr\t@r%i\n", r8);
		else if (lo4 == 0xc)
			printf("shad\tr%i,r%i\n", r4, r8);
		else if (lo4 == 0xd)
			printf("shld\tr%i,r%i\n", r4, r8);
		else if (lo8 == 0x0e)
			printf("ldc\tr%i,sr\n", r8);
		else if (lo8 == 0x10)
			printf("dt\tr%i\n", r8);
		else if (lo8 == 0x11)
			printf("cmp/pz\tr%i\n", r8);
		else if (lo8 == 0x12)
			printf("sts.l\tmacl,@-r%i\n", r8);
		else if (lo8 == 0x13)
			printf("stc.l\tgbr,@-r%i\n", r8);
		else if (lo8 == 0x15)
			printf("cmp/pl\tr%i\n", r8);
		else if (lo8 == 0x16)
			printf("lds.l\t@r%i+,macl\n", r8);
		else if (lo8 == 0x17)
			printf("ldc.l\t@r%i+,gbr\n", r8);
		else if (lo8 == 0x18)
			printf("shll8\tr%i\n", r8);
		else if (lo8 == 0x19)
			printf("shlr8\tr%i\n", r8);
		else if (lo8 == 0x1a)
			printf("lds\tr%i,macl\n", r8);
		else if (lo8 == 0x1b)
			printf("tas.b\t@r%i\n", r8);
		else if (lo8 == 0x1e)
			printf("ldc\tr%i,gbr\n", r8);
		else if (lo8 == 0x20)
			printf("shal\tr%i\n", r8);
		else if (lo8 == 0x21)
			printf("shar\tr%i\n", r8);
		else if (lo8 == 0x22)
			printf("sts.l\tpr,@-r%i\n", r8);
		else if (lo8 == 0x23)
			printf("stc.l\tvbr,@-r%i\n", r8);
		else if (lo8 == 0x24)
			printf("rotcl\tr%i\n", r8);
		else if (lo8 == 0x25)
			printf("rotcr\tr%i\n", r8);
		else if (lo8 == 0x26)
			printf("lds.l\t@r%i+,pr\n", r8);
		else if (lo8 == 0x27)
			printf("ldc.l\t@r%i+,vbr\n", r8);
		else if (lo8 == 0x28)
			printf("shll16\tr%i\n", r8);
		else if (lo8 == 0x29)
			printf("shlr16\tr%i\n", r8);
		else if (lo8 == 0x2a)
			printf("lds\tr%i,pr\n", r8);
		else if (lo8 == 0x2b) {
			printf("jmp\t@r%i", r8);
			if (running) {
				symbol = get_symbol_name(
				    &cpu->machine->symbol_context,
				    cpu->cd.sh.r[r8], &offset);
				if (symbol != NULL)
					printf("\t\t; <%s>", symbol);
			}
			printf("\n");
		} else if (lo8 == 0x2e)
			printf("ldc\tr%i,vbr\n", r8);
		else if (lo8 == 0x33)
			printf("stc.l\tssr,@-r%i\n", r8);
		else if (lo8 == 0x37)
			printf("ldc.l\t@r%i+,ssr\n", r8);
		else if (lo8 == 0x3e)
			printf("ldc\tr%i,ssr\n", r8);
		else if (lo8 == 0x43)
			printf("stc.l\tspc,@-r%i\n", r8);
		else if (lo8 == 0x47)
			printf("ldc.l\t@r%i+,spc\n", r8);
		else if (lo8 == 0x4e)
			printf("ldc\tr%i,spc\n", r8);
		else if (lo8 == 0x52)
			printf("sts.l\tfpul,@-r%i\n", r8);
		else if (lo8 == 0x56)
			printf("lds.l\t@r%i+,fpul\n", r8);
		else if (lo8 == 0x5a)
			printf("lds\tr%i,fpul\n", r8);
		else if (lo8 == 0x62)
			printf("sts.l\tfpscr,@-r%i\n", r8);
		else if (lo8 == 0x66)
			printf("lds.l\t@r%i+,fpscr\n", r8);
		else if (lo8 == 0x6a)
			printf("lds\tr%i,fpscr\n", r8);
		else if ((lo8 & 0x8f) == 0x83)
			printf("stc.l\tr%i_bank,@-r%i\n", (lo8 >> 4) & 7, r8);
		else if ((lo8 & 0x8f) == 0x87)
			printf("ldc.l\t@r%i,r%i_bank\n", r8, (lo8 >> 4) & 7);
		else if ((lo8 & 0x8f) == 0x8e)
			printf("ldc\tr%i,r%i_bank\n", r8, (lo8 >> 4) & 7);
		else if (lo8 == 0xfa)
			printf("ldc\tr%i,dbr\n", r8);
		else
			printf("UNIMPLEMENTED hi4=0x%x, lo8=0x%02x\n", hi4, lo8);
		break;
	case 0x5:
		printf("mov.l\t@(%i,r%i),r%i", lo4 * 4, r4, r8);
		if (running) {
			addr = cpu->cd.sh.r[r4] + lo4 * 4;
			symbol = get_symbol_name(&cpu->machine->symbol_context, addr, &offset);
			if (symbol != NULL)
				printf("\t; r%i+%i <%s>", r4, lo4 * 4, symbol);
			else
				printf("\t; r%i+%i = 0x%08" PRIx32, r4, lo4 * 4, (int)addr);
		}
		printf("\n");
		break;
	case 0x6:
		if (lo4 == 0x0)
			printf("mov.b\t@r%i,r%i", r4, r8);
		else if (lo4 == 0x1)
			printf("mov.w\t@r%i,r%i", r4, r8);
		else if (lo4 == 0x2)
			printf("mov.l\t@r%i,r%i", r4, r8);
		else if (lo4 == 0x3)
			printf("mov\tr%i,r%i", r4, r8);
		else if (lo4 == 0x4)
			printf("mov.b\t@r%i+,r%i", r4, r8);
		else if (lo4 == 0x5)
			printf("mov.w\t@r%i+,r%i", r4, r8);
		else if (lo4 == 0x6)
			printf("mov.l\t@r%i+,r%i", r4, r8);
		else if (lo4 == 0x7)
			printf("not\tr%i,r%i", r4, r8);
		else if (lo4 == 0x8)
			printf("swap.b\tr%i,r%i", r4, r8);
		else if (lo4 == 0x9)
			printf("swap.w\tr%i,r%i", r4, r8);
		else if (lo4 == 0xa)
			printf("negc\tr%i,r%i", r4, r8);
		else if (lo4 == 0xb)
			printf("neg\tr%i,r%i", r4, r8);
		else if (lo4 == 0xc)
			printf("extu.b\tr%i,r%i", r4, r8);
		else if (lo4 == 0xd)
			printf("extu.w\tr%i,r%i", r4, r8);
		else if (lo4 == 0xe)
			printf("exts.b\tr%i,r%i", r4, r8);
		else if (lo4 == 0xf)
			printf("exts.w\tr%i,r%i", r4, r8);
		else
			printf("UNIMPLEMENTED hi4=0x%x, lo8=0x%02x", hi4, lo8);
		if (running && lo4 < 8 && (lo4 & 3) < 3) {
			printf("\t; r%i = 0x%08" PRIx32, r4, cpu->cd.sh.r[r4]);
		}
		printf("\n");
		break;
	case 0x7:
		printf("add\t#%i,r%i\n", (int8_t)lo8, r8);
		break;
	case 0x8:
		if (r8 == 0 || r8 == 4) {
			if (r8 == 0x0)
				printf("mov.b\tr0,@(%i,r%i)", lo4, r4);
			else if (r8 == 0x4)
				printf("mov.b\t@(%i,r%i),r0", lo4, r4);
			if (running) {
				printf("\t; r%i+%i = 0x%08" PRIx32, r4, lo4,
				    cpu->cd.sh.r[r4] + lo4);
			}
			printf("\n");
		} else if (r8 == 1 || r8 == 5) {
			if (r8 == 0x1)
				printf("mov.w\tr0,@(%i,r%i)", lo4 * 2, r4);
			else if (r8 == 0x5)
				printf("mov.w\t@(%i,r%i),r0", lo4 * 2, r4);
			if (running) {
				printf("\t; r%i+%i = 0x%08" PRIx32, r4, lo4 * 2,
				    cpu->cd.sh.r[r4] + lo4 * 2);
			}
			printf("\n");
		} else if (r8 == 0x8) {
			printf("cmp/eq\t#%i,r0\n", (int8_t)lo8);
		} else if (r8 == 0x9 || r8 == 0xb || r8 == 0xd || r8 == 0xf) {
			addr = (int8_t)lo8;
			addr = dumpaddr + 4 + (addr << 1);
			printf("b%s%s\t0x%x",
			    (r8 == 0x9 || r8 == 0xd)? "t" : "f",
			    (r8 == 0x9 || r8 == 0xb)? "" : "/s", (int)addr);
			symbol = get_symbol_name(&cpu->machine->symbol_context,
			    addr, &offset);
			if (symbol != NULL)
				printf("\t; <%s>", symbol);
			printf("\n");
		} else
			printf("UNIMPLEMENTED hi4=0x%x,0x%x\n", hi4, r8);
		break;
	case 0x9:
	case 0xd:
		addr = lo8 * (hi4==9? 2 : 4);
		addr += (dumpaddr & ~(hi4==9? 1 : 3)) + 4;
		printf("mov.%s\t0x%x,r%i", hi4==9? "w":"l", (int)addr, r8);
		symbol = get_symbol_name(&cpu->machine->symbol_context,
		    addr, &offset);
		if (symbol != NULL)
			printf("\t; <%s>", symbol);
		printf("\n");
		break;
	case 0xa:
	case 0xb:
		addr = (int32_t)(int16_t)((iword & 0xfff) << 4);
		addr = ((int32_t)addr >> 3);
		addr += dumpaddr + 4;
		printf("%s\t0x%x", hi4==0xa? "bra":"bsr", (int)addr);

		symbol = get_symbol_name(&cpu->machine->symbol_context,
		    addr, &offset);
		if (symbol != NULL)
			printf("\t; <%s>", symbol);
		printf("\n");
		break;
	case 0xc:
		if (r8 == 0x0)
			printf("mov.b\tr0,@(%i,gbr)\n", lo8);
		else if (r8 == 0x1)
			printf("mov.w\tr0,@(%i,gbr)\n", lo8 * 2);
		else if (r8 == 0x2)
			printf("mov.l\tr0,@(%i,gbr)\n", lo8 * 4);
		else if (r8 == 0x3)
			printf("trapa\t#%i\n", (uint8_t)lo8);
		else if (r8 == 0x4)
			printf("mov.b\t(%i,gbr),r0\n", lo8);
		else if (r8 == 0x5)
			printf("mov.w\t(%i,gbr),r0\n", lo8 * 2);
		else if (r8 == 0x6)
			printf("mov.l\t(%i,gbr),r0\n", lo8 * 4);
		else if (r8 == 0x7) {
			addr = lo8 * 4 + (dumpaddr & ~3) + 4;
			printf("mova\t0x%x,r0\n", (int)addr);
		} else if (r8 == 0x8)
			printf("tst\t#%i,r0\n", (uint8_t)lo8);
		else if (r8 == 0x9)
			printf("and\t#%i,r0\n", (uint8_t)lo8);
		else if (r8 == 0xa)
			printf("xor\t#%i,r0\n", (uint8_t)lo8);
		else if (r8 == 0xb)
			printf("or\t#%i,r0\n", (uint8_t)lo8);
		else if (r8 == 0xc)
			printf("tst.b\t#%i,@(r0,gbr)\n", (uint8_t)lo8);
		else if (r8 == 0xd)
			printf("and.b\t#%i,@(r0,gbr)\n", (uint8_t)lo8);
		else if (r8 == 0xe)
			printf("xor.b\t#%i,@(r0,gbr)\n", (uint8_t)lo8);
		else if (r8 == 0xf)
			printf("or.b\t#%i,@(r0,gbr)\n", (uint8_t)lo8);
		else
			printf("UNIMPLEMENTED hi4=0x%x,0x%x\n", hi4, r8);
		break;
	case 0xe:
		printf("mov\t#%i,r%i\n", (int8_t)lo8, r8);
		break;
	case 0xf:
		if (lo4 == 0x0)
			printf("fadd\t%sr%i,%sr%i\n",
			    cpu->cd.sh.fpscr & SH_FPSCR_PR? "d" : "f", r4,
			    cpu->cd.sh.fpscr & SH_FPSCR_PR? "d" : "f", r8);
		else if (lo4 == 0x1)
			printf("fsub\t%sr%i,%sr%i\n",
			    cpu->cd.sh.fpscr & SH_FPSCR_PR? "d" : "f", r4,
			    cpu->cd.sh.fpscr & SH_FPSCR_PR? "d" : "f", r8);
		else if (lo4 == 0x2)
			printf("fmul\t%sr%i,%sr%i\n",
			    cpu->cd.sh.fpscr & SH_FPSCR_PR? "d" : "f", r4,
			    cpu->cd.sh.fpscr & SH_FPSCR_PR? "d" : "f", r8);
		else if (lo4 == 0x3)
			printf("fdiv\t%sr%i,%sr%i\n",
			    cpu->cd.sh.fpscr & SH_FPSCR_PR? "d" : "f", r4,
			    cpu->cd.sh.fpscr & SH_FPSCR_PR? "d" : "f", r8);
		else if (lo4 == 0x4)
			printf("fcmp/eq\t%sr%i,%sr%i\n",
			    cpu->cd.sh.fpscr & SH_FPSCR_PR? "d" : "f", r4,
			    cpu->cd.sh.fpscr & SH_FPSCR_PR? "d" : "f", r8);
		else if (lo4 == 0x5)
			printf("fcmp/gt\t%sr%i,%sr%i\n",
			    cpu->cd.sh.fpscr & SH_FPSCR_PR? "d" : "f", r4,
			    cpu->cd.sh.fpscr & SH_FPSCR_PR? "d" : "f", r8);
		else if (lo4 == 0x6) {
			const char *n = "fr";
			if (cpu->cd.sh.fpscr & SH_FPSCR_SZ) {
				n = (r8 & 1)? "xd" : "dr";
				r8 &= ~1;
			}
			printf("fmov\t@(r0,r%i),%s%i\n", r4, n, r8);
		} else if (lo4 == 0x7) {
			const char *n = "fr";
			if (cpu->cd.sh.fpscr & SH_FPSCR_SZ) {
				n = (r4 & 1)? "xd" : "dr";
				r4 &= ~1;
			}
			printf("fmov\t%s%i,@(r0,r%i)\n", n, r4, r8);
		} else if (lo4 == 0x8) {
			const char *n = "fr";
			if (cpu->cd.sh.fpscr & SH_FPSCR_SZ) {
				n = (r8 & 1)? "xd" : "dr";
				r8 &= ~1;
			}
			printf("fmov\t@r%i,%s%i\n", r4, n, r8);
		} else if (lo4 == 0x9) {
			const char *n = "fr";
			if (cpu->cd.sh.fpscr & SH_FPSCR_SZ) {
				n = (r8 & 1)? "xd" : "dr";
				r8 &= ~1;
			}
			printf("fmov\t@r%i+,%s%i\n", r4, n, r8);
		} else if (lo4 == 0xa) {
			const char *n = "fr";
			if (cpu->cd.sh.fpscr & SH_FPSCR_SZ) {
				n = (r4 & 1)? "xd" : "dr";
				r4 &= ~1;
			}
			printf("fmov\t%s%i,@r%i\n", n, r4, r8);
		} else if (lo4 == 0xb) {
			const char *n = "fr";
			if (cpu->cd.sh.fpscr & SH_FPSCR_SZ) {
				n = (r4 & 1)? "xd" : "dr";
				r4 &= ~1;
			}
			printf("fmov\t%s%i,@-r%i\n", n, r4, r8);
		} else if (lo4 == 0xc) {
			const char *n1 = "fr", *n2 = "fr";
			if (cpu->cd.sh.fpscr & SH_FPSCR_SZ) {
				n1 = (r4 & 1)? "xd" : "dr";
				n2 = (r8 & 1)? "xd" : "dr";
				r4 &= ~1; r8 &= ~1;
			}
			printf("fmov\t%s%i,%s%i\n", n1, r4, n2, r8);
		} else if (lo8 == 0x0d)
			printf("fsts\tfpul,fr%i\n", r8);
		else if (lo8 == 0x1d)
			printf("flds\tfr%i,fpul\n", r8);
		else if (lo8 == 0x2d)
			printf("float\tfpul,%sr%i\n",
			    cpu->cd.sh.fpscr & SH_FPSCR_PR? "d" : "f", r8);
		else if (lo8 == 0x3d)
			printf("ftrc\t%sr%i,fpul\n",
			    cpu->cd.sh.fpscr & SH_FPSCR_PR? "d" : "f", r8);
		else if (lo8 == 0x4d)
			printf("fneg\t%sr%i\n",
			    cpu->cd.sh.fpscr & SH_FPSCR_PR? "d" : "f", r8);
		else if (lo8 == 0x5d)
			printf("fabs\t%sr%i\n",
			    cpu->cd.sh.fpscr & SH_FPSCR_PR? "d" : "f", r8);
		else if (lo8 == 0x6d)
			printf("fsqrt\t%sr%i\n",
			    cpu->cd.sh.fpscr & SH_FPSCR_PR? "d" : "f", r8);
		else if (lo8 == 0x7d)
			printf("fsrra\tfr%i\n", r8);
		else if (lo8 == 0x8d)
			printf("fldi0\tfr%i\n", r8);
		else if (lo8 == 0x9d)
			printf("fldi1\tfr%i\n", r8);
		else if (lo8 == 0xad)
			printf("fcnvsd\tfpul,dr%i\n", r8);
		else if (lo8 == 0xbd)
			printf("fcnvds\tdr%i,fpul\n", r8);
		else if (lo8 == 0xed)
			printf("fipr\tfv%i,fv%i\n", (r8 & 3) << 2, r8 & 0xc);
		else if ((iword & 0x01ff) == 0x00fd)
			printf("fsca\tfpul,dr%i\n", r8);
		else if (iword == 0xf3fd)
			printf("fschg\n");
		else if (iword == 0xfbfd)
			printf("frchg\n");
		else if ((iword & 0xf3ff) == 0xf1fd)
			printf("ftrv\txmtrx,fv%i\n", r8 & 0xc);
		else if (lo4 == 0xe)
			printf("fmac\tfr0,fr%i,fr%i\n", r4, r8);
		else
			printf("UNIMPLEMENTED hi4=0x%x,0x%x\n", hi4, lo8);
		break;
	default:printf("UNIMPLEMENTED hi4=0x%x\n", hi4);
	}

	return sizeof(iword);
}


#include "tmp_sh_tail.c"

