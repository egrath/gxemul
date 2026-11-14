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
 *  COMMENT: Symbios (NCR) 53C810 PCI SCSI I/O Processor
 *
 *  The operating registers (in the I/O and the memory BAR, 128 bytes,
 *  mirrored in the upper half), the SCRIPTS processor, and a SCSI bus with
 *  the emulated SCSI disks (diskimage) as targets. The configuration space
 *  side is in bus_pci.c (PCIINIT(sym53c810)), which tells this device where
 *  its BARs are.
 *
 *  The chip is the initiator only; targets don't disconnect in the middle
 *  of a command (so nothing ever reselects the chip), take any message out
 *  bytes the chip sends while it asserts ATN, agree to synchronous transfer
 *  requests, and answer wide transfer requests with 8 bits. Bus timing,
 *  parity, the SCSI and DMA FIFOs (they are always empty when looked at)
 *  and target mode are not emulated. SCRIPTS run from the device tick (and
 *  at once when started), a number of instructions at a time.
 *
 *  SCRIPTS fetch instructions and move data as a PCI bus master; the host
 *  bridge translates the bus addresses (bus_pci_dma_translate()). The chip's
 *  own memory BAR is decoded, too: on the 53C810 (which has no LOAD and
 *  STORE) SCRIPTS load and save registers with memory moves to and from it.
 *
 *  See the "SYM53C810A PCI-SCSI I/O Processor Data Manual" for the
 *  registers and the SCRIPTS instruction set.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bus_pci.h"
#include "cpu.h"
#include "device.h"
#include "devices.h"
#include "diskimage.h"
#include "interrupt.h"
#include "machine.h"
#include "memory.h"
#include "misc.h"


#define	SYM_NREGS		0x80
#define	SYM_BAR_SIZE		0x100

#define	SYM_TICK_SHIFT		14
#define	SYM_INSTRS_PER_RUN	1000

/*  Registers:  */
#define	SCNTL0		0x00
#define	SCNTL1		0x01
#define	SCNTL2		0x02
#define	SCNTL3		0x03
#define	SCID		0x04
#define	SXFER		0x05
#define	SDID		0x06
#define	GPREG		0x07
#define	SFBR		0x08
#define	SOCL		0x09
#define	SSID		0x0a
#define	SBCL		0x0b
#define	DSTAT		0x0c
#define	SSTAT0		0x0d
#define	SSTAT1		0x0e
#define	SSTAT2		0x0f
#define	DSA		0x10
#define	ISTAT		0x14
#define	CTEST0		0x18
#define	CTEST1		0x19
#define	CTEST2		0x1a
#define	CTEST3		0x1b
#define	TEMP		0x1c
#define	DFIFO		0x20
#define	CTEST4		0x21
#define	CTEST5		0x22
#define	CTEST6		0x23
#define	DBC		0x24	/*  3 bytes  */
#define	DCMD		0x27
#define	DNAD		0x28
#define	DSP		0x2c
#define	DSPS		0x30
#define	SCRATCHA	0x34
#define	DMODE		0x38
#define	DIEN		0x39
#define	SBR		0x3a
#define	DCNTL		0x3b
#define	ADDER		0x3c
#define	SIEN0		0x40
#define	SIEN1		0x41
#define	SIST0		0x42
#define	SIST1		0x43
#define	SLPAR		0x44
#define	MACNTL		0x46
#define	GPCNTL		0x47
#define	STIME0		0x48
#define	STIME1		0x49
#define	RESPID		0x4a
#define	STEST0		0x4c
#define	STEST1		0x4d
#define	STEST2		0x4e
#define	STEST3		0x4f
#define	SIDL		0x50
#define	SODL		0x54
#define	SBDL		0x58
#define	SCRATCHB	0x5c

#define	SCNTL0_TRG	0x01
#define	SCNTL1_RST	0x08
#define	SCNTL1_CON	0x10

/*  SOCL and SBCL; the phase is in the low three bits.  */
#define	SBCL_REQ	0x80
#define	SBCL_ACK	0x40
#define	SBCL_BSY	0x20
#define	SBCL_SEL	0x10
#define	SBCL_ATN	0x08

#define	DSTAT_DFE	0x80
#define	DSTAT_MDPE	0x40
#define	DSTAT_BF	0x20
#define	DSTAT_ABRT	0x10
#define	DSTAT_SSI	0x08
#define	DSTAT_SIR	0x04
#define	DSTAT_IID	0x01

#define	ISTAT_ABRT	0x80
#define	ISTAT_SRST	0x40
#define	ISTAT_SIGP	0x20
#define	ISTAT_SEM	0x10
#define	ISTAT_CON	0x08
#define	ISTAT_INTF	0x04
#define	ISTAT_SIP	0x02
#define	ISTAT_DIP	0x01

#define	SIST0_MA	0x80
#define	SIST0_CMP	0x40
#define	SIST0_SEL	0x20
#define	SIST0_RSL	0x10
#define	SIST0_SGE	0x08
#define	SIST0_UDC	0x04
#define	SIST0_RST	0x02
#define	SIST0_PAR	0x01
#define	SIST1_STO	0x04
#define	SIST1_GEN	0x02
#define	SIST1_HTH	0x01

/*  SCSI interrupts that halt SCRIPTS, and so are seen even if masked:  */
#define	SIST0_FATAL	(SIST0_MA | SIST0_SGE | SIST0_UDC | SIST0_RST | \
			    SIST0_PAR)
#define	SIST1_FATAL	(SIST1_STO)

#define	CTEST2_SIGP	0x40
#define	DMODE_MAN	0x01
#define	DCNTL_SSM	0x10
#define	DCNTL_STD	0x04
#define	DCNTL_IRQD	0x02

#define	CTEST3_REVISION	0x20	/*  as the PCI revision ID, 2  */

/*  SCSI bus phases (MSG, C/D, I/O):  */
#define	PHASE_DATA_OUT	0
#define	PHASE_DATA_IN	1
#define	PHASE_COMMAND	2
#define	PHASE_STATUS	3
#define	PHASE_MSG_OUT	6
#define	PHASE_MSG_IN	7

static const char *phase_name[8] = { "DATA_OUT", "DATA_IN", "COMMAND",
	"STATUS", "RESERVED_OUT", "RESERVED_IN", "MSG_OUT", "MSG_IN" };

#define	MSG_COMMAND_COMPLETE	0x00
#define	MSG_EXTENDED		0x01
#define	MSG_ABORT		0x06
#define	MSG_REJECT		0x07
#define	MSG_NOP			0x08
#define	MSG_DEVICE_RESET	0x0c
#define	MSG_ABORT_TAG		0x0d
#define	MSG_CLEAR_QUEUE		0x0e
#define	MSG_IDENTIFY		0x80

#define	MSG_EXT_SDTR		0x01
#define	MSG_EXT_WDTR		0x03


/*  The target that is connected to the chip:  */
struct sym_target {
	int			connected;
	int			id;
	int			lun;
	int			phase;
	int			atn;		/*  the chip asserts ATN  */
	int			ack_held;	/*  after a message in byte  */
	int			disconnect_pending;

	unsigned char		msg_out[64];
	int			msg_out_len;

	unsigned char		cmd[16];
	int			cmd_len;
	int			cmd_done;

	struct scsi_transfer	*xfer;
	int			data_out_pending;
	size_t			data_in_ofs;
	int			status;
	int			status_done;
	int			complete_sent;

	unsigned char		msg_in[16];
	int			msg_in_len;
	int			msg_in_ofs;
};

struct sym53c810_data {
	struct machine		*machine;
	struct memory		*mem;
	struct pci_data		*pci;
	int			subsys;

	struct interrupt	irq;
	int			irq_asserted;

	/*  Where the registers are:  */
	uint64_t		io_addr, mem_addr;
	int			io_mapped, mem_mapped;
	uint32_t		mem_busaddr;	/*  the memory BAR  */

	uint8_t			reg[SYM_NREGS];
	int			carry;

	int			running;	/*  SCRIPTS  */
	int			wait_reselect;
	int			in_scripts;	/*  register access from SCRIPTS  */
	uint32_t		insn_addr;	/*  of the current instruction  */

	struct sym_target	t;
};


static uint32_t reg32(struct sym53c810_data *d, int r)
{
	return d->reg[r] | (d->reg[r + 1] << 8) | (d->reg[r + 2] << 16) |
	    ((uint32_t) d->reg[r + 3] << 24);
}

static void set_reg32(struct sym53c810_data *d, int r, uint32_t v)
{
	d->reg[r] = v;
	d->reg[r + 1] = v >> 8;
	d->reg[r + 2] = v >> 16;
	d->reg[r + 3] = v >> 24;
}

static void set_dbc(struct sym53c810_data *d, uint32_t v)
{
	d->reg[DBC] = v;
	d->reg[DBC + 1] = v >> 8;
	d->reg[DBC + 2] = v >> 16;
}

static int32_t sext24(uint32_t v)
{
	return (int32_t) (v << 8) >> 8;
}


/*
 *  Interrupts: ISTAT's DIP for DMA interrupts (DSTAT), SIP for SCSI
 *  interrupts (SIST0/1) that are enabled, or fatal. The IRQ line is asserted
 *  for enabled interrupts and INTF.
 */
static void sym_update_irq(struct sym53c810_data *d)
{
	int dma = d->reg[DSTAT] & 0x7f;
	int scsi_fatal = (d->reg[SIST0] & SIST0_FATAL) ||
	    (d->reg[SIST1] & SIST1_FATAL);
	int scsi_enabled = (d->reg[SIST0] & d->reg[SIEN0]) ||
	    (d->reg[SIST1] & d->reg[SIEN1] & 7);
	int assert;

	d->reg[ISTAT] &= ~(ISTAT_SIP | ISTAT_DIP | ISTAT_CON);
	if (dma)
		d->reg[ISTAT] |= ISTAT_DIP;
	if (scsi_fatal || scsi_enabled)
		d->reg[ISTAT] |= ISTAT_SIP;
	if (d->t.connected)
		d->reg[ISTAT] |= ISTAT_CON;

	assert = (dma & d->reg[DIEN]) || scsi_enabled ||
	    (d->reg[ISTAT] & ISTAT_INTF);
	if (d->reg[DCNTL] & DCNTL_IRQD)
		assert = 0;

	if (assert && !d->irq_asserted)
		INTERRUPT_ASSERT(d->irq);
	if (!assert && d->irq_asserted)
		INTERRUPT_DEASSERT(d->irq);
	d->irq_asserted = assert;
}

/*  An interrupt that stops SCRIPTS:  */
static void sym_halt(struct sym53c810_data *d, int dstat, int sist0,
	int sist1)
{
	d->reg[DSTAT] |= dstat;
	d->reg[SIST0] |= sist0;
	d->reg[SIST1] |= sist1;
	d->running = 0;
	d->wait_reselect = 0;

	sym_update_irq(d);

	debugmsg(d->subsys, "sym53c810", VERBOSITY_DEBUG,
	    "SCRIPTS halt at 0x%08x: DSTAT 0x%02x SIST0 0x%02x SIST1 0x%02x"
	    " DSPS 0x%08x (DIEN 0x%02x SIEN 0x%02x%02x DCNTL 0x%02x: IRQ %s)\n",
	    (int) reg32(d, DSP), d->reg[DSTAT], d->reg[SIST0], d->reg[SIST1],
	    (int) reg32(d, DSPS), d->reg[DIEN], d->reg[SIEN1], d->reg[SIEN0],
	    d->reg[DCNTL], d->irq_asserted? "asserted" : "not asserted");
}


/******************************************************************************
 *
 *  Bus master cycles: memory through the host bridge, or the chip's own
 *  registers.
 */

static void sym_reg_access(struct sym53c810_data *d, struct cpu *cpu,
	int r, uint8_t *val, int writeflag);

static int sym_dma(struct sym53c810_data *d, struct cpu *cpu,
	uint32_t busaddr, unsigned char *buf, size_t len, int writeflag)
{
	while (len > 0) {
		/*  At most up to the next 8 KB page (scatter/gather):  */
		size_t chunk = 0x2000 - (busaddr & 0x1fff);
		uint64_t paddr;

		if (chunk > len)
			chunk = len;

		if (d->mem_mapped && busaddr >= d->mem_busaddr &&
		    busaddr < d->mem_busaddr + SYM_BAR_SIZE) {
			sym_reg_access(d, cpu, busaddr & (SYM_NREGS - 1), buf,
			    writeflag);
			chunk = 1;
		} else {
			if (!bus_pci_dma_translate(cpu, d->pci, busaddr,
			    &paddr)) {
				debugmsg(d->subsys, "sym53c810",
				    VERBOSITY_ERROR, "bus fault at 0x%08x\n",
				    (int) busaddr);
				return 0;
			}

			cpu->memory_rw(cpu, cpu->mem, paddr, buf, chunk,
			    writeflag, PHYSICAL | NO_EXCEPTIONS);
		}

		busaddr += chunk;
		buf += chunk;
		len -= chunk;
	}

	return 1;
}

static int sym_dma_read32(struct sym53c810_data *d, struct cpu *cpu,
	uint32_t busaddr, uint32_t *valp)
{
	unsigned char b[4];

	if (!sym_dma(d, cpu, busaddr, b, 4, MEM_READ))
		return 0;

	*valp = b[0] | (b[1] << 8) | (b[2] << 16) | ((uint32_t) b[3] << 24);
	return 1;
}


/******************************************************************************
 *
 *  The SCSI bus, and the target that is connected.
 */

static void tgt_disconnect(struct sym53c810_data *d)
{
	struct sym_target *t = &d->t;

	if (t->xfer != NULL)
		scsi_transfer_free(t->xfer);

	memset(t, 0, sizeof(*t));
	d->reg[SCNTL1] &= ~SCNTL1_CON;
}

static void tgt_queue_msg_in(struct sym_target *t, const unsigned char *msg,
	int len)
{
	if (t->msg_in_ofs >= t->msg_in_len)
		t->msg_in_ofs = t->msg_in_len = 0;
	if (t->msg_in_len + len > (int) sizeof(t->msg_in))
		return;
	memcpy(t->msg_in + t->msg_in_len, msg, len);
	t->msg_in_len += len;
}

/*  What the target does next, once the chip has let go of ACK.  */
static void tgt_advance(struct sym53c810_data *d)
{
	struct sym_target *t = &d->t;

	if (!t->connected || t->ack_held)
		return;

	if (t->atn)
		t->phase = PHASE_MSG_OUT;
	else if (t->msg_in_ofs < t->msg_in_len)
		t->phase = PHASE_MSG_IN;
	else if (t->disconnect_pending)
		tgt_disconnect(d);
	else if (!t->cmd_done)
		t->phase = PHASE_COMMAND;
	else if (t->data_out_pending)
		t->phase = PHASE_DATA_OUT;
	else if (t->xfer != NULL && t->data_in_ofs < t->xfer->data_in_len)
		t->phase = PHASE_DATA_IN;
	else if (!t->status_done)
		t->phase = PHASE_STATUS;
	else {
		static const unsigned char complete = MSG_COMMAND_COMPLETE;
		tgt_queue_msg_in(t, &complete, 1);
		t->disconnect_pending = 1;
		t->phase = PHASE_MSG_IN;
	}
}

/*  Messages from the chip. An incomplete one is kept for later.  */
static void tgt_messages(struct sym53c810_data *d)
{
	struct sym_target *t = &d->t;
	int i = 0, n;

	while (i < t->msg_out_len) {
		unsigned char *m = t->msg_out + i;
		int left = t->msg_out_len - i;

		if (m[0] & MSG_IDENTIFY) {
			t->lun = m[0] & 7;
			n = 1;
		} else if (m[0] == MSG_EXTENDED) {
			if (left < 2 || left < m[1] + 2)
				break;
			n = m[1] + 2;
			if (m[1] == 3 && m[2] == MSG_EXT_SDTR) {
				/*  Agreed, as asked.  */
				tgt_queue_msg_in(t, m, 5);
			} else if (m[1] == 2 && m[2] == MSG_EXT_WDTR) {
				unsigned char wdtr[4] = { MSG_EXTENDED, 2,
				    MSG_EXT_WDTR, 0 };
				tgt_queue_msg_in(t, wdtr, 4);
			} else {
				static const unsigned char rej = MSG_REJECT;
				tgt_queue_msg_in(t, &rej, 1);
			}
		} else if (m[0] >= 0x20 && m[0] <= 0x2f) {
			/*  Two byte messages (queue tags, ...):  */
			if (left < 2)
				break;
			n = 2;
		} else {
			n = 1;
			switch (m[0]) {
			case MSG_ABORT:
			case MSG_DEVICE_RESET:
			case MSG_ABORT_TAG:
			case MSG_CLEAR_QUEUE:
				t->disconnect_pending = 1;
				t->msg_in_ofs = t->msg_in_len = 0;
				break;
			case MSG_REJECT:
				/*  Of what we were saying: stop saying it.  */
				t->msg_in_ofs = t->msg_in_len = 0;
				break;
			case MSG_NOP:
			case 0x05:	/*  initiator detected error  */
			case 0x09:	/*  message parity error  */
				break;
			default:{
				static const unsigned char rej = MSG_REJECT;
				tgt_queue_msg_in(t, &rej, 1);
				}
			}
		}

		debugmsg(d->subsys, "sym53c810", VERBOSITY_DEBUG,
		    "id %i: message out 0x%02x (%i bytes)\n", t->id, m[0], n);
		i += n;
	}

	memmove(t->msg_out, t->msg_out + i, t->msg_out_len - i);
	t->msg_out_len -= i;
}

static int scsi_cmd_len(uint8_t opcode)
{
	switch (opcode >> 5) {
	case 0:	return 6;
	case 1:
	case 2:	return 10;
	case 4:	return 16;
	case 5:	return 12;
	default:return 10;
	}
}

/*  The command has arrived:  */
static void tgt_execute(struct sym53c810_data *d, struct cpu *cpu)
{
	struct sym_target *t = &d->t;
	struct scsi_transfer *x;
	int res;

	if (t->lun == 0 && (t->cmd[1] >> 5) != 0 && t->msg_out_len == 0)
		t->lun = t->cmd[1] >> 5;

	t->cmd_done = 1;
	t->xfer = x = scsi_transfer_alloc();
	scsi_transfer_allocbuf(&x->cmd_len, &x->cmd, t->cmd_len, 0);
	memcpy(x->cmd, t->cmd, t->cmd_len);

	debugmsg(d->subsys, "sym53c810", VERBOSITY_DEBUG,
	    "id %i lun %i: command 0x%02x\n", t->id, t->lun, t->cmd[0]);

	/*  Only LUN 0 is there.  */
	if (t->lun != 0) {
		if (t->cmd[0] == 0x12) {	/*  INQUIRY  */
			size_t n = t->cmd[4] < 36? t->cmd[4] : 36;
			scsi_transfer_allocbuf(&x->data_in_len, &x->data_in,
			    n, 1);
			if (n > 0)
				x->data_in[0] = 0x7f;	/*  no such LUN  */
			t->status = 0;
		} else if (t->cmd[0] == 0x03) {	/*  REQUEST SENSE  */
			size_t n = t->cmd[4] < 18? t->cmd[4] : 18;
			unsigned char sense[18] = { 0x70, 0, 0x05, 0,0,0,0,
			    10, 0,0,0,0, 0x25, 0 };
			scsi_transfer_allocbuf(&x->data_in_len, &x->data_in,
			    n, 1);
			memcpy(x->data_in, sense, n);
			t->status = 0;
		} else
			t->status = 0x02;	/*  CHECK CONDITION  */
		return;
	}

	res = diskimage_scsicommand(cpu, t->id, DISKIMAGE_SCSI, x);
	if (res == 2) {
		if (x->data_out == NULL)
			scsi_transfer_allocbuf(&x->data_out_len,
			    &x->data_out, x->data_out_len, 0);
		t->data_out_pending = 1;
	}

	t->status = x->status_len > 0? x->status[0] : (res? 0 : 0x02);
}

/*  The next byte the target would send (for SFBR, when waiting).  */
static int tgt_peek(struct sym53c810_data *d)
{
	struct sym_target *t = &d->t;

	switch (t->phase) {
	case PHASE_DATA_IN:
		return t->xfer->data_in[t->data_in_ofs];
	case PHASE_STATUS:
		return t->status;
	case PHASE_MSG_IN:
		return t->msg_in[t->msg_in_ofs];
	}
	return d->reg[SFBR];
}

/*
 *  A block move in the phase the target is in: up to count bytes at bus
 *  address addr. Returns the number of bytes moved, or -1 on a bus fault.
 */
static int tgt_transfer(struct sym53c810_data *d, struct cpu *cpu,
	uint32_t addr, uint32_t count)
{
	struct sym_target *t = &d->t;
	struct scsi_transfer *x = t->xfer;
	uint32_t n = 0;
	int res;

	switch (t->phase) {

	case PHASE_MSG_OUT:
		n = count;
		if (n > sizeof(t->msg_out) - t->msg_out_len)
			n = sizeof(t->msg_out) - t->msg_out_len;
		if (!sym_dma(d, cpu, addr, t->msg_out + t->msg_out_len, n,
		    MEM_READ))
			return -1;
		t->msg_out_len += n;
		/*  The chip lets go of ATN before the last byte.  */
		if (n == count) {
			t->atn = 0;
			d->reg[SOCL] &= ~SBCL_ATN;
		}
		tgt_messages(d);
		if (n == 0)
			n = count;	/*  (a runaway message)  */
		break;

	case PHASE_COMMAND:
		while (n < count && !t->cmd_done) {
			if (!sym_dma(d, cpu, addr + n, t->cmd + t->cmd_len, 1,
			    MEM_READ))
				return -1;
			t->cmd_len ++;
			n ++;
			if (t->cmd_len >= scsi_cmd_len(t->cmd[0]))
				tgt_execute(d, cpu);
		}
		break;

	case PHASE_DATA_OUT:
		n = x->data_out_len - x->data_out_offset;
		if (n > count)
			n = count;
		if (!sym_dma(d, cpu, addr, x->data_out + x->data_out_offset,
		    n, MEM_READ))
			return -1;
		x->data_out_offset += n;
		if (x->data_out_offset >= x->data_out_len) {
			t->data_out_pending = 0;
			res = diskimage_scsicommand(cpu, t->id,
			    DISKIMAGE_SCSI, x);
			if (res == 2)
				t->data_out_pending = 1;
			t->status = x->status_len > 0? x->status[0] :
			    (res? 0 : 0x02);
		}
		break;

	case PHASE_DATA_IN:
		n = x->data_in_len - t->data_in_ofs;
		if (n > count)
			n = count;
		if (n > 0)
			d->reg[SFBR] = x->data_in[t->data_in_ofs];
		if (!sym_dma(d, cpu, addr, x->data_in + t->data_in_ofs, n,
		    MEM_WRITE))
			return -1;
		if (n >= 4 && debugmsg_is_active(d->subsys, VERBOSITY_DEBUG)) {
			uint64_t pa = 0;
			bus_pci_dma_translate(cpu, d->pci, addr, &pa);
			debugmsg(d->subsys, "sym53c810", VERBOSITY_DEBUG,
			    "data in: %i bytes to 0x%08x (0x%llx): %02x %02x "
			    "%02x %02x ...\n", (int) n, (int) addr,
			    (long long) pa, x->data_in[t->data_in_ofs],
			    x->data_in[t->data_in_ofs + 1],
			    x->data_in[t->data_in_ofs + 2],
			    x->data_in[t->data_in_ofs + 3]);
		}
		t->data_in_ofs += n;
		break;

	case PHASE_STATUS:
		{
			unsigned char s = t->status;
			d->reg[SFBR] = s;
			if (!sym_dma(d, cpu, addr, &s, 1, MEM_WRITE))
				return -1;
			t->status_done = 1;
			n = 1;
		}
		break;

	case PHASE_MSG_IN:
		n = t->msg_in_len - t->msg_in_ofs;
		if (n > count)
			n = count;
		d->reg[SFBR] = t->msg_in[t->msg_in_ofs];
		if (!sym_dma(d, cpu, addr, t->msg_in + t->msg_in_ofs, n,
		    MEM_WRITE))
			return -1;
		t->msg_in_ofs += n;
		/*  The chip holds ACK on the last byte of a message in move,
		    until SCRIPTS clear it.  */
		if (n == count)
			t->ack_held = 1;
		break;
	}

	tgt_advance(d);
	return n;
}

/*  SCSI bus reset:  */
static void sym_scsi_reset(struct sym53c810_data *d)
{
	tgt_disconnect(d);
	d->reg[SIST0] |= SIST0_RST;
	if (d->running)
		sym_halt(d, 0, 0, 0);
	sym_update_irq(d);
}


/******************************************************************************
 *
 *  SCRIPTS
 */

/*  Waits for REQ (the target to be in a phase). 0 if it disconnected.  */
static int sym_wait_req(struct sym53c810_data *d)
{
	if (d->t.connected)
		return 1;

	sym_halt(d, 0, SIST0_UDC, 0);
	return 0;
}

/*  Block move  */
static void sym_block_move(struct sym53c810_data *d, struct cpu *cpu,
	uint32_t insn, uint32_t addr)
{
	int phase = (insn >> 24) & 7;
	uint32_t count = insn & 0xffffff;

	if (insn & 0x10000000) {
		/*  Table indirect: count and address at DSA + offset.  */
		uint32_t table = reg32(d, DSA) + sext24(addr);
		if (!sym_dma_read32(d, cpu, table, &count) ||
		    !sym_dma_read32(d, cpu, table + 4, &addr)) {
			sym_halt(d, DSTAT_BF, 0, 0);
			return;
		}
		count &= 0xffffff;
	} else if (insn & 0x20000000) {
		/*  Indirect: the address is at addr.  */
		if (!sym_dma_read32(d, cpu, addr, &addr)) {
			sym_halt(d, DSTAT_BF, 0, 0);
			return;
		}
	}

	set_dbc(d, count);
	set_reg32(d, DNAD, addr);

	debugmsg(d->subsys, "sym53c810", VERBOSITY_DEBUG,
	    "MOVE %s, %i bytes at 0x%08x\n", phase_name[phase], (int) count,
	    (int) addr);

	if (d->reg[SCNTL0] & SCNTL0_TRG) {
		debugmsg(d->subsys, "sym53c810", VERBOSITY_ERROR,
		    "target mode is not emulated\n");
		sym_halt(d, DSTAT_IID, 0, 0);
		return;
	}

	if (!sym_wait_req(d))
		return;

	/*  Releasing a held ACK lets the target go on.  */
	if (d->t.ack_held) {
		d->t.ack_held = 0;
		tgt_advance(d);
		if (!sym_wait_req(d))
			return;
	}

	while (count > 0) {
		int n;

		if (!d->t.connected) {
			sym_halt(d, 0, SIST0_UDC, 0);
			return;
		}
		if (d->t.phase != phase) {
			debugmsg(d->subsys, "sym53c810", VERBOSITY_DEBUG,
			    "phase mismatch: MOVE %s, target in %s (%i bytes "
			    "left)\n", phase_name[phase],
			    phase_name[d->t.phase], (int) count);
			d->reg[SSTAT1] = (d->reg[SSTAT1] & ~7) | d->t.phase;
			sym_halt(d, 0, SIST0_MA, 0);
			return;
		}

		n = tgt_transfer(d, cpu, addr, count);
		if (n < 0) {
			sym_halt(d, DSTAT_BF, 0, 0);
			return;
		}

		addr += n;
		count -= n;
		set_dbc(d, count);
		set_reg32(d, DNAD, addr);

		if (n == 0 && d->t.connected && d->t.phase == phase) {
			debugmsg(d->subsys, "sym53c810", VERBOSITY_ERROR,
			    "MOVE %s: the target is stuck\n",
			    phase_name[phase]);
			sym_halt(d, DSTAT_IID, 0, 0);
			return;
		}
	}

	if (d->t.connected)
		d->reg[SSTAT1] = (d->reg[SSTAT1] & ~7) | d->t.phase;
}

/*  Arithmetic for register moves:  */
static uint8_t sym_alu(struct sym53c810_data *d, int op, uint8_t a,
	uint8_t b)
{
	int r;

	switch (op) {
	case 0:	return b;
	case 1:	r = (a << 1) | d->carry;
		d->carry = (a >> 7) & 1;
		return r;
	case 2:	return a | b;
	case 3:	return a ^ b;
	case 4:	return a & b;
	case 5:	r = (a >> 1) | (d->carry << 7);
		d->carry = a & 1;
		return r;
	case 6:	r = a + b;
		d->carry = r >> 8;
		return r;
	default:r = a + b + d->carry;
		d->carry = r >> 8;
		return r;
	}
}

static uint8_t sym_reg_read_internal(struct sym53c810_data *d,
	struct cpu *cpu, int r)
{
	uint8_t v;
	sym_reg_access(d, cpu, r & (SYM_NREGS - 1), &v, MEM_READ);
	return v;
}

static void sym_reg_write_internal(struct sym53c810_data *d,
	struct cpu *cpu, int r, uint8_t v)
{
	sym_reg_access(d, cpu, r & (SYM_NREGS - 1), &v, MEM_WRITE);
}

/*  I/O and register move instructions  */
static void sym_io(struct sym53c810_data *d, struct cpu *cpu, uint32_t insn,
	uint32_t addr2)
{
	int opcode = (insn >> 27) & 7;
	uint32_t alt = (insn & 0x04000000)? reg32(d, DSP) + sext24(addr2) :
	    addr2;
	struct sym_target *t = &d->t;

	switch (opcode) {

	case 0:	/*  SELECT  */
		{
			int id = (insn >> 16) & 0xf;
			int atn = insn & 0x01000000;

			if (insn & 0x02000000) {
				uint32_t w;
				if (!sym_dma_read32(d, cpu, reg32(d, DSA) +
				    sext24(insn), &w)) {
					sym_halt(d, DSTAT_BF, 0, 0);
					return;
				}
				id = (w >> 16) & 0xf;
				d->reg[SCNTL3] = w >> 24;
				d->reg[SXFER] = w >> 8;
			}

			d->reg[SDID] = id;

			if (t->connected) {
				/*  (Can't arbitrate.)  */
				debugmsg(d->subsys, "sym53c810",
				    VERBOSITY_WARNING, "SELECT while "
				    "connected\n");
				set_reg32(d, DSP, alt);
				return;
			}

			if (id == (d->reg[SCID] & 0xf) ||
			    !diskimage_exist(d->machine, id, DISKIMAGE_SCSI)) {
				debugmsg(d->subsys, "sym53c810",
				    VERBOSITY_DEBUG, "select id %i: "
				    "timeout\n", id);
				sym_halt(d, 0, 0, SIST1_STO);
				return;
			}

			debugmsg(d->subsys, "sym53c810", VERBOSITY_DEBUG,
			    "select id %i%s\n", id, atn? " with ATN" : "");

			memset(t, 0, sizeof(*t));
			t->connected = 1;
			t->id = id;
			t->atn = atn != 0;
			if (atn)
				d->reg[SOCL] |= SBCL_ATN;
			d->reg[SCNTL1] |= SCNTL1_CON;
			tgt_advance(d);
			d->reg[SSTAT1] = (d->reg[SSTAT1] & ~7) | t->phase;
		}
		break;

	case 1:	/*  WAIT DISCONNECT  */
		if (t->connected && t->ack_held && t->disconnect_pending &&
		    t->msg_in_ofs >= t->msg_in_len) {
			t->ack_held = 0;
			tgt_advance(d);
		}
		if (t->connected) {
			debugmsg(d->subsys, "sym53c810", VERBOSITY_WARNING,
			    "WAIT DISCONNECT: the target stays (in %s)\n",
			    phase_name[t->phase]);
			/*  It will, when the chip is reset.  */
			d->running = 0;
		}
		break;

	case 2:	/*  WAIT RESELECT: nothing reselects; until SIGP.  */
		if (d->reg[ISTAT] & ISTAT_SIGP)
			set_reg32(d, DSP, alt);
		else {
			d->wait_reselect = 1;
			d->running = 0;
			set_reg32(d, DSPS, alt);
		}
		break;

	case 3:	/*  SET  */
	case 4:	/*  CLEAR  */
		{
			int set = opcode == 3;

			if (insn & 0x400)
				d->carry = set;
			if (insn & 0x200) {
				if (set)
					d->reg[SCNTL0] |= SCNTL0_TRG;
				else
					d->reg[SCNTL0] &= ~SCNTL0_TRG;
			}
			if (insn & 0x8) {
				if (set)
					d->reg[SOCL] |= SBCL_ATN;
				else
					d->reg[SOCL] &= ~SBCL_ATN;
				t->atn = set;
				if (set && t->connected && !t->ack_held &&
				    t->phase != PHASE_MSG_OUT)
					tgt_advance(d);
			}
			if ((insn & 0x40) && !set && t->ack_held) {
				t->ack_held = 0;
				tgt_advance(d);
			}
			if (t->connected)
				d->reg[SSTAT1] = (d->reg[SSTAT1] & ~7) |
				    t->phase;
		}
		break;

	default:/*  Register moves: 5 = reg = SFBR op data,
		    6 = SFBR = reg op data, 7 = reg = reg op data.  */
		{
			int op = (insn >> 24) & 7;
			int r = (insn >> 16) & 0x7f;
			uint8_t data = insn >> 8;
			uint8_t a, v;

			if (insn & 0x00800000)
				data = d->reg[SFBR];

			if (opcode == 5)
				a = d->reg[SFBR];
			else
				a = sym_reg_read_internal(d, cpu, r);

			v = sym_alu(d, op, a, data);

			if (opcode == 6)
				d->reg[SFBR] = v;
			else
				sym_reg_write_internal(d, cpu, r, v);
		}
	}
}

/*  Transfer control instructions  */
static void sym_transfer_control(struct sym53c810_data *d, struct cpu *cpu,
	uint32_t insn, uint32_t addr2)
{
	int opcode = (insn >> 27) & 7;
	int phase = (insn >> 24) & 7;
	int cond = 1;
	uint32_t target = (insn & 0x00800000)? reg32(d, DSP) + sext24(addr2) :
	    addr2;

	if (insn & 0x00200000) {
		cond = d->carry;
	} else if (insn & 0x00060000) {
		if (insn & 0x00010000) {
			/*  WHEN: wait for the target to ask (REQ).  */
			if (!sym_wait_req(d))
				return;
			if (d->t.ack_held) {
				d->t.ack_held = 0;
				tgt_advance(d);
				if (!sym_wait_req(d))
					return;
			}
			d->reg[SSTAT1] = (d->reg[SSTAT1] & ~7) | d->t.phase;
			if ((insn & 0x00040000) && (d->t.phase & 1))
				d->reg[SFBR] = tgt_peek(d);
		}

		if ((insn & 0x00020000) && (!d->t.connected ||
		    d->t.phase != phase))
			cond = 0;

		if (insn & 0x00040000) {
			uint8_t mask = insn >> 8, data = insn;
			if ((d->reg[SFBR] ^ data) & ~mask)
				cond = 0;
		}
	}

	if (!(insn & 0x00080000))
		cond = !cond;

	if (!cond)
		return;

	switch (opcode) {
	case 0:	/*  JUMP  */
		set_reg32(d, DSP, target);
		break;
	case 1:	/*  CALL  */
		set_reg32(d, TEMP, reg32(d, DSP));
		set_reg32(d, DSP, target);
		break;
	case 2:	/*  RETURN  */
		set_reg32(d, DSP, reg32(d, TEMP));
		break;
	case 3:	/*  INT, or INTFLY  */
		if (insn & 0x00100000) {
			d->reg[ISTAT] |= ISTAT_INTF;
			sym_update_irq(d);
		} else
			sym_halt(d, DSTAT_SIR, 0, 0);
		break;
	default:
		sym_halt(d, DSTAT_IID, 0, 0);
	}
}

/*  One instruction.  */
static void sym_execute(struct sym53c810_data *d, struct cpu *cpu)
{
	uint32_t dsp = reg32(d, DSP), insn, addr2, addr3;

	d->insn_addr = dsp;

	if (!sym_dma_read32(d, cpu, dsp, &insn) ||
	    !sym_dma_read32(d, cpu, dsp + 4, &addr2)) {
		sym_halt(d, DSTAT_BF, 0, 0);
		return;
	}

	set_reg32(d, DSP, dsp + 8);
	d->reg[DCMD] = insn >> 24;
	set_dbc(d, insn & 0xffffff);
	set_reg32(d, DSPS, addr2);

	debugmsg(d->subsys, "sym53c810", VERBOSITY_DEBUG,
	    "SCRIPTS 0x%08x: %08x %08x\n", (int) dsp, (int) insn, (int) addr2);

	switch (insn >> 30) {

	case 0:	sym_block_move(d, cpu, insn, addr2);
		break;

	case 1:	sym_io(d, cpu, insn, addr2);
		break;

	case 2:	sym_transfer_control(d, cpu, insn, addr2);
		break;

	case 3:	if (!sym_dma_read32(d, cpu, dsp + 8, &addr3)) {
			sym_halt(d, DSTAT_BF, 0, 0);
			return;
		}

		if ((insn & 0xe0000000) == 0xc0000000) {
			/*  Memory move, the destination is in TEMP.  */
			uint32_t n = insn & 0xffffff, i;

			set_reg32(d, DSP, dsp + 12);
			set_reg32(d, TEMP, addr3);
			for (i = 0; i < n; i++) {
				unsigned char b;
				if (!sym_dma(d, cpu, addr2 + i, &b, 1,
				    MEM_READ) || !sym_dma(d, cpu, addr3 + i,
				    &b, 1, MEM_WRITE)) {
					sym_halt(d, DSTAT_BF, 0, 0);
					return;
				}
			}
		} else {
			/*  LOAD and STORE (the 53C810A's), 1..4 bytes.  */
			int r = (insn >> 16) & 0x7f, n = insn & 7, i;
			uint32_t a = addr2;

			if (insn & 0x10000000)
				a = reg32(d, DSA) + sext24(addr2);

			set_reg32(d, DSP, dsp + 8);
			for (i = 0; i < n; i++) {
				unsigned char b = 0;
				if (insn & 0x01000000) {
					if (!sym_dma(d, cpu, a + i, &b, 1,
					    MEM_READ)) {
						sym_halt(d, DSTAT_BF, 0, 0);
						return;
					}
					sym_reg_write_internal(d, cpu, r + i, b);
				} else {
					b = sym_reg_read_internal(d, cpu,
					    r + i);
					if (!sym_dma(d, cpu, a + i, &b, 1,
					    MEM_WRITE)) {
						sym_halt(d, DSTAT_BF, 0, 0);
						return;
					}
				}
			}
		}
		break;
	}

	if (d->running && (d->reg[DCNTL] & DCNTL_SSM))
		sym_halt(d, DSTAT_SSI, 0, 0);
}

static void sym_run(struct sym53c810_data *d, struct cpu *cpu)
{
	int n = 0;

	if (d->wait_reselect && (d->reg[ISTAT] & ISTAT_SIGP)) {
		d->wait_reselect = 0;
		d->running = 1;
		set_reg32(d, DSP, reg32(d, DSPS));
	}

	d->in_scripts = 1;
	while (d->running && n++ < SYM_INSTRS_PER_RUN)
		sym_execute(d, cpu);
	d->in_scripts = 0;

	sym_update_irq(d);
}

static void sym_start(struct sym53c810_data *d, struct cpu *cpu)
{
	debugmsg(d->subsys, "sym53c810", VERBOSITY_DEBUG,
	    "SCRIPTS start at 0x%08x\n", (int) reg32(d, DSP));
	d->running = 1;
	d->wait_reselect = 0;
	sym_run(d, cpu);
}


/******************************************************************************
 *
 *  Registers
 */

static void sym_reset(struct sym53c810_data *d)
{
	tgt_disconnect(d);

	memset(d->reg, 0, sizeof(d->reg));
	d->reg[SCNTL0] = 0xc0;
	d->reg[DSTAT] = DSTAT_DFE;
	d->reg[CTEST1] = 0xf0;		/*  DMA FIFO empty  */
	d->reg[CTEST3] = CTEST3_REVISION;
	d->reg[GPCNTL] = 0x0f;
	d->reg[STEST0] = 0x03;		/*  SOM, SOZ  */
	d->carry = 0;
	d->running = 0;
	d->wait_reselect = 0;

	sym_update_irq(d);
}

static void sym_reg_access(struct sym53c810_data *d, struct cpu *cpu,
	int r, uint8_t *val, int writeflag)
{
	uint8_t v = *val, old = d->reg[r];

	if (writeflag == MEM_READ) {
		v = d->reg[r];

		switch (r) {
		case DSTAT:
			v |= DSTAT_DFE;
			d->reg[DSTAT] = DSTAT_DFE;
			sym_update_irq(d);
			break;
		case SIST0:
		case SIST1:
			d->reg[r] = 0;
			sym_update_irq(d);
			break;
		case CTEST2:
			v = (v & ~CTEST2_SIGP) | ((d->reg[ISTAT] &
			    ISTAT_SIGP)? CTEST2_SIGP : 0);
			d->reg[ISTAT] &= ~ISTAT_SIGP;
			break;
		case SBCL:
			v = 0;
			if (d->t.connected) {
				v = SBCL_BSY | d->t.phase;
				if (!d->t.ack_held)
					v |= SBCL_REQ;
				else
					v |= SBCL_ACK;
			}
			if (d->reg[SOCL] & SBCL_ATN)
				v |= SBCL_ATN;
			break;
		case DFIFO:
			/*  The byte count, as DBC: nothing is in the FIFO.  */
			v = d->reg[DBC] & 0x7f;
			break;
		case CTEST3:
			v = (v & 0x0f) | CTEST3_REVISION;
			break;
		case SSTAT0:
		case SSTAT2:
		case SIDL:
		case SIDL + 1:
		case SBDL:
		case SBDL + 1:
			v = 0;
			break;
		}

		*val = v;
		return;
	}

	d->reg[r] = v;

	switch (r) {

	case SCNTL1:
		if ((v & SCNTL1_RST) && !(old & SCNTL1_RST)) {
			debugmsg(d->subsys, "sym53c810", VERBOSITY_DEBUG,
			    "SCSI bus reset\n");
			sym_scsi_reset(d);
		}
		if (!(v & SCNTL1_CON) && d->t.connected)
			tgt_disconnect(d);
		if (d->t.connected)
			d->reg[SCNTL1] |= SCNTL1_CON;
		break;

	case SOCL:
		d->t.atn = (v & SBCL_ATN) != 0;
		break;

	case DSTAT:
	case SIST0:
	case SIST1:
		d->reg[r] = old;	/*  read only  */
		break;

	case ISTAT:
		d->reg[ISTAT] = (v & (ISTAT_ABRT | ISTAT_SRST | ISTAT_SIGP |
		    ISTAT_SEM)) | (old & ISTAT_INTF & ~v) |
		    (old & (ISTAT_SIP | ISTAT_DIP | ISTAT_CON));
		if (v & ISTAT_SRST) {
			debugmsg(d->subsys, "sym53c810", VERBOSITY_DEBUG,
			    "chip reset\n");
			sym_reset(d);
			d->reg[ISTAT] |= ISTAT_SRST;
			break;
		}
		if ((v & ISTAT_ABRT) && !(old & ISTAT_ABRT)) {
			debugmsg(d->subsys, "sym53c810", VERBOSITY_DEBUG,
			    "abort\n");
			sym_halt(d, DSTAT_ABRT, 0, 0);
		}
		if ((v & ISTAT_SIGP) && d->wait_reselect && !d->in_scripts)
			sym_run(d, cpu);
		sym_update_irq(d);
		break;

	case CTEST3:
		d->reg[r] &= ~0x0c;	/*  FLF, CLF  */
		break;

	case STEST3:
		d->reg[r] &= ~0x02;	/*  CSF  */
		break;

	case DSP + 3:
		if (!d->in_scripts && !(d->reg[DMODE] & DMODE_MAN))
			sym_start(d, cpu);
		break;

	case DCNTL:
		d->reg[r] &= ~DCNTL_STD;
		if ((v & DCNTL_STD) && !d->in_scripts)
			sym_start(d, cpu);
		sym_update_irq(d);
		break;

	case DIEN:
	case SIEN0:
	case SIEN1:
		sym_update_irq(d);
		break;
	}
}


DEVICE_ACCESS(sym53c810)
{
	struct sym53c810_data *d = (struct sym53c810_data *) extra;
	size_t i;

	for (i = 0; i < len; i++) {
		int r = (relative_addr + i) & (SYM_NREGS - 1);
		sym_reg_access(d, cpu, r, data + i, writeflag);
	}

	return 1;
}


DEVICE_TICK(sym53c810)
{
	struct sym53c810_data *d = (struct sym53c810_data *) extra;

	if (d->running || (d->wait_reselect &&
	    (d->reg[ISTAT] & ISTAT_SIGP)))
		sym_run(d, cpu);
}


/*  Is [addr, addr + SYM_BAR_SIZE) free?  */
static int sym_space_free(struct memory *mem, uint64_t addr)
{
	int i;

	for (i = 0; i < mem->n_mmapped_devices; i++)
		if (addr < mem->devices[i].endaddr &&
		    addr + SYM_BAR_SIZE > mem->devices[i].baseaddr)
			return 0;

	return 1;
}

static void sym_unmap(struct sym53c810_data *d, uint64_t addr)
{
	int i;

	for (i = 0; i < d->mem->n_mmapped_devices; i++)
		if (d->mem->devices[i].extra == d &&
		    d->mem->devices[i].baseaddr == addr) {
			memory_device_remove(d->mem, i);
			return;
		}
}

static void sym_map(struct sym53c810_data *d, uint64_t *cur, int *mapped,
	uint64_t addr, int enable, const char *what, const char *name)
{
	if (*mapped && (!enable || addr != *cur)) {
		sym_unmap(d, *cur);
		*mapped = 0;
	}

	if (!enable || *mapped)
		return;

	if (!sym_space_free(d->mem, addr)) {
		debugmsg(d->subsys, "sym53c810", VERBOSITY_WARNING,
		    "%s space at 0x%llx is taken; not mapped\n", what,
		    (long long) addr);
		return;
	}

	memory_device_register(d->mem, name, addr, SYM_BAR_SIZE,
	    dev_sym53c810_access, d, DM_DEFAULT, NULL);
	*cur = addr;
	*mapped = 1;
}


/*
 *  dev_sym53c810_set_bars():
 *
 *  Where the registers are (the emulated addresses of the I/O and memory
 *  BARs, and whether they are enabled); mem_busaddr is the memory BAR's PCI
 *  address.
 */
void dev_sym53c810_set_bars(struct sym53c810_data *d, uint64_t io_addr,
	int io_enable, uint64_t mem_addr, int mem_enable, uint32_t mem_busaddr)
{
	sym_map(d, &d->io_addr, &d->io_mapped, io_addr, io_enable, "I/O",
	    "sym53c810_io");
	sym_map(d, &d->mem_addr, &d->mem_mapped, mem_addr, mem_enable,
	    "memory", "sym53c810_mem");
	d->mem_busaddr = mem_busaddr;
}


/*
 *  dev_sym53c810_init():
 *
 *  The chip on PCI bus pci, interrupting on irq_path. The registers appear
 *  once the BARs are set (dev_sym53c810_set_bars()).
 */
struct sym53c810_data *dev_sym53c810_init(struct machine *machine,
	struct memory *mem, struct pci_data *pci, const char *irq_path)
{
	struct sym53c810_data *d;

	CHECK_ALLOCATION(d = (struct sym53c810_data *) malloc(sizeof(*d)));
	memset(d, 0, sizeof(*d));

	d->machine = machine;
	d->mem = mem;
	d->pci = pci;
	d->subsys = debugmsg_register_subsystem("sym53c810");

	INTERRUPT_CONNECT(irq_path, d->irq);

	sym_reset(d);

	machine_add_tickfunction(machine, dev_sym53c810_tick, d,
	    SYM_TICK_SHIFT);

	return d;
}
