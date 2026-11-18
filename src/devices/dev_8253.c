/*
 *  Copyright (C) 2005-2020  Anders Gavare.  All rights reserved.
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
 *  COMMENT: Intel 8253/8254 Programmable Interval Timer
 *
 *  Timer 0 causes clock interrupts at its frequency.  On machines that ask
 *  for it ("name2=counting"), counters 0 and 2 count down at 1.193182 MHz in
 *  host time, as the CPU's cycle counter does on machines that know their
 *  clock rate, so firmware that times the CPU against them finds that rate;
 *  otherwise, and for counter 1 (DMA refresh), a counter reads back the
 *  value loaded (see pit_count()).  Each counter has its own access mode
 *  (LSB, MSB or both) and byte order, and the counter latch and 8254
 *  read-back commands latch counts.  Counter 2 makes no tone; its output
 *  and gate (port 61h, dev_pckbc.c) follow its count when counting.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "cpu.h"
#include "device.h"
#include "devices.h"
#include "emul.h"
#include "interrupt.h"
#include "machine.h"
#include "memory.h"
#include "misc.h"
#include "timer.h"

#include "thirdparty/i8253reg.h"


/*  #define debug fatal  */

#define	DEV_8253_LENGTH		4
#define	TICK_SHIFT		14


struct pit8253_data {
	int		in_use;
	int		counting;	/*  counters 0 and 2 count down  */

	int		mode[3];
	int		counter[3];		/*  reload value  */
	double		hz[3];

	/*  Per counter: access mode (I8253_TIMER_LSB, _MSB or _16BIT), which
	    byte comes next when written or read as 16 bits, and a latched
	    count:  */
	int		rw_mode[3];
	int		write_msb_next[3];
	int		read_msb_next[3];
	int		latched[3];
	int		latch_value[3];
	int		gate2;		/*  counter 2's gate (port 61h bit 0)  */

	/*  Counting (see pit_count()): when the counter was loaded, and how
	    many times it had reached zero when it was last read.  */
	struct timeval	load_time[3];
	uint64_t	zeros_shown[3];

	struct timer	*timer0;
	struct interrupt irq;
	int		pending_interrupts_timer0;
};


static void timer0_tick(struct timer *t, void *extra)
{
	struct pit8253_data *d = (struct pit8253_data *) extra;
	d->pending_interrupts_timer0 ++;

	/*  printf("%i ", d->pending_interrupts_timer0); fflush(stdout);  */
}


DEVICE_TICK(8253)
{
	struct pit8253_data *d = (struct pit8253_data *) extra;

	if (!d->in_use)
		return;

	// Generate interrupts regardless of (d->mode[0] & 0x0e)?
	// (It seems like Linux/MALTA kernels like this.)
	if (d->pending_interrupts_timer0 > 0)
		INTERRUPT_ASSERT(d->irq);
}


/*
 *  The current count of a counter: counters 0 and 2 count down from the
 *  value loaded at 1.193182 MHz in host time. In mode 3 (square wave) the
 *  count goes down by two, from the even value at or below the one loaded,
 *  and shows zero for one clock as it reloads; firmware that waits for zero
 *  by polling (as the Avanti DROM does to time the CPU) would miss that one
 *  clock between the reads of an emulated loop, so a zero not yet shown is
 *  returned by the next read. In mode 2, the count goes from the value
 *  loaded down to 1; in the one-shot modes, it runs down through zero.
 *
 *  Counter 1 (DMA refresh), and all counters on machines that don't ask for
 *  counting, read back the value loaded: firmware tests such as the Jensen
 *  ARC console's "Timer 1" check expect the count read back soon after
 *  loading to be close to the loaded value, which host-time counting at the
 *  emulator's speed does not give, and with counters 0 and 2 counting, the
 *  Jensen's ARC console finds its EISA configuration inconsistent.
 */
static int pit_count(struct pit8253_data *d, int c)
{
	uint64_t n = d->counter[c] & 0xffff, t, zeros;
	struct timeval now;

	if (c == 1 || !d->counting)
		return n;

	if (n == 0)
		n = 65536;

	gettimeofday(&now, NULL);
	t = (uint64_t) ((now.tv_sec - d->load_time[c].tv_sec) * 1e6 +
	    (now.tv_usec - d->load_time[c].tv_usec)) * I8253_TIMER_FREQ /
	    1000000;

	switch (d->mode[c] & 6) {
	case I8253_TIMER_SQWAVE & 6:
		n &= ~1;
		if (n == 0)
			n = 65536;
		zeros = t * 2 / n;
		if (zeros > d->zeros_shown[c]) {
			d->zeros_shown[c] = zeros;
			return 0;
		}
		return (n - (t * 2) % n) & 0xffff;
	case I8253_TIMER_RATEGEN & 6:
		return n - t % n;
	default:
		return (n - t) & 0xffff;
	}
}


/*
 *  Counter 2's output, as port 61h bit 5 shows it, or -1 if the counters
 *  don't count (port 61h then makes it up). In mode 3 (square wave) it is
 *  high for the first half of each period (the Alpha ARC firmware's
 *  KeStallExecution calibration counts its edges against the cycle
 *  counter); in mode 2 it is low for one clock per period; in mode 0 it
 *  goes high at terminal count. With the gate low, it stays high.
 */
int dev_8253_counter2_out(struct pit8253_data *d)
{
	uint64_t n, t;
	struct timeval now;

	if (d == NULL || !d->counting)
		return -1;
	n = d->counter[2] & 0xffff;
	if (!d->gate2)
		return 1;
	if (n == 0)
		n = 65536;

	gettimeofday(&now, NULL);
	t = (uint64_t) ((now.tv_sec - d->load_time[2].tv_sec) * 1e6 +
	    (now.tv_usec - d->load_time[2].tv_usec)) * I8253_TIMER_FREQ /
	    1000000;

	switch (d->mode[2] & 6) {
	case I8253_TIMER_SQWAVE & 6:
		n &= ~1;
		if (n == 0)
			n = 65536;
		return (t % n) < n / 2;
	case I8253_TIMER_RATEGEN & 6:
		return (t % n) != n - 1;
	case I8253_TIMER_INTTC & 6:
		return t >= n;
	default:
		return 1;
	}
}


/*  Counter 2's gate: a rising edge starts the count again from the value
    loaded.  */
void dev_8253_counter2_gate(struct pit8253_data *d, int gate)
{
	if (d == NULL)
		return;
	if (gate && !d->gate2) {
		gettimeofday(&d->load_time[2], NULL);
		d->zeros_shown[2] = 0;
	}
	d->gate2 = gate;
}


/*  A new reload value has been written to counter c:  */
static void pit_load(struct pit8253_data *d, int c)
{
	gettimeofday(&d->load_time[c], NULL);
	d->zeros_shown[c] = 0;

	/*  A reload value of 0 means 65536 (18.2 Hz for counter 0, which the
	    Alpha ARC firmware times the CPU with).  */
	d->hz[c] = I8253_TIMER_FREQ / (double) (d->counter[c] == 0?
	    65536 : d->counter[c]);
	debugmsg(SUBSYS_DEVICE, "dev_8253", VERBOSITY_DEBUG, "[ 8253: counter %i set to %i (%.3f Hz) ]\n", c, d->counter[c],
	    d->hz[c]);

	switch (c) {
	case 0:	if (d->timer0 == NULL)
			d->timer0 = timer_add(d->hz[0], timer0_tick, d);
		else
			timer_update_frequency(d->timer0, d->hz[0]);
		break;
	case 1:	debugmsg(SUBSYS_DEVICE, "dev_8253", VERBOSITY_DEBUG, "[ 8253: counter 1 (DMA refresh) not emulated ]\n");
		break;
	case 2:	debugmsg(SUBSYS_DEVICE, "dev_8253", VERBOSITY_DEBUG, "[ 8253: counter 2 (tone generation) not emulated ]\n");
		break;
	}
}


static void pit_latch(struct pit8253_data *d, int c)
{
	if (d->latched[c])
		return;
	d->latched[c] = 1;
	d->latch_value[c] = pit_count(d, c);
	d->read_msb_next[c] = 0;
}


DEVICE_ACCESS(8253)
{
	struct pit8253_data *d = (struct pit8253_data *) extra;
	uint64_t idata = 0, odata = 0;
	int c = relative_addr, value;

	if (writeflag == MEM_WRITE)
		idata = memory_readmax64(cpu, data, len);

	d->in_use = 1;

	switch (relative_addr) {

	case I8253_TIMER_CNTR0:
	case I8253_TIMER_CNTR1:
	case I8253_TIMER_CNTR2:
		if (writeflag == MEM_WRITE) {
			switch (d->rw_mode[c]) {
			case I8253_TIMER_LSB:
				d->counter[c] = idata & 0xff;
				pit_load(d, c);
				break;
			case I8253_TIMER_MSB:
				d->counter[c] = (idata & 0xff) << 8;
				pit_load(d, c);
				break;
			default:
				if (!d->write_msb_next[c]) {
					d->counter[c] = (d->counter[c] & 0xff00)
					    | (idata & 0xff);
					d->write_msb_next[c] = 1;
				} else {
					d->counter[c] = (d->counter[c] & 0x00ff)
					    | ((idata & 0xff) << 8);
					d->write_msb_next[c] = 0;
					pit_load(d, c);
				}
			}
		} else {
			value = d->latched[c]? d->latch_value[c] :
			    pit_count(d, c);
			switch (d->rw_mode[c]) {
			case I8253_TIMER_LSB:
				odata = value & 0xff;
				d->latched[c] = 0;
				break;
			case I8253_TIMER_MSB:
				odata = (value >> 8) & 0xff;
				d->latched[c] = 0;
				break;
			default:
				if (!d->read_msb_next[c]) {
					odata = value & 0xff;
					d->read_msb_next[c] = 1;
				} else {
					odata = (value >> 8) & 0xff;
					d->read_msb_next[c] = 0;
					d->latched[c] = 0;
				}
			}
		}
		break;

	case I8253_TIMER_MODE:
		if (writeflag == MEM_READ) {
			debugmsg(SUBSYS_DEVICE, "dev_8253", VERBOSITY_DEBUG, "[ 8253: read; can this actually happen? ]\n");
			break;
		}

		c = idata >> 6;
		if (c == 3) {
			/*  8254 read-back command: latch the counts of the
			    selected counters (status latching is not
			    emulated).  */
			if (!(idata & 0x20))
				for (c = 0; c < 3; c++)
					if (idata & (2 << c))
						pit_latch(d, c);
			break;
		}

		if ((idata & 0x30) == 0) {
			/*  Counter latch command:  */
			pit_latch(d, c);
			break;
		}

		d->rw_mode[c] = idata & 0x30;
		d->mode[c] = idata & 0x0e;
		d->write_msb_next[c] = d->read_msb_next[c] = 0;
		d->latched[c] = 0;
		debugmsg(SUBSYS_DEVICE, "dev_8253", VERBOSITY_DEBUG, "[ 8253: counter %i mode=0x%x access=0x%x ]\n", c,
		    d->mode[c], d->rw_mode[c]);

		if (idata & I8253_TIMER_BCD)
			debugmsg(SUBSYS_DEVICE, "dev_8253", VERBOSITY_DEBUG, "[ 8253: BCD counting not emulated ]\n");
		break;

	default:if (writeflag == MEM_WRITE) {
			debugmsg(SUBSYS_DEVICE, "8253", VERBOSITY_DEBUG, "[ 8253: unimplemented write to address 0x%x"
			    " data=0x%02x ]", (int)relative_addr, (int)idata);
		} else {
			debugmsg(SUBSYS_DEVICE, "8253", VERBOSITY_DEBUG, "[ 8253: unimplemented read from address 0x%x "
			    "]", (int)relative_addr);
		}
	}

	if (writeflag == MEM_READ)
		memory_writemax64(cpu, data, len, odata);

	return 1;
}


DEVINIT(8253)
{
	struct pit8253_data *d;

	CHECK_ALLOCATION(d = (struct pit8253_data *) malloc(sizeof(struct pit8253_data)));
	memset(d, 0, sizeof(struct pit8253_data));

	d->in_use = devinit->in_use;
	d->counting = devinit->name2 != NULL &&
	    strcmp(devinit->name2, "counting") == 0;

	INTERRUPT_CONNECT(devinit->interrupt_path, d->irq);

	/*  Don't cause interrupt, by default.  */
	d->mode[0] = I8253_TIMER_RATEGEN;
	d->mode[1] = I8253_TIMER_RATEGEN;
	d->mode[2] = I8253_TIMER_RATEGEN;
	d->rw_mode[0] = d->rw_mode[1] = d->rw_mode[2] = I8253_TIMER_16BIT;

	devinit->machine->isa_pic_data.pending_timer_interrupts =
	    &d->pending_interrupts_timer0;
	devinit->machine->isa_pic_data.pit = d;

	memory_device_register(devinit->machine->memory, devinit->name,
	    devinit->addr, DEV_8253_LENGTH, dev_8253_access, (void *)d,
	    DM_DEFAULT, NULL);

	machine_add_tickfunction(devinit->machine, dev_8253_tick,
	    d, TICK_SHIFT);

	return 1;
}

