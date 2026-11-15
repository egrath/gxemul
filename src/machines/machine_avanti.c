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
 *  COMMENT: AlphaStation 200/400 ("Avanti")
 *
 *  Digital's first PCI Alpha workstations: an EV4 (21064, in the 4/166) or
 *  EV45 (21064A: the 200 and 400 4/233, the 250 4/266) on the APECS
 *  chipset, with an 82378ZB SIO for ISA. The machine runs the real
 *  firmware: the board's DROM and the SRM (or ARC) console in its flash,
 *  given as one file, the chip dumps one after the other:
 *
 *	E13 E19		SRM console (flash 0..7.FFFF)
 *	E33 E46		ARC console (flash 8.0000..F.FFFF)
 *	E47		DROM (64 KB)
 *
 *  See src/devices/dev_avanti.c and doc/avanti-plan.md.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cpu.h"
#include "device.h"
#include "devices.h"
#include "machine.h"
#include "memory.h"
#include "misc.h"


#define	AVANTI_FLASH_SIZE	(1024 * 1024)
#define	AVANTI_DROM_SIZE	(64 * 1024)


MACHINE_SETUP(avanti)
{
	char tmp[200];

	switch (machine->machine_subtype) {
	case MACHINE_AVANTI_200_233:
		machine->machine_name = strdup("AlphaStation 200 4/233");
		if (machine->emulated_hz == 0)
			machine->emulated_hz = 233333333;
		break;
	case MACHINE_AVANTI_400_233:
		machine->machine_name = strdup("AlphaStation 400 4/233");
		if (machine->emulated_hz == 0)
			machine->emulated_hz = 233333333;
		break;
	case MACHINE_AVANTI_250_266:
		machine->machine_name = strdup("AlphaStation 250 4/266");
		if (machine->emulated_hz == 0)
			machine->emulated_hz = 266666666;
		break;
	case MACHINE_AVANTI_200_166:
	default:
		machine->machine_subtype = MACHINE_AVANTI_200_166;
		machine->machine_name = strdup("AlphaStation 200 4/166");
		if (machine->emulated_hz == 0)
			machine->emulated_hz = 166666666;
		break;
	}

	if (machine->physical_ram_in_mb > 384) {
		fatal("The AlphaStation 200 and 400 can have at most 384 MB "
		    "of memory.\n");
		exit(1);
	}

	/*  The firmware's own PALcode runs on the CPU:  */
	cpu->cd.alpha.ev4_palcode = 1;

	snprintf(tmp, sizeof(tmp), "%s.cpu[%i]", machine->path,
	    machine->bootstrap_cpu);
	machine->md.avanti = dev_avanti_init(machine, machine->memory,
	    (uint64_t) machine->physical_ram_in_mb << 20, tmp);
}


/*  Is there a ROM image header with this name at p?  */
static int avanti_rom_header(const unsigned char *p, const char *name)
{
	return p[4] == 0x55 && p[5] == 0x00 && p[6] == 0xaa && p[7] == 0xff &&
	    memcmp(p + 0x30, name, strlen(name)) == 0;
}


/*
 *  avanti_load_rom():
 *
 *  Loads the flash and DROM image (see above). Returns 1 on success, 0 if
 *  the file isn't one.
 */
int avanti_load_rom(struct machine *machine, const char *filename,
	uint64_t *entrypointp)
{
	struct avanti_data *d = machine->md.avanti;
	unsigned char *buf;
	size_t size = AVANTI_FLASH_SIZE + AVANTI_DROM_SIZE;
	FILE *f;

	if (d == NULL)
		return 0;

	f = fopen(filename, "rb");
	if (f == NULL)
		return 0;

	CHECK_ALLOCATION(buf = (unsigned char *) malloc(size + 1));
	if (fread(buf, 1, size + 1, f) != size) {
		fclose(f);
		free(buf);
		fatal("%s: not an AlphaStation 200 ROM image (the flash, "
		    "1 MB, and the DROM, 64 KB)\n", filename);
		return 0;
	}
	fclose(f);

	if (!avanti_rom_header(buf + AVANTI_FLASH_SIZE, "AVANTI-DROM") ||
	    (!avanti_rom_header(buf, "AVANTI_SRM") &&
	    !avanti_rom_header(buf + AVANTI_FLASH_SIZE / 2, "AVANTI_ARC"))) {
		free(buf);
		fatal("%s: no AVANTI_SRM/AVANTI_ARC and AVANTI-DROM image "
		    "headers\n", filename);
		return 0;
	}

	dev_avanti_set_rom(d, buf, buf + AVANTI_FLASH_SIZE);
	free(buf);

	*entrypointp = dev_avanti_srom(d, machine->cpus[machine->bootstrap_cpu]);
	return 1;
}


MACHINE_DEFAULT_CPU(avanti)
{
	/*  The 4/233 and 4/266 have a 21064A (EV45):  */
	if (machine->machine_subtype == MACHINE_AVANTI_200_233 ||
	    machine->machine_subtype == MACHINE_AVANTI_400_233 ||
	    machine->machine_subtype == MACHINE_AVANTI_250_266)
		machine->cpu_name = strdup("21064A");
	else
		machine->cpu_name = strdup("21064");
}


MACHINE_DEFAULT_RAM(avanti)
{
	machine->physical_ram_in_mb = 64;
}


MACHINE_REGISTER(avanti)
{
	MR_DEFAULT(avanti, "AlphaStation 200/400 (Avanti)", MACHINE_AVANTI);

	machine_entry_add_alias(me, "avanti");
	machine_entry_add_alias(me, "as200");

	machine_entry_add_subtype(me, "AlphaStation 200 4/166",
	    MACHINE_AVANTI_200_166, "200", "as200", "4/166", "", NULL);
	machine_entry_add_subtype(me, "AlphaStation 200 4/233",
	    MACHINE_AVANTI_200_233, "200-233", "as200-233", NULL);
	machine_entry_add_subtype(me, "AlphaStation 400 4/233",
	    MACHINE_AVANTI_400_233, "400", "as400", "4/233", NULL);
	machine_entry_add_subtype(me, "AlphaStation 250 4/266",
	    MACHINE_AVANTI_250_266, "250", "as250", "4/266", NULL);

	me->set_default_ram = machine_default_ram_avanti;
}
