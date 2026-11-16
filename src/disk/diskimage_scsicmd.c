/*
 *  Copyright (C) 2003-2021  Anders Gavare.  All rights reserved.
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
 *  Disk image support: SCSI command emulation.
 *
 *  TODO:  There are LOTS of ugly magic values in this module. These should
 *         be replaced by proper defines.
 *
 *  TODO:  scsi_transfer_free could save away previous allocations, and
 *         scsi_transfer_allocbuf could reuse them (for given sizes).
 *
 *  TODO:  There's probably a bug in the (raw, non-SIMH) tape support:
 *         Let's say there are 10240 bytes left in a file, and 10240
 *         bytes are read. Then feof() is not true yet (?), so the next
 *         read will also return 10240 bytes (but all zeroes), and then after
 *         that return feof (which results in a filemark).  This is probably
 *         trivial to fix, but I don't feel like it right now.
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "cpu.h"
#include "diskimage.h"
#include "machine.h"
#include "misc.h"


static const char *diskimage_types[] = DISKIMAGE_TYPES;
static struct scsi_transfer *first_free_scsi_transfer_alloc = NULL;
static int scsi_subsys = -1;


/*
 *  scsi_transfer_alloc():
 *
 *  Allocates memory for a new scsi_transfer struct, and fills it with
 *  sane data (NULL pointers).
 *  The return value is a pointer to the new struct.  If allocation
 *  failed, the program exits.
 */
struct scsi_transfer *scsi_transfer_alloc(void)
{
	struct scsi_transfer *p;

	if (first_free_scsi_transfer_alloc != NULL) {
		p = first_free_scsi_transfer_alloc;
		first_free_scsi_transfer_alloc = p->next_free;
	} else {
		p = (struct scsi_transfer *) malloc(sizeof(struct scsi_transfer));
		if (p == NULL) {
			fatal_error(SUBSYS_DISK, "diskimage_scsicmd",  "scsi_transfer_alloc(): out "
			    "of memory\n");
		}
	}

	memset(p, 0, sizeof(struct scsi_transfer));

	return p;
}


/*
 *  scsi_transfer_free():
 *
 *  Frees the space used by a scsi_transfer struct.  All buffers refered
 *  to by the scsi_transfer struct are freed.
 */
void scsi_transfer_free(struct scsi_transfer *p)
{
	if (p == NULL) {
		fatal_error(SUBSYS_DISK, "diskimage_scsicmd",  "scsi_transfer_free(): p == NULL");
	}
}

