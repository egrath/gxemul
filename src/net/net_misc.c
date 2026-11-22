/*
 *  Copyright (C) 2004-2018  Anders Gavare.  All rights reserved.
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
 *  Misc. helper functions.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

#include "machine.h"
#include "misc.h"
#include "net.h"


/*
 *  net_debugaddr():
 *
 *  Print an address using debug().
 */
void net_debugaddr(void *addr, int type)
{
	int i;
	unsigned char *p = (unsigned char *) addr;

	switch (type) {

	case NET_ADDR_IPV4:
		for (i=0; i<4; i++)
			debugmsg(SUBSYS_NET, "net_misc", VERBOSITY_DEBUG, "%s%i", i? "." : "", p[i]);
		break;

	case NET_ADDR_IPV6:
		for (i=0; i<16; i+=2)
			debugmsg(SUBSYS_NET, "net_misc", VERBOSITY_DEBUG, "%s%4x", i? ":" : "", p[i] * 256 + p[i+1]);
		break;

	case NET_ADDR_ETHERNET:
		for (i=0; i<6; i++)
			debugmsg(SUBSYS_NET, "net_misc", VERBOSITY_DEBUG, "%s%02x", i? ":" : "", p[i]);
		break;

	default:
		fatal_error(SUBSYS_NET, "net_misc", "net_debugaddr(): UNIMPLEMTED type %i", type);
	}
}


/*
 *  net_random_mac():
 *
 *  A random Ethernet address in the block of the company that made the
 *  machine: DEC's 08:00:2b, Motorola's 08:00:3e:2x for MVME boards, and so
 *  on.  Machines without a known maker get DEC's.
 */
void net_random_mac(int machine_type, unsigned char *macbuf)
{
	unsigned char oui[3] = { 0x08, 0x00, 0x2b };	/*  DEC  */
	int fixed_nibble = -1;

	switch (machine_type) {
	case MACHINE_SGI:
		oui[2] = 0x69;			/*  Silicon Graphics  */
		break;
	case MACHINE_MVME88K:
	case MACHINE_MVMEPPC:
		oui[2] = 0x3e;			/*  Motorola, 08:00:3e:2x  */
		fixed_nibble = 0x20;
		break;
	case MACHINE_LUNA88K:
		oui[0] = 0x00; oui[1] = 0x00; oui[2] = 0x0a;	/*  Omron  */
		break;
	case MACHINE_HP700RX:
		oui[2] = 0x09;			/*  Hewlett-Packard  */
		break;
	case MACHINE_PREP:
		oui[2] = 0x5a;			/*  IBM  */
		break;
	case MACHINE_MACPPC:
		oui[0] = 0x00; oui[1] = 0x05; oui[2] = 0x02;	/*  Apple  */
		break;
	case MACHINE_COBALT:
		oui[0] = 0x00; oui[1] = 0x10; oui[2] = 0xe0;	/*  Cobalt  */
		break;
	}

	memcpy(macbuf, oui, 3);
	macbuf[3] = random() & 0xff;
	if (fixed_nibble >= 0)
		macbuf[3] = fixed_nibble | (macbuf[3] & 0x0f);
	macbuf[4] = random() & 0xff;
	macbuf[5] = random() & 0xff;
}


/*
 *  net_generate_unique_mac():
 *
 *  Generate a MAC address for a machine's NIC (see net_random_mac()).  The
 *  first NIC's address can be given in a machine.json, "mac" under
 *  "network".
 *
 *  The return value (6 bytes) are written to macbuf.
 */
void net_generate_unique_mac(struct machine *machine, unsigned char *macbuf)
{
	if (macbuf == NULL || machine == NULL) {
		fatal_error(SUBSYS_NET, "net_misc", "**\n**  net_generate_unique_mac(): NULL ptr\n**\n");
		return;
	}

	/*  The first NIC's address may be given in the configuration:  */
	if (machine->nic_mac_set && machine->nr_of_nics == 0) {
		memcpy(macbuf, machine->nic_mac, 6);
		machine->nr_of_nics ++;
		return;
	}

	net_random_mac(machine->machine_type, macbuf);
	machine->nr_of_nics ++;
}


/*
 *  send_udp():
 *
 *  Send a simple UDP packet to a real (physical) host. Used for distributed
 *  network simulations.
 */
void send_udp(struct in_addr *addrp, int portnr, unsigned char *packet, size_t len)
{
	int s;
	struct sockaddr_in si;

	s = socket(AF_INET, SOCK_DGRAM, 0);
	if (s < 0) {
		fatal_error(SUBSYS_NET, "net_misc", "%s: %s", "send_udp(): socket", strerror(errno));
		return;
	}

	/*  fatal_error(SUBSYS_NET, "net_misc", "send_udp(): sending to port %i", portnr);  */

	si.sin_family = AF_INET;
	si.sin_addr = *addrp;
	si.sin_port = htons(portnr);

	if (sendto(s, (const char *) packet, len, 0, (struct sockaddr *)&si,
	    sizeof(si)) != (ssize_t)len) {
		fatal_error(SUBSYS_NET, "net_misc", "%s: %s", "send_udp(): sendto", strerror(errno));
	}

	socket_close(s);
}

