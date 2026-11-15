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
 *  COMMENT: DECchip 21030 ("TGA") PCI graphics, 8 planes, with a Bt485
 *
 *  The ZLXp-E1 (and its kin): a 21030 with a 2 MB, 8-bit frame buffer (the
 *  "T8-02" configuration) and a Brooktree Bt485 RAMDAC, at 1280x1024.
 *
 *  The 128 MB BAR holds 32 copies of the 4 MB core space:
 *
 *	0 .. 1 MB	alternate ROM space (reads: a byte of the option ROM
 *			in the low byte of each longword; there is no ROM
 *			here. Writes to the first 512 KB alternate between
 *			GADR and GCTR.)
 *	1 .. 2 MB	the registers, repeated every 512 bytes
 *	2 .. 4 MB	the frame buffer (display buffer 0)
 *
 *  A write to the frame buffer, or to a graphics command register, starts
 *  an operation in the mode that the mode register gives: simple (the data
 *  are pixels), opaque/transparent/block stipple (32 pixels from a mask),
 *  opaque/transparent/block fill (spans of up to 2048 pixels), opaque and
 *  transparent lines (16 pixels along a Bresenham line), and copy (32
 *  pixels into the copy buffer, and out of it shifted). The DMA and 3D
 *  modes are not here (they are logged).
 *
 *  As no console firmware has run the board's option ROM, the video timing
 *  is set up for 1280x1024 at reset, with the screen at VRAM offset 4 KB
 *  (VVBR = 1, as DEC's firmware leaves it), and the video on.
 *
 *  See the "DECchip 21030 PCI Graphics Accelerator Reference Manual"
 *  (EC-N0683-72), NetBSD's sys/dev/pci/tga.c and sys/dev/ic/bt485.c, and
 *  the XFree86 tga driver.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cpu.h"
#include "device.h"
#include "devices.h"
#include "display.h"
#include "interrupt.h"
#include "machine.h"
#include "memory.h"
#include "misc.h"


#define	TGA_TICK_SHIFT		14
#define	TGA_RETRACE_HZ		72

#define	TGA_BAR_SIZE		0x8000000	/*  128 MB  */
#define	TGA_CORE_SIZE		0x400000	/*  4 MB (T8-02)  */
#define	TGA_REGS_OFFSET		0x100000
#define	TGA_FB_OFFSET		0x200000
#define	TGA_VRAM_SIZE		0x200000

#define	TGA_XSIZE		1280
#define	TGA_YSIZE		1024

#define	N_TGA_REGS		128		/*  512 bytes  */

/*  Register numbers (byte offset / 4):  */
#define	GCBR0	0x00	/*  copy buffer 0..7  */
#define	GFGR	0x08
#define	GBGR	0x09
#define	GPMR	0x0a	/*  plane mask  */
#define	GPXR_S	0x0b	/*  pixel mask, one-shot  */
#define	GMOR	0x0c
#define	GOPR	0x0d
#define	GPSR	0x0e
#define	GADR	0x0f
#define	GB1R	0x10
#define	GB2R	0x11
#define	GB3R	0x12
#define	GCTR	0x13
#define	GDER	0x14
#define	GREV	0x15
#define	GSMR	0x16
#define	GPXR_P	0x17	/*  pixel mask, persistent  */
#define	CCBR	0x18
#define	VHCR	0x19
#define	VVCR	0x1a
#define	VVBR	0x1b
#define	VVVR	0x1c
#define	CXYR	0x1d
#define	VSAR	0x1e
#define	SISR	0x1f
#define	GDAR	0x20
#define	GBWR	0x27
#define	GADR2	0x2b	/*  an alias of GADR  */
#define	GSWR	0x2f
#define	EPSR	0x30
#define	GSNR0	0x40	/*  slope-no-go 0..7  */
#define	GSLR0	0x48	/*  slope 0..7  */
#define	GBCR0	0x50	/*  block color 0..7  */
#define	GCSR	0x58	/*  copy 64 source (and its aliases)  */
#define	GCDR	0x59	/*  copy 64 destination  */
#define	ERWR	0x78
#define	ECGR	0x7a
#define	EPDR	0x7c
#define	SCSR	0x7e

/*  Mode register:  */
#define	GMOR_MODE		0x7f
#define	GMOR_CAPENDS		0x8000
#define	GMOR_CD			0x100000	/*  (read) copy direction  */
#define	GMOR_BA			0x200000	/*  (read) Bresenham age  */
#define	GMOR_AA			0x400000	/*  (read) address age  */
#define	GMOR_GS			0x800000	/*  (read) GPXR persistent  */

#define	MODE_SIMPLE		0x00
#define	MODE_OPAQUESTIPPLE	0x01
#define	MODE_OPAQUELINE		0x02
#define	MODE_TRANSPARENTSTIPPLE	0x05
#define	MODE_TRANSPARENTLINE	0x06
#define	MODE_COPY		0x07
#define	MODE_BLOCKSTIPPLE	0x0d
#define	MODE_OPAQUEFILL		0x21
#define	MODE_TRANSPARENTFILL	0x25
#define	MODE_BLOCKFILL		0x2d

#define	ROP_COPY		3

/*  Video valid register:  */
#define	VVVR_VIDEO		1
#define	VVVR_BLANK		2
#define	VVVR_CURSOR		4

/*  Interrupt status register:  */
#define	SISR_EOFIP		0x00001
#define	SISR_SAIP		0x00002
#define	SISR_TIP		0x00010
#define	SISR_EOFIE		0x10000
#define	SISR_SAIE		0x20000
#define	SISR_TIE		0x100000

/*  The Bt485's registers (RS3..RS0):  */
#define	BT_PALADDR_W		0x0
#define	BT_PALDATA		0x1
#define	BT_PIXMASK		0x2
#define	BT_PALADDR_R		0x3
#define	BT_COCADDR_W		0x4
#define	BT_COCDATA		0x5
#define	BT_CR0			0x6
#define	BT_COCADDR_R		0x7
#define	BT_CR1			0x8
#define	BT_CR2			0x9
#define	BT_STATUS		0xa	/*  and CR3, indirectly  */
#define	BT_CURSORRAM		0xb
#define	BT_CURXLO		0xc
#define	BT_CURXHI		0xd
#define	BT_CURYLO		0xe
#define	BT_CURYHI		0xf

#define	BT_CR0_8BIT		0x02
#define	BT_CR0_CR3		0x80	/*  CR3 through the status address  */
#define	BT_CR2_CURSOR		0x03	/*  cursor mode  */
#define	BT_CR3_64		0x04	/*  64x64 cursor  */

struct bt485 {
	unsigned char	pixmask;
	unsigned char	cr[4];		/*  CR0..CR3  */
	int		addr;		/*  palette or cursor RAM address  */
	int		rgb;		/*  0..2: the next of R, G, B  */
	unsigned char	rgb_in[3];
	int		coc_addr;	/*  cursor/overscan color address  */
	int		coc_rgb;
	unsigned char	coc[4][3];	/*  overscan, cursor colors 1..3  */
	unsigned char	cursor_ram[1024];
	int		cursor_x, cursor_y;	/*  as written  */
};

struct tga_data {
	int		subsys;
	struct machine	*machine;
	struct memory	*mem;

	/*  The BAR, as the firmware set it:  */
	uint64_t	bar_addr;
	int		bar_mapped;

	struct vfb_data	*vfb_data;
	struct memory	*fb_mem;	/*  holds only the window's framebuffer  */
	unsigned char	*vram;
	uint32_t	screen_base;	/*  VRAM offset of the top left pixel  */

	uint32_t	reg[N_TGA_REGS];
	uint32_t	gpxr;		/*  the pixel mask  */
	int		gpxr_persistent;

	/*  Copy mode:  */
	unsigned char	copybuf[64];
	unsigned char	residue[8];	/*  the quadword last shifted  */
	int		copy_dest;	/*  the next copy write is a destination  */
	int		cbuf_entry;	/*  copy buffer entry written next  */
	uint32_t	cbuf_low;	/*  an even GCBR write, held  */

	/*  The Bresenham engine's working set:  */
	uint32_t	line_addr;
	int32_t		line_error;
	int		line_length;
	int		address_new;	/*  GADR written since the last op.  */
	int		bres_new;	/*  GB3R (or a slope) written since  */
	int		slope_bits;	/*  dxGEdy, dxGE0, dyGE0 (GSWR read)  */

	/*  The VRAM offsets of the first and last pixels that changed:  */
	uint32_t	changed_low, changed_high;

	struct bt485	bt;
	uint32_t	epdr_read;	/*  the byte read from the RAMDAC  */

	int		clock_bits;	/*  ICS1562 serial bits written  */

	struct interrupt irq;
	int		irq_asserted;
	int		retrace_ticks, retrace_ticks_left;

	int		warned_mode[128];
};


/*****************************************************************************
 *
 *  The screen.
 */

/*
 *  tga_redraw():
 *
 *  Marks the part of the screen that shows the VRAM from offset low to
 *  offset high for redrawing.
 */
static void tga_redraw(struct tga_data *d, uint32_t low, uint32_t high)
{
	struct vfb_data *fb = d->vfb_data;
	uint32_t size = fb->bytes_per_line * fb->ysize;
	int x1, y1, x2, y2;

	if (low > high || high < d->screen_base ||
	    low >= d->screen_base + size)
		return;

	low = low < d->screen_base? 0 : low - d->screen_base;
	high -= d->screen_base;
	if (high >= size)
		high = size - 1;

	x1 = low % fb->bytes_per_line;   y1 = low / fb->bytes_per_line;
	x2 = high % fb->bytes_per_line;  y2 = high / fb->bytes_per_line;
	if (y1 != y2) {
		x1 = 0;
		x2 = fb->xsize - 1;
	}

	if (x1 < fb->update_x1)  fb->update_x1 = x1;
	if (x2 > fb->update_x2)  fb->update_x2 = x2;
	if (y1 < fb->update_y1)  fb->update_y1 = y1;
	if (y2 > fb->update_y2)  fb->update_y2 = y2;
}


static void tga_redraw_all(struct tga_data *d)
{
	struct vfb_data *fb = d->vfb_data;

	fb->update_x1 = fb->update_y1 = 0;
	fb->update_x2 = fb->xsize - 1;
	fb->update_y2 = fb->ysize - 1;
}


/*
 *  tga_update_screen():
 *
 *  Shows the VRAM from the video base address on, in the size that the
 *  video timing registers give. Sizes and base addresses that don't fit in
 *  the VRAM are ignored.
 */
static void tga_update_screen(struct tga_data *d)
{
	struct vfb_data *fb = d->vfb_data;
	uint32_t vhcr = d->reg[VHCR];
	int xsize = ((vhcr & 0x1ff) | ((vhcr >> 19) & 0x600)) * 4;
	int ysize = d->reg[VVCR] & 0x7ff;
	/*  The video base is in 4 KB rows (256K x n VRAMs, GDER<9> clear),
	    or 2 KB rows:  */
	uint32_t base = (d->reg[VVBR] & 0x1ff) *
	    (d->reg[GDER] & 0x200? 2048 : 4096);

	if (xsize < 64 || ysize < 64 ||
	    (uint32_t) (xsize * ysize) > TGA_VRAM_SIZE) {
		xsize = fb->xsize;
		ysize = fb->ysize;
	}

	if (base + xsize * ysize > TGA_VRAM_SIZE)
		base = d->screen_base;
	if (base + xsize * ysize > TGA_VRAM_SIZE)
		base = 0;

	if (xsize != fb->xsize || ysize != fb->ysize) {
		/*  dev_fb_resize() allocates a framebuffer of its own, which
		    isn't needed, and mustn't free the VRAM:  */
		fb->framebuffer = NULL;
		dev_fb_resize(fb, xsize, ysize);
		free(fb->framebuffer);
		fb->framebuffer = NULL;
	}

	if (fb->framebuffer == d->vram + base)
		return;

	d->screen_base = base;
	fb->framebuffer = d->vram + base;
	memory_device_update_data(d->fb_mem, fb, fb->framebuffer);
	tga_redraw_all(d);
}


/*****************************************************************************
 *
 *  The Bt485 RAMDAC.
 */



/*  A cursor color, as a gray level for the window's cursor:  */
static int bt485_gray(struct bt485 *bt, int color)
{
	unsigned char *c = bt->coc[color];
	int scale = bt->cr[0] & BT_CR0_8BIT? 1 : 4;

	return ((c[0] * 30 + c[1] * 59 + c[2] * 11) * scale / 100) *
	    N_GRAYCOLORS / 256;
}


/*
 *  tga_update_cursor():
 *
 *  Shows the Bt485's cursor (or the 21030's own, which goes through the
 *  RAMDAC's cursor colors), or neither.
 */
static void tga_update_cursor(struct tga_data *d)
{
	struct bt485 *bt = &d->bt;
	struct fb_window *win = d->vfb_data->fb_window;
	int mode = bt->cr[2] & BT_CR2_CURSOR, x, y, size = 64;
	int tga_cursor = d->reg[VVVR] & VVVR_CURSOR;
	int on = mode != 0 || tga_cursor, cx, cy;

	if (!(bt->cr[3] & BT_CR3_64) && !tga_cursor)
		size = 32;

	if (win != NULL && on) {
		for (y = 0; y < size && y < CURSOR_MAXY; y++)
			for (x = 0; x < size && x < CURSOR_MAXX; x++) {
				int p0, p1, v, c = CURSOR_COLOR_TRANSPARENT;

				if (tga_cursor) {
					/*  2 bits per pixel, a row in 16 bytes,
					    from the CCBR's base on:  */
					uint32_t a = ((d->reg[CCBR] & 0x3f0) +
					    y * 16 + x / 4) & 1023;
					v = (d->vram[a] >> ((x & 3) * 2)) & 3;
				} else {
					/*  Plane 0, then plane 1, MSB first:  */
					int i = y * (size / 8) + x / 8;
					p0 = (bt->cursor_ram[i] >> (7 - (x & 7)))
					    & 1;
					p1 = (bt->cursor_ram[i + size * size / 8]
					    >> (7 - (x & 7))) & 1;
					v = p0 | (p1 << 1);
				}

				switch (tga_cursor? 1 : mode) {
				case 1:	/*  three colors  */
					if (v)
						c = bt485_gray(bt, v);
					break;
				case 2:	/*  XGA: 0, 1 colors, 2 transparent,
					    3 complement  */
					if (v == 3)
						c = CURSOR_COLOR_INVERT;
					else if (v != 2)
						c = bt485_gray(bt, v + 1);
					break;
				case 3:	/*  X: plane 1 enables, plane 0 picks
					    color 1 or 2  */
					if (v & 2)
						c = bt485_gray(bt, (v & 1) + 1);
					break;
				}
				win->cursor_pixels[y][x] = c;
			}
	}

	if (tga_cursor) {
		cx = d->reg[CXYR] & 0xfff;
		cy = (d->reg[CXYR] >> 12) & 0xfff;
	} else {
		/*  The position is that of the lower right corner:  */
		cx = bt->cursor_x - size;
		cy = bt->cursor_y - size;
	}

	dev_fb_setcursor(d->vfb_data, cx, cy, on, size, size);
}


static void bt485_write(struct tga_data *d, int rs, uint8_t v)
{
	struct bt485 *bt = &d->bt;
	unsigned char *pal = d->vfb_data->rgb_palette;

	switch (rs) {
	case BT_PALADDR_W:
	case BT_PALADDR_R:
		bt->addr = v;
		bt->rgb = 0;
		break;
	case BT_PALDATA:
		bt->rgb_in[bt->rgb++] = v;
		if (bt->rgb == 3) {
			int c, scale = bt->cr[0] & BT_CR0_8BIT? 1 : 4;
			for (c = 0; c < 3; c++)
				pal[(bt->addr & 255) * 3 + c] =
				    bt->rgb_in[c] * scale;
			tga_redraw_all(d);
			bt->addr = (bt->addr + 1) & 255;
			bt->rgb = 0;
		}
		break;
	case BT_PIXMASK:
		bt->pixmask = v;
		break;
	case BT_COCADDR_W:
	case BT_COCADDR_R:
		bt->coc_addr = v & 3;
		bt->coc_rgb = 0;
		break;
	case BT_COCDATA:
		bt->coc[bt->coc_addr][bt->coc_rgb++] = v;
		if (bt->coc_rgb == 3) {
			bt->coc_rgb = 0;
			bt->coc_addr = (bt->coc_addr + 1) & 3;
			tga_update_cursor(d);
		}
		break;
	case BT_CR0:
		bt->cr[0] = v;
		break;
	case BT_CR1:
		bt->cr[1] = v;
		break;
	case BT_CR2:
		bt->cr[2] = v;
		tga_update_cursor(d);
		break;
	case BT_STATUS:
		/*  CR3, if CR0 says so and the address register is 1:  */
		if (bt->cr[0] & BT_CR0_CR3 && bt->addr == 1) {
			bt->cr[3] = v;
			tga_update_cursor(d);
		}
		break;
	case BT_CURSORRAM:
		/*  The address register's high bits come from CR3<1:0>:  */
		bt->cursor_ram[(((bt->cr[3] & 3) << 8) | (bt->addr & 255))
		    & 1023] = v;
		bt->addr = (bt->addr + 1) & 255;
		if (bt->addr == 0)
			bt->cr[3] = (bt->cr[3] & ~3) | ((bt->cr[3] + 1) & 3);
		tga_update_cursor(d);
		break;
	case BT_CURXLO:
		bt->cursor_x = (bt->cursor_x & 0xf00) | v;
		tga_update_cursor(d);
		break;
	case BT_CURXHI:
		bt->cursor_x = (bt->cursor_x & 0xff) | ((v & 15) << 8);
		tga_update_cursor(d);
		break;
	case BT_CURYLO:
		bt->cursor_y = (bt->cursor_y & 0xf00) | v;
		tga_update_cursor(d);
		break;
	case BT_CURYHI:
		bt->cursor_y = (bt->cursor_y & 0xff) | ((v & 15) << 8);
		tga_update_cursor(d);
		break;
	}

	debugmsg(d->subsys, "tga", VERBOSITY_DEBUG,
	    "Bt485 register %i = 0x%02x\n", rs, v);
}


static uint8_t bt485_read(struct tga_data *d, int rs)
{
	struct bt485 *bt = &d->bt;
	unsigned char *pal = d->vfb_data->rgb_palette;
	uint8_t v = 0;

	switch (rs) {
	case BT_PALADDR_W:
	case BT_PALADDR_R:
		v = bt->addr;
		break;
	case BT_PALDATA:
		v = pal[(bt->addr & 255) * 3 + bt->rgb] /
		    (bt->cr[0] & BT_CR0_8BIT? 1 : 4);
		if (++ bt->rgb == 3) {
			bt->rgb = 0;
			bt->addr = (bt->addr + 1) & 255;
		}
		break;
	case BT_PIXMASK:
		v = bt->pixmask;
		break;
	case BT_COCADDR_W:
	case BT_COCADDR_R:
		v = bt->coc_addr;
		break;
	case BT_COCDATA:
		v = bt->coc[bt->coc_addr][bt->coc_rgb];
		if (++ bt->coc_rgb == 3) {
			bt->coc_rgb = 0;
			bt->coc_addr = (bt->coc_addr + 1) & 3;
		}
		break;
	case BT_CR0:
		v = bt->cr[0];
		break;
	case BT_CR1:
		v = bt->cr[1];
		break;
	case BT_CR2:
		v = bt->cr[2];
		break;
	case BT_STATUS:
		if (bt->cr[0] & BT_CR0_CR3 && bt->addr == 1)
			v = bt->cr[3];
		else
			v = 0x80;	/*  the Bt485's ID  */
		break;
	case BT_CURSORRAM:
		v = bt->cursor_ram[(((bt->cr[3] & 3) << 8) |
		    (bt->addr & 255)) & 1023];
		bt->addr = (bt->addr + 1) & 255;
		break;
	case BT_CURXLO:	v = bt->cursor_x;		break;
	case BT_CURXHI:	v = bt->cursor_x >> 8;		break;
	case BT_CURYLO:	v = bt->cursor_y;		break;
	case BT_CURYHI:	v = bt->cursor_y >> 8;		break;
	}

	return v;
}


/*****************************************************************************
 *
 *  Drawing.
 */

/*
 *  tga_put():
 *
 *  Stores a pixel in the VRAM, through the raster operation (numbered as
 *  in X11, GXcopy = 3) and the plane mask (one byte for each of the four
 *  pixels in a longword).
 */
static void tga_put_rop(struct tga_data *d, uint32_t ofs, unsigned char src,
	int rop)
{
	unsigned char dst, pixel = 0, planes;

	ofs &= TGA_VRAM_SIZE - 1;
	dst = d->vram[ofs];
	planes = d->reg[GPMR] >> (8 * (ofs & 3));

	if (rop & 1)	pixel |= src & dst;
	if (rop & 2)	pixel |= src & ~dst;
	if (rop & 4)	pixel |= ~src & dst;
	if (rop & 8)	pixel |= ~src & ~dst;
	pixel = (pixel & planes) | (dst & ~planes);

	if (pixel == dst)
		return;

	d->vram[ofs] = pixel;
	if (ofs < d->changed_low)
		d->changed_low = ofs;
	if (ofs > d->changed_high)
		d->changed_high = ofs;
}

static void tga_put(struct tga_data *d, uint32_t ofs, unsigned char src)
{
	tga_put_rop(d, ofs, src, d->reg[GOPR] & 15);
}


/*  Simple mode: the bytes written are the pixels (bytemask: of 4).  */
static void tga_simple(struct tga_data *d, uint32_t ofs, uint32_t data,
	int bytemask)
{
	int i;

	bytemask &= d->gpxr & 15;
	for (i = 0; i < 4; i++)
		if (bytemask & (1 << i))
			tga_put(d, (ofs & ~3) + i, data >> (8 * i));
}


/*
 *  The stipple modes: bit i of the mask is the pixel at ofs + i (ofs is
 *  aligned to 4 pixels). Transparent: 1 bits store the foreground color,
 *  0 bits leave the pixel. Opaque: 1 bits store the foreground color and
 *  0 bits the background color, in the pixels whose pixel mask bits are
 *  set. The colors, like the plane mask, hold one byte for each of the
 *  four pixels in a longword.
 */
static void tga_stipple(struct tga_data *d, uint32_t ofs, uint32_t stipple,
	int opaque)
{
	uint32_t pixels = opaque? d->gpxr : stipple;
	uint32_t color;
	int i;

	ofs &= ~3;
	for (i = 0; i < 32; i++) {
		if (!(pixels & ((uint32_t) 1 << i)))
			continue;

		color = stipple & ((uint32_t) 1 << i)?
		    d->reg[GFGR] : d->reg[GBGR];
		tga_put(d, ofs + i, color >> (8 * ((ofs + i) & 3)));
	}

	d->line_addr = ofs + 32;
}


/*
 *  The block modes: the pixels from ofs on (any byte) whose mask bits are
 *  set get the block color pattern (GBCR1..0: pixel j of each 8 has color
 *  j, by its address), with only the plane mask (no raster operation).
 *  The mask is aligned to 4 pixels: bit k is for the pixel at
 *  (ofs & ~3) + k, modulo 32 (the chip rotates it).
 */
static void tga_block(struct tga_data *d, uint32_t ofs, uint32_t mask,
	int count)
{
	uint32_t base = ofs & ~3;
	int i;

	for (i = 0; i < count; i++) {
		uint32_t p = ofs + i;
		uint32_t bit = (p - base) & 31;
		int j = p & 7;

		if (!(mask & ((uint32_t) 1 << bit)))
			continue;

		tga_put_rop(d, p, d->reg[GBCR0 + (j >> 2)] >> (8 * (j & 3)),
		    ROP_COPY);
	}
}


/*
 *  The fill modes: a span of 1..2048 pixels from the address with the
 *  low two bits from the data, where the fill mask (GDAR) repeats every 32
 *  pixels, aligned as for the block modes.
 */
static void tga_fill(struct tga_data *d, uint32_t ofs, uint32_t data,
	int mode)
{
	uint32_t mask = d->reg[GDAR], base;
	int i, count = (data & 0x7ff) + 1;

	ofs = (ofs & ~3) | ((data >> 16) & 3);
	base = ofs & ~3;

	if (mode == MODE_BLOCKFILL) {
		tga_block(d, ofs, mask, count);
		return;
	}

	for (i = 0; i < count; i++) {
		uint32_t p = ofs + i;
		uint32_t bit = (uint32_t) 1 << ((p - base) & 31);
		uint32_t color;

		if (mask & bit)
			color = d->reg[GFGR];
		else if (mode == MODE_OPAQUEFILL && d->gpxr & bit)
			color = d->reg[GBGR];
		else
			continue;

		tga_put(d, p, color >> (8 * (p & 3)));
	}
}


/*
 *  tga_line():
 *
 *  The line modes draw up to 16 pixels along a line from the Bresenham
 *  engine's address: pixel i is the foreground color if bit i of the mask
 *  is set, or else the background color (opaque) or left as it is
 *  (transparent). Between pixels, the address moves by address increment
 *  1 (and the error grows by error increment 1) while the error is
 *  negative, and else by address increment 2 (and the error shrinks by
 *  error increment 2). Afterwards, the engine is at the next pixel, with a
 *  length of 16, for a continuation.
 */
static void tga_line(struct tga_data *d, uint32_t mask, int opaque)
{
	uint32_t b1 = d->reg[GB1R], b2 = d->reg[GB2R];
	int i, n;

	if (d->bres_new) {
		d->line_error = (int32_t) d->reg[GB3R] >> 15;
		d->line_length = d->reg[GB3R] & 15;
		if (d->line_length == 0)
			d->line_length = 16;
		d->bres_new = 0;
	}
	n = d->line_length;

	for (i = 0; i < n; i++) {
		uint32_t color = d->reg[GFGR];

		if (!(mask & (1 << i))) {
			if (!opaque)
				goto next;
			color = d->reg[GBGR];
		}
		tga_put(d, d->line_addr, color >> (8 * (d->line_addr & 3)));

next:
		if (d->line_error < 0) {
			d->line_addr += (int16_t) (b1 >> 16);
			d->line_error += b1 & 0xffff;
		} else {
			d->line_addr += (int16_t) (b2 >> 16);
			d->line_error -= b2 & 0xffff;
		}
	}

	d->line_length = 16;
	d->reg[GB3R] = ((uint32_t) d->line_error << 15) | 0;
}


/*
 *  tga_slope():
 *
 *  A write to a slope (or slope-no-go) register sets up the Bresenham
 *  terms for an octant: bit 2 of the register number is set for a line
 *  whose x distance is the larger, bit 1 if x increases, and bit 0 if y
 *  increases.
 */
static void tga_slope(struct tga_data *d, int octant, uint32_t value)
{
	int adx = value & 0xffff, ady = value >> 16;
	int dxgedy = (octant >> 2) & 1, dxge0 = (octant >> 1) & 1;
	int dyge0 = octant & 1;
	int width = d->reg[GBWR] & 0xffff;
	int pixbytes = d->reg[GDER] & 1? 4 : 1;
	int dmajor = dxgedy? adx : ady, dminor = dxgedy? ady : adx;
	int majorge0 = dxgedy? dxge0 : dyge0;
	int minorge0 = dxgedy? dyge0 : dxge0;
	int amajor = dxgedy? pixbytes : width;
	int aminor = dxgedy? width : pixbytes;
	int errinc, a1, a2, length;
	int32_t error;

	if (d->reg[GMOR] & 0x2000)	/*  Win32  */
		errinc = dxgedy? dyge0 : !dxge0;
	else
		errinc = majorge0;

	length = (dmajor + (d->reg[GMOR] & GMOR_CAPENDS? 1 : 0)) & 15;
	error = ((dminor << 1) - dmajor - 1 + errinc) >> 1;
	a1 = majorge0? amajor : -amajor;
	a2 = (minorge0? aminor : -aminor) + a1;

	d->reg[GB1R] = ((uint32_t) (a1 & 0xffff) << 16) | dminor;
	d->reg[GB2R] = ((uint32_t) (a2 & 0xffff) << 16) |
	    ((dmajor - dminor) & 0xffff);
	d->reg[GB3R] = ((uint32_t) error << 15) | length;
	d->bres_new = 1;
	d->slope_bits = octant & 7;
}


/*
 *  Copy mode, as on the SFB (see dev_sfb.c): writes take turns at being a
 *  source and a destination. A source write loads 32 pixels into the copy
 *  buffer: the quadword at the address and the three to the right of it,
 *  or (with a negative pixel shift) to the left of it, in the order they
 *  are read, through a byte shifter with the residue of the last quadword
 *  read. A destination write stores the buffer's pixels whose mask bits
 *  are set, in the same order.
 */
static int tga_copy_index(int i, int shift)
{
	return shift < 0? 24 - (i & ~7) + (i & 7) : i;
}

static void tga_copy(struct tga_data *d, uint32_t ofs, uint32_t mask)
{
	int i, shift = d->reg[GPSR] & 15;
	unsigned char pixels[40];	/*  the buffer and the residue  */
	unsigned char *buf;

	if (shift & 8)
		shift -= 16;

	/*  The leftmost of the 32 pixels:  */
	ofs = (ofs & ~7) - (shift < 0? 24 : 0);

	if (!d->copy_dest) {
		for (i = 0; i < 32; i++)
			pixels[i] = d->vram[(ofs + i) & (TGA_VRAM_SIZE - 1)];

		/*  The shifted pixels, from left to right, with the residue
		    at the left end (or, shifting the other way, the right):  */
		buf = shift < 0? pixels : pixels + 8;
		memmove(buf, pixels, 32);
		memcpy(shift < 0? pixels + 32 : pixels, d->residue, 8);
		memcpy(d->residue, buf + (shift < 0? 0 : 24), 8);

		for (i = 0; i < 32; i++)
			d->copybuf[tga_copy_index(i, shift)] = buf[i - shift];

		d->copy_dest = 1;
		return;
	}

	for (i = 0; i < 32; i++)
		if (mask & ((uint32_t) 1 << tga_copy_index(i, shift)))
			tga_put(d, ofs + i,
			    d->copybuf[tga_copy_index(i, shift)]);

	d->copy_dest = 0;
	d->line_addr = ofs + (shift < 0? -8 : 32);
}


/*  The copy 64 registers: unmasked 64-byte copies through the buffer.  */
static void tga_copy64(struct tga_data *d, uint32_t addr, int dest)
{
	int i, shift = d->reg[GPSR] & 15;
	unsigned char in[72];

	if (shift & 8)
		shift -= 16;

	addr &= 0xffffff & ~7;
	d->changed_low = (uint32_t) -1;
	d->changed_high = 0;

	if (!dest) {
		/*  Read forward (or backward), shifting as copy mode does:  */
		for (i = 0; i < 64; i++)
			in[shift < 0? i : i + 8] = d->vram[(addr -
			    (shift < 0? 56 : 0) + i) & (TGA_VRAM_SIZE - 1)];
		memcpy(shift < 0? in + 64 : in, d->residue, 8);
		memcpy(d->residue, (shift < 0? in : in + 64), 8);
		for (i = 0; i < 64; i++)
			d->copybuf[i] = in[(shift < 0? i : i + 8) - shift];
		return;
	}

	for (i = 0; i < 64; i++)
		tga_put(d, addr - (shift < 0? 56 : 0) + i, d->copybuf[i]);

	tga_redraw(d, d->changed_low, d->changed_high);
}


/*
 *  tga_operation():
 *
 *  Does what a write of data to the frame buffer at offset ofs (or to a
 *  graphics command register, with the address from GADR or the engine)
 *  does in the current mode, and marks the pixels that changed for
 *  redrawing.
 */
static void tga_operation(struct tga_data *d, uint32_t ofs, uint32_t data,
	int bytemask)
{
	int mode = d->reg[GMOR] & GMOR_MODE;

	ofs &= TGA_VRAM_SIZE - 1;

	d->changed_low = (uint32_t) -1;
	d->changed_high = 0;

	switch (mode) {
	case MODE_SIMPLE:
		tga_simple(d, ofs, data, bytemask);
		break;
	case MODE_OPAQUESTIPPLE:
	case MODE_TRANSPARENTSTIPPLE:
		tga_stipple(d, ofs, data, mode == MODE_OPAQUESTIPPLE);
		break;
	case MODE_BLOCKSTIPPLE:
		tga_block(d, ofs, data, 32);
		break;
	case MODE_OPAQUEFILL:
	case MODE_TRANSPARENTFILL:
	case MODE_BLOCKFILL:
		tga_fill(d, ofs, data, mode);
		break;
	case MODE_OPAQUELINE:
	case MODE_TRANSPARENTLINE:
		tga_line(d, data, mode == MODE_OPAQUELINE);
		break;
	case MODE_COPY:
		tga_copy(d, ofs, data);
		break;
	default:
		if (!d->warned_mode[mode]) {
			debugmsg(d->subsys, "tga", VERBOSITY_WARNING,
			    "mode 0x%02x is not implemented\n", mode);
			d->warned_mode[mode] = 1;
		}
	}

	/*  A one-shot pixel mask is for one operation only:  */
	if (!d->gpxr_persistent)
		d->gpxr = 0xffffffff;
	d->address_new = 0;

	tga_redraw(d, d->changed_low, d->changed_high);
}


/*  A frame buffer write: in the line modes, the data's bits 17..16 are
    the low address bits.  */
static void tga_fb_write(struct tga_data *d, uint32_t ofs, uint32_t data,
	int bytemask)
{
	int mode = d->reg[GMOR] & GMOR_MODE;

	if (mode == MODE_OPAQUELINE || mode == MODE_TRANSPARENTLINE)
		d->line_addr = (ofs & ~3) | ((data >> 16) & 3);

	tga_operation(d, ofs, data, bytemask);
}


/*  A write to GCTR (or a slope register): the address is GADR's, if it
    was written since the last operation, else the engine's.  */
static void tga_command(struct tga_data *d, uint32_t data)
{
	int mode = d->reg[GMOR] & GMOR_MODE;
	uint32_t ofs;

	if (d->address_new)
		d->line_addr = d->reg[GADR];
	ofs = d->line_addr;

	if (mode == MODE_OPAQUELINE || mode == MODE_TRANSPARENTLINE)
		data &= 0xffff;

	tga_operation(d, ofs, data, 15);
}


/*****************************************************************************
 *
 *  Registers.
 */

static void tga_irq_update(struct tga_data *d)
{
	uint32_t s = d->reg[SISR];
	int on = ((s & SISR_EOFIP) && (s & SISR_EOFIE)) ||
	    ((s & SISR_SAIP) && (s & SISR_SAIE)) ||
	    ((s & SISR_TIP) && (s & SISR_TIE));

	if (on && !d->irq_asserted)
		INTERRUPT_ASSERT(d->irq);
	if (!on && d->irq_asserted)
		INTERRUPT_DEASSERT(d->irq);
	d->irq_asserted = on;
}


static uint32_t tga_reg_read(struct tga_data *d, int r)
{
	uint32_t v = d->reg[r];
	int i;

	switch (r) {
	case GCBR0: case GCBR0+1: case GCBR0+2: case GCBR0+3:
	case GCBR0+4: case GCBR0+5: case GCBR0+6: case GCBR0+7:
	case GSNR0: case GSNR0+1: case GSNR0+2: case GSNR0+3:
	case GSNR0+4: case GSNR0+5: case GSNR0+6: case GSNR0+7:
		/*  The copy buffer, entries 0..3 and (GSNR) 4..7:  */
		i = (r >= GSNR0? 32 + (r - GSNR0) * 4 : r * 4);
		v = d->copybuf[i] | (d->copybuf[i+1] << 8) |
		    (d->copybuf[i+2] << 16) | ((uint32_t) d->copybuf[i+3] << 24);
		break;
	case GMOR:
		v = (v & 0xffff) | (d->copy_dest? GMOR_CD : 0) |
		    (d->bres_new? GMOR_BA : 0) | (d->address_new? GMOR_AA : 0) |
		    (d->gpxr_persistent? GMOR_GS : 0);
		break;
	case GPXR_S:
	case GPXR_P:
		v = d->gpxr;
		break;
	case GADR:
	case GADR2:
		v = d->reg[GADR];
		break;
	case GCTR:
		v = 0;		/*  the Z address increments  */
		break;
	case GREV:
		v = 0x02;	/*  21030 step B  */
		break;
	case GSWR:
		v = d->slope_bits;
		break;
	case SCSR:
		v = 0;		/*  never busy  */
		break;
	case EPDR:
		v = d->epdr_read;
		break;
	}

	return v;
}


static void tga_reg_write(struct tga_data *d, int r, uint32_t v)
{
	uint32_t old = d->reg[r];

	switch (r) {

	case GCBR0: case GCBR0+1: case GCBR0+2: case GCBR0+3:
	case GCBR0+4: case GCBR0+5: case GCBR0+6: case GCBR0+7:
		/*  Even registers hold the low longword, odd ones store it
		    with the high one into the next copy buffer entry:  */
		if (!(r & 1)) {
			d->cbuf_low = v;
		} else {
			unsigned char *p = d->copybuf + (d->cbuf_entry & 7) * 8;
			int i;
			for (i = 0; i < 4; i++) {
				p[i] = d->cbuf_low >> (8 * i);
				p[i + 4] = v >> (8 * i);
			}
			d->cbuf_entry ++;
		}
		d->copy_dest = 1;
		return;

	case GPXR_S:
		d->gpxr = v;
		d->gpxr_persistent = 0;
		return;
	case GPXR_P:
		d->gpxr = v;
		d->gpxr_persistent = 1;
		return;

	case GPSR:
		d->reg[r] = v & 15;
		d->copy_dest = 0;	/*  the next copy reads the source  */
		d->cbuf_entry = 0;
		return;

	case GADR:
	case GADR2:
		d->reg[GADR] = v;
		d->address_new = 1;
		return;

	case GB3R:
		d->reg[r] = v;
		d->bres_new = 1;
		return;

	case GCTR:
		d->reg[r] = v;
		tga_command(d, v);
		return;

	case GDER:
		d->reg[r] = v & 0x157fd;
		tga_update_screen(d);
		return;

	case VHCR:
	case VVCR:
	case VVBR:
		d->reg[r] = v;
		if (r == VVBR || d->reg[VVVR] & VVVR_VIDEO)
			tga_update_screen(d);
		return;

	case VVVR:
		d->reg[r] = v & 7;
		if ((v & VVVR_VIDEO) && !(old & VVVR_VIDEO))
			tga_update_screen(d);
		if ((v ^ old) & VVVR_CURSOR)
			tga_update_cursor(d);
		return;

	case CXYR:
	case CCBR:
		d->reg[r] = v;
		if (d->reg[VVVR] & VVVR_CURSOR)
			tga_update_cursor(d);
		return;

	case SISR:
		/*  Pending bits are cleared by writing ones:  */
		d->reg[r] = (v & (SISR_EOFIE | SISR_SAIE | SISR_TIE)) |
		    (old & ~v & (SISR_EOFIP | SISR_SAIP | SISR_TIP));
		tga_irq_update(d);
		return;

	case EPSR:
		d->reg[r] = v & 0x1f;
		/*  A read strobe (bit 0) of a Bt485 register (bits 4..1):  */
		if (v & 1)
			d->epdr_read = (uint32_t) bt485_read(d, (v >> 1) & 15)
			    << 16;
		return;

	case EPDR:
		/*  The register select is in bits 12..9, and bit 8 clear
		    strobes the write:  */
		if (!(v & 0x100))
			bt485_write(d, (v >> 9) & 15, v & 0xff);
		return;

	case ECGR:
		/*  The ICS1562 clock: 56 bits, the last with hold set.  */
		d->clock_bits = v & 2? 0 : d->clock_bits + 1;
		return;

	case SCSR:
	case ERWR:
		return;

	case GCSR: case GCSR+2: case GCSR+4: case GCSR+6:
		tga_copy64(d, v, 0);
		return;
	case GCDR: case GCDR+2: case GCDR+4: case GCDR+6:
		tga_copy64(d, v, 1);
		return;
	}

	if (r >= GSNR0 && r < GSNR0 + 8) {
		tga_slope(d, r - GSNR0, v);
		return;
	}

	if ((r >= GSLR0 && r < GSLR0 + 8) || r == GSWR) {
		int mode = d->reg[GMOR] & GMOR_MODE;
		tga_slope(d, r == GSWR? 7 : r - GSLR0, v);
		if (mode == MODE_OPAQUELINE || mode == MODE_TRANSPARENTLINE)
			tga_command(d, d->reg[GDAR] & 0xffff);
		return;
	}

	d->reg[r] = v;
}


/*****************************************************************************
 *
 *  The BAR.
 */

static uint32_t tga_load32(const unsigned char *p)
{
	return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t) p[3] << 24);
}

static void tga_store32(unsigned char *p, uint32_t v)
{
	p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24;
}


/*  One longword's access at offset ofs (aligned), of the bytes in mask.  */
static void tga_access32(struct tga_data *d, uint32_t ofs, unsigned char *buf,
	int bytemask, int writeflag)
{
	uint32_t core = ofs & (TGA_CORE_SIZE - 1);
	uint32_t v = tga_load32(buf);

	if (core < TGA_REGS_OFFSET) {
		/*  Alternate ROM space: no ROM. Writes to the first 512 KB
		    are GADR (even longwords) and GCTR (odd ones).  */
		if (writeflag == MEM_WRITE) {
			if (core < 0x80000)
				tga_reg_write(d, (core & 4)? GCTR : GADR, v);
		} else
			tga_store32(buf, 0xff);
	} else if (core < TGA_FB_OFFSET) {
		int r = (core >> 2) & (N_TGA_REGS - 1);

		if (writeflag == MEM_WRITE) {
			debugmsg(d->subsys, "tga", VERBOSITY_DEBUG,
			    "write to register 0x%03x: 0x%08x\n", r * 4, v);
			tga_reg_write(d, r, v);
		} else {
			v = tga_reg_read(d, r);
			debugmsg(d->subsys, "tga", VERBOSITY_DEBUG,
			    "read from register 0x%03x: 0x%08x\n", r * 4, v);
			tga_store32(buf, v);
		}
	} else {
		uint32_t fb = core - TGA_FB_OFFSET;

		if (writeflag == MEM_WRITE)
			tga_fb_write(d, fb, v, bytemask);
		else
			memcpy(buf, d->vram + fb, 4);
	}
}


DEVICE_ACCESS(tga)
{
	struct tga_data *d = (struct tga_data *) extra;
	uint32_t ofs = relative_addr & (TGA_BAR_SIZE - 1);
	size_t i;

	/*  Longword by longword, with the bytes that are accessed:  */
	for (i = 0; i < len; ) {
		uint32_t lw = (ofs + i) & ~3;
		int first = (ofs + i) & 3, n = 4 - first, j, mask = 0;
		unsigned char buf[4] = { 0, 0, 0, 0 };

		if ((size_t) n > len - i)
			n = len - i;
		for (j = 0; j < n; j++) {
			mask |= 1 << (first + j);
			if (writeflag == MEM_WRITE)
				buf[first + j] = data[i + j];
		}

		tga_access32(d, lw, buf, mask, writeflag);

		if (writeflag == MEM_READ)
			for (j = 0; j < n; j++)
				data[i + j] = buf[first + j];
		i += n;
	}

	return 1;
}


DEVICE_TICK(tga)
{
	struct tga_data *d = (struct tga_data *) extra;

	if (d->reg[SISR] & SISR_TIE)
		d->reg[SISR] |= SISR_TIP;

	if (-- d->retrace_ticks_left <= 0) {
		d->retrace_ticks_left = d->retrace_ticks;

		/*  The end of a frame, at the start of vertical sync:  */
		if (d->reg[VVVR] & VVVR_VIDEO)
			d->reg[SISR] |= SISR_EOFIP;
	}

	tga_irq_update(d);
}


/*
 *  dev_tga_set_bar():
 *
 *  Where the BAR is (in the emulated physical address space), and whether
 *  memory accesses are enabled.
 */
void dev_tga_set_bar(struct tga_data *d, uint64_t addr, int enable)
{
	int i;

	if (d->bar_mapped && (!enable || addr != d->bar_addr)) {
		for (i = 0; i < d->mem->n_mmapped_devices; i++)
			if (d->mem->devices[i].extra == d &&
			    d->mem->devices[i].baseaddr == d->bar_addr) {
				memory_device_remove(d->mem, i);
				break;
			}
		d->bar_mapped = 0;
	}

	if (!enable || d->bar_mapped)
		return;

	for (i = 0; i < d->mem->n_mmapped_devices; i++)
		if (addr < d->mem->devices[i].endaddr &&
		    addr + TGA_BAR_SIZE > d->mem->devices[i].baseaddr) {
			debugmsg(d->subsys, "tga", VERBOSITY_WARNING,
			    "BAR at 0x%llx is taken; not mapped\n",
			    (long long) addr);
			return;
		}

	memory_device_register(d->mem, "tga", addr, TGA_BAR_SIZE,
	    dev_tga_access, d, DM_DEFAULT, NULL);
	d->bar_addr = addr;
	d->bar_mapped = 1;
}


/*
 *  dev_tga_init():
 *
 *  A ZLXp-E1 class TGA, interrupting on irq_path. Its registers appear once
 *  the BAR is set (dev_tga_set_bar()).
 */
struct tga_data *dev_tga_init(struct machine *machine, struct memory *mem,
	const char *irq_path)
{
	struct tga_data *d;
	int i;

	CHECK_ALLOCATION(d = (struct tga_data *) malloc(sizeof(*d)));
	memset(d, 0, sizeof(*d));
	d->subsys = debugmsg_register_subsystem("tga");
	d->machine = machine;
	d->mem = mem;

	CHECK_ALLOCATION(d->vram = (unsigned char *) malloc(TGA_VRAM_SIZE));
	memset(d->vram, 0, TGA_VRAM_SIZE);

	/*  The window's framebuffer is in memory of its own, pointed at the
	    part of the VRAM that is on the screen (as for the SFB):  */
	d->fb_mem = memory_new(TGA_VRAM_SIZE);
	d->vfb_data = dev_fb_init(machine, d->fb_mem, 0, VFB_GENERIC,
	    TGA_XSIZE, TGA_YSIZE, TGA_XSIZE, TGA_YSIZE, 8, "ZLXp-E1");
	free(d->vfb_data->framebuffer);
	d->vfb_data->framebuffer = NULL;

	/*  At reset (GOPR = GXcopy, all pixels enabled):  */
	d->reg[GOPR] = ROP_COPY;
	d->reg[GDAR] = 0xffffffff;
	d->reg[GPMR] = 0xffffffff;
	d->gpxr = 0xffffffff;

	/*
	 *  As the console firmware leaves it: 1280x1024 (active 320
	 *  quad-pixels, 1024 lines; porches and syncs as DEC's 130 MHz
	 *  timing), the screen at the second 4 KB row, the video on, and a
	 *  gray scale palette.
	 */
	d->reg[VHCR] = (320 & 0x1ff) | (14 << 9) | (28 << 14) | (58 << 21);
	d->reg[VVCR] = 1024 | (1 << 11) | (3 << 16) | (33 << 22);
	d->reg[VVBR] = 1;
	d->reg[VVVR] = VVVR_VIDEO;
	d->bt.cr[0] = BT_CR0_8BIT;
	d->bt.pixmask = 0xff;
	for (i = 0; i < 256; i++)
		d->vfb_data->rgb_palette[i * 3] =
		    d->vfb_data->rgb_palette[i * 3 + 1] =
		    d->vfb_data->rgb_palette[i * 3 + 2] = i;
	for (i = 1; i < 4; i++)
		d->bt.coc[i][0] = d->bt.coc[i][1] = d->bt.coc[i][2] =
		    i == 1? 0 : 255;
	tga_update_screen(d);

	INTERRUPT_CONNECT(irq_path, d->irq);

	d->retrace_ticks = machine->emulated_hz /
	    (TGA_RETRACE_HZ << TGA_TICK_SHIFT);
	if (d->retrace_ticks < 1)
		d->retrace_ticks = 1;
	d->retrace_ticks_left = d->retrace_ticks;

	machine_add_tickfunction(machine, dev_tga_tick, d, TGA_TICK_SHIFT);

	return d;
}

