/*
 *  COMMENT: Dreamcast emulation (helper module)
 *
 *  This is from Marcus Comstedt's Dreamcast development files
 *  (http://mc.pp.se/dc/files/scramble.c). Public Domain, according to
 *  Marcus software page.
 *
 *  The only modification done to make it fit into the emulator is that
 *  it is executed internally from src/file.c.
 */

/*  #define STAND_ALONE  */

#include <stdio.h>
#include "misc.h"
#include <stdlib.h>
#include <string.h>

#define MAXCHUNK (2048*1024)

static unsigned int seed;

void my_srand(unsigned int n)
{
  seed = n & 0xffff;
}

unsigned int my_rand(void)
{
  seed = (seed * 2109 + 9273) & 0x7fff;
  return (seed + 0xc000) & 0xffff;
}

void load(FILE *fh, unsigned char *ptr, unsigned long sz)
{
  if(fread(ptr, 1, sz, fh) != sz)
    {
      debugmsg(SUBSYS_PROMEMUL, "dreamcast_scramble", VERBOSITY_DEBUG,  "Read error!\n");
    }
}

void load_chunk(FILE *fh, unsigned char *ptr, unsigned long sz)
{
  static int idx[MAXCHUNK/32];
  int i;

  /* Convert chunk size to number of slices */
  sz /= 32;

  /* Initialize index table with unity,
     so that each slice gets loaded exactly once */
  for(i = 0; i < (int)sz; i++)
    idx[i] = i;

  for(i = sz-1; i >= 0; --i)
    {
      /* Select a replacement index */
      int x = (my_rand() * i) >> 16;

      /* Swap */
      int tmp = idx[i];
      idx[i] = idx[x];
      idx[x] = tmp;

      /* Load resulting slice */
      load(fh, ptr+32*idx[i], 32);
    }
}

void load_file(FILE *fh, unsigned char *ptr, unsigned long filesz)
{
  unsigned long chunksz;

  my_srand(filesz);

  /* Descramble 2 meg blocks for as long as possible, then
     gradually reduce the window down to 32 bytes (1 slice) */
  for(chunksz = MAXCHUNK; chunksz >= 32; chunksz >>= 1)
    while(filesz >= chunksz)
      {
	load_chunk(fh, ptr, chunksz);
	filesz -= chunksz;
	ptr += chunksz;
      }

  /* Load final incomplete slice */
  if(filesz)
    load(fh, ptr, filesz);
}

void read_file(char *filename, unsigned char **ptr, unsigned long *sz)
{
  FILE *fh = fopen(filename, "rb");
  if(fh == NULL)
    {
      debugmsg(SUBSYS_PROMEMUL, "dreamcast_scramble", VERBOSITY_DEBUG,  "Can't open \"%s\".", filename);
    }
  if(fseek(fh, 0, SEEK_END)<0)
    {
      debugmsg(SUBSYS_PROMEMUL, "dreamcast_scramble", VERBOSITY_DEBUG,  "Seek error.\n");
    }
  *sz = ftell(fh);
  *ptr = (unsigned char *) malloc(*sz);
  if( *ptr == NULL )
    {
      debugmsg(SUBSYS_PROMEMUL, "dreamcast_scramble", VERBOSITY_DEBUG,  "Out of memory.\n");
    }
  if(fseek(fh, 0, SEEK_SET)<0)
    {
      debugmsg(SUBSYS_PROMEMUL, "dreamcast_scramble", VERBOSITY_DEBUG,  "Seek error.\n");
    }
  load_file(fh, *ptr, *sz);
  fclose(fh);
}

void save(FILE *fh, unsigned char *ptr, unsigned long sz)
{
  if(fwrite(ptr, 1, sz, fh) != sz)
    {
      debugmsg(SUBSYS_PROMEMUL, "dreamcast_scramble", VERBOSITY_DEBUG,  "Write error!\n");
    }
}

void save_chunk(FILE *fh, unsigned char *ptr, unsigned long sz)
{
  static int idx[MAXCHUNK/32];
  int i;

  /* Convert chunk size to number of slices */
  sz /= 32;

  /* Initialize index table with unity,
     so that each slice gets saved exactly once */
  for(i = 0; i < (int)sz; i++)
    idx[i] = i;

  for(i = sz-1; i >= 0; --i)
    {
      /* Select a replacement index */
      int x = (my_rand() * i) >> 16;

      /* Swap */
      int tmp = idx[i];
      idx[i] = idx[x];
      idx[x] = tmp;

      /* Save resulting slice */
      save(fh, ptr+32*idx[i], 32);
    }
}

void save_file(FILE *fh, unsigned char *ptr, unsigned long filesz)
{
  unsigned long chunksz;

  my_srand(filesz);

  /* Descramble 2 meg blocks for as long as possible, then
     gradually reduce the window down to 32 bytes (1 slice) */
  for(chunksz = MAXCHUNK; chunksz >= 32; chunksz >>= 1)
    while(filesz >= chunksz)
      {
	save_chunk(fh, ptr, chunksz);
	filesz -= chunksz;
	ptr += chunksz;
      }

  /* Save final incomplete slice */
  if(filesz)
    save(fh, ptr, filesz);
}

void write_file(char *filename, unsigned char *ptr, unsigned long sz)
{
  FILE *fh = fopen(filename, "wb");
  if(fh == NULL)
    {
      debugmsg(SUBSYS_PROMEMUL, "dreamcast_scramble", VERBOSITY_DEBUG,  "Can't open \"%s\".", filename);
    }
  save_file(fh, ptr, sz);
  fclose(fh);
}

void dreamcast_descramble(char *src, char *dst)
{
  unsigned char *ptr = NULL;
  unsigned long sz = 0;
  FILE *fh;

  read_file(src, &ptr, &sz);

  fh = fopen(dst, "wb");
  if(fh == NULL)
    {
      debugmsg(SUBSYS_PROMEMUL, "dreamcast_scramble", VERBOSITY_DEBUG,  "Can't open \"%s\".", dst);
    }
  if( fwrite(ptr, 1, sz, fh) != sz )
    {
      debugmsg(SUBSYS_PROMEMUL, "dreamcast_scramble", VERBOSITY_DEBUG,  "Write error.\n");
    }
  fclose(fh);
  free(ptr);
}

void scramble(char *src, char *dst)
{
  unsigned char *ptr = NULL;
  unsigned long sz = 0;
  FILE *fh;

  fh = fopen(src, "rb");
  if(fh == NULL)
    {
      debugmsg(SUBSYS_PROMEMUL, "dreamcast_scramble", VERBOSITY_DEBUG,  "Can't open \"%s\".", src);
    }
  if(fseek(fh, 0, SEEK_END)<0)
    {
      debugmsg(SUBSYS_PROMEMUL, "dreamcast_scramble", VERBOSITY_DEBUG,  "Seek error.\n");
    }
  sz = ftell(fh);
  ptr = (unsigned char *) malloc(sz);
  if( ptr == NULL )
    {
      debugmsg(SUBSYS_PROMEMUL, "dreamcast_scramble", VERBOSITY_DEBUG,  "Out of memory.\n");
    }
  if(fseek(fh, 0, SEEK_SET)<0)
    {
      debugmsg(SUBSYS_PROMEMUL, "dreamcast_scramble", VERBOSITY_DEBUG,  "Seek error.\n");
    }
  if( fread(ptr, 1, sz, fh) != sz )
    {
      debugmsg(SUBSYS_PROMEMUL, "dreamcast_scramble", VERBOSITY_DEBUG,  "Read error.\n");
    }
  fclose(fh);

  write_file(dst, ptr, sz);

  free(ptr);
}


#ifdef STAND_ALONE

int main(int argc, char *argv[])
{
  int opt = 0;

  if(argc > 1 && !strcmp(argv[1], "-d"))
    opt ++;

  if(argc != 3+opt)
    {
      debugmsg(SUBSYS_PROMEMUL, "dreamcast_scramble", VERBOSITY_DEBUG,  "Usage: %s [-d] from to", argv[0]);
    }
  
  if(opt)
    dreamcast_descramble(argv[2], argv[3]);
  else
    scramble(argv[1], argv[2]);

  return 0;
}

#endif
