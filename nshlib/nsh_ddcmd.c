/****************************************************************************
 * apps/nshlib/nsh_ddcmd.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/clock.h>

#include <sys/types.h>
#include <sys/stat.h>

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <time.h>

#include "nsh.h"
#include "nsh_console.h"

#ifndef CONFIG_NSH_DISABLE_DD

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If no sector size is specified with BS=, then the following default value
 * is used.
 */

#define DEFAULT_SECTSIZE 512
#define g_dd "dd"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct dd_s
{
  FAR struct nsh_vtbl_s *vtbl;

  int          infd;       /* File descriptor of the input device */
  int          outfd;      /* File descriptor of the output device */
  uint32_t     nsectors;   /* Number of sectors to transfer */
  uint32_t     skip;       /* The number of sectors skipped on input */
  uint32_t     seek;       /* The number of bytes skipped on output */
  int          oflags;     /* The open flags on output deivce */
  bool         eof;        /* true: The end of the input or output file has been hit */
  size_t       sectsize;   /* Size of one sector */
  size_t       nbytes;     /* Number of valid bytes in the buffer */
  FAR uint8_t *buffer;     /* Buffer of data to write to the output file */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dd_write
 ****************************************************************************/

static int dd_write(FAR struct dd_s *dd)
{
  FAR uint8_t *buffer = dd->buffer;
  size_t written;
  ssize_t nbytes;

  /* Is the out buffer full (or is this the last one)? */

  written = 0;
  do
    {
      nbytes = write(dd->outfd, buffer, dd->nbytes - written);
      if (nbytes < 0)
        {
          FAR struct nsh_vtbl_s *vtbl = dd->vtbl;
          nsh_error(vtbl, g_fmtcmdfailed, g_dd, "write", NSH_ERRNO);
          return ERROR;
        }

      written += nbytes;
      buffer  += nbytes;
    }
  while (written < dd->nbytes);

  return OK;
}

/****************************************************************************
 * Name: dd_read
 ****************************************************************************/

static int dd_read(FAR struct dd_s *dd)
{
  FAR uint8_t *buffer = dd->buffer;
  ssize_t nbytes;

  dd->nbytes = 0;
  do
    {
      nbytes = read(dd->infd, buffer, dd->sectsize - dd->nbytes);
      if (nbytes < 0)
        {
          if (errno == EINTR)
            {
              continue;
            }

          FAR struct nsh_vtbl_s *vtbl = dd->vtbl;
          nsh_error(vtbl, g_fmtcmdfailed, g_dd, "read", NSH_ERRNO);
          return ERROR;
        }

      dd->nbytes += nbytes;
      buffer     += nbytes;
    }
  while (dd->nbytes < dd->sectsize && nbytes != 0);

  dd->eof |= (dd->nbytes == 0);
  return OK;
}

/****************************************************************************
 * Name: dd_infopen
 ****************************************************************************/

static inline int dd_infopen(FAR const char *name, FAR struct dd_s *dd)
{
  dd->infd = open(name, O_RDONLY);
  if (dd->infd < 0)
    {
      FAR struct nsh_vtbl_s *vtbl = dd->vtbl;
      nsh_error(vtbl, g_fmtcmdfailed, g_dd, "open", NSH_ERRNO);
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: dd_outfopen
 ****************************************************************************/

static inline int dd_outfopen(FAR const char *name, FAR struct dd_s *dd)
{
  dd->outfd = open(name, dd->oflags, 0644);
  if (dd->outfd < 0)
    {
      FAR struct nsh_vtbl_s *vtbl = dd->vtbl;
      nsh_error(vtbl, g_fmtcmdfailed, g_dd, "open", NSH_ERRNO);
      return ERROR;
    }

  return OK;
}

static int dd_verify(FAR const char *infile, FAR const char *outfile,
                     FAR struct dd_s *dd)
{
  FAR struct nsh_vtbl_s *vtbl = dd->vtbl;
  FAR uint8_t *buffer;
  unsigned sector = 0;
  int ret = OK;

  UNUSED(infile);
  UNUSED(outfile);

  ret = lseek(dd->infd, dd->skip ? dd->skip * dd->sectsize : 0, SEEK_SET);
  if (ret < 0)
    {
      nsh_error(vtbl, g_fmtcmdfailed, g_dd, "lseek", NSH_ERRNO);
      return ret;
    }

  dd->eof = 0;
  ret = lseek(dd->outfd, 0, SEEK_SET);
  if (ret < 0)
    {
      nsh_error(vtbl, g_fmtcmdfailed, g_dd, "lseek", NSH_ERRNO);
      return ret;
    }

  buffer = malloc(dd->sectsize);
  if (buffer == NULL)
    {
      return ERROR;
    }

  while (!dd->eof && sector < dd->nsectors)
    {
      ret = dd_read(dd);
      if (ret < 0)
        {
          break;
        }

      ret = read(dd->outfd, buffer, dd->nbytes);
      if (ret != dd->nbytes)
        {
          nsh_error(vtbl, g_fmtcmdfailed, g_dd, "read", NSH_ERRNO);
          break;
        }

      if (memcmp(dd->buffer, buffer, dd->nbytes) != 0)
        {
          char msg[32];
          snprintf(msg, sizeof(msg), "infile sector %d", sector);
          nsh_dumpbuffer(vtbl, msg, dd->buffer, dd->nbytes);
          snprintf(msg, sizeof(msg), "\noutfile sector %d", sector);
          nsh_dumpbuffer(vtbl, msg, buffer, dd->nbytes);
          nsh_output(vtbl, "\n");
          ret = ERROR;
          break;
        }

      sector++;
    }

  if (ret < 0)
    {
      nsh_error(vtbl, g_fmtcmdfailed, g_dd, "dd_verify", ret);
    }

  free(buffer);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cmd_dd
 *
 * At present, redirect of input and output are supported.
 * of= and if= arguments are required only when verify enabled.
 *
 ****************************************************************************/

int cmd_dd(FAR struct nsh_vtbl_s *vtbl, int argc, FAR char **argv)
{
  FAR struct console_stdio_s *pstate = (FAR struct console_stdio_s *)vtbl;
  struct dd_s dd;
  FAR char *infile = NULL;
  FAR char *outfile = NULL;
#ifdef CONFIG_NSH_CMDOPT_DD_STATS
  struct timespec ts0;
  struct timespec ts1;
  uint64_t elapsed;
  uint64_t total;
#endif
  uint32_t sector = 0;
  int ret = ERROR;
  int i;

  /* Initialize the dd structure */

  memset(&dd, 0, sizeof(struct dd_s));
  dd.vtbl      = vtbl;              /* For nsh_output */
  dd.sectsize  = DEFAULT_SECTSIZE;  /* Sector size if 'bs=' not provided */
  dd.nsectors  = 0xffffffff;        /* MAX_UINT32 */
  dd.oflags    = O_WRONLY | O_CREAT | O_TRUNC;

  /* If no IF= option is provided on the command line, then read
   * from stdin.
   */

  dd.infd      = INFD(pstate);      /* stdin */

  /* If no OF= option is provided on the command line, then write
   * to stdout.
   */

  dd.outfd     = OUTFD(pstate);     /* stdout */

  /* Parse command line parameters */

  for (i = 1; i < argc; i++)
    {
      if (strncmp(argv[i], "if=", 3) == 0)
        {
          if (infile != NULL)
            {
              free(infile);
            }

          infile = nsh_getfullpath(vtbl, &argv[i][3]);
        }
      else if (strncmp(argv[i], "of=", 3) == 0)
        {
          if (outfile != NULL)
            {
              free(outfile);
            }

          outfile = nsh_getfullpath(vtbl, &argv[i][3]);
        }
      else if (strncmp(argv[i], "bs=", 3) == 0)
        {
          dd.sectsize = atoi(&argv[i][3]);
        }
      else if (strncmp(argv[i], "count=", 6) == 0)
        {
          dd.nsectors = atoi(&argv[i][6]);
        }
      else if (strncmp(argv[i], "skip=", 5) == 0)
        {
          dd.skip = atoi(&argv[i][5]);
        }
      else if (strncmp(argv[i], "seek=", 5) == 0)
        {
          dd.seek = atoi(&argv[i][5]);
        }
      else if (strncmp(argv[i], "verify", 6) == 0)
        {
          dd.oflags |= O_RDONLY;
        }
      else if (strncmp(argv[i], "conv=", 5) == 0)
        {
          if (strstr(argv[i], "nocreat") != NULL)
            {
              dd.oflags &= ~(O_CREAT | O_TRUNC);
            }
          else if (strstr(argv[i], "notrunc") != NULL)
            {
              dd.oflags &= ~O_TRUNC;
            }
        }
    }

  /* If verify enabled, infile and outfile are mandatory */

  if ((dd.oflags & O_RDONLY) && (infile == NULL || outfile == NULL))
    {
      nsh_error(vtbl, g_fmtargrequired, g_dd);
      goto errout_with_paths;
    }

  /* Allocate the I/O buffer */

  dd.buffer = malloc(dd.sectsize);
  if (!dd.buffer)
    {
      nsh_error(vtbl, g_fmtcmdoutofmemory, g_dd);
      goto errout_with_paths;
    }

  /* Open the input file */

  if (infile)
    {
      ret = dd_infopen(infile, &dd);
      if (ret < 0)
        {
          goto errout_with_alloc;
        }
    }

  /* Open the output file */

  if (outfile)
    {
      ret = dd_outfopen(outfile, &dd);
      if (ret < 0)
        {
          goto errout_with_inf;
        }
    }

  /* Then perform the data transfer */

#ifdef CONFIG_NSH_CMDOPT_DD_STATS
  clock_gettime(CLOCK_MONOTONIC, &ts0);
#endif

  if (dd.skip)
    {
      ret = lseek(dd.infd, dd.skip * dd.sectsize, SEEK_SET);
      if (ret < 0)
        {
          nsh_error(vtbl, g_fmtcmdfailed, g_dd, "skip lseek", NSH_ERRNO);
          ret = ERROR;
          goto errout_with_outf;
        }
    }

  if (dd.seek)
    {
      ret = lseek(dd.outfd, dd.seek * dd.sectsize, SEEK_SET);
      if (ret < 0)
        {
          nsh_error(vtbl, g_fmtcmdfailed, g_dd, "seek lseek", NSH_ERRNO);
          ret = ERROR;
          goto errout_with_outf;
        }
    }

  while (!dd.eof && sector < dd.nsectors)
    {
      /* Read one sector from from the input */

      ret = dd_read(&dd);
      if (ret < 0)
        {
          goto errout_with_outf;
        }

      /* Has the incoming data stream ended? */

      if (!dd.eof)
        {
          /* Write one sector to the output file */

          ret = dd_write(&dd);
          if (ret < 0)
            {
              goto errout_with_outf;
            }

          /* Increment the sector number */

          sector++;
        }
    }

  ret = OK;

#ifdef CONFIG_NSH_CMDOPT_DD_STATS
  clock_gettime(CLOCK_MONOTONIC, &ts1);

  elapsed  = (((uint64_t)ts1.tv_sec * NSEC_PER_SEC) + ts1.tv_nsec);
  elapsed -= (((uint64_t)ts0.tv_sec * NSEC_PER_SEC) + ts0.tv_nsec);
  elapsed /= NSEC_PER_USEC; /* usec */

  total = ((uint64_t)sector * (uint64_t)dd.sectsize);

  nsh_output(vtbl, "%" PRIu64 "bytes copied, %" PRIu64 " usec, ",
             total, elapsed);
  nsh_output(vtbl, "%u KB/s\n" ,
             (unsigned int)(((double)total / 1024)
             / ((double)elapsed / USEC_PER_SEC)));
#endif

  if (ret == 0 && (dd.oflags & O_RDONLY) != 0)
    {
      ret = dd_verify(infile, outfile, &dd);
    }

errout_with_outf:
  if (outfile)
    {
      close(dd.outfd);
    }

errout_with_inf:
  if (infile)
    {
      close(dd.infd);
    }

errout_with_alloc:
  free(dd.buffer);

errout_with_paths:
  if (infile)
    {
      nsh_freefullpath(infile);
    }

  if (outfile)
    {
      nsh_freefullpath(outfile);
    }

  return ret;
}

#endif /* !CONFIG_NSH_DISABLE_DD */
