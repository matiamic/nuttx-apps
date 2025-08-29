/****************************************************************************
 * apps/netutils/plcatool/plcatool.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

#include "oa_tc14.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PLCA_CFG_SET_BIT 0x8000
#define PLCA_CFG_SET(cfg, field, val) \
    do {(cfg)->field = PLCA_CFG_SET_BIT | ((val) & 0xff);} while (0)

#define PLCA_CFG_IS_SET(cfg, field) ((cfg)->field & PLCA_CFG_SET_BIT)
#define PLCA_CFG_VAL(cfg, field)    ((cfg)->field & 0xff)

#define NODE_ID_MIN   0
#define NODE_ID_MAX   255

#define NODE_CNT_MIN  1
#define NODE_CNT_MAX  255

#define TO_TMR_MIN    0
#define TO_TMR_MAX    255

#define BURST_CNT_MIN 0
#define BURST_CNT_MAX 255

#define BURST_TMR_MIN 0
#define BURST_TMR_MAX 255

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Lower 8 bits are value, the most significant bit indicates whether set */

struct plca_cfg_s
{
  uint16_t enable;
  uint16_t node_id;
  uint16_t node_cnt;
  uint16_t to_tmr;
  uint16_t burst_cnt;
  uint16_t burst_tmr;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int get_num(FAR const char *str, FAR int *result)
{
  char *endptr;
  *result = (int) strtol(str, &endptr, 0);
  if (errno)
    {
      return ERROR;
    }

  if (*endptr != '\0')
    {
      return ERROR;
    }

  return OK;
}

static void plcatool_usage(void)
{
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  const char *cmd;
  const char *dev;

  if (argc < 3)
    {
      fprintf(stderr, "Too few args\n");
      plcatool_usage();
      return 1;
    }

  cmd = argv[1];
  dev = argv[2];

  if (strcmp(cmd, "set") == 0)
    {
      struct plca_cfg_s cfg = { 0 };
      int i;

      if (argc < 5)
        {
          fprintf(stderr, "At least one name-value pair expected\n");
          plcatool_usage();
          return 1;
        }

      i = 3;
      while (i < argc - 1)
        {
          const char *name  = argv[i++];
          const char *value = argv[i++];

          if (strcmp(name, "enable") == 0)
            {
              if (strcmp(value, "on") == 0)
                {
                  PLCA_CFG_SET(&cfg, enable, 1);
                }
              else if (strcmp(value, "off") == 0)
                {
                  PLCA_CFG_SET(&cfg, enable, 0);
                }
              else
                {
                  fprintf(stderr, "on | off are acceptable value for enable\n");
                  plcatool_usage();
                  return 1;
                }
            }
          else if (strcmp(name, "node-id") == 0)
            {
              int N;
              if (get_num(value, &N))
                {
                  fprintf(stderr, "Not a correct number format\n");
                  plcatool_usage();
                  return 1;
                }

              if (NODE_ID_MIN <= N && N <= NODE_ID_MAX)
                {
                  PLCA_CFG_SET(&cfg, node_id, N);
                }
              else
                {
                  fprintf(stderr, "Out of range\n");
                  plcatool_usage();
                  return 1;
                }
            }
          else if (strcmp(name, "node-cnt") == 0)
            {
              int N;
              if (get_num(value, &N))
                {
                  fprintf(stderr, "Not a correct number format\n");
                  plcatool_usage();
                  return 1;
                }

              if (NODE_CNT_MIN <= N && N <= NODE_CNT_MAX)
                {
                  PLCA_CFG_SET(&cfg, node_cnt, N);
                }
              else
                {
                  fprintf(stderr, "Out of range\n");
                  plcatool_usage();
                  return 1;
                }
            }
          else if (strcmp(name, "to-tmr") == 0)
            {
              int N;
              if (get_num(value, &N))
                {
                  fprintf(stderr, "Not a correct number format\n");
                  plcatool_usage();
                  return 1;
                }

              if (TO_TMR_MIN <= N && N <= TO_TMR_MAX)
                {
                  PLCA_CFG_SET(&cfg, to_tmr, N);
                }
              else
                {
                  fprintf(stderr, "Out of range\n");
                  plcatool_usage();
                  return 1;
                }
            }
          else if (strcmp(name, "burst-cnt") == 0)
            {
              int N;
              if (get_num(value, &N))
                {
                  fprintf(stderr, "Not a correct number format\n");
                  plcatool_usage();
                  return 1;
                }

              if (BURST_CNT_MIN <= N && N <= BURST_CNT_MAX)
                {
                  PLCA_CFG_SET(&cfg, burst_cnt, N);
                }
              else
                {
                  fprintf(stderr, "Out of range\n");
                  plcatool_usage();
                  return 1;
                }
            }
          else if (strcmp(name, "burst-tmr") == 0)
            {
              int N;
              if (get_num(value, &N))
                {
                  fprintf(stderr, "Not a correct number format\n");
                  plcatool_usage();
                  return 1;
                }

              if (BURST_TMR_MIN <= N && N <= BURST_TMR_MAX)
                {
                  PLCA_CFG_SET(&cfg, burst_tmr, N);
                }
              else
                {
                  fprintf(stderr, "Out of range\n");
                  plcatool_usage();
                  return 1;
                }
            }
          else
            {
              fprintf(stderr, "Usage\n");
              plcatool_usage();
              return 1;
            }
        }

      if (i != argc)
        {
          fprintf(stderr, "Trailing arguments\n");
          plcatool_usage();
          return 1;
        }
    }

  return OK;
}
