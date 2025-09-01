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

enum plcatool_cmd_e
{
  PLCA_CMD_SET,
  PLCA_CMD_GET,
  PLCA_CMD_STATUS,
  PLCA_CMD_HELP
};

/* Lower 8 bits are value, the most significant bit indicates whether set */

struct plca_cfg_s
{
  enum plcatool_cmd_e cmd;

  FAR char *ifname;

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

static int parse_args(int argc, FAR char *argv[], FAR struct plca_cfg_s *cfg)
{
  const char *cmd;

  if (argc < 2)
    {
      return ERROR;
    }

  cmd = argv[1];

  if (strcmp(cmd, "set") == 0)
    {
      int i;

      cfg->cmd = PLCA_CMD_SET;

      if (argc < 5)
        {
          return ERROR;
        }

      cfg->ifname = argv[2];

      i = 3;
      while (i < argc - 1)
        {
          const char *param  = argv[i++];
          const char *value = argv[i++];

          if (strcmp(param, "enable") == 0)
            {
              if (PLCA_CFG_IS_SET(cfg, enable))
                {
                  return ERROR;
                }

              if (strcmp(value, "on") == 0)
                {
                  PLCA_CFG_SET(cfg, enable, 1);
                }
              else if (strcmp(value, "off") == 0)
                {
                  PLCA_CFG_SET(cfg, enable, 0);
                }
              else
                {
                  return ERROR;
                }
            }
          else if (strcmp(param, "node-id") == 0)
            {
              int N;

              if (PLCA_CFG_IS_SET(cfg, node_id))
                {
                  return ERROR;
                }

              if (get_num(value, &N))
                {
                  return ERROR;
                }

              if (NODE_ID_MIN <= N && N <= NODE_ID_MAX)
                {
                  PLCA_CFG_SET(cfg, node_id, N);
                }
              else
                {
                  fprintf(stderr, "node-id out of range\n");
                  return ERROR;
                }
            }
          else if (strcmp(param, "node-cnt") == 0)
            {
              int N;

              if (PLCA_CFG_IS_SET(cfg, node_cnt))
                {
                  return ERROR;
                }

              if (get_num(value, &N))
                {
                  fprintf(stderr, "Not a valid interger\n");
                  return ERROR;
                }

              if (NODE_CNT_MIN <= N && N <= NODE_CNT_MAX)
                {
                  PLCA_CFG_SET(cfg, node_cnt, N);
                }
              else
                {
                  fprintf(stderr, "node-cnt out of range\n");
                  return ERROR;
                }
            }
          else if (strcmp(param, "to-tmr") == 0)
            {
              int N;

              if (PLCA_CFG_IS_SET(cfg, to_tmr))
                {
                  return ERROR;
                }

              if (get_num(value, &N))
                {
                  fprintf(stderr, "Not a valid interger\n");
                  return ERROR;
                }

              if (TO_TMR_MIN <= N && N <= TO_TMR_MAX)
                {
                  PLCA_CFG_SET(cfg, to_tmr, N);
                }
              else
                {
                  fprintf(stderr, "to_tmr out of range\n");
                  return ERROR;
                }
            }
          else if (strcmp(param, "burst-cnt") == 0)
            {
              int N;

              if (PLCA_CFG_IS_SET(cfg, burst_cnt))
                {
                  return ERROR;
                }

              if (get_num(value, &N))
                {
                  fprintf(stderr, "Not a valid interger\n");
                  return ERROR;
                }

              if (BURST_CNT_MIN <= N && N <= BURST_CNT_MAX)
                {
                  PLCA_CFG_SET(cfg, burst_cnt, N);
                }
              else
                {
                  fprintf(stderr, "burst-cnt out of range\n");
                  return ERROR;
                }
            }
          else if (strcmp(param, "burst-tmr") == 0)
            {
              int N;

              if (PLCA_CFG_IS_SET(cfg, burst_tmr))
                {
                  return ERROR;
                }

              if (get_num(value, &N))
                {
                  fprintf(stderr, "Not a valid interger\n");
                  return ERROR;
                }

              if (BURST_TMR_MIN <= N && N <= BURST_TMR_MAX)
                {
                  PLCA_CFG_SET(cfg, burst_tmr, N);
                }
              else
                {
                  fprintf(stderr, "burst-tmr out of range\n");
                  return ERROR;
                }
            }
          else
            {
              return ERROR;
            }
        }

      if (i != argc)
        {
          return ERROR;
        }
    }
  else if (strcmp(cmd, "get") == 0)
    {
      cfg->cmd = PLCA_CMD_SET;

      if (argc != 3)
        {
          return ERROR;
        }

      cfg->ifname = argv[2];
    }
  else if (strcmp(cmd, "status") == 0)
    {
      cfg->cmd = PLCA_CMD_STATUS;

      if (argc != 3)
        {
          return ERROR;
        }

      cfg->ifname = argv[2];
    }
  else if (strcmp(cmd, "-h") == 0)
    {
      cfg->cmd = PLCA_CMD_HELP;

      if (argc != 2)
        {
          return ERROR;
        }
    }

  return OK;
}

static int plcatool_set(FAR struct plca_cfg_s *cfg)
{
  return OK;
}

static int plcatool_get(FAR struct plca_cfg_s *cfg)
{
  return OK;
}

static int plcatool_status(FAR struct plca_cfg_s *cfg)
{
  return OK;
}

static void plcatool_usage(bool err)
{
  FAR FILE *out = err ? stderr : stdout;

  fprintf(out, "Usage:\n");
  fprintf(out, "  plcatool status <ifname>\n");
  fprintf(out, "  plcatool get <ifname>\n");
  fprintf(out, "  plcatool set <ifname> <param> <value> "
               "[<param> <value>] ...\n");
  fprintf(out, "  plcatool -h\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  struct plca_cfg_s cfg = { 0 };

  int err = parse_args(argc, argv, &cfg);
  if (err)
    {
      plcatool_usage(err);
      return 1;
    }

  switch(cfg.cmd)
    {
      case PLCA_CMD_SET:
          return plcatool_set(&cfg);

      case PLCA_CMD_GET:
          return plcatool_get(&cfg);

      case PLCA_CMD_STATUS:
          return plcatool_status(&cfg);

      case PLCA_CMD_HELP:
          plcatool_usage(false);
          break;
    }

  return OK;
}
