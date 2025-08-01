#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/socket.h>

#include <sys/ioctl.h>
#include <net/if.h>

#define IFNAME "eth0"

#define PHY_CONTROL_ADDR   0x00
#define PHY_CONTROL_PD_POS 11

/* In the MAC-PHY HW the following is the reflection of the above,
 * but the following must be accessed with MMD while, the above
 * is accessed through the basic MII interface access mechanism*/

#define PMA_MMD 1
#define PMA_CONTROL_ADDR 0x08F9
#define PMA_CONTROL_LPE_POS 11

static void prep_mmd(struct ifreq *req,
                     uint8_t mmd, uint16_t address, uint16_t data)
{
  strcpy(req->ifr_name, IFNAME);
  req->ifr_ifru.ifru_mii_data.reg_num = mmd;
  req->ifr_ifru.ifru_mii_data.addr = address;
  req->ifr_ifru.ifru_mii_data.val_in = data;
}

static void prep_mii(struct ifreq *req, uint8_t reg, uint16_t data)
{
  strcpy(req->ifr_name, IFNAME);
  req->ifr_ifru.ifru_mii_data.reg_num = reg;
  req->ifr_ifru.ifru_mii_data.val_in = data;
}

static int read_mmd(int socket, uint8_t mmd, uint16_t address, uint16_t *data)
{
  struct ifreq req;
  prep_mmd(&req, mmd, address, 0);
  int retval = ioctl(socket, SIOCGMMDREG, (unsigned long)(&req));
  if (retval)
    {
      fprintf(stderr, "read_mmd: ioctl failed: %d, %d\n", retval, errno);
      return ERROR;
    }

  *data = req.ifr_ifru.ifru_mii_data.val_out;
  return OK;
}

static int write_mmd(int socket, uint8_t mmd, uint16_t address, uint16_t data)
{
  struct ifreq req;
  prep_mmd(&req, mmd, address, data);
  int retval = ioctl(socket, SIOCSMMDREG, (unsigned long)(&req));
  if (retval)
    {
      fprintf(stderr, "write_mmd: ioctl failed: %d, %d\n", retval, errno);
      return ERROR;
    }

  return OK;
}

static int read_mii(int socket, uint8_t reg, uint16_t *data)
{
  struct ifreq req;
  prep_mii(&req, reg, 0);
  int retval = ioctl(socket, SIOCSMIIREG, (unsigned long)(&req));
  if (retval)
    {
      fprintf(stderr, "read_mii: ioctl failed: %d, %d\n", retval, errno);
      return ERROR;
    }

  *data = req.ifr_ifru.ifru_mii_data.val_out;
  return OK;
}

static int write_mii(int socket, uint8_t reg, uint16_t data)
{
  struct ifreq req;
  prep_mii(&req, reg, data);
  int retval = ioctl(socket, SIOCSMIIREG, (unsigned long)(&req));
  if (retval)
    {
      fprintf(stderr, "write_mii: ioctl failed: %d, %d\n", retval, errno);
      return ERROR;
    }

  return OK;
}

int main(int argc, char *argv[])
{
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (argc < 2)
    {
      fprintf(stderr, "Too few arguments\n");
      return ERROR;
    }

  if (! strcmp(argv[1], "up"))
    {
      write_mii(sock, PHY_CONTROL_ADDR, 0);
    }

  if (! strcmp(argv[1], "down"))
    {
      write_mii(sock, PHY_CONTROL_ADDR, 1 << PHY_CONTROL_PD_POS);
    }

  if (! strcmp(argv[1], "mmdup"))
    {
      write_mmd(sock, PMA_MMD, PMA_CONTROL_ADDR, 0);
    }

  if (! strcmp(argv[1], "mmddown"))
    {
      write_mmd(sock, PMA_MMD, PMA_CONTROL_ADDR, 1 << PMA_CONTROL_LPE_POS);
    }

  return 0;
}
