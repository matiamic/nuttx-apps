#include <stdio.h>
#include <unistd.h>

#include <nuttx/spi/spi_transfer.h>
#include <nuttx/spi/spi.h>
#include <sys/ioctl.h>
#include <sys/fcntl.h>

#include "oa_tc6_control_transaction_protocol.h"

#define SPI_DRIVER_PATH "/dev/spi2"

int main(int argc, FAR char *argv[])
{
  int fd;
  uint32_t reg;

  fd = open(SPI_DRIVER_PATH, O_RDONLY);

  // test read
  if (oa_tc6_read_register(fd, 0x0, 0x0000, &reg)) printf("erorr: ");
  printf("SPI Identification Register, IDVER:  0x%08lx, MAJVER: %ld, MINVER: %ld\n",
         reg,
         (reg & (0xF << 4)) >> 4,
         reg & 0xF);
  if (oa_tc6_read_register(fd, 0x0, 0x0001, &reg)) printf("error: ");
  printf("SPI Identification Register, PHY ID: 0x%08lx, MODEL: 0x%02lx, REV: %ld\n", reg,
         (reg & (0x3F << 4)) >> 4,
         reg & 0xF);
  if (oa_tc6_read_register(fd, 12, 0x0012, &reg)) printf("error: ");
  printf("DIO Configuration Register: 0x%08lx\n", reg);

  // test write -- blink with dio0 and dio1 in counterphase
  for (int i = 0; i < 4; i++)
    {
      reg = 0x0302;
      if (oa_tc6_write_register(fd, 12, 0x0012, reg)) printf("error writing\n");
      usleep(250000);
      reg = 0x0203;
      if (oa_tc6_write_register(fd, 12, 0x0012, reg)) printf("error writing\n");
      usleep(250000);
    }
  reg = 0x6060;  // return the default value
  if (oa_tc6_write_register(fd, 12, 0x0012, reg)) printf("error writing\n");
  return 0;
}

