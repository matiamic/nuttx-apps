#include <stdio.h>
#include <unistd.h>

#include <nuttx/spi/spi_transfer.h>
#include <nuttx/spi/spi.h>
#include <sys/ioctl.h>
#include <sys/fcntl.h>

#include "oa_tc6_control_transaction_protocol.h"

#include "ncv7410_registers.h"

#define SPI_DRIVER_PATH "/dev/spi2"

#define N_TRIES 4  // arbitrary

static int fd;

static void init(void)
{
  uint32_t regval;
  int tries;

  fd = open(SPI_DRIVER_PATH, O_RDONLY);

  // reset
  regval = 0x00000001;
  if (oa_tc6_write_register(fd, RESET_MMS, RESET_ADDR, regval)) printf("error writing\n");

  tries = N_TRIES;
  do
    {
      if (oa_tc6_read_register(fd, RESET_MMS, RESET_ADDR, &regval)) printf("erorr: ");
    }
  while (tries-- && (regval & 1));
  if (regval & 1)
    {
      printf("reset unsuccessful\n");
      return;
    }

  // blink with dio0 and dio1 in counterphase
  for (int i = 0; i < 4; i++)
    {
      regval = 0x0302;
      if (oa_tc6_write_register(fd, DIO_CONFIG_REG_MMS, DIO_CONFIG_REG_ADDR, regval)) printf("error writing\n");
      usleep(250000);
      regval = 0x0203;
      if (oa_tc6_write_register(fd, DIO_CONFIG_REG_MMS, DIO_CONFIG_REG_ADDR, regval)) printf("error writing\n");
      usleep(250000);
    }

  // setup LEDs
  regval =   (DIO_TXRX_FUNC << DIO0_FUNC_POS)   | (1 << DIO0_OUT_VAL_POS) \
           | (DIO_SFD_TXRX_FUNC << DIO1_FUNC_POS) | (1 << DIO1_OUT_VAL_POS);
           /* | (DIO_LINK_CTRL_FUNC << DIO1_FUNC_POS) | (1 << DIO1_OUT_VAL_POS); */
  if (oa_tc6_write_register(fd, DIO_CONFIG_REG_MMS, DIO_CONFIG_REG_ADDR, regval)) printf("error writing\n");

  // setup and enable MAC
  regval = (1 << MAC_CONTROL0_FCSA_POS) | (1 << MAC_CONTROL0_TXEN_POS) | (1 << MAC_CONTROL0_RXEN_POS);
  if (oa_tc6_write_register(fd, MAC_CONTROL0_REG_MMS, MAC_CONTROL0_REG_ADDR, regval)) printf("error writing\n");

  // enable PHY
  regval = (1 << PHY_CONTROL_LCTL_POS);
  if (oa_tc6_write_register(fd, PHY_CONTROL_REG_MMS, PHY_CONTROL_REG_ADDR, regval)) printf("error writing\n");


  // setup SPI protocol and enable (see page 63 of datasheet)
  regval = 0x0000BC06;
  if (oa_tc6_write_register(fd, CONFIG0_MMS, CONFIG0_ADDR, regval)) printf("error writing\n");
  return;
}

int main(int argc, FAR char *argv[])
{
  uint32_t reg;
  init();

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

  while (1)
    {
      char cmd = getchar();
      if (cmd == 'q') break;
      if (oa_tc6_read_register(fd, STATUS0_MMS, STATUS0_ADDR, &reg)) printf("error: ");
      printf("STATUS0 register:       0x%08lx\n", reg);
      if (oa_tc6_read_register(fd, CONFIG0_MMS, CONFIG0_ADDR, &reg)) printf("error: ");
      printf("CONFIG0 register:       0x%08lx\n", reg);
      if (oa_tc6_read_register(fd, BUFSTS_MMS, BUFSTS_ADDR, &reg)) printf("error: ");
      printf("BUFFER_STATUS register: 0x%08lx\n", reg);
      if (oa_tc6_read_register(fd, PHY_CONTROL_REG_MMS, PHY_CONTROL_REG_ADDR, &reg)) printf("error: ");
      printf("PHY CONTROL register:   0x%08lx\n", reg);
      if (oa_tc6_read_register(fd, PHY_STATUS_REG_MMS, PHY_STATUS_REG_ADDR, &reg)) printf("error: ");
      printf("PHY STATUS register:    0x%08lx\n", reg);
    }
  return 0;
}

