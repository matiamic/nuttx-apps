#include <stdio.h>

#include <nuttx/spi/spi_transfer.h>
#include <nuttx/spi/spi.h>
#include <sys/ioctl.h>
#include <sys/fcntl.h>

#include <unistd.h>

#define SPI_DRIVER_PATH "/dev/spi2"
#define BUFFER_SIZE 10
#define NCV7410_SPI_MODE 0

static int ncv7410_spi_transfer(int fd, FAR struct spi_sequence_s *seq)
{
  /* Perform the IOCTL */

  return ioctl(fd, SPIIOC_TRANSFER, (unsigned long)((uintptr_t)seq));
}

int main(int argc, FAR char *argv[])
{
  struct spi_trans_s trans;
  struct spi_sequence_s seq;
  int fd;

  uint8_t txdata[BUFFER_SIZE] = { 0xAA, 0x01 };
  uint8_t rxdata[BUFFER_SIZE] = { 0 };

  fd = open(SPI_DRIVER_PATH, O_RDONLY);

  seq.dev = SPIDEV_ID(SPIDEVTYPE_USER, 0);
  seq.mode = NCV7410_SPI_MODE;
  seq.nbits = 8;
  seq.frequency = 20000000;
  seq.ntrans = 1;
  seq.trans = &trans;

  // must be true in between control - data / data - control
  trans.deselect = true;
  trans.delay = 0;
  trans.nwords = 2;
  trans.txbuffer = txdata;
  trans.rxbuffer = rxdata;

  while (1)
    {
      seq.mode = 0; // 0 - 3
      ncv7410_spi_transfer(fd, &seq);
      seq.mode = 1;
      ncv7410_spi_transfer(fd, &seq);
      seq.mode = 2;
      ncv7410_spi_transfer(fd, &seq);
      seq.mode = 3;
      ncv7410_spi_transfer(fd, &seq);
      usleep(250000);
    }

  printf("hello");
  return 0;
}

