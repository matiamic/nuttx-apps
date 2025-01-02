#include <stdint.h>

#include <nuttx/spi/spi_transfer.h>
#include <nuttx/spi/spi.h>
#include <sys/ioctl.h>
#include <sys/fcntl.h>

#include <sys/endian.h>
#include "oa_tc6_control_transaction_protocol.h"

#define OA_TC6_SPI_MODE 0

static int get_parity(uint32_t p)
{
  // 1 if odd parity, zero otherwise
  return __builtin_popcount(p) % 2;
}

static void prep_spi_seq32(struct spi_sequence_s *seq,
                           struct spi_trans_s *trans,
                           uint32_t *txdata,
                           uint32_t *rxdata,
                           int len)
{
  // prepare transaction
  seq->dev = SPIDEV_ID(SPIDEVTYPE_USER, 0);
  seq->mode = OA_TC6_SPI_MODE;
  seq->nbits = 8;
  seq->frequency = 20000000;
  seq->ntrans = 1;
  seq->trans = trans;

  // must be true in between control - data / data - control
  trans->deselect = false;
  trans->delay = 0;
  trans->nwords = 4 * len;
  trans->txbuffer = (uint8_t *) txdata;
  trans->rxbuffer = (uint8_t *) rxdata;
}

int oa_tc6_read_register(int spi_fd, uint8_t mms, uint16_t addr, uint32_t *reg)
{
  struct spi_trans_s trans;
  struct spi_sequence_s seq;

  uint32_t txdata[3] = { 0 };
  uint32_t rxdata[3] = { 0 };

  // prepare header
  uint32_t header = \
        FIELD_PREP(CTP_MMS, mms)
      | FIELD_PREP(CTP_ADDR, addr);

  int parity = get_parity(header);
  header |= parity ? 0 : CTP_P;
  header = htobe32(header);

  // prepare transaction
  txdata[0] = header;
  prep_spi_seq32(&seq, &trans, txdata, rxdata, 3);
  ioctl(spi_fd, SPIIOC_TRANSFER, (unsigned long)((uintptr_t)&seq));
  *reg = be32toh(rxdata[2]);
  if (rxdata[1] != header) return 1;  // error
  return 0;
}

int oa_tc6_write_register(int spi_fd, uint8_t mms, uint16_t addr, uint32_t reg)
{
  struct spi_trans_s trans;
  struct spi_sequence_s seq;

  uint32_t txdata[3] = { 0 };
  uint32_t rxdata[3] = { 0 };

  // prepare header
  uint32_t header = \
        CTP_WNR
      | FIELD_PREP(CTP_MMS, mms)
      | FIELD_PREP(CTP_ADDR, addr);

  int parity = get_parity(header);
  header |= parity ? 0 : CTP_P;

  header = htobe32(header);
  reg = htobe32(reg);

  // prepare transaction
  txdata[0] = header;
  txdata[1] = reg;
  prep_spi_seq32(&seq, &trans, txdata, rxdata, 3);
  ioctl(spi_fd, SPIIOC_TRANSFER, (unsigned long)((uintptr_t)&seq));
  if (rxdata[1] != header) return 1;  // error
  return 0;
}

/* int oa_tc6_read_registers(int spi_fd, uint8_t mms, uint16_t addr, uint8_t len, uint32_t *reg_buffer) */
/* { */
/*   // prepare header */
/*   uint32_t header = \ */
/*         FIELD_PREP(CTP_MMS, mms) */
/*       | FIELD_PREP(CTP_ADDR, addr) */
/*       | FIELD_PREP(CTP_LEN, len - 1); */

/*   int parity = get_parity(header); */
/*   header |= parity ? 0 : CTP_P; */
/*   header = htobe32(header); */

/*   // prepare transaction */
/*   struct spi_trans_s trans; */
/*   struct spi_sequence_s seq; */

/*   uint8_t txdata[4]; */
/*   uint8_t *rxdata = (uint8_t) regbuffer; */

/*   seq.dev = SPIDEV_ID(SPIDEVTYPE_USER, 0); */
/*   seq.mode = NCV7410_SPI_MODE; */
/*   seq.nbits = 8; */
/*   seq.frequency = 20000000; */
/*   seq.ntrans = 1; */
/*   seq.trans = &trans; */

/*   // must be true in between control - data / data - control */
/*   trans.deselect = true; */
/*   trans.delay = 0; */
/*   trans.nwords = 2; */
/*   trans.txbuffer = txdata; */
/*   trans.rxbuffer = rxdata; */
/*   return 0; */
/* } */
