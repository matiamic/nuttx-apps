#include <stdio.h>
#include <unistd.h>

#include <nuttx/spi/spi_transfer.h>
#include <nuttx/spi/spi.h>
#include <sys/ioctl.h>
#include <sys/fcntl.h>

#include <sys/endian.h>

#include "ncv7410.h"

#define SPI_DRIVER_PATH "/dev/spi2"

#define N_TRIES 4  // arbitrary

static int get_parity(uint32_t w)
{
  /* www-graphics.stanford.edu/~seander/bithacks.html */
  w ^= w >> 1;
  w ^= w >> 2;
  w = (w & 0x11111111U) * 0x11111111U;
  return (w >> 28) & 1;
}

static void prep_spi_seq(struct spi_sequence_s *seq,
                         struct spi_trans_s *trans,
                         uint8_t *txdata,
                         uint8_t *rxdata,
                         int len)
{
  // prepare transaction
  seq->dev = SPIDEV_ID(SPIDEVTYPE_USER, 0);
  seq->mode = OA_TC6_SPI_MODE;
  seq->nbits = 8;
  seq->frequency = SPI_FREQ;
  seq->ntrans = 1;
  seq->trans = trans;

  // must be true in between control - data / data - control
  trans->deselect = false;
  trans->delay = 0;
  trans->nwords = len;
  trans->txbuffer = txdata;
  trans->rxbuffer = rxdata;
}

static void prep_spi_seq32(struct spi_sequence_s *seq,
                           struct spi_trans_s *trans,
                           uint32_t *txdata,
                           uint32_t *rxdata,
                           int len)
{
  prep_spi_seq(seq, trans, (uint8_t *) txdata, (uint8_t *) rxdata, len * 4);
}

int read_reg(int spifd, uint8_t mms, uint16_t addr, uint32_t *reg)
{
  struct spi_trans_s trans;
  struct spi_sequence_s seq;

  uint32_t txdata[3] = { 0 };
  uint32_t rxdata[3] = { 0 };

  // prepare header
  uint32_t header =   (mms << CTP_MMS_POS)
                    | (addr << CTP_ADDR_POS);

  int parity = get_parity(header);
  header |= parity ? 0 : CTP_P_MASK;
  header = htobe32(header);

  // prepare transaction
  txdata[0] = header;
  prep_spi_seq32(&seq, &trans, txdata, rxdata, 3);
  ioctl(spifd, SPIIOC_TRANSFER, (unsigned long)((uintptr_t)&seq));
  *reg = be32toh(rxdata[2]);
  if (rxdata[1] != header) return 1;  // error
  return 0;
}

int write_reg(int spifd, uint8_t mms, uint16_t addr, uint32_t reg)
{
  struct spi_trans_s trans;
  struct spi_sequence_s seq;

  uint32_t txdata[3] = { 0 };
  uint32_t rxdata[3] = { 0 };

  // prepare header
  uint32_t header =   (1 << CTP_WNR_POS)
                    | (mms << CTP_MMS_POS)
                    | (addr << CTP_ADDR_POS);

  int parity = get_parity(header);
  header |= parity ? 0 : CTP_P_MASK;

  header = htobe32(header);
  reg = htobe32(reg);

  // prepare transaction
  txdata[0] = header;
  txdata[1] = reg;
  prep_spi_seq32(&seq, &trans, txdata, rxdata, 3);
  ioctl(spifd, SPIIOC_TRANSFER, (unsigned long)((uintptr_t)&seq));
  if (rxdata[1] != header) return 1;  // error
  return 0;
}

uint32_t poll_footer(int spifd, struct ncv7410_state *s)
{
  uint32_t header;
  uint32_t footer;
  uint8_t txbuf[CHUNK_DEFAULT_SIZE];
  uint8_t rxbuf[CHUNK_DEFAULT_SIZE];
  struct spi_trans_s trans;
  struct spi_sequence_s seq;

  header =   (1 << DTPH_DNC_POS)
           | (1 << DTPH_NORX_POS)
           | (0 << DTPH_DV_POS)
           | (0 << DTPH_SV_POS);

  header |= (!get_parity(header) << DTPH_P_POS);
  header = htobe32(header);
  *((uint32_t *) txbuf) = header;

  prep_spi_seq(&seq, &trans, txbuf, rxbuf, CHUNK_DEFAULT_SIZE);
  ioctl(spifd, SPIIOC_TRANSFER, (unsigned long)((uintptr_t)&seq));

  // CHUNK_DEFAULT_PAYLOAD_SIZE is the actual index of the footer
  footer = *((uint32_t *) (&rxbuf[CHUNK_DEFAULT_PAYLOAD_SIZE]));
  footer = be32toh(footer);
  s->txc  = (footer & DTPF_TXC_MASK)  >> DTPF_TXC_POS;
  s->rca  = (footer & DTPF_RCA_MASK)  >> DTPF_RCA_POS;
  s->hdrb = (footer & DTPF_HDRB_MASK) >> DTPF_HDRB_POS;
  s->exst = (footer & DTPF_EXST_MASK) >> DTPF_EXST_POS;
  s->sync = (footer & DTPF_SYNC_MASK) >> DTPF_SYNC_POS;
  return footer;
}

uint32_t read_chunk(int spifd, struct ncv7410_state *s, uint8_t *rxbuf)
{
  uint32_t header;
  uint32_t footer;
  uint8_t txbuf[CHUNK_DEFAULT_SIZE];
  struct spi_trans_s trans;
  struct spi_sequence_s seq;

  header =   (1 << DTPH_DNC_POS)
           | (0 << DTPH_NORX_POS)
           | (0 << DTPH_DV_POS)
           | (0 << DTPH_SV_POS);

  header |= (!get_parity(header) << DTPH_P_POS);
  header = htobe32(header);
  *((uint32_t *) txbuf) = header;
  prep_spi_seq(&seq, &trans, txbuf, rxbuf, CHUNK_DEFAULT_SIZE);
  ioctl(spifd, SPIIOC_TRANSFER, (unsigned long)((uintptr_t)&seq));

  // CHUNK_DEFAULT_PAYLOAD_SIZE is the actual index of the footer
  footer = *((uint32_t *) (&rxbuf[CHUNK_DEFAULT_PAYLOAD_SIZE]));
  footer = be32toh(footer);
  if (get_parity(footer))
    {
      s->txc  = (footer & DTPF_TXC_MASK)  >> DTPF_TXC_POS;
      s->rca  = (footer & DTPF_RCA_MASK)  >> DTPF_RCA_POS;
      s->hdrb = (footer & DTPF_HDRB_MASK) >> DTPF_HDRB_POS;
      s->exst = (footer & DTPF_EXST_MASK) >> DTPF_EXST_POS;
      s->sync = (footer & DTPF_SYNC_MASK) >> DTPF_SYNC_POS;
    }
  return footer;
}

void write_chunk(void)
{

}

static void reset(int fd)
{
  // reset
  uint32_t regval = (1 << RESET_SWRESET_POS);
  if (write_reg(fd, RESET_MMS, RESET_ADDR, regval)) printf("error writing\n");
}

static void init(int *fd)
{
  uint32_t regval;
  int tries;
  int spifd;

  spifd = open(SPI_DRIVER_PATH, O_RDONLY);

  // reset
  regval = (1 << RESET_SWRESET_POS);
  if (write_reg(spifd, RESET_MMS, RESET_ADDR, regval)) printf("error writing\n");

  tries = N_TRIES;
  do
    {
      if (read_reg(spifd, RESET_MMS, RESET_ADDR, &regval)) printf("erorr: ");
    }
  while (tries-- && (regval & 1));
  if (regval & 1)
    {
      printf("reset unsuccessful\n");
      *fd = -1;
      return;
    }

  // for some reason HDRB is sometimes set right after restart -> clear it
  if (write_reg(spifd, STATUS0_MMS, STATUS0_ADDR, (1 << STATUS0_HDRE_POS)))
    {
      printf("error resetting HDRBE in STATUS0\n");
    }

  // blink with dio0 and dio1 in counterphase
  for (int i = 0; i < 4; i++)
    {
      regval = 0x0302;
      if (write_reg(spifd, DIO_CONFIG_REG_MMS, DIO_CONFIG_REG_ADDR, regval)) printf("error writing\n");
      usleep(250000);
      regval = 0x0203;
      if (write_reg(spifd, DIO_CONFIG_REG_MMS, DIO_CONFIG_REG_ADDR, regval)) printf("error writing\n");
      usleep(250000);
    }

  // setup LEDs
  regval =   (DIO_TXRX_FUNC << DIO0_FUNC_POS)   | (1 << DIO0_OUT_VAL_POS) \
           | (DIO_SFD_TXRX_FUNC << DIO1_FUNC_POS) | (1 << DIO1_OUT_VAL_POS);
           /* | (DIO_LINK_CTRL_FUNC << DIO1_FUNC_POS) | (1 << DIO1_OUT_VAL_POS); */
  if (write_reg(spifd, DIO_CONFIG_REG_MMS, DIO_CONFIG_REG_ADDR, regval)) printf("error writing\n");

  // setup and enable MAC
  regval = (1 << MAC_CONTROL0_FCSA_POS) | (1 << MAC_CONTROL0_TXEN_POS) | (1 << MAC_CONTROL0_RXEN_POS);
  if (write_reg(spifd, MAC_CONTROL0_REG_MMS, MAC_CONTROL0_REG_ADDR, regval)) printf("error writing\n");

  // enable PHY
  regval = (1 << PHY_CONTROL_LCTL_POS);
  if (write_reg(spifd, PHY_CONTROL_REG_MMS, PHY_CONTROL_REG_ADDR, regval)) printf("error writing\n");


  // setup SPI protocol and enable (see page 63 of datasheet)
  regval = 0x0000BC06;
  regval &= ~(CONFIG0_CPS_MASK);     // clear Chunk Payload Size
  regval |= (5 << CONFIG0_CPS_POS);  // set Chunk Payload Size to 2^5 = 32
  if (write_reg(spifd, CONFIG0_MMS, CONFIG0_ADDR, regval)) printf("error writing\n");
  *fd = spifd;
  return;
}

void print_frame(uint8_t *frame, int len, int frame_num)
{
#define HEX_LINE_LEN 64
#define ASCII_LINE_LEN (3 * HEX_LINE_LEN)
  printf("Frame %04d (%d B) hex:\n", frame_num, len);
  for (int i = 0; i < (len + HEX_LINE_LEN - 1) / HEX_LINE_LEN; i++)
    {
      for (int j = 0; j < HEX_LINE_LEN; j++)
        {
          int idx = HEX_LINE_LEN * i + j;
          if (idx >= len) break;
          printf("%02x ", frame[idx]);
        }
      printf("\n");
    }
  printf("\n");

  printf("Frame %04d (%d B) ASCII:\n", frame_num, len);
  for (int i = 0; i < (len + ASCII_LINE_LEN - 1) / ASCII_LINE_LEN; i++)
    {
      for (int j = 0; j < ASCII_LINE_LEN; j++)
        {
          int idx = ASCII_LINE_LEN * i + j;
          if (idx >= len) break;
          if (32 <= frame[idx] && frame[idx] <= 126)  // printable range
            {
              printf("%c", frame[idx]);
            }
          else
            {
              printf(".");
            }
        }
      printf("\n");
    }
  printf("\n\n");
}

int main(int argc, FAR char *argv[])
{
  int fd;
  uint32_t reg;
  struct ncv7410_state s = { 0 };
  char cmd = '\0';
  init(&fd);

  // test read
  if (read_reg(fd, IDVER_MMS, IDVER_ADDR, &reg)) printf("erorr: ");
  printf("SPI Identification Register, IDVER:  0x%08lx, MAJVER: %ld, MINVER: %ld\n",
         reg,
         (reg & (0xF << 4)) >> 4,
         reg & 0xF);
  if (read_reg(fd, PHY_ID_MMS, PHY_ID_ADDR, &reg)) printf("error: ");
  printf("SPI Identification Register, PHY ID: 0x%08lx, MODEL: 0x%02lx, REV: %ld\n", reg,
         (reg & (0x3F << 4)) >> 4,
         reg & 0xF);

  while (1)
    {
      if (read_reg(fd, STATUS0_MMS, STATUS0_ADDR, &reg)) printf("error: ");
      printf("STATUS0 register:       0x%08lx\n", reg);
      if (read_reg(fd, CONFIG0_MMS, CONFIG0_ADDR, &reg)) printf("error: ");
      printf("CONFIG0 register:       0x%08lx\n", reg);
      if (read_reg(fd, BUFSTS_MMS, BUFSTS_ADDR, &reg)) printf("error: ");
      printf("BUFFER_STATUS register: 0x%08lx\n", reg);
      if (read_reg(fd, PHY_CONTROL_REG_MMS, PHY_CONTROL_REG_ADDR, &reg)) printf("error: ");
      printf("PHY CONTROL register:   0x%08lx\n", reg);
      if (read_reg(fd, PHY_STATUS_REG_MMS, PHY_STATUS_REG_ADDR, &reg)) printf("error: ");
      printf("PHY STATUS register:    0x%08lx\n", reg);
      poll_footer(fd, &s);
      printf("Chunk available to read: %d\nChunks available to write: %d\n", s.rca, s.txc);
      cmd = getchar();
      if (cmd == 'q') break;
      if (cmd == 'r') reset(fd);
    }
#define MAX_FRAME_SIZE 1024
  uint8_t rxbuf[CHUNK_DEFAULT_SIZE];  // provide whole (+4) buffer for now
  uint8_t frame[MAX_FRAME_SIZE];
  int frame_idx = 0;
  int in_frame = 0;
  int size_in_chunk = 0;
  int frame_num = 0;
  int end = 0;
  uint32_t f;
  while (1)
    {
      f = poll_footer(fd, &s);
      if (!get_parity(f))
        {
          printf("poll footer: footer has bad parity\n");
          return 1;
        }
      if (f & DTPF_HDRB_MASK)
        {
          printf("HDRB flag in footer\n");
          return 1;
        }

      while (s.rca)
        {
          f = read_chunk(fd, &s, rxbuf);
          if (!get_parity(f))
            {
              printf("read chunk: footer has bad parity\n");
              return 1;
            }
          if (f & DTPF_FD_MASK)  // frame drop
            {
              frame_idx = 0;
              in_frame = 0;
              break;
            }
          if (!(f & DTPF_DV_MASK)) // not valid data
            {
              continue;
            }


          if (f & DTPF_SV_MASK)  // start valid
            {
              if (! in_frame)
                {
                  in_frame = 1;
                }
              else
                {  // drop the frame start came before end
                  frame_idx = 0;
                  in_frame = 0;
                  break;
                }
            }
          if (f & DTPF_EV_MASK)  // end valid
            {
              if (in_frame)  // ok -> finish the frame
                {
                  size_in_chunk = ((f & DTPF_EBO_MASK) >> DTPF_EBO_POS) + 1;
                  end = 1;
                }
              else // drop
                {
                  continue;
                }
            }
          else
            {
              size_in_chunk = CHUNK_DEFAULT_PAYLOAD_SIZE;
            }

          // right here I have valid data and initialized size_in_chunk
          if (! in_frame)  // drop
            {
              continue;
            }

          // right here I have valid data and initialized size_in_chunk and I am in frame
          for (int i = 0; i < size_in_chunk; i++)
            {
              frame[frame_idx++] = rxbuf[i];
            }
          if (end)
            {
              print_frame(frame, frame_idx, frame_num);

              end = 0;
              frame_idx = 0;
              in_frame = 0;
              frame_num++;
            }
        }
    }
  close(fd);
  return 0;
}
