#include <nuttx/bits.h>
#include <stdint.h>

// lifted from linux kernel's bitfield.h
#define __bf_shf(x) (__builtin_ffsll(x) - 1)
#define FIELD_PREP(_mask, _val)                                 \
    ({                                                          \
     ((typeof(_mask))(_val) << __bf_shf(_mask)) & (_mask);      \
     })

#define CTP_DNC  BIT(31)
#define CTP_HDRB BIT(30)
#define CTP_WNR  BIT(29)
#define CTP_AID  BIT(28)
#define CTP_MMS  GENMASK(27, 24)
#define CTP_ADDR GENMASK(23, 8)
#define CTP_LEN  GENMASK(7, 1)
#define CTP_P    BIT(0)

int oa_tc6_read_register(int spi_fd, uint8_t mms, uint16_t addr, uint32_t *reg);
int oa_tc6_write_register(int spi_fd, uint8_t mms, uint16_t addr, uint32_t reg);
