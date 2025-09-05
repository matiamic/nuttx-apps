#include <nuttx/bits.h>

#define OA_TC14_PLCA_MMD          31

#define OA_TC14_IDVER_ADDR        0xCA00
#define OA_TC14_IDVER_IDM_MASK    GENMASK(15, 8)
#define OA_TC14_IDVER_IDM_POS     8
#define OA_TC14_IDVER_VER_MASK    GENMASK(7, 0)
#define OA_TC14_IDVER_VER_POS     0
#define OA_TC14_IDVER_VAL         0x0A10

#define OA_TC14_CTRL0_ADDR        0xCA01
#define OA_TC14_CTRL0_EN_MASK     BIT(15)
#define OA_TC14_CTRL0_EN_POS      15
#define OA_TC14_CTRL0_RST_MASK    BIT(14)
#define OA_TC14_CTRL0_RST_POS     14

#define OA_TC14_CTRL1_ADDR        0xCA02
#define OA_TC14_CTRL1_NCNT_MASK   GENMASK(15, 8)
#define OA_TC14_CTRL1_NCNT_POS    8
#define OA_TC14_CTRL1_ID_MASK     GENMASK(7, 0)
#define OA_TC14_CTRL1_ID_POS      0

#define OA_TC14_STATUS_ADDR       0xCA03
#define OA_TC14_STATUS_PST_MASK   BIT(15)
#define OA_TC14_STATUS_PST_POS    15

#define OA_TC14_TOTMR_ADDR        0xCA04
#define OA_TC14_TOTMR_TOT_MASK    GENMASK(7, 0)
#define OA_TC14_TOTMR_TOT_POS     0

#define OA_TC14_BURST_ADDR        0xCA05
#define OA_TC14_BURST_MAXBC_MASK  GENMASK(15, 8)
#define OA_TC14_BURST_MAXBC_POS   8
#define OA_TC14_BURST_BTMR_MASK   GENMASK(7, 0)
#define OA_TC14_BURST_BTMR_POS    0

#define OA_TC14_DIAG_ADDR         0xCA06
#define OA_TC14_DIAG_RXINTO_MASK  BIT(2)
#define OA_TC14_DIAG_RXINTO_POS   2
#define OA_TC14_DIAG_UNEXPB_MASK  BIT(1)
#define OA_TC14_DIAG_UNEXPB_POS   1
#define OA_TC14_DIAG_BCNBFTO_MASK BIT(0)
#define OA_TC14_DIAG_BCNBFTO_POS  0

#define oa_tc14_get_field(r, fieldname) \
    (((r) & OA_TC14_##fieldname##_MASK) >> OA_TC14_##fieldname##_POS)

#define oa_tc14_field(val, fieldname) \
    ((val << OA_TC14_##fieldname##_POS) & OA_TC14_##fieldname##_MASK)
