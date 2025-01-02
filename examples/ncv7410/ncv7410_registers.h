#define IDVER_ADDR               0x0U
#define IDVER_MMS                0

#define PHY_ID_ADDR              0x1U
#define PHY_ID_MMS               0

#define SPICAP_ADDR              0x2U
#define SPICAP_MMS               0

#define RESET_ADDR               0x3U
#define RESET_MMS                0

#define CONFIG0_ADDR             0x4U
#define CONFIG0_MMS              0

#define STATUS0_ADDR             0x8U
#define STATUS0_MMS              0

#define BUFSTS_ADDR              0xBU
#define BUFSTS_MMS               0

#define IMASK_ADDR               0xCU
#define IMASK_MMS                0

#define PHY_CONTROL_REG_ADDR     0xFF00U
#define PHY_CONTROL_REG_MMS      0
#define PHY_CONTROL_LCTL_POS     12

#define PHY_STATUS_REG_ADDR      0xFF01U
#define PHY_STATUS_REG_MMS       0

#define PHY_IDENTIFIER0_REG_ADDR 0xFF02U
#define PHY_IDENTIFIER0_REG_MMS  0

#define PHY_IDENTIFIER1_REG_ADDR 0xFF03U
#define PHY_IDENTIFIER1_REG_MMS  0


#define MAC_CONTROL0_REG_ADDR    0x0U
#define MAC_CONTROL0_REG_MMS     1
#define MAC_CONTROL0_FCSA_POS    8
#define MAC_CONTROL0_TXEN_POS    1
#define MAC_CONTROL0_RXEN_POS    0


#define DIO_CONFIG_REG_ADDR      0x0012U
#define DIO_CONFIG_REG_MMS       12
#define DIO0_FUNC_POS            1
#define DIO1_FUNC_POS            9
#define DIO0_OUT_VAL_POS         0
#define DIO1_OUT_VAL_POS         8
#define DIO_TRISTATE_FUNC        0x0
#define DIO_GPIO_FUNC            0x1
#define DIO_SFD_TX_FUNC          0x2
#define DIO_SFD_RX_FUNC          0x3
#define DIO_LINK_CTRL_FUNC       0x4
#define DIO_SFD_TXRX_FUNC        0xB
#define DIO_TXRX_FUNC            0xF
