/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright (c) Nuvoton Technology Corp. All rights reserved.                                             */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

#ifndef __SPI_H__
#define __SPI_H__

/*------------------------- SPI Interface Controller -------------------------*/
// CNTRL(Control and Status Register)
#define SPI_CNTRL_GO_BUSY		BIT0					// Go and Busy Status
#define SPI_CNTRL_RX_NEG		BIT1					// Receive On Negative Edge
#define SPI_CNTRL_TX_NEG		BIT2					// Transmit On Negative Edge
#define SPI_CNTRL_TX_BIT_LEN	BIT(7,3)				// Transmit Bit Length
#define SPI_CNTRL_TX_NUM		BIT(9,8)				// Transmit/Receive Numbers
#define SPI_CNTRL_LSB			BIT10					// Send LSB First
#define SPI_CNTRL_CLKP			BIT11					// Clock Polarity
#define SPI_CNTRL_SLEEP			BIT(15,12)				// Suspend Interval
#define SPI_CNTRL_IF			BIT16					// Interrupt Flag for 072 name is SPIIF
#define SPI_CNTRL_IE			BIT17					// Interrupt Enable for 072 name is SPIIE
#define SPI_CNTRL_SLAVE			BIT18					// Master/Slave Mode Select
#define SPI_CNTRL_BYTE_ENDIAN	BIT20					// BYTE ENDIAN

// DIVIDER(Clock Divider Register)
#define SPI_DIVIDER_DIVIDER		BIT(15,0)				// Clock Divider
// SSR(Slave Select Register)
#define SPI_SSR_LTRIG_FLAG				BIT5					// Level Trigger Flag
#define SPI_SSR_SS_LTRIG				BIT4					// Slave Select Level Trigger
#define SPI_SSR_ASS				BIT3					// Automatic Slave Select
#define SPI_SSR_SS_LVL			BIT2					// Slave Select Active Level
#define SPI_SSR_SSPC			BIT(1,0)				// Slave Select Pin Control

// SPI0_RCLK(SPI0 Receive Timing Control Register)
#define SPI_SPI0_RCLK_SPI0_CTIM				BIT(3,2)				// Coarse Timing Control for SPI0 Data Receiving
#define SPI_SPI0_RCLK_SPI0_FTIM				BIT(1,0)					// Fine Timing Control for SPI0 Data Receiving

// Rx(Data Receive Register 0, 1)
// Tx(Data Transmit Register 0,1)

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
	    __IO uint32_t  GO_BUSY:1;						// Go and Busy Status
	    __IO uint32_t  RX_NEG:1;						// Receive On Negative Edge
	    __IO uint32_t  TX_NEG:1;						// Transmit On Negative Edge
	    __IO uint32_t  TX_BIT_LEN:5;					// Transmit Bit Length
	    __IO uint32_t  TX_NUM:2;						// Transmit/Receive Numbers
	    __IO uint32_t  LSB:1;							// Send LSB First
	    __IO uint32_t  CLKP:1;							// Clock Polarity
	    __IO uint32_t  SLEEP:4;							// Suspend Interval
	    __IO uint32_t  IF:1;							// Interrupt Flag for 072 name is SPIIF
	    __IO uint32_t  IE:1;							// Interrupt Enable for 072 name is SPIIE
	    __IO uint32_t  SLAVE:1;							// Master/Slave Mode Select
	    __I  uint32_t  RESERVE0:1;
	    __IO uint32_t  BYTE_ENDIAN:1;					// BYTE ENDIAN
    };
} SPI_CNTRL_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
	    __IO uint32_t  DIVIDER:16;						// Clock Divider
    };
} SPI_DIVIDER_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
	    __IO uint32_t  SSPC:2;							// Slave Select Pin Control
	    __IO uint32_t  SS_LVL:1;						// Slave Select Active Level
	    __IO uint32_t  ASS:1;							// Automatic Slave Select
	    __IO uint32_t  SS_LTRIG:1;							// Slave Select Level Trigger
	    __I uint32_t  LTRIG_FLAG:1;							// Level Trigger Flag
    };
} SPI_SSR_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
	    __IO uint32_t  SPI0_FTIM:2;							// Fine Timing Control for SPI0 Data Receiving
	    __IO uint32_t  SPI0_CTIM:2;						// Coarse Timing Control for SPI0 Data Receiving
    };
} SPI_SPI0_RCLK_T;

typedef __I  uint32_t   SPI_RX_T;
typedef __O  uint32_t   SPI_TX_T;

typedef struct
{
    SPI_CNTRL_T     CNTRL;								// Control and Status Register
    SPI_DIVIDER_T   DIVIDER;							// Clock Divider Register
    SPI_SSR_T       SSR;								// Slave Select Register
    uint32_t        RESERVE0;
    SPI_RX_T        RX[2];								// Rx(Data Receive Register 0, 1)
    uint32_t        RESERVE1[2];
    SPI_TX_T        TX[2];								// Tx(Data Transmit Register 0,1)
    uint32_t        RESERVE2[2];
    SPI_SPI0_RCLK_T	SPI0_RCLK;					//SPI0 Receive TimingControl Register
} SPI_T;

#endif
