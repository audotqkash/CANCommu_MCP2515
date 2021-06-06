/**
 * @file    mcp2515_defs.h
 * @brief   Resister Address and Function Bit definition for MCP2515
 * @author  audotqkash (github.com/audotqkash)
 * @date    May 31th, 2021.
 * $Version 0.0 
 * @par     May 31th, 2021 : MCP2515 Register Address, Operation Mode, Instruction 
 * 			June 6th, 2021 : Registor Mask and functional bit offset for CTRL,STAT and CNF
 * 			June 6th, 2021 : Receive CAN Data
 *
 */

#ifndef __MCP2515_DEFS_H_
#define __MCP2515_DEFS_H_

/* Digital Pin */
#define MCP2515_PIN_RX0BUF 11
#define MCP2515_PIN_RX1BUF 10
#define MCP2515_PIN_INT    12

/* CAN Control */
#define MCP2515_REG_CANCTRL	0x0F
#define MCP2515_MSK_REQOP   0b11100000
#define MCP2515_OFST_REQOP  5
#define MCP2515_MSK_ABAT    0b00010000  /*abort (送信全停止)*/
#define MCP2515_OFST_ABAT   4
#define MCP2515_MSK_OSM     0b00001000  /*ﾜﾝｼｮｯﾄﾓｰﾄﾞ*/
#define MCP2515_OFST_OSM    3
#define MCP2515_MSK_CLKEN   0b00000100  /* CLKOUTピン有効*/
#define MCP2515_OFST_CLKEN  2
#define MCP2515_MSK_CLKPRE  0b00000011  /* CLKOUTピン分周設定*/
#define MCP2515_OFST_CLKPRE 0

/* CAN Status */
#define MCP2515_REG_CANSTAT	0x0E
#define MCP2515_MSK_OPMOD   0b11100000
#define MCP2515_OFST_OPMOD  5
#define MCP2515_MSK_ICOD    0b00001110
#define MCP2515_OFST_ICOD   1

/* MCP2515 Configuration #1 */
#define MCP2515_REG_CNF1	0x2A
#define MCP2515_MSK_BRP     0b00111111
#define MCP2515_OFST_BRP    0
#define MCP2515_MSK_SJW     0b11000000
#define MCP2515_OFST_SJW    6

/* MCP2515 Configuration #2 */
#define MCP2515_REG_CNF2	0x29
#define MCP2515_MSK_PRSEG   0b00000111
#define MCP2515_OFST_PRSEG  0
#define MCP2515_MSK_PS1     0b00111000
#define MCP2515_OFST_PS1    3
#define MCP2515_MSK_SAM     0b01000000
#define MCP2515_OFST_SAM    6
#define MCP2515_MSK_BTLMD   0b10000000
#define MCP2515_OFST_BTLMD  7

/* MCP2515 Configuration #3 */
#define MCP2515_REG_CNF3	0x28
#define MCP2515_MSK_PS2     0b00000111
#define MCP2515_OFST_PS2    0
#define MCP2515_MSK_WAKFIL  0b01000000
#define MCP2515_OFST_WAKFIL 5
#define MCP2515_MSK_SOF     0b10000000
#define MCP2515_OFST_SOF    7

/* MCP2515 Run Mode */
#define MCP2515_MD_NORMAL	0b000
#define MCP2515_MD_SLEEP	0b001
#define MCP2515_MD_LOOPBACK	0b010
#define MCP2515_MD_LISTEN	0b011
#define MCP2515_MD_CONFIG	0b100

/* MCP2515 Quick Command */
#define MCP2515_CMD_RESET	0xC0
#define MCP2515_CMD_READ	0x03
#define MCP2515_CMD_WRITE	0x20
#define MCP2515_CMD_BITMOD	0x05
#define MCP2515_CMD_RDRXBUF	0x90 /* ref: DS20001801J FIGURE 12-3  */
#define MCP2515_CMD_LDTXBUF	0x40 /* ref: DS20001801J FIGURE 12-5  */
#define MCP2515_CMD_RTS 	0x80 /* 0b10000nnn [TXB2][TXB1][TXB0] */
#define MCP2515_CMD_RDSTAT	0xA0 /* ref: DS20001801J FIGURE 12-8  */
 #define MCP2515_MSK_MSGBUF 0xC0 /* use to ret val of CMD_RDSTAT*/
 #define MCP2515_MSK_MSGRX0 0x40 /* use to ret val of CMD_RDSTAT*/
 #define MCP2515_MSK_MSGRX1 0x80 /* use to ret val of CMD_RDSTAT*/
#define MCP2515_CMD_RDRXSTAT	0xB0 /* ref: DS20001801J FIGURE 12-9  */

/* RX Buffer */
#define MCP2515_REG_RXB0CTRL 0x60
#define MCP2515_REG_RXB0SIDH 0x61
#define MCP2515_REG_RXB0EID8 0x63
#define MCP2515_REG_RXB0DLC  0x65
 #define MCP2515_MSK_EXIDE    0b1000
#define MCP2515_REG_RXB0D0   0x66

#define MCP2515_REG_RXB1CTRL 0x70
#define MCP2515_REG_RXB1SIDH 0x71
#define MCP2515_REG_RXB1EID8 0x73
#define MCP2515_REG_RXB1DLC  0x75
#define MCP2515_REG_RXB1D0   0x76

/* Interruption Flags */
enum MCP2515_INTTYPE{
	MCP2515_INT_RX0, MCP2515_INT_RX1,
	MCP2515_INT_TX0, MCP2515_INT_TX1, MCP2515_INT_TX2,
	MCP2515_INT_ERR, MCP2515_INT_WAK, MCP2515_INT_MERR /* Receive Interrupt
							      Wake-up Interrupt
							      Message Error     */
};

/* Pin Control */
#define MCP2515_REG_BFPCTRL	0x0C

/* Flag Register */
#define MCP2515_REG_EFLG	0x2D
#define MCP2515_REG_CANINTE     0x2B

#define MCP2515_REG_CANINTF	0x2C
#define MCP2515_MSK_RX0IF    0b00000001
#define MCP2515_OFST_RX0IF   0
#define MCP2515_MSK_RX1IF    0b00000010
#define MCP2515_OFST_RX1IF   1

#define MCP2515_REG_TEC		0x1C /* Transmit Error Counter */
#define MCP2515_REG_REC		0x1D /* Receive  Error Counter */

/* Data Send */
#define MCP2515_PRI_URGENT	0b11
#define MCP2515_PRI_HIGH	0b10
#define MCP2515_PRI_MEDIUM	0b01
#define MCP2515_PRI_LOW		0b00

#define MCP2515_REG_TXB0CTRL	0x30
#define MCP2515_REG_TXB1CTRL	0x40
#define MCP2515_REG_TXB2CTRL	0x50
#define MCP2515_REG_TXRTSCTRL	0x0D

/* Data Receive */
#define MCP2515_REG_RXB0CTRL	0x60
#define MCP2515_REG_RXB1CTRL	0x70

#define MCP2515_RMD_FREE	0b01100000
#define MCP2515_RMD_RESTRICT	0b00000000

#endif
