/**
 * @file    mcp2515_defs.h
 * @brief   Resister Address and Function Bit definition for MCP2515
 * @author  audotqkash (github.com/audotqkash)
 * @date    May 31th, 2021.
 * $Version 0.0 
 * @par     May 31th, 2021 : MCP2515 Register Address, Operation Mode, Instruction 
 *
 *
 */

/* CAN Control */
#define MCP2515_REG_CANCTRL	0x0F

/* CAN Status */
#define MCP2515_REG_CANSTAT	0x0E

/* MCP2515 Configuration #1 */
#define MCP2515_REG_CNF1	0x2A

/* MCP2515 Configuration #2 */
#define MCP2515_REG_CNF2	0x29

/* MCP2515 Configuration #3 */
#define MCP2515_REG_CNF3	0x28

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
#define MCP2515_CMD_RXBUF	0x90 /* ref: DS20001801J FIGURE 12-3  */
#define MCP2515_CMD_TXBUF	0x40 /* ref: DS20001801J FIGURE 12-5  */
#define MCP2515_CMD_RTS 	0x80 /* 0b10000nnn [TXB2][TXB1][TXB0] */
#define MCP2515_CMD_RDSTAT	0xA0 /* ref: DS20001801J FIGURE 12-8  */
#define MCP2515_CMD_RDRXSTAT	0xB0 /* ref: DS20001801J FIGURE 12-9  */

/* Interruption Flags */
enum MCP2515_INTTYPE{
	MCP2515_INT_RX0, MCP2515_INT_RX1,
	MCP2515_INT_TX0, MCP2515_INT_TX1, MCP2515_INT_TX2,
	MCP2515_INT_ERR, MCP2515_INT_WAK, MCP2515_INT_MERR /* Receive Interrupt
							      Wake-up Interrupt
							      Message Error     */
}

/* Pin Control */
#define MCP2515_REG_BFPCTRL	0x0C

/* Flag Register */
#define MCP2515_REG_EFLG	0x2D
#define MCP2515_REG_CANINTE     0x2B
#define MCP2515_REG_CANINTF	0x2C
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


