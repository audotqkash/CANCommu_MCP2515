/**
 * @file mcp2515_stdefs.h
 * @brief Data structures for MCP2515 
 * @author audotqkash (github.com/audotqkash)
 * @date June 6th, 2021.
 * @par  June 6th, 2021 : Configulation parameters
 *       June 6h, 2021  : Package Type, Digital pin
 */
#ifndef _MCP2515_STDEFS_H_
#define _MCP2515_STDEFS_H_
#include <stdint.h>
#include "mcp2515_defs.h"

#define MAXCHAR_ONEMESSAGE 8

enum canbaudrate{
    OSC4MHZ =  4000000,
    OSC8MHZ =  8000000,
    OSC16MHZ = 16000000,
    OSC20MHZ = 20000000,
};

enum mcp2515pin{
    RX0BUF = MCP2515_PIN_RX0BUF,
    RX1BUF = MCP2515_PIN_RX1BUF,
    INTERRUPT = MCP2515_PIN_INT,
};

enum mcp2515pinmd{
    disable = 0,
    interrupt = 6,
    low = 4,
    high = 5,
};

enum mcp2515pktype{
    pdip,
    soic,
    tssop,
    qfn
};

typedef struct{
    uint32_t time	= 0;
    uint32_t id		= 0;
    bool     eid_en	= false;
    uint8_t  bnum	= 0;
    uint8_t  data[MAXCHAR_ONEMESSAGE] = {0};
    uint8_t  len	= 0;
}candata_st;

typedef struct
{
    double tq;                   /* Time Quantum        */
    uint8_t brp;                  /* Bitrate Prescaler   */
    uint8_t syncSeg;              /* Sync Segment        */
    uint8_t propSeg;              /* Propagation Segment */
    uint8_t ps1;                  /* PhaseSegment 1      */
    uint8_t ps2;                  /* PhaseSegment 2      */
    uint8_t sjw;
    uint8_t tdelay;
    uint8_t tbit;
}canctlprm;



#endif