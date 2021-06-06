/**
 * @file mcp2515_stdefs.h
 * @brief Data structures for MCP2515 
 * @author audotqkash (github.com/audotqkash)
 * @date June 6th, 2021.
 * @par  June 6th, 2021 : Configulation parameters
 */
#ifndef _MCP2515_STDEFS_H_
#define _MCP2515_STDEFS_H_
#include <stdint.h>

#define MAXCHAR_ONEMESSAGE 8

enum canbaudrate{
    OSC4MHZ =  4000000,
    OSC8MHZ =  8000000,
    OSC16MHZ = 16000000,
    OSC20MHZ = 20000000,
};

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