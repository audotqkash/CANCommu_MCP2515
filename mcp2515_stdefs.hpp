/**
 * @file mcp2515_stdefs.h
 * @brief Data structures for MCP2515 
 * @author audotqkash (github.com/audotqkash)
 * @date June 6th, 2021.
 * @par  June 6th, 2021 : Configulation parameters
 *       June 6th, 2021 : Package Type, Digital pin
 *       June 6th, 2021 : Receive CAN Data
 */
#ifndef _MCP2515_STDEFS_H_
#define _MCP2515_STDEFS_H_
#include <Arduino.h>
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

    /**
     * @brief show packet info 
     */
    void show(char type){
        if(type == 'c'){
            /*TIME,format,ID,LENGTH,DATA,BUF#*/
            Serial.printf("%lu,%c,0x%X,%d,0x%02X-%02X-%02X-%02X%02X%02X%02X%02X,%d,\"recv\"\n",
                    time, 
                    eid_en? 'e' : 's',
                    id,
                    len,
                    data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],
                    bnum);
        }else{
            Serial.println("  |   time   |    id    |idtype| data len | " );
            Serial.printf("   %8lu    0x%04X       %c  %d\n", time,(uint32_t)id, eid_en? 'e' : 's', len);
            Serial.printf("data(hex)  %02X%02X%02X%02X%02X%02X%02X%02X", data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7]);
        }
    }
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