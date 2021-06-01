/**
 * @file    cc_mcp2515.h
 * @brief   MCP2515 Operations
 * @author  audotqkash (github.com/audotqkash)
 * @date    June 1st, 2021.
 * $Version 1.0
 * @par     June 1st, 2021 : Implemented the SPI Interface
 *
 *
 */

#ifndef _ccmcp2515_H_
#define _ccmcp2515_H_

#include <stdint.h>
#include <SPI.h>
#include "mcp2515_defs.h"

#define MAXCHAR_ONEMESSAGE 8

class CCM2515
{
    private:
        uint32_t _mcp_cs = 0;
        uint32_t _canspd = 0;
        uint32_t _oscfrq = 0;
        uint8_t  mode    = 0xff;

        SPISettings mcp2515_spicnf;


        void _init(uint32_t cs);
        uint8_t checkinit(void); 
        

        uint8_t orderInst(uint8_t inst);
        uint8_t orderSend(uint8_t inst, uint8_t *sendData, int slen);
        uint8_t orderRecv(uint8_t inst, uint8_t *recvData, int rlen);
        uint8_t orderRecv(uint8_t inst, uint8_t address, uint8_t *recvData, int rlen);
        uint8_t orderRecv(uint8_t inst, uint8_t *address, uint8_t *recvData, int rlen);


    public: 
	    CCM2515(uint32_t cs);           /* constructor  */

        uint8_t begin(void);            /* end of setup */
        uint8_t reset(void);            /* reset        */

};

typedef struct{
    uint32_t time	= 0;
    uint32_t id		= 0;
    bool     eid_en	= false;
    uint8_t  bnum	= 0;
    uint8_t  data[MAXCHAR_ONEMESSAGE] = {0};
    uint8_t  len	= 0;
}candata_st;

#endif
