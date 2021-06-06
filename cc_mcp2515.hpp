/**
 * @file    cc_mcp2515.hpp
 * @brief   MCP2515 Operations
 * @details 
 * @author  audotqkash (github.com/audotqkash)
 * @date    June 1st, 2021.
 * $Version 1.0
 * @par     June 1st, 2021 : Implemented the SPI Interface
 *          June 6th, 2021 : Implemented the CAN Configurator
 *          June 6th 2021  : Implemented the Registor Access Functions
 *
 *
 */

#pragma once

#include <stdint.h>
#include <SPI.h>
#include "mcp2515_defs.h"
#include "mcp2515_stdefs.h"

#undef bitWrite

class CCM2515
{
    private:
        uint32_t _mcp_cs = 0;
        uint32_t _canspd = 0;
        uint32_t _oscfrq = 0;
        uint8_t  mode    = 0xff;
        mcp2515pktype _pktype = mcp2515pktype::soic;

        SPISettings mcp2515_spicnf;


        void _init(uint32_t cs);
        uint8_t checkinit(void); 
        

        uint8_t orderInst(uint8_t inst);
        uint8_t orderSend(uint8_t inst, uint8_t *sendData, int slen);
        uint8_t orderRecv(uint8_t inst, uint8_t *recvData, int rlen);
        uint8_t orderRecv(uint8_t inst, uint8_t address, uint8_t *recvData, int rlen);
        uint8_t orderRecv(uint8_t inst, uint8_t *address, uint8_t *recvData, int rlen);


        void printDouble(double num);


    public: 
        CCM2515(uint32_t cs, uint32_t spispd);           /* constructor  */
	    CCM2515(uint32_t cs, uint32_t spispd, mcp2515pktype type);           /* constructor  */

        uint8_t begin(void);            /* end of setup */
        uint8_t reset(void);            /* reset        */

        bool setConfig(uint32_t canspd, canbaudrate oscfreq);
        void setConfig(uint8_t brp, uint8_t ps1, uint8_t ps2, uint8_t prseg);
        void setConfig(uint8_t brp, uint8_t ps1, uint8_t ps2, uint8_t prseg, uint8_t sjw);
        void setConfig(uint8_t cnf1, uint8_t cnf2, uint8_t cnf3);

        void pinMode(mcp2515pin rxbnum, mcp2515pinmd md);

        uint8_t readByte(uint8_t address);
        size_t  readBytes(uint8_t address, uint8_t *data, size_t len);
        void    bitWrite(uint8_t address, uint8_t bitnum, uint8_t value);
        void    bitsWrite(uint8_t address, uint8_t mask, uint8_t value);

};

