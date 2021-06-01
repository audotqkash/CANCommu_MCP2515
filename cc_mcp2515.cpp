/**
 * @file    cc_mcp2515.cpp
 * @brief   MCP2515 Operations
 * @author  audotqkash (github.com/audotqkash)
 * @date    June 1st, 2021.
 * $Version 1.0
 * @par     June 1st, 2021 : Implemented the SPI Interface
 *  
 * 
 */

#include <cc_mcp2515.h>

#define MCP2515_SELECT()   digitalWrite(_mcp_cs, LOW)
#define MCP2515_DESELECT() digitalWrite(_mcp_cs, HIGH)

#define LIMITCNT_INITCHK 10
#define WAIT1000MS      1000
#define MAXCONTINUOUSREAD 16

CCM2515::CCM2515(uint32_t cs){
    _init(cs);
    mode = 0xff;
}

/**
 *  @brief set spi configuration
 */
void CCM2515::_init(uint32_t cs){
    _mcp_cs = cs;
    mcp2515_spicnf = SPISettings(1000000,MSBFIRST,SPI_MODE0);
}

/**
 * @brief   Start CAN Communication (Normal Mode)
 * @param   none
 * @attention  Call after init() and setConf()        
 * @return  OK:0 ,NG:1
 */
uint8_t CCM2515::begin(void){
    mode = MCP2515_MD_NORMAL;
    return 0;
}

/**
 * @brief   Start CAN Communication (Normal Mode)
 * @param   none
 * @attention  Call after init() and setConf()        
 * @return  OK:0, NG:1
 */
uint8_t CCM2515::reset(void){
    int chkcnt = 0;
    while(true){

        orderInst(MCP2515_CMD_RESET);
        delay(100);
        
        if(chkcnt > LIMITCNT_INITCHK){
            return 1;
        }else if(checkinit() != 0){                      /* Mode check */
            delay(WAIT1000MS);
        }else{
            break;
        }
        chkcnt++;
    }

    mode = MCP2515_MD_CONFIG;
    return 0;
}

uint8_t CCM2515::checkinit(void){
    uint8_t result = 0;
    uint8_t ret;
    uint8_t data[MAXCONTINUOUSREAD];

    orderRecv(MCP2515_CMD_READ, 0x20, data, MAXCONTINUOUSREAD);

    for(int regL = 0x0; regL < MAXCONTINUOUSREAD; regL++){
        switch(regL){
            case 0x0E:
                if((ret & 0xEE) != 0x80){
                    ret += 1;
                }
                break;
            case 0x0F:
                if((ret & 0xFF) != 0x87){
                    ret += 1;
                }
                break;
            default:
                if((ret & 0xFF) != 0x0){
                    ret += 1;
                }
                break;
        }
    }
    return (result > 0);
}


/**
 *  @brief SPI Interface : Send an instruction
 *  @param [in] inst     :  MCP2515_CMD_####
 */
uint8_t CCM2515::orderInst(uint8_t inst){
    return orderSend(inst, (uint8_t *)0, 0);
}

/**
 *  @brief SPI Interface : Send an instruction and data
 *  @param [in] inst     : MCP2515_CMD_####
 *  @param [in] address  : pointer to buffer of send data
 *  @param [in] slen     : request data length
 */
uint8_t CCM2515::orderSend(uint8_t inst, uint8_t *sendData, int slen){
    SPI.beginTransaction(mcp2515_spicnf);
    MCP2515_SELECT();
    SPI.transfer(inst);
    if(sendData != 0){
        for(int i = 0; i < slen; i++){
            SPI.transfer(sendData[i]);
        }
    }
    MCP2515_DESELECT();
    SPI.endTransaction();
    return 0;
}

/**
 *  @brief SPI Interface : Receive Data by send an instruction
 */
uint8_t CCM2515::orderRecv(uint8_t inst, uint8_t *recvData, int rlen){
    return orderRecv(inst, (uint8_t *)0, recvData, rlen);
}

/**
 *  @brief SPI Interface : Receive Data by send an Instruction and a register address
 *  @param [in] inst      : MCP2515_CMD_####
 *  @param [in] address   : MCP2515_REG_####
 *  @param [out] recvData : pointer to buffer of received data
 *  @param [in]  rlen     : request data length
 */
uint8_t CCM2515::orderRecv(uint8_t inst, uint8_t address, uint8_t *recvData, int rlen){
    return orderRecv(inst, &address, recvData, rlen);
}

/**
 *  @brief SPI Interface : Receive Data by send an Instruction and a register address
 *  @param [in] inst      : MCP2515_CMD_####
 *  @param [in] address   : (pointer to register number) MCP2515_REG_####
 *  @param [out] recvData : pointer to buffer of received data
 *  @param [in]  rlen     : request data length
 */
uint8_t CCM2515::orderRecv(uint8_t inst, uint8_t *address, uint8_t *recvData, int rlen){
    SPI.beginTransaction(mcp2515_spicnf);
    MCP2515_SELECT();

    SPI.transfer(inst);
    if(address != 0){
        SPI.transfer(*address);
    }
    for(int i = 0; i < rlen; i++){
        recvData[i] = SPI.transfer(0);
    }
    MCP2515_DESELECT();
    SPI.endTransaction();
    return 0;
}