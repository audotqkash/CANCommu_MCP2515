/**
 * @file    cc_mcp2515.cpp
 * @brief   MCP2515 Operations
 * @author  audotqkash (github.com/audotqkash)
 * @date    June 1st, 2021.
 * $Version 1.0
 * @par     June 1st, 2021 : Implemented the SPI Interface
 *          June 6th, 2021 : Implemented the CAN Configurator
 *          June 15th, 2021: Implemented the CAN Recv&Send function
 * 
 */

#include "cc_mcp2515.hpp"

#define MCP2515_SELECT()   digitalWrite(_mcp_cs, LOW)
#define MCP2515_DESELECT() digitalWrite(_mcp_cs, HIGH)

#define LIMITCNT_INITCHK 10
#define WAIT1000MS      1000
#define MAXCONTINUOUSREAD 16

/**
 * @param [in] cs       SPI chip select(slave select) pin number
 * @param [in] spispd   SPI clock speed (Default = 10MHz)
*/
CCM2515::CCM2515(uint32_t cs, uint32_t spisped = 10000000){
    _init(cs);
    mode = 0xff;
}

/**
 * @param [in] cs       SPI chip select(slave select) pin number
 * @param [in] spispd   SPI clock speed (Default = 10MHz)
 * @param [in] 
*/
CCM2515::CCM2515(uint32_t cs, uint32_t spisped = 10000000, mcp2515pktype type = soic){
    _init(cs);
    mode = 0xff;
    _pktype = type;
}

/**
 *  @brief setup spi configuration
 *  @details SPI baudrate, bit Order, SPImode.
 *  @param[in] cs : slave select pin
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
    uint8_t ret;
    uint8_t data[3];
    
//
//    Serial.print("[ MODE CHANGE ]");
//    orderRecv(MCP2515_CMD_READ, MCP2515_REG_CANSTAT, data, 1);
//    ret =  (data[0] & MCP2515_MSK_OPMOD) >> MCP2515_OFST_OPMOD;
//    Serial.printf("   %d ->",ret);
//

    data[0] = MCP2515_REG_CANCTRL;
    data[1] =  MCP2515_MSK_REQOP;
    data[2] = MCP2515_MD_NORMAL << MCP2515_OFST_REQOP;
    orderSend(MCP2515_CMD_BITMOD, data, 3);
    delay(500);

//
//    orderRecv(MCP2515_CMD_READ, MCP2515_REG_CANSTAT, data, 1);
//    ret =  (data[0] & MCP2515_MSK_OPMOD) >> MCP2515_OFST_OPMOD;
//    Serial.printf(" %d\n\n",ret);


//    Serial.printf(" # Start Normal Mode\n");
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

    readBytes(0x20, data, MAXCONTINUOUSREAD);

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


#define MCP2515_PARAMCALC_LOOPMAX 30
/**
 * @brief Automatic Configuration of Time Segments
 * @param [in] canspd : CAN Communication baudrate (bps)
 * @param [in] oscfreq: Connected Clock Source of MCP2515 (Hz)
 */
bool CCM2515::setConfig(uint32_t canspd, canbaudrate oscfreq){
    bool ret =  false;                                  /* ??????????????????????????????????????????  */
    
    int lp0 = 0;                                        /* ????????????????????????????????? */
    
    const double oscts = (double)1 / oscfreq; 
    const double cants = (double)1 / canspd;
    uint8_t cond1 = 1, cond2 = 1, cond3 = 1, total = 1;   /* ????????????????????????????????? */


    canctlprm params = {0};                                     /* ???????????????????????? */

    /* ????????????????????????????????????????????????????????????*/
    params.sjw     = 1;        /*RANGE: 1 to 4 */
    params.propSeg = 2;        /*RANGE: 1 to 8 */
    params.tdelay  = 1;        /*RANGE: 1 to 2 */

    params.syncSeg = 1;           /* 1 ~ 1 */


//   Serial.printf(" OSC %d,CAN %d\n\n",oscfreq, canspd);         /* ????????????????????????CANBUS?????????????????????????????????*/
   
    _oscfrq = oscfreq;
    _canspd  = canspd;  

//    Serial.printf("        ----  TQ TABLE  ----\n");
//    Serial.printf(" osc : %lu\n", _oscfrq);
//    Serial.printf(" can : %lu\n", _canspd);
//    Serial.printf("  |  BRP  |  TQ   |\n");
//    Serial.printf("   ------- -------\n");
//    for(int brp = 0; brp < 64; brp++){                                 /* make a list of brp values */
//        double req_tq = (double)_oscfrq / (2*(brp+1)) / _canspd;
//        if((uint32_t)(req_tq * 1000) % 1000 == 0){
//            Serial.printf("   |  %2d  |", brp + 1);
//            Serial.print(req_tq);
//            Serial.printf(" |\n");
//        }else;
//    }

    for(lp0 = 1; lp0 < 64; lp0++){                                      /* Extract a brp value */
        params.tq = (double)(2 * lp0) / (double)oscfreq;
       
        double tmpd = cants / params.tq;
 
        if((tmpd - (double)((int)tmpd)) == 0){
            params.brp = lp0;
            params.tbit = (uint8_t)tmpd;
            lp0 = 999;                      /*   ????????????????????????BRP????????????1??????????????????????????????  */

//            Serial.printf(" tq = ");
//            printDouble(params.tq);
//            Serial.printf(", brp = %d",params.brp);
//            Serial.printf(", Tbit = %d\n",params.tbit);                                                                       
        }
    }
        
    while(params.ps2 <= params.sjw){                                         /* PreRequired Condition    */
        params.ps2 += 1;                                                     /* ??????????????????(1) ps2:sjw   */
    }
      
    int limit = MCP2515_PARAMCALC_LOOPMAX;
    while(limit > 0 && ret == false){                                        /* ???????????????????????????       */
        cond1 = params.propSeg + params.ps1;
        cond2 = params.syncSeg + cond1;
        cond3 = params.ps2;
        total = cond2 + cond3;
        
        while(limit == MCP2515_PARAMCALC_LOOPMAX && params.tbit < total){      /* Segment calculation     */
            if(params.tdelay > 1 || params.propSeg > 1 || params.sjw > 1){
                if(params.tdelay > 1)
                    params.tdelay -= 1;
                else if(params.propSeg > 1){
                    params.propSeg -= 1;
                }else if(params.sjw > 1){
                    params.sjw -= 1;
                    if(params.ps2 > params.sjw + 1){
                        params.ps2 -= 1;
                    }else;
                }else;
                
                cond1 = params.propSeg + params.ps1;
                cond2 = params.syncSeg + cond1;
                cond3 = params.ps2;
                total = cond2 + cond3;
            }else{
                break;
            }
        }    
//       Serial.printf("  %d  %d  %d  %d\n",params.syncSeg, params.propSeg, 
//                            params.ps1, params.ps2);
        
        
        if(cond1 >= params.ps2 &&                                       /* ??????????????????(2)          */
          ((float)cond2/(float)total > 0.59) && 
          ((float)cond2/(float)total < 0.71))
        {
            //Serial.printf("   [pos] %f\n", (float)cond2/(float)total);
            if(cond1 >= params.tdelay)                                  /* ??????????????????(3)          */
            {
                if(total == params.tbit){                               /* ??????????????????(4)          */
                    ret = true;                                         /* ????????????????????????         */
                                                                        /*                         */
                }else;                                                  /* TQ??????                   */
                
//                Serial.printf("   total  %d < %d tq\n", total, params.tbit);
                
            }else;                                                      /* ??????????????????(3)??????      */
            
        }else;                                                          /* ??????????????????(2)??????      */
        
        if(ret == false){                            /* ??????????????????????????????????????? */
            //printf("%f\n",(float)(cond2 + 1)/(float)(total + 1) - (float)0.65);
            //printf("%f\n",(float)(cond2    )/(float)(total + 1) - (float)0.65);
            float v1 = ((float)(cond2 + 1)/(float)(total + 1) - (float)0.65);
            float v2 = ((float)(cond2    )/(float)(total + 1) - (float)0.65);
            if(v1*v1 < v2*v2){
                params.ps1 += 1;     /* 1 ~ 8 */
            }else{
                params.ps2 += 1;     /* 2 ~ 8 */
            }
        }
        
        limit -= 1;
        if(total > params.tbit){                                           /* TQ??????????????????        */
            limit = 0;
        }
        
    }                                                               /* end of ???????????????????????????       */   

    
    if(ret == true){
        Serial.print("        |- ------------------------------------------------------------\n");
        Serial.print("        |- |    BRP      |    SJW      |          TOTAL TQ            |\n");
        Serial.printf("        |- |    %5d    |    %5d    |           %5d              |\n", 
                                                params.brp, params.sjw, params.tbit);
        Serial.print("        |- ------------------------------------------------------------\n");
        Serial.print("        |- ---------------------------------------------------------\n");
        Serial.print("        |- |   SYNC      |   PropSeg   |    PhSEG1   |    PhSeg2   |\n");
        Serial.print("        |-");
        Serial.printf(" |    %5d    ", params.syncSeg);
        Serial.printf("|    %5d    ",  params.propSeg);
        Serial.printf("|    %5d    ",  params.ps1);
        Serial.printf("|    %5d    |\n", params.ps2);
        Serial.print("        |- ---------------------------------------------------------\n");
       setConfig(params.brp - 1, params.ps1 -1, params.ps2 -1, params.propSeg-1, params.sjw);
    }else{
       Serial.printf("        |- Avort!\n");
    }

    return ret;
}

/**
 * @brief  Manual Configuration of Time Segments
 * @param[in] brp    : baudrate prescaler controll value
 * @param[in] ps1    : phase segment 1 length
 * @param[in] ps2    : phase segment 2 length
 * @param[in] prseg  : Propagation Segment Length bits
 * @return           none
 */
void CCM2515::setConfig(uint8_t brp, uint8_t ps1, uint8_t ps2, uint8_t prseg){
    setConfig(brp, ps1,ps2,prseg,(uint8_t)1);
}

/**
 * @brief  Manual Configuration of Time Segments
 * @param[in] brp    : baudrate prescaler controll value
 * @param[in] ps1    : phase segment 1 length
 * @param[in] ps2    : phase segment 2 length
 * @param[in] prseg  : Propagation Segment Length bits
 * @param[in] sjw    : Synchronization Jump Width
 * @return           none
 * @par       Jan 6, 2021 : Change Registor change method
 */
void CCM2515::setConfig(uint8_t brp, uint8_t ps1, uint8_t ps2, uint8_t prseg, uint8_t sjw){
    uint8_t data[3] = {0};

    data[0] |=  brp << MCP2515_OFST_BRP;
    data[0] |=  sjw << MCP2515_OFST_SJW;

    data[1] |=  ps1 << MCP2515_OFST_PS1;
    data[1] |=  prseg << MCP2515_OFST_PRSEG;
    data[1] |=  0 << MCP2515_OFST_SAM;
    data[1] |=  1 << MCP2515_OFST_BTLMD;

    data[2] |=  data[2] = ps2 << MCP2515_OFST_PS2;

    setConfig(data[0], data[1], data[2]);
}


/**
 * @brief  Manual Configuration of Time Segments by set MCP2515 CNF1 registor directly
 * @details  
 * 
 * @param[in] cnf1    : Raw data of MCP2515 CNF1 registor 
 * @param[in] cnf2    : Raw data of MCP2515 CNF2 registor 
 * @param[in] cnf3    : Raw data of MCP2515 CNF3 registor 
 * @return           none
 * @note https://www.kvaser.com/support/calculators/bit-timing-calculator/
 */
void CCM2515::setConfig(uint8_t cnf1, uint8_t cnf2, uint8_t cnf3){
    uint8_t data[3];

    bitsWrite(MCP2515_REG_CNF1, MCP2515_MSK_PS1 | MCP2515_MSK_PRSEG | MCP2515_MSK_SAM | MCP2515_MSK_BTLMD, cnf1);

    bitsWrite(MCP2515_REG_CNF2, MCP2515_MSK_SJW | MCP2515_MSK_BRP, cnf2);

    bitsWrite(MCP2515_REG_CNF3, MCP2515_MSK_PS2, cnf3);

    readBytes(MCP2515_REG_CNF3, data,3);
    Serial.printf("        |- CNF1 %02X, ", data[2]);
    Serial.printf("CNF2 %02X, ",data[1]);
    Serial.printf("CNF3 %02X\n",data[0]);
}

/**
 * @brief ???????????????????????????????????????????????????
 * @param[in] rxbnum ????????????????????????
 * @param[in] md 0: ??????, 1: ????????????, 0xff: ??????????????????1, ????????????: 0
*/
void CCM2515::pinMode(mcp2515pin pin, mcp2515pinmd md){
    /* | BnBFS | BnBFE  | BnBFM  | */
    /* |   ??   |   0    |   ??    | *//* ??????????????????, ???????????????????????????*/
    /* |   ??   |   1    |   1    | *//* ??????????????????, ??????????????????*/
    /* |   0   |   1    |   0    | *//* ??????????????????  LOW        */
    /* |   1   |   1    |   0    | *//* ??????????????????  HIGH       */

    uint8_t dat = 0x0;
    uint8_t ret = 0x0;
    uint8_t bufnum = 0x0;

    if(_pktype == mcp2515pktype::soic){}
    if(pin < mcp2515pin::RX1BUF ||  mcp2515pin::RX0BUF < pin){
        Serial.printf(" [!] ???????????????????????????11~12????????????. (req : %d)\n", pin);
    }

    if(_pktype == mcp2515pktype::soic || _pktype == mcp2515pktype::pdip){
        bufnum = (pin + 1) % 2;
    }else{
        bufnum = pin % 2;
    }

    switch(md){
        case mcp2515pinmd::disable:          /* ??????????????????, ???????????????????????????*/
            dat = 0b00000000;
            break;
        case mcp2515pinmd::interrupt:          /* ??????????????????, ??????????????????*/
            dat = 0b00001111;
            break;
        case mcp2515pinmd::low:
            dat = 0b00001100;  /*Digital Low Signal*/
            break;
        case mcp2515pinmd::high:
            dat = 0b00111100;  /*Digital High Signal*/
            break;
        default:
            dat = 0b00111100;  /*Digital High Signal*/
            break;
    }

    bitsWrite(MCP2515_REG_BFPCTRL, 0b00010101 << bufnum, dat);
    ret = readByte(MCP2515_REG_BFPCTRL);
    
    if((ret & (0b00100 << bufnum)) != 0){
        if((ret & (0b00001 << bufnum)) != 0){
            Serial.printf("        |- RXB%d : %s\n", bufnum, "Interrupt Notify");
        }else{
            if((ret & (0b010000 << bufnum)) != 0){
                Serial.printf("        |- RXB%d : %s\n", bufnum, "DigSig : High");
            }else{
                Serial.printf("        |- RXB%d : %s\n", bufnum, "DigSig : Low");
            }
        }
    }else{
         Serial.printf("        |- RXB%d : %s\n", bufnum, "High-impedance");
    }
    
}

/**
 * @brief Check if a data in any Rx Buffer.
 * @param[in,out] void Nothing 
 * @return 0x0 : ???????????????
 *         0x1 : RXB0????????????
 *         0x2 : RXB1????????????
 * @note ????????????
 */
uint8_t CCM2515::getRxStat(void){
    uint8_t data;
    orderRecv(MCP2515_CMD_RDRXSTAT, &data, 1);
    return data;
}


/**
 * @brief Extract data from RX Buffer #0 or #1
 */
candata_st CCM2515::recv(uint8_t rxnum){
    candata_st dat;
    uint8_t ret;
    //Serial.printf("   ----- RX%d Data Extract -----\n", buffnum);
    dat.bnum = 0xff;
    memset(dat.data, 0x0, sizeof(uint8_t) * 8);
    dat.len = 0;

    ret = getRxStat();
    if(rxnum == 0 && (ret & MCP2515_MSK_MSGRX0) != 0){
        dat.bnum = 0;
        dat.len = extRxBuf(&dat);
        bitsWrite(MCP2515_REG_CANINTF, MCP2515_MSK_RX0IF, 0x0);
    }else;

    if(rxnum == 1 && (ret & MCP2515_MSK_MSGRX1) != 0){
        dat.bnum = 1;
        dat.len = extRxBuf(&dat);
        bitsWrite(MCP2515_REG_CANINTF, MCP2515_MSK_RX1IF, 0x0);
    }else;

    return dat;
}

/**
 * @brief
 * @pre   set dat.bnum
 * @retval packet length
 * @retval 0xFF : Error
 */
uint8_t CCM2515::extRxBuf(candata_st *dat){
    uint8_t pcthead[5];
    uint8_t cmdbit = 0;
    
    
    if(dat->bnum == 0){  
        cmdbit = 000;         /* RX0BUF / SIDH*/
    }else if(dat->bnum == 1){
        cmdbit = 100;         /* RX1BUF / SIDH */
    }else{
        return 0xFF;         /* ????????????????????????????????? */
    }

    dat->time = millis();

    orderRecv((uint8_t)(MCP2515_CMD_RDRXBUF | cmdbit), pcthead, 5);

    dat->eid_en = (pcthead[1] & MCP2515_MSK_EXIDE) != 0? 1 : 0;
    if(dat->eid_en == 0){
        dat->id  = (uint32_t)pcthead[0] << 3;
        dat->id |= (pcthead[1] & 0xE0) >> 5;
    }else{
        dat->id  = (uint32_t)pcthead[0] << 21;
        dat->id |= ((uint32_t)pcthead[1] & 0xE0) << 13 ;
        dat->id |= ((uint32_t)pcthead[1] & 0x03) << 16 ;
        dat->id |= ((uint32_t)pcthead[2] & 0xFF) << 8 ;
        dat->id |= ((uint32_t)pcthead[3] & 0xFF)  ;
    }

    dat->len = pcthead[4] & 0xF;                               /* Extact DLC */

    if(dat->len > 0){
        orderRecv((uint8_t)(MCP2515_CMD_RDRXBUF | cmdbit | 0x2), dat->data, dat->len);
    }

    return dat->len;
}

/**
 * @brief Send CAN Data
 */
uint8_t CCM2515::send(candata_st dat){
    setTxData(&dat);
    send(dat.bnum);
    return 0;
}

/**
 * @brief Set a CAN Data to TX Buffer, however do not send yet.
 */
void CCM2515::setTxData(candata_st *dat){
    uint8_t packet[13];
    uint8_t cmdbit = 0;

    if(dat->bnum >= 3){
        dat->bnum = 2;
    }
    cmdbit = 1 * dat->bnum;

    if(dat->eid_en == false){
        Serial.printf("dat->id 0x%2x\n", dat->id);
        packet[0] = (dat->id & 0x7F8) >> 3;
        packet[1] = (dat->id & 0x7) << 5;
        packet[2] = 0;
        packet[3] = 0;
    }else{
        packet[1] = MCP2515_MSK_EXIDE;
        packet[0] = (dat->id & 0x1FE00000) >> 21;
        packet[1] |= (dat->id &  0x1C0000) >> 13;
        packet[1] |= (dat->id &   0x30000) >> 16;
        packet[2] = (dat->id &     0xFF00) >> 8;
        packet[3] = (dat->id &       0xFF);
    }
    if(dat->len >= 9){
        dat->len = 8;
    }
    packet[4] = dat->len;

    for(int i = 0; i < dat->len; i++){
        packet[5 + i] = dat->data[i];
    }

    orderSend(MCP2515_CMD_LDTXBUF | cmdbit, packet, 5 + dat->len);
}

/**
 * @brief send TxBuffer Data
 * @pre setTxData()
 */
uint8_t CCM2515::send(uint8_t bnum){
    if(bnum >= 3){
        return 1;
    }
    orderInst(MCP2515_CMD_RTS | (0x1 << bnum));
    return 0;
}

/**
 * @brief SPI Interface : One registor read
 */
uint8_t CCM2515::readByte(uint8_t address){
    uint8_t data;
    orderRecv(MCP2515_CMD_READ, address, &data, 1);
    return data;
}

/**
 * @brief SPI Interface : Registors read
 */
size_t CCM2515::readBytes(uint8_t address, uint8_t *data, size_t len){
    orderRecv(MCP2515_CMD_READ, address, data, len);
    return len;
}

/**
 * @brief modify bit value of registor
 */
void CCM2515::bitWrite(uint8_t address, uint8_t bitnum, uint8_t value){
    bitsWrite(address, bit(bitnum), (value & 0x1) << bitnum);
}

/**
 * @brief modify selected bits value of registor
 * @param address[in]   : registor value
 * @param mask   write bit mask
 * @param value  write value
 * @return  none
 */ 
void CCM2515::bitsWrite(uint8_t address, uint8_t mask, uint8_t value){
    uint8_t data[3] = {0};

    data[0] = address;
    data[1] = mask;
    data[2] = value;
    orderSend(MCP2515_CMD_BITMOD, data, 3);
}


/**
 *  @brief SPI Interface : Send an instruction
 *  @param [in] inst     :  MCP2515_CMD_RESET or MCP2515_CMD_RTS
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
 *  @param [in] inst      : READ RX BUFFER or READ STATUS or READ RX STATUS
 *  @retval  0  : success
 *  @retval > 0 : error 
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
 *  @retval  0  : success
 *  @retval > 0 : error 
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
 *  @retval  0  : success
 *  @retval > 0 : error 
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


/**
 * @brief [DEBUGTOOL] print a double value
 * @details  
 * @pre  Serial.begin();
 * @note In this function, Use "Serial.printf"
*/
void CCM2515::printDouble(double num){
    int lp1 = 0;
    int lp2 = 0;
    
    if(num >= 1){
       Serial.printf("%d.",(int)num);
    }else{
       Serial.printf("0.");
    }
    
    num -= (double)((int)num);
    
    if(num < 1 && num > 0){
        for(lp1 = 1000000000; lp1 > 1; lp1 /= 1000){
            int tmpi = (int)(num * lp1);
            if((int)(num * lp1) < 1000 && (int)(num * lp1) > 0){
                for(lp2 = lp1; lp2 > 1000; lp2 /= 10){
                   Serial.printf("0");
                }
                for(lp2 = 100; lp2 > 1; lp2 /= 10){
                    if(tmpi < lp2){
                       Serial.printf("0");
                    }
                }
               Serial.printf("%d", tmpi);
            }
        }
    }else{
        //printf("%f",num);
       Serial.printf("0");
    }
}