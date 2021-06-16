/**
 * @file    sample3_echoback.ino
 * @brief   MCP2515 test code
 * @details This program is for WioTerminal.
 * @note    for Arduino platform (include platformio)
 * @date    June 6, 2021.
 * @par     June 6, 2021. : Implemented minimum setup flow
 * @par     June 16,2021  : Implemented send method
 */

#include <Arduino.h>
#include <SPI.h>
#include <cc_mcp2515.h>

#define WIO_MCP_CS  BCM8
#define WIO_D4      BCM25
#define WIO_D3      BCM24
#define WIO_D2      BCM23

/* Create a Instance of CANCommu MCP2515*/
CCM2515 CAN(WIO_MCP_CS, 1000000);

/* function declaration */
void mcp2515_setup();

void setup() {
    Serial.end();

/* Serial port setting.
 * Please change the value suit for your environment.
 */
    Serial.begin(115200);
    while(!Serial);
    
    /* Setup (1)  : Slave Select Signal Pin */
    pinMode(WIO_MCP_CS, OUTPUT);
    digitalWrite(WIO_MCP_CS, HIGH);

    SPI.begin();                                        /* SPI Setup        */

    mcp2515_setup();                                    /* mcp2515 setup    */
}

/**
 * @brief setup MCP2515 using the library "CANCommu_MCP2515"
 * @details In this function, set 
 *  (1) CAN Communication
 *  (2) CAN Packet filtering
 *  (3) Interrupt
 *  (4) MCP2515 Operation mode
 * @pre   SPI.begin();
*/
void mcp2515_setup(){
    Serial.print(" [     setup start     ]\n");
    Serial.print("    |- Reset\n");
    while(CAN.reset() != 0){
        Serial.print("[!] failed\n");
        delay(1000);
    }

    Serial.print("        |- OK\n");

    /**
     *  [ CAN Communication setting ]
     */
    CAN.setConfig(0,6,4,2,1);

    /** 
     * [ RXnBF pin mode setting]
     * 
    */
    Serial.print("    |- Output Pin\n");
    CAN.pinMode(mcp2515pin::RX0BUF, mcp2515pinmd::interrupt);
    CAN.pinMode(mcp2515pin::RX1BUF, mcp2515pinmd::interrupt);

    Serial.print(" [     setup end       ]\n");

    /* Change Operation Mode to Normal  */
    CAN.begin();
    Serial.print(" [       Normal        ]\n");
}


void loop(){
    uint8_t ret;
    static candata_st candata;
    /* Data Receive           */
    do{
    ret = CAN.getRxStat();                      /* Check RX Buffer             */
    if((ret & MCP2515_MSK_MSGRX0) != 0){        /* RX Buffer #0 has data       */
        candata = CAN.recv(0);                  /* Extract data from RX0BF     */
        candata.show('c');                      /* Serial output (csv)         */
        candata.bnum = 0;
        CAN.send(candata);                      /* send back the received data */
    }
    if((ret & MCP2515_MSK_MSGRX1) != 0){        /* RX Buffer #1 has data       */
        candata = CAN.recv(1);                  /* Extract data from RX1BF     */
        candata.show('c');                      /* Serial output (csv)         */
        candata.bnum = 1;
        CAN.send(candata);                      /* send back the received data */
    }
    }while(ret != 0);
}