/**
 * @file    sample2_L-chika.ino
 * @brief   MCP2515 test code
 * @details This program is for WioTerminal.
 * @note    for Arduino platform (include platformio)
 * @date    June 6, 2021.
 * @par     June 6, 2021. : Implemented minimum setup flow
 * @par     June 6, 2021  : Implemented setting of digital pin
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
}


void loop(){
    char rnd = micros() % 4;
    switch(rnd){
        case 0:
            CAN.pinMode(mcp2515pin::RX0BUF, mcp2515pinmd::high);
            CAN.pinMode(mcp2515pin::RX1BUF, mcp2515pinmd::high);
            break;
        case 1:
            CAN.pinMode(mcp2515pin::RX0BUF, mcp2515pinmd::high);
            CAN.pinMode(mcp2515pin::RX1BUF, mcp2515pinmd::low);
            break;
        case 2:
            CAN.pinMode(mcp2515pin::RX0BUF, mcp2515pinmd::low);
            CAN.pinMode(mcp2515pin::RX1BUF, mcp2515pinmd::high);
            break;
        case 3:
            CAN.pinMode(mcp2515pin::RX0BUF, mcp2515pinmd::low);
            CAN.pinMode(mcp2515pin::RX1BUF, mcp2515pinmd::low);
            break;
    }
    delay(240);
}