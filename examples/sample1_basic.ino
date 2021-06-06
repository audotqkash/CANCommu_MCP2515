/**
 * @file    sample1_basicuse.ino
 * @brief   MCP2515 test code
 * @details This program is for WioTerminal.
 * @note    for Arduino platform (include platformio)
 * @date    June 6, 2021.
 * @par     June 6, 2021. : Implemented minimum setup flow
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
     *  Pattern 1. Set the desired CAN Baudrate, and the equipped oscillator clock speed.
     *  Pattern 2. Set MCP2515 configuration parameters by manually calculated.
     *  Pattern 3. Set MCP2515 "cnfX" registor value by manually calculated or refered known value;
     * */
    /* Pattern 1    ----------------------------------------*/
        //CAN.setConfig(0,6,4,2,1);
    /* Pattern 2    ----------------------------------------*/
        Serial.printf("    |- Auto Calculation\n");
        if(CAN.setConfig(500000, canbaudrate::OSC16MHZ) == false){
            Serial.printf("        |- Failed\n");
            Serial.printf(" ========== STOP ==========\n");
            while(true){
                delay(100);
            }
        }
        Serial.printf("        |- Normal END\n");
    /* Pattern 3    ----------------------------------------*/
        //CAN.setConfig(0x40, 0x9a, 0x07);

    /*<< CAN Communication Setting                          */
    

    Serial.print(" [     setup end       ]\n");

    /* Change Operation Mode to Normal  */
    CAN.begin();
    Serial.print(" [       Normal        ]\n");
}


void loop(){
    delay(1000);
}