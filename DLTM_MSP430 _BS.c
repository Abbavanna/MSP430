//******************************************************************************
// DLTS Steuersoftware
//
//               MSP430G2xx3
//           --------------------
//          |                    |
//          |                    |
//          |                    |
//          |                    |
//          |                    |
//          |VCC              GND|
//          |P1.0/A0    XIN/P2.6 |-XTAL 32kHz
// UART-RXD-|P1.1      XOUT/P2.7 |-XTAL
// UART-TXD-|P1.2           TEST |                      DAC
//          |P1.3/A3         RST |                  ------------
// intens  -|P1.4/A4 P1.7/UCB0SDA|<--------------->|SDA         |
//         -|P1.5/A5 P1.6/UCB0SCL|---------------->|SCL  I2C    |
//          |                    |
//          |P2.0            P2.5|- Y-STEP
// LED_GRN -|P2.1            P2.4|- Y-DIR
// LED_BLUE-|P2.2            P2.3|- PULSOUT = Pulsgeneratorausgang
//          |                    |
//
//******************************************************************************
//Fehlersuche sporadisches aufhängen:
//-readMultiADC() auskommentiert: ändert nichts
//-I2C-Geschwindigkeit reduzieren: scheint aufhängen zu verringern
//passiert nur, wenn zu schnell Befehle über UART nachgeschoben werden(TBC)

//Todo:
//-cancel I2C-transfer if no ACK comming +send error message (maybe with UCNACKIE ?)
//-in  SendACK():  100ms wait rauswerfen
//-MCP4728 Ref auf intern ändern! (210910: VDD=Vref -> Laserintens beeinflusst +5V (~80mV) -> X,Z,Tilt werden mit beeinflusst!
//  -> DAC-Platine auch modifizieren: R1-R6->300k

//Version:
//170925:-aus 170505 Poti+Tasterauslesungen rausgeworfen
//180117: fusioniertes Ü-Protokoll eingebaut (angefangen)
//180222: added scan mode
//180418: modified ADC-read for scanning, added UART-FIFO
//180xxx: SEL-Scan added, I2C functions changed (Andreas) Bug: SEL-Scan sends SEL-Pos only each 5.pos -> bei SEL zu schnell gescannt
//180xxx: Baudrate changed to 115200 (Andreas)
//1908ff: SEL-scan-Bug-fixing
//          -Datenkonvertierung von MAX9611 korrigiert
//          -Wartedelay in SEL-Fall invertiert (hat vorher gewartet, wenn kein SEL)
//          -Init nach Scanende zerstört UART-Verbindung
//          -UART-Debugstrings entfernt (waren ohne Protokollbezug und zerstören Datenverbindung)
//          -delayms() Aufrufe durch SysTick ersetzt
//191113: Multiscan "asm" implementiert, scan-Funktionsteile(init+step) ausgelagert
//200123: ?
//200517:   "slf" "slp" in UART korrigiert (waren vermischt)
//          timings in SEE_puls() vergroessert (Verhalten von "alp" war nicht richtig -viele Pulse und I nicht immer zurueckgesetzt)
//200531:   -MAX9611-Doppelinit korrigiert -> Comp-Mode (ohne 1ms-Delay funzt jetzt)
//          -DAC MCP4728 schnelle Aktualisierung (cmd SingleWrite(30ms, wegen EEPROM-write) -> MultiWrite)
//2006xx: use DUT-voltage as SEL-Detection while scanning
//200828: added commands "gdc" and "gdv" (GetDutCurrent, GetDutVoltage)
//210525: MAX9611 & SELDelay -I2C-Disabled
//211021: MAX9611 & SELDelay -I2C-Enabled and optional disable-#defines implemented
//211022: MCP4728 umkonfiguriert (Vref=Int,Gain=2) (bisher war Vdd=Vref)


//to disable I2C-communication: (for testing with LaunchPad without I2C-slaves)
//#define MCP4728DISABLED
//#define MAX9611DISABLED

#include <msp430.h>     //  /usr/lib/gcc/msp430/4.6.3/plugin/include/config/msp430/msp430.h     /usr/msp430/include
#include <stdint.h>
#include <stdbool.h>
void enablePulsGen(unsigned short freq);    //----- TimerA0-Init (Pulsgen)------
void disablePulsGen();
void MCP4728_FastWrite(unsigned short chan0, unsigned short chan1, unsigned short chan2, unsigned short chan3);
void MCP4728_MultiWrite(unsigned char channel, unsigned short data);
void delay(unsigned short);
void uart0_receive(unsigned char data);     //analyzes received UART data
void UART_TX(char TX);      //Byte über UART senden
void initialize(void);                      // Configure modules and control Registers
void readMultiADC();
void readSingleADC();
void SEE_Puls();    //einzelnen Puls mit anderer Intensitaet rausballern
void SendACK();
void SendERR(char message[]);
unsigned short readMAX9611_current();
unsigned short readMAX9611_voltage();

//Andreas - define - begin
#define VOLTAGE_CALIBRATION_CONSTANT        1.372
#define CURRENT_CALIBRATION_CONSTANT_Gain1  0.315
#define CURRENT_CALIBRATION_CONSTANT_Gain4  0.315   //not tested
#define CURRENT_CALIBRATION_CONSTANT_Gain8  0.315   //not tested

#define MCP4728_ADDR  0x60  //DAC I2C ADRESS
#define MAX9611_ADDR  0xFF  //CSM I2C ADRESS
#define MSP430_SLAVE_ADDR  0x48  //Latchup delay timer
#define AD7997_ADR    0x20  //Adresse von I2C-ADC (unused)

#define CMD_TYPE_0_MASTER      3
#define CMD_TYPE_1_MASTER      4
#define CMD_TYPE_2_MASTER      5

#define TYPE_0_LENGTH   1
#define TYPE_1_LENGTH   2
#define TYPE_2_LENGTH   3

#define MAX_BUFFER_SIZE     20

//MAX9611 data register
#define CMD_CSAMSB_SLAVE        0x00
#define CMD_CSALSB_SLAVE        0x01
#define CMD_RSMSB_SLAVE         0x02
#define CMD_RSLSB_SLAVE         0x03
#define CMD_OUTMSB_SLAVE        0x04
#define CMD_OUTLSB_SLAVE        0x05
#define CMD_SETMSB_SLAVE        0x06
#define CMD_SETLSB_SLAVE        0x07
#define CMD_TEMPMSB_SLAVE       0x08
#define CMD_TEMPLSB_SLAVE       0x09
#define CMD_CTL1_SLAVE          0x0A
#define CMD_CTL2_SLAVE          0x0B

//MAX9611 control register
uint8_t CR1_CSA_Gain1       [TYPE_0_LENGTH] = {0x00}; //Read CSA output from ADC, gain = 1x
uint8_t CR1_CSA_Gain1_COMP  [TYPE_0_LENGTH] = {0xE0}; //Read CSA output from ADC, gain = 1x
uint8_t CR1_CSA_Gain4       [TYPE_0_LENGTH] = {0x01}; //Read CSA output from ADC, gain = 4x
uint8_t CR1_CSA_Gain4_COMP  [TYPE_0_LENGTH] = {0xE1}; //Read CSA output from ADC, gain = 4x
uint8_t CR1_CSA_Gain8       [TYPE_0_LENGTH] = {0x02}; //Read CSA output from ADC, gain = 8x
uint8_t CR1_CSA_Gain8_COMP  [TYPE_0_LENGTH] = {0xE2}; //Read CSA output from ADC, gain = 8x
uint8_t CR1_RS              [TYPE_0_LENGTH] = {0x03}; //Read average voltage of RS+ (input common-mode voltage) from ADC
uint8_t CR1_FRM             [TYPE_0_LENGTH] = {0x07}; //Read all channels in fast-read mode, sequentially every 2ms. Uses last gain setting
uint8_t CR1_FRM_COMP        [TYPE_0_LENGTH] = {0xE7}; //Read all channels in fast-read mode, sequentially every 2ms. Uses last gain setting
uint8_t CR1_TEMP            [TYPE_0_LENGTH] = {0x06}; //Read internal die temperature from ADC
uint8_t CR2                 [TYPE_0_LENGTH] = {0x00}; //MAX9611-WDT settings

//Andreas - define - end

#define LEDb            (BIT2)      // green LEDb on P2.2
#define LEDg            (BIT1)      // blue  LEDg on P2.1
#define PULSOUT         (BIT3)      //Ausgang zum Pulsgenerator
#define Y_STEP          (BIT5)      //Ausgang zum Y-Schrittmotor
#define Y_DIR           (BIT4)      //Ausgang zum Y-Schrittmotor
#define INTENSITY       (BIT4)      //Input from detector of reflected light (original: diff. bitstream signal)

#define POSYMAX         0x0000ffff  //maximale Y-Position

#define I2C_ack         0
#define I2C_nack        1
#define Address         0
#define DEVICE_CODE     0xC0U

#define GENERAL_CALL    0x00U
#define RESET           0x06U
#define WAKEUP          0x09U
#define UPDATE          0x08U
#define READADDRESS     0x0CU
#define READRESTART     0xC1U

#define FASTWRITE       0x00U   // update input DAC registers from channels A to D sequentially
#define MULTIWRITE      0x40U   // write DAC input register, one at a time
#define SEQWRITE        0x50U   // writes DAC input registers and EEPROM sequentially from starting ch.to D
#define SINGLEWRITE     0x58U   // writes selected single DAC input register and EEPROM

#define WRITEADDRESS    0x60U
#define WRITECURRENT    0x61U
#define WRITENEW        0x62U
#define WRITECONFIRM    0x63U

#define WRITEVREF       0x80U
#define WRITEPD         0xA0U
#define WRITEGAIN       0xC0U
//----------------------------

/* unsigned short ADCdata[3];  //buffer for ADC-values
    unsigned long  SysTick=0;    //milliseconds counter
    unsigned short PulsFreq=1;  //Enable für Pulsgenerator
    unsigned short PosX=0x0800, PosZ=2240, PosT=0x0800;   //Linsenpositionsvariablen
    unsigned short PosY=2000, PosYIst=2000; //Y soll- und ist-Position
    unsigned short PosP=4096;   //SEE-Puls Intensity
    unsigned short PosF=100;    //Pulse freq
    unsigned short PosXmin=1000, PosXmax=1500, PosYmin=1000, PosYmax=1500, PosZmin=2100, PosZmax=2450,PosImin=2400; //range for scan mode  unsigned short PosXmin=2000, PosXmax=2100, PosYmin=2220, PosYmax=2400, PosZmin=2100, PosZmax=2450,PosImin=100; //range for scan mode
    unsigned short PosXstep=1, PosYstep=1, PosZstep=10,PosIstep=100, PosXdelay=100, PosYdelay=500 ;   //range for scan mode
    unsigned char uart_rx[4];       //buffer for received bytes
    unsigned short PosAltX, PosAltY, PosAltZ, PosAltT;  //previous positions
unsigned short PosAltI,PosI=2400,PosImax=3040;    //Laser-Intensity */



unsigned short ADCdata[3];  //buffer for ADC-values
unsigned long  SysTick=0;    //milliseconds counter
unsigned short PulsFreq=1;  //Enable für Pulsgenerator
unsigned short PosX=0x0800, PosZ=0x0800, PosT=0x0800;   //Linsenpositionsvariablen
unsigned short PosY=2000, PosYIst=2000; //Y soll- und ist-Position
unsigned short PosP=4096;   //SEE-Puls Intensity
unsigned short PosF=100;    //Pulse freq
unsigned short PosXmin=1000, PosXmax=1500, PosYmin=1000, PosYmax=1500, PosZmin=2100, PosZmax=2450,PosImin=3000; //range for scan mode
unsigned short PosXstep=1, PosYstep=1, PosZstep=10,PosIstep=100, PosXdelay=100, PosYdelay=500 ;   //range for scan mode
unsigned char uart_rx[4];       //buffer for received bytes
unsigned short PosAltX, PosAltY, PosAltZ, PosAltT;  //previous positions
unsigned short PosAltI,PosI=3000,PosImax=4095,PosI_mul_min=3000,PosI_mul_max=4095,PosImid=547;   //Laser-Intensity // New variables added for multi int and auto focus functions !



unsigned int steps_auto=0,SEE_FLAG=0,flage_rev=0,vol_limit=100;
unsigned char *I2C_PTxData;                     // Pointer to TX data
unsigned char I2C_TXByteCtr;
unsigned char I2C_TxData[8];    // Table of data to transmit

unsigned short max_adc_val, min_adc_val=0x03ff,avg_adc_val,best_adc_val; //AUTO FOCUS PRAMETER

enum { START='0', SET1='1', SET2='2', SET3='3', SET4='4', ACTION1='5', ACTION2='6', GET1='7', GET2='8' } uart_state;    //states of UART-state maschine
enum { MODE_MANUPOS='0', MODE_DEMO='1', MODE_SCANAREA='2', MODE_SCANPAUSE='3', MODE_SCANAREA_SEL='4', MODE_SCANAREA_SEL2='5', MODE_MULTIPULSE='6',  MODE_PARALLELSCAN='7',  MODE_MULTINSSCAN='8' ,  MODE_AUTOFOCUS='9' } mode;  //states of mode state maschine

//******************************************************************************
// General I2C State Machine ***************************************************
//******************************************************************************
typedef enum I2C_ModeEnum{
    IDLE_MODE,
    NACK_MODE,
    TX_REG_ADDRESS_MODE,
    RX_REG_ADDRESS_MODE,
    TX_DATA_MODE,
    RX_DATA_MODE,
    SWITCH_TO_RX_MODE,
    SWITHC_TO_TX_MODE,
    TIMEOUT_MODE
} I2C_Mode;

/* Used to track the state of the software state machine*/
I2C_Mode MasterMode = IDLE_MODE;

/* The Register Address/Command to use*/
uint8_t TransmitRegAddr = 0;

/* ReceiveBuffer: Buffer used to receive data in the ISR
    * RXByteCtr: Number of bytes left to receive
    * ReceiveIndex: The index of the next byte to be received in ReceiveBuffer
    * TransmitBuffer: Buffer used to transmit data in the ISR
    * TXByteCtr: Number of bytes left to transfer
    * TransmitIndex: The index of the next byte to be transmitted in TransmitBuffer
* */
uint8_t ReceiveBuffer[MAX_BUFFER_SIZE] = {0};
uint8_t RXByteCtr = 0;
uint8_t ReceiveIndex = 0;
uint8_t TransmitBuffer[MAX_BUFFER_SIZE] = {0};
uint8_t TXByteCtr = 0;
uint8_t TransmitIndex = 0;

/* I2C Write and Read Functions */
void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count);

/* For slave device with dev_addr, read the data specified in slaves reg_addr.
    * The received data is available in ReceiveBuffer
    *
    * dev_addr: The slave device address.
    *           Example: MAX9611_ADDR
    * reg_addr: The register or command to send to the slave.
    *           Example: CMD_TYPE_0_SLAVE
    * count: The length of data to read
    *           Example: TYPE_0_LENGTH
*  */
I2C_Mode I2C_Master_ReadReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t count)
{   /* Initialize state machine */
    MasterMode = TX_REG_ADDRESS_MODE;
    TransmitRegAddr = reg_addr;
    RXByteCtr = count;
    TXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;

    /* Initialize slave address and interrupts */
    UCB0I2CSA = dev_addr;
    IFG2 &= ~(UCB0TXIFG + UCB0RXIFG);       // Clear any pending interrupts
    IE2 &= ~UCB0RXIE;                       // Disable RX interrupt
    IE2 |= UCB0TXIE;                        // Enable TX interrupt

    UCB0CTL1 |= UCTR + UCTXSTT;             // I2C TX, start condition
    __bis_SR_register(CPUOFF + GIE);              // Enter LPM0 w/ interrupts

    return MasterMode;
}

/* For slave device with dev_addr, writes the data specified in *reg_data
    *
    * dev_addr: The slave device address.
    *           Example: MAX9611_ADDR
    * reg_addr: The register or command to send to the slave.
    *           Example: CMD_TYPE_0_MASTER
    * *reg_data: The buffer to write
    *           Example: MasterType0
    * count: The length of *reg_data
    *           Example: TYPE_0_LENGTH
*  */
I2C_Mode I2C_Master_WriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{
    /* Initialize state machine */
    MasterMode = TX_REG_ADDRESS_MODE;
    TransmitRegAddr = reg_addr;

    //Copy register data to TransmitBuffer
    CopyArray(reg_data, TransmitBuffer, count);

    TXByteCtr = count;
    RXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;

    /* Initialize slave address and interrupts */
    UCB0I2CSA = dev_addr;
    IFG2 &= ~(UCB0TXIFG + UCB0RXIFG);       // Clear any pending interrupts
    IE2 &= ~UCB0RXIE;                       // Disable RX interrupt
    IE2 |= UCB0TXIE;                        // Enable TX interrupt

    UCB0CTL1 |= UCTR + UCTXSTT;             // I2C TX, start condition
    __bis_SR_register(CPUOFF + GIE);              // Enter LPM0 w/ interrupts

    return MasterMode;
}

void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count)
{
    uint8_t copyIndex = 0;
    for (copyIndex = 0; copyIndex < count; copyIndex++)
    {   dest[copyIndex] = source[copyIndex];
    }
}
//---------- UART FIFO -------------
#define FIFO_MEM_SIZE   8
#define FIFO_MASK       FIFO_MEM_SIZE - 1
#define uint8_t unsigned char
typedef struct
{   uint8_t read;
    uint8_t write;
}FIFO_CTRL_t;
static FIFO_CTRL_t fifo_ctrl;
static uint8_t fifo_mem[FIFO_MEM_SIZE];

void fifo_init();
uint8_t fifo_add(uint8_t chr);
uint8_t fifo_get(uint8_t *chr);

void fifo_init()
{
    fifo_ctrl.read = 0;
    fifo_ctrl.write = 0;
}

uint8_t fifo_add(uint8_t chr)
{
    if((fifo_ctrl.write & FIFO_MASK) != (fifo_ctrl.read & FIFO_MASK)
    || fifo_ctrl.write == fifo_ctrl.read)
    {
        fifo_mem[fifo_ctrl.write & FIFO_MASK] = chr;
        fifo_ctrl.write++;
        return 0;
    }
    return 1;
}

uint8_t fifo_get(uint8_t *chr)
{   if(fifo_ctrl.read != fifo_ctrl.write)
    {
        *chr = fifo_mem[fifo_ctrl.read & FIFO_MASK];
        fifo_ctrl.read++;
        return 0;
    }
    return 1;
}
//---------- UART FIFO end -------------

//Andreas - fun - begin
void delayms(unsigned int ms)
{   unsigned int i = 0;
    for (i = 0; i < ms; ++i)
    {   delay(1600);
    }
}

void initMAX9611()
{
    #ifdef MAX9611DISABLED
        return;
    #endif
    I2C_Master_WriteReg(MAX9611_ADDR, CMD_CTL1_SLAVE, CR1_CSA_Gain1, TYPE_0_LENGTH);  //Set Gain
    delayms(100);
    I2C_Master_WriteReg(MAX9611_ADDR, CMD_CTL1_SLAVE, CR1_FRM, TYPE_0_LENGTH);    //Set Channel
    delayms(100);
    I2C_Master_WriteReg(MAX9611_ADDR, CMD_CTL2_SLAVE, CR2, TYPE_0_LENGTH);    //Set MAX9611 WDT-Settings
    delayms(100);
    I2C_Master_ReadReg(MAX9611_ADDR, CMD_CSAMSB_SLAVE, 12);
    delayms(10);
}

unsigned short readMAX9611_current()    //Read current and return mA
{
    #ifdef MAX9611DISABLED
        return 0x5A;
    #endif
    unsigned short current;
    I2C_Master_ReadReg(MAX9611_ADDR, CMD_CSAMSB_SLAVE, 2); //read first two registers from MAX9611
    current = (((unsigned short)ReceiveBuffer[1] & 0xFF) >> 4) | (((unsigned short)ReceiveBuffer[0] & 0xFF) << 4); //shift registers to unsigned short
    current = current*CURRENT_CALIBRATION_CONSTANT_Gain1; //with calibration
    //UART_TX((unsigned char)(current & 0xFF) );    //send measured current for debugging
    return current;
}

unsigned short readMAX9611_voltage()    //Read voltage and returns value in 10mV
{
    #ifdef MAX9611DISABLED
        return 0xA5;
    #endif
    unsigned short voltage;
    I2C_Master_ReadReg(MAX9611_ADDR, CMD_RSMSB_SLAVE, 2); //read first two registers from MAX9611
    voltage = (((unsigned short)ReceiveBuffer[1] & 0xFF) >> 4) | (((unsigned short)ReceiveBuffer[0] & 0xFF) << 4); //shift registers to unsigned short
    voltage = voltage*VOLTAGE_CALIBRATION_CONSTANT; //with calibration
    //UART_TX((unsigned char)(current & 0xFF) );    //send measured voltage for debugging
    return voltage;
}
#define MCP4728WriteVref    (1<<7)
#define MCP4728WritePowerDown   (1<<7)+(1<<5)
#define MCP4728WriteGainSelect  (1<<7)+(1<<6)

void initMCP4728()  //initialize DAC -nicht gebraucht, wird bei jeder Wertaktualisierung (nur MultiWrite) mit übergeben
{   I2C_Master_WriteReg(MCP4728_ADDR, MCP4728WriteVref + 0x0F, 0, 0);  //Set Vref=2,048V intern

    //I2C_Master_WriteReg(MCP4728_ADDR, MCP4728WriteVref + 0x00, 0, 0);  //Set Vref=Vcc
    //I2C_Master_WriteReg(MCP4728_ADDR, MCP4728WriteGainSelect + 0x00, 0, 0);  //Set Gain=1
    I2C_Master_WriteReg(MCP4728_ADDR, MCP4728WriteGainSelect + 0x0F, 0, 0);  //Set Gain=2
    //delayms(100);
}

// Schreibt Werte in alle 4 DAC-Kanäle
//Vref+Gain wird hier nicht mit übergeben
//MCP4728_FastWrite(chan0, chan1, chan2, chan3);
//MCP4728_FastWrite(Y, Z, T, X);
void MCP4728_FastWrite(unsigned short chan0, unsigned short chan1, unsigned short chan2, unsigned short chan3)
{
    #ifdef MCP4728DISABLED
        return;
    #endif
    I2C_TxData[0]=(FASTWRITE | ((unsigned char)((chan0 >> 8) & 0x3F)));
    I2C_TxData[1]=(FASTWRITE |  (unsigned char)(chan0 & 0x00FF));
    I2C_TxData[2]=(FASTWRITE | ((unsigned char)((chan1 >> 8) & 0x3F)));
    I2C_TxData[3]=(FASTWRITE |  (unsigned char)(chan1 & 0x00FF));
    I2C_TxData[4]=(FASTWRITE | ((unsigned char)((chan2 >> 8) & 0x3F)));
    I2C_TxData[5]=(FASTWRITE |  (unsigned char)(chan2 & 0x00FF));
    I2C_TxData[6]=(FASTWRITE | ((unsigned char)((chan3 >> 8) & 0x3F)));
    I2C_TxData[7]=(FASTWRITE |  (unsigned char)(chan3 & 0x00FF));
    //I2C_write(0x60, I2C_TxData, 8); //(Adresse, Datenarray, Laenge)
    I2C_Master_WriteReg(MCP4728_ADDR, 0x00, I2C_TxData, 8);
}

#define MCP4728RefInt   0x80
#define MCP4728RefVdd   0x00
#define MCP4728Gain1    0x00
#define MCP4728Gain2    0x10
// Schreibt neuen Wert in einen DAC-Kanal
//MCP4728_MultiWrite(kanal, wert);
void MCP4728_MultiWrite(unsigned char channel, unsigned short data)
{
    #ifdef MCP4728DISABLED
        return;
    #endif
    I2C_TxData[0]=((unsigned char)((data >> 8)& 0x0f)) + MCP4728RefInt + MCP4728Gain2;
    I2C_TxData[1]=((unsigned char)(data & 0x00FF));
    I2C_Master_WriteReg(MCP4728_ADDR, (MULTIWRITE | (channel << 1)), I2C_TxData, 2);
}

void AD7997_SingleRead(unsigned char channel)   //was never used yet
{   int adc_cmd = ((channel<<4) + 0x80) & 0xFF;
    I2C_Master_ReadReg(AD7997_ADR, adc_cmd, 1);
}

void initI2C()
{   UCB0CTL1 |= UCSWRST;                      // Enable SW reset
    UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;     // I2C Master, synchronous mode
    UCB0CTL1 = UCSSEL_2 + UCSWRST;            // Use SMCLK, keep SW reset
    UCB0BR0 = 40;                            // fSCL = SMCLK/160 = ~100kHz
    UCB0BR1 = 0;
    UCB0I2CSA = MAX9611_ADDR;                   // Slave Address
    UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
    UCB0I2CIE |= UCNACKIE;
    //-------- Folgendes aus Initialize() - I2C-Init -------------
    //UCB0CTL1 = UCSSEL_1 + UCSWRST;            // Use ACLK(32kHz), keep SW reset
    //UCB0BR0 = 32;                             // fSCL = SMCLK/12 = ~100kHz  (mit 100kHz bleibt er gelegentlich hängen!!!)
    //UCB0BR0 = 2;                             // fSCL = ACLK/1 = ~32kHz    ( >1 ! )
    P1SEL |= BIT6 + BIT7;                     // Assign I2C pins to USCI_B0
    P1SEL2|= BIT6 + BIT7;                     // Assign I2C pins to USCI_B0
    IE2 |= UCB0TXIE;                        // Enable TX ready interrupt
    IE2 |= UCB0RXIE;                        // Enable RX interrupt
}

//Andreas - fun - end

unsigned long setNextScanPos(void)   //move to next X-pos for scanning (called from main())
{   unsigned long  SysTickScanStep;
    SysTickScanStep = SysTick + PosXdelay;  //calculate and set time for next scan x-step
    PosX += PosXstep;                   //move to next X-pos
    if (PosX <= PosXmax)                //still in range of X
    {   MCP4728_MultiWrite(3, PosX);   //set X-pos
    }
    else                                //out of range in X
    {   PosX = PosXmin;                 //reset X-Position to min
        MCP4728_MultiWrite(3, PosX);   //set X-pos
        SysTickScanStep = SysTick + PosYdelay;  //set wait delay for Y
        PosY += PosYstep;               //move to next Y-pos

        if (PosY > PosYmax)             //whole picture scanned
        {   SysTickScanStep=0;
            mode=MODE_MANUPOS;
            enablePulsGen(PosF);        // Pulsgenerator anschalten Parameter: Frequenz(8Hz-500kHz(?))
        }
    }
    return SysTickScanStep;
}

unsigned long setNextScanPos_mul(void)   //move to next X-pos for scanning (called from main())
{   unsigned long  SysTickScanStep;
    if (PosI<PosImin){

        SysTickScanStep=0;
        mode=MODE_MANUPOS;
        enablePulsGen(PosF);        // Pulsgenerator anschalten Parameter: Frequenz(8Hz-500kHz(?))
    }

    SysTickScanStep = SysTick + PosXdelay;  //calculate and set time for next scan x-step
    PosX += PosXstep;                   //move to next X-pos
    if (PosX <= PosXmax)                //still in range of X
    {   MCP4728_MultiWrite(3, PosX);   //set X-pos
    }
    else                                //out of range in X
    {   PosX = PosXmin;                 //reset X-Position to min
        MCP4728_MultiWrite(3, PosX);   //set X-pos
        SysTickScanStep = SysTick + PosYdelay;  //set wait delay for Y
        PosY += PosYstep;               //move to next Y-pos
        if (PosY > PosYmax)             //whole picture scanned

        { PosI= PosI-PosIstep;
            MCP4728_MultiWrite(0, PosI);
            PosY=PosYmin;
            PosX=PosXmin;
        }
    }
    return SysTickScanStep;
}

unsigned long focus(void)    //Auto focus function implemented by pavan.. Need to update based on hardware changes !
{   unsigned long  SysTickScanStep;
    unsigned short reflectivity, voltage;

    switch (steps_auto)

    {

        case 0:
        PosX = PosXmin;
        PosY=PosYmin;
        break;
        case 1:
        PosX = PosXmin;
        PosY=PosYmax;
        break;
        case 2:
        PosX = PosXmax;
        PosY=PosYmax;
        break;
        case 3:
        PosX = PosXmax;
        PosY=PosYmin;
        break;
        case 4:
        PosX = (PosXmax - PosXmin)/2;
        PosY= (PosYmax - PosYmin)/2;
        break;
        case 5:
        PosX = (PosXmax - PosXmin)/2;
        PosY= PosYmin;
        break;
        case 6:
        PosX = (PosXmax - PosXmin)/2;
        PosY= PosYmax;
        break;
        case 7:
        PosX = PosXmax;
        PosY= (PosYmax - PosYmin)/2;
        break;
        case 8:
        PosX =PosXmin;
        PosY= (PosYmax - PosYmin)/2;
        break;
        default:
        SysTickScanStep=0;
        mode=MODE_MANUPOS;
        enablePulsGen(PosF);
        min_adc_val=0x03ff;
        PosZ=best_adc_val;

        steps_auto=0;

        MCP4728_MultiWrite(1, best_adc_val);
        max_adc_val=0;
        return SysTickScanStep;
    }
    MCP4728_MultiWrite(3, PosX);

    SysTickScanStep = SysTick + PosXdelay;  //calculate and set time for next scan x-step
    PosZ += PosZstep;                   //move to next Z-pos

    if (PosZ <= PosZmax)                //still in range of X
    {   MCP4728_MultiWrite(1, PosZ);   //set X-pos
    }
    else                                //out of range in X
    {   PosZ = PosZmin;                 //reset X-Position to min
        MCP4728_MultiWrite(1, PosZ);   //set X-pos
        steps_auto++;
    }


    P2OUT |= PULSOUT;                   // P2.3 high
    P2OUT &= ~PULSOUT;                  // P2.3 low
    readSingleADC();                    // read out internal ADC to get reflectivity value
    delayms(12);    // delay removed for fast scanning (!!! no current mesurement after latchup possible !!!)
    //SysTickScanStep = SysTick + 10;     //wait for new adc conversion (2*2ms(conv time)->async->nyquist) and register transfer complete
    reflectivity = 0x03ff - ADC10MEM;   //invert reflectivity value from ADC
    voltage = readMAX9611_voltage(); //read voltage from DUT and returns value in 10mV
    if (reflectivity < min_adc_val){
        min_adc_val=reflectivity;
    }
    if (reflectivity > max_adc_val){
        max_adc_val = reflectivity;
        best_adc_val=PosZ;
    }
    avg_adc_val = (max_adc_val - min_adc_val)/2;

    UART_TX((unsigned char)(reflectivity >> 8));  //transmit MSB of refletivity
    UART_TX((unsigned char)reflectivity);  //transmit LSB of reflectivity
    //  UART_TX((unsigned char)(voltage >> 8)); //transmit MSB
    //UART_TX((unsigned char)voltage);        //transmit LSB
    UART_TX((unsigned char)(avg_adc_val >> 8)); //transmit MSB
    UART_TX((unsigned char)avg_adc_val);        //transmit LSB
    //UART_TX((unsigned char)(PosZ >> 8)); //transmit MSB
    //UART_TX((unsigned char)PosZ);        //transmit LSB
    UART_TX((unsigned char)(best_adc_val >> 8)); //transmit MSB
    UART_TX((unsigned char)best_adc_val);        //transmit LSB
    //UART_TX((unsigned char)'-');
    return SysTickScanStep;
}

int main(void)
{
    unsigned long  SysTickScanStep=0;
    unsigned char data; //buffer for received data

    // Initialize system +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    initialize();
    initI2C();
    initMAX9611();  //initialize current-sense-monitor
    //initMCP4728();    //initialize DAC

    //for testing only: init SEL-Delay
    //  uart_rx[1]='u';    I2C_TxData[0]=0;    I2C_TxData[1]=4;
    //  I2C_Master_WriteReg(MSP430_SLAVE_ADDR, uart_rx[1], I2C_TxData, TYPE_1_LENGTH);

    P2OUT ^= LEDb;              // Toggle P1.0 using exclusive-OR
    P2OUT ^= LEDg;
    P2OUT &= ~LEDg;             // LED off
    // Program begin
    mode=MODE_MANUPOS;
    uart_state=START;           // zurück in Grundzustand
    enablePulsGen(PosF);        // Pulsgenerator anschalten Parameter: Frequenz(8Hz-500kHz(?))
    //unsigned short cnt=4095;
    while(1)
    {   /*if(!(SysTick%1000))
        {   MCP4728_MultiWrite(0,(cnt));// only for testing DAC (channel, data)<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        MCP4728_MultiWrite(1,(cnt));// only for testing DAC (channel, data)<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        MCP4728_MultiWrite(2,(cnt));// only for testing DAC (channel, data)<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        MCP4728_MultiWrite(3,(cnt));// only for testing DAC (channel, data)<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        //          cnt+=50;
        if(cnt>4095) cnt=0;
    }*/
    //readMAX9611_current();
    if (!fifo_get(&data))   //check if fifo-buffer not empty and read 1 byte
    {   uart0_receive(data);//empfangenes Zeichen auswerten
    }
    if(!(SysTick%1000))
    {   P2OUT |= LEDb;      // LED on
        //I2C_Master_WriteReg(MSP430_SLAVE_ADDR, uart_rx[1], I2C_TxData, TYPE_1_LENGTH);
        //delayms(1);
    }
    if((SysTick%1000)==64)
    {   P2OUT &= ~LEDb;     // LED off
    }


    if((PosAltX!=PosX) || (PosAltY!=PosY) || (PosAltI!=PosI) || (PosAltT!=PosT) || (PosAltZ!=PosZ)) //bei Positionsänderung
    {   P2OUT ^= LEDb;                          // Toggle LED when something changed
        PosAltX=PosX; PosAltY=PosY; PosAltI=PosI; PosAltT=PosT; PosAltZ=PosZ;   //alte Werte neu beschreiben
    }
    if((mode == MODE_MULTIPULSE) && (SysTick >= SysTickScanStep))       //Multi Pulse Mode
    {   SysTickScanStep = SysTick + PosXdelay;  //calculate delay
        if (PosXmax--)  //if still pulses to generate
        {   P2OUT |= PULSOUT;           // P2.3 high
            P2OUT &= ~PULSOUT;          // P2.3 low
        }
        else                //all pulses done
        {   mode = MODE_MANUPOS;
            MCP4728_MultiWrite(0,PosI);// back to previous intensity (channel, data)
            enablePulsGen(PosF);    //start pulse generator
            SendACK();          //Bestaetigung antworten
        }
    }
    if((mode == MODE_SCANAREA) && (SysTick >= SysTickScanStep))     //Surface Scan Modus
    {   P2OUT |= PULSOUT;                   // P2.3 high
        P2OUT &= ~PULSOUT;                  // P2.3 low
        readSingleADC();                    // read out internal ADC to get intensity value
        UART_TX(0xff-(char)(ADC10MEM>>2));  //ADCdata[0]);    //send PixelIntensityData (from ADC channel 4)
        SysTickScanStep = setNextScanPos();                   //move to next X-pos and set new waiting-time
    }
    if((mode == MODE_SCANAREA_SEL) && (SysTick >= SysTickScanStep))     //SEL Scan Modus
    {   P2OUT |= PULSOUT;                   // P2.3 high
        P2OUT &= ~PULSOUT;                  // P2.3 low
        mode = MODE_SCANAREA_SEL2;

        SysTickScanStep = SysTick + 10;     //wait for new adc conversion (2*2ms(conv time)->async->nyquist) and register transfer complete

        unsigned short current;
        mode = MODE_SCANAREA_SEL;
        current = readMAX9611_current(); //read current from DUT and returns value in mA

        UART_TX((unsigned char)(current >> 8)); //transmit MSB
        UART_TX((unsigned char)current);        //transmit LSB

        //current = current / 2;
        //if(current>0xff){current=0xff;}   //limit to 8bit
        //UART_TX( (unsigned char)(current & 0xFF) );    //send measured current
        SysTickScanStep=setNextScanPos();       //move to next X-pos and set new waiting-time
        if(current>13) //wait if an SEL occours;    //Todo: make this dynamic/coupled with SEL-Delay!
        {
            SysTickScanStep+=1000;  //increase Step delay in case of Latchup (time to recover is around 800ms in ATtiny20)
        }
    }
    if((mode == MODE_PARALLELSCAN) && (SysTick >= SysTickScanStep))     //Parallel Scan Modus
    {   unsigned short reflectivity, current, voltage;
        current = readMAX9611_current(); //read current from DUT and returns value in mA
        P2OUT |= PULSOUT;                   // P2.3 high
        P2OUT &= ~PULSOUT;                  // P2.3 low
        readSingleADC();                    // read out internal ADC to get reflectivity value
        delayms(12);    // delay removed for fast scanning (!!! no current mesurement after latchup possible !!!)
        //SysTickScanStep = SysTick + 10;     //wait for new adc conversion (2*2ms(conv time)->async->nyquist) and register transfer complete
        reflectivity = 0x03ff - ADC10MEM;   //invert reflectivity value from ADC
        voltage = readMAX9611_voltage(); //read voltage from DUT and returns value in 10mV

        /*   if((0x08 & P1IN))
            voltage = 0;
            else
        voltage = 999;*/


        UART_TX((unsigned char)(reflectivity >> 8));  //transmit MSB of refletivity
        UART_TX((unsigned char)reflectivity);  //transmit LSB of reflectivity
        UART_TX((unsigned char)(current >> 8)); //transmit MSB
        UART_TX((unsigned char)current);        //transmit LSB
        UART_TX((unsigned char)(voltage >> 8)); //transmit MSB
        UART_TX((unsigned char)voltage);        //transmit LSB

        SysTickScanStep = setNextScanPos();     //move to next X-pos and set new waiting-time
        //if(current>13) //wait if an SEL occours;  //Todo: make this dynamic/coupled with SEL-Delay!
        if(voltage<100) //wait if an SEL occours (after SEL DUT-Vcc is switched off);   //Todo: make this dynamic/coupled with SEL-Delay!
        {
            SysTickScanStep+=40;  //increase Step delay in case of Latchup (time to recover is around 800ms in ATtiny20)
        }
    }

    if((mode == MODE_MULTINSSCAN) && (SysTick >= SysTickScanStep))     //Multi Intensity Scan algorithm implemented by pavan

    {
        unsigned short reflectivity, current, voltage;
        // uint16_t reflectivity, current, voltage_mul;
        current = readMAX9611_current(); //read current from DUT and returns value in mA
        P2OUT |= PULSOUT;                   // P2.3 high
        P2OUT &= ~PULSOUT;                  // P2.3 low
        readSingleADC();                    // read out internal ADC to get reflectivity value
        delayms(12);    // delay removed for fast scanning (!!! no current measurement after latchup possible !!!)
        reflectivity = 0x03ff - ADC10MEM;   //invert reflectivity value from ADC
        voltage = readMAX9611_voltage(); //read voltage from DUT and returns value in 10mV


        /* Binary search */

        if((PosI_mul_max - PosI_mul_min) <= PosIstep){

        flage_rev=0;
            goto send;
        }
        if((PosI_mul_max == PosImid) || (PosI_mul_min == PosImid)){

        flage_rev=0;
            goto send;
        }

        if((voltage<vol_limit)&&(flage_rev==1)){

            // PosImax = PosI;
            //flage_rev=0;         // Debugging
            //goto send;
            PosI_mul_max=PosImid;
        PosImid=((PosI_mul_max - PosI_mul_min)/ 2) + PosI_mul_min;
        PosI=PosImid;
        MCP4728_MultiWrite(0, PosI);
        }

        if((voltage>=vol_limit)&&(flage_rev==1)){
        /*  if(PosI<PosImax){
                PosI= PosI+PosIstep;                    // Debugging
            MCP4728_MultiWrite(0, PosI);}
            else{
                flage_rev=0;
                goto send;
        }*/

        PosI_mul_min=PosImid;
        PosImid=(PosI_mul_max - PosI_mul_min)/ 2;
        PosI=PosImid;
        MCP4728_MultiWrite(0, PosI);
        }

        if((voltage<vol_limit)&&(flage_rev==0)) //Checking the latchup condition

        //voltage<100,( SEE_FLAG>1  add SEE_FLAG=0),current>13, 0x08 & P1IN    // various conditions to check for SEL
        {

            //PosI_mul_max=PosImax;
            //PosI_mul_min=PosImin;

            PosImid=(PosImax - PosImin)/ 2;
            flage_rev=1;
            PosI=PosImid;
            MCP4728_MultiWrite(0, PosI);
            //PosImax = PosI;
            // Practically, when the latchup occurs, it should send Intensity,Voltage and current here

        }

        send:
        if(flage_rev==0){
            UART_TX((unsigned char)(reflectivity >> 8));  //transmit MSB of reflectivity
            UART_TX((unsigned char)reflectivity);  //transmit LSB of reflectivity

            UART_TX((unsigned char)(PosI >> 8));   //transmit MSB of intensity
            UART_TX((unsigned char)PosI);          //transmit LSB of intensity

            UART_TX((unsigned char)(current >> 8));  // transmit MSB of current
            UART_TX((unsigned char)current);         // transmit LSB of current

            UART_TX((unsigned char)(voltage >> 8));  //transmit MSB of voltage
            UART_TX((unsigned char)voltage);        //transmit LSB of voltage
            SysTickScanStep = setNextScanPos();   // Here it moves to the next scan position
            // PosI=PosImin;                         // Here The Intensity is again brought back to minimum for the next scan position
            SysTickScanStep+=40;
            //PosI= PosI+PosIstep;                    // Intensity Increment
            //MCP4728_MultiWrite(0, PosI);
            PosI=PosImax;
            MCP4728_MultiWrite(0, PosI);
        }





    }

    if((mode == MODE_AUTOFOCUS) && (SysTick >= SysTickScanStep))     //Auto Focus mode implemented by pavan
    {
        SysTickScanStep = focus();      //move to next X-pos and set new waiting-time

    }
    }
}

void SEE_Puls()                 //einzelnen Puls mit anderer Intensitaet
{   disablePulsGen();
    P2OUT ^= LEDg;              // toggle LED
    P2SEL &= ~PULSOUT;          // P2.3 option PIO select
    P2OUT |= PULSOUT;           // P2.3 high
    delayms(1);
    MCP4728_MultiWrite(0,PosP);// SEE-Puls Intensität setzen (channel, data)
    delayms(30);                 //hier is ne Pause notwendig, zwischen 2*SingleWrite wird mind delay(2000) benötigt (veraltet?! (wegen SingleWrite))
    P2OUT &= ~PULSOUT;          // P2.3 low => trigger pulse
    delayms(2);                 //hier is ne Pause notwendig delayms(2);
    MCP4728_MultiWrite(0,PosI);// zurück auf alte Intensität (channel, data)
    delayms(30);                    //delayms(1);
    enablePulsGen(PosF);        //-----TimerA0-Init (Pulsgen)------
    P2OUT ^= LEDg;              // toggle LED
}
void UART_TX(char TX)
{   while (!(IFG2&UCA0TXIFG)); UCA0TXBUF = TX;                    // TX character
}

void initialize(void)   // setup ports, Timers, and variables - run once at start
{   volatile int i;
    WDTCTL = WDTPW + WDTHOLD;                 // Stop Watchdog Timer
    //----- PIO-Init ------
    P2DIR |= LEDb+LEDg;         // Set LED to output direction
    P2DIR |= Y_STEP+Y_DIR;      // Set Y-Achsensteuerpins
    //  P2SEL  &=~(BIT6+BIT7);  //Quarz-Pins als PIO konfigurieren (G2553.pdf S54)
    //  P2SEL2 &=~(BIT6+BIT7);  //Quarz-Pins als PIO konfigurieren (G2553.pdf S54)
    //  P2DIR |= BIT6+BIT7;     //Quarz-Pins als PIO konfigurieren (G2553.pdf S54)

    //  P2REN |= TASTER1+TASTER2;   // Enable Pull Up/Down
    //  P2OUT |= TASTER1+TASTER2;           // Enable Pull Up??? Ändert nichts
    //---------- init CLK -----------
    if (CALBC1_16MHZ==0xFF)         // If calibration constant erased
    {   while(1)                    // stop program and start panic-blink
        {   P2OUT |= LEDb + LEDg;       // LED on
            delay(2000);                // short wait
            P2OUT &= ~(LEDb + LEDg);    // LED off
            delay(2000);                // short wait
        }
    }
    DCOCTL = 0;                               // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_16MHZ;                    // Set DCO for 16MHz
    DCOCTL = CALDCO_16MHZ;
    //---------- init UART -----------
    P1SEL |= BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
    P1SEL2 |= BIT1 + BIT2 ;                    // P1.1 = RXD, P1.2=TXD
    UCA0CTL1 |= UCSSEL_2;                     // use SMCLK for UART
    UCA0BR0 = 138;                             // 16MHz 115200Baud
    UCA0BR1 = 0;
    UCA0MCTL = UCBRS2 + UCBRS0;               // Modulation UCBRSx = 5
    UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    IE2 |= UCA0RXIE;                // Enable USCI_A0 RX interrupt
    //-------- I2C-Init -------------
    P1SEL |= BIT6 + BIT7;                     // Assign I2C pins to USCI_B0
    P1SEL2|= BIT6 + BIT7;                     // Assign I2C pins to USCI_B0
    UCB0CTL1 |= UCSWRST;                      // Enable SW reset
    UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;     // I2C Master, synchronous mode
    UCB0CTL1 = UCSSEL_2 + UCSWRST;            // Use SMCLK, keep SW reset
    //UCB0CTL1 = UCSSEL_1 + UCSWRST;            // Use ACLK(32kHz), keep SW reset
    UCB0BR0 = 32;                             // fSCL = SMCLK/12 = ~100kHz  (mit 100kHz bleibt er gelegentlich hängen!!!)
    //UCB0BR0 = 2;                             // fSCL = ACLK/1 = ~32kHz    ( >1 ! )
    UCB0BR1 = 0;
    UCB0I2CSA = 0x60;                       // Set slave address
    UCB0CTL1 &= ~UCSWRST;                   // Clear SW reset, resume operation
    IE2 |= UCB0TXIE;                        // Enable TX ready interrupt
    IE2 |= UCB0RXIE;                        // Enable RX interrupt
    //----------- ADC-Init von msp430g2x33_adc10_10.c
    ADC10AE0 |= /*BIT0+BIT3+*/BIT4;//+BIT5;     // ADC option select = Analog (Input) Enable Control
    //  ADC10CTL1 = INCH_4 + CONSEQ_1;                      //(sequence-mode) highest Channel in Sequence, single sequence
    //  ADC10CTL0 = ADC10SHT_2 + MSC + ADC10ON + ADC10IE;   //(sequence-mode)
    //  ADC10DTC1 = 2;                          // 2 conversions (=Kanäle)
    ADC10CTL0 = /*SREF_1 + REFON + */ADC10SHT_2 + ADC10ON;// + ADC10IE; // (1chan-mode) ADC10ON, interrupt enabled
    ADC10CTL1 = INCH_4;                         // (1chan-mode) input A4 ( = INTENSITY)
    //----- SysTick-Timer-Init ------
    TA0CCTL0 = CCIE;                             // CCR0 interrupt enabled
    //  TA0CCR0 = 1000-1;   //10000 = 100ms pro Systick
    //  TA0CTL = TASSEL_2 + MC_1;                  // SMCLK(1MHz), upmode
    CCR0 = 33-1;    //33 =~1ms 32768 =1s pro Systick
    TACTL = TASSEL_1 + MC_1;                  // ACLK(32kHz), upmode
    //-----------p1.3 interrupt-init ------
    P1DIR &= BIT3;
    P1IES |= BIT3;                             // P1.1 Hi/Lo edge
    P1IFG = 0;                                // Clear all P1 interrupt flags
    P1IE = BIT3;                              // P1.1 interrupt enabled
    P1IFG &=~BIT3;
    //PM5CTL0 &= ~LOCKLPM5;
    //--------
    __enable_interrupt();                     // Enable interrupts
}
//>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void enablePulsGen(unsigned short freq) //----- TimerA0-Init (Pulsgen)------
{   long period;
    P2DIR |= PULSOUT;                            // P2.3 output
    P2SEL |= PULSOUT;                            // P2.3 option select
    TA1CCTL0 = OUTMOD_4;                         // CCR0 toggle mode
    if(freq>10000) freq=10000;  //Maximalfrequent auf 10kHz begrenzen
    //  period=500000/(long)freq;   // freq in CCR0-Periodenwert umrechnen (TA1 mit SMCLK(1MHz))
    period=(32768/2)/(long)freq;    // freq in CCR0-Periodenwert umrechnen (TA1 mit ACLK(32kHz))
    if(period>0xffff)   period=0xffff;  // Wert auf 16bit begrenzen (500000/65535Hz ~7,62951Hz)
    TA1CCR0 = (short) period;   // 500-1=1kHz
    //  TA1CTL = TASSEL_2 + MC_1;                  // SMCLK(1MHz), upmode
    TA1CTL = TASSEL_1 + MC_1;                  // ACLK(32kHz), upmode
}
void disablePulsGen()
{   TA1CTL = MC_0;                  // disable timer (MC=0)
}
//>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void readMultiADC() //liest mehrere ADC-Kanäle auf einmal
{   ADC10CTL0 &= ~ENC;
    while (ADC10CTL1 & ADC10BUSY);          // Wait if ADC10 core is active
    ADC10SA = (short) &ADCdata[0];          // Data buffer start
    ADC10CTL0 |= ENC + ADC10SC;              // Sampling and conversion start
    __bis_SR_register(/*CPUOFF + */GIE);        // LPM0, ADC10_ISR will force exit
}
//>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//from msp430g2x33_adc10_01.c
void readSingleADC()    //reads 1 ADC-channel (=read reflected intensity)
{//  ADC10AE0 |= 0x02;                         // PA.1 ADC option select
    ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
    while(ADC10CTL1 & 0x01);//ADC10BUSY);   // wait until conversion completed
    //__bis_SR_register(CPUOFF + GIE);      // LPM0, ADC10_ISR will force exit
}
//>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<

#define NOP asm volatile("  nop");
void delay(unsigned short dauer)
{   while(dauer--) { NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP }
}

void init_scanStart(void)   //prepare to start scanning (called from uart0_receive() )
{   disablePulsGen();   //stop pulse generator
    P2SEL &= ~PULSOUT;  // set pulse-out pin to PIO-mode
    PosX = PosXmin;
    PosY = PosYmin; //initial position for scan mode
    MCP4728_MultiWrite(3, PosX);   //set X-pos (channel, data)
    UART_TX('d');
    UART_TX('a');
    UART_TX('t');
}
//----------------------------- UART stuff ---------------------------

// Meldung auf UART absetzen:
void SendACK()   //send Acknowledge
{    // # TODO: Fix real problem, delay is work around for not applying sent values
    delayms(100);
    UART_TX('a');
    UART_TX('c');
    UART_TX('k');
}
void SendERR(char message[])        //send error
{   char pos=0;
    UART_TX('e');
    UART_TX('r');
    UART_TX('r');
    while ((message[pos]!=0) && (pos<100))
    UART_TX(message[pos++]);
    UART_TX('\r');
    UART_TX('\n');
}

// Hier werden die empfangenen UART-Daten entsprechend Protokolldefinition ausgewertet:
void uart0_receive(unsigned char data)      //analyzes received UART data
{   unsigned short databuffer;              //buffer for received data
    switch (uart_state)         // Zustand der uart-Statemachine
    {   case START:         // Befehlsauswahl
        switch (data)       //empfangenen Befehl auswerten
        {   case 's':   uart_state=SET1;    break;  //set parameters
            case 'a':   uart_state=ACTION1; break;  //perform action
            case 'g':   uart_state=GET1;    break;  //get stuff from setup
            default :   SendERR("Wrong stuff received!");           //Fehler melden
        }
        break;
        case SET1:{  uart_rx[0]=data;    uart_state=SET2;    break;}  // read command
        case SET2:{  uart_rx[1]=data;    uart_state=SET3;    break;}  // read value high part
        case SET3:{  uart_rx[2]=data;    uart_state=SET4;    break;}  // read value low  part
        case SET4:{  uart_rx[3]=data;
            databuffer=((uart_rx[2]<<8)+uart_rx[3]);    //convert received databytes into 16bit value
            switch (uart_rx[0]) //select set command
            {   case 'x':   // set x-scan positions
                switch  (uart_rx[1])
                {   case 'l':   PosXmin=databuffer; break;  // set y_axis low
                    case 'h':   PosXmax=databuffer; break;  // set y_axis high
                    default:                //in case of other values
                    SendERR("SetPosition: wrong scan parameter!");      //send error
                    uart_state=START;       // zurück in Grundzustand
                }
                break;
                case 'z':   // set z-scan positions
                switch  (uart_rx[1])
                {   case 'l':   PosZmin=databuffer; break;  // set Z_axis low
                    case 'h':   PosZmax=databuffer; break;  // set Z_axis high
                    default:                //in case of other values
                    SendERR("SetPosition: wrong scan parameter!");      //send error
                    uart_state=START;       // zurück in Grundzustand
                }
                break;
                case 'y':   // set y-scan positions
                switch  (uart_rx[1])
                {   case 'l':   PosYmin=databuffer; break;  // set y_axis low
                    case 'h':   PosYmax=databuffer; break;  // set y_axis high
                    default:                //in case of other values
                    SendERR("SetPosition: wrong scan parameter!");      //send error
                    uart_state=START;       // zurück in Grundzustand
                }
                break;
                case 's':   // set scan stepsizes
                switch  (uart_rx[1])
                {   case 'x':   PosXstep=databuffer;    break;  // set scanstep in X-direction
                    case 'y':   PosYstep=databuffer;    break;  // set scanstep in Y-direction
                    case 'z':   PosZstep=databuffer;    break;  // set scanstep in Z-direction
                    case 'i':   PosIstep=databuffer;    break;  // set scanstep in Z-direction
                    default:                //in case of other values
                    SendERR("SetPosition: wrong scan parameter!");      //send error
                    uart_state=START;       // zurück in Grundzustand
                }
                break;
                case 'd':   // set delays
                switch  (uart_rx[1])
                {   case 'l':   PosYdelay=databuffer;   break;  //set line-delay
                    case 'm':   I2C_TxData[0]=databuffer>>8;
                    I2C_TxData[1]=databuffer;
                    I2C_Master_WriteReg(MSP430_SLAVE_ADDR, uart_rx[1], I2C_TxData, TYPE_1_LENGTH);
                    break; //sdm<uint16>     Set latchup-turn-off-delay in miliseconds
                    case 'u':   I2C_TxData[0]=databuffer>>8;
                    I2C_TxData[1]=databuffer;
                    I2C_Master_WriteReg(MSP430_SLAVE_ADDR, uart_rx[1], I2C_TxData, TYPE_1_LENGTH);
                    break; //sdu<uint16>      Set latchup-turn-off-delay in microseconds
                    case 'p':   PosXdelay=databuffer;   break;  //set pixel-delay
                    default:                //in case of other values
                    SendERR("SetPosition: wrong scan parameter!");      //send error
                    uart_state=START;       // zurück in Grundzustand
                }
                break;
                case 'p':   // set position
                switch  (uart_rx[1])
                {   case 'x':   MCP4728_MultiWrite(3, databuffer); PosX=databuffer;    break;//(I, Z, T, X)
                    case 't':   MCP4728_MultiWrite(2, databuffer); PosT=databuffer;    break;//(I, Z, T, X)
                    case 'z':   MCP4728_MultiWrite(1, databuffer); PosZ=databuffer;    break;//(I, Z, T, X)
                    case 'y':                                       PosY=databuffer;    break;//(I, Z, T, X)
                    default:                //Bei anderen Werten
                    SendERR("SetPosition: wrong parameter!");       //Fehler melden
                    uart_state=START;       // zurück in Grundzustand
                }
                break;
                case 'l':   // set laser parameters
                switch  (uart_rx[1])
                {   case 'i':   MCP4728_MultiWrite(0, PosI);    PosI=databuffer;    break;  // set laser intensity
                    case 'e':   PosImin=databuffer;    break;  // set min laser intensity
                    case 'g':   PosImax=databuffer;    break;  // set max laser intensity
                    case 'f':   enablePulsGen(PosF);            PosF=databuffer;    break;  // set pulse frequency
                    case 'p':                                   PosP=databuffer;    break;  // set SEE-pulse intensity
                    default:    //Bei anderen Werten
                    SendERR("SetLaser:wrong parameter!");       //Fehler melden
                    uart_state=START;       // zurück in Grundzustand
                }
                break;
                default:                //Bei anderen Werten
                SendERR("Wrong Set command received!");     //Fehler melden
                uart_state=START;       // zurück in Grundzustand
            }
            if(uart_state != START) //Wenn kein Fehler aufgetreten
            SendACK();          //answer with acknowledge
            uart_state=START;               // back to initial UART state
            break;
        }
        case GET1:
        {   uart_rx[0] = data;
            uart_state = GET2;
            break;
        }
        case GET2:
        {   uart_rx[1] = data;
            int tmpPosition = 0;
            switch (uart_rx[0])
            {   case 'd':   //DUT
                switch (uart_rx[1])
                {   case 'c':   tmpPosition = readMAX9611_current(); break; // gdc   - read current from DUT and return value in mA
                    case 'v':   tmpPosition = readMAX9611_voltage(); break; // gdv   - read voltage from DUT and return value in 10mV
                    default: uart_state = START;    break;
                }
                break;
                case 'p':
                switch (uart_rx[1])
                {   case 'x':   tmpPosition = PosX; break;  // gpx
                    case 'y':   tmpPosition = PosY; break;  // gpy
                    case 'z':   tmpPosition = PosZ; break;  // gpz
                    case 't':   tmpPosition = PosT; break;  // gpt
                    default: uart_state = START;    break;
                }
                break;
                case 'l':
                switch (uart_rx[1])
                {   case 'i':   tmpPosition = PosI; break;  // gli
                    case 'f':   tmpPosition = PosF; break;  // glf
                    case 'p':   tmpPosition = PosP; break;  // glp
                    default:    uart_state = START; break;
                }
                break;
                default:
                uart_state = START;
                break;
            }
            if (uart_state == START)
            SendERR("Wrong GET command received!");
            else
            {   UART_TX('d');
                UART_TX('a');
                UART_TX('t');
                UART_TX((char)(tmpPosition >> 8));
                UART_TX((char)tmpPosition);
                uart_state = START;
            }
            break;
        }
        case ACTION1:
        {   uart_rx[0]=data;
            uart_state=ACTION2;
            break;  // action command
        }
        case ACTION2:
        uart_rx[1] = data;
        switch (uart_rx[0])                 //action command select
        {   case 'l':                       // action laser
            switch  (uart_rx[1])
            {   case 'p':
                SEE_Puls();
                SendACK();
                uart_state = START;
                break;              // einzelnen Puls mit anderer Intensitaet rausballern
                case 'm':
                mode = MODE_MULTIPULSE; //set mode to multiple pulses
                P2SEL &= ~PULSOUT;          // P2.3 option PIO select
                MCP4728_MultiWrite(0,PosP);// SEE-Puls Intensität setzen (char channel, char data)
                delay(5000);
                disablePulsGen();   //stop pulse generator
                break;
                default:                //Bei anderen Werten
                SendERR("Wrong Laser Action command received!");        //Fehler melden
                uart_state=START;   // zurück in Grundzustand
            }
            break;
            case 's':                       // action scan
            switch  (uart_rx[1])
            {   case 'a':               // action scan area - asa
                if (mode != MODE_SCANPAUSE && mode != MODE_SCANAREA)    init_scanStart();
                // [hen] if scanning is paused, just return to scan mode
                uart_state=START;   // uart back to start condition
                mode = MODE_SCANAREA;   //set mode to scan-mode
                break;
                case 'p':               // asp: action scan pause   // [hen] added scan pause mode
                mode = MODE_SCANPAUSE;
                break;
                case 's':   mode = MODE_MANUPOS;    break;  // action scan stop - ass
                case 'l':   // action scan latchup (was scan line in older revision)
                case 'u':   //action scan latch up - asu (asu shall be removed soon -> new is asl)
                if (mode != MODE_SCANPAUSE && mode != MODE_SCANAREA_SEL)    init_scanStart();
                uart_state=START;   // uart back to start condition
                mode = MODE_SCANAREA_SEL;   //set mode to scan-mode
                case 'm': //action scan multi (scans refl., current and voltage values)
                if (mode != MODE_SCANPAUSE && mode != MODE_PARALLELSCAN)    init_scanStart();
                uart_state=START;   // uart back to start condition
                mode = MODE_PARALLELSCAN;   //set mode to scan-mode
                break;                  // action scan parallel("both")

                case 'n': //action scan multi (scans refl., current and voltage values)
                if (mode != MODE_SCANPAUSE && mode != MODE_MULTINSSCAN) init_scanStart();
                uart_state=START;   // uart back to start condition
                mode = MODE_MULTINSSCAN;   //set mode to scan-mode
                PosAltI = PosImax;//PosAltI=PosImin;
                PosI = PosImax;
                PosI_mul_max=PosImax;
                PosI_mul_min=PosImin;
                MCP4728_MultiWrite(0, PosI);
                break;
                case 'J': //action scan multi (scans refl., current and voltage values)
                if (mode != MODE_SCANPAUSE && mode != MODE_AUTOFOCUS)   init_scanStart();
                uart_state=START;   // uart back to start condition
                MCP4728_MultiWrite(0, PosI);
                mode = MODE_AUTOFOCUS;   //set mode to scan-mode
                break;
                default:                //Bei anderen Werten
                SendERR("Wrong Scan Action command received!");     // Fehler melden
                uart_state=START;   // zurück in Grundzustand
            }
            break;
            default:                        //Bei anderen Werten
            SendERR("Wrong Action command received!");      //Fehler melden
            uart_state=START;           // zurück in Grundzustand
        }
        if(uart_state != START) // Wenn kein Fehler aufgetreten [hen] oder kein ack gesendet werden soll
        {   SendACK();                      // Bestaetigung antworten
        }
        uart_state=START;                   // zurück in Grundzustand
        break;
        default :   SendERR("uart_state-maschine error!");          //Fehler melden
        uart_state=START;               // zurück in Grundzustand
    }
}

//>>>>>>>>>>>>>>>>>>>> ISR <<<<<<<<<<<<<<<<<<<<<<<<<
// Timer A0 interrupt service routine
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer_A (void)
{   SysTick++;
    if(SysTick%2)   // if odd millisecond
    {   if(PosYIst>PosY) //Y_Schritt(1);
        {   //Hier Endlage abfragen!!!
            P2OUT|=Y_DIR;
            if(PosYIst>0) PosYIst--;
            P2OUT|=Y_STEP;  //rising edge = make motor step
        }
        if(PosYIst<PosY) //Y_Schritt(0);
        {   P2OUT&=~Y_DIR;
            if(PosYIst<POSYMAX) PosYIst++;
            P2OUT|=Y_STEP;  //rising edge = make motor step
        }
    }
    else    // if even millisecond
    P2OUT&=~Y_STEP; //Schrittausgang auf 0 setzen (TI: min.Pulsbreite 1us!)
}
//------------------------------------------------------------------------------
// The USCIAB0TX_ISR is structured such that it can be used to transmit any
// number of bytes by pre-loading TXByteCtr with the byte count. Also, TXData
// points to the next byte to transmit.
// aus: msp430g2xx3_uscib0_i2c_08.c
//------------------------------------------------------------------------------
void __attribute__ ((interrupt(USCIAB0TX_VECTOR))) USCIAB0TX_ISR (void)
{   if (IFG2 & UCB0RXIFG)                 // Receive Data Interrupt
    {   //Must read from UCB0RXBUF
        uint8_t rx_val = UCB0RXBUF;

        if (RXByteCtr)
        {   ReceiveBuffer[ReceiveIndex++] = rx_val;
            RXByteCtr--;
        }
        if (RXByteCtr == 1)
        {   UCB0CTL1 |= UCTXSTP;
        }
        else if (RXByteCtr == 0)
        {   IE2 &= ~UCB0RXIE;
            MasterMode = IDLE_MODE;
            __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
        }
    }
    else if (IFG2 & UCB0TXIFG)            // Transmit Data Interrupt
    {   switch (MasterMode)
        {   case TX_REG_ADDRESS_MODE:
            UCB0TXBUF = TransmitRegAddr;
            if (RXByteCtr)
            MasterMode = SWITCH_TO_RX_MODE;   // Need to start receiving now
            else
            MasterMode = TX_DATA_MODE;        // Continue to transmision with the data in Transmit Buffer
            break;
            case SWITCH_TO_RX_MODE:
            IE2 |= UCB0RXIE;              // Enable RX interrupt
            IE2 &= ~UCB0TXIE;             // Disable TX interrupt
            UCB0CTL1 &= ~UCTR;            // Switch to receiver
            MasterMode = RX_DATA_MODE;    // State state is to receive data
            UCB0CTL1 |= UCTXSTT;          // Send repeated start
            if (RXByteCtr == 1)
            {   //Must send stop since this is the N-1 byte
                while((UCB0CTL1 & UCTXSTT));
                UCB0CTL1 |= UCTXSTP;      // Send stop condition
            }
            break;
            case TX_DATA_MODE:
            if (TXByteCtr)
            {   UCB0TXBUF = TransmitBuffer[TransmitIndex++];
                TXByteCtr--;
            }
            else
            {   //Done with transmission
                UCB0CTL1 |= UCTXSTP;     // Send stop condition
                MasterMode = IDLE_MODE;
                IE2 &= ~UCB0TXIE;                       // disable TX interrupt
                __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
            }
            break;
            default:
            __no_operation();
            break;
        }
    }
}
//  Wenn Zeichen über UART empfangen
void __attribute__ ((interrupt(USCIAB0RX_VECTOR))) USCI0RX_ISR (void)
{    fifo_add(UCA0RXBUF);        //put received byte into fifo-buffer
}
// ADC10 interrupt service routine (=Potiauswertung)
void __attribute__ ((interrupt(ADC10_VECTOR))) ADC10_ISR (void)
{    __bic_SR_register_on_exit(CPUOFF);        // Clear CPUOFF bit from 0(SR)
}



// Port 1 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
    #pragma vector=PORT1_VECTOR
    __interrupt void Port_1(void)
    #elif defined(__GNUC__)
    void __attribute__ ((interrupt(PORT1_VECTOR))) Port_1 (void)
    #else
    #error Compiler not supported!
#endif
{
    SEE_FLAG++;
    P1IFG &= ~BIT3;                           // Clear P1.3 IFG

}
