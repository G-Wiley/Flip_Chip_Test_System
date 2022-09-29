// tester_gw.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

//#include <iostream>

/************************************************************************/
/*                                                                      */
/*      tester.c        PDP-8 card tester via SPI                       */
/*                                                                      */
/*                                                                      */
/*      compile with Microsoft C 1.52 with command line:                */
/*              cl /W4 /AL tester.c                                     */
/*                                                                      */
/************************************************************************/

#include "FS.h"
#include "SD.h"
#include "SPI.h"

//#define VERSION_STRING  "version 0.91 November 27, 2015"
#define VERSION_STRING  "version 0.91a August 4, 2021"

//#define _CRT_SECURE_NO_WARNINGS  1 /* disable Microsoft 'old library' warnings */

// define MCP chip addresses
#define IC1 1
#define IC2 2
#define IC3 3
#define IC4 4
#define IC5 5
#define IODIR_AB 0
#define GPPUA_AB 0x0c
#define GPIO_AB  0x12

const int onboard_ledPin = 2;
const int SDchipSelectPin = 4;
const int MCPchipSelectIC1 = 21;
const int MCPchipSelectIC2 = 17;
const int MCPchipSelectIC3 = 16;
const int MCPchipSelectIC4 = 15;
const int MCPchipSelectIC5 = 22;
const int MCPnresetPin = 5;

//#define SERIAL_TIMEOUT 600000

#define LOG_FILENAME "tester.log"
char  print_buffer[1024];
//char  print_buffer[512];
File file_test;

// modification to the original print function
// outputs a char array called "buffer" to the console and to the Log File
//
void print(const char* pbuffer)
{
  Serial.print(String(pbuffer));
  //appendFile(SD, LOG_FILENAME, buffer);
}

// function to print a char buffer
//void print(const char* buffer)
//{
//  printST(string(buffer));
//}

String millis2time() {
  String Time = "";
  unsigned long ss;
  byte mm, hh, dd;
  ss = millis() / 1000;
  dd = ss / 86400;
  hh = (ss - dd * 86400) / 3600;
  mm = (ss - dd * 86400 - hh * 3600) / 60;
  ss = (ss - dd * 86400 - hh * 3600) - mm * 60;
  Time += (String)dd + ":";
  if (hh < 10)Time += "0";
  Time += (String)hh + ":";
  if (mm < 10)Time += "0";
  Time += (String)mm + ":";
  if (ss < 10)Time += "0";
  Time += (String)ss;
  return Time;
}

void str_to_upr(char * cbuffer)
{
  while(*cbuffer != '\0'){
    if((*cbuffer >= 'a') && (*cbuffer <= 'z')) *cbuffer = *cbuffer & 0xdf;
    cbuffer++;
  }
}

void sd_fgets(char * sdfgbuffer, int sd_buffer_size/*, file_test*/)
{
  int sdcharcount = 1;
  char *sdbufferpointer;

  //Serial.println("sd_fgets called");
  sdbufferpointer = sdfgbuffer;
  while(file_test.available()){
    *sdbufferpointer = file_test.read();
    if(*sdbufferpointer == '\n') break;
    //if(*sdbufferpointer == '\r') break;
    if(*sdbufferpointer == '\0') break;
    if(++sdcharcount >= sd_buffer_size) break;
    if(*sdbufferpointer != '\r')
      ++sdbufferpointer;
  }
  *sdbufferpointer = '\0';
  Serial.println("sd_fgets exit [" + String(sdcharcount) + "]: " + String(sdfgbuffer));
}

void print_timestamp(void)
{
  #define TIME_BUFFER_LENGTH 26
  //uint32_t   now_time32;         /* MSVC 1.52 is only 32 bit times.  BLOWS UP in 2038  */
  char  time_buffer[TIME_BUFFER_LENGTH];
  char* ptr;

  //time(&now_time32);
  String time_since_boot = millis2time();
  //strcpy(time_buffer, ctime(&now_time32));  /* ctime() returns pointer to static 26 character string  */
  // copy the time_since_boot String to the time_buffer char array
  time_since_boot.toCharArray(time_buffer, TIME_BUFFER_LENGTH);
  
  ptr = strchr(time_buffer, '\n'); // these two lines are from the original code, probably no longer needed
  if (ptr != (char*)NULL) *ptr = '\0';

  print(time_buffer);
} 

// Function to write to the MCP23S17
// parameters are chip address, register address, register data
//
void mcpwrite(uint16_t chipaddr, uint16_t regaddr, uint16_t regdata) {
  //MCP23S17 write to IC chipaddr, register pair regaddr, with regdata
  switch(chipaddr){               // assert the chip select signal of the specified MCP chip
    case IC1:
      digitalWrite(MCPchipSelectIC1, LOW);
      break;
    case IC2:
      digitalWrite(MCPchipSelectIC2, LOW);
      break;
    case IC3:
      digitalWrite(MCPchipSelectIC3, LOW);
      break;
    case IC4:
      digitalWrite(MCPchipSelectIC4, LOW);
      break;
    case IC5:
      digitalWrite(MCPchipSelectIC5, LOW);
      break;
    default:
      print("broken sofware, invalid MCP chip address");
  }
  SPI.transfer16(0x4000 | (chipaddr << 9) | regaddr); //Device Opcode, Chip Address, Write, Reg Addr
  //SPI.transfer16(0x4000 | regaddr); //Device Opcode, Write, Reg Addr
  SPI.transfer16(regdata); // Register Data
  digitalWrite(MCPchipSelectIC1, HIGH);
  digitalWrite(MCPchipSelectIC2, HIGH);
  digitalWrite(MCPchipSelectIC3, HIGH);
  digitalWrite(MCPchipSelectIC4, HIGH);
  digitalWrite(MCPchipSelectIC5, HIGH);
}

// Function to read from the MCP23S17
// parameters are chip address, register address
// returns 16-bit value read from the specified chip and register
//
uint16_t mcpread(uint16_t chipaddr, uint16_t regaddr) {
  //MCP23S17 read from IC chipaddr, register pair regaddr, with regdata
  uint16_t mcp_read_data;
  
  switch(chipaddr){               // assert the chip select signal of the specified MCP chip
    case IC1:
      digitalWrite(MCPchipSelectIC1, LOW);
      break;
    case IC2:
      digitalWrite(MCPchipSelectIC2, LOW);
      break;
    case IC3:
      digitalWrite(MCPchipSelectIC3, LOW);
      break;
    case IC4:
      digitalWrite(MCPchipSelectIC4, LOW);
      break;
    case IC5:
      digitalWrite(MCPchipSelectIC5, LOW);
      break;
    default:
      print("broken sofware, invalid MCP chip address");
  }
  SPI.transfer16(0x4100 | (chipaddr << 9) | regaddr); //Device Opcode, Chip Address, Read, Reg Addr
  //SPI.transfer16(0x4100 | regaddr); //Device Opcode, Read, Reg Addr
  mcp_read_data = SPI.transfer16(0x0000); //sending dummy data to read the port
  digitalWrite(MCPchipSelectIC1, HIGH);
  digitalWrite(MCPchipSelectIC2, HIGH);
  digitalWrite(MCPchipSelectIC3, HIGH);
  digitalWrite(MCPchipSelectIC4, HIGH);
  digitalWrite(MCPchipSelectIC5, HIGH);
  return(mcp_read_data);
}


unsigned short  lpt_base = 0x378;   /* LPT1 on Dell D610    */

unsigned char   lpt_value_last_data_out;
unsigned char   lpt_value_last_control_out;
unsigned char   lpt_value_new_data;
unsigned char   lpt_value_new_control;
unsigned char   lpt_value_status_last_read;
unsigned char   lpt_value_data_last_read;
unsigned char   lpt_value_control_last_read;

void lpt_input_data(void)
{
    //lpt_value_data_last_read = (unsigned char)_inp(lpt_base + 0);
}

void lpt_input_status(void)
{
    //lpt_value_status_last_read = (unsigned char)_inp(lpt_base + 1);
}

void lpt_input_control(void)
{
    //lpt_value_control_last_read = (unsigned char)_inp(lpt_base + 2);
}

void lpt_output_data(void)
{
    lpt_value_last_data_out = lpt_value_new_data;
    //_outp(lpt_base + 0, lpt_value_last_data_out);
}

void lpt_output_control(void)
{
    lpt_value_last_control_out = lpt_value_new_control;
    //_outp(lpt_base + 2, lpt_value_last_control_out);
}

void lpt_init(void)
{
    lpt_input_data();
    lpt_input_control();
    lpt_input_status();

    lpt_value_new_data = lpt_value_data_last_read;
    lpt_value_new_control = lpt_value_control_last_read;

    lpt_output_data();
    lpt_output_control();
}



/* routines that stage next output or use last input    */

void lpt_set_data_bit(unsigned int bit_number)
{
    if (bit_number > 7)
    {
        print("ERROR: broken software\r\n");
        exit(1);
    }
    lpt_value_new_data |= (1 << bit_number);
}

void lpt_clr_data_bit(unsigned int bit_number)
{
    if (bit_number > 7)
    {
        print("ERROR: broken software\r\n");
        exit(1);
    }
    lpt_value_new_data &= ~(1 << bit_number);
}

void lpt_set_control_bit(unsigned int bit_number)
{
    if (bit_number > 7)
    {
        print("ERROR: broken software\r\n");
        exit(1);
    }
    lpt_value_new_control |= (1 << bit_number);
}

void lpt_clr_control_bit(unsigned int bit_number)
{
    if (bit_number > 7)
    {
        print("ERROR: broken software\r\n");
        exit(1);
    }
    lpt_value_new_control &= ~(1 << bit_number);
}

void lpt_toggle_data_bit(unsigned int bit_number)
{
    if (bit_number > 7)
    {
        print("ERROR: broken software\r\n");
        exit(1);
    }
    lpt_value_new_data ^= (1 << bit_number);
}

void lpt_toggle_control_bit(unsigned int bit_number)
{
    if (bit_number > 7)
    {
        print("ERROR: broken software\r\n");
        exit(1);
    }
    lpt_value_new_control ^= (1 << bit_number);
}


//#define SPI_SPARE_PIN_1_LO      (lpt_set_control_bit( 0 ))                          /* pin  1 C0- -Strobe   */
//#define SPI_SPARE_PIN_1_HI      (lpt_clr_control_bit( 0 ))                          /* pin  1 C0- -Strobe   */
//#define SPI_RESET_N_PIN_2_LO    (lpt_clr_data_bit( 0 ))                             /* pin  2 D0+ +Data0    */
//#define SPI_RESET_N_PIN_2_HI    (lpt_set_data_bit( 0 ))                             /* pin  2 D0+ +Data0    */
//#define SPI_CS_N_PIN_3_LO       (lpt_clr_data_bit( 1 ))                             /* pin  3 D1+ +Data1    */
//#define SPI_CS_N_PIN_3_HI       (lpt_set_data_bit( 1 ))                             /* pin  3 D1+ +Data1    */
//#define SPI_CLK_PIN_4_LO        (lpt_clr_data_bit( 2 ))                             /* pin  4 D2+ +Data2    */
//#define SPI_CLK_PIN_4_HI        (lpt_set_data_bit( 2 ))                             /* pin  4 D2+ +Data2    */
//#define SPI_SO_A_PIN_5_LO       (lpt_clr_data_bit( 3 ))                             /* pin  5 D3+ +Data3    */
//#define SPI_SO_A_PIN_5_HI       (lpt_set_data_bit( 3 ))                             /* pin  5 D3+ +Data3    */
//#define SPI_SO_B_PIN_6_LO       (lpt_clr_data_bit( 4 ))                             /* pin  6 D4+ +Data4    */
//#define SPI_SO_B_PIN_6_HI       (lpt_set_data_bit( 4 ))                             /* pin  6 D4+ +Data4    */
//#define SPI_SO_C_PIN_7_LO       (lpt_clr_data_bit( 5 ))                             /* pin  7 D5+ +Data5    */
//#define SPI_SO_C_PIN_7_HI       (lpt_set_data_bit( 5 ))                             /* pin  7 D5+ +Data5    */
//#define SPI_SO_D_PIN_8_LO       (lpt_clr_data_bit( 6 ))                             /* pin  8 D6+ +Data6    */
//#define SPI_SO_D_PIN_8_HI       (lpt_set_data_bit( 6 ))                             /* pin  8 D6+ +Data6    */
//#define SPI_SO_E_PIN_9_LO       (lpt_clr_data_bit( 7 ))                             /* pin  9 D7+ +Data7    */
//#define SPI_SO_E_PIN_9_HI       (lpt_set_data_bit( 7 ))                             /* pin  9 D7+ +Data7    */
//#define SPI_SI_D_PIN_10_IN      ((lpt_value_status_last_read & (1 << 6)) ? 1 : 0)   /* pin 10 S6+ -Ack      */
//#define SPI_SI_E_PIN_11_IN      ((lpt_value_status_last_read & (1 << 7)) ? 0 : 1)   /* pin 11 S7- +Busy     */
//#define SPI_SI_C_PIN_12_IN      ((lpt_value_status_last_read & (1 << 5)) ? 1 : 0)   /* pin 12 S5+ +PaperEnd */
//#define SPI_SI_B_PIN_13_IN      ((lpt_value_status_last_read & (1 << 4)) ? 1 : 0)   /* pin 13 S4+ +SelectIn */
//#define SPI_SPARE_PIN_14_LO     (lpt_set_control_bit( 1 ))                          /* pin 14 C1- -AutoFd   */
//#define SPI_SPARE_PIN_14_HI     (lpt_clr_control_bit( 1 ))                          /* pin 14 C1- -AutoFd   */
//#define SPI_SI_A_PIN_15_IN      ((lpt_value_status_last_read & (1 << 3)) ? 1 : 0)   /* pin 15 S3+ -Error    */
//#define SPI_SPARE_PIN_16_LO     (lpt_clr_control_bit( 2 ))                          /* pin 16 C2+ -Init     */
//#define SPI_SPARE_PIN_16_HI     (lpt_set_control_bit( 2 ))                          /* pin 16 C2+ -Init     */
//#define SPI_SPARE_PIN_17_LO     (lpt_set_control_bit( 3 ))                          /* pin 17 C3- -Select   */
//#define SPI_SPARE_PIN_17_HI     (lpt_clr_control_bit( 3 ))                          /* pin 17 C3- -Select   */
/* pins 18 thru 25 are ground   */


//void spi_output(void)
//{
//    lpt_output_data();
//}


//void spi_input(void)
//{
//    lpt_input_status();
//}


//void spi_init(void)
//{
//    lpt_init();
//
//    SPI_SPARE_PIN_1_LO;     /* spare lo             */
//    SPI_RESET_N_PIN_2_HI;   /* reset-n hi           */
//    SPI_CS_N_PIN_3_HI;      /* remove chip select   */
//    SPI_CLK_PIN_4_HI;       /* clock hi             */
//    SPI_SO_A_PIN_5_LO;      /* data out lo          */
//    SPI_SO_B_PIN_6_LO;      /* data out lo          */
//    SPI_SO_C_PIN_7_LO;      /* data out lo          */
//    SPI_SO_D_PIN_8_LO;      /* data out lo          */
//    SPI_SO_E_PIN_9_LO;      /* data out lo          */
//    SPI_SI_D_PIN_10_IN;     /* input                */
//    SPI_SI_E_PIN_11_IN;     /* input                */
//    SPI_SI_C_PIN_12_IN;     /* input                */
//    SPI_SI_B_PIN_13_IN;     /* input                */
//    SPI_SPARE_PIN_14_LO;    /* spare lo             */
//    SPI_SI_A_PIN_15_IN;     /* input                */
//    SPI_SPARE_PIN_16_LO;    /* spare lo             */
//    SPI_SPARE_PIN_17_LO;    /* spare lo             */
//    spi_output();
//}

//void spi_open(void)
//{
//    SPI_CS_N_PIN_3_LO;      /* apply chip select    */
//    spi_output();
//}

//void spi_close(void)
//{
//    SPI_CS_N_PIN_3_HI;      /* remove chip select   */
//    spi_output();
//}

//void spi_txrx(unsigned char* out_ptr, unsigned char* in_ptr)
//{
//    int             i;
//    unsigned char   mask;
//
//    for (i = 0; i < 5; i++) in_ptr[i] = 0;  /* zero input array */
//    mask = 0x80;        /* MSB first    */
//    do
//    {
//        if (out_ptr[0] & mask) SPI_SO_A_PIN_5_HI; else SPI_SO_A_PIN_5_LO;
//        if (out_ptr[1] & mask) SPI_SO_B_PIN_6_HI; else SPI_SO_B_PIN_6_LO;
//        if (out_ptr[2] & mask) SPI_SO_C_PIN_7_HI; else SPI_SO_C_PIN_7_LO;
//        if (out_ptr[3] & mask) SPI_SO_D_PIN_8_HI; else SPI_SO_D_PIN_8_LO;
//        if (out_ptr[4] & mask) SPI_SO_E_PIN_9_HI; else SPI_SO_E_PIN_9_LO;
//        SPI_CLK_PIN_4_LO;
//        spi_output();
//
//        SPI_CLK_PIN_4_HI;   /* rising edge  */
//        spi_output();
//        spi_input();
//
//        if (SPI_SI_A_PIN_15_IN == 0) in_ptr[0] |= mask;
//        if (SPI_SI_B_PIN_13_IN == 0) in_ptr[1] |= mask;
//        if (SPI_SI_C_PIN_12_IN == 0) in_ptr[2] |= mask;
//        if (SPI_SI_D_PIN_10_IN == 0) in_ptr[3] |= mask;
//        if (SPI_SI_E_PIN_11_IN == 0) in_ptr[4] |= mask;
//
//        mask >>= 1;
//    } while (mask);
//}


// MCP23S17 Register Address definitions for IOCON.BANK = 0
// The A & B registers are in the same bank, not separated into different banks
// This is best for the tester so that A & B can be written and read using a single 16-bit SPI transfer
// These are the original register definitions, not changed for Arduino, just added the header comments.
//
#define REG_IODIR   0x00        /* I/O direction (1=in) */
#define REG_IOPOL   0x02        /* I/O polarity         */
#define REG_GPINTEN 0x04        /* interrupt enables    */
#define REG_DEFVAL  0x06        /* default values       */
#define REG_INTCON  0x08        /* interrupt config     */
#define REG_IOCON   0x0A        /* I/O config           */
#define REG_GPPU    0x0C        /* pullup enables       */
#define REG_INTF    0x0E        /* interrupt flag       */
#define REG_INTCAP  0x10        /* interrupt capture    */
#define REG_GPIO    0x12        /* input/output         */
#define REG_OLAT    0x14        /* output latches       */

// Print the name of the MCP23S17 register specified by the "reg" parameter
// the register LSB is removed in this function so the name is not specific to A or B
//
void print_reg_name(unsigned char reg)
{
    switch (reg & 0xFE)
    {
    case REG_IODIR:     print("IODIR  "); break;
    case REG_IOPOL:     print("IOPOL  "); break;
    case REG_GPINTEN:   print("GPINTEN"); break;
    case REG_DEFVAL:    print("DEFVAL "); break;
    case REG_INTCON:    print("INTCON "); break;
    case REG_IOCON:     print("IOCON  "); break;
    case REG_GPPU:      print("GPPU   "); break;
    case REG_INTF:      print("INTF   "); break;
    case REG_INTCAP:    print("INTCAP "); break;
    case REG_GPIO:      print("GPIO   "); break;
    case REG_OLAT:      print("OLAT   "); break;
    default:            print("???????"); break;
    }
}


// reg_init() is from the original code. 
// It asserts reset for 100 msec and de-asserts reset for an additional 100 msec.
// The MCP23S17 resets when RESET/ is asserted for > 1 usec so use the Arduino delay
// function to delay 2 msec to guarantee the delay is at least 1 msec, far more than 1 usec
//
void reg_init(void)       /* initialize GPIO chip registers   */
{
    digitalWrite(MCPchipSelectIC1, LOW); // drive all five chip selects inactive, not sure if it's necessary
    digitalWrite(MCPchipSelectIC2, LOW);
    digitalWrite(MCPchipSelectIC3, LOW);
    digitalWrite(MCPchipSelectIC4, LOW);
    digitalWrite(MCPchipSelectIC5, LOW);
    digitalWrite(MCPnresetPin, LOW); // set MCP not-reset to the active (LOW) state
    delay(2); //delay 2 msec to hold not-reset LOW 
    digitalWrite(MCPnresetPin, HIGH); // restore MCP not-reset to the inactive (HIGH) state
    digitalWrite(MCPchipSelectIC1, HIGH); //drive all five MCP chip selects inactive
    digitalWrite(MCPchipSelectIC2, HIGH);
    digitalWrite(MCPchipSelectIC3, HIGH);
    digitalWrite(MCPchipSelectIC4, HIGH);
    digitalWrite(MCPchipSelectIC5, HIGH);
    delay(2); //delay 2 msec to hold not-reset HIGH after reset
    MCPinitializeIOCON(); // initializes BANK and HAEN
//    unsigned long   i;
//
//    spi_init();                 /* initialize SPI signals       */
//
//#define RESET_LOOPS 20000UL     /* 20,000 is about 100ms on D610,WinXp,allowio  */
//    for (i = 0; i < RESET_LOOPS; i++)
//    {
//        SPI_RESET_N_PIN_2_LO;   /* reset active          */
//        spi_output();
//        spi_input();
//    }
//    for (i = 0; i < RESET_LOOPS; i++)
//    {
//        SPI_RESET_N_PIN_2_HI;   /* reset inactive       */
//        spi_output();
//        spi_input();
//    }
}



// function to read from the registers in all 5 MCP23S17 chips
// "reg" specifies the register address, "value" is an array to return the 5 data values
// that were read... one value for each chip. value[0] is from IC1 and value[4] is from IC5.
// The addresses of the MCP chips are numbered 1 through 5, corresponding to the IC numbers.
// This function has been converted to use the Arduino SPI hardware and accesses
// each MCP chip individually. 
//
void reg_read(unsigned char reg, unsigned int* value)
{
  // "old" code for the PC parallel port is commented out
  //unsigned int    i;
  unsigned int chipaddress;
  unsigned int tempreadvalue; // because tester code expects A reg in the low byte and B reg in the high byte
  //unsigned char   rx_array[5];
  //unsigned char   tx_array[5];

  //spi_open();
  //for (i = 0; i < 5; i++) tx_array[i] = 0x4F;     /* b0 = 1 read  */
  //spi_txrx(tx_array, rx_array);
  //for (i = 0; i < 5; i++) tx_array[i] = reg;
  //spi_txrx(tx_array, rx_array);
  //for (i = 0; i < 5; i++) tx_array[i] = 0x5A;
  //spi_txrx(tx_array, rx_array);
  //for (i = 0; i < 5; i++) value[i] = (unsigned int)rx_array[i];
  //spi_txrx(tx_array, rx_array);
  //for (i = 0; i < 5; i++) value[i] |= (unsigned int)rx_array[i] << 8;
  //spi_close();

  for(chipaddress = IC1; chipaddress <= IC5; chipaddress++){
    // Exchange the high and low byte of the data value because tester code expects 
    // the A reg in the low byte of the mapping and the B reg in the high byte of the mapping.
    // However, SPI serializes the high byte first to the A register, then the low byte to the B register.
    tempreadvalue = mcpread(chipaddress, reg);
    value[chipaddress - 1] = ((tempreadvalue << 8) & 0xff00) | ((tempreadvalue >> 8) & 0xff);
  }
}


// function to write to the registers in all 5 MCP23S17 chips
// "reg" specifies the register address, "value" is an array of 5 data values
// to be written... one for each chip. value[0] to IC1 and value[4] to IC5.
// The addresses of the MCP chips are numbered 1 through 5, corresponding to the IC numbers.
// This function has been converted to use the Arduino SPI hardware and accesses
// each MCP chip individually. 
//
void reg_write(unsigned char reg, unsigned int* value)
{
  // "old" code for the PC parallel port is commented out
  //unsigned int    i;
  unsigned int chipaddress;
  unsigned int flippedvalue; // because tester code expects A reg in the low byte and B reg in the high byte
  //unsigned char   rx_array[5];
  //unsigned char   tx_array[5];

  //spi_open();
  //for (i = 0; i < 5; i++) tx_array[i] = 0x4E;     /* b0 = 0 write */
  //spi_txrx(tx_array, rx_array);
  //for (i = 0; i < 5; i++) tx_array[i] = reg;
  //spi_txrx(tx_array, rx_array);
  //for (i = 0; i < 5; i++) tx_array[i] = (unsigned char)(value[i]);
  //spi_txrx(tx_array, rx_array);
  //for (i = 0; i < 5; i++) tx_array[i] = (unsigned char)(value[i] >> 8);
  //spi_txrx(tx_array, rx_array);
  //spi_close();

  for(chipaddress = IC1; chipaddress <= IC5; chipaddress++){
    // Exchange the high and low byte of the data value because the tester code expects 
    // the A reg in the low byte of the mapping and the B reg in the high byte of the mapping.
    // However, SPI serializes the high byte first to the A register, then the low byte to the B register.
    flippedvalue = ((value[chipaddress - 1] << 8) & 0xff00) | ((value[chipaddress - 1] >> 8) & 0xff);
    mcpwrite(chipaddress, reg, flippedvalue);
  }
}



unsigned int reg_init_and_verify(void)  /* returns 0 if okay    */
{
    unsigned int    result;
    unsigned int    i;
    unsigned int    data_in[5];
    unsigned int    uTemp;


    reg_init();

    for (i = 0; i < 5; i++) data_in[i] = 0;
    reg_read(REG_IODIR, data_in);
    Serial.println(String("IODIR ")+String(data_in[0],HEX)+String(" ")+String(data_in[1],HEX)+String(" ")+String(data_in[2],HEX)+String(" ")+String(data_in[3],HEX)+String(" ")+String(data_in[4],HEX));
    uTemp = 0xFFFF;
    for (i = 0; i < 5; i++) uTemp &= data_in[i];    /* s/b FFFFs    */
    Serial.println(String("uTemp after IODIR read test: ")+String(uTemp,HEX));
    reg_read(REG_OLAT, data_in);
    Serial.println(String("OLAT ")+String(data_in[0],HEX)+String(" ")+String(data_in[1],HEX)+String(" ")+String(data_in[2],HEX)+String(" ")+String(data_in[3],HEX)+String(" ")+String(data_in[4],HEX));
    uTemp = (~uTemp);                               /* s/b 0000     */
    for (i = 0; i < 5; i++) uTemp |= data_in[i];    /* s/b 0000s    */
    Serial.println(String("uTemp after OLAT read test: ")+String(uTemp,HEX));
    //if (uTemp == 0x0000)
    if ((uTemp & 0xffff) == 0x0000) // must test only the least significant 16 bits
    {
        result = 0;         /* no error */
    }
    else
    {
        result = 1;         /* error    */
        print("\r\n");
        print("\r\n");
        print("***************************************************************************\r\n");
        print("*  did not verify registers after initialize (chip reset).                *\r\n");
        print("*  check that the tester is cabled to LPT port and that the power is on.  *\r\n");
        print("***************************************************************************\r\n");
        print("\r\n");
        print("\r\n");
    }
    return (result);
}



const char edge_pins[19] = "ABCDEFHJKLMNPRSTUV";
#define PIN_DRIVERS  80
#define TEST_COLUMNS 72
const struct
{
    unsigned int  offset;
    unsigned int  mask;
} mapping[PIN_DRIVERS] =     /* 80 pin drivers   */
{
    { 0, (1U << 15)  },     /* AA1  */          /* PIN 0    */
    { 0, (1U << 14)  },     /* AB1  */
    { 0, (1U << 13)  },     /* AC1  */
    { 0, (1U << 12)  },     /* AD1  */
    { 0, (1U << 11)  },     /* AE1  */
    { 0, (1U << 10)  },     /* AF1  */      /* PIN  5 */
    { 0, (1U << 9)   },     /* AH1  */
    { 0, (1U << 8)   },     /* AJ1  */
    { 1, (1U << 15)  },     /* AK1  */
    { 1, (1U << 14)  },     /* AL1  */
    { 1, (1U << 13)  },     /* AM1  */      /* PIN 10 */
    { 1, (1U << 12)  },     /* AN1  */
    { 1, (1U << 11)  },     /* AP1  */
    { 1, (1U << 10)  },     /* AR1  */
    { 1, (1U << 9)   },     /* AS1  */
    { 1, (1U << 8)   },     /* AT1  */          /* PIN 15 is PIN_GROUND_AT1 */
    { 2, (1U << 15)  },     /* AU1  */
    { 2, (1U << 14)  },     /* AV1  */

    { 0, (1U << 7)   },     /* AA2  */          /* PIN 18 is PIN_POWER_AA2  */
    { 0, (1U << 6)   },     /* AB2  */
    { 0, (1U << 5)   },     /* AC2  */          /* PIN 20 is PIN_GROUND_AC2 */
    { 0, (1U << 4)   },     /* AD2  */
    { 0, (1U << 3)   },     /* AE2  */
    { 0, (1U << 2)   },     /* AF2  */
    { 0, (1U << 1)   },     /* AH2  */
    { 0, (1U << 0)   },     /* AJ2  */      /* PIN 25 */
    { 1, (1U << 0)   },     /* AK2  */
    { 1, (1U << 1)   },     /* AL2  */
    { 1, (1U << 2)   },     /* AM2  */
    { 1, (1U << 3)   },     /* AN2  */
    { 1, (1U << 4)   },     /* AP2  */      /* PIN 30 */
    { 1, (1U << 5)   },     /* AR2  */
    { 1, (1U << 6)   },     /* AS2  */
    { 1, (1U << 7)   },     /* AT2  */
    { 2, (1U << 0)   },     /* AU2  */
    { 2, (1U << 1)   },     /* AV2  */      /* PIN 35 */

    { 2, (1U << 13)  },     /* BA1  */          /* PIN 36   */
    { 2, (1U << 12)  },     /* BB1  */
    { 2, (1U << 11)  },     /* BC1  */
    { 2, (1U << 10)  },     /* BD1  */
    { 2, (1U << 9)   },     /* BE1  */      /* PIN 40 */
    { 2, (1U << 8)   },     /* BF1  */
    { 3, (1U << 15)  },     /* BH1  */
    { 3, (1U << 14)  },     /* BJ1  */
    { 3, (1U << 13)  },     /* BK1  */
    { 3, (1U << 12)  },     /* BL1  */      /* PIN 45 */
    { 3, (1U << 11)  },     /* BM1  */
    { 3, (1U << 10)  },     /* BN1  */
    { 3, (1U << 9)   },     /* BP1  */
    { 3, (1U << 8)   },     /* BR1  */
    { 4, (1U << 15)  },     /* BS1  */      /* PIN 50 */
    { 4, (1U << 14)  },     /* BT1  */          /* PIN 51 is PIN_GROUND_BT1 */
    { 4, (1U << 13)  },     /* BU1  */
    { 4, (1U << 12)  },     /* BV1  */

    { 2, (1U << 2)   },     /* BA2  */          /* PIN 54 is PIN_POWER_BA2_NC (no connection)  */
    { 2, (1U << 3)   },     /* BB2  */      /* PIN 55 */
    { 2, (1U << 4)   },     /* BC2  */          /* PIN 56 is PIN_GROUND_BC2 */
    { 2, (1U << 5)   },     /* BD2  */
    { 2, (1U << 6)   },     /* BE2  */
    { 2, (1U << 7)   },     /* BF2  */
    { 3, (1U << 0)   },     /* BH2  */      /* PIN 60 */
    { 3, (1U << 1)   },     /* BJ2  */
    { 3, (1U << 2)   },     /* BK2  */
    { 3, (1U << 3)   },     /* BL2  */
    { 3, (1U << 4)   },     /* BM2  */
    { 3, (1U << 5)   },     /* BN2  */      /* PIN 65 */
    { 3, (1U << 6)   },     /* BP2  */
    { 3, (1U << 7)   },     /* BR2  */
    { 4, (1U << 0)   },     /* BS2  */
    { 4, (1U << 1)   },     /* BT2  */
    { 4, (1U << 2)   },     /* BU2  */      /* PIN 70 */
    { 4, (1U << 3)   },     /* BV2  */

    { 4, (1U << 4)   },     /* PROBE_1      */  /* PIN 72    */
    { 4, (1U << 5)   },     /* PROBE_2      */
    { 4, (1U << 6)   },     /* PROBE_3      */
    { 4, (1U << 7)   },     /* PROBE_4      */  /* PIN 75 */
    { 4, (1U << 8)   },     /* GREEN  LED   */
    { 4, (1U << 9)   },     /* RED    LED   */
    { 4, (1U << 10)  },     /* YELLOW LED   */
    { 4, (1U << 11)  },     /* RED2   LED   */
};

#define PIN_GROUND_AT1  15
#define PIN_GROUND_AC2  20
#define PIN_GROUND_BT1  51
#define PIN_GROUND_BC2  56

unsigned int ground_pin[] =
{
    PIN_GROUND_AT1,
    PIN_GROUND_AC2,
    PIN_GROUND_BT1,
    PIN_GROUND_BC2,
};
#define NUMBER_GROUND_PINS  (sizeof(ground_pin) / sizeof(ground_pin[0]))


#define PIN_POWER_AA2   18

unsigned int power_pin[1] =
{
    PIN_POWER_AA2,
};
#define NUMBER_POWER_PINS  (sizeof(power_pin) / sizeof(power_pin[0]))

#define PIN_POWER_BA2_NC    54      /* no connect, so just make it an output lo */
#define PIN_LED_GREEN       76
#define PIN_LED_RED         77
#define PIN_LED_YELLOW      78
#define PIN_LED_RED2        79


unsigned tester_output_lo[] =
{
    PIN_POWER_BA2_NC,
    PIN_LED_GREEN,
    PIN_LED_RED,
    PIN_LED_YELLOW,
    PIN_LED_RED2,
};
#define NUMBER_OUTPUT_LO_PINS  (sizeof(tester_output_lo) / sizeof(tester_output_lo[0]))


#define PIN_PROBE_1     72
#define PIN_PROBE_2     73
#define PIN_PROBE_3     74
#define PIN_PROBE_4     75

unsigned int tester_input_w_pullup[] =
{
    PIN_PROBE_1,
    PIN_PROBE_2,
    PIN_PROBE_3,
    PIN_PROBE_4,
};
#define NUMBER_INPUT_W_PULLUP   (sizeof(tester_input_w_pullup) / sizeof(tester_input_w_pullup[0]))



char get_pin_type_char(unsigned int pin)
{
    char            result;
    unsigned int    i;

    result = ' ';
    for (i = 0; i < NUMBER_GROUND_PINS; i++)
    {
        if (pin == ground_pin[i]) result = 'G';
    }
    for (i = 0; i < NUMBER_POWER_PINS; i++)
    {
        if (pin == power_pin[i]) result = 'P';
    }
    for (i = 0; i < NUMBER_OUTPUT_LO_PINS; i++)
    {
        if (pin == tester_output_lo[i]) result = 'T';
    }
    for (i = 0; i < NUMBER_INPUT_W_PULLUP; i++)
    {
        if (pin == tester_input_w_pullup[i]) result = 'T';
    }
    return (result);
}



void set_default_directions(unsigned int* data_out)
{
    unsigned int    i;
    unsigned int    pin;

    for (i = 0; i < 5; i++)
    {
        data_out[i] = 0xFFFF;       /* all inputs   */
    }
    for (i = 0; i < NUMBER_OUTPUT_LO_PINS; i++)
    {
        pin = tester_output_lo[i];
        data_out[mapping[pin].offset] &= (~mapping[pin].mask);
    }
}



void set_default_outputs(unsigned int* data_out)
{
    unsigned int    i;

    for (i = 0; i < 5; i++)
    {
        data_out[i] = 0;    /* all outputs lo   */
    }
}



void set_default_pullups(unsigned int* data_out)
{
    unsigned int    i;
    unsigned int    pin;

    for (i = 0; i < 5; i++)
    {
        data_out[i] = 0;    /* all pullups off  */
    }
    for (i = 0; i < NUMBER_INPUT_W_PULLUP; i++)
    {
        pin = tester_input_w_pullup[i];
        data_out[mapping[pin].offset] &= mapping[pin].mask;
    }
}



unsigned int verify_power(unsigned int* data_in)      /* !=0 if okay  */
{
    unsigned int    result;
    unsigned int    i;
    unsigned int    pin;


    Serial.println(String("verify_power 0x")+String(data_in[0],HEX)+String(" 0x")+String(data_in[1],HEX)+String(" 0x")+String(data_in[2],HEX)+String(" 0x")+String(data_in[3],HEX)+String(" 0x")+String(data_in[4],HEX));
    result = 1;     /* assume okay  */
    for (i = 0; i < NUMBER_GROUND_PINS; i++)
    {
        pin = ground_pin[i];
        if ((data_in[mapping[pin].offset] & mapping[pin].mask) != 0) result = 0;
        Serial.println(String("  ground pin result: 0x")+String(result,HEX)+String(" index = ")+String(i));
    }
    Serial.println(String("verify_power ground pin result: ")+String(result,HEX));

    for (i = 0; i < NUMBER_POWER_PINS; i++)
    {
        pin = power_pin[i];
        Serial.println(String("  pin: ")+String(pin));
        Serial.println(String("  data_in[mapping[pin].offset]: 0x")+String(data_in[mapping[pin].offset],HEX)+String(", mapping[pin].mask: 0x")+String(mapping[pin].mask,HEX));
        if ((data_in[mapping[pin].offset] & mapping[pin].mask) == 0) result = 0;
        Serial.println(String("  power pin result: ")+String(result,HEX)+String(" index = ")+String(i));
    }
    Serial.println(String("verify_power power pin result: ")+String(result,HEX));
    return (result);
}



void set_default_used(unsigned int* used)
{
    unsigned int    i;
    unsigned int    pin;

    for (i = 0; i < 5; i++) used[i] = 0x0000;

    for (i = 0; i < NUMBER_GROUND_PINS; i++)
    {
        pin = ground_pin[i];
        used[mapping[pin].offset] |= mapping[pin].mask;
    }
    for (i = 0; i < NUMBER_POWER_PINS; i++)
    {
        pin = power_pin[i];
        used[mapping[pin].offset] |= mapping[pin].mask;
    }
    for (i = 0; i < NUMBER_OUTPUT_LO_PINS; i++)
    {
        pin = tester_output_lo[i];
        used[mapping[pin].offset] |= mapping[pin].mask;
    }
    for (i = 0; i < NUMBER_INPUT_W_PULLUP; i++)
    {
        pin = tester_input_w_pullup[i];
        used[mapping[pin].offset] |= mapping[pin].mask;
    }
}



void stage_pin(unsigned int pin, unsigned int flag, unsigned int* data_out)
{
    if (flag)
    {
        data_out[mapping[pin].offset] |= mapping[pin].mask;
    }
    else
    {
        data_out[mapping[pin].offset] &= (~mapping[pin].mask);
    }
}




void verify_mapping(void)
{
    int             i;
    unsigned int    temp[5];
    unsigned int    offset;
    unsigned int    mask;


    for (i = 0; i < 5; i++) temp[i] = 0;
    for (i = 0; i < PIN_DRIVERS; i++)
    {
        offset = mapping[i].offset;
        if (offset > 4)
        {
            sprintf(print_buffer, "broken software: mapping[%u].offset is bad\r\n", i);
            print(print_buffer);
            exit(1);
        }
        /* expect one and only one bit in the mask  */
        mask = mapping[i].mask;
        mask = mask & (unsigned)(-(signed)mask);    /* keeps lowest bit that is set */
        if ((mask == 0) || (mask != mapping[i].mask)) /* no bits or more than one bit?  */
        {
            sprintf(print_buffer, "broken software: mapping[%u].mask is bad\r\n", i);
            print(print_buffer);
            exit(1);
        }
        if (temp[offset] & mask)  /* mask already used? */
        {
            sprintf(print_buffer, "broken software: mapping[%u] is bad (already used)\r\n", i);
            print(print_buffer);
            exit(1);
        }
        temp[offset] |= mask;
    }
    print("\r\n");
    for (i = 0; i < 5; i++)
    {
        if (temp[i] != 0xFFFF)
        {
            print("broken software: mapping[] does not define all 80 bits\r\n");
            exit(1);
        }
    }
    print("mapping[] is verified\r\n");
}



void print_bin8(unsigned char data)
{
    unsigned char   mask;

    mask = 0x80;
    do
    {
        sprintf(print_buffer, "%1d", data & mask ? 1 : 0);
        print(print_buffer);
        mask >>= 1;
    } while (mask);
}



unsigned int tester_init_and_uut_power_verify(void)       /* returns 0 if no error    */
{
    unsigned int    result;
    unsigned int    data_in[5];
    unsigned int    data_out[5];
    unsigned int    power_valid;

    result = reg_init_and_verify();     /* leaves all pins inputs; output latches = 0   */
    Serial.println(String("reg_init_and_verify result: ")+String(result,HEX));
    if (result == 0)            /* success? 0 is good*/
    {
        /* set I/O directions   */
        set_default_directions(data_out);     /* all inputs + fixed outputs   */
        Serial.println(String("*set_default_directions ")+String(data_out[0],HEX)+String(" ")+String(data_out[1],HEX)+String(" ")+String(data_out[2],HEX)+String(" ")+String(data_out[3],HEX)+String(" ")+String(data_out[4],HEX));
        reg_write(REG_IODIR, data_out);

        /* set pullups          */
        set_default_pullups(data_out);
        Serial.println(String("*set_default_pullups ")+String(data_out[0],HEX)+String(" ")+String(data_out[1],HEX)+String(" ")+String(data_out[2],HEX)+String(" ")+String(data_out[3],HEX)+String(" ")+String(data_out[4],HEX));
        reg_write(REG_GPPU, data_out);

        /* verify power pins    */
        reg_read(REG_GPIO, data_in);
        Serial.println(String("*read power pins ")+String(data_in[0],HEX)+String(" ")+String(data_in[1],HEX)+String(" ")+String(data_in[2],HEX)+String(" ")+String(data_in[3],HEX)+String(" ")+String(data_in[4],HEX));
        power_valid = verify_power(data_in);
        Serial.println(String("power_valid result: ")+String(power_valid,HEX));

        /* set up outputs   */
        set_default_outputs(data_out);
        if (power_valid)
        {
            print("\r\n");
            print("UUT power is okay    "); print_timestamp();  print("\r\n");
            print("\r\n");
            stage_pin(PIN_LED_RED, 1, data_out);
        }
        else
        {
            result = 1;         /* error    */
            print("\r\n");
            print("\r\n");
            print("*****************************\r\n");
            print("  UUT power is OFF          *\r\n");
            print("*****************************\r\n");
            print("\r\n");
            print("\r\n");
        }
        reg_write(REG_OLAT, data_out);
    }
    return (result);
}

#define KEY_ESCAPE      0x001B
#define KEY_CURSOR_UP       0x0148
#define KEY_CURSOR_LEFT     0x014B
#define KEY_CURSOR_RIGHT    0x014D
#define KEY_CURSOR_DOWN     0x0150

int get_a_key(void)
{
  int key;

  for(; Serial.available()<=0;);
  key = Serial.read();
  if ((key == 0) || (key == 0xE0))
  {
    key = (key << 8) | 0x0100 | Serial.read();   /* key is 0x01dd or 0xE0dd  */
  }
  return (key);
}



int get_a_key_convert_to_upper(void)
{
  int     key;

  key = get_a_key();
  if ((key >= 'a') && (key <= 'z')) key += 'A' - 'a';
  return (key);
}

void do_diags(void)
{
    int             i;
    int             key;
    int             command;
    int             index;
    unsigned char   dstatus;
    unsigned char   last_status;
    //unsigned long   delay;
    unsigned char   reg;
    unsigned char   spi_out[5];
    unsigned char   spi_in[5];
    unsigned int    reg_out[5];
    unsigned int    reg_in[5];
    unsigned int    reg_in_old[5];
    unsigned char   byte0;
    unsigned char   byte1;
    unsigned int    used[5];
    unsigned char   lo_count;
    unsigned char   hi_count;
    unsigned int    offset;
    unsigned int    mask;
    unsigned int    uTemp;

    set_default_used(used);   /* not available for test (no ground, power, leds, etc) */
    do
    {

        print("\r\n");
        print("diags menu\r\n");
        print(" 1 card edge input  test\r\n");
        print(" 2 card edge output test\r\n");
        print(" 3 tester_init_and_uut_power_verify\r\n");
        print(" 6 register tests\r\n");
        print(" 7 SPI functions\r\n");
        print(" 8 LPT functions\r\n");
        print(" 9 exit diags\r\n");

        key = get_a_key();
        switch (key)
        {
        case '1':   /* card edge test   */
            index = 999;
            for (i = 0; i < 5; i++) reg_in_old[i] = 0;
            do
            {
                if (index > 20)
                {
                    index = 0;
                    print("\r\nhit \'1\' to quit\r\n");
                    print("green led:  on = all power good (UUT Power Pins (5V) high; Ground pins low\r\n");
                    print("red2  led:  on = only one pin grounded\r\n");
                    print("pullups on, test with 100k to ground;     Ground Pin; Power pin\r\n");
                    print("<---------------SLOT A-------------><---------------SLOT B------------->\r\n");
                    for (i = 0; i < TEST_COLUMNS; i++)
                    {
                        sprintf(print_buffer, "%c", get_pin_type_char(i));
                        print(print_buffer);
                    }
                    print("\r\n");
                    print("ABCDEFHJKLMNPRSTUVABCDEFHJKLMNPRSTUVABCDEFHJKLMNPRSTUVABCDEFHJKLMNPRSTUV\r\n");
                    print("111111111111111111222222222222222222111111111111111111222222222222222222\r\n");
                }

                set_default_directions(reg_out);  /* all inputs + fixed outputs   */
                reg_write(REG_IODIR, reg_out);

                set_default_pullups(reg_out);
                for (i = 0; i < 5; i++) reg_out[i] |= (~used[i]);   /* pullups on unused pins */
                reg_write(REG_GPPU, reg_out);

                reg_read(REG_GPIO, reg_in);
                if (index == 0)
                {
                    for (i = 0; i < 5; i++) reg_in_old[i] = ~reg_in[i]; /* force printout   */
                }

                for (i = 0; i < 5; i++) if (reg_in[i] != reg_in_old[i]) break;
                if (i != 5)
                {
                    index++;
                    lo_count = 0;
                    hi_count = 0;
                    for (i = 0; i < TEST_COLUMNS; i++)
                    {
                        offset = mapping[i].offset;
                        mask = mapping[i].mask;
                        if (reg_in[offset] & mask)
                        {
                            /* new bit is high  */
                            if (reg_in_old[offset] & mask)
                            {
                                /* old bit is high (no change)  */
                                print(" ");
                            }
                            else
                            {
                                /* old bit is low (changed)     */
                                print("1");
                            }
                            if ((used[offset] & mask) == 0) /* test pin?    */
                            {
                                hi_count++;
                            }
                        }
                        else
                        {
                            /* new bit is low   */
                            if (reg_in_old[offset] & mask)
                            {
                                /* old bit is high (changed)    */
                                print("0");
                            }
                            else
                            {
                                /* old bit is low (no change)   */
                                print(" ");
                            }

                            if ((used[offset] & mask) == 0)     /* test pin?    */
                            {
                                lo_count++;
                            }
                        }
                    }
                    print("\r\n");
                }
                for (i = 0; i < 5; i++) reg_in_old[i] = reg_in[i];
                set_default_outputs(reg_out);
                if (verify_power(reg_in)) stage_pin(PIN_LED_GREEN, 1, reg_out);
                if (lo_count == 1) stage_pin(PIN_LED_RED2, 1, reg_out);
                reg_write(REG_OLAT, reg_out);
            } while (Serial.available()<=0 /*!_kbhit()*/);
            if (Serial.available()>0/*_kbhit()*/) key = get_a_key();
            key = '1';      /* make sure it's not 'quit'    */
            break;

        case '2':   /* card edge output test   */
            print("green led:  on = all power good (UUT Power Pins (5V) high; Ground pins low\r\n");

            set_default_directions(reg_out);  /* all inputs + fixed outputs   */
            reg_write(REG_IODIR, reg_out);

            set_default_pullups(reg_out);
            reg_write(REG_GPPU, reg_out);

            for (i = 0; i < 5; i++) reg_in_old[i] = 0;

            index = 999;
            do
            {
                if (index > 79)
                {
                    index = 0;
                    print("\r\n");
                    print("walking one output                       Ground Pin; Power pin\r\n");
                    print("<---------------SLOT A-------------><---------------SLOT B------------->\r\n");
                    for (i = 0; i < TEST_COLUMNS; i++)
                    {
                        sprintf(print_buffer, "%c", get_pin_type_char(i));
                        print(print_buffer);
                    }
                    print("\r\n");
                    print("ABCDEFHJKLMNPRSTUVABCDEFHJKLMNPRSTUVABCDEFHJKLMNPRSTUVABCDEFHJKLMNPRSTUV\r\n");
                    print("111111111111111111222222222222222222111111111111111111222222222222222222\r\n");

                }

                for (i = 0; i < 5; i++) reg_out[i] = 0;
                reg_out[mapping[index].offset] |= mapping[index].mask;
                reg_write(REG_GPIO, reg_out);
                reg_read(REG_GPIO, reg_in);
                if (index == 0)
                {
                    reg_in_old[0] = ~reg_in[0]; /* force printout   */
                }

                for (i = 0; i < 5; i++) if (reg_in[i] != reg_in_old[i]) break;
                if (i != 5)
                {
                    index++;
                    lo_count = 0;
                    hi_count = 0;
                    for (i = 0; i < TEST_COLUMNS; i++)
                    {
                        offset = mapping[i].offset;
                        mask = mapping[i].mask;
                        if (reg_in[offset] & mask)
                        {
                            /* new bit is high  */
                            if (reg_in_old[offset] & mask)
                            {
                                /* old bit is high (no change)  */
                                print(" ");
                            }
                            else
                            {
                                /* old bit is low (changed)     */
                                print("1");
                            }

                            if ((used[offset] & mask) == 0)     /* test pin?    */
                            {
                                hi_count++;
                            }
                        }
                        else
                        {
                            /* new bit is low   */
                            if (reg_in_old[offset] & mask)
                            {
                                /* old bit is high (changed)    */
                                print("0");
                            }
                            else
                            {
                                /* old bit is low (no change)   */
                                print(" ");
                            }
                            if ((used[offset] & mask) == 0)     /* test pin?    */
                            {
                                lo_count++;
                            }
                        }
                    }
                    print("\r\n");
                    for (i = 0; i < 5; i++) reg_in_old[i] = reg_in[i];
                    set_default_outputs(reg_out);
                    if (verify_power(reg_in)) stage_pin(PIN_LED_GREEN, 1, reg_out);
                    if (hi_count == 1)          stage_pin(PIN_LED_RED, 1, reg_out);
                    reg_write(REG_OLAT, reg_out);
                }
            } while (Serial.available()<=0/*!_kbhit()*/);
            if (Serial.available()>0/*_kbhit()*/) key = get_a_key();
            key = '1';      /* make sure it's not 'quit'    */
            break;

        case '3':   /* tester_init_and_uut_power_verify */
            print("calling 'tester_init_and_uut_power_verify'\r\n");
            uTemp = tester_init_and_uut_power_verify();
            sprintf(print_buffer, "tester_init_and_uut_power_verify() returns %u\r\n", uTemp);
            print(print_buffer);
            if (uTemp == 0) print("UUT power is okay\r\n");
            if (uTemp != 0) print("Error occurred or power is not on\r\n");
            break;

        case '6':   /* register tests   */
            do
            {
                print("\r\nregister tests menu\r\n");
                print(" 1 reg_init()\r\n");
                print(" 2 reg_init_and_verify (reset chips)\r\n");
                print(" 3 inputs (with pullups)\r\n");
                print(" 4 outputs toggle\r\n");
                print(" 5 read all registers\r\n");
                print(" 6 read GPIO registers (no changes)\r\n");
                print(" 9 quit register tests\r\n");

                key = get_a_key_convert_to_upper();
                switch (key)
                {
                case '1':   /* reg_init()  */
                    print("calling 'reg_init'\r\n");
                    reg_init();
                    break;

                case '2':   /* reg_init_and_verify      */
                    print("calling 'reg_init_and_verify()'\r\n");
                    uTemp = reg_init_and_verify();     /* leaves all pins inputs; output latches = 0   */
                    sprintf(print_buffer, "reg_init_and_verify() returns %u\r\n", uTemp);
                    print(print_buffer);
                    if (uTemp == 0) print("initialize was verified\r\n");
                    if (uTemp != 0) print("ERROR, initialize did not verify\r\n");
                    break;

                case '3':   /* inputs (with pullups)    */
                    print("  A print chip A in binary\r\n");
                    print("  B print chip B in binary\r\n");
                    print("  C print chip C in binary\r\n");
                    print("  D print chip D in binary\r\n");
                    print("  E print chip E in binary\r\n");
                    print("  F no print (scope)\r\n");
                    key = get_a_key_convert_to_upper();

                    print("setting to inputs\r\n");
                    reg_init(); // added by gw, SI looks like there's contention
                    set_default_directions(reg_out);  /* all inputs + fixed outputs   */
                    reg_write(REG_IODIR, reg_out);

                    print("setting pullups\r\n");
                    for (i = 0; i < 5; i++) reg_out[i] = 0xFFFF;    /* direction in     */
                    reg_write(REG_GPPU, reg_out);

                    reg = 0x12;
                    do
                    {
                        if (key != 'F')
                        {
                            print_reg_name(reg);
                            sprintf(print_buffer, " %02X/%02X", reg + 1, reg);
                            print(print_buffer);
                        }
                        reg_read(reg, reg_in);
                        if (key != 'F')
                        {
                            for (i = 0; i < 5; i++)
                            {
                                byte0 = (unsigned char)(reg_in[i] >> 0);
                                byte1 = (unsigned char)(reg_in[i] >> 8);
                                sprintf(print_buffer, "  %04Xh", reg_in[i]);
                                print(print_buffer);
                                if (i == (key - 'A'))
                                {
                                    print(" "); print_bin8(byte1);
                                    print(" "); print_bin8(byte0);
                                }
                            }
                            print("\r\n");
                        }
                        if (Serial.available()>0/*_kbhit()*/) key = get_a_key_convert_to_upper();
                    } while ((key >= 'A') && (key <= 'F'));
                    key = '3';      /* make sure it's not 'quit'    */
                    break;

                case '4':   /* output toggle    */
                    print("toggling outputs\r\n");

                    for (i = 0; i < 5; i++) reg_out[i] = 0;     /* direction out    */
                    reg_write(REG_IODIR, reg_out);

                    do
                    {
                        print(" S or space  single step\r\n");
                        print(" G           go (run)\r\n");
                        print(" Q           quit\r\n");
                        key = get_a_key_convert_to_upper();

                        index = 0;
                        do
                        {
                            for (i = 0; i < 5; i++) reg_out[i] = ~reg_out[i];
                            reg_write(REG_OLAT, reg_out);         /* set output latches   */
                            index++;
                            if (index > 20)
                            {
                                index = 0;
                                if (Serial.available()>0/*_kbhit()*/) break;
                            }
                        } while (key == 'G');
                    } while (key != 'Q');
                    if (Serial.available()>0/*_kbhit()*/) key = get_a_key();
                    key = '4';      /* make sure it's not 'quit'    */
                    break;

                case '5':   /* read all registers   */
                    print("\r\n");
                    print("\r\n");
                    print("  A print chip A in binary\r\n");
                    print("  B print chip B in binary\r\n");
                    print("  C print chip C in binary\r\n");
                    print("  D print chip D in binary\r\n");
                    print("  E print chip E in binary\r\n");
                    print("  F no print (scope)\r\n");
                    key = get_a_key_convert_to_upper();
                    do
                    {
                        if (key != 'F') print("\r\n");
                        for (reg = 0; reg < 0x16; reg += 2)
                        {
                            if (key != 'F')
                            {
                                print_reg_name(reg);
                                sprintf(print_buffer, " %02X/%02X", reg, reg + 1);
                                print(print_buffer);
                            }
                            reg_read(reg, reg_in);
                            if (key != 'F')
                            {
                                for (i = 0; i < 5; i++)
                                {
                                    byte0 = (unsigned char)(reg_in[i] >> 0);
                                    byte1 = (unsigned char)(reg_in[i] >> 8);
                                    sprintf(print_buffer, "  %02Xh %02Xh", byte0, byte1);
                                    print(print_buffer);
                                    if (i == (key - 'A'))
                                    {
                                        print(" "); print_bin8(byte0);
                                        print(" "); print_bin8(byte1);
                                    }
                                }
                                print("\r\n");
                            }
                        }
                        //for (delay = 0UL; delay < 100000UL; delay++) spi_output();
                        // some delay loop, not sure how long, seems to use SPI for predictable time?
                        delay(2); // temporary substitute for the delay loop above
                        if (Serial.available()>0/*_kbhit()*/) key = get_a_key_convert_to_upper();
                    } while ((key >= 'A') && (key <= 'F'));
                    key = '5';      /* make sure it's not 'quit'    */
                    break;

                case '6':   /* read GPIO registers (no changes) */
                    print("\r\n");
                    print("\r\n");
                    print("  A print chip A in binary\r\n");
                    print("  B print chip B in binary\r\n");
                    print("  C print chip C in binary\r\n");
                    print("  D print chip D in binary\r\n");
                    print("  E print chip E in binary\r\n");
                    print("  F no print (scope)\r\n");
                    key = get_a_key_convert_to_upper();
                    reg = 0x12;
                    do
                    {
                        if (key != 'F')
                        {
                            print_reg_name(reg);
                            sprintf(print_buffer, " %02X/%02X", reg + 1, reg);
                            print(print_buffer);
                        }
                        reg_read(reg, reg_in);
                        if (key != 'F')
                        {
                            for (i = 0; i < 5; i++)
                            {
                                byte0 = (unsigned char)(reg_in[i] >> 0);
                                byte1 = (unsigned char)(reg_in[i] >> 8);
                                sprintf(print_buffer, "  %04Xh", reg_in[i]);
                                print(print_buffer);
                                if (i == (key - 'A'))
                                {
                                    print(" "); print_bin8(byte1);
                                    print(" "); print_bin8(byte0);
                                }
                            }
                            print("\r\n");
                        }
                        if (Serial.available()>0/*_kbhit()*/) key = get_a_key_convert_to_upper();
                    } while ((key >= 'A') && (key <= 'F'));
                    key = '6';      /* make sure it's not 'quit'    */
                    break;

                case '9':   /* quit register tests  */
                    break;
                }
            } while (key != '9');
            key = '6';      /* make sure it's not 'quit'    */
            break;

        case '7':   /* SPI functions    */
            print("\r\n");
            print("\r\nNo SPI Functions for Arduino, SPI is in hardware\r\n");
            break;
//            print("  1 spi_init()\r\n");
//            print("  2 spi_open()\r\n");
//            print("  3 spi_close()\r\n");
//            print("  4 spi_open(),spi_close (scope)\r\n");
//            print("  5 spi_txrx()\r\n");
//            print("  6 spi_txrx(), no print (scope)\r\n");
//            key = get_a_key();
//            switch (key)
//            {
//            case '1':
//                print("calling spi_init()");
//                spi_init();
//                print("\r\n");
//                break;
//            case '2':
//                print("calling spi_open()");
//                spi_open();
//                print("\r\n");
//                break;
//            case '3':
//                print("calling spi_close()");
//                spi_close();
//                print("\r\n");
//                break;
//            case '4':
//                print("\r\ntesting spi_open, spi_close\r\n");
//                index = 0;
//                for (;;)        /* break on _kbhit() */
//                {
//                    spi_open();
//                    spi_close();
//                    index++;
//                    if (index > 9)
//                    {
//                        index = 0;
//                        if (Serial.available()>0/*_kbhit()*/) break;
//                    }
//                }
//                if (Serial.available()>0/*_kbhit()*/) key = get_a_key();
//                break;
//            case '5':
//            case '6':
//                print("\r\ntesting spi_txrx\r\n");
//                index = 0;
//                for (;;)        /* break on _kbhit() */
//                {
//                    for (i = 0; i < 5; i++) spi_out[i] = 0;
//                    for (i = 0; i < 5; i++) spi_in[i] = 0;
//                    spi_out[index / 8] = (unsigned char)(1 << (7 - (index % 8)));
//                    spi_txrx(spi_out, spi_in);
//                    if (key == '5')     /* print? */
//                    {
//                        for (i = 0; i < 5; i++)
//                        {
//                            if ((index / 8) == i)
//                            {
//                                sprintf(print_buffer, " %c", (index / 8) + 'A');
//                                print(print_buffer);
//                                sprintf(print_buffer, "%u ", 7 - (index % 7));
//                                print(print_buffer);
//                            }
//                            else
//                            {
//                                print("    ");
//                            }
//                            print_bin8(spi_in[i]);
//                        }
//                        print("b\r\n");
//                    }
//                    index++;
//                    if (index > 39)
//                    {
//                        index = 0;
//                        if (key == '6')
//                        {
//                            if (Serial.available()>0/*_kbhit()*/) break;
//                        }
//                    }
//                    if (key == '5')     /* print?   */
//                    {
//                        for (i = 0; i < 10000; i++) spi_output();
//                        if (Serial.available()>0/*_kbhit()*/) break;
//                    }
//                }
//                if (Serial.available()>0/*_kbhit()*/) key = get_a_key();
//                key = '5';      /* make sure it's not 'quit'    */
//                break;
//            default:
//                break;
//            }
//            break;
        case '8':   /* LPT function     */
            print("\r\n");
            print("\r\nNo LPT functions in Arduino. SPI is in hardware.\r\n");
            break;
//            print("  D change DATA    output\r\n");
//            print("  C change CONTROL output\r\n");
//            print("  S read   STATUS  inputs\r\n");
//            print("  R SPI init\r\n");
//            key = get_a_key();
//            switch (key)
//            {
//            case 'D':
//            case 'd':
//            case 'C':
//            case 'c':
//            case 'S':
//            case 's':
//                print("COMMAND  -----DATA-----  ----CONTROL---  ----STATUS----  ---CHANGED----\r\n");
//                command = toupper(key);
//                lpt_init();
//                status = lpt_value_status_last_read;
//                last_status = dstatus;
//                for (;;)
//                {
//                    sprintf(print_buffer, "\r\n   %c     ", command);
//                    print(print_buffer);
//                    sprintf(print_buffer, "0x%02X ", lpt_value_last_data_out);
//                    print(print_buffer);
//                    print_bin8((unsigned char)lpt_value_last_data_out);
//                    sprintf(print_buffer, "b  0x%02X ", lpt_value_last_control_out);
//                    print(print_buffer);
//                    print_bin8((unsigned char)lpt_value_last_control_out);
//                    sprintf(print_buffer, "b  0x%02X ", dstatus);
//                    print(print_buffer);
//                    print_bin8(dstatus);
//                    sprintf(print_buffer, "b  0x%02X ", dstatus ^ last_status);
//                    print(print_buffer);
//                    print_bin8((unsigned char)(dstatus ^ last_status));
//                    print("b");
//                    last_status = dstatus;
//
//                    do
//                    {
//                        lpt_input_status();
//                        dstatus = lpt_value_status_last_read;
//                    } while ((dstatus == last_status) && (Serial.available()<=0/*!_kbhit()*/));
//
//                    if (Serial.available()>0/*_kbhit()*/)
//                    {
//                        key = get_a_key();
//                        if ((key >= '0') && (key <= '7'))
//                        {
//                            if (command == 'D') lpt_toggle_data_bit(key - '0');
//                            if (command == 'C') lpt_toggle_control_bit(key - '0');
//                        }
//                        else if ((key == 'D') ||
//                            (key == 'd') ||
//                            (key == 'C') ||
//                            (key == 'c'))
//                        {
//                            command = toupper(key);
//                        }
//                        else
//                        {
//                            key = '1';      /* make sure it's not 'quit'    */
//                            break;      /* unknown key exits    */
//                        }
//                    }
//                    lpt_output_control();
//                    lpt_output_data();
//                }
//                break;
//            case 'R':
//            case 'r':
//                spi_init();
//                break;
//            default:
//                break;
//            }
//            break;
//        case '9':
//            break;
        }
    } while (key != '9');
}




#define MAX_STRING  120
#define MAX_LINES   4000
//#define MAX_LINES   2000
//#define MAX_STEPS   2200
#define MAX_STEPS   1700

struct
{
    unsigned int    number_lines;
    char* line[MAX_LINES];
} raw_test;


#define TEST_FILENAME_NONE  "<none>"
#define FAILURE_MODE_MASK_I_LO_J_LO     (1 << 0)
#define FAILURE_MODE_MASK_I_LO_J_HI     (1 << 1)
#define FAILURE_MODE_MASK_I_HI_J_LO     (1 << 2)
#define FAILURE_MODE_MASK_I_HI_J_HI     (1 << 3)

struct
{
    char            filename[MAX_STRING];
    unsigned int    number_columns;
    unsigned int    number_steps;
    unsigned int    raw_line_pins;
    unsigned int    raw_line_direction;
    struct
    {
        unsigned char   slot;
        unsigned char   edge;
        unsigned char   side;
        unsigned char   pdirection;
        unsigned int    mapping_index;
        unsigned int    offset;
        unsigned int    mask;
        char            string[MAX_STRING];
        unsigned char   failure_mode[TEST_COLUMNS];
    } columns[TEST_COLUMNS];
    unsigned int    pdirection[5];
    unsigned int    pullup[5];
    struct
    {
        unsigned int raw_line_test;
        unsigned int dont_care[5];
        unsigned int out_data[5];
    } step[MAX_STEPS];
} test;


int read_test(/*File* file_test*/)    /* returns 0 if no error    */
{
    int             result;
    int             compare;
    unsigned int    i;
    unsigned int    j;
    unsigned int    uTemp;
    unsigned int    offset;
    unsigned int    mask;
    unsigned int    pin_used[5];
    char            rtbuffer[MAX_STRING];
    char            character;
    char            dir_character;
    unsigned int    pdirection;
    char            string[MAX_STRING];
    unsigned int    length;
    unsigned int    number_UUT_inputs;
    unsigned int    number_UUT_outputs;
    char            previous[MAX_STRING];
    char* ptr;
    unsigned int    pin_slot;
    unsigned int    pin_edge;
    unsigned int    pin_side;
    boolean         endfound;

    /* free any mallocs from previous test  */
    Serial.println(" *read_test entry");
    endfound = false;
    while (raw_test.number_lines != 0)
    {
        raw_test.number_lines--;
        if (raw_test.line[raw_test.number_lines] != (void*)NULL)
        {
            free(raw_test.line[raw_test.number_lines]);
        }
    }

    result = 0;     /* assume success   */

    number_UUT_inputs = 0;
    number_UUT_outputs = 0;

    /* initialize test struct   */
    test.number_columns = 0;
    test.number_steps = 0;
    test.raw_line_pins = 0;
    test.raw_line_direction = 0;
    set_default_directions(test.pdirection);   /* all inputs + fixed outputs   */
    set_default_pullups(test.pullup);      /* test input pullups           */

    set_default_used(pin_used);   /* not available for test   */
    Serial.println(" *read_test begin file reading");
    for (;;)        /* will break on error or success   */
    //while(true)  // updated to while statement for clarity -gw
    {
        /* print comments until 'PINS'  */
        Serial.println("   *read_test print comments start");
        while (file_test.available())
        {
            Serial.println("   *read_test while(available) loop start");
            if (!file_test.available()) break;; // moved the data available test to the beginning
            sd_fgets(rtbuffer, sizeof(rtbuffer)/*, file_test*/);
            Serial.println(String(rtbuffer)); // for testing to view the file contents
            //if (!file_test.available()) break;;

            //ptr = strchr(rtbuffer, '\n');
            //if (ptr != (char*)NULL) *ptr = '\0';
            //ptr = strchr(rtbuffer, '\r');
            //if (ptr != (char*)NULL) *ptr = '\0';

            if (raw_test.number_lines >= MAX_LINES)
            {
                sprintf(print_buffer, "error: test file is too long\r\n");
                print(print_buffer);
                result = 1;
                break;              /* break on error   */
            }
            ptr = (char*)malloc(strlen(rtbuffer) + 1);
            if (ptr == (char*)NULL)
            {
                sprintf(print_buffer, "error: malloc() failed, out of memory!\r\n");
                print(print_buffer);
                result = 1;
                break;              /* break on error   */
            }
            strcpy(ptr, rtbuffer);
            raw_test.line[raw_test.number_lines] = ptr;
            raw_test.number_lines++;

            (void)str_to_upr(rtbuffer);
            compare = strcmp("PINS", rtbuffer);
            if (compare != 0)
            {
                sprintf(print_buffer, "  comment: %s\r\n", ptr);
                print(print_buffer);
            }
            else
            {
                test.raw_line_pins = raw_test.number_lines;
                sprintf(print_buffer, "     pins: %s\r\n", ptr);        /* show file contents  */
                print(print_buffer);
                break;
            }
        }
        if (!file_test.available())
        {
            print("error: did not find 'PINS' line\r\n");
            result = 1;         /* error    */
        }
        if (result == 1) break;          /* break on error   */

        /* process 'PINS'   */
        while (file_test.available())
        {
            sd_fgets(rtbuffer, sizeof(rtbuffer)/*, file_test*/);
            //if (!file_test.available()) break;;

            //ptr = strchr(rtbuffer, '\n');
            //if (ptr != (char*)NULL) *ptr = '\0';
            //ptr = strchr(rtbuffer, '\r');
            //if (ptr != (char*)NULL) *ptr = '\0';

            if (raw_test.number_lines >= MAX_LINES)
            {
                sprintf(print_buffer, "error: test file is too long\r\n");
                print(print_buffer);
                result = 1;
                break;              /* break on error   */
            }
            ptr = (char*)malloc(strlen(rtbuffer) + 1);
            if (ptr == (char*)NULL)
            {
                sprintf(print_buffer, "error: malloc() failed, out of memory!\r\n");
                print(print_buffer);
                result = 1;
                break;              /* break on error   */
            }
            strcpy(ptr, rtbuffer);
            raw_test.line[raw_test.number_lines] = ptr;
            raw_test.number_lines++;

            (void)str_to_upr(rtbuffer);
            sprintf(print_buffer, "     pins: %s\r\n", rtbuffer);        /* show file contents  */
            print(print_buffer);
            if (strlen(rtbuffer) == 0) break;           /* blank line ends PINS */

            if ((strlen(rtbuffer) + 1) >= sizeof(test.columns[0].string))
            {
                print("error: line is too long\r\n");
                result = 1;
                break;
            }
            strcpy(test.columns[test.number_columns].string, rtbuffer);


            sscanf(rtbuffer, " %u %c %80s", &i, &dir_character, string);   /* MAX_STRING   */
            if (i != (test.number_columns + 1))
            {
                sprintf(print_buffer, "error: expected column %d: %s\r\n", test.number_columns + 1, rtbuffer);
                print(print_buffer);
                result = 1;     /* error    */
                break;
            }
            if (test.number_columns >= (sizeof(test.columns) / sizeof(test.columns[0])))
            {
                print("error: too many columns\r\n");
                result = 1;
                break;
            }
            if ((dir_character != 'I') && (dir_character != 'O') && (dir_character != 'P'))
            {
                print("error: expected 'I' (input) or 'O' (output) or 'P' (open drain output + pullup)\r\n");
                result = 1;
                break;
            }
            if (strlen(string) < 3)
            {
                print("error: expected pin location (ex: BV2)\r\n");
                result = 1;
                break;
            }

            /* get pin location */
            /* process SLOT */
            character = string[0];
            if ((character != 'A') && (character != 'B'))
            {
                print("error: expected SLOT 'A' or 'B'\r\n");
                result = 1;
                break;
            }
            pin_slot = character - 'A';
            test.columns[test.number_columns].slot = character;

            character = string[1];
            ptr = (char *) strchr(edge_pins, character);
            if (ptr == (char*)NULL)
            {
                sprintf(print_buffer, "error: expected EDGE CONNECTOR PIN: %s\r\n", edge_pins);
                print(print_buffer);
                result = 1;
                break;          /* error    */
            }
            pin_edge = (unsigned int)(ptr - edge_pins); /* pin index    */
            test.columns[test.number_columns].edge = character;

            character = string[2];
            if ((character != '1') && (character != '2'))
            {
                print("error: expected EDGE CONNECTOR PIN, side 1 or 2\r\n");
                result = 1;
                break;          /* error    */
            }
            pin_side = character - '1';
            test.columns[test.number_columns].side = character;

            /* calculate mapping index  */
            uTemp = pin_edge + (pin_side * 18) + (pin_slot * 36);

            offset = mapping[uTemp].offset;
            mask = mapping[uTemp].mask;

            test.columns[test.number_columns].mapping_index = uTemp;
            test.columns[test.number_columns].offset = offset;
            test.columns[test.number_columns].mask = mask;

            test.columns[test.number_columns].pdirection = dir_character;
            if (dir_character == 'I')           /* UUT input is tester output   */
            {
                /* setup tester output  */
                test.pdirection[offset] &= (~mask);
            }

            if (dir_character == 'P')           /* UUT open drain output is tester input w/pullup   */
            {
                /* setup tester pullup  */
                test.pullup[offset] |= (mask);
            }

            if (pin_used[offset] & mask)    /* already used?    */
            {
                print("error: PIN is already used\r\n");
                sprintf(print_buffer, "column %u is not a unique pin (another column uses it)\r\n", test.number_columns + 1);
                print(print_buffer);
                result = 1;
                break;          /* error    */
            }
            pin_used[offset] |= mask;   /* mark as used */

            test.number_columns++;
        }
        if (!file_test.available())
        {
            print("error: did not find blank line following 'PINS'");
            result = 1;         /* error    */
            break;
        }
        if (test.number_columns == 0)
        {
            print("error: no PINS columns\r\n");
            result = 1;
            break;
        }

        /* process direction line   */
        while (file_test.available())
        {
            sd_fgets(rtbuffer, sizeof(rtbuffer)/*, file_test*/);
            Serial.println("rtb[" + String(strlen(rtbuffer)) + "]->" + String(rtbuffer));
            //if (!file_test.available()) break;; // this causes us to miss the END command at the end of the test file

            (void)str_to_upr(rtbuffer);
            //ptr = strchr(rtbuffer, '\n');
            //if (ptr != (char*)NULL) *ptr = '\0';
            //ptr = strchr(rtbuffer, '\r');
            //if (ptr != (char*)NULL) *ptr = '\0';

            if (raw_test.number_lines >= MAX_LINES)
            {
                sprintf(print_buffer, "error: test file is too long\r\n");
                print(print_buffer);
                result = 1;
                break;              /* break on error   */
            }
            ptr = (char*)malloc(strlen(rtbuffer) + 1);
            if (ptr == (char*)NULL)
            {
                sprintf(print_buffer, "error: malloc() failed, out of memory!\r\n");
                print(print_buffer);
                result = 1;
                break;              /* break on error   */
            }
            strcpy(ptr, rtbuffer);
            raw_test.line[raw_test.number_lines] = ptr;
            test.raw_line_direction = raw_test.number_lines;
            raw_test.number_lines++;

            sprintf(print_buffer, "direction: %s\r\n", rtbuffer);         /* show file contents  */
            print(print_buffer);
            length = strlen(rtbuffer);
            if (length != test.number_columns)
            {
                sprintf(print_buffer, "expected 'direction' (%d columns of 'I' or 'O' or 'P')\r\n", test.number_columns);
                print(print_buffer);
                result = 1;     /* error    */
                break;
            }
            for (i = 0; i < length; i++)
            {
                offset = test.columns[i].offset;
                mask = test.columns[i].mask;
                character = rtbuffer[i];
                if (test.pdirection[offset] & mask)
                {
                    dir_character = 'O';    /* tester input; UUT output */
                    if (test.pullup[offset] & mask)
                    {
                        dir_character = 'P';    /* tester input w/pullup; UUT output    */
                    }
                }
                else
                {
                    dir_character = 'I';    /* tester output; UUT input */
                }
                if (character != dir_character)
                {
                    print("error: direction did not match PIN line\r\n");
                    result = 1;
                    break;
                }

                if (character == 'I')
                {
                    /* UUT is an input                  */
                    number_UUT_inputs++;
                }
                else if ((character == 'O') || (character == 'P'))
                {
                    /* UUT is an output                 */
                    number_UUT_outputs++;
                }
                else
                {
                    print("error: expected 'I' or 'O' or 'P' for pin direction\r\n");
                    result = 1;     /* error    */
                    break;
                }
            }
            strcpy(previous, rtbuffer);     /* use direction line (correct length)    */
            break;
        }
        if (result) break;      /* break on error   */


        /* process test steps */

        while (file_test.available())
        {
            sd_fgets(rtbuffer, sizeof(rtbuffer)/*, file_test*/);
            //if (!file_test.available()) break;;

            //ptr = strchr(rtbuffer, '\n');
            //if (ptr != (char*)NULL) *ptr = '\0';
            //ptr = strchr(rtbuffer, '\r');
            //if (ptr != (char*)NULL) *ptr = '\0';

            if (raw_test.number_lines >= MAX_LINES)
            {
                sprintf(print_buffer, "error: test file is too long\r\n");
                print(print_buffer);
                result = 1;
                break;              /* break on error   */
            }
            ptr = (char*)malloc(strlen(rtbuffer) + 1);
            if (ptr == (char*)NULL)
            {
                sprintf(print_buffer, "error: malloc() failed, out of memory!\r\n");
                print(print_buffer);
                result = 1;
                break;              /* break on error   */
            }
            strcpy(ptr, rtbuffer);
            raw_test.line[raw_test.number_lines] = ptr;
            test.step[test.number_steps].raw_line_test = raw_test.number_lines;
            raw_test.number_lines++;

            if ((rtbuffer[0] == ';') || (rtbuffer[0] == 0))
            {
                sprintf(print_buffer, "  comment: %s\r\n", rtbuffer);    /* show file contents  */
                print(print_buffer);
                continue;
            }

            compare = strcmp("END", rtbuffer);
            if (compare == 0)
            {
                sprintf(print_buffer, "      end: %s\r\n", rtbuffer);    /* show file contents  */
                print(print_buffer);
                endfound = true;
                break;
            }
            else
            {
                sprintf(print_buffer, "test %4u: %s\r\n", test.number_steps + 1, rtbuffer);  /* show file contents  */
                print(print_buffer);
            }
            length = strlen(rtbuffer);
            if (length > test.number_columns)
            {
                print("'test step' is too long\r\n");
                sprintf(print_buffer, "expected 'test step' (%d columns of '0','1','X', or ' ')\r\n", test.number_columns);
                print(print_buffer);
                result = 1;     /* error    */
                break;
            }
            if (length < test.number_columns)
            {
                if (test.number_steps == 0)
                {
                    print("error: the first 'test step' is too short\r\n");
                    print("error: it must set EVERY column\r\n");
                    sprintf(print_buffer, "expected 'test step' (%d columns of '0','1','X', or ' ')\r\n", test.number_columns);
                    print(print_buffer);
                    result = 1;     /* error    */
                    break;
                }
                while (strlen(rtbuffer) < test.number_columns) strcat(rtbuffer, " ");
                length = strlen(rtbuffer);
            }
            if (test.number_steps >= MAX_STEPS)
            {
                sprintf(print_buffer, "error: too many test steps: %u\r\n", test.number_steps);
                print(print_buffer);
                result = 1;
                break;      /* error    */
            }
            /* initial the test step entry  */
            for (i = 0; i < 5; i++)
            {
                test.step[test.number_steps].dont_care[i] = 0x0000;
                test.step[test.number_steps].out_data[i] = 0x0000;
            }

            /* decode the test step */
            for (i = 0; i < length; i++)
            {
                character = rtbuffer[i];
                if (character == ' ')
                {
                    rtbuffer[i] = previous[i];    /* use previous value   */
                }

                character = rtbuffer[i];
                previous[i] = character;        /* store in previous    */

                offset = test.columns[i].offset;
                mask = test.columns[i].mask;
                pdirection = test.pdirection[offset] & mask;

                if (character == '0')
                {
                    test.step[test.number_steps].out_data[offset] &= ~mask;
                }
                else if (character == '1')
                {
                    test.step[test.number_steps].out_data[offset] |= mask;
                }
                else if (character == 'X')
                {
                    test.step[test.number_steps].dont_care[offset] |= mask;
                }
                else
                {
                    sprintf(print_buffer, "error: unexpected character: 0x%02X %c\r\n", character, character);
                    print(print_buffer);
                    print("error: expected '0', '1', or 'X' for test step\r\n");
                    result = 1;     /* error    */
                    break;
                }
            }
            test.number_steps++;
        }
        if (result) break;      /* break on error   */
        if (!file_test.available() && !endfound)
        {
            print("error: did not find 'END' following 'test steps'\r\n");
            Serial.println("number of steps: " + String(test.number_steps));
            result = 1;         /* error    */
        }
        if (test.number_steps == 0)
        {
            print("error: no TEST STEPs\r\n");
            result = 1;
        }
        if (result) break;      /* break on error   */

        print("\r\n");
        print("\r\n");
        print("summary\r\n");
        print("\r\n");
        for (i = 0; i < test.number_columns; i++)
        {
            sprintf(print_buffer, "column %2u: offset %u, mask 0x%04X\r\n",
                i + 1,
                test.columns[i].offset,
                test.columns[i].mask);
            print(print_buffer);
        }
        print("\r\n");
        print("direction bits (1=input)\r\n");
        sprintf(print_buffer, "   0x%04X 0x%04X 0x%04X 0x%04X 0x%04X\r\n",
            test.pdirection[0],
            test.pdirection[1],
            test.pdirection[2],
            test.pdirection[3],
            test.pdirection[4]);
        print(print_buffer);
        print("\r\n");
        print("pullup bits (1=pullup)\r\n");
        sprintf(print_buffer, "   0x%04X 0x%04X 0x%04X 0x%04X 0x%04X\r\n",
            test.pullup[0],
            test.pullup[1],
            test.pullup[2],
            test.pullup[3],
            test.pullup[4]);
        print(print_buffer);
        print("\r\n");
        print("test step    out_data                     dont care\r\n");
        for (i = 0; i < test.number_steps; i++)
        {
            sprintf(print_buffer, "%4u:", i + 1);
            print(print_buffer);
            uTemp = 0;
            for (j = 0; j < 5; j++)
            {
                sprintf(print_buffer, " 0x%04X", test.step[i].out_data[j]);
                print(print_buffer);
                uTemp |= test.step[i].dont_care[j];
            }
            if (uTemp)
            {
                print(" ");
                for (j = 0; j < 5; j++)
                {
                    sprintf(print_buffer, " 0x%04X", test.step[i].dont_care[j]);
                    print(print_buffer);
                }
            }
            print("\r\n");
        }

        /* print stats  */
        print("\r\n");
        print("PINs used (in edge connector order) (G=ground,P=power)\r\n");
        print("SLOT   ");
        for (i = 0; i < TEST_COLUMNS; i++)
        {
            sprintf(print_buffer, "%c", (i / 36) + 'A');
            print(print_buffer);
        }
        print("\r\n");
        print("LETTER ");
        for (i = 0; i < TEST_COLUMNS; i++)
        {
            sprintf(print_buffer, "%c", edge_pins[i % 18]);
            print(print_buffer);
        }
        print("\r\n");
        print("SIDE   ");
        for (i = 0; i < TEST_COLUMNS; i++)
        {
            sprintf(print_buffer, "%u", 1 + ((i / 18) % 2));
            print(print_buffer);
        }
        print("\r\n");
        print("USAGE  ");
        for (i = 0; i < TEST_COLUMNS; i++)
        {
            offset = mapping[i].offset;
            mask = mapping[i].mask;
            if ((i == PIN_GROUND_AT1) ||
                (i == PIN_GROUND_AC2) ||
                (i == PIN_GROUND_BT1) ||
                (i == PIN_GROUND_BC2))
            {
                print("G");
            }
            else if ((i == PIN_POWER_AA2) ||
                (i == PIN_POWER_BA2_NC))
            {
                print("P");
            }
            else if (pin_used[offset] & mask)
            {
                if (test.pdirection[offset] & mask)
                {
                    if (test.pullup[offset] & mask)
                    {
                        print("P");
                    }
                    else
                    {
                        print("O");
                    }
                }
                else
                {
                    print("I");
                }
            }
            else
            {
                print(" ");
            }
        }
        print("\r\n");

        printf("\r\n");
        sprintf(print_buffer, "UUT inputs:  %2u\r\n", number_UUT_inputs);
        print(print_buffer);
        sprintf(print_buffer, "UUT outputs: %2u\r\n", number_UUT_outputs);
        print(print_buffer);

        sprintf(print_buffer, "pins used:   %2u\r\n", test.number_columns);
        print(print_buffer);
        sprintf(print_buffer, " not used:   %2u\r\n", 66 - test.number_columns);
        print(print_buffer);
        sprintf(print_buffer, "%4u 'test steps'\r\n", test.number_steps);
        print(print_buffer);
        sprintf(print_buffer, "%4u lines\r\n", raw_test.number_lines);
        print(print_buffer);

        sprintf(print_buffer, "\r\n");
        print(print_buffer);

        for (i = 0; i < test.raw_line_pins; i++)
        {
            sprintf(print_buffer, "%s\r\n", raw_test.line[i]);
            print(print_buffer);
        }

        result = 0;     /* good file    */
        break;
    }
    if (result)     /* failed?  */
    {
        strcpy(test.filename, TEST_FILENAME_NONE);
        test.number_columns = 0;
        test.number_steps = 0;
    }
    return (result);
}



void print_pin_heading(void)
{
    unsigned int    i;


    print("SLOT      ");
    for (i = 0; i < test.number_columns; i++)
    {
        sprintf(print_buffer, "%c", test.columns[i].slot);
        print(print_buffer);
    }
    print("\r\n");

    print("LETTER    ");
    for (i = 0; i < test.number_columns; i++)
    {
        sprintf(print_buffer, "%c", test.columns[i].edge);
        print(print_buffer);
    }
    print("\r\n");

    print("SIDE      ");
    for (i = 0; i < test.number_columns; i++)
    {
        sprintf(print_buffer, "%c", test.columns[i].side);
        print(print_buffer);
    }
    print("\r\n");
}



void print_direction_heading(void)
{
    unsigned int    i;


    print("DIRECTION ");
    for (i = 0; i < test.number_columns; i++)
    {
        sprintf(print_buffer, "%c", test.columns[i].pdirection);
        print(print_buffer);
    }
    print("\r\n");
}

// set the hardware error indicator bit to trigger or view on the scope
//
void set_error_strobe_trigger() {
  digitalWrite(onboard_ledPin, HIGH); // turn on the onboard LED, also used as the error trigger for the oscilloscope
}

// clear the hardware error indicator bit to trigger or view on the scope
//
void clear_error_strobe_trigger() {
  digitalWrite(onboard_ledPin, LOW); // turn off the onboard LED, also used as the error trigger for the oscilloscope
}


void print_pin_and_direction_heading(void)
{
    print_pin_heading();
    print_direction_heading();
}



char    blank_string[MAX_STRING];
char    input_string[MAX_STRING];
char    error_string[MAX_STRING];
char    change_string[MAX_STRING];
char    test_fail_string[MAX_STRING];
char    all_fail_string[MAX_STRING];
char    lo_string[MAX_STRING];
char    hi_string[MAX_STRING];
char    rising_string[MAX_STRING];
char    falling_string[MAX_STRING];

void run_a_test(unsigned int testdelay, unsigned int trigger)
{
    unsigned long   tests_run;
    unsigned long   tests_pass;
    unsigned long   tests_fail;
    unsigned int  scope_cols_left;
    unsigned int    i;
    unsigned int    j;
    unsigned int    stepval;
    unsigned int    failure_mode_bit;
    int             key;
    int             step_fail_count;
    int             print_flag;
    int             error_flag;
    int             first_flag;
    int             comment_flag;
    int             stop_on_fail_flag;
    int             all_pins_flag;
    int             need_comment;
    int             need_change;
    int             toggle_value;
    unsigned long   delay_count;
    unsigned int    dont_care;
    unsigned int    expected;
    unsigned int    data_out[5];
    unsigned int    data_in[5];
    unsigned int    data_in_old[5];
    unsigned int    offset;
    unsigned int    mask;
    unsigned int    value_now;
    unsigned int    value_old;
    char            uut_direction;
    enum
    {
        TEST_MODE_UNKNOWN = 0,
        TEST_MODE_SINGLE_STEP,
        TEST_MODE_BACKUP,
        TEST_MODE_TOGGLE,
        TEST_MODE_RUN_ONCE,
        TEST_MODE_GO,
        TEST_MODE_STOP_ON_FAIL,
        TEST_MODE_STOP_ON_FAIL_NO_PRINT,
        TEST_MODE_SCOPE,
        TEST_MODE_QUIT,
    } test_mode;


    strcpy(blank_string, "");
    for (i = 0; i < test.number_columns; i++) strcat(blank_string, " ");
    strcpy(all_fail_string, blank_string);
    strcpy(lo_string, blank_string);
    strcpy(hi_string, blank_string);
    strcpy(rising_string, blank_string);
    strcpy(falling_string, blank_string);
    for (i = 0; i < test.number_columns; i++)
    {
        memset(test.columns[i].failure_mode, 0, TEST_COLUMNS);
    }
    comment_flag = 0;    /* no comment printout      */

    do
    {
        test_mode = TEST_MODE_UNKNOWN;
        print(" space  single step\r\n");
        print(" O      run once (one test, all steps)\r\n");
        print(" F      run, stop on fail\r\n");
        print(" G      go (run tests\r\n");
        print(" N      run, stop on fail, no print\r\n");
        print(" S      scope (run, no print)\r\n");
        print(" C      turn ");
        print((comment_flag) ? "off" : "on");
        print(" comment printout\r\n");
        print(" +      increase speed (less delay)\r\n");
        print(" -      decrease speed (more delay)(slower)\r\n");
        print(" Q      quit test\r\n");
        key = get_a_key_convert_to_upper();
        if (key == '+')
        {
            if (testdelay > 0) testdelay--;
            sprintf(print_buffer, "the delay is now %u\r\n", testdelay);
            print(print_buffer);
        }
        if (key == '-')
        {
            if (testdelay < 100) testdelay++;
            sprintf(print_buffer, "the delay is now %u\r\n", testdelay);
            print(print_buffer);
        }
        if (key == ' ') test_mode = TEST_MODE_SINGLE_STEP;
        if (key == 'O') test_mode = TEST_MODE_RUN_ONCE;
        if (key == 'F') test_mode = TEST_MODE_STOP_ON_FAIL;
        if (key == 'G') test_mode = TEST_MODE_GO;
        if (key == 'N') test_mode = TEST_MODE_STOP_ON_FAIL_NO_PRINT;
        if (key == 'S') test_mode = TEST_MODE_SCOPE;
        if (key == 'Q') test_mode = TEST_MODE_QUIT;
        if (key == 'C')
        {
            comment_flag = 1 - comment_flag;
            sprintf(print_buffer, "comment_flag is %u\r\n", comment_flag);
            print(print_buffer);
        }
    } while (test_mode == TEST_MODE_UNKNOWN);

    print_flag = 1;    /* assume printing is on    */
    stop_on_fail_flag = 0;    /* assume not stop on fail  */
    toggle_value = 0;    /* assume no toggle         */
    if (test_mode == TEST_MODE_STOP_ON_FAIL)
    {
        stop_on_fail_flag = 1;
    }
    if (test_mode == TEST_MODE_STOP_ON_FAIL_NO_PRINT)
    {
        print_flag = 0;
        stop_on_fail_flag = 1;
        print("run, stop on fail, no print\r\n");
    }
    if (test_mode == TEST_MODE_SCOPE)
    {
        print_flag = 0;
        print("scope (run, no print)\r\n");
    }

    tests_run = 0UL;
    tests_pass = 0UL;
    tests_fail = 0UL;
    first_flag = 1;     /* first time   */
    scope_cols_left = 79;

    stepval = 0;
    while (test_mode != TEST_MODE_QUIT)
    {
        if (stepval == 0)
        {
            step_fail_count = 0;
            strcpy(test_fail_string, blank_string);
            reg_write(REG_IODIR, test.pdirection);
            reg_write(REG_GPPU, test.pullup);
        }

        clear_error_strobe_trigger(); // clear the hardware error indicator trigger

        /* do one step of a test    */
        all_pins_flag = 0;
        for (i = 0; i < 5; i++) data_out[i] = test.step[stepval].out_data[i];
        if ((stepval + 1) == trigger) stage_pin(PIN_LED_RED2, 1, data_out);
        reg_write(REG_OLAT, data_out);
        reg_read(REG_GPIO, data_in);
        if (first_flag)
        {
            for (i = 0; i < 5; i++) data_in_old[i] = data_in[i];
            first_flag = 0;
        }

        /* process the test columns     */
        strcpy(error_string, blank_string);
        strcpy(input_string, blank_string);
        strcpy(change_string, blank_string);
        error_flag = 0;     /* =1 if any column is failed   */
        for (i = 0; i < test.number_columns; i++)
        {
            offset = test.columns[i].offset;
            mask = test.columns[i].mask;
            dont_care = test.step[stepval].dont_care[offset] & mask;
            expected = data_out[offset] & mask;
            value_now = data_in[offset] & mask;
            value_old = data_in_old[offset] & mask;
            if (value_now)
            {
                input_string[i] = '1';
                hi_string[i] = '1';
                if (value_old == 0)
                {
                    rising_string[i] = '^';
                    change_string[i] = '1';
                }
            }
            else
            {
                input_string[i] = '0';
                lo_string[i] = '0';
                if (value_old != 0)
                {
                    falling_string[i] = 'v';
                    change_string[i] = '0';
                }
            }
            if ((dont_care) || (value_now == expected))
            {
                error_string[i] = ' ';
            }
            else
            {
                error_flag = 1;
                error_string[i] = '^';
                if (test.pdirection[offset] & mask)
                {
                    if (test.pullup[offset] & mask)
                    {
                        uut_direction = 'P';
                    }
                    else
                    {
                        uut_direction = 'O';
                    }
                }
                else
                {
                    uut_direction = 'I';
                }
                test_fail_string[i] = uut_direction;
                all_fail_string[i] = uut_direction;
            }
        }
        if (error_flag)
        {
            step_fail_count++;
            for (i = 0; i < test.number_columns; i++)
            {
                if (error_string[i] != ' ')
                {
                    for (j = 0; j < test.number_columns; j++)
                    {
                        if (i != j)
                        {
                            failure_mode_bit = 0;
                            if (input_string[i] == '0')
                            {
                                if (input_string[j] == '0')
                                {
                                    failure_mode_bit = FAILURE_MODE_MASK_I_LO_J_LO;
                                }
                                else
                                {
                                    failure_mode_bit = FAILURE_MODE_MASK_I_LO_J_HI;
                                }
                            }
                            else
                            {
                                if (input_string[j] == '0')
                                {
                                    failure_mode_bit = FAILURE_MODE_MASK_I_HI_J_LO;
                                }
                                else
                                {
                                    failure_mode_bit = FAILURE_MODE_MASK_I_HI_J_HI;
                                }
                            }
                            test.columns[i].failure_mode[j] |= failure_mode_bit;
                        }
                    }
                }
            }
        }

        for (i = 0; i < 5; i++) data_in_old[i] = data_in[i];
        /* done with all test columns   */

        if(error_flag)  // set the trigger testpoint to indicate which vectors have an error
            set_error_strobe_trigger();

        if (error_flag && stop_on_fail_flag)
        {
            if (print_flag == 0) print("\r\n");
            print_flag = 1;
            test_mode = TEST_MODE_SINGLE_STEP;
        }
        if (print_flag)
        {
            /* show headings if error or single stepping    */
            if (error_flag || (test_mode == TEST_MODE_SINGLE_STEP))
            {

                print_pin_and_direction_heading();
            }
            need_comment = 0;
            need_change = 0;

            if (comment_flag)
            {
                need_comment = 1;
                need_change = 1;
            }

            if (error_flag)                       need_change = 1;
            if (test_mode == TEST_MODE_SINGLE_STEP) need_change = 1;

            if (need_comment)
            {
                if (stepval == 0)
                {
                    i = test.raw_line_direction + 1;
                }
                else
                {
                    i = test.step[stepval - 1].raw_line_test + 1;
                }
                while (i <= test.step[stepval].raw_line_test)
                {
                    sprintf(print_buffer, "source:   %s\r\n", raw_test.line[i]);
                    print(print_buffer);
                    i++;
                }
            }

            if (need_change)
            {
                sprintf(print_buffer, "changed:  %s\r\n", change_string);
                print(print_buffer);
            }

            /* print the test step results  */
            sprintf(print_buffer, "step %4u %s\r\n", stepval + 1, input_string);
            print(print_buffer);
            if (error_flag)     /* errors?  */
            {
                sprintf(print_buffer, "fail      %s\r\n", error_string);
                print(print_buffer);
            }
            else if (test_mode == TEST_MODE_SINGLE_STEP)
            {
                print("okay\r\n");
            }
        }
        if (stepval >= (test.number_steps - 1))    /* last test?   */
        {
            if (test_mode == TEST_MODE_RUN_ONCE) test_mode = TEST_MODE_SINGLE_STEP;
            tests_run++;
            if (step_fail_count)
            {
                tests_fail++;
            }
            else
            {
                tests_pass++;
            }

            if (print_flag == 0)
            {
                if (step_fail_count)
                {
                    print("F");
                }
                else
                {
                    print("p");
                }
                scope_cols_left--;
                if (scope_cols_left == 0)
                {
                    scope_cols_left = 79;
                    print("\r\n");
                }
            }
            else if (test_mode != TEST_MODE_TOGGLE)
            {
                print("\r\n");
                sprintf(print_buffer, "test %lu: ", tests_run);
                print(print_buffer);
                if (step_fail_count)
                {
                    sprintf(print_buffer, "*** FAIL *************************** %u steps failed", step_fail_count);
                    print(print_buffer);
                }
                else
                {
                    print("pass ");
                }
                print("\r\n");
                print("\r\n");
                print_pin_and_direction_heading();
                sprintf(print_buffer, "this fail %s\r\n", test_fail_string);
                print(print_buffer);
                sprintf(print_buffer, "all fails %s\r\n", all_fail_string);
                print(print_buffer);
                sprintf(print_buffer, "was hi    %s\r\n", hi_string);
                print(print_buffer);
                sprintf(print_buffer, "rising    %s\r\n", rising_string);
                print(print_buffer);
                sprintf(print_buffer, "falling   %s\r\n", falling_string);
                print(print_buffer);
                sprintf(print_buffer, "was lo    %s\r\n", lo_string);
                print(print_buffer);
                print("\r\n");
                sprintf(print_buffer, " total fails %lu, total passes %lu\r\n", tests_fail, tests_pass);
                print(print_buffer);
                print("\r\n");
                print("\r\n");
                print("\r\n");
            }
        }

        if (test_mode != TEST_MODE_SINGLE_STEP)
        {
            if (print_flag)
            {
                /* extra slow when printing */
                for (delay_count = 0; delay_count < (testdelay * 20000UL); delay_count++)
                {
                    if (Serial.available()>0/*_kbhit()*/) break;
                }
            }
        }

        key = 0;
        if (Serial.available()>0/*_kbhit()*/) key = get_a_key_convert_to_upper();
        if ((key == '+') || (key == '=') || (key == KEY_CURSOR_UP))
        {
            if (testdelay > 0) testdelay--;
            key = 0;        /* key is used up   */
        }
        if ((key == '-') || (key == KEY_CURSOR_DOWN))
        {
            if (testdelay < 100) testdelay++;
            key = 0;        /* key is used up   */
        }

        if ((key != 0) || (test_mode == TEST_MODE_SINGLE_STEP))
        {
            key = 0;        /* key is used up   */
            do
            {
                test_mode = TEST_MODE_UNKNOWN;
                print("\r\n");
                print("\r\n");
                print(" space  single step\r\n");
                print(" B      go back one step\r\n");
                print(" T      toggle (previous and current step)\r\n");
                print(" O      run one test (all steps)\r\n");
                print(" G      go\r\n");
                print(" F      go, stop on failure\r\n");
                print(" N      run, stop on fail, no print\r\n");
                print(" S      scope (run, no print)\r\n");
                print(" P      show pins\r\n");
                print(" M      mapping\r\n");
                print(" R      show registers\r\n");
                print(" D      diagnostics\r\n");
                print(" C      turn ");
                print((comment_flag) ? "off" : "on");
                print(" comment printout\r\n");
                print(" A      failure mode analysis\r\n");
                print(" +      increase speed (less delay)\r\n");
                print(" -      decrease speed (more delay)(slower)\r\n");
                print(" Q      quit\r\n");
                key = get_a_key_convert_to_upper();
                if (key == '+')
                {
                    if (testdelay > 0) testdelay--;
                    sprintf(print_buffer, "the delay is now %u\r\n", testdelay);
                    print(print_buffer);
                }
                if (key == '-')
                {
                    if (testdelay < 100) testdelay++;
                    sprintf(print_buffer, "the delay is now %u\r\n", testdelay);
                    print(print_buffer);
                }


                if (key == ' ') test_mode = TEST_MODE_SINGLE_STEP;
                if (key == 'O') test_mode = TEST_MODE_RUN_ONCE;
                if (key == 'B') test_mode = TEST_MODE_BACKUP;
                if (key == 'T') test_mode = TEST_MODE_TOGGLE;
                if (key == 'F') test_mode = TEST_MODE_STOP_ON_FAIL;
                if (key == 'G') test_mode = TEST_MODE_GO;
                if (key == 'N') test_mode = TEST_MODE_STOP_ON_FAIL_NO_PRINT;
                if (key == 'S') test_mode = TEST_MODE_SCOPE;
                if (key == 'Q') test_mode = TEST_MODE_QUIT;
                if (key == 'C')
                {
                    comment_flag = 1 - comment_flag;
                    sprintf(print_buffer, "comment_flag is %u\r\n", comment_flag);
                    print(print_buffer);
                }

                if (key == 'P')
                {
                    for (i = 0; i < test.number_columns; i++)
                    {
                        if ((error_string[i] != ' ') ||
                            (change_string[i] != ' ') ||
                            (all_pins_flag))
                        {
                            sprintf(print_buffer, "%s\r\n", test.columns[i].string);
                            print(print_buffer);
                        }
                    }
                    all_pins_flag = 1 - all_pins_flag;
                }

                if (key == 'M')
                {
                    for (i = 0; i < test.number_columns; i++)
                    {
                        sprintf(print_buffer, "pin %2u: offset is %u; mask is 0x%04X\r\n",
                            i + 1, test.columns[i].offset, test.columns[i].mask);
                        print(print_buffer);
                    }
                }

                if (key == 'R')
                {
                    int             index;
                    int             i;
                    unsigned char   reg;
                    unsigned int    reg_in[5];
                    unsigned char   byte0;
                    unsigned char   byte1;

                    for (index = 0; index < 8; index++)
                    {
                        reg = REG_IODIR;
                        if (index == 1) reg = REG_GPPU;
                        if (index == 2) reg = REG_OLAT;
                        if (index == 3) reg = REG_GPIO;
                        if (index == 4) reg = REG_IODIR;
                        if (index == 5) reg = REG_GPPU;
                        if (index == 6) reg = REG_OLAT;
                        if (index == 7) reg = REG_GPIO;
                        print("\r\n");
                        print_reg_name(reg);
                        sprintf(print_buffer, " %02X/%02X", reg + 1, reg);
                        print(print_buffer);
                        reg_read(reg, reg_in);
                        for (i = 0; i < 5; i++)
                        {
                            byte0 = (unsigned char)(reg_in[i] >> 0);
                            byte1 = (unsigned char)(reg_in[i] >> 8);
                            sprintf(print_buffer, "  %04Xh", reg_in[i]);
                            print(print_buffer);
                            if (i == 0)
                            {
                                print(" "); print_bin8(byte1);
                                print(" "); print_bin8(byte0);
                            }
                        }
                    }
                    print("\r\n");
                }

                if (key == 'D')
                {
                    print("\r\n");
                    print("\r\n");
                    print("up to this point:\r\n");
                    print("\r\n");
                    print("PINS that are always low\r\n");
                    print_flag = 0;
                    for (i = 0; i < test.number_columns; i++)
                    {
                        if ((lo_string[i] != ' ') &&
                            (falling_string[i] == ' ') &&
                            (rising_string[i] == ' ') &&
                            (hi_string[i] == ' '))
                        {
                            print_flag = 1;
                            sprintf(print_buffer, "%s\r\n", test.columns[i].string);
                            print(print_buffer);
                        }
                    }
                    if (print_flag == 0) print("<none>\r\n");

                    print("\r\n");
                    print("PINS that are always high\r\n");
                    print_flag = 0;
                    for (i = 0; i < test.number_columns; i++)
                    {
                        if ((lo_string[i] == ' ') &&
                            (falling_string[i] == ' ') &&
                            (rising_string[i] == ' ') &&
                            (hi_string[i] != ' '))
                        {
                            print_flag = 1;
                            sprintf(print_buffer, "%s\r\n", test.columns[i].string);
                            print(print_buffer);
                        }
                    }
                    if (print_flag == 0) print("<none>\r\n");
                    print("\r\n");
                    print("\r\n");
                }

                if (key == 'A')
                {
                    print("\r\n");
                    print("\r\n");
                    print("failure mode analysis:\r\n");

                    for (i = 0; i < test.number_columns; i++)
                    {
                        if (all_fail_string[i] != ' ')
                        {
                            print("\r\n");
                            sprintf(print_buffer, "pin:      %s\r\n", test.columns[i].string);
                            print(print_buffer);
                            print_pin_and_direction_heading();

                            strcpy(error_string, blank_string);
                            for (j = 0; j < test.number_columns; j++)
                            {
                                if (test.columns[i].failure_mode[j] & FAILURE_MODE_MASK_I_LO_J_HI)
                                {
                                    error_string[j] = '1';
                                }
                            }
                            sprintf(print_buffer, "fails LO: %s\r\n", error_string);
                            print(print_buffer);

                            strcpy(error_string, blank_string);
                            for (j = 0; j < test.number_columns; j++)
                            {
                                if (test.columns[i].failure_mode[j] & FAILURE_MODE_MASK_I_LO_J_LO)
                                {
                                    error_string[j] = '0';
                                }
                            }
                            sprintf(print_buffer, "fails LO: %s\r\n", error_string);
                            print(print_buffer);

                            strcpy(error_string, blank_string);
                            for (j = 0; j < test.number_columns; j++)
                            {
                                if (test.columns[i].failure_mode[j] & FAILURE_MODE_MASK_I_HI_J_HI)
                                {
                                    error_string[j] = '1';
                                }
                            }
                            sprintf(print_buffer, "fails HI: %s\r\n", error_string);
                            print(print_buffer);
                            strcpy(error_string, blank_string);
                            for (j = 0; j < test.number_columns; j++)
                            {
                                if (test.columns[i].failure_mode[j] & FAILURE_MODE_MASK_I_HI_J_LO)
                                {
                                    error_string[j] = '0';
                                }
                            }
                            sprintf(print_buffer, "fails HI: %s\r\n", error_string);
                            print(print_buffer);
                        }
                    }
                    print("\r\n");
                    print("\r\n");
                }

                print_flag = 1;   /* assume printing is on    */
                stop_on_fail_flag = 0;   /* assume not stop on fail  */
                toggle_value = 0;   /* assume no toggle         */
                if (test_mode == TEST_MODE_STOP_ON_FAIL)
                {
                    stop_on_fail_flag = 1;
                }
                if (test_mode == TEST_MODE_STOP_ON_FAIL_NO_PRINT)
                {
                    print_flag = 0;
                    stop_on_fail_flag = 1;
                    print("run, stop on fail, no print\r\n");
                }
                if (test_mode == TEST_MODE_SCOPE)
                {
                    print_flag = 0;
                    print("scope (run, no print)\r\n");
                }
                if (test_mode == TEST_MODE_TOGGLE)
                {
                    toggle_value = -1;
                }
            } while (test_mode == TEST_MODE_UNKNOWN);
        }

        /* find next step       */
        switch (test_mode)
        {
        default:
        case TEST_MODE_UNKNOWN:
        case TEST_MODE_QUIT:
            break;
        case TEST_MODE_SINGLE_STEP:
        case TEST_MODE_RUN_ONCE:
        case TEST_MODE_GO:
        case TEST_MODE_STOP_ON_FAIL:
        case TEST_MODE_STOP_ON_FAIL_NO_PRINT:
        case TEST_MODE_SCOPE:
            stepval++;                 /* will fixup later */
            break;

        case TEST_MODE_BACKUP:
            stepval--;                 /* will fixup later */
            test_mode = TEST_MODE_SINGLE_STEP;
            break;

        case TEST_MODE_TOGGLE:
            stepval += toggle_value;   /* will fixup later */
            toggle_value = -toggle_value;
            break;
        }
        /* fixup wrap around    */
        if (stepval == test.number_steps) stepval = 0;
        if (stepval > test.number_steps) stepval = test.number_steps - 1;
    }

    print("\r\n");
    print("\r\n");

    print_pin_and_direction_heading();
    sprintf(print_buffer, "all fails %s\r\n", all_fail_string);
    print(print_buffer);
    sprintf(print_buffer, "was lo    %s\r\n", lo_string);
    print(print_buffer);
    sprintf(print_buffer, "falling   %s\r\n", falling_string);
    print(print_buffer);
    sprintf(print_buffer, "rising    %s\r\n", rising_string);
    print(print_buffer);
    sprintf(print_buffer, "was hi    %s\r\n", hi_string);
    print(print_buffer);
    print("\r\n");
    sprintf(print_buffer, " total fails %lu, total passes %lu\r\n", tests_fail, tests_pass);
    print(print_buffer);
    print("\r\n");
    print("\r\n");
}



void output_load_test(void)
{
    unsigned int    i;
    unsigned int    stepval;
    int             key;
    int             first_flag;
    int             no_output_flag;
    unsigned int    pin_driver;
    unsigned int    column;
    unsigned int    data_out[5];
    unsigned int    data_in[5];
    unsigned int    data_in_old[5];
    unsigned int    offset;
    unsigned int    mask;
    unsigned int    value_now;
    unsigned int    value_old;


    strcpy(blank_string, "");
    for (i = 0; i < test.number_columns; i++) strcat(blank_string, " ");

    first_flag = 1;
    pin_driver = PIN_DRIVERS;
    stepval = 0;
    column = 0;
    do
    {
        /* find the next pin driver that is a test output       */

        for (;;)            /* will break on next output column */
        {
            no_output_flag = 0;
            pin_driver++;
            if (pin_driver >= PIN_DRIVERS)
            {
                no_output_flag++;
                if (no_output_flag > 1)
                {
                    print("ERROR: no outputs found\r\n");
                    break;
                }
                pin_driver = 0;
            }
            /* see if pin driver is a test column   */
            for (i = 0; i < test.number_columns; i++)
            {
                if (test.columns[i].mapping_index == pin_driver) break;
            }
            if (i >= test.number_columns) continue;         /* not a test column?   */

            /* found a pin driver that is used in the test  */
            if (test.columns[i].pdirection == 'I') continue;  /* not an output?       */

            /* have the next output column  */
            no_output_flag = 0;
            column = i;
            break;
        }   /* for(;;)  next output column  */
        if (no_output_flag) break;      /* no outputs   */

        for (;;)        /* will break on 'N' or 'Q' */
        {
            print("output is:\r\n");
            sprintf(print_buffer, "%s\r\n", test.columns[column].string);
            print(print_buffer);
            print("\r\n");
            print(" space  toggle output\r\n");
            print(" N      next output\r\n");
            print(" Q      quit\r\n");
            key = get_a_key_convert_to_upper();
            if (key == ' ')
            {
                for (;;)            /* will break on changing output    */
                {
                    /* do a test step   */
                    if (stepval == 0)
                    {
                        reg_write(REG_IODIR, test.pdirection);
                        reg_write(REG_GPPU, test.pullup);
                    }

                    /* do one step of a test    */
                    for (i = 0; i < 5; i++) data_out[i] = test.step[stepval].out_data[i];
                    reg_write(REG_OLAT, data_out);
                    reg_read(REG_GPIO, data_in);
                    if (first_flag)
                    {
                        for (i = 0; i < 5; i++) data_in_old[i] = data_in[i];
                        first_flag = 0;
                    }

                    /* process the test columns     */
                    strcpy(input_string, blank_string);
                    strcpy(change_string, blank_string);
                    for (i = 0; i < test.number_columns; i++)
                    {
                        offset = test.columns[i].offset;
                        mask = test.columns[i].mask;
                        value_now = data_in[offset] & mask;
                        value_old = data_in_old[offset] & mask;
                        if (value_now)
                        {
                            input_string[i] = '1';
                            if (value_old == 0) change_string[i] = '1';
                        }
                        else
                        {
                            input_string[i] = '0';
                            if (value_old != 0) change_string[i] = '0';
                        }
                    }
                    for (i = 0; i < 5; i++) data_in_old[i] = data_in[i];
                    /* done with all test columns   */

                    /* print the test step results  */
                    sprintf(print_buffer, "step %4u %s\r\n", stepval + 1, input_string);
                    print(print_buffer);
                    /* find next step       */
                    stepval++;
                    if (stepval >= test.number_steps) stepval = 0;

                    if (change_string[column] != ' ')
                    {
                        for (i = 0; i < test.number_columns; i++)
                        {
                            if (i != column) change_string[i] = ' ';
                        }
                        sprintf(print_buffer, "output:   %s\r\n", change_string);
                        print(print_buffer);
                        break;
                    }
                    if (Serial.available()>0/*_kbhit()*/) break;
                }   /* for (;;) break on changing output    */
            }   /* if (key == ' ')  */
            if (key == 'N') break;
            if (key == 'Q') break;
        }
    } while (key != 'Q');
    print("\r\n");
    print("\r\n");
}

// Arduino append to file function for SD file system
//
void appendFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file){
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)){
    Serial.println("Message appended");
  }
  else {
    Serial.println("Append failed");
  }
  file.close();
}

// Arduino list directory function for SD file system
//
void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if(!root){
    Serial.println("Failed to open directory");
    return;
  }
  if(!root.isDirectory()){
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while(file){
    if(file.isDirectory()){
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if(levels){
        listDir(fs, file.name(), levels -1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

// MCPinitializeIOCON initializes BANK and HAEN so registers are not banked and hardware addr is used
// MCP23S17 8-bit write to IOCON, BANK=0, SEQOP=0, HAEN=1, 0x08, all 5 chips at once
//   all 5 chips are written simultaneously with the following because HAEN=0 after power reset
//
void MCPinitializeIOCON(){
  int tchipaddress;
  for(tchipaddress=1; tchipaddress<=5; tchipaddress++){
    mcpwrite(tchipaddress, REG_IOCON, 0x0000); //IOCON data value: BANK=0, HAEN=0, 0x08; was BANK=0, HAEN=1, 0x08
  }
  //digitalWrite(MCPchipSelectIC1, LOW);
  //digitalWrite(MCPchipSelectIC2, LOW);
  //digitalWrite(MCPchipSelectIC3, LOW);
  //digitalWrite(MCPchipSelectIC4, LOW);
  //digitalWrite(MCPchipSelectIC5, LOW);
  //SPI.transfer(0x40); //Device Opcode A2 - A0 are don't-care until HAEN is 1
  //SPI.transfer(0x14); //Register Address of IOCON=0x0a with BANK=0
  //SPI.transfer(0x00); //IOCON data value: BANK=0, HAEN=0, 0x08; was BANK=0, HAEN=1, 0x08
  //SPI.transfer(0x00); //IOCON data value: BANK=0, HAEN=0, 0x08; was BANK=0, HAEN=1, 0x08
  //digitalWrite(MCPchipSelectIC1, HIGH);
  //digitalWrite(MCPchipSelectIC2, HIGH);
  //digitalWrite(MCPchipSelectIC3, HIGH);
  //digitalWrite(MCPchipSelectIC4, HIGH);
  //digitalWrite(MCPchipSelectIC5, HIGH);
}

void setup() {
  pinMode(onboard_ledPin, OUTPUT); // initialize the onboard LED pin as an output
  digitalWrite(onboard_ledPin, LOW); // turn off the onboard LED, also used as the error trigger for the oscilloscope

  //initialize chip select pins
  pinMode(MCPchipSelectIC1, OUTPUT); // initialize MCP SPI Chip Select pin for IC1
  digitalWrite(MCPchipSelectIC1, HIGH); // initialize MCP Chip Selectpin for IC1 to the inactive (HIGH) state
  pinMode(MCPchipSelectIC2, OUTPUT); // initialize MCP SPI Chip Select pin for IC2
  digitalWrite(MCPchipSelectIC2, HIGH); // initialize MCP Chip Selectpin for IC2 to the inactive (HIGH) state
  pinMode(MCPchipSelectIC3, OUTPUT); // initialize MCP SPI Chip Select pin for IC3
  digitalWrite(MCPchipSelectIC3, HIGH); // initialize MCP Chip Selectpin for IC3 to the inactive (HIGH) state
  pinMode(MCPchipSelectIC4, OUTPUT); // initialize MCP SPI Chip Select pin for IC4
  digitalWrite(MCPchipSelectIC4, HIGH); // initialize MCP Chip Selectpin for IC4 to the inactive (HIGH) state
  pinMode(MCPchipSelectIC5, OUTPUT); // initialize MCP SPI Chip Select pin for IC5
  digitalWrite(MCPchipSelectIC5, HIGH); // initialize MCP Chip Selectpin for IC5 to the inactive (HIGH) state
  pinMode(MCPnresetPin, OUTPUT); // initialize MCP not-reset pin for IC1-IC5
  digitalWrite(MCPnresetPin, HIGH); // initialize MCP not-reset to the inactive (HIGH) state
  pinMode(SDchipSelectPin, OUTPUT); // initialize SD Chip Select pin
  digitalWrite(SDchipSelectPin, HIGH); // initialize SD Chip Select to the inactive (HIGH) state

  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));
  SPI.begin();
  reg_init(); // briefly assert the MCP reset pin, also initializes BANK and HAEN afterward
  delay(2);

  //Serial.begin(115200);
  Serial.begin(921600);
  if(!SD.begin(SDchipSelectPin)){
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
    Serial.println("MMC");
  } else if(cardType == CARD_SD){
    Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  listDir(SD, "/", 0);

  print("\r\n");
  print("BOOT - PDP-8 card tester, Arduino version\r\n");
  print(VERSION_STRING);
  print("\r\n");

  //if (argc != 1)
  //{
  //    exit(1);
  //}

  verify_mapping();
  strcpy(test.filename, TEST_FILENAME_NONE);
  test.number_columns = 0;
  test.number_steps = 0;
  raw_test.number_lines = 0;
  reg_init();     /* initialize GPIO chip registers   */
  //exit_flag = 0;
  //for (;;)            /* will loop unit 'exit'    */

  //Serial.setTimeout(SERIAL_TIMEOUT);
}

void loop() {
  int             key;
  //int           exit_flag;
  unsigned int    uTemp;
  static unsigned int    test_delay = 0UL;   // must be static to hold the value
  static unsigned int    test_trigger = 1UL; // must be static to hold the value

  int       i;
  static char      lbuffer[MAX_STRING];
  static char      buffer2[MAX_STRING];
  char* ptr;
  //File file_test;
  char incomingbyte;
  String theinputstring;

  //Serial.println("  *1 test_delay = " + String(test_delay));
  print("\r\n");
  print("\r\n");
  print("Main menu   ");  print_timestamp();  print("\r\n");
  sprintf(print_buffer, "   test file is: %s\r\n", test.filename);
  print(print_buffer);
  sprintf(print_buffer, "   delay is:   %u\r\n", test_delay);
  print(print_buffer);
  //Serial.println("  *2 test_delay = " + String(test_delay));
  sprintf(print_buffer, "   trigger is: %u\r\n", test_trigger);
  print(print_buffer);
  print(" 1 read test file\r\n");
  print(" 2 set test delay\r\n");
  print(" 3 set test trigger\r\n");
  print(" 4 run test\r\n");
  print(" 5 output loading test\r\n");
  print(" 8 diags\r\n");
  print(" 9 exit\r\n");

  key = get_a_key();
  switch (key)
  {
  case '1':   /* read test file    */
    for (;;)      /* will break on success  */
    {
      print("Enter test file name? ");
      //(void)fgets(lbuffer, sizeof(lbuffer), stdin);
      theinputstring = String("");
      while(true){
        if(Serial.available()>0){
          incomingbyte = Serial.read();
          Serial.print(incomingbyte);
          if((incomingbyte == '\n') || (incomingbyte == '\r')) break;
          theinputstring = theinputstring + incomingbyte;
        }
      }
      Serial.println("\nentered: " + String(theinputstring));
      theinputstring.toCharArray(lbuffer, sizeof(lbuffer));  // copy the input string to "lbuffer" in legacy code
      
      //ptr = strchr(lbuffer, '\n');
      //if (ptr != (char*)NULL) *ptr = '\0';
      //ptr = strchr(lbuffer, '\r');
      //if (ptr != (char*)NULL) *ptr = '\0';

      //strcpy(buffer2, "tests\\");     /* directory of the tests */
      strcpy(buffer2, "/");              // tests are at the root level on the microSD card
      // for some reason the microSD filesystem wants the leading slash, temporary fix for now -gw
      if ((strlen(buffer2) + strlen(lbuffer) + 2) < sizeof(buffer2)) break;    /* break on success */

      print("test file name is too long. Try again.\r\n");
    }
    strcat(buffer2, lbuffer);    /* lengths have been checked  */
    str_to_upr(buffer2); // convert to uppercase, some issue with microSD filesystem being case sensitive -gw

    Serial.println("main menu, corrected filename: " + String(buffer2));
    sprintf(print_buffer, "trying to open test file: %s\r\n", buffer2);
    print(print_buffer);
    file_test = SD.open(buffer2 /*, "rt"*/);
    if (!file_test /*== (File*)NULL*/)
    {
      print("could not open test file.\r\n");
      print("valid test files are:\r\n");
      //system("dir /a-d tests\\*.*");
      //system("dir /a-d /w tests\\*.*");
      listDir(SD, "/", 0);
      sprintf(print_buffer, "reverting back to test file: %s\r\n", test.filename);
      print(print_buffer);
    }
    else
    {
      strcpy(test.filename, buffer2);
      sprintf(print_buffer, "reading test file: %s\r\n", test.filename);
      print(print_buffer);
      if (read_test(/*file_test*/)) // file_test is now global to simplify conversion to Arduino SD file system
      {
        print("bad test file\r\n");
      }
    }
    break;

    case '2':   /* set test delay   */
      print("Enter test delay (0 to 100)? ");
      //(void)fgets(lbuffer, sizeof(lbuffer), stdin);
      theinputstring = String("");
      while(true){
        if(Serial.available()>0){
          incomingbyte = Serial.read();
          Serial.print(incomingbyte);
          if((incomingbyte == '\n') || (incomingbyte == '\r')) break;
          theinputstring = theinputstring + incomingbyte;
        }
      }
      Serial.println("\nentered: " + theinputstring);
      theinputstring.toCharArray(lbuffer, sizeof(lbuffer));  // copy the input string to "lbuffer" in legacy code

      //ptr = strchr(lbuffer, '\n');
      //if (ptr != (char*)NULL) *ptr = '\0';
      //ptr = strchr(lbuffer, '\r');
      //if (ptr != (char*)NULL) *ptr = '\0';

      i = sscanf(lbuffer, "%u", &uTemp);
      if (i != 1)
      {
        sprintf(print_buffer, "could not 'sscanf(...\"%s\"..)\r\n", lbuffer); // gw - changed to %s and added lbuffer because not enough arguments passed
        print(print_buffer);
      }
      else if (uTemp > 100)
      {
        print("test_delay must be 0 to 100)\r\n");
      }
      else
      {

        sprintf(print_buffer, "setting test delay to: %u\r\n", uTemp);
        print(print_buffer);
        test_delay = uTemp;
        //Serial.println("  *test_delay = " + String(test_delay));
      }
      break;

    case '3':   /* set test trigger   */
      if (test.number_steps == 0)
      {
        print("there are no test steps\r\n");
      }
      else
      {
        sprintf(print_buffer, "Enter test trigger (1 to %u)? ", test.number_steps);
        print(print_buffer);
        //(void)fgets(lbuffer, sizeof(lbuffer), stdin);
        theinputstring = String("");
        while(true){
          if(Serial.available()>0){
            incomingbyte = Serial.read();
            Serial.print(incomingbyte);
            if((incomingbyte == '\n') || (incomingbyte == '\r')) break;
            theinputstring = theinputstring + incomingbyte;
          }
        }
        Serial.println("\nentered: " + theinputstring);
        theinputstring.toCharArray(lbuffer, sizeof(lbuffer));  // copy the input string to "lbuffer" in legacy code

        //ptr = strchr(lbuffer, '\n');
        //if (ptr != (char*)NULL) *ptr = '\0';
        //ptr = strchr(lbuffer, '\r');
        //if (ptr != (char*)NULL) *ptr = '\0';

        i = sscanf(lbuffer, "%u", &uTemp);
        if (i != 1)
        {
          sprintf(print_buffer, "could not 'sscanf(...\"%s\"..)\r\n", lbuffer); // gw - changed to %s and added lbuffer because not enough arguments passed
          print(print_buffer);
        }
        else if ((uTemp < 1) || (uTemp > test.number_steps))
        {
          sprintf(print_buffer, "test_trigger must be 1 to %u)\r\n", test.number_steps);
          print(print_buffer);
        }
        else
        {
          sprintf(print_buffer, "setting test delay to: %u\r\n", uTemp);
          print(print_buffer);
          test_trigger = uTemp;
        }
      }
      break;

    case '4':   /* run test             */
    case '5':   /* output loading test  */
      if (strcmp(test.filename, TEST_FILENAME_NONE) == 0)
      {
        print("there is no test file\r\n");
      }
      else if (test.number_columns == 0)
      {
        print("there are no test columns\r\n");
      }
      else if (test.number_steps == 0)
      {
        print("there are no test steps\r\n");
      }
      else if (tester_init_and_uut_power_verify())
      {
        /* error already printed    */
      }
      else
      {
        if (key == '4')
        {
          run_a_test(test_delay, test_trigger);
        }
        else if (key == '5')
        {
          /* check for outputs    */
          for (uTemp = 0; uTemp < test.number_columns; uTemp++)
          {
            if (test.columns[uTemp].pdirection == 'O') break;
          }
          if (uTemp >= test.number_columns)
          {
            print("there are no outputs\r\n");
          }
          else
          {
            output_load_test();
          }
        }
      }
      break;

    case '8':   /* diags    */
      do_diags();
      break;

    case '9':
    case 'Q':
    case 'q':
    case KEY_ESCAPE:
      print("\r\nExit command entered, no exit on Arduino\r\n"); //added for Arduino version -gw
      //exit_flag = 1;
      break;

    default:
      sprintf(print_buffer, "\r\nkey is (gw) 0x%04X\r\n", key);
      print(print_buffer);
      break;
    //}
    //if (exit_flag) break;       /* break on exit    */
  }
  //print("exiting\r\n");
  //exit(0);
}
