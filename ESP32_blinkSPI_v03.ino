// Board: "DOIT ESP32 DEVKIT V1"
// Board: "ESP32 Dev Module"
#include <SPI.h>

#define IC1 1
#define IC2 2
#define IC3 3
#define IC4 4
#define IC5 5
#define IODIR_AB 0
#define GPPUA_AB 0x0c
#define GPIO_AB  0x12
#define REG_IOCON   0x0A        /* I/O config           */

const int onboard_ledPin = 2;
const int SDchipSelectPin = 4;
const int MCPchipSelectIC1 = 21;
const int MCPchipSelectIC2 = 17;
const int MCPchipSelectIC3 = 16;
const int MCPchipSelectIC4 = 15;
const int MCPchipSelectIC5 = 22;
const int MCPnresetPin = 5;
int icount, portread;

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
      Serial.println("broken sofware, invalid MCP chip address");
  }
  SPI.transfer16(0x4000 | (chipaddr << 9) | regaddr); //Device Opcode, Chip Address, Write, Reg Addr
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
      Serial.println("broken sofware, invalid MCP chip address");
  }
  SPI.transfer16(0x4100 | (chipaddr << 9) | regaddr); //Device Opcode, Chip Address, Read, Reg Addr
  mcp_read_data = SPI.transfer16(0x0000); //sending dummy data to read the port
  digitalWrite(MCPchipSelectIC1, HIGH);
  digitalWrite(MCPchipSelectIC2, HIGH);
  digitalWrite(MCPchipSelectIC3, HIGH);
  digitalWrite(MCPchipSelectIC4, HIGH);
  digitalWrite(MCPchipSelectIC5, HIGH);
  return(mcp_read_data);
}

// MCPinitializeIOCON initializes BANK and HAEN so registers are not banked and hardware addr is used
// MCP23S17 8-bit write to IOCON, BANK=0, SEQOP=0, HAEN=0, 0x00, all 5 chips
//
void MCPinitializeIOCON(){
  int tchipaddress;
  for(tchipaddress=1; tchipaddress<=5; tchipaddress++){
    mcpwrite(tchipaddress, REG_IOCON, 0x0000); //IOCON data value: BANK=0, HAEN=0, 0x00; was BANK=0, HAEN=1, 0x08
  }
}

void setup() {
  pinMode(onboard_ledPin, OUTPUT); // initialize the onboard LED pin as an output

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

  Serial.begin(115200);
  icount = 0;
  Serial.println();
  Serial.println("Start blink test");

  delay(100);
  //initialize SPI (VSPI pins)
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));
  SPI.begin();
  
  //MCP23S17 8-bit write to IOCON, BANK=0, SEQOP=0, HAEN=1, 0x08, all 5 chips at once
  //  all 5 chips are written simultaneously with the following because HAEN=0 after power reset
  MCPinitializeIOCON();
  //delay(10);

  //MCP23S17 write to IC5, IODIRA & IODIRB, with IODIRB bits 0-3 as outputs, 0xf0
  mcpwrite(IC5, IODIR_AB, 0xfff0);

  //MCP23S17 write to IC5, GPPUA & GPPUB, with PROBE1 - PROBE4 having pullups, GPPUA[7:4]
  mcpwrite(IC5, GPPUA_AB, 0xf000);

  //MCP23S17 write to IC1, IODIRA & IODIRB, with all IODIRA & IODIRB bits as outputs, 0x0000
  mcpwrite(IC1, IODIR_AB, 0x0000);
}

void loop() {
  delay(800);
  digitalWrite(onboard_ledPin, HIGH);  // Turn the LED on by setting the voltage HIGH
  mcpwrite(IC5, GPIO_AB, 0x0008); //MCP23S17 write to IC5, GPIOA & GPIOB, 0x08

  portread = mcpread(IC5, GPIO_AB); //MCP23S17 read from IC5, GPIOA & GPIOB
  Serial.println(String("  Read 1st ") + String(portread,BIN));

  delay(500);
  digitalWrite(onboard_ledPin, LOW);  // Turn the LED off by setting the voltage LOW
  mcpwrite(IC5, GPIO_AB, 0x0002); //MCP23S17 write to IC5, GPIOA & GPIOB,0x00

  portread = mcpread(IC5, GPIO_AB); //MCP23S17 read from IC5, GPIOA & GPIOB
  Serial.println(String("  Read 2nd ") + String(portread,BIN));

  delay(500);
  mcpwrite(IC1, GPIO_AB, 0x55aa); //MCP23S17 write to IC1, GPIOA & GPIOB,0x55aa

  delay(500);
  mcpwrite(IC1, GPIO_AB, 0xaa55); //MCP23S17 write to IC1, GPIOA & GPIOB,0xaa55

  Serial.println(String(icount++) + " " + String(portread,BIN));
}
