/********************************************************************** 
  This is an example for the Adafruit VS1053 Codec Breakout

  Designed specifically to work with the Adafruit VS1053 Codec Breakout 
  ----> https://www.adafruit.com/products/1381

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution

  Code adapted by Tobias van Dyk: Arduino Uno  December 2018
                                  Teensy 3x 4x November 2020
 
 ***********************************************************************/
 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 // This is a fully functional audio effects preamp using the VLSI VS1053b Audio DSP IC. 
 //
 // It has a potentiometer to adjust the volume and the five effect parameters. 
 // It has nine fixed effects and one customizable effect, where each effect has five effects settings namely delay, 
 // decay-repeat, modulation speed and depth, and the mix-ratio of the processed and direct audio. It includes adjustments 
 // for bass and treble boost, bass and treble centre frequency, a selection of six input gain values, an option for saving 
 // or retrieving the current/saved parameters to/from the Teensy Eeprom-simulated, and a basic/normal/advanced/edit menu option 
 // that determines the number of functions that are cycled through. Adjustments are made using three pushbuttons, namely a function 
 // select button and two buttons to increase and decrease values for the selected function .
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// include SPI, MP3 and SD libraries
#include <SPI.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>
// #include <Fonts/FreeSans9pt7b.h>
// #include <EEPROM.h>
// #include <Bounce.h>

// Connect SCLK, MISO and MOSI to standard hardware SPI pins. 
#define SCLK 13       // SPI Clock shared with SD card
#define MISO 12       // Input data from vs1053 or SD card
#define MOSI 11       // Output data to vs1053 or SD card

// These are the pins used for the Adafruit vs1053B breakout module
#define XRST  9                // vs1053 reset (output)
#define XCS   10               // vs1053 chip select (output)
#define XDCS  8                // vs1053 Data select (output)
#define XDREQ 3                // vs1053 Data Ready an Interrupt pin (input)
#define SDCS  BUILTIN_SDCARD   // Use Teensy built-in card
// For Teensy 3.5, 3.6, 4.0, 4.1 better to use its built-in SDCard
//#define SDCS 4                // Use vs1053 SDCard Card chip select pin

/////////////////////////////////////////////////////////////////////////////////
// vs1053B.h
/////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>

#define vs1053_SCI_READ 0x03
#define vs1053_SCI_WRITE 0x02

#define vs1053_REG_MODE  0x00
#define vs1053_REG_STATUS 0x01
#define vs1053_REG_BASS 0x02
#define vs1053_REG_CLOCKF 0x03
#define vs1053_REG_DECODETIME 0x04
#define vs1053_REG_AUDATA 0x05
#define vs1053_REG_WRAM 0x06
#define vs1053_REG_WRAMADDR 0x07
#define vs1053_REG_HDAT0 0x08
#define vs1053_REG_HDAT1 0x09
#define vs1053_REG_VOLUME 0x0B

#define vs1053_GPIO_DDR 0xC017
#define vs1053_GPIO_IDATA 0xC018
#define vs1053_GPIO_ODATA 0xC019

#define vs1053_INT_ENABLE  0xC01A

#define vs1053_MODE_SM_DIFF 0x0001
#define vs1053_MODE_SM_LAYER12 0x0002
#define vs1053_MODE_SM_RESET 0x0004
#define vs1053_MODE_SM_CANCEL 0x0008
#define vs1053_MODE_SM_EARSPKLO 0x0010
#define vs1053_MODE_SM_TESTS 0x0020
#define vs1053_MODE_SM_STREAM 0x0040
#define vs1053_MODE_SM_SDINEW 0x0800
#define vs1053_MODE_SM_ADPCM 0x1000
#define vs1053_MODE_SM_LINE1 0x4000
#define vs1053_MODE_SM_CLKRANGE 0x8000

#define vs1053_SCI_AIADDR 0x0A
#define vs1053_SCI_AICTRL0 0x0C
#define vs1053_SCI_AICTRL1 0x0D
#define vs1053_SCI_AICTRL2 0x0E
#define vs1053_SCI_AICTRL3 0x0F

////////////////////////////////////////////////////////////////////////////////
// vs1053B.cpp
////////////////////////////////////////////////////////////////////////////////
#define vs1053_CONTROL_SPI_SETTING  SPISettings(250000,  MSBFIRST, SPI_MODE0) // Control: 2.5 MHz SPI speed  
#define vs1053_DATA_SPI_SETTING   SPISettings(8000000, MSBFIRST, SPI_MODE0)   // Data: 8 MHz SPI speed
// Strange stuff here with Teensy 4.1 compared to Teensy 3.6 which clocks as it should

void vs1053SetVolume(uint8_t left, uint8_t right);
void vs1053SoftReset();
void vs1053Reset();
uint8_t vs1053Begin();
uint16_t vs1053SciRead(uint8_t addr);
void vs1053SciWrite(uint8_t addr, uint16_t data);
uint8_t vs1053SpiRead();
void vs1053SpiWrite(uint8_t c);

//////////////////////////////////////////////////
void vs1053SetVolume(uint8_t left, uint8_t right) 
{ uint16_t v;
  v = left;
  v <<= 8;
  v |= right;

  cli();
  vs1053SciWrite(vs1053_REG_VOLUME, v);
  sei();
}

///////////////////////
void vs1053SoftReset() 
{ vs1053SciWrite(vs1053_REG_MODE, vs1053_MODE_SM_SDINEW | vs1053_MODE_SM_RESET);
  delay(100);
}

//////////////////////////
void vs1053Reset() 
{ if (XRST >= 0) 
     { digitalWrite(XRST, LOW);
       delay(100);
       digitalWrite(XRST, HIGH);
     }
  digitalWrite(XCS, HIGH);
  digitalWrite(XDCS, HIGH);
  delay(100);
  vs1053SoftReset();
  delay(100);

  vs1053SciWrite(vs1053_REG_CLOCKF, 0x6000);

  vs1053SetVolume(40, 40);
}

//////////////////////
uint8_t vs1053Begin() 
{ if (XRST >= 0) { pinMode(XRST, OUTPUT);  // if reset = -1 ignore
                             digitalWrite(XRST, LOW);
                           }

  pinMode(XCS, OUTPUT);
  digitalWrite(XCS, HIGH);
  pinMode(XDCS, OUTPUT);
  digitalWrite(XDCS, HIGH);
  pinMode(XDREQ, INPUT);

  // SPI.beginTransaction(SPISettings(clockspeed, MSBFIRST, SPI_MODE0))
  // Begin using the SPI bus. Normally this is called before asserting the chip select signal. 
  // The SPI is configured to use the clock, data order (MSBFIRST or LSBFIRST) and data mode (SPI_MODE0, SPI_MODE1, SPI_MODE2, 
  // or SPI_MODE3). The clock speed should be the maximum speed the SPI slave device can accept. 
  SPI.begin();
  //SPI.beginTransaction(SPISettings(250000,  MSBFIRST, SPI_MODE0));
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV128); 
  
  vs1053Reset();

  return (vs1053SciRead(vs1053_REG_STATUS) >> 4) & 0x0F;
}

/////////////////////////////////////
uint16_t vs1053SciRead(uint8_t addr) 
{ uint16_t data;
  SPI.beginTransaction(vs1053_CONTROL_SPI_SETTING);
  digitalWrite(XCS, LOW);  
  vs1053SpiWrite(vs1053_SCI_READ);
  vs1053SpiWrite(addr);
  delayMicroseconds(10);
  data = vs1053SpiRead();
  data <<= 8;
  data |= vs1053SpiRead();
  digitalWrite(XCS, HIGH);
  SPI.endTransaction();
  return data;
}

//////////////////////////////////////////////
void vs1053SciWrite(uint8_t addr, uint16_t data) 
{ 
  Serial.println("In SCI Write");
  SPI.beginTransaction(vs1053_CONTROL_SPI_SETTING);
  // Serial.println("line 197");
  digitalWrite(XCS, LOW);  
  // Serial.println("line 199");
  vs1053SpiWrite(vs1053_SCI_WRITE);
  // Serial.println("line 201");
  vs1053SpiWrite(addr);
  // Serial.println("line 203");
  vs1053SpiWrite(data >> 8);
  // Serial.println("line 205");
  vs1053SpiWrite(data & 0xFF);
  // Serial.println("line 207");
  digitalWrite(XCS, HIGH);
  SPI.endTransaction();
  Serial.println("Complete SCI Write");
}

////////////////////////
uint8_t vs1053SpiRead()
{ int8_t x;
  x = 0;
  // Transmit a byte from master to slave, and simultaneously receive a byte from slave to master. 
  // SPI always transmits and receives at the same time, but often the received byte is ignored. 
  // When only reception is needed, 0 or 255 is transmitted to cause the reception. 
  x = SPI.transfer(0x00);

  return x;
}

///////////////////////////////
void vs1053SpiWrite(uint8_t c)
{ SPI.transfer(c);
}

/////////////////////////////////////
// End .cpp
/////////////////////////////////////




  
