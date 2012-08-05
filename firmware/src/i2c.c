//***************************************************************************
//  File Name   : i2cmaster.c
//  Version  : 1.0
//  Description  : AVR I2C Bus Master
//  Author       : RWB
//  Target       : AVRJazz Mega328 Board
//  Compiler     : AVR-GCC 4.3.0; avr-libc 1.6.2 (WinAVR 20090313)
//  IDE          : Atmel AVR Studio 4.17
//  Programmer   : AVRJazz Mega328 STK500 v2.0 Bootloader
//               : AVR Visual Studio 4.17, STK500 programmer
//  Last Updated : 12 September 2009
//  Taken from   : http://www.ermicro.com/blog/?p=1239
//***************************************************************************
#include <avr/io.h>
#include <util/delay.h>
#include <compat/twi.h>

#define MAX_TRIES 50

#define MCP23008_ID    0x40  // MCP23008 Device Identifier
#define MCP23008_ADDR  0x0E  // MCP23008 Device Address
#define IODIR 0x00           // MCP23008 I/O Direction Register
#define GPIO  0x09           // MCP23008 General Purpose I/O Register
#define OLAT  0x0A           // MCP23008 Output Latch Register

#define I2C_START 0
#define I2C_DATA 1
#define I2C_DATA_ACK 2
#define I2C_STOP 3
#define ACK 1
#define NACK 0

#define DATASIZE 32

/* START I2C Routine */
unsigned char i2c_transmit(unsigned char type) {
  switch(type) {
     case I2C_START:    // Send Start Condition
       TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
       break;
     case I2C_DATA:     // Send Data with No-Acknowledge
       TWCR = (1 << TWINT) | (1 << TWEN);
       break;
     case I2C_DATA_ACK: // Send Data with Acknowledge
       TWCR = (1 << TWEA) | (1 << TWINT) | (1 << TWEN);
       break;
     case I2C_STOP:     // Send Stop Condition
   TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
   return 0;
  }

  // Wait for TWINT flag set on Register TWCR
  while (!(TWCR & (1 << TWINT)));

  // Return TWI Status Register, mask the prescaler bits (TWPS1,TWPS0)
  return (TWSR & 0xF8);
}

char i2c_start(unsigned int dev_id, unsigned int dev_addr, unsigned char rw_type)
{
  unsigned char n = 0;
  unsigned char twi_status;
  char r_val = -1;

i2c_retry:
  if (n++ >= MAX_TRIES) return r_val;

  // Transmit Start Condition
  twi_status=i2c_transmit(I2C_START);

  // Check the TWI Status
  if (twi_status == TW_MT_ARB_LOST) goto i2c_retry;
  if ((twi_status != TW_START) && (twi_status != TW_REP_START)) goto i2c_quit;

  // Send slave address (SLA_W)
  TWDR = (dev_id & 0xF0) | (dev_addr & 0x0E) | rw_type;

  // Transmit I2C Data
  twi_status=i2c_transmit(I2C_DATA);

  // Check the TWSR status
  if ((twi_status == TW_MT_SLA_NACK) || (twi_status == TW_MT_ARB_LOST)) goto i2c_retry;
  if (twi_status != TW_MT_SLA_ACK) goto i2c_quit;

  r_val=0;

i2c_quit:
  return r_val;
}

void i2c_stop(void)
{
  unsigned char twi_status;

  // Transmit I2C Data
  twi_status=i2c_transmit(I2C_STOP);
}

char i2c_write(char data)
{
  unsigned char twi_status;
  char r_val = -1;

  // Send the Data to I2C Bus
  TWDR = data;

  // Transmit I2C Data
  twi_status=i2c_transmit(I2C_DATA);

  // Check the TWSR status
  if (twi_status != TW_MT_DATA_ACK) goto i2c_quit;

  r_val=0;

i2c_quit:
  return r_val;
}

char i2c_read(char *data,char ack_type)
{
  unsigned char twi_status;
  char r_val = -1;               

  if (ack_type) {
    // Read I2C Data and Send Acknowledge
    twi_status=i2c_transmit(I2C_DATA_ACK);

    if (twi_status != TW_MR_DATA_ACK) goto i2c_quit;
  } else {
    // Read I2C Data and Send No Acknowledge
    twi_status=i2c_transmit(I2C_DATA);

    if (twi_status != TW_MR_DATA_NACK) goto i2c_quit;
  }

  // Get the Data
  *data=TWDR;
  r_val=0;

i2c_quit:
  return r_val;
}

/*
void Write_MCP23008(unsigned char reg_addr,unsigned char data)
{
   // Start the I2C Write Transmission
   i2c_start(MCP23008_ID,MCP23008_ADDR,TW_WRITE);

   // Sending the Register Address
   i2c_write(reg_addr);

   // Write data to MCP23008 Register
   i2c_write(data);

   // Stop I2C Transmission
   i2c_stop();
}

unsigned char Read_MCP23008(unsigned char reg_addr)
{
   char data;

   // Start the I2C Write Transmission
   i2c_start(MCP23008_ID,MCP23008_ADDR,TW_WRITE);

   // Read data from MCP23008 Register Address
   i2c_write(reg_addr);

   // Stop I2C Transmission
   i2c_stop();

   // Re-Start the I2C Read Transmission
   i2c_start(MCP23008_ID,MCP23008_ADDR,TW_READ);

   i2c_read(&data,NACK);

   // Stop I2C Transmission
   i2c_stop();

   return data;
}
*/

void i2c_init(void)
{
  // Initial ATMega328P TWI/I2C Peripheral
  TWSR = 0x00;         // Select Prescaler of 1

  // SCL frequency = 11059200 / (16 + 2 * 48 * 1) = 98.743 kHz
  TWBR = 0x30;        // 48 Decimal
}
/*
int main(void)
{
  unsigned char ptr,data;
  unsigned int iDelay;       

  char led_pattern[DATASIZE]=
                        {0b00000001,
          0b00000011,
          0b00000110,
          0b00001100,
          0b00011001,
          0b00110011,
          0b01100110,
          0b11001100,
          0b10011000,
          0b00110000,
          0b01100000,
          0b11000000,
          0b10000000,
          0b00000000,
          0b00000000,
          0b00000000,
          0b10000000,
          0b11000000,
          0b01100000,
          0b00110000,
          0b10011000,
          0b11001100,
          0b01100110,
          0b00110011,
          0b00011001,
          0b00001100,
          0b00000110,
          0b00000011,
          0b00000001,
          0b00000000,
          0b00000000,
          0b00000000
         };                       

  DDRD=0xFF;          // Set PORTD as Output
  PORTD=0x00;         // Set All PORTD to Low

   // Set ADCSRA Register on ATMega328
  ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1);

  // Set ADMUX Register on ATMega328
  ADMUX = 0x00;       // Use Left Justified, Select Channel 0

  // Initial Master I2C
  i2c_init();

  // Initial the MCP23008 GP0 to GP7 as Output
  Write_MCP23008(IODIR,0b00000000);
  Write_MCP23008(GPIO,0b00000000);    // Reset all the Output Port           

  // Loop Forever
  for (;;) {
    for(ptr=0;ptr < DATASIZE;ptr++) {
      // Start conversion by setting ADSC on ADCSRA Register
      ADCSRA |= (1<<ADSC);

      // wait until conversion complete ADSC=0 -> Complete
      while (ADCSRA & (1<<ADSC));

      // Get ADC the Result
      iDelay = ADCW;

      // Write to MCP23008 GPIO Register
      Write_MCP23008(GPIO,led_pattern[ptr]);

      // Read MCP23008 OLAT Register
      data=Read_MCP23008(OLAT);

      PORTD=data;                     // Write data to the ATMega328 Port-D

      _delay_ms(iDelay);             // Give some delay here
    }
  } 

  return 0;
}
*/

