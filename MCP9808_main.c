
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_i2c.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/i2c.h"
#include "utils/uartstdio.h"

#define SLAVE_ADDRESS 			0x18
#define AMBIENT_TEMPERATURE		0x05
#define DEVICE_ID_REGISTER  		0x06
#define MANUFACTURE_ID_REGISTER		0x07

uint16_t convertTemp(uint8_t, uint8_t);
void ConfigureUART(void);
void I2C_Init(void);
void I2C_Send(void);
uint16_t I2C_readMode(void);
uint16_t I2C_TempRead(void);
void Device_ID(uint8_t device_reg);
void Manufacture_ID(uint8_t manufacture_reg);
void Ambient_Temp(uint8_t ambient_temp_reg);


int main(void)
{
  	FPULazyStackingEnable();
  	uint16_t Device_id = 0;
  	uint16_t Manufacture_id = 0;
  
	// Set the clocking to run directly from the crystal
	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
	
	// Enable the GPIO Port that is used for the on-board LEDs
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); 
	// Enable the GPIO Pins for the LEDs (PF1, PF2, PF3);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	
	// Initialize the UART
	ConfigureUART();
	SysCtlDelay(10000000);
	UARTprintf("Program Starting....\n\n");
	SysCtlDelay(50000000);
	UARTprintf("UART Initialized\n");
	SysCtlDelay(50000000);
	
	I2C_Init();
	SysCtlDelay(50000000);
	// I2C Send data to Slave Address - Device ID register - See MCP9808 datasheet
	I2C_Send();
	Device_ID(DEVICE_ID_REGISTER); 
	while(I2CMasterBusy(I2C0_BASE))
	{
	}
	Device_id = I2C_readMode();
	UARTprintf("Received Device ID from Slave: 0x%x\n\r", Device_id);
	SysCtlDelay(50000000);
	// I2C Send data to Slave Address - Manufacture ID register - See MCP9808 datasheet
	I2C_Send();
	Manufacture_ID(MANUFACTURE_ID_REGISTER);  
	while(I2CMasterBusy(I2C0_BASE))
	{
	}
	Manufacture_id = I2C_readMode();
	SysCtlDelay(50000000);
	
	
	UARTprintf("Received Manufacture ID from Slave: 0x%x\n\r", Manufacture_id);
	SysCtlDelay(50000000);
	while(1)
	{
	  // I2C Send data to Slave Address - Ambient Temperature register - See MCP9808 datasheet
	  I2C_Send();
	  Ambient_Temp(AMBIENT_TEMPERATURE);
	  while(I2CMasterBusy(I2C0_BASE))
	  {
	  }
	  uint16_t Temperature = I2C_TempRead();
	  
	  uint16_t Temperaturef = Temperature * (9.0 / 5.0) + 32;
	  
	  SysCtlDelay(SysCtlClockGet()/10/3);
	  UARTprintf("Received Temperature data from MCP9808: \n\r");
	  UARTprintf("MCP9808 I2C Sensor temperature readings: %u *C\n\r", Temperature);
	  UARTprintf("Converted *C to *F: %u *F\n\n\n\n\r", Temperaturef);
	  SysCtlDelay(50000000);
	  	  
	  //////////////////////////////////////////////////////////////////////
	  // if (Temperature == 0x0400)		// Device ID
	  // {	// Turn on Red LED
	  //	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
	  // }
	  // else
	  // {	// Turn on Green LED
	  //	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);  
	  // }
	  //////////////////////////////////////////////////////////////////////
	  
	}	
}

void ConfigureUART(void)
{
  // Enable the GPIO Peripheral used by the UART
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  
  // Configure the GPIO Pins for UART mode
  GPIOPinConfigure(GPIO_PA0_U0RX);
  GPIOPinConfigure(GPIO_PA1_U0TX);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  
  // Use the internal 16MHz oscillator as the UART Clock source
  UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
  
  // Initialize the UART for console I/O.
  UARTStdioConfig(0, 9600, 16000000);
}

void I2C_Init()
{
    // Enable I2C1 peripheral
  SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0); 
  SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
  
  // Enable GPIO Port B to be used for I2C0
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  
  GPIOPinConfigure(GPIO_PB2_I2C0SCL);
  GPIOPinConfigure(GPIO_PB3_I2C0SDA);
  
  // Configure the pin muxing for I2C1 functions on Port B2 and B3
  GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
  GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
 
  I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
  // Set write mode
  UARTprintf("I2C Master init communication with Slave Address\n");
  SysCtlDelay(10000);
  UARTprintf("I2C Init complete!\n");
  SysCtlDelay(50000000);
}

void I2C_Send()
{
  // Specify Slave device address to write to
  I2CMasterSlaveAddrSet(I2C0_BASE, SLAVE_ADDRESS, false);
  UARTprintf("Master transmit to Slave address\n\n");
}

void Device_ID(uint8_t device_reg)
{
  // Send Register address on Slave device
  I2CMasterDataPut(I2C0_BASE, device_reg); 
  UARTprintf("Writing to device id register on Slave address\n");
  
  // Initiate send of register address from Master to Slave
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
  UARTprintf("Device ID register sent to Slave address\n");
}

void Manufacture_ID(uint8_t manufacture_reg)
{
  // Send Register address on Slave device
  I2CMasterDataPut(I2C0_BASE, manufacture_reg); 
  UARTprintf("Writing to manufacture id register on Slave address\n");
  
  // Initiate send of register address from Master to Slave
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
  UARTprintf("Manufacture ID register sent to Slave address\n");
}

void Ambient_Temp(uint8_t ambient_temp_reg)
{
  // Send Register address on Slave device
  I2CMasterDataPut(I2C0_BASE, ambient_temp_reg); 
  UARTprintf("Writing to ambient temperature register on Slave address\n");
  
  // Initiate send of register address from Master to Slave
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
  UARTprintf("Ambient temperature register sent to Slave address\n");
}



uint16_t I2C_readMode()
{
  uint8_t UpperByte = 0;
  uint8_t LowerByte = 0;
  uint16_t data = 0;
  // Set read mode
  I2CMasterSlaveAddrSet(I2C0_BASE, SLAVE_ADDRESS, true);
  
  // Get first byte from slave and ack for more
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START); 
  while(I2CMasterBusy(I2C0_BASE));

  UpperByte = I2CMasterDataGet(I2C0_BASE); 
  
  // Get second byte from slave and nack for complete
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
  while(I2CMasterBusy(I2C0_BASE)); 
  LowerByte = I2CMasterDataGet(I2C0_BASE);
  
  // See MCP9808 Data Sheet for each of the register information requested for
  data = UpperByte<<8|LowerByte;
  return data;
}

uint16_t I2C_TempRead()
{
  uint8_t UpperByte = 0;
  uint8_t LowerByte = 0;
  uint16_t Temperature = 0;
  // Set read mode
  I2CMasterSlaveAddrSet(I2C0_BASE, SLAVE_ADDRESS, true);
  
  // Get first byte from slave and ack for more
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START); 
  while(I2CMasterBusy(I2C0_BASE));
  UpperByte = I2CMasterDataGet(I2C0_BASE); 
  
  // Get second byte from slave and nack for complete
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
  
  while(I2CMasterBusy(I2C0_BASE)); 
  LowerByte = I2CMasterDataGet(I2C0_BASE);
  
  UpperByte = UpperByte & 0x1F;
	if( (UpperByte & 0x10) == 0x10)
	{
	  UpperByte = UpperByte & 0x0F;					// Clear sign
	  Temperature = 256 - (UpperByte * 16 + LowerByte / 16);	// 
	}
	else
	{
	  Temperature = (UpperByte * 16 + LowerByte / 16);
	}
	return Temperature;
}


