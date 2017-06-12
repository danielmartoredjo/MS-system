//http://www.farnell.com/datasheets/1697937.pdf?_ga=2.241682626.1344182831.1497259807-872291831.1497259807

#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>

/**************************************************************************************
 * spiReadReg
 *
 * Reads an 8-bit register with the SPI port.
 * Data is returned.
 **************************************************************************************/

//unsigned char spiReadReg (const unsigned char regAddr){
void main(void){
	unsigned char SPICount;	// Counter used to clock out the data 
 	unsigned char SPIData;

	int SPI_CK = 11;
	int SPI_MOSI = 10;
	int SPI_MISO = 9;
	int SPI_CS = 13;

	pinMode(10, OUTPUT); //MOSI
	pinMode(9, INPUT);   //MISO
	pinMode(11, OUTPUT); //CLK
	pinMode(13, OUTPUT); //CS

 	digitalWrite(SPI_CS, HIGH);   	// Make sure we start with active-low CS high
  	digitalWrite(SPI_CK, LOW);	// and CK low
//  	SPIData = regAddr;		// Preload the data to be sent with Address and Data

  	digitalWrite(SPI_CS, LOW);	// Set active-low CS low to start the SPI cycle
  	for (SPICount = 0; SPICount < 8; SPICount++){  // Prepare to clock out the Address and Data
    		if (SPIData & 0x80)
      			digitalWrite(SPI_MOSI, HIGH);
    		else
      			digitalWrite(SPI_MOSI, LOW);
    		digitalWrite(SPI_CK, HIGH);
    		digitalWrite(SPI_CK, LOW);
    		SPIData <<= 1;
  	}   // and loop back to send the next bit
  	digitalWrite(SPI_MOSI, LOW);		// Reset the MOSI data line
  	SPIData = 0;
  	for (SPICount = 0; SPICount < 8; SPICount++){ // Prepare to clock in the data to be read
    		SPIData <<=1;	// Rotate the data
    		digitalWrite(SPI_CK, HIGH);	// Raise the clock to clock the data out of the MAX7456
    		SPIData += digitalRead(SPI_MISO); // Read the data bit
    		digitalWrite(SPI_CK, LOW);	// Drop the clock ready for the next bit
  	}
  	digitalWrite(SPI_CS, HIGH);		// Raise CS
  	printf("%d", SPIData);
	//return ((unsigned char)SPIData); // Finally return the read data

}
