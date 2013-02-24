
#include <Arduino.h>
#include <SPI.h>
#include <string>

#include "cc2500.h"
#include "cc2500_regs.h"
#include "cc2500_vals.h"


#define CC2500_IDLE    0x36      // Exit RX / TX, turn
#define CC2500_TX      0x35      // Enable TX. If in RX state, only enable TX if CCA passes
#define CC2500_RX      0x34      // Enable RX. Perform calibration if enabled
#define CC2500_FTX     0x3B      // Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states
#define CC2500_FRX     0x3A      // Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states
#define CC2500_TXFIFO  0x3F
#define CC2500_RXFIFO  0xBF
#define SRES           0x30


CC2500::CC2500(uint8_t irqpin) : _irqpin(irqpin)
{
	
}

void CC2500::initialize()
{
	pinMode(_irqpin, INPUT); 
	pinMode(SS, OUTPUT);
	
	SPI.begin();
	digitalWrite(SS, HIGH);

	//delay(10);
	
	_sendstrobe(SRES);
	
	_writereg(REG_IOCFG2,VAL_IOCFG2);
	_writereg(REG_IOCFG1,VAL_IOCFG1);
	_writereg(REG_IOCFG0,VAL_IOCFG0);

	_writereg(REG_FIFOTHR,VAL_FIFOTHR);
	_writereg(REG_SYNC1,VAL_SYNC1);
	_writereg(REG_SYNC0,VAL_SYNC0);
	_writereg(REG_PKTLEN,VAL_PKTLEN);
	_writereg(REG_PKTCTRL1,VAL_PKTCTRL1);
	_writereg(REG_PKTCTRL0,VAL_PKTCTRL0);
	_writereg(REG_ADDR,VAL_ADDR);
	_writereg(REG_CHANNR,VAL_CHANNR);
	_writereg(REG_FSCTRL1,VAL_FSCTRL1);
	_writereg(REG_FSCTRL0,VAL_FSCTRL0);
	_writereg(REG_FREQ2,VAL_FREQ2);
	_writereg(REG_FREQ1,VAL_FREQ1);
	_writereg(REG_FREQ0,VAL_FREQ0);
	_writereg(REG_MDMCFG4,VAL_MDMCFG4);
	_writereg(REG_MDMCFG3,VAL_MDMCFG3);
	_writereg(REG_MDMCFG2,VAL_MDMCFG2);
	_writereg(REG_MDMCFG1,VAL_MDMCFG1);
	_writereg(REG_MDMCFG0,VAL_MDMCFG0);
	_writereg(REG_DEVIATN,VAL_DEVIATN);
	_writereg(REG_MCSM2,VAL_MCSM2);
	_writereg(REG_MCSM1,VAL_MCSM1);
	_writereg(REG_MCSM0,VAL_MCSM0);
	_writereg(REG_FOCCFG,VAL_FOCCFG);

	_writereg(REG_BSCFG,VAL_BSCFG);
	_writereg(REG_AGCCTRL2,VAL_AGCCTRL2);
	_writereg(REG_AGCCTRL1,VAL_AGCCTRL1);
	_writereg(REG_AGCCTRL0,VAL_AGCCTRL0);
	_writereg(REG_WOREVT1,VAL_WOREVT1);
	_writereg(REG_WOREVT0,VAL_WOREVT0);
	_writereg(REG_WORCTRL,VAL_WORCTRL);
	_writereg(REG_FREND1,VAL_FREND1);
	_writereg(REG_FREND0,VAL_FREND0);
	_writereg(REG_FSCAL3,VAL_FSCAL3);
	_writereg(REG_FSCAL2,VAL_FSCAL2);
	_writereg(REG_FSCAL1,VAL_FSCAL1);
	_writereg(REG_FSCAL0,VAL_FSCAL0);
	_writereg(REG_RCCTRL1,VAL_RCCTRL1);
	_writereg(REG_RCCTRL0,VAL_RCCTRL0);
	_writereg(REG_FSTEST,VAL_FSTEST);
	_writereg(REG_PTEST,VAL_PTEST);
	_writereg(REG_AGCTEST,VAL_AGCTEST);
	_writereg(REG_TEST2,VAL_TEST2);
	_writereg(REG_TEST1,VAL_TEST1);
	_writereg(REG_TEST0,VAL_TEST0);
}

void CC2500::write(const byte * data, uint8_t length)
{
	// Make sure that the radio is in IDLE state before flushing the FIFO
	_sendstrobe(CC2500_IDLE);
	
	// Flush TX FIFO
	_sendstrobe(CC2500_FTX);

	// Put back in idle TODO - needed?
	// SIDLE: exit RX/TX
	_sendstrobe(CC2500_IDLE);
	
	//first byte is data length for variable packet length mode
	_writereg(CC2500_TXFIFO, length);

	//load up the rest of the bytes
	for(uint8_t i = 0; i < length; i++)
	{	  
		_writereg(CC2500_TXFIFO, data[i]);
	}
	
	// STX: enable TX
	_sendstrobe(CC2500_TX);

	// Wait for IRQ pin to be set -> sync transmitted
	while (!digitalRead(_irqpin));

	// Wait for IRQ pin to be cleared -> end of packet
	while (digitalRead(_irqpin));
}

#if 0
void CC2500::write(const String * str)
{
	byte data[str->length()];
	write(data, str->length());
}
#endif

void CC2500::write(byte data)
{
	write(&data, 1);
}

signed int CC2500::read(byte * buf, uint8_t size)
{
	uint8_t bytes = 0;
	
	// TODO - make this a _readreg??
	_sendstrobe(CC2500_RX);
	if(_sendstrobe(00) & 0x4) //mode is RX mode
	{
		unsigned int count = 0;
		for (; count < 250 && !digitalRead(_irqpin); count++)
		{
			delay(1);
		}
		
		if (count == 250)
		{
			flushread();
			return -1;
		}
		
		//Serial.println("IN");
		
		//while (digitalRead(_irqpin));
		
		//Serial.println("OUT");
		
		byte state = 0;
		do
		{
			digitalWrite(SS, LOW);
			while (digitalRead(MISO) == HIGH);

			state = SPI.transfer(CC2500_RXFIFO);
			delay(1);

			buf[bytes++] = SPI.transfer(0);
			digitalWrite(SS, HIGH);
			
			if((state >> 4) > 1)
			{
				flushread();
				
				// Negative code means bad
				return -(state >> 4);
			}
		} while ((state & 0xF) && bytes < size);
		
		return bytes;
	}
	
	// Receiver not enabled
	return -2;
}

void CC2500::flushread()
{
	// Make sure that the radio is in IDLE state before flushing the FIFO
	// (Unless RXOFF_MODE has been changed, the radio should be in IDLE state at this point)
	delay(10); 
	_sendstrobe(CC2500_IDLE);
	
	delay(10);
	
	// Flush RX FIFO
	_sendstrobe(CC2500_FRX); 
	delay(10); 
}

byte CC2500::_sendstrobe(byte strobe)
{
	
	digitalWrite(SS, LOW);
	while (digitalRead(MISO) == HIGH);

	byte result =  SPI.transfer(strobe);
	digitalWrite(SS, HIGH);
	
	//delay(1);
	return result;
}

void CC2500::_writereg(byte addr, byte value)
{
	// Set slave-select low
	digitalWrite(SS, LOW);
	while (digitalRead(MISO) == HIGH);

	SPI.transfer(addr);
	//delay(1);
	SPI.transfer(value);
	
	digitalWrite(SS, HIGH);
}

byte CC2500::_readreg(byte addr)
{
	digitalWrite(SS, LOW);
	while (digitalRead(MISO) == HIGH);
	
	// Addresses start at 0x80
	SPI.transfer(addr | 0x80);
	//delay(1);
	byte value = SPI.transfer(0);
	
	digitalWrite(SS, HIGH);
	return value;
}

