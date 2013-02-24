#ifndef __CC2500_H
#define __CC2500_H

#include <Arduino.h>


class CC2500
{
	public:
		CC2500(uint8_t irqpin);
		void initialize();
		
		void write(const byte * data, uint8_t length);
		//void write(const String * str);
		void write(byte data);
		
		signed int read(byte * buf, uint8_t size);
		void flushread();
	
	private:
		uint8_t _irqpin;
		
		byte _sendstrobe(byte strobe);
		void _writereg(byte addr, byte value);
		byte _readreg(byte addr);

};


#endif

