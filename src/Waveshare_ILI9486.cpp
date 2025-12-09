// The MIT License
//
// Copyright 2019-2020 M Hotchin
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>

#include "Waveshare_ILI9486.h"

namespace Waveshare_ILI9486_Config
{
#ifdef ARDUINO_ESP8266_WEMOS_D1R1
	//GPIO config
	//LCD

	//  D1 R1 pins are not 'general purpose". D5, D6 and D7 are actually *duplicates* of
	//  D11, D12 and D13, so you can't use them at all.  D9 is hard to use, it's part of
	//  the reboot process.  D1 is Serial TX, you probably want that for debugging.
	//  So, you can't plug the shield into a D1 R1, you need to map the pins.
	//
	int LCD_CS = D10; //  LCD Chip Select
	int LCD_BL = D8;  //  LCD Backlight
	int LCD_RST = D4;  //  LCD Reset
	int LCD_DC = D3;  //  LCD Data/Control

	int TP_CS = D0;
	int TP_IRQ = -1;   // -1=unused (just not init)
	int TP_BUSY = -1; // -1=unused (just not init)

	int SD_CS = D2;
#elif defined ARDUINO_ESP32_DEV

	//  TO-DO - is this specific enough?  Pins below are for Wemos D1 R32, but that
	//  doesn't seem to have its own board.

	//  SERIOUSLY?  No analogWrite on this platform?  Idiots.
	void analogWrite(uint8_t pin, uint8_t val)
	{
		//  Shortcut - it's either on or off
		digitalWrite(pin, val ? HIGH : LOW);
	}

	int LCD_CS = 5;  // 10; //  LCD Chip Select
	int LCD_BL = 13; // 9;  //  LCD Backlight
	int LCD_RST = 12; // 8;  //  LCD Reset
	int LCD_DC = 14; // 7;  //  LCD Data/Control

	int TP_CS = 17;   // 4;
	int TP_IRQ = 25;  // 3
	int TP_BUSY = 27; // 6

	int SD_CS = 16;   // 5;

#elif defined(ARDUINO_ARCH_RP2040)

	//GPIO config
	// LCD
	int LCD_CS = 5;   //  LCD Chip Select
	int LCD_BL = -1;  //  LCD Backlight -1=unused
	int LCD_RST = 10; //  LCD Reset
	int LCD_DC = 11;  //  LCD Data/Control

	// Touch panel
	int TP_CS = 7;    // -1=unused
	int TP_IRQ = 8;   // -1=unused (just not init)
	int TP_BUSY = 12; // -1=unused (just not init)

	int SD_CS = 13;   // -1=unused (just not init)

#else

	//GPIO config
	//LCD
	int LCD_CS = 10; //  LCD Chip Select
	int LCD_BL = 9;  //  LCD Backlight
	int LCD_RST = 8;  //  LCD Reset
	int LCD_DC = 7;  //  LCD Data/Control

	int TP_CS = 4;
	int TP_IRQ = 3;
	int TP_BUSY = 6;

	int SD_CS = 5;

#endif

	// SPI Channel to use (SPI or SPI1)
	decltype(SPI)*  SPI_PORT_P = &SPI; // By default spi0

	#if defined(ARDUINO_ARCH_RP2040)
	decltype(spi0) getPicoSPI(decltype(SPI)& s) 
	{
		return (((&(s)) == (&(SPI))) ? spi0 : spi1); 
	}
	#endif
}

#define SPI (*(SPI_PORT_P)) // to keep changes minimum

namespace
{
	using namespace Waveshare_ILI9486_Config;
	
	#if !defined(digitalWriteFast) // For platforms that has digitalWriteFast like pi pico
	#define digitalWriteFast digitalWrite
	#define digitalWriteFastDefinedLocally
	#endif


	//  Dimensions in default rotation.
	constexpr int16_t LCD_WIDTH = 320;
	constexpr int16_t LCD_HEIGHT = 480;

	//  Data sheets says min clock width is 66ns, for a max clock of 15 MHz.  Except, this
	//  thing isn't *actually* SPI!  It's a 16 bit shift register connected to the parallel
	//  interface, and that can run at 20 Mhz
#if defined(ARDUINO_ARCH_RP2040)
	SPISettings _tftSpiSettingsWrite(40000000, MSBFIRST, SPI_MODE0); // (from testing this one runs at a max of 40MHz(!))
#else
	SPISettings _tftSpiSettingsWrite(20000000, MSBFIRST, SPI_MODE0);
#endif

	//  TFT reads are slower, 150 ns period.
	//  Nevermind, Waveshare shield doesn't support reads at all!
	//SPISettings _tftSpiSettingsRead(6500000, MSBFIRST, SPI_MODE0);

	//  Touch screen wants 400 ns period.
	SPISettings tsSpiSettings(2500000, MSBFIRST, SPI_MODE0);


	inline void lcdWriteReg(uint8_t reg)
	{

		digitalWriteFast(LCD_DC, LOW);

#ifdef ARDUINO_ESP32_DEV
		SPI.write16(reg);
#else
		SPI.transfer(0);
		SPI.transfer(reg);
#endif
	}

	inline void lcdWriteData(uint8_t data)
	{
		digitalWriteFast(LCD_DC, HIGH);

#ifdef ARDUINO_ESP32_DEV
		SPI.write16(data);
#elif defined(ARDUINO_ARCH_RP2040)
		uint8_t buf[] = { 0, data };
		spi_write_blocking(getPicoSPI(SPI), buf, sizeof(buf));
#else
		SPI.transfer(0);
		SPI.transfer(data);
#endif
	}

	inline void lcdWriteDataContinue(uint8_t data)
	{
#ifdef ARDUINO_ESP32_DEV
		SPI.write16(data);
#elif defined(ARDUINO_ARCH_RP2040)
		uint8_t buf[] = { 0, data };
		spi_write_blocking(getPicoSPI(SPI), buf, sizeof(buf));
#else
		SPI.transfer(0);
		SPI.transfer(data);
#endif
	}


	inline void lcdWriteData16(uint16_t data)
	{
		digitalWriteFast(LCD_DC, HIGH);
#if defined(ARDUINO_ARCH_RP2040)
		spi_write16_blocking(getPicoSPI(SPI), &data, 1);
#else
		lcdWriteData(uint8_t(data >> 8));
		lcdWriteDataContinue(uint8_t(data & 0xff));
#endif
	}

	inline void lcdWriteDataContinue16(uint16_t data)
	{
#if defined(ARDUINO_ARCH_RP2040)
		spi_write16_blocking(getPicoSPI(SPI), &data, 1);
#else
		lcdWriteDataContinue(uint8_t(data >> 8));
		lcdWriteDataContinue(uint8_t(data & 0xff));
#endif
	}

#ifdef ARDUINO_ARCH_AVR
	//  Version of SPI.transfer(...) that *doesn't* read data back into the buffer.
	//  

	//  Wrote this, then went with a bulk repeat implementation instead.
	//inline static void transferOut(void *buf, size_t count)
	//{
	//	if (count == 0) return;
	//	uint8_t *p = (uint8_t *)buf;
	//	SPDR = *p++;
	//	while (--count > 0)
	//	{
	//		uint8_t out = *p++;
	//		while (!(SPSR & _BV(SPIF)));
	//		SPDR = out;
	//	}
	//	while (!(SPSR & _BV(SPIF)));
	//}


	inline static void transfer16Repeat(uint16_t data, unsigned long count)
	{
		if (count == 0)
		{
			return;
		}

		//  Fast way to split a 16bit into into 2 8 bit ints.
		union
		{
			uint16_t val;
			struct
			{
				uint8_t lsb; uint8_t msb;
			};
		} in;
		in.val = data;

		//  Slightly faster using these, I think the optimizer can take advantage that
		//  there is no aliasing?
		const uint8_t msb = in.msb;
		const uint8_t lsb = in.lsb;

		//  Loop phase shifting AND Duff's device?  Inconceivable!
		SPDR = msb;
		count--;
		while (!(SPSR & _BV(SPIF)));
		SPDR = lsb;
		switch (count & 0x01)
		{
		case 0:
			while (count)
			{
				count--;
				while (!(SPSR & _BV(SPIF)));
				SPDR = msb;
				asm volatile("nop");
				while (!(SPSR & _BV(SPIF)));
				SPDR = lsb;
		case 1:
			count--;
			while (!(SPSR & _BV(SPIF)));
			SPDR = msb;
			asm volatile("nop");
			while (!(SPSR & _BV(SPIF)));
			SPDR = lsb;
			}
		}
		// MUST wait for final shift out to complete!  Otherwise subsequent commands
		// happen quickly enough to stomp on it.
		asm volatile("nop");
		while (!(SPSR & _BV(SPIF)));

	}
#endif

	inline void lcdWriteDataRepeat(uint16_t data, unsigned long count)
	{
		lcdWriteReg(0x2C);
		digitalWrite(LCD_DC, HIGH);

#ifdef ARDUINO_ARCH_ESP32
		//
		//  ESP8266 seems to have better bulk transfer APIs for SPI.  
		uint8_t pattern[2];
		pattern[0] = data >> 8;
		pattern[1] = data & 0xff;

		SPI.writePattern(pattern, 2, count);

#elif defined ARDUINO_ARCH_AVR
		//  On AVR it's pretty easy to write a better bulk repeat directly, so use that.
		transfer16Repeat(data, count);
#elif defined(ARDUINO_ARCH_RP2040)
		if (count <= 0) { return; }
		constexpr long  bufMaxSize = 64; // 16 seems min, 32 good, 64 best
		static uint16_t buf[bufMaxSize]; // make static to keep stack
		uint16_t        bufFill = data;

		if (_tftSpiSettingsWrite.getBitOrder() == MSBFIRST)
		{
			bufFill = (data >> 8) | (data << 8); // swap uint16_t bytes
		}
		else
		{
			bufFill = data;
		}

		if (count <= bufMaxSize)
		{
			for (int i = 0; i < count; i++) { buf[i] = bufFill; }
			SPI.transfer(&buf, nullptr, count * sizeof(buf[0]));
		}
		else
		{
			for (int i = 0; i < bufMaxSize; i++) { buf[i] = bufFill; }
			while(count >= bufMaxSize)
			{
				SPI.transfer(&buf, nullptr, sizeof(buf));
				count -= bufMaxSize;
			}
			if (count > 0)
			{
				SPI.transfer(&buf, nullptr, count * sizeof(buf[0]));
			}
		}
#else
		//  Otherwise, just do it the boring way.
		for (unsigned long i = 0; i < count; i++)
		{
			SPI.transfer16(data);
		}
#endif
	}

	inline void lcdWriteDataCount(uint16_t *pData, unsigned long count)
	{
		lcdWriteReg(0x2C);
		digitalWrite(LCD_DC, HIGH);

#ifdef ARDUINO_ESP32_DEV
		SPI.writePixels((const uint8_t *)pData, count * 2);
#elif defined(ARDUINO_ARCH_RP2040)
		spi_write16_blocking(getPicoSPI(SPI), pData, count);
#else
		while (count--)
		{
			SPI.transfer16(*pData++);
		}
#endif
	}

	inline void lcdWriteCommand(uint8_t reg, uint8_t data)
	{
		lcdWriteReg(reg);
		lcdWriteData(data);
	}

	inline void lcdWriteCommand(uint8_t reg, uint8_t data, uint8_t data2)
	{
		lcdWriteReg(reg);
		lcdWriteData(data);
		lcdWriteDataContinue(data2);
	}

	struct ActiveBounds
	{
		uint8_t data[8];
	};

	inline void lcdWriteActiveRect(uint16_t xUL, uint16_t yUL, uint16_t xSize, uint16_t ySize)
	{
		uint16_t xStart = xUL, xEnd = xUL + xSize - 1;
		uint16_t yStart = yUL, yEnd = yUL + ySize - 1;

		//  Writing datablocks is quite a bit faster than transfering out one byte at a
		//  time.
		ActiveBounds b = {0, (uint8_t)(xStart >> 8), 0, (uint8_t)(xStart & 0xFF), 0, (uint8_t)(xEnd >> 8), 0, (uint8_t)(xEnd & 0xFF)};
		lcdWriteReg(0x2a);
		digitalWriteFast(LCD_DC, HIGH);
#if defined(ARDUINO_ARCH_RP2040)
		spi_write_blocking(getPicoSPI(SPI), (byte *)&b, sizeof(b));
#else
		SPI.writeBytes((byte *)&b, sizeof(b));
#endif

		b = {0, (uint8_t)(yStart >> 8), 0, (uint8_t)(yStart & 0xFF), 0, (uint8_t)(yEnd >> 8), 0, (uint8_t)(yEnd & 0xFF)};
		lcdWriteReg(0x2b);
		digitalWriteFast(LCD_DC, HIGH);
#if defined(ARDUINO_ARCH_RP2040)
		spi_write_blocking(getPicoSPI(SPI), (byte *)&b, sizeof(b));
#else
		SPI.writeBytes((byte *)&b, sizeof(b));
#endif
	}
}


namespace Waveshare_ILI9486_Impl
{
	void initializePins()
	{
		//  Set input pins in a sane state, so SPI can be initialized
		pinMode(LCD_CS, OUTPUT);
		digitalWrite(LCD_CS, HIGH);
		pinMode(LCD_RST, OUTPUT);
		digitalWrite(LCD_RST, HIGH);
		pinMode(LCD_DC, OUTPUT);

		if (LCD_BL >= 0)
		{
			pinMode(LCD_BL, OUTPUT);
			analogWrite(LCD_BL, 0);
		}

		if (TP_CS >= 0)
		{
			pinMode(TP_CS, OUTPUT);
			digitalWrite(TP_CS, HIGH);

			if (TP_IRQ >= 0)
			{
				pinMode(TP_IRQ, INPUT_PULLUP);
			}

			if (TP_BUSY >= 0)
			{
				pinMode(TP_BUSY, INPUT_PULLUP);
			}
		}

		if (SD_CS >= 0)
		{
			pinMode(SD_CS, OUTPUT);
			digitalWrite(SD_CS, HIGH);
		}
	}

	void startWrite()
	{
		SPI.beginTransaction(_tftSpiSettingsWrite);
		digitalWriteFast(LCD_CS, LOW);
	}

	void initializeLcd()
	{
#if defined(ARDUINO_ARCH_RP2040)
		spi_set_format(getPicoSPI(SPI), 16, (spi_cpol_t)0, (spi_cpha_t)0, SPI_MSB_FIRST);
#endif
		//  Trigger hardware reset.
		digitalWrite(LCD_RST, HIGH);
		delay(5);
		digitalWrite(LCD_RST, LOW);
		delayMicroseconds(20);
		digitalWrite(LCD_RST, HIGH);

		//  TO-DO - how long after a reset until the screen can be used?  Doesn't seem to be
		//  specified in the datasheet.
		//  Experimentally, any less than this and the initial screen clear is incomplete.
		delay(65);

		startWrite();
		{
			//  Power control settings
			lcdWriteCommand(0xC0, 0x19, 0x1a);
			lcdWriteCommand(0xC1, 0x45, 0x00);
			lcdWriteCommand(0xC2, 0x33);        //  Power/Reset on default

			lcdWriteCommand(0xC5, 0x00, 0x28);  //  VCOM control

			lcdWriteCommand(0xB1, 0xA0, 0x11);  //  Frame rate control

			lcdWriteCommand(0xB4, 0x02);        //  Display Z Inversion

			lcdWriteReg(0xB6);                  //  Display Control Function      
			lcdWriteData(0x00);
			lcdWriteDataContinue(0x42);
			lcdWriteDataContinue(0x3B);

			lcdWriteReg(0xE0);                  //  Positive Gamma control
			lcdWriteData(0x1F);
			lcdWriteDataContinue(0x25);
			lcdWriteDataContinue(0x22);
			lcdWriteDataContinue(0x0B);
			lcdWriteDataContinue(0x06);
			lcdWriteDataContinue(0x0A);
			lcdWriteDataContinue(0x4E);
			lcdWriteDataContinue(0xC6);
			lcdWriteDataContinue(0x39);
			lcdWriteDataContinue(0x00);
			lcdWriteDataContinue(0x00);
			lcdWriteDataContinue(0x00);
			lcdWriteDataContinue(0x00);
			lcdWriteDataContinue(0x00);
			lcdWriteDataContinue(0x00);

			lcdWriteReg(0XE1);                  //  Negative Gamma control
			lcdWriteData(0x1F);
			lcdWriteDataContinue(0x3F);
			lcdWriteDataContinue(0x3F);
			lcdWriteDataContinue(0x0F);
			lcdWriteDataContinue(0x1F);
			lcdWriteDataContinue(0x0F);
			lcdWriteDataContinue(0x46);
			lcdWriteDataContinue(0x49);
			lcdWriteDataContinue(0x31);
			lcdWriteDataContinue(0x05);
			lcdWriteDataContinue(0x09);
			lcdWriteDataContinue(0x03);
			lcdWriteDataContinue(0x1C);
			lcdWriteDataContinue(0x1A);
			lcdWriteDataContinue(0x00);

			//  From original driver, but register numbers don't make any sense.
			if (0)
			{
				lcdWriteReg(0XF1);
				lcdWriteData(0x36);
				lcdWriteDataContinue(0x04);
				lcdWriteDataContinue(0x00);
				lcdWriteDataContinue(0x3C);
				lcdWriteDataContinue(0x0F);
				lcdWriteDataContinue(0x0F);
				lcdWriteDataContinue(0xA4);
				lcdWriteDataContinue(0x02);

				lcdWriteReg(0XF2);
				lcdWriteData(0x18);
				lcdWriteDataContinue(0xA3);
				lcdWriteDataContinue(0x12);
				lcdWriteDataContinue(0x02);
				lcdWriteDataContinue(0x32);
				lcdWriteDataContinue(0x12);
				lcdWriteDataContinue(0xFF);
				lcdWriteDataContinue(0x32);
				lcdWriteDataContinue(0x00);

				lcdWriteReg(0XF4);
				lcdWriteData(0x40);
				lcdWriteDataContinue(0x00);
				lcdWriteDataContinue(0x08);
				lcdWriteDataContinue(0x91);
				lcdWriteDataContinue(0x04);

				lcdWriteReg(0XF8);
				lcdWriteData(0x21);
				lcdWriteDataContinue(0x04);
			}

			lcdWriteCommand(0x3A, 0x55);

			//  Set initial rotation to match AFX defaults - tall / narrow
			lcdWriteCommand(0xB6, 0x00, 0x22);
			lcdWriteCommand(0x36, 0x08);

			lcdWriteReg(0x11); // Sleep out

			//  Fill screen to black
			writeFillRect2(0, 0, LCD_WIDTH, LCD_HEIGHT, 0x0000);

			lcdWriteReg(0x29);  // Turn on display
		}
		endWrite();
	}

	//  Version with NO bounds checking!
	void writeFillRect2(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
	{
		lcdWriteActiveRect(x, y, w, h);
		lcdWriteDataRepeat(color, (int32_t)w * (int32_t)h);
	}


	void writeColors(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t *pColors)
	{
		lcdWriteActiveRect(x, y, w, h);

		lcdWriteDataCount(pColors, (unsigned long)w * (unsigned long)h);
	}

	void endWrite()
	{
		digitalWriteFast(LCD_CS, HIGH);
		SPI.endTransaction();
	}

	void setRotation(uint8_t r)
	{
		uint8_t MemoryAccessControl_0x36 = 0;

		switch (r & 0x03)
		{
		case 0x00:
			MemoryAccessControl_0x36 = 0x08;
			break;

		case 0x01:
			MemoryAccessControl_0x36 = 0x68;
			break;

		case 0x02:
			MemoryAccessControl_0x36 = 0xC8;
			break;

		case 0x03:
			MemoryAccessControl_0x36 = 0xA8;
			break;
		}

		startWrite();
		{
			lcdWriteCommand(0x36, MemoryAccessControl_0x36);
		}
		endWrite();
	}

	void invertDisplay(boolean i)
	{
		startWrite();
		{
			lcdWriteReg(i ? 0x21 : 0x20);
		}
		endWrite();
	}

	void setIdleMode(bool idle)
	{
		startWrite();
		{
			lcdWriteReg(idle ? 0x39 : 0x38);
		}
		endWrite();
	}

	void setScreenBrightness(uint8_t brightness)
	{
		if (LCD_BL >= 0)
		{
			analogWrite(LCD_BL, brightness);
		}
	}

	unsigned int GetSdCardCS()
	{
		return SD_CS;
	}

	#if !defined(digitalWriteFastDefinedLocally)
	#undef  digitalWriteFast
	#endif
}

//  Touchscreen interface

// increase or decrease the touchscreen oversampling. This is a little different than you make think:
// 1 is no oversampling, whatever data we get is immediately returned
// 2 is double-sampling and we only return valid data if both points are the same
// 3+ uses insert sort to get the median value.
// We found 2 is precise yet not too slow so we suggest sticking with it!

#define NUMSAMPLES 2


//  TSPoint code taken from Adafruit 'Touchscreen' library, MIT licence.
TSPoint::TSPoint(void)
{
	x = y = 0;
}

TSPoint::TSPoint(int16_t x0, int16_t y0, int16_t z0)
{
	x = x0;
	y = y0;
	z = z0;
}

bool TSPoint::operator==(TSPoint p1)
{
	return  ((p1.x == x) && (p1.y == y) && (p1.z == z));
}

bool TSPoint::operator!=(TSPoint p1)
{
	return  ((p1.x != x) || (p1.y != y) || (p1.z != z));
}

#if (NUMSAMPLES > 2)
static void insert_sort(int array[], uint8_t size)
{
	uint8_t j;
	int save;

	for (int i = 1; i < size; i++)
	{
		save = array[i];
		for (j = i; j >= 1 && save < array[j - 1]; j--)
			array[j] = array[j - 1];
		array[j] = save;
	}
}
#endif


//  Not implemented in Adafruit libraries?
//bool
//Waveshare_ILI9486::isTouching()
//{
//	return false;
//}

namespace
{
	uint16_t readChannel(uint8_t channel)
	{
		if (TP_CS < 0) { return 0; }

		uint16_t data = 0;
		
		SPI.beginTransaction(tsSpiSettings);
		digitalWrite(TP_CS, LOW);

		SPI.transfer(channel);

		//  Delay 8 serial clocks (3200 nS)
		delayMicroseconds(3);

		data = SPI.transfer(0x00);
		data <<= 8;
		data |= SPI.transfer(0x00);
		data >>= 3;

		digitalWrite(TP_CS, HIGH);
		SPI.endTransaction();

		return data;
	}
}

//  Approx resistance of the touchplate across the X-axis
constexpr uint16_t _rxplate = 300;

uint16_t
WaveshareTouchScreen::pressure()
{
	uint32_t z1 = readChannel(0b10110000);
	uint32_t z2 = readChannel(0b11000000);

	float rtouch;
	rtouch = z2;
	rtouch /= z1;
	//rtouch -= 1;
	rtouch *= ((uint16_t)(readTouchX()));
	rtouch *= _rxplate;
	rtouch /= 1024;

	return (rtouch);
}


int16_t
WaveshareTouchScreen::readTouchY()
{
	return readChannel(0b10010000);
}

int16_t
WaveshareTouchScreen::readTouchX()
{
	return readChannel(0b11010000);
}

constexpr int slop = 7;

// Returns un-normalized data, oriented the same as the rotation 0 setting.
TSPoint
WaveshareTouchScreen::getPoint()
{
	int x, y, z;
	int samples[NUMSAMPLES];
	uint8_t i, valid;

	valid = 1;

	//  Discard first reading, other code indicates it's noisy.
	readTouchX();

	for (i = 0; i < NUMSAMPLES; i++)
	{
		samples[i] = readTouchX();
	}

#if NUMSAMPLES > 2
	insert_sort(samples, NUMSAMPLES);
#endif
#if NUMSAMPLES == 2
	// Allow small amount of measurement noise, because capacitive
	// coupling to a TFT display's signals can induce some noise.
	if (samples[0] - samples[1] < -slop || samples[0] - samples[1] > slop)
	{
		valid = 0;
	}
	else
	{
		samples[1] = (samples[0] + samples[1]) >> 1; // average 2 samples
	}
#endif

	// Match 10 bit resolution of Adafruit touchplates
	x = (1023 - (samples[NUMSAMPLES / 2] >> 2));
	if (x == 1023) x = 0;


	for (i = 0; i < NUMSAMPLES; i++)
	{
		samples[i] = readTouchY();
	}

#if NUMSAMPLES > 2
	insert_sort(samples, NUMSAMPLES);
#endif
#if NUMSAMPLES == 2
	// Allow small amount of measurement noise, because capacitive
	// coupling to a TFT display's signals can induce some noise.
	if (samples[0] - samples[1] < -slop || samples[0] - samples[1] > slop)
	{
		valid = 0;
	}
	else
	{
		samples[1] = (samples[0] + samples[1]) >> 1; // average 2 samples
	}
#endif

	y = (1023 - (samples[NUMSAMPLES / 2] >> 2));

	if (!valid)
	{
		return TSPoint(0, 0, 0);
	}
	else if ((x == 0) && (y == 0))
	{
		return TSPoint(0, 0, 0);
	}
	else
	{
		z = (1023 - (pressure() >> 2));
	}

	return TSPoint(x, y, z);
}


namespace
{
	//  Some starting values that seem be just inside the actual limits.
	TSConfigData tscd = {100, 900, 75, 900};
}

const TSConfigData &
WaveshareTouchScreen::getTsConfigData()
{
	return tscd;
}

void
WaveshareTouchScreen::setTsConfigData(const TSConfigData &newData)
{
	tscd = newData;
}

void
WaveshareTouchScreen::resetTsConfigData()
{
	tscd = {100, 900, 75, 900};
}


namespace
{
	void swap(int16_t &x, int16_t &y)
	{
		int16_t t = x;
		x = y;
		y = t;
	}
}
//  Normalize the touchscreen readings to the dimensions of the screen.  Automatically
//  adjusts the limits over time.  To calibrate, just run the stylus off each of the four
//  edges of the screen.
bool
WaveshareTouchScreen::normalizeTsPoint(
	TSPoint &p,
	uint8_t rotation)
{
	bool fReturn = false;

	if (p.x > 0)
	{
		if (p.x < tscd.xMin)
		{
			fReturn = true;
			tscd.xMin = p.x;
		}
		if (p.x > tscd.xMax)
		{
			fReturn = true;
			tscd.xMax = p.x;
		}
	}
	if (p.y > 0)
	{
		if (p.y < tscd.yMin)
		{
			fReturn = true;
			tscd.yMin = p.y;
		}
		if (p.y > tscd.yMax)
		{
			fReturn = true;
			tscd.yMax = p.y;
		}
	}
	p.x = map(p.x, tscd.xMin, tscd.xMax, 0, LCD_WIDTH - 1);

	//  Touchscreen area extends past bottom of screen.
	p.y = map(p.y, tscd.yMin, tscd.yMax, 0, LCD_HEIGHT + 10);
	if (p.y >= LCD_HEIGHT) p.y = LCD_HEIGHT - 1;

	switch (rotation)
	{
	case 0:
		break;

	case 1:
		swap(p.x, p.y);
		p.y = LCD_WIDTH - 1 - p.y;
		break;

	case 2:
		p.x = LCD_WIDTH - 1 - p.x;
		p.y = LCD_HEIGHT - 1 - p.y;
		break;

	case 3:
		swap(p.x, p.y);
		p.x = LCD_HEIGHT - 1 - p.x;
		break;

	}

	return fReturn;
}

#if defined(ARDUINO_ARCH_RP2040)
#undef SPI
#endif
