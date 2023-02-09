/*
 * Copyright (c) , Majenko Technologies
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *
 *  3. Neither the name of Majenko Technologies nor the names of its contributors may be used
 *     to endorse or promote products derived from this software without
 *     specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef _MCP23S17_H
#define _MCP23S17_H

#if (ARDUINO >= 100)
# include <Arduino.h>
#else
# include <WProgram.h>
#endif

#ifdef __PIC32MX__
#include <DSPI.h>
#else
#include <SPI.h>
#endif

class MCP23S17 {
    public:
#ifdef __PIC32MX__
        /*! SPI object created from the chipKIT DSPI library. */
        typedef DSPI _SPIClass;
#else
        /*! SPI object created from the Arduino compatible SPI library. */
        typedef SPIClass _SPIClass;
#endif

        // MCP23XXX IOCON settings - initialized as 0 on POR
        static const uint8_t IOCON_BANK   = 7;  // Split registers to two banks
        static const uint8_t IOCON_MIRROR = 6;  // Mirror INT on both pins
        static const uint8_t IOCON_SEQOP  = 5;  // Disable R/W sequential addressing
        static const uint8_t IOCON_DISSLW = 4;  // Disable SDA slew rate control (I2C only)
        static const uint8_t IOCON_HAEN   = 3;  // Use addressing from hardware address pins (vs. addr 0)
        static const uint8_t IOCON_ODR    = 2;  // INT output is open-drain
        static const uint8_t IOCON_INTPOL = 1;  // INT output pin active-high (vs. low), ignored if open-drain
        static const uint8_t IOCON_INTCC  = 0;  // Interrupt cleared on reading INTCAP (not GPIO) register (MCP23XX8 only?)

        // default IOCON settings for MCP23S17
        static const uint8_t DEFAULT_IOCON = (1 << IOCON_DISSLW) | (1 << IOCON_HAEN);

    private:
        _SPIClass *_spi; /*! This points to a valid SPI object */
        uint8_t _cs;    /*! Chip select pin */
        uint8_t _chip_addr;  /*! 3-bit chip address */

        enum {
            MCP_IODIRA,     MCP_IODIRB,
            MCP_IPOLA,      MCP_IPOLB,
            MCP_GPINTENA,   MCP_GPINTENB,
            MCP_DEFVALA,    MCP_DEFVALB,
            MCP_INTCONA,    MCP_INTCONB,
            MCP_IOCONA,     MCP_IOCONB,
            MCP_GPPUA,      MCP_GPPUB,
            MCP_INTFA,      MCP_INTFB,
            MCP_INTCAPA,    MCP_INTCAPB,
            MCP_GPIOA,      MCP_GPIOB,
            MCP_OLATA,      MCP_OLATB,
            MCP_REG_COUNT
        };

        uint8_t _reg[MCP_REG_COUNT];   /*! Local mirrors of the 22 internal registers of the MCP23S17 chip */

        void spi_begin();
        void spi_end();

        void readRegister(uint8_t addr, uint8_t size = 1);
        /* Get cached value of register (1 byte) */
        uint8_t getRegister8(uint8_t addr) { return _reg[addr]; }
        /* Read register and return value (1 byte) */
        uint8_t readRegister8(uint8_t addr) { readRegister(addr, 1); return getRegister8(addr); }
        /* Get cached value of register (2 bytes, little-endian) */
        uint16_t getRegister16(uint8_t addr) { return (((uint16_t) _reg[addr + 1]) << 8) | _reg[addr]; }
        /* Read register and return value (2 bytes, little-endian) */
        uint16_t readRegister16(uint8_t addr) { readRegister(addr, 2); return getRegister16(addr); }
        void writeRegister(uint8_t addr, uint8_t size = 1);
        void writeRegister8(uint8_t addr, uint8_t val) { _reg[addr] = val; writeRegister(addr, 1); }
        void writeRegister16(uint8_t addr, uint16_t val) {
            _reg[addr] = val & 0xff; _reg[addr + 1] = val >> 8; writeRegister(addr, 2);
        }
        void readAll();
        void writeAll();
        void write_iocon_default();

        SPISettings spiSettings = SPISettings((uint32_t)8000000, MSBFIRST, SPI_MODE0);

    public:
        MCP23S17(_SPIClass *spi, uint8_t cs, uint8_t addr);
        MCP23S17(_SPIClass &spi, uint8_t cs, uint8_t addr): MCP23S17(&spi, cs, addr) {};

        void begin_tree();
        void begin_light(bool preserve_vals);
        void pinMode(uint8_t pin, uint8_t mode);
        void digitalWrite(uint8_t pin, uint8_t value);
        uint8_t digitalRead(uint8_t pin);

        uint8_t readPort(uint8_t port);
        uint16_t readPort();
        void writePort(uint8_t port, uint8_t val);
        void writePort(uint16_t val);
        void enableInterrupt(uint8_t pin, uint8_t type);
        void disableInterrupt(uint8_t pin);
        void setMirror(bool m);
        uint16_t getInterruptPins();
        uint16_t getInterruptValue();
        void setInterruptLevel(uint8_t level);
        void setInterruptOD(bool openDrain);
        uint8_t getInterruptAPins();
        uint8_t getInterruptAValue();
        uint8_t getInterruptBPins();
        uint8_t getInterruptBValue();

        void setDir(uint8_t pin, uint8_t mode);
        uint16_t getDirsInput() { return getRegister16(MCP_IODIRA); }
        void enablePullup(uint8_t pin, bool enable);
        bool getEnabledPullup(uint8_t pin) { return !!bitRead(getEnabledPullups(), pin); }
        uint16_t getEnabledPullups() { return getRegister16(MCP_GPPUA); }
};
#endif
